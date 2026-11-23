"""工艺 trace 的记录与回放（JSONL 一行一事件 + 只读回放）。

三条硬边界（由 ``tests/test_process_trace.py`` 守卫）
------------------------------------------------------

1. **零运动、零 SDK、零网络**：本模块**只**用标准库（``json`` / ``os`` /
   ``time``），**不** import ``rm_executor`` / ``rm_hw_bridge`` / 厂商 SDK，
   **不**打开任何 socket。回放是**纯读**动作：只把磁盘上的 JSONL 读回来
   解释，**绝不**下发任何运动指令。
2. **补齐只针对元数据**：写入时缺的 ``ts`` / ``severity`` / ``status`` 等
   **元字段**会被补齐，但 ``reason`` / ``next_action`` / ``evidence``
   **绝不**被改写（**不伪造判定依据、不伪造下一步**）。
3. **零严重度排序表**：本模块**不**定义严重度等级集合，**不**写任何严重度
   排序字典。回放只做**计数**与「文件里最后一条非空 next_action」的抽取，
   排序/裁决的真值仍在 ``rm_executor.events``（执行层）。

事件 schema（``schema_version = 1``）
-------------------------------------

一行一事件（JSONL，UTF-8，追加写）::

    {"schema_version": 1, "ts": 1710000000.123, "iteration": 3,
     "phase": "plan|execute|gripper|judge|recover|loop", "segment": "grasp",
     "status": "ok|blocked|error",
     "severity": "INFO|RECOVERABLE|NEEDS_PARAM|NEEDS_HUMAN|FATAL",
     "reason": "<判定依据>", "next_action": "<可粘贴下一步>",
     "evidence": [...], "report": {...}}

``report`` 是**可选**的运动段报告摘要；其余字段缺省时由
:class:`TraceWriter` 补齐（**只补元数据**）。

坏行处理
--------

``read_trace`` **不抛裸异常**：非 JSON 行 / 非字典行被跳过，并在解析统计
``stats["skipped"]`` 与 ``stats["warnings"]`` 里留下证据；现场看到的
「跳过数」本身就是「trace 被截断/被第三方写坏」的信号。
"""
from __future__ import annotations

import json
import os
import time
from typing import Any, Dict, Iterable, List, Optional

__all__ = [
    "SCHEMA_VERSION",
    "TraceRead",
    "TraceWriter",
    "format_event",
    "read_trace",
    "replay_trace",
]


#: trace 事件 schema 版本（**冻结**：字段口径变化必须升版本）。
SCHEMA_VERSION = 1

#: 元字段缺省值（**只用于补齐缺失的元数据**）。
_DEFAULT_TS = None          # 缺 ts → 写入时刻
_DEFAULT_SEVERITY = "INFO"
_DEFAULT_STATUS = "ok"
_DEFAULT_PHASE = "loop"
_DEFAULT_ITERATION = 0


# --------------------------------------------------------------------------
# 归一化
# --------------------------------------------------------------------------

def _normalize(event: Any) -> Dict[str, Any]:
    """归一化一条事件（**只补元数据**）。

    * ``schema_version`` 恒为 :data:`SCHEMA_VERSION`；
    * 缺 ``ts`` → 当前时间；缺 ``severity`` → ``INFO``；缺 ``status`` → ``ok``；
    * 缺 ``iteration`` → 0；缺 ``phase`` → ``loop``；缺 ``segment`` → ``""``；
    * ``reason`` / ``next_action`` / ``evidence`` **绝不**被改写：缺失就留空，
      有值就**原样保留**（不截断、不润色、不补默认文案）。

    非字典入参 → 退化为一条最小事件（``reason`` 记原始形态），**不抛异常**。
    """
    if not isinstance(event, dict):
        return {
            "schema_version": SCHEMA_VERSION,
            "ts": time.time(),
            "iteration": _DEFAULT_ITERATION,
            "phase": _DEFAULT_PHASE,
            "segment": "",
            "status": _DEFAULT_STATUS,
            "severity": _DEFAULT_SEVERITY,
            "reason": "非字典事件: %r" % (event,),
            "next_action": "",
            "evidence": [],
        }

    out: Dict[str, Any] = {}
    out["schema_version"] = SCHEMA_VERSION
    out["ts"] = event["ts"] if event.get("ts") is not None else time.time()
    out["iteration"] = (event["iteration"]
                        if event.get("iteration") is not None
                        else _DEFAULT_ITERATION)
    out["phase"] = event["phase"] if event.get("phase") else _DEFAULT_PHASE
    out["segment"] = event["segment"] if event.get("segment") else ""
    out["status"] = event["status"] if event.get("status") else _DEFAULT_STATUS
    out["severity"] = (event["severity"]
                       if event.get("severity") else _DEFAULT_SEVERITY)
    # -- 以下三项**原样保留**（缺失即留空，绝不改写）--------------------
    out["reason"] = event.get("reason") if event.get("reason") is not None else ""
    out["next_action"] = (event.get("next_action")
                          if event.get("next_action") is not None else "")
    out["evidence"] = event.get("evidence") if event.get("evidence") is not None else []
    if event.get("report") is not None:
        out["report"] = event["report"]
    return out


# --------------------------------------------------------------------------
# 写
# --------------------------------------------------------------------------

class TraceWriter:
    """JSONL trace 写入器（**追加写**，一行一事件）。

    :param path: trace 文件路径（父目录不存在 → 自动创建）。
    """

    def __init__(self, path: str) -> None:
        self._path = str(path)
        parent = os.path.dirname(os.path.abspath(self._path))
        if parent and not os.path.isdir(parent):
            os.makedirs(parent, exist_ok=True)
        self._fh = open(self._path, "a", encoding="utf-8")
        self._closed = False
        self._count = 0
        #: 写入期错误（磁盘满 / 只读 / 已关闭）——**不抛裸异常**。
        self.errors: List[str] = []

    # -- 属性 --------------------------------------------------------------
    @property
    def path(self) -> str:
        """trace 文件路径。"""
        return self._path

    @property
    def count(self) -> int:
        """已写入的事件条数。"""
        return self._count

    @property
    def closed(self) -> bool:
        """是否已关闭。"""
        return self._closed

    # -- 写 ----------------------------------------------------------------
    def record(self, event: Dict[str, Any]) -> Dict[str, Any]:
        """追加一条事件（**立即 flush**），返回归一化后的事件。

        写入失败（磁盘满 / 只读）**不抛裸异常**：记入 ``self.errors`` 并
        返回归一化事件（**工艺主流程不被 trace 拖垮**）。
        """
        normalized = _normalize(event)
        if self._closed:
            self.errors.append("trace 已关闭，事件被丢弃")
            return normalized
        try:
            self._fh.write(json.dumps(normalized, ensure_ascii=False) + "\n")
            self._fh.flush()
            self._count += 1
        except (OSError, TypeError, ValueError) as exc:
            self.errors.append("trace 写入失败: %r" % (exc,))
        return normalized

    def close(self) -> None:
        """关闭文件（幂等）。"""
        if self._closed:
            return
        self._closed = True
        try:
            self._fh.close()
        except OSError:
            pass

    # -- 上下文管理器 ------------------------------------------------------
    def __enter__(self) -> "TraceWriter":
        return self

    def __exit__(self, exc_type, exc, tb) -> bool:
        self.close()
        return False


# --------------------------------------------------------------------------
# 读
# --------------------------------------------------------------------------

class TraceRead(list):
    """``list[dict]`` 子类：读回的事件 + 解析统计（``.stats``）。"""

    def __init__(self, events: Optional[Iterable[Dict[str, Any]]] = None,
                 stats: Optional[Dict[str, Any]] = None) -> None:
        super().__init__(events or [])
        self.stats: Dict[str, Any] = dict(stats or {})


def read_trace(path: str) -> TraceRead:
    """逐行读回 trace（**坏行跳过，不抛裸异常**）。

    :return: :class:`TraceRead`（``list[dict]`` 子类，带 ``.stats``）：
        ``lines``（非空行总数）/ ``parsed``（成功解析条数）/
        ``skipped``（跳过条数）/ ``warnings``（逐条证据）/
        ``missing``（文件不存在时为 ``True``）。
    """
    stats: Dict[str, Any] = {
        "path": str(path),
        "lines": 0,
        "parsed": 0,
        "skipped": 0,
        "warnings": [],
        "missing": False,
    }
    events: List[Dict[str, Any]] = []
    try:
        with open(str(path), "r", encoding="utf-8") as fh:
            raw_lines = fh.readlines()
    except FileNotFoundError:
        stats["missing"] = True
        stats["warnings"].append("trace 文件不存在: %s" % path)
        return TraceRead(events, stats)
    except (OSError, UnicodeDecodeError) as exc:
        stats["warnings"].append("trace 文件读取失败: %r" % (exc,))
        return TraceRead(events, stats)

    for number, line in enumerate(raw_lines, start=1):
        text = line.strip()
        if not text:
            continue
        stats["lines"] += 1
        try:
            item = json.loads(text)
        except (ValueError, TypeError) as exc:
            stats["skipped"] += 1
            stats["warnings"].append("第 %d 行不是合法 JSON（已跳过）: %s"
                                     % (number, exc))
            continue
        if not isinstance(item, dict):
            stats["skipped"] += 1
            stats["warnings"].append("第 %d 行不是 JSON 对象（已跳过）" % number)
            continue
        stats["parsed"] += 1
        events.append(item)
    return TraceRead(events, stats)


# --------------------------------------------------------------------------
# 人读格式
# --------------------------------------------------------------------------

def format_event(event: Dict[str, Any]) -> str:
    """把一条事件格式成**人读一行**。

    形如：``2026-01-02 03:04:05 | 圈次 3 | 段 grasp | judge | NEEDS_PARAM |
    blocked | 段 grasp 判定=needs_param | 下一步: 补齐夹爪标定四项后重跑``
    """
    if not isinstance(event, dict):
        return "事件非法: %r" % (event,)
    ts = event.get("ts")
    try:
        stamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(float(ts)))
    except (TypeError, ValueError):
        stamp = str(ts) if ts is not None else "未知时间"
    iteration = event.get("iteration")
    segment = event.get("segment") or "-"
    phase = event.get("phase") or "-"
    severity = event.get("severity") or _DEFAULT_SEVERITY
    status = event.get("status") or _DEFAULT_STATUS
    reason = event.get("reason") or ""
    next_action = event.get("next_action") or "（无下一步）"
    return "%s | 圈次 %s | 段 %s | %s | %s | %s | %s | 下一步: %s" % (
        stamp, iteration, segment, phase, severity, status, reason, next_action)


# --------------------------------------------------------------------------
# 回放（纯读）
# --------------------------------------------------------------------------

def _is_failure(event: Dict[str, Any]) -> bool:
    """是否算「失败事件」。

    口径：**状态非 ok** 或 **严重度非 INFO** 即失败。**不**做严重度排序
    （排序真值在 ``rm_executor.events``，本模块不复制）。
    """
    status = str(event.get("status") or _DEFAULT_STATUS)
    severity = str(event.get("severity") or _DEFAULT_SEVERITY)
    return status != "ok" or severity != _DEFAULT_SEVERITY


def replay_trace(path: str) -> Dict[str, Any]:
    """只读回放一份 trace（**零运动、零 SDK、零网络**）。

    :return: ``{"status", "reason", "events", "lines", "summary"}``：
        * ``events`` —— 解析出的事件列表（坏行已跳过）；
        * ``lines`` —— 逐事件的人读行（:func:`format_event`）；
        * ``summary`` —— ``events`` 总数 / 各 severity 计数 / 失败事件数 /
          **最后一条非空 next_action**（现场直接可粘贴）。
    """
    read = read_trace(path)
    events = list(read)
    stats = dict(read.stats)

    severity_counts: Dict[str, int] = {}
    status_counts: Dict[str, int] = {}
    failures = 0
    last_next_action = ""
    last_failure_reason = ""
    for event in events:
        severity = str(event.get("severity") or _DEFAULT_SEVERITY)
        severity_counts[severity] = severity_counts.get(severity, 0) + 1
        status = str(event.get("status") or _DEFAULT_STATUS)
        status_counts[status] = status_counts.get(status, 0) + 1
        if _is_failure(event):
            failures += 1
            last_failure_reason = str(event.get("reason") or "")
        action = event.get("next_action")
        if isinstance(action, str) and action.strip():
            last_next_action = action

    summary: Dict[str, Any] = {
        "events": len(events),
        "lines_read": int(stats.get("lines", 0)),
        "parsed": int(stats.get("parsed", 0)),
        "skipped": int(stats.get("skipped", 0)),
        "severity_counts": severity_counts,
        "status_counts": status_counts,
        "failures": failures,
        "last_next_action": last_next_action,
        "warnings": list(stats.get("warnings") or []),
    }

    if failures:
        status = "blocked"
        reason = (last_failure_reason
                  or "trace 中有 %d 条失败事件" % failures)
    else:
        status = "ok"
        reason = "trace 回放完成（%d 条事件，无失败）" % len(events)

    return {
        "status": status,
        "reason": reason,
        "events": events,
        "lines": [format_event(event) for event in events],
        "summary": summary,
    }
