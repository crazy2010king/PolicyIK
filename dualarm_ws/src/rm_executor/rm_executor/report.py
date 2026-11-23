"""rm_executor 统一报告（**离线 fake 与真机 live 同一份 schema、同一份代码**）。

报告是执行器对上层（S04 安全链路 / S05 入口 / 操作者）的**唯一输出面**。
本模块只做**装配与校验**，不含任何执行逻辑，因此：

* ``fake`` 与 ``live`` 走的是**同一个** :func:`build_report`；
* 真机窗口只需回填数字（``mode="live"``），**报告结构不变**；
* 字段顺序固定 → 报告可 diff、可逐版本对比。

顶层结构（7 个键）::

    {
      "status": "ok|blocked|error",
      "reason": "", "next_action": "", "evidence": [],
      "mode": "fake|live",
      "metrics": {...},          # 15 个键，见 METRIC_KEYS
      "events": [...]            # 每项 6 个键，见 EVENT_KEYS
    }

``metrics`` 的 15 个键（口径全部由 :mod:`rm_executor.metrics` 冻结）::

    ticks / duration_s / rt_factor
    tick_jitter_p50_ms / tick_jitter_p95_ms / tick_jitter_p99_ms / tick_jitter_max_ms
    follow_err_rms_deg / follow_err_max_deg
    arrival_err_deg / arrival_hold_ms
    sched_fifo / pre_position_hold_ms / hold_ticks / planned_duration_s

**空指标用 ``None``，绝不返回 0 冒充**（0 会被误读成「抖动为零」「误差为零」）。

**零 rmctl 依赖、零 tesseract 依赖、零 SDK 调用、零时钟**。
"""
from __future__ import annotations

import json
import os
from typing import Any, Dict, Iterable, List, Optional, Sequence

from rm_executor import contract, events as events_mod

__all__ = [
    "EVENT_KEYS",
    "METRIC_KEYS",
    "TOP_LEVEL_KEYS",
    "build_report",
    "dump_report",
    "empty_metrics",
    "status_from_events",
    "validate_report",
]

#: 报告顶层键（顺序固定）
TOP_LEVEL_KEYS = (
    "status",
    "reason",
    "next_action",
    "evidence",
    "mode",
    "metrics",
    "events",
)

#: 报告 metrics 键（**恰好 15 个**，顺序固定）
METRIC_KEYS = (
    "ticks",
    "duration_s",
    "rt_factor",
    "tick_jitter_p50_ms",
    "tick_jitter_p95_ms",
    "tick_jitter_p99_ms",
    "tick_jitter_max_ms",
    "follow_err_rms_deg",
    "follow_err_max_deg",
    "arrival_err_deg",
    "arrival_hold_ms",
    "sched_fifo",
    "pre_position_hold_ms",
    "hold_ticks",
    "planned_duration_s",
)

#: 单条事件的键（顺序固定）
EVENT_KEYS = (
    "type",
    "severity",
    "t",
    "reason",
    "evidence",
    "next_action",
)

#: 合法 mode
MODES = ("fake", "live")


def empty_metrics() -> Dict[str, Any]:
    """全空指标（**15 个键齐全**，未知值一律 ``None``，不返回 0 冒充）。"""
    out: Dict[str, Any] = {key: None for key in METRIC_KEYS}
    out["ticks"] = 0
    out["hold_ticks"] = 0
    out["sched_fifo"] = False
    return out


def status_from_events(event_dicts: Sequence[Dict[str, Any]]) -> str:
    """由事件列表推导报告 status（**最严重事件决定**）。

    * ``FATAL`` → ``error``（退出码 4）；
    * ``RECOVERABLE`` / ``NEEDS_PARAM`` / ``NEEDS_HUMAN`` → ``blocked``（退出码 3）；
    * 无事件或只有 ``INFO`` → ``ok``（退出码 0）。
    """
    worst = None
    for event in event_dicts or []:
        severity = event.get("severity", "INFO")
        if (worst is None
                or events_mod.SEVERITY_RANK[severity] > events_mod.SEVERITY_RANK[worst]):
            worst = severity
    return events_mod.status_for_severity(worst) if worst else contract.STATUS_OK


def build_report(status: Optional[str] = None,
                 reason: str = "",
                 next_action: str = "",
                 evidence: Optional[Iterable[Any]] = None,
                 mode: str = "fake",
                 metrics: Optional[Dict[str, Any]] = None,
                 events: Optional[Iterable[Dict[str, Any]]] = None) -> Dict[str, Any]:
    """装配统一报告（**唯一构造点**：fake / live 共用）。

    参数
    ----
    status
        ``ok`` / ``blocked`` / ``error``；``None`` → 由事件严重度推导
        （:func:`status_from_events`）。
    metrics
        指标字典；缺键 → ``None``（不返回 0 冒充）；**未知键直接报错**
        （防止两处真相悄悄长歪）。
    events
        事件字典列表（通常是 ``EventLog.to_list()`` 的结果）。

    返回
    ----
    dict
        顶层键**恰好** :data:`TOP_LEVEL_KEYS`；``metrics`` 键**恰好**
        :data:`METRIC_KEYS`。
    """
    event_list: List[Dict[str, Any]] = []
    for item in events or []:
        if not isinstance(item, dict):
            raise ValueError("events 必须是字典列表（当前 %r）" % type(item).__name__)
        event_list.append({key: item.get(key) for key in EVENT_KEYS})

    if status is None:
        status = status_from_events(event_list)
    if status not in (contract.STATUS_OK, contract.STATUS_BLOCKED, contract.STATUS_ERROR):
        raise ValueError("未知 status: %r（只允许 ok/blocked/error）" % (status,))

    if mode not in MODES:
        raise ValueError("未知 mode: %r（只允许 fake/live）" % (mode,))

    metric_values = empty_metrics()
    for key, value in (metrics or {}).items():
        if key not in METRIC_KEYS:
            raise ValueError(
                "未知 metrics 键: %r（只允许 %s）" % (key, ", ".join(METRIC_KEYS))
            )
        metric_values[key] = value

    evidence_list = [e if isinstance(e, str) else str(e) for e in (evidence or [])]

    if status in (contract.STATUS_BLOCKED, contract.STATUS_ERROR) and not next_action:
        next_action = contract.NEXT_ACTION_FALLBACK

    return {
        "status": status,
        "reason": reason,
        "next_action": next_action,
        "evidence": evidence_list,
        "mode": mode,
        "metrics": metric_values,
        "events": event_list,
    }


def validate_report(report: Dict[str, Any]) -> List[str]:
    """校验报告结构；返回问题列表（**空列表 = 结构合规**）。"""
    problems: List[str] = []
    if not isinstance(report, dict):
        return ["报告必须是字典（当前 %r）" % type(report).__name__]
    missing = [k for k in TOP_LEVEL_KEYS if k not in report]
    extra = [k for k in report if k not in TOP_LEVEL_KEYS]
    if missing:
        problems.append("缺顶层键: %s" % ", ".join(missing))
    if extra:
        problems.append("多顶层键: %s" % ", ".join(extra))
    metrics = report.get("metrics")
    if not isinstance(metrics, dict):
        problems.append("metrics 必须是字典")
    else:
        m_missing = [k for k in METRIC_KEYS if k not in metrics]
        m_extra = [k for k in metrics if k not in METRIC_KEYS]
        if m_missing:
            problems.append("缺 metrics 键: %s" % ", ".join(m_missing))
        if m_extra:
            problems.append("多 metrics 键: %s" % ", ".join(m_extra))
    events = report.get("events")
    if not isinstance(events, list):
        problems.append("events 必须是列表")
    else:
        for idx, item in enumerate(events):
            if not isinstance(item, dict):
                problems.append("events[%d] 必须是字典" % idx)
                continue
            for key in EVENT_KEYS:
                if key not in item:
                    problems.append("events[%d] 缺键 %s" % (idx, key))
    if report.get("status") not in (contract.STATUS_OK, contract.STATUS_BLOCKED,
                                    contract.STATUS_ERROR):
        problems.append("status 非法: %r" % (report.get("status"),))
    if report.get("mode") not in MODES:
        problems.append("mode 非法: %r" % (report.get("mode"),))
    return problems


def dump_report(report: Dict[str, Any], path: str) -> str:
    """把报告落盘（JSON，UTF-8 不转义）；返回绝对路径。

    落盘失败（目录不存在 / 权限不足）→ **抛异常**（调用方决定退出码），
    绝不静默丢报告。
    """
    target = os.path.abspath(path)
    parent = os.path.dirname(target)
    if parent and not os.path.isdir(parent):
        os.makedirs(parent, exist_ok=True)
    with open(target, "w", encoding="utf-8") as fh:
        json.dump(report, fh, ensure_ascii=False, indent=2, sort_keys=False)
        fh.write("\n")
    return target
