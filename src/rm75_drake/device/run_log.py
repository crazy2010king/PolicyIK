"""设备层全链路运行日志（S03-T07，**独立 schema**）。

为什么**另立**一份 schema
------------------------
S02 的 :mod:`rm75_drake.recording` 已经是 ``schema_version=1``，并且被
S07 的 golden-file 基线钉死。设备层的运行日志要额外携带
**状态帧摘要 / 逐轴电流 / 逐关节错误码 / 安全七闸违规 / 状态机转移 /
抖动直方图**——这些东西塞进录制文件就必须把 ``schema_version`` 升到 2，
从而击穿 S07 的基线。

⇒ 结论：**另立独立 schema**（本模块，``schema_version=1``），
**绝不修改** :mod:`rm75_drake.recording`（S02 的那一行 ``schema_version = 1``
一个字符都不动）。

格式（JSON / UTF-8）
--------------------
顶层键固定为::

    {
      "schema_version": 1,
      "created_at": "2026-01-01T00:00:00Z",
      "meta":    {"command": "exec", "mode": "dry_run", ...},
      "events": [{"seq": 0, "t_mono_ns": 123, "kind": "command",
                  "payload": {...}}, ...],
      "summary": {"status": "ok", "reason": "", "num_events": 3, ...},
      "checksum": "<sha256>"
    }

事件类型（``kind``）
--------------------
=====================  ====================================================
``kind``               含义
=====================  ====================================================
``command``            下发指令（seq / t_abs / 关节角 rad）
``state_frame``        状态帧摘要（关节角 / 电流 / 使能 / 错误码 / age）
``safety_violation``   安全七闸违规（reason / gate / action / severity）
``state_transition``   状态机（或驱动相位）转移
``metric``             指标汇总（抖动 / 延迟直方图分位数）
``note``               人工备注（跳过原因、未验证指标清单 …）
=====================  ====================================================

失败可解释
----------
* 未知顶层键 / 未知事件键 / 未知事件类型 / 未知 payload 键
  → :class:`RunLogError`，消息**点名具体的键**；
* 缺必填字段 → :class:`RunLogError`（点名字段）；
* **篡改检出**：顶层 ``checksum`` 是 ``meta + events + summary + created_at``
  的规范化 JSON 的 sha256；读回时重算并比对，不一致 → :class:`RunLogError`
  （消息说明「疑似篡改」并指出不一致的字段路径）。
"""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

__all__ = [
    "RUN_LOG_SCHEMA_VERSION",
    "TOP_LEVEL_KEYS",
    "META_KEYS",
    "SUMMARY_KEYS",
    "EVENT_KEYS",
    "EVENT_KINDS",
    "PAYLOAD_KEYS",
    "RunLogError",
    "RunLogEvent",
    "RunLog",
    "RunLogBuilder",
    "build_run_log",
    "run_log_from_dict",
    "write_run_log",
    "load_run_log",
    "verify_run_log",
    "compute_checksum",
    "metric_payload",
]


#: 运行日志 schema 版本（**独立于** 录制文件的 schema）。
RUN_LOG_SCHEMA_VERSION = 1

#: 顶层键（固定集合；出现未知键 → :class:`RunLogError`）。
TOP_LEVEL_KEYS: Tuple[str, ...] = (
    "schema_version",
    "created_at",
    "meta",
    "events",
    "summary",
    "checksum",
)

#: ``meta`` 的合法键（固定集合）。
META_KEYS: Tuple[str, ...] = (
    "command",
    "mode",
    "device_name",
    "arm_ip",
    "scenario_path",
    "recording_path",
    "robot_name",
    "dry_run",
)

#: ``summary`` 的合法键（固定集合）。
SUMMARY_KEYS: Tuple[str, ...] = (
    "status",
    "reason",
    "num_events",
    "num_samples",
    "num_violations",
    "jitter",
    "notes",
    "unverified_metrics",
    "dropped_events",
)

#: 单个事件的合法键（固定集合）。
EVENT_KEYS: Tuple[str, ...] = ("seq", "t_mono_ns", "kind", "payload")

#: 合法事件类型（固定集合）。
EVENT_KINDS: Tuple[str, ...] = (
    "command",
    "state_frame",
    "safety_violation",
    "state_transition",
    "metric",
    "note",
)

#: 每种事件类型的 payload 键（``required`` 必填，``optional`` 可选）。
PAYLOAD_KEYS: Dict[str, Dict[str, Tuple[str, ...]]] = {
    "command": {
        "required": ("seq", "t_abs", "joint_rad"),
        "optional": (),
    },
    "state_frame": {
        "required": (
            "seq",
            "t_abs",
            "age_s",
            "joint_rad",
            "currents_ma",
            "enabled",
            "joint_err_codes",
            "controller_status",
            "gripper_online",
            "complete",
        ),
        "optional": (),
    },
    "safety_violation": {
        "required": ("reason", "gate", "action", "severity", "detail"),
        "optional": ("joints",),
    },
    "state_transition": {
        "required": ("from_state", "to_state", "event", "reason"),
        "optional": (),
    },
    "metric": {
        "required": (
            "name",
            "count",
            "p50",
            "p99",
            "p999",
            "max",
            "min",
            "mean",
            "dropped",
        ),
        "optional": (),
    },
    "note": {
        "required": ("text",),
        "optional": (),
    },
}


class RunLogError(ValueError):
    """运行日志结构 / 内容非法。

    消息必须指出**具体的键**（未知键 / 缺字段 / 校验和不一致）。
    """


# ── 内部工具 ────────────────────────────────────────────────────────────
def _require_mapping(value: Any, path: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise RunLogError(f"{path} 必须是对象（mapping），实际：{type(value).__name__}")
    return value


def _check_keys(
    mapping: Mapping[str, Any],
    path: str,
    allowed: Sequence[str],
    required: Sequence[str],
) -> None:
    """未知键 / 缺必填键 → :class:`RunLogError`（点名键名）。"""
    unknown = sorted(str(key) for key in mapping if key not in allowed)
    if unknown:
        raise RunLogError(
            f"{path} 出现未知键：{unknown}；该处允许的键只有 {list(allowed)}"
        )
    missing = [key for key in required if key not in mapping]
    if missing:
        raise RunLogError(
            f"{path} 缺少必填字段：{missing}；该处允许的键只有 {list(allowed)}"
        )


def _is_json_scalar(value: Any) -> bool:
    return value is None or isinstance(value, (bool, int, float, str))


def _check_json_value(value: Any, path: str) -> Any:
    """校验「JSON 安全」的值（标量 / 标量列表 / 嵌套映射），返回原值。"""
    if _is_json_scalar(value):
        if isinstance(value, float) and not math.isfinite(value):
            raise RunLogError(f"{path} 必须是有限数值，实际：{value!r}")
        return value
    if isinstance(value, (list, tuple)):
        return [
            _check_json_value(item, f"{path}[{index}]")
            for index, item in enumerate(value)
        ]
    if isinstance(value, Mapping):
        result: Dict[str, Any] = {}
        for key, item in value.items():
            if not isinstance(key, str):
                raise RunLogError(f"{path} 的键必须是字符串，实际：{key!r}")
            result[key] = _check_json_value(item, f"{path}.{key}")
        return result
    raise RunLogError(
        f"{path} 必须是 JSON 安全值（标量 / 列表 / 映射），实际：{type(value).__name__}"
    )


# ── 事件 ────────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class RunLogEvent:
    """运行日志里的一个事件。

    字段
    ----
    seq:
        事件序号（从 0 逐条 +1）。
    t_mono_ns:
        事件时刻（**单调时钟**，纳秒）。
    kind:
        事件类型（见 :data:`EVENT_KINDS`）。
    payload:
        事件载荷（键集合由 ``kind`` 决定，见 :data:`PAYLOAD_KEYS`）。
    """

    seq: int
    t_mono_ns: int
    kind: str
    payload: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        """转成 JSON 可序列化的字典（键序固定）。"""
        return {
            "seq": int(self.seq),
            "t_mono_ns": int(self.t_mono_ns),
            "kind": str(self.kind),
            "payload": dict(self.payload),
        }

    @classmethod
    def from_dict(cls, data: Any, *, where: str = "event") -> "RunLogEvent":
        """从字典构造（结构非法 → :class:`RunLogError`，点名键名）。"""
        mapping = _require_mapping(data, where)
        _check_keys(mapping, where, EVENT_KEYS, EVENT_KEYS)
        kind = mapping["kind"]
        if not isinstance(kind, str) or kind not in EVENT_KINDS:
            raise RunLogError(
                f"{where}.kind 必须是 {list(EVENT_KINDS)} 之一，实际：{kind!r}"
            )
        schema = PAYLOAD_KEYS[kind]
        allowed = tuple(schema["required"]) + tuple(schema["optional"])
        payload = _require_mapping(mapping["payload"], f"{where}.payload")
        _check_keys(payload, f"{where}.payload", allowed, schema["required"])
        try:
            seq = int(mapping["seq"])
            t_mono_ns = int(mapping["t_mono_ns"])
        except (TypeError, ValueError) as exc:
            raise RunLogError(f"{where} 的 seq / t_mono_ns 必须是整数：{exc}") from exc
        if seq < 0:
            raise RunLogError(f"{where}.seq 必须是非负整数，实际：{seq}")
        if t_mono_ns < 0:
            raise RunLogError(f"{where}.t_mono_ns 必须是非负整数，实际：{t_mono_ns}")
        clean = {
            key: _check_json_value(value, f"{where}.payload.{key}")
            for key, value in payload.items()
        }
        return cls(seq=seq, t_mono_ns=t_mono_ns, kind=kind, payload=clean)


# ── 运行日志 ────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class RunLog:
    """一次设备层运行的完整记录。

    字段
    ----
    schema_version:
        格式版本（当前 :data:`RUN_LOG_SCHEMA_VERSION`）。
    created_at:
        创建时间（ISO-8601 UTC 字符串）。
    meta:
        元信息（键集合见 :data:`META_KEYS`）。
    events:
        事件序列。
    summary:
        汇总（键集合见 :data:`SUMMARY_KEYS`）。
    checksum:
        ``meta + events + summary + created_at`` 的规范化 JSON 的 sha256
        （**篡改检出**）。
    """

    schema_version: int = RUN_LOG_SCHEMA_VERSION
    created_at: str = ""
    meta: Mapping[str, Any] = field(default_factory=dict)
    events: Tuple[RunLogEvent, ...] = ()
    summary: Mapping[str, Any] = field(default_factory=dict)
    checksum: str = ""

    # -- 序列化 ----------------------------------------------------------
    def to_dict(self) -> Dict[str, Any]:
        """转成 JSON 可序列化的字典（顶层键顺序固定）。"""
        return {
            "schema_version": int(self.schema_version),
            "created_at": str(self.created_at),
            "meta": {key: self.meta.get(key) for key in META_KEYS},
            "events": [event.to_dict() for event in self.events],
            "summary": {key: self.summary.get(key) for key in SUMMARY_KEYS},
            "checksum": str(self.checksum),
        }

    @classmethod
    def from_dict(cls, data: Any, *, verify_checksum: bool = True) -> "RunLog":
        """从字典构造（结构非法 → :class:`RunLogError`，点名键名）。"""
        mapping = _require_mapping(data, "运行日志")
        _check_keys(mapping, "运行日志", TOP_LEVEL_KEYS, TOP_LEVEL_KEYS)
        version = mapping["schema_version"]
        if not isinstance(version, int) or isinstance(version, bool):
            raise RunLogError(f"schema_version 必须是整数，实际：{version!r}")
        if version != RUN_LOG_SCHEMA_VERSION:
            raise RunLogError(
                f"运行日志 schema_version={version}，本实现只支持 "
                f"{RUN_LOG_SCHEMA_VERSION}"
            )
        created_at = mapping["created_at"]
        if not isinstance(created_at, str):
            raise RunLogError(f"created_at 必须是字符串，实际：{created_at!r}")
        meta = _require_mapping(mapping["meta"], "运行日志.meta")
        _check_keys(meta, "运行日志.meta", META_KEYS, ())
        clean_meta = {
            key: _check_json_value(value, f"运行日志.meta.{key}")
            for key, value in meta.items()
        }
        raw_events = mapping["events"]
        if not isinstance(raw_events, list):
            raise RunLogError(
                f"运行日志.events 必须是数组，实际：{type(raw_events).__name__}"
            )
        events = tuple(
            RunLogEvent.from_dict(item, where=f"events[{index}]")
            for index, item in enumerate(raw_events)
        )
        for index in range(1, len(events)):
            if events[index].seq != events[index - 1].seq + 1:
                raise RunLogError(
                    f"events[{index}].seq 必须逐条 +1："
                    f"{events[index - 1].seq} → {events[index].seq}"
                )
        summary = _require_mapping(mapping["summary"], "运行日志.summary")
        _check_keys(summary, "运行日志.summary", SUMMARY_KEYS, ())
        clean_summary = {
            key: _check_json_value(value, f"运行日志.summary.{key}")
            for key, value in summary.items()
        }
        checksum = mapping["checksum"]
        if not isinstance(checksum, str):
            raise RunLogError(f"checksum 必须是字符串，实际：{checksum!r}")
        run_log = cls(
            schema_version=version,
            created_at=created_at,
            meta=clean_meta,
            events=events,
            summary=clean_summary,
            checksum=checksum,
        )
        if verify_checksum:
            actual = compute_checksum(run_log)
            if actual != checksum:
                raise RunLogError(
                    "运行日志校验和不匹配（疑似篡改）："
                    f"记录值 {checksum!r}，重算值 {actual!r}"
                )
        return run_log

    # -- 便捷视图 --------------------------------------------------------
    def events_of_kind(self, kind: str) -> Tuple[RunLogEvent, ...]:
        """返回某一类型的所有事件。"""
        if kind not in EVENT_KINDS:
            raise RunLogError(f"kind 必须是 {list(EVENT_KINDS)} 之一，实际：{kind!r}")
        return tuple(event for event in self.events if event.kind == kind)


# ── 构造 / 校验和 ───────────────────────────────────────────────────────
def compute_checksum(run_log: RunLog) -> str:
    """计算运行日志的校验和（sha256，覆盖除 ``checksum`` 外的全部内容）。"""
    material = {
        "schema_version": int(run_log.schema_version),
        "created_at": str(run_log.created_at),
        "meta": {key: run_log.meta.get(key) for key in META_KEYS},
        "events": [event.to_dict() for event in run_log.events],
        "summary": {key: run_log.summary.get(key) for key in SUMMARY_KEYS},
    }
    text = json.dumps(
        material, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    )
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def build_run_log(
    events: Sequence[RunLogEvent],
    *,
    meta: Optional[Mapping[str, Any]] = None,
    summary: Optional[Mapping[str, Any]] = None,
    created_at: Optional[str] = None,
    dropped_events: int = 0,
) -> RunLog:
    """构造一份运行日志（自动填 ``checksum`` 与缺失的 ``meta`` / ``summary`` 键）。

    失败模式
    --------
    * ``meta`` / ``summary`` 含未知键 → :class:`RunLogError`（点名键名）；
    * 事件序号不连续 → :class:`RunLogError`。
    """
    clean_meta = {key: None for key in META_KEYS}
    if meta:
        unknown = sorted(str(key) for key in meta if key not in META_KEYS)
        if unknown:
            raise RunLogError(
                f"运行日志.meta 出现未知键：{unknown}；允许的键只有 {list(META_KEYS)}"
            )
        clean_meta.update(meta)
    clean_summary: Dict[str, Any] = {
        "status": None,
        "reason": None,
        "num_events": len(events),
        "num_samples": None,
        "num_violations": None,
        "jitter": None,
        "notes": [],
        "unverified_metrics": [],
        "dropped_events": int(dropped_events),
    }
    if summary:
        unknown = sorted(str(key) for key in summary if key not in SUMMARY_KEYS)
        if unknown:
            raise RunLogError(
                f"运行日志.summary 出现未知键：{unknown}；允许的键只有 {list(SUMMARY_KEYS)}"
            )
        clean_summary.update(summary)
    clean_summary["num_events"] = len(events)
    for index in range(1, len(events)):
        if events[index].seq != events[index - 1].seq + 1:
            raise RunLogError(
                f"事件序号必须逐条 +1：events[{index}].seq={events[index].seq}，"
                f"上一条 seq={events[index - 1].seq}"
            )
    stamp = created_at or datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
    run_log = RunLog(
        schema_version=RUN_LOG_SCHEMA_VERSION,
        created_at=stamp,
        meta=clean_meta,
        events=tuple(events),
        summary=clean_summary,
        checksum="",
    )
    return RunLog(
        schema_version=run_log.schema_version,
        created_at=run_log.created_at,
        meta=run_log.meta,
        events=run_log.events,
        summary=run_log.summary,
        checksum=compute_checksum(run_log),
    )


def verify_run_log(run_log: RunLog) -> bool:
    """校验运行日志的校验和是否仍然匹配（``True`` = 未被篡改）。"""
    return compute_checksum(run_log) == run_log.checksum


def run_log_from_dict(data: Any, *, verify_checksum: bool = True) -> RunLog:
    """从字典构造运行日志（结构非法 → :class:`RunLogError`）。"""
    return RunLog.from_dict(data, verify_checksum=verify_checksum)


def write_run_log(run_log: RunLog, path: Any) -> Path:
    """把运行日志写到 JSON 文件（UTF-8、缩进 2、键序固定）。"""
    if not isinstance(run_log, RunLog):
        raise RunLogError(
            f"run_log 必须是 RunLog，实际：{type(run_log).__name__}"
        )
    target = Path(path)
    if target.parent and not target.parent.exists():
        target.parent.mkdir(parents=True, exist_ok=True)
    text = json.dumps(run_log.to_dict(), ensure_ascii=False, indent=2)
    target.write_text(text + "\n", encoding="utf-8")
    return target


def load_run_log(path: Any) -> RunLog:
    """读回运行日志并**严格校验**（含校验和 ⇒ 篡改可检出）。

    失败模式
    --------
    * 文件不存在 → :class:`FileNotFoundError`；
    * JSON 语法错误 → :class:`RunLogError`；
    * 结构 / 字段非法 → :class:`RunLogError`（消息指出具体的键）；
    * 校验和不匹配 → :class:`RunLogError`（疑似篡改）。
    """
    target = Path(path)
    text = target.read_text(encoding="utf-8")
    try:
        data = json.loads(text)
    except json.JSONDecodeError as exc:
        raise RunLogError(f"运行日志不是合法 JSON（{target}）：{exc}") from exc
    return RunLog.from_dict(data)


# ── 构造器 ──────────────────────────────────────────────────────────────
class RunLogBuilder:
    """运行日志的增量构造器（自动分配事件序号）。

    用途：执行环 / CLI 在**运行期**往里追加事件，最后一次性
    :meth:`build` 出来（带校验和）。
    """

    def __init__(
        self, *, start_ns: Optional[int] = None, max_events: Optional[int] = None
    ) -> None:
        """构造器。

        参数
        ----
        start_ns:
            事件时间戳的起点（不给 ⇒ 事件时间戳记 0）。
        max_events:
            **事件数上限**（``None`` ⇒ 不限）。达到上限后**停止追加并计数**
            （:attr:`dropped`），与本项目「超限计数、不静默丢弃」的约定一致
            （见 :mod:`rm75_drake.device.safety` 的违规上限）。
        """
        if max_events is not None:
            if isinstance(max_events, bool) or not isinstance(max_events, int):
                raise ValueError(f"max_events 必须是整数，实际：{max_events!r}")
            if max_events < 1:
                raise ValueError(f"max_events 必须 ≥ 1，实际：{max_events}")
        self._events: List[RunLogEvent] = []
        self._start_ns = start_ns
        self._max_events = max_events
        self._dropped = 0

    def __len__(self) -> int:
        return len(self._events)

    @property
    def dropped(self) -> int:
        """因达到事件上限而**未落盘**的事件数（不静默丢弃）。"""
        return self._dropped

    @property
    def max_events(self) -> Optional[int]:
        """事件数上限（``None`` ⇒ 不限）。"""
        return self._max_events

    @property
    def events(self) -> Tuple[RunLogEvent, ...]:
        """已追加的事件（不可变视图）。"""
        return tuple(self._events)

    def append(
        self,
        kind: str,
        payload: Mapping[str, Any],
        *,
        t_mono_ns: Optional[int] = None,
    ) -> Optional[RunLogEvent]:
        """追加一个事件（返回落库的事件对象）。

        达到 :attr:`max_events` 上限 ⇒ **不再追加**并计数，返回 ``None``
        （**不静默丢弃**：丢弃数见 :attr:`dropped`，并写入 summary）。

        失败模式
        --------
        * 未知 ``kind`` → :class:`RunLogError`（点名）；
        * payload 含未知键 / 缺必填键 → :class:`RunLogError`（点名）。
        """
        if not isinstance(kind, str) or kind not in EVENT_KINDS:
            raise RunLogError(
                f"事件 kind 必须是 {list(EVENT_KINDS)} 之一，实际：{kind!r}"
            )
        schema = PAYLOAD_KEYS[kind]
        allowed = tuple(schema["required"]) + tuple(schema["optional"])
        mapping = _require_mapping(payload, "payload")
        _check_keys(mapping, f"payload({kind})", allowed, schema["required"])
        if self._max_events is not None and len(self._events) >= self._max_events:
            self._dropped += 1
            return None
        stamp = (
            int(t_mono_ns)
            if t_mono_ns is not None
            else (self._start_ns if self._start_ns is not None else 0)
        )
        event = RunLogEvent(
            seq=len(self._events),
            t_mono_ns=stamp,
            kind=kind,
            payload={
                key: _check_json_value(value, f"payload({kind}).{key}")
                for key, value in mapping.items()
            },
        )
        self._events.append(event)
        return event

    def build(
        self,
        *,
        meta: Optional[Mapping[str, Any]] = None,
        summary: Optional[Mapping[str, Any]] = None,
        created_at: Optional[str] = None,
    ) -> RunLog:
        """构造最终运行日志（自动填校验和）。"""
        return build_run_log(
            self._events,
            meta=meta,
            summary=summary,
            created_at=created_at,
            dropped_events=self._dropped,
        )


# ── 指标载荷 ────────────────────────────────────────────────────────────
def metric_payload(name: str, histogram: Any) -> Dict[str, Any]:
    """把 :mod:`rm75_drake.device.metrics` 的直方图转成 ``metric`` 事件载荷。

    与真机脚本**同一口径**（直方图实现只有一份）——禁止另算一套分位数。
    """
    summary = histogram.summary() if hasattr(histogram, "summary") else dict(histogram)
    return {
        "name": str(name),
        "count": int(summary.get("count", 0)),
        "p50": summary.get("p50"),
        "p99": summary.get("p99"),
        "p999": summary.get("p999"),
        "max": summary.get("max"),
        "min": summary.get("min"),
        "mean": summary.get("mean"),
        "dropped": int(summary.get("dropped", 0)),
    }
