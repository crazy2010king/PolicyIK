"""rm_executor 事件模型（五级：INFO / RECOVERABLE / NEEDS_PARAM / NEEDS_HUMAN / FATAL）。

事件是执行器对上层的**唯一异常表达**：每个事件都带
``type`` / ``severity`` / ``t`` / ``reason`` / ``evidence`` / ``next_action``，
其中 ``reason`` 说清「发生了什么」、``evidence`` 给客观证据、``next_action``
给**可粘贴**的下一步。

严重度 → 报告 status → 进程退出码的口径与 ``rm_hw_bridge.errors`` 一致
（0 / 3 / 4）：

=================  ======  ==========  ==================================
severity          status  退出码      含义
=================  ======  ==========  ==================================
``INFO``          ok      0           正常信息
``RECOVERABLE``   blocked 3           可恢复异常（降速/续跑即可）
``NEEDS_PARAM``   blocked 3           缺参数/配置，需人工补齐
``NEEDS_HUMAN``   blocked 3           需人工介入（安全兜底）
``FATAL``         error   4           硬红线，立即停机
=================  ======  ==========  ==================================

**最严重事件决定报告 status**（:meth:`EventLog.status`）。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

__all__ = [
    "Event",
    "EventLog",
    "SEVERITIES",
    "SEVERITY_EXIT_CODES",
    "SEVERITY_RANK",
    "SEVERITY_STATUS",
    "exit_code_for_severity",
    "severity_rank",
    "sort_events",
    "status_for_severity",
    "worst_severity",
]

#: 五个严重度（由轻到重）
SEVERITIES: Tuple[str, ...] = (
    "INFO",
    "RECOVERABLE",
    "NEEDS_PARAM",
    "NEEDS_HUMAN",
    "FATAL",
)

#: 严重度排序权重（越大越严重）
SEVERITY_RANK: Dict[str, int] = {
    "INFO": 0,
    "RECOVERABLE": 1,
    "NEEDS_PARAM": 2,
    "NEEDS_HUMAN": 3,
    "FATAL": 4,
}

#: 严重度 → 报告 status（与 rm_hw_bridge.errors 的 status 口径一致）
SEVERITY_STATUS: Dict[str, str] = {
    "INFO": "ok",
    "RECOVERABLE": "blocked",
    "NEEDS_PARAM": "blocked",
    "NEEDS_HUMAN": "blocked",
    "FATAL": "error",
}

#: 严重度 → 进程退出码（ok=0 / blocked=3 / error=4）
SEVERITY_EXIT_CODES: Dict[str, int] = {
    "INFO": 0,
    "RECOVERABLE": 3,
    "NEEDS_PARAM": 3,
    "NEEDS_HUMAN": 3,
    "FATAL": 4,
}


def severity_rank(severity: str) -> int:
    """严重度 → 排序权重；未知严重度抛 ``ValueError``（不静默当成 INFO）。"""
    if severity not in SEVERITY_RANK:
        raise ValueError("未知 severity: %r（只允许 %s）" % (severity, list(SEVERITIES)))
    return SEVERITY_RANK[severity]


def status_for_severity(severity: str) -> str:
    """严重度 → 报告 status。"""
    severity_rank(severity)  # 校验
    return SEVERITY_STATUS[severity]


def exit_code_for_severity(severity: str) -> int:
    """严重度 → 进程退出码（0 / 3 / 4）。"""
    severity_rank(severity)  # 校验
    return SEVERITY_EXIT_CODES[severity]


@dataclass
class Event:
    """一条结构化事件。

    字段
    ----
    type
        事件类型（如 ``follow_error_warn`` / ``feedback_stream_lost``）；
    severity
        五级之一（见 :data:`SEVERITIES`）；
    t
        事件发生时刻（秒，float）；
    reason
        发生了什么（必须写明接口名 / 数值 / 阈值）；
    evidence
        客观证据列表（路径、实测值、阈值……）；
    next_action
        **可粘贴**的下一步。
    """

    type: str
    severity: str
    t: float = 0.0
    reason: str = ""
    evidence: List[str] = field(default_factory=list)
    next_action: str = ""

    def __post_init__(self) -> None:
        if self.severity not in SEVERITY_RANK:
            raise ValueError(
                "未知 severity: %r（只允许 %s）" % (self.severity, list(SEVERITIES))
            )
        self.evidence = [e if isinstance(e, str) else str(e) for e in (self.evidence or [])]

    def to_dict(self) -> Dict[str, Any]:
        """转成 JSON 友好字典（字段顺序固定，便于报告 diff）。"""
        return {
            "type": self.type,
            "severity": self.severity,
            "t": self.t,
            "reason": self.reason,
            "evidence": list(self.evidence),
            "next_action": self.next_action,
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "Event":
        """从字典还原事件。"""
        return cls(
            type=data.get("type", ""),
            severity=data.get("severity", "INFO"),
            t=float(data.get("t", 0.0) or 0.0),
            reason=data.get("reason", ""),
            evidence=list(data.get("evidence") or []),
            next_action=data.get("next_action", ""),
        )


def sort_events(events: Iterable[Event]) -> List[Event]:
    """按严重度排序（同级保持插入顺序，稳定排序）。"""
    return sorted(list(events), key=lambda e: SEVERITY_RANK[e.severity])


def worst_severity(events: Iterable[Event]) -> Optional[str]:
    """最严重的 severity；空集合返回 ``None``（**不返回 INFO 冒充**）。"""
    worst = None
    for event in events:
        if worst is None or SEVERITY_RANK[event.severity] > SEVERITY_RANK[worst]:
            worst = event.severity
    return worst


class EventLog:
    """事件日志（追加 + 按严重度过滤 + 导出）。"""

    def __init__(self, events: Optional[Sequence[Event]] = None) -> None:
        self._events: List[Event] = list(events or [])

    # ------------------------------------------------------------- 追加
    def append(self, event: Event) -> Event:
        """追加一条事件（返回该事件，便于链式使用）。"""
        if not isinstance(event, Event):
            raise TypeError("EventLog 只接受 Event 实例（当前 %r）" % type(event).__name__)
        self._events.append(event)
        return event

    def add(self, type: str, severity: str, t: float = 0.0, reason: str = "",
            evidence: Optional[Iterable[Any]] = None, next_action: str = "") -> Event:
        """追加一条事件（关键字形式）。"""
        return self.append(
            Event(type=type, severity=severity, t=t, reason=reason,
                  evidence=list(evidence or []), next_action=next_action)
        )

    def extend(self, events: Iterable[Event]) -> None:
        """批量追加。"""
        for event in events:
            self.append(event)

    # ------------------------------------------------------------- 查询
    def __len__(self) -> int:
        return len(self._events)

    def __iter__(self):
        return iter(self._events)

    @property
    def events(self) -> List[Event]:
        """事件列表副本。"""
        return list(self._events)

    def filter(self, severities: Iterable[str]) -> List[Event]:
        """按严重度集合过滤（顺序保持）。"""
        wanted = set(severities)
        return [e for e in self._events if e.severity in wanted]

    def at_least(self, severity: str) -> List[Event]:
        """过滤出严重度 ≥ 给定级别的事件。"""
        threshold = severity_rank(severity)
        return [e for e in self._events if SEVERITY_RANK[e.severity] >= threshold]

    def by_type(self, type: str) -> List[Event]:
        """按事件类型过滤。"""
        return [e for e in self._events if e.type == type]

    def worst_severity(self) -> Optional[str]:
        """最严重事件级别（空 → ``None``）。"""
        return worst_severity(self._events)

    def status(self) -> str:
        """按最严重事件推导报告 status（空日志 → ``ok``）。"""
        worst = self.worst_severity()
        return SEVERITY_STATUS[worst] if worst else "ok"

    def exit_code(self) -> int:
        """按最严重事件推导退出码（0 / 3 / 4）。"""
        worst = self.worst_severity()
        return SEVERITY_EXIT_CODES[worst] if worst else 0

    def to_list(self) -> List[Dict[str, Any]]:
        """导出为 JSON 友好字典列表（**按严重度排序**，同级保持插入顺序）。"""
        return [e.to_dict() for e in sort_events(self._events)]
