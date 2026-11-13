"""设备状态机（S03-T03）：**纯转移表 + 守卫条件**，零真机 / 零 SDK 依赖。

为什么是「纯」
--------------
S03 交接给 S04 的边界承诺里明确要求这条状态链：

``DISCONNECTED → CONNECTED → WAIT_ENABLE → READY → STREAMING → HOLDING →
FAULT → RECOVER → ESTOP``

它必须在**没有真机**（甚至没有 SDK）的环境里被**逐条验证**，因此：

* 状态与事件是**枚举**（不是字符串裸值）；
* 转移表是**纯数据**（``(状态, 事件) → (目标状态, 守卫名)``）；
* 守卫是**纯函数**（输入 = 当前帧 + 上下文，输出 = 是否允许 / 拒绝理由）；
* 非法转移**抛显式错误**并带**稳定 reason 标签**（不静默吞掉）。

守卫条件（硬要求）
------------------
* ``WAIT_ENABLE → READY``：**全部 7 轴** ``joint_en_flag == 1``；
* ``FAULT`` 的触发源：逐关节错误码非 0 / 系统错误码非 0 /
  ``arm_current_status == 9``（急停）；
* ``RECOVER → READY``：错误码清零 + 全部 7 轴已使能。

事件流
------
每次成功转移都会追加一条 :class:`Transition` 记录（源状态 / 目标状态 /
事件 / reason / 时间戳），供运行日志（:mod:`rm75_drake.device.run_log`）
消费——**状态机本身不做 I/O**。

单位纪律
--------
关节角 **rad**、电流 **mA**、时间 **s**。
本模块**不实现** rad↔deg 换算（唯一转换点在 :mod:`rm75_drake.device.sdk`）。
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from enum import Enum
from typing import Any, Callable, Dict, List, Mapping, Optional, Tuple

from ..planning.trajectory import JOINT_COUNT
from .transport import StateFrame

__all__ = [
    # 状态
    "DeviceState",
    "ALL_STATES",
    # 事件
    "DeviceEvent",
    "ALL_EVENTS",
    # 稳定 reason 标签
    "REASON_ILLEGAL_TRANSITION",
    "REASON_GUARD_REJECTED",
    "REASON_JOINT_ERROR",
    "REASON_SYSTEM_ERROR",
    "REASON_ESTOP_STATUS",
    "REASON_LINK_DOWN",
    "REASON_ALL_ENABLED",
    "REASON_CLEARED",
    "REASON_MANUAL",
    "REASON_RECOVERED",
    "REASON_STREAM_START",
    "REASON_STREAM_STOP",
    "REASON_HOLD",
    "REASON_RESUME",
    "REASON_CONNECT",
    "REASON_DISCONNECT",
    "REASON_ENABLE",
    "REASON_RECOVER_BEGIN",
    "REASON_RECOVER_FAILED",
    # 错误
    "StateMachineError",
    "IllegalTransitionError",
    "GuardRejectedError",
    # 数据结构
    "Transition",
    # 状态机
    "DeviceStateMachine",
    "detect_fault",
    "transition_table",
]


# ── 稳定 reason 标签 ────────────────────────────────────────────────────
REASON_ILLEGAL_TRANSITION = "illegal_transition"
REASON_GUARD_REJECTED = "guard_rejected"
REASON_JOINT_ERROR = "joint_error"
REASON_SYSTEM_ERROR = "system_error"
REASON_ESTOP_STATUS = "estop_status"
REASON_LINK_DOWN = "link_down"
REASON_ALL_ENABLED = "all_enabled"
REASON_CLEARED = "cleared"
REASON_MANUAL = "manual"
REASON_RECOVERED = "recovered"
REASON_STREAM_START = "stream_start"
REASON_STREAM_STOP = "stream_stop"
REASON_HOLD = "hold"
REASON_RESUME = "resume"
REASON_CONNECT = "connect"
REASON_DISCONNECT = "disconnect"
REASON_ENABLE = "enable"
REASON_RECOVER_BEGIN = "recover_begin"
REASON_RECOVER_FAILED = "recover_failed"


# ── 状态 ────────────────────────────────────────────────────────────────
class DeviceState(str, Enum):
    """设备状态（取值即写进运行日志的稳定标签）。"""

    DISCONNECTED = "disconnected"
    CONNECTED = "connected"
    WAIT_ENABLE = "wait_enable"
    READY = "ready"
    STREAMING = "streaming"
    HOLDING = "holding"
    FAULT = "fault"
    RECOVER = "recover"
    ESTOP = "estop"


#: 全部状态（顺序即正常链路顺序，ESTOP / FAULT / RECOVER 为异常支路）。
ALL_STATES: Tuple[DeviceState, ...] = (
    DeviceState.DISCONNECTED,
    DeviceState.CONNECTED,
    DeviceState.WAIT_ENABLE,
    DeviceState.READY,
    DeviceState.STREAMING,
    DeviceState.HOLDING,
    DeviceState.FAULT,
    DeviceState.RECOVER,
    DeviceState.ESTOP,
)


# ── 事件 ────────────────────────────────────────────────────────────────
class DeviceEvent(str, Enum):
    """驱动状态机的事件。"""

    CONNECT = "connect"
    DISCONNECT = "disconnect"
    ENABLE = "enable"
    ALL_ENABLED = "all_enabled"
    START_STREAM = "start_stream"
    STOP_STREAM = "stop_stream"
    HOLD = "hold"
    RESUME = "resume"
    FAULT = "fault"
    RECOVER_BEGIN = "recover_begin"
    RECOVER_END = "recover_end"
    RECOVER_FAILED = "recover_failed"
    ESTOP = "estop"
    CLEAR_ESTOP = "clear_estop"
    RESET = "reset"


#: 全部事件。
ALL_EVENTS: Tuple[DeviceEvent, ...] = tuple(DeviceEvent)


# ── 错误 ────────────────────────────────────────────────────────────────
class StateMachineError(RuntimeError):
    """状态机错误（带稳定 ``reason`` 标签）。"""

    def __init__(
        self,
        message: str,
        *,
        reason: str = REASON_ILLEGAL_TRANSITION,
        state: Optional[DeviceState] = None,
        event: Optional[DeviceEvent] = None,
    ) -> None:
        super().__init__(message)
        self.reason = reason
        self.state = state
        self.event = event


class IllegalTransitionError(StateMachineError):
    """非法转移（当前状态不接受该事件）。"""

    def __init__(
        self,
        message: str,
        *,
        state: DeviceState,
        event: DeviceEvent,
        allowed: Tuple[DeviceEvent, ...] = (),
    ) -> None:
        super().__init__(message, reason=REASON_ILLEGAL_TRANSITION, state=state, event=event)
        self.allowed = allowed


class GuardRejectedError(StateMachineError):
    """守卫条件不满足（转移在表里，但前置条件没达成）。"""

    def __init__(
        self,
        message: str,
        *,
        state: DeviceState,
        event: DeviceEvent,
        guard_reason: str,
    ) -> None:
        super().__init__(message, reason=REASON_GUARD_REJECTED, state=state, event=event)
        self.guard_reason = guard_reason


# ── 数据结构 ────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class Transition:
    """一次状态转移记录（写进运行日志）。

    字段
    ----
    source:
        源状态。
    target:
        目标状态。
    event:
        触发事件。
    reason:
        稳定 reason 标签。
    timestamp_s:
        发生时刻（**单调时钟**，秒）。
    detail:
        人类可读补充（可选）。
    """

    source: DeviceState
    target: DeviceState
    event: DeviceEvent
    reason: str
    timestamp_s: float
    detail: str = ""

    def to_dict(self) -> Dict[str, Any]:
        """返回可 JSON 序列化的字典。"""
        return {
            "source": self.source.value,
            "target": self.target.value,
            "event": self.event.value,
            "reason": self.reason,
            "timestamp_s": float(self.timestamp_s),
            "detail": self.detail,
        }


@dataclass(frozen=True)
class _Spec:
    """一条转移的规格（目标状态 + 守卫名）。"""

    target: DeviceState
    guard: Optional[str] = None


#: 故障判定用的「急停」状态码（``arm_current_status == 9``）。
ESTOP_STATUS = 9


# ── 守卫（纯函数）────────────────────────────────────────────────────────
def _guard_all_enabled(ctx: Mapping[str, Any]) -> Optional[str]:
    """守卫：**全部 7 轴**已使能。

    拒绝时返回 reason 标签（``None`` = 通过）。
    """
    frame = ctx.get("frame")
    if not isinstance(frame, StateFrame):
        return "frame_missing"
    if len(frame.joint_en_flags) != JOINT_COUNT:
        return "frame_invalid"
    if not all(bool(flag) for flag in frame.joint_en_flags):
        return "not_all_enabled"
    return None


def _guard_no_fault(ctx: Mapping[str, Any]) -> Optional[str]:
    """守卫：无故障（错误码清零 + 非急停状态）。

    拒绝时返回**具体**的故障 reason 标签。
    """
    frame = ctx.get("frame")
    if not isinstance(frame, StateFrame):
        return "frame_missing"
    fault = detect_fault(frame)
    if fault is not None:
        return fault
    if not all(bool(flag) for flag in frame.joint_en_flags):
        return "not_all_enabled"
    return None


#: 守卫名 → 守卫函数。
_GUARDS: Dict[str, Callable[[Mapping[str, Any]], Optional[str]]] = {
    "all_enabled": _guard_all_enabled,
    "no_fault": _guard_no_fault,
}


# ── 转移表（纯数据）──────────────────────────────────────────────────────
def _build_table() -> Dict[Tuple[DeviceState, DeviceEvent], _Spec]:
    table: Dict[Tuple[DeviceState, DeviceEvent], _Spec] = {}

    def add(
        source: DeviceState,
        event: DeviceEvent,
        target: DeviceState,
        guard: Optional[str] = None,
    ) -> None:
        table[(source, event)] = _Spec(target=target, guard=guard)

    # 正常链路
    add(DeviceState.DISCONNECTED, DeviceEvent.CONNECT, DeviceState.CONNECTED)
    add(DeviceState.CONNECTED, DeviceEvent.ENABLE, DeviceState.WAIT_ENABLE)
    add(DeviceState.WAIT_ENABLE, DeviceEvent.ALL_ENABLED, DeviceState.READY, "all_enabled")
    add(DeviceState.READY, DeviceEvent.START_STREAM, DeviceState.STREAMING)
    add(DeviceState.STREAMING, DeviceEvent.STOP_STREAM, DeviceState.READY)
    add(DeviceState.STREAMING, DeviceEvent.HOLD, DeviceState.HOLDING)
    add(DeviceState.HOLDING, DeviceEvent.RESUME, DeviceState.STREAMING)
    add(DeviceState.HOLDING, DeviceEvent.STOP_STREAM, DeviceState.READY)

    # 断链
    add(DeviceState.CONNECTED, DeviceEvent.DISCONNECT, DeviceState.DISCONNECTED)
    add(DeviceState.WAIT_ENABLE, DeviceEvent.DISCONNECT, DeviceState.DISCONNECTED)
    add(DeviceState.READY, DeviceEvent.DISCONNECT, DeviceState.DISCONNECTED)
    add(DeviceState.STREAMING, DeviceEvent.DISCONNECT, DeviceState.DISCONNECTED)
    add(DeviceState.HOLDING, DeviceEvent.DISCONNECT, DeviceState.DISCONNECTED)

    # 故障与恢复
    add(DeviceState.CONNECTED, DeviceEvent.FAULT, DeviceState.FAULT)
    add(DeviceState.WAIT_ENABLE, DeviceEvent.FAULT, DeviceState.FAULT)
    add(DeviceState.READY, DeviceEvent.FAULT, DeviceState.FAULT)
    add(DeviceState.STREAMING, DeviceEvent.FAULT, DeviceState.FAULT)
    add(DeviceState.HOLDING, DeviceEvent.FAULT, DeviceState.FAULT)
    add(DeviceState.FAULT, DeviceEvent.RECOVER_BEGIN, DeviceState.RECOVER)
    add(DeviceState.RECOVER, DeviceEvent.RECOVER_END, DeviceState.READY, "no_fault")
    add(DeviceState.RECOVER, DeviceEvent.RECOVER_FAILED, DeviceState.FAULT)

    # 急停
    add(DeviceState.CONNECTED, DeviceEvent.ESTOP, DeviceState.ESTOP)
    add(DeviceState.WAIT_ENABLE, DeviceEvent.ESTOP, DeviceState.ESTOP)
    add(DeviceState.READY, DeviceEvent.ESTOP, DeviceState.ESTOP)
    add(DeviceState.STREAMING, DeviceEvent.ESTOP, DeviceState.ESTOP)
    add(DeviceState.HOLDING, DeviceEvent.ESTOP, DeviceState.ESTOP)
    add(DeviceState.FAULT, DeviceEvent.ESTOP, DeviceState.ESTOP)
    add(DeviceState.RECOVER, DeviceEvent.ESTOP, DeviceState.ESTOP)
    add(DeviceState.ESTOP, DeviceEvent.CLEAR_ESTOP, DeviceState.DISCONNECTED)

    # 复位（从异常态回到链路起点）
    add(DeviceState.FAULT, DeviceEvent.RESET, DeviceState.DISCONNECTED)
    add(DeviceState.RECOVER, DeviceEvent.RESET, DeviceState.DISCONNECTED)
    add(DeviceState.ESTOP, DeviceEvent.RESET, DeviceState.DISCONNECTED)

    return table


#: 转移表（``(状态, 事件) → (目标状态, 守卫名)``）。
_TRANSITIONS: Dict[Tuple[DeviceState, DeviceEvent], _Spec] = _build_table()


def transition_table() -> Dict[Tuple[DeviceState, DeviceEvent], _Spec]:
    """返回转移表的一份**拷贝**（调用方不得就地改坏全局表）。"""
    return dict(_TRANSITIONS)


def detect_fault(frame: StateFrame) -> Optional[str]:
    """判定一帧是否携带**故障**信号。

    返回
    ----
    ``None``
        无故障；
    否则返回稳定 reason 标签（``joint_error`` / ``system_error`` /
    ``estop_status``）。

    优先级
    ------
    急停 > 系统错误 > 逐关节错误（急停是最强的信号）。

    说明
    ----
    :class:`~rm75_drake.device.transport.StateFrame` 只有**一个**错误码字段
    ``err_code``：``-3`` = 推送数据不完整（由状态源丢弃，不会到这里），
    其余非 0 值一律视为**系统错误码**。
    """
    if not isinstance(frame, StateFrame):
        raise StateMachineError(
            f"frame 必须是 StateFrame，收到 {type(frame).__name__}",
            reason=REASON_GUARD_REJECTED,
        )
    if int(frame.arm_current_status) == ESTOP_STATUS:
        return REASON_ESTOP_STATUS
    if int(frame.err_code) != 0 and int(frame.err_code) != -3:
        return REASON_SYSTEM_ERROR
    if any(int(code) != 0 for code in frame.joint_err_codes):
        return REASON_JOINT_ERROR
    return None


# ── 状态机 ──────────────────────────────────────────────────────────────
class DeviceStateMachine:
    """设备状态机（纯转移表 + 守卫）。

    参数
    ----
    clock:
        单调时钟（默认 :func:`time.monotonic`）；测试可注入确定性时钟。
    """

    def __init__(self, *, clock: Callable[[], float] = time.monotonic) -> None:
        self._state = DeviceState.DISCONNECTED
        self._clock = clock
        self._history: List[Transition] = []

    # -- 观测 ------------------------------------------------------------
    @property
    def state(self) -> DeviceState:
        """当前状态。"""
        return self._state

    @property
    def history(self) -> List[Transition]:
        """转移事件流（**拷贝**，供运行日志消费）。"""
        return list(self._history)

    def allowed_events(self) -> Tuple[DeviceEvent, ...]:
        """返回当前状态下**允许**的事件（按转移表顺序）。"""
        return tuple(
            event for (state, event) in _TRANSITIONS if state == self._state
        )

    def can(self, event: DeviceEvent) -> bool:
        """判定某事件在当前状态下是否**在转移表里**（不评估守卫）。"""
        return (self._state, event) in _TRANSITIONS

    # -- 转移 ------------------------------------------------------------
    def transition(
        self,
        event: DeviceEvent,
        *,
        frame: Optional[StateFrame] = None,
        reason: Optional[str] = None,
        detail: str = "",
        context: Optional[Mapping[str, Any]] = None,
    ) -> Transition:
        """执行一次转移。

        参数
        ----
        event:
            触发事件。
        frame:
            当前状态帧（守卫需要）。
        reason:
            稳定 reason 标签；``None`` ⇒ 用事件的默认标签。
        detail:
            人类可读补充。
        context:
            额外的守卫上下文（会与 ``frame`` 合并）。

        返回
        ----
        :class:`Transition` 记录（已追加到 :attr:`history`）。

        异常
        ----
        :class:`IllegalTransitionError`
            当前状态不接受该事件（带 ``allowed`` 列表）。
        :class:`GuardRejectedError`
            转移在表里，但守卫条件不满足（带 ``guard_reason``）。
        """
        if not isinstance(event, DeviceEvent):
            raise StateMachineError(
                f"event 必须是 DeviceEvent，收到 {type(event).__name__}",
                reason=REASON_ILLEGAL_TRANSITION,
                state=self._state,
            )
        spec = _TRANSITIONS.get((self._state, event))
        if spec is None:
            allowed = self.allowed_events()
            raise IllegalTransitionError(
                f"状态 {self._state.value} 不接受事件 {event.value}"
                f"（允许：{[e.value for e in allowed]}）",
                state=self._state,
                event=event,
                allowed=allowed,
            )

        ctx: Dict[str, Any] = dict(context or {})
        if frame is not None:
            ctx["frame"] = frame

        guard_reason: Optional[str] = None
        if spec.guard is not None:
            guard = _GUARDS[spec.guard]
            guard_reason = guard(ctx)
            if guard_reason is not None:
                raise GuardRejectedError(
                    f"状态 {self._state.value} 的事件 {event.value} 被守卫 "
                    f"{spec.guard} 拒绝：{guard_reason}",
                    state=self._state,
                    event=event,
                    guard_reason=guard_reason,
                )

        source = self._state
        self._state = spec.target
        transition = Transition(
            source=source,
            target=spec.target,
            event=event,
            reason=reason if reason is not None else _default_reason(event),
            timestamp_s=float(self._clock()),
            detail=detail,
        )
        self._history.append(transition)
        return transition

    # -- 便捷封装 --------------------------------------------------------
    def connect(self, **kwargs: Any) -> Transition:
        """``DISCONNECTED → CONNECTED``。"""
        kwargs.setdefault("reason", REASON_CONNECT)
        return self.transition(DeviceEvent.CONNECT, **kwargs)

    def disconnect(self, **kwargs: Any) -> Transition:
        """链路断开（拔网线）→ ``DISCONNECTED``。"""
        kwargs.setdefault("reason", REASON_LINK_DOWN)
        return self.transition(DeviceEvent.DISCONNECT, **kwargs)

    def enable(self, **kwargs: Any) -> Transition:
        """发出使能请求 → ``WAIT_ENABLE``。"""
        kwargs.setdefault("reason", REASON_ENABLE)
        return self.transition(DeviceEvent.ENABLE, **kwargs)

    def mark_all_enabled(self, frame: StateFrame, **kwargs: Any) -> Transition:
        """7 轴全部使能 → ``READY``（守卫会再校验一次）。"""
        kwargs.setdefault("reason", REASON_ALL_ENABLED)
        return self.transition(DeviceEvent.ALL_ENABLED, frame=frame, **kwargs)

    def start_stream(self, **kwargs: Any) -> Transition:
        """开始透传 → ``STREAMING``。"""
        kwargs.setdefault("reason", REASON_STREAM_START)
        return self.transition(DeviceEvent.START_STREAM, **kwargs)

    def stop_stream(self, **kwargs: Any) -> Transition:
        """停止透传 → ``READY``。"""
        kwargs.setdefault("reason", REASON_STREAM_STOP)
        return self.transition(DeviceEvent.STOP_STREAM, **kwargs)

    def hold(self, **kwargs: Any) -> Transition:
        """保持（暂停下发）→ ``HOLDING``。"""
        kwargs.setdefault("reason", REASON_HOLD)
        return self.transition(DeviceEvent.HOLD, **kwargs)

    def resume(self, **kwargs: Any) -> Transition:
        """恢复下发 → ``STREAMING``。"""
        kwargs.setdefault("reason", REASON_RESUME)
        return self.transition(DeviceEvent.RESUME, **kwargs)

    def fault(
        self,
        *,
        frame: Optional[StateFrame] = None,
        reason: Optional[str] = None,
        detail: str = "",
    ) -> Transition:
        """进入 ``FAULT``；``reason`` 缺省时由帧内容自动判定。"""
        if reason is None and frame is not None:
            reason = detect_fault(frame)
        return self.transition(
            DeviceEvent.FAULT,
            frame=frame,
            reason=reason if reason is not None else REASON_MANUAL,
            detail=detail,
        )

    def recover_begin(self, **kwargs: Any) -> Transition:
        """开始恢复流程 → ``RECOVER``。"""
        kwargs.setdefault("reason", REASON_RECOVER_BEGIN)
        return self.transition(DeviceEvent.RECOVER_BEGIN, **kwargs)

    def recover_end(self, frame: StateFrame, **kwargs: Any) -> Transition:
        """恢复完成 → ``READY``（守卫：错误码清零 + 全轴使能）。"""
        kwargs.setdefault("reason", REASON_RECOVERED)
        return self.transition(DeviceEvent.RECOVER_END, frame=frame, **kwargs)

    def recover_failed(self, **kwargs: Any) -> Transition:
        """恢复失败 → 回到 ``FAULT``。"""
        kwargs.setdefault("reason", REASON_RECOVER_FAILED)
        return self.transition(DeviceEvent.RECOVER_FAILED, **kwargs)

    def estop(self, **kwargs: Any) -> Transition:
        """急停 → ``ESTOP``。"""
        kwargs.setdefault("reason", REASON_ESTOP_STATUS)
        return self.transition(DeviceEvent.ESTOP, **kwargs)

    def clear_estop(self, **kwargs: Any) -> Transition:
        """清除急停 → ``DISCONNECTED``（必须重新走链路）。"""
        kwargs.setdefault("reason", REASON_CLEARED)
        return self.transition(DeviceEvent.CLEAR_ESTOP, **kwargs)

    def reset(self, **kwargs: Any) -> Transition:
        """复位 → ``DISCONNECTED``。"""
        kwargs.setdefault("reason", REASON_MANUAL)
        return self.transition(DeviceEvent.RESET, **kwargs)


def _default_reason(event: DeviceEvent) -> str:
    """事件 → 默认 reason 标签。"""
    mapping = {
        DeviceEvent.CONNECT: REASON_CONNECT,
        DeviceEvent.DISCONNECT: REASON_DISCONNECT,
        DeviceEvent.ENABLE: REASON_ENABLE,
        DeviceEvent.ALL_ENABLED: REASON_ALL_ENABLED,
        DeviceEvent.START_STREAM: REASON_STREAM_START,
        DeviceEvent.STOP_STREAM: REASON_STREAM_STOP,
        DeviceEvent.HOLD: REASON_HOLD,
        DeviceEvent.RESUME: REASON_RESUME,
        DeviceEvent.FAULT: REASON_MANUAL,
        DeviceEvent.RECOVER_BEGIN: REASON_RECOVER_BEGIN,
        DeviceEvent.RECOVER_END: REASON_RECOVERED,
        DeviceEvent.RECOVER_FAILED: REASON_RECOVER_FAILED,
        DeviceEvent.ESTOP: REASON_ESTOP_STATUS,
        DeviceEvent.CLEAR_ESTOP: REASON_CLEARED,
        DeviceEvent.RESET: REASON_MANUAL,
    }
    return mapping[event]
