"""5 ms 实时执行环（S03-T04）。

为什么必须可注入时钟
--------------------
研究 F1：真机在当前环境**不可达**，但「5 ms 透传实时性」的口径
（P50 / P99 / P99.9 抖动、命令→执行延迟）必须能在 **CI 里确定性复核**。
⇒ 时钟与睡眠被收敛成 :class:`Clock` 协议：

* 生产：:class:`SystemClock`（``time.perf_counter_ns`` + ``time.sleep``）；
* 测试：:class:`VirtualClock`（**虚拟时间**，每拍间隔可精确手算）。

一拍的流水线
------------
.. code-block:: text

    取轨迹点 → 读状态（age / 电流） → 安全七闸钳位 → 下发（rm_movej_canfd）
    → 累计抖动 / 延迟 → 对齐到理想节拍栅格 → 下一拍

纪律
----
* **唯一运动通道**：:meth:`ArmTransport.move_j_canfd`（``follow=True``、
  ``trajectory_mode=0``，由传输层负责）。
* **环内禁 I/O**：不写日志、不打印；事件通过**可注入 sink** 外送，
  违规只进内存列表。
* **预分配**：指标直方图与指令缓冲都预分配，稳态下无周期性分配。
* **违规不吞掉**：全部进 :attr:`ExecLoopResult.violations`。
* **安全优先**：七闸要求 ``stop`` / ``slow_stop`` 时**立即停发**并
  上报稳定 reason（``watchdog`` / ``state_age`` / ``current`` / ``fence`` …）。

实时调度（可选）
----------------
:func:`apply_realtime_scheduling` 尝试 ``SCHED_FIFO`` + CPU 亲和性；
**失败必须显式降级并记录**（:class:`RtSchedulingReport`），
**不得假装已生效**。
"""

from __future__ import annotations

import math
import os
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

try:  # pragma: no cover - typing only
    from typing import Protocol
except ImportError:  # pragma: no cover
    Protocol = object  # type: ignore[assignment]

from ..planning.trajectory import JOINT_COUNT
from .metrics import JitterHistogram, LatencyHistogram
from .safety import (
    ACTION_CLAMP,
    ACTION_SLOW_STOP,
    ACTION_STOP,
    SafetyLimits,
    SafetyState,
    SafetyViolation,
    apply_safety,
)
from .transport import ArmTransport, StateFrame, TransportError

__all__ = [
    "PERIOD_S",
    "REASON_COMPLETED",
    "REASON_SAFE_STOP",
    "REASON_TRANSPORT_ERROR",
    "REASON_STATE_UNAVAILABLE",
    "Clock",
    "SystemClock",
    "VirtualClock",
    "ExecLoopConfig",
    "RtSchedulingReport",
    "ExecLoopResult",
    "ExecLoop",
    "apply_realtime_scheduling",
]


#: 5 ms 透传周期（秒）。
PERIOD_S = 0.005

#: 稳定 reason 标签。
REASON_COMPLETED = "completed"
REASON_SAFE_STOP = "safe_stop"
REASON_TRANSPORT_ERROR = "transport_error"
REASON_STATE_UNAVAILABLE = "state_unavailable"


# ── 时钟协议 ────────────────────────────────────────────────────────────
class Clock(Protocol):
    """执行环依赖的**最小**时钟协议（可注入）。"""

    def now_ns(self) -> int:
        """返回当前单调时刻（**纳秒**）。"""

    def sleep(self, seconds: float) -> None:
        """睡眠（秒）；实现可把虚拟时间直接推进。"""


class SystemClock:
    """生产时钟：``time.perf_counter_ns`` + ``time.sleep``。"""

    __slots__ = ()

    def now_ns(self) -> int:
        """返回当前单调时刻（纳秒）。"""
        return time.perf_counter_ns()

    def sleep(self, seconds: float) -> None:
        """睡眠（秒）；非正值直接返回。"""
        if seconds > 0.0:
            time.sleep(seconds)


class VirtualClock:
    """**确定性虚拟时钟**（测试用；零真实等待）。

    * :meth:`sleep` 把虚拟时间**精确推进**给定的秒数；
    * :meth:`advance` 可模拟「本拍的工作耗时」（用于制造抖动）；
    * :attr:`sleep_calls` 记录每次睡眠请求（断言用）。
    """

    __slots__ = ("_now_ns", "sleep_calls")

    def __init__(self, start_ns: int = 0) -> None:
        self._now_ns = int(start_ns)
        self.sleep_calls: List[float] = []

    # -- 时钟协议 --------------------------------------------------------
    def now_ns(self) -> int:
        """返回当前虚拟时刻（纳秒）。"""
        return self._now_ns

    def sleep(self, seconds: float) -> None:
        """把虚拟时间推进 ``seconds``（秒）。"""
        value = float(seconds)
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f"seconds 必须是非负有限数值，实际是 {seconds!r}")
        self.sleep_calls.append(value)
        self._now_ns += int(round(value * 1e9))

    # -- 辅助 ------------------------------------------------------------
    @property
    def now_s(self) -> float:
        """当前虚拟时刻（秒）。"""
        return self._now_ns / 1e9

    def advance(self, seconds: float) -> None:
        """推进虚拟时间（模拟工作耗时）。"""
        value = float(seconds)
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f"seconds 必须是非负有限数值，实际是 {seconds!r}")
        self._now_ns += int(round(value * 1e9))

    def advance_ns(self, nanoseconds: int) -> None:
        """推进虚拟时间（纳秒）。"""
        self._now_ns += int(nanoseconds)


# ── 配置 ────────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class ExecLoopConfig:
    """执行环配置（**不可变**）。

    字段
    ----
    period_s:
        固定节拍周期（s，默认 5 ms）。
    follow:
        透传是否跟随（真机必须 ``True``）。
    max_ticks:
        最多执行多少拍；``None`` ⇒ 轨迹点数 + ``tail_hold_ticks``。
    tail_hold_ticks:
        轨迹走完后的**保持拍数**（继续下发最后一个点）；默认 ``0``
        ⇒ **不多发**（下发次数 == 轨迹点数）。
    require_state:
        是否要求每拍都能读到状态帧。``True``（默认）时读不到状态 ⇒
        按「状态超时」处理并停机（**安全优先**）；``False`` 时
        允许无状态运行（离线 dry-run），此时 age 记为 0（**明确标注
        为「未知」，不是测量值**）。
    enable_rt_scheduling:
        是否尝试 ``SCHED_FIFO`` + CPU 亲和性。
    rt_priority:
        ``SCHED_FIFO`` 优先级（1–99）。
    cpu_affinity:
        CPU 亲和性集合；``None`` ⇒ 不设置。
    max_events:
        事件缓冲上限（超出后**停止追加**并计数，不静默丢弃）。
    """

    period_s: float = PERIOD_S
    follow: bool = True
    max_ticks: Optional[int] = None
    tail_hold_ticks: int = 0
    require_state: bool = True
    enable_rt_scheduling: bool = False
    rt_priority: int = 10
    cpu_affinity: Optional[Tuple[int, ...]] = None
    max_events: int = 4096

    def __post_init__(self) -> None:
        period = float(self.period_s)
        if not math.isfinite(period) or period <= 0.0:
            raise ValueError(f"period_s 必须是正有限数值，实际是 {self.period_s!r}")
        object.__setattr__(self, "period_s", period)
        if self.max_ticks is not None:
            if isinstance(self.max_ticks, bool) or not isinstance(self.max_ticks, int):
                raise ValueError(f"max_ticks 必须是整数或 None，实际是 {self.max_ticks!r}")
            if self.max_ticks < 1:
                raise ValueError(f"max_ticks 必须 ≥ 1，实际是 {self.max_ticks}")
        if isinstance(self.tail_hold_ticks, bool) or not isinstance(
            self.tail_hold_ticks, int
        ):
            raise ValueError(
                f"tail_hold_ticks 必须是整数，实际是 {self.tail_hold_ticks!r}"
            )
        if self.tail_hold_ticks < 0:
            raise ValueError(
                f"tail_hold_ticks 必须 ≥ 0，实际是 {self.tail_hold_ticks}"
            )
        if isinstance(self.rt_priority, bool) or not isinstance(self.rt_priority, int):
            raise ValueError(f"rt_priority 必须是整数，实际是 {self.rt_priority!r}")
        if not 1 <= self.rt_priority <= 99:
            raise ValueError(
                f"rt_priority 必须在 [1, 99] 内，实际是 {self.rt_priority}"
            )
        if self.cpu_affinity is not None:
            if not self.cpu_affinity:
                raise ValueError("cpu_affinity 不能是空元组")
            for cpu in self.cpu_affinity:
                if isinstance(cpu, bool) or not isinstance(cpu, int) or cpu < 0:
                    raise ValueError(
                        f"cpu_affinity 必须是非负整数序列，实际是 {self.cpu_affinity!r}"
                    )
        if isinstance(self.max_events, bool) or not isinstance(self.max_events, int):
            raise ValueError(f"max_events 必须是整数，实际是 {self.max_events!r}")
        if self.max_events < 1:
            raise ValueError(f"max_events 必须 ≥ 1，实际是 {self.max_events}")


# ── 实时调度 ────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class RtSchedulingReport:
    """实时调度结果（**失败必须显式可见**）。

    字段
    ----
    applied:
        是否真的生效（``False`` ⇒ 已降级）。
    reason:
        ``not_requested`` / ``applied`` / ``degraded``。
    detail:
        人类可读说明（降级时必须包含失败原因）。
    """

    applied: bool
    reason: str
    detail: str

    def to_dict(self) -> Dict[str, Any]:
        """返回可 JSON 序列化的字典。"""
        return {
            "applied": bool(self.applied),
            "reason": self.reason,
            "detail": self.detail,
        }


def apply_realtime_scheduling(config: ExecLoopConfig) -> RtSchedulingReport:
    """尝试启用 ``SCHED_FIFO`` + CPU 亲和性。

    **失败不会抛出**，而是显式降级并把原因写进 :class:`RtSchedulingReport`
    （绝不假装已生效）。
    """
    if not config.enable_rt_scheduling:
        return RtSchedulingReport(
            False,
            "not_requested",
            "未请求实时调度（保持默认调度策略）",
        )
    notes: List[str] = []
    applied = True

    sched_setscheduler = getattr(os, "sched_setscheduler", None)
    sched_fifo = getattr(os, "SCHED_FIFO", None)
    if sched_setscheduler is None or sched_fifo is None:
        applied = False
        notes.append("平台不支持 os.sched_setscheduler / os.SCHED_FIFO")
    else:
        try:
            sched_setscheduler(
                0, sched_fifo, os.sched_param(config.rt_priority)  # type: ignore[arg-type]
            )
        except (PermissionError, OSError, AttributeError) as exc:
            applied = False
            notes.append(f"SCHED_FIFO 设置失败：{exc!r}")

    if config.cpu_affinity is not None:
        sched_setaffinity = getattr(os, "sched_setaffinity", None)
        if sched_setaffinity is None:
            applied = False
            notes.append("平台不支持 os.sched_setaffinity")
        else:
            try:
                sched_setaffinity(0, set(config.cpu_affinity))
            except (PermissionError, OSError, AttributeError) as exc:
                applied = False
                notes.append(f"CPU 亲和性设置失败：{exc!r}")

    if applied:
        return RtSchedulingReport(True, "applied", "SCHED_FIFO + CPU 亲和性已生效")
    return RtSchedulingReport(False, "degraded", "；".join(notes) or "实时调度降级")


# ── 结果 ────────────────────────────────────────────────────────────────
@dataclass
class ExecLoopResult:
    """执行环一次运行的结果（**证据**，可直接写进运行日志）。"""

    ticks: int = 0
    commands_sent: int = 0
    final_state: str = "running"
    stop_reason: Optional[str] = None
    violations: List[SafetyViolation] = field(default_factory=list)
    errors: List[Dict[str, Any]] = field(default_factory=list)
    tick_intervals_s: List[float] = field(default_factory=list)
    jitter: JitterHistogram = field(default_factory=JitterHistogram)
    latency: LatencyHistogram = field(default_factory=LatencyHistogram)
    rt_report: RtSchedulingReport = field(
        default_factory=lambda: RtSchedulingReport(False, "not_requested", "未运行")
    )
    events_dropped: int = 0

    @property
    def reasons(self) -> Tuple[str, ...]:
        """本次触发的安全 reason 标签（去重、保序）。"""
        seen: List[str] = []
        for violation in self.violations:
            if violation.reason not in seen:
                seen.append(violation.reason)
        return tuple(seen)

    @property
    def ok(self) -> bool:
        """是否**正常走完**轨迹（无安全停机、无传输错误）。"""
        return self.final_state == REASON_COMPLETED

    def to_dict(self) -> Dict[str, Any]:
        """返回可 JSON 序列化的摘要。"""
        return {
            "ticks": self.ticks,
            "commands_sent": self.commands_sent,
            "final_state": self.final_state,
            "stop_reason": self.stop_reason,
            "reasons": list(self.reasons),
            "violations": [v.to_dict() for v in self.violations],
            "errors": list(self.errors),
            "tick_intervals_s": list(self.tick_intervals_s),
            "jitter": self.jitter.summary(),
            "command_latency": self.latency.summary(),
            "rt_scheduling": self.rt_report.to_dict(),
            "events_dropped": self.events_dropped,
        }


# ── 执行环 ──────────────────────────────────────────────────────────────
class ExecLoop:
    """5 ms 固定节拍执行环（**零 SDK 依赖**：只依赖传输层边界）。

    参数
    ----
    transport:
        传输层（真机 :class:`~rm75_drake.device.transport.SdkTransport` /
        离线 :class:`~rm75_drake.device.transport.ScriptedTransport`）。
    points:
        关节目标序列（rad，每点 7 维）。
    limits:
        安全七闸阈值（:class:`~rm75_drake.device.safety.SafetyLimits`）。
    config:
        执行环配置。
    clock:
        时钟（默认 :class:`SystemClock`；测试注入 :class:`VirtualClock`）。
    sink:
        事件回调 ``(event, payload) -> None``；**环内不做 I/O**，
        由调用方保证 sink 非阻塞。
    state_provider:
        状态提供者 ``() -> Optional[StateFrame]``；``None`` ⇒ 用
        ``transport.read_state()``。
    initial_command:
        上一拍指令的初值（增量/加速度闸的基准）；``None`` ⇒ 取轨迹首点
        （避免起步拍被误判为「巨大跳变」）。
    """

    def __init__(
        self,
        transport: ArmTransport,
        points: Sequence[Sequence[float]],
        limits: SafetyLimits,
        *,
        config: Optional[ExecLoopConfig] = None,
        clock: Optional[Clock] = None,
        sink: Optional[Callable[[str, Dict[str, Any]], None]] = None,
        state_provider: Optional[Callable[[], Optional[StateFrame]]] = None,
        initial_command: Optional[Sequence[float]] = None,
    ) -> None:
        self._transport = transport
        self._points = _validate_points(points)
        self._limits = limits
        self._config = config if config is not None else ExecLoopConfig()
        self._clock: Clock = clock if clock is not None else SystemClock()
        self._sink = sink
        self._state_provider = state_provider
        if initial_command is None:
            self._initial_command: Tuple[float, ...] = self._points[0]
        else:
            self._initial_command = tuple(
                float(v) for v in _validate_vector(initial_command, "initial_command")
            )
        self._events: List[Tuple[str, Dict[str, Any]]] = []
        self._events_dropped = 0
        # 预分配：稳态下不产生新的列表/字典分配（指令缓冲复用）
        self._command_buffer: List[float] = [0.0] * JOINT_COUNT
        self._prev_velocity: List[float] = [0.0] * JOINT_COUNT

    # -- 观测 ------------------------------------------------------------
    @property
    def events(self) -> List[Tuple[str, Dict[str, Any]]]:
        """已采集的事件序列（``(event, payload)``）。"""
        return list(self._events)

    @property
    def config(self) -> ExecLoopConfig:
        """执行环配置。"""
        return self._config

    # -- 主循环 ----------------------------------------------------------
    def run(self) -> ExecLoopResult:
        """跑完整个轨迹（或提前进入安全态）。

        返回
        ----
        :class:`ExecLoopResult`
            含拍数、下发数、安全违规、抖动 / 延迟统计与停止原因。
        """
        result = ExecLoopResult()
        try:
            self._run(result)
        finally:
            result.events_dropped = self._events_dropped
        return result

    def _run(self, result: ExecLoopResult) -> None:
        """执行环主体（由 :meth:`run` 包装，保证事件计数一定被回填）。"""
        config = self._config
        clock = self._clock
        result.rt_report = apply_realtime_scheduling(config)
        if not result.rt_report.applied and config.enable_rt_scheduling:
            self._emit(
                "rt_degraded",
                {"reason": result.rt_report.reason, "detail": result.rt_report.detail},
            )

        total_points = len(self._points)
        max_ticks = (
            config.max_ticks
            if config.max_ticks is not None
            else total_points + config.tail_hold_ticks
        )
        period_ns = int(round(config.period_s * 1e9))
        deadline_ns = clock.now_ns()
        prev_tick_ns: Optional[int] = None

        prev_command = self._initial_command
        prev_velocity = self._prev_velocity

        tick = 0
        while tick < max_ticks:
            tick_start = clock.now_ns()
            jitter_s = 0.0
            if prev_tick_ns is not None:
                interval_s = (tick_start - prev_tick_ns) / 1e9
                result.tick_intervals_s.append(interval_s)
                # 抖动 = 实际周期与标称周期的偏差（睡眠过冲 / 拍内超时都会体现）
                jitter_s = abs(interval_s - config.period_s)

            # ── 取轨迹点（末尾保持：不多发）────────────────────────────
            if tick < total_points:
                desired = self._points[tick]
            else:
                desired = self._points[-1]

            # ── 读状态（age / 电流）────────────────────────────────────
            frame = self._read_state()
            state = self._build_state(frame, tick_start)

            # ── 安全七闸（唯一计算「钳位后指令」的地方）────────────────
            verdict = apply_safety(
                desired,
                prev_command=prev_command,
                limits=self._limits,
                state=state,
                prev_velocity_rad_s=prev_velocity,
                dt_s=config.period_s,
            )
            if verdict.violations:
                result.violations.extend(verdict.violations)
                for violation in verdict.violations:
                    self._emit("violation", violation.to_dict())

            # ── 安全态裁决（停发 / 缓停）──────────────────────────────
            if verdict.action in (ACTION_STOP, ACTION_SLOW_STOP):
                reason = self._worst_reason(verdict.violations)
                self._enter_safe_state(result, verdict.action, reason)
                result.final_state = REASON_SAFE_STOP
                result.stop_reason = reason
                result.ticks = tick
                self._emit(
                    "safe_stop",
                    {
                        "reason": reason,
                        "action": verdict.action,
                        "tick": tick,
                    },
                )
                return

            # ── 下发（唯一运动通道）────────────────────────────────────
            command = verdict.command
            issue_ns = clock.now_ns()
            try:
                self._transport.move_j_canfd(command, follow=config.follow)
            except TransportError as exc:
                result.errors.append(
                    {"reason": exc.reason, "message": str(exc), "tick": tick}
                )
                result.final_state = REASON_TRANSPORT_ERROR
                result.stop_reason = exc.reason
                result.ticks = tick
                self._emit(
                    "transport_error",
                    {"reason": exc.reason, "message": str(exc), "tick": tick},
                )
                return
            done_ns = clock.now_ns()

            result.latency.add((done_ns - issue_ns) / 1e9)
            if prev_tick_ns is not None:
                # 首拍没有「上一拍」⇒ 不产生抖动样本（避免伪造 0）。
                result.jitter.add(jitter_s)
            result.commands_sent += 1

            # ── 更新上一拍（指令 + 速度）───────────────────────────────
            dt = config.period_s
            for i in range(JOINT_COUNT):
                prev_velocity[i] = (command[i] - prev_command[i]) / dt
                self._command_buffer[i] = command[i]
            prev_command = tuple(self._command_buffer)

            # ── 对齐到理想节拍栅格 ─────────────────────────────────────
            deadline_ns += period_ns
            remaining_ns = deadline_ns - clock.now_ns()
            if remaining_ns > 0:
                clock.sleep(remaining_ns / 1e9)

            prev_tick_ns = tick_start
            tick += 1
            result.ticks = tick

        result.final_state = REASON_COMPLETED
        result.stop_reason = REASON_COMPLETED
        self._emit("completed", {"ticks": result.ticks})

    # -- 内部工具 --------------------------------------------------------
    def _read_state(self) -> Optional[StateFrame]:
        """读取最新状态帧（注入的 provider 优先）。"""
        if self._state_provider is not None:
            return self._state_provider()
        return self._transport.read_state()

    def _build_state(self, frame: Optional[StateFrame], now_ns: int) -> SafetyState:
        """构造本拍的安全状态（电流 + age）。

        * 读到帧 ⇒ ``age = now − frame.received_at``；
        * 没读到帧 ⇒ **按「状态超时」处理**（age 取阈值的 2 倍），
          触发 G7 停机——**安全优先**，绝不把「无数据」伪装成「正常」；
        * ``require_state=False`` ⇒ 允许无状态运行，age 记 0
          （明确是「未知」，不是测量值）。
        """
        now_s = now_ns / 1e9
        if frame is None:
            if not self._config.require_state:
                return SafetyState()
            timeout = float(self._limits.state_age_timeout_s)
            return SafetyState(state_age_s=timeout * 2.0)
        age = now_s - float(frame.received_at)
        if age < 0.0:
            age = 0.0
        return SafetyState(joint_currents_ma=frame.joint_currents_ma, state_age_s=age)

    def _worst_reason(self, violations: Sequence[SafetyViolation]) -> str:
        """从违规列表里挑出**最严重**的稳定 reason 标签。"""
        for violation in violations:
            if violation.action == ACTION_STOP:
                return violation.reason
        for violation in violations:
            if violation.action == ACTION_SLOW_STOP:
                return violation.reason
        for violation in reversed(violations):
            if violation.action == ACTION_CLAMP:
                return violation.reason
        return REASON_SAFE_STOP

    def _enter_safe_state(
        self, result: ExecLoopResult, action: str, reason: str
    ) -> None:
        """进入安全态：``stop`` ⇒ 急停；``slow_stop`` ⇒ 缓停。

        急停 / 缓停调用本身也可能失败（断链）——失败**记录**而不是抛出，
        以免掩盖原始原因。
        """
        try:
            if action == ACTION_STOP:
                self._transport.emergency_stop()
            else:
                self._transport.slow_stop()
        except TransportError as exc:
            result.errors.append(
                {"reason": exc.reason, "message": str(exc), "phase": "safe_stop"}
            )

    def _emit(self, event: str, payload: Dict[str, Any]) -> None:
        """采集事件（**环内无 I/O**；sink 由调用方保证非阻塞）。"""
        if len(self._events) >= self._config.max_events:
            self._events_dropped += 1
            return
        self._events.append((event, payload))
        if self._sink is not None:
            self._sink(event, payload)


def _validate_vector(values: Sequence[float], name: str) -> Tuple[float, ...]:
    """校验一个 7 维关节向量（长度 / 有限性）。"""
    if isinstance(values, (str, bytes)) or not isinstance(values, Sequence):
        raise ValueError(f"{name} 必须是 {JOINT_COUNT} 维序列，收到 {type(values).__name__}")
    if len(values) != JOINT_COUNT:
        raise ValueError(f"{name} 必须是 {JOINT_COUNT} 维序列（实际 {len(values)} 维）")
    out: List[float] = []
    for i, value in enumerate(values):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"{name}[{i}] 必须是数值，收到 {value!r}")
        number = float(value)
        if not math.isfinite(number):
            raise ValueError(f"{name}[{i}] 必须是有限数值，收到 {value!r}")
        out.append(number)
    return tuple(out)


def _validate_points(points: Sequence[Sequence[float]]) -> Tuple[Tuple[float, ...], ...]:
    """校验轨迹点序列（非空、每点 7 维有限）。"""
    if isinstance(points, (str, bytes)) or not isinstance(points, Sequence):
        raise ValueError(
            f"points 必须是序列，收到 {type(points).__name__}"
        )
    if not points:
        raise ValueError("points 不能为空（执行环至少需要一个轨迹点）")
    return tuple(_validate_vector(point, f"points[{i}]") for i, point in enumerate(points))
