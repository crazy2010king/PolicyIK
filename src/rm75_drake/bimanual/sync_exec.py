"""双臂同一 tick 同步执行器（S04-T07）。

它补上的正是 S03 SUMMARY 里「监控缺口 #3」原文所指的观测面：

    双臂时间对齐仅在单进程内保证，``rm_exec`` 尚无跨臂绝对时间戳对齐的可观测输出。

模块边界（**不得**新建第二套实时层 / 第二套指标口径）
------------------------------------------------------
本模块**只做编排**，所有底层能力都**复用** S03 的既有实现：

============================  ==========================================
能力                          复用来源（唯一真相源）
============================  ==========================================
节拍 / 周期 / 尾保持 / 事件上限  :class:`rm75_drake.device.exec_loop.ExecLoopConfig`
时钟（生产 / 虚拟）            :class:`rm75_drake.device.exec_loop.SystemClock` /
                              :class:`rm75_drake.device.exec_loop.VirtualClock`
安全七闸（钳位 + 监视）        :func:`rm75_drake.device.safety.apply_safety`
阈值                          :class:`rm75_drake.device.safety.SafetyLimits`
直方图口径（字段名 / 分位数）   :class:`rm75_drake.device.metrics.MetricHistogram` /
                              :class:`rm75_drake.device.metrics.JitterHistogram`
稳定 reason 标签 / 停止语义     :mod:`rm75_drake.device.safety` /
                              :mod:`rm75_drake.device.exec_loop`
============================  ==========================================

因此**没有**第二套时钟、**没有**第二套安全闸、**没有**第二套直方图口径；
也**没有**给 ``rm75_drake.drivers`` 追加任何 abstractmethod（既有 Driver
子类不受影响）。

一拍（tick）做什么
------------------
1. 按固定次序读**两臂**状态帧；
2. **逐臂**过 :func:`apply_safety`（七闸）——每臂各自维护 ``prev_command`` /
   ``prev_velocity``，互不串扰；
3. 按固定次序（默认 ``left → right``）**依次下发**；
4. 记录两臂的**下发时刻**与**下发时间差（ns）**，并写入
   :class:`~rm75_drake.device.metrics.MetricHistogram`。

安全态语义（与 S03 一致，但有一处**显式**加强）
------------------------------------------------
* **监视类停机闸**（G4 慢看门狗 / G5 电流 / G6 围栏 / G7 状态超时，
  ``action ∈ {stop, slow_stop}``）：任一臂触发 ⇒ **两臂同时停止下发**并进入
  安全态（``stop`` ⇒ ``emergency_stop``；``slow_stop`` ⇒ ``slow_stop``），
  停止原因取 S03 的**稳定 reason 标签**（如 ``fence`` / ``state_age``）。
* **钳位类违规**（G1 软限位 / G2 增量 / G3 加速度，``action == clamp``）：
  与 S03 单臂执行环一致——**钳位后继续下发**，不因此失败。
  **唯一的加强**：``soft_limit``（软限位）例外——轨迹点**越出关节限位**属于
  *规划错误*，静默钳位会掩盖它，因此双臂执行器**停发**并把
  ``soft_limit`` 作为停止原因上报（见 :data:`DUAL_ARM_HALT_REASONS`）。
  这是本模块相对 S03 单臂语义**唯一**的加强，且只加严、不放松。
* **传输层错误**：与 S03 一致 ⇒ 记录错误、停止下发、``final_state`` 取
  ``transport_error``。

「< 1 ms」**不声称**已达标
--------------------------
物理口径「两臂下发时间差 < 1 ms」依赖**真机**（真实时钟 + 真实网络栈）。
本环境**无真机、无实测数据**，因此：

* 结果对象恒带 ``hardware_required = True``；
* ``unverified_metrics`` 恒含 ``"双臂下发时间差 < 1 ms（真机）"``；
* 离线（虚拟时钟）只能证明「**同一 tick 内按固定次序下发**」与
  「**逻辑时间差可测**」——虚拟时钟下时间差不代表真机时延。

这正是 MEM083 纪律：**指标不达标式声称**。

失败形状
--------
所有本模块自己发现的输入错误都抛 :class:`SyncExecError`（``ValueError``
家族），并带稳定 ``reason`` 与**点名**信息：

* ``trajectory_length_mismatch`` —— 左右轨迹点数不一致（点名两侧点数）；
* ``joint_dimension_mismatch`` —— 轨迹点不是 7 维有限数值（点名臂与下标）；
* ``unknown_arm`` —— ``order`` 含未知臂名（点名合法臂名）；
* ``duplicate_arm`` —— ``order`` 里同一臂出现多次；
* ``invalid_max_ticks`` —— ``max_ticks`` 非法（非正整数）。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

from ..device.exec_loop import (
    REASON_COMPLETED,
    REASON_SAFE_STOP,
    REASON_TRANSPORT_ERROR,
    Clock,
    ExecLoopConfig,
    SystemClock,
)
from ..device.metrics import JitterHistogram, MetricHistogram
from ..device.safety import (
    ACTION_SLOW_STOP,
    ACTION_STOP,
    REASON_SOFT_LIMIT,
    SafetyLimits,
    SafetyState,
    SafetyViolation,
    apply_safety,
)
from ..device.transport import ArmTransport, StateFrame, TransportError
from ..planning.trajectory import JOINT_COUNT

__all__ = [
    "ARM_LEFT",
    "ARM_RIGHT",
    "ARMS",
    "DUAL_ARM_HALT_REASONS",
    "UNVERIFIED_DISPATCH_SKEW_METRIC",
    "SyncExecError",
    "DualSyncResult",
    "DualSyncExecutor",
]

#: 臂名（唯一真相源：只认这两个）。
ARM_LEFT = "left"
ARM_RIGHT = "right"

#: 合法臂名集合。
ARMS = (ARM_LEFT, ARM_RIGHT)

#: 双臂执行器**额外**停发的 reason（相对 S03 单臂语义只加严，见模块文档）。
DUAL_ARM_HALT_REASONS = (REASON_SOFT_LIMIT,)

#: 未实测指标名（真机「双臂下发时间差 < 1 ms」）。
UNVERIFIED_DISPATCH_SKEW_METRIC = "双臂下发时间差 < 1 ms（真机）"

#: 下发时间差直方图名（与 ``device/metrics.py`` 的命名口径一致）。
DISPATCH_SKEW_HISTOGRAM_NAME = "dual_arm_dispatch_skew"


class SyncExecError(ValueError):
    """双臂执行器的输入错误（稳定 ``reason`` + 点名信息）。"""

    def __init__(self, message: str, *, reason: str = "invalid_input") -> None:
        super().__init__(message)
        self.reason = str(reason)


# ── 结果 ────────────────────────────────────────────────────────────────
@dataclass
class DualSyncResult:
    """双臂同步执行的结果（**证据**，可直接写进运行日志）。"""

    ticks: int = 0
    left_dispatches: int = 0
    right_dispatches: int = 0
    final_state: str = "running"
    stop_reason: Optional[str] = None
    violations: List[SafetyViolation] = field(default_factory=list)
    violation_records: List[Dict[str, Any]] = field(default_factory=list)
    errors: List[Dict[str, Any]] = field(default_factory=list)
    dispatch_skew_ns: List[int] = field(default_factory=list)
    dispatch_skew: MetricHistogram = field(
        default_factory=lambda: MetricHistogram(DISPATCH_SKEW_HISTOGRAM_NAME, "s")
    )
    jitter: JitterHistogram = field(default_factory=JitterHistogram)
    tick_intervals_s: List[float] = field(default_factory=list)
    hardware_required: bool = True
    unverified_metrics: Tuple[str, ...] = (UNVERIFIED_DISPATCH_SKEW_METRIC,)
    events_dropped: int = 0

    @property
    def reasons(self) -> Tuple[str, ...]:
        """本次触发的安全 reason 标签（去重、保序；S03 同口径）。"""
        seen: List[str] = []
        for violation in self.violations:
            if violation.reason not in seen:
                seen.append(violation.reason)
        return tuple(seen)

    @property
    def ok(self) -> bool:
        """是否**正常走完**轨迹（无安全停机、无传输错误）。"""
        return self.final_state == REASON_COMPLETED

    @property
    def dispatch_skew_ms(self) -> Optional[float]:
        """两臂下发时间差的最大值（ms）；无样本时为 ``None``。"""
        maximum = self.dispatch_skew.max
        return None if maximum is None else float(maximum) * 1000.0

    def to_dict(self) -> Dict[str, Any]:
        """返回可 JSON 序列化的摘要。"""
        return {
            "ticks": self.ticks,
            "left_dispatches": self.left_dispatches,
            "right_dispatches": self.right_dispatches,
            "final_state": self.final_state,
            "stop_reason": self.stop_reason,
            "reasons": list(self.reasons),
            "violations": [v.to_dict() for v in self.violations],
            "violation_records": list(self.violation_records),
            "errors": list(self.errors),
            "dispatch_skew_ns": list(self.dispatch_skew_ns),
            "dispatch_skew": self.dispatch_skew.summary(),
            "dispatch_skew_ms": self.dispatch_skew.summary_ms(),
            "jitter": self.jitter.summary(),
            "tick_intervals_s": list(self.tick_intervals_s),
            "hardware_required": self.hardware_required,
            "unverified_metrics": list(self.unverified_metrics),
            "events_dropped": self.events_dropped,
        }


# ── 内部状态 ────────────────────────────────────────────────────────────
@dataclass
class _ArmState:
    """单臂在执行器内部的可变状态。"""

    name: str
    transport: ArmTransport
    points: Tuple[Tuple[float, ...], ...]
    prev_command: List[float]
    prev_velocity: List[float]
    dispatches: int = 0


# ── 执行器 ──────────────────────────────────────────────────────────────
class DualSyncExecutor:
    """双臂同一 tick 同步执行器（零 SDK 依赖：只依赖传输层边界）。

    参数
    ----
    left_transport / right_transport:
        两臂的传输层（真机 :class:`~rm75_drake.device.transport.SdkTransport` /
        离线 :class:`~rm75_drake.device.transport.ScriptedTransport`）。
        两者都必须已 ``open()``。
    left_points / right_points:
        两臂的关节目标序列（rad，每点 7 维）。**点数必须一致**
        （否则无法在同一 tick 上对齐 ⇒ 显式报错）。
    limits:
        安全七闸阈值（:class:`~rm75_drake.device.safety.SafetyLimits`）——
        两臂共用同一份阈值。
    config:
        执行环配置（:class:`~rm75_drake.device.exec_loop.ExecLoopConfig`）——
        节拍周期、``follow``、``require_state``、事件上限等。
    clock:
        时钟（默认 :class:`~rm75_drake.device.exec_loop.SystemClock`；
        测试注入 :class:`~rm75_drake.device.exec_loop.VirtualClock`）。
    order:
        下发次序（默认 ``("left", "right")``）——**必须**是
        ``("left", "right")`` 的一个排列。
    sink:
        事件回调 ``(event, payload) -> None``；**环内不做 I/O**。
    """

    def __init__(
        self,
        left_transport: ArmTransport,
        right_transport: ArmTransport,
        *,
        left_points: Sequence[Sequence[float]],
        right_points: Sequence[Sequence[float]],
        limits: SafetyLimits,
        config: Optional[ExecLoopConfig] = None,
        clock: Optional[Clock] = None,
        order: Tuple[str, ...] = (ARM_LEFT, ARM_RIGHT),
        sink: Optional[Callable[[str, Dict[str, Any]], None]] = None,
    ) -> None:
        left = _validate_points(left_points, f"{ARM_LEFT}_points")
        right = _validate_points(right_points, f"{ARM_RIGHT}_points")
        if len(left) != len(right):
            raise SyncExecError(
                f"左右轨迹点数不一致：{ARM_LEFT}={len(left)} 点，"
                f"{ARM_RIGHT}={len(right)} 点（双臂必须逐拍对齐）",
                reason="trajectory_length_mismatch",
            )
        self._order = _validate_order(order)
        self._transports = {ARM_LEFT: left_transport, ARM_RIGHT: right_transport}
        self._points = {ARM_LEFT: left, ARM_RIGHT: right}
        self._limits = limits
        self._config = config if config is not None else ExecLoopConfig()
        self._clock: Clock = clock if clock is not None else SystemClock()
        self._sink = sink
        self._events: List[Tuple[str, Dict[str, Any]]] = []
        self._events_dropped = 0
        self._arms = {
            ARM_LEFT: _ArmState(
                name=ARM_LEFT,
                transport=left_transport,
                points=left,
                prev_command=list(left[0]),
                prev_velocity=[0.0] * JOINT_COUNT,
            ),
            ARM_RIGHT: _ArmState(
                name=ARM_RIGHT,
                transport=right_transport,
                points=right,
                prev_command=list(right[0]),
                prev_velocity=[0.0] * JOINT_COUNT,
            ),
        }

    # -- 观测 ------------------------------------------------------------
    @property
    def events(self) -> List[Tuple[str, Dict[str, Any]]]:
        """已采集的事件序列（``(event, payload)``）。"""
        return list(self._events)

    @property
    def config(self) -> ExecLoopConfig:
        """执行环配置。"""
        return self._config

    @property
    def order(self) -> Tuple[str, ...]:
        """下发次序。"""
        return self._order

    # -- 主循环 ----------------------------------------------------------
    def run(self, max_ticks: Optional[int] = None) -> DualSyncResult:
        """跑完整个轨迹（或提前进入安全态）。

        参数
        ----
        max_ticks:
            最多执行多少拍；``None`` ⇒ 用 ``config.max_ticks``；
            再 ``None`` ⇒ 轨迹点数 + ``config.tail_hold_ticks``。

        返回
        ----
        :class:`DualSyncResult`
            含拍数、两臂下发计数、逐拍下发时间差（ns）与直方图、
            安全违规列表、停止原因与未验证指标清单。
        """
        if max_ticks is not None:
            if isinstance(max_ticks, bool) or not isinstance(max_ticks, int):
                raise SyncExecError(
                    f"max_ticks 必须是整数或 None，实际是 {max_ticks!r}",
                    reason="invalid_max_ticks",
                )
            if max_ticks < 1:
                raise SyncExecError(
                    f"max_ticks 必须 ≥ 1，实际是 {max_ticks}",
                    reason="invalid_max_ticks",
                )
        result = DualSyncResult()
        try:
            self._run(result, max_ticks)
        finally:
            result.events_dropped = self._events_dropped
        return result

    def _run(self, result: DualSyncResult, max_ticks: Optional[int]) -> None:
        """执行器主体（由 :meth:`run` 包装，保证事件计数一定被回填）。"""
        config = self._config
        clock = self._clock
        total_points = len(self._arms[ARM_LEFT].points)
        if max_ticks is not None:
            limit = max_ticks
        elif config.max_ticks is not None:
            limit = config.max_ticks
        else:
            limit = total_points + config.tail_hold_ticks
        period_ns = int(round(config.period_s * 1e9))
        deadline_ns = clock.now_ns()
        prev_tick_ns: Optional[int] = None

        tick = 0
        while tick < limit:
            tick_start = clock.now_ns()
            if prev_tick_ns is not None:
                interval_s = (tick_start - prev_tick_ns) / 1e9
                result.tick_intervals_s.append(interval_s)
                result.jitter.add(abs(interval_s - config.period_s))
            prev_tick_ns = tick_start

            # ── 1. 读两臂状态 + 2. 逐臂过安全七闸 ────────────────────
            verdicts: Dict[str, Tuple[Tuple[float, ...], List[SafetyViolation], str]] = {}
            halt_reason: Optional[str] = None
            halt_action: Optional[str] = None
            for name in self._order:
                arm = self._arms[name]
                desired = (
                    arm.points[tick] if tick < total_points else arm.points[-1]
                )
                frame = self._read_state(name)
                state = self._build_state(frame, tick_start)
                verdict = apply_safety(
                    desired,
                    prev_command=arm.prev_command,
                    limits=self._limits,
                    state=state,
                    prev_velocity_rad_s=arm.prev_velocity,
                    dt_s=config.period_s,
                )
                verdicts[name] = (verdict.command, list(verdict.violations), verdict.action)
                for violation in verdict.violations:
                    result.violations.append(violation)
                    result.violation_records.append(
                        {
                            "arm": name,
                            "tick": tick,
                            "reason": violation.reason,
                            "gate": violation.gate,
                            "action": violation.action,
                            "severity": violation.severity,
                            "detail": violation.detail,
                        }
                    )
                    self._emit(
                        "violation",
                        {"arm": name, "tick": tick, **violation.to_dict()},
                    )
                # 停机类闸（stop / slow_stop）⇒ 立即停发（S03 语义）
                if verdict.action in (ACTION_STOP, ACTION_SLOW_STOP):
                    reason = _worst_reason(verdict.violations)
                    if halt_reason is None or _severity_rank(reason) > _severity_rank(
                        halt_reason
                    ):
                        halt_reason = reason
                        halt_action = verdict.action
                # 软限位越限 ⇒ 规划错误 ⇒ 双臂停发（只加严，不放松）
                elif any(v.reason == REASON_SOFT_LIMIT for v in verdict.violations):
                    if halt_reason is None:
                        halt_reason = REASON_SOFT_LIMIT
                        halt_action = "halt"

            if halt_reason is not None:
                self._enter_safe_state(halt_action or "halt")
                result.final_state = REASON_SAFE_STOP
                result.stop_reason = halt_reason
                result.ticks = tick
                result.left_dispatches = self._arms[ARM_LEFT].dispatches
                result.right_dispatches = self._arms[ARM_RIGHT].dispatches
                self._emit(
                    "safe_stop",
                    {"reason": halt_reason, "action": halt_action, "tick": tick},
                )
                return

            # ── 3. 按固定次序依次下发（记录下发时刻） ──────────────────
            issue_ns: Dict[str, int] = {}
            for name in self._order:
                command = verdicts[name][0]
                arm = self._arms[name]
                issue = clock.now_ns()
                issue_ns[name] = issue
                try:
                    arm.transport.move_j_canfd(command, follow=config.follow)
                except TransportError as exc:
                    result.errors.append(
                        {"reason": exc.reason, "message": str(exc), "tick": tick, "arm": name}
                    )
                    result.final_state = REASON_TRANSPORT_ERROR
                    result.stop_reason = exc.reason
                    result.ticks = tick
                    result.left_dispatches = self._arms[ARM_LEFT].dispatches
                    result.right_dispatches = self._arms[ARM_RIGHT].dispatches
                    self._emit(
                        "transport_error",
                        {
                            "reason": exc.reason,
                            "message": str(exc),
                            "tick": tick,
                            "arm": name,
                        },
                    )
                    return
                arm.dispatches += 1
                self._emit(
                    "dispatch",
                    {"arm": name, "tick": tick, "issue_ns": issue},
                )

            # ── 4. 记录两臂下发时间差（ns）+ 直方图 ────────────────────
            skew_ns = abs(issue_ns[ARM_RIGHT] - issue_ns[ARM_LEFT])
            result.dispatch_skew_ns.append(skew_ns)
            result.dispatch_skew.add(skew_ns / 1e9)

            # ── 5. 更新各臂上一拍（指令 + 速度） ───────────────────────
            dt = config.period_s
            for name in self._order:
                arm = self._arms[name]
                command = verdicts[name][0]
                for i in range(JOINT_COUNT):
                    arm.prev_velocity[i] = (command[i] - arm.prev_command[i]) / dt
                    arm.prev_command[i] = command[i]

            result.left_dispatches = self._arms[ARM_LEFT].dispatches
            result.right_dispatches = self._arms[ARM_RIGHT].dispatches

            # ── 6. 对齐到理想节拍栅格 ─────────────────────────────────
            deadline_ns += period_ns
            remaining_ns = deadline_ns - clock.now_ns()
            if remaining_ns > 0:
                clock.sleep(remaining_ns / 1e9)

            tick += 1
            result.ticks = tick

        result.left_dispatches = self._arms[ARM_LEFT].dispatches
        result.right_dispatches = self._arms[ARM_RIGHT].dispatches
        result.final_state = REASON_COMPLETED
        result.stop_reason = REASON_COMPLETED
        self._emit("completed", {"ticks": result.ticks})

    # -- 内部工具 --------------------------------------------------------
    def _read_state(self, name: str) -> Optional[StateFrame]:
        """读取某臂最新状态帧（读失败 ⇒ 按传输错误处理）。"""
        return self._arms[name].transport.read_state()

    def _build_state(self, frame: Optional[StateFrame], now_ns: int) -> SafetyState:
        """构造本拍的安全状态（电流 + age）——**与 S03 执行环同口径**。

        * 读到帧 ⇒ ``age = now − frame.received_at``；
        * 没读到帧 ⇒ **按「状态超时」处理**（age 取阈值的 2 倍）触发 G7
          停机——安全优先，绝不把「无数据」伪装成「正常」；
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

    def _enter_safe_state(self, action: str) -> None:
        """进入安全态：``stop`` ⇒ 两臂急停；``slow_stop`` ⇒ 两臂缓停。

        ``halt``（软限位越限）⇒ **不发停止指令**（保持当前位置即可），
        急停会引入不必要的机械冲击。

        停止指令本身也可能失败（断链）——失败**记录**而不是抛出，
        以免掩盖原始原因。
        """
        if action not in (ACTION_STOP, ACTION_SLOW_STOP):
            return
        for name in self._order:
            transport = self._arms[name].transport
            try:
                if action == ACTION_STOP:
                    transport.emergency_stop()
                else:
                    transport.slow_stop()
            except TransportError as exc:
                self._emit(
                    "safe_stop_failed",
                    {"arm": name, "reason": exc.reason, "message": str(exc)},
                )

    def _emit(self, event: str, payload: Dict[str, Any]) -> None:
        """采集事件（**环内无 I/O**；sink 由调用方保证非阻塞）。"""
        if len(self._events) >= self._config.max_events:
            self._events_dropped += 1
            return
        self._events.append((event, payload))
        if self._sink is not None:
            self._sink(event, payload)


# ── 校验与工具 ──────────────────────────────────────────────────────────
def _validate_points(
    points: Sequence[Sequence[float]], name: str
) -> Tuple[Tuple[float, ...], ...]:
    """校验一条轨迹（非空；每点 7 维有限数值）。"""
    if isinstance(points, (str, bytes)) or not isinstance(points, Sequence):
        raise SyncExecError(
            f"{name} 必须是序列，收到 {type(points).__name__}",
            reason="joint_dimension_mismatch",
        )
    if not points:
        raise SyncExecError(
            f"{name} 不能为空（双臂执行器至少需要一个轨迹点）",
            reason="joint_dimension_mismatch",
        )
    out: List[Tuple[float, ...]] = []
    for i, point in enumerate(points):
        if isinstance(point, (str, bytes)) or not isinstance(point, Sequence):
            raise SyncExecError(
                f"{name}[{i}] 必须是 {JOINT_COUNT} 维序列，"
                f"收到 {type(point).__name__}",
                reason="joint_dimension_mismatch",
            )
        if len(point) != JOINT_COUNT:
            raise SyncExecError(
                f"{name}[{i}] 必须是 {JOINT_COUNT} 维序列（实际 {len(point)} 维）",
                reason="joint_dimension_mismatch",
            )
        values: List[float] = []
        for j, value in enumerate(point):
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise SyncExecError(
                    f"{name}[{i}][{j}] 必须是数值，收到 {value!r}",
                    reason="joint_dimension_mismatch",
                )
            number = float(value)
            if not math.isfinite(number):
                raise SyncExecError(
                    f"{name}[{i}][{j}] 必须是有限数值，收到 {value!r}",
                    reason="joint_dimension_mismatch",
                )
            values.append(number)
        out.append(tuple(values))
    return tuple(out)


def _validate_order(order: Sequence[str]) -> Tuple[str, ...]:
    """校验下发次序：必须是 ``("left", "right")`` 的一个排列。"""
    if isinstance(order, (str, bytes)) or not isinstance(order, Sequence):
        raise SyncExecError(
            f"order 必须是序列，收到 {type(order).__name__}", reason="unknown_arm"
        )
    names = tuple(str(name) for name in order)
    unknown = [name for name in names if name not in ARMS]
    if unknown:
        raise SyncExecError(
            f"order 含未知臂名 {unknown}（合法臂名：{list(ARMS)}）",
            reason="unknown_arm",
        )
    if len(names) != len(ARMS) or len(set(names)) != len(ARMS):
        raise SyncExecError(
            f"order 必须恰好包含每个臂一次（合法臂名：{list(ARMS)}），实际是 {list(names)}",
            reason="duplicate_arm",
        )
    return names


def _worst_reason(violations: Sequence[SafetyViolation]) -> str:
    """从违规列表里挑出**最严重**的稳定 reason 标签（S03 同口径）。"""
    for violation in violations:
        if violation.action == ACTION_STOP:
            return violation.reason
    for violation in violations:
        if violation.action == ACTION_SLOW_STOP:
            return violation.reason
    for violation in reversed(violations):
        if violation.action == "clamp":
            return violation.reason
    return REASON_SAFE_STOP


def _severity_rank(reason: str) -> int:
    """reason 的严重度排序（越大越严重）——用于多臂同时违规时取最严重者。"""
    order = (REASON_SOFT_LIMIT,)
    if reason in order:
        return 1
    return 2
