"""真机驱动（S03-T06）：把设备层能力包成 :class:`~rm75_drake.drivers.base.Driver`。

它解决什么
----------
S02 的驱动协议（:class:`~rm75_drake.drivers.base.Driver`）只有
``open / close / submit_trajectory / read_state``，**没有**「流式执行」语义
（FakeDriver 的 ``step`` / ``replay`` / ``phase`` 都不在基类里）。
S03 补齐了流式协议（:class:`~rm75_drake.drivers.base.StreamingDriver`），
本模块把**真机**（或离线替身）接上去，让「假驱动 / 仿真驱动 / 真机驱动」
遵守**同一套**流式语义 ⇒ 同源断言可以在三者之间直接做。

分层纪律
--------
* **唯一运动通道**：:meth:`ArmTransport.move_j_canfd`
  （真机侧即 ``rm_movej_canfd``，``follow=True`` / ``trajectory_mode=0``）。
  驱动层**不得**调用任何其它运动接口。
* **安全七闸唯一实现**：钳位只发生在
  :func:`rm75_drake.device.safety.apply_safety`（本模块**复用**它，
  不复制任何钳位逻辑）；安全七闸的七闸实现见
  :mod:`rm75_drake.device.safety`。
* **状态不伪装**：读不到状态帧时按「状态超时」处理（``require_state=True``），
  **不**把「无数据」当成「正常」；离线 dry-run 必须显式
  ``require_state=False``。
* **驱动不得修改轨迹**：父类的深拷贝快照自检钩子
  (:meth:`~rm75_drake.drivers.base.Driver.verify_trajectory_unchanged`) 保留。

相位与流式语义
--------------
============  ==========================================================
``idle``      已打开，尚未提交轨迹。
``streaming`` 轨迹已提交，正在按 5 ms 节拍下发。
``done``      轨迹已全部下发完（之后 ``step()`` **不再下发**，只回读）。
``holding``   :meth:`hold` 之后：保持当前位置，不再推进。
``stopped``   :meth:`stop` 之后（或安全闸停机之后）：本次执行终止。
============  ==========================================================
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

from ..device.exec_loop import PERIOD_S, Clock, SystemClock
from ..device.metrics import JitterHistogram
from ..device.safety import (
    ACTION_SLOW_STOP,
    ACTION_STOP,
    SafetyLimits,
    SafetyState,
    SafetyViolation,
    apply_safety,
)
from ..device.transport import ArmTransport, StateFrame, TransportError
from ..planning.trajectory import DEFAULT_DT, JOINT_COUNT
from .base import (
    PHASE_DONE,
    PHASE_HOLDING,
    PHASE_STREAMING,
    PHASE_STOPPED,
    REASON_NO_TRAJECTORY,
    Driver,
    DriverError,
    DriverState,
)

__all__ = [
    "RmDeviceDriver",
    "RmDeviceDriverConfig",
    "REASON_STATE_UNAVAILABLE",
]

#: 真机驱动在「传输层报错」之外新增的稳定 reason。
REASON_STATE_UNAVAILABLE = "state_unavailable"


@dataclass(frozen=True)
class RmDeviceDriverConfig:
    """真机驱动的运行配置（**不可变**）。

    字段
    ----
    follow:
        透传是否跟随（真机必须 ``True``）。
    require_state:
        是否要求每拍都读得到状态帧。``True``（默认）时读不到状态 ⇒
        按「状态超时」处理并触发安全闸停机；``False`` ⇒ 允许离线
        dry-run（此时 age 记 0，**明确是「未知」而不是测量值**）。
    period_s:
        执行周期（s，默认 5 ms）——同时是安全闸 G2/G3 的 ``dt``。
    pace:
        是否在每拍末尾对齐到理想节拍栅格（5 ms 透传节拍）。
    max_violations:
        最多保留多少条安全违规（超出后**停止追加**并计数，不静默丢弃）。
    """

    follow: bool = True
    require_state: bool = True
    period_s: float = PERIOD_S
    pace: bool = True
    max_violations: int = 256

    def __post_init__(self) -> None:
        period = float(self.period_s)
        if not math.isfinite(period) or period <= 0.0:
            raise ValueError(
                f"period_s 必须是正有限数值，实际是 {self.period_s!r}"
            )
        object.__setattr__(self, "period_s", period)
        if isinstance(self.max_violations, bool) or not isinstance(
            self.max_violations, int
        ):
            raise ValueError(
                f"max_violations 必须是整数，实际是 {self.max_violations!r}"
            )
        if self.max_violations < 1:
            raise ValueError(
                f"max_violations 必须 ≥ 1，实际是 {self.max_violations}"
            )


class RmDeviceDriver(Driver):
    """真机驱动（把设备层包成统一的 Driver 接口）。

    参数
    ----
    transport:
        传输层：真机 :class:`~rm75_drake.device.transport.SdkTransport`，
        离线 :class:`~rm75_drake.device.transport.ScriptedTransport`。
    limits:
        安全七闸阈值（:class:`~rm75_drake.device.safety.SafetyLimits`）；
        ``None`` ⇒ 首次 ``open()`` 时从「限位交接件 + 设备配置」加载。
    config:
        运行配置（:class:`RmDeviceDriverConfig`）。
    clock:
        时钟（默认 :class:`~rm75_drake.device.exec_loop.SystemClock`；
        测试注入 :class:`~rm75_drake.device.exec_loop.VirtualClock`）。
    state_provider:
        状态提供者 ``() -> Optional[StateFrame]``；``None`` ⇒ 用
        ``transport.read_state()``（真机侧接 UDP 状态源）。
    dt:
        驱动节拍（s，默认 5 ms）——与轨迹采样周期必须一致。

    失败
    ----
    :class:`~rm75_drake.drivers.base.DriverError`
        未打开（``not_open`` / ``closed``）、未提交轨迹（``no_trajectory``）、
        安全闸停机（reason = 安全闸的稳定标签，如 ``state_age`` /
        ``fence`` / ``current``）、传输层失败（reason = 传输层标签，
        如 ``link_down``）。
    """

    name = "rm_device"

    def __init__(
        self,
        transport: ArmTransport,
        *,
        limits: Optional[SafetyLimits] = None,
        config: Optional[RmDeviceDriverConfig] = None,
        clock: Optional[Clock] = None,
        state_provider: Optional[Callable[[], Optional[StateFrame]]] = None,
        dt: float = DEFAULT_DT,
        name: str = "rm_device",
    ) -> None:
        super().__init__(dt=dt)
        self.transport = transport
        self.name = name
        self._limits = limits
        self._config = config if config is not None else RmDeviceDriverConfig(
            period_s=float(dt)
        )
        self._clock: Clock = clock if clock is not None else SystemClock()
        self._state_provider = state_provider
        self._points: List[Tuple[float, ...]] = []
        self._index = 0
        self._last_command: Tuple[float, ...] = (0.0,) * JOINT_COUNT
        self._last_velocity: Tuple[float, ...] = (0.0,) * JOINT_COUNT
        self._violations: List[SafetyViolation] = []
        self._violations_dropped = 0
        self._errors: List[Dict[str, Any]] = []
        self._jitter = JitterHistogram()
        self._prev_tick_ns: Optional[int] = None
        self._deadline_ns: int = 0

    # -- 观测 ------------------------------------------------------------
    @property
    def limits(self) -> SafetyLimits:
        """安全七闸阈值（首次访问时按需加载）。"""
        if self._limits is None:
            from ..device.safety import load_safety_limits

            self._limits = load_safety_limits()
        return self._limits

    @property
    def config(self) -> RmDeviceDriverConfig:
        """运行配置。"""
        return self._config

    @property
    def index(self) -> int:
        """下一个待下发的采样点下标（已下发的点数）。"""
        return self._index

    @property
    def num_samples(self) -> int:
        """已提交轨迹的采样点数（未提交时为 0）。"""
        return len(self._points)

    @property
    def violations(self) -> Tuple[SafetyViolation, ...]:
        """本次执行累积的安全违规（**不吞掉**）。"""
        return tuple(self._violations)

    @property
    def errors(self) -> Tuple[Dict[str, Any], ...]:
        """传输层 / 安全停机等错误记录。"""
        return tuple(self._errors)

    @property
    def jitter(self) -> JitterHistogram:
        """节拍抖动直方图（与真机脚本**同一口径**）。"""
        return self._jitter

    @property
    def state_age(self) -> Optional[float]:
        """最新状态帧的年龄（s）；无状态帧 ⇒ ``None``（**未知**，不是 0）。"""
        if not self.is_open:
            return None
        try:
            frame = self._read_frame()
        except TransportError:
            return None
        if frame is None:
            return None
        age = time.monotonic() - float(frame.received_at)
        return max(0.0, age)

    # -- 生命周期 --------------------------------------------------------
    def open(self) -> None:
        """打开传输层（幂等）。

        注意：本驱动**不会**自动使能 / 复位错误码 / 放宽限位——
        那些动作属于 :mod:`rm75_drake.device.recovery` 的恢复流程，
        驱动层**不越权**。
        """
        super().open()
        self.transport.open()
        self._index = 0
        self._points = []
        self._violations = []
        self._violations_dropped = 0
        self._errors = []
        self._jitter = JitterHistogram()
        self._prev_tick_ns = None

    def close(self) -> None:
        """关闭传输层（幂等）。"""
        self.transport.close()
        super().close()
        self._index = 0
        self._points = []

    # -- 轨迹提交 --------------------------------------------------------
    def _on_submit(self, trajectory: Any) -> None:
        """缓存轨迹点（**不修改**轨迹内容），相位进入 ``streaming``。"""
        samples = list(getattr(trajectory, "samples", ()))
        self._points = [tuple(float(v) for v in sample.q) for sample in samples]
        self._index = 0
        self._last_command = self._points[0] if self._points else (0.0,) * JOINT_COUNT
        self._last_velocity = (0.0,) * JOINT_COUNT
        self._violations = []
        self._violations_dropped = 0
        self._phase = PHASE_STREAMING
        self._prev_tick_ns = None
        self._deadline_ns = self._clock.now_ns()

    # -- 流式推进 --------------------------------------------------------
    def step(self) -> DriverState:
        """推进**一个** 5 ms 节拍：读状态 → 七闸钳位 → 透传下发。

        返回
        ----
        :class:`~rm75_drake.drivers.base.DriverState`
            本拍**实际下发**的关节角 + 该采样点的 seq / 绝对时刻。

        失败
        ----
        :class:`~rm75_drake.drivers.base.DriverError`
            * 未打开 / 已保持 / 已停止 / 未提交轨迹；
            * 安全闸要求停机（``reason`` = 安全闸稳定标签）；
            * 传输层失败（``reason`` = 传输层稳定标签，如 ``link_down``）。
        """
        self._require_open()
        self._require_streaming()
        trajectory = self.trajectory
        if trajectory is None:
            raise DriverError(
                "尚未提交轨迹，无法推进节拍", reason=REASON_NO_TRAJECTORY
            )
        samples = trajectory.samples
        index = min(self._index, len(samples) - 1)

        # 轨迹已走完：不再下发（**不多发**），只回读最后一点。
        if self._index >= len(samples):
            self._phase = PHASE_DONE
            sample = samples[index]
            return DriverState(
                joint_positions=self._last_command,
                seq=int(sample.seq),
                t_abs=float(sample.t_abs),
            )

        sample = samples[index]
        desired = self._points[index]

        frame = self._read_frame()
        state = self._build_state(frame)
        verdict = apply_safety(
            desired,
            prev_command=self._last_command,
            limits=self.limits,
            state=state,
            prev_velocity_rad_s=self._last_velocity,
            dt_s=self._config.period_s,
        )
        self._record_violations(verdict.violations)

        if verdict.action in (ACTION_STOP, ACTION_SLOW_STOP):
            reason = self._worst_reason(verdict.violations)
            self._enter_safe_state(verdict.action, reason)
            self._phase = PHASE_STOPPED
            self._errors.append(
                {"reason": reason, "message": "安全闸触发停机", "phase": "safe_stop"}
            )
            raise DriverError(
                f"安全闸触发停机（reason={reason}）", reason=reason
            )

        command = verdict.command
        try:
            self.transport.move_j_canfd(command, follow=self._config.follow)
        except TransportError as exc:
            self._phase = PHASE_STOPPED
            self._errors.append({"reason": exc.reason, "message": str(exc)})
            raise DriverError(f"透传下发失败：{exc}", reason=exc.reason) from exc

        # 更新上一拍（指令 + 速度）
        dt = self._config.period_s
        self._last_velocity = tuple(
            (command[i] - self._last_command[i]) / dt for i in range(JOINT_COUNT)
        )
        self._last_command = command
        self._index = index + 1
        if self._index >= len(samples):
            self._phase = PHASE_DONE

        self._record_tick()
        self._pace()
        return DriverState(
            joint_positions=command,
            seq=int(sample.seq),
            t_abs=float(sample.t_abs),
        )

    def replay(self) -> List[DriverState]:
        """逐拍执行整条轨迹，返回**逐点状态序列**（长度 == 采样点数）。"""
        self._require_open()
        trajectory = self.trajectory
        if trajectory is None:
            raise DriverError(
                "尚未提交轨迹，无法回放", reason=REASON_NO_TRAJECTORY
            )
        return [self.step() for _ in range(len(trajectory.samples))]

    def joint_sequence(self) -> List[tuple]:
        """执行整条轨迹并返回**关节角序列**（tuple 列表）。

        这是「同源比对」的最小接口：假驱动 / 仿真驱动 / 真机驱动
        对同一条 Planner 输出产出的关节序列必须逐点一致。
        """
        return [state.joint_positions for state in self.replay()]

    # -- 状态读取 --------------------------------------------------------
    def read_state(self) -> DriverState:
        """读回当前状态。

        * 有状态帧 ⇒ 关节角取**设备实测值**；
        * 无状态帧 ⇒ 回退到「最近一次下发的指令」（**命令回读**，
          不是设备实测；真机上这一路径应视为异常，配合
          :attr:`state_age` 一起看）。
        """
        self._require_open()
        trajectory = self.trajectory
        if trajectory is None:
            raise DriverError(
                "尚未提交轨迹，无法读回状态", reason=REASON_NO_TRAJECTORY
            )
        samples = trajectory.samples
        index = min(self._index, len(samples) - 1)
        sample = samples[index]
        frame = self._read_frame()
        self._mark_state_read()
        if frame is not None:
            positions = tuple(float(v) for v in frame.joint_positions_rad)
        else:
            positions = self._last_command
        return DriverState(
            joint_positions=positions,
            seq=int(sample.seq),
            t_abs=float(sample.t_abs),
        )

    def read_device_frame(self) -> Optional[StateFrame]:
        """读回**原始状态帧**（没有则 ``None``）。"""
        if not self.is_open:
            return None
        return self._read_frame()

    # -- 流式控制 --------------------------------------------------------
    def hold(self) -> DriverState:
        """保持当前位置：停止推进（不下发新指令）。"""
        self._require_open()
        self._held = True
        self._phase = PHASE_HOLDING
        return self.read_state()

    def stop(self) -> None:
        """终止本次流式执行：先**缓停**机械臂，再进入 ``stopped`` 相位。

        缓停失败（断链等）只记录、不抛出——不能因为「停不下来」
        而掩盖真正的失败原因。
        """
        self._require_open()
        try:
            self.transport.slow_stop()
        except TransportError as exc:
            self._errors.append(
                {"reason": exc.reason, "message": str(exc), "phase": "stop"}
            )
        super().stop()

    # -- 内部工具 --------------------------------------------------------
    def _read_frame(self) -> Optional[StateFrame]:
        """读取最新状态帧（注入的 provider 优先）。

        **帧完整性校验**在这里收口：``errCode == -3``（推送数据不完整）
        的帧**一律丢弃**（当作没有状态帧）——它绝不参与安全闸与状态机。
        """
        frame = (
            self._state_provider()
            if self._state_provider is not None
            else self.transport.read_state()
        )
        if frame is None:
            return None
        if not frame.is_complete:
            return None
        return frame

    def _build_state(self, frame: Optional[StateFrame]) -> SafetyState:
        """构造本拍的安全状态（电流 + age）。

        * 有帧 ⇒ ``age = now − frame.received_at``；
        * 无帧且 ``require_state=True`` ⇒ 按**状态超时**处理
          （age 取阈值 2 倍）⇒ 触发 G7 停机（**安全优先**）；
        * 无帧且 ``require_state=False`` ⇒ age 记 0
          （明确是「未知」，不是测量值）。
        """
        if frame is None:
            if not self._config.require_state:
                return SafetyState()
            timeout = float(self.limits.state_age_timeout_s)
            return SafetyState(state_age_s=timeout * 2.0)
        age = time.monotonic() - float(frame.received_at)
        if age < 0.0:
            age = 0.0
        return SafetyState(
            joint_currents_ma=tuple(frame.joint_currents_ma),
            state_age_s=age,
        )

    def _record_violations(self, violations: Sequence[SafetyViolation]) -> None:
        """记录安全违规（超出上限后停止追加并计数，不静默丢弃）。"""
        for violation in violations:
            if len(self._violations) >= self._config.max_violations:
                self._violations_dropped += 1
                continue
            self._violations.append(violation)

    def _worst_reason(self, violations: Sequence[SafetyViolation]) -> str:
        """从违规列表里挑出**最严重**的稳定 reason 标签。"""
        for violation in violations:
            if violation.action == ACTION_STOP:
                return violation.reason
        for violation in violations:
            if violation.action == ACTION_SLOW_STOP:
                return violation.reason
        return REASON_STATE_UNAVAILABLE

    def _enter_safe_state(self, action: str, reason: str) -> None:
        """进入安全态：``stop`` ⇒ 急停；``slow_stop`` ⇒ 缓停。

        急停 / 缓停调用本身失败（断链）只记录，不抛出。
        """
        try:
            if action == ACTION_STOP:
                self.transport.emergency_stop()
            else:
                self.transport.slow_stop()
        except TransportError as exc:
            self._errors.append(
                {"reason": exc.reason, "message": str(exc), "phase": "safe_stop"}
            )

    def _record_tick(self) -> None:
        """记录本拍抖动（首拍没有「上一拍」⇒ 不产生样本）。"""
        now_ns = self._clock.now_ns()
        if self._prev_tick_ns is not None:
            interval_s = (now_ns - self._prev_tick_ns) / 1e9
            self._jitter.add(abs(interval_s - self._config.period_s))
        self._prev_tick_ns = now_ns

    def _pace(self) -> None:
        """对齐到理想节拍栅格（绝对 deadline，不累积漂移）。"""
        if not self._config.pace:
            return
        period_ns = int(round(self._config.period_s * 1e9))
        self._deadline_ns += period_ns
        remaining_ns = self._deadline_ns - self._clock.now_ns()
        if remaining_ns > 0:
            self._clock.sleep(remaining_ns / 1e9)
