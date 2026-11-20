"""故障恢复流程与五类故障注入回归（S03-T05）。

为什么是「流程 + 替身」
----------------------
研究 F1：真机在当前环境**不可达**（两台开发机都 ping 不到机械臂），
物理故障无法在离线环境制造；但**传输层边界的替身可以精确注入**
（:class:`~rm75_drake.device.transport.ScriptedTransport`）：

======================  ====================================================
故障（研究 F1）          替身上的注入方式
======================  ====================================================
拔网线                    ``drop_link()``（读不到状态帧 + 下发失败）
物体掉落                  ``set_gripper_offline(True)``（夹爪反馈丢失）
夹爪夹空                  夹爪位置回读值 ≠ 目标位置
关节超限位                逐关节错误码非 0 / 位置越软限位
供电跌落                  ``set_system_error(code)`` + ``push_frame()``
======================  ====================================================

供电跌落的语义载体
------------------
状态帧只有一个帧级系统错误码字段 ``err_code``（``-3`` = 推送数据不完整，
其余非 0 = 系统错误码）⇒ 「供电跌落」由**帧级系统错误码**承载。
**如实标注**：厂商错误码 ↔ 供电跌落的对应关系在硬件可达前**未核实**；
``power_supply_drop`` 是本项目自定的**稳定 reason 标签**，不代表厂商错误码表。
帧级系统错误码的判定统一走
:func:`rm75_drake.device.state_machine.system_error_code`（与状态机同源）。

恢复流程的**官方顺序**（研究 F6 的符号清单，全部在白名单内）::

    清错 → 放宽限位 → 上使能 → 回位 → 恢复限位 → 重同步 q → 复位 READY

其中「清错」在传输层边界上是 :meth:`ArmTransport.clear_error`
（内部依次调用 ``rm_set_joint_clear_err`` → ``rm_clear_system_err``）。

三条硬纪律
----------
1. **每一步都走** :class:`~rm75_drake.device.transport.ArmTransport`
   ⇒ 替身可测、真机可执行，流程只有**一份**实现。
2. **步骤失败必须给出下一步可执行的诊断**（哪个符号失败、当前错误码、
   建议动作）⇒ 失败信息非空，绝不静默跳过。
3. **安全态先于恢复**：故障 → 进入安全态（FAULT / ESTOP / HOLDING），
   恢复成功后回到 ``READY`` 且 ``q`` 已与状态源**重新同步**。

单位纪律
--------
关节角 **rad**、电流 **mA**、时间 **s**。
本模块**不实现** rad↔deg 换算（唯一转换点在 :mod:`rm75_drake.device.sdk`）。
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

from ..planning.trajectory import JOINT_COUNT
from .state_machine import REASON_ESTOP_STATUS as _REASON_ESTOP_STATUS
from .state_machine import REASON_JOINT_ERROR as _REASON_JOINT_ERROR
from .state_machine import (
    DeviceState,
    DeviceStateMachine,
    StateMachineError,
    system_error_code,
)
from .transport import REASON_GRIPPER_OFFLINE as _REASON_GRIPPER_OFFLINE
from .transport import REASON_SDK_REJECTED as _REASON_SDK_REJECTED
from .transport import REASON_LINK_DOWN as _REASON_LINK_DOWN
from .transport import ArmTransport, StateFrame, TransportError

__all__ = [
    # 稳定 reason 标签
    "REASON_LINK_DOWN",
    "REASON_GRIPPER_OFFLINE",
    "REASON_GRIPPER_NOT_AT_TARGET",
    "REASON_JOINT_ERROR",
    "REASON_ESTOP_STATUS",
    "REASON_POWER_SUPPLY_DROP",
    "REASON_RECOVERY_FAILED",
    "REASON_RECOVERY_OK",
    "FAULT_REASONS",
    "SAFE_STATE_BY_REASON",
    # 常量
    "RELAX_MARGIN_RAD",
    "RECOVERY_STEPS",
    "STEP_OK",
    "STEP_WARNING",
    "STEP_FAILED",
    "STEP_SKIPPED",
    # 数据结构
    "FaultDiagnosis",
    "RecoveryStep",
    "RecoveryError",
    "RecoveryResult",
    # 流程
    "RecoveryProcedure",
    "classify_fault",
    "enter_safe_state",
    "recover",
]


# ── 稳定 reason 标签 ────────────────────────────────────────────────────
#: 拔网线（链路不可达）。
REASON_LINK_DOWN = _REASON_LINK_DOWN
#: 物体掉落 / 夹爪反馈丢失（夹爪离线）。
REASON_GRIPPER_OFFLINE = _REASON_GRIPPER_OFFLINE
#: 夹爪夹空（位置回读未到位）。
REASON_GRIPPER_NOT_AT_TARGET = "gripper_not_at_target"
#: 关节超限位 / 逐关节错误码非 0。
REASON_JOINT_ERROR = _REASON_JOINT_ERROR
#: 急停状态（``arm_current_status == 9``）。
REASON_ESTOP_STATUS = _REASON_ESTOP_STATUS
#: 控制器拒绝请求（返回码非 0）。
REASON_SDK_REJECTED = _REASON_SDK_REJECTED
#: 供电跌落 / 欠压（由**帧级系统错误码**承载）。
#:
#: 说明：厂商帧级系统错误码 ↔ 供电跌落的对应关系在硬件可达前**未核实**；
#: 本标签是本项目自定的**稳定 reason**，不代表厂商错误码表。
REASON_POWER_SUPPLY_DROP = "power_supply_drop"
#: 恢复流程失败。
REASON_RECOVERY_FAILED = "recovery_failed"
#: 恢复流程成功。
REASON_RECOVERY_OK = "recovery_ok"

#: 五类故障 + 急停的稳定 reason 标签（顺序即研究 F1 的故障清单）。
FAULT_REASONS: Tuple[str, ...] = (
    REASON_LINK_DOWN,
    REASON_GRIPPER_OFFLINE,
    REASON_GRIPPER_NOT_AT_TARGET,
    REASON_JOINT_ERROR,
    REASON_POWER_SUPPLY_DROP,
    REASON_ESTOP_STATUS,
)

#: 各故障对应的**安全态**（FAULT / ESTOP / HOLDING）。
SAFE_STATE_BY_REASON: Dict[str, DeviceState] = {
    REASON_LINK_DOWN: DeviceState.FAULT,
    REASON_JOINT_ERROR: DeviceState.FAULT,
    REASON_POWER_SUPPLY_DROP: DeviceState.FAULT,
    REASON_ESTOP_STATUS: DeviceState.ESTOP,
    REASON_GRIPPER_OFFLINE: DeviceState.HOLDING,
    REASON_GRIPPER_NOT_AT_TARGET: DeviceState.HOLDING,
}

#: 放宽限位时每侧额外放宽的余量（rad，10°）——限位必须比软件软限位更宽，
#: 否则「放宽限位」这一步没有意义。
RELAX_MARGIN_RAD = 0.1745329

#: 步骤状态标签。
STEP_OK = "ok"
#: 步骤**有告警但未失败**（真机实测：控制器在停止/示教模式下**拒绝**写限位，
#: 返回 1；此时软件软限位闸（G1）仍然生效 ⇒ 恢复流程不该被它卡死，
#: 但也**不得**把它写成「成功」）。
STEP_WARNING = "warning"
STEP_FAILED = "failed"
STEP_SKIPPED = "skipped"


# ── 数据结构 ────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class FaultDiagnosis:
    """一次故障判定结果。

    字段
    ----
    reason:
        稳定 reason 标签（见 :data:`FAULT_REASONS`）。
    safe_state:
        该故障应进入的**安全态**（FAULT / ESTOP / HOLDING）。
    detail:
        人类可读描述（写进运行日志）。
    joint_err_codes:
        判定时刻的逐关节错误码。
    action:
        ``stop``（停发 / 缓停）或 ``hold``（保持位置，停止下发）。
    """

    reason: str
    safe_state: DeviceState
    detail: str
    joint_err_codes: Tuple[int, ...] = ()
    action: str = "stop"

    def to_dict(self) -> Dict[str, Any]:
        """返回可 JSON 序列化的字典。"""
        return {
            "reason": self.reason,
            "safe_state": self.safe_state.value,
            "detail": self.detail,
            "joint_err_codes": list(self.joint_err_codes),
            "action": self.action,
        }


@dataclass(frozen=True)
class RecoveryStep:
    """恢复流程的一步（**每一步都留痕**）。

    字段
    ----
    name:
        步骤名（稳定标签，见 :data:`RECOVERY_STEPS`）。
    symbols:
        该步骤涉及的 SDK 符号（诊断用；空元组 = 无直接 SDK 调用）。
    status:
        ``ok`` / ``failed`` / ``skipped``。
    detail:
        人类可读说明。
    """

    name: str
    symbols: Tuple[str, ...] = ()
    status: str = STEP_OK
    detail: str = ""

    def to_dict(self) -> Dict[str, Any]:
        """返回可 JSON 序列化的字典。"""
        return {
            "name": self.name,
            "symbols": list(self.symbols),
            "status": self.status,
            "detail": self.detail,
        }


#: 恢复流程的步骤顺序（**官方顺序**，测试会断言）。
RECOVERY_STEPS: Tuple[str, ...] = (
    "verify_link",
    "check_gripper",
    "clear_error",
    "relax_limits",
    "enable",
    "homing",
    "restore_limits",
    "resync_q",
    "reset_ready",
)


class RecoveryError(RuntimeError):
    """恢复流程失败（**带可执行诊断**，绝不静默跳过）。

    属性
    ----
    step:
        失败步骤名。
    symbols:
        涉及的 SDK 符号。
    cause:
        失败原因（异常文本）。
    error_codes:
        失败时刻的逐关节错误码。
    suggestion:
        **下一步可执行**的建议动作。
    """

    def __init__(
        self,
        message: str,
        *,
        step: str,
        symbols: Sequence[str] = (),
        cause: str = "",
        error_codes: Sequence[int] = (),
        suggestion: str = "",
    ) -> None:
        super().__init__(message)
        self.step = step
        self.symbols = tuple(symbols)
        self.cause = cause
        self.error_codes = tuple(int(c) for c in error_codes)
        self.suggestion = suggestion

    def diagnosis(self) -> str:
        """返回完整的诊断文本（失败符号 / 错误码 / 建议动作）。"""
        symbols = (
            "、".join(self.symbols) if self.symbols else "（无直接 SDK 调用）"
        )
        return (
            f"恢复流程在步骤「{self.step}」失败"
            f"；涉及 SDK 符号：{symbols}"
            f"；失败原因：{self.cause or '未知'}"
            f"；当前逐关节错误码：{list(self.error_codes)}"
            f"；建议动作：{self.suggestion or '检查设备状态后重新执行恢复流程'}"
        )


@dataclass(frozen=True)
class RecoveryResult:
    """恢复流程的执行结果（**证据**）。

    字段
    ----
    ok:
        是否恢复成功（回到 ``READY`` 且 ``q`` 已重同步）。
    steps:
        逐步留痕（含失败后被跳过的步骤）。
    failed_step:
        失败步骤名（成功时为 ``None``）。
    reason:
        稳定 reason 标签（``recovery_ok`` / ``recovery_failed``）。
    diagnosis:
        失败诊断文本（成功时为空串）。
    resynced_q_rad:
        重同步后的关节角（rad，7 维；未重同步时为 ``None``）。
    final_state:
        结束时状态机的状态标签。
    """

    ok: bool
    steps: Tuple[RecoveryStep, ...] = ()
    failed_step: Optional[str] = None
    reason: str = REASON_RECOVERY_OK
    diagnosis: str = ""
    resynced_q_rad: Optional[Tuple[float, ...]] = None
    final_state: str = ""

    def step(self, name: str) -> Optional[RecoveryStep]:
        """按名字返回某一步（找不到返回 ``None``）。"""
        for item in self.steps:
            if item.name == name:
                return item
        return None

    def to_dict(self) -> Dict[str, Any]:
        """返回可 JSON 序列化的字典（写进运行日志）。"""
        return {
            "ok": self.ok,
            "reason": self.reason,
            "failed_step": self.failed_step,
            "diagnosis": self.diagnosis,
            "final_state": self.final_state,
            "resynced_q_rad": (
                list(self.resynced_q_rad) if self.resynced_q_rad is not None else None
            ),
            "steps": [s.to_dict() for s in self.steps],
        }


# ── 故障判定 ────────────────────────────────────────────────────────────
def classify_fault(
    transport: ArmTransport,
    *,
    gripper_target: Optional[int] = None,
) -> Optional[FaultDiagnosis]:
    """判定当前是否处于故障态（**纯读**，不改任何状态）。

    判定优先级（强 → 弱）

    1. 读不到状态帧 ⇒ ``link_down``（拔网线）；
    2. ``arm_current_status == 9`` ⇒ ``estop_status``（急停）；
    3. 帧级系统错误码非 0（非 ``-3``）⇒ ``power_supply_drop``（供电跌落）；
    4. 逐关节错误码非 0 ⇒ ``joint_error``（关节超限位 / 报错）；
    5. 夹爪离线 ⇒ ``gripper_offline``（物体掉落 / 反馈丢失）；
    6. 夹爪位置回读 ≠ 目标 ⇒ ``gripper_not_at_target``（夹爪夹空）。

    判定**同源**
    ------------
    帧级系统错误码的判定走
    :func:`rm75_drake.device.state_machine.system_error_code`（与状态机的
    :func:`~rm75_drake.device.state_machine.detect_fault` **同一个函数**）
    ⇒ 两条链路对同一条信号绝不会给出不同结论。

    返回
    ----
    ``None``
        无故障；
    否则返回 :class:`FaultDiagnosis`（含**安全态**与建议动作）。
    """
    frame = transport.read_state()
    if frame is None:
        return FaultDiagnosis(
            reason=REASON_LINK_DOWN,
            safe_state=SAFE_STATE_BY_REASON[REASON_LINK_DOWN],
            detail="读不到状态帧（链路不可达，疑似拔网线或控制器掉线）",
            action="stop",
        )

    codes = tuple(int(c) for c in frame.joint_err_codes)
    if int(frame.arm_current_status) == 9:
        return FaultDiagnosis(
            reason=REASON_ESTOP_STATUS,
            safe_state=SAFE_STATE_BY_REASON[REASON_ESTOP_STATUS],
            detail="机械臂处于急停状态（arm_current_status == 9）",
            joint_err_codes=codes,
            action="stop",
        )
    system_code = system_error_code(frame)
    if system_code is not None:
        return FaultDiagnosis(
            reason=REASON_POWER_SUPPLY_DROP,
            safe_state=SAFE_STATE_BY_REASON[REASON_POWER_SUPPLY_DROP],
            detail=(
                f"帧级系统错误码非 0（errCode={system_code}），"
                "判定为供电跌落 / 欠压；"
                "注意：厂商帧级系统错误码与供电跌落的对应关系在硬件可达前未核实"
                "（本 reason 为本项目自定的稳定标签，不代表厂商错误码表）"
            ),
            joint_err_codes=codes,
            action="stop",
        )
    if any(code != 0 for code in codes):
        return FaultDiagnosis(
            reason=REASON_JOINT_ERROR,
            safe_state=SAFE_STATE_BY_REASON[REASON_JOINT_ERROR],
            detail=(
                "逐关节错误码非 0："
                + "、".join(
                    f"joint_{i + 1}={code}" for i, code in enumerate(codes) if code != 0
                )
            ),
            joint_err_codes=codes,
            action="stop",
        )

    gripper = transport.read_gripper()
    if not gripper.online:
        return FaultDiagnosis(
            reason=REASON_GRIPPER_OFFLINE,
            safe_state=SAFE_STATE_BY_REASON[REASON_GRIPPER_OFFLINE],
            detail="夹爪离线（rm_plus_state != 0，疑似物体掉落或夹爪反馈丢失）",
            joint_err_codes=codes,
            action="hold",
        )
    if gripper_target is not None and int(gripper.position) != int(gripper_target):
        return FaultDiagnosis(
            reason=REASON_GRIPPER_NOT_AT_TARGET,
            safe_state=SAFE_STATE_BY_REASON[REASON_GRIPPER_NOT_AT_TARGET],
            detail=(
                f"夹爪未到位（目标 {int(gripper_target)}，"
                f"回读 {int(gripper.position)}，疑似夹爪夹空）"
            ),
            joint_err_codes=codes,
            action="hold",
        )
    return None


def enter_safe_state(
    transport: ArmTransport,
    machine: DeviceStateMachine,
    diagnosis: FaultDiagnosis,
) -> DeviceState:
    """把设备带入**安全态**（停发 / 缓停 / 急停 + 状态机转移）。

    返回
    ----
    进入后的状态。
    """
    if diagnosis.action == "stop":
        if diagnosis.reason == REASON_ESTOP_STATUS:
            transport.emergency_stop()
        else:
            transport.slow_stop()
    else:
        # hold 类故障：停止下发，但保持当前姿态（不缓停）
        pass

    state = machine.state
    target = diagnosis.safe_state
    if target == DeviceState.FAULT:
        if state != DeviceState.FAULT:
            machine.fault(reason=diagnosis.reason, detail=diagnosis.detail)
    elif target == DeviceState.ESTOP:
        if state != DeviceState.ESTOP:
            machine.estop(reason=diagnosis.reason, detail=diagnosis.detail)
    elif target == DeviceState.HOLDING:
        if state == DeviceState.STREAMING:
            machine.hold(reason=diagnosis.reason, detail=diagnosis.detail)
        elif state != DeviceState.HOLDING:
            # 非流动态故障：先停流再保持
            if state == DeviceState.READY:
                machine.start_stream()
                machine.hold(reason=diagnosis.reason, detail=diagnosis.detail)
    return machine.state


# ── 恢复流程 ────────────────────────────────────────────────────────────
class RecoveryProcedure:
    """官方顺序的故障恢复流程（**每一步都走传输层**）。

    参数
    ----
    transport:
        传输层（真机 / 替身）。
    machine:
        设备状态机（恢复结束时复位到 ``READY``）。
    home_position_rad:
        回位目标关节角（rad，7 维）；``None`` ⇒ 全零位。
    soft_lower_rad / soft_upper_rad:
        软件软限位（rad，7 维）——恢复流程末尾要**写回**它们。
    relaxed_lower_rad / relaxed_upper_rad:
        放宽后的限位（rad，7 维）；``None`` ⇒ 由软限位每侧放宽
        :data:`RELAX_MARGIN_RAD` 得到。
    gripper_target:
        期望的夹爪位置（0–1000）；``None`` ⇒ 不检查夹爪到位。
    clock:
        单调时钟（默认 :func:`time.monotonic`）。
    """

    def __init__(
        self,
        transport: ArmTransport,
        machine: DeviceStateMachine,
        *,
        home_position_rad: Optional[Sequence[float]] = None,
        soft_lower_rad: Sequence[float],
        soft_upper_rad: Sequence[float],
        relaxed_lower_rad: Optional[Sequence[float]] = None,
        relaxed_upper_rad: Optional[Sequence[float]] = None,
        gripper_target: Optional[int] = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._transport = transport
        self._machine = machine
        self._clock = clock
        self._soft_lower = _vector(soft_lower_rad, "soft_lower_rad")
        self._soft_upper = _vector(soft_upper_rad, "soft_upper_rad")
        if relaxed_lower_rad is None or relaxed_upper_rad is None:
            lower = tuple(lo - RELAX_MARGIN_RAD for lo in self._soft_lower)
            upper = tuple(hi + RELAX_MARGIN_RAD for hi in self._soft_upper)
        else:
            lower = _vector(relaxed_lower_rad, "relaxed_lower_rad")
            upper = _vector(relaxed_upper_rad, "relaxed_upper_rad")
        self._relaxed_lower = lower
        self._relaxed_upper = upper
        self._home = (
            (0.0,) * JOINT_COUNT
            if home_position_rad is None
            else _vector(home_position_rad, "home_position_rad")
        )
        self._gripper_target = None if gripper_target is None else int(gripper_target)

    # -- 观测 ------------------------------------------------------------
    @property
    def steps(self) -> Tuple[str, ...]:
        """步骤顺序（官方顺序）。"""
        return RECOVERY_STEPS

    # -- 主流程 ----------------------------------------------------------
    def run(self) -> RecoveryResult:
        """执行完整恢复流程。

        返回
        ----
        :class:`RecoveryResult`
            成功 → ``ok=True``、状态机回到 ``READY``、``q`` 已重同步；
            失败 → ``ok=False``，并给出**非空**诊断。
        """
        steps: List[RecoveryStep] = []
        resynced: Optional[Tuple[float, ...]] = None

        try:
            frame = self._step_verify_link(steps)
            self._step_check_gripper(steps)
            self._step_clear_error(steps)
            self._step_relax_limits(steps)
            self._step_enable(steps)
            self._step_homing(steps)
            self._step_restore_limits(steps)
            frame, resynced = self._step_resync_q(steps)
            self._step_reset_ready(steps, frame)
        except RecoveryError as exc:
            steps.append(
                RecoveryStep(
                    name=exc.step,
                    symbols=exc.symbols,
                    status=STEP_FAILED,
                    detail=exc.diagnosis(),
                )
            )
            for name in RECOVERY_STEPS[len(steps) :]:
                steps.append(
                    RecoveryStep(
                        name=name,
                        status=STEP_SKIPPED,
                        detail=f"前置步骤「{exc.step}」失败，本步骤被跳过",
                    )
                )
            return RecoveryResult(
                ok=False,
                steps=tuple(steps),
                failed_step=exc.step,
                reason=REASON_RECOVERY_FAILED,
                diagnosis=exc.diagnosis(),
                resynced_q_rad=resynced,
                final_state=self._machine.state.value,
            )
        except StateMachineError as exc:
            failed_name = steps[-1].name if steps else "reset_ready"
            steps.append(
                RecoveryStep(
                    name=failed_name,
                    status=STEP_FAILED,
                    detail=f"状态机转移失败：{exc}",
                )
            )
            for name in RECOVERY_STEPS[len(steps) :]:
                steps.append(
                    RecoveryStep(
                        name=name,
                        status=STEP_SKIPPED,
                        detail=f"前置步骤「{failed_name}」失败，本步骤被跳过",
                    )
                )
            error = RecoveryError(
                f"状态机转移失败：{exc}",
                step=failed_name,
                symbols=(),
                cause=f"{type(exc).__name__}: {exc}",
                error_codes=self._current_error_codes(),
                suggestion="检查状态机当前状态与恢复流程的前置条件是否匹配（见状态机转移表）",
            )
            return RecoveryResult(
                ok=False,
                steps=tuple(steps),
                failed_step=error.step,
                reason=REASON_RECOVERY_FAILED,
                diagnosis=error.diagnosis(),
                resynced_q_rad=resynced,
                final_state=self._machine.state.value,
            )

        return RecoveryResult(
            ok=True,
            steps=tuple(steps),
            failed_step=None,
            reason=REASON_RECOVERY_OK,
            diagnosis="",
            resynced_q_rad=resynced,
            final_state=self._machine.state.value,
        )

    # -- 各步骤 ----------------------------------------------------------
    def _step_verify_link(self, steps: List[RecoveryStep]) -> StateFrame:
        """步骤 1：链路可达性 + 取一帧状态。"""
        frame = self._transport.read_state()
        if frame is None:
            raise RecoveryError(
                "链路不可达（读不到状态帧）",
                step="verify_link",
                symbols=(),
                cause="transport.read_state() 返回 None",
                error_codes=(),
                suggestion=(
                    "恢复网线并确认机械臂已上电、控制器在线、"
                    "devices/*.yaml 的 arm_ip 与机械臂网段一致；链路恢复后重跑恢复流程"
                ),
            )
        steps.append(
            RecoveryStep(
                name="verify_link",
                status=STEP_OK,
                detail="链路可达，已读到状态帧",
            )
        )
        return frame

    def _step_check_gripper(self, steps: List[RecoveryStep]) -> None:
        """步骤 2：夹爪在线且（若给定目标）已到位。"""
        gripper = self._transport.read_gripper()
        if not gripper.online:
            raise RecoveryError(
                "夹爪离线",
                step="check_gripper",
                symbols=("rm_get_gripper_state",),
                cause="rm_plus_state != 0（夹爪不在线）",
                error_codes=self._current_error_codes(),
                suggestion=(
                    "检查夹爪供电与通信（重新插拔夹爪 / 重启夹爪模块）；"
                    "夹爪在线后重跑恢复流程（重新夹持需由操作员完成）"
                ),
            )
        if (
            self._gripper_target is not None
            and int(gripper.position) != int(self._gripper_target)
        ):
            raise RecoveryError(
                "夹爪未到位",
                step="check_gripper",
                symbols=("rm_get_gripper_state", "rm_set_gripper_position"),
                cause=(
                    f"夹爪位置回读 {int(gripper.position)} ≠ 目标 "
                    f"{int(self._gripper_target)}"
                ),
                error_codes=self._current_error_codes(),
                suggestion=(
                    "重新执行夹取动作（rm_set_gripper_position）并确认未夹空；"
                    "夹爪到位后重跑恢复流程"
                ),
            )
        steps.append(
            RecoveryStep(
                name="check_gripper",
                symbols=("rm_get_gripper_state",),
                status=STEP_OK,
                detail="夹爪在线",
            )
        )

    def _step_clear_error(self, steps: List[RecoveryStep]) -> None:
        """步骤 3：清错（``rm_set_joint_clear_err`` → ``rm_clear_system_err``）。"""
        try:
            self._transport.clear_error()
        except TransportError as exc:
            raise RecoveryError(
                "清除错误码失败",
                step="clear_error",
                symbols=("rm_set_joint_clear_err", "rm_clear_system_err"),
                cause=f"{type(exc).__name__}: {exc}",
                error_codes=self._current_error_codes(),
                suggestion=(
                    "确认机械臂已上电且控制器在线；错误码持续存在时按厂商错误码表处理"
                    "（必要时 rm_clear_system_runtime 清运行时错误）后重试"
                ),
            ) from exc
        steps.append(
            RecoveryStep(
                name="clear_error",
                symbols=("rm_set_joint_clear_err", "rm_clear_system_err"),
                status=STEP_OK,
                detail="逐关节错误码与系统错误码已清除",
            )
        )

    def _step_relax_limits(self, steps: List[RecoveryStep]) -> None:
        """步骤 4：放宽限位（``rm_set_joint_min_pos`` / ``rm_set_joint_max_pos``）。

        真机实测：控制器在**停止/示教模式**下会拒绝写限位（返回 1）⇒
        这一步记为**告警**（warning）并继续——软件软限位闸（G1）仍然生效，
        但**不得**把它写成「成功」。
        """
        try:
            self._transport.set_joint_limits(self._relaxed_lower, self._relaxed_upper)
        except TransportError as exc:
            if getattr(exc, "reason", "") == REASON_SDK_REJECTED:
                steps.append(
                    RecoveryStep(
                        name="relax_limits",
                        symbols=("rm_set_joint_min_pos", "rm_set_joint_max_pos"),
                        status=STEP_WARNING,
                        detail=(
                            "控制器拒绝放宽限位（机械臂可能处于停止/示教模式）；"
                            "已跳过该步，软件软限位闸（G1）仍然生效"
                        ),
                    )
                )
                return
            raise RecoveryError(
                "放宽限位失败",
                step="relax_limits",
                symbols=("rm_set_joint_min_pos", "rm_set_joint_max_pos"),
                cause=f"{type(exc).__name__}: {exc}",
                error_codes=self._current_error_codes(),
                suggestion=(
                    "确认控制器接受限位写入且参数合法（角度已由传输层换算为度）；"
                    "若机械臂处于急停或示教模式，写限位会被拒绝——先退出该模式"
                ),
            ) from exc
        steps.append(
            RecoveryStep(
                name="relax_limits",
                symbols=("rm_set_joint_min_pos", "rm_set_joint_max_pos"),
                status=STEP_OK,
                detail="限位已放宽",
            )
        )

    def _step_enable(self, steps: List[RecoveryStep]) -> None:
        """步骤 5：上使能（``rm_set_joint_en_state``）。"""
        try:
            self._transport.enable(True)
        except TransportError as exc:
            raise RecoveryError(
                "上使能失败",
                step="enable",
                symbols=("rm_set_joint_en_state",),
                cause=f"{type(exc).__name__}: {exc}",
                error_codes=self._current_error_codes(),
                suggestion=(
                    "确认机械臂已上电、未处于急停状态、关节未被物理卡死或存在干涉；"
                    "排除机械问题后重新上使能"
                ),
            ) from exc
        steps.append(
            RecoveryStep(
                name="enable",
                symbols=("rm_set_joint_en_state",),
                status=STEP_OK,
                detail="已下发使能",
            )
        )

    def _step_homing(self, steps: List[RecoveryStep]) -> None:
        """步骤 6：回位运动（唯一运动通道 ``rm_movej_canfd``）。"""
        try:
            self._transport.move_j_canfd(self._home, follow=True)
        except TransportError as exc:
            raise RecoveryError(
                "回位运动失败",
                step="homing",
                symbols=("rm_movej_canfd",),
                cause=f"{type(exc).__name__}: {exc}",
                error_codes=self._current_error_codes(),
                suggestion=(
                    "确认透传通道可用（rm_movej_canfd 是唯一运动通道）、机械臂未处于急停、"
                    "与外部设备无干涉；链路不可达时先恢复网络再重试"
                ),
            ) from exc
        steps.append(
            RecoveryStep(
                name="homing",
                symbols=("rm_movej_canfd",),
                status=STEP_OK,
                detail="已下发回位指令",
            )
        )

    def _step_restore_limits(self, steps: List[RecoveryStep]) -> None:
        """步骤 7：恢复限位（写回软件软限位）。

        同 :meth:`_step_relax_limits`：控制器拒绝时记**告警**并继续
        （软件侧仍按软限位钳位），**不得**写成「成功」。
        """
        try:
            self._transport.set_joint_limits(self._soft_lower, self._soft_upper)
        except TransportError as exc:
            if getattr(exc, "reason", "") == REASON_SDK_REJECTED:
                steps.append(
                    RecoveryStep(
                        name="restore_limits",
                        symbols=("rm_set_joint_min_pos", "rm_set_joint_max_pos"),
                        status=STEP_WARNING,
                        detail=(
                            "控制器拒绝写回软限位（机械臂可能处于停止/示教模式）；"
                            "软件侧仍按软限位钳位"
                        ),
                    )
                )
                return
            raise RecoveryError(
                "恢复限位失败",
                step="restore_limits",
                symbols=("rm_set_joint_min_pos", "rm_set_joint_max_pos"),
                cause=f"{type(exc).__name__}: {exc}",
                error_codes=self._current_error_codes(),
                suggestion=(
                    "写限位失败时不要继续运动；先确认控制器状态与限位参数合法性，"
                    "再重新写入软限位"
                ),
            ) from exc
        steps.append(
            RecoveryStep(
                name="restore_limits",
                symbols=("rm_set_joint_min_pos", "rm_set_joint_max_pos"),
                status=STEP_OK,
                detail="软件软限位已写回",
            )
        )

    def _step_resync_q(
        self, steps: List[RecoveryStep]
    ) -> Tuple[StateFrame, Tuple[float, ...]]:
        """步骤 8：重同步 ``q``（以状态源最新帧为准）。"""
        frame = self._transport.read_state()
        if frame is None:
            raise RecoveryError(
                "状态重同步失败",
                step="resync_q",
                symbols=(),
                cause="重同步时读不到状态帧",
                error_codes=self._current_error_codes(),
                suggestion="等待下一帧完整状态后重试；持续无帧说明链路仍未恢复",
            )
        if not frame.is_complete:
            raise RecoveryError(
                "状态重同步失败",
                step="resync_q",
                symbols=(),
                cause=f"状态帧不完整（errCode={int(frame.err_code)}，-3 表示推送数据不完整）",
                error_codes=tuple(int(c) for c in frame.joint_err_codes),
                suggestion="等待下一帧完整状态（errCode == 0）后重试",
            )
        codes = tuple(int(c) for c in frame.joint_err_codes)
        if any(code != 0 for code in codes):
            raise RecoveryError(
                "状态重同步失败",
                step="resync_q",
                symbols=("rm_set_joint_clear_err", "rm_clear_system_err"),
                cause=f"错误码未清零：{list(codes)}",
                error_codes=codes,
                suggestion="先清错（rm_set_joint_clear_err → rm_clear_system_err）再重同步",
            )
        system_code = system_error_code(frame)
        if system_code is not None:
            raise RecoveryError(
                "状态重同步失败",
                step="resync_q",
                symbols=("rm_set_joint_clear_err", "rm_clear_system_err"),
                cause=(
                    f"帧级系统错误码未清零：errCode={system_code}"
                    "（供电跌落 / 欠压等系统级故障尚未清除）"
                ),
                error_codes=codes,
                suggestion=(
                    "先确认供电已恢复（欠压 / 掉电需先恢复供电），"
                    "再清错（rm_set_joint_clear_err → rm_clear_system_err）后重同步"
                ),
            )
        if not frame.all_enabled:
            raise RecoveryError(
                "状态重同步失败",
                step="resync_q",
                symbols=("rm_set_joint_en_state",),
                cause="并非全部 7 轴都已使能",
                error_codes=codes,
                suggestion="先上使能（rm_set_joint_en_state）并等待全轴使能后再重同步",
            )
        steps.append(
            RecoveryStep(
                name="resync_q",
                status=STEP_OK,
                detail="q 已与状态源重新同步",
            )
        )
        return frame, tuple(float(v) for v in frame.joint_positions_rad)

    def _step_reset_ready(self, steps: List[RecoveryStep], frame: StateFrame) -> None:
        """步骤 9：状态机复位到 ``READY``。"""
        try:
            self._reset_machine(frame)
        except StateMachineError as exc:
            raise RecoveryError(
                "状态机复位失败",
                step="reset_ready",
                symbols=(),
                cause=f"{type(exc).__name__}: {exc}",
                error_codes=self._current_error_codes(),
                suggestion="检查状态机当前状态与恢复流程前置条件是否匹配（见转移表）",
            ) from exc
        steps.append(
            RecoveryStep(
                name="reset_ready",
                status=STEP_OK,
                detail=f"状态机已复位到 {self._machine.state.value}",
            )
        )

    # -- 内部工具 --------------------------------------------------------
    def _reset_machine(self, frame: StateFrame) -> None:
        """把状态机复位到 ``READY``（按当前状态选择路径）。"""
        machine = self._machine
        state = machine.state
        if state == DeviceState.FAULT:
            machine.recover_begin()
            machine.recover_end(frame)
        elif state == DeviceState.RECOVER:
            machine.recover_end(frame)
        elif state == DeviceState.HOLDING:
            machine.stop_stream()
        elif state == DeviceState.ESTOP:
            machine.clear_estop()
            machine.connect()
            machine.enable()
            machine.mark_all_enabled(frame)
        elif state == DeviceState.STREAMING:
            machine.stop_stream()
        elif state == DeviceState.WAIT_ENABLE:
            machine.mark_all_enabled(frame)
        elif state == DeviceState.CONNECTED:
            machine.enable()
            machine.mark_all_enabled(frame)
        elif state == DeviceState.DISCONNECTED:
            machine.connect()
            machine.enable()
            machine.mark_all_enabled(frame)
        # READY：已经是目标态

    def _current_error_codes(self) -> Tuple[int, ...]:
        """返回当前逐关节错误码（读不到时返回空元组）。"""
        try:
            frame = self._transport.read_state()
        except TransportError:
            return ()
        if frame is None:
            return ()
        return tuple(int(c) for c in frame.joint_err_codes)


def _vector(values: Sequence[float], name: str) -> Tuple[float, ...]:
    """校验关节向量：长度 7、元素为有限实数。"""
    if isinstance(values, (str, bytes)) or not isinstance(values, Sequence):
        raise ValueError(f"{name} 必须是 {JOINT_COUNT} 维序列")
    if len(values) != JOINT_COUNT:
        raise ValueError(
            f"{name} 必须是 {JOINT_COUNT} 维序列（实际 {len(values)} 维）"
        )
    out = []
    for i, v in enumerate(values):
        if isinstance(v, bool) or not isinstance(v, (int, float)):
            raise ValueError(f"{name}[{i}] 必须是数值，收到 {v!r}")
        fv = float(v)
        if not math.isfinite(fv):
            raise ValueError(f"{name}[{i}] 必须是有限数值，收到 {v!r}")
        out.append(fv)
    return tuple(out)


def recover(
    transport: ArmTransport,
    machine: DeviceStateMachine,
    **kwargs: Any,
) -> RecoveryResult:
    """便捷入口：构造 :class:`RecoveryProcedure` 并执行。

    参数
    ----
    transport:
        传输层。
    machine:
        设备状态机。
    **kwargs:
        透传给 :class:`RecoveryProcedure`（软限位等）。
    """
    procedure = RecoveryProcedure(transport, machine, **kwargs)
    return procedure.run()
