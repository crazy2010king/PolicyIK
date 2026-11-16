"""故障恢复流程与五类故障注入回归（S03-T05）。

零硬件：全部用 :class:`~rm75_drake.device.transport.ScriptedTransport` 替身 +
:class:`~rm75_drake.device.state_machine.DeviceStateMachine` 纯状态机。

覆盖
----
* 五类故障（拔网线 / 物体掉落 / 夹爪夹空 / 关节超限位 / 供电跌落）各一条
  「注入 → 安全态 → 恢复成功 → 回到 READY 且 q 已重同步」；
* 恢复流程的**官方顺序**（清错 → 放宽限位 → 上使能 → 回位 → 恢复限位 →
  重同步 q → 复位 READY）；
* 失败路径：诊断信息**非空**（失败符号 / 错误码 / 建议动作），且后续步骤
  被显式标记为 skipped（绝不静默跳过）。
"""

from __future__ import annotations

from dataclasses import replace as _replace

import pytest

from rm75_drake.device.recovery import (
    FAULT_REASONS,
    RECOVERY_STEPS,
    RELAX_MARGIN_RAD,
    SAFE_STATE_BY_REASON,
    STEP_FAILED,
    STEP_OK,
    STEP_SKIPPED,
    FaultDiagnosis,
    RecoveryError,
    RecoveryProcedure,
    classify_fault,
    enter_safe_state,
    recover,
)
from rm75_drake.device.state_machine import DeviceState, DeviceStateMachine
from rm75_drake.device.transport import (
    ScriptedTransport,
    StateFrame,
    TransportError,
)

DOF = 7

#: 测试用的软件软限位（rad）。
SOFT_LOWER = tuple([-2.0] * DOF)
SOFT_UPPER = tuple([2.0] * DOF)


# ── 辅助 ────────────────────────────────────────────────────────────────
def _frame(
    positions=None,
    err_codes=None,
    enabled: bool = True,
    arm_status: int = 0,
    err_code: int = 0,
    received_at: float = 0.0,
) -> StateFrame:
    """构造一帧状态（默认：全零位、无错、全使能）。"""
    return StateFrame(
        joint_positions_rad=tuple(positions or [0.0] * DOF),
        joint_currents_ma=(0.0,) * DOF,
        joint_en_flags=(enabled,) * DOF,
        joint_err_codes=tuple(err_codes or [0] * DOF),
        controller_status=0,
        gripper_online=True,
        arm_current_status=arm_status,
        received_at=received_at,
        err_code=err_code,
    )


def _streaming_machine(frame: StateFrame) -> DeviceStateMachine:
    """构造一台已连上、已使能、正在透传的状态机。"""
    machine = DeviceStateMachine(clock=lambda: 1.0)
    machine.connect()
    machine.enable()
    machine.mark_all_enabled(frame)
    machine.start_stream()
    return machine


def _procedure(transport, machine, **kwargs) -> RecoveryProcedure:
    """构造恢复流程（软限位用测试常量）。"""
    kwargs.setdefault("soft_lower_rad", SOFT_LOWER)
    kwargs.setdefault("soft_upper_rad", SOFT_UPPER)
    return RecoveryProcedure(transport, machine, **kwargs)


class _FailingTransport(ScriptedTransport):
    """在指定动作上抛 :class:`TransportError` 的替身（注入失败路径）。"""

    def __init__(self, fail_on: str, frames=None) -> None:
        super().__init__(frames)
        self.fail_on = fail_on

    def _guard(self, name: str) -> None:
        if self.fail_on == name:
            raise TransportError(f"注入的传输层故障：{name}", reason="link_down")

    def clear_error(self) -> None:
        self._guard("clear_error")
        super().clear_error()

    def enable(self, state: bool) -> None:
        self._guard("enable")
        super().enable(state)

    def move_j_canfd(self, joint_rad, *, follow: bool = True) -> None:
        self._guard("move_j_canfd")
        super().move_j_canfd(joint_rad, follow=follow)

    def set_joint_limits(self, lower_rad, upper_rad) -> None:
        self._guard("set_joint_limits")
        super().set_joint_limits(lower_rad, upper_rad)


# ── 官方顺序 ────────────────────────────────────────────────────────────
def test_recovery_steps_are_in_official_order():
    """恢复步骤顺序必须是官方顺序（研究 F6）。"""
    assert RECOVERY_STEPS == (
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


def test_recovery_calls_transport_in_official_order():
    """清错 → 放宽限位 → 上使能 → 回位 → 恢复限位 的传输层调用顺序。"""
    transport = ScriptedTransport()
    transport.open()
    transport.push_frame(_frame())
    machine = _streaming_machine(_frame())

    result = _procedure(transport, machine).run()

    assert result.ok
    names = [
        name
        for name in transport.call_names()
        if name in ("clear_error", "set_joint_limits", "enable", "move_j_canfd")
    ]
    assert names == [
        "clear_error",
        "set_joint_limits",  # 放宽限位
        "enable",
        "move_j_canfd",  # 回位
        "set_joint_limits",  # 恢复限位
    ]


def test_relaxed_limits_are_wider_than_soft_limits():
    """放宽限位必须比软件软限位更宽，否则这一步没有意义。"""
    transport = ScriptedTransport()
    transport.open()
    transport.push_frame(_frame())
    machine = _streaming_machine(_frame())

    _procedure(transport, machine).run()

    limit_calls = [
        args for name, args in transport.calls if name == "set_joint_limits"
    ]
    relaxed_lower, relaxed_upper = limit_calls[0]
    assert relaxed_lower[0] == pytest.approx(SOFT_LOWER[0] - RELAX_MARGIN_RAD)
    assert relaxed_upper[0] == pytest.approx(SOFT_UPPER[0] + RELAX_MARGIN_RAD)
    # 恢复限位写回软限位
    assert limit_calls[1][0][0] == pytest.approx(SOFT_LOWER[0])
    assert limit_calls[1][1][0] == pytest.approx(SOFT_UPPER[0])


# ── 故障注入 → 安全态 → 恢复成功 ───────────────────────────────────────
def test_fault_link_down_enters_fault_and_recovers():
    """拔网线：注入 → FAULT → 恢复 → READY 且 q 已重同步。"""
    transport = ScriptedTransport()
    transport.open()
    transport.push_frame(_frame(positions=[0.1] * DOF))
    machine = _streaming_machine(_frame())

    transport.drop_link()

    diagnosis = classify_fault(transport)
    assert diagnosis is not None
    assert diagnosis.reason == "link_down"
    assert enter_safe_state(transport, machine, diagnosis) == DeviceState.FAULT

    # 操作员恢复链路 + 控制器回帧（清错、全轴使能）
    transport.restore_link()
    transport.push_frame(_frame(positions=[0.2] * DOF))

    result = _procedure(transport, machine).run()

    assert result.ok, result.diagnosis
    assert result.failed_step is None
    assert machine.state == DeviceState.READY
    assert result.resynced_q_rad == pytest.approx((0.2,) * DOF)


def test_fault_object_dropped_enters_holding_and_recovers():
    """物体掉落（夹爪离线）：注入 → HOLDING → 恢复 → READY。"""
    transport = ScriptedTransport()
    transport.open()
    transport.push_frame(_frame())
    machine = _streaming_machine(_frame())

    transport.set_gripper_offline(True)

    diagnosis = classify_fault(transport)
    assert diagnosis is not None
    assert diagnosis.reason == "gripper_offline"
    assert diagnosis.action == "hold"
    assert enter_safe_state(transport, machine, diagnosis) == DeviceState.HOLDING

    # 操作员把夹爪接回
    transport.set_gripper_offline(False)
    transport.push_frame(_frame(positions=[0.3] * DOF))

    result = _procedure(transport, machine).run()

    assert result.ok, result.diagnosis
    assert machine.state == DeviceState.READY
    assert result.resynced_q_rad == pytest.approx((0.3,) * DOF)


def test_fault_gripper_empty_enters_holding_and_recovers():
    """夹爪夹空（位置未到位）：注入 → HOLDING → 恢复 → READY。"""
    transport = ScriptedTransport()
    transport.open()
    transport.push_frame(_frame())
    machine = _streaming_machine(_frame())

    transport.set_gripper_position(120)  # 目标 500，实际没夹到

    diagnosis = classify_fault(transport, gripper_target=500)
    assert diagnosis is not None
    assert diagnosis.reason == "gripper_not_at_target"
    assert enter_safe_state(transport, machine, diagnosis) == DeviceState.HOLDING

    # 重新夹取到位
    transport.set_gripper_position(500)
    transport.push_frame(_frame(positions=[0.4] * DOF))

    result = _procedure(transport, machine, gripper_target=500).run()

    assert result.ok, result.diagnosis
    assert machine.state == DeviceState.READY
    assert result.resynced_q_rad == pytest.approx((0.4,) * DOF)


def test_fault_joint_overlimit_enters_fault_and_recovers():
    """关节超限位（逐关节错误码非 0）：注入 → FAULT → 恢复 → READY。"""
    transport = ScriptedTransport()
    transport.open()
    transport.push_frame(_frame())
    machine = _streaming_machine(_frame())

    transport.push_frame(_frame(err_codes=[7, 0, 0, 0, 0, 0, 0]))

    diagnosis = classify_fault(transport)
    assert diagnosis is not None
    assert diagnosis.reason == "joint_error"
    assert diagnosis.joint_err_codes == (7, 0, 0, 0, 0, 0, 0)
    assert enter_safe_state(transport, machine, diagnosis) == DeviceState.FAULT

    # 控制器清错后回帧（全轴使能、无错）
    transport.push_frame(_frame(positions=[0.5] * DOF))

    result = _procedure(transport, machine).run()

    assert result.ok, result.diagnosis
    assert machine.state == DeviceState.READY
    assert result.resynced_q_rad == pytest.approx((0.5,) * DOF)


def test_estop_state_recovers_through_reconnect():
    """急停态：恢复流程必须走「清急停 → 重连 → 使能 → READY」。"""
    transport = ScriptedTransport()
    transport.open()
    transport.push_frame(_frame())
    machine = _streaming_machine(_frame())

    transport.push_frame(_frame(arm_status=9))

    diagnosis = classify_fault(transport)
    assert diagnosis is not None
    assert diagnosis.reason == "estop_status"
    assert enter_safe_state(transport, machine, diagnosis) == DeviceState.ESTOP

    transport.push_frame(_frame(positions=[0.6] * DOF))
    result = _procedure(transport, machine).run()

    assert result.ok, result.diagnosis
    assert machine.state == DeviceState.READY


# ── 失败路径：诊断必须非空 ──────────────────────────────────────────────
def test_recovery_fails_with_diagnosis_when_link_still_down():
    """链路仍未恢复：失败在 verify_link，诊断非空，后续步骤显式 skipped。"""
    transport = ScriptedTransport()
    transport.open()
    transport.push_frame(_frame())
    machine = _streaming_machine(_frame())
    transport.drop_link()

    result = _procedure(transport, machine).run()

    assert not result.ok
    assert result.failed_step == "verify_link"
    assert result.reason == "recovery_failed"
    assert result.diagnosis
    assert "建议动作" in result.diagnosis
    assert result.step("verify_link").status == STEP_FAILED
    assert result.step("clear_error").status == STEP_SKIPPED
    assert result.resynced_q_rad is None


def test_recovery_fails_with_diagnosis_when_gripper_offline():
    """夹爪仍离线：失败在 check_gripper，诊断点名 SDK 符号。"""
    transport = ScriptedTransport()
    transport.open()
    transport.push_frame(_frame())
    machine = _streaming_machine(_frame())
    transport.set_gripper_offline(True)

    result = _procedure(transport, machine).run()

    assert not result.ok
    assert result.failed_step == "check_gripper"
    assert "rm_get_gripper_state" in result.diagnosis
    assert "建议动作" in result.diagnosis
    assert result.step("enable").status == STEP_SKIPPED


def test_recovery_fails_with_diagnosis_when_homing_fails():
    """回位下发失败：诊断点名唯一运动通道 rm_movej_canfd。"""
    transport = _FailingTransport("move_j_canfd")
    transport.open()
    transport.push_frame(_frame())
    machine = _streaming_machine(_frame())

    result = _procedure(transport, machine).run()

    assert not result.ok
    assert result.failed_step == "homing"
    assert "rm_movej_canfd" in result.diagnosis
    assert "建议动作" in result.diagnosis
    assert result.step("restore_limits").status == STEP_SKIPPED


def test_recovery_fails_with_diagnosis_when_clear_error_fails():
    """清错失败：诊断必须包含错误码与建议动作。"""
    transport = _FailingTransport("clear_error")
    transport.open()
    transport.push_frame(_frame())
    machine = _streaming_machine(_frame())

    result = _procedure(transport, machine).run()

    assert not result.ok
    assert result.failed_step == "clear_error"
    assert "rm_set_joint_clear_err" in result.diagnosis
    assert "逐关节错误码" in result.diagnosis


def test_resync_rejects_incomplete_frame():
    """重同步必须拒绝不完整帧（errCode == -3）。"""
    transport = ScriptedTransport()
    transport.open()
    transport.push_frame(_frame())
    machine = _streaming_machine(_frame())
    transport.push_frame(_frame(err_code=-3))

    result = _procedure(transport, machine).run()

    assert not result.ok
    assert result.failed_step == "resync_q"
    assert "errCode=-3" in result.diagnosis or "-3" in result.diagnosis
    assert result.step("reset_ready").status == STEP_SKIPPED


def test_resync_rejects_not_all_enabled():
    """重同步必须拒绝「未全部使能」的帧。"""
    transport = ScriptedTransport()
    transport.open()
    transport.push_frame(_frame())
    machine = _streaming_machine(_frame())
    transport.push_frame(_frame(enabled=False))

    result = _procedure(transport, machine).run()

    assert not result.ok
    assert result.failed_step == "resync_q"
    assert "rm_set_joint_en_state" in result.diagnosis


def test_resync_rejects_remaining_error_codes():
    """重同步必须拒绝「错误码未清零」的帧。"""
    transport = ScriptedTransport()
    transport.open()
    transport.push_frame(_frame())
    machine = _streaming_machine(_frame())
    transport.push_frame(_frame(err_codes=[3, 0, 0, 0, 0, 0, 0]))

    result = _procedure(transport, machine).run()

    assert not result.ok
    assert result.failed_step == "resync_q"
    assert "rm_set_joint_clear_err" in result.diagnosis


# ── 判定与诊断的基本性质 ────────────────────────────────────────────────
def test_classify_fault_returns_none_when_healthy():
    """健康状态不应被判成故障。"""
    transport = ScriptedTransport()
    transport.open()
    transport.push_frame(_frame())
    assert classify_fault(transport) is None


def test_fault_reasons_cover_five_injected_faults():
    """五类故障的 reason 标签必须稳定且齐备。"""
    assert "link_down" in FAULT_REASONS
    assert "gripper_offline" in FAULT_REASONS
    assert "gripper_not_at_target" in FAULT_REASONS
    assert "joint_error" in FAULT_REASONS
    assert "power_supply_drop" in FAULT_REASONS
    assert SAFE_STATE_BY_REASON["link_down"] == DeviceState.FAULT
    assert SAFE_STATE_BY_REASON["joint_error"] == DeviceState.FAULT
    assert SAFE_STATE_BY_REASON["power_supply_drop"] == DeviceState.FAULT
    assert SAFE_STATE_BY_REASON["gripper_offline"] == DeviceState.HOLDING
    assert SAFE_STATE_BY_REASON["gripper_not_at_target"] == DeviceState.HOLDING


def test_recovery_error_diagnosis_contains_symbols_codes_and_action():
    """诊断文本必须含：失败符号、当前错误码、建议动作。"""
    error = RecoveryError(
        "boom",
        step="clear_error",
        symbols=("rm_set_joint_clear_err",),
        cause="TransportError: boom",
        error_codes=[1, 0, 0, 0, 0, 0, 0],
        suggestion="先清错",
    )
    text = error.diagnosis()
    assert "rm_set_joint_clear_err" in text
    assert "[1, 0, 0, 0, 0, 0, 0]" in text
    assert "先清错" in text


def test_enter_safe_state_is_idempotent():
    """重复进入安全态不应抛异常（状态机不允许二次 FAULT 转移）。"""
    transport = ScriptedTransport()
    transport.open()
    transport.push_frame(_frame())
    machine = _streaming_machine(_frame())
    diagnosis = FaultDiagnosis(
        reason="joint_error",
        safe_state=DeviceState.FAULT,
        detail="测试",
    )
    assert enter_safe_state(transport, machine, diagnosis) == DeviceState.FAULT
    assert enter_safe_state(transport, machine, diagnosis) == DeviceState.FAULT


def test_recover_convenience_entry_matches_procedure():
    """便捷入口与显式流程结果一致。"""
    transport = ScriptedTransport()
    transport.open()
    transport.push_frame(_frame())
    machine = _streaming_machine(_frame())

    result = recover(
        transport,
        machine,
        soft_lower_rad=SOFT_LOWER,
        soft_upper_rad=SOFT_UPPER,
    )
    assert result.ok
    assert machine.state == DeviceState.READY
    assert all(s.status == STEP_OK for s in result.steps)


def test_recovery_result_is_json_serialisable():
    """恢复结果必须可 JSON 序列化（写进运行日志）。"""
    import json

    transport = ScriptedTransport()
    transport.open()
    transport.push_frame(_frame())
    machine = _streaming_machine(_frame())
    result = _procedure(transport, machine).run()
    payload = json.dumps(result.to_dict(), ensure_ascii=False)
    assert '"ok": true' in payload


# ── 第五类故障：供电跌落（帧级系统错误码）──────────────────────────────
def test_fault_power_supply_drop_enters_fault_and_recovers():
    """供电跌落：注入帧级系统错误码 → FAULT → 恢复 → READY 且 q 已重同步。"""
    transport = ScriptedTransport()
    transport.open()
    transport.push_frame(_frame())
    machine = _streaming_machine(_frame())

    # 注入「供电跌落」：帧级系统错误码非 0（非 -3）
    transport.set_system_error(1)
    transport.push_frame(_frame(err_code=1))
    assert transport.read_state().err_code == 1

    diagnosis = classify_fault(transport)
    assert diagnosis is not None
    assert diagnosis.reason == "power_supply_drop"
    assert diagnosis.safe_state == DeviceState.FAULT
    assert diagnosis.action == "stop"
    # 诊断必须写明错误码数值，并如实标注厂商错误码 ↔ 供电跌落未核实
    assert "errCode=1" in diagnosis.detail
    assert "未核实" in diagnosis.detail

    assert enter_safe_state(transport, machine, diagnosis) == DeviceState.FAULT

    # 操作员恢复供电 → 控制器回一帧正常帧（errCode == 0）
    transport.set_system_error(0)
    transport.push_frame(_frame(positions=[0.7] * DOF))

    result = _procedure(transport, machine).run()

    assert result.ok, result.diagnosis
    assert result.failed_step is None
    assert machine.state == DeviceState.READY
    assert result.final_state == "ready"
    assert result.resynced_q_rad == pytest.approx((0.7,) * DOF)


class _StickySystemErrorTransport(ScriptedTransport):
    """**只清逐轴错误码、不清系统错误码**的替身（回归老行为的负例）。"""

    def clear_error(self) -> None:
        self._require_open()
        self.calls.append(("clear_error", ()))
        self._template = _replace(self._template, joint_err_codes=(0,) * DOF)


def test_power_supply_drop_recovery_fails_when_system_error_not_cleared():
    """负例：清错不清系统错误码 ⇒ 恢复**必须失败**（停在 resync_q），不得静默通过。"""
    transport = _StickySystemErrorTransport()
    transport.open()
    transport.push_frame(_frame())
    machine = _streaming_machine(_frame())

    transport.set_system_error(1)
    transport.push_frame(_frame(err_code=1))

    diagnosis = classify_fault(transport)
    assert diagnosis is not None
    assert diagnosis.reason == "power_supply_drop"
    assert enter_safe_state(transport, machine, diagnosis) == DeviceState.FAULT

    # 供电尚未恢复：最后一帧仍带系统错误码（清错也没清掉）
    result = _procedure(transport, machine).run()

    assert result.ok is False
    assert result.failed_step == "resync_q"
    assert "rm_clear_system_err" in result.diagnosis
    assert "errCode=1" in result.diagnosis
    assert machine.state == DeviceState.FAULT


def test_incomplete_frame_marker_is_not_a_fault():
    """负例：``err_code == -3``（帧不完整）**不是**故障原因。"""
    transport = ScriptedTransport()
    transport.open()
    transport.push_frame(_frame(err_code=-3))
    assert classify_fault(transport) is None


def test_zero_system_error_code_is_not_a_fault():
    """负例：``err_code == 0`` 不是故障。"""
    transport = ScriptedTransport()
    transport.open()
    transport.push_frame(_frame(err_code=0))
    assert classify_fault(transport) is None


def test_joint_error_still_detected_when_no_system_error():
    """负例：逐轴错误码非 0 仍判 ``joint_error``（优先级不倒置）。"""
    transport = ScriptedTransport()
    transport.open()
    transport.push_frame(_frame(err_codes=[7, 0, 0, 0, 0, 0, 0]))
    diagnosis = classify_fault(transport)
    assert diagnosis is not None
    assert diagnosis.reason == "joint_error"


def test_power_supply_drop_wins_over_joint_error():
    """优先级：供电跌落（系统错误码）> 逐关节错误码。"""
    transport = ScriptedTransport()
    transport.open()
    transport.push_frame(_frame(err_code=1, err_codes=[7, 0, 0, 0, 0, 0, 0]))
    diagnosis = classify_fault(transport)
    assert diagnosis is not None
    assert diagnosis.reason == "power_supply_drop"


def test_estop_wins_over_power_supply_drop():
    """优先级：急停 > 供电跌落（急停是最强信号）。"""
    transport = ScriptedTransport()
    transport.open()
    transport.push_frame(_frame(err_code=1, arm_status=9))
    diagnosis = classify_fault(transport)
    assert diagnosis is not None
    assert diagnosis.reason == "estop_status"
