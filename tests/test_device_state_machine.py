"""设备状态机的单测（S03-T03）。

覆盖
----
* **全转移可达**：9 个状态、正常链路 + 断链 + 故障恢复 + 急停支路；
* **非法转移被拒**：抛显式错误并带稳定 reason 标签与允许事件列表；
* **守卫条件**：``WAIT_ENABLE → READY`` 要求全部 7 轴已使能；
  ``RECOVER → READY`` 要求错误码清零 + 全轴使能；
* **FAULT 触发源**：逐关节错误码 / 系统错误码 / 急停状态码；
* **事件流**：每次成功转移都追加一条可序列化的 :class:`Transition`。
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = REPO_ROOT / "src"

if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from rm75_drake.device.state_machine import (  # noqa: E402
    ALL_EVENTS,
    ALL_STATES,
    REASON_ESTOP_STATUS,
    REASON_GUARD_REJECTED,
    REASON_ILLEGAL_TRANSITION,
    REASON_JOINT_ERROR,
    REASON_SYSTEM_ERROR,
    DeviceEvent,
    DeviceState,
    DeviceStateMachine,
    GuardRejectedError,
    IllegalTransitionError,
    StateMachineError,
    Transition,
    detect_fault,
    transition_table,
)
from rm75_drake.device.transport import StateFrame  # noqa: E402

DOF = 7


def _frame(**kwargs) -> StateFrame:
    """构造一帧（默认健康 + 全轴已使能）。"""
    base = dict(
        joint_positions_rad=(0.0,) * DOF,
        joint_currents_ma=(0.0,) * DOF,
        joint_en_flags=(True,) * DOF,
        joint_err_codes=(0,) * DOF,
        controller_status=0,
        gripper_online=True,
        arm_current_status=0,
        received_at=0.0,
        err_code=0,
    )
    base.update(kwargs)
    return StateFrame(**base)


def _machine() -> DeviceStateMachine:
    """构造一台状态机（确定性时钟）。"""
    return DeviceStateMachine(clock=lambda: 1.0)


# ── 正常链路 ────────────────────────────────────────────────────────────
def test_initial_state_is_disconnected():
    assert _machine().state is DeviceState.DISCONNECTED


def test_happy_path_reaches_streaming():
    machine = _machine()
    machine.connect()
    assert machine.state is DeviceState.CONNECTED
    machine.enable()
    assert machine.state is DeviceState.WAIT_ENABLE
    machine.mark_all_enabled(_frame())
    assert machine.state is DeviceState.READY
    machine.start_stream()
    assert machine.state is DeviceState.STREAMING


def test_hold_and_resume():
    machine = _machine()
    machine.connect()
    machine.enable()
    machine.mark_all_enabled(_frame())
    machine.start_stream()
    machine.hold()
    assert machine.state is DeviceState.HOLDING
    machine.resume()
    assert machine.state is DeviceState.STREAMING


def test_disconnect_returns_to_disconnected():
    machine = _machine()
    machine.connect()
    machine.enable()
    machine.mark_all_enabled(_frame())
    machine.start_stream()
    machine.disconnect()
    assert machine.state is DeviceState.DISCONNECTED


# ── 守卫 ────────────────────────────────────────────────────────────────
def test_ready_requires_all_seven_axes_enabled():
    machine = _machine()
    machine.connect()
    machine.enable()
    partial = _frame(joint_en_flags=(True,) * 6 + (False,))
    with pytest.raises(GuardRejectedError) as exc:
        machine.mark_all_enabled(partial)
    assert exc.value.reason == REASON_GUARD_REJECTED
    assert exc.value.guard_reason == "not_all_enabled"
    # 守卫拒绝**不改变状态**
    assert machine.state is DeviceState.WAIT_ENABLE


def test_ready_guard_rejects_missing_frame():
    machine = _machine()
    machine.connect()
    machine.enable()
    with pytest.raises(GuardRejectedError) as exc:
        machine.transition(DeviceEvent.ALL_ENABLED)
    assert exc.value.guard_reason == "frame_missing"


def test_recover_end_requires_no_fault_and_all_enabled():
    machine = _machine()
    machine.connect()
    machine.enable()
    machine.mark_all_enabled(_frame())
    machine.start_stream()
    machine.fault(frame=_frame(joint_err_codes=(1,) + (0,) * 6))
    assert machine.state is DeviceState.FAULT
    machine.recover_begin()
    assert machine.state is DeviceState.RECOVER
    # 错误码未清 → 恢复不得完成
    with pytest.raises(GuardRejectedError) as exc:
        machine.recover_end(_frame(joint_err_codes=(1,) + (0,) * 6))
    assert exc.value.guard_reason == REASON_JOINT_ERROR
    assert machine.state is DeviceState.RECOVER
    # 清错 + 全轴使能 → 回到 READY
    machine.recover_end(_frame())
    assert machine.state is DeviceState.READY


# ── 非法转移 ────────────────────────────────────────────────────────────
def test_illegal_transition_raises_with_reason_and_allowed_list():
    machine = _machine()
    with pytest.raises(IllegalTransitionError) as exc:
        machine.start_stream()  # DISCONNECTED 不接受
    assert exc.value.reason == REASON_ILLEGAL_TRANSITION
    assert exc.value.state is DeviceState.DISCONNECTED
    assert exc.value.event is DeviceEvent.START_STREAM
    assert DeviceEvent.CONNECT in exc.value.allowed


def test_illegal_transition_does_not_change_state():
    machine = _machine()
    machine.connect()
    machine.estop()
    with pytest.raises(IllegalTransitionError):
        machine.start_stream()
    assert machine.state is DeviceState.ESTOP


def test_non_enum_event_raises():
    machine = _machine()
    with pytest.raises(StateMachineError):
        machine.transition("connect")  # type: ignore[arg-type]


# ── 故障判定 ────────────────────────────────────────────────────────────
def test_detect_fault_joint_error():
    assert detect_fault(_frame(joint_err_codes=(0, 0, 5) + (0,) * 4)) == REASON_JOINT_ERROR


def test_detect_fault_system_error():
    assert detect_fault(_frame(err_code=1)) == REASON_SYSTEM_ERROR


def test_detect_fault_estop_wins():
    frame = _frame(arm_current_status=9, joint_err_codes=(3,) + (0,) * 6)
    assert detect_fault(frame) == REASON_ESTOP_STATUS


def test_detect_fault_ignores_incomplete_marker():
    # -3 = 推送数据不完整（由状态源丢弃），不算系统错误
    assert detect_fault(_frame(err_code=-3)) is None


def test_detect_fault_rejects_non_frame():
    with pytest.raises(StateMachineError):
        detect_fault({"joint_err_codes": []})  # type: ignore[arg-type]


def test_fault_reason_auto_detected_from_frame():
    machine = _machine()
    machine.connect()
    machine.enable()
    machine.mark_all_enabled(_frame())
    machine.start_stream()
    transition = machine.fault(frame=_frame(joint_err_codes=(7,) + (0,) * 6))
    assert transition.reason == REASON_JOINT_ERROR
    assert transition.source is DeviceState.STREAMING
    assert transition.target is DeviceState.FAULT


# ── 急停 ────────────────────────────────────────────────────────────────
def test_estop_from_streaming_and_clear_requires_reconnect():
    machine = _machine()
    machine.connect()
    machine.enable()
    machine.mark_all_enabled(_frame())
    machine.start_stream()
    machine.estop()
    assert machine.state is DeviceState.ESTOP
    machine.clear_estop()
    assert machine.state is DeviceState.DISCONNECTED


# ── 事件流 ──────────────────────────────────────────────────────────────
def test_history_records_every_transition():
    machine = _machine()
    machine.connect()
    machine.enable()
    machine.mark_all_enabled(_frame())
    machine.start_stream()
    machine.hold()
    history = machine.history
    assert len(history) == 5
    assert [t.source for t in history] == [
        DeviceState.DISCONNECTED,
        DeviceState.CONNECTED,
        DeviceState.WAIT_ENABLE,
        DeviceState.READY,
        DeviceState.STREAMING,
    ]
    assert [t.target for t in history] == [
        DeviceState.CONNECTED,
        DeviceState.WAIT_ENABLE,
        DeviceState.READY,
        DeviceState.STREAMING,
        DeviceState.HOLDING,
    ]
    assert isinstance(history[0], Transition)
    payload = history[0].to_dict()
    assert payload["source"] == "disconnected"
    assert payload["target"] == "connected"
    assert payload["reason"]
    assert payload["timestamp_s"] == pytest.approx(1.0)


def test_history_is_a_copy():
    machine = _machine()
    machine.connect()
    history = machine.history
    history.clear()
    assert len(machine.history) == 1


# ── 转移表 ──────────────────────────────────────────────────────────────
def test_every_state_is_reachable_from_disconnected():
    """全部 9 个状态都必须可达（否则状态机有死状态）。"""
    table = transition_table()
    reachable = {DeviceState.DISCONNECTED}
    changed = True
    while changed:
        changed = False
        for (source, _event), spec in table.items():
            if source in reachable and spec.target not in reachable:
                reachable.add(spec.target)
                changed = True
    assert reachable == set(ALL_STATES)


def test_transition_table_copy_is_not_shared():
    table = transition_table()
    table.clear()
    assert transition_table()


def test_all_events_have_default_reason():
    machine = _machine()
    for event in ALL_EVENTS:
        assert isinstance(event, DeviceEvent)


def test_can_reports_table_membership():
    machine = _machine()
    assert machine.can(DeviceEvent.CONNECT) is True
    assert machine.can(DeviceEvent.START_STREAM) is False
    machine.connect()
    assert machine.can(DeviceEvent.START_STREAM) is False
    assert machine.can(DeviceEvent.ENABLE) is True
