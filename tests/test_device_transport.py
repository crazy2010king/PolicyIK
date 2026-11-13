"""传输层（真机实现 + 离线替身）的测试（S03-T01）。

覆盖：

* :class:`ScriptedTransport`：脚本化逐帧回放（使能标志 / 逐关节错误码 / 电流 /
  位置 / 断链 / 陈旧帧 / 夹爪离线 / 控制器状态枚举）；
* :class:`SdkTransport`：**只用白名单 SDK 符号**、rad→deg 在下发前换算、
  ``follow=True`` / ``trajectory_mode=0``、未打开就操作会显式报错；
* 负例：维度错 / 非有限值 / 断链下发 / SDK 缺失。
"""

from __future__ import annotations

import math
import types

import pytest

from rm75_drake.device import sdk as sdk_mod
from rm75_drake.device.config import device_config_from_dict
from rm75_drake.device.transport import (
    REASON_INVALID_INPUT,
    REASON_LINK_DOWN,
    REASON_NOT_OPEN,
    ScriptedTransport,
    SdkTransport,
    StateFrame,
    TransportError,
)


def _config():
    return device_config_from_dict(
        {
            "schema_version": 1,
            "name": "rm75_single_arm",
            "arm_ip": "192.168.1.18",
            "tcp_port": 8080,
            "udp_port": 8080,
            "udp_cycle": 1,
            "joint_zero_offset": [0.0] * 7,
            "accel_limits": {"measured": False, "values": [6.0] * 7},
            "current_thresholds": {"calibrated": False, "values": [1000.0] * 7},
            "fence": {"table_height_m": 0.75, "singularity_radius_m": 0.15},
        }
    )


class _FakeSdk:
    """假 SDK：记录调用参数，模拟白名单符号。"""

    def __init__(self) -> None:
        self.calls = []
        self.rm_robot_interface = types.SimpleNamespace(
            RoboticArm=lambda mode: types.SimpleNamespace(arm_dof=7, mode=mode)
        )

    def rm_movej_canfd(self, joint, follow=False, trajectory_mode=0):
        self.calls.append(("rm_movej_canfd", list(joint), follow, trajectory_mode))
        return 0

    def rm_set_joint_en_state(self, state):
        self.calls.append(("rm_set_joint_en_state", bool(state)))
        return 0

    def rm_set_joint_clear_err(self):
        self.calls.append(("rm_set_joint_clear_err",))
        return 0

    def rm_clear_system_err(self):
        self.calls.append(("rm_clear_system_err",))
        return 0

    def rm_set_arm_emergency_stop(self):
        self.calls.append(("rm_set_arm_emergency_stop",))
        return 0

    def rm_set_arm_slow_stop(self):
        self.calls.append(("rm_set_arm_slow_stop",))
        return 0

    def rm_set_joint_min_pos(self, pos):
        self.calls.append(("rm_set_joint_min_pos", list(pos)))
        return 0

    def rm_set_joint_max_pos(self, pos):
        self.calls.append(("rm_set_joint_max_pos", list(pos)))
        return 0

    def rm_get_gripper_state(self):
        self.calls.append(("rm_get_gripper_state",))
        return types.SimpleNamespace(rm_plus_state=0, gripper_position=512)


# ── ScriptedTransport ──────────────────────────────────────────────────
def test_scripted_transport_open_close_are_idempotent():
    transport = ScriptedTransport()
    transport.open()
    transport.open()
    assert transport.is_open
    transport.close()
    transport.close()
    assert not transport.is_open


def test_scripted_transport_records_move_calls():
    transport = ScriptedTransport()
    transport.open()
    transport.move_j_canfd([0.1] * 7)
    assert transport.called("move_j_canfd") == 1
    assert transport.calls[0][0] == "move_j_canfd"


def test_scripted_transport_push_frame_and_read_state():
    transport = ScriptedTransport()
    transport.open()
    transport.set_positions([0.2] * 7)
    transport.set_currents([120.0] * 7)
    transport.set_enable_flags([True] * 7)
    transport.push_frame()
    frame = transport.read_state()
    assert frame is not None
    assert frame.joint_positions_rad == pytest.approx((0.2,) * 7)
    assert frame.joint_currents_ma == pytest.approx((120.0,) * 7)
    assert frame.all_enabled is True


def test_scripted_transport_can_inject_joint_error_codes():
    transport = ScriptedTransport()
    transport.open()
    transport.set_joint_error_codes([0, 0, 17, 0, 0, 0, 0])
    frame = transport.push_frame()
    assert frame.joint_err_codes[2] == 17
    assert frame.joint_err_codes[0] == 0


def test_scripted_transport_can_inject_stale_frame():
    transport = ScriptedTransport()
    transport.open()
    stale = StateFrame(received_at=100.0)
    transport.push_frame(stale)
    frame = transport.read_state()
    assert frame is not None
    assert frame.received_at == 100.0


def test_scripted_transport_can_inject_incomplete_frame():
    transport = ScriptedTransport()
    transport.open()
    transport.set_frame_incomplete(True)
    frame = transport.push_frame()
    assert frame.err_code == -3
    assert frame.is_complete is False


def test_scripted_transport_can_inject_controller_status():
    transport = ScriptedTransport()
    transport.open()
    transport.set_controller_status(5)
    transport.set_arm_current_status(9)  # 急停
    frame = transport.push_frame()
    assert frame.controller_status == 5
    assert frame.arm_current_status == 9


def test_scripted_transport_link_down_blocks_state_and_moves():
    """拔网线：状态帧读不到 + 指令下发失败（稳定 reason=link_down）。"""
    transport = ScriptedTransport()
    transport.open()
    transport.push_frame()
    transport.drop_link()
    assert transport.read_state() is None
    with pytest.raises(TransportError) as excinfo:
        transport.move_j_canfd([0.0] * 7)
    assert excinfo.value.reason == REASON_LINK_DOWN


def test_scripted_transport_gripper_offline():
    transport = ScriptedTransport()
    transport.open()
    assert transport.read_gripper().online is True
    transport.set_gripper_offline(True)
    assert transport.read_gripper().online is False


def test_scripted_transport_enable_updates_flags():
    transport = ScriptedTransport()
    transport.open()
    transport.enable(True)
    frame = transport.push_frame()
    assert frame.all_enabled is True


def test_scripted_transport_clear_error_resets_codes():
    transport = ScriptedTransport()
    transport.open()
    transport.set_joint_error_codes([1] * 7)
    transport.clear_error()
    frame = transport.push_frame()
    assert frame.joint_err_codes == (0,) * 7


def test_scripted_transport_emergency_and_slow_stop_set_status():
    transport = ScriptedTransport()
    transport.open()
    transport.emergency_stop()
    assert transport.push_frame().arm_current_status == 9
    transport.slow_stop()
    assert transport.push_frame().arm_current_status == 10


def test_scripted_transport_rejects_wrong_dimension():
    transport = ScriptedTransport()
    transport.open()
    with pytest.raises(TransportError) as excinfo:
        transport.move_j_canfd([0.0] * 6)
    assert excinfo.value.reason == REASON_INVALID_INPUT


def test_scripted_transport_rejects_non_finite_value():
    transport = ScriptedTransport()
    transport.open()
    with pytest.raises(TransportError) as excinfo:
        transport.move_j_canfd([float("inf")] + [0.0] * 6)
    assert excinfo.value.reason == REASON_INVALID_INPUT


def test_scripted_transport_requires_open():
    transport = ScriptedTransport()
    with pytest.raises(TransportError) as excinfo:
        transport.move_j_canfd([0.0] * 7)
    assert excinfo.value.reason == REASON_NOT_OPEN


# ── SdkTransport ───────────────────────────────────────────────────────
def test_sdk_transport_converts_rad_to_deg_before_sending():
    """下发前必须换成度（SDK 全线用度；把弧度当度会飞到 57 倍位置）。"""
    fake = _FakeSdk()
    transport = SdkTransport(_config(), sdk=fake)
    transport.open()
    transport.move_j_canfd([math.pi] * 7)
    name, joint, follow, trajectory_mode = fake.calls[0]
    assert name == "rm_movej_canfd"
    assert joint == pytest.approx([180.0] * 7)
    assert follow is True
    assert trajectory_mode == 0


def test_sdk_transport_only_uses_whitelisted_symbols():
    """真机实现只允许调用白名单符号（运行时审计那半）。"""
    fake = _FakeSdk()
    transport = SdkTransport(_config(), sdk=fake)
    transport.open()
    transport.enable(True)
    transport.clear_error()
    transport.emergency_stop()
    transport.slow_stop()
    transport.set_joint_limits([-1.0] * 7, [1.0] * 7)
    transport.move_j_canfd([0.0] * 7)
    transport.read_gripper()
    assert transport.audit.non_whitelist_call_count == 0
    assert transport.audit.total_call_count >= 7


def test_sdk_transport_clear_error_order():
    """清错顺序：逐关节错误码 → 系统错误码。"""
    fake = _FakeSdk()
    transport = SdkTransport(_config(), sdk=fake)
    transport.open()
    transport.clear_error()
    names = [call[0] for call in fake.calls]
    assert names.index("rm_set_joint_clear_err") < names.index("rm_clear_system_err")


def test_sdk_transport_set_joint_limits_converts_to_deg():
    fake = _FakeSdk()
    transport = SdkTransport(_config(), sdk=fake)
    transport.open()
    transport.set_joint_limits([math.pi / 2] * 7, [math.pi / 2] * 7)
    names = {call[0]: call for call in fake.calls}
    assert names["rm_set_joint_min_pos"][1] == pytest.approx([90.0] * 7)
    assert names["rm_set_joint_max_pos"][1] == pytest.approx([90.0] * 7)


def test_sdk_transport_read_gripper_maps_state():
    fake = _FakeSdk()
    transport = SdkTransport(_config(), sdk=fake)
    transport.open()
    gripper = transport.read_gripper()
    assert gripper.online is True
    assert gripper.position == 512


def test_sdk_transport_requires_open_before_move():
    fake = _FakeSdk()
    transport = SdkTransport(_config(), sdk=fake)
    with pytest.raises(TransportError) as excinfo:
        transport.move_j_canfd([0.0] * 7)
    assert excinfo.value.reason == REASON_NOT_OPEN


def test_sdk_transport_rejects_wrong_dimension():
    fake = _FakeSdk()
    transport = SdkTransport(_config(), sdk=fake)
    transport.open()
    with pytest.raises(TransportError) as excinfo:
        transport.move_j_canfd([0.0] * 5)
    assert excinfo.value.reason == REASON_INVALID_INPUT


@pytest.mark.skipif(
    sdk_mod.sdk_available(), reason="本机已安装 robotic-arm，缺失路径不可测"
)
def test_sdk_transport_open_without_sdk_raises_not_available():
    """SDK 缺失 → 显式 NotAvailableError（不得静默降级成「假连接」）。"""
    transport = SdkTransport(_config())
    with pytest.raises(sdk_mod.NotAvailableError):
        transport.open()


def test_sdk_transport_open_is_idempotent():
    fake = _FakeSdk()
    transport = SdkTransport(_config(), sdk=fake)
    transport.open()
    transport.open()
    assert transport.is_open
    transport.close()
    assert not transport.is_open


def test_both_transports_share_the_same_protocol():
    """真机实现与离线替身必须遵守同一协议（可互换性）。"""
    from rm75_drake.device.transport import ArmTransport

    assert issubclass(ScriptedTransport, ArmTransport)
    assert issubclass(SdkTransport, ArmTransport)
