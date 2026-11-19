"""真机 SDK **绑定**与 UDP 主动上报的回归测试（零硬件）。

覆盖
----
* 绑定层：连接（``rm_create_robot_arm``）、句柄校验、**实例方法优先**的调用路由、
  审计记账、连接失败时的显式报错；
* 主动上报：配置周期口径、帧解码（度→rad）、周期统计分位数。
"""

from __future__ import annotations

import types

import pytest

from rm75_drake.device.binding import RM_TRIPLE_MODE_E, ArmBinding
from rm75_drake.device.config import device_config_from_dict
from rm75_drake.device.realtime import (
    REALTIME_PUSH_CYCLE_UNIT_MS,
    RealtimePushSettings,
    decode_realtime_state,
)
from rm75_drake.device.sdk import SdkAudit
from rm75_drake.device.state_source import StateSource


def _config():
    return device_config_from_dict(
        {
            "schema_version": 1,
            "name": "rm75_b",
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


class _FakeArm:
    """假 ``RoboticArm`` 实例（模型化真 SDK 的实例方法形态）。"""

    def __init__(self, mode, calls, handle_id=1):
        self.mode = mode
        self.calls = calls
        self.arm_dof = 7
        self.robot_controller_version = 3
        self.handle = types.SimpleNamespace(id=handle_id)

    def rm_create_robot_arm(self, ip, port):
        self.calls.append(("rm_create_robot_arm", ip, int(port)))
        return self.handle

    def rm_movej_canfd(self, joint, follow=False, expand=0, trajectory_mode=0, radio=0):
        self.calls.append(("rm_movej_canfd", list(joint)))
        return 0


def _fake_sdk(handle_id=1):
    calls = []
    sdk = types.SimpleNamespace()
    sdk.rm_robot_interface = types.SimpleNamespace(
        RoboticArm=lambda mode: _FakeArm(mode, calls, handle_id=handle_id)
    )
    sdk.calls = calls
    return sdk


# ── 绑定 ────────────────────────────────────────────────────────────────
def test_triple_mode_constant_matches_sdk_enum():
    """真机实测：RM_TRIPLE_MODE_E = 2（传 3 会被 SDK 判 Invalid thread mode）。"""
    assert RM_TRIPLE_MODE_E == 2


def test_binding_connects_and_reports_capabilities():
    sdk = _fake_sdk()
    binding = ArmBinding(_config(), sdk=sdk)
    report = binding.open()
    assert report.connected is True
    assert report.arm_dof == 7
    assert report.controller_version == 3
    assert sdk.calls[0] == ("rm_create_robot_arm", "192.168.1.18", 8080)


def test_binding_reports_invalid_handle_as_not_connected():
    sdk = _fake_sdk(handle_id=-1)
    binding = ArmBinding(_config(), sdk=sdk)
    report = binding.open()
    assert report.connected is False
    assert "句柄" in report.detail


def test_binding_routes_calls_to_instance_methods():
    sdk = _fake_sdk()
    binding = ArmBinding(_config(), sdk=sdk)
    binding.open()
    binding.call("rm_movej_canfd", [1.0] * 7, follow=True, trajectory_mode=0)
    assert ("rm_movej_canfd", [1.0] * 7) in sdk.calls


def test_binding_unknown_symbol_raises_attribute_error():
    sdk = _fake_sdk()
    binding = ArmBinding(_config(), sdk=sdk)
    binding.open()
    with pytest.raises(AttributeError):
        binding.call("rm_not_exist")


def test_binding_audits_every_call():
    audit = SdkAudit()
    sdk = _fake_sdk()
    binding = ArmBinding(_config(), sdk=sdk, audit=audit)
    binding.open()
    binding.call("rm_movej_canfd", [0.0] * 7)
    assert audit.called("rm_movej_canfd") == 1
    assert audit.non_whitelist_call_count == 0
    assert audit.forbidden_call_count == 0


def test_binding_close_does_not_delete_handle():
    """真机实测：``rm_delete_robot_arm`` 会让 SDK 接收线程段错误 ⇒ 默认不删。"""
    sdk = _fake_sdk()
    binding = ArmBinding(_config(), sdk=sdk)
    binding.open()
    binding.close()
    assert not any(call[0] == "rm_delete_robot_arm" for call in sdk.calls)


# ── 主动上报 ────────────────────────────────────────────────────────────
def test_push_settings_expected_period_uses_measured_unit():
    """真机实测标定：cycle 单位是 **5 ms 的倍数**（cycle=1 ⇒ 5 ms）。"""
    settings = RealtimePushSettings(ip="192.168.1.198", port=8080, cycle=1)
    assert settings.expected_period_ms() == 5.0
    assert REALTIME_PUSH_CYCLE_UNIT_MS == 5.0


def test_decode_realtime_state_converts_degrees_to_radians():
    raw = types.SimpleNamespace(
        errCode=0,
        joint_status=types.SimpleNamespace(
            joint_position=[180.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            joint_current=[1.0] * 7,
            joint_en_flag=[True] * 7,
            joint_err_code=[0] * 7,
        ),
        arm_current_status=0,
        rm_plus_state=0,
    )
    frame = decode_realtime_state(raw)
    assert frame.joint_positions_rad[0] == pytest.approx(3.14159265, abs=1e-6)
    assert frame.all_en if hasattr(frame, "all_en") else True
    assert frame.gripper_online is True
    assert frame.err_code == 0


def test_decode_realtime_state_keeps_incomplete_marker():
    raw = types.SimpleNamespace(
        errCode=-3,
        joint_status=types.SimpleNamespace(
            joint_position=[0.0] * 7,
            joint_current=[0.0] * 7,
            joint_en_flag=[False] * 7,
            joint_err_code=[0] * 7,
        ),
        arm_current_status=0,
        rm_plus_state=1,
    )
    frame = decode_realtime_state(raw)
    assert frame.err_code == -3
    assert frame.gripper_online is False


def test_state_source_rejects_incomplete_frames():
    source = StateSource()
    raw = types.SimpleNamespace(
        errCode=-3,
        joint_status=types.SimpleNamespace(
            joint_position=[0.0] * 7,
            joint_current=[0.0] * 7,
            joint_en_flag=[False] * 7,
            joint_err_code=[0] * 7,
        ),
        arm_current_status=0,
        rm_plus_state=0,
    )
    frame = decode_realtime_state(raw).to_state_frame()
    assert source.push(frame) is False
    assert source.frames_rejected_incomplete == 1
