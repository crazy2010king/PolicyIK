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
    ArmTransport,
    REASON_INVALID_INPUT,
    REASON_LINK_DOWN,
    REASON_NOT_OPEN,
    REASON_UNSUPPORTED,
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


class _FakeArm:
    """假 ``RoboticArm`` **实例**（模型化真 SDK 的真实形态）。

    真机实测（RM75-B / SDK 1.1.7）确认：厂商 API 的调用形态是
    **实例方法**，且：

    * 建连接必须显式调 ``rm_create_robot_arm(ip, port)``；
    * 逐关节接口（使能 / 清错 / 限位）是**逐轴**调用（``joint_num`` + 值）；
    * 夹爪位置接口是 ``rm_set_gripper_position(position, block, timeout)``；
    * 急停接口是 ``rm_set_arm_emergency_stop(state: bool)``。
    """

    def __init__(self, mode, calls):
        self.mode = mode
        self.calls = calls
        self.arm_dof = 7
        self.robot_controller_version = 3
        self.handle = types.SimpleNamespace(id=1)

    def rm_create_robot_arm(self, ip, port):
        self.calls.append(("rm_create_robot_arm", ip, int(port)))
        return self.handle

    def rm_movej_canfd(self, joint, follow=False, expand=0, trajectory_mode=0, radio=0):
        self.calls.append(("rm_movej_canfd", list(joint), follow, trajectory_mode))
        return 0

    def rm_set_joint_en_state(self, joint_num, en_state):
        self.calls.append(("rm_set_joint_en_state", int(joint_num), int(en_state)))
        # 真机实测：joint_num=0 会被控制器拒绝（返回 1）
        return 1 if int(joint_num) < 1 else 0

    def rm_set_joint_clear_err(self, joint_num):
        self.calls.append(("rm_set_joint_clear_err", int(joint_num)))
        return 0

    def rm_clear_system_err(self):
        self.calls.append(("rm_clear_system_err",))
        return 0

    def rm_set_arm_emergency_stop(self, state=True):
        self.calls.append(("rm_set_arm_emergency_stop", bool(state)))
        return 0

    def rm_set_arm_slow_stop(self):
        self.calls.append(("rm_set_arm_slow_stop",))
        return 0

    def rm_set_joint_min_pos(self, joint_num, min_pos):
        self.calls.append(("rm_set_joint_min_pos", int(joint_num), float(min_pos)))
        return 1 if int(joint_num) < 1 else 0

    def rm_set_joint_max_pos(self, joint_num, max_pos):
        self.calls.append(("rm_set_joint_max_pos", int(joint_num), float(max_pos)))
        return 0

    def rm_get_gripper_state(self):
        self.calls.append(("rm_get_gripper_state",))
        return 0, {
            "enable_state": 1,
            "status": 1,
            "error": 0,
            "mode": 1,
            "current_force": 0,
            "temperature": 25,
            "actpos": 512,
        }

    def rm_set_gripper_position(self, position, block, timeout):
        self.calls.append(
            ("rm_set_gripper_position", int(position), bool(block), int(timeout))
        )
        return 0

    def rm_get_current_arm_state(self):
        self.calls.append(("rm_get_current_arm_state",))
        return 0, {
            "joint": [0.0] * 7,
            "pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "err": {"err_len": 1, "err": ["0"]},
        }

    def rm_get_current_joint_current(self):
        self.calls.append(("rm_get_current_joint_current",))
        return 0, [0.0] * 7


class _FakeSdk:
    """假 SDK 命名空间（只提供 ``rm_robot_interface``，模拟官方包的空顶层）。"""

    def __init__(self) -> None:
        self.calls = []
        self.rm_robot_interface = types.SimpleNamespace(
            RoboticArm=lambda mode: _FakeArm(mode, self.calls)
        )


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
    name, joint, follow, trajectory_mode = fake.calls[1]
    assert name == "rm_movej_canfd"
    assert joint == pytest.approx([180.0] * 7)
    assert follow is True
    assert trajectory_mode == 0


def test_sdk_transport_connects_with_rm_create_robot_arm():
    """真机形态：必须显式建连接（不调 ``rm_create_robot_arm`` 就没有句柄）。"""
    fake = _FakeSdk()
    transport = SdkTransport(_config(), sdk=fake)
    transport.open()
    assert fake.calls[0] == ("rm_create_robot_arm", "192.168.1.18", 8080)


def test_sdk_transport_reports_controller_rejection():
    """控制器拒绝（返回非 0）⇒ 必须显式报错，不能当成成功（真机实测：假绿灯的来源）。"""
    fake = _FakeSdk()
    transport = SdkTransport(_config(), sdk=fake)

    class _Rejecting(_FakeArm):
        def rm_set_joint_en_state(self, joint_num, en_state):
            self.calls.append(("rm_set_joint_en_state", int(joint_num), int(en_state)))
            return 1  # 控制器拒绝

    fake.rm_robot_interface = types.SimpleNamespace(
        RoboticArm=lambda mode: _Rejecting(mode, fake.calls)
    )
    transport.open()
    with pytest.raises(TransportError) as excinfo:
        transport.enable(True)
    assert excinfo.value.reason == "sdk_rejected"


def test_sdk_transport_enable_and_clear_error_are_per_joint():
    """逐关节接口（使能 / 清错）是**逐轴**调用（真机实测签名）。"""
    fake = _FakeSdk()
    transport = SdkTransport(_config(), sdk=fake)
    transport.open()
    transport.enable(True)
    transport.clear_error()
    enables = [c for c in fake.calls if c[0] == "rm_set_joint_en_state"]
    clears = [c for c in fake.calls if c[0] == "rm_set_joint_clear_err"]
    assert len(enables) == 7
    assert len(clears) == 7
    # ⚠️ 厂商 joint_num 是 **1-based（1~7）**（真机实测：传 0 被控制器拒绝）
    assert enables[0] == ("rm_set_joint_en_state", 1, 1)
    assert enables[-1] == ("rm_set_joint_en_state", 7, 1)
    assert clears[0] == ("rm_set_joint_clear_err", 1)
    assert clears[-1] == ("rm_set_joint_clear_err", 7)


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
    assert names["rm_set_joint_min_pos"][2] == pytest.approx(90.0)
    assert names["rm_set_joint_max_pos"][2] == pytest.approx(90.0)
    first_min = next(c for c in fake.calls if c[0] == "rm_set_joint_min_pos")
    assert first_min[1] == 1  # joint_num 是 1-based
    assert len([c for c in fake.calls if c[0] == "rm_set_joint_min_pos"]) == 7


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


# ── 夹爪下发通道（S06-T02）─────────────────────────────────────────────
class _BareTransport(ArmTransport):
    """最小可实例化传输层：只实现抽象方法，用于验证**基类默认实现**。"""

    name = "bare_transport"

    def open(self) -> None:
        self._open = True
        self._closed = False

    def close(self) -> None:
        self._open = False
        self._closed = True

    def move_j_canfd(self, joint_rad, *, follow: bool = True) -> None:
        self._require_open()

    def enable(self, state: bool) -> None:
        self._require_open()

    def clear_error(self) -> None:
        self._require_open()

    def emergency_stop(self) -> None:
        self._require_open()

    def slow_stop(self) -> None:
        self._require_open()

    def set_joint_limits(self, lower_rad, upper_rad) -> None:
        self._require_open()

    def read_state(self):
        self._require_open()
        return None

    def read_gripper(self):
        self._require_open()
        from rm75_drake.device.transport import GripperState

        return GripperState(online=True, position=0)


def test_base_transport_command_gripper_position_is_unsupported():
    """基类默认实现：抛 TransportError（reason=unsupported），不得静默无操作。"""
    transport = _BareTransport()
    transport.open()
    with pytest.raises(TransportError) as excinfo:
        transport.command_gripper_position(500)
    assert excinfo.value.reason == REASON_UNSUPPORTED
    assert "command_gripper_position" in str(excinfo.value)


def test_sdk_transport_command_gripper_position_uses_whitelisted_symbol():
    """真机实现：走白名单符号 rm_set_gripper_position（审计可见）。"""
    fake = _FakeSdk()
    transport = SdkTransport(_config(), sdk=fake)
    transport.open()
    transport.command_gripper_position(700)
    names = [call[0] for call in fake.calls]
    assert "rm_set_gripper_position" in names
    assert transport.audit.called("rm_set_gripper_position") == 1
    assert transport.audit.non_whitelist_call_count == 0


def test_sdk_transport_command_gripper_position_passes_raw_dimensionless_value():
    """夹爪位置是无量纲 0–1000 整数：不换算，原值下发。"""
    fake = _FakeSdk()
    transport = SdkTransport(_config(), sdk=fake)
    transport.open()
    transport.command_gripper_position(800)
    args = [call for call in fake.calls if call[0] == "rm_set_gripper_position"]
    assert args[0][1] == 800


def test_sdk_transport_command_gripper_position_requires_open():
    fake = _FakeSdk()
    transport = SdkTransport(_config(), sdk=fake)
    with pytest.raises(TransportError) as excinfo:
        transport.command_gripper_position(500)
    assert excinfo.value.reason == REASON_NOT_OPEN


@pytest.mark.parametrize("bad", [1001, -1, 0.5, "500", None, True])
def test_sdk_transport_command_gripper_position_rejects_invalid(bad):
    """越界 / 非整数：显式报错并点名 position。"""
    fake = _FakeSdk()
    transport = SdkTransport(_config(), sdk=fake)
    transport.open()
    with pytest.raises(TransportError) as excinfo:
        transport.command_gripper_position(bad)
    assert excinfo.value.reason == REASON_INVALID_INPUT
    assert "position" in str(excinfo.value)


def test_sdk_transport_command_gripper_position_rejects_invalid_speed():
    fake = _FakeSdk()
    transport = SdkTransport(_config(), sdk=fake)
    transport.open()
    with pytest.raises(TransportError) as excinfo:
        transport.command_gripper_position(500, speed=500)
    assert excinfo.value.reason == REASON_INVALID_INPUT
    assert "speed" in str(excinfo.value)


def test_scripted_transport_command_gripper_position_records_and_updates_readback():
    """离线替身：记录下发调用，并把回读值更新为该位置（夹爪到位语义）。"""
    transport = ScriptedTransport()
    transport.open()
    transport.command_gripper_position(300)
    assert transport.called("command_gripper_position") == 1
    assert transport.calls[-1] == ("command_gripper_position", (300,))
    assert transport.read_gripper().position == 300


def test_scripted_transport_command_gripper_position_requires_open():
    transport = ScriptedTransport()
    with pytest.raises(TransportError) as excinfo:
        transport.command_gripper_position(300)
    assert excinfo.value.reason == REASON_NOT_OPEN


def test_scripted_transport_set_gripper_position_semantics_unchanged():
    """回读设定器 set_gripper_position 不记录调用（语义与下发通道不同）。"""
    transport = ScriptedTransport()
    transport.open()
    transport.set_gripper_position(42)
    assert transport.called("command_gripper_position") == 0
    assert transport.read_gripper().position == 42


def test_both_transports_expose_the_same_gripper_command_surface():
    """真机实现与离线替身共享同一夹爪下发签名（可互换性）。"""
    import inspect

    for klass in (ArmTransport, SdkTransport, ScriptedTransport):
        sig = inspect.signature(klass.command_gripper_position)
        assert list(sig.parameters) == ["self", "position", "speed"]
        assert sig.parameters["speed"].kind is inspect.Parameter.KEYWORD_ONLY
        assert sig.parameters["speed"].default == 100


# ── 帧级系统错误码（供电跌落的语义载体）────────────────────────────────
def test_scripted_transport_can_inject_system_error_code():
    """注入帧级系统错误码 ⇒ 推入的帧携带该 err_code（供电跌落的载体）。"""
    transport = ScriptedTransport()
    transport.open()
    transport.set_system_error(1)
    frame = transport.push_frame()
    assert frame.err_code == 1
    assert transport.read_state().err_code == 1


def test_scripted_transport_set_system_error_requires_push_frame():
    """只改模板不推帧 ⇒ read_state 仍返回上一帧（注入必须真的推帧）。"""
    transport = ScriptedTransport()
    transport.open()
    transport.push_frame()
    transport.set_system_error(1)
    assert transport.read_state().err_code == 0


def test_scripted_transport_clear_error_clears_system_error_of_last_frame():
    """清错必须同时清「模板 + 最后一帧」的**系统错误码**（对齐 rm_clear_system_err）。

    不清最后一帧 ⇒ 恢复流程的 resync_q / reset_ready 读到的仍是带错误码的帧
    ⇒ 恢复必然失败。
    """
    transport = ScriptedTransport()
    transport.open()
    transport.set_joint_error_codes([3] * 7)
    transport.set_system_error(1)
    transport.push_frame()
    assert transport.read_state().err_code == 1

    transport.clear_error()

    # 最后一帧的系统错误码已清（否则恢复守卫会拒绝它）
    frame = transport.read_state()
    assert frame.err_code == 0
    # 模板的逐轴错误码已清零（下一帧无错）
    fresh = transport.push_frame()
    assert fresh.joint_err_codes == (0,) * 7
    assert fresh.err_code == 0


def test_scripted_transport_clear_error_keeps_incomplete_marker():
    """清错**不得**把帧不完整标记（-3）抹掉（坏帧不能被清成好帧）。"""
    transport = ScriptedTransport()
    transport.open()
    transport.set_frame_incomplete(True)
    transport.push_frame()

    transport.clear_error()

    assert transport.read_state().err_code == -3
    assert transport.read_state().is_complete is False


def test_scripted_transport_frame_incomplete_marker_stays_minus_three():
    """``set_frame_incomplete(True)`` 仍是 -3（帧不完整 ≠ 故障原因）。"""
    transport = ScriptedTransport()
    transport.open()
    transport.set_frame_incomplete(True)
    frame = transport.push_frame()
    assert frame.err_code == -3
    assert frame.is_complete is False


def test_scripted_transport_set_frame_incomplete_false_clears_system_error():
    """``set_frame_incomplete(False)`` 会把 errCode 归零（交互必须被钉死）。"""
    transport = ScriptedTransport()
    transport.open()
    transport.set_system_error(9)
    transport.set_frame_incomplete(False)
    frame = transport.push_frame()
    assert frame.err_code == 0
    assert frame.is_complete is True


def test_scripted_transport_set_system_error_rejects_non_integer():
    """负例：系统错误码必须是整数（bool 也不算）。"""
    transport = ScriptedTransport()
    transport.open()
    with pytest.raises(TransportError) as excinfo:
        transport.set_system_error("1")  # type: ignore[arg-type]
    assert excinfo.value.reason == REASON_INVALID_INPUT
    with pytest.raises(TransportError):
        transport.set_system_error(True)  # type: ignore[arg-type]
