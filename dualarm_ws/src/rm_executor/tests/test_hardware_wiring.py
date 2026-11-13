"""硬件接线测试（执行层 ↔ 硬件层）。

覆盖：

* ``UdpFeedback`` 从**假 realtime_state** 读取**拷贝**快照（改返回值不影响源）；
* ``UdpFeedback`` 对接真实 ``rm_hw_bridge.bridge.realtime_state()``
  （``latest_joint_state()`` 拷贝语义）——真机路径的接线闭环；
* ``BridgeTransport`` 在 ArmLink 未连接时返回 blocked 且 **0 次 SDK 调用**；
* 执行层**不 import rmctl**、**不 import tesseract***（import 后 sys.modules 检查）。
"""
from __future__ import annotations

import sys

import pytest

from rm_executor import feedback as fb
from rm_executor import transport as tr


class _FakeJointStatus:
    def __init__(self, position, speed):
        self.joint_position = list(position)
        self.joint_speed = list(speed)


class _FakeState:
    """假 realtime_state（提供 latest_joint_state 接口）。"""

    def __init__(self, position=None, speed=None, frames=1, ts=1.0):
        self._data = {
            "joint_position_deg": list(position) if position is not None else None,
            "joint_speed_deg_s": list(speed) if speed is not None else None,
            "last_frame_ts": ts,
            "frames": frames,
        }

    def latest_joint_state(self):
        return dict(self._data)


class _OldState:
    """旧版 realtime_state：只有 snapshot()（没有 latest_joint_state）。"""

    def __init__(self, frames=3):
        self._frames = frames

    def snapshot(self, period_ms):
        return {"frames": self._frames, "last_ts": 1.0, "period_ms": period_ms}


# ------------------------------------------------------------------ UDP 反馈
def test_udp_feedback_reads_copied_snapshot():
    """从假 realtime_state 读取拷贝快照。"""
    state = _FakeState([1.0] * 7, [2.0] * 7, frames=5)
    source = fb.UdpFeedback(state)
    snap = source.latest()
    assert snap["joint_position_deg"] == [1.0] * 7
    assert snap["joint_speed_deg_s"] == [2.0] * 7
    assert snap["frames"] == 5
    assert source.last_error == ""


def test_udp_feedback_returns_copies():
    """返回值是拷贝：调用方改动不影响源状态。"""
    state = _FakeState([1.0] * 7, [2.0] * 7)
    source = fb.UdpFeedback(state)
    snap = source.latest()
    snap["joint_position_deg"][0] = 99.0
    assert source.latest()["joint_position_deg"][0] == 1.0


def test_udp_feedback_degrades_without_accessor():
    """没有 latest_joint_state() → 降级为「只有帧计数」并写明原因（不返回 0 冒充）。"""
    source = fb.UdpFeedback(_OldState(frames=7))
    snap = source.latest()
    assert snap["joint_position_deg"] is None
    assert snap["joint_speed_deg_s"] is None
    assert snap["frames"] == 7
    assert "latest_joint_state" in source.last_error


def test_udp_feedback_without_data_reports_none():
    """无数据时关节字段为 None（**不返回 0 冒充**）。"""
    source = fb.UdpFeedback(_FakeState(None, None, frames=0, ts=None))
    snap = source.latest()
    assert snap["joint_position_deg"] is None
    assert snap["joint_speed_deg_s"] is None
    assert snap["frames"] == 0


def test_udp_feedback_reads_real_bridge_state():
    """对接真实 ``rm_hw_bridge.bridge.realtime_state()``（拷贝语义闭环）。"""
    from rm_hw_bridge import bridge as hw_bridge

    state = hw_bridge.realtime_state()
    before = state.latest_joint_state()
    joint = _FakeJointStatus([1.0] * 7, [0.5] * 7)

    class _S:
        arm_current_status = 1
        joint_status = joint

    state.push(_S())
    source = fb.UdpFeedback()
    snap = source.latest()
    assert snap["joint_position_deg"] == [1.0] * 7
    assert snap["joint_speed_deg_s"] == [0.5] * 7
    assert snap["frames"] == before["frames"] + 1
    # 拷贝：改原结构体后快照不变
    joint.joint_position[0] = 42.0
    assert source.latest()["joint_position_deg"][0] == 1.0


# ------------------------------------------------------------------ 真机传输
def test_bridge_transport_without_link_is_blocked():
    """ArmLink 未连接 → blocked + 0 次 SDK 调用。"""
    transport = tr.BridgeTransport()
    result = transport.transmit([1.0] * 7)
    assert result["status"] == "blocked"
    assert result["reason"]
    assert result["next_action"]
    assert result.get("sdk_calls") == 0
    assert result.get("arm_link_available") is False


def test_bridge_transport_unconnected_link_is_blocked():
    """ArmLink 存在但未连接 → blocked + 0 次 SDK 调用。"""

    class _DeadLink:
        connected = False
        reason = "控制器不可达"
        next_action = "先检查控制器上电"
        evidence = []

    transport = tr.BridgeTransport(link=_DeadLink())
    result = transport.preposition([1.0] * 7, 10)
    assert result["status"] == "blocked"
    assert result.get("sdk_calls") == 0


def test_bridge_transport_rejects_invalid_deg7_without_sdk_call():
    """非法关节角 → error 且 0 次 SDK 调用（拒绝路径不触达控制器）。"""
    transport = tr.BridgeTransport()
    result = transport.transmit([1.0, 2.0])
    assert result["status"] == "error"
    assert result.get("sdk_calls") == 0


def test_make_live_transport_is_lazy_and_blocked_without_controller():
    """真机工厂：惰性建链；控制器不可达 → blocked（不抛裸异常）。"""
    transport = tr.make_live_transport(ip="127.0.0.1", port=1)
    result = transport.stop()
    assert result["status"] in ("blocked", "error")
    assert result["reason"]
    assert result["next_action"]


def test_live_feedback_factory_builds_pump():
    """真机反馈栈工厂：后台采集泵 + UDP 反馈源（tick 线程零 SDK 调用）。"""
    pump = fb.make_live_feedback(period_s=0.01)
    assert isinstance(pump, fb.FeedbackPump)
    assert isinstance(pump.source, fb.UdpFeedback)
    assert pump.to_dict()["source"]["name"] == "udp"


# ------------------------------------------------------------------ 分层红线
def test_executor_does_not_import_rmctl_or_tesseract():
    """执行层不得 import rmctl / tesseract*（import 后 sys.modules 检查）。"""
    import rm_executor  # noqa: F401
    from rm_executor import clock, contract, events, executor, metrics  # noqa: F401
    from rm_executor import report, resample  # noqa: F401

    assert "rmctl" not in sys.modules
    assert not [m for m in sys.modules if m.split(".")[0].startswith("tesseract")]
