"""实时上报状态测试（``rm_hw_bridge/bridge.py::_RealtimeState``）。

覆盖**拷贝语义**（SDK 复用结构体不得泄漏给调用方）：

* push 后 ``latest_joint_state()`` 返回拷贝的 7 个关节角（°）/ 速度（°/s）；
* push 之后再改原结构体 → 快照**不受影响**（证明是拷贝不是引用）；
* 无数据时字段为 ``None``（**不返回 0 冒充**）；
* 并发 push / read 不抛异常（线程安全只读访问器）；
* ``snapshot()`` 行为**保持不变**（S02 语义不回退）。
"""
from __future__ import annotations

import threading

from rm_hw_bridge import bridge


class _FakeJointStatus:
    """假 ``rm_joint_status_t``（字段语义与 SDK 结构体一致）。"""

    def __init__(self, position=None, speed=None):
        self.joint_position = list(position) if position is not None else [0.0] * 7
        self.joint_speed = list(speed) if speed is not None else [0.0] * 7


class _FakeRealtimeState:
    """假 ``rm_realtime_arm_joint_state_t``。"""

    def __init__(self, joint_status=None, arm_current_status=1):
        self.joint_status = joint_status
        self.arm_current_status = arm_current_status


def _state(position=None, speed=None):
    return _FakeRealtimeState(_FakeJointStatus(position, speed))


# ------------------------------------------------------------------ 正例
def test_latest_joint_state_returns_copies():
    """push 后返回拷贝的 7 个关节角 / 速度。"""
    state = bridge._RealtimeState()
    state.push(_state([1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0],
                       [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]))
    snap = state.latest_joint_state()
    assert snap["joint_position_deg"] == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]
    assert snap["joint_speed_deg_s"] == [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
    assert snap["frames"] == 1
    assert snap["last_frame_ts"] is not None


def test_snapshot_is_a_copy_not_a_reference():
    """push 之后再改原结构体 → 快照不受影响（拷贝而非引用）。"""
    joint = _FakeJointStatus([1.0] * 7, [2.0] * 7)
    state = bridge._RealtimeState()
    state.push(_FakeRealtimeState(joint))
    # 模拟 SDK 复用结构体：下一次回调原地覆写
    joint.joint_position[0] = 99.0
    joint.joint_speed[0] = 88.0
    snap = state.latest_joint_state()
    assert snap["joint_position_deg"][0] == 1.0
    assert snap["joint_speed_deg_s"][0] == 2.0


def test_returned_lists_are_copies():
    """返回值里的 list 是拷贝：调用方改动不影响内部缓存。"""
    state = bridge._RealtimeState()
    state.push(_state([1.0] * 7, [2.0] * 7))
    snap = state.latest_joint_state()
    snap["joint_position_deg"][0] = 42.0
    assert state.latest_joint_state()["joint_position_deg"][0] == 1.0


def test_no_data_returns_none_fields():
    """无数据时字段为 None（**不返回 0 冒充**）。"""
    state = bridge._RealtimeState()
    snap = state.latest_joint_state()
    assert snap["joint_position_deg"] is None
    assert snap["joint_speed_deg_s"] is None
    assert snap["last_frame_ts"] is None
    assert snap["frames"] == 0


def test_snapshot_behavior_unchanged():
    """``snapshot()`` 语义保持不变（S02 不回退）。"""
    state = bridge._RealtimeState()
    state.push(_state([1.0] * 7))
    snap = state.snapshot(20)
    assert snap["frames"] == 1
    assert snap["period_ms"] == 20
    assert snap["last_ts"] is not None
    assert state.arm_current_status == 1
    assert state.last_joint_status is not None


# ------------------------------------------------------------------ 负例
def test_malformed_joint_arrays_yield_none():
    """关节数组长度不对 / 含非数值 → 字段为 None（不静默补 0）。"""
    state = bridge._RealtimeState()
    state.push(_state([1.0, 2.0], [1.0] * 7))
    snap = state.latest_joint_state()
    assert snap["joint_position_deg"] is None
    assert snap["joint_speed_deg_s"] is not None  # 速度字段合法

    state2 = bridge._RealtimeState()
    state2.push(_state([1.0] * 7, ["x"] * 7))
    assert state2.latest_joint_state()["joint_speed_deg_s"] is None


def test_missing_joint_status_yields_none():
    """没有 joint_status 字段 → 关节字段为 None。"""
    state = bridge._RealtimeState()
    state.push(_FakeRealtimeState(None))
    snap = state.latest_joint_state()
    assert snap["joint_position_deg"] is None
    assert snap["joint_speed_deg_s"] is None
    assert snap["frames"] == 1


def test_concurrent_push_and_read_is_safe():
    """并发 push / read 不抛异常（线程安全只读访问器）。"""
    state = bridge._RealtimeState()
    errors = []

    def _writer():
        try:
            for i in range(300):
                state.push(_state([float(i)] * 7, [float(i)] * 7))
        except Exception as exc:  # noqa: BLE001
            errors.append(exc)

    def _reader():
        try:
            for _ in range(300):
                snap = state.latest_joint_state()
                pos = snap["joint_position_deg"]
                assert pos is None or len(pos) == 7
        except Exception as exc:  # noqa: BLE001
            errors.append(exc)

    threads = [threading.Thread(target=_writer), threading.Thread(target=_reader)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    assert errors == []
    assert state.latest_joint_state()["frames"] == 300
