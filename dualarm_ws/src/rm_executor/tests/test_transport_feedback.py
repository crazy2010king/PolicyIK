"""S03-T3 传输与反馈抽象的契约测试。

覆盖：

* :class:`rm_executor.transport.FakeTransport` —— 调用记录 + 参数校验 +
  脚本化故障注入；
* :class:`rm_executor.transport.BridgeTransport` —— 未连接 ArmLink 时
  **blocked 且 0 次 SDK 调用**、已连接时正确转发（不重复实现 SDK 调用）；
* :class:`rm_executor.feedback.ScriptedFeedback` —— 跟随误差 / 断流 / 静止
  场景注入；
* :class:`rm_executor.feedback.PollFeedback` / :class:`rm_executor.feedback.UdpFeedback`
  —— 成功路径与失败路径（rc≠0 / 访问器缺失）；
* :class:`rm_executor.feedback.FeedbackPump` —— 快照拷贝语义、
  ``last_age_ms`` 单调性、后台线程采集。

只用 git 跟踪的仓库文件 + 内联夹具；不依赖 .gitignore 目录。
"""
from __future__ import annotations

import time

import pytest

from rm_executor import feedback as fb
from rm_executor import transport as tp


DEG7 = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0]


# --------------------------------------------------------------- 假 ArmLink
class FakeArmLink:
    """最小 ArmLink 替身（**只记录调用**，不触达 SDK）。"""

    def __init__(self, connected=True, raises=False, rc=0,
                 status="ok", reason="", next_action=""):
        self.connected = connected
        self.reason = reason
        self.next_action = next_action
        self.evidence = []
        self.calls = []
        self._raises = raises
        self._rc = rc
        self._status = status

    def _result(self, symbol):
        if self._raises:
            raise RuntimeError("模拟 SDK 异常")
        return {
            "status": self._status,
            "reason": reason_text(self._status, symbol),
            "next_action": "",
            "evidence": ["%s rc=%d" % (symbol, self._rc)],
            "rc": self._rc,
            "sdk_calls": 1,
        }

    def preposition_movej(self, deg7, speed=10, **kwargs):
        self.calls.append(("preposition_movej", list(deg7), speed))
        return self._result("rm_movej")

    def transmit_joint(self, deg7, follow=False, trajectory_mode=1, radio=30, **kwargs):
        self.calls.append(
            ("transmit_joint", list(deg7), follow, trajectory_mode, radio)
        )
        return self._result("rm_movej_canfd")

    def stop(self):
        self.calls.append(("stop",))
        return self._result("rm_set_arm_stop")

    def slow_stop(self):
        self.calls.append(("slow_stop",))
        return self._result("rm_set_arm_slow_stop")

    def pause(self):
        self.calls.append(("pause",))
        return self._result("rm_set_arm_pause")

    def continue_(self):
        self.calls.append(("continue_",))
        return self._result("rm_set_arm_continue")

    def current_trajectory_type(self):
        self.calls.append(("current_trajectory_type",))
        return self._result("rm_get_arm_current_trajectory")


def reason_text(status, symbol):
    if status == "ok":
        return "%s 已下发" % symbol
    return "%s 返回非 0" % symbol


# ---------------------------------------------------------------- FakeTransport
def test_fake_transport_records_calls_with_all_parameters():
    tr = tp.FakeTransport()
    res = tr.transmit(DEG7, follow=True, trajectory_mode=1, radio=30)
    assert res["status"] == "ok"
    assert res["reason"]
    assert tr.count("transmit") == 1
    call = tr.last_call("transmit")
    assert call["args"]["deg7"] == DEG7
    assert call["args"]["follow"] is True
    assert call["args"]["trajectory_mode"] == 1
    assert call["args"]["radio"] == 30


def test_fake_transport_records_safety_calls():
    tr = tp.FakeTransport()
    for name in ("stop", "slow_stop", "pause", "continue_"):
        assert getattr(tr, name)()["status"] == "ok"
    assert tr.count("stop") == 1
    assert tr.count("slow_stop") == 1
    assert tr.count("pause") == 1
    assert tr.count("continue_") == 1
    assert tr.call_count == 4


def test_fake_transport_rejects_bad_deg7_without_recording_call():
    tr = tp.FakeTransport()
    res = tr.transmit([1.0, 2.0])
    assert res["status"] == "error"
    assert res["reason"]
    assert res["next_action"]
    assert res["sdk_calls"] == 0
    # 拒绝路径 **0 次下行**
    assert tr.count("transmit") == 0


@pytest.mark.parametrize(
    "bad",
    [None, "1234567", [1.0] * 6, [1.0] * 8, [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, "x"],
     [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, float("nan")], [True] * 7],
)
def test_fake_transport_rejects_malformed_deg7(bad):
    tr = tp.FakeTransport()
    res = tr.transmit(bad)
    assert res["status"] == "error"
    assert res["sdk_calls"] == 0
    assert tr.count("transmit") == 0


def test_fake_transport_scripted_failure_injects_rc():
    tr = tp.FakeTransport(script={"transmit": 7})
    res = tr.transmit(DEG7)
    assert res["status"] == "error"
    assert res["rc"] == 7
    assert "rc=7" in " ".join(res["evidence"])
    # 失败调用**仍被记录**（供 T4/T5 断言「已尝试下发」）
    assert tr.count("transmit") == 1


def test_fake_transport_scripted_dict_result_is_passed_through():
    tr = tp.FakeTransport(script={
        "transmit": {
            "status": "blocked",
            "reason": "控制器忙",
            "next_action": "等待一拍后重试",
            "evidence": ["busy"],
        }
    })
    res = tr.transmit(DEG7)
    assert res["status"] == "blocked"
    assert res["reason"] == "控制器忙"
    assert res["next_action"] == "等待一拍后重试"


def test_fake_transport_scripted_callable_can_pass_through():
    tr = tp.FakeTransport(script={
        "transmit": lambda method, args: None,  # 走默认成功路径
    })
    assert tr.transmit(DEG7)["status"] == "ok"


def test_fake_transport_current_trajectory_type_reports_no_planning():
    tr = tp.FakeTransport()
    res = tr.current_trajectory_type()
    assert res["status"] == "ok"
    assert res["type_name"] == "RM_NO_PLANNING_E"
    assert res["planning"] is False


def test_fake_transport_bounded_records_keep_exact_counts():
    """负载保护：调用记录可设上限（内存有界），但计数**永不被淘汰**。"""
    tr = tp.FakeTransport(max_records=5)
    for _ in range(20):
        tr.transmit(DEG7)
    assert len(tr.calls) == 5          # 内存有界
    assert tr.count("transmit") == 20  # 计数精确
    assert tr.call_count == 20


# -------------------------------------------------------------- BridgeTransport
def test_bridge_transport_blocked_when_link_missing_and_zero_sdk_calls():
    tr = tp.BridgeTransport(link=None)
    for name in ("stop", "slow_stop", "pause", "continue_", "current_trajectory_type"):
        res = getattr(tr, name)()
        assert res["status"] == "blocked"
        assert res["reason"]
        assert res["next_action"]
        assert res["sdk_calls"] == 0
    res = tr.transmit(DEG7)
    assert res["status"] == "blocked"
    assert res["sdk_calls"] == 0


def test_bridge_transport_blocked_when_link_not_connected_and_zero_sdk_calls():
    link = FakeArmLink(connected=False, reason="控制器不可达",
                       next_action="检查控制器电源与网线")
    tr = tp.BridgeTransport(link=link)
    res = tr.transmit(DEG7)
    assert res["status"] == "blocked"
    assert res["sdk_calls"] == 0
    assert res["reason"] == "控制器不可达"
    assert res["next_action"] == "检查控制器电源与网线"
    # **0 次 SDK 调用**
    assert link.calls == []


def test_bridge_transport_link_factory_failure_degrades_to_blocked():
    def boom():
        raise RuntimeError("无法连接控制器")

    tr = tp.BridgeTransport(link_factory=boom)
    res = tr.transmit(DEG7)
    assert res["status"] == "blocked"
    assert res["sdk_calls"] == 0
    assert "无法连接控制器" in res["reason"]


def test_bridge_transport_forwards_to_arm_link_without_own_sdk_calls():
    link = FakeArmLink()
    tr = tp.BridgeTransport(link=link)
    res = tr.transmit(DEG7, follow=True, trajectory_mode=1, radio=30)
    assert res["status"] == "ok"
    assert link.calls == [("transmit_joint", DEG7, True, 1, 30)]


def test_bridge_transport_preposition_converts_speed_to_int():
    link = FakeArmLink()
    tr = tp.BridgeTransport(link=link)
    res = tr.preposition(DEG7, speed=10.0)
    assert res["status"] == "ok"
    assert link.calls == [("preposition_movej", DEG7, 10)]


@pytest.mark.parametrize("bad_speed", [0, 0.5, 101, -1, "fast", None])
def test_bridge_transport_preposition_rejects_bad_speed_without_sdk_call(bad_speed):
    link = FakeArmLink()
    tr = tp.BridgeTransport(link=link)
    res = tr.preposition(DEG7, speed=bad_speed)
    assert res["status"] == "error"
    assert res["sdk_calls"] == 0
    assert link.calls == []


def test_bridge_transport_rejects_bad_deg7_without_sdk_call():
    link = FakeArmLink()
    tr = tp.BridgeTransport(link=link)
    res = tr.transmit([1.0, 2.0])
    assert res["status"] == "error"
    assert res["sdk_calls"] == 0
    assert link.calls == []


def test_bridge_transport_sdk_exception_becomes_error_result():
    link = FakeArmLink(raises=True)
    tr = tp.BridgeTransport(link=link)
    res = tr.transmit(DEG7)
    assert res["status"] == "error"
    assert res["reason"]
    assert res["next_action"]


def test_bridge_transport_sdk_error_status_is_preserved():
    link = FakeArmLink(status="error", rc=1)
    tr = tp.BridgeTransport(link=link)
    res = tr.transmit(DEG7)
    assert res["status"] == "error"
    assert res["rc"] == 1


# ------------------------------------------------------------- ScriptedFeedback
def test_scripted_feedback_returns_frames_in_order():
    src = fb.ScriptedFeedback(frames=[[0.0] * 7, [1.0] * 7])
    first = src.latest()
    second = src.latest()
    assert first["joint_position_deg"] == [0.0] * 7
    assert second["joint_position_deg"] == [1.0] * 7
    assert first["frames"] == 1
    assert second["frames"] == 2
    assert second["last_frame_ts"] > first["last_frame_ts"]


def test_scripted_feedback_injects_follow_error():
    src = fb.ScriptedFeedback(frames=[[0.0] * 7], follow_error_deg=6.0)
    snap = src.latest()
    assert snap["joint_position_deg"] == [6.0] * 7


def test_scripted_feedback_stream_loss_keeps_timestamp_and_frame_count():
    def script(index):
        return [0.0] * 7 if index == 0 else None

    src = fb.ScriptedFeedback(script=script)
    ok_snap = src.latest()
    stall = src.latest()
    assert stall["frames"] == ok_snap["frames"]      # 断流不产生新帧
    assert stall["last_frame_ts"] == ok_snap["last_frame_ts"]
    assert src.last_error


def test_scripted_feedback_static_scene():
    src = fb.ScriptedFeedback(static_deg=[3.0] * 7)
    first = src.latest()
    second = src.latest()
    assert first["joint_position_deg"] == [3.0] * 7
    assert second["joint_position_deg"] == [3.0] * 7
    assert second["frames"] == 2


def test_scripted_feedback_malformed_frame_is_reported_not_faked():
    src = fb.ScriptedFeedback(frames=[[0.0] * 3])
    snap = src.latest()
    assert snap["joint_position_deg"] is None
    assert snap["frames"] == 0
    assert src.last_error


def test_scripted_feedback_returns_copies():
    src = fb.ScriptedFeedback(frames=[[1.0] * 7])
    snap = src.latest()
    snap["joint_position_deg"][0] = 999.0
    again = src.latest()
    assert again["joint_position_deg"][0] != 999.0


# ------------------------------------------------------------------ PollFeedback
class FakeBinding:
    def __init__(self, rc=0, values=None):
        self.rc = rc
        self.values = values or [1.0] * 7
        self.calls = 0

    def rm_get_joint_degree(self, handle, joints):
        self.calls += 1
        for i, value in enumerate(self.values):
            joints[i] = value
        return self.rc


class FakeBridge:
    def __init__(self, binding):
        self.binding = binding


class FakeLink:
    def __init__(self, binding=None, connected=True):
        self.connected = connected
        self.handle = object()
        self.bridge = FakeBridge(binding) if binding else None
        self.reason = ""
        self.next_action = ""
        self.evidence = []


def test_poll_feedback_reads_joint_degree():
    binding = FakeBinding(values=[2.0] * 7)
    src = fb.PollFeedback(FakeLink(binding=binding))
    snap = src.latest()
    assert snap["joint_position_deg"] == [2.0] * 7
    assert snap["frames"] == 1
    assert binding.calls == 1
    assert not src.last_error


def test_poll_feedback_reports_sdk_rc_failure_without_faking_values():
    binding = FakeBinding(rc=1)
    src = fb.PollFeedback(FakeLink(binding=binding))
    snap = src.latest()
    assert snap["joint_position_deg"] is None
    assert snap["frames"] == 0
    assert "rm_get_joint_degree" in src.last_error
    assert "1" in src.last_error


def test_poll_feedback_blocked_when_link_not_connected():
    src = fb.PollFeedback(FakeLink(connected=False))
    snap = src.latest()
    assert snap["joint_position_deg"] is None
    assert src.last_error


# ------------------------------------------------------------------ UdpFeedback
class FakeRealtimeState:
    def __init__(self, data=None, frames=3):
        self._data = data
        self._frames = frames

    def latest_joint_state(self):
        return dict(self._data) if self._data is not None else {
            "joint_position_deg": None,
            "joint_speed_deg_s": None,
            "last_frame_ts": None,
            "frames": self._frames,
        }

    def snapshot(self, period_ms):
        return {"frames": self._frames, "last_ts": 1.5, "period_ms": period_ms}


def test_udp_feedback_reads_latest_joint_state_with_copy_semantics():
    state = FakeRealtimeState(data={
        "joint_position_deg": [1.0] * 7,
        "joint_speed_deg_s": [0.5] * 7,
        "last_frame_ts": 12.5,
        "frames": 42,
    })
    src = fb.UdpFeedback(realtime_state=state)
    snap = src.latest()
    assert snap["joint_position_deg"] == [1.0] * 7
    assert snap["joint_speed_deg_s"] == [0.5] * 7
    assert snap["frames"] == 42
    snap["joint_position_deg"][0] = 42.0
    assert src.latest()["joint_position_deg"][0] == 1.0


def test_udp_feedback_without_joint_state_reports_none_not_zero():
    class LegacyState:
        def snapshot(self, period_ms):
            return {"frames": 9, "last_ts": 2.0, "period_ms": period_ms}

    src = fb.UdpFeedback(realtime_state=LegacyState())
    snap = src.latest()
    assert snap["joint_position_deg"] is None
    assert snap["joint_speed_deg_s"] is None
    assert snap["frames"] == 9
    assert "latest_joint_state" in src.last_error


# ---------------------------------------------------------------- FeedbackPump
def test_feedback_pump_pull_once_updates_snapshot():
    src = fb.ScriptedFeedback(frames=[[1.0] * 7, [2.0] * 7])
    pump = fb.FeedbackPump(src, period_s=0.001)
    pump.pull_once()
    assert pump.frames == 1
    assert pump.latest()["joint_position_deg"] == [1.0] * 7
    pump.pull_once()
    assert pump.frames == 2
    assert pump.latest()["joint_position_deg"] == [2.0] * 7


def test_feedback_pump_snapshot_is_a_copy():
    src = fb.ScriptedFeedback(frames=[[1.0] * 7])
    pump = fb.FeedbackPump(src, period_s=0.001)
    pump.pull_once()
    snap = pump.latest()
    snap["joint_position_deg"][0] = 123.0
    assert pump.latest()["joint_position_deg"][0] == 1.0


def test_feedback_pump_last_age_is_monotonic_and_grows_on_stream_loss():
    state = {"alive": True}

    def script(index):
        if state["alive"] and index < 3:
            return [1.0] * 7
        return None

    src = fb.ScriptedFeedback(script=script)
    pump = fb.FeedbackPump(src, period_s=0.001)
    pump.pull_once()
    age0 = pump.last_age_ms()
    time.sleep(0.03)
    age1 = pump.last_age_ms()
    assert age1 >= age0
    # 断流后 age 继续增长（看门狗输入）
    state["alive"] = False
    pump.pull_once()
    time.sleep(0.03)
    age2 = pump.last_age_ms()
    assert age2 > age1
    assert pump.frames == 1  # 断流不产生新帧


def test_feedback_pump_background_thread_collects():
    src = fb.ScriptedFeedback(frames=[[float(i)] * 7 for i in range(50)])
    pump = fb.FeedbackPump(src, period_s=0.001)
    pump.start()
    try:
        deadline = time.monotonic() + 2.0
        while pump.frames < 5 and time.monotonic() < deadline:
            time.sleep(0.005)
        assert pump.frames >= 5
        assert pump.running
    finally:
        pump.stop()
    assert not pump.running


def test_feedback_pump_records_source_exception_without_dying():
    class Boom:
        def latest(self):
            raise RuntimeError("反馈源炸了")

        def to_dict(self):
            return {"name": "boom"}

    pump = fb.FeedbackPump(Boom(), period_s=0.001)
    pump.pull_once()
    assert pump.errors
    assert "反馈源炸了" in pump.errors[0]
    assert pump.latest()["joint_position_deg"] is None


def test_feedback_pump_tolerates_non_dict_source_output():
    class Weird:
        def latest(self):
            return [1, 2, 3]

        def to_dict(self):
            return {"name": "weird"}

    pump = fb.FeedbackPump(Weird(), period_s=0.001)
    pump.pull_once()
    assert pump.errors
    assert pump.latest()["frames"] == 0


def test_feedback_pump_zero_sdk_calls_in_read_path():
    """tick 线程只读缓存：``latest()`` 不会触发任何反馈源拉取。"""
    calls = {"n": 0}

    class Counting:
        def latest(self):
            calls["n"] += 1
            return {"joint_position_deg": [0.0] * 7, "joint_speed_deg_s": None,
                    "last_frame_ts": 1.0, "frames": 1}

        def to_dict(self):
            return {"name": "counting"}

    pump = fb.FeedbackPump(Counting(), period_s=0.001)
    pump.pull_once()
    before = calls["n"]
    for _ in range(10):
        pump.latest()
    assert calls["n"] == before
