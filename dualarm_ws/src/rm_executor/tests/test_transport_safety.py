"""S04-T3 传输层安全窄接口的契约测试。

覆盖：

* :class:`rm_executor.transport.FakeTransport`
  - ``set_speed_scale``：合法比例 → 记录调用（含 ``scale`` / ``joint_index``），
    返回 ``applied_scale``；**比例越界 / 非有限 / 非数值 → 0 次记录**；
  - ``controller_state``：默认**无错误码**（``arm_err=[]``、
    ``arm_current_status=None``），可用 ``script`` 脚本化（故障注入）；
* :class:`rm_executor.transport.BridgeTransport`
  - ArmLink 不可用 / 未连接 → **blocked 且 0 次 SDK 调用**；
  - ``set_speed_scale`` 的目标速度 = **声明限值 × scale**（度制数值断言）；
  - 声明限值不可用 → blocked + 0 次 SDK 调用；
  - SDK 返回失败 → error + 可粘贴 next_action + 失败关节写明；
  - ``controller_state``：成功透传 ``arm_err`` / ``arm_current_status`` /
    ``arm_current_status_name``；失败路径**不伪造**（字段为 None）。

只用 git 跟踪的仓库文件 + 内联夹具；不依赖 .gitignore 目录。
"""
from __future__ import annotations

import pytest

from rm_executor import transport as tp


# --------------------------------------------------------------- 假 ArmLink
class FakeArmLink:
    """最小 ArmLink 替身（**只记录调用**，不触达 SDK）。"""

    def __init__(self, connected=True, speed_status="ok", speed_reason="",
                 speed_next_action="", raises=False,
                 arm_err=None, arm_current_status=None,
                 arm_current_status_name=None, state_status="ok"):
        self.connected = connected
        self.reason = ""
        self.next_action = ""
        self.evidence = []
        self.calls = []
        self._speed_status = speed_status
        self._speed_reason = speed_reason
        self._speed_next_action = speed_next_action
        self._raises = raises
        self._arm_err = arm_err
        self._arm_current_status = arm_current_status
        self._arm_current_status_name = arm_current_status_name
        self._state_status = state_status

    # -- S04 新增窄接口 ---------------------------------------------------
    def set_drive_max_speed(self, joint_index, deg_s):
        self.calls.append(("set_drive_max_speed", joint_index, deg_s))
        if self._raises:
            raise RuntimeError("模拟 SDK 异常")
        return {
            "status": self._speed_status,
            "reason": self._speed_reason,
            "next_action": self._speed_next_action,
            "evidence": ["rm_set_joint_drive_max_speed rc=0"],
            "sdk_calls": 1,
        }

    def controller_error_state(self):
        self.calls.append(("controller_error_state",))
        if self._raises:
            raise RuntimeError("模拟 SDK 异常")
        return {
            "status": self._state_status,
            "reason": "" if self._state_status == "ok" else "控制器错误态读取失败",
            "next_action": "",
            "evidence": ["rm_get_current_arm_state rc=0"],
            "sdk_calls": 1,
            "arm_err": self._arm_err,
            "arm_current_status": self._arm_current_status,
            "arm_current_status_name": self._arm_current_status_name,
        }

    # -- 既有接口（本用例不需要，保持接口面最小） -------------------------
    def preposition_movej(self, deg7, speed=10, **kwargs):
        self.calls.append(("preposition_movej",))
        return {"status": "ok", "reason": "", "next_action": "", "evidence": []}

    def transmit_joint(self, deg7, follow=False, trajectory_mode=1, radio=30, **kwargs):
        self.calls.append(("transmit_joint",))
        return {"status": "ok", "reason": "", "next_action": "", "evidence": []}

    def stop(self):
        self.calls.append(("stop",))
        return {"status": "ok", "reason": "", "next_action": "", "evidence": []}

    def slow_stop(self):
        self.calls.append(("slow_stop",))
        return {"status": "ok", "reason": "", "next_action": "", "evidence": []}

    def pause(self):
        self.calls.append(("pause",))
        return {"status": "ok", "reason": "", "next_action": "", "evidence": []}

    def continue_(self):
        self.calls.append(("continue_",))
        return {"status": "ok", "reason": "", "next_action": "", "evidence": []}

    def current_trajectory_type(self):
        self.calls.append(("current_trajectory_type",))
        return {"status": "ok", "reason": "", "next_action": "", "evidence": []}


def _speed_calls(link):
    return [c for c in link.calls if c[0] == "set_drive_max_speed"]


# ------------------------------------------------------------ FakeTransport
def test_fake_set_speed_scale_records_call():
    fake = tp.FakeTransport()
    res = fake.set_speed_scale(0.5, joint_index=3)
    assert res["status"] == "ok"
    assert res["applied_scale"] == 0.5
    assert res["sdk_calls"] == 0
    calls = fake.calls_for("set_speed_scale")
    assert len(calls) == 1
    assert calls[0]["args"]["scale"] == 0.5
    assert calls[0]["args"]["joint_index"] == 3
    assert fake.count("set_speed_scale") == 1


def test_fake_set_speed_scale_defaults_to_all_joints():
    fake = tp.FakeTransport()
    res = fake.set_speed_scale(1.0)
    assert res["status"] == "ok"
    assert fake.last_call("set_speed_scale")["args"]["joint_index"] is None


@pytest.mark.parametrize("bad", [0, 0.0, -0.5, 1.5, 2, float("nan"),
                                 float("inf"), "0.5", None, True])
def test_fake_set_speed_scale_rejects_bad_scale_without_recording(bad):
    fake = tp.FakeTransport()
    res = fake.set_speed_scale(bad)
    assert res["status"] == "error"
    assert res["sdk_calls"] == 0
    assert res["applied_scale"] is None
    assert res["next_action"], "拒绝路径必须给出可粘贴的下一步"
    assert fake.calls_for("set_speed_scale") == []
    assert fake.count("set_speed_scale") == 0


def test_fake_controller_state_defaults_no_error_code():
    fake = tp.FakeTransport()
    res = fake.controller_state()
    assert res["status"] == "ok"
    assert res["arm_err"] == []
    assert res["arm_current_status"] is None
    assert res["arm_current_status_name"] is None
    assert fake.count("controller_state") == 1


def test_fake_controller_state_scriptable_for_injection():
    fake = tp.FakeTransport(script={"controller_state": {
        "status": "ok",
        "reason": "注入：保护停",
        "next_action": "",
        "evidence": ["injected"],
        "arm_err": [3, 5],
        "arm_current_status": 7,
        "arm_current_status_name": "RM_ARM_ERR_PROTECTION_STOP",
    }})
    res = fake.controller_state()
    assert res["status"] == "ok"
    assert res["arm_err"] == [3, 5]
    assert res["arm_current_status"] == 7
    assert res["arm_current_status_name"] == "RM_ARM_ERR_PROTECTION_STOP"


def test_fake_controller_state_script_failure_does_not_fake_empty_error_code():
    fake = tp.FakeTransport(script={"controller_state": 1})
    res = fake.controller_state()
    assert res["status"] == "error"
    assert res["arm_err"] is None


# ---------------------------------------------------------- BridgeTransport
def test_bridge_without_link_is_blocked_with_zero_sdk_calls():
    transport = tp.BridgeTransport()
    res = transport.set_speed_scale(0.5)
    assert res["status"] == "blocked"
    assert res["sdk_calls"] == 0
    assert res["applied_scale"] is None
    assert res["next_action"]

    state = transport.controller_state()
    assert state["status"] == "blocked"
    assert state["sdk_calls"] == 0
    assert state["arm_err"] is None
    assert state["arm_current_status"] is None
    assert state["arm_current_status_name"] is None


def test_bridge_not_connected_is_blocked_with_zero_sdk_calls():
    link = FakeArmLink(connected=False)
    transport = tp.BridgeTransport(link=link)
    res = transport.set_speed_scale(0.5)
    assert res["status"] == "blocked"
    assert res["sdk_calls"] == 0
    assert link.calls == []

    state = transport.controller_state()
    assert state["status"] == "blocked"
    assert state["sdk_calls"] == 0
    assert link.calls == []


def test_bridge_set_speed_scale_uses_declared_limits_times_scale(joint_limits):
    from rm_hw_bridge.bridge import RAD2DEG

    link = FakeArmLink()
    transport = tp.BridgeTransport(link=link)
    scale = 0.5
    res = transport.set_speed_scale(scale)
    assert res["status"] == "ok"
    assert res["applied_scale"] == scale
    assert res["sdk_calls"] == 7

    calls = _speed_calls(link)
    assert len(calls) == 7
    for idx, (_sym, joint_index, deg_s) in enumerate(calls):
        assert joint_index == idx
        expected = joint_limits["joint%d" % (idx + 1)]["max_speed"] * RAD2DEG * scale
        assert deg_s == pytest.approx(expected, rel=1e-12)
    # 声明限值（rad/s）→ 度制：目标速度必须显著大于 SI 数值
    assert calls[0][2] > joint_limits["joint1"]["max_speed"]


def test_bridge_set_speed_scale_single_joint(joint_limits):
    from rm_hw_bridge.bridge import RAD2DEG

    link = FakeArmLink()
    transport = tp.BridgeTransport(link=link)
    res = transport.set_speed_scale(0.25, joint_index=2)
    assert res["status"] == "ok"
    assert res["sdk_calls"] == 1
    calls = _speed_calls(link)
    assert len(calls) == 1
    assert calls[0][1] == 2
    expected = joint_limits["joint3"]["max_speed"] * RAD2DEG * 0.25
    assert calls[0][2] == pytest.approx(expected, rel=1e-12)


@pytest.mark.parametrize("bad", [0, 1.5, float("nan"), "x"])
def test_bridge_set_speed_scale_rejects_bad_scale_without_sdk_call(bad):
    link = FakeArmLink()
    transport = tp.BridgeTransport(link=link)
    res = transport.set_speed_scale(bad)
    assert res["status"] == "error"
    assert res["sdk_calls"] == 0
    assert res["applied_scale"] is None
    assert link.calls == []


@pytest.mark.parametrize("bad_idx", [7, -1, 1.5, True, "1"])
def test_bridge_set_speed_scale_rejects_bad_joint_index_without_sdk_call(bad_idx):
    link = FakeArmLink()
    transport = tp.BridgeTransport(link=link)
    res = transport.set_speed_scale(0.5, joint_index=bad_idx)
    assert res["status"] == "error"
    assert res["sdk_calls"] == 0
    assert link.calls == []


def test_bridge_set_speed_scale_blocked_when_limits_unavailable(monkeypatch):
    from rm_hw_bridge import limits as hw_limits

    def _boom(ws=None):
        raise FileNotFoundError("找不到关节限值文件")

    monkeypatch.setattr(hw_limits, "load_joint_limits", _boom)
    link = FakeArmLink()
    transport = tp.BridgeTransport(link=link)
    res = transport.set_speed_scale(0.5)
    assert res["status"] == "blocked"
    assert res["sdk_calls"] == 0
    assert res["applied_scale"] is None
    assert res["next_action"]
    assert link.calls == []


def test_bridge_set_speed_scale_blocked_when_limits_structure_invalid(monkeypatch):
    from rm_hw_bridge import limits as hw_limits

    monkeypatch.setattr(hw_limits, "load_joint_limits", lambda ws=None: {})
    link = FakeArmLink()
    transport = tp.BridgeTransport(link=link)
    res = transport.set_speed_scale(0.5)
    assert res["status"] == "blocked"
    assert res["sdk_calls"] == 0
    assert link.calls == []


def test_bridge_set_speed_scale_reports_sdk_failure():
    link = FakeArmLink(speed_status="error", speed_reason="rm_set_joint_drive_max_speed rc=-1",
                       speed_next_action="确认控制器在线后重跑")
    transport = tp.BridgeTransport(link=link)
    res = transport.set_speed_scale(0.5)
    assert res["status"] == "error"
    assert res["applied_scale"] is None
    assert res["sdk_calls"] == 1
    assert "joint1" in res["reason"]
    assert res["next_action"] == "确认控制器在线后重跑"


def test_bridge_set_speed_scale_reports_sdk_exception():
    link = FakeArmLink(raises=True)
    transport = tp.BridgeTransport(link=link)
    res = transport.set_speed_scale(0.5)
    assert res["status"] == "error"
    assert res["sdk_calls"] == 1
    assert res["next_action"]
    assert len(_speed_calls(link)) == 1


def test_bridge_controller_state_passes_through_error_code():
    link = FakeArmLink(arm_err=[2], arm_current_status=9,
                       arm_current_status_name="RM_ARM_ERR_COLLISION")
    transport = tp.BridgeTransport(link=link)
    res = transport.controller_state()
    assert res["status"] == "ok"
    assert res["arm_err"] == [2]
    assert res["arm_current_status"] == 9
    assert res["arm_current_status_name"] == "RM_ARM_ERR_COLLISION"
    assert ("controller_error_state",) in link.calls


def test_bridge_controller_state_sdk_failure_does_not_fake_data():
    link = FakeArmLink(raises=True)
    transport = tp.BridgeTransport(link=link)
    res = transport.controller_state()
    assert res["status"] == "error"
    assert res["arm_err"] is None
    assert res["arm_current_status"] is None
    assert res["arm_current_status_name"] is None


def test_bridge_controller_state_error_status_does_not_fake_data():
    link = FakeArmLink(state_status="error")
    transport = tp.BridgeTransport(link=link)
    res = transport.controller_state()
    assert res["status"] == "error"
    assert res["arm_err"] is None
