"""RM75-B 真机最小动作安全探针（``scripts/rm75_safe_probe.py``）的**纯逻辑**用例。

为什么单独测（且**不碰真机**）：安全探针的唯一价值是"出问题时立刻停手"，
因此它的**判定逻辑**必须能被离线钉死：

* 安全包络（位移 / 速度 / 关节序号）——越界必须在 **0 次 SDK 调用**前被拒；
* 只读前置检查（错误码 / 使能位 / 控制器是否在跑别的轨迹 / 限位余量）；
* 回读判定（到位误差 / 其它关节漂移 / 回程归位）；
* **失败路径必须急停**（``rm_set_arm_stop``）且**不得再发运动指令**。

假句柄注入 ``run_probe(link_factory=...)`` → 本文件**不依赖 RM-API2 真库**。
"""
from __future__ import annotations

import importlib.util
import os
import sys

import pytest

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(TESTS_DIR, "..", "..", "..", ".."))
SCRIPT_PATH = os.path.join(REPO_ROOT, "scripts", "rm75_safe_probe.py")


def _load_script():
    spec = importlib.util.spec_from_file_location("rm75_safe_probe", SCRIPT_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def probe():
    return _load_script()


# ------------------------------------------------------------------ 假句柄
class FakeLink(object):
    """假句柄：实现探针用到的 4 个接口，并记录全部调用。"""

    def __init__(self, joints=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                 snapshot_overrides=None, move_status="ok", safety_status="ok",
                 follow_error_deg=0.0, return_error_deg=0.0):
        self.joints = [float(v) for v in joints]
        self.snapshot_overrides = dict(snapshot_overrides or {})
        self.move_status = move_status
        self.safety_status = safety_status
        self.follow_error_deg = float(follow_error_deg)
        self.return_error_deg = float(return_error_deg)
        self.calls = []
        self.moves = []
        self.closed = False

    # --- 采集
    def snapshot(self):
        self.calls.append(("snapshot",))
        snap = {
            "joint_deg": list(self.joints),
            "joint_current_mA": [0.0] * 7,
            "joint_temperature_c": [30.0] * 7,
            "joint_voltage_v": [48.0] * 7,
            "joint_en_flag": [1] * 7,
            "joint_err_code": [0] * 7,
            "arm_err": [],
            "controller_trajectory_type": 0,
            "controller_trajectory_type_name": "RM_NO_PLANNING_E",
            "realtime": {"frames": 100, "last_ts": 1.0, "period_ms": 5},
        }
        snap.update(self.snapshot_overrides)
        return {"status": "ok", "reason": "", "snapshot": snap}

    def set_safety(self, collision_stage=None, self_collision_enable=None, dry_run=False):
        self.calls.append(("set_safety", collision_stage, self_collision_enable))
        if self.safety_status != "ok":
            return {"status": self.safety_status, "reason": "安全开关下发失败"}
        return {"status": "ok", "reason": "", "applied": []}

    def preposition_movej(self, deg7, speed=10, radius=0, block=0, trajectory_connect=0):
        self.calls.append(("movej", [float(v) for v in deg7], speed, radius, block))
        self.moves.append([float(v) for v in deg7])
        if self.move_status != "ok":
            return {"status": self.move_status, "reason": "rm_movej 返回 1"}
        # 去程：按目标角更新（含跟随误差）；回程：回到原位（带回程误差）
        target = [float(v) for v in deg7]
        if len(self.moves) == 1:
            self.joints = [target[0] + self.follow_error_deg] + list(target[1:])
        else:
            self.joints = [target[0] + self.return_error_deg] + list(target[1:])
        return {"status": "ok", "reason": ""}

    def transmit_joint(self, deg7, follow=False, trajectory_mode=1, radio=30, expand=0.0):
        """关节角透传（非阻塞）—— 假实现：立即到位。"""
        self.calls.append(("canfd", [float(v) for v in deg7], follow, trajectory_mode, radio))
        self.moves.append([float(v) for v in deg7])
        if self.move_status != "ok":
            return {"status": self.move_status, "reason": "rm_movej_canfd 返回 1"}
        target = [float(v) for v in deg7]
        if len(self.moves) == 1:
            self.joints = [target[0] + self.follow_error_deg] + list(target[1:])
        else:
            self.joints = [target[0] + self.return_error_deg] + list(target[1:])
        return {"status": "ok", "reason": ""}

    def stop(self):
        self.calls.append(("stop",))
        return {"status": "ok", "reason": ""}

    def close(self):
        self.closed = True


def _factory(link, recorder=None):
    def _make(**kwargs):
        if recorder is not None:
            recorder.append(kwargs)
        return link, None
    return _make


def _ok_snapshot(**overrides):
    return overrides


# ------------------------------------------------------------------ 包络
def test_envelope_rejects_oversized_delta(probe):
    ok, reason = probe.check_envelope(0, 5.0, 2)
    assert not ok and "1.0" in reason


def test_envelope_rejects_oversized_speed(probe):
    ok, reason = probe.check_envelope(0, 0.3, 50)
    assert not ok and "10" in reason


def test_envelope_rejects_bad_joint_index(probe):
    ok, reason = probe.check_envelope(9, 0.3, 2)
    assert not ok and "越界" in reason


def test_envelope_accepts_minimal_motion(probe):
    ok, reason = probe.check_envelope(0, 0.3, 2)
    assert ok and reason == ""


def test_plan_target_touches_single_joint(probe):
    target = probe.plan_target([1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0], 2, 0.3)
    assert target[2] == pytest.approx(3.3)
    assert target[:2] == [1.0, 2.0]
    assert target[3:] == [4.0, 5.0, 6.0, 7.0]


# ------------------------------------------------------------------ 前置检查
def _limits():
    return {"joint1": {"lower": -3.106, "upper": 3.106}}


def test_preflight_flags_arm_error(probe):
    snap = {"arm_err": [1], "joint_err_code": [0] * 7, "joint_en_flag": [1] * 7,
            "controller_trajectory_type_name": "RM_NO_PLANNING_E",
            "joint_deg": [0.0] * 7}
    checks = {c["name"]: c for c in probe.preflight_checks(snap, _limits(), 0, 0.3)}
    assert not checks["arm_err_empty"]["ok"]


def test_preflight_accepts_zero_arm_error_code(probe):
    """厂商 err[] 带长度、值为 0 = 无错误 → 不得误判。"""
    snap = {"arm_err": [0], "joint_err_code": [0] * 7, "joint_en_flag": [1] * 7,
            "controller_trajectory_type_name": "RM_NO_PLANNING_E",
            "joint_deg": [0.0] * 7}
    checks = {c["name"]: c for c in probe.preflight_checks(snap, _limits(), 0, 0.3)}
    assert checks["arm_err_empty"]["ok"]


def test_preflight_flags_joint_error_code(probe):
    snap = {"arm_err": [], "joint_err_code": [0, 0, 7] + [0] * 4,
            "joint_en_flag": [1] * 7,
            "controller_trajectory_type_name": "RM_NO_PLANNING_E",
            "joint_deg": [0.0] * 7}
    checks = {c["name"]: c for c in probe.preflight_checks(snap, _limits(), 0, 0.3)}
    assert not checks["joint_err_code_zero"]["ok"]


def test_preflight_flags_disabled_joint(probe):
    snap = {"arm_err": [], "joint_err_code": [0] * 7, "joint_en_flag": [1, 0] + [1] * 5,
            "controller_trajectory_type_name": "RM_NO_PLANNING_E",
            "joint_deg": [0.0] * 7}
    checks = {c["name"]: c for c in probe.preflight_checks(snap, _limits(), 0, 0.3)}
    assert not checks["all_joints_enabled"]["ok"]


def test_preflight_flags_busy_controller(probe):
    """控制器正在执行别的轨迹 → 绝不允许插队运动。"""
    snap = {"arm_err": [], "joint_err_code": [0] * 7, "joint_en_flag": [1] * 7,
            "controller_trajectory_type_name": "RM_MOVE_J_E",
            "joint_deg": [0.0] * 7}
    checks = {c["name"]: c for c in probe.preflight_checks(snap, _limits(), 0, 0.3)}
    assert not checks["controller_idle"]["ok"]


def test_preflight_flags_target_beyond_limit(probe):
    """目标位姿离限位太近（余量不足）→ 拒绝。"""
    lim = {"joint1": {"lower": -0.02, "upper": 3.106}}   # ≈ -1.15°
    snap = {"arm_err": [], "joint_err_code": [0] * 7, "joint_en_flag": [1] * 7,
            "controller_trajectory_type_name": "RM_NO_PLANNING_E",
            "joint_deg": [0.0] * 7}
    checks = {c["name"]: c for c in probe.preflight_checks(snap, lim, 0, -0.3)}
    assert not checks["target_within_limits"]["ok"]
    assert checks["within_limits"]["ok"]   # 当前位姿本身仍在安全区


def test_preflight_flags_current_position_beyond_limit(probe):
    """当前位姿已经贴着限位 → 拒绝（连回程余量都没有）。"""
    lim = {"joint1": {"lower": -3.106, "upper": 0.02}}   # ≈ +1.15°
    snap = {"arm_err": [], "joint_err_code": [0] * 7, "joint_en_flag": [1] * 7,
            "controller_trajectory_type_name": "RM_NO_PLANNING_E",
            "joint_deg": [1.0] * 7}
    checks = {c["name"]: c for c in probe.preflight_checks(snap, lim, 0, 0.3)}
    assert not checks["within_limits"]["ok"]


# ------------------------------------------------------------------ 回读判定
def test_evaluate_move_detects_arrival(probe):
    before = [0.0] * 7
    target = [0.3] + [0.0] * 6
    res = probe.evaluate_move(before, list(target), target, 0)
    assert res["ok"] and res["arrival_error_deg"] == pytest.approx(0.0)


def test_evaluate_move_detects_drift(probe):
    before = [0.0] * 7
    target = [0.3] + [0.0] * 6
    measured = [0.3, 0.5] + [0.0] * 5
    res = probe.evaluate_move(before, measured, target, 0)
    assert not res["ok"] and res["max_drift_deg"] == pytest.approx(0.5)


def test_evaluate_return_detects_residual(probe):
    before = [0.0] * 7
    res = probe.evaluate_return(before, [0.0, 0.0, 0.0, 0.9] + [0.0] * 3)
    assert not res["ok"] and res["worst_joint"] == 4


# ------------------------------------------------------------------ 端到端（假句柄）
def test_probe_happy_path_moves_out_and_back(probe):
    link = FakeLink()
    report = probe.run_probe(arm="left", ip="192.168.1.18", joint_index=0,
                              delta_deg=0.3, speed_pct=2,
                              link_factory=_factory(link))
    assert report["status"] == "ok", report
    names = [p["name"] for p in report["phases"]]
    assert names == ["snapshot", "preflight", "arm_safety", "move_out",
                     "verify_out", "move_back", "verify_back"]
    # 只动了关节 0，且去了 0.3° 又回到原位
    assert len(link.moves) == 2
    assert link.moves[0][0] == pytest.approx(0.3)
    assert link.moves[1][0] == pytest.approx(0.0)
    # 任何关节都没有被停机族打断
    assert ("stop",) not in link.calls
    # 安全开关确实武装了
    assert ("set_safety", 8, True) in link.calls


def test_probe_precheck_only_never_moves(probe):
    link = FakeLink()
    report = probe.run_probe(arm="left", ip="192.168.1.18", precheck_only=True,
                              link_factory=_factory(link))
    assert report["status"] == "ok"
    assert link.moves == []
    assert not any(c[0] == "movej" for c in link.calls)


def test_probe_refuses_when_preflight_fails(probe):
    """前置检查不通过 → 不许发出任何运动指令（也不需要急停，因为根本没动）。"""
    link = FakeLink(snapshot_overrides={"joint_err_code": [0, 0, 5] + [0] * 4})
    report = probe.run_probe(arm="left", ip="192.168.1.18",
                              link_factory=_factory(link))
    assert report["status"] == "error"
    assert link.moves == []
    assert not any(c[0] == "movej" for c in link.calls)


def test_probe_triggers_estop_when_move_fails(probe):
    link = FakeLink(move_status="error")
    report = probe.run_probe(arm="left", ip="192.168.1.18",
                              link_factory=_factory(link))
    assert report["status"] == "error"
    assert ("stop",) in link.calls
    phases = {p["name"]: p for p in report["phases"]}
    assert phases["estop"]["status"] == "ok"


def test_probe_triggers_estop_on_arrival_miss(probe):
    """回读不到位 → 急停，且不允许继续回程。"""
    link = FakeLink(follow_error_deg=2.0)
    report = probe.run_probe(arm="left", ip="192.168.1.18",
                              link_factory=_factory(link))
    assert report["status"] == "error"
    assert ("stop",) in link.calls
    assert len(link.moves) == 1  # 只发了去程，回程没有继续


def test_probe_refuses_oversized_delta_before_any_sdk_call(probe):
    link = FakeLink()
    report = probe.run_probe(arm="left", ip="192.168.1.18", delta_deg=30.0,
                              link_factory=_factory(link))
    assert report["status"] == "error"
    assert link.calls == []  # 一次 SDK 调用都没有
    assert report["phases"][0]["name"] == "envelope"


def test_probe_dry_run_makes_no_sdk_call(probe):
    link = FakeLink()
    report = probe.run_probe(arm="left", ip="192.168.1.18", dry_run=True,
                              link_factory=_factory(link))
    assert report["status"] == "dry_run"
    assert link.calls == []


def test_probe_closes_link(probe):
    link = FakeLink()
    probe.run_probe(arm="left", ip="192.168.1.18", link_factory=_factory(link))
    assert link.closed


def test_probe_canfd_channel_uses_passthrough(probe):
    """canfd 通道：只走 rm_movej_canfd，不发 rm_movej。"""
    link = FakeLink()
    report = probe.run_probe(arm="left", ip="192.168.1.18", channel="canfd",
                              link_factory=_factory(link))
    assert report["status"] == "ok", report
    assert [c[0] for c in link.calls if c[0] in ("movej", "canfd")] == ["canfd", "canfd"]
    assert link.moves[0][0] == pytest.approx(0.3)
    assert link.moves[1][0] == pytest.approx(0.0)


def test_probe_canfd_channel_triggers_estop_on_miss(probe):
    link = FakeLink(follow_error_deg=2.0)
    report = probe.run_probe(arm="left", ip="192.168.1.18", channel="canfd",
                              move_timeout_s=0.3, link_factory=_factory(link))
    assert report["status"] == "error"
    assert ("stop",) in link.calls


def test_probe_rejects_unknown_channel(probe):
    link = FakeLink()
    report = probe.run_probe(arm="left", ip="192.168.1.18", channel="movep",
                              link_factory=_factory(link))
    assert report["status"] == "error"
    assert link.calls == []
