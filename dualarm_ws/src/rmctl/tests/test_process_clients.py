"""``rmctl.process.clients`` 的协议 / fake / live 实现测试。

覆盖（本文件是 S06/T2 的验收证据）：

* ``FakePlanner``：无 ``verify_fn`` → **blocked 且不产出任何轨迹**；有
  ``verify_fn`` → 轨迹**直接用** ``rm_executor.contract.load_trajectory``
  断言全部字段约束；笛卡尔目标 → blocked；目标互斥 / 缺目标 / 维度错 /
  校验函数非法 → blocked。
* ``FakeMotionClient``：跑一条**真轨迹** → 报告 ``status=ok`` + 顶层 7 键 +
  ``metrics`` 键数按 ``rm_executor.report.METRIC_KEYS`` 断言（不写死数字）；
  轨迹文件缺失 → blocked（不抛裸异常）；夹爪互锁闸门未判定 → 拒绝运动。
* ``LivePlanner``：厂商内核不可用 → blocked + 可粘贴 next_action（不抛裸
  异常）；规划异常 → blocked；缓存命中返回同一条轨迹且 ``cached=true``。
* ``FakeGripperClient``：动作走 ``gripper_action`` 上下文（动作后闸门 =
  ``GRIPPER_UNJUDGED``）；判定走**真实** ``gripper_state.judge``
  （grasp_ok / grasp_empty）；标定四键为 ``None`` → ``needs_param``
  （**不得**产出 grasp_ok / grasp_empty）。
* **代理阈值单一来源**：``rmctl.cli.GRIPPER_PROXY_THRESHOLDS is
  rmctl.gripper_proxy.GRIPPER_PROXY_THRESHOLDS``；``rmctl.cli._gripper_thresholds``
  与 ``effective_thresholds`` 对同一 config 给出**相同**结果。
* **职责红线**：工艺层零 SDK 直调 / 零阈值副本；import 工艺层**不**拉起
  厂商规划内核（子进程隔离断言）。

只用 git 跟踪的仓库文件 + pytest 临时目录（``tmp_path``），绝不依赖
.gitignore 目录。
"""
import inspect
import json
import os
import subprocess
import sys

import pytest

from rmctl import cli as cli_mod
from rmctl import gripper_proxy
from rmctl.process import clients as clients_mod

from rm_executor import contract as exec_contract
from rm_executor import report as exec_report


#: 一组合法的 7 关节起点/目标（度）
START_DEG = [0.0, -30.0, 60.0, 0.0, -60.0, 0.0, 0.0]
GOAL_DEG = [10.0, -20.0, 50.0, 5.0, -50.0, 10.0, 0.0]


def ok_verify(times, points):
    """一份**形状合法**的校验报告（``ok=True``）。"""
    return {
        "ok": True,
        "points": len(points),
        "checks": {
            "joint_limits": {"ok": True, "issues": []},
            "collision": {"ok": True, "issues": []},
            "continuity": {"ok": True, "issues": []},
            "velocity": {"ok": True, "issues": []},
        },
        "issues": [],
    }


def proxy_thresholds():
    """代理阈值（**唯一来源** = ``rmctl.gripper_proxy``）。"""
    thresholds, _used, _source = gripper_proxy.effective_thresholds({})
    return thresholds


@pytest.fixture
def fake_gripper_client():
    """一个带独立闸门的离线夹爪客户端。"""
    from rm_hw_bridge import gripper_state

    gate = gripper_state.MotionGate()
    return clients_mod.FakeGripperClient(thresholds=proxy_thresholds(), gate=gate), gate


# --------------------------------------------------------------------------
# FakePlanner
# --------------------------------------------------------------------------

def test_fake_planner_without_verify_fn_is_blocked_and_produces_no_trajectory():
    """无 ``verify_fn`` → blocked，且**不产出任何轨迹**（绝不合成 verify.ok）。"""
    planner = clients_mod.FakePlanner()
    res = planner.plan_segment(START_DEG, goal_deg=GOAL_DEG)
    assert res["status"] == "blocked", res
    assert "未提供校验函数" in res["reason"], res
    assert res["next_action"], res
    assert res["trajectory"] is None, res
    assert res["times"] == [] and res["points"] == [], res


def test_fake_planner_with_verify_fn_satisfies_executor_contract(tmp_path):
    """有 ``verify_fn`` → 轨迹**直接用契约函数**断言全部字段约束。"""
    planner = clients_mod.FakePlanner(verify_fn=ok_verify)
    res = planner.plan_segment(START_DEG, goal_deg=GOAL_DEG)
    assert res["status"] == "ok", res
    payload = res["trajectory"]
    assert payload is not None, res
    # 落盘后交给执行层契约函数裁决（不自己复述约束）
    path = tmp_path / "traj.json"
    clients_mod.save_trajectory(res, str(path))
    loaded = exec_contract.load_trajectory(str(path))
    assert loaded["status"] == "ok", loaded
    traj = loaded["trajectory"]
    assert traj["group"] == exec_contract.JOINT_GROUP
    assert list(traj["joint_names"]) == list(exec_contract.JOINT_NAMES)
    assert traj["units"] == exec_contract.UNITS
    assert traj["schema_version"] == exec_contract.SCHEMA_VERSION
    assert traj["verify"]["ok"] is True
    assert len(traj["times"]) == len(traj["points"])
    assert all(traj["times"][i] > traj["times"][i - 1]
               for i in range(1, len(traj["times"])))
    assert all(len(row) == exec_contract.ARM_DOF for row in traj["points"])


def test_fake_planner_verify_failure_is_blocked_without_trajectory():
    """``verify.ok=False`` → blocked 且不产出轨迹（不绕过校验）。"""
    def bad_verify(times, points):
        report = ok_verify(times, points)
        report["ok"] = False
        report["checks"]["joint_limits"] = {"ok": False, "issues": ["超限"]}
        return report

    res = clients_mod.FakePlanner(verify_fn=bad_verify).plan_segment(
        START_DEG, goal_deg=GOAL_DEG)
    assert res["status"] == "blocked", res
    assert "四道校验未通过" in res["reason"], res
    assert res["trajectory"] is None, res


def test_fake_planner_cartesian_goal_is_blocked_with_reason():
    """笛卡尔目标 → blocked 且**说明原因**（离线代理不支持笛卡尔目标）。"""
    res = clients_mod.FakePlanner(verify_fn=ok_verify).plan_segment(
        START_DEG, goal_pose=[0.3, 0.0, 0.4, 0.0, 0.0, 0.0])
    assert res["status"] == "blocked", res
    assert "笛卡尔" in res["reason"], res
    assert res["next_action"], res
    assert res["trajectory"] is None, res


def test_fake_planner_rejects_both_goals_and_missing_goal():
    """目标互斥 / 缺目标 → blocked（不猜、不选一个）。"""
    planner = clients_mod.FakePlanner(verify_fn=ok_verify)
    both = planner.plan_segment(START_DEG, goal_deg=GOAL_DEG,
                                goal_pose=[0.3, 0.0, 0.4, 0.0, 0.0, 0.0])
    assert both["status"] == "blocked", both
    assert "互斥" in both["reason"], both
    none = planner.plan_segment(START_DEG)
    assert none["status"] == "blocked", none
    assert "缺少运动目标" in none["reason"], none


def test_fake_planner_rejects_wrong_dimensionality():
    """维度错 / 非有限数 → blocked（不静默补齐）。"""
    planner = clients_mod.FakePlanner(verify_fn=ok_verify)
    short = planner.plan_segment(START_DEG[:3], goal_deg=GOAL_DEG)
    assert short["status"] == "blocked", short
    assert "非法" in short["reason"], short
    nan_goal = list(GOAL_DEG)
    nan_goal[0] = float("nan")
    bad = planner.plan_segment(START_DEG, goal_deg=nan_goal)
    assert bad["status"] == "blocked", bad


def test_fake_planner_rejects_malformed_verify_fn_result():
    """``verify_fn`` 返回非法形态 → blocked（不当成通过）。"""
    res = clients_mod.FakePlanner(verify_fn=lambda t, p: "not-a-report").plan_segment(
        START_DEG, goal_deg=GOAL_DEG)
    assert res["status"] == "blocked", res
    assert "非法" in res["reason"], res
    assert res["trajectory"] is None, res


# --------------------------------------------------------------------------
# FakeMotionClient
# --------------------------------------------------------------------------

def _write_ok_trajectory(tmp_path):
    """造一条满足执行层契约的轨迹并落盘。"""
    planner = clients_mod.FakePlanner(verify_fn=ok_verify)
    res = planner.plan_segment(START_DEG, goal_deg=GOAL_DEG)
    assert res["status"] == "ok", res
    path = tmp_path / "traj.json"
    clients_mod.save_trajectory(res, str(path))
    return str(path)


def test_fake_motion_client_runs_trajectory_and_returns_unified_report(tmp_path):
    """离线假执行 → ``rm_executor`` **统一报告**（顶层 7 键 / metrics 按契约）。"""
    path = _write_ok_trajectory(tmp_path)
    client = clients_mod.FakeMotionClient()
    report = client.run_trajectory(path)
    assert report["status"] == "ok", report
    assert set(report.keys()) == set(exec_report.TOP_LEVEL_KEYS), report
    assert report["mode"] == "fake", report
    assert len(report["metrics"]) == len(exec_report.METRIC_KEYS), report["metrics"]
    assert set(report["metrics"].keys()) == set(exec_report.METRIC_KEYS)


def test_fake_motion_client_missing_trajectory_is_blocked_not_exception(tmp_path):
    """轨迹文件缺失 → blocked（**不抛裸异常**）。"""
    report = clients_mod.FakeMotionClient().run_trajectory(str(tmp_path / "nope.json"))
    assert report["status"] != "ok", report
    assert report["reason"], report
    assert report["next_action"], report


def test_fake_motion_client_respects_gripper_interlock_gate(tmp_path, fake_gripper_client):
    """夹爪动作未判定 → 执行器**拒绝运动**（互锁生效）。"""
    gripper, gate = fake_gripper_client
    act = gripper.act("close")
    assert act["status"] == "ok", act
    assert gate.state == gate.STATE_GRIPPER_UNJUDGED, gate.state
    path = _write_ok_trajectory(tmp_path)
    report = clients_mod.FakeMotionClient(motion_gate=gate).run_trajectory(path)
    assert report["status"] != "ok", report
    evidence = " ".join(str(x) for x in report.get("evidence") or [])
    assert "GRIPPER_UNJUDGED" in evidence or "夹爪" in report.get("reason", ""), report


# --------------------------------------------------------------------------
# 夹爪客户端（判定真值 = rm_hw_bridge.gripper_state.judge）
# --------------------------------------------------------------------------

def test_fake_gripper_close_sets_gate_to_unjudged(fake_gripper_client):
    """动作后闸门 = ``GRIPPER_UNJUDGED``（动作期间 = GRIPPER_BUSY）。"""
    gripper, gate = fake_gripper_client
    assert gate.state == gate.STATE_IDLE
    res = gripper.act("close")
    assert res["status"] == "ok", res
    assert gate.state == gate.STATE_GRIPPER_UNJUDGED, gate.state


def test_fake_gripper_judge_after_close_is_grasp_ok(fake_gripper_client):
    """close 后判定 = ``grasp_ok``（走**真实** ``gripper_state.judge``）。"""
    gripper, gate = fake_gripper_client
    gripper.act("close")
    res = gripper.judge()
    assert res["result"] == "grasp_ok", res
    assert res["severity"] == "OK", res
    assert res["note"] == gripper_proxy.GRIPPER_PROXY_NOTE, res
    assert "result" in res and "severity" in res and "reason" in res
    assert "evidence" in res and "next_action" in res and "retryable" in res
    assert gate.state == gate.STATE_IDLE, gate.state


def test_fake_gripper_judge_after_open_is_grasp_empty(fake_gripper_client):
    """open 后判定 = ``grasp_empty``。"""
    gripper, _gate = fake_gripper_client
    gripper.act("close")
    gripper.act("open")
    res = gripper.judge()
    assert res["result"] == "grasp_empty", res


def test_fake_gripper_judge_without_calibration_is_needs_param():
    """标定四键为 ``None`` → ``needs_param``（**不得**产出 grasp_ok/grasp_empty）。"""
    from rm_hw_bridge import gripper_state

    uncalibrated = {
        "force_threshold": None,
        "grasp_position_window": None,
        "position_mm_per_unit": None,
        "force_scale": None,
    }
    # 注入**独立**闸门：避免污染进程内共享闸门（needs_param 路径**不**复位，
    # 共享闸门会被留在 GRIPPER_UNJUDGED → 影响后续运动）
    gate = gripper_state.MotionGate()
    gripper = clients_mod.FakeGripperClient(thresholds=uncalibrated, gate=gate)
    res = gripper.act("close")
    assert res["status"] == "ok", res
    judged = gripper.judge()
    assert judged["result"] == "needs_param", judged
    assert judged["result"] not in ("grasp_ok", "grasp_empty"), judged
    # needs_param → 保守保持阻塞（不复位闸门）
    assert gate.state == gate.STATE_GRIPPER_UNJUDGED, gate.state


def test_fake_gripper_rejects_unknown_action(fake_gripper_client):
    """未知夹爪动作 → blocked（不猜动作语义）。"""
    gripper, _gate = fake_gripper_client
    res = gripper.act("wiggle")
    assert res["status"] == "blocked", res
    assert res["next_action"], res


def test_gripper_clients_share_no_threshold_copy():
    """工艺层**零阈值副本**：阈值只来自注入 / 代理唯一来源。"""
    src = inspect.getsource(clients_mod)
    for literal in ("100.0", "10.0, 30.0", "0.05", "0.01"):
        assert literal not in src, "工艺层不得内嵌代理阈值数值: %s" % literal
    # 阈值**键名**可以出现（只作为注入字典的取用），但**数值**不得出现
    assert "GRIPPER_PROXY_THRESHOLDS[" not in src


# --------------------------------------------------------------------------
# LivePlanner
# --------------------------------------------------------------------------

def _fake_outcome(times, points):
    class _Outcome(object):
        ok = True
        message = "ok"
        pipeline = "FreespacePipeline"

    out = _Outcome()
    out.times = list(times)
    out.points = [list(row) for row in points]
    return out


def test_live_planner_blocks_when_vendor_kernel_unavailable(monkeypatch):
    """厂商规划内核不可用 → blocked + 可粘贴 next_action（**不抛裸异常**）。"""
    monkeypatch.setattr(clients_mod.LivePlanner, "_load_planner",
                        lambda self: (None, "ModuleNotFoundError: tesseract_robotics"))
    res = clients_mod.LivePlanner().plan_segment(START_DEG, goal_deg=GOAL_DEG)
    assert res["status"] == "blocked", res
    assert "规划内核不可用" in res["reason"], res
    assert res["next_action"], res
    assert res["trajectory"] is None, res


def test_live_planner_blocks_on_planner_exception(monkeypatch):
    """规划抛异常 → blocked（不把异常甩给调用者）。"""
    monkeypatch.setattr(clients_mod.LivePlanner, "_load_planner",
                        lambda self: (object(), ""))

    def boom(model, start, goal):
        raise RuntimeError("规划内核炸了")

    res = clients_mod.LivePlanner(model=object(), plan_joint=boom).plan_segment(
        START_DEG, goal_deg=GOAL_DEG)
    assert res["status"] == "blocked", res
    assert "规划失败" in res["reason"], res
    assert res["next_action"], res


def test_live_planner_cache_hit_returns_same_trajectory(monkeypatch):
    """缓存命中 → 同一条轨迹 + ``cached=true``（确定性规划，语义不变）。"""
    times = [0.0, 0.5, 1.0]
    points = [[0.0] * 7, [0.1] * 7, [0.2] * 7]
    calls = {"n": 0}

    def plan_joint(model, start, goal):
        calls["n"] += 1
        return _fake_outcome(times, points)

    def verify_with_retime(model, pts, tms):
        return ok_verify(tms, pts), list(tms), [list(r) for r in pts], None

    monkeypatch.setattr(clients_mod.LivePlanner, "_load_planner",
                        lambda self: (object(), ""))
    planner = clients_mod.LivePlanner(model=object(), plan_joint=plan_joint,
                                      verify_with_retime=verify_with_retime,
                                      cache=True)
    first = planner.plan_segment(START_DEG, goal_deg=GOAL_DEG)
    second = planner.plan_segment(START_DEG, goal_deg=GOAL_DEG)
    assert first["status"] == "ok", first
    assert first["cached"] is False, first
    assert second["status"] == "ok", second
    assert second["cached"] is True, second
    assert second["trajectory"] == first["trajectory"]
    assert calls["n"] == 1, calls


def test_live_planner_blocks_when_verify_fails(monkeypatch):
    """四道校验不过 → blocked（**不伪造** ``verify.ok=True``）。"""
    times = [0.0, 0.5, 1.0]
    points = [[0.0] * 7, [0.1] * 7, [0.2] * 7]

    def plan_joint(model, start, goal):
        return _fake_outcome(times, points)

    def verify_with_retime(model, pts, tms):
        report = ok_verify(tms, pts)
        report["ok"] = False
        report["checks"]["collision"] = {"ok": False, "issues": ["碰撞"]}
        return report, list(tms), [list(r) for r in pts], None

    monkeypatch.setattr(clients_mod.LivePlanner, "_load_planner",
                        lambda self: (object(), ""))
    res = clients_mod.LivePlanner(model=object(), plan_joint=plan_joint,
                                  verify_with_retime=verify_with_retime).plan_segment(
        START_DEG, goal_deg=GOAL_DEG)
    assert res["status"] == "blocked", res
    assert "四道校验未通过" in res["reason"], res
    assert res["trajectory"] is None, res


# --------------------------------------------------------------------------
# 代理阈值单一来源
# --------------------------------------------------------------------------

def test_proxy_thresholds_have_single_source():
    """``rmctl.cli`` 只是**再导出**：与 ``rmctl.gripper_proxy`` 同一对象。"""
    assert cli_mod.GRIPPER_PROXY_THRESHOLDS is gripper_proxy.GRIPPER_PROXY_THRESHOLDS
    assert cli_mod.GRIPPER_PROXY_NOTE == gripper_proxy.GRIPPER_PROXY_NOTE


def test_cli_thresholds_wrapper_matches_effective_thresholds():
    """``cli._gripper_thresholds`` 与 ``effective_thresholds`` 结果**逐字相同**。"""
    cases = [
        {},
        {"thresholds": {}},
        {"thresholds": {"force_threshold": 42.0}},
        {"thresholds": {"force_threshold": None, "force_scale": 0.02}},
        {"thresholds": {"grasp_position_window": [1.0, 2.0]}},
    ]
    for cfg in cases:
        thresholds, proxy_used, source = gripper_proxy.effective_thresholds(cfg)
        cli_thresholds, cli_proxy_used = cli_mod._gripper_thresholds(cfg)
        assert cli_thresholds == thresholds, cfg
        assert cli_proxy_used == proxy_used, cfg
        assert source in ("workcell_config", "proxy_default"), cfg


def test_proxy_threshold_values_are_unchanged():
    """代理阈值数值与历史一致（防止手滑改数）。"""
    assert gripper_proxy.GRIPPER_PROXY_THRESHOLDS == {
        "force_threshold": 100.0,
        "grasp_position_window": [10.0, 30.0],
        "position_mm_per_unit": 0.05,
        "force_scale": 0.01,
    }


# --------------------------------------------------------------------------
# 职责红线
# --------------------------------------------------------------------------

def test_process_layer_has_no_direct_sdk_calls():
    """工艺层**零 SDK 直调**（无 ctypes / rm_hw_bridge.generated 直调）。

    用 **AST** 判定（不看字符串：docstring 里提到这些名字不算调用）。
    """
    import ast

    tree = ast.parse(inspect.getsource(clients_mod))

    def _imported_names(node):
        names = []
        if isinstance(node, ast.Import):
            names = [a.name for a in node.names]
        elif isinstance(node, ast.ImportFrom):
            names = [node.module or ""] + [a.name for a in node.names]
        return names

    all_imports = [_imported_names(node)
                   for node in ast.walk(tree)
                   if isinstance(node, (ast.Import, ast.ImportFrom))]
    flat = [" ".join(names) for names in all_imports]
    for text in flat:
        assert "ctypes" not in text, text
        assert "rm_hw_bridge.generated" not in text, text

    # 硬件层 / 执行层 / 规划内核只允许**函数体内**惰性导入 → 顶层零依赖
    top_level = []
    for node in tree.body:
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            top_level += _imported_names(node)
    for name in top_level:
        assert "rm_hw_bridge" not in name, name
        assert "rm_executor" not in name, name
        assert "tesseract" not in name, name


def test_importing_process_clients_does_not_load_vendor_kernel():
    """import 工艺层**不拉起**厂商规划内核（子进程隔离断言）。"""
    tests_dir = os.path.dirname(os.path.abspath(__file__))
    repo_root = os.path.abspath(os.path.join(tests_dir, "..", "..", "..", ".."))
    env = dict(os.environ)
    env["PYTHONPATH"] = os.pathsep.join([
        os.path.join(repo_root, "dualarm_ws", "src", "rmctl"),
        os.path.join(repo_root, "dualarm_ws", "src", "rm_hw_bridge"),
        os.path.join(repo_root, "dualarm_ws", "src", "rm_executor"),
        env.get("PYTHONPATH", ""),
    ])
    code = (
        "import sys, json;"
        "import rmctl.process.clients as c;"
        "print(json.dumps({'tesseract': 'tesseract_robotics' in sys.modules,"
        " 'planner': 'rmctl.planner' in sys.modules,"
        " 'verify': 'rmctl.verify' in sys.modules}))"
    )
    proc = subprocess.run([sys.executable, "-c", code], env=env,
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    assert proc.returncode == 0, proc.stderr.decode("utf-8", "replace")
    info = json.loads(proc.stdout.decode("utf-8").strip().splitlines()[-1])
    assert info["tesseract"] is False, info
    assert info["planner"] is False, info
    assert info["verify"] is False, info
