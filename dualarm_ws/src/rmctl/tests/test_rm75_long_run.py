"""``scripts/rm75_long_run.py``（真机 600 s 长跑工具）的**离线**用例。

为什么测：这个工具的退出码语义（0 达标 / 3 前置检查不过 / 4 未达标）是
S10 真机结论的**唯一入口**，一旦被改坏，后面的结案工件就可能把
"没跑成"说成 "跑成了"。另外"用 fake/离线结果冒充真机结论"的路径必须被
测试挡住——所以这里同时钉死：

* ``build_trajectory`` 的形状（点数 / 关节名 / rad 单位 / 只有指定关节动）；
* 前置检查不过 → 退出码 3，且**执行器一次都没被调用**、不产生轨迹文件；
* 执行器 status=ok → 退出码 0 且证据字段齐全；
* 执行器 status!=ok → 退出码 4；
* 注入的假执行器产出的证据 mode **不得**标成 ``hardware``。

全部用**假探针 + 假执行器**注入 → 不连真机、不发任何运动指令。
"""
from __future__ import annotations

import importlib.util
import json
import math
import os
import sys
import types

import pytest

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(TESTS_DIR, "..", "..", "..", ".."))
SCRIPT_PATH = os.path.join(REPO_ROOT, "scripts", "rm75_long_run.py")


def _load_script():
    spec = importlib.util.spec_from_file_location("rm75_long_run_under_test",
                                                  SCRIPT_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def script():
    return _load_script()


# ------------------------------------------------------------------ 假依赖
JOINTS_DEG = [0.0, 42.0, 19.0, -22.0, -1.0, 5.0, -200.0]


def _probe_ok(**kwargs):
    return {
        "status": "ok",
        "reason": "",
        "phases": [
            {"name": "snapshot", "status": "ok",
             "data": {"snapshot": {"joint_deg": list(JOINTS_DEG)}}},
            {"name": "preflight", "status": "ok", "data": {"checks": []}},
        ],
    }


def _probe_fail(**kwargs):
    return {"status": "blocked", "reason": "7 轴未使能", "phases": []}


def _probe_no_joints(**kwargs):
    """前置检查过、但没读到 7 个关节角（无法生成轨迹）。"""
    return {"status": "ok", "reason": "", "phases": [
        {"name": "snapshot", "status": "ok", "data": {"snapshot": {}}}]}


class FakeExecutor(object):
    """假执行器：记录调用次数，绝不发任何运动指令。"""

    def __init__(self, report=None):
        self.calls = []
        self.report = report if report is not None else {
            "status": "ok", "reason": "", "metrics": {"ticks": 60000},
            "events": [{"type": "tick"}],
        }

    def execute(self, **kwargs):
        self.calls.append(kwargs)
        return json.loads(json.dumps(self.report))


def _install(monkeypatch, probe_result=None, executor=None):
    """注入假探针 / 假执行器到 sys.modules（脚本优先复用已注入模块）。"""
    probe = types.ModuleType("rm75_safe_probe")
    probe.run_probe = probe_result or _probe_ok
    monkeypatch.setitem(sys.modules, "rm75_safe_probe", probe)

    fake_executor = executor or FakeExecutor()
    cli = types.ModuleType("rm_executor.cli")
    cli.__file__ = os.path.join(TESTS_DIR, "fake_rm_executor_cli.py")
    cli.execute = fake_executor.execute
    monkeypatch.setitem(sys.modules, "rm_executor.cli", cli)
    return fake_executor


# ------------------------------------------------------------------ 轨迹形状
def test_trajectory_point_count_and_joint_names(script):
    traj = script.build_trajectory(JOINTS_DEG, 0, 0.2, duration_s=2.0,
                                    period_s=0.010)
    assert len(traj["times"]) == round(2.0 / 0.010) + 1
    assert len(traj["points"]) == len(traj["times"])
    assert traj["joint_names"] == ["joint%d" % (i + 1) for i in range(7)]


def test_trajectory_points_are_radians(script):
    traj = script.build_trajectory(JOINTS_DEG, 0, 0.2, duration_s=1.0,
                                    period_s=0.010)
    assert traj["units"] == {"times": "s", "points": "rad"}
    assert traj["points"][0][1] == pytest.approx(math.radians(JOINTS_DEG[1]))
    # 数值量级是 rad（不是 deg）：-1° 的关节在 rad 下远小于 1
    assert abs(traj["points"][0][4]) == pytest.approx(abs(math.radians(-1.0)))
    assert abs(traj["points"][0][4]) < 1.0


def test_only_selected_joint_moves(script):
    traj = script.build_trajectory(JOINTS_DEG, 2, 0.3, duration_s=1.0,
                                    period_s=0.010)
    offsets = [p[2] - math.radians(JOINTS_DEG[2]) for p in traj["points"]]
    assert max(offsets) == pytest.approx(math.radians(0.3), abs=1e-9)
    for i in (0, 1, 3, 4, 5, 6):
        for row in traj["points"]:
            assert row[i] == pytest.approx(math.radians(JOINTS_DEG[i]))


def test_times_strictly_increase_with_fixed_step(script):
    period = 0.010
    traj = script.build_trajectory(JOINTS_DEG, 0, 0.2, duration_s=1.0,
                                    period_s=period)
    times = traj["times"]
    assert all(b > a for a, b in zip(times, times[1:]))
    for a, b in zip(times, times[1:]):
        assert round(b - a, 6) == pytest.approx(period, abs=1e-6)
    assert times[0] == 0.0


# ------------------------------------------------------------------ 退出码语义
def test_precheck_failure_returns_3_and_never_calls_executor(script, tmp_path,
                                                             monkeypatch):
    executor = _install(monkeypatch, probe_result=_probe_fail)
    out = tmp_path / "long_run.json"
    rc = script.main(["--ws", str(tmp_path), "--out", str(out),
                      "--duration-s", "1.0"])
    assert rc == 3
    assert executor.calls == [], "前置检查不过时执行器一次都不能被调用"
    assert not out.exists()
    traj_path = tmp_path / "build" / "reports" / "validation" / \
        "long_run_trajectory.json"
    assert not traj_path.exists(), "前置检查不过时不得产生轨迹文件"


def test_missing_joint_snapshot_returns_3(script, tmp_path, monkeypatch):
    executor = _install(monkeypatch, probe_result=_probe_no_joints)
    rc = script.main(["--ws", str(tmp_path), "--duration-s", "1.0"])
    assert rc == 3
    assert executor.calls == []


def test_executor_ok_returns_0_with_complete_evidence(script, tmp_path,
                                                       monkeypatch):
    executor = _install(monkeypatch)
    out = tmp_path / "long_run.json"
    rc = script.main(["--ws", str(tmp_path), "--out", str(out),
                      "--duration-s", "1.0"])
    assert rc == 0
    assert len(executor.calls) == 1
    assert executor.calls[0]["mode"] == "live"
    assert os.path.exists(executor.calls[0]["trajectory_path"])
    evidence = json.loads(out.read_text(encoding="utf-8"))
    for field in ("schema", "mode", "status", "reason", "events",
                  "protection_stop", "stream_lost", "wall_s", "planned_points",
                  "metrics"):
        assert field in evidence, "证据缺字段 %s" % field
    assert evidence["status"] == "ok"
    assert evidence["planned_points"] == round(1.0 / 0.010) + 1
    assert evidence["protection_stop"] is False
    assert evidence["stream_lost"] is False


def test_executor_not_ok_returns_4(script, tmp_path, monkeypatch):
    executor = _install(monkeypatch, executor=FakeExecutor({
        "status": "error", "reason": "保护停", "metrics": {"ticks": 12},
        "events": [{"type": "protection_stop"}],
    }))
    out = tmp_path / "long_run.json"
    rc = script.main(["--ws", str(tmp_path), "--out", str(out),
                      "--duration-s", "1.0"])
    assert rc == 4
    assert len(executor.calls) == 1
    evidence = json.loads(out.read_text(encoding="utf-8"))
    assert evidence["status"] == "error"
    assert evidence["protection_stop"] is True


def test_stream_lost_event_is_reflected_in_evidence(script, tmp_path, monkeypatch):
    _install(monkeypatch, executor=FakeExecutor({
        "status": "error", "reason": "断流停机", "metrics": {},
        "events": [{"type": "feedback_stream_lost"}],
    }))
    out = tmp_path / "long_run.json"
    rc = script.main(["--ws", str(tmp_path), "--out", str(out),
                      "--duration-s", "1.0"])
    assert rc == 4
    evidence = json.loads(out.read_text(encoding="utf-8"))
    assert evidence["stream_lost"] is True
    assert "feedback_stream_lost" in evidence["events"]


# ------------------------------------------------------------------ 诚实性
def test_fake_executor_evidence_is_not_labelled_hardware(script, tmp_path,
                                                          monkeypatch):
    """假执行器产出的证据 **mode 不得**是 hardware（禁止冒充真机结论）。"""
    _install(monkeypatch)
    out = tmp_path / "long_run.json"
    script.main(["--ws", str(tmp_path), "--out", str(out), "--duration-s", "1.0"])
    evidence = json.loads(out.read_text(encoding="utf-8"))
    assert evidence["mode"] in script.MODES
    assert evidence["mode"] != "hardware"


def test_real_executor_module_is_labelled_hardware(script):
    """真实 rm_executor.cli 模块 → mode=hardware（生产语义不变）。"""
    real_cli = importlib.import_module("rm_executor.cli")
    assert script._evidence_mode(real_cli) == "hardware"


def test_mode_vocabulary_is_explicit(script):
    assert script.MODES == ("hardware", "offline", "fake")
