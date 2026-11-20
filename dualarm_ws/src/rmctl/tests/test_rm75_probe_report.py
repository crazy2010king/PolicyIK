"""``scripts/rm75_probe_report.py``（报告速览）的**纯逻辑**用例。

为什么测：它是"看证据"的那一层 —— 一旦把关键判定数字打错（到位误差、
漂移、回程偏差），现场就会按错数字做决策。它**只读文件、不碰硬件**，
因此可以完全离线覆盖。

覆盖：正常报告的摘要行、blocked / error 的退出码、坏文件的诚实报错、
以及「不得伪造数值」—— 报告里没有的字段就打印 `None`，不编数字。
"""
from __future__ import annotations

import importlib.util
import json
import os
import subprocess
import sys

import pytest

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(TESTS_DIR, "..", "..", "..", ".."))
SCRIPT_PATH = os.path.join(REPO_ROOT, "scripts", "rm75_probe_report.py")


def _load_script():
    spec = importlib.util.spec_from_file_location("rm75_probe_report", SCRIPT_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def reporter():
    return _load_script()


def _report(status="ok", phases=None):
    return {
        "schema": "rmctl.rm75_safe_probe/1",
        "status": status,
        "arm": "left",
        "endpoint": {"ip": "192.168.1.18", "port": 8080},
        "params": {"channel": "movej", "joint_index": 0, "delta_deg": 0.3,
                   "speed_pct": 2},
        "phases": phases if phases is not None else [
            {"name": "snapshot", "status": "ok", "detail": "读到 7 个关节角",
             "data": {"snapshot": {"joint_deg": [0.0] * 7,
                                   "joint_current_mA": [0.0] * 7,
                                   "joint_temperature_c": [30.0] * 7}}},
            {"name": "verify_out", "status": "ok", "detail": "到位误差 0.045°",
             "data": {"arrival_error_deg": 0.045, "arrival_tol_deg": 0.2,
                      "max_drift_deg": 0.004, "drift_tol_deg": 0.1}},
            {"name": "verify_back", "status": "ok", "detail": "回程最大偏差 0.045°",
             "data": {"max_return_error_deg": 0.045, "tol_deg": 0.2,
                      "worst_joint": 1}},
        ],
        "reason": "控制链路验证通过（最小动作）",
        "next_action": "进入下一阶段",
    }


def test_summary_contains_key_numbers(reporter):
    lines = "\n".join(reporter.summarize(_report()))
    assert "0.045" in lines
    assert "arrival_error_deg" in lines
    assert "max_drift_deg" in lines
    assert "max_return_error_deg" in lines
    assert "movej" in lines


def test_summary_does_not_fabricate_missing_numbers(reporter):
    """报告里没有的字段 → 不得编数字出来。"""
    lines = "\n".join(reporter.summarize(_report(phases=[])))
    assert "arrival_error_deg" not in lines
    assert "joint_deg" not in lines


def test_cli_exit_codes(tmp_path):
    ok_path = tmp_path / "ok.json"
    ok_path.write_text(json.dumps(_report()), encoding="utf-8")
    proc = subprocess.run([sys.executable, SCRIPT_PATH, str(ok_path)],
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    assert proc.returncode == 0, proc.stderr
    assert "arrival_error_deg" in proc.stdout

    bad_path = tmp_path / "bad.json"
    bad_path.write_text(json.dumps(_report(status="error")), encoding="utf-8")
    proc = subprocess.run([sys.executable, SCRIPT_PATH, str(bad_path)],
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    assert proc.returncode == 4

    broken_path = tmp_path / "broken.json"
    broken_path.write_text("{ not json", encoding="utf-8")
    proc = subprocess.run([sys.executable, SCRIPT_PATH, str(broken_path)],
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    assert proc.returncode == 4
    assert "读取失败" in proc.stdout


def test_cli_reads_real_report_shape(tmp_path):
    """用探针真实报告结构（snapshot.data.snapshot）读一遍。"""
    report = _report()
    path = tmp_path / "r.json"
    path.write_text(json.dumps(report, ensure_ascii=False), encoding="utf-8")
    proc = subprocess.run([sys.executable, SCRIPT_PATH, str(path)],
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    assert proc.returncode == 0
    assert "关节: 1" in proc.stdout


# --------------------------------------------------------------- 限值下发工具
def test_limit_downlink_scales_speed_and_keeps_execution_range():
    """下发目标：速度按倍率收紧、加速度按倍率收紧、位置用**执行域**。"""
    import importlib.util
    path = os.path.join(REPO_ROOT, "scripts", "rm75_limit_downlink.py")
    spec = importlib.util.spec_from_file_location("rm75_limit_downlink", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)

    defaults = {"status": "placeholder", "source": "joint_limits.yaml (placeholder)",
                "values": {"joint1": {"max_speed": 3.1416, "max_acc": 5.2359,
                                       "min_pos": -6.283185, "max_pos": 6.283185,
                                       "jerk": 30.0}},
                "current": {"joint1": {"max_speed": 3.1416, "max_acc": 5.2359}}}
    limits = module.build_limits(defaults, speed_scale=0.2, acc_scale=0.5)
    item = limits["values"]["joint1"]
    assert item["max_speed"] == pytest.approx(3.1416 * 0.2)
    assert item["max_acc"] == pytest.approx(5.2359 * 0.5)
    # 位置限值必须是执行域（±360°），不能被倍率改动
    assert item["min_pos"] == pytest.approx(-6.283185)
    assert item["max_pos"] == pytest.approx(6.283185)
    assert "执行域" in limits["position_source"]


def test_limit_downlink_summary_lists_return_codes():
    import importlib.util
    path = os.path.join(REPO_ROOT, "scripts", "rm75_limit_downlink.py")
    spec = importlib.util.spec_from_file_location("rm75_limit_downlink2", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)

    lines = module.summarize({"applied": [
        {"symbol": "rm_set_joint_drive_max_speed", "joint": "joint1",
         "value": 36.0, "rc": 0}]})
    assert "rm_set_joint_drive_max_speed" in lines[0] and "rc=0" in lines[0]
    assert module.summarize({}) == ["applied 为空（dry-run 或未下发）"]


# --------------------------------------------------------------- 长跑轨迹
def test_long_run_trajectory_shape_is_micro_and_returns():
    """长跑轨迹：只有 1 个关节偏离，且最终回到初始位姿。"""
    import importlib.util
    path = os.path.join(REPO_ROOT, "scripts", "rm75_long_run.py")
    spec = importlib.util.spec_from_file_location("rm75_long_run", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)

    base = [0.0, 42.0, 19.0, -22.0, -1.0, 5.0, -200.0]
    traj = module.build_trajectory(base, joint_index=0, delta_deg=0.2,
                                   duration_s=20.0, period_s=0.010)
    times, points = traj["times"], traj["points"]
    assert len(times) == len(points) == 2001
    assert all(b > a for a, b in zip(times, times[1:]))
    # 只有 joint1 偏离，其余恒定
    for row in points:
        for i in range(1, 7):
            assert row[i] == pytest.approx(base[i] * 3.141592653589793 / 180.0)
    offsets = [(row[0] - base[0] * 3.141592653589793 / 180.0) for row in points]
    assert max(offsets) == pytest.approx(0.2 * 3.141592653589793 / 180.0, abs=1e-9)
    assert offsets[-1] == pytest.approx(0.0, abs=1e-9)     # 回到初始位姿
