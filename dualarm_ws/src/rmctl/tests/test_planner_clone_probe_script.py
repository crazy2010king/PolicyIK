"""S08/T5 克隆探针脚本（``scripts/rm_planner_clone_probe.py``）的用例。

分两层：

* **纯逻辑层**（秒级，可进 CI）：失败消息挑选（不能把厂商的传播消息
  ``"Error (Abort Triggered)"`` 当成失败原因）、笛卡尔可行性判据
  （带姿态约束的用例失败 ⇒ 不可行）、范围声明的措辞；
* **端到端层**（跑真实厂商内核，约 10 s）：克隆画像 + 隔离性断言 +
  笛卡尔探针的真实结论 —— 断言的是**实测事实**（隔离性成立、笛卡尔
  带姿态约束路径失败并给出 ``Failed to build graph``），不是硬编码的
  「厂商一定失败」。
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
SCRIPT_PATH = os.path.join(REPO_ROOT, "scripts", "rm_planner_clone_probe.py")


def _load_script():
    """按路径加载脚本模块（脚本不是包，用 importlib 加载）。"""
    spec = importlib.util.spec_from_file_location("rm_planner_clone_probe",
                                                  SCRIPT_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def probe():
    return _load_script()


# --------------------------------------------------------------------------
# 纯逻辑层
# --------------------------------------------------------------------------

def test_scope_statement_is_explicit(probe):
    """范围声明必须写明「不交付笛卡尔路径规划功能」+ 远期依赖。"""
    assert "不交付笛卡尔路径规划功能" in probe.SCOPE_STATEMENT
    assert "TRAC-IK" in probe.SCOPE_STATEMENT


def test_pick_informative_prefers_real_cause(probe):
    """失败消息要挑**有信息量**的那条，而不是厂商的传播消息。"""
    nodes = [
        "MotionPlanningTask: Error (Abort Triggered)",
        "ErrorTask: Error (Abort Triggered)",
        "DescartesMotionPlannerTask: Failed to build graph",
    ]
    assert probe._pick_informative(nodes) == \
        "DescartesMotionPlannerTask: Failed to build graph"


def test_pick_informative_ignores_propagation_only(probe):
    """全是传播消息时 → 空串（不拿传播消息冒充失败原因）。"""
    assert probe._pick_informative(
        ["MotionPlanningTask: Error (Abort Triggered)"]) == ""


def test_case_helpers_read_kind(probe):
    """按 kind 取用例结果（baseline / feasibility）。"""
    cases = [
        {"name": "a", "kind": "baseline", "successful": True,
         "max_tcp_deviation_m": 0.002},
        {"name": "b", "kind": "feasibility", "successful": False,
         "max_tcp_deviation_m": None},
    ]
    assert probe._case_ok(cases, "baseline") is True
    assert probe._case_ok(cases, "feasibility") is False
    assert probe._case_ok(cases, "structural") is None
    assert probe._case_deviation(cases, "baseline") == pytest.approx(0.002)
    assert probe._case_deviation(cases, "feasibility") is None


def test_tracking_tolerance_is_tight(probe):
    """笛卡尔跟踪容差必须是厘米级（1 cm）—— 放宽就等于放弃笛卡尔语义。"""
    assert 0.0 < probe.TRACKING_TOLERANCE_M <= 0.01


# --------------------------------------------------------------------------
# 端到端层（真实厂商内核）
# --------------------------------------------------------------------------

def test_probe_report_is_complete(tmp_path, probe):
    """探针跑完 → 报告齐备：克隆画像、隔离性断言、笛卡尔探针结论。"""
    out = tmp_path / "clone_probe.json"
    proc = subprocess.run(
        [sys.executable, SCRIPT_PATH, "--out", str(out), "--clones", "2"],
        cwd=REPO_ROOT, capture_output=True, text=True, timeout=600,
    )
    assert proc.returncode == 0, proc.stdout[-3000:] + proc.stderr[-3000:]
    report = json.loads(out.read_text(encoding="utf-8"))

    # -- 克隆画像 ------------------------------------------------------
    profile = report["clone_profile"]
    assert profile["clone_count"] == 2
    assert profile["links_match_base"] is True
    for clone in profile["clones"]:
        assert clone["elapsed_ms"] > 0.0
        assert clone["link_count"] == profile["base_env"]["link_count"]
    assert profile["mean_elapsed_ms"] > 0.0

    # -- 隔离性（**功能断言**：克隆体必须与原 env 完全隔离）--------------
    isolation = report["isolation"]
    assert isolation["ok"] is True, isolation
    names = {item["name"] for item in isolation["checks"]}
    assert "clone1_received_obstacle" in names
    assert "clone2_links_unchanged" in names
    assert "original_links_unchanged" in names
    assert "clone1_collides_with_obstacle" in names
    assert "clone2_collision_state_matches_original" in names
    # 障碍物真的落在 clone1 上（原 env / clone2 不受影响）
    assert isolation["contacts"]["clone1"] > 0
    assert isolation["contacts"]["clone2"] == isolation["contacts"]["original"]

    # -- 笛卡尔探针结论 ------------------------------------------------
    cart = report["cartesian_pipeline"]
    assert isinstance(cart["feasible"], bool)
    assert cart["cases"], "笛卡尔探针必须跑出用例"
    assert "conclusion" in cart and cart["conclusion"]
    if not cart["feasible"]:
        # 不可行时必须给出**有信息量**的原始错误（不是传播消息）
        assert cart["raw_error"]
        assert "Error (Abort Triggered)" not in cart["raw_error"]
    # 范围声明落在报告里
    assert report["scope_statement"] == probe.SCOPE_STATEMENT
    assert report["status"] == "ok"
