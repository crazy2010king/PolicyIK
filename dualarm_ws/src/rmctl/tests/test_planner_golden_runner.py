"""黄金场景集 runner 的**独立**验收（S09/T3）。

覆盖面（与 ``test_planner_golden.py`` 互补——那份验的是**既有断言零回归**，
这份验的是 **runner 本身的契约**）：

1. **报告同构**：runner 产出的汇总 JSON 含 S07 既有键
   （``scenario_count`` / ``p50_s`` / ``p95_s`` / ``success_rate`` /
   ``success_only``），S08 直接取用；
2. **逐场景 status 与 expect_status 一致**（真实规划服务跑出来的）；
3. **同 seed 两次请求 trajectory_digest 逐点一致**；
4. **模型不可用时该段为 blocked 而非 fail**（厂商内核缺失 = 环境问题，
   不是代码回归）——用 monkeypatch 模拟，**不改仓库场景集**；
5. **负例**：故意把某场景的 ``expect_status`` 改错 → 该场景判定失败、
   段级 fail、退出码 4（monkeypatch，不改仓库场景集）。

场景集来源**唯一**（``tests/golden/planner_scenarios.json``）；本文件
**只读**它。
"""
from __future__ import annotations

import importlib.util
import os
import sys

import pytest

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(TESTS_DIR, "..", "..", "..", ".."))
PKG_SRC = os.path.join(REPO_ROOT, "dualarm_ws", "src", "rmctl")
EXEC_SRC = os.path.join(REPO_ROOT, "dualarm_ws", "src", "rm_executor")
RM_DESC_SCRIPTS = os.path.join(REPO_ROOT, "dualarm_ws", "src",
                                "rm_description", "scripts")

for _path in (PKG_SRC, EXEC_SRC, RM_DESC_SCRIPTS):
    if os.path.isdir(_path) and _path not in sys.path:
        sys.path.insert(0, _path)

from rmctl import golden_runner as gr  # noqa: E402
from rmctl.planner_service.client import PlannerClient  # noqa: E402
from rmctl.planner_service.server import PlannerServer  # noqa: E402
from rmctl.regression import orchestrator as orch  # noqa: E402


# --------------------------------------------------------------------------
# 夹具（真实规划服务；模型现场物化 → 不依赖 gitignore 目录）
# --------------------------------------------------------------------------

@pytest.fixture(scope="module")
def scenarios():
    return gr.load_scenarios()


@pytest.fixture(scope="module")
def planner_ws(tmp_path_factory):
    return gr.prepare_workspace(str(tmp_path_factory.mktemp("runner_ws")),
                                repo_root=REPO_ROOT)


@pytest.fixture(scope="module")
def server(planner_ws, tmp_path_factory):
    sock_path = os.path.join(
        str(tmp_path_factory.mktemp("runner_sock")), "rm_planner.sock")
    srv = PlannerServer(socket_path=sock_path, ws=planner_ws,
                        worker_pool_size=1)
    srv.start()
    try:
        yield srv
    finally:
        srv.stop()


@pytest.fixture(scope="module")
def client(server, planner_ws):
    return PlannerClient(socket_path=server.socket_path, ws=planner_ws)


@pytest.fixture(scope="module")
def rows(scenarios, client):
    return gr.run_scenarios(scenarios, client)


@pytest.fixture(scope="module")
def golden_script():
    """按路径加载 ``scripts/rm_planner_golden.py``（可 monkeypatch 内部依赖）。"""
    path = os.path.join(REPO_ROOT, "scripts", "rm_planner_golden.py")
    spec = importlib.util.spec_from_file_location("rm_planner_golden_mod",
                                                  path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


# --------------------------------------------------------------------------
# 1. 报告同构
# --------------------------------------------------------------------------

def test_report_is_structurally_compatible_with_s07(rows, scenarios):
    """runner 产出的汇总 JSON 含 S07 既有键（S08 直接取用，语义不变）。"""
    report = gr.summarize(rows, failures=gr.evaluate_rows(rows, scenarios))
    assert report["version"] == 1
    assert report["scenario_count"] == len(scenarios)
    assert report["p50_s"] is not None
    assert report["p95_s"] is not None
    assert 0.0 <= report["success_rate"] <= 1.0
    only = report["success_only"]
    assert set(["count", "p50_s", "p95_s"]).issubset(only)
    assert only["count"] >= 1
    assert only["p50_s"] is not None
    assert only["p95_s"] is not None
    assert len(report["scenarios"]) == len(scenarios)
    # 契约键也在（编排器归一化要用）
    assert report["status"] in ("ok", "error")
    assert isinstance(report["failures"], list)
    assert isinstance(report["evidence"], list)


# --------------------------------------------------------------------------
# 2. 逐场景 status 一致
# --------------------------------------------------------------------------

def test_every_scenario_status_matches_expectation(rows, scenarios):
    """逐场景 status 与 expect_status 一致（真实规划服务跑出来的）。"""
    by_id = {row["id"]: row for row in rows}
    for scenario in scenarios:
        row = by_id[scenario["id"]]
        assert row["status"] == scenario["expect_status"], (
            "场景 %s: status=%s，期望 %s"
            % (scenario["id"], row["status"], scenario["expect_status"]))
    assert gr.evaluate_rows(rows, scenarios) == []


# --------------------------------------------------------------------------
# 3. 同 seed 逐点一致
# --------------------------------------------------------------------------

def test_same_seed_two_requests_are_pointwise_identical(client):
    """同 seed 两次请求 → trajectory_digest 逐点一致。"""
    result = gr.determinism_check(client)
    assert result["status_a"] == "SUCCESS", result
    assert result["status_b"] == "SUCCESS", result
    assert result["digest_a"] == result["digest_b"]
    assert result["ok"], result["reason"]


# --------------------------------------------------------------------------
# 4. 模型不可用 → blocked 而非 fail
# --------------------------------------------------------------------------

def test_model_unavailable_is_blocked_not_fail(golden_script, tmp_path,
                                               monkeypatch):
    """厂商内核不可用 → 段级 blocked（不是 fail），退出码 3。

    模拟方式：monkeypatch ``golden_runner.check_vendor`` 返回不可用
    （**不改仓库场景集、不改环境**）。
    """
    monkeypatch.setattr(gr, "check_vendor",
                        lambda ws: (False, "厂商规划内核不可用（缺 "
                                          "libstdcxx / 缺 build/model）: 模拟"))
    out_path = str(tmp_path / "blocked.json")
    code = golden_script.main(["--out", out_path])
    assert code == 3, code

    import json
    with open(out_path, "r", encoding="utf-8") as fh:
        report = json.load(fh)
    assert report["status"] == "blocked", report
    assert report["scenarios"] == []
    assert "libstdcxx" in report["reason"] or "build/model" in report["reason"]

    # 编排器归一化口径：blocked → 段级 blocked（**不是** fail）
    norm = orch.normalize_report(3, report, script_name="rm_planner_golden")
    assert norm["verdict"] == "blocked", norm
    assert norm["next_action"], norm


# --------------------------------------------------------------------------
# 5. 负例：expect_status 改错 → 段级 fail、退出码 4
# --------------------------------------------------------------------------

def test_wrong_expectation_makes_segment_fail(golden_script, tmp_path,
                                               monkeypatch, scenarios):
    """负例：故意把某场景 expect_status 改错 → 该场景失败、段级 fail。

    模拟方式：**在内存里**改场景（monkeypatch ``golden_runner.load_scenarios``
    返回改坏的副本），**不改仓库场景集**。真实跑一遍规划 → 该场景判定失败。
    """
    broken = []
    for item in scenarios:
        item = dict(item)
        if item["id"] == scenarios[0]["id"]:
            item["expect_status"] = "TIMEOUT"      # 故意写错
        broken.append(item)

    monkeypatch.setattr(gr, "load_scenarios", lambda *a, **k: broken)
    out_path = str(tmp_path / "failed.json")
    code = golden_script.main(["--out", out_path])
    assert code == 4, code

    import json
    with open(out_path, "r", encoding="utf-8") as fh:
        report = json.load(fh)
    assert report["status"] == "error", report
    assert report["failures"], report
    assert scenarios[0]["id"] in report["failures"][0]

    norm = orch.normalize_report(4, report, script_name="rm_planner_golden")
    assert norm["verdict"] == "fail", norm
    assert norm["next_action"], norm


# --------------------------------------------------------------------------
# 6. 纯函数单元（不依赖规划服务）
# --------------------------------------------------------------------------

def test_percentile_is_linear_interpolated():
    assert gr.percentile([], 0.5) is None
    assert gr.percentile([2.0], 0.95) == 2.0
    assert gr.percentile([1.0, 2.0, 3.0, 4.0], 0.5) == pytest.approx(2.5)
    assert gr.percentile([1.0, 2.0, 3.0, 4.0], 0.95) == pytest.approx(3.85)


def test_load_scenarios_rejects_unknown_id():
    with pytest.raises(ValueError):
        gr.load_scenarios(only=["no_such_scenario"])


def test_evaluate_rows_reports_each_broken_expectation():
    rows = [{
        "id": "s1", "status": "FAILED", "expect_status": "SUCCESS",
        "verify_ok": False, "expect_verify_ok": True,
        "elapsed_s": 99.0, "max_elapsed_s": 1.0, "failure_reason": "",
    }]
    failures = gr.evaluate_rows(rows, [{"id": "s1"}])
    assert len(failures) == 3, failures   # status / verify_ok / 耗时
    joined = " ".join(failures)
    assert "s1" in joined


def test_blocked_report_normalizes_to_blocked():
    report = gr.blocked_report(reason="缺 libstdcxx / 缺 build/model")
    assert report["status"] == "blocked"
    norm = orch.normalize_report(3, report, script_name="rm_planner_golden")
    assert norm["verdict"] == "blocked", norm
