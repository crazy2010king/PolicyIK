"""rm_planner **黄金场景集**的自动验收（S07/T5；S09/T3 薄封装）。

**薄封装纪律（S09/T3 起）**
--------------------------

执行逻辑**唯一**地住在 :mod:`rmctl.golden_runner`（场景加载 → 构造请求 →
起规划服务 → 逐场景执行 → 断言 → 汇总 → 写报告）。本文件只做三件事：

1. 提供 pytest 夹具（临时工作区 + 规划服务 + 客户端）；
2. 调 ``golden_runner`` 的**同一份**实现跑一遍场景集；
3. 保留 S07 的**既有断言**（status / 四道校验 / 墙钟耗时 / 同种子逐点一致
   / 报告结构）—— 断言口径**不放宽**，场景集的 ``expect_*`` 冻结值
   **不修改**。

一键回归编排器走脚本入口 ``scripts/rm_planner_golden.py``（同一份实现），
因此「pytest 全绿」与「一键回归该段 pass」**必然一致**。

场景集本体 = ``tests/golden/planner_scenarios.json``（**版本化、git 跟踪、
可 diff**）；报告落到 ``build/reports/s07_planner_golden.json``（运行时产物，
gitignore），供 **S08（性能 p95）直接取用**——S08 不另造一份基准。

夹具纪律：模型在 pytest 临时目录里用 ``build_model.build_model()`` 物化
（绝对路径 mesh）；规划配置从仓库复制进临时工作区。**不依赖 gitignore
目录**（``.gsd/``）。
"""
from __future__ import annotations

import os
import sys

import pytest

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(TESTS_DIR, "..", "..", "..", ".."))
PKG_SRC = os.path.join(REPO_ROOT, "dualarm_ws", "src", "rmctl")
RM_DESC_SRC = os.path.join(REPO_ROOT, "dualarm_ws", "src", "rm_description")
RM_DESC_SCRIPTS = os.path.join(RM_DESC_SRC, "scripts")
PLANNER_CFG_DIR = os.path.join(REPO_ROOT, "workcell", "planner")

for _path in (PKG_SRC, RM_DESC_SCRIPTS):
    if _path not in sys.path:
        sys.path.insert(0, _path)

from rmctl import golden_runner as gr  # noqa: E402
from rmctl.planner_service.client import PlannerClient  # noqa: E402
from rmctl.planner_service.server import PlannerServer  # noqa: E402

#: 场景集文件（git 跟踪；**不是** .gitignore 目录里的东西）。
SCENARIOS_PATH = os.path.join(TESTS_DIR, "golden", "planner_scenarios.json")

#: 汇总报告落盘位置（运行时产物，gitignore；供 S08 直接取用）。
REPORT_PATH = os.path.join(REPO_ROOT, "build", "reports",
                            "s07_planner_golden.json")

#: 场景集里要求的最小场景数（防止有人把场景集删到只剩一两条还「全绿」）。
MIN_SCENARIOS = gr.MIN_SCENARIOS

#: 场景集必须覆盖的结局（每一种至少一条）——防止场景集退化成「只有成功」。
REQUIRED_STATUSES = gr.REQUIRED_STATUSES


# --------------------------------------------------------------------------
# 夹具
# --------------------------------------------------------------------------

@pytest.fixture(scope="session")
def scenarios():
    """读场景集（版本化 JSON；校验规则与 S07 一致）。"""
    return gr.load_scenarios(SCENARIOS_PATH)


@pytest.fixture(scope="session")
def planner_ws(tmp_path_factory):
    """临时工作区：物化好的绝对路径模型 + 复制进来的规划配置。"""
    return gr.prepare_workspace(str(tmp_path_factory.mktemp("golden")),
                                repo_root=REPO_ROOT)


@pytest.fixture(scope="session")
def server(planner_ws, tmp_path_factory):
    """起在 session 级临时 socket 上的规划服务（整个场景集共用一个）。"""
    sock_path = os.path.join(
        str(tmp_path_factory.mktemp("golden_sock")), "rm_planner.sock")
    srv = PlannerServer(socket_path=sock_path, ws=planner_ws,
                        worker_pool_size=1)
    srv.start()
    try:
        yield srv
    finally:
        srv.stop()


@pytest.fixture(scope="session")
def client(server, planner_ws):
    """对着上面那个真实服务的客户端。"""
    return PlannerClient(socket_path=server.socket_path, ws=planner_ws)


@pytest.fixture(scope="session")
def golden_results(scenarios, client):
    """把**全部场景**跑一遍（session 级缓存），返回逐场景结果行。

    只跑一次 → 两条用例（逐场景断言 / 汇总报告）消费**同一份**结果，
    既避免重复规划（每条请求都要 spawn 一个 worker 子进程），也让报告里的
    数字与断言里的数字**必然一致**。
    """
    return gr.run_scenarios(scenarios, client)


# --------------------------------------------------------------------------
# 助手（薄封装：转发到 golden_runner）
# --------------------------------------------------------------------------

def build_request(scenario, plan_id):
    """场景 → 契约层请求字典（**不补业务缺省**：场景写什么就发什么）。"""
    return gr.build_request(scenario, plan_id)


def percentile(values, fraction):
    """线性插值分位数（p50 / p95）。"""
    return gr.percentile(values, fraction)


# --------------------------------------------------------------------------
# 用例
# --------------------------------------------------------------------------

def test_golden_scenarios_match_expectations(scenarios, golden_results):
    """每个场景经服务跑一遍：status / 四道校验 / 耗时 三项断言。

    这是本任务的主验收面。断言口径与 S07/T5 **逐条相同**。
    """
    assert len(golden_results) == len(scenarios)
    by_id = {row["id"]: row for row in golden_results}
    for scenario in scenarios:
        row = by_id[scenario["id"]]
        expected_status = scenario["expect_status"]
        assert row["status"] == expected_status, (
            "场景 %s: status=%s，期望 %s；failure_reason=%r"
            % (scenario["id"], row["status"], expected_status,
               row["failure_reason"])
        )

        # -- 四道校验（TIMEOUT 没有 verification：规划被服务端 SIGKILL）--
        expected_verify = scenario.get("expect_verify_ok")
        if expected_verify is not None:
            assert row["verify_ok"] is expected_verify, (
                "场景 %s: verification.ok=%r，期望 %r；failure_reason=%r"
                % (scenario["id"], row["verify_ok"], expected_verify,
                   row["failure_reason"])
            )

        # -- 失败原因指名（可选项）--
        needle = scenario.get("expect_failure_reason_contains")
        if needle:
            assert needle in row["failure_reason"], (
                "场景 %s: failure_reason=%r，未包含 %r"
                % (scenario["id"], row["failure_reason"], needle)
            )

        # -- 墙钟耗时上限--
        assert row["elapsed_s"] <= float(scenario["max_elapsed_s"]), (
            "场景 %s: 耗时 %.2f s 超过上限 %.2f s"
            % (scenario["id"], row["elapsed_s"],
               float(scenario["max_elapsed_s"]))
        )


def test_golden_report_is_written(scenarios, golden_results):
    """汇总报告落到 ``build/reports/s07_planner_golden.json``（S08 直接取用）。

    报告含 **p50 / p95 / 成功率**：p95 的分母是**全部**场景（含失败场景的
    墙钟），因为 S08 要的是「端到端请求延迟」而不是「只统计成功请求」。
    """
    report = _write_report(golden_results, scenarios)
    assert os.path.isfile(REPORT_PATH), REPORT_PATH
    with open(REPORT_PATH, "r", encoding="utf-8") as fh:
        written = fh.read()
    assert report["version"] == 1
    assert report["scenario_count"] == len(scenarios)
    assert report["p50_s"] is not None
    assert report["p95_s"] is not None
    assert 0.0 <= report["success_rate"] <= 1.0
    assert report["scenarios"], "报告里必须逐场景有行（S08 要按场景看）"
    assert report["success_rate"] == 1.0, report["scenarios"]
    # S08 直接消费的两组延迟都在
    assert report["success_only"]["count"] >= 1, report
    assert report["success_only"]["p50_s"] is not None, report
    assert report["success_only"]["p95_s"] is not None, report
    assert written


def test_golden_same_seed_is_pointwise_identical(client):
    """同请求同种子 → **逐点一致**（验收项 3 的场景面）。

    同一场景跑两遍（两次各自 spawn 独立 worker 子进程 → 进程级 RNG_setSeed
    各来一次），比对 ``trajectory_digest``：摘要相同即「逐点一致」。
    """
    result = gr.determinism_check(client)
    assert result["status_a"] == "SUCCESS", result
    assert result["status_b"] == "SUCCESS", result
    assert result["ok"], result["reason"]
    assert result["digest_a"] == result["digest_b"], "同种子两次结果不一致"


# --------------------------------------------------------------------------
# 报告
# --------------------------------------------------------------------------

def _write_report(rows, scenarios=None):
    """写汇总报告（p50 / p95 / 成功率 / 逐场景行）。

    **两组延迟分开给**：
    * ``p50_s`` / ``p95_s`` —— **全部**场景的端到端墙钟（含故意失败/超时
      场景，它们是真实请求延迟的一部分）；
    * ``success_only.*`` —— 只统计 **status == SUCCESS** 的场景。

    S08 要的是「正常请求的延迟分位数」→ 用 ``success_only``；
    「请求面整体行为」→ 用顶层的 p50/p95。两组都给，不让 S08 误读。
    """
    failures = gr.evaluate_rows(rows, scenarios)
    report = gr.summarize(rows, failures=failures)
    gr.write_report(report, REPORT_PATH)
    return report
