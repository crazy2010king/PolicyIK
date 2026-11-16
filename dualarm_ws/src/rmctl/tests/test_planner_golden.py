"""rm_planner **黄金场景集**的自动验收（S07/T5）。

场景集本体 = ``tests/golden/planner_scenarios.json``（**版本化、git 跟踪、
可 diff**）；本文件是它的**执行器**：每个场景**经 rm_planner 服务**跑一遍
（复用 T3 的 :class:`PlannerServer` 与 T4 的 :class:`PlannerClient`），
断言三件事：

1. ``status`` 与场景声明的 ``expect_status`` 一致；
2. 四道校验结论与 ``expect_verify_ok`` 一致（``TIMEOUT`` 场景除外：它没有
   verification —— 服务端在 deadline 处 SIGKILL worker，规划根本没跑完）；
3. 墙钟耗时 ≤ ``max_elapsed_s``。

同时把 **p50 / p95 / 成功率**汇总到 ``build/reports/s07_planner_golden.json``
（运行时产物，gitignore，不进版本库），供 **S08（性能 p95）直接取用**——
S08 不另造一份基准场景。

场景集本身的设计原则（与 S08/S09 的复用契约）：

* 每条期望都是**实测过的真实行为**，不是猜的：SUCCESS / FAILED（IK 无解）/
  FAILED（四道校验未通过）/ TIMEOUT（服务端强制中断）/ FAILED（规划器自己
  放弃）五种结局各至少一条；
* **TIMEOUT 与「规划器自己放弃」用同一个障碍物场景区分**：前者给 1200 ms
  的服务端 deadline（规划器本来要烧 5 s），后者给足 20 s —— 于是「超时是
  服务端强制中断，不是规划器放弃」这条验收在同一场景面上被证明；
* ``seed`` 固定 → 同一场景跨进程逐点一致（确定性由 T2/T3 保证，本文件
  只消费，不重复实现）。

夹具纪律：模型在 pytest 临时目录里用 ``build_model.build_model()`` 物化
（绝对路径 mesh）；规划配置从仓库复制进临时工作区；报告落到 ``build/reports/``
（运行时产物）。**不依赖 gitignore 目录**（``.gsd/``）。
"""
from __future__ import annotations

import json
import os
import shutil
import sys
import time

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

import build_model  # noqa: E402

from rmctl.planner_service.client import PlannerClient  # noqa: E402
from rmctl.planner_service.server import PlannerServer  # noqa: E402

#: 场景集文件（git 跟踪；**不是** .gitignore 目录里的东西）。
SCENARIOS_PATH = os.path.join(TESTS_DIR, "golden", "planner_scenarios.json")

#: 汇总报告落盘位置（运行时产物，gitignore；供 S08 直接取用）。
REPORT_PATH = os.path.join(REPO_ROOT, "build", "reports",
                            "s07_planner_golden.json")

#: 场景集里要求的最小场景数（防止有人把场景集删到只剩一两条还「全绿」）。
MIN_SCENARIOS = 6

#: 场景集必须覆盖的结局（每一种至少一条）——防止场景集退化成「只有成功」。
REQUIRED_STATUSES = ("SUCCESS", "FAILED", "TIMEOUT")


# --------------------------------------------------------------------------
# 夹具
# --------------------------------------------------------------------------

@pytest.fixture(scope="session")
def scenarios():
    """读场景集（版本化 JSON）。"""
    with open(SCENARIOS_PATH, "r", encoding="utf-8") as fh:
        data = json.load(fh)
    assert data.get("version") == 1, data.get("version")
    items = data.get("scenarios") or []
    assert len(items) >= MIN_SCENARIOS, (
        "场景集至少 %d 条，当前 %d 条" % (MIN_SCENARIOS, len(items))
    )
    ids = [item["id"] for item in items]
    assert len(set(ids)) == len(ids), "场景 id 必须唯一: %s" % ids
    covered = {item["expect_status"] for item in items}
    missing = [status for status in REQUIRED_STATUSES
               if status not in covered]
    assert not missing, "场景集缺少这些结局的覆盖: %s" % missing
    return items


@pytest.fixture(scope="session")
def planner_ws(tmp_path_factory):
    """临时工作区：物化好的绝对路径模型 + 复制进来的规划配置。"""
    ws = tmp_path_factory.mktemp("planner_golden_ws")
    build_model.build_model(
        ws=str(ws),
        out=str(ws / "build" / "model"),
        pkg=RM_DESC_SRC,
    )
    cfg_dst = ws / "workcell" / "planner"
    cfg_dst.mkdir(parents=True)
    for name in ("pipeline.yaml", "profiles.yaml"):
        shutil.copyfile(os.path.join(PLANNER_CFG_DIR, name),
                        str(cfg_dst / name))
    return str(ws)


@pytest.fixture(scope="session")
def server(planner_ws, tmp_path_factory):
    """起在 session 级临时 socket 上的规划服务（整个场景集共用一个）。"""
    sock_path = os.path.join(
        str(tmp_path_factory.mktemp("planner_golden_sock")), "rm_planner.sock")
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
    rows = []
    for scenario in scenarios:
        plan_id = "golden-%s" % scenario["id"]
        request = build_request(scenario, plan_id)
        started = time.time()
        resp = client.plan(request)
        elapsed = time.time() - started
        metrics = resp.get("metrics") or {}
        rows.append({
            "id": scenario["id"],
            "name": scenario["name"],
            "expect_status": scenario["expect_status"],
            "status": resp["status"],
            "expect_verify_ok": scenario.get("expect_verify_ok"),
            "verify_ok": (resp.get("verification") or {}).get("ok"),
            "elapsed_s": round(elapsed, 3),
            "max_elapsed_s": float(scenario["max_elapsed_s"]),
            "seed": int(scenario["seed"]),
            "timeout_ms": int(scenario["timeout_ms"]),
            "points": metrics.get("points"),
            "failure_reason": resp.get("failure_reason") or "",
        })
    return rows


# --------------------------------------------------------------------------
# 助手
# --------------------------------------------------------------------------

def build_request(scenario, plan_id):
    """场景 → 契约层请求字典（**不补业务缺省**：场景写什么就发什么）。"""
    data = {
        "plan_id": plan_id,
        "profile": "FREESPACE",
        "group": "rm75_arm",
        "start_state": list(scenario["start"]),
        "goal": {
            "type": scenario["goal"]["type"],
            "values": list(scenario["goal"]["values"]),
        },
        "constraints": dict(scenario.get("constraints") or {}),
        "seed": int(scenario["seed"]),
        "timeout_ms": int(scenario["timeout_ms"]),
        "scene_version": 1,
    }
    obstacle = scenario.get("obstacle")
    if obstacle:
        data["scene"] = {"obstacles": [dict(obstacle)]}
    return data


def percentile(values, fraction):
    """线性插值分位数（p50 / p95）。"""
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return float(ordered[0])
    position = fraction * (len(ordered) - 1)
    lower = int(position)
    upper = min(lower + 1, len(ordered) - 1)
    weight = position - lower
    return float(ordered[lower] * (1.0 - weight) + ordered[upper] * weight)


# --------------------------------------------------------------------------
# 用例
# --------------------------------------------------------------------------

def test_golden_scenarios_match_expectations(scenarios, golden_results):
    """每个场景经服务跑一遍：status / 四道校验 / 耗时 三项断言。

    这是本任务的主验收面。
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
    _write_report(golden_results)
    assert os.path.isfile(REPORT_PATH), REPORT_PATH
    with open(REPORT_PATH, "r", encoding="utf-8") as fh:
        report = json.load(fh)
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


def test_golden_same_seed_is_pointwise_identical(client):
    """同请求同种子 → **逐点一致**（验收项 3 的场景面）。

    同一场景跑两遍（两次各自 spawn 独立 worker 子进程 → 进程级 RNG_setSeed
    各来一次），比对 ``trajectory_digest``：摘要相同即「逐点一致」。
    """
    from rmctl.planner_service.contract import trajectory_digest

    scenario = {
        "start": [0.0] * 7,
        "goal": {"type": "joint", "values": [0.2, -0.3, 0.1, 0.0, 0.0, 0.0, 0.0]},
        "constraints": {},
        "seed": 11,
        "timeout_ms": 20000,
    }
    first = client.plan(build_request(scenario, "det-a"))
    second = client.plan(build_request(scenario, "det-b"))
    assert first["status"] == "SUCCESS", first
    assert second["status"] == "SUCCESS", second
    assert trajectory_digest(first["trajectory"]) == \
        trajectory_digest(second["trajectory"]), "同种子两次结果不一致"


# --------------------------------------------------------------------------
# 报告
# --------------------------------------------------------------------------

def _write_report(rows):
    """写汇总报告（p50 / p95 / 成功率 / 逐场景行）。

    **两组延迟分开给**：
    * ``p50_s`` / ``p95_s`` —— **全部**场景的端到端墙钟（含故意失败/超时
      场景，它们是真实请求延迟的一部分）；
    * ``success_only.*`` —— 只统计 **status == SUCCESS** 的场景。

    S08 要的是「正常请求的延迟分位数」→ 用 ``success_only``；
    「请求面整体行为」→ 用顶层的 p50/p95。两组都给，不让 S08 误读。
    """
    elapsed = [row["elapsed_s"] for row in rows]
    success_elapsed = [row["elapsed_s"] for row in rows
                       if row["status"] == "SUCCESS"]
    matched = [row for row in rows
               if row["status"] == row["expect_status"]]
    report = {
        "version": 1,
        "source": "S07/T5 黄金场景集（tests/golden/planner_scenarios.json）",
        "scenario_count": len(rows),
        "p50_s": round(percentile(elapsed, 0.50), 3) if elapsed else None,
        "p95_s": round(percentile(elapsed, 0.95), 3) if elapsed else None,
        "success_rate": (len(matched) / len(rows)) if rows else 0.0,
        "success_only": {
            "count": len(success_elapsed),
            "p50_s": (round(percentile(success_elapsed, 0.50), 3)
                      if success_elapsed else None),
            "p95_s": (round(percentile(success_elapsed, 0.95), 3)
                      if success_elapsed else None),
        },
        "scenarios": rows,
    }
    parent = os.path.dirname(REPORT_PATH)
    if parent:
        os.makedirs(parent, exist_ok=True)
    with open(REPORT_PATH, "w", encoding="utf-8") as fh:
        json.dump(report, fh, ensure_ascii=False, indent=2)
    return report
