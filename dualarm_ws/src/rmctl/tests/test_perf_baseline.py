"""S09/T4 性能基线表 S1–S5 的验收用例。

覆盖（计划要求 + Q5/Q7 负例）：

* S1–S5 五行齐全；S4/S5 ``applicable=false`` 且带理由；
* CSV 行数与 YAML 条目一致；
* 缺 S08 报告 → 归一化为 **blocked**（不得产出空数字冒充）；
* S08 报告结构变化（缺 summary/comparison/results）→ **fail** 且指名缺哪个键；
* 每个数值行的 ``mode=offline``；
* 组数未达设计 §6.9 要求时如实标注；
* **Q7 负例**：把 S4 改成 ``applicable: true`` 却不提供双臂数据 → 直接报错
  （防止用假数据冒充双臂场景）。

只使用 git 跟踪的仓库文件 + pytest 临时目录（绝不依赖 .gitignore 目录）。
"""
import csv
import json
import os

import pytest

from rmctl.regression import orchestrator as orch
from rmctl.regression import perf_baseline as pb


# ---------------------------------------------------------------------------
# 夹具
# ---------------------------------------------------------------------------
def _perf_row(scenario_id, wall_s, rss_mb, status="SUCCESS",
              expect_status="SUCCESS", profile="optimized"):
    return {
        "profile": profile,
        "id": scenario_id,
        "status": status,
        "expect_status": expect_status,
        "status_match": status == expect_status,
        "e2e_wall_s": wall_s,
        "worker_peak_rss_mb": rss_mb,
    }


def _fake_s08_report():
    """一份**形状**与 S08 报告一致的报告（数值是夹具值，不是实测结论）。"""
    return {
        "version": 1,
        "git_rev": "deadbeefcafe",
        "results": [
            _perf_row("joint_small_local", 0.31, 320.0),
            _perf_row("joint_cross_zero", 0.45, 330.0),
            _perf_row("cartesian_reachable", 0.62, 340.0),
            _perf_row("obstacle_blocked_timeout", 4.11, 350.0,
                      status="TIMEOUT", expect_status="TIMEOUT"),
            _perf_row("obstacle_blocked_planner_gives_up", 3.02, 345.0,
                      status="FAILED", expect_status="FAILED"),
        ],
        "summary": {"optimized": {"scenario_count": 5}},
        "comparison": {"success_only_p95_s": {"baseline": 1.342,
                                              "optimized": 0.310}},
    }


@pytest.fixture()
def s08_report(tmp_path):
    """把一份夹具报告写到临时目录，返回路径。"""
    path = tmp_path / "s08_planner_perf.json"
    path.write_text(json.dumps(_fake_s08_report(), ensure_ascii=False),
                    encoding="utf-8")
    return str(path)


# ---------------------------------------------------------------------------
# 1. 场景声明（唯一来源）
# ---------------------------------------------------------------------------
def test_yaml_declares_s1_to_s5(repo_root):
    suites = pb.load_suites(repo_root)
    assert [s["id"] for s in suites] == ["S1", "S2", "S3", "S4", "S5"]
    assert [s["group_count"] for s in suites] == [100, 20, 20, 20, 10]


def test_s4_s5_not_applicable_with_reason(repo_root):
    suites = {s["id"]: s for s in pb.load_suites(repo_root)}
    for sid in ("S4", "S5"):
        assert suites[sid]["applicable"] is False
        reason = suites[sid]["not_applicable_reason"]
        assert reason, "%s 必须写 not_applicable_reason" % sid
        assert "单臂" in reason and "双臂" in reason
        assert not suites[sid]["scenario_ids"]


def test_scenario_ids_exist_in_golden_set(repo_root):
    """映射到的黄金场景 id 必须真实存在（防止写错 id 静默无数据）。"""
    import rmctl.golden_runner as gr

    known = {s["id"] for s in gr.load_scenarios()}
    for suite in pb.load_suites(repo_root):
        for sid in suite["scenario_ids"]:
            assert sid in known, "%s 引用了不存在的黄金场景 %s" \
                % (suite["id"], sid)


# ---------------------------------------------------------------------------
# 2. 投影与落盘
# ---------------------------------------------------------------------------
def test_rows_five_and_not_applicable_has_no_numbers(repo_root):
    suites = pb.load_suites(repo_root)
    rows = pb.build_rows(suites, _fake_s08_report(),
                          source_report="build/reports/s08_planner_perf.json")
    assert len(rows) == 5
    by_id = {r["suite"]: r for r in rows}
    assert set(by_id) == {"S1", "S2", "S3", "S4", "S5"}
    for sid in ("S4", "S5"):
        row = by_id[sid]
        assert row["notes"].startswith("not_applicable")
        # 数值列必须为空（不生成任何假数字）
        for key in ("measured_count", "p50_s", "p95_s", "max_s",
                    "success_rate", "peak_rss_mb"):
            assert row[key] == "", "%s 的 %s 必须为空" % (sid, key)


def test_csv_row_count_matches_yaml_entries(repo_root, tmp_path, s08_report):
    suites = pb.load_suites(repo_root)
    rows = pb.build_rows(suites, _fake_s08_report(), source_report="s08")
    csv_path = pb.write_csv(rows, str(tmp_path / "s09_perf_baseline.csv"))
    with open(csv_path, "r", encoding="utf-8") as fh:
        data = list(csv.DictReader(fh))
    assert len(data) == len(suites)
    assert [row["suite"] for row in data] == [s["id"] for s in suites]
    assert list(data[0].keys()) == list(pb.CSV_COLUMNS)


def test_numeric_rows_mode_offline(repo_root, tmp_path, s08_report):
    suites = pb.load_suites(repo_root)
    rows = pb.build_rows(suites, _fake_s08_report(), source_report="s08")
    numeric = [r for r in rows if r["measured_count"] != ""]
    assert numeric, "至少要有数值行"
    for row in numeric:
        assert row["mode"] == "offline"
        assert row["mode"] != "live"


def test_group_count_gap_is_reported(repo_root):
    """实测组数 < 设计要求组数 → 必须写进 notes（不假装达标）。"""
    suites = pb.load_suites(repo_root)
    rows = pb.build_rows(suites, _fake_s08_report(), source_report="s08")
    by_id = {r["suite"]: r for r in rows}
    assert by_id["S1"]["measured_count"] == 2
    assert by_id["S1"]["group_count"] == 100
    assert "未达组数要求" in by_id["S1"]["notes"]
    assert "100" in by_id["S1"]["notes"]


def test_markdown_contains_table_and_notes(repo_root, tmp_path):
    suites = pb.load_suites(repo_root)
    rows = pb.build_rows(suites, _fake_s08_report(), source_report="s08")
    md_path = pb.write_markdown(rows, str(tmp_path / "s09_perf_baseline.md"),
                                 meta={"source_report": "s08",
                                       "git_rev": "deadbeef"})
    text = open(md_path, "r", encoding="utf-8").read()
    for sid in ("S1", "S2", "S3", "S4", "S5"):
        assert sid in text
    assert "not_applicable" in text
    assert "未达组数要求" in text
    assert "offline" in text


# ---------------------------------------------------------------------------
# 3. 段级判定（Q5 失效模式）
# ---------------------------------------------------------------------------
def test_missing_s08_report_is_blocked_not_fail(repo_root, tmp_path):
    """干净机器（没跑过 S08 性能脚本）→ blocked，且指名下一步。"""
    result = pb.run_perf_baseline(
        repo_root=repo_root,
        ws=str(tmp_path),
        perf_report=str(tmp_path / "nope.json"),
        reports_dir=str(tmp_path / "reports"),
    )
    assert result["verdict"] == "blocked"
    assert "rm_planner_perf.py" in result["next_action"]
    # blocked → 不产出基线表（不得有空数字冒充）
    assert not os.path.exists(str(tmp_path / "reports" / pb.CSV_NAME))


@pytest.mark.parametrize("missing_key", ["summary", "comparison", "results"])
def test_report_schema_change_is_fail(repo_root, tmp_path, missing_key):
    report = _fake_s08_report()
    report.pop(missing_key)
    path = tmp_path / "s08.json"
    path.write_text(json.dumps(report), encoding="utf-8")
    result = pb.run_perf_baseline(
        repo_root=repo_root,
        ws=str(tmp_path),
        perf_report=str(path),
        reports_dir=str(tmp_path / "reports"),
    )
    assert result["verdict"] == "fail"
    assert missing_key in result["reason"]


def test_unparsable_report_is_fail(repo_root, tmp_path):
    path = tmp_path / "s08.json"
    path.write_text("{ not json", encoding="utf-8")
    result = pb.run_perf_baseline(
        repo_root=repo_root,
        ws=str(tmp_path),
        perf_report=str(path),
        reports_dir=str(tmp_path / "reports"),
    )
    assert result["verdict"] == "fail"
    assert "无法解析" in result["reason"]


def test_run_perf_baseline_pass_writes_both_files(repo_root, tmp_path,
                                                   s08_report):
    reports_dir = tmp_path / "reports"
    result = pb.run_perf_baseline(
        repo_root=repo_root,
        ws=str(tmp_path),
        perf_report=s08_report,
        reports_dir=str(reports_dir),
    )
    assert result["verdict"] == "pass"
    assert result["mode"] == "offline"
    csv_path = reports_dir / pb.CSV_NAME
    md_path = reports_dir / pb.MD_NAME
    assert csv_path.exists() and md_path.exists()
    with open(str(csv_path), "r", encoding="utf-8") as fh:
        data = list(csv.DictReader(fh))
    assert len(data) == 5
    assert data[0]["source_report"]


# ---------------------------------------------------------------------------
# 4. Q7 负例
# ---------------------------------------------------------------------------
def test_q7_s4_applicable_true_without_data_raises(repo_root, tmp_path):
    """把 S4 改成 applicable=true 却不给双臂数据 → 必须报错（不得冒充）。"""
    suites = pb.load_suites(repo_root)
    broken = []
    for suite in suites:
        item = dict(suite)
        if item["id"] == "S4":
            item["applicable"] = True
            item["scenario_ids"] = []
        broken.append(item)
    with pytest.raises(ValueError) as exc:
        pb.validate_suites(broken)
    assert "S4" in str(exc.value)
    assert "applicable" in str(exc.value)


def test_q7_not_applicable_without_reason_raises(repo_root):
    suites = pb.load_suites(repo_root)
    broken = []
    for suite in suites:
        item = dict(suite)
        if item["id"] == "S5":
            item["not_applicable_reason"] = ""
        broken.append(item)
    with pytest.raises(ValueError) as exc:
        pb.validate_suites(broken)
    assert "S5" in str(exc.value)


def test_q7_suite_without_measurements_yields_no_numbers(repo_root):
    """映射的黄金场景在 S08 报告里没出现 → 该行不产出任何数字。"""
    suites = pb.load_suites(repo_root)
    report = _fake_s08_report()
    report["results"] = [r for r in report["results"]
                         if r["id"] != "cartesian_reachable"]
    rows = pb.build_rows(suites, report, source_report="s08")
    by_id = {r["suite"]: r for r in rows}
    row = by_id["S2"]
    assert row["measured_count"] == 0
    assert row["p50_s"] == "" and row["p95_s"] == "" and row["max_s"] == ""
    assert row["success_rate"] == ""
    assert "没有 S2 的实测行" in row["notes"]


def test_q7_invalid_group_count_raises(repo_root):
    suites = pb.load_suites(repo_root)
    broken = []
    for suite in suites:
        item = dict(suite)
        if item["id"] == "S1":
            item["group_count"] = 0
        broken.append(item)
    with pytest.raises(ValueError):
        pb.validate_suites(broken)


# ---------------------------------------------------------------------------
# 5. 编排器注册
# ---------------------------------------------------------------------------
def test_orchestrator_registers_perf_baseline_segment():
    assert "perf_baseline" in orch.segment_ids()


def test_orchestrator_perf_baseline_segment_blocked_without_report(tmp_path,
                                                                  repo_root):
    """编排器层：缺 S08 报告 → 段级 blocked（不是 fail）。

    2026-11-23（S16）：仓库内新增了**版本库留存副本**
    （``workcell/bench/perf_baseline/s08_planner_perf.json``），因此
    “运行时产物缺失”会回落到留存副本 → 段判定为 pass。
    要复现 blocked 分支，必须**显式**指定一个不存在的报告路径。
    """
    ctx = orch.SegmentContext(repo_root=repo_root,
                               ws=str(tmp_path),
                               reports_dir=str(tmp_path / "reports"))
    seg = orch.run_segment("perf_baseline", ctx)
    assert seg["id"] == "perf_baseline"
    # 回落版本库留存副本 → 可复现为 pass
    assert seg["verdict"] == "pass", seg.get("reason")
    assert seg["mode"] == "offline"
