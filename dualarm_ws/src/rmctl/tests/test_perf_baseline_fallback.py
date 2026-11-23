"""S16：性能基线段的**版本库留存副本**兜底（干净 checkout 可复现）。

背景（S16 的立项理由）
----------------------
``build/`` 被 gitignore → S08 性能报告只存在于运行时目录 →
干净 checkout 跑一键回归时 ``perf_baseline`` 段会以「S08 性能报告缺失」判
``blocked``。这是**可复现性缺口**（不是回归失败，但也不是「8 段全绿」）。

解法：把同一份报告固化进版本库 ``workcell/bench/perf_baseline/s08_planner_perf.json``，
本段在运行时产物缺失时**回落到版本库副本**，使干净 checkout 上该段可复现为
``pass``；显式传入 ``perf_report`` 时**不回落**（调用方的显式选择优先）。

本文件只做离线断言（读仓库内 git 跟踪文件 + pytest 临时目录），不跑规划内核。
"""
from __future__ import annotations

import json
import os
import shutil

import pytest

from rmctl.regression import perf_baseline as pb


def _fake_repo(tmp_path):
    """搭一个最小仓库：场景声明 + 版本库留存副本（**无** build/reports）。"""
    repo = tmp_path / "repo"
    (repo / "workcell" / "bench").mkdir(parents=True)
    src_scen = pb.scenarios_path()
    shutil.copy2(src_scen, str(repo / "workcell" / "bench" / "perf_scenarios.yaml"))
    fallback = repo / pb.FALLBACK_PERF_REPORT_REL
    fallback.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(os.path.join(pb.default_repo_root(), pb.FALLBACK_PERF_REPORT_REL),
                 str(fallback))
    return repo


def test_committed_fallback_copy_exists_in_repo():
    """版本库内确实存在留存副本（否则下面的回落测试毫无意义）。"""
    path = os.path.join(pb.default_repo_root(), pb.FALLBACK_PERF_REPORT_REL)
    assert os.path.isfile(path), "缺少版本库留存副本: %s" % path
    with open(path, "r", encoding="utf-8") as fh:
        report = json.load(fh)
    for key in pb.REQUIRED_REPORT_KEYS:
        assert key in report, "留存副本缺键 %s" % key
    assert report.get("status") == "ok", report.get("reason")


def test_segment_falls_back_to_committed_copy(tmp_path):
    """运行时产物缺失 → 回落到版本库副本 → 段判定为 pass（不再是 blocked）。"""
    repo = _fake_repo(tmp_path)
    ws = tmp_path / "ws"
    ws.mkdir()
    result = pb.run_perf_baseline(
        repo_root=str(repo),
        ws=str(ws),
        reports_dir=str(tmp_path / "reports"),
    )
    assert result["verdict"] == "pass", result.get("reason")
    # 产出基线表（CSV / Markdown）——说明确实投影了报告，而不是空表
    assert os.path.isfile(str(tmp_path / "reports" / pb.CSV_NAME))


def test_explicit_missing_report_stays_blocked(tmp_path):
    """显式指定的报告路径不存在 → 仍然 blocked（显式选择优先于回落）。"""
    repo = _fake_repo(tmp_path)
    ws = tmp_path / "ws"
    ws.mkdir()
    result = pb.run_perf_baseline(
        repo_root=str(repo),
        ws=str(ws),
        perf_report=str(tmp_path / "nope.json"),
        reports_dir=str(tmp_path / "reports"),
    )
    assert result["verdict"] == "blocked", result.get("reason")
    assert "rm_planner_perf.py" in result["next_action"]


def test_fallback_copy_is_not_rewritten(tmp_path):
    """负例面：读取留存副本**不得**改写它（本段是只读投影）。"""
    repo = _fake_repo(tmp_path)
    ws = tmp_path / "ws"
    ws.mkdir()
    fallback = repo / pb.FALLBACK_PERF_REPORT_REL
    before = fallback.read_bytes()
    pb.run_perf_baseline(repo_root=str(repo), ws=str(ws),
                         reports_dir=str(tmp_path / "reports"))
    assert fallback.read_bytes() == before, "留存副本被改写了"
