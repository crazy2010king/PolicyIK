"""段级判定模型（``rmctl.regression.segments``）的单元测试。

覆盖：verdict 全集、退出码映射、聚合三态、contract 兜底 next_action、
顶层键齐全，以及负例（未知 verdict / 空段列表 / 非法 mode / 非字典段）。
"""
import ast
import os

import pytest

from rmctl.regression import segments as seg
from rmctl.regression.segments import (
    MODES,
    VERDICTS,
    aggregate,
    segment_result,
    verdict_to_exit_code,
    verdict_to_status,
)

SEGMENT_KEYS = ("id", "verdict", "reason", "next_action", "evidence",
                "mode", "report_path")
TOP_KEYS = ("status", "reason", "next_action", "evidence", "segments", "mode")


def _contract():
    return seg.contract()


# ------------------------------------------------------------- verdict 全集
def test_verdicts_are_exactly_four():
    assert VERDICTS == ("pass", "blocked", "fail", "skipped")


def test_every_verdict_builds_a_segment():
    for verdict in VERDICTS:
        out = segment_result(verdict, reason="r", id="seg-" + verdict)
        assert out["verdict"] == verdict
        assert out["id"] == "seg-" + verdict
        for key in SEGMENT_KEYS:
            assert key in out, "段结果缺键: %s" % key


def test_status_comes_from_executor_contract():
    c = _contract()
    assert verdict_to_status("pass") == c.STATUS_OK
    assert verdict_to_status("skipped") == c.STATUS_OK
    assert verdict_to_status("blocked") == c.STATUS_BLOCKED
    assert verdict_to_status("fail") == c.STATUS_ERROR


# ------------------------------------------------------------- 退出码映射
def test_exit_code_mapping_matches_contract():
    c = _contract()
    assert verdict_to_exit_code("pass") == 0
    assert verdict_to_exit_code("skipped") == 0
    assert verdict_to_exit_code("blocked") == 3
    assert verdict_to_exit_code("fail") == 4
    for verdict in VERDICTS:
        assert verdict_to_exit_code(verdict) == \
            c.EXIT_CODES[verdict_to_status(verdict)]


# ------------------------------------------------------------- 聚合三态
def test_aggregate_all_pass():
    segs = [segment_result("pass", id="a"), segment_result("skipped", id="b")]
    report = aggregate(segs)
    assert report["status"] == _contract().STATUS_OK
    assert report["exit_code"] == 0
    assert len(report["segments"]) == 2


def test_aggregate_blocked_yields_exit_3():
    segs = [segment_result("pass", id="a"),
            segment_result("blocked", reason="无 build/model", id="b")]
    report = aggregate(segs)
    assert report["status"] == _contract().STATUS_BLOCKED
    assert report["exit_code"] == 3


def test_aggregate_fail_wins_over_blocked():
    segs = [segment_result("blocked", id="a"),
            segment_result("fail", reason="期望值不符", id="b")]
    report = aggregate(segs)
    assert report["status"] == _contract().STATUS_ERROR
    assert report["exit_code"] == 4


def test_aggregate_top_level_keys_complete():
    report = aggregate([segment_result("pass", id="a")], mode="offline")
    for key in TOP_KEYS:
        assert key in report, "顶层报告缺键: %s" % key
    assert report["mode"] == "offline"
    assert isinstance(report["segments"], list)
    assert isinstance(report["evidence"], list)


def test_aggregate_mode_round_trip():
    for mode in MODES:
        report = aggregate([segment_result("pass", id="a", mode=mode)], mode=mode)
        assert report["mode"] == mode
        assert report["segments"][0]["mode"] == mode


# ------------------------------------------------------- next_action 兜底
def test_blocked_segment_without_next_action_gets_contract_fallback():
    out = segment_result("blocked", reason="真机不可达", id="hw")
    assert out["next_action"], "blocked 段的 next_action 不得静默丢失"
    assert out["next_action"] == _contract().NEXT_ACTION_FALLBACK


def test_fail_segment_without_next_action_gets_contract_fallback():
    out = segment_result("fail", reason="契约回归", id="reg")
    assert out["next_action"] == _contract().NEXT_ACTION_FALLBACK


def test_aggregate_blocked_report_has_next_action():
    report = aggregate([segment_result("blocked", id="a")])
    assert report["next_action"], "顶层 blocked 报告必须带可粘贴的 next_action"


# ------------------------------------------------------------- 负例
def test_unknown_verdict_raises():
    with pytest.raises(ValueError):
        segment_result("unknown")
    with pytest.raises(ValueError):
        verdict_to_status("PASS")
    with pytest.raises(ValueError):
        verdict_to_exit_code(None)


def test_aggregate_rejects_unknown_verdict():
    with pytest.raises(ValueError):
        aggregate([{"id": "a", "verdict": "boom"}])


def test_aggregate_rejects_malformed_segment():
    with pytest.raises(ValueError):
        aggregate(["not-a-dict"])
    with pytest.raises(ValueError):
        aggregate([{"id": "a"}])


def test_empty_segments_is_valid_report():
    report = aggregate([])
    assert report["status"] == _contract().STATUS_OK
    assert report["exit_code"] == 0
    assert report["segments"] == []


def test_unknown_mode_raises():
    with pytest.raises(ValueError):
        segment_result("pass", mode="simulated")
    with pytest.raises(ValueError):
        aggregate([], mode="simulated")


# -------------------------------------------------- 无第二套 status 词表
def _module_source():
    here = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(here, "..", "rmctl", "regression", "segments.py")
    with open(path, "r", encoding="utf-8") as fh:
        return fh.read()


def test_module_has_no_second_status_vocabulary():
    """模块内不得出现裸的 status 字符串常量（"ok"/"error" 必须是字面量禁止）。"""
    tree = ast.parse(_module_source())
    allowed = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign):
            targets = [t.id for t in node.targets
                       if isinstance(t, ast.Name)]
            if "VERDICTS" in targets:
                allowed.update(
                    elt.value for elt in node.value.elts
                    if isinstance(elt, ast.Constant) and isinstance(elt.value, str)
                )
    status_literals = {"ok", "error"}
    for node in ast.walk(tree):
        if isinstance(node, ast.Constant) and isinstance(node.value, str):
            if node.value in status_literals:
                raise AssertionError(
                    "segments.py 内出现裸 status 字面量 %r（禁止第二套 status 词表）"
                    % node.value
                )
            if node.value == "blocked" and node.value not in allowed:
                raise AssertionError(
                    "segments.py 内出现 VERDICTS 之外的 'blocked' 字面量"
                )


def test_module_does_not_import_vendor_or_hardware():
    src = _module_source()
    assert "tesseract_robotics" not in src
    assert "rm_hw_bridge" not in src
