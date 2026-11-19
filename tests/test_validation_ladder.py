"""真机验证**阶梯**的回归测试（零硬件、零 Drake）。

覆盖
----
* 阶梯定义自身的完整性（id 唯一、层级单调、判据非空、依赖存在）；
* 判据求值语义（边界、缺失指标、集合类比较）；
* 门禁语义（依赖未通过 ⇒ blocked；运动级别未授权 ⇒ blocked）；
* 人工证据回填（manual 级 ⇒ 用证据判定）；
* 阶梯与**文档**的一致性（文档里出现的级别 id 必须都存在，反之亦然）——
  防止「文档写了一套、代码跑的是另一套」。
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest

from rm75_drake.validation.ladder import (
    STATUS_BLOCKED,
    STATUS_FAIL,
    STATUS_MANUAL,
    STATUS_PASS,
    TIERS,
    Criterion,
    LadderContext,
    LadderError,
    build_ladder,
    evaluate_criteria,
    run_ladder,
    stage_by_id,
)

REPO_ROOT = Path(__file__).resolve().parents[1]
DOC_PATH = REPO_ROOT / "docs" / "validation" / "RM75-B_真机验证阶梯.md"


def _context(**kwargs) -> LadderContext:
    return LadderContext(device_config=None, **kwargs)


# ── 阶梯定义 ────────────────────────────────────────────────────────────
def test_ladder_has_unique_ordered_ids():
    ids = [stage.id for stage in build_ladder()]
    assert ids == sorted(ids, key=lambda s: int(s[1:]))
    assert len(ids) == len(set(ids))
    assert ids[0] == "L0"


def test_every_stage_has_criteria_and_metadata():
    for stage in build_ladder():
        assert stage.criteria, f"{stage.id} 没有判据"
        assert stage.tier in TIERS
        assert stage.goal
        assert stage.title
        assert stage.design_refs, f"{stage.id} 没有指向设计文档"
        assert stage.milestone_refs, f"{stage.id} 没有指向 M001 切片"
        assert stage.failure_actions, f"{stage.id} 没有写失败处置"


def test_stage_dependencies_reference_existing_stages():
    ids = {stage.id for stage in build_ladder()}
    for stage in build_ladder():
        for dep in stage.depends_on:
            assert dep in ids, f"{stage.id} 依赖了不存在的级别 {dep}"


def test_motion_stages_come_after_readonly_stages():
    """「先只读、后运动」——阶梯必须按复杂度递进。"""
    stages = build_ladder()
    first_motion = next(i for i, s in enumerate(stages) if s.motion)
    assert all(not s.motion for s in stages[:first_motion])


def test_stage_by_id_rejects_unknown():
    with pytest.raises(LadderError):
        stage_by_id(build_ladder(), "L99")


# ── 判据求值 ────────────────────────────────────────────────────────────
def test_criterion_numeric_comparison_is_inclusive():
    criterion = Criterion("x", "示例", "<=", 1.0)
    ok, _ = criterion.evaluate(1.0)
    assert ok
    ok, _ = criterion.evaluate(1.0001)
    assert not ok


def test_criterion_missing_metric_fails():
    criterion = Criterion("x", "示例", "<=", 1.0)
    ok, message = criterion.evaluate(None)
    assert not ok
    assert "缺失" in message


def test_criterion_set_membership():
    inside = Criterion("s", "示例", "in", {200, 404})
    outside = Criterion("s", "示例", "not_in", {9, 10})
    assert inside.evaluate(200)[0]
    assert not inside.evaluate(500)[0]
    assert outside.evaluate(0)[0]
    assert not outside.evaluate(9)[0]


def test_criterion_boolean_ops():
    assert Criterion("b", "示例", "is_true").evaluate(True)[0]
    assert not Criterion("b", "示例", "is_true").evaluate(False)[0]
    assert Criterion("b", "示例", "is_false").evaluate(False)[0]


def test_evaluate_criteria_reports_failures():
    stage = stage_by_id(build_ladder(), "L0")
    passed, failed = evaluate_criteria(stage, {"tcp_connected": True, "http_status": 200})
    assert not failed
    assert passed
    _, failed = evaluate_criteria(stage, {"tcp_connected": False})
    assert failed


# ── 门禁 ────────────────────────────────────────────────────────────────
def _fake_executor(metrics):
    def _executor(stage, context):
        return dict(metrics.get(stage.id, {}))

    return _executor


def test_dependency_failure_blocks_dependent_stage():
    stages = build_ladder()
    context = _context()
    results = run_ladder(
        stages,
        _fake_executor({"L0": {"tcp_connected": True, "http_status": 500}}),
        context=context,
        stage_ids=["L0", "L1"],
    )
    assert results[0].status == STATUS_FAIL
    assert results[1].status == STATUS_BLOCKED
    assert "L0" in results[1].notes[0]


def test_motion_stage_requires_allow_motion():
    stages = build_ladder()
    results = run_ladder(
        stages,
        _fake_executor({}),
        context=_context(allow_motion=False),
        stage_ids=["L6"],
    )
    assert results[0].status == STATUS_BLOCKED
    assert "allow-motion" in results[0].notes[0]


def test_manual_stage_without_evidence_is_manual():
    stages = build_ladder()
    results = run_ladder(
        stages,
        _fake_executor({}),
        context=_context(allow_motion=True),
        stage_ids=["L8"],
    )
    assert results[0].status == STATUS_MANUAL
    assert "人工" in results[0].notes[0]


def test_manual_stage_with_evidence_is_evaluated():
    stages = build_ladder()
    results = run_ladder(
        stages,
        _fake_executor({"L8": {"exec_exit_code": 0}}),
        context=_context(allow_motion=True),
        manual_evidence={
            "L8": {"tcp_tracking_error_mm": 2.4, "exec_exit_code": 0}
        },
        stage_ids=["L8"],
    )
    assert results[0].status == STATUS_PASS
    assert results[0].metrics["tcp_tracking_error_mm"] == 2.4


def test_executor_exception_fails_stage_without_swallowing():
    def _boom(stage, context):
        raise RuntimeError("模拟硬件炸了")

    results = run_ladder(
        build_ladder(), _boom, context=_context(), stage_ids=["L0"]
    )
    assert results[0].status == STATUS_FAIL
    assert "模拟硬件炸了" in results[0].failures[0]


def test_all_stages_pass_when_every_metric_satisfies_criteria():
    """用「满足全部判据」的假指标跑一遍：阶梯应当全绿（门禁语义自洽）。"""
    stages = build_ladder()
    metrics = {}
    for stage in stages:
        values = {}
        for criterion in stage.criteria:
            if criterion.op == "is_true":
                values[criterion.key] = True
            elif criterion.op == "is_false":
                values[criterion.key] = False
            elif criterion.op == "in":
                values[criterion.key] = sorted(criterion.threshold)[0]
            elif criterion.op == "not_in":
                values[criterion.key] = -1
            elif criterion.op == "==":
                values[criterion.key] = criterion.threshold
            elif criterion.op == "<=":
                values[criterion.key] = criterion.threshold
            elif criterion.op == ">=":
                values[criterion.key] = criterion.threshold
        metrics[stage.id] = values
    results = run_ladder(
        stages,
        _fake_executor(metrics),
        context=_context(allow_motion=True),
        manual_evidence={
            stage.id: {
                c.key: (
                    True
                    if c.op == "is_true"
                    else (False if c.op == "is_false" else c.threshold)
                )
                for c in stage.criteria
            }
            for stage in stages
            if stage.manual
        },
        stage_ids=[stage.id for stage in stages],
    )
    assert [r.status for r in results] == [STATUS_PASS] * len(stages)


# ── 文档一致性 ──────────────────────────────────────────────────────────
@pytest.mark.skipif(not DOC_PATH.is_file(), reason="阶梯文档尚未落盘")
def test_doc_mentions_every_stage_and_vice_versa():
    text = DOC_PATH.read_text(encoding="utf-8")
    doc_ids = set(re.findall(r"\bL(\d{1,2})\b", text))
    code_ids = {stage.id[1:] for stage in build_ladder()}
    assert code_ids <= doc_ids, f"文档缺少级别：{sorted(code_ids - doc_ids)}"
    assert doc_ids <= code_ids, f"文档里有阶梯里没有的级别：{sorted(doc_ids - code_ids)}"
