"""``scripts/rm75_validation_ladder.py``（真机验证阶梯编排器）的**离线**用例。

为什么测：阶梯是"从只读到真机运动"的编排层，**编排纪律错了就会误动臂**，
所以它的逻辑必须能完全离线钉死：

* 阶段注册表的顺序、风险标注、高风险阶段的授权闸门；
* **前序失败 → 后续 skipped**（阶梯纪律）与 ``--continue-on-failure`` 的例外；
* ``--dry-run`` **一次执行器都不调**；
* 各阶段的状态映射（ok→pass / blocked→blocked / error→error）与证据提取；
* 验收条款覆盖表：CC1..CC8 全在表里，不在本阶梯覆盖的条款**必须写明原因**；
* 微动轨迹生成：时间严格递增、位移 = 设定值、rad 单位、首点 = 当前位姿。

全部用**假执行器**注入 → 不连真机、不需要真 SDK。
"""
from __future__ import annotations

import importlib.util
import json
import math
import os
import sys

import pytest

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(TESTS_DIR, "..", "..", "..", ".."))
SCRIPT_PATH = os.path.join(REPO_ROOT, "scripts", "rm75_validation_ladder.py")


def _load_script():
    spec = importlib.util.spec_from_file_location("rm75_validation_ladder", SCRIPT_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def ladder():
    return _load_script()


# ------------------------------------------------------------------ 假执行器
def _probe_ok(**kwargs):
    return {
        "status": "ok",
        "reason": "",
        "evidence": [{"kind": "ok", "detail": "ok"}],
        "phases": [
            {"name": "snapshot", "status": "ok", "detail": "读到 7 个关节角",
             "data": {"snapshot": {"joint_deg": [0.0] * 7,
                                   "realtime": {"frames": 12},
                                   "controller_trajectory_type_name": "RM_NO_PLANNING_E",
                                   "arm_current_status_name": "RM_IDLE_E"}}},
            {"name": "preflight", "status": "ok", "detail": "前置检查 7/7 通过",
             "data": {"checks": [{"name": "arm_err_empty", "ok": True, "detail": "无"}]}},
            {"name": "verify_out", "status": "ok", "detail": "到位误差 0.045°",
             "data": {"arrival_error_deg": 0.045, "max_drift_deg": 0.004}},
            {"name": "verify_back", "status": "ok", "detail": "回程 0.045°",
             "data": {"max_return_error_deg": 0.045}},
        ],
    }


def _probe_fail(**kwargs):
    return {"status": "error", "reason": "前置检查未通过：controller_idle",
            "evidence": [{"kind": "error", "detail": "控制器忙"}], "phases": []}


class FakeLink(object):
    """假句柄：验证阶梯的**共享连接**语义（只建一次链、全程复用）。"""

    def __init__(self):
        self.closed = False

    def close(self):
        self.closed = True


def _ctx(ladder, tmp_path, probe=None, hwcollect=None, subprocess_=None,
         executor=None, **kwargs):
    link = FakeLink()
    return ladder.LadderContext(
        out_dir=str(tmp_path / "validation"),
        link_provider=lambda: (link, None),
        probe_runner=probe if probe is not None else _probe_ok,
        hwcollect_runner=hwcollect if hwcollect is not None else (
            lambda name, **kw: {"status": "ok", "summary": {"frames": 3000,
                                                             "period_ms": 5},
                                "reason": ""}),
        subprocess_runner=subprocess_ if subprocess_ is not None else (
            lambda args, cwd=None: (0, json.dumps({
                "status": "ok",
                "checks": [{"name": "hw:sdk", "ok": True, "level": "INFO"},
                           {"name": "hw:symbols", "ok": True, "level": "INFO"},
                           {"name": "hw:contract", "ok": True, "level": "INFO"}]}),
                "审计通过：0 项违规")),
        executor_runner=executor if executor is not None else (
            lambda **kw: {"status": "ok", "metrics": {"ticks": 500}}),
        **kwargs)

# ------------------------------------------------------------------ 注册表
def test_stage_registry_is_ordered_and_complete(ladder):
    ids = [s.id for s in ladder.STAGES]
    assert ids == ["V%d" % i for i in range(10)]
    assert ladder.STAGE_INDEX["V3"].moves_arm is True
    assert ladder.STAGE_INDEX["V1"].moves_arm is False


def test_risky_stages_require_authorization(ladder):
    risky = [s.id for s in ladder.STAGES if s.needs_authorization]
    assert risky == ["V8", "V9"]
    for stage in ladder.STAGES:
        if stage.needs_authorization:
            assert stage.risk == ladder.RISK_HIGH


def test_all_criteria_are_listed_with_reason_when_uncovered(ladder):
    assert ladder.ALL_CRITERIA == ["CC%d" % i for i in range(1, 9)]
    for cc in ("CC1", "CC3", "CC8"):
        assert ladder.COVERAGE_NOTES.get(cc), "%s 必须写明未覆盖原因" % cc


# ------------------------------------------------------------------ 编排纪律
def test_failure_stops_later_stages(ladder, tmp_path):
    """V1 失败 → V2 及之后全部 skipped（阶梯纪律）。"""
    def probe(**kwargs):
        if kwargs.get("precheck_only"):
            return _probe_fail(**kwargs)
        return _probe_ok(**kwargs)

    ctx = _ctx(ladder, tmp_path, probe=probe)
    report = ladder.run_ladder(ctx)
    statuses = {s["stage"]: s["status"] for s in report["stages"]}
    assert statuses["V1"] == ladder.STATUS_ERROR
    assert statuses["V2"] == ladder.STATUS_SKIPPED
    assert statuses["V9"] == ladder.STATUS_SKIPPED
    assert report["status"] == ladder.STATUS_ERROR


def test_continue_on_failure_keeps_going(ladder, tmp_path):
    def probe(**kwargs):
        if kwargs.get("precheck_only"):
            return _probe_fail(**kwargs)
        return _probe_ok(**kwargs)

    ctx = _ctx(ladder, tmp_path, probe=probe)
    report = ladder.run_ladder(ctx, continue_on_failure=True)
    statuses = {s["stage"]: s["status"] for s in report["stages"]}
    assert statuses["V1"] == ladder.STATUS_ERROR
    assert statuses["V2"] != ladder.STATUS_SKIPPED


def test_risky_stages_skipped_without_authorization(ladder, tmp_path):
    ctx = _ctx(ladder, tmp_path)
    report = ladder.run_ladder(ctx)
    statuses = {s["stage"]: s["status"] for s in report["stages"]}
    assert statuses["V8"] == ladder.STATUS_SKIPPED
    assert statuses["V9"] == ladder.STATUS_SKIPPED
    assert "allow-risky" in [s.get("reason", "") for s in report["stages"]
                             if s["stage"] == "V8"][0]


def test_allow_risky_runs_risky_stages(ladder, tmp_path):
    ctx = _ctx(ladder, tmp_path)
    report = ladder.run_ladder(ctx, stage_ids=["V8", "V9"], allow_risky=True)
    statuses = {s["stage"]: s["status"] for s in report["stages"]}
    assert statuses["V8"] == ladder.STATUS_PASS
    assert statuses["V9"] == ladder.STATUS_PASS


def test_dry_run_never_calls_executors(ladder, tmp_path):
    calls = []

    def probe(**kwargs):
        calls.append("probe")
        return _probe_ok(**kwargs)

    def hwcollect(name, **kwargs):
        calls.append("hwcollect")
        return {"status": "ok", "summary": {}}

    def sub(args, cwd=None):
        calls.append("subprocess")
        return (0, "", "")

    ctx = _ctx(ladder, tmp_path, probe=probe, hwcollect=hwcollect, subprocess_=sub)
    report = ladder.run_ladder(ctx, dry_run=True)
    assert calls == []
    assert report["status"] == "dry_run"
    assert all(s["status"] == ladder.STATUS_SKIPPED for s in report["stages"])


def test_unknown_stage_raises(ladder, tmp_path):
    ctx = _ctx(ladder, tmp_path)
    with pytest.raises(ValueError):
        ladder.run_ladder(ctx, stage_ids=["V99"])


# ------------------------------------------------------------------ 各阶段
def test_v0_passes_with_symbols_and_clean_audit(ladder, tmp_path):
    ctx = _ctx(ladder, tmp_path)
    report = ladder.run_ladder(ctx, stage_ids=["V0"])
    stage = report["stages"][0]
    assert stage["status"] == ladder.STATUS_PASS
    assert stage["data"]["audit_exit"] == 0


def test_v0_blocks_when_audit_fails(ladder, tmp_path):
    def sub(args, cwd=None):
        if "audit" in " ".join(args):
            return (3, "", "调用黑名单符号 rm_algo_xxx")
        return (0, json.dumps({"status": "ok", "checks": []}), "")

    ctx = _ctx(ladder, tmp_path, subprocess_=sub)
    report = ladder.run_ladder(ctx, stage_ids=["V0"])
    assert report["stages"][0]["status"] == ladder.STATUS_ERROR


def test_v0_blocks_when_doctor_has_no_hw_checks(ladder, tmp_path):
    def sub(args, cwd=None):
        if "audit" in " ".join(args):
            return (0, "审计通过：0 项违规", "")
        return (0, json.dumps({"status": "ok", "checks": []}), "")

    ctx = _ctx(ladder, tmp_path, subprocess_=sub)
    report = ladder.run_ladder(ctx, stage_ids=["V0"])
    assert report["stages"][0]["status"] == ladder.STATUS_BLOCKED


def test_v1_extracts_snapshot_metrics(ladder, tmp_path):
    ctx = _ctx(ladder, tmp_path)
    report = ladder.run_ladder(ctx, stage_ids=["V1"])
    stage = report["stages"][0]
    assert stage["status"] == ladder.STATUS_PASS
    assert stage["data"]["controller_type"] == "RM_NO_PLANNING_E"
    assert stage["data"]["udp_frames"] == 12


def test_v2_requires_nonzero_frames(ladder, tmp_path):
    def hwcollect(name, **kwargs):
        return {"status": "ok", "summary": {"frames": 0, "period_ms": 5}, "reason": ""}

    ctx = _ctx(ladder, tmp_path, hwcollect=hwcollect)
    report = ladder.run_ladder(ctx, stage_ids=["V2"])
    assert report["stages"][0]["status"] == ladder.STATUS_ERROR


def test_v2_reports_observed_period_not_configured(ladder, tmp_path):
    """实测周期必须按 frames 反算，不能把配置值当实测值报出去。"""
    def hwcollect(name, **kwargs):
        return {"status": "ok", "reason": "", "summary": {
            "frames": 2400, "expected_frames": 12000, "elapsed_s": 60.0,
            "period_ms": 5, "period_multiple_ok": True, "dropout_count": 0,
            "max_gap_ms": 0.0}}

    ctx = _ctx(ladder, tmp_path, hwcollect=hwcollect)
    report = ladder.run_ladder(ctx, stage_ids=["V2"])
    stage = report["stages"][0]
    assert stage["status"] == ladder.STATUS_PASS
    assert stage["data"]["observed_period_ms"] == pytest.approx(25.0)
    assert stage["data"]["observed_period_multiple"] == pytest.approx(5.0)
    assert stage["data"]["arrival_ratio"] == pytest.approx(0.2)
    assert "实测周期 25.0 ms" in stage["reason"]


def test_v2_period_not_multiple_is_quality_failure_not_blocking(ladder, tmp_path):
    """周期不合格 = **质量失败**：如实标红，但**不阻断**后续阶段。"""
    def hwcollect(name, **kwargs):
        return {"status": "ok", "reason": "", "summary": {
            "frames": 3000, "expected_frames": 12000, "elapsed_s": 60.0,
            "period_ms": 5, "period_multiple_ok": False, "dropout_count": 0}}

    ctx = _ctx(ladder, tmp_path, hwcollect=hwcollect)
    report = ladder.run_ladder(ctx, stage_ids=["V2", "V3"])
    statuses = {s["stage"]: s["status"] for s in report["stages"]}
    assert statuses["V2"] == ladder.STATUS_FAILED
    assert statuses["V3"] == ladder.STATUS_PASS      # 不被质量失败阻断
    assert report["status"] == ladder.STATUS_FAILED  # 整体状态如实降级
    assert report["summary"]["failed"] == 1


def test_v2_dropout_windows_is_quality_failure(ladder, tmp_path):
    def hwcollect(name, **kwargs):
        return {"status": "ok", "reason": "", "summary": {
            "frames": 2400, "expected_frames": 12000, "elapsed_s": 60.0,
            "period_ms": 5, "period_multiple_ok": True, "dropout_count": 3,
            "max_gap_ms": 150.0}}

    ctx = _ctx(ladder, tmp_path, hwcollect=hwcollect)
    report = ladder.run_ladder(ctx, stage_ids=["V2"])
    assert report["stages"][0]["status"] == ladder.STATUS_FAILED


def test_v2_zero_frames_is_blocking_error(ladder, tmp_path):
    """0 帧 = 反馈通道死了 → **阻断**（error），后续阶段不再跑。"""
    def hwcollect(name, **kwargs):
        return {"status": "ok", "reason": "", "summary": {
            "frames": 0, "expected_frames": 12000, "elapsed_s": 60.0,
            "period_ms": 5, "period_multiple_ok": True, "dropout_count": 60}}

    ctx = _ctx(ladder, tmp_path, hwcollect=hwcollect)
    report = ladder.run_ladder(ctx, stage_ids=["V2", "V3"])
    statuses = {s["stage"]: s["status"] for s in report["stages"]}
    assert statuses["V2"] == ladder.STATUS_ERROR
    assert statuses["V3"] == ladder.STATUS_SKIPPED


def test_v3_and_v4_report_motion_metrics(ladder, tmp_path):
    ctx = _ctx(ladder, tmp_path)
    report = ladder.run_ladder(ctx, stage_ids=["V3", "V4"])
    for stage in report["stages"]:
        assert stage["status"] == ladder.STATUS_PASS
        assert stage["data"]["arrival_error_deg"] == 0.045


def test_v5_writes_trajectory_and_runs_executor(ladder, tmp_path):
    seen = {}

    def executor(**kwargs):
        seen.update(kwargs)
        return {"status": "ok", "metrics": {"ticks": 500, "arrival_ok": True}}

    ctx = _ctx(ladder, tmp_path, executor=executor)
    report = ladder.run_ladder(ctx, stage_ids=["V5"])
    stage = report["stages"][0]
    assert stage["status"] == ladder.STATUS_PASS
    assert seen["mode"] == "live"
    assert os.path.exists(seen["trajectory_path"])
    with open(seen["trajectory_path"], encoding="utf-8") as fh:
        traj = json.load(fh)
    assert traj["units"] == {"times": "s", "points": "rad"}
    assert traj["verify"]["ok"] is True


def test_v7_defaults_to_current_pose_target(ladder, tmp_path):
    captured = {}

    def hwcollect(name, **kwargs):
        captured["name"] = name
        captured.update(kwargs)
        return {"status": "ok", "summary": {"max_deviation_mm": 0.03}}

    ctx = _ctx(ladder, tmp_path, hwcollect=hwcollect)
    report = ladder.run_ladder(ctx, stage_ids=["V7"])
    assert report["stages"][0]["status"] == ladder.STATUS_PASS
    assert captured["name"] == "tcp_repeatability"
    assert captured["target_deg"] == [0.0] * 7     # 默认 = 当前位姿（不乱动）


# ------------------------------------------------------------------ 覆盖表
def test_coverage_lists_every_criterion(ladder, tmp_path):
    ctx = _ctx(ladder, tmp_path)
    report = ladder.run_ladder(ctx, stage_ids=["V0", "V1", "V3"])
    coverage = report["coverage"]
    assert set(coverage) == set(ladder.ALL_CRITERIA)
    assert coverage["CC5"]["passed_stages"] == ["V0"]
    assert coverage["CC6"]["passed_stages"] == ["V1"]
    assert coverage["CC2"]["passed_stages"] == ["V3"]
    # 没跑的条款必须仍然 open
    assert coverage["CC7"]["open"] is True


def test_markdown_report_contains_stage_table(ladder, tmp_path):
    ctx = _ctx(ladder, tmp_path)
    report = ladder.run_ladder(ctx, stage_ids=["V0"])
    md = ladder.render_markdown(report)
    assert "| V0 |" in md
    assert "验收条款覆盖" in md
    assert "CC5" in md


def test_write_report_creates_files(ladder, tmp_path):
    ctx = _ctx(ladder, tmp_path)
    report = ladder.run_ladder(ctx, stage_ids=["V0"])
    json_path, md_path = ladder.write_report(report, str(tmp_path / "out"))
    assert os.path.exists(json_path) and os.path.exists(md_path)
    with open(json_path, encoding="utf-8") as fh:
        assert json.load(fh)["schema"] == ladder.SCHEMA


# ------------------------------------------------------------------ 轨迹生成
def test_micro_trajectory_shape(ladder):
    base = [0.0, 42.0, 19.0, -22.0, -1.0, 5.0, -200.0]
    traj = ladder._build_micro_trajectory(base, joint_index=0, delta_deg=0.3,
                                          period_s=0.010, move_s=2.0, hold_s=0.5)
    times = traj["times"]
    points = traj["points"]
    assert len(times) == len(points)
    assert all(b > a for a, b in zip(times, times[1:]))          # 严格递增
    assert points[0] == [round(math.radians(v), 9) for v in base]  # 首点 = 当前位姿
    offsets = [math.degrees(p[0] - math.radians(base[0])) for p in points]
    assert abs(max(offsets) - 0.3) < 1e-6                        # 位移 = 设定值
    assert abs(offsets[-1]) < 1e-6                               # 回到原位
    assert traj["units"] == {"times": "s", "points": "rad"}
    assert traj["verify"]["ok"] is True
    # 其它关节**完全不动**
    for row in points:
        for i in range(1, 7):
            assert row[i] == pytest.approx(math.radians(base[i]))


def test_micro_trajectory_respects_joint_limit(ladder):
    """0.3° / 2 s 的速度远低于关节限值（3.1416 rad/s）。"""
    base = [0.0] * 7
    traj = ladder._build_micro_trajectory(base, 0, 0.3, period_s=0.010)
    pts = traj["points"]
    times = traj["times"]
    vel = max(abs((b[0] - a[0]) / (times[i + 1] - times[i]))
              for i, (a, b) in enumerate(zip(pts, pts[1:])))
    assert vel < 0.01            # rad/s


# ------------------------------------------------------------------ 共享连接
def test_shared_link_is_created_once_and_reused(ladder, tmp_path):
    """整轮验证**只建一次链**（控制器客户端连接数上限实测 5）。"""
    created = []

    def provider():
        created.append(1)
        return FakeLink(), None

    ctx = ladder.LadderContext(out_dir=str(tmp_path / "v"),
                               link_provider=provider,
                               probe_runner=_probe_ok,
                               hwcollect_runner=lambda name, **kw: {
                                   "status": "ok", "summary": {"frames": 10}},
                               subprocess_runner=lambda args, cwd=None: (
                                   0, json.dumps({"status": "ok", "checks": [
                                       {"name": "hw:symbols", "ok": True,
                                        "level": "INFO"}]}), "审计通过"),
                               executor_runner=lambda **kw: {"status": "ok",
                                                           "metrics": {}})
    report = ladder.run_ladder(ctx, stage_ids=["V1", "V2", "V3", "V4", "V5", "V6"])
    assert len(created) == 1, "共享句柄应只创建一次，实际 %d 次" % len(created)


def test_probe_stages_do_not_close_shared_link(ladder, tmp_path):
    """探针阶段复用共享句柄时**不得关闭**它（否则后续阶段无链可用）。"""
    seen = {}

    def probe(**kwargs):
        seen.update(kwargs)
        return _probe_ok(**kwargs)

    ctx = _ctx(ladder, tmp_path, probe=probe)
    ladder.run_ladder(ctx, stage_ids=["V1"])
    assert seen.get("close_link") is False
    assert seen.get("link_factory") is not None


def test_stage_blocks_when_connection_cannot_be_established(ladder, tmp_path):
    """连不上 → 该阶段 blocked（外部条件不具备），不伪造结论。"""
    ctx = ladder.LadderContext(out_dir=str(tmp_path / "v"),
                               link_provider=lambda: (None, {
                                   "status": "blocked",
                                   "reason": "客户端连接数已达上限"}),
                               probe_runner=_probe_ok,
                               hwcollect_runner=lambda name, **kw: {"status": "ok"},
                               subprocess_runner=lambda args, cwd=None: (0, "", ""),
                               executor_runner=lambda **kw: {"status": "ok"})
    report = ladder.run_ladder(ctx, stage_ids=["V1"])
    stage = report["stages"][0]
    assert stage["status"] == ladder.STATUS_BLOCKED
    assert "连接数" in stage["reason"]


# ------------------------------------------------------------------ 判据修正
def test_v6_judges_on_fk_conclusion_not_collect_status(ladder, tmp_path):
    """fk_cross 的**结论**不合格时，即使采集 status=ok 也必须判不合格。

    回归（2026-11-19 真机实测）：FK 交叉校验最大位置偏差 78.1 mm（阈值 1 mm），
    但采集层 status 仍是 "ok" —— 只看 status 会把不合格误报成通过。
    """
    def hwcollect(name, **kwargs):
        return {"status": "ok", "reason": "", "summary": {
            "samples": 20, "max_pos_dev_mm": 78.12588158244492,
            "max_ori_dev_deg": None, "ok": False, "violations": list(range(20)),
            "mean_dev_norm_mm": 78.1, "dev_spread_mm": 0.8,
            "dev_interpretation": "恒定偏置（TCP 未标定的典型特征）",
            "conclusion": "FAIL：最大位置偏差 78.1259 mm（阈值 1.0000 mm）"}}

    ctx = _ctx(ladder, tmp_path, hwcollect=hwcollect)
    report = ladder.run_ladder(ctx, stage_ids=["V6"])
    stage = report["stages"][0]
    assert stage["status"] == ladder.STATUS_FAILED
    assert "78.12" in stage["reason"]
    assert "TCP" in stage["reason"]


def test_v6_passes_when_fk_conclusion_ok(ladder, tmp_path):
    def hwcollect(name, **kwargs):
        return {"status": "ok", "reason": "", "summary": {
            "samples": 20, "max_pos_dev_mm": 0.4, "max_ori_dev_deg": 0.01,
            "ok": True, "violations": []}}

    ctx = _ctx(ladder, tmp_path, hwcollect=hwcollect)
    report = ladder.run_ladder(ctx, stage_ids=["V6"])
    assert report["stages"][0]["status"] == ladder.STATUS_PASS


def test_v5_fails_when_cc2_thresholds_breached(ladder, tmp_path):
    """CC2：到位误差 < 0.1°、跟随误差 RMS ≤ 0.3° —— 超限即不合格。"""
    def executor(**kwargs):
        return {"status": "ok", "metrics": {"ticks": 501, "arrival_err_deg": 0.35,
                                            "follow_err_rms_deg": 0.8,
                                            "tick_jitter_p99_ms": 0.19}}

    ctx = _ctx(ladder, tmp_path, executor=executor)
    report = ladder.run_ladder(ctx, stage_ids=["V5"])
    stage = report["stages"][0]
    assert stage["status"] == ladder.STATUS_FAILED
    assert "0.35" in stage["reason"]


def test_v5_passes_within_cc2_thresholds(ladder, tmp_path):
    def executor(**kwargs):
        return {"status": "ok", "metrics": {"ticks": 501, "arrival_err_deg": 0.004,
                                            "follow_err_rms_deg": 0.05,
                                            "tick_jitter_p99_ms": 0.19}}

    ctx = _ctx(ladder, tmp_path, executor=executor)
    report = ladder.run_ladder(ctx, stage_ids=["V5"])
    assert report["stages"][0]["status"] == ladder.STATUS_PASS


def test_v7_fails_when_spread_exceeds_nominal(ladder, tmp_path):
    def hwcollect(name, **kwargs):
        return {"status": "ok", "reason": "", "summary": {
            "max_deviation_mm": 0.2, "nominal_repeatability_mm": 0.05,
            "within_nominal": False}}

    ctx = _ctx(ladder, tmp_path, hwcollect=hwcollect)
    report = ladder.run_ladder(ctx, stage_ids=["V7"])
    assert report["stages"][0]["status"] == ladder.STATUS_FAILED


def test_selected_stages_are_labelled_correctly(ladder, tmp_path):
    """只跑部分阶段时，报告里的阶段号/名称必须与实际执行的阶段对应。

    回归：曾用全局 STAGES 去 zip 部分结果 → 报告把 "V5 执行层" 显示成 "V0"。
    """
    ctx = _ctx(ladder, tmp_path)
    report = ladder.run_ladder(ctx, stage_ids=["V5", "V6"])
    assert [s["stage"] for s in report["stages"]] == ["V5", "V6"]
    assert report["stages"][0]["title"] == ladder.STAGE_INDEX["V5"].title
    assert report["stages"][1]["title"] == ladder.STAGE_INDEX["V6"].title


def test_evidence_artifact_drops_volatile_paths(ladder, tmp_path):
    """证据工件必须去掉易变路径（trajectory 绝对路径等），保留判据与数值。"""
    ctx = _ctx(ladder, tmp_path)
    report = ladder.run_ladder(ctx, stage_ids=["V1"])
    evidence = ladder.build_evidence(report)
    assert evidence["schema"] == "rmctl.rm75_validation_evidence/1"
    assert evidence["stages"][0]["id"] == "V1"
    assert "trajectory" not in (evidence["stages"][0].get("data") or {})
    # 可 JSON 化（入库前的硬要求）
    json.dumps(evidence, ensure_ascii=False)


# ------------------------------------------------------------------ 证据合并
def test_evidence_merge_keeps_last_run_per_stage(ladder, tmp_path):
    """同一阶段多次执行 → 保留最后一次，并记执行次数。"""
    merge_path = os.path.join(REPO_ROOT, "scripts", "rm75_evidence_merge.py")
    spec = importlib.util.spec_from_file_location("rm75_evidence_merge", merge_path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)

    def _report(status, reason):
        return {"arm": "left", "endpoint": {"ip": "192.168.1.18", "port": 8080},
                "status": status, "finished_at": "2026-11-19T10:00:00",
                "params": {}, "coverage": {},
                "stages": [{"id": "V1", "title": "连接与状态快照", "status": status,
                            "risk": "none", "moves_arm": False, "reason": reason,
                            "evidence": [], "data": {"trajectory": "/tmp/x.json"}}]}

    merged = module.merge([_report("error", "第一次失败"), _report("pass", "第二次通过")],
                          ["a.json", "b.json"])
    stage = merged["stages"][0]
    assert stage["status"] == "pass"
    assert stage["reason"] == "第二次通过"
    assert stage["runs"] == 2
    assert "trajectory" not in stage["data"]
    json.dumps(merged, ensure_ascii=False)
