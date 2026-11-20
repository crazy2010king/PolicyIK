"""S11-T5 四类基线**诚实回填**的离线一致性护栏（可执行，不连真机）。

本测试钉死 S11-T5 的核心纪律：

1. **回归段 verdict 与四张表的 status 逐项一致** —— 一键回归的 ``baselines``
   段 verdict **不得**与 YAML 里的 status 脱节（例如表是 blocked 而段报
   pass，就是「为了求 pass 改 status」的镜像错误）。
2. **status != measured 的表里，实测值字段必须保持 null** —— 禁止占位数值
   （``zero_baseline`` 的零位/固件/示教器/照片、``measured_limits`` 的
   acc/jerk/重复性、``version_matrix`` 的固件/示教器/镜像标签）。
   唯一例外是 ``zero_baseline.fk_recheck``：S11-T4 的真机 FK 复测**就是**
   本表要求的归档内容之一，其数值必须是**工件的逐字副本**（不得四舍五入、
   不得编造）—— 本文件对此有专门断言。
3. **每张 blocked 表都必须自证**（``blocked.reason`` + 非空
   ``blocked.attempted_actions``），否则 ``rmctl baseline check`` 直接报错。
4. **判定来源唯一** = ``rmctl.baseline.validate_all``：其对四表的判定必须
   与 YAML 字面一致（不多报、不少报）。

全部只读：只读仓库内 git 跟踪的文件（``workcell/baselines/*.yaml`` 与
``workcell/baselines/evidence/rm75_s11_fk_recheck.json``），不连真机、
不发任何运动指令。回归报告写到工作区的 ``build/reports/``（gitignore 目录，
由本测试自己生成，不作为仓库内容依赖）。
"""
from __future__ import annotations

import json
import os
import shutil
import subprocess
import sys

import pytest
import yaml

from rmctl import baseline as bl
from rmctl import paths
from rmctl.regression import baselines as blp

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(TESTS_DIR, "..", "..", "..", ".."))

#: 一键回归 baselines 段的报告（本测试自己生成）
REPORT_REL = os.path.join("build", "reports", "s11_baselines.json")

#: S11-T4 的真机 FK 复测工件（数值真值来源）
EVIDENCE_REL = os.path.join("workcell", "baselines", "evidence",
                             "rm75_s11_fk_recheck.json")
WORKCELL_REL = os.path.join("workcell", "rm_workcell.yaml")

#: 各表中「status != measured 时必须为 null」的实测值字段（按表分列）
NULL_WHEN_UNMEASURED = {
    "zero_baseline": ("firmware", "teachpendant_version"),
    "measured_limits": (),
    "version_matrix": ("controller_firmware", "teachpendant",
                       "container_image"),
    "network_plan": (),
}


# ---------------------------------------------------------------------------
# 夹具 / 工具
# ---------------------------------------------------------------------------
def _yaml_path(ws, kind):
    return os.path.join(paths.baselines_dir(ws), "%s.yaml" % kind)


def _read_yaml(ws, kind):
    with open(_yaml_path(ws, kind), "r", encoding="utf-8") as fh:
        return yaml.safe_load(fh) or {}


def _write_yaml(ws, kind, data):
    with open(_yaml_path(ws, kind), "w", encoding="utf-8") as fh:
        yaml.safe_dump(data, fh, allow_unicode=True, sort_keys=False)


def _expected_verdict(statuses):
    """段级期望 verdict —— 从 status 推导，不写死 blocked。"""
    return "pass" if all(s == "measured" for s in statuses) else "blocked"


@pytest.fixture(scope="module")
def regression_report():
    """跑一键回归的 baselines 段，返回 ``(report, exit_code)``。

    走**真实入口** ``scripts/rm_regression.py --only baselines``（与计划里
    的命令一字不差）；退出码 3 = 存在 blocked 段（本轮预期），4 = 有 fail
    段（真回归，必须为 0/3）。
    """
    script = os.path.join(REPO_ROOT, "scripts", "rm_regression.py")
    proc = subprocess.run(
        [sys.executable, script, "--only", "baselines", "--out", REPORT_REL],
        cwd=REPO_ROOT, capture_output=True, text=True)
    out_path = os.path.join(REPO_ROOT, REPORT_REL)
    assert os.path.isfile(out_path), (
        "一键回归未生成报告：%s（stdout: %s / stderr: %s）"
        % (out_path, proc.stdout[-800:], proc.stderr[-800:]))
    with open(out_path, "r", encoding="utf-8") as fh:
        report = json.load(fh)
    return report, proc.returncode


@pytest.fixture(scope="module")
def evidence():
    """S11-T4 真机 FK 复测工件。"""
    path = os.path.join(REPO_ROOT, EVIDENCE_REL)
    assert os.path.isfile(path), "缺少 S11-T4 证据工件：%s" % path
    with open(path, "r", encoding="utf-8") as fh:
        return json.load(fh)


@pytest.fixture(scope="module")
def statuses():
    """四张表的 YAML status（逐字）。"""
    return {kind: _read_yaml(REPO_ROOT, kind).get("status") for kind in bl.KINDS}


# ---------------------------------------------------------------------------
# 1. 回归段 verdict 与四张表 status 逐项一致
# ---------------------------------------------------------------------------
def test_regression_report_exists_with_baselines_segment(regression_report):
    """报告存在，且含 baselines 段。"""
    report, _ = regression_report
    assert report.get("segments"), "回归报告没有 segments"
    ids = [seg.get("id") for seg in report.get("segments") or []]
    assert "baselines" in ids, ids


def test_segment_exit_code_is_blocked_not_fail(regression_report):
    """退出码 3（存在 blocked 段）是本轮预期结局；4（fail）才是回归。"""
    _, code = regression_report
    assert code in (0, 3), "baselines 段退出码应为 0/3（blocked 不算失败），实际 %r" % code


def test_segment_verdict_matches_table_statuses(regression_report, statuses):
    """段级 verdict 必须与四张表的 status 一致（不得脱节）。"""
    report, _ = regression_report
    segment = next(seg for seg in report["segments"] if seg.get("id") == "baselines")
    expected = _expected_verdict([statuses[kind] for kind in bl.KINDS])
    assert segment.get("verdict") == expected, (
        "段级 verdict=%r 与四表 status 推出的期望 %r 不一致（statuses=%r）"
        % (segment.get("verdict"), expected, statuses))


def test_segment_rows_match_table_statuses(regression_report, statuses):
    """段内 rows 的逐类 status 必须与 YAML 逐字一致。"""
    report, _ = regression_report
    segment = next(seg for seg in report["segments"] if seg.get("id") == "baselines")
    rows = {row.get("kind"): row for row in segment.get("rows") or []}
    assert set(rows) == set(bl.KINDS), sorted(rows)
    for kind in bl.KINDS:
        assert rows[kind].get("status") == statuses[kind], \
            "%s 的段内 status=%r 与 YAML 的 %r 不一致" % (
                kind, rows[kind].get("status"), statuses[kind])


def test_top_level_status_matches_segment(regression_report):
    """顶层 status 与段级 verdict 必须同向（不得顶层报 ok 而段是 blocked）。"""
    report, _ = regression_report
    segment = next(seg for seg in report["segments"] if seg.get("id") == "baselines")
    if segment.get("verdict") == "blocked":
        assert report.get("status") == "blocked", report.get("status")
        assert report.get("verdict") == "blocked", report.get("verdict")
    elif segment.get("verdict") == "pass":
        assert report.get("status") == "ok", report.get("status")


# ---------------------------------------------------------------------------
# 2. status != measured 的表里，实测值字段必须保持 null
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("kind", list(bl.KINDS))
def test_unmeasured_tables_keep_value_fields_null(kind):
    """禁止占位数值：未 measured 的表里实测值字段必须为 null。"""
    data = _read_yaml(REPO_ROOT, kind)
    status = data.get("status")
    if status == "measured":
        pytest.skip("%s 已是 measured（值字段非空是合法的）" % kind)
    for field in NULL_WHEN_UNMEASURED.get(kind, ()):
        assert data.get(field) is None, \
            "%s.%s 在 status=%s 时必须为 null（实际 %r）" % (kind, field, status,
                                                           data.get(field))


def test_measured_limits_joint_values_stay_null():
    """measured_limits：7 个关节的 acc/jerk 与 TCP 重复性必须仍是 null。"""
    data = _read_yaml(REPO_ROOT, "measured_limits")
    if data.get("status") == "measured":
        pytest.skip("measured_limits 已是 measured")
    assert len(data.get("joints") or []) == 7
    for joint in data["joints"]:
        for field in ("acceleration", "jerk"):
            item = joint.get(field) or {}
            assert item.get("value") is None, \
                "%s.%s.value 在 status=blocked 时必须为 null（实际 %r）" % (
                    joint.get("name"), field, item.get("value"))
    tcp = data.get("tcp_repeatability") or {}
    assert tcp.get("value_mm") is None, \
        "tcp_repeatability.value_mm 在 status=blocked 时必须为 null（实际 %r）" % \
        tcp.get("value_mm")


def test_zero_baseline_zero_degrees_and_photos_stay_empty():
    """zero_baseline：每关节零位与凹槽照片必须仍是 null / 空。"""
    data = _read_yaml(REPO_ROOT, "zero_baseline")
    if data.get("status") == "measured":
        pytest.skip("zero_baseline 已是 measured")
    for joint in data.get("joints") or []:
        assert joint.get("zero_deg") is None, \
            "%s.zero_deg 在 status=blocked 时必须为 null" % joint.get("name")
        assert joint.get("groove_photo") is None, \
            "%s.groove_photo 在 status=blocked 时必须为 null" % joint.get("name")
    assert not (data.get("groove_photos") or []), \
        "groove_photos 在 status=blocked 时必须为空列表"


def test_zero_baseline_fk_numbers_are_verbatim_from_evidence(evidence):
    """fk_recheck 的数值必须是 S11-T4 工件的**逐字副本**（不得编造/取整）。"""
    data = _read_yaml(REPO_ROOT, "zero_baseline")
    fk = data.get("fk_recheck") or {}
    if fk.get("max_pos_dev_mm") is None:
        pytest.skip("zero_baseline 未回填 fk_recheck 数值")
    assert evidence.get("status") == "ok", \
        "工件不是 ok（blocked 工件不得作为数值来源）：%r" % evidence.get("status")
    src = (evidence.get("fk_recheck") or {}).get("raw") or {}
    assert fk.get("max_pos_dev_mm") == src.get("max_pos_dev_mm"), \
        "fk_recheck.max_pos_dev_mm 与工件不一致：%r vs %r" % (
            fk.get("max_pos_dev_mm"), src.get("max_pos_dev_mm"))
    assert fk.get("max_ori_dev_deg") == src.get("max_ori_dev_deg"), \
        "fk_recheck.max_ori_dev_deg 与工件不一致：%r vs %r" % (
            fk.get("max_ori_dev_deg"), src.get("max_ori_dev_deg"))
    assert fk.get("residual_kind") == (evidence.get("fk_recheck") or {}).get("residual_kind"), \
        "fk_recheck.residual_kind 与工件不一致"
    assert fk.get("samples") == (evidence.get("fk_recheck") or {}).get("samples"), \
        "fk_recheck.samples 与工件不一致"


def test_no_calibration_offset_written_in_workcell(evidence):
    """工件声明未写偏置 → 工位标定里就必须是 null（禁止伪造标定）。"""
    disposition = evidence.get("branch_disposition") or {}
    if disposition.get("calibration_written") is not False:
        pytest.skip("工件声明写了偏置，本断言不适用")
    with open(os.path.join(REPO_ROOT, WORKCELL_REL), "r", encoding="utf-8") as fh:
        calibration = (yaml.safe_load(fh) or {}).get("calibration") or {}
    assert calibration.get("tcp") is None, "calibration.tcp 必须为 null：%r" % calibration.get("tcp")
    assert calibration.get("base_to_base") is None, \
        "calibration.base_to_base 必须为 null：%r" % calibration.get("base_to_base")


# ---------------------------------------------------------------------------
# 3. 每张 blocked 表都要自证
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("kind", list(bl.KINDS))
def test_blocked_tables_self_document(kind):
    """status=blocked → blocked.reason 与 blocked.attempted_actions 必填。"""
    data = _read_yaml(REPO_ROOT, kind)
    if data.get("status") != "blocked":
        pytest.skip("%s 不是 blocked" % kind)
    blocked = data.get("blocked") or {}
    reason = str(blocked.get("reason") or "").strip()
    assert reason, "%s 缺 blocked.reason" % kind
    actions = blocked.get("attempted_actions")
    assert isinstance(actions, list) and actions, \
        "%s 缺 blocked.attempted_actions（禁止空口说被阻塞）" % kind
    for action in actions:
        assert str(action).strip(), "%s 有空的 attempted_actions 条目" % kind
    assert str(blocked.get("next_action") or "").strip(), \
        "%s 缺 blocked.next_action（必须给出可粘贴的下一步）" % kind


def test_projection_never_claims_measured_for_unmeasured_tables(statuses):
    """投影自身（表）的措辞不得把 blocked 说成 measured（措辞纪律）。

    注：YAML 原文里可能出现历史事实（如「最小动作验证通过」），不参与判定；
    这里只判定**投影自撰**的结论列。
    """
    rows, _ = blp.collect_rows(REPO_ROOT)
    assert len(rows) == len(bl.KINDS), rows
    for row in rows:
        kind = row.get("kind")
        assert row.get("status") == statuses[kind], (kind, row.get("status"))
        if row.get("status") == "measured":
            continue
        claim = str(row.get("claim") or "")
        assert claim, kind
        for phrase in (blp.FORBIDDEN_CLAIM_PHRASES
                       + blp.FORBIDDEN_STATUS_TOKENS):
            assert phrase not in claim, (kind, phrase)


# ---------------------------------------------------------------------------
# 4. 判定来源唯一：validate_all 与 YAML 一致
# ---------------------------------------------------------------------------
def test_validate_all_agrees_with_yaml(statuses):
    """rmctl.baseline.validate_all 的判定必须与 YAML 字面一致。"""
    report = bl.validate_all(REPO_ROOT)
    assert report["ok"], "validate_all 判定不通过：%r" % report.get("issues")
    for kind in bl.KINDS:
        entry = report["kinds"][kind]
        assert entry["present"], "%s 缺失" % kind
        assert entry["status"] == statuses[kind], \
            "%s 的 validate_all status=%r 与 YAML 的 %r 不一致" % (
                kind, entry["status"], statuses[kind])
        errors = [i for i in entry["issues"] if i["level"] == "error"]
        assert not errors, "%s 存在 error 级 issue：%r" % (kind, errors)


def test_baseline_check_cli_exits_zero():
    """`python3 -m rmctl baseline check` 必须退出码 0（走 rmenv 环境）。"""
    cmd = "source scripts/rmenv.sh >/dev/null 2>&1 && python3 -m rmctl baseline check"
    proc = subprocess.run(["bash", "-lc", cmd], cwd=REPO_ROOT,
                          capture_output=True, text=True)
    assert proc.returncode == 0, \
        "baseline check 退出码 %r：\n%s\n%s" % (proc.returncode, proc.stdout[-2000:],
                                                 proc.stderr[-2000:])


def test_version_numbers_are_monotonic():
    """基线修订号必须单调递增（≥1 的整数）。"""
    for kind in bl.KINDS:
        version = _read_yaml(REPO_ROOT, kind).get("version")
        assert isinstance(version, int) and version >= 1, \
            "%s 的 version 必须是 ≥1 的整数（实际 %r）" % (kind, version)


# ---------------------------------------------------------------------------
# 5. 负例（Q7）：伪造 measured 主张 / 缺自证 → 必须被判出来
# ---------------------------------------------------------------------------
def _copy_workspace(tmp_path):
    """把四张基线 YAML 复制到临时工作区（负例用）。"""
    ws = str(tmp_path / "ws")
    target = os.path.join(ws, "workcell", "baselines")
    os.makedirs(target, exist_ok=True)
    for kind in bl.KINDS:
        shutil.copy2(_yaml_path(REPO_ROOT, kind),
                     os.path.join(target, "%s.yaml" % kind))
    return ws


def _error_codes(report):
    return {(i.get("code"), i.get("path")) for i in report.get("issues") or []
            if i.get("level") == "error"}


def test_placeholder_value_in_blocked_table_is_rejected(tmp_path):
    """负例：blocked 的 measured_limits 里填了占位 acc → 必须报 error。"""
    ws = _copy_workspace(tmp_path)
    data = _read_yaml(ws, "measured_limits")
    data["joints"][0]["acceleration"]["value"] = 5.0
    _write_yaml(ws, "measured_limits", data)
    report = bl.validate_all(ws)
    assert not report["ok"], "填了占位数值却判定通过"
    codes = {code for code, _ in _error_codes(report)}
    assert "unmeasured_value_present" in codes, sorted(codes)


def test_blocked_without_reason_is_rejected(tmp_path):
    """负例：blocked 但没有 reason → 必须报 error。"""
    ws = _copy_workspace(tmp_path)
    data = _read_yaml(ws, "version_matrix")
    data["blocked"] = {"attempted_actions": ["示例"]}
    _write_yaml(ws, "version_matrix", data)
    report = bl.validate_all(ws)
    codes = {code for code, _ in _error_codes(report)}
    assert "blocked_reason_missing" in codes, sorted(codes)


def test_blocked_without_attempted_actions_is_rejected(tmp_path):
    """负例：blocked 但 attempted_actions 为空 → 必须报 error。"""
    ws = _copy_workspace(tmp_path)
    data = _read_yaml(ws, "network_plan")
    data["blocked"] = {"reason": "示例原因", "attempted_actions": []}
    _write_yaml(ws, "network_plan", data)
    report = bl.validate_all(ws)
    codes = {code for code, _ in _error_codes(report)}
    assert "blocked_actions_missing" in codes, sorted(codes)


def test_placeholder_value_makes_regression_segment_fail(tmp_path):
    """负例：伪造实测值 → 回归 baselines 段必须判 fail（不得静默 pass）。"""
    ws = _copy_workspace(tmp_path)
    data = _read_yaml(ws, "measured_limits")
    data["joints"][1]["jerk"]["value"] = 30.0
    _write_yaml(ws, "measured_limits", data)
    reports_dir = str(tmp_path / "reports")
    result = blp.run_baselines(repo_root=ws, ws=ws, reports_dir=reports_dir)
    assert result["verdict"] == "fail", result.get("reason")
    assert "measured_limits" in result.get("reason", "")


def test_version_must_be_monotonic():
    """负例：修订号倒退 → 必须报 error。"""
    issues = bl.check_version_monotonic({"version": 5}, {"version": 5}, "zero_baseline")
    assert any(i["code"] == "version_not_monotonic" for i in issues), issues


def test_segment_projection_is_read_only(tmp_path):
    """负例面：投影段不得改写 workcell/baselines（只读投影）。"""
    ws = _copy_workspace(tmp_path)
    before = {kind: _read_yaml(ws, kind) for kind in bl.KINDS}
    blp.run_baselines(repo_root=ws, ws=ws, reports_dir=str(tmp_path / "reports"))
    for kind in bl.KINDS:
        assert _read_yaml(ws, kind) == before[kind], "%s 被投影段改写了" % kind
