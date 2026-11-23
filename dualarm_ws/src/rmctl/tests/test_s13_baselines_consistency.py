"""S13-T5 三表**诚实回填** + 文档同步 + baselines 段一致性复核的离线护栏。

本文件钉死 S13-T5 的纪律（全部**只读**，不连真机、不发任何运动指令）：

1. **四表 status 逐字** —— ``measured_limits=measured``（S13 已实测升格），
   ``zero_baseline`` / ``version_matrix`` / ``network_plan`` 仍为 ``blocked``
   （全表口径未测齐）。**不得**为了求 pass 改 status。
2. **blocked 表的正式值字段必须仍是 null / 空** —— ``version_matrix`` 的
   ``controller_firmware`` / ``teachpendant`` / ``container_image``，
   ``zero_baseline`` 的 ``firmware`` / ``teachpendant_version`` / 每关节
   ``zero_deg`` / ``groove_photos``。
3. **三张 blocked 表的 ``partial_observations`` 里的 S13 结论必须与两份 S13
   工件逐字一致**（版本串、机型判定、容器镜像、右臂 reachable 布尔）——
   禁止手填数值，禁止与工件脱节。
4. **一键回归 ``baselines`` 段 verdict 与四表 status 同向**（段内 rows 的 status
   与 YAML 逐字一致、顶层 status 与段级 verdict 同向、退出码 ∈ {0,3}）。
5. **正本文档必须出现 S13 的关键值**（ctrl_version / ctrl_build_time /
   容器镜像 / 右臂 IP / 白名单符号真名），且不得改写 S11 已钉死的 FK 结论。

覆盖的负例（Q7）：

* ``version_matrix`` 的正式字段被填成非 null → 护栏必须报出违规；
* 段级 verdict 被改成 ``pass``（含“把 blocked 表的 status 偷改成 measured”
  这种绕过）→ 必须被判失败。
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
REPORT_REL = os.path.join("build", "reports", "s13_baselines.json")

#: 两份 S13 真机工件（数值真值来源）
VERSIONS_EVIDENCE_REL = os.path.join(
    "workcell", "baselines", "evidence", "rm75_s13_versions_connectivity.json")
ACC_JERK_EVIDENCE_REL = os.path.join(
    "workcell", "baselines", "evidence", "rm75_s13_acc_jerk.json")

#: 正本文档（git 跟踪；``docs/code_doc/`` 是外部仓库的符号链接副本，不得改）
ARCHIVE_DOC = os.path.join("docs", "真机基线与实测限值归档说明.md")
LADDER_DOC = os.path.join("docs", "RM75-B真机验证执行情况与使用说明.md")

#: 本轮期望的四表 status（逐字）
EXPECTED_STATUS = {
    "zero_baseline": "blocked",
    "measured_limits": "measured",
    "version_matrix": "blocked",
    "network_plan": "blocked",
}

#: blocked 表里必须保持 null / 空的正式值字段
NULL_WHEN_UNMEASURED = {
    "zero_baseline": ("firmware", "teachpendant_version"),
    "version_matrix": ("controller_firmware", "teachpendant", "container_image"),
    "measured_limits": (),
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


def _read_json(rel):
    path = os.path.join(REPO_ROOT, rel)
    assert os.path.isfile(path), "缺少 S13 证据工件：%s" % path
    with open(path, "r", encoding="utf-8") as fh:
        return json.load(fh)


def _read_text(rel):
    path = os.path.join(REPO_ROOT, rel)
    assert os.path.isfile(path), "缺少文件：%s" % path
    with open(path, "r", encoding="utf-8") as fh:
        return fh.read()


def _expected_verdict(statuses):
    """段级期望 verdict —— 从 status 推导，不写死 blocked。"""
    return "pass" if all(s == "measured" for s in statuses) else "blocked"


@pytest.fixture(scope="module")
def versions_artifact():
    """S13 版本/连通性只读工件。"""
    return _read_json(VERSIONS_EVIDENCE_REL)


@pytest.fixture(scope="module")
def acc_jerk_artifact():
    """S13 joint2..joint7 阶梯 acc/jerk 工件。"""
    return _read_json(ACC_JERK_EVIDENCE_REL)


@pytest.fixture(scope="module")
def statuses():
    return {kind: _read_yaml(REPO_ROOT, kind).get("status") for kind in bl.KINDS}


@pytest.fixture(scope="module")
def regression_report():
    """跑一键回归的 baselines 段，返回 ``(report, exit_code)``。

    走**真实入口** ``scripts/rm_regression.py --only baselines``；
    退出码 3 = 存在 blocked 段（本轮预期），4 = 有 fail 段（真回归，必须为 0/3）。
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


def _copy_workspace(tmp_path):
    """把四张基线 YAML + 关节限值文件复制到临时工作区（负例用）。"""
    ws = str(tmp_path / "ws")
    target = os.path.join(ws, "workcell", "baselines")
    os.makedirs(target, exist_ok=True)
    for kind in bl.KINDS:
        shutil.copy2(_yaml_path(REPO_ROOT, kind),
                     os.path.join(target, "%s.yaml" % kind))
    limits_dst = os.path.join(ws, "dualarm_ws", "src", "rm_description", "config")
    os.makedirs(limits_dst, exist_ok=True)
    shutil.copy2(
        os.path.join(REPO_ROOT, "dualarm_ws", "src", "rm_description", "config",
                     "joint_limits.yaml"),
        os.path.join(limits_dst, "joint_limits.yaml"))
    return ws


def _write_yaml(ws, kind, data):
    with open(_yaml_path(ws, kind), "w", encoding="utf-8") as fh:
        yaml.safe_dump(data, fh, allow_unicode=True, sort_keys=False)


def _null_field_violations(kind, data):
    """返回该表里「status != measured 却填了正式值字段」的违规列表。"""
    violations = []
    if data.get("status") == "measured":
        return violations
    for field in NULL_WHEN_UNMEASURED.get(kind, ()):
        if data.get(field) is not None:
            violations.append("%s.%s 非 null（实际 %r）" % (kind, field, data.get(field)))
    if kind == "zero_baseline":
        for joint in data.get("joints") or []:
            if joint.get("zero_deg") is not None:
                violations.append("zero_baseline.%s.zero_deg 非 null" % joint.get("name"))
            if joint.get("groove_photo") is not None:
                violations.append("zero_baseline.%s.groove_photo 非 null" % joint.get("name"))
        if data.get("groove_photos"):
            violations.append("zero_baseline.groove_photos 非空")
    return violations


# ---------------------------------------------------------------------------
# 1. 四表 status 逐字
# ---------------------------------------------------------------------------
def test_table_statuses_are_exact(statuses):
    """四表 status 必须逐字等于本轮期望（measured_limits 升格，其余仍 blocked）。"""
    assert statuses == EXPECTED_STATUS, statuses


def test_validate_all_agrees_with_yaml(statuses):
    """基线校验的判定必须与 YAML 字面一致（不多报、不少报）。"""
    report = bl.validate_all(REPO_ROOT)
    assert report["ok"], "validate_all 判定不通过：%r" % report.get("issues")
    for kind in bl.KINDS:
        entry = report["kinds"][kind]
        assert entry["status"] == statuses[kind], \
            "%s 的 validate_all status=%r 与 YAML 的 %r 不一致" % (
                kind, entry["status"], statuses[kind])
        errors = [i for i in entry["issues"] if i["level"] == "error"]
        assert not errors, "%s 存在 error 级 issue：%r" % (kind, errors)


def test_baseline_check_cli_exits_zero():
    """`python3 -m rmctl baseline check` 必须退出码 0。"""
    cmd = "source scripts/rmenv.sh >/dev/null 2>&1 && python3 -m rmctl baseline check"
    proc = subprocess.run(["bash", "-lc", cmd], cwd=REPO_ROOT,
                          capture_output=True, text=True)
    assert proc.returncode == 0, \
        "baseline check 退出码 %r：\n%s\n%s" % (proc.returncode, proc.stdout[-2000:],
                                                 proc.stderr[-2000:])


# ---------------------------------------------------------------------------
# 2. blocked 表的正式值字段必须仍是 null / 空
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("kind", list(bl.KINDS))
def test_unmeasured_tables_keep_value_fields_null(kind):
    """禁止占位数值：blocked 表里实测值字段必须为 null / 空。"""
    violations = _null_field_violations(kind, _read_yaml(REPO_ROOT, kind))
    assert not violations, violations


# ---------------------------------------------------------------------------
# 3. partial_observations 的 S13 结论必须与工件逐字一致
# ---------------------------------------------------------------------------
def test_version_matrix_partial_observations_match_artifact(versions_artifact):
    """version_matrix 的版本/机型/镜像读数必须与 S13 工件逐字一致。"""
    assert versions_artifact["status"] == "ok", \
        "S13 版本工件不是 ok（blocked 工件不得作为数值来源）：%r" % \
        versions_artifact.get("status")
    obs = (_read_yaml(REPO_ROOT, "version_matrix").get("partial_observations") or {})
    block = obs.get("s13_controller_software_read") or {}
    assert block.get("controller_software") == versions_artifact["controller_software"], \
        "controller_software 与工件不一致：%r vs %r" % (
            block.get("controller_software"), versions_artifact["controller_software"])
    assert block.get("robot_info") == versions_artifact["robot_info"], \
        "robot_info 与工件不一致：%r vs %r" % (
            block.get("robot_info"), versions_artifact["robot_info"])
    img = (obs.get("s13_container_image") or {}).get("value")
    assert img == versions_artifact["container_image"], \
        "容器镜像与工件不一致：%r vs %r" % (img, versions_artifact["container_image"])
    for name in ("ctrl_version", "ctrl_build_time", "plan_version",
                 "plan_build_time", "algorithm_version", "product_version"):
        assert (block.get("controller_software") or {}).get(name), \
            "版本串 %s 缺失" % name


def test_zero_baseline_software_versions_match_artifact(versions_artifact):
    """zero_baseline 的 software_versions 观测必须与 S13 工件逐字一致。"""
    obs = (_read_yaml(REPO_ROOT, "zero_baseline").get("partial_observations") or {})
    block = obs.get("software_versions") or {}
    sw = versions_artifact["controller_software"]
    assert block.get("ctrl_version") == sw["ctrl_version"]
    assert block.get("ctrl_build_time") == sw["ctrl_build_time"]
    assert block.get("plan_version") == sw["plan_version"]
    assert block.get("plan_build_time") == sw["plan_build_time"]
    assert block.get("algorithm_version") == sw["algorithm_version"]
    assert block.get("product_version") == sw["product_version"]
    assert block.get("robot_controller_version") == \
        versions_artifact["robot_info"]["robot_controller_version"]


def test_network_plan_connectivity_matches_artifact(versions_artifact):
    """network_plan 的双臂 reachable 布尔必须与 S13 工件逐字一致。"""
    obs = (_read_yaml(REPO_ROOT, "network_plan").get("partial_observations") or {})
    block = obs.get("s13_connectivity") or {}
    for side, key in (("left", "left_connectivity"), ("right", "right_connectivity")):
        assert (block.get(side) or {}).get("reachable") == \
            versions_artifact[key]["reachable"], \
            "%s 的 reachable 与工件不一致" % side
    assert (block.get("right") or {}).get("ip") == \
        versions_artifact["right_connectivity"]["ip"] == "192.168.1.19"


def test_measured_limits_values_match_acc_jerk_artifact(acc_jerk_artifact):
    """measured_limits 的 joint2..joint7 acc 值必须来自 S13 工件（保守下界）。"""
    assert acc_jerk_artifact["status"] == "ok"
    data = _read_yaml(REPO_ROOT, "measured_limits")
    assert data.get("status") == "measured"
    by_name = {j.get("name"): j for j in data.get("joints") or []}
    for name, entry in (acc_jerk_artifact.get("joints") or {}).items():
        joint = by_name.get(name)
        assert joint is not None, "measured_limits 缺关节 %s" % name
        assert joint["acceleration"]["value"] <= entry["acceleration_rad_s2"] + 0.0, \
            "%s 的 acc 值高于实测包络（不得放宽）" % name
        assert joint["jerk"]["value"] <= entry["jerk_rad_s3"] + 0.0, \
            "%s 的 jerk 值高于实测包络（不得放宽）" % name


# ---------------------------------------------------------------------------
# 4. 一键回归 baselines 段与四表 status 同向
# ---------------------------------------------------------------------------
def test_segment_exit_code_is_blocked_not_fail(regression_report):
    _, code = regression_report
    assert code in (0, 3), "baselines 段退出码应为 0/3（blocked 不算失败），实际 %r" % code


def test_segment_verdict_matches_table_statuses(regression_report, statuses):
    report, _ = regression_report
    segment = next(seg for seg in report["segments"] if seg.get("id") == "baselines")
    expected = _expected_verdict([statuses[kind] for kind in bl.KINDS])
    assert segment.get("verdict") == expected, (
        "段级 verdict=%r 与四表 status 推出的期望 %r 不一致（statuses=%r）"
        % (segment.get("verdict"), expected, statuses))


def test_segment_rows_match_table_statuses(regression_report, statuses):
    report, _ = regression_report
    segment = next(seg for seg in report["segments"] if seg.get("id") == "baselines")
    rows = {row.get("kind"): row for row in segment.get("rows") or []}
    assert set(rows) == set(bl.KINDS), sorted(rows)
    for kind in bl.KINDS:
        assert rows[kind].get("status") == statuses[kind], \
            "%s 的段内 status=%r 与 YAML 的 %r 不一致" % (
                kind, rows[kind].get("status"), statuses[kind])


def test_top_level_status_matches_segment(regression_report):
    report, _ = regression_report
    segment = next(seg for seg in report["segments"] if seg.get("id") == "baselines")
    if segment.get("verdict") == "blocked":
        assert report.get("status") == "blocked", report.get("status")
        assert report.get("verdict") == "blocked", report.get("verdict")
    elif segment.get("verdict") == "pass":
        assert report.get("status") == "ok", report.get("status")


def test_segment_projection_rows_follow_yaml_statuses():
    """投影层不得自带真相：段内行的 status 直接来自 YAML。"""
    rows, _ = blp.collect_rows(REPO_ROOT)
    assert len(rows) == len(bl.KINDS), rows
    for row in rows:
        kind = row.get("kind")
        assert row.get("status") == _read_yaml(REPO_ROOT, kind).get("status")


# ---------------------------------------------------------------------------
# 5. 正本文档必须出现 S13 的关键值
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("rel", [ARCHIVE_DOC, LADDER_DOC])
def test_docs_carry_s13_key_values(rel, versions_artifact):
    """两份正本文档都必须逐字出现 S13 的关键值。"""
    text = _read_text(rel)
    sw = versions_artifact["controller_software"]
    for value in (sw["ctrl_version"], sw["ctrl_build_time"], sw["plan_version"],
                  sw["algorithm_version"], sw["product_version"],
                  versions_artifact["container_image"]):
        assert value in text, "%s 缺 S13 关键值：%r" % (rel, value)
    assert "192.168.1.19" in text, "%s 缺右臂 IP" % rel
    assert "rm_get_arm_software_info" in text, \
        "%s 缺 SDK 读数符号真名 rm_get_arm_software_info" % rel


def test_archive_doc_states_the_symbol_name_truth():
    """归档正本必须写明符号名真值与白名单口径（笔误名不得作为事实）。"""
    text = _read_text(ARCHIVE_DOC)
    assert "rm_get_arm_software_info" in text
    assert "笔误" in text
    assert "support" in text and "core" in text


def test_ladder_doc_keeps_s11_fk_conclusion_untouched():
    """S11 已钉死的 FK 数值与 residual_kind 结论不得被改写。"""
    text = _read_text(LADDER_DOC)
    assert "orientation_dependent" in text
    assert "151.460" in text
    assert "179.965" in text
    assert "75.03" in text and "已作废" in text


def test_docs_have_no_unevidenced_passing_claims():
    """正本文档不得出现无证据的「达标 / 已复现」断言。"""
    forbidden = ("长跑已达标", "长跑达标", "连续 10 分钟已达标",
                 "断流已复现", "断流复现成功", "已复现断流",
                 "600 s 执行层长跑已完成", "执行层长跑已达标")
    for rel in (ARCHIVE_DOC, LADDER_DOC):
        text = _read_text(rel)
        for phrase in forbidden:
            assert phrase not in text, "%s 无证据断言：%r" % (rel, phrase)


# ---------------------------------------------------------------------------
# 6. 负例（Q7）
# ---------------------------------------------------------------------------
def test_negative_version_matrix_filled_value_is_detected():
    """负例：version_matrix 正式字段被填成非 null → 护栏必须报违规。"""
    data = _read_yaml(REPO_ROOT, "version_matrix")
    assert not _null_field_violations("version_matrix", data)
    broken = json.loads(json.dumps(data))
    broken["controller_firmware"] = "V1.7.1-f95e896"
    violations = _null_field_violations("version_matrix", broken)
    assert violations, "正式字段被填非 null 却未被护栏判出"
    assert "controller_firmware" in violations[0]


def test_negative_zero_baseline_zero_degree_is_detected():
    """负例：zero_baseline 的每关节 zero_deg 被填非 null → 护栏必须报违规。"""
    data = _read_yaml(REPO_ROOT, "zero_baseline")
    broken = json.loads(json.dumps(data))
    broken["joints"][0]["zero_deg"] = 0.0
    violations = _null_field_violations("zero_baseline", broken)
    assert violations, "zero_deg 被填非 null 却未被护栏判出"


def test_negative_blocked_table_marked_measured_is_rejected(tmp_path):
    """负例：为求 pass 把 blocked 表的 status 偷改成 measured → 段级必须 fail。

    改了 status 却没补齐正式值字段 → ``validate_all`` 报 error → 段级 fail
    （而不是被判成 pass）。
    """
    ws = _copy_workspace(tmp_path)
    data = _read_yaml(ws, "version_matrix")
    data["status"] = "measured"          # 偷改 status，正式字段仍是 null
    _write_yaml(ws, "version_matrix", data)
    report = bl.validate_all(ws)
    assert not report["ok"], "偷改 status 却判定通过"
    codes = {i.get("code") for i in report.get("issues") or [] if i.get("level") == "error"}
    assert "missing_measurement" in codes, sorted(codes)
    result = blp.run_baselines(repo_root=ws, ws=ws,
                               reports_dir=str(tmp_path / "reports"))
    assert result["verdict"] == "fail", result.get("reason")


def test_negative_segment_verdict_is_not_hardcoded(tmp_path):
    """负例面：段级 verdict 必须由 YAML status 推出（不得写死 blocked/pass）。

    把四张表在临时工作区里全部改成 measured → 期望 verdict 变成 pass；
    真实工作区里有三张 blocked → verdict 必须是 blocked。
    """
    ws = _copy_workspace(tmp_path)
    for kind in bl.KINDS:
        data = _read_yaml(ws, kind)
        data["status"] = "measured"
        _write_yaml(ws, kind, data)
    rows, _ = blp.collect_rows(ws)
    assert _expected_verdict([row.get("status") for row in rows]) == "pass"
    real_rows, _ = blp.collect_rows(REPO_ROOT)
    assert _expected_verdict([row.get("status") for row in real_rows]) == "blocked"
