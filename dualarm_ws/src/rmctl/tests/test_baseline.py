"""真机基线 schema 校验器的契约测试。

覆盖：
  - 四份基线文件存在且通过校验（pending 态合法）；
  - 缺必填字段 / 单位标注不符 / status 非法 / 版本倒退 都有明确报错文本；
  - status=pending 不阻塞校验；未实测时填数值（占位数值）必须报错；
  - status=measured 时 measured_limits 与 joint_limits.yaml 口径一致
    （不得高于已生效限值），样本数不足 / 缺来源必须拒绝；
  - 零位基线的 FK 复测阈值（< 1 mm）与"控制器位姿仅用于交叉校验"标注（R003）。
"""
import copy
import json
import os

import pytest
import yaml

from rmctl import baseline as bl
from rmctl import paths

KINDS = bl.KINDS


# ------------------------------------------------------------------ 仓库内基线
def test_four_baseline_files_exist(repo_root):
    bdir = paths.baselines_dir(repo_root)
    for kind in KINDS:
        assert os.path.exists(os.path.join(bdir, "%s.yaml" % kind)), kind


def test_repo_baselines_pass_validation(repo_root):
    report = bl.validate_all(ws=repo_root)
    errs = [i for i in report["issues"] if i["level"] == "error"]
    assert errs == [], bl.format_issues(errs)
    assert report["ok"], report
    assert not report["missing"]
    for kind in KINDS:
        assert report["kinds"][kind]["present"]
        assert report["kinds"][kind]["status"] in bl.STATUSES


def test_pending_state_is_informational_not_blocking(tmp_workspace):
    """pending 态合法：不阻塞（error_count=0），但必须显式提示未实测。"""
    for name in sorted(os.listdir(os.path.join(tmp_workspace, "workcell", "baselines"))):
        if not name.endswith(".yaml"):
            continue
        path = os.path.join(tmp_workspace, "workcell", "baselines", name)
        with open(path, "r", encoding="utf-8") as fh:
            data = yaml.safe_load(fh)
        data["status"] = "pending"
        data.pop("blocked", None)
        with open(path, "w", encoding="utf-8") as fh:
            yaml.safe_dump(data, fh, allow_unicode=True, sort_keys=False)
    report = bl.validate_all(ws=tmp_workspace)
    infos = [i for i in report["issues"] if i["code"] == "pending"]
    assert len(infos) == len(KINDS), infos
    assert all(i["level"] == "info" for i in infos)
    assert report["error_count"] == 0, bl.format_issues(report["issues"])


def test_repo_baselines_are_unmeasured_honest(repo_root):
    """未实测项必须是 null（禁止占位数值）。"""
    for kind in KINDS:
        data = bl.load_baseline(kind, repo_root)
        assert data["status"] in ("pending", "blocked"), kind
        if kind == "measured_limits":
            for j in data["joints"]:
                assert j["acceleration"]["value"] is None
                assert j["jerk"]["value"] is None
            assert data["tcp_repeatability"]["value_mm"] is None
        if kind == "zero_baseline":
            for j in data["joints"]:
                assert j["zero_deg"] is None


def test_repo_baselines_are_either_measured_or_blocked(repo_root):
    """S01-T7 的 Done-when：不允许出现 pending 却宣称已完成的中间态。

    四份基线必须是 measured（附证据）或 blocked（附阻塞原因）二者之一。
    """
    for kind in KINDS:
        data = bl.load_baseline(kind, repo_root)
        assert data["status"] in ("measured", "blocked"), \
            "%s status=%r 属于中间态" % (kind, data["status"])
        if data["status"] == "blocked":
            assert data.get("blocked"), kind
            assert data["blocked"].get("reason"), kind
            assert data["blocked"].get("attempted_actions"), kind


def test_blocked_baselines_carry_info_issue(repo_root):
    """blocked 态必须带 info 级说明（不阻塞，但要看得见）。"""
    report = bl.validate_all(ws=repo_root)
    infos = [i for i in report["issues"] if i["code"] == "blocked"]
    assert infos, "blocked 基线应带 info 级提示"
    assert all(i["level"] == "info" for i in infos)
    assert report["error_count"] == 0, bl.format_issues(report["issues"])


def test_baselines_dir_matches_workcell_layout(repo_root):
    assert paths.baselines_dir(repo_root) == os.path.join(repo_root, "workcell", "baselines")
    assert bl.baselines_dir(repo_root) == paths.baselines_dir(repo_root)


# ------------------------------------------------------------------ 通用校验
def _minimal(kind):
    """构造一份合法但 pending 的基线数据（用于逐条注入错误）。"""
    if kind == "zero_baseline":
        return {
            "kind": "zero_baseline", "schema_version": 1, "version": 1, "status": "pending",
            "captured_at": None, "arm": "left", "firmware": None,
            "teachpendant_version": None,
            "joints": [{"name": "joint%d" % i, "zero_deg": None, "unit": "deg",
                        "groove_photo": None} for i in range(1, 8)],
            "groove_photos": [],
            "fk_recheck": {"method": None, "samples": None, "max_pos_dev_mm": None,
                           "max_ori_dev_deg": None, "conclusion": None,
                           "controller_pose_usage": bl.CONTROLLER_POSE_USAGE},
            "recalibration_triggers": ["换电池"],
        }
    if kind == "measured_limits":
        return {
            "kind": "measured_limits", "schema_version": 1, "version": 1, "status": "pending",
            "captured_at": None, "arm": "left", "udp_period_ms": 5,
            "joints": [{"name": "joint%d" % i,
                        "acceleration": {"value": None, "unit": "rad/s^2", "source": None,
                                         "method": None, "samples": None},
                        "jerk": {"value": None, "unit": "rad/s^3", "source": None,
                                 "method": None, "samples": None}} for i in range(1, 8)],
            "tcp_repeatability": {"value_mm": None, "unit": "mm", "source": None,
                                   "method": None, "samples": None},
        }
    if kind == "version_matrix":
        return {
            "kind": "version_matrix", "schema_version": 1, "version": 1, "status": "pending",
            "captured_at": None, "controller_firmware": None, "teachpendant": None,
            "rm_api2_sdk": None, "ros2_rm_robot_tag": None,
            "tesseract_robotics_nanobind": "0.35.0.8", "libstdcxx": None, "python": None,
            "os": None, "container_image": None,
        }
    return {
        "kind": "network_plan", "schema_version": 1, "version": 1, "status": "pending",
        "captured_at": None,
        "arms": {"left": {"ip": "192.168.1.18", "tcp_port": 8080, "udp_port": 8091},
                 "right": {"ip": "192.168.1.19", "tcp_port": 8080, "udp_port": 8092}},
        "realtime_report": {"period_ms": 5, "cycle_multiple_ms": 5, "mode": "triple_thread"},
        "constraints": {"single_client_per_endpoint": True, "triple_thread_mode": True,
                        "wired_direct": True},
    }


@pytest.mark.parametrize("kind", KINDS)
def test_minimal_valid_baseline_has_no_error(kind):
    issues = bl.validate_baseline(kind, _minimal(kind))
    errs = [i for i in issues if i["level"] == "error"]
    assert errs == [], bl.format_issues(errs)


@pytest.mark.parametrize("kind", KINDS)
def test_missing_required_field_is_reported(kind):
    data = _minimal(kind)
    data.pop("status")
    issues = bl.validate_baseline(kind, data)
    codes = [i["code"] for i in issues if i["level"] == "error"]
    assert "status_invalid" in codes
    assert any("status" in i["message"] for i in issues)


@pytest.mark.parametrize("kind", KINDS)
def test_invalid_status_is_reported(kind):
    data = _minimal(kind)
    data["status"] = "done"
    codes = [i["code"] for i in bl.validate_baseline(kind, data) if i["level"] == "error"]
    assert "status_invalid" in codes


@pytest.mark.parametrize("kind", KINDS)
def test_invalid_version_is_reported(kind):
    data = _minimal(kind)
    data["version"] = 0
    codes = [i["code"] for i in bl.validate_baseline(kind, data) if i["level"] == "error"]
    assert "version_invalid" in codes


@pytest.mark.parametrize("kind", KINDS)
def test_kind_mismatch_is_reported(kind):
    data = _minimal(kind)
    data["kind"] = "other"
    codes = [i["code"] for i in bl.validate_baseline(kind, data) if i["level"] == "error"]
    assert "kind_mismatch" in codes


def test_unknown_kind_is_reported():
    codes = [i["code"] for i in bl.validate_baseline("nope", {}) if i["level"] == "error"]
    assert "unknown_kind" in codes


def test_non_mapping_is_reported():
    codes = [i["code"] for i in bl.validate_baseline("version_matrix", []) if i["level"] == "error"]
    assert "not_mapping" in codes


def test_version_must_be_monotonic():
    prev = {"version": 3}
    cur = {"version": 2}
    codes = [i["code"] for i in bl.check_version_monotonic(prev, cur, "zero_baseline")
             if i["level"] == "error"]
    assert codes == ["version_not_monotonic"]
    ok = bl.check_version_monotonic({"version": 3}, {"version": 4}, "zero_baseline")
    assert ok == []


def test_validate_baseline_flags_version_regression():
    data = _minimal("version_matrix")
    data["version"] = 1
    codes = [i["code"] for i in bl.validate_baseline("version_matrix", data,
                                                     previous={"version": 2})
             if i["level"] == "error"]
    assert "version_not_monotonic" in codes


# ------------------------------------------------------------------ 单位标注
def test_measured_limits_unit_mismatch_is_reported():
    data = _minimal("measured_limits")
    data["joints"][0]["acceleration"]["unit"] = "deg/s^2"
    codes = [i["code"] for i in bl.validate_baseline("measured_limits", data)
             if i["level"] == "error"]
    assert "unit_mismatch" in codes


def test_measured_limits_jerk_unit_mismatch_is_reported():
    data = _minimal("measured_limits")
    data["joints"][3]["jerk"]["unit"] = "rad/s^2"
    codes = [i["code"] for i in bl.validate_baseline("measured_limits", data)
             if i["level"] == "error"]
    assert "unit_mismatch" in codes


def test_tcp_repeatability_unit_mismatch_is_reported():
    data = _minimal("measured_limits")
    data["tcp_repeatability"]["unit"] = "m"
    codes = [i["code"] for i in bl.validate_baseline("measured_limits", data)
             if i["level"] == "error"]
    assert "unit_mismatch" in codes


def test_zero_baseline_unit_mismatch_is_reported():
    data = _minimal("zero_baseline")
    data["joints"][0]["unit"] = "rad"
    codes = [i["code"] for i in bl.validate_baseline("zero_baseline", data)
             if i["level"] == "error"]
    assert "unit_mismatch" in codes


# ------------------------------------------------------------------ 占位数值
def test_pending_with_placeholder_value_is_rejected():
    """status=pending 却填了数值 → 视为占位数值，必须报错。"""
    data = _minimal("measured_limits")
    data["joints"][0]["acceleration"]["value"] = 5.2359
    codes = [i["code"] for i in bl.validate_baseline("measured_limits", data)
             if i["level"] == "error"]
    assert "unmeasured_value_present" in codes


def test_pending_with_placeholder_tcp_value_is_rejected():
    data = _minimal("measured_limits")
    data["tcp_repeatability"]["value_mm"] = 0.05
    codes = [i["code"] for i in bl.validate_baseline("measured_limits", data)
             if i["level"] == "error"]
    assert "unmeasured_value_present" in codes


def test_blocked_status_also_forbids_placeholder_values():
    data = _minimal("measured_limits")
    data["status"] = "blocked"
    data["joints"][1]["jerk"]["value"] = 30.0
    codes = [i["code"] for i in bl.validate_baseline("measured_limits", data)
             if i["level"] == "error"]
    assert "unmeasured_value_present" in codes


# ------------------------------------------------------------------ measured 口径
def _measured_limits_data(acc=5.2359, jerk=30.0, samples=5, source="ladder_acc_jerk",
                          method="阶梯速度反推"):
    data = _minimal("measured_limits")
    data["status"] = "measured"
    data["captured_at"] = "2026-11-13T10:00:00+08:00"
    for j in data["joints"]:
        j["acceleration"].update({"value": acc, "source": source, "method": method,
                                  "samples": samples})
        j["jerk"].update({"value": jerk, "source": source, "method": method,
                          "samples": samples})
    data["tcp_repeatability"].update({"value_mm": 0.04, "source": source, "method": method,
                                      "samples": 30})
    return data


def test_measured_limits_equal_to_joint_limits_is_accepted(tmp_workspace):
    limits = bl.load_joint_limits(tmp_workspace)
    assert limits and "joint1" in limits
    data = _measured_limits_data(acc=limits["joint1"]["acceleration"],
                                 jerk=limits["joint1"]["jerk"])
    codes = [i["code"] for i in bl.validate_baseline("measured_limits", data, ws=tmp_workspace)
             if i["level"] == "error"]
    assert codes == [], codes


def test_measured_limits_more_conservative_is_accepted(tmp_workspace):
    limits = bl.load_joint_limits(tmp_workspace)
    data = _measured_limits_data(acc=limits["joint1"]["acceleration"] * 0.5,
                                 jerk=limits["joint1"]["jerk"] * 0.5)
    codes = [i["code"] for i in bl.validate_baseline("measured_limits", data, ws=tmp_workspace)
             if i["level"] == "error"]
    assert codes == [], codes


def test_measured_limits_above_joint_limits_is_rejected(tmp_workspace):
    limits = bl.load_joint_limits(tmp_workspace)
    data = _measured_limits_data(acc=limits["joint1"]["acceleration"] * 1.01,
                                 jerk=limits["joint1"]["jerk"])
    codes = [i["code"] for i in bl.validate_baseline("measured_limits", data, ws=tmp_workspace)
             if i["level"] == "error"]
    assert "limit_above_joint_limits" in codes


def test_measured_limits_insufficient_samples_is_rejected(tmp_workspace):
    data = _measured_limits_data(samples=1)
    codes = [i["code"] for i in bl.validate_baseline("measured_limits", data, ws=tmp_workspace)
             if i["level"] == "error"]
    assert "insufficient_samples" in codes


def test_measured_limits_missing_source_is_rejected(tmp_workspace):
    data = _measured_limits_data()
    data["joints"][2]["acceleration"]["source"] = None
    codes = [i["code"] for i in bl.validate_baseline("measured_limits", data, ws=tmp_workspace)
             if i["level"] == "error"]
    assert "missing_source" in codes


def test_measured_limits_requires_captured_at(tmp_workspace):
    data = _measured_limits_data()
    data["captured_at"] = None
    codes = [i["code"] for i in bl.validate_baseline("measured_limits", data, ws=tmp_workspace)
             if i["level"] == "error"]
    assert "captured_at_required" in codes


def test_measured_limits_without_joint_limits_is_rejected(tmp_path):
    """读不到 joint_limits.yaml → 保守性无法校验，必须报错而不是静默跳过。"""
    data = _measured_limits_data()
    codes = [i["code"] for i in bl.validate_baseline("measured_limits", data, ws=str(tmp_path))
             if i["level"] == "error"]
    assert "joint_limits_unavailable" in codes


def test_measured_limits_joint_absent_from_joint_limits_is_rejected(tmp_workspace):
    path = os.path.join(tmp_workspace, "dualarm_ws", "src", "rm_description", "config",
                        "joint_limits.yaml")
    with open(path, "r", encoding="utf-8") as fh:
        raw = yaml.safe_load(fh)
    raw["rm75_arm"].pop("joint4")
    with open(path, "w", encoding="utf-8") as fh:
        yaml.safe_dump(raw, fh, allow_unicode=True)
    data = _measured_limits_data()
    codes = [i["code"] for i in bl.validate_baseline("measured_limits", data, ws=tmp_workspace)
             if i["level"] == "error"]
    assert "joint_limits_missing_joint" in codes


def test_udp_period_must_be_multiple_of_5():
    data = _minimal("measured_limits")
    data["udp_period_ms"] = 7
    codes = [i["code"] for i in bl.validate_baseline("measured_limits", data)
             if i["level"] == "error"]
    assert "udp_period_not_multiple" in codes


def test_missing_joint_entry_is_reported():
    data = _minimal("measured_limits")
    data["joints"] = data["joints"][:6]
    codes = [i["code"] for i in bl.validate_baseline("measured_limits", data)
             if i["level"] == "error"]
    assert "missing_joints" in codes


# ------------------------------------------------------------------ 零位基线
def _measured_zero(dev_mm=0.4):
    data = _minimal("zero_baseline")
    data["status"] = "measured"
    data["captured_at"] = "2026-11-13T10:00:00+08:00"
    data["firmware"] = "V1.0"
    data["teachpendant_version"] = "1.2.3"
    data["groove_photos"] = ["docs/code_doc/photos/joint1_groove.jpg"]
    for j in data["joints"]:
        j["zero_deg"] = 0.0
        j["groove_photo"] = "docs/code_doc/photos/x.jpg"
    data["fk_recheck"]["max_pos_dev_mm"] = dev_mm
    data["fk_recheck"]["max_ori_dev_deg"] = 0.01
    data["fk_recheck"]["samples"] = 20
    data["fk_recheck"]["method"] = "rmctl.fk_cross"
    data["fk_recheck"]["conclusion"] = "通过"
    return data


def test_zero_baseline_measured_passes():
    codes = [i["code"] for i in bl.validate_baseline("zero_baseline", _measured_zero())
             if i["level"] == "error"]
    assert codes == [], codes


def test_zero_baseline_fk_deviation_over_1mm_is_rejected():
    codes = [i["code"] for i in bl.validate_baseline("zero_baseline", _measured_zero(1.5))
             if i["level"] == "error"]
    assert "fk_deviation_exceeded" in codes


def test_zero_baseline_requires_controller_pose_usage_annotation():
    data = _measured_zero()
    data["fk_recheck"]["controller_pose_usage"] = "控制器位姿"
    codes = [i["code"] for i in bl.validate_baseline("zero_baseline", data)
             if i["level"] == "error"]
    assert "controller_pose_usage_missing" in codes


def test_zero_baseline_requires_groove_photo_evidence():
    data = _measured_zero()
    data["groove_photos"] = []
    codes = [i["code"] for i in bl.validate_baseline("zero_baseline", data)
             if i["level"] == "error"]
    assert "missing_evidence" in codes


# ------------------------------------------------------------------ 网络规划
def test_network_plan_duplicate_udp_port_is_rejected():
    data = _minimal("network_plan")
    data["arms"]["right"]["udp_port"] = 8091
    codes = [i["code"] for i in bl.validate_baseline("network_plan", data)
             if i["level"] == "error"]
    assert "duplicate_udp_port" in codes


def test_network_plan_invalid_ip_is_rejected():
    data = _minimal("network_plan")
    data["arms"]["left"]["ip"] = "192.168.1.999"
    codes = [i["code"] for i in bl.validate_baseline("network_plan", data)
             if i["level"] == "error"]
    assert "invalid_ip" in codes


def test_network_plan_requires_triple_thread_mode():
    data = _minimal("network_plan")
    data["realtime_report"]["mode"] = "double_thread"
    data["constraints"]["triple_thread_mode"] = False
    codes = [i["code"] for i in bl.validate_baseline("network_plan", data)
             if i["level"] == "error"]
    assert "invalid_report_mode" in codes
    assert "constraint_violated" in codes


def test_network_plan_period_must_be_cycle_multiple():
    data = _minimal("network_plan")
    data["realtime_report"]["period_ms"] = 7
    codes = [i["code"] for i in bl.validate_baseline("network_plan", data)
             if i["level"] == "error"]
    assert "udp_period_not_multiple" in codes


def test_network_plan_repo_values_match_design():
    data = bl.load_baseline("network_plan", paths.workspace_root())
    assert data["arms"]["left"]["ip"] == "192.168.1.18"
    assert data["arms"]["right"]["ip"] == "192.168.1.19"
    assert data["arms"]["left"]["tcp_port"] == 8080
    assert data["arms"]["left"]["udp_port"] == 8091
    assert data["arms"]["right"]["udp_port"] == 8092


# ------------------------------------------------------------------ 版本矩阵
def test_version_matrix_measured_requires_all_entries():
    data = _minimal("version_matrix")
    data["status"] = "measured"
    data["captured_at"] = "2026-11-13T10:00:00+08:00"
    codes = [i["code"] for i in bl.validate_baseline("version_matrix", data)
             if i["level"] == "error"]
    assert "missing_measurement" in codes


def test_version_matrix_repo_entry_is_pinned():
    data = bl.load_baseline("version_matrix", paths.workspace_root())
    assert data["tesseract_robotics_nanobind"] == "0.35.0.8"
    # RM-API2 版本由 rm_api_version() 实测得来（非编造）；控制器固件/示教器仍 null
    assert data["rm_api2_sdk"] == "1.1.7"
    assert data["controller_firmware"] is None
    assert data["teachpendant"] is None
    assert data["container_image"] is None


# ------------------------------------------------------------------ blocked 语义
def test_blocked_requires_reason_and_attempted_actions():
    """blocked 态必须自证：缺 reason 或 attempted_actions 都要报错。"""
    data = _minimal("measured_limits")
    data["status"] = "blocked"
    codes = [i["code"] for i in bl.validate_baseline("measured_limits", data)
             if i["level"] == "error"]
    assert "blocked_missing" in codes

    data["blocked"] = {"reason": None, "attempted_actions": []}
    codes = [i["code"] for i in bl.validate_baseline("measured_limits", data)
             if i["level"] == "error"]
    assert "blocked_reason_missing" in codes
    assert "blocked_actions_missing" in codes


def test_blocked_with_reason_and_actions_passes():
    data = _minimal("measured_limits")
    data["status"] = "blocked"
    data["blocked"] = {"reason": "臂不可达", "attempted_actions": ["ping 失败"]}
    codes = [i["code"] for i in bl.validate_baseline("measured_limits", data)
             if i["level"] == "error"]
    assert codes == [], codes


def test_blocked_status_emits_info_issue():
    data = _minimal("measured_limits")
    data["status"] = "blocked"
    data["blocked"] = {"reason": "臂不可达", "attempted_actions": ["ping 失败"]}
    issues = bl.validate_baseline("measured_limits", data)
    infos = [i for i in issues if i["code"] == "blocked"]
    assert len(infos) == 1
    assert infos[0]["level"] == "info"
    assert "臂不可达" in infos[0]["message"]


def test_repo_blocked_baselines_have_no_fabricated_values(repo_root):
    """blocked 基线不得夹带任何实测数值（禁止用占位值掩盖阻塞）。

    **S11 例外（只针对 zero_baseline.fk_recheck）**：FK 交叉校验复测本身就是
    零位基线要求归档的内容之一，S11-T4 已在远端真机窗口完成实测。此时
    blocked 表里允许携带 FK 残差数值，但必须**逐字等于**真机复测工件
    （workcell/baselines/evidence/rm75_s11_fk_recheck.json）里的实测值 ——
    没有工件背书的数值仍按「占位数值」论处（必须为 null）。
    """
    evidence = _s11_fk_evidence(repo_root)
    for kind in KINDS:
        data = bl.load_baseline(kind, repo_root)
        if data["status"] != "blocked":
            continue
        assert data["captured_at"] is None, kind
        if kind == "measured_limits":
            for j in data["joints"]:
                assert j["acceleration"]["value"] is None
                assert j["jerk"]["value"] is None
                assert j["acceleration"]["source"] is None
                assert j["jerk"]["source"] is None
            assert data["tcp_repeatability"]["value_mm"] is None
        if kind == "zero_baseline":
            for j in data["joints"]:
                assert j["zero_deg"] is None
            fk = data["fk_recheck"]
            if evidence is None:
                assert fk["max_pos_dev_mm"] is None
                assert fk["max_ori_dev_deg"] is None
            else:
                raw = evidence["fk_recheck"]["raw"]
                assert fk["max_pos_dev_mm"] == raw["max_pos_dev_mm"]
                assert fk["max_ori_dev_deg"] == raw["max_ori_dev_deg"]


def _s11_fk_evidence(repo_root):
    """S11-T4 真机 FK 复测工件（无工件 / 非 ok → 返回 None）。"""
    path = os.path.join(repo_root, "workcell", "baselines", "evidence",
                        "rm75_s11_fk_recheck.json")
    if not os.path.isfile(path):
        return None
    with open(path, "r", encoding="utf-8") as fh:
        payload = json.load(fh)
    if payload.get("status") != "ok":
        return None
    return payload


# ------------------------------------------------------------------ 汇总报告
def test_validate_all_reports_missing_file(tmp_workspace):
    os.remove(os.path.join(tmp_workspace, "workcell", "baselines", "zero_baseline.yaml"))
    report = bl.validate_all(ws=tmp_workspace)
    assert "zero_baseline" in report["missing"]
    assert not report["ok"]
    codes = [i["code"] for i in report["issues"]]
    assert "file_missing" in codes
    assert any("baseline scaffold" in i["message"] for i in report["issues"])


def test_validate_all_reports_broken_yaml(tmp_workspace):
    path = os.path.join(tmp_workspace, "workcell", "baselines", "network_plan.yaml")
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("kind: network_plan\n  bad indent: [\n")
    report = bl.validate_all(ws=tmp_workspace)
    assert not report["ok"]
    assert report["kinds"]["network_plan"]["error_count"] >= 1


def test_load_baseline_missing_file_raises_with_fix_hint(tmp_workspace):
    os.remove(os.path.join(tmp_workspace, "workcell", "baselines", "version_matrix.yaml"))
    with pytest.raises(bl.BaselineError) as excinfo:
        bl.load_baseline("version_matrix", tmp_workspace)
    assert "baseline scaffold" in str(excinfo.value)


def test_load_baseline_unknown_kind_raises():
    with pytest.raises(bl.BaselineError):
        bl.load_baseline("nope", paths.workspace_root())


def test_baseline_files_are_yaml_serialisable(repo_root):
    """基线文件必须能原样 round-trip（可 diff、可评审）。"""
    for kind in KINDS:
        path = os.path.join(paths.baselines_dir(repo_root), "%s.yaml" % kind)
        with open(path, "r", encoding="utf-8") as fh:
            data = yaml.safe_load(fh)
        again = yaml.safe_load(yaml.safe_dump(data, allow_unicode=True))
        assert again == data
        assert copy.deepcopy(data) == data
