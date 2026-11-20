"""S11-T4 真机 FK 复测证据工件的**离线校验**护栏（可执行，不连真机）。

工件：``workcell/baselines/evidence/rm75_s11_fk_recheck.json``
（schema ``rmctl.rm75_s11_fk_recheck/1``）。

为什么测：S11-T4 允许两种结案口径 ——

* ``ok``：真机窗口在**远端主机**跑通了门禁 + 只读快照 + 有界多样位形 FK 复测，
  工件里必须带**实测数值**（raw / corrected / residual_kind / 样本数）；
* ``blocked``：外部条件不具备（门禁不过、臂不可达、SDK 未物化……），
  此时工件里**不得出现任何实测数值主张**（最怕的是 blocked 里却写着
  "最大偏差 75.03 mm" —— 那就是拿旧数据或合成数据冒充真机结论）。

另外钉死两条纪律：

* ``residual_kind`` 决定**分支处置**：只有 ``tool_frame_offset`` 才允许写
  ``calibration.tcp``；``orientation_dependent`` / ``insufficient_diversity``
  一律不写偏置（否则会把「坐标系未判定」误记成「已标定」）。
* 工位 ``workcell/rm_workcell.yaml::calibration.tcp`` / ``base_to_base``
  与工件声明一致：工件说不写偏置 → yaml 里就必须是 null（**不得伪造**）。

全部只读：只读仓库内 git 跟踪的文件，不连真机、不发任何运动指令。
"""
from __future__ import annotations

import json
import os

import pytest
import yaml

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(TESTS_DIR, "..", "..", "..", ".."))

ARTIFACT_PATH = os.path.join(REPO_ROOT, "workcell", "baselines", "evidence",
                             "rm75_s11_fk_recheck.json")
WORKCELL_PATH = os.path.join(REPO_ROOT, "workcell", "rm_workcell.yaml")

SCHEMA = "rmctl.rm75_s11_fk_recheck/1"
STATUSES = ("ok", "blocked")

#: 与 rmctl.fk_cross.RESIDUAL_KINDS 对齐（避免 import 硬件层，保持纯离线）
RESIDUAL_KINDS = ("within_tolerance", "base_frame_offset", "tool_frame_offset",
                  "orientation_dependent", "insufficient_diversity")

#: 这些 residual_kind 一律**不允许**写标定偏置
NO_CALIBRATION_KINDS = ("orientation_dependent", "insufficient_diversity")


def _load_artifact() -> dict:
    assert os.path.isfile(ARTIFACT_PATH), "缺少证据工件：%s" % ARTIFACT_PATH
    with open(ARTIFACT_PATH, "r", encoding="utf-8") as handle:
        return json.load(handle)


def _load_workcell() -> dict:
    assert os.path.isfile(WORKCELL_PATH), "缺少工位定义：%s" % WORKCELL_PATH
    with open(WORKCELL_PATH, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def _validate(artifact: dict) -> None:
    """工件的**契约校验**（真实工件与合成样本共用同一条规则）。"""
    assert isinstance(artifact, dict), "工件必须是对象"
    assert artifact.get("schema") == SCHEMA, "schema 错：%r" % artifact.get("schema")
    status = artifact.get("status")
    assert status in STATUSES, "status 必须是 ok/blocked：%r" % status

    fk = artifact.get("fk_recheck") or {}
    raw = fk.get("raw") or {}
    corr = fk.get("corrected") or {}
    residual_kind = fk.get("residual_kind")

    if status == "blocked":
        # **blocked 不得含任何实测数值主张**
        assert not raw, "blocked 工件不得含 raw 实测数值：%r" % raw
        assert not corr, "blocked 工件不得含 corrected 实测数值：%r" % corr
        assert residual_kind is None, \
            "blocked 工件不得含 residual_kind 结论：%r" % residual_kind
        assert not fk.get("samples"), "blocked 工件不得声称采样数：%r" % fk.get("samples")
        assert not fk.get("dev_vectors_mm"), "blocked 工件不得含偏差向量"
        assert artifact.get("reason") or (artifact.get("branch_disposition") or {}).get("rationale"), \
            "blocked 工件必须写明原因"
        assert artifact.get("next_action"), "blocked 工件必须给出下一步命令"
        return

    # ok：实测数值必须齐全
    assert fk.get("samples", 0) >= 8, "ok 工件的样本数必须 ≥8：%r" % fk.get("samples")
    assert fk.get("distinct_configs", 0) >= 8, \
        "ok 工件的互异位形数必须 ≥8：%r" % fk.get("distinct_configs")
    for name, block in (("raw", raw), ("corrected", corr)):
        assert block, "ok 工件缺少 %s 指标" % name
        assert block.get("max_pos_dev_mm") is not None, "%s.max_pos_dev_mm 缺失" % name
        assert block.get("max_ori_dev_deg") is not None, "%s.max_ori_dev_deg 缺失" % name
    assert residual_kind in RESIDUAL_KINDS, "residual_kind 非法：%r" % residual_kind

    disposition = artifact.get("branch_disposition") or {}
    assert disposition, "ok 工件必须写明分支处置"
    if residual_kind in NO_CALIBRATION_KINDS:
        assert disposition.get("calibration_written") is False, \
            "residual_kind=%s 不得写任何偏置：%r" % (residual_kind, disposition)
        assert disposition.get("calibration_tcp_written") is False, \
            "residual_kind=%s 不得写 calibration.tcp" % residual_kind
        assert disposition.get("calibration_base_to_base_written") is False, \
            "residual_kind=%s 不得写 calibration.base_to_base" % residual_kind
    assert artifact.get("next_action"), "ok 工件也必须给出下一步命令"
    assert artifact.get("honesty"), "工件必须带诚实声明"


# ------------------------------------------------------------------ 真实工件
def test_real_artifact_satisfies_contract():
    """仓库里的真机复测工件必须满足契约。"""
    _validate(_load_artifact())


def test_real_artifact_records_gate_and_snapshot():
    """工件必须留下门禁逐项结果与只读快照摘要（可追溯）。"""
    artifact = _load_artifact()
    gate = artifact.get("gate") or {}
    assert gate, "工件缺少门禁结果"
    assert gate.get("status") in ("ok", "blocked"), "门禁 status 非法：%r" % gate.get("status")
    checks = gate.get("checks") or []
    assert checks, "门禁缺少逐项结果"
    names = {c.get("name") for c in checks}
    assert "ip_reachable" in names and "tcp_reachable" in names, names
    for check in checks:
        assert "ok" in check, "门禁项缺少 ok 布尔：%r" % check

    snapshot = artifact.get("snapshot") or {}
    assert snapshot, "工件缺少只读快照摘要"
    assert snapshot.get("joint_en_flag") == [1] * 7, \
        "快照必须记录 7 轴全部使能：%r" % snapshot.get("joint_en_flag")
    assert not any(int(v) != 0 for v in (snapshot.get("arm_err") or [])), \
        "快照必须记录臂级错误码为 0：%r" % snapshot.get("arm_err")
    assert not any(int(v) != 0 for v in (snapshot.get("joint_err_code") or [])), \
        "快照必须记录关节级错误码为 0：%r" % snapshot.get("joint_err_code")
    assert snapshot.get("controller_trajectory_type_name"), "快照缺少控制器状态机名"


def test_workcell_calibration_matches_artifact_disposition():
    """工件说不写偏置 → 工位 yaml 里就必须是 null（不得伪造）。"""
    artifact = _load_artifact()
    disposition = artifact.get("branch_disposition") or {}
    calibration = (_load_workcell().get("calibration") or {})
    if disposition.get("calibration_written") is False:
        assert calibration.get("tcp") is None, \
            "工件声明未写偏置，但 calibration.tcp 非 null：%r" % calibration.get("tcp")
        assert calibration.get("base_to_base") is None, \
            "工件声明未写偏置，但 calibration.base_to_base 非 null：%r" % calibration.get("base_to_base")


# ------------------------------------------------------------------ 负向用例
def _synthetic(status: str, **overrides) -> dict:
    payload = {
        "schema": SCHEMA,
        "status": status,
        "fk_recheck": {
            "samples": 8,
            "distinct_configs": 8,
            "raw": {"max_pos_dev_mm": 1.0, "max_ori_dev_deg": 0.01},
            "corrected": {"max_pos_dev_mm": 1.0, "max_ori_dev_deg": 0.01},
            "residual_kind": "within_tolerance",
        },
        "branch_disposition": {"calibration_written": False,
                               "calibration_tcp_written": False,
                               "calibration_base_to_base_written": False},
        "next_action": ["示例命令"],
        "honesty": "示例",
    }
    if status == "blocked":
        payload["fk_recheck"] = {}
        payload["reason"] = "示例原因"
    for key, value in overrides.items():
        payload[key] = value
    return payload


def test_blocked_payload_without_measurements_is_accepted():
    """负向基线：blocked 且不含任何实测数值 → 合法。"""
    _validate(_synthetic("blocked"))


def test_blocked_payload_with_measured_numbers_is_rejected():
    """负向：blocked 却带 raw 数值 → 必须判失败（不得冒充真机结论）。"""
    payload = _synthetic("blocked")
    payload["fk_recheck"]["raw"] = {"max_pos_dev_mm": 75.03, "max_ori_dev_deg": 0.4}
    with pytest.raises(AssertionError):
        _validate(payload)


def test_blocked_payload_with_residual_kind_is_rejected():
    """负向：blocked 却带 residual_kind 结论 → 必须判失败。"""
    payload = _synthetic("blocked")
    payload["fk_recheck"]["residual_kind"] = "tool_frame_offset"
    with pytest.raises(AssertionError):
        _validate(payload)


def test_blocked_payload_without_next_action_is_rejected():
    """负向：blocked 但没有下一步命令 → 必须判失败。"""
    payload = _synthetic("blocked")
    payload["next_action"] = []
    with pytest.raises(AssertionError):
        _validate(payload)


def test_ok_payload_with_too_few_samples_is_rejected():
    """负向：ok 但样本数 <8 → 必须判失败（采样量不足不得声称复测完成）。"""
    payload = _synthetic("ok")
    payload["fk_recheck"]["samples"] = 4
    payload["fk_recheck"]["distinct_configs"] = 4
    with pytest.raises(AssertionError):
        _validate(payload)


def test_ok_payload_missing_corrected_metrics_is_rejected():
    """负向：ok 但缺 corrected 指标 → 必须判失败（raw/corrected 必须成对）。"""
    payload = _synthetic("ok")
    payload["fk_recheck"]["corrected"] = {}
    with pytest.raises(AssertionError):
        _validate(payload)


def test_ok_payload_with_illegal_residual_kind_is_rejected():
    """负向：residual_kind 不在词表内 → 必须判失败。"""
    payload = _synthetic("ok")
    payload["fk_recheck"]["residual_kind"] = "看起来差不多"
    with pytest.raises(AssertionError):
        _validate(payload)


@pytest.mark.parametrize("kind", list(NO_CALIBRATION_KINDS))
def test_no_calibration_kinds_forbid_writing_offsets(kind):
    """负向：orientation_dependent / insufficient_diversity 不得声称写了偏置。"""
    payload = _synthetic("ok")
    payload["fk_recheck"]["residual_kind"] = kind
    payload["branch_disposition"] = {"calibration_written": True,
                                    "calibration_tcp_written": True,
                                    "calibration_base_to_base_written": False}
    with pytest.raises(AssertionError):
        _validate(payload)


def test_unknown_schema_is_rejected():
    """负向：schema 不对 → 必须判失败（避免把别的工件当成本轮证据）。"""
    payload = _synthetic("ok")
    payload["schema"] = "rmctl.rm75_s10_window_outcome/1"
    with pytest.raises(AssertionError):
        _validate(payload)


def test_unknown_status_is_rejected():
    """负向：status 非法（例如 failed）→ 必须判失败。"""
    payload = _synthetic("ok")
    payload["status"] = "failed"
    with pytest.raises(AssertionError):
        _validate(payload)
