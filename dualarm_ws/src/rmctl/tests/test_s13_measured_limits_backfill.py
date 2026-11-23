"""S13-T4 守卫：measured_limits 全表 measured 回填 + conservative 回填真正生效。

钉死的口径（任一条被破坏即失败）：

1. ``workcell/baselines/measured_limits.yaml`` 的 status == ``measured``；
2. 7 个关节 × {acceleration, jerk} 的 value 为正数、source/method 非空、samples ≥ 3、
   单位正确（rad/s^2 / rad/s^3）；
3. 逐关节 value **逐字等于** ``min(工件实测包络, 回填前已生效限值)``——
   joint1 取自 S01 工件 V8 阶段，joint2..joint7 取自 S13 工件
   ``rm75_s13_acc_jerk.json``（**不是**手抄的转录值）；
4. joint1 的 jerk 因实测包络高于已生效限值 → value 必须是 30.0（不得写实测值），
   且 method 必须写明"实测包络高于已生效限值 → 保持现值"的原因；
5. ``joint_limits.yaml`` 逐关节 acceleration/jerk **不大于** 回填前现值
   （5.2359 / 30.0）——只收紧不放宽；
6. ``tcp_repeatability.value_mm`` 与 S01 工件 V7 阶段 ``max_deviation_mm`` 逐字相等，
   且此前**无工件背书**的转录值已被作废（不得与正式值并存）；
7. ``python3 -m rmctl baseline check`` 退出码 0。

负例（Q7）：
* 基线 value 高于 joint_limits 现值 → ``rmctl.baseline`` 必须判 error
  （``limit_above_joint_limits``）；
* joint_limits 被放宽（> 回填前现值）→ 守卫判失败；
* 回填工具写盘时把实测值截断（精度丢失）→ 守卫判失败。

只用 git 跟踪的仓库文件 + pytest 临时目录（conftest.tmp_workspace）。
"""
import importlib.util
import json
import os
import subprocess
import sys

import pytest
import yaml

from rmctl import baseline as bl

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(TESTS_DIR, "..", "..", "..", ".."))

BASELINE_REL = os.path.join("workcell", "baselines", "measured_limits.yaml")
LIMITS_REL = os.path.join("dualarm_ws", "src", "rm_description", "config",
                          "joint_limits.yaml")
S01_EVIDENCE_REL = os.path.join("workcell", "baselines", "evidence",
                                "rm75_validation_ladder_20261119.json")
S13_EVIDENCE_REL = os.path.join("workcell", "baselines", "evidence",
                                "rm75_s13_acc_jerk.json")

JOINTS = tuple("joint%d" % i for i in range(1, 8))

#: 回填**前** joint_limits.yaml 的已生效限值（保守占位）
OLD_ACCELERATION = 5.2359
OLD_JERK = 30.0

#: 已被作废的转录值（S01 工件里找不到背书）——不得再出现在基线文件里
VOIDED_TCP_VALUE = 0.0340506211177


# ------------------------------------------------------------------ 夹具
def _path(rel):
    return os.path.join(REPO_ROOT, rel)


def _load_yaml(rel):
    with open(_path(rel), "r", encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def _load_json(rel):
    with open(_path(rel), "r", encoding="utf-8") as fh:
        return json.load(fh)


def _baseline():
    return _load_yaml(BASELINE_REL)


def _limits():
    return (_load_yaml(LIMITS_REL) or {}).get("rm75_arm") or {}


def _baseline_text():
    with open(_path(BASELINE_REL), "r", encoding="utf-8") as fh:
        return fh.read()


def _s01_stage(stage_id):
    data = _load_json(S01_EVIDENCE_REL)
    for stage in data.get("stages") or []:
        if stage.get("id") == stage_id:
            return stage
    raise AssertionError("S01 工件里没有阶段 %s" % stage_id)


def _s01_joint1_envelope():
    """S01 工件 V8 阶段 joint1 的 acc/jerk 包络（逐字）。"""
    data = _s01_stage("V8")["data"]
    return (data["acceleration_rad_s2"]["joint1"],
            data["jerk_rad_s3"]["joint1"])


def _s13_envelope(joint):
    """S13 工件里某关节的 acc/jerk 包络（逐字）。"""
    data = _load_json(S13_EVIDENCE_REL)
    assert data.get("status") == "ok", "S13 工件不是 ok（blocked 工件不得作为数值来源）"
    entry = (data.get("joints") or {})[joint]
    return entry["acceleration_rad_s2"], entry["jerk_rad_s3"]


def _envelope(joint):
    """该关节的真机实测包络（acc, jerk）——按来源工件取值。"""
    if joint == "joint1":
        return _s01_joint1_envelope()
    return _s13_envelope(joint)


def _expected_value(joint, field):
    """保守口径的期望值：min(实测包络, 回填前已生效限值)。"""
    acc, jerk = _envelope(joint)
    envelope = acc if field == "acceleration" else jerk
    cap = OLD_ACCELERATION if field == "acceleration" else OLD_JERK
    return min(envelope, cap)


def _joint_entry(baseline, joint):
    for entry in baseline.get("joints") or []:
        if entry.get("name") == joint:
            return entry
    raise AssertionError("基线里缺少关节 %s" % joint)


def _numeric_leaves(node):
    """递归收集映射/列表里的数值叶子。"""
    out = []
    if isinstance(node, dict):
        for value in node.values():
            out.extend(_numeric_leaves(value))
    elif isinstance(node, list):
        for value in node:
            out.extend(_numeric_leaves(value))
    elif isinstance(node, (int, float)) and not isinstance(node, bool):
        out.append(float(node))
    return out


# ------------------------------------------------------------------ 正例
def test_baseline_status_is_measured():
    """基线 status 必须是 measured（首次诚实升位）。"""
    baseline = _baseline()
    assert baseline.get("status") == "measured", baseline.get("status")
    assert baseline.get("version") == 6, baseline.get("version")
    assert baseline.get("captured_at"), "status=measured 时 captured_at 必填"


def test_blocked_section_renamed_to_blocked_history():
    """旧的 blocked 段改名为 blocked_history，且 reason 标注「历史：S13 已闭合」。"""
    baseline = _baseline()
    assert "blocked" not in baseline, "measured 的表不应再挂生效的 blocked 段"
    history = baseline.get("blocked_history")
    assert isinstance(history, dict), "缺 blocked_history（历史可追溯性）"
    reason = str(history.get("reason") or "")
    assert reason.startswith("历史：S13 已闭合"), reason[:60]
    assert history.get("attempted_actions"), "blocked_history 必须保留已尝试动作"
    assert str(history.get("next_action") or "").strip(), "blocked_history 缺 next_action"


@pytest.mark.parametrize("joint", list(JOINTS))
@pytest.mark.parametrize("field,unit", [("acceleration", "rad/s^2"),
                                         ("jerk", "rad/s^3")])
def test_joint_value_has_evidence_discipline(joint, field, unit):
    """每条实测值：正数 + source/method 非空 + samples ≥ 3 + 单位正确。"""
    entry = _joint_entry(_baseline(), joint)
    item = entry.get(field)
    assert isinstance(item, dict), "%s.%s 必须是映射" % (joint, field)
    assert item.get("unit") == unit, "%s.%s 单位应为 %s" % (joint, field, unit)
    value = item.get("value")
    assert isinstance(value, (int, float)) and not isinstance(value, bool), \
        "%s.%s.value 必须是数值" % (joint, field)
    assert value > 0, "%s.%s.value 必须为正数" % (joint, field)
    assert str(item.get("source") or "").strip(), "%s.%s.source 必填" % (joint, field)
    assert str(item.get("method") or "").strip(), "%s.%s.method 必填" % (joint, field)
    samples = item.get("samples")
    assert isinstance(samples, int) and samples >= 3, \
        "%s.%s.samples 必须 ≥ 3（单次极值不算证据）" % (joint, field)


@pytest.mark.parametrize("joint", list(JOINTS))
@pytest.mark.parametrize("field", ["acceleration", "jerk"])
def test_joint_value_is_verbatim_conservative_min(joint, field):
    """value 逐字等于 min(工件实测包络, 回填前已生效限值)——不得手抄/取整。"""
    item = _joint_entry(_baseline(), joint)[field]
    expected = _expected_value(joint, field)
    assert item["value"] == expected, \
        "%s.%s.value=%r 与 min(工件包络, 已生效限值)=%r 不一致" % (
            joint, field, item["value"], expected)
    # 保守口径：实测包络更高时不得放宽
    envelope = _envelope(joint)[0 if field == "acceleration" else 1]
    assert item["value"] <= envelope + 0.0, \
        "%s.%s.value 高于实测包络（不得放宽）" % (joint, field)


def test_joint1_jerk_keeps_existing_limit_and_states_reason():
    """joint1 的 jerk 实测包络高于已生效限值 → value 必须是 30.0，method 写明原因。"""
    entry = _joint_entry(_baseline(), "joint1")
    jerk = entry["jerk"]
    envelope = _s01_joint1_envelope()[1]
    assert envelope > OLD_JERK, "S01 工件的 joint1 jerk 包络应高于 30.0（前提）"
    assert jerk["value"] == OLD_JERK, \
        "joint1.jerk.value 必须保持 30.0（不得放宽成实测值 %r）" % envelope
    assert jerk["value"] != pytest.approx(envelope, rel=1e-3), \
        "joint1.jerk.value 不得写成实测包络"
    method = str(jerk.get("method") or "")
    assert repr(envelope) in method, "method 必须写明实测包络 %r" % envelope
    assert "30.0" in method, "method 必须写明已生效限值 30.0"
    assert ("保持" in method) or ("不放宽" in method), \
        "method 必须写明原因（实测包络高于已生效限值 → 保持 30.0，不得放宽）"
    assert str(jerk.get("source") or "").find("rm75_validation_ladder_20261119.json") >= 0, \
        "joint1 的 source 必须指向 S01 工件"


@pytest.mark.parametrize("joint", list(JOINTS))
def test_joint_limits_only_tightened(joint):
    """joint_limits.yaml 逐关节 acceleration/jerk 不大于回填前现值（只收紧不放宽）。"""
    limits = _limits()
    assert joint in limits, "joint_limits.yaml 缺 %s" % joint
    acc = limits[joint].get("acceleration")
    jerk = limits[joint].get("jerk")
    assert isinstance(acc, (int, float)) and acc <= OLD_ACCELERATION + 1e-12, \
        "%s.acceleration=%r 不得高于回填前现值 %r" % (joint, acc, OLD_ACCELERATION)
    assert isinstance(jerk, (int, float)) and jerk <= OLD_JERK + 1e-12, \
        "%s.jerk=%r 不得高于回填前现值 %r" % (joint, jerk, OLD_JERK)


@pytest.mark.parametrize("joint", list(JOINTS))
def test_joint_limits_match_baseline_values(joint):
    """已生效限值必须与基线逐字相等（回填真的生效了，不是各写各的）。"""
    limits = _limits()
    entry = _joint_entry(_baseline(), joint)
    assert limits[joint].get("acceleration") == entry["acceleration"]["value"], \
        "%s.acceleration 已生效值与基线不一致" % joint
    assert limits[joint].get("jerk") == entry["jerk"]["value"], \
        "%s.jerk 已生效值与基线不一致" % joint


def test_build_model_copy_is_in_sync_when_present():
    """build/model/joint_limits.yaml（rmctl 实际读取的那份，gitignore）若存在必须同步。

    不刷新副本的话「回填」不会生效（rmctl.paths.joint_limits_path() 优先读副本）。
    """
    built = os.path.join(REPO_ROOT, "build", "model", "joint_limits.yaml")
    if not os.path.exists(built):
        pytest.skip("本机没有 build/model 副本（干净检出）")
    with open(built, "r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh) or {}
    group = data.get("rm75_arm") or {}
    limits = _limits()
    for joint in JOINTS:
        assert group.get(joint, {}).get("acceleration") == limits[joint]["acceleration"], \
            "build/model 副本的 %s.acceleration 与真值不一致（回填未生效）" % joint
        assert group.get(joint, {}).get("jerk") == limits[joint]["jerk"], \
            "build/model 副本的 %s.jerk 与真值不一致（回填未生效）" % joint


def test_tcp_repeatability_is_verbatim_from_s01_v7():
    """TCP 重复性必须逐字等于 S01 工件 V7 阶段 max_deviation_mm。"""
    expected = _s01_stage("V7")["data"]["max_deviation_mm"]
    tcp = _baseline().get("tcp_repeatability") or {}
    assert tcp.get("value_mm") == expected, \
        "tcp_repeatability.value_mm=%r 与 S01 工件 V7 的 %r 不一致" % (
            tcp.get("value_mm"), expected)
    assert tcp.get("samples") == 30, tcp.get("samples")
    assert str(tcp.get("source") or "").find("rm75_validation_ladder_20261119.json") >= 0
    assert str(tcp.get("method") or "").strip()


def test_voided_transcribed_tcp_value_is_gone():
    """无工件背书的旧转录值必须被作废，且不得与正式值并存。"""
    text = _baseline_text()
    assert repr(VOIDED_TCP_VALUE) not in text, \
        "已作废的转录值 %r 仍出现在基线文件里" % VOIDED_TCP_VALUE
    partial = _baseline().get("partial_observations") or {}
    voided = partial.get("voided_transcription") or {}
    assert voided, "partial_observations 必须显式记录被作废的转录值"
    assert "作废" in str(voided.get("reason") or ""), voided.get("reason")


def test_partial_observations_holds_no_numeric_values():
    """partial_observations 不得再保留数值（避免与正式字段重复/无背书的数字）。"""
    partial = _baseline().get("partial_observations") or {}
    assert partial, "partial_observations 段应保留（口径说明）"
    assert _numeric_leaves(partial) == [], \
        "partial_observations 里有数值叶子：%r" % _numeric_leaves(partial)


def test_baseline_check_cli_exits_zero():
    """python3 -m rmctl baseline check 退出码 0（基线自洽）。"""
    env = dict(os.environ)
    for rel in ("dualarm_ws/src/rmctl", "dualarm_ws/src/rm_hw_bridge",
                "dualarm_ws/src/rm_safety", "dualarm_ws/src/rm_executor"):
        path = os.path.join(REPO_ROOT, rel)
        if path not in env.get("PYTHONPATH", "").split(os.pathsep):
            env["PYTHONPATH"] = path + os.pathsep + env.get("PYTHONPATH", "")
    proc = subprocess.run([sys.executable, "-m", "rmctl", "baseline", "check"],
                          cwd=REPO_ROOT, env=env, stdout=subprocess.PIPE,
                          stderr=subprocess.STDOUT)
    out = proc.stdout.decode("utf-8", "replace")
    assert proc.returncode == 0, "baseline check 退出码 %r：\n%s" % (
        proc.returncode, out[-3000:])
    assert "measured_limits" not in out.split("基线被阻塞")[-1], \
        "measured_limits 不应再出现在被阻塞清单里：\n%s" % out[-2000:]


# ------------------------------------------------------------------ 负例（Q7）
def test_value_above_joint_limit_is_rejected_by_validator(tmp_workspace):
    """负例：基线 value 高于已生效限值 → rmctl.baseline 必须判 error。"""
    path = os.path.join(tmp_workspace, "workcell", "baselines", "measured_limits.yaml")
    with open(path, "r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    data["joints"][0]["acceleration"]["value"] = OLD_ACCELERATION  # > 已生效限值
    with open(path, "w", encoding="utf-8") as fh:
        yaml.safe_dump(data, fh, allow_unicode=True, sort_keys=False)
    report = bl.validate_all(tmp_workspace)
    assert not report["ok"], "基线值高于已生效限值却判定通过"
    codes = {i.get("code") for i in report.get("issues") or []
             if i.get("level") == "error"}
    assert "limit_above_joint_limits" in codes, sorted(codes)


def test_loosened_joint_limits_fail_the_guard():
    """负例：joint_limits 被放宽（> 回填前现值）→ 守卫判失败。"""
    limits = _limits()
    assert _loosened_joints(limits) == [], \
        "已生效限值不得被放宽：%r" % _loosened_joints(limits)
    loosened = {j: dict(v) for j, v in limits.items()}
    loosened["joint3"]["acceleration"] = OLD_ACCELERATION + 0.1
    assert _loosened_joints(loosened) == [("joint3", "acceleration")]


def _loosened_joints(limits, cap_acc=OLD_ACCELERATION, cap_jerk=OLD_JERK):
    """返回"比回填前现值更宽松"的 (joint, field) 列表。"""
    out = []
    for joint, values in (limits or {}).items():
        acc = values.get("acceleration")
        jerk = values.get("jerk")
        if isinstance(acc, (int, float)) and acc > cap_acc + 1e-12:
            out.append((joint, "acceleration"))
        if isinstance(jerk, (int, float)) and jerk > cap_jerk + 1e-12:
            out.append((joint, "jerk"))
    return out


def test_backfill_tool_writes_measured_value_without_truncation(tmp_workspace):
    """负例：回填工具若把实测值截断（如只留 9 位小数）→ 守卫判失败。

    截断会让写盘值与基线值（工件逐字值）出现 <1e-9 的差，
    ``rmctl baseline check`` 是精确比较 → 会自相矛盾地报 ``limit_above_joint_limits``。
    """
    script = os.path.join(REPO_ROOT, "dualarm_ws", "src", "rm_description",
                          "scripts", "apply_measured_limits.py")
    spec = importlib.util.spec_from_file_location("apply_measured_limits_s13", script)
    aml = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(aml)

    rc = aml.main(["--ws", tmp_workspace, "--mode", "conservative",
                   "--date", "2026-11-23"])
    assert rc == 0, "保守回填应成功（退出码 %r）" % rc
    written_path = os.path.join(tmp_workspace, "dualarm_ws", "src", "rm_description",
                                "config", "joint_limits.yaml")
    with open(written_path, "r", encoding="utf-8") as fh:
        text = fh.read()
    written = (yaml.safe_load(text) or {}).get("rm75_arm") or {}
    baseline = _baseline()
    for joint in JOINTS:
        expected = _joint_entry(baseline, joint)["acceleration"]["value"]
        assert written[joint]["acceleration"] == expected, \
            "%s.acceleration 写盘值 %r ≠ 基线值 %r（精度被截断）" % (
                joint, written[joint]["acceleration"], expected)
        assert repr(expected) in text, \
            "%s.acceleration 的写盘文本不含完整实测值 %r" % (joint, expected)
