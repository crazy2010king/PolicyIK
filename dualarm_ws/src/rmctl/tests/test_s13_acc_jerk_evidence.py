"""S13-T3 阶梯 acc/jerk 真机实测工件的验收用例（``rm75_s13_acc_jerk.json``）。

覆盖四层：

* **工件合规层**：schema / status / joint2..joint7 六关节齐全 /
  每关节样本数 ≥3 且与 trials 一致 / 每样本 ``returned_to_origin`` 为真 /
  单位是 rad/s^2 与 rad/s^3 / honesty 语义 /
  **工件里不含 joint1**（joint1 的唯一来源是 S01 工件的 V8 阶段）；
* **CLI 契约层**：`--joints` 的筛选**真的生效**（dry-run 只输出所选关节）、
  缺省仍是全部 7 个、非法输入退出码 2、空输入退出码 2；
* **负例层**：样本数 <3 即失败 / 出现 joint1 即失败；
* **脚本行为层**：``--dry-run`` 不连任何设备、输出 ``status=dry_run`` 骨架，
  且**不会覆盖**真机工件。
"""
from __future__ import annotations

import copy
import importlib.util
import json
import os
import subprocess
import sys

import pytest

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(TESTS_DIR, "..", "..", "..", ".."))
SCRIPT_PATH = os.path.join(REPO_ROOT, "scripts", "rm75_s13_acc_jerk_evidence.py")
ARTIFACT_PATH = os.path.join(
    REPO_ROOT, "workcell", "baselines", "evidence", "rm75_s13_acc_jerk.json",
)
#: S01 工件（joint1 的**唯一**来源）
S01_ARTIFACT_PATH = os.path.join(
    REPO_ROOT, "workcell", "baselines", "evidence",
    "rm75_validation_ladder_20261119.json",
)
EXPECTED_JOINTS = ("joint2", "joint3", "joint4", "joint5", "joint6", "joint7")


def _load_script():
    """按路径加载脚本模块（脚本不是包，用 importlib 加载）。"""
    spec = importlib.util.spec_from_file_location("rm75_s13_acc_jerk_evidence",
                                                  SCRIPT_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def script():
    return _load_script()


@pytest.fixture(scope="module")
def artifact():
    assert os.path.exists(ARTIFACT_PATH), (
        "缺少 S13-T3 证据工件：%s" % ARTIFACT_PATH)
    with open(ARTIFACT_PATH, "r", encoding="utf-8") as fh:
        return json.load(fh)


def _synthetic_ok_artifact(script):
    """构造一份 **status=ok 形状**的工件（只用于负例层）。

    负例要验证的是「合规判据会拒绝坏数据」；真实工件当前可能是 blocked
    （窗口不可达），没有样本可改，因此负例用一份**形状正确**的合成工件。
    """
    joints = {}
    for number in script.DEFAULT_JOINTS:
        name = "joint%d" % number
        joints[name] = {
            "acceleration_rad_s2": 1.0,
            "jerk_rad_s3": 2.0,
            "envelope": "min over trials（保守下界）",
            "trials": 3,
            "speeds": [5, 8, 11],
            "samples": [
                {"trial": t, "speed": 5 + 3 * t, "acc_peak_rad_s2": 1.0,
                 "jerk_peak_rad_s3": 2.0, "returned_to_origin": True}
                for t in range(3)
            ],
        }
    return {
        "schema": script.SCHEMA,
        "status": "ok",
        "joints": joints,
        "unit": {"acceleration": "rad/s^2", "jerk": "rad/s^3"},
        "derivative_method": "rmctl.retime.derivatives",
        "step_deg": script.STEP_DEG,
        "honesty": script.HONESTY_STATEMENT,
        "safety_preconditions": {"ok": True, "checks": [{"name": "x", "ok": True,
                                                         "detail": "y"}]},
        "next_action": ["n/a"],
    }


def _run_cli(*args, expect=None):
    """跑 ``python3 -m rmctl.hwcollect ladder_acc_jerk ...`` 并返回 CompletedProcess。"""
    cmd = [sys.executable, "-m", "rmctl.hwcollect", "ladder_acc_jerk", *args]
    env = dict(os.environ)
    env["PYTHONPATH"] = os.pathsep.join(
        [os.path.join(REPO_ROOT, "dualarm_ws", "src", "rmctl"),
         os.path.join(REPO_ROOT, "dualarm_ws", "src", "rm_hw_bridge"),
         env.get("PYTHONPATH", "")])
    proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                          env=env, timeout=300)
    if expect is not None:
        assert proc.returncode == expect, (
            "退出码 %d（期望 %d）：\n%s"
            % (proc.returncode, expect, proc.stdout.decode("utf-8", "replace")))
    return proc


# --------------------------------------------------------------------------
# 工件合规层
# --------------------------------------------------------------------------

def test_artifact_schema_and_status(artifact, script):
    """schema 固定，status 合法。"""
    assert artifact["schema"] == "rmctl.rm75_s13_acc_jerk/1"
    assert artifact["schema"] == script.SCHEMA
    assert artifact["status"] in ("ok", "blocked", "error")


def test_artifact_passes_self_validation(artifact, script):
    """工件必须通过脚本内置自检（问题列表为空）。"""
    assert script.validate_artifact(artifact, list(script.DEFAULT_JOINTS)) == []


def test_all_six_joints_present(artifact):
    """joint2..joint7 六关节齐全（缺一即失败）。"""
    joints = artifact["joints"]
    for name in EXPECTED_JOINTS:
        assert name in joints, "缺关节 %s" % name
        assert isinstance(joints[name], dict)


def test_artifact_has_no_joint1(artifact):
    """**工件中不含 joint1**（joint1 的唯一来源是 S01 工件，避免两个来源打架）。"""
    assert "joint1" not in artifact["joints"], (
        "工件不得包含 joint1（joint1 的唯一来源是 S01 工件）")
    # 逐字扫一遍整个工件，确保没有别处藏 joint1
    text = json.dumps(artifact, ensure_ascii=False)
    assert "joint1" not in text, "工件正文出现 joint1"


def test_joint1_source_is_the_s01_artifact():
    """joint1 的唯一来源是 S01 工件的 V8 阶段（守卫测试自身也钉死这一点）。"""
    with open(S01_ARTIFACT_PATH, "r", encoding="utf-8") as fh:
        s01 = json.load(fh)
    stage = next(s for s in s01["stages"] if s["id"] == "V8")
    data = stage["data"]
    assert data["acceleration_rad_s2"]["joint1"] == 1.755718584483999
    assert data["jerk_rad_s3"]["joint1"] == 81.09893830324319


def test_each_joint_has_at_least_three_samples(artifact):
    """每关节 acc/jerk 的样本数 ≥3，且逐样本条数与 trials 一致。

    仅在 ``status=ok``（真机实测完成）时要求；``status=blocked`` 时工件
    **不得含任何读数**（由 :func:`test_blocked_artifact_has_no_fabricated_numbers` 钉死）。
    """
    if artifact["status"] != "ok":
        pytest.skip("status=%s：真机读数不存在，本组断言不适用"
                    % artifact["status"])
    for name in EXPECTED_JOINTS:
        entry = artifact["joints"][name]
        samples = entry["samples"]
        assert len(samples) >= 3, "%s 样本数 %d < 3" % (name, len(samples))
        assert entry["trials"] == len(samples), (
            "%s 的 trials(%r) 与逐样本条数(%d) 不一致"
            % (name, entry["trials"], len(samples)))
        assert entry["acceleration_rad_s2"] is not None
        assert entry["jerk_rad_s3"] is not None
        for sample in samples:
            assert sample["acc_peak_rad_s2"] is not None
            assert sample["jerk_peak_rad_s3"] is not None
            assert sample["speed"] is not None


def test_every_sample_returned_to_origin(artifact):
    """每样本 returned_to_origin 必须为真（净位移 0，档位不叠加）。"""
    if artifact["status"] != "ok":
        pytest.skip("status=%s：真机读数不存在，本组断言不适用"
                    % artifact["status"])
    for name in EXPECTED_JOINTS:
        for sample in artifact["joints"][name]["samples"]:
            assert sample["returned_to_origin"] is True, (
                "%s 有样本未回原位" % name)


def test_blocked_artifact_has_no_fabricated_numbers(artifact):
    """**status≠ok 的工件不得含任何读数**（禁止用 dry-run / 合成值冒充真机读数）。"""
    if artifact["status"] == "ok":
        pytest.skip("status=ok：工件含真机读数，本组断言不适用")
    for name in EXPECTED_JOINTS:
        entry = artifact["joints"][name]
        assert entry["acceleration_rad_s2"] is None, (
            "%s 在 status=%s 时不得含 acc 数值" % (name, artifact["status"]))
        assert entry["jerk_rad_s3"] is None, (
            "%s 在 status=%s 时不得含 jerk 数值" % (name, artifact["status"]))
        assert not entry["samples"], (
            "%s 在 status=%s 时不得含逐样本读数" % (name, artifact["status"]))


def test_blocked_artifact_self_certifies(artifact):
    """status≠ok 的工件必须**自证**：前置检查逐项结果 + 可粘贴下一步。"""
    if artifact["status"] == "ok":
        pytest.skip("status=ok：工件含真机读数，本组断言不适用")
    checks = artifact["safety_preconditions"]["checks"]
    assert checks, "blocked 工件必须保留前置检查逐项结果"
    for check in checks:
        assert "name" in check and "ok" in check
        assert check["detail"], "每项前置检查必须有可读的 detail"
    assert artifact["safety_preconditions"]["ok"] is False
    assert artifact["next_action"], "blocked 工件必须有可粘贴 next_action"
    assert artifact["reason"], "blocked 工件必须写明阻塞原因"


def test_units_are_rad(artifact):
    """单位是 rad/s^2 与 rad/s^3。"""
    assert artifact["unit"]["acceleration"] == "rad/s^2"
    assert artifact["unit"]["jerk"] == "rad/s^3"


def test_derivative_method_and_step_deg(artifact):
    """微分语义与步长与 hwcollect 一致。"""
    assert artifact["derivative_method"] == "rmctl.retime.derivatives"
    assert artifact["step_deg"] == 10.0


def test_honesty_statement_present(artifact):
    """honesty 必须显式声明「包络…不代表控制器最大能力…保守下界」。"""
    honesty = artifact["honesty"]
    assert isinstance(honesty, str) and honesty.strip()
    assert "包络" in honesty
    assert "不代表控制器最大能力" in honesty
    assert "保守下界" in honesty


def test_safety_preconditions_recorded(artifact):
    """前置检查逐项结果必须落在工件里。"""
    pre = artifact["safety_preconditions"]
    assert isinstance(pre, dict)
    checks = pre["checks"]
    assert isinstance(checks, list) and checks
    names = {c["name"] for c in checks}
    if artifact["status"] == "ok":
        for required in ("all_joints_enabled", "joint_err_code_zero",
                         "controller_no_planning", "controller_idle",
                         "target_within_execution_domain"):
            assert required in names, "前置检查缺 %s" % required
        assert pre["ok"] is True, "status=ok 时前置检查必须全过"
    else:
        # 窗口不可达时，前置检查在第一项（只读快照）就失败并如实记录。
        assert pre["ok"] is False
        assert names, "blocked 工件必须记录失败的前置检查项"


def test_next_action_is_pasteable(artifact):
    """必须给出可粘贴的下一步。"""
    action = artifact["next_action"]
    assert isinstance(action, list) and action
    assert all(isinstance(item, str) and item.strip() for item in action)


# --------------------------------------------------------------------------
# 负例层
# --------------------------------------------------------------------------

def test_synthetic_ok_artifact_passes_validation(script):
    """**ok 形状**的工件必须完全合规。

    证明：一旦真机窗口可用（远端自动执行器写出 status=ok 工件），
    本文件对 ok 路径的断言（样本数 ≥3 / 逐样本回原位 / 单位 / honesty /
    六关节齐全）会全部成立 —— 避免「ok 路径其实没被测过」。
    """
    good = _synthetic_ok_artifact(script)
    assert script.validate_artifact(good, list(script.DEFAULT_JOINTS)) == [], (
        "ok 形状的工件必须自检通过")
    for name in EXPECTED_JOINTS:
        entry = good["joints"][name]
        assert len(entry["samples"]) >= 3
        assert entry["trials"] == len(entry["samples"])
        assert all(s["returned_to_origin"] is True for s in entry["samples"])


def test_negative_too_few_samples_is_rejected(script):
    """负例：样本数 <3 ⇒ 不合规。"""
    broken = _synthetic_ok_artifact(script)
    entry = broken["joints"]["joint3"]
    entry["samples"] = entry["samples"][:2]
    entry["trials"] = 2
    problems = script.validate_artifact(broken, list(script.DEFAULT_JOINTS))
    assert problems, "样本数 <3 必须被判为不合规"
    assert any("joint3" in p and "样本数" in p for p in problems)


def test_negative_joint1_present_is_rejected(script):
    """负例：工件出现 joint1 ⇒ 不合规。"""
    broken = _synthetic_ok_artifact(script)
    broken["joints"]["joint1"] = copy.deepcopy(broken["joints"]["joint2"])
    problems = script.validate_artifact(broken, list(script.DEFAULT_JOINTS))
    assert problems, "出现 joint1 必须被判为不合规"
    assert any("joint1" in p for p in problems)


def test_negative_not_returned_to_origin_is_rejected(script):
    """负例：有样本没回原位 ⇒ 不合规。"""
    broken = _synthetic_ok_artifact(script)
    broken["joints"]["joint5"]["samples"][0]["returned_to_origin"] = False
    problems = script.validate_artifact(broken, list(script.DEFAULT_JOINTS))
    assert problems, "未回原位必须被判为不合规"
    assert any("未回原位" in p for p in problems)


def test_negative_wrong_unit_is_rejected(artifact, script):
    """负例：单位不是 rad/s^2 / rad/s^3 ⇒ 不合规。"""
    broken = copy.deepcopy(artifact)
    broken["unit"]["jerk"] = "deg/s^3"
    problems = script.validate_artifact(broken, list(script.DEFAULT_JOINTS))
    assert any("rad/s^3" in p for p in problems)


def test_negative_missing_honesty_is_rejected(artifact, script):
    """负例：缺 honesty 声明 ⇒ 不合规。"""
    broken = copy.deepcopy(artifact)
    broken["honesty"] = ""
    problems = script.validate_artifact(broken, list(script.DEFAULT_JOINTS))
    assert any("honesty" in p for p in problems)


def test_negative_fabricated_numbers_in_blocked_artifact_are_rejected(script):
    """负例：blocked 工件里塞进数值读数 ⇒ 不合规（禁止伪造真机读数）。"""
    broken = _synthetic_ok_artifact(script)
    broken["status"] = "blocked"
    problems = script.validate_artifact(broken, list(script.DEFAULT_JOINTS))
    assert problems, "blocked 工件含读数必须被判为不合规"
    assert any("禁止伪造读数" in p for p in problems)


def test_negative_joint1_cli_is_rejected(script):
    """负例：--joints 含 joint1 ⇒ 拒绝（joint1 来源唯一，不允许本轮重测）。"""
    with pytest.raises(ValueError):
        script.parse_s13_joints("1,2")
    with pytest.raises(ValueError):
        script.parse_s13_joints("joint1")


# --------------------------------------------------------------------------
# CLI 契约层
# --------------------------------------------------------------------------

def test_cli_joints_filter_only_outputs_selected_joints(tmp_path):
    """--joints 真的生效：dry-run 只输出所选关节。"""
    out = str(tmp_path / "ladder")
    proc = _run_cli("--joints", "2,3", "--trials", "3", "--dry-run", "--out", out)
    text = proc.stdout.decode("utf-8", "replace")
    assert "joint2" in text
    assert "joint3" in text
    for absent in ("joint1", "joint4", "joint5", "joint6", "joint7"):
        assert absent not in text, "未选中的关节 %s 出现在输出里" % absent

    with open(os.path.join(out, "ladder_acc_jerk.left.json"), "r",
              encoding="utf-8") as fh:
        data = json.load(fh)
    assert set(data["summary"]["acceleration_rad_s2"]) == {"joint2", "joint3"}
    assert set(data["summary"]["jerk_rad_s3"]) == {"joint2", "joint3"}


def test_cli_jointn_notation_accepted(tmp_path):
    """--joints 支持 jointN 写法。"""
    out = str(tmp_path / "ladder")
    proc = _run_cli("--joints", "joint2,joint3", "--trials", "3", "--dry-run",
                    "--out", out)
    text = proc.stdout.decode("utf-8", "replace")
    assert "joint2" in text and "joint3" in text


def test_cli_default_is_all_seven_joints(tmp_path):
    """缺省 --joints 时行为不变：仍是全部 7 个关节。"""
    out = str(tmp_path / "ladder")
    _run_cli("--trials", "3", "--dry-run", "--out", out)
    with open(os.path.join(out, "ladder_acc_jerk.left.json"), "r",
              encoding="utf-8") as fh:
        data = json.load(fh)
    assert len(data["summary"]["acceleration_rad_s2"]) == 7


def test_cli_invalid_joints_exits_2(tmp_path):
    """非法 --joints ⇒ stderr 提示 + 退出码 2。"""
    out = str(tmp_path / "ladder")
    proc = _run_cli("--joints", "abc", "--dry-run", "--out", out, expect=2)
    text = proc.stdout.decode("utf-8", "replace")
    assert "--joints" in text


def test_cli_out_of_range_joints_exits_2(tmp_path):
    """越界关节号（0 / 8）⇒ 退出码 2。"""
    out = str(tmp_path / "ladder")
    _run_cli("--joints", "0", "--dry-run", "--out", out, expect=2)
    _run_cli("--joints", "8", "--dry-run", "--out", out, expect=2)


def test_cli_empty_joints_exits_2(tmp_path):
    """空 --joints ⇒ 退出码 2。"""
    out = str(tmp_path / "ladder")
    proc = _run_cli("--joints", "", "--dry-run", "--out", out, expect=2)
    assert "不能为空" in proc.stdout.decode("utf-8", "replace")


# --------------------------------------------------------------------------
# 脚本行为层
# --------------------------------------------------------------------------

def test_script_dry_run_does_not_connect_and_writes_skeleton(tmp_path):
    """--dry-run：不连任何设备、status=dry_run、且不覆盖真机工件。"""
    out = tmp_path / "s13_acc_jerk"
    proc = subprocess.run(
        [sys.executable, SCRIPT_PATH, "--stage", "all", "--dry-run",
         "--out", str(out)],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=300,
    )
    assert proc.returncode == 0, proc.stdout.decode("utf-8", "replace")
    written = tmp_path / "s13_acc_jerk.dryrun.json"
    assert written.exists()
    with open(written, "r", encoding="utf-8") as fh:
        data = json.load(fh)
    assert data["status"] == "dry_run"
    assert data["dry_run"] is True


def test_script_dry_run_never_overwrites_the_real_artifact(tmp_path):
    """dry-run 的输出路径必须与真机工件路径错开。"""
    before = open(ARTIFACT_PATH, "rb").read()
    out = tmp_path / os.path.basename(ARTIFACT_PATH)
    proc = subprocess.run(
        [sys.executable, SCRIPT_PATH, "--stage", "all", "--dry-run",
         "--out", str(out)],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=300,
    )
    assert proc.returncode == 0
    assert open(ARTIFACT_PATH, "rb").read() == before


# --------------------------------------------------------------------------
# --stage assemble 组装层（不重跑运动）
# --------------------------------------------------------------------------

def _write_precheck_record(path, checks):
    """写一份「运动前的前置检查记录」（门禁留痕）。"""
    with open(path, "w", encoding="utf-8") as fh:
        json.dump({"checks": checks, "snapshot": {"joint_deg": [0.0] * 7},
                   "reason": "", "realtime_enabled": True}, fh,
                  ensure_ascii=False, indent=2)


def _write_ladder_report(path, script):
    """写一份 hwcollect 阶梯报告（形状与真机报告一致）。"""
    joints = {}
    samples = []
    for number in script.DEFAULT_JOINTS:
        name = "joint%d" % number
        accs, jerks = [], []
        for trial, speed in enumerate((5, 8, 11)):
            acc = 1.0 + 0.5 * trial
            jerk = 2.0 + 0.5 * trial
            accs.append(acc)
            jerks.append(jerk)
            samples.append({"joint": name, "trial": trial, "speed": speed,
                            "acc_peak_rad_s2": acc, "jerk_peak_rad_s3": jerk,
                            "returned_to_origin": True})
        joints[name] = (min(accs), min(jerks))
    report = {
        "status": "ok",
        "summary": {
            "acceleration_rad_s2": {k: v[0] for k, v in joints.items()},
            "jerk_rad_s3": {k: v[1] for k, v in joints.items()},
        },
        "samples": samples,
    }
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(report, fh, ensure_ascii=False, indent=2)


def test_stage_assemble_builds_ok_artifact_without_moving_the_arm(script,
                                                                   tmp_path):
    """``--stage assemble``：读回前置检查记录 + hwcollect 报告 → status=ok 工件。

    这条路径保证「运动已跑完、但组装阶段被会话断开杀掉」时**不必重跑运动**
    就能把真实读数落进工件。
    """
    report_dir = tmp_path / "ladder"
    report_dir.mkdir()
    _write_ladder_report(str(report_dir / "ladder_acc_jerk.left.json"), script)
    checks = [{"name": "all_joints_enabled", "ok": True, "detail": "7 轴全部使能"},
              {"name": "joint_err_code_zero", "ok": True, "detail": "无"},
              {"name": "controller_idle", "ok": True, "detail": "RM_IDLE_E"},
              {"name": "controller_no_planning", "ok": True,
               "detail": "RM_NO_PLANNING_E"},
              {"name": "target_within_execution_domain", "ok": True,
               "detail": "执行域内"}]
    _write_precheck_record(str(report_dir / "precheck.json"), checks)
    out = tmp_path / "assembled.json"
    rc = script.main([
        "--stage", "assemble", "--arm", "left", "--ip", "192.168.1.18",
        "--joints", "2,3,4,5,6,7", "--trials", "3",
        "--out", str(out), "--report-dir", str(report_dir),
        "--precheck-out", str(report_dir / "precheck.json"),
    ])
    assert rc == 0
    with open(str(out), "r", encoding="utf-8") as fh:
        data = json.load(fh)
    assert data["status"] == "ok"
    assert data["self_check"]["ok"] is True
    for name in EXPECTED_JOINTS:
        entry = data["joints"][name]
        assert entry["acceleration_rad_s2"] is not None
        assert len(entry["samples"]) >= 3


def test_stage_assemble_without_precheck_record_is_not_ok(script, tmp_path):
    """负例：没有前置检查记录 ⇒ 工件不得是 ok（运动门禁不可缺）。"""
    report_dir = tmp_path / "ladder"
    report_dir.mkdir()
    _write_ladder_report(str(report_dir / "ladder_acc_jerk.left.json"), script)
    out = tmp_path / "assembled.json"
    rc = script.main([
        "--stage", "assemble", "--arm", "left", "--ip", "192.168.1.18",
        "--joints", "2,3,4,5,6,7", "--trials", "3",
        "--out", str(out), "--report-dir", str(report_dir),
        "--precheck-out", str(report_dir / "missing-precheck.json"),
    ])
    assert rc != 0
    with open(str(out), "r", encoding="utf-8") as fh:
        data = json.load(fh)
    assert data["status"] != "ok"


# --------------------------------------------------------------------------
# controller_idle 判据（臂状态来源）
# --------------------------------------------------------------------------

def _snapshot_for_precheck(status_name):
    return {
        "arm_err": [0],
        "joint_err_code": [0] * 7,
        "joint_en_flag": [1] * 7,
        "controller_trajectory_type_name": "RM_NO_PLANNING_E",
        "arm_current_status_name": status_name,
        "joint_deg": [0.0] * 7,
        "realtime": {"frames": 1},
    }


def _limits_for_precheck():
    limits = {}
    for number in range(1, 8):
        limits["joint%d" % number] = {
            "execution_lower_rad": -1.7, "execution_upper_rad": 1.7,
            "plan_lower_rad": -3.14, "plan_upper_rad": 3.14,
            "criterion": "execution domain（执行域）",
        }
    return limits


def test_controller_idle_passes_when_arm_state_is_reported(script):
    """臂状态读到 RM_IDLE_E ⇒ controller_idle 通过（来源写进 detail）。"""
    checks = script.precheck(_snapshot_for_precheck("RM_IDLE_E"),
                             list(script.DEFAULT_JOINTS),
                             _limits_for_precheck())
    idle = [c for c in checks if c["name"] == "controller_idle"]
    assert idle and idle[0]["ok"] is True
    assert "RM_IDLE_E" in idle[0]["detail"]
    assert "UDP" in idle[0]["detail"]


def test_controller_idle_fails_when_arm_state_is_missing(script):
    """负例：臂状态读不到（UDP 上报没建立）⇒ controller_idle 必须判失败。

    回归这条是因为历史上前置检查把 realtime 关掉了，而臂状态**只**存在于
    UDP 上报里 —— 那样 ``controller_idle`` 会永远判失败（真机窗口永远 blocked）。
    """
    checks = script.precheck(_snapshot_for_precheck(None),
                             list(script.DEFAULT_JOINTS),
                             _limits_for_precheck())
    idle = [c for c in checks if c["name"] == "controller_idle"]
    assert idle and idle[0]["ok"] is False
    assert "未取到" in idle[0]["detail"]
