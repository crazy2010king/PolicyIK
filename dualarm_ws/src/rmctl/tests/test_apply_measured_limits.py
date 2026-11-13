"""实测限值回填工具（apply_measured_limits.py）的行为测试。

覆盖的正例 / 负例：

* 正例：保守模式取 min、实测模式取实测值、--dry-run 不写盘、写盘后仍是合法
  YAML 且 7 个关节键齐全、原注释逐字保留、来源与日期追加在文件尾。
* 负例：基线未实测（status≠measured）、样本数不足、缺 source、缺 method、
  值为 null、单位不符、负值 / 零、超出物理合理性上限、限值文件里没有该关节、
  基线文件缺失（退出码 2）。
* 回归口径：新限值更严 → 保路径降速兜底（required_scale > 1）；更松 → 直接通过。

只用 git 跟踪的仓库文件 + pytest 临时目录（conftest.tmp_workspace）。
"""
import importlib.util
import os
import shutil

import numpy as np
import pytest
import yaml

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(TESTS_DIR, "..", "..", "..", ".."))
SCRIPT = os.path.join(REPO_ROOT, "dualarm_ws", "src", "rm_description", "scripts",
                      "apply_measured_limits.py")

_spec = importlib.util.spec_from_file_location("apply_measured_limits", SCRIPT)
aml = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(aml)

JOINTS = tuple("joint%d" % i for i in range(1, 8))


# ------------------------------------------------------------------ 夹具
def _item(value, unit, source="真机阶梯实验", method="阶梯速度包络", samples=5):
    return {"value": value, "unit": unit, "source": source, "method": method,
            "samples": samples}


def measured_baseline(acc=4.0, jerk=20.0, status="measured"):
    """构造一份合法的 measured_limits 数据（可按需破坏某个字段）。"""
    return {
        "kind": "measured_limits",
        "schema_version": 1,
        "version": 2,
        "status": status,
        "captured_at": "2026-11-13T09:00:00",
        "arm": "left",
        "udp_period_ms": 5,
        "joints": [{"name": j,
                    "acceleration": _item(acc, "rad/s^2"),
                    "jerk": _item(jerk, "rad/s^3")} for j in JOINTS],
        "tcp_repeatability": {"value_mm": 0.05, "unit": "mm",
                              "source": "真机 30 次", "method": "rm_movej 固定目标",
                              "samples": 30},
    }


def write_baseline(ws, data):
    path = os.path.join(ws, "workcell", "baselines", "measured_limits.yaml")
    with open(path, "w", encoding="utf-8") as fh:
        yaml.safe_dump(data, fh, allow_unicode=True, sort_keys=False)
    return path


def limits_path(ws):
    return os.path.join(ws, "dualarm_ws", "src", "rm_description", "config",
                        "joint_limits.yaml")


def read_limits(ws):
    with open(limits_path(ws), "r", encoding="utf-8") as fh:
        text = fh.read()
    return text, yaml.safe_load(text)


def codes(report, level="error"):
    return [i["code"] for i in report["issues"] if i["level"] == level]


def seven_limits(acc=5.2359, jerk=30.0):
    """限值文件的 rm75_arm 组（7 个关节齐全）。"""
    return {"rm75_arm": {j: {"acceleration": acc, "jerk": jerk} for j in JOINTS}}


# ------------------------------------------------------------------ 正例
def test_conservative_mode_takes_min_of_measured_and_current(tmp_workspace):
    """保守模式：新值 = min(实测, 现值)，只可能更保守。"""
    limits = seven_limits()
    base = measured_baseline(acc=4.0, jerk=20.0)
    rep = aml.plan_backfill(base, limits, mode="conservative")
    assert rep["ok"], rep["issues"]
    assert rep["updates"]["joint1"]["acceleration"]["new"] == pytest.approx(4.0)
    assert rep["updates"]["joint1"]["jerk"]["new"] == pytest.approx(20.0)

    # 实测比现值更宽松 → 保守模式保持现值不变
    base2 = measured_baseline(acc=8.0, jerk=90.0)
    rep2 = aml.plan_backfill(base2, limits, mode="conservative")
    assert rep2["ok"]
    assert rep2["updates"]["joint1"]["acceleration"]["new"] == pytest.approx(5.2359)
    assert rep2["updates"]["joint1"]["jerk"]["new"] == pytest.approx(30.0)
    assert "joint1.acceleration" not in rep2["changed"]


def test_measured_mode_uses_measured_value_and_warns_on_loosening(tmp_workspace):
    """measured 模式：直接采用实测值；放宽时给出 warn（需评审）。"""
    limits = seven_limits()
    rep = aml.plan_backfill(measured_baseline(acc=8.0, jerk=90.0), limits,
                            mode="measured")
    assert rep["ok"], rep["issues"]
    assert rep["updates"]["joint1"]["acceleration"]["new"] == pytest.approx(8.0)
    # 放宽属于 warn（不阻塞，但要评审）
    assert "limit_loosened" in codes(rep, "warn")


def test_dry_run_does_not_write(tmp_workspace, capsys):
    """--dry-run：打印 diff，文件字节不变，退出码 0。"""
    write_baseline(tmp_workspace, measured_baseline())
    before = open(limits_path(tmp_workspace), "rb").read()
    rc = aml.main(["--ws", tmp_workspace, "--dry-run"])
    after = open(limits_path(tmp_workspace), "rb").read()
    assert rc == 0
    assert before == after
    out = capsys.readouterr().out
    assert "逐关节差异" in out
    assert "joint1" in out


def test_write_keeps_yaml_valid_comments_and_provenance(tmp_workspace):
    """写盘后：仍是合法 YAML、7 个关节键齐全、原注释未丢失、来源与日期已追加。"""
    write_baseline(tmp_workspace, measured_baseline(acc=4.0, jerk=20.0))
    original_text, _ = read_limits(tmp_workspace)
    rc = aml.main(["--ws", tmp_workspace, "--date", "2026-11-13"])
    assert rc == 0
    text, data = read_limits(tmp_workspace)
    # 合法 YAML + 7 个关节齐全
    group = data["rm75_arm"]
    assert set(group.keys()) == set(JOINTS)
    assert group["joint1"]["acceleration"] == pytest.approx(4.0)
    assert group["joint1"]["jerk"] == pytest.approx(20.0)
    # 原注释逐字保留（文件头 + 每个关节行尾）
    assert "# 自建关节限值（唯一真值）" in text
    assert "±178°, 180°/s" in text
    assert original_text.splitlines()[0] == text.splitlines()[0]
    # 来源与采集日期追加在文件尾
    assert "实测限值回填记录" in text
    assert "回填日期: 2026-11-13" in text
    assert "source=真机阶梯实验" in text
    assert "samples=5" in text


def test_provenance_block_is_replaced_not_appended(tmp_workspace):
    """反复回填时记录块**替换**旧块（不无限追加），文件保持有界。"""
    write_baseline(tmp_workspace, measured_baseline(acc=4.0, jerk=20.0))
    assert aml.main(["--ws", tmp_workspace, "--date", "2026-11-13"]) == 0
    first, _ = read_limits(tmp_workspace)
    # 第二次回填：实测值更小 → 仍然只应有一个记录块
    write_baseline(tmp_workspace, measured_baseline(acc=3.0, jerk=15.0))
    assert aml.main(["--ws", tmp_workspace, "--date", "2026-11-14"]) == 0
    second, data = read_limits(tmp_workspace)
    assert second.count("实测限值回填记录") == 1
    assert "回填日期: 2026-11-14" in second
    assert "回填日期: 2026-11-13" not in second
    assert data["rm75_arm"]["joint1"]["acceleration"] == pytest.approx(3.0)
    assert set(data["rm75_arm"].keys()) == set(JOINTS)
    assert len(second.splitlines()) <= len(first.splitlines()) + 1


def test_write_refreshes_build_model_copy(tmp_workspace):
    """机器本地的 build/model 副本也刷新（rmctl 实际读这一份）。"""
    write_baseline(tmp_workspace, measured_baseline(acc=3.0, jerk=10.0))
    build_dir = os.path.join(tmp_workspace, "build", "model")
    os.makedirs(build_dir, exist_ok=True)
    build_copy = os.path.join(build_dir, "joint_limits.yaml")
    shutil.copyfile(limits_path(tmp_workspace), build_copy)
    rc = aml.main(["--ws", tmp_workspace])
    assert rc == 0
    with open(build_copy, "r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    assert data["rm75_arm"]["joint3"]["acceleration"] == pytest.approx(3.0)


def test_diff_summary_lists_per_joint(tmp_workspace):
    """逐关节差异摘要包含关节名与新旧值。"""
    write_baseline(tmp_workspace, measured_baseline())
    rep = aml.plan_backfill(measured_baseline(), read_limits(tmp_workspace)[1])
    diff = aml.format_diff(rep)
    for j in JOINTS:
        assert j in diff
    assert "→" in diff


# ------------------------------------------------------------------ 负例
def test_pending_baseline_is_refused(tmp_workspace, capsys):
    """未实测（status=pending）一律拒绝回填。"""
    write_baseline(tmp_workspace, measured_baseline(status="pending"))
    rc = aml.main(["--ws", tmp_workspace, "--dry-run"])
    assert rc == 3
    assert "baseline_not_measured" in capsys.readouterr().out


def test_blocked_baseline_is_refused(tmp_workspace):
    rep = aml.plan_backfill(measured_baseline(status="blocked"), seven_limits())
    assert not rep["ok"]
    assert "baseline_not_measured" in codes(rep)


@pytest.mark.parametrize("mutate,expect_code", [
    (lambda b: b["joints"][0]["acceleration"].__setitem__("samples", 1), "insufficient_samples"),
    (lambda b: b["joints"][0]["jerk"].__setitem__("samples", None), "insufficient_samples"),
    (lambda b: b["joints"][1]["acceleration"].__setitem__("source", None), "missing_source"),
    (lambda b: b["joints"][1]["jerk"].__setitem__("source", "  "), "missing_source"),
    (lambda b: b["joints"][2]["acceleration"].__setitem__("method", None), "missing_source"),
    (lambda b: b["joints"][2]["jerk"].__setitem__("value", None), "missing_measurement"),
    (lambda b: b["joints"][3]["acceleration"].__setitem__("unit", "rad/s"), "unit_mismatch"),
    (lambda b: b["joints"][3]["jerk"].__setitem__("value", -1.0), "non_positive_value"),
    (lambda b: b["joints"][4]["acceleration"].__setitem__("value", 0.0), "non_positive_value"),
    (lambda b: b["joints"][4]["jerk"].__setitem__("value", 1e9), "implausible_value"),
    (lambda b: b["joints"][5]["acceleration"].__setitem__("value", 500.0), "implausible_value"),
])
def test_broken_evidence_is_refused(tmp_workspace, mutate, expect_code):
    """单次极值污染 / 缺来源 / 缺样本数 / 单位错 / 负值 —— 任一必须拒绝回填。"""
    base = measured_baseline()
    mutate(base)
    write_baseline(tmp_workspace, base)
    rc = aml.main(["--ws", tmp_workspace])
    assert rc == 3
    # 拒绝原因必须明确指向该条证据缺失
    rep = aml.plan_backfill(base, read_limits(tmp_workspace)[1])
    assert expect_code in codes(rep)
    # 拒绝时不得写坏限值文件
    _, data = read_limits(tmp_workspace)
    assert data["rm75_arm"]["joint1"]["acceleration"] == pytest.approx(5.2359)


def test_missing_joint_in_limits_is_refused(tmp_workspace):
    """限值文件里没有该关节 → 拒绝回填（不静默跳过）。"""
    limits = {"rm75_arm": {"joint1": {"acceleration": 5.2359, "jerk": 30.0}}}
    rep = aml.plan_backfill(measured_baseline(), limits)  # 只有 joint1 → 其余 6 个关节缺失
    assert not rep["ok"]
    assert "joint_limits_missing_joint" in codes(rep)


def test_missing_baseline_file_exits_2(tmp_workspace, capsys):
    """基线文件缺失 → 退出码 2（输入非法），不是拒绝回填的 3。"""
    os.remove(os.path.join(tmp_workspace, "workcell", "baselines", "measured_limits.yaml"))
    rc = aml.main(["--ws", tmp_workspace])
    assert rc == 2
    assert "不存在" in capsys.readouterr().out


def test_missing_joint_entry_is_refused(tmp_workspace):
    base = measured_baseline()
    base["joints"] = base["joints"][:6]
    write_baseline(tmp_workspace, base)
    rc = aml.main(["--ws", tmp_workspace])
    assert rc == 3


# ------------------------------------------------------------------ 回归口径
class _FakeModel(object):
    """最小模型替身：只提供 required_scale 需要的 joint_names / limit()。"""

    def __init__(self, limits):
        self.joint_names = list(JOINTS)
        self._limits = limits

    def limit(self, joint):
        return self._limits.get(joint, {})


def _demo_traj(duration=2.0, dt=0.005, amp=0.2, freq=0.5):
    """一条平滑 demo 轨迹（正弦），用于回归口径。"""
    from rmctl import retime
    t = np.arange(0.0, duration + dt, dt)
    q = np.stack([amp * np.sin(2 * np.pi * freq * t) for _ in JOINTS], axis=1)
    vel, acc, jerk = retime.derivatives(list(t), q)
    return t, q, vel, acc, jerk


def test_regression_tighter_limits_trigger_rescale_and_looser_pass(tmp_workspace):
    """回填后回归口径：新限值更严 → 校验拦下并由保路径降速兜底；更松 → 直接通过。"""
    from rmctl import retime
    t, q, vel, acc, jerk = _demo_traj()
    base_limits = {"acceleration": 5.2359, "jerk": 30.0, "velocity": 3.1416}

    def limits_of(acc, jerk):
        return {j: dict(base_limits, acceleration=acc, jerk=jerk) for j in JOINTS}

    # 现值：acc/jerk 都够宽 → 直接通过
    s_cur = retime.required_scale(_FakeModel(limits_of(5.2359, 30.0)), t, q)
    assert s_cur == pytest.approx(1.0)

    # 更严（如实测回填出更小的 acc）→ 必须拦下并给出降速系数 > 1
    s_tight = retime.required_scale(_FakeModel(limits_of(1.0, 30.0)), t, q)
    assert s_tight > 1.001
    acc_peak = float(np.abs(acc).max())
    assert acc_peak > 1.0  # 轨迹本身确实越过了新限值 → 校验会报"加速度超限"

    # 更松 → required_scale 仍为 1（四道校验直接通过）
    s_loose = retime.required_scale(_FakeModel(limits_of(100.0, 1000.0)), t, q)
    assert s_loose == pytest.approx(1.0)
