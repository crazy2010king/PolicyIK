"""安全用例跑批测试（S04/T6）——五类故障各自落到 L1–L4 并带齐证据。

全部用例跑在 **ManualClock + FakeTransport** 上（逐 tick 可重放，断言确定）。

覆盖

* 六条用例的**等级**与**故障类**逐条命中（follow_error_light → L1、
  follow_error_heavy → L2、limit_exceeded → L2、collision → L3、
  protection_stop → L4、stream_lost → L4）；
* **动作序列**正确（L1=``set_speed_scale`` / L2=``pause`` /
  L3=``slow_stop`` / L4=``stop``），且**单一动作权威**（不重复、不叠加）；
* **未知错误码 → L3 保守**（码表外的码不假装「无故障」）；
* **报告字段齐**（fault_kind / level / threshold / observed /
  margin_to_limit / next_action / 动作序列 / SDK 调用序列 / 恢复判定 /
  verdict）；
* **负例**：注入参数非法在**解析阶段**被拒（不产出空报告）。

**诚实声明**：本文件断言的是**离线代理结论**（fake 模式），不代表真机结论；
碰撞码是 ``workcell/safety/error_codes.yaml`` 里的**工程声明映射码**
（厂商 ``err[]`` 码表本轮未取得，待真机窗口回填）。
"""
from __future__ import annotations

import json
import os

import pytest

from rm_executor import contract
from rm_executor import injection as inj_mod
from rm_executor import safety_batch as batch_mod
from rm_executor import sample as sample_mod

#: 期望的六条用例（与 safety_batch.CASES 一一对应）
EXPECTED = [
    ("follow_error_light", "L1", "follow_error", "set_speed_scale"),
    ("follow_error_heavy", "L2", "follow_error", "pause"),
    ("limit_exceeded", "L2", "limit_exceeded", "pause"),
    ("collision", "L3", "collision", "slow_stop"),
    ("protection_stop", "L4", "protection_stop", "stop"),
    ("stream_lost", "L4", "stream_lost", "stop"),
]


@pytest.fixture(scope="module")
def trajectory(tmp_path_factory):
    """确定性合成轨迹（跑批输入）。"""
    path = tmp_path_factory.mktemp("traj") / "traj.json"
    return sample_mod.write_sample_trajectory(str(path))


@pytest.fixture(scope="module")
def batch_results(trajectory):
    """跑一遍全部六条用例（一次跑批，逐条返回报告）。"""
    results = {}
    for spec in batch_mod.CASES:
        results[spec["case"]] = batch_mod.run_case(spec, trajectory_path=trajectory)
    return results


# ------------------------------------------------------------------ 逐条断言
@pytest.mark.parametrize("case,level,fault_class,action", EXPECTED)
def test_case_hits_expected_level_and_fault_class(batch_results, case, level,
                                                   fault_class, action):
    """六条用例的等级与故障类逐条命中。"""
    rep = batch_results[case]
    assert rep["verdict"] == "pass", rep["verdict_reason"]
    assert rep["level"] == level, rep
    assert rep["fault_kind"] == fault_class, rep
    assert rep["verdict_reason"]


@pytest.mark.parametrize("case,level,fault_class,action", EXPECTED)
def test_case_action_sequence_is_correct(batch_results, case, level,
                                          fault_class, action):
    """动作序列正确：L1=set_speed_scale / L2=pause / L3=slow_stop / L4=stop。"""
    rep = batch_results[case]
    names = [a["action"] for a in rep["action_sequence"]]
    assert action in names, rep["action_sequence"]
    # 单一动作权威：L4 急停**只发一次**，且不与其它等级动作叠加
    if level == "L4":
        assert names.count("stop") == 1, rep["action_sequence"]
    assert "continue_" not in names, rep["action_sequence"]


@pytest.mark.parametrize("case,level,fault_class,action", EXPECTED)
def test_case_report_has_all_evidence_fields(batch_results, case, level,
                                             fault_class, action):
    """报告字段齐：等级 / 故障类 / 严重度 / 退出码 / 动作序列 / SDK 调用序列 /
    证据字段 / 恢复判定 / verdict。"""
    rep = batch_results[case]
    for key in ("fault_kind", "level", "threshold", "observed",
                "margin_to_limit", "next_action", "severity", "exit_code",
                "action_sequence", "sdk_call_sequence", "recovery",
                "verdict"):
        assert key in rep, key
    assert rep["severity"] in ("RECOVERABLE", "NEEDS_HUMAN", "FATAL"), rep
    assert rep["exit_code"] in (3, 4), rep
    assert rep["next_action"], rep
    assert rep["recovery"] in ("auto", "manual", "manual_reset"), rep
    # SDK 调用序列：安全动作确实落到传输端口上
    assert action in rep["sdk_call_sequence"], rep["sdk_call_sequence"]


def test_l1_case_auto_recovery_is_auto_and_not_manual(batch_results):
    """L1 是唯一可自动恢复的等级（恢复方式 = auto），L2+ 一律人工。"""
    assert batch_results["follow_error_light"]["recovery"] == "auto"
    for case in ("follow_error_heavy", "limit_exceeded", "collision"):
        assert batch_results[case]["recovery"] == "manual", case
    for case in ("protection_stop", "stream_lost"):
        assert batch_results[case]["recovery"] == "manual_reset", case


# ------------------------------------------------------------- 未知错误码
def test_unknown_error_code_falls_back_to_l3(trajectory):
    """码表外的错误码 → **L3 保守档**（不假装「无故障」，也不升级到 L4）。"""
    spec = {
        "case": "collision_unknown_code",
        "injection": "collision:99999",
        "expected_level": "L3",
        "expected_fault_class": "unknown_error_code",
        "expected_action": "slow_stop",
    }
    rep = batch_mod.run_case(spec, trajectory_path=trajectory)
    assert rep["verdict"] == "pass", rep["verdict_reason"]
    assert rep["level"] == "L3", rep
    assert rep["fault_kind"] == "unknown_error_code", rep
    assert "slow_stop" in [a["action"] for a in rep["action_sequence"]], rep


# ------------------------------------------------------------- 解析阶段负例
@pytest.mark.parametrize("spec", [
    "follow_error_light:1.0",     # 未超过告警阈值 → 不会判 L1
    "follow_error_light:6.0",     # 超过停机阈值 → 属于 follow_error 的场景
    "follow_error_light:abc",     # 非数值
    "limit_exceeded:0",           # 越界量必须 > 0
    "limit_exceeded:-1",          # 越界量必须 > 0
    "collision:-1",               # 错误码必须非负
    "collision:abc",              # 错误码必须是非负整数
    "protection_stop:1",          # 不接受参数
])
def test_invalid_injection_rejected_at_parse_stage(spec):
    """非法注入规格在**解析阶段**被拒（带可粘贴的下一步）。"""
    with pytest.raises(ValueError) as exc:
        inj_mod.parse_injection(spec)
    assert str(exc.value)


def test_invalid_injection_produces_no_report(tmp_path):
    """非法注入 → **不产出空报告**（不落一份「跑了但没动作」的报告）。"""
    spec = dict(batch_mod.CASES[0])
    spec["injection"] = "follow_error_light:1.0"
    out_dir = tmp_path / "reports"
    rep = batch_mod.run_case(spec, out_dir=str(out_dir),
                              trajectory_path=str(tmp_path / "missing.json"))
    assert rep["verdict"] == "invalid_spec", rep
    assert rep["verdict_reason"]
    assert not (out_dir / ("s04_%s.json" % spec["case"])).exists()


# ------------------------------------------------------------------ 跑批落盘
def test_cli_live_mode_rejects_new_injection_kinds(tmp_path):
    """负例：真机路径拒绝**任何**注入（含 S04 新增四类）。"""
    from rm_executor import cli as cli_mod

    path = sample_mod.write_sample_trajectory(str(tmp_path / "traj.json"))
    for spec in ("collision:90001", "protection_stop", "limit_exceeded:2.0",
                 "follow_error_light:3.0"):
        rep = cli_mod.execute(mode="live", trajectory_path=path,
                               injections=[spec])
        assert rep["status"] == "blocked", spec
        assert "真机路径不接受故障注入" in rep["reason"], spec
        assert not rep["metrics"]["ticks"], spec


def test_run_all_writes_case_reports_and_summary(tmp_path):
    """``run_all`` 落 6 份用例报告 + 1 份汇总报告，退出码口径正确。"""
    out_dir = tmp_path / "reports"
    summary = batch_mod.run_all(out_dir=str(out_dir))
    assert summary["total"] == len(batch_mod.CASES)
    assert summary["passed"] == len(batch_mod.CASES), summary
    assert summary["failed"] == 0, summary
    assert summary["verdict"] == "pass", summary
    for spec in batch_mod.CASES:
        path = out_dir / ("s04_%s.json" % spec["case"])
        assert path.exists(), path
        payload = json.loads(path.read_text(encoding="utf-8"))
        assert payload["level"] == spec["expected_level"], payload
        assert payload["verdict"] == "pass", payload
    summary_path = out_dir / batch_mod.SUMMARY_FILENAME
    assert summary_path.exists(), summary_path
    payload = json.loads(summary_path.read_text(encoding="utf-8"))
    assert payload["passed"] == len(batch_mod.CASES), payload


def test_cli_main_exit_code_zero(tmp_path):
    """``python3 -m rm_executor.safety_batch --all`` → 退出码 0。"""
    out_dir = tmp_path / "reports"
    rc = batch_mod.main(["--all", "--out-dir", str(out_dir)])
    assert rc == contract.EXIT_CODES[contract.STATUS_OK]
    assert (out_dir / batch_mod.SUMMARY_FILENAME).exists()


def test_cli_main_rejects_unknown_case(tmp_path):
    """未知用例名 → 退出码 3（blocked）+ 明确提示。"""
    rc = batch_mod.main(["--case", "no_such_case", "--out-dir",
                         str(tmp_path / "reports")])
    assert rc == contract.EXIT_CODES[contract.STATUS_BLOCKED]
