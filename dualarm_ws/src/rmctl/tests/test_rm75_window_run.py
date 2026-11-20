"""``scripts/rm75_window_run.py``（真机窗口单一入口编排器）的离线用例。

为什么测：这是 S10 的**唯一入口**——它把「门禁 → 长跑 → 断流 → 阶梯 → 结案工件」
串起来。改坏它的后果比改坏单个工具严重得多：

* 门禁不过却发了运动指令 → 现场安全事故；
* 长跑 / 断流没跑却写成「已达标」 → 用离线/假结果冒充真机结论；
* 结案工件不自洽 → 下游（S11 基线回填 / S12 未闭合声明）读到两套结论。

所以这里钉死：

* **门禁 blocked → 长跑/断流/阶梯 runner 一次都不被调用**（零运动指令）；
* 门禁 ok 且全部达标 → 工件 ``status="ok"``，逐条判据 pass；
* 门禁 ok 但长跑未达标（如 wall_s=5.53）→ 工件**不得**判 ok；
* 断流事件缺失 → 判未达标，且退出码语义与 ``rm_executor.contract`` 一致；
* 结案工件必带 ``honesty`` 声明，未执行项必写明原因；
* ``--validate`` 能区分自洽 / 不自洽（0 / 3）。

全部用**假 runner**注入 → 不连真机、不发任何运动指令。
"""
from __future__ import annotations

import importlib.util
import json
import os
import sys

import pytest

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(TESTS_DIR, "..", "..", "..", ".."))
SCRIPT_PATH = os.path.join(REPO_ROOT, "scripts", "rm75_window_run.py")

for _rel in ("dualarm_ws/src/rm_executor", "dualarm_ws/src/rm_hw_bridge",
             "dualarm_ws/src/rm_safety"):
    _p = os.path.join(REPO_ROOT, _rel)
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

from rm_executor import contract  # noqa: E402


def _load_script():
    spec = importlib.util.spec_from_file_location("rm75_window_run_under_test",
                                                  SCRIPT_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def script():
    return _load_script()


# ------------------------------------------------------------------ 假实现
def _gate_ok(**kwargs):
    return {
        "schema": "rmctl.rm75_window_gate/1",
        "status": "ok",
        "reason": "全部门禁通过",
        "checks": [{"name": "ip_reachable", "ok": True, "detail": "ping 通",
                    "evidence": {}}],
        "attempted_actions": [{"action": "icmp_ping", "command": "ping",
                                "result": "通", "ok": True}],
        "next_action": ["在示教器上重新使能 7 轴（joint_en_flag 全 1）"],
        "honesty": "只读门禁结果",
    }


def _gate_blocked(**kwargs):
    return {
        "schema": "rmctl.rm75_window_gate/1",
        "status": "blocked",
        "reason": "门禁未通过：ip_reachable（100% 丢包）",
        "checks": [{"name": "ip_reachable", "ok": False, "detail": "100% 丢包",
                    "evidence": {}}],
        "attempted_actions": [{"action": "icmp_ping", "command": "ping",
                                "result": "100% 丢包", "ok": False}],
        "next_action": ["ping -c 3 192.168.1.18",
                        "在示教器上重新使能 7 轴（joint_en_flag 全 1）"],
        "honesty": "只读门禁结果",
    }


def _long_run_ok(**kwargs):
    return {
        "schema": "rmctl.rm75_long_run/1",
        "mode": "hardware",
        "status": "ok",
        "reason": "",
        "wall_s": 601.4,
        "protection_stop": False,
        "stream_lost": False,
        "planned_points": 60001,
        "events": [],
        "metrics": {"tick_jitter_p99_ms": 0.21, "arrival_err_deg": 0.003,
                    "follow_err_rms_deg": 0.05, "follow_err_max_deg": 0.11},
    }


def _long_run_short(**kwargs):
    """2026-11-19 真机短跑：量化门限达标但时长未达标（5.53 s）。"""
    data = _long_run_ok()
    data["wall_s"] = 5.53
    data["planned_points"] = 501
    return data


def _stream_loss_ok(**kwargs):
    return {
        "schema": "rmctl.rm75_stream_loss_drill/1",
        "mode": "hardware",
        "status": "error",
        "reason": "反馈断流（安全层已 L4 急停）",
        "stream_lost": True,
        "events": ["feedback_stream_lost", "safety_l4_emergency_stop"],
        "l4_action": "rm_set_arm_stop",
        "stop_latency_ms": 180.0,
        "cut_result": {"status": "ok", "cut_at_perf": 100.0},
    }


def _stream_loss_no_event(**kwargs):
    data = _stream_loss_ok()
    data["events"] = ["safety_l1_recovered"]
    data["stream_lost"] = False
    data["stop_latency_ms"] = None
    return data


class FakeRunner(object):
    """假 runner：记录调用次数与参数，返回固定的证据。"""

    def __init__(self, evidence=None, exit_code=0):
        self.calls = []
        self.evidence = evidence
        self.exit_code = exit_code

    def __call__(self, argv, out_path):
        self.calls.append({"argv": list(argv), "out_path": out_path})
        return {"exit_code": self.exit_code, "evidence": self.evidence,
                "path": out_path}


def _run(script, gate=None, long_run=None, stream_loss=None, ladder=None,
         merge=None, **kwargs):
    runners = {
        "gate": gate or _gate_ok,
        "long_run": long_run or FakeRunner(_long_run_ok()),
        "stream_loss": stream_loss or FakeRunner(_stream_loss_ok()),
        "ladder": ladder or FakeRunner({"schema": "rmctl.rm75_validation_evidence/1",
                                        "status": "pass",
                                        "summary": {"total": 10, "passed": 10}}),
        "merge": merge or FakeRunner({"status": "ok"}),
    }
    return script.run_window(gate=runners["gate"], long_run=runners["long_run"],
                             stream_loss=runners["stream_loss"],
                             ladder=runners["ladder"], merge=runners["merge"],
                             **kwargs)


# ------------------------------------------------------- 门禁不过 → 零运动
def test_blocked_gate_never_calls_any_runner(script):
    long_run = FakeRunner(_long_run_ok())
    stream_loss = FakeRunner(_stream_loss_ok())
    ladder = FakeRunner({"status": "pass"})
    merge = FakeRunner({"status": "ok"})
    artifact = _run(script, gate=_gate_blocked, long_run=long_run,
                    stream_loss=stream_loss, ladder=ladder, merge=merge)
    assert long_run.calls == [], "门禁不过时长跑 runner 绝不能被调用"
    assert stream_loss.calls == [], "门禁不过时断流 runner 绝不能被调用"
    assert ladder.calls == [], "门禁不过时阶梯 runner 绝不能被调用"
    assert merge.calls == [], "门禁不过时入库 runner 绝不能被调用"
    assert artifact["status"] == contract.STATUS_BLOCKED
    assert artifact["exit_code"] == contract.EXIT_CODES[contract.STATUS_BLOCKED] == 3


def test_blocked_artifact_has_reason_next_action_and_honesty(script):
    artifact = _run(script, gate=_gate_blocked)
    assert artifact["reason"]
    assert artifact["next_action"]
    assert artifact["honesty"]
    assert "离线" in artifact["honesty"]


def test_blocked_artifact_marks_not_executed_with_reason(script):
    artifact = _run(script, gate=_gate_blocked)
    assert artifact["long_run"] is None
    assert artifact["stream_loss"] is None
    assert artifact["ladder"] is None
    for name in ("long_run", "stream_loss", "ladder"):
        assert artifact["not_executed"][name].strip(), \
            "%s 未执行必须写明原因" % name


def test_blocked_artifact_contains_no_measured_numbers(script):
    """blocked 分支下工件里不得出现任何伪造的长跑/断流实测数字。"""
    artifact = _run(script, gate=_gate_blocked)
    blob = json.dumps(artifact, ensure_ascii=False)
    assert "601.4" not in blob
    assert "180.0" not in blob
    assert artifact["long_run"] is None
    assert artifact["stream_loss"] is None


def test_blocked_mode_is_blocked_not_hardware(script):
    artifact = _run(script, gate=_gate_blocked)
    assert artifact["mode"] == "blocked"
    assert artifact["mode"] != "hardware"


def test_gate_checks_are_carried_into_outcome(script):
    artifact = _run(script, gate=_gate_blocked)
    assert artifact["gate"]["status"] == "blocked"
    assert artifact["gate"]["checks"]


# ------------------------------------------------------------- 全达标 → ok
def test_all_pass_gives_status_ok_and_every_criterion_passes(script):
    artifact = _run(script)
    assert artifact["status"] == contract.STATUS_OK
    assert artifact["exit_code"] == 0
    for name in ("long_run", "stream_loss"):
        section = artifact[name]
        assert section["passed"] is True
        assert section["criteria"]
        for crit in section["criteria"]:
            assert crit["ok"] is True, "%s 判据 %s 未通过" % (name, crit["name"])


def test_all_pass_records_measured_values_and_evidence_paths(script):
    artifact = _run(script)
    long_run = artifact["long_run"]
    assert long_run["measured"]["wall_s"] == 601.4
    assert long_run["evidence_path"]
    assert artifact["stream_loss"]["measured"]["l4_action"] == "rm_set_arm_stop"
    assert artifact["stream_loss"]["measured"]["stop_latency_ms"] == 180.0
    assert artifact["stream_loss"]["evidence_path"]


def test_all_pass_mode_is_hardware(script):
    artifact = _run(script)
    assert artifact["mode"] == "hardware"


def test_runners_are_called_with_expected_arguments(script):
    long_run = FakeRunner(_long_run_ok())
    stream_loss = FakeRunner(_stream_loss_ok())
    ladder = FakeRunner({"status": "pass"})
    _run(script, long_run=long_run, stream_loss=stream_loss, ladder=ladder)
    assert "--duration-s" in long_run.calls[0]["argv"]
    assert "600.0" in long_run.calls[0]["argv"]
    assert "--max-stage" in ladder.calls[0]["argv"]
    assert "V9" in ladder.calls[0]["argv"]
    assert "--allow-risky" in ladder.calls[0]["argv"]


def test_long_run_and_stream_loss_are_never_concurrent(script):
    """单连接纪律：长跑与断流顺序执行（断流不得早于长跑开始）。"""
    order = []

    def long_run(argv, out_path):
        order.append("long_run")
        return {"exit_code": 0, "evidence": _long_run_ok(), "path": out_path}

    def stream_loss(argv, out_path):
        order.append("stream_loss")
        return {"exit_code": 0, "evidence": _stream_loss_ok(), "path": out_path}

    _run(script, long_run=long_run, stream_loss=stream_loss)
    assert order == ["long_run", "stream_loss"]


# --------------------------------------------------------- 未达标 → 不得 ok
def test_short_long_run_is_not_ok(script):
    artifact = _run(script, long_run=FakeRunner(_long_run_short()))
    assert artifact["status"] != contract.STATUS_OK
    assert artifact["exit_code"] == contract.EXIT_CODES[contract.STATUS_ERROR] == 4
    assert artifact["long_run"]["passed"] is False
    assert "wall_s_at_least_600s" in artifact["long_run"]["failed_criteria"]
    assert artifact["honesty"]


def test_missing_stream_loss_event_is_not_ok(script):
    artifact = _run(script, stream_loss=FakeRunner(_stream_loss_no_event()))
    assert artifact["status"] != contract.STATUS_OK
    assert artifact["exit_code"] == contract.EXIT_CODES[contract.STATUS_ERROR]
    assert artifact["stream_loss"]["passed"] is False
    assert "event_feedback_stream_lost" in artifact["stream_loss"]["failed_criteria"]
    assert artifact["honesty"]


def test_long_run_criteria_missing_metrics_is_not_ok(script):
    """证据缺字段 → 无法判定 → 按未达标处理（绝不静默当成通过）。"""
    data = _long_run_ok()
    del data["metrics"]["tick_jitter_p99_ms"]
    artifact = _run(script, long_run=FakeRunner(data))
    assert artifact["long_run"]["passed"] is False
    assert "tick_jitter_p99_lt_2ms" in artifact["long_run"]["failed_criteria"]
    crit = [c for c in artifact["long_run"]["criteria"]
            if c["name"] == "tick_jitter_p99_lt_2ms"][0]
    assert crit["present"] is False


def test_fake_evidence_is_not_reported_as_hardware(script):
    """假证据（mode=fake）不得被报成 hardware。"""
    data = _long_run_ok()
    data["mode"] = "fake"
    artifact = _run(script, long_run=FakeRunner(data))
    assert artifact["mode"] != "hardware"
    assert artifact["mode"] in ("fake", "mixed")


# --------------------------------------------------------------- 判据表纪律
def test_criteria_tables_are_the_single_source(script):
    """判据表是脚本常量，测试直接引用（唯一来源纪律）。"""
    long_names = {c["name"] for c in script.LONG_RUN_CRITERIA}
    assert long_names == {
        "status_ok", "wall_s_at_least_600s", "no_protection_stop",
        "no_stream_lost", "tick_jitter_p99_lt_2ms", "arrival_err_lt_0p1deg",
        "follow_err_rms_le_0p3deg",
    }
    loss_names = {c["name"] for c in script.STREAM_LOSS_CRITERIA}
    assert loss_names == {
        "stream_lost_true", "event_feedback_stream_lost",
        "l4_action_rm_set_arm_stop", "stop_latency_le_stream_stop_ms",
    }


def test_criteria_thresholds_match_the_slice_contract(script):
    by_name = {c["name"]: c for c in script.LONG_RUN_CRITERIA}
    assert by_name["wall_s_at_least_600s"]["value"] == 600.0
    assert by_name["tick_jitter_p99_lt_2ms"]["value"] == 2.0
    assert by_name["arrival_err_lt_0p1deg"]["value"] == 0.1
    assert by_name["follow_err_rms_le_0p3deg"]["value"] == 0.3
    by_loss = {c["name"]: c for c in script.STREAM_LOSS_CRITERIA}
    assert by_loss["stop_latency_le_stream_stop_ms"]["value"] == \
        contract.STREAM_STOP_MS == 200
    assert by_loss["l4_action_rm_set_arm_stop"]["value"] == "rm_set_arm_stop"


def test_status_vocabulary_comes_from_contract(script):
    assert script.STATUS_OK == contract.STATUS_OK
    assert script.STATUS_BLOCKED == contract.STATUS_BLOCKED
    assert script.STATUS_ERROR == contract.STATUS_ERROR
    assert script.EXIT_CODES == contract.EXIT_CODES
    assert script.STREAM_STOP_MS == contract.STREAM_STOP_MS


def test_schema_is_stable(script):
    assert script.SCHEMA == "rmctl.rm75_s10_window_outcome/1"
    artifact = _run(script)
    assert artifact["schema"] == "rmctl.rm75_s10_window_outcome/1"


# --------------------------------------------------------------- 自校验
def test_validate_accepts_self_consistent_artifact(script):
    artifact = _run(script)
    ok, problems = script.validate_artifact(artifact)
    assert ok, problems


def test_validate_accepts_blocked_artifact(script):
    artifact = _run(script, gate=_gate_blocked)
    ok, problems = script.validate_artifact(artifact)
    assert ok, problems


def test_validate_rejects_artifact_without_honesty(script):
    artifact = _run(script)
    artifact["honesty"] = ""
    ok, problems = script.validate_artifact(artifact)
    assert not ok
    assert any("honesty" in p for p in problems)


def test_validate_rejects_blocked_without_next_action(script):
    artifact = _run(script, gate=_gate_blocked)
    artifact["next_action"] = []
    ok, problems = script.validate_artifact(artifact)
    assert not ok
    assert any("next_action" in p for p in problems)


def test_validate_rejects_not_executed_without_reason(script):
    artifact = _run(script, gate=_gate_blocked)
    artifact["not_executed"]["long_run"] = ""
    ok, problems = script.validate_artifact(artifact)
    assert not ok
    assert any("long_run" in p for p in problems)


def test_validate_rejects_missing_criteria(script):
    artifact = _run(script)
    artifact["long_run"]["criteria"] = []
    ok, problems = script.validate_artifact(artifact)
    assert not ok
    assert any("判据" in p for p in problems)


def test_validate_rejects_illegal_status(script):
    artifact = _run(script)
    artifact["status"] = "passed"
    ok, problems = script.validate_artifact(artifact)
    assert not ok
    assert any("status" in p for p in problems)


def test_validate_cli_distinguishes_consistent_and_broken(script, tmp_path):
    good = tmp_path / "good.json"
    good.write_text(json.dumps(_run(script), ensure_ascii=False), encoding="utf-8")
    assert script.main(["--validate", str(good)]) == 0
    broken = tmp_path / "broken.json"
    payload = _run(script)
    payload["status"] = "passed"
    broken.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")
    assert script.main(["--validate", str(broken)]) == \
        contract.EXIT_CODES[contract.STATUS_BLOCKED] == 3


def test_validate_cli_on_missing_file_returns_blocked(script, tmp_path):
    assert script.main(["--validate", str(tmp_path / "nope.json")]) == 3


# --------------------------------------------------------------- CLI 落盘
def test_cli_writes_artifact_and_returns_contract_exit_code(script, tmp_path):
    out = tmp_path / "nested" / "outcome.json"
    rc = script.main(["--out", str(out)],
                     gate=_gate_blocked, long_run=FakeRunner(_long_run_ok()),
                     stream_loss=FakeRunner(_stream_loss_ok()),
                     ladder=FakeRunner({"status": "pass"}),
                     merge=FakeRunner({"status": "ok"}))
    assert rc == contract.EXIT_CODES[contract.STATUS_BLOCKED] == 3
    assert out.exists()
    saved = json.loads(out.read_text(encoding="utf-8"))
    assert saved["status"] == "blocked"
    for field in ("schema", "mode", "status", "reason", "gate", "long_run",
                  "stream_loss", "ladder", "not_executed", "attempted_actions",
                  "next_action", "honesty"):
        assert field in saved, "工件缺字段 %s" % field


def test_cli_ok_path_returns_zero(script, tmp_path):
    out = tmp_path / "outcome.json"
    rc = script.main(["--out", str(out)], gate=_gate_ok,
                     long_run=FakeRunner(_long_run_ok()),
                     stream_loss=FakeRunner(_stream_loss_ok()),
                     ladder=FakeRunner({"status": "pass"}),
                     merge=FakeRunner({"status": "ok"}))
    assert rc == 0
    saved = json.loads(out.read_text(encoding="utf-8"))
    assert saved["status"] == "ok"
    ok, problems = script.validate_artifact(saved)
    assert ok, problems


# ------------------------------------------------------- 断流证据派生
def test_derive_l4_action_from_explicit_field(script):
    assert script.derive_l4_action({"l4_action": "rm_set_arm_stop"}) == \
        "rm_set_arm_stop"


def test_derive_l4_action_from_safety_event(script):
    action = script.derive_l4_action(
        {"events": ["feedback_stream_lost", "safety_l4_emergency_stop"]})
    assert action == "rm_set_arm_stop"


def test_derive_l4_action_none_when_no_l4_event(script):
    assert script.derive_l4_action({"events": ["feedback_stream_lag"]}) is None


def test_derive_stop_latency_from_cut_and_lost_event(script):
    evidence = {
        "cut_result": {"cut_at_perf": 100.0},
        "stream_lost_evidence": [{"type": "feedback_stream_lost", "t": 100.18}],
    }
    latency = script.derive_stop_latency_ms(evidence)
    assert latency is not None
    assert abs(latency - 180.0) < 1e-6


def test_derive_stop_latency_none_when_cut_time_missing(script):
    assert script.derive_stop_latency_ms({"events": ["feedback_stream_lost"]}) \
        is None


# --------------------------------------------------------------- 参考值
def test_reference_values_are_recorded_but_not_used_as_criteria(script):
    artifact = _run(script)
    ref = artifact["reference_values"]
    assert ref["wall_s"] == 5.53
    assert ref["tick_jitter_p99_ms"] == 0.201
    # 参照值不得变成判据：短跑（5.53 s）必须判未达标
    artifact = _run(script, long_run=FakeRunner(_long_run_short()))
    assert artifact["long_run"]["passed"] is False
