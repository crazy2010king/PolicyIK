"""体验基准 E1–E4 的验收用例（S09/T5）。

覆盖：

* 声明表（``workcell/bench/experience_e1_e4.yaml``）**唯一来源**且 E1–E4 齐全；
* 每个指标都带 ``mode``；**没有任何** ``mode=live`` 的数字（D012 分层纪律）；
* E3 的真 MTTR 允许 ``null`` 但**必须** pending；被填上 0 必须失败（负例）；
* E3 报错可诊断率 = 1.0 或明确 blocked；
* 目标值与设计 §6.2 逐条一致（T2FM ≤ 30 min、命令条数 ≤ 3、首次成功率
  ≥ 95%、MTTR ≤ 2 min、错误可诊断性 100%）。

所有用例只用 **git 跟踪的文件 + pytest 临时目录 + 注入的假 runner**，
不依赖 ``build/``、``.gsd/`` 等 gitignore 目录，也不真跑规划链路。
"""
import json
import os
import shutil

import pytest

from rmctl.regression import experience as exp
from rmctl.regression import experience_helpers as helpers
from rmctl.regression import orchestrator as orch


# ---------------------------------------------------------------------------
# 夹具
# ---------------------------------------------------------------------------
@pytest.fixture(scope="module")
def scenarios(repo_root):
    return exp.load_scenarios(repo_root)


def _fake_result(stdout="", exit_code=0, duration=0.5):
    return exp.CommandResult(cmd=["fake"], exit_code=exit_code,
                             stdout=stdout, stderr="", duration_s=duration)


_DOCTOR_OK = """
体检结果: 17/17 通过

  [PASS] workspace                  工作区: /tmp/ws
  [PASS] model:urdf                 /tmp/ws/build/model/rm75_clean.urdf
  [PASS] model:load                 组=rm75_arm 关节=7 连杆=8
  [INFO] hardware:connected         未检测到真机连接（原型阶段为模拟执行）
        → 下一步: rmctl connect --left <ip> --right <ip>
  [PASS] baselines:present          四类基线 4/4 齐备
"""

_DOCTOR_BAD = """
体检结果: 15/17 通过

  [PASS] workspace                  工作区: /tmp/ws
  [NEEDS_HUMAN] model:load          厂商规划内核加载模型失败
        → 下一步: 检查 URDF/SRDF 与插件配置
  [INFO] hardware:connected         未检测到真机连接
  [RECOVERABLE] baselines:present   四类基线 2/4 齐备
"""


def _write_trace(path, events):
    with open(path, "w", encoding="utf-8") as fh:
        for event in events:
            fh.write(json.dumps(event, ensure_ascii=False) + "\n")


def _write_raw_trace(path, lines):
    with open(path, "w", encoding="utf-8") as fh:
        for line in lines:
            fh.write(line + "\n")


def _event(ts, severity="INFO", status="ok", next_action="", reason=""):
    return {"schema_version": 1, "ts": ts, "phase": "judge", "segment": "grasp",
            "status": status, "severity": severity, "reason": reason,
            "next_action": next_action, "evidence": []}


# ---------------------------------------------------------------------------
# 1. 声明表（唯一来源）+ 目标值与设计 §6.2 逐条一致
# ---------------------------------------------------------------------------
def test_scenarios_cover_e1_to_e4(scenarios):
    assert sorted(scenarios["scenarios"]) == ["E1", "E2", "E3", "E4"]


def test_targets_match_design_62(scenarios):
    """目标值逐条对齐设计 §6.2（不得在代码里另写一份目标）。"""
    e1 = scenarios["scenarios"]["E1"]["metrics"]
    e2 = scenarios["scenarios"]["E2"]["metrics"]
    e3 = scenarios["scenarios"]["E3"]["metrics"]
    e4 = scenarios["scenarios"]["E4"]["metrics"]

    # T2FM ≤ 30 min
    assert e1["t2fm_s"]["target"] == 1800.0
    assert e1["t2fm_s"]["target_op"] == "<="
    # 接入新机型：代码改动 = 0 行
    assert e1["code_lines_changed"]["target"] == 0
    assert e1["code_lines_changed"]["target_op"] == "<="
    # 首次成功率 ≥ 95%
    assert e2["first_success_rate"]["target"] == 0.95
    assert e2["first_success_rate"]["target_op"] == ">="
    # MTTR ≤ 2 min（人工字段）
    assert e3["mttr_human_s"]["target"] == 120.0
    assert e3["mttr_human_s"]["target_op"] == "<="
    # 错误可诊断性 100%
    assert e3["diagnosability_rate"]["target"] == 1.0
    assert e3["diagnosability_rate"]["target_op"] == ">="
    # 日常操作 ≤ 3 条命令
    assert e4["command_count"]["target"] == 3
    assert e4["command_count"]["target_op"] == "<="


def test_every_declared_metric_has_mode_and_unit(scenarios):
    for sid, scenario in scenarios["scenarios"].items():
        for mid in scenario["metric_order"]:
            metric = scenario["metrics"][mid]
            assert metric["mode"] in exp.METRIC_MODES, (sid, mid)
            assert metric["unit"], (sid, mid)


def test_no_live_metric_in_declaration(scenarios):
    """本声明表**不出现** mode=live 的数字（真机口径本切片未实证）。"""
    for sid, scenario in scenarios["scenarios"].items():
        for mid in scenario["metric_order"]:
            assert scenario["metrics"][mid]["mode"] != "live", (sid, mid)


def test_missing_declaration_is_rejected(repo_root, tmp_path):
    """声明文件非法 → ValueError（绝不静默当 pass）。"""
    bad = tmp_path / "bad.yaml"
    bad.write_text("version: 1\nscenarios: []\n", encoding="utf-8")
    with pytest.raises(ValueError):
        exp.load_scenarios(repo_root, path=str(bad))


def test_declaration_rejects_target_without_op(repo_root, tmp_path):
    """target 有值但 target_op 缺失 → ValueError（不编造比较口径）。"""
    bad = tmp_path / "bad.yaml"
    bad.write_text(
        "version: 1\nscenarios:\n"
        "  - id: E1\n    metrics:\n"
        "      - id: x\n        unit: s\n        target: 1.0\n        mode: offline\n"
        "  - id: E2\n    metrics:\n"
        "      - id: x\n        unit: s\n        target: null\n        mode: offline\n"
        "  - id: E3\n    metrics:\n"
        "      - id: x\n        unit: s\n        target: null\n        mode: offline\n"
        "  - id: E4\n    metrics:\n"
        "      - id: x\n        unit: s\n        target: null\n        mode: offline\n",
        encoding="utf-8")
    with pytest.raises(ValueError):
        exp.load_scenarios(repo_root, path=str(bad))


# ---------------------------------------------------------------------------
# 2. 指标纪律（mode / pending / blocked / live）
# ---------------------------------------------------------------------------
def test_validate_metrics_rejects_pending_with_value(scenarios):
    """负例：人工 MTTR 被填上 0 → 校验必须失败（Q7）。"""
    metric = exp.build_metric(scenarios["scenarios"]["E3"]["metrics"]
                              ["mttr_human_s"], "E3", 0, status="pending")
    with pytest.raises(ValueError):
        exp.validate_metrics([metric])


def test_validate_metrics_rejects_blocked_with_value(scenarios):
    """负例：blocked 指标用 0 冒充真实值 → 校验必须失败。"""
    metric = exp.build_metric(scenarios["scenarios"]["E1"]["metrics"]
                              ["t2fm_s"], "E1", 0, status="blocked")
    with pytest.raises(ValueError):
        exp.validate_metrics([metric])


def test_validate_metrics_rejects_live_without_backing(scenarios):
    """负例：mode=live 但没有真机背书 → 校验必须失败（D012）。"""
    metric = exp.build_metric(scenarios["scenarios"]["E1"]["metrics"]
                              ["t2fm_s"], "E1", 12.0, status="measured")
    metric["mode"] = "live"
    with pytest.raises(ValueError):
        exp.validate_metrics([metric])


def test_validate_metrics_requires_every_declared_metric(scenarios):
    """负例：声明里的指标漏测 → 校验必须失败。"""
    metric = exp.build_metric(scenarios["scenarios"]["E1"]["metrics"]
                              ["t2fm_s"], "E1", 1.0, status="measured")
    with pytest.raises(ValueError):
        exp.validate_metrics([metric], scenarios)


def test_validate_metrics_accepts_honest_pending(scenarios):
    """人工字段留 null + pending → 合法。"""
    metric = exp.build_metric(scenarios["scenarios"]["E3"]["metrics"]
                              ["mttr_human_s"], "E3", None, status="pending")
    exp.validate_metrics([metric])


# ---------------------------------------------------------------------------
# 3. 目标比较
# ---------------------------------------------------------------------------
def test_target_met_semantics():
    assert exp.target_met({"target": 1800.0, "target_op": "<=",
                            "value": 12.0, "status": "measured"}) is True
    assert exp.target_met({"target": 1800.0, "target_op": "<=",
                            "value": 3600.0, "status": "measured"}) is False
    assert exp.target_met({"target": None, "target_op": None,
                            "value": 1.0, "status": "measured"}) is None
    assert exp.target_met({"target": 1.0, "target_op": "<=",
                            "value": None, "status": "blocked"}) is None


# ---------------------------------------------------------------------------
# 4. E1：doctor 输出解析 + 临时工作区测量
# ---------------------------------------------------------------------------
def test_parse_doctor_counts():
    parsed = helpers.parse_doctor(_DOCTOR_OK)
    assert parsed["checks"] == 5
    assert parsed["errors"] == 0        # INFO 不算「需处理」
    assert parsed["non_pass"] == 1
    parsed_bad = helpers.parse_doctor(_DOCTOR_BAD)
    assert parsed_bad["checks"] == 4
    assert parsed_bad["errors"] == 2    # NEEDS_HUMAN + RECOVERABLE
    assert parsed_bad["non_pass"] == 3


def test_measure_e1_with_fake_runner(repo_root, ws, scenarios, tmp_path):
    """E1：注入假 runner → 指标齐全、T2FM 有值、改码行数 0。"""
    clean_urdf = os.path.join(repo_root, "dualarm_ws", "src", "rm_description",
                              "urdf", "rm75_clean.urdf")

    def fake_runner(cmd, cwd=None, env=None, timeout_s=None):
        text = " ".join(str(c) for c in cmd)
        if "clean_urdf.py" in text:
            # 假「prepare」：真做清洗动作（把清洗后的 URDF 装到 --dst）
            dst = text.split("--dst ")[1].split(" ")[0]
            shutil.copyfile(clean_urdf, dst)
            return _fake_result(duration=0.1)
        if "doctor" in text:
            return _fake_result(stdout=_DOCTOR_OK, duration=0.2)
        return _fake_result(duration=0.1)

    metrics = exp.measure_e1(repo_root, ws, scenarios, runner=fake_runner,
                             tmp_root=str(tmp_path))
    by_id = {m["id"]: m for m in metrics}
    assert set(by_id) == {"E1.t2fm_s", "E1.command_count",
                           "E1.config_lines_changed", "E1.code_lines_changed",
                           "E1.error_count", "E1.doctor_check_count"}
    assert by_id["E1.t2fm_s"]["value"] is not None
    assert by_id["E1.t2fm_s"]["value"] <= 1800.0
    assert by_id["E1.error_count"]["value"] == 0
    assert by_id["E1.doctor_check_count"]["value"] == 5
    assert by_id["E1.code_lines_changed"]["value"] == 0
    assert by_id["E1.config_lines_changed"]["value"] == 0
    for metric in metrics:
        assert metric["mode"] == "offline"
        assert metric["status"] == "measured"


def test_measure_e1_variant_failure_gives_null_t2fm(repo_root, ws, scenarios,
                                                      tmp_path, monkeypatch):
    """负例：变体装不上 → T2FM 记 null + reason，**不伪造数字**。"""

    def fake_runner(cmd, cwd=None, env=None, timeout_s=None):
        text = " ".join(str(c) for c in cmd)
        if "doctor" in text:
            return _fake_result(stdout=_DOCTOR_BAD, duration=0.2)
        if "plan" in text:
            return _fake_result(exit_code=1, duration=0.1)
        return _fake_result(duration=0.1)

    metrics = exp.measure_e1(repo_root, ws, scenarios, runner=fake_runner,
                             tmp_root=str(tmp_path))
    by_id = {m["id"]: m for m in metrics}
    assert by_id["E1.t2fm_s"]["value"] is None
    assert by_id["E1.t2fm_s"]["status"] == "blocked"
    assert by_id["E1.t2fm_s"]["evidence"]
    assert by_id["E1.error_count"]["value"] == 2   # 报错次数照实计入


# ---------------------------------------------------------------------------
# 5. E2 / E4：可数指标
# ---------------------------------------------------------------------------
def test_measure_e2_with_fake_runner(repo_root, ws, scenarios, tmp_path):
    def fake_runner(cmd, cwd=None, env=None, timeout_s=None):
        return _fake_result(duration=0.4)

    metrics = exp.measure_e2(repo_root, ws, scenarios, runner=fake_runner,
                             tmp_root=str(tmp_path))
    by_id = {m["id"]: m for m in metrics}
    assert by_id["E2.command_count"]["value"] == 1
    assert by_id["E2.first_success_rate"]["value"] == 1.0
    assert by_id["E2.new_task_first_run_ok"]["value"] is True
    assert by_id["E2.template_lines"]["value"] > 0
    for metric in metrics:
        assert metric["mode"] == "offline"


def test_measure_e2_failure_lowers_success_rate(repo_root, ws, scenarios,
                                                 tmp_path):
    calls = {"n": 0}

    def fake_runner(cmd, cwd=None, env=None, timeout_s=None):
        calls["n"] += 1
        # 前两次是 pick_place 连跑：第二次失败
        if calls["n"] == 2:
            return _fake_result(exit_code=4, duration=0.4)
        return _fake_result(duration=0.4)

    metrics = exp.measure_e2(repo_root, ws, scenarios, runner=fake_runner,
                             tmp_root=str(tmp_path))
    by_id = {m["id"]: m for m in metrics}
    assert by_id["E2.first_success_rate"]["value"] == pytest.approx(2 / 3.0)


def test_measure_e4_counts_two_commands(repo_root, ws, scenarios, tmp_path):
    def fake_runner(cmd, cwd=None, env=None, timeout_s=None):
        return _fake_result(duration=0.6)

    metrics = exp.measure_e4(repo_root, ws, scenarios, runner=fake_runner,
                             tmp_root=str(tmp_path))
    by_id = {m["id"]: m for m in metrics}
    assert by_id["E4.command_count"]["value"] == 2
    assert by_id["E4.command_count"]["value"] <= 3
    assert by_id["E4.operation_duration_s"]["value"] == pytest.approx(0.6)


# ---------------------------------------------------------------------------
# 6. E3：从 trace 派生（机器侧）+ 人工 pending
# ---------------------------------------------------------------------------
def test_derive_e3_from_traces(tmp_path):
    trace = tmp_path / "trace.jsonl"
    _write_raw_trace(trace, [
        json.dumps(_event(100.0), ensure_ascii=False),
        json.dumps(_event(101.0, severity="RECOVERABLE", status="ok",
                          next_action="松手后重试闭合", reason="判定=stall"),
                   ensure_ascii=False),
        json.dumps(_event(102.0, severity="RECOVERABLE", status="blocked",
                          next_action="需人工检查夹爪", reason="升级人工"),
                   ensure_ascii=False),
        "{bad json line",
    ])
    derived = exp.derive_e3_from_traces([str(trace)])
    assert derived["failures"] == 2
    assert derived["diagnosability_rate"] == 1.0
    assert derived["mttr_proxy_s"] is not None
    assert derived["mttr_proxy_s"] >= 0.0
    assert derived["restart_required"] is False
    assert derived["skipped_lines"] == 1


def test_derive_e3_partial_diagnosability(tmp_path):
    trace = tmp_path / "trace.jsonl"
    _write_trace(trace, [
        _event(100.0, severity="RECOVERABLE", next_action="重试"),
        _event(101.0, severity="RECOVERABLE", next_action=""),
    ])
    derived = exp.derive_e3_from_traces([str(trace)])
    assert derived["diagnosability_rate"] == 0.5


def test_measure_e3_derives_and_keeps_human_pending(repo_root, ws, scenarios,
                                                      tmp_path, monkeypatch):
    trace = tmp_path / "trace.jsonl"
    _write_trace(trace, [
        _event(100.0, severity="RECOVERABLE", next_action="松手后重试闭合"),
        _event(101.0, severity="NEEDS_HUMAN", status="blocked",
               next_action="需人工检查夹爪"),
    ])
    monkeypatch.setattr(exp, "_trace_paths", lambda _ws: [str(trace)])

    metrics = exp.measure_e3(repo_root, ws, scenarios,
                              runner=lambda *a, **k: _fake_result(),
                              tmp_root=str(tmp_path))
    by_id = {m["id"]: m for m in metrics}
    # 可诊断率：1.0 或明确 blocked（两者之外不允许）
    rate = by_id["E3.diagnosability_rate"]
    assert rate["value"] == 1.0 or rate["status"] == "blocked"
    assert rate["value"] == 1.0
    # 真 MTTR：null + pending（人工字段）
    human = by_id["E3.mttr_human_s"]
    assert human["value"] is None
    assert human["status"] == "pending"
    assert human["mode"] == "human_pending"


def test_measure_e3_blocked_without_trace(repo_root, ws, scenarios,
                                           tmp_path, monkeypatch):
    """负例：trace 缺失 → 相关指标 blocked + null（**不产出 0**）。"""
    monkeypatch.setattr(exp, "_trace_paths", lambda _ws: [])
    metrics = exp.measure_e3(repo_root, ws, scenarios,
                              runner=lambda *a, **k: _fake_result(),
                              tmp_root=str(tmp_path),
                              reports_dir=str(tmp_path))
    by_id = {m["id"]: m for m in metrics}
    assert by_id["E3.diagnosability_rate"]["value"] is None
    assert by_id["E3.diagnosability_rate"]["status"] == "blocked"
    assert by_id["E3.mttr_proxy_s"]["value"] is None
    assert by_id["E3.mttr_human_s"]["status"] == "pending"


# ---------------------------------------------------------------------------
# 7. 段级入口：报告落盘 + 每行带 mode
# ---------------------------------------------------------------------------
def _fake_metrics(scenarios, sid):
    rows = []
    for mid in scenarios["scenarios"][sid]["metric_order"]:
        spec = scenarios["scenarios"][sid]["metrics"][mid]
        if spec["mode"] == "human_pending":
            rows.append(exp.build_metric(spec, sid, None, status="pending"))
        else:
            value = 1.0
            if spec["target_op"] == ">=":
                value = max(float(spec["target"]), 1.0)
            elif spec["target_op"] == "<=":
                value = min(float(spec["target"]), 1.0)
            elif spec["target_op"] == "==":
                value = spec["target"]
            rows.append(exp.build_metric(spec, sid, value, status="measured"))
    return rows


def test_run_experience_writes_reports_with_mode(repo_root, ws, scenarios,
                                                 tmp_path, monkeypatch):
    for sid in ("E1", "E2", "E3", "E4"):
        monkeypatch.setattr(exp, "measure_%s" % sid.lower(),
                            (lambda s: (lambda *a, **k: _fake_metrics(scenarios, s)))
                            (sid))
    reports_dir = tmp_path / "reports"
    result = exp.run_experience(repo_root=repo_root, ws=ws,
                                reports_dir=str(reports_dir))
    assert result["verdict"] == "pass"
    json_path = os.path.join(str(reports_dir), exp.JSON_NAME)
    md_path = os.path.join(str(reports_dir), exp.MD_NAME)
    assert os.path.isfile(json_path)
    assert os.path.isfile(md_path)

    with open(json_path, "r", encoding="utf-8") as fh:
        payload = json.load(fh)
    assert [s["id"] for s in payload["scenarios"]] == ["E1", "E2", "E3", "E4"]
    for scenario in payload["scenarios"]:
        assert scenario["metrics"], scenario["id"]
        for metric in scenario["metrics"]:
            assert metric["mode"] in exp.METRIC_MODES
            assert metric["mode"] != "live"

    with open(md_path, "r", encoding="utf-8") as fh:
        markdown = fh.read()
    assert "E1" in markdown and "E4" in markdown
    assert "`offline`" in markdown
    assert "`human_pending`" in markdown
    # 人工字段必须是 null + pending
    human = [m for m in payload["metrics"] if m["id"] == "E3.mttr_human_s"][0]
    assert human["value"] is None
    assert human["status"] == "pending"


def test_run_experience_fails_on_target_violation(repo_root, ws, scenarios,
                                                  tmp_path, monkeypatch):
    def _bad(sid):
        rows = []
        for mid in scenarios["scenarios"][sid]["metric_order"]:
            spec = scenarios["scenarios"][sid]["metrics"][mid]
            if spec["mode"] == "human_pending":
                rows.append(exp.build_metric(spec, sid, None, status="pending"))
            elif spec["id"] == "t2fm_s":
                rows.append(exp.build_metric(spec, sid, 9999.0,
                                             status="measured"))
            else:
                rows.append(exp.build_metric(spec, sid, 1.0, status="measured"))
        return rows

    for sid in ("E1", "E2", "E3", "E4"):
        monkeypatch.setattr(exp, "measure_%s" % sid.lower(),
                            (lambda s: (lambda *a, **k: _bad(s)))(sid))
    result = exp.run_experience(repo_root=repo_root, ws=ws,
                                reports_dir=str(tmp_path / "reports"))
    assert result["verdict"] == "fail"
    assert "E1.t2fm_s" in result["reason"]


def test_run_experience_blocks_when_data_source_missing(repo_root, ws,
                                                        scenarios, tmp_path,
                                                        monkeypatch):
    def _blocked(sid):
        rows = []
        for mid in scenarios["scenarios"][sid]["metric_order"]:
            spec = scenarios["scenarios"][sid]["metrics"][mid]
            if spec["mode"] == "human_pending":
                rows.append(exp.build_metric(spec, sid, None, status="pending"))
            else:
                rows.append(exp.build_metric(spec, sid, None, status="blocked"))
        return rows

    for sid in ("E1", "E2", "E3", "E4"):
        monkeypatch.setattr(exp, "measure_%s" % sid.lower(),
                            (lambda s: (lambda *a, **k: _blocked(s)))(sid))
    result = exp.run_experience(repo_root=repo_root, ws=ws,
                                reports_dir=str(tmp_path / "reports"))
    assert result["verdict"] == "blocked"
    assert result["next_action"]


def test_orchestrator_registers_experience_segment():
    """一键回归编排器必须注册 experience 段。"""
    assert "experience" in orch.segment_ids()
    assert orch.SPECS["experience"].mode == "offline"
