"""doctor 的机器可读输出 / 退出码口径 / 自检覆盖矩阵（S09/T6）测试。

覆盖：

* ``--json`` 的 stdout **可被 json.loads 解析**，且含 ``checks[]`` 与
  ``coverage_matrix[]``；
* ``--report <path>`` 落盘内容与 stdout 的 JSON **等价**；
* 自检覆盖矩阵 **≥ 8 类**，每类有 ``covered`` 布尔；未覆盖时必须给出
  缺口说明（``gap``）与**可粘贴**的 ``next_action``；
* 退出码三态：全 PASS → 0；存在需人工/外部条件的项 → 3；存在硬红线 → 4；
* **既有检查项名一条不少**（workspace / model:* / dep:nanobind / model:load /
  fk:self_vs_tesseract / collision:home / limits:consistent /
  baselines:present / limits:measured / hw:*）；
* 退出码与执行层契约 ``rm_executor.contract.EXIT_CODES`` **同值**；
* JSON 里**不含**凭据类字段（Q3：doctor 输出会被脚本消费）。

**不依赖本机是否有真机 / 厂商库**：所有"缺失"场景都在临时工作区或
monkeypatch 的检查项集合里构造。
"""
import json
import os

import pytest

from rmctl import cli
from rmctl import doctor


def _run(capsys, argv):
    code = cli.main(argv)
    out = capsys.readouterr().out
    return code, out


# --------------------------------------------------------------- JSON 形态
def test_json_output_is_parsable_with_checks_and_coverage(capsys, repo_root):
    """--json 的 stdout 是合法 JSON，且含 checks[] 与 coverage_matrix[]。"""
    code, out = _run(capsys, ["--ws", repo_root, "doctor", "--json"])
    assert code in (0, 3, 4), out  # 退出码口径见下（不再是 0/1）
    payload = json.loads(out)
    assert isinstance(payload["checks"], list) and payload["checks"]
    assert isinstance(payload["coverage_matrix"], list)
    for c in payload["checks"]:
        for key in ("name", "ok", "detail", "action", "level"):
            assert key in c, (c.get("name"), key)
    assert payload["exit_code"] == code


def test_report_file_equals_stdout_json(capsys, repo_root, tmp_path):
    """--report 落盘内容与 stdout 的 JSON 等价（同一份报告）。"""
    out_path = tmp_path / "doctor.json"
    code, out = _run(capsys, ["--ws", repo_root, "doctor", "--json",
                              "--report", str(out_path)])
    assert code in (0, 3, 4)
    assert out_path.exists()
    from_file = json.loads(out_path.read_text(encoding="utf-8"))
    from_stdout = json.loads(out)
    assert from_file == from_stdout


def test_report_without_json_still_writes_json(capsys, repo_root, tmp_path):
    """只给 --report（不给 --json）时：stdout 仍是人读文本，落盘是 JSON。"""
    out_path = tmp_path / "doctor_only_report.json"
    code, out = _run(capsys, ["--ws", repo_root, "doctor",
                              "--report", str(out_path)])
    assert code in (0, 3, 4)
    assert "[PASS]" in out or "[NEEDS_HUMAN]" in out or "[BLOCKED]" in out
    payload = json.loads(out_path.read_text(encoding="utf-8"))
    assert payload["checks"] and payload["coverage_matrix"]


def test_json_has_no_credential_fields(capsys, repo_root):
    """Q3：JSON 里不得含凭据/口令类字段（doctor 输出会被脚本消费）。"""
    _, out = _run(capsys, ["--ws", repo_root, "doctor", "--json"])
    payload = json.loads(out)
    banned = ("password", "passwd", "secret", "token", "credential", "apikey")

    def _walk(node):
        if isinstance(node, dict):
            for key, value in node.items():
                assert str(key).lower() not in banned, key
                _walk(value)
        elif isinstance(node, list):
            for item in node:
                _walk(item)

    _walk(payload)


# ----------------------------------------------------------- 覆盖矩阵
def test_coverage_matrix_has_at_least_eight_categories(capsys, repo_root):
    """设计 §6.2：自检覆盖 ≥ 8 类（网络/供电/夹爪/零位/FK/限值/控制器状态…）。"""
    _, out = _run(capsys, ["--ws", repo_root, "doctor", "--json"])
    matrix = json.loads(out)["coverage_matrix"]
    assert len(matrix) >= 8
    for entry in matrix:
        assert "category" in entry and "label" in entry
        assert isinstance(entry["covered"], bool)
        assert entry["checks"], entry
        if not entry["covered"]:
            assert entry["gap"], entry
            assert entry["next_action"], entry


def test_coverage_categories_cover_named_design_classes(capsys, repo_root):
    """点名的 7 类 + 依赖与模型必须都在矩阵里。"""
    _, out = _run(capsys, ["--ws", repo_root, "doctor", "--json"])
    matrix = json.loads(out)["coverage_matrix"]
    cats = {e["category"] for e in matrix}
    for expected in ("network", "power", "gripper", "zero", "fk", "limits",
                     "controller", "deps_model"):
        assert expected in cats, (expected, sorted(cats))


def test_uncovered_category_is_explicit_not_pass():
    """负例（Q7）：某类别无对应 check → covered=false + 缺口说明 + 修复命令。

    这里模拟「厂商库不可导入 → model:load 失败 → FK/碰撞/限值一致性被
    级联跳过」：FK 类没有对应 check，**不得**被标成「通过」。
    """
    checks = [
        {"name": "workspace", "ok": True, "detail": "", "action": "",
         "level": "INFO"},
        {"name": "model:urdf", "ok": True, "detail": "", "action": "",
         "level": "INFO"},
        {"name": "model:srdf", "ok": True, "detail": "", "action": "",
         "level": "INFO"},
        {"name": "model:joint_limits", "ok": True, "detail": "", "action": "",
         "level": "INFO"},
        {"name": "dep:nanobind", "ok": True, "detail": "", "action": "",
         "level": "INFO"},
        {"name": "model:load", "ok": False, "detail": "厂商库不可导入",
         "action": "pip install tesseract-robotics-nanobind==0.35.0.8",
         "level": "NEEDS_HUMAN"},
    ]
    matrix = {e["category"]: e for e in doctor.coverage_matrix(checks)}
    assert matrix["fk"]["covered"] is False
    assert matrix["fk"]["status"] == "missing"
    assert matrix["fk"]["gap"]
    assert matrix["fk"]["next_action"]
    assert matrix["collision"]["covered"] is False
    # 依赖与模型这一类：model:load **在列**（只是没过）→ 仍然 covered
    assert matrix["deps_model"]["covered"] is True


def test_coverage_category_with_failing_check_is_covered_but_not_ok():
    """检查项在列但没过 → covered=true，status 不得是 ok。"""
    checks = [{"name": "fk:self_vs_tesseract", "ok": False, "detail": "偏差过大",
               "action": "检查 URDF 清洗结果", "level": "RECOVERABLE"}]
    matrix = {e["category"]: e for e in doctor.coverage_matrix(checks)}
    assert matrix["fk"]["covered"] is True
    assert matrix["fk"]["status"] == "blocked"


# ----------------------------------------------------------- 退出码三态
def _fake_checks(*specs):
    return [{"name": n, "ok": ok, "detail": "d", "action": "a", "level": lvl}
            for n, ok, lvl in specs]


def test_exit_code_zero_when_all_pass(monkeypatch, capsys, repo_root):
    monkeypatch.setattr(doctor, "collect_checks",
                        lambda ws=None, group="rm75_arm": _fake_checks(
                            ("workspace", True, "INFO"),
                            ("model:load", True, "INFO")))
    code, out = _run(capsys, ["--ws", repo_root, "doctor", "--json"])
    assert code == 0
    assert json.loads(out)["status"] == "ok"


def test_exit_code_three_when_blocked_item(monkeypatch, capsys, repo_root):
    """存在需人工/外部条件的项 → 3（不是 1，也**不是** 0）。"""
    monkeypatch.setattr(doctor, "collect_checks",
                        lambda ws=None, group="rm75_arm": _fake_checks(
                            ("workspace", True, "INFO"),
                            ("power:supply", False, "NEEDS_HUMAN"),
                            ("controller:state", False, "BLOCKED")))
    code, out = _run(capsys, ["--ws", repo_root, "doctor", "--json"])
    assert code == 3
    payload = json.loads(out)
    assert payload["status"] == "blocked"
    assert payload["exit_code"] == 3


def test_exit_code_four_when_hard_failure(monkeypatch, capsys, repo_root):
    """存在硬红线项 → 4。"""
    monkeypatch.setattr(doctor, "collect_checks",
                        lambda ws=None, group="rm75_arm": _fake_checks(
                            ("workspace", True, "INFO"),
                            ("hw:contract", False, "FAIL")))
    code, out = _run(capsys, ["--ws", repo_root, "doctor", "--json"])
    assert code == 4
    payload = json.loads(out)
    assert payload["status"] == "error"
    assert payload["exit_code"] == 4


def test_info_level_failure_is_not_a_failure():
    """INFO 级未通过**不算**失败（与既有 limits:measured 语义一致）。"""
    assert doctor.exit_code_for(_fake_checks(
        ("limits:measured", False, "INFO"),
        ("hardware:connected", False, "INFO"))) == 0


def test_exit_codes_match_executor_contract():
    """doctor 的 0/3/4 必须与 rm_executor.contract.EXIT_CODES 同值。"""
    from rm_executor import contract

    assert doctor.EXIT_OK == contract.EXIT_CODES[contract.STATUS_OK]
    assert doctor.EXIT_BLOCKED == contract.EXIT_CODES[contract.STATUS_BLOCKED]
    assert doctor.EXIT_ERROR == contract.EXIT_CODES[contract.STATUS_ERROR]


# --------------------------------------------------------- 既有检查项不回归
def test_existing_check_names_still_present(capsys, repo_root):
    """既有检查项一条不少（外加新增的 4 条一等检查项）。"""
    _, out = _run(capsys, ["--ws", repo_root, "doctor", "--json"])
    names = {c["name"] for c in json.loads(out)["checks"]}
    for expected in ("workspace", "model:urdf", "model:srdf",
                     "model:joint_limits", "dep:nanobind", "model:load",
                     "fk:self_vs_tesseract", "collision:home",
                     "limits:consistent", "baselines:present",
                     "limits:measured", "hw:sdk", "hw:symbols", "hw:contract",
                     "hw:limits_source", "hw:gripper",
                     # 新增一等检查项
                     "network:plan", "zero:baseline", "power:supply",
                     "controller:state"):
        assert expected in names, (expected, sorted(names))


def test_new_checks_carry_copyable_actions(capsys, repo_root):
    """新增的一等检查项必须带**可粘贴**的下一步（R010）。"""
    _, out = _run(capsys, ["--ws", repo_root, "doctor", "--json"])
    by_name = {c["name"]: c for c in json.loads(out)["checks"]}
    assert "供电" in by_name["power:supply"]["action"] or \
        "供电" in by_name["power:supply"]["detail"]
    assert "rmctl connect" in by_name["controller:state"]["action"]
    for name in ("network:plan", "zero:baseline", "power:supply",
                 "controller:state"):
        assert by_name[name]["action"], name


# --------------------------------------------------------------- 回归段注册
def test_orchestrator_registers_doctor_segment():
    """一键回归编排器必须注册 doctor 段（读 doctor 的 JSON 报告）。"""
    from rmctl.regression import orchestrator

    assert "doctor" in orchestrator.segment_ids()


def test_doctor_segment_passes_when_doctor_has_blocked_items(monkeypatch,
                                                              repo_root,
                                                              tmp_path):
    """doctor 有 blocked 项时该段仍为 pass（doctor 本身执行成功）。

    段级语义见 orchestrator._run_doctor 的 docstring。
    """
    from rmctl.regression import orchestrator

    ctx = orchestrator.build_context(repo_root=repo_root, ws=repo_root)
    ctx.reports_dir = str(tmp_path)

    payload = {
        "schema_version": 1,
        "status": "blocked",
        "exit_code": 3,
        "checks": [
            {"name": "workspace", "ok": True, "detail": "", "action": "",
             "level": "INFO"},
            {"name": "power:supply", "ok": False, "detail": "无真机信号",
             "action": "检查外部供电与共地", "level": "NEEDS_HUMAN"},
        ],
        "coverage_matrix": [
            {"category": "power", "label": "供电与共地", "checks": ["power:supply"],
             "covered": True, "status": "blocked", "detail": "", "gap": "",
             "next_action": ""},
            {"category": "fk", "label": "FK 一致性", "checks": ["fk:self_vs_tesseract"],
             "covered": False, "status": "missing", "detail": "", "gap": "缺检查项",
             "next_action": "先让模型可加载"},
        ],
    }
    report_path = os.path.join(ctx.reports_dir, "s09_doctor.json")

    def _fake_run(cmd, cwd=None, env=None, text=True, stdout=None,
                  stderr=None, timeout=None):
        with open(report_path, "w", encoding="utf-8") as fh:
            json.dump(payload, fh)
        return type("P", (), {"returncode": 3, "stdout": "", "stderr": ""})()

    monkeypatch.setattr(orchestrator.subprocess, "run", _fake_run)
    result = orchestrator.run_segment("doctor", ctx)
    assert result["verdict"] == "pass"
    assert any("power:supply" in str(e) for e in result["evidence"])
    # 未覆盖的类别必须在证据里点名（不隐藏）
    assert any("FK" in str(e) for e in result["evidence"])


def test_doctor_segment_fails_when_report_unparsable(monkeypatch, repo_root,
                                                      tmp_path):
    """负例：doctor 报告不可解析 → 段 fail（不是 pass）。"""
    from rmctl.regression import orchestrator

    ctx = orchestrator.build_context(repo_root=repo_root, ws=repo_root)
    ctx.reports_dir = str(tmp_path)
    report_path = os.path.join(ctx.reports_dir, "s09_doctor.json")

    def _fake_run(cmd, cwd=None, env=None, text=True, stdout=None,
                  stderr=None, timeout=None):
        with open(report_path, "w", encoding="utf-8") as fh:
            fh.write("{ 这不是 JSON")
        return type("P", (), {"returncode": 3, "stdout": "", "stderr": ""})()

    monkeypatch.setattr(orchestrator.subprocess, "run", _fake_run)
    result = orchestrator.run_segment("doctor", ctx)
    assert result["verdict"] == "fail"
