"""证据收集器（scripts/collect_evidence.py）的判定逻辑与退出码语义单测。

**为什么用假命令**
----------------
收集器会真实地跑重负载命令（规划链、CI 档）。本测试**不得**真的跑规划，
因此全部用例都在 ``tmp_path`` 下生成**假可执行脚本**（``#!/bin/sh`` 小脚本），
用来驱动四种判定分支：

* 通过（退出码 0 + 判定正则命中）；
* 失败（退出码不符 / 判定正则不命中）；
* 超时（超过 ``timeout_s`` 未结束）；
* 缺命令（可执行文件不存在 → 环境缺失 → SKIP，不得伪装成 PASS）。

同时验证收集器自身的退出码语义：0 = 无 FAIL、1 = 存在 FAIL、2 = 参数错误。
"""

from __future__ import annotations

import importlib.util
import json
import os
import stat
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = REPO_ROOT / "scripts" / "collect_evidence.py"


def _load_module():
    """按文件路径加载 ``scripts/collect_evidence.py``（scripts 不是包）。"""

    spec = importlib.util.spec_from_file_location("collect_evidence", SCRIPT_PATH)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    # 必须先注册进 sys.modules：dataclass 装饰器会回查 sys.modules[cls.__module__]。
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


collect_evidence = _load_module()


def _make_script(tmp_path: Path, name: str, body: str) -> str:
    """生成一个可执行的假命令脚本，返回其绝对路径。"""

    path = tmp_path / name
    path.write_text("#!/bin/sh\n" + body, encoding="utf-8")
    path.chmod(path.stat().st_mode | stat.S_IXUSR | stat.S_IXGRP)
    return str(path)


# ── 四种判定分支 ─────────────────────────────────────────────────────
def test_pass_case_records_exit_code_and_duration(tmp_path):
    """通过：退出码 0 且判定正则命中 → PASS。"""

    script = _make_script(tmp_path, "ok.sh", "echo 'OK: 42 passed'\nexit 0\n")
    step = collect_evidence.Step(command=(script,), expect_exit=0, pattern=r"\d+ passed")
    result = collect_evidence.run_step(step, tmp_path)

    assert result.status == "PASS"
    assert result.exit_code == 0
    assert result.matched is True
    assert result.duration_s >= 0.0
    assert "42 passed" in result.summary


def test_fail_case_on_exit_code_mismatch(tmp_path):
    """失败：退出码与期望不符 → FAIL（不得记为通过）。"""

    script = _make_script(tmp_path, "bad_exit.sh", "echo 'boom'\nexit 3\n")
    step = collect_evidence.Step(command=(script,), expect_exit=0)
    result = collect_evidence.run_step(step, tmp_path)

    assert result.status == "FAIL"
    assert result.exit_code == 3
    assert "exit_code_mismatch" in result.reason


def test_fail_case_on_pattern_mismatch(tmp_path):
    """失败：退出码 0 但判定正则不命中 → FAIL。"""

    script = _make_script(tmp_path, "bad_pattern.sh", "echo 'no marker here'\nexit 0\n")
    step = collect_evidence.Step(command=(script,), expect_exit=0, pattern=r"MUST_NOT_MATCH_\d+")
    result = collect_evidence.run_step(step, tmp_path)

    assert result.status == "FAIL"
    assert result.exit_code == 0
    assert "pattern_mismatch" in result.reason


def test_fail_case_on_timeout(tmp_path):
    """失败：超过超时未结束 → FAIL（reason=timeout）。"""

    script = _make_script(tmp_path, "slow.sh", "sleep 5\nexit 0\n")
    step = collect_evidence.Step(command=(script,), expect_exit=0, timeout_s=0.3)
    result = collect_evidence.run_step(step, tmp_path)

    assert result.status == "FAIL"
    assert result.exit_code is None
    assert "timeout" in result.reason


def test_skip_case_on_missing_command(tmp_path):
    """缺命令：可执行文件不存在 → SKIP（环境缺失，不计为通过，也不静默丢弃）。"""

    missing = str(tmp_path / "definitely_missing_binary")
    step = collect_evidence.Step(command=(missing,), expect_exit=0)
    result = collect_evidence.run_step(step, tmp_path)

    assert result.status == "SKIP"
    assert result.exit_code is None
    assert "command_not_found" in result.reason


def test_skip_exit_code_is_reported_as_skip(tmp_path):
    """声明式 skip_exit_codes：退出码命中该集合 → SKIP。"""

    script = _make_script(tmp_path, "skipme.sh", "echo 'env not ready'\nexit 2\n")
    step = collect_evidence.Step(command=(script,), expect_exit=0, skip_exit_codes=(2,))
    result = collect_evidence.run_step(step, tmp_path)

    assert result.status == "SKIP"
    assert result.exit_code == 2
    assert "skip_exit_code" in result.reason


# ── 检查级聚合 ───────────────────────────────────────────────────────
def test_check_with_multiple_steps_stops_at_first_failure(tmp_path):
    """多步骤检查：第一步失败即整条 FAIL，后续步骤不再执行。"""

    marker = tmp_path / "second_ran.txt"
    first = _make_script(tmp_path, "step1.sh", "echo 'nope'\nexit 1\n")
    second = _make_script(tmp_path, "step2.sh", f"touch {marker}\necho 'ran'\n")
    check = collect_evidence.Check(
        id="multi",
        group="fake",
        description="多步骤检查",
        steps=(
            collect_evidence.Step(command=(first,), expect_exit=0),
            collect_evidence.Step(command=(second,), expect_exit=0),
        ),
    )
    result = collect_evidence.run_check(check, tmp_path)

    assert result.status == "FAIL"
    assert len(result.steps) == 1
    assert not marker.exists()


def test_check_with_missing_command_is_skipped_not_failed(tmp_path):
    """整条检查的 SKIP 传播：缺命令 → 检查 SKIP（不伪装 PASS）。"""

    check = collect_evidence.Check(
        id="missing",
        group="fake",
        description="缺命令检查",
        steps=(
            collect_evidence.Step(command=(str(tmp_path / "nope_binary"),), expect_exit=0),
        ),
    )
    result = collect_evidence.run_check(check, tmp_path)

    assert result.status == "SKIP"
    counts = collect_evidence.summarize([result])
    assert counts == {"passed": 0, "skipped": 1, "failed": 0, "total": 1}


# ── 选择语义 ─────────────────────────────────────────────────────────
def test_select_checks_by_group_and_by_id():
    """``--checks`` 既能选组也能选 id；未知名字抛 ValueError。"""

    checks = collect_evidence.select_checks("static")
    assert checks and all(c.group == "static" for c in checks)

    by_id = collect_evidence.select_checks("ci-fast")
    assert [c.id for c in by_id] == ["ci-fast"]

    with pytest.raises(ValueError):
        collect_evidence.select_checks("no-such-group")


def test_builtin_checklist_covers_required_s02_scope():
    """内置清单必须覆盖 S02 全量复跑要求的 7 类检查。"""

    ids = set(collect_evidence.ALL_CHECK_IDS)
    assert {
        "pytest-not-slow",
        "verify-env-model",
        "verify-env-plan",
        "forbidden-apis-scan",
        "cli-single-replay",
        "plan-stats",
        "ci-fast",
    } <= ids


# ── 退出码语义 ───────────────────────────────────────────────────────
def _patch_checks(monkeypatch, checks):
    monkeypatch.setattr(collect_evidence, "CHECKS", checks)


def test_main_exit_code_zero_when_all_pass(monkeypatch, tmp_path, capsys):
    """全部通过 → 退出码 0。"""

    script = _make_script(tmp_path, "ok.sh", "echo 'OK'\nexit 0\n")
    check = collect_evidence.Check(
        id="ok",
        group="fake",
        description="通过",
        steps=(collect_evidence.Step(command=(script,), expect_exit=0),),
    )
    _patch_checks(monkeypatch, [check])

    rc = collect_evidence.main(["--json"])
    assert rc == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["summary"]["passed"] == 1
    assert payload["summary"]["failed"] == 0


def test_main_exit_code_one_when_a_check_fails(monkeypatch, tmp_path, capsys):
    """存在 FAIL → 退出码 1，且 JSON 里记录命令 / 退出码 / 耗时。"""

    script = _make_script(tmp_path, "bad.sh", "echo 'boom'\nexit 7\n")
    check = collect_evidence.Check(
        id="bad",
        group="fake",
        description="失败",
        steps=(collect_evidence.Step(command=(script,), expect_exit=0),),
    )
    _patch_checks(monkeypatch, [check])

    rc = collect_evidence.main(["--json"])
    assert rc == 1
    payload = json.loads(capsys.readouterr().out)
    assert payload["summary"]["failed"] == 1
    step = payload["checks"][0]["steps"][0]
    assert step["exit_code"] == 7
    assert step["duration_s"] >= 0.0
    assert step["command"] == [script]


def test_main_exit_code_two_on_unknown_selector(monkeypatch, capsys):
    """参数错误（未知检查组）→ 退出码 2。"""

    _patch_checks(monkeypatch, [])
    rc = collect_evidence.main(["--checks", "definitely-not-a-group"])
    assert rc == 2
    assert "未知的检查组" in capsys.readouterr().err


def test_main_writes_json_output_file(monkeypatch, tmp_path, capsys):
    """``--output`` 把机器可读结果写入文件。"""

    script = _make_script(tmp_path, "ok.sh", "echo 'OK'\nexit 0\n")
    check = collect_evidence.Check(
        id="ok",
        group="fake",
        description="通过",
        steps=(collect_evidence.Step(command=(script,), expect_exit=0),),
    )
    _patch_checks(monkeypatch, [check])
    out = tmp_path / "evidence.json"

    rc = collect_evidence.main(["--output", str(out)])
    assert rc == 0
    capsys.readouterr()
    payload = json.loads(out.read_text(encoding="utf-8"))
    assert payload["generator"] == "scripts/collect_evidence.py"
    assert payload["checks"][0]["id"] == "ok"


def test_main_skip_is_counted_and_not_reported_as_pass(monkeypatch, tmp_path, capsys):
    """SKIP 项计入汇总，且不计入 passed。"""

    check = collect_evidence.Check(
        id="missing",
        group="fake",
        description="缺命令",
        steps=(collect_evidence.Step(command=(str(tmp_path / "nope"),), expect_exit=0),),
    )
    _patch_checks(monkeypatch, [check])

    rc = collect_evidence.main(["--json"])
    assert rc == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["summary"]["skipped"] == 1
    assert payload["summary"]["passed"] == 0


def test_human_report_lists_every_check_and_summary(monkeypatch, tmp_path, capsys):
    """人类可读报告逐条列出检查，并给出汇总计数。"""

    script = _make_script(tmp_path, "ok.sh", "echo 'OK'\nexit 0\n")
    check = collect_evidence.Check(
        id="ok",
        group="fake",
        description="通过检查",
        steps=(collect_evidence.Step(command=(script,), expect_exit=0),),
    )
    _patch_checks(monkeypatch, [check])

    rc = collect_evidence.main([])
    assert rc == 0
    out = capsys.readouterr().out
    assert "[PASS] ok" in out
    assert "汇总：PASS=1 SKIP=0 FAIL=0" in out


def test_list_option_prints_check_inventory(capsys):
    """``--list`` 列出清单（id / 组）。"""

    rc = collect_evidence.main(["--list"])
    assert rc == 0
    out = capsys.readouterr().out
    for check_id in collect_evidence.ALL_CHECK_IDS:
        assert check_id in out


def test_environment_info_reports_python_and_drake():
    """环境指纹含 Python / Drake 版本，便于标注 fallback 档。"""

    info = collect_evidence.environment_info()
    assert info["python_version"]
    assert "baseline_note" in info
    assert info["drake_version"] is None or isinstance(info["drake_version"], str)


def test_module_has_no_syntax_error_and_is_executable():
    """收集器自身语法合法（py_compile 兜底）。"""

    import py_compile

    py_compile.compile(str(SCRIPT_PATH), doraise=True)
    assert os.access(SCRIPT_PATH, os.R_OK)
