"""S09/T2 一键回归编排器的**守卫用例**（段级 verdict / 退出码 / 负例）。

本文件守住四件事：

1. **正例**：一条命令产出 ``build/reports/s09_regression.json``，顶层含
   ``status`` / ``reason`` / ``next_action`` / ``evidence`` + ``segments[]``，
   每段含 ``id`` / ``verdict`` / ``reason`` / ``next_action`` / ``evidence`` /
   ``report_path``；
2. **负例 1（干净机器）**：没有 ``build/model`` 的临时工作区 → 依赖模型的段
   ``blocked``、整体退出码 3、报告里**没有任何伪造数字**；
3. **负例 2（真回归）**：某段断言不过 → 该段 ``fail``、整体退出码 4；
4. **负例 3 / 4**：未知段 id → argparse 退出码 2；一段 ``blocked`` 不得让
   整体变成 ``fail``。

外加失效模式（Q5）：子进程超时 / 被信号杀死 → 该段 ``fail``（不是 blocked），
reason 指名脚本与超时 / 信号。

夹具纪律：临时工作区与临时报告都落在 pytest ``tmp_path``（**不依赖任何
gitignore 目录**）；既有证据链报告由脚本在运行时产出。
"""
from __future__ import annotations

import json
import os
import subprocess
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(TESTS_DIR, "..", "..", "..", ".."))
CLI = os.path.join(REPO_ROOT, "scripts", "rm_regression.py")

from rmctl.regression import orchestrator as orch  # noqa: E402
from rmctl.regression import segments as seg_mod  # noqa: E402

#: 段级结果里**不允许**出现的「成功态指标」键（blocked 段不得带数字结论）
SUCCESS_METRIC_KEYS = ("numbers", "metrics", "cycles", "judge_count",
                       "success_rate", "p50_s", "p95_s", "load_profile")


def _run_cli(args, timeout=900):
    """跑一键回归 CLI（返回 CompletedProcess）。"""
    env = os.environ.copy()
    return subprocess.run(
        [sys.executable, CLI] + args,
        env=env,
        cwd=REPO_ROOT,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        timeout=timeout,
    )


def _load(path):
    with open(path, "r", encoding="utf-8") as fh:
        return json.load(fh)


def _segment(report, seg_id):
    for seg in report.get("segments") or []:
        if seg.get("id") == seg_id:
            return seg
    raise AssertionError("报告里没有段 %r: %s" % (seg_id, report))


# ---------------------------------------------------------------------------
# 正例
# ---------------------------------------------------------------------------
def test_cli_produces_report_with_expected_schema(tmp_path):
    """正例：一条命令产出报告；顶层与段级键齐全，段报告文件真实存在。"""
    out_path = tmp_path / "s09_regression.json"
    proc = _run_cli(["--out", str(out_path), "--only", "gripper_offline"])
    assert proc.returncode == 0, (
        "退出码 %s\nstdout:\n%s\nstderr:\n%s"
        % (proc.returncode, proc.stdout, proc.stderr)
    )
    assert out_path.is_file(), out_path
    report = _load(str(out_path))

    for key in ("status", "reason", "next_action", "evidence", "segments"):
        assert key in report, report
    assert isinstance(report["segments"], list) and report["segments"], report

    seg = _segment(report, "gripper_offline")
    for key in ("id", "verdict", "reason", "next_action", "evidence",
                "report_path"):
        assert key in seg, seg
    assert seg["verdict"] == "pass", seg
    assert seg["mode"] == "fake", seg
    assert seg["report_path"], seg
    assert os.path.isfile(seg["report_path"]), seg

    # 段报告 = 既有 S05 证据链的原报告（编排器只做归一化，不重写证据链）
    with open(seg["report_path"], "r", encoding="utf-8") as fh:
        raw = json.load(fh)
    assert raw["status"] == "ok", raw
    assert raw["mode"] == "fake", raw


def test_report_top_level_uses_executor_contract_exit_codes():
    """顶层 status / 退出码口径唯一来源 = rm_executor.contract。"""
    contract = seg_mod.contract()
    assert contract.EXIT_CODES[contract.STATUS_OK] == 0
    assert contract.EXIT_CODES[contract.STATUS_BLOCKED] == 3
    assert contract.EXIT_CODES[contract.STATUS_ERROR] == 4


# ---------------------------------------------------------------------------
# 负例 1：干净机器（无 build/model）
# ---------------------------------------------------------------------------
def test_clean_workspace_blocks_model_dependent_segments(tmp_path):
    """负例 1：没有 build/model 的工作区 → 依赖模型的段 blocked、退出码 3、
    报告里没有任何伪造数字。"""
    clean_ws = tmp_path / "clean_ws"
    clean_ws.mkdir()
    out_path = tmp_path / "clean.json"
    proc = _run_cli(["--ws", str(clean_ws), "--out", str(out_path)])
    assert proc.returncode == 3, (
        "退出码 %s\nstdout:\n%s\nstderr:\n%s"
        % (proc.returncode, proc.stdout, proc.stderr)
    )
    report = _load(str(out_path))
    assert report["exit_code"] == 3, report
    assert report["status"] == seg_mod.contract().STATUS_BLOCKED, report

    verdicts = [seg["verdict"] for seg in report["segments"]]
    assert "fail" not in verdicts, report
    assert "blocked" in verdicts, report

    for seg_id in ("process_offline", "planner_acceptance"):
        seg = _segment(report, seg_id)
        assert seg["verdict"] == "blocked", seg
        # blocked 段不得带任何「成功态指标」（不许拿 0 / 空值冒充结论）
        for key in SUCCESS_METRIC_KEYS:
            assert key not in seg, "blocked 段带成功态指标 %r: %s" % (key, seg)
        assert seg["next_action"], seg
        assert not seg["report_path"], seg


# ---------------------------------------------------------------------------
# 负例 2：真回归（段内断言不过）
# ---------------------------------------------------------------------------
def test_failing_segment_makes_overall_fail_with_exit_4(monkeypatch):
    """负例 2：某段报告 status=error（failures 非 model）→ 该段 fail、
    整体退出码 4。"""
    failing = orch.ExecResult(
        exit_code=4,
        report={
            "status": "error",
            "reason": "离线证据链失败段落: cycle",
            "next_action": "对照失败段落的 assertions 字段修正后重跑",
            "failures": ["cycle"],
            "evidence": ["cycle: 断言未通过"],
        },
    )
    monkeypatch.setattr(orch, "_execute", lambda spec, ctx, out: failing)

    ctx = orch.build_context(repo_root=REPO_ROOT, ws=REPO_ROOT)
    report = orch.run_regression(ctx, only=["gripper_offline"])
    seg = _segment(report, "gripper_offline")
    assert seg["verdict"] == "fail", seg
    assert report["exit_code"] == 4, report
    assert report["status"] == seg_mod.contract().STATUS_ERROR, report


# ---------------------------------------------------------------------------
# 负例 3：未知段 id → argparse 退出码 2
# ---------------------------------------------------------------------------
def test_unknown_segment_id_exits_2(tmp_path):
    """负例 3：未知段 id → argparse 拒绝（退出码 2），不静默跑空。"""
    proc = _run_cli(["--only", "no_such_segment",
                     "--out", str(tmp_path / "x.json")])
    assert proc.returncode == 2, (proc.returncode, proc.stdout, proc.stderr)
    assert "未知的段 id" in proc.stderr, proc.stderr


# ---------------------------------------------------------------------------
# 负例 4：blocked 不得让整体变成 fail
# ---------------------------------------------------------------------------
def test_blocked_segment_does_not_make_overall_fail(monkeypatch):
    """负例 4：一段 blocked + 一段 pass → 整体 blocked（退出码 3），不是 fail。"""
    monkeypatch.setitem(
        orch.SEGMENTS, "gripper_offline",
        lambda ctx: seg_mod.segment_result(
            "pass", reason="假段通过", next_action="", evidence=["ok"],
            mode="fake", report_path="", id="gripper_offline"))
    monkeypatch.setitem(
        orch.SEGMENTS, "process_offline",
        lambda ctx: seg_mod.segment_result(
            "blocked", reason="外部依赖未就绪", next_action="先 `python3 -m rmctl init`",
            evidence=["缺 build/model"], mode="fake", report_path="",
            id="process_offline"))

    ctx = orch.build_context(repo_root=REPO_ROOT, ws=REPO_ROOT)
    report = orch.run_regression(ctx, only=["gripper_offline", "process_offline"])
    verdicts = [seg["verdict"] for seg in report["segments"]]
    assert verdicts == ["pass", "blocked"], report
    assert report["verdict"] == "blocked", report
    assert report["exit_code"] == 3, report
    assert report["status"] != seg_mod.contract().STATUS_ERROR, report


# ---------------------------------------------------------------------------
# 失效模式（Q5）：超时 / 信号 / 报告缺失
# ---------------------------------------------------------------------------
def test_timeout_is_fail_and_names_script_and_budget(monkeypatch):
    """子进程超时 → 该段 fail（不是 blocked），reason 指名脚本与超时秒数。"""
    monkeypatch.setattr(
        orch, "_execute",
        lambda spec, ctx, out: orch.ExecResult(timed_out=True,
                                                timeout_s=spec.timeout_s))
    ctx = orch.build_context(repo_root=REPO_ROOT, ws=REPO_ROOT)
    report = orch.run_regression(ctx, only=["gripper_offline"])
    seg = _segment(report, "gripper_offline")
    assert seg["verdict"] == "fail", seg
    assert "gripper_offline_evidence.py" in seg["reason"], seg
    assert "超时" in seg["reason"], seg
    assert report["exit_code"] == 4, report


def test_signal_kill_is_fail_and_names_signal(monkeypatch):
    """子进程被信号杀死 → 该段 fail，reason 指名退出信号。"""
    monkeypatch.setattr(
        orch, "_execute",
        lambda spec, ctx, out: orch.ExecResult(exit_code=-9, signal_num=9))
    ctx = orch.build_context(repo_root=REPO_ROOT, ws=REPO_ROOT)
    report = orch.run_regression(ctx, only=["gripper_offline"])
    seg = _segment(report, "gripper_offline")
    assert seg["verdict"] == "fail", seg
    assert "信号 9" in seg["reason"], seg
    assert report["exit_code"] == 4, report


def test_missing_report_is_fail_not_blocked(monkeypatch):
    """报告缺失 → 该段 fail（编排器契约问题，不是外部条件未满足）。"""
    monkeypatch.setattr(
        orch, "_execute",
        lambda spec, ctx, out: orch.ExecResult(
            exit_code=4, report=None, report_error="脚本未产出报告文件"))
    ctx = orch.build_context(repo_root=REPO_ROOT, ws=REPO_ROOT)
    report = orch.run_regression(ctx, only=["gripper_offline"])
    seg = _segment(report, "gripper_offline")
    assert seg["verdict"] == "fail", seg
    assert report["exit_code"] == 4, report


# ---------------------------------------------------------------------------
# 归一化纯函数（降级陷阱 a/b）
# ---------------------------------------------------------------------------
def test_normalize_model_failure_is_blocked_not_fail():
    """降级陷阱：failures=[model] → blocked（缺模型是环境问题，不是回归）。"""
    norm = orch.normalize_report(
        4,
        {"status": "error", "reason": "模型不可用", "failures": ["model"],
         "next_action": "执行 `python3 -m rmctl init` 物化 build/model 后重跑",
         "evidence": []},
        script_name="scripts/rm_planner_acceptance.py")
    assert norm["verdict"] == "blocked", norm
    assert norm["next_action"], norm


def test_normalize_other_failures_are_fail():
    """其余 failures（段内断言不过）→ fail。"""
    norm = orch.normalize_report(
        4,
        {"status": "error", "reason": "离线证据链失败段落: cycle",
         "failures": ["cycle"], "evidence": []},
        script_name="scripts/gripper_offline_evidence.py")
    assert norm["verdict"] == "fail", norm


def test_normalize_missing_report_is_fail():
    """报告缺失 → fail（不是 blocked）。"""
    assert orch.normalize_report(4, None)["verdict"] == "fail"
    assert orch.normalize_report(0, "not-a-dict")["verdict"] == "fail"


def test_normalize_ok_with_nonzero_exit_is_fail():
    """报告 ok 但退出码非 0 → fail（自相矛盾 = 契约回归）。"""
    norm = orch.normalize_report(4, {"status": "ok", "failures": []})
    assert norm["verdict"] == "fail", norm


def test_normalize_unknown_status_is_fail():
    """未知 status → fail（绝不静默当 pass）。"""
    norm = orch.normalize_report(0, {"status": "weird", "failures": []})
    assert norm["verdict"] == "fail", norm


# ---------------------------------------------------------------------------
# 注册表
# ---------------------------------------------------------------------------
def test_registry_exposes_three_evidence_chains():
    """本任务收编三条既有证据链（后续任务各自追加注册行）。"""
    assert orch.segment_ids() == ["gripper_offline", "process_offline",
                                  "planner_acceptance"]
    for seg_id, spec in orch.SPECS.items():
        assert os.path.isfile(os.path.join(REPO_ROOT, spec.script)), spec
        assert spec.report_name.endswith(".json"), spec
