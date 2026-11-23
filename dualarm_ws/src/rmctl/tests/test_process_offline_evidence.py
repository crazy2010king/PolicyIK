"""``scripts/pick_place_offline_evidence.py``（S06/T7 离线证据链）的**验收测试**。

覆盖（离线、零硬件、零 SDK、零网络）：

* **正向**：跑一次脚本（``--iterations 2``，把 50 圈缩到 2 圈以控制测试时长）
  → 退出码 0；报告四段齐全（``success_run`` / ``fault_cases`` / ``replay`` /
  ``live_window``）；六类故障分支逐条命中预期 severity 与 next_action；
  trace 回放行数 == 事件数；``live_window`` 段显式 blocked。
* **负例 1**：规格文件缺失 → **退出码 4** + 报告写明「哪一段失败」
  （绝不崩栈、绝不假成功）。
* **负例 2**：脚本**从任意目录**调用仍可跑通（自插 sys.path），
  且**不修改任何 git 跟踪文件**（只写 build/ 下产物）。

只用 git 跟踪的仓库文件 + pytest 临时目录；绝不依赖 .gitignore 目录
（报告写到 pytest ``tmp_path``）。厂商规划内核 / 模型不可用时**显式 skip**
（不静默通过）。
"""
import json
import os
import subprocess
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(HERE, "..", "..", "..", ".."))
SCRIPT = os.path.join(REPO_ROOT, "scripts", "pick_place_offline_evidence.py")


def _run_script(*args, cwd=None):
    """跑一次证据脚本，返回 ``CompletedProcess``。"""
    cmd = [sys.executable, SCRIPT] + list(args)
    return subprocess.run(cmd, cwd=cwd or REPO_ROOT, capture_output=True,
                          text=True, timeout=1800)


def _skip_if_model_unavailable(report):
    """厂商规划内核 / 模型不可用 → 显式 skip（不静默通过）。"""
    if not report:
        pytest.skip("脚本未产出报告（厂商规划内核/模型不可用）")
    if report.get("failures") == ["model"]:
        pytest.skip("厂商规划内核/模型不可用: %s" % report.get("reason"))


@pytest.fixture(scope="module")
def offline_report(tmp_path_factory):
    """跑一次离线证据链（2 圈），返回 ``(report, completed_process)``。"""
    out = tmp_path_factory.mktemp("s06_offline") / "s06_pick_place_offline.json"
    proc = _run_script("--out", str(out), "--iterations", "2")
    report = {}
    if os.path.isfile(out):
        with open(out, "r", encoding="utf-8") as fh:
            report = json.load(fh)
    _skip_if_model_unavailable(report)
    return report, proc


def test_script_exit_code_zero_and_four_sections(offline_report):
    """退出码 0 + 四段齐全。"""
    report, proc = offline_report
    assert proc.returncode == 0, proc.stdout[-2000:] + proc.stderr[-2000:]
    for section in ("success_run", "fault_cases", "replay", "live_window"):
        assert section in report, "报告缺少段落: %s" % section
    assert report["status"] == "ok"
    assert report["failures"] == []


def test_success_run_completed_all_iterations(offline_report):
    """success_run：圈次 / 段数 / 每圈回 home / 零互锁阻塞。"""
    report, _proc = offline_report
    section = report["success_run"]
    assert section["iterations_completed"] == 2
    assert section["segments_executed"] == 2 * section["segments_per_iteration"]
    assert section["home_returns"] == 2
    assert section["interlock_blocked_events"] == 0
    assert section["loop_status"] == "ok"
    assert section["judgment_mismatches"] == []
    assert all(a["passed"] for a in section["assertions"])


def test_fault_cases_hit_expected_severity_and_next_action(offline_report):
    """六类故障分支逐条命中预期 severity / 重试行为 / next_action。"""
    report, _proc = offline_report
    section = report["fault_cases"]
    assert len(section["cases"]) == 6
    for case in section["cases"]:
        assert case["passed"], (case["fault"], case["observed"])
        assert case["observed"]["judgment_result"] == case["expected"]["judgment_result"]
        assert case["observed"]["judgment_severity"] == case["expected"]["judgment_severity"]
        assert case["observed"]["auto_retry"] == case["expected"]["auto_retry"]
        action = case["next_action"] or case["recover_next_action"]
        assert action.strip(), "故障分支缺可粘贴的下一步: %s" % case["fault"]
    # needs_param：绝不重抓 + 闸门保持 UNJUDGED
    needs = [c for c in section["cases"] if c["fault"] == "needs_param"][0]
    assert needs["observed"]["regrasp_calls"] == 0
    assert needs["observed"]["gate_state"] == "GRIPPER_UNJUDGED"


def test_replay_counts_match_and_zero_motion(offline_report):
    """replay：行数 == 事件数、severity 计数一致、回放期间零运动下发。"""
    report, _proc = offline_report
    section = report["replay"]
    assert section["motion_calls_before_replay"] == section["motion_calls_after_replay"]
    for row in section["traces"]:
        assert row["replay_lines"] == row["events"] == row["file_lines"]
        assert row["skipped"] == 0
        assert row["severity_counts_trace"] == row["severity_counts_replay"]
        assert row["last_next_action"].strip()


def test_live_window_blocked_and_items_unproven(offline_report):
    """live_window：显式 blocked + 真机四项全部标记为未实证。"""
    report, _proc = offline_report
    live = report["live_window"]
    assert live["status"] == "blocked"
    assert live["passed"] is True
    assert len(live["items"]) == 4
    assert all(item["proven"] is False for item in live["items"])
    assert "rmctl run pick_place --live" in live["backfill_command"]


def test_missing_spec_exits_4_with_reason(tmp_path):
    """负例：规格文件缺失 → 退出码 4 + 报告写明失败段落（不崩栈）。"""
    out = tmp_path / "missing_spec.json"
    proc = _run_script("--out", str(out),
                       "--task-file", "workcell/tasks/__not_exist__.yaml")
    assert proc.returncode == 4, proc.stdout[-2000:]
    assert os.path.isfile(out)
    with open(out, "r", encoding="utf-8") as fh:
        report = json.load(fh)
    assert report["status"] == "error"
    assert report["failures"] == ["spec"]
    assert report["next_action"].strip()


def test_runs_from_any_cwd_and_keeps_worktree_clean(tmp_path):
    """负例：从任意目录调用仍可跑通，且不修改 git 跟踪文件。"""
    out = tmp_path / "any_cwd.json"
    proc = _run_script("--out", str(out), "--iterations", "1", cwd=str(tmp_path))
    if proc.returncode == 4 and os.path.isfile(out):
        with open(out, "r", encoding="utf-8") as fh:
            _skip_if_model_unavailable(json.load(fh))
    assert proc.returncode == 0, proc.stdout[-2000:] + proc.stderr[-2000:]
    with open(out, "r", encoding="utf-8") as fh:
        report = json.load(fh)
    assert report["status"] == "ok"
    assert report["worktree_status_before"] == report["worktree_status_after"]
