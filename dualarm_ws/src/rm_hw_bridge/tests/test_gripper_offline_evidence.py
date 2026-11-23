"""S05 **离线证据链脚本**的回归测试（`scripts/gripper_offline_evidence.py`）。

守住的契约：

* 脚本**可复跑**且退出码为 0（三段断言全过）；
* 报告顶层是既有口径（``status`` / ``reason`` / ``next_action`` / ``evidence`` / ``mode``），
  且 ``mode=fake``；
* **每一段**（``cycle`` / ``judgment`` / ``interlock``）的 evidence 都含
  「mode=fake / 代理模型，不代表真机结论」字样——离线数字**不得**冒充真机结论；
* 真机回填入口在报告里（``live_window``）且状态为 ``blocked``；
* **失败路径**：任一段断言失败 → 退出码 **4**、报告 ``status=error``；
* 互锁段必须留下**零运动下发**证据（``preposition=0`` 且 ``transmit=0``）。

只使用 git 跟踪的仓库文件 + pytest 临时目录；
报告写到 pytest 的 ``tmp_path``（**不**写进仓库的 ``build/``）。
"""
from __future__ import annotations

import importlib.util
import json
import os
import subprocess
import sys

FAKE_MARKERS = ("mode=fake", "代理模型，不代表真机结论")


def _script_path(repo_root: str) -> str:
    return os.path.join(repo_root, "scripts", "gripper_offline_evidence.py")


def _load_script(repo_root: str):
    """按文件路径加载脚本模块（``scripts/`` 不是包）。"""
    path = _script_path(repo_root)
    spec = importlib.util.spec_from_file_location("gripper_offline_evidence", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _run_script(repo_root: str, tmp_path, *extra):
    out = os.path.join(str(tmp_path), "s05_offline_gripper.json")
    proc = subprocess.run(
        [sys.executable, _script_path(repo_root), "--out", out] + list(extra),
        cwd=repo_root, capture_output=True, text=True,
    )
    report = None
    if os.path.isfile(out):
        with open(out, "r", encoding="utf-8") as fh:
            report = json.load(fh)
    return proc, report


# --------------------------------------------------------------------- 正例
def test_script_runs_and_exits_zero(repo_root, tmp_path):
    """脚本可复跑：退出码 0，报告落盘且顶层状态为 ok。"""
    proc, report = _run_script(repo_root, tmp_path)
    assert proc.returncode == 0, proc.stdout + proc.stderr
    assert report is not None
    assert report["status"] == "ok", report
    assert report["mode"] == "fake"
    for key in ("status", "reason", "next_action", "evidence", "mode"):
        assert key in report, key


def test_report_has_three_sections_with_fake_marker(repo_root, tmp_path):
    """三段明细齐全，且**每段** evidence 都带「mode=fake / 代理模型」字样。"""
    proc, report = _run_script(repo_root, tmp_path)
    assert proc.returncode == 0, proc.stdout + proc.stderr
    for section in ("cycle", "judgment", "interlock"):
        assert section in report, section
        joined = " ".join(report[section].get("evidence", []))
        for marker in FAKE_MARKERS:
            assert marker in joined, (section, joined)
        assert report[section]["status"] == "ok", report[section]


def test_cycle_section_has_100_cycles_zero_timeout(repo_root, tmp_path):
    """开合 100 次：零超时、零失败，且超时检测自检通过。"""
    proc, report = _run_script(repo_root, tmp_path)
    assert proc.returncode == 0, proc.stdout + proc.stderr
    cycle = report["cycle"]
    assert cycle["cycles"] == 100
    assert cycle["actions"] == 200
    assert cycle["timeouts"] == 0
    assert cycle["failure_count"] == 0
    # 超时计数器不是恒零的空断言
    assert cycle["timeout_detector_selfcheck"] is True


def test_judgment_section_is_six_faults_x_10(repo_root, tmp_path):
    """六类判定各 10 次全对，且负例（标定为 null → needs_param）通过。"""
    proc, report = _run_script(repo_root, tmp_path)
    assert proc.returncode == 0, proc.stdout + proc.stderr
    judgment = report["judgment"]
    faults = {row["fault"]: row for row in judgment["faults"]}
    assert set(faults) == {"grasp_ok", "grasp_empty", "stall", "overtemp",
                           "overcurrent", "offline"}
    for fault, row in faults.items():
        assert row["injected"] == 10, row
        assert row["correct"] == 10, row
        assert row["severity_mismatch"] == 0, row
        assert row["passed"] is True, row
    assert judgment["mismatches"] == 0
    # 负例：标定系数为 null → needs_param，且不得产出夹到/夹空
    negative = judgment["negative"]
    assert negative["result"] == "needs_param"
    assert negative["severity"] == "NEEDS_PARAM"
    assert negative["passed"] is True
    # 红线：恢复阶梯里不得出现「写 0x0008 清错误码」
    assert judgment["recovery_ladder_violations"] == []


def test_interlock_section_shows_zero_motion(repo_root, tmp_path):
    """互锁段：闸门非 IDLE → 事件存在且 **零运动下发**。"""
    proc, report = _run_script(repo_root, tmp_path)
    assert proc.returncode == 0, proc.stdout + proc.stderr
    cases = report["interlock"]["cases"]
    assert {case["gate_state"] for case in cases} == {"GRIPPER_BUSY", "GRIPPER_UNJUDGED"}
    for case in cases:
        assert case["blocked_events"] == 1, case
        assert case["severity"] == "NEEDS_HUMAN", case
        assert case["report_status"] == "blocked", case
        assert case["preposition_calls"] == 0, case
        assert case["transmit_calls"] == 0, case


def test_report_declares_live_window_blocked(repo_root, tmp_path):
    """真机窗口未实证：报告显式声明 blocked 并给出回填入口。"""
    proc, report = _run_script(repo_root, tmp_path)
    assert proc.returncode == 0, proc.stdout + proc.stderr
    live = report["live_window"]
    assert live["status"] == "blocked"
    assert "未实证" in live["reason"]
    assert live["next_action"]
    joined = " ".join(report["evidence"])
    assert "未实证" in joined


# --------------------------------------------------------------------- 负例
def test_failed_section_yields_exit_code_4(repo_root, tmp_path, monkeypatch):
    """任一段断言失败 → 退出码 4、报告 status=error（不是静默通过）。"""
    module = _load_script(repo_root)

    def _broken_cycle(config, thresholds, count):
        return {
            "status": "error",
            "reason": "人为注入的失败",
            "next_action": "",
            "evidence": ["mode=fake（离线假夹爪，代理模型，不代表真机结论）"],
            "mode": "fake",
            "assertions": [],
        }

    monkeypatch.setattr(module, "run_cycle", _broken_cycle)
    out = os.path.join(str(tmp_path), "broken.json")
    code = module.main(["--out", out])
    assert code == 4
    with open(out, "r", encoding="utf-8") as fh:
        report = json.load(fh)
    assert report["status"] == "error"
    assert "cycle" in report["failures"]


def test_script_rejects_unknown_judge_count_argument(repo_root):
    """未知参数 → argparse 以退出码 2 失败（不静默产出报告）。"""
    proc = subprocess.run(
        [sys.executable, _script_path(repo_root), "--no-such-flag"],
        cwd=repo_root, capture_output=True, text=True,
    )
    assert proc.returncode == 2, proc.stdout + proc.stderr
