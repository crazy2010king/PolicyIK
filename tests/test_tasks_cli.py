"""CLI ``task`` 子命令的测试（S06-T07）。

覆盖面
------
* **退出码 0**：整条原语链跑通且逐原语门槛达标；
* **退出码 1**：业务失败（原语失败 / 超门槛）；
* **退出码 2**：用法错误（任务文件不存在 / schema 非法 / 参数非法）；
* ``--json``：**单行** JSON，含逐原语结果、门槛与**未验证指标**；
* 真机物理指标**硬件门控 SKIP**：逐条列入 ``unverified_metrics``，
  但**不**影响退出码 0（既不伪装成功，也不伪装失败）。

零硬件：夹爪命令下发到 :class:`rm75_drake.device.transport.ScriptedTransport` 替身。
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = REPO_ROOT / "src"

from rm75_drake import cli as cli_module
from rm75_drake.cli import EXIT_FAILURE, EXIT_OK, EXIT_USAGE, main
from rm75_drake.tasks.metrics import UNVERIFIED_TASK_METRICS

#: 部署实例任务文件（仓库根 ``tasks/``）。
TASK_FILE = REPO_ROOT / "tasks" / "pick_place.yaml"

#: 单臂场景真相源。
SCENARIO_FILE = REPO_ROOT / "scenarios" / "single_arm.yaml"

#: 原语级指标统计脚本。
STATS_SCRIPT = REPO_ROOT / "scripts" / "task_stats.py"


# ── 退出码 0：成功 ───────────────────────────────────────────────────────
def test_task_command_succeeds_and_emits_single_line_json(capsys) -> None:
    """整条链跑通 ⇒ 退出码 0，``--json`` 输出**单行** JSON。"""
    code = main(
        [
            "task",
            "--task",
            str(TASK_FILE),
            "--scenario",
            str(SCENARIO_FILE),
            "--json",
        ]
    )
    captured = capsys.readouterr()
    assert code == EXIT_OK, captured.err

    lines = [line for line in captured.out.splitlines() if line.strip()]
    assert len(lines) == 1, captured.out
    payload = json.loads(lines[0])

    assert payload["command"] == "task"
    assert payload["status"] == "ok"
    assert payload["success"] is True
    assert payload["stage"] == "done"
    assert payload["reason"] == "completed"

    # 逐原语结果
    names = [item["name"] for item in payload["primitive_results"]]
    assert names == [
        "approach",
        "pre_grasp",
        "close",
        "verify",
        "lift",
        "transit",
        "place",
        "open",
        "retreat",
    ]
    assert all(item["success"] for item in payload["primitive_results"])

    # 门槛（验收口径）
    acceptance = payload["acceptance_metrics"]
    assert acceptance["thresholds_met"] is True
    assert acceptance["thresholds"]["success_rate"] == 0.95
    assert set(acceptance["primitives"]) == set(names)

    # 端到端口径**单列**
    end_to_end = payload["end_to_end_metrics"]
    assert "per_sample_duration_s" in end_to_end
    assert "primitives" not in end_to_end

    # 未验证指标：逐条列出（硬件门控 SKIP，但退出码仍是 0）
    assert payload["unverified_metrics"] == list(UNVERIFIED_TASK_METRICS)
    assert payload["hardware_gate"]["status"] == "SKIP"


def test_task_command_human_readable_output_lists_unverified_metrics(capsys) -> None:
    """非 JSON 输出：验收口径 / 端到端口径**分行**，并逐条列出未验证指标。"""
    code = main(
        ["task", "--task", str(TASK_FILE), "--scenario", str(SCENARIO_FILE)]
    )
    captured = capsys.readouterr()
    assert code == EXIT_OK, captured.err

    assert "[验收口径]" in captured.out
    assert "[端到端]" in captured.out
    assert "未验证" in captured.out
    for item in UNVERIFIED_TASK_METRICS:
        assert item in captured.out


# ── 退出码 1：业务失败 ──────────────────────────────────────────────────
def test_task_command_reports_business_failure(tmp_path, capsys) -> None:
    """原语失败（目标不可达）⇒ 退出码 1。"""
    task_file = tmp_path / "unreachable.yaml"
    task_file.write_text(
        "\n".join(
            [
                "schema_version: 1",
                "name: unreachable_case",
                "robot: left",
                "primitives:",
                "  - name: approach",
                "    kind: motion",
                "    target:",
                "      xyz: [1.20, 0.0, 0.90]",
                "      rpy: [0.0, 0.0, 0.0]",
            ]
        ),
        encoding="utf-8",
    )

    code = main(
        [
            "task",
            "--task",
            str(task_file),
            "--scenario",
            str(SCENARIO_FILE),
            "--json",
        ]
    )
    captured = capsys.readouterr()
    assert code == EXIT_FAILURE, captured.out

    payload = json.loads(captured.out.strip().splitlines()[-1])
    assert payload["status"] == "failed"
    assert payload["success"] is False
    assert payload["stage"] == "ik"
    assert payload["reason"] == "unreachable"
    # 未验证指标**照样**逐条列出
    assert payload["unverified_metrics"] == list(UNVERIFIED_TASK_METRICS)


def test_task_command_fails_when_threshold_is_not_met(monkeypatch, capsys) -> None:
    """门槛不达标 ⇒ 退出码 1（stage=metrics / reason=threshold_not_met）。

    这里用**替身**注入一份「指标不达标」的报告（指标口径本身的判定由
    ``tests/test_tasks_metrics.py`` 覆盖），验证 CLI 的**退出码映射**。
    """
    from types import SimpleNamespace

    from rm75_drake.tasks.metrics import build_task_metrics_report

    failed_sample = SimpleNamespace(
        results=(
            SimpleNamespace(
                name="approach",
                success=False,
                stage="ik",
                reason="unreachable",
                position_error_m=None,
                orientation_error_rad=None,
            ),
        ),
        success=False,
        duration_s=0.1,
    )

    def fake_run_task_chain(task_path, **kwargs):
        report = build_task_metrics_report([failed_sample], task_name="forced_failure")
        return {
            "task": str(task_path),
            "task_name": report.task_name,
            "scenario": "n/a",
            "samples_requested": 1,
            "samples_completed": 1,
            "primitive_results": [
                {
                    "name": "approach",
                    "success": False,
                    "stage": "ik",
                    "reason": "unreachable",
                    "position_error_m": None,
                    "orientation_error_rad": None,
                }
            ],
            "task_results": [],
            "metrics": report.as_dict(),
            "report_object": report,
            "scene_assembly_s": 0.0,
            "wall_time_s": 0.0,
        }

    monkeypatch.setattr(cli_module, "run_task_chain", fake_run_task_chain)

    code = main(["task", "--task", str(TASK_FILE), "--json"])
    captured = capsys.readouterr()
    assert code == EXIT_FAILURE, captured.out

    payload = json.loads(captured.out.strip().splitlines()[-1])
    assert payload["success"] is False
    assert payload["stage"] == "metrics"
    assert payload["reason"] == "threshold_not_met"
    assert payload["acceptance_metrics"]["thresholds_met"] is False


# ── 退出码 2：用法错误 ──────────────────────────────────────────────────
def test_task_command_missing_task_file_is_usage_error(capsys) -> None:
    """任务文件不存在 ⇒ 退出码 2。"""
    code = main(["task", "--task", str(REPO_ROOT / "tasks" / "does_not_exist.yaml")])
    captured = capsys.readouterr()
    assert code == EXIT_USAGE
    assert "任务文件不存在" in captured.err


def test_task_command_invalid_schema_is_usage_error(tmp_path, capsys) -> None:
    """任务文件 schema 非法（未知键）⇒ 退出码 2。"""
    task_file = tmp_path / "broken.yaml"
    task_file.write_text(
        "\n".join(
            [
                "schema_version: 1",
                "name: broken_case",
                "robot: left",
                "unknown_top_level_key: 1",
                "primitives: []",
            ]
        ),
        encoding="utf-8",
    )

    code = main(["task", "--task", str(task_file), "--scenario", str(SCENARIO_FILE)])
    captured = capsys.readouterr()
    assert code == EXIT_USAGE
    assert "参数错误" in captured.err


def test_task_command_invalid_samples_argument_is_usage_error(capsys) -> None:
    """``--samples 0`` ⇒ 退出码 2。"""
    code = main(["task", "--task", str(TASK_FILE), "--samples", "0"])
    captured = capsys.readouterr()
    assert code == EXIT_USAGE
    assert "参数错误" in captured.err


def test_task_command_missing_scenario_is_usage_error(tmp_path, capsys) -> None:
    """场景文件不存在 ⇒ 退出码 2。"""
    code = main(
        [
            "task",
            "--task",
            str(TASK_FILE),
            "--scenario",
            str(REPO_ROOT / "scenarios" / "does_not_exist.yaml"),
        ]
    )
    captured = capsys.readouterr()
    assert code == EXIT_USAGE
    assert "参数错误" in captured.err


# ── 统计脚本 ────────────────────────────────────────────────────────────
def _run_stats(*args: str) -> subprocess.CompletedProcess:
    """跑统计脚本（子进程，零硬件）。"""
    env = dict(os.environ)
    env["PYTHONPATH"] = str(SRC_DIR)
    return subprocess.run(
        [sys.executable, str(STATS_SCRIPT), *args],
        cwd=str(REPO_ROOT),
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )


def test_task_stats_script_reports_three_metric_sections() -> None:
    """统计脚本：JSON 含 **acceptance / end_to_end / unverified** 三段，退出码 0。"""
    completed = _run_stats("--samples", "2", "--json")
    assert completed.returncode == 0, completed.stderr

    payload = json.loads(completed.stdout)
    assert "acceptance_metrics" in payload
    assert "end_to_end_metrics" in payload
    assert "unverified_metrics" in payload
    assert payload["unverified_metrics"] == list(UNVERIFIED_TASK_METRICS)
    # 真机物理指标硬件门控 SKIP ⇒ 退出码仍是 0
    assert payload["hardware_gate"]["status"] == "SKIP"
    assert payload["thresholds_met"] is True


def test_task_stats_script_time_budget_stops_early() -> None:
    """``--max-seconds`` 预算耗尽 ⇒ 提前结束并标记部分样本（过载保护）。"""
    completed = _run_stats("--samples", "500", "--max-seconds", "1", "--json")
    assert completed.returncode == 0, completed.stderr

    payload = json.loads(completed.stdout)
    assert payload["truncated"] is True
    assert payload["samples_completed"] < payload["samples_requested"]


def test_task_stats_script_missing_task_is_usage_error() -> None:
    """任务文件不存在 ⇒ 退出码 2（用法错误）。"""
    completed = _run_stats("--task", str(REPO_ROOT / "tasks" / "nope.yaml"))
    assert completed.returncode == 2
    assert "任务文件不存在" in completed.stderr


# ── 子命令本身存在 ──────────────────────────────────────────────────────
def test_task_subcommand_is_registered() -> None:
    """``task`` 子命令已注册（``--help`` 不报错）。"""
    parser = cli_module.build_parser()
    args = parser.parse_args(["task", "--task", str(TASK_FILE)])
    assert args.command == "task"
    assert args.json is False
