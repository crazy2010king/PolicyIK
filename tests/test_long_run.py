"""**任务级长稳载体**（:mod:`scripts.long_run`）的行为测试（S07-T05）。

覆盖面
------
* **离线演练**（``--dry-run``）：退出码 0、``offline=true``、
  ``physical_verified=false``、``thresholds_met`` **恒为 false**
  （物理长稳未验证 ⇒ 不得报达标）、输出标注「物理长稳未验证」；
* **真机不可达**（未加 ``--dry-run``）：``SKIP(reason=hardware_unreachable)`` +
  退出码 0 + **未验证指标非空**（既不伪装成功，也不伪装失败）；
* **有鉴别力**：注入一次失败轮次 ⇒ ``thresholds_met=false`` 且
  ``threshold_failures`` **非空**（而全成功轮次时为空）——
  证明判定**对成功率敏感**，不会恒绿；
* **未验证指标唯一性**：8 h 长稳条目只出现在
  :data:`rm75_drake.tasks.metrics.UNVERIFIED_TASK_METRICS`，
  **不得**同时登记到
  :data:`rm75_drake.device.hardware_gate.UNVERIFIED_HARDWARE_METRICS`（防双写）；
* **用法错误**：非法参数 / 任务文件不存在 ⇒ 退出码 2。

脚本一律以**子进程**方式调用（真实 CLI 行为），不 monkeypatch 内部函数。
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = REPO_ROOT / "src"
SCRIPT = REPO_ROOT / "scripts" / "long_run.py"

#: 部署实例任务文件（仓库根 ``tasks/``）。
TASK_FILE = REPO_ROOT / "tasks" / "pick_place.yaml"

#: 单臂场景真相源。
SCENARIO_FILE = REPO_ROOT / "scenarios" / "single_arm.yaml"

#: 8 h 长稳未验证条目（唯一真相源 = 任务级指标模块）。
LONG_RUN_METRIC_KEY = "8 h 连续 pick-place 长稳"


def run_script(*args: str, timeout: int = 600) -> subprocess.CompletedProcess:
    """以子进程方式调用长稳载体脚本（真实 CLI 行为）。"""
    env = dict(os.environ)
    env["PYTHONPATH"] = str(SRC_DIR)
    return subprocess.run(
        [sys.executable, str(SCRIPT), *args],
        cwd=str(REPO_ROOT),
        capture_output=True,
        text=True,
        timeout=timeout,
        env=env,
    )


def parse_json(result: subprocess.CompletedProcess) -> dict:
    """解析脚本的 JSON 输出（stdout 必须**只**含 JSON）。"""
    payload = json.loads(result.stdout)
    assert isinstance(payload, dict)
    return payload


@pytest.fixture(scope="module")
def failing_task(tmp_path_factory) -> str:
    """构造一份**必然失败**的任务文件（首个原语目标落在可达包络之外）。

    失败是真的来自求解器（``stage=ik`` / ``reason=unreachable``），
    不是靠 mock —— 因此该用例证明判定链路确实对失败敏感。
    """
    data = yaml.safe_load(TASK_FILE.read_text(encoding="utf-8"))
    data["primitives"][0]["target"]["xyz"] = [0.9, 0.0, 0.9]
    target = tmp_path_factory.mktemp("long-run") / "failing_task.yaml"
    target.write_text(yaml.safe_dump(data, allow_unicode=True), encoding="utf-8")
    return str(target)


# ── 存在性 ──────────────────────────────────────────────────────────────
def test_script_exists() -> None:
    """载体脚本必须存在（8 h 长稳的入口）。"""
    assert SCRIPT.is_file(), f"{SCRIPT} 必须存在"


# ── 离线演练 ────────────────────────────────────────────────────────────
def test_dry_run_exits_zero_and_marks_physical_stability_unverified() -> None:
    """离线演练：退出码 0，且输出**必须标注**「物理长稳未验证」。"""
    result = run_script("--dry-run", "--duration-s", "5")
    combined = result.stdout + result.stderr
    assert result.returncode == 0, combined
    assert "物理长稳未验证" in result.stdout, combined


def test_dry_run_json_contract() -> None:
    """离线演练的 JSON 契约：offline / physical_verified / 门槛 / 未验证指标。"""
    result = run_script(
        "--dry-run",
        "--duration-s",
        "5",
        "--task",
        str(TASK_FILE),
        "--scenario",
        str(SCENARIO_FILE),
        "--json",
    )
    combined = result.stdout + result.stderr
    assert result.returncode == 0, combined
    payload = parse_json(result)

    assert payload["offline"] is True
    assert payload["physical_verified"] is False
    # 物理长稳未验证 ⇒ **不得**报达标（离线演练恒为 false）。
    assert payload["thresholds_met"] is False

    assert payload["cycles"] >= 1
    assert payload["success_count"] >= 1
    rate = payload["success_rate"]
    assert rate is not None and 0.0 <= rate <= 1.0
    assert payload["exceptions"] == 0

    text = "\n".join(payload["unverified_metrics"])
    assert LONG_RUN_METRIC_KEY in text, payload["unverified_metrics"]


# ── 真机不可达 ⇒ SKIP ──────────────────────────────────────────────────
def test_hardware_unreachable_skips_with_exit_zero() -> None:
    """真机不可达 ⇒ ``SKIP(reason=hardware_unreachable)`` + 退出码 0。"""
    result = run_script("--duration-s", "5")
    combined = result.stdout + result.stderr
    assert result.returncode == 0, combined
    assert "SKIP(reason=hardware_unreachable)" in result.stdout, combined
    for metric in _task_metrics():
        assert metric in result.stdout, f"未验证指标未列出：{metric}\n{combined}"


def test_hardware_unreachable_json_reports_unverified_metrics() -> None:
    """真机不可达（``--json``）：``status=skipped`` 且未验证指标非空。"""
    result = run_script("--duration-s", "5", "--json")
    combined = result.stdout + result.stderr
    assert result.returncode == 0, combined
    payload = parse_json(result)

    assert payload["status"] == "skipped"
    assert payload["reason"] == "hardware_unreachable"
    assert payload["physical_verified"] is False
    assert payload["thresholds_met"] is False
    assert payload["unverified_metrics"], "未验证指标不得为空"
    assert any(
        LONG_RUN_METRIC_KEY in item for item in payload["unverified_metrics"]
    ), payload["unverified_metrics"]
    assert "SKIP(reason=hardware_unreachable)" in payload["skip_notice"]


# ── 有鉴别力：注入失败轮次 ⇒ 门槛结论不恒绿 ─────────────────────────────
def test_injected_failure_makes_threshold_failures_nonempty(failing_task: str) -> None:
    """注入一次失败轮次 ⇒ ``thresholds_met=false`` 且 ``threshold_failures`` 非空。"""
    result = run_script(
        "--dry-run",
        "--duration-s",
        "5",
        "--task",
        failing_task,
        "--scenario",
        str(SCENARIO_FILE),
        "--json",
    )
    combined = result.stdout + result.stderr
    assert result.returncode == 0, combined
    payload = parse_json(result)

    assert payload["cycles"] >= 1
    assert payload["success_count"] == 0
    assert payload["success_rate"] == 0.0
    assert payload["aborted"] is True
    assert payload["thresholds_met"] is False
    assert payload["threshold_failures"], "失败轮次必须产出非空 threshold_failures"
    assert payload["failures"] and payload["failures"][0]["stage"] == "ik"


def test_threshold_verdict_is_sensitive_to_success_rate(failing_task: str) -> None:
    """同一门槛下：全成功 ⇒ 无门槛失败；注入失败 ⇒ 有门槛失败（不会恒绿）。"""
    good = run_script(
        "--dry-run",
        "--duration-s",
        "5",
        "--task",
        str(TASK_FILE),
        "--scenario",
        str(SCENARIO_FILE),
        "--json",
    )
    bad = run_script(
        "--dry-run",
        "--duration-s",
        "5",
        "--task",
        failing_task,
        "--scenario",
        str(SCENARIO_FILE),
        "--json",
    )
    assert good.returncode == 0, good.stdout + good.stderr
    assert bad.returncode == 0, bad.stdout + bad.stderr

    good_payload = parse_json(good)
    bad_payload = parse_json(bad)

    assert good_payload["success_rate"] > bad_payload["success_rate"]
    assert good_payload["threshold_failures"] == []
    assert bad_payload["threshold_failures"]


def test_threshold_verdict_has_two_independent_dimensions(failing_task: str) -> None:
    """门槛判定有**两个独立维度**：成功率门槛 + 异常停机。

    * 全成功轮次 ⇒ 即使门槛抬到 1.0 也不产生门槛失败（门槛参数被尊重）；
    * 注入失败轮次且把成功率门槛降到 0.0 ⇒ 仍然因为**异常停机**判失败
      （成功率维度被关掉也逃不掉）。
    """
    good = run_script(
        "--dry-run",
        "--duration-s",
        "5",
        "--min-success-rate",
        "1.0",
        "--task",
        str(TASK_FILE),
        "--scenario",
        str(SCENARIO_FILE),
        "--json",
    )
    bad = run_script(
        "--dry-run",
        "--duration-s",
        "5",
        "--min-success-rate",
        "0.0",
        "--task",
        failing_task,
        "--scenario",
        str(SCENARIO_FILE),
        "--json",
    )
    assert good.returncode == 0, good.stdout + good.stderr
    assert bad.returncode == 0, bad.stdout + bad.stderr

    good_payload = parse_json(good)
    bad_payload = parse_json(bad)

    assert good_payload["success_rate"] == 1.0
    assert good_payload["threshold_failures"] == []

    # 成功率门槛降到 0.0 ⇒ 成功率维度不再触发，但异常停机仍判失败。
    assert bad_payload["success_rate"] == 0.0
    assert bad_payload["aborted"] is True
    assert bad_payload["threshold_failures"]
    assert not any("成功率" in item for item in bad_payload["threshold_failures"])


# ── 未验证指标唯一性（禁止双写）────────────────────────────────────────
def test_long_run_metric_registered_only_in_task_metrics() -> None:
    """8 h 长稳条目只在**任务级**清单里，不得双写进硬件级清单。"""
    sys.path.insert(0, str(SRC_DIR))
    from rm75_drake.device.hardware_gate import UNVERIFIED_HARDWARE_METRICS
    from rm75_drake.tasks.metrics import (
        UNVERIFIED_TASK_METRICS,
        UNVERIFIED_TASK_METRIC_LONG_RUN_8H,
    )

    assert LONG_RUN_METRIC_KEY in UNVERIFIED_TASK_METRIC_LONG_RUN_8H
    assert UNVERIFIED_TASK_METRIC_LONG_RUN_8H in UNVERIFIED_TASK_METRICS
    # 防双写：硬件级清单里**不得**出现 8 h 任务级长稳条目。
    for item in UNVERIFIED_HARDWARE_METRICS:
        assert LONG_RUN_METRIC_KEY not in item, f"硬件级清单双写了任务级条目：{item}"


def test_unverified_task_metrics_entries_have_unlock_conditions() -> None:
    """每条未验证指标都必须写明「未验证」与**解锁条件**。"""
    sys.path.insert(0, str(SRC_DIR))
    from rm75_drake.tasks.metrics import UNVERIFIED_TASK_METRICS

    assert len(UNVERIFIED_TASK_METRICS) >= 7
    for item in UNVERIFIED_TASK_METRICS:
        assert "未验证" in item, item
        assert "解锁条件" in item, item


# ── 用法错误（退出码 2）────────────────────────────────────────────────
@pytest.mark.parametrize(
    "args",
    [
        ("--dry-run", "--duration-s", "0"),
        ("--dry-run", "--duration-s", "-1"),
        ("--dry-run", "--max-cycles", "0"),
        ("--dry-run", "--min-success-rate", "1.5"),
        ("--dry-run", "--task", "does/not/exist.yaml"),
        ("--dry-run", "--scenario", "does/not/exist.yaml"),
    ],
    ids=[
        "duration-zero",
        "duration-negative",
        "max-cycles-zero",
        "min-success-rate-out-of-range",
        "task-missing",
        "scenario-missing",
    ],
)
def test_usage_errors_exit_two(args: tuple) -> None:
    """非法参数 / 文件缺失 ⇒ 退出码 2（不得被当成「演练通过」）。"""
    result = run_script(*args)
    assert result.returncode == 2, result.stdout + result.stderr


# ── 辅助 ────────────────────────────────────────────────────────────────
def _task_metrics() -> list:
    """任务级未验证指标清单（真相源 = 任务级指标模块）。"""
    sys.path.insert(0, str(SRC_DIR))
    from rm75_drake.tasks.metrics import UNVERIFIED_TASK_METRICS

    return list(UNVERIFIED_TASK_METRICS)
