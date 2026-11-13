"""硬件门控测量脚本的行为测试（S03-T08）。

这些断言守住的契约
------------------
* **真机不可达 ⇒ 退出码 0 + ``SKIP(reason=hardware_unreachable)`` +
  逐条未验证指标清单**（既不伪装成失败，也不伪装成成功）；
* **未验证指标清单只有一份**（真相源 = ``device/hardware_gate.py``，
  与 CLI 的 ``exec`` 子命令完全一致）；
* **离线演练（``--dry-run``）永远标注「不代表真机指标」**，且仍列出
  未验证指标（不得因为跑了个替身就把真机指标说成已验证）；
* 故障注入脚本的**四类故障**都必须走完「注入 → 安全态 → 恢复 → READY」；
* **负例**：未知参数 / 缺设备配置 / 非法位姿 → 退出码 2（用法错误），
  不得被当成「SKIP 成功」。

脚本一律以**子进程**方式调用（真实 CLI 行为），不 monkeypatch 内部函数。
"""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = REPO_ROOT / "scripts"
DEVICE_CONFIG = REPO_ROOT / "devices" / "single_arm.yaml"

MEASURE_JITTER = SCRIPTS_DIR / "measure_jitter.py"
SOAK_TEST = SCRIPTS_DIR / "soak_test.py"
FAULT_INJECT = SCRIPTS_DIR / "fault_inject.py"

#: 三个脚本都必须存在（缺一个 ⇒ 硬件门控入口不完整）。
ALL_SCRIPTS = (MEASURE_JITTER, SOAK_TEST, FAULT_INJECT)

#: 四类故障的人类可读标签（脚本输出里必须逐条出现）。
FAULT_LABELS = ("拔网线", "物体掉落", "夹爪夹空", "关节超限位")


def run_script(script: Path, *args: str, timeout: int = 300) -> subprocess.CompletedProcess:
    """以子进程方式调用脚本（真实 CLI 行为）。"""
    return subprocess.run(
        [sys.executable, str(script), *args],
        cwd=str(REPO_ROOT),
        capture_output=True,
        text=True,
        timeout=timeout,
    )


@pytest.fixture(scope="module")
def unverified_metrics() -> tuple:
    """未验证指标清单（真相源 = 设备层的硬件门控模块）。"""
    sys.path.insert(0, str(REPO_ROOT / "src"))
    from rm75_drake.device.hardware_gate import UNVERIFIED_HARDWARE_METRICS

    return tuple(UNVERIFIED_HARDWARE_METRICS)


@pytest.fixture(scope="module")
def unreachable_config(tmp_path_factory) -> str:
    """构造一份**必然不可达**的设备配置（127.0.0.1:1 ⇒ 连接被拒）。

    这样即便测试机上装了厂商 SDK，脚本也只会走 SKIP 分支——
    测试不依赖「测试机上恰好没有机械臂」这一偶然事实。
    """
    data = yaml.safe_load(DEVICE_CONFIG.read_text(encoding="utf-8"))
    data["arm_ip"] = "127.0.0.1"
    data["tcp_port"] = 1
    target = tmp_path_factory.mktemp("device-config") / "unreachable.yaml"
    target.write_text(yaml.safe_dump(data, allow_unicode=True), encoding="utf-8")
    return str(target)


# --------------------------------------------------------------------------- #
# 存在性
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("script", ALL_SCRIPTS, ids=lambda p: p.name)
def test_script_exists(script):
    assert script.is_file(), f"{script} 必须存在（硬件门控入口）"


# --------------------------------------------------------------------------- #
# 正例：无硬件 ⇒ SKIP + 未验证指标清单 + 退出码 0
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("script", ALL_SCRIPTS, ids=lambda p: p.name)
def test_script_skips_when_hardware_unreachable(
    script, unreachable_config, unverified_metrics
):
    """真机不可达 ⇒ 退出码 0，并输出 SKIP 与逐条未验证指标。"""
    result = run_script(script, "--device-config", unreachable_config)
    combined = result.stdout + result.stderr
    assert result.returncode == 0, combined
    assert "SKIP(reason=hardware_unreachable)" in result.stdout, combined
    for metric in unverified_metrics:
        assert metric in result.stdout, f"未验证指标未列出：{metric}\n{combined}"


def test_measure_jitter_skip_reports_jitter_metric(unreachable_config):
    """抖动脚本必须把「抖动」列进未验证指标（它自己没测到）。"""
    result = run_script(MEASURE_JITTER, "--device-config", unreachable_config)
    assert result.returncode == 0
    assert "5 ms 透传周期抖动" in result.stdout


def test_fault_inject_marks_physical_faults_unverified(unreachable_config):
    """故障脚本必须标注「物理故障未验证」（只演练了软件流程）。"""
    result = run_script(FAULT_INJECT, "--device-config", unreachable_config)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "物理故障未验证" in result.stdout


# --------------------------------------------------------------------------- #
# 真相源一致性（禁止第二份清单）
# --------------------------------------------------------------------------- #
def test_unverified_metrics_match_cli_truth_source(unverified_metrics):
    """CLI 的未验证指标清单必须与设备层门控模块**完全一致**。"""
    sys.path.insert(0, str(REPO_ROOT / "src"))
    from rm75_drake.cli import UNVERIFIED_HARDWARE_METRICS

    assert tuple(UNVERIFIED_HARDWARE_METRICS) == tuple(unverified_metrics)


def test_cli_probe_and_gate_share_implementation():
    """CLI 的硬件探测必须复用设备层的门控实现（禁止第二份探测逻辑）。"""
    cli_text = (REPO_ROOT / "src" / "rm75_drake" / "cli.py").read_text(encoding="utf-8")
    assert "from .device.hardware_gate import" in cli_text
    assert "probe_hardware(device_config" in cli_text


# --------------------------------------------------------------------------- #
# 离线演练（替身）：跑得通、且**不代表真机指标**
# --------------------------------------------------------------------------- #
def test_measure_jitter_dry_run_rehearsal_reports_histogram():
    """离线演练：跑出抖动直方图（口径与单测同源），并列出未验证指标。"""
    result = run_script(
        MEASURE_JITTER,
        "--dry-run",
        "--duration-s",
        "0.05",
        "--device-config",
        str(DEVICE_CONFIG),
    )
    combined = result.stdout + result.stderr
    assert result.returncode == 0, combined
    assert "抖动（ms）" in result.stdout
    assert "p99.9=" in result.stdout
    assert "未验证" in result.stdout


def test_measure_jitter_dry_run_json_summary_shape():
    """离线演练的 JSON 摘要字段固定（可被机器消费）。"""
    result = run_script(
        MEASURE_JITTER,
        "--dry-run",
        "--duration-s",
        "0.05",
        "--json",
        "--device-config",
        str(DEVICE_CONFIG),
    )
    assert result.returncode == 0, result.stdout + result.stderr
    payload = json.loads(result.stdout)
    assert payload["mode"] == "dry_run"
    assert payload["status"] == "ok"
    assert payload["jitter_ms"]["count"] > 0
    for key in ("p50", "p99", "p999", "max", "mean", "min"):
        assert key in payload["jitter_ms"]
    assert payload["unverified_metrics"]


def test_soak_test_dry_run_rehearsal():
    """离线演练：长稳脚本能跑完并给出拍数 / 违规数。"""
    result = run_script(
        SOAK_TEST,
        "--dry-run",
        "--duration-s",
        "0.05",
        "--device-config",
        str(DEVICE_CONFIG),
    )
    combined = result.stdout + result.stderr
    assert result.returncode == 0, combined
    assert "拍数" in result.stdout
    assert "安全违规数" in result.stdout


def test_soak_test_rejects_non_positive_duration():
    """负例：非法时长 → 退出码 2（用法错误不得被当成 SKIP）。"""
    result = run_script(
        SOAK_TEST, "--dry-run", "--duration-s", "0", "--device-config", str(DEVICE_CONFIG)
    )
    assert result.returncode == 2, result.stdout + result.stderr


def test_fault_inject_offline_rehearsal_covers_four_faults():
    """离线演练：四类故障逐条「注入 → 安全态 → 恢复 → READY」全绿。"""
    result = run_script(
        FAULT_INJECT, "--json", "--device-config", str(DEVICE_CONFIG)
    )
    combined = result.stdout + result.stderr
    assert result.returncode == 0, combined
    payload = json.loads(result.stdout)
    assert payload["physical_faults_verified"] is False
    assert len(payload["cases"]) == 4
    for case in payload["cases"]:
        assert case["ok"] is True, case
        assert case["recovered"] is True, case
        assert case["final_state"] == "ready", case
    assert payload["failed"] == []


def test_fault_inject_offline_rehearsal_lists_all_fault_labels():
    """四类故障的标签必须逐条出现在输出里（便于人工核对）。"""
    result = run_script(FAULT_INJECT, "--device-config", str(DEVICE_CONFIG))
    assert result.returncode == 0, result.stdout + result.stderr
    for label in FAULT_LABELS:
        assert label in result.stdout, f"故障标签未出现：{label}"


def test_fault_inject_only_single_case():
    """``--only`` 只演练一类故障（其余不出现）。"""
    result = run_script(
        FAULT_INJECT,
        "--json",
        "--only",
        "link_down",
        "--device-config",
        str(DEVICE_CONFIG),
    )
    assert result.returncode == 0, result.stdout + result.stderr
    payload = json.loads(result.stdout)
    assert len(payload["cases"]) == 1
    assert payload["cases"][0]["key"] == "link_down"


# --------------------------------------------------------------------------- #
# 负例：用法错误 ⇒ 退出码 2
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("script", ALL_SCRIPTS, ids=lambda p: p.name)
def test_unknown_argument_exits_two(script, unreachable_config):
    """负例：未知参数 → 退出码 2。"""
    result = run_script(script, "--no-such-option")
    assert result.returncode == 2, result.stdout + result.stderr


@pytest.mark.parametrize("script", ALL_SCRIPTS, ids=lambda p: p.name)
def test_missing_device_config_exits_two(script, tmp_path):
    """负例：设备配置不存在 → 退出码 2（不是 SKIP）。"""
    missing = str(tmp_path / "nope.yaml")
    result = run_script(script, "--device-config", missing)
    combined = result.stdout + result.stderr
    assert result.returncode == 2, combined
    assert "SKIP(reason=hardware_unreachable)" not in result.stdout


@pytest.mark.parametrize("script", (MEASURE_JITTER, SOAK_TEST), ids=lambda p: p.name)
def test_invalid_joint_rad_exits_two(script, unreachable_config):
    """负例：位姿个数不对 → 退出码 2。"""
    result = run_script(
        script, "--device-config", unreachable_config, "--joint-rad", "0 0 0"
    )
    combined = result.stdout + result.stderr
    assert result.returncode == 2, combined


@pytest.mark.parametrize("script", (MEASURE_JITTER, SOAK_TEST), ids=lambda p: p.name)
def test_non_numeric_joint_rad_exits_two(script, unreachable_config):
    """负例：位姿含非数值 → 退出码 2（点名问题）。"""
    result = run_script(
        script,
        "--device-config",
        unreachable_config,
        "--joint-rad",
        "0 0 0 x 0 0 0",
    )
    combined = result.stdout + result.stderr
    assert result.returncode == 2, combined
    assert "joint-rad" in combined
