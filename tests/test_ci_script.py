"""``scripts/ci.sh`` 的行为测试（T05）。

这些断言守住的契约：

* ``scripts/ci.sh`` 存在且 ``bash -n`` 语法检查通过；
* ``--help`` 输出里三档（fast / medium / nightly）齐全，并说明耗时预期；
* **fast 档在当前仓库可跑通**（退出码 0）——CI 必须是"能跑的检查"，
  而不是一份躺在仓库里的空壳脚本；
* 未知档位必须以退出码 2 显式失败（参数错误不得被当成通过）。

负例同样重要：坏档位不得静默返回 0。
"""

from __future__ import annotations

import subprocess
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
CI_SCRIPT = REPO_ROOT / "scripts" / "ci.sh"


def run_ci(*args: str, timeout: int = 1800) -> subprocess.CompletedProcess:
    """以子进程方式调用 ci.sh（真实 CLI 行为）。"""
    return subprocess.run(
        ["bash", str(CI_SCRIPT), *args],
        cwd=str(REPO_ROOT),
        capture_output=True,
        text=True,
        timeout=timeout,
    )


def test_ci_script_exists():
    assert CI_SCRIPT.is_file(), "scripts/ci.sh 必须存在（CI 的唯一实现）"


def test_ci_script_syntax_is_valid():
    """bash -n 语法检查必须通过。"""
    result = subprocess.run(
        ["bash", "-n", str(CI_SCRIPT)],
        cwd=str(REPO_ROOT),
        capture_output=True,
        text=True,
        timeout=60,
    )
    assert result.returncode == 0, result.stderr


def test_ci_help_lists_three_tiers():
    """--help 必须列出 fast / medium / nightly 三档。"""
    result = run_ci("--help", timeout=60)
    assert result.returncode == 0, result.stdout + result.stderr
    for tier in ("fast", "medium", "nightly"):
        assert tier in result.stdout


def test_ci_unknown_tier_exits_two():
    """负例：未知档位 → 退出码 2（参数错误不得被当成通过）。"""
    result = run_ci("no-such-tier", timeout=60)
    assert result.returncode == 2
    assert "fast" in (result.stdout + result.stderr)


def test_ci_fast_tier_passes_in_current_repo():
    """fast 档在当前仓库必须跑通（退出码 0），并输出 PASS/FAIL 摘要。"""
    result = run_ci("fast")
    combined = result.stdout + result.stderr
    assert result.returncode == 0, combined
    assert "[PASS]" in combined
    assert "[FAIL]" not in combined
    assert "汇总" in combined


def test_ci_fast_tier_runs_model_regression_and_model_check():
    """fast 档必须纳入模型回归与 verify_env --model（防止模型悄悄漂移）。"""
    result = run_ci("fast")
    combined = result.stdout + result.stderr
    assert result.returncode == 0, combined
    assert "tests/model" in combined
    assert "model/verify-env" in combined
    assert "assets/visual-mesh" in combined
    try:
        import pydrake  # noqa: F401
    except ImportError:
        return  # 环境无 Drake：模型项按设计 SKIP
    # 模型本体零外部资产依赖 → Drake 可用时模型回归必须恒跑，不得 SKIP
    assert "[SKIP] tests/model" not in combined
    assert "[SKIP] model/verify-env" not in combined


def test_ci_usage_mentions_model_regression():
    """--help 必须说明 fast 档已含模型回归（避免"以为没跑"）。"""
    result = run_ci("--help", timeout=60)
    assert result.returncode == 0
    assert "模型回归" in result.stdout


def test_ci_fast_tier_runs_device_suite():
    """fast 档必须纳入「设备层与安全闸（零硬件）」回归项。

    设备层（安全七闸 / 状态机 / 执行环 / 恢复流程 / 运行日志 / 硬件门控脚本）
    **零硬件即可验证** ⇒ 必须**恒跑**，不得因环境缺硬件而 SKIP。
    """
    result = run_ci("fast")
    combined = result.stdout + result.stderr
    assert result.returncode == 0, combined
    assert "tests/device" in combined
    assert "[SKIP] tests/device" not in combined


def test_ci_fast_tier_device_check_fails_the_build():
    """新增的设备层回归项失败即整体失败（不得被降级成 SKIP）。"""
    text = CI_SCRIPT.read_text(encoding="utf-8")
    assert 'run_check "tests/device"' in text, "设备层回归项必须用 run_check（失败即整体失败）"
    assert 'run_check_skippable "tests/device"' not in text


def test_ci_usage_mentions_device_suite():
    """--help 必须说明 fast 档已含设备层回归（避免"以为没跑"）。"""
    result = run_ci("--help", timeout=60)
    assert result.returncode == 0
    assert "设备层" in result.stdout


def test_ci_fast_tier_runs_bimanual_coordination_suite():
    """fast 档必须纳入「双臂紧协调」回归项（S05-T07）。

    解析 IK / 双臂限位 / 共享时间轴 / 最小坐标参数化 / 凸区域 / 协调规划 /
    指标脚本 / 协调 CLI 全部**零硬件即可验证** ⇒ 必须**恒跑**，不得 SKIP。
    """
    result = run_ci("fast")
    combined = result.stdout + result.stderr
    assert result.returncode == 0, combined
    assert "tests/bimanual-coordination" in combined
    assert "[SKIP] tests/bimanual-coordination" not in combined


def test_ci_bimanual_coordination_check_fails_the_build():
    """新增的紧协调回归项失败即整体失败（不得被降级成 SKIP）。"""
    text = CI_SCRIPT.read_text(encoding="utf-8")
    assert (
        'run_check "tests/bimanual-coordination"' in text
    ), "紧协调回归项必须用 run_check（失败即整体失败）"
    assert 'run_check_skippable "tests/bimanual-coordination"' not in text


def test_ci_usage_mentions_bimanual_coordination():
    """--help 必须说明 fast 档已含双臂紧协调回归（避免"以为没跑"）。"""
    result = run_ci("--help", timeout=60)
    assert result.returncode == 0
    assert "紧协调" in result.stdout
