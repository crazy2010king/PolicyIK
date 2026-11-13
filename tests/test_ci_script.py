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
