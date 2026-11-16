"""``bimanual`` CLI 的行为测试（S05-T06）。

这些断言守住的契约：

* ``bimanual --json`` 输出**单行** JSON（可 ``json.loads`` 解析），
  字段齐全（status / reason / stage / diagnostics / unverified_metrics）；
* **成功路径**：退出码 0，``status="ok"``、``reason=""``、``stage=""``，
  ``diagnostics["num_escapes"] == 0``（零逃逸是硬要求）；
* **用法错误**（场景缺失 / 参数非法）⇒ 退出码 2；
* **业务失败**（follower 不可达）⇒ 退出码 1，且 ``stage`` / ``reason``
  **点名**（沿用解析 IK 的稳定词表）；
* 真机 / 物理指标**硬件门控 SKIP**：逐条列出，不冒充物理达标。

外部依赖：只有文件系统（场景 YAML）+ 子进程（CLI 自身）；无网络。
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path
from typing import Any, Dict, List

REPO_ROOT = Path(__file__).resolve().parents[1]
SCENARIO = REPO_ROOT / "scenarios" / "dual_arm.yaml"


def _run_cli(*args: str, timeout: float = 900.0) -> subprocess.CompletedProcess:
    """跑一次 ``python3 -m rm75_drake.cli bimanual ...``。"""
    env = dict(os.environ)
    env["PYTHONPATH"] = (
        str(REPO_ROOT / "src") + os.pathsep + env.get("PYTHONPATH", "")
    )
    return subprocess.run(
        [sys.executable, "-m", "rm75_drake.cli", "bimanual", *args],
        cwd=str(REPO_ROOT),
        capture_output=True,
        text=True,
        env=env,
        timeout=timeout,
    )


def _payload(stdout: str) -> Dict[str, Any]:
    """从 ``--json`` 的**单行**输出里解析载荷。"""
    lines: List[str] = [line for line in stdout.splitlines() if line.strip()]
    assert len(lines) == 1, f"--json 必须输出单行 JSON，实际 {len(lines)} 行"
    return json.loads(lines[0])


# ── 成功路径 ───────────────────────────────────────────────────────────
def test_bimanual_json_is_a_single_line_with_complete_fields() -> None:
    """正例：单行 JSON + 字段齐全 + 零逃逸。"""
    result = _run_cli("--scenario", str(SCENARIO), "--json")
    assert result.returncode == 0, result.stdout + result.stderr
    payload = _payload(result.stdout)
    assert payload["status"] == "ok"
    assert payload["success"] is True
    assert payload["reason"] == ""
    assert payload["stage"] == ""
    assert payload["command"] == "bimanual"
    diagnostics = payload["diagnostics"]
    assert diagnostics["num_escapes"] == 0
    assert diagnostics["num_regions"] >= 1
    assert float(diagnostics["relative_pose_residual_max_position_m"]) <= 1e-6
    assert float(diagnostics["relative_pose_residual_max_orientation_rad"]) <= 1e-6


def test_bimanual_lists_unverified_physical_metrics() -> None:
    """真机 / 物理指标逐条列出（**不**用合成数据冒充）。"""
    result = _run_cli("--scenario", str(SCENARIO), "--json")
    assert result.returncode == 0, result.stdout + result.stderr
    payload = _payload(result.stdout)
    unverified = payload["unverified_metrics"]
    assert unverified, "未验证指标清单不得为空"
    joined = "\n".join(unverified)
    assert "真机" in joined
    assert "逃逸" in joined


def test_bimanual_human_readable_output_shows_the_summary() -> None:
    """正例：人类可读模式打印摘要 + 未验证项清单。"""
    result = _run_cli("--scenario", str(SCENARIO))
    assert result.returncode == 0, result.stdout + result.stderr
    assert "[bimanual]" in result.stdout
    assert "未验证" in result.stdout


# ── 用法错误（退出码 2）───────────────────────────────────────────────
def test_missing_scenario_is_a_usage_error() -> None:
    """负例：场景文件缺失 ⇒ 退出码 2（用法错误）。"""
    result = _run_cli("--scenario", "does/not/exist.yaml", "--json")
    assert result.returncode == 2, result.stdout + result.stderr


def test_invalid_parameter_is_a_usage_error() -> None:
    """负例：参数非法（``--path-step 0``）⇒ 退出码 2。"""
    result = _run_cli("--scenario", str(SCENARIO), "--path-step", "0", "--json")
    assert result.returncode == 2, result.stdout + result.stderr


def test_invalid_x_lr_is_a_usage_error() -> None:
    """负例：``--x-matrix`` 末行不合法 ⇒ 退出码 2（点名）。"""
    result = _run_cli(
        "--scenario",
        str(SCENARIO),
        "--x-matrix",
        "1",
        "0",
        "0",
        "0",
        "0",
        "1",
        "0",
        "0",
        "0",
        "0",
        "1",
        "0",
        "0",
        "0",
        "2",
        "1",
        "--json",
    )
    assert result.returncode == 2, result.stdout + result.stderr


# ── 业务失败（退出码 1）───────────────────────────────────────────────
def test_unreachable_follower_exits_one_with_a_named_reason() -> None:
    """负例：follower 不可达 ⇒ 退出码 1 + ``stage`` / ``reason`` 点名。"""
    result = _run_cli(
        "--scenario",
        str(SCENARIO),
        "--start-joints",
        "-0.2",
        "1.0",
        "-1.0",
        "-1.0",
        "0.0",
        "0.2",
        "0.0",
        "--x-xyz",
        "5.0",
        "0.0",
        "0.0",
        "--json",
    )
    assert result.returncode == 1, result.stdout + result.stderr
    payload = _payload(result.stdout)
    assert payload["status"] == "failed"
    assert payload["success"] is False
    assert payload["stage"] == "parameterize"
    assert payload["reason"] in {
        "unreachable",
        "branch_exhausted",
        "limit_violation",
        "wrist_singular",
        "invalid_target",
        "path_infeasible",
    }
    assert payload["message"]


# ── X_LR 的显式性（**不许隐式假设**）────────────────────────────────────
def test_default_x_lr_is_flagged_as_a_placeholder() -> None:
    """不给 ``--x-*`` ⇒ 载荷里**明确标注**是占位值（不隐瞒）。"""
    result = _run_cli("--scenario", str(SCENARIO), "--json")
    assert result.returncode == 0, result.stdout + result.stderr
    payload = _payload(result.stdout)
    assert payload["x_lr"]["source"] == "default_placeholder"


def test_explicit_x_lr_is_reported_as_explicit() -> None:
    """显式给的 ``--x-xyz`` ⇒ ``source="explicit"``（值原样回报）。"""
    result = _run_cli(
        "--scenario", str(SCENARIO), "--x-xyz", "0.25", "0.0", "0.0", "--json"
    )
    assert result.returncode == 0, result.stdout + result.stderr
    payload = _payload(result.stdout)
    assert payload["x_lr"]["source"] == "explicit"
    assert payload["x_lr"]["translation"] == [0.25, 0.0, 0.0]


def test_x_lr_matrix_helper_rejects_malformed_input() -> None:
    """负例（单元级）：``bimanual_x_lr_matrix`` 对非法输入**点名报错**。"""
    import pytest

    from rm75_drake.cli import bimanual_x_lr_matrix

    with pytest.raises(ValueError) as excinfo:
        bimanual_x_lr_matrix(matrix=[1.0] * 15 + [2.0])
    assert "末行" in str(excinfo.value)

    with pytest.raises(ValueError):
        bimanual_x_lr_matrix(translation=[0.1, 0.2])

    with pytest.raises(ValueError):
        bimanual_x_lr_matrix(translation=[float("nan"), 0.0, 0.0])


def test_x_lr_matrix_helper_builds_the_expected_transform() -> None:
    """正例（单元级）：``--x-xyz`` / ``--x-rpy`` ⇒ 正确的 4×4 变换。"""
    import numpy as np

    from rm75_drake.cli import bimanual_x_lr_matrix

    matrix, meta = bimanual_x_lr_matrix(translation=[0.2, 0.0, 0.0])
    assert matrix.shape == (4, 4)
    assert np.allclose(matrix[:3, 3], [0.2, 0.0, 0.0])
    assert np.allclose(matrix[:3, :3], np.eye(3))
    assert meta["source"] == "explicit"

    # 绕 z 轴转 90° ⇒ R = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]。
    matrix, _meta = bimanual_x_lr_matrix(rpy=[0.0, 0.0, np.pi / 2])
    assert np.allclose(matrix[:3, :3], [[0, -1, 0], [1, 0, 0], [0, 0, 1]], atol=1e-9)
