"""tools/verify_env.py 的**退出码契约**守卫（S08-T01）。

本文件守住的唯一契约行是 ``.gsd/phases/01-acm1jg-phase/01-01-UAT.md:47``：

    | 传入非法 ``--joint-limits`` 路径 | 退出码 2（参数错误） |

契约要求把两类失败**严格分层**，不得合并成一个退出码：

* **参数错误（退出码 2）**：``--joint-limits`` 指向的路径不存在或不是文件。
  这属于「用法/输入」错误，必须在**任何文件读取、任何 Drake 导入之前**暴露，
  并在 stderr 上**点名该路径**（否则无法与「Drake 缺失导致的环境错误 2」区分）。
* **检查失败（退出码 1）**：路径合法，但限位交接件**内容**非法或与 URDF 漂移。

本文件只依赖真实 CLI 子进程调用；前两条用例**不依赖 Drake**（无 Drake 环境里
``--model`` 本身也会返回 2，因此必须断言 stderr 点名路径才能证明是参数错误）。
"""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
VERIFY = REPO_ROOT / "tools" / "verify_env.py"
LIMITS_PATH = REPO_ROOT / "models" / "rm75_drake" / "joint_limits.yaml"

#: 契约里逐字写死的非法路径样例。
NONEXISTENT_PATH = "/nonexistent/path.yaml"


def run_verify(*args: str) -> subprocess.CompletedProcess:
    """以子进程方式调用 verify_env.py（真实 CLI 行为，非 mock）。"""
    return subprocess.run(
        [sys.executable, str(VERIFY), *args],
        cwd=str(REPO_ROOT),
        capture_output=True,
        text=True,
        timeout=300,
    )


def test_nonexistent_joint_limits_path_exits_with_usage_error():
    """契约（01-01-UAT.md:47）：非法 --joint-limits 路径 → 退出码 2（参数错误）。"""
    result = run_verify("--model", "--joint-limits", NONEXISTENT_PATH)
    assert result.returncode == 2, result.stdout + result.stderr
    # stderr 必须点名该路径 —— 否则无法证明这是「参数错误」而不是「Drake 环境错误」
    assert NONEXISTENT_PATH in result.stderr
    assert "参数错误" in result.stderr


def test_joint_limits_path_pointing_to_directory_exits_with_usage_error(tmp_path):
    """负例：--joint-limits 指向目录（不是文件）→ 同样属于参数错误，退出码 2。"""
    directory = tmp_path / "joint_limits_dir"
    directory.mkdir()
    result = run_verify("--model", "--joint-limits", str(directory))
    assert result.returncode == 2, result.stdout + result.stderr
    assert str(directory) in result.stderr


def test_joint_limits_parameter_error_is_reported_before_any_model_load():
    """参数错误必须前置暴露：stderr 点名路径，且不产生模型自检的任何输出。"""
    result = run_verify("--model", "--joint-limits", NONEXISTENT_PATH)
    assert result.returncode == 2, result.stdout + result.stderr
    assert "DoF=" not in result.stdout


def test_joint_limits_parameter_error_json_output_is_parseable():
    """--json 变体同样返回 2，且 JSON 报告仍可被 json.loads 解析。"""
    result = run_verify("--model", "--joint-limits", NONEXISTENT_PATH, "--json")
    assert result.returncode == 2, result.stdout + result.stderr
    payload = json.loads(result.stdout)
    assert payload["ok"] is False
    entry = payload["checks"]["joint-limits"]
    assert entry["status"] == "ERROR"
    assert any(NONEXISTENT_PATH in problem for problem in entry["problems"])
    assert NONEXISTENT_PATH in result.stderr


def test_tampered_joint_limits_content_still_exits_one(tmp_path):
    """负例：路径合法但**内容**被篡改 → 仍是检查失败（退出码 1），不得变成 2。"""
    pytest.importorskip("pydrake")
    data = yaml.safe_load(LIMITS_PATH.read_text(encoding="utf-8"))
    data["joints"][1]["hardware"]["velocity_rad_s"] = 3.14
    target = tmp_path / "joint_limits.yaml"
    target.write_text(yaml.safe_dump(data, allow_unicode=True), encoding="utf-8")
    result = run_verify("--model", "--joint-limits", str(target))
    assert result.returncode == 1, result.stdout + result.stderr
    assert "joint_2" in result.stdout
