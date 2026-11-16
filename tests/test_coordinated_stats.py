"""协调规划指标脚本的行为测试（S05-T06）。

这些断言守住的契约：

* ``scripts/coordinated_stats.py`` **可运行**、退出码 0；
* JSON 摘要字段齐全：``success_rate`` / ``solve_p95_s`` / ``end_to_end_p95_s``
  / ``unverified_metrics``；
* **验收口径**与**端到端口径分行**打印（报告里能一眼分清哪条对应验收）；
* **真机 / 物理指标一律硬件门控 SKIP**：``unverified_metrics`` **非空**且
  逐条列出物理指标（绝不拿合成数据冒充物理达标），且**退出码仍是 0**；
* 参数非法（``--samples 0``）⇒ 非 0 退出码（不静默跑 0 组还报成功）。

外部依赖：只有文件系统（场景 YAML）+ 子进程（脚本自身）；无网络。
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path
from typing import Any, Dict

REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = REPO_ROOT / "scripts" / "coordinated_stats.py"


def _run_script(*args: str, timeout: float = 900.0) -> subprocess.CompletedProcess:
    """跑一次指标脚本（``PYTHONPATH`` 指向 ``src``）。"""
    env = dict(os.environ)
    env["PYTHONPATH"] = (
        str(REPO_ROOT / "src") + os.pathsep + env.get("PYTHONPATH", "")
    )
    return subprocess.run(
        [sys.executable, str(SCRIPT), *args],
        cwd=str(REPO_ROOT),
        capture_output=True,
        text=True,
        env=env,
        timeout=timeout,
    )


def _payload(stdout: str) -> Dict[str, Any]:
    """解析脚本的 JSON 输出（``--json`` ⇒ 单个 JSON 文档）。"""
    return json.loads(stdout)


# ── JSON 摘要 ──────────────────────────────────────────────────────────
def test_script_json_summary_has_the_required_fields() -> None:
    """JSON 摘要：字段齐全 + 退出码 0（无真机也**不**失败）。"""
    result = _run_script("--samples", "2", "--json")
    assert result.returncode == 0, result.stdout + result.stderr
    payload = _payload(result.stdout)
    for key in (
        "success_rate",
        "solve_p95_s",
        "end_to_end_p95_s",
        "unverified_metrics",
    ):
        assert key in payload, f"JSON 摘要缺少字段：{key}"
    assert 0.0 <= float(payload["success_rate"]) <= 1.0
    assert payload["solve_p95_s"] is not None
    assert payload["end_to_end_p95_s"] is not None
    assert payload["samples_completed"] == 2


def test_script_lists_unverified_physical_metrics() -> None:
    """真机 / 物理指标**逐条列出**（硬件门控 SKIP），且退出码仍为 0。"""
    result = _run_script("--samples", "1", "--json")
    assert result.returncode == 0, result.stdout + result.stderr
    payload = _payload(result.stdout)
    unverified = payload["unverified_metrics"]
    assert unverified, "未验证指标清单不得为空"
    joined = "\n".join(unverified)
    for expected in ("成功率", "P95", "漂移", "相对位姿", "逃逸"):
        assert expected in joined, f"未验证清单缺少：{expected}"
    assert "真机" in joined


def test_script_separates_acceptance_and_end_to_end_metrics() -> None:
    """**验收口径**与**端到端口径**分行（口径不许混用）。"""
    result = _run_script("--samples", "1")
    assert result.returncode == 0, result.stdout + result.stderr
    out = result.stdout
    assert "[验收口径]" in out
    assert "[端到端]" in out
    assert "未验证" in out
    # 两类口径必须**分行**（不能挤在同一行里）。
    acceptance_lines = [line for line in out.splitlines() if "[验收口径]" in line]
    end_to_end_lines = [line for line in out.splitlines() if "[端到端]" in line]
    assert acceptance_lines and end_to_end_lines
    for line in acceptance_lines:
        assert "[端到端]" not in line
    for line in end_to_end_lines:
        assert "[验收口径]" not in line


def test_script_reports_solve_and_end_to_end_separately() -> None:
    """JSON 里求解阶段与端到端**是两个独立字段**（口径绑定 R017）。"""
    result = _run_script("--samples", "1", "--json")
    assert result.returncode == 0, result.stdout + result.stderr
    payload = _payload(result.stdout)
    assert "acceptance_metrics" in payload
    assert "end_to_end_metrics" in payload
    assert "solve_p95_s" in payload["acceptance_metrics"]
    assert "end_to_end_p95_s" in payload["end_to_end_metrics"]
    assert payload["solve_p95_target_s"] == 3.0


# ── 负例 ───────────────────────────────────────────────────────────────
def test_script_rejects_non_positive_samples() -> None:
    """负例：``--samples 0`` ⇒ 非 0 退出码（不静默跑 0 组还报成功）。"""
    result = _run_script("--samples", "0")
    assert result.returncode != 0
    assert "samples" in (result.stdout + result.stderr)
