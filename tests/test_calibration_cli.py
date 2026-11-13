"""``calibrate`` CLI 与标定脚本的行为测试（S04-T08）。

这些断言守住的契约：

* ``calibrate --json``（默认合成采样）⇒ **退出码 0** + ``status == "skipped"``
  + 稳定的 ``reason`` 标签 + **未验证清单非空**（绝不把合成数据当实测精度）；
* ``calibrate`` 的**用法错误**（路径为空 / 文件不存在）⇒ 退出码 2；
* ``calibrate --accept-synthetic`` ⇒ 退出码 0 且摘要**含残差字段**（RMS / max）；
* **残差超阈值** ⇒ 退出码 1（业务失败），不返回「看起来正常」的结果；
* **数据文件内容非法** ⇒ 退出码 1；
* ``--out`` 落盘的 JSON 与 stdout 的结构一致；
* 脚本 ``scripts/calibrate_dual_arm.py`` 与 CLI 给出**同一份结论**（同一真相源）。

外部依赖：只有文件系统（YAML 读写）；无网络、无子进程（除脚本自检那条用子进程）。
"""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path
from typing import Any, Dict

import pytest

from rm75_drake.cli import EXIT_FAILURE, EXIT_OK, EXIT_USAGE, main

REPO_ROOT = Path(__file__).resolve().parents[1]
SAMPLES_FILE = REPO_ROOT / "calibration" / "samples" / "demo.yaml"
CALIBRATION_FILE = REPO_ROOT / "calibration" / "dual_arm.yaml"
SCRIPT = REPO_ROOT / "scripts" / "calibrate_dual_arm.py"


def _run_cli(argv, capsys) -> tuple[int, str, str]:
    """跑一次 CLI，返回 (退出码, stdout, stderr)。"""
    code = main(argv)
    captured = capsys.readouterr()
    return code, captured.out, captured.err


def _json_payload(stdout: str) -> Dict[str, Any]:
    """从 ``--json`` 的**单行**输出里解析载荷。"""
    lines = [line for line in stdout.splitlines() if line.strip()]
    assert len(lines) == 1, f"--json 必须输出单行 JSON，实际 {len(lines)} 行"
    return json.loads(lines[0])


# ── 默认路径：无实测数据 ⇒ SKIP + 逐条未验证 ──────────────────────────
def test_calibrate_default_is_skipped_with_unverified_list(capsys):
    """默认（合成采样 + 未实测标定文件）⇒ 退出码 0 / status=skipped / 未验证清单非空。"""
    code, out, _err = _run_cli(["calibrate", "--json"], capsys)
    assert code == EXIT_OK
    payload = _json_payload(out)
    assert payload["status"] == "skipped"
    assert payload["reason"] == "calibration_not_measured"
    assert payload["measured"] is False
    unverified = payload["unverified_metrics"]
    assert unverified, "未验证清单不得为空"
    joined = "\n".join(unverified)
    for keyword in ("真实标定精度", "共同工作区 TCP 同一点偏差 < 0.5 mm", "下发时间差 < 1 ms", "无碰撞"):
        assert keyword in joined, f"未验证清单缺条目：{keyword}"


def test_calibrate_json_is_single_line(capsys):
    """``--json`` 必须是**单行** JSON（便于机器消费）。"""
    _code, out, _err = _run_cli(["calibrate", "--json"], capsys)
    assert len([line for line in out.splitlines() if line.strip()]) == 1


def test_calibrate_human_output_lists_unverified_metrics(capsys):
    """人类可读输出也必须逐条列出未验证指标。"""
    _code, out, _err = _run_cli(["calibrate"], capsys)
    assert "未验证" in out
    assert "calibration_not_measured" in out


# ── 用法错误 ⇒ 退出码 2 ────────────────────────────────────────────────
def test_calibrate_with_empty_samples_path_is_usage_error(capsys):
    """负例：``--samples`` 为空（缺必填参数）⇒ 退出码 2。"""
    code, _out, err = _run_cli(["calibrate", "--samples", ""], capsys)
    assert code == EXIT_USAGE
    assert "samples" in err.lower() or "samples" in err


def test_calibrate_with_missing_samples_file_is_usage_error(capsys, tmp_path):
    """负例：采样文件不存在 ⇒ 退出码 2（本仓约定：路径问题 = 用法错误）。

    与 ``cli.replay``（录制文件不存在 → 2）/ ``cli.exec``（设备配置不存在 → 2）
    保持一致，便于脚本与 CI 按退出码分流。
    """
    missing = tmp_path / "no_such_samples.yaml"
    code, _out, err = _run_cli(["calibrate", "--samples", str(missing)], capsys)
    assert code == EXIT_USAGE
    assert "no_such_samples.yaml" in err


def test_calibrate_with_missing_calibration_file_is_usage_error(capsys, tmp_path):
    """负例：标定文件不存在 ⇒ 退出码 2。"""
    missing = tmp_path / "no_such_calibration.yaml"
    code, _out, err = _run_cli(["calibrate", "--calibration", str(missing)], capsys)
    assert code == EXIT_USAGE
    assert "no_such_calibration.yaml" in err


# ── 显式接受合成数据 ⇒ 解算结果可复算 ─────────────────────────────────
def test_calibrate_accept_synthetic_reports_residuals(capsys):
    """``--accept-synthetic`` ⇒ 退出码 0，摘要含残差字段（RMS / max）。"""
    code, out, _err = _run_cli(
        ["calibrate", "--json", "--samples", str(SAMPLES_FILE), "--accept-synthetic"], capsys
    )
    assert code == EXIT_OK
    payload = _json_payload(out)
    assert payload["status"] == "ok"
    assert payload["measured"] is False, "合成数据即便被接受也不得声称实测"
    residuals = payload["residuals"]
    assert "tool_tcp" in residuals and "base_extrinsic" in residuals
    for name in ("tool_tcp", "base_extrinsic"):
        assert "rms" in residuals[name] and "max" in residuals[name]
        assert residuals[name]["sample_count"] > 0
    # 即使状态 ok，未验证清单仍不得为空（合成数据不构成实测精度）
    assert payload["unverified_metrics"]


def test_calibrate_reports_unidentifiable_joints(capsys):
    """合成采样里 FK 不敏感的关节必须**显式**列为不可辨识（不得静默丢弃）。"""
    _code, out, _err = _run_cli(
        ["calibrate", "--json", "--samples", str(SAMPLES_FILE), "--accept-synthetic"], capsys
    )
    payload = _json_payload(out)
    section = payload["sections"]["joint_zero"]
    assert section["status"] == "partial"
    assert section["unidentifiable_joints"], "不可辨识关节必须被点名"


# ── 残差超阈值 ⇒ 退出码 1 ─────────────────────────────────────────────
def _write_noisy_samples(path: Path) -> None:
    """写一份 tool_tcp 带离群点的采样（残差必然超阈值）。"""
    local = [[0.0, 0.0, 0.0], [0.05, 0.0, 0.0], [0.0, 0.05, 0.0], [0.0, 0.0, 0.05]]
    world = [
        [0.01, 0.02, 0.03],
        [0.06, 0.02, 0.03],
        [0.01, 0.07, 0.03],
        [0.01, 0.02, 0.09],  # 离群点：+0.06 m 偏差 ⇒ 残差远超 5e-4
    ]
    lines = ["tool_tcp:"]
    for loc, wld in zip(local, world):
        lines.append(f"  - local_point: {loc}")
        lines.append(f"    world_point: {wld}")
        lines.append("    synthetic: true")
        lines.append("    source: synthetic")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def test_calibrate_residual_over_threshold_fails_with_exit_one(capsys, tmp_path):
    """负例：残差超阈值 ⇒ status=failed + 退出码 1（绝不返回「看起来正常」）。"""
    noisy = tmp_path / "noisy_samples.yaml"
    _write_noisy_samples(noisy)
    code, out, _err = _run_cli(
        ["calibrate", "--json", "--samples", str(noisy), "--accept-synthetic"], capsys
    )
    assert code == EXIT_FAILURE
    payload = _json_payload(out)
    assert payload["status"] == "failed"
    assert payload["reason"] == "residual_exceeded"
    assert payload["sections"]["tool_tcp"]["status"] == "failed"


# ── 数据文件内容非法 ⇒ 退出码 1 ──────────────────────────────────────
def test_calibrate_with_invalid_samples_content_exits_one(capsys, tmp_path):
    """负例：采样文件内容非法（未知顶层键）⇒ 退出码 1（业务失败）。"""
    bad = tmp_path / "bad_samples.yaml"
    bad.write_text("oops_key: 1\n", encoding="utf-8")
    code, _out, err = _run_cli(["calibrate", "--samples", str(bad)], capsys)
    assert code == EXIT_FAILURE
    assert "oops_key" in err


def test_calibrate_with_invalid_calibration_content_exits_one(capsys, tmp_path):
    """负例：标定文件内容非法（未知顶层键）⇒ 退出码 1。"""
    bad = tmp_path / "bad_calibration.yaml"
    bad.write_text(
        "schema_version: 1\nmeasured: false\ncalibrated: false\n"
        "method: demo\ndate: '2026-11-13'\noops_key: 1\n",
        encoding="utf-8",
    )
    code, _out, err = _run_cli(["calibrate", "--calibration", str(bad)], capsys)
    assert code == EXIT_FAILURE
    assert "oops_key" in err


# ── --out 落盘 ─────────────────────────────────────────────────────────
def test_calibrate_out_writes_same_structure(capsys, tmp_path):
    """``--out`` 落盘的 JSON 与 stdout 的结构一致（字段集合相同）。"""
    out_file = tmp_path / "result.json"
    code, out, _err = _run_cli(
        ["calibrate", "--json", "--out", str(out_file)], capsys
    )
    assert code == EXIT_OK
    payload = _json_payload(out)
    saved = json.loads(out_file.read_text(encoding="utf-8"))
    assert set(saved) >= {"status", "reason", "residuals", "measured", "unverified_metrics"}
    assert saved["status"] == payload["status"]
    assert saved["reason"] == payload["reason"]


# ── 脚本与 CLI 同源 ────────────────────────────────────────────────────
def test_script_and_cli_agree_on_status_and_reason():
    """脚本与 CLI 必须给出**同一份**结论（同一真相源，不是两套实现）。"""
    proc = subprocess.run(
        [sys.executable, str(SCRIPT), "--json"],
        cwd=str(REPO_ROOT),
        capture_output=True,
        text=True,
        timeout=300,
    )
    assert proc.returncode == EXIT_OK, proc.stdout + proc.stderr
    lines = [line for line in proc.stdout.splitlines() if line.strip()]
    assert len(lines) == 1
    payload = json.loads(lines[0])
    assert payload["status"] == "skipped"
    assert payload["reason"] == "calibration_not_measured"
    assert payload["unverified_metrics"]
    assert set(payload) >= {"status", "reason", "residuals", "measured", "unverified_metrics"}


def test_script_with_missing_samples_is_usage_error():
    """脚本的用法错误同样映射到退出码 2。"""
    proc = subprocess.run(
        [sys.executable, str(SCRIPT), "--samples", "no_such_file.yaml", "--json"],
        cwd=str(REPO_ROOT),
        capture_output=True,
        text=True,
        timeout=300,
    )
    assert proc.returncode == EXIT_USAGE


# ── 既有 CLI 契约零变化 ───────────────────────────────────────────────
def test_cli_help_lists_calibrate(capsys):
    """``--help`` 必须列出 calibrate 子命令。"""
    with pytest.raises(SystemExit) as excinfo:
        main(["--help"])
    assert excinfo.value.code == 0
    captured = capsys.readouterr()
    assert "calibrate" in captured.out
