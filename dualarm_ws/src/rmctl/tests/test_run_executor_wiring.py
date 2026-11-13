"""rmctl run 的执行器接线测试（T7）。

覆盖三条路径：

* ``--executor rm_executor`` → 走**新执行器**（离线 fake），报告落盘、退出码 0；
* 默认（legacy）→ **保留** MockExecutor 回放，但明确标注 **legacy / 不作验收证据**；
* ``--live`` → 走新执行器的**真机**路径；控制器不可达时退出码 4 +
  reason 明确（诚实降级，不伪造数值）。
"""
from __future__ import annotations

import json
import os

import pytest

from rmctl import cli as rmctl_cli


def _payload(verify=True):
    payload = {
        "schema_version": 1,
        "group": "rm75_arm",
        "joint_names": ["joint%d" % i for i in range(1, 8)],
        "units": {"times": "s", "points": "rad"},
        "times": [0.0, 0.05, 0.1],
        "points": [[0.0] * 7, [0.02] * 7, [0.04] * 7],
    }
    if verify is not None:
        payload["verify"] = {"ok": bool(verify), "checks": {}}
    return payload


@pytest.fixture()
def traj_file(tmp_path):
    path = tmp_path / "traj.json"
    path.write_text(json.dumps(_payload(), ensure_ascii=False), encoding="utf-8")
    return str(path)


def test_run_with_rm_executor_writes_report(traj_file, tmp_path, capsys):
    """--executor rm_executor → 走新执行器，报告落盘、退出码 0。"""
    out = str(tmp_path / "report.json")
    rc = rmctl_cli.main(["run", "--traj", traj_file, "--executor", "rm_executor",
                          "--report", out])
    assert rc == 0, capsys.readouterr().out
    assert os.path.isfile(out)
    with open(out, "r", encoding="utf-8") as fh:
        rep = json.load(fh)
    assert rep["status"] == "ok"
    assert rep["mode"] == "fake"
    assert set(rep["metrics"]) >= {"ticks", "tick_jitter_p99_ms", "arrival_err_deg"}


def test_run_default_is_marked_legacy(traj_file, capsys):
    """默认路径 = legacy MockExecutor 回放，且**明确标注不作验收证据**。"""
    rc = rmctl_cli.main(["run", "--traj", traj_file, "--fast"])
    out = capsys.readouterr().out
    assert "legacy" in out
    assert "不作验收证据" in out
    assert rc == 0


def test_run_live_unreachable_controller_is_error(traj_file, tmp_path, capsys):
    """--live 且控制器不可达 → 退出码 4 + reason 明确 + 无伪造数值。"""
    out = str(tmp_path / "live_report.json")
    rc = rmctl_cli.main(["run", "--traj", traj_file, "--live",
                          "--ip", "127.0.0.1", "--port", "59999",
                          "--report", out])
    assert rc == 4, capsys.readouterr().out
    with open(out, "r", encoding="utf-8") as fh:
        rep = json.load(fh)
    assert rep["status"] == "error"
    assert rep["mode"] == "live"
    assert "控制器不可达" in rep["reason"]
    assert rep["metrics"]["tick_jitter_p99_ms"] is None
