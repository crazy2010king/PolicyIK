"""rm_executor 入口与报告落盘测试（T7）。

覆盖契约：

* ``--mode fake`` 跑通 → 退出码 0，``--report`` 落盘统一报告，schema 字段齐
  （顶层 7 键 + metrics 15 键 + 事件 6 键），``validate_report`` 无问题；
* ``--mode live`` 控制器不可达 → **退出码 4**，报告 reason 明确（「控制器不可达」），
  且**没有任何伪造数值**（实测类指标一律 ``None``）；
* ``verify`` 缺失且未开 ``--allow-unverified`` → 被拒（退出码 3）；
* ``--allow-unverified`` 开启 → 离线假执行放行，且 evidence 里写明放行；
* 真机路径**拒绝** ``--allow-unverified``（诚实分层）；
* ``--duration`` 延展出等效长跑的 tick 序列；
* ``rm_executor.runtime`` 入口（真机窗口角色）在控制器不可达时同样诚实降级。
"""
from __future__ import annotations

import json
import os

import pytest

from rm_executor import cli as exec_cli
from rm_executor import contract, report as report_mod
from rm_executor import runtime as exec_runtime


# --------------------------------------------------------------------- 夹具
def _payload(times, points, verify=True):
    payload = {
        "schema_version": 1,
        "group": "rm75_arm",
        "joint_names": ["joint%d" % i for i in range(1, 8)],
        "units": {"times": "s", "points": "rad"},
        "times": list(times),
        "points": [list(row) for row in points],
    }
    if verify is not None:
        payload["verify"] = {"ok": bool(verify), "checks": {}}
    return payload


def _short_payload(verify=True):
    """一条限值内的短轨迹（0 → 0.05 rad，0.05 s）。"""
    return _payload([0.0, 0.05], [[0.0] * 7, [0.05] * 7], verify=verify)


def _read_report(path):
    with open(path, "r", encoding="utf-8") as fh:
        return json.load(fh)


# --------------------------------------------------------------------- 用例
def test_fake_mode_runs_and_writes_report(tmp_path, write_traj, capsys):
    """fake 模式跑通 + 报告落盘 + 退出码 0。"""
    traj = write_traj(_short_payload())
    out = str(tmp_path / "report.json")

    rc = exec_cli.main(["--traj", traj, "--mode", "fake", "--report", out])

    assert rc == 0, capsys.readouterr().out
    assert os.path.isfile(out)
    rep = _read_report(out)
    assert rep["status"] == "ok", rep
    assert rep["mode"] == "fake"
    # schema 校验：0 问题
    assert report_mod.validate_report(rep) == []
    # 顶层键 + metrics 键齐全
    assert set(rep.keys()) == set(report_mod.TOP_LEVEL_KEYS)
    assert set(rep["metrics"].keys()) == set(report_mod.METRIC_KEYS)
    # 关键指标有值（离线代理跑出了真实数字，不是 None 冒充）
    assert rep["metrics"]["ticks"] > 0
    assert rep["metrics"]["tick_jitter_p99_ms"] is not None
    assert rep["metrics"]["arrival_err_deg"] is not None


def test_fake_mode_report_is_identical_schema_for_both_modes(tmp_path, write_traj):
    """fake 与 live 用的是**同一份报告 schema**（键集合一致）。"""
    traj = write_traj(_short_payload())
    out = str(tmp_path / "report.json")
    exec_cli.main(["--traj", traj, "--mode", "fake", "--report", out])
    fake_rep = _read_report(out)

    exec_cli.main(["--traj", traj, "--mode", "live", "--ip", "127.0.0.1",
                   "--port", "59999", "--report", out])
    live_rep = _read_report(out)

    assert set(fake_rep.keys()) == set(live_rep.keys())
    assert set(fake_rep["metrics"].keys()) == set(live_rep["metrics"].keys())
    assert fake_rep["mode"] == "fake" and live_rep["mode"] == "live"


def test_live_mode_unreachable_controller_is_error_without_fabrication(
        tmp_path, write_traj, capsys):
    """控制器不可达 → 退出码 4 + reason 明确 + **零伪造数值**。"""
    traj = write_traj(_short_payload())
    out = str(tmp_path / "report.json")

    rc = exec_cli.main(["--traj", traj, "--mode", "live", "--ip", "127.0.0.1",
                        "--port", "59999", "--report", out])

    assert rc == 4, capsys.readouterr().out
    rep = _read_report(out)
    assert rep["status"] == "error"
    assert rep["mode"] == "live"
    assert "控制器不可达" in rep["reason"]
    assert rep["next_action"], "失败报告必须带可粘贴的下一步"
    assert report_mod.validate_report(rep) == []
    # 实测类指标一律 None（**不返回 0 冒充**）
    metrics = rep["metrics"]
    for key in ("tick_jitter_p99_ms", "tick_jitter_max_ms", "follow_err_rms_deg",
                "follow_err_max_deg", "arrival_err_deg", "arrival_hold_ms",
                "duration_s", "rt_factor"):
        assert metrics[key] is None, "%s 不应被伪造" % key
    assert metrics["ticks"] == 0
    # 事件里留下结构性痕迹
    assert any(ev["severity"] == "FATAL" for ev in rep["events"])


def test_live_mode_without_ip_is_error(tmp_path, write_traj):
    """live 模式缺 --ip → 诚实报错（退出码 4），不猜 IP。"""
    traj = write_traj(_short_payload())
    out = str(tmp_path / "report.json")
    rc = exec_cli.main(["--traj", traj, "--mode", "live", "--report", out])
    assert rc == 4
    rep = _read_report(out)
    assert rep["status"] == "error"
    assert "控制器不可达" in rep["reason"]


def test_live_mode_rejects_allow_unverified(tmp_path, write_traj, capsys):
    """真机路径拒绝 --allow-unverified（离线放行不得用于真机）。"""
    traj = write_traj(_short_payload(verify=None))
    out = str(tmp_path / "report.json")
    rc = exec_cli.main(["--traj", traj, "--mode", "live", "--ip", "127.0.0.1",
                        "--allow-unverified", "--report", out])
    assert rc == 3, capsys.readouterr().out
    rep = _read_report(out)
    assert rep["status"] == "blocked"
    assert "allow-unverified" in rep["reason"]


def test_missing_verify_is_rejected_without_allow_unverified(tmp_path, write_traj,
                                                             capsys):
    """verify 缺失且未开 --allow-unverified → 被拒（退出码 3，0 次下发）。"""
    traj = write_traj(_short_payload(verify=None))
    out = str(tmp_path / "report.json")
    rc = exec_cli.main(["--traj", traj, "--mode", "fake", "--report", out])
    assert rc == 3, capsys.readouterr().out
    rep = _read_report(out)
    assert rep["status"] == "blocked"
    assert "verify" in rep["reason"]
    assert rep["metrics"]["ticks"] == 0
    assert rep["next_action"]


def test_allow_unverified_passes_and_is_recorded(tmp_path, write_traj, capsys):
    """--allow-unverified：离线放行，且 evidence 写明「不代表真机已校验」。"""
    traj = write_traj(_short_payload(verify=None))
    out = str(tmp_path / "report.json")
    rc = exec_cli.main(["--traj", traj, "--mode", "fake", "--allow-unverified",
                        "--report", out])
    assert rc == 0, capsys.readouterr().out
    rep = _read_report(out)
    assert rep["status"] == "ok"
    joined = " ".join(rep["evidence"])
    assert "allow_unverified" in joined or "未校验" in joined


def test_duration_extends_equivalent_ticks(tmp_path, write_traj, capsys):
    """--duration：末点保持延展 → 等效时长的 tick 序列。"""
    traj = write_traj(_short_payload())
    out = str(tmp_path / "report.json")
    rc = exec_cli.main(["--traj", traj, "--mode", "fake", "--duration", "1.0",
                        "--report", out])
    assert rc == 0, capsys.readouterr().out
    rep = _read_report(out)
    assert rep["metrics"]["planned_duration_s"] == pytest.approx(1.0, abs=0.02)
    assert rep["metrics"]["ticks"] >= 90


def test_missing_trajectory_file_is_blocked(tmp_path, capsys):
    """轨迹文件不存在 → blocked（退出码 3），不抛裸异常。"""
    out = str(tmp_path / "report.json")
    rc = exec_cli.main(["--traj", str(tmp_path / "nope.json"), "--mode", "fake",
                        "--report", out])
    assert rc == 3, capsys.readouterr().out
    rep = _read_report(out)
    assert rep["status"] == "blocked"
    assert rep["next_action"]


def test_runtime_entry_unreachable_controller(tmp_path, write_traj, capsys):
    """rm_runtime（真机窗口角色）：控制器不可达 → 退出码 4 + 诚实降级。"""
    traj = write_traj(_short_payload())
    out = str(tmp_path / "report.json")
    rc = exec_runtime.main(["--arm", "left", "--traj", traj, "--ip", "127.0.0.1",
                             "--port", "59999", "--report", out])
    assert rc == 4, capsys.readouterr().out
    rep = _read_report(out)
    assert rep["status"] == "error"
    assert rep["mode"] == "live"
    assert "控制器不可达" in rep["reason"]
    assert rep["metrics"]["tick_jitter_p99_ms"] is None


def test_runtime_parser_has_no_mode_or_allow_unverified():
    """runtime 入口**强制 live**：不接受 --mode / --allow-unverified。"""
    parser = exec_runtime.build_parser()
    opts = set()
    for action in parser._actions:
        opts.update(action.option_strings)
    assert "--mode" not in opts
    assert "--allow-unverified" not in opts
    assert "--arm" in opts and "--report" in opts


def test_exit_code_mapping_matches_contract():
    """退出码口径与 contract.EXIT_CODES 一致（0 / 3 / 4）。"""
    assert contract.EXIT_CODES["ok"] == 0
    assert contract.EXIT_CODES["blocked"] == 3
    assert contract.EXIT_CODES["error"] == 4
