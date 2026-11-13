"""`rmctl gripper` 子命令组的 CLI 契约测试（T5 / S05）。

覆盖：

* 参数非法 → 退出码 2（速度/力度越界、count 越界）；
* 配置缺字段 → ``status=blocked`` + 退出码 3，reason **指名**缺哪个字段；
* ``--json`` 输出是**单个可 json.loads 的对象**，且含 status/reason/next_action/
  evidence/mode；
* ``cycle --count 100`` 在**代理模式**下完成 100 次开合、0 次超时，且输出带
  ``mode=fake`` 与「代理模型，不代表真机结论」；
* ``judge --fault each --count 10`` 六类判定全部正确（与 T2/T4 的纯逻辑一致）；
  任一注入不符 → 退出码非 0；
* doctor 仍能跑通，且新增 ``hw:gripper`` 检查项出现在输出里（既有 doctor 行为不回归）。

只用 git 跟踪的仓库文件 + pytest 临时目录，**不依赖真机 / 不依赖厂商 SDK**。
"""
from __future__ import annotations

import json
import os

import pytest

from rmctl import cli


def _run(capsys, argv):
    """跑一次 CLI，返回 (exit_code, stdout, stderr)。"""
    code = cli.main(argv)
    cap = capsys.readouterr()
    return code, cap.out, cap.err


@pytest.fixture(autouse=True)
def _no_rm_ws(monkeypatch):
    """清掉 RM_WS：临时工作区场景才不会被环境变量劫持。"""
    monkeypatch.delenv("RM_WS", raising=False)


def _make_ws(tmp_path, gripper_yaml):
    """造一个「像仓库根」的临时工作区（workcell/ + dualarm_ws/src/rm_description/）。

    ``rm_hw_bridge.paths.workspace_root`` 只认同时含这两个标记的目录，
    所以临时工作区必须补上 ``dualarm_ws/src/rm_description``。
    """
    ws = tmp_path / "ws"
    (ws / "workcell").mkdir(parents=True)
    (ws / "dualarm_ws" / "src" / "rm_description").mkdir(parents=True)
    (ws / "workcell" / "rm_workcell.yaml").write_text(gripper_yaml, encoding="utf-8")
    return str(ws)


# ------------------------------------------------------------------ 参数非法
def test_gripper_invalid_speed_exits_two(capsys, repo_root):
    """速度越界 → 退出码 2（参数非法）。"""
    code, _out, err = _run(capsys, ["--ws", repo_root, "gripper", "open",
                                    "--speed", "5000"])
    assert code == 2, err
    assert "--speed" in err


def test_gripper_invalid_force_exits_two(capsys, repo_root):
    """力度越界 → 退出码 2。"""
    code, _out, err = _run(capsys, ["--ws", repo_root, "gripper", "close",
                                    "--force", "-1"])
    assert code == 2, err
    assert "--force" in err


def test_gripper_invalid_count_exits_two(capsys, repo_root):
    """count 越界 → 退出码 2。"""
    code, _out, err = _run(capsys, ["--ws", repo_root, "gripper", "cycle",
                                    "--count", "0"])
    assert code == 2, err
    assert "--count" in err


def test_gripper_without_subcommand_exits_two(capsys, repo_root):
    """缺子命令 → 退出码 2（不是 0，避免「看着像成功」）。"""
    code, _out, err = _run(capsys, ["--ws", repo_root, "gripper"])
    assert code == 2, err
    assert "probe" in err


# --------------------------------------------------------- 配置缺字段 → blocked
def test_gripper_missing_config_field_is_blocked(capsys, tmp_path):
    """配置缺必需字段 → status=blocked + 退出码 3，reason 指名字段。"""
    ws = _make_ws(tmp_path, "gripper:\n  model: RMG22-65\n  protocol: modbus_rtu\n")
    code, out, _err = _run(capsys, ["--ws", ws, "gripper", "probe",
                                    "--dry-run", "--json"])
    assert code == 3, out
    data = json.loads(out)
    assert data["status"] == "blocked"
    assert "device_id" in data["reason"]
    assert data["next_action"]
    assert data["evidence"]


def test_gripper_missing_gripper_section_is_blocked(capsys, tmp_path):
    """完全没有 gripper: 段 → 同样 blocked（退出码 3）。"""
    ws = _make_ws(tmp_path, "calibration:\n  note: 没有夹爪段\n")
    code, out, _err = _run(capsys, ["--ws", ws, "gripper", "cycle",
                                    "--count", "1", "--json"])
    assert code == 3, out
    data = json.loads(out)
    assert data["status"] == "blocked"
    assert "gripper" in data["reason"]


# ------------------------------------------------------------- JSON 输出形状
def test_gripper_probe_dry_run_json_is_single_object(capsys, repo_root):
    """--json 输出必须是**单个**可 json.loads 的对象，且字段齐全。"""
    code, out, err = _run(capsys, ["--ws", repo_root, "gripper", "probe",
                                   "--dry-run", "--json"])
    assert code == 0, (out, err)
    data = json.loads(out)
    for key in ("status", "reason", "next_action", "evidence", "mode"):
        assert key in data, (key, data)
    assert data["status"] == "ok"
    assert data["mode"] == "dry_run"
    assert isinstance(data["evidence"], list) and data["evidence"]


def test_gripper_dry_run_does_not_dispatch(capsys, repo_root):
    """dry-run 只解析参数与配置，**不下发任何动作**。"""
    code, out, _err = _run(capsys, ["--ws", repo_root, "gripper", "open",
                                    "--dry-run", "--json"])
    assert code == 0, out
    data = json.loads(out)
    assert data["mode"] == "dry_run"
    assert data["action"] == "open"
    assert any("SDK 写调用次数: 0" in str(ev) for ev in data["evidence"])


def test_gripper_probe_real_path_blocked_without_sdk(capsys, repo_root, monkeypatch):
    """真机路径缺 SDK → blocked + 退出码 3（**绝不伪造探针结论**）。"""
    monkeypatch.delenv("RM_API2_LIB", raising=False)
    code, out, _err = _run(capsys, ["--ws", repo_root, "gripper", "probe",
                                    "--json"])
    assert code == 3, out
    data = json.loads(out)
    assert data["status"] == "blocked"
    assert "真机窗口" in data["next_action"]
    assert "probe" not in data or data.get("mode") in ("live", "gripper")


# ------------------------------------------------------------------- cycle
def test_gripper_cycle_100_proxy_reports_no_timeout(capsys, repo_root):
    """cycle --count 100 在代理模式下完成 100 次开合、0 次超时，并带 mode=fake 标注。"""
    code, out, err = _run(capsys, ["--ws", repo_root, "gripper", "cycle",
                                   "--count", "100", "--json"])
    assert code == 0, (out, err)
    data = json.loads(out)
    assert data["mode"] == "fake"
    assert data["cycles"] == 100
    assert data["timeouts"] == 0
    assert data["failure_count"] == 0
    assert "代理模型，不代表真机结论" in data["note"]
    assert "mode=fake" in out


def test_gripper_cycle_human_output_marks_proxy(capsys, repo_root):
    """人读摘要里也必须写明「代理模型，不代表真机结论」。"""
    code, out, _err = _run(capsys, ["--ws", repo_root, "gripper", "cycle",
                                    "--count", "2"])
    assert code == 0, out
    assert "代理模型，不代表真机结论" in out


# ------------------------------------------------------------------- judge
def test_gripper_judge_each_ten_all_correct(capsys, repo_root):
    """judge --fault each --count 10 → 六类判定全部正确（与纯逻辑一致）。"""
    code, out, err = _run(capsys, ["--ws", repo_root, "gripper", "judge",
                                   "--fault", "each", "--count", "10", "--json"])
    assert code == 0, (out, err)
    data = json.loads(out)
    assert data["status"] == "ok"
    assert data["mismatches"] == 0
    rows = data["faults"]
    assert len(rows) == 6
    for row in rows:
        assert row["injected"] == 10
        assert row["correct"] == 10
        assert row["accuracy"] == 1.0
    assert {row["fault"] for row in rows} == set(cli.GRIPPER_FAULT_CLASSES)


def test_gripper_judge_mismatch_exits_nonzero(capsys, repo_root, monkeypatch):
    """任一注入不符 → 退出码非 0（绝不把「判错」报成成功）。"""
    original = cli._gripper_fixture

    def _always_grasp_ok(hw, fault, thresholds):
        # 无论注入哪类，都给出「夹到」的夹具 → 其余五类必然不符
        return original(hw, "grasp_ok", thresholds)

    monkeypatch.setattr(cli, "_gripper_fixture", _always_grasp_ok)
    code, out, _err = _run(capsys, ["--ws", repo_root, "gripper", "judge",
                                    "--fault", "each", "--count", "1", "--json"])
    assert code != 0, out
    data = json.loads(out)
    assert data["status"] == "error"
    assert data["mismatches"] > 0


def test_gripper_judge_single_fault(capsys, repo_root):
    """单类注入也走同一条链路。"""
    code, out, err = _run(capsys, ["--ws", repo_root, "gripper", "judge",
                                   "--fault", "stall", "--count", "3", "--json"])
    assert code == 0, (out, err)
    data = json.loads(out)
    assert len(data["faults"]) == 1
    assert data["faults"][0]["fault"] == "stall"
    assert data["faults"][0]["correct"] == 3


# ------------------------------------------------------------------ doctor
def test_doctor_still_runs_and_lists_hw_gripper(capsys, repo_root):
    """doctor 仍能跑通，且 hw:gripper 出现在输出里。"""
    code, out, _err = _run(capsys, ["--ws", repo_root, "doctor"])
    assert code in (0, 1), out
    assert "hw:gripper" in out


def test_hw_gripper_check_is_info_when_calibration_pending(repo_root):
    """标定系数为 null → INFO 级（**不阻塞**）+ 可粘贴的回填动作。"""
    chk = cli._hw_gripper_check(repo_root)
    assert chk["name"] == "hw:gripper"
    assert chk["ok"] is False
    assert chk["level"] == "INFO"
    assert "回填标定系数" in chk["action"]
    assert "position_mm_per_unit" in chk["detail"]


def test_hw_gripper_check_is_fail_when_config_incomplete(tmp_path):
    """必需字段缺失 → FAIL 级（阻塞）且 reason 指名字段。"""
    ws = _make_ws(tmp_path, "gripper:\n  model: RMG22-65\n")
    chk = cli._hw_gripper_check(ws)
    assert chk["name"] == "hw:gripper"
    assert chk["ok"] is False
    assert chk["level"] == "FAIL"
    assert "device_id" in chk["detail"]
    assert chk["action"]


def test_hw_check_names_include_gripper():
    """hw:* 检查项共 5 条，顺序固定。"""
    assert len(cli.HW_CHECK_NAMES) == 5
    assert cli.HW_CHECK_NAMES[-1] == "hw:gripper"


def test_doctor_hw_checks_expose_gripper_fields(repo_root):
    """单元级：_hw_checks 里含 hw:gripper，字段齐全。"""
    checks = cli._hw_checks(repo_root)
    names = [c["name"] for c in checks]
    assert names == list(cli.HW_CHECK_NAMES)
    for c in checks:
        for key in ("name", "ok", "detail", "action", "level"):
            assert key in c
        assert c["action"]
