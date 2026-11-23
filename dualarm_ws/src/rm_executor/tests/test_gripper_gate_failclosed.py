"""夹爪闸门**不可用**时的执行层 **fail-closed** 行为（S12/T4）。

背景：工位文件 ``workcell/rm_workcell.yaml`` 有两种截然不同的语义，执行层
必须**区分**：

* 工位**未配置**夹爪（没有 ``gripper:`` 段）→ **不启用互锁**是正确语义，
  行为必须**逐字不变**（运动照常下发）；
* 工位**已配置**夹爪但**闸门不可用**（配置非法 / 硬件层导入失败）→
  静默降级为「无互锁」是**不安全降级**，必须 **fail-closed**：拒绝一切运动。

覆盖

* (a) 工位**未配置**夹爪 → 闸门为 ``None``、证据写「未启用夹爪互锁」、
  执行照常完成（``preposition`` / ``transmit`` 均发生）；
* (b) 工位**已配置**夹爪但配置**非法**（``gripper:`` 段缺 ``device_id``）
  → 闸门状态为 ``GATE_UNAVAILABLE``（**不复用** ``GRIPPER_BUSY`` /
  ``GRIPPER_UNJUDGED``）、报告 ``status=blocked`` / 退出码 3、
  事件写明「工位已配置夹爪但闸门不可用 → 拒绝运动」、
  ``next_action`` 非空且指名原因、**零运动下发**；
* (c) CLI 端到端：同一临时工作区下报告口径与退出码一致。

只用 git 跟踪的仓库文件 + pytest 临时目录；**不改仓库文件**。
"""
from __future__ import annotations

import os
import shutil
from typing import Any, Dict, List, Optional

import pytest
import yaml

from rm_executor import cli as exec_cli
from rm_executor import contract
from rm_executor.clock import ManualClock
from rm_executor.executor import Executor, ExecutorConfig
from rm_executor.feedback import ScriptedFeedback
from rm_executor.transport import FakeTransport

#: 确定性执行参数（与 workcell/rm_workcell.yaml::execution 一致）
CONFIG = ExecutorConfig(
    period_s=0.010,
    follow=False,
    trajectory_mode=1,
    radio=30,
    pre_position_speed=10,
)

#: 闸门不可用时的状态名（与运行时互锁态可区分）
STATE_GATE_UNAVAILABLE = "GATE_UNAVAILABLE"


@pytest.fixture(autouse=True)
def _no_rm_ws_env(monkeypatch):
    """``$RM_WS`` 会覆盖显式 ``ws`` 参数——测试里必须清掉。"""
    monkeypatch.delenv("RM_WS", raising=False)


def _traj_payload():
    """限值内的短轨迹（0 → 0.03 rad，0.5 s）。"""
    return {
        "schema_version": 1,
        "group": "rm75_arm",
        "joint_names": ["joint%d" % i for i in range(1, 8)],
        "units": {"times": "s", "points": "rad"},
        "times": [0.0, 0.5],
        "points": [[0.0] * 7, [0.03] * 7],
        "verify": {"ok": True, "checks": {}},
    }


def _events_of(rep, type_name):
    return [e for e in rep["events"] if e["type"] == type_name]


def _make_ws(tmp_path, repo_root: str, *, drop_gripper: bool,
             break_gripper: bool) -> str:
    """造一个临时工作区（仓库根布局），工位文件的 ``gripper:`` 段按需改造。

    * ``drop_gripper=True`` → 工位**未配置**夹爪（删掉整个 ``gripper:`` 段）；
    * ``break_gripper=True`` → 工位**已配置**夹爪但**字段缺失**
      （删掉 ``device_id`` → ``load_gripper_config`` 走「字段缺失」分支）。
    """
    ws = os.path.join(str(tmp_path), "ws")
    # 仓库根标记 + 关节限值（git 跟踪文件）
    cfg_dir = os.path.join(ws, "dualarm_ws", "src", "rm_description", "config")
    os.makedirs(cfg_dir)
    shutil.copyfile(
        os.path.join(repo_root, "dualarm_ws", "src", "rm_description", "config",
                     "joint_limits.yaml"),
        os.path.join(cfg_dir, "joint_limits.yaml"),
    )
    # 整个 workcell/ 目录（safety 段引用的 error_codes.yaml 等）
    shutil.copytree(os.path.join(repo_root, "workcell"),
                    os.path.join(ws, "workcell"))
    # 工位文件：按需改造 gripper: 段
    with open(os.path.join(repo_root, "workcell", "rm_workcell.yaml"),
              "r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    if drop_gripper:
        data.pop("gripper", None)
    if break_gripper:
        data["gripper"].pop("device_id", None)
    with open(os.path.join(ws, "workcell", "rm_workcell.yaml"),
              "w", encoding="utf-8") as fh:
        yaml.safe_dump(data, fh, allow_unicode=True, sort_keys=False)
    return ws


def _run(path, gate, transport=None):
    """跑一次执行（带闸门），返回 ``(report, transport)``。"""
    transport = transport or FakeTransport()

    def _script(_index):
        call = transport.last_call("transmit") or transport.last_call("preposition")
        if call is None:
            return [0.0] * 7
        return list(call["args"]["deg7"])

    feedback = ScriptedFeedback(script=_script, speed_deg_s=0.0, period_s=0.010)
    executor = Executor(transport, feedback, config=CONFIG,
                        clock=ManualClock(period_s=0.010), mode="fake",
                        motion_gate=gate)
    rep = executor.run(path)
    return rep, transport


# ---------------------------------------------------- (a) 工位未配置夹爪
def test_workcell_without_gripper_section_keeps_legacy_behaviour(
        tmp_path, repo_root, write_traj):
    """工位**未配置**夹爪 → 不启用互锁，运动照常下发（行为逐字不变）。"""
    ws = _make_ws(tmp_path, repo_root, drop_gripper=True, break_gripper=False)

    gate, note = exec_cli._make_motion_gate(ws)
    assert gate is None, note
    assert "未启用夹爪互锁" in note, note

    path = write_traj(_traj_payload())
    rep, transport = _run(path, gate)
    assert _events_of(rep, "gripper_interlock_blocked") == [], rep["events"]
    assert rep["status"] == "ok", rep
    assert contract.EXIT_CODES[rep["status"]] == 0
    assert transport.count("preposition") == 1
    assert transport.count("transmit") > 0


def test_cli_without_gripper_section_runs_normally(tmp_path, repo_root, write_traj):
    """CLI 端到端：工位未配置夹爪 → 报告 ok + 证据写「未启用夹爪互锁」。"""
    ws = _make_ws(tmp_path, repo_root, drop_gripper=True, break_gripper=False)
    path = write_traj(_traj_payload())

    rep = exec_cli.execute(mode="fake", trajectory_path=path, ws=ws)
    notes = [e for e in rep["evidence"] if "夹爪互锁" in e]
    assert any("未启用夹爪互锁" in n for n in notes), notes
    assert rep["status"] == "ok", rep


# ------------------------------------------- (b) 已配置夹爪但闸门不可用
def test_configured_gripper_with_broken_config_fails_closed(
        tmp_path, repo_root, write_traj):
    """工位**已配置**夹爪但配置非法 → fail-closed：零运动下发 + blocked(3)。"""
    ws = _make_ws(tmp_path, repo_root, drop_gripper=False, break_gripper=True)

    gate, note = exec_cli._make_motion_gate(ws)
    assert gate is not None, note
    # 状态名**不复用**运行时互锁态
    assert gate.state == STATE_GATE_UNAVAILABLE, gate.state
    assert gate.state not in ("GRIPPER_BUSY", "GRIPPER_UNJUDGED")
    assert "工位已配置夹爪但闸门不可用" in gate.reason, gate.reason
    # 闸门**永远**拒绝运动（不返回 None）
    assert gate.check() is not None

    path = write_traj(_traj_payload())
    rep, transport = _run(path, gate)

    blocked = _events_of(rep, "gripper_interlock_blocked")
    assert len(blocked) == 1, rep["events"]
    assert blocked[0]["severity"] == "NEEDS_HUMAN"
    # 拒绝语义**不是**「夹爪动作期间」
    assert "夹爪动作期间禁止运动" not in blocked[0]["reason"], blocked[0]["reason"]
    assert "工位已配置夹爪但闸门不可用" in blocked[0]["reason"], blocked[0]["reason"]
    # 可粘贴的下一步（非空）
    assert blocked[0]["next_action"], blocked
    # 报告口径：blocked / 退出码 3
    assert rep["status"] == "blocked", rep
    assert contract.EXIT_CODES[rep["status"]] == 3
    # **零运动下发**
    assert transport.count("preposition") == 0, transport.calls
    assert transport.count("transmit") == 0, transport.calls
    assert transport.calls_for("preposition") == [], transport.calls
    assert transport.calls_for("transmit") == [], transport.calls


def test_cli_with_broken_gripper_config_fails_closed(tmp_path, repo_root, write_traj):
    """CLI 端到端：已配置夹爪但配置非法 → blocked(3) + 零运动下发语义。"""
    ws = _make_ws(tmp_path, repo_root, drop_gripper=False, break_gripper=True)
    path = write_traj(_traj_payload())

    rep = exec_cli.execute(mode="fake", trajectory_path=path, ws=ws)
    assert rep["status"] == "blocked", rep
    assert contract.EXIT_CODES[rep["status"]] == 3

    blocked = _events_of(rep, "gripper_interlock_blocked")
    assert len(blocked) == 1, rep["events"]
    assert "工位已配置夹爪但闸门不可用" in blocked[0]["reason"], blocked[0]["reason"]
    assert blocked[0]["next_action"], blocked
    # 报告证据里写明「闸门不可用」（不是「已启用夹爪互锁」）
    notes = [e for e in rep["evidence"] if "夹爪互锁" in e]
    assert any("闸门不可用" in n for n in notes), notes


def test_repo_workcell_keeps_interlock_enabled(tmp_path, repo_root, write_traj):
    """回归保护：仓库真实工位（gripper 段合法）→ 闸门可用，行为不变。"""
    gate, note = exec_cli._make_motion_gate(repo_root)
    assert gate is not None, note
    assert gate.state != STATE_GATE_UNAVAILABLE, gate.state
    assert "已启用夹爪互锁" in note, note
