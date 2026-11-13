"""执行层**夹爪互锁**（运动闸门）的确定性测试（S05 / T6）。

夹爪动作是离散、慢（0.4–2 s）、阻塞的，**绝不能**跑在 10 ms tick 线程里；
因此互锁落在**执行层的阶段边界**：阶段 B（预就位）之前与阶段 C（起流）之前
各查一次闸门，闸门**非 IDLE** → **拒绝下发任何运动指令**并留下证据。

覆盖

* 闸门 IDLE → 正常执行，报告中**无** ``gripper_interlock_blocked`` 事件；
* 闸门 GRIPPER_BUSY → 阶段 B 前被拒：事件 severity=NEEDS_HUMAN、
  报告 status=blocked、退出码 3，且 ``preposition`` / ``transmit``
  **调用数为 0**（零运动下发断言）；
* 闸门 GRIPPER_UNJUDGED → 同样被拒（**不得**因为「未判定」而放行）；
* 阶段 B 之后闸门才变 BUSY → 阶段 C 起流前被拒，``transmit`` 调用数为 0；
* 闸门返回 None（放行）→ 执行照常完成（正例）。

全部用例跑在 **ManualClock + FakeTransport + ScriptedFeedback** 上：逐 tick
可完全重放，断言是确定性的（零真机依赖）。
"""
from __future__ import annotations

from typing import Any, Dict, List, Optional

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

#: 闸门状态名（与 rm_hw_bridge.gripper_state.MotionGate 的常量一致）
STATE_IDLE = "IDLE"
STATE_BUSY = "GRIPPER_BUSY"
STATE_UNJUDGED = "GRIPPER_UNJUDGED"


class ScriptedGate:
    """可脚本化的运动闸门（鸭子类型：``check()`` + ``state``）。

    ``states`` 按调用顺序返回；``None`` 表示该次检查**放行**。
    最后一次之后重复返回最后一项。
    """

    def __init__(self, states: List[Any], reason: str = "夹爪动作进行中") -> None:
        self._states = list(states)
        self._calls = 0
        self._reason = reason
        self.last_state = STATE_IDLE

    def check(self) -> Optional[Dict[str, Any]]:
        idx = min(self._calls, len(self._states) - 1)
        self._calls += 1
        spec = self._states[idx]
        if spec is None:
            self.last_state = STATE_IDLE
            return None
        self.last_state = spec
        return {
            "state": spec,
            "reason": self._reason,
            "next_action": "等待夹爪动作结束并完成判定后重跑",
        }

    @property
    def state(self) -> str:
        return self.last_state

    @property
    def calls(self) -> int:
        return self._calls


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


def _exit_code(rep):
    return contract.EXIT_CODES[rep["status"]]


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


# --------------------------------------------------------------------- 用例
def test_gate_idle_runs_normally_without_interlock_event(write_traj):
    """闸门 IDLE → 正常执行，报告中**无** gripper_interlock_blocked 事件。"""
    path = write_traj(_traj_payload())
    gate = ScriptedGate([None])
    rep, transport = _run(path, gate)

    assert _events_of(rep, "gripper_interlock_blocked") == [], rep["events"]
    assert rep["status"] == "ok", rep
    assert _exit_code(rep) == 0
    # 正常执行确实下发了运动（正例：闸门不误伤）
    assert transport.count("preposition") == 1
    assert transport.count("transmit") > 0


def test_gate_busy_blocks_before_stage_b_with_zero_motion(write_traj):
    """闸门 GRIPPER_BUSY → 阶段 B 前被拒，**零运动下发**。"""
    path = write_traj(_traj_payload())
    gate = ScriptedGate([STATE_BUSY])
    rep, transport = _run(path, gate)

    blocked = _events_of(rep, "gripper_interlock_blocked")
    assert len(blocked) == 1, rep["events"]
    assert blocked[0]["severity"] == "NEEDS_HUMAN"
    assert "夹爪动作期间禁止运动" in blocked[0]["reason"]
    assert "stage=pre_position" in blocked[0]["reason"]
    # 证据：闸门状态 / 闸门 reason / stage
    joined = " ".join(blocked[0]["evidence"])
    assert "motion_gate_state=GRIPPER_BUSY" in joined, blocked[0]["evidence"]
    assert "motion_gate_reason=" in joined, blocked[0]["evidence"]
    assert "stage=pre_position" in joined, blocked[0]["evidence"]
    # 可粘贴的下一步
    assert blocked[0]["next_action"]
    # 报告口径：status=blocked / 退出码 3
    assert rep["status"] == "blocked", rep
    assert _exit_code(rep) == 3
    # **零运动下发断言**：预就位与流式都没发生
    assert transport.calls_for("preposition") == [], transport.calls
    assert transport.calls_for("transmit") == [], transport.calls
    assert transport.count("preposition") == 0
    assert transport.count("transmit") == 0


def test_gate_unjudged_also_blocks(write_traj):
    """闸门 GRIPPER_UNJUDGED → 同样被拒（**不得**因「未判定」放行）。"""
    path = write_traj(_traj_payload())
    gate = ScriptedGate([STATE_UNJUDGED])
    rep, transport = _run(path, gate)

    blocked = _events_of(rep, "gripper_interlock_blocked")
    assert len(blocked) == 1, rep["events"]
    assert blocked[0]["severity"] == "NEEDS_HUMAN"
    assert "GRIPPER_UNJUDGED" in " ".join(blocked[0]["evidence"])
    assert rep["status"] == "blocked"
    assert _exit_code(rep) == 3
    assert transport.calls_for("preposition") == [], transport.calls
    assert transport.calls_for("transmit") == [], transport.calls


def test_gate_busy_after_stage_b_blocks_before_stream_start(write_traj):
    """阶段 B 之后闸门才变 BUSY → 阶段 C 起流前被拒，**无 transmit**。"""
    path = write_traj(_traj_payload())
    # 第一次检查（阶段 B 前）放行；第二次（阶段 C 起流前）拒绝
    gate = ScriptedGate([None, STATE_BUSY])
    rep, transport = _run(path, gate)

    blocked = _events_of(rep, "gripper_interlock_blocked")
    assert len(blocked) == 1, rep["events"]
    assert blocked[0]["severity"] == "NEEDS_HUMAN"
    assert "stage=stream_start" in blocked[0]["reason"], blocked[0]["reason"]
    assert rep["status"] == "blocked"
    assert _exit_code(rep) == 3
    # 阶段 B 已完成（预就位发生），但**起流没有发生**
    assert transport.count("preposition") == 1, transport.calls
    assert transport.calls_for("transmit") == [], transport.calls


def test_gate_check_exception_blocks_conservatively(write_traj):
    """闸门 ``check()`` 抛异常 → **保守拒绝**（不假装有闸门）。"""

    class BrokenGate:
        state = "UNKNOWN"

        def check(self):
            raise RuntimeError("闸门读失败")

    path = write_traj(_traj_payload())
    rep, transport = _run(path, BrokenGate())

    blocked = _events_of(rep, "gripper_interlock_blocked")
    assert len(blocked) == 1, rep["events"]
    assert blocked[0]["severity"] == "NEEDS_HUMAN"
    assert rep["status"] == "blocked"
    assert transport.calls_for("preposition") == [], transport.calls
    assert transport.calls_for("transmit") == [], transport.calls


def test_gate_returning_none_allows_execution(write_traj):
    """闸门返回 None（放行）→ 执行照常完成（正例）。"""
    path = write_traj(_traj_payload())
    gate = ScriptedGate([None, None, None])
    rep, transport = _run(path, gate)

    assert _events_of(rep, "gripper_interlock_blocked") == []
    assert rep["status"] == "ok", rep
    assert transport.count("preposition") == 1
    assert transport.count("transmit") > 0


def test_no_motion_gate_keeps_legacy_behaviour(write_traj):
    """未注入闸门（``None``）→ 既有行为完全不变（不启用互锁）。"""
    path = write_traj(_traj_payload())
    transport = FakeTransport()

    def _script(_index):
        call = transport.last_call("transmit") or transport.last_call("preposition")
        if call is None:
            return [0.0] * 7
        return list(call["args"]["deg7"])

    feedback = ScriptedFeedback(script=_script, speed_deg_s=0.0, period_s=0.010)
    executor = Executor(transport, feedback, config=CONFIG,
                        clock=ManualClock(period_s=0.010), mode="fake")
    rep = executor.run(path)

    assert _events_of(rep, "gripper_interlock_blocked") == []
    assert rep["status"] == "ok", rep
    assert any("未启用" in e for e in rep["evidence"]), rep["evidence"]


# --------------------------------------------------------------- CLI 接线
def test_shared_motion_gate_is_process_wide_singleton():
    """共享闸门：同进程拿到的是**同一个对象**（夹爪客户端与执行器共用）。"""
    from rm_hw_bridge.gripper_state import shared_motion_gate

    assert shared_motion_gate() is shared_motion_gate()


def test_gripper_action_guard_marks_gate_busy_then_unjudged():
    """夹爪动作护栏：动作期间 BUSY；动作结束（含异常）→ UNJUDGED。"""
    from rm_hw_bridge.gripper_state import MotionGate, gripper_action

    gate = MotionGate()
    with gripper_action("夹取中", gate=gate):
        assert gate.state == MotionGate.STATE_GRIPPER_BUSY
        assert gate.check() is not None
    # 动作结束但**未判定** → 运动仍被拒
    assert gate.state == MotionGate.STATE_GRIPPER_UNJUDGED
    assert gate.check()["state"] == MotionGate.STATE_GRIPPER_UNJUDGED

    # 异常路径同样落到 UNJUDGED（不假装判定过）
    gate2 = MotionGate()
    try:
        with gripper_action("夹取中", gate=gate2):
            raise RuntimeError("夹爪动作超时")
    except RuntimeError:
        pass
    assert gate2.state == MotionGate.STATE_GRIPPER_UNJUDGED


def test_cli_execute_wires_shared_motion_gate(write_traj):
    """CLI 默认接线：报告 evidence 里写明**已启用夹爪互锁**（共享闸门）。"""
    from rm_executor import cli as exec_cli
    from rm_hw_bridge.gripper_state import shared_motion_gate

    # 进程内共享闸门：测试前先复位（保证确定性；不伪造状态）
    shared_motion_gate().reset()

    path = write_traj(_traj_payload())
    rep = exec_cli.execute(mode="fake", trajectory_path=path)

    notes = [e for e in rep["evidence"] if "夹爪互锁" in e]
    assert notes, rep["evidence"]
    assert any("已启用夹爪互锁" in n for n in notes), notes
    assert rep["status"] == "ok", rep


def test_cli_execute_can_disable_interlock_explicitly(write_traj):
    """CLI 显式 ``motion_gate=None`` → evidence 写明**未启用夹爪互锁**。"""
    from rm_executor import cli as exec_cli

    path = write_traj(_traj_payload())
    rep = exec_cli.execute(mode="fake", trajectory_path=path, motion_gate=None)

    notes = [e for e in rep["evidence"] if "夹爪互锁" in e]
    assert notes, rep["evidence"]
    assert any("未启用夹爪互锁" in n for n in notes), notes
    assert rep["status"] == "ok", rep


def test_cli_execute_with_busy_gate_blocks_motion(write_traj):
    """CLI 注入 BUSY 闸门 → 报告 status=blocked / 退出码 3 / 零运动下发。"""
    from rm_executor import cli as exec_cli

    path = write_traj(_traj_payload())
    gate = ScriptedGate([STATE_BUSY])
    rep = exec_cli.execute(mode="fake", trajectory_path=path, motion_gate=gate)

    blocked = _events_of(rep, "gripper_interlock_blocked")
    assert len(blocked) == 1, rep["events"]
    assert rep["status"] == "blocked", rep
    assert _exit_code(rep) == 3
    notes = [e for e in rep["evidence"] if "夹爪互锁" in e]
    assert any("已启用夹爪互锁" in n for n in notes), notes
