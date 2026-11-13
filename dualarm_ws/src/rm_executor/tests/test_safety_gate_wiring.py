"""执行器接入**安全闸门**（进程外强制停机）的确定性测试（S04 / T7）。

全部用例跑在 **ManualClock + FakeTransport + ScriptedFeedback** 上：逐 tick
可完全重放，断言是确定性的。

覆盖

* 运行中收到闸门停机请求 → **立即** L4 急停（``transport.stop()`` 恰好 1 次）
  + ``safety_l4_emergency_stop`` 事件（reason 写明「外部闸门请求」）
  + tick 下一拍退出 + 退出码 4；
* 执行**开始前**就收到请求 → **不下发任何运动指令**（无 preposition /
  无 transmit）；
* 闸门重复触发 → 停机**只发一次**（幂等）。
"""
from __future__ import annotations

from rm_executor.clock import ManualClock
from rm_executor.executor import Executor, ExecutorConfig
from rm_executor.feedback import ScriptedFeedback
from rm_executor.transport import FakeTransport
from rm_safety.gate import InProcessGate

#: 确定性执行参数（与 workcell/rm_workcell.yaml::execution 一致）
CONFIG = ExecutorConfig(
    period_s=0.010,
    follow=False,
    trajectory_mode=1,
    radio=30,
    pre_position_speed=10,
)


def _traj_payload():
    """限值内的长轨迹（0 → 0.03 rad，0.5 s）。"""
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
    from rm_executor import contract

    return contract.EXIT_CODES[rep["status"]]


# --------------------------------------------------------------------- 用例
def test_gate_stop_triggers_l4_emergency_stop_once(write_traj):
    """运行中收到闸门请求 → 立即 L4 急停（stop 恰好 1 次）+ 事件 + 退出码 4。"""
    path = write_traj(_traj_payload())
    transport = FakeTransport()
    gate = InProcessGate()
    fired = {"done": False}

    def _script(_index):
        """第 3 次下发后请求闸门停机（模拟外部操作者按下急停）。"""
        if transport.count("transmit") >= 3 and not fired["done"]:
            fired["done"] = True
            gate.request_stop(level="L4", reason="外部闸门请求", source="socket")
        call = transport.last_call("transmit") or transport.last_call("preposition")
        if call is None:
            return [0.0] * 7
        return list(call["args"]["deg7"])

    feedback = ScriptedFeedback(script=_script, speed_deg_s=0.0, period_s=0.010)
    executor = Executor(transport, feedback, config=CONFIG,
                        clock=ManualClock(period_s=0.010), mode="fake", gate=gate)
    rep = executor.run(path)

    # 急停恰好 1 次（唯一动作权威：闸门 → supervisor → actuator → transport）
    assert transport.count("stop") == 1, transport.calls
    # 事件：safety_l4_emergency_stop，reason 写明「外部闸门请求」
    stops = _events_of(rep, "safety_l4_emergency_stop")
    assert len(stops) == 1, rep["events"]
    assert stops[0]["severity"] == "FATAL"
    assert "外部闸门请求" in stops[0]["reason"]
    # tick 提前退出（没有跑完整条轨迹）
    assert rep["metrics"]["ticks"] < 50, rep["metrics"]
    # 退出码口径：FATAL → error → 4
    assert _exit_code(rep) == 4
    # 安全层状态：当前等级 L4
    assert executor.safety_status["current_level"] == "L4"


def test_gate_stop_before_run_does_not_move(write_traj):
    """执行开始前就收到闸门请求 → **不下发任何运动指令**。"""
    path = write_traj(_traj_payload())
    transport = FakeTransport()
    gate = InProcessGate()
    gate.request_stop(level="L4", reason="外部闸门请求", source="socket")

    feedback = ScriptedFeedback(script=lambda _i: [0.0] * 7, speed_deg_s=0.0,
                                period_s=0.010)
    executor = Executor(transport, feedback, config=CONFIG,
                        clock=ManualClock(period_s=0.010), mode="fake", gate=gate)
    rep = executor.run(path)

    assert transport.count("preposition") == 0
    assert transport.count("transmit") == 0
    assert _events_of(rep, "safety_l4_emergency_stop")
    assert _exit_code(rep) == 4


def test_repeated_gate_requests_stop_once(write_traj):
    """闸门重复触发 → 停机**只发一次**（幂等，不重复下发急停）。"""
    path = write_traj(_traj_payload())
    transport = FakeTransport()
    gate = InProcessGate()
    fired = {"n": 0}

    def _script(_index):
        if transport.count("transmit") >= 3 and fired["n"] < 2:
            fired["n"] += 1
            gate.request_stop(level="L4", reason="外部闸门请求", source="socket")
        call = transport.last_call("transmit") or transport.last_call("preposition")
        if call is None:
            return [0.0] * 7
        return list(call["args"]["deg7"])

    feedback = ScriptedFeedback(script=_script, speed_deg_s=0.0, period_s=0.010)
    executor = Executor(transport, feedback, config=CONFIG,
                        clock=ManualClock(period_s=0.010), mode="fake", gate=gate)
    rep = executor.run(path)

    assert transport.count("stop") == 1, transport.calls
    assert len(_events_of(rep, "safety_l4_emergency_stop")) == 1
