"""执行器接入安全包（**唯一动作权威**）的确定性测试（S04 / T5）。

全部用例跑在 **ManualClock + FakeTransport + ScriptedFeedback** 上：逐 tick
可完全重放，断言是确定性的。

覆盖

* **L1 降速**：跟随误差 3°（> 告警阈值 2°、≤ 停机阈值 5°）→ **只**下发
  ``set_speed_scale``，**不** pause / stop；
* **L2 暂停**：跟随误差 6°（> 停机阈值 5°）→ ``pause`` **恰好 1 次**
  + ``safety_l2_pause`` 事件 + 退出码 3；
* **L4 急停**：反馈断流（age > 200 ms）→ ``stop`` **恰好 1 次**
  + ``safety_l4_emergency_stop`` 事件 + 退出码 4；
* **零 SDK 进 tick 线程**：``controller_state`` 只在看门狗线程被调用；
* **S03 事件与退出码口径不回退**：``follow_error_warn`` /
  ``follow_error_exceeded`` / ``feedback_stream_lag`` /
  ``feedback_stream_lost`` 照旧产生，最严重事件仍决定 status 与退出码；
* **负例**：安全层不可用（降级路径）、控制器错误态查询失败（不伪造错误码）、
  声明限值不可用（``limit_check=unavailable``，**不静默**）。
"""
from __future__ import annotations

import threading

from rm_executor import events as events_mod
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

TRAJ_DURATION_S = 0.5
TRAJ_AMPLITUDE_RAD = 0.03


# --------------------------------------------------------------------- 夹具
def _traj_payload():
    """限值内的长轨迹（0 → 0.03 rad，0.5 s）。"""
    return {
        "schema_version": 1,
        "group": "rm75_arm",
        "joint_names": ["joint%d" % i for i in range(1, 8)],
        "units": {"times": "s", "points": "rad"},
        "times": [0.0, TRAJ_DURATION_S],
        "points": [[0.0] * 7, [TRAJ_AMPLITUDE_RAD] * 7],
        "verify": {"ok": True, "checks": {}},
    }


def _mirror_deg(transport, error_deg=0.0):
    """镜像最近一次下发指令角（理想跟随模型）+ 可选恒定跟随误差。"""
    call = transport.last_call("transmit") or transport.last_call("preposition")
    if call is None:
        return [0.0] * 7
    deg = list(call["args"]["deg7"])
    if error_deg and transport.count("transmit") > 0:
        deg = [v + error_deg for v in deg]
    return deg


def _script(transport, error_deg=0.0, stall_from=None, stall_to=None):
    """反馈脚本：镜像指令角；可选断流（返回 ``None`` → 不产出新帧）。"""
    def _inner(_index):
        count = transport.count("transmit")
        if stall_from is not None and count >= stall_from:
            if stall_to is None or count < stall_to:
                return None  # 断流
        return _mirror_deg(transport, error_deg)

    return _inner


def _make_executor(transport, feedback, **kwargs):
    params = dict(config=CONFIG, clock=ManualClock(period_s=0.010), mode="fake")
    params.update(kwargs)
    return Executor(transport, feedback, **params)


def _events_of(rep, type_name):
    return [e for e in rep["events"] if e["type"] == type_name]


def _worst_severity(rep):
    worst = None
    for event in rep["events"]:
        if (worst is None
                or events_mod.SEVERITY_RANK[event["severity"]]
                > events_mod.SEVERITY_RANK[worst]):
            worst = event["severity"]
    return worst


def _exit_code(rep):
    """报告对应的进程退出码（口径与 rm_executor.contract 一致）。"""
    from rm_executor import contract

    return contract.EXIT_CODES[rep["status"]]


class _StateProbeTransport(FakeTransport):
    """记录 ``controller_state`` 的**调用线程**（证明不在 tick 线程）。"""

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.state_query_threads = []

    def controller_state(self):
        self.state_query_threads.append(threading.current_thread().name)
        return super().controller_state()


# --------------------------------------------------------------------- 用例
def test_follow_error_light_triggers_l1_slowdown_only(write_traj):
    """跟随误差 3° → **只** L1 降速（``set_speed_scale``），不 pause / stop。"""
    path = write_traj(_traj_payload())
    transport = FakeTransport()
    feedback = ScriptedFeedback(script=_script(transport, error_deg=3.0),
                                speed_deg_s=0.0, period_s=0.010)
    executor = _make_executor(transport, feedback, arrival_timeout_s=0.3)
    rep = executor.run(path)

    # L1 动作：降速（比例来自安全配置 safety.l1_speed_scale）
    assert transport.count("set_speed_scale") == 1, transport.calls
    call = transport.last_call("set_speed_scale")
    assert call["args"]["scale"] == 0.5
    # **不** pause / stop（没有双重动作）
    assert transport.count("pause") == 0
    assert transport.count("stop") == 0
    # 事件：安全包事件 + S03 既有事件都在
    slow = _events_of(rep, "safety_l1_slowdown")
    assert len(slow) == 1, rep["events"]
    assert slow[0]["severity"] == "RECOVERABLE"
    assert _events_of(rep, "follow_error_warn")
    assert not _events_of(rep, "follow_error_exceeded")
    # 降速不中止执行（栅格继续跑完）
    assert rep["metrics"]["ticks"] > 1
    # 退出码口径：L1 = 3（RECOVERABLE → blocked）
    assert _exit_code(rep) == 3


def test_l1_slowdown_is_auto_recovered_when_error_returns(write_traj):
    """L1 是**唯一**可自动恢复的等级：误差回落到告警阈值内 → 恢复原速。"""
    path = write_traj(_traj_payload())
    transport = FakeTransport()

    def _script_transient(_index):
        """流式前 5 拍注入 3° 误差（触发 L1），之后误差回落到 0°。

        预就位阶段**不**注入（否则预就位容差 0.5° 过不去，执行根本进不了
        流式阶段）。
        """
        call = transport.last_call("transmit") or transport.last_call("preposition")
        if call is None:
            return [0.0] * 7
        deg = list(call["args"]["deg7"])
        if 0 < transport.count("transmit") <= 5:
            deg = [v + 3.0 for v in deg]
        return deg

    feedback = ScriptedFeedback(script=_script_transient, speed_deg_s=0.0,
                                period_s=0.010)
    executor = _make_executor(transport, feedback, arrival_timeout_s=1.0)
    rep = executor.run(path)

    # 降速 1 次 + 恢复原速 1 次（仅 L1 自动恢复）
    scales = [c["args"]["scale"] for c in transport.calls
              if c["method"] == "set_speed_scale"]
    assert scales == [0.5, 1.0], scales
    cleared = _events_of(rep, "safety_l1_slowdown_cleared")
    assert len(cleared) == 1, rep["events"]
    assert cleared[0]["severity"] == "RECOVERABLE"
    # 自动恢复**不**产生 pause / stop
    assert transport.count("pause") == 0
    assert transport.count("stop") == 0


def test_follow_error_heavy_triggers_l2_pause_exactly_once(write_traj):
    """跟随误差 6° → L2：``pause`` **恰好 1 次** + ``safety_l2_pause`` + 退出码 3。"""
    path = write_traj(_traj_payload())
    transport = FakeTransport()
    feedback = ScriptedFeedback(script=_script(transport, error_deg=6.0),
                                speed_deg_s=0.0, period_s=0.010)
    executor = _make_executor(transport, feedback)
    rep = executor.run(path)

    # **单一动作权威**：pause 只发一次（不重复、不叠加 stop）
    assert transport.count("pause") == 1, transport.calls
    assert transport.count("stop") == 0
    assert transport.count("set_speed_scale") == 0
    # 事件：安全包 L2 事件 + S03 既有事件
    pause = _events_of(rep, "safety_l2_pause")
    assert len(pause) == 1, rep["events"]
    assert pause[0]["severity"] == "NEEDS_HUMAN"
    exceeded = _events_of(rep, "follow_error_exceeded")
    assert len(exceeded) == 1, rep["events"]
    assert exceeded[0]["severity"] == "NEEDS_HUMAN"
    # 退出码：L2 = 3
    assert rep["status"] == "blocked"
    assert _exit_code(rep) == 3
    assert _worst_severity(rep) == "NEEDS_HUMAN"


def test_stream_lost_triggers_l4_emergency_stop_exactly_once(write_traj):
    """反馈断流（> 200 ms）→ L4：``stop`` **恰好 1 次** + 急停事件 + 退出码 4。"""
    path = write_traj(_traj_payload())
    transport = FakeTransport()
    feedback = ScriptedFeedback(script=_script(transport, stall_from=3),
                                speed_deg_s=0.0, period_s=0.010)
    executor = _make_executor(transport, feedback)
    rep = executor.run(path)

    assert transport.count("stop") == 1, transport.calls
    assert transport.count("pause") == 0
    stop = _events_of(rep, "safety_l4_emergency_stop")
    assert len(stop) == 1, rep["events"]
    assert stop[0]["severity"] == "FATAL"
    # S03 既有事件口径不回退
    lost = _events_of(rep, "feedback_stream_lost")
    assert len(lost) == 1, rep["events"]
    assert lost[0]["severity"] == "FATAL"
    # 退出码：L4 = 4
    assert rep["status"] == "error"
    assert _exit_code(rep) == 4
    assert _worst_severity(rep) == "FATAL"


def test_controller_state_is_never_called_from_tick_thread(write_traj):
    """控制器错误态采集**只在看门狗线程**：tick 线程 0 次调用。"""
    path = write_traj(_traj_payload())
    transport = _StateProbeTransport()
    feedback = ScriptedFeedback(script=_script(transport), speed_deg_s=0.0,
                                period_s=0.010)
    executor = _make_executor(transport, feedback)
    rep = executor.run(path)

    assert rep["status"] == "ok", rep
    # 采集确实发生了（否则「不在 tick 线程」无从谈起）
    assert transport.count("controller_state") >= 1
    assert transport.state_query_threads
    tick_thread = threading.current_thread().name
    assert tick_thread not in transport.state_query_threads, \
        "controller_state 不得在 tick 线程内调用"
    assert any(t == "rm-controller-watchdog" for t in transport.state_query_threads)


def test_safety_layer_unavailable_falls_back_to_in_place_action(write_traj,
                                                                monkeypatch):
    """负例：安全层不可用 → **降级**就地暂停 + 明确事件（**不静默**）。"""
    import rm_executor.executor as exec_mod

    monkeypatch.setattr(exec_mod, "_import_safety_modules", lambda: None)
    path = write_traj(_traj_payload())
    transport = FakeTransport()
    feedback = ScriptedFeedback(script=_script(transport, error_deg=6.0),
                                speed_deg_s=0.0, period_s=0.010)
    executor = _make_executor(transport, feedback)
    rep = executor.run(path)

    unavailable = _events_of(rep, "safety_layer_unavailable")
    assert len(unavailable) == 1, rep["events"]
    assert unavailable[0]["severity"] == "NEEDS_PARAM"
    # 降级路径仍然暂停（安全兜底不丢）
    assert transport.count("pause") == 1
    fallback = _events_of(rep, "safety_layer_unavailable_fallback")
    assert len(fallback) == 1, rep["events"]
    assert rep["status"] == "blocked"


def test_controller_state_query_failure_does_not_fake_error_codes(write_traj):
    """负例：控制器错误态查询失败 → 不伪造错误码、不炸执行。"""
    path = write_traj(_traj_payload())
    transport = FakeTransport(script={
        "controller_state": {"status": "error", "reason": "控制器句柄失效（注入）"},
    })
    feedback = ScriptedFeedback(script=_script(transport), speed_deg_s=0.0,
                                period_s=0.010)
    executor = _make_executor(transport, feedback)
    rep = executor.run(path)

    assert rep["status"] == "ok", rep["events"]
    # 查询失败不产生任何故障类事件（不伪造 arm_err）
    assert not [e for e in rep["events"] if e["type"].startswith("fault_")]


def test_limit_check_unavailable_is_recorded(write_traj, monkeypatch):
    """负例：声明限值不可用 → 跳过超限判据但**记 limit_check=unavailable**。"""
    path = write_traj(_traj_payload())
    transport = FakeTransport()
    feedback = ScriptedFeedback(script=_script(transport), speed_deg_s=0.0,
                                period_s=0.010)
    executor = _make_executor(transport, feedback)
    monkeypatch.setattr(executor, "_load_limits", lambda: None)
    rep = executor.run(path)

    joined = " ".join(rep["evidence"])
    assert "limit_check=unavailable" in joined, rep["evidence"]
    # 限值不可用**不**被当成越限（不误停机）
    assert transport.count("pause") == 0
    assert transport.count("stop") == 0
    assert not _events_of(rep, "fault_limit_exceeded")


def test_normal_run_has_no_safety_action(write_traj):
    """正常执行 → 无任何安全动作、无安全事件（口径干净）。"""
    path = write_traj(_traj_payload())
    transport = FakeTransport()
    feedback = ScriptedFeedback(script=_script(transport), speed_deg_s=0.0,
                                period_s=0.010)
    executor = _make_executor(transport, feedback, arrival_timeout_s=1.0)
    rep = executor.run(path)

    assert transport.count("pause") == 0
    assert transport.count("stop") == 0
    assert transport.count("set_speed_scale") == 0
    assert not [e for e in rep["events"] if e["type"].startswith("safety_")]
    assert not [e for e in rep["events"] if e["type"].startswith("fault_")]
    assert rep["status"] == "ok", rep["events"]
