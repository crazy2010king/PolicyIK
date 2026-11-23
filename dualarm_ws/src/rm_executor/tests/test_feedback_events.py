"""rm_executor 反馈闭环异常判定（T5）确定性测试。

全部用例跑在 **ManualClock + FakeTransport + ScriptedFeedback** 上：
逐 tick 可完全重放，断言是确定性的，不依赖真实机器负载。

覆盖四类运行期异常 → **结构化事件**（S04 的 L1–L4 安全链路直接复用这些
事件类型与阈值口径）：

* 跟随误差两档：``>2°`` → RECOVERABLE ``follow_error_warn``；
  ``>5°`` → NEEDS_HUMAN ``follow_error_exceeded`` + ``transport.pause()``；
* 断流两档：``>50 ms`` → RECOVERABLE ``feedback_stream_lag``；
  ``>200 ms`` → FATAL ``feedback_stream_lost`` + ``transport.stop()``；
* 控制器抢轨迹：FATAL ``controller_trajectory_conflict`` + 立即停机
  （**查询发生在独立看门狗线程**，绝不进 tick 线程）；
* 严重度 → 报告 status / 退出码的映射表。
"""
from __future__ import annotations

import threading

from rm_executor import contract, events as events_mod
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

#: 轨迹总长（秒）—— 需足够长，使断流判据（200 ms）在轨迹结束前触发
TRAJ_DURATION_S = 0.5
#: 轨迹位移（rad）—— 1.72°，小到断流时冻结的位置不会误触跟随误差预警
TRAJ_AMPLITUDE_RAD = 0.03


# --------------------------------------------------------------------- 夹具
def _traj_payload():
    """限值内的长轨迹（0 → 0.03 rad，0.5 s；51 个 10 ms 栅格点）。"""
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
    """反馈脚本：镜像指令角；可选在 ``[stall_from, stall_to)`` 次 transmit
    之后**断流**（返回 ``None`` → 不产出新帧 → age 增长）。
    """
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


class _ThreadProbeTransport(FakeTransport):
    """记录 ``current_trajectory_type`` 的**调用线程**（证明不在 tick 线程）。"""

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.type_query_threads = []

    def current_trajectory_type(self):
        self.type_query_threads.append(threading.current_thread().name)
        return super().current_trajectory_type()


# --------------------------------------------------------------------- 用例
def test_follow_error_exceeded_pauses_execution_and_blocks(write_traj):
    """注入 6° 跟随误差 → ``follow_error_exceeded`` + ``pause`` + blocked。"""
    path = write_traj(_traj_payload())
    transport = FakeTransport()
    feedback = ScriptedFeedback(script=_script(transport, error_deg=6.0),
                                speed_deg_s=0.0, period_s=0.010)
    executor = _make_executor(transport, feedback)
    rep = executor.run(path)

    # 事件：NEEDS_HUMAN（不是 FATAL——超限需人工介入，不是硬红线）
    exceeded = _events_of(rep, "follow_error_exceeded")
    assert len(exceeded) == 1, rep["events"]
    assert exceeded[0]["severity"] == "NEEDS_HUMAN"
    assert exceeded[0]["next_action"]
    # 证据里写明偏差值与阈值
    evidence = " ".join(exceeded[0]["evidence"])
    assert "follow_error_deg=6.0000" in evidence
    assert "follow_stop_deg=5.00" in evidence
    # 安全动作：暂停（**不**急停——降速/缓停处置属 S04）
    assert transport.count("pause") == 1
    assert transport.count("stop") == 0
    # 报告：最严重事件决定 status
    assert rep["status"] == "blocked"
    assert _worst_severity(rep) == "NEEDS_HUMAN"
    assert events_mod.exit_code_for_severity(_worst_severity(rep)) == 3


def test_follow_error_warn_is_recoverable_and_keeps_running(write_traj):
    """注入 3° 跟随误差 → RECOVERABLE ``follow_error_warn``，执行继续。"""
    path = write_traj(_traj_payload())
    transport = FakeTransport()
    feedback = ScriptedFeedback(script=_script(transport, error_deg=3.0),
                                speed_deg_s=0.0, period_s=0.010)
    executor = _make_executor(transport, feedback, arrival_timeout_s=0.3)
    rep = executor.run(path)

    warn = _events_of(rep, "follow_error_warn")
    assert len(warn) == 1, rep["events"]
    assert warn[0]["severity"] == "RECOVERABLE"
    assert "follow_error_deg=3.0000" in " ".join(warn[0]["evidence"])
    # 3° < 5° → 不暂停、不急停
    assert transport.count("pause") == 0
    assert transport.count("stop") == 0
    # 3° > 2° 不会升级成超限事件
    assert not _events_of(rep, "follow_error_exceeded")
    # 栅格流**继续跑完**（不是提前中止）
    assert rep["metrics"]["ticks"] > 1


def test_feedback_stream_lost_is_fatal_and_stops_machine(write_traj):
    """注入断流（>200 ms）→ FATAL ``feedback_stream_lost`` + ``stop`` + error。"""
    path = write_traj(_traj_payload())
    transport = FakeTransport()
    feedback = ScriptedFeedback(script=_script(transport, stall_from=3),
                                speed_deg_s=0.0, period_s=0.010)
    executor = _make_executor(transport, feedback)
    rep = executor.run(path)

    lost = _events_of(rep, "feedback_stream_lost")
    assert len(lost) == 1, rep["events"]
    assert lost[0]["severity"] == "FATAL"
    evidence = " ".join(lost[0]["evidence"])
    assert "stream_stop_ms=200" in evidence
    assert "stream_age_ms=" in evidence
    # 硬红线：立即停机
    assert transport.count("stop") == 1
    # 报告 status=error（退出码 4）
    assert rep["status"] == "error"
    assert _worst_severity(rep) == "FATAL"
    assert events_mod.exit_code_for_severity(_worst_severity(rep)) == 4
    # 断流后不再继续下发（提前中止）
    assert rep["metrics"]["ticks"] < int(TRAJ_DURATION_S / CONFIG.period_s)


def test_feedback_stream_lag_is_recoverable_when_stream_recovers(write_traj):
    """断流 50–200 ms → RECOVERABLE ``feedback_stream_lag``，恢复后不升级。"""
    path = write_traj(_traj_payload())
    transport = FakeTransport()
    # 第 3 次 transmit 起断流，第 9 次起恢复（断流窗口 ≈ 60 ms）
    feedback = ScriptedFeedback(script=_script(transport, stall_from=3, stall_to=9),
                                speed_deg_s=0.0, period_s=0.010)
    executor = _make_executor(transport, feedback, arrival_timeout_s=0.3)
    rep = executor.run(path)

    lag = _events_of(rep, "feedback_stream_lag")
    assert len(lag) == 1, rep["events"]
    assert lag[0]["severity"] == "RECOVERABLE"
    assert "stream_warn_ms=50" in " ".join(lag[0]["evidence"])
    # 未升级为 FATAL：既不停机，也不产生断流丢失事件
    assert not _events_of(rep, "feedback_stream_lost")
    assert transport.count("stop") == 0


def test_controller_trajectory_conflict_is_fatal_and_stops_immediately(write_traj):
    """控制器抢轨迹 → FATAL ``controller_trajectory_conflict`` + 立即停机。"""
    path = write_traj(_traj_payload())
    transport = FakeTransport(script={
        "current_trajectory_type": {
            "status": "ok",
            "reason": "控制器处于规划态（注入）",
            "type": 1,
            "type_name": "RM_PLANNING",
        }
    })
    feedback = ScriptedFeedback(script=_script(transport), speed_deg_s=0.0,
                                period_s=0.010)
    executor = _make_executor(transport, feedback)
    rep = executor.run(path)

    conflict = _events_of(rep, "controller_trajectory_conflict")
    assert len(conflict) == 1, rep["events"]
    assert conflict[0]["severity"] == "FATAL"
    evidence = " ".join(conflict[0]["evidence"])
    assert "expected_type_name=RM_NO_PLANNING_E" in evidence
    assert "observed_type_name=RM_PLANNING" in evidence
    # 立即停机（硬红线）
    assert transport.count("stop") == 1
    assert rep["status"] == "error"
    assert events_mod.exit_code_for_severity(_worst_severity(rep)) == 4


class _PlanningDuringPrepositionTransport(FakeTransport):
    """预就位期间控制器处于**规划态**（真机行为：``rm_movej`` → 控制器规划）。

    回归（2026-11-19 真机实测）：看门狗若在阶段 B 就绪，会把我们自己下发的
    预就位规划运动误判为“控制器抢轨迹”（FATAL + 急停）。因此看门狗必须在
    阶段 B 结束、进入纯透传阶段之后才启动。
    """

    def preposition(self, deg7, speed=10):
        result = super().preposition(deg7, speed=speed)
        self.calls.append(("preposition_done",))
        return result

    def current_trajectory_type(self):
        # 预就位尚未完成 → 规划态；完成后 → 纯透传态
        planning = not any(c[0] == "preposition_done" for c in self.calls)
        if planning:
            return {"status": "ok", "reason": "控制器规划态（预就位中）",
                    "type": 1, "type_name": "RM_JOINT_SPACE_PLANNING_E"}
        return {"status": "ok", "reason": "", "type": 0,
                "type_name": "RM_NO_PLANNING_E"}


def test_blend_trajectory_type_is_accepted_during_streaming(write_traj):
    """透传阶段控制器处于**混合轨迹规划**（RM_BLEND_TRAJECTORY_PLANNING_E）→ 不算抢轨迹。

    回归（2026-11-19 真机实测）：canfd 透传时控制器就是混合规划态；若期望值只写
    RM_NO_PLANNING_E，会把正常透传误判为“控制器抢轨迹”（FATAL + 急停）。
    """
    path = write_traj(_traj_payload())
    transport = FakeTransport(script={
        "current_trajectory_type": {
            "status": "ok", "reason": "混合轨迹规划（canfd 真机行为）",
            "type": 6, "type_name": "RM_BLEND_TRAJECTORY_PLANNING_E",
        }
    })
    feedback = ScriptedFeedback(script=_script(transport), speed_deg_s=0.0,
                                period_s=0.010)
    executor = _make_executor(transport, feedback)
    rep = executor.run(path)

    assert not _events_of(rep, "controller_trajectory_conflict"), rep["events"]
    assert transport.count("stop") == 0


def test_spline_curve_motion_type_is_accepted_during_streaming(write_traj):
    """透传阶段控制器处于**样条曲线运动**（RM_SPLINE_CURVE_MOTION_PLANNING_E）→ 不算抢轨迹。

    回归（2026-11-19 真机实测）：``trajectory_mode=1``（曲线拟合，本项目默认）
    时控制器上报的正是这个类型 —— 把它当抢轨迹会 FATAL + 急停。
    """
    path = write_traj(_traj_payload())
    transport = FakeTransport(script={
        "current_trajectory_type": {
            "status": "ok", "reason": "曲线拟合（canfd 真机行为）",
            "type": 4, "type_name": "RM_SPLINE_CURVE_MOTION_PLANNING_E",
        }
    })
    feedback = ScriptedFeedback(script=_script(transport), speed_deg_s=0.0,
                                period_s=0.010)
    executor = _make_executor(transport, feedback)
    rep = executor.run(path)

    assert not _events_of(rep, "controller_trajectory_conflict"), rep["events"]
    assert transport.count("stop") == 0


def test_other_client_joint_space_trajectory_is_a_conflict(write_traj):
    """**别人下发的关节空间轨迹**仍然是真抢轨迹（必须 FATAL + 停机）。"""
    path = write_traj(_traj_payload())
    transport = FakeTransport(script={
        "current_trajectory_type": {
            "status": "ok", "reason": "另一个客户端下发了关节空间轨迹",
            "type": 1, "type_name": "RM_JOINT_SPACE_PLANNING_E",
        }
    })
    feedback = ScriptedFeedback(script=_script(transport), speed_deg_s=0.0,
                                period_s=0.010)
    executor = _make_executor(transport, feedback)
    rep = executor.run(path)

    conflict = _events_of(rep, "controller_trajectory_conflict")
    assert len(conflict) == 1, rep["events"]
    assert conflict[0]["severity"] == "FATAL"
    assert transport.count("stop") == 1


def test_cartesian_arc_type_is_accepted_during_streaming(write_traj):
    """透传期间控制器上报**笛卡尔圆弧**（RM_CARTESIAN_ARC_PLANNING_E）→ 不算抢轨迹。

    回归（2026-11-19 真机实测）：同一段 CANFD 透传里，控制器先后上报过 spline 与
    cartesian-arc —— 它在曲线拟合下会切换内部形态，单次采样不能判定"被抢"。
    """
    path = write_traj(_traj_payload())
    transport = FakeTransport(script={
        "current_trajectory_type": {
            "status": "ok", "reason": "笛卡尔圆弧（canfd 真机行为）",
            "type": 3, "type_name": "RM_CARTESIAN_ARC_PLANNING_E",
        }
    })
    feedback = ScriptedFeedback(script=_script(transport), speed_deg_s=0.0,
                                period_s=0.010)
    executor = _make_executor(transport, feedback)
    rep = executor.run(path)

    assert not _events_of(rep, "controller_trajectory_conflict"), rep["events"]
    assert transport.count("stop") == 0


def test_transient_conflict_does_not_stop_the_arm(write_traj):
    """**单次**外部轨迹类型（瞬态）不得急停 —— 需连续确认才算抢轨迹。

    回归（2026-11-19 真机实测）：控制器在透传期间会短暂上报其它形态。
    """
    path = write_traj(_traj_payload())

    class _OneShotConflictTransport(FakeTransport):
        """只有第一次查询返回外部轨迹类型，之后恢复透传态。"""

        def __init__(self, **kwargs):
            super().__init__(**kwargs)
            self._queries = 0

        def current_trajectory_type(self):
            self._queries += 1
            if self._queries == 1:
                return {"status": "ok", "reason": "瞬态",
                        "type": 1, "type_name": "RM_JOINT_SPACE_PLANNING_E"}
            return {"status": "ok", "reason": "", "type": 0,
                    "type_name": "RM_NO_PLANNING_E"}

    transport = _OneShotConflictTransport()
    feedback = ScriptedFeedback(script=_script(transport), speed_deg_s=0.0,
                                period_s=0.010)
    executor = _make_executor(transport, feedback)
    rep = executor.run(path)

    assert not _events_of(rep, "controller_trajectory_conflict"), rep["events"]
    assert transport.count("stop") == 0


def test_preposition_planning_state_is_not_a_controller_conflict(write_traj):
    """阶段 B（预就位，控制器规划态）**不得**报抢轨迹。"""
    path = write_traj(_traj_payload())
    transport = _PlanningDuringPrepositionTransport()
    feedback = ScriptedFeedback(script=_script(transport), speed_deg_s=0.0,
                                period_s=0.010)
    executor = _make_executor(transport, feedback)
    rep = executor.run(path)

    assert not _events_of(rep, "controller_trajectory_conflict"), rep["events"]
    assert transport.count("stop") == 0


def test_current_trajectory_type_is_never_called_from_tick_thread(write_traj):
    """控制器状态查询**只在看门狗线程**：tick 线程 0 次调用。"""
    path = write_traj(_traj_payload())
    transport = _ThreadProbeTransport()
    feedback = ScriptedFeedback(script=_script(transport), speed_deg_s=0.0,
                                period_s=0.010)
    executor = _make_executor(transport, feedback)
    rep = executor.run(path)

    assert rep["status"] == "ok", rep
    # 查询确实发生了（否则「不在 tick 线程」无从谈起）
    assert transport.count("current_trajectory_type") >= 1
    assert transport.type_query_threads
    tick_thread = threading.current_thread().name
    assert tick_thread not in transport.type_query_threads, \
        "current_trajectory_type 不得在 tick 线程内调用"
    # 调用发生在看门狗线程（记录可事后审计）
    checks = executor.watchdog_checks
    assert checks
    assert all(c["thread"] != tick_thread for c in checks)
    assert any(c["thread"] == "rm-controller-watchdog" for c in checks)


def test_severity_to_status_and_exit_code_mapping(write_traj):
    """严重度 → 报告 status / 退出码的映射表（RECOVERABLE→blocked=3 等）。"""
    assert events_mod.SEVERITY_STATUS == {
        "INFO": "ok",
        "RECOVERABLE": "blocked",
        "NEEDS_PARAM": "blocked",
        "NEEDS_HUMAN": "blocked",
        "FATAL": "error",
    }
    assert events_mod.SEVERITY_EXIT_CODES == {
        "INFO": 0,
        "RECOVERABLE": 3,
        "NEEDS_PARAM": 3,
        "NEEDS_HUMAN": 3,
        "FATAL": 4,
    }
    # 三个负例的报告 status 都等于「最严重事件」推导出的 status
    path = write_traj(_traj_payload())

    transport = FakeTransport()
    executor = _make_executor(
        transport,
        ScriptedFeedback(script=_script(transport, error_deg=6.0),
                         speed_deg_s=0.0, period_s=0.010),
    )
    rep = executor.run(path)
    assert rep["status"] == events_mod.SEVERITY_STATUS[_worst_severity(rep)]

    transport = FakeTransport()
    executor = _make_executor(
        transport,
        ScriptedFeedback(script=_script(transport, stall_from=3),
                         speed_deg_s=0.0, period_s=0.010),
    )
    rep = executor.run(path)
    assert rep["status"] == events_mod.SEVERITY_STATUS[_worst_severity(rep)]


def test_thresholds_are_frozen_in_contract(write_traj):
    """阈值唯一来源 = ``rm_executor.contract``（执行层不内嵌第二份常量）。"""
    assert contract.FOLLOW_WARN_DEG == 2.0
    assert contract.FOLLOW_STOP_DEG == 5.0
    assert contract.STREAM_WARN_MS == 50
    assert contract.STREAM_STOP_MS == 200
    # 两档必须**分档**（预警档 < 停机档），否则退化成单档
    assert contract.FOLLOW_WARN_DEG < contract.FOLLOW_STOP_DEG
    assert contract.STREAM_WARN_MS < contract.STREAM_STOP_MS
