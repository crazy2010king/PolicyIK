"""5 ms 实时执行环的单测（S03-T04）。

设计要点
--------
* **确定性节拍**：虚拟时钟下每拍间隔**精确 5 ms**（不依赖真实硬件节拍）。
* **七闸在环内生效**：越限指令被钳位，且违规**不被吞掉**。
* **平顺收尾**：轨迹走完不多发、不越界。
* **看门狗超限 ⇒ 安全态**（缓停 / 急停）并上报稳定 reason。
* **零 SDK 依赖**：全程走 :class:`ScriptedTransport` 替身。
* 负例覆盖：非法轨迹点、非法配置、状态缺失、断链、实时调度降级。
"""

from __future__ import annotations

import dataclasses
import math
import os
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = REPO_ROOT / "src"

if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from rm75_drake.device.exec_loop import (  # noqa: E402
    REASON_COMPLETED,
    REASON_SAFE_STOP,
    REASON_TRANSPORT_ERROR,
    ExecLoop,
    ExecLoopConfig,
    SystemClock,
    VirtualClock,
    apply_realtime_scheduling,
)
from rm75_drake.device.safety import (  # noqa: E402
    REASON_DELTA_CLAMP,
    REASON_STATE_AGE,
    REASON_WATCHDOG,
    load_safety_limits,
)
from rm75_drake.device.transport import (  # noqa: E402
    REASON_LINK_DOWN,
    ScriptedTransport,
    StateFrame,
    TransportError,
)

#: 5 ms 执行周期（s）。
DT = 0.005

#: 一个**远离围栏**的合法构型（TCP 高度 0.787 m > 桌面 0.75 m，
#: 离基座轴线 0.256 m > 奇异半径 0.15 m）。
BASE = (0.5, 0.3, -0.8, 0.3, 0.0, 0.0, 0.0)

LIMITS = load_safety_limits()


def make_points(count: int, step: float = 0.0001) -> list:
    """生成 ``count`` 个相邻点（joint_1 每拍 +``step`` rad）。"""
    return [
        (BASE[0] + index * step,) + BASE[1:] for index in range(count)
    ]


def fresh_frame(clock: VirtualClock) -> StateFrame:
    """构造一帧「刚刚到达」的状态（age = 0）。"""
    return StateFrame(
        joint_positions_rad=BASE,
        joint_currents_ma=(10.0,) * 7,
        joint_en_flags=(True,) * 7,
        joint_err_codes=(0,) * 7,
        received_at=clock.now_s,
    )


def open_transport() -> ScriptedTransport:
    transport = ScriptedTransport()
    transport.open()
    return transport


# ── 确定性节拍 ──────────────────────────────────────────────────────────
def test_loop_runs_with_exact_5ms_ticks() -> None:
    """虚拟时钟下每拍间隔精确 5 ms，抖动为 0。"""
    clock = VirtualClock()
    transport = open_transport()
    loop = ExecLoop(
        transport,
        make_points(5),
        LIMITS,
        clock=clock,
        state_provider=lambda: fresh_frame(clock),
    )
    result = loop.run()

    assert result.ticks == 5
    assert result.commands_sent == 5
    assert result.final_state == REASON_COMPLETED
    assert result.ok is True
    assert len(result.tick_intervals_s) == 4
    for interval in result.tick_intervals_s:
        assert interval == pytest.approx(DT, abs=1e-12)
    assert result.jitter.count == 4
    assert result.jitter.max == pytest.approx(0.0, abs=1e-12)
    assert result.jitter.p999 == pytest.approx(0.0, abs=1e-12)
    assert transport.called("move_j_canfd") == 5


def test_loop_uses_only_the_canfd_motion_channel() -> None:
    """环内唯一运动通道是 ``move_j_canfd``（不得混入其他动作）。"""
    clock = VirtualClock()
    transport = open_transport()
    loop = ExecLoop(
        transport,
        make_points(3),
        LIMITS,
        clock=clock,
        state_provider=lambda: fresh_frame(clock),
    )
    loop.run()
    assert set(transport.call_names()) == {"move_j_canfd"}


def test_loop_records_command_latency() -> None:
    """命令→执行延迟被统计（用会耗时工作的替身制造非零延迟）。"""

    class _SlowTransport(ScriptedTransport):
        def __init__(self, clock: VirtualClock, delay_s: float) -> None:
            super().__init__()
            self._clock = clock
            self._delay = delay_s

        def move_j_canfd(self, joint_rad, *, follow: bool = True) -> None:
            self._clock.advance(self._delay)
            super().move_j_canfd(joint_rad, follow=follow)

    clock = VirtualClock()
    transport = _SlowTransport(clock, 0.0007)
    transport.open()
    loop = ExecLoop(
        transport,
        make_points(3),
        LIMITS,
        clock=clock,
        state_provider=lambda: fresh_frame(clock),
    )
    result = loop.run()

    assert result.latency.count == 3
    assert result.latency.max == pytest.approx(0.0007, abs=1e-9)
    assert result.latency.p50 == pytest.approx(0.0007, abs=1e-9)


def test_loop_reports_jitter_when_tick_overruns_period() -> None:
    """拍内工作超过周期 ⇒ 抖动被如实记录（不隐藏）。"""

    class _SlowTransport(ScriptedTransport):
        def __init__(self, clock: VirtualClock, delay_s: float) -> None:
            super().__init__()
            self._clock = clock
            self._delay = delay_s

        def move_j_canfd(self, joint_rad, *, follow: bool = True) -> None:
            self._clock.advance(self._delay)
            super().move_j_canfd(joint_rad, follow=follow)

    clock = VirtualClock()
    transport = _SlowTransport(clock, 0.006)  # 6 ms > 5 ms 周期
    transport.open()
    loop = ExecLoop(
        transport,
        make_points(4),
        LIMITS,
        clock=clock,
        state_provider=lambda: fresh_frame(clock),
    )
    result = loop.run()

    assert result.jitter.count == 3
    assert result.jitter.max == pytest.approx(0.001, abs=1e-9)


# ── 七闸在环内生效 ──────────────────────────────────────────────────────
def test_loop_clamps_out_of_range_command_and_keeps_violations() -> None:
    """越限指令被钳位；违规进结果（不吞掉）；下发值仍在安全区间内。"""
    clock = VirtualClock()
    transport = open_transport()
    jump = list(BASE)
    jump[0] = BASE[0] + 0.5  # 单拍跳 0.5 rad ≫ ω_max·dt
    loop = ExecLoop(
        transport,
        [BASE, tuple(jump)],
        LIMITS,
        clock=clock,
        state_provider=lambda: fresh_frame(clock),
    )
    result = loop.run()

    assert REASON_DELTA_CLAMP in result.reasons
    assert result.commands_sent == 2
    sent = transport.calls[-1][1]
    max_step = LIMITS.max_velocity_rad_s[0] * DT
    assert sent[0] <= BASE[0] + max_step + 1e-12
    assert sent[0] < BASE[0] + 0.5
    # 软限位仍然成立（钳位后仍满足 G1）
    for index in range(7):
        assert LIMITS.soft_lower_rad[index] <= sent[index] <= LIMITS.soft_upper_rad[index]


def test_loop_reports_violations_via_sink_and_events() -> None:
    """违规通过可注入 sink / 事件缓冲外送（环内无日志 I/O）。"""
    clock = VirtualClock()
    transport = open_transport()
    seen: list = []
    jump = list(BASE)
    jump[0] = BASE[0] + 0.5
    loop = ExecLoop(
        transport,
        [BASE, tuple(jump)],
        LIMITS,
        clock=clock,
        state_provider=lambda: fresh_frame(clock),
        sink=lambda event, payload: seen.append((event, payload)),
    )
    result = loop.run()

    events = [name for name, _ in seen]
    assert "violation" in events
    assert events[-1] == "completed"
    assert any(
        payload.get("reason") == REASON_DELTA_CLAMP
        for name, payload in seen
        if name == "violation"
    )
    assert result.violations


# ── 轨迹收尾 ────────────────────────────────────────────────────────────
def test_loop_finishes_without_extra_commands() -> None:
    """轨迹走完：**不多发**（下发次数 == 轨迹点数），也不越界。"""
    clock = VirtualClock()
    transport = open_transport()
    points = make_points(4)
    loop = ExecLoop(
        transport,
        points,
        LIMITS,
        clock=clock,
        state_provider=lambda: fresh_frame(clock),
    )
    result = loop.run()

    assert result.commands_sent == len(points)
    assert transport.called("move_j_canfd") == len(points)
    last_sent = transport.calls[-1][1]
    assert last_sent[0] == pytest.approx(points[-1][0], abs=1e-12)
    assert last_sent[0] <= points[-1][0] + 1e-12  # 不越界（不超发）
    for index in range(7):
        assert LIMITS.soft_lower_rad[index] <= last_sent[index] <= LIMITS.soft_upper_rad[index]


def test_loop_tail_hold_ticks_repeats_last_point() -> None:
    """``tail_hold_ticks`` 显式开启时才会保持最后一拍（默认不多发）。"""
    clock = VirtualClock()
    transport = open_transport()
    points = make_points(2)
    loop = ExecLoop(
        transport,
        points,
        LIMITS,
        config=ExecLoopConfig(tail_hold_ticks=2),
        clock=clock,
        state_provider=lambda: fresh_frame(clock),
    )
    result = loop.run()

    assert result.commands_sent == 4
    assert transport.calls[-1][1][0] == pytest.approx(points[-1][0], abs=1e-12)


# ── 安全态 ──────────────────────────────────────────────────────────────
def test_watchdog_timeout_triggers_slow_stop() -> None:
    """状态帧陈旧（心跳缺失 > 缓停阈值）⇒ 缓停 + 稳定 reason。"""
    clock = VirtualClock()
    transport = open_transport()
    limits = dataclasses.replace(LIMITS, state_age_timeout_s=1.0)
    stale = StateFrame(
        joint_positions_rad=BASE,
        joint_currents_ma=(10.0,) * 7,
        joint_en_flags=(True,) * 7,
        received_at=clock.now_s - 0.06,  # 12 个心跳周期缺失
    )
    loop = ExecLoop(
        transport,
        make_points(5),
        limits,
        clock=clock,
        state_provider=lambda: stale,
    )
    result = loop.run()

    assert result.final_state == REASON_SAFE_STOP
    assert result.stop_reason == REASON_WATCHDOG
    assert result.commands_sent == 0
    assert transport.called("slow_stop") == 1
    assert transport.called("move_j_canfd") == 0


def test_missing_state_triggers_emergency_stop() -> None:
    """读不到状态 ⇒ 按「状态超时」处理并急停（安全优先）。"""
    clock = VirtualClock()
    transport = open_transport()
    loop = ExecLoop(
        transport,
        make_points(3),
        LIMITS,
        clock=clock,
        state_provider=lambda: None,
    )
    result = loop.run()

    assert result.final_state == REASON_SAFE_STOP
    assert result.stop_reason == REASON_STATE_AGE
    assert transport.called("emergency_stop") == 1
    assert transport.called("move_j_canfd") == 0


def test_missing_state_is_allowed_when_state_is_not_required() -> None:
    """``require_state=False`` ⇒ 允许离线 dry-run（age 明确记 0）。"""
    clock = VirtualClock()
    transport = open_transport()
    loop = ExecLoop(
        transport,
        make_points(3),
        LIMITS,
        config=ExecLoopConfig(require_state=False),
        clock=clock,
        state_provider=lambda: None,
    )
    result = loop.run()

    assert result.final_state == REASON_COMPLETED
    assert result.commands_sent == 3
    assert REASON_STATE_AGE not in result.reasons


def test_transport_error_is_surfaced_not_swallowed() -> None:
    """断链（拔网线）⇒ 记录稳定 reason，循环停止（不假装成功）。"""
    clock = VirtualClock()
    transport = open_transport()
    transport.drop_link()
    loop = ExecLoop(
        transport,
        make_points(3),
        LIMITS,
        clock=clock,
        state_provider=lambda: fresh_frame(clock),
    )
    result = loop.run()

    assert result.final_state == REASON_TRANSPORT_ERROR
    assert result.stop_reason == REASON_LINK_DOWN
    assert result.errors
    assert result.errors[0]["reason"] == REASON_LINK_DOWN
    assert result.ok is False


def test_result_summary_is_json_serialisable_shape() -> None:
    """结果摘要结构稳定（运行日志按字段名取）。"""
    clock = VirtualClock()
    transport = open_transport()
    loop = ExecLoop(
        transport,
        make_points(2),
        LIMITS,
        clock=clock,
        state_provider=lambda: fresh_frame(clock),
    )
    summary = loop.run().to_dict()
    for key in (
        "ticks",
        "commands_sent",
        "final_state",
        "stop_reason",
        "reasons",
        "violations",
        "errors",
        "tick_intervals_s",
        "jitter",
        "command_latency",
        "rt_scheduling",
    ):
        assert key in summary
    assert summary["jitter"]["name"] == "jitter"
    assert summary["command_latency"]["name"] == "command_latency"
    assert summary["rt_scheduling"]["reason"] == "not_requested"


# ── 实时调度 ────────────────────────────────────────────────────────────
def test_realtime_scheduling_not_requested_is_explicit() -> None:
    """未请求实时调度 ⇒ 显式 ``not_requested``（不假装已生效）。"""
    report = apply_realtime_scheduling(ExecLoopConfig())
    assert report.applied is False
    assert report.reason == "not_requested"
    assert report.detail


def test_realtime_scheduling_degrades_explicitly(monkeypatch) -> None:
    """权限不足 ⇒ 显式降级并记录失败原因（**不假装已生效**）。"""

    def _boom(*args, **kwargs):
        raise PermissionError("Operation not permitted")

    monkeypatch.setattr(os, "sched_setscheduler", _boom)
    report = apply_realtime_scheduling(ExecLoopConfig(enable_rt_scheduling=True))
    assert report.applied is False
    assert report.reason == "degraded"
    assert "SCHED_FIFO" in report.detail


def test_realtime_scheduling_cpu_affinity_failure_is_recorded(monkeypatch) -> None:
    """CPU 亲和性设置失败也要显式记录。"""

    def _boom(*args, **kwargs):
        raise OSError("invalid argument")

    monkeypatch.setattr(os, "sched_setaffinity", _boom)
    report = apply_realtime_scheduling(
        ExecLoopConfig(enable_rt_scheduling=True, cpu_affinity=(0,))
    )
    assert report.applied is False
    assert report.reason == "degraded"
    assert report.detail


# ── 负例：非法输入 ──────────────────────────────────────────────────────
def test_loop_rejects_malformed_points() -> None:
    """负例：空轨迹 / 维度错 / NaN —— 一律显式报错并点名。"""
    clock = VirtualClock()
    transport = open_transport()
    with pytest.raises(ValueError, match="points 不能为空"):
        ExecLoop(transport, [], LIMITS, clock=clock)
    with pytest.raises(ValueError, match=r"points\[0\]"):
        ExecLoop(transport, [(0.0, 0.0)], LIMITS, clock=clock)
    bad = list(BASE)
    bad[2] = math.nan
    with pytest.raises(ValueError, match=r"points\[0\]\[2\]"):
        ExecLoop(transport, [tuple(bad)], LIMITS, clock=clock)
    with pytest.raises(ValueError, match="initial_command"):
        ExecLoop(
            transport,
            [BASE],
            LIMITS,
            clock=clock,
            initial_command=(0.0, 0.0),
        )


def test_config_rejects_invalid_values() -> None:
    """负例：非法周期 / 拍数 / 优先级 / 亲和性 / 事件上限。"""
    with pytest.raises(ValueError, match="period_s"):
        ExecLoopConfig(period_s=0.0)
    with pytest.raises(ValueError, match="period_s"):
        ExecLoopConfig(period_s=math.nan)
    with pytest.raises(ValueError, match="max_ticks"):
        ExecLoopConfig(max_ticks=0)
    with pytest.raises(ValueError, match="tail_hold_ticks"):
        ExecLoopConfig(tail_hold_ticks=-1)
    with pytest.raises(ValueError, match="rt_priority"):
        ExecLoopConfig(rt_priority=0)
    with pytest.raises(ValueError, match="rt_priority"):
        ExecLoopConfig(rt_priority=100)
    with pytest.raises(ValueError, match="cpu_affinity"):
        ExecLoopConfig(cpu_affinity=())
    with pytest.raises(ValueError, match="max_events"):
        ExecLoopConfig(max_events=0)


def test_virtual_clock_rejects_bad_sleep() -> None:
    """负例：虚拟时钟拒绝非法睡眠时长。"""
    clock = VirtualClock()
    with pytest.raises(ValueError, match="seconds"):
        clock.sleep(-1.0)
    with pytest.raises(ValueError, match="seconds"):
        clock.advance(math.inf)


def test_system_clock_is_monotonic_and_sleeps() -> None:
    """生产时钟可用（单调 + 睡眠不抛错）。"""
    clock = SystemClock()
    first = clock.now_ns()
    clock.sleep(0.001)
    second = clock.now_ns()
    assert second >= first


def test_event_buffer_overflow_is_counted() -> None:
    """事件缓冲溢出 ⇒ 计数（不静默丢弃）。"""
    clock = VirtualClock()
    transport = open_transport()
    jump = list(BASE)
    jump[0] = BASE[0] + 0.5
    loop = ExecLoop(
        transport,
        [BASE, tuple(jump)],
        LIMITS,
        config=ExecLoopConfig(max_events=1),
        clock=clock,
        state_provider=lambda: fresh_frame(clock),
    )
    result = loop.run()
    # 至少两个事件（violation + completed），但缓冲只留 1 个 ⇒ 溢出被计数
    assert result.events_dropped >= 1


def test_initial_command_can_be_overridden() -> None:
    """显式 ``initial_command`` ⇒ 起步拍以它为增量/加速度基准。"""
    clock = VirtualClock()
    transport = open_transport()
    loop = ExecLoop(
        transport,
        make_points(1),
        LIMITS,
        clock=clock,
        state_provider=lambda: fresh_frame(clock),
        # 起点略低于轨迹首点 ⇒ 起步拍被增量闸钳位（但仍在围栏安全区）
        initial_command=(0.45, 0.3, -0.8, 0.3, 0.0, 0.0, 0.0),
    )
    result = loop.run()
    # 起步拍从初始指令到轨迹首点的跳变会被增量闸钳位（违规留痕）
    assert REASON_DELTA_CLAMP in result.reasons
    assert result.commands_sent == 1


def test_optional_state_provider_falls_back_to_transport() -> None:
    """未注入 provider 时使用 ``transport.read_state()``。"""
    clock = VirtualClock()
    transport = open_transport()
    transport.push_frame(fresh_frame(clock))
    loop = ExecLoop(transport, make_points(2), LIMITS, clock=clock)
    result = loop.run()
    assert result.final_state == REASON_COMPLETED
    assert result.commands_sent == 2


def test_transport_error_during_safe_stop_is_recorded() -> None:
    """急停调用本身失败（断链）⇒ 记录错误，不掩盖原始原因。"""

    class _BrokenStopTransport(ScriptedTransport):
        def emergency_stop(self) -> None:
            raise TransportError("链路已断开，无法急停", reason=REASON_LINK_DOWN)

    clock = VirtualClock()
    transport = _BrokenStopTransport()
    transport.open()
    loop = ExecLoop(
        transport,
        make_points(2),
        LIMITS,
        clock=clock,
        state_provider=lambda: None,
    )
    result = loop.run()
    assert result.final_state == REASON_SAFE_STOP
    assert result.stop_reason == REASON_STATE_AGE
    assert result.errors
    assert result.errors[0]["phase"] == "safe_stop"
    assert result.errors[0]["reason"] == REASON_LINK_DOWN
