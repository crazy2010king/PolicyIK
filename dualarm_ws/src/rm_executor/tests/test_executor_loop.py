"""rm_executor 主循环（T4）确定性测试。

全部用例跑在 **ManualClock + FakeTransport + ScriptedFeedback** 上：
逐 tick 可完全重放，断言是确定性的，不依赖真实机器负载。

覆盖：

* 正常路径：栅格 tick 数 == 栅格点数、每拍下发角与栅格一致、到位误差与保持
  时长正确、报告 schema 字段齐（15 个 metrics 键 + 顶层键）；
* 预校验失败（缺 verify / verify.ok=false）→ **0 次 transmit**；
* 时间域限值复检失败 → **0 次 transmit**；
* 预就位未到位 → blocked 且**未下发任何 tick**；
* transport 中途报错 → 调用了 ``stop`` 且 status=error；
* tick 线程**零 SDK 状态查询**（``current_trajectory_type`` 调用数 == 0）。
"""
from __future__ import annotations

import math
import os

import pytest

from rm_executor import contract, report as report_mod, resample
from rm_executor.clock import ManualClock
from rm_executor.executor import Executor, ExecutorConfig
from rm_executor.feedback import ScriptedFeedback
from rm_executor.transport import FakeTransport

RAD2DEG = 180.0 / math.pi

#: 确定性执行参数（与 workcell/rm_workcell.yaml::execution 一致）
CONFIG = ExecutorConfig(
    period_s=0.010,
    follow=False,
    trajectory_mode=1,
    radio=30,
    pre_position_speed=10,
)


# --------------------------------------------------------------------- 夹具
def _traj_payload(times, points, verify=True):
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


def _short_trajectory():
    """一条限值内的短轨迹（0 → 0.05 rad，0.05 s）。"""
    times = [0.0, 0.05]
    points = [[0.0] * 7, [0.05] * 7]
    return times, points


def _mirror_feedback(transport, error_after_transmit=0.0, fallback=None):
    """构造「镜像最近一次下发指令角」的脚本化反馈（理想跟随模型）。

    * 预就位阶段：镜像 ``preposition`` 的首点 → 预就位必然到位；
    * 流式阶段：镜像每一拍的 ``transmit`` 指令 → 跟随误差为 0；
    * ``error_after_transmit > 0`` → 首次 transmit 之后注入恒定跟随误差
      （用于负例：到位判定必然失败）。

    速度恒为 0（``speed_deg_s=0.0``）→ 到位判定的速度条件成立。
    """
    fb_fallback = list(fallback or [0.0] * 7)

    def _script(_index):
        call = transport.last_call("transmit") or transport.last_call("preposition")
        if call is None:
            return list(fb_fallback)
        deg = list(call["args"]["deg7"])
        if error_after_transmit and transport.count("transmit") > 0:
            deg = [v + error_after_transmit for v in deg]
        return deg

    return ScriptedFeedback(script=_script, speed_deg_s=0.0, period_s=0.010)


def _make_executor(transport, feedback, **kwargs):
    params = dict(config=CONFIG, clock=ManualClock(period_s=0.010), mode="fake")
    params.update(kwargs)
    return Executor(transport, feedback, **params)


# --------------------------------------------------------------------- 用例
def test_normal_path_ticks_match_grid_and_arrival_ok(write_traj):
    """正常路径：tick 数 == 栅格点数、每拍角与栅格一致、到位判定正确。"""
    times, points = _short_trajectory()
    path = write_traj(_traj_payload(times, points))
    grid_t, grid_p = resample.resample_to_grid(times, points, CONFIG.period_s)

    transport = FakeTransport()
    feedback = _mirror_feedback(transport)
    executor = _make_executor(transport, feedback)
    rep = executor.run(path)

    assert rep["status"] == "ok", rep
    assert rep["reason"]

    # 栅格 tick 数
    assert rep["metrics"]["ticks"] == len(grid_p)
    assert rep["metrics"]["planned_duration_s"] == pytest.approx(grid_t[-1] - grid_t[0])

    # 每一拍下发的角与栅格一致（前 N 拍 = 栅格点）
    transmits = transport.calls_for("transmit")
    assert len(transmits) == len(grid_p) + rep["metrics"]["hold_ticks"]
    for idx, expected in enumerate(grid_p):
        got = transmits[idx]["args"]["deg7"]
        assert got == pytest.approx([v * RAD2DEG for v in expected], abs=1e-9)
        assert transmits[idx]["args"]["follow"] is CONFIG.follow
        assert transmits[idx]["args"]["trajectory_mode"] == CONFIG.trajectory_mode
        assert transmits[idx]["args"]["radio"] == CONFIG.radio

    # 预就位下发的是首点
    pre = transport.last_call("preposition")
    assert pre["args"]["deg7"] == pytest.approx([v * RAD2DEG for v in grid_p[0]], abs=1e-9)
    assert pre["args"]["speed"] == CONFIG.pre_position_speed

    # 到位判定：误差 < 0.1°，保持 ≥ 300 ms
    assert rep["metrics"]["arrival_err_deg"] is not None
    assert rep["metrics"]["arrival_err_deg"] < contract.ARRIVAL_TOL_DEG
    assert rep["metrics"]["arrival_hold_ms"] >= contract.ARRIVAL_HOLD_MS
    assert rep["metrics"]["hold_ticks"] >= contract.ARRIVAL_HOLD_MS / 1000.0 / CONFIG.period_s

    # 理想跟随 → 跟随误差为 0；虚拟时钟 → 抖动为 0，实时因子为 1
    assert rep["metrics"]["follow_err_max_deg"] == pytest.approx(0.0, abs=1e-9)
    assert rep["metrics"]["follow_err_rms_deg"] == pytest.approx(0.0, abs=1e-9)
    assert rep["metrics"]["tick_jitter_p99_ms"] == pytest.approx(0.0, abs=1e-9)
    assert rep["metrics"]["tick_jitter_max_ms"] == pytest.approx(0.0, abs=1e-9)
    assert rep["metrics"]["rt_factor"] == pytest.approx(1.0, abs=1e-9)
    assert rep["metrics"]["duration_s"] > 0.0

    # tick 线程零 SDK 状态查询
    assert transport.count("current_trajectory_type") == 0


def test_report_schema_has_exact_top_level_and_metric_keys(write_traj):
    """报告 schema：顶层键固定、metrics 恰好 15 个键、事件字段齐。"""
    times, points = _short_trajectory()
    path = write_traj(_traj_payload(times, points))
    transport = FakeTransport()
    executor = _make_executor(transport, _mirror_feedback(transport))
    rep = executor.run(path)

    assert set(rep.keys()) == set(report_mod.TOP_LEVEL_KEYS)
    assert rep["mode"] == "fake"
    assert set(rep["metrics"].keys()) == set(report_mod.METRIC_KEYS)
    assert len(report_mod.METRIC_KEYS) == 15
    assert report_mod.validate_report(rep) == []
    for event in rep["events"]:
        assert set(event.keys()) == set(report_mod.EVENT_KEYS)
        assert event["severity"] in ("INFO", "RECOVERABLE", "NEEDS_PARAM",
                                     "NEEDS_HUMAN", "FATAL")
    assert rep["events"], "正常路径至少应有一条事件"
    # 事件按严重度排序
    rank = {"INFO": 0, "RECOVERABLE": 1, "NEEDS_PARAM": 2, "NEEDS_HUMAN": 3,
            "FATAL": 4}
    ranks = [rank[e["severity"]] for e in rep["events"]]
    assert ranks == sorted(ranks)


def test_missing_verify_is_blocked_and_sends_nothing(write_traj):
    """预校验失败（缺 verify）→ blocked 且 0 次 transmit。"""
    times, points = _short_trajectory()
    path = write_traj(_traj_payload(times, points, verify=None))
    transport = FakeTransport()
    executor = _make_executor(transport, _mirror_feedback(transport))
    rep = executor.run(path)

    assert rep["status"] == "blocked"
    assert rep["reason"]
    assert rep["next_action"]
    assert transport.count("transmit") == 0
    assert transport.count("preposition") == 0
    assert any(e["type"] == "trajectory_contract_rejected" for e in rep["events"])


def test_verify_not_ok_is_blocked_and_sends_nothing(write_traj):
    """预校验失败（verify.ok=false）→ blocked 且 0 次 transmit。"""
    times, points = _short_trajectory()
    path = write_traj(_traj_payload(times, points, verify=False))
    transport = FakeTransport()
    executor = _make_executor(transport, _mirror_feedback(transport))
    rep = executor.run(path)

    assert rep["status"] == "blocked"
    assert transport.count("transmit") == 0
    assert transport.count("preposition") == 0
    assert any(e["type"] == "trajectory_contract_rejected" for e in rep["events"])


def test_allow_unverified_flag_is_honoured(write_traj):
    """``allow_unverified=True`` → 放行（仅离线假执行），并留下证据。"""
    times, points = _short_trajectory()
    path = write_traj(_traj_payload(times, points, verify=None))
    transport = FakeTransport()
    executor = _make_executor(transport, _mirror_feedback(transport))
    rep = executor.run(path, allow_unverified=True)

    assert rep["status"] == "ok", rep
    assert transport.count("transmit") > 0
    assert any("allow_unverified" in line for line in rep["evidence"])


def test_time_domain_limit_violation_blocks_before_any_tick(write_traj):
    """时间域限值复检失败 → blocked 且 0 次 transmit。"""
    times = [0.0, 0.05]
    points = [[0.0] * 7, [3.0] * 7]  # 速度 60 rad/s，远超 3.14 rad/s
    path = write_traj(_traj_payload(times, points))
    transport = FakeTransport()
    executor = _make_executor(transport, _mirror_feedback(transport))
    rep = executor.run(path)

    assert rep["status"] == "blocked"
    assert transport.count("transmit") == 0
    assert transport.count("preposition") == 0
    assert any(e["type"] == "time_domain_limits_rejected" for e in rep["events"])


def test_pre_position_not_reached_blocks_before_any_tick(write_traj):
    """预就位未到位 → blocked 且未下发任何 tick。"""
    times, points = _short_trajectory()
    path = write_traj(_traj_payload(times, points))
    transport = FakeTransport()
    # 反馈恒定在远离目标的位置 → 预就位永远不满足容差
    feedback = ScriptedFeedback(static_deg=[10.0] * 7, speed_deg_s=0.0)
    executor = _make_executor(transport, feedback,
                               pre_position_timeout_s=0.5)
    rep = executor.run(path)

    assert rep["status"] == "blocked"
    assert transport.count("transmit") == 0
    assert transport.count("preposition") == 1
    assert any(e["type"] == "pre_position_not_reached" for e in rep["events"])
    assert rep["metrics"]["arrival_err_deg"] is None


def test_transport_failure_midway_stops_and_errors(write_traj):
    """transport 中途报错 → 调用了 stop 且 status=error。"""
    times, points = _short_trajectory()
    path = write_traj(_traj_payload(times, points))
    transport = FakeTransport(script={"transmit": 7})
    executor = _make_executor(transport, _mirror_feedback(transport))
    rep = executor.run(path)

    assert rep["status"] == "error"
    assert transport.count("stop") == 1
    assert transport.count("transmit") == 1  # 失败的那一拍仍被记录（已尝试下发）
    fatal = [e for e in rep["events"] if e["severity"] == "FATAL"]
    assert fatal and fatal[0]["type"] == "transport_failed"
    assert rep["next_action"]


def test_follow_error_injection_is_measured_and_blocks_arrival(write_traj):
    """注入跟随误差 → 指标里看得到，且到位判定不通过（blocked）。"""
    times, points = _short_trajectory()
    path = write_traj(_traj_payload(times, points))
    transport = FakeTransport()
    feedback = _mirror_feedback(transport, error_after_transmit=1.5)
    executor = _make_executor(transport, feedback, arrival_timeout_s=0.5)
    rep = executor.run(path)

    assert rep["status"] == "blocked"
    assert rep["metrics"]["follow_err_max_deg"] == pytest.approx(1.5, abs=1e-6)
    assert rep["metrics"]["follow_err_rms_deg"] > 0.0
    assert rep["metrics"]["arrival_err_deg"] is None
    assert any(e["type"] == "arrival_not_reached" for e in rep["events"])


def test_execution_config_comes_from_workcell_file_only(write_traj):
    """执行参数唯一来源 = workcell/rm_workcell.yaml::execution（缺则 blocked）。"""
    times, points = _short_trajectory()
    path = write_traj(_traj_payload(times, points))
    transport = FakeTransport()
    executor = Executor(transport, _mirror_feedback(transport),
                        clock=ManualClock(period_s=0.010), mode="fake")
    # 配置来自工位文件（仓库内 git 跟踪文件）
    assert executor._config is not None
    assert executor._config.period_s == 0.010
    assert executor._config.trajectory_mode == 1
    rep = executor.run(path)
    assert rep["status"] == "ok", rep


def test_missing_workcell_config_is_blocked(write_traj, fake_ws):
    """工位文件缺 execution 段 → blocked（执行层不补默认值）。"""
    times, points = _short_trajectory()
    path = write_traj(_traj_payload(times, points))
    transport = FakeTransport()
    executor = Executor(transport, _mirror_feedback(transport),
                        clock=ManualClock(period_s=0.010), mode="fake", ws=fake_ws)
    assert executor._config is None
    rep = executor.run(path)
    assert rep["status"] == "blocked"
    assert transport.count("transmit") == 0
    assert any(e["type"] == "execution_config_invalid" for e in rep["events"])


def test_no_trajectory_file_is_blocked_without_any_call():
    """轨迹文件不存在 → blocked，0 次调用，且不伪造任何数值。"""
    transport = FakeTransport()
    executor = _make_executor(transport, _mirror_feedback(transport))
    rep = executor.run(os.path.join("/nonexistent", "traj.json"))

    assert rep["status"] == "blocked"
    assert transport.call_count == 0
    assert rep["metrics"]["ticks"] == 0
    assert rep["metrics"]["arrival_err_deg"] is None
    assert rep["metrics"]["follow_err_rms_deg"] is None


def test_report_round_trips_to_json(write_traj, tmp_path):
    """报告可落盘为 JSON 且回读后结构不变（T6 落盘路径的前置保证）。"""
    import json

    times, points = _short_trajectory()
    path = write_traj(_traj_payload(times, points))
    transport = FakeTransport()
    executor = _make_executor(transport, _mirror_feedback(transport))
    rep = executor.run(path)

    out = report_mod.dump_report(rep, str(tmp_path / "reports" / "s03.json"))
    assert os.path.isfile(out)
    with open(out, "r", encoding="utf-8") as fh:
        loaded = json.load(fh)
    assert loaded == rep
    assert report_mod.validate_report(loaded) == []
