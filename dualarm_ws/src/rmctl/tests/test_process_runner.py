"""``rmctl.process.runner`` 的工艺状态机测试（S06/T3 验收证据）。

覆盖（离线、零硬件、零厂商内核、零 SDK 调用）：

* **正常路径**：3 圈全绿（圈数 / 段数 / 判定结果 / status）。
* **当前关节跟踪**：第二段规划起点 == 第一段轨迹终点（度）；笛卡尔目标
  ``goal_pose`` **原样透传**给 planner。
* **闸门红线**：判定未复位 → 运动段被 ``gripper_interlock_blocked`` 拒绝且
  **运动客户端零调用**；判定完成 → ``finish_action`` → 运动放行。
* **needs_param**：停循环 / severity=NEEDS_PARAM / next_action 指向补配置，
  且**绝不**被当成 ``grasp_empty`` 去重抓（调用计数不增）。
* **恢复阶梯**：``grasp_empty`` / ``stall`` / ``overtemp`` / ``overcurrent`` /
  ``offline`` 逐条落到预期严重度与重试次数（阶梯表来自上游唯一真值）。
* **失败分级**：运动段 FATAL → 立即停循环；循环级 next_action 取**最严重者**。
* **循环控制**：未回到 home → 本圈失败并停循环；iterations 上限 50。
* **trace 出口**：duck-type sink（只实现 ``record``）收到的事件齐全。
* **上游唯一真值**：runner 源码里**不出现**本地严重度字典；阶梯与严重度
  排序均取自上游。
* **零 SDK**：走真实 ``rm_executor`` 离线假执行时仍零 SDK / 零厂商内核。

只用 git 跟踪的仓库文件 + pytest 临时目录，绝不依赖 .gitignore 目录。
"""
import ast
import inspect
import math
import os
import sys

import pytest

from rmctl.process import clients as clients_mod
from rmctl.process import runner as runner_mod

from rm_hw_bridge import gripper_state


# --------------------------------------------------------------------------
# 假客户端 / 假闸门
# --------------------------------------------------------------------------

def _judgment(result):
    """一份 ``Judgment.to_dict()`` 形态的判定结果。"""
    table = {
        "grasp_ok": ("OK", "", "夹持成功"),
        "grasp_empty": ("RECOVERABLE", "重抓一次", "夹空（力度不足）"),
        "stall": ("RECOVERABLE", "松手重试", "堵转"),
        "overtemp": ("NEEDS_HUMAN", "停机冷却", "过温"),
        "overcurrent": ("NEEDS_PARAM", "检查 24 V 供电并共地", "过流"),
        "offline": ("RECOVERABLE", "重连后重试", "夹爪离线"),
        "needs_param": ("NEEDS_PARAM", "补齐夹爪标定四项后重跑", "标定缺失"),
        "unknown": ("NEEDS_HUMAN", "人工确认夹爪状态", "判定未知"),
    }
    severity, next_action, reason = table[result]
    return {
        "result": result,
        "severity": severity,
        "reason": reason,
        "evidence": {"result": result},
        "next_action": next_action,
        "retryable": severity == "RECOVERABLE",
    }


class RecordingPlanner:
    """记录调用的假 planner（线性插值，轨迹符合执行层契约）。"""

    def __init__(self):
        self.calls = []

    def plan_segment(self, start_deg, goal_deg=None, goal_pose=None):
        self.calls.append({
            "start_deg": list(start_deg),
            "goal_deg": list(goal_deg) if goal_deg is not None else None,
            "goal_pose": list(goal_pose) if goal_pose is not None else None,
        })
        if goal_pose is not None:
            goal_rad = [math.radians(float(v)) for v in start_deg]
        else:
            goal_rad = [math.radians(float(v)) for v in (goal_deg or [])]
        start_rad = [math.radians(float(v)) for v in start_deg]
        n = 3
        times = [0.1 * i for i in range(n)]
        points = []
        for i in range(n):
            ratio = float(i) / float(n - 1)
            points.append([a + (b - a) * ratio for a, b in zip(start_rad, goal_rad)])
        payload = clients_mod.trajectory_payload(times, points, {"ok": True})
        return {
            "status": "ok",
            "reason": "离线线性插值轨迹已生成",
            "next_action": "",
            "evidence": [],
            "times": times,
            "points": points,
            "verify": {"ok": True},
            "trajectory": payload,
        }


class RecordingMotion:
    """记录调用的假运动客户端（可脚本化报告）。"""

    def __init__(self, reports=None):
        self.calls = []
        self._reports = list(reports or [])

    def run_trajectory(self, traj_path):
        self.calls.append(traj_path)
        if self._reports:
            return self._reports.pop(0)
        return {
            "status": "ok",
            "reason": "离线假执行完成",
            "next_action": "",
            "evidence": [],
            "mode": "fake",
            "metrics": {},
            "events": [],
        }


class ScriptedGripper:
    """脚本化假夹爪（``act`` 记调用，``judge`` 按脚本给结果）。"""

    def __init__(self, results=None, gate=None, by_action=None):
        self.actions = []
        self.judgments = []
        self._results = list(results or [])
        self._by_action = dict(by_action or {})
        self._gate = gate

    def act(self, action, speed=None, force=None):
        self.actions.append(action)
        if self._gate is not None:
            self._gate.mark_unjudged("夹爪动作 %s" % action)
        return {"status": "ok", "reason": "夹爪动作已下发", "next_action": "",
                "evidence": []}

    def judge(self):
        if self._results:
            result = self._results.pop(0)
        else:
            result = self._by_action.get(self.actions[-1] if self.actions else "",
                                          "grasp_ok")
        self.judgments.append(result)
        return _judgment(result)


class StuckGate:
    """一个**复位无效**的闸门（模拟判定后忘记 finish_action 的最坏情况）。"""

    def __init__(self):
        self.state = "IDLE"
        self.finish_calls = 0

    def check(self):
        if self.state == "IDLE":
            return None
        return {"state": self.state, "reason": "夹爪状态未判定",
                "next_action": "先完成夹爪状态判定后再下发运动"}

    def finish_action(self):
        self.finish_calls += 1
        return self.state  # **不复位**（故意的）

    def mark_unjudged(self, reason=""):
        self.state = "GRIPPER_UNJUDGED"
        return self.state


class RecordingSink:
    """只实现 ``record(event)`` 的 trace sink（duck-type）。"""

    def __init__(self):
        self.events = []
        self.path = ""

    def record(self, event):
        self.events.append(event)


# --------------------------------------------------------------------------
# 规格与阈值
# --------------------------------------------------------------------------

HOME = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def motion(name, goal=None, pose=None):
    seg = {"name": name, "type": "motion"}
    if pose is not None:
        seg["goal_pose"] = list(pose)
    else:
        seg["goal_joints_deg"] = list(goal if goal is not None else HOME)
    return seg


def gripper(name, action, expect):
    return {"name": name, "type": "gripper", "action": action,
            "speed": 100, "force": 100, "expect": expect}


def make_spec(segments, iterations=1, home=None):
    return {
        "version": 1,
        "name": "pick_place",
        "iterations": iterations,
        "home_joints_deg": list(home if home is not None else HOME),
        "segments": list(segments),
        "source": "test",
    }


def pick_place_segments():
    """与 ``workcell/tasks/pick_place.yaml`` 同形的段序列。"""
    return [
        motion("approach_pick", [0, 20, 30, 0, 0, 0, 0]),
        gripper("grasp", "close", "grasp_ok"),
        motion("lift", [0, 25, 35, 0, 0, 0, 0]),
        motion("place_approach", [10, 20, 30, 0, 0, 0, 0]),
        gripper("release", "open", "grasp_empty"),
        motion("retreat", [0, 20, 30, 0, 0, 0, 0]),
        motion("home", HOME),
    ]


def thresholds(max_regrasp=2, max_stall_retry=1, max_offline_retry=1):
    """注入的阈值（恢复上限取自工位配置口径）。"""
    return {
        "max_regrasp": max_regrasp,
        "max_stall_retry": max_stall_retry,
        "max_offline_retry": max_offline_retry,
    }


def make_runner(spec, gripper_results=None, motion_reports=None, gate=None,
                by_action=None, sink=None, th=None, max_iterations=None):
    planner = RecordingPlanner()
    motion_client = RecordingMotion(motion_reports)
    gripper_client = ScriptedGripper(gripper_results, gate=gate,
                                     by_action=by_action)
    runner = runner_mod.ProcessRunner(
        spec, planner, motion_client, gripper_client,
        trace_sink=sink, gate=gate, thresholds=th or thresholds(),
        max_iterations=max_iterations)
    return runner, planner, motion_client, gripper_client


# --------------------------------------------------------------------------
# 正常路径
# --------------------------------------------------------------------------

def test_three_iterations_all_green():
    """3 圈全绿：圈数 / 段数 / 判定结果 / status 全部正确。"""
    gate = gripper_state.MotionGate()
    runner, _planner, motion_client, gripper_client = make_runner(
        make_spec(pick_place_segments(), iterations=3), gate=gate,
        by_action={"close": "grasp_ok", "open": "grasp_empty"})
    report = runner.run()

    assert report["status"] == "ok", report
    assert report["severity"] == "INFO", report
    assert report["iterations_planned"] == 3, report
    assert report["iterations_completed"] == 3, report
    assert report["segments_executed"] == 3 * 7, report
    assert report["failures"] == [], report
    assert len(report["judgments"]) == 3 * 2, report
    assert [j["result"] for j in report["judgments"]] == (
        ["grasp_ok", "grasp_empty"] * 3), report
    assert report["exit_code"] == 0, report
    # 每圈 5 个运动段 → 15 次运动调用
    assert len(motion_client.calls) == 3 * 5, motion_client.calls
    assert len(gripper_client.actions) == 3 * 2, gripper_client.actions


def test_current_joint_tracking_uses_trajectory_end_point():
    """第二段规划起点 == 第一段轨迹最后一点（度）。"""
    spec = make_spec([
        motion("a", [10, 0, 0, 0, 0, 0, 0]),
        motion("b", [20, 0, 0, 0, 0, 0, 0]),
        motion("home", HOME),
    ])
    runner, planner, _motion, _gripper = make_runner(spec)
    report = runner.run()

    assert report["status"] == "ok", report
    assert len(planner.calls) == 3, planner.calls
    assert planner.calls[0]["start_deg"] == HOME, planner.calls
    assert planner.calls[1]["start_deg"] == pytest.approx(
        [10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]), planner.calls
    assert planner.calls[2]["start_deg"] == pytest.approx(
        [20.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]), planner.calls


def test_cartesian_goal_pose_is_passed_through():
    """笛卡尔目标 ``goal_pose`` **原样透传**给 planner（``goal_deg`` 为 None）。"""
    pose = [0.3, -0.1, 0.25, 0.0, 1.57, 0.0]
    spec = make_spec([motion("cart", pose=pose), motion("home", HOME)])
    runner, planner, _motion, _gripper = make_runner(spec)
    report = runner.run()

    assert report["status"] == "ok", report
    assert planner.calls[0]["goal_pose"] == pose, planner.calls
    assert planner.calls[0]["goal_deg"] is None, planner.calls


# --------------------------------------------------------------------------
# 闸门红线
# --------------------------------------------------------------------------

def test_motion_blocked_when_gate_not_reset_after_judgment():
    """判定后未复位 → 运动段被互锁拒绝，且**运动客户端零调用**。"""
    gate = StuckGate()
    spec = make_spec([
        gripper("grasp", "close", "grasp_ok"),
        motion("home", HOME),
    ])
    runner, _planner, motion_client, gripper_client = make_runner(
        spec, gate=gate, by_action={"close": "grasp_ok"})
    report = runner.run()

    assert report["status"] == "blocked", report
    assert report["severity"] == "NEEDS_HUMAN", report
    assert motion_client.calls == [], motion_client.calls
    kinds = [f["type"] for f in report["failures"]]
    assert "gripper_interlock_blocked" in kinds, report["failures"]
    assert gate.finish_calls == 1, gate.finish_calls
    assert report["next_action"], report


def test_motion_allowed_after_judgment_resets_gate():
    """判定完成 → ``finish_action`` → 闸门回 IDLE → 运动放行。"""
    gate = gripper_state.MotionGate()
    spec = make_spec([
        gripper("grasp", "close", "grasp_ok"),
        motion("home", HOME),
    ])
    runner, _planner, motion_client, _gripper = make_runner(
        spec, gate=gate, by_action={"close": "grasp_ok"})
    report = runner.run()

    assert report["status"] == "ok", report
    assert gate.state == gripper_state.MotionGate.STATE_IDLE, gate.state
    assert len(motion_client.calls) == 1, motion_client.calls
    kinds = [f["type"] for f in report["failures"]]
    assert "gripper_interlock_blocked" not in kinds, report["failures"]


# --------------------------------------------------------------------------
# needs_param：绝不重抓、绝不复位闸门
# --------------------------------------------------------------------------

def test_needs_param_stops_loop_with_param_action():
    """``needs_param`` → 停循环 / severity=NEEDS_PARAM / next_action 指向补配置。"""
    gate = gripper_state.MotionGate()
    spec = make_spec([
        gripper("grasp", "close", "grasp_ok"),
        motion("home", HOME),
    ])
    runner, _planner, motion_client, gripper_client = make_runner(
        spec, gate=gate, gripper_results=["needs_param"])
    report = runner.run()

    assert report["status"] == "blocked", report
    assert report["severity"] == "NEEDS_PARAM", report
    assert "标定" in report["next_action"], report
    assert report["iterations_completed"] == 0, report
    # 闸门保持 UNJUDGED（不复位）
    assert gate.state == gripper_state.MotionGate.STATE_GRIPPER_UNJUDGED, gate.state
    assert motion_client.calls == [], motion_client.calls


def test_needs_param_never_triggers_regrasp():
    """``needs_param`` **绝不**被当成 ``grasp_empty`` 重抓（调用计数不增）。"""
    spec = make_spec([
        gripper("grasp", "close", "grasp_ok"),
        motion("home", HOME),
    ])
    runner, _planner, _motion, gripper_client = make_runner(
        spec, gripper_results=["needs_param"])
    report = runner.run()

    assert gripper_client.actions == ["close"], gripper_client.actions
    assert gripper_client.judgments == ["needs_param"], gripper_client.judgments
    assert report["severity"] == "NEEDS_PARAM", report


# --------------------------------------------------------------------------
# 恢复阶梯（逐条）
# --------------------------------------------------------------------------

def test_grasp_empty_regrasp_until_limit_then_needs_human():
    """``grasp_empty`` → 按阶梯重抓 ≤ max_regrasp，用尽后 NEEDS_HUMAN 停循环。"""
    spec = make_spec([
        gripper("grasp", "close", "grasp_ok"),
        motion("home", HOME),
    ])
    runner, _planner, _motion, gripper_client = make_runner(
        spec, gripper_results=["grasp_empty"] * 8, th=thresholds(max_regrasp=2))
    report = runner.run()

    closes = gripper_client.actions.count("close")
    opens = gripper_client.actions.count("open")
    assert closes == 1 + 2, gripper_client.actions
    assert opens == 2, gripper_client.actions
    assert report["severity"] == "NEEDS_HUMAN", report
    assert report["status"] == "blocked", report
    assert report["next_action"], report


def test_stall_retry_once_then_needs_human():
    """``stall`` → 松手重试 1 次后仍堵转 → NEEDS_HUMAN 停循环。"""
    spec = make_spec([
        gripper("grasp", "close", "grasp_ok"),
        motion("home", HOME),
    ])
    runner, _planner, _motion, gripper_client = make_runner(
        spec, gripper_results=["stall"] * 8, th=thresholds(max_stall_retry=1))
    report = runner.run()

    assert gripper_client.actions.count("close") == 2, gripper_client.actions
    assert gripper_client.actions.count("open") == 1, gripper_client.actions
    assert report["severity"] == "NEEDS_HUMAN", report
    assert report["status"] == "blocked", report


def test_overtemp_is_not_retried():
    """``overtemp`` → **不重试**、直接 NEEDS_HUMAN。"""
    spec = make_spec([
        gripper("grasp", "close", "grasp_ok"),
        motion("home", HOME),
    ])
    runner, _planner, _motion, gripper_client = make_runner(
        spec, gripper_results=["overtemp"] * 4)
    report = runner.run()

    assert gripper_client.actions == ["close"], gripper_client.actions
    assert report["severity"] == "NEEDS_HUMAN", report


def test_overcurrent_is_needs_param():
    """``overcurrent`` → NEEDS_PARAM。"""
    spec = make_spec([
        gripper("grasp", "close", "grasp_ok"),
        motion("home", HOME),
    ])
    runner, _planner, _motion, gripper_client = make_runner(
        spec, gripper_results=["overcurrent"] * 4)
    report = runner.run()

    assert gripper_client.actions == ["close"], gripper_client.actions
    assert report["severity"] == "NEEDS_PARAM", report
    assert report["status"] == "blocked", report


def test_offline_retries_then_escalates_to_needs_human():
    """``offline`` → RECOVERABLE 重试后用尽 → NEEDS_HUMAN。"""
    spec = make_spec([
        gripper("grasp", "close", "grasp_ok"),
        motion("home", HOME),
    ])
    runner, _planner, _motion, gripper_client = make_runner(
        spec, gripper_results=["offline"] * 8, th=thresholds(max_offline_retry=1))
    report = runner.run()

    assert gripper_client.actions.count("close") == 2, gripper_client.actions
    assert report["severity"] == "NEEDS_HUMAN", report
    kinds = [f["type"] for f in report["failures"]]
    assert "gripper_mismatch" in kinds, report["failures"]


def test_recovery_ladder_comes_from_upstream_table():
    """阶梯的严重度 / next_action **逐字**来自上游 ``RECOVERY_LADDER``。"""
    ladder = gripper_state.RECOVERY_LADDER
    step = ladder["grasp_empty"][-1]
    spec = make_spec([
        gripper("grasp", "close", "grasp_ok"),
        motion("home", HOME),
    ])
    runner, _planner, _motion, _gripper = make_runner(
        spec, gripper_results=["grasp_empty"] * 8, th=thresholds(max_regrasp=1))
    report = runner.run()

    assert report["severity"] == step["severity"], report
    assert report["next_action"] == step["next_action"].format(
        attempt=1, max_regrasp=1), report["next_action"]


# --------------------------------------------------------------------------
# 失败分级
# --------------------------------------------------------------------------

def test_motion_fatal_event_stops_loop_immediately():
    """运动段 FATAL 事件 → 立即停循环、severity=FATAL、next_action 非空。"""
    fatal_report = {
        "status": "error",
        "reason": "跟随误差超限",
        "next_action": "立即停机并检查工件",
        "evidence": ["follow_err_max_deg=12.0"],
        "mode": "fake",
        "metrics": {},
        "events": [{"type": "follow_error_exceeded", "severity": "FATAL",
                    "t": 0.1, "reason": "跟随误差超限", "evidence": [],
                    "next_action": "立即停机并检查工件"}],
    }
    spec = make_spec([
        motion("approach_pick", [0, 20, 30, 0, 0, 0, 0]),
        motion("home", HOME),
    ])
    runner, _planner, motion_client, _gripper = make_runner(
        spec, motion_reports=[fatal_report])
    report = runner.run()

    assert report["status"] == "error", report
    assert report["severity"] == "FATAL", report
    assert report["next_action"], report
    assert report["iterations_completed"] == 0, report
    assert len(motion_client.calls) == 1, motion_client.calls


def test_loop_next_action_takes_the_worst_failure():
    """循环级 next_action 取**最严重者**（夹爪 NEEDS_HUMAN > 运动 RECOVERABLE）。"""
    recoverable = {
        "status": "blocked",
        "reason": "保持时间不足",
        "next_action": "延长保持时间后重跑",
        "evidence": [],
        "mode": "fake",
        "metrics": {},
        "events": [{"type": "arrival_hold_short", "severity": "RECOVERABLE",
                    "t": 0.1, "reason": "保持时间不足", "evidence": [],
                    "next_action": "延长保持时间后重跑"}],
    }
    spec = make_spec([
        motion("approach_pick", [0, 20, 30, 0, 0, 0, 0]),
        gripper("grasp", "close", "grasp_ok"),
        motion("home", HOME),
    ])
    runner, _planner, _motion, _gripper = make_runner(
        spec, motion_reports=[recoverable], gripper_results=["overtemp"])
    report = runner.run()

    severities = [f["severity"] for f in report["failures"]]
    assert "RECOVERABLE" in severities, report["failures"]
    assert "NEEDS_HUMAN" in severities, report["failures"]
    assert report["severity"] == "NEEDS_HUMAN", report
    assert report["next_action"] != "延长保持时间后重跑", report


# --------------------------------------------------------------------------
# 循环控制
# --------------------------------------------------------------------------

def test_not_returning_home_fails_the_iteration():
    """未回到 home → 本圈判失败并停循环。"""
    blocked_report = {
        "status": "blocked",
        "reason": "保持时间不足",
        "next_action": "延长保持时间后重跑",
        "evidence": [],
        "mode": "fake",
        "metrics": {},
        "events": [{"type": "arrival_hold_short", "severity": "RECOVERABLE",
                    "t": 0.1, "reason": "保持时间不足", "evidence": [],
                    "next_action": "延长保持时间后重跑"}],
    }
    spec = make_spec([motion("home", HOME)])
    runner, _planner, _motion, _gripper = make_runner(
        spec, motion_reports=[blocked_report])
    report = runner.run()

    kinds = [f["type"] for f in report["failures"]]
    assert "home_not_reached" in kinds, report["failures"]
    assert report["iterations_completed"] == 0, report
    assert report["status"] == "blocked", report


def test_iterations_upper_bound_is_50():
    """iterations 上限 50：传 51（或规格 51）→ blocked / 参数非法。"""
    spec = make_spec([motion("home", HOME)], iterations=1)
    runner, _planner, _motion, _gripper = make_runner(spec, max_iterations=51)
    report = runner.run()
    assert report["status"] == "blocked", report
    assert report["iterations_planned"] == 0, report
    assert report["next_action"], report

    spec2 = make_spec([motion("home", HOME)], iterations=51)
    runner2, _p, _m, _g = make_runner(spec2)
    report2 = runner2.run()
    assert report2["status"] == "blocked", report2


def test_iterations_zero_is_blocked():
    """iterations=0 → 参数非法 blocked（不静默跑 0 圈）。"""
    spec = make_spec([motion("home", HOME)], iterations=0)
    runner, _planner, _motion, _gripper = make_runner(spec)
    report = runner.run()
    assert report["status"] == "blocked", report
    assert report["iterations_completed"] == 0, report


# --------------------------------------------------------------------------
# trace 出口
# --------------------------------------------------------------------------

def test_trace_sink_receives_events():
    """duck-type sink（只实现 ``record``）收到齐全事件。"""
    sink = RecordingSink()
    spec = make_spec(pick_place_segments(), iterations=1)
    runner, _planner, _motion, _gripper = make_runner(
        spec, by_action={"close": "grasp_ok", "open": "grasp_empty"}, sink=sink)
    report = runner.run()

    assert report["status"] == "ok", report
    assert len(sink.events) >= 7, sink.events
    for event in sink.events:
        assert event["phase"], event
        assert "segment" in event, event
        assert event["status"], event
        assert event["severity"], event
    phases = set(e["phase"] for e in sink.events)
    assert {"loop", "execute", "gripper", "judge"} <= phases, phases


def test_trace_sink_failure_does_not_break_the_run():
    """sink 抛异常 → 工艺照常跑完（证据里留痕）。"""
    class BrokenSink:
        path = ""

        def record(self, event):
            raise RuntimeError("磁盘满")

    spec = make_spec([motion("home", HOME)], iterations=1)
    runner, _planner, _motion, _gripper = make_runner(spec, sink=BrokenSink())
    report = runner.run()
    assert report["status"] == "ok", report
    assert any("trace sink" in item for item in report["evidence"]), report


# --------------------------------------------------------------------------
# 上游唯一真值 / 零 SDK
# --------------------------------------------------------------------------

def test_runner_source_has_no_local_severity_dictionary():
    """runner 源码里**不出现**本地严重度字典（严重度排序唯一在上游）。

    用 **AST** 判定（不用正则：docstring 里提到该名字不算「自带字典」）。
    """
    tree = ast.parse(inspect.getsource(runner_mod))
    assigned = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name):
                    assigned.append(target.id)
                elif isinstance(target, ast.Attribute):
                    assigned.append(target.attr)
    for banned in ("SEVERITY_RANK", "SEVERITIES", "SEVERITY_STATUS",
                   "SEVERITY_EXIT_CODES"):
        assert banned not in assigned, assigned
    source = inspect.getsource(runner_mod)
    assert "severity_rank" in source, "严重度排序必须用 rm_executor.events"
    assert "worst_severity" in source, "最严重者必须用 rm_executor.events"


def test_runner_uses_upstream_severity_ranking():
    """runner 的严重度排序与上游 ``rm_executor.events`` 完全一致。"""
    from rm_executor import events as events_mod

    source = inspect.getsource(runner_mod)
    assert "events_mod.severity_rank" in source
    assert events_mod.severity_rank("FATAL") > events_mod.severity_rank(
        "NEEDS_HUMAN")
    assert events_mod.status_for_severity("FATAL") == "error"
    assert events_mod.status_for_severity("NEEDS_HUMAN") == "blocked"


def test_runner_module_does_not_import_sdk():
    """runner 模块零 SDK 直调（不 import ctypes / 厂商内核 / 硬件绑定）。"""
    tree = ast.parse(inspect.getsource(runner_mod))
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                imported.add(alias.name.split(".")[0])
        elif isinstance(node, ast.ImportFrom):
            if node.module:
                imported.add(node.module.split(".")[0])
    assert "ctypes" not in imported, imported
    assert "tesseract_robotics" not in imported, imported
    assert "rm_hw_bridge" not in imported, imported
    assert "rm_executor" not in imported, imported


def test_offline_run_with_real_executor_is_ok_and_sdk_free(repo_root):
    """走真实 ``rm_executor`` 离线假执行：status=ok，且零 SDK / 零厂商内核。"""
    from rmctl import gripper_proxy

    thresholds, _used, _source = gripper_proxy.effective_thresholds({})
    gate = gripper_state.MotionGate()
    planner = clients_mod.FakePlanner(
        verify_fn=lambda times, points: {"ok": True, "checks": {}})
    motion_client = clients_mod.FakeMotionClient(motion_gate=gate,
                                                  ws=repo_root)
    gripper_client = clients_mod.FakeGripperClient(thresholds=thresholds,
                                                    gate=gate)
    runner = runner_mod.ProcessRunner(
        make_spec(pick_place_segments(), iterations=1),
        planner, motion_client, gripper_client,
        gate=gate, thresholds=thresholds)
    before = set(sys.modules)
    report = runner.run()

    assert report["status"] == "ok", report
    assert report["iterations_completed"] == 1, report
    assert report["segments_executed"] == 7, report
    # 本次运行**不得**新拉起厂商规划内核（其它用例可能已导入过）
    assert "tesseract_robotics" not in (set(sys.modules) - before)
