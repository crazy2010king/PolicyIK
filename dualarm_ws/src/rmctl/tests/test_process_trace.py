"""``rmctl.process.trace`` 的 trace 记录与回放测试（S06/T4 验收证据）。

覆盖（离线、零硬件、零 SDK、零网络）：

* **写入/读回**：3 条事件逐行读回且字段齐全；追加写不覆盖既有内容。
* **坏行容错**：第 2 行非 JSON → **跳过且不抛裸异常**，解析统计 ``skipped=1``。
* **空文件 / 缺文件**：→ 空列表（不抛异常）。
* **元数据补齐**：缺 ``ts`` 自动补当前时间、缺 ``severity`` 补 INFO、
  缺 ``status`` 补 ok；``reason`` / ``next_action`` / ``evidence``
  **绝不被改写**。
* **schema 冻结**：``schema_version`` 恒为 1。
* **人读格式**：``format_event`` 含「圈次 / 段名 / 下一步」三要素。
* **回放统计**：severity 计数、失败事件数、**最后一条非空 next_action**。
* **回放零副作用**：``trace`` 模块**不** import ``rm_executor`` /
  ``rm_hw_bridge`` / ``ctypes`` / ``socket``；回放期间假运动传输调用计数
  **不增**（回放绝不下发运动指令）。
* **接线**：用 T3 的 ``ProcessRunner`` 跑 1 圈 → trace 文件行数 ≥ 事件数
  且每行可解析。

只用 git 跟踪的仓库文件 + pytest 临时目录，绝不依赖 .gitignore 目录。
"""
import ast
import inspect
import math
import os
import time

import pytest

from rmctl.process import clients as clients_mod
from rmctl.process import runner as runner_mod
from rmctl.process import trace as trace_mod


TRACE_FILE = inspect.getfile(trace_mod)


# --------------------------------------------------------------------------
# 夹具 / 假对象
# --------------------------------------------------------------------------

@pytest.fixture()
def trace_path(tmp_path):
    """一个空的 trace 文件路径（文件不存在）。"""
    return str(tmp_path / "trace" / "pick_place.jsonl")


def event(iteration=1, phase="execute", segment="grasp", status="ok",
          severity="INFO", reason="段 grasp 执行成功",
          next_action="", evidence=None, ts=1710000000.5):
    """一条测试事件。"""
    return {
        "schema_version": 1,
        "ts": ts,
        "iteration": iteration,
        "phase": phase,
        "segment": segment,
        "status": status,
        "severity": severity,
        "reason": reason,
        "next_action": next_action,
        "evidence": list(evidence or []),
    }


class CountingTransport:
    """计数型假传输（任何「下发」都会留下痕迹）。"""

    def __init__(self):
        self.calls = []

    def send(self, payload):
        self.calls.append(payload)
        return {"status": "ok"}


class RecordingPlanner:
    """线性插值假 planner（轨迹符合执行层契约）。"""

    def __init__(self):
        self.calls = []

    def plan_segment(self, start_deg, goal_deg=None, goal_pose=None):
        self.calls.append(list(start_deg))
        start_rad = [math.radians(float(v)) for v in start_deg]
        goal_rad = [math.radians(float(v))
                    for v in (goal_deg if goal_deg is not None else start_deg)]
        n = 3
        times = [0.1 * i for i in range(n)]
        points = []
        for i in range(n):
            ratio = float(i) / float(n - 1)
            points.append([a + (b - a) * ratio
                           for a, b in zip(start_rad, goal_rad)])
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


class CountingMotion:
    """假运动客户端：每次执行都记一次「下发」。"""

    def __init__(self, transport):
        self._transport = transport

    def run_trajectory(self, traj_path):
        self._transport.send(traj_path)
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
    """脚本化假夹爪（判定固定给 ``grasp_ok``）。"""

    def __init__(self, result="grasp_ok"):
        self.actions = []
        self._result = result

    def act(self, action, speed=None, force=None):
        self.actions.append(action)
        return {"status": "ok", "reason": "夹爪动作已下发", "next_action": "",
                "evidence": []}

    def judge(self):
        return {
            "result": self._result,
            "severity": "OK",
            "reason": "夹持成功",
            "evidence": {},
            "next_action": "",
            "retryable": False,
        }


HOME = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def make_spec():
    """一份最小工艺规格（1 圈：运动 → 夹爪 → 回 home）。"""
    return {
        "version": 1,
        "name": "pick_place",
        "iterations": 1,
        "home_joints_deg": list(HOME),
        "segments": [
            {"name": "approach", "type": "motion",
             "goal_joints_deg": [10, 0, 0, 0, 0, 0, 0]},
            {"name": "grasp", "type": "gripper", "action": "close",
             "speed": 100, "force": 100, "expect": "grasp_ok"},
            {"name": "home", "type": "motion", "goal_joints_deg": list(HOME)},
        ],
        "source": "test",
    }


# --------------------------------------------------------------------------
# 写入 / 读回
# --------------------------------------------------------------------------

def test_write_three_events_then_read_back_all_fields(trace_path):
    """写入 3 条事件 → 逐行读回，字段齐全且顺序不乱。"""
    with trace_mod.TraceWriter(trace_path) as writer:
        writer.record(event(iteration=1, segment="approach"))
        writer.record(event(iteration=1, segment="grasp", phase="gripper"))
        writer.record(event(iteration=2, segment="home", phase="execute"))

    read = trace_mod.read_trace(trace_path)
    assert len(read) == 3
    assert read.stats["parsed"] == 3
    assert read.stats["skipped"] == 0
    for item in read:
        assert item["schema_version"] == 1
        assert isinstance(item["ts"], float)
        assert item["phase"]
        assert item["status"] == "ok"
        assert item["severity"] == "INFO"
        assert "reason" in item and "next_action" in item
        assert isinstance(item["evidence"], list)
    assert [item["segment"] for item in read] == ["approach", "grasp", "home"]
    assert [item["iteration"] for item in read] == [1, 1, 2]


def test_trace_file_is_one_event_per_line(trace_path):
    """JSONL 形态：一行一事件（物理行数 == 事件数）。"""
    with trace_mod.TraceWriter(trace_path) as writer:
        for index in range(5):
            writer.record(event(iteration=index + 1))
    with open(trace_path, "r", encoding="utf-8") as fh:
        lines = [line for line in fh.read().splitlines() if line.strip()]
    assert len(lines) == 5


def test_append_mode_keeps_previous_content(trace_path):
    """追加写：后开的 writer **不覆盖**既有内容。"""
    with trace_mod.TraceWriter(trace_path) as writer:
        writer.record(event(reason="第一条"))
    with trace_mod.TraceWriter(trace_path) as writer:
        writer.record(event(reason="第二条"))
    read = trace_mod.read_trace(trace_path)
    assert len(read) == 2
    assert [item["reason"] for item in read] == ["第一条", "第二条"]


def test_missing_metadata_is_autofilled(trace_path):
    """缺 ts 自动补当前时间、缺 severity 补 INFO、缺 status 补 ok。"""
    before = time.time()
    with trace_mod.TraceWriter(trace_path) as writer:
        writer.record({"iteration": 3, "phase": "judge", "segment": "grasp",
                       "reason": "缺元数据的事件"})
    after = time.time()
    item = list(trace_mod.read_trace(trace_path))[0]
    assert before <= item["ts"] <= after
    assert item["severity"] == "INFO"
    assert item["status"] == "ok"
    assert item["schema_version"] == 1


def test_writer_never_rewrites_reason_next_action_evidence(trace_path):
    """补齐**只针对元数据**：reason / next_action / evidence 原样保留。"""
    original = event(reason="判定依据原文", next_action="可粘贴下一步",
                     evidence=["证据 1", "证据 2"])
    with trace_mod.TraceWriter(trace_path) as writer:
        writer.record(original)
    item = list(trace_mod.read_trace(trace_path))[0]
    assert item["reason"] == "判定依据原文"
    assert item["next_action"] == "可粘贴下一步"
    assert item["evidence"] == ["证据 1", "证据 2"]


def test_schema_version_is_always_one(trace_path):
    """schema_version 恒为 1（即使入参写了别的版本）。"""
    with trace_mod.TraceWriter(trace_path) as writer:
        writer.record({"schema_version": 99, "reason": "版本越界"})
    item = list(trace_mod.read_trace(trace_path))[0]
    assert item["schema_version"] == 1
    assert trace_mod.SCHEMA_VERSION == 1


# --------------------------------------------------------------------------
# 坏行 / 空文件
# --------------------------------------------------------------------------

def test_bad_line_is_skipped_without_exception(trace_path):
    """第 2 行非 JSON → 跳过、**不抛裸异常**，统计里 ``skipped=1``。"""
    with trace_mod.TraceWriter(trace_path) as writer:
        writer.record(event(reason="第一条"))
    with open(trace_path, "a", encoding="utf-8") as fh:
        fh.write("{这不是合法 JSON\n")
    with trace_mod.TraceWriter(trace_path) as writer:
        writer.record(event(reason="第三条"))

    read = trace_mod.read_trace(trace_path)
    assert len(read) == 2, read.stats
    assert read.stats["skipped"] == 1, read.stats
    assert read.stats["parsed"] == 2, read.stats
    assert read.stats["warnings"], read.stats


def test_non_object_line_is_skipped(trace_path):
    """非 JSON 对象的行（``[1,2]``）同样被跳过。"""
    os.makedirs(os.path.dirname(trace_path), exist_ok=True)
    with open(trace_path, "w", encoding="utf-8") as fh:
        fh.write("[1, 2, 3]\n")
        fh.write("{\"reason\": \"正常事件\"}\n")
    read = trace_mod.read_trace(trace_path)
    assert len(read) == 1, read.stats
    assert read.stats["skipped"] == 1, read.stats


def test_empty_file_returns_empty_list(trace_path):
    """空文件 → 空列表（不抛异常）。"""
    os.makedirs(os.path.dirname(trace_path), exist_ok=True)
    with open(trace_path, "w", encoding="utf-8") as fh:
        fh.write("")
    read = trace_mod.read_trace(trace_path)
    assert list(read) == []
    assert read.stats["parsed"] == 0
    assert read.stats["skipped"] == 0


def test_missing_file_returns_empty_list(tmp_path):
    """文件不存在 → 空列表 + ``missing=True``（不抛异常）。"""
    read = trace_mod.read_trace(str(tmp_path / "不存在.jsonl"))
    assert list(read) == []
    assert read.stats["missing"] is True


# --------------------------------------------------------------------------
# 人读格式
# --------------------------------------------------------------------------

def test_format_event_contains_iteration_segment_and_next_action():
    """``format_event`` 含「圈次 / 段名 / 下一步」三要素。"""
    text = trace_mod.format_event(event(iteration=7, phase="judge",
                                        segment="grasp",
                                        next_action="补齐夹爪标定四项后重跑"))
    assert "圈次 7" in text, text
    assert "段 grasp" in text, text
    assert "下一步: 补齐夹爪标定四项后重跑" in text, text
    assert "judge" in text, text
    assert "INFO" in text, text


def test_format_event_handles_missing_next_action():
    """缺 next_action → 明确写出「（无下一步）」，不抛异常。"""
    item = event()
    item.pop("next_action")
    text = trace_mod.format_event(item)
    assert "下一步:" in text, text
    assert trace_mod.format_event("不是字典")


# --------------------------------------------------------------------------
# 回放
# --------------------------------------------------------------------------

def test_replay_summary_counts_and_last_next_action(trace_path):
    """``replay_trace`` 的 summary 计数正确，最后一条 next_action 非空。"""
    with trace_mod.TraceWriter(trace_path) as writer:
        writer.record(event(iteration=1, severity="INFO", status="ok"))
        writer.record(event(iteration=1, phase="judge", segment="grasp",
                            status="blocked", severity="NEEDS_PARAM",
                            reason="标定缺失",
                            next_action="补齐夹爪标定四项后重跑"))
        writer.record(event(iteration=2, phase="recover", segment="grasp",
                            status="blocked", severity="NEEDS_HUMAN",
                            reason="重试用尽",
                            next_action="人工确认夹爪与工件状态后重跑"))
        writer.record(event(iteration=2, phase="loop", segment="",
                            status="ok", severity="INFO"))

    result = trace_mod.replay_trace(trace_path)
    summary = result["summary"]
    assert result["status"] == "blocked", result
    assert summary["events"] == 4, summary
    assert summary["severity_counts"] == {"INFO": 2, "NEEDS_PARAM": 1,
                                          "NEEDS_HUMAN": 1}, summary
    assert summary["failures"] == 2, summary
    assert summary["last_next_action"] == "人工确认夹爪与工件状态后重跑", summary
    assert len(result["lines"]) == 4, result
    assert result["reason"] == "重试用尽", result


def test_replay_of_clean_trace_is_ok(trace_path):
    """无失败事件的 trace → 回放 status=ok。"""
    with trace_mod.TraceWriter(trace_path) as writer:
        writer.record(event())
        writer.record(event(iteration=2))
    result = trace_mod.replay_trace(trace_path)
    assert result["status"] == "ok", result
    assert result["summary"]["failures"] == 0
    assert result["summary"]["events"] == 2


def test_replay_of_empty_trace_is_empty(trace_path):
    """空文件回放 → 0 事件、空行、无异常。"""
    os.makedirs(os.path.dirname(trace_path), exist_ok=True)
    open(trace_path, "w", encoding="utf-8").close()
    result = trace_mod.replay_trace(trace_path)
    assert result["events"] == []
    assert result["lines"] == []
    assert result["summary"]["events"] == 0


# --------------------------------------------------------------------------
# 回放零副作用（红线）
# --------------------------------------------------------------------------

def test_trace_module_imports_no_hardware_or_sdk_modules():
    """``trace`` 模块**不** import rm_executor / rm_hw_bridge / ctypes / socket。"""
    with open(TRACE_FILE, "r", encoding="utf-8") as fh:
        tree = ast.parse(fh.read())
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                imported.add(alias.name.split(".")[0])
        elif isinstance(node, ast.ImportFrom):
            if node.module:
                imported.add(node.module.split(".")[0])
    for forbidden in ("rm_executor", "rm_hw_bridge", "ctypes", "socket",
                      "tesseract_robotics"):
        assert forbidden not in imported, imported


def test_replay_does_not_dispatch_any_motion(trace_path):
    """回放期间假运动传输调用计数**不增**（回放绝不下发运动指令）。"""
    transport = CountingTransport()
    writer = trace_mod.TraceWriter(trace_path)
    runner = runner_mod.ProcessRunner(
        make_spec(), RecordingPlanner(), CountingMotion(transport),
        ScriptedGripper(), trace_sink=writer,
        thresholds={"max_regrasp": 1, "max_stall_retry": 1,
                    "max_offline_retry": 1})
    report = runner.run()
    writer.close()
    assert report["status"] == "ok", report
    calls_after_run = len(transport.calls)
    assert calls_after_run > 0, transport.calls

    result = trace_mod.replay_trace(trace_path)
    assert result["events"], result
    assert len(transport.calls) == calls_after_run, transport.calls


# --------------------------------------------------------------------------
# 与 T3 runner 的接线
# --------------------------------------------------------------------------

def test_runner_writes_trace_file_line_by_line(trace_path):
    """接线用例：runner 跑 1 圈 → trace 行数 ≥ 事件数，且每行可解析。"""
    transport = CountingTransport()
    writer = trace_mod.TraceWriter(trace_path)
    runner = runner_mod.ProcessRunner(
        make_spec(), RecordingPlanner(), CountingMotion(transport),
        ScriptedGripper(), trace_sink=writer,
        thresholds={"max_regrasp": 1, "max_stall_retry": 1,
                    "max_offline_retry": 1})
    report = runner.run()
    writer.close()

    assert report["status"] == "ok", report
    assert report["events"], report
    with open(trace_path, "r", encoding="utf-8") as fh:
        lines = [line for line in fh.read().splitlines() if line.strip()]
    assert len(lines) >= len(report["events"]), (len(lines), len(report["events"]))

    read = trace_mod.read_trace(trace_path)
    assert read.stats["skipped"] == 0, read.stats
    assert len(read) == len(lines)
    for item in read:
        assert item["schema_version"] == 1
        assert item["phase"]
        assert item["status"] in ("ok", "blocked", "error")
    # 段名与圈次都在 trace 里（现场可读）
    segments = {item["segment"] for item in read}
    assert "grasp" in segments, segments
    assert {item["iteration"] for item in read} == {1}
    # trace 路径对 runner 报告可见
    assert report["trace_path"] == trace_path, report


def test_runner_failure_is_recorded_in_trace(trace_path):
    """失败事件也进 trace（含可粘贴下一步）→ 回放能给出下一步。"""
    transport = CountingTransport()
    writer = trace_mod.TraceWriter(trace_path)

    class FailingMotion(CountingMotion):
        def run_trajectory(self, traj_path):
            self._transport.send(traj_path)
            return {
                "status": "error",
                "reason": "离线假执行失败",
                "next_action": "检查轨迹后重跑",
                "evidence": [],
                "mode": "fake",
                "metrics": {},
                "events": [],
            }

    runner = runner_mod.ProcessRunner(
        make_spec(), RecordingPlanner(), FailingMotion(transport),
        ScriptedGripper(), trace_sink=writer,
        thresholds={"max_regrasp": 1, "max_stall_retry": 1,
                    "max_offline_retry": 1})
    report = runner.run()
    writer.close()

    assert report["status"] != "ok", report
    result = trace_mod.replay_trace(trace_path)
    assert result["summary"]["failures"] >= 1, result
    assert result["summary"]["last_next_action"], result
    assert "下一步" in result["lines"][0], result["lines"][0]
