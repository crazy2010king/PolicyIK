#!/usr/bin/env python3
"""S06 取放工艺编排层的**离线证据链**（50 圈全流程 + 六类故障分支 + trace 回放对照）。

用法
----
    python3 scripts/pick_place_offline_evidence.py --out build/reports/s06_pick_place_offline.json

退出码
------
* ``0`` —— 四段证据全部通过（断言全绿）；
* ``4`` —— 存在断言失败（报告会写明**哪一段、哪一条**断言失败）。

**诚实分层（重要）**
--------------------
本脚本跑在 **真实规划**（厂商 tesseract 内核）+ **真实四道校验**
（``rmctl.verify``）之上，但**运动执行是假的**（``rm_executor`` 的 fake 传输）、
**夹爪是假的**（``rm_hw_bridge.gripper.FakeGripper``）。因此报告里所有数字
一律带 ``mode=fake`` 标注——它们**证明工艺编排代码路径正确**，
但**不是真机结论**。

真机四项验收（真机 50 圈连续成功 / 真机 trace 来自真机执行 / 真机失败下一步 /
真机单连接多段建连）本轮**一律未实证**：真机窗口 blocked
（``workcell/baselines/network_plan.yaml`` status=blocked），回填入口见报告
``live_window`` 段。

四段证据
--------
1. **success_run**：用 ``workcell/tasks/pick_place.yaml``（``iterations=50``）
   跑 ``rmctl.process.runner.ProcessRunner``：``LivePlanner``（真实规划 +
   真实四道校验 + retime 自愈阶梯 + 缓存）、``FakeMotionClient``（假运动）、
   ``FakeGripperClient``（假夹爪）、``TraceWriter``（JSONL trace）。
   断言：50 圈全完成、每圈段数 == 规格段数、每圈判定与段声明 ``expect`` 一致、
   每圈末尾回到 home、全程零 ``gripper_interlock_blocked``、循环级 status=ok。
2. **fault_cases**：逐类注入六种故障（``grasp_empty`` / ``stall`` / ``overtemp`` /
   ``overcurrent`` / ``offline`` / ``needs_param``），断言「判定结果 / severity /
   是否自动重试 / 停循环后的 next_action」。
3. **replay**：``rmctl.process.trace.replay_trace`` 只读回放（零运动 / 零 SDK /
   零网络），断言回放行数 == trace 事件数、severity 计数一致、**最后一条非空
   next_action** 可直接粘贴。
4. **live_window**：显式 ``status="blocked"``，写明真机四项本轮未实证 + 回填命令。

零新依赖：只用标准库 + 仓库内既有包（``rmctl`` / ``rm_hw_bridge`` /
``rm_executor`` / ``rm_description``）。
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from datetime import datetime
from typing import Any, Dict, List, Optional, Tuple

# --------------------------------------------------------------------------
# 0. 自插 sys.path（脚本可从任意目录调用）
# --------------------------------------------------------------------------

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(HERE)

for _rel in (
    "dualarm_ws/src/rmctl",
    "dualarm_ws/src/rm_hw_bridge",
    "dualarm_ws/src/rm_executor",
    "dualarm_ws/src/rm_safety",
    "dualarm_ws/src/rm_description/scripts",
):
    _p = os.path.join(REPO_ROOT, _rel)
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

from rm_hw_bridge import gripper as G  # noqa: E402
from rm_hw_bridge import gripper_state as S  # noqa: E402
from rmctl.process import clients as clients_mod  # noqa: E402
from rmctl.process.runner import ProcessRunner  # noqa: E402
from rmctl.process.spec import load_task_spec  # noqa: E402
from rmctl.process.trace import TraceWriter, read_trace, replay_trace  # noqa: E402

#: 离线代理的**固定声明**（必须出现在每一段 evidence 里）
FAKE_NOTE = "mode=fake（离线代理：真规划 + 真校验 + 假运动 + 假夹爪，代理模型，不代表真机结论）"

#: 真机四项验收的**未实证声明**
LIVE_NOTE = (
    "真机四项验收（真机 50 圈连续成功 / 真机 trace 来自真机执行 / 真机失败给出下一步 / "
    "真机单连接多段建连）本轮未实证：真机窗口 blocked"
    "（workcell/baselines/network_plan.yaml status=blocked）"
)

#: 工艺任务规格（唯一来源 = workcell/tasks/*.yaml）
DEFAULT_TASK_FILE = os.path.join("workcell", "tasks", "pick_place.yaml")

#: 离线 trace 落盘路径（build/ 下，gitignored）
TRACE_MAIN = os.path.join("build", "run", "pick_place_trace_offline.jsonl")
TRACE_FAULT_DIR = os.path.join("build", "run")

#: 六类故障分支的期望（severity 取自判定层 / 执行层的同一口径）
FAULT_EXPECT: Dict[str, Dict[str, Any]] = {
    "grasp_empty": {
        "label": "夹空",
        "judgment_result": S.RESULT_GRASP_EMPTY,
        "judgment_severity": S.SEV_RECOVERABLE,
        "auto_retry": True,
        "final_severity": "INFO",
        "final_status": "ok",
        "close_calls": 2,
        "next_action_required": False,
        "note": "按恢复阶梯重抓（第 2 次夹取成功）",
    },
    "stall": {
        "label": "堵转",
        "judgment_result": S.RESULT_STALL,
        "judgment_severity": S.SEV_RECOVERABLE,
        "auto_retry": True,
        "final_severity": "NEEDS_HUMAN",
        "final_status": "blocked",
        "close_calls": 2,
        "next_action_required": True,
        "note": "松手重试 1 次后仍堵转 → 升级人工",
    },
    "overtemp": {
        "label": "过温",
        "judgment_result": S.RESULT_OVERTEMP,
        "judgment_severity": S.SEV_NEEDS_HUMAN,
        "auto_retry": False,
        "final_severity": "NEEDS_HUMAN",
        "final_status": "blocked",
        "close_calls": 1,
        "next_action_required": True,
        "note": "禁止自动重试 → 立即升级人工",
    },
    "overcurrent": {
        "label": "过流",
        "judgment_result": S.RESULT_OVERCURRENT,
        "judgment_severity": S.SEV_NEEDS_PARAM,
        "auto_retry": False,
        "final_severity": "NEEDS_PARAM",
        "final_status": "blocked",
        "close_calls": 1,
        "next_action_required": True,
        "note": "不重试 → 需要参数（外部 24 V 供电）",
    },
    "offline": {
        "label": "离线",
        "judgment_result": S.RESULT_OFFLINE,
        "judgment_severity": S.SEV_RECOVERABLE,
        "auto_retry": True,
        "final_severity": "NEEDS_HUMAN",
        "final_status": "blocked",
        "close_calls": 2,
        "next_action_required": True,
        "note": "重连重试 1 次后仍离线 → 升级人工",
    },
    "needs_param": {
        "label": "标定缺失",
        "judgment_result": S.RESULT_NEEDS_PARAM,
        "judgment_severity": S.SEV_NEEDS_PARAM,
        "auto_retry": False,
        "final_severity": "NEEDS_PARAM",
        "final_status": "blocked",
        "close_calls": 1,
        "next_action_required": True,
        "gate_unjudged": True,
        "note": "结论未定 → 绝不重抓、闸门保持 UNJUDGED",
    },
}


# --------------------------------------------------------------------------
# 1. 工具
# --------------------------------------------------------------------------

def _now() -> str:
    return datetime.now().isoformat(timespec="seconds")


def peak_rss_kb() -> Optional[int]:
    """本进程峰值常驻内存（KiB）；平台不支持时返回 ``None``。"""
    try:
        import resource

        return int(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)
    except Exception:  # noqa: BLE001 - 非 POSIX 平台 → 不阻断
        return None


def git_status() -> List[str]:
    """``git status --porcelain``（不可用时返回一行说明，不抛异常）。"""
    try:
        proc = subprocess.run(
            ["git", "status", "--porcelain"],
            cwd=REPO_ROOT, capture_output=True, text=True, timeout=120,
        )
    except Exception as exc:  # noqa: BLE001 - git 不可用 → 记录而不是崩
        return ["<git status 异常: %r>" % (exc,)]
    if proc.returncode != 0:
        return ["<git status 失败: %s>" % proc.stderr.strip()]
    return [line for line in proc.stdout.splitlines() if line.strip()]


def ensure_model(ws: str) -> Tuple[Any, str, List[str]]:
    """确保模型可用；必要时**重新物化** ``build/model``。

    :return: ``(model, model_source, evidence)``；``model_source`` ∈
        ``{"prebuilt", "rebuilt"}``。
    """
    from rmctl.model import Model, VendorUnavailable

    evidence: List[str] = []
    try:
        model = Model(ws=ws)
        evidence.append("模型已就绪（build/model 已物化，未重建）")
        return model, "prebuilt", evidence
    except VendorUnavailable as exc:
        evidence.append("模型加载失败（VendorUnavailable）: %s" % exc)

    import build_model  # dualarm_ws/src/rm_description/scripts/build_model.py

    build_model.build_model(ws=ws)
    evidence.append("已调用 build_model.build_model(ws=%s) 重新物化 build/model"
                    % ws)
    model = Model(ws=ws)
    evidence.append("重新物化后模型加载成功")
    return model, "rebuilt", evidence


def load_thresholds(ws: str) -> Tuple[Optional[Dict[str, Any]], Dict[str, Any]]:
    """判定阈值 + 恢复阶梯上限（**唯一来源链**，脚本不写死任何数值）。

    * 判定四键 → ``rmctl.gripper_proxy.load_effective_thresholds``
      （工位配置优先，缺失项用代理值补齐）；
    * 恢复上限 → ``workcell/rm_workcell.yaml::gripper.max_regrasp`` +
      ``rm_hw_bridge.gripper_state`` 的 stall / offline 默认上限。

    :return: ``(thresholds, info)``；``thresholds is None`` → 工位配置不可用。
    """
    from rmctl.gripper_proxy import load_effective_thresholds

    thresholds, proxy_used, source, cfg = load_effective_thresholds(ws)
    info: Dict[str, Any] = {
        "thresholds_source": source,
        "proxy_used_keys": list(proxy_used),
        "config_status": cfg.get("status"),
        "config_reason": cfg.get("reason", ""),
        "config_source": (cfg.get("config") or {}).get("source", ""),
        "calibration_pending": list((cfg.get("config") or {}).get(
            "calibration_pending") or []),
    }
    if thresholds is None:
        return None, info
    config = cfg.get("config") or {}
    thresholds["max_regrasp"] = int(config.get("max_regrasp") or 0)
    thresholds["max_stall_retry"] = int(S.DEFAULT_MAX_STALL_RETRY)
    thresholds["max_offline_retry"] = int(S.DEFAULT_MAX_OFFLINE_RETRY)
    info["recovery_limits"] = {
        "max_regrasp": thresholds["max_regrasp"],
        "max_stall_retry": thresholds["max_stall_retry"],
        "max_offline_retry": thresholds["max_offline_retry"],
        "source": ("workcell/rm_workcell.yaml::gripper.max_regrasp + "
                   "rm_hw_bridge.gripper_state 默认上限"),
    }
    return thresholds, info


class CountingMotionClient:
    """给运动客户端套一层**只计数不改行为**的壳。

    用途：证明「trace 回放期间零运动下发」（回放前后计数不变）。
    """

    def __init__(self, inner: Any) -> None:
        self._inner = inner
        self.calls = 0

    def run_trajectory(self, traj_path: str) -> Dict[str, Any]:
        self.calls += 1
        return self._inner.run_trajectory(traj_path)


class FaultGripperClient(clients_mod.FakeGripperClient):
    """假夹爪 + **物理故障注入**（只注入寄存器 / 在线位，**不复制判定逻辑**）。

    注入口径：

    * ``grasp_empty`` —— 第 1 次夹取后力度寄存器置 0（力度不足 → 夹空），
      之后恢复正常（模拟「按阶梯重抓后成功」）；
    * ``stall`` / ``overtemp`` / ``overcurrent`` —— 置对应**错误位**
      （``rm_hw_bridge.gripper_state.ERROR_BIT_*``，判定真值唯一来源）；
    * ``offline`` —— 置离线（Modbus 路径读不到「离线」，故走官方夹爪快路径
      ``gripper_api``，与 S05 证据脚本同一口径）；
    * ``needs_param`` —— **不注入**，改用**未补代理**的原始阈值（四键为
      ``None``）→ 判定自然落到 ``needs_param``。

    判定与严重度**始终**由 ``rm_hw_bridge.gripper_state.judge`` 给出。
    """

    def __init__(self, thresholds: Optional[Dict[str, Any]], gate: Any,
                 fault: str, path: str = "modbus", port: Any = None) -> None:
        super().__init__(thresholds=thresholds, gate=gate, path=path, port=port)
        self.fault = fault
        self.close_calls = 0
        self.open_calls = 0

    def act(self, action: str, speed: Optional[int] = None,
            force: Optional[int] = None) -> Dict[str, Any]:
        res = super().act(action, speed=speed, force=force)
        if res.get("status") != "ok":
            return res
        if action == "close":
            self.close_calls += 1
        else:
            self.open_calls += 1
        self._inject(action)
        return res

    def _inject(self, action: str) -> None:
        """把故障写进**假夹爪的物理状态**（判定真值仍在硬件层）。"""
        if self.fault in ("needs_param", "", None) or action != "close":
            return
        port = getattr(self._adapter, "port", None)
        if port is None:
            return
        if self.fault == "grasp_empty":
            if self.close_calls != 1:
                return  # 只第一次夹空 → 之后重抓成功
            port.set_register(G.REG_ACTUAL_FORCE, 0)
        elif self.fault == "stall":
            # Modbus 路径读的是**寄存器** 0x0013（错误字），不是 SDK 结构体
            port.set_register(G.REG_ERROR, S.ERROR_BIT_STALL)
        elif self.fault == "overtemp":
            port.set_register(G.REG_ERROR, S.ERROR_BIT_OVERTEMP)
            port.set_register(G.REG_ACTUAL_TEMP, 85)
        elif self.fault == "overcurrent":
            port.set_register(G.REG_ERROR, S.ERROR_BIT_OVERCURRENT)
        elif self.fault == "offline":
            port.set_online(False)


def _fresh_gate() -> Any:
    """取共享运动闸门并**复位到 IDLE**（用例隔离，避免上一个用例残留状态）。"""
    gate = S.shared_motion_gate()
    gate.finish_action()
    return gate


def _write_json(path: str, payload: Dict[str, Any]) -> str:
    parent = os.path.dirname(os.path.abspath(path))
    if parent:
        os.makedirs(parent, exist_ok=True)
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(payload, fh, ensure_ascii=False, indent=2, default=str)
    return path


def _remove(path: str) -> None:
    """删除旧 trace（避免追加写导致计数失真；只在 build/ 下操作）。"""
    try:
        if os.path.isfile(path):
            os.remove(path)
    except OSError:
        pass


def _file_size(path: str) -> Optional[int]:
    """文件大小（字节）；不存在时返回 ``None``。"""
    try:
        return int(os.path.getsize(path))
    except OSError:
        return None


# --------------------------------------------------------------------------
# 2. 第一段：50 圈全流程
# --------------------------------------------------------------------------

def run_success(ws: str, spec: Dict[str, Any], thresholds: Dict[str, Any],
                planner: Any, trace_path: str) -> Tuple[Dict[str, Any], Any]:
    """第一段证据：50 圈取放工艺全流程（真实规划 + 真实校验 + 假运动 + 假夹爪）。

    :return: ``(本段证据, 运动客户端计数器)``（计数器用于证明回放期间零运动）。
    """
    _remove(trace_path)
    gate = _fresh_gate()
    motion = CountingMotionClient(
        clients_mod.FakeMotionClient(motion_gate=gate, ws=ws, arm="left"))
    gripper = clients_mod.FakeGripperClient(thresholds=thresholds, gate=gate)
    sink = TraceWriter(trace_path)
    started = time.time()
    try:
        runner = ProcessRunner(
            spec, planner, motion, gripper,
            trace_sink=sink, gate=gate, thresholds=thresholds,
        )
        report = runner.run()
    finally:
        sink.close()
    trace_errors = list(getattr(sink, "errors", []) or [])
    elapsed = time.time() - started

    iterations = int(spec.get("iterations") or 0)
    segments = list(spec.get("segments") or [])
    judged = [seg for seg in segments if seg.get("expect")]

    judgments = list(report.get("judgments") or [])
    failures = list(report.get("failures") or [])
    events = list(report.get("events") or [])

    # -- 判定与 expect 的一致性（逐圈逐段）----------------------------
    per_iteration: Dict[int, List[Dict[str, Any]]] = {}
    for item in judgments:
        per_iteration.setdefault(int(item.get("iteration") or 0), []).append(item)
    mismatches: List[Dict[str, Any]] = []
    for iteration, items in sorted(per_iteration.items()):
        for item in items:
            if item.get("result") != item.get("expect"):
                mismatches.append({
                    "iteration": iteration,
                    "segment": item.get("segment"),
                    "expect": item.get("expect"),
                    "result": item.get("result"),
                })
    per_iteration_counts = {it: len(items) for it, items in per_iteration.items()}
    count_mismatch = [it for it, n in per_iteration_counts.items()
                      if n != len(judged)]

    # -- 每圈末尾回到 home ------------------------------------------------
    home_events = [e for e in events
                   if e.get("phase") == "execute" and e.get("segment") == "home"
                   and e.get("status") == "ok"]
    interlock = [f for f in failures if f.get("type") == "gripper_interlock_blocked"]

    # -- 每圈耗时（来自 trace 的「圈开始 / 圈完成」事件）-----------------
    loop_events: Dict[int, Dict[str, float]] = {}
    for event in events:
        if event.get("phase") != "loop":
            continue
        iteration = int(event.get("iteration") or 0)
        slot = loop_events.setdefault(iteration, {})
        text = str(event.get("reason") or "")
        if "圈开始" in text:
            slot["start"] = float(event.get("ts") or 0.0)
        elif "圈完成" in text:
            slot["end"] = float(event.get("ts") or 0.0)
    durations = []
    for iteration in sorted(loop_events):
        slot = loop_events[iteration]
        if "start" in slot and "end" in slot:
            durations.append(round(slot["end"] - slot["start"], 4))

    passed = (
        report.get("status") == "ok"
        and int(report.get("iterations_completed") or 0) == iterations
        and int(report.get("segments_executed") or 0) == iterations * len(segments)
        and not mismatches
        and not count_mismatch
        and len(home_events) == iterations
        and not interlock
        and not failures
        and not trace_errors
    )

    section = {
        "status": "ok" if passed else "error",
        "passed": passed,
        "reason": ("%d 圈取放工艺全流程通过（%d/%d 圈，%d 段，%d 条判定，"
                   "零互锁阻塞）" % (iterations, report.get("iterations_completed"), iterations,
                                    report.get("segments_executed"), len(judgments))
                   if passed else
                   "50 圈全流程未通过（%s）" % report.get("reason")),
        "next_action": ("" if passed else
                        "对照本段 assertions 修正后重跑本脚本"),
        "mode": "fake",
        "evidence": [
            FAKE_NOTE,
            "规划: rmctl.process.clients.LivePlanner（真实厂商规划 + 真实四道校验 + "
            "保路径降速重定时；带缓存）",
            "运动: rmctl.process.clients.FakeMotionClient（rm_executor 同一 Executor / "
            "同一报告 schema，假传输）",
            "夹爪: rmctl.process.clients.FakeGripperClient（假夹爪：close → 力=2×阈值、"
            "位置=窗口中心 → grasp_ok；open → 力=0 → grasp_empty）",
            "trace: %s" % trace_path,
        ],
        "iterations_planned": iterations,
        "iterations_completed": int(report.get("iterations_completed") or 0),
        "segments_per_iteration": len(segments),
        "segments_executed": int(report.get("segments_executed") or 0),
        "judgments": len(judgments),
        "judgments_per_iteration": len(judged),
        "judgment_mismatches": mismatches,
        "iteration_judgment_count_mismatch": count_mismatch,
        "home_returns": len(home_events),
        "interlock_blocked_events": len(interlock),
        "failures": failures,
        "loop_status": report.get("status"),
        "loop_severity": report.get("severity"),
        "loop_reason": report.get("reason"),
        "motion_calls": motion.calls,
        "trace_path": trace_path,
        "trace_events": len(events),
        "trace_write_errors": trace_errors,
        "wall_time_s": round(elapsed, 3),
        "iteration_durations_s": durations,
        "iteration_duration_stats": {
            "min": min(durations) if durations else None,
            "max": max(durations) if durations else None,
            "mean": (round(sum(durations) / len(durations), 4) if durations else None),
        },
        "assertions": [
            {"name": "iterations_completed == %d" % iterations,
             "passed": int(report.get("iterations_completed") or 0) == iterations,
             "observed": report.get("iterations_completed")},
            {"name": "segments_executed == %d（每圈段数 == 规格段数）"
             % (iterations * len(segments)),
             "passed": int(report.get("segments_executed") or 0)
             == iterations * len(segments),
             "observed": report.get("segments_executed")},
            {"name": "每圈判定结果与段声明 expect 一致",
             "passed": not mismatches,
             "observed": len(mismatches)},
            {"name": "每圈判定条数 == 规格声明 expect 的段数（%d）" % len(judged),
             "passed": not count_mismatch,
             "observed": count_mismatch[:5]},
            {"name": "每圈末尾回到 home（%d 圈）" % iterations,
             "passed": len(home_events) == iterations,
             "observed": len(home_events)},
            {"name": "全程零 gripper_interlock_blocked 事件",
             "passed": not interlock,
             "observed": len(interlock)},
            {"name": "循环级报告 status == ok",
             "passed": report.get("status") == "ok",
             "observed": report.get("status")},
            {"name": "零失败记录",
             "passed": not failures,
             "observed": len(failures)},
            {"name": "零 trace 写入错误（磁盘满 / 只读）",
             "passed": not trace_errors,
             "observed": trace_errors[:3]},
        ],
    }
    return section, motion


# --------------------------------------------------------------------------
# 3. 第二段：六类故障分支
# --------------------------------------------------------------------------

def run_fault_case(ws: str, spec: Dict[str, Any], thresholds: Dict[str, Any],
                   planner: Any, fault: str, trace_path: str) -> Dict[str, Any]:
    """跑一类故障分支（单圈），返回该分支的证据行。"""
    expect = FAULT_EXPECT[fault]
    _remove(trace_path)
    gate = _fresh_gate()

    # needs_param：判定阈值用**未补代理**的原始阈值（四键为 None）
    if fault == "needs_param":
        client_thresholds: Dict[str, Any] = {
            key: None for key in S.REQUIRED_GRASP_THRESHOLDS
        }
        for key in ("max_regrasp", "max_stall_retry", "max_offline_retry"):
            client_thresholds[key] = thresholds.get(key)
    else:
        client_thresholds = dict(thresholds)

    # 离线判定：Modbus 路径读不到「离线」→ 走官方夹爪快路径（与 S05 同口径）
    path = "gripper_api" if fault == "offline" else "modbus"
    port = None
    if path == "gripper_api":
        port = G.FakeGripper(base=1, mode=2, online=True)

    motion = CountingMotionClient(
        clients_mod.FakeMotionClient(motion_gate=gate, ws=ws, arm="left"))
    gripper = FaultGripperClient(client_thresholds, gate, fault,
                                  path=path, port=port)
    sink = TraceWriter(trace_path)
    try:
        runner = ProcessRunner(
            spec, planner, motion, gripper,
            trace_sink=sink, gate=gate, thresholds=thresholds,
            max_iterations=1,
        )
        report = runner.run()
    finally:
        sink.close()
    trace_errors = list(getattr(sink, "errors", []) or [])

    gate_state = str(getattr(gate, "state", "") or "")
    judgments = list(report.get("judgments") or [])
    failures = list(report.get("failures") or [])
    first_judgment = judgments[0] if judgments else {}
    regrasp_calls = max(0, gripper.close_calls - 1)  # 首次夹取之外的重抓次数

    result_ok = first_judgment.get("result") == expect["judgment_result"]
    severity_ok = first_judgment.get("severity") == expect["judgment_severity"]
    retried = regrasp_calls > 0
    retry_ok = retried == expect["auto_retry"]
    final_severity_ok = report.get("severity") == expect["final_severity"]
    final_status_ok = report.get("status") == expect["final_status"]
    next_action = str(report.get("next_action") or "")
    recover_events = [e for e in (report.get("events") or [])
                      if e.get("phase") == "recover"]
    recover_action = next((str(e.get("next_action") or "")
                            for e in reversed(recover_events)
                            if str(e.get("next_action") or "").strip()), "")
    if expect.get("next_action_required"):
        # 停循环的分支：循环级 next_action 必须非空且可粘贴
        next_action_ok = bool(next_action.strip())
    else:
        # 自动恢复成功的分支：循环级无失败（next_action 可为空），
        # 但**恢复阶梯那一步**必须留下可粘贴的下一步
        next_action_ok = bool(next_action.strip()) or bool(recover_action.strip())
    close_calls_ok = gripper.close_calls == expect["close_calls"]

    gate_unjudged_ok = True
    if expect.get("gate_unjudged"):
        gate_unjudged_ok = gate_state == S.MotionGate.STATE_GRIPPER_UNJUDGED

    passed = all([result_ok, severity_ok, retry_ok, final_severity_ok,
                  final_status_ok, next_action_ok, close_calls_ok,
                  gate_unjudged_ok])

    return {
        "fault": fault,
        "label": expect["label"],
        "status": "ok" if passed else "error",
        "passed": passed,
        "mode": "fake",
        "note": expect["note"],
        "expected": {
            "judgment_result": expect["judgment_result"],
            "judgment_severity": expect["judgment_severity"],
            "auto_retry": expect["auto_retry"],
            "final_severity": expect["final_severity"],
            "final_status": expect["final_status"],
            "close_calls": expect["close_calls"],
        },
        "observed": {
            "judgment_result": first_judgment.get("result"),
            "judgment_severity": first_judgment.get("severity"),
            "auto_retry": retried,
            "regrasp_calls": regrasp_calls,
            "close_calls": gripper.close_calls,
            "open_calls": gripper.open_calls,
            "final_severity": report.get("severity"),
            "final_status": report.get("status"),
            "gate_state": gate_state,
            "iterations_completed": report.get("iterations_completed"),
            "segments_executed": report.get("segments_executed"),
        },
        "next_action": next_action,
        "recover_next_action": recover_action,
        "judgment_reason": first_judgment.get("reason"),
        "trace_write_errors": trace_errors,
        "failure": failures[0] if failures else None,
        "trace_path": trace_path,
        "assertions": [
            {"name": "判定结果 == %s" % expect["judgment_result"],
             "passed": result_ok, "observed": first_judgment.get("result")},
            {"name": "判定 severity == %s" % expect["judgment_severity"],
             "passed": severity_ok, "observed": first_judgment.get("severity")},
            {"name": "自动重试 %s" % ("发生" if expect["auto_retry"] else "不发生"),
             "passed": retry_ok, "observed": "regrasp=%d" % regrasp_calls},
            {"name": "夹取次数 == %d" % expect["close_calls"],
             "passed": close_calls_ok, "observed": gripper.close_calls},
            {"name": "停循环 severity == %s" % expect["final_severity"],
             "passed": final_severity_ok, "observed": report.get("severity")},
            {"name": "停循环 status == %s" % expect["final_status"],
             "passed": final_status_ok, "observed": report.get("status")},
            {"name": ("停循环给出非空 next_action（可粘贴）"
                      if expect.get("next_action_required")
                      else "恢复阶梯那一步给出非空 next_action（可粘贴）"),
             "passed": next_action_ok,
             "observed": next_action or recover_action},
            {"name": "needs_param 时闸门保持 UNJUDGED",
             "passed": gate_unjudged_ok, "observed": gate_state},
        ],
    }


def run_fault_cases(ws: str, spec: Dict[str, Any], thresholds: Dict[str, Any],
                    planner: Any) -> Dict[str, Any]:
    """第二段证据：六类故障分支逐条命中预期 severity 与 next_action。"""
    cases: List[Dict[str, Any]] = []
    for fault in FAULT_EXPECT:
        trace_path = os.path.join(REPO_ROOT, TRACE_FAULT_DIR,
                                  "pick_place_trace_fault_%s.jsonl" % fault)
        cases.append(run_fault_case(ws, spec, thresholds, planner, fault,
                                    trace_path))
    failed = [c["fault"] for c in cases if not c.get("passed")]
    return {
        "status": "ok" if not failed else "error",
        "passed": not failed,
        "reason": ("六类故障分支逐条命中预期 severity 与 next_action（%d/%d 通过）"
                   % (len(cases) - len(failed), len(cases))
                   if not failed else
                   "未命中的故障分支: %s" % ", ".join(failed)),
        "next_action": ("" if not failed else
                        "对照失败分支的 assertions 字段修正注入夹具后重跑本脚本"),
        "mode": "fake",
        "evidence": [
            FAKE_NOTE,
            "判定真值: rm_hw_bridge.gripper_state.judge（脚本只注入物理状态，"
            "不复制判定/严重度逻辑）",
            "恢复阶梯: rm_hw_bridge.gripper_state.RECOVERY_LADDER（表驱动）",
            "恢复上限: %s" % json.dumps(
                {"max_regrasp": thresholds.get("max_regrasp"),
                 "max_stall_retry": thresholds.get("max_stall_retry"),
                 "max_offline_retry": thresholds.get("max_offline_retry")},
                ensure_ascii=False),
        ],
        "cases": cases,
        "failed_cases": failed,
        "assertions": [
            {"name": "六类故障分支全部命中预期",
             "passed": not failed, "observed": failed},
        ],
    }


# --------------------------------------------------------------------------
# 4. 第三段：trace 回放对照
# --------------------------------------------------------------------------

def _replay_one(path: str) -> Dict[str, Any]:
    """回放一份 trace（只读）+ 与原始事件的一致性对照。"""
    replay = replay_trace(path)
    read = read_trace(path)
    events = list(read)
    summary = dict(replay.get("summary") or {})

    severity_counts: Dict[str, int] = {}
    for event in events:
        key = str(event.get("severity") or "INFO")
        severity_counts[key] = severity_counts.get(key, 0) + 1

    lines = list(replay.get("lines") or [])
    last_action = str(summary.get("last_next_action") or "")
    return {
        "trace_path": path,
        "file_lines": int(read.stats.get("lines", 0)),
        "parsed": int(read.stats.get("parsed", 0)),
        "skipped": int(read.stats.get("skipped", 0)),
        "events": len(events),
        "replay_lines": len(lines),
        "replay_status": replay.get("status"),
        "replay_reason": replay.get("reason"),
        "severity_counts_trace": severity_counts,
        "severity_counts_replay": summary.get("severity_counts"),
        "last_next_action": last_action,
        "sample_lines": lines[:5],
    }


def run_replay(trace_paths: List[str], motion_counter: Any) -> Dict[str, Any]:
    """第三段证据：trace 只读回放对照（零运动 / 零 SDK / 零网络）。"""
    before = int(getattr(motion_counter, "calls", 0))
    rows = [_replay_one(path) for path in trace_paths]
    after = int(getattr(motion_counter, "calls", 0))

    counts_ok = all(row["replay_lines"] == row["events"] == row["file_lines"]
                    for row in rows)
    severity_ok = all(row["severity_counts_trace"] == row["severity_counts_replay"]
                      for row in rows)
    skipped_ok = all(row["skipped"] == 0 for row in rows)
    action_ok = all(bool(row["last_next_action"].strip()) for row in rows)
    zero_motion = before == after
    replay_passed = bool(counts_ok and severity_ok and skipped_ok and action_ok
                         and zero_motion)

    return {
        "status": "ok" if replay_passed else "error",
        "passed": replay_passed,
        "reason": ("trace 回放对照一致（%d 份 trace：行数 == 事件数、severity 计数一致、"
                   "最后一条非空 next_action 可粘贴、回放期间零运动下发）"
                   % len(rows)
                   if replay_passed
                   else "trace 回放对照不一致"),
        "next_action": ("" if zero_motion else
                        "检查是否有模块在回放期间下发运动（回放应纯读）"),
        "mode": "fake",
        "evidence": [
            FAKE_NOTE,
            "回放实现: rmctl.process.trace.replay_trace（只读 JSONL；"
            "零 import rm_executor / rm_hw_bridge / ctypes / socket）",
            "回放期间运动下发次数: before=%d after=%d" % (before, after),
        ],
        "traces": rows,
        "motion_calls_before_replay": before,
        "motion_calls_after_replay": after,
        "assertions": [
            {"name": "回放行数 == trace 事件数 == 文件行数",
             "passed": counts_ok,
             "observed": {os.path.basename(r["trace_path"]):
                          "%d/%d/%d" % (r["replay_lines"], r["events"],
                                        r["file_lines"]) for r in rows}},
            {"name": "回放 summary 的 severity 计数与 trace 一致",
             "passed": severity_ok, "observed": len(rows)},
            {"name": "trace 无坏行（skipped == 0）",
             "passed": skipped_ok,
             "observed": {os.path.basename(r["trace_path"]): r["skipped"]
                          for r in rows}},
            {"name": "最后一条非空 next_action 可直接粘贴",
             "passed": action_ok,
             "observed": {os.path.basename(r["trace_path"]):
                          (r["last_next_action"] or "")[:60] for r in rows}},
            {"name": "回放期间零运动下发",
             "passed": zero_motion, "observed": "%d → %d" % (before, after)},
        ],
    }


# --------------------------------------------------------------------------
# 5. 第四段：live_window（真机窗口声明）
# --------------------------------------------------------------------------

def build_live_window(ws: str) -> Dict[str, Any]:
    """第四段：真机窗口**显式 blocked** + 回填入口命令与清单。"""
    plan_path = os.path.join(ws, "workcell", "baselines", "network_plan.yaml")
    status = "blocked"
    reason = ""
    try:
        import yaml

        with open(plan_path, "r", encoding="utf-8") as fh:
            plan = yaml.safe_load(fh) or {}
        status = str(plan.get("status") or "blocked")
        reason = str(((plan.get("blocked") or {}).get("reason") or ""))[:400]
    except Exception as exc:  # noqa: BLE001 - 读不到基线 → 保守 blocked
        reason = "读取 network_plan.yaml 失败: %r" % (exc,)

    backfill_cmd = (
        "rmctl run pick_place --live --arm left --ip <ip> --count 50 "
        "--trace build/run/pick_place_trace_live.jsonl "
        "--report build/reports/s06_pick_place_live.json"
    )
    items = [
        {"item": "真机 50 圈连续成功（无人干预）",
         "proven": False,
         "backfill": "真机窗口执行上条命令（--count 50），核对报告 "
                     "iterations_completed == 50 且 status == ok"},
        {"item": "真机 trace 来自真机执行",
         "proven": False,
         "backfill": "核对 build/run/pick_place_trace_live.jsonl 的事件来源"
                     "（真机执行层事件，非 fake）"},
        {"item": "真机失败给出下一步",
         "proven": False,
         "backfill": "在真机窗口注入一次失败，"
                     "用 `rmctl run --replay build/run/pick_place_trace_live.jsonl` "
                     "核对最后一条非空 next_action"},
        {"item": "真机单连接多段建连（rm_executor.session）",
         "proven": False,
         "backfill": "真机窗口用同一进程完成多段执行（一条连接、不重复建连），"
                     "核对会话层报告"},
    ]
    live_assertions = [
        {"name": "live_window 段显式 blocked（真机窗口不可达）",
         "passed": status == "blocked", "observed": status},
        {"name": "真机四项验收均标记为未实证",
         "passed": all(not item["proven"] for item in items),
         "observed": len(items)},
        {"name": "给出可粘贴的真机回填入口命令",
         "passed": bool(backfill_cmd.strip()), "observed": backfill_cmd},
    ]
    return {
        "status": status,
        # 本段的 status 是**真机窗口状态**（blocked），不是「断言是否通过」；
        # 断言结论单独放在 passed 里。
        "passed": all(bool(a.get("passed")) for a in live_assertions),
        "reason": reason or LIVE_NOTE,
        "next_action": ("真机窗口内执行: %s" % backfill_cmd),
        "mode": "live",
        "note": LIVE_NOTE,
        "network_plan": os.path.relpath(plan_path, REPO_ROOT),
        "items": items,
        "backfill_command": backfill_cmd,
        "backfill_checklist": [
            "1) 真机窗口确认 workcell/baselines/network_plan.yaml status=measured"
            "（连通性实测确认）",
            "2) 执行: %s" % backfill_cmd,
            "3) 核对 build/reports/s06_pick_place_live.json："
            "iterations_completed == 50、status == ok、无失败记录",
            "4) 核对 build/run/pick_place_trace_live.jsonl 可回放，"
            "失败时给出非空 next_action",
            "5) 把本报告 live_window 段的四项 proven 逐条改为 true 并附证据路径",
        ],
        "assertions": live_assertions,
    }


# --------------------------------------------------------------------------
# 6. 主流程
# --------------------------------------------------------------------------

def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="S06 取放工艺编排层离线证据链（mode=fake，不代表真机结论）")
    parser.add_argument("--out",
                        default=os.path.join("build", "reports",
                                             "s06_pick_place_offline.json"),
                        help="报告输出路径（默认 build/reports/s06_pick_place_offline.json）")
    parser.add_argument("--ws", default=REPO_ROOT, help="工作区根（含 workcell/ 的目录）")
    parser.add_argument("--task-file", default=DEFAULT_TASK_FILE,
                        help="工艺任务规格（默认 workcell/tasks/pick_place.yaml）")
    parser.add_argument("--iterations", type=int, default=0,
                        help="覆盖圈数（默认 0 = 用规格里的 iterations=50）")
    args = parser.parse_args(argv)

    ws = os.path.abspath(args.ws)
    out_path = args.out if os.path.isabs(args.out) else os.path.join(REPO_ROOT, args.out)
    trace_path = os.path.join(REPO_ROOT, TRACE_MAIN)

    status_before = git_status()

    # -- 依赖就绪：模型（必要时重新物化）-------------------------------
    try:
        model, model_source, model_evidence = ensure_model(ws)
    except Exception as exc:  # noqa: BLE001 - 模型不可用 → 诚实降级（不崩）
        report = {
            "status": "error",
            "reason": "模型不可用（厂商规划内核无法加载 build/model）: %s" % exc,
            "next_action": ("执行 `python3 -m rmctl init` 物化 build/model 后重跑"
                            "（若提示 GLIBCXX，先 `source scripts/rmenv.sh`）"),
            "mode": "fake",
            "generated_at": _now(),
            "repo_root": REPO_ROOT,
            "script": os.path.relpath(os.path.abspath(__file__), REPO_ROOT),
            "failures": ["model"],
        }
        _write_json(out_path, report)
        print("模型不可用: %s" % exc)
        return 4

    # -- 工艺规格（唯一入口）-------------------------------------------
    task_path = args.task_file if os.path.isabs(args.task_file) \
        else os.path.join(ws, args.task_file)
    spec_result = load_task_spec(path=task_path, ws=ws)
    if spec_result.get("status") != "ok":
        report = {
            "status": "error",
            "reason": "工艺任务规格不可用: %s" % spec_result.get("reason"),
            "next_action": spec_result.get("next_action", ""),
            "mode": "fake",
            "generated_at": _now(),
            "repo_root": REPO_ROOT,
            "script": os.path.relpath(os.path.abspath(__file__), REPO_ROOT),
            "failures": ["spec"],
        }
        _write_json(out_path, report)
        print("工艺任务规格不可用: %s" % spec_result.get("reason"))
        print("下一步: %s" % spec_result.get("next_action"))
        return 4
    spec = dict(spec_result["spec"])
    if args.iterations:
        spec["iterations"] = int(args.iterations)

    # -- 阈值（唯一来源链）---------------------------------------------
    thresholds, threshold_info = load_thresholds(ws)
    if thresholds is None:
        report = {
            "status": "error",
            "reason": "夹爪配置不可用: %s" % threshold_info.get("config_reason"),
            "next_action": "按 config_status 的 next_action 修复工位文件后重跑",
            "mode": "fake",
            "generated_at": _now(),
            "repo_root": REPO_ROOT,
            "failures": ["thresholds"],
        }
        _write_json(out_path, report)
        print("夹爪配置不可用: %s" % threshold_info.get("config_reason"))
        return 4

    # -- 规划器（真实规划 + 真实四道校验 + retime 自愈 + 缓存）----------
    planner = clients_mod.LivePlanner(ws=ws, model=model, cache=True)

    # -- 四段证据 -------------------------------------------------------
    success, motion_counter = run_success(ws, spec, thresholds, planner, trace_path)
    faults = run_fault_cases(ws, spec, thresholds, planner)
    fault_traces = [os.path.join(REPO_ROOT, TRACE_FAULT_DIR,
                                 "pick_place_trace_fault_%s.jsonl" % fault)
                    for fault in FAULT_EXPECT]
    replay = run_replay([trace_path] + fault_traces, motion_counter)
    live = build_live_window(ws)

    status_after = git_status()
    tracked_before = [l for l in status_before if not l.startswith("??")]
    tracked_after = [l for l in status_after if not l.startswith("??")]
    worktree_clean = (status_before == status_after) and not tracked_after

    sections = (("success_run", success), ("fault_cases", faults),
                ("replay", replay), ("live_window", live))
    failed_sections = [name for name, section in sections
                       if not bool(section.get("passed"))]

    report: Dict[str, Any] = {
        "status": "ok" if not failed_sections else "error",
        "reason": ("离线证据链四段全部通过：50 圈全流程 / 六类故障分支 / "
                   "trace 回放对照 / live_window 声明"
                   if not failed_sections else
                   "离线证据链失败段落: %s" % ", ".join(failed_sections)),
        "next_action": ("" if not failed_sections else
                        "对照失败段落的 assertions 字段修正后重跑本脚本"),
        "mode": "fake",
        "evidence": [
            FAKE_NOTE,
            LIVE_NOTE,
            "规格来源: %s" % spec_result.get("reason"),
            "阈值来源: %s（代理补齐: %s）"
            % (threshold_info.get("thresholds_source"),
               ", ".join(threshold_info.get("proxy_used_keys") or []) or "无"),
            "模型来源: %s（%s）" % (model_source, "；".join(model_evidence)),
        ],
        "generated_at": _now(),
        "repo_root": REPO_ROOT,
        "script": os.path.relpath(os.path.abspath(__file__), REPO_ROOT),
        "model_source": model_source,
        "model_evidence": model_evidence,
        "planner_cache_entries": len(getattr(planner, "_cache", {}) or {}),
        "planner_cache_note": ("规划缓存条目数 = 不同（起点, 目标）组合数；"
                              "50 圈 × 5 段共 250 次规划请求 → 只做 N 次真实规划"),
        "load_profile": {
            "iterations": spec.get("iterations"),
            "motion_calls": success.get("motion_calls"),
            "planner_cache_entries": len(getattr(planner, "_cache", {}) or {}),
            "trace_events": success.get("trace_events"),
            "wall_time_s": success.get("wall_time_s"),
            "iteration_duration_stats": success.get("iteration_duration_stats"),
            "peak_rss_kb": peak_rss_kb(),
            "trace_bytes": _file_size(trace_path),
            "report_bytes": _file_size(out_path),
        },
        "task_spec": {
            "path": os.path.relpath(task_path, REPO_ROOT),
            "iterations": spec.get("iterations"),
            "segments": [seg.get("name") for seg in (spec.get("segments") or [])],
            "version": spec.get("version"),
        },
        "thresholds": {
            key: (list(value) if isinstance(value, tuple) else value)
            for key, value in thresholds.items()
        },
        "thresholds_info": threshold_info,
        "worktree_clean": worktree_clean,
        "worktree_status_before": status_before,
        "worktree_status_after": status_after,
        "failures": failed_sections,
    }
    for name, section in sections:
        report[name] = section

    _write_json(out_path, report)
    # 报告自身大小（写盘后回填，再写一次）
    report["load_profile"]["report_bytes"] = _file_size(out_path)
    _write_json(out_path, report)

    print("离线证据链报告: %s" % out_path)
    for name, section in sections:
        print("  %-12s %-6s %s" % (name, section.get("status"),
                                   section.get("reason")))
    print("  总体: %s（%s）" % (report["status"], report["reason"]))
    print("  worktree_clean=%s（脚本只写 build/ 下产物）" % worktree_clean)
    print("注意: %s" % LIVE_NOTE)
    return 0 if not failed_sections else 4


if __name__ == "__main__":
    sys.exit(main())
