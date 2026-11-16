#!/usr/bin/env python3
"""S05 夹爪接入的**离线证据链**（开合 100 次 / 六类判定各 10 次 / 运动互锁演示）。

用法
----
    python3 scripts/gripper_offline_evidence.py --out build/reports/s05_offline_gripper.json

退出码
------
* ``0`` —— 三段证据全部通过（断言全绿）；
* ``4`` —— 存在断言失败（报告会写明哪一段、哪一条断言失败）。

**诚实分层（重要）**
--------------------
本脚本跑在 **离线假夹爪**（``rm_hw_bridge.gripper.FakeGripper``）与
**假传输层**（``rm_executor.transport.FakeTransport``）上，因此报告里**所有**
数字一律带 ``mode=fake`` 标注——它们**证明代码路径正确**，但**不是真机结论**。
真机窗口的四项验收（真实开合 100 次、真实六类判定、真实互锁拒绝、真实标定）
本轮**一律未实证**，回填入口见 ``docs/rm_gripper接入与状态判定.md`` §8。

三段证据
--------
1. **开合 100 次**：``GripperAdapter`` + ``FakeGripper`` 走 100 轮「张开 / 夹取」，
   统计超时次数与失败明细；断言 ``timeout_count == 0``。
   额外做一次**超时检测自检**（脚本化一次超时），证明超时计数器不是恒零的空断言。
2. **六类判定各 10 次**：对「夹到 / 夹空 / 堵转 / 过温 / 过流 / 离线」各注入
   10 组状态，调用**唯一判定真值** ``rm_hw_bridge.gripper_state.judge``，
   断言每类 10/10 判定结果与 severity 都符合预期；另跑**负例**：
   标定系数为 ``null`` 时判定必须是 ``needs_param``，且**不得**产出
   ``grasp_ok`` / ``grasp_empty``。
3. **互锁演示**：把 ``MotionGate`` 置成 ``GRIPPER_BUSY`` 与 ``GRIPPER_UNJUDGED``
   各跑一次 ``rm_executor.Executor.run()``（ManualClock + FakeTransport），
   断言报告里出现 ``gripper_interlock_blocked`` 事件，且
   ``preposition`` / ``transmit`` 调用数**均为 0**（零运动下发）。

零新依赖：只用标准库 + 仓库内既有包（``rm_hw_bridge`` / ``rm_executor`` / ``rm_safety``）。
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import tempfile
from datetime import datetime
from typing import Any, Dict, List, Optional, Tuple

# --------------------------------------------------------------------------
# 0. 自插 sys.path（脚本可从任意目录调用）
# --------------------------------------------------------------------------

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(HERE)

for _rel in (
    "dualarm_ws/src/rm_hw_bridge",
    "dualarm_ws/src/rm_executor",
    "dualarm_ws/src/rm_safety",
):
    _p = os.path.join(REPO_ROOT, _rel)
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

from rm_executor.clock import ManualClock  # noqa: E402
from rm_executor.executor import Executor, ExecutorConfig  # noqa: E402
from rm_executor.feedback import ScriptedFeedback  # noqa: E402
from rm_executor.transport import FakeTransport  # noqa: E402
from rm_hw_bridge import gripper as G  # noqa: E402
from rm_hw_bridge import gripper_config as GC  # noqa: E402
from rm_hw_bridge import gripper_state as S  # noqa: E402

#: 离线代理的**固定声明**（必须出现在每一段 evidence 里）
FAKE_NOTE = "mode=fake（离线假夹爪，代理模型，不代表真机结论）"

#: 真机四项验收的**未实证声明**（必须出现在报告里）
LIVE_NOTE = (
    "真机四项验收（真实开合 100 次 / 真实六类判定 / 真实互锁拒绝 / 真实标定）"
    "本轮未实证：真机窗口 blocked（workcell/baselines/network_plan.yaml 状态），"
    "回填入口见 docs/rm_gripper接入与状态判定.md §8"
)

#: **代理阈值**（真机标定未回填时的离线判据；必须标注「不代表真机结论」）
PROXY_THRESHOLDS: Dict[str, Any] = {
    "force_threshold": 100.0,
    "grasp_position_window": (10.0, 30.0),
    "position_mm_per_unit": 0.05,
    "force_scale": 0.01,
}

#: 六类判定 → （中文标签，期望 severity）
FAULT_EXPECT: Dict[str, Tuple[str, str]] = {
    "grasp_ok": ("夹到", S.SEV_OK),
    "grasp_empty": ("夹空", S.SEV_RECOVERABLE),
    "stall": ("堵转", S.SEV_RECOVERABLE),
    "overtemp": ("过温", S.SEV_NEEDS_HUMAN),
    "overcurrent": ("过流", S.SEV_NEEDS_PARAM),
    "offline": ("离线", S.SEV_RECOVERABLE),
}


# --------------------------------------------------------------------------
# 1. 工具：配置 / 阈值
# --------------------------------------------------------------------------

def load_config(ws: str) -> Dict[str, Any]:
    """读夹爪配置（唯一来源 = ``workcell/rm_workcell.yaml::gripper``）。

    配置不可用时**不猜默认值**：退回代理阈值并显式标注来源。
    """
    res = GC.load_gripper_config(ws)
    config: Dict[str, Any] = dict(res.get("config") or {})
    config["_status"] = res.get("status")
    config["_reason"] = res.get("reason", "")
    config["_evidence"] = list(res.get("evidence") or [])
    return config


def effective_thresholds(config: Dict[str, Any]) -> Tuple[Dict[str, Any], List[str], str]:
    """阈值 = 工位配置（已回填项） + 代理缺省（未回填项）。

    返回 ``(thresholds, proxy_used_keys, source)``；``source`` 取
    ``"workcell_config"``（全部来自工位文件）或 ``"proxy_default"``（有代理补齐）。
    """
    raw = dict((config or {}).get("thresholds") or {})
    thresholds: Dict[str, Any] = {}
    proxy_used: List[str] = []
    for key, proxy_value in PROXY_THRESHOLDS.items():
        value = raw.get(key)
        if value is None:
            thresholds[key] = proxy_value
            proxy_used.append(key)
        else:
            thresholds[key] = value
    window = thresholds.get("grasp_position_window")
    if isinstance(window, (list, tuple)):
        thresholds["grasp_position_window"] = (float(window[0]), float(window[1]))
    source = "proxy_default" if proxy_used else "workcell_config"
    return thresholds, proxy_used, source


# --------------------------------------------------------------------------
# 2. 第一段：开合 100 次
# --------------------------------------------------------------------------

def apply_action(adapter: "G.GripperAdapter", path: str, action: str,
                 speed: int, force: int, timeout_ms: int) -> Dict[str, Any]:
    """下发一次夹爪动作（open = 张开 / close = 夹取），返回硬件层统一结果。

    * ``gripper_api`` 路径 → 官方快路径（``rm_set_gripper_release`` / ``rm_set_gripper_pick``）；
    * ``modbus`` 路径 → 手册寄存器（0x0006 松开到最大开口；0x0009 目标开口度 +
      0x000A 速度 + 0x000B 力度）。
    """
    if path == "gripper_api":
        if action == "open":
            return adapter.gripper_release(speed, True, timeout_ms)
        return adapter.gripper_pick(speed, force, True, timeout_ms)
    if action == "open":
        return adapter.write_register(G.REG_OPEN_MAX, 1)
    for reg, value in ((G.REG_SPEED, int(speed)), (G.REG_FORCE, int(force)),
                       (G.REG_TARGET_POS, 0)):
        res = adapter.write_register(reg, value)
        if res.get("status") != "ok":
            return res
    return {
        "status": "ok",
        "reason": "夹爪闭合动作已下发（目标开口度=0，速度=%d，力度=%d）" % (speed, force),
        "next_action": "",
        "evidence": ["写寄存器 0x%04X=0（目标开口度）" % G.REG_TARGET_POS],
    }


def _make_adapter(config: Dict[str, Any], path: str, port: "G.GripperPort",
                  thresholds: Dict[str, Any]) -> "G.GripperAdapter":
    return G.GripperAdapter(
        port,
        path=path,
        device_id=int((config or {}).get("device_id") or 1),
        baud=int((config or {}).get("baud") or 115200),
        timeout_100ms=max(1, int((config or {}).get("timeout_ms") or 500) // 100),
        base=1,
        position_mm_per_unit=thresholds.get("position_mm_per_unit"),
        force_scale=thresholds.get("force_scale"),
    )


def run_cycle(config: Dict[str, Any], thresholds: Dict[str, Any],
              count: int) -> Dict[str, Any]:
    """第一段证据：``count`` 次开合循环，统计超时与失败。"""
    path = (config or {}).get("path") or "modbus"
    timeout_ms = int((config or {}).get("action_timeout_ms") or 2000)
    port = G.FakeGripper(base=1, registers={
        G.REG_STATE: 2, G.REG_ACTUAL_TEMP: 25, G.REG_ACTUAL_CURRENT: 120,
    }, max_records=1000)
    adapter = _make_adapter(config, path, port, thresholds)

    timeouts = 0
    failures: List[Dict[str, Any]] = []
    actions = 0
    opened = adapter.open_protocol()
    try:
        for index in range(count):
            for act in ("open", "close"):
                res = apply_action(adapter, path, act, 100, 100, timeout_ms)
                actions += 1
                if res.get("status") != "ok":
                    if res.get("rc") == G.RC_RECV_TIMEOUT:
                        timeouts += 1
                    failures.append({
                        "cycle": index,
                        "action": act,
                        "status": res.get("status"),
                        "rc": res.get("rc"),
                        "reason": res.get("reason"),
                    })
    finally:
        closed = adapter.close_protocol()

    # -- 超时检测自检：脚本化一次超时，证明计数器不是恒零的空断言 ------
    probe_port = G.FakeGripper(base=1, timeout_all=True)
    probe_adapter = _make_adapter(config, path, probe_port, thresholds)
    probe_res = apply_action(probe_adapter, path, "open", 100, 100, timeout_ms)
    timeout_detected = (
        probe_res.get("rc") == G.RC_RECV_TIMEOUT
        and probe_res.get("status") != "ok"
    )

    ok = (not failures) and timeouts == 0 and timeout_detected and opened.get("status") == "ok"
    return {
        "status": "ok" if ok else "error",
        "reason": ("代理模式完成 %d 次开合（%d 次动作，%d 次超时，%d 次失败）"
                   % (count, actions, timeouts, len(failures))),
        "next_action": ("在真机窗口用 `rmctl gripper cycle --count 100` 复核真实动作"
                        "（本次为%s）" % FAKE_NOTE),
        "evidence": [
            FAKE_NOTE,
            "协议生命周期: %s → %s" % (opened.get("reason"), closed.get("reason")),
            "配置来源: %s" % ((config or {}).get("source") or "workcell/rm_workcell.yaml::gripper"),
            "超时检测自检: %s（脚本化一次超时，计数器非恒零）"
            % ("通过" if timeout_detected else "失败"),
        ],
        "mode": "fake",
        "cycles": count,
        "actions": actions,
        "timeouts": timeouts,
        "failures": failures[:20],
        "failure_count": len(failures),
        "timeout_detector_selfcheck": timeout_detected,
        "protocol_open": opened.get("status") == "ok",
        "protocol_close": closed.get("status") == "ok",
        "assertions": [
            {"name": "timeout_count == 0", "passed": timeouts == 0,
             "observed": timeouts},
            {"name": "failure_count == 0", "passed": not failures,
             "observed": len(failures)},
            {"name": "timeout_detector_selfcheck", "passed": timeout_detected,
             "observed": timeout_detected},
            {"name": "protocol lifecycle open→close", "passed":
                opened.get("status") == "ok" and closed.get("status") == "ok",
             "observed": "%s / %s" % (opened.get("status"), closed.get("status"))},
        ],
    }


# --------------------------------------------------------------------------
# 3. 第二段：六类判定各 10 次
# --------------------------------------------------------------------------

def make_fixture(fault: str, thresholds: Dict[str, Any]) -> Tuple["G.GripperPort", str]:
    """构造一个能触发指定判定的假夹爪（返回 ``(port, path)``）。

    * 夹到 / 夹空 / 堵转 / 过温 / 过流 → Modbus 路径（力度寄存器 0x000E 本身
      即 0–1000 标度，可直接得 ``force_norm``）；
    * 离线 → 官方夹爪快路径（Modbus 路径下「读不到」只能是通讯错误，
      **不是离线判定**）。
    """
    window = list(thresholds.get("grasp_position_window") or [10.0, 30.0])
    center = (float(window[0]) + float(window[1])) / 2.0
    ppu = float(thresholds.get("position_mm_per_unit") or 0.05)
    pos_raw = int(round(center / ppu)) if ppu else 400
    force_threshold = float(thresholds.get("force_threshold") or 100.0)

    error_bits = 0
    force_raw = min(1000, int(round(force_threshold * 2)))
    temp = 25
    path = "modbus"
    online = True

    if fault == "grasp_empty":
        force_raw = max(0, int(force_threshold // 2))
    elif fault == "stall":
        error_bits = S.ERROR_BIT_STALL
    elif fault == "overtemp":
        error_bits = 2
        temp = 85
    elif fault == "overcurrent":
        error_bits = 4
    elif fault == "offline":
        path = "gripper_api"
        online = False

    registers = {
        G.REG_ACTUAL_POS: pos_raw,
        G.REG_ACTUAL_FORCE: force_raw,
        G.REG_ACTUAL_TEMP: temp,
        G.REG_ACTUAL_CURRENT: 120,
        G.REG_ERROR: error_bits,
        G.REG_STATE: 2,
    }
    return G.FakeGripper(base=1, registers=registers, online=online, mode=2), path


def run_judgments(config: Dict[str, Any], thresholds: Dict[str, Any],
                  count: int) -> Dict[str, Any]:
    """第二段证据：六类判定各注入 ``count`` 次，断言结果与 severity。"""
    rows: List[Dict[str, Any]] = []
    mismatches_total = 0

    for fault, (label, expected_severity) in FAULT_EXPECT.items():
        correct = 0
        severity_mismatch = 0
        mismatches: List[Dict[str, Any]] = []
        for index in range(count):
            port, path = make_fixture(fault, thresholds)
            adapter = _make_adapter(config, path, port, thresholds)
            state_res = adapter.read_state()
            if state_res.get("status") != "ok":
                mismatches.append({
                    "index": index, "expected": fault, "actual": "read_error",
                    "reason": state_res.get("reason"),
                })
                continue
            judgment = S.judge(state_res.get("state"), thresholds)
            if judgment.result == fault:
                correct += 1
            else:
                mismatches.append({
                    "index": index, "expected": fault, "actual": judgment.result,
                    "reason": judgment.reason,
                    "error_flags": judgment.evidence.get("error_flags"),
                })
            if judgment.severity != expected_severity:
                severity_mismatch += 1
                mismatches.append({
                    "index": index, "expected_severity": expected_severity,
                    "actual_severity": judgment.severity, "kind": "severity",
                })
        mismatches_total += len(mismatches)
        rows.append({
            "fault": fault,
            "label": label,
            "injected": count,
            "correct": correct,
            "accuracy": (correct / float(count)) if count else 0.0,
            "expected_severity": expected_severity,
            "severity_mismatch": severity_mismatch,
            "mismatches": mismatches[:10],
            "passed": correct == count and severity_mismatch == 0,
        })

    # -- 负例：标定系数为 null → 必须 needs_param，且不得产出夹到/夹空 ----
    null_thresholds = {key: None for key in S.REQUIRED_GRASP_THRESHOLDS}
    negative: Dict[str, Any] = {}
    port, path = make_fixture("grasp_ok", thresholds)
    adapter = _make_adapter(config, path, port, null_thresholds)
    state_res = adapter.read_state()
    neg_judgment = S.judge(state_res.get("state"), null_thresholds)
    negative = {
        "name": "标定系数为 null → needs_param（不得产出 grasp_ok / grasp_empty）",
        "result": neg_judgment.result,
        "severity": neg_judgment.severity,
        "reason": neg_judgment.reason,
        "thresholds_missing": neg_judgment.evidence.get("thresholds_missing"),
        "passed": (neg_judgment.result == S.RESULT_NEEDS_PARAM
                   and neg_judgment.result not in (S.RESULT_GRASP_OK, S.RESULT_GRASP_EMPTY)
                   and neg_judgment.severity == S.SEV_NEEDS_PARAM),
    }

    # -- 红线自检：恢复阶梯里不得出现「写 0x0008 清错误码」 --------------
    ladder_violations = S.recovery_ladder_violations()

    ok = (all(row["passed"] for row in rows)
          and negative["passed"]
          and not ladder_violations)
    return {
        "status": "ok" if ok else "error",
        "reason": ("六类判定各 %d 次全部正确（共 %d 次注入，%d 处不符）"
                   % (count, count * len(FAULT_EXPECT), mismatches_total)
                   if ok else
                   "判定不符 %d 处（%d 类 × %d 次注入）"
                   % (mismatches_total, len(FAULT_EXPECT), count)),
        "next_action": ("" if ok else
                        "对照 rm_hw_bridge.gripper_state 的判定优先级复核注入夹具与阈值后重跑"),
        "evidence": [
            FAKE_NOTE,
            "判定真值: rm_hw_bridge.gripper_state.judge（本脚本不另写判定）",
            "注入: %s" % ", ".join("%s×%d" % (f, count) for f in FAULT_EXPECT),
            "恢复阶梯红线自检: %s" % ("干净（无禁止动作）" if not ladder_violations
                                     else "违规 %d 处" % len(ladder_violations)),
        ],
        "mode": "fake",
        "faults": rows,
        "mismatches": mismatches_total,
        "negative": negative,
        "recovery_ladder_violations": ladder_violations,
        "assertions": [
            {"name": "六类判定各 %d/%d 正确" % (count, count),
             "passed": all(row["passed"] for row in rows),
             "observed": {row["fault"]: "%d/%d" % (row["correct"], row["injected"])
                          for row in rows}},
            {"name": "severity 符合预期", "passed": ok,
             "observed": {row["fault"]: row["expected_severity"] for row in rows}},
            {"name": "负例: 标定为 null → needs_param", "passed": negative["passed"],
             "observed": negative["result"]},
            {"name": "恢复阶梯无禁止动作（不写 0x0008）",
             "passed": not ladder_violations,
             "observed": len(ladder_violations)},
        ],
    }


# --------------------------------------------------------------------------
# 4. 第三段：运动互锁演示
# --------------------------------------------------------------------------

EXEC_CONFIG = ExecutorConfig(
    period_s=0.010,
    follow=False,
    trajectory_mode=1,
    radio=30,
    pre_position_speed=10,
)


def _traj_payload() -> Dict[str, Any]:
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


def _run_executor_with_gate(gate: Any, traj_path: str) -> Tuple[Dict[str, Any], FakeTransport]:
    """跑一次执行（带闸门），返回 ``(report, transport)``。"""
    transport = FakeTransport()

    def _script(_index):
        call = transport.last_call("transmit") or transport.last_call("preposition")
        if call is None:
            return [0.0] * 7
        return list(call["args"]["deg7"])

    feedback = ScriptedFeedback(script=_script, speed_deg_s=0.0, period_s=0.010)
    executor = Executor(transport, feedback, config=EXEC_CONFIG,
                        clock=ManualClock(period_s=0.010), mode="fake",
                        motion_gate=gate)
    report = executor.run(traj_path)
    return report, transport


def run_interlock(work_dir: str) -> Dict[str, Any]:
    """第三段证据：闸门非 IDLE → 拒绝下发任何运动。"""
    traj_path = os.path.join(work_dir, "s05_interlock_traj.json")
    with open(traj_path, "w", encoding="utf-8") as fh:
        json.dump(_traj_payload(), fh, ensure_ascii=False, indent=2)

    cases: List[Dict[str, Any]] = []
    for gate_state in (S.MotionGate.STATE_GRIPPER_BUSY, S.MotionGate.STATE_GRIPPER_UNJUDGED):
        gate = S.MotionGate()
        if gate_state == S.MotionGate.STATE_GRIPPER_BUSY:
            gate.begin_action("离线演示：夹爪动作进行中")
        else:
            gate.mark_unjudged("离线演示：夹爪动作已结束但状态未判定")
        report, transport = _run_executor_with_gate(gate, traj_path)
        blocked = [e for e in report.get("events", []) if e.get("type") == "gripper_interlock_blocked"]
        preposition = transport.count("preposition")
        transmit = transport.count("transmit")
        passed = (len(blocked) == 1
                  and blocked[0].get("severity") == S.SEV_NEEDS_HUMAN
                  and report.get("status") == "blocked"
                  and preposition == 0 and transmit == 0)
        cases.append({
            "gate_state": gate_state,
            "blocked_events": len(blocked),
            "severity": blocked[0].get("severity") if blocked else None,
            "reason": blocked[0].get("reason") if blocked else None,
            "evidence": blocked[0].get("evidence") if blocked else [],
            "next_action": blocked[0].get("next_action") if blocked else "",
            "report_status": report.get("status"),
            "preposition_calls": preposition,
            "transmit_calls": transmit,
            "passed": passed,
        })

    ok = all(case["passed"] for case in cases)
    return {
        "status": "ok" if ok else "error",
        "reason": ("闸门非 IDLE 时运动被正确互锁（%d 个场景，%d 次零运动下发断言通过）"
                   % (len(cases), len(cases))
                   if ok else "互锁演示未通过"),
        "next_action": ("在真机窗口由 rm_runtime 与夹爪客户端同进程复核互锁拒绝"
                        "（本次为%s）" % FAKE_NOTE),
        "evidence": [
            FAKE_NOTE,
            "互锁接线: 阶段 B（预就位）前 + 阶段 C（起流）前各查一次闸门",
            "零运动下发断言: preposition=0 且 transmit=0",
        ],
        "mode": "fake",
        "cases": cases,
        "assertions": [
            {"name": "闸门非 IDLE → gripper_interlock_blocked 事件",
             "passed": all(case["blocked_events"] == 1 for case in cases),
             "observed": {case["gate_state"]: case["blocked_events"] for case in cases}},
            {"name": "零运动下发（preposition=0, transmit=0）",
             "passed": all(case["preposition_calls"] == 0 and case["transmit_calls"] == 0
                           for case in cases),
             "observed": {case["gate_state"]: "pre=%d, transmit=%d"
                          % (case["preposition_calls"], case["transmit_calls"])
                          for case in cases}},
        ],
    }


# --------------------------------------------------------------------------
# 5. 主流程
# --------------------------------------------------------------------------

def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="S05 夹爪接入离线证据链（mode=fake，不代表真机结论）")
    parser.add_argument("--out", default=os.path.join("build", "reports", "s05_offline_gripper.json"),
                        help="报告输出路径（相对仓库根；默认 build/reports/s05_offline_gripper.json）")
    parser.add_argument("--ws", default=REPO_ROOT, help="工作区根（含 workcell/ 的目录）")
    parser.add_argument("--cycles", type=int, default=100, help="开合循环次数（默认 100）")
    parser.add_argument("--judge-count", type=int, default=10, help="每类判定注入次数（默认 10）")
    args = parser.parse_args(argv)

    out_path = args.out if os.path.isabs(args.out) else os.path.join(REPO_ROOT, args.out)
    out_dir = os.path.dirname(out_path)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)

    config = load_config(args.ws)
    thresholds, proxy_used, thresholds_source = effective_thresholds(config)

    cycle = run_cycle(config, thresholds, args.cycles)
    judgment = run_judgments(config, thresholds, args.judge_count)
    with tempfile.TemporaryDirectory(prefix="s05_gripper_evidence_") as work_dir:
        interlock = run_interlock(work_dir)

    sections = (("cycle", cycle), ("judgment", judgment), ("interlock", interlock))
    failures = [name for name, section in sections if section.get("status") != "ok"]

    report: Dict[str, Any] = {
        "status": "ok" if not failures else "error",
        "reason": ("离线证据链三段全部通过：开合 %d 次零超时 / 六类判定各 %d 次全对 / "
                   "运动互锁零下发" % (args.cycles, args.judge_count)
                   if not failures else
                   "离线证据链失败段落: %s" % ", ".join(failures)),
        "next_action": ("" if not failures else
                        "对照失败段落的 assertions 字段修正后重跑本脚本"),
        "mode": "fake",
        "evidence": [
            FAKE_NOTE,
            LIVE_NOTE,
            "阈值来源: %s%s" % (
                thresholds_source,
                ("（代理补齐: %s）" % ", ".join(proxy_used)) if proxy_used else ""),
            "配置来源: %s" % ((config or {}).get("source") or "workcell/rm_workcell.yaml::gripper"),
            "配置状态: %s%s" % (config.get("_status"),
                               ("（%s）" % config.get("_reason")) if config.get("_reason") else ""),
        ],
        "generated_at": datetime.now().isoformat(timespec="seconds"),
        "repo_root": REPO_ROOT,
        "script": os.path.relpath(os.path.abspath(__file__), REPO_ROOT),
        "cycles_requested": args.cycles,
        "judge_count_requested": args.judge_count,
        "thresholds": {
            key: (list(value) if isinstance(value, tuple) else value)
            for key, value in thresholds.items()
        },
        "thresholds_source": thresholds_source,
        "live_window": {
            "status": "blocked",
            "reason": LIVE_NOTE,
            "next_action": ("按 docs/rm_gripper接入与状态判定.md §8 执行："
                            "rmctl gripper probe → 回填标定 → cycle --count 100 → "
                            "judge --fault each --count 10，再把本报告 mode 改为 live"),
        },
        "failures": failures,
    }
    for name, section in sections:
        report[name] = section

    with open(out_path, "w", encoding="utf-8") as fh:
        json.dump(report, fh, ensure_ascii=False, indent=2, default=str)

    print("离线证据链报告: %s" % out_path)
    for name, section in sections:
        print("  %-10s %-6s %s" % (name, section.get("status"), section.get("reason")))
    print("  总体: %s（%s）" % (report["status"], report["reason"]))
    print("注意: %s" % LIVE_NOTE)
    return 0 if not failures else 4


if __name__ == "__main__":
    sys.exit(main())
