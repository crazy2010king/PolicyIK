"""rm_executor 命令行入口（离线 fake 与真机 live **同一 Executor、同一报告 schema**）。

用法::

    python3 -m rm_executor.cli --traj <path> --mode fake|live \
        [--report <out.json>] [--duration <s>] [--realtime] \
        [--allow-unverified] [--arm left|right]

设计要点（与切片契约一致）：

* ``--mode fake`` —— **零真机依赖**：假传输（``FakeTransport``）+ 脚本化反馈
  （``ScriptedFeedback``，镜像最近一次下发的关节角 → 理想跟随模型）；
* ``--mode live`` —— 真机路径（``BridgeTransport`` + UDP 反馈泵），**与 fake
  共用同一个** :class:`rm_executor.executor.Executor` **与同一份** 报告 schema
  （:mod:`rm_executor.report`）；
* ``--duration`` —— 离线长跑：按指定秒数**重复执行**该轨迹（每一轮都是一次
  完整的「预就位 → 流式 → 到位」），合并成一份等效时长的报告；
* ``--realtime`` —— 用真实墙钟（``TickClock``）而不是确定性 ``ManualClock``；
* ``--allow-unverified`` —— 仅供**离线假执行**放行缺 ``verify`` 的轨迹
  （真机路径**拒绝**该放行）；
* ``--inject`` —— **仅离线 fake**：受控故障注入（跟随误差 / 反馈断流 /
  控制器抢轨迹），用于离线复跑异常判定路径；真机路径**拒绝**任何注入。

**退出码**沿用 ``rm_hw_bridge.errors`` 口径：``0`` ok / ``3`` blocked /
``4`` error。

**诚实分层**：控制器不可达时返回 ``status="error"`` + 明确的 reason +
可粘贴 next_action，**不伪造任何快照数值**（指标一律 ``None``）。

零 rmctl 依赖、零 tesseract 依赖；硬件层（``rm_hw_bridge``）只在 live
路径惰性导入。
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from typing import Any, Dict, List, Optional, Tuple

from rm_executor import contract
from rm_executor import injection as injection_mod
from rm_executor import metrics as metrics_mod
from rm_executor import report as report_mod
from rm_executor.clock import ManualClock, TickClock
from rm_executor.executor import Executor, ExecutorConfig
from rm_executor.feedback import ScriptedFeedback
from rm_executor.transport import FakeTransport

__all__ = [
    "DEFAULT_PERIOD_S",
    "LIVE_FEEDBACK_PERIOD_S",
    "build_parser",
    "execute",
    "main",
]

#: 默认栅格周期（秒）—— 10 ms 单时钟
DEFAULT_PERIOD_S = 0.010

#: 真机反馈采集周期（秒）—— 落在计划 §5.7 的 10–20 ms 区间
LIVE_FEEDBACK_PERIOD_S = 0.015

#: 离线假传输的**调用记录保留上限**（长跑时防内存无界增长；计数不受影响）
FAKE_CALL_RECORD_LIMIT = 2000

#: status 严重度排序（合并多轮报告时取最严重者）
_STATUS_RANK = {
    contract.STATUS_OK: 0,
    contract.STATUS_BLOCKED: 1,
    contract.STATUS_ERROR: 2,
}

#: 可粘贴的下一步：把执行层放进 PYTHONPATH
NEXT_ACTION_EXECUTOR_PATH = (
    "export PYTHONPATH=$PWD/dualarm_ws/src/rm_executor:$PYTHONPATH  "
    "# 执行层包 rm_executor"
)

#: 「未启用夹爪互锁」的证据前缀（**固定文案**，便于检索）
INTERLOCK_DISABLED_NOTE = "未启用夹爪互锁"

#: 「已启用夹爪互锁」的证据前缀（**固定文案**，便于检索）
INTERLOCK_ENABLED_NOTE = "已启用夹爪互锁"


class _Unset:
    """哨兵：区分「未传 motion_gate」与「显式传 None（不启用互锁）」。"""

    def __repr__(self) -> str:  # pragma: no cover - 调试辅助
        return "<unset>"


#: ``execute(motion_gate=...)`` 的缺省哨兵
_UNSET = _Unset()


def _make_motion_gate(ws: Optional[str] = None):
    """构造**共享**的夹爪运动闸门（``MotionGate``）。

    返回 ``(gate, note)``：

    * ``gate`` 为 ``MotionGate`` 实例 → 注入执行器，阶段 B / C 前会查；
    * ``gate`` 为 ``None`` → **不启用互锁**（既有行为完全不变），
      ``note`` 写明原因（固定前缀 :data:`INTERLOCK_DISABLED_NOTE`）。

    **惰性导入**（与 ``rm_executor.contract`` 同一纪律）：执行层不模块级
    import ``rm_hw_bridge``；导入失败 → 不启用互锁并留证据，**不假装有闸门**。

    共享对象：同进程内的夹爪动作客户端与执行器共用
    ``rm_hw_bridge.gripper_state.shared_motion_gate()`` 这**同一个**实例
    → 夹爪一动作，执行器的闸门立刻变为非 IDLE。
    """
    try:
        from rm_hw_bridge.gripper_state import shared_motion_gate
    except Exception as exc:  # noqa: BLE001 - 硬件层不可用 → 不启用互锁
        return None, ("%s（rm_hw_bridge.gripper_state 不可用: %s: %s）"
                      % (INTERLOCK_DISABLED_NOTE, type(exc).__name__, exc))
    # 夹爪配置未启用（工位文件缺 gripper: 段等）→ 不启用互锁
    try:
        from rm_hw_bridge import gripper_config as _gripper_config
        res = _gripper_config.load_gripper_config(ws)
    except Exception as exc:  # noqa: BLE001 - 配置读取异常 → 不启用互锁
        return None, ("%s（夹爪配置读取失败: %s: %s）"
                      % (INTERLOCK_DISABLED_NOTE, type(exc).__name__, exc))
    if not isinstance(res, dict) or res.get("status") != "ok":
        reason = ""
        if isinstance(res, dict):
            reason = str(res.get("reason") or "")
        return None, ("%s（夹爪配置不可用: %s）"
                      % (INTERLOCK_DISABLED_NOTE, reason or "未知原因"))
    gate = shared_motion_gate()
    return gate, ("%s（共享 MotionGate 实例，阶段 B / 阶段 C 前各查一次）"
                  % INTERLOCK_ENABLED_NOTE)


# --------------------------------------------------------------------- 工具
def _load_config(ws: Optional[str]) -> Tuple[Optional[ExecutorConfig], Dict[str, Any]]:
    """读执行配置（唯一来源 = workcell/rm_workcell.yaml::execution）。

    返回 ``(config, error)``；``config is None`` → ``error`` 非空（blocked）。
    """
    res = contract.load_execution_config(ws)
    if res.get("status") != contract.STATUS_OK:
        return None, res
    try:
        return ExecutorConfig.from_execution_config(res["config"]), {}
    except Exception as exc:  # noqa: BLE001 - 配置异常一律降级为 blocked
        return None, contract.blocked_result(
            "执行配置字段非法（%s）: %s" % (type(exc).__name__, exc),
            "修复 workcell/rm_workcell.yaml::execution 后重跑",
            [repr(exc)],
        )


def _mirror_feedback(transport: FakeTransport, period_s: float) -> ScriptedFeedback:
    """构造「镜像最近一次下发指令角」的脚本化反馈（理想跟随模型）。

    * 预就位阶段：镜像 ``preposition`` 的首点 → 预就位必然到位；
    * 流式阶段：镜像每一拍的 ``transmit`` 指令 → 跟随误差恒为 0；
    * 速度恒为 0 → 到位判定的速度条件成立。

    **这是离线代理模型，不是真机结论**（见 ``docs/rm_executor执行线程契约.md``）。
    """
    def _script(_index: int) -> Optional[List[float]]:
        call = transport.last_call("transmit") or transport.last_call("preposition")
        if call is None:
            return None  # 尚未下发任何指令 → 断流（不伪造快照）
        return list(call["args"]["deg7"])

    return ScriptedFeedback(script=_script, speed_deg_s=0.0, period_s=period_s)


def _leg_count(duration_s: Optional[float], traj_duration_s: Optional[float]) -> int:
    """长跑轮数：``--duration`` 指定的等效时长需要几轮轨迹。

    * ``duration_s`` 缺省 / 不大于轨迹时长 → 1 轮；
    * 否则 ``ceil(duration / traj_duration)``（轨迹时长非法 → 1 轮）。

    **每一轮都是一次完整执行**（预就位 → 流式 → 到位），因此长跑的每一段
    都受同一套限值与闭环判据约束（不会靠「延展轨迹」绕过限值）。
    """
    if not duration_s or duration_s <= 0:
        return 1
    if not traj_duration_s or traj_duration_s <= 0:
        return 1
    if duration_s <= traj_duration_s:
        return 1
    return int(math.ceil(duration_s / float(traj_duration_s)))


def _merge_reports(reports: List[Dict[str, Any]],
                   jitter_samples: List[float],
                   follow_errors: List[float]) -> Dict[str, Any]:
    """合并多轮（长跑）报告为一份统一报告。

    * status：取**最严重**的一轮；
    * reason / next_action：取最严重那一轮的；
    * events / evidence：按轮次拼接（evidence 带 ``legN:`` 前缀）；
    * metrics：tick 数 / 保持拍数 / 计划时长 **求和**，抖动与跟随误差
      **按原始样本重新统计**（不是对分位数再求分位数）。
    """
    if not reports:
        return report_mod.build_report(status=contract.STATUS_ERROR,
                                       reason="没有任何一轮执行结果",
                                       next_action="检查轨迹与执行配置后重跑",
                                       evidence=[], mode="fake")
    if len(reports) == 1:
        return reports[0]

    worst = reports[0]
    for rep in reports[1:]:
        if _STATUS_RANK.get(rep.get("status"), 0) > _STATUS_RANK.get(worst.get("status"), 0):
            worst = rep

    events: List[Dict[str, Any]] = []
    evidence: List[str] = []
    ticks = 0
    hold_ticks = 0
    planned_total = 0.0
    planned_seen = False
    actual_total = 0.0
    actual_seen = False
    sched_fifo = False
    for idx, rep in enumerate(reports):
        events.extend(rep.get("events") or [])
        for item in (rep.get("evidence") or []):
            evidence.append("leg%d: %s" % (idx + 1, item))
        m = rep.get("metrics") or {}
        ticks += int(m.get("ticks") or 0)
        hold_ticks += int(m.get("hold_ticks") or 0)
        if m.get("planned_duration_s") is not None:
            planned_total += float(m["planned_duration_s"])
            planned_seen = True
        if m.get("duration_s") is not None:
            actual_total += float(m["duration_s"])
            actual_seen = True
        sched_fifo = sched_fifo or bool(m.get("sched_fifo"))

    last_metrics = (reports[-1].get("metrics") or {})
    jitter = metrics_mod.jitter_stats(jitter_samples)
    follow = metrics_mod.follow_error_stats(follow_errors)
    merged_metrics = {
        "ticks": ticks,
        "duration_s": actual_total if actual_seen else None,
        "rt_factor": metrics_mod.rt_factor(
            planned_total if planned_seen else None,
            actual_total if actual_seen else None,
        ),
        "tick_jitter_p50_ms": jitter["p50_ms"],
        "tick_jitter_p95_ms": jitter["p95_ms"],
        "tick_jitter_p99_ms": jitter["p99_ms"],
        "tick_jitter_max_ms": jitter["max_ms"],
        "follow_err_rms_deg": follow["rms_deg"],
        "follow_err_max_deg": follow["max_deg"],
        "arrival_err_deg": last_metrics.get("arrival_err_deg"),
        "arrival_hold_ms": last_metrics.get("arrival_hold_ms"),
        "sched_fifo": sched_fifo,
        "pre_position_hold_ms": last_metrics.get("pre_position_hold_ms"),
        "hold_ticks": hold_ticks,
        "planned_duration_s": planned_total if planned_seen else None,
    }
    merged_evidence = evidence + ["legs: %d" % len(reports)]
    return report_mod.build_report(
        status=worst.get("status"),
        reason=worst.get("reason") or "",
        next_action=worst.get("next_action") or "",
        evidence=merged_evidence,
        mode=worst.get("mode") or "fake",
        metrics=merged_metrics,
        events=events,
    )


def _failure_report(status: str, reason: str, next_action: str,
                    evidence: Optional[List[str]], mode: str,
                    events: Optional[List[Dict[str, Any]]] = None) -> Dict[str, Any]:
    """构造失败报告（**不伪造任何数值**：指标全空）。"""
    return report_mod.build_report(
        status=status,
        reason=reason,
        next_action=next_action,
        evidence=evidence or [],
        mode=mode,
        events=events or [],
    )


def _unreachable_report(reason: str, next_action: str,
                        evidence: Optional[List[str]]) -> Dict[str, Any]:
    """控制器不可达 → error 报告（退出码 4，**零伪造数值**）。"""
    event = {
        "type": "controller_unreachable",
        "severity": "FATAL",
        "t": 0.0,
        "reason": reason,
        "evidence": list(evidence or []),
        "next_action": next_action,
    }
    return _failure_report(contract.STATUS_ERROR, reason, next_action,
                           evidence, "live", [event])


# --------------------------------------------------------------- 执行（核心）
def execute(mode: str = "fake",
            trajectory_path: str = "",
            allow_unverified: bool = False,
            duration_s: Optional[float] = None,
            realtime: bool = False,
            arm: str = "left",
            ip: Optional[str] = None,
            port: Optional[int] = None,
            udp_port: Optional[int] = None,
            ws: Optional[str] = None,
            injections: Optional[List[str]] = None,
            gate: Any = None,
            motion_gate: Any = _UNSET,
            motion_gate_note: Optional[str] = None) -> Dict[str, Any]:
    """跑一次执行，返回**统一报告**（fake / live 同一 schema）。

    本函数**不抛裸异常**：任何失败都变成 ``status=blocked|error`` 的报告
    （带 reason + 可粘贴 next_action）。

    ``gate``：安全闸门（进程外强制停机通道，鸭子类型）。给了之后执行器
    会在每 tick 查询 ``gate.stop_requested``，并在闸门请求停机时**立即**
    执行 L4 急停（详见 ``rm_executor.executor.Executor``）。

    ``motion_gate``：夹爪运动闸门（鸭子类型：``check()`` + ``state``）。
    缺省（``_UNSET``）→ 自动构造进程内**共享**的 ``MotionGate``
    （见 :func:`_make_motion_gate`）；显式传 ``None`` → **不启用互锁**。
    ``motion_gate_note``：调用方自带的互锁说明（写进报告 evidence）；
    缺省 → 由本函数按实际状态生成。
    """
    mode = "live" if str(mode) == "live" else "fake"

    if not trajectory_path:
        return _failure_report(
            contract.STATUS_BLOCKED, "未提供轨迹路径（--traj 必填）",
            "用 `python3 -m rm_executor.cli --traj <path> ...` 指定轨迹文件",
            [], mode)

    if mode == "live" and allow_unverified:
        return _failure_report(
            contract.STATUS_BLOCKED,
            "真机路径不接受未校验轨迹（--allow-unverified 仅限离线假执行）",
            "先用 `python3 -m rmctl.cli plan ... --out <traj.json>` 生成带 verify "
            "报告的轨迹，再在真机路径执行",
            ["--allow-unverified 在 live 模式被拒绝"], mode)

    # -- 故障注入解析（只在离线 fake 开放）
    injection_specs: List[Dict[str, Any]] = []
    for spec in (injections or []):
        try:
            injection_specs.append(injection_mod.parse_injection(spec))
        except ValueError as exc:
            return _failure_report(
                contract.STATUS_BLOCKED,
                "故障注入规格非法：%s" % exc,
                "用法：--inject follow_error:<deg> | --inject follow_error_light:<deg> "
                "| --inject feedback_stall:<ms> | --inject controller_conflict "
                "| --inject limit_exceeded:<deg> | --inject collision:<code> "
                "| --inject protection_stop",
                ["注入规格: %r" % spec], mode)
    if injection_specs and mode == "live":
        return _failure_report(
            contract.STATUS_BLOCKED,
            "真机路径不接受故障注入（--inject 仅限离线 fake）",
            "去掉 --inject 后重跑；故障注入只能用于离线复跑异常判定路径",
            ["--inject 在 live 模式被拒绝"], mode)

    config, _config_error = _load_config(ws)

    # -- 轨迹时长（用于 --duration 的长跑轮数）；加载失败时**交给执行器自己
    #    报错**（唯一真相：失败报告由 Executor 生成）
    traj_duration_s: Optional[float] = None
    loaded = contract.load_trajectory(trajectory_path,
                                      allow_unverified=allow_unverified)
    if loaded.get("status") == contract.STATUS_OK:
        times = loaded["trajectory"].get("times") or []
        if len(times) >= 2:
            traj_duration_s = float(times[-1]) - float(times[0])

    legs = _leg_count(duration_s, traj_duration_s) if mode == "fake" else 1

    # -- 夹爪运动闸门（共享实例；不可用 → 不启用互锁 + 证据）-----------
    if motion_gate is _UNSET:
        motion_gate, interlock_note = _make_motion_gate(ws)
    elif motion_gate is None:
        interlock_note = "%s（调用方显式关闭）" % INTERLOCK_DISABLED_NOTE
    else:
        interlock_note = ("%s（调用方注入的闸门，阶段 B / 阶段 C 前各查一次）"
                          % INTERLOCK_ENABLED_NOTE)
    if motion_gate_note:
        interlock_note = motion_gate_note

    try:
        if mode == "live":
            report = _run_live(trajectory_path, config, arm, ip, port, udp_port,
                               ws, duration_s, gate, motion_gate)
        else:
            report = _run_fake(trajectory_path, config, ws, allow_unverified,
                               realtime, legs, injection_specs, gate, motion_gate)
    except Exception as exc:  # noqa: BLE001 - 兜底：异常 → error 报告
        report = _failure_report(
            contract.STATUS_ERROR,
            "执行器异常终止（%s）: %s" % (type(exc).__name__, exc),
            "查看报告 evidence 与终端栈信息后重跑",
            [repr(exc)], mode)
    report.setdefault("evidence", []).append(interlock_note)
    return report


def _run_fake(trajectory_path: str, config: Optional[ExecutorConfig],
              ws: Optional[str], allow_unverified: bool, realtime: bool,
              legs: int,
              injections: Optional[List[Dict[str, Any]]] = None,
              gate: Any = None,
              motion_gate: Any = None) -> Dict[str, Any]:
    """离线假执行（零真机依赖）；长跑 = 重复执行 ``legs`` 轮后合并报告。"""
    reports: List[Dict[str, Any]] = []
    jitter_samples: List[float] = []
    follow_errors: List[float] = []
    for leg in range(max(1, int(legs))):
        # 调用记录**有界**（长跑 600 s @100 Hz ≈ 6 万次下发）：计数不受影响
        transport = FakeTransport(max_records=FAKE_CALL_RECORD_LIMIT)
        feedback = _mirror_feedback(transport, DEFAULT_PERIOD_S)
        if injections:
            transport, feedback = injection_mod.apply_injections(
                transport, feedback, injections, period_s=DEFAULT_PERIOD_S)
        clock = (TickClock(period_s=DEFAULT_PERIOD_S) if realtime
                 else ManualClock(period_s=DEFAULT_PERIOD_S))
        # config=None → 执行器自己按 ws 读工位配置（读不到就记 NEEDS_PARAM 事件）
        executor = Executor(transport, feedback, config=config, clock=clock,
                            mode="fake", ws=ws, gate=gate,
                            motion_gate=motion_gate)
        rep = executor.run(trajectory_path, allow_unverified=allow_unverified)
        reports.append(rep)
        jitter_samples.extend(executor.jitter_samples_ms)
        follow_errors.extend(executor.follow_errors_deg)
        if rep.get("status") != contract.STATUS_OK:
            break  # 失败即停（不继续刷后续轮次）
    merged = _merge_reports(reports, jitter_samples, follow_errors)
    # 诚实分层：离线代理的抖动/跟随误差是**代理值**，写进 evidence 以免被误读
    merged.setdefault("evidence", []).append(
        "mode=fake：抖动与跟随误差来自虚拟时钟 + 理想跟随模型（代理值），"
        "**不代表真机结论**")
    for line in injection_mod.describe(injections or []):
        merged.setdefault("evidence", []).append(line)
    return merged


def _run_live(trajectory_path: str, config: Optional[ExecutorConfig],
              arm: str, ip: Optional[str], port: Optional[int],
              udp_port: Optional[int], ws: Optional[str],
              duration_s: Optional[float], gate: Any = None,
              motion_gate: Any = None) -> Dict[str, Any]:
    """真机执行（**与 fake 同一 Executor、同一报告 schema**）。"""
    try:
        from rm_hw_bridge import bridge as hw_bridge
    except Exception as exc:  # noqa: BLE001 - 硬件层不可用 → 诚实降级
        return _unreachable_report(
            "控制器不可达：硬件适配层 rm_hw_bridge 不可用（%s）: %s"
            % (type(exc).__name__, exc),
            "export PYTHONPATH=$PWD/dualarm_ws/src/rm_hw_bridge:$PYTHONPATH  "
            "# 让执行层看到硬件适配层",
            [repr(exc)])

    if not ip:
        return _unreachable_report(
            "控制器不可达：未提供控制器 ip（--ip 必填）",
            "加 --ip <控制器IP> --port <TCP端口> 重跑"
            "（ip 也可从 workcell/baselines/network_plan.yaml 取）",
            ["未提供 --ip"])

    tcp_port = int(port or getattr(hw_bridge, "DEFAULT_TCP_PORT", 8080))
    try:
        link = hw_bridge.connect(ip, tcp_port,
                                 udp_port=int(udp_port) if udp_port else None,
                                 arm=arm)
    except Exception as exc:  # noqa: BLE001 - 建链异常 → 诚实降级
        return _unreachable_report(
            "控制器不可达：建立连接异常（%s）: %s" % (type(exc).__name__, exc),
            "确认控制器已上电、网络可达（ip=%s port=%s）后重跑" % (ip, tcp_port),
            [repr(exc)])

    if not getattr(link, "connected", False):
        reason = getattr(link, "reason", "") or "ArmLink 未连接"
        next_action = getattr(link, "next_action", "") or (
            "确认控制器已上电、网络可达（ip=%s port=%s）后重跑" % (ip, tcp_port))
        evidence = list(getattr(link, "evidence", []) or [])
        evidence.append("ArmLink.connected = False")
        return _unreachable_report("控制器不可达：%s" % reason,
                                   next_action, evidence)

    from rm_executor.feedback import make_live_feedback
    from rm_executor.transport import BridgeTransport

    transport = BridgeTransport(link=link)
    feedback = make_live_feedback(period_s=LIVE_FEEDBACK_PERIOD_S)
    clock = TickClock(period_s=DEFAULT_PERIOD_S)
    try:
        # config=None → 执行器自己按 ws 读工位配置（读不到就记 NEEDS_PARAM 事件）
        executor = Executor(transport, feedback, config=config, clock=clock,
                            mode="live", ws=ws, gate=gate,
                            motion_gate=motion_gate)
        rep = executor.run(trajectory_path, allow_unverified=False)
    finally:
        try:
            link.close()
        except Exception:  # pragma: no cover - 释放失败不影响报告
            pass
    if duration_s and duration_s > 0:
        # 真机按轨迹实际时长执行；长跑请在真机窗口重复执行（诚实：不假装跑过）
        rep.setdefault("evidence", []).append(
            "live 模式忽略 --duration=%.3f（真机按轨迹实际时长执行；"
            "长跑请在真机窗口重复执行本命令）" % float(duration_s))
    return rep


# ------------------------------------------------------------------- CLI
def build_parser() -> argparse.ArgumentParser:
    """构造参数解析器。"""
    parser = argparse.ArgumentParser(
        prog="rm_executor.cli",
        description="10 ms 单时钟执行器（离线 fake 与真机 live 同一报告 schema）",
    )
    parser.add_argument("--traj", required=True, help="轨迹 JSON 路径")
    parser.add_argument("--mode", choices=["fake", "live"], default="fake",
                        help="fake=离线代理（默认）；live=真机")
    parser.add_argument("--report", default=None, help="统一报告 JSON 落盘路径")
    parser.add_argument("--duration", type=float, default=None,
                        help="离线长跑：等效运行时长（秒），默认取轨迹时长")
    parser.add_argument("--realtime", action="store_true",
                        help="用真实墙钟（TickClock）而不是确定性 ManualClock")
    parser.add_argument("--allow-unverified", dest="allow_unverified",
                        action="store_true",
                        help="放行缺 verify 的轨迹（**仅限离线假执行**）")
    parser.add_argument("--arm", choices=["left", "right"], default="left",
                        help="选哪条臂（live 模式）")
    parser.add_argument("--ip", default=None, help="控制器 IP（live 模式必填）")
    parser.add_argument("--port", type=int, default=None, help="控制器 TCP 端口")
    parser.add_argument("--udp-port", dest="udp_port", type=int, default=None,
                        help="控制器 UDP 主动上报端口")
    parser.add_argument("--ws", default=None, help="工作区根（默认自动发现或 $RM_WS）")
    parser.add_argument("--inject", action="append", default=None, metavar="SPEC",
                        help="离线故障注入（可重复，仅 fake）："
                             "follow_error:<deg> / feedback_stall:<ms> / "
                             "controller_conflict / follow_error_light:<deg> / "
                             "limit_exceeded:<deg> / collision:<code> / "
                             "protection_stop")
    return parser


def _print_summary(rep: Dict[str, Any]) -> None:
    """打印人读摘要（机器读请用 --report 或 stdout 的 JSON）。"""
    metrics = rep.get("metrics") or {}
    print("status      : %s" % rep.get("status"))
    print("mode        : %s" % rep.get("mode"))
    if rep.get("reason"):
        print("reason      : %s" % rep["reason"])
    if rep.get("next_action"):
        print("next_action : %s" % rep["next_action"])
    print("ticks       : %s" % metrics.get("ticks"))
    print("jitter p99  : %s ms" % metrics.get("tick_jitter_p99_ms"))
    print("follow rms  : %s deg" % metrics.get("follow_err_rms_deg"))
    print("arrival err : %s deg" % metrics.get("arrival_err_deg"))
    print("events      : %d" % len(rep.get("events") or []))


def main(argv: Optional[List[str]] = None) -> int:
    """CLI 主入口；返回进程退出码（0 / 3 / 4）。"""
    args = build_parser().parse_args(argv)
    rep = execute(mode=args.mode,
                  trajectory_path=args.traj,
                  allow_unverified=args.allow_unverified,
                  duration_s=args.duration,
                  realtime=args.realtime,
                  arm=args.arm,
                  ip=args.ip,
                  port=args.port,
                  udp_port=args.udp_port,
                  ws=args.ws,
                  injections=list(args.inject or []))
    rc = contract.EXIT_CODES.get(rep.get("status"), 4)
    if args.report:
        try:
            path = report_mod.dump_report(rep, args.report)
            print("report      : %s" % path)
        except OSError as exc:
            print("报告落盘失败: %s" % exc, file=sys.stderr)
            rc = contract.EXIT_CODES[contract.STATUS_ERROR]
    else:
        print(json.dumps(rep, ensure_ascii=False, indent=2))
    _print_summary(rep)
    return rc


if __name__ == "__main__":
    sys.exit(main())
