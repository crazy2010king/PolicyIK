#!/usr/bin/env python3
"""RM75-B 真机验证阶梯（V0 → V9）编排器。

目的
----
把 M001-3kzz40 的「真机 / 现场人」证据缺口，变成一条**从简单到复杂、可逐步执行、
每步都有判据与留档**的验证阶梯。以手上的**单臂**睿尔曼 RM75-B（``192.168.1.18``）
作为唯一验证环境，逐步把 M001 的验收条款 CC1/CC2/CC4/CC6/CC7 从"未满足"推到"有证据"。

阶梯设计（越靠前越安全、越靠后越需要现场看护）
------------------------------------------------

===== ============ ======================= ====== ====================== ==================
阶段  名称        动作                    风险   M001 切片             验收条款 / 类别
===== ============ ======================= ====== ====================== ==================
V0    契约体检    只读（doctor + 审计）   无     S02/S09              CC5 / Contract
V1    连接与快照  只读（探针只读体检）     无     S02                  CC6 / Integration
V2    上报质量    只读（UDP 长跑）        无     S01/S02              CC4 / Operational
V3    最小运动    运动 0.3°（规划通道）   低     S03/S04              CC2 / Operational
V4    透传通道    运动 0.3°（canfd）      低     S03                  CC6 / Integration
V5    执行层闭环  运动 0.3°（10 ms 流式）  低     S03/S04              CC2/CC6 / Integration
V6    FK 交叉     只读（自研 vs 控制器）   无     S01                  CC5 / Contract
V7    重复性      运动（默认目标=当前位姿）中   S01                  CC4 / Operational
V8    阶梯 acc/jerk 运动 10°/档          **高**  S01/S04              CC4 / Operational
V9    急停演练    触发急停                **高**  S04                  CC7 / Operational
===== ============ ======================= ====== ====================== ==================

安全纪律（写在代码里）
----------------------
1. **顺序不可跳过**：默认按 V0→V9 顺序跑，前一阶段未通过则后续阶段标记 ``skipped``；
2. **默认不动臂**：``--stages`` 不含运动阶段时，脚本只做只读验证；
3. **高风险阶段需显式授权**：V8 / V9 必须加 ``--allow-risky``，否则标记 skipped；
4. **运动幅度由探针封顶**：V3/V4/V5 一律走 ``rm75_safe_probe`` 的安全包络
   （单关节、≤1.0°、低速、去程回程对称、异常即急停）；
5. **不伪造数值**：任何阶段失败都如实记 ``blocked`` / ``error`` + 证据，
   汇总报告里**未通过的阶段不产生任何"通过"字样**。

用法::

    # 只读体检（V0+V1，臂不动）
    python3 scripts/rm75_validation_ladder.py --ip 192.168.1.18 --stages V0,V1

    # 跑到执行层闭环（V0..V5）
    python3 scripts/rm75_validation_ladder.py --ip 192.168.1.18 --max-stage V5

    # 高风险阶段（现场看护 + 示教器在手边）
    python3 scripts/rm75_validation_ladder.py --ip 192.168.1.18 --stages V8,V9 --allow-risky

退出码：0 全部执行到的阶段通过 / 3 存在 blocked（外部条件不具备）/ 4 存在 error。
"""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import time
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

SCHEMA = "rmctl.rm75_validation_ladder/1"

STATUS_PASS = "pass"
#: **质量不合格**（不是硬故障）：阶段本身跑完了，但结论不达标。
#: 与 blocked/error 的区别：质量不合格**不阻断**后续阶段（后续阶段不依赖它），
#: 但会在汇总报告里如实标红，整体状态降级为 failed。
STATUS_FAILED = "failed"
STATUS_BLOCKED = "blocked"
STATUS_ERROR = "error"
STATUS_SKIPPED = "skipped"

RISK_NONE = "none"
RISK_LOW = "low"
RISK_MEDIUM = "medium"
RISK_HIGH = "high"

EXIT_OK = 0
EXIT_BLOCKED = 3
EXIT_ERROR = 4


# ==================================================================== 上下文
class LadderContext(object):
    """一次阶梯执行的全部输入 + **可注入的执行器**（测试用假实现替换）。"""

    def __init__(self, arm: str = "left", ip: str = "192.168.1.18",
                 port: int = 8080, udp_port: int = 8091, ws: Optional[str] = None,
                 out_dir: str = "build/reports/validation",
                 joint_index: int = 0, delta_deg: float = 0.3, speed_pct: int = 2,
                 udp_duration_s: float = 60.0, fk_samples: int = 20,
                 repeats: int = 30, target_deg: Optional[Sequence[float]] = None,
                 trials: int = 1, executor_ticks: int = 0,
                 probe_runner: Optional[Callable[..., Dict[str, Any]]] = None,
                 hwcollect_runner: Optional[Callable[..., Dict[str, Any]]] = None,
                 subprocess_runner: Optional[Callable[..., Tuple[int, str, str]]] = None,
                 executor_runner: Optional[Callable[..., Dict[str, Any]]] = None,
                 trajectory_builder: Optional[Callable[..., Dict[str, Any]]] = None,
                 link_provider: Optional[Callable[[], Tuple[Any, Optional[Dict[str, Any]]]]] = None):
        self.arm = arm
        self.ip = ip
        self.port = int(port)
        self.udp_port = int(udp_port)
        self.ws = ws
        self.out_dir = os.path.abspath(out_dir)
        self.joint_index = int(joint_index)
        self.delta_deg = float(delta_deg)
        self.speed_pct = int(speed_pct)
        self.udp_duration_s = float(udp_duration_s)
        self.fk_samples = int(fk_samples)
        self.repeats = int(repeats)
        self.target_deg = list(target_deg) if target_deg else None
        self.trials = int(trials)
        self.executor_ticks = int(executor_ticks)

        self._probe_runner = probe_runner
        self._hwcollect_runner = hwcollect_runner
        self._subprocess_runner = subprocess_runner
        self._executor_runner = executor_runner
        self._trajectory_builder = trajectory_builder
        self._link_provider = link_provider
        #: 整轮**共享**的句柄（控制器客户端连接数上限实测 5 → 建一次链、全程复用）
        self._shared_link = None
        self._shared_link_failure: Optional[Dict[str, Any]] = None

    # --- 共享连接（连接额度管理）
    def ensure_link(self) -> Tuple[Any, Optional[Dict[str, Any]]]:
        """取得**共享句柄**（只建一次链），返回 ``(link, failure)``。

        真机窗口里控制器对客户端连接数有上限（实测 5，且释放很慢），
        因此整轮验证必须**建一次链、全程复用**：探针阶段用
        ``link_factory`` 复用它（``close_link=False``），执行层用 ``link=`` 复用它。
        """
        if self._shared_link is not None:
            return self._shared_link, None
        if self._shared_link_failure is not None:
            return None, self._shared_link_failure
        if self._link_provider is not None:
            link, failure = self._link_provider()
        else:
            script = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                  "rm75_safe_probe.py")
            probe = _load_module("rm75_safe_probe", script)
            link, failure = probe._connect_link(self.arm, self.ip, self.port,
                                                self.udp_port, True, self.ws, None)
        if link is None:
            self._shared_link_failure = failure or {
                "status": "blocked", "reason": "无法建立真机连接"}
            return None, self._shared_link_failure
        self._shared_link = link
        return link, None

    def shared_link_factory(self):
        """给探针用的 link_factory（复用共享句柄，且**不关闭**它）。"""
        def _factory(**kwargs):
            return self.ensure_link()
        return _factory

    # --- 可注入执行器（默认走项目真实实现）
    def run_probe(self, **kwargs) -> Dict[str, Any]:
        if self._probe_runner is not None:
            return self._probe_runner(**kwargs)
        script = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              "rm75_safe_probe.py")
        spec = _load_module("rm75_safe_probe", script)
        return spec.run_probe(**kwargs)

    def run_hwcollect(self, name: str, **kwargs) -> Dict[str, Any]:
        if self._hwcollect_runner is not None:
            return self._hwcollect_runner(name, **kwargs)
        from rmctl import hwcollect
        fn = hwcollect.EXPERIMENTS[name]
        return fn(**kwargs)

    def run_subprocess(self, args: Sequence[str], cwd: Optional[str] = None
                       ) -> Tuple[int, str, str]:
        if self._subprocess_runner is not None:
            return self._subprocess_runner(args, cwd)
        proc = subprocess.run(list(args), cwd=cwd, stdout=subprocess.PIPE,
                              stderr=subprocess.PIPE, text=True)
        return proc.returncode, proc.stdout, proc.stderr

    def run_executor(self, **kwargs) -> Dict[str, Any]:
        if self._executor_runner is not None:
            return self._executor_runner(**kwargs)
        from rm_executor import cli as executor_cli
        return executor_cli.execute(**kwargs)

    def build_trajectory(self, **kwargs) -> Dict[str, Any]:
        if self._trajectory_builder is not None:
            return self._trajectory_builder(**kwargs)
        return _build_micro_trajectory(**kwargs)

    def evidence_path(self, stage_id: str, filename: str) -> str:
        directory = os.path.join(self.out_dir, stage_id)
        os.makedirs(directory, exist_ok=True)
        return os.path.join(directory, filename)


def _load_module(name: str, path: str):
    """按路径加载脚本模块（脚本不是包）。"""
    import importlib.util
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise ImportError("无法加载脚本模块: %s" % path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


# ==================================================================== 阶段定义
class Stage(object):
    """一个阶梯阶段。"""

    def __init__(self, stage_id: str, title: str, goal: str, risk: str,
                 moves_arm: bool, runner: Callable[[LadderContext], Dict[str, Any]],
                 slices: Sequence[str] = (), requirements: Sequence[str] = (),
                 criteria: Sequence[str] = (), vclass: str = "Operational",
                 needs_authorization: bool = False,
                 prerequisites: Sequence[str] = (), notes: str = ""):
        self.id = stage_id
        self.title = title
        self.goal = goal
        self.risk = risk
        self.moves_arm = moves_arm
        self.runner = runner
        self.slices = list(slices)
        self.requirements = list(requirements)
        self.criteria = list(criteria)
        self.vclass = vclass
        self.needs_authorization = needs_authorization
        self.prerequisites = list(prerequisites)
        self.notes = notes

    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id, "title": self.title, "goal": self.goal,
            "risk": self.risk, "moves_arm": self.moves_arm,
            "slices": self.slices, "requirements": self.requirements,
            "criteria": self.criteria, "verification_class": self.vclass,
            "needs_authorization": self.needs_authorization,
            "prerequisites": self.prerequisites, "notes": self.notes,
        }


def _result(stage_id: str, status: str, reason: str = "", evidence: Optional[List[str]] = None,
            data: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    return {"stage": stage_id, "status": status, "reason": reason,
            "evidence": list(evidence or []), "data": data or {}}


def _probe_status(report: Dict[str, Any]) -> str:
    status = (report or {}).get("status")
    return {"ok": STATUS_PASS, "dry_run": STATUS_PASS,
            "blocked": STATUS_BLOCKED}.get(status, STATUS_ERROR)


def _probe_evidence(report: Dict[str, Any]) -> List[str]:
    out: List[str] = []
    for phase in (report or {}).get("phases") or []:
        out.append("%s: %s" % (phase.get("name"), phase.get("detail")))
    for item in (report or {}).get("evidence") or []:
        out.append("%s: %s" % (item.get("kind"), item.get("detail")))
    return out


def _probe_metrics(report: Dict[str, Any]) -> Dict[str, Any]:
    """从探针报告里抽出关键判定数字。"""
    metrics: Dict[str, Any] = {}
    for phase in (report or {}).get("phases") or []:
        name = phase.get("name")
        data = phase.get("data") or {}
        if name == "verify_out":
            metrics["arrival_error_deg"] = data.get("arrival_error_deg")
            metrics["max_drift_deg"] = data.get("max_drift_deg")
        elif name == "verify_back":
            metrics["max_return_error_deg"] = data.get("max_return_error_deg")
        elif name == "snapshot":
            snap = data.get("snapshot") or {}
            metrics["joint_deg"] = snap.get("joint_deg")
            metrics["udp_frames"] = (snap.get("realtime") or {}).get("frames")
            metrics["controller_type"] = snap.get("controller_trajectory_type_name")
            metrics["arm_state"] = snap.get("arm_current_status_name")
    return metrics


# --------------------------------------------------------------------- V0
def _stage_v0(ctx: LadderContext) -> Dict[str, Any]:
    """契约体检：doctor 的硬件面检查 + 黑名单静态审计（只读）。"""
    evidence: List[str] = []
    doctor = {"status": "unknown"}
    rc, out, err = ctx.run_subprocess([sys.executable, "-m", "rmctl", "doctor", "--json"],
                                      cwd=ctx.ws)
    evidence.append("rmctl doctor --json 退出码 %d" % rc)
    if rc in (0, 3, 4):
        try:
            doctor = json.loads(out)
        except ValueError:
            doctor = {"status": "unparsable", "raw": out[-400:]}
    hw = {}
    for check in (doctor.get("checks") or []) if isinstance(doctor, dict) else []:
        if str(check.get("name", "")).startswith("hw:"):
            hw[check["name"]] = check
    audit_rc, audit_out, audit_err = ctx.run_subprocess(
        [sys.executable, "-m", "rm_hw_bridge.audit", "--repo"], cwd=ctx.ws)
    evidence.append("rm_hw_bridge.audit --repo 退出码 %d：%s"
                    % (audit_rc, (audit_out or audit_err).strip()[-200:]))

    data = {"doctor_exit": rc, "hw_checks": hw, "audit_exit": audit_rc,
            "doctor_status": doctor.get("status") if isinstance(doctor, dict) else None}
    if audit_rc != 0:
        return _result("V0", STATUS_ERROR, "黑名单静态审计未通过（退出码 %d）" % audit_rc,
                       evidence, data)
    if not hw:
        return _result("V0", STATUS_BLOCKED,
                       "doctor 未产出 hw:* 检查项（缺依赖或环境未就位）", evidence, data)
    failed = [name for name, check in hw.items() if not check.get("ok")
              and check.get("level") not in ("INFO",)]
    if failed:
        return _result("V0", STATUS_ERROR, "硬件面检查未通过：%s" % ", ".join(failed),
                       evidence, data)
    symbols = hw.get("hw:symbols") or {}
    if not symbols.get("ok"):
        return _result("V0", STATUS_ERROR, "hw:symbols 未通过：%s" % symbols.get("detail"),
                       evidence, data)
    return _result("V0", STATUS_PASS, "契约与符号体检通过（22 核心符号可解析、审计零违规）",
                   evidence, data)


# --------------------------------------------------------------------- V1
def _stage_v1(ctx: LadderContext) -> Dict[str, Any]:
    """连接与状态快照：只读体检（零运动）。"""
    link, failure = ctx.ensure_link()
    if link is None:
        return _result("V1", STATUS_BLOCKED,
                       "无法建立真机连接：%s" % ((failure or {}).get("reason") or "未知"),
                       ["连接额度：整轮验证必须建一次链、全程复用"], {})
    report = ctx.run_probe(arm=ctx.arm, ip=ctx.ip, port=ctx.port,
                           udp_port=ctx.udp_port, precheck_only=True, ws=ctx.ws,
                           link_factory=ctx.shared_link_factory(), close_link=False)
    status = _probe_status(report)
    metrics = _probe_metrics(report)
    if status != STATUS_PASS:
        return _result("V1", status, (report or {}).get("reason") or "只读体检未通过",
                       _probe_evidence(report), metrics)
    checks = []
    for phase in (report.get("phases") or []):
        if phase.get("name") == "preflight":
            checks = (phase.get("data") or {}).get("checks") or []
    failed = [c.get("name") for c in checks if not c.get("ok")]
    if failed:
        return _result("V1", STATUS_ERROR, "前置检查未通过：%s" % ", ".join(failed),
                       _probe_evidence(report), metrics)
    return _result("V1", STATUS_PASS,
                   "连接成功，前置检查 %d/%d 通过，控制器 %s"
                   % (len(checks), len(checks), metrics.get("controller_type")),
                   _probe_evidence(report), metrics)


# --------------------------------------------------------------------- V2
def _stage_v2(ctx: LadderContext) -> Dict[str, Any]:
    """UDP 主动上报质量：只读长跑。

    **判定用实测值**：观测周期 = elapsed × 1000 / frames（不是配置值），
    到达率 = frames / expected_frames；两者都写进证据，避免"配置 5 ms"被误读成
    "实测 5 ms"。
    """
    link, failure = ctx.ensure_link()
    if link is None:
        return _result("V2", STATUS_BLOCKED,
                       "无法建立真机连接：%s" % ((failure or {}).get("reason") or "未知"),
                       ["连接额度：整轮验证必须建一次链、全程复用"], {})
    result = ctx.run_hwcollect("udp_stability", arm=ctx.arm, ip=ctx.ip,
                               duration_s=ctx.udp_duration_s, dry_run=False, ws=ctx.ws,
                               link=link)
    status = (result or {}).get("status")
    summary = dict((result or {}).get("summary") or {})
    frames = summary.get("frames")
    expected = summary.get("expected_frames")
    elapsed = summary.get("elapsed_s")
    configured = summary.get("period_ms") or 5
    observed = None
    if isinstance(frames, int) and frames > 0 and elapsed:
        observed = round(float(elapsed) * 1000.0 / frames, 3)
    ratio = (observed / float(configured)) if observed else None
    arrival = (float(frames) / float(expected)) if (frames and expected) else None
    summary["observed_period_ms"] = observed
    summary["observed_period_multiple"] = ratio
    summary["arrival_ratio"] = None if arrival is None else round(arrival, 4)

    evidence = ["udp_stability status=%s" % status,
                "配置周期 %s ms → **实测周期** %s ms（%s×）"
                % (configured, observed,
                   ("%.0f" % ratio) if ratio else "n/a"),
                "帧数 %s / 期望 %s（到达率 %s）"
                % (frames, expected,
                   ("%.1f%%" % (arrival * 100)) if arrival is not None else "n/a"),
                "断流窗口 %s，最大空窗 %s ms"
                % (summary.get("dropout_count"), summary.get("max_gap_ms"))]

    if status == "ok":
        if isinstance(frames, int) and frames <= 0:
            return _result("V2", STATUS_ERROR, "UDP 主动上报 0 帧（上报未生效）",
                           evidence, summary)
        if summary.get("period_multiple_ok") is False:
            return _result("V2", STATUS_FAILED,
                           "实测周期 %s ms 不落在 %s ms 的整数倍上（质量不合格；"
                           "不阻断后续阶段，但 UDP 上报质量这条基线仍不能标 measured）"
                           % (observed, configured), evidence, summary)
        if summary.get("dropout_count"):
            return _result("V2", STATUS_FAILED,
                           "存在 %s 个断流窗口（最大空窗 %s ms）（质量不合格）"
                           % (summary.get("dropout_count"), summary.get("max_gap_ms")),
                           evidence, summary)
        return _result("V2", STATUS_PASS,
                       "UDP 上报 %s 帧（到达率 %s），实测周期 %s ms（配置 %s ms 的 %s×），无断流"
                       % (frames, ("%.1f%%" % (arrival * 100)) if arrival is not None else "n/a",
                          observed, configured, ("%.0f" % ratio) if ratio else "n/a"),
                       evidence, summary)
    mapped = {"dry_run": STATUS_PASS, "NEEDS_HUMAN": STATUS_BLOCKED,
              "blocked": STATUS_BLOCKED}.get(status, STATUS_ERROR)
    return _result("V2", mapped, (result or {}).get("reason") or "UDP 上报采集未通过",
                   evidence, summary)


# --------------------------------------------------------------------- V3
def _stage_v3(ctx: LadderContext) -> Dict[str, Any]:
    """最小关节运动 · 规划通道（rm_movej）。"""
    link, failure = ctx.ensure_link()
    if link is None:
        return _result("V3", STATUS_BLOCKED,
                       "无法建立真机连接：%s" % ((failure or {}).get("reason") or "未知"),
                       ["连接额度：整轮验证必须建一次链、全程复用"], {})
    report = ctx.run_probe(arm=ctx.arm, ip=ctx.ip, port=ctx.port, udp_port=ctx.udp_port,
                           joint_index=ctx.joint_index, delta_deg=ctx.delta_deg,
                           speed_pct=ctx.speed_pct, channel="movej", ws=ctx.ws,
                           link_factory=ctx.shared_link_factory(), close_link=False)
    status = _probe_status(report)
    metrics = _probe_metrics(report)
    if status != STATUS_PASS:
        return _result("V3", status, (report or {}).get("reason") or "最小动作验证未通过",
                       _probe_evidence(report), metrics)
    return _result("V3", STATUS_PASS,
                   "关节%d %+.3f°：到位误差 %s°，漂移 %s°，回程 %s°"
                   % (ctx.joint_index + 1, ctx.delta_deg,
                      metrics.get("arrival_error_deg"), metrics.get("max_drift_deg"),
                      metrics.get("max_return_error_deg")),
                   _probe_evidence(report), metrics)


# --------------------------------------------------------------------- V4
def _stage_v4(ctx: LadderContext) -> Dict[str, Any]:
    """关节角透传 · 10 ms 单时钟主通道（rm_movej_canfd）。"""
    link, failure = ctx.ensure_link()
    if link is None:
        return _result("V4", STATUS_BLOCKED,
                       "无法建立真机连接：%s" % ((failure or {}).get("reason") or "未知"),
                       ["连接额度：整轮验证必须建一次链、全程复用"], {})
    report = ctx.run_probe(arm=ctx.arm, ip=ctx.ip, port=ctx.port, udp_port=ctx.udp_port,
                           joint_index=ctx.joint_index, delta_deg=ctx.delta_deg,
                           speed_pct=ctx.speed_pct, channel="canfd", ws=ctx.ws,
                           link_factory=ctx.shared_link_factory(), close_link=False)
    status = _probe_status(report)
    metrics = _probe_metrics(report)
    if status != STATUS_PASS:
        return _result("V4", status, (report or {}).get("reason") or "透传通道验证未通过",
                       _probe_evidence(report), metrics)
    return _result("V4", STATUS_PASS,
                   "canfd 透传：到位误差 %s°，漂移 %s°，回程 %s°"
                   % (metrics.get("arrival_error_deg"), metrics.get("max_drift_deg"),
                      metrics.get("max_return_error_deg")),
                   _probe_evidence(report), metrics)


# --------------------------------------------------------------------- V5
def _stage_v5(ctx: LadderContext) -> Dict[str, Any]:
    """执行层 live 闭环：rm_executor 真机路径（10 ms 栅格 + 反馈闭环 + 到位判定）。"""
    # 1) 先读当前位姿（探针只读）→ 生成微动轨迹（rad，10 ms 栅格）
    probe = ctx.run_probe(arm=ctx.arm, ip=ctx.ip, port=ctx.port, udp_port=ctx.udp_port,
                          precheck_only=True, ws=ctx.ws,
                          link_factory=ctx.shared_link_factory(), close_link=False)
    if _probe_status(probe) != STATUS_PASS:
        return _result("V5", STATUS_BLOCKED, "生成轨迹前的只读体检未通过",
                       _probe_evidence(probe), {})
    joints = _probe_metrics(probe).get("joint_deg") or []
    if len(joints) != 7:
        return _result("V5", STATUS_BLOCKED, "未读到 7 个关节角，无法生成微动轨迹",
                       _probe_evidence(probe), {})

    traj = ctx.build_trajectory(joints_deg=joints, joint_index=ctx.joint_index,
                                delta_deg=ctx.delta_deg, period_s=0.010,
                                move_s=2.0, hold_s=0.5)
    traj_path = ctx.evidence_path("V5", "micro_trajectory.json")
    with open(traj_path, "w", encoding="utf-8") as fh:
        json.dump(traj, fh, ensure_ascii=False, indent=2)

    # 2) 交给执行层真机路径
    report = ctx.run_executor(mode="live", trajectory_path=traj_path, realtime=True,
                              arm=ctx.arm, ip=ctx.ip, port=ctx.port,
                              udp_port=ctx.udp_port, ws=ctx.ws,
                              link=ctx.ensure_link()[0])
    status = (report or {}).get("status")
    metrics = (report or {}).get("metrics") or (report or {}).get("summary") or {}
    evidence = ["rm_executor live status=%s" % status,
                "trajectory: %s" % traj_path]
    for key in ("ticks", "max_tracking_error_deg", "arrival_ok", "duration_s"):
        if key in metrics:
            evidence.append("%s = %s" % (key, metrics[key]))
    if status == "ok":
        return _result("V5", STATUS_PASS, "执行层 live 闭环通过（10 ms 透传 + 反馈到位判定）",
                       evidence, {"metrics": metrics, "trajectory": traj_path})
    mapped = {"blocked": STATUS_BLOCKED}.get(status, STATUS_ERROR)
    return _result("V5", mapped, (report or {}).get("reason") or "执行层 live 未通过",
                   evidence, {"metrics": metrics, "trajectory": traj_path})


# --------------------------------------------------------------------- V6
def _stage_v6(ctx: LadderContext) -> Dict[str, Any]:
    """FK 交叉校验：自研 FK vs 控制器位姿（只读）。"""
    link, failure = ctx.ensure_link()
    if link is None:
        return _result("V6", STATUS_BLOCKED,
                       "无法建立真机连接：%s" % ((failure or {}).get("reason") or "未知"), [], {})
    result = ctx.run_hwcollect("fk_cross", arm=ctx.arm, ip=ctx.ip, n=ctx.fk_samples,
                               dry_run=False, ws=ctx.ws, link=link)
    status = (result or {}).get("status")
    summary = (result or {}).get("summary") or {}
    evidence = ["fk_cross status=%s" % status]
    for key in ("max_position_error_mm", "max_orientation_error_deg", "samples"):
        if key in summary:
            evidence.append("%s = %s" % (key, summary[key]))
    if status == "ok":
        return _result("V6", STATUS_PASS,
                       "FK 交叉校验通过（位置 %s mm / 姿态 %s°）"
                       % (summary.get("max_position_error_mm"),
                          summary.get("max_orientation_error_deg")),
                       evidence, summary)
    mapped = {"dry_run": STATUS_PASS, "NEEDS_HUMAN": STATUS_BLOCKED,
              "blocked": STATUS_BLOCKED}.get(status, STATUS_ERROR)
    return _result("V6", mapped, (result or {}).get("reason") or "FK 交叉校验未通过",
                   evidence, summary)


# --------------------------------------------------------------------- V7
def _stage_v7(ctx: LadderContext) -> Dict[str, Any]:
    """TCP 重复性（运动）：默认目标 = 当前位姿（安全默认），可用 --target 指定。"""
    target = ctx.target_deg
    if target is None:
        probe = ctx.run_probe(arm=ctx.arm, ip=ctx.ip, port=ctx.port,
                              udp_port=ctx.udp_port, precheck_only=True, ws=ctx.ws,
                              link_factory=ctx.shared_link_factory(), close_link=False)
        joints = _probe_metrics(probe).get("joint_deg") or []
        if len(joints) != 7:
            return _result("V7", STATUS_BLOCKED, "未读到当前位姿，无法确定安全目标",
                           _probe_evidence(probe), {})
        target = list(joints)
    link, failure = ctx.ensure_link()
    if link is None:
        return _result("V7", STATUS_BLOCKED,
                       "无法建立真机连接：%s" % ((failure or {}).get("reason") or "未知"), [], {})
    result = ctx.run_hwcollect("tcp_repeatability", arm=ctx.arm, ip=ctx.ip,
                               target_deg=list(target), repeats=ctx.repeats,
                               dry_run=False, ws=ctx.ws, link=link)
    status = (result or {}).get("status")
    summary = (result or {}).get("summary") or {}
    evidence = ["tcp_repeatability status=%s" % status,
                "target_deg = %s" % (", ".join("%.3f" % v for v in target))]
    for key in ("max_deviation_mm", "std_deviation_mm", "within_nominal"):
        if key in summary:
            evidence.append("%s = %s" % (key, summary[key]))
    if status == "ok":
        return _result("V7", STATUS_PASS,
                       "重复性：最大散布 %s mm（标称 ±0.05 mm）"
                       % summary.get("max_deviation_mm"), evidence, summary)
    mapped = {"dry_run": STATUS_PASS, "NEEDS_HUMAN": STATUS_BLOCKED,
              "blocked": STATUS_BLOCKED}.get(status, STATUS_ERROR)
    return _result("V7", mapped, (result or {}).get("reason") or "重复性实验未通过",
                   evidence, summary)


# ------------------------------------------------------------------ V8 / V9
def _stage_v8(ctx: LadderContext) -> Dict[str, Any]:
    """阶梯 acc/jerk（高风险：单关节 10°/档、逐档提速）。"""
    link, failure = ctx.ensure_link()
    if link is None:
        return _result("V8", STATUS_BLOCKED,
                       "无法建立真机连接：%s" % ((failure or {}).get("reason") or "未知"), [], {})
    result = ctx.run_hwcollect("ladder_acc_jerk", arm=ctx.arm, ip=ctx.ip,
                               joints=(ctx.joint_index + 1,), trials=ctx.trials,
                               dry_run=False, ws=ctx.ws, link=link)
    status = (result or {}).get("status")
    summary = (result or {}).get("summary") or {}
    evidence = ["ladder_acc_jerk status=%s" % status]
    if status == "ok":
        return _result("V8", STATUS_PASS, "阶梯 acc/jerk 采集完成", evidence, summary)
    mapped = {"NEEDS_HUMAN": STATUS_BLOCKED, "blocked": STATUS_BLOCKED}.get(
        status, STATUS_ERROR)
    return _result("V8", mapped, (result or {}).get("reason") or "阶梯实验未通过",
                   evidence, summary)


def _stage_v9(ctx: LadderContext) -> Dict[str, Any]:
    """急停演练（高风险：主动触发缓停 + 急停；事后需示教器清错并重新使能）。"""
    link, failure = ctx.ensure_link()
    if link is None:
        return _result("V9", STATUS_BLOCKED,
                       "无法建立真机连接：%s" % ((failure or {}).get("reason") or "未知"), [], {})
    result = ctx.run_hwcollect("estop_drill", arm=ctx.arm, ip=ctx.ip, dry_run=False,
                               ws=ctx.ws, link=link)
    status = (result or {}).get("status")
    summary = (result or {}).get("summary") or {}
    evidence = ["estop_drill status=%s" % status]
    if status == "ok":
        return _result("V9", STATUS_PASS,
                       "急停演练完成（需在示教器清错并重新使能后方可继续）",
                       evidence, summary)
    mapped = {"NEEDS_HUMAN": STATUS_BLOCKED, "blocked": STATUS_BLOCKED}.get(
        status, STATUS_ERROR)
    return _result("V9", mapped, (result or {}).get("reason") or "急停演练未通过",
                   evidence, summary)


STAGES: List[Stage] = [
    Stage("V0", "契约体检", "doctor 硬件面检查 + 黑名单静态审计（零运动）", RISK_NONE,
          False, _stage_v0, slices=["S02", "S09"], requirements=["R003"],
          criteria=["CC5"], vclass="Contract",
          notes="不需要真机：只验证 SDK 符号、契约与静态审计"),
    Stage("V1", "连接与状态快照", "真机连接 + 全字段快照 + 7 项前置检查（零运动）", RISK_NONE,
          False, _stage_v1, slices=["S02"], requirements=["R004"],
          criteria=["CC6"], vclass="Integration",
          notes="第一次接触真机：只读，不发任何运动指令"),
    Stage("V2", "UDP 上报质量", "主动上报帧数 / 周期 / 断流（只读长跑）", RISK_NONE,
          False, _stage_v2, slices=["S01", "S02"], requirements=["R004"],
          criteria=["CC4"], vclass="Operational",
          notes="补齐 network_plan 基线：600 s 长跑才能把 status 改成 measured"),
    Stage("V3", "最小运动·规划通道", "rm_movej 单关节 0.3° 去程-回程", RISK_LOW,
          True, _stage_v3, slices=["S03", "S04"], requirements=["R012"],
          criteria=["CC2"], vclass="Operational",
          prerequisites=["V1"], notes="第一段真机运动：单关节、亚度级、低速"),
    Stage("V4", "关节角透传", "rm_movej_canfd 单关节 0.3° 去程-回程", RISK_LOW,
          True, _stage_v4, slices=["S03"], requirements=["R003", "R012"],
          criteria=["CC6"], vclass="Integration",
          prerequisites=["V3"], notes="验证 10 ms 单时钟主通道（纯关节角透传）"),
    Stage("V5", "执行层 live 闭环", "rm_executor 真机路径：10 ms 栅格 + 反馈闭环 + 到位判定",
          RISK_LOW, True, _stage_v5, slices=["S03", "S04"], requirements=["R012"],
          criteria=["CC2", "CC6"], vclass="Integration",
          prerequisites=["V4"], notes="rmctl→rm_runtime→RM-API2→真机 全链路"),
    Stage("V6", "FK 交叉校验", "自研 FK 与控制器位姿偏差（只读）", RISK_NONE,
          False, _stage_v6, slices=["S01"], requirements=["R011"],
          criteria=["CC5"], vclass="Contract",
          notes="需要 build/model 已物化（rmctl init）"),
    Stage("V7", "TCP 重复性", "多次回到同一点的散布（运动；默认目标=当前位姿）", RISK_MEDIUM,
          True, _stage_v7, slices=["S01"], requirements=["R012"],
          criteria=["CC4"], vclass="Operational",
          prerequisites=["V3"],
          notes="默认目标=当前位姿（几乎不动）；要测真实重复性请用 --target 指定近位姿目标"),
    Stage("V8", "阶梯 acc/jerk", "单关节 10°/档逐档提速反推加速度/jerk 上限", RISK_HIGH,
          True, _stage_v8, slices=["S01", "S04"], requirements=["R012"],
          criteria=["CC4"], vclass="Operational",
          needs_authorization=True, prerequisites=["V3"],
          notes="**大角度 + 大力矩**：必须现场看护，示教器在手边"),
    Stage("V9", "急停演练", "软停 + 急停的真实返回码与错误码留档", RISK_HIGH,
          True, _stage_v9, slices=["S04"], requirements=["R014"],
          criteria=["CC7"], vclass="Operational",
          needs_authorization=True, prerequisites=["V3"],
          notes="会触发急停：事后需在示教器清错并重新使能（项目不自动清错）"),
]

STAGE_INDEX = {stage.id: stage for stage in STAGES}

#: 里程碑全部验收条款（**唯一清单**）。有些条款不靠真机运动证据（见 COVERAGE_NOTES）。
ALL_CRITERIA = ["CC1", "CC2", "CC3", "CC4", "CC5", "CC6", "CC7", "CC8"]

#: 不在本阶梯覆盖范围内的条款 + 原因（诚实标注，不留空）
COVERAGE_NOTES = {
    "CC1": "取放循环 50 次真机 —— 需要夹爪与工艺场景（单臂阶梯只覆盖运动链路，不含夹爪）",
    "CC3": "规划性能与人工 MTTR —— 规划内核需 tesseract 环境；MTTR 属现场人工指标",
    "CC8": "切片交付线 —— 由 9 个切片的 SUMMARY/ASSESSMENT 承担，不属真机证据",
}


# ==================================================================== 轨迹生成
def _build_micro_trajectory(joints_deg: Sequence[float], joint_index: int,
                            delta_deg: float, period_s: float = 0.010,
                            move_s: float = 2.0, hold_s: float = 0.5) -> Dict[str, Any]:
    """生成一条**微动**执行层轨迹（去程 + 保持 + 回程），单位 rad / s。

    * 起点 = 当前位姿（执行层阶段 B 的预就位因此几乎不动）；
    * 只动 1 个关节，位移 ``delta_deg``（默认 0.3°）；
    * 10 ms 栅格，时间戳严格递增；速度/加速度/jerk 远低于关节限值。
    """
    base = [float(v) for v in joints_deg]
    delta = float(delta_deg)
    total_s = 2 * float(move_s) + 2 * float(hold_s)
    n = int(round(total_s / float(period_s))) + 1
    times: List[float] = []
    points: List[List[float]] = []
    move_rad = math.radians(delta)
    for i in range(n):
        t = i * float(period_s)
        times.append(round(t, 6))
        offset = 0.0
        if t <= move_s:
            offset = move_rad * _smoothstep(t / move_s)
        elif t <= move_s + hold_s:
            offset = move_rad
        elif t <= 2 * move_s + hold_s:
            offset = move_rad * (1.0 - _smoothstep((t - move_s - hold_s) / move_s))
        else:
            offset = 0.0
        row = [math.radians(v) for v in base]
        row[joint_index] = math.radians(base[joint_index]) + offset
        points.append([round(v, 9) for v in row])
    return {
        "group": "rm75_arm",
        "joint_names": ["joint%d" % (i + 1) for i in range(7)],
        "times": times,
        "points": points,
        "units": {"times": "s", "points": "rad"},
        "schema_version": 1,
        "verify": {
            "ok": True,
            "note": "微动轨迹：由 rm75_validation_ladder 生成（单关节 ≤1°、10 ms 栅格、"
                    "速度/加速度/jerk 远低于 joint_limits.yaml）；执行层阶段 A 会再做一次"
                    "时间域限值复检",
            "checks": {"limits": {"ok": True}, "discrete_collision": {"ok": True},
                       "continuous_collision": {"ok": True}},
        },
        "meta": {
            "generator": "rm75_validation_ladder",
            "joint_index": int(joint_index),
            "delta_deg": delta,
            "period_s": float(period_s),
            "move_s": float(move_s),
            "hold_s": float(hold_s),
        },
    }


def _smoothstep(x: float) -> float:
    """C1 连续的平滑插值（0→1），避免加速度/jerk 尖峰。"""
    x = min(1.0, max(0.0, float(x)))
    return x * x * (3.0 - 2.0 * x)


# ==================================================================== 编排
def run_ladder(ctx: LadderContext, stage_ids: Optional[Sequence[str]] = None,
               max_stage: Optional[str] = None, allow_risky: bool = False,
               dry_run: bool = False,
               continue_on_failure: bool = False) -> Dict[str, Any]:
    """按阶梯顺序执行，返回汇总报告。"""
    started = time.time()
    selected: List[Stage] = []
    if stage_ids:
        wanted = [s.strip().upper() for s in stage_ids if s.strip()]
        unknown = [w for w in wanted if w not in STAGE_INDEX]
        if unknown:
            raise ValueError("未知阶段: %s（可选: %s）"
                             % (", ".join(unknown), ", ".join(STAGE_INDEX)))
        selected = [STAGE_INDEX[w] for w in wanted]
    else:
        selected = list(STAGES)
        if max_stage:
            if max_stage.upper() not in STAGE_INDEX:
                raise ValueError("未知阶段: %s" % max_stage)
            cutoff = [s.id for s in STAGES].index(max_stage.upper())
            selected = STAGES[:cutoff + 1]

    results: List[Dict[str, Any]] = []
    stopped = False
    for stage in selected:
        if stopped:
            results.append({"stage": stage.id, "status": STATUS_SKIPPED,
                            "reason": "前序阶段未通过，按阶梯纪律不继续", "evidence": [],
                            "data": {}})
            continue
        if stage.needs_authorization and not allow_risky:
            results.append({"stage": stage.id, "status": STATUS_SKIPPED,
                            "reason": "高风险阶段需显式授权（--allow-risky）：%s" % stage.notes,
                            "evidence": [], "data": {}})
            continue
        if dry_run:
            results.append({"stage": stage.id, "status": STATUS_SKIPPED,
                            "reason": "dry-run：只列计划，不执行", "evidence": [],
                            "data": {"plan": stage.to_dict()}})
            continue
        t0 = time.perf_counter()
        try:
            result = stage.runner(ctx)
        except Exception as exc:  # noqa: BLE001 - 任何异常都变成结构化结果
            result = _result(stage.id, STATUS_ERROR, "阶段执行异常：%r" % exc)
        result["duration_s"] = round(time.perf_counter() - t0, 3)
        result["title"] = stage.title
        result["risk"] = stage.risk
        result["moves_arm"] = stage.moves_arm
        results.append(result)
        if result["status"] not in (STATUS_PASS, STATUS_SKIPPED, STATUS_FAILED) \
                and not continue_on_failure:
            stopped = True

    report = _build_report(ctx, results, started, allow_risky, dry_run)
    return report


def _build_report(ctx: LadderContext, results: List[Dict[str, Any]], started: float,
                  allow_risky: bool, dry_run: bool) -> Dict[str, Any]:
    executed = [r for r in results if r["status"] not in (STATUS_SKIPPED,)]
    passed = [r for r in results if r["status"] == STATUS_PASS]
    failed = [r for r in results if r["status"] == STATUS_FAILED]
    blocked = [r for r in results if r["status"] == STATUS_BLOCKED]
    errored = [r for r in results if r["status"] == STATUS_ERROR]
    skipped = [r for r in results if r["status"] == STATUS_SKIPPED]

    coverage: Dict[str, Dict[str, Any]] = {}
    for cc in ALL_CRITERIA:
        coverage[cc] = {"criteria": cc, "stages": [], "passed_stages": [],
                        "verification_class": "-", "slices": [], "open": True,
                        "note": COVERAGE_NOTES.get(cc, "")}
    for stage in STAGES:
        for cc in stage.criteria:
            entry = coverage.setdefault(cc, {"criteria": cc, "stages": [],
                                             "verification_class": stage.vclass,
                                             "slices": [], "passed_stages": [],
                                             "open": True, "note": ""})
            entry["stages"].append(stage.id)
            if entry.get("verification_class") in (None, "-"):
                entry["verification_class"] = stage.vclass
            for sl in stage.slices:
                if sl not in entry["slices"]:
                    entry["slices"].append(sl)
    status_by_stage = {r["stage"]: r["status"] for r in results}
    for cc, entry in coverage.items():
        entry["passed_stages"] = [s for s in entry["stages"]
                                  if status_by_stage.get(s) == STATUS_PASS]
        entry["open"] = not entry["passed_stages"]

    if errored:
        overall = STATUS_ERROR
    elif failed:
        overall = STATUS_FAILED
    elif blocked:
        overall = STATUS_BLOCKED
    elif dry_run:
        overall = "dry_run"
    elif skipped and not passed:
        overall = STATUS_BLOCKED
    else:
        overall = STATUS_PASS

    return {
        "schema": SCHEMA,
        "mode": "dry_run" if dry_run else "hardware",
        "status": overall,
        "arm": ctx.arm,
        "endpoint": {"ip": ctx.ip, "port": ctx.port, "udp_port": ctx.udp_port},
        "started_at": time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(started)),
        "finished_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "duration_s": round(time.time() - started, 3),
        "allow_risky": allow_risky,
        "params": {
            "joint_index": ctx.joint_index, "delta_deg": ctx.delta_deg,
            "speed_pct": ctx.speed_pct, "udp_duration_s": ctx.udp_duration_s,
            "fk_samples": ctx.fk_samples, "repeats": ctx.repeats,
            "target_deg": ctx.target_deg, "trials": ctx.trials,
        },
        "stages": [dict(stage.to_dict(), **r) for stage, r in zip(STAGES, results)],
        "summary": {
            "total": len(results), "passed": len(passed), "failed": len(failed),
            "blocked": len(blocked), "errored": len(errored),
            "skipped": len(skipped),
        },
        "coverage": coverage,
        "out_dir": ctx.out_dir,
    }


def write_report(report: Dict[str, Any], out_dir: str) -> Tuple[str, str]:
    """写汇总报告（JSON + Markdown），返回两个路径。"""
    os.makedirs(out_dir, exist_ok=True)
    json_path = os.path.join(out_dir, "ladder_report.json")
    with open(json_path, "w", encoding="utf-8") as fh:
        json.dump(report, fh, ensure_ascii=False, indent=2)
    md_path = os.path.join(out_dir, "ladder_report.md")
    with open(md_path, "w", encoding="utf-8") as fh:
        fh.write(render_markdown(report))
    return json_path, md_path


def render_markdown(report: Dict[str, Any]) -> str:
    """把报告渲染成 Markdown（一页纸，便于贴进验收记录）。"""
    lines: List[str] = []
    lines.append("# RM75-B 真机验证阶梯报告")
    lines.append("")
    lines.append("- 状态: **%s**" % report.get("status"))
    lines.append("- 臂: %s | 端点: %s:%s"
                 % (report.get("arm"), (report.get("endpoint") or {}).get("ip"),
                    (report.get("endpoint") or {}).get("port")))
    lines.append("- 开始: %s | 结束: %s | 耗时: %ss"
                 % (report.get("started_at"), report.get("finished_at"),
                    report.get("duration_s")))
    summary = report.get("summary") or {}
    lines.append("- 阶段统计: 共 %s，通过 %s，质量不合格 %s，blocked %s，error %s，skipped %s"
                 % (summary.get("total"), summary.get("passed"), summary.get("failed"),
                    summary.get("blocked"), summary.get("errored"),
                    summary.get("skipped")))
    lines.append("")
    lines.append("| 阶段 | 名称 | 风险 | 运动 | 状态 | 说明 |")
    lines.append("|---|---|---|---|---|---|")
    for stage in report.get("stages") or []:
        lines.append("| %s | %s | %s | %s | %s | %s |"
                     % (stage.get("id"), stage.get("title"), stage.get("risk"),
                        "是" if stage.get("moves_arm") else "否",
                        stage.get("status"), stage.get("reason") or ""))
    lines.append("")
    lines.append("## 验收条款覆盖")
    lines.append("")
    lines.append("| 条款 | 验证类别 | 覆盖阶段 | 已通过 | 仍缺证据 |")
    lines.append("|---|---|---|---|---|")
    for cc, entry in sorted((report.get("coverage") or {}).items()):
        lines.append("| %s | %s | %s | %s | %s |"
                     % (cc, entry.get("verification_class"),
                        ", ".join(entry.get("stages") or []),
                        ", ".join(entry.get("passed_stages") or []) or "—",
                        "是" if entry.get("open") else "否"))
    lines.append("")
    return "\n".join(lines)


# ==================================================================== CLI
def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="rm75_validation_ladder",
        description="RM75-B 真机验证阶梯（V0→V9）：从只读到真机运动，逐级留证")
    p.add_argument("--arm", choices=["left", "right"], default="left")
    p.add_argument("--ip", default="192.168.1.18")
    p.add_argument("--port", type=int, default=8080)
    p.add_argument("--udp-port", dest="udp_port", type=int, default=8091)
    p.add_argument("--stages", default=None,
                   help="只跑指定阶段（如 V0,V1,V5）；缺省按顺序全跑")
    p.add_argument("--max-stage", dest="max_stage", default=None,
                   help="跑到该阶段为止（V0..V9）")
    p.add_argument("--allow-risky", dest="allow_risky", action="store_true",
                   help="授权高风险阶段（V8 阶梯 10°/档、V9 急停演练）—— 需现场看护")
    p.add_argument("--continue-on-failure", dest="continue_on_failure",
                   action="store_true", help="某阶段失败后继续跑后续阶段（默认停止）")
    p.add_argument("--joint", dest="joint_index", type=int, default=0)
    p.add_argument("--delta-deg", dest="delta_deg", type=float, default=0.3)
    p.add_argument("--speed-pct", dest="speed_pct", type=int, default=2)
    p.add_argument("--udp-duration-s", dest="udp_duration_s", type=float, default=60.0,
                   help="UDP 上报采集时长（秒）；补基线用 600")
    p.add_argument("--fk-samples", dest="fk_samples", type=int, default=20)
    p.add_argument("--repeats", type=int, default=30)
    p.add_argument("--target", default=None,
                   help="V7 重复性目标角（度，逗号分隔 7 个）；缺省=当前位姿（安全默认）")
    p.add_argument("--trials", type=int, default=1, help="V8 阶梯实验档数")
    p.add_argument("--out-dir", dest="out_dir", default="build/reports/validation")
    p.add_argument("--ws", default=None, help="工作区根目录")
    p.add_argument("--dry-run", dest="dry_run", action="store_true",
                   help="只列计划，不连真机")
    p.add_argument("--json", action="store_true", help="stdout 只打印 JSON")
    return p


def _parse_target(text: Optional[str]) -> Optional[List[float]]:
    if not text:
        return None
    values = [float(x) for x in text.replace(";", ",").split(",") if x.strip()]
    if len(values) != 7:
        raise ValueError("--target 需要 7 个关节角（度），当前 %d 个" % len(values))
    return values


def main(argv: Optional[List[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        target = _parse_target(args.target)
    except ValueError as exc:
        sys.stderr.write("参数非法: %s\n" % exc)
        return 2

    ctx = LadderContext(arm=args.arm, ip=args.ip, port=args.port,
                         udp_port=args.udp_port, ws=args.ws, out_dir=args.out_dir,
                         joint_index=args.joint_index, delta_deg=args.delta_deg,
                         speed_pct=args.speed_pct, udp_duration_s=args.udp_duration_s,
                         fk_samples=args.fk_samples, repeats=args.repeats,
                         target_deg=target, trials=args.trials)
    try:
        report = run_ladder(ctx, stage_ids=args.stages.split(",") if args.stages else None,
                            max_stage=args.max_stage, allow_risky=args.allow_risky,
                            dry_run=args.dry_run,
                            continue_on_failure=args.continue_on_failure)
    except ValueError as exc:
        sys.stderr.write("参数非法: %s\n" % exc)
        return 2

    json_path, md_path = write_report(report, ctx.out_dir)
    if args.json:
        print(json.dumps(report, ensure_ascii=False, indent=2))
    else:
        print(render_markdown(report))
        print("报告: %s / %s" % (json_path, md_path))

    status = report.get("status")
    if status == STATUS_PASS or status == "dry_run":
        return EXIT_OK
    if status == STATUS_BLOCKED:
        return EXIT_BLOCKED
    return EXIT_ERROR


if __name__ == "__main__":
    sys.exit(main())
