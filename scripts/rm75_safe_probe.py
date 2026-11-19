#!/usr/bin/env python3
"""RM75-B 真机「最小动作」安全探针（safest possible test）。

目的
----
在**不产生任何危险动作**的前提下，验证本项目的控制接口是否真的能控制真机
（睿尔曼 RM75-B，控制器 ``192.168.1.18:8080``）：

* 连接与句柄语义（``rm_hw_bridge.bridge.connect`` → ``rm_create_robot_arm``）；
* 只读状态采集（``rm_get_joint_degree`` / ``rm_get_arm_all_state`` / …）；
* 安全开关下发（``rm_set_collision_state`` / ``rm_set_self_collision_enable``）；
* **关节角通道**运动（``rm_movej``，单关节、亚度级位移、低速度百分比）；
* 停机族（``rm_set_arm_stop``）——任一环节异常立即急停。

安全设计（硬约束**写在代码里**，不靠"人记得"）
----------------------------------------------
1. **单关节、单次、亚度级**：默认只动 1 个关节，位移 ``0.3°``（硬上限 ``1.0°``）；
2. **低速**：``rm_movej`` 的速度百分比默认 ``2%``（硬上限 ``10%``）；
3. **先只读后运动**：默认先跑 ``--precheck-only``（纯只读），确认臂在线、无错误、
   未在执行其它轨迹，才允许运动；
4. **安全开关先武装**：运动前必须成功下发碰撞检测等级 ``8``（最灵敏）+ 自碰撞检测
   使能，失败即中止（**不动**）；
5. **去程-回程对称**：先动 ``+δ``，回读校验，再回到原位并回读校验 —— 臂最终状态
   与初始状态一致；
6. **异常即急停**：任何一步失败/超时/回读偏差超限 → 调用 ``rm_set_arm_stop``
   （L4 急停），并把证据写进报告；
7. **不伪造数值**：连不上 / 读失败一律 ``blocked`` / ``error`` + 证据，**不填占位值**。

用法
----
::

    # 1) 先只读体检（零运动）
    python3 scripts/rm75_safe_probe.py --ip 192.168.1.18 --precheck-only

    # 2) 最小动作验证（0.3° / 2% 速度 / 单关节 / 去程回程）
    python3 scripts/rm75_safe_probe.py --ip 192.168.1.18 --out build/reports/rm75_safe_probe.json

    # 3) 离线预演（零 SDK 调用）
    python3 scripts/rm75_safe_probe.py --ip 192.168.1.18 --dry-run

退出码：``0`` 通过 / ``2`` 参数非法 / ``3`` blocked（外部条件不具备）/ ``4`` error。
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from typing import Any, Dict, List, Optional, Tuple

SCHEMA = "rmctl.rm75_safe_probe/1"

# ------------------------------------------------------------------ 安全常量
#: 位移硬上限（度）——超过就拒绝，**一次 SDK 调用都不发**
MAX_DELTA_DEG = 1.0
#: 默认位移（度）
DEFAULT_DELTA_DEG = 0.3
#: ``rm_movej`` 速度百分比硬上限
MAX_SPEED_PCT = 10
#: 默认速度百分比（2% ≈ 3.6°/s @180°/s）
DEFAULT_SPEED_PCT = 2
#: 到位判定容差（度）
ARRIVAL_TOL_DEG = 0.2
#: 其它关节漂移容差（度）
DRIFT_TOL_DEG = 0.1
#: 回程到位容差（度）
RETURN_TOL_DEG = 0.2
#: 碰撞检测等级（0=不检测 … 8=最灵敏）——运动前一律武装到 8
COLLISION_STAGE = 8
#: 关节限位余量（度）——目标位姿必须离限位至少这么远
LIMIT_MARGIN_DEG = 1.0
#: 单次 ``rm_movej`` 的最长等待（秒）——阻塞调用超时保护
MOVE_TIMEOUT_S = 30.0
#: 控制器无规划（纯透传/空闲）时的轨迹类型名
NO_PLANNING_TYPE = "RM_NO_PLANNING_E"

DEFAULT_IP = "192.168.1.18"
DEFAULT_PORT = 8080
DEFAULT_UDP_PORT = 8091

STATUS_OK = "ok"
STATUS_DRY_RUN = "dry_run"
STATUS_BLOCKED = "blocked"
STATUS_ERROR = "error"

EXIT_OK = 0
EXIT_USAGE = 2
EXIT_BLOCKED = 3
EXIT_ERROR = 4

#: 现场威胁面提示（与 rmctl.hwcollect 的口径一致，便于检索）
THREAT_NOTES = (
    {"id": "micro_motion_only",
     "text": "本探针只做单关节亚度级位移（默认 0.3°，硬上限 1.0°）+ 2% 速度，"
             "去程回程对称，结束时臂回到初始位姿。"},
    {"id": "collision_stage_8",
     "text": "运动前必须成功下发碰撞检测等级 8（最灵敏）与自碰撞检测使能，"
             "失败即中止且不运动。"},
    {"id": "estop_on_anomaly",
     "text": "任一环节失败/回读超差 → 立即 rm_set_arm_stop（L4 急停）并留证据。"},
    {"id": "single_client_per_endpoint",
     "text": "同一 ip:port 只允许一个客户端；采集前确认端口未被占用。"},
    {"id": "no_fabricated_values",
     "text": "读不到就报错，绝不填占位数值。"},
)


class ProbeInputError(Exception):
    """参数非法（面向操作者，可直接粘贴）。"""


# ------------------------------------------------------------------ 纯函数层
def check_envelope(joint_index: Any, delta_deg: Any, speed_pct: Any,
                   dof: int = 7) -> Tuple[bool, str]:
    """安全包络校验：**任何**越界都在 0 次 SDK 调用前被拒。

    返回 ``(ok, reason)``。
    """
    if isinstance(joint_index, bool) or not isinstance(joint_index, int):
        return False, "joint_index 必须是整数（当前 %r）" % (joint_index,)
    if not 0 <= joint_index < dof:
        return False, ("joint_index=%d 越界（允许 0..%d）"
                       % (joint_index, dof - 1))
    if isinstance(delta_deg, bool) or not isinstance(delta_deg, (int, float)):
        return False, "delta_deg 必须是数字（当前 %r）" % (delta_deg,)
    if not math.isfinite(float(delta_deg)):
        return False, "delta_deg 必须是有限数（当前 %r）" % (delta_deg,)
    if abs(float(delta_deg)) > MAX_DELTA_DEG:
        return False, ("delta_deg=%.3f° 超过安全上限 %.1f°（本探针只允许亚度级微动）"
                       % (float(delta_deg), MAX_DELTA_DEG))
    if float(delta_deg) == 0.0:
        return False, "delta_deg=0 无验证意义（必须非零）"
    if isinstance(speed_pct, bool) or not isinstance(speed_pct, int):
        return False, "speed_pct 必须是整数（当前 %r）" % (speed_pct,)
    if speed_pct < 1:
        return False, "speed_pct=%d 非法（rm_movej 速度百分比允许 1..100）" % speed_pct
    if speed_pct > MAX_SPEED_PCT:
        return False, ("speed_pct=%d 超过安全上限 %d%%（本探针只允许低速微动）"
                       % (speed_pct, MAX_SPEED_PCT))
    return True, ""


def plan_target(joints: List[float], joint_index: int, delta_deg: float) -> List[float]:
    """由当前关节角算出目标位姿（只改一个关节）。"""
    target = [float(v) for v in joints]
    target[joint_index] = float(target[joint_index]) + float(delta_deg)
    return target


def _limit_bounds_deg(limits: Dict[str, Any], joint_index: int,
                       dof: int = 7) -> Tuple[Optional[float], Optional[float]]:
    """取关节限位（度）。``joint_limits.yaml`` 是弧度，这里换算成度。"""
    if not limits:
        return None, None
    key = "joint%d" % (joint_index + 1)
    entry = limits.get(key)
    if not isinstance(entry, dict):
        return None, None
    lower = entry.get("lower")
    upper = entry.get("upper")
    lo = math.degrees(float(lower)) if isinstance(lower, (int, float)) else None
    hi = math.degrees(float(upper)) if isinstance(upper, (int, float)) else None
    return lo, hi


def preflight_checks(snapshot: Dict[str, Any], limits: Dict[str, Any],
                     joint_index: int, delta_deg: float,
                     limit_margin_deg: float = LIMIT_MARGIN_DEG) -> List[Dict[str, Any]]:
    """运动前**只读**检查（全部通过才允许运动）。

    返回逐条检查结果 ``[{"name", "ok", "detail"}]``。
    """
    checks: List[Dict[str, Any]] = []

    def add(name, ok, detail):
        checks.append({"name": name, "ok": bool(ok), "detail": detail})

    arm_err = snapshot.get("arm_err") or []
    # 厂商结构体里 err[] 会带长度（err_len），值为 0 表示"无错误"——
    # 因此只看**非零**错误码，全 0 视为正常。
    arm_err_bad = [int(c) for c in arm_err if int(c) != 0]
    add("arm_err_empty", not arm_err_bad,
        "臂级错误码: %s" % (arm_err_bad or "无（原始: %s）" % (list(arm_err) or "无")))

    err_codes = snapshot.get("joint_err_code") or []
    bad = [(i + 1, c) for i, c in enumerate(err_codes) if int(c) != 0]
    add("joint_err_code_zero", not bad, "非零关节错误码: %s" % (bad or "无"))

    en_flags = snapshot.get("joint_en_flag") or []
    disabled = [i + 1 for i, f in enumerate(en_flags) if int(f) != 1]
    add("all_joints_enabled", not disabled,
        "未使能关节: %s" % (disabled or "无（7 轴全部使能）"))

    traj_name = snapshot.get("controller_trajectory_type_name")
    add("controller_idle", traj_name == NO_PLANNING_TYPE,
        "控制器规划类型: %s（期望 %s = 控制器无规划）" % (traj_name, NO_PLANNING_TYPE))

    joints = snapshot.get("joint_deg") or []
    add("joints_readable", len(joints) == 7,
        "读到 %d 个关节角" % len(joints))

    if len(joints) == 7:
        lo, hi = _limit_bounds_deg(limits, joint_index)
        if lo is None or hi is None:
            add("within_limits", True, "无该关节限位条目（跳过限位检查）")
        else:
            value = float(joints[joint_index])
            ok = (lo + limit_margin_deg) <= value <= (hi - limit_margin_deg)
            add("within_limits", ok,
                "关节%d 当前 %.3f°，允许区间 [%.3f°, %.3f°]（余量 %.1f°）"
                % (joint_index + 1, value, lo + limit_margin_deg, hi - limit_margin_deg,
                   limit_margin_deg))
            target = plan_target(joints, joint_index, delta_deg)
            tvalue = float(target[joint_index])
            tok = (lo + limit_margin_deg) <= tvalue <= (hi - limit_margin_deg)
            add("target_within_limits", tok,
                "关节%d 目标 %.3f°，允许区间 [%.3f°, %.3f°]（余量 %.1f°）"
                % (joint_index + 1, tvalue, lo + limit_margin_deg, hi - limit_margin_deg,
                   limit_margin_deg))
    return checks


def evaluate_move(before: List[float], measured: List[float], target: List[float],
                  joint_index: int, arrival_tol_deg: float = ARRIVAL_TOL_DEG,
                  drift_tol_deg: float = DRIFT_TOL_DEG) -> Dict[str, Any]:
    """回读判定：运动关节到位 + 其它关节不漂移。"""
    moved_err = abs(float(measured[joint_index]) - float(target[joint_index]))
    drifts = []
    for i, value in enumerate(measured):
        if i == joint_index:
            continue
        drifts.append(abs(float(value) - float(before[i])))
    max_drift = max(drifts) if drifts else 0.0
    ok = moved_err <= arrival_tol_deg and max_drift <= drift_tol_deg
    return {
        "ok": ok,
        "moved_joint": joint_index + 1,
        "arrival_error_deg": round(moved_err, 6),
        "arrival_tol_deg": arrival_tol_deg,
        "max_drift_deg": round(max_drift, 6),
        "drift_tol_deg": drift_tol_deg,
        "measured_deg": [round(float(v), 6) for v in measured],
        "target_deg": [round(float(v), 6) for v in target],
    }


def evaluate_return(before: List[float], measured: List[float],
                    tol_deg: float = RETURN_TOL_DEG) -> Dict[str, Any]:
    """回程判定：全部关节回到初始位姿附近。"""
    diffs = [abs(float(a) - float(b)) for a, b in zip(measured, before)]
    worst = max(diffs) if diffs else 0.0
    worst_joint = (diffs.index(worst) + 1) if diffs else 0
    return {
        "ok": worst <= tol_deg,
        "max_return_error_deg": round(worst, 6),
        "worst_joint": worst_joint,
        "tol_deg": tol_deg,
        "measured_deg": [round(float(v), 6) for v in measured],
        "origin_deg": [round(float(v), 6) for v in before],
    }


# ------------------------------------------------------------------ 报告骨架
def _now_iso() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime())


def _phase(name: str, status: str, detail: str, data: Optional[Dict[str, Any]] = None
           ) -> Dict[str, Any]:
    return {"name": name, "status": status, "detail": detail, "data": data or {}}


def _report(status: str, arm: str, endpoint: Dict[str, Any], params: Dict[str, Any],
            phases: List[Dict[str, Any]], evidence: List[Dict[str, Any]],
            reason: str = "", next_action: str = "", mode: str = "hardware",
            started_at: Optional[str] = None) -> Dict[str, Any]:
    return {
        "schema": SCHEMA,
        "mode": mode,
        "status": status,
        "arm": arm,
        "endpoint": endpoint,
        "params": params,
        "started_at": started_at or _now_iso(),
        "finished_at": _now_iso(),
        "phases": phases,
        "evidence": evidence,
        "reason": reason,
        "next_action": next_action,
        "notes": list(THREAT_NOTES),
    }


# ------------------------------------------------------------------ 硬件接入
def _import_bridge(ws: Optional[str] = None):
    """惰性导入 ``rm_hw_bridge.bridge``（硬件适配层）。

    失败返回 ``(None, reason)`` —— 调用方据此走 ``blocked`` 降级，**不伪造数值**。
    """
    try:
        from rm_hw_bridge import bridge as _bridge
        return _bridge, ""
    except Exception as exc:  # pragma: no cover - 依赖缺失路径
        here = os.path.dirname(os.path.abspath(__file__))
        sibling = os.path.abspath(os.path.join(here, "..", "dualarm_ws", "src",
                                              "rm_hw_bridge"))
        if os.path.isdir(sibling) and sibling not in sys.path:
            sys.path.insert(0, sibling)
        try:
            from rm_hw_bridge import bridge as _bridge
            return _bridge, ""
        except Exception as exc2:  # pragma: no cover
            return None, "%r / 兜底路径 %s: %r" % (exc, sibling, exc2)


def _load_limits(ws: Optional[str] = None) -> Dict[str, Any]:
    """读关节限值（``joint_limits.yaml`` 的 ``rm75_arm`` 段，弧度）。"""
    try:
        from rm_hw_bridge import paths as hw_paths
        limit_path = hw_paths.joint_limits_path(ws)
    except Exception:
        limit_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "dualarm_ws", "src", "rm_description", "config", "joint_limits.yaml")
    if not os.path.exists(limit_path):
        return {}
    try:
        import yaml
        with open(limit_path, "r", encoding="utf-8") as fh:
            data = yaml.safe_load(fh) or {}
        return data.get("rm75_arm", {}) or {}
    except Exception:
        return {}


def _local_ip_for(target: str) -> Optional[str]:
    """推断本机（能到达 ``target`` 的那个网卡）IP —— UDP 主动上报的接收地址。"""
    import socket
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.connect((str(target), 9))  # 9 = discard，不会真的发包
            return sock.getsockname()[0]
        finally:
            sock.close()
    except OSError:
        return None


def _connect_link(arm: str, ip: str, port: int, udp_port: Optional[int],
                  enable_realtime: bool, ws: Optional[str], lib: Any,
                  realtime_period_ms: int = 5) -> Tuple[Any, Optional[Dict[str, Any]]]:
    """建立真机句柄。返回 ``(link, failure)``；``failure`` 非 None 表示失败。"""
    bridge, err = _import_bridge(ws)
    if bridge is None:
        return None, _report(
            STATUS_BLOCKED, arm,
            {"ip": ip, "port": port, "udp_port": udp_port},
            {}, [],
            [{"kind": "blocker",
              "detail": "硬件适配层 rm_hw_bridge 不可用：%s" % err,
              "next_action": "export PYTHONPATH=$PWD/dualarm_ws/src/rm_hw_bridge:$PYTHONPATH 后重跑"}],
            reason="硬件适配层不可用",
            next_action="把 rm_hw_bridge 加入 PYTHONPATH 后重跑",
        )
    kwargs = {}
    if enable_realtime:
        local_ip = _local_ip_for(ip)
        if local_ip:
            kwargs["realtime_target_ip"] = local_ip
        if udp_port:
            kwargs["realtime_target_port"] = int(udp_port)
    link = bridge.connect(ip=ip, port=int(port), udp_port=udp_port,
                          enable_realtime=bool(enable_realtime),
                          realtime_period_ms=int(realtime_period_ms),
                          lib=lib, ws=ws, arm=arm, **kwargs)
    if getattr(link, "status", "") == "ok":
        return link, None
    status = STATUS_BLOCKED if getattr(link, "status", "") == "blocked" else STATUS_ERROR
    return None, _report(
        status, arm, {"ip": ip, "port": port, "udp_port": udp_port}, {},
        [_phase("connect", status, getattr(link, "reason", "") or "连接失败",
                {"next_action": getattr(link, "next_action", "")})],
        [{"kind": "blocker" if status == STATUS_BLOCKED else "error",
          "detail": str(getattr(link, "reason", "") or "连接失败"),
          "next_action": str(getattr(link, "next_action", "") or "")}]
        + [str(e) for e in (getattr(link, "evidence", None) or [])],
        reason=str(getattr(link, "reason", "") or "连接失败"),
        next_action=str(getattr(link, "next_action", "") or ""),
    )


def _snapshot(link: Any) -> Tuple[Optional[Dict[str, Any]], Optional[Dict[str, Any]]]:
    """采一次状态快照。返回 ``(snapshot, failure)``。"""
    result = link.snapshot()
    if not isinstance(result, dict) or result.get("status") != "ok":
        reason = (result or {}).get("reason") or "状态快照采集失败"
        next_action = (result or {}).get("next_action") or ""
        return None, {
            "reason": reason,
            "next_action": next_action,
            "evidence": [str(e) for e in ((result or {}).get("evidence") or [])],
        }
    return (result or {}).get("snapshot"), None


def _estop(link: Any) -> Dict[str, Any]:
    """急停（L4）——异常时的兜底动作。"""
    try:
        result = link.stop()
        return {"ok": bool(isinstance(result, dict) and result.get("status") == "ok"),
                "detail": (result or {}).get("reason") or ""}
    except Exception as exc:  # pragma: no cover - 兜底路径
        return {"ok": False, "detail": repr(exc)}


def _execute_move(link: Any, target: List[float], channel: str, speed_pct: int,
                  origin: List[float], joint_index: int,
                  timeout_s: float = MOVE_TIMEOUT_S) -> Tuple[bool, str, Dict[str, Any]]:
    """执行一次微动。返回 ``(ok, detail, data)``。

    两条通道都只用**关节角**（位姿类接口在黑名单里）：

    * ``movej`` —— ``rm_movej`` 关节空间规划运动（阻塞到到位）；
    * ``canfd`` —— ``rm_movej_canfd`` 关节角透传（本项目 10 ms 单时钟主通道，
      非阻塞 → 轮询回读判定到位）。
    """
    data: Dict[str, Any] = {"channel": channel, "target_deg": list(target)}
    if channel == "canfd":
        t0 = time.perf_counter()
        result = link.transmit_joint(target, follow=False, trajectory_mode=1, radio=30)
        ok = isinstance(result, dict) and result.get("status") == "ok"
        data["result"] = result
        data["elapsed_s"] = round(time.perf_counter() - t0, 3)
        if not ok:
            return False, "透传指令失败", data
        # 透传是非阻塞的 → 轮询回读直到到位
        deadline = time.perf_counter() + float(timeout_s)
        measured = list(origin)
        while True:
            snap, _fail = _snapshot(link)
            if snap is None:
                return False, "透传后回读失败", data
            measured = [float(v) for v in (snap.get("joint_deg") or [])]
            verdict = evaluate_move(origin, measured, target, joint_index)
            if verdict["ok"]:
                data["verdict"] = verdict
                return True, ("透传到位（到位误差 %.3f°）" % verdict["arrival_error_deg"]), data
            if time.perf_counter() >= deadline:
                data["verdict"] = verdict
                return False, "透传 %.1fs 内未到位" % float(timeout_s), data
            time.sleep(0.05)
    # 默认：rm_movej（阻塞）
    t0 = time.perf_counter()
    result = link.preposition_movej(target, speed=int(speed_pct), radius=0, block=1)
    ok = isinstance(result, dict) and result.get("status") == "ok"
    data["result"] = result
    data["elapsed_s"] = round(time.perf_counter() - t0, 3)
    return ok, ("指令已下发" if ok else "指令失败"), data


# ------------------------------------------------------------------ 探针主体
def run_probe(arm: str = "left", ip: str = DEFAULT_IP, port: int = DEFAULT_PORT,
              udp_port: Optional[int] = DEFAULT_UDP_PORT,
              joint_index: int = 0, delta_deg: float = DEFAULT_DELTA_DEG,
              speed_pct: int = DEFAULT_SPEED_PCT, enable_realtime: bool = True,
              precheck_only: bool = False, dry_run: bool = False,
              channel: str = "movej", move_timeout_s: float = MOVE_TIMEOUT_S,
              ws: Optional[str] = None, lib: Any = None,
              link_factory=None) -> Dict[str, Any]:
    """跑一次安全探针，返回报告字典。

    ``link_factory`` 只在测试里注入假句柄；默认走真机连接。
    """
    started = _now_iso()
    endpoint = {"ip": ip, "port": int(port), "udp_port": udp_port}
    params = {
        "joint_index": joint_index,
        "delta_deg": float(delta_deg),
        "speed_pct": int(speed_pct),
        "channel": channel,
        "move_timeout_s": float(move_timeout_s),
        "enable_realtime": bool(enable_realtime),
        "precheck_only": bool(precheck_only),
        "limits": {"max_delta_deg": MAX_DELTA_DEG, "max_speed_pct": MAX_SPEED_PCT,
                   "collision_stage": COLLISION_STAGE,
                   "arrival_tol_deg": ARRIVAL_TOL_DEG,
                   "drift_tol_deg": DRIFT_TOL_DEG,
                   "return_tol_deg": RETURN_TOL_DEG},
    }

    # --- 0) 安全包络（0 次 SDK 调用）
    ok, reason = check_envelope(joint_index, delta_deg, speed_pct)
    if channel not in ("movej", "canfd"):
        reason = "channel=%r 非法（可选 movej / canfd）" % (channel,)
        ok = False
    if not ok:
        return _report(STATUS_ERROR, arm, endpoint, params,
                       [_phase("envelope", STATUS_ERROR, reason)],
                       [{"kind": "error", "detail": reason,
                         "next_action": "把 delta_deg 调到 ±%.1f° 以内、speed_pct 调到 %d%% 以内后重跑"
                                        % (MAX_DELTA_DEG, MAX_SPEED_PCT)}],
                       reason=reason,
                       next_action="缩小动作幅度后重跑（未发出任何 SDK 调用）",
                       started_at=started)

    if dry_run:
        limits = _load_limits(ws)
        phases = [
            _phase("envelope", STATUS_OK, "安全包络校验通过（未发出任何 SDK 调用）"),
            _phase("plan", STATUS_DRY_RUN,
                   "计划：关节%d 位移 %+.3f°，通道 %s%s，去程-回程对称"
                   % (joint_index + 1, float(delta_deg), channel,
                      "（速度 %d%%）" % int(speed_pct) if channel == "movej" else ""),
                   {"collision_stage": COLLISION_STAGE,
                    "target_deg": plan_target([0.0] * 7, joint_index, delta_deg)}),
        ]
        return _report(STATUS_DRY_RUN, arm, endpoint, params, phases,
                       [{"kind": "synthetic", "detail": "dry-run：未连接真机，无任何真机读数"}],
                       reason="dry-run（零 SDK 调用）",
                       next_action="去掉 --dry-run 后重跑真机路径",
                       mode="dry_run", started_at=started)

    # --- 1) 连接
    if link_factory is not None:
        link, failure = link_factory(arm=arm, ip=ip, port=port, udp_port=udp_port,
                                     enable_realtime=enable_realtime, ws=ws, lib=lib)
    else:
        link, failure = _connect_link(arm, ip, port, udp_port, enable_realtime, ws, lib)
    if failure is not None:
        return failure
    if link is None:
        return _report(STATUS_ERROR, arm, endpoint, params,
                       [_phase("connect", STATUS_ERROR, "未取得句柄")],
                       [{"kind": "error", "detail": "未取得句柄"}],
                       reason="未取得句柄", next_action="检查 SDK 与网络后重跑",
                       started_at=started)

    phases: List[Dict[str, Any]] = []
    evidence: List[Dict[str, Any]] = []
    limits = _load_limits(ws)
    origin: Optional[List[float]] = None
    estop_done = False

    try:
        # --- 2) 只读快照
        snap, snap_fail = _snapshot(link)
        if snap is None:
            phases.append(_phase("snapshot", STATUS_ERROR, snap_fail["reason"]))
            evidence.append({"kind": "error", "detail": snap_fail["reason"],
                             "next_action": snap_fail["next_action"]})
            estop = _estop(link)
            estop_done = True
            phases.append(_phase("estop", STATUS_OK if estop["ok"] else STATUS_ERROR,
                                 "急停已下发" if estop["ok"] else "急停下发失败",
                                 estop))
            return _report(STATUS_ERROR, arm, endpoint, params, phases, evidence,
                           reason=snap_fail["reason"],
                           next_action=snap_fail["next_action"], started_at=started)

        joints = [float(v) for v in (snap.get("joint_deg") or [])]
        origin = joints
        phases.append(_phase("snapshot", STATUS_OK,
                             "读到 7 个关节角 / 控制器 %s"
                             % (snap.get("controller_trajectory_type_name"),),
                             {"snapshot": snap}))

        # --- 3) 只读前置检查
        checks = preflight_checks(snap, limits, joint_index, float(delta_deg))
        failed = [c for c in checks if not c["ok"]]
        phases.append(_phase("preflight", STATUS_OK if not failed else STATUS_ERROR,
                             "前置检查 %d/%d 通过" % (len(checks) - len(failed), len(checks)),
                             {"checks": checks}))
        for c in failed:
            evidence.append({"kind": "error", "detail": "%s: %s" % (c["name"], c["detail"]),
                             "next_action": "处理该项后重跑（本次未发出任何运动指令）"})
        if failed:
            return _report(STATUS_ERROR, arm, endpoint, params, phases, evidence,
                           reason="前置检查未通过：%s" % "；".join(c["name"] for c in failed),
                           next_action="按 evidence 里的 next_action 处理后重跑（本次未运动）",
                           started_at=started)

        if precheck_only:
            phases.append(_phase("motion", STATUS_OK,
                                 "precheck-only：只读检查通过，**未发出任何运动指令**"))
            return _report(STATUS_OK, arm, endpoint, params, phases, evidence,
                           reason="只读前置检查全部通过",
                           next_action="去掉 --precheck-only 跑最小动作验证",
                           started_at=started)

        # --- 4) 武装安全开关（失败即中止，不运动）
        armed = link.set_safety(collision_stage=COLLISION_STAGE,
                                self_collision_enable=True)
        armed_ok = isinstance(armed, dict) and armed.get("status") == "ok"
        phases.append(_phase("arm_safety", STATUS_OK if armed_ok else STATUS_ERROR,
                             "碰撞检测等级 %d + 自碰撞检测使能 %s"
                             % (COLLISION_STAGE, "已下发" if armed_ok else "失败"),
                             {"result": armed}))
        if not armed_ok:
            evidence.append({"kind": "error",
                             "detail": "安全开关下发失败：%s" % (armed or {}).get("reason", ""),
                             "next_action": "确认控制器在线后重跑（本次未运动）"})
            return _report(STATUS_ERROR, arm, endpoint, params, phases, evidence,
                           reason="安全开关下发失败（未运动）",
                           next_action="确认控制器在线后重跑", started_at=started)

        # --- 5) 去程（单关节微动）
        target = plan_target(origin, joint_index, float(delta_deg))
        move_ok, move_detail, move_data = _execute_move(
            link, target, channel, int(speed_pct), origin, joint_index,
            timeout_s=move_timeout_s)
        phases.append(_phase("move_out", STATUS_OK if move_ok else STATUS_ERROR,
                             "去程指令（%s 通道）%s"
                             % (channel, move_detail),
                             move_data))
        if not move_ok:
            estop = _estop(link)
            estop_done = True
            phases.append(_phase("estop", STATUS_OK if estop["ok"] else STATUS_ERROR,
                                 "急停已下发" if estop["ok"] else "急停下发失败", estop))
            evidence.append({"kind": "error",
                             "detail": "去程指令失败：%s" % move_detail,
                             "next_action": "检查控制器状态后人工确认臂位姿"})
            return _report(STATUS_ERROR, arm, endpoint, params, phases, evidence,
                           reason="去程指令失败（已急停）",
                           next_action="人工确认臂位姿后再重跑", started_at=started)

        snap2, fail2 = _snapshot(link)
        if snap2 is None:
            estop = _estop(link)
            estop_done = True
            phases.append(_phase("estop", STATUS_OK if estop["ok"] else STATUS_ERROR,
                                 "急停已下发" if estop["ok"] else "急停下发失败", estop))
            evidence.append({"kind": "error", "detail": "去程后回读失败：%s" % fail2["reason"]})
            return _report(STATUS_ERROR, arm, endpoint, params, phases, evidence,
                           reason="去程后回读失败（已急停）",
                           next_action="人工确认臂位姿后再重跑", started_at=started)

        measured = [float(v) for v in (snap2.get("joint_deg") or [])]
        verdict = evaluate_move(origin, measured, target, joint_index)
        phases.append(_phase("verify_out", STATUS_OK if verdict["ok"] else STATUS_ERROR,
                             "到位误差 %.3f°（容差 %.1f°），其它关节最大漂移 %.3f°（容差 %.1f°）"
                             % (verdict["arrival_error_deg"], ARRIVAL_TOL_DEG,
                                verdict["max_drift_deg"], DRIFT_TOL_DEG),
                             verdict))
        if not verdict["ok"]:
            estop = _estop(link)
            estop_done = True
            phases.append(_phase("estop", STATUS_OK if estop["ok"] else STATUS_ERROR,
                                 "急停已下发" if estop["ok"] else "急停下发失败", estop))
            evidence.append({"kind": "error", "detail": "去程到位判定未通过",
                             "next_action": "人工确认臂位姿"})
            return _report(STATUS_ERROR, arm, endpoint, params, phases, evidence,
                           reason="去程到位判定未通过（已急停）",
                           next_action="人工确认臂位姿后再重跑", started_at=started)

        # --- 6) 回程（回到初始位姿）
        back_ok, back_detail, back_data = _execute_move(
            link, origin, channel, int(speed_pct), origin, joint_index,
            timeout_s=move_timeout_s)
        phases.append(_phase("move_back", STATUS_OK if back_ok else STATUS_ERROR,
                             "回程指令（%s 通道）%s" % (channel, back_detail),
                             back_data))
        if not back_ok:
            estop = _estop(link)
            estop_done = True
            phases.append(_phase("estop", STATUS_OK if estop["ok"] else STATUS_ERROR,
                                 "急停已下发" if estop["ok"] else "急停下发失败", estop))
            evidence.append({"kind": "error", "detail": "回程指令失败：%s" % back_detail,
                             "next_action": "人工确认臂位姿"})
            return _report(STATUS_ERROR, arm, endpoint, params, phases, evidence,
                           reason="回程指令失败（已急停）",
                           next_action="人工确认臂位姿后再重跑", started_at=started)

        snap3, fail3 = _snapshot(link)
        if snap3 is None:
            estop = _estop(link)
            estop_done = True
            phases.append(_phase("estop", STATUS_OK if estop["ok"] else STATUS_ERROR,
                                 "急停已下发" if estop["ok"] else "急停下发失败", estop))
            evidence.append({"kind": "error", "detail": "回程后回读失败：%s" % fail3["reason"]})
            return _report(STATUS_ERROR, arm, endpoint, params, phases, evidence,
                           reason="回程后回读失败（已急停）",
                           next_action="人工确认臂位姿后再重跑", started_at=started)

        measured3 = [float(v) for v in (snap3.get("joint_deg") or [])]
        back_verdict = evaluate_return(origin, measured3)
        phases.append(_phase("verify_back", STATUS_OK if back_verdict["ok"] else STATUS_ERROR,
                             "回程最大偏差 %.3f°（容差 %.1f°，关节%s）"
                             % (back_verdict["max_return_error_deg"], RETURN_TOL_DEG,
                                back_verdict["worst_joint"]),
                             back_verdict))
        if not back_verdict["ok"]:
            estop = _estop(link)
            estop_done = True
            phases.append(_phase("estop", STATUS_OK if estop["ok"] else STATUS_ERROR,
                                 "急停已下发" if estop["ok"] else "急停下发失败", estop))
            evidence.append({"kind": "error", "detail": "回程到位判定未通过",
                             "next_action": "人工确认臂位姿"})
            return _report(STATUS_ERROR, arm, endpoint, params, phases, evidence,
                           reason="回程到位判定未通过（已急停）",
                           next_action="人工确认臂位姿后再重跑", started_at=started)

        evidence.append({"kind": "ok",
                         "detail": "关节角透传通道可用：关节%d 去程 %.3f°→%.3f°，回程回到原位"
                                   % (joint_index + 1, origin[joint_index],
                                      measured[joint_index]),
                         "next_action": "如需更大动作，另开窗口并重新评估安全包络"})
        return _report(STATUS_OK, arm, endpoint, params, phases, evidence,
                       reason="控制链路验证通过（最小动作）",
                       next_action="可进入下一阶段：扩大基线采集（如 rmctl.hwcollect）",
                       started_at=started)
    finally:
        try:
            link.close()
        except Exception:  # pragma: no cover - 释放失败不影响结论
            pass


# ------------------------------------------------------------------ CLI
def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="rm75_safe_probe",
        description="RM75-B 真机最小动作安全探针（单关节亚度级 + 低速 + 去程回程）")
    p.add_argument("--arm", choices=["left", "right"], default="left")
    p.add_argument("--ip", default=DEFAULT_IP, help="控制器 IP（默认 %s）" % DEFAULT_IP)
    p.add_argument("--port", type=int, default=DEFAULT_PORT)
    p.add_argument("--udp-port", dest="udp_port", type=int, default=DEFAULT_UDP_PORT)
    p.add_argument("--joint", dest="joint_index", type=int, default=0,
                   help="要微动的关节序号（0 基，默认 0 = J1）")
    p.add_argument("--delta-deg", dest="delta_deg", type=float, default=DEFAULT_DELTA_DEG,
                   help="微动角度（度，默认 %.2f，硬上限 %.1f）"
                        % (DEFAULT_DELTA_DEG, MAX_DELTA_DEG))
    p.add_argument("--speed-pct", dest="speed_pct", type=int, default=DEFAULT_SPEED_PCT,
                   help="rm_movej 速度百分比（默认 %d，硬上限 %d）"
                        % (DEFAULT_SPEED_PCT, MAX_SPEED_PCT))
    p.add_argument("--channel", choices=["movej", "canfd"], default="movej",
                   help="运动通道：movej=rm_movej 关节规划运动；"
                        "canfd=rm_movej_canfd 关节角透传（10 ms 单时钟主通道）")
    p.add_argument("--no-realtime", dest="enable_realtime", action="store_false",
                   help="不开 UDP 主动上报（默认开，用于顺带验证反馈通道）")
    p.add_argument("--precheck-only", dest="precheck_only", action="store_true",
                   help="只做只读检查，不发出任何运动指令")
    p.add_argument("--dry-run", dest="dry_run", action="store_true",
                   help="离线预演（零 SDK 调用）")
    p.add_argument("--ws", default=None, help="工作区根目录（默认自动发现）")
    p.add_argument("--out", default=None, help="报告 JSON 输出路径")
    p.add_argument("--json", action="store_true", help="stdout 只打印 JSON")
    return p


def main(argv: Optional[List[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        report = run_probe(arm=args.arm, ip=args.ip, port=args.port,
                           udp_port=args.udp_port, joint_index=args.joint_index,
                           delta_deg=args.delta_deg, speed_pct=args.speed_pct,
                           enable_realtime=args.enable_realtime,
                           precheck_only=args.precheck_only, dry_run=args.dry_run,
                           channel=args.channel, ws=args.ws)
    except ProbeInputError as exc:
        sys.stderr.write("参数非法: %s\n" % exc)
        return EXIT_USAGE

    if args.out:
        out_dir = os.path.dirname(os.path.abspath(args.out))
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)
        with open(args.out, "w", encoding="utf-8") as fh:
            json.dump(report, fh, ensure_ascii=False, indent=2)

    if args.json:
        print(json.dumps(report, ensure_ascii=False, indent=2))
    else:
        _print_summary(report)
        if args.out:
            print("\n报告已写入: %s" % args.out)

    status = report.get("status")
    if status == STATUS_OK or status == STATUS_DRY_RUN:
        return EXIT_OK
    if status == STATUS_BLOCKED:
        return EXIT_BLOCKED
    return EXIT_ERROR


def _print_summary(report: Dict[str, Any]) -> None:
    print("RM75-B 安全探针 | 状态=%s | 臂=%s | 端点=%s:%s"
          % (report.get("status"), report.get("arm"),
             report.get("endpoint", {}).get("ip"), report.get("endpoint", {}).get("port")))
    for phase in report.get("phases", []):
        print("  [%s] %-12s %s" % ("PASS" if phase.get("status") in ("ok", "dry_run")
                                   else phase.get("status"),
                                   phase.get("name"), phase.get("detail")))
        for check in (phase.get("data") or {}).get("checks", []) or []:
            print("        - [%s] %s: %s" % ("PASS" if check.get("ok") else "FAIL",
                                            check.get("name"), check.get("detail")))
    if report.get("reason"):
        print("  原因: %s" % report["reason"])
    if report.get("next_action"):
        print("  下一步: %s" % report["next_action"])


if __name__ == "__main__":
    sys.exit(main())
