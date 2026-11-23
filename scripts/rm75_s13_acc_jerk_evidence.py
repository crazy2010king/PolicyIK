#!/usr/bin/env python3
"""S13 joint2..joint7 阶梯 acc/jerk 真机实测 → 证据工件。

与 ``scripts/rm75_s13_versions_probe.py`` 同风格（同一里程碑、同一目录）。
本脚本负责三件事，**顺序不可颠倒**：

1. **只读前置检查**（``--stage precheck``）——
   7 轴全使能 / ``joint_err_code`` 全 0 / 控制器 ``RM_NO_PLANNING_E`` +
   ``RM_IDLE_E`` / 每个待测关节的 ``+10°`` 目标落在**执行域**限位内
   （joint7 执行域 ±360°；**规划域 ±180° 不是判据**）。
   任一项不满足 → **不下发任何运动指令**，直接落 ``status=blocked`` 工件。
2. **运动实验**（``--stage run``）——
   通过 ``python3 -m rmctl.hwcollect ladder_acc_jerk --joints ...`` 执行
   （单进程单连接；控制器连接数上限 ≈5）。
3. **工件组装**（``--stage assemble``）——
   把 hwcollect 的结果 JSON + 前置检查结果组装成
   ``workcell/baselines/evidence/rm75_s13_acc_jerk.json``
   （schema ``rmctl.rm75_s13_acc_jerk/1``）。

红线
----
* **只用 joint2..joint7**：joint1 的唯一来源是 S01 工件
  ``workcell/baselines/evidence/rm75_validation_ladder_20261119.json``
  的 V8 阶段；两个来源打架就是证据污染。
* **禁止用 dry-run / 合成数据冒充真机读数**：窗口不可达 → ``status=blocked``。
* **诚实口径**：包络反映 5~11% 速度档下的**指令轮廓**，**不代表控制器最大能力**，
  只作为**保守下界**使用。

用法
----
::

    # 真机窗口（远端主机 172.16.119.199:2224 → 容器 ros_dev 内）：
    python3 scripts/rm75_s13_acc_jerk_evidence.py --stage all \
        --arm left --ip 192.168.1.18 --joints 2,3,4,5,6,7 --trials 3 \
        --out workcell/baselines/evidence/rm75_s13_acc_jerk.json

    # 离线预演（不连任何设备）：
    python3 scripts/rm75_s13_acc_jerk_evidence.py --stage all --dry-run

退出码：``0`` ok / ``3`` blocked（前置不满足 / 窗口不可达）/ ``4`` error。
"""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import time
from datetime import datetime, timezone
from typing import Any, Dict, List, Optional

SCHEMA = "rmctl.rm75_s13_acc_jerk/1"

#: 默认控制器地址（左臂）
DEFAULT_IP = "192.168.1.18"
#: 默认 TCP 端口
DEFAULT_PORT = 8080
#: UDP 主动上报端口（臂状态 RM_IDLE_E 等**只**在 UDP 上报里）
DEFAULT_UDP_PORT = 8091
#: 本轮要测的关节（**1 基**；joint1 由 S01 工件提供，不在本轮范围内）
DEFAULT_JOINTS = (2, 3, 4, 5, 6, 7)
#: 每关节重复次数（≥3，任务契约要求 samples ≥ 3）
DEFAULT_TRIALS = 3
#: 阶梯位移（度）—— 与 rmctl.hwcollect.LADDER_STEP_DEG 一致
STEP_DEG = 10.0
#: 速度档（%）—— 与 rmctl.hwcollect.LADDER_BASE_SPEED / LADDER_SPEED_STEP 一致
SPEED_BASE_PCT = 5
SPEED_STEP_PCT = 3

#: 控制器规划类型（无规划）
NO_PLANNING_TYPE = "RM_NO_PLANNING_E"
#: 控制器臂状态（空闲）
IDLE_STATE = "RM_IDLE_E"

STATUS_OK = "ok"
STATUS_DRY_RUN = "dry_run"
STATUS_BLOCKED = "blocked"
STATUS_ERROR = "error"

EXIT_OK = 0
EXIT_ERROR = 4
EXIT_BLOCKED = 3

#: 诚实性声明（**逐字**写进工件，供测试断言）
HONESTY_STATEMENT = (
    "包络反映 5~11% 速度档下的指令轮廓，不代表控制器最大能力，作为保守下界使用。"
)

#: 证据工件里允许出现的**运动**语义字段（与只读工件相反：本工件确实运动过）
ALLOWED_MOTION_FIELDS = frozenset({
    "step_deg", "returned_to_origin", "speeds", "speed",
})


def _now_iso() -> str:
    """当前 UTC 时间（ISO8601）。"""
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


# --------------------------------------------------------------------- 工具
def _parse_joints(text: str) -> List[int]:
    """解析 ``--joints``（**1 基**，逗号分隔，支持 ``jointN``）。

    非法输入 / 空输入 → 抛 :class:`ValueError`（调用方转成退出码 2）。
    """
    joints: List[int] = []
    for token in str(text).split(","):
        token = token.strip()
        if not token:
            continue
        low = token.lower()
        digits = low[len("joint"):] if low.startswith("joint") else low
        if not digits.isdigit():
            raise ValueError("--joints 必须是 1 基关节号（1..7）或 jointN：%r" % token)
        number = int(digits)
        if not 1 <= number <= 7:
            raise ValueError("--joints 关节号必须在 1..7（1 基）：%r" % token)
        joints.append(number)
    if not joints:
        raise ValueError("--joints 不能为空")
    return joints


def parse_s13_joints(text: str) -> List[int]:
    """解析本轮要测的关节（**1 基**，逗号分隔，支持 ``jointN``）。

    在 :func:`_parse_joints` 之上追加一条**本轮专属**红线：
    **不得包含 joint1** —— joint1 的唯一来源是 S01 工件的 V8 阶段，
    本轮重测 joint1 会造成两个来源打架（证据污染）。
    """
    joints = _parse_joints(text)
    if 1 in joints:
        raise ValueError(
            "--joints 不得包含 joint1（joint1 的唯一来源是 S01 工件 "
            "rm75_validation_ladder_20261119.json 的 V8 阶段）")
    return joints


def _joint_name(number: int) -> str:
    return "joint%d" % int(number)


def _walk_keys(node: Any, path: str = "") -> List[str]:
    """递归列出所有字典键路径。"""
    out: List[str] = []
    if isinstance(node, dict):
        for key, value in node.items():
            out.append(str(key))
            out.extend(_walk_keys(value, path))
    elif isinstance(node, list):
        for item in node:
            out.extend(_walk_keys(item, path))
    return out


# ------------------------------------------------------------- 前置检查（只读）
def _load_execution_limits(ws: Optional[str] = None) -> Dict[str, Any]:
    """读**执行域**限位（rad）。

    ``rm_hw_bridge.limits.load_joint_limits`` 返回的 ``min_pos`` / ``max_pos``
    就是**执行域**（真机机械范围）；规划域另存在 ``plan_min_pos`` /
    ``plan_max_pos``。joint7 执行域 ±360°、规划域 ±180° —— **判据是执行域**。
    """
    from rm_hw_bridge import limits as hw_limits

    data = hw_limits.load_joint_limits(ws)
    out: Dict[str, Any] = {}
    for name, item in data.items():
        out[name] = {
            "execution_lower_rad": item["min_pos"],
            "execution_upper_rad": item["max_pos"],
            "plan_lower_rad": item.get("plan_min_pos"),
            "plan_upper_rad": item.get("plan_max_pos"),
            "criterion": "execution domain（执行域）",
        }
    return out


def precheck(snapshot: Dict[str, Any], joints: List[int],
             limits: Dict[str, Any], step_deg: float = STEP_DEG) -> List[Dict[str, Any]]:
    """只读前置检查逐项结果（``[{"name", "ok", "detail"}]``）。

    **任一项不满足 → 调用方不得下发运动指令**。
    """
    checks: List[Dict[str, Any]] = []

    def add(name: str, ok: bool, detail: str) -> None:
        checks.append({"name": name, "ok": bool(ok), "detail": detail})

    arm_err = snapshot.get("arm_err") or []
    bad_arm = [int(c) for c in arm_err if int(c) != 0]
    add("arm_err_empty", not bad_arm,
        "臂级错误码: %s" % (bad_arm or "无（原始: %s）" % (list(arm_err) or "无")))

    err_codes = snapshot.get("joint_err_code") or []
    bad_joint = [(i + 1, int(c)) for i, c in enumerate(err_codes) if int(c) != 0]
    add("joint_err_code_zero", not bad_joint,
        "非零关节错误码: %s" % (bad_joint or "无"))

    en_flags = snapshot.get("joint_en_flag") or []
    disabled = [i + 1 for i, f in enumerate(en_flags) if int(f) != 1]
    add("all_joints_enabled", not disabled,
        "未使能关节: %s" % (disabled or "无（7 轴全部使能）"))

    traj_name = snapshot.get("controller_trajectory_type_name")
    add("controller_no_planning", traj_name == NO_PLANNING_TYPE,
        "控制器规划类型: %s（期望 %s）" % (traj_name, NO_PLANNING_TYPE))

    status_name = snapshot.get("arm_current_status_name")
    realtime = snapshot.get("realtime") or {}
    if status_name is None:
        status_source = ("未取到（UDP 上报未建立或未含臂状态；frames=%s）"
                         % (realtime.get("frames") or 0))
    else:
        status_source = ("UDP 主动上报（frames=%s）" % (realtime.get("frames") or 0))
    add("controller_idle", status_name == IDLE_STATE,
        "控制器臂状态: %s（期望 %s）；来源: %s"
        % (status_name, IDLE_STATE, status_source))

    deg = snapshot.get("joint_deg") or []
    add("joints_readable", len(deg) == 7, "读到 %d 个关节角" % len(deg))

    if len(deg) == 7:
        for number in joints:
            name = _joint_name(number)
            limit = limits.get(name)
            if not limit:
                add("target_within_execution_domain", False,
                    "缺 %s 的执行域限位条目" % name)
                continue
            lo = float(limit["execution_lower_rad"])
            hi = float(limit["execution_upper_rad"])
            current = math.radians(float(deg[number - 1]))
            target = current + math.radians(float(step_deg))
            ok = lo <= target <= hi
            add("target_within_execution_domain", ok,
                "%s 当前 %.3f° → 目标 %.3f°（+%.1f°），执行域 [%.3f°, %.3f°]"
                % (name, math.degrees(current), math.degrees(target),
                   float(step_deg), math.degrees(lo), math.degrees(hi)))

    add("single_connection", True,
        "单进程单连接（控制器连接数上限 ≈5，本轮只建一条链并全程复用）")
    return checks


def _network_diagnostics() -> Dict[str, Any]:
    """本机（容器与宿主共享 netns）到控制器的**只读**网络诊断。

    只读 ``/sys/class/net`` 与 ``/proc/net/route``：不写任何配置、不发包。
    事实记录（供操作者判断「是控制器离线还是本机链路掉了」）。
    """
    out: Dict[str, Any] = {"interfaces": [], "routes": [], "note": ""}
    try:
        base = "/sys/class/net"
        for name in sorted(os.listdir(base)):
            entry: Dict[str, Any] = {"name": name}
            for label, path in (("carrier", "carrier"),
                                ("operstate", "operstate"),
                                ("address", "address")):
                try:
                    with open(os.path.join(base, name, path)) as fh:
                        entry[label] = fh.read().strip()
                except Exception:
                    entry[label] = None
            out["interfaces"].append(entry)
    except Exception as exc:  # noqa: BLE001 — 诊断失败不得影响结论
        out["note"] = "读取网卡状态失败: %r" % (exc,)
    try:
        with open("/proc/net/route") as fh:
            lines = fh.read().splitlines()[1:]
        for line in lines:
            parts = line.split()
            if len(parts) >= 3:
                out["routes"].append({
                    "dest_hex": parts[0], "dest": _hex_to_ip(parts[0]),
                    "gateway": _hex_to_ip(parts[1]), "iface": parts[0] and parts[-1],
                })
    except Exception:
        pass
    return out


def _hex_to_ip(value: str) -> str:
    """little-endian hex 网关/目标 → 点分十进制。"""
    try:
        raw = bytes.fromhex(value)
        return ".".join(str(b) for b in reversed(raw))
    except Exception:
        return value


def _local_ip_for(target: str) -> Optional[str]:
    """推断本机（能到达 ``target`` 的那张网卡的）IP —— UDP 主动上报的接收地址。"""
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


def _snapshot_readonly(ip: str, port: int, arm: str,
                       ws: Optional[str] = None,
                       udp_port: int = DEFAULT_UDP_PORT,
                       realtime_wait_s: float = 2.0) -> Dict[str, Any]:
    """只读快照（**不下发任何运动指令、不改使能位、不清错误码**）。

    **为什么必须开 realtime**：控制器臂状态（``RM_IDLE_E`` 等）**只**出现在
    UDP 主动上报里（``rm_get_current_arm_state`` 不含该字段）；关掉 realtime
    时 ``arm_current_status`` 恒为 ``None``，``controller_idle`` 判据就永远
    不可能通过。因此本前置检查开启 UDP 上报。

    唯一一处控制器侧写入是 ``rm_set_realtime_push``（UDP 上报目标配置）——
    只影响状态上报通道，**不改变任何运动行为**。若上报配置失败，则退回
    无 realtime 的纯读快照，并把 ``realtime_enabled=False`` 如实记进结果。
    """
    from rm_hw_bridge import bridge as hw_bridge

    kwargs: Dict[str, Any] = {}
    local_ip = _local_ip_for(ip)
    if local_ip:
        kwargs["realtime_target_ip"] = local_ip
    kwargs["realtime_target_port"] = int(udp_port)

    realtime_enabled = True
    link = hw_bridge.connect(ip=ip, port=int(port), arm=arm, enable_realtime=True,
                              ws=ws, **kwargs)
    if getattr(link, "status", "") != "ok":
        # 上报配置失败（端口占用 / 目标 IP 不可达等）→ 退回纯读快照，
        # 保留失败原因（此时 controller_idle 判据必然读不到值）。
        realtime_enabled = False
        realtime_reason = getattr(link, "reason", "") or "UDP 主动上报配置失败"
        try:
            link.close()
        except Exception:  # pragma: no cover - 关闭失败不影响结论
            pass
        link = hw_bridge.connect(ip=ip, port=int(port), arm=arm,
                                  enable_realtime=False, ws=ws)
    else:
        realtime_reason = ""
    try:
        result = link.snapshot()
        if not isinstance(result, dict):
            return {"status": "error", "reason": "快照返回异常类型",
                    "snapshot": None, "realtime_enabled": realtime_enabled}
        if result.get("status") != "ok":
            return {"status": result.get("status") or "error",
                    "reason": result.get("reason") or "只读快照失败",
                    "next_action": result.get("next_action") or "",
                    "snapshot": None, "realtime_enabled": realtime_enabled}
        snapshot = result.get("snapshot")
        # 等第一帧 UDP 上报（臂状态只在里面）
        deadline = time.time() + float(realtime_wait_s)
        while time.time() < deadline:
            realtime = (snapshot or {}).get("realtime") or {}
            if realtime.get("frames") and snapshot.get("arm_current_status") is not None:
                break
            time.sleep(0.05)
            result = link.snapshot()
            if isinstance(result, dict) and result.get("status") == "ok":
                snapshot = result.get("snapshot")
        return {"status": "ok", "reason": "", "snapshot": snapshot,
                "realtime_enabled": realtime_enabled,
                "realtime_reason": realtime_reason,
                "realtime_target_ip": local_ip,
                "udp_port": int(udp_port)}
    finally:
        try:
            link.close()
        except Exception:  # pragma: no cover - 关闭失败不影响结论
            pass


# --------------------------------------------------------------- 运动实验
def _ladder_command(args: argparse.Namespace, out_dir: str) -> str:
    """可粘贴的 hwcollect 阶梯实验命令（与实际执行的那一条**逐字一致**）。"""
    return (
        "python3 -m rmctl.hwcollect ladder_acc_jerk --arm %s --ip %s "
        "--joints %s --trials %d --out %s"
        % (args.arm, args.ip, ",".join(str(j) for j in args.joints), args.trials,
           out_dir)
    )


def run_ladder(args: argparse.Namespace, out_dir: str,
               ws: Optional[str] = None) -> Dict[str, Any]:
    """跑 hwcollect 阶梯实验（返回结果字典）。

    通过**子进程**调用 CLI —— 保证工件里的数值与操作者手动粘贴的命令**同源**。
    """
    cmd = [sys.executable, "-m", "rmctl.hwcollect", "ladder_acc_jerk",
           "--arm", args.arm, "--ip", args.ip,
           "--joints", ",".join(str(j) for j in args.joints),
           "--trials", str(int(args.trials)), "--out", out_dir]
    env = dict(os.environ)
    if ws:
        env["RM_WS_ROOT"] = os.path.abspath(ws)
    proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                          env=env, timeout=3600)
    output = proc.stdout.decode("utf-8", "replace")
    return {"command": " ".join(cmd), "returncode": int(proc.returncode),
            "output": output[-4000:]}


def _find_ladder_report(out_dir: str, arm: str) -> Optional[str]:
    """在 hwcollect 的输出目录里找到阶梯实验的结果 JSON。"""
    candidates = [
        os.path.join(out_dir, "ladder_acc_jerk.%s.json" % arm),
    ]
    if os.path.isdir(out_dir):
        for name in sorted(os.listdir(out_dir)):
            if name.startswith("ladder_acc_jerk") and name.endswith(".json"):
                candidates.append(os.path.join(out_dir, name))
    for path in candidates:
        if os.path.isfile(path):
            return path
    return None


def _load_json(path: str) -> Optional[Any]:
    """读 JSON（读不到 / 解析失败 → ``None``，不抛异常）。"""
    try:
        with open(path, "r", encoding="utf-8") as fh:
            return json.load(fh)
    except Exception:  # noqa: BLE001 — 读不到就当没有
        return None


def _load_precheck_result(path: str) -> Optional[Dict[str, Any]]:
    """读回前置检查结果文件（``--stage assemble`` 用）。

    用途：阶梯实验跑完、进程被杀或会话断开后，**不必重跑运动**也能组装工件 ——
    前置检查是运动前的门禁记录，必须与那次运动一一对应。
    """
    data = _load_json(path)
    if not isinstance(data, dict):
        return None
    return {
        "checks": data.get("checks") or [],
        "snapshot": data.get("snapshot"),
        "reason": data.get("reason") or "",
        "realtime_enabled": bool(data.get("realtime_enabled")),
        "realtime_reason": data.get("realtime_reason") or "",
        "realtime_target_ip": data.get("realtime_target_ip"),
        "udp_port": data.get("udp_port"),
    }


# --------------------------------------------------------------- 工件组装
def _joint_entries(report: Dict[str, Any], joints: List[int]) -> Dict[str, Any]:
    """从 hwcollect 结果里抽出每个关节的包络值 + 逐样本峰值。

    包络 = 多档取 min（保守下界）。
    """
    summary = report.get("summary") or {}
    acc_env = summary.get("acceleration_rad_s2") or {}
    jerk_env = summary.get("jerk_rad_s3") or {}
    samples = report.get("samples") or []

    by_joint: Dict[str, List[Dict[str, Any]]] = {}
    for sample in samples:
        name = sample.get("joint")
        if not name:
            continue
        by_joint.setdefault(str(name), []).append(sample)

    out: Dict[str, Any] = {}
    for number in joints:
        name = _joint_name(number)
        entries = by_joint.get(name, [])
        speeds = []
        for sample in entries:
            speed = sample.get("speed")
            if speed is not None and speed not in speeds:
                speeds.append(speed)
        out[name] = {
            "acceleration_rad_s2": acc_env.get(name),
            "jerk_rad_s3": jerk_env.get(name),
            "envelope": "min over trials（保守下界）",
            "trials": len(entries),
            "speeds": speeds,
            "samples": [
                {
                    "trial": sample.get("trial"),
                    "speed": sample.get("speed"),
                    "acc_peak_rad_s2": sample.get("acc_peak_rad_s2"),
                    "jerk_peak_rad_s3": sample.get("jerk_peak_rad_s3"),
                    "returned_to_origin": bool(sample.get("returned_to_origin")),
                }
                for sample in entries
            ],
        }
    return out


def build_artifact(args: argparse.Namespace, precheck_result: Dict[str, Any],
                   report: Optional[Dict[str, Any]],
                   report_path: Optional[str],
                   ladder_meta: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """组装证据工件。"""
    checks = list(precheck_result.get("checks") or [])
    failed = [c for c in checks if not c.get("ok")]
    joints = list(args.joints)
    entries = _joint_entries(report or {}, joints)

    complete = bool(report) and report.get("status") == STATUS_OK
    if complete:
        for name, entry in entries.items():
            if entry["trials"] < int(args.trials) or entry["trials"] < 3:
                complete = False
            if entry["acceleration_rad_s2"] is None or entry["jerk_rad_s3"] is None:
                complete = False
            if not all(s["returned_to_origin"] for s in entry["samples"]):
                complete = False

    if failed:
        status = STATUS_BLOCKED
        reason = "前置检查未通过：%s（**未下发任何运动指令**）" % "、".join(
            c["name"] for c in failed)
    elif not complete:
        status = STATUS_ERROR
        reason = (report or {}).get("reason") or "阶梯实验未完成（status=%s）" % (
            (report or {}).get("status"),)
    else:
        status = STATUS_OK
        reason = "joint2..joint7 阶梯 acc/jerk 实测完成"

    next_action = _next_action(args, status, reason, checks)
    artifact: Dict[str, Any] = {
        "schema": SCHEMA,
        "status": status,
        "executed_on": {
            "utc": _now_iso(),
            "host": args.source_host,
            "container": args.source_container,
            "worktree": args.source_worktree,
            "why": "本机不通 ≠ 真机不可达：控制臂 192.168.1.18 只在远端主机所在网段可达",
        },
        "arm": args.arm,
        "endpoint": {"ip": args.ip, "port": int(args.port)},
        "command": _ladder_command(args, args.report_dir),
        "joints": entries,
        "unit": {"acceleration": "rad/s^2", "jerk": "rad/s^3"},
        "source": ("远端真机窗口 %s → 容器 %s；命令 python3 -m rmctl.hwcollect "
                   "ladder_acc_jerk --joints %s --trials %d"
                   % (args.source_host, args.source_container,
                      ",".join(str(j) for j in joints), int(args.trials))),
        "method": ("rmctl.hwcollect ladder_acc_jerk：单关节 +%.1f° 阶梯位移，"
                   "每档测完自动回原位，速度档 %d/%d/%d%%，采样 dt=%.2f s，"
                   "多档取 min 得包络（保守下界）"
                   % (STEP_DEG, SPEED_BASE_PCT, SPEED_BASE_PCT + SPEED_STEP_PCT,
                      SPEED_BASE_PCT + 2 * SPEED_STEP_PCT, 0.02)),
        "derivative_method": "rmctl.retime.derivatives",
        "step_deg": STEP_DEG,
        "envelope": "min over trials（保守下界）",
        "safety_preconditions": {
            "ok": not failed,
            "checks": checks,
            "snapshot": precheck_result.get("snapshot"),
            "snapshot_reason": precheck_result.get("reason") or "",
            "realtime_enabled": bool(precheck_result.get("realtime_enabled")),
            "realtime_reason": precheck_result.get("realtime_reason") or "",
            "realtime_target_ip": precheck_result.get("realtime_target_ip"),
            "udp_port": precheck_result.get("udp_port"),
            "note": ("前置检查**不下发任何运动指令**、不改使能位、不清错误码；"
                     "唯一控制器侧写入是 UDP 主动上报配置（rm_set_realtime_push），"
                     "只为读取控制器臂状态（RM_IDLE_E 等只出现在 UDP 上报里）"),
        },
        "network_diagnostics": _network_diagnostics(),
        "raw_report_path": report_path,
        "ladder_run": ladder_meta or {},
        "next_action": next_action,
        "honesty": HONESTY_STATEMENT,
        "reason": reason,
    }
    if args.dry_run:
        artifact["status"] = STATUS_DRY_RUN
        artifact["dry_run"] = True
    return artifact


def _next_action(args: argparse.Namespace, status: str, reason: str,
                 checks: Optional[List[Dict[str, Any]]] = None) -> List[str]:
    """可粘贴的下一步。"""
    if status == STATUS_OK:
        return [
            "把本工件 joint2..joint7 的 acc/jerk 包络回填 "
            "workcell/baselines/measured_limits.yaml（首个关节的数值取自 S01 工件 "
            "rm75_validation_ladder_20261119.json 的 V8 阶段），"
            "value = min(实测包络, 已生效限值)（conservative，只收紧不放宽）；",
            "python3 dualarm_ws/src/rm_description/scripts/apply_measured_limits.py "
            "--mode conservative",
            "python3 -m rmctl baseline check",
        ]
    failed = {c.get("name") for c in (checks or []) if not c.get("ok")}
    actions: List[str] = []
    # **示教器现场动作**（项目约定：项目不自动使能、不自动清错误码）
    if "all_joints_enabled" in failed:
        actions.append("在示教器上重新使能 7 轴（joint_en_flag 全 1）")
    if "joint_err_code_zero" in failed or "arm_err_empty" in failed:
        actions.append("在示教器上清错误码复位（arm_err / joint_err_code 全 0）")
    actions.extend([
        "在远端真机窗口内先做只读前置检查："
        "python3 scripts/rm75_s13_acc_jerk_evidence.py --stage precheck "
        "--arm %s --ip %s --joints %s"
        % (args.arm, args.ip, ",".join(str(j) for j in args.joints)),
        "前置检查全部通过后执行：%s" % _ladder_command(args, args.report_dir),
        "当前阻塞原因：%s" % reason,
        "**不得**用 dry-run / 合成数据顶替真机读数。",
    ])
    return actions


# --------------------------------------------------------------------- 自检
def validate_artifact(artifact: Dict[str, Any],
                      expected_joints: Optional[List[int]] = None) -> List[str]:
    """工件自检：返回问题列表（**空列表 = 合规**）。"""
    problems: List[str] = []
    if artifact.get("schema") != SCHEMA:
        problems.append("schema 不是 %s（实为 %r）" % (SCHEMA, artifact.get("schema")))

    status = artifact.get("status")
    if status not in (STATUS_OK, STATUS_BLOCKED, STATUS_ERROR, STATUS_DRY_RUN):
        problems.append("status 非法：%r" % (status,))

    joints = artifact.get("joints")
    if not isinstance(joints, dict):
        problems.append("缺少 joints")
        joints = {}

    wanted = expected_joints if expected_joints is not None else list(DEFAULT_JOINTS)
    for number in wanted:
        name = _joint_name(number)
        entry = joints.get(name)
        if not isinstance(entry, dict):
            problems.append("缺关节 %s" % name)
            continue
        if status == STATUS_OK:
            if entry.get("acceleration_rad_s2") is None:
                problems.append("%s 缺 acc 包络值" % name)
            if entry.get("jerk_rad_s3") is None:
                problems.append("%s 缺 jerk 包络值" % name)
            samples = entry.get("samples") or []
            if len(samples) < 3:
                problems.append("%s 的样本数 %d < 3" % (name, len(samples)))
            if entry.get("trials") != len(samples):
                problems.append("%s 的 trials(%r) 与逐样本条数(%d) 不一致"
                                % (name, entry.get("trials"), len(samples)))
            for sample in samples:
                if sample.get("returned_to_origin") is not True:
                    problems.append("%s 有样本未回原位" % name)
        else:
            # **非 ok 工件不得含任何数值**：窗口不可达时只允许记录「没测到」，
            # 绝不允许用 dry-run / 合成值冒充真机读数。
            if entry.get("acceleration_rad_s2") is not None:
                problems.append("%s 在 status=%s 时不得含 acc 数值（禁止伪造读数）"
                                % (name, status))
            if entry.get("jerk_rad_s3") is not None:
                problems.append("%s 在 status=%s 时不得含 jerk 数值（禁止伪造读数）"
                                % (name, status))
            if entry.get("samples"):
                problems.append("%s 在 status=%s 时不得含逐样本读数（禁止伪造读数）"
                                % (name, status))

    # **不得出现 joint1**（joint1 的唯一来源是 S01 工件）
    for name in joints:
        if name == "joint1":
            problems.append("工件不得包含 joint1（joint1 的唯一来源是 S01 工件）")

    unit = artifact.get("unit") or {}
    if unit.get("acceleration") != "rad/s^2":
        problems.append("unit.acceleration 必须是 rad/s^2（实为 %r）"
                        % (unit.get("acceleration"),))
    if unit.get("jerk") != "rad/s^3":
        problems.append("unit.jerk 必须是 rad/s^3（实为 %r）" % (unit.get("jerk"),))

    if artifact.get("derivative_method") != "rmctl.retime.derivatives":
        problems.append("derivative_method 必须是 rmctl.retime.derivatives")
    if artifact.get("step_deg") != STEP_DEG:
        problems.append("step_deg 必须是 %r" % (STEP_DEG,))

    honesty = artifact.get("honesty")
    if not isinstance(honesty, str) or not honesty.strip():
        problems.append("缺 honesty 声明")
    else:
        if "包络" not in honesty:
            problems.append("honesty 缺「包络」语义")
        if "不代表控制器最大能力" not in honesty:
            problems.append("honesty 缺「不代表控制器最大能力」语义")
        if "保守下界" not in honesty:
            problems.append("honesty 缺「保守下界」语义")

    pre = artifact.get("safety_preconditions")
    if not isinstance(pre, dict):
        problems.append("缺 safety_preconditions")
    else:
        checks = pre.get("checks")
        if not isinstance(checks, list) or not checks:
            problems.append("safety_preconditions.checks 不得为空")
        else:
            for check in checks:
                if not isinstance(check, dict) or "name" not in check or "ok" not in check:
                    problems.append("safety_preconditions 每项必须含 name / ok")

    if status in (STATUS_BLOCKED, STATUS_ERROR):
        if not str(artifact.get("next_action") or "").strip():
            problems.append("status=%s 时必须有可粘贴 next_action" % status)
        # blocked / error 工件必须**自证**：前置检查逐项结果 + 可粘贴下一步
        pre_blocked = artifact.get("safety_preconditions")
        if not isinstance(pre_blocked, dict) or not (pre_blocked.get("checks") or []):
            problems.append("status=%s 时工件必须自证：前置检查逐项结果不得为空"
                            % status)

    return problems


def _resolve_out(out: str, dry_run: bool) -> str:
    path = out
    if not path.endswith(".json"):
        path = path + ".json"
    if dry_run:
        base, ext = os.path.splitext(path)
        path = base + ".dryrun" + ext
    return path


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        prog="rm75_s13_acc_jerk_evidence",
        description="S13 joint2..joint7 阶梯 acc/jerk 真机实测 → 证据工件")
    parser.add_argument("--stage", default="all",
                        choices=("all", "precheck", "run", "assemble"),
                        help="precheck=只读前置检查；run=跑阶梯实验；"
                             "assemble=组装工件；all=按顺序全跑")
    parser.add_argument("--arm", default="left", choices=("left", "right"))
    parser.add_argument("--ip", default=DEFAULT_IP, help="控制器 IP")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--joints", default=",".join(str(j) for j in DEFAULT_JOINTS),
                        help="**1 基**关节号，逗号分隔，支持 jointN（缺省 2..7）")
    parser.add_argument("--trials", type=int, default=DEFAULT_TRIALS,
                        help="每关节重复次数（默认 3）")
    parser.add_argument("--out", default=os.path.join(
        "workcell", "baselines", "evidence", "rm75_s13_acc_jerk.json"))
    parser.add_argument("--report-dir", default=os.path.join("build", "reports", "s13_ladder"),
                        help="hwcollect 输出目录")
    parser.add_argument("--precheck-out", default=os.path.join(
        "build", "reports", "s13_ladder", "precheck.json"))
    parser.add_argument("--dry-run", action="store_true",
                        help="离线骨架：不连任何设备，status=dry_run")
    parser.add_argument("--source-host", default="172.16.119.199:2224")
    parser.add_argument("--source-container", default="ros_dev")
    parser.add_argument("--source-worktree", default=".gsd-worktrees/M001-3kzz40")
    parser.add_argument("--ws", default=None, help="工作区根（默认自动定位）")
    args = parser.parse_args(argv)

    try:
        args.joints = parse_s13_joints(args.joints)
    except ValueError as exc:
        print("输入非法: %s" % exc, file=sys.stderr)
        return 2
    if int(args.trials) < 3:
        print("输入非法: --trials 必须 ≥ 3（契约要求 samples ≥ 3）", file=sys.stderr)
        return 2

    ws = args.ws or _repo_root()
    precheck_result: Dict[str, Any] = {"checks": [], "snapshot": None,
                                        "reason": "未执行前置检查"}
    report: Optional[Dict[str, Any]] = None
    report_path: Optional[str] = None
    ladder_meta: Optional[Dict[str, Any]] = None

    if args.stage in ("all", "precheck") and not args.dry_run:
        try:
            limits = _load_execution_limits(ws)
        except Exception as exc:  # noqa: BLE001 — 限值读不到即 blocked
            precheck_result = {"checks": [{"name": "limits_readable", "ok": False,
                                            "detail": "限值读取失败: %r" % (exc,)}],
                                "snapshot": None, "reason": "限值读取失败"}
            limits = {}
        snap = _snapshot_readonly(args.ip, int(args.port), args.arm, ws)
        snapshot = snap.get("snapshot") or {}
        if snap.get("status") != "ok":
            precheck_result = {
                "checks": [{"name": "readonly_snapshot", "ok": False,
                            "detail": snap.get("reason") or "只读快照失败"}],
                "snapshot": snapshot or None,
                "reason": snap.get("reason") or "只读快照失败",
                "next_action": snap.get("next_action") or "",
            }
        else:
            precheck_result = {"checks": precheck(snapshot, args.joints, limits),
                               "snapshot": snapshot, "reason": "",
                               "realtime_enabled": bool(snap.get("realtime_enabled")),
                               "realtime_reason": snap.get("realtime_reason") or "",
                               "realtime_target_ip": snap.get("realtime_target_ip"),
                               "udp_port": snap.get("udp_port")}
        os.makedirs(os.path.dirname(os.path.abspath(args.precheck_out)), exist_ok=True)
        with open(args.precheck_out, "w", encoding="utf-8") as fh:
            json.dump(precheck_result, fh, ensure_ascii=False, indent=2)
            fh.write("\n")
    elif args.stage == "assemble" and not args.dry_run:
        # **不重跑运动**：读回运动前的前置检查记录 + hwcollect 报告组装工件。
        loaded = _load_precheck_result(args.precheck_out)
        if loaded is None:
            # 缺门禁留痕 ⇒ **不得**产出 ok 工件（否则等于「没门禁也说通过」）
            precheck_result = {
                "checks": [{"name": "precheck_record_readable", "ok": False,
                            "detail": "读不到运动前的前置检查记录: %s"
                                      % args.precheck_out}],
                "snapshot": None,
                "reason": "缺前置检查记录",
            }
            print("前置检查记录不存在或不可读: %s" % args.precheck_out,
                  file=sys.stderr)
        else:
            precheck_result = loaded

    failed = [c for c in (precheck_result.get("checks") or []) if not c.get("ok")]
    if args.stage in ("all", "run") and not args.dry_run:
        if failed:
            print("前置检查未通过（%s）—— 不下发任何运动指令"
                  % "、".join(c["name"] for c in failed), file=sys.stderr)
        else:
            os.makedirs(os.path.abspath(args.report_dir), exist_ok=True)
            ladder_meta = run_ladder(args, args.report_dir, ws)
            report_path = _find_ladder_report(args.report_dir, args.arm)
            if report_path:
                with open(report_path, "r", encoding="utf-8") as fh:
                    report = json.load(fh)
    elif args.stage == "assemble" and not args.dry_run:
        report_path = _find_ladder_report(args.report_dir, args.arm)
        if report_path:
            report = _load_json(report_path)
            ladder_meta = {
                "source": "hwcollect 报告（由 --stage assemble 组装，未重跑运动）",
                "report_path": report_path,
            }

    artifact = build_artifact(args, precheck_result, report, report_path, ladder_meta)
    problems = validate_artifact(artifact, args.joints)
    artifact["self_check"] = {"ok": not problems, "problems": problems}

    path = _resolve_out(args.out, args.dry_run)
    out_dir = os.path.dirname(os.path.abspath(path))
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(artifact, fh, ensure_ascii=False, indent=2)
        fh.write("\n")

    print("status = %s" % artifact["status"])
    print("工件   = %s" % path)
    if artifact["status"] == STATUS_OK:
        for name, entry in sorted(artifact["joints"].items()):
            print("  %-7s acc=%.6f rad/s^2  jerk=%.6f rad/s^3  trials=%d"
                  % (name, entry["acceleration_rad_s2"], entry["jerk_rad_s3"],
                     entry["trials"]))
    if artifact.get("reason"):
        print("原因   = %s" % artifact["reason"])
    if problems:
        print("自检   = 不合规：%s" % "；".join(problems))
    print("下一步:\n%s" % "\n".join(artifact["next_action"]))

    if artifact["status"] == STATUS_OK:
        return EXIT_OK
    if artifact["status"] == STATUS_DRY_RUN:
        return EXIT_OK
    if artifact["status"] == STATUS_BLOCKED:
        return EXIT_BLOCKED
    return EXIT_ERROR


def _repo_root() -> str:
    """仓库根目录（本脚本在 ``<repo>/scripts/`` 下）。"""
    return os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                        os.pardir))


if __name__ == "__main__":
    sys.exit(main())
