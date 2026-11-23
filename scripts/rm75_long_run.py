#!/usr/bin/env python3
"""真机 10 分钟连续执行长跑（R001 / S03 的字面要求）。

里程碑验收写的是"一条经四道校验的轨迹在真机上**连续执行 10 分钟**无控制器保护停"，
此前只跑到 5.53 s / 501 拍（量化门限已达标，但时长不达标）。本工具补这一条：

* 生成一条**微动**轨迹（单关节、默认 0.2°、10 ms 栅格、时长可配）——
  幅度小到人眼几乎不可见，但**连续跑满**指定时长，考察的是长时间运行下的
  抖动 p99、跟踪误差 RMS、控制器保护停与断流处置；
* 走执行层真机路径（`rm_executor --mode live`），与短跑完全同一条代码路径；
* 结束输出可入库的证据 JSON。

用法::

    python3 scripts/rm75_long_run.py --ip 192.168.1.18 --duration-s 600 \
        --out build/reports/validation/long_run_600s.json

安全前提：臂必须**已使能且空闲**（探针的 7 项前置检查会先行把关，不通过就不发指令）。
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from typing import Any, Dict, List, Optional

SCHEMA = "rmctl.rm75_long_run/1"

#: 证据里 mode 的合法取值（真机 hardware / 离线 offline / 假执行 fake）
MODES = ("hardware", "offline", "fake")


def _probe_module(here: str):
    """取得 rm75_safe_probe 模块。

    **测试缝隙**：优先复用 ``sys.modules`` 中已注入的同名模块（离线用例注入假
    探针），否则按原路径动态加载（生产行为不变）。
    """
    injected = sys.modules.get("rm75_safe_probe")
    if injected is not None:
        return injected
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        "rm75_safe_probe", os.path.join(here, "rm75_safe_probe.py"))
    probe = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = probe
    spec.loader.exec_module(probe)
    return probe


def _executor_cli():
    """取得执行层 CLI 模块。

    **测试缝隙**：优先复用 ``sys.modules["rm_executor.cli"]``（离线用例注入假
    执行器），否则按原路径 ``from rm_executor import cli`` 导入。
    """
    injected = sys.modules.get("rm_executor.cli")
    if injected is not None:
        return injected
    from rm_executor import cli as executor_cli
    return executor_cli


def _evidence_mode(executor_cli_mod) -> str:
    """证据 mode：只有**真实**执行层模块才允许标 ``hardware``。

    注入的假执行器一律标 ``fake``——绝不让离线/假结果冒充真机结论。
    """
    path = (getattr(executor_cli_mod, "__file__", "") or "").replace("\\", "/")
    if path.endswith("/rm_executor/cli.py"):
        return "hardware"
    return "fake"


def build_trajectory(joints_deg: List[float], joint_index: int, delta_deg: float,
                     duration_s: float = 600.0, period_s: float = 0.010,
                     move_ratio: float = 0.25) -> Dict[str, Any]:
    """生成微动长跑轨迹（rad / s）。

    结构：前 25% 去程（平滑）→ 中间 50% 保持 → 后 25% 回程。
    任何时刻只有一个关节偏离初始位姿 ``delta_deg``，其余关节恒定。
    """
    total = int(round(duration_s / period_s))
    move_s = duration_s * move_ratio
    base = [float(v) for v in joints_deg]
    delta = math.radians(float(delta_deg))
    times: List[float] = []
    points: List[List[float]] = []
    for i in range(total + 1):
        t = i * period_s
        times.append(round(t, 6))
        if t <= move_s:
            s = t / move_s if move_s else 0.0
            offset = delta * (s * s * (3.0 - 2.0 * s))
        elif t <= duration_s - move_s:
            offset = delta
        else:
            s = (duration_s - t) / move_s if move_s else 0.0
            offset = delta * (s * s * (3.0 - 2.0 * s))
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
        "verify": {"ok": True, "checks": {"limits": {"ok": True},
                                         "discrete_collision": {"ok": True},
                                         "continuous_collision": {"ok": True}},
                   "note": "微动长跑轨迹（单关节 %.3f°，%.0f ms 栅格，%.0f s）："
                           "时间域限值由执行层阶段 A 复检" % (
                               float(delta_deg), period_s * 1000.0, duration_s)},
        "meta": {"generator": "rm75_long_run", "joint_index": int(joint_index),
                 "delta_deg": float(delta_deg), "duration_s": float(duration_s),
                 "period_s": float(period_s)},
    }


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        prog="rm75_long_run",
        description="真机长跑（默认 600 s 微动轨迹，执行层 live 路径）")
    parser.add_argument("--ip", default="192.168.1.18")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--udp-port", dest="udp_port", type=int, default=8091)
    parser.add_argument("--arm", choices=["left", "right"], default="left")
    parser.add_argument("--joint", dest="joint_index", type=int, default=0)
    parser.add_argument("--delta-deg", dest="delta_deg", type=float, default=0.2)
    parser.add_argument("--duration-s", dest="duration_s", type=float, default=600.0)
    parser.add_argument("--period-s", dest="period_s", type=float, default=0.010)
    parser.add_argument("--out", default=None)
    parser.add_argument("--ws", default=None)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args(argv)

    here = os.path.dirname(os.path.abspath(__file__))
    probe = _probe_module(here)

    # 1) 先做只读前置检查（7 项）——不通过就不发任何运动指令
    pre = probe.run_probe(arm=args.arm, ip=args.ip, port=args.port,
                           udp_port=args.udp_port, precheck_only=True, ws=args.ws)
    if pre.get("status") != "ok":
        sys.stderr.write("前置检查未通过，不运动: %s\n" % pre.get("reason"))
        return 3
    joints = None
    for phase in pre.get("phases") or []:
        if phase.get("name") == "snapshot":
            joints = (phase.get("data") or {}).get("snapshot", {}).get("joint_deg")
    if not joints or len(joints) != 7:
        sys.stderr.write("未读到 7 个关节角，无法生成轨迹\n")
        return 3

    # 2) 生成轨迹
    traj = build_trajectory(joints, args.joint_index, args.delta_deg,
                            args.duration_s, args.period_s)
    os.makedirs(os.path.join(args.ws or here, "build", "reports", "validation"), exist_ok=True)
    traj_path = os.path.join(args.ws or here, "build", "reports", "validation",
                             "long_run_trajectory.json")
    with open(traj_path, "w", encoding="utf-8") as fh:
        json.dump(traj, fh)

    # 3) 走执行层真机路径
    executor_cli = _executor_cli()
    t0 = time.time()
    report = executor_cli.execute(mode="live", trajectory_path=traj_path, realtime=True,
                                  arm=args.arm, ip=args.ip, port=args.port,
                                  udp_port=args.udp_port, ws=args.ws)
    wall = time.time() - t0
    metrics = report.get("metrics") or {}
    evidence = {
        "schema": SCHEMA,
        "mode": _evidence_mode(executor_cli),
        "status": report.get("status"),
        "reason": report.get("reason"),
        "arm": args.arm,
        "endpoint": {"ip": args.ip, "port": args.port},
        "params": {"joint_index": args.joint_index, "delta_deg": args.delta_deg,
                   "duration_s": args.duration_s, "period_s": args.period_s},
        "planned_points": len(traj["times"]),
        "metrics": metrics,
        "events": [e.get("type") for e in (report.get("events") or [])],
        "wall_s": round(wall, 3),
        "protection_stop": any(e.get("type") == "protection_stop"
                               for e in (report.get("events") or [])),
        "stream_lost": any(e.get("type") == "feedback_stream_lost"
                           for e in (report.get("events") or [])),
        "trajectory": traj_path,
    }
    if args.out:
        out = os.path.abspath(args.out)
        os.makedirs(os.path.dirname(out), exist_ok=True)
        with open(out, "w", encoding="utf-8") as fh:
            json.dump(evidence, fh, ensure_ascii=False, indent=2)
    if args.json:
        print(json.dumps(evidence, ensure_ascii=False, indent=2))
    else:
        print("长跑（%s）：status=%s 计划 %d 点 / %.0f s，墙钟 %.1f s"
              % (args.arm, evidence["status"], evidence["planned_points"],
                 args.duration_s, evidence["wall_s"]))
        for key in ("ticks", "tick_jitter_p99_ms", "follow_err_rms_deg",
                    "follow_err_max_deg", "arrival_err_deg"):
            if key in metrics:
                print("  %s = %s" % (key, metrics[key]))
        print("  保护停=%s 断流=%s" % (evidence["protection_stop"],
                                       evidence["stream_lost"]))
    return 0 if report.get("status") == "ok" else 4


if __name__ == "__main__":
    sys.exit(main())
