#!/usr/bin/env python3
"""真机 UDP 断流复现（S03 / R001 的"断流 200 ms 内停机"验收项）。

离线只能**注入**假断流；真机证据必须让反馈通道**真的**断掉。本工具的做法：

1. 建链并**开启**主动上报（`rm_set_realtime_push(enable=1)`）；
2. 启动执行层真机路径（10 ms 栅格流式透传）；
3. 流式开始后 ~1 s，**关闭主动上报**（`rm_set_realtime_push(enable=0)`）
   —— 控制器不再上报，SDK 反馈缓存随之变陈旧；
4. 执行层的反馈龄期越过 ``STREAM_STOP_MS``(200 ms) → 判 **L4 断流** →
   ``rm_set_arm_stop`` 急停；本工具记录"关上报 → L4 动作"的时延与事件链。

只使用契约白名单内的符号（``rm_set_realtime_push`` 属 C 类：纯配置）。

用法::

    python3 scripts/rm75_stream_loss_drill.py --ip 192.168.1.18 \
        --out build/reports/validation/stream_loss_drill.json
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time
from typing import Any, Dict, Optional

SCHEMA = "rmctl.rm75_stream_loss_drill/1"

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


def _long_run_module(here: str):
    """取得 rm75_long_run 模块。

    **测试缝隙**：优先复用 ``sys.modules["rm75_long_run"]``（离线用例注入假模块），
    否则按原路径动态加载（生产行为不变）。
    """
    injected = sys.modules.get("rm75_long_run")
    if injected is not None:
        return injected
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        "rm75_long_run", os.path.join(here, "rm75_long_run.py"))
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


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


def _hw_bridge_module():
    """取得 rm_hw_bridge.bridge 模块。

    **测试缝隙**：优先复用 ``sys.modules`` 中已注入的同名模块，否则按原路径导入。
    """
    injected = sys.modules.get("rm_hw_bridge.bridge")
    if injected is not None:
        return injected
    from rm_hw_bridge import bridge as hw_bridge
    return hw_bridge


def _evidence_mode(executor_cli_mod) -> str:
    """证据 mode：只有**真实**执行层模块才允许标 ``hardware``。

    注入的假执行器一律标 ``fake``——绝不让离线/假结果冒充真机结论。
    """
    path = (getattr(executor_cli_mod, "__file__", "") or "").replace("\\", "/")
    if path.endswith("/rm_executor/cli.py"):
        return "hardware"
    return "fake"


def main(argv: Optional[list] = None) -> int:
    parser = argparse.ArgumentParser(
        prog="rm75_stream_loss_drill",
        description="真机 UDP 断流复现（关主动上报 → 期望 200 ms 内 L4 停机）")
    parser.add_argument("--ip", default="192.168.1.18")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--udp-port", dest="udp_port", type=int, default=8091)
    parser.add_argument("--arm", choices=["left", "right"], default="left")
    parser.add_argument("--cut-after-s", dest="cut_after_s", type=float, default=1.0,
                        help="流式开始后多久关闭主动上报（秒）")
    parser.add_argument("--duration-s", dest="duration_s", type=float, default=8.0)
    parser.add_argument("--out", default=None)
    parser.add_argument("--ws", default=None)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args(argv)

    here = os.path.dirname(os.path.abspath(__file__))
    probe = _probe_module(here)
    executor_cli = _executor_cli()

    # 1) 前置检查（只读）
    pre = probe.run_probe(arm=args.arm, ip=args.ip, port=args.port,
                           udp_port=args.udp_port, precheck_only=True, ws=args.ws)
    if pre.get("status") != "ok":
        sys.stderr.write("前置检查未通过，不运动: %s\n" % pre.get("reason"))
        return 3
    joints = None
    for phase in pre.get("phases") or []:
        if phase.get("name") == "snapshot":
            joints = (phase.get("data") or {}).get("snapshot", {}).get("joint_deg")

    # 2) 生成一条 8 s 的微动轨迹（断流演练要的是"运动中断流"）
    long_run = _long_run_module(here)
    traj = long_run.build_trajectory(joints, 0, 0.2, args.duration_s, 0.010)
    ws = args.ws or here
    traj_path = os.path.join(ws, "build", "reports", "validation",
                             "stream_loss_trajectory.json")
    os.makedirs(os.path.dirname(traj_path), exist_ok=True)
    with open(traj_path, "w", encoding="utf-8") as fh:
        json.dump(traj, fh)

    # 3) 起一个线程在 cut_after_s 后关闭主动上报
    hw_bridge = _hw_bridge_module()
    cut_at = [None]
    cut_result = [None]

    def _cut():
        time.sleep(args.cut_after_s)
        link, failure = probe._connect_link(args.arm, args.ip, args.port,
                                            args.udp_port, True, args.ws, None)
        if link is None:
            cut_result[0] = {"status": "error",
                              "reason": (failure or {}).get("reason", "连接失败")}
            return
        try:
            res = link.set_realtime_push_enabled(False)
            cut_at[0] = time.time()
            # 与执行器主时钟**同一时钟域**（perf_counter）：用于计算
            # 「关主动上报 → 判流」时延（编排器 scripts/rm75_window_run.py 读取）。
            cut_result[0] = {"status": res.get("status"),
                              "reason": res.get("reason"),
                              "sdk_calls": res.get("sdk_calls"),
                              "cut_at_perf": time.perf_counter()}
        finally:
            try:
                link.close()
            except Exception:
                pass

    worker = threading.Thread(target=_cut, daemon=True)
    worker.start()

    # 4) 执行层真机路径（它自己会建链；注意：这会占第二个连接额度）
    t0 = time.time()
    report = executor_cli.execute(mode="live", trajectory_path=traj_path, realtime=True,
                                  arm=args.arm, ip=args.ip, port=args.port,
                                  udp_port=args.udp_port, ws=args.ws)
    worker.join(timeout=5.0)

    events = report.get("events") or []
    stream_lost = [e for e in events if e.get("type") == "feedback_stream_lost"]
    safety_events = [e for e in events
                     if str(e.get("type") or "").startswith("safety_")]
    l4_action = None
    for e in safety_events:
        if str(e.get("type") or "") == "safety_l4_emergency_stop":
            l4_action = "rm_set_arm_stop"
            break
    stop_latency_ms = None
    cut_perf = (cut_result[0] or {}).get("cut_at_perf")
    if cut_perf is not None and stream_lost:
        lost_t = stream_lost[0].get("t")
        if isinstance(lost_t, (int, float)) and not isinstance(lost_t, bool):
            stop_latency_ms = round((float(lost_t) - float(cut_perf)) * 1000.0, 3)
    evidence = {
        "schema": SCHEMA,
        "mode": _evidence_mode(executor_cli),
        "status": report.get("status"),
        "reason": report.get("reason"),
        "arm": args.arm,
        "endpoint": {"ip": args.ip, "port": args.port},
        "params": {"cut_after_s": args.cut_after_s, "duration_s": args.duration_s},
        "cut_result": cut_result[0],
        "events": [e.get("type") for e in events],
        "stream_lost": bool(stream_lost),
        "stream_lost_evidence": stream_lost[:2],
        "safety_events": safety_events[:4],
        "l4_action": l4_action,
        "stop_latency_ms": stop_latency_ms,
        "wall_s": round(time.time() - t0, 3),
        "trajectory": traj_path,
        "expectation": "关闭主动上报后，反馈龄期越过 STREAM_STOP_MS(200 ms) → L4 断流 → rm_set_arm_stop",
    }
    if args.out:
        out = os.path.abspath(args.out)
        os.makedirs(os.path.dirname(out), exist_ok=True)
        with open(out, "w", encoding="utf-8") as fh:
            json.dump(evidence, fh, ensure_ascii=False, indent=2)
    if args.json:
        print(json.dumps(evidence, ensure_ascii=False, indent=2))
    else:
        print("断流演练：status=%s 断流事件=%s" % (evidence["status"],
                                                  evidence["stream_lost"]))
        print("  关上报: %s" % (cut_result[0] or {}).get("reason"))
        for e in evidence["stream_lost_evidence"]:
            print("  - %s" % e)
    return 0 if evidence["stream_lost"] else 4


if __name__ == "__main__":
    sys.exit(main())
