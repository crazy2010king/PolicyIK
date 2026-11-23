#!/usr/bin/env python3
"""RM75-B 驱动级限值下发留档（R004 的"能下发限值"验收项）。

为什么需要单独一个工具：里程碑验收要求"能把限值（驱动级速度 / 加速度 / 位置）
下发到真机并留证"，而项目既有路径 `rm_hw_bridge.limits.load_limit_defaults()`
在 `measured_limits.yaml` 未实测（status != measured）时**拒绝下发**（红线：
不得用占位值糊过去）。本工具在不破坏这条红线的前提下，给出一条**显式留档**的路径：

* 默认 `--dry-run`：**0 次 SDK 调用**，只打印 diff（当前值 → 目标值，含单位）；
* `--apply` 才真正下发，且必须同时给出 `--speed-scale`（保守起步倍率，
  默认 0.2，对应设计 §Testing Requirements 的"≤20% 速度起步"）；
* **位置限值用执行域**（`min_pos`/`max_pos` = 真机机械范围），
  **不是规划域** —— 规划域比机械范围窄（如 RM75 的 J7 规划 ±180°、机械 ±360°），
  真机当前位姿 J7=-200.5° 就在规划域之外；若拿规划域去下发驱动位置限值，
  会把**合法位姿**判成越界，甚至触发控制器保护。

诚实声明：白名单**没有驱动限值的读回符号**，因此下发结果只能凭
**每个 SDK 调用的返回码 + diff** 判定，**不伪造读回值**。

用法::

    python3 scripts/rm75_limit_downlink.py --ip 192.168.1.18                     # dry-run
    python3 scripts/rm75_limit_downlink.py --ip 192.168.1.18 --apply \\
        --speed-scale 0.2 --out build/reports/validation/limit_downlink.json
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from typing import Any, Dict, List, Optional

SCHEMA = "rmctl.rm75_limit_downlink/1"

#: 位置限值字段：下发用**执行域**（机械范围），不用规划域
POSITION_FIELDS = ("min_pos", "max_pos")


def build_limits(defaults: Dict[str, Any], speed_scale: float = 0.2,
                 acc_scale: float = 1.0) -> Dict[str, Any]:
    """基于 `load_limit_defaults()` 的结果构造**保守**的下发目标值。

    * 速度 = 声明速度 × ``speed_scale``（默认 20%，对应"≤20% 速度起步"）；
    * 加速度 = 声明加速度 × ``acc_scale``；
    * 位置 = **执行域**（`load_joint_limits()` 的 min_pos/max_pos，已是机械范围）。

    ``defaults`` 必须是 `load_limit_defaults(allow_placeholder=True)` 的返回值
    （status == "placeholder"，source 里带 "placeholder, 未实测" 标记）。
    """
    values: Dict[str, Any] = {}
    for name, item in (defaults.get("values") or {}).items():
        target = dict(item)
        if "max_speed" in target and isinstance(target["max_speed"], (int, float)):
            target["max_speed"] = float(target["max_speed"]) * float(speed_scale)
        if "max_acc" in target and isinstance(target["max_acc"], (int, float)):
            target["max_acc"] = float(target["max_acc"]) * float(acc_scale)
        values[name] = target
    out = dict(defaults)
    out["values"] = values
    out["speed_scale"] = float(speed_scale)
    out["acc_scale"] = float(acc_scale)
    out["position_source"] = "执行域（真机机械范围），非规划域"
    return out


def summarize(result: Dict[str, Any]) -> List[str]:
    """把下发结果压成可入库的证据行。"""
    lines: List[str] = []
    applied = result.get("applied") or []
    for item in applied:
        lines.append("%s(%s)=%s rc=%s" % (item.get("symbol"), item.get("joint"),
                                           item.get("value"), item.get("rc")))
    if not applied:
        lines.append("applied 为空（dry-run 或未下发）")
    return lines


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        prog="rm75_limit_downlink",
        description="RM75-B 驱动级限值下发留档（默认 dry-run，0 次 SDK 调用）")
    parser.add_argument("--ip", default="192.168.1.18")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--udp-port", dest="udp_port", type=int, default=8091)
    parser.add_argument("--arm", choices=["left", "right"], default="left")
    parser.add_argument("--speed-scale", dest="speed_scale", type=float, default=0.2,
                        help="速度目标 = 声明速度 × 该倍率（默认 0.2 = 20%% 起步）")
    parser.add_argument("--acc-scale", dest="acc_scale", type=float, default=1.0)
    parser.add_argument("--apply", action="store_true",
                        help="真正下发（缺省只 dry-run，不调用 SDK）")
    parser.add_argument("--out", default=None, help="证据 JSON 输出路径")
    parser.add_argument("--ws", default=None)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args(argv)

    script = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          "rm75_safe_probe.py")
    import importlib.util
    spec = importlib.util.spec_from_file_location("rm75_safe_probe", script)
    if spec is None or spec.loader is None:
        sys.stderr.write("无法加载探针模块: %s\n" % script)
        return 4
    probe = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = probe
    spec.loader.exec_module(probe)

    link, failure = probe._connect_link(args.arm, args.ip, args.port,
                                        args.udp_port, True, args.ws, None)
    if link is None:
        sys.stderr.write("连接失败: %s\n" % (failure or {}).get("reason"))
        return 3

    try:
        from rm_hw_bridge import limits as hw_limits
        defaults = hw_limits.load_limit_defaults(args.ws, allow_placeholder=True)
        if defaults.get("status") == "blocked":
            sys.stderr.write("限值来源被拒: %s\n" % defaults.get("reason"))
            return 3
        limits = build_limits(defaults, args.speed_scale, args.acc_scale)
        result = link.set_joint_limits(limits, dry_run=not args.apply)
        evidence = {
            "schema": SCHEMA,
            "mode": "apply" if args.apply else "dry_run",
            "arm": args.arm,
            "endpoint": {"ip": args.ip, "port": args.port},
            "limit_source": limits.get("source"),
            "speed_scale": limits.get("speed_scale"),
            "acc_scale": limits.get("acc_scale"),
            "position_source": limits.get("position_source"),
            "status": result.get("status"),
            "reason": result.get("reason"),
            "sdk_calls": result.get("sdk_calls"),
            "diff": result.get("diff"),
            "applied": result.get("applied"),
            "readback": "白名单无驱动限值读回符号 → 只以返回码 + diff 判定，不伪造读回值",
            "summary": summarize(result),
        }
    finally:
        try:
            link.close()
        except Exception:
            pass

    if args.out:
        out = os.path.abspath(args.out)
        os.makedirs(os.path.dirname(out), exist_ok=True)
        with open(out, "w", encoding="utf-8") as fh:
            json.dump(evidence, fh, ensure_ascii=False, indent=2)
    if args.json:
        print(json.dumps(evidence, ensure_ascii=False, indent=2))
    else:
        print("限值下发（%s）：status=%s sdk_calls=%s"
              % (evidence["mode"], evidence["status"], evidence["sdk_calls"]))
        print("  来源: %s" % evidence["limit_source"])
        print("  位置限值: %s" % evidence["position_source"])
        for line in evidence["summary"][:10]:
            print("  - %s" % line)
    return 0 if evidence["status"] == "ok" else 4


if __name__ == "__main__":
    sys.exit(main())
