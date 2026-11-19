#!/usr/bin/env python3
"""RM75-B 安全探针报告速览（纯读文件，**不碰硬件**）。

把 ``rm75_safe_probe`` 的 JSON 报告里各阶段的关键判定数字打成一览表，
便于在真机窗口里快速复核「到底动了多少、准不准、有没有急停」。

用法::

    python3 scripts/rm75_probe_report.py build/reports/rm75_safe_probe.json
    python3 scripts/rm75_probe_report.py build/reports/rm75_safe_probe*.json

退出码：0 至少一份报告可读且状态为 ok / 3 报告为 blocked / 4 报告为 error 或不可读。
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import sys
from typing import Any, Dict, List

#: 阶段 → 关注的字段（按出现顺序打印）
_KEY_FIELDS = {
    "snapshot": ("joint_deg", "arm_current_status_name",
                 "controller_trajectory_type_name", "joint_current_mA",
                 "joint_temperature_c"),
    "preflight": ("checks",),
    "arm_safety": ("result",),
    "move_out": ("channel", "elapsed_s", "target_deg"),
    "verify_out": ("arrival_error_deg", "arrival_tol_deg", "max_drift_deg",
                   "drift_tol_deg"),
    "move_back": ("channel", "elapsed_s"),
    "verify_back": ("max_return_error_deg", "tol_deg", "worst_joint"),
    "estop": ("ok", "detail"),
}


def _fmt(value: Any) -> str:
    if isinstance(value, float):
        return "%.4f" % value
    if isinstance(value, list):
        if value and all(isinstance(v, (int, float)) for v in value):
            return "[" + ", ".join("%.3f" % float(v) for v in value) + "]"
        return json.dumps(value, ensure_ascii=False)
    if isinstance(value, dict):
        return json.dumps(value, ensure_ascii=False)
    return str(value)


def summarize(report: Dict[str, Any]) -> List[str]:
    """把一份报告压成若干行文本。"""
    lines: List[str] = []
    params = report.get("params") or {}
    lines.append("状态: %s | 臂: %s | 端点: %s:%s | 通道: %s | 关节: %s | 位移: %s° | 速度: %s%%"
                 % (report.get("status"), report.get("arm"),
                    (report.get("endpoint") or {}).get("ip"),
                    (report.get("endpoint") or {}).get("port"),
                    params.get("channel", "-"),
                    (int(params["joint_index"]) + 1) if "joint_index" in params else "-",
                    params.get("delta_deg", "-"), params.get("speed_pct", "-")))
    for phase in report.get("phases") or []:
        name = phase.get("name")
        lines.append("  [%s] %-12s %s"
                     % ("PASS" if phase.get("status") in ("ok", "dry_run")
                        else phase.get("status"), name, phase.get("detail") or ""))
        data = phase.get("data") or {}
        for field in _KEY_FIELDS.get(name, ()):  # 关键字段
            if field not in data:
                continue
            value = data[field]
            if field == "checks":
                for check in value:
                    lines.append("        - [%s] %s: %s"
                                 % ("PASS" if check.get("ok") else "FAIL",
                                    check.get("name"), check.get("detail")))
                continue
            if field in ("joint_deg", "joint_current_mA", "joint_temperature_c"):
                continue        # 已在 snapshot 摘要里给过，避免刷屏
            if field == "result" and isinstance(value, dict):
                applied = value.get("applied")
                if isinstance(applied, list) and applied:
                    lines.append("        applied = %s"
                                 % ", ".join("%s=%r(rc=%s)" % (a.get("symbol"),
                                                               a.get("arg"), a.get("rc"))
                                             for a in applied))
                else:
                    lines.append("        status = %s" % value.get("status"))
                continue
            lines.append("        %s = %s" % (field, _fmt(value)))
        if name == "snapshot":
            snap = data.get("snapshot") or {}
            lines.append("        joint_deg = %s" % _fmt(snap.get("joint_deg")))
            lines.append("        电流 mA = %s" % _fmt(snap.get("joint_current_mA")))
            lines.append("        温度 ℃ = %s" % _fmt(snap.get("joint_temperature_c")))
    if report.get("reason"):
        lines.append("  原因: %s" % report["reason"])
    if report.get("next_action"):
        lines.append("  下一步: %s" % report["next_action"])
    return lines


def load(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as fh:
        return json.load(fh)


def main(argv: List[str] = None) -> int:
    parser = argparse.ArgumentParser(
        prog="rm75_probe_report",
        description="RM75-B 安全探针报告速览（纯读文件，不碰硬件）")
    parser.add_argument("reports", nargs="+", help="报告 JSON 路径（支持通配符）")
    args = parser.parse_args(argv)

    paths: List[str] = []
    for pattern in args.reports:
        matched = sorted(glob.glob(pattern))
        paths.extend(matched or [pattern])

    worst = 0
    for path in paths:
        print("=== %s" % path)
        try:
            report = load(path)
        except Exception as exc:  # noqa: BLE001 - 读不动就如实说
            print("  读取失败: %r" % exc)
            worst = 4
            continue
        for line in summarize(report):
            print(line)
        status = report.get("status")
        if status == "error":
            worst = max(worst, 4)
        elif status == "blocked" and worst < 4:
            worst = 3
    return worst


if __name__ == "__main__":
    sys.exit(main())
