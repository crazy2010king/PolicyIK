#!/usr/bin/env python3
"""S09 **一键回归** CLI（薄入口：只做参数解析 + 打印 + 退出码）。

用法
----
    python3 scripts/rm_regression.py
    python3 scripts/rm_regression.py --out build/reports/s09_regression.json
    python3 scripts/rm_regression.py --ws /path/to/ws
    python3 scripts/rm_regression.py --only gripper_offline,planner_acceptance

退出码（**唯一来源** = ``rm_executor.contract.EXIT_CODES``）
-----------------------------------------------------------
* ``0`` —— 没有段失败（可能存在 blocked 段，会逐条列出）；
* ``2`` —— argparse 参数错误（未知 ``--only`` 段 id 等）；
* ``3`` —— 没有段失败、但存在 blocked 段（干净机器 / 无真机窗口的正常结局）；
* ``4`` —— 存在 fail 段（真回归）。

报告
----
默认写到 ``build/reports/s09_regression.json``（相对仓库根）；顶层键为
``status`` / ``reason`` / ``next_action`` / ``evidence`` + ``segments[]``，
另带一个**只含 fail / blocked 段**的汇总键 ``problem_segments``
（每项含 ``id`` / ``verdict`` / ``reason`` / ``next_action``），
每段含 ``id`` / ``verdict`` / ``reason`` / ``next_action`` / ``evidence`` /
``mode`` / ``report_path``。段级语义与归一化规则见
``rmctl.regression.orchestrator`` 的模块 docstring。
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from typing import List, Optional

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(HERE)

for _rel in (
    "dualarm_ws/src/rmctl",
    "dualarm_ws/src/rm_executor",
    "dualarm_ws/src/rm_hw_bridge",
    "dualarm_ws/src/rm_safety",
    "dualarm_ws/src/rm_description/scripts",
):
    _p = os.path.join(REPO_ROOT, _rel)
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

from rmctl.regression import orchestrator as orch  # noqa: E402
from rmctl.regression import segments as seg_mod  # noqa: E402

#: 默认报告路径（相对仓库根）
DEFAULT_OUT = os.path.join("build", "reports", "s09_regression.json")


def _segment_id(value: str) -> str:
    """``--only`` 的取值校验（未知段 id → argparse 错误 → 退出码 2）。"""
    candidate = (value or "").strip()
    known = orch.segment_ids()
    if candidate not in known:
        raise argparse.ArgumentTypeError(
            "未知的段 id: %r（可选: %s）" % (value, " / ".join(known)))
    return candidate


def _parse_only(raw: str) -> List[str]:
    """把 ``--only a,b`` 拆成段 id 列表（逐个校验）。"""
    items = [item.strip() for item in str(raw or "").split(",") if item.strip()]
    return [_segment_id(item) for item in items]


def _print_report(report: dict, out_path: str) -> None:
    """打印回归报告：**先打汇总**（只列 fail / blocked 段并指名失败段），
    再打其余段明细。

    汇总为空时**明确打印**「无 fail/blocked 段」（绝不静默）。
    """
    print("一键回归报告: %s" % out_path)

    summary = report.get(seg_mod.SUMMARY_KEY)
    if not isinstance(summary, list):
        summary = seg_mod.summary_of(report.get("segments") or [])
    fails = [item for item in summary if item.get("verdict") == "fail"]
    blocked = [item for item in summary if item.get("verdict") == "blocked"]

    print("  汇总（fail / blocked 段）:")
    if not summary:
        print("    无 fail/blocked 段")
    for item in fails:
        print("    [fail] %s" % item.get("id"))
        print("        原因: %s" % item.get("reason"))
        print("        下一步: %s" % item.get("next_action"))
    for item in blocked:
        print("    [blocked] %s" % item.get("id"))
        print("        原因: %s" % item.get("reason"))
        print("        下一步: %s" % item.get("next_action"))

    print("  段明细:")
    for seg in report.get("segments") or []:
        print("  %-20s %-8s %s" % (seg.get("id"), seg.get("verdict"),
                                   seg.get("reason")))
        next_action = str(seg.get("next_action") or "")
        if next_action:
            print("      下一步: %s" % next_action)
    print("  总体: %s（%s）" % (report.get("status"), report.get("reason")))
    print("  退出码: %s" % report.get("exit_code"))


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="S09 一键回归（段级 verdict + 退出码 0/3/4）")
    parser.add_argument("--out", default=DEFAULT_OUT,
                        help="报告输出路径（相对仓库根；默认 %s）"
                             % DEFAULT_OUT)
    parser.add_argument("--ws", default=REPO_ROOT,
                        help="工作区根（含 workcell/ 与 build/model）")
    parser.add_argument("--only", default="",
                        help="只跑指定段（逗号分隔；可选值: %s；缺省 = 全部）"
                             % " / ".join(orch.segment_ids()))
    args = parser.parse_args(argv)

    try:
        only = _parse_only(args.only)
    except argparse.ArgumentTypeError as exc:
        parser.error(str(exc))  # argparse 错误 → 退出码 2

    out_path = args.out if os.path.isabs(args.out) \
        else os.path.join(REPO_ROOT, args.out)
    out_dir = os.path.dirname(out_path)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)

    ctx = orch.build_context(repo_root=REPO_ROOT, ws=args.ws)
    report = orch.run_regression(ctx, only=only or None)

    with open(out_path, "w", encoding="utf-8") as fh:
        json.dump(report, fh, ensure_ascii=False, indent=2, default=str)

    _print_report(report, out_path)
    return int(report.get("exit_code", 0))


if __name__ == "__main__":
    sys.exit(main())
