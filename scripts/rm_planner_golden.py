#!/usr/bin/env python3
"""S09 **黄金场景集回归脚本**（规划段）—— 一键回归编排器可子进程调用。

用法
----
    python3 scripts/rm_planner_golden.py
    python3 scripts/rm_planner_golden.py --out build/reports/s07_planner_golden.json
    python3 scripts/rm_planner_golden.py --only joint_small_local,unreachable_ik

退出码
------
* ``0`` —— 全部场景的 status / 四道校验 / 耗时断言通过；
* ``3`` —— **blocked**：外部条件未满足（厂商内核不可用 / 缺 build/model），
  **不是**代码回归；
* ``4`` —— 有场景断言不过（真回归）。

报告
----
默认写到 ``build/reports/s07_planner_golden.json``（相对仓库根），结构与
S07 的黄金场景集汇总**同构**（``version`` / ``scenario_count`` / ``p50_s`` /
``p95_s`` / ``success_rate`` / ``success_only`` / ``scenarios``），并追加执行层
契约键（``status`` / ``reason`` / ``next_action`` / ``evidence`` /
``failures``）供一键回归编排器归一化。

场景集来源**唯一**：`dualarm_ws/src/rmctl/tests/golden/planner_scenarios.json`
（git 跟踪）。执行逻辑**唯一**：`rmctl.golden_runner`（与 pytest 薄封装
`tests/test_planner_golden.py` 共用同一份实现）。
"""
from __future__ import annotations

import argparse
import os
import shutil
import sys
import tempfile
from typing import Any, Dict, List, Optional

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

from rmctl import golden_runner as gr  # noqa: E402
from rmctl.planner_service.client import PlannerClient  # noqa: E402
from rmctl.planner_service.server import PlannerServer  # noqa: E402

#: 默认报告路径（相对仓库根；与 S07 黄金场景集汇总同名同构）
DEFAULT_OUT = os.path.join("build", "reports", "s07_planner_golden.json")


def _parse_only(raw: str) -> Optional[List[str]]:
    """``--only a,b`` → 场景 id 列表（空 = 全部）。"""
    items = [item.strip() for item in str(raw or "").split(",") if item.strip()]
    return items or None


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="S09 黄金场景集回归脚本（规划段）")
    parser.add_argument("--out", default=DEFAULT_OUT,
                        help="报告输出路径（相对仓库根；默认 %s）" % DEFAULT_OUT)
    parser.add_argument("--ws", default=REPO_ROOT,
                        help="工作区根（含 workcell/ 的目录）")
    parser.add_argument("--only", default="",
                        help="只跑指定场景（逗号分隔；缺省 = 全部）")
    args = parser.parse_args(argv)

    out_path = args.out if os.path.isabs(args.out) \
        else os.path.join(REPO_ROOT, args.out)

    # -- 场景集加载（未知场景 id → 退出码 2，参数错误）------------------
    try:
        scenarios = gr.load_scenarios(only=_parse_only(args.only))
    except ValueError as exc:
        print("参数错误: %s" % exc, file=sys.stderr)
        return 2

    tmp_root = tempfile.mkdtemp(prefix="rm-golden-")
    try:
        ws = gr.prepare_workspace(tmp_root, repo_root=args.ws)

        # -- 厂商内核 / 模型可用性 → blocked（不是 fail）----------------
        ok, reason = gr.check_vendor(ws)
        if not ok:
            report = gr.blocked_report(
                reason=reason,
                next_action=("先 `source scripts/rmenv.sh` 提供 GLIBCXX_3.4.31+，"
                             "并 `python3 -m rmctl init` 物化 build/model 后重跑"),
                evidence=["工作区: %s" % ws],
            )
            gr.write_report(report, out_path)
            print("黄金场景集回归: %s（blocked）" % out_path)
            print("  原因: %s" % report["reason"])
            return 3

        # -- 起规划服务（真实进程内服务 + 独占 worker 子进程）----------
        sock_path = os.path.join(tmp_root, "rm_planner.sock")
        server = PlannerServer(socket_path=sock_path, ws=ws, worker_pool_size=1)
        server.start()
        client = PlannerClient(socket_path=sock_path, ws=ws)
        try:
            rows = gr.run_scenarios(scenarios, client)
            failures = gr.evaluate_rows(rows, scenarios)
        finally:
            server.stop()
    except Exception as exc:  # noqa: BLE001 - 诚实失败，不静默
        report = gr.summarize([], failures=["脚本异常终止（%s）: %s"
                                           % (type(exc).__name__, exc)])
        gr.write_report(report, out_path)
        print("黄金场景集回归异常终止: %s" % exc, file=sys.stderr)
        return 4
    finally:
        shutil.rmtree(tmp_root, ignore_errors=True)

    report = gr.summarize(rows, failures=failures)
    gr.write_report(report, out_path)

    print("黄金场景集回归报告: %s" % out_path)
    print("  场景数: %d  p50=%.3f s  p95=%.3f s  成功率=%.2f"
          % (report["scenario_count"], report["p50_s"] or 0.0,
             report["p95_s"] or 0.0, report["success_rate"]))
    for row in report["scenarios"]:
        print("  %-24s %-8s (期望 %-8s) %.3f s"
              % (row["id"], row["status"], row["expect_status"],
                 row["elapsed_s"]))
    for item in failures:
        print("  失败: %s" % item)
    return 4 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
