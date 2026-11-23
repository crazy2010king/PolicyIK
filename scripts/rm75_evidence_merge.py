#!/usr/bin/env python3
"""把多份验证阶梯报告合并成**一份可入库的证据工件**。

为什么需要：``build/`` 被 gitignore，阶梯报告默认落在 ``build/`` 里**不进版本库**；
而里程碑验证要引用的是"哪一版代码、在哪台臂上、测到什么数"，因此必须有一份
**可追溯、可评审**的证据文件（``workcell/baselines/evidence/``）。

用法::

    python3 scripts/rm75_evidence_merge.py \\
        --out workcell/baselines/evidence/rm75_validation_ladder_20261119.json \\
        build/reports/validation/ladder_report.json \\
        build/reports/validation_udp600/ladder_report.json \\
        build/reports/validation_v8/ladder_report.json \\
        build/reports/validation_v9/ladder_report.json

合并规则：

* 同一阶段多次执行 → **保留最后一次**（并记 `runs` 次数），因为最后一次是最新代码的结果；
* 阶段被拆分到多份报告（例如 V2 单独跑 600 s 长跑）→ 同上；
* 数值**不做任何加工**：一律来自真机读数，脚本只做搬运与计数；
* 输出里**不含**易变路径（trajectory 绝对路径等）。

退出码：0 至少合并进一个阶段 / 3 没有可合并的报告。
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from typing import Any, Dict, List

SCHEMA = "rmctl.rm75_validation_evidence/1"


def _load(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as fh:
        return json.load(fh)


def merge(reports: List[Dict[str, Any]], sources: List[str]) -> Dict[str, Any]:
    """合并多份阶梯报告（按阶段 id 聚合，保留最后一次执行）。"""
    by_stage: Dict[str, Dict[str, Any]] = {}
    order: List[str] = []
    for report in reports:
        for stage in report.get("stages") or []:
            sid = stage.get("id")
            if not sid:
                continue
            if sid not in by_stage:
                order.append(sid)
                by_stage[sid] = {"id": sid, "title": stage.get("title"),
                                  "risk": stage.get("risk"),
                                  "moves_arm": stage.get("moves_arm"),
                                  "status": stage.get("status"),
                                  "reason": stage.get("reason"),
                                  "duration_s": stage.get("duration_s"),
                                  "evidence": [str(e) for e in (stage.get("evidence") or [])],
                                  "data": {k: v for k, v in (stage.get("data") or {}).items()
                                           if k not in ("trajectory",)},
                                  "runs": 1}
            else:
                prev = by_stage[sid]
                prev["runs"] += 1
                prev["status"] = stage.get("status")
                prev["reason"] = stage.get("reason")
                prev["duration_s"] = stage.get("duration_s")
                prev["evidence"] = [str(e) for e in (stage.get("evidence") or [])]
                prev["data"] = {k: v for k, v in (stage.get("data") or {}).items()
                                if k not in ("trajectory",)}

    coverage = {}
    for report in reports:
        for cc, entry in (report.get("coverage") or {}).items():
            coverage.setdefault(cc, entry)
    for entry in coverage.values():
        passed = [s["id"] for s in by_stage.values() if s.get("status") == "pass"]
        entry["passed_stages"] = [s for s in (entry.get("stages") or []) if s in passed]
        entry["open"] = not entry["passed_stages"]

    return {
        "schema": SCHEMA,
        "generated_at": reports[-1].get("finished_at") if reports else None,
        "arm": reports[-1].get("arm") if reports else None,
        "endpoint": reports[-1].get("endpoint") if reports else None,
        "status": reports[-1].get("status") if reports else None,
        "params": reports[-1].get("params") if reports else None,
        "summary": {
            "stages": len(by_stage),
            "passed": sum(1 for s in by_stage.values() if s.get("status") == "pass"),
            "failed": sum(1 for s in by_stage.values() if s.get("status") == "failed"),
            "blocked": sum(1 for s in by_stage.values() if s.get("status") == "blocked"),
            "errored": sum(1 for s in by_stage.values() if s.get("status") == "error"),
            "skipped": sum(1 for s in by_stage.values() if s.get("status") == "skipped"),
            "sources": sources,
        },
        "coverage": coverage,
        "stages": [by_stage[sid] for sid in order],
        "note": "本文件由 scripts/rm75_evidence_merge.py 合并 scripts/rm75_validation_ladder.py "
                "的真机报告生成；数值均来自真机读数，未做任何人工填写或加工。",
    }


def main(argv: List[str] = None) -> int:
    parser = argparse.ArgumentParser(
        prog="rm75_evidence_merge",
        description="把多份验证阶梯报告合并成可入库的证据工件")
    parser.add_argument("reports", nargs="+", help="阶梯报告 JSON 路径")
    parser.add_argument("--out", required=True, help="证据工件输出路径（入库用）")
    args = parser.parse_args(argv)

    reports, sources = [], []
    for path in args.reports:
        if not os.path.exists(path):
            sys.stderr.write("跳过（不存在）: %s\n" % path)
            continue
        try:
            reports.append(_load(path))
            sources.append(os.path.abspath(path))
        except Exception as exc:  # noqa: BLE001
            sys.stderr.write("跳过（无法解析）: %s: %r\n" % (path, exc))
    if not reports:
        sys.stderr.write("没有可合并的报告\n")
        return 3

    evidence = merge(reports, sources)
    out = os.path.abspath(args.out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w", encoding="utf-8") as fh:
        json.dump(evidence, fh, ensure_ascii=False, indent=2)
    print("证据工件: %s（%d 个阶段，来自 %d 份报告）"
          % (out, len(evidence["stages"]), len(reports)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
