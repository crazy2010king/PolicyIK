#!/usr/bin/env python3
"""真机轨迹执行的**跟踪误差**评估（从全链路运行日志算）。

用途
----
《设计文档》§六 主张 8：末端跟踪误差 < 3 mm。本脚本把
``rm75_drake.cli exec --log-out`` 产出的运行日志（``command`` 事件 = 实际下发的
关节角；``state_frame`` 事件 = UDP 回读的实测关节角）逐点配对，算出：

* 关节空间误差（rad / deg）：``|指令 - 实测|``；
* **末端位置误差（mm）**：用 MDH 正运动学把两组关节角各自算成 link7 位置，
  再取差值——这是**末端跟踪误差的机器侧估计**（无 F/T 传感器时，
  只能到「指令位形 vs 实测位形」这一层；绝对精度仍建议外部测量复核）。

用法
----
::

    python3 scripts/eval_tracking.py --run-log run_logs/xxx.json --json

退出码
------
``0`` 评估完成且末端误差达标（``--max-tcp-error-mm`` 默认 3.0）；
``1`` 误差超标 / 日志结构非法；
``2`` 用法错误。
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = REPO_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from rm75_drake.device.sdk import deg_to_rad, rad_to_deg  # noqa: E402
from rm75_drake.kinematics.mdh import link7_position  # noqa: E402

EXIT_OK = 0
EXIT_FAILURE = 1
EXIT_USAGE = 2

#: 默认末端跟踪误差阈值（mm）——《设计文档》§六 主张 8。
DEFAULT_MAX_TCP_ERROR_MM = 3.0


def build_parser() -> argparse.ArgumentParser:
    """构造命令行解析器。"""
    parser = argparse.ArgumentParser(
        description="真机轨迹跟踪误差评估（关节空间 + 末端位置，来源：全链路运行日志）"
    )
    parser.add_argument("--run-log", required=True, help="运行日志路径（JSON）")
    parser.add_argument(
        "--max-tcp-error-mm",
        type=float,
        default=DEFAULT_MAX_TCP_ERROR_MM,
        help=f"末端跟踪误差阈值（mm，默认 {DEFAULT_MAX_TCP_ERROR_MM}）",
    )
    parser.add_argument("--json", action="store_true", help="以 JSON 输出")
    return parser


def _pairs(events: Sequence[Dict[str, Any]]) -> List[Tuple[List[float], List[float]]]:
    """把 ``command`` 与随后的 ``state_frame`` 逐点配对。"""
    pending: Optional[List[float]] = None
    out: List[Tuple[List[float], List[float]]] = []
    for event in events:
        payload = event.get("payload") or {}
        kind = event.get("kind") or event.get("event")
        if kind == "command":
            pending = [float(v) for v in payload.get("joint_rad", [])]
        elif kind == "state_frame":
            feedback = [float(v) for v in payload.get("joint_rad", [])]
            if pending is not None and len(pending) == len(feedback) == 7:
                out.append((pending, feedback))
            pending = None
    return out


def evaluate(run_log: Dict[str, Any]) -> Dict[str, Any]:
    """从运行日志算出跟踪误差统计。"""
    events = run_log.get("events") or []
    pairs = _pairs(events)
    if not pairs:
        raise ValueError("运行日志里没有可配对的 command / state_frame 事件")
    joint_errors: List[float] = []
    tcp_errors: List[float] = []
    per_axis_max = [0.0] * 7
    for command, feedback in pairs:
        for axis in range(7):
            error = abs(command[axis] - feedback[axis])
            joint_errors.append(error)
            if error > per_axis_max[axis]:
                per_axis_max[axis] = error
        cmd_pos = link7_position(command)
        fb_pos = link7_position(feedback)
        distance = math.sqrt(
            sum((float(cmd_pos[i]) - float(fb_pos[i])) ** 2 for i in range(3))
        )
        tcp_errors.append(distance * 1000.0)
    joint_errors.sort()
    tcp_errors.sort()

    def _pct(values: Sequence[float], fraction: float) -> float:
        if not values:
            return 0.0
        index = min(len(values) - 1, int(len(values) * fraction))
        return float(values[index])

    return {
        "num_pairs": len(pairs),
        "joint_error_rad": {
            "p50": _pct(joint_errors, 0.50),
            "p95": _pct(joint_errors, 0.95),
            "max": joint_errors[-1] if joint_errors else 0.0,
        },
        "joint_error_deg": {
            "p50": rad_to_deg(_pct(joint_errors, 0.50)),
            "p95": rad_to_deg(_pct(joint_errors, 0.95)),
            "max": rad_to_deg(joint_errors[-1]) if joint_errors else 0.0,
        },
        "per_axis_max_error_deg": [rad_to_deg(v) for v in per_axis_max],
        "tcp_error_mm": {
            "p50": _pct(tcp_errors, 0.50),
            "p95": _pct(tcp_errors, 0.95),
            "max": tcp_errors[-1] if tcp_errors else 0.0,
        },
    }


def main(argv: Optional[Sequence[str]] = None) -> int:
    """脚本入口（返回退出码）。"""
    parser = build_parser()
    args = parser.parse_args(list(argv) if argv is not None else None)
    path = Path(args.run_log)
    if not path.is_file():
        print(f"eval_tracking: 运行日志不存在：{path}", file=sys.stderr)
        return EXIT_USAGE
    try:
        run_log = json.loads(path.read_text(encoding="utf-8"))
        summary = evaluate(run_log)
    except Exception as exc:
        print(f"eval_tracking: 评估失败：{exc}", file=sys.stderr)
        return EXIT_FAILURE

    threshold = float(args.max_tcp_error_mm)
    max_tcp = float(summary["tcp_error_mm"]["max"])
    passed = max_tcp <= threshold
    summary["threshold_tcp_error_mm"] = threshold
    summary["status"] = "pass" if passed else "failed"

    if args.json:
        print(json.dumps(summary, ensure_ascii=False, indent=2))
    else:
        print(f"[eval_tracking] 配对点数：{summary['num_pairs']}")
        print(
            "[eval_tracking] 关节误差（deg）："
            f"p50={summary['joint_error_deg']['p50']:.4f} "
            f"p95={summary['joint_error_deg']['p95']:.4f} "
            f"max={summary['joint_error_deg']['max']:.4f}"
        )
        print(
            "[eval_tracking] 末端误差（mm，MDH FK 估计）："
            f"p50={summary['tcp_error_mm']['p50']:.3f} "
            f"p95={summary['tcp_error_mm']['p95']:.3f} "
            f"max={summary['tcp_error_mm']['max']:.3f}"
        )
        print(f"[eval_tracking] 判定：{'PASS' if passed else 'FAIL'}（阈值 {threshold} mm）")
    return EXIT_OK if passed else EXIT_FAILURE


if __name__ == "__main__":
    sys.exit(main())
