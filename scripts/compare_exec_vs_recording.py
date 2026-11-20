"""真机运行录制 ↔ 离线回放的**逐点一致性**比较（SC3 证据）。

比较对象
--------
* 真机运行日志（``cli exec --log-out``）里的 ``command`` 事件 = **真机实际下发**的
  关节角序列（安全层钳位后的版本）；
* 离线录制文件（``--recording``）里的采样序列 = **规划/回放**的关节角序列。

判据：两者**逐点相等**（容差 1e-9 rad）⇒ 真机跑的那条轨迹与离线重放的那条
**完全一致**（真机没有被安全层改写、没有丢点、没有重排）。

用法
----
::

    python3 scripts/compare_exec_vs_recording.py --run-log run.json --recording plan.json --json
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = REPO_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

EXIT_OK = 0
EXIT_FAILURE = 1
EXIT_USAGE = 2

#: 逐点比较容差（rad）。
DEFAULT_TOLERANCE_RAD = 1e-9


def build_parser() -> argparse.ArgumentParser:
    """构造命令行解析器。"""
    parser = argparse.ArgumentParser(
        description="真机运行日志 vs 离线录制：关节序列逐点一致性比较"
    )
    parser.add_argument("--run-log", required=True, help="真机运行日志（JSON）")
    parser.add_argument("--recording", required=True, help="离线录制文件（JSON）")
    parser.add_argument(
        "--tolerance-rad",
        type=float,
        default=DEFAULT_TOLERANCE_RAD,
        help=f"逐点容差（rad，默认 {DEFAULT_TOLERANCE_RAD}）",
    )
    parser.add_argument("--json", action="store_true", help="以 JSON 输出")
    return parser


def compare(run_log: Dict[str, Any], recording: Dict[str, Any], tol: float) -> Dict[str, Any]:
    """比较真机下发序列与录制采样序列。"""
    commands: List[List[float]] = [
        [float(v) for v in (event.get("payload") or {}).get("joint_rad", [])]
        for event in (run_log.get("events") or [])
        if event.get("kind") == "command"
    ]
    samples: List[List[float]] = [
        [float(v) for v in sample.get("q", [])]
        for sample in (recording.get("samples") or [])
    ]
    if not commands:
        raise ValueError("运行日志里没有 command 事件")
    if not samples:
        raise ValueError("录制文件里没有采样点")
    if len(commands) != len(samples):
        raise ValueError(
            f"点数不一致：真机下发 {len(commands)} 点 vs 录制 {len(samples)} 点"
        )
    max_diff = 0.0
    worst_index = -1
    for index, (cmd, rec) in enumerate(zip(commands, samples)):
        for axis in range(min(len(cmd), len(rec))):
            diff = abs(cmd[axis] - rec[axis])
            if diff > max_diff:
                max_diff = diff
                worst_index = index
    return {
        "num_points": len(commands),
        "num_axes": len(commands[0]),
        "max_joint_diff_rad": max_diff,
        "worst_point_index": worst_index,
        "tolerance_rad": float(tol),
        "pointwise_identical": bool(max_diff <= tol),
    }


def main(argv: Optional[Sequence[str]] = None) -> int:
    """脚本入口（返回退出码）。"""
    parser = build_parser()
    args = parser.parse_args(list(argv) if argv is not None else None)
    run_path = Path(args.run_log)
    rec_path = Path(args.recording)
    for path in (run_path, rec_path):
        if not path.is_file():
            print(f"compare_exec_vs_recording: 文件不存在：{path}", file=sys.stderr)
            return EXIT_USAGE
    try:
        summary = compare(
            json.loads(run_path.read_text(encoding="utf-8")),
            json.loads(rec_path.read_text(encoding="utf-8")),
            float(args.tolerance_rad),
        )
    except Exception as exc:
        print(f"compare_exec_vs_recording: 比较失败：{exc}", file=sys.stderr)
        return EXIT_FAILURE
    if args.json:
        print(json.dumps(summary, ensure_ascii=False, indent=2))
    else:
        print(
            f"[compare] 点数 {summary['num_points']}（{summary['num_axes']} 轴）"
            f" 最大逐点差 {summary['max_joint_diff_rad']:.3e} rad"
            f" ⇒ {'逐点一致' if summary['pointwise_identical'] else '不一致'}"
        )
    return EXIT_OK if summary["pointwise_identical"] else EXIT_FAILURE


if __name__ == "__main__":
    sys.exit(main())
