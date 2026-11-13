#!/usr/bin/env python3
"""双臂标定作业脚本（S04-T08）：**离线**解算 + 残差报告。

与 ``python3 -m rm75_drake.cli calibrate`` 的关系
------------------------------------------------

* **同一份结论**：解算编排在 :mod:`rm75_drake.calibration.pipeline`（包内唯一实现），
  本脚本与 CLI 都只是它的薄壳 —— 禁止在脚本里另写一套解算口径；
* **同一份 JSON 结构**：``--json`` 输出与 CLI 完全一致的字段
  （``status`` / ``reason`` / ``residuals`` / ``measured`` / ``unverified_metrics`` …）。

自引导
------

脚本把仓库的 ``src/`` 插入 ``sys.path``（与 ``tools/verify_env.py`` 同一做法），
因此**无需 ``pip install -e .```` 即可直接运行::

    python3 scripts/calibrate_dual_arm.py --json

退出码（与 CLI 一致）
---------------------

====  ==========================================================
码    含义
====  ==========================================================
0     成功 / **无实测数据时的 SKIP**（逐条列出未验证的物理指标）
1     业务失败（残差超阈值 / 求解器错误 / 数据文件内容非法）
2     用法错误（路径缺失 / 参数非法）
====  ==========================================================

**硬件门控语义**：本环境无真机、无实测数据，因此默认输出
``SKIP(reason=calibration_not_measured)`` 并以 **0** 退出 ——
既不把合成数据的残差冒充实测精度，也不把「没测」伪装成失败。
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict, Optional, Sequence

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = REPO_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from rm75_drake.calibration.pipeline import (  # noqa: E402
    CalibrationUsageError,
    run_calibration,
)
from rm75_drake.calibration.schema import CalibrationError  # noqa: E402
from rm75_drake.calibration.solvers import (  # noqa: E402
    DEFAULT_BASE_EXTRINSIC_THRESHOLD_M,
    DEFAULT_JOINT_ZERO_THRESHOLD_M,
    DEFAULT_TOOL_TCP_THRESHOLD_M,
)

EXIT_OK = 0
EXIT_FAILURE = 1
EXIT_USAGE = 2

DEFAULT_SAMPLES = REPO_ROOT / "calibration" / "samples" / "demo.yaml"
DEFAULT_CALIBRATION = REPO_ROOT / "calibration" / "dual_arm.yaml"


def build_parser() -> argparse.ArgumentParser:
    """构造参数解析器（与 CLI 的 ``calibrate`` 子命令**同名同义**）。"""
    parser = argparse.ArgumentParser(
        prog="scripts/calibrate_dual_arm.py",
        description=(
            "双臂标定作业（离线解算 + 残差报告）。"
            "无实测数据时输出 SKIP(reason=calibration_not_measured) 并以 0 退出，"
            "同时逐条列出未验证的物理指标。"
        ),
    )
    parser.add_argument(
        "--samples",
        default=str(DEFAULT_SAMPLES),
        help=f"采样点集文件路径（默认：{DEFAULT_SAMPLES}）",
    )
    parser.add_argument(
        "--calibration",
        default=str(DEFAULT_CALIBRATION),
        help=f"标定文件路径（默认：{DEFAULT_CALIBRATION}）",
    )
    parser.add_argument("--out", default=None, help="把结果摘要写成 JSON 文件（路径）")
    parser.add_argument(
        "--accept-synthetic",
        action="store_true",
        help="显式接受合成数据（**仅离线演练**；measured 仍为 false）",
    )
    parser.add_argument(
        "--fk",
        choices=("auto", "model", "synthetic"),
        default="auto",
        help="关节零位求解的 FK 对照口径（默认 auto）",
    )
    parser.add_argument(
        "--joint-zero-threshold",
        type=float,
        default=DEFAULT_JOINT_ZERO_THRESHOLD_M,
        help=f"关节零位残差阈值（m，默认 {DEFAULT_JOINT_ZERO_THRESHOLD_M}）",
    )
    parser.add_argument(
        "--tool-tcp-threshold",
        type=float,
        default=DEFAULT_TOOL_TCP_THRESHOLD_M,
        help=f"工具 TCP 残差阈值（m，默认 {DEFAULT_TOOL_TCP_THRESHOLD_M}）",
    )
    parser.add_argument(
        "--base-extrinsic-threshold",
        type=float,
        default=DEFAULT_BASE_EXTRINSIC_THRESHOLD_M,
        help=f"基座外参残差阈值（m，默认 {DEFAULT_BASE_EXTRINSIC_THRESHOLD_M}）",
    )
    parser.add_argument("--json", action="store_true", help="以**单行** JSON 输出结果摘要")
    return parser


def _print_summary(payload: Dict[str, Any]) -> None:
    """人类可读摘要（**必含**未验证项清单）。"""
    print(f"[calibrate_dual_arm] 状态：{payload['status']}"
          + (f"（reason={payload['reason']}）" if payload["reason"] else ""))
    print(f"[calibrate_dual_arm] 采样文件：{payload['samples_path']}")
    print(f"[calibrate_dual_arm] 标定文件：{payload['calibration_path']}")
    print(f"[calibrate_dual_arm] 实测 measured：{payload['measured']}"
          f"（FK 口径：{payload['fk_mode']}）")
    for name, section in payload["sections"].items():
        if section["status"] == "absent":
            print(f"[calibrate_dual_arm] {name}：无采样（skipped）")
            continue
        print(
            f"[calibrate_dual_arm] {name}：{section['status']} "
            f"RMS={section['rms']:.6e} m 最大={section['max']:.6e} m "
            f"样本数={section['sample_count']} 阈值={section['threshold']:.6e} m"
        )
        if section.get("detail"):
            print(f"[calibrate_dual_arm]   说明：{section['detail']}")
    if payload["detail"]:
        print(f"[calibrate_dual_arm] 结论：{payload['detail']}")
    if payload["unverified_metrics"]:
        print("[calibrate_dual_arm] 以下物理指标**未验证**（不得当作通过）：")
        for item in payload["unverified_metrics"]:
            print(f"[calibrate_dual_arm]   - {item}")
    if payload.get("output_path"):
        print(f"[calibrate_dual_arm] 结果文件：{payload['output_path']}")


def main(argv: Optional[Sequence[str]] = None) -> int:
    """脚本主入口（返回退出码，不直接 ``sys.exit``）。"""
    parser = build_parser()
    args = parser.parse_args(list(argv) if argv is not None else None)
    try:
        run = run_calibration(
            samples_path=args.samples,
            calibration_path=args.calibration,
            accept_synthetic=bool(args.accept_synthetic),
            fk_mode=str(args.fk),
            joint_zero_threshold=float(args.joint_zero_threshold),
            tool_tcp_threshold=float(args.tool_tcp_threshold),
            base_extrinsic_threshold=float(args.base_extrinsic_threshold),
        )
    except CalibrationUsageError as exc:
        print(f"calibrate_dual_arm: 参数错误：{exc}", file=sys.stderr)
        return EXIT_USAGE
    except CalibrationError as exc:
        print(f"calibrate_dual_arm: 标定数据非法：{exc}", file=sys.stderr)
        return EXIT_FAILURE

    payload = run.as_dict()
    if args.out:
        out_file = Path(args.out)
        out_file.parent.mkdir(parents=True, exist_ok=True)
        out_file.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2, default=str), encoding="utf-8"
        )
        payload["output_path"] = str(out_file)

    if args.json:
        print(json.dumps(payload, ensure_ascii=False, default=str))
    else:
        _print_summary(payload)
    return run.exit_code


if __name__ == "__main__":
    sys.exit(main())
