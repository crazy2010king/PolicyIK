#!/usr/bin/env python3
"""真机 **100 Hz × 30 min 长稳**测试（S03-T08）。

用途
----
以固定频率（默认 100 Hz）持续透传一个**保持位姿**，长时间观察：

* 节拍抖动（P50 / P99 / P99.9 / max，毫秒口径）；
* 安全七闸违规次数（应当为 0）；
* 传输层错误次数（应当为 0）；
* 执行环是否全程正常走完（中途停机即长稳失败）。

口径纪律
--------
抖动 / 延迟统计**只有一份实现**
（:class:`rm75_drake.device.metrics.MetricHistogram`），
与 ``measure_jitter.py`` 完全同源。

硬件门控
--------
真机不可达 ⇒ ``SKIP(reason=hardware_unreachable)`` + **逐条**列出未验证的
真机指标，退出码 **0**。离线演练（``--dry-run``）**不代表**真机长稳结果，
输出里必须标注「物理长稳未验证」。

用法
----
::

    python3 scripts/soak_test.py --dry-run --duration-s 5
    python3 scripts/soak_test.py --joint-rad "0 -0.6 0 0.9 0 0.5 0" \
        --duration-s 1800 --hz 100 --json

退出码
------
``0``  长稳通过（或真机不可达时的 SKIP）；
``1``  长稳失败（中途停机 / 传输错误 / 安全违规 / 抖动超阈）；
``2``  用法错误。
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import List, Optional, Sequence

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = REPO_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from rm75_drake.device.config import DeviceConfigError  # noqa: E402
from rm75_drake.device.hardware_gate import (  # noqa: E402
    UNVERIFIED_HARDWARE_METRICS,
    format_skip,
    probe_hardware,
)
from rm75_drake.device.measure_support import (  # noqa: E402
    add_common_arguments,
    build_safety_limits,
    default_hold_pose,
    format_joint_rad,
    measurement_summary,
    parse_joint_rad,
    resolve_device_config,
    run_hold_measurement,
)

EXIT_OK = 0
EXIT_FAILURE = 1
EXIT_USAGE = 2

#: 本脚本**覆盖**的指标（其它指标仍未验证，必须列出）。
COVERED_METRIC = "100 Hz × 30 min 长稳"

#: 真机长稳的默认时长（s）——30 min。
DEFAULT_SOAK_DURATION_S = 1800.0

#: 离线演练的默认时长（s）——**不代表**真机长稳。
DEFAULT_DRY_RUN_DURATION_S = 5.0

#: 默认频率（Hz）。
DEFAULT_HZ = 100.0

#: 抖动阈值：P99.9 必须 < 0.5 ms（仅在**真机**测量时作为判定依据）。
DEFAULT_MAX_JITTER_P999_MS = 0.5


def build_parser() -> argparse.ArgumentParser:
    """构造命令行解析器。"""
    parser = argparse.ArgumentParser(
        description="真机 100 Hz × 30 min 长稳测试（真机不可达时 SKIP 并列出未验证指标）"
    )
    add_common_arguments(parser)
    parser.add_argument(
        "--hz",
        type=float,
        default=DEFAULT_HZ,
        help=f"下发频率（Hz，默认：{DEFAULT_HZ}）",
    )
    parser.add_argument(
        "--duration-s",
        type=float,
        default=None,
        help=(
            "长稳时长（s）。默认：真机 1800（30 min）、"
            f"离线演练 {DEFAULT_DRY_RUN_DURATION_S}"
        ),
    )
    parser.add_argument(
        "--max-jitter-p999-ms",
        type=float,
        default=DEFAULT_MAX_JITTER_P999_MS,
        help=(
            "抖动 P99.9 阈值（ms，默认："
            f"{DEFAULT_MAX_JITTER_P999_MS}）；仅在**真机**测量时作为判定依据"
        ),
    )
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    """脚本入口（返回退出码）。"""
    parser = build_parser()
    args = parser.parse_args(list(argv) if argv is not None else None)

    hz = float(args.hz)
    if hz <= 0.0:
        print("soak_test: --hz 必须为正数", file=sys.stderr)
        return EXIT_USAGE
    dry_run = bool(args.dry_run)
    if args.duration_s is None:
        duration_s = DEFAULT_DRY_RUN_DURATION_S if dry_run else DEFAULT_SOAK_DURATION_S
    else:
        duration_s = float(args.duration_s)
    if duration_s <= 0.0:
        print("soak_test: --duration-s 必须为正数", file=sys.stderr)
        return EXIT_USAGE

    try:
        joint_rad = parse_joint_rad(args.joint_rad)
    except ValueError as exc:
        print(f"soak_test: {exc}", file=sys.stderr)
        return EXIT_USAGE

    try:
        device_config = resolve_device_config(args.device_config)
    except DeviceConfigError as exc:
        print(f"soak_test: 设备配置非法：{exc}", file=sys.stderr)
        return EXIT_USAGE

    # ── 硬件门控 ────────────────────────────────────────────────────
    limits = build_safety_limits(device_config)
    if not dry_run:
        probe = probe_hardware(device_config, timeout=float(args.connect_timeout))
        if not probe.reachable:
            if not args.json:
                print(format_skip(probe))
            else:
                print(
                    json.dumps(
                        {
                            "status": "skipped",
                            "reason": probe.reason,
                            "detail": probe.detail,
                            "unverified_metrics": list(UNVERIFIED_HARDWARE_METRICS),
                        },
                        ensure_ascii=False,
                        indent=2,
                    )
                )
            return EXIT_OK
        if joint_rad is None:
            print(
                "soak_test: 真机长稳必须显式给出 --joint-rad"
                "（7 个关节角，rad）——脚本不会自己编造下发目标",
                file=sys.stderr,
            )
            return EXIT_USAGE
    if joint_rad is None:
        # 离线演练：用一个**通过围栏闸**的默认位姿（零位会被 G6 判越界）。
        joint_rad = default_hold_pose(limits)

    # ── 长稳 ────────────────────────────────────────────────────────
    from rm75_drake.device.transport import ScriptedTransport, SdkTransport

    transport = ScriptedTransport() if dry_run else SdkTransport(device_config)
    period_s = 1.0 / hz
    try:
        transport.open()
    except Exception as exc:
        print(f"soak_test: 传输层打开失败：{exc!r}", file=sys.stderr)
        return EXIT_FAILURE
    try:
        result = run_hold_measurement(
            transport=transport,
            joint_rad=joint_rad,
            limits=limits,
            period_s=period_s,
            duration_s=duration_s,
            require_state=False,
        )
    finally:
        transport.close()

    summary = measurement_summary(result, joint_rad=joint_rad, period_s=period_s)
    summary["mode"] = "dry_run" if dry_run else "hardware"
    summary["requested_duration_s"] = duration_s
    summary["hz"] = hz
    summary["device"] = device_config.to_dict()

    jitter = summary["jitter_ms"]
    p999 = jitter.get("p999")
    threshold = float(args.max_jitter_p999_ms)
    failures: List[str] = []
    if result.final_state != "completed":
        failures.append(f"执行环未正常走完（final_state={result.final_state}）")
    if result.errors:
        failures.append(f"传输层错误 {len(result.errors)} 次：{result.errors[0]}")
    if summary["num_violations"]:
        failures.append(f"安全七闸违规 {summary['num_violations']} 次")
    if not dry_run and p999 is not None and float(p999) > threshold:
        failures.append(f"抖动 P99.9 = {float(p999):.4f} ms > 阈值 {threshold} ms")

    passed = not failures
    summary["status"] = "ok" if passed else "failed"
    summary["threshold_p999_ms"] = threshold
    summary["failures"] = failures

    if dry_run:
        unverified = list(UNVERIFIED_HARDWARE_METRICS)
    else:
        unverified = [m for m in UNVERIFIED_HARDWARE_METRICS if m != COVERED_METRIC]
    summary["unverified_metrics"] = unverified

    if args.json:
        print(json.dumps(summary, ensure_ascii=False, indent=2))
    else:
        mode = "离线演练（**物理长稳未验证**）" if dry_run else "真机长稳"
        print(f"[soak_test] 模式：{mode}")
        print(f"[soak_test] 位姿：{format_joint_rad(joint_rad)}")
        print(
            f"[soak_test] 时长：{duration_s} s（{hz} Hz），"
            f"拍数：{summary['ticks']}（下发 {summary['commands_sent']} 次）"
        )
        print(
            "[soak_test] 抖动（ms）："
            f"p50={jitter.get('p50')} p99={jitter.get('p99')} "
            f"p99.9={jitter.get('p999')} max={jitter.get('max')} "
            f"count={jitter.get('count')}"
        )
        print(
            f"[soak_test] 安全违规数：{summary['num_violations']}，"
            f"传输错误数：{summary['num_errors']}"
        )
        for item in failures:
            print(f"[soak_test] FAIL：{item}")
        print("以下真机指标**未验证**（不得当作通过）：")
        for item in unverified:
            print(f"  - {item}")

    return EXIT_OK if passed else EXIT_FAILURE


if __name__ == "__main__":
    sys.exit(main())
