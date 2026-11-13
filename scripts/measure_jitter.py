#!/usr/bin/env python3
"""真机 **5 ms 透传周期抖动**测量（S03-T08）。

用途
----
在真机上以 5 ms 固定节拍持续透传一个**保持位姿**，统计每拍实际周期与
标称 5 ms 的偏差，给出 **P50 / P99 / P99.9 / max**（毫秒口径）。

口径纪律
--------
抖动统计**只有一份实现**：:class:`rm75_drake.device.metrics.JitterHistogram`
（离线单测与真机脚本共用），禁止脚本自己再算一套。

硬件门控
--------
真机不可达（SDK 缺失 / TCP 指令通道连不上）⇒ 输出
``SKIP(reason=hardware_unreachable)`` 并**逐条列出未验证的真机指标**，
退出码 **0**（既不伪装成失败，也不伪装成成功）。

真机测量**必须显式给出保持位姿**（``--joint-rad``）——脚本绝不自己编造
下发目标（否则可能把机械臂拽到未预期姿态）。

用法
----
::

    python3 scripts/measure_jitter.py --dry-run --duration-s 1.0
    python3 scripts/measure_jitter.py --joint-rad "0 -0.6 0 0.9 0 0.5 0" \
        --duration-s 60 --json

退出码
------
``0``  测量完成（或真机不可达时的 SKIP）；
``1``  测量不达标（抖动超阈 / 安全停机 / 传输错误）；
``2``  用法错误（参数非法 / 设备配置缺失或非法）。
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
    DEFAULT_DURATION_S,
    DEFAULT_PERIOD_S,
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
COVERED_METRIC = "5 ms 透传周期抖动（P50/P99/P99.9/max）"

#: 抖动阈值：P99.9 必须 < 0.5 ms（ROADMAP 的真机指标）。
DEFAULT_MAX_JITTER_P999_MS = 0.5


def build_parser() -> argparse.ArgumentParser:
    """构造命令行解析器。"""
    parser = argparse.ArgumentParser(
        description="真机 5 ms 透传周期抖动测量（真机不可达时 SKIP 并列出未验证指标）"
    )
    add_common_arguments(parser)
    parser.add_argument(
        "--period-ms",
        type=float,
        default=DEFAULT_PERIOD_S * 1000.0,
        help="标称节拍周期（ms，默认：5.0）",
    )
    parser.add_argument(
        "--duration-s",
        type=float,
        default=DEFAULT_DURATION_S,
        help=f"测量时长（s，默认：{DEFAULT_DURATION_S}）",
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

    period_s = float(args.period_ms) / 1000.0
    if period_s <= 0.0:
        print("measure_jitter: --period-ms 必须为正数", file=sys.stderr)
        return EXIT_USAGE
    if float(args.duration_s) <= 0.0:
        print("measure_jitter: --duration-s 必须为正数", file=sys.stderr)
        return EXIT_USAGE

    try:
        joint_rad = parse_joint_rad(args.joint_rad)
    except ValueError as exc:
        print(f"measure_jitter: {exc}", file=sys.stderr)
        return EXIT_USAGE

    try:
        device_config = resolve_device_config(args.device_config)
    except DeviceConfigError as exc:
        print(f"measure_jitter: 设备配置非法：{exc}", file=sys.stderr)
        return EXIT_USAGE

    # ── 硬件门控 ────────────────────────────────────────────────────
    limits = build_safety_limits(device_config)
    dry_run = bool(args.dry_run)
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
                "measure_jitter: 真机测量必须显式给出 --joint-rad"
                "（7 个关节角，rad）——脚本不会自己编造下发目标",
                file=sys.stderr,
            )
            return EXIT_USAGE
    if joint_rad is None:
        # 离线演练：用一个**通过围栏闸**的默认位姿（零位会被 G6 判越界）。
        joint_rad = default_hold_pose(limits)

    # ── 测量 ────────────────────────────────────────────────────────
    from rm75_drake.device.transport import ScriptedTransport, SdkTransport

    transport = ScriptedTransport() if dry_run else SdkTransport(device_config)
    try:
        transport.open()
    except Exception as exc:
        print(f"measure_jitter: 传输层打开失败：{exc!r}", file=sys.stderr)
        return EXIT_FAILURE
    try:
        result = run_hold_measurement(
            transport=transport,
            joint_rad=joint_rad,
            limits=limits,
            period_s=period_s,
            duration_s=float(args.duration_s),
            require_state=False,
        )
    finally:
        transport.close()

    summary = measurement_summary(result, joint_rad=joint_rad, period_s=period_s)
    summary["mode"] = "dry_run" if dry_run else "hardware"
    summary["device"] = device_config.to_dict()

    jitter = summary["jitter_ms"]
    p999 = jitter.get("p999")
    threshold = float(args.max_jitter_p999_ms)
    passed = True
    failures: List[str] = []
    if result.final_state != "completed":
        passed = False
        failures.append(f"执行环未正常走完（final_state={result.final_state}）")
    if result.errors:
        passed = False
        failures.append(f"传输层错误 {len(result.errors)} 次：{result.errors[0]}")
    if not dry_run and p999 is not None and float(p999) > threshold:
        passed = False
        failures.append(f"抖动 P99.9 = {float(p999):.4f} ms > 阈值 {threshold} ms")

    summary["status"] = "ok" if passed else "failed"
    summary["threshold_p999_ms"] = threshold
    summary["failures"] = failures

    # ── 未验证指标清单（诚实纪律）────────────────────────────────────
    if dry_run:
        unverified = list(UNVERIFIED_HARDWARE_METRICS)
    else:
        unverified = [m for m in UNVERIFIED_HARDWARE_METRICS if m != COVERED_METRIC]
    summary["unverified_metrics"] = unverified

    if args.json:
        print(json.dumps(summary, ensure_ascii=False, indent=2))
    else:
        mode = "离线演练（**不代表真机指标**）" if dry_run else "真机测量"
        print(f"[measure_jitter] 模式：{mode}")
        print(f"[measure_jitter] 位姿：{format_joint_rad(joint_rad)}")
        print(
            f"[measure_jitter] 拍数：{summary['ticks']}"
            f"（下发 {summary['commands_sent']} 次，周期 {summary['period_ms']} ms）"
        )
        print(
            "[measure_jitter] 抖动（ms）："
            f"p50={jitter.get('p50')} p99={jitter.get('p99')} "
            f"p99.9={jitter.get('p999')} max={jitter.get('max')} "
            f"count={jitter.get('count')}"
        )
        latency = summary["command_latency_ms"]
        print(
            "[measure_jitter] 命令→执行延迟（ms）："
            f"p50={latency.get('p50')} p99={latency.get('p99')} max={latency.get('max')}"
        )
        print(f"[measure_jitter] 安全违规数：{summary['num_violations']}")
        for item in failures:
            print(f"[measure_jitter] FAIL：{item}")
        print("以下真机指标**未验证**（不得当作通过）：")
        for item in unverified:
            print(f"  - {item}")

    return EXIT_OK if passed else EXIT_FAILURE


if __name__ == "__main__":
    sys.exit(main())
