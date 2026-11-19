#!/usr/bin/env python3
"""真机验证**阶梯**执行器（RM75-B / 192.168.1.18）。

用途
----
把「在真实机械臂上从简单到复杂完成整套验证」拆成 15 级阶梯（L0–L14），
并**按顺序**跑完，逐级给出 **PASS / FAIL / BLOCKED / MANUAL** 与证据。

阶梯总览
--------
=====  ==========================================  ======  ==============
级别  目标                                        会动?  判据来源
=====  ==========================================  ======  ==============
L0    链路与网关可达（TCP 8080 + 厂商网页）        否     自动
L1    厂商 SDK 与控制器能力                        否     自动
L2    真机连接与只读状态快照                      否     自动
L3    UDP 主动上报接入与周期标定                  否     自动
L4    使能标志与就绪判定                          否     自动
L5    运动前静态安全自检（围栏 / 软限位）          否     自动
L6    单轴小幅点动与回零                          **是**  自动
L7    5 ms 透传保持位姿与周期抖动                **是**  自动
L8    规划轨迹执行与末端跟踪                      **是**  自动 + 人工测量
L9    夹爪开合与夹持反馈                          **是**  自动
L10   安全七闸逐闸验证                            否     自动（离线注入 + 真机围栏）
L11   故障注入与恢复（五类）                      否     人工编排
L12   长稳 100 Hz × 30 min → 8 h                  **是**  人工编排
L13   双臂接入、标定与相对位姿搬运                **是**  探测 + 人工
L14   录制回放与 golden-file 回归                 否     自动（离线）
=====  ==========================================  ======  ==============

门禁规则
--------
* 任一级 FAIL ⇒ 后续级一律 **BLOCKED**（设计文档 §七「不达标不进下一阶段」）；
* 会动的级别必须显式 ``--allow-motion``，否则 BLOCKED（不偷偷动真机）；
* 需要人工的级别（L8 的末端误差 / L11 / L12 / L13）用 ``--evidence-file``
  回填实测值，格式 ``{"L8": {"tcp_tracking_error_mm": 2.1}}``；
* 真机不可达 ⇒ 输出 ``SKIP`` 并**逐条列出未验证指标**（沿用硬件门控纪律）。

用法
----
::

    # 0) 看阶梯定义（不连真机）
    python3 scripts/real_machine_ladder.py --list

    # 1) 只跑只读级别（最安全的一批）
    python3 scripts/real_machine_ladder.py --to L5 \\
        --device-config devices/rm75_b_192_168_1_18.yaml --json

    # 2) 跑含运动的级别（必须显式授权）
    python3 scripts/real_machine_ladder.py --from L6 --to L7 \\
        --device-config devices/rm75_b_192_168_1_18.yaml \\
        --joint-rad "0 -0.6 0 0.9 0 0.5 0" --allow-motion --json

退出码
------
``0``  全部级别通过（或因真机不可达而 SKIP）；
``1``  至少一级 FAIL；
``2``  用法错误（参数非法 / 设备配置缺失或非法）。
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = REPO_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from rm75_drake.device.config import DeviceConfigError  # noqa: E402
from rm75_drake.device.hardware_gate import (  # noqa: E402
    SKIP_REASON_HARDWARE_UNREACHABLE,
    UNVERIFIED_HARDWARE_METRICS,
    format_skip,
    probe_hardware,
)
from rm75_drake.device.measure_support import (  # noqa: E402
    DEFAULT_DEVICE_CONFIG,
    parse_joint_rad,
    resolve_device_config,
)
from rm75_drake.validation.ladder import (  # noqa: E402
    STATUS_FAIL,
    LadderContext,
    LadderError,
    build_ladder,
    ladder_report,
    run_ladder,
)

EXIT_OK = 0
EXIT_FAILURE = 1
EXIT_USAGE = 2

#: 本次运行是否**真的碰过真机**（碰过 ⇒ 退出时绕开解释器清理，见 :func:`_safe_exit`）。
_HARDWARE_TOUCHED = False


def build_parser() -> argparse.ArgumentParser:
    """构造命令行解析器。"""
    parser = argparse.ArgumentParser(
        description="真机验证阶梯（RM75-B / 192.168.1.18）：从只读到双臂的 15 级验证"
    )
    parser.add_argument("--list", action="store_true", help="打印阶梯定义后退出")
    parser.add_argument(
        "--stage", action="append", default=None, help="只跑指定级别（可重复，如 L0 L2）"
    )
    parser.add_argument("--from", dest="from_stage", default=None, help="起始级别（含）")
    parser.add_argument("--to", dest="to_stage", default=None, help="结束级别（含）")
    parser.add_argument(
        "--device-config",
        default=DEFAULT_DEVICE_CONFIG,
        help=f"设备（部署）配置路径（默认：{DEFAULT_DEVICE_CONFIG}）",
    )
    parser.add_argument(
        "--connect-timeout", type=float, default=2.0, help="TCP 探测超时（s，默认 2.0）"
    )
    parser.add_argument(
        "--allow-motion",
        action="store_true",
        help="授权会动真机的级别（L6/L7/L8/L9/L12/L13）；未给出时这些级别一律 BLOCKED",
    )
    parser.add_argument(
        "--joint-rad",
        default=None,
        help="保持位姿（7 个关节角，rad）——L7 必须显式给出，脚本不自己编造目标",
    )
    parser.add_argument(
        "--push-target-ip",
        default=None,
        help="UDP 主动上报的**目标 IP**（默认自动探测本机在机械臂网段上的地址）",
    )
    parser.add_argument(
        "--realtime-seconds",
        type=float,
        default=3.0,
        help="L3 采集上报帧的时长（s，默认 3.0）",
    )
    parser.add_argument(
        "--jitter-duration-s",
        type=float,
        default=10.0,
        help="L7 抖动测量时长（s，默认 10.0；正式测量建议 60）",
    )
    parser.add_argument(
        "--second-arm-ip",
        default="192.168.1.19",
        help="L13 探测的第二台臂 IP（默认 192.168.1.19）",
    )
    parser.add_argument(
        "--evidence-file",
        default=None,
        help="人工证据 JSON：{\"L8\": {\"tcp_tracking_error_mm\": 2.1}}",
    )
    parser.add_argument(
        "--out", default=None, help="把 JSON 报告写到该文件（默认打印到 stdout）"
    )
    parser.add_argument("--json", action="store_true", help="以 JSON 输出报告")
    return parser


def _select_stages(args: argparse.Namespace, stages: Sequence[Any]) -> List[str]:
    """根据 ``--stage/--from/--to`` 选出要跑的级别 id 列表。"""
    ids = [stage.id for stage in stages]
    if args.stage:
        wanted = [str(s).upper() for s in args.stage]
    elif args.from_stage or args.to_stage:
        start = str(args.from_stage).upper() if args.from_stage else ids[0]
        end = str(args.to_stage).upper() if args.to_stage else ids[-1]
        if start not in ids or end not in ids:
            raise LadderError(f"--from/--to 级别非法：{start}..{end}；可选：{ids}")
        wanted = ids[ids.index(start) : ids.index(end) + 1]
    else:
        wanted = ids
    unknown = [sid for sid in wanted if sid not in ids]
    if unknown:
        raise LadderError(f"未知级别：{unknown}；可选：{ids}")
    return wanted


def _load_evidence(path: Optional[str]) -> Dict[str, Dict[str, Any]]:
    """加载人工证据文件。"""
    if not path:
        return {}
    target = Path(path)
    if not target.is_file():
        raise LadderError(f"证据文件不存在：{target}")
    try:
        data = json.loads(target.read_text(encoding="utf-8"))
    except Exception as exc:
        raise LadderError(f"证据文件解析失败：{target}（{exc}）") from exc
    if not isinstance(data, dict):
        raise LadderError("证据文件顶层必须是对象：{级别 id: {判据 key: 值}}")
    return {str(k).upper(): dict(v) for k, v in data.items()}


def _print_ladder(stages: Sequence[Any]) -> None:
    """打印阶梯定义（人读表格）。"""
    print("真机验证阶梯（RM75-B / 192.168.1.18）")
    print("=" * 100)
    for stage in stages:
        motion = "会动" if stage.motion else "只读"
        manual = "人工" if stage.manual else "自动"
        print(f"[{stage.id}] {stage.title}   层级 {stage.tier} / {motion} / {manual}")
        print(f"    目标：{stage.goal}")
        for criterion in stage.criteria:
            threshold = (
                "" if criterion.threshold is None else f" {criterion.op} {criterion.threshold}"
            )
            print(f"    判据：{criterion.key} — {criterion.description}{threshold}")
        if stage.prerequisites:
            print(f"    前置：{'；'.join(stage.prerequisites)}")
        if stage.commands:
            print("    命令：")
            for command in stage.commands:
                print(f"      $ {command}")
        if stage.evidence:
            print(f"    证据：{'；'.join(stage.evidence)}")
        if stage.design_refs:
            print(f"    设计文档：{'；'.join(stage.design_refs)}")
        if stage.milestone_refs:
            print(f"    M001 切片：{'；'.join(stage.milestone_refs)}")
        if stage.failure_actions:
            print(f"    不通过时：{'；'.join(stage.failure_actions)}")
        print("-" * 100)


def main(argv: Optional[Sequence[str]] = None) -> int:
    """脚本入口（返回退出码）。"""
    parser = build_parser()
    args = parser.parse_args(list(argv) if argv is not None else None)
    stages = build_ladder()

    if args.list:
        _print_ladder(stages)
        return EXIT_OK

    try:
        selected = _select_stages(args, stages)
        evidence = _load_evidence(args.evidence_file)
    except LadderError as exc:
        print(f"real_machine_ladder: {exc}", file=sys.stderr)
        return EXIT_USAGE

    try:
        device_config = resolve_device_config(args.device_config)
    except DeviceConfigError as exc:
        print(f"real_machine_ladder: 设备配置非法：{exc}", file=sys.stderr)
        return EXIT_USAGE

    try:
        joint_rad = parse_joint_rad(args.joint_rad)
    except ValueError as exc:
        print(f"real_machine_ladder: {exc}", file=sys.stderr)
        return EXIT_USAGE

    # ── 硬件门控：真机不可达 ⇒ SKIP + 未验证指标清单 ──────────────────
    probe = probe_hardware(device_config, timeout=float(args.connect_timeout))
    if not probe.reachable:
        payload = {
            "status": "skipped",
            "reason": probe.reason or SKIP_REASON_HARDWARE_UNREACHABLE,
            "detail": probe.detail,
            "unverified_metrics": list(UNVERIFIED_HARDWARE_METRICS),
        }
        if args.json:
            print(json.dumps(payload, ensure_ascii=False, indent=2))
        else:
            print(format_skip(probe))
        return EXIT_OK

    # ── 执行阶梯 ─────────────────────────────────────────────────────
    context = LadderContext(
        device_config=device_config,
        dry_run=False,
        allow_motion=bool(args.allow_motion),
        options={
            "joint_rad": joint_rad,
            "push_target_ip": args.push_target_ip,
            "realtime_seconds": float(args.realtime_seconds),
            "jitter_duration_s": float(args.jitter_duration_s),
            "second_arm_ip": args.second_arm_ip,
        },
    )

    from rm75_drake.validation.hardware import HardwareExecutor

    global _HARDWARE_TOUCHED
    _HARDWARE_TOUCHED = True
    executor = HardwareExecutor(context)
    try:
        results = run_ladder(
            stages,
            executor,
            context=context,
            manual_evidence=evidence,
            stage_ids=selected,
        )
    finally:
        executor.close()

    report = ladder_report(results, context=context, device=device_config.to_dict())
    report["unverified_metrics"] = [
        metric
        for metric in UNVERIFIED_HARDWARE_METRICS
        if metric not in _covered_metrics(results)
    ]

    if args.out:
        out_path = Path(args.out)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(
            json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
        )

    if args.json:
        print(json.dumps(report, ensure_ascii=False, indent=2))
    else:
        for result in results:
            line = f"[{result.stage_id}] {result.status.upper():8s} {result.title}"
            print(line)
            for failure in result.failures:
                print(f"    ✗ {failure}")
            for note in result.notes:
                print(f"    · {note}")
        summary = report["summary"]
        print(
            "--- 阶梯汇总："
            f"PASS={summary.get('pass', 0)} FAIL={summary.get('fail', 0)} "
            f"BLOCKED={summary.get('blocked', 0)} MANUAL={summary.get('manual', 0)} ---"
        )
        if report["unverified_metrics"]:
            print("以下真机指标**未验证**（不得当作通过）：")
            for metric in report["unverified_metrics"]:
                print(f"  - {metric}")

    return EXIT_FAILURE if report["summary"].get(STATUS_FAIL, 0) else EXIT_OK


def _safe_exit(code: int) -> None:
    """退出进程（**绕开解释器清理阶段**）。

    ⚠️ 真机实测：厂商 SDK 的 UDP 接收线程是**原生线程**，在解释器退出阶段
    仍可能回调已释放的 Python 对象 ⇒ ``Fatal Python error: Segmentation fault``，
    且崩溃时机随机（可能发生在报告写盘之前 ⇒ 证据丢失）。
    ⇒ 报告与输出都已 flush 之后，用 ``os._exit`` 直接退出：
    句柄、socket、线程全部由内核回收（不会段错误，也不会丢证据）。
    """
    try:
        sys.stdout.flush()
        sys.stderr.flush()
    except Exception:  # pragma: no cover - 收尾尽力而为
        pass
    os._exit(int(code))


def _covered_metrics(results: Sequence[Any]) -> List[str]:
    """返回**已被本次阶梯覆盖**的真机指标名（用于未验证清单的扣减）。"""
    covered: List[str] = []
    for result in results:
        if result.status != "pass":
            continue
        if result.stage_id == "L7":
            covered.append("5 ms 透传周期抖动（P50/P99/P99.9/max）")
        if result.stage_id == "L8":
            covered.append("末端跟踪误差（目标 < 3 mm）")
        if result.stage_id == "L11":
            covered.append(
                "五类故障注入的物理验证（拔网线 / 物体掉落 / 夹爪夹空 / 关节超限位 / 供电跌落）"
            )
        if result.stage_id == "L12":
            covered.append("100 Hz × 30 min 长稳")
    return covered


if __name__ == "__main__":
    _code = main()
    if _HARDWARE_TOUCHED:
        _safe_exit(_code)
    sys.exit(_code)
