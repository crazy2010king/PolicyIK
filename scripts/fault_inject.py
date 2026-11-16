#!/usr/bin/env python3
"""五类故障注入的**人工操作编排**与恢复流程校验（S03-T08）。

五类故障（研究 F1）
------------------
====================  ==========================  ========================
故障                  物理操作（人工）            期望安全态
====================  ==========================  ========================
拔网线                拔掉机械臂网线              ``FAULT``
物体掉落              夹爪上的物体掉落            ``HOLDING``
夹爪夹空              夹爪空夹 / 未夹到            ``HOLDING``
关节超限位            手动撞限位 / 关节报错        ``FAULT``
供电跌落              断开供电或制造欠压          ``FAULT``
====================  ==========================  ========================

第五类「供电跌落」由**帧级系统错误码**（``errCode``）承载，reason 标签为
``power_supply_drop``（本项目自定的稳定标签，**不代表厂商错误码表**）。

两种模式
--------
* **默认（离线演练）**：用传输层替身
  （:class:`~rm75_drake.device.transport.ScriptedTransport`）精确注入五类故障，
  逐条走「注入 → 安全态 → 恢复 → 回到 READY 且 q 已重同步」，
  **并明确标注「物理故障未验证」**——流程正确 ≠ 物理故障已复现；
* **``--hardware``（真机）**：按 runbook 打印**逐步人工操作**，
  操作员执行物理动作后脚本检测故障 → 进入安全态 → 走恢复流程 → 校验 READY。

硬件门控
--------
真机不可达 ⇒ 输出 ``SKIP(reason=hardware_unreachable)`` 并**逐条**列出
未验证的真机指标，退出码 **0**；离线演练失败（流程回归）⇒ 退出码 **1**。

用法
----
::

    python3 scripts/fault_inject.py                 # 离线流程演练
    python3 scripts/fault_inject.py --hardware      # 真机人工编排
    python3 scripts/fault_inject.py --json

退出码
------
``0``  演练通过（或真机不可达时的 SKIP）；
``1``  至少一类故障的「注入 → 安全态 → 恢复」链路失败；
``2``  用法错误。
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

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
    resolve_device_config,
)
from rm75_drake.device.recovery import (  # noqa: E402
    REASON_GRIPPER_NOT_AT_TARGET,
    REASON_GRIPPER_OFFLINE,
    REASON_JOINT_ERROR,
    REASON_LINK_DOWN,
    REASON_POWER_SUPPLY_DROP,
    RecoveryProcedure,
    classify_fault,
    enter_safe_state,
)
from rm75_drake.device.sdk import audit_snapshot  # noqa: E402
from rm75_drake.device.state_machine import DeviceState, DeviceStateMachine  # noqa: E402
from rm75_drake.device.transport import (  # noqa: E402
    ScriptedTransport,
    SdkTransport,
    StateFrame,
)

EXIT_OK = 0
EXIT_FAILURE = 1
EXIT_USAGE = 2

#: 本脚本**覆盖**的指标（其它指标仍未验证，必须列出）。
COVERED_METRIC = "五类故障注入的物理验证（拔网线 / 物体掉落 / 夹爪夹空 / 关节超限位 / 供电跌落）"

#: 期望的夹爪位置（0–1000）——夹爪夹空判定用。
GRIPPER_TARGET = 500

#: 真机模式下等待人工注入故障的超时（s）。
INJECT_TIMEOUT_S = 30.0

#: 离线演练注入的帧级系统错误码（非 0 且非 ``-3`` ⇒ 系统错误）。
POWER_SUPPLY_DROP_ERR_CODE = 1


@dataclass(frozen=True)
class FaultCase:
    """一类故障的编排描述。

    字段
    ----
    key:
        稳定标识（脚本输出 / JSON 报告用）。
    label:
        人类可读名称。
    expected_reason:
        期望的故障 reason 标签。
    expected_state:
        期望进入的安全态。
    manual_steps:
        人工操作的逐步说明（真机模式打印）。
    """

    key: str
    label: str
    expected_reason: str
    expected_state: DeviceState
    manual_steps: Tuple[str, ...]


#: 五类故障（顺序即 runbook 里的操作顺序）。
FAULT_CASES: Tuple[FaultCase, ...] = (
    FaultCase(
        key="link_down",
        label="拔网线",
        expected_reason=REASON_LINK_DOWN,
        expected_state=DeviceState.FAULT,
        manual_steps=(
            "确认机械臂处于透传中（或准备透传）状态。",
            "拔掉机械臂网线（或断电重启控制器）。",
            "观察脚本是否检测到 link_down 并进入安全态。",
            "插回网线，等待控制器重新上线后执行恢复流程。",
        ),
    ),
    FaultCase(
        key="object_dropped",
        label="物体掉落（夹爪反馈丢失）",
        expected_reason=REASON_GRIPPER_OFFLINE,
        expected_state=DeviceState.HOLDING,
        manual_steps=(
            "夹爪夹持一个可掉落物体（离线时可用空夹爪模拟）。",
            "取走 / 掉落物体，使夹爪反馈丢失（rm_plus_state != 0）。",
            "观察脚本是否检测到 gripper_offline 并进入保持态。",
            "重新夹持 / 复位夹爪后执行恢复流程。",
        ),
    ),
    FaultCase(
        key="gripper_miss",
        label="夹爪夹空（位置未到位）",
        expected_reason=REASON_GRIPPER_NOT_AT_TARGET,
        expected_state=DeviceState.HOLDING,
        manual_steps=(
            "向夹爪下发一个夹持指令（目标位置见 --gripper-target）。",
            "让夹爪空夹（不夹物体），位置回读达不到目标。",
            "观察脚本是否检测到 gripper_not_at_target 并进入保持态。",
            "重新夹取到位后执行恢复流程。",
        ),
    ),
    FaultCase(
        key="joint_overlimit",
        label="关节超限位（逐关节错误码非 0）",
        expected_reason=REASON_JOINT_ERROR,
        expected_state=DeviceState.FAULT,
        manual_steps=(
            "确认急停手柄可用、周围无人。",
            "手动把某个关节推过软限位（或用厂商调试工具制造关节报错）。",
            "观察脚本是否检测到 joint_error 并进入故障态。",
            "按厂商流程清错后执行恢复流程。",
        ),
    ),
    FaultCase(
        key="power_supply_drop",
        label="供电跌落（帧级系统错误码）",
        expected_reason=REASON_POWER_SUPPLY_DROP,
        expected_state=DeviceState.FAULT,
        manual_steps=(
            "确认急停手柄可用、周围无人、机械臂处于可控姿态。",
            "按现场安全规程断开机械臂供电或制造欠压。",
            "观察脚本是否检测到 power_supply_drop 并进入故障态。",
            "恢复供电后按厂商顺序清错（rm_set_joint_clear_err → rm_clear_system_err）并执行恢复流程。",
        ),
    ),
)


def _frame(
    positions: Optional[Sequence[float]] = None,
    err_codes: Optional[Sequence[int]] = None,
    *,
    enabled: bool = True,
    received_at: float = 0.0,
    err_code: int = 0,
) -> StateFrame:
    """构造一帧状态（默认：全零位、无错、全使能）。

    ``err_code`` 是**帧级系统错误码**：非 0 且非 ``-3`` ⇒ 系统错误
    （本项目用它承载「供电跌落」）；``-3`` 是帧不完整标记。
    """
    return StateFrame(
        joint_positions_rad=tuple(positions or [0.0] * 7),
        joint_currents_ma=(0.0,) * 7,
        joint_en_flags=tuple([enabled] * 7),
        joint_err_codes=tuple(err_codes or [0] * 7),
        controller_status=0,
        gripper_online=True,
        arm_current_status=0,
        received_at=received_at,
        err_code=int(err_code),
    )


def _streaming_machine(frame: StateFrame) -> DeviceStateMachine:
    """构造一台已连上、已使能、正在透传的状态机。"""
    machine = DeviceStateMachine(clock=lambda: 1.0)
    machine.connect()
    machine.enable()
    machine.mark_all_enabled(frame)
    machine.start_stream()
    return machine


def _procedure(
    transport: Any, machine: DeviceStateMachine, limits: Any, gripper_target: int
) -> RecoveryProcedure:
    """构造恢复流程（软限位取自安全阈值——与执行环同一份）。"""
    return RecoveryProcedure(
        transport,
        machine,
        soft_lower_rad=limits.soft_lower_rad,
        soft_upper_rad=limits.soft_upper_rad,
        gripper_target=gripper_target,
    )


def _inject_offline(
    transport: ScriptedTransport, case: FaultCase, frame: StateFrame
) -> None:
    """在替身上精确注入故障（**五类故障一一对应**）。"""
    if case.key == "link_down":
        transport.drop_link()
    elif case.key == "object_dropped":
        transport.set_gripper_offline(True)
    elif case.key == "gripper_miss":
        transport.set_gripper_position(GRIPPER_TARGET // 2)
    elif case.key == "joint_overlimit":
        transport.push_frame(_frame(err_codes=[7, 0, 0, 0, 0, 0, 0]))
    elif case.key == "power_supply_drop":
        # 帧级系统错误码承载「供电跌落」：必须**真的推帧**，
        # 否则 read_state() 返回的仍是健康帧 ⇒ 故障检测不到。
        transport.set_system_error(POWER_SUPPLY_DROP_ERR_CODE)
        transport.push_frame(_frame(err_code=POWER_SUPPLY_DROP_ERR_CODE))
    else:  # pragma: no cover - 防御
        raise ValueError(f"未知故障类型：{case.key}")


def _fix_offline(
    transport: ScriptedTransport, case: FaultCase, frame: StateFrame
) -> None:
    """模拟操作员排除物理故障（恢复流程的前提）。

    无论哪类故障，恢复流程的第一步都包含「夹爪在线且已到位」
    （``check_gripper``）⇒ 这里统一把夹爪复位到「在线 + 已到位」，
    只把**该类故障**的物理条件修好。
    """
    if case.key == "link_down":
        transport.restore_link()
    elif case.key == "object_dropped":
        transport.set_gripper_offline(False)
    elif case.key == "gripper_miss":
        transport.set_gripper_position(GRIPPER_TARGET)
    elif case.key == "joint_overlimit":
        pass  # 错误码由恢复流程的 clear_error 步骤清除
    elif case.key == "power_supply_drop":
        # 模拟「供电已恢复」：帧级系统错误码归零（恢复流程的 clear_error 会再清一次）。
        transport.set_system_error(0)
    else:  # pragma: no cover - 防御
        raise ValueError(f"未知故障类型：{case.key}")
    transport.set_gripper_offline(False)
    transport.set_gripper_position(GRIPPER_TARGET)
    transport.push_frame(frame)


def rehearse_case(case: FaultCase, limits: Any) -> Dict[str, Any]:
    """离线演练一类故障：注入 → 安全态 → 恢复 → READY。

    返回
    ----
    dict：``key`` / ``detected_reason`` / ``safe_state`` / ``recovered`` /
    ``final_state`` / ``ok`` / ``detail``。
    """
    transport = ScriptedTransport()
    transport.open()
    healthy = _frame(received_at=time.monotonic())
    transport.push_frame(healthy)
    machine = _streaming_machine(healthy)

    _inject_offline(transport, case, healthy)
    diagnosis = classify_fault(transport, gripper_target=GRIPPER_TARGET)
    if diagnosis is None:
        return {
            "key": case.key,
            "label": case.label,
            "detected_reason": "",
            "safe_state": machine.state.value,
            "recovered": False,
            "final_state": machine.state.value,
            "ok": False,
            "detail": "故障未被检测到（分类器返回空）",
        }
    detected = diagnosis.reason
    state = enter_safe_state(transport, machine, diagnosis)

    _fix_offline(transport, case, _frame(positions=[0.3] * 7, received_at=time.monotonic()))
    result = _procedure(transport, machine, limits, GRIPPER_TARGET).run()

    ok = (
        detected == case.expected_reason
        and state == case.expected_state
        and result.ok
        and machine.state == DeviceState.READY
    )
    detail = (
        f"检测={detected}（期望 {case.expected_reason}），"
        f"安全态={state.value}（期望 {case.expected_state.value}），"
        f"恢复={'成功' if result.ok else '失败'}"
        + ("" if result.ok else f"：{result.diagnosis}")
    )
    return {
        "key": case.key,
        "label": case.label,
        "detected_reason": detected,
        "safe_state": state.value,
        "recovered": bool(result.ok),
        "final_state": machine.state.value,
        "ok": bool(ok),
        "detail": detail,
    }


def run_hardware_case(
    case: FaultCase, limits: Any, transport: Any, *, timeout_s: float
) -> Dict[str, Any]:
    """真机模式：人工注入 → 检测 → 安全态 → 恢复 → READY。"""
    print(f"[fault_inject] ==== {case.label} ====")
    for index, step in enumerate(case.manual_steps, start=1):
        print(f"  步骤 {index}：{step}")
    print("  （等待人工完成上述操作…）")
    try:
        input("  按回车继续（Ctrl-C 放弃）：")
    except EOFError:
        print("  （非交互环境：直接继续检测）")

    deadline = time.monotonic() + max(1.0, float(timeout_s))
    diagnosis = None
    while time.monotonic() < deadline:
        diagnosis = classify_fault(transport, gripper_target=GRIPPER_TARGET)
        if diagnosis is not None:
            break
        time.sleep(0.2)
    if diagnosis is None:
        return {
            "key": case.key,
            "label": case.label,
            "detected_reason": "",
            "safe_state": "",
            "recovered": False,
            "final_state": "",
            "ok": False,
            "detail": f"{timeout_s} s 内未检测到故障（人工操作未生效？）",
        }

    machine = DeviceStateMachine(clock=time.monotonic)
    state = enter_safe_state(transport, machine, diagnosis)
    result = _procedure(transport, machine, limits, GRIPPER_TARGET).run()
    ok = (
        diagnosis.reason == case.expected_reason
        and state == case.expected_state
        and result.ok
    )
    return {
        "key": case.key,
        "label": case.label,
        "detected_reason": diagnosis.reason,
        "safe_state": state.value,
        "recovered": bool(result.ok),
        "final_state": machine.state.value,
        "ok": bool(ok),
        "detail": result.diagnosis,
    }


def build_parser() -> argparse.ArgumentParser:
    """构造命令行解析器。"""
    parser = argparse.ArgumentParser(
        description="五类故障注入的人工编排与恢复流程校验（真机不可达时 SKIP）"
    )
    add_common_arguments(parser)
    parser.add_argument(
        "--hardware",
        action="store_true",
        help="真机模式：按 runbook 逐步人工注入故障并校验恢复流程",
    )
    parser.add_argument(
        "--only",
        default=None,
        choices=[case.key for case in FAULT_CASES],
        help="只演练其中一类故障（默认全部五类）",
    )
    parser.add_argument(
        "--inject-timeout-s",
        type=float,
        default=INJECT_TIMEOUT_S,
        help=f"真机模式等待人工注入故障的超时（s，默认：{INJECT_TIMEOUT_S}）",
    )
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    """脚本入口（返回退出码）。"""
    parser = build_parser()
    args = parser.parse_args(list(argv) if argv is not None else None)

    try:
        device_config = resolve_device_config(args.device_config)
    except DeviceConfigError as exc:
        print(f"fault_inject: 设备配置非法：{exc}", file=sys.stderr)
        return EXIT_USAGE

    limits = build_safety_limits(device_config)
    cases = [
        case for case in FAULT_CASES if args.only is None or case.key == args.only
    ]

    hardware_mode = bool(args.hardware) and not bool(args.dry_run)
    reachable = False
    skip_reason = ""
    skip_detail = ""
    if not bool(args.dry_run):
        probe = probe_hardware(device_config, timeout=float(args.connect_timeout))
        reachable = probe.reachable
        if not reachable:
            skip_reason = probe.reason
            skip_detail = probe.detail
            if not args.json:
                print(format_skip(probe))

    results: List[Dict[str, Any]] = []
    transport = None
    if hardware_mode and reachable:
        transport = SdkTransport(device_config)
        try:
            transport.open()
        except Exception as exc:
            print(f"fault_inject: 传输层打开失败：{exc!r}", file=sys.stderr)
            return EXIT_FAILURE
        try:
            for case in cases:
                results.append(
                    run_hardware_case(
                        case,
                        limits,
                        transport,
                        timeout_s=float(args.inject_timeout_s),
                    )
                )
        finally:
            transport.close()
    else:
        for case in cases:
            results.append(rehearse_case(case, limits))

    failed = [item for item in results if not item["ok"]]
    report = {
        "mode": "hardware" if hardware_mode and reachable else "offline_rehearsal",
        "physical_faults_verified": bool(hardware_mode and reachable),
        "hardware_reachable": bool(reachable),
        "skip_reason": skip_reason,
        "skip_detail": skip_detail,
        "cases": results,
        "failed": [item["key"] for item in failed],
        # 运行时调用审计（R008 运行时那半）：真机路径的 SDK 调用计数必须可读。
        "sdk_audit": audit_snapshot(getattr(transport, "audit", None)),
        "unverified_metrics": (
            [m for m in UNVERIFIED_HARDWARE_METRICS if m != COVERED_METRIC]
            if hardware_mode and reachable
            else list(UNVERIFIED_HARDWARE_METRICS)
        ),
    }

    if args.json:
        print(json.dumps(report, ensure_ascii=False, indent=2))
    else:
        mode = "真机（人工注入）" if report["physical_faults_verified"] else "离线演练"
        print(f"[fault_inject] 模式：{mode}")
        if not report["physical_faults_verified"]:
            print("[fault_inject] **物理故障未验证**（本次只演练了软件流程）")
        for item in results:
            mark = "OK" if item["ok"] else "FAIL"
            print(
                f"[fault_inject] [{mark}] {item['label']}：{item['detail']}"
                f"（最终状态 {item['final_state']}）"
            )
        print("以下真机指标**未验证**（不得当作通过）：")
        for item in report["unverified_metrics"]:
            print(f"  - {item}")

    return EXIT_OK if not failed else EXIT_FAILURE


if __name__ == "__main__":
    sys.exit(main())
