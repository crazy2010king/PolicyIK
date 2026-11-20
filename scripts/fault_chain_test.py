#!/usr/bin/env python3
"""真机**故障链**验证：注入 → 检测 → 安全态 → 恢复 → READY。

为什么要有这个脚本
------------------
《设计文档》§九 的「故障注入」五类里，四类需要**人手物理操作**
（拔网线 / 物体掉落 / 夹爪夹空 / 关节超限位 / 供电跌落）
⇒ 无人值守时不能自动做。但「**夹爪未到位**」（``gripper_not_at_target``）
这一类的**触发条件可以由软件制造**：把夹爪指令到 0（闭合），
空夹时实际停在 12 左右 ≠ 0 ⇒ 位置回读判定成立 ⇒ 故障链可以**全自动**跑完。

可跑的 case
-----------
``gripper_not_at_target``
    夹爪闭合到 0 ⇒ 回读 ≠ 目标 ⇒ 检测 → HOLDING 安全态 → 恢复 → READY。
``estop``
    用厂商接口触发急停。⚠️ 真机实测（控制器代次 3）：
    ``rm_set_arm_emergency_stop(True)`` 返回 **-4**（控制器超时拒绝）
    ⇒ 这台设备**不支持软件急停**，急停必须用**物理急停**；
    本 case 会如实报告「未检测到故障」，并提示改用物理急停。

用法
----
::

    python3 scripts/fault_chain_test.py --device-config devices/rm75_b_192_168_1_18.yaml \\
        --case gripper_not_at_target --json

退出码
------
``0`` 故障链完整闭合（检测 → 安全态 → 恢复 → READY）；
``1`` 链路未闭合 / 故障未被检测到；
``2`` 用法错误。
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any, Dict, Optional, Sequence

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = REPO_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from rm75_drake.device.config import DeviceConfigError, load_device_config  # noqa: E402
from rm75_drake.device.hardware_gate import format_skip, probe_hardware  # noqa: E402

EXIT_OK = 0
EXIT_FAILURE = 1
EXIT_USAGE = 2

#: 夹爪闭合目标（0–1000 无量纲）。
GRIPPER_CLOSE_TARGET = 0
#: 夹爪张开目标。
GRIPPER_OPEN_TARGET = 1000


def build_parser() -> argparse.ArgumentParser:
    """构造命令行解析器。"""
    parser = argparse.ArgumentParser(
        description="真机故障链验证（注入 → 检测 → 安全态 → 恢复 → READY）"
    )
    parser.add_argument("--device-config", required=True, help="设备配置路径")
    parser.add_argument(
        "--case",
        default="gripper_not_at_target",
        choices=["gripper_not_at_target", "estop"],
        help="要验证的故障类别（默认 gripper_not_at_target）",
    )
    parser.add_argument(
        "--detect-timeout-s",
        type=float,
        default=8.0,
        help="触发故障后等待检测到的时间（s，默认 8）",
    )
    parser.add_argument("--json", action="store_true", help="以 JSON 输出")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    """脚本入口（返回退出码）。"""
    parser = build_parser()
    args = parser.parse_args(list(argv) if argv is not None else None)

    try:
        config = load_device_config(Path(args.device_config))
    except DeviceConfigError as exc:
        print(f"fault_chain_test: 设备配置非法：{exc}", file=sys.stderr)
        return EXIT_USAGE

    probe = probe_hardware(config, timeout=2.0)
    if not probe.reachable:
        if args.json:
            print(json.dumps({"status": "skipped", "detail": probe.detail},
                             ensure_ascii=False, indent=2))
        else:
            print(format_skip(probe))
        return EXIT_FAILURE

    from rm75_drake.device.binding import ArmBinding
    from rm75_drake.device.realtime import (
        RealtimePushSettings,
        RealtimeStateReceiver,
        detect_local_ip,
    )
    from rm75_drake.device.recovery import (
        RecoveryProcedure,
        classify_fault,
        enter_safe_state,
    )
    from rm75_drake.device.safety import safety_limits_from_sources
    from rm75_drake.device.state_machine import DeviceStateMachine
    from rm75_drake.device.state_source import StateSource
    from rm75_drake.device.transport import SdkTransport
    from rm75_drake.limits import load_joint_limits

    binding = ArmBinding(config)
    report = binding.open()
    if not report.connected:
        print(f"fault_chain_test: 连接失败：{report.detail}", file=sys.stderr)
        return EXIT_FAILURE

    source = StateSource()
    receiver = RealtimeStateReceiver(
        binding,
        source,
        RealtimePushSettings(
            ip=detect_local_ip(config.arm_ip, config.tcp_port),
            port=int(config.udp_port),
            cycle=int(config.udp_cycle),
        ),
    )
    receiver.start()
    time.sleep(0.5)

    transport = SdkTransport(config, binding=binding)
    transport.open()
    transport.attach_state_source(source)

    result: Dict[str, Any] = {"status": "failed", "case": args.case, "stages": {}}

    def _log(text: str) -> None:
        if not args.json:
            print(f"[fault_chain_test] {text}", flush=True)

    try:
        frame = source.latest()
        result["stages"]["before"] = {
            "arm_current_status": int(frame.arm_current_status) if frame else None,
            "joint_positions_rad": (
                [round(float(v), 6) for v in frame.joint_positions_rad]
                if frame
                else None
            ),
        }

        # ── 注入 ────────────────────────────────────────────────────
        if args.case == "gripper_not_at_target":
            _log(f"注入夹爪未到位：闭合到 {GRIPPER_CLOSE_TARGET}（空夹）…")
            transport.command_gripper_position(GRIPPER_CLOSE_TARGET)
            time.sleep(1.5)
            gripper = transport.read_gripper()
            result["stages"]["injected"] = {
                "gripper_target": GRIPPER_CLOSE_TARGET,
                "gripper_readback": int(gripper.position),
                "gripper_online": bool(gripper.online),
            }
            _log(
                f"夹爪回读 {gripper.position}（目标 {GRIPPER_CLOSE_TARGET}）"
                f"⇒ 位置差 {abs(int(gripper.position) - GRIPPER_CLOSE_TARGET)}"
            )
        else:
            _log("注入急停（rm_set_arm_emergency_stop(True)）…")
            transport.emergency_stop()
            result["stages"]["injected"] = {"api": "rm_set_arm_emergency_stop(True)"}
            _log(
                "注意：本控制器实测返回 -4（拒绝软件急停）⇒ 急停必须用物理急停"
            )

        # ── 检测 ────────────────────────────────────────────────────
        deadline = time.monotonic() + float(args.detect_timeout_s)
        diagnosis = None
        while time.monotonic() < deadline:
            diagnosis = classify_fault(
                transport, gripper_target=GRIPPER_CLOSE_TARGET
            )
            if diagnosis is not None:
                break
            time.sleep(0.1)
        result["stages"]["detected"] = {
            "reason": diagnosis.reason if diagnosis else "",
            "detail": diagnosis.detail if diagnosis else "未检测到故障",
        }
        _log(
            "检测结果："
            + (diagnosis.reason if diagnosis else "未检测到故障")
        )
        if diagnosis is None:
            result["status"] = "failed"
            result["detail"] = (
                "未检测到故障（软件急停在本控制器上不生效 ⇒ 急停态需物理急停 + 人手确认）"
                if args.case == "estop"
                else "未检测到夹爪未到位"
            )
            return EXIT_FAILURE

        # ── 安全态 ──────────────────────────────────────────────────
        _log("进入安全态…")
        machine = DeviceStateMachine(clock=time.monotonic)
        # 状态机必须先走到 READY（链路已通、7 轴已使能），
        # 否则「hold 类」故障的安全态转移没有合法起点（会停在 disconnected）。
        machine.connect()
        current_frame = source.latest()
        if current_frame is not None:
            machine.enable()
            machine.mark_all_enabled(current_frame)
        state = enter_safe_state(transport, machine, diagnosis)
        result["stages"]["safe_state"] = {"state": state.value}
        _log(f"安全态：{state.value}")

        # ── 恢复 ────────────────────────────────────────────────────
        limits = safety_limits_from_sources(load_joint_limits(), config)
        home = None
        current = source.latest()
        if current is not None:
            home = tuple(float(v) for v in current.joint_positions_rad)
        _log("执行恢复流程（官方顺序）…")
        recovery = RecoveryProcedure(
            transport=transport,
            machine=machine,
            home_position_rad=home,
            soft_lower_rad=limits.soft_lower_rad,
            soft_upper_rad=limits.soft_upper_rad,
            gripper_target=None,
        ).run()
        result["stages"]["recovery"] = {
            "ok": bool(recovery.ok),
            "final_state": machine.state.value,
            "failed_step": recovery.failed_step,
            "detail": recovery.diagnosis,
            "steps": [
                {"name": step.name, "status": step.status, "detail": step.detail}
                for step in recovery.steps
            ],
        }
        result["status"] = "ok" if recovery.ok else "failed"
        _log(f"恢复：ok={recovery.ok} final={machine.state.value}")
    finally:
        _log("收尾…")
        receiver.stop()
        transport.close()

    if args.json:
        print(json.dumps(result, ensure_ascii=False, indent=2))
    else:
        print(f"[fault_chain_test] 结果：{result['status']}")
    return EXIT_OK if result["status"] == "ok" else EXIT_FAILURE


if __name__ == "__main__":
    from rm75_drake.device.teardown import safe_exit

    safe_exit(main())
