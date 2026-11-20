#!/usr/bin/env python3
"""真机 **急停态故障注入**（estop_status）：检测 → 安全态 → 恢复 → READY。

为什么单独做这个
----------------
《设计文档》§九 的「故障注入」五类里，**四类**都需要物理动作
（拔网线 / 物体掉落 / 夹爪夹空 / 关节超限位 / 供电跌落）⇒ 现场必须有人配合。
但**急停态**是厂商 API 可**程序触发**的（``rm_set_arm_emergency_stop(True)``）：
控制器进入急停 ⇒ UDP 帧的 ``arm_current_status`` 变成 9 ⇒
:class:`~rm75_drake.device.recovery.classify_fault` 判定 ``estop_status``
⇒ 进入 ESTOP 安全态 ⇒ 走恢复流程回到 READY。

⇒ 本脚本在**真机**上把「故障检测 → 安全态 → 恢复」这条链路完整跑一遍
（不改厂商错误码语义、不替代物理急停）。

用法
----
::

    python3 scripts/estop_fault_test.py --device-config devices/rm75_b_192_168_1_18.yaml --json

退出码
------
``0`` 检测→安全态→恢复全链路通过；``1`` 链路未闭合；``2`` 用法错误。
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


def build_parser() -> argparse.ArgumentParser:
    """构造命令行解析器。"""
    parser = argparse.ArgumentParser(
        description="真机急停态故障注入（estop_status）：检测 → 安全态 → 恢复 → READY"
    )
    parser.add_argument("--device-config", required=True, help="设备配置路径")
    parser.add_argument(
        "--detect-timeout-s",
        type=float,
        default=5.0,
        help="触发急停后等待状态上报反映出来的时间（s，默认 5）",
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
        print(f"estop_fault_test: 设备配置非法：{exc}", file=sys.stderr)
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
    from rm75_drake.device.state_machine import DeviceStateMachine
    from rm75_drake.device.state_source import StateSource
    from rm75_drake.device.transport import SdkTransport

    binding = ArmBinding(config)
    report = binding.open()
    if not report.connected:
        print(f"estop_fault_test: 连接失败：{report.detail}", file=sys.stderr)
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

    result: Dict[str, Any] = {"status": "failed", "stages": {}}
    try:
        frame = source.latest()
        result["stages"]["before"] = {
            "arm_current_status": int(frame.arm_current_status) if frame else None,
        }
        # ── 注入：厂商接口触发急停 ────────────────────────────────
        transport.emergency_stop()
        deadline = time.monotonic() + float(args.detect_timeout_s)
        diagnosis = None
        while time.monotonic() < deadline:
            diagnosis = classify_fault(transport)
            if diagnosis is not None:
                break
            time.sleep(0.1)
        result["stages"]["detected"] = {
            "reason": diagnosis.reason if diagnosis else "",
            "detail": diagnosis.detail if diagnosis else "未检测到故障",
        }
        if diagnosis is None or diagnosis.reason != "estop_status":
            result["status"] = "failed"
            result["detail"] = "未检测到 estop_status 故障"
            return EXIT_FAILURE

        machine = DeviceStateMachine(clock=time.monotonic)
        state = enter_safe_state(transport, machine, diagnosis)
        result["stages"]["safe_state"] = {"state": state.value}

        # ── 恢复：官方顺序（清错 → 放宽限位 → 上使能 → 回位 → 恢复限位）──
        limits = _limits(config)
        recovery = RecoveryProcedure(
            transport=transport,
            machine=machine,
            soft_lower_rad=limits.soft_lower_rad,
            soft_upper_rad=limits.soft_upper_rad,
            gripper_target=None,
        ).run()
        result["stages"]["recovery"] = {
            "ok": bool(recovery.ok),
            "final_state": machine.state.value,
            "detail": recovery.diagnosis,
            "steps": [
                {
                    "name": step.name,
                    "status": step.status,
                    "detail": step.detail,
                }
                for step in recovery.steps
            ],
        }
        result["status"] = "ok" if recovery.ok else "failed"
    finally:
        receiver.stop()
        transport.close()

    if args.json:
        print(json.dumps(result, ensure_ascii=False, indent=2))
    else:
        print(f"[estop_fault_test] 状态：{result['status']}")
        for name, payload in result["stages"].items():
            print(f"  {name}: {json.dumps(payload, ensure_ascii=False)}")
    return EXIT_OK if result["status"] == "ok" else EXIT_FAILURE


def _limits(config: Any) -> Any:
    """构造安全七闸阈值（软限位 + 设备配置）。"""
    from rm75_drake.device.safety import safety_limits_from_sources
    from rm75_drake.limits import load_joint_limits

    return safety_limits_from_sources(load_joint_limits(), config)


if __name__ == "__main__":
    from rm75_drake.device.teardown import safe_exit

    safe_exit(main())
