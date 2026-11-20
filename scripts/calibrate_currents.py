#!/usr/bin/env python3
"""真机**电流阈值标定**（安全闸 G5 的真值来源）。

为什么需要这个脚本
------------------
`devices/*.yaml` 里的 `current_thresholds` 目前是**保守兜底值**
（J1/J2 1500 mA、J3/J4 1200 mA、J5–J7 1000 mA，`calibrated: false`）。
真机实测（RM75-B / 192.168.1.18）发现：**静态重力保持姿态下 J2 电流就已达
-1413 ~ -1824 mA** ⇒ 兜底阈值**低于**真机的静态工作电流 ⇒
安全闸 G5 在真机上会**误触发停机**（或长期处于告警态）。

厂商未公布 RM75-B 的逐轴电流上限 ⇒ 只能用**实测包络 + 安全系数**标定：

    阈值[i] = 安全系数 × 实测 |电流| 包络[i]

默认安全系数 **1.5**（可调）。标定结果用 ``--write`` 回写到设备配置
（``current_thresholds.values`` + ``calibrated: true``）。

用法
----
::

    # ① 只测当前姿态的静态电流（零运动）
    python3 scripts/calibrate_currents.py \
        --device-config devices/rm75_b_192_168_1_18.yaml --duration-s 20 --json

    # ② 含小幅点动的包络（需要授权）
    python3 scripts/calibrate_currents.py --device-config devices/... \
        --allow-motion --jog-deg 5 --duration-s 10 --json

    # ③ 回写标定值
    python3 scripts/calibrate_currents.py --device-config devices/... \
        --duration-s 20 --write --json

退出码
------
``0`` 标定完成（或写入成功）；``1`` 真机不可达 / 采样不足；
``2`` 用法错误。
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = REPO_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from rm75_drake.device.config import DeviceConfigError, load_device_config  # noqa: E402
from rm75_drake.device.hardware_gate import format_skip, probe_hardware  # noqa: E402
from rm75_drake.device.sdk import deg_to_rad  # noqa: E402

EXIT_OK = 0
EXIT_FAILURE = 1
EXIT_USAGE = 2

#: 默认安全系数（实测包络 × 系数 = 阈值）。
DEFAULT_SAFETY_FACTOR = 1.5

#: 阈值下限（mA）——小关节（J5–J7）静态电流只有几十~几百 mA，
#: 包络×系数会得到过紧的阈值（一次轻微姿态变化就误触发）⇒ 取下限。
DEFAULT_FLOOR_MA = 600.0

#: 采样不足时的最小帧数。
MIN_SAMPLES = 50

#: 「放松电流闸」用的有限大阈值（mA）——标定时让轨迹跑完，采真实包络。
RELAXED_CURRENT_MA = 1.0e9

#: 包络默认取的分位数（真机存在单点尖峰 ⇒ 不用 max）。
DEFAULT_PERCENTILE = 0.995

#: 电流闸去抖拍数（与驱动层 RmDeviceDriverConfig.current_stop_after_ticks 一致）。
DEFAULT_DEBOUNCE_TICKS = 3


def build_parser() -> argparse.ArgumentParser:
    """构造命令行解析器。"""
    parser = argparse.ArgumentParser(
        description="真机电流阈值标定（实测包络 × 安全系数 → 回写设备配置）"
    )
    parser.add_argument("--device-config", required=True, help="设备配置路径")
    parser.add_argument(
        "--duration-s", type=float, default=20.0, help="每个姿态的采样时长（s，默认 20）"
    )
    parser.add_argument(
        "--jog-deg",
        type=float,
        default=0.0,
        help="点动幅度（度，0 = 不动）；>0 时需要 --allow-motion",
    )
    parser.add_argument(
        "--allow-motion", action="store_true", help="授权点动（会动真机）"
    )
    parser.add_argument(
        "--safety-factor",
        type=float,
        default=DEFAULT_SAFETY_FACTOR,
        help=f"安全系数（默认 {DEFAULT_SAFETY_FACTOR}）",
    )
    parser.add_argument(
        "--percentile",
        type=float,
        default=DEFAULT_PERCENTILE,
        help=(
            "包络取的分位数（默认 "
            f"{DEFAULT_PERCENTILE}）：真机实测存在**单点尖峰**"
            "（J1 偶发 11988 mA，而 p95 只有 452 mA）⇒ 用分位数而不是 max，"
            "免得阈值被尖峰撑大到失去保护意义"
        ),
    )
    parser.add_argument(
        "--debounce-ticks",
        type=int,
        default=DEFAULT_DEBOUNCE_TICKS,
        help=(
            "去抖拍数（默认 "
            f"{DEFAULT_DEBOUNCE_TICKS}）：驱动层的 G5 电流闸是"
            "「连续 N 拍越限才停机」⇒ 包络必须按**同样**的口径统计"
            "（窗口内最小值的最大值），否则单拍尖峰会把阈值撑爆"
        ),
    )
    parser.add_argument(
        "--floor-ma",
        type=float,
        default=DEFAULT_FLOOR_MA,
        help=(
            "阈值下限（mA，默认 "
            f"{DEFAULT_FLOOR_MA}）：小关节静态电流极小，"
            "包络×系数会得到过紧的阈值 ⇒ 取 max(包络×系数, 下限)"
        ),
    )
    parser.add_argument(
        "--merge-with",
        default=None,
        help="与另一次标定的 JSON 结果合并（逐轴取更大的包络）——多次采样取并集更稳",
    )
    parser.add_argument(
        "--recording",
        default=None,
        help=(
            "执行一条录制轨迹（真机）并用它的**动态电流**补齐包络："
            "静态保持测不到**加速电流**（真机实测：J1 加速时 1072 mA "
            "> 静态包络 645 mA）⇒ 阈值必须覆盖动态包络"
        ),
    )
    parser.add_argument(
        "--write", action="store_true", help="把标定结果回写进设备配置"
    )
    parser.add_argument("--json", action="store_true", help="以 JSON 输出")
    return parser


def _debounced_envelope(
    values: Sequence[float], window: int, percentile: float = 0.995
) -> float:
    """去抖口径的包络（**按时间顺序**算窗口最小值，再取分位数）。

    驱动层的 G5 是「连续 N 拍越限才停机」⇒ 只有当某个 N 拍窗口
    **全部**越限时才会停机 ⇒ 阈值必须覆盖「窗口内最小值」的**持续水平**。

    取分位数（而不是最大值）是为了再滤掉「连续 N 拍都高」的偶发尖峰
    ——真机实测：这种尖峰在 J1 上可达 4–10 A，且持续拍数不定，
    用最大值会把阈值撑到失去保护意义。
    """
    if not values:
        return 0.0
    if len(values) <= window:
        return float(min(values))
    mins = [
        float(min(values[start : start + window]))
        for start in range(len(values) - window + 1)
    ]
    mins.sort()
    index = min(len(mins) - 1, int(len(mins) * percentile))
    return float(mins[index])


def main(argv: Optional[Sequence[str]] = None) -> int:
    """脚本入口（返回退出码）。"""
    parser = build_parser()
    args = parser.parse_args(list(argv) if argv is not None else None)

    if float(args.duration_s) <= 0.0:
        print("calibrate_currents: --duration-s 必须为正数", file=sys.stderr)
        return EXIT_USAGE
    if float(args.safety_factor) < 1.0:
        print("calibrate_currents: --safety-factor 必须 ≥ 1.0", file=sys.stderr)
        return EXIT_USAGE
    if float(args.jog_deg) > 0.0 and not args.allow_motion:
        print(
            "calibrate_currents: 点动需要显式授权（--allow-motion）",
            file=sys.stderr,
        )
        return EXIT_USAGE

    try:
        config = load_device_config(Path(args.device_config))
    except DeviceConfigError as exc:
        print(f"calibrate_currents: 设备配置非法：{exc}", file=sys.stderr)
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
    from rm75_drake.device.realtime import RealtimePushSettings, RealtimeStateReceiver
    from rm75_drake.device.state_source import StateSource
    from rm75_drake.device.transport import SdkTransport
    from rm75_drake.validation.hardware import detect_local_ip

    binding = ArmBinding(config)
    report = binding.open()
    if not report.connected:
        print(f"calibrate_currents: 连接失败：{report.detail}", file=sys.stderr)
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

    samples: List[List[float]] = []
    static_samples: List[List[float]] = []
    dynamic_samples: List[List[float]] = []
    poses: List[Dict[str, Any]] = []
    dynamic: Dict[str, Any] = {}

    def _sample(seconds: float) -> None:
        deadline = time.monotonic() + float(seconds)
        while time.monotonic() < deadline:
            frame = source.latest()
            if frame is not None:
                row = [abs(float(v)) for v in frame.joint_currents_ma]
                samples.append(row)
                static_samples.append(row)
            time.sleep(0.01)

    try:
        frame = source.latest()
        if frame is None:
            print("calibrate_currents: 没有收到 UDP 状态帧", file=sys.stderr)
            return EXIT_FAILURE
        origin = tuple(float(v) for v in frame.joint_positions_rad)
        poses.append({"name": "origin", "joint_rad": list(origin)})
        _sample(float(args.duration_s))

        if float(args.jog_deg) > 0.0:
            delta = deg_to_rad(float(args.jog_deg))
            for axis in (0, 1, 2, 3):
                for sign in (1.0, -1.0):
                    target = list(origin)
                    target[axis] = origin[axis] + sign * delta
                    _hold(transport, target, 1.0)
                    poses.append(
                        {
                            "name": f"jog_axis{axis + 1}_{'plus' if sign > 0 else 'minus'}",
                            "joint_rad": target,
                        }
                    )
                    _sample(float(args.duration_s))
                    _hold(transport, origin, 1.0)

        if args.recording:
            dynamic = _run_recording_for_envelope(
                transport, source, config, str(args.recording), dynamic_samples
            )
            samples.extend(dynamic_samples)
    finally:
        receiver.stop()
        transport.close()

    if len(samples) < MIN_SAMPLES:
        print(
            f"calibrate_currents: 采样不足（{len(samples)} < {MIN_SAMPLES}）",
            file=sys.stderr,
        )
        return EXIT_FAILURE

    percentile = float(args.percentile)
    if not 0.5 <= percentile < 1.0:
        print("calibrate_currents: --percentile 必须在 [0.5, 1.0) 之间", file=sys.stderr)
        return EXIT_USAGE

    debounce = int(args.debounce_ticks)
    if debounce < 1:
        print("calibrate_currents: --debounce-ticks 必须 ≥ 1", file=sys.stderr)
        return EXIT_USAGE

    def _group_envelope(group: Sequence[Sequence[float]]) -> List[float]:
        """一组样本的逐轴**去抖**包络（与驱动层的判定口径一致）。

        注意：窗口必须在**时间顺序**上滑动 ⇒ 这里用原始顺序，不排序。
        """
        out: List[float] = []
        for axis in range(7):
            values = [float(row[axis]) for row in group]
            out.append(_debounced_envelope(values, debounce, percentile))
        return out

    # 包络 = max(静态包络, 动态包络)——**必须分组**：
    # 混在一起算分位数会把动态尖峰稀释掉（真机实测：轨迹只有 53 帧，
    # 混进 5000+ 静态帧后 J1 的分位数从 2800 mA 掉到 569 mA）。
    static_env = _group_envelope(static_samples)
    dynamic_env = _group_envelope(dynamic_samples)
    envelope: List[float] = [
        max(a, b) for a, b in zip(static_env, dynamic_env)
    ]
    stats: List[Dict[str, float]] = []
    for axis in range(7):
        values = sorted(row[axis] for row in samples)
        p95 = values[int(len(values) * 0.95)]
        worst = values[-1]
        stats.append(
            {
                "axis": axis + 1,
                "abs_current_p50_ma": values[len(values) // 2],
                "abs_current_p95_ma": p95,
                "abs_current_p99_ma": values[
                    min(len(values) - 1, int(len(values) * 0.99))
                ],
                "static_envelope_ma": static_env[axis],
                "dynamic_envelope_ma": dynamic_env[axis],
                "abs_current_envelope_ma": envelope[axis],
                "abs_current_max_ma": worst,
            }
        )

    factor = float(args.safety_factor)
    floor = float(args.floor_ma)
    merged = list(envelope)
    merged_from: List[str] = []
    if args.merge_with:
        other_path = Path(args.merge_with)
        try:
            text = other_path.read_text(encoding="utf-8")
            other = json.loads(text[text.index("{") :]) if "{" in text else json.loads(text)
        except Exception as exc:
            print(f"calibrate_currents: 合并文件解析失败：{exc}", file=sys.stderr)
            return EXIT_USAGE
        other_envelope = [float(v) for v in other.get("envelope_ma", [])]
        if len(other_envelope) == 7:
            merged = [max(a, b) for a, b in zip(merged, other_envelope)]
            merged_from.append(str(other_path))
    proposed = [max(round(value * factor, 1), floor) for value in merged]
    result = {
        "status": "ok",
        "device": config.to_dict(),
        "samples": len(samples),
        "duration_s": float(args.duration_s),
        "poses": poses,
        "per_axis": stats,
        "envelope_ma": [round(v, 1) for v in envelope],
        "static_envelope_ma": [round(v, 1) for v in static_env],
        "dynamic_envelope_ma": [round(v, 1) for v in dynamic_env],
        "merged_envelope_ma": [round(v, 1) for v in merged],
        "merged_with": merged_from,
        "dynamic": dynamic,
        "safety_factor": factor,
        "floor_ma": floor,
        "percentile": percentile,
        "debounce_ticks": debounce,
        "proposed_thresholds_ma": proposed,
        "current_thresholds_calibrated": bool(args.write),
    }

    if args.write:
        result["written_to"] = str(Path(args.device_config))
        result["write_status"] = _write_thresholds(
            Path(args.device_config), proposed
        )

    if args.json:
        print(json.dumps(result, ensure_ascii=False, indent=2))
    else:
        print(f"[calibrate_currents] 采样 {len(samples)} 帧（{len(poses)} 个姿态）")
        for item in stats:
            print(
                f"  joint_{item['axis']}: p50={item['abs_current_p50_ma']:.1f} "
                f"p95={item['abs_current_p95_ma']:.1f} "
                f"max={item['abs_current_max_ma']:.1f} mA"
            )
        print(f"[calibrate_currents] 建议阈值（×{factor}）：{proposed} mA")
        if args.write:
            print(f"[calibrate_currents] 已回写：{args.device_config}")
    return EXIT_OK


def _run_recording_for_envelope(
    transport: Any,
    source: Any,
    config: Any,
    recording_path: str,
    samples: List[List[float]],
) -> Dict[str, Any]:
    """执行一条录制轨迹，采集**动态电流**（含加速电流）并追加进 ``samples``。

    电流闸在本次执行中被**故意放宽**（阈值置为无穷大）⇒ 轨迹能跑完，
    采集到的就是「该运动下的真实电流包络」。

    返回
    ----
    dict：执行摘要（点数 / 最终相位 / 是否被安全闸拦下）。
    """
    import dataclasses

    from rm75_drake.drivers.rm_device import RmDeviceDriver, RmDeviceDriverConfig
    from rm75_drake.limits import load_joint_limits
    from rm75_drake.planning.trajectory import trajectory_from_arrays
    from rm75_drake.recording import load_recording
    from rm75_drake.device.safety import safety_limits_from_sources

    recording = load_recording(recording_path)
    trajectory = trajectory_from_arrays(
        recording.q_matrix(),
        t0=recording.samples[0].t_abs,
        dt=recording.dt,
        robot_name=recording.robot_name,
        gripper_schedule=recording.gripper_schedule,
    )
    limits = safety_limits_from_sources(load_joint_limits(), config)
    # 阈值置成一个**有限的大值**（安全阈值类不允许 inf：NaN/Inf 会被拒）。
    relaxed = dataclasses.replace(
        limits, current_threshold_ma=(RELAXED_CURRENT_MA,) * 7
    )
    driver = RmDeviceDriver(
        transport,
        limits=relaxed,
        config=RmDeviceDriverConfig(require_state=True),
        dt=float(trajectory.dt),
    )
    driver.open()
    summary: Dict[str, Any] = {"recording": str(recording_path)}
    try:
        driver.submit_trajectory(trajectory)
        while driver.index < driver.num_samples:
            driver.step()
            frame = driver.read_device_frame()
            if frame is not None:
                samples.append([abs(float(v)) for v in frame.joint_currents_ma])
        summary["final_phase"] = driver.phase
        summary["num_samples"] = int(driver.num_samples)
        summary["violations"] = [str(v.reason) for v in driver.violations]
    finally:
        driver.close()
    return summary


def _hold(transport: Any, target: Sequence[float], seconds: float) -> None:
    """持续下发保持位姿（透传 + follow）。"""
    deadline = time.monotonic() + float(seconds)
    while time.monotonic() < deadline:
        transport.move_j_canfd(tuple(float(v) for v in target), follow=True)
        time.sleep(0.05)


def _write_thresholds(path: Path, thresholds: Sequence[float]) -> str:
    """把标定值回写进设备配置（保持 YAML 结构与注释风格）。

    只改两处：``current_thresholds.values`` 与 ``current_thresholds.calibrated``。
    """
    text = path.read_text(encoding="utf-8")
    lines = text.splitlines()
    out: List[str] = []
    in_thresholds = False
    wrote_values = False
    wrote_flag = False
    for line in lines:
        stripped = line.strip()
        if stripped.startswith("current_thresholds:"):
            in_thresholds = True
            out.append(line)
            continue
        if in_thresholds and stripped.startswith("calibrated:"):
            out.append("  calibrated: true")
            wrote_flag = True
            continue
        if in_thresholds and stripped.startswith("values:"):
            out.append(f"  values: {[float(v) for v in thresholds]}")
            wrote_values = True
            in_thresholds = False
            continue
        out.append(line)
    if not (wrote_values and wrote_flag):
        return "failed"
    path.write_text("\n".join(out) + "\n", encoding="utf-8")
    return "ok"


if __name__ == "__main__":
    from rm75_drake.device.teardown import safe_exit

    # 真机路径：SDK 的原生线程会在解释器退出阶段回调已释放对象 ⇒ 段错误。
    # 证据必须先落盘 ⇒ 刷新后直接退出（见 device/teardown.py）。
    safe_exit(main())
