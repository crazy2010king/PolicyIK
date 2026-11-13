"""CLI 族（S02-T08）：``single`` / ``pick`` / ``bimanual`` / ``replay``。

统一入口
--------
::

    python3 -m rm75_drake.cli <subcommand> [options]

四个子命令的语义
----------------
``single``
    单臂规划：走完整链路（IK → IRIS → 空间 GCS → Toppra 重定时 → 碰撞复检），
    打印诊断，可选把结果录制成 JSON（``--output``）。
``pick``
    预抓位姿规划：在预抓位姿规划并给出**开爪动作调度**
    （:func:`rm75_drake.gripper.open_action`，开口 ``u ∈ [0, 1000]``）。
    **不做视觉伺服、不做力控夹取**（感知属 S06；力曲线未标定，
    见 :func:`rm75_drake.gripper.force_from_position`）。
``bimanual``
    **未实现**：双臂紧协调属 S05。本子命令明确报错并返回**非 0** 退出码，
    绝不静默成功、也绝不假装规划。
``replay``
    离线回放：对录制文件做碰撞复检（与 S02-T05 同一 ``edge_step_size``），
    打印逃逸数；**逃逸 > 0 → 退出码 1**。
``exec``
    真机执行入口：**Planner（或录制轨迹）→ rm_device 驱动 → 5 ms 执行环**，
    并产出**全链路运行日志**（独立 schema，见
    :mod:`rm75_drake.device.run_log`）。
    ``--dry-run`` 走**传输层替身**（零硬件）；真机不可达时输出
    ``SKIP(reason=hardware_unreachable)`` 并以 **0** 退出
    （既不伪装成失败，也不伪装成成功）。

退出码语义
----------
====  ==========================================================
码    含义
====  ==========================================================
0     成功
1     业务失败（规划失败 / 回放发现逃逸 / 安全态触发 / 执行违规）
2     用法错误（参数非法）或**命令未实现**（bimanual）
====  ==========================================================

headless 约定
-------------
* **默认不开可视化**（``--no-viz``）；``--viz`` 只在显式请求时启用；
* 可视化后端不可用时**如实打印原因**并继续（规划结果仍然有效），
  **绝不假装已渲染三维视图**（本机没有 meshcat 服务端，见 :mod:`rm75_drake.viz`）；
* matplotlib 兜底使用 ``Agg`` 后端 → **不弹窗、不连显示器**。
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

from . import paths
from .device.hardware_gate import (
    SKIP_REASON_HARDWARE_UNREACHABLE,
    UNVERIFIED_HARDWARE_METRICS as _UNVERIFIED_HARDWARE_METRICS,
    probe_hardware,
)
from .gripper import open_action
from .planning.gcs import DEFAULT_EDGE_STEP_SIZE

__all__ = [
    "EXIT_OK",
    "EXIT_FAILURE",
    "EXIT_USAGE",
    "REASON_HARDWARE_UNREACHABLE",
    "UNVERIFIED_HARDWARE_METRICS",
    "build_parser",
    "main",
]


#: 退出码：成功。
EXIT_OK = 0
#: 退出码：业务失败（规划失败 / 回放逃逸）。
EXIT_FAILURE = 1
#: 退出码：用法错误或命令未实现。
EXIT_USAGE = 2

#: 默认场景真相源。
DEFAULT_SCENARIO = str(paths.scenarios_dir / "single_arm.yaml")

#: 默认**设备（部署）配置**真相源（S03；与场景真相源分开）。
DEFAULT_DEVICE_CONFIG = str(paths.devices_dir / "single_arm.yaml")

#: ``exec`` 在「真机不可达」时的**稳定原因标签**（脚本与 CI 按它判定 SKIP）。
#: 真相源 = :mod:`rm75_drake.device.hardware_gate`（CLI 与三个测量脚本共用同一份）。
REASON_HARDWARE_UNREACHABLE = SKIP_REASON_HARDWARE_UNREACHABLE

#: 真机不可达时**必须如实列出**的未验证指标（绝不假装通过、绝不省略）。
#: 真相源同上——禁止在 CLI 里另立第二份清单。
UNVERIFIED_HARDWARE_METRICS: Tuple[str, ...] = tuple(_UNVERIFIED_HARDWARE_METRICS)

#: bimanual 的归属说明（**必须**出现在输出里，便于人工核对路线图）。
BIMANUAL_MESSAGE = (
    "bimanual 未实现：双臂紧协调（双臂 IK / 双臂 IRIS / 双臂 GCS / 协同抓取）"
    "归属 **S05**（多机器人协同切片），本切片 S02 只交付单臂链路。"
    "本命令**不做任何规划**，并以非 0 退出码退出（不静默成功）。"
)


# ── 参数解析 ────────────────────────────────────────────────────────────
def _add_planning_options(parser: argparse.ArgumentParser) -> None:
    """给 ``single`` / ``pick`` 挂上规划相关的公共选项。"""
    parser.add_argument(
        "--scenario",
        default=DEFAULT_SCENARIO,
        help=f"场景真相源 YAML（默认：{DEFAULT_SCENARIO}）",
    )
    parser.add_argument(
        "--target-xyz",
        type=float,
        nargs=3,
        metavar=("X", "Y", "Z"),
        default=None,
        help="任务空间目标位置（m，世界系）",
    )
    parser.add_argument(
        "--target-rpy",
        type=float,
        nargs=3,
        metavar=("ROLL", "PITCH", "YAW"),
        default=(0.0, 0.0, 0.0),
        help="任务空间目标姿态（**rad**，默认 0 0 0）",
    )
    parser.add_argument(
        "--target-joints",
        type=float,
        nargs=7,
        metavar=("Q1", "Q2", "Q3", "Q4", "Q5", "Q6", "Q7"),
        default=None,
        help="关节空间目标（rad，7 项）；与 --target-xyz 互斥",
    )
    parser.add_argument(
        "--start-joints",
        type=float,
        nargs=7,
        metavar=("Q1", "Q2", "Q3", "Q4", "Q5", "Q6", "Q7"),
        default=None,
        help="起点位形（rad，7 项）；默认用场景当前位形",
    )
    parser.add_argument("--output", default=None, help="录制文件输出路径（JSON）")
    parser.add_argument(
        "--region-spacing",
        type=float,
        default=0.15,
        help="IRIS 区域链种子间距（rad，默认 0.15；越大 → 区域越少、越快）",
    )
    parser.add_argument(
        "--iris-samples",
        type=int,
        default=None,
        help="IRIS 碰撞不可行采样数（调小 → 更快、认证更弱）",
    )
    parser.add_argument(
        "--no-viz",
        dest="viz",
        action="store_false",
        default=False,
        help="禁用可视化（**默认**，headless 安全）",
    )
    parser.add_argument(
        "--viz",
        dest="viz",
        action="store_true",
        default=False,
        help="启用可视化（后端不可用时如实打印原因，不影响规划结果）",
    )
    parser.add_argument(
        "--viz-backend",
        choices=("auto", "meshcat", "fallback"),
        default="auto",
        help="可视化后端（默认 auto：优先 meshcat，不可用退回 2D 兜底）",
    )
    parser.add_argument(
        "--viz-output", default=None, help="把可视化结果存成图片（PNG）的路径"
    )
    parser.add_argument("--json", action="store_true", help="以 JSON 输出结果摘要")


def build_parser() -> argparse.ArgumentParser:
    """构造参数解析器（``--help`` 一定能跑，退出码 0）。"""
    parser = argparse.ArgumentParser(
        prog="python3 -m rm75_drake.cli",
        description=(
            "RM75-D 规划 CLI 族：single / pick / bimanual / replay / exec。"
            "退出码：0 成功，1 业务失败（规划失败 / 回放逃逸 / 安全态触发），"
            "2 用法错误或命令未实现。"
        ),
    )
    subparsers = parser.add_subparsers(dest="command", metavar="<subcommand>")

    single = subparsers.add_parser(
        "single",
        help="单臂规划（IK → IRIS → GCS → 重定时 → 碰撞复检）",
        description="单臂规划：走完整链路并打印诊断。",
    )
    _add_planning_options(single)

    pick = subparsers.add_parser(
        "pick",
        help="预抓位姿规划 + 开爪动作调度",
        description=(
            "预抓位姿规划：规划到预抓位姿并给出开爪动作调度。"
            "**不做视觉伺服、不做力控夹取**（感知属 S06、力曲线未标定）。"
        ),
    )
    _add_planning_options(pick)
    pick.add_argument(
        "--pre-grasp-xyz",
        type=float,
        nargs=3,
        metavar=("X", "Y", "Z"),
        default=None,
        help="预抓位置（m，世界系）",
    )
    pick.add_argument(
        "--pre-grasp-rpy",
        type=float,
        nargs=3,
        metavar=("ROLL", "PITCH", "YAW"),
        default=(0.0, 0.0, 0.0),
        help="预抓姿态（**rad**，默认 0 0 0）",
    )
    pick.add_argument(
        "--gripper-speed",
        type=int,
        default=100,
        help="开爪速度档位（0–100，默认 100）",
    )
    pick.add_argument(
        "--gripper-hold",
        type=float,
        default=0.5,
        help="开爪后的保持时间（s，默认 0.5；**占位时序**，真机须用夹爪反馈替换）",
    )

    bimanual = subparsers.add_parser(
        "bimanual",
        help="双臂规划（**未实现**，属 S05；返回非 0 退出码）",
        description=BIMANUAL_MESSAGE,
    )
    bimanual.add_argument("--scenario", default=DEFAULT_SCENARIO, help=argparse.SUPPRESS)

    replay = subparsers.add_parser(
        "replay",
        help="离线回放：碰撞复检（逃逸 > 0 → 退出码 1）",
        description=(
            "离线回放：对录制文件做碰撞复检。"
            "复检口径与 S02-T05 一致（同一 edge_step_size）。"
        ),
    )
    replay.add_argument("--recording", required=True, help="录制文件路径（JSON）")
    replay.add_argument(
        "--scenario",
        default=None,
        help="场景真相源 YAML（默认取录制文件 meta.scenario_path）",
    )
    replay.add_argument(
        "--edge-step-size",
        type=float,
        default=DEFAULT_EDGE_STEP_SIZE,
        help=f"复检密采样步长（rad，默认 {DEFAULT_EDGE_STEP_SIZE}）",
    )
    replay.add_argument("--json", action="store_true", help="以 JSON 输出结果摘要")

    exec_parser = subparsers.add_parser(
        "exec",
        help="真机执行（Planner → rm_device → 5 ms 执行环 + 全链路运行日志）",
        description=(
            "真机执行入口：走 Planner（或 --recording 里的轨迹）→ rm_device 驱动 "
            "→ 5 ms 执行环，并产出全链路运行日志（独立 schema）。\n"
            "* --dry-run：走传输层替身（零硬件），用于离线验证执行链路；\n"
            "* 真机不可达（SDK 缺失 / TCP 不通）且未指定 --dry-run：输出 "
            "SKIP(reason=hardware_unreachable) 并以 0 退出，同时逐条列出"
            "**未验证的真机指标**（既不伪装成功，也不伪装失败）。"
        ),
    )
    exec_parser.add_argument(
        "--device-config",
        default=DEFAULT_DEVICE_CONFIG,
        help=f"设备（部署）配置真相源 YAML（默认：{DEFAULT_DEVICE_CONFIG}）",
    )
    exec_parser.add_argument(
        "--scenario",
        default=DEFAULT_SCENARIO,
        help=f"场景真相源 YAML（仅规划路径使用；默认：{DEFAULT_SCENARIO}）",
    )
    exec_parser.add_argument(
        "--target-joints",
        type=float,
        nargs=7,
        metavar=("Q1", "Q2", "Q3", "Q4", "Q5", "Q6", "Q7"),
        default=None,
        help="关节空间目标（rad，7 项）；与 --target-xyz / --recording 互斥",
    )
    exec_parser.add_argument(
        "--target-xyz",
        type=float,
        nargs=3,
        metavar=("X", "Y", "Z"),
        default=None,
        help="任务空间目标位置（m，世界系）；与 --target-joints / --recording 互斥",
    )
    exec_parser.add_argument(
        "--target-rpy",
        type=float,
        nargs=3,
        metavar=("ROLL", "PITCH", "YAW"),
        default=(0.0, 0.0, 0.0),
        help="任务空间目标姿态（**rad**，默认 0 0 0）",
    )
    exec_parser.add_argument(
        "--start-joints",
        type=float,
        nargs=7,
        metavar=("Q1", "Q2", "Q3", "Q4", "Q5", "Q6", "Q7"),
        default=None,
        help="起点位形（rad，7 项）；默认用场景当前位形",
    )
    exec_parser.add_argument(
        "--recording",
        default=None,
        help="录制文件路径（JSON）：直接回放其中的关节序列，**不再规划**",
    )
    exec_parser.add_argument(
        "--dry-run",
        action="store_true",
        help="离线执行：使用传输层替身（ScriptedTransport），**不连真机**",
    )
    exec_parser.add_argument(
        "--log-out",
        default=None,
        help="全链路运行日志的输出路径（JSON；不给则不落盘）",
    )
    exec_parser.add_argument(
        "--log-max-events",
        type=int,
        default=100000,
        help=(
            "运行日志的**事件数上限**（默认 100000；超出后停止追加并计数，"
            "丢弃数写进 summary.dropped_events，不静默丢弃）"
        ),
    )
    exec_parser.add_argument(
        "--connect-timeout",
        type=float,
        default=1.0,
        help="真机 TCP 指令通道的探测超时（s，默认 1.0）",
    )
    exec_parser.add_argument(
        "--region-spacing",
        type=float,
        default=0.15,
        help="IRIS 区域链种子间距（rad，默认 0.15；仅规划路径使用）",
    )
    exec_parser.add_argument(
        "--iris-samples",
        type=int,
        default=None,
        help="IRIS 碰撞不可行采样数（仅规划路径使用）",
    )
    exec_parser.add_argument("--json", action="store_true", help="以 JSON 输出结果摘要")

    return parser


# ── 主入口 ──────────────────────────────────────────────────────────────
def main(argv: Optional[Sequence[str]] = None) -> int:
    """CLI 主入口（返回退出码；**不**直接 ``sys.exit``）。"""
    parser = build_parser()
    args = parser.parse_args(list(argv) if argv is not None else None)
    command = getattr(args, "command", None)
    if not command:
        parser.print_help()
        return EXIT_USAGE
    try:
        if command == "single":
            return _cmd_single(args)
        if command == "pick":
            return _cmd_pick(args)
        if command == "bimanual":
            return _cmd_bimanual(args)
        if command == "replay":
            return _cmd_replay(args)
        if command == "exec":
            return _cmd_exec(args)
    except _UsageError as exc:
        print(f"cli: 参数错误：{exc}", file=sys.stderr)
        return EXIT_USAGE
    except Exception as exc:  # 其它异常 → 业务失败（消息带类型，便于排查）
        print(f"cli: 执行失败（{type(exc).__name__}）：{exc}", file=sys.stderr)
        return EXIT_FAILURE
    parser.print_help()
    return EXIT_USAGE


class _UsageError(RuntimeError):
    """CLI 参数用法错误（退出码 2）。"""


# ── single ──────────────────────────────────────────────────────────────
def _cmd_single(args: argparse.Namespace) -> int:
    """``single``：单臂规划 + 诊断 +（可选）录制。"""
    plan = _plan_once(args, label="single")
    if not plan["success"]:
        _emit_failure(args, plan)
        return EXIT_FAILURE
    _emit_success(args, plan)
    return EXIT_OK


# ── pick ────────────────────────────────────────────────────────────────
def _cmd_pick(args: argparse.Namespace) -> int:
    """``pick``：规划到预抓位姿 + 开爪动作调度。"""
    if args.pre_grasp_xyz is None:
        raise _UsageError("pick 需要 --pre-grasp-xyz（预抓位置，m）")
    action = open_action(speed=int(args.gripper_speed), hold_seconds=float(args.gripper_hold))

    pre_args = _clone_args(args)
    pre_args.target_xyz = list(args.pre_grasp_xyz)
    pre_args.target_rpy = list(args.pre_grasp_rpy)
    pre_args.target_joints = None
    plan = _plan_once(
        pre_args, label="pick:pre-grasp", gripper_schedule=(action,), write_output=False
    )
    if not plan["success"]:
        _emit_failure(args, plan)
        return EXIT_FAILURE

    trajectory = plan["trajectory"]
    schedule = plan["gripper_schedule"]

    # 可选第二段：从预抓位姿走到最终目标（--target-xyz 给了才做）。
    if args.target_xyz is not None or args.target_joints is not None:
        second_args = _clone_args(args)
        second_args.start_joints = list(trajectory.samples[-1].q)
        plan2 = _plan_once(
            second_args, label="pick:approach", gripper_schedule=(), write_output=False
        )
        if not plan2["success"]:
            _emit_failure(args, plan2)
            return EXIT_FAILURE
        trajectory = _merge_legs(trajectory, plan2["trajectory"])
        schedule = tuple(schedule) + tuple(plan2["gripper_schedule"])

    plan["trajectory"] = trajectory
    plan["gripper_schedule"] = schedule
    plan["recording_path"] = _write_recording(args, trajectory, schedule)
    plan["notes"] = [
        "预抓位姿规划完成；开爪动作调度已给出（开口 u=1000，标准夹指 65 mm）",
        "**未做视觉伺服、未做力控夹取**：感知属 S06；夹爪力曲线未标定"
        "（force 档位不是牛顿值）",
    ]
    _emit_success(args, plan)
    return EXIT_OK


# ── bimanual ────────────────────────────────────────────────────────────
def _cmd_bimanual(args: argparse.Namespace) -> int:
    """``bimanual``：**未实现**（属 S05），返回非 0 退出码。"""
    print(BIMANUAL_MESSAGE)
    print("[bimanual] 本命令不做任何规划，也不产生任何轨迹。")
    return EXIT_USAGE


# ── replay ──────────────────────────────────────────────────────────────
def _cmd_replay(args: argparse.Namespace) -> int:
    """``replay``：离线回放碰撞复检（逃逸 > 0 → 退出码 1）。"""
    from .planning.scene import build_scene
    from .recording import load_recording, replay_check
    from .scenario import load_scenario

    recording_path = Path(args.recording)
    if not recording_path.is_file():
        raise _UsageError(f"录制文件不存在：{recording_path}")
    recording = load_recording(recording_path)

    scenario_path = args.scenario or recording.meta.get("scenario_path") or ""
    if not scenario_path:
        raise _UsageError(
            "录制文件没有 meta.scenario_path，请用 --scenario 指定场景真相源"
        )
    scenario_file = Path(scenario_path)
    if not scenario_file.is_absolute():
        scenario_file = (Path.cwd() / scenario_file).resolve()
    if not scenario_file.is_file():
        raise _UsageError(f"场景真相源不存在：{scenario_file}")
    scenario = load_scenario(scenario_file)
    scene = build_scene(scenario)
    result = replay_check(recording, scene, edge_step_size=float(args.edge_step_size))

    payload = {
        "command": "replay",
        "recording": str(recording_path),
        "robot_name": recording.robot_name,
        "num_recorded_samples": len(recording.samples),
        "num_samples": result.num_samples,
        "num_escapes": result.num_escapes,
        "max_joint_step": result.max_joint_step,
        "edge_step_size": float(args.edge_step_size),
        "success": result.ok,
    }
    if args.json:
        print(json.dumps(payload, ensure_ascii=False, indent=2))
    else:
        print(f"[replay] 录制文件：{recording_path}")
        print(f"[replay] 录制点数：{len(recording.samples)}（机器人 {recording.robot_name}）")
        print(f"[replay] 复检位形数：{result.num_samples}")
        print(f"[replay] 逃逸数：{result.num_escapes}")
        print(f"[replay] 最大单步关节变化：{result.max_joint_step:.6f} rad")
    if not result.ok:
        print(
            f"[replay] 失败：发现 {result.num_escapes} 处碰撞逃逸"
            f"（首个下标 {result.escape_indices[0] if result.escape_indices else '-'}）",
            file=sys.stderr,
        )
        return EXIT_FAILURE
    return EXIT_OK


# ── exec（真机执行入口）────────────────────────────────────────────────
def _cmd_exec(args: argparse.Namespace) -> int:
    """``exec``：Planner / 录制轨迹 → rm_device 驱动 → 5 ms 执行环。

    退出码
    ------
    * ``0``：执行成功（``--dry-run`` 离线执行也算成功）**或**真机不可达时的
      ``SKIP(reason=hardware_unreachable)``；
    * ``1``：业务失败（规划失败 / 安全态触发 / 传输层失败）；
    * ``2``：用法错误（参数非法、目标缺失、配置文件缺失等）。
    """
    from .device.config import DeviceConfigError, load_device_config

    sources = [
        ("--target-joints", getattr(args, "target_joints", None) is not None),
        ("--target-xyz", getattr(args, "target_xyz", None) is not None),
        ("--recording", getattr(args, "recording", None) is not None),
    ]
    given = [name for name, present in sources if present]
    if len(given) != 1:
        raise _UsageError(
            "exec 需要且只需要一个目标来源"
            "（--target-joints / --target-xyz / --recording），当前给了："
            f"{given if given else '无'}"
        )

    config_path = Path(args.device_config)
    if not config_path.is_file():
        raise _UsageError(f"设备配置不存在：{config_path}")
    try:
        device_config = load_device_config(config_path)
    except DeviceConfigError as exc:
        raise _UsageError(f"设备配置非法：{exc}") from exc

    # ── 硬件门控：真机不可达 ⇒ SKIP（退出码 0），绝不伪装成功/失败 ──
    if not args.dry_run:
        probe = _probe_hardware(device_config, timeout=float(args.connect_timeout))
        if not probe["reachable"]:
            return _emit_exec_skipped(args, device_config, probe)

    # ── 轨迹来源：Planner 或录制文件 ──
    if args.recording is not None:
        trajectory = _exec_trajectory_from_recording(args.recording)
        trajectory_source = "recording"
    else:
        plan = _plan_once(args, label="exec", write_output=False)
        if not plan["success"]:
            _emit_failure(args, plan)
            return EXIT_FAILURE
        trajectory = plan["trajectory"]
        trajectory_source = "planner"

    result = _exec_run_driver(args, device_config, trajectory)
    result["trajectory_source"] = trajectory_source
    return _emit_exec_result(args, device_config, result)


def _exec_trajectory_from_recording(path: str) -> Any:
    """从录制文件构造轨迹（**不再规划**）。

    失败模式
    --------
    * 文件不存在 / 录制结构非法 → :class:`_UsageError`（退出码 2）；
    * 录制内容不满足轨迹不变量（软限位 / 速度 / 加速度）→ :class:`_UsageError`。
    """
    from .planning.trajectory import TrajectoryError, trajectory_from_arrays
    from .recording import RecordingError, load_recording

    recording_path = Path(path)
    if not recording_path.is_file():
        raise _UsageError(f"录制文件不存在：{recording_path}")
    try:
        recording = load_recording(recording_path)
    except RecordingError as exc:
        raise _UsageError(f"录制文件非法：{exc}") from exc
    try:
        return trajectory_from_arrays(
            recording.q_matrix(),
            t0=recording.samples[0].t_abs,
            dt=recording.dt,
            robot_name=recording.robot_name,
            gripper_schedule=recording.gripper_schedule,
        )
    except TrajectoryError as exc:
        raise _UsageError(f"录制内容不满足轨迹不变量：{exc}") from exc


def _probe_hardware(device_config: Any, *, timeout: float) -> Dict[str, Any]:
    """探测真机是否可达（**不抛异常**，返回可解释的结果字典）。

    真相源 = :func:`rm75_drake.device.hardware_gate.probe_hardware`
    （CLI 与 ``scripts/`` 下的三个测量脚本共用同一份门控实现）。

    返回
    ----
    dict：``reachable``（bool）/ ``reason``（稳定标签）/ ``detail``（人类可读原因）。
    """
    return probe_hardware(device_config, timeout=timeout).to_dict()


def _exec_run_driver(
    args: argparse.Namespace, device_config: Any, trajectory: Any
) -> Dict[str, Any]:
    """在 5 ms 执行环里跑完轨迹，返回统一结果字典。

    * ``--dry-run`` ⇒ :class:`ScriptedTransport` + 虚拟时钟（零硬件、确定性）；
    * 否则 ⇒ :class:`SdkTransport` + 系统时钟（真机）。
    """
    from .device.exec_loop import SystemClock, VirtualClock
    from .device.run_log import RunLogBuilder, metric_payload
    from .device.safety import safety_limits_from_sources
    from .device.transport import ScriptedTransport, SdkTransport
    from .drivers.base import DriverError
    from .drivers.rm_device import RmDeviceDriver, RmDeviceDriverConfig
    from .limits import load_joint_limits

    limits = safety_limits_from_sources(load_joint_limits(), device_config)
    dry_run = bool(args.dry_run)
    transport = ScriptedTransport() if dry_run else SdkTransport(device_config)
    clock = VirtualClock() if dry_run else SystemClock()
    driver = RmDeviceDriver(
        transport,
        limits=limits,
        config=RmDeviceDriverConfig(require_state=not dry_run),
        clock=clock,
        dt=float(trajectory.dt),
    )

    builder = RunLogBuilder(
        start_ns=time.monotonic_ns(),
        max_events=int(getattr(args, "log_max_events", 100000) or 100000),
    )
    status = "ok"
    reason = ""
    message = ""
    num_commands = 0
    num_samples = 0
    final_phase = "idle"
    driver.open()
    try:
        driver.submit_trajectory(trajectory)
        builder.append(
            "state_transition",
            {
                "from_state": "idle",
                "to_state": driver.phase,
                "event": "submit_trajectory",
                "reason": "",
            },
        )
        while driver.index < driver.num_samples:
            try:
                state = driver.step()
            except DriverError as exc:
                status = "failed"
                reason = str(exc.reason)
                message = str(exc)
                break
            num_commands += 1
            builder.append(
                "command",
                {
                    "seq": int(state.seq),
                    "t_abs": float(state.t_abs),
                    "joint_rad": [float(v) for v in state.joint_positions],
                },
            )
            frame = driver.read_device_frame()
            if frame is not None:
                builder.append(
                    "state_frame",
                    {
                        "seq": int(state.seq),
                        "t_abs": float(state.t_abs),
                        "age_s": float(max(0.0, time.monotonic() - float(frame.received_at))),
                        "joint_rad": [float(v) for v in frame.joint_positions_rad],
                        "currents_ma": [float(v) for v in frame.joint_currents_ma],
                        "enabled": [bool(v) for v in frame.joint_en_flags],
                        "joint_err_codes": [int(v) for v in frame.joint_err_codes],
                        "controller_status": int(frame.controller_status),
                        "gripper_online": bool(frame.gripper_online),
                        "complete": bool(frame.is_complete),
                    },
                )
        for violation in driver.violations:
            builder.append(
                "safety_violation",
                {
                    "reason": str(violation.reason),
                    "gate": str(violation.gate),
                    "action": str(violation.action),
                    "severity": str(violation.severity),
                    "detail": str(violation.detail),
                    "joints": [int(v) for v in violation.joints],
                },
            )
        for error in driver.errors:
            builder.append(
                "note",
                {"text": f"驱动错误：{error.get('reason', '')} {error.get('message', '')}".strip()},
            )
        builder.append(
            "state_transition",
            {
                "from_state": "streaming",
                "to_state": driver.phase,
                "event": "finished",
                "reason": reason,
            },
        )
        builder.append("metric", metric_payload("jitter", driver.jitter))
        # 关闭前先取快照（close() 会清空轨迹引用并把相位复位为 idle）
        num_samples = int(driver.num_samples)
        final_phase = driver.phase
    finally:
        driver.close()

    jitter = driver.jitter.summary()
    return {
        "status": status,
        "reason": reason,
        "message": message,
        "num_commands": num_commands,
        "num_samples": num_samples,
        "violations": [
            {
                "reason": str(v.reason),
                "gate": str(v.gate),
                "action": str(v.action),
                "severity": str(v.severity),
                "detail": str(v.detail),
            }
            for v in driver.violations
        ],
        "jitter": jitter,
        "run_log": builder,
        "phase": final_phase,
    }


def _emit_exec_skipped(
    args: argparse.Namespace, device_config: Any, probe: Dict[str, Any]
) -> int:
    """真机不可达：输出 ``SKIP(reason=hardware_unreachable)`` 并以 **0** 退出。

    同时**逐条列出未验证的真机指标**（绝不假装通过、绝不省略）。
    """
    from .device.run_log import RunLogBuilder, build_run_log

    builder = RunLogBuilder(start_ns=time.monotonic_ns())
    builder.append(
        "note",
        {"text": f"SKIP(reason={probe['reason']})：{probe['detail']}"},
    )
    run_log = build_run_log(
        builder.events,
        meta={
            "command": "exec",
            "mode": "hardware",
            "device_name": device_config.name,
            "arm_ip": device_config.arm_ip,
            "scenario_path": str(args.scenario),
            "recording_path": str(args.recording) if args.recording else "",
            "robot_name": "rm75",
            "dry_run": False,
        },
        summary={
            "status": "skipped",
            "reason": probe["reason"],
            "num_samples": 0,
            "num_violations": 0,
            "jitter": None,
            "notes": [probe["detail"]],
            "unverified_metrics": list(UNVERIFIED_HARDWARE_METRICS),
        },
    )
    log_path = _write_exec_run_log(args, run_log)

    payload = {
        "command": "exec",
        "status": "skipped",
        "success": False,
        "skipped": True,
        "reason": probe["reason"],
        "detail": probe["detail"],
        "unverified_metrics": list(UNVERIFIED_HARDWARE_METRICS),
        "run_log_path": log_path,
    }
    if getattr(args, "json", False):
        print(json.dumps(payload, ensure_ascii=False, indent=2, default=str))
    else:
        print(f"[exec] SKIP(reason={probe['reason']})：{probe['detail']}")
        print("[exec] 以下真机指标**未验证**（不得当作通过）：")
        for item in UNVERIFIED_HARDWARE_METRICS:
            print(f"[exec]   - {item}")
        if log_path:
            print(f"[exec] 运行日志：{log_path}")
    return EXIT_OK


def _write_exec_run_log(args: argparse.Namespace, run_log: Any) -> Optional[str]:
    """按 ``--log-out`` 写运行日志（不给则不落盘；返回路径或 ``None``）。"""
    output = getattr(args, "log_out", None)
    if not output:
        return None
    from .device.run_log import write_run_log

    write_run_log(run_log, output)
    return str(Path(output))


def _emit_exec_result(
    args: argparse.Namespace, device_config: Any, result: Dict[str, Any]
) -> int:
    """打印 exec 结果摘要（JSON 或人类可读），返回退出码。"""
    builder = result["run_log"]
    run_log = builder.build(
        meta={
            "command": "exec",
            "mode": "dry_run" if args.dry_run else "hardware",
            "device_name": device_config.name,
            "arm_ip": device_config.arm_ip,
            "scenario_path": str(args.scenario),
            "recording_path": str(args.recording) if args.recording else "",
            "robot_name": "rm75",
            "dry_run": bool(args.dry_run),
        },
        summary={
            "status": result["status"],
            "reason": result["reason"],
            "num_samples": result["num_samples"],
            "num_violations": len(result["violations"]),
            "jitter": result["jitter"],
            "notes": [result["message"]] if result["message"] else [],
            "unverified_metrics": (
                [] if args.dry_run else list(UNVERIFIED_HARDWARE_METRICS)
            ),
        },
    )
    log_path = _write_exec_run_log(args, run_log)

    payload = {
        "command": "exec",
        "status": result["status"],
        "success": result["status"] == "ok",
        "skipped": False,
        "reason": result["reason"],
        "message": result["message"],
        "mode": "dry_run" if args.dry_run else "hardware",
        "trajectory_source": result.get("trajectory_source"),
        "num_samples": result["num_samples"],
        "num_commands": result["num_commands"],
        "phase": result["phase"],
        "num_violations": len(result["violations"]),
        "violations": result["violations"],
        "jitter": result["jitter"],
        "run_log_path": log_path,
        "run_log_events": len(run_log.events),
        "dropped_events": int(run_log.summary.get("dropped_events") or 0),
    }
    if getattr(args, "json", False):
        print(json.dumps(payload, ensure_ascii=False, indent=2, default=str))
    else:
        print(f"[exec] 状态：{result['status']}（模式：{payload['mode']}）")
        print(
            f"[exec] 下发点数：{result['num_commands']} / {result['num_samples']}"
            f"（相位：{result['phase']}）"
        )
        print(f"[exec] 安全违规数：{len(result['violations'])}")
        dropped = int(run_log.summary.get("dropped_events") or 0)
        if dropped:
            print(
                f"[exec] 运行日志事件数已达上限：丢弃 {dropped} 条"
                "（summary.dropped_events；不静默丢弃）",
                file=sys.stderr,
            )
        jitter = result["jitter"] or {}
        print(
            f"[exec] 抖动：count={jitter.get('count')} p50={jitter.get('p50')} "
            f"p99={jitter.get('p99')} p99.9={jitter.get('p999')} max={jitter.get('max')}"
        )
        if result["message"]:
            print(f"[exec] 失败原因（{result['reason']}）：{result['message']}", file=sys.stderr)
        if log_path:
            print(f"[exec] 运行日志：{log_path}")
    if result["status"] != "ok":
        return EXIT_FAILURE
    return EXIT_OK


# ── 规划实现 ────────────────────────────────────────────────────────────
def _clone_args(args: argparse.Namespace) -> argparse.Namespace:
    """复制参数命名空间（改目标时不污染原对象）。"""
    import copy

    return copy.deepcopy(args)


def _plan_once(
    args: argparse.Namespace,
    *,
    label: str,
    gripper_schedule: Sequence[Any] = (),
    write_output: bool = True,
) -> Dict[str, Any]:
    """跑一次完整规划链路，返回统一结构的字典。

    返回
    ----
    dict：``success`` / ``message`` / ``trajectory`` / ``gripper_schedule`` /
    ``diagnostics`` / ``recording_path``。
    """
    from .planner import Planner, PlanOptions
    from .planning.scene import build_scene
    from .scenario import load_scenario

    scenario_file = Path(args.scenario)
    if not scenario_file.is_file():
        raise _UsageError(f"场景真相源不存在：{scenario_file}")
    scenario = load_scenario(scenario_file)
    scene = build_scene(scenario)

    region_options: Dict[str, Any] = {}
    if args.iris_samples is not None:
        region_options["num_collision_infeasible_samples"] = int(args.iris_samples)

    options = PlanOptions(
        start_configuration=tuple(args.start_joints) if args.start_joints else None,
        region_spacing=float(args.region_spacing),
        gripper_schedule=tuple(gripper_schedule),
        region_options=region_options,
    )
    planner = Planner(scenario, scene=scene, options=options)

    if args.target_joints is not None and args.target_xyz is not None:
        raise _UsageError("--target-joints 与 --target-xyz 互斥（只能给一个）")
    if args.target_joints is not None:
        result = planner.plan_joint_target(args.target_joints)
    elif args.target_xyz is not None:
        result = planner.plan_task_space_target(
            _pose_from_xyz_rpy(args.target_xyz, args.target_rpy)
        )
    else:
        raise _UsageError("必须给 --target-xyz 或 --target-joints 之一")

    payload: Dict[str, Any] = {
        "command": label,
        "success": bool(result.success),
        "message": result.message,
        "stage": result.stage,
        "reason": result.reason,
        "diagnostics": dict(result.diagnostics),
        "trajectory": result.trajectory,
        "gripper_schedule": tuple(result.gripper_schedule),
        "recording_path": None,
        "scene": scene,
        "scenario": scenario,
        "notes": [],
    }
    if not result.success or result.trajectory is None:
        return payload

    if write_output and getattr(args, "output", None):
        payload["recording_path"] = _write_recording(
            args, result.trajectory, tuple(result.gripper_schedule), scenario=scenario
        )
    payload["gripper_schedule"] = tuple(result.gripper_schedule)
    return payload


def _write_recording(
    args: argparse.Namespace,
    trajectory: Any,
    gripper_schedule: Sequence[Any],
    *,
    scenario: Any = None,
) -> Optional[str]:
    """把轨迹录制成 JSON（返回路径；没有 ``--output`` → ``None``）。"""
    output = getattr(args, "output", None)
    if not output:
        return None
    from .recording import record_trajectory

    scenario_path = None
    if scenario is not None:
        source = getattr(scenario, "source_path", None)
        scenario_path = str(Path(source).resolve()) if source else str(Path(args.scenario).resolve())
    record_trajectory(
        trajectory,
        output,
        scenario_path=scenario_path,
    )
    return str(Path(output))


def _pose_from_xyz_rpy(xyz: Sequence[float], rpy: Sequence[float]) -> Any:
    """把 (xyz, rpy) 变成世界系的 :class:`pydrake.math.RigidTransform`。"""
    from pydrake.math import RigidTransform

    from . import drake_compat

    rotation = drake_compat.rotation_from_rpy(tuple(float(v) for v in rpy))
    return RigidTransform(rotation, np.asarray([float(v) for v in xyz], dtype=float))


def _merge_legs(first: Any, second: Any) -> Any:
    """把两段轨迹接成一条（去掉重复的衔接点，保持 5 ms 网格）。"""
    from .planning.trajectory import trajectory_from_arrays

    q_first = first.q_matrix()
    q_second = second.q_matrix()
    rows = np.vstack([q_first, q_second[1:]]) if q_second.shape[0] > 1 else q_first
    return trajectory_from_arrays(
        rows,
        t0=first.samples[0].t_abs,
        dt=first.dt,
        robot_name=first.robot_name,
        gripper_schedule=tuple(first.gripper_schedule) + tuple(second.gripper_schedule),
    )


# ── 输出 ────────────────────────────────────────────────────────────────
def _emit_success(args: argparse.Namespace, plan: Dict[str, Any]) -> None:
    """打印成功结果（JSON 或人类可读）。"""
    trajectory = plan["trajectory"]
    diagnostics = plan.get("diagnostics", {})
    schedule = tuple(plan.get("gripper_schedule", ()))
    notes: List[str] = list(plan.get("notes", ()))

    viz_info = None
    if getattr(args, "viz", False):
        viz_info = _maybe_visualize(args, plan)

    payload = {
        "command": plan["command"],
        "success": True,
        "message": plan["message"],
        "num_samples": len(trajectory.samples) if trajectory is not None else 0,
        "dt": float(trajectory.dt) if trajectory is not None else None,
        "duration_s": float(trajectory.duration) if trajectory is not None else None,
        "stage_durations_s": {
            stage: diagnostics.get(stage, {}).get("duration_s")
            for stage in ("ik", "regions", "gcs", "retime", "collision")
            if isinstance(diagnostics.get(stage), dict)
        },
        "num_escapes": diagnostics.get("collision", {}).get("num_escapes"),
        "gripper_schedule": [
            {
                "kind": action.kind,
                "position": action.position,
                "speed": action.speed,
                "force": action.force,
                "hold_seconds": action.hold_seconds,
            }
            for action in schedule
        ],
        "recording_path": plan.get("recording_path"),
        "notes": notes,
        "viz": viz_info,
    }
    if getattr(args, "json", False):
        print(json.dumps(payload, ensure_ascii=False, indent=2, default=str))
        return

    print(f"[{plan['command']}] {plan['message']}")
    for stage, duration in payload["stage_durations_s"].items():
        print(f"[{plan['command']}] 阶段耗时 {stage}: {duration} s")
    if payload["num_escapes"] is not None:
        print(f"[{plan['command']}] 碰撞复检逃逸数：{payload['num_escapes']}")
    for action in schedule:
        print(
            f"[{plan['command']}] 夹爪动作：kind={action.kind} position={action.position} "
            f"speed={action.speed} hold={action.hold_seconds}s"
        )
    for note in notes:
        print(f"[{plan['command']}] 说明：{note}")
    if plan.get("recording_path"):
        print(f"[{plan['command']}] 录制文件：{plan['recording_path']}")
    if viz_info:
        print(f"[{plan['command']}] 可视化：{viz_info}")


def _emit_failure(args: argparse.Namespace, plan: Dict[str, Any]) -> None:
    """打印失败结果（JSON 或人类可读）。"""
    if getattr(args, "json", False):
        print(
            json.dumps(
                {
                    "command": plan["command"],
                    "success": False,
                    "message": plan["message"],
                    "stage": plan["stage"],
                    "reason": plan["reason"],
                },
                ensure_ascii=False,
                indent=2,
            )
        )
        return
    print(f"[{plan['command']}] 规划失败：{plan['message']}", file=sys.stderr)


def _headless() -> bool:
    """判断当前是否处于 headless 环境（无 X11 显示）。

    Linux 下没有 ``DISPLAY`` 即为 headless；此时**绝不允许** meshcat 后端
    （它会拉起浏览器窗口）。
    """
    if sys.platform.startswith("linux"):
        return not os.environ.get("DISPLAY", "").strip()
    return False


def _maybe_visualize(args: argparse.Namespace, plan: Dict[str, Any]) -> Dict[str, Any]:
    """尝试可视化；**不可用时如实打印原因**，不伪装成功。

    headless 约定：headless 环境下**只**用 2D 兜底（matplotlib Agg），
    绝不尝试打开浏览器（meshcat 会拉起浏览器窗口）。
    """
    from . import viz as viz_mod

    backend = getattr(args, "viz_backend", "auto")
    headless = _headless()
    if headless and backend in ("auto", "meshcat"):
        backend = "fallback"
        print(
            "[viz] headless 环境（无 DISPLAY）：只用 2D 兜底后端，不打开浏览器",
            file=sys.stderr,
        )
    try:
        visualizer = viz_mod.make_visualizer(backend)
    except viz_mod.NotAvailableError as exc:
        message = f"可视化后端不可用：{exc}"
        print(f"[viz] {message}", file=sys.stderr)
        return {"available": False, "backend": None, "message": message}
    try:
        visualizer.show_trajectory(plan.get("scene"), plan["trajectory"])
        saved = None
        if getattr(args, "viz_output", None):
            saved = str(visualizer.save(args.viz_output))
        message = f"已用后端 {visualizer.name} 渲染" + (
            f"，图片：{saved}" if saved else ""
        )
        print(f"[viz] {message}")
        return {"available": True, "backend": visualizer.name, "message": message}
    except Exception as exc:
        message = f"可视化失败（{type(exc).__name__}）：{exc}"
        print(f"[viz] {message}", file=sys.stderr)
        return {"available": False, "backend": visualizer.name, "message": message}


if __name__ == "__main__":  # pragma: no cover - 进程入口
    sys.exit(main())
