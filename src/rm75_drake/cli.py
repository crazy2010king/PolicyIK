"""CLI 族（S02-T08）：``single`` / ``pick`` / ``bimanual`` / ``replay``。

统一入口
--------
::

    python3 -m rm75_drake.cli <subcommand> [options]

四个子命令的语义
----------------
（``single`` / ``pick`` / ``bimanual`` / ``replay``；``exec`` 与 ``calibrate``
 见下）
``single``
    单臂规划：走完整链路（IK → IRIS → 空间 GCS → Toppra 重定时 → 碰撞复检），
    打印诊断，可选把结果录制成 JSON（``--output``）。
``pick``
    预抓位姿规划：在预抓位姿规划并给出**开爪动作调度**
    （:func:`rm75_drake.gripper.open_action`，开口 ``u ∈ [0, 1000]``）。
    **不做视觉伺服、不做力控夹取**（感知属 S06；力曲线未标定，
    见 :func:`rm75_drake.gripper.force_from_position`）。
``bimanual``
    **双臂紧协调规划（S05 已实现）**：走完整链路（最小坐标参数化 →
    自研 8 维凸区域 → GCS → 14 维重定时 → 双轨迹 → 14 维零逃逸复检），
    打印诊断；``--json`` 输出**单行** JSON 摘要
    （status / reason / stage / diagnostics）。
    「相对位姿固定」由**构造性满足**（最小坐标映射），残差逐点 ≤1e-6。
    真机 / 物理指标**无真机 ⇒ 硬件门控 SKIP**：逐条列入
    ``unverified_metrics``，**绝不**用合成数据冒充物理达标。
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
``calibrate``
    双臂标定作业入口（**离线**，零硬件）：解算关节零位偏差 / 工具 TCP 6D /
    双臂基座外参 ``X_LR``，输出残差摘要（RMS / 最大残差 / 逐点残差 /
    重复性 / 交叉验证差异）。
    **无实测数据时输出 ``SKIP(reason=calibration_not_measured)`` 并以 0 退出**，
    同时逐条列出**未验证的物理指标**（绝不把合成数据的残差冒充实测精度）。

退出码语义
----------
====  ==========================================================
码    含义
====  ==========================================================
0     成功
1     业务失败（规划失败 / 回放发现逃逸 / 安全态触发 / 执行违规）
2     用法错误（参数非法 / 路径缺失 / 命令未实现）
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
import math
import os
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

from . import paths
from .calibration.pipeline import (
    REASON_CALIBRATION_NOT_MEASURED as _REASON_CALIBRATION_NOT_MEASURED,
    UNVERIFIED_CALIBRATION_METRICS as _UNVERIFIED_CALIBRATION_METRICS,
)
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
    "REASON_CALIBRATION_NOT_MEASURED",
    "UNVERIFIED_CALIBRATION_METRICS",
    "DEFAULT_CALIBRATION_SAMPLES",
    "DEFAULT_CALIBRATION_FILE",
    "DEFAULT_DUAL_SCENARIO",
    "BIMANUAL_MESSAGE",
    "BIMANUAL_DEFAULT_X_XYZ",
    "BIMANUAL_DEFAULT_X_RPY",
    "BIMANUAL_CANDIDATE_LEADER_POSTURES",
    "UNVERIFIED_COORDINATED_METRICS",
    "bimanual_x_lr_matrix",
    "select_bimanual_start",
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

#: ``calibrate`` 在「无实测数据」时的**稳定原因标签**（脚本与 CI 按它判定 SKIP）。
#: 真相源 = :mod:`rm75_drake.calibration.pipeline`（CLI 与脚本共用同一份）。
REASON_CALIBRATION_NOT_MEASURED = "calibration_not_measured"

#: 无实测数据时**必须如实列出**的未验证物理指标（真相源同上）。
UNVERIFIED_CALIBRATION_METRICS: Tuple[str, ...] = tuple(_UNVERIFIED_CALIBRATION_METRICS)

#: ``calibrate`` 的默认采样文件（仓库根 ``calibration/samples/demo.yaml``，合成演练数据）。
DEFAULT_CALIBRATION_SAMPLES = str(paths.calibration_files_dir / "samples" / "demo.yaml")

#: ``calibrate`` 的默认标定文件（仓库根 ``calibration/dual_arm.yaml``，未实测占位）。
DEFAULT_CALIBRATION_FILE = str(paths.calibration_files_dir / "dual_arm.yaml")

#: bimanual 的归属说明（**必须**出现在输出里，便于人工核对路线图）。
BIMANUAL_MESSAGE = (
    "bimanual 已实现：双臂紧协调规划（S05）—— 最小坐标参数化（相对位姿约束"
    "**构造性满足**）→ 自研 8 维凸区域 → GCS → 14 维重定时 → 共享时间轴双轨迹"
    "→ 14 维零逃逸复检。\n"
    "真机 / 物理指标**无真机 ⇒ 硬件门控 SKIP**（逐条列入 unverified_metrics，"
    "绝不用合成数据冒充物理达标）。"
)

#: 双臂场景真相源（``bimanual`` 的默认场景；S04 的双臂场景）。
DEFAULT_DUAL_SCENARIO = str(paths.scenarios_dir / "dual_arm.yaml")

#: 相对位姿 ``X_LR``（leader TCP 系 → follower TCP 系）的**默认占位值**：
#: 沿 leader TCP 的 x 轴偏 0.2 m（“两臂夹持同一物体”的最小几何占位）。
#: ⚠ 这是**占位**，不是标定结果：真实值由任务 / 标定给定，**必须显式传入**
#: （``--x-xyz`` / ``--x-rpy`` / ``--x-matrix``）才是真实约束。
BIMANUAL_DEFAULT_X_XYZ: Tuple[float, float, float] = (0.2, 0.0, 0.0)
BIMANUAL_DEFAULT_X_RPY: Tuple[float, float, float] = (0.0, 0.0, 0.0)

#: leader 起始位形的**候选表**（按顺序取第一个可行的）：
#: 每个候选都要满足「follower 有解析 IK 解 + 14 维限位内 + 14 维碰撞自由」，
#: 且 follower 的可行 ψ **窗口足够宽**（≥3 个连续网格点）时才会被选中。
BIMANUAL_CANDIDATE_LEADER_POSTURES: Tuple[Tuple[float, ...], ...] = (
    (-0.2, 1.0, -1.0, -1.0, 0.0, 0.2, 0.0),
    (0.2, 2.0, -2.0, 1.2, 0.0, -0.3, 0.0),
    (0.2, 1.0, -1.0, 1.2, 0.0, -0.3, 0.0),
    (-0.2, 1.0, 0.0, -1.0, 0.0, 0.6, 0.0),
    (0.2, 2.0, 0.0, -1.0, 0.0, 0.6, 0.0),
    (0.0, -0.3, 0.0, 1.2, 0.0, 0.2, 0.0),
)

#: 无真机时**必须如实列出**的未验证物理指标（真相源；CLI 与
#: ``scripts/coordinated_stats.py`` 共用同一份，**禁止另立第二份清单**）。
UNVERIFIED_COORDINATED_METRICS: Tuple[str, ...] = (
    "真实（真机）双臂紧协调规划成功率 ≥ 95%（本环境无真机，未验证）",
    "真实（真机）协调求解阶段 P95 < 3 s（未验证）",
    "真实（真机）共持位姿漂移 < 2 mm / 10 s（未验证）",
    "真实（真机）双臂相对位姿保持（构造性满足仅在仿真中验证）（未验证）",
    "真实（真机）14 维零逃逸（离线仿真零逃逸 ≠ 真机零逃逸）（未验证）",
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
            "RM75-D 规划 CLI 族：single / pick / bimanual / replay / exec / calibrate。"
            "退出码：0 成功（或无实测数据 / 真机不可达时的 SKIP），"
            "1 业务失败（规划失败 / 回放逃逸 / 安全态触发 / 残差超阈值），"
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
        help="双臂紧协调规划（S05 已实现：最小坐标 → 区域 → GCS → 重定时 → 复检）",
        description=BIMANUAL_MESSAGE,
    )
    bimanual.add_argument(
        "--scenario",
        default=DEFAULT_DUAL_SCENARIO,
        help=f"双臂场景真相源 YAML（默认：{DEFAULT_DUAL_SCENARIO}）",
    )
    bimanual.add_argument(
        "--x-xyz",
        type=float,
        nargs=3,
        metavar=("X", "Y", "Z"),
        default=None,
        help=(
            "相对位姿 X_LR 的平移（m，leader TCP 系 → follower TCP 系）；"
            f"默认 {BIMANUAL_DEFAULT_X_XYZ}（**占位值**：真实值请显式给出）"
        ),
    )
    bimanual.add_argument(
        "--x-rpy",
        type=float,
        nargs=3,
        metavar=("ROLL", "PITCH", "YAW"),
        default=None,
        help=(
            "相对位姿 X_LR 的姿态（**rad**）；"
            f"默认 {BIMANUAL_DEFAULT_X_RPY}（占位值）"
        ),
    )
    bimanual.add_argument(
        "--x-matrix",
        type=float,
        nargs=16,
        metavar=tuple(f"M{i}" for i in range(16)),
        default=None,
        help="相对位姿 X_LR 的完整 4×4 矩阵（行优先；给了就覆盖 --x-xyz/--x-rpy）",
    )
    bimanual.add_argument(
        "--leader",
        choices=("left", "right"),
        default="left",
        help="leader 臂（约束的参考臂；默认 left）",
    )
    bimanual.add_argument(
        "--follower",
        choices=("left", "right"),
        default="right",
        help="follower 臂（跟随 leader 保持相对位姿；默认 right）",
    )
    bimanual.add_argument(
        "--start-joints",
        type=float,
        nargs=7,
        metavar=("Q1", "Q2", "Q3", "Q4", "Q5", "Q6", "Q7"),
        default=None,
        help=(
            "leader 起始位形（rad，7 项）；不给则从内置候选表里挑一个"
            "**可行**的（follower 有解 + 限位内 + 碰撞自由）"
        ),
    )
    bimanual.add_argument(
        "--goal-joints",
        type=float,
        nargs=7,
        metavar=("Q1", "Q2", "Q3", "Q4", "Q5", "Q6", "Q7"),
        default=None,
        help="leader 终点位形（rad，7 项）；不给则取起点 + --goal-step（单轴微移）",
    )
    bimanual.add_argument(
        "--goal-axis",
        type=int,
        default=0,
        help="默认终点在 leader 的哪个关节上微移（0 起；默认 0）",
    )
    bimanual.add_argument(
        "--goal-step",
        type=float,
        default=0.02,
        help="默认终点的微移幅度（rad，默认 0.02）",
    )
    bimanual.add_argument(
        "--num-regions",
        type=int,
        default=3,
        help="最小坐标凸区域数（默认 3）",
    )
    bimanual.add_argument(
        "--path-step",
        type=float,
        default=0.005,
        help="leader 路径推进步长（rad，默认 0.005；越大越快但重定时残差越大）",
    )
    bimanual.add_argument(
        "--psi-grid",
        type=int,
        default=33,
        help="follower 肘点方位角 ψ 的扫描网格数（默认 33）",
    )
    bimanual.add_argument(
        "--collision-edge-step-size",
        type=float,
        default=0.02,
        help="14 维碰撞复检的密采样步长（rad，默认 0.02）",
    )
    bimanual.add_argument(
        "--dt",
        type=float,
        default=0.005,
        help="输出轨迹的采样周期（s，默认 0.005）",
    )
    bimanual.add_argument(
        "--seed",
        type=int,
        default=20250516,
        help="区域生成 / 采样基线的随机种子（默认 20250516；固定 ⇒ 可复现）",
    )
    bimanual.add_argument(
        "--no-degradation",
        dest="allow_degradation",
        action="store_false",
        default=True,
        help="禁止降级（区域链不完整时直接失败，不走采样基线）",
    )
    bimanual.add_argument(
        "--json",
        action="store_true",
        help="以**单行** JSON 输出结果摘要（status / reason / stage / diagnostics）",
    )

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

    calibrate = subparsers.add_parser(
        "calibrate",
        help="双臂标定作业（离线解算 + 残差报告；无实测数据 ⇒ SKIP）",
        description=(
            "双臂标定作业入口（**离线**，零硬件）：解算关节零位偏差 / 工具 TCP 6D /"
            "双臂基座外参 X_LR，输出残差摘要（RMS / 最大残差 / 逐点残差 / 重复性 /"
            "交叉验证差异）。\n"
            "* 无实测数据（标定文件 measured=false 或采样是合成数据且未显式"
            "--accept-synthetic）：输出 SKIP(reason=calibration_not_measured) 并以 "
            "0 退出，同时逐条列出**未验证的物理指标**（既不伪装成功，也不伪装失败）；\n"
            "* 残差超阈值 ⇒ 退出码 1；用法错误（路径缺失 / 参数非法）⇒ 退出码 2。"
        ),
    )
    calibrate.add_argument(
        "--samples",
        default=DEFAULT_CALIBRATION_SAMPLES,
        help=f"采样点集文件路径（默认：{DEFAULT_CALIBRATION_SAMPLES}）",
    )
    calibrate.add_argument(
        "--calibration",
        default=DEFAULT_CALIBRATION_FILE,
        help=f"标定文件路径（默认：{DEFAULT_CALIBRATION_FILE}）",
    )
    calibrate.add_argument(
        "--out", default=None, help="把结果摘要写成 JSON 文件（路径）"
    )
    calibrate.add_argument(
        "--accept-synthetic",
        action="store_true",
        help=(
            "显式接受合成数据（**仅离线演练**）：状态可以是 ok，"
            "但 measured 仍为 false，未验证的物理指标照旧逐条列出"
        ),
    )
    calibrate.add_argument(
        "--fk",
        choices=("auto", "model", "synthetic"),
        default="auto",
        help=(
            "关节零位求解的 FK 对照口径：auto（默认，合成采样 ⇒ synthetic，"
            "否则 ⇒ model）/ model（MDH 独立正解，实测口径）/ synthetic"
            "（合成演练口径）"
        ),
    )
    calibrate.add_argument(
        "--joint-zero-threshold",
        type=float,
        default=None,
        help="关节零位残差阈值（m，默认 1e-3）",
    )
    calibrate.add_argument(
        "--tool-tcp-threshold",
        type=float,
        default=None,
        help="工具 TCP 残差阈值（m，默认 5e-4）",
    )
    calibrate.add_argument(
        "--base-extrinsic-threshold",
        type=float,
        default=None,
        help="基座外参残差阈值（m，默认 5e-4）",
    )
    calibrate.add_argument("--json", action="store_true", help="以**单行** JSON 输出结果摘要")

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
        if command == "calibrate":
            return _cmd_calibrate(args)
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


# ── bimanual（双臂紧协调，S05）──────────────────────────────────────────
def _cmd_bimanual(args: argparse.Namespace) -> int:
    """``bimanual``：双臂紧协调规划（S05）。

    退出码
    ------
    * ``0``：规划成功（两条**共享时间轴**的 7 维轨迹 + 零逃逸 + 相对位姿
      残差逐点达标）；
    * ``1``：业务失败（follower 无解 / 无路径 / 逃逸 / 残差超阈值 /
      找不到可行起始位形）；
    * ``2``：用法错误（场景缺失 / 参数非法 / X_LR 非法）。

    真机 / 物理指标**无真机 ⇒ 硬件门控 SKIP**：逐条列入
    ``unverified_metrics``（绝不拿合成数据冒充物理达标）。
    """
    from .bimanual.dual_scene import build_dual_scene_from_path
    from .coordinated.planner import CoordinatedPlanOptions, plan_coordinated

    scenario_path = Path(args.scenario)
    if not scenario_path.is_file():
        raise _UsageError(f"双臂场景真相源不存在：{scenario_path}")
    try:
        bundle = build_dual_scene_from_path(scenario_path)
    except Exception as exc:  # 场景非法 → 用法错误（点名原因）
        raise _UsageError(f"双臂场景装配失败：{exc}") from exc

    x_lr, x_meta = _bimanual_x_lr(args)
    leader = str(getattr(args, "leader", "left"))
    follower = str(getattr(args, "follower", "right"))
    if leader == follower:
        raise _UsageError(f"--leader 与 --follower 不能相同（当前都是 {leader}）")
    psi_grid = _bimanual_positive_int(getattr(args, "psi_grid", 33), "--psi-grid")
    path_step = _bimanual_positive_float(
        getattr(args, "path_step", 0.005), "--path-step"
    )
    dt = _bimanual_positive_float(getattr(args, "dt", 0.005), "--dt")
    edge_step = _bimanual_positive_float(
        getattr(args, "collision_edge_step_size", 0.02), "--collision-edge-step-size"
    )
    num_regions = _bimanual_positive_int(
        getattr(args, "num_regions", 3), "--num-regions"
    )
    goal_step = _bimanual_non_negative_float(
        getattr(args, "goal_step", 0.02), "--goal-step"
    )

    # ── 起点 / 终点 ────────────────────────────────────────────────────
    start, psi_start, start_source = _bimanual_resolve_start(
        args, bundle, x_lr, psi_grid=psi_grid, leader=leader, follower=follower
    )
    if start is None:
        payload = _bimanual_payload(
            success=False,
            status="failed",
            reason="no_feasible_start",
            stage="parameterize",
            message=(
                "[parameterize] 失败（no_feasible_start）：候选表里没有"
                "任何**可行**的 leader 起始位形（follower 有解析 IK 解 + "
                "14 维限位内 + 14 维碰撞自由）；请显式给 --start-joints "
                "或调整 --x-xyz / --x-rpy"
            ),
            scenario_path=scenario_path,
            x_meta=x_meta,
            leader=leader,
            follower=follower,
            start=None,
            goal=None,
            diagnostics={},
        )
        _emit_bimanual(args, payload)
        return EXIT_FAILURE

    goal = _bimanual_resolve_goal(args, start, goal_step=goal_step)

    options = CoordinatedPlanOptions(
        num_regions=num_regions,
        path_step=path_step,
        dt=dt,
        psi_grid=psi_grid,
        collision_edge_step_size=edge_step,
        allow_degradation=bool(getattr(args, "allow_degradation", True)),
        random_seed=int(getattr(args, "seed", 20250516)),
    )
    result = plan_coordinated(
        bundle,
        x_lr=x_lr,
        start_leader_q=start,
        goal_leader_q=goal,
        leader=leader,
        follower=follower,
        psi_start=psi_start,
        options=options,
    )
    diagnostics = dict(result.diagnostics)
    diagnostics["start_source"] = start_source
    payload = _bimanual_payload(
        success=bool(result.success),
        status="ok" if result.success else "failed",
        reason=str(result.reason or ""),
        stage=str(result.stage or ""),
        message=str(result.message),
        scenario_path=scenario_path,
        x_meta=x_meta,
        leader=leader,
        follower=follower,
        start=start,
        goal=goal,
        diagnostics=diagnostics,
    )
    _emit_bimanual(args, payload)
    return EXIT_OK if result.success else EXIT_FAILURE


def _bimanual_payload(
    *,
    success: bool,
    status: str,
    reason: str,
    stage: str,
    message: str,
    scenario_path: Path,
    x_meta: Dict[str, Any],
    leader: str,
    follower: str,
    start: Optional[np.ndarray],
    goal: Optional[np.ndarray],
    diagnostics: Dict[str, Any],
) -> Dict[str, Any]:
    """组装 ``bimanual`` 的载荷（**成功 / 失败同构**）。"""
    return {
        "command": "bimanual",
        "status": status,
        "success": bool(success),
        "reason": reason,
        "stage": stage,
        "message": message,
        "scenario": str(scenario_path),
        "x_lr": x_meta,
        "leader": leader,
        "follower": follower,
        "start_joints": None if start is None else [float(v) for v in start],
        "goal_joints": None if goal is None else [float(v) for v in goal],
        "diagnostics": diagnostics,
        "unverified_metrics": list(UNVERIFIED_COORDINATED_METRICS),
    }


def _emit_bimanual(args: argparse.Namespace, payload: Dict[str, Any]) -> None:
    """输出 ``bimanual`` 结果（``--json`` ⇒ **单行** JSON；否则人类可读）。"""
    if getattr(args, "json", False):
        print(json.dumps(payload, ensure_ascii=False, default=str))
        return
    print(f"[bimanual] 状态：{payload['status']}"
          + (f"（reason={payload['reason']}）" if payload["reason"] else ""))
    print(f"[bimanual] 场景：{payload['scenario']}")
    x_meta = payload["x_lr"]
    print(
        f"[bimanual] 相对位姿 X_LR：平移={x_meta['translation']} "
        f"姿态={x_meta['rpy']}（来源：{x_meta['source']}）"
    )
    print(f"[bimanual] {payload['message']}")
    diagnostics = payload.get("diagnostics") or {}
    for key in (
        "num_regions",
        "num_escapes",
        "branch_switches",
        "psi_switches",
        "relative_pose_residual_max_position_m",
        "relative_pose_residual_max_orientation_rad",
        "region_duration_s",
        "gcs_duration_s",
        "map_duration_s",
        "retime_duration_s",
        "collision_duration_s",
        "total_duration_s",
    ):
        if key in diagnostics:
            print(f"[bimanual] {key} = {diagnostics[key]}")
    if diagnostics.get("degraded"):
        print(
            "[bimanual] **已降级**（采样基线兜底）："
            f"{diagnostics.get('degradation_reason') or 'n/a'}"
        )
    print("[bimanual] 以下物理指标**未验证**（不得当作通过）：")
    for item in payload["unverified_metrics"]:
        print(f"[bimanual]   - {item}")


def _bimanual_positive_float(value: Any, name: str) -> float:
    """把参数规整成**正有限浮点数**（非法 → :class:`_UsageError`，点名）。"""
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise _UsageError(f"{name} 必须是数值，实际：{value!r}") from exc
    if not np.isfinite(number):
        raise _UsageError(f"{name} 必须是有限值，实际：{value!r}")
    if number <= 0.0:
        raise _UsageError(f"{name} 必须是正有限值，实际：{value!r}")
    return number


def _bimanual_non_negative_float(value: Any, name: str) -> float:
    """把参数规整成**非负有限浮点数**（非法 → :class:`_UsageError`，点名）。"""
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise _UsageError(f"{name} 必须是数值，实际：{value!r}") from exc
    if not np.isfinite(number):
        raise _UsageError(f"{name} 必须是有限值，实际：{value!r}")
    if number < 0.0:
        raise _UsageError(f"{name} 必须是非负有限值，实际：{value!r}")
    return number


def _bimanual_positive_int(value: Any, name: str) -> int:
    """把参数规整成**正整数**（非法 → :class:`_UsageError`，点名）。"""
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise _UsageError(f"{name} 必须是整数，实际：{value!r}") from exc
    if not np.isfinite(number) or abs(number - round(number)) > 1e-9:
        raise _UsageError(f"{name} 必须是整数，实际：{value!r}")
    if int(round(number)) < 1:
        raise _UsageError(f"{name} 必须是正整数，实际：{value!r}")
    return int(round(number))


def _bimanual_x_lr(args: argparse.Namespace) -> Tuple[np.ndarray, Dict[str, Any]]:
    """从命令行参数构造相对位姿 ``X_LR``（**参数非法 ⇒ 用法错误**）。"""
    try:
        return bimanual_x_lr_matrix(
            translation=getattr(args, "x_xyz", None),
            rpy=getattr(args, "x_rpy", None),
            matrix=getattr(args, "x_matrix", None),
        )
    except ValueError as exc:
        raise _UsageError(str(exc)) from exc


def bimanual_x_lr_matrix(
    *,
    translation: Optional[Sequence[float]] = None,
    rpy: Optional[Sequence[float]] = None,
    matrix: Optional[Sequence[float]] = None,
) -> Tuple[np.ndarray, Dict[str, Any]]:
    """构造相对位姿 ``X_LR``（4×4，leader TCP 系 → follower TCP 系）。

    * ``matrix`` 给了 → 直接用（行优先；末行必须是 ``[0,0,0,1]``）；
    * 否则用 ``translation`` / ``rpy``；都不给 → 用**占位默认值**
      （:data:`BIMANUAL_DEFAULT_X_XYZ` / :data:`BIMANUAL_DEFAULT_X_RPY`），
      并在元信息里标注 ``source="default_placeholder"``（**不隐瞒**）。

    非法输入 → :class:`ValueError`（**点名**）。
    """
    if matrix is not None:
        array = np.asarray(matrix, dtype=float).reshape(4, 4)
        if not np.all(np.isfinite(array)):
            raise ValueError("--x-matrix 含 NaN/Inf")
        if not np.allclose(array[3, :], np.array([0.0, 0.0, 0.0, 1.0]), atol=1e-9):
            raise ValueError(
                "--x-matrix 的末行必须是 [0, 0, 0, 1]，实际："
                f"{array[3, :].tolist()}"
            )
        rotation = array[:3, :3]
        if not np.allclose(rotation @ rotation.T, np.eye(3), atol=1e-6):
            raise ValueError("--x-matrix 的左上 3×3 必须是正交阵")
        translation = array[:3, 3]
        rpy = _bimanual_rpy_from_matrix(rotation)
        return array, {
            "translation": [float(v) for v in translation],
            "rpy": [float(v) for v in rpy],
            "source": "explicit_matrix",
            "matrix": [[float(v) for v in row] for row in array],
        }

    explicit = translation is not None or rpy is not None
    if translation is None:
        translation = BIMANUAL_DEFAULT_X_XYZ
    if rpy is None:
        rpy = BIMANUAL_DEFAULT_X_RPY
    translation = np.asarray([float(v) for v in translation], dtype=float)
    angles = np.asarray([float(v) for v in rpy], dtype=float)
    if translation.shape != (3,) or angles.shape != (3,):
        raise ValueError("--x-xyz / --x-rpy 必须各给 3 个数")
    if not np.all(np.isfinite(translation)) or not np.all(np.isfinite(angles)):
        raise ValueError("--x-xyz / --x-rpy 含 NaN/Inf")
    matrix = np.eye(4)
    matrix[:3, :3] = _bimanual_rotation_from_rpy(angles)
    matrix[:3, 3] = translation
    return matrix, {
        "translation": [float(v) for v in translation],
        "rpy": [float(v) for v in angles],
        "source": "explicit" if explicit else "default_placeholder",
        "matrix": [[float(v) for v in row] for row in matrix],
    }


def _bimanual_rotation_from_rpy(rpy: Sequence[float]) -> np.ndarray:
    """由 (roll, pitch, yaw)（rad）构造旋转矩阵（**numpy-only**）。

    约定 ``R = Rz(yaw) · Ry(pitch) · Rx(roll)``（与 S02 的
    :func:`rm75_drake.drake_compat.rotation_from_rpy` 一致）。
    """
    roll, pitch, yaw = (float(v) for v in rpy)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def _bimanual_rpy_from_matrix(rotation: np.ndarray) -> Tuple[float, float, float]:
    """由旋转矩阵反解 (roll, pitch, yaw)（rad；与上面的约定互逆）。"""
    sy = -float(rotation[2, 0])
    sy = max(-1.0, min(1.0, sy))
    pitch = math.asin(sy)
    if abs(sy) < 1.0 - 1e-9:
        roll = math.atan2(float(rotation[2, 1]), float(rotation[2, 2]))
        yaw = math.atan2(float(rotation[1, 0]), float(rotation[0, 0]))
    else:  # 万向节奇异：取 roll = 0
        roll = 0.0
        yaw = math.atan2(-float(rotation[0, 1]), float(rotation[1, 1]))
    return (roll, pitch, yaw)


def _bimanual_resolve_start(
    args: argparse.Namespace,
    bundle: Any,
    x_lr: np.ndarray,
    *,
    psi_grid: int,
    leader: str,
    follower: str,
) -> Tuple[Optional[np.ndarray], Optional[float], str]:
    """解析 leader 起始位形。

    * 给了 ``--start-joints`` → 直接用（维度 / 数值非法 → 用法错误）；
    * 否则从 :data:`BIMANUAL_CANDIDATE_LEADER_POSTURES` 里挑**第一个可行**
      的（follower 有解 + 限位内 + 碰撞自由 + ψ 窗口够宽），
      并返回它的 ψ（连续跟踪的起点提示）。

    返回 ``(start, psi_start, source)``；找不到可行起点 ⇒
    ``(None, None, "none")``（**业务失败**，不是用法错误）。
    """
    explicit = getattr(args, "start_joints", None)
    if explicit is not None:
        start = np.asarray(explicit, dtype=float).reshape(-1)
        if start.size != 7:
            raise _UsageError(
                f"--start-joints 必须是 7 个关节角，实际：{start.size} 个"
            )
        if not np.all(np.isfinite(start)):
            raise _UsageError("--start-joints 含 NaN/Inf")
        return start, None, "explicit"

    found = select_bimanual_start(
        bundle, x_lr, psi_grid=psi_grid, leader=leader, follower=follower
    )
    if found is None:
        return None, None, "none"
    start, psi = found
    return start, psi, "candidate_table"


def select_bimanual_start(
    bundle: Any,
    x_lr: np.ndarray,
    *,
    psi_grid: int = 33,
    leader: str = "left",
    follower: str = "right",
) -> Optional[Tuple[np.ndarray, float]]:
    """从候选表里挑一个**可行**的 leader 起始位形（找不到 → ``None``）。

    判据（逐条点名，**不静默换目标**）：follower 有解析 IK 解 + 14 维限位内
    + 14 维碰撞自由 + 可行 ψ 窗口 ≥3 个连续网格点。

    返回 ``(leader_q, psi)``；``psi`` 是连续跟踪的起点提示。
    """
    from .coordinated.parameterization import CoordinatedParameterization

    parameterization = CoordinatedParameterization(
        bundle, x_lr=x_lr, leader=leader, follower=follower
    )
    grid = np.linspace(0.0, 2.0 * math.pi, max(3, int(psi_grid)), endpoint=False)
    for candidate in BIMANUAL_CANDIDATE_LEADER_POSTURES:
        q = np.asarray(candidate, dtype=float)
        psi = _bimanual_pick_psi(parameterization, q, grid)
        if psi is not None:
            return q, psi
    return None


def _bimanual_pick_psi(
    parameterization: Any,
    leader_q: np.ndarray,
    grid: np.ndarray,
) -> Optional[float]:
    """挑一个**可行且窗口够宽**的 ψ（找不到 → ``None``）。

    判据：ψ 网格上**至少 3 个连续可行点**（⇒ 规划器的余量门槛
    ``psi_min_margin`` 能挑到点），取该连续段的中点。
    """
    feasible: List[bool] = []
    for psi in grid:
        x = np.concatenate([leader_q, np.array([float(psi)])])
        try:
            q14 = parameterization.to_joint_config(x)
        except Exception:
            feasible.append(False)
            continue
        feasible.append(
            bool(parameterization.within_limits(q14))
            and bool(parameterization.is_collision_free(q14))
        )
    count = len(grid)
    best_run: List[int] = []
    current: List[int] = []
    # 多走一格（绕回起点）⇒ 跨边界的连续段也能被识别。
    for offset in range(count + 1):
        index = offset % count
        if feasible[index]:
            current.append(index)
        else:
            if len(current) > len(best_run):
                best_run = list(current)
            current = []
    if len(current) > len(best_run):
        best_run = list(current)
    if len(best_run) < 3:
        return None
    middle = best_run[len(best_run) // 2]
    return float(grid[middle % len(grid)])


def _bimanual_resolve_goal(
    args: argparse.Namespace,
    start: np.ndarray,
    *,
    goal_step: float,
) -> np.ndarray:
    """解析 leader 终点位形（不给 ⇒ 起点 + 单轴微移 ``--goal-step``）。"""
    explicit = getattr(args, "goal_joints", None)
    if explicit is not None:
        goal = np.asarray(explicit, dtype=float).reshape(-1)
        if goal.size != 7:
            raise _UsageError(f"--goal-joints 必须是 7 个关节角，实际：{goal.size} 个")
        if not np.all(np.isfinite(goal)):
            raise _UsageError("--goal-joints 含 NaN/Inf")
        return goal
    goal = np.asarray(start, dtype=float).copy()
    axis = int(getattr(args, "goal_axis", 0))
    if axis < 0 or axis >= 7:
        raise _UsageError(f"--goal-axis 必须在 0..6 之间，实际：{axis}")
    goal[axis] = float(goal[axis] + goal_step)
    return goal


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


# ── calibrate（双臂标定作业入口）────────────────────────────────────────
def _cmd_calibrate(args: argparse.Namespace) -> int:
    """``calibrate``：离线标定解算 + 残差报告。

    退出码
    ------
    * ``0``：解算完成（``skipped`` 表示**无实测数据**，只做离线演练；``ok`` 表示
      结论可复算）；
    * ``1``：业务失败（残差超阈值 / 求解器错误 / 数据文件内容非法）；
    * ``2``：用法错误（路径缺失 / 参数非法）。
    """
    from .calibration.pipeline import CalibrationUsageError, run_calibration
    from .calibration.schema import CalibrationError
    from .calibration.solvers import (
        DEFAULT_BASE_EXTRINSIC_THRESHOLD_M,
        DEFAULT_JOINT_ZERO_THRESHOLD_M,
        DEFAULT_TOOL_TCP_THRESHOLD_M,
    )

    try:
        run = run_calibration(
            samples_path=args.samples,
            calibration_path=args.calibration,
            accept_synthetic=bool(getattr(args, "accept_synthetic", False)),
            fk_mode=str(getattr(args, "fk", "auto")),
            joint_zero_threshold=(
                DEFAULT_JOINT_ZERO_THRESHOLD_M
                if args.joint_zero_threshold is None
                else float(args.joint_zero_threshold)
            ),
            tool_tcp_threshold=(
                DEFAULT_TOOL_TCP_THRESHOLD_M
                if args.tool_tcp_threshold is None
                else float(args.tool_tcp_threshold)
            ),
            base_extrinsic_threshold=(
                DEFAULT_BASE_EXTRINSIC_THRESHOLD_M
                if args.base_extrinsic_threshold is None
                else float(args.base_extrinsic_threshold)
            ),
        )
    except CalibrationUsageError as exc:
        raise _UsageError(str(exc)) from exc
    except CalibrationError as exc:
        print(f"cli: 标定数据非法：{exc}", file=sys.stderr)
        return EXIT_FAILURE

    payload = run.as_dict()
    out_path = getattr(args, "out", None)
    if out_path:
        out_file = Path(out_path)
        out_file.parent.mkdir(parents=True, exist_ok=True)
        out_file.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2, default=str), encoding="utf-8"
        )
        payload["output_path"] = str(out_file)

    if getattr(args, "json", False):
        print(json.dumps(payload, ensure_ascii=False, default=str))
    else:
        _print_calibrate_summary(payload)
    return run.exit_code


def _print_calibrate_summary(payload: Dict[str, Any]) -> None:
    """打印人类可读的标定摘要（**必含**未验证项清单）。"""
    status = payload["status"]
    reason = payload["reason"]
    print(f"[calibrate] 状态：{status}" + (f"（reason={reason}）" if reason else ""))
    print(f"[calibrate] 采样文件：{payload['samples_path']}")
    print(f"[calibrate] 标定文件：{payload['calibration_path']}")
    print(f"[calibrate] 实测 measured：{payload['measured']}（FK 口径：{payload['fk_mode']}）")
    for name, section in payload["sections"].items():
        if section["status"] == "absent":
            print(f"[calibrate] {name}：无采样（skipped）")
            continue
        print(
            f"[calibrate] {name}：{section['status']} "
            f"RMS={section['rms']:.6e} m 最大={section['max']:.6e} m "
            f"样本数={section['sample_count']} 阈值={section['threshold']:.6e} m"
        )
        if section.get("detail"):
            print(f"[calibrate]   说明：{section['detail']}")
    if payload["detail"]:
        print(f"[calibrate] 结论：{payload['detail']}")
    if payload["unverified_metrics"]:
        print("[calibrate] 以下物理指标**未验证**（不得当作通过）：")
        for item in payload["unverified_metrics"]:
            print(f"[calibrate]   - {item}")
    if payload.get("output_path"):
        print(f"[calibrate] 结果文件：{payload['output_path']}")


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
