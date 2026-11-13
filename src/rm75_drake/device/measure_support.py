"""真机测量脚本的**共享支撑层**（S03-T08）。

为什么需要它
------------
``scripts/measure_jitter.py`` / ``scripts/soak_test.py`` /
``scripts/fault_inject.py`` 三个脚本必须共用**同一套**：

* 设备配置加载（``devices/*.yaml``）与**严格校验**；
* 安全七闸阈值来源（软件软限位 + 设备配置里的标定值）；
* 「保持位姿 + 5 ms 节拍」的测量循环（抖动 / 延迟口径**唯一**，
  直接复用 :mod:`rm75_drake.device.exec_loop` 与
  :mod:`rm75_drake.device.metrics`，**禁止两套口径**）；
* 硬件门控与 ``SKIP`` 输出（:mod:`rm75_drake.device.hardware_gate`）。

⇒ 支撑逻辑收敛到本模块，三个脚本只保留**各自的口径与门控**。

安全纪律
--------
* 真机测量**必须显式给出保持位姿**（``--joint-rad``）——脚本绝不自己编一个
  关节目标下发到真机（否则可能把机械臂拽到一个未预期的姿态）；
* 离线替身（``--dry-run``）走
  :class:`~rm75_drake.device.transport.ScriptedTransport`，**零硬件**；
* 本模块**零 pydrake 依赖**。
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

from .. import paths
from ..planning.trajectory import JOINT_COUNT
from .config import DeviceConfig, DeviceConfigError, load_device_config
from .exec_loop import ExecLoop, ExecLoopConfig, ExecLoopResult, SystemClock
from .safety import SafetyLimits, gate_fence, safety_limits_from_sources

__all__ = [
    "DEFAULT_DEVICE_CONFIG",
    "DEFAULT_PERIOD_S",
    "DEFAULT_DURATION_S",
    "HOLD_POSE_CANDIDATES",
    "add_common_arguments",
    "resolve_device_config",
    "parse_joint_rad",
    "format_joint_rad",
    "default_hold_pose",
    "build_safety_limits",
    "run_hold_measurement",
    "measurement_summary",
]


#: 默认设备配置真相源。
DEFAULT_DEVICE_CONFIG = str(paths.devices_dir / "single_arm.yaml")

#: 默认节拍周期（s）——5 ms 透传。
DEFAULT_PERIOD_S = 0.005

#: 默认测量时长（s）。
DEFAULT_DURATION_S = 60.0

#: **离线演练**用的候选保持位姿（rad，7 维）。
#:
#: 零位构型的 TCP 正在基座轴线上方（水平距离 0 m < 奇异圆柱半径 0.15 m），
#: 会被安全闸 G6 判为越界 ⇒ **不能**拿零位当默认保持位姿。
#: 下面这组候选按「水平距离 ≥ 奇异圆柱半径 且 高度 ≥ 桌面高度」挑选，
#: 由 :func:`default_hold_pose` 按当前设备配置的围栏参数**逐个校验**后取第一个
#: 合法者（确定性、无随机）。
HOLD_POSE_CANDIDATES = (
    (0.0, -0.5, 0.0, 0.4, 0.0, -0.6, 0.0),
    (0.0, -0.7, 0.0, 0.4, 0.0, 0.2, 0.0),
    (0.0, -0.6, 0.0, 0.6, 0.0, 0.0, 0.0),
    (0.0, -0.4, 0.0, 0.5, 0.0, -0.5, 0.0),
)


# ── 命令行公共项 ────────────────────────────────────────────────────────
def add_common_arguments(parser: argparse.ArgumentParser) -> argparse.ArgumentParser:
    """给测量脚本挂上**公共**命令行项。

    公共项：``--device-config`` / ``--connect-timeout`` / ``--joint-rad`` /
    ``--dry-run`` / ``--json``。
    """
    parser.add_argument(
        "--device-config",
        default=DEFAULT_DEVICE_CONFIG,
        help=f"设备（部署）配置路径（默认：{DEFAULT_DEVICE_CONFIG}）",
    )
    parser.add_argument(
        "--connect-timeout",
        type=float,
        default=1.0,
        help="探测真机 TCP 指令通道的超时秒数（默认：1.0）",
    )
    parser.add_argument(
        "--joint-rad",
        default=None,
        help=(
            "保持位姿的 7 个关节角（rad，空格分隔）。"
            "真机测量**必须**显式给出（脚本不会自己编造下发目标）"
        ),
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="离线演练：走传输层替身（零硬件），结果**不代表真机指标**",
    )
    parser.add_argument("--json", action="store_true", help="以 JSON 输出结果摘要")
    return parser


# ── 配置 / 位姿 ─────────────────────────────────────────────────────────
def resolve_device_config(path: str) -> DeviceConfig:
    """加载并校验设备配置（**异常向上冒泡**，不静默兜底）。"""
    target = Path(path)
    if not target.is_file():
        raise DeviceConfigError(f"设备配置文件不存在：{target}")
    return load_device_config(target)


def parse_joint_rad(text: Optional[str]) -> Optional[Tuple[float, ...]]:
    """解析 ``--joint-rad``（7 个以空白分隔的弧度值）。

    返回
    ----
    ``None``
        未提供；
    否则返回 7 维元组（rad）。

    异常
    ----
    ValueError
        个数不是 7 / 含非数值 / 含 NaN/Inf（消息点名问题）。
    """
    if text is None:
        return None
    parts = [item for item in str(text).replace(",", " ").split() if item]
    if len(parts) != JOINT_COUNT:
        raise ValueError(
            f"--joint-rad 必须是 {JOINT_COUNT} 个关节角（rad），实际是 {len(parts)} 个"
        )
    values: List[float] = []
    for index, item in enumerate(parts):
        try:
            value = float(item)
        except ValueError as exc:
            raise ValueError(f"--joint-rad[{index}] 不是数值：{item!r}") from exc
        if not math.isfinite(value):
            raise ValueError(f"--joint-rad[{index}] 必须是有限数值：{item!r}")
        values.append(value)
    return tuple(values)


def format_joint_rad(joint_rad: Sequence[float]) -> str:
    """把关节角格式化成一行可读文本（rad）。"""
    return "[" + ", ".join(f"{float(v):.6f}" for v in joint_rad) + "] rad"


# ── 安全阈值 ────────────────────────────────────────────────────────────
def default_hold_pose(limits: SafetyLimits) -> Tuple[float, ...]:
    """返回**离线演练**用的默认保持位姿（rad，7 维）。

    候选位姿按 :data:`HOLD_POSE_CANDIDATES` 顺序逐个检查：
    * 必须**不**触发围栏闸 G6（TCP 在桌面之上且在奇异圆柱之外）；
    * 必须落在软件软限位区间内。

    若候选全部不合法（设备配置的围栏很苛刻），返回第一个候选
    （让安全闸照常停机并给出诊断——**不静默换一个非法位姿**）。
    """
    for pose in HOLD_POSE_CANDIDATES:
        if gate_fence(pose, limits):
            continue
        if not (
            all(lo <= v <= hi for lo, hi, v in zip(limits.soft_lower_rad, limits.soft_upper_rad, pose))
        ):
            continue
        return tuple(float(v) for v in pose)
    return tuple(float(v) for v in HOLD_POSE_CANDIDATES[0])


def build_safety_limits(device_config: DeviceConfig) -> SafetyLimits:
    """构造安全七闸阈值（软件软限位 + 设备配置里的标定值）。"""
    from ..limits import load_joint_limits

    return safety_limits_from_sources(load_joint_limits(), device_config)


# ── 测量循环 ────────────────────────────────────────────────────────────
def run_hold_measurement(
    *,
    transport: Any,
    joint_rad: Sequence[float],
    limits: SafetyLimits,
    period_s: float = DEFAULT_PERIOD_S,
    duration_s: float = DEFAULT_DURATION_S,
    require_state: bool = False,
    clock: Optional[Any] = None,
) -> ExecLoopResult:
    """按固定节拍**保持**一个位姿，返回执行环结果（含抖动 / 延迟统计）。

    参数
    ----
    transport:
        传输层（真机 / 替身）。
    joint_rad:
        保持位姿（rad，7 维）。
    limits:
        安全七闸阈值。
    period_s:
        节拍周期（s）。
    duration_s:
        测量时长（s）⇒ 拍数 = ``round(duration_s / period_s)``。
    require_state:
        是否要求每拍都读到状态帧。**默认 ``False``**：抖动测量关心的是
        「下发节拍」，状态帧的在线监视由真机运行时的执行环负责。
    clock:
        时钟（默认 :class:`~rm75_drake.device.exec_loop.SystemClock`）。

    返回
    ----
    :class:`~rm75_drake.device.exec_loop.ExecLoopResult`
    """
    period = float(period_s)
    if not math.isfinite(period) or period <= 0.0:
        raise ValueError(f"period_s 必须是正有限数值，实际是 {period_s!r}")
    duration = float(duration_s)
    if not math.isfinite(duration) or duration <= 0.0:
        raise ValueError(f"duration_s 必须是正有限数值，实际是 {duration_s!r}")
    ticks = max(1, int(round(duration / period)))
    points = [tuple(float(v) for v in joint_rad) for _ in range(ticks)]
    loop = ExecLoop(
        transport,
        points,
        limits,
        config=ExecLoopConfig(
            period_s=period,
            max_ticks=ticks,
            require_state=require_state,
        ),
        clock=clock if clock is not None else SystemClock(),
    )
    return loop.run()


def measurement_summary(
    result: ExecLoopResult, *, joint_rad: Sequence[float], period_s: float
) -> Dict[str, Any]:
    """把执行环结果整理成可 JSON 序列化的测量摘要。

    抖动 / 延迟统计**直接来自**
    :class:`~rm75_drake.device.metrics.MetricHistogram`（唯一口径）。
    """
    jitter = result.jitter.summary_ms()
    latency = result.latency.summary_ms()
    return {
        "joint_rad": [float(v) for v in joint_rad],
        "period_ms": float(period_s) * 1000.0,
        "ticks": int(result.ticks),
        "commands_sent": int(result.commands_sent),
        "final_state": result.final_state,
        "stop_reason": result.stop_reason,
        "reasons": list(result.reasons),
        "num_violations": len(result.violations),
        "num_errors": len(result.errors),
        "errors": list(result.errors),
        "jitter_ms": jitter,
        "command_latency_ms": latency,
    }
