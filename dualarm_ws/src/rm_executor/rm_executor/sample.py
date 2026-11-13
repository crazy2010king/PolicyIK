"""确定性最小轨迹生成器（离线长跑 / 故障注入复跑的**可复现**输入）。

为什么需要它
------------

真机轨迹由 ``rmctl plan``（规划内核 tesseract）产出，而规划内核在离线机器上
不一定可用；S03 的离线证据（600 s 等效长跑、三类故障注入复跑）需要一条
**仓库内可复现**、且**必然通过执行层四道校验**的轨迹。本模块按**闭式公式**
生成一条 7 关节小幅轨迹：同样的参数永远得到同样的字节，因此证据可复跑。

用法::

    python3 -m rm_executor.sample --out build/reports/s03_sample_traj.json

**限值口径（重要）**：执行层的限值复检用中心差分逐阶求导，**分段线性的
拐点会产生真实的加速度 / jerk 尖峰**（按越界处理，见
:func:`rm_executor.contract.check_time_domain_limits`）。因此本轨迹用
**光滑**的幅值包络（正弦）而不是带拐点的折线，保证位置 / 速度 / 加速度 /
jerk 四项都落在 ``rm_hw_bridge.limits`` 之内。

**诚实声明**：本轨迹是**合成**的，不是规划器产物。``verify.ok=true`` 只表示
「本文件满足执行层轨迹契约」；时间域限值由执行器阶段 A 独立复检。离线
证据文档里对此有明确标注。
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from typing import Any, Dict, List, Optional, Sequence

from rm_executor import contract

__all__ = [
    "DEFAULT_AMPLITUDE_RAD",
    "DEFAULT_N_POINTS",
    "DEFAULT_PERIOD_S",
    "build_sample_trajectory",
    "main",
    "write_sample_trajectory",
]

#: 采样周期（秒）—— 与执行层 10 ms 栅格同源的「粗采样」
DEFAULT_PERIOD_S = 0.05

#: 采样点数（41 × 0.05 s = 2.0 s 一条轨迹）
DEFAULT_N_POINTS = 41

#: 关节 1 的幅值（rad）≈ 2.9°；其余关节按系数线性放大
DEFAULT_AMPLITUDE_RAD = 0.05


def _joint_amplitude(amplitude_rad: float, joint_index: int) -> float:
    """第 ``joint_index`` 个关节（0 起）的幅值系数（0.6 → 0.96 逐关节略放大）。"""
    return float(amplitude_rad) * (0.6 + 0.06 * int(joint_index))


def build_sample_trajectory(period_s: float = DEFAULT_PERIOD_S,
                            n_points: int = DEFAULT_N_POINTS,
                            amplitude_rad: float = DEFAULT_AMPLITUDE_RAD,
                            ) -> Dict[str, Any]:
    """构造一条确定性最小轨迹（7 关节小幅平滑运动，单位 s / rad）。

    参数
    ----
    period_s
        采样周期（秒）；必须 > 0。
    n_points
        采样点数；必须 ≥ 2（时间轴严格递增）。
    amplitude_rad
        关节 1 的幅值（rad）；其余关节按 0.6 + 0.06·j 缩放。

    返回
    ----
    dict
        满足 :func:`rm_executor.contract.load_trajectory` 契约的轨迹 payload。
    """
    period_s = float(period_s)
    n_points = int(n_points)
    amplitude_rad = float(amplitude_rad)
    if period_s <= 0:
        raise ValueError("period_s 必须 > 0（当前 %r）" % period_s)
    if n_points < 2:
        raise ValueError("n_points 必须 ≥ 2（当前 %r）" % n_points)

    times: List[float] = [round(i * period_s, 9) for i in range(n_points)]
    duration = times[-1] - times[0]
    omega = 2.0 * math.pi / duration if duration > 0 else 0.0

    points: List[List[float]] = []
    for t in times:
        row: List[float] = []
        for j in range(contract.ARM_DOF):
            amp = _joint_amplitude(amplitude_rad, j)
            phase = 0.35 * j  # 逐关节相位错开 → 多关节（不是单关节）运动
            row.append(round(amp * math.sin(omega * t + phase), 9))
        points.append(row)

    return {
        "schema_version": contract.SCHEMA_VERSION,
        "group": contract.JOINT_GROUP,
        "joint_names": list(contract.JOINT_NAMES),
        "units": dict(contract.UNITS),
        "times": times,
        "points": points,
        "verify": {
            "ok": True,
            "checks": {
                "origin": "rm_executor.sample 合成轨迹（**非规划器产物**）",
                "contract": "满足 rm_executor 轨迹契约（group/joint_names/units/schema_version/times/points）",
                "time_domain_limits": "由执行层阶段 A 用 rm_hw_bridge.limits 独立复检",
                "note": "verify.ok=true 仅代表契约合规，不代表真机轨迹或规划器校验结论",
            },
        },
    }


def write_sample_trajectory(path: str,
                            period_s: float = DEFAULT_PERIOD_S,
                            n_points: int = DEFAULT_N_POINTS,
                            amplitude_rad: float = DEFAULT_AMPLITUDE_RAD,
                            ) -> str:
    """把样本轨迹写到 ``path``（UTF-8，不转义）；返回绝对路径。"""
    payload = build_sample_trajectory(period_s=period_s, n_points=n_points,
                                      amplitude_rad=amplitude_rad)
    target = os.path.abspath(path)
    parent = os.path.dirname(target)
    if parent and not os.path.isdir(parent):
        os.makedirs(parent, exist_ok=True)
    with open(target, "w", encoding="utf-8") as fh:
        json.dump(payload, fh, ensure_ascii=False, indent=2, sort_keys=False)
        fh.write("\n")
    return target


def build_parser() -> argparse.ArgumentParser:
    """构造参数解析器。"""
    parser = argparse.ArgumentParser(
        prog="rm_executor.sample",
        description="生成确定性最小轨迹（离线长跑 / 故障注入复跑的输入）",
    )
    parser.add_argument("--out", required=True, help="输出轨迹 JSON 路径")
    parser.add_argument("--period-s", dest="period_s", type=float,
                        default=DEFAULT_PERIOD_S, help="采样周期（秒）")
    parser.add_argument("--n-points", dest="n_points", type=int,
                        default=DEFAULT_N_POINTS, help="采样点数")
    parser.add_argument("--amplitude-rad", dest="amplitude_rad", type=float,
                        default=DEFAULT_AMPLITUDE_RAD, help="关节 1 幅值（rad）")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    """CLI 主入口；返回进程退出码（0 成功 / 3 参数非法）。"""
    args = build_parser().parse_args(argv)
    try:
        path = write_sample_trajectory(args.out, period_s=args.period_s,
                                       n_points=args.n_points,
                                       amplitude_rad=args.amplitude_rad)
    except ValueError as exc:
        print("生成失败: %s" % exc, file=sys.stderr)
        return contract.EXIT_CODES[contract.STATUS_BLOCKED]
    except OSError as exc:
        print("写入失败: %s" % exc, file=sys.stderr)
        return contract.EXIT_CODES[contract.STATUS_ERROR]
    print("trajectory  : %s" % path)
    print("duration_s  : %s" % ((args.n_points - 1) * args.period_s))
    return contract.EXIT_CODES[contract.STATUS_OK]


if __name__ == "__main__":
    sys.exit(main())
