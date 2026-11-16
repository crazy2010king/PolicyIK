#!/usr/bin/env python3
"""协调规划指标脚本（S05-T06）。

用途
----
跑 **N 组确定性的「leader 起点 → 终点」紧协调任务**，输出：

* **成功率**（规划成功的比例）；
* **求解阶段耗时 P50 / P95**（对应 R017 的验收口径：参数化 + GCS + 映射 +
  重定时）；
* **端到端耗时 P50 / P95**（**另列**：含区域生成等开销）；
* **区域生成单区域平均耗时**；
* **相对位姿残差统计**（位置 / 姿态，逐点口径的 max）；
* **零逃逸率**（14 维复检逃逸数 = 0 的比例）。

口径纪律（与 ``scripts/plan_stats.py`` 同构）
--------------------------------------------
**验收口径**与**端到端口径分行打印**，并写明哪一条对应验收：

* **验收口径** = 「离线成功率 ≥ 95%、**求解阶段** P95 < 3 s」（R017）。
  求解阶段 = 参数化 + GCS + 映射回 14 维 + 重定时；**不含**区域生成。
* **端到端口径** = 整条链路（含区域生成），**单列**，不与验收口径混用。

**真机 / 物理指标一律硬件门控 SKIP**：真实成功率 ≥95%、真实求解 P95、
真实共持位姿漂移 <2 mm/10 s、真实相对位姿保持、真机零逃逸 —— 本环境
**无真机** ⇒ 逐条列入 ``unverified_metrics``，**退出码 0**，
**绝不用合成数据冒充物理达标**。

用法
----
::

    python3 scripts/coordinated_stats.py                     # 默认 100 组
    python3 scripts/coordinated_stats.py --samples 20        # 调整规模
    python3 scripts/coordinated_stats.py --json              # 机器可读输出
    python3 scripts/coordinated_stats.py --max-seconds 600   # 时间预算

退出码
------
``0``  至少完成一组采样并打印了实测指标；
``1``  一组都没跑成（环境不可用 / 参数非法）。
"""

from __future__ import annotations

import argparse
import json
import math
import random
import statistics
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = REPO_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

#: 默认场景真相源（双臂场景；S04 的装配器）。
DEFAULT_SCENARIO = REPO_ROOT / "scenarios" / "dual_arm.yaml"

#: **求解阶段** P95 的验收目标（s；R017）。
SOLVE_P95_TARGET_S = 3.0

#: 起点扰动幅度（rad）——紧协调任务的「小位移」口径。
START_JITTER_RAD = 0.005

#: 终点相对起点的位移幅度区间（rad）。
GOAL_STEP_RANGE_RAD = (0.01, 0.03)


def _percentile(values: Sequence[float], fraction: float) -> Optional[float]:
    """返回分位数（线性插值）；空序列 → ``None``。"""
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return float(ordered[0])
    position = fraction * (len(ordered) - 1)
    low = int(math.floor(position))
    high = int(math.ceil(position))
    if low == high:
        return float(ordered[low])
    weight = position - low
    return float(ordered[low] * (1.0 - weight) + ordered[high] * weight)


def _fmt(value: Optional[float], digits: int = 3, unit: str = "") -> str:
    """格式化一个可能为 ``None`` 的数值。"""
    if value is None:
        return "n/a"
    return f"{value:.{digits}f}{unit}"


def _solve_duration(diagnostics: Dict[str, Any]) -> Optional[float]:
    """求解阶段耗时（s）= 参数化 + GCS + 映射 + 重定时（**不含区域生成**）。"""
    total = 0.0
    found = False
    for stage in ("parameterize", "gcs", "map", "retime"):
        section = diagnostics.get(stage)
        if isinstance(section, dict) and section.get("duration_s") is not None:
            total += float(section["duration_s"])
            found = True
    return total if found else None


def _fmt_sci(value: Optional[float]) -> str:
    """把残差这类**极小量**格式化成科学计数法（``None`` → ``n/a``）。"""
    if value is None:
        return "n/a"
    return f"{value:.3e}"


def _sample_task(
    rng: random.Random,
    base_start: Sequence[float],
    limits: Any,
) -> List[Any]:
    """采样一组任务（起点扰动 + 终点单轴位移），落在软限位内。"""
    lower = list(limits.lower)
    upper = list(limits.upper)
    start: List[float] = []
    for index, base in enumerate(base_start):
        value = float(base) + rng.uniform(-START_JITTER_RAD, START_JITTER_RAD)
        value = min(max(value, lower[index]), upper[index])
        start.append(value)
    axis = rng.randrange(len(start))
    step = rng.uniform(*GOAL_STEP_RANGE_RAD)
    goal = list(start)
    goal[axis] = min(goal[axis] + step, upper[axis])
    return [start, goal, axis, step]


def run_stats(
    *,
    samples: int = 100,
    seed: int = 0,
    scenario_path: Path = DEFAULT_SCENARIO,
    max_seconds: Optional[float] = None,
    x_xyz: Optional[Sequence[float]] = None,
    x_rpy: Optional[Sequence[float]] = None,
    num_regions: int = 3,
    path_step: float = 0.005,
    psi_grid: int = 33,
) -> Dict[str, Any]:
    """跑指标采样并返回结果字典（**如实**记录实测值）。"""
    from rm75_drake.bimanual.dual_scene import build_dual_scene_from_path
    from rm75_drake.cli import (
        UNVERIFIED_COORDINATED_METRICS,
        bimanual_x_lr_matrix,
        select_bimanual_start,
    )
    from rm75_drake.coordinated.planner import CoordinatedPlanOptions, plan_coordinated
    from rm75_drake.planning.trajectory import software_limits

    bundle = build_dual_scene_from_path(scenario_path)
    x_lr, x_meta = bimanual_x_lr_matrix(translation=x_xyz, rpy=x_rpy)
    limits = software_limits()
    rng = random.Random(seed)

    found = select_bimanual_start(bundle, x_lr, psi_grid=psi_grid)
    if found is None:
        return {
            "samples_requested": samples,
            "samples_completed": 0,
            "truncated": False,
            "seed": seed,
            "scenario": str(scenario_path),
            "x_lr": x_meta,
            "success_rate": 0.0,
            "success_count": 0,
            "failure_breakdown": {"parameterize:no_feasible_start": samples},
            "solve_duration_s": {"p50": None, "p95": None, "mean": None, "max": None},
            "end_to_end_duration_s": {"p50": None, "p95": None, "mean": None, "max": None},
            "solve_p95_s": None,
            "end_to_end_p95_s": None,
            "region_per_region_mean_s": None,
            "relative_pose_residual": {
                "max_position_m": None,
                "max_orientation_rad": None,
                "mean_position_m": None,
            },
            "zero_escape_rate": 0.0,
            "solve_p95_target_s": SOLVE_P95_TARGET_S,
            "solve_p95_target_met": False,
            "acceptance_metrics": {
                "success_rate": 0.0,
                "solve_p95_s": None,
                "zero_escape_rate": 0.0,
            },
            "end_to_end_metrics": {"end_to_end_p50_s": None, "end_to_end_p95_s": None},
            "unverified_metrics": list(UNVERIFIED_COORDINATED_METRICS),
            "total_wall_time_s": 0.0,
        }

    base_start, psi_start = found
    options = CoordinatedPlanOptions(
        num_regions=num_regions,
        path_step=path_step,
        psi_grid=psi_grid,
    )

    solve_durations: List[float] = []
    end_to_end_durations: List[float] = []
    region_per_region: List[float] = []
    residuals_position: List[float] = []
    residuals_orientation: List[float] = []
    successes = 0
    zero_escape = 0
    failures: Dict[str, int] = {}
    truncated = False
    started = time.perf_counter()

    for _index in range(samples):
        if max_seconds is not None and (time.perf_counter() - started) > max_seconds:
            truncated = True
            break
        start_q, goal_q, _axis, _step = _sample_task(rng, base_start, limits)
        result = plan_coordinated(
            bundle,
            x_lr=x_lr,
            start_leader_q=start_q,
            goal_leader_q=goal_q,
            psi_start=psi_start,
            options=options,
        )
        diagnostics = dict(result.diagnostics)
        end_to_end = result.duration
        if end_to_end is None:
            end_to_end = float(diagnostics.get("total_duration_s", 0.0))
        end_to_end_durations.append(float(end_to_end))
        solve = _solve_duration(diagnostics)
        if solve is not None:
            solve_durations.append(float(solve))
        regions = diagnostics.get("regions")
        if isinstance(regions, dict):
            count = int(regions.get("num_regions", 0) or 0)
            if count > 0 and regions.get("duration_s") is not None:
                region_per_region.append(float(regions["duration_s"]) / count)
        position = diagnostics.get("relative_pose_residual_max_position_m")
        if position is not None:
            residuals_position.append(float(position))
        orientation = diagnostics.get("relative_pose_residual_max_orientation_rad")
        if orientation is not None:
            residuals_orientation.append(float(orientation))
        escapes = int(diagnostics.get("num_escapes", 0) or 0)
        if escapes == 0:
            zero_escape += 1
        if result.success:
            successes += 1
        else:
            key = f"{result.stage or 'unknown'}:{result.reason or 'unknown'}"
            failures[key] = failures.get(key, 0) + 1

    completed = len(end_to_end_durations)
    solve_p95 = _percentile(solve_durations, 0.95)
    end_to_end_p95 = _percentile(end_to_end_durations, 0.95)
    report: Dict[str, Any] = {
        "samples_requested": samples,
        "samples_completed": completed,
        "truncated": truncated,
        "seed": seed,
        "scenario": str(scenario_path),
        "x_lr": x_meta,
        "success_rate": (successes / completed) if completed else 0.0,
        "success_count": successes,
        "failure_breakdown": failures,
        "solve_duration_s": {
            "p50": _percentile(solve_durations, 0.50),
            "p95": solve_p95,
            "mean": (statistics.fmean(solve_durations) if solve_durations else None),
            "max": (max(solve_durations) if solve_durations else None),
        },
        "end_to_end_duration_s": {
            "p50": _percentile(end_to_end_durations, 0.50),
            "p95": end_to_end_p95,
            "mean": (statistics.fmean(end_to_end_durations) if end_to_end_durations else None),
            "max": (max(end_to_end_durations) if end_to_end_durations else None),
        },
        "solve_p95_s": solve_p95,
        "end_to_end_p95_s": end_to_end_p95,
        "region_per_region_mean_s": (
            statistics.fmean(region_per_region) if region_per_region else None
        ),
        "relative_pose_residual": {
            "max_position_m": (max(residuals_position) if residuals_position else None),
            "max_orientation_rad": (
                max(residuals_orientation) if residuals_orientation else None
            ),
            "mean_position_m": (
                statistics.fmean(residuals_position) if residuals_position else None
            ),
        },
        "zero_escape_rate": (zero_escape / completed) if completed else 0.0,
        "solve_p95_target_s": SOLVE_P95_TARGET_S,
        "solve_p95_target_met": bool(
            solve_p95 is not None and solve_p95 < SOLVE_P95_TARGET_S
        ),
        "total_wall_time_s": time.perf_counter() - started,
    }
    # **验收口径**与**端到端口径**分列（口径不许混用）。
    report["acceptance_metrics"] = {
        "success_rate": report["success_rate"],
        "solve_p95_s": report["solve_p95_s"],
        "zero_escape_rate": report["zero_escape_rate"],
        "note": (
            "验收口径（R017）：离线成功率 ≥ 95% 且**求解阶段** P95 < 3 s；"
            "求解阶段 = 参数化 + GCS + 映射 + 重定时（**不含**区域生成）"
        ),
    }
    report["end_to_end_metrics"] = {
        "end_to_end_p50_s": report["end_to_end_duration_s"]["p50"],
        "end_to_end_p95_s": report["end_to_end_p95_s"],
        "note": "端到端口径：整条链路（含区域生成）——**单列**，不作为验收口径",
    }
    # 真机 / 物理指标：**硬件门控 SKIP**（无真机 ⇒ 逐条列出，退出码仍为 0）。
    report["unverified_metrics"] = list(UNVERIFIED_COORDINATED_METRICS)
    return report


def _print_report(report: Dict[str, Any]) -> None:
    """打印人类可读报告（验收口径 / 端到端口径**分行**）。"""
    solve = report["solve_duration_s"]
    end_to_end = report["end_to_end_duration_s"]
    residual = report["relative_pose_residual"]
    print(
        f"coordinated-stats: 采样 {report['samples_completed']}/"
        f"{report['samples_requested']} 组（seed={report['seed']}，"
        f"耗时 {report['total_wall_time_s']:.1f}s）"
    )
    if report["truncated"]:
        print("coordinated-stats: **因 --max-seconds 预算耗尽提前结束**（部分样本）")
    print(f"coordinated-stats: 成功率 = {report['success_rate'] * 100:.1f}%")
    print(
        "coordinated-stats: [验收口径] 求解阶段耗时 "
        f"P50={_fmt(solve['p50'], 6, 's')} P95={_fmt(solve['p95'], 6, 's')} "
        f"（目标 P95 < {SOLVE_P95_TARGET_S}s → "
        f"{'达标' if report['solve_p95_target_met'] else '**未达标**'}）"
    )
    print(
        "coordinated-stats: [验收口径] 零逃逸率 = "
        f"{report['zero_escape_rate'] * 100:.1f}%"
    )
    print(
        "coordinated-stats: [验收口径] 相对位姿残差 max 位置 = "
        f"{_fmt_sci(residual['max_position_m'])} m / 姿态 = "
        f"{_fmt_sci(residual['max_orientation_rad'])} rad"
    )
    print(
        "coordinated-stats: [端到端] 端到端耗时 "
        f"P50={_fmt(end_to_end['p50'], 3, 's')} "
        f"P95={_fmt(end_to_end['p95'], 3, 's')} "
        f"均值={_fmt(end_to_end['mean'], 3, 's')} "
        f"最大={_fmt(end_to_end['max'], 3, 's')}"
    )
    print(
        "coordinated-stats: [端到端] 区域生成单区域平均耗时 = "
        f"{_fmt(report['region_per_region_mean_s'], 3, 's')}"
    )
    if report["failure_breakdown"]:
        print(
            "coordinated-stats: 失败分类："
            + "，".join(
                f"{key} × {count}"
                for key, count in sorted(report["failure_breakdown"].items())
            )
        )
    print(
        "coordinated-stats: 说明：验收口径对应 R017「离线成功率 ≥ 95%、"
        "求解阶段 P95 < 3 s」；**求解阶段** = 参数化 + GCS + 映射 + 重定时，"
        "端到端**另列**（含区域生成）。"
    )
    print("coordinated-stats: 以下物理指标**未验证**（无真机 ⇒ 硬件门控 SKIP）：")
    for item in report["unverified_metrics"]:
        print(f"coordinated-stats:   - {item}")


def main(argv: Optional[Sequence[str]] = None) -> int:
    """脚本入口（返回退出码）。"""
    parser = argparse.ArgumentParser(
        description="协调规划指标：N 组确定性任务的成功率 / 求解阶段 P95 / 端到端 P95"
    )
    parser.add_argument("--samples", type=int, default=100, help="采样组数（默认 100）")
    parser.add_argument("--seed", type=int, default=0, help="随机种子（默认 0）")
    parser.add_argument(
        "--scenario", type=Path, default=DEFAULT_SCENARIO, help="双臂场景真相源 YAML"
    )
    parser.add_argument(
        "--max-seconds",
        type=float,
        default=None,
        help="总时间预算（s）；超出即停并标记结果为部分样本",
    )
    parser.add_argument(
        "--x-xyz",
        type=float,
        nargs=3,
        metavar=("X", "Y", "Z"),
        default=None,
        help="相对位姿 X_LR 的平移（m；默认取占位值 0.2 0 0）",
    )
    parser.add_argument(
        "--x-rpy",
        type=float,
        nargs=3,
        metavar=("ROLL", "PITCH", "YAW"),
        default=None,
        help="相对位姿 X_LR 的姿态（rad；默认 0 0 0）",
    )
    parser.add_argument(
        "--num-regions", type=int, default=3, help="最小坐标凸区域数（默认 3）"
    )
    parser.add_argument(
        "--path-step",
        type=float,
        default=0.005,
        help="leader 路径推进步长（rad，默认 0.005）",
    )
    parser.add_argument(
        "--psi-grid", type=int, default=33, help="follower ψ 扫描网格数（默认 33）"
    )
    parser.add_argument("--json", action="store_true", help="以 JSON 输出")
    args = parser.parse_args(list(argv) if argv is not None else None)

    if args.samples <= 0:
        print("coordinated_stats: --samples 必须为正整数", file=sys.stderr)
        return 1
    try:
        report = run_stats(
            samples=args.samples,
            seed=args.seed,
            scenario_path=args.scenario,
            max_seconds=args.max_seconds,
            x_xyz=args.x_xyz,
            x_rpy=args.x_rpy,
            num_regions=args.num_regions,
            path_step=args.path_step,
            psi_grid=args.psi_grid,
        )
    except ImportError as exc:
        print(f"coordinated_stats: 环境不可用（需要 Drake）：{exc}", file=sys.stderr)
        return 1
    if args.json:
        print(json.dumps(report, ensure_ascii=False, indent=2, default=str))
    else:
        _print_report(report)
    if report["samples_completed"] == 0:
        print("coordinated_stats: 一组都没跑成", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
