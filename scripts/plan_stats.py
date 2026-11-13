#!/usr/bin/env python3
"""规划指标脚本（S02-T08）。

用途
----
跑 **N 组随机的「起点构型 → 目标关节」** 规划，输出：

* **成功率**（成功规划的比例）；
* **P50 / P95 规划耗时**（端到端：IRIS + GCS + 重定时 + 碰撞复检）；
* **GCS 阶段 P50 / P95 耗时**（目标：单臂 GCS **P95 < 1 s**）；
* **IRIS 单区域平均耗时**（区域阶段耗时 ÷ 区域数）。

诚实原则
--------
**目标没达到就如实打印实测值**（并标记「未达标」），不做任何美化、不截断、
不换口径。IRIS 是本机最慢的阶段（本机单区域 ~5–8 s），因此端到端 P95 通常
**远大于** 1 s；GCS 阶段的 P95 才是切片验收里那条「单臂 GCS P95 < 1 s」
所指的指标——脚本两者都打印，**并注明各自口径**。

用法
----
::

    python3 scripts/plan_stats.py                    # 默认 100 组
    python3 scripts/plan_stats.py --samples 20       # 调整规模
    python3 scripts/plan_stats.py --json             # 机器可读输出
    python3 scripts/plan_stats.py --max-seconds 600  # 时间预算（超时即停）

退出码
------
``0``  至少完成一组采样并打印了实测指标；
``1``  一组都没跑成（环境不可用 / 场景装不起来）。
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

#: 默认场景真相源。
DEFAULT_SCENARIO = REPO_ROOT / "scenarios" / "single_arm.yaml"

#: 起点名义姿态（抬肘，碰撞自由；与 IK 模块的 NOMINAL_POSTURE 一致）。
NOMINAL_POSTURE = (0.0, -0.6, 0.0, 0.9, 0.0, 0.5, 0.0)

#: 起点扰动幅度（rad）。
START_JITTER_RAD = 0.25

#: 目标相对起点的扰动幅度（rad）。
TARGET_JITTER_RAD = 0.2

#: GCS 阶段 P95 目标（s）。
GCS_P95_TARGET_S = 1.0


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


def _sample_configurations(
    rng: random.Random, limits: Any
) -> List[List[float]]:
    """随机采样起点 / 目标关节角（落在软件软限位内）。"""
    lower = list(limits.lower)
    upper = list(limits.upper)
    start: List[float] = []
    target: List[float] = []
    for index in range(len(NOMINAL_POSTURE)):
        base = NOMINAL_POSTURE[index]
        start_value = base + rng.uniform(-START_JITTER_RAD, START_JITTER_RAD)
        start_value = min(max(start_value, lower[index]), upper[index])
        target_value = start_value + rng.uniform(-TARGET_JITTER_RAD, TARGET_JITTER_RAD)
        target_value = min(max(target_value, lower[index]), upper[index])
        start.append(start_value)
        target.append(target_value)
    return [start, target]


def run_stats(
    *,
    samples: int = 100,
    seed: int = 0,
    scenario_path: Path = DEFAULT_SCENARIO,
    max_seconds: Optional[float] = None,
    region_spacing: float = 0.5,
    iris_samples: Optional[int] = 3,
) -> Dict[str, Any]:
    """跑指标采样并返回结果字典（**如实**记录实测值）。"""
    from rm75_drake.planner import Planner, PlanOptions
    from rm75_drake.planning.scene import build_scene
    from rm75_drake.planning.trajectory import software_limits
    from rm75_drake.scenario import load_scenario

    scenario = load_scenario(scenario_path)
    scene = build_scene(scenario)
    limits = software_limits()
    rng = random.Random(seed)

    region_options: Dict[str, Any] = {}
    if iris_samples is not None:
        region_options["num_collision_infeasible_samples"] = int(iris_samples)

    durations: List[float] = []
    gcs_durations: List[float] = []
    iris_per_region: List[float] = []
    successes = 0
    failures: Dict[str, int] = {}
    truncated = False
    started = time.perf_counter()

    for index in range(samples):
        if max_seconds is not None and (time.perf_counter() - started) > max_seconds:
            truncated = True
            break
        start_q, target_q = _sample_configurations(rng, limits)
        planner = Planner(
            scenario,
            scene=scene,
            options=PlanOptions(
                start_configuration=tuple(start_q),
                region_spacing=region_spacing,
                region_options=region_options,
            ),
        )
        plan_started = time.perf_counter()
        result = planner.plan_joint_target(target_q)
        duration = time.perf_counter() - plan_started
        durations.append(duration)
        diagnostics = result.diagnostics
        gcs = diagnostics.get("gcs") or {}
        if isinstance(gcs, dict) and gcs.get("duration_s") is not None:
            gcs_durations.append(float(gcs["duration_s"]))
        regions = diagnostics.get("regions") or {}
        if isinstance(regions, dict):
            num_regions = int(regions.get("num_regions", 0) or 0)
            if num_regions > 0 and regions.get("duration_s") is not None:
                iris_per_region.append(float(regions["duration_s"]) / num_regions)
        if result.success:
            successes += 1
        else:
            key = f"{result.stage or 'unknown'}:{result.reason or 'unknown'}"
            failures[key] = failures.get(key, 0) + 1

    completed = len(durations)
    report: Dict[str, Any] = {
        "samples_requested": samples,
        "samples_completed": completed,
        "truncated": truncated,
        "seed": seed,
        "scenario": str(scenario_path),
        "success_rate": (successes / completed) if completed else 0.0,
        "success_count": successes,
        "failure_breakdown": failures,
        "plan_duration_s": {
            "p50": _percentile(durations, 0.50),
            "p95": _percentile(durations, 0.95),
            "mean": (statistics.fmean(durations) if durations else None),
            "max": (max(durations) if durations else None),
        },
        "gcs_duration_s": {
            "p50": _percentile(gcs_durations, 0.50),
            "p95": _percentile(gcs_durations, 0.95),
        },
        "gcs_p95_target_s": GCS_P95_TARGET_S,
        "iris_per_region_mean_s": (
            statistics.fmean(iris_per_region) if iris_per_region else None
        ),
        "total_wall_time_s": time.perf_counter() - started,
    }
    gcs_p95 = report["gcs_duration_s"]["p95"]
    report["gcs_p95_target_met"] = bool(
        gcs_p95 is not None and gcs_p95 < GCS_P95_TARGET_S
    )
    return report


def _print_report(report: Dict[str, Any]) -> None:
    """打印人类可读报告（实测值 + 是否达标）。"""
    plan = report["plan_duration_s"]
    gcs = report["gcs_duration_s"]
    print(
        f"plan-stats: 采样 {report['samples_completed']}/{report['samples_requested']} 组"
        f"（seed={report['seed']}，耗时 {report['total_wall_time_s']:.1f}s）"
    )
    if report["truncated"]:
        print("plan-stats: **因 --max-seconds 预算耗尽提前结束**（结果为部分样本）")
    print(f"plan-stats: 成功率 = {report['success_rate'] * 100:.1f}%")
    print(
        "plan-stats: 端到端规划耗时 "
        f"P50={_fmt(plan['p50'], 3, 's')} P95={_fmt(plan['p95'], 3, 's')} "
        f"均值={_fmt(plan['mean'], 3, 's')} 最大={_fmt(plan['max'], 3, 's')}"
    )
    print(
        "plan-stats: GCS 阶段耗时 "
        f"P50={_fmt(gcs['p50'], 6, 's')} P95={_fmt(gcs['p95'], 6, 's')}"
        f"（目标 P95 < {GCS_P95_TARGET_S}s → "
        f"{'达标' if report['gcs_p95_target_met'] else '**未达标**'}）"
    )
    print(
        "plan-stats: IRIS 单区域平均耗时 = "
        f"{_fmt(report['iris_per_region_mean_s'], 3, 's')}"
    )
    if report["failure_breakdown"]:
        print("plan-stats: 失败分类：" + "，".join(
            f"{key} × {count}" for key, count in sorted(report["failure_breakdown"].items())
        ))
    print(
        "plan-stats: 说明：端到端耗时含 IRIS（最慢阶段），"
        "验收口径「单臂 GCS P95 < 1 s」对应上面的 **GCS 阶段** P95。"
    )


def main(argv: Optional[Sequence[str]] = None) -> int:
    """脚本入口（返回退出码）。"""
    parser = argparse.ArgumentParser(
        description="规划指标：N 组随机起终点的成功率 / P50 / P95 耗时"
    )
    parser.add_argument("--samples", type=int, default=100, help="采样组数（默认 100）")
    parser.add_argument("--seed", type=int, default=0, help="随机种子（默认 0）")
    parser.add_argument(
        "--scenario", type=Path, default=DEFAULT_SCENARIO, help="场景真相源 YAML"
    )
    parser.add_argument(
        "--max-seconds",
        type=float,
        default=None,
        help="总时间预算（s）；超出即停并标记结果为部分样本",
    )
    parser.add_argument(
        "--region-spacing",
        type=float,
        default=0.5,
        help="IRIS 区域链种子间距（rad，默认 0.5；越大 → 区域越少、越快）",
    )
    parser.add_argument(
        "--iris-samples",
        type=int,
        default=3,
        help="IRIS 碰撞不可行采样数（默认 3；调小 → 更快、认证更弱）",
    )
    parser.add_argument("--json", action="store_true", help="以 JSON 输出")
    args = parser.parse_args(list(argv) if argv is not None else None)

    if args.samples <= 0:
        print("plan_stats: --samples 必须为正整数", file=sys.stderr)
        return 1
    try:
        report = run_stats(
            samples=args.samples,
            seed=args.seed,
            scenario_path=args.scenario,
            max_seconds=args.max_seconds,
            region_spacing=args.region_spacing,
            iris_samples=args.iris_samples,
        )
    except ImportError as exc:
        print(f"plan_stats: 环境不可用（需要 Drake）：{exc}", file=sys.stderr)
        return 1
    if args.json:
        print(json.dumps(report, ensure_ascii=False, indent=2))
    else:
        _print_report(report)
    if report["samples_completed"] == 0:
        print("plan_stats: 一组都没跑成", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
