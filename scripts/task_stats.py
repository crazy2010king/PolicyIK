#!/usr/bin/env python3
"""任务原语级指标脚本（S06-T07）。

用途
----
跑 **N 组确定性的整条任务原语链**（默认任务 = ``tasks/pick_place.yaml``），
输出：

* **逐原语成功率**（每个原语各自的成功率）；
* **逐原语误差分位数**（位置误差 P95 / 姿态误差 P95）；
* **逐原语零逃逸率**；
* **整体成功率**与**整体零逃逸率**。

口径纪律（与 ``scripts/coordinated_stats.py`` 同构）
----------------------------------------------------
**验收口径**与**端到端口径分行打印**（并写明哪一条对应验收）：

* **验收口径** = **求解阶段**的逐原语指标（R011：每个原语都有可测的
  成功率与误差门槛）；
* **端到端口径** = 含**场景装配**与**整条原语链**的开销，**单列**，
  **不混入**验收口径。

**真机 / 物理指标一律硬件门控 SKIP**：真机抓放成功率 / 夹持误差 /
交接保持漂移 / 感知延迟 / 夹爪力控与夹空检测 / 长稳节拍 —— 本环境
**无真机** ⇒ 逐条列入 ``unverified_metrics``，**退出码仍为 0**，
**绝不用合成数据冒充物理达标**。

唯一真相源
----------
* 未验证物理指标清单：:mod:`rm75_drake.tasks.metrics`（canonical），
  本脚本与 :mod:`rm75_drake.cli` 只**再导出**，**禁止复制第二份**；
* 跑链实现：:func:`rm75_drake.cli.run_task_chain`（与 ``task`` 子命令共用）。

用法
----
::

    python3 scripts/task_stats.py                      # 默认 50 组
    python3 scripts/task_stats.py --samples 20         # 调整规模
    python3 scripts/task_stats.py --json               # 机器可读输出
    python3 scripts/task_stats.py --task tasks/handover.yaml
        # ⇒ 退出码 2（reason=task_single_arm_only：task 仅支持单臂场景，见
        #   runbooks/s06_task_primitives.md §7）
    python3 scripts/task_stats.py --max-seconds 60      # 时间预算（过载保护）

退出码
------
``0``  采样完成且**逐原语门槛达标**（真机物理指标硬件门控 SKIP）；
``1``  业务失败（门槛未达标 / 一组都没跑成 / 环境不可用）；
``2``  用法错误（任务文件不存在 / 参数非法）或**命令未支持**
      （多臂场景 ⇒ reason=task_single_arm_only）。
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict, Optional, Sequence

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = REPO_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

#: 默认任务文件（部署实例数据；仓库根 ``tasks/``）。
DEFAULT_TASK = REPO_ROOT / "tasks" / "pick_place.yaml"

#: 默认场景真相源（任务文件里没写 ``scenario`` 时的兜底）。
DEFAULT_SCENARIO = REPO_ROOT / "scenarios" / "single_arm.yaml"

#: 默认采样组数。
DEFAULT_SAMPLES = 50

#: 初始位形抖动幅度（rad）——让重复采样不是**完全**同一条轨迹，
#: 但幅度极小（远小于任何工程意义），保证成功率口径稳定。
DEFAULT_JITTER_RAD = 0.005


def _fmt(value: Optional[float], digits: int = 3, unit: str = "") -> str:
    """格式化一个可能为 ``None`` 的数值。"""
    if value is None:
        return "n/a"
    return f"{value:.{digits}f}{unit}"


def _fmt_sci(value: Optional[float]) -> str:
    """把误差这类**极小量**格式化成科学计数法（``None`` → ``n/a``）。"""
    if value is None:
        return "n/a"
    return f"{value:.3e}"


def run_stats(
    *,
    samples: int = DEFAULT_SAMPLES,
    seed: int = 0,
    task_path: Path = DEFAULT_TASK,
    scenario_path: Optional[Path] = None,
    jitter_rad: float = DEFAULT_JITTER_RAD,
    max_seconds: Optional[float] = None,
) -> Dict[str, Any]:
    """跑指标采样并返回结果字典（**如实**记录实测值）。

    参数
    ----
    samples:
        采样组数（整条链各跑一次）。
    seed:
        随机种子（保证**可复现**）。
    task_path:
        任务文件路径。
    scenario_path:
        场景真相源；``None`` → 取任务文件里的 ``scenario`` 字段。
    jitter_rad:
        初始位形抖动幅度（rad）。
    max_seconds:
        **时间预算**（s）；超出即停（部分样本，``truncated=True``）。
    """
    from rm75_drake.cli import run_task_chain

    outcome = run_task_chain(
        Path(task_path),
        scenario_path=Path(scenario_path) if scenario_path is not None else None,
        samples=int(samples),
        seed=int(seed),
        jitter_rad=float(jitter_rad),
        max_seconds=max_seconds,
    )
    report = outcome["report_object"]
    metrics: Dict[str, Any] = dict(outcome["metrics"])
    metrics.update(
        {
            "task": outcome["task"],
            "task_name": outcome["task_name"],
            "scenario": outcome["scenario"],
            "samples_requested": int(samples),
            "samples_completed": int(outcome["samples_completed"]),
            "truncated": bool(outcome.get("truncated", False)),
            "seed": int(seed),
            "jitter_rad": float(jitter_rad),
            "scene_assembly_s": outcome["scene_assembly_s"],
            "total_wall_time_s": outcome["wall_time_s"],
            "failure_breakdown": _failure_breakdown(report),
            "hardware_gate": {
                "status": "SKIP",
                "reason": "offline_only_no_hardware",
                "note": (
                    "无真机 ⇒ 真机物理指标硬件门控 SKIP（退出码仍为 0），"
                    "逐条列入 unverified_metrics。"
                ),
            },
        }
    )
    # **验收口径**与**端到端口径**分列（口径不许混用）。
    metrics["acceptance_metrics"] = dict(report.acceptance_metrics)
    metrics["end_to_end_metrics"] = dict(report.end_to_end_metrics)
    # 真机 / 物理指标：**硬件门控 SKIP**（无真机 ⇒ 逐条列出，退出码仍为 0）。
    metrics["unverified_metrics"] = list(report.unverified_metrics)
    metrics["thresholds_met"] = bool(report.thresholds_met)
    metrics["threshold_failures"] = list(report.threshold_failures)
    return metrics


def _failure_breakdown(report: Any) -> Dict[str, int]:
    """汇总逐原语的失败分类（``"{stage}:{reason}"`` → 次数）。"""
    merged: Dict[str, int] = {}
    for metric in report.primitives:
        for key, count in dict(metric.failure_breakdown).items():
            merged[key] = merged.get(key, 0) + int(count)
    return merged


def _print_report(report: Dict[str, Any]) -> None:
    """打印人类可读报告（验收口径 / 端到端口径**分行**）。"""
    print(
        f"task-stats: 采样 {report['samples_completed']}/{report['samples_requested']} 组"
        f"（seed={report['seed']}，抖动 {report['jitter_rad']} rad，"
        f"耗时 {report['total_wall_time_s']:.1f}s）"
    )
    if report.get("truncated"):
        print("task-stats: **因 --max-seconds 预算耗尽提前结束**（部分样本）")
    print(f"task-stats: 任务 = {report['task']}（场景：{report['scenario']}）")
    print(f"task-stats: 整体成功率 = {report['success_rate'] * 100:.1f}%")
    print(f"task-stats: 整体零逃逸率 = {report['zero_escape_rate'] * 100:.1f}%")
    print("task-stats: [验收口径] 逐原语指标（求解阶段）：")
    for metric in report.get("primitives", []):
        print(
            f"task-stats:   - {metric['name']}：成功率 "
            f"{metric['success_rate'] * 100:.1f}%，位置误差 P95="
            f"{_fmt_sci(metric['position_error_p95_m'])} m，姿态误差 P95="
            f"{_fmt_sci(metric['orientation_error_p95_rad'])} rad，零逃逸率 "
            f"{metric['zero_escape_rate'] * 100:.1f}%"
            + ("（达标）" if metric["thresholds_met"] else "（**未达标**）")
        )
    end_to_end = report.get("end_to_end_metrics") or {}
    per_sample = end_to_end.get("per_sample_duration_s") or {}
    print(
        "task-stats: [端到端] 场景装配耗时 "
        f"{_fmt(end_to_end.get('scene_assembly_s'), 3, 's')}，"
        "单样本轨迹时长（仿真时间）"
        f"P50={_fmt(per_sample.get('p50'), 3, 's')} "
        f"P95={_fmt(per_sample.get('p95'), 3, 's')}"
        f"，总耗时 {_fmt(report['total_wall_time_s'], 3, 's')}"
        "（**单列**，不作为验收口径）"
    )
    breakdown = report.get("failure_breakdown") or {}
    if breakdown:
        print(
            "task-stats: 失败分类："
            + "，".join(f"{key} × {count}" for key, count in sorted(breakdown.items()))
        )
    if not report.get("thresholds_met", True):
        print("task-stats: **门槛未达标**：")
        for item in report.get("threshold_failures", []):
            print(f"task-stats:   - {item}")
    print(
        "task-stats: 说明：验收口径对应 R011「每个原语有可测的成功率与误差门槛」；"
        "端到端口径含场景装配与整链开销，**另列**，不混入验收口径。"
    )
    print("task-stats: 以下物理指标**未验证**（无真机 ⇒ 硬件门控 SKIP）：")
    for item in report.get("unverified_metrics", []):
        print(f"task-stats:   - {item}")


def main(argv: Optional[Sequence[str]] = None) -> int:
    """脚本入口（返回退出码）。"""
    parser = argparse.ArgumentParser(
        description="任务原语级指标：N 组整链的成功率 / 误差分位数 / 零逃逸率"
    )
    parser.add_argument(
        "--samples", type=int, default=DEFAULT_SAMPLES, help=f"采样组数（默认 {DEFAULT_SAMPLES}）"
    )
    parser.add_argument("--seed", type=int, default=0, help="随机种子（默认 0）")
    parser.add_argument(
        "--task", type=Path, default=DEFAULT_TASK, help="任务文件路径（默认 tasks/pick_place.yaml）"
    )
    parser.add_argument(
        "--scenario",
        type=Path,
        default=None,
        help="场景真相源 YAML（默认取任务文件里的 scenario 字段）",
    )
    parser.add_argument(
        "--jitter",
        type=float,
        default=DEFAULT_JITTER_RAD,
        help=f"初始位形抖动幅度（rad，默认 {DEFAULT_JITTER_RAD}；0 = 完全确定）",
    )
    parser.add_argument(
        "--max-seconds",
        type=float,
        default=None,
        help="总时间预算（s）；超出即停并标记结果为部分样本（过载保护）",
    )
    parser.add_argument("--json", action="store_true", help="以 JSON 输出")
    args = parser.parse_args(list(argv) if argv is not None else None)

    if args.samples <= 0:
        print("task_stats: --samples 必须为正整数", file=sys.stderr)
        return 2
    if not Path(args.task).is_file():
        print(f"task_stats: 任务文件不存在：{args.task}", file=sys.stderr)
        return 2
    if not float(args.jitter) >= 0.0:
        print("task_stats: --jitter 必须是非负有限数", file=sys.stderr)
        return 2

    from rm75_drake.cli import REASON_TASK_SINGLE_ARM_ONLY, TaskSingleArmOnlyError

    try:
        report = run_stats(
            samples=args.samples,
            seed=args.seed,
            task_path=Path(args.task),
            scenario_path=Path(args.scenario) if args.scenario else None,
            jitter_rad=float(args.jitter),
            max_seconds=args.max_seconds,
        )
    except ImportError as exc:
        print(f"task_stats: 环境不可用（需要 Drake）：{exc}", file=sys.stderr)
        return 1
    except FileNotFoundError as exc:
        print(f"task_stats: 文件不存在：{exc}", file=sys.stderr)
        return 2
    except TaskSingleArmOnlyError as exc:
        # 能力边界（多臂场景）⇒ 退出码 2（用法 / 未支持），不是业务失败。
        print(
            f"task_stats: 未支持（{REASON_TASK_SINGLE_ARM_ONLY}）：{exc}",
            file=sys.stderr,
        )
        return 2

    if args.json:
        print(json.dumps(report, ensure_ascii=False, indent=2, default=str))
    else:
        _print_report(report)

    if report["samples_completed"] == 0:
        print("task_stats: 一组都没跑成", file=sys.stderr)
        return 1
    if not report.get("thresholds_met", True):
        print("task_stats: 逐原语门槛未达标（业务失败）", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
