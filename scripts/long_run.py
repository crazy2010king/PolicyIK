#!/usr/bin/env python3
"""**任务级长稳载体**（S07-T05）：8 h 连续 pick-place 长稳的运行载体。

为什么需要它
------------
验收项「8 h 长稳达标」此前**没有载体**：
``scripts/soak_test.py`` 只做 100 Hz **保持位姿**（30 min），
既不是**任务级循环**，也不是 8 h；仓库里没有任何连续 pick-place 的运行载体。

本脚本提供的就是那个载体：

* **任务级循环**：每轮完整跑一遍 S06 的任务原语链
  （:func:`rm75_drake.tasks.solver.solve_task`），
  **不重写求解器**（复用同一条链路 ⇒ 口径唯一）；
* **8 h 时长**：默认 ``--duration-s 28800``（8 h）；
* **判定**：任务级成功率 = 成功轮数 / 总轮数；门槛默认 0.95；
  **异常停机** = 出现未捕获异常，或求解硬失败（stage ∈ {ik, path, collision, …}）。

诚实边界（**本机无真机**）
------------------------
本环境**没有真机**，因此本任务交付的是：

* 载体（harness）+ 离线演练（``--dry-run``）+ **硬件门控 SKIP 契约** +
  **未验证指标清单**；

**绝不用合成数据冒充「8 h 达标」**（沿用 K001 的硬件门控纪律）：

* 真机不可达 ⇒ 输出 ``SKIP(reason=hardware_unreachable)`` +
  **逐条**列出未验证指标，**退出码仍为 0**
  （既不伪装成功，也不伪装失败）；
* 离线演练（``--dry-run``）⇒ JSON 里 ``offline=true``、
  ``physical_verified=false``、``thresholds_met`` **恒为 false**
  （物理长稳未验证 ⇒ 不得报达标），输出必须标注「**物理长稳未验证**」。

载体当前的覆盖范围（**已知缺口，见任务小结**）
------------------------------------------------
* 每轮**真实求解**整条原语链（解析 IK → 插值 → 碰撞复检 → 重定时），
  并把夹爪指令下发到传输层（离线替身 / 真机 SDK）；
* **尚未**包含「把关节轨迹下发到真机并回读状态」的**运动执行闭环**
  （真机路径由 :mod:`rm75_drake.device.exec_loop` 承担安全七闸，
  不在本载体里绕开安全闸直发指令）。
  ⇒ 真机 8 h 长稳仍然是**未验证指标**（见
  :data:`rm75_drake.tasks.metrics.UNVERIFIED_TASK_METRICS`）。

用法
----
::

    # 离线演练（不代表真机结果；退出码 0）
    PYTHONPATH=src python3 scripts/long_run.py --dry-run --duration-s 5 --json

    # 真机长稳（真机不可达时 SKIP，退出码 0）
    PYTHONPATH=src python3 scripts/long_run.py --duration-s 28800 --json

退出码
------
``0``  通过 / 真机不可达时的 SKIP / 离线演练（演练结果**不代表**真机达标，
       达标与否只看 ``thresholds_met`` 与 ``threshold_failures``）；
``1``  真机路径下门槛未达标（成功率不足 / 异常停机）；
``2``  用法错误（参数非法 / 任务文件或场景文件不存在 / 任务 schema 非法）。
"""

from __future__ import annotations

import argparse
import json
import math
import random
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = REPO_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from rm75_drake.device.config import DeviceConfigError, load_device_config  # noqa: E402
from rm75_drake.device.hardware_gate import (  # noqa: E402
    SKIP_REASON_HARDWARE_UNREACHABLE,
    format_skip,
    probe_hardware,
)
from rm75_drake.tasks.metrics import (  # noqa: E402
    UNVERIFIED_TASK_METRICS,
    UNVERIFIED_TASK_METRIC_LONG_RUN_8H,
)

EXIT_OK = 0
EXIT_FAILURE = 1
EXIT_USAGE = 2

#: 默认任务文件（仓库根 ``tasks/``）。
DEFAULT_TASK = REPO_ROOT / "tasks" / "pick_place.yaml"

#: 默认场景真相源（仓库根 ``scenarios/``）。
DEFAULT_SCENARIO = REPO_ROOT / "scenarios" / "single_arm.yaml"

#: 默认长稳时长（s）——8 h。
DEFAULT_DURATION_S = 28800.0

#: 默认任务级成功率门槛。
DEFAULT_MIN_SUCCESS_RATE = 0.95

#: 循环数硬上限（防止「单轮周期时长为 0」导致的死循环；远大于 8 h 的真实轮数）。
DEFAULT_MAX_CYCLES_CAP = 100000

#: 离线演练的默认时长（s）——**不代表**真机长稳。
DEFAULT_DRY_RUN_DURATION_S = 5.0

#: 离线演练下必须出现在输出里的标注（**物理长稳未验证**）。
OFFLINE_NOTICE = "物理长稳未验证"


class UsageError(RuntimeError):
    """用法错误（⇒ 退出码 2）。"""


# ── 参数 ────────────────────────────────────────────────────────────────
def build_parser() -> argparse.ArgumentParser:
    """构造命令行解析器。"""
    parser = argparse.ArgumentParser(
        description=(
            "任务级长稳载体（8 h 连续 pick-place；真机不可达时 SKIP 并列出未验证指标）"
        )
    )
    parser.add_argument(
        "--task",
        default=str(DEFAULT_TASK),
        help=f"任务文件（YAML，默认：{DEFAULT_TASK}）",
    )
    parser.add_argument(
        "--scenario",
        default=str(DEFAULT_SCENARIO),
        help=f"场景真相源（默认：{DEFAULT_SCENARIO}）",
    )
    parser.add_argument(
        "--duration-s",
        type=float,
        default=DEFAULT_DURATION_S,
        help=(
            "长稳时长（s，默认：28800 = 8 h；"
            f"离线演练建议 {DEFAULT_DRY_RUN_DURATION_S}）"
        ),
    )
    parser.add_argument(
        "--max-cycles",
        type=int,
        default=None,
        help="最大循环轮数（可选上限；不给则由时长决定）",
    )
    parser.add_argument(
        "--min-success-rate",
        type=float,
        default=DEFAULT_MIN_SUCCESS_RATE,
        help=(
            "任务级成功率门槛（默认："
            f"{DEFAULT_MIN_SUCCESS_RATE}；离线演练下不用于报达标）"
        ),
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=0,
        help="随机种子（仅在启用抖动时生效；保证可复现）",
    )
    parser.add_argument(
        "--jitter-rad",
        type=float,
        default=0.0,
        help="初始位形抖动幅度（rad，默认 0 = 完全确定性）",
    )
    parser.add_argument(
        "--device-config",
        default=None,
        help="设备配置（真机路径的硬件门控探测用）",
    )
    parser.add_argument(
        "--connect-timeout",
        type=float,
        default=1.0,
        help="真机可达性探测的 TCP 超时（s，默认：1.0）",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="离线演练（虚拟时钟 + 替身传输层；**不代表真机长稳结果**）",
    )
    parser.add_argument("--json", action="store_true", help="输出单行 JSON")
    return parser


# ── 参数校验 ────────────────────────────────────────────────────────────
def _positive(value: float, name: str) -> float:
    """校验「正有限数」。"""
    number = float(value)
    if not math.isfinite(number) or number <= 0.0:
        raise UsageError(f"{name} 必须是正有限数，收到 {value!r}")
    return number


def _validate_args(args: argparse.Namespace) -> Dict[str, Any]:
    """校验参数并返回规范化后的运行配置。"""
    duration_s = _positive(args.duration_s, "--duration-s")
    min_success_rate = float(args.min_success_rate)
    if not math.isfinite(min_success_rate) or not 0.0 <= min_success_rate <= 1.0:
        raise UsageError(f"--min-success-rate 必须落在 [0, 1]，收到 {args.min_success_rate!r}")
    jitter_rad = float(args.jitter_rad)
    if not math.isfinite(jitter_rad) or jitter_rad < 0.0:
        raise UsageError(f"--jitter-rad 必须是非负有限数，收到 {args.jitter_rad!r}")
    max_cycles = args.max_cycles
    if max_cycles is not None:
        if isinstance(max_cycles, bool) or int(max_cycles) < 1:
            raise UsageError(f"--max-cycles 必须是正整数，收到 {max_cycles!r}")
        max_cycles = int(max_cycles)
    task_path = Path(str(args.task))
    if not task_path.is_file():
        raise UsageError(f"任务文件不存在：{task_path}")
    scenario_path = Path(str(args.scenario))
    if not scenario_path.is_file():
        raise UsageError(f"场景文件不存在：{scenario_path}")
    return {
        "task_path": task_path,
        "scenario_path": scenario_path,
        "duration_s": duration_s,
        "max_cycles": max_cycles,
        "min_success_rate": min_success_rate,
        "seed": int(args.seed),
        "jitter_rad": jitter_rad,
        "dry_run": bool(args.dry_run),
        "device_config_path": (
            Path(str(args.device_config)) if args.device_config else None
        ),
        "connect_timeout": _positive(args.connect_timeout, "--connect-timeout"),
    }


# ── 核心：任务级长稳循环 ───────────────────────────────────────────────
def run_long_run(
    *,
    task_path: Path,
    scenario_path: Path,
    duration_s: float = DEFAULT_DURATION_S,
    max_cycles: Optional[int] = None,
    min_success_rate: float = DEFAULT_MIN_SUCCESS_RATE,
    seed: int = 0,
    jitter_rad: float = 0.0,
    dry_run: bool = True,
    device_config: Any = None,
) -> Dict[str, Any]:
    """跑**任务级**长稳循环，返回可 JSON 序列化的结果字典。

    参数
    ----
    task_path:
        任务文件（YAML）。
    scenario_path:
        场景真相源（YAML）。
    duration_s:
        长稳时长预算（s）。
    max_cycles:
        循环轮数上限（``None`` ⇒ 由时长决定）。
    min_success_rate:
        任务级成功率门槛。
    seed / jitter_rad:
        初始位形抖动（默认 0 ⇒ 完全确定性、可复现）。
    dry_run:
        ``True`` ⇒ 离线演练（替身传输层 + 虚拟时钟）。
    device_config:
        真机路径用的设备配置（``None`` ⇒ 真机路径不可用）。

    返回
    ----
    dict：见模块文档的 JSON 字段清单。

    异常
    ----
    :class:`UsageError`
        任务文件 / 场景文件不存在，或任务 schema 非法。
    """
    from rm75_drake.planning.scene import build_scene
    from rm75_drake.scenario import load_scenario
    from rm75_drake.tasks.schema import load_task_file
    from rm75_drake.tasks.solver import SolverOptions, solve_task

    try:
        task = load_task_file(task_path)
        scenario = load_scenario(scenario_path)
    except FileNotFoundError as exc:
        raise UsageError(str(exc)) from exc
    except Exception as exc:  # TaskError / ScenarioError
        raise UsageError(f"任务或场景文件非法：{exc}") from exc

    scene = build_scene(scenario)

    transport = None
    if dry_run:
        from rm75_drake.device.transport import ScriptedTransport

        transport = ScriptedTransport()
    else:
        from rm75_drake.device.transport import SdkTransport

        if device_config is None:
            raise UsageError("真机路径缺少设备配置")
        transport = SdkTransport(device_config)
    transport.open()

    cap = int(max_cycles) if max_cycles else DEFAULT_MAX_CYCLES_CAP
    rng = random.Random(int(seed))

    cycles = 0
    success_count = 0
    failure_count = 0
    exceptions = 0
    aborted = False
    aborted_reason = ""
    elapsed_s = 0.0
    failures: List[Dict[str, Any]] = []
    breakdown: Dict[str, int] = {}
    started = time.perf_counter()
    real_started = time.monotonic()

    try:
        while True:
            if cycles >= cap:
                aborted_reason = f"达到最大循环数上限 {cap}"
                break
            # 至少跑一轮；随后按「已用时长 ≥ 预算」停。
            if cycles > 0 and elapsed_s >= float(duration_s):
                break

            options = SolverOptions()
            if float(jitter_rad) > 0.0:
                options = SolverOptions(
                    initial_q=tuple(
                        rng.uniform(-float(jitter_rad), float(jitter_rad))
                        for _ in range(7)
                    )
                )
            cycle_started = time.monotonic()
            try:
                result = solve_task(task, scene=scene, transport=transport, options=options)
            except Exception as exc:  # 未捕获异常 ⇒ 异常停机
                exceptions += 1
                aborted = True
                aborted_reason = f"未捕获异常：{type(exc).__name__}: {exc}"
                break

            cycles += 1
            if dry_run:
                # 离线演练：虚拟时钟按**求解口径时长**推进（不真实等待）。
                elapsed_s += float(result.duration_s)
            else:
                # 真机长稳：按**真实时钟**推进并配速（8 h 必须是真实 8 h）。
                elapsed_s = time.monotonic() - real_started
                remaining = float(result.duration_s) - (time.monotonic() - cycle_started)
                if remaining > 0.0:
                    time.sleep(remaining)
            if result.success:
                success_count += 1
                continue
            failure_count += 1
            key = f"{result.stage}:{result.reason}"
            breakdown[key] = breakdown.get(key, 0) + 1
            failures.append(
                {
                    "cycle": cycles,
                    "stage": result.stage,
                    "reason": result.reason,
                    "message": result.message,
                }
            )
            aborted = True
            aborted_reason = f"求解硬失败停机（stage={result.stage}, reason={result.reason}）"
            break
    finally:
        try:
            transport.close()
        except Exception:  # 关闭失败不影响已采集的统计
            pass

    wall_time_s = time.perf_counter() - started
    success_rate = (success_count / cycles) if cycles else None

    threshold_failures: List[str] = []
    if success_rate is None:
        threshold_failures.append("未跑任何循环：无成功率可判")
    elif success_rate < float(min_success_rate):
        threshold_failures.append(
            f"任务级成功率 {success_rate:.4f} < 门槛 {float(min_success_rate):.4f}"
        )
    if exceptions:
        threshold_failures.append(f"出现未捕获异常 {exceptions} 次")
    if aborted and failure_count:
        threshold_failures.append(f"异常停机：{aborted_reason}")

    # 离线演练 ⇒ 物理长稳未验证 ⇒ **恒不报达标**（不得用合成数据冒充）。
    physical_verified = bool(not dry_run and not aborted and not threshold_failures)
    thresholds_met = bool(physical_verified)

    return {
        "command": "long_run",
        "status": "ok" if not threshold_failures else "failed",
        "mode": "dry_run" if dry_run else "hardware",
        "offline": bool(dry_run),
        "physical_verified": physical_verified,
        "task": str(task_path),
        "task_name": task.name,
        "scenario": str(scenario_path),
        "duration_s": float(duration_s),
        "elapsed_s": elapsed_s,
        "cycles": cycles,
        "success_count": success_count,
        "failure_count": failure_count,
        "success_rate": success_rate,
        "exceptions": exceptions,
        "aborted": aborted,
        "aborted_reason": aborted_reason,
        "failure_breakdown": breakdown,
        "failures": failures,
        "seed": int(seed),
        "jitter_rad": float(jitter_rad),
        "max_cycles": max_cycles,
        "wall_time_s": wall_time_s,
        "thresholds": {
            "min_success_rate": float(min_success_rate),
            "max_exceptions": 0,
        },
        "thresholds_met": thresholds_met,
        "threshold_failures": threshold_failures,
        "unverified_metrics": list(UNVERIFIED_TASK_METRICS),
        "long_run_metric": UNVERIFIED_TASK_METRIC_LONG_RUN_8H,
        "notes": (
            [f"离线演练：{OFFLINE_NOTICE}（不得当作真机长稳达标）"]
            if dry_run
            else []
        ),
    }


# ── 输出 ────────────────────────────────────────────────────────────────
def _emit_human(payload: Dict[str, Any]) -> None:
    """打印人类可读摘要。"""
    mode = "离线演练" if payload["offline"] else "真机"
    print(f"[long_run] 模式：{mode}")
    print(
        f"[long_run] 任务：{payload['task_name']}"
        f"（{payload['task']}）"
    )
    print(
        f"[long_run] 时长预算：{payload['duration_s']} s，"
        f"已用（求解口径）：{payload['elapsed_s']:.3f} s，"
        f"实际耗时：{payload['wall_time_s']:.3f} s"
    )
    print(
        f"[long_run] 循环：{payload['cycles']} 轮，"
        f"成功 {payload['success_count']}，失败 {payload['failure_count']}，"
        f"异常 {payload['exceptions']}"
    )
    rate = payload["success_rate"]
    print(
        "[long_run] 任务级成功率："
        + ("n/a（未跑任何循环）" if rate is None else f"{rate:.4f}")
        + f"（门槛 {payload['thresholds']['min_success_rate']:.4f}）"
    )
    if payload["aborted"]:
        print(f"[long_run] 异常停机：{payload['aborted_reason']}")
    for item in payload["threshold_failures"]:
        print(f"[long_run] 门槛未达标：{item}")
    if not payload["offline"]:
        print("[long_run] 物理长稳：已验证" if payload["physical_verified"] else "[long_run] 物理长稳：**未验证**")
    else:
        print(f"[long_run] 物理长稳：**{OFFLINE_NOTICE}**（离线演练不代表真机结果）")
    print("以下真机指标**未验证**（不得当作通过）：")
    for item in payload["unverified_metrics"]:
        print(f"  - {item}")


def _emit_skip(probe: Any, *, as_json: bool) -> int:
    """真机不可达 ⇒ ``SKIP(reason=hardware_unreachable)`` + 未验证指标，退出码 0。"""
    reason = probe.reason or SKIP_REASON_HARDWARE_UNREACHABLE
    skip_notice = format_skip(probe, UNVERIFIED_TASK_METRICS)
    if as_json:
        payload = {
            "command": "long_run",
            "status": "skipped",
            "mode": "hardware",
            "offline": False,
            "physical_verified": False,
            "reason": reason,
            "detail": probe.detail,
            "skip_notice": skip_notice,
            "cycles": 0,
            "success_count": 0,
            "failure_count": 0,
            "success_rate": None,
            "exceptions": 0,
            "aborted": False,
            "aborted_reason": "",
            "duration_s": None,
            "thresholds_met": False,
            "threshold_failures": ["真机不可达：物理长稳未验证"],
            "unverified_metrics": list(UNVERIFIED_TASK_METRICS),
            "long_run_metric": UNVERIFIED_TASK_METRIC_LONG_RUN_8H,
        }
        print(json.dumps(payload, ensure_ascii=False, indent=2, default=str))
    else:
        print(skip_notice)
    return EXIT_OK


# ── 入口 ────────────────────────────────────────────────────────────────
def main(argv: Optional[Sequence[str]] = None) -> int:
    """脚本入口（返回退出码）。"""
    parser = build_parser()
    args = parser.parse_args(list(argv) if argv is not None else None)

    try:
        config = _validate_args(args)
    except UsageError as exc:
        print(f"long_run: {exc}", file=sys.stderr)
        return EXIT_USAGE

    dry_run = bool(config["dry_run"])
    device_config = None

    # ── 硬件门控（真机不可达 ⇒ SKIP，退出码 0）─────────────────────
    if not dry_run:
        try:
            device_config = load_device_config(config["device_config_path"])
        except DeviceConfigError as exc:
            print(f"long_run: 设备配置非法：{exc}", file=sys.stderr)
            return EXIT_USAGE
        probe = probe_hardware(device_config, timeout=float(config["connect_timeout"]))
        if not probe.reachable:
            return _emit_skip(probe, as_json=bool(args.json))

    try:
        payload = run_long_run(
            task_path=config["task_path"],
            scenario_path=config["scenario_path"],
            duration_s=float(config["duration_s"]),
            max_cycles=config["max_cycles"],
            min_success_rate=float(config["min_success_rate"]),
            seed=int(config["seed"]),
            jitter_rad=float(config["jitter_rad"]),
            dry_run=dry_run,
            device_config=device_config,
        )
    except UsageError as exc:
        print(f"long_run: {exc}", file=sys.stderr)
        return EXIT_USAGE

    if args.json:
        print(json.dumps(payload, ensure_ascii=False, indent=2, default=str))
    else:
        _emit_human(payload)

    if dry_run:
        # 离线演练：结果**不代表**真机达标 ⇒ 退出码 0（门槛结论看 JSON 字段）。
        return EXIT_OK
    return EXIT_OK if payload["thresholds_met"] else EXIT_FAILURE


if __name__ == "__main__":
    sys.exit(main())
