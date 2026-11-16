"""任务原语级指标口径（S06-T07）。

为什么需要这一层
----------------
R011 的验收口径是「**每个原语有可测的成功率与误差门槛**」——因此必须有
一个**逐原语**的指标口径，而不是只有「任务整体成功 / 失败」。

口径纪律（与 S05 的 :mod:`scripts.coordinated_stats` 同构，**不得另立第二套**）
----------------------------------------------------------------------------
* **验收口径**（:data:`TaskMetricsReport.acceptance_metrics`）= **求解阶段**
  的逐原语指标：成功率、位置误差 P95、姿态误差 P95、零逃逸率。
* **端到端口径**（:data:`TaskMetricsReport.end_to_end_metrics`）= 含
  **场景装配**与**整条原语链**的开销，**单列**，**不混入**验收口径。
* **真机 / 物理指标一律硬件门控 SKIP**：本环境无真机 ⇒ 真机指标逐条列入
  :data:`UNVERIFIED_TASK_METRICS`，**退出码仍为 0**，
  **绝不用合成数据冒充物理达标**（K001）。

默认门槛（与 S06-T03 的原语容差同源）
------------------------------------
* 每原语 ``success_rate ≥ 0.95``；
* 位置误差 ``P95 ≤ 1e-5 m``、姿态误差 ``P95 ≤ 1e-5 rad``
  （与 :data:`rm75_drake.tasks.primitives.DEFAULT_POSITION_TOLERANCE_M`
  / :data:`rm75_drake.tasks.primitives.DEFAULT_ORIENTATION_TOLERANCE_RAD` 同源）；
* 零逃逸率 ``= 1.0``（100%）。

零 pydrake 依赖
---------------
本模块只用标准库（:func:`dataclasses` / :mod:`typing` / :mod:`math`），
**不 import** 求解器与场景装配层——聚合器只消费「结果对象」
（鸭子类型：``name`` / ``success`` / ``stage`` / ``reason`` /
``position_error_m`` / ``orientation_error_rad``），
因此指标口径可以**零硬件、零 Drake** 单测。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

__all__ = [
    "UNVERIFIED_TASK_METRICS",
    "DEFAULT_PRIMITIVE_SUCCESS_RATE_THRESHOLD",
    "DEFAULT_PRIMITIVE_POSITION_ERROR_P95_M",
    "DEFAULT_PRIMITIVE_ORIENTATION_ERROR_P95_RAD",
    "DEFAULT_ZERO_ESCAPE_RATE_THRESHOLD",
    "PrimitiveMetric",
    "TaskMetricsReport",
    "percentile",
    "build_task_metrics_report",
]


#: 真机 / 物理指标清单（**唯一真相源**，canonical tuple）。
#:
#: 真相源在**本模块**——CLI（:mod:`rm75_drake.cli`）与
#: :mod:`scripts.task_stats` 只允许**再导出**，**禁止复制第二份**
#: （与 :data:`rm75_drake.cli.UNVERIFIED_COORDINATED_METRICS` 同款纪律）。
#:
#: 每条都附**解锁条件**：没有真机实测数据之前，这些指标**一律未验证**。
UNVERIFIED_TASK_METRICS: Tuple[str, ...] = (
    "真实（真机）抓放成功率 ≥ 95%（10×10 目标网格，未验证；"
    "解锁条件：真机在 10×10 网格上完成抓放并记录成功率）",
    "真实（真机）夹持误差（夹爪夹持后物体实际位姿偏差，未验证；"
    "解锁条件：真机夹持后测量物体位姿偏差 ≤ 1 mm）",
    "真实（真机）交接保持漂移（双臂交接期间相对位姿漂移，未验证；"
    "解锁条件：真机交接保持 10 s 且漂移 ≤ 2 mm）",
    "真实（真机）感知延迟（检测到物体 → 指令下发的端到端延迟，未验证；"
    "解锁条件：真机时间戳实测 P95 ≤ 100 ms）",
    "真实（真机）夹爪力控 / 夹空检测（力曲线阈值与空夹判据，未验证；"
    "解锁条件：真机标定力曲线并统计空夹误判率）",
    "真实（真机）长稳节拍（连续运行的成功率与节拍，未验证；"
    "解锁条件：真机连续运行 ≥ 2 h 且任务成功率 ≥ 95%）",
)


#: 默认门槛：每原语成功率下限。
DEFAULT_PRIMITIVE_SUCCESS_RATE_THRESHOLD: float = 0.95

#: 默认门槛：位置误差 P95 上限（m）。
DEFAULT_PRIMITIVE_POSITION_ERROR_P95_M: float = 1.0e-5

#: 默认门槛：姿态误差 P95 上限（rad）。
DEFAULT_PRIMITIVE_ORIENTATION_ERROR_P95_RAD: float = 1.0e-5

#: 默认门槛：零逃逸率下限（100%）。
DEFAULT_ZERO_ESCAPE_RATE_THRESHOLD: float = 1.0


# ── 小工具 ──────────────────────────────────────────────────────────────
def percentile(values: Sequence[float], fraction: float) -> Optional[float]:
    """返回分位数（线性插值口径，与 :mod:`scripts.coordinated_stats` 一致）。

    空序列 → ``None``（表示「该指标不适用」，**不**伪造 0.0）。
    """
    if not values:
        return None
    ordered = sorted(float(v) for v in values)
    if len(ordered) == 1:
        return float(ordered[0])
    position = float(fraction) * (len(ordered) - 1)
    low = int(math.floor(position))
    high = int(math.ceil(position))
    if low == high:
        return float(ordered[low])
    weight = position - low
    return float(ordered[low] * (1.0 - weight) + ordered[high] * weight)


def _frozen_mapping(data: Optional[Mapping[str, Any]]) -> Mapping[str, Any]:
    """把可变映射冻结成只读映射（frozen dataclass 的字段纪律）。"""
    return MappingProxyType(dict(data or {}))


# ── 逐原语指标 ──────────────────────────────────────────────────────────
@dataclass(frozen=True)
class PrimitiveMetric:
    """**单个原语**的指标（验收口径）。

    字段
    ----
    name:
        原语名。
    samples:
        该原语被尝试的样本数（= 跑到的样本数）。
    success_rate:
        成功率（0–1）。
    position_error_p95_m:
        位置误差 P95（m）；该原语不产生位置误差（如夹爪类）→ ``None``。
    orientation_error_p95_rad:
        姿态误差 P95（rad）；同上 → ``None``。
    zero_escape_rate:
        零逃逸率（该原语**没有**出现碰撞逃逸的样本比例，0–1）。
    failure_breakdown:
        失败分类计数（``"{stage}:{reason}"`` → 次数）。
    """

    name: str
    samples: int
    success_rate: float
    position_error_p95_m: Optional[float]
    orientation_error_p95_rad: Optional[float]
    zero_escape_rate: float
    failure_breakdown: Mapping[str, int] = field(default_factory=dict)
    success_rate_threshold: float = DEFAULT_PRIMITIVE_SUCCESS_RATE_THRESHOLD
    position_error_p95_threshold_m: float = DEFAULT_PRIMITIVE_POSITION_ERROR_P95_M
    orientation_error_p95_threshold_rad: float = DEFAULT_PRIMITIVE_ORIENTATION_ERROR_P95_RAD
    zero_escape_rate_threshold: float = DEFAULT_ZERO_ESCAPE_RATE_THRESHOLD

    def __post_init__(self) -> None:
        object.__setattr__(self, "failure_breakdown", _frozen_mapping(self.failure_breakdown))

    @property
    def thresholds_met(self) -> bool:
        """该原语是否**全部**达标。"""
        return not self.threshold_failures()

    def threshold_failures(self) -> Tuple[str, ...]:
        """返回**未达标项**的说明（空元组 ⇒ 全部达标）。"""
        problems: List[str] = []
        if self.samples > 0 and self.success_rate < self.success_rate_threshold:
            problems.append(
                f"成功率 {self.success_rate:.3f} < 门槛 {self.success_rate_threshold:.3f}"
            )
        if (
            self.position_error_p95_m is not None
            and self.position_error_p95_m > self.position_error_p95_threshold_m
        ):
            problems.append(
                f"位置误差 P95 {self.position_error_p95_m:.3e} m > 门槛 "
                f"{self.position_error_p95_threshold_m:.3e} m"
            )
        if (
            self.orientation_error_p95_rad is not None
            and self.orientation_error_p95_rad > self.orientation_error_p95_threshold_rad
        ):
            problems.append(
                f"姿态误差 P95 {self.orientation_error_p95_rad:.3e} rad > 门槛 "
                f"{self.orientation_error_p95_threshold_rad:.3e} rad"
            )
        if self.zero_escape_rate < self.zero_escape_rate_threshold:
            problems.append(
                f"零逃逸率 {self.zero_escape_rate:.3f} < 门槛 "
                f"{self.zero_escape_rate_threshold:.3f}"
            )
        return tuple(problems)

    def as_dict(self) -> Dict[str, Any]:
        """返回可序列化的字典视图（CLI / 脚本 JSON 输出用）。"""
        return {
            "name": self.name,
            "samples": int(self.samples),
            "success_rate": float(self.success_rate),
            "position_error_p95_m": self.position_error_p95_m,
            "orientation_error_p95_rad": self.orientation_error_p95_rad,
            "zero_escape_rate": float(self.zero_escape_rate),
            "failure_breakdown": dict(self.failure_breakdown),
            "thresholds": {
                "success_rate": float(self.success_rate_threshold),
                "position_error_p95_m": float(self.position_error_p95_threshold_m),
                "orientation_error_p95_rad": float(
                    self.orientation_error_p95_threshold_rad
                ),
                "zero_escape_rate": float(self.zero_escape_rate_threshold),
            },
            "thresholds_met": self.thresholds_met,
            "threshold_failures": list(self.threshold_failures()),
        }


# ── 任务级报告 ──────────────────────────────────────────────────────────
@dataclass(frozen=True)
class TaskMetricsReport:
    """**任务级**指标报告（验收口径 / 端到端口径**分行**）。

    字段
    ----
    task_name:
        任务名。
    samples:
        样本数（跑了几次整条原语链）。
    primitives:
        逐原语指标（按首次出现顺序）。
    success_rate:
        整条链的成功率（0–1）。
    zero_escape_rate:
        整条链的零逃逸率（0–1）。
    acceptance_metrics:
        **验收口径**（求解阶段）：逐原语成功率 / 误差分位数 / 零逃逸率。
    end_to_end_metrics:
        **端到端口径**（含场景装配与整链开销）：**单列**，不混入验收。
    unverified_metrics:
        真机 / 物理指标清单（硬件门控 SKIP ⇒ 逐条列出，退出码仍为 0）。
    thresholds_met:
        全部原语是否达标。
    threshold_failures:
        未达标项的说明（``"{原语}：{原因}"``）。
    """

    task_name: str
    samples: int
    primitives: Tuple[PrimitiveMetric, ...]
    success_rate: float
    zero_escape_rate: float
    acceptance_metrics: Mapping[str, Any] = field(default_factory=dict)
    end_to_end_metrics: Mapping[str, Any] = field(default_factory=dict)
    unverified_metrics: Tuple[str, ...] = UNVERIFIED_TASK_METRICS
    thresholds_met: bool = True
    threshold_failures: Tuple[str, ...] = ()

    def __post_init__(self) -> None:
        object.__setattr__(self, "primitives", tuple(self.primitives))
        object.__setattr__(
            self, "acceptance_metrics", _frozen_mapping(self.acceptance_metrics)
        )
        object.__setattr__(
            self, "end_to_end_metrics", _frozen_mapping(self.end_to_end_metrics)
        )
        object.__setattr__(self, "unverified_metrics", tuple(self.unverified_metrics))
        object.__setattr__(self, "threshold_failures", tuple(self.threshold_failures))

    def primitive(self, name: str) -> Optional[PrimitiveMetric]:
        """按原语名取指标（找不到 → ``None``）。"""
        for item in self.primitives:
            if item.name == name:
                return item
        return None

    def as_dict(self) -> Dict[str, Any]:
        """返回可序列化的字典视图（CLI / 脚本 JSON 输出用）。"""
        return {
            "task_name": self.task_name,
            "samples": int(self.samples),
            "success_rate": float(self.success_rate),
            "zero_escape_rate": float(self.zero_escape_rate),
            "primitives": [item.as_dict() for item in self.primitives],
            "acceptance_metrics": dict(self.acceptance_metrics),
            "end_to_end_metrics": dict(self.end_to_end_metrics),
            "unverified_metrics": list(self.unverified_metrics),
            "thresholds_met": bool(self.thresholds_met),
            "threshold_failures": list(self.threshold_failures),
        }


# ── 聚合器 ──────────────────────────────────────────────────────────────
def _is_escape(reason: str, stage: str) -> bool:
    """判断一次原语失败是否属于**碰撞逃逸**（稳定 reason 词表）。"""
    return str(reason) == "escape" or str(stage) == "escape"


def build_task_metrics_report(
    samples: Sequence[Any],
    *,
    task_name: str,
    acceptance_extra: Optional[Mapping[str, Any]] = None,
    end_to_end_extra: Optional[Mapping[str, Any]] = None,
    unverified_metrics: Tuple[str, ...] = UNVERIFIED_TASK_METRICS,
    success_rate_threshold: float = DEFAULT_PRIMITIVE_SUCCESS_RATE_THRESHOLD,
    position_error_p95_threshold_m: float = DEFAULT_PRIMITIVE_POSITION_ERROR_P95_M,
    orientation_error_p95_threshold_rad: float = DEFAULT_PRIMITIVE_ORIENTATION_ERROR_P95_RAD,
    zero_escape_rate_threshold: float = DEFAULT_ZERO_ESCAPE_RATE_THRESHOLD,
) -> TaskMetricsReport:
    """把「N 次整链运行」聚合成 :class:`TaskMetricsReport`。

    参数
    ----
    samples:
        每次整条原语链的运行结果序列（鸭子类型：需有 ``results`` 序列，
        每个元素有 ``name`` / ``success`` / ``stage`` / ``reason`` /
        ``position_error_m`` / ``orientation_error_rad``；典型是
        :class:`rm75_drake.tasks.solver.TaskResult`）。
    task_name:
        任务名。
    acceptance_extra:
        追加到**验收口径**段的自定义字段（如耗时口径说明）。
    end_to_end_extra:
        追加到**端到端口径**段的自定义字段（如场景装配耗时）。
    unverified_metrics:
        未验证的物理指标清单（默认取 canonical 清单）。

    返回
    ----
    :class:`TaskMetricsReport`

    备注
    ----
    * **失败即停止**：某个样本在首个失败原语处终止 ⇒ 该样本中**后续原语
      不计入**分母（分母 = 「被尝试到的样本数」），否则失败样本会把
      后续原语的成功率凭空拉低。
    * **零逃逸率**只统计**碰撞逃逸**（``reason="escape"``），
      其余失败（IK 不可达等）不计入逃逸。
    """
    attempts: Dict[str, int] = {}
    successes: Dict[str, int] = {}
    escapes: Dict[str, int] = {}
    position_errors: Dict[str, List[float]] = {}
    orientation_errors: Dict[str, List[float]] = {}
    breakdown: Dict[str, Dict[str, int]] = {}
    order: List[str] = []

    task_successes = 0
    task_escapes = 0
    durations: List[float] = []

    for sample in samples:
        results = list(getattr(sample, "results", ()) or ())
        sample_has_escape = False
        for result in results:
            name = str(getattr(result, "name", "?"))
            if name not in attempts:
                attempts[name] = 0
                successes[name] = 0
                escapes[name] = 0
                position_errors[name] = []
                orientation_errors[name] = []
                breakdown[name] = {}
                order.append(name)
            attempts[name] += 1
            success = bool(getattr(result, "success", False))
            stage = str(getattr(result, "stage", "") or "")
            reason = str(getattr(result, "reason", "") or "")
            if success:
                successes[name] += 1
            else:
                key = f"{stage}:{reason}" if stage or reason else "unknown"
                breakdown[name][key] = breakdown[name].get(key, 0) + 1
                if _is_escape(reason, stage):
                    escapes[name] += 1
                    sample_has_escape = True
            position_error = getattr(result, "position_error_m", None)
            if position_error is not None:
                position_errors[name].append(float(position_error))
            orientation_error = getattr(result, "orientation_error_rad", None)
            if orientation_error is not None:
                orientation_errors[name].append(float(orientation_error))
        if bool(getattr(sample, "success", False)):
            task_successes += 1
        if sample_has_escape:
            task_escapes += 1
        duration = getattr(sample, "duration_s", None)
        if duration is not None:
            durations.append(float(duration))

    primitive_metrics: List[PrimitiveMetric] = []
    for name in order:
        count = attempts[name]
        metric = PrimitiveMetric(
            name=name,
            samples=count,
            success_rate=(successes[name] / count) if count else 0.0,
            position_error_p95_m=percentile(position_errors[name], 0.95),
            orientation_error_p95_rad=percentile(orientation_errors[name], 0.95),
            zero_escape_rate=((count - escapes[name]) / count) if count else 0.0,
            failure_breakdown=breakdown[name],
            success_rate_threshold=success_rate_threshold,
            position_error_p95_threshold_m=position_error_p95_threshold_m,
            orientation_error_p95_threshold_rad=orientation_error_p95_threshold_rad,
            zero_escape_rate_threshold=zero_escape_rate_threshold,
        )
        primitive_metrics.append(metric)

    total = len(samples)
    task_success_rate = (task_successes / total) if total else 0.0
    task_zero_escape_rate = ((total - task_escapes) / total) if total else 0.0

    failures: List[str] = []
    for metric in primitive_metrics:
        for problem in metric.threshold_failures():
            failures.append(f"{metric.name}：{problem}")

    acceptance: Dict[str, Any] = {
        "note": (
            "验收口径（R011）：逐原语成功率 ≥ "
            f"{success_rate_threshold:.2f}、位置误差 P95 ≤ "
            f"{position_error_p95_threshold_m:.1e} m、姿态误差 P95 ≤ "
            f"{orientation_error_p95_threshold_rad:.1e} rad、零逃逸率 = "
            "100%；**只含求解阶段**（不含场景装配与整链开销）。"
        ),
        "task_success_rate": task_success_rate,
        "task_zero_escape_rate": task_zero_escape_rate,
        "thresholds": {
            "success_rate": success_rate_threshold,
            "position_error_p95_m": position_error_p95_threshold_m,
            "orientation_error_p95_rad": orientation_error_p95_threshold_rad,
            "zero_escape_rate": zero_escape_rate_threshold,
        },
        "thresholds_met": not failures,
        "threshold_failures": list(failures),
        "primitives": {
            metric.name: {
                "samples": metric.samples,
                "success_rate": metric.success_rate,
                "position_error_p95_m": metric.position_error_p95_m,
                "orientation_error_p95_rad": metric.orientation_error_p95_rad,
                "zero_escape_rate": metric.zero_escape_rate,
                "thresholds_met": metric.thresholds_met,
            }
            for metric in primitive_metrics
        },
    }
    if acceptance_extra:
        acceptance.update(dict(acceptance_extra))

    end_to_end: Dict[str, Any] = {
        "note": (
            "端到端口径：含**场景装配**与**整条原语链**的开销 —— "
            "**单列**，不作为验收口径。"
        ),
        "samples": total,
        "per_sample_duration_s": {
            "p50": percentile(durations, 0.50),
            "p95": percentile(durations, 0.95),
            "mean": (sum(durations) / len(durations)) if durations else None,
            "max": (max(durations) if durations else None),
        },
    }
    if end_to_end_extra:
        end_to_end.update(dict(end_to_end_extra))

    return TaskMetricsReport(
        task_name=task_name,
        samples=total,
        primitives=tuple(primitive_metrics),
        success_rate=task_success_rate,
        zero_escape_rate=task_zero_escape_rate,
        acceptance_metrics=acceptance,
        end_to_end_metrics=end_to_end,
        unverified_metrics=tuple(unverified_metrics),
        thresholds_met=not failures,
        threshold_failures=tuple(failures),
    )
