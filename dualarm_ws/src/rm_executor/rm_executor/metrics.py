"""rm_executor 指标口径（**纯函数、零 I/O、零时钟**）。

本模块冻结 S03 报告里所有指标的计算口径，后续时钟/闭环/入口任务只调用
这里的函数，**不再各写一套**：

* tick 抖动：样本 = 「实际 tick 时刻 − 理想截止时刻」，单位**毫秒**；
  分位数用**线性插值法**（与 numpy 的 ``linear`` 插值同口径）；
* ``rt_factor`` = 实际耗时 / 计划耗时；
* 跟随误差：``|q_cmd − q_act|``（**度**）的 RMS 与峰值；
* 到位误差与保持时长。

**空样本返回 ``None``，绝不返回 0 冒充**（0 会被误读成「抖动为零」/「误差为零」）。
"""
from __future__ import annotations

import math
from typing import Any, Dict, Iterable, List, Optional, Sequence

__all__ = [
    "arrival_metrics",
    "duration_s",
    "follow_error_stats",
    "jitter_stats",
    "percentile",
    "rt_factor",
]


def _finite(values: Iterable[Any]) -> List[float]:
    """挑出有限数值（跳过 None / NaN / Inf / 非数值）。"""
    out: List[float] = []
    for value in values or []:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            continue
        fval = float(value)
        if math.isfinite(fval):
            out.append(fval)
    return out


def percentile(samples: Sequence[float], q: float) -> Optional[float]:
    """线性插值分位数（numpy ``linear`` 口径）。

    参数
    ----
    q
        百分位（0..100）。

    返回
    ----
    float | None
        空样本 → ``None``（不返回 0 冒充）。
    """
    values = _finite(samples)
    if not values:
        return None
    if q < 0 or q > 100:
        raise ValueError("百分位必须在 0..100（当前 %r）" % (q,))
    values.sort()
    if len(values) == 1:
        return values[0]
    pos = (len(values) - 1) * (q / 100.0)
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return values[lo]
    return values[lo] + (values[hi] - values[lo]) * (pos - lo)


def jitter_stats(samples_ms: Sequence[float]) -> Dict[str, Any]:
    """tick 抖动统计（样本单位：毫秒）。

    返回 ``{"count": int, "p50_ms", "p95_ms", "p99_ms", "max_ms"}``；
    **空样本时后四项为 ``None``**（不返回 0 冒充）。
    """
    values = _finite(samples_ms)
    if not values:
        return {
            "count": 0,
            "p50_ms": None,
            "p95_ms": None,
            "p99_ms": None,
            "max_ms": None,
        }
    return {
        "count": len(values),
        "p50_ms": percentile(values, 50),
        "p95_ms": percentile(values, 95),
        "p99_ms": percentile(values, 99),
        "max_ms": max(values),
    }


def rt_factor(planned_s: Optional[float], actual_s: Optional[float]) -> Optional[float]:
    """实时因子 = 实际耗时 / 计划耗时。

    计划耗时 ≤ 0 或缺值 → ``None``（不返回 0 冒充）。
    """
    if planned_s is None or actual_s is None:
        return None
    try:
        planned = float(planned_s)
        actual = float(actual_s)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(planned) or not math.isfinite(actual) or planned <= 0:
        return None
    return actual / planned


def follow_error_stats(errors_deg: Sequence[float]) -> Dict[str, Any]:
    """跟随误差统计（单位：度）。

    返回 ``{"count": int, "rms_deg", "max_deg"}``；
    **空样本时后两项为 ``None``**。
    """
    values = _finite(errors_deg)
    if not values:
        return {"count": 0, "rms_deg": None, "max_deg": None}
    rms = math.sqrt(sum(v * v for v in values) / len(values))
    return {"count": len(values), "rms_deg": rms, "max_deg": max(values)}


def arrival_metrics(err_deg: Optional[float] = None,
                    hold_ms: Optional[float] = None) -> Dict[str, Any]:
    """到位指标（到位角误差 / 保持时长）。

    未到位 → ``{"arrival_err_deg": None, "arrival_hold_ms": None}``（**不伪造 0**）。
    """
    err = None if err_deg is None else float(err_deg)
    hold = None if hold_ms is None else float(hold_ms)
    if err is not None and not math.isfinite(err):
        err = None
    if hold is not None and not math.isfinite(hold):
        hold = None
    return {"arrival_err_deg": err, "arrival_hold_ms": hold}


def duration_s(t0: Optional[float], t1: Optional[float]) -> Optional[float]:
    """时长（秒）；缺值或非法 → ``None``。"""
    if t0 is None or t1 is None:
        return None
    try:
        delta = float(t1) - float(t0)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(delta) or delta < 0:
        return None
    return delta
