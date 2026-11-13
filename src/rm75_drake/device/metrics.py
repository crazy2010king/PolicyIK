"""抖动直方图与命令→执行延迟指标（S03-T04）。

口径纪律（**禁止两套口径**）
--------------------------
本模块是**唯一**的指标实现：离线单测（:mod:`tests.test_device_metrics`）
与真机测量脚本（``scripts/measure_jitter.py``，S03-T08）**必须**复用同一份
:class:`MetricHistogram`，否则「CI 里看起来达标、真机上不达标」将无法排查。

分位数口径
----------
统一采用 **线性插值**（等价于 numpy 的 ``method="linear"``）：

.. math::

    pos = (n - 1) \\cdot q/100, \\quad
    v = x_{\\lfloor pos\\rfloor} + (pos - \\lfloor pos\\rfloor)
        \\cdot (x_{\\lceil pos\\rceil} - x_{\\lfloor pos\\rfloor})

即 ``P50 / P99 / P99.9`` 都可**手算复核**（单测里逐条钉死）。

单位纪律
--------
内部一律 **秒（s）**；展示层（运行日志 / 真机报告）再换算成毫秒，
换算只发生在 :meth:`MetricHistogram.summary_ms` 一处。

预分配
------
:class:`MetricHistogram` 内部是**预分配列表**（初始 ``initial_capacity``
个槽位，稳态下零分配；容量不足时按**倍增**扩容，摊还 O(1)）。
超过 ``max_samples`` 后**不静默丢弃**：计数到 ``dropped``，
由报告显式暴露。
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Sequence

__all__ = [
    "PERCENTILE_LABELS",
    "DEFAULT_MAX_SAMPLES",
    "MetricHistogram",
    "JitterHistogram",
    "LatencyHistogram",
    "linear_percentile",
]


#: 报告里固定出现的分位数标签（顺序即 P50 → P99 → P99.9）。
PERCENTILE_LABELS = ("p50", "p99", "p999")

#: 直方图默认容量（预分配槽位上限）。
DEFAULT_MAX_SAMPLES = 2_000_000


def linear_percentile(sorted_values: Sequence[float], q: float) -> float:
    """线性插值分位数（numpy ``method="linear"`` 口径）。

    参数
    ----
    sorted_values:
        **已升序**的样本序列（长度 ≥ 1）。
    q:
        分位（0–100）。

    异常
    ----
    ValueError
        样本为空、``q`` 越界、样本含 NaN/Inf。
    """
    values = list(sorted_values)
    if not values:
        raise ValueError("分位数要求至少一个样本")
    quantile = float(q)
    if not math.isfinite(quantile) or not 0.0 <= quantile <= 100.0:
        raise ValueError(f"q 必须在 [0, 100] 内，实际是 {q!r}")
    for value in values:
        if not math.isfinite(float(value)):
            raise ValueError(f"样本含有 NaN/Inf：{value!r}")
    if len(values) == 1:
        return float(values[0])
    position = (len(values) - 1) * (quantile / 100.0)
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return float(values[int(position)])
    weight = position - lower
    return float(values[lower]) * (1.0 - weight) + float(values[upper]) * weight


class MetricHistogram:
    """一个**预分配**的指标直方图（样本 + 分位数 + 基础统计）。

    参数
    ----
    name:
        指标名（如 ``jitter`` / ``command_latency``）。
    unit:
        单位标签（内部一律 ``"s"``）。
    max_samples:
        容量上限；超出后**不再收样本**，但计数到 ``dropped``
        （**不静默丢弃**）。
    initial_capacity:
        **预分配**槽位数（稳态下不产生周期性分配）；容量不足时按
        **倍增**扩容（摊还 O(1)），上限为 ``max_samples``。
    """

    __slots__ = (
        "name",
        "unit",
        "_max_samples",
        "_samples",
        "_capacity",
        "_count",
        "_dropped",
        "_sum",
        "_min",
        "_max",
    )

    def __init__(
        self,
        name: str,
        unit: str = "s",
        *,
        max_samples: int = DEFAULT_MAX_SAMPLES,
        initial_capacity: int = 4096,
    ) -> None:
        if not isinstance(name, str) or not name:
            raise ValueError(f"name 必须是非空字符串，实际是 {name!r}")
        if isinstance(max_samples, bool) or not isinstance(max_samples, int):
            raise ValueError(f"max_samples 必须是整数，实际是 {max_samples!r}")
        if max_samples < 1:
            raise ValueError(f"max_samples 必须 ≥ 1，实际是 {max_samples}")
        if isinstance(initial_capacity, bool) or not isinstance(initial_capacity, int):
            raise ValueError(
                f"initial_capacity 必须是整数，实际是 {initial_capacity!r}"
            )
        if initial_capacity < 1:
            raise ValueError(
                f"initial_capacity 必须 ≥ 1，实际是 {initial_capacity}"
            )
        self.name = name
        self.unit = unit
        self._max_samples = int(max_samples)
        self._capacity = min(int(initial_capacity), self._max_samples)
        self._samples: List[float] = [0.0] * self._capacity
        self._count = 0
        self._dropped = 0
        self._sum = 0.0
        self._min = math.inf
        self._max = -math.inf

    # -- 写入 ------------------------------------------------------------
    def add(self, value: float) -> None:
        """记录一个样本（秒）。

        异常
        ----
        ValueError
            样本不是有限实数（点名字段）。
        """
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"{self.name} 的样本必须是数值，收到 {value!r}")
        sample = float(value)
        if not math.isfinite(sample):
            raise ValueError(f"{self.name} 的样本必须是有限数值，收到 {value!r}")
        if self._count >= self._capacity:
            if self._capacity < self._max_samples:
                # 倍增扩容（摊还 O(1)）：稳态下不发生周期性分配。
                new_capacity = min(
                    self._max_samples, max(self._capacity * 2, 1)
                )
                self._samples.extend([0.0] * (new_capacity - self._capacity))
                self._capacity = new_capacity
            else:
                self._dropped += 1
                return
        self._samples[self._count] = sample
        self._count += 1
        self._sum += sample
        if sample < self._min:
            self._min = sample
        if sample > self._max:
            self._max = sample

    # -- 读取 ------------------------------------------------------------
    @property
    def count(self) -> int:
        """已记录样本数。"""
        return self._count

    @property
    def dropped(self) -> int:
        """因超出容量而**未记录**的样本数（显式暴露，不静默丢弃）。"""
        return self._dropped

    @property
    def capacity(self) -> int:
        """当前**预分配**容量（随样本增长按倍增扩容）。"""
        return self._capacity

    @property
    def max_samples(self) -> int:
        """容量上限（超过后样本被计入 ``dropped``）。"""
        return self._max_samples

    @property
    def min(self) -> Optional[float]:
        """最小样本；空直方图返回 ``None``。"""
        return None if self._count == 0 else self._min

    @property
    def max(self) -> Optional[float]:
        """最大样本；空直方图返回 ``None``。"""
        return None if self._count == 0 else self._max

    @property
    def mean(self) -> Optional[float]:
        """平均样本；空直方图返回 ``None``。"""
        if self._count == 0:
            return None
        return self._sum / self._count

    def sorted_samples(self) -> List[float]:
        """返回升序样本副本。"""
        return sorted(self._samples[: self._count])

    def percentile(self, q: float) -> float:
        """返回指定分位数（线性插值口径）。"""
        return linear_percentile(self.sorted_samples(), q)

    @property
    def p50(self) -> Optional[float]:
        """P50（空直方图为 ``None``）。"""
        return None if self._count == 0 else self.percentile(50.0)

    @property
    def p99(self) -> Optional[float]:
        """P99（空直方图为 ``None``）。"""
        return None if self._count == 0 else self.percentile(99.0)

    @property
    def p999(self) -> Optional[float]:
        """P99.9（空直方图为 ``None``）。"""
        return None if self._count == 0 else self.percentile(99.9)

    # -- 报告 ------------------------------------------------------------
    def summary(self) -> Dict[str, Any]:
        """返回可 JSON 序列化的统计摘要（单位与 ``self.unit`` 一致）。"""
        return {
            "name": self.name,
            "unit": self.unit,
            "count": self._count,
            "dropped": self._dropped,
            "min": self.min,
            "max": self.max,
            "mean": self.mean,
            "p50": self.p50,
            "p99": self.p99,
            "p999": self.p999,
        }

    def summary_ms(self) -> Dict[str, Any]:
        """返回**毫秒**口径的统计摘要（唯一换算点）。"""
        summary = self.summary()
        summary["unit"] = "ms"
        for key in ("min", "max", "mean", "p50", "p99", "p999"):
            value = summary[key]
            summary[key] = None if value is None else float(value) * 1000.0
        return summary

    def __len__(self) -> int:
        return self._count

    def __repr__(self) -> str:  # pragma: no cover - 调试辅助
        return (
            f"MetricHistogram(name={self.name!r}, count={self._count}, "
            f"max={self.max}, p99={self.p99})"
        )


class JitterHistogram(MetricHistogram):
    """**节拍抖动**直方图（秒）。

    抖动定义：每一拍的**实际周期**（与上一拍起点之差）与**标称周期**
    （5 ms）的偏差绝对值（ms 级）。睡眠过冲、拍内超时都会体现在这里。
    """

    __slots__ = ()

    def __init__(self, *, max_samples: int = DEFAULT_MAX_SAMPLES) -> None:
        super().__init__("jitter", "s", max_samples=max_samples)


class LatencyHistogram(MetricHistogram):
    """**命令→执行延迟**直方图（秒）。

    延迟定义：从「指令交给传输层」到「传输层返回」的时间。
    """

    __slots__ = ()

    def __init__(self, *, max_samples: int = DEFAULT_MAX_SAMPLES) -> None:
        super().__init__("command_latency", "s", max_samples=max_samples)
