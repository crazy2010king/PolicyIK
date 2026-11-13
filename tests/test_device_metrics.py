"""抖动直方图与延迟指标的单测（S03-T04）。

设计要点
--------
* **分位数与手算一致**：P50 / P99 / P99.9 逐条钉死（线性插值口径）。
* **同一份实现**：单测与真机脚本复用 :mod:`rm75_drake.device.metrics`，
  不允许出现第二套口径。
* 负例覆盖：空样本、NaN/Inf、越界分位、容量溢出、非法构造参数。
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = REPO_ROOT / "src"

if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from rm75_drake.device.metrics import (  # noqa: E402
    DEFAULT_MAX_SAMPLES,
    PERCENTILE_LABELS,
    JitterHistogram,
    LatencyHistogram,
    MetricHistogram,
    linear_percentile,
)


# ── 分位数：手算复核 ────────────────────────────────────────────────────
def test_linear_percentile_matches_hand_computation() -> None:
    """1..100 的升序样本：P50=50.5、P99=99.01、P99.9=99.901。"""
    values = [float(i) for i in range(1, 101)]
    assert linear_percentile(values, 0.0) == pytest.approx(1.0)
    assert linear_percentile(values, 50.0) == pytest.approx(50.5)
    assert linear_percentile(values, 99.0) == pytest.approx(99.01)
    assert linear_percentile(values, 99.9) == pytest.approx(99.901)
    assert linear_percentile(values, 100.0) == pytest.approx(100.0)


def test_linear_percentile_single_sample() -> None:
    assert linear_percentile([0.25], 50.0) == pytest.approx(0.25)
    assert linear_percentile([0.25], 99.9) == pytest.approx(0.25)


def test_linear_percentile_rejects_bad_input() -> None:
    """负例：空样本 / 越界 q / NaN 样本。"""
    with pytest.raises(ValueError, match="至少一个样本"):
        linear_percentile([], 50.0)
    with pytest.raises(ValueError, match=r"\[0, 100\]"):
        linear_percentile([1.0], 101.0)
    with pytest.raises(ValueError, match="NaN/Inf"):
        linear_percentile([1.0, math.nan], 50.0)


# ── 直方图 ──────────────────────────────────────────────────────────────
def test_histogram_basic_statistics() -> None:
    """样本数 / 最小 / 最大 / 平均 / 分位数。"""
    histogram = MetricHistogram("demo", "s")
    for value in (0.001, 0.002, 0.003, 0.004):
        histogram.add(value)
    assert histogram.count == 4
    assert len(histogram) == 4
    assert histogram.min == pytest.approx(0.001)
    assert histogram.max == pytest.approx(0.004)
    assert histogram.mean == pytest.approx(0.0025)
    assert histogram.p50 == pytest.approx(0.0025)
    assert histogram.p99 == pytest.approx(0.00397)
    assert histogram.p999 == pytest.approx(0.003997)
    assert histogram.dropped == 0


def test_histogram_percentile_labels_present_in_summary() -> None:
    """报告里必须同时出现 P50 / P99 / P99.9（口径标签固定）。"""
    histogram = MetricHistogram("demo", "s")
    histogram.add(0.001)
    summary = histogram.summary()
    for label in PERCENTILE_LABELS:
        assert label in summary
    assert summary["unit"] == "s"
    assert summary["count"] == 1


def test_histogram_empty_is_explicit() -> None:
    """空直方图：统计值为 ``None``（不是 0），分位数显式报错。"""
    histogram = MetricHistogram("demo", "s")
    assert histogram.count == 0
    assert histogram.min is None
    assert histogram.max is None
    assert histogram.mean is None
    assert histogram.p50 is None
    with pytest.raises(ValueError, match="至少一个样本"):
        histogram.percentile(50.0)


def test_histogram_rejects_non_finite_samples() -> None:
    """负例：NaN / Inf / 非数值样本一律显式报错并点名指标。"""
    histogram = MetricHistogram("jitter_probe", "s")
    with pytest.raises(ValueError, match="jitter_probe"):
        histogram.add(math.nan)
    with pytest.raises(ValueError, match="jitter_probe"):
        histogram.add(math.inf)
    with pytest.raises(ValueError, match="jitter_probe"):
        histogram.add("0.1")  # type: ignore[arg-type]
    assert histogram.count == 0


def test_histogram_capacity_overflow_is_counted_not_silent() -> None:
    """负例：超过容量 ⇒ 计数到 ``dropped``，**不静默丢弃**。"""
    histogram = MetricHistogram("demo", "s", max_samples=4, initial_capacity=2)
    for index in range(10):
        histogram.add(index / 1000.0)
    assert histogram.count == 4
    assert histogram.dropped == 6
    assert histogram.max == pytest.approx(3 / 1000.0)


def test_histogram_grows_without_losing_samples() -> None:
    """扩容（倍增）不得丢样本或错序。"""
    histogram = MetricHistogram("demo", "s", max_samples=64, initial_capacity=2)
    for index in range(40):
        histogram.add(index / 1000.0)
    assert histogram.count == 40
    assert histogram.dropped == 0
    assert histogram.sorted_samples() == pytest.approx(
        [index / 1000.0 for index in range(40)]
    )
    assert histogram.max == pytest.approx(39 / 1000.0)


def test_histogram_summary_ms_is_the_only_conversion_point() -> None:
    """毫秒换算只发生在 ``summary_ms``（秒口径保持不变）。"""
    histogram = MetricHistogram("demo", "s")
    histogram.add(0.0015)
    seconds = histogram.summary()
    millis = histogram.summary_ms()
    assert seconds["p50"] == pytest.approx(0.0015)
    assert millis["unit"] == "ms"
    assert millis["p50"] == pytest.approx(1.5)


def test_named_histograms_expose_stable_names() -> None:
    """抖动 / 延迟两个直方图的名字固定（运行日志按名字取）。"""
    jitter = JitterHistogram()
    latency = LatencyHistogram()
    assert jitter.name == "jitter"
    assert latency.name == "command_latency"
    assert jitter.unit == "s"
    assert latency.unit == "s"
    assert jitter.max_samples == DEFAULT_MAX_SAMPLES


def test_histogram_rejects_invalid_construction() -> None:
    """负例：非法构造参数。"""
    with pytest.raises(ValueError, match="name"):
        MetricHistogram("", "s")
    with pytest.raises(ValueError, match="max_samples"):
        MetricHistogram("demo", "s", max_samples=0)
    with pytest.raises(ValueError, match="initial_capacity"):
        MetricHistogram("demo", "s", initial_capacity=0)
    with pytest.raises(ValueError, match="initial_capacity"):
        MetricHistogram("demo", "s", initial_capacity=1.5)  # type: ignore[arg-type]
