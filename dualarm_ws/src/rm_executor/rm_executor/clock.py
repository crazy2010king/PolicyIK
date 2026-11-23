"""rm_executor 时钟层：绝对截止时间栅格（10 ms 单时钟）。

S03 的核心承诺是「10 ms 单时钟」。本模块提供三个要素：

* :class:`TimeSource` —— 「当前时刻来源」接口（把时间来源从调度逻辑里抽出来）；
* :class:`TickClock` —— **绝对截止时间**调度：第 k 个 tick 的截止时刻
  ``start + k * period``（**不是**「睡 period 再累加」，后者必然漂移）；
* :class:`ManualClock` —— 确定性虚拟时钟，``sleep_until`` 只推进虚拟时间，
  用于单元测试（T4/T5 的主循环必须在 ManualClock 下可完全确定性重放）。

抖动口径：每次 tick 记录 **实际唤醒时刻 − 理想截止时刻**（毫秒），样本交给
:func:`rm_executor.metrics.jitter_stats` 统计。

调度优先级：:func:`enable_sched_fifo` **尽力而为**——本机容器实测返回 EPERM，
因此失败必须**静默降级**为普通调度，并把 ``sched_fifo=False`` 暴露给报告层，
绝不可作为硬依赖（否则真机现场一跑就炸）。
"""
from __future__ import annotations

import os
import time
from abc import ABC, abstractmethod
from typing import Any, Dict, List, Optional

__all__ = [
    "Clock",
    "ManualClock",
    "PerfTimeSource",
    "TickClock",
    "TimeSource",
    "VirtualTimeSource",
    "enable_sched_fifo",
]

#: 默认栅格周期（秒）—— 与 workcell/rm_workcell.yaml::execution.period_s 对齐
DEFAULT_PERIOD_S = 0.010


def enable_sched_fifo(prio: int = 80) -> bool:
    """尽力把当前进程切到 ``SCHED_FIFO`` 实时调度。

    返回 ``True`` 表示成功、``False`` 表示**已静默降级**为普通调度
    （容器 / 非 root 环境几乎必然 EPERM）。**永不抛异常**——实时调度只是
    锦上添花，栅格正确性由绝对截止时间保证，不依赖它。
    """
    try:
        os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(int(prio)))
        return True
    except (AttributeError, OSError, PermissionError, ValueError):
        # 平台不支持 / 权限不足 / 参数非法 —— 一律降级，不阻断执行
        return False


class TimeSource(ABC):
    """「当前时刻来源」接口。

    抽成接口的目的：**主循环代码路径在 ManualClock 与 TickClock 之间完全一致**
    （T4/T5 的执行主循环只依赖本接口），测试可确定性重放，现场跑真实时钟。
    """

    @abstractmethod
    def now(self) -> float:
        """返回当前时刻（秒，单调时钟域）。"""


class PerfTimeSource(TimeSource):
    """真实墙钟来源：``time.perf_counter``（单调，不受系统时间调整影响）。"""

    def now(self) -> float:
        return time.perf_counter()


class VirtualTimeSource(TimeSource):
    """虚拟时间来源：只由 :meth:`advance_to` 推进，测试用。"""

    def __init__(self, start: float = 0.0) -> None:
        self._now = float(start)

    def now(self) -> float:
        return self._now

    def advance_to(self, t: float) -> float:
        """把虚拟时间推进到 ``t``（只进不退）。"""
        t = float(t)
        if t < self._now:
            raise ValueError(
                "虚拟时间不可回退（current=%r, target=%r）" % (self._now, t)
            )
        self._now = t
        return self._now


class Clock(ABC):
    """栅格时钟基类：绝对截止时间 + 抖动样本。

    子类必须实现 :meth:`now` 与 :meth:`sleep_until`。
    """

    def __init__(self, period_s: float = DEFAULT_PERIOD_S, start: Optional[float] = None) -> None:
        period = float(period_s)
        if not (period > 0.0):
            raise ValueError("period_s 必须为正数（当前 %r）" % (period_s,))
        self.period_s = period
        self._tick_index = 0
        self._start = self.now() if start is None else float(start)
        self.jitter_samples_ms: List[float] = []
        #: 是否已启用 SCHED_FIFO 实时调度（默认 False，调用方可后置启用）
        self.sched_fifo: bool = False

    # -- 时间来源 ---------------------------------------------------------
    @abstractmethod
    def now(self) -> float:
        """当前时刻（秒）。"""

    @abstractmethod
    def sleep_until(self, deadline: float) -> float:
        """睡到绝对截止时刻 ``deadline``；返回实际唤醒时刻。"""

    # -- 栅格 -------------------------------------------------------------
    @property
    def start_time(self) -> float:
        """栅格起点（第 0 个 tick 的理想截止时刻）。"""
        return self._start

    def deadline_for(self, k: int) -> float:
        """第 ``k`` 个 tick 的**理想绝对截止时刻** = ``start + k * period``。"""
        k = int(k)
        if k < 0:
            raise ValueError("tick 序号不可为负（当前 %r）" % (k,))
        return self._start + k * self.period_s

    def next_deadline(self) -> float:
        """下一个 tick 的理想截止时刻（不推进计数）。"""
        return self.deadline_for(self._tick_index)

    # -- tick -------------------------------------------------------------
    def tick(self) -> float:
        """执行一个栅格 tick：睡到截止时刻，返回并记录抖动样本（毫秒）。

        抖动 = 实际唤醒时刻 − 理想截止时刻。虚拟时钟下恒为 ``0.0``。

        语义：执行第 k 个 tick（k 从 0 计）后 ``now() == start + k * period``，
        且 ``next_deadline() == start + (k+1) * period``。
        """
        deadline = self.deadline_for(self._tick_index)
        wake = self.sleep_until(deadline)
        jitter_ms = (wake - deadline) * 1000.0
        self.jitter_samples_ms.append(jitter_ms)
        self._tick_index += 1
        return jitter_ms

    @property
    def tick_index(self) -> int:
        """已执行的 tick 数。"""
        return self._tick_index

    def stats(self) -> Dict[str, Any]:
        """抖动统计（委托给 metrics 层口径，空样本 → None）。"""
        from rm_executor import metrics

        return metrics.jitter_stats(self.jitter_samples_ms)


class TickClock(Clock):
    """真实墙钟 + 绝对截止时间调度。

    第 k 个 tick 的截止时刻固定为 ``start + k * period``，**不因上一 tick 的
    超时而顺延**（顺延即漂移，会破坏「单时钟」语义）。若某 tick 超期
    （``sleep_until`` 醒来时已过截止时刻），下个 tick 依然按绝对栅格走，
    超期量会体现在抖动样本上。
    """

    def __init__(
        self,
        period_s: float = DEFAULT_PERIOD_S,
        source: Optional[TimeSource] = None,
        start: Optional[float] = None,
        enable_fifo: bool = False,
        fifo_prio: int = 80,
    ) -> None:
        self._source = source or PerfTimeSource()
        super().__init__(period_s=period_s, start=start)
        if enable_fifo:
            self.sched_fifo = enable_sched_fifo(fifo_prio)

    def now(self) -> float:
        return self._source.now()

    def sleep_until(self, deadline: float) -> float:
        """睡到绝对截止时刻，返回实际唤醒时刻（秒）。

        使用 ``time.sleep`` 主动让出 CPU；醒来后若已过截止时刻，**不再补睡**
        （补睡等于漂移）。
        """
        remaining = deadline - self.now()
        if remaining > 0.0:
            time.sleep(remaining)
        return self.now()


class ManualClock(Clock):
    """确定性虚拟时钟：``sleep_until`` 只推进虚拟时间，不真实等待。

    主循环（含执行、采样、事件判定）在 ManualClock 下可**逐 tick 完全重放**，
    测试断言因此是确定性的，不依赖真实机器负载。
    """

    def __init__(
        self,
        period_s: float = DEFAULT_PERIOD_S,
        source: Optional[VirtualTimeSource] = None,
        start: float = 0.0,
    ) -> None:
        self._source = source or VirtualTimeSource(start)
        super().__init__(period_s=period_s, start=start)

    @property
    def source(self) -> VirtualTimeSource:
        return self._source

    def now(self) -> float:
        return self._source.now()

    def sleep_until(self, deadline: float) -> float:
        """把虚拟时间推进到 ``deadline``（不真实等待）。"""
        return self._source.advance_to(deadline)
