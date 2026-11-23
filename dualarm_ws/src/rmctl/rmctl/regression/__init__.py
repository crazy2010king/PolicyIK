"""一键回归的**段级判定模型**（rmctl 体验层）。

本包只提供**纯函数**：段级 verdict 词表、verdict → status / 退出码映射，
以及统一报告骨架的构造器。**不**含任何 I/O、硬件访问或厂商依赖。

状态与退出码口径**唯一来源** = ``rm_executor.contract``（执行层契约）。
"""
from __future__ import annotations

from rmctl.regression.segments import (
    MODES,
    VERDICTS,
    aggregate,
    segment_result,
    verdict_to_exit_code,
    verdict_to_status,
)

__all__ = [
    "MODES",
    "VERDICTS",
    "aggregate",
    "segment_result",
    "verdict_to_exit_code",
    "verdict_to_status",
]
