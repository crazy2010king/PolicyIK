"""rm_executor —— 10 ms 单时钟执行器（执行层）。

本包负责「把一条经四道校验的关节轨迹，按 10 ms 绝对截止时间栅格连续透传到
真机，并用反馈闭环判定到位与异常」所需的**执行层**能力，进程角色
``rm_runtime``。

分层约束（与 ``rm_hw_bridge/contract.yaml: rules`` 一致）：

* **不 import** ``rmctl``（体验层不得被反向依赖）；
* **不 import** ``tesseract*``（不依赖规划内核）；
* 只允许依赖 ``rm_hw_bridge``（硬件层）与 ``pyyaml``（可选 numpy）；
* 零编译（纯 Python 包）。

T1 交付的是**执行契约骨架**：轨迹 JSON 契约（:mod:`rm_executor.contract`）、
五级事件模型（:mod:`rm_executor.events`）、指标口径
（:mod:`rm_executor.metrics`）与工作区路径解析（:mod:`rm_executor.paths`）。
"""
from __future__ import annotations

from rm_executor import contract, events, metrics, paths

__all__ = [
    "__version__",
    "contract",
    "events",
    "metrics",
    "paths",
]

__version__ = "0.1.0"
