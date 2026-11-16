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

T2 交付的是**时钟与栅格层**：绝对截止时间时钟（:mod:`rm_executor.clock`，
含真实 ``TickClock`` 与确定性 ``ManualClock``）与 10 ms 栅格重采样
（:mod:`rm_executor.resample`）。

T3 交付的是**传输与反馈抽象**：透传通道接口（:mod:`rm_executor.transport`，
含离线 ``FakeTransport`` 与真机 ``BridgeTransport``）与反馈源接口
（:mod:`rm_executor.feedback`，含 ``ScriptedFeedback`` / ``PollFeedback`` /
``UdpFeedback`` 与后台采集泵 ``FeedbackPump``）。

T4 交付的是**执行内核**：主循环（:mod:`rm_executor.executor`，阶段
预校验 → 预就位 → 10 ms tick 流式透传 → 到位判定 → 收尾）与统一报告
（:mod:`rm_executor.report`，fake 与 live 同一份 schema、同一份生成代码）。

T5 交付的是**反馈闭环异常判定**：跟随误差两档 / 断流两档 / 控制器抢轨迹
→ 全部转成**五级结构化事件**（S04 的 L1–L4 安全链路直接复用），控制器
状态查询由**独立看门狗线程**承担（绝不进 tick 线程）。
"""
from __future__ import annotations

from rm_executor import (
    clock,
    contract,
    events,
    executor,
    feedback,
    metrics,
    paths,
    report,
    resample,
    session,
    transport,
)

__all__ = [
    "__version__",
    "clock",
    "contract",
    "events",
    "executor",
    "feedback",
    "metrics",
    "paths",
    "report",
    "resample",
    "session",
    "transport",
]

__version__ = "0.1.0"
