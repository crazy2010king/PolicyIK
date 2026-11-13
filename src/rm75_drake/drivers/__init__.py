"""驱动层（S02-T07）：仿真 / 假驱动 / 真机共用的**同一套接口**。

分层
----
* :mod:`rm75_drake.drivers.base` —— 驱动协议（:class:`Driver`）、
  状态结构（:class:`DriverState`）、错误标签（:class:`DriverError`）、
  以及「驱动不得修改轨迹内容」的自检钩子。
* :mod:`rm75_drake.drivers.fake` —— :class:`FakeDriver`：
  消费 Planner 输出，按 5 ms 节拍模拟下发与回读（**零硬件、零仿真**）。
* :mod:`rm75_drake.drivers.sim` —— :class:`SimulationDriver`：
  同样的 5 ms 节拍，但状态**真的写进 Drake plant 再读回**。

同源保证
--------
仿真驱动与假驱动消费**同一条 Planner 输出**时，必须产出**逐点一致**的
关节序列（容差 1e-12）——这就是「同一 scenario YAML 驱动仿真与真机，
切换只发生在驱动实现层」（R007）的可执行证明。

本包**零 pydrake 依赖**（不 import 任何 pydrake 模块），
因此假驱动可以脱离 Drake 环境单独使用与测试。
"""

from __future__ import annotations

from .base import (
    PHASE_DONE,
    PHASE_IDLE,
    PHASE_STREAMING,
    REASON_CLOSED,
    REASON_DT_MISMATCH,
    REASON_EMPTY,
    REASON_INTEGRITY,
    REASON_INVALID_INPUT,
    REASON_NOT_OPEN,
    REASON_NO_TRAJECTORY,
    Driver,
    DriverError,
    DriverState,
)
from .fake import FakeDriver
from .sim import SimulationDriver

__all__ = [
    "Driver",
    "DriverError",
    "DriverState",
    "FakeDriver",
    "SimulationDriver",
    "PHASE_IDLE",
    "PHASE_STREAMING",
    "PHASE_DONE",
    "REASON_INVALID_INPUT",
    "REASON_EMPTY",
    "REASON_DT_MISMATCH",
    "REASON_NOT_OPEN",
    "REASON_NO_TRAJECTORY",
    "REASON_INTEGRITY",
    "REASON_CLOSED",
]
