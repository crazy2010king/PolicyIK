"""rmctl 侧规划服务契约层（设计 §4.3 / D001 / D004）。

本包只做**包声明**，逻辑在两个子模块：

* :mod:`rmctl.planner_service.contract` —— 内部 IDL（``PlanRequest`` /
  ``PlanResponse``）、字段级校验、轨迹摘要（确定性验收用）。
* :mod:`rmctl.planner_service.config` —— 声明式配置的**只读**加载器
  （``workcell/planner/pipeline.yaml`` 与 ``workcell/planner/profiles.yaml``）。
* :mod:`rmctl.planner_service.client` —— 规划服务的**薄客户端**
  （Unix socket 控制面；失败一律结构化，不抛裸异常）。

**唯一来源纪律（D001）**：管线 / profile / 场景参数只由 ``workcell/planner/*.yaml``
决定；Python 只读不改，**绝不内嵌第二份默认值**。

**三进程模型（D004）**：``rm_runtime`` / ``rm_planner`` / ``rmctl``。
本包属于 ``rm_planner`` 侧的契约真相，供服务端与薄客户端（``rmctl``）共用，
因此放在 ``rmctl`` 包内**不依赖**任何厂商规划内核（零 SDK、零 I/O）。
"""
from __future__ import annotations

__all__ = [
    "PROCESS_ROLE",
    "client",
    "config",
    "contract",
]

#: 进程角色名（D004 三进程模型之一）：规划服务进程的角色标识。
PROCESS_ROLE = "rm_planner"
