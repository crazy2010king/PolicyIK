"""rmctl 工艺编排层（取放工艺的声明式规格 + 状态机 + trace）。

本包只做**包声明**，不放任何逻辑：

* :mod:`rmctl.process.spec` —— 工艺任务规格的加载与校验（唯一入口）。
* :mod:`rmctl.process.clients` —— 工艺客户端抽象（Planner / MotionClient /
  GripperClient 协议 + fake / live 实现）。

后续的段状态机 / 客户端 / trace / CLI 都建立在本包之上，但**规格的真相
只有一处**（``workcell/tasks/*.yaml``），不得在别处另立一份隐式约定。
"""
from __future__ import annotations

__all__ = [
    "clients",
    "spec",
]
