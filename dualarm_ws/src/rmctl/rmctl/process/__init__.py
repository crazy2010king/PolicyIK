"""rmctl 工艺编排层（取放工艺的声明式规格 + 状态机 + trace）。

本包只做**包声明**，不放任何逻辑：

* :mod:`rmctl.process.spec` —— 工艺任务规格的加载与校验（唯一入口）。
* :mod:`rmctl.process.clients` —— 工艺客户端抽象（Planner / MotionClient /
  GripperClient 协议 + fake / live 实现）。
* :mod:`rmctl.process.runner` —— 工艺段状态机（段执行 → 夹爪判定 →
  恢复阶梯 → 失败分级 → 循环控制）。
* :mod:`rmctl.process.trace` —— 工艺 trace 的记录与回放（JSONL 一行一事件
  + 只读回放；**回放零运动、零 SDK、零网络**）。

后续的 CLI / 真机会话都建立在本包之上，但**规格的真相只有一处**
（``workcell/tasks/*.yaml``），不得在别处另立一份隐式约定。
"""
from __future__ import annotations

__all__ = [
    "clients",
    "runner",
    "spec",
    "trace",
]
