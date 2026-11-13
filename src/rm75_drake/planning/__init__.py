"""规划层（S02）：与仿真/真机同源的 Planner API。

本包里的所有模块都消费同一个 :class:`rm75_drake.scenario.Scenario`（场景真相源），
保证「同一 scenario YAML 驱动仿真与真机，切换只发生在驱动实现层」（R007）。

子模块
------
``scene``
    scenario → Drake 场景（plant / scene_graph / 碰撞检查器）的装配层。
``trajectory`` / ``ik`` / ``regions`` / ``gcs`` / ``retime``
    S02-T02 ~ S02-T06 依次落地（见切片计划）。

**惰性导入**：本包 ``__init__`` 不在 import 时拉起 pydrake（与
:mod:`rm75_drake` 顶层包的既有约定一致），子模块按需导入。
"""

from __future__ import annotations

from typing import Any

__all__ = ["scene"]


def __getattr__(name: str) -> Any:
    """PEP 562 惰性子模块导入（避免 ``import rm75_drake.planning`` 就拉起 pydrake）。"""
    if name in __all__:
        import importlib

        module = importlib.import_module(f"{__name__}.{name}")
        globals()[name] = module
        return module
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
