"""规划层（S02）：与仿真/真机同源的 Planner API。

本包里的所有模块都消费同一个 :class:`rm75_drake.scenario.Scenario`（场景真相源），
保证「同一 scenario YAML 驱动仿真与真机，切换只发生在驱动实现层」（R007）。

子模块
------
``scene``
    scenario → Drake 场景（plant / scene_graph / 碰撞检查器）的装配层。
``trajectory``
    轨迹数据结构与不变量（S02-T02）：5 ms 网格 / 单调 seq / 绝对时刻 / 软限位与速度限值。
``ik``
    逆运动学（S02-T03）：DiffIK 风格求解 + 名义姿态 + 软限位复核 + 可解释失败。
``regions`` / ``gcs``
    C 空间凸区域（IRIS，S02-T04）与**空间凸集图最短路**（S02-T05）。
    注意：``gcs`` 是**空间凸集图最短路**，**不是**轨迹级 GCS（见该模块 docstring）。
``retime``
    S02-T06 时间参数化（Toppra 重定时 + 5 ms 网格采样 + C2 连续与限值复核）。

**惰性导入**：本包 ``__init__`` 不在 import 时拉起 pydrake（与
:mod:`rm75_drake` 顶层包的既有约定一致），子模块按需导入。
"""

from __future__ import annotations

from typing import Any

__all__ = ["scene", "trajectory", "ik", "regions", "gcs", "retime"]


def __getattr__(name: str) -> Any:
    """PEP 562 惰性子模块导入（避免 ``import rm75_drake.planning`` 就拉起 pydrake）。"""
    if name in __all__:
        import importlib

        module = importlib.import_module(f"{__name__}.{name}")
        globals()[name] = module
        return module
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
