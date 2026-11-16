"""任务层（S06）：任务 schema、原子目标与（后续）可组合任务原语。

分层
----
* :mod:`rm75_drake.tasks.schema` —— **任务 / 物体 / 原语**的正式 schema 与严格校验
  （未知顶层键 / 未知子键 / 未知原语名 / 缺字段 / 维度错 / 单位错 ⇒
  :class:`rm75_drake.tasks.schema.TaskError`，**点名该键**）。
* :mod:`rm75_drake.tasks.goals` —— **原子目标**类型：:class:`TaskGoal` /
  :class:`PoseGoal` / :class:`RelativePoseGoal` / :class:`ObjectGoal`
  （``goal_id`` 非空、任务内唯一）。
* （后续切片）:mod:`rm75_drake.tasks.primitives` / :mod:`rm75_drake.tasks.state_machine`
  / :mod:`rm75_drake.tasks.bimanual` / :mod:`rm75_drake.tasks.metrics`
  —— 原语库、状态机、双臂搬运/交接与原语级指标口径。

目录语义（**同名易混，必须分清**）
--------------------------------

* ``src/rm75_drake/tasks/`` → **本 Python 包**（代码）；
* ``<repo root>/tasks/``      → **部署实例数据**（``*.yaml`` 任务定义），
  路径常量是 :data:`rm75_drake.paths.task_files_dir`。
  （与 ``calibration_dir`` / ``calibration_files_dir`` 的分层约定同款。）

零 pydrake 依赖
---------------
本包采用 **PEP 562 惰性导入**（仿 :mod:`rm75_drake.coordinated`）：
``import rm75_drake.tasks`` **不会**把 pydrake 拉进 ``sys.modules``，
因此任务 schema / 目标解析可离线单测、不被 Drake 版本绑死。
"""

from __future__ import annotations

import importlib
from typing import Any

__all__ = [
    "schema",
    "goals",
    "TaskError",
    "GoalError",
    "SCHEMA_VERSION",
    "TOP_LEVEL_KEYS",
    "REQUIRED_TOP_LEVEL_KEYS",
    "PRIMITIVE_NAMES",
    "PRIMITIVE_KINDS",
    "GRIPPER_ACTIONS",
    "PoseSpec",
    "Tolerance",
    "ObjectSpec",
    "PrimitiveSpec",
    "TaskSpec",
    "parse_task",
    "load_task_file",
    "goal_for_primitive",
    "TaskGoal",
    "PoseGoal",
    "RelativePoseGoal",
    "ObjectGoal",
    "GoalRegistry",
    "build_goal_registry",
    "GOAL_KINDS",
]

#: 惰性子模块（PEP 562）：import 本包不拉起任何重依赖。
_LAZY_MODULES = ("schema", "goals")

#: 从 ``schema`` 子模块透出的符号。
_SCHEMA_EXPORTS = (
    "TaskError",
    "GoalError",
    "SCHEMA_VERSION",
    "TOP_LEVEL_KEYS",
    "REQUIRED_TOP_LEVEL_KEYS",
    "PRIMITIVE_NAMES",
    "PRIMITIVE_KINDS",
    "GRIPPER_ACTIONS",
    "PoseSpec",
    "Tolerance",
    "ObjectSpec",
    "PrimitiveSpec",
    "TaskSpec",
    "parse_task",
    "load_task_file",
    "goal_for_primitive",
)

#: 从 ``goals`` 子模块透出的符号。
_GOALS_EXPORTS = (
    "GOAL_KINDS",
    "TaskGoal",
    "PoseGoal",
    "RelativePoseGoal",
    "ObjectGoal",
    "GoalRegistry",
    "build_goal_registry",
)


def __getattr__(name: str) -> Any:
    """PEP 562 惰性导入 / 符号透出。"""
    if name in _LAZY_MODULES:
        module = importlib.import_module(f"{__name__}.{name}")
        globals()[name] = module
        return module
    if name in _SCHEMA_EXPORTS:
        module = importlib.import_module(f"{__name__}.schema")
        value = getattr(module, name)
        globals()[name] = value
        return value
    if name in _GOALS_EXPORTS:
        module = importlib.import_module(f"{__name__}.goals")
        value = getattr(module, name)
        globals()[name] = value
        return value
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
