"""任务层（S06）：任务 schema、原子目标与（后续）可组合任务原语。

分层
----
* :mod:`rm75_drake.tasks.schema` —— **任务 / 物体 / 原语**的正式 schema 与严格校验
  （未知顶层键 / 未知子键 / 未知原语名 / 缺字段 / 维度错 / 单位错 ⇒
  :class:`rm75_drake.tasks.schema.TaskError`，**点名该键**）。
* :mod:`rm75_drake.tasks.goals` —— **原子目标**类型：:class:`TaskGoal` /
  :class:`PoseGoal` / :class:`RelativePoseGoal` / :class:`ObjectGoal`
  （``goal_id`` 非空、任务内唯一）。
* :mod:`rm75_drake.tasks.primitives` —— **原语目录**（10 个原语的静态元数据
  与默认误差门槛）。
* :mod:`rm75_drake.tasks.solver` —— **单一求解器入口**
  （:func:`~rm75_drake.tasks.solver.solve_task`：numpy 快路径）。
* :mod:`rm75_drake.tasks.state_machine` —— **任务状态机**
  （单一主状态源 + 原子 goal_id 推进；零 pydrake 依赖）。
* :mod:`rm75_drake.tasks.perception` —— **感知适配接口**（S06-T06）：
  非阻塞离散事件（回调 + 有界队列 + ``poll``）+ 保守场景表示
  （包围盒按 ``base_margin + 速率 × 延迟`` 膨胀）+ **重建碰撞检查器**
  （旧 checker 看不见新注册的几何 ⇒ 不重建等于保守场景静默失效）。
* :mod:`rm75_drake.tasks.bimanual` —— **双臂搬运 / 交接原语**（S06-T05）：
  复用 :mod:`rm75_drake.coordinated` 的**常数 ``X_LR``**（角色互换用
  ``inv(X_LR)``）⇒ **不新建第二套协调规划**；交接点的相对位姿残差
  ≤ 1e-6 m / ≤ 1e-6 rad。
* （后续切片）:mod:`rm75_drake.tasks.metrics` —— 原语级指标口径。

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
    "PRIMITIVE_SPECS",
    "primitive_spec",
    "primitive_names",
    "DEFAULT_POSITION_TOLERANCE_M",
    "DEFAULT_ORIENTATION_TOLERANCE_RAD",
    "PrimitiveCatalogSpec",
    "solve_task",
    "solve_primitive",
    "SolverOptions",
    "SolveState",
    "PrimitiveResult",
    "TaskResult",
    "state_machine",
    "bimanual",
    "perception",
    "PerceptionError",
    "PerceptionObservation",
    "PerceptionPolicy",
    "PerceptionSnapshot",
    "PerceptionBus",
    "PerceptionSceneResult",
    "apply_perception_scene",
    "TaskState",
    "TERMINAL_STATES",
    "TASK_STAGES",
    "TASK_REASONS",
    "TaskFailure",
    "StateTransition",
    "TaskStateMachine",
]

#: 惰性子模块（PEP 562）：import 本包不拉起任何重依赖。
_LAZY_MODULES = (
    "schema",
    "goals",
    "primitives",
    "solver",
    "state_machine",
    "bimanual",
    "perception",
)

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

#: 从 ``primitives`` 子模块透出的符号。
#:
#: 注意：:class:`rm75_drake.tasks.primitives.PrimitiveSpec`（**原语目录**条目）
#: 与 :class:`rm75_drake.tasks.schema.PrimitiveSpec`（**任务文件里的原语条目**）
#: 是两个不同的类型；包级 ``PrimitiveSpec`` 沿用 schema 的那个（不变），
#: 原语目录的那个在本包以 ``PrimitiveCatalogSpec`` 暴露，避免同名歧义。
_PRIMITIVES_EXPORTS = (
    "PRIMITIVE_SPECS",
    "primitive_spec",
    "primitive_names",
    "DEFAULT_POSITION_TOLERANCE_M",
    "DEFAULT_ORIENTATION_TOLERANCE_RAD",
    "PrimitiveCatalogSpec",
)

#: 从 ``solver`` 子模块透出的符号。
_SOLVER_EXPORTS = (
    "solve_task",
    "solve_primitive",
    "SolverOptions",
    "SolveState",
    "PrimitiveResult",
    "TaskResult",
)

#: 从 ``state_machine`` 子模块透出的符号。
_STATE_MACHINE_EXPORTS = (
    "TaskState",
    "TERMINAL_STATES",
    "TASK_STAGES",
    "TASK_REASONS",
    "TaskFailure",
    "StateTransition",
    "TaskStateMachine",
)

#: 从 ``perception`` 子模块透出的符号。
_PERCEPTION_EXPORTS = (
    "PerceptionError",
    "PerceptionObservation",
    "PerceptionPolicy",
    "PerceptionSnapshot",
    "PerceptionBus",
    "PerceptionSceneResult",
    "apply_perception_scene",
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
    if name in _PERCEPTION_EXPORTS:
        module = importlib.import_module(f"{__name__}.perception")
        value = getattr(module, name)
        globals()[name] = value
        return value
    if name in _GOALS_EXPORTS:
        module = importlib.import_module(f"{__name__}.goals")
        value = getattr(module, name)
        globals()[name] = value
        return value
    if name in _PRIMITIVES_EXPORTS:
        module = importlib.import_module(f"{__name__}.primitives")
        value = getattr(module, name)
        globals()[name] = value
        return value
    if name in _SOLVER_EXPORTS:
        module = importlib.import_module(f"{__name__}.solver")
        value = getattr(module, name)
        globals()[name] = value
        return value
    if name in _STATE_MACHINE_EXPORTS:
        module = importlib.import_module(f"{__name__}.state_machine")
        value = getattr(module, name)
        globals()[name] = value
        return value
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
