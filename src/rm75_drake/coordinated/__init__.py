"""双臂紧协调规划层（S05）。

分层
----
* :mod:`rm75_drake.coordinated.limits` —— **双臂 14 维软件限位的程序派生**：
  同一台机器人 ⇒ 两臂逐项相同；真值来源仍是 S01 的单臂交接件
  ``models/rm75_drake/joint_limits.yaml``（**不得改该文件**）；派生后按
  plant 的实际 ``joint_names`` 顺序装配，并断言名字形如
  ``left_joint_i`` / ``right_joint_i``（S04 的重命名 URDF 变体，见
  ``scenarios/dual_arm.yaml``），不一致即显式报错
  （:class:`CoordinatedLimitError`，**点名期望 / 实际**）。
  另含**夹爪自由度口径**：装夹爪时 ``plant.num_positions() == 18``
  （两夹爪各 2 个 prismatic），本层只产出 14 维臂关节，查询碰撞前必须
  按约定补齐夹爪 DOF（:func:`pad_gripper_positions`）。
* :mod:`rm75_drake.coordinated.parameterization` —— **最小坐标参数化**：
  把「双臂相对位姿固定」从 14 维里的零测集约束流形降成 **8 维参数**
  （leader 7 关节 + follower ψ 1），并由 **leader FK + 常数相对变换 +
  follower 解析 IK** 的映射**构造性**保证相对位姿约束成立（残差 ~1e-15 m）。
* :mod:`rm75_drake.coordinated.trajectory` —— **共享时间轴**的双臂轨迹：
  **不新建 14 维轨迹类型**（S02 的 7 维 ``Trajectory`` 是冻结的交接面），
  而是把 14 维联合路径拆成两条 7 维 ``Trajectory`` 组成
  :class:`DualTrajectory`；构造期断言两者的 ``t_abs`` / ``dt``
  **逐点相同** ⇒ **共享时间轴由构造保证**，并复用 S02 的全部不变量校验
  （软限位 / 速度 / 加速度 / seq / 时间网格），**零重复实现**。

零 pydrake 依赖
---------------
本包的 ``limits`` / ``trajectory`` 两个模块只依赖 ``numpy`` +
:mod:`rm75_drake.planning.trajectory`（后者同样是 numpy-only），
**不 import pydrake**，也不通过 ``rm75_drake.models`` /
``rm75_drake.drake_compat`` 绕回 Drake；``parameterization`` 模块的
Drake 侧依赖（碰撞查询 / 基座位姿）一律**函数内惰性导入**。因此
``import rm75_drake.coordinated`` **不会**把 pydrake 拉进 ``sys.modules``
（PEP 562 惰性导入，仿 :mod:`rm75_drake.bimanual`）。
"""

from __future__ import annotations

import importlib
from typing import Any

__all__ = [
    "limits",
    "trajectory",
    "parameterization",
    "CoordinatedLimitError",
    "DualLimits",
    "derive_dual_limits",
    "pad_gripper_positions",
    "DualTrajectory",
    "dual_trajectory_from_arrays",
    "CoordinatedParameterization",
    "CoordinatedParameterizationError",
    "RelativePoseResidual",
    "ParameterizationDiagnostics",
]

#: 惰性子模块（PEP 562）：import 本包不拉起任何重依赖。
_LAZY_MODULES = ("limits", "trajectory", "parameterization")

#: 从 ``limits`` 子模块透出的符号。
_LIMITS_EXPORTS = (
    "CoordinatedLimitError",
    "DualLimits",
    "derive_dual_limits",
    "pad_gripper_positions",
)

#: 从 ``trajectory`` 子模块透出的符号。
_TRAJECTORY_EXPORTS = (
    "DualTrajectory",
    "dual_trajectory_from_arrays",
)

#: 从 ``parameterization`` 子模块透出的符号。
_PARAMETERIZATION_EXPORTS = (
    "CoordinatedParameterization",
    "CoordinatedParameterizationError",
    "RelativePoseResidual",
    "ParameterizationDiagnostics",
)


def __getattr__(name: str) -> Any:
    """PEP 562 惰性导入 / 符号透出。"""
    if name in _LAZY_MODULES:
        module = importlib.import_module(f"{__name__}.{name}")
        globals()[name] = module
        return module
    if name in _LIMITS_EXPORTS:
        module = importlib.import_module(f"{__name__}.limits")
        value = getattr(module, name)
        globals()[name] = value
        return value
    if name in _TRAJECTORY_EXPORTS:
        module = importlib.import_module(f"{__name__}.trajectory")
        value = getattr(module, name)
        globals()[name] = value
        return value
    if name in _PARAMETERIZATION_EXPORTS:
        module = importlib.import_module(f"{__name__}.parameterization")
        value = getattr(module, name)
        globals()[name] = value
        return value
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
