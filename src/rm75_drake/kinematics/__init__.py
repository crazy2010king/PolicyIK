"""RM75 运动学层（S01-T03 / S05-T01）。

**零 Drake 依赖**：本子包只依赖 numpy，导入它不会拉起 pydrake。
这样 :mod:`rm75_drake.kinematics.mdh` 才能作为 Drake FK 的独立被测对照
（见模块 docstring 中关于官方 rpy=1.5708 舍入残差的说明）。

:mod:`rm75_drake.kinematics.analytic_ik` 沿用同一隔离约定：纯 numpy 的解析
逆运动学（分支枚举 + 肘点方位角 ψ），不 import pydrake。
"""

from __future__ import annotations

from . import analytic_ik
from .analytic_ik import (
    AnalyticIkResult,
    Solution,
    analytic_solutions,
    solve_analytic_ik,
)
from .mdh import (
    MDH_A,
    MDH_ALPHA,
    MDH_ALPHA_EXACT_PI_2,
    MDH_D,
    MDH_ZERO_HEIGHT_M,
    N_JOINTS,
    fk,
    link7_position,
    link7_rotation,
)

__all__ = [
    "AnalyticIkResult",
    "MDH_A",
    "MDH_ALPHA",
    "MDH_ALPHA_EXACT_PI_2",
    "MDH_D",
    "MDH_ZERO_HEIGHT_M",
    "N_JOINTS",
    "Solution",
    "analytic_ik",
    "analytic_solutions",
    "fk",
    "link7_position",
    "link7_rotation",
    "solve_analytic_ik",
]
