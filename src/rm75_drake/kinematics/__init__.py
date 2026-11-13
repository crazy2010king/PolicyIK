"""RM75 运动学层（S01-T03）。

**零 Drake 依赖**：本子包只依赖 numpy，导入它不会拉起 pydrake。
这样 :mod:`rm75_drake.kinematics.mdh` 才能作为 Drake FK 的独立被测对照
（见模块 docstring 中关于官方 rpy=1.5708 舍入残差的说明）。
"""

from __future__ import annotations

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
    "MDH_A",
    "MDH_ALPHA",
    "MDH_ALPHA_EXACT_PI_2",
    "MDH_D",
    "MDH_ZERO_HEIGHT_M",
    "N_JOINTS",
    "fk",
    "link7_position",
    "link7_rotation",
]
