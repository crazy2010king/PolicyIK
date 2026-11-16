"""双臂 14 维软件限位的**程序派生**（S05-T02）。

为什么是「程序派生」而不是「再写一份 YAML」
------------------------------------------
双臂是**同一台机器人 RM75-B 的两份重命名 URDF 变体**（S04-T01 派生自
S01 的官方改造版 URDF，**只改名、不改数值**），因此两臂的限位**逐项相同**。
若在仓库里另写一份 ``joint_limits.yaml``，就会出现**两个真相源**，一旦官方
参数表更新就会静默漂移——这正是 S01 用「交接件 + 校验脚本」消灭的问题。

因此本模块的做法是：

1. **读取** S01 的单臂真值 ``models/rm75_drake/joint_limits.yaml``
   （**该文件不被本模块修改**，也不复制其数值）；
2. **程序派生** 14 维限位 = 左臂 7 维 + 右臂 7 维（逐项复制）；
3. **按 plant 的实际 ``joint_names`` 顺序装配**，并断言名字形如
   ``left_joint_i`` / ``right_joint_i``（i = 1..7）——名字错配（改前缀 /
   改顺序 / 少一个）一律**显式报错**，错误信息**点名期望与实际**
   （:class:`CoordinatedLimitError`），**不静默纠正**。

数值的来源（**不另起数值**）
--------------------------
* **位置软限位 / 速度上限**：:func:`rm75_drake.planning.trajectory.software_limits`
  （读 ``models/rm75_drake/joint_limits.yaml`` 的 ``software`` 段）；
* **加速度上限**：:data:`rm75_drake.planning.trajectory.DEFAULT_ACCELERATION_LIMITS_RAD_S2`
  （J1/J2 = 6.0、J3–J7 = 8.0 rad/s²，**本项目自研默认值**，官方 RM75 参数表
  未公布加速度上限）——双臂沿用同一组数值，**不另起一套**。

夹爪自由度口径（S04 继承，**必须显式**）
----------------------------------------
场景装夹爪时 ``plant.num_positions() == 18``（双臂 14 + 两夹爪各 2 个
prismatic）。本层（协调规划）**只产出 14 维臂关节**，因此在把 14 维联合
配置交给 Drake 做碰撞查询**之前**，必须按 plant 的位置变量顺序补齐夹爪 DOF。

plant 的位置变量顺序 = model instance 顺序 =
**左臂 7 → 右臂 7 → 左夹爪 2 → 右夹爪 2**
（见 :mod:`rm75_drake.bimanual.dual_scene` 的装配顺序：两臂先 ``AddModels``，
再装两夹爪）。因此 :func:`pad_gripper_positions` 按
「左夹爪 2 个 prismatic → 右夹爪 2 个 prismatic」补齐。

S04 的 ``include_grippers_in_collision`` 默认 ``False``：夹爪默认**不**进入
碰撞判定；一旦置 ``True``，q **必须**包含这 4 个 prismatic（否则夹爪几何
不随 q 更新，检不出「手指插进物体」）。

零 pydrake 依赖
---------------
本模块只依赖 ``numpy`` + :mod:`rm75_drake.planning.trajectory`（均为
numpy-only），**不 import pydrake**，也不通过 ``rm75_drake.models`` /
``rm75_drake.drake_compat`` 绕回 Drake。
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Sequence, Tuple

import numpy as np

from ..planning.trajectory import (
    DEFAULT_ACCELERATION_LIMITS_RAD_S2,
    JOINT_COUNT,
    software_limits,
)

__all__ = [
    "CoordinatedLimitError",
    "ARM_JOINT_COUNT",
    "DUAL_JOINT_COUNT",
    "JOINT_NAME_PATTERN",
    "DualLimits",
    "derive_dual_limits",
    "canonical_joint_names",
    "pad_gripper_positions",
]

#: 单臂关节数（7 DoF）。
ARM_JOINT_COUNT = JOINT_COUNT

#: 双臂联合关节数（14 DoF）。
DUAL_JOINT_COUNT = 2 * ARM_JOINT_COUNT

#: 合法的双臂关节名形态：``left_joint_1`` .. ``left_joint_7`` /
#: ``right_joint_1`` .. ``right_joint_7``。
JOINT_NAME_PATTERN = r"^(left|right)_joint_([1-7])$"

_NAME_RE = re.compile(JOINT_NAME_PATTERN)

#: 规范顺序：左臂 7 关节在前、右臂 7 关节在后（各臂内部 joint_1..joint_7）。
CANONICAL_SIDES: Tuple[str, ...] = ("left",) * ARM_JOINT_COUNT + ("right",) * ARM_JOINT_COUNT


class CoordinatedLimitError(RuntimeError):
    """双臂协调层的限位装配失败。

    错误信息统一带一个**稳定标签**，供调用方与测试按标签分类：

    ``[joint_count]``（维度不符）/ ``[joint_names]``（名字形态或顺序不符）/
    ``[limits]``（数值非法）/ ``[gripper]``（夹爪 DOF 补齐口径不合法）。
    """


@dataclass(frozen=True)
class DualLimits:
    """双臂 14 维的软件限位（程序派生自 S01 的单臂真值）。

    字段
    ----
    joint_names:
        装配所依据的 14 个关节名（顺序 = plant 的实际顺序）。
    lower / upper:
        逐轴位置软限位（rad，长度 14）。
    velocity:
        逐轴速度上限（rad/s，长度 14）。
    acceleration:
        逐轴加速度上限（rad/s²，长度 14，**自研默认值**）。
    sides:
        每一轴归属的侧别（``"left"`` / ``"right"``，长度 14）。
    """

    joint_names: Tuple[str, ...]
    lower: Tuple[float, ...]
    upper: Tuple[float, ...]
    velocity: Tuple[float, ...]
    acceleration: Tuple[float, ...]
    sides: Tuple[str, ...]

    def __post_init__(self) -> None:
        size = len(self.joint_names)
        for name, values in (
            ("lower", self.lower),
            ("upper", self.upper),
            ("velocity", self.velocity),
            ("acceleration", self.acceleration),
            ("sides", self.sides),
        ):
            if len(values) != size:
                raise CoordinatedLimitError(
                    f"[joint_count] {name} 的长度必须是 {size}（与 joint_names 一致），"
                    f"实际 {len(values)}"
                )
        if size != DUAL_JOINT_COUNT:
            raise CoordinatedLimitError(
                f"[joint_count] 双臂限位必须是 {DUAL_JOINT_COUNT} 维，实际 {size} 维"
            )

    # -- 便捷访问 -------------------------------------------------------
    @property
    def num_joints(self) -> int:
        """维度（14）。"""
        return len(self.joint_names)

    def side_slice(self, side: str) -> slice:
        """返回某侧（``"left"`` / ``"right"``）在 14 维中的切片。"""
        if side not in ("left", "right"):
            raise CoordinatedLimitError(
                f"[joint_names] 侧别必须是 'left' / 'right'，实际：{side!r}"
            )
        return slice(0, ARM_JOINT_COUNT) if side == "left" else slice(
            ARM_JOINT_COUNT, DUAL_JOINT_COUNT
        )

    def lower_array(self) -> np.ndarray:
        """位置软限位下限（numpy，形状 ``(14,)``）。"""
        return np.asarray(self.lower, dtype=float)

    def upper_array(self) -> np.ndarray:
        """位置软限位上限（numpy，形状 ``(14,)``）。"""
        return np.asarray(self.upper, dtype=float)

    def velocity_array(self) -> np.ndarray:
        """速度上限（numpy，形状 ``(14,)``）。"""
        return np.asarray(self.velocity, dtype=float)

    def acceleration_array(self) -> np.ndarray:
        """加速度上限（numpy，形状 ``(14,)``）。"""
        return np.asarray(self.acceleration, dtype=float)


def canonical_joint_names() -> Tuple[str, ...]:
    """返回规范的双臂关节名序列（``left_joint_1..7`` → ``right_joint_1..7``）。"""
    return tuple(
        f"{side}_joint_{index + 1}"
        for side in ("left", "right")
        for index in range(ARM_JOINT_COUNT)
    )


def _parse_joint_name(name: str) -> Tuple[str, int]:
    """解析 ``left_joint_3`` → ``("left", 3)``；形态不符 → 抛错（点名实际值）。"""
    match = _NAME_RE.match(name)
    if match is None:
        raise CoordinatedLimitError(
            f"[joint_names] 关节名必须形如 left_joint_i / right_joint_i（i=1..7），"
            f"实际：{name!r}"
        )
    return match.group(1), int(match.group(2))


def derive_dual_limits(joint_names: Sequence[str]) -> DualLimits:
    """由 plant 的实际关节名派生双臂 14 维软件限位。

    参数
    ----
    joint_names:
        plant 的**实际**关节名序列（长度必须是 14；顺序必须是
        ``left_joint_1..left_joint_7`` → ``right_joint_1..right_joint_7``）。
        该序列通常来自
        :attr:`rm75_drake.bimanual.dual_scene.DualSceneBundle.joint_names`。

    返回
    ----
    :class:`DualLimits`

    失败（全部抛 :class:`CoordinatedLimitError`，**点名期望 / 实际**）
    --------------------------------------------------------------
    * ``[joint_count]`` —— 关节名数量不是 14；
    * ``[joint_names]`` —— 名字形态不符（既不是 ``left_joint_i`` 也不是
      ``right_joint_i``）、或顺序错配（左臂不在前 / 臂内序号乱序）。
    """
    names = tuple(str(name) for name in joint_names)
    if len(names) != DUAL_JOINT_COUNT:
        raise CoordinatedLimitError(
            f"[joint_count] 双臂限位需要 {DUAL_JOINT_COUNT} 个关节名，"
            f"实际 {len(names)} 个：{list(names)}"
        )

    parsed = [_parse_joint_name(name) for name in names]
    sides = tuple(side for side, _ in parsed)
    indices = tuple(index for _, index in parsed)
    if sides != CANONICAL_SIDES:
        raise CoordinatedLimitError(
            f"[joint_names] 双臂关节顺序必须是 "
            f"{list(canonical_joint_names())}，实际：{list(names)}"
        )
    if indices != tuple(list(range(1, ARM_JOINT_COUNT + 1)) * 2):
        raise CoordinatedLimitError(
            f"[joint_names] 每臂内部的关节序号必须是 1..{ARM_JOINT_COUNT}，"
            f"实际：{list(indices)}（关节名：{list(names)}）"
        )

    single = software_limits()
    lower = tuple(float(v) for v in single.lower) * 2
    upper = tuple(float(v) for v in single.upper) * 2
    velocity = tuple(float(v) for v in single.velocity) * 2
    acceleration = tuple(float(v) for v in DEFAULT_ACCELERATION_LIMITS_RAD_S2) * 2

    return DualLimits(
        joint_names=names,
        lower=lower,
        upper=upper,
        velocity=velocity,
        acceleration=acceleration,
        sides=sides,
    )


def pad_gripper_positions(
    q14: Sequence[Sequence[float]],
    *,
    gripper_joint_names: Sequence[str] = (),
    gripper_value: float = 0.0,
) -> np.ndarray:
    """把 14 维臂关节补齐成 plant 的**完整位置向量**（装夹爪时为 18 维）。

    顺序约定（**必须与 plant 的位置变量顺序一致**）
    -----------------------------------------------
    plant 的位置变量顺序 = model instance 顺序 =
    **左臂 7 → 右臂 7 → 左夹爪 2 → 右夹爪 2**（见
    :mod:`rm75_drake.bimanual.dual_scene` 的装配顺序：两臂先 ``AddModels``，
    夹爪随后）。因此补齐值按 ``gripper_joint_names`` 的顺序追加——该序列来自
    :attr:`rm75_drake.bimanual.dual_scene.DualSceneBundle.gripper_joint_names`
    （同样是「左夹爪 → 右夹爪」）。

    参数
    ----
    q14:
        ``(N, 14)``（或 ``(14,)``）的臂关节序列。
    gripper_joint_names:
        plant 里的夹爪可动关节名；长度 0 = 不装夹爪（返回 14 维）。
    gripper_value:
        夹爪 prismatic 的**固定开度**（m）。默认 0.0（闭合）。

    返回
    ----
    ``(N, 14 + len(gripper_joint_names))`` 的 numpy 数组。
    """
    array = np.asarray(q14, dtype=float)
    if array.ndim == 1:
        array = array.reshape(1, -1)
    if array.ndim != 2 or array.shape[1] != DUAL_JOINT_COUNT:
        raise CoordinatedLimitError(
            f"[joint_count] 臂关节序列必须是 {DUAL_JOINT_COUNT} 维，实际：{array.shape}"
        )
    if not np.all(np.isfinite(array)):
        raise CoordinatedLimitError(
            f"[limits] 臂关节序列含 NaN/Inf，无法补齐夹爪自由度"
        )

    names = tuple(str(name) for name in gripper_joint_names)
    if not names:
        return array.copy()

    value = float(gripper_value)
    if not np.isfinite(value):
        raise CoordinatedLimitError(
            f"[gripper] 夹爪开度必须是有限值，实际：{gripper_value!r}"
        )

    pad = np.full((array.shape[0], len(names)), value, dtype=float)
    return np.hstack([array, pad])
