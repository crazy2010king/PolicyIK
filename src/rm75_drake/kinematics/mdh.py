"""RM75 的官方 MDH 参数表与独立正解（**零 Drake 依赖**）。

本模块是 S01 的"独立被测对照"：它只用 numpy 实现正解，**不 import pydrake**，
因此可以作为 Drake FK 的**外部参照**（而不是用 Drake 验证 Drake）。

参数表来源
----------
* 官方开发者中心 **RM75 参数表**（改进 D-H / Modified D-H）；
* 上游 commit ``cff49bb40f5ed0728505c03bff0e521a0dc21acd``（rm_models）；
* 评审记录 **E9**：零位末端高度锚点 ``0.8505 m`` = d1 + d3 + d5 + d7
  = 0.2405 + 0.256 + 0.210 + 0.144。

约定
----
正解递推（classic D-H 的"关节角在前"写法）::

    T = ∏_{i=1..7}  Rotz(q_i) · Transz(d_i) · Rotx(alpha_i)

每一步把"绕 z 转 q_i → 沿 z 平移 d_i → 绕 x 转 alpha_i"复合为一个 4×4 齐次变换，
逐级左乘。零位（q = 0）时该链的末端高度恰为 d1+d3+d5+d7 = 0.8505 m。

与改造后 URDF 的关系（**已实测**）
------------------------------------
改造后的 URDF（``models/rm75_drake/rm75_drake.urdf``）在每个关节上用的是
**官方 rpy 舍入值 1.5708**（而非精确 ±π/2）。用同样的舍入值填本表的 alpha 时：

* 姿态误差 < 1e-15 rad（旋转部分完全一致）；
* 位置误差 ≈ 2.24e-6 m。

这个位置残差**不是本模块的缺陷**，而是官方 URDF 舍入的直接后果：把 URDF 的
rpy 全部换成精确 ±π/2 后，本表（同样换成精确 ±π/2）与 URDF 链的残差降到
~1e-16 m（约定完全等价）。也就是说 **Drake 载入的 FK 相对精确模型本身就有
~2.2e-6 m 的偏差**，任何独立的解析模型都无法在 1e-6 m 以内与它对齐。
该残差在 :mod:`tests.test_mdh_fk` 中被显式钉死为已知量（而不是未知误差）。

零 Drake 依赖
-------------
本模块**只** import numpy（不 import pydrake / rm75_drake.models），
以保证"独立对照"的性质；:func:`rm75_drake.kinematics.__init__` 的导入同样不拉起 Drake。
"""

from __future__ import annotations

from typing import Optional, Sequence

import numpy as np

__all__ = [
    "MDH_D",
    "MDH_A",
    "MDH_ALPHA",
    "MDH_ALPHA_EXACT_PI_2",
    "MDH_ZERO_HEIGHT_M",
    "N_JOINTS",
    "fk",
    "link7_position",
    "link7_rotation",
]

#: 关节数（RM75 = 7 DoF）。
N_JOINTS = 7

#: MDH 的 d_i（沿 z 平移量，m）。
#:
#: 非零项出现在 i = 1, 3, 5, 7（与官方 RM75 参数表一致）。
MDH_D: tuple = (0.2405, 0.0, 0.256, 0.0, 0.210, 0.0, 0.144)

#: MDH 的 a_i（沿 x 平移量，m）。RM75 各轴共线，恒为 0。
MDH_A: tuple = (0.0,) * N_JOINTS

#: MDH 的 alpha_i（绕 x 的连杆扭角，rad）。
#:
#: 数值**刻意取改造后 URDF 里 joint rpy 的同一组舍入值**（±1.5708），
#: 使本表与改造后的 URDF 在姿态上完全一致（实测 < 1e-15 rad），
#: 位置残差仅剩官方 rpy 舍入导致的 ~2.24e-6 m（见模块 docstring）。
MDH_ALPHA: tuple = (
    -1.5708,
    1.5708,
    -1.5708,
    1.5708,
    -1.5708,
    1.5708,
    0.0,
)

#: 精确 ±π/2 版本的 alpha 表（理论对照用）。
#:
#: 把它与"URDF 的 rpy 换成精确 ±π/2"配对使用时，两者残差 ~1e-16 m，
#: 即 **classic D-H 与 URDF 的关节链约定完全等价**（证明见
#: :mod:`tests.test_mdh_fk` 的等价性断言）。
MDH_ALPHA_EXACT_PI_2: tuple = (
    -np.pi / 2,
    np.pi / 2,
    -np.pi / 2,
    np.pi / 2,
    -np.pi / 2,
    np.pi / 2,
    0.0,
)

#: 零位末端高度锚点（m）= d1 + d3 + d5 + d7。
#:
#: 来源：官方开发者中心 RM75 参数表 + 评审记录 E9。
MDH_ZERO_HEIGHT_M = 0.8505


def _rot_z(angle: float) -> np.ndarray:
    """绕 z 轴的 4×4 旋转矩阵。"""
    c = float(np.cos(angle))
    s = float(np.sin(angle))
    T = np.eye(4)
    T[0, 0] = c
    T[0, 1] = -s
    T[1, 0] = s
    T[1, 1] = c
    return T


def _rot_x(angle: float) -> np.ndarray:
    """绕 x 轴的 4×4 旋转矩阵。"""
    c = float(np.cos(angle))
    s = float(np.sin(angle))
    T = np.eye(4)
    T[1, 1] = c
    T[1, 2] = -s
    T[2, 1] = s
    T[2, 2] = c
    return T


def _trans_z(distance: float) -> np.ndarray:
    """沿 z 平移的 4×4 齐次矩阵。"""
    T = np.eye(4)
    T[2, 3] = float(distance)
    return T


def _as_q(q: Sequence[float]) -> np.ndarray:
    """把关节角序列转成 ``(7,)`` float 数组，形状不对时抛 :class:`ValueError`。"""
    arr = np.asarray(q, dtype=float).reshape(-1)
    if arr.size != N_JOINTS:
        raise ValueError(
            f"RM75 需要 {N_JOINTS} 个关节角，收到 {arr.size} 个：{q!r}"
        )
    if not np.all(np.isfinite(arr)):
        raise ValueError(f"关节角含有 NaN/Inf：{q!r}")
    return arr


def fk(
    q: Sequence[float],
    *,
    d: Optional[Sequence[float]] = None,
    alpha: Optional[Sequence[float]] = None,
) -> np.ndarray:
    """返回 link_7 在 base_link 系下的位姿（4×4 齐次矩阵）。

    参数
    ----
    q:
        7 个关节角（rad），顺序为 joint_1..joint_7。
    d:
        可选的 d 表覆盖（默认 :data:`MDH_D`）。
    alpha:
        可选的 alpha 表覆盖（默认 :data:`MDH_ALPHA`；理论对照可用
        :data:`MDH_ALPHA_EXACT_PI_2`）。

    返回
    ----
    numpy.ndarray
        4×4 齐次矩阵 ``X_BaseLink_Link7``。

    异常
    ----
    ValueError
        关节角个数不是 7、含 NaN/Inf，或 d/alpha 表长度不是 7。
    """
    angles = _as_q(q)
    d_table = MDH_D if d is None else tuple(float(x) for x in d)
    alpha_table = MDH_ALPHA if alpha is None else tuple(float(x) for x in alpha)
    if len(d_table) != N_JOINTS:
        raise ValueError(f"d 表长度必须是 {N_JOINTS}，收到 {len(d_table)}")
    if len(alpha_table) != N_JOINTS:
        raise ValueError(f"alpha 表长度必须是 {N_JOINTS}，收到 {len(alpha_table)}")

    T = np.eye(4)
    for i in range(N_JOINTS):
        T = T @ _rot_z(angles[i]) @ _trans_z(d_table[i]) @ _rot_x(alpha_table[i])
    return T


def link7_position(
    q: Sequence[float],
    *,
    d: Optional[Sequence[float]] = None,
    alpha: Optional[Sequence[float]] = None,
) -> np.ndarray:
    """返回 link_7 原点在 base_link 系下的位置（3 维数组，单位 m）。"""
    return fk(q, d=d, alpha=alpha)[:3, 3].copy()


def link7_rotation(
    q: Sequence[float],
    *,
    d: Optional[Sequence[float]] = None,
    alpha: Optional[Sequence[float]] = None,
) -> np.ndarray:
    """返回 link_7 的旋转矩阵（3×3，行主序，作用在列向量上）。"""
    return fk(q, d=d, alpha=alpha)[:3, :3].copy()
