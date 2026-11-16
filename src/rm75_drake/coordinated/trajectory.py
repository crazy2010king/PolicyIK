"""**共享时间轴**的双臂轨迹容器（S05-T02）。

为什么不新建 14 维轨迹类型
--------------------------
S02 的 :class:`rm75_drake.planning.trajectory.Trajectory` 是 **S02 → S03 的
冻结交接面**：它把「软限位 / 速度 / 加速度 / seq / 时间网格」五类不变量
收口成**一份实现**，S03 的实时安全闸与 S07 的 golden-file 回归都消费它。
若为双臂另写一个 14 维轨迹类，就会出现**第二份不变量实现**——两条校验一旦
漂移，「看起来合法」的错误轨迹就会溜进真机层。

因此本模块的做法是：

* 把 **14 维联合几何路径**（重定时之后）**拆成两个 7 维** ``Trajectory``
  （用 :func:`rm75_drake.planning.trajectory.trajectory_from_arrays`），
  组成 :class:`DualTrajectory`；
* 构造期断言两条轨迹的 ``t_abs`` / ``dt`` **逐点相同**
  ⇒ **共享时间轴由构造保证**（而不是靠调用方自觉）；
* 每条 7 维轨迹各自走一遍 S02 的全部不变量校验（软限位 / 速度 / 加速度 /
  seq / 时间网格）——**零重复实现**。

关节顺序约定
------------
14 维联合向量的列顺序 = **左臂 7（joint_1..joint_7）→ 右臂 7**
（与 :mod:`rm75_drake.coordinated.limits` 的装配顺序、plant 的位置变量顺序
一致）。

夹爪自由度口径
--------------
本模块只产出 **14 维臂关节**。场景装夹爪时 ``plant.num_positions() == 18``
（两夹爪各 2 个 prismatic），**碰撞查询前**必须按
:func:`rm75_drake.coordinated.limits.pad_gripper_positions` 补齐夹爪 DOF
（固定开度或当前值）。S04 的 ``include_grippers_in_collision`` 默认
``False``——夹爪默认不进入碰撞判定；置 ``True`` 时 q **必须**包含这 4 个
prismatic。

零 pydrake 依赖
---------------
只依赖 ``numpy`` + :mod:`rm75_drake.planning.trajectory`（均为 numpy-only），
**不 import pydrake**。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np

from ..planning.trajectory import (
    DEFAULT_DT,
    GRID_TOLERANCE,
    JOINT_COUNT,
    Trajectory,
    TrajectoryError,
    trajectory_from_arrays,
)

__all__ = [
    "DualTrajectory",
    "dual_trajectory_from_arrays",
    "DUAL_JOINT_COUNT",
]

#: 双臂联合关节数（14 DoF）。
DUAL_JOINT_COUNT = 2 * JOINT_COUNT


@dataclass(frozen=True)
class DualTrajectory:
    """两条**共享同一时间轴**的 7 维轨迹（左臂 / 右臂）。

    字段
    ----
    left:
        左臂轨迹（:class:`rm75_drake.planning.trajectory.Trajectory`，7 维）。
    right:
        右臂轨迹（同上，7 维）。

    构造期不变量
    ------------
    * 两条轨迹都必须是 ``Trajectory``（标签 ``[joint_count]``）；
    * 采样点数相同、``dt`` 相同、``t_abs`` **逐点相同**
      （标签 ``[time_axis]``）⇒ **共享时间轴由构造保证**。
    """

    left: Trajectory
    right: Trajectory

    def __post_init__(self) -> None:
        if not isinstance(self.left, Trajectory) or not isinstance(self.right, Trajectory):
            raise TrajectoryError(
                f"[joint_count] DualTrajectory 的两条轨迹都必须是 Trajectory，"
                f"实际：{type(self.left).__name__} / {type(self.right).__name__}"
            )
        if len(self.left) != len(self.right):
            raise TrajectoryError(
                f"[time_axis] 两条轨迹的采样点数必须相同："
                f"left={len(self.left)} right={len(self.right)}"
            )
        if abs(float(self.left.dt) - float(self.right.dt)) > GRID_TOLERANCE:
            raise TrajectoryError(
                f"[time_axis] 两条轨迹的 dt 必须相同（共享时间轴）："
                f"left={self.left.dt!r} right={self.right.dt!r}"
            )
        left_times = np.asarray([s.t_abs for s in self.left.samples], dtype=float)
        right_times = np.asarray([s.t_abs for s in self.right.samples], dtype=float)
        mismatch = np.abs(left_times - right_times) > GRID_TOLERANCE
        if np.any(mismatch):
            index = int(np.argmax(mismatch))
            raise TrajectoryError(
                f"[time_axis] 第 {index} 个采样点的绝对时刻不一致："
                f"left={left_times[index]!r} right={right_times[index]!r}"
            )

    # -- 便捷访问 -------------------------------------------------------
    def __len__(self) -> int:
        """采样点数（两臂相同）。"""
        return len(self.left)

    @property
    def dt(self) -> float:
        """采样周期（s）——两臂相同。"""
        return float(self.left.dt)

    @property
    def start_time(self) -> float:
        """首点绝对时刻（s）。"""
        return float(self.left.start_time)

    @property
    def end_time(self) -> float:
        """末点绝对时刻（s）。"""
        return float(self.left.end_time)

    @property
    def duration(self) -> float:
        """轨迹时长（s）。"""
        return float(self.left.duration)

    @property
    def t_abs(self) -> np.ndarray:
        """共享的绝对时刻序列（``(N,)``）。"""
        return np.asarray([s.t_abs for s in self.left.samples], dtype=float)

    def q_matrix(self) -> np.ndarray:
        """14 维联合关节角矩阵（``(N, 14)``，左 7 + 右 7）。"""
        left = self.left.q_matrix()
        right = self.right.q_matrix()
        return np.hstack([left, right])

    def axis_q(self, axis: int) -> np.ndarray:
        """返回 14 维联合空间的某一轴序列（numpy ``(N,)``）。"""
        index = int(axis)
        if not 0 <= index < DUAL_JOINT_COUNT:
            raise TrajectoryError(
                f"[joint_count] 轴号必须在 [0, {DUAL_JOINT_COUNT}) 内，实际：{axis!r}"
            )
        return self.q_matrix()[:, index]


def dual_trajectory_from_arrays(
    q_seq: Sequence[Sequence[float]],
    t0: float = 0.0,
    dt: float = DEFAULT_DT,
    *,
    left_robot_name: str = "rm75_left",
    right_robot_name: str = "rm75_right",
    acceleration_limits: Optional[Sequence[float]] = None,
) -> DualTrajectory:
    """由 **14 维**联合关节序列构造 :class:`DualTrajectory`。

    参数
    ----
    q_seq:
        ``(N, 14)`` 的联合关节序列（rad），列顺序 = 左臂 7 → 右臂 7。
    t0:
        首点的绝对时刻（s），**两臂共享**。
    dt:
        采样周期（s），**两臂共享**。
    left_robot_name / right_robot_name:
        写入两条轨迹的机器人名（供回放校验）。
    acceleration_limits:
        可选的逐轴加速度上限（长度 7，**两臂相同**）；``None`` → S02 的自研
        默认值。

    失败
    ----
    :class:`rm75_drake.planning.trajectory.TrajectoryError`（标签
    ``[joint_count]`` / ``[empty]`` / ``[soft_limits]`` / ``[velocity]`` /
    ``[acceleration]`` / ``[time_grid]``）——**由 S02 的 ``Trajectory`` 统一
    抛出**，本函数不重复实现任何校验。
    """
    array = np.asarray(q_seq, dtype=float)
    if array.ndim != 2 or array.shape[1] != DUAL_JOINT_COUNT:
        raise TrajectoryError(
            f"[joint_count] 双臂联合路径必须是 {DUAL_JOINT_COUNT} 维"
            f"（左臂 {JOINT_COUNT} + 右臂 {JOINT_COUNT}），实际：{array.shape}"
        )
    if array.shape[0] == 0:
        raise TrajectoryError("[empty] 轨迹不能为空：至少需要 1 个采样点")

    limits = None if acceleration_limits is None else tuple(float(v) for v in acceleration_limits)
    left = trajectory_from_arrays(
        array[:, :JOINT_COUNT],
        t0=t0,
        dt=dt,
        robot_name=left_robot_name,
        acceleration_limits=limits,
    )
    right = trajectory_from_arrays(
        array[:, JOINT_COUNT:],
        t0=t0,
        dt=dt,
        robot_name=right_robot_name,
        acceleration_limits=limits,
    )
    return DualTrajectory(left=left, right=right)
