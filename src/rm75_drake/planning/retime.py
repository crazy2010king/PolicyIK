"""轨迹重定时（Toppra）+ 5 ms 网格采样 + C2 连续与限值复核（S02-T06）。

职责边界
--------
本模块是 **S02 规划内核 → S03 实时层** 的时间参数化部件：把一条**纯几何**的
关节路径（``q_seq``）变成一条**可直接下发**的时间参数化轨迹——

* 逐点落在 **5 ms 固定网格**（``t_abs = t0 + k * dt``，``dt`` 默认 0.005 s）；
* 每点带单调 ``seq`` 与严格递增的绝对时刻；
* 逐轴速度 / 加速度 **不超过真机限值**；
* 底层曲线 **C2 连续**（位置、速度、加速度连续），并额外在 5 ms 网格上
  **复核离散速度 / 加速度的跳变**（含首个违规索引）。

重定时内核
----------
使用 **toppra 0.6.11**（0.6.x 的 API 形态，已在本机与 ``vendor/wheels`` 具备）：

* ``toppra.interpolator.SplineInterpolator`` —— 几何路径（关节空间的三次样条）；
* ``toppra.constraint.JointVelocityConstraint`` / ``JointAccelerationConstraint``
  —— 速度 / 加速度约束；
* ``toppra.algorithm.TOPPRA`` —— 时间最优参数化；
* ``toppra.ParametrizeSpline`` —— 参数化结果本身是 **CubicSpline**，
  因此输出轨迹在时间上 **C2 连续**（位置 / 速度 / 加速度均连续）。

约束的来源（**不得省略**）
---------------------------
* **速度上限**：软件限位（``models/rm75_drake/joint_limits.yaml`` 的 ``software``
  段，J1/J2 = 3.1416 rad/s、J3–J7 = 3.927 rad/s），来自
  :func:`rm75_drake.planning.trajectory.software_limits`，本模块**不复制常量**。
* **加速度上限**：**本项目自研默认值**（官方 RM75 参数表**未公布**加速度上限），
  即 :data:`rm75_drake.planning.trajectory.DEFAULT_ACCELERATION_LIMITS_RAD_S2`
  （J1/J2 = 6.0、J3–J7 = 8.0 rad/s²）。**S03 真机层将按实测收紧**。
* **jerk 上限**（连续性复核用）：同样是**自研默认值**
  :data:`DEFAULT_JERK_LIMIT_RAD_S3` = 2000 rad/s³，用于判定 5 ms 网格上的
  加速度跳变是否超差；S03 按实测收紧。

数值裕度（safety margin）
-------------------------
TOPPRA 的约束是在**参数化网格点**上成立的，而 5 ms 采样后的**有限差分**
速度 / 加速度会略高于连续曲线的极值（样条插值误差）。因此实现采用
**裕度回退**（:data:`MARGIN_BACKOFF`）：从 1.0 开始逐档收紧约束，
取**第一档同时满足**「有限差分速度/加速度不超限 + C2 连续性复核通过」的解。
这样既不静默放行越界轨迹，也不会因为 1e-4 级别的离散误差误报失败。

复用而非重写不变量
------------------
最终轨迹一律通过 :func:`rm75_drake.planning.trajectory.trajectory_from_arrays`
构造 —— 软限位 / 速度 / 加速度 / seq / 时间网格的校验**只有一份实现**
（S02-T02），本模块**不另写一份**。

依赖
----
``numpy`` + ``toppra``（0.6.11）。**toppra 缺失时抛**
:class:`RetimeError`（标签 ``[dependency]``），而不是在 import 期崩溃——
远端基线未装 toppra 时调用方能拿到可解释错误。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np

from .trajectory import (
    DEFAULT_ACCELERATION_LIMITS_RAD_S2,
    DEFAULT_DT,
    JOINT_COUNT,
    Trajectory,
    TrajectoryError,
    accelerations as _traj_accelerations,
    software_limits,
    trajectory_from_arrays,
    velocities as _traj_velocities,
)

__all__ = [
    "RetimeError",
    "DEFAULT_JERK_LIMIT_RAD_S3",
    "DEFAULT_GRIDPOINTS",
    "MARGIN_BACKOFF",
    "RetimedTrajectory",
    "retime",
    "retime_from_trajectory",
    "check_c2_continuity",
]


class RetimeError(RuntimeError):
    """重定时失败。

    错误信息统一带一个**稳定标签**，供调用方与测试按标签分类：

    ``[dt]`` / ``[shape]`` / ``[finite]`` / ``[limits]`` / ``[path]`` /
    ``[dependency]`` / ``[parameterization]`` / ``[velocity]`` /
    ``[acceleration]`` / ``[continuity]`` / ``[duration]``。
    """


#: **自研默认 jerk 上限**（rad/s³）——用于 5 ms 网格上的加速度连续性复核。
#:
#: 官方 RM75 参数表既未公布加速度上限，也未公布 jerk 上限；本常量是
#: **本项目自研默认值**，用于判定「相邻采样点的加速度跳变」是否超差。
#: **S03 真机层将按实测收紧。**
DEFAULT_JERK_LIMIT_RAD_S3: float = 2000.0

#: TOPPRA 参数化网格点数（几何路径上的离散点数）。
DEFAULT_GRIDPOINTS: int = 50

#: 约束裕度回退序列（乘到速度 / 加速度上限上；越小 = 越保守 = 越慢）。
#:
#: 逐档回退，取**第一档通过全部检查**的解；全部失败 → ``[parameterization]``。
MARGIN_BACKOFF: Tuple[float, ...] = (1.0, 0.95, 0.9, 0.85, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3)

#: 数值比较容差（rad / rad/s / rad/s² / s）。
_TOL = 1e-9


# ── 统计信息 ────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class RetimedTrajectory:
    """重定时结果：一条 5 ms 网格轨迹 + 统计信息。

    字段
    ----
    trajectory:
        重定时后的轨迹（:class:`rm75_drake.planning.trajectory.Trajectory`），
        **已经过** S02-T02 的全部不变量校验（软限位 / 速度 / 加速度 /
        seq / 时间网格）。
    duration:
        总时长（s）= ``trajectory.duration``。
    peak_velocity:
        逐轴峰值速度（rad/s），长度 7。
    peak_acceleration:
        逐轴峰值加速度（rad/s²），长度 7。
    max_step_velocity:
        **最大单步速度**（rad/s）：相邻采样点之间速度差的 **L2 范数**的最大值
        （即「一步之内七轴合起来最快的瞬时速度」）。
    max_step_displacement:
        最大单步位移（rad）：相邻采样点位移的 L2 范数的最大值。
    margin:
        实际采用的**约束裕度**因子（见模块 docstring「数值裕度」）。
    gridpoints:
        实际采用的 TOPPRA 参数化网格点数。
    """

    trajectory: Trajectory
    duration: float
    peak_velocity: Tuple[float, ...]
    peak_acceleration: Tuple[float, ...]
    max_step_velocity: float
    max_step_displacement: float
    margin: float
    gridpoints: int

    # -- 便捷访问 -------------------------------------------------------
    @property
    def samples(self):
        """采样点序列（委托给内部轨迹）。"""
        return self.trajectory.samples

    @property
    def q_matrix(self) -> np.ndarray:
        """``(N, 7)`` 关节角矩阵（委托给内部轨迹）。"""
        return self.trajectory.q_matrix()

    @property
    def peak_velocity_max(self) -> float:
        """逐轴峰值速度中的最大值（rad/s）。"""
        return max(self.peak_velocity) if self.peak_velocity else 0.0

    @property
    def peak_acceleration_max(self) -> float:
        """逐轴峰值加速度中的最大值（rad/s²）。"""
        return max(self.peak_acceleration) if self.peak_acceleration else 0.0

    def to_trajectory(self) -> Trajectory:
        """返回内部轨迹（S03 实时层消费的对象）。"""
        return self.trajectory


# ── 连续性复核 ──────────────────────────────────────────────────────────
def check_c2_continuity(
    velocities: np.ndarray,
    accelerations: np.ndarray,
    *,
    dt: float,
    acceleration_limits: Sequence[float],
    jerk_limit: float = DEFAULT_JERK_LIMIT_RAD_S3,
) -> None:
    """C2 连续性复核（离散版本）。

    检查两条：

    1. **速度连续**：相邻采样点的速度差 ``|v[k+1] - v[k]|`` 不超过
       ``a_lim[axis] * dt``（容差 1e-9）——即速度变化率不超过加速度上限；
    2. **加速度连续**：相邻采样点的加速度差 ``|a[k+1] - a[k]|`` 不超过
       ``jerk_limit * dt``（容差 1e-9）——即加速度变化率不超过 jerk 上限。

    超差即抛 :class:`RetimeError`（标签 ``[continuity]``），错误信息给出
    **首个违规索引**与轴号。

    参数
    ----
    velocities:
        ``(N-1, 7)`` 的速度矩阵（rad/s）。
    accelerations:
        ``(N-2, 7)`` 的加速度矩阵（rad/s²）；长度不足 2 时跳过第 2 条检查。
    dt:
        采样周期（s），必须为正。
    acceleration_limits:
        逐轴加速度上限（rad/s²），长度 7。
    jerk_limit:
        jerk 上限（rad/s³）。

    备注
    ----
    底层曲线本身是 C2 的（toppra 的 ``ParametrizeSpline`` 基于 ``CubicSpline``），
    本函数复核的是**5 ms 离散网格**上的表现——离散化会把曲线的局部跳变
    放大成有限差分上的毛刺，这里给出可执行的判定。
    """
    step = float(dt)
    if not math.isfinite(step) or step <= 0.0:
        raise RetimeError(f"[dt] dt 必须是正有限值，实际：{dt!r}")

    limits = np.asarray(acceleration_limits, dtype=float)
    if limits.shape != (JOINT_COUNT,):
        raise RetimeError(
            f"[limits] 加速度上限必须是 {JOINT_COUNT} 维，实际 {limits.shape}"
        )

    vel = np.asarray(velocities, dtype=float)
    acc = np.asarray(accelerations, dtype=float)

    if vel.ndim == 2 and vel.shape[0] >= 2:
        jump = np.abs(np.diff(vel, axis=0)) - limits[None, :] * step
        bad = np.argwhere(jump > _TOL)
        if bad.size:
            index, axis = int(bad[0][0]), int(bad[0][1])
            raise RetimeError(
                f"[continuity] 速度不连续：第 {index}→{index + 1} 个采样点第 {axis} 轴"
                f"速度跳变 {abs(vel[index + 1, axis] - vel[index, axis])!r} rad/s 超过"
                f"容差 a_lim*dt = {limits[axis] * step!r} rad/s"
                f"（a_lim={limits[axis]} rad/s²，dt={step} s）"
            )

    if acc.ndim == 2 and acc.shape[0] >= 2:
        jump = np.abs(np.diff(acc, axis=0)) - float(jerk_limit) * step
        bad = np.argwhere(jump > _TOL)
        if bad.size:
            index, axis = int(bad[0][0]), int(bad[0][1])
            raise RetimeError(
                f"[continuity] 加速度不连续：第 {index}→{index + 1} 个采样点第 {axis} 轴"
                f"加速度跳变 {abs(acc[index + 1, axis] - acc[index, axis])!r} rad/s² 超过"
                f"容差 jerk*dt = {float(jerk_limit) * step!r} rad/s²"
                f"（jerk_limit={jerk_limit} rad/s³）"
            )


# ── 主流程 ──────────────────────────────────────────────────────────────
def retime(
    q_seq: Sequence[Sequence[float]],
    *,
    velocity_limits: Optional[Sequence[float]] = None,
    acceleration_limits: Optional[Sequence[float]] = None,
    dt: float = DEFAULT_DT,
    t0: float = 0.0,
    robot_name: str = "rm75",
    gripper_schedule: Sequence = (),
    duration_limit: Optional[float] = None,
    jerk_limit: float = DEFAULT_JERK_LIMIT_RAD_S3,
    gridpoints: int = DEFAULT_GRIDPOINTS,
) -> RetimedTrajectory:
    """把一条几何关节路径重定时为 5 ms 网格的时间参数化轨迹。

    参数
    ----
    q_seq:
        ``(N, 7)`` 的**几何**关节路径（rad），``N >= 2``。相邻重复点会被
        合并（不影响路径形状），全部重合则抛 ``[path]``。
    velocity_limits:
        逐轴速度上限（rad/s），长度 7；``None`` → 软件限位
        （J1/J2 = 3.1416、J3–J7 = 3.927 rad/s，来自 ``joint_limits.yaml``）。
    acceleration_limits:
        逐轴加速度上限（rad/s²），长度 7；``None`` → **自研默认值**
        :data:`rm75_drake.planning.trajectory.DEFAULT_ACCELERATION_LIMITS_RAD_S2`
        （J1/J2 = 6.0、J3–J7 = 8.0 rad/s²；官方未公布，S03 将按实测收紧）。
    dt:
        采样周期（s），默认 **5 ms**；必须为正有限值。
    t0:
        首点的绝对时刻（s）。
    robot_name:
        机器人名，写入轨迹供回放校验。
    gripper_schedule:
        夹爪动作调度表（原样透传给轨迹）。
    duration_limit:
        可选的**总时长上限**（s）。若最小可行时长超过它 → 抛 ``[duration]``，
        错误信息说明被**速度**还是**加速度**上限卡住。
    jerk_limit:
        jerk 上限（rad/s³），用于连续性复核；默认
        :data:`DEFAULT_JERK_LIMIT_RAD_S3`。
    gridpoints:
        TOPPRA 参数化网格点数。

    返回
    ----
    :class:`RetimedTrajectory`

    失败
    ----
    * ``[dt]`` —— ``dt`` 非正 / 非有限；
    * ``[shape]`` / ``[finite]`` —— 输入形状或数值非法；
    * ``[limits]`` —— 上限非法（维度错、非正、非有限）；
    * ``[path]`` —— 路径长度为零；
    * ``[dependency]`` —— ``toppra`` 不可用；
    * ``[parameterization]`` —— TOPPRA 无法在裕度回退序列内找到可行参数化；
    * ``[continuity]`` —— 5 ms 网格上的速度 / 加速度跳变超差（给出首个违规索引）；
    * ``[duration]`` —— 超过 ``duration_limit``；
    * :class:`rm75_drake.planning.trajectory.TrajectoryError` —— 最终轨迹违反
      S02-T02 的不变量（软限位 / 速度 / 加速度 / seq / 时间网格）。
    """
    step = float(dt)
    if not math.isfinite(step) or step <= 0.0:
        raise RetimeError(f"[dt] dt 必须是正有限值，实际：{dt!r}")

    start = float(t0)
    if not math.isfinite(start):
        raise RetimeError(f"[dt] t0 必须是有限值，实际：{t0!r}")

    points = np.asarray(q_seq, dtype=float)
    if points.ndim != 2 or points.shape[1] != JOINT_COUNT:
        raise RetimeError(
            f"[shape] q_seq 必须是 ({JOINT_COUNT}, N) 的二维数组，实际：{points.shape}"
        )
    if points.shape[0] < 2:
        raise RetimeError(
            f"[shape] q_seq 至少需要 2 个点（起点与终点），实际：{points.shape[0]}"
        )
    if not np.all(np.isfinite(points)):
        row, axis = (int(v) for v in np.argwhere(~np.isfinite(points))[0])
        raise RetimeError(
            f"[finite] 第 {row} 个路径点第 {axis} 轴关节角非有限值：{points[row, axis]!r}"
        )

    vel_limits = np.asarray(
        software_limits().velocity if velocity_limits is None else velocity_limits,
        dtype=float,
    )
    acc_limits = np.asarray(
        DEFAULT_ACCELERATION_LIMITS_RAD_S2
        if acceleration_limits is None
        else acceleration_limits,
        dtype=float,
    )
    for name, arr in (("速度", vel_limits), ("加速度", acc_limits)):
        if arr.shape != (JOINT_COUNT,):
            raise RetimeError(
                f"[limits] {name}上限必须是 {JOINT_COUNT} 维，实际 {arr.shape}"
            )
        if not np.all(np.isfinite(arr)) or np.any(arr <= 0.0):
            raise RetimeError(
                f"[limits] {name}上限必须是正有限值，实际：{arr.tolist()}"
            )
    if not math.isfinite(float(jerk_limit)) or float(jerk_limit) <= 0.0:
        raise RetimeError(f"[limits] jerk 上限必须是正有限值，实际：{jerk_limit!r}")

    waypoints, path_length = _dedupe_waypoints(points)
    if path_length <= 1e-12:
        raise RetimeError("[path] 路径长度为零（所有路径点重合），无法重定时")

    toppra = _load_toppra()
    grid_count = int(gridpoints)
    if grid_count < 2:
        raise RetimeError(f"[limits] gridpoints 至少为 2，实际：{gridpoints!r}")

    failures: List[str] = []
    for margin in MARGIN_BACKOFF:
        try:
            sampler, duration = _parameterize(
                toppra,
                waypoints,
                vel_limits=vel_limits * margin,
                acc_limits=acc_limits * margin,
                gridpoints=grid_count,
            )
        except RetimeError as exc:
            failures.append(f"margin={margin}: {exc}")
            continue

        samples_q = _sample_grid(sampler, duration, step)
        try:
            traj = trajectory_from_arrays(
                samples_q,
                t0=start,
                dt=step,
                robot_name=robot_name,
                gripper_schedule=gripper_schedule,
                acceleration_limits=tuple(float(v) for v in acc_limits),
            )
        except TrajectoryError as exc:
            # 软限位等**与裕度无关**的不变量被违反 → 立即报错（回退裕度也救不回来）。
            if "[soft_limits]" in str(exc) or "[finite]" in str(exc):
                raise
            # 速度 / 加速度越界：记入失败原因，由更保守的裕度档重试；
            # 全部失败则抛 [parameterization]。
            failures.append(f"margin={margin}: 轨迹不变量被违反：{exc}")
            continue
        vel = _traj_velocities(traj)
        acc = _traj_accelerations(traj)

        limit_hit = _first_limit_violation(vel, acc, vel_limits, acc_limits)
        if limit_hit is not None:
            failures.append(f"margin={margin}: {limit_hit}")
            continue
        try:
            check_c2_continuity(
                vel,
                acc,
                dt=step,
                acceleration_limits=acc_limits,
                jerk_limit=jerk_limit,
            )
        except RetimeError as exc:
            failures.append(f"margin={margin}: {exc}")
            continue

        actual_duration = float(traj.duration)
        if duration_limit is not None and actual_duration > float(duration_limit) + _TOL:
            limit = float(duration_limit)
            required_speed = path_length / limit
            # 判定被哪个上限卡住：平均速度需求超过速度上限 → 速度；否则是加速度
            #（要在 limit 内完成起步/刹停，加速度需求超过加速度上限）。
            if required_speed > float(vel_limits.max()):
                binding = (
                    f"速度上限卡住（平均速度需求 {required_speed!r} rad/s > 最大速度上限 "
                    f"{float(vel_limits.max())!r} rad/s）"
                )
            else:
                binding = (
                    f"加速度上限卡住（平均速度需求 {required_speed!r} rad/s 在速度上限内，"
                    f"但在 {limit!r} s 内完成起步/刹停所需的加速度超过 "
                    f"{float(acc_limits.max())!r} rad/s²）"
                )
            raise RetimeError(
                f"[duration] 最短可行时长 {actual_duration!r} s 超过上限 {limit!r} s："
                f"{binding}（路径长度 {path_length!r} rad）"
            )

        displacement = np.linalg.norm(np.diff(samples_q, axis=0), axis=1)
        step_velocity = np.linalg.norm(vel, axis=1)
        return RetimedTrajectory(
            trajectory=traj,
            duration=actual_duration,
            peak_velocity=tuple(
                float(v)
                for v in (np.abs(vel).max(axis=0) if vel.size else np.zeros(JOINT_COUNT))
            ),
            peak_acceleration=tuple(
                float(v)
                for v in (np.abs(acc).max(axis=0) if acc.size else np.zeros(JOINT_COUNT))
            ),
            max_step_velocity=float(step_velocity.max()) if step_velocity.size else 0.0,
            max_step_displacement=float(displacement.max()) if displacement.size else 0.0,
            margin=float(margin),
            gridpoints=grid_count,
        )

    # 失败归类：区分「速度越界」/「加速度越界」/「参数化失败」/「连续性失败」。
    joined = "\n".join(failures)
    if "有限差分速度越界" in joined:
        tag = "[velocity]"
    elif "有限差分加速度越界" in joined:
        tag = "[acceleration]"
    elif "[continuity]" in joined:
        tag = "[continuity]"
    else:
        tag = "[parameterization]"
    raise RetimeError(
        f"{tag} 无法在约束裕度回退序列 "
        f"{list(MARGIN_BACKOFF)} 内找到满足速度/加速度上限与 C2 连续性的参数化；"
        "各档失败原因：\n  - " + "\n  - ".join(failures)
    )


def retime_from_trajectory(
    traj: Trajectory,
    *,
    velocity_limits: Optional[Sequence[float]] = None,
    acceleration_limits: Optional[Sequence[float]] = None,
    dt: float = DEFAULT_DT,
    jerk_limit: float = DEFAULT_JERK_LIMIT_RAD_S3,
    gridpoints: int = DEFAULT_GRIDPOINTS,
    duration_limit: Optional[float] = None,
) -> RetimedTrajectory:
    """对一条已有 :class:`~rm75_drake.planning.trajectory.Trajectory` 做重定时。

    保留原轨迹的 ``robot_name`` 与夹爪调度表；起始绝对时刻取原轨迹的
    ``start_time``。
    """
    if not isinstance(traj, Trajectory):
        raise RetimeError(f"[shape] 期望 Trajectory，实际：{traj!r}")
    return retime(
        traj.q_matrix(),
        velocity_limits=velocity_limits,
        acceleration_limits=acceleration_limits,
        dt=dt,
        t0=traj.start_time,
        robot_name=traj.robot_name,
        gripper_schedule=traj.gripper_schedule,
        duration_limit=duration_limit,
        jerk_limit=jerk_limit,
        gridpoints=gridpoints,
    )


# ── 内部工具 ────────────────────────────────────────────────────────────
def _load_toppra():
    """导入 toppra（0.6.11）；不可用时抛 ``[dependency]``。"""
    try:
        import toppra  # noqa: F401
        from toppra.algorithm import TOPPRA
        from toppra.constraint import (
            JointAccelerationConstraint,
            JointVelocityConstraint,
        )
        from toppra.interpolator import SplineInterpolator
    except Exception as exc:  # pragma: no cover - 依赖缺失路径
        raise RetimeError(
            "[dependency] 无法导入 toppra（0.6.11）："
            f"{type(exc).__name__}: {exc}。"
            "离线安装：`pip install --no-index --find-links vendor/wheels toppra`"
        ) from exc
    return TOPPRA, JointVelocityConstraint, JointAccelerationConstraint, SplineInterpolator


def _dedupe_waypoints(points: np.ndarray) -> Tuple[np.ndarray, float]:
    """合并相邻重复点，返回（路径点数组，路径长度）。"""
    keep = [0]
    for index in range(1, points.shape[0]):
        if not np.allclose(points[index], points[keep[-1]], rtol=0.0, atol=1e-12):
            keep.append(index)
    waypoints = points[keep]
    if waypoints.shape[0] < 2:
        return waypoints, 0.0
    segments = np.linalg.norm(np.diff(waypoints, axis=0), axis=1)
    return waypoints, float(segments.sum())


def _parameterize(
    toppra_modules,
    waypoints: np.ndarray,
    *,
    vel_limits: np.ndarray,
    acc_limits: np.ndarray,
    gridpoints: int,
) -> Tuple[object, float]:
    """跑一遍 TOPPRA 参数化，返回（可调用的时间参数化曲线，总时长）。"""
    TOPPRA, JointVelocityConstraint, JointAccelerationConstraint, SplineInterpolator = (
        toppra_modules
    )
    # 几何路径：按弦长参数化的关节空间三次样条。
    chord = np.linalg.norm(np.diff(waypoints, axis=0), axis=1)
    ss = np.concatenate([[0.0], np.cumsum(chord)])
    path = SplineInterpolator(ss, waypoints)
    interval = np.asarray(path.path_interval, dtype=float)
    grid = np.linspace(interval[0], interval[1], int(gridpoints))

    constraints = [
        JointVelocityConstraint(vel_limits),
        JointAccelerationConstraint(acc_limits),
    ]
    instance = TOPPRA(constraints, path, gridpoints=grid)
    result = instance.compute_trajectory(0.0, 0.0)
    if result is None:
        raise RetimeError("TOPPRA 参数化失败（返回码非 Ok）")
    duration = float(result.path_interval[1])
    if not math.isfinite(duration) or duration <= 0.0:
        raise RetimeError(f"TOPPRA 返回的时长非法：{duration!r}")
    return result, duration


def _sample_grid(
    sampler,
    duration: float,
    dt: float,
) -> np.ndarray:
    """在 5 ms 固定网格上采样时间参数化曲线。

    网格点数 ``N = max(1, ceil(duration / dt))``，采样点对应的**曲线参数时刻**
    ``u_k = k * duration / N``，而**绝对时刻**恒为 ``t_abs = t0 + k * dt``。

    * 时间拉伸因子 ``f = N * dt / duration >= 1``：实际下发时间轴比 TOPPRA 的
      最优时间轴**更慢**，因此真实速度 / 加速度是 TOPPRA 约束值的 ``1/f`` /
      ``1/f²`` 倍——**只会更保守，不会越限**；
    * 末点 ``u_N = duration`` 恰好落在路径终点（不会因为取整丢掉最后一段运动）。
    """
    count = max(int(math.ceil(duration / dt)), 1)
    u = np.arange(count + 1, dtype=float) * (duration / count)
    u[-1] = duration
    return np.asarray(
        [np.asarray(sampler(float(value)), dtype=float) for value in u]
    )


def _first_limit_violation(
    vel: np.ndarray,
    acc: np.ndarray,
    vel_limits: np.ndarray,
    acc_limits: np.ndarray,
) -> Optional[str]:
    """返回第一条越界描述（速度优先），无越界返回 ``None``。"""
    if vel.size:
        excess = np.abs(vel) - vel_limits[None, :]
        bad = np.argwhere(excess > _TOL)
        if bad.size:
            index, axis = int(bad[0][0]), int(bad[0][1])
            return (
                f"有限差分速度越界：第 {index}→{index + 1} 个采样点第 {axis} 轴"
                f"速度 {abs(vel[index, axis])!r} rad/s 超过上限 {vel_limits[axis]} rad/s"
            )
    if acc.size:
        excess = np.abs(acc) - acc_limits[None, :]
        bad = np.argwhere(excess > _TOL)
        if bad.size:
            index, axis = int(bad[0][0]), int(bad[0][1])
            return (
                f"有限差分加速度越界：第 {index}/{index + 1}/{index + 2} 个采样点第 "
                f"{axis} 轴加速度 {abs(acc[index, axis])!r} rad/s² 超过上限 "
                f"{acc_limits[axis]} rad/s²"
            )
    return None
