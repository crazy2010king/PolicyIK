"""轨迹数据结构与不变量（S02-T02）。

职责边界
--------
本模块是 **S02 → S03 的交接面**：把「规划结果」冻结成一个**可测试的不变量**，
让 S03 实时层（5 ms 透传、真机限值闸）与 S07 golden-file 回归消费同一份数据契约。

它**只**依赖 ``numpy`` + :mod:`rm75_drake.limits`（限位交接件）+ :mod:`rm75_drake.gripper`
（夹爪动作），**不含任何 pydrake 硬依赖**——因此可以被 S03（实时层，不一定装 Drake）
与 S07（离线回归）直接复用。

输出契约（ROADMAP「S02 → S03」边界）
------------------------------------
* **5 ms 固定网格**的关节角序列（``dt`` 默认 0.005 s）；
* 每点带**单调 seq**（从 0 开始，逐点 +1）与**绝对时刻** ``t_abs``（秒）；
* 逐点落在**软件软限位**内（每侧已再收 4°，见 :mod:`rm75_drake.limits`）；
* 相邻点**逐轴速度**不超过软件速度上限；
* 相邻点**逐轴加速度**不超过自研默认上限（见下）；
* 附带**夹爪动作调度表**（:class:`rm75_drake.gripper.GripperAction`）。

不变量一律在**构造时**校验，违反抛 :class:`TrajectoryError`——
**绝不静默纠正**（不改 seq、不重采样、不截断越界值），因为静默纠正会让
「越界轨迹看起来合法」，直接击穿 S03 的安全闸。

加速度上限的来源（**重要，不得省略**）
--------------------------------------
官方 RM75 产品参数表**只给角速度**（J1/J2 = 180°/s = 3.1416 rad/s，
J3–J7 = 225°/s = 3.927 rad/s），**不公布加速度上限**；
``models/rm75_drake/joint_limits.yaml`` 里也只有 ``position / velocity / effort``
三段，没有加速度段。因此本模块的加速度上限是**本项目自研默认值**：

* :data:`DEFAULT_ACCELERATION_LIMITS_RAD_S2` = J1/J2 **6.0 rad/s²**，J3–J7 **8.0 rad/s²**
  （≈ 2 倍速度上限所对应的 0.5 s 起步时间）；
* 该常量集中定义在**本文件**，planner 可通过 ``Trajectory(acceleration_limits=...)`` 覆盖；
* **自研默认值，S03 真机层将按实测收紧**（实测前不得把它当成官方真值对外宣称）。

速度上限与位置限位**不是**自研值：它们来自
``models/rm75_drake/joint_limits.yaml`` 的 ``software`` 段（官方参数表真值 + 每侧 4° 收紧），
由 :func:`rm75_drake.limits.load_joint_limits` 读取，本模块不复制常量。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from functools import lru_cache
from typing import List, Optional, Sequence, Tuple

import numpy as np

from .. import limits as _limits
from ..gripper import GripperAction

__all__ = [
    "TrajectoryError",
    "JOINT_COUNT",
    "DEFAULT_DT",
    "DEFAULT_ACCELERATION_LIMITS_RAD_S2",
    "GRID_TOLERANCE",
    "SoftwareLimits",
    "TrajectorySample",
    "Trajectory",
    "software_limits",
    "trajectory_from_arrays",
    "q_matrix",
    "velocities",
    "accelerations",
    "resample",
]


class TrajectoryError(RuntimeError):
    """轨迹不变量被违反。

    错误信息统一带一个**稳定的不变量标签**（``[seq]`` / ``[time_grid]`` /
    ``[joint_count]`` / ``[soft_limits]`` / ``[velocity]`` / ``[acceleration]`` /
    ``[finite]`` / ``[empty]`` / ``[gripper]``），供调用方与测试**按标签**分类，
    避免依赖自然语言措辞。
    """


#: RM75-B 的关节数（7 DoF）。
JOINT_COUNT = 7

#: 默认采样周期：5 ms（真机透传周期）。
DEFAULT_DT = 0.005

#: **自研默认加速度上限**（rad/s²）——官方 RM75 参数表未公布加速度上限。
#:
#: J1/J2 = 6.0、J3–J7 = 8.0，约等于"2 倍速度上限 / 0.5 s 起步时间"。
#: **自研默认值，S03 真机层将按实测收紧**；planner 可在构造 ``Trajectory`` 时覆盖。
DEFAULT_ACCELERATION_LIMITS_RAD_S2: Tuple[float, ...] = (
    6.0,  # joint_1
    6.0,  # joint_2
    8.0,  # joint_3
    8.0,  # joint_4
    8.0,  # joint_5
    8.0,  # joint_6
    8.0,  # joint_7
)

#: 数值比较容差（rad / rad/s / rad/s² / s）。
GRID_TOLERANCE = 1e-9


# ── 软件限位（软限位真值来自 limits.py，不在本模块复制）────────────────
@dataclass(frozen=True)
class SoftwareLimits:
    """软件限位（位置上下限 + 速度上限），按 ``joint_1..joint_7`` 顺序排列。"""

    lower: Tuple[float, ...]
    upper: Tuple[float, ...]
    velocity: Tuple[float, ...]

    def __post_init__(self) -> None:
        if not (len(self.lower) == len(self.upper) == len(self.velocity) == JOINT_COUNT):
            raise TrajectoryError(
                f"[joint_count] 软件限位必须是 {JOINT_COUNT} 维："
                f"lower={len(self.lower)} upper={len(self.upper)} "
                f"velocity={len(self.velocity)}"
            )


@lru_cache(maxsize=4)
def _software_limits_cached(path_key: Optional[str]) -> SoftwareLimits:
    """读取并缓存软件限位。

    ``path_key`` 为 ``None`` 时使用默认交接件
    ``models/rm75_drake/joint_limits.yaml``（经 :mod:`rm75_drake.limits` 定位，
    不依赖 cwd）。
    """
    path = None if path_key is None else _limits.joint_limits_path(path_key)
    data = _limits.load_joint_limits(path)
    joints = data.get("joints")
    if not isinstance(joints, list) or len(joints) != JOINT_COUNT:
        raise TrajectoryError(
            f"[joint_count] 限位交接件的 joints 段必须是 {JOINT_COUNT} 项，"
            f"实际：{joints!r}"
        )
    lower: List[float] = []
    upper: List[float] = []
    velocity: List[float] = []
    for joint in joints:
        software = joint.get("software") or {}
        for key in ("position_lower_rad", "position_upper_rad", "velocity_rad_s"):
            value = software.get(key)
            if not isinstance(value, (int, float)) or isinstance(value, bool):
                raise TrajectoryError(
                    f"[soft_limits] 限位交接件的 {joint.get('name')}.software.{key} "
                    f"缺失或不是数值：{value!r}"
                )
        lower.append(float(software["position_lower_rad"]))
        upper.append(float(software["position_upper_rad"]))
        velocity.append(float(software["velocity_rad_s"]))
    return SoftwareLimits(tuple(lower), tuple(upper), tuple(velocity))


def software_limits(path: Optional[str] = None) -> SoftwareLimits:
    """返回软件限位（位置上下限 + 速度上限）。

    参数
    ----
    path:
        可选的限位交接件路径；``None`` 时用默认交接件
        ``models/rm75_drake/joint_limits.yaml``。

    交接件缺失 / 解析失败 / 结构非法 → 抛
    :class:`rm75_drake.limits.LimitsError`（**不**回退到硬编码常量）。
    """
    return _software_limits_cached(None if path is None else str(path))


# ── 采样点 ──────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class TrajectorySample:
    """轨迹上的一个采样点。

    字段
    ----
    seq:
        序号，**从 0 开始逐点 +1**（跨设备/跨进程可对齐的唯一键）。
    t_abs:
        **绝对时刻**（秒）。首点的绝对时刻由调用方给定（例如与真机时钟对齐），
        相邻点间隔恒为 ``dt``。
    q:
        关节角（rad），长度必须是 :data:`JOINT_COUNT`。
    gripper_action:
        可选的夹爪动作（:class:`rm75_drake.gripper.GripperAction`），
        表示**在该采样点生效**的夹爪指令。
    """

    seq: int
    t_abs: float
    q: Tuple[float, ...]
    gripper_action: Optional[GripperAction] = None

    def __post_init__(self) -> None:
        try:
            q = tuple(float(v) for v in self.q)
        except (TypeError, ValueError) as exc:
            raise TrajectoryError(f"[joint_count] q 必须是数值序列：{self.q!r}") from exc
        if len(q) != JOINT_COUNT:
            raise TrajectoryError(
                f"[joint_count] q 必须是 {JOINT_COUNT} 维（RM75-B 7 DoF），"
                f"实际 {len(q)} 维：{self.q!r}"
            )
        object.__setattr__(self, "q", q)


# ── 轨迹 ────────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class Trajectory:
    """一条已通过全部不变量的关节轨迹。

    字段
    ----
    samples:
        采样点序列（至少 1 个；空序列非法）。
    dt:
        采样周期（s），默认 :data:`DEFAULT_DT`（5 ms）。
    robot_name:
        机器人名（默认 ``"rm75"``），写入录制文件供回放时校验。
    gripper_schedule:
        夹爪动作调度表（:class:`rm75_drake.gripper.GripperAction` 序列）。
    acceleration_limits:
        可选的**逐轴加速度上限**覆盖（rad/s²，长度 7）。
        ``None`` → 使用 :data:`DEFAULT_ACCELERATION_LIMITS_RAD_S2`
        （**自研默认值**，官方未公布加速度上限）。
    """

    samples: Tuple[TrajectorySample, ...]
    dt: float = DEFAULT_DT
    robot_name: str = "rm75"
    gripper_schedule: Tuple[GripperAction, ...] = ()
    acceleration_limits: Optional[Tuple[float, ...]] = None

    # -- 构造与不变量 ---------------------------------------------------
    def __post_init__(self) -> None:
        object.__setattr__(self, "samples", tuple(self.samples))
        object.__setattr__(self, "gripper_schedule", tuple(self.gripper_schedule))
        if self.acceleration_limits is not None:
            object.__setattr__(
                self,
                "acceleration_limits",
                tuple(float(v) for v in self.acceleration_limits),
            )
        self._validate()

    def _validate(self) -> None:
        """逐条校验不变量；违反即抛 :class:`TrajectoryError`（不静默纠正）。"""
        samples = self.samples
        if len(samples) == 0:
            raise TrajectoryError("[empty] 轨迹不能为空：至少需要 1 个采样点")

        dt = float(self.dt)
        if not math.isfinite(dt) or dt <= 0.0:
            raise TrajectoryError(f"[time_grid] dt 必须是正有限值，实际：{self.dt!r}")

        for action in self.gripper_schedule:
            if not isinstance(action, GripperAction):
                raise TrajectoryError(
                    f"[gripper] 夹爪调度表只能包含 GripperAction，实际：{action!r}"
                )
        for sample in samples:
            if sample.gripper_action is not None and not isinstance(
                sample.gripper_action, GripperAction
            ):
                raise TrajectoryError(
                    f"[gripper] 采样点的 gripper_action 只能是 GripperAction 或 None，"
                    f"实际：{sample.gripper_action!r}"
                )

        # (a) seq 严格单调 +1，从 0 开始
        seqs = np.fromiter((s.seq for s in samples), dtype=float, count=len(samples))
        expected = np.arange(len(samples), dtype=float)
        if not np.array_equal(seqs, expected):
            index = int(np.argmax(seqs != expected))
            raise TrajectoryError(
                f"[seq] seq 必须从 0 开始逐点 +1：第 {index} 个采样点的 seq="
                f"{samples[index].seq!r}（期望 {index}）"
            )

        # (b) t_abs 严格递增，且相邻差与 dt 的偏差不超过容差（5 ms 网格）
        times = np.fromiter((s.t_abs for s in samples), dtype=float, count=len(samples))
        if not np.all(np.isfinite(times)):
            index = int(np.argmax(~np.isfinite(times)))
            raise TrajectoryError(
                f"[finite] t_abs 必须是有限值：第 {index} 个采样点为 "
                f"{samples[index].t_abs!r}"
            )
        steps = np.diff(times)
        if steps.size:
            if not np.all(steps > 0.0):
                index = int(np.argmax(steps <= 0.0))
                raise TrajectoryError(
                    f"[time_grid] t_abs 必须严格递增：第 {index} 个采样点 t_abs="
                    f"{samples[index].t_abs!r}，第 {index + 1} 个采样点 t_abs="
                    f"{samples[index + 1].t_abs!r}"
                )
            off_grid = np.abs(steps - dt) > GRID_TOLERANCE
            if np.any(off_grid):
                index = int(np.argmax(off_grid))
                raise TrajectoryError(
                    f"[time_grid] 相邻点间隔必须等于 dt={dt} s"
                    f"（容差 {GRID_TOLERANCE}）：第 {index}→{index + 1} 个采样点"
                    f"实际间隔 {steps[index]!r} s"
                )

        # (c) 逐点 q 落在软件软限位内
        matrix = np.asarray([s.q for s in samples], dtype=float)
        if not np.all(np.isfinite(matrix)):
            row, axis = (int(v) for v in np.argwhere(~np.isfinite(matrix))[0])
            raise TrajectoryError(
                f"[finite] 第 {row} 个采样点第 {axis} 轴的关节角非有限值："
                f"{matrix[row, axis]!r}"
            )
        limits = software_limits()
        lower = np.asarray(limits.lower, dtype=float)
        upper = np.asarray(limits.upper, dtype=float)
        outside = (matrix < lower - GRID_TOLERANCE) | (matrix > upper + GRID_TOLERANCE)
        if np.any(outside):
            row, axis = (int(v) for v in np.argwhere(outside)[0])
            raise TrajectoryError(
                f"[soft_limits] 第 {row} 个采样点第 {axis} 轴关节角 "
                f"{matrix[row, axis]!r} 超出软件软限位 "
                f"[{limits.lower[axis]}, {limits.upper[axis]}]"
            )

        # (d)(e) 相邻点速度 / 加速度上限
        self._validate_velocity(matrix, limits)
        self._validate_acceleration(matrix)

    def _validate_velocity(
        self, matrix: Optional[np.ndarray] = None, limits: Optional[SoftwareLimits] = None
    ) -> None:
        """相邻点逐轴速度不得超过软件速度上限。"""
        if len(self.samples) < 2:
            return
        limits = limits or software_limits()
        matrix = self.q_matrix() if matrix is None else matrix
        dt = float(self.dt)
        speed = np.diff(matrix, axis=0) / dt
        excess = np.abs(speed) - np.asarray(limits.velocity, dtype=float)[None, :]
        bad = np.argwhere(excess > GRID_TOLERANCE)
        if bad.size:
            index, axis = int(bad[0][0]), int(bad[0][1])
            raise TrajectoryError(
                f"[velocity] 第 {index}→{index + 1} 个采样点第 {axis} 轴速度 "
                f"{speed[index, axis]!r} rad/s 超过软件速度上限 "
                f"{limits.velocity[axis]} rad/s"
            )

    def _validate_acceleration(self, matrix: Optional[np.ndarray] = None) -> None:
        """相邻点逐轴加速度不得超过上限（默认自研值，可覆盖）。"""
        if len(self.samples) < 3:
            return
        limits = (
            tuple(self.acceleration_limits)
            if self.acceleration_limits is not None
            else DEFAULT_ACCELERATION_LIMITS_RAD_S2
        )
        if len(limits) != JOINT_COUNT:
            raise TrajectoryError(
                f"[acceleration] 加速度上限必须是 {JOINT_COUNT} 维，实际 {len(limits)} 维"
            )
        matrix = self.q_matrix() if matrix is None else matrix
        dt = float(self.dt)
        accel = np.diff(matrix, n=2, axis=0) / (dt * dt)
        excess = np.abs(accel) - np.asarray(limits, dtype=float)[None, :]
        bad = np.argwhere(excess > GRID_TOLERANCE)
        if bad.size:
            index, axis = int(bad[0][0]), int(bad[0][1])
            raise TrajectoryError(
                f"[acceleration] 第 {index}/{index + 1}/{index + 2} 个采样点第 "
                f"{axis} 轴加速度 {accel[index, axis]!r} rad/s² 超过上限 "
                f"{limits[axis]} rad/s²"
            )

    # -- 便捷访问 -------------------------------------------------------
    def __len__(self) -> int:
        """采样点数。"""
        return len(self.samples)

    @property
    def start_time(self) -> float:
        """首点的绝对时刻（s）。"""
        return self.samples[0].t_abs

    @property
    def end_time(self) -> float:
        """末点的绝对时刻（s）。"""
        return self.samples[-1].t_abs

    @property
    def duration(self) -> float:
        """轨迹时长（s）= 末点绝对时刻 − 首点绝对时刻。"""
        return self.end_time - self.start_time

    def q_matrix(self) -> np.ndarray:
        """返回 ``(N, 7)`` 的关节角矩阵（numpy 视图副本）。"""
        return np.asarray([sample.q for sample in self.samples], dtype=float)

    def axis_q(self, axis: int) -> np.ndarray:
        """返回某一轴的关节角序列（numpy ``(N,)``）。"""
        if not 0 <= int(axis) < JOINT_COUNT:
            raise TrajectoryError(
                f"[joint_count] 轴号必须在 [0, {JOINT_COUNT}) 内，实际：{axis!r}"
            )
        return self.q_matrix()[:, int(axis)]

    def velocities(self) -> np.ndarray:
        """返回相邻点速度矩阵 ``(N-1, 7)``（rad/s）。"""
        return velocities(self)

    def accelerations(self) -> np.ndarray:
        """返回相邻点加速度矩阵 ``(N-2, 7)``（rad/s²）。"""
        return accelerations(self)

    def sample_at(self, t_abs: float) -> TrajectorySample:
        """在**绝对时刻** ``t_abs`` 处线性插值取样。

        返回的采样点：``seq`` 取左侧节点的序号，``gripper_action`` 取左侧节点的动作，
        ``q`` 为线性插值结果。``t_abs`` 超出 ``[start_time, end_time]`` → 抛
        :class:`TrajectoryError`（**不**做端点钳位，否则越界调用会被静默纠正）。
        """
        t = float(t_abs)
        if not math.isfinite(t):
            raise TrajectoryError(f"[finite] 采样时刻必须是有限值，实际：{t_abs!r}")
        if t < self.start_time - GRID_TOLERANCE or t > self.end_time + GRID_TOLERANCE:
            raise TrajectoryError(
                f"[time_grid] 采样时刻 {t!r} 超出轨迹时间范围 "
                f"[{self.start_time}, {self.end_time}]"
            )
        return _interpolate(self, t)


# ── 辅助函数 ────────────────────────────────────────────────────────────
def trajectory_from_arrays(
    q_seq: Sequence[Sequence[float]],
    t0: float = 0.0,
    dt: float = DEFAULT_DT,
    *,
    robot_name: str = "rm75",
    gripper_schedule: Sequence[GripperAction] = (),
    acceleration_limits: Optional[Sequence[float]] = None,
) -> Trajectory:
    """由关节角序列构造 :class:`Trajectory`（seq 从 0 递增，时刻 = ``t0 + k*dt``）。

    参数
    ----
    q_seq:
        ``(N, 7)`` 的关节角序列（rad）。
    t0:
        首点的**绝对时刻**（s），默认 0.0。
    dt:
        采样周期（s），默认 5 ms。

    构造时的不变量校验（seq / 时间网格 / 软限位 / 速度 / 加速度）与
    :class:`Trajectory` 完全一致——违反抛 :class:`TrajectoryError`。
    """
    q_rows = [tuple(float(v) for v in row) for row in q_seq]
    if not q_rows:
        raise TrajectoryError("[empty] 轨迹不能为空：至少需要 1 个采样点")
    start = float(t0)
    step = float(dt)
    samples = tuple(
        TrajectorySample(seq=index, t_abs=start + index * step, q=row)
        for index, row in enumerate(q_rows)
    )
    return Trajectory(
        samples=samples,
        dt=step,
        robot_name=robot_name,
        gripper_schedule=tuple(gripper_schedule),
        acceleration_limits=(
            None if acceleration_limits is None else tuple(acceleration_limits)
        ),
    )


def q_matrix(traj: Trajectory) -> np.ndarray:
    """返回 ``(N, 7)`` 的关节角矩阵（numpy）。"""
    if not isinstance(traj, Trajectory):
        raise TrajectoryError(f"[joint_count] 期望 Trajectory，实际：{traj!r}")
    return traj.q_matrix()


def velocities(traj: Trajectory) -> np.ndarray:
    """返回相邻点速度矩阵 ``(N-1, 7)``（rad/s，前向差分）。"""
    if not isinstance(traj, Trajectory):
        raise TrajectoryError(f"[joint_count] 期望 Trajectory，实际：{traj!r}")
    matrix = traj.q_matrix()
    if matrix.shape[0] < 2:
        return np.zeros((0, JOINT_COUNT), dtype=float)
    return np.diff(matrix, axis=0) / float(traj.dt)


def accelerations(traj: Trajectory) -> np.ndarray:
    """返回相邻点加速度矩阵 ``(N-2, 7)``（rad/s²，二阶差分）。"""
    if not isinstance(traj, Trajectory):
        raise TrajectoryError(f"[joint_count] 期望 Trajectory，实际：{traj!r}")
    matrix = traj.q_matrix()
    if matrix.shape[0] < 3:
        return np.zeros((0, JOINT_COUNT), dtype=float)
    return np.diff(matrix, n=2, axis=0) / (float(traj.dt) ** 2)


def resample(traj: Trajectory, dt: float) -> Trajectory:
    """把轨迹重采样到新的固定网格 ``dt``（线性插值）。

    * 新网格从 ``traj.start_time`` 起，间隔恒为 ``dt``；
    * 末尾不足一个整步的余量**被丢弃**（否则会破坏「相邻差 == dt」的不变量）；
    * 重建后的轨迹**重新走一遍全部不变量校验**——若重采样导致越界（例如网格过粗
      拉直了尖峰），抛 :class:`TrajectoryError`（**不**静默放行）。

    夹爪动作按**左侧原节点**继承；``robot_name`` / ``gripper_schedule`` /
    ``acceleration_limits`` 保持不变。
    """
    if not isinstance(traj, Trajectory):
        raise TrajectoryError(f"[joint_count] 期望 Trajectory，实际：{traj!r}")
    step = float(dt)
    if not math.isfinite(step) or step <= 0.0:
        raise TrajectoryError(f"[time_grid] 重采样 dt 必须是正有限值，实际：{dt!r}")

    start = traj.start_time
    end = traj.end_time
    times: List[float] = []
    index = 0
    while True:
        t = start + index * step
        if t > end + GRID_TOLERANCE:
            break
        # 末尾点若与 end 只差浮点噪声，钳到 end（钳动量 < 1e-12，不破坏 dt 网格）。
        times.append(min(t, end) if abs(t - end) <= 1e-12 else t)
        index += 1
    if not times:
        times = [start]

    samples = tuple(
        _interpolate(traj, t) for t in times
    )
    # 插值结果的 seq 必须重新编号（保持从 0 连续 +1 的不变量）。
    renumbered = tuple(
        TrajectorySample(
            seq=i,
            t_abs=sample.t_abs,
            q=sample.q,
            gripper_action=sample.gripper_action,
        )
        for i, sample in enumerate(samples)
    )
    return Trajectory(
        samples=renumbered,
        dt=step,
        robot_name=traj.robot_name,
        gripper_schedule=traj.gripper_schedule,
        acceleration_limits=traj.acceleration_limits,
    )


def _interpolate(traj: Trajectory, t: float) -> TrajectorySample:
    """在绝对时刻 ``t`` 处做线性插值（内部工具，不做范围校验）。"""
    samples = traj.samples
    if len(samples) == 1:
        only = samples[0]
        return TrajectorySample(
            seq=0,
            t_abs=t,
            q=only.q,
            gripper_action=only.gripper_action,
        )
    if t <= samples[0].t_abs:
        left = right = 0
    elif t >= samples[-1].t_abs:
        left = right = len(samples) - 1
    else:
        # 二分查找 t 所在的区间 [left, right]
        lo, hi = 0, len(samples) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if samples[mid].t_abs <= t:
                lo = mid
            else:
                hi = mid
        left, right = lo, hi
    if left == right:
        node = samples[left]
        return TrajectorySample(
            seq=left,
            t_abs=t,
            q=node.q,
            gripper_action=node.gripper_action,
        )
    t0 = samples[left].t_abs
    t1 = samples[right].t_abs
    ratio = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
    q = tuple(
        samples[left].q[axis] + ratio * (samples[right].q[axis] - samples[left].q[axis])
        for axis in range(JOINT_COUNT)
    )
    return TrajectorySample(
        seq=left,
        t_abs=t,
        q=q,
        gripper_action=samples[left].gripper_action,
    )
