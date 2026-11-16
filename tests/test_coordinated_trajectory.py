"""共享时间轴的双臂轨迹（S05-T02）。

覆盖
----
* 正例：14 维联合路径 → 两条 7 维 ``Trajectory``；``t_abs`` / ``dt``
  **逐点相同**（共享时间轴**由构造保证**）；
* 正例：14 维重定时（维度无关的通用核）→ ``DualTrajectory`` 闭环；
* **负例**：速度超限 / 加速度超限 / 软限位超限 / q 维度不符 / 两条轨迹
  时间轴不一致 → 全部显式抛错（``TrajectoryError``），**绝不静默纠正**。
"""

from __future__ import annotations

import numpy as np
import pytest

from rm75_drake.coordinated.limits import DUAL_JOINT_COUNT, derive_dual_limits
from rm75_drake.coordinated.trajectory import (
    DualTrajectory,
    dual_trajectory_from_arrays,
)
from rm75_drake.planning.retime import retime_path
from rm75_drake.planning.trajectory import (
    DEFAULT_ACCELERATION_LIMITS_RAD_S2,
    DEFAULT_DT,
    Trajectory,
    TrajectoryError,
    software_limits,
    trajectory_from_arrays,
)


def _slow_path(seconds: float = 1.0) -> np.ndarray:
    """一条**慢速**的 14 维联合路径（逐点落在软限位内）。"""
    count = 21
    frac = np.linspace(0.0, 1.0, count)
    left = np.column_stack(
        [
            0.20 * frac,
            -0.10 * frac,
            0.05 * frac,
            np.zeros(count),
            0.10 * frac,
            np.zeros(count),
            0.05 * frac,
        ]
    )
    right = -left
    return np.hstack([left, right])


def _joint_names() -> list[str]:
    from rm75_drake.coordinated.limits import canonical_joint_names

    return list(canonical_joint_names())


# ── 正例 ────────────────────────────────────────────────────────────────
def test_dual_trajectory_shares_time_axis_pointwise():
    """正例：两条 7 维轨迹的 t_abs / dt 逐点相同（共享时间轴）。"""
    dual = dual_trajectory_from_arrays(_slow_path())

    assert isinstance(dual, DualTrajectory)
    assert isinstance(dual.left, Trajectory)
    assert isinstance(dual.right, Trajectory)
    assert len(dual.left) == len(dual.right) == 21
    assert dual.dt == DEFAULT_DT

    left_times = np.asarray([s.t_abs for s in dual.left.samples], dtype=float)
    right_times = np.asarray([s.t_abs for s in dual.right.samples], dtype=float)
    assert np.array_equal(left_times, right_times)
    assert np.allclose(np.diff(left_times), DEFAULT_DT)
    assert dual.start_time == 0.0
    assert dual.duration == pytest.approx(20 * DEFAULT_DT)


def test_dual_trajectory_joint_order_is_left_then_right():
    """正例：14 维列顺序 = 左臂 7 → 右臂 7。"""
    path = _slow_path()
    dual = dual_trajectory_from_arrays(path)
    matrix = dual.q_matrix()
    assert matrix.shape == (21, DUAL_JOINT_COUNT)
    assert np.allclose(matrix[:, :7], path[:, :7])
    assert np.allclose(matrix[:, 7:], path[:, 7:])
    assert np.allclose(dual.axis_q(0), path[:, 0])
    assert np.allclose(dual.axis_q(13), path[:, 13])


def test_dual_trajectory_reuses_s02_invariants():
    """正例：每条 7 维轨迹都通过 S02 的全部不变量（软限位 / 速度 / 加速度）。"""
    dual = dual_trajectory_from_arrays(_slow_path())
    limits = software_limits()
    for traj in (dual.left, dual.right):
        matrix = traj.q_matrix()
        assert np.all(matrix >= np.asarray(limits.lower) - 1e-9)
        assert np.all(matrix <= np.asarray(limits.upper) + 1e-9)
        assert [s.seq for s in traj.samples] == list(range(len(traj.samples)))


def test_dual_trajectory_from_retimed_14d_path():
    """正例：14 维重定时（维度无关核）→ 共享时间轴的双轨迹闭环。"""
    dual_limits = derive_dual_limits(_joint_names())
    path = _slow_path()
    retimed = retime_path(
        path,
        velocity_limits=dual_limits.velocity_array(),
        acceleration_limits=dual_limits.acceleration_array(),
    )
    assert retimed.q.shape[1] == DUAL_JOINT_COUNT
    assert retimed.num_samples >= 2

    dual = dual_trajectory_from_arrays(retimed.q, t0=retimed.t0, dt=retimed.dt)
    assert dual.dt == retimed.dt
    assert len(dual) == retimed.num_samples
    left_times = np.asarray([s.t_abs for s in dual.left.samples], dtype=float)
    right_times = np.asarray([s.t_abs for s in dual.right.samples], dtype=float)
    assert np.array_equal(left_times, right_times)
    assert np.allclose(left_times, retimed.t_abs)


# ── 负例 ────────────────────────────────────────────────────────────────
def test_dual_trajectory_rejects_wrong_dimension():
    """负例：q 维度不是 14 → 显式报错并点名期望 / 实际。"""
    with pytest.raises(TrajectoryError) as excinfo:
        dual_trajectory_from_arrays(np.zeros((5, 13)))
    message = str(excinfo.value)
    assert "[joint_count]" in message
    assert "14" in message


def test_dual_trajectory_rejects_velocity_violation():
    """负例：相邻点速度超限 → TrajectoryError（S02 的 [velocity] 标签）。"""
    path = _slow_path()
    path[1:, 0] = path[0, 0] + 2.0  # 单步 2 rad >> 速度上限
    with pytest.raises(TrajectoryError) as excinfo:
        dual_trajectory_from_arrays(path)
    assert "[velocity]" in str(excinfo.value)


def test_dual_trajectory_rejects_acceleration_violation():
    """负例：相邻点加速度超限 → TrajectoryError（S02 的 [acceleration] 标签）。

    构造**只**违反加速度、不违反速度的路径：
    速度 ≤ 2.04 rad/s（< 3.1416 rad/s 上限），加速度 = 8 rad/s²（> 6 rad/s² 上限）。
    """
    path = np.zeros((4, DUAL_JOINT_COUNT))
    path[1, 0] = 0.010
    path[2, 0] = 0.0202
    path[3, 0] = 0.030
    with pytest.raises(TrajectoryError) as excinfo:
        dual_trajectory_from_arrays(path)
    assert "[acceleration]" in str(excinfo.value)


def test_dual_trajectory_rejects_soft_limit_violation():
    """负例：软限位超限 → TrajectoryError（S02 的 [soft_limits] 标签）。"""
    limits = software_limits()
    path = _slow_path()
    path[:, 1] = limits.upper[1] + 0.1  # 越过 J2 软件软限位
    with pytest.raises(TrajectoryError) as excinfo:
        dual_trajectory_from_arrays(path)
    assert "[soft_limits]" in str(excinfo.value)


def test_dual_trajectory_rejects_mismatched_time_axis():
    """负例：两条轨迹的 dt 不同 → 显式报错（共享时间轴不可协商）。"""
    left = trajectory_from_arrays(np.zeros((3, 7)), dt=0.005)
    right = trajectory_from_arrays(np.zeros((3, 7)), dt=0.01)
    with pytest.raises(TrajectoryError) as excinfo:
        DualTrajectory(left=left, right=right)
    assert "[time_axis]" in str(excinfo.value)


def test_dual_trajectory_rejects_mismatched_sample_count():
    """负例：采样点数不同 → 显式报错。"""
    left = trajectory_from_arrays(np.zeros((3, 7)))
    right = trajectory_from_arrays(np.zeros((5, 7)))
    with pytest.raises(TrajectoryError) as excinfo:
        DualTrajectory(left=left, right=right)
    assert "[time_axis]" in str(excinfo.value)


def test_dual_trajectory_rejects_non_trajectory_members():
    """负例：成员不是 Trajectory → 显式报错。"""
    with pytest.raises(TrajectoryError) as excinfo:
        DualTrajectory(left=np.zeros((3, 7)), right=np.zeros((3, 7)))
    assert "[joint_count]" in str(excinfo.value)


def test_dual_trajectory_rejects_bad_axis_index():
    """负例：轴号越界 → 显式报错。"""
    dual = dual_trajectory_from_arrays(_slow_path())
    with pytest.raises(TrajectoryError) as excinfo:
        dual.axis_q(DUAL_JOINT_COUNT)
    assert "[joint_count]" in str(excinfo.value)


def test_dual_trajectory_rejects_bad_acceleration_limits():
    """负例：加速度上限维度不符 → 显式报错（由 S02 的 Trajectory 抛出）。

    正例：7 维的 S02 自研默认值对两臂各自合法（**两臂共享同一组数值**）。
    """
    dual = dual_trajectory_from_arrays(
        _slow_path(),
        acceleration_limits=DEFAULT_ACCELERATION_LIMITS_RAD_S2,
    )
    assert dual.left.acceleration_limits == tuple(DEFAULT_ACCELERATION_LIMITS_RAD_S2)
    assert dual.right.acceleration_limits == tuple(DEFAULT_ACCELERATION_LIMITS_RAD_S2)

    with pytest.raises(TrajectoryError) as excinfo:
        trajectory_from_arrays(
            np.zeros((3, 7)), acceleration_limits=(1.0, 2.0)
        )
    assert "[acceleration]" in str(excinfo.value)
