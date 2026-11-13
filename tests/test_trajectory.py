"""S02-T02：轨迹数据结构与不变量的测试。

覆盖：
* **正例**：合成一条满足全部不变量的 5 ms 轨迹（seq / 绝对时刻 / 软限位 / 速度 / 加速度）；
* **负例**：seq 跳号、时间不递增、时间网格错位、超软限位、超速、超加速、空轨迹、
  q 维度不是 7 —— 每条都断言抛出 :class:`TrajectoryError`，且错误信息**指出是哪条不变量**
  （靠稳定的不变量标签断言，不依赖自然语言措辞）。
"""

from __future__ import annotations

import math
import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

from rm75_drake.gripper import close_action, open_action
from rm75_drake.planning import trajectory as tj
from rm75_drake.planning.trajectory import (
    DEFAULT_ACCELERATION_LIMITS_RAD_S2,
    DEFAULT_DT,
    JOINT_COUNT,
    Trajectory,
    TrajectoryError,
    TrajectorySample,
    accelerations,
    q_matrix,
    resample,
    software_limits,
    trajectory_from_arrays,
    velocities,
)

SRC_DIR = Path(__file__).resolve().parents[1] / "src"

#: 合成正例用的参数：振幅 0.1 rad、频率 0.5 Hz。
#: 峰值速度 = 0.1 * 2π * 0.5 ≈ 0.314 rad/s（限值 3.1416 / 3.927）；
#: 峰值加速度 = 0.1 * (2π * 0.5)² ≈ 0.987 rad/s²（自研默认限值 6.0 / 8.0）。
_AMP = 0.1
_FREQ_HZ = 0.5
_N = 201


def _sine_rows(n: int = _N, amp: float = _AMP, freq: float = _FREQ_HZ):
    """合成 ``(N, 7)`` 的正弦关节序列（每轴相位错开）。"""
    rows = []
    for k in range(n):
        t = k * DEFAULT_DT
        rows.append(
            tuple(
                amp * math.sin(2.0 * math.pi * freq * t + 0.3 * axis)
                for axis in range(JOINT_COUNT)
            )
        )
    return rows


def _valid_trajectory(**kwargs) -> Trajectory:
    """构造一条满足全部不变量的 5 ms 轨迹（测试基线）。"""
    return trajectory_from_arrays(_sine_rows(), t0=0.0, dt=DEFAULT_DT, **kwargs)


# ── 正例 ────────────────────────────────────────────────────────────────
def test_valid_trajectory_passes_all_invariants():
    traj = _valid_trajectory()

    assert len(traj) == _N
    assert traj.dt == DEFAULT_DT
    assert traj.start_time == 0.0
    assert traj.duration == pytest.approx((_N - 1) * DEFAULT_DT)

    # seq 从 0 逐点 +1
    assert [s.seq for s in traj.samples] == list(range(_N))
    # t_abs 严格递增且落在 5 ms 网格上
    steps = [
        traj.samples[i + 1].t_abs - traj.samples[i].t_abs for i in range(len(traj) - 1)
    ]
    assert all(abs(step - DEFAULT_DT) <= tj.GRID_TOLERANCE for step in steps)


def test_q_matrix_and_axis_view():
    traj = _valid_trajectory()

    matrix = q_matrix(traj)
    assert matrix.shape == (_N, JOINT_COUNT)
    assert matrix.dtype == float
    # 模块函数与方法给出一致的 numpy 视图
    assert np.array_equal(matrix, traj.q_matrix())
    assert traj.axis_q(0).shape == (_N,)


def test_velocities_and_accelerations_within_limits():
    traj = _valid_trajectory()
    limits = software_limits()

    vel = velocities(traj)
    acc = accelerations(traj)
    assert vel.shape == (_N - 1, JOINT_COUNT)
    assert acc.shape == (_N - 2, JOINT_COUNT)

    assert np.all(
        np.max(np.abs(vel), axis=0) <= np.asarray(limits.velocity) + 1e-9
    )
    assert np.all(
        np.max(np.abs(acc), axis=0)
        <= np.asarray(DEFAULT_ACCELERATION_LIMITS_RAD_S2) + 1e-9
    )


def test_sample_at_interpolates():
    traj = _valid_trajectory()
    matrix = traj.q_matrix()

    t = 10 * DEFAULT_DT + 0.5 * DEFAULT_DT  # 第 10 与第 11 点之间
    sample = traj.sample_at(t)
    expected = 0.5 * (matrix[10] + matrix[11])
    assert np.allclose(sample.q, expected, atol=1e-12)
    assert sample.seq == 10
    assert sample.t_abs == pytest.approx(t)

    # 落在节点上时取该节点原值
    node = traj.sample_at(10 * DEFAULT_DT)
    assert np.allclose(node.q, matrix[10], atol=1e-15)
    assert node.seq == 10


def test_absolute_time_offset_is_preserved():
    traj = trajectory_from_arrays(_sine_rows(n=50), t0=12.5, dt=DEFAULT_DT)
    assert traj.start_time == pytest.approx(12.5)
    assert traj.end_time == pytest.approx(12.5 + 49 * DEFAULT_DT)
    assert traj.duration == pytest.approx(49 * DEFAULT_DT)


def test_gripper_schedule_and_sample_action():
    schedule = (open_action(speed=80, hold_seconds=0.2), close_action())
    samples = [
        TrajectorySample(seq=0, t_abs=0.0, q=tuple([0.0] * 7), gripper_action=schedule[0]),
        TrajectorySample(seq=1, t_abs=DEFAULT_DT, q=tuple([0.0] * 7)),
    ]
    traj = Trajectory(samples=samples, dt=DEFAULT_DT, gripper_schedule=schedule)

    assert traj.gripper_schedule == schedule
    assert traj.samples[0].gripper_action == schedule[0]
    assert traj.samples[1].gripper_action is None


def test_acceleration_limits_can_be_overridden_by_planner():
    """planner 可覆盖自研默认加速度上限（官方未公布加速度上限，默认值只作占位）。"""
    # 峰值加速度 ≈ 0.987 rad/s²：放宽到 2.0 通过，收紧到 0.5 则拒绝
    relaxed = Trajectory(
        samples=tuple(
            TrajectorySample(
                seq=i, t_abs=i * DEFAULT_DT, q=row
            )
            for i, row in enumerate(_sine_rows())
        ),
        dt=DEFAULT_DT,
        acceleration_limits=(2.0,) * JOINT_COUNT,
    )
    assert relaxed is not None  # 构造成功即通过

    with pytest.raises(TrajectoryError, match=r"\[acceleration\]"):
        Trajectory(
            samples=tuple(
                TrajectorySample(seq=i, t_abs=i * DEFAULT_DT, q=row)
                for i, row in enumerate(_sine_rows())
            ),
            dt=DEFAULT_DT,
            acceleration_limits=(0.1,) * JOINT_COUNT,
        )


def test_resample_to_finer_grid_keeps_values_and_invariants():
    traj = _valid_trajectory()
    fine = resample(traj, DEFAULT_DT / 2)

    assert fine.dt == DEFAULT_DT / 2
    assert len(fine) == 2 * (_N - 1) + 1
    assert fine.start_time == pytest.approx(traj.start_time)
    # 原节点上的取值保持不变
    assert np.allclose(fine.q_matrix()[::2], traj.q_matrix(), atol=1e-12)
    # 重采样结果依然满足全部不变量（构造即校验）
    assert fine.duration == pytest.approx(traj.duration)


# ── 负例：每条都断言错误信息指出了具体不变量 ────────────────────────────
def test_empty_trajectory_rejected():
    with pytest.raises(TrajectoryError, match=r"\[empty\]"):
        Trajectory(samples=(), dt=DEFAULT_DT)


def test_seq_gap_rejected():
    """seq 跳号（缺 2）→ [seq]。"""
    samples = [
        TrajectorySample(seq=0, t_abs=0.0, q=tuple([0.0] * 7)),
        TrajectorySample(seq=1, t_abs=DEFAULT_DT, q=tuple([0.0] * 7)),
        TrajectorySample(seq=3, t_abs=2 * DEFAULT_DT, q=tuple([0.0] * 7)),
    ]
    with pytest.raises(TrajectoryError, match=r"\[seq\]"):
        Trajectory(samples=samples, dt=DEFAULT_DT)


def test_seq_not_starting_at_zero_rejected():
    samples = [
        TrajectorySample(seq=1, t_abs=0.0, q=tuple([0.0] * 7)),
        TrajectorySample(seq=2, t_abs=DEFAULT_DT, q=tuple([0.0] * 7)),
    ]
    with pytest.raises(TrajectoryError, match=r"\[seq\]"):
        Trajectory(samples=samples, dt=DEFAULT_DT)


def test_non_increasing_time_rejected():
    """t_abs 不递增 → [time_grid]。"""
    samples = [
        TrajectorySample(seq=0, t_abs=0.0, q=tuple([0.0] * 7)),
        TrajectorySample(seq=1, t_abs=DEFAULT_DT, q=tuple([0.0] * 7)),
        TrajectorySample(seq=2, t_abs=0.5 * DEFAULT_DT, q=tuple([0.0] * 7)),
    ]
    with pytest.raises(TrajectoryError, match=r"\[time_grid\]"):
        Trajectory(samples=samples, dt=DEFAULT_DT)


def test_off_grid_time_rejected():
    """相邻点间隔与 dt 的偏差超过 1e-9 → [time_grid]。"""
    samples = [
        TrajectorySample(seq=0, t_abs=0.0, q=tuple([0.0] * 7)),
        TrajectorySample(seq=1, t_abs=DEFAULT_DT + 1e-6, q=tuple([0.0] * 7)),
    ]
    with pytest.raises(TrajectoryError, match=r"\[time_grid\]"):
        Trajectory(samples=samples, dt=DEFAULT_DT)


def test_non_positive_dt_rejected():
    samples = [
        TrajectorySample(seq=0, t_abs=0.0, q=tuple([0.0] * 7)),
        TrajectorySample(seq=1, t_abs=DEFAULT_DT, q=tuple([0.0] * 7)),
    ]
    with pytest.raises(TrajectoryError, match=r"\[time_grid\]"):
        Trajectory(samples=samples, dt=0.0)


def test_soft_limit_violation_rejected():
    """关节角超出软件软限位 → [soft_limits]（joint_1 软件上限 ≈ 3.036 rad）。"""
    limits = software_limits()
    q = tuple([0.0] * JOINT_COUNT)
    q = (limits.upper[0] + 0.5,) + q[1:]
    samples = [
        TrajectorySample(seq=0, t_abs=0.0, q=q),
        TrajectorySample(seq=1, t_abs=DEFAULT_DT, q=q),
    ]
    with pytest.raises(TrajectoryError, match=r"\[soft_limits\]"):
        Trajectory(samples=samples, dt=DEFAULT_DT)


def test_velocity_violation_rejected():
    """相邻点速度超上限 → [velocity]（常量速度 10 rad/s > 3.1416 rad/s）。"""
    speed = 10.0
    samples = []
    for k in range(21):
        q = tuple([0.0] * JOINT_COUNT)
        q = (speed * k * DEFAULT_DT,) + q[1:]
        samples.append(TrajectorySample(seq=k, t_abs=k * DEFAULT_DT, q=q))
    with pytest.raises(TrajectoryError, match=r"\[velocity\]"):
        Trajectory(samples=samples, dt=DEFAULT_DT)


def test_acceleration_violation_rejected():
    """相邻点加速度超上限 → [acceleration]（速度阶跃 3.0 rad/s → 600 rad/s²）。"""
    speed = 3.0  # < 3.1416 rad/s（速度合法）
    switch_index = 5
    samples = []
    for k in range(21):
        q = tuple([0.0] * JOINT_COUNT)
        if k >= switch_index:
            q = (speed * (k - (switch_index - 1)) * DEFAULT_DT,) + q[1:]
        samples.append(TrajectorySample(seq=k, t_abs=k * DEFAULT_DT, q=q))
    with pytest.raises(TrajectoryError, match=r"\[acceleration\]"):
        Trajectory(samples=samples, dt=DEFAULT_DT)


def test_wrong_joint_dimension_rejected():
    """q 维度不是 7 → [joint_count]。"""
    with pytest.raises(TrajectoryError, match=r"\[joint_count\]"):
        TrajectorySample(seq=0, t_abs=0.0, q=(0.0, 0.1, 0.2))
    with pytest.raises(TrajectoryError, match=r"\[joint_count\]"):
        Trajectory(
            samples=(
                TrajectorySample(seq=0, t_abs=0.0, q=(0.0,) * 6),
                TrajectorySample(seq=1, t_abs=DEFAULT_DT, q=(0.0,) * 6),
            ),
            dt=DEFAULT_DT,
        )


def test_non_finite_joint_value_rejected():
    samples = [
        TrajectorySample(seq=0, t_abs=0.0, q=(float("nan"),) + (0.0,) * 6),
        TrajectorySample(seq=1, t_abs=DEFAULT_DT, q=(0.0,) * 7),
    ]
    with pytest.raises(TrajectoryError, match=r"\[finite\]"):
        Trajectory(samples=samples, dt=DEFAULT_DT)


def test_sample_at_out_of_range_rejected():
    traj = _valid_trajectory()
    with pytest.raises(TrajectoryError, match=r"\[time_grid\]"):
        traj.sample_at(traj.end_time + 1.0)
    with pytest.raises(TrajectoryError, match=r"\[time_grid\]"):
        traj.sample_at(traj.start_time - 1.0)


def test_resample_rejects_non_positive_dt():
    traj = _valid_trajectory()
    with pytest.raises(TrajectoryError, match=r"\[time_grid\]"):
        resample(traj, 0.0)


def test_trajectory_from_arrays_rejects_empty():
    with pytest.raises(TrajectoryError, match=r"\[empty\]"):
        trajectory_from_arrays([])


# ── 依赖边界：本模块不得引入 pydrake 硬依赖 ─────────────────────────────
def test_module_has_no_pydrake_dependency():
    """``rm75_drake.planning.trajectory`` 不得把 pydrake 拉进 import 图。

    S03 实时层与 S07 离线回归要复用这个结构，不应被迫安装 Drake。
    """
    code = (
        "import sys;"
        "import rm75_drake.planning.trajectory as t;"
        "assert 'pydrake' not in sys.modules, "
        "sorted(k for k in sys.modules if 'pydrake' in k)"
    )
    env = dict(os.environ)
    env["PYTHONPATH"] = str(SRC_DIR)
    result = subprocess.run(
        [sys.executable, "-c", code],
        env=env,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr
