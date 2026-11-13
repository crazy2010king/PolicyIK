"""S02-T06：Toppra 重定时 + 5 ms 网格采样 + C2 连续与限值复核。

覆盖
----
* 正例：直线关节路径重定时 → 5 ms 网格、总时长 > 0、峰值速度/加速度不超限、
  相邻速度差连续、轨迹通过 S02-T02 的不变量校验；
* 负例：``dt`` 非正、路径形状/数值非法、零长度路径、时长上限不可行
  （错误信息区分**速度**卡点还是**加速度**卡点）、jerk 上限过紧导致的
  连续性违规；
* ``check_c2_continuity`` 的直接单元测试（首个违规索引）。

标记 ``@pytest.mark.slow``（规划重负载用例）。
"""

from __future__ import annotations

import numpy as np
import pytest

from rm75_drake.planning.retime import (
    DEFAULT_JERK_LIMIT_RAD_S3,
    RetimeError,
    RetimedTrajectory,
    check_c2_continuity,
    retime,
    retime_from_trajectory,
)
from rm75_drake.planning.trajectory import (
    DEFAULT_ACCELERATION_LIMITS_RAD_S2,
    DEFAULT_DT,
    Trajectory,
    TrajectoryError,
    software_limits,
    trajectory_from_arrays,
)

pytestmark = pytest.mark.slow

#: 速度上限（软件限位真值，来自 joint_limits.yaml）。
VEL_LIMITS = software_limits().velocity
#: 加速度上限（**自研默认值**，官方 RM75 参数表未公布加速度上限）。
ACC_LIMITS = DEFAULT_ACCELERATION_LIMITS_RAD_S2

#: 直线关节路径：起点全零 → 终点小幅偏移。
STRAIGHT_PATH = np.array(
    [
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.20, -0.10, 0.05, 0.0, 0.10, 0.0, 0.05],
    ]
)


def _assert_valid_grid(result: RetimedTrajectory, dt: float = DEFAULT_DT) -> None:
    """断言 5 ms 网格与时间戳严格递增（容差 1e-9）。"""
    samples = result.samples
    assert len(samples) >= 2
    times = np.array([s.t_abs for s in samples], dtype=float)
    steps = np.diff(times)
    assert np.all(steps > 0.0)
    assert np.max(np.abs(steps - dt)) <= 1e-9
    assert [s.seq for s in samples] == list(range(len(samples)))


# ── 正例 ────────────────────────────────────────────────────────────────
def test_retime_straight_path_produces_valid_5ms_trajectory():
    """正例：直线关节路径重定时后落在 5 ms 网格且满足全部限值。"""
    result = retime(STRAIGHT_PATH, velocity_limits=VEL_LIMITS, acceleration_limits=ACC_LIMITS)

    assert isinstance(result, RetimedTrajectory)
    _assert_valid_grid(result)

    # 总时长 > 0
    assert result.duration > 0.0
    assert result.trajectory.duration == pytest.approx(result.duration)

    # 峰值速度 / 加速度不超限（逐轴）
    for axis in range(7):
        assert result.peak_velocity[axis] <= VEL_LIMITS[axis] + 1e-9
        assert result.peak_acceleration[axis] <= ACC_LIMITS[axis] + 1e-9

    # 统计信息非负且与轨迹一致
    assert result.max_step_velocity >= 0.0
    assert result.max_step_displacement >= 0.0
    assert result.max_step_displacement <= result.max_step_velocity * DEFAULT_DT + 1e-9

    # 相邻速度差连续（离散 C2 复核的等价条件）
    vel = result.trajectory.velocities()
    jumps = np.abs(np.diff(vel, axis=0))
    assert np.all(jumps <= np.asarray(ACC_LIMITS)[None, :] * DEFAULT_DT + 1e-9)


def test_retime_output_trajectory_passes_s02_t02_invariants():
    """正例：输出轨迹可被 S02-T02 的校验函数直接接受（同一套不变量）。"""
    result = retime(STRAIGHT_PATH)
    traj = result.trajectory

    assert isinstance(traj, Trajectory)
    # 构造即校验 → 不抛 TrajectoryError 即通过
    again = trajectory_from_arrays(
        traj.q_matrix(),
        t0=traj.start_time,
        dt=traj.dt,
        robot_name=traj.robot_name,
        acceleration_limits=ACC_LIMITS,
    )
    assert np.allclose(again.q_matrix(), traj.q_matrix(), atol=0.0, rtol=0.0)

    # 软限位：逐点落在软件软限位内
    limits = software_limits()
    matrix = traj.q_matrix()
    assert np.all(matrix >= np.asarray(limits.lower) - 1e-9)
    assert np.all(matrix <= np.asarray(limits.upper) + 1e-9)


def test_retime_preserves_path_endpoints():
    """正例：重定时保持起点与终点（不丢最后一段运动）。"""
    result = retime(STRAIGHT_PATH)
    matrix = result.q_matrix
    assert np.allclose(matrix[0], STRAIGHT_PATH[0], atol=1e-9)
    assert np.allclose(matrix[-1], STRAIGHT_PATH[-1], atol=1e-6)


def test_retime_from_trajectory_roundtrip():
    """正例：retime_from_trajectory 消费已有 Trajectory 并保留元数据。"""
    seed = trajectory_from_arrays(
        np.linspace(STRAIGHT_PATH[0], STRAIGHT_PATH[1], 20),
        t0=1.25,
        dt=DEFAULT_DT,
        robot_name="rm75",
    )
    result = retime_from_trajectory(seed)
    assert result.trajectory.start_time == pytest.approx(1.25)
    assert result.trajectory.robot_name == "rm75"
    _assert_valid_grid(result)
    assert result.duration > 0.0


def test_retime_accepts_custom_limits_and_slows_down():
    """正例：更紧的速度上限 → 轨迹更慢，但仍在限值内。"""
    tight = np.full(7, 0.5)
    result = retime(STRAIGHT_PATH, velocity_limits=tight, acceleration_limits=ACC_LIMITS)
    assert result.peak_velocity_max <= 0.5 + 1e-9
    assert result.duration > 0.0
    _assert_valid_grid(result)


# ── 负例 ────────────────────────────────────────────────────────────────
def test_retime_rejects_non_positive_dt():
    """负例：dt 非正 → 抛错（标签 [dt]）。"""
    for bad_dt in (0.0, -0.005):
        with pytest.raises(RetimeError) as excinfo:
            retime(STRAIGHT_PATH, dt=bad_dt)
        assert "[dt]" in str(excinfo.value)


def test_retime_rejects_bad_shape_and_non_finite():
    """负例：路径维度/点数/数值非法 → 抛错（标签 [shape] / [finite]）。"""
    with pytest.raises(RetimeError) as excinfo:
        retime(np.zeros((3, 6)))
    assert "[shape]" in str(excinfo.value)

    with pytest.raises(RetimeError) as excinfo:
        retime(np.zeros((1, 7)))
    assert "[shape]" in str(excinfo.value)

    broken = STRAIGHT_PATH.copy()
    broken[1, 0] = np.nan
    with pytest.raises(RetimeError) as excinfo:
        retime(broken)
    assert "[finite]" in str(excinfo.value)


def test_retime_rejects_zero_length_path():
    """负例：所有路径点重合 → 抛错（标签 [path]）。"""
    with pytest.raises(RetimeError) as excinfo:
        retime(np.zeros((4, 7)))
    assert "[path]" in str(excinfo.value)


def test_retime_rejects_invalid_limits():
    """负例：上限维度错 / 非正 → 抛错（标签 [limits]）。"""
    with pytest.raises(RetimeError) as excinfo:
        retime(STRAIGHT_PATH, velocity_limits=np.full(7, 1.0), acceleration_limits=np.zeros(7))
    assert "[limits]" in str(excinfo.value)

    with pytest.raises(RetimeError) as excinfo:
        retime(STRAIGHT_PATH, velocity_limits=np.full(3, 1.0))
    assert "[limits]" in str(excinfo.value)


def test_retime_infeasible_duration_reports_binding_limit():
    """负例：时长上限不可行 → 抛错，并说明是**速度**还是**加速度**卡点。

    两条分支：
    * 平均速度需求超过最大速度上限 → 报「速度上限卡住」；
    * 平均速度需求在速度上限内，但起步/刹停所需加速度超限 → 报「加速度上限卡住」。
    """
    path_length = float(np.linalg.norm(STRAIGHT_PATH[-1] - STRAIGHT_PATH[0]))

    # 速度卡住：0.01 s 内走完 ≈0.25 rad → 平均速度需求远超速度上限
    with pytest.raises(RetimeError) as excinfo:
        retime(
            STRAIGHT_PATH,
            velocity_limits=VEL_LIMITS,
            acceleration_limits=ACC_LIMITS,
            duration_limit=0.01,
        )
    message = str(excinfo.value)
    assert "[duration]" in message
    assert "速度上限卡住" in message

    # 加速度卡住：0.2 s 预算 → 平均速度需求在速度上限内，但加速度不够
    limit = 0.2
    assert path_length / limit <= max(VEL_LIMITS)
    with pytest.raises(RetimeError) as excinfo:
        retime(
            STRAIGHT_PATH,
            velocity_limits=VEL_LIMITS,
            acceleration_limits=ACC_LIMITS,
            duration_limit=limit,
        )
    message = str(excinfo.value)
    assert "[duration]" in message
    assert "加速度上限卡住" in message


def test_retime_tight_jerk_limit_triggers_continuity_failure():
    """负例：jerk 上限过紧 → 连续性复核报错（消息含 [continuity]）。"""
    with pytest.raises(RetimeError) as excinfo:
        retime(STRAIGHT_PATH, jerk_limit=1.0)
    assert "[continuity]" in str(excinfo.value)


def test_retime_from_trajectory_rejects_wrong_type():
    """负例：retime_from_trajectory 传入非 Trajectory → 抛错。"""
    with pytest.raises(RetimeError) as excinfo:
        retime_from_trajectory(STRAIGHT_PATH)
    assert "[shape]" in str(excinfo.value)


# ── check_c2_continuity 的直接单元测试 ────────────────────────────────
def test_check_c2_continuity_passes_on_smooth_profile():
    """正例：平滑速度/加速度序列通过连续性复核。"""
    dt = DEFAULT_DT
    n = 40
    t = np.arange(n) * dt
    vel = np.tile((0.5 * t[:, None]), (1, 7))
    acc = np.tile((0.5 * np.ones((n, 1))), (1, 7))
    check_c2_continuity(vel, acc, dt=dt, acceleration_limits=ACC_LIMITS)


def test_check_c2_continuity_reports_first_violating_index():
    """负例：速度跳变超差 → 抛错并给出首个违规索引。"""
    dt = DEFAULT_DT
    vel = np.zeros((10, 7))
    vel[4, 2] = 100.0  # 第 4 行与第 5 行之间出现巨大跳变
    acc = np.zeros((8, 7))
    with pytest.raises(RetimeError) as excinfo:
        check_c2_continuity(vel, acc, dt=dt, acceleration_limits=ACC_LIMITS)
    message = str(excinfo.value)
    assert "[continuity]" in message
    assert "第 3→4" in message


def test_check_c2_continuity_rejects_bad_inputs():
    """负例：dt 非正 / 上限维度错 → 抛错。"""
    with pytest.raises(RetimeError) as excinfo:
        check_c2_continuity(np.zeros((3, 7)), np.zeros((2, 7)), dt=0.0, acceleration_limits=ACC_LIMITS)
    assert "[dt]" in str(excinfo.value)

    with pytest.raises(RetimeError) as excinfo:
        check_c2_continuity(
            np.zeros((3, 7)), np.zeros((2, 7)), dt=DEFAULT_DT, acceleration_limits=np.zeros(3)
        )
    assert "[limits]" in str(excinfo.value)


def test_default_jerk_limit_is_documented_self_research_value():
    """正例：jerk 上限是**自研默认值**（官方未公布），常量存在且为正。"""
    assert DEFAULT_JERK_LIMIT_RAD_S3 > 0.0
