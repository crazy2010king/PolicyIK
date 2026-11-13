"""IK 模块测试（S02-T03）：正例 + 负例 + 独立 FK 交叉校验。

被测对象
--------
:mod:`rm75_drake.planning.ik` —— 任务空间目标 → 关节位形，含**可解释失败**。

覆盖的契约
----------
**正例**
* 可达目标能收敛（末端落在 ``X ≈ [0.35, 0.0, 0.55]`` m）；
* 收敛解逐轴落在**软件软限位**内；
* 用 **MDH 独立正解**（S01 的零 Drake 依赖基准）交叉校验末端位置，误差 < 1e-3 m
  ——这是"被测对照"约定：Drake FK 不自己验自己。

**负例（可解释失败）**
* 目标离工作空间极远（> 2 m）→ ``success=False`` 且 message 非空；
* 目标含 NaN → 抛明确异常（``IkError``），不是静默失败；
* 起点软限位越界 → ``success=False`` 且 message 点名越界关节；
* 超时预算耗尽 → ``reason="timeout"``；
* 名义姿态全零 → 抛异常（零位 6 轴退化，必须显式拒绝）；
* 非法参数（容差 ≤ 0、迭代次数 ≤ 0）→ 抛异常。

场景装配复用 ``tests/conftest.py`` 的 ``single_arm_scene`` fixture
（S02-T01 的 ``build_scene`` 是唯一装配入口，测试里**不重复造场景**）。
"""

from __future__ import annotations

import numpy as np
import pytest

from rm75_drake.kinematics import mdh
from rm75_drake.planning import ik as ik_mod

#: 参考位形：其 MDH 正解末端位置恰为 ``[0.35, 0.0, 0.55]`` m
#: （用 :mod:`rm75_drake.kinematics.mdh` 在软限位内数值精修得到，残差 < 1e-16 m）。
REFERENCE_Q = (
    2.708359,
    -0.762284,
    -2.961942,
    0.130607,
    1.449474,
    1.702438,
    3.062573,
)

#: 参考位形的 MDH 末端位置（m）。
REFERENCE_POSITION = np.array([0.35, 0.0, 0.55])

#: 交叉校验阈值（m）。MDH 与 URDF 链的已知残差 ~2.2e-6 m（见 mdh 模块 docstring），
#: 加上 IK 位置容差 1e-4 m，1e-3 m 是有鉴别力又不苛刻的阈值。
MDH_POSITION_TOLERANCE = 1e-3


def fk_pose(scene, q):
    """用场景的 plant 做正运动学，返回末端 frame 在世界系下的位姿。"""
    plant = scene.plant
    context = plant.CreateDefaultContext()
    plant.SetPositions(context, np.asarray(q, dtype=float))
    return plant.CalcRelativeTransform(
        context, plant.world_frame(), plant.GetFrameByName(ik_mod.EE_FRAME_NAME)
    )


class _NonFinitePose:
    """位姿替身：位置含 NaN（Drake 的 RigidTransform 构造器会自己拒绝 NaN，
    因此用替身把"调用方传进来的坏位姿"送到 IK 的校验分支上）。"""

    def __init__(self, translation):
        self._translation = np.asarray(translation, dtype=float)

    def translation(self):
        return self._translation

    def rotation(self):
        from pydrake.math import RotationMatrix

        return RotationMatrix.Identity()


# ── 正例 ────────────────────────────────────────────────────────────────
def test_nominal_posture_is_not_zero_and_within_soft_limits():
    """名义姿态不得为全零，且必须逐轴落在软件软限位内。"""
    assert any(abs(v) > 0.0 for v in ik_mod.NOMINAL_POSTURE)
    names, lower, upper = ik_mod.software_joint_limits()
    assert len(names) == 7
    posture = np.asarray(ik_mod.NOMINAL_POSTURE, dtype=float)
    assert np.all(posture >= lower - 1e-9)
    assert np.all(posture <= upper + 1e-9)


def test_solve_ik_reaches_reference_workspace_point(single_arm_scene):
    """正例：可达目标收敛，且位置/姿态残差都进容差。"""
    target = fk_pose(single_arm_scene, REFERENCE_Q)
    result = ik_mod.solve_ik(single_arm_scene, target)

    assert result.success, result.message
    assert result.reason == ik_mod.REASON_NONE
    assert result.iterations > 0
    assert result.message
    assert result.position_error <= 1e-4
    assert result.orientation_error <= 1e-3
    assert len(result.q) == 7


def test_solution_is_within_software_limits(single_arm_scene):
    """正例：收敛解逐轴落在软件软限位内（复核不通过就不该返回 success=True）。"""
    target = fk_pose(single_arm_scene, REFERENCE_Q)
    result = ik_mod.solve_ik(single_arm_scene, target)
    assert result.success, result.message

    names, lower, upper = ik_mod.software_joint_limits()
    q = result.q_array
    for index, name in enumerate(names):
        assert lower[index] - 1e-9 <= q[index] <= upper[index] + 1e-9, (
            f"{name}={q[index]} 越界 [{lower[index]}, {upper[index]}]"
        )


def test_mdh_independent_fk_cross_check(single_arm_scene):
    """正例：用 MDH 独立正解（S01 零 Drake 依赖基准）交叉校验末端位置。"""
    target = fk_pose(single_arm_scene, REFERENCE_Q)
    result = ik_mod.solve_ik(single_arm_scene, target)
    assert result.success, result.message

    mdh_position = mdh.link7_position(result.q)
    error = float(np.linalg.norm(mdh_position - REFERENCE_POSITION))
    assert error < MDH_POSITION_TOLERANCE, (
        f"MDH 交叉校验失败：误差 {error:.3e} m > {MDH_POSITION_TOLERANCE} m"
    )


def test_batch_solve_ik_returns_one_result_per_target(single_arm_scene):
    """正例：批量接口返回与目标等长、顺序一致的结果列表。"""
    reachable = fk_pose(single_arm_scene, REFERENCE_Q)
    unreachable = ik_mod.target_pose_from_position([3.0, 0.0, 0.55])
    results = ik_mod.batch_solve_ik(single_arm_scene, [reachable, unreachable])

    assert len(results) == 2
    assert all(isinstance(item, ik_mod.IkResult) for item in results)
    assert results[0].success
    assert not results[1].success


# ── 负例：可解释失败 ────────────────────────────────────────────────────
def test_far_target_fails_with_explanatory_message(single_arm_scene):
    """负例：目标离工作空间极远（> 2 m）→ 失败且 message 非空。"""
    far = ik_mod.target_pose_from_position([3.0, 0.0, 0.55])
    result = ik_mod.solve_ik(single_arm_scene, far)

    assert not result.success
    assert result.message
    assert result.reason == ik_mod.REASON_UNREACHABLE
    assert result.position_error > 0.1


def test_nan_target_raises_explicit_error(single_arm_scene):
    """负例：目标位姿含 NaN → 抛明确异常（不是静默失败）。"""
    bad = _NonFinitePose([float("nan"), 0.0, 0.55])
    with pytest.raises(ik_mod.IkError) as excinfo:
        ik_mod.solve_ik(single_arm_scene, bad)
    assert excinfo.value.reason == ik_mod.REASON_INVALID_TARGET


def test_infinite_target_raises_explicit_error(single_arm_scene):
    """负例：目标位姿含 Inf → 抛明确异常。"""
    bad = _NonFinitePose([float("inf"), 0.0, 0.55])
    with pytest.raises(ik_mod.IkError) as excinfo:
        ik_mod.solve_ik(single_arm_scene, bad)
    assert excinfo.value.reason == ik_mod.REASON_INVALID_TARGET


def test_non_pose_target_raises_explicit_error(single_arm_scene):
    """负例：目标不是位姿对象 → 抛明确异常。"""
    with pytest.raises(ik_mod.IkError) as excinfo:
        ik_mod.solve_ik(single_arm_scene, (0.35, 0.0, 0.55))
    assert excinfo.value.reason == ik_mod.REASON_INVALID_TARGET


def test_seed_outside_soft_limits_fails_with_joint_name(single_arm_scene):
    """负例：起点软限位越界 → 失败，且 message 点名越界关节与数值。"""
    target = fk_pose(single_arm_scene, REFERENCE_Q)
    seed = list(REFERENCE_Q)
    seed[1] = 3.0  # joint_2 软件软限位是 [-2.1990868, 2.1990868]

    result = ik_mod.solve_ik(single_arm_scene, target, initial_posture=seed)
    assert not result.success
    assert result.reason == ik_mod.REASON_SEED_OUT_OF_LIMITS
    assert result.message
    assert "joint_2" in result.message
    assert result.iterations == 0


def test_timeout_budget_is_reported(single_arm_scene):
    """负例：时间预算耗尽 → reason=timeout（而不是伪装成"不可达"）。"""
    target = fk_pose(single_arm_scene, REFERENCE_Q)
    result = ik_mod.solve_ik(single_arm_scene, target, timeout=1e-9)

    assert not result.success
    assert result.reason == ik_mod.REASON_TIMEOUT
    assert result.message


def test_zero_nominal_posture_is_rejected(single_arm_scene):
    """负例：名义姿态全零必须被显式拒绝（零位 6 轴退化）。"""
    target = fk_pose(single_arm_scene, REFERENCE_Q)
    with pytest.raises(ik_mod.IkError):
        ik_mod.solve_ik(single_arm_scene, target, nominal_posture=[0.0] * 7)


@pytest.mark.parametrize(
    "kwargs",
    [
        {"position_tol": 0.0},
        {"orientation_tol": -1.0},
        {"timeout": 0.0},
        {"max_iterations": 0},
    ],
)
def test_invalid_budget_parameters_raise(single_arm_scene, kwargs):
    """负例：非法预算参数 → 抛异常（不静默按默认值跑）。"""
    target = fk_pose(single_arm_scene, REFERENCE_Q)
    with pytest.raises(ik_mod.IkError):
        ik_mod.solve_ik(single_arm_scene, target, **kwargs)


def test_failure_message_is_never_empty(single_arm_scene):
    """负例：所有失败路径的 message 都必须非空（可解释失败的基本要求）。"""
    far = ik_mod.target_pose_from_position([3.0, 0.0, 0.55])
    result = ik_mod.solve_ik(single_arm_scene, far)
    assert result.message.strip()
    assert result.reason in ik_mod.FAILURE_REASONS
