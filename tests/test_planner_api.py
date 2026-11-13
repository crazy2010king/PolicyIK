"""Planner API 测试（S02-T07）：同源规划入口 + 可解释失败。

被测对象
--------
:mod:`rm75_drake.planner` —— :class:`~rm75_drake.planner.Planner`
（任务空间目标 / 关节空间目标 → 5 ms 网格轨迹 + 夹爪调度表 + 诊断）。

覆盖的契约
----------
**正例**
* 关节目标 → ``success=True``，轨迹是 **5 ms 网格**、**seq 从 0 逐点 +1**、
  **绝对时刻严格递增**，逐点落在**软件软限位**内；
* 轨迹**到位**（末点 ≈ 目标位形）；
* 任务空间目标（RigidTransform）→ 同样成功，且末点 FK ≈ 目标位姿；
* ``diagnostics`` 含**各阶段耗时**、**区域数**、**边数**、**求解模式**、
  **碰撞复检逃逸数 = 0**、**是否命中区域缓存**；
* 夹爪调度表随结果一起返回；
* **同源保证**：``rm75_drake.planner`` **不依赖驱动层**（不 import drivers）。

**负例（可解释失败）**
* 关节目标越出软件软限位 → ``success=False`` + ``stage="ik"`` +
  ``reason="soft_limit_violation"``；
* 任务空间目标不可达 → ``success=False`` + ``stage="ik"`` +
  ``reason`` ∈ {``unreachable``, ``timeout``}；
* 目标维度 / 数值非法 → ``stage="ik"`` + ``reason="invalid_target"``；
* 非法选项（区域间距 ≤ 0、未知 IRIS 旋钮）→ 构造期即抛
  :class:`~rm75_drake.planner.PlannerError`。

耗时说明
--------
链路里有 IRIS（本机单区域 ~8 s），因此规划用例打 ``@pytest.mark.slow``；
规划产物用 **session 级** fixture（``planner_bundle``）共享，
``test_fake_driver.py`` 也复用同一条轨迹。
"""

from __future__ import annotations

import inspect
import logging
import re

import numpy as np
import pytest

from rm75_drake import planner as planner_mod
from rm75_drake.gripper import open_action
from rm75_drake.planner import PlanOptions, Planner, PlannerError
from rm75_drake.planning.ik import REASON_INVALID_TARGET, REASON_SOFT_LIMIT_VIOLATION
from rm75_drake.planning.trajectory import DEFAULT_DT, software_limits

logging.getLogger("drake").setLevel(logging.WARNING)

pytestmark = pytest.mark.slow

#: 起点位形：抬肘名义姿态（碰撞自由）。
PLANNER_START_Q = (0.0, -0.6, 0.0, 0.9, 0.0, 0.5, 0.0)
#: 目标位形：沿 joint_1 偏移 0.3 rad（直线段碰撞自由）。
PLANNER_TARGET_Q = (0.3, -0.6, 0.0, 0.9, 0.0, 0.5, 0.0)


# ── 工具 ────────────────────────────────────────────────────────────────
def _fk_pose(scene, q):
    """返回给定关节角下末端 frame 的世界位姿（``RigidTransform``）。"""
    from pydrake.math import RigidTransform

    context = scene.plant.CreateDefaultContext()
    scene.plant.SetPositions(context, np.asarray(q, dtype=float))
    pose = scene.plant.GetFrameByName("link_7").CalcPoseInWorld(context)
    return RigidTransform(pose.rotation(), pose.translation())


def _rotation_angle(r1, r2) -> float:
    """两个旋转矩阵之间的夹角（rad）（Drake 1.51 的 Quaternion 无 angle_between）。"""
    relative = np.asarray(r1.matrix(), dtype=float) @ np.asarray(
        r2.matrix(), dtype=float
    ).T
    trace = float(np.trace(relative))
    return float(np.arccos(np.clip((trace - 1.0) / 2.0, -1.0, 1.0)))


def _assert_grid_invariants(trajectory, dt: float = DEFAULT_DT) -> None:
    """断言 5 ms 网格 + seq 连续 + 时刻严格递增 + 逐点软限位内。"""
    samples = trajectory.samples
    assert len(samples) >= 2
    assert [s.seq for s in samples] == list(range(len(samples)))
    times = np.array([s.t_abs for s in samples], dtype=float)
    steps = np.diff(times)
    assert np.all(steps > 0.0)
    assert np.max(np.abs(steps - dt)) <= 1e-9
    limits = software_limits()
    matrix = trajectory.q_matrix()
    assert np.all(matrix >= np.asarray(limits.lower) - 1e-9)
    assert np.all(matrix <= np.asarray(limits.upper) + 1e-9)


# ── 正例 ────────────────────────────────────────────────────────────────
def test_plan_joint_target_succeeds(planner_bundle):
    """正例：可达的关节目标 → 成功的 5 ms 网格轨迹。"""
    result = planner_bundle["result"]
    assert result.success, f"规划失败：{result.message}"
    assert result.stage == "" and result.reason == ""
    assert result.message
    trajectory = result.trajectory
    assert trajectory is not None
    assert trajectory.dt == DEFAULT_DT
    _assert_grid_invariants(trajectory)


def test_plan_joint_target_reaches_target(planner_bundle):
    """正例：轨迹到位（末点 ≈ 目标位形）。"""
    trajectory = planner_bundle["result"].trajectory
    assert trajectory is not None
    assert np.allclose(trajectory.samples[-1].q, PLANNER_TARGET_Q, atol=1e-9)
    assert np.allclose(trajectory.samples[0].q, PLANNER_START_Q, atol=1e-9)


def test_plan_diagnostics_expose_stage_evidence(planner_bundle):
    """正例：diagnostics 含各阶段耗时 / 区域数 / 边数 / 求解模式 / 逃逸数 / 缓存命中。"""
    diagnostics = planner_bundle["result"].diagnostics
    for stage in ("ik", "regions", "gcs", "retime", "collision"):
        assert stage in diagnostics, f"diagnostics 缺少阶段 {stage!r}"
        assert diagnostics[stage]["duration_s"] >= 0.0

    regions = diagnostics["regions"]
    assert regions["num_regions"] >= 2
    assert regions["iris_runs"] == regions["num_regions"]  # 首次规划全是新算
    assert regions["cache_hit"] is False

    gcs = diagnostics["gcs"]
    assert gcs["success"] is True
    assert gcs["num_regions"] == regions["num_regions"]
    assert gcs["num_edges"] >= 1
    assert gcs["solver_mode"] in ("mixed_integer", "convex_relaxation")

    collision = diagnostics["collision"]
    assert collision["num_escapes"] == 0
    assert collision["success"] is True

    retime = diagnostics["retime"]
    assert retime["num_samples"] == len(planner_bundle["result"].trajectory)
    assert retime["trajectory_duration_s"] > 0.0


def test_plan_task_space_target_succeeds(planner_bundle):
    """正例：任务空间目标（RigidTransform）→ 成功，且末点 FK ≈ 目标位姿。"""
    scene = planner_bundle["scene"]
    planner = planner_bundle["planner"]
    target_pose = _fk_pose(scene, PLANNER_TARGET_Q)

    # IK 的种子直接给到解（避免再引入一条不同的 IK 分支），
    # 于是区域链与关节目标那次完全一致 → 区域缓存命中（也顺带验证缓存）。
    options = PlanOptions(
        start_configuration=PLANNER_START_Q,
        ik_initial_posture=PLANNER_TARGET_Q,
        region_options=planner_bundle["options"].region_options,
    )
    result = planner.plan_task_space_target(target_pose, options=options)
    assert result.success, f"规划失败：{result.message}"
    assert result.diagnostics["ik"]["success"] is True
    assert result.diagnostics["regions"]["cache_hit"] is True

    trajectory = result.trajectory
    assert trajectory is not None
    _assert_grid_invariants(trajectory)
    assert result.diagnostics["collision"]["num_escapes"] == 0

    # 末点 FK 必须落在目标位姿的容差内。
    actual = _fk_pose(scene, trajectory.samples[-1].q)
    assert np.linalg.norm(
        actual.translation() - target_pose.translation()
    ) <= 1e-3
    assert (
        _rotation_angle(actual.rotation(), target_pose.rotation()) <= 1e-2
    )


def test_plan_result_carries_gripper_schedule(planner_bundle):
    """正例：夹爪调度表随结果返回，并写进轨迹。"""
    planner = planner_bundle["planner"]
    action = open_action()
    options = PlanOptions(
        start_configuration=PLANNER_START_Q,
        region_options=planner_bundle["options"].region_options,
        gripper_schedule=(action,),
    )
    result = planner.plan_joint_target(PLANNER_TARGET_Q, options=options)
    assert result.success, f"规划失败：{result.message}"
    assert tuple(result.gripper_schedule) == (action,)
    assert tuple(result.trajectory.gripper_schedule) == (action,)


def test_planner_does_not_depend_on_driver_layer():
    """同源保证：Planner **只产出数据**，不 import 驱动层。"""
    source = inspect.getsource(planner_mod)
    offending = re.findall(
        r"^\s*(?:from|import)\s+[^\n]*drivers[^\n]*$", source, flags=re.MULTILINE
    )
    assert not offending, f"Planner 不得 import 驱动层（命中：{offending}）"


# ── 负例（可解释失败）───────────────────────────────────────────────────
def test_unreachable_joint_target_reports_soft_limit_violation(planner_bundle):
    """负例：关节目标越出软件软限位 → 可解释失败。"""
    planner = planner_bundle["planner"]
    bad_target = list(PLANNER_TARGET_Q)
    bad_target[5] = 3.0  # joint_6 的软件上限是 2.164 rad
    result = planner.plan_joint_target(bad_target)
    assert result.success is False
    assert result.trajectory is None
    assert result.stage == "ik"
    assert result.reason == REASON_SOFT_LIMIT_VIOLATION
    assert result.message
    assert "ik" in result.message


def test_invalid_joint_target_shape_is_rejected(planner_bundle):
    """负例：目标维度不对 / 含 NaN → 可解释失败。"""
    planner = planner_bundle["planner"]
    for bad in ([0.0, 0.1], [0.0] * 6 + [float("nan")]):
        result = planner.plan_joint_target(bad)
        assert result.success is False
        assert result.stage == "ik"
        assert result.reason == REASON_INVALID_TARGET
        assert result.message


def test_unreachable_task_space_target_reports_ik_failure(planner_bundle):
    """负例：任务空间目标不可达（5 m 外）→ IK 阶段可解释失败。"""
    from pydrake.math import RigidTransform
    from pydrake.math import RotationMatrix

    planner = planner_bundle["planner"]
    target = RigidTransform(RotationMatrix.Identity(), np.array([5.0, 0.0, 0.0]))
    result = planner.plan_task_space_target(target)
    assert result.success is False
    assert result.stage == "ik"
    assert result.reason in ("unreachable", "timeout")
    assert result.message
    assert result.diagnostics["ik"]["success"] is False


def test_invalid_options_are_rejected_at_construction():
    """负例：非法选项在构造期即报错（不静默兜底）。"""
    with pytest.raises(PlannerError):
        PlanOptions(region_spacing=0.0)
    with pytest.raises(PlannerError):
        PlanOptions(region_options={"not_a_real_knob": 1})
    with pytest.raises(PlannerError):
        PlanOptions(adjacency_factor=0.5)


def test_planner_requires_scenario_or_scene():
    """负例：既没有 scenario 也没有 scene → 显式报错。"""
    with pytest.raises(PlannerError):
        Planner()
