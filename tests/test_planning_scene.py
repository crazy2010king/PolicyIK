"""场景装配冒烟测试（S02-T01）。

覆盖：
* 场景能装配出来（DoF = 7、joint_names 顺序与 scenario 一致、障碍物几何数 = 1）；
* 碰撞检查器可构造，且零位配置下的碰撞判定**调用成功**；
* **障碍物真的参与碰撞判定**（把箱子挪到末端正下方 → 判定为碰撞）：
  这条断言专门钉死 "没有 proximity 角色 → 碰撞检查看不见障碍物" 这个坑；
* 关节顺序与 scenario 不一致 → 明确报错；
* ``drake_compat`` 对 ``CollisionCheckerParams.model`` 的类型约束（plant /
  ModelInstanceIndex 都必须给出可读错误，而不是 pybind 的 opaque cast 错误）。
"""

from __future__ import annotations

import dataclasses
from pathlib import Path

import pytest

from rm75_drake import drake_compat
from rm75_drake.planning import scene as scene_mod
from rm75_drake.planning.scene import SceneError, build_scene, build_scene_from_path
from rm75_drake.scenario import load_scenario

REPO_ROOT = Path(__file__).resolve().parents[1]
SCENARIO_PATH = REPO_ROOT / "scenarios" / "single_arm.yaml"


@pytest.fixture(scope="module")
def scenario():
    return load_scenario(SCENARIO_PATH)


@pytest.fixture(scope="module")
def bundle(scenario):
    return build_scene(scenario)


# ── 装配冒烟 ────────────────────────────────────────────────────────────
def test_scene_builds(bundle):
    assert bundle.plant is not None
    assert bundle.scene_graph is not None
    assert bundle.robot_diagram is not None
    assert bundle.plant_context is not None
    assert bundle.checker is not None
    assert bundle.source_id is not None


def test_dof_is_seven(bundle):
    """固定基座后 DoF = 7。"""
    assert bundle.plant.num_positions() == 7
    assert bundle.plant.num_velocities() == 7


def test_joint_names_order_matches_scenario(bundle, scenario):
    assert bundle.joint_names == tuple(f"joint_{i}" for i in range(1, 8))
    assert bundle.joint_names == scenario.robot("rm75").joint_names


def test_model_instance_is_robot(bundle):
    assert bundle.model_instance is not None
    assert bundle.model_instances == (bundle.model_instance,)
    assert bundle.robot_name == "rm75"


def test_obstacle_geometry_count_is_one(bundle, scenario):
    assert len(scenario.obstacles) == 1
    assert len(bundle.obstacle_geometry_ids) == 1

    inspector = bundle.scene_graph.model_inspector()
    obstacle_geometries = [
        gid
        for gid in inspector.GetAllGeometryIds()
        if inspector.GetOwningSourceName(gid) == scene_mod.OBSTACLE_SOURCE_NAME
    ]
    assert len(obstacle_geometries) == 1
    assert inspector.GetName(obstacle_geometries[0]) == "workbench_box"


def test_checker_reports_collision_free_at_zero_configuration(bundle):
    """零位配置 + 箱子在工作台位置 → 无碰撞，且判定调用本身成功。"""
    q = bundle.plant.GetPositions(bundle.plant_context)
    assert q.shape == (7,)
    assert bundle.checker.CheckConfigCollisionFree(q) is True


def test_obstacle_participates_in_collision_checking(scenario):
    """把箱子挪到零位末端正下方 → 必须判定为碰撞。

    这条断言钉死 "障碍物没有 proximity 角色 → 碰撞检查看不见它" 的坑
    （Drake 1.51/1.55 的 RegisterAnchoredGeometry 不会自动分配 proximity 角色）。
    """
    base = scenario.obstacles[0]
    moved = dataclasses.replace(
        base,
        name="tip_box",
        xyz=(0.0, 0.0, 0.85),
        rpy=(0.0, 0.0, 0.0),
    )
    tipped = dataclasses.replace(scenario, obstacles=(moved,))

    bundle = build_scene(tipped)
    q = bundle.plant.GetPositions(bundle.plant_context)
    assert bundle.checker.CheckConfigCollisionFree(q) is False


def test_build_scene_from_path_matches_scenario():
    bundle = build_scene_from_path(SCENARIO_PATH)
    assert bundle.plant.num_positions() == 7
    assert bundle.joint_names == tuple(f"joint_{i}" for i in range(1, 8))


# ── 负例 ────────────────────────────────────────────────────────────────
def test_joint_order_mismatch_raises_scene_error(scenario):
    """scenario 里的关节顺序与 URDF 不符 → 明确报错（不静默接受）。"""
    robot = scenario.robots[0]
    bad_joints = ("joint_2", "joint_1") + tuple(robot.joint_names[2:])
    bad_robot = dataclasses.replace(robot, joint_names=bad_joints)
    broken = dataclasses.replace(scenario, robots=(bad_robot,))

    with pytest.raises(SceneError) as excinfo:
        build_scene(broken)
    assert "关节顺序" in str(excinfo.value)


def test_missing_base_link_raises_scene_error(scenario):
    robot = scenario.robots[0]
    bad_robot = dataclasses.replace(robot, base_link="no_such_link")
    broken = dataclasses.replace(scenario, robots=(bad_robot,))

    with pytest.raises(SceneError) as excinfo:
        build_scene(broken)
    assert "no_such_link" in str(excinfo.value)


def test_missing_extrinsic_raises_scene_error(scenario):
    broken = dataclasses.replace(scenario, base_extrinsics={})
    with pytest.raises(SceneError) as excinfo:
        build_scene(broken)
    assert "外参" in str(excinfo.value)


# ── drake_compat：CollisionCheckerParams.model 的类型约束 ───────────────
def test_collision_params_reject_plant(bundle):
    with pytest.raises(TypeError) as excinfo:
        drake_compat.make_collision_checker_params(
            robot_model_instances=[bundle.model_instance], model=bundle.plant
        )
    assert "RobotDiagram" in str(excinfo.value)


def test_collision_params_reject_model_instance(bundle):
    with pytest.raises(TypeError) as excinfo:
        drake_compat.make_collision_checker_params(model=bundle.model_instance)
    assert "robot_model_instances" in str(excinfo.value)


def test_collision_checker_factory_requires_diagram():
    with pytest.raises(TypeError) as excinfo:
        drake_compat.make_collision_checker()
    assert "RobotDiagram" in str(excinfo.value)


def test_collision_checker_factory_builds_checker(bundle):
    checker = drake_compat.make_collision_checker(
        robot_diagram=bundle.robot_diagram,
        robot_model_instances=[bundle.model_instance],
        edge_step_size=0.01,
    )
    q = bundle.plant.GetPositions(bundle.plant_context)
    assert checker.CheckConfigCollisionFree(q) is True


def test_non_positive_edge_step_size_rejected(bundle):
    """edge_step_size 必须 > 0：提前抛可读 ValueError，而不是 Drake CHECK。"""
    with pytest.raises(ValueError) as excinfo:
        drake_compat.make_collision_checker(
            robot_diagram=bundle.robot_diagram,
            robot_model_instances=[bundle.model_instance],
            edge_step_size=0.0,
        )
    assert "edge_step_size" in str(excinfo.value)


def test_non_numeric_edge_step_size_rejected(bundle):
    with pytest.raises(ValueError) as excinfo:
        drake_compat.make_collision_checker(
            robot_diagram=bundle.robot_diagram,
            robot_model_instances=[bundle.model_instance],
            edge_step_size="0.01",
        )
    assert "edge_step_size" in str(excinfo.value)
