"""双臂单一 plant 装配器的可执行断言（S04-T02）。

覆盖：
* 双臂（不含夹爪）→ ``plant.num_positions() == 14``；含两夹爪 → ``== 18``；
* 位置变量名顺序（Drake 位置名 = model instance 名 + 关节名）；
* 零位位形（左 ``link_7`` 高度 0.8505 m，右 ``link_7`` 世界 x = 0.6 m）；
* **臂间碰撞**可检出（左 j2=+0.5 / 右 j2=−0.5 → 有碰）；
* **同臂自碰撞**仍可检出（左 j2=+1.5 → 有碰）；
* **相邻体过滤**仍生效（全零位形 → 无碰）；
* 负例：q 维度不符 → :class:`SceneError`（点名期望/实际，且**不 abort**）；
  scenario 缺右臂 ``base_extrinsics`` → :class:`SceneError`（点名机器人）。
"""

from __future__ import annotations

import dataclasses
from pathlib import Path

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
DUAL_SCENARIO_PATH = REPO_ROOT / "scenarios" / "dual_arm.yaml"

#: 左臂 j2 的下标（q 顺序 = 左 7 + 右 7）。
LEFT_J2 = 1
#: 右臂 j2 的下标。
RIGHT_J2 = 7 + 1


@pytest.fixture(scope="module")
def dual_scenario():
    from rm75_drake.scenario import load_scenario

    return load_scenario(DUAL_SCENARIO_PATH)


@pytest.fixture(scope="module")
def dual_scene(dual_scenario):
    """双臂（**不含夹爪**）的单 plant 场景。"""
    from rm75_drake.bimanual.dual_scene import build_dual_scene

    return build_dual_scene(dual_scenario)


@pytest.fixture(scope="module")
def dual_scene_with_grippers(dual_scenario):
    """双臂 + 双夹爪的单 plant 场景（夹爪 weld 到各自法兰）。"""
    from rm75_drake.bimanual.dual_scene import build_dual_scene, default_gripper_urdfs

    return build_dual_scene(
        dual_scenario,
        gripper_urdfs=default_gripper_urdfs(("left", "right")),
    )


# ── 场景真相源 ──────────────────────────────────────────────────────────
def test_dual_scenario_loads_and_declares_two_arms(dual_scenario):
    from rm75_drake.scenario import SCHEMA_VERSION

    assert dual_scenario.schema_version == SCHEMA_VERSION
    assert dual_scenario.robot_names == ("left", "right")
    assert dual_scenario.robot("left").base_link == "left_base_link"
    assert dual_scenario.robot("right").base_link == "right_base_link"
    assert dual_scenario.robot("left").joint_names[0] == "left_joint_1"
    assert dual_scenario.robot("right").joint_names[-1] == "right_joint_7"
    assert dual_scenario.obstacles == ()
    # S04-T05：场景改为**引用**标定文件（标定文件是基座外参的唯一真相源），
    # 因此 calibration 段不再为空。
    assert list(dual_scenario.calibration) == ["base_extrinsic"]
    assert dual_scenario.calibration["base_extrinsic"]["file"].endswith("dual_arm.yaml")


# ── 单一 plant 装配 ─────────────────────────────────────────────────────
def test_two_arms_share_one_plant_with_14_positions(dual_scene):
    assert len(dual_scene.model_instances) == 2
    assert dual_scene.num_positions == 14
    assert dual_scene.joint_names == tuple(
        [f"left_joint_{i}" for i in range(1, 8)] + [f"right_joint_{i}" for i in range(1, 8)]
    )


def test_position_variable_names_order(dual_scene):
    names = list(dual_scene.plant.GetPositionNames())
    assert len(names) == 14
    assert names[0] == "RM75-B-left_left_joint_1_q"
    assert names[6] == "RM75-B-left_left_joint_7_q"
    assert names[7] == "RM75-B-right_right_joint_1_q"
    assert names[-1] == "RM75-B-right_right_joint_7_q"


def test_grippers_add_four_prismatic_positions(dual_scene_with_grippers):
    bundle = dual_scene_with_grippers
    assert bundle.num_positions == 18
    assert len(bundle.gripper_model_instances) == 2
    assert bundle.gripper_joint_names == (
        "left_finger_joint_left",
        "left_finger_joint_right",
        "right_finger_joint_left",
        "right_finger_joint_right",
    )
    # 夹爪被 weld 到各自法兰 → 不是浮动基座（浮动基座会额外贡献 7 个位置变量）。
    assert bundle.tip_links == {"left": "left_link_7", "right": "right_link_7"}
    assert bundle.tcp_frames == {"left": "left_tcp", "right": "right_tcp"}


# ── 零位位形 ────────────────────────────────────────────────────────────
def test_zero_configuration_link7_poses(dual_scene):
    from rm75_drake import drake_compat

    left = drake_compat.eval_body_pose_in_world(
        dual_scene.context, dual_scene.plant.GetBodyByName("left_link_7"), dual_scene.plant
    )
    right = drake_compat.eval_body_pose_in_world(
        dual_scene.context, dual_scene.plant.GetBodyByName("right_link_7"), dual_scene.plant
    )
    assert left.translation()[2] == pytest.approx(0.8505, abs=1e-6)
    assert right.translation()[2] == pytest.approx(0.8505, abs=1e-6)
    assert right.translation()[0] == pytest.approx(0.6, abs=1e-6)


# ── 碰撞判据 ────────────────────────────────────────────────────────────
def test_inter_arm_collision_is_detected(dual_scene):
    from rm75_drake.bimanual.dual_scene import check_config_free

    q = np.zeros(14)
    q[LEFT_J2] = 0.5
    q[RIGHT_J2] = -0.5
    assert check_config_free(dual_scene, q) is False


def test_same_arm_self_collision_is_detected(dual_scene):
    from rm75_drake.bimanual.dual_scene import check_config_free

    q = np.zeros(14)
    q[LEFT_J2] = 1.5
    assert check_config_free(dual_scene, q) is False


def test_adjacent_body_filter_still_active_at_zero(dual_scene):
    """全零位形 → 无碰（证明改名未破坏 Drake 默认的父子相邻体过滤）。"""
    from rm75_drake.bimanual.dual_scene import check_config_free

    assert check_config_free(dual_scene, np.zeros(14)) is True


# ── 负例 ────────────────────────────────────────────────────────────────
def test_wrong_q_length_raises_scene_error_instead_of_abort(dual_scene):
    from rm75_drake.bimanual.dual_scene import SceneError, check_config_free

    with pytest.raises(SceneError) as excinfo:
        check_config_free(dual_scene, np.zeros(13))
    message = str(excinfo.value)
    assert "14" in message  # 期望值
    assert "13" in message  # 实际值

    # 更长/更短的 q 同样点名（而不是 abort）。
    with pytest.raises(SceneError) as excinfo:
        check_config_free(dual_scene, np.zeros(99))
    assert "99" in str(excinfo.value)


def test_missing_base_extrinsic_names_the_robot(dual_scenario):
    from rm75_drake.bimanual.dual_scene import SceneError, build_dual_scene

    broken = dataclasses.replace(
        dual_scenario, base_extrinsics={"left": dual_scenario.base_extrinsics["left"]}
    )
    with pytest.raises(SceneError) as excinfo:
        build_dual_scene(broken)
    assert "right" in str(excinfo.value)


def test_single_robot_scenario_is_rejected(dual_scenario):
    """双臂装配器不接受单机器人场景（显式报错，不静默跑成单臂）。"""
    from rm75_drake.bimanual.dual_scene import SceneError, build_dual_scene

    only_left = dataclasses.replace(dual_scenario, robots=dual_scenario.robots[:1])
    with pytest.raises(SceneError) as excinfo:
        build_dual_scene(only_left)
    assert "left" in str(excinfo.value)


def test_joint_name_mismatch_is_rejected(dual_scenario):
    """plant 解析出的关节序列与 scenario 声明不一致 → 显式报错（点名机器人）。"""
    from rm75_drake.bimanual.dual_scene import SceneError, build_dual_scene

    left = dual_scenario.robot("left")
    broken_left = dataclasses.replace(left, joint_names=("left_joint_1",))
    broken = dataclasses.replace(
        dual_scenario,
        robots=(broken_left,) + dual_scenario.robots[1:],
    )
    with pytest.raises(SceneError) as excinfo:
        build_dual_scene(broken)
    assert "left" in str(excinfo.value)


def test_missing_gripper_url_names_the_robot(dual_scenario):
    from rm75_drake.bimanual.dual_scene import SceneError, build_dual_scene

    with pytest.raises(SceneError) as excinfo:
        build_dual_scene(
            dual_scenario,
            gripper_urdfs={"left": "package://rm75_drake/rmg24_left.urdf"},
        )
    assert "right" in str(excinfo.value)
