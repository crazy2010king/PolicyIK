"""双臂碰撞过滤矩阵的可执行断言（S04-T03）。

覆盖（四条规则各有正例 + 反例）
------------------------------
* ``adjacent_same_arm``：零位形**无碰**（正例）；装配期关掉相邻体过滤后
  **有碰**（反例：证明这条规则的开关语义）；
* ``left_right_mutual``：施加全部规则后左右臂靠近**仍然有碰**（反例：过滤矩阵
  没有把臂间互检关掉）；反向地，若显式 ``ExcludeBetween(left, right)``，
  碰撞就被藏起来了（证明这条规则确实在起作用）；
* ``gripper_finger_vs_grasped_object``：夹爪指夹住物体时**有碰**（未施加规则），
  施加「放行」规则后**无碰**（正例：过滤真的改变了碰撞判定结果）；
* ``gripper_finger_vs_own_palm``：该对在场景图里被过滤（正例）；把 plant 的
  相邻体过滤关掉后、施加本规则前该对**未被**过滤（反例：证明本规则做了事）；
* 规则标签集合恰为四个稳定标签（CI / 报告按标签断言）；
* 负例：非法 ``action`` / ``scope`` / ``mechanism`` / 未知角色 / 不存在的
  model instance → 显式抛 :class:`FilterRuleError` 并点名（**不静默跳过**）。
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
#: 左臂 j4 的下标（q 顺序 = 左 7 + 右 7）：j4 = 3.0 rad 会造成**同臂非相邻**
#: 连杆互相穿透（纯自碰）。
LEFT_J4 = 3

#: 夹爪指张开量（m）：0 = 闭合（两指相碰），张开后两指分离。
FINGER_OPEN = 0.02


# ── 夹具 ────────────────────────────────────────────────────────────────
@pytest.fixture(scope="module")
def dual_scenario():
    from rm75_drake.scenario import load_scenario

    return load_scenario(DUAL_SCENARIO_PATH)


def _build(scenario, **kwargs):
    from rm75_drake.bimanual.dual_scene import build_dual_scene

    return build_dual_scene(scenario, **kwargs)


@pytest.fixture
def dual_scene(dual_scenario):
    """双臂（不含夹爪），相邻体过滤**开启**（Drake 默认）。"""
    return _build(dual_scenario)


@pytest.fixture
def dual_scene_without_adjacent_filter(dual_scenario):
    """双臂（不含夹爪），相邻体过滤**关闭**（装配期设置）。"""
    return _build(dual_scenario, adjacent_bodies_collision_filters=False)


@pytest.fixture
def gripper_scene(dual_scenario):
    """双臂 + 双夹爪（夹爪进入碰撞判定）。"""
    from rm75_drake.bimanual.dual_scene import default_gripper_urdfs

    return _build(
        dual_scenario,
        gripper_urdfs=default_gripper_urdfs(("left", "right")),
        include_grippers_in_collision=True,
    )


@pytest.fixture
def gripper_scene_without_adjacent_filter(dual_scenario):
    """双臂 + 双夹爪，相邻体过滤**关闭**（装配期设置）。"""
    from rm75_drake.bimanual.dual_scene import default_gripper_urdfs

    return _build(
        dual_scenario,
        gripper_urdfs=default_gripper_urdfs(("left", "right")),
        include_grippers_in_collision=True,
        adjacent_bodies_collision_filters=False,
    )


# ── 辅助 ────────────────────────────────────────────────────────────────
def _open_finger_q(bundle) -> np.ndarray:
    """双臂 + 夹爪的位形：机械臂零位、夹爪指张开（避免两指相碰）。"""
    q = np.zeros(bundle.num_positions)
    for index, name in enumerate(bundle.plant.GetPositionNames()):
        if "finger" in name:
            q[index] = FINGER_OPEN
    return q


def _geometry_ids(bundle, model_instance, link_pattern: str = "*"):
    """某个 model instance 里匹配 ``link_pattern`` 的几何 id 列表。"""
    plant = bundle.plant
    inspector = bundle.scene_graph.model_inspector()
    out = []
    for body_index in plant.GetBodyIndices(model_instance):
        name = plant.get_body(body_index).name()
        if not _match(name, link_pattern):
            continue
        frame_id = plant.GetBodyFrameIdOrThrow(body_index)
        out.extend(list(inspector.GetGeometries(frame_id)))
    return out


def _match(name: str, pattern: str) -> bool:
    import fnmatch

    return fnmatch.fnmatch(name, pattern)


def _grasped_object_center(bundle, link_name: str = "left_finger_left"):
    """某个连杆上**碰撞几何**中心在世界系下的位置（用于摆放被抓物体）。"""
    from rm75_drake import drake_compat

    plant = bundle.plant
    inspector = bundle.scene_graph.model_inspector()
    body_index = next(
        index
        for index in plant.GetBodyIndices(bundle.gripper_model_instances[0])
        if plant.get_body(index).name() == link_name
    )
    frame_id = plant.GetBodyFrameIdOrThrow(body_index)
    geometry_id = next(iter(inspector.GetGeometries(frame_id)))
    offset = inspector.GetPoseInFrame(geometry_id)
    body_pose = drake_compat.eval_body_pose_in_world(
        bundle.context, plant.get_body(body_index), plant
    )
    point = np.asarray(body_pose.translation()) + np.asarray(
        body_pose.rotation().matrix()
    ) @ np.asarray(offset.translation())
    return tuple(float(value) for value in point)


@pytest.fixture
def grasped_scene(dual_scenario):
    """双臂 + 双夹爪 + **被抓物体**（锚定 box，正好压住张开的左夹爪指）。"""
    from rm75_drake.bimanual.dual_scene import default_gripper_urdfs
    from rm75_drake.scenario import Obstacle

    probe = _build(
        dual_scenario,
        gripper_urdfs=default_gripper_urdfs(("left", "right")),
        include_grippers_in_collision=True,
    )
    probe.plant.SetPositions(probe.context, _open_finger_q(probe))
    center = _grasped_object_center(probe)
    obstacle = Obstacle(
        name="grasped_object",
        type="box",
        size=(0.008, 0.008, 0.008),
        xyz=center,
        rpy=(0.0, 0.0, 0.0),
    )
    scenario = dataclasses.replace(dual_scenario, obstacles=(obstacle,))
    return _build(
        scenario,
        gripper_urdfs=default_gripper_urdfs(("left", "right")),
        include_grippers_in_collision=True,
    )


def _apply(bundle, **kwargs):
    """对双臂场景施加默认过滤矩阵（含检查器 → 碰撞判定真的变）。"""
    from rm75_drake.bimanual.collision_filter import apply_dual_arm_filters

    options = {
        "left_instance": bundle.model_instances[0],
        "right_instance": bundle.model_instances[1],
        "gripper_instances": bundle.gripper_model_instances,
        "checker": bundle.checker,
    }
    options.update(kwargs)
    return apply_dual_arm_filters(bundle.plant, bundle.scene_graph, **options)


# ── 规则标签 / 规则结构 ─────────────────────────────────────────────────
def test_rule_labels_are_exactly_the_four_stable_labels():
    from rm75_drake.bimanual.collision_filter import RULE_LABELS, default_dual_arm_rules

    rules = default_dual_arm_rules()
    assert len(rules) == 4
    assert {rule.name for rule in rules} == set(RULE_LABELS)
    assert tuple(rule.name for rule in rules) == RULE_LABELS


def test_default_rules_are_valid_and_carry_rationale():
    from rm75_drake.bimanual.collision_filter import default_dual_arm_rules, validate_rule

    rules = {rule.name: rule for rule in default_dual_arm_rules()}
    for rule in rules.values():
        validate_rule(rule)
        assert rule.rationale, f"规则 {rule.name} 必须写清 rationale"
    # 语义钉死（action = Drake 语义，contact = 物理语义）
    assert rules["adjacent_same_arm"].mechanism == "plant_adjacent"
    assert rules["left_right_mutual"].action == "allow"
    assert rules["left_right_mutual"].contact == "forbid"
    assert rules["gripper_finger_vs_grasped_object"].action == "exclude"
    assert rules["gripper_finger_vs_grasped_object"].contact == "permit"
    assert rules["gripper_finger_vs_own_palm"].contact == "permit"


# ── 规则一：adjacent_same_arm ──────────────────────────────────────────
def test_adjacent_same_arm_keeps_zero_configuration_collision_free(dual_scene):
    """正例：相邻体过滤开启 → 零位形无碰（相邻连杆不误报）。"""
    from rm75_drake.bimanual.dual_scene import check_config_free

    report = _apply(dual_scene)
    assert report.application("adjacent_same_arm").status == "applied"
    assert report.adjacent_filters_enabled is True
    assert check_config_free(dual_scene, np.zeros(dual_scene.num_positions)) is True


def test_disabling_adjacent_filters_reports_collision_at_zero(dual_scene_without_adjacent_filter):
    """反例：关掉相邻体过滤 → 同一零位形出现碰撞（开关语义真的生效）。"""
    from rm75_drake.bimanual.dual_scene import check_config_free

    assert check_config_free(
        dual_scene_without_adjacent_filter,
        np.zeros(dual_scene_without_adjacent_filter.num_positions),
    ) is False


def test_set_adjacent_filters_enabled_after_finalize_raises_naming_the_constraint(dual_scene):
    """post-finalize 设置相邻体过滤 → 显式报错（点名必须装配期设置）。"""
    from rm75_drake.bimanual.collision_filter import FilterRuleError, set_adjacent_filters_enabled

    with pytest.raises(FilterRuleError) as excinfo:
        set_adjacent_filters_enabled(dual_scene.plant, False)
    message = str(excinfo.value)
    assert "Finalize" in message
    assert "adjacent_bodies_collision_filters" in message


# ── 规则二：left_right_mutual ──────────────────────────────────────────
def test_inter_arm_collision_is_still_detected_after_applying_all_rules(dual_scene):
    """反例（对「臂间互检被关掉」）：施加全部规则后左右臂靠近仍然有碰。"""
    from rm75_drake.bimanual.dual_scene import check_config_free

    report = _apply(dual_scene)
    assert report.application("left_right_mutual").status == "applied"

    q = np.zeros(dual_scene.num_positions)
    q[LEFT_J2] = 0.5
    q[RIGHT_J2] = -0.5
    assert check_config_free(dual_scene, q) is False


def test_cross_arm_pairs_stay_collision_candidates(dual_scene):
    """正例：臂间几何对**没有被过滤**（互检保持启用）。"""
    inspector = dual_scene.scene_graph.model_inspector()
    left = _geometry_ids(dual_scene, dual_scene.model_instances[0])
    right = _geometry_ids(dual_scene, dual_scene.model_instances[1])
    assert left and right
    _apply(dual_scene)
    for left_id in left:
        for right_id in right:
            assert inspector.CollisionFiltered(left_id, right_id) is False


def test_excluding_inter_arm_pairs_would_hide_the_collision(dual_scene):
    """反例：若显式 ExcludeBetween(left, right)，臂间碰撞就被藏起来。

    这条断言证明 ``left_right_mutual`` 规则**确实在起作用**（默认规则用的是
    ``allow``，所以碰撞照旧报）。
    """
    from rm75_drake.bimanual.collision_filter import FilterRule, RuleTarget
    from rm75_drake.bimanual.dual_scene import check_config_free

    q = np.zeros(dual_scene.num_positions)
    q[LEFT_J2] = 0.5
    q[RIGHT_J2] = -0.5
    assert check_config_free(dual_scene, q) is False

    hostile = FilterRule(
        name="hostile_exclude_inter_arm",
        action="exclude",
        scope="between",
        pairs=(
            (
                RuleTarget(role="left_arm", label="左臂"),
                RuleTarget(role="right_arm", label="右臂"),
            ),
        ),
        rationale="反例规则：故意关掉臂间互检",
    )
    _apply(dual_scene, rules=[hostile])
    assert check_config_free(dual_scene, q) is True


# ── 规则三：gripper_finger_vs_grasped_object ──────────────────────────
def test_grasped_object_collision_is_reported_without_the_rule(grasped_scene):
    """反例：未施加「放行」规则 → 夹爪指夹住物体被报成碰撞。"""
    from rm75_drake.bimanual.dual_scene import check_config_free

    q = _open_finger_q(grasped_scene)
    assert check_config_free(grasped_scene, q) is False


def test_grasped_object_rule_turns_the_collision_off(grasped_scene):
    """正例：施加「放行」规则 → 判定翻面（**碰撞判定结果变了**，不是只记日志）。"""
    from rm75_drake.bimanual.dual_scene import check_config_free

    q = _open_finger_q(grasped_scene)
    assert check_config_free(grasped_scene, q) is False

    report = _apply(
        grasped_scene,
        grasped_object_instances=list(grasped_scene.obstacle_geometry_ids),
    )
    application = report.application("gripper_finger_vs_grasped_object")
    assert application.status == "applied"
    assert check_config_free(grasped_scene, q) is True


def test_grasped_object_pair_is_filtered_in_the_scene_graph(grasped_scene):
    """正例：过滤**真的作用在场景图**（该对被移出碰撞候选集）。"""
    inspector = grasped_scene.scene_graph.model_inspector()
    fingers = _geometry_ids(grasped_scene, grasped_scene.gripper_model_instances[0], "*finger*")
    obstacle = list(grasped_scene.obstacle_geometry_ids)
    assert fingers and obstacle
    assert inspector.CollisionFiltered(fingers[0], obstacle[0]) is False

    _apply(
        grasped_scene,
        grasped_object_instances=list(grasped_scene.obstacle_geometry_ids),
    )
    assert inspector.CollisionFiltered(fingers[0], obstacle[0]) is True


# ── 规则四：gripper_finger_vs_own_palm ────────────────────────────────
def test_own_palm_pair_is_not_filtered_without_the_rule(gripper_scene_without_adjacent_filter):
    """反例：关掉 plant 的相邻体过滤后，指 vs 掌部**未被**过滤（规则确实做了事）。"""
    inspector = gripper_scene_without_adjacent_filter.scene_graph.model_inspector()
    fingers = _geometry_ids(
        gripper_scene_without_adjacent_filter,
        gripper_scene_without_adjacent_filter.gripper_model_instances[0],
        "*finger*",
    )
    palms = _geometry_ids(
        gripper_scene_without_adjacent_filter,
        gripper_scene_without_adjacent_filter.gripper_model_instances[0],
        "*gripper_base*",
    )
    assert fingers and palms
    assert inspector.CollisionFiltered(fingers[0], palms[0]) is False


def test_own_palm_rule_filters_the_pair(gripper_scene_without_adjacent_filter):
    """正例：施加规则后，指 vs 掌部在场景图里被过滤。"""
    inspector = gripper_scene_without_adjacent_filter.scene_graph.model_inspector()
    fingers = _geometry_ids(
        gripper_scene_without_adjacent_filter,
        gripper_scene_without_adjacent_filter.gripper_model_instances[0],
        "*finger*",
    )
    palms = _geometry_ids(
        gripper_scene_without_adjacent_filter,
        gripper_scene_without_adjacent_filter.gripper_model_instances[0],
        "*gripper_base*",
    )
    report = _apply(gripper_scene_without_adjacent_filter)
    assert report.application("gripper_finger_vs_own_palm").status == "applied"
    assert inspector.CollisionFiltered(fingers[0], palms[0]) is True


# ── 报告 ────────────────────────────────────────────────────────────────
def test_report_records_every_rule_with_status(dual_scene):
    report = _apply(dual_scene)
    assert report.labels == (
        "adjacent_same_arm",
        "left_right_mutual",
        "gripper_finger_vs_grasped_object",
        "gripper_finger_vs_own_palm",
    )
    assert report.application("adjacent_same_arm").status == "applied"
    # 没装夹爪 → 夹爪类规则显式 skipped（不是静默）
    skipped = report.application("gripper_finger_vs_grasped_object")
    assert skipped.status == "skipped"
    assert skipped.reason  # 明确写出为什么跳过
    summary = report.summary()
    assert "碰撞过滤矩阵" in summary
    assert "left_right_mutual" in summary
    assert report.as_dict()["rules"][0]["name"] == "adjacent_same_arm"


def test_report_lookup_of_unknown_rule_raises(dual_scene):
    report = _apply(dual_scene)
    with pytest.raises(Exception) as excinfo:
        report.application("no_such_rule")
    assert "no_such_rule" in str(excinfo.value)


# ── 负例：非法规则必须显式报错并点名 ────────────────────────────────────
def _rule(**kwargs):
    from rm75_drake.bimanual.collision_filter import FilterRule, RuleTarget

    options = {
        "name": "bad_rule",
        "action": "exclude",
        "scope": "between",
        "pairs": ((RuleTarget(role="left_arm"), RuleTarget(role="right_arm")),),
        "rationale": "负例",
    }
    options.update(kwargs)
    return FilterRule(**options)


def test_invalid_action_is_rejected_by_name(dual_scene):
    from rm75_drake.bimanual.collision_filter import FilterRuleError, validate_rule

    with pytest.raises(FilterRuleError) as excinfo:
        validate_rule(_rule(action="maybe"))
    message = str(excinfo.value)
    assert "bad_rule" in message
    assert "maybe" in message


def test_invalid_scope_is_rejected_by_name(dual_scene):
    from rm75_drake.bimanual.collision_filter import FilterRuleError, validate_rule

    with pytest.raises(FilterRuleError) as excinfo:
        validate_rule(_rule(scope="across"))
    message = str(excinfo.value)
    assert "bad_rule" in message
    assert "across" in message


def test_invalid_mechanism_is_rejected_by_name(dual_scene):
    from rm75_drake.bimanual.collision_filter import FilterRuleError, validate_rule

    with pytest.raises(FilterRuleError) as excinfo:
        validate_rule(_rule(mechanism="magic"))
    assert "bad_rule" in str(excinfo.value)


def test_unknown_role_is_rejected_by_name(dual_scene):
    from rm75_drake.bimanual.collision_filter import FilterRuleError, RuleTarget, validate_rule

    rule = _rule(
        pairs=((RuleTarget(role="third_arm"), RuleTarget(role="right_arm")),),
    )
    with pytest.raises(FilterRuleError) as excinfo:
        validate_rule(rule)
    assert "third_arm" in str(excinfo.value)


def test_within_scope_requires_a_self_pair(dual_scene):
    from rm75_drake.bimanual.collision_filter import FilterRuleError, RuleTarget, validate_rule

    rule = _rule(
        scope="within",
        pairs=((RuleTarget(role="left_arm"), RuleTarget(role="right_arm")),),
    )
    with pytest.raises(FilterRuleError) as excinfo:
        validate_rule(rule)
    assert "within" in str(excinfo.value)


def test_nonexistent_model_instance_is_rejected_by_name(dual_scene):
    """负例：规则引用不存在的 model instance → 显式报错（点名），不静默跳过。"""
    from rm75_drake.bimanual.collision_filter import FilterRuleError, RuleTarget

    rule = _rule(
        pairs=((RuleTarget(model="RM75-B-ghost"), RuleTarget(role="right_arm")),),
    )
    with pytest.raises(FilterRuleError) as excinfo:
        _apply(dual_scene, rules=[rule])
    message = str(excinfo.value)
    assert "RM75-B-ghost" in message


def test_nonexistent_model_instance_index_is_rejected(dual_scene):
    from pydrake.multibody.tree import ModelInstanceIndex

    from rm75_drake.bimanual.collision_filter import FilterRuleError, RuleTarget

    rule = _rule(
        pairs=((RuleTarget(model=ModelInstanceIndex(99)), RuleTarget(role="right_arm")),),
    )
    with pytest.raises(FilterRuleError) as excinfo:
        _apply(dual_scene, rules=[rule])
    assert "99" in str(excinfo.value)


def test_rule_without_pairs_is_rejected(dual_scene):
    from rm75_drake.bimanual.collision_filter import FilterRuleError, validate_rule

    with pytest.raises(FilterRuleError) as excinfo:
        validate_rule(_rule(pairs=()))
    assert "bad_rule" in str(excinfo.value)


# ── within 作用域（ExcludeWithin）的可执行效果 ──────────────────────────
def test_within_scope_rule_excludes_intra_arm_self_collision(dual_scene):
    """``scope="within"`` 通路：排除左臂内部配对 → 同臂自碰不再报。"""
    from rm75_drake.bimanual.collision_filter import FilterRule, RuleTarget
    from rm75_drake.bimanual.dual_scene import check_config_free

    # 左臂 joint_4 = 3.0 rad → 左臂 link_1/link_6 等**同臂非相邻**连杆互相穿透
    # （纯自碰，不是臂间互碰）。
    q = np.zeros(dual_scene.num_positions)
    q[LEFT_J4] = 3.0
    assert check_config_free(dual_scene, q) is False  # 同臂自碰（反例基线）

    rule = FilterRule(
        name="exclude_within_left_arm",
        action="exclude",
        scope="within",
        pairs=((RuleTarget(role="left_arm"), RuleTarget(role="left_arm")),),
        rationale="负例用：把左臂内部配对整体排除",
    )
    _apply(dual_scene, rules=[rule])
    assert check_config_free(dual_scene, q) is True
