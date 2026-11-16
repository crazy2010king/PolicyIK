"""任务 / 物体 / 原语 schema 与原子目标测试（S06-T01）。

覆盖
----
* **正例**：``tasks/pick_place.yaml`` / ``tasks/handover.yaml`` 两个部署实例
  可被严格 schema 解析，原语链与目标种类符合预期；
* **反例逐条点名**：未知顶层键 / 未知子键 / 未知原语名 / 缺字段 /
  单位错（rpy 角度制、长度毫米）/ 维度错 / 重复 goal_id / 非法 kind /
  非法 action / 未声明的 target_object ⇒ 抛 ``TaskError`` 且消息**点名该键**；
* **惰性导入**：`import rm75_drake.tasks` 后 **pydrake 未被拉起**
  （独立解释器子进程断言）。
"""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

import pytest

from rm75_drake import paths
from rm75_drake.tasks import (
    GOAL_KINDS,
    GRIPPER_ACTIONS,
    PRIMITIVE_KINDS,
    PRIMITIVE_NAMES,
    SCHEMA_VERSION,
    GoalError,
    ObjectGoal,
    PoseGoal,
    RelativePoseGoal,
    TaskError,
    TaskGoal,
    build_goal_registry,
    load_task_file,
    parse_task,
)

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = REPO_ROOT / "src"


def _valid_task_dict() -> dict:
    """一份合法的任务内容（内存 dict）。"""
    return {
        "schema_version": 1,
        "name": "demo",
        "description": "demo task",
        "robot": "left",
        "scenario": "single_arm.yaml",
        "objects": [
            {
                "name": "cube_a",
                "half_size": [0.02, 0.02, 0.02],
                "pose": {"xyz": [0.45, 0.0, 0.02], "rpy": [0.0, 0.0, 0.0]},
            }
        ],
        "thresholds": {"position_m": 0.005, "orientation_rad": 0.05},
        "primitives": [
            {
                "name": "approach",
                "kind": "motion",
                "target": {"xyz": [0.45, 0.0, 0.18], "rpy": [0.0, 0.0, 0.0]},
                "tolerance": {"position_m": 0.01, "orientation_rad": 0.1},
            },
            {
                "name": "close",
                "kind": "gripper",
                "action": "close",
                "target_object": "cube_a",
                "hold_seconds": 0.5,
            },
        ],
    }


# ── 正例 ────────────────────────────────────────────────────────────────
def test_pick_place_yaml_parses() -> None:
    """``tasks/pick_place.yaml`` 可被严格 schema 解析，原语链齐全。"""
    spec = load_task_file(paths.task_files_dir / "pick_place.yaml")

    assert spec.schema_version == SCHEMA_VERSION
    assert spec.name == "pick_place"
    assert spec.robot == "left"
    assert spec.scenario == "single_arm.yaml"
    assert [p.name for p in spec.primitives] == [
        "approach",
        "pre_grasp",
        "close",
        "verify",
        "lift",
        "transit",
        "place",
        "open",
        "retreat",
    ]
    assert spec.primitive("close").kind == "gripper"
    assert spec.primitive("close").action == "close"
    assert spec.primitive("close").target_object == "cube_a"
    assert spec.primitive("place").target is not None
    assert spec.thresholds["position_m"] == 0.005
    assert spec.object("cube_a").half_size == (0.02, 0.02, 0.02)


def test_handover_yaml_parses() -> None:
    """``tasks/handover.yaml`` 可被解析，含双臂原语（bimanual_motion）。"""
    spec = load_task_file(paths.task_files_dir / "handover.yaml")

    assert spec.name == "handover"
    assert spec.robot == "dual_arm"
    kinds = {p.name: p.kind for p in spec.primitives}
    assert kinds["transit"] == "bimanual_motion"
    assert kinds["handover"] == "bimanual_motion"
    assert kinds["close"] == "gripper"
    # 双臂原语的相对位姿必须声明参照物体
    assert spec.primitive("transit").target_object == "crate_a"


def test_goals_are_built_per_primitive_kind() -> None:
    """原语 → 原子目标的映射：motion→pose / bimanual_motion→relative / gripper→object。"""
    spec = load_task_file(paths.task_files_dir / "handover.yaml")
    goals = spec.goals()

    assert len(goals) == len(spec.primitives)
    assert all(isinstance(g, TaskGoal) for g in goals)
    assert all(g.goal_id in spec.goal_ids() for g in goals)
    assert isinstance(spec.goal_registry()[spec.goal_ids()[0]], PoseGoal)
    assert isinstance(spec.goal_registry()["bimanual_transit"], RelativePoseGoal)
    assert isinstance(spec.goal_registry()["left_close"], ObjectGoal)
    for goal in goals:
        assert goal.kind in GOAL_KINDS


def test_goal_registry_rejects_duplicate_goal_id() -> None:
    """目标表构造时重复 goal_id ⇒ 报错并点名。"""
    with pytest.raises(GoalError) as exc:
        build_goal_registry(
            [PoseGoal(goal_id="g1"), PoseGoal(goal_id="g1", xyz=(1.0, 0.0, 0.0))]
        )
    assert "g1" in str(exc.value)


def test_valid_task_dict_parses() -> None:
    """内存 dict 正例。"""
    spec = parse_task(_valid_task_dict())
    assert spec.name == "demo"
    assert len(spec.primitives) == 2
    assert spec.goal_ids() == ("approach", "close")


# ── 反例：未知键 ────────────────────────────────────────────────────────
def test_unknown_top_level_key_is_named() -> None:
    """未知顶层键 ⇒ TaskError 且点名该键。"""
    data = _valid_task_dict()
    data["unexpected_key"] = 1
    with pytest.raises(TaskError) as exc:
        parse_task(data)
    assert "unexpected_key" in str(exc.value)


def test_unknown_object_subkey_is_named() -> None:
    """未知子键 ⇒ TaskError 且点名该键。"""
    data = _valid_task_dict()
    data["objects"][0]["colour"] = "red"
    with pytest.raises(TaskError) as exc:
        parse_task(data)
    assert "colour" in str(exc.value)


def test_unknown_primitive_subkey_is_named() -> None:
    """原语条目的未知子键 ⇒ TaskError 且点名该键。"""
    data = _valid_task_dict()
    data["primitives"][0]["speed"] = 0.1
    with pytest.raises(TaskError) as exc:
        parse_task(data)
    assert "speed" in str(exc.value)


def test_unknown_threshold_key_is_named() -> None:
    """thresholds 的未知子键 ⇒ TaskError 且点名该键。"""
    data = _valid_task_dict()
    data["thresholds"]["time_s"] = 1.0
    with pytest.raises(TaskError) as exc:
        parse_task(data)
    assert "time_s" in str(exc.value)


# ── 反例：原语名 / kind / action 白名单 ──────────────────────────────────
def test_unknown_primitive_name_is_named() -> None:
    """未知原语名 ⇒ TaskError 且点名该名字。"""
    data = _valid_task_dict()
    data["primitives"][0]["name"] = "fly_away"
    with pytest.raises(TaskError) as exc:
        parse_task(data)
    assert "fly_away" in str(exc.value)
    # 白名单本身必须是那 10 个原语
    assert set(PRIMITIVE_NAMES) == {
        "approach",
        "pre_grasp",
        "close",
        "verify",
        "lift",
        "transit",
        "place",
        "open",
        "retreat",
        "handover",
    }


def test_unknown_primitive_kind_is_named() -> None:
    """非法 kind ⇒ TaskError 且点名。"""
    data = _valid_task_dict()
    data["primitives"][0]["kind"] = "teleport"
    with pytest.raises(TaskError) as exc:
        parse_task(data)
    assert "teleport" in str(exc.value)
    assert set(PRIMITIVE_KINDS) == {"motion", "gripper", "bimanual_motion"}


def test_unknown_gripper_action_is_named() -> None:
    """非法 action ⇒ TaskError 且点名。"""
    data = _valid_task_dict()
    data["primitives"][1]["action"] = "squeeze"
    with pytest.raises(TaskError) as exc:
        parse_task(data)
    assert "squeeze" in str(exc.value)
    assert set(GRIPPER_ACTIONS) == {"close", "open"}


# ── 反例：缺字段 ────────────────────────────────────────────────────────
def test_missing_top_level_field_is_named() -> None:
    """缺必填顶层键 ⇒ TaskError 且点名缺哪个键。"""
    data = _valid_task_dict()
    del data["robot"]
    with pytest.raises(TaskError) as exc:
        parse_task(data)
    assert "robot" in str(exc.value)


def test_missing_primitive_field_is_named() -> None:
    """缺必填原语字段（kind）⇒ TaskError 且点名。"""
    data = _valid_task_dict()
    del data["primitives"][0]["kind"]
    with pytest.raises(TaskError) as exc:
        parse_task(data)
    assert "kind" in str(exc.value)


def test_gripper_primitive_requires_action_and_target_object() -> None:
    """kind=gripper 必须显式给出 action 与 target_object（点名缺失字段）。"""
    data = _valid_task_dict()
    del data["primitives"][1]["action"]
    with pytest.raises(TaskError) as exc:
        parse_task(data)
    assert "action" in str(exc.value)

    data = _valid_task_dict()
    del data["primitives"][1]["target_object"]
    with pytest.raises(TaskError) as exc:
        parse_task(data)
    assert "target_object" in str(exc.value)


def test_motion_primitive_requires_target() -> None:
    """kind=motion 必须给出 target（点名缺失字段）。"""
    data = _valid_task_dict()
    del data["primitives"][0]["target"]
    with pytest.raises(TaskError) as exc:
        parse_task(data)
    assert "target" in str(exc.value)


def test_action_not_allowed_on_motion_primitive() -> None:
    """非夹爪原语不得出现 action（点名该键）。"""
    data = _valid_task_dict()
    data["primitives"][0]["action"] = "close"
    with pytest.raises(TaskError) as exc:
        parse_task(data)
    assert "action" in str(exc.value)


def test_undeclared_target_object_is_named() -> None:
    """target_object 未在 objects 中声明 ⇒ 点名。"""
    data = _valid_task_dict()
    data["primitives"][1]["target_object"] = "ghost"
    with pytest.raises(TaskError) as exc:
        parse_task(data)
    assert "ghost" in str(exc.value)


# ── 反例：单位 / 维度 ───────────────────────────────────────────────────
def test_rpy_in_degrees_is_rejected() -> None:
    """rpy 用角度制（> π）⇒ TaskError 并点名该字段。"""
    data = _valid_task_dict()
    data["primitives"][0]["target"]["rpy"] = [90.0, 0.0, 0.0]
    with pytest.raises(TaskError) as exc:
        parse_task(data)
    assert "rpy" in str(exc.value)


def test_length_in_millimeters_is_rejected() -> None:
    """长度写成毫米（> 100 m）⇒ TaskError 并点名该字段。"""
    data = _valid_task_dict()
    data["primitives"][0]["target"]["xyz"] = [450.0, 0.0, 180.0]
    with pytest.raises(TaskError) as exc:
        parse_task(data)
    assert "xyz" in str(exc.value)


def test_wrong_dimension_is_named() -> None:
    """xyz 不是 3 维 ⇒ TaskError 并点名该字段。"""
    data = _valid_task_dict()
    data["primitives"][0]["target"]["xyz"] = [0.45, 0.0]
    with pytest.raises(TaskError) as exc:
        parse_task(data)
    assert "xyz" in str(exc.value)


def test_half_size_must_be_three_numbers() -> None:
    """half_size 维度错 ⇒ 点名该字段。"""
    data = _valid_task_dict()
    data["objects"][0]["half_size"] = [0.02, 0.02]
    with pytest.raises(TaskError) as exc:
        parse_task(data)
    assert "half_size" in str(exc.value)


def test_negative_hold_seconds_is_rejected() -> None:
    """hold_seconds 为负 ⇒ 点名该字段。"""
    data = _valid_task_dict()
    data["primitives"][1]["hold_seconds"] = -1.0
    with pytest.raises(TaskError) as exc:
        parse_task(data)
    assert "hold_seconds" in str(exc.value)


# ── 反例：重复 goal_id ──────────────────────────────────────────────────
def test_duplicate_goal_id_is_named() -> None:
    """同一任务内重复 goal_id ⇒ 报错并点名该 goal_id。"""
    data = _valid_task_dict()
    data["primitives"][1]["goal_id"] = "approach"
    with pytest.raises(TaskError) as exc:
        parse_task(data)
    assert "approach" in str(exc.value)


def test_duplicate_primitive_name_without_explicit_goal_id_is_named() -> None:
    """未显式给 goal_id 时，重复的原语名 ⇒ 重复 goal_id 报错并点名。"""
    data = _valid_task_dict()
    data["primitives"].append(
        {
            "name": "close",
            "kind": "gripper",
            "action": "close",
            "target_object": "cube_a",
        }
    )
    with pytest.raises(TaskError) as exc:
        parse_task(data)
    assert "close" in str(exc.value)


# ── 惰性导入（零 pydrake 依赖）──────────────────────────────────────────
def test_tasks_package_import_does_not_load_pydrake() -> None:
    """独立解释器里 ``import rm75_drake.tasks`` 不得把 pydrake 拉进 sys.modules。"""
    env = dict(os.environ)
    env["PYTHONPATH"] = str(SRC_DIR) + os.pathsep + env.get("PYTHONPATH", "")
    code = (
        "import sys; import rm75_drake.tasks; "
        "assert 'pydrake' not in sys.modules, sorted(sys.modules); "
        # PEP 562 惰性：import 本包不拉起子模块
        "assert 'rm75_drake.tasks.schema' not in sys.modules; "
        # 惰性取符号后子模块才被拉起，且仍然不引入 pydrake
        "assert callable(rm75_drake.tasks.parse_task); "
        "assert 'rm75_drake.tasks.schema' in sys.modules; "
        "assert 'pydrake' not in sys.modules, sorted(sys.modules); "
        "print('ok')"
    )
    proc = subprocess.run(
        [sys.executable, "-c", code],
        cwd=str(REPO_ROOT),
        env=env,
        capture_output=True,
        text=True,
        timeout=120,
    )
    assert proc.returncode == 0, proc.stderr
    assert "ok" in proc.stdout
