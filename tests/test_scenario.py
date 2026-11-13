"""scenario 正式 schema 的正例与负例（S02-T01）。

覆盖：
* 正例：仓库内 ``scenarios/single_arm.yaml`` 能被加载、顶层四键不变；
* 负例：未知顶层键 / 未知子键 / 缺必填字段 / 版本不对 / 维度不对 / 单位写错。

负例一律断言异常类型（:class:`rm75_drake.scenario.ScenarioError`）**且**
断言异常信息里含有出错的键名或字段名（避免"报了个错但不知道错在哪"）。
"""

from __future__ import annotations

import copy
from pathlib import Path

import pytest

from rm75_drake import scenario as scenario_mod
from rm75_drake.scenario import (
    SCHEMA_VERSION,
    ScenarioError,
    load_scenario,
    load_scenario_dict,
)

REPO_ROOT = Path(__file__).resolve().parents[1]
SCENARIO_PATH = REPO_ROOT / "scenarios" / "single_arm.yaml"


def _minimal_dict() -> dict:
    """一份最小的合法场景（schema_version 1）。"""
    return {
        "schema_version": 1,
        "robots": [
            {
                "name": "rm75",
                "urdf": "package://rm75_drake/rm75_drake.urdf",
                "base_link": "base_link",
                "joint_names": [f"joint_{i}" for i in range(1, 8)],
            }
        ],
        "base_extrinsics": {"rm75": {"xyz": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]}},
        "obstacles": [
            {
                "name": "workbench_box",
                "type": "box",
                "size": [0.6, 0.6, 0.05],
                "pose": {"xyz": [0.45, 0.0, 0.025], "rpy": [0.0, 0.0, 0.0]},
            }
        ],
        "calibration": {},
    }


# ── 正例 ────────────────────────────────────────────────────────────────
def test_repository_scenario_loads():
    scenario = load_scenario(SCENARIO_PATH)
    assert scenario.schema_version == SCHEMA_VERSION
    assert scenario.source_path == str(SCENARIO_PATH)

    robot = scenario.robot("rm75")
    assert robot.urdf == "package://rm75_drake/rm75_drake.urdf"
    assert robot.base_link == "base_link"
    assert robot.joint_names == tuple(f"joint_{i}" for i in range(1, 8))

    assert len(scenario.obstacles) == 1
    obstacle = scenario.obstacles[0]
    assert obstacle.type == "box"
    assert obstacle.size == (0.6, 0.6, 0.05)
    # size 是外形全长，Drake Box 用半长。
    assert obstacle.half_size == pytest.approx((0.3, 0.3, 0.025))

    extrinsic = scenario.base_extrinsics["rm75"]
    assert extrinsic.xyz == (0.0, 0.0, 0.0)
    assert extrinsic.rpy == (0.0, 0.0, 0.0)

    assert "gripper_force" in scenario.calibration


def test_top_level_keys_are_unchanged():
    """顶层四键固定（改名会击穿 S07 golden-file 回归）。"""
    import yaml

    raw = yaml.safe_load(SCENARIO_PATH.read_text(encoding="utf-8"))
    assert set(raw) == {
        "schema_version",
        "robots",
        "base_extrinsics",
        "obstacles",
        "calibration",
    }
    assert set(scenario_mod.TOP_LEVEL_KEYS) == set(raw)


def test_load_scenario_dict_minimal():
    scenario = load_scenario_dict(_minimal_dict())
    assert scenario.schema_version == 1
    assert scenario.robot_names == ("rm75",)
    assert len(scenario.obstacles) == 1
    assert scenario.calibration == {}


def test_robot_lookup_unknown_name_raises():
    scenario = load_scenario_dict(_minimal_dict())
    with pytest.raises(ScenarioError) as excinfo:
        scenario.robot("no_such_robot")
    assert "no_such_robot" in str(excinfo.value)


def test_scenario_error_is_value_error():
    assert issubclass(ScenarioError, ValueError)


# ── 负例：未知键 ────────────────────────────────────────────────────────
def test_unknown_top_level_key_lists_the_key():
    data = _minimal_dict()
    data["extra_section"] = {"a": 1}
    with pytest.raises(ScenarioError) as excinfo:
        load_scenario_dict(data)
    assert "extra_section" in str(excinfo.value)


def test_unknown_robot_subkey_lists_the_key():
    data = _minimal_dict()
    data["robots"][0]["payload"] = 3.0
    with pytest.raises(ScenarioError) as excinfo:
        load_scenario_dict(data)
    assert "payload" in str(excinfo.value)


def test_unknown_obstacle_subkey_lists_the_key():
    data = _minimal_dict()
    data["obstacles"][0]["colour"] = "red"
    with pytest.raises(ScenarioError) as excinfo:
        load_scenario_dict(data)
    assert "colour" in str(excinfo.value)


def test_unknown_calibration_key_lists_the_key():
    data = _minimal_dict()
    data["calibration"]["magic"] = 1
    with pytest.raises(ScenarioError) as excinfo:
        load_scenario_dict(data)
    assert "magic" in str(excinfo.value)


def test_unknown_pose_subkey_lists_the_key():
    data = _minimal_dict()
    data["obstacles"][0]["pose"]["quaternion"] = [0, 0, 0, 1]
    with pytest.raises(ScenarioError) as excinfo:
        load_scenario_dict(data)
    assert "quaternion" in str(excinfo.value)


# ── 负例：缺必填字段 ────────────────────────────────────────────────────
def test_missing_top_level_key_raises():
    data = _minimal_dict()
    del data["calibration"]
    with pytest.raises(ScenarioError) as excinfo:
        load_scenario_dict(data)
    assert "calibration" in str(excinfo.value)


def test_missing_robot_field_raises():
    data = _minimal_dict()
    del data["robots"][0]["base_link"]
    with pytest.raises(ScenarioError) as excinfo:
        load_scenario_dict(data)
    assert "base_link" in str(excinfo.value)


def test_missing_extrinsic_raises_no_silent_default():
    """机器人缺外参 → 报错（禁止静默默认成单位阵）。"""
    data = _minimal_dict()
    data["base_extrinsics"] = {}
    with pytest.raises(ScenarioError) as excinfo:
        load_scenario_dict(data)
    assert "rm75" in str(excinfo.value)


def test_missing_extrinsic_field_raises():
    data = _minimal_dict()
    del data["base_extrinsics"]["rm75"]["rpy"]
    with pytest.raises(ScenarioError) as excinfo:
        load_scenario_dict(data)
    assert "rpy" in str(excinfo.value)


def test_empty_robots_raises():
    data = _minimal_dict()
    data["robots"] = []
    with pytest.raises(ScenarioError) as excinfo:
        load_scenario_dict(data)
    assert "robots" in str(excinfo.value)


# ── 负例：版本 ──────────────────────────────────────────────────────────
def test_placeholder_schema_version_0_rejected():
    data = _minimal_dict()
    data["schema_version"] = 0
    with pytest.raises(ScenarioError) as excinfo:
        load_scenario_dict(data)
    assert "schema_version" in str(excinfo.value)


def test_future_schema_version_rejected():
    data = _minimal_dict()
    data["schema_version"] = 99
    with pytest.raises(ScenarioError) as excinfo:
        load_scenario_dict(data)
    assert "schema_version" in str(excinfo.value)


# ── 负例：维度与单位 ────────────────────────────────────────────────────
def test_xyz_must_have_three_components():
    data = _minimal_dict()
    data["base_extrinsics"]["rm75"]["xyz"] = [0.0, 0.0]
    with pytest.raises(ScenarioError) as excinfo:
        load_scenario_dict(data)
    assert "xyz" in str(excinfo.value)


def test_rpy_must_have_three_components():
    data = _minimal_dict()
    data["base_extrinsics"]["rm75"]["rpy"] = [0.0, 0.0, 0.0, 0.0]
    with pytest.raises(ScenarioError) as excinfo:
        load_scenario_dict(data)
    assert "rpy" in str(excinfo.value)


def test_rpy_in_degrees_is_rejected():
    """rpy 必须是弧度：写成角度制必须报错（并给换算提示）。"""
    data = _minimal_dict()
    data["base_extrinsics"]["rm75"]["rpy"] = [90.0, 0.0, 0.0]
    with pytest.raises(ScenarioError) as excinfo:
        load_scenario_dict(data)
    message = str(excinfo.value)
    assert "rad" in message and "rpy" in message


def test_obstacle_pose_rpy_in_degrees_is_rejected():
    data = _minimal_dict()
    data["obstacles"][0]["pose"]["rpy"] = [0.0, 0.0, 180.0]
    with pytest.raises(ScenarioError) as excinfo:
        load_scenario_dict(data)
    assert "rpy" in str(excinfo.value)


def test_xyz_in_millimetres_is_rejected():
    """长度一律用米：把毫米当米填必须报错。"""
    data = _minimal_dict()
    data["base_extrinsics"]["rm75"]["xyz"] = [450.0, 0.0, 0.0]
    with pytest.raises(ScenarioError) as excinfo:
        load_scenario_dict(data)
    assert "m" in str(excinfo.value)


def test_obstacle_size_must_have_three_components():
    data = _minimal_dict()
    data["obstacles"][0]["size"] = [0.6, 0.6]
    with pytest.raises(ScenarioError) as excinfo:
        load_scenario_dict(data)
    assert "size" in str(excinfo.value)


def test_obstacle_size_must_be_positive():
    data = _minimal_dict()
    data["obstacles"][0]["size"] = [0.6, 0.0, 0.05]
    with pytest.raises(ScenarioError) as excinfo:
        load_scenario_dict(data)
    assert "size" in str(excinfo.value)


def test_obstacle_size_must_be_negative_rejected():
    data = _minimal_dict()
    data["obstacles"][0]["size"] = [-0.6, 0.6, 0.05]
    with pytest.raises(ScenarioError) as excinfo:
        load_scenario_dict(data)
    assert "size" in str(excinfo.value)


def test_unsupported_obstacle_type_rejected():
    data = _minimal_dict()
    data["obstacles"][0]["type"] = "cylinder"
    with pytest.raises(ScenarioError) as excinfo:
        load_scenario_dict(data)
    assert "cylinder" in str(excinfo.value)


def test_non_numeric_value_rejected():
    data = _minimal_dict()
    data["base_extrinsics"]["rm75"]["xyz"] = ["0.0", "0.0", "0.0"]
    with pytest.raises(ScenarioError) as excinfo:
        load_scenario_dict(data)
    assert "xyz" in str(excinfo.value)


def test_non_finite_value_rejected():
    data = _minimal_dict()
    data["base_extrinsics"]["rm75"]["xyz"] = [float("nan"), 0.0, 0.0]
    with pytest.raises(ScenarioError) as excinfo:
        load_scenario_dict(data)
    assert "xyz" in str(excinfo.value)


def test_root_must_be_mapping():
    with pytest.raises(ScenarioError):
        load_scenario_dict(["not", "a", "mapping"])


# ── 负例：文件与标定 ────────────────────────────────────────────────────
def test_missing_file_raises_file_not_found():
    with pytest.raises(FileNotFoundError):
        load_scenario(REPO_ROOT / "scenarios" / "no_such_scenario.yaml")


def test_malformed_yaml_raises_scenario_error(tmp_path):
    bad = tmp_path / "bad.yaml"
    bad.write_text("robots: [\n", encoding="utf-8")
    with pytest.raises(ScenarioError) as excinfo:
        load_scenario(bad)
    assert "YAML" in str(excinfo.value)


def test_empty_file_raises_scenario_error(tmp_path):
    empty = tmp_path / "empty.yaml"
    empty.write_text("", encoding="utf-8")
    with pytest.raises(ScenarioError):
        load_scenario(empty)


def test_missing_calibration_file_raises():
    data = _minimal_dict()
    data["calibration"] = {"gripper_force": {"file": "no_such_calibration.yaml"}}
    with pytest.raises(ScenarioError) as excinfo:
        load_scenario_dict(data)
    assert "gripper_force" in str(excinfo.value)


def test_scenario_dict_is_not_mutated_by_loading():
    """加载不修改调用方传入的 dict（避免"半加载"状态）。"""
    data = _minimal_dict()
    snapshot = copy.deepcopy(data)
    load_scenario_dict(data)
    assert data == snapshot
