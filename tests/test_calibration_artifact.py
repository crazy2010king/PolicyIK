"""标定产出物与场景消费链路（S04-T05）。

覆盖：
* ``calibration/dual_arm.yaml``（仓库根，部署实例标定）能被
  :func:`rm75_drake.calibration.schema.load_calibration_file` 加载，
  且 **未实测不得冒充已标定**（``measured=false`` / ``calibrated=false``）；
* ``scenarios/dual_arm.yaml`` 通过 **引用** 消费标定文件，
  场景的 ``base_extrinsics`` 与标定文件**一致**（唯一真相源生效）；
* 手写 ``base_extrinsics`` 与标定文件冲突 → **以标定文件为准**；
* 负例：标定文件缺失 / schema 非法（未知键）/ 机器人不匹配 → ``ScenarioError``
  并**点名路径与出错键**（不得静默回退到手写值后假装成功）；
* 回归：``scenarios/single_arm.yaml`` 仍成功、行为完全不变；
  scenario 的顶层四键与 ``schema_version`` **未变**。

外部依赖：只有文件系统（YAML 读取）。无网络、无子进程、无真机。
"""

from __future__ import annotations

import copy
from pathlib import Path

import pytest
import yaml

from rm75_drake import paths
from rm75_drake import scenario as scenario_mod
from rm75_drake.calibration.schema import load_calibration_file
from rm75_drake.scenario import ScenarioError, load_scenario, load_scenario_dict

REPO_ROOT = Path(__file__).resolve().parents[1]
CALIBRATION_FILE = REPO_ROOT / "calibration" / "dual_arm.yaml"
SAMPLES_FILE = REPO_ROOT / "calibration" / "samples" / "demo.yaml"
DUAL_SCENARIO_PATH = REPO_ROOT / "scenarios" / "dual_arm.yaml"
SINGLE_SCENARIO_PATH = REPO_ROOT / "scenarios" / "single_arm.yaml"


def _dual_scenario_dict() -> dict:
    """一份与 scenarios/dual_arm.yaml 等价的 dict（便于改字段做负例/反例）。"""
    return {
        "schema_version": 1,
        "robots": [
            {
                "name": "left",
                "urdf": "package://rm75_drake/rm75_left.urdf",
                "base_link": "left_base_link",
                "joint_names": [f"left_joint_{i}" for i in range(1, 8)],
            },
            {
                "name": "right",
                "urdf": "package://rm75_drake/rm75_right.urdf",
                "base_link": "right_base_link",
                "joint_names": [f"right_joint_{i}" for i in range(1, 8)],
            },
        ],
        "base_extrinsics": {
            "left": {"xyz": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]},
            "right": {"xyz": [0.6, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]},
        },
        "obstacles": [],
        "calibration": {"base_extrinsic": {"file": "dual_arm.yaml"}},
    }


# ── 路径层：仓库根标定目录 vs 包内标定目录 ────────────────────────────
def test_calibration_files_dir_is_repo_root_and_distinct_from_package_dir():
    """两个同名目录必须分清：包内（随包分发）vs 仓库根（部署实例）。"""
    assert paths.calibration_files_dir == paths.REPO_ROOT / "calibration"
    assert paths.calibration_files_dir.is_dir()
    assert paths.calibration_dir != paths.calibration_files_dir
    assert paths.calibration_dir == paths.package_dir / "calibration"
    assert (paths.calibration_files_dir / "dual_arm.yaml").is_file()


# ── 标定产出文件本身 ──────────────────────────────────────────────────
def test_calibration_file_loads_and_declares_unmeasured():
    doc = load_calibration_file(CALIBRATION_FILE)
    assert doc.schema_version == 1
    # 未实测不得冒充已标定
    assert doc.measured is False
    assert doc.calibrated is False
    assert doc.source_path == str(CALIBRATION_FILE)


def test_calibration_file_declares_no_fabricated_precision():
    """残差只能是占位（samples=0），不得写成"实测且零误差"。"""
    doc = load_calibration_file(CALIBRATION_FILE)
    assert set(doc.base_extrinsic) == {"left", "right"}
    for robot, entry in doc.base_extrinsic.items():
        assert entry.residual is not None, f"{robot} 缺残差摘要"
        # samples = 0 ⇒ 没有任何实测样本 ⇒ 数字不得被解读为实测精度
        assert entry.residual.samples == 0
        assert entry.residual.within_threshold is False


def test_calibration_file_base_extrinsic_matches_design_layout():
    doc = load_calibration_file(CALIBRATION_FILE)
    assert doc.base_extrinsic["left"].xyz == (0.0, 0.0, 0.0)
    assert doc.base_extrinsic["right"].xyz == (0.6, 0.0, 0.0)
    assert doc.base_extrinsic["right"].rpy == (0.0, 0.0, 0.0)
    # 关节零位：每臂恰好 7 个关节
    for robot in ("left", "right"):
        assert len(doc.joint_zero_offset[robot]) == 7
    assert set(doc.tool_offset) == {"left", "right"}


def test_demo_samples_file_is_synthetic():
    """演练用采样点集：每条采样都标注 synthetic=true + source=synthetic。"""
    from rm75_drake.calibration.samples import load_samples_file

    samples = load_samples_file(SAMPLES_FILE)
    assert len(samples.joint_zero) == 7
    assert len(samples.tool_tcp) >= 4
    assert len(samples.base_extrinsic) >= 15
    for group in (samples.joint_zero, samples.tool_tcp, samples.base_extrinsic):
        for item in group:
            assert item.synthetic is True
            assert item.source == "synthetic"


# ── 场景消费链路（唯一真相源）──────────────────────────────────────────
def test_dual_scenario_consumes_calibration_file():
    scenario = load_scenario(DUAL_SCENARIO_PATH)
    doc = load_calibration_file(CALIBRATION_FILE)

    # 场景引用了标定文件
    assert scenario.calibration["base_extrinsic"]["file"] == str(CALIBRATION_FILE)

    # 场景的基座外参 == 标定文件里的基座外参（唯一真相源）
    assert set(scenario.base_extrinsics) == set(doc.base_extrinsic)
    for robot, entry in doc.base_extrinsic.items():
        got = scenario.base_extrinsics[robot]
        assert got.xyz == entry.xyz
        assert got.rpy == entry.rpy


def test_handwritten_extrinsics_conflict_is_overridden_by_calibration_file():
    """反例：手写值与标定文件冲突 → 最终值以**标定文件**为准。"""
    data = _dual_scenario_dict()
    data["base_extrinsics"]["right"] = {"xyz": [9.9, -9.9, 9.9], "rpy": [0.5, 0.5, 0.5]}
    scenario = load_scenario_dict(data)
    doc = load_calibration_file(CALIBRATION_FILE)
    assert scenario.base_extrinsics["right"].xyz == doc.base_extrinsic["right"].xyz
    assert scenario.base_extrinsics["right"].xyz == (0.6, 0.0, 0.0)
    assert scenario.base_extrinsics["right"].rpy == (0.0, 0.0, 0.0)


def test_scenario_without_calibration_reference_keeps_handwritten_values():
    """不引用标定文件时，手写 base_extrinsics 原样生效（回退路径）。"""
    data = _dual_scenario_dict()
    data["calibration"] = {}
    scenario = load_scenario_dict(data)
    assert scenario.base_extrinsics["right"].xyz == (0.6, 0.0, 0.0)
    assert "base_extrinsic" not in scenario.calibration


# ── 负例：缺失 / 非法 / 不匹配 ─────────────────────────────────────────
def test_missing_calibration_file_raises_scenario_error_naming_path():
    data = _dual_scenario_dict()
    data["calibration"] = {"base_extrinsic": {"file": "no_such_calibration.yaml"}}
    with pytest.raises(ScenarioError) as excinfo:
        load_scenario_dict(data)
    message = str(excinfo.value)
    assert "no_such_calibration.yaml" in message
    assert str(paths.calibration_files_dir) in message


def test_calibration_file_with_unknown_key_raises_scenario_error_naming_key(tmp_path):
    bad = tmp_path / "bad_calibration.yaml"
    bad.write_text(
        "schema_version: 1\n"
        "measured: false\n"
        "calibrated: false\n"
        "method: demo\n"
        "date: '2026-11-13'\n"
        "oops_key: 1\n",
        encoding="utf-8",
    )
    data = _dual_scenario_dict()
    data["calibration"] = {"base_extrinsic": {"file": str(bad)}}
    with pytest.raises(ScenarioError) as excinfo:
        load_scenario_dict(data)
    message = str(excinfo.value)
    assert "oops_key" in message  # 点名出错键
    assert str(bad) in message  # 点名出错文件路径


def test_calibration_file_with_missing_required_key_raises_naming_key(tmp_path):
    bad = tmp_path / "incomplete_calibration.yaml"
    bad.write_text(
        "schema_version: 1\nmeasured: false\ncalibrated: false\ndate: '2026-11-13'\n",
        encoding="utf-8",
    )
    data = _dual_scenario_dict()
    data["calibration"] = {"base_extrinsic": {"file": str(bad)}}
    with pytest.raises(ScenarioError) as excinfo:
        load_scenario_dict(data)
    message = str(excinfo.value)
    assert "method" in message
    assert str(bad) in message


def test_calibration_file_missing_robot_raises_instead_of_silent_fallback(tmp_path):
    """标定文件缺一台臂的外参 → 显式报错（静默回退会形成两个真相源）。"""
    tmp = tmp_path / "partial_calibration.yaml"
    tmp.write_text(
        "schema_version: 1\n"
        "measured: false\n"
        "calibrated: false\n"
        "method: demo\n"
        "date: '2026-11-13'\n"
        "base_extrinsic:\n"
        "  left:\n"
        "    xyz: [0.0, 0.0, 0.0]\n"
        "    rpy: [0.0, 0.0, 0.0]\n",
        encoding="utf-8",
    )
    data = _dual_scenario_dict()
    data["calibration"] = {"base_extrinsic": {"file": str(tmp)}}
    with pytest.raises(ScenarioError) as excinfo:
        load_scenario_dict(data)
    message = str(excinfo.value)
    assert "right" in message
    assert str(tmp) in message


def test_calibration_file_with_unknown_robot_raises(tmp_path):
    extra = tmp_path / "extra_robot.yaml"
    extra.write_text(
        "schema_version: 1\n"
        "measured: false\n"
        "calibrated: false\n"
        "method: demo\n"
        "date: '2026-11-13'\n"
        "base_extrinsic:\n"
        "  left:\n"
        "    xyz: [0.0, 0.0, 0.0]\n"
        "    rpy: [0.0, 0.0, 0.0]\n"
        "  right:\n"
        "    xyz: [0.6, 0.0, 0.0]\n"
        "    rpy: [0.0, 0.0, 0.0]\n"
        "  third_arm:\n"
        "    xyz: [1.2, 0.0, 0.0]\n"
        "    rpy: [0.0, 0.0, 0.0]\n",
        encoding="utf-8",
    )
    data = _dual_scenario_dict()
    data["calibration"] = {"base_extrinsic": {"file": str(extra)}}
    with pytest.raises(ScenarioError) as excinfo:
        load_scenario_dict(data)
    message = str(excinfo.value)
    assert "third_arm" in message
    assert str(extra) in message


def test_calibration_file_without_base_extrinsic_section_raises(tmp_path):
    empty = tmp_path / "no_extrinsic.yaml"
    empty.write_text(
        "schema_version: 1\n"
        "measured: false\n"
        "calibrated: false\n"
        "method: demo\n"
        "date: '2026-11-13'\n",
        encoding="utf-8",
    )
    data = _dual_scenario_dict()
    data["calibration"] = {"base_extrinsic": {"file": str(empty)}}
    with pytest.raises(ScenarioError) as excinfo:
        load_scenario_dict(data)
    message = str(excinfo.value)
    assert "base_extrinsic" in message
    assert str(empty) in message


def test_calibration_reference_must_be_mapping_with_file_key():
    data = _dual_scenario_dict()
    data["calibration"] = {"base_extrinsic": {"file": 123}}
    with pytest.raises(ScenarioError) as excinfo:
        load_scenario_dict(data)
    assert "base_extrinsic.file" in str(excinfo.value)

    data = _dual_scenario_dict()
    data["calibration"] = {"base_extrinsic": {"path": "dual_arm.yaml"}}
    with pytest.raises(ScenarioError) as excinfo:
        load_scenario_dict(data)
    assert "base_extrinsic" in str(excinfo.value)


def test_calibration_file_measured_false_but_calibrated_true_is_rejected(tmp_path):
    """一致性自洽：未实测不得声明已标定（错误在标定层点名，场景层转成 ScenarioError）。"""
    bad = tmp_path / "inconsistent.yaml"
    bad.write_text(
        "schema_version: 1\n"
        "measured: false\n"
        "calibrated: true\n"
        "method: demo\n"
        "date: '2026-11-13'\n",
        encoding="utf-8",
    )
    data = _dual_scenario_dict()
    data["calibration"] = {"base_extrinsic": {"file": str(bad)}}
    with pytest.raises(ScenarioError) as excinfo:
        load_scenario_dict(data)
    message = str(excinfo.value)
    assert "calibrated" in message
    assert str(bad) in message


# ── 回归：既有场景与 schema 契约零变化 ────────────────────────────────
def test_single_arm_scenario_still_loads_unchanged():
    scenario = load_scenario(SINGLE_SCENARIO_PATH)
    assert scenario.schema_version == scenario_mod.SCHEMA_VERSION
    assert "base_extrinsic" not in scenario.calibration
    assert scenario.base_extrinsics["rm75"].xyz == (0.0, 0.0, 0.0)
    assert scenario.calibration["gripper_force"]["file"].endswith("gripper_force.yaml")


def test_scenario_top_level_keys_and_version_not_changed():
    """顶层四键与 schema_version **不得**变化（会击穿 S07 golden-file 基线）。"""
    assert scenario_mod.TOP_LEVEL_KEYS == (
        "schema_version",
        "robots",
        "base_extrinsics",
        "obstacles",
        "calibration",
    )
    assert scenario_mod.SCHEMA_VERSION == 1


def test_existing_scenario_dict_behavior_unchanged():
    """既有场景文件（不含 base_extrinsic 键）行为完全不变。"""
    data = {
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
        "obstacles": [],
        "calibration": {},
    }
    scenario = load_scenario_dict(copy.deepcopy(data))
    assert scenario.calibration == {}
    assert scenario.base_extrinsics["rm75"].xyz == (0.0, 0.0, 0.0)


def test_dual_scenario_yaml_is_valid_yaml_with_reference():
    """scenarios/dual_arm.yaml 通过**引用**消费标定文件（不是内联外参）。"""
    with DUAL_SCENARIO_PATH.open("r", encoding="utf-8") as fh:
        raw = yaml.safe_load(fh)
    assert raw["calibration"]["base_extrinsic"]["file"] == "dual_arm.yaml"
    # 顶层四键不变
    assert set(raw) == {"schema_version", "robots", "base_extrinsics", "obstacles", "calibration"}
