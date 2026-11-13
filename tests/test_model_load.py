"""改造版 RM75-B 模型的加载冒烟测试（S01-T02）。

覆盖计划要求的断言：

* DoF = 7（weld 后）；未 weld = 14（对照）；
* 总质量 7.776 kg（±1e-6）；
* 零位 link_7 位置 z = 0.8505 m（±1e-6）、x ≈ 0、|y| < 1e-5
  （y 的 2.24e-6 偏差来自官方 URDF 的 rpy = 1.5708 舍入，不是模型错误）；
* 7 条关节位置限位与速度限位逐条等于官方参数表；
* 碰撞几何为 box，且模型加载**不依赖任何网格文件**；
* 相邻 link 对被碰撞过滤、非相邻对不被过滤（自碰撞过滤策略被锁死）。

每条正向断言都配负例（见文件末尾），杜绝“恒真绿灯”。
"""

from __future__ import annotations

import math
import os
import pathlib
import xml.etree.ElementTree as ET

import pytest

from rm75_drake import models
from rm75_drake import package_map

# ── 官方参数表（真值来源：官方开发者中心 RM75 参数表 + 评审记录 E9）──────────
POSITION_LIMITS = {
    "joint_1": (-3.106, 3.106),
    "joint_2": (-2.2689, 2.2689),
    "joint_3": (-3.106, 3.106),
    "joint_4": (-2.356, 2.356),
    "joint_5": (-3.106, 3.106),
    "joint_6": (-2.234, 2.234),
    "joint_7": (-6.283185307179586, 6.283185307179586),
}

VELOCITY_LIMITS = {
    "joint_1": 3.1416,
    "joint_2": 3.1416,
    "joint_3": 3.927,
    "joint_4": 3.927,
    "joint_5": 3.927,
    "joint_6": 3.927,
    "joint_7": 3.927,
}

EFFORT_LIMITS = {
    "joint_1": 60.0,
    "joint_2": 60.0,
    "joint_3": 30.0,
    "joint_4": 30.0,
    "joint_5": 10.0,
    "joint_6": 10.0,
    "joint_7": 10.0,
}

#: 官方 URDF 的原始速度限位（全部 3.14）——本切片的唯一数值修正项。
OFFICIAL_URDF_VELOCITY = 3.14

#: 零位末端高度（= d1 + d3 + d5 + d7 = 240.5 + 256 + 210 + 144 mm）。
ZERO_POSE_HEIGHT_M = 0.8505

#: 相邻 link 对（父子直接相连）：Drake 默认过滤。
ADJACENT_PAIRS = (
    ("base_link", "link_1"),
    ("link_1", "link_2"),
    ("link_2", "link_3"),
    ("link_3", "link_4"),
    ("link_4", "link_5"),
    ("link_5", "link_6"),
    ("link_6", "link_7"),
)


@pytest.fixture(scope="module")
def loaded():
    """装配好的（已 Finalize 的）plant 与 model instance 索引。"""
    plant, instance, _parser = models.load_rm75()
    plant.Finalize()
    return plant, instance


@pytest.fixture(scope="module")
def scene_graph_plant():
    """带 SceneGraph 的装配结果（供碰撞过滤查询用）。"""
    from pydrake.geometry import SceneGraph

    scene_graph = SceneGraph()
    plant, _instance, _parser = models.load_rm75(scene_graph=scene_graph)
    plant.Finalize()
    return scene_graph, plant


def _limit_mismatches(limits, expected_position=None, expected_velocity=None):
    """返回与官方表不一致的条目列表（空列表 = 全对）。

    抽成独立函数是为了让负例能复用同一条判定路径：断言"判定函数本身有鉴别力"。
    """
    by_name = {item["name"]: item for item in limits}
    mismatches = []
    for name, item in by_name.items():
        if expected_position is not None and name in expected_position:
            lower, upper = expected_position[name]
            if not math.isclose(item["lower"], lower, abs_tol=1e-12) or not math.isclose(
                item["upper"], upper, abs_tol=1e-12
            ):
                mismatches.append((name, "position", item["lower"], item["upper"]))
        if expected_velocity is not None and name in expected_velocity:
            if not math.isclose(item["velocity"], expected_velocity[name], abs_tol=1e-12):
                mismatches.append((name, "velocity", item["velocity"], None))
    return mismatches


# ── DoF ───────────────────────────────────────────────────────────────
def test_dof_is_7_after_weld(loaded):
    plant, _instance = loaded
    assert plant.num_positions() == 7


def test_dof_is_14_without_weld():
    """对照：不 weld 时基座自由（7 关节 + 7 自由基座 = 14）。"""
    plant, _instance, _parser = models.load_rm75(weld=False)
    plant.Finalize()
    assert plant.num_positions() == 14


# ── 质量 ───────────────────────────────────────────────────────────────
def test_total_mass_is_7_776_kg(loaded):
    plant, _instance = loaded
    context = plant.CreateDefaultContext()
    assert plant.CalcTotalMass(context) == pytest.approx(7.776, abs=1e-6)


# ── 零位 FK ────────────────────────────────────────────────────────────
def test_zero_pose_tip_position(loaded):
    plant, _instance = loaded
    context = plant.CreateDefaultContext()
    pose = models.eval_link_pose_in_world(plant, context, "link_7")
    translation = pose.translation()
    assert translation[2] == pytest.approx(ZERO_POSE_HEIGHT_M, abs=1e-6)
    assert translation[0] == pytest.approx(0.0, abs=1e-6)
    # y 的 2.24e-6 偏差来自官方 URDF 的 rpy = 1.5708 舍入（不是模型错误）。
    assert abs(translation[1]) < 1e-5


def test_zero_pose_base_link_at_origin(loaded):
    plant, _instance = loaded
    context = plant.CreateDefaultContext()
    pose = models.eval_link_pose_in_world(plant, context, "base_link")
    translation = pose.translation()
    assert translation[0] == pytest.approx(0.0, abs=1e-9)
    assert translation[1] == pytest.approx(0.0, abs=1e-9)
    assert translation[2] == pytest.approx(0.0, abs=1e-9)


# ── 限位 ───────────────────────────────────────────────────────────────
def test_joint_names_and_order(loaded):
    plant, _instance = loaded
    limits = models.joint_limits_from_plant(plant)
    assert [item["name"] for item in limits] == list(models.RM75_JOINT_NAMES)


def test_position_limits_match_official_table(loaded):
    plant, _instance = loaded
    limits = models.joint_limits_from_plant(plant)
    assert _limit_mismatches(limits, expected_position=POSITION_LIMITS) == []


def test_velocity_limits_match_official_table(loaded):
    plant, _instance = loaded
    limits = models.joint_limits_from_plant(plant)
    assert _limit_mismatches(limits, expected_velocity=VELOCITY_LIMITS) == []


def test_velocity_limits_are_not_the_official_urdf_value(loaded):
    """负例：官方 URDF 的一律 3.14 必须与官方参数表**不同**（否则修正没生效）。"""
    plant, _instance = loaded
    limits = models.joint_limits_from_plant(plant)
    for item in limits:
        assert item["velocity"] != pytest.approx(OFFICIAL_URDF_VELOCITY, abs=1e-9)
    # 用同一个判定函数跑官方 URDF 的值 → 必须判红（证明判定有鉴别力）。
    official_style = [
        {"name": name, "velocity": OFFICIAL_URDF_VELOCITY} for name in VELOCITY_LIMITS
    ]
    assert _limit_mismatches(official_style, expected_velocity=VELOCITY_LIMITS) != []


def test_effort_limits_read_from_urdf():
    limits = {item["name"]: item["effort"] for item in models.joint_limits_from_urdf()}
    assert limits == EFFORT_LIMITS


def test_urdf_declares_exactly_seven_joints():
    limits = models.joint_limits_from_urdf()
    assert len(limits) == 7
    assert all(item["type"] == "revolute" for item in limits)


# ── 碰撞几何：原始几何体 box，零外部资产依赖 ──────────────────────────
def test_urdf_has_no_mesh_and_no_visual():
    """负例导向：URDF 里一旦出现 <mesh> 或 <visual>，模型就不再零外部资产依赖。"""
    pkg_map = package_map.build_package_map()
    path = package_map.resolve_package_url(pkg_map, models.RM75_URDF_URL)
    text = pathlib.Path(path).read_text(encoding="utf-8")
    root = ET.fromstring(text)
    assert root.findall(".//mesh") == []
    assert root.findall(".//visual") == []
    # 任何网格引用都会带 filename 属性（这里逐元素断言，注释里的文字不算）。
    for element in root.iter():
        assert "filename" not in element.attrib


def test_collision_geometry_is_box(scene_graph_plant):
    scene_graph, _plant = scene_graph_plant
    inspector = scene_graph.model_inspector()
    names = sorted(inspector.GetName(gid) for gid in inspector.GetAllGeometryIds())
    assert names == [f"RM75-B::{link}_collision" for link in models.RM75_LINK_NAMES]
    for gid in inspector.GetAllGeometryIds():
        assert type(inspector.GetShape(gid)).__name__ == "Box"


def test_model_loads_without_upstream_assets(monkeypatch):
    """模型本体不得依赖上游归档 / 网格目录（把搜索路径指向空目录也应失败在解析前）。"""
    from rm75_drake import paths

    monkeypatch.delenv(paths.MODEL_PATH_ENV_VAR, raising=False)
    plant, _instance, _parser = models.load_rm75()
    plant.Finalize()
    assert plant.num_positions() == 7


def test_load_is_independent_of_cwd(monkeypatch, tmp_path):
    """路径不依赖 cwd：换工作目录后仍能加载。"""
    monkeypatch.chdir(tmp_path)
    plant, _instance, _parser = models.load_rm75()
    plant.Finalize()
    assert plant.num_positions() == 7


# ── 自碰撞过滤 ─────────────────────────────────────────────────────────
def _geometry_ids(inspector):
    return {
        inspector.GetName(gid).split("::")[-1]: gid
        for gid in inspector.GetAllGeometryIds()
    }


def test_adjacent_pairs_are_collision_filtered(scene_graph_plant):
    scene_graph, _plant = scene_graph_plant
    inspector = scene_graph.model_inspector()
    ids = _geometry_ids(inspector)
    for a, b in ADJACENT_PAIRS:
        assert inspector.CollisionFiltered(ids[f"{a}_collision"], ids[f"{b}_collision"])


def test_non_adjacent_pairs_are_not_filtered(scene_graph_plant):
    scene_graph, _plant = scene_graph_plant
    inspector = scene_graph.model_inspector()
    ids = _geometry_ids(inspector)
    for a, b in (("base_link", "link_3"), ("link_1", "link_3"), ("link_1", "link_5")):
        assert not inspector.CollisionFiltered(ids[f"{a}_collision"], ids[f"{b}_collision"])


# ── 负例：错误路径必须显式报错 ─────────────────────────────────────────
def test_unknown_link_name_raises(loaded):
    plant, _instance = loaded
    context = plant.CreateDefaultContext()
    with pytest.raises(Exception):
        models.eval_link_pose_in_world(plant, context, "link_42")


def test_missing_model_file_raises(monkeypatch, tmp_path):
    """包名写错 / 模型缺失时必须显式抛错，不能静默绿灯。"""
    from rm75_drake import paths

    monkeypatch.setenv(paths.MODEL_PATH_ENV_VAR, str(tmp_path))
    with pytest.raises(Exception):
        models.load_rm75()


def test_gripper_loader_loads_after_t04():
    """S01-T04 起夹爪 URDF 已落地 → 装配层必须能正常加载（2 个 prismatic DoF）。"""
    plant, _instance, _parser = models.load_rm75_gripper()
    plant.Finalize()
    assert plant.num_positions() == 2
    assert plant.HasFrameNamed(models.GRIPPER_BASE_LINK)
    assert plant.HasFrameNamed(models.GRIPPER_TCP_LINK)


def test_models_module_has_no_hardcoded_absolute_paths():
    """库内不得硬编码绝对路径（路径一律走 rm75_drake.paths）。"""
    import inspect as _inspect

    source = _inspect.getsource(models)
    repo_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(models.__file__))))
    assert repo_root not in source
    assert "/mnt/" not in source and "/home/" not in source
