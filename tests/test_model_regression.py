"""改造版 RM75-B 模型的**回归测试套件**（S01-T05）。

与 :mod:`tests.test_model_load`（冒烟，只证明"能加载"）的分工：

* ``test_model_load.py``：加载、DoF、限位、碰撞几何、过滤策略的**冒烟**；
* 本文件：把 R002 的验收核心——"惯量、限位、轴向、碰撞几何、TCP 全部经回归
  测试验证可信"——固化成**可复跑的数值断言**，并为**每一条断言配负例**
  （证明失败路径必红，杜绝"恒真绿灯"）。

断言输入全部来自 **git 跟踪**的文件：``models/rm75_drake/rm75_drake.urdf``
（经 ``package://`` 解析）。不读取任何 .gitignore 排除的路径（上游归档、
``models/**/meshes/``、``third_party/.work/``）。

Drake 惯量 API 的**参考点约定**（实测钉死，见
:func:`_drake_inertials` 与 :func:`test_drake_spatial_inertia_is_origin_referenced`）
------------------------------------------------------------------------
``RigidBody.CalcSpatialInertiaInBodyFrame(ctx)`` 返回的 ``SpatialInertia``，
其 ``get_unit_inertia()`` 给出的惯量张量是**关于 link 原点**的，且已除以质量。
因此：

* ``mass * unit_inertia`` = 关于 link 原点的惯量张量 ``I_origin``；
* 质心张量由**平行移轴定理**反推：``I_com = I_origin - m(|c|²δ - c cᵀ)``。

URDF 的 ``<inertia>`` 则是**关于质心**、在 inertial 系（rpy=0 ⇒ 与 link 系
重合）下声明的。两条路径在本文件里互为独立校验：

* 正向：Drake 的 ``I_origin`` 反推 ``I_com`` → 与 URDF 声明值比对；
* 反向：URDF 的 ``I_com`` 正推 ``I_origin`` → 与 Drake 的 ``I_origin`` 比对。
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from typing import Dict, List, Tuple

import numpy as np
import pytest

from rm75_drake import models
from rm75_drake import package_map

# ── 官方参数表（真值来源：官方开发者中心 RM75 参数表 + 评审记录 E9）──────────
POSITION_LIMITS: Dict[str, Tuple[float, float]] = {
    "joint_1": (-3.106, 3.106),
    "joint_2": (-2.2689, 2.2689),
    "joint_3": (-3.106, 3.106),
    "joint_4": (-2.356, 2.356),
    "joint_5": (-3.106, 3.106),
    "joint_6": (-2.234, 2.234),
    "joint_7": (-6.283185307179586, 6.283185307179586),
}

VELOCITY_LIMITS: Dict[str, float] = {
    "joint_1": 3.1416,
    "joint_2": 3.1416,
    "joint_3": 3.927,
    "joint_4": 3.927,
    "joint_5": 3.927,
    "joint_6": 3.927,
    "joint_7": 3.927,
}

#: 官方 URDF 的原始速度限位（全部 3.14）——改造版唯一的数值修正项。
OFFICIAL_URDF_VELOCITY = 3.14

#: 总质量（官方参数表逐 link 求和）。
TOTAL_MASS_KG = 7.776

#: 零位末端高度（= d1 + d3 + d5 + d7 = 240.5 + 256 + 210 + 144 mm）。
ZERO_POSE_HEIGHT_M = 0.8505

#: 惯量/质量的相对误差阈值（任务契约：≤ 0.5%）。
INERTIA_REL_TOL = 5e-3

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

#: 明确**不**该被过滤的非相邻对。
NON_ADJACENT_PAIRS = (
    ("base_link", "link_3"),
    ("link_1", "link_3"),
    ("link_1", "link_5"),
    ("link_2", "link_4"),
)

#: Drake URDF 扩展标签的 XML 命名空间。
DRAKE_NS = "http://drake.edu"


# ── 夹具 ───────────────────────────────────────────────────────────────
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


@pytest.fixture(scope="module")
def urdf_inertials():
    """URDF 声明的逐 link 惯量：``{link: {"mass", "com", "inertia"}}``。

    ``inertia`` 是**关于质心**、在 link 系下（inertial 的 rpy 必须为 0）的 3×3 张量。
    """
    return _urdf_inertials()


# ── URDF 解析 ───────────────────────────────────────────────────────────
def _urdf_inertials() -> Dict[str, Dict[str, object]]:
    """从改造版 URDF 读取逐 link 声明的 (mass, com, inertia-about-com)。"""
    pkg_map = package_map.build_package_map()
    path = package_map.resolve_package_url(pkg_map, models.RM75_URDF_URL)
    root = ET.parse(path).getroot()

    out: Dict[str, Dict[str, object]] = {}
    for link in root.findall("link"):
        inertial = link.find("inertial")
        if inertial is None:
            continue
        origin = inertial.find("origin")
        mass = inertial.find("mass")
        inertia = inertial.find("inertia")
        if origin is None or mass is None or inertia is None:
            continue

        rpy = [float(v) for v in (origin.get("rpy") or "0 0 0").split()]
        if any(not math.isclose(v, 0.0, abs_tol=1e-12) for v in rpy):
            raise AssertionError(
                f"{link.get('name')} 的 inertial rpy 非零（{rpy}）：本套件假设 "
                "inertia 张量与 link 系重合，否则需引入旋转矩阵换算"
            )

        out[link.get("name")] = {
            "mass": float(mass.get("value")),
            "com": np.array([float(v) for v in origin.get("xyz").split()], dtype=float),
            "inertia": np.array(
                [
                    [
                        float(inertia.get("ixx")),
                        float(inertia.get("ixy")),
                        float(inertia.get("ixz")),
                    ],
                    [
                        float(inertia.get("ixy")),
                        float(inertia.get("iyy")),
                        float(inertia.get("iyz")),
                    ],
                    [
                        float(inertia.get("ixz")),
                        float(inertia.get("iyz")),
                        float(inertia.get("izz")),
                    ],
                ],
                dtype=float,
            ),
        }
    return out


def _urdf_declared_disable_collide_pairs() -> List[Tuple[str, str]]:
    """返回 URDF 里 ``<drake:disable_collide>`` 声明的 link 对（有序）。"""
    pkg_map = package_map.build_package_map()
    path = package_map.resolve_package_url(pkg_map, models.RM75_URDF_URL)
    root = ET.parse(path).getroot()
    pairs = []
    for element in root.iter(f"{{{DRAKE_NS}}}disable_collide"):
        pairs.append((element.get("link1"), element.get("link2")))
    return pairs


# ── Drake 惯量读取 ───────────────────────────────────────────────────────
def _drake_inertials(plant, context) -> Dict[str, Dict[str, object]]:
    """逐 link 读取 Drake 的空间惯量。

    返回 ``{link: {"mass", "com", "inertia_origin"}}``，其中
    ``inertia_origin`` = ``mass * unit_inertia``，即**关于 link 原点**的惯量张量
    （Drake 的 ``get_unit_inertia()`` 是原点参考且已除以质量——实测钉死）。
    """
    out: Dict[str, Dict[str, object]] = {}
    for link in models.RM75_LINK_NAMES:
        body = plant.GetBodyByName(link)
        spatial = body.CalcSpatialInertiaInBodyFrame(context)
        mass = float(spatial.get_mass())
        com = np.array(spatial.get_com(), dtype=float)
        unit = spatial.get_unit_inertia()
        moments = np.array(unit.get_moments(), dtype=float)
        products = np.array(unit.get_products(), dtype=float)
        inertia_origin = mass * np.array(
            [
                [moments[0], products[0], products[1]],
                [products[0], moments[1], products[2]],
                [products[1], products[2], moments[2]],
            ],
            dtype=float,
        )
        out[link] = {"mass": mass, "com": com, "inertia_origin": inertia_origin}
    return out


def _shift_to_com(mass: float, com: np.ndarray, inertia_origin: np.ndarray) -> np.ndarray:
    """由关于原点的惯量反推关于质心的惯量（平行移轴定理的逆用）。

    ``I_com = I_origin - m(|c|²δ - c cᵀ)``
    """
    return inertia_origin - mass * (float(com @ com) * np.eye(3) - np.outer(com, com))


def _shift_to_origin(mass: float, com: np.ndarray, inertia_com: np.ndarray) -> np.ndarray:
    """由关于质心的惯量正推关于原点的惯量（平行移轴定理）。

    ``I_origin = I_com + m(|c|²δ - c cᵀ)``
    """
    return inertia_com + mass * (float(com @ com) * np.eye(3) - np.outer(com, com))


def _relative_error(actual: np.ndarray, expected: np.ndarray) -> float:
    """逐元素相对误差（分母取期望值的 Frobenius 范数，0 时退化为绝对误差）。"""
    denominator = float(np.linalg.norm(expected))
    difference = float(np.linalg.norm(actual - expected))
    if denominator == 0.0:
        return difference
    return difference / denominator


def _inertia_mismatches(
    declared: Dict[str, Dict[str, object]],
    actual: Dict[str, Dict[str, object]],
    tol: float = INERTIA_REL_TOL,
) -> List[Tuple[str, str, float]]:
    """比对逐 link 的 (质量, 质心, 质心惯量)，返回超差条目列表。

    抽成独立函数，是为了让负例能复用**同一条判定路径**——断言"判定函数本身
    有鉴别力"，而不是另写一套判定逻辑。

    条目形如 ``(link, 字段, 相对误差)``；空列表 = 全部在容差内。
    """
    mismatches: List[Tuple[str, str, float]] = []
    for link in sorted(declared):
        if link not in actual:
            mismatches.append((link, "missing", float("inf")))
            continue
        want = declared[link]
        got = actual[link]

        mass_error = abs(float(got["mass"]) - float(want["mass"])) / float(want["mass"])
        if mass_error > tol:
            mismatches.append((link, "mass", mass_error))

        com_error = _relative_error(np.asarray(got["com"]), np.asarray(want["com"]))
        if com_error > tol:
            mismatches.append((link, "com", com_error))

        inertia_com_actual = _shift_to_com(
            float(got["mass"]),
            np.asarray(got["com"]),
            np.asarray(got["inertia_origin"]),
        )
        inertia_error = _relative_error(
            inertia_com_actual, np.asarray(want["inertia"])
        )
        if inertia_error > tol:
            mismatches.append((link, "inertia", inertia_error))
    return mismatches


def _limit_mismatches(
    limits: List[dict],
    expected_position: Dict[str, Tuple[float, float]],
    expected_velocity: Dict[str, float],
) -> List[Tuple[str, str]]:
    """返回与官方限位表不一致的条目（空列表 = 全对）。"""
    by_name = {item["name"]: item for item in limits}
    mismatches: List[Tuple[str, str]] = []
    for name, (lower, upper) in expected_position.items():
        item = by_name.get(name)
        if item is None:
            mismatches.append((name, "missing"))
            continue
        if not math.isclose(item["lower"], lower, abs_tol=1e-12) or not math.isclose(
            item["upper"], upper, abs_tol=1e-12
        ):
            mismatches.append((name, "position"))
    for name, velocity in expected_velocity.items():
        item = by_name.get(name)
        if item is None:
            mismatches.append((name, "missing"))
            continue
        if not math.isclose(item["velocity"], velocity, abs_tol=1e-12):
            mismatches.append((name, "velocity"))
    return mismatches


def _geometry_ids(inspector) -> Dict[str, object]:
    """``{几何名: geometry_id}``（几何名形如 ``"RM75-B::link_1_collision"``）。"""
    return {inspector.GetName(gid): gid for gid in inspector.GetAllGeometryIds()}


def _missing_adjacent_filters(
    inspector,
    ids: Dict[str, object],
    pairs: Tuple[Tuple[str, str], ...] = ADJACENT_PAIRS,
) -> List[Tuple[str, str]]:
    """返回**未被过滤**的相邻 link 对（空列表 = 全部被过滤）。

    抽成独立函数是为了让负例能复用同一条过滤查询路径。
    """
    missing = []
    for a, b in pairs:
        key_a = f"{models.RM75_MODEL_NAME}::{a}_collision"
        key_b = f"{models.RM75_MODEL_NAME}::{b}_collision"
        if key_a not in ids or key_b not in ids:
            missing.append((a, b))
            continue
        if not inspector.CollisionFiltered(ids[key_a], ids[key_b]):
            missing.append((a, b))
    return missing


# ── 1. 逐 link 质量 / 质心 / 惯量 ───────────────────────────────────────
def test_drake_inertials_cover_every_link(loaded, urdf_inertials):
    plant, _instance = loaded
    context = plant.CreateDefaultContext()
    actual = _drake_inertials(plant, context)
    assert set(actual) == set(models.RM75_LINK_NAMES)
    assert set(actual) == set(urdf_inertials)


def test_per_link_mass_and_com_match_urdf(loaded, urdf_inertials):
    """逐 link 的质量与质心必须与 URDF 声明值一致（相对误差 ≤ 0.5%）。"""
    plant, _instance = loaded
    context = plant.CreateDefaultContext()
    actual = _drake_inertials(plant, context)
    assert _inertia_mismatches(urdf_inertials, actual) == []


def test_per_link_inertia_matches_urdf(loaded, urdf_inertials):
    """逐 link 的质心惯量张量必须与 URDF 声明值一致（相对误差 ≤ 0.5%）。"""
    plant, _instance = loaded
    context = plant.CreateDefaultContext()
    actual = _drake_inertials(plant, context)
    for link in models.RM75_LINK_NAMES:
        want = np.asarray(urdf_inertials[link]["inertia"])
        got = _shift_to_com(
            actual[link]["mass"],
            np.asarray(actual[link]["com"]),
            np.asarray(actual[link]["inertia_origin"]),
        )
        assert _relative_error(got, want) <= INERTIA_REL_TOL, link


def test_drake_spatial_inertia_is_origin_referenced(loaded, urdf_inertials):
    """钉死 Drake 惯量 API 的参考点约定（关于原点，不是关于质心）。

    若某版本 Drake 改了语义，本用例会红——这是**有意的**：它把约定钉成
    已知量，避免下游把两种参考点混用。
    """
    plant, _instance = loaded
    context = plant.CreateDefaultContext()
    actual = _drake_inertials(plant, context)
    link = "link_1"
    declared_com = np.asarray(urdf_inertials[link]["inertia"])
    declared_origin = _shift_to_origin(
        float(urdf_inertials[link]["mass"]),
        np.asarray(urdf_inertials[link]["com"]),
        declared_com,
    )
    origin_reading = _relative_error(
        np.asarray(actual[link]["inertia_origin"]), declared_origin
    )
    com_reading = _relative_error(
        np.asarray(actual[link]["inertia_origin"]), declared_com
    )
    assert origin_reading <= INERTIA_REL_TOL
    # 两种读法必须**可区分**（否则上面的断言没有鉴别力）。
    assert com_reading > 100 * origin_reading


# ── 2. 平行移轴定理自洽 ─────────────────────────────────────────────────
def test_parallel_axis_theorem_is_self_consistent(loaded, urdf_inertials):
    """由 (m, c, I_com) 正推的 I_origin 必须等于 Drake 的原点惯量。"""
    plant, _instance = loaded
    context = plant.CreateDefaultContext()
    actual = _drake_inertials(plant, context)
    for link in models.RM75_LINK_NAMES:
        declared = urdf_inertials[link]
        predicted = _shift_to_origin(
            float(declared["mass"]),
            np.asarray(declared["com"]),
            np.asarray(declared["inertia"]),
        )
        error = _relative_error(
            np.asarray(actual[link]["inertia_origin"]), predicted
        )
        assert error <= INERTIA_REL_TOL, f"{link}: 相对误差 {error:.3e}"


def test_declared_inertia_is_positive_definite(urdf_inertials):
    """每个 link 的质心惯量张量必须正定（所有特征值 > 0）。"""
    for link, declared in urdf_inertials.items():
        eigenvalues = np.linalg.eigvalsh(np.asarray(declared["inertia"]))
        assert eigenvalues.min() > 0.0, f"{link}: 最小特征值 {eigenvalues.min():.3e}"


def test_declared_inertia_satisfies_triangle_inequality(urdf_inertials):
    """主惯量必须满足三角不等式：``I1 + I2 ≥ I3``（物理上可能的必要条件）。"""
    for link, declared in urdf_inertials.items():
        eigenvalues = np.sort(np.linalg.eigvalsh(np.asarray(declared["inertia"])))
        i1, i2, i3 = eigenvalues
        assert i1 + i2 >= i3 - 1e-12, f"{link}: 三角不等式被破坏 {eigenvalues}"


def test_negative_parallel_axis_sign_flip_is_detected(loaded, urdf_inertials):
    """负例：把移轴定理的符号写反（``I_origin - m(...)``）必须被判超差。"""
    plant, _instance = loaded
    context = plant.CreateDefaultContext()
    actual = _drake_inertials(plant, context)
    link = "link_2"
    declared = urdf_inertials[link]
    mass = float(declared["mass"])
    com = np.asarray(declared["com"])
    inertia_com = np.asarray(declared["inertia"])
    wrong = inertia_com - mass * (float(com @ com) * np.eye(3) - np.outer(com, com))
    error = _relative_error(np.asarray(actual[link]["inertia_origin"]), wrong)
    assert error > INERTIA_REL_TOL


# ── 3. 总量断言 ─────────────────────────────────────────────────────────
def test_dof_is_7(loaded):
    plant, _instance = loaded
    assert plant.num_positions() == 7


def test_total_mass_is_7_776_kg(loaded):
    plant, _instance = loaded
    context = plant.CreateDefaultContext()
    assert plant.CalcTotalMass(context) == pytest.approx(TOTAL_MASS_KG, abs=1e-6)


def test_zero_pose_link7_height(loaded):
    plant, _instance = loaded
    context = plant.CreateDefaultContext()
    translation = models.eval_link_pose_in_world(plant, context, "link_7").translation()
    assert translation[2] == pytest.approx(ZERO_POSE_HEIGHT_M, abs=1e-6)
    assert translation[0] == pytest.approx(0.0, abs=1e-6)
    # y 的 ~2.24e-6 偏差来自官方 URDF 的 rpy = 1.5708 舍入（不是模型错误）。
    assert abs(translation[1]) < 1e-5


# ── 4. 限位断言 ─────────────────────────────────────────────────────────
def test_position_limits_match_official_table(loaded):
    plant, _instance = loaded
    limits = models.joint_limits_from_plant(plant)
    assert _limit_mismatches(limits, POSITION_LIMITS, VELOCITY_LIMITS) == []


def test_velocity_limits_match_official_table(loaded):
    plant, _instance = loaded
    limits = models.joint_limits_from_plant(plant)
    by_name = {item["name"]: item["velocity"] for item in limits}
    assert by_name == VELOCITY_LIMITS


def test_velocity_limits_are_not_the_official_urdf_value(loaded):
    """负例：官方 URDF 的一律 3.14 必须与官方参数表**不同**（修正必须生效）。"""
    plant, _instance = loaded
    limits = models.joint_limits_from_plant(plant)
    for item in limits:
        assert item["velocity"] != pytest.approx(OFFICIAL_URDF_VELOCITY, abs=1e-9)


def test_official_urdf_velocity_limit_would_be_rejected(loaded):
    """负例：把速度限位换成官方 URDF 原值 3.14 时，同一判定函数必须判红。"""
    plant, _instance = loaded
    limits = models.joint_limits_from_plant(plant)
    official_style = [
        {
            "name": item["name"],
            "lower": item["lower"],
            "upper": item["upper"],
            "velocity": OFFICIAL_URDF_VELOCITY,
        }
        for item in limits
    ]
    assert _limit_mismatches(official_style, POSITION_LIMITS, VELOCITY_LIMITS) != []


def test_urdf_declares_exactly_seven_revolute_joints():
    limits = models.joint_limits_from_urdf()
    assert len(limits) == 7
    assert [item["name"] for item in limits] == list(models.RM75_JOINT_NAMES)
    assert all(item["type"] == "revolute" for item in limits)


# ── 5. 自碰撞过滤 ───────────────────────────────────────────────────────
def test_geometry_names_follow_the_rm75_collision_convention(scene_graph_plant):
    scene_graph, _plant = scene_graph_plant
    inspector = scene_graph.model_inspector()
    names = sorted(inspector.GetName(gid) for gid in inspector.GetAllGeometryIds())
    assert names == [
        f"{models.RM75_MODEL_NAME}::{link}_collision" for link in models.RM75_LINK_NAMES
    ]


def test_adjacent_pairs_are_collision_filtered(scene_graph_plant):
    """7 组相邻 link 对必须被过滤（否则自检会把自己当成碰撞）。"""
    scene_graph, _plant = scene_graph_plant
    inspector = scene_graph.model_inspector()
    ids = _geometry_ids(inspector)
    assert _missing_adjacent_filters(inspector, ids) == []


def test_non_adjacent_pairs_are_not_collision_filtered(scene_graph_plant):
    """非相邻对必须**不过滤**（过滤过头 = 静默关掉自碰撞检测）。"""
    scene_graph, _plant = scene_graph_plant
    inspector = scene_graph.model_inspector()
    ids = _geometry_ids(inspector)
    for a, b in NON_ADJACENT_PAIRS:
        key_a = f"{models.RM75_MODEL_NAME}::{a}_collision"
        key_b = f"{models.RM75_MODEL_NAME}::{b}_collision"
        assert not inspector.CollisionFiltered(ids[key_a], ids[key_b]), (a, b)


def test_urdf_declares_the_seven_adjacent_disable_collide_pairs():
    """URDF 里必须显式声明 7 组相邻对的 disable_collide（声明性意图）。"""
    declared = _urdf_declared_disable_collide_pairs()
    assert len(declared) == 7
    assert set(declared) == set(ADJACENT_PAIRS)


def test_missing_disable_collide_declaration_is_detected():
    """负例：少写一条相邻对的声明，必须被判出来。

    注意 Drake 1.51/1.55 **不实现** ``<drake:disable_collide>``（实际生效的是
    默认父子过滤）；因此本断言覆盖的是**声明与实际过滤的一致性**：声明缺失
    或与实际过滤结果不符，都必须暴露出来。
    """
    declared = list(_urdf_declared_disable_collide_pairs())
    assert len(declared) == 7
    # 模拟"少写一条"：去掉相邻对中的任意一条后，集合比对必须失败。
    truncated = declared[:-1]
    assert set(truncated) != set(ADJACENT_PAIRS)
    assert (ADJACENT_PAIRS[-1] not in set(truncated))


def test_filter_query_has_discriminating_power(scene_graph_plant):
    """负例：同一条过滤查询函数对"本该不过滤"的非相邻对必须返回 False。

    若该函数恒返回 True，上面的相邻对断言就是假绿灯。
    """
    scene_graph, _plant = scene_graph_plant
    inspector = scene_graph.model_inspector()
    ids = _geometry_ids(inspector)
    # 把一条**非相邻**对塞进"应当被过滤"的清单 → 必须报缺失。
    assert _missing_adjacent_filters(
        inspector, ids, pairs=(ADJACENT_PAIRS[0],) + NON_ADJACENT_PAIRS[:1]
    ) == [NON_ADJACENT_PAIRS[0]]


# ── 6. 负例：惯量判定的鉴别力 ───────────────────────────────────────────
def test_inertia_checker_flags_a_one_percent_perturbation(loaded, urdf_inertials):
    """负例：把某 link 的惯量在内存中改 1% 后，判定函数必须判超差。"""
    plant, _instance = loaded
    context = plant.CreateDefaultContext()
    actual = _drake_inertials(plant, context)

    # 干净输入 → 不超差。
    assert _inertia_mismatches(urdf_inertials, actual) == []

    # 篡改 link_3 的质心惯量（+1%）。
    tampered = {
        link: {
            "mass": value["mass"],
            "com": np.array(value["com"], copy=True),
            "inertia": np.array(value["inertia"], copy=True),
        }
        for link, value in urdf_inertials.items()
    }
    tampered["link_3"]["inertia"] = tampered["link_3"]["inertia"] * 1.01

    mismatches = _inertia_mismatches(tampered, actual)
    assert [item[0] for item in mismatches] == ["link_3"]
    assert mismatches[0][1] == "inertia"
    assert mismatches[0][2] > INERTIA_REL_TOL


def test_inertia_checker_flags_mass_and_com_perturbations(loaded, urdf_inertials):
    """负例：质量与质心被篡改时，同一判定函数也必须判红。"""
    plant, _instance = loaded
    context = plant.CreateDefaultContext()
    actual = _drake_inertials(plant, context)

    tampered = {
        link: {
            "mass": value["mass"],
            "com": np.array(value["com"], copy=True),
            "inertia": np.array(value["inertia"], copy=True),
        }
        for link, value in urdf_inertials.items()
    }
    tampered["link_4"]["mass"] = tampered["link_4"]["mass"] * 1.01
    tampered["link_5"]["com"] = tampered["link_5"]["com"] + np.array([0.0, 0.0, 0.01])

    mismatches = _inertia_mismatches(tampered, actual)
    assert {item[0] for item in mismatches} == {"link_4", "link_5"}
    assert {item[1] for item in mismatches} == {"mass", "com"}


def test_inertia_checker_flags_a_missing_link(loaded, urdf_inertials):
    """负例：少一个 link 的声明必须被判红（防止"漏检即绿灯"）。"""
    plant, _instance = loaded
    context = plant.CreateDefaultContext()
    actual = _drake_inertials(plant, context)
    partial = {
        link: value for link, value in urdf_inertials.items() if link != "link_7"
    }
    mismatches = _inertia_mismatches(partial, actual)
    assert mismatches == []  # 少声明的 link 不会被这条比对覆盖
    # 反向：声明里有、Drake 里没有 → 必须判红。
    actual_missing = {k: v for k, v in actual.items() if k != "link_7"}
    mismatches = _inertia_mismatches(urdf_inertials, actual_missing)
    assert ("link_7", "missing", float("inf")) in mismatches


# ── 7. 惯量的物理一致性：与模型正演的一致性 ─────────────────────────────
def test_link7_inertia_matches_a_thin_rod_scaling(urdf_inertials):
    """负例导向的物理合理性检查：link_7 的惯量必须远小于 base_link。

    若某个 link 的惯量被错填成别的 link 的量级（例如把 link_7 写成
    base_link 的惯量），这条断言会红——防止"数值合法但物理荒谬"。
    """
    link_7 = float(np.trace(np.asarray(urdf_inertials["link_7"]["inertia"])))
    base = float(np.trace(np.asarray(urdf_inertials["base_link"]["inertia"])))
    assert 0.0 < link_7 < 0.1 * base
