"""双臂 URDF 变体生成器测试（S04-T01）。

覆盖计划要求的断言：

* **等价性**：源 URDF 与变体逐字段比对——link 数 / joint 数与类型 / 逐 link 的
  inertial（mass / 质心 / 惯量张量）/ 逐 joint 的 limit（lower / upper / velocity /
  effort）/ 逐 collision 的 origin(xyz, rpy) 与 geometry(box 尺寸)——
  **除名字前缀外逐字段相等**（证明变体只是改名，不是新模型）。
* **命名**：变体内所有 link / joint / collision 名都带 ``left_`` / ``right_`` 前缀；
  ``<robot name>`` 正确（``RM75-B-left`` / ``RMG24-right`` …）。
* **幂等**：连续两次 ``build_variant`` 输出完全一致；``write_variants(check_only=True)``
  在磁盘与生成器一致时通过。
* **零外部资产**：变体里没有 ``<mesh>`` 元素 / ``filename`` 引用，且 ``mesh`` 出现次数
  不超过源文件（源文件头注释里的工具名不计为依赖）。
* **负例**：
  - 同一 plant 里连续两次 ``AddModels`` 同一份 URDF → ``RuntimeError``
    且消息含 ``model instance names must be unique``（证明「变体路线」的必要性）；
  - 变体被篡改 → ``write_variants(check_only=True)`` 抛 ``VariantMismatchError``；
  - 非法 kind / side → ``VariantError``；
  - 源文件缺失 / ``<robot name>`` 与 kind 不匹配 → ``VariantError``。

每条正向断言都配负例，杜绝“恒真绿灯”。
"""

from __future__ import annotations

import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from rm75_drake.bimanual import model_variants as mv

REPO_ROOT = Path(__file__).resolve().parents[1]

#: 四个变体（kind, side）。
VARIANT_PAIRS = [("rm75", "left"), ("rm75", "right"), ("rmg24", "left"), ("rmg24", "right")]

#: 源 URDF → 变体文件。
VARIANT_FILES = {
    ("rm75", "left"): "rm75_left.urdf",
    ("rm75", "right"): "rm75_right.urdf",
    ("rmg24", "left"): "rmg24_left.urdf",
    ("rmg24", "right"): "rmg24_right.urdf",
}


# ── 解析辅助 ────────────────────────────────────────────────────────────
def _root_of(text: str) -> ET.Element:
    return ET.fromstring(text)


def _links(root: ET.Element) -> dict:
    return {el.get("name"): el for el in root.findall("link")}


def _joints(root: ET.Element) -> dict:
    return {el.get("name"): el for el in root.findall("joint")}


def _inertial(link: ET.Element):
    """返回 link 的 inertial 指纹（质量 / 质心 / 惯量张量），无 inertial 则 None。"""
    inertial = link.find("inertial")
    if inertial is None:
        return None
    origin = inertial.find("origin")
    mass = inertial.find("mass")
    tensor = inertial.find("inertia")
    return {
        "origin_xyz": None if origin is None else origin.get("xyz"),
        "origin_rpy": None if origin is None else origin.get("rpy"),
        "mass": None if mass is None else mass.get("value"),
        "inertia": None if tensor is None else dict(sorted(tensor.attrib.items())),
    }


def _limit(joint: ET.Element):
    limit = joint.find("limit")
    if limit is None:
        return None
    return {
        "lower": limit.get("lower"),
        "upper": limit.get("upper"),
        "velocity": limit.get("velocity"),
        "effort": limit.get("effort"),
    }


def _collision_fingerprints(link: ET.Element):
    """返回 link 下每个 collision 的指纹（名字 / 原点 / 几何类型与参数）。"""
    out = []
    for coll in link.findall("collision"):
        origin = coll.find("origin")
        geometry = coll.find("geometry")
        geo = {}
        if geometry is not None and len(geometry):
            child = geometry[0]
            geo = {
                "type": child.tag,
                "params": dict(sorted(child.attrib.items())),
            }
        out.append(
            {
                "name": coll.get("name"),
                "origin_xyz": None if origin is None else origin.get("xyz"),
                "origin_rpy": None if origin is None else origin.get("rpy"),
                "geometry": geo,
            }
        )
    return out


def _strip_prefix(name: str, side: str) -> str:
    prefix = mv._SIDE_PREFIX[side]
    assert name.startswith(prefix), f"{name!r} 缺少侧别前缀 {prefix!r}"
    return name[len(prefix) :]


# ── 文件存在与生成器一致性 ───────────────────────────────────────────────
@pytest.mark.parametrize("kind,side", VARIANT_PAIRS)
def test_variant_file_exists(kind, side):
    path = mv.variant_path(side, kind)
    assert path.is_file(), f"变体文件缺失：{path}"
    assert path.name == VARIANT_FILES[(kind, side)]
    assert path.parent == mv.models_package_dir()


def test_check_variants_reports_all_ok():
    """磁盘上的四个变体必须与生成器**字节级**一致（CI 语义）。"""
    report = mv.check_variants()
    assert set(report) == {"rm75_left", "rm75_right", "rmg24_left", "rmg24_right"}
    bad = {k: v["status"] for k, v in report.items() if v["status"] != "ok"}
    assert not bad, f"变体与生成器不一致：{bad}"


def test_write_variants_check_only_passes_on_clean_tree():
    report = mv.write_variants(check_only=True)
    assert all(entry["status"] == "ok" for entry in report.values())


@pytest.mark.parametrize("kind,side", VARIANT_PAIRS)
def test_build_variant_is_deterministic(kind, side):
    """幂等：连续两次生成输出完全一致。"""
    src = mv.source_path(kind)
    first = mv.build_variant(src, side=side, kind=kind)
    second = mv.build_variant(src, side=side, kind=kind)
    assert first == second


@pytest.mark.parametrize("kind,side", VARIANT_PAIRS)
def test_variant_on_disk_equals_generator_output(kind, side):
    path = mv.variant_path(side, kind)
    expected = mv.build_variant(mv.source_path(kind), side=side, kind=kind)
    assert path.read_text(encoding="utf-8") == expected


# ── 等价性断言（除改名外逐字段相等）─────────────────────────────────────
@pytest.mark.parametrize("kind,side", VARIANT_PAIRS)
def test_variant_is_renamed_only(kind, side):
    src_root = _root_of(mv.source_path(kind).read_text(encoding="utf-8"))
    var_root = _root_of(mv.variant_path(side, kind).read_text(encoding="utf-8"))

    src_links = _links(src_root)
    var_links = _links(var_root)
    assert len(src_links) == len(var_links) == len(var_root.findall("link"))

    src_joints = _joints(src_root)
    var_joints = _joints(var_root)
    assert len(src_joints) == len(var_joints) == len(var_root.findall("joint"))

    for name, src_link in src_links.items():
        var_name = mv._SIDE_PREFIX[side] + name
        assert var_name in var_links, f"变体缺少 link {var_name!r}"
        var_link = var_links[var_name]
        # 逐 link 的 inertial（质量 / 质心 / 惯量张量）
        assert _inertial(var_link) == _inertial(src_link), f"link {name!r} 惯量被改动"
        # 逐 collision 的原点与几何
        src_coll = _collision_fingerprints(src_link)
        var_coll = _collision_fingerprints(var_link)
        assert len(src_coll) == len(var_coll)
        for s, v in zip(src_coll, var_coll):
            assert v["name"] == mv._SIDE_PREFIX[side] + s["name"]
            assert v["origin_xyz"] == s["origin_xyz"]
            assert v["origin_rpy"] == s["origin_rpy"]
            assert v["geometry"] == s["geometry"]

    for name, src_joint in src_joints.items():
        var_name = mv._SIDE_PREFIX[side] + name
        assert var_name in var_joints, f"变体缺少 joint {var_name!r}"
        var_joint = var_joints[var_name]
        # 类型、限位、轴、关节原点均不变（固定关节无 <axis>，两侧都必须没有）
        assert var_joint.get("type") == src_joint.get("type")
        assert _limit(var_joint) == _limit(src_joint)
        src_axis = src_joint.find("axis")
        var_axis = var_joint.find("axis")
        assert (src_axis is None) == (var_axis is None)
        if src_axis is not None:
            assert var_axis.attrib == src_axis.attrib
        assert var_joint.find("origin").attrib == src_joint.find("origin").attrib
        # 连接关系不变（仅 link 名带前缀）
        assert (
            _strip_prefix(var_joint.find("parent").get("link"), side)
            == src_joint.find("parent").get("link")
        )
        assert (
            _strip_prefix(var_joint.find("child").get("link"), side)
            == src_joint.find("child").get("link")
        )


@pytest.mark.parametrize("kind,side", VARIANT_PAIRS)
def test_variant_robot_name(kind, side):
    root = _root_of(mv.variant_path(side, kind).read_text(encoding="utf-8"))
    expected = mv.variant_robot_name(kind, side)
    assert root.get("name") == expected
    assert expected in {"RM75-B-left", "RM75-B-right", "RMG24-left", "RMG24-right"}


# ── 命名断言 ────────────────────────────────────────────────────────────
@pytest.mark.parametrize("kind,side", VARIANT_PAIRS)
def test_all_names_carry_side_prefix(kind, side):
    prefix = mv._SIDE_PREFIX[side]
    root = _root_of(mv.variant_path(side, kind).read_text(encoding="utf-8"))
    for link in root.findall("link"):
        assert link.get("name").startswith(prefix)
    for joint in root.findall("joint"):
        assert joint.get("name").startswith(prefix)
    for coll in root.iter("collision"):
        assert coll.get("name").startswith(prefix)
    for joint in root.findall("joint"):
        assert joint.find("parent").get("link").startswith(prefix)
        assert joint.find("child").get("link").startswith(prefix)


def test_left_and_right_variants_share_no_frame_names():
    """左/右变体的 frame 名必须完全不重叠（否则同 plant 装配会撞名）。"""
    left = _root_of(mv.variant_path("left", "rm75").read_text(encoding="utf-8"))
    right = _root_of(mv.variant_path("right", "rm75").read_text(encoding="utf-8"))
    left_names = {el.get("name") for el in left.iter() if el.get("name")}
    right_names = {el.get("name") for el in right.iter() if el.get("name")}
    assert not (left_names & right_names)


# ── 零外部资产 ──────────────────────────────────────────────────────────
@pytest.mark.parametrize("kind,side", VARIANT_PAIRS)
def test_variant_has_no_mesh_reference(kind, side):
    """零外部资产：变体里**没有任何网格几何元素**，也没引入源文件之外的新引用。

    注意口径：源 URDF 的**文件头注释**里出现过 ``mesh``（工具名
    ``convert_meshes.py`` / ``mesh_tools.py``）与 ``.STL``（历史说明），
    因此不能对整份文本做子串断言；正确的判据是

    1. 解析后的模型里**没有** ``<mesh>`` 元素、没有任何 ``filename`` 属性；
    2. 变体文本里 ``mesh`` 的出现次数**不超过源文件**（不得引入新依赖）。
    """
    text = mv.variant_path(side, kind).read_text(encoding="utf-8")
    src_text = mv.source_path(kind).read_text(encoding="utf-8")
    root = _root_of(text)

    for element in root.iter():
        tag = element.tag if isinstance(element.tag, str) else ""
        assert not tag.endswith("mesh"), f"变体引入了 <mesh> 元素：{tag}"
        assert "filename" not in element.attrib, "变体引入了 filename（外部资产）"

    assert text.lower().count("mesh") <= src_text.lower().count("mesh")
    assert text.lower().count(".stl") <= src_text.lower().count(".stl")


# ── 负例 ────────────────────────────────────────────────────────────────
def test_same_urdf_twice_in_one_plant_raises():
    """负例（证明变体路线的必要性）：同一份 URDF 在同一 plant 里只能加一次。"""
    pytest.importorskip("pydrake")
    from rm75_drake import package_map

    parser, _ = package_map.make_parser()
    parser.AddModels(url="package://rm75_drake/rm75_drake.urdf")
    with pytest.raises(RuntimeError) as excinfo:
        parser.AddModels(url="package://rm75_drake/rm75_drake.urdf")
    assert "model instance names must be unique" in str(excinfo.value).lower()


def test_left_and_right_variants_load_into_one_plant():
    """正向：两个变体各加一次即可进同一 plant（不抛错）。"""
    pytest.importorskip("pydrake")
    from rm75_drake import package_map

    parser, _ = package_map.make_parser()
    left = parser.AddModels(url="package://rm75_drake/rm75_left.urdf")
    right = parser.AddModels(url="package://rm75_drake/rm75_right.urdf")
    assert left and right
    assert set(left).isdisjoint(right)


def test_check_only_detects_tampered_variant(tmp_path):
    """负例：变体被手改后 ``check_only=True`` 必须报错（不静默通过）。"""
    out = tmp_path / "models"
    out.mkdir()
    for kind, side in VARIANT_PAIRS:
        shutil.copy(mv.variant_path(side, kind), out / VARIANT_FILES[(kind, side)])
    # 篡改一个变体
    tampered = out / VARIANT_FILES[("rm75", "left")]
    tampered.write_text(
        tampered.read_text(encoding="utf-8").replace(
            'lower="-3.106"', 'lower="-3.0"'
        ),
        encoding="utf-8",
    )
    with pytest.raises(mv.VariantMismatchError):
        mv.write_variants(check_only=True, out_dir=out)
    report = mv.check_variants(out_dir=out)
    assert report["rm75_left"]["status"] == "stale"
    assert report["rm75_right"]["status"] == "ok"


def test_check_only_detects_missing_variant(tmp_path):
    """负例：变体缺失 → 报 ``missing``。"""
    report = mv.check_variants(out_dir=tmp_path)
    assert all(entry["status"] == "missing" for entry in report.values())


def test_invalid_kind_or_side_raises():
    with pytest.raises(mv.VariantError):
        mv.variant_robot_name("rm65", "left")
    with pytest.raises(mv.VariantError):
        mv.variant_robot_name("rm75", "middle")
    with pytest.raises(mv.VariantError):
        mv.build_variant(mv.source_path("rm75"), side="left", kind="rm65")


def test_missing_source_file_raises(tmp_path):
    with pytest.raises(mv.VariantError):
        mv.build_variant(tmp_path / "nope.urdf", side="left", kind="rm75")


def test_source_robot_name_mismatch_raises(tmp_path):
    """负例：源 URDF 的 ``<robot name>`` 与 kind 不匹配 → 显式报错（不静默改名）。"""
    fake = tmp_path / "fake.urdf"
    fake.write_text(
        '<?xml version="1.0" encoding="utf-8"?>\n<robot name="SOMETHING-ELSE">\n'
        '  <link name="base_link"/>\n</robot>\n',
        encoding="utf-8",
    )
    with pytest.raises(mv.VariantError):
        mv.build_variant(fake, side="left", kind="rm75")


def test_unnamed_link_or_joint_raises(tmp_path):
    """负例：源 URDF 里有无名 link/joint → 显式报错。"""
    fake = tmp_path / "fake.urdf"
    fake.write_text(
        '<?xml version="1.0" encoding="utf-8"?>\n<robot name="RM75-B">\n'
        '  <link/>\n</robot>\n',
        encoding="utf-8",
    )
    with pytest.raises(mv.VariantError):
        mv.build_variant(fake, side="left", kind="rm75")
