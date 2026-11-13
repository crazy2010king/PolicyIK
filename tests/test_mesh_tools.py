"""tests/test_mesh_tools.py —— T01：STL 解析 / 测量 / 确定性 OBJ 转换

覆盖（正例）
  - binary STL 与 ascii STL 解析同一组三角面 → 顶点与面索引完全一致
  - 轴对齐包围盒 / 中心 与已知值一致（合成立方体 + 官方 base_link 实测值）
  - 三面体体积法：单位立方体体积 = 1.0 m³
  - scipy ConvexHull：单位立方体 → 8 个凸包顶点、体积 1.0 m³
  - OBJ 写出确定性：同一输入两次写出**字节一致**
  - 官方 8 个 STL 可被解析并产出 OBJ（上游资产缺失时显式 SKIP）

覆盖（负例）
  - 非法 STL：截断的 binary / 垃圾字节 / 缺 solid 头的 ascii / facet 顶点数不对
    → 必须抛 MeshError（不得静默返回空网格）
  - 顶点索引越界的 Mesh 构造 → 必须抛 MeshError
  - 退化几何（所有点共线）的凸包 → 必须抛 MeshError 而不是崩溃
  - 反向断言：mesh_tools.py / convert_meshes.py 源码不得 import
    trimesh / stl / numpy-stl / vtk（引入它们要改 lock + constraints + 镜像三处）
"""

from __future__ import annotations

import struct
import sys
from pathlib import Path

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
TOOLS_DIR = REPO_ROOT / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from mesh_tools import (  # noqa: E402
    Mesh,
    MeshError,
    bounding_box,
    convex_hull_info,
    load_stl,
    mesh_bounding_box,
    mesh_to_obj_text,
    mesh_volume,
    parse_ascii_stl,
    parse_binary_stl,
    read_stl_bytes,
    write_obj,
)

#: 官方 RM75-B 网格（由 scripts/extract_upstream_assets.py 物化，不入 git）。
UPSTREAM_MESH_DIR = (
    REPO_ROOT / "third_party" / ".work" / "rm75b" / "RM75" / "urdf" / "RM75-B" / "meshes"
)

#: 官方 base_link.STL 的实测包围盒（容差 1e-4 m，见 S01 计划）。
BASE_LINK_EXTENT = (0.1143, 0.1100, 0.1385)
BASE_LINK_CENTER = (-0.00215, 0.0, 0.06925)


# ── fixture 工具 ─────────────────────────────────────────────────────────
def unit_cube_triangles() -> np.ndarray:
    """单位立方体（边长 1）的 12 个三角面，形状 (12, 3, 3)。"""
    c = np.array(
        [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, 1.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 1.0],
            [1.0, 1.0, 1.0],
            [0.0, 1.0, 1.0],
        ]
    )
    quads = [
        (0, 1, 2, 3),  # z=0
        (4, 5, 6, 7),  # z=1
        (0, 1, 5, 4),  # y=0
        (2, 3, 7, 6),  # y=1
        (1, 2, 6, 5),  # x=1
        (0, 3, 7, 4),  # x=0
    ]
    tris = []
    for a, b, cc, d in quads:
        tris.append([c[a], c[b], c[cc]])
        tris.append([c[a], c[cc], c[d]])
    return np.asarray(tris, dtype=np.float64)


def to_binary_stl(triangles: np.ndarray) -> bytes:
    """把 (M, 3, 3) 三角面数组编码成 binary STL 字节流（float32 量化）。"""
    tris = np.asarray(triangles, dtype=np.float32)
    out = bytearray(b"\0" * 80)
    out += struct.pack("<I", tris.shape[0])
    for t in tris:
        normal = np.cross(t[1] - t[0], t[2] - t[0])
        norm = np.linalg.norm(normal)
        normal = normal / norm if norm > 0 else np.zeros(3, dtype=np.float32)
        out += np.asarray(normal, dtype="<f4").tobytes()
        out += np.asarray(t, dtype="<f4").tobytes()
        out += struct.pack("<H", 0)
    return bytes(out)


def to_ascii_stl(triangles: np.ndarray, name: str = "fixture") -> str:
    """把 (M, 3, 3) 三角面数组编码成 ascii STL 文本。

    坐标用 ``%.17g`` 输出 float32 量化后的精确值 → ascii 与 binary 解析结果**完全相等**。
    """
    tris = np.asarray(triangles, dtype=np.float32).astype(np.float64)
    lines = [f"solid {name}"]
    for t in tris:
        normal = np.cross(t[1] - t[0], t[2] - t[0])
        norm = np.linalg.norm(normal)
        normal = normal / norm if norm > 0 else np.zeros(3)
        lines.append("  facet normal " + " ".join(f"{v:.17g}" for v in normal))
        lines.append("    outer loop")
        for v in t:
            lines.append("      vertex " + " ".join(f"{x:.17g}" for x in v))
        lines.append("    endloop")
        lines.append("  endfacet")
    lines.append("endsolid " + name)
    return "\n".join(lines) + "\n"


@pytest.fixture()
def cube_tris() -> np.ndarray:
    return unit_cube_triangles()


# ── 解析一致性 ───────────────────────────────────────────────────────────
def test_binary_and_ascii_parse_to_identical_mesh(cube_tris):
    """同一组三角面 → binary / ascii 解析出的顶点与面索引完全一致。"""
    bin_mesh = parse_binary_stl(to_binary_stl(cube_tris))
    asc_mesh = parse_ascii_stl(to_ascii_stl(cube_tris))
    assert bin_mesh.vertex_count == asc_mesh.vertex_count == 36
    assert bin_mesh.face_count == asc_mesh.face_count == 12
    np.testing.assert_array_equal(bin_mesh.vertices, asc_mesh.vertices)
    np.testing.assert_array_equal(bin_mesh.faces, asc_mesh.faces)


def test_read_stl_bytes_auto_detects_format(cube_tris):
    """统一入口按内容自动判定 binary / ascii。"""
    m_bin = read_stl_bytes(to_binary_stl(cube_tris))
    m_asc = read_stl_bytes(to_ascii_stl(cube_tris).encode("utf-8"))
    np.testing.assert_array_equal(m_bin.vertices, m_asc.vertices)


# ── 测量 ─────────────────────────────────────────────────────────────────
def test_bounding_box_of_unit_cube(cube_tris):
    mesh = parse_binary_stl(to_binary_stl(cube_tris))
    box = mesh_bounding_box(mesh)
    np.testing.assert_allclose(box.extent, (1.0, 1.0, 1.0), atol=1e-7)
    np.testing.assert_allclose(box.center, (0.5, 0.5, 0.5), atol=1e-7)
    np.testing.assert_allclose(box.min, (0.0, 0.0, 0.0), atol=1e-7)


def test_bounding_box_of_known_box():
    """包围盒 API 对任意点集给出正确的 min/max/extent/中心。"""
    pts = np.array([[-1.0, 2.0, 0.5], [3.0, 2.0, 0.5], [1.0, -4.0, 0.5]])
    box = bounding_box(pts)
    np.testing.assert_allclose(box.min, (-1.0, -4.0, 0.5), atol=1e-12)
    np.testing.assert_allclose(box.max, (3.0, 2.0, 0.5), atol=1e-12)
    np.testing.assert_allclose(box.extent, (4.0, 6.0, 0.0), atol=1e-12)
    np.testing.assert_allclose(box.center, (1.0, -1.0, 0.5), atol=1e-12)


def test_volume_of_unit_cube(cube_tris):
    """三面体体积法：单位立方体体积 = 1.0 m³。"""
    mesh = parse_binary_stl(to_binary_stl(cube_tris))
    assert mesh_volume(mesh) == pytest.approx(1.0, abs=1e-6)


def test_convex_hull_of_unit_cube(cube_tris):
    """scipy ConvexHull：单位立方体 → 8 个凸包顶点、体积 1.0 m³。"""
    mesh = parse_binary_stl(to_binary_stl(cube_tris))
    hull = convex_hull_info(mesh.vertices)
    assert hull.vertex_count == 8
    assert hull.volume == pytest.approx(1.0, abs=1e-6)


# ── 确定性 OBJ ───────────────────────────────────────────────────────────
def test_obj_output_is_byte_identical_across_runs(cube_tris, tmp_path):
    """同一输入两次写出 → 字节一致（确定性输出）。"""
    mesh = parse_binary_stl(to_binary_stl(cube_tris))
    a = write_obj(mesh, tmp_path / "a.obj", source="cube.STL")
    b = write_obj(mesh, tmp_path / "b.obj", source="cube.STL")
    assert a.read_bytes() == b.read_bytes()
    # 同一路径重复写也必须字节一致
    first = a.read_bytes()
    write_obj(mesh, a, source="cube.STL")
    assert a.read_bytes() == first


def test_obj_text_keeps_vertex_order_and_has_no_timestamp(cube_tris):
    """顶点顺序 = 输入顺序；输出里不含时间戳等易变内容。"""
    mesh = parse_binary_stl(to_binary_stl(cube_tris))
    text = mesh_to_obj_text(mesh, source="cube.STL")
    lines = [ln for ln in text.splitlines() if ln.startswith("v ")]
    assert len(lines) == mesh.vertex_count
    first = mesh.vertices[0]
    assert lines[0].split()[1:4] == [
        f"{float(first[0]) + 0.0:.9f}",
        f"{float(first[1]) + 0.0:.9f}",
        f"{float(first[2]) + 0.0:.9f}",
    ]
    assert "# source: cube.STL" in text


# ── 负例：非法输入 ───────────────────────────────────────────────────────
def test_truncated_binary_stl_raises():
    """binary STL 被截断 → 必须抛 MeshError（不得返回空/半截网格）。"""
    data = to_binary_stl(unit_cube_triangles())
    with pytest.raises(MeshError):
        parse_binary_stl(data[:-30])


def test_garbage_bytes_raise():
    """既不是 binary 布局也不是 ascii 的垃圾字节 → 必须抛 MeshError。"""
    with pytest.raises(MeshError):
        read_stl_bytes(b"\x01\x02\x03not a stl at all\x00" * 4)


def test_ascii_without_solid_header_raises():
    """缺 solid 头的 ascii → 必须抛 MeshError。"""
    text = "facet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 1 0 0\nvertex 0 1 0\nendloop\nendfacet\n"
    with pytest.raises(MeshError):
        parse_ascii_stl(text)


def test_ascii_facet_with_wrong_vertex_count_raises():
    """facet 顶点不是 3 个 → 必须抛 MeshError。"""
    text = (
        "solid bad\nfacet normal 0 0 1\nouter loop\n"
        "vertex 0 0 0\nvertex 1 0 0\nendloop\nendfacet\nendsolid bad\n"
    )
    with pytest.raises(MeshError):
        parse_ascii_stl(text)


def test_ascii_without_endsolid_raises():
    """缺 endsolid 尾 → 必须抛 MeshError。"""
    text = (
        "solid bad\nfacet normal 0 0 1\nouter loop\n"
        "vertex 0 0 0\nvertex 1 0 0\nvertex 0 1 0\nendloop\nendfacet\n"
    )
    with pytest.raises(MeshError):
        parse_ascii_stl(text)


def test_mesh_with_out_of_range_face_index_raises():
    """面索引越界 → Mesh 构造必须抛 MeshError。"""
    with pytest.raises(MeshError):
        Mesh(
            vertices=np.zeros((3, 3)),
            faces=np.array([[0, 1, 9]]),
        )


def test_degenerate_convex_hull_raises():
    """退化几何（共线点）的凸包 → 必须抛 MeshError 而不是崩溃。"""
    pts = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]])
    with pytest.raises(MeshError):
        convex_hull_info(pts)


def test_bounding_box_rejects_empty_and_nan():
    """空点集 / NaN 点集 → 必须抛 MeshError。"""
    with pytest.raises(MeshError):
        bounding_box(np.zeros((0, 3)))
    with pytest.raises(MeshError):
        bounding_box(np.array([[np.nan, 0.0, 0.0], [1.0, 1.0, 1.0]]))


# ── 上游官方网格（资产缺失时显式 SKIP）────────────────────────────────────
@pytest.mark.skipif(
    not (UPSTREAM_MESH_DIR / "base_link.STL").is_file(),
    reason="上游官方 STL 未物化（先运行 tools/fetch_upstream.py + scripts/extract_upstream_assets.py）",
)
def test_official_base_link_bounding_box():
    """官方 base_link.STL 实测 ext = 0.1143 × 0.1100 × 0.1385 m（容差 1e-4）。"""
    mesh = load_stl(UPSTREAM_MESH_DIR / "base_link.STL")
    box = mesh_bounding_box(mesh)
    np.testing.assert_allclose(box.extent, BASE_LINK_EXTENT, atol=1e-4)
    np.testing.assert_allclose(box.center, BASE_LINK_CENTER, atol=1e-4)


@pytest.mark.skipif(
    not UPSTREAM_MESH_DIR.is_dir(),
    reason="上游官方 STL 未物化",
)
def test_official_meshes_parse_and_convert(tmp_path):
    """官方 8 个 STL 全部可解析并可确定性转成 OBJ。"""
    stls = sorted(UPSTREAM_MESH_DIR.glob("*.STL"))
    assert len(stls) == 8, f"期望 8 个官方 STL，实际 {len(stls)}"
    for stl in stls:
        mesh = load_stl(stl)
        assert mesh.face_count > 0
        out = write_obj(mesh, tmp_path / f"{stl.stem}.obj", source=stl.name)
        again = write_obj(mesh, tmp_path / f"{stl.stem}.again.obj", source=stl.name)
        assert out.read_bytes() == again.read_bytes()


# ── 反向断言：禁止引入未钉版的网格库 ───────────────────────────────────
@pytest.mark.parametrize(
    "relative_path",
    ["tools/mesh_tools.py", "scripts/convert_meshes.py", "scripts/extract_upstream_assets.py"],
)
def test_no_forbidden_mesh_library_imports(relative_path):
    """源码**实际 import 的模块**里不得出现 trimesh / stl / numpy-stl / vtk。

    只看真实 import 语句（AST 解析），不扫注释与文档字符串——文档里可以自由讨论
    为什么不用这些库。
    """
    import ast

    source = (REPO_ROOT / relative_path).read_text(encoding="utf-8")
    banned = {"trimesh", "stl", "numpy_stl", "vtk", "meshio", "open3d", "pyassimp"}
    imported: set[str] = set()
    for node in ast.walk(ast.parse(source)):
        if isinstance(node, ast.Import):
            for alias in node.names:
                imported.add(alias.name.split(".")[0])
        elif isinstance(node, ast.ImportFrom):
            if node.level == 0 and node.module:
                imported.add(node.module.split(".")[0])
    assert not (imported & banned), (
        f"{relative_path} 引入了未钉版的网格库：{sorted(imported & banned)}"
    )
