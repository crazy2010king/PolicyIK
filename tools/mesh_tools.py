#!/usr/bin/env python3
"""tools/mesh_tools.py —— 纯 numpy/scipy 的 STL 解析 / 测量 / OBJ 转换工具

为什么自己写（T01 决策，勿轻易引入新依赖）
--------------------------------------------
* Drake 1.51.1 / 1.55.0 **都不解析 .stl**（``MakeConvexHull`` 只吃 .obj/.vtk/.gltf），
  因此官方 RM75-B 的 STL 必须转成 OBJ 才能进 Drake。
* ``versions.lock`` 钉死的 18 个包里**没有** trimesh / numpy-stl；引入它们要同时改
  lock + 两份 constraints + wheel 镜像三处，且 numpy-stl 会带进 python-stl 传递依赖。
* 因此本模块只用已钉版的 ``numpy`` + ``scipy`` 实现，**禁止** import
  ``trimesh`` / ``stl`` / ``numpy-stl`` / ``vtk``（tests/test_mesh_tools.py 有反向断言）。

功能
----
* STL 解析：binary（80 字节头 + uint32 三角面数 + 每面 50 字节）与 ascii
  （``solid/facet/outer loop/vertex``）两种格式；
* 三角面 → 顶点数组（顶点按输入顺序保留，便于确定性输出）；
* 确定性 OBJ 写出：固定小数位、顶点顺序与输入一致、同一输入两次运行**字节一致**；
* 轴对齐包围盒（min/max/extent/中心）；
* 三面体体积法（散度定理）求网格体积；
* ``scipy.spatial.ConvexHull`` 求凸包顶点数与体积。

异常
----
所有输入错误统一抛 :class:`MeshError`（``ValueError`` 子类），消息里说明**哪一处**
不合法（偏移量 / 行号 / 期望长度），便于定位损坏的上游资产。
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence

import numpy as np
from scipy.spatial import ConvexHull, QhullError

__all__ = [
    "MeshError",
    "Mesh",
    "BoundingBox",
    "HullInfo",
    "parse_binary_stl",
    "parse_ascii_stl",
    "load_stl",
    "read_stl_bytes",
    "mesh_to_obj_text",
    "write_obj",
    "bounding_box",
    "mesh_bounding_box",
    "signed_volume",
    "mesh_volume",
    "convex_hull_info",
    "mesh_stats",
]

#: OBJ 写出的小数位数（固定值 → 输出字节可复现）。
OBJ_DECIMALS = 9

_BINARY_HEADER_BYTES = 80
_BINARY_TRIANGLE_COUNT_BYTES = 4
_BINARY_TRIANGLE_BYTES = 50


class MeshError(ValueError):
    """网格解析 / 测量错误（输入非法）。"""


@dataclass(frozen=True, eq=False)
class Mesh:
    """三角网格。

    * ``vertices``：形状 (N, 3) 的 float64 顶点数组，**按文件中的出现顺序**排列；
    * ``faces``：形状 (M, 3) 的 int64 顶点索引，每行一个三角面。
    """

    vertices: np.ndarray
    faces: np.ndarray
    name: str = ""

    def __post_init__(self) -> None:
        v = np.asarray(self.vertices, dtype=np.float64)
        f = np.asarray(self.faces, dtype=np.int64)
        if v.ndim != 2 or v.shape[1] != 3:
            raise MeshError(f"顶点数组形状非法：{v.shape}（应为 (N, 3)）")
        if f.ndim != 2 or f.shape[1] != 3:
            raise MeshError(f"面索引数组形状非法：{f.shape}（应为 (M, 3)）")
        if f.size and (f.min() < 0 or f.max() >= v.shape[0]):
            raise MeshError(
                f"面索引越界：范围 [{f.min()}, {f.max()}] 超出顶点数 {v.shape[0]}"
            )
        object.__setattr__(self, "vertices", v)
        object.__setattr__(self, "faces", f)

    @property
    def vertex_count(self) -> int:
        return int(self.vertices.shape[0])

    @property
    def face_count(self) -> int:
        return int(self.faces.shape[0])

    def triangles(self) -> np.ndarray:
        """返回形状 (M, 3, 3) 的三角面坐标数组。"""
        return self.vertices[self.faces]


@dataclass(frozen=True)
class BoundingBox:
    """轴对齐包围盒。"""

    min: np.ndarray
    max: np.ndarray

    @property
    def extent(self) -> np.ndarray:
        return self.max - self.min

    @property
    def center(self) -> np.ndarray:
        return 0.5 * (self.min + self.max)

    def as_dict(self) -> dict:
        return {
            "min": [float(x) for x in self.min],
            "max": [float(x) for x in self.max],
            "extent": [float(x) for x in self.extent],
            "center": [float(x) for x in self.center],
        }


@dataclass(frozen=True)
class HullInfo:
    """凸包信息。"""

    vertex_count: int
    volume: float
    indices: tuple


# ── binary STL ───────────────────────────────────────────────────────────
def parse_binary_stl(data: bytes, name: str = "") -> Mesh:
    """解析 binary STL。

    布局：80 字节头 + uint32 三角面数 + 每个三角面 50 字节
    （法向 3×float32 + 顶点 3×3×float32 + uint16 属性字节数）。
    """
    if not isinstance(data, (bytes, bytearray, memoryview)):
        raise MeshError("binary STL 输入必须是 bytes")
    data = bytes(data)
    if len(data) < _BINARY_HEADER_BYTES + _BINARY_TRIANGLE_COUNT_BYTES:
        raise MeshError(
            f"binary STL 太短：{len(data)} 字节（至少需要 "
            f"{_BINARY_HEADER_BYTES + _BINARY_TRIANGLE_COUNT_BYTES}）"
        )
    (count,) = struct.unpack_from("<I", data, _BINARY_HEADER_BYTES)
    expected = (
        _BINARY_HEADER_BYTES
        + _BINARY_TRIANGLE_COUNT_BYTES
        + _BINARY_TRIANGLE_BYTES * count
    )
    if len(data) != expected:
        raise MeshError(
            f"binary STL 长度不符：声明 {count} 个三角面 → 期望 {expected} 字节，"
            f"实际 {len(data)} 字节（文件被截断或不是 binary STL）"
        )
    if count == 0:
        raise MeshError("binary STL 不含任何三角面")

    raw = np.frombuffer(
        data,
        dtype=np.dtype(
            [
                ("normal", "<f4", 3),
                ("v", "<f4", (3, 3)),
                ("attr", "<u2"),
            ]
        ),
        offset=_BINARY_HEADER_BYTES + _BINARY_TRIANGLE_COUNT_BYTES,
        count=count,
    )
    verts = raw["v"].astype(np.float64)  # (M, 3, 3)
    vertices = verts.reshape(-1, 3)
    faces = np.arange(count * 3, dtype=np.int64).reshape(count, 3)
    return Mesh(vertices=vertices, faces=faces, name=name)


# ── ascii STL ────────────────────────────────────────────────────────────
def parse_ascii_stl(data: bytes | str, name: str = "") -> Mesh:
    """解析 ascii STL（``solid`` / ``facet normal`` / ``outer loop`` / ``vertex``）。"""
    if isinstance(data, (bytes, bytearray, memoryview)):
        try:
            text = bytes(data).decode("utf-8")
        except UnicodeDecodeError as exc:  # pragma: no cover - 极少发生
            raise MeshError(f"ascii STL 不是合法 UTF-8 文本：{exc}") from exc
    elif isinstance(data, str):
        text = data
    else:
        raise MeshError("ascii STL 输入必须是 str 或 bytes")

    vertices: list[tuple[float, float, float]] = []
    in_facet = False
    facet_vertices = 0
    saw_solid = False
    saw_endsolid = False

    for lineno, raw_line in enumerate(text.splitlines(), start=1):
        line = raw_line.strip()
        if not line or line.startswith("solid"):
            # 头部 solid <name> 允许存在
            if line.startswith("solid"):
                saw_solid = True
            continue
        if line.startswith("endsolid"):
            saw_endsolid = True
            continue
        if line.startswith("facet"):
            if in_facet:
                raise MeshError(f"ascii STL 第 {lineno} 行：facet 未闭合就开始新 facet")
            in_facet = True
            facet_vertices = 0
            continue
        if line.startswith("endfacet"):
            if not in_facet:
                raise MeshError(f"ascii STL 第 {lineno} 行：出现多余的 endfacet")
            if facet_vertices != 3:
                raise MeshError(
                    f"ascii STL 第 {lineno} 行：facet 只有 {facet_vertices} 个顶点（应为 3）"
                )
            in_facet = False
            continue
        if line.startswith("vertex"):
            if not in_facet:
                raise MeshError(f"ascii STL 第 {lineno} 行：vertex 出现在 facet 之外")
            parts = line.split()
            if len(parts) != 4:
                raise MeshError(
                    f"ascii STL 第 {lineno} 行：vertex 需要 3 个坐标，实得 {len(parts) - 1}"
                )
            try:
                xyz = (float(parts[1]), float(parts[2]), float(parts[3]))
            except ValueError as exc:
                raise MeshError(f"ascii STL 第 {lineno} 行：坐标不是合法浮点数：{exc}") from exc
            vertices.append(xyz)
            facet_vertices += 1
            continue
        # outer loop / endloop / 其他关键字直接忽略

    if in_facet:
        raise MeshError("ascii STL 结尾时仍有未闭合的 facet")
    if not saw_solid:
        raise MeshError("ascii STL 缺少 solid 头")
    if not saw_endsolid:
        raise MeshError("ascii STL 缺少 endsolid 尾")
    if not vertices:
        raise MeshError("ascii STL 不含任何顶点")

    v = np.asarray(vertices, dtype=np.float64)
    faces = np.arange(v.shape[0], dtype=np.int64).reshape(-1, 3)
    return Mesh(vertices=v, faces=faces, name=name)


# ── 统一入口 ─────────────────────────────────────────────────────────────
def read_stl_bytes(data: bytes, name: str = "") -> Mesh:
    """按内容自动判定 binary / ascii 并解析。"""
    data = bytes(data)
    if len(data) >= _BINARY_HEADER_BYTES + _BINARY_TRIANGLE_COUNT_BYTES:
        (count,) = struct.unpack_from("<I", data, _BINARY_HEADER_BYTES)
        expected = (
            _BINARY_HEADER_BYTES
            + _BINARY_TRIANGLE_COUNT_BYTES
            + _BINARY_TRIANGLE_BYTES * count
        )
        if len(data) == expected:
            return parse_binary_stl(data, name=name)
    stripped = data.lstrip()[:5].lower()
    if stripped.startswith(b"solid"):
        return parse_ascii_stl(data, name=name)
    raise MeshError(
        f"无法识别的 STL：{len(data)} 字节，既不符合 binary 布局也不是 ascii 'solid' 开头"
    )


def load_stl(path: str | Path) -> Mesh:
    """从磁盘读取并解析一个 STL 文件。"""
    p = Path(path)
    try:
        data = p.read_bytes()
    except OSError as exc:
        raise MeshError(f"读取 STL 失败：{p}（{exc}）") from exc
    return read_stl_bytes(data, name=p.stem)


# ── OBJ 写出 ─────────────────────────────────────────────────────────────
def mesh_to_obj_text(mesh: Mesh, decimals: int = OBJ_DECIMALS, source: str = "") -> str:
    """把网格渲染成确定性 OBJ 文本。

    确定性来源：固定小数位、顶点顺序 = 输入顺序、面索引引用同一顺序、
    行尾恒为 ``\\n``、**不写入任何时间戳或随机量** → 同一输入两次运行字节一致。
    """
    if not isinstance(decimals, int) or decimals < 0 or decimals > 17:
        raise MeshError(f"OBJ 小数位数非法：{decimals}（应为 0..17 的整数）")
    fmt = f"%.{decimals}f"
    lines: list[str] = ["# OBJ generated by tools/mesh_tools.py (deterministic)"]
    if source:
        lines.append(f"# source: {source}")
    lines.append(f"# vertices: {mesh.vertex_count}  faces: {mesh.face_count}")
    for v in mesh.vertices:
        # + 0.0 把 -0.0 归一成 0.0，避免输出里出现 "-0.000000000"
        lines.append(
            "v "
            f"{fmt % (float(v[0]) + 0.0)} "
            f"{fmt % (float(v[1]) + 0.0)} "
            f"{fmt % (float(v[2]) + 0.0)}"
        )
    for f in mesh.faces:
        lines.append(f"f {int(f[0]) + 1} {int(f[1]) + 1} {int(f[2]) + 1}")
    return "\n".join(lines) + "\n"


def write_obj(
    mesh: Mesh,
    path: str | Path,
    decimals: int = OBJ_DECIMALS,
    source: str = "",
) -> Path:
    """确定性写出 OBJ 文件（原子替换），返回写入路径。"""
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    text = mesh_to_obj_text(mesh, decimals=decimals, source=source)
    tmp = p.with_suffix(p.suffix + ".tmp")
    tmp.write_text(text, encoding="utf-8", newline="\n")
    tmp.replace(p)
    return p


# ── 测量 ─────────────────────────────────────────────────────────────────
def bounding_box(vertices: np.ndarray | Iterable[Sequence[float]]) -> BoundingBox:
    """轴对齐包围盒（min/max/extent/中心）。"""
    v = np.asarray(vertices, dtype=np.float64)
    if v.ndim != 2 or v.shape[1] != 3 or v.shape[0] == 0:
        raise MeshError(f"包围盒输入非法：形状 {v.shape}")
    if not np.isfinite(v).all():
        raise MeshError("包围盒输入含 NaN/Inf")
    return BoundingBox(min=v.min(axis=0), max=v.max(axis=0))


def mesh_bounding_box(mesh: Mesh) -> BoundingBox:
    """网格的轴对齐包围盒。"""
    return bounding_box(mesh.vertices)


def signed_volume(mesh: Mesh) -> float:
    """三面体体积法（散度定理）求网格体积（带符号）。"""
    tris = mesh.triangles()
    if tris.shape[0] == 0:
        raise MeshError("空网格没有体积")
    v0 = tris[:, 0, :]
    v1 = tris[:, 1, :]
    v2 = tris[:, 2, :]
    vol = np.einsum("ij,ij->i", v0, np.cross(v1, v2)) / 6.0
    return float(vol.sum())


def mesh_volume(mesh: Mesh) -> float:
    """网格体积（绝对值）。"""
    return abs(signed_volume(mesh))


def convex_hull_info(vertices: np.ndarray | Iterable[Sequence[float]]) -> HullInfo:
    """凸包顶点数与体积（``scipy.spatial.ConvexHull``）。"""
    v = np.asarray(vertices, dtype=np.float64)
    if v.ndim != 2 or v.shape[1] != 3 or v.shape[0] == 0:
        raise MeshError(f"凸包输入非法：形状 {v.shape}")
    if not np.isfinite(v).all():
        raise MeshError("凸包输入含 NaN/Inf")
    try:
        hull = ConvexHull(v)
    except QhullError as exc:
        raise MeshError(f"凸包计算失败（退化几何？）：{exc}") from exc
    return HullInfo(
        vertex_count=int(len(hull.vertices)),
        volume=float(hull.volume),
        indices=tuple(int(i) for i in hull.vertices),
    )


def mesh_stats(mesh: Mesh) -> dict:
    """一次性汇总网格的常用度量（供 CLI 打印包围盒表）。"""
    box = mesh_bounding_box(mesh)
    try:
        hull = convex_hull_info(mesh.vertices)
        hull_vertices = hull.vertex_count
        hull_volume = hull.volume
    except MeshError:
        hull_vertices = -1
        hull_volume = float("nan")
    return {
        "name": mesh.name,
        "vertices": mesh.vertex_count,
        "faces": mesh.face_count,
        "min": [float(x) for x in box.min],
        "max": [float(x) for x in box.max],
        "extent": [float(x) for x in box.extent],
        "center": [float(x) for x in box.center],
        "volume": mesh_volume(mesh),
        "hull_vertices": hull_vertices,
        "hull_volume": hull_volume,
    }
