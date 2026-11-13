"""空间凸集图最短路 + 轨迹生成 + 碰撞复检（S02-T05）。

硬修正（本切片的 GCS 口径，S05 复用同一口径）
------------------------------------------------
**本模块的 GCS = 空间凸集图最短路（space GCS shortest path），
不是轨迹级 GCS（trajectory-level GCS）。**

原因（**实测，Drake 1.51.1 / py3.10 / 无 Mosek 许可**）：

* Drake 的轨迹级 GCS（``GcsTrajectoryOptimization``）在官方 wheel 里**没有**
  ``Region`` / ``ContinuousConvexSet`` 的 Python 绑定，**不可用**；
* 可用的 GCS 是 :class:`pydrake.geometry.optimization.GraphOfConvexSets`
  的**最短路**问题（Marcucci, Umenberger, Parrilo, Tedrake,
  *Shortest Paths in Graphs of Convex Sets*, arXiv:2101.11565）——
  求解「从起点凸集到终点凸集、沿途经过若干凸集的最短路径」，
  **不**做时间参数化、**不**做动力学可行、**不**保证关节空间直线段无碰撞。

因此本模块的职责边界是：

1. 把 S02-T04 产出的 **C 空间凸区域**（IRIS 区域 + 软限位盒求交）
   组织成一张**凸集图**（:func:`build_graph`）；
2. 在图上求**最短路**（:func:`solve_path`），并把解路径转成
   **关节空间的分段线性折线**（:func:`path_to_q_sequence`）；
3. 对折线做**密采样碰撞复检**（:func:`collision_recheck`），
   给出**可解释的逃逸位形**（零逃逸是硬要求）。

**轨迹级 GCS 的缺口由 S05（Toppra 时间参数化）补齐**：
本模块只保证「几何上从起点到终点的路径」与「碰撞复检零逃逸」，
速度/加速度可行性由 S05 的时间参数化 + S02-T02 的
:class:`~rm75_drake.planning.trajectory.Trajectory` 不变量保证。

求解器模式（**必须记录在结果里**）
----------------------------------
整数版 GCS 需要混合整数求解器（Mosek / Gurobi / CPLEX）。本机实测：

* **无 Mosek 许可** → 整数版直接抛
  ``RuntimeError: GraphOfConvexSets: There is no solver available that can
  solve the mixed-integer version of this problem. ...``；
* 因此实现必须走 ``options.convex_relaxation = True`` +
  ``options.max_rounded_paths = N``（本机实测成功：path len = 2）；
* 有 Mosek 许可的环境（远端基线 172.16.119.199:2224 → ``ros_dev`` →
  conda ``drake_dev``，python 3.12 / Drake 1.55.0 / mosek 可 import）
  可切到整数版。

实现采用**能力探测**：先试整数版，捕获上述 RuntimeError 后回退凸松弛，
并把实际使用的模式记在 :attr:`GcsResult.solver_mode`
（``"mixed_integer"`` / ``"convex_relaxation"``）。

失败可解释
----------
所有失败都带稳定 ``reason`` 标签（:data:`FAILURE_REASONS`）：

===========================  ================================================
``no_path``                 起点/终点区域之间**无可行边路径**（图连通性不足）
``solver_unavailable``      求解器不可用（整数版与凸松弛都失败）
``empty_region``            区域为空（HPolyhedron.IsEmpty()）
``unknown_region``          给出的区域 id 不在图里
``invalid_input``           入参非法（维度、NaN、非正步长等）
===========================  ================================================

Drake 侧的其它错误原样冒泡（带上下文），**不**静默降级。
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Any, Iterable, List, Optional, Sequence, Tuple

import numpy as np

__all__ = [
    "GcsError",
    "GcsResult",
    "RecheckResult",
    "FAILURE_REASONS",
    "REASON_NO_PATH",
    "REASON_SOLVER_UNAVAILABLE",
    "REASON_EMPTY_REGION",
    "REASON_UNKNOWN_REGION",
    "REASON_INVALID_INPUT",
    "DEFAULT_ADJACENCY_THRESHOLD",
    "DEFAULT_MAX_ROUNDED_PATHS",
    "DEFAULT_STEP_SIZE",
    "DEFAULT_EDGE_STEP_SIZE",
    "SOLVER_MODE_MIXED_INTEGER",
    "SOLVER_MODE_CONVEX_RELAXATION",
    "build_graph",
    "solve_path",
    "path_to_q_sequence",
    "collision_recheck",
    "plan_q_sequence",
    "region_anchor",
]


LOGGER = logging.getLogger(__name__)

# ── 失败类别（稳定标签）────────────────────────────────────────────────
REASON_NO_PATH = "no_path"
REASON_SOLVER_UNAVAILABLE = "solver_unavailable"
REASON_EMPTY_REGION = "empty_region"
REASON_UNKNOWN_REGION = "unknown_region"
REASON_INVALID_INPUT = "invalid_input"

#: 全部失败类别。
FAILURE_REASONS: Tuple[str, ...] = (
    REASON_NO_PATH,
    REASON_SOLVER_UNAVAILABLE,
    REASON_EMPTY_REGION,
    REASON_UNKNOWN_REGION,
    REASON_INVALID_INPUT,
)

#: 求解模式标签。
SOLVER_MODE_MIXED_INTEGER = "mixed_integer"
SOLVER_MODE_CONVEX_RELAXATION = "convex_relaxation"

#: 默认邻接阈值（rad，C 空间欧氏距离，Chebyshev 中心之间）。
DEFAULT_ADJACENCY_THRESHOLD = 0.5

#: 凸松弛回退时的最大舍入路径数（``max_rounded_paths``）。
DEFAULT_MAX_ROUNDED_PATHS = 10

#: 轨迹稠密化的默认步长（rad，C 空间无穷范数相邻点距离上限）。
DEFAULT_STEP_SIZE = 0.01

#: 碰撞复检的默认边采样步长（rad），与 S02-T01 的场景装配口径一致。
DEFAULT_EDGE_STEP_SIZE = 0.01

#: 规划变量维度（RM75-B 7 DoF）。
JOINT_COUNT = 7


class GcsError(RuntimeError):
    """GCS 图构建 / 求解的显式失败。

    属性
    ----
    reason:
        稳定标签（见 :data:`FAILURE_REASONS`），默认 ``"gcs_error"``。
    """

    def __init__(self, message: str, *, reason: str = "gcs_error") -> None:
        super().__init__(message)
        self.reason = reason


# ── 数据结构 ─────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class GcsResult:
    """一次最短路求解的结果。

    字段
    ----
    success:
        是否找到可行路径。
    message:
        **可解释**的结果说明（成功时描述路径长度与求解模式；
        失败时说明失败类别与原因）。
    reason:
        失败类别标签（成功时为空串）。
    solver_mode:
        实际使用的求解模式（``"mixed_integer"`` / ``"convex_relaxation"``）；
        求解器完全不可用时为空串。
    edges:
        路径上的边序列（:class:`pydrake.geometry.optimization.GraphOfConvexSets.Edge`），
        顺序为起点 → 终点。
    edge_points:
        与 :attr:`edges` 一一对应的 ``(xu, xv)`` 数值解（numpy，7 维）。
    result:
        原始的 ``MathematicalProgramResult``（供进一步诊断）。
    """

    success: bool
    message: str
    reason: str = ""
    solver_mode: str = ""
    edges: Tuple[Any, ...] = ()
    edge_points: Tuple[Tuple[np.ndarray, np.ndarray], ...] = ()
    result: Any = None

    @property
    def num_edges(self) -> int:
        """路径上的边数。"""
        return len(self.edges)

    @property
    def waypoints(self) -> np.ndarray:
        """路径的**角点序列**（``(M, 7)``）。

        角点 = 每条边两端的解点；相邻边的共享顶点解点相同（连续性），
        因此这里对连续重复点做了去重。
        """
        if not self.edge_points:
            return np.zeros((0, JOINT_COUNT), dtype=float)
        rows: List[np.ndarray] = []
        for index, (xu, xv) in enumerate(self.edge_points):
            if index == 0:
                rows.append(np.asarray(xu, dtype=float))
            rows.append(np.asarray(xv, dtype=float))
        matrix = np.vstack(rows)
        keep = [0]
        for index in range(1, matrix.shape[0]):
            if np.max(np.abs(matrix[index] - matrix[keep[-1]])) > 1e-9:
                keep.append(index)
        return matrix[keep, :]


@dataclass(frozen=True)
class RecheckResult:
    """碰撞复检结果。

    字段
    ----
    num_samples:
        实际检查的位形数（稠密化之后）。
    num_escapes:
        **逃逸数**（发生碰撞的位形数）。硬要求为 0。
    escape_indices:
        逃逸位形在稠密点列中的下标（升序）。
    escaped_configs:
        逃逸位形本身（numpy ``(K, 7)``），便于人工排查。
    """

    num_samples: int
    num_escapes: int
    escape_indices: Tuple[int, ...] = ()
    escaped_configs: Tuple[np.ndarray, ...] = ()

    @property
    def ok(self) -> bool:
        """是否零逃逸。"""
        return self.num_escapes == 0


# ── 区域规范化 ─────────────────────────────────────────────────────────
def _as_hpolyhedron(value: Any) -> Any:
    """把区域条目翻译成 :class:`HPolyhedron`（非法 → ``invalid_input``）。"""
    from pydrake.geometry import optimization as go

    if isinstance(value, go.HPolyhedron):
        return value
    for attr in ("hpolyhedron", "set", "polyhedron"):
        candidate = getattr(value, attr, None)
        if isinstance(candidate, go.HPolyhedron):
            return candidate
    raise GcsError(
        f"区域条目既不是 HPolyhedron，也没有 hpolyhedron 属性：{value!r}",
        reason=REASON_INVALID_INPUT,
    )


def _as_region_entry(entry: Any, index: int) -> Tuple[str, Any, Optional[np.ndarray]]:
    """把区域条目规范化为 ``(区域 id, HPolyhedron, 锚点或 None)``。

    支持的输入形态：

    * S02-T04 的 :class:`~rm75_drake.planning.regions.Region`
      （有 ``id`` / ``hpolyhedron``，且 ``seed_q`` 可作锚点）；
    * ``(id, hpolyhedron)`` 二元组；
    * ``{"id": ..., "hpolyhedron": ...}`` 映射（可选 ``"anchor"``）；
    * 裸 :class:`HPolyhedron`（自动编号 ``region-<index>``）。

    锚点（anchor）是**邻接判据用的「区域中心」**：显式给出时优先用显式值，
    否则用 :func:`region_anchor`（Chebyshev 中心）。
    """
    from pydrake.geometry import optimization as go

    if isinstance(entry, go.HPolyhedron):
        return f"region-{index}", entry, None
    if isinstance(entry, (tuple, list)) and len(entry) == 2:
        region_id, poly = entry
        return str(region_id), _as_hpolyhedron(poly), None
    if isinstance(entry, dict):
        if "hpolyhedron" not in entry:
            raise GcsError(
                f"区域映射缺少 hpolyhedron 键：{sorted(entry)}",
                reason=REASON_INVALID_INPUT,
            )
        anchor = entry.get("anchor")
        return (
            str(entry.get("id") or f"region-{index}"),
            _as_hpolyhedron(entry["hpolyhedron"]),
            None if anchor is None else np.asarray(anchor, dtype=float).reshape(-1),
        )
    region_id = getattr(entry, "id", None)
    if region_id is None:
        raise GcsError(
            f"无法确定区域 id（条目类型 {type(entry).__name__}）；"
            "请给 Region 对象、映射或 (id, hpolyhedron) 二元组",
            reason=REASON_INVALID_INPUT,
        )
    seed = getattr(entry, "seed_q", None)
    anchor = None if seed is None else np.asarray(seed, dtype=float).reshape(-1)
    return str(region_id), _as_hpolyhedron(entry), anchor


def region_anchor(polyhedron: Any) -> np.ndarray:
    """返回区域的**锚点**（Chebyshev 中心）。

    Chebyshev 中心是多面体内距边界最远的点，用作「区域中心」比顶点平均更稳定。
    退化时回落到任意可行点；再不行则回落到零向量（并记录告警）。
    """
    center = polyhedron.ChebyshevCenter()
    # Drake 版本差异（实测 1.51.1 返回 ndarray；较新版本可能返回 (point, radius)）。
    if isinstance(center, tuple) and len(center) == 2:
        center = center[0]
    point = np.asarray(center, dtype=float).reshape(-1)
    if np.all(np.isfinite(point)):
        return point
    feasible = polyhedron.MaybeGetFeasiblePoint()
    if feasible is not None:
        return np.asarray(feasible, dtype=float).reshape(-1)
    LOGGER.warning("区域既没有 Chebyshev 中心也没有可行点，锚点回落到零向量")
    return np.zeros(int(polyhedron.ambient_dimension()), dtype=float)


# ── 图构建 ─────────────────────────────────────────────────────────────
def build_graph(
    regions: Iterable[Any],
    *,
    adjacency_threshold: float = DEFAULT_ADJACENCY_THRESHOLD,
) -> Any:
    """把区域列表构造成 :class:`GraphOfConvexSets`。

    参数
    ----
    regions:
        区域条目序列（见 :func:`_as_region_entry` 支持的形态）。
        **顺序即顶点 id 的顺序**，区域 id 映射为顶点名（``AddVertex(name=...)``）。
    adjacency_threshold:
        邻接判据的阈值（rad）：两个区域的**锚点**（Chebyshev 中心）欧氏距离
        ≤ 该阈值时连一条边。阈值可配；设 ``float("inf")`` 则全连通。

    返回
    ----
    :class:`pydrake.geometry.optimization.GraphOfConvexSets`
    （顶点名 = 区域 id，边名 = ``<id_u>__<id_v>``）。

    异常
    ----
    GcsError
        区域为空（``empty_region``）、区域 id 重复或非法入参
        （``invalid_input``）。
    """
    from pydrake.geometry import optimization as go

    threshold = float(adjacency_threshold)
    if threshold <= 0.0:
        raise GcsError(
            f"adjacency_threshold 必须 > 0，实际：{adjacency_threshold!r}",
            reason=REASON_INVALID_INPUT,
        )

    entries = [_as_region_entry(entry, index) for index, entry in enumerate(regions)]
    seen = set()
    for region_id, _, _ in entries:
        if region_id in seen:
            raise GcsError(
                f"区域 id 重复：{region_id!r}（区域 id 必须唯一，否则顶点名不可解析）",
                reason=REASON_INVALID_INPUT,
            )
        seen.add(region_id)

    graph = go.GraphOfConvexSets()
    vertices = []
    anchors: List[np.ndarray] = []
    for region_id, poly, anchor in entries:
        if poly.IsEmpty():
            raise GcsError(
                f"区域 {region_id!r} 是空集，不能加入 GCS 图",
                reason=REASON_EMPTY_REGION,
            )
        vertices.append(graph.AddVertex(poly, name=region_id))
        if anchor is None:
            anchors.append(region_anchor(poly))
        elif anchor.size != poly.ambient_dimension() or not np.all(np.isfinite(anchor)):
            raise GcsError(
                f"区域 {region_id!r} 的锚点维度/有限性非法：{anchor}",
                reason=REASON_INVALID_INPUT,
            )
        elif not poly.PointInSet(anchor):
            LOGGER.warning(
                "区域 %r 的显式锚点不在区域内，改用 Chebyshev 中心", region_id
            )
            anchors.append(region_anchor(poly))
        else:
            anchors.append(anchor)

    num_edges = 0
    for i in range(len(vertices)):
        for j in range(i + 1, len(vertices)):
            distance = float(np.linalg.norm(anchors[i] - anchors[j]))
            if distance > threshold:
                continue
            graph.AddEdge(vertices[i], vertices[j], name=f"{entries[i][0]}__{entries[j][0]}")
            num_edges += 1
            LOGGER.info(
                "GCS 图连边 %s ↔ %s（锚点距离 %.4f rad ≤ 阈值 %.4f）",
                entries[i][0],
                entries[j][0],
                distance,
                threshold,
            )
    LOGGER.info(
        "GCS 图构建完成：%d 个区域、%d 条边", len(vertices), num_edges
    )
    return graph


# ── 最短路求解 ─────────────────────────────────────────────────────────
def _vertex_by_id(graph: Any, region_id: str) -> Any:
    """按名字取顶点；不存在 → ``GcsError(unknown_region)``。"""
    try:
        vertex = graph.GetVertexByName(str(region_id))
    except Exception as exc:  # pragma: no cover - Drake 内部异常
        raise GcsError(
            f"查询区域 {region_id!r} 的顶点失败：{exc}",
            reason=REASON_UNKNOWN_REGION,
        ) from exc
    if vertex is None:
        known = sorted(v.name() for v in graph.Vertices())
        raise GcsError(
            f"图里没有区域 {region_id!r}；已知区域：{known}",
            reason=REASON_UNKNOWN_REGION,
        )
    return vertex


def _is_missing_integer_solver(message: str) -> bool:
    """判断 RuntimeError 是否是「缺混合整数求解器」导致的。"""
    lowered = message.lower()
    return "mixed-integer" in lowered or "no solver available" in lowered


def _edge_points(edges: Sequence[Any], solution: Any) -> Tuple[Tuple[np.ndarray, np.ndarray], ...]:
    """提取每条边两端的数值解 ``(xu, xv)``。

    **实测坑**（Drake 1.51.1）：``edge.xu()`` / ``edge.xv()`` 是**方法**不是属性，
    返回的是决策变量索引数组；数值要用 ``result.GetSolution(...)`` 取。
    """
    points: List[Tuple[np.ndarray, np.ndarray]] = []
    for edge in edges:
        xu = np.asarray(solution.GetSolution(edge.xu()), dtype=float).reshape(-1)
        xv = np.asarray(solution.GetSolution(edge.xv()), dtype=float).reshape(-1)
        points.append((xu, xv))
    return tuple(points)


def solve_path(
    graph: Any,
    src_region: str,
    dst_region: str,
    *,
    max_rounded_paths: int = DEFAULT_MAX_ROUNDED_PATHS,
    prefer_integer: bool = True,
) -> GcsResult:
    """在凸集图上求最短路。

    参数
    ----
    graph:
        :func:`build_graph` 的产物。
    src_region / dst_region:
        起点 / 终点的**区域 id**（= 顶点名）。不存在 → ``GcsError``。
    max_rounded_paths:
        凸松弛回退时的 ``max_rounded_paths``（舍入出整数可行路径的尝试次数）。
    prefer_integer:
        是否优先尝试整数版 GCS（需要 Mosek/Gurobi/CPLEX）。
        整数版不可用时**自动回退**凸松弛，并把模式记进结果。

    返回
    ----
    :class:`GcsResult`

    异常
    ----
    GcsError
        区域 id 不存在（``unknown_region``）、入参非法（``invalid_input``）。
    """
    from pydrake.geometry import optimization as go

    if not isinstance(max_rounded_paths, int) or isinstance(max_rounded_paths, bool):
        raise GcsError(
            f"max_rounded_paths 必须是整数，实际：{max_rounded_paths!r}",
            reason=REASON_INVALID_INPUT,
        )
    if max_rounded_paths < 0:
        raise GcsError(
            f"max_rounded_paths 必须 ≥ 0，实际：{max_rounded_paths}",
            reason=REASON_INVALID_INPUT,
        )

    source = _vertex_by_id(graph, src_region)
    target = _vertex_by_id(graph, dst_region)

    integer_error: Optional[str] = None
    if prefer_integer:
        options = go.GraphOfConvexSetsOptions()
        options.convex_relaxation = False
        try:
            result = graph.SolveShortestPath(source, target, options)
        except RuntimeError as exc:
            integer_error = str(exc)
            if not _is_missing_integer_solver(integer_error):
                raise GcsError(
                    "整数版 GCS 求解失败（不是缺少求解器）：" + integer_error,
                    reason=REASON_SOLVER_UNAVAILABLE,
                ) from exc
            LOGGER.warning(
                "整数版 GCS 不可用，回退凸松弛：%s", integer_error.splitlines()[0]
            )
        else:
            if not result.is_success():
                return GcsResult(
                    success=False,
                    message=(
                        f"起点区域 {src_region!r} 与终点区域 {dst_region!r} 之间"
                        "无可行边路径（整数版 GCS 求解未成功）"
                    ),
                    reason=REASON_NO_PATH,
                    solver_mode=SOLVER_MODE_MIXED_INTEGER,
                    result=result,
                )
            edges = tuple(graph.GetSolutionPath(source, target, result))
            return GcsResult(
                success=True,
                message=(
                    f"整数版 GCS 求解成功：{len(edges)} 条边"
                    f"（{src_region!r} → {dst_region!r}）"
                ),
                solver_mode=SOLVER_MODE_MIXED_INTEGER,
                edges=edges,
                edge_points=_edge_points(edges, result),
                result=result,
            )

    # 凸松弛回退（无 Mosek 许可时的唯一可用路径）。
    options = go.GraphOfConvexSetsOptions()
    options.convex_relaxation = True
    options.max_rounded_paths = int(max_rounded_paths)
    try:
        result = graph.SolveShortestPath(source, target, options)
    except RuntimeError as exc:
        raise GcsError(
            "GCS 求解器不可用（整数版与凸松弛都失败）：" + str(exc),
            reason=REASON_SOLVER_UNAVAILABLE,
        ) from exc
    if not result.is_success():
        detail = integer_error.splitlines()[0] if integer_error else ""
        return GcsResult(
            success=False,
            message=(
                f"起点区域 {src_region!r} 与终点区域 {dst_region!r} 之间"
                "无可行边路径（凸松弛求解未成功）"
                + (f"；整数版不可用原因：{detail}" if detail else "")
            ),
            reason=REASON_NO_PATH,
            solver_mode=SOLVER_MODE_CONVEX_RELAXATION,
            result=result,
        )
    try:
        edges = tuple(graph.GetSolutionPath(source, target, result))
    except RuntimeError as exc:
        raise GcsError(
            "GCS 求解成功但无法提取路径：" + str(exc),
            reason=REASON_SOLVER_UNAVAILABLE,
        ) from exc
    LOGGER.info(
        "凸松弛 GCS 求解成功：%d 条边（%s → %s）", len(edges), src_region, dst_region
    )
    return GcsResult(
        success=True,
        message=(
            f"凸松弛 GCS 求解成功：{len(edges)} 条边"
            f"（{src_region!r} → {dst_region!r}）"
        ),
        solver_mode=SOLVER_MODE_CONVEX_RELAXATION,
        edges=edges,
        edge_points=_edge_points(edges, result),
        result=result,
    )


# ── 轨迹生成 ───────────────────────────────────────────────────────────
def _densify(waypoints: np.ndarray, step_size: float) -> np.ndarray:
    """把角点序列稠密化为相邻点距离 ≤ ``step_size`` 的点列。"""
    if waypoints.shape[0] == 0:
        return np.zeros((0, waypoints.shape[1]), dtype=float)
    rows: List[np.ndarray] = [waypoints[0]]
    for index in range(1, waypoints.shape[0]):
        start = waypoints[index - 1]
        end = waypoints[index]
        distance = float(np.max(np.abs(end - start)))
        segments = max(1, int(np.ceil(distance / step_size)))
        for k in range(1, segments + 1):
            ratio = k / segments
            rows.append(start + ratio * (end - start))
    return np.vstack(rows)


def path_to_q_sequence(
    edges: Sequence[Any],
    *,
    edge_points: Optional[Sequence[Tuple[np.ndarray, np.ndarray]]] = None,
    solution: Any = None,
    step_size: float = DEFAULT_STEP_SIZE,
) -> np.ndarray:
    """把解路径转成**稠密的关节角点列**（分段线性插值）。

    参数
    ----
    edges:
        路径上的边序列（:class:`GcsResult.edges`）。
    edge_points:
        每条边两端的数值解 ``(xu, xv)``（:class:`GcsResult.edge_points`）。
        给出时优先使用；否则需要 ``solution``（``MathematicalProgramResult``）。
    solution:
        用于从 ``edge.xu()`` / ``edge.xv()`` 取值的结果对象。
    step_size:
        稠密化步长（rad，C 空间无穷范数）。

    返回
    ----
    ``(N, 7)`` 的 numpy 数组（首点 = 起点区域解点，末点 = 终点区域解点）。
    """
    step = float(step_size)
    if not np.isfinite(step) or step <= 0.0:
        raise GcsError(
            f"step_size 必须是正有限值，实际：{step_size!r}", reason=REASON_INVALID_INPUT
        )
    edge_list = list(edges)
    if not edge_list:
        raise GcsError("边序列为空，无法生成 q 序列", reason=REASON_INVALID_INPUT)

    if edge_points is None:
        if solution is None:
            raise GcsError(
                "缺少 edge_points 或 solution，无法取边上的解点",
                reason=REASON_INVALID_INPUT,
            )
        edge_points = _edge_points(edge_list, solution)
    points = list(edge_points)
    if len(points) != len(edge_list):
        raise GcsError(
            f"edge_points 数量（{len(points)}）与边数量（{len(edge_list)}）不一致",
            reason=REASON_INVALID_INPUT,
        )

    waypoints: List[np.ndarray] = []
    for index, (xu, xv) in enumerate(points):
        xu = np.asarray(xu, dtype=float).reshape(-1)
        xv = np.asarray(xv, dtype=float).reshape(-1)
        if index == 0:
            waypoints.append(xu)
        waypoints.append(xv)
    matrix = np.vstack(waypoints)
    if not np.all(np.isfinite(matrix)):
        raise GcsError(
            "解路径含非有限值（无法生成 q 序列）", reason=REASON_INVALID_INPUT
        )
    # 连续性自检：相邻边共享顶点的解点应一致（Drake 的 GCS 强制连续）。
    for index in range(1, len(points)):
        previous = np.asarray(points[index - 1][1], dtype=float).reshape(-1)
        current = np.asarray(points[index][0], dtype=float).reshape(-1)
        gap = float(np.max(np.abs(previous - current)))
        if gap > 1e-6:
            LOGGER.warning(
                "相邻边在共享顶点处不连续（偏差 %.3e），按后一条边的解点续接", gap
            )
    return _densify(matrix, step)


# ── 碰撞复检 ───────────────────────────────────────────────────────────
def collision_recheck(
    scene: Any,
    q_seq: Sequence[Sequence[float]],
    *,
    edge_step_size: float = DEFAULT_EDGE_STEP_SIZE,
) -> RecheckResult:
    """对 q 点列做**密采样碰撞复检**（零逃逸是硬要求）。

    参数
    ----
    scene:
        :class:`rm75_drake.planning.scene.SceneBundle`（提供 ``checker``）。
    q_seq:
        关节角点列（``(N, 7)``）。
    edge_step_size:
        复检时的密采样步长（rad）：相邻点距离超过该步长就再插值。

    返回
    ----
    :class:`RecheckResult`（``num_escapes`` 必须为 0 才算通过）。
    """
    step = float(edge_step_size)
    if not np.isfinite(step) or step <= 0.0:
        raise GcsError(
            f"edge_step_size 必须是正有限值，实际：{edge_step_size!r}",
            reason=REASON_INVALID_INPUT,
        )
    matrix = np.asarray(q_seq, dtype=float)
    if matrix.ndim != 2 or matrix.shape[1] != JOINT_COUNT:
        raise GcsError(
            f"q_seq 必须是 (N, {JOINT_COUNT}) 的矩阵，实际形状：{matrix.shape}",
            reason=REASON_INVALID_INPUT,
        )
    if matrix.shape[0] == 0:
        raise GcsError("q_seq 为空，无法复检", reason=REASON_INVALID_INPUT)
    if not np.all(np.isfinite(matrix)):
        raise GcsError("q_seq 含 NaN/Inf，无法复检", reason=REASON_INVALID_INPUT)

    checker = getattr(scene, "checker", None)
    if checker is None:
        raise GcsError(
            "场景没有 checker（请用 build_scene 装配场景）",
            reason=REASON_INVALID_INPUT,
        )

    dense = _densify(matrix, step)
    configs = [np.asarray(row, dtype=float) for row in dense]
    flags = checker.CheckConfigsCollisionFree(configs)
    escapes = [index for index, flag in enumerate(flags) if not int(flag)]
    return RecheckResult(
        num_samples=len(configs),
        num_escapes=len(escapes),
        escape_indices=tuple(escapes),
        escaped_configs=tuple(dense[index] for index in escapes),
    )


# ── 一站式规划 ─────────────────────────────────────────────────────────
def plan_q_sequence(
    graph: Any,
    src_region: str,
    dst_region: str,
    *,
    step_size: float = DEFAULT_STEP_SIZE,
    max_rounded_paths: int = DEFAULT_MAX_ROUNDED_PATHS,
) -> Tuple[np.ndarray, GcsResult]:
    """求最短路并直接产出稠密 q 点列。

    返回 ``(q_sequence, gcs_result)``。求解失败（``success=False``）时
    ``q_sequence`` 为空 ``(0, 7)`` 数组。
    """
    result = solve_path(
        graph,
        src_region,
        dst_region,
        max_rounded_paths=max_rounded_paths,
    )
    if not result.success:
        return np.zeros((0, JOINT_COUNT), dtype=float), result
    q_sequence = path_to_q_sequence(
        result.edges,
        edge_points=result.edge_points,
        step_size=step_size,
    )
    return q_sequence, result
