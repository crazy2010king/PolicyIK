"""空间 GCS 最短路 + 轨迹生成 + 碰撞复检测试（S02-T05）。

被测对象
--------
:mod:`rm75_drake.planning.gcs` —— 凸集图最短路（**不是**轨迹级 GCS）+ 关节空间
折线生成 + 密采样碰撞复检。

覆盖的契约
----------
**正例**
* 链式图（3 个区域）可解：边序列非空、**相邻边共享顶点**（首尾顶点衔接）；
* 求解器模式被记录（``mixed_integer`` 或 ``convex_relaxation``）；
* 由边解点插值出的 q 序列**全部落在软件软限位内**；
* 沿轨迹密采样碰撞复检**逃逸数 = 0**；
* **IRIS 真区域**（S02-T04 的种子约定）同样走通全链路（``@pytest.mark.slow``）。

**负例（可解释失败）**
* 起点/终点之间无可行边路径 → ``success=False`` + ``reason="no_path"``，
  message 明说「无可行边路径」；
* 区域 id 不存在 → ``GcsError(reason="unknown_region")``；
* 区域为空 → ``GcsError(reason="empty_region")``；
* 区域 id 重复 / 非法步长 / 非法 q 序列 → ``GcsError(reason="invalid_input")``；
* 碰撞复检能**如实报出逃逸位形**（不是恒零的假通过）。

耗时说明
--------
IRIS 单区域在本机耗时 1.5–18 s（Drake 1.51.1，取决于认证强度）。IRIS 相关
用例统一打 ``@pytest.mark.slow``；其余用例用**合成凸盒**（秒级）覆盖同一套契约。
"""

from __future__ import annotations

import logging

import numpy as np
import pytest

from rm75_drake.planning import gcs as gcs_mod
from rm75_drake.planning import regions as regions_mod

logging.getLogger("drake").setLevel(logging.WARNING)

#: 种子 A（S02-T04 测试用的抬肘名义姿态，碰撞自由）。
SEED_A = (0.0, -0.6, 0.0, 0.9, 0.0, 0.5, 0.0)

#: 链式图的三个种子：沿 joint_1 方向小幅偏移（保持在 SEED_A 的自由邻域内）。
CHAIN_SEEDS = (
    SEED_A,
    tuple(v + (0.04 if i == 0 else 0.0) for i, v in enumerate(SEED_A)),
    tuple(v + (0.08 if i == 0 else 0.0) for i, v in enumerate(SEED_A)),
)

#: 合成凸盒的半宽（rad）——刻意做小，保证插值轨迹停留在碰撞自由邻域内。
BOX_HALF_WIDTH = 0.01

#: 碰撞种子候选（与 S02-T04 测试同源，用于「复检能报出逃逸」用例）。
COLLIDING_CANDIDATES = (
    (0.7596, 1.747, 1.6741, -1.2565, -1.2135, 1.6169, -6.1479),
    (-2.8519, -1.6586, 2.8367, 0.7213, -0.4359, 0.1028, 4.6328),
)


def _box(center, half_width: float = BOX_HALF_WIDTH):
    """返回一个以 ``center`` 为中心的小凸盒（HPolyhedron）。"""
    from pydrake.geometry import optimization as go

    c = np.asarray(center, dtype=float)
    h = np.full(c.size, float(half_width))
    return go.HPolyhedron.MakeBox(c - h, c + h)


def _chain_regions():
    """三个合成区域（id = r0/r1/r2），沿 joint_1 方向依次偏移。"""
    return [
        (f"r{index}", _box(seed))
        for index, seed in enumerate(CHAIN_SEEDS)
    ]


# ── 正例 ────────────────────────────────────────────────────────────────
def test_chain_graph_solves_with_connected_edges():
    """正例：链式图可解 → 边序列非空、相邻边共享顶点、首尾衔接。"""
    graph = gcs_mod.build_graph(_chain_regions(), adjacency_threshold=0.2)
    result = gcs_mod.solve_path(graph, "r0", "r2")

    assert result.success, result.message
    assert result.num_edges >= 1
    assert result.reason == ""
    assert result.solver_mode in (
        gcs_mod.SOLVER_MODE_MIXED_INTEGER,
        gcs_mod.SOLVER_MODE_CONVEX_RELAXATION,
    ), "求解模式必须被记录（整数版 / 凸松弛）"

    # 相邻边共享顶点：第 k 条边的终点 == 第 k+1 条边的起点。
    for index in range(1, len(result.edges)):
        previous = result.edges[index - 1]
        current = result.edges[index]
        assert previous.v().name() == current.u().name()
    # 首尾分别是起点/终点区域。
    assert result.edges[0].u().name() == "r0"
    assert result.edges[-1].v().name() == "r2"


def test_interpolated_q_sequence_respects_software_limits():
    """正例：插值出的 q 序列全部落在软件软限位内。"""
    graph = gcs_mod.build_graph(_chain_regions(), adjacency_threshold=0.2)
    q_sequence, result = gcs_mod.plan_q_sequence(graph, "r0", "r2", step_size=0.005)
    assert result.success
    assert q_sequence.ndim == 2 and q_sequence.shape[1] == 7
    assert q_sequence.shape[0] >= 2

    from rm75_drake.planning.trajectory import software_limits

    limits = software_limits()
    lower = np.asarray(limits.lower, dtype=float)
    upper = np.asarray(limits.upper, dtype=float)
    assert np.all(q_sequence >= lower - 1e-9)
    assert np.all(q_sequence <= upper + 1e-9)


def test_collision_recheck_reports_zero_escapes(single_arm_scene):
    """正例：沿轨迹密采样碰撞复检逃逸数 = 0。"""
    graph = gcs_mod.build_graph(_chain_regions(), adjacency_threshold=0.2)
    q_sequence, result = gcs_mod.plan_q_sequence(graph, "r0", "r2", step_size=0.005)
    assert result.success

    recheck = gcs_mod.collision_recheck(
        single_arm_scene, q_sequence, edge_step_size=0.01
    )
    assert recheck.num_escapes == 0, (
        f"碰撞复检出现逃逸：{recheck.escape_indices} / {recheck.num_samples}"
    )
    assert recheck.ok
    assert recheck.num_samples >= q_sequence.shape[0]


@pytest.mark.slow
def test_iris_regions_solve_and_recheck_without_escape(single_arm_scene):
    """正例：**真实 IRIS 区域**（S02-T04 的种子约定）走通全链路。"""
    specs = [
        regions_mod.RegionSpec(
            seed_q=tuple(seed), num_collision_infeasible_samples=1
        )
        for seed in CHAIN_SEEDS
    ]
    region_list = [
        regions_mod.generate_region(single_arm_scene, spec.seed_q, spec=spec)
        for spec in specs
    ]
    graph = gcs_mod.build_graph(region_list, adjacency_threshold=0.2)
    q_sequence, result = gcs_mod.plan_q_sequence(graph, region_list[0].id, region_list[2].id)
    assert result.success, result.message
    assert result.num_edges >= 1

    from rm75_drake.planning.trajectory import software_limits

    limits = software_limits()
    lower = np.asarray(limits.lower, dtype=float)
    upper = np.asarray(limits.upper, dtype=float)
    assert np.all(q_sequence >= lower - 1e-9)
    assert np.all(q_sequence <= upper + 1e-9)

    recheck = gcs_mod.collision_recheck(single_arm_scene, q_sequence)
    assert recheck.num_escapes == 0, (
        f"IRIS 区域路径出现逃逸：{recheck.escape_indices}"
    )


# ── 负例：可解释失败 ────────────────────────────────────────────────────
def test_no_edge_path_is_reported_explainably():
    """负例：起点/终点之间无可行边路径 → success=False + 可解释 message。"""
    far = [
        ("r0", _box(SEED_A)),
        ("r1", _box(tuple(v + 0.5 if i == 0 else v for i, v in enumerate(SEED_A)))),
        ("r2", _box(tuple(v + 1.0 if i == 0 else v for i, v in enumerate(SEED_A)))),
    ]
    # 阈值 0.2 < 相邻锚点距离 0.5 → 图里没有任何边。
    graph = gcs_mod.build_graph(far, adjacency_threshold=0.2)
    result = gcs_mod.solve_path(graph, "r0", "r2")

    assert not result.success
    assert result.reason == gcs_mod.REASON_NO_PATH
    assert "无可行边路径" in result.message
    assert result.num_edges == 0


def test_unknown_region_id_raises():
    """负例：区域 id 不存在 → 显式报错（unknown_region）。"""
    graph = gcs_mod.build_graph(_chain_regions(), adjacency_threshold=0.2)
    with pytest.raises(gcs_mod.GcsError) as excinfo:
        gcs_mod.solve_path(graph, "no-such-region", "r2")
    assert excinfo.value.reason == gcs_mod.REASON_UNKNOWN_REGION


def test_empty_region_is_rejected():
    """负例：空区域 → 显式报错（empty_region）。"""
    from pydrake.geometry import optimization as go

    empty = go.HPolyhedron(np.zeros((1, 7)), np.asarray([-1.0]))
    assert empty.IsEmpty()
    with pytest.raises(gcs_mod.GcsError) as excinfo:
        gcs_mod.build_graph([("empty", empty)])
    assert excinfo.value.reason == gcs_mod.REASON_EMPTY_REGION


def test_duplicate_region_ids_are_rejected():
    """负例：区域 id 重复 → 显式报错（invalid_input）。"""
    with pytest.raises(gcs_mod.GcsError) as excinfo:
        gcs_mod.build_graph([("r0", _box(SEED_A)), ("r0", _box(CHAIN_SEEDS[1]))])
    assert excinfo.value.reason == gcs_mod.REASON_INVALID_INPUT


@pytest.mark.parametrize(
    "kwargs",
    [
        {"adjacency_threshold": 0.0},
        {"adjacency_threshold": -1.0},
    ],
)
def test_invalid_graph_options_are_rejected(kwargs):
    """负例：非法建图参数 → 显式报错（invalid_input）。"""
    with pytest.raises(gcs_mod.GcsError) as excinfo:
        gcs_mod.build_graph(_chain_regions(), **kwargs)
    assert excinfo.value.reason == gcs_mod.REASON_INVALID_INPUT


def test_invalid_step_sizes_are_rejected():
    """负例：非法步长 → 显式报错（invalid_input）。"""
    graph = gcs_mod.build_graph(_chain_regions(), adjacency_threshold=0.2)
    result = gcs_mod.solve_path(graph, "r0", "r2")
    assert result.success
    with pytest.raises(gcs_mod.GcsError) as excinfo:
        gcs_mod.path_to_q_sequence(
            result.edges, edge_points=result.edge_points, step_size=0.0
        )
    assert excinfo.value.reason == gcs_mod.REASON_INVALID_INPUT

    with pytest.raises(gcs_mod.GcsError) as excinfo:
        gcs_mod.path_to_q_sequence(result.edges)
    assert excinfo.value.reason == gcs_mod.REASON_INVALID_INPUT


@pytest.mark.parametrize(
    "bad_seq",
    [
        np.zeros((3, 6)),          # 维度不对
        np.zeros((0, 7)),          # 空序列
        np.full((2, 7), np.nan),   # 含 NaN
    ],
)
def test_collision_recheck_rejects_malformed_sequences(single_arm_scene, bad_seq):
    """负例：非法 q 序列 → 显式报错（invalid_input），不是静默通过。"""
    with pytest.raises(gcs_mod.GcsError) as excinfo:
        gcs_mod.collision_recheck(single_arm_scene, bad_seq)
    assert excinfo.value.reason == gcs_mod.REASON_INVALID_INPUT


def test_collision_recheck_detects_escapes(single_arm_scene):
    """负例：碰撞复检能**如实报出**逃逸位形（不是恒零的假通过）。"""
    colliding = None
    for candidate in COLLIDING_CANDIDATES:
        values = np.asarray(candidate, dtype=float)
        if not single_arm_scene.checker.CheckConfigCollisionFree(values):
            colliding = values
            break
    assert colliding is not None, "找不到碰撞位形（场景变了？）"

    recheck = gcs_mod.collision_recheck(single_arm_scene, [colliding])
    assert recheck.num_escapes == 1
    assert recheck.escape_indices == (0,)
    assert not recheck.ok
    assert recheck.escaped_configs[0].shape == (7,)


def test_module_docstring_records_gcs_scope():
    """契约：模块 docstring 必须写明「GCS = 空间凸集图最短路，不是轨迹级 GCS」。"""
    doc = gcs_mod.__doc__ or ""
    assert "空间凸集图最短路" in doc
    assert "不是轨迹级 GCS" in doc
