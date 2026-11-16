"""最小坐标（8 维）凸区域生成 + 缓存的可执行断言（S05-T04）。

覆盖
----
* **区域包含种子 + 维度 = 8**；
* **保守性**：区域内取样点映射回 14 维后**全部碰撞自由**
  （用 S04 的**同一个 plant** 判定）；
* **规格可复现**：同种子 + 同参数两次生成结果**逐位相同**；
* **缓存行为**：首次生成 → 落盘 → 再查**命中**（命中计数 +1）；
  ``model_version`` 不匹配 ⇒ **版本门禁生效**（明确 miss 并重新生成，
  不用陈旧区域）；
* **降级路径**：无法生成区域的场景 ⇒ 走采样基线且 ``reason`` / ``stage``
  **显式标注降级**（不静默降级）；
* **失败路径反例**：维度错 / 非法 oracle / 缓存损坏 / 空区域 ⇒ **明确 reason**；
* **零 pydrake 依赖**（模块级）：``import rm75_drake.coordinated.regions``
  不把 pydrake 拉进 ``sys.modules``；
* **oracle 接口是纯 numpy**（一批 x → free/busy 布尔数组，无 Drake 类型）。
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
DUAL_SCENARIO_PATH = REPO_ROOT / "scenarios" / "dual_arm.yaml"
REGIONS_PATH = REPO_ROOT / "src" / "rm75_drake" / "coordinated" / "regions.py"

#: 相对变换：follower 的 TCP 在 leader 的 TCP 系里沿 +x 偏 0.2 m。
X_LR = np.eye(4)
X_LR[0, 3] = 0.2

#: 已知**可行**的 8 维种子（与 tests/test_coordinated_parameterization.py 同源）。
SEED_X = np.array(
    [
        0.071785, 1.981218, -2.160796, 2.051393, -1.142630, -0.331872, 4.072276,
        2.571074,
    ]
)


# ── fixture ─────────────────────────────────────────────────────────────
@pytest.fixture(scope="module")
def dual_scene():
    """双臂（不含夹爪）的单 plant 场景（S04 的装配器）。"""
    from rm75_drake.bimanual.dual_scene import build_dual_scene_from_path

    return build_dual_scene_from_path(DUAL_SCENARIO_PATH)


@pytest.fixture(scope="module")
def parameterization(dual_scene):
    """参数化实例（模块级共享 ⇒ 分支状态稳定）。"""
    from rm75_drake.coordinated.parameterization import CoordinatedParameterization

    return CoordinatedParameterization(dual_scene, x_lr=X_LR)


@pytest.fixture(scope="module")
def oracle(parameterization):
    """最小坐标 oracle（纯 numpy 接口）。"""
    from rm75_drake.coordinated.regions import make_parameterization_oracle

    return make_parameterization_oracle(parameterization)


@pytest.fixture(scope="module")
def spec():
    """区域规格（确定性）。"""
    from rm75_drake.coordinated.regions import MinimalRegionSpec

    return MinimalRegionSpec(
        seed_x=tuple(float(v) for v in SEED_X),
        random_seed=4242,
        step=0.05,
        max_expand=8,
        margin=0.02,
        num_validation_samples=24,
        max_rounds=6,
    )


@pytest.fixture(scope="module")
def region(spec, oracle, dual_scene):
    """真实场景下生成的一个区域（模块级共享，避免重复生成开销）。"""
    from rm75_drake.coordinated.regions import (
        generate_minimal_region,
        minimal_model_version,
    )

    return generate_minimal_region(
        spec, oracle, model_version=minimal_model_version(dual_scene)
    )


# ── 区域本体 ───────────────────────────────────────────────────────────
def test_region_contains_seed_and_is_eight_dimensional(region, spec):
    """区域必须包含种子，且**恒为 8 维**。"""
    assert region.ambient_dimension == 8
    assert region.hpolyhedron.ambient_dimension() == 8
    assert not region.is_empty
    assert region.contains(np.asarray(spec.seed_x, dtype=float))
    assert region.num_constraints > 0
    assert np.asarray(region.a).shape[1] == 8


def test_region_spec_dimension_is_enforced():
    """维度纪律：非 8 维的种子 / spec ⇒ 显式报错并点名期望 / 实际。"""
    from rm75_drake.coordinated.regions import MinimalRegionError, MinimalRegionSpec

    with pytest.raises(MinimalRegionError) as excinfo:
        MinimalRegionSpec(seed_x=(0.0,) * 7)
    assert excinfo.value.reason == "invalid_spec"
    assert "8" in str(excinfo.value)

    with pytest.raises(MinimalRegionError) as excinfo:
        MinimalRegionSpec(seed_x=(0.0,) * 8, dim=7)
    assert excinfo.value.reason == "invalid_spec"


def test_region_is_conservative_in_dual_scene(region, spec, parameterization, dual_scene):
    """保守性：区域内取样点映射回 14 维后**全部碰撞自由**（同一个 plant）。"""
    from rm75_drake.bimanual.dual_scene import check_config_free

    samples = region.sample(24, seed=7)
    assert len(samples) == 24
    for point in samples:
        assert region.contains(point)
        q14 = parameterization.to_joint_config(point)
        assert parameterization.is_collision_free(q14)
        # 同一个 plant 的独立判定（不靠参数化内部缓存）。
        assert check_config_free(dual_scene, q14)


def test_region_generation_is_reproducible(spec, oracle, dual_scene):
    """同种子 + 同参数两次生成 ⇒ 结果逐位相同。"""
    from rm75_drake.coordinated.regions import (
        generate_minimal_region,
        minimal_model_version,
    )

    version = minimal_model_version(dual_scene)
    first = generate_minimal_region(spec, oracle, model_version=version)
    second = generate_minimal_region(spec, oracle, model_version=version)
    assert first.id == second.id
    assert np.array_equal(np.asarray(first.a), np.asarray(second.a))
    assert np.array_equal(np.asarray(first.b), np.asarray(second.b))


def test_region_graph_entry_is_compatible_with_gcs(region):
    """区域可直接喂给 ``planning.gcs.build_graph``（(id, HPolyhedron) 元组）。"""
    from rm75_drake.planning.gcs import build_graph

    entry = region.as_graph_entry()
    assert isinstance(entry, tuple) and len(entry) == 2
    graph = build_graph([entry])
    assert graph.num_vertices() == 1


# ── 缓存 ───────────────────────────────────────────────────────────────
def test_cache_roundtrip_hits_on_second_lookup(spec, oracle, tmp_path, dual_scene):
    """首次生成 → 落盘 → 再查命中（命中计数 +1，且不再生成）。"""
    from rm75_drake.coordinated.regions import (
        MinimalRegionCache,
        generate_minimal_region,
        minimal_model_version,
    )

    version = minimal_model_version(dual_scene)
    cache = MinimalRegionCache(model_version=version)
    generated = cache.get(
        spec, lambda s: generate_minimal_region(s, oracle, model_version=version)
    )
    assert cache.generation_count == 1
    assert cache.hit_count == 0

    path = cache.save_cache(tmp_path / "minimal_regions.yaml")
    assert path.is_file()

    restored = MinimalRegionCache.load_cache(path, model_version=version)
    assert restored.version_mismatch is False
    assert len(restored.specs) == 1
    fetched = restored.get(
        spec, lambda s: generate_minimal_region(s, oracle, model_version=version)
    )
    assert restored.hit_count == 1
    assert restored.generation_count == 0
    assert np.array_equal(np.asarray(fetched.a), np.asarray(generated.a))


def test_cache_version_gate_rejects_stale_regions(spec, oracle, tmp_path, dual_scene):
    """``model_version`` 不匹配 ⇒ 版本门禁生效：明确 miss 并重新生成。"""
    from rm75_drake.coordinated.regions import (
        MinimalRegionCache,
        generate_minimal_region,
        minimal_model_version,
    )

    version = minimal_model_version(dual_scene)
    cache = MinimalRegionCache(model_version=version)
    cache.get(spec, lambda s: generate_minimal_region(s, oracle, model_version=version))
    path = cache.save_cache(tmp_path / "stale_regions.yaml")

    stale = MinimalRegionCache.load_cache(path, model_version="0" * 64)
    assert stale.version_mismatch is True
    assert stale.stale_entries_dropped == 1
    assert len(stale.regions()) == 0

    regenerated = stale.get(
        spec, lambda s: generate_minimal_region(s, oracle, model_version=version)
    )
    assert stale.generation_count == 1
    assert stale.hit_count == 0
    assert regenerated.contains(np.asarray(spec.seed_x, dtype=float))


def test_cache_strict_version_gate_raises(spec, oracle, tmp_path, dual_scene):
    """``strict=True`` 时版本不匹配 ⇒ 显式抛错（``version_mismatch``）。"""
    from rm75_drake.coordinated.regions import (
        MinimalRegionCache,
        MinimalRegionError,
        generate_minimal_region,
        minimal_model_version,
    )

    version = minimal_model_version(dual_scene)
    cache = MinimalRegionCache(model_version=version)
    cache.get(spec, lambda s: generate_minimal_region(s, oracle, model_version=version))
    path = cache.save_cache(tmp_path / "strict_regions.yaml")

    with pytest.raises(MinimalRegionError) as excinfo:
        MinimalRegionCache.load_cache(path, model_version="1" * 64, strict=True)
    assert excinfo.value.reason == "version_mismatch"


def test_cache_missing_file_is_explicit(tmp_path):
    """缓存文件缺失 ⇒ 明确 ``cache_corrupt``（不静默新建）。"""
    from rm75_drake.coordinated.regions import MinimalRegionCache, MinimalRegionError

    with pytest.raises(MinimalRegionError) as excinfo:
        MinimalRegionCache.load_cache(tmp_path / "nope.yaml")
    assert excinfo.value.reason == "cache_corrupt"


# ── 降级链 ─────────────────────────────────────────────────────────────
def test_degradation_to_sampling_baseline_is_explicit(spec):
    """无法生成区域 ⇒ 走采样基线，且 ``reason`` / ``stage`` **显式标注降级**。"""
    from rm75_drake.coordinated.regions import (
        generate_region_with_fallback,
    )

    def busy_oracle(x_batch):
        x = np.asarray(x_batch, dtype=float).reshape(-1, 8)
        return np.zeros(len(x), dtype=bool)

    result = generate_region_with_fallback(spec, busy_oracle)
    assert result.success is False
    assert result.degraded is True
    assert result.stage == "sample_baseline"
    assert result.reason == "seed_infeasible"
    assert result.region is not None
    assert result.region.is_empty is True
    assert "sample" in result.message or "采样" in result.message


def test_degradation_reports_sampling_failure(spec):
    """区域生成失败 + 采样基线也失败 ⇒ reason 指向采样失败（原因链完整）。"""
    from rm75_drake.coordinated.regions import generate_region_with_fallback

    seed = np.asarray(spec.seed_x, dtype=float)

    def picky_oracle(x_batch):
        x = np.asarray(x_batch, dtype=float).reshape(-1, 8)
        free = np.zeros(len(x), dtype=bool)
        for index, point in enumerate(x):
            if np.allclose(point, seed) or np.allclose(point, seed + 0.01):
                free[index] = True
        return free

    result = generate_region_with_fallback(
        spec, picky_oracle, goal_x=tuple(seed + 0.01)
    )
    assert result.success is False
    assert result.degraded is True
    assert result.stage == "sample_baseline"
    assert result.reason == "sampling_failed"
    assert result.region is not None and result.region.is_empty is True


def test_degradation_to_sampling_baseline_produces_path(spec):
    """区域生成失败（无体积）⇒ 采样基线给出路径，且降级**显式**。"""
    from rm75_drake.coordinated.regions import generate_region_with_fallback

    seed = np.asarray(spec.seed_x, dtype=float)
    goal = seed + np.array([0.05, 0.02, -0.01, 0.03, 0.0, 0.01, -0.02, 0.04])
    direction = goal - seed
    length = float(np.linalg.norm(direction))
    unit = direction / length

    def tube_oracle(x_batch):
        """只有「起点→终点」这条细管内的点才可行。"""
        x = np.asarray(x_batch, dtype=float).reshape(-1, 8)
        free = np.zeros(len(x), dtype=bool)
        for index, point in enumerate(x):
            offset = point - seed
            along = float(np.dot(offset, unit))
            clamped = min(max(along, 0.0), length)
            closest = seed + clamped * unit
            if float(np.linalg.norm(point - closest)) <= 0.01:
                free[index] = True
        return free

    result = generate_region_with_fallback(spec, tube_oracle, goal_x=tuple(goal))
    assert result.success is False
    assert result.degraded is True
    assert result.stage == "sample_baseline"
    assert result.reason in ("empty_region", "conservatism_failed")
    assert result.path is not None and len(result.path) >= 2
    assert np.allclose(result.path[0], seed)
    assert np.allclose(result.path[-1], goal)
    # 采样基线的每个路径点都必须是**可行点**（同一套 oracle）。
    assert np.all(tube_oracle(result.path))


def test_sampling_baseline_reports_failure_reason(spec):
    """采样基线不可连通 ⇒ 显式 ``sampling_failed``。"""
    from rm75_drake.coordinated.regions import (
        MinimalRegionError,
        sample_baseline_path,
    )

    seed = np.asarray(spec.seed_x, dtype=float)

    def picky_oracle(x_batch):
        x = np.asarray(x_batch, dtype=float).reshape(-1, 8)
        free = np.zeros(len(x), dtype=bool)
        for index, point in enumerate(x):
            if np.allclose(point, seed) or np.allclose(point, seed + 0.01):
                free[index] = True
        return free

    with pytest.raises(MinimalRegionError) as excinfo:
        sample_baseline_path(seed, seed + 0.01, picky_oracle)
    assert excinfo.value.reason == "sampling_failed"


def test_empty_region_has_explicit_reason(spec):
    """空区域 ⇒ ``is_empty`` 为真且 ``reason`` 明确。"""
    from rm75_drake.coordinated.regions import MinimalRegion

    empty = MinimalRegion.empty(spec, reason="seed_infeasible")
    assert empty.is_empty is True
    assert empty.reason == "seed_infeasible"
    assert empty.ambient_dimension == 8


# ── oracle 接口 ────────────────────────────────────────────────────────
def test_numpy_oracle_interface_is_pure_numpy():
    """oracle 必须是纯 numpy 接口：一批 x → free/busy 布尔数组。"""
    from rm75_drake.coordinated.regions import make_numpy_oracle

    def mapper(x):
        q = np.zeros((len(x), 14))
        q[:, 0] = x[:, 0]
        return q

    def collision_free(q):
        return np.array([row[0] > 0.0 for row in q])

    oracle = make_numpy_oracle(mapper, collision_free)
    flags = oracle(np.array([[1.0] * 8, [-1.0] * 8]))
    assert isinstance(flags, np.ndarray)
    assert flags.dtype == bool
    assert flags.tolist() == [True, False]


def test_invalid_oracle_is_reported(spec):
    """oracle 输出维度不符 ⇒ 显式 ``invalid_oracle``。"""
    from rm75_drake.coordinated.regions import (
        MinimalRegionError,
        generate_minimal_region,
    )

    def bad_oracle(x_batch):
        return np.ones(len(np.asarray(x_batch).reshape(-1, 8)) + 1, dtype=bool)

    with pytest.raises(MinimalRegionError) as excinfo:
        generate_minimal_region(spec, bad_oracle)
    assert excinfo.value.reason == "invalid_oracle"


def test_contains_rejects_wrong_dimension(region):
    """``contains`` 的判定点维度不符 ⇒ 显式报错并点名。"""
    from rm75_drake.coordinated.regions import MinimalRegionError

    with pytest.raises(MinimalRegionError) as excinfo:
        region.contains(np.zeros(7))
    assert excinfo.value.reason == "invalid_spec"


# ── 零 pydrake 依赖 ────────────────────────────────────────────────────
def test_regions_module_has_no_drake_dependency():
    """模块顶层不得 import pydrake / Drake 侧子模块。"""
    import ast

    source = REGIONS_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    offenders = []
    for node in tree.body:  # 只看**顶层** import
        if isinstance(node, ast.Import):
            names = [alias.name for alias in node.names]
        elif isinstance(node, ast.ImportFrom):
            names = [node.module or ""]
        else:
            continue
        for name in names:
            if name.startswith("pydrake") or name.startswith("rm75_drake"):
                offenders.append(name)
    assert offenders == []


def test_importing_regions_does_not_pull_pydrake():
    """``import rm75_drake.coordinated.regions`` 不得把 pydrake 拉进 sys.modules。"""
    code = (
        "import sys; import rm75_drake.coordinated.regions as r; "
        "assert not [m for m in sys.modules if m.startswith('pydrake')], "
        "sorted(m for m in sys.modules if m.startswith('pydrake')); "
        "print('ok')"
    )
    env = {"PYTHONPATH": str(REPO_ROOT / "src")}
    result = subprocess.run(
        [sys.executable, "-c", code], capture_output=True, text=True, env=env, cwd=REPO_ROOT
    )
    assert result.returncode == 0, result.stderr
    assert "ok" in result.stdout


def test_importing_coordinated_package_does_not_pull_pydrake():
    """``import rm75_drake.coordinated`` 不得把 pydrake 拉进 sys.modules。"""
    code = (
        "import sys; import rm75_drake.coordinated as c; "
        "assert not [m for m in sys.modules if m.startswith('pydrake')]; "
        "print('ok')"
    )
    env = {"PYTHONPATH": str(REPO_ROOT / "src")}
    result = subprocess.run(
        [sys.executable, "-c", code], capture_output=True, text=True, env=env, cwd=REPO_ROOT
    )
    assert result.returncode == 0, result.stderr
    assert "ok" in result.stdout
