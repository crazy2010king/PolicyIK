"""IRIS 区域生成与凸区域缓存测试（S02-T04）。

被测对象
--------
:mod:`rm75_drake.planning.regions` —— 种子位形 → C 空间凸区域 + 惰性缓存。

覆盖的契约
----------
**正例**
* 生成的区域是 7 维、**非空**、**⊆ 软件软限位盒**，且**包含种子位形**；
* 两个不同种子 → 两个不同区域；
* **同 spec 两次生成结果逐位一致**（IRIS 确定性，可复现）；
* 缓存命中后**不再重跑 IRIS**（``RegionCache.iris_runs`` 计数断言）；
* 缓存可落盘、可回读（多面体数值一致），且**URDF 版本不符时拒绝复用**。

**负例（可解释失败）**
* 种子维度不对 / 含 NaN / 越软件软限位 → ``invalid_seed``；
* 种子位形本身碰撞（不可行）→ ``seed_infeasible``；
* 配置非法（margin < 0、迭代上限 ≤ 0、未知 options 旋钮）→ ``invalid_config``；
* 缓存文件损坏 / schema 版本不符 → ``cache_corrupt``；
* 缓存 URDF 版本不符 → ``version_mismatch``。

耗时说明
--------
IRIS 单区域在本机耗时 8–18 s（Drake 1.51.1）。为控制测试时长，本文件用
``num_collision_infeasible_samples=3``（比默认 5 快约一倍，认证强度略降），
并把 IRIS 重负载用例统一打上 ``@pytest.mark.slow`` 标记：CI fast 档用
``-m "not slow"`` 可完全跳过（注册见 ``pyproject.toml`` 的
``[tool.pytest.ini_options] markers``）。

场景装配复用 ``tests/conftest.py`` 的 ``single_arm_scene`` fixture
（S02-T01 的 ``build_scene`` 是唯一装配入口，测试里**不重复造场景**）。
"""

from __future__ import annotations

import logging

import numpy as np
import pytest

from rm75_drake.planning import regions as regions_mod

# IRIS 内部会打大量 INFO 日志（每次碰撞认证一行），测试里降到 WARNING。
logging.getLogger("drake").setLevel(logging.WARNING)

#: 种子 A：抬肘名义姿态（碰撞自由，本文件用 checker 自证）。
SEED_A = (0.0, -0.6, 0.0, 0.9, 0.0, 0.5, 0.0)

#: 种子 B：IK 测试里用过的参考位形（另一处工作空间点）。
SEED_B = (2.708359, -0.762284, -2.961942, 0.130607, 1.449474, 1.702438, 3.062573)

#: 碰撞种子候选（先试固定候选，再按固定随机种子采样，保证可复现）。
COLLIDING_CANDIDATES = (
    (0.7596, 1.747, 1.6741, -1.2565, -1.2135, 1.6169, -6.1479),
    (-2.8519, -1.6586, 2.8367, 0.7213, -0.4359, 0.1028, 4.6328),
)

#: 测试用的 IRIS 认证强度（比默认 5 快约一倍）。
TEST_COLLISION_SAMPLES = 3


def _spec(seed_q, **kwargs):
    """构造测试用的 :class:`RegionSpec`（认证强度调低以控耗时）。"""
    return regions_mod.RegionSpec(
        seed_q=tuple(seed_q),
        num_collision_infeasible_samples=TEST_COLLISION_SAMPLES,
        **kwargs,
    )


@pytest.fixture(scope="module")
def spec_a():
    """种子 A 的区域 spec。"""
    return _spec(SEED_A)


@pytest.fixture(scope="module")
def spec_b():
    """种子 B 的区域 spec。"""
    return _spec(SEED_B)


@pytest.fixture(scope="module")
def region_a(single_arm_scene, spec_a):
    """种子 A 的区域（跑一次 IRIS，模块内共享）。"""
    return regions_mod.generate_region(single_arm_scene, spec_a.seed_q, spec=spec_a)


@pytest.fixture(scope="module")
def region_b(single_arm_scene, spec_b):
    """种子 B 的区域（跑一次 IRIS，模块内共享）。"""
    return regions_mod.generate_region(single_arm_scene, spec_b.seed_q, spec=spec_b)


# ── 正例 ────────────────────────────────────────────────────────────────
@pytest.mark.slow
def test_region_has_expected_shape_and_is_nonempty(region_a):
    """正例：区域是 7 维且非空。"""
    assert region_a.ambient_dimension == 7
    assert not region_a.is_empty
    assert region_a.seed_options.seed_q == tuple(SEED_A)
    assert region_a.duration_s > 0.0


@pytest.mark.slow
def test_region_is_contained_in_software_limit_box(region_a):
    """正例：区域 ⊆ 软件软限位盒（域 = 软限位的硬性保证）。"""
    box = regions_mod.soft_limit_box()
    assert region_a.hpolyhedron.ContainedIn(box), (
        "IRIS 区域越出了软件软限位盒（IRIS 自身的 domain 是 URDF 硬件限位，"
        "必须靠与软限位盒求交来保证）"
    )


@pytest.mark.slow
def test_region_contains_its_seed(region_a):
    """正例：区域包含自己的种子位形。"""
    assert region_a.contains(SEED_A)
    # 种子本身必须碰撞自由（IRIS 的前提）。
    assert region_a.seed_q == tuple(SEED_A)


@pytest.mark.slow
def test_two_seeds_produce_two_distinct_regions(region_a, region_b):
    """正例：两个不同种子 → 两个不同的区域。"""
    assert region_a.id != region_b.id
    assert region_a.seed_q != region_b.seed_q
    assert region_b.ambient_dimension == 7
    assert not region_b.is_empty


@pytest.mark.slow
def test_same_spec_is_reproducible(single_arm_scene, region_a, spec_a):
    """正例：同 spec 两次生成得到**逐位相同**的区域（IRIS 确定性）。"""
    again = regions_mod.generate_region(
        single_arm_scene, spec_a.seed_q, spec=spec_a
    )
    assert np.array_equal(again.hpolyhedron.A(), region_a.hpolyhedron.A())
    assert np.array_equal(again.hpolyhedron.b(), region_a.hpolyhedron.b())
    assert again.id == region_a.id


@pytest.mark.slow
def test_cache_reuses_region_without_recomputing(single_arm_scene, spec_a, region_a):
    """正例：缓存命中后**不重跑 IRIS**（计数器 + 数值一致双重断言）。"""
    cache = regions_mod.RegionCache(single_arm_scene, [spec_a])
    assert cache.iris_runs == 0

    first = cache.get(spec_a)
    assert cache.iris_runs == 1

    second = cache.get(spec_a)
    assert cache.iris_runs == 1, "命中缓存却重跑了 IRIS"
    assert first is second
    assert np.array_equal(first.hpolyhedron.A(), region_a.hpolyhedron.A())

    # regions() 走同一份缓存，不额外计算。
    assert len(cache.regions()) == 1
    assert cache.iris_runs == 1


@pytest.mark.slow
def test_cache_save_and_load_roundtrip(single_arm_scene, spec_a, tmp_path):
    """正例：缓存落盘 → 回读，多面体数值一致，且回读后**不再**触发 IRIS。"""
    cache = regions_mod.RegionCache(single_arm_scene, [spec_a])
    original = cache.get(spec_a)
    assert cache.iris_runs == 1

    path = cache.save_cache(tmp_path / "regions.yaml")
    assert path.is_file()

    loaded = regions_mod.RegionCache.load_cache(path, single_arm_scene)
    assert loaded.iris_runs == 0
    assert loaded.model_version == cache.model_version
    assert loaded.model_version, "场景必须提供 URDF 版本标识"

    restored = loaded.get(spec_a)
    assert loaded.iris_runs == 0, "回读后的缓存不该再跑 IRIS"
    assert np.array_equal(
        restored.hpolyhedron.A(), original.hpolyhedron.A()
    )
    assert np.array_equal(
        restored.hpolyhedron.b(), original.hpolyhedron.b()
    )
    assert restored.id == original.id


# ── 负例：可解释失败 ────────────────────────────────────────────────────
def test_seed_with_wrong_dimension_is_rejected(single_arm_scene):
    """负例：种子维度不对 → 显式报错（invalid_seed）。"""
    with pytest.raises(regions_mod.RegionError) as excinfo:
        regions_mod.generate_region(single_arm_scene, (0.0, -0.6, 0.0))
    assert excinfo.value.reason == regions_mod.REASON_INVALID_SEED


def test_seed_with_nan_is_rejected(single_arm_scene):
    """负例：种子含 NaN → 显式报错（invalid_seed），不是静默失败。"""
    bad = (0.0, -0.6, float("nan"), 0.9, 0.0, 0.5, 0.0)
    with pytest.raises(regions_mod.RegionError) as excinfo:
        regions_mod.generate_region(single_arm_scene, bad)
    assert excinfo.value.reason == regions_mod.REASON_INVALID_SEED


def test_seed_outside_software_limits_is_rejected(single_arm_scene):
    """负例：种子越软件软限位 → 显式报错并点名越界关节。"""
    bad = list(SEED_A)
    bad[1] = 3.0  # joint_2 的软件软限位是 [-2.1990868, 2.1990868]
    with pytest.raises(regions_mod.RegionError) as excinfo:
        regions_mod.generate_region(single_arm_scene, bad)
    assert excinfo.value.reason == regions_mod.REASON_INVALID_SEED
    assert "joint_2" in str(excinfo.value)


@pytest.mark.slow
def test_colliding_seed_is_reported_as_infeasible(single_arm_scene):
    """负例：种子位形本身碰撞 → ``seed_infeasible``（不是空泛的 iris_error）。"""
    names, lower, upper = regions_mod.software_joint_limits()
    candidates = []
    for candidate in COLLIDING_CANDIDATES:
        values = np.asarray(candidate, dtype=float)
        if np.all(values >= lower) and np.all(values <= upper):
            candidates.append(values)
    if not candidates:
        rng = np.random.default_rng(20240517)
        for _ in range(2000):
            values = lower + rng.random(7) * (upper - lower)
            candidates.append(values)

    colliding = None
    for values in candidates:
        if not single_arm_scene.checker.CheckConfigCollisionFree(values):
            colliding = values
            break
    assert colliding is not None, "找不到碰撞种子（场景变了？）"

    with pytest.raises(regions_mod.RegionError) as excinfo:
        regions_mod.generate_region(single_arm_scene, colliding)
    assert excinfo.value.reason == regions_mod.REASON_SEED_INFEASIBLE


@pytest.mark.parametrize(
    "kwargs",
    [
        {"configuration_space_margin": -0.1},
        {"iteration_limit": 0},
        {"random_seed": -1},
        {"num_collision_infeasible_samples": 0},
    ],
)
def test_invalid_configuration_is_rejected(kwargs):
    """负例：非法 IRIS 配置 → 显式报错（invalid_config）。"""
    with pytest.raises(regions_mod.RegionError) as excinfo:
        regions_mod.RegionSpec(seed_q=SEED_A, **kwargs)
    assert excinfo.value.reason == regions_mod.REASON_INVALID_CONFIG


def test_unknown_option_knob_is_rejected(single_arm_scene):
    """负例：未知的 options 旋钮 → 显式报错（否则 spec 无法完整决定区域）。"""
    with pytest.raises(regions_mod.RegionError) as excinfo:
        regions_mod.generate_region(
            single_arm_scene, SEED_A, options={"not_a_knob": 1}
        )
    assert excinfo.value.reason == regions_mod.REASON_INVALID_CONFIG


def test_spec_and_seed_mismatch_is_rejected(single_arm_scene, spec_a):
    """负例：``seed_q`` 与 ``spec.seed_q`` 不一致 → 显式报错。"""
    with pytest.raises(regions_mod.RegionError) as excinfo:
        regions_mod.generate_region(single_arm_scene, SEED_B, spec=spec_a)
    assert excinfo.value.reason == regions_mod.REASON_INVALID_CONFIG


def test_corrupt_cache_file_is_reported(single_arm_scene, tmp_path):
    """负例：缓存文件损坏 → ``cache_corrupt``（不静默复用）。"""
    path = tmp_path / "broken.yaml"
    path.write_text("schema_version: 999\nregions: []\n", encoding="utf-8")
    with pytest.raises(regions_mod.RegionError) as excinfo:
        regions_mod.RegionCache.load_cache(path, single_arm_scene)
    assert excinfo.value.reason == regions_mod.REASON_CACHE_CORRUPT


@pytest.mark.slow
def test_cache_with_wrong_model_version_is_refused(single_arm_scene, spec_a, tmp_path):
    """负例：缓存的 URDF 版本与当前场景不符 → **拒绝复用**（version_mismatch）。"""
    import yaml

    cache = regions_mod.RegionCache(single_arm_scene, [spec_a])
    cache.get(spec_a)
    path = cache.save_cache(tmp_path / "regions.yaml")

    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    payload["model_version"] = "0" * 64
    path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")

    with pytest.raises(regions_mod.RegionError) as excinfo:
        regions_mod.RegionCache.load_cache(path, single_arm_scene)
    assert excinfo.value.reason == regions_mod.REASON_VERSION_MISMATCH
