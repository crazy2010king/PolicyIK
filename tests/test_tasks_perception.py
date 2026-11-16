"""感知适配接口测试（S06-T06）：**非阻塞离散事件** + **保守场景膨胀**。

覆盖面
------
* **P0-a（鉴别力正例）**：膨胀后的感知盒放在 TCP 轨迹必经处 ⇒ 重建 checker 后
  :func:`rm75_drake.planning.gcs.collision_recheck` 的 ``num_escapes > 0``；
* **P0-b（鉴别力反例）**：**唯一变量 = 裕量**（``margin=0`` ⇒ 盒子不膨胀）
  ⇒ ``num_escapes == 0`` ⇒ 证明「裕量」是起作用的那个变量；
* **P0-c（钉死已知量）**：注册后**旧 checker** 仍判自由
  ⇒ 证明「**重建 checker**」这一步不可省；
* 非阻塞语义：队列满时 :meth:`PerceptionBus.publish` **不阻塞**、丢弃最旧并计数
  ``dropped``；:meth:`PerceptionBus.poll` 返回最新观测且 ``stale`` 标记正确；
  :meth:`PerceptionPolicy.effective_margin` 随延迟**单调增**、到 ``max_margin``
  **封顶**；
* 惰性导入：``import rm75_drake.tasks.perception`` **不**拉起 pydrake。

三条鉴别力断言的口径（**唯一变量**）
------------------------------------
P0-a 与 P0-b 用**同一个观测**、**同一条轨迹**、**同一份策略族**，唯一差别是
``PerceptionPolicy.margin``（0.01 m vs 0 m）⇒ 碰撞与否由**裕量**决定，
而不是由「装了个盒子」决定。

场景事实（本机 Drake 1.51.1 实测，已钉死）
-----------------------------------------
* 单臂场景零位附近，TCP（``link_7`` 原点）上方 ``+0.01 m``（世界系 z）处放一个
  **半长 0.005 m** 的盒 ⇒ 自由；半长 ``0.010 m`` ⇒ **逃逸**（与 ``link_6/link_7``
  几何相交）。⇒ 半长每轴差 5 mm 就是「自由 / 碰撞」的分界 ⇒ 裕量必须足够大
  才能让保守场景生效。
* 旧 checker 对**任何**新注册几何都判自由（实测 ``old_free=True``）⇒ 必须重建。
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = REPO_ROOT / "src"

from rm75_drake.planning import gcs
from rm75_drake.tasks.perception import (
    PerceptionBus,
    PerceptionError,
    PerceptionObservation,
    PerceptionPolicy,
    apply_perception_scene,
)

#: 场景真相源（与 S02 规划层同一份）。
SCENARIO_PATH = REPO_ROOT / "scenarios" / "single_arm.yaml"

#: 轨迹起点（抬肘名义姿态，碰撞自由）。
Q_START = (0.0, -0.6, 0.0, 0.9, 0.0, 0.5, 0.0)

#: 轨迹终点（沿 joint_1 偏移 0.3 rad）。
Q_END = (0.3, -0.6, 0.0, 0.9, 0.0, 0.5, 0.0)

#: 感知盒中心相对 TCP（``link_7`` 原点）的世界系 z 偏移（m）。
TCP_Z_OFFSET = 0.01

#: 感知盒原始半长（m）：自由 / 碰撞分界在 0.005 ~ 0.010 之间。
BASE_HALF_SIZE = 0.005


# ── 夹具 ────────────────────────────────────────────────────────────────
@pytest.fixture(scope="module")
def scenario():
    from rm75_drake.scenario import load_scenario

    return load_scenario(SCENARIO_PATH)


@pytest.fixture()
def bundle(scenario):
    """每次测试**独立装配**场景（感知更新会改写 ``bundle.checker``）。"""
    from rm75_drake.planning.scene import build_scene

    return build_scene(scenario)


@pytest.fixture()
def trajectory() -> np.ndarray:
    """TCP 必经的关节轨迹（直线插值，31 点）。"""
    return np.linspace(np.asarray(Q_START), np.asarray(Q_END), 31)


def _tcp_position(bundle) -> np.ndarray:
    """返回当前零位配置下 TCP（``link_7`` 原点）的世界系位置。"""
    context = bundle.plant_context
    bundle.plant.SetPositions(context, np.asarray(Q_START))
    pose = bundle.plant.EvalBodyPoseInWorld(
        context, bundle.plant.GetBodyByName("link_7")
    )
    return np.array(pose.translation())


def _observation(bundle, **overrides) -> PerceptionObservation:
    """构造一个「贴在 TCP 轨迹必经处」的感知观测。"""
    center = _tcp_position(bundle) + np.array([0.0, 0.0, TCP_Z_OFFSET])
    kwargs = dict(
        observation_id="obs_tcp",
        position_xyz=tuple(float(v) for v in center),
        rpy=(0.0, 0.0, 0.0),
        half_size=(BASE_HALF_SIZE, BASE_HALF_SIZE, BASE_HALF_SIZE),
        timestamp_s=10.0,
        source="synthetic",
    )
    kwargs.update(overrides)
    return PerceptionObservation(**kwargs)


# ── P0-a：膨胀裕量生效 ⇒ 碰撞逃逸 ─────────────────────────────────────
def test_inflated_perception_box_causes_collision_escape(bundle, trajectory):
    """P0-a：膨胀盒贴在 TCP 轨迹必经处 ⇒ 重建 checker 后 ``num_escapes > 0``。"""
    observation = _observation(bundle)
    policy = PerceptionPolicy(margin=0.01, latency_margin_rate=0.0, max_margin=0.15)

    result = apply_perception_scene(bundle, [observation], policy, now_s=10.0)

    assert result.checker_rebuilt is True
    assert len(result.geometry_ids) == 1
    # 生效裕量 = 0.01 m ⇒ 每轴半长从 0.005 膨胀到 0.015。
    assert result.effective_margins["obs_tcp"] == pytest.approx(0.01)
    assert bundle.checker is not None

    recheck = gcs.collision_recheck(bundle, trajectory, edge_step_size=0.01)
    assert recheck.num_escapes > 0, (
        "膨胀后的感知盒必须让 TCP 轨迹产生碰撞逃逸；"
        f"实际 num_escapes={recheck.num_escapes}"
    )


# ── P0-b：唯一变量 = 裕量 ⇒ margin=0 不碰撞 ───────────────────────────
def test_zero_margin_perception_box_stays_free(bundle, trajectory):
    """P0-b（反例）：``margin=0`` ⇒ 盒子不膨胀 ⇒ ``num_escapes == 0``。

    与 P0-a **唯一差别**是 ``PerceptionPolicy.margin``（观测、轨迹、场景全同）
    ⇒ 证明「裕量」是起作用的那个变量。
    """
    observation = _observation(bundle)
    policy = PerceptionPolicy(margin=0.0, latency_margin_rate=0.0, max_margin=0.15)

    result = apply_perception_scene(bundle, [observation], policy, now_s=10.0)

    assert result.effective_margins["obs_tcp"] == pytest.approx(0.0)

    recheck = gcs.collision_recheck(bundle, trajectory, edge_step_size=0.01)
    assert recheck.num_escapes == 0, (
        "零裕量时感知盒不应改变碰撞判定；"
        f"实际 num_escapes={recheck.num_escapes}"
    )


# ── P0-c：旧 checker 看不见新几何 ⇒ 必须重建 ───────────────────────────
def test_old_checker_cannot_see_registered_geometry(bundle, trajectory):
    """P0-c：注册后**旧 checker** 仍判自由 ⇒ 「重建 checker」这一步不可省。"""
    observation = _observation(bundle)
    policy = PerceptionPolicy(margin=0.01, latency_margin_rate=0.0, max_margin=0.15)

    old_checker = bundle.checker
    apply_perception_scene(bundle, [observation], policy, now_s=10.0)
    new_checker = bundle.checker

    assert new_checker is not old_checker, "必须重建碰撞检查器并回填 bundle.checker"

    # 旧 checker 对新注册的几何**完全看不见** ⇒ 仍判自由（钉死的已知量）。
    assert old_checker.CheckConfigCollisionFree(np.asarray(Q_START)) is True

    # 新 checker 看得见 ⇒ 同一份位形判碰撞。
    assert new_checker.CheckConfigCollisionFree(np.asarray(Q_START)) is False


# ── 膨胀量与延迟的关系（保守场景吸收延迟）──────────────────────────────
def test_effective_margin_grows_with_latency_and_caps():
    """延迟越大裕量越大（延迟被保守场景吸收），到 ``max_margin`` 封顶。"""
    observation = PerceptionObservation(
        observation_id="o",
        position_xyz=(0.0, 0.0, 0.0),
        rpy=(0.0, 0.0, 0.0),
        half_size=(0.01, 0.01, 0.01),
        timestamp_s=100.0,
        source="synthetic",
    )
    policy = PerceptionPolicy(
        margin=0.02, latency_margin_rate=0.05, max_margin=0.15, stale_after_s=0.5
    )

    assert policy.effective_margin(observation, 100.0) == pytest.approx(0.02)
    assert policy.effective_margin(observation, 101.0) == pytest.approx(0.07)
    assert policy.effective_margin(observation, 102.0) == pytest.approx(0.12)
    # 封顶
    assert policy.effective_margin(observation, 103.0) == pytest.approx(0.15)
    assert policy.effective_margin(observation, 1000.0) == pytest.approx(0.15)
    # 单调不减
    margins = [policy.effective_margin(observation, 100.0 + 0.1 * i) for i in range(20)]
    assert all(b >= a for a, b in zip(margins, margins[1:]))


def test_stale_observation_is_flagged_and_keeps_conservative_margin(bundle):
    """过期观测：快照 ``stale=True``，且仍按（更大的）裕量注册 ⇒ 更保守。"""
    observation = _observation(bundle, timestamp_s=10.0)
    policy = PerceptionPolicy(
        margin=0.02, latency_margin_rate=0.05, max_margin=0.15, stale_after_s=0.5
    )

    result = apply_perception_scene(bundle, [observation], policy, now_s=12.0)

    assert result.stale_observation_ids == ("obs_tcp",)
    # 延迟 2 s ⇒ 0.02 + 0.05 × 2 = 0.12 m。
    assert result.effective_margins["obs_tcp"] == pytest.approx(0.12)


# ── 非阻塞事件总线 ──────────────────────────────────────────────────────
def _make_observation(index: int, timestamp_s: float = 0.0) -> PerceptionObservation:
    return PerceptionObservation(
        observation_id=f"o{index}",
        position_xyz=(0.0, 0.0, float(index)),
        rpy=(0.0, 0.0, 0.0),
        half_size=(0.01, 0.01, 0.01),
        timestamp_s=timestamp_s,
        source="synthetic",
    )


def test_publish_never_blocks_and_counts_dropped():
    """队列满时 ``publish`` 立即返回、丢弃最旧、``dropped`` 计数 +1。"""
    bus = PerceptionBus(capacity=2)
    seen = []
    bus.subscribe(seen.append)

    bus.publish(_make_observation(0))
    bus.publish(_make_observation(1))
    assert bus.dropped == 0
    assert bus.queued == 2

    bus.publish(_make_observation(2))  # 队列满 ⇒ 丢弃最旧
    assert bus.dropped == 1
    assert bus.queued == 2
    # 回调收到全部三次（订阅者不受丢弃影响）。
    assert [o.observation_id for o in seen] == ["o0", "o1", "o2"]


def test_poll_returns_latest_snapshot_with_stale_flag():
    """``poll`` 返回最新观测；年龄超阈值 ⇒ ``stale=True``。"""
    bus = PerceptionBus(capacity=8)
    bus.publish(_make_observation(0, timestamp_s=10.0))
    bus.publish(_make_observation(1, timestamp_s=10.2))

    snapshot = bus.poll(10.2)
    assert snapshot is not None
    assert snapshot.observation.observation_id == "o1"
    # 最新观测就是 o1（timestamp 10.2）⇒ 年龄 0 ⇒ 未过期。
    assert snapshot.age_s == pytest.approx(0.0)
    assert snapshot.stale is False

    stale = bus.poll(20.0)
    assert stale is not None
    assert stale.observation.observation_id == "o1"
    assert stale.age_s == pytest.approx(9.8)
    assert stale.stale is True


def test_poll_on_empty_bus_returns_none():
    bus = PerceptionBus()
    assert bus.poll(0.0) is None


def test_failing_subscriber_does_not_break_publish():
    """坏订阅者抛异常不许拖垮发布方（异常被吞并计数）。"""

    def boom(_observation):
        raise RuntimeError("subscriber failed")

    bus = PerceptionBus(capacity=4)
    bus.subscribe(boom)
    bus.publish(_make_observation(0))
    assert bus.callback_errors == 1
    assert bus.published == 1
    assert bus.queued == 1


# ── 保守场景落地的其它可观察行为 ────────────────────────────────────────
def test_duplicate_observation_id_is_not_registered_twice(bundle):
    """同一 ``observation_id`` 重复到达 ⇒ 不重复注册几何（登记幂等）。"""
    observation = _observation(bundle)
    policy = PerceptionPolicy(margin=0.0, latency_margin_rate=0.0, max_margin=0.15)

    first = apply_perception_scene(bundle, [observation], policy, now_s=10.0)
    second = apply_perception_scene(bundle, [observation], policy, now_s=10.0)

    assert len(first.geometry_ids) == 1
    assert second.geometry_ids == ()
    assert second.dropped_observations == 1


def test_apply_perception_scene_rejects_invalid_inputs(bundle):
    """非法输入必须**显式报错**，不许静默通过。"""
    policy = PerceptionPolicy()
    with pytest.raises(PerceptionError):
        apply_perception_scene(bundle, [object()], policy, now_s=0.0)
    with pytest.raises(PerceptionError):
        apply_perception_scene(None, [], policy, now_s=0.0)
    with pytest.raises(PerceptionError):
        apply_perception_scene(bundle, [], "not-a-policy", now_s=0.0)


def test_observation_rejects_non_positive_half_size():
    """包围盒半长必须严格为正（否则是「量纲 / 约定写错」）。"""
    with pytest.raises(PerceptionError):
        PerceptionObservation(
            observation_id="bad",
            position_xyz=(0.0, 0.0, 0.0),
            rpy=(0.0, 0.0, 0.0),
            half_size=(0.0, 0.01, 0.01),
            timestamp_s=0.0,
            source="synthetic",
        )


# ── 惰性导入（零 pydrake 依赖）─────────────────────────────────────────
def test_importing_perception_module_does_not_load_pydrake() -> None:
    """``import rm75_drake.tasks.perception`` **不得**把 pydrake 拉进 sys.modules。"""
    code = (
        "import sys; sys.path.insert(0, %r);"
        "import rm75_drake.tasks.perception as p;"
        "assert 'pydrake' not in sys.modules, sorted(m for m in sys.modules if 'drake' in m);"
        "assert p.PerceptionPolicy is not None;"
        "print('ok')" % str(SRC_DIR)
    )
    result = subprocess.run(
        [sys.executable, "-c", code], capture_output=True, text=True, timeout=120
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "ok" in result.stdout
