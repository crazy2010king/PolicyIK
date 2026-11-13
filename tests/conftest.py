"""pytest 全局配置：把 ``src/`` 注入 ``sys.path``。

这样测试无需 ``pip install -e .`` 即可 ``import rm75_drake``（CI 的 fast 档也依赖这一点）。

**共享场景 fixture**（S02 规划层）
--------------------------------
S02 的规划测试（IK / IRIS / GCS / Toppra）都需要同一个"单臂场景"，
因此把场景装配集中在这里的 fixture，**禁止各测试文件重复造场景装配代码**
（S02-T01 的 :func:`rm75_drake.planning.scene.build_scene` 是唯一装配入口）。

* ``single_arm_scenario``：读 ``scenarios/single_arm.yaml`` 的场景真相源；
* ``single_arm_scene``：装配好的 :class:`~rm75_drake.planning.scene.SceneBundle`
  （模块级缓存：一次装配，多个测试复用）。

**共享规划 fixture**（S02-T07）
-------------------------------
Planner 的链路里有 IRIS（本机单区域 5–18 s），因此把「一次成功规划」的产物
做成 **session 级** fixture，供 ``test_planner_api.py`` 与 ``test_fake_driver.py``
**共用同一条轨迹**——既省时间，也保证「同源证明」比对的确实是同一条输出。

* ``planner_bundle``：Planner 实例 + 场景 + 一次成功的关节目标规划结果。
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = REPO_ROOT / "src"

if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

#: 单臂场景 YAML（场景真相源）。
SCENARIO_PATH = REPO_ROOT / "scenarios" / "single_arm.yaml"


@pytest.fixture(scope="module")
def single_arm_scenario():
    """单臂场景（``scenarios/single_arm.yaml`` → :class:`Scenario`）。"""
    from rm75_drake.scenario import load_scenario

    return load_scenario(SCENARIO_PATH)


@pytest.fixture(scope="module")
def single_arm_scene(single_arm_scenario):
    """装配好的单臂场景（S02-T01 的 ``build_scene``，全测试共享）。"""
    from rm75_drake.planning.scene import build_scene

    return build_scene(single_arm_scenario)


# ── S02-T07 的共享规划参数（Planner + 驱动）────────────────────────────
#: 起点位形：抬肘名义姿态（IK 模块的 NOMINAL_POSTURE，碰撞自由）。
PLANNER_START_Q = (0.0, -0.6, 0.0, 0.9, 0.0, 0.5, 0.0)

#: 目标位形：沿 joint_1 方向偏移 0.3 rad（直线段碰撞自由，见测试断言）。
PLANNER_TARGET_Q = (0.3, -0.6, 0.0, 0.9, 0.0, 0.5, 0.0)

#: IRIS 认证强度（比默认 5 快约一倍：单区域 ~8 s）。
PLANNER_REGION_OPTIONS = {"num_collision_infeasible_samples": 3}


@pytest.fixture(scope="session")
def planner_bundle():
    """S02-T07 的共享规划产物（session 级：IRIS 很贵）。

    返回
    ----
    dict：
        ``planner``（:class:`rm75_drake.planner.Planner`）、
        ``scene``（场景装配产物）、``scenario``（场景真相源）、
        ``options``（本次规划用的 :class:`rm75_drake.planner.PlanOptions`）、
        ``result``（一次成功的关节目标规划结果）。
    """
    from rm75_drake.planner import Planner, PlanOptions
    from rm75_drake.scenario import load_scenario
    from rm75_drake.planning.scene import build_scene

    scenario = load_scenario(SCENARIO_PATH)
    scene = build_scene(scenario)
    options = PlanOptions(
        start_configuration=PLANNER_START_Q,
        region_options=PLANNER_REGION_OPTIONS,
    )
    planner = Planner(scenario, scene=scene, options=options)
    result = planner.plan_joint_target(PLANNER_TARGET_Q)
    return {
        "planner": planner,
        "scene": scene,
        "scenario": scenario,
        "options": options,
        "result": result,
        "start_q": PLANNER_START_Q,
        "target_q": PLANNER_TARGET_Q,
    }
