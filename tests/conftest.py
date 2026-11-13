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
