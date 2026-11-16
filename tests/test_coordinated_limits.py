"""双臂 14 维软件限位的**程序派生**（S05-T02）。

覆盖
----
* 14 维限位与 S02 的 7 维单臂限位**逐项一致**（同一台机器人 ⇒ 两臂相同）；
* 加速度上限沿用 S02 的自研默认值 ``DEFAULT_ACCELERATION_LIMITS_RAD_S2``；
* **负例**：关节名数量错 / 名字形态不符（非 ``left_joint_i`` / ``right_joint_i``）/
  顺序错配（左右互换、臂内乱序）→ 显式报错并**点名期望 / 实际**；
* 夹爪 DOF 补齐口径（14 → 18 维，顺序 = plant 的位置变量顺序）；
* 零 pydrake 依赖（静态断言 + ``import rm75_drake.coordinated`` 不把 pydrake
  拉进 ``sys.modules``）。
"""

from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

from rm75_drake.coordinated import limits as coordinated_limits
from rm75_drake.coordinated.limits import (
    ARM_JOINT_COUNT,
    DUAL_JOINT_COUNT,
    CoordinatedLimitError,
    DualLimits,
    canonical_joint_names,
    derive_dual_limits,
    pad_gripper_positions,
)
from rm75_drake.planning.trajectory import (
    DEFAULT_ACCELERATION_LIMITS_RAD_S2,
    software_limits,
)

REPO_ROOT = Path(__file__).resolve().parents[1]
DUAL_SCENARIO_PATH = REPO_ROOT / "scenarios" / "dual_arm.yaml"


@pytest.fixture(scope="module")
def scenario_joint_names():
    """场景真相源里的双臂关节名（左 7 + 右 7）。"""
    from rm75_drake.scenario import load_scenario

    scenario = load_scenario(DUAL_SCENARIO_PATH)
    names: list[str] = []
    for robot in scenario.robots:
        names.extend(robot.joint_names)
    return tuple(names)


@pytest.fixture(scope="module")
def dual_limits(scenario_joint_names):
    return derive_dual_limits(scenario_joint_names)


# ── 正例 ────────────────────────────────────────────────────────────────
def test_dual_limits_match_single_arm_limits_itemwise(dual_limits):
    """14 维限位 = 7 维单臂限位逐项复制两遍（同一台机器人 ⇒ 两臂相同）。"""
    single = software_limits()
    assert len(dual_limits.lower) == DUAL_JOINT_COUNT == 14
    assert dual_limits.lower[:ARM_JOINT_COUNT] == tuple(single.lower)
    assert dual_limits.lower[ARM_JOINT_COUNT:] == tuple(single.lower)
    assert dual_limits.upper[:ARM_JOINT_COUNT] == tuple(single.upper)
    assert dual_limits.upper[ARM_JOINT_COUNT:] == tuple(single.upper)
    assert dual_limits.velocity[:ARM_JOINT_COUNT] == tuple(single.velocity)
    assert dual_limits.velocity[ARM_JOINT_COUNT:] == tuple(single.velocity)


def test_dual_limits_acceleration_reuses_s02_self_research_default(dual_limits):
    """加速度上限沿用 S02 的自研默认值（**不另起数值**）。"""
    expected = tuple(DEFAULT_ACCELERATION_LIMITS_RAD_S2) * 2
    assert dual_limits.acceleration == expected
    assert dual_limits.acceleration[:ARM_JOINT_COUNT] == tuple(
        DEFAULT_ACCELERATION_LIMITS_RAD_S2
    )


def test_dual_limits_sides_and_slices(dual_limits, scenario_joint_names):
    """侧别归属与切片正确（左 7 + 右 7）。"""
    assert dual_limits.joint_names == tuple(scenario_joint_names)
    assert dual_limits.sides == ("left",) * 7 + ("right",) * 7
    left = dual_limits.side_slice("left")
    right = dual_limits.side_slice("right")
    assert left == slice(0, 7)
    assert right == slice(7, 14)
    assert dual_limits.lower_array().shape == (14,)
    assert dual_limits.velocity_array().shape == (14,)
    assert dual_limits.acceleration_array().shape == (14,)


def test_canonical_joint_names_matches_scenario(scenario_joint_names):
    """场景声明的关节名与规范顺序逐项一致（S04 的重命名 URDF 变体）。"""
    assert canonical_joint_names() == tuple(scenario_joint_names)


def test_pad_gripper_positions_appends_four_prismatic_do_f():
    """夹爪 DOF 口径：14 维 → 18 维（两夹爪各 2 个 prismatic）。"""
    q14 = np.zeros((3, DUAL_JOINT_COUNT))
    padded = pad_gripper_positions(
        q14,
        gripper_joint_names=(
            "left_finger_joint_left",
            "left_finger_joint_right",
            "right_finger_joint_left",
            "right_finger_joint_right",
        ),
        gripper_value=0.01,
    )
    assert padded.shape == (3, 18)
    assert np.allclose(padded[:, :14], q14)
    assert np.allclose(padded[:, 14:], 0.01)

    # 不装夹爪 → 原样返回 14 维
    assert pad_gripper_positions(q14).shape == (3, 14)


# ── 负例 ────────────────────────────────────────────────────────────────
def test_dual_limits_reject_wrong_joint_count():
    """负例：关节名数量不是 14 → 显式报错并点名期望 / 实际。"""
    with pytest.raises(CoordinatedLimitError) as excinfo:
        derive_dual_limits(canonical_joint_names()[:13])
    message = str(excinfo.value)
    assert "[joint_count]" in message
    assert "14" in message
    assert "13" in message


def test_dual_limits_reject_non_canonical_name_shape():
    """负例：名字不是 left_joint_i / right_joint_i 形态 → 显式报错并点名。"""
    names = list(canonical_joint_names())
    names[3] = "arm_joint_4"
    with pytest.raises(CoordinatedLimitError) as excinfo:
        derive_dual_limits(names)
    message = str(excinfo.value)
    assert "[joint_names]" in message
    assert "arm_joint_4" in message
    assert "left_joint_i" in message


def test_dual_limits_reject_swapped_arm_order():
    """负例：左右臂顺序错配 → 显式报错并点名期望 / 实际。"""
    names = list(canonical_joint_names())
    names[0], names[7] = names[7], names[0]
    with pytest.raises(CoordinatedLimitError) as excinfo:
        derive_dual_limits(names)
    message = str(excinfo.value)
    assert "[joint_names]" in message
    assert "right_joint_1" in message
    assert "left_joint_1" in message


def test_dual_limits_reject_shuffled_joint_index():
    """负例：臂内关节序号乱序 → 显式报错并点名。"""
    names = list(canonical_joint_names())
    names[1], names[2] = names[2], names[1]
    with pytest.raises(CoordinatedLimitError) as excinfo:
        derive_dual_limits(names)
    message = str(excinfo.value)
    assert "[joint_names]" in message
    assert "left_joint_2" in message


def test_dual_limits_reject_non_dual_limits_object():
    """负例：DualLimits 维度不符 → 显式报错。"""
    with pytest.raises(CoordinatedLimitError) as excinfo:
        DualLimits(
            joint_names=canonical_joint_names()[:7],
            lower=(0.0,) * 7,
            upper=(1.0,) * 7,
            velocity=(1.0,) * 7,
            acceleration=(1.0,) * 7,
            sides=("left",) * 7,
        )
    assert "[joint_count]" in str(excinfo.value)


def test_pad_gripper_positions_rejects_wrong_dimension():
    """负例：臂关节维度不符 → 显式报错（绝不静默补齐）。"""
    with pytest.raises(CoordinatedLimitError) as excinfo:
        pad_gripper_positions(np.zeros((2, 7)), gripper_joint_names=("a",))
    assert "[joint_count]" in str(excinfo.value)


def test_pad_gripper_positions_rejects_non_finite_and_bad_value():
    """负例：NaN / Inf 与非法夹爪开度 → 显式报错。"""
    broken = np.zeros((2, DUAL_JOINT_COUNT))
    broken[0, 0] = np.nan
    with pytest.raises(CoordinatedLimitError) as excinfo:
        pad_gripper_positions(broken, gripper_joint_names=("a",))
    assert "[limits]" in str(excinfo.value)

    with pytest.raises(CoordinatedLimitError) as excinfo:
        pad_gripper_positions(
            np.zeros((2, DUAL_JOINT_COUNT)),
            gripper_joint_names=("a",),
            gripper_value=float("nan"),
        )
    assert "[gripper]" in str(excinfo.value)


# ── 零 pydrake 依赖 ─────────────────────────────────────────────────────
def test_coordinated_modules_have_no_drake_dependency():
    """协调层模块不得 import pydrake（也不得绕回 rm75_drake 的 Drake 侧）。"""
    for module in (coordinated_limits,):
        source = Path(module.__file__).read_text(encoding="utf-8")
        offenders = [
            line.strip()
            for line in source.splitlines()
            if re.search(r"^\s*(import|from)\s+pydrake", line)
        ]
        assert offenders == [], f"{module.__name__} 出现了 Drake 导入：{offenders}"


def test_coordinated_package_import_does_not_load_pydrake():
    """import rm75_drake.coordinated 不应把 pydrake 拉进 sys.modules。"""
    code = (
        "import sys; sys.path.insert(0, %r);"
        "import rm75_drake.coordinated;"
        "import rm75_drake.coordinated.limits;"
        "import rm75_drake.coordinated.trajectory;"
        "print('pydrake' in sys.modules)" % str(REPO_ROOT / "src")
    )
    out = subprocess.run(
        [sys.executable, "-c", code], capture_output=True, text=True, check=True
    )
    assert out.stdout.strip().endswith("False"), out.stdout
