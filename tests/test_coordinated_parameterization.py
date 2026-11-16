"""最小坐标参数化的可执行断言（S05-T03）。

覆盖
----
* **构造性满足**：随机 / 固定的 8 维参数点 → ``to_joint_config`` →
  ``relative_pose``，**逐点**位置残差 ≤1e-6 m、姿态残差 ≤1e-6 rad
  （不是统计口径）；
* **维度断言**：x 长度不是 8 ⇒ 显式报错并**点名期望 / 实际**；
* **失败必须点名**：follower 不可达 ⇒ ``reason == "unreachable"``；
* **分支连续性**：同一分支热路径不产生切换；``diagnostics.branch_switches``
  与 ``branch_history`` **可观测且自洽**；
* **碰撞走同一个 plant**：与 :func:`rm75_drake.bimanual.dual_scene.check_config_free`
  逐点一致（同一 plant、同一过滤矩阵），而不是各自单臂判定；
* **零 pydrake 依赖**（模块级）：``import rm75_drake.coordinated.parameterization``
  不把 pydrake 拉进 ``sys.modules``。

测试固件（``FEASIBLE_X`` / ``COLLIDING_X``）是**确定性**的：它们由固定随机种子
的搜索得到，并在本文件内被断言（限位内 / 碰撞自由 / 碰撞）——因此它们不是
"碰巧"的数据，而是**被断言的固定场景坐标**。
"""

from __future__ import annotations

import ast
import re
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
DUAL_SCENARIO_PATH = REPO_ROOT / "scenarios" / "dual_arm.yaml"
PARAMETERIZATION_PATH = (
    REPO_ROOT / "src" / "rm75_drake" / "coordinated" / "parameterization.py"
)

#: 相对变换：follower 的 TCP 在 leader 的 TCP 系里沿 +x 偏 0.2 m
#: （"两臂夹持同一物体" 的最小几何；右臂基座在 +x 0.6 m）。
X_LR = np.eye(4)
X_LR[0, 3] = 0.2

#: 逐点残差阈值（构造性满足的判定口径）。
POSITION_TOL = 1e-6
ORIENTATION_TOL = 1e-6

#: 已知**可行**（限位内 + 碰撞自由 + follower 有解）的 8 维参数点。
FEASIBLE_X = (
    np.array(
        [
            0.071785, 1.981218, -2.160796, 2.051393, -1.142630, -0.331872, 4.072276,
            2.571074,
        ]
    ),
    np.array(
        [
            2.396862, -0.339904, 0.543490, -2.174206, 1.053313, 1.813972, 4.061375,
            5.563888,
        ]
    ),
    np.array(
        [
            -0.914594, 1.122509, -2.639060, -1.526235, -1.353330, 0.217796, 0.713405,
            3.135224,
        ]
    ),
    np.array(
        [
            -0.165484, 1.319470, 0.868046, 0.285728, 2.245973, -1.309987, -4.919959,
            2.473848,
        ]
    ),
    np.array(
        [
            0.181811, -0.242497, 0.488343, 1.436267, -1.715518, -0.011972, -4.996538,
            3.224767,
        ]
    ),
)

#: 已知在**夹爪工具偏置**（左 0.144 / 右 0.204 m）下同样可行的参数点。
GRIPPED_X = np.array(
    [
        2.325132, -0.897912, -0.750978, 0.981562, 1.774020, 0.492543, -3.989017,
        1.135594,
    ]
)

#: 已知**碰撞**（限位内、相对位姿残差极小，但两臂在同一 plant 里相碰）的参数点。
COLLIDING_X = np.array(
    [
        -2.830380, -1.377536, 1.060779, 0.322648, -2.073381, 1.956552, -4.295259,
        3.206330,
    ]
)


# ── fixture ─────────────────────────────────────────────────────────────
@pytest.fixture(scope="module")
def dual_scene():
    """双臂（不含夹爪）的单 plant 场景（S04 的装配器）。"""
    from rm75_drake.bimanual.dual_scene import build_dual_scene_from_path

    return build_dual_scene_from_path(DUAL_SCENARIO_PATH)


@pytest.fixture
def parameterization(dual_scene):
    """参数化实例（每次测试独立 ⇒ 分支连续性状态互不污染）。"""
    from rm75_drake.coordinated.parameterization import CoordinatedParameterization

    return CoordinatedParameterization(dual_scene, x_lr=X_LR)


# ── 零 pydrake 依赖 ─────────────────────────────────────────────────────
def test_parameterization_module_has_no_drake_dependency():
    """模块顶层不得 import pydrake / Drake 侧子模块。"""
    source = PARAMETERIZATION_PATH.read_text(encoding="utf-8")
    offenders = []
    for number, line in enumerate(source.splitlines(), start=1):
        if re.match(r"^\s*(import|from)\s+pydrake", line):
            offenders.append((number, line.strip()))
    assert offenders == [], f"parameterization.py 出现了 Drake 导入：{offenders}"

    # Drake 侧依赖只能出现在函数体内（惰性导入）。
    tree = ast.parse(source)
    top_level = []
    for node in tree.body:
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            top_level.append(ast.dump(node))
        elif isinstance(node, ast.Try):
            top_level.append(ast.dump(node))
    joined = " ".join(top_level)
    assert "pydrake" not in joined
    assert "bimanual" not in joined, "Drake 侧子模块不得在模块顶层导入"
    assert "drake_compat" not in joined, "Drake 侧子模块不得在模块顶层导入"


def test_importing_parameterization_does_not_load_pydrake():
    """import 本模块不应把 pydrake 拉进 sys.modules。"""
    code = (
        "import sys; import rm75_drake.coordinated.parameterization; "
        "print('pydrake' in sys.modules)"
    )
    result = subprocess.run(
        [sys.executable, "-c", code],
        cwd=str(REPO_ROOT),
        env={"PYTHONPATH": str(REPO_ROOT / "src"), "PATH": "/usr/bin:/bin"},
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == "False"


# ── 维度与形状断言 ─────────────────────────────────────────────────────
def test_parameter_dimension_is_eight(parameterization):
    """参数化维度恒为 8（leader 7 + follower ψ 1）。"""
    from rm75_drake.coordinated.parameterization import PARAMETER_DIM

    assert PARAMETER_DIM == 8
    assert parameterization.parameter_dim == 8


@pytest.mark.parametrize("bad_size", [0, 7, 9, 14])
def test_wrong_parameter_dimension_is_named(parameterization, bad_size):
    """x 维度不符 ⇒ 显式报错并点名期望 / 实际。"""
    from rm75_drake.coordinated.parameterization import CoordinatedParameterizationError

    x = np.zeros(bad_size)
    with pytest.raises(CoordinatedParameterizationError) as excinfo:
        parameterization.to_joint_config(x)
    error = excinfo.value
    assert error.reason == "invalid_dimension"
    assert "8" in str(error)
    assert str(bad_size) in str(error)


def test_wrong_configuration_dimension_is_named(parameterization):
    """``relative_pose`` 的位形维度不符 ⇒ 显式报错并点名。"""
    from rm75_drake.coordinated.parameterization import CoordinatedParameterizationError

    with pytest.raises(CoordinatedParameterizationError) as excinfo:
        parameterization.relative_pose(np.zeros(7))
    assert excinfo.value.reason == "invalid_dimension"
    assert "14" in str(excinfo.value)


def test_invalid_relative_transform_is_named(dual_scene):
    """X_LR 非法 ⇒ 显式报错并点名期望 / 实际。"""
    from rm75_drake.coordinated.parameterization import (
        CoordinatedParameterization,
        CoordinatedParameterizationError,
    )

    with pytest.raises(CoordinatedParameterizationError) as excinfo:
        CoordinatedParameterization(dual_scene, x_lr=np.eye(3))
    assert excinfo.value.reason == "invalid_transform"
    assert "4×4" in str(excinfo.value)


def test_same_leader_and_follower_is_rejected(dual_scene):
    """leader == follower ⇒ 显式报错。"""
    from rm75_drake.coordinated.parameterization import (
        CoordinatedParameterization,
        CoordinatedParameterizationError,
    )

    with pytest.raises(CoordinatedParameterizationError) as excinfo:
        CoordinatedParameterization(dual_scene, x_lr=X_LR, leader="left", follower="left")
    assert excinfo.value.reason == "invalid_scene"


# ── 构造性满足（R006 的核心） ───────────────────────────────────────────
def test_relative_pose_is_satisfied_pointwise(parameterization):
    """逐点残差 ≤1e-6 m / ≤1e-6 rad（不是统计口径）。"""
    for x in FEASIBLE_X:
        q14 = parameterization.to_joint_config(x)
        assert q14.shape == (14,)
        residual = parameterization.relative_pose(q14)
        assert residual.position_error <= POSITION_TOL, residual
        assert residual.orientation_error <= ORIENTATION_TOL, residual


def test_relative_pose_is_satisfied_pointwise_for_random_points(parameterization):
    """随机参数点（映射成功时）同样逐点成立。"""
    rng = np.random.default_rng(20251116)
    lower = parameterization.dual_limits.lower_array()[:7]
    upper = parameterization.dual_limits.upper_array()[:7]
    checked = 0
    for _ in range(120):
        x = np.concatenate([rng.uniform(lower, upper), [rng.uniform(0.0, 2.0 * np.pi)]])
        try:
            q14 = parameterization.to_joint_config(x)
        except Exception as exc:  # noqa: BLE001 - 无解是正常分支
            assert getattr(exc, "reason", "") in (
                "unreachable",
                "branch_exhausted",
                "limit_violation",
                "wrist_singular",
                "invalid_target",
            ), f"未知的失败标签：{exc}"
            continue
        residual = parameterization.relative_pose(q14)
        assert residual.position_error <= POSITION_TOL, residual
        assert residual.orientation_error <= ORIENTATION_TOL, residual
        checked += 1
    assert checked >= 3, "随机样本里应至少有几个可解点"


def test_relative_pose_residual_shape_and_helpers(parameterization):
    """残差对象：字段齐全 + ``within`` / ``as_dict`` 口径一致。"""
    x = FEASIBLE_X[0]
    q14 = parameterization.to_joint_config(x)
    residual = parameterization.relative_pose(q14)
    payload = residual.as_dict()
    assert set(payload) == {"position_error", "orientation_error", "position_error_vector"}
    assert len(payload["position_error_vector"]) == 3
    assert residual.within(POSITION_TOL, ORIENTATION_TOL)
    assert not residual.within(0.0, 0.0)


def test_swapped_leader_follower_also_satisfies_constraint(dual_scene):
    """leader / follower 互换时约束同样**构造性**成立（X_LR 语义随之翻转）。"""
    from rm75_drake.coordinated.parameterization import (
        CoordinatedParameterization,
        CoordinatedParameterizationError,
    )

    swapped = CoordinatedParameterization(
        dual_scene, x_lr=X_LR, leader="right", follower="left"
    )
    rng = np.random.default_rng(7)
    lower = swapped.dual_limits.lower_array()[:7]
    upper = swapped.dual_limits.upper_array()[:7]
    checked = 0
    for _ in range(200):
        x = np.concatenate([rng.uniform(lower, upper), [rng.uniform(0.0, 2.0 * np.pi)]])
        try:
            q14 = swapped.to_joint_config(x)
        except CoordinatedParameterizationError:
            continue
        residual = swapped.relative_pose(q14)
        assert residual.position_error <= POSITION_TOL, residual
        assert residual.orientation_error <= ORIENTATION_TOL, residual
        checked += 1
        if checked >= 3:
            break
    assert checked >= 1, "互换指派后也应存在可解点"


# ── 失败路径（反例断言） ───────────────────────────────────────────────
def test_unreachable_follower_is_named(parameterization):
    """leader 零位 ⇒ follower 目标不可达 ⇒ 显式失败且 reason 可读。"""
    from rm75_drake.coordinated.parameterization import CoordinatedParameterizationError

    x = np.zeros(8)
    with pytest.raises(CoordinatedParameterizationError) as excinfo:
        parameterization.to_joint_config(x)
    error = excinfo.value
    assert error.reason == "unreachable"
    assert "可达" in str(error)
    assert parameterization.diagnostics.last_reason == "unreachable"
    assert parameterization.diagnostics.failure_count == 1


def test_unreachable_point_is_not_feasible(parameterization):
    """无解点 ⇒ ``is_feasible`` 为 False（不抛未分类异常）。"""
    assert parameterization.is_feasible(np.zeros(8)) is False


# ── 分支 / ψ 连续性 ────────────────────────────────────────────────────
def test_branch_history_is_observable_and_consistent(parameterization):
    """分支切换次数与 ``branch_history`` 自洽（可观测）。"""
    for x in FEASIBLE_X:
        parameterization.to_joint_config(x)
    diagnostics = parameterization.diagnostics
    history = diagnostics.branch_history
    assert len(history) == diagnostics.solve_count == len(FEASIBLE_X)
    switches = sum(
        1
        for previous, current in zip(history, history[1:])
        if previous is not None and current is not None and previous != current
    )
    assert diagnostics.branch_switches == switches
    assert diagnostics.psi_switches == 0, "ψ 由参数给定 ⇒ 构造上不可能切换"
    assert diagnostics.last_branch is not None


def test_repeated_solve_keeps_the_same_branch(parameterization):
    """同一参数重复求解 ⇒ 分支不变（连续性热路径，不产生切换）。"""
    x = FEASIBLE_X[0]
    parameterization.to_joint_config(x)
    first = parameterization.diagnostics.branch_history[-1]
    switches = parameterization.diagnostics.branch_switches
    parameterization.to_joint_config(x)
    assert parameterization.diagnostics.branch_history[-1] == first
    assert parameterization.diagnostics.branch_switches == switches


def test_diagnostics_serialisable(parameterization):
    """diagnostics 可 JSON 序列化（进 planner 的 diagnostics 字段）。"""
    import json

    parameterization.to_joint_config(FEASIBLE_X[0])
    payload = parameterization.diagnostics.as_dict()
    assert json.loads(json.dumps(payload))["solve_count"] == 1


# ── 碰撞：同一个 plant ─────────────────────────────────────────────────
def test_collision_check_uses_the_same_plant(parameterization, dual_scene):
    """碰撞判定与 ``check_config_free``（同一 plant）逐点一致。"""
    from rm75_drake.bimanual.dual_scene import check_config_free

    free_x = FEASIBLE_X[0]
    busy_x = COLLIDING_X
    for x, expected_free in ((free_x, True), (busy_x, False)):
        q14 = parameterization.to_joint_config(x)
        assert parameterization.is_collision_free(q14) is expected_free
        assert check_config_free(dual_scene, q14) is expected_free


def test_colliding_point_is_rejected_while_constraint_holds(parameterization):
    """碰撞点：限位内 + 残差极小，但 ``is_feasible`` 为 False（碰撞是判别项）。"""
    x = COLLIDING_X
    q14 = parameterization.to_joint_config(x)
    assert parameterization.within_limits(q14)
    residual = parameterization.relative_pose(q14)
    assert residual.position_error <= POSITION_TOL
    assert residual.orientation_error <= ORIENTATION_TOL
    assert parameterization.is_collision_free(q14) is False
    assert parameterization.is_feasible(x) is False


def test_feasible_points_are_accepted(parameterization):
    """已知可行点 ⇒ ``is_feasible`` 为 True。"""
    for x in FEASIBLE_X:
        assert parameterization.is_feasible(x) is True


# ── 工具偏置口径 ───────────────────────────────────────────────────────
def test_tool_offset_changes_the_mapping_but_not_the_constraint(dual_scene):
    """工具偏置改变 TCP ⇒ 映射改变，但相对位姿约束仍**构造性**成立。"""
    from rm75_drake.coordinated.parameterization import CoordinatedParameterization

    bare = CoordinatedParameterization(dual_scene, x_lr=X_LR)
    gripped = CoordinatedParameterization(
        dual_scene, x_lr=X_LR, tool_offsets={"left": 0.144, "right": 0.204}
    )
    x = GRIPPED_X
    q_bare = bare.to_joint_config(x)
    q_gripped = gripped.to_joint_config(x)
    assert not np.allclose(q_bare, q_gripped)
    residual = gripped.relative_pose(q_gripped)
    assert residual.position_error <= POSITION_TOL
    assert residual.orientation_error <= ORIENTATION_TOL
