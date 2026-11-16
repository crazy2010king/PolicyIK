"""双臂搬运 / 交接原语的可执行断言（S06-T05）。

覆盖
----
**第一层（快速单测，不构造 plant，秒级）**

* ``X_LR`` **非法**时的点名报错：形状非 4×4 / **奇异** / 含 NaN /
  最后一行不是 ``[0, 0, 0, 1]`` / 旋转部分非正交 / 镜像；
* 关节角维度错（6 维 / 8 维）与含 NaN ⇒ 点名报错；
* ``leader == follower`` / 侧别非法 ⇒ 点名报错；
* **S05 的 ``plan_coordinated`` 公开签名未被改动**（参数名与顺序钉死）；
* **零 pydrake 依赖**：``import rm75_drake.tasks.bimanual`` 后
  ``'pydrake' not in sys.modules``（子进程实测）；
* **依赖方向**：``tasks/bimanual.py`` **不 import** ``rm75_drake.drivers``
  （AST 静态扫描）。

**第二层（真实双臂场景的端到端集成用例，**同一个 plant**）**

* ``plan_carry``：成功、**零逃逸**、**逐点**相对位姿残差 ≤1e-6 m / ≤1e-6 rad、
  共享时间轴、诊断四口径键齐全；
* ``plan_handover``：两阶段均成功、交接点残差 ≤1e-6、
  ``x_lr_inverse == inv(x_lr)``、阶段 2 在**角色互换**约定下逐点残差达标、
  交接点 **TCP 连续性** ≤1e-9 m、新 leader（阶段 2 的 leader 臂）关节连续；
* 失败路径：阶段 2 的目标 IK 不可达 ⇒ ``stage="ik"`` 且 ``reason`` 落在
  解析 IK 的稳定词表内。

集成用例的**总耗时预算 < 60 s**（用 ``pytest --durations=10`` 自查）。
"""

from __future__ import annotations

import ast
import inspect
import subprocess
import sys
from pathlib import Path
from typing import Any, List

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
DUAL_SCENARIO_PATH = REPO_ROOT / "scenarios" / "dual_arm.yaml"
BIMANUAL_PATH = REPO_ROOT / "src" / "rm75_drake" / "tasks" / "bimanual.py"

#: 相对变换：follower 的 TCP 在 leader 的 TCP 系里沿 +x 偏 0.2 m
#: （"两臂夹持同一物体" 的最小几何；右臂基座在 +x 0.6 m）。
X_LR = np.eye(4)
X_LR[0, 3] = 0.2

#: 已知**可行**的 leader 关节角（与 S05 的集成用例同款起点）。
LEADER_START = np.array(
    [
        0.071785, 1.981218, -2.160796, 2.051393, -1.142630, -0.331872, 4.072276,
    ]
)

#: 逐点残差阈值（构造性满足的判定口径，**不可放宽**）。
POSITION_TOL = 1e-6
ORIENTATION_TOL = 1e-6

#: 交接点的 **TCP 连续性**阈值（m）—— 被携带物在交接瞬间不得位移。
TCP_CONTINUITY_TOL = 1e-9


# ── 工具 ───────────────────────────────────────────────────────────────
@pytest.fixture(scope="module")
def dual_scene():
    """双臂（不含夹爪）的**同一个 plant** 场景（S04 的装配器）。"""
    from rm75_drake.bimanual.dual_scene import build_dual_scene_from_path

    return build_dual_scene_from_path(DUAL_SCENARIO_PATH)


def _integration_options(**overrides: Any):
    """集成用例的选项（**显式覆盖**为小规模实例）。"""
    from rm75_drake.coordinated.planner import CoordinatedPlanOptions

    kwargs: dict = {
        "num_regions": 3,
        "path_step": 0.005,
        "psi_grid": 17,
        "collision_edge_step_size": 0.02,
        "dt": 0.005,
        "t0": 0.0,
        "allow_degradation": True,
    }
    kwargs.update(overrides)
    return CoordinatedPlanOptions(**kwargs)


def _delta(value: float) -> np.ndarray:
    """单轴小位移（rad）。"""
    out = np.zeros(7)
    out[0] = float(value)
    return out


# ── 第一层：输入校验 ───────────────────────────────────────────────────
def test_invalid_x_lr_is_named() -> None:
    """``X_LR`` 非法 ⇒ **点名报错**（形状 / 奇异 / NaN / 最后一行 / 镜像）。"""
    from rm75_drake.tasks.bimanual import BimanualPlanError, plan_carry

    singular = np.eye(4)
    singular[:3, :3] = 0.0  # 旋转块全零 ⇒ det = 0 ⇒ 奇异

    mirrored = np.eye(4)
    mirrored[2, 2] = -1.0  # 反射：旋转部分行列式为负

    bad_last_row = np.eye(4)
    bad_last_row[3, 0] = 0.5

    not_orthogonal = np.eye(4)
    not_orthogonal[0, 1] = 0.3

    cases = [
        (np.eye(3), "4×4"),
        (singular, "奇异"),
        (np.full((4, 4), np.nan), "NaN"),
        (bad_last_row, "[0, 0, 0, 1]"),
        (mirrored, "镜像"),
        (not_orthogonal, "正交阵"),
    ]
    for matrix, expected in cases:
        with pytest.raises(BimanualPlanError) as excinfo:
            plan_carry(
                None,
                x_lr=matrix,
                start_leader_q=LEADER_START,
                goal_leader_q=LEADER_START,
            )
        message = str(excinfo.value)
        assert expected in message, f"错误信息没有点名「{expected}」：{message}"
        assert "X_LR" in message
        assert excinfo.value.reason == "invalid_input"


def test_invalid_joint_vectors_are_named() -> None:
    """关节角维度错 / 含 NaN ⇒ 点名报错。"""
    from rm75_drake.tasks.bimanual import BimanualPlanError, plan_carry

    with pytest.raises(BimanualPlanError) as excinfo:
        plan_carry(
            None,
            x_lr=X_LR,
            start_leader_q=LEADER_START[:6],
            goal_leader_q=LEADER_START,
        )
    assert "start_leader_q" in str(excinfo.value)
    assert "7 维" in str(excinfo.value)

    nan_start = LEADER_START.copy()
    nan_start[2] = float("nan")
    with pytest.raises(BimanualPlanError) as excinfo:
        plan_carry(
            None,
            x_lr=X_LR,
            start_leader_q=nan_start,
            goal_leader_q=LEADER_START,
        )
    assert "start_leader_q" in str(excinfo.value)
    assert "NaN" in str(excinfo.value)


def test_invalid_side_assignment_is_named() -> None:
    """``leader == follower`` / 侧别非法 ⇒ 点名报错。"""
    from rm75_drake.tasks.bimanual import BimanualPlanError, plan_carry

    with pytest.raises(BimanualPlanError) as excinfo:
        plan_carry(
            None,
            x_lr=X_LR,
            start_leader_q=LEADER_START,
            goal_leader_q=LEADER_START,
            leader="left",
            follower="left",
        )
    assert "不能相同" in str(excinfo.value)

    with pytest.raises(BimanualPlanError) as excinfo:
        plan_carry(
            None,
            x_lr=X_LR,
            start_leader_q=LEADER_START,
            goal_leader_q=LEADER_START,
            leader="middle",
            follower="right",
        )
    assert "'left'" in str(excinfo.value) and "'right'" in str(excinfo.value)


def test_plan_coordinated_public_signature_is_unchanged() -> None:
    """**不得**改 S05 的 ``plan_coordinated`` 公开签名（钉死参数名与顺序）。"""
    from rm75_drake.coordinated.planner import plan_coordinated

    signature = inspect.signature(plan_coordinated)
    names = [
        name
        for name, parameter in signature.parameters.items()
        if parameter.kind
        in (parameter.POSITIONAL_OR_KEYWORD, parameter.KEYWORD_ONLY)
    ]
    assert names == [
        "bundle",
        "x_lr",
        "start_leader_q",
        "goal_leader_q",
        "leader",
        "follower",
        "tool_offsets",
        "psi_start",
        "psi_goal",
        "options",
    ]


def test_importing_bimanual_does_not_load_pydrake() -> None:
    """``import rm75_drake.tasks.bimanual`` **不得**把 pydrake 拉进 sys.modules。"""
    code = (
        "import sys; import rm75_drake.tasks.bimanual; "
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


def test_bimanual_module_does_not_import_drivers() -> None:
    """``tasks/*`` **不得** import ``rm75_drake.drivers``（P002 边界纪律）。"""
    tree = ast.parse(BIMANUAL_PATH.read_text(encoding="utf-8"))
    imported: List[str] = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported.extend(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            module = node.module or ""
            imported.append(module)
            imported.extend(
                f"{module}.{alias.name}" for alias in node.names if module
            )
    offenders = [
        name
        for name in imported
        if "drivers" in name and "rm75_drake" in name
    ]
    assert not offenders, f"tasks/bimanual.py 不得 import drivers：{offenders}"


# ── 第二层：真实双臂场景 ───────────────────────────────────────────────
def test_carry_plan_succeeds_with_zero_escape_and_tiny_residual(dual_scene) -> None:
    """``plan_carry``：成功 + **零逃逸** + **逐点**残差达标 + 共享时间轴。"""
    from rm75_drake.coordinated.parameterization import CoordinatedParameterization
    from rm75_drake.tasks.bimanual import plan_carry

    plan = plan_carry(
        dual_scene,
        x_lr=X_LR,
        start_leader_q=LEADER_START,
        goal_leader_q=LEADER_START + _delta(0.02),
        options=_integration_options(),
    )
    assert plan.success is True, plan.message
    assert plan.reason == "" and plan.stage == ""
    assert plan.dual_trajectory is not None
    assert bool(plan) is True

    # ① 零逃逸（**硬要求**）。
    assert plan.num_escapes == 0
    assert plan.diagnostics["collision"]["num_escapes"] == 0
    assert plan.degraded is False

    # ② 相对位姿残差：**逐点** ≤1e-6 m / ≤1e-6 rad。
    parameterization = CoordinatedParameterization(dual_scene, x_lr=X_LR)
    q14 = plan.dual_trajectory.q_matrix()
    assert q14.shape[1] == 14
    for row in q14:
        residual = parameterization.relative_pose(row)
        assert residual.position_error <= POSITION_TOL
        assert residual.orientation_error <= ORIENTATION_TOL

    # ③ 诊断四口径键齐全。
    for key in (
        "relative_pose_residual_max_position_m",
        "relative_pose_residual_max_orientation_rad",
        "num_escapes",
        "degraded",
    ):
        assert key in plan.diagnostics, f"diagnostics 缺少关键字段：{key}"
    assert plan.relative_pose_residual_max_position_m <= POSITION_TOL
    assert plan.relative_pose_residual_max_orientation_rad <= ORIENTATION_TOL

    # ④ 共享时间轴（两条 7 维轨迹的 t_abs / dt 逐点相同）。
    left = plan.dual_trajectory.left
    right = plan.dual_trajectory.right
    assert left.q_matrix().shape[1] == 7
    assert right.q_matrix().shape[1] == 7
    left_times = np.asarray([sample.t_abs for sample in left.samples], dtype=float)
    right_times = np.asarray([sample.t_abs for sample in right.samples], dtype=float)
    assert np.array_equal(left_times, right_times)


def test_handover_two_phases_succeed_and_switch_residual_is_tiny(dual_scene) -> None:
    """``plan_handover``：两阶段均成功 + 交接点残差达标 + 角色互换后逐点残差达标。"""
    from rm75_drake.coordinated.parameterization import CoordinatedParameterization
    from rm75_drake.tasks.bimanual import plan_handover

    plan = plan_handover(
        dual_scene,
        x_lr=X_LR,
        start_leader_q=LEADER_START,
        handover_leader_q=LEADER_START + _delta(0.02),
        goal_leader_q=LEADER_START + _delta(0.04),
        options=_integration_options(),
    )
    assert plan.success is True, plan.message
    assert plan.reason == "" and plan.stage == ""
    assert plan.phase1 is not None and plan.phase2 is not None
    assert plan.phase1.success is True and plan.phase2.success is True

    # ① 交接点残差 ≤ 1e-6（与 S05 的 R006 口径一致）。
    assert plan.switch_relative_residual_position_m <= POSITION_TOL
    assert plan.switch_relative_residual_orientation_rad <= ORIENTATION_TOL

    # ② ``x_lr_inverse`` 就是 ``inv(x_lr)``。
    assert np.allclose(plan.x_lr_inverse, np.linalg.inv(plan.x_lr), atol=1e-12)
    assert np.allclose(plan.x_lr, X_LR, atol=1e-12)

    # ③ 零逃逸（两个阶段合计）。
    assert plan.num_escapes == 0
    assert plan.phase1.num_escapes == 0
    assert plan.phase2.num_escapes == 0

    # ④ 阶段 1 用 X_LR、阶段 2 用 inv(X_LR)（**角色互换**），各自逐点残差达标。
    phase1_parameterization = CoordinatedParameterization(dual_scene, x_lr=X_LR)
    for row in plan.phase1.dual_trajectory.q_matrix():
        residual = phase1_parameterization.relative_pose(row)
        assert residual.position_error <= POSITION_TOL
        assert residual.orientation_error <= ORIENTATION_TOL

    phase2_parameterization = CoordinatedParameterization(
        dual_scene, x_lr=plan.x_lr_inverse, leader="right", follower="left"
    )
    for row in plan.phase2.dual_trajectory.q_matrix():
        residual = phase2_parameterization.relative_pose(row)
        assert residual.position_error <= POSITION_TOL
        assert residual.orientation_error <= ORIENTATION_TOL

    # ⑤ 交接点的 **TCP 连续性**：被携带物在交接瞬间不得位移。
    tcp_gap = plan.diagnostics["switch_tcp_discontinuity"]
    assert float(tcp_gap["left"]["position_m"]) <= TCP_CONTINUITY_TOL
    assert float(tcp_gap["right"]["position_m"]) <= TCP_CONTINUITY_TOL
    assert float(plan.diagnostics["switch_tcp_position_discontinuity_m"]) <= (
        TCP_CONTINUITY_TOL
    )

    # ⑥ 新 leader（阶段 2 的 leader 臂 = 右臂）**关节级连续**（起点即交接位）。
    joint_gap = plan.diagnostics["switch_joint_discontinuity_max_rad"]
    assert float(joint_gap["right"]) == pytest.approx(0.0, abs=1e-9)


def test_handover_names_unreachable_phase_two_goal(dual_scene) -> None:
    """阶段 2 的目标 IK 不可达 ⇒ ``stage="ik"`` + 解析 IK 词表的 ``reason``。"""
    from rm75_drake.coordinated.planner import IK_FAILURE_REASONS
    from rm75_drake.tasks.bimanual import plan_handover

    unreachable_goal = LEADER_START.copy()
    unreachable_goal[0] += 1.5  # leader 伸到另一条臂的工作空间之外

    plan = plan_handover(
        dual_scene,
        x_lr=X_LR,
        start_leader_q=LEADER_START,
        handover_leader_q=LEADER_START + _delta(0.02),
        goal_leader_q=unreachable_goal,
        options=_integration_options(),
    )
    assert plan.success is False
    assert plan.stage == "ik"
    assert plan.reason in IK_FAILURE_REASONS
    assert plan.message
    assert plan.phase1 is not None and plan.phase1.success is True
    assert plan.phase2 is None
