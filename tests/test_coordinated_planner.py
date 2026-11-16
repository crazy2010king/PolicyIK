"""协调规划 API 的可执行断言（S05-T05）。

覆盖（**两层**）
----------------
**第一层（快速单测，不构造 plant，秒级）**

* :class:`CoordinatedPlanOptions` 的参数校验：``num_regions<=0`` /
  ``path_step<=0`` / ``dt<=0`` / ``adjacency_threshold<=0`` /
  ``collision_edge_step_size<=0`` / ``psi_grid<=0`` / NaN / Inf ⇒ 抛
  :class:`CoordinatedPlanError` 且 ``reason == "invalid_input"``；
* 默认值钉住**性能预算**（``path_step=0.02`` / ``psi_grid=17`` /
  ``collision_edge_step_size=0.02`` / ``num_regions=3``）；
* :func:`plan_coordinated` 的**输入非法路径**（leader 关节角维度错 / 含
  NaN / leader == follower / X_LR 非法）⇒ 返回 ``success=False`` 且
  ``stage`` / ``reason`` **点名**，**不抛未分类异常**；
* 结果对象契约（成功 / 失败两种形态）：``message`` 非空、失败时
  ``left_trajectory is None``、``diagnostics`` 的计时字段**存在且非负**；
* 阶段词表：``STAGES`` 含 ``parameterize`` + S02 的
  ``ik`` / ``regions`` / ``gcs`` / ``retime`` / ``collision``；
* **零 pydrake 依赖**（模块级）：AST 静态断言 + 子进程 ``sys.modules`` 断言
  （沿用 S05-T03/T04 的同款写法）。

**第二层（真实双臂场景的端到端集成用例，**同一个 plant**）**

* 一个**小规模紧协调任务**（单轴 0.02 rad）：成功、**零逃逸**、**共享
  时间轴**、**相对位姿残差逐点** ≤1e-6 m / ≤1e-6 rad、``diagnostics``
  关键字段齐全；
* 失败路径反例：follower 不可达 ⇒ ``stage="parameterize"`` 且 ``reason``
  ∈ 解析 IK 词表；``allow_degradation=False`` + 区域被破坏 ⇒
  ``stage="regions"`` 且 ``reason`` 点名。

集成用例的**总耗时预算 < 60 s**（用 ``pytest --durations=10`` 自查）。
"""

from __future__ import annotations

import ast
import re
import subprocess
import sys
from pathlib import Path
from typing import Any, Dict, List

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
DUAL_SCENARIO_PATH = REPO_ROOT / "scenarios" / "dual_arm.yaml"
PLANNER_PATH = (
    REPO_ROOT / "src" / "rm75_drake" / "coordinated" / "planner.py"
)

#: 相对变换：follower 的 TCP 在 leader 的 TCP 系里沿 +x 偏 0.2 m
#: （"两臂夹持同一物体" 的最小几何；右臂基座在 +x 0.6 m）。
X_LR = np.eye(4)
X_LR[0, 3] = 0.2

#: 已知**可行**的 leader 关节角（取自 S05-T03 的 ``FEASIBLE_X[0][:7]``，
#: 该点在限位内、碰撞自由、follower 有解）。
LEADER_START = np.array(
    [
        0.071785, 1.981218, -2.160796, 2.051393, -1.142630, -0.331872, 4.072276,
    ]
)

#: 紧协调终点：**单轴 0.02 rad** 的小位移（相邻关节角）。
LEADER_GOAL = LEADER_START.copy()
LEADER_GOAL[0] += 0.02

#: 逐点残差阈值（构造性满足的判定口径，**不可放宽**）。
POSITION_TOL = 1e-6
ORIENTATION_TOL = 1e-6


# ── 工具 ───────────────────────────────────────────────────────────────
def _joint_names() -> List[str]:
    from rm75_drake.coordinated.limits import canonical_joint_names

    return list(canonical_joint_names())


class _FakeBundle:
    """**不构造 plant** 的最小替身（只提供关节名）。

    参数化对象的构造期只需要 ``bundle.joint_names``；Drake 侧的一切
    （基座位姿 / 碰撞）都是**惰性**的 ⇒ 第一层用例可以零 plant 运行。
    """

    def __init__(self) -> None:
        self.joint_names = _joint_names()
        self.gripper_joint_names: List[str] = []


# ── 第一层：选项校验 ───────────────────────────────────────────────────
@pytest.mark.parametrize(
    "field,value",
    [
        ("num_regions", 0),
        ("num_regions", -1),
        ("path_step", 0.0),
        ("path_step", -0.1),
        ("dt", 0.0),
        ("dt", -0.005),
        ("adjacency_threshold", 0.0),
        ("adjacency_threshold", -1.0),
        ("collision_edge_step_size", 0.0),
        ("collision_edge_step_size", -0.01),
        ("psi_grid", 0),
        ("psi_grid", -3),
        ("path_step", float("nan")),
        ("path_step", float("inf")),
        ("dt", float("nan")),
        ("adjacency_threshold", float("nan")),
        ("collision_edge_step_size", float("inf")),
        ("t0", float("nan")),
        ("num_regions", float("nan")),
        ("psi_grid", float("nan")),
        ("region_max_expand", 0),
    ],
)
def test_options_reject_invalid_values(field: str, value: Any) -> None:
    """非法选项 ⇒ 抛 :class:`CoordinatedPlanError` 且 ``reason`` 稳定。"""
    from rm75_drake.coordinated.planner import (
        CoordinatedPlanError,
        CoordinatedPlanOptions,
    )

    with pytest.raises(CoordinatedPlanError) as excinfo:
        CoordinatedPlanOptions(**{field: value})
    assert excinfo.value.reason == "invalid_input"
    assert field in str(excinfo.value)


def test_options_defaults_pin_the_performance_budget() -> None:
    """默认值必须**足够粗**（否则 ψ 扫描 × 路径点数 × 碰撞查询会失控）。"""
    from rm75_drake.coordinated.planner import CoordinatedPlanOptions

    options = CoordinatedPlanOptions()
    assert options.num_regions == 3
    assert options.path_step == pytest.approx(0.02)
    assert options.psi_grid == 17
    assert options.collision_edge_step_size == pytest.approx(0.02)
    assert options.dt == pytest.approx(0.005)


def test_options_accept_the_documented_knobs() -> None:
    """合法选项必须被接受（默认值 + 显式覆盖）。"""
    from rm75_drake.coordinated.planner import CoordinatedPlanOptions

    options = CoordinatedPlanOptions(
        num_regions=2,
        path_step=0.01,
        psi_grid=9,
        collision_edge_step_size=0.05,
        dt=0.01,
        t0=1.5,
        allow_degradation=False,
    )
    assert options.num_regions == 2
    assert options.t0 == pytest.approx(1.5)
    assert options.allow_degradation is False


# ── 第一层：输入非法路径 ───────────────────────────────────────────────
@pytest.mark.parametrize(
    "case",
    ["wrong_dimension", "nan_leader_q", "same_leader_follower", "invalid_x_lr"],
)
def test_plan_coordinated_names_invalid_inputs(case: str) -> None:
    """输入非法 ⇒ ``success=False`` + ``stage`` / ``reason`` 点名。"""
    from rm75_drake.coordinated.planner import plan_coordinated

    start = LEADER_START.copy()
    goal = LEADER_GOAL.copy()
    leader = "left"
    follower = "right"
    x_lr = X_LR
    if case == "wrong_dimension":
        start = LEADER_START[:6]
    elif case == "nan_leader_q":
        goal = LEADER_GOAL.copy()
        goal[2] = float("nan")
    elif case == "same_leader_follower":
        follower = "left"
    elif case == "invalid_x_lr":
        x_lr = np.eye(3)

    result = plan_coordinated(
        _FakeBundle(),
        x_lr=x_lr,
        start_leader_q=start,
        goal_leader_q=goal,
        leader=leader,
        follower=follower,
    )
    assert result.success is False
    assert result.stage == "parameterize"
    assert result.reason == "invalid_input"
    assert result.message
    assert result.left_trajectory is None
    assert result.right_trajectory is None


def test_failure_result_carries_non_negative_timings() -> None:
    """失败结果：**计时字段存在且非负**（诊断可复核）。"""
    from rm75_drake.coordinated.planner import plan_coordinated

    result = plan_coordinated(
        _FakeBundle(),
        x_lr=X_LR,
        start_leader_q=LEADER_START[:5],
        goal_leader_q=LEADER_GOAL,
    )
    assert result.success is False
    diagnostics = dict(result.diagnostics)
    assert "total_duration_s" in diagnostics
    assert float(diagnostics["total_duration_s"]) >= 0.0
    stage_payload = diagnostics.get("parameterize")
    assert isinstance(stage_payload, dict)
    assert float(stage_payload["duration_s"]) >= 0.0
    assert stage_payload["success"] is False
    assert stage_payload["reason"] == "invalid_input"
    assert float(result.duration) >= 0.0


def test_result_dataclass_contract() -> None:
    """结果对象：字段齐全 + ``bool(result) == result.success``。"""
    from rm75_drake.coordinated.planner import CoordinatedPlanResult

    ok = CoordinatedPlanResult(success=True, message="ok")
    assert bool(ok) is True
    assert ok.reason == "" and ok.stage == ""
    assert ok.left_trajectory is None and ok.right_trajectory is None
    assert ok.dual_trajectory is None
    assert dict(ok.diagnostics) == {}

    bad = CoordinatedPlanResult(success=False, message="bad", reason="r", stage="s")
    assert bool(bad) is False
    assert bad.message == "bad"
    assert bad.reason == "r" and bad.stage == "s"


def test_stage_vocabulary_contains_parameterize_and_s02_stages() -> None:
    """阶段词表 = S02 词表 + 新增 ``parameterize``。"""
    from rm75_drake.coordinated.planner import STAGES

    for stage in ("parameterize", "ik", "regions", "gcs", "retime", "collision"):
        assert stage in STAGES
    assert len(set(STAGES)) == len(STAGES)


# ── 第一层：零 pydrake 依赖 ────────────────────────────────────────────
def test_planner_module_has_no_drake_dependency() -> None:
    """模块顶层不得 import pydrake / Drake 侧子模块。"""
    source = PLANNER_PATH.read_text(encoding="utf-8")
    offenders = []
    for number, line in enumerate(source.splitlines(), start=1):
        if re.match(r"^\s*(import|from)\s+pydrake", line):
            offenders.append((number, line.strip()))
    assert offenders == [], f"planner.py 出现了 Drake 导入：{offenders}"

    tree = ast.parse(source)
    top_level: List[str] = []
    for node in tree.body:
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            top_level.append(ast.dump(node))
        elif isinstance(node, ast.Try):
            top_level.append(ast.dump(node))
    joined = " ".join(top_level)
    assert "pydrake" not in joined
    assert "bimanual" not in joined, "Drake 侧子模块不得在模块顶层导入"
    assert "drake_compat" not in joined, "Drake 侧子模块不得在模块顶层导入"


def test_importing_planner_does_not_load_pydrake() -> None:
    """import 本模块不应把 pydrake 拉进 ``sys.modules``。"""
    code = (
        "import sys; import rm75_drake.coordinated.planner; "
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


# ── 第二层：真实双臂场景 ───────────────────────────────────────────────
@pytest.fixture(scope="module")
def dual_scene():
    """双臂（不含夹爪）的**同一个 plant** 场景（S04 的装配器）。"""
    from rm75_drake.bimanual.dual_scene import build_dual_scene_from_path

    return build_dual_scene_from_path(DUAL_SCENARIO_PATH)


def _integration_options(**overrides: Any):
    """集成用例的选项（**显式覆盖**为小规模实例）。"""
    from rm75_drake.coordinated.planner import CoordinatedPlanOptions

    kwargs: Dict[str, Any] = {
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


def test_tight_coordination_plan_succeeds_with_zero_escape(dual_scene) -> None:
    """端到端：成功 + **零逃逸** + **共享时间轴** + **逐点残差**达标。"""
    from rm75_drake.coordinated.parameterization import CoordinatedParameterization
    from rm75_drake.coordinated.planner import plan_coordinated

    result = plan_coordinated(
        dual_scene,
        x_lr=X_LR,
        start_leader_q=LEADER_START,
        goal_leader_q=LEADER_GOAL,
        options=_integration_options(),
    )
    assert result.success is True, result.message
    assert result.reason == "" and result.stage == ""
    assert result.left_trajectory is not None and result.right_trajectory is not None

    diagnostics = dict(result.diagnostics)

    # ① 零逃逸（**硬要求**）。
    assert diagnostics["num_escapes"] == 0
    assert diagnostics["collision"]["num_escapes"] == 0
    assert diagnostics["collision"]["num_samples"] > 0

    # ② 共享时间轴：两条 7 维轨迹的 ``t_abs`` / ``dt`` **逐点相同**。
    left = result.left_trajectory
    right = result.right_trajectory
    assert left.q_matrix().shape[1] == 7
    assert right.q_matrix().shape[1] == 7
    left_times = np.asarray([sample.t_abs for sample in left.samples], dtype=float)
    right_times = np.asarray([sample.t_abs for sample in right.samples], dtype=float)
    assert left_times.shape == right_times.shape
    assert np.array_equal(left_times, right_times)
    assert float(left.dt) == float(right.dt)
    assert float(left.dt) > 0.0

    # ③ 相对位姿残差：**逐点** ≤1e-6 m / ≤1e-6 rad。
    parameterization = CoordinatedParameterization(dual_scene, x_lr=X_LR)
    q14 = result.dual_trajectory.q_matrix()
    assert q14.shape[1] == 14
    for row in q14:
        residual = parameterization.relative_pose(row)
        assert residual.position_error <= POSITION_TOL
        assert residual.orientation_error <= ORIENTATION_TOL

    # ④ diagnostics 关键字段齐全且非负。
    for key in (
        "num_regions",
        "num_escapes",
        "branch_switches",
        "psi_switches",
        "relative_pose_residual_max_position_m",
        "relative_pose_residual_max_orientation_rad",
        "region_duration_s",
        "gcs_duration_s",
        "map_duration_s",
        "retime_duration_s",
        "collision_duration_s",
        "total_duration_s",
    ):
        assert key in diagnostics, f"diagnostics 缺少关键字段：{key}"
        assert float(diagnostics[key]) >= 0.0
    assert "degraded" in diagnostics
    assert diagnostics["num_regions"] >= 1
    assert diagnostics["relative_pose_residual_max_position_m"] <= POSITION_TOL
    assert diagnostics["relative_pose_residual_max_orientation_rad"] <= ORIENTATION_TOL
    assert float(diagnostics["total_duration_s"]) > 0.0


def test_unreachable_follower_is_named_in_the_ik_vocabulary(dual_scene) -> None:
    """终点不可达 ⇒ ``stage="parameterize"`` + 解析 IK 词表的 ``reason``。"""
    from rm75_drake.coordinated.planner import IK_FAILURE_REASONS, plan_coordinated

    unreachable_goal = LEADER_START.copy()
    unreachable_goal[0] += 1.5  # leader 伸到 follower 工作空间之外

    result = plan_coordinated(
        dual_scene,
        x_lr=X_LR,
        start_leader_q=LEADER_START,
        goal_leader_q=unreachable_goal,
        options=_integration_options(),
    )
    assert result.success is False
    assert result.stage == "parameterize"
    assert result.reason in IK_FAILURE_REASONS
    assert result.message
    assert result.left_trajectory is None and result.right_trajectory is None
    assert float(result.diagnostics["total_duration_s"]) >= 0.0


def test_broken_regions_without_degradation_is_named(dual_scene, monkeypatch) -> None:
    """``allow_degradation=False`` + 区域被破坏 ⇒ ``stage="regions"`` + reason。"""
    from rm75_drake.coordinated import planner as planner_module
    from rm75_drake.coordinated.regions import MinimalRegionError

    def _broken(*args: Any, **kwargs: Any):
        raise MinimalRegionError("[test] 人为破坏区域生成", reason="seed_infeasible")

    monkeypatch.setattr(planner_module, "generate_minimal_region", _broken)

    result = planner_module.plan_coordinated(
        dual_scene,
        x_lr=X_LR,
        start_leader_q=LEADER_START,
        goal_leader_q=LEADER_GOAL,
        options=_integration_options(allow_degradation=False),
    )
    assert result.success is False
    assert result.stage == "regions"
    assert result.reason == "seed_infeasible"
    assert result.message
    assert result.left_trajectory is None and result.right_trajectory is None
    assert result.diagnostics["degraded"] is True
    assert result.diagnostics["degradation_reason"] == "seed_infeasible"


def test_broken_regions_with_degradation_falls_back_to_sampling_baseline(
    dual_scene, monkeypatch
) -> None:
    """``allow_degradation=True`` + 区域被破坏 ⇒ **显式降级**到采样基线。

    降级必须**可观测**（``degraded=True`` + ``degradation_reason``），且降级
    路径同样满足硬要求（零逃逸 + 逐点残差达标）——**绝不返回半成品**。
    """
    from rm75_drake.coordinated import planner as planner_module
    from rm75_drake.coordinated.regions import MinimalRegionError

    def _broken(*args: Any, **kwargs: Any):
        raise MinimalRegionError("[test] 人为破坏区域生成", reason="seed_infeasible")

    monkeypatch.setattr(planner_module, "generate_minimal_region", _broken)

    result = planner_module.plan_coordinated(
        dual_scene,
        x_lr=X_LR,
        start_leader_q=LEADER_START,
        goal_leader_q=LEADER_GOAL,
        options=_integration_options(allow_degradation=True),
    )
    assert result.success is True, result.message
    assert result.left_trajectory is not None and result.right_trajectory is not None

    diagnostics = dict(result.diagnostics)
    assert diagnostics["degraded"] is True
    assert diagnostics["degradation_reason"] == "empty_region"
    assert diagnostics["degradation_message"]
    # 采样基线在最小坐标空间被**稠密化**（PRM 只有两个端点 ⇒ 残差过大）。
    assert int(diagnostics["degradation_path_points"]) > 2
    assert int(diagnostics["num_regions"]) == 0
    # 降级路径同样满足硬要求。
    assert diagnostics["num_escapes"] == 0
    assert diagnostics["relative_pose_residual_max_position_m"] <= POSITION_TOL
    assert diagnostics["relative_pose_residual_max_orientation_rad"] <= ORIENTATION_TOL
