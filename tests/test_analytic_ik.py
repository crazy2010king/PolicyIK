"""RM75 解析 IK 的验收测试（S05-T01）。

覆盖
----
* **零 Drake 依赖**：模块不得 import pydrake，也不得绕回 ``rm75_drake`` 的
  Drake 侧模块；``import rm75_drake.kinematics`` 不把 pydrake 拉进 sys.modules。
* **参数表引用而非复制**：d / alpha 表必须来自 ``rm75_drake.kinematics.mdh``。
* **往返自洽**：随机 q → FK → 解析 IK → FK 比对
  （精确 ±π/2 表 ≤1e-9 m / 1e-9 rad；官方舍入表 ≤5e-6 m / 1e-9 rad）。
* **与 Drake FK 一致性**（``models.py`` 的 link_7）：≤5e-6 m / 1e-9 rad。
* **失败路径反例**：不可达 / 限位外 / 腕奇异 / 非法目标，均返回明确 reason。
* **分支覆盖**：8 组镜像分支（q2 / q4 / q6 各 0/1）都被枚举到。
"""

from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

from rm75_drake.kinematics import analytic_ik
from rm75_drake.kinematics import mdh
from rm75_drake.models import eval_link_pose_in_world, load_rm75

REPO_ROOT = Path(__file__).resolve().parents[1]

#: 随机位形数量（计划要求 100 组量级；这里取 40 组以控制测试时长）。
N_RANDOM_CONFIGS = 40

#: 固定随机种子（回归可复现）。
RANDOM_SEED = 20260113

#: 精确表的往返容差（IK 自身的往返自洽门槛，严于 FK 对照门槛）。
EXACT_POS_TOL = 1e-9
EXACT_ROT_TOL = 1e-9

#: 官方舍入表（URDF rpy=1.5708）的往返容差 —— 阈值来源唯一真相源：
#: rm75_drake.kinematics.mdh（FK 门槛常量，见 docs/architecture/fk-tolerance.md）。
ROUNDED_POS_TOL = mdh.FK_POSITION_TOLERANCE_URDF_M
ROUNDED_ROT_TOL = mdh.FK_ROTATION_TOLERANCE_RAD

#: 与 Drake FK 的一致性容差（官方 URDF 舍入残差的同阶，来源同上）。
DRAKE_POS_TOL = mdh.FK_POSITION_TOLERANCE_URDF_M
DRAKE_ROT_TOL = mdh.FK_ROTATION_TOLERANCE_RAD


# ── fixtures ────────────────────────────────────────────────────────────
@pytest.fixture(scope="module")
def drake_plant():
    """装配好的改造版 RM75（weld 基座，DoF=7），返回 ``(plant, context)``。"""
    from pydrake.multibody.plant import MultibodyPlant

    plant = MultibodyPlant(0.0)
    plant, _index, _parser = load_rm75(plant, weld=True)
    plant.Finalize()
    context = plant.CreateDefaultContext()
    return plant, context


def _random_configs(count=N_RANDOM_CONFIGS, seed=RANDOM_SEED):
    """固定种子的随机位形（q_i ∈ [-1, 1] rad）。"""
    rng = np.random.default_rng(seed)
    return [rng.uniform(-1.0, 1.0, mdh.N_JOINTS) for _ in range(count)]


def _pose(position, rotation=None):
    """构造 4×4 位姿。"""
    pose = np.eye(4)
    pose[:3, 3] = np.asarray(position, dtype=float)
    if rotation is not None:
        pose[:3, :3] = np.asarray(rotation, dtype=float)
    return pose


def _rotation_error(r1, r2):
    """两个旋转矩阵的夹角（rad）——用 atan2 形式，避免 acos 的抵消误差。"""
    rel = r1.T @ r2
    skew = 0.5 * (rel - rel.T)
    sin_theta = float(np.linalg.norm(skew))
    cos_theta = (float(np.trace(rel)) - 1.0) * 0.5
    return float(np.arctan2(sin_theta, max(-1.0, min(1.0, cos_theta))))


# ── 零 Drake 依赖 ───────────────────────────────────────────────────────
def test_analytic_ik_module_has_no_drake_dependency():
    """解析 IK 模块不得 import pydrake，也不得绕回 Drake 侧模块。"""
    source = Path(analytic_ik.__file__).read_text(encoding="utf-8")
    offenders = [
        line.strip()
        for line in source.splitlines()
        if re.search(r"^\s*(import|from)\s+pydrake", line)
    ]
    assert offenders == [], f"analytic_ik.py 出现了 Drake 导入：{offenders}"
    # 也不能绕回会拉起 pydrake 的 rm75_drake 顶层模块（models / drake_compat / viz）。
    offenders = [
        line.strip()
        for line in source.splitlines()
        if re.search(r"^\s*(from|import)\s+rm75_drake", line)
    ]
    assert offenders == [], f"analytic_ik.py 绕回了 rm75_drake 顶层：{offenders}"
    offenders = [
        line.strip()
        for line in source.splitlines()
        if re.search(r"^\s*(from|import)\s+\.\.(models|drake_compat|viz|planning)", line)
    ]
    assert offenders == [], f"analytic_ik.py 引入了 Drake 侧子模块：{offenders}"


def test_kinematics_package_import_does_not_load_pydrake():
    """import rm75_drake.kinematics 不应把 pydrake 拉进 sys.modules。"""
    code = (
        "import sys; sys.path.insert(0, %r);"
        "import rm75_drake.kinematics;"
        "print('pydrake' in sys.modules)" % str(REPO_ROOT / "src")
    )
    out = subprocess.run(
        [sys.executable, "-c", code], capture_output=True, text=True, check=True
    )
    assert out.stdout.strip().endswith("False"), out.stdout


def test_parameter_tables_are_referenced_not_copied():
    """d / alpha 表必须**引用** mdh 的表，模块里不得复制数值。"""
    assert analytic_ik.mdh.MDH_D is mdh.MDH_D
    assert analytic_ik.mdh.MDH_ALPHA is mdh.MDH_ALPHA
    assert analytic_ik.mdh.MDH_ALPHA_EXACT_PI_2 is mdh.MDH_ALPHA_EXACT_PI_2
    source = Path(analytic_ik.__file__).read_text(encoding="utf-8")
    for literal in ("0.2405", "0.256", "0.210", "1.5708"):
        assert literal not in source, f"analytic_ik.py 复制了参数表数值：{literal}"


def test_tool_offset_default_is_bare_arm_d7():
    """工具偏置默认取裸臂 d7（0.144 m），装夹爪场景由调用方显式传 0.204。"""
    assert mdh.MDH_D[6] == pytest.approx(0.144, abs=1e-12)
    pose = _pose([0.3, 0.0, 0.4])
    solutions = analytic_ik.analytic_solutions(pose, tool_offset=mdh.MDH_D[6])
    assert solutions, "默认工具偏置下应有解"
    # 传 0.204（装夹爪）时目标位姿随之改变 ⇒ 解集必须不同。
    other = analytic_ik.analytic_solutions(pose, tool_offset=0.204)
    assert [s.q_tuple for s in other] != [s.q_tuple for s in solutions]


# ── 往返自洽 ────────────────────────────────────────────────────────────
@pytest.mark.parametrize(
    "alpha, pos_tol, rot_tol, label",
    [
        (mdh.MDH_ALPHA_EXACT_PI_2, EXACT_POS_TOL, EXACT_ROT_TOL, "精确 ±π/2 表"),
        (mdh.MDH_ALPHA, ROUNDED_POS_TOL, ROUNDED_ROT_TOL, "官方舍入表"),
    ],
)
def test_roundtrip_self_consistency(alpha, pos_tol, rot_tol, label):
    """随机 q → FK → 解析 IK → FK 比对（逐条解检查）。"""
    worst_pos = 0.0
    worst_rot = 0.0
    for q in _random_configs():
        target = mdh.fk(q, alpha=alpha)
        solutions = analytic_ik.analytic_solutions(target, alpha=alpha, psi_count=16)
        assert solutions, f"{label}：目标 {q} 没有解析解"
        for solution in solutions:
            frame = mdh.fk(solution.q, alpha=alpha)
            worst_pos = max(
                worst_pos, float(np.linalg.norm(frame[:3, 3] - target[:3, 3]))
            )
            worst_rot = max(worst_rot, _rotation_error(frame[:3, :3], target[:3, :3]))
    assert worst_pos <= pos_tol, f"{label}：位置往返误差 {worst_pos} > {pos_tol}"
    assert worst_rot <= rot_tol, f"{label}：姿态往返误差 {worst_rot} > {rot_tol}"


def test_roundtrip_reports_per_solution_errors():
    """每条解都自带 FK 复算误差（供 diagnostics 使用）。"""
    q = np.array([0.1, -0.2, 0.3, 0.4, -0.5, 0.6, -0.7])
    target = mdh.fk(q)
    solutions = analytic_ik.analytic_solutions(target)
    assert solutions
    for solution in solutions:
        assert solution.position_error <= 1e-9
        assert solution.orientation_error <= 1e-9


def test_psi_parameter_controls_the_redundancy():
    """ψ 是连续自由度：不同 ψ 给出不同 q，但都精确复现同一位姿。"""
    q = np.array([0.2, 0.4, -0.3, 0.5, 0.6, -0.7, 0.8])
    target = mdh.fk(q)
    first = analytic_ik.analytic_solutions(target, psi=0.0)
    second = analytic_ik.analytic_solutions(target, psi=np.pi / 3)
    assert first and second
    assert first[0].psi == pytest.approx(0.0)
    assert second[0].psi == pytest.approx(np.pi / 3)
    assert not np.allclose(first[0].q, second[0].q)
    for solution in first + second:
        frame = mdh.fk(solution.q)
        assert np.linalg.norm(frame[:3, 3] - target[:3, 3]) <= 1e-9


def test_branch_enumeration_covers_eight_mirror_combinations():
    """8 组镜像分支（q2 / q4 / q6 各 0/1）都被枚举到。"""
    q = np.array([0.15, -0.35, 0.45, -0.55, 0.65, -0.75, 0.85])
    target = mdh.fk(q)
    solutions = analytic_ik.analytic_solutions(target, psi_count=8)
    branches = {solution.branch for solution in solutions}
    assert len(branches) == 8, f"镜像分支覆盖不足：{sorted(branches)}"
    for solution in solutions:
        assert len(solution.branch) == 3
        assert all(flag in (0, 1) for flag in solution.branch)


def test_branch_restriction_keeps_only_requested_branch():
    """指定镜像分支时只保留该分支（热循环用：候选从 ~130 条降到 1 条）。"""
    q = np.array([0.15, -0.35, 0.45, -0.55, 0.65, -0.75, 0.85])
    target = mdh.fk(q)
    full = analytic_ik.analytic_solutions(target, psi_count=8)
    assert full
    for branch in sorted({s.branch for s in full}):
        subset = analytic_ik.analytic_solutions(target, psi_count=8, branch=branch)
        assert subset, f"分支 {branch} 过滤后不应为空"
        assert {s.branch for s in subset} == {branch}
        for solution in subset:
            assert solution.q_tuple in [s.q_tuple for s in full]


def test_solution_is_reproducible():
    """同目标 + 同参数两次求解结果一致（可复现）。"""
    q = np.array([0.11, 0.22, -0.33, 0.44, -0.55, 0.66, -0.77])
    target = mdh.fk(q)
    first = analytic_ik.analytic_solutions(target)
    second = analytic_ik.analytic_solutions(target)
    assert [s.q_tuple for s in first] == [s.q_tuple for s in second]
    assert [s.branch for s in first] == [s.branch for s in second]


# ── 与 Drake FK 的一致性 ────────────────────────────────────────────────
def test_consistency_with_drake_forward_kinematics(drake_plant):
    """与 Drake FK（models.py 的 link_7）一致性 ≤5e-6 m / 1e-9 rad。"""
    plant, context = drake_plant
    worst_pos = 0.0
    worst_rot = 0.0
    for q in _random_configs(10):
        plant.SetPositions(context, np.asarray(q, dtype=float))
        frame = eval_link_pose_in_world(plant, context, "link_7")
        target = np.eye(4)
        target[:3, :3] = frame.rotation().matrix()
        target[:3, 3] = frame.translation()
        result = analytic_ik.solve_analytic_ik(target, seed=q)
        assert result.success, result.message
        # 解析解用同一张 d/alpha 表复算，必须复现 Drake 的位姿。
        replay = mdh.fk(result.q)
        worst_pos = max(
            worst_pos, float(np.linalg.norm(replay[:3, 3] - target[:3, 3]))
        )
        worst_rot = max(worst_rot, _rotation_error(replay[:3, :3], target[:3, :3]))
    assert worst_pos <= DRAKE_POS_TOL, f"与 Drake 的位置差 {worst_pos} > {DRAKE_POS_TOL}"
    assert worst_rot <= DRAKE_ROT_TOL, f"与 Drake 的姿态差 {worst_rot} > {DRAKE_ROT_TOL}"


def test_mdh_chain_and_drake_agree_within_official_rounding_residual(drake_plant):
    """鉴别力：MDH 链与 Drake 的残差是官方 rpy 舍入量级（~2e-6 m）。"""
    plant, context = drake_plant
    q = np.array([0.3, -0.4, 0.5, 0.6, -0.7, 0.8, -0.9])
    plant.SetPositions(context, q)
    frame = eval_link_pose_in_world(plant, context, "link_7")
    replay = mdh.fk(q)
    drake_pos = np.asarray(frame.translation())
    drake_rot = np.asarray(frame.rotation().matrix())
    residual = float(np.linalg.norm(replay[:3, 3] - drake_pos))
    assert residual <= DRAKE_POS_TOL
    assert residual > 1e-9, "残差不可能为 0（官方 rpy 舍入真实存在）"
    assert _rotation_error(replay[:3, :3], drake_rot) <= DRAKE_ROT_TOL


# ── 失败路径（反例断言） ────────────────────────────────────────────────
def test_unreachable_target_names_reason():
    """不可达目标 ⇒ reason = unreachable（不是空泛失败）。"""
    far = _pose([2.0, 0.0, 1.0])
    result = analytic_ik.solve_analytic_ik(far)
    assert not result.success
    assert result.reason == analytic_ik.REASON_UNREACHABLE
    assert "可达" in result.message
    assert analytic_ik.analytic_solutions(far) == []


def test_limit_violation_names_reason():
    """限位外目标 ⇒ reason = limit_violation。"""
    # 腕心在基座正下方 ⇒ 肩角 q2 必然 ≈ ±2.84 rad > 软件限位 2.199。
    below = _pose([0.0, 0.0, -0.056])
    result = analytic_ik.solve_analytic_ik(below)
    assert not result.success
    assert result.reason == analytic_ik.REASON_LIMIT_VIOLATION
    assert result.solutions, "限位外也应报告几何解，便于诊断"
    for solution in result.solutions:
        assert not analytic_ik.within_limits(solution.q)


def test_wrist_singular_names_reason():
    """腕奇异位形 ⇒ reason = wrist_singular（显式处理，不静默通过）。"""
    # 完全伸展（肘圆退化为点）+ 工具轴与 J5 轴平行 ⇒ sin q6 = 0。
    rot = np.array([[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]])
    target = _pose([0.610, 0.0, 0.2405], rot)
    result = analytic_ik.solve_analytic_ik(target)
    assert not result.success
    assert result.reason == analytic_ik.REASON_WRIST_SINGULAR
    assert result.wrist_singular
    for solution in result.solutions:
        assert solution.wrist_singular
        assert abs(np.sin(solution.q[5])) < 1e-6


def test_invalid_target_names_reason():
    """非法目标 ⇒ reason = invalid_target（形状错 / NaN / 非正交）。"""
    nan_pose = _pose([0.3, 0.0, 0.4])
    nan_pose[0, 3] = np.nan
    result = analytic_ik.solve_analytic_ik(nan_pose)
    assert not result.success
    assert result.reason == analytic_ik.REASON_INVALID_TARGET
    assert result.q is None

    result = analytic_ik.solve_analytic_ik(np.eye(3))
    assert not result.success
    assert result.reason == analytic_ik.REASON_INVALID_TARGET

    mirror = np.eye(4)
    mirror[0, 0] = -1.0
    result = analytic_ik.solve_analytic_ik(mirror)
    assert not result.success
    assert result.reason == analytic_ik.REASON_INVALID_TARGET


def test_analytic_solutions_raises_on_invalid_target():
    """枚举 API 对非法输入直接抛 ValueError（不静默返回空列表）。"""
    with pytest.raises(ValueError):
        analytic_ik.analytic_solutions(np.eye(3))
    with pytest.raises(ValueError):
        analytic_ik.analytic_solutions(_pose([0.3, 0.0, 0.4]), psi_count=0)
    with pytest.raises(ValueError):
        analytic_ik.analytic_solutions(_pose([0.3, 0.0, 0.4]), d=(1.0, 2.0))


# ── 选支策略 ────────────────────────────────────────────────────────────
def test_nearest_policy_picks_closest_solution_to_seed():
    """branch_policy='nearest' 选离 seed 最近的解（S05-T03 连续性管理的基础）。"""
    q = np.array([0.12, -0.23, 0.34, -0.45, 0.56, -0.67, 0.78])
    target = mdh.fk(q)
    result = analytic_ik.solve_analytic_ik(target, seed=q, branch_policy="nearest")
    assert result.success
    candidates = result.solutions
    best = min(candidates, key=lambda s: float(np.linalg.norm(s.q - q)))
    assert np.allclose(result.q, best.q)


def test_max_margin_policy_picks_largest_margin():
    """branch_policy='max_margin' 选限位裕度最大的解。"""
    q = np.array([0.12, -0.23, 0.34, -0.45, 0.56, -0.67, 0.78])
    target = mdh.fk(q)
    result = analytic_ik.solve_analytic_ik(target, branch_policy="max_margin")
    assert result.success
    margins = [analytic_ik.limit_margin(s.q) for s in result.solutions]
    assert analytic_ik.limit_margin(result.q) == pytest.approx(max(margins))


def test_unknown_branch_policy_is_rejected():
    """未知选支策略 ⇒ 显式报错。"""
    target = mdh.fk(np.zeros(7))
    with pytest.raises(ValueError):
        analytic_ik.solve_analytic_ik(target, branch_policy="nope")


# ── 软件限位来源 ────────────────────────────────────────────────────────
def test_software_limits_come_from_s01_truth_file():
    """软件限位取自 S01 真值 YAML（7 项，逐项对应 joint_1..joint_7）。"""
    from rm75_drake.limits import load_joint_limits

    data = load_joint_limits()
    expected = [
        (
            float(joint["software"]["position_lower_rad"]),
            float(joint["software"]["position_upper_rad"]),
        )
        for joint in data["joints"]
    ]
    assert analytic_ik.software_limits() == tuple(expected)
    assert len(expected) == 7


def test_within_limits_rejects_out_of_range():
    """限位判定：越界即拒。"""
    limits = analytic_ik.software_limits()
    inside = np.zeros(7)
    assert analytic_ik.within_limits(inside, limits)
    outside = inside.copy()
    outside[0] = limits[0][1] + 0.1
    assert not analytic_ik.within_limits(outside, limits)
    assert analytic_ik.limit_margin(outside, limits) < 0.0
