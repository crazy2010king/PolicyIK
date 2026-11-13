"""共同点一致性判定器测试（S04-T06）。

覆盖三条路径
------------
* **PASS**：两臂 TCP 被解到共同工作区的同一点 → ``verdict == "pass"``；
* **FAIL**：右臂 TCP 目标人为偏移 2 mm → ``verdict == "fail"``，
  ``exceed_mm`` 报告超出量；
* **未验证**：``measured=False`` → ``verdict == "unverified"``，
  ``unverified_metrics`` 非空，**绝不返回 pass**（MEM083 纪律：未实测不声称达标）。

负例覆盖：frame 名不存在（必须点名该 frame）、plant 未 finalize、阈值非法、
姿态阈值非法。

构型怎么来的
------------
两臂 + 双夹爪装进**同一个** plant（:func:`rm75_drake.bimanual.dual_scene.build_dual_scene`），
然后用**纯 numpy 的位置 IK**（有限差分雅可比 + 软件限位钳位）把两臂 TCP
分别解到同一目标点——这不是"编一个位姿"，而是**真实运动学解**：
残差 < 1e-12 m（PASS 构型）与 < 1e-12 m（FAIL 构型）。
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from rm75_drake import limits as _limits
from rm75_drake.bimanual.consistency import (
    DEFAULT_THRESHOLD_MM,
    REASON_CALIBRATION_NOT_MEASURED,
    REASON_EXCEEDS_THRESHOLD,
    REASON_ORIENTATION_EXCEEDS_THRESHOLD,
    REASON_WITHIN_THRESHOLD,
    VERDICT_FAIL,
    VERDICT_PASS,
    VERDICT_UNVERIFIED,
    ConsistencyError,
    ConsistencyResult,
    evaluate_common_point,
)
from rm75_drake.bimanual.dual_scene import build_dual_scene, default_gripper_urdfs
from rm75_drake.scenario import load_scenario

REPO_ROOT = Path(__file__).resolve().parents[1]
SCENARIO_PATH = REPO_ROOT / "scenarios" / "dual_arm.yaml"

#: 共同工作区的目标点（world 系，m）：两臂基座连线中点正上方。
COMMON_POINT_M = np.array([0.30, 0.0, 0.45])

#: FAIL 构型：右臂 TCP 目标沿 +x 偏移 2 mm。
FAIL_OFFSET_M = np.array([0.002, 0.0, 0.0])


# ── fixture ────────────────────────────────────────────────────────────
@pytest.fixture(scope="module")
def dual_bundle():
    """双臂 + 双夹爪的**同一个** plant（真实 TCP frame 由此而来）。"""
    scenario = load_scenario(SCENARIO_PATH)
    return build_dual_scene(
        scenario, gripper_urdfs=default_gripper_urdfs(["left", "right"])
    )


@pytest.fixture(scope="module")
def software_limits():
    """RM75 软件软限位（下限数组, 上限数组）——位置 IK 的钳位边界。"""
    data = _limits.load_joint_limits()
    lower = np.array(
        [j["software"]["position_lower_rad"] for j in data["joints"]], dtype=float
    )
    upper = np.array(
        [j["software"]["position_upper_rad"] for j in data["joints"]], dtype=float
    )
    return lower, upper


def _arm_position_indices(plant, side: str):
    """返回某侧 7 个机械臂关节对应的**位置变量索引**（按关节序）。"""
    names = plant.GetPositionNames()
    indices = []
    for i in range(1, 8):
        joint = f"{side}_joint_{i}"
        matches = [k for k, name in enumerate(names) if name.endswith(f"{joint}_q")]
        assert len(matches) == 1, f"关节 {joint} 的位置变量不唯一：{matches}"
        indices.append(matches[0])
    return indices


def _solve_tcp_position(
    plant,
    context,
    frame_name: str,
    target,
    indices,
    seed,
    lower,
    upper,
    *,
    max_iterations: int = 400,
    tolerance: float = 1e-12,
):
    """位置 IK（纯 numpy：有限差分雅可比 + 最小二乘 + 限位钳位）。

    返回 ``(q, 残差 m)``：``q`` 是**整条 plant** 的位置向量。
    """
    world = plant.world_frame()
    frame = plant.GetFrameByName(frame_name)
    target = np.asarray(target, dtype=float).reshape(3)
    q = plant.GetPositions(context).copy()
    q[indices] = np.asarray(seed, dtype=float)

    def position(current_q):
        plant.SetPositions(context, current_q)
        pose = plant.CalcRelativeTransform(context, world, frame)
        return np.asarray(pose.translation(), dtype=float)

    residual = float("nan")
    for _ in range(max_iterations):
        current = position(q)
        error = target - current
        residual = float(np.linalg.norm(error))
        if residual < tolerance:
            break
        jacobian = np.zeros((3, len(indices)))
        epsilon = 1e-6
        for column, index in enumerate(indices):
            perturbed = q.copy()
            perturbed[index] += epsilon
            jacobian[:, column] = (position(perturbed) - current) / epsilon
        step = np.linalg.lstsq(jacobian, error, rcond=None)[0]
        q[indices] = np.clip(q[indices] + step, lower, upper)

    final = position(q)
    residual = float(np.linalg.norm(target - final))
    return q, residual


def _build_configuration(bundle, limits, right_offset_m):
    """解出「左臂 TCP 在目标点、右臂 TCP 在目标点+偏移」的整条 q。

    返回 ``(q, left_residual, right_residual)``（残差单位 m）。
    """
    plant, context = bundle.plant, bundle.context
    lower, upper = limits
    left_frame = bundle.tcp_frames["left"]
    right_frame = bundle.tcp_frames["right"]

    left_q, left_residual = _solve_tcp_position(
        plant,
        context,
        left_frame,
        COMMON_POINT_M,
        _arm_position_indices(plant, "left"),
        np.array([0.0, -0.6, 0.0, 0.9, 0.0, 0.5, 0.0]),
        lower,
        upper,
    )
    right_q, right_residual = _solve_tcp_position(
        plant,
        context,
        right_frame,
        COMMON_POINT_M + np.asarray(right_offset_m, dtype=float),
        _arm_position_indices(plant, "right"),
        np.array([3.0, -0.6, 0.0, 0.9, 0.0, 0.5, 0.0]),
        lower,
        upper,
    )
    q = plant.GetPositions(context).copy()
    q[_arm_position_indices(plant, "left")] = left_q[
        _arm_position_indices(plant, "left")
    ]
    q[_arm_position_indices(plant, "right")] = right_q[
        _arm_position_indices(plant, "right")
    ]
    plant.SetPositions(context, q)
    return q, left_residual, right_residual


@pytest.fixture(scope="module")
def pass_configuration(dual_bundle, software_limits):
    """PASS 构型：两臂 TCP 落在同一点（真实运动学解）。"""
    q, left_residual, right_residual = _build_configuration(
        dual_bundle, software_limits, np.zeros(3)
    )
    assert left_residual < 1e-9, f"左臂 IK 残差过大：{left_residual}"
    assert right_residual < 1e-9, f"右臂 IK 残差过大：{right_residual}"
    return q


@pytest.fixture(scope="module")
def fail_configuration(dual_bundle, software_limits):
    """FAIL 构型：右臂 TCP 目标偏移 2 mm。"""
    q, left_residual, right_residual = _build_configuration(
        dual_bundle, software_limits, FAIL_OFFSET_M
    )
    assert left_residual < 1e-9, f"左臂 IK 残差过大：{left_residual}"
    assert right_residual < 1e-9, f"右臂 IK 残差过大：{right_residual}"
    return q


def _evaluate(bundle, q, **kwargs):
    """把指定构型写进 plant 上下文后做判定（保证每次判定用的是该构型）。"""
    bundle.plant.SetPositions(bundle.context, q)
    return evaluate_common_point(
        bundle.plant,
        bundle.context,
        left_tcp_frame=bundle.tcp_frames["left"],
        right_tcp_frame=bundle.tcp_frames["right"],
        **kwargs,
    )


# ── PASS ───────────────────────────────────────────────────────────────
def test_pass_when_both_tcps_coincide(dual_bundle, pass_configuration):
    """两臂 TCP 同点 → pass，且实际位置偏差远小于 0.5 mm。"""
    result = _evaluate(dual_bundle, pass_configuration, measured=True)
    assert isinstance(result, ConsistencyResult)
    assert result.verdict == VERDICT_PASS
    assert result.reason == REASON_WITHIN_THRESHOLD
    assert result.position_error_mm < DEFAULT_THRESHOLD_MM
    assert result.position_error_mm < 1e-3  # 真实 IK 解：偏差在微米级
    assert result.exceed_mm == 0.0
    assert result.unverified_metrics == ()
    assert result.measured is True
    assert result.is_pass and not result.is_fail and not result.is_unverified


# ── FAIL ───────────────────────────────────────────────────────────────
def test_fail_when_right_tcp_offset_by_2mm(dual_bundle, fail_configuration):
    """右臂 TCP 偏移 2 mm → fail，且报告超出量。"""
    result = _evaluate(dual_bundle, fail_configuration, measured=True)
    assert result.verdict == VERDICT_FAIL
    assert result.reason == REASON_EXCEEDS_THRESHOLD
    assert result.position_error_mm == pytest.approx(2.0, abs=1e-3)
    assert result.exceed_mm == pytest.approx(2.0 - DEFAULT_THRESHOLD_MM, abs=1e-3)
    assert result.exceed_mm == pytest.approx(
        result.position_error_mm - result.threshold_mm, abs=1e-9
    )
    assert result.unverified_metrics == ()


# ── 未验证 ─────────────────────────────────────────────────────────────
def test_unverified_when_calibration_not_measured(dual_bundle, pass_configuration):
    """标定未实测 → 只能报「未验证」，**不得**报 pass。"""
    result = _evaluate(dual_bundle, pass_configuration, measured=False)
    assert result.verdict == VERDICT_UNVERIFIED
    assert result.verdict != VERDICT_PASS
    assert result.reason == REASON_CALIBRATION_NOT_MEASURED
    assert result.unverified_metrics  # 非空
    assert "position_error_mm" in result.unverified_metrics
    assert result.is_unverified and not result.is_pass and not result.is_fail
    assert result.measured is False


def test_unverified_takes_precedence_over_fail(dual_bundle, fail_configuration):
    """未实测时即便偏差超阈也只报「未验证」（不伪装达标，也不伪装失败结论）。"""
    result = _evaluate(dual_bundle, fail_configuration, measured=False)
    assert result.verdict == VERDICT_UNVERIFIED
    assert result.reason == REASON_CALIBRATION_NOT_MEASURED
    assert result.unverified_metrics


# ── 阈值语义 ───────────────────────────────────────────────────────────
def test_threshold_default_is_half_millimeter(dual_bundle, pass_configuration):
    """默认阈值 0.5 mm（里程碑 ROADMAP 口径），且结果回报实际使用的阈值。"""
    result = _evaluate(dual_bundle, pass_configuration, measured=True)
    assert DEFAULT_THRESHOLD_MM == 0.5
    assert result.threshold_mm == 0.5


def test_threshold_override_is_reported_and_changes_verdict(
    dual_bundle, fail_configuration
):
    """显式给阈值 → 结果回报该阈值；阈值放宽到 5 mm 后同一构型判 pass。"""
    result = _evaluate(dual_bundle, fail_configuration, measured=True, threshold_mm=5.0)
    assert result.threshold_mm == 5.0
    assert result.verdict == VERDICT_PASS
    assert result.position_error_mm == pytest.approx(2.0, abs=1e-3)
    assert result.exceed_mm == 0.0


# ── 姿态阈值（可选判据）────────────────────────────────────────────────
def test_orientation_threshold_violation_is_fail(dual_bundle, pass_configuration):
    """显式给姿态阈值且超阈 → fail（标签不同于位置超阈）。"""
    result = _evaluate(
        dual_bundle, pass_configuration, measured=True, max_orientation_error_deg=0.0
    )
    assert result.verdict == VERDICT_FAIL
    assert result.reason == REASON_ORIENTATION_EXCEEDS_THRESHOLD
    assert result.orientation_error_deg > 0.0
    assert result.exceed_deg == pytest.approx(result.orientation_error_deg, abs=1e-9)
    assert result.position_error_mm < DEFAULT_THRESHOLD_MM


def test_orientation_threshold_satisfied_is_pass(dual_bundle, pass_configuration):
    """姿态阈值足够宽松 → 判 pass（位置判据主导）。"""
    result = _evaluate(
        dual_bundle, pass_configuration, measured=True, max_orientation_error_deg=180.0
    )
    assert result.verdict == VERDICT_PASS
    assert result.exceed_deg == 0.0


# ── 负例 ───────────────────────────────────────────────────────────────
@pytest.mark.parametrize("bad_frame", ["no_such_tcp_frame", "left_tcp_typo"])
def test_unknown_frame_raises_and_names_it(dual_bundle, pass_configuration, bad_frame):
    """frame 不存在 → 抛错并**点名该 frame**（两侧都覆盖）。"""
    with pytest.raises(ConsistencyError) as excinfo:
        evaluate_common_point(
            dual_bundle.plant,
            dual_bundle.context,
            left_tcp_frame=bad_frame,
            right_tcp_frame=dual_bundle.tcp_frames["right"],
            measured=True,
        )
    assert bad_frame in str(excinfo.value)
    assert excinfo.value.reason == "unknown_frame"

    with pytest.raises(ConsistencyError) as excinfo:
        evaluate_common_point(
            dual_bundle.plant,
            dual_bundle.context,
            left_tcp_frame=dual_bundle.tcp_frames["left"],
            right_tcp_frame=bad_frame,
            measured=True,
        )
    assert bad_frame in str(excinfo.value)
    assert excinfo.value.reason == "unknown_frame"


def test_plant_not_finalized_raises_and_names_frames():
    """plant 未 Finalize → 显式错误（消息点名两个 frame）。"""
    from pydrake.multibody.plant import MultibodyPlant

    plant = MultibodyPlant(0.0)
    assert plant.is_finalized() is False
    with pytest.raises(ConsistencyError) as excinfo:
        evaluate_common_point(
            plant,
            None,
            left_tcp_frame="left_tcp",
            right_tcp_frame="right_tcp",
            measured=True,
        )
    assert excinfo.value.reason == "plant_not_finalized"
    assert "left_tcp" in str(excinfo.value)
    assert "right_tcp" in str(excinfo.value)


def test_foreign_context_is_rejected(dual_bundle, pass_configuration):
    """context 与 plant 不匹配 → 显式错误（不是裸 RuntimeError 泄漏）。"""
    from pydrake.multibody.plant import MultibodyPlant

    other = MultibodyPlant(0.0)
    other.Finalize()
    with pytest.raises(ConsistencyError) as excinfo:
        evaluate_common_point(
            dual_bundle.plant,
            other.CreateDefaultContext(),
            left_tcp_frame=dual_bundle.tcp_frames["left"],
            right_tcp_frame=dual_bundle.tcp_frames["right"],
            measured=True,
        )
    assert excinfo.value.reason == "invalid_context"


@pytest.mark.parametrize("bad_threshold", [0.0, -1.0, float("nan"), float("inf"), "0.5"])
def test_invalid_threshold_rejected(dual_bundle, pass_configuration, bad_threshold):
    """阈值非法 → 显式错误（不静默改默认值）。"""
    with pytest.raises(ConsistencyError) as excinfo:
        evaluate_common_point(
            dual_bundle.plant,
            dual_bundle.context,
            left_tcp_frame=dual_bundle.tcp_frames["left"],
            right_tcp_frame=dual_bundle.tcp_frames["right"],
            measured=True,
            threshold_mm=bad_threshold,
        )
    assert excinfo.value.reason == "invalid_threshold"


@pytest.mark.parametrize("bad_orientation", [-1.0, float("nan"), "5"])
def test_invalid_orientation_threshold_rejected(
    dual_bundle, pass_configuration, bad_orientation
):
    """姿态阈值非法 → 显式错误。"""
    with pytest.raises(ConsistencyError) as excinfo:
        evaluate_common_point(
            dual_bundle.plant,
            dual_bundle.context,
            left_tcp_frame=dual_bundle.tcp_frames["left"],
            right_tcp_frame=dual_bundle.tcp_frames["right"],
            measured=True,
            max_orientation_error_deg=bad_orientation,
        )
    assert excinfo.value.reason == "invalid_threshold"


# ── 报告面 ─────────────────────────────────────────────────────────────
def test_result_reports_metrics_and_summary(dual_bundle, pass_configuration):
    """结果自带可观测面：可序列化字典 + 一行摘要。"""
    result = _evaluate(dual_bundle, pass_configuration, measured=True)
    payload = result.as_dict()
    for key in (
        "verdict",
        "position_error_mm",
        "orientation_error_deg",
        "threshold_mm",
        "exceed_mm",
        "reason",
        "unverified_metrics",
        "measured",
    ):
        assert key in payload
    assert payload["verdict"] == VERDICT_PASS
    summary = result.summary()
    assert "PASS" in summary
    assert "0.5" in summary


def test_consistency_module_has_no_pydrake_dependency():
    """判定器**零 pydrake 依赖**（只吃 plant/context 接口，便于离线复用）。"""
    source = (
        REPO_ROOT / "src" / "rm75_drake" / "bimanual" / "consistency.py"
    ).read_text(encoding="utf-8")
    assert "import pydrake" not in source
    assert "from pydrake" not in source
