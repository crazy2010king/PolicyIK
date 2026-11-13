"""安全七闸的逐闸单测（S03-T02）。

设计要点
--------
* **每闸至少一条正例 + 一条反例**（越限 → 违规非空；合规 → 无违规）。
* 纯函数性质：同样的输入必须给出同样的输出，且**不修改**入参。
* 围栏闸用 **MDH FK 的已知构型**验证（与
  :mod:`rm75_drake.kinematics.mdh` 的正解一致），而不是拍脑袋写期望值。
* 负例覆盖：维度错、NaN/Inf、非法 dt、非法阈值配置、限位交接件结构非法。
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = REPO_ROOT / "src"

if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from rm75_drake.device.config import load_device_config  # noqa: E402
from rm75_drake.device.safety import (  # noqa: E402
    ACTION_CLAMP,
    ACTION_PROCEED,
    ACTION_SLOW_STOP,
    ACTION_STOP,
    ALL_REASONS,
    REASON_ACCEL_CLAMP,
    REASON_CURRENT,
    REASON_DELTA_CLAMP,
    REASON_FENCE,
    REASON_SOFT_LIMIT,
    REASON_STATE_AGE,
    REASON_WATCHDOG,
    SafetyLimits,
    SafetyResult,
    SafetyState,
    SafetyViolation,
    apply_safety,
    gate_accel_clamp,
    gate_current,
    gate_delta_clamp,
    gate_fence,
    gate_soft_limit,
    gate_state_age,
    gate_watchdog,
    load_safety_limits,
    safety_limits_from_sources,
)
from rm75_drake.kinematics.mdh import MDH_ZERO_HEIGHT_M, link7_position  # noqa: E402
from rm75_drake.limits import load_joint_limits  # noqa: E402

#: 5 ms 执行周期（s）。
DT = 0.005

#: 零位构型（rad）。
ZERO = (0.0,) * 7

#: 一个**既不撞桌面也不进奇异圆柱**的已知构型（由 MDH FK 验证，见下方断言）。
SAFE_POSE = (1.0, -0.5, 0.0, 0.0, 0.0, 0.0, 0.0)


# ── 夹具 ────────────────────────────────────────────────────────────────
@pytest.fixture(scope="module")
def limits() -> SafetyLimits:
    """真实仓库阈值（限位交接件 + 设备配置）。"""
    return load_safety_limits()


@pytest.fixture(scope="module")
def device_config():
    return load_device_config()


@pytest.fixture(scope="module")
def joint_limits():
    return load_joint_limits()


# ── 阈值配置本身的正确性 ────────────────────────────────────────────────
def test_limits_from_repo_sources_match_truth(limits, device_config, joint_limits):
    """软限位 / ω_max 来自限位交接件的 software 段；a_max / 电流 / 围栏来自设备配置。"""
    software = [j["software"] for j in joint_limits["joints"]]
    assert list(limits.soft_lower_rad) == [
        float(s["position_lower_rad"]) for s in software
    ]
    assert list(limits.soft_upper_rad) == [
        float(s["position_upper_rad"]) for s in software
    ]
    assert list(limits.max_velocity_rad_s) == [
        float(s["velocity_rad_s"]) for s in software
    ]
    # 官方参数表：J1/J2 = 3.1416 rad/s，J3–J7 = 3.927 rad/s
    assert limits.max_velocity_rad_s[0] == pytest.approx(3.1416)
    assert limits.max_velocity_rad_s[1] == pytest.approx(3.1416)
    assert limits.max_velocity_rad_s[6] == pytest.approx(3.927)
    assert list(limits.max_accel_rad_s2) == list(device_config.accel_limits_rad_s2)
    assert list(limits.current_threshold_ma) == list(device_config.current_thresholds_ma)
    assert limits.table_height_m == pytest.approx(device_config.fence_table_height_m)
    assert limits.singularity_radius_m == pytest.approx(
        device_config.fence_singularity_radius_m
    )


def test_limits_rejects_bad_shapes(limits):
    """负例：维度不对 / 非正阈值 / 非法看门狗次序 → ValueError（点名字段）。"""
    kwargs = dict(
        soft_lower_rad=limits.soft_lower_rad,
        soft_upper_rad=limits.soft_upper_rad,
        max_velocity_rad_s=limits.max_velocity_rad_s,
        max_accel_rad_s2=limits.max_accel_rad_s2,
        current_threshold_ma=limits.current_threshold_ma,
        table_height_m=limits.table_height_m,
        singularity_radius_m=limits.singularity_radius_m,
    )
    with pytest.raises(ValueError, match="soft_lower_rad"):
        SafetyLimits(**{**kwargs, "soft_lower_rad": (0.0,) * 6})
    with pytest.raises(ValueError, match="max_velocity_rad_s"):
        SafetyLimits(
            **{**kwargs, "max_velocity_rad_s": (0.0,) + limits.max_velocity_rad_s[1:]}
        )
    with pytest.raises(ValueError, match="singularity_radius_m"):
        SafetyLimits(**{**kwargs, "singularity_radius_m": 0.0})
    with pytest.raises(ValueError, match="watchdog_slow_stop_cycles"):
        SafetyLimits(**kwargs, watchdog_warn_cycles=10, watchdog_slow_stop_cycles=10)


def test_safety_limits_from_sources_rejects_broken_joint_limits(device_config):
    """负例：限位交接件结构非法 → ValueError（点名缺哪一段）。"""
    with pytest.raises(ValueError, match="joints"):
        safety_limits_from_sources({"joints": []}, device_config)
    with pytest.raises(ValueError, match="software"):
        safety_limits_from_sources(
            {"joints": [{"name": "joint_1"}] + [{"name": f"joint_{i}"} for i in range(2, 8)]},
            device_config,
        )
    with pytest.raises(ValueError, match="velocity_rad_s"):
        broken = {
            "joints": [
                {
                    "name": f"joint_{i}",
                    "software": {
                        "position_lower_rad": -1.0,
                        "position_upper_rad": 1.0,
                        **({} if i == 1 else {"velocity_rad_s": 1.0}),
                    },
                }
                for i in range(1, 8)
            ]
        }
        safety_limits_from_sources(broken, device_config)


# ── G1：软件软限位 ──────────────────────────────────────────────────────
def test_g1_soft_limit_clamps_and_reports(limits):
    """反例：越限 → 钳到限位值 + 违规非空（reason=soft_limit）。"""
    upper = limits.soft_upper_rad[0]
    command = [upper + 0.5] + [0.0] * 6
    clamped, violations = gate_soft_limit(command, limits)
    assert clamped[0] == pytest.approx(upper)
    assert violations, "越限必须有违规记录"
    assert violations[0].reason == REASON_SOFT_LIMIT
    assert violations[0].gate == "G1"
    assert violations[0].joints == (0,)
    assert "joint_1" in violations[0].detail


def test_g1_soft_limit_accepts_in_range(limits):
    """正例：限位内 → 原样通过 + 无违规。"""
    command = list(limits.soft_upper_rad)
    clamped, violations = gate_soft_limit(command, limits)
    assert violations == []
    assert clamped == pytest.approx(tuple(command))


def test_g1_soft_limit_uses_repo_software_segment(limits, joint_limits):
    """G1 消费的是 ``load_joint_limits`` 的 software 段（比硬件每侧再收 4°）。"""
    hardware = joint_limits["joints"][0]["hardware"]
    software = joint_limits["joints"][0]["software"]
    assert limits.soft_upper_rad[0] == pytest.approx(float(software["position_upper_rad"]))
    assert limits.soft_upper_rad[0] == pytest.approx(
        float(hardware["position_upper_rad"]) - 0.0698132
    )


# ── G2：单步增量钳位 ────────────────────────────────────────────────────
def test_g2_delta_clamp_clamps_and_reports(limits):
    """反例：单步增量超 ω_max·dt → 钳位 + 违规（reason=delta_clamp）。"""
    prev = list(ZERO)
    max_step = limits.max_velocity_rad_s[0] * DT
    command = [0.5] + [0.0] * 6
    clamped, violations = gate_delta_clamp(command, prev, limits, DT)
    assert clamped[0] == pytest.approx(max_step)
    assert violations and violations[0].reason == REASON_DELTA_CLAMP
    assert violations[0].joints == (0,)


def test_g2_delta_clamp_accepts_small_step(limits):
    """正例：增量在阈值内 → 原样通过 + 无违规。"""
    prev = list(ZERO)
    command = [0.001] + [0.0] * 6
    clamped, violations = gate_delta_clamp(command, prev, limits, DT)
    assert violations == []
    assert clamped == pytest.approx(tuple(command))


def test_g2_delta_clamp_respects_per_axis_velocity(limits):
    """G2 是**逐轴**的：J1（3.1416 rad/s）与 J7（3.927 rad/s）阈值不同。"""
    prev = list(ZERO)
    command = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05]
    clamped, violations = gate_delta_clamp(command, prev, limits, DT)
    assert clamped[6] == pytest.approx(limits.max_velocity_rad_s[6] * DT)
    assert violations[0].joints == (6,)


# ── G3：加速度钳位 ──────────────────────────────────────────────────────
def test_g3_accel_clamp_clamps_and_reports(limits):
    """反例：加速度超 a_max → 钳位 + 违规（reason=accel_clamp）。"""
    prev = list(ZERO)
    prev_v = list(ZERO)
    max_dv = limits.max_accel_rad_s2[0] * DT
    command = [1.0] + [0.0] * 6
    clamped, violations = gate_accel_clamp(command, prev, prev_v, limits, DT)
    assert clamped[0] == pytest.approx(max_dv * DT)
    assert violations and violations[0].reason == REASON_ACCEL_CLAMP
    assert violations[0].joints == (0,)


def test_g3_accel_clamp_accepts_feasible_velocity(limits):
    """正例：速度变化在 a_max 内 → 原样通过 + 无违规。"""
    prev = list(ZERO)
    prev_v = list(ZERO)
    # v = 0.01 rad/s ⇒ Δv = 0.01 < a_max·dt = 0.03
    command = [0.01 * DT] + [0.0] * 6
    clamped, violations = gate_accel_clamp(command, prev, prev_v, limits, DT)
    assert violations == []
    assert clamped == pytest.approx(tuple(command))


def test_g3_accel_clamp_is_relative_to_previous_velocity(limits):
    """G3 以**上一拍速度**为基准（不是相对 0）。"""
    prev = list(ZERO)
    prev_v = [0.5] + [0.0] * 6
    command = [0.5 * DT] + [0.0] * 6
    _, violations = gate_accel_clamp(command, prev, prev_v, limits, DT)
    assert violations == [], "以 prev_v 为基准时不应报加速度违规"


# ── G4：心跳看门狗 ──────────────────────────────────────────────────────
def test_g4_watchdog_warns_after_threshold(limits):
    """反例：心跳缺失达到告警周期数 → 违规（reason=watchdog）。"""
    state = SafetyState(state_age_s=4 * DT)  # 4 个周期 ≥ 告警阈值 3
    violations = gate_watchdog(state, limits)
    assert violations and violations[0].reason == REASON_WATCHDOG
    assert violations[0].gate == "G4"
    assert violations[0].action == ACTION_CLAMP  # 仅告警，不要求停机


def test_g4_watchdog_slow_stop_beyond_threshold(limits):
    """反例：超过缓停阈值 → 要求缓停（action=slow_stop）。"""
    state = SafetyState(state_age_s=11 * DT)  # 11 > 10
    violations = gate_watchdog(state, limits)
    assert violations[0].action == ACTION_SLOW_STOP
    assert violations[0].severity == "critical"


def test_g4_watchdog_accepts_fresh_heartbeat(limits):
    """正例：心跳新鲜 → 无违规。"""
    assert gate_watchdog(SafetyState(state_age_s=0.0), limits) == []
    assert gate_watchdog(SafetyState(state_age_s=2 * DT), limits) == []


# ── G5：电流阈值 ────────────────────────────────────────────────────────
def test_g5_current_reports_over_threshold(limits):
    """反例：电流越限 → 违规（reason=current，action=stop）。"""
    currents = [0.0] * 7
    currents[2] = limits.current_threshold_ma[2] + 100.0
    violations = gate_current(SafetyState(joint_currents_ma=currents), limits)
    assert violations and violations[0].reason == REASON_CURRENT
    assert violations[0].joints == (2,)
    assert violations[0].action == ACTION_STOP


def test_g5_current_accepts_under_threshold(limits):
    """正例：电流在阈值内 → 无违规（阈值本身不算越限）。"""
    currents = list(limits.current_threshold_ma)
    assert gate_current(SafetyState(joint_currents_ma=currents), limits) == []


# ── G6：几何围栏 ────────────────────────────────────────────────────────
def test_g6_fence_known_configuration_matches_mdh_fk(limits):
    """围栏判定与 MDH FK 的正解一致（正例：安全构型无违规）。"""
    position = link7_position(SAFE_POSE)
    assert position[2] > limits.table_height_m
    assert math.hypot(position[0], position[1]) > limits.singularity_radius_m
    assert gate_fence(SAFE_POSE, limits) == []


def test_g6_fence_flags_zero_posture_singularity_cylinder(limits):
    """反例：零位构型（TCP 在基座轴线上方）落入奇异圆柱 → 违规。"""
    position = link7_position(ZERO)
    assert math.hypot(position[0], position[1]) == pytest.approx(0.0, abs=1e-12)
    assert position[2] == pytest.approx(MDH_ZERO_HEIGHT_M)
    violations = gate_fence(ZERO, limits)
    assert violations and violations[0].reason == REASON_FENCE
    assert violations[0].action == ACTION_STOP


def test_g6_fence_flags_below_table(limits):
    """反例：TCP 低于桌面 → 违规。"""
    low_pose = (0.0, 0.8, 0.0, 0.0, 0.0, 0.0, 0.0)
    assert link7_position(low_pose)[2] < limits.table_height_m
    violations = gate_fence(low_pose, limits)
    assert any("桌面" in v.detail for v in violations)
    assert all(v.reason == REASON_FENCE for v in violations)


# ── G7：状态帧 age 超时 ─────────────────────────────────────────────────
def test_g7_state_age_reports_timeout(limits):
    """反例：age 超阈值 → 违规（reason=state_age，action=stop）。"""
    violations = gate_state_age(SafetyState(state_age_s=0.2), limits)
    assert violations and violations[0].reason == REASON_STATE_AGE
    assert violations[0].gate == "G7"
    assert violations[0].action == ACTION_STOP


def test_g7_state_age_accepts_fresh_frame(limits):
    """正例：age 未超阈值 → 无违规。"""
    assert gate_state_age(SafetyState(state_age_s=0.001), limits) == []
    assert gate_state_age(SafetyState(state_age_s=0.05), limits) == []


# ── 编排：apply_safety ─────────────────────────────────────────────────
def test_apply_safety_clamps_and_reports_violations(limits):
    """越限输入被钳到安全值，且违规列表非空。

    基准取安全构型（否则零位构型会额外触发 G6 围栏闸）。
    """
    prev = list(SAFE_POSE)
    command = [50.0] + prev[1:]
    result = apply_safety(command, prev_command=prev, limits=limits)
    assert isinstance(result, SafetyResult)
    assert result.violations, "越限必须有违规"
    assert result.action == ACTION_CLAMP
    # 钳位后仍满足 G1（软限位）
    for i in range(7):
        assert limits.soft_lower_rad[i] <= result.command[i] <= limits.soft_upper_rad[i]
    assert REASON_SOFT_LIMIT in result.reasons()


def test_apply_safety_result_still_satisfies_g1(limits):
    """钳位顺序固定：软限位 ∩ 增量 ∩ 加速度 ⇒ 钳位后同时满足 G1/G2/G3。"""
    prev = list(SAFE_POSE)
    result = apply_safety(
        [100.0, -100.0, 0.0, 0.0, 0.0, 0.0, 0.0], prev_command=prev, limits=limits
    )
    for i in range(7):
        assert limits.soft_lower_rad[i] <= result.command[i] <= limits.soft_upper_rad[i]
        assert abs(result.command[i] - prev[i]) <= limits.max_velocity_rad_s[i] * DT + 1e-12


def test_apply_safety_resolves_conflict_with_soft_limit_priority(limits):
    """约束冲突（上一拍指令本身越限）⇒ 软限位优先，且必须留痕。"""
    prev = [limits.soft_upper_rad[0] + 0.5] + [0.0] * 6
    result = apply_safety([0.0] * 7, prev_command=prev, limits=limits)
    assert result.command[0] <= limits.soft_upper_rad[0]
    soft = result.violations_of(REASON_SOFT_LIMIT)
    assert soft, "冲突必须报软限位违规"
    assert "冲突" in soft[0].detail


def test_apply_safety_is_pure(limits):
    """纯函数：同样的输入 → 同样的输出；入参不被修改。"""
    command = [0.3, -0.2, 0.1, 0.0, 0.0, 0.0, 0.0]
    prev = list(ZERO)
    command_copy = list(command)
    prev_copy = list(prev)
    first = apply_safety(command, prev_command=prev, limits=limits)
    second = apply_safety(command, prev_command=prev, limits=limits)
    assert first.command == second.command
    assert first.violations == second.violations
    assert first.action == second.action
    assert command == command_copy
    assert prev == prev_copy


def test_apply_safety_reports_monitor_gates(limits):
    """监视闸（G4/G5/G6/G7）不改写指令，但会抬高 action。"""
    safe_command = list(SAFE_POSE)
    result = apply_safety(
        safe_command,
        prev_command=list(SAFE_POSE),
        limits=limits,
        state=SafetyState(joint_currents_ma=[9000.0] * 7, state_age_s=0.5),
    )
    assert result.command == pytest.approx(tuple(safe_command))
    reasons = set(result.reasons())
    assert REASON_CURRENT in reasons
    assert REASON_STATE_AGE in reasons
    assert REASON_WATCHDOG in reasons
    assert result.action == ACTION_STOP


def test_apply_safety_clean_case_has_no_violation(limits):
    """正例：安全构型 + 小步进 → 无违规、action=proceed。"""
    prev = list(SAFE_POSE)
    command = [prev[0] + 0.0001] + prev[1:]
    result = apply_safety(command, prev_command=prev, limits=limits)
    assert result.ok
    assert result.action == ACTION_PROCEED
    assert result.violations == ()


def test_apply_safety_accepts_previous_velocity(limits):
    """上一拍速度参与加速度闸（起步拍可显式给 0）。"""
    prev = list(ZERO)
    result = apply_safety(
        [0.001] + [0.0] * 6,
        prev_command=prev,
        prev_velocity_rad_s=[0.2] + [0.0] * 6,
        limits=limits,
    )
    assert REASON_ACCEL_CLAMP not in result.reasons()


def test_apply_safety_rejects_invalid_inputs(limits):
    """负例：维度错 / NaN / 非法 dt → ValueError（点名参数）。"""
    with pytest.raises(ValueError, match="command"):
        apply_safety([0.0] * 6, prev_command=list(ZERO), limits=limits)
    with pytest.raises(ValueError, match="command"):
        apply_safety(
            [float("nan")] + [0.0] * 6, prev_command=list(ZERO), limits=limits
        )
    with pytest.raises(ValueError, match="prev_command"):
        apply_safety(list(ZERO), prev_command=[0.0] * 3, limits=limits)
    with pytest.raises(ValueError, match="dt_s"):
        apply_safety(list(ZERO), prev_command=list(ZERO), limits=limits, dt_s=0.0)


def test_reason_labels_are_stable_and_complete():
    """七闸 reason 标签必须与切片计划一致（写进运行日志的稳定口径）。"""
    assert ALL_REASONS == (
        "soft_limit",
        "delta_clamp",
        "accel_clamp",
        "watchdog",
        "current",
        "fence",
        "state_age",
    )
    for reason in ALL_REASONS:
        assert reason == reason.strip().lower()
        assert " " not in reason


def test_violation_and_result_are_serializable(limits):
    """违规 / 结果可 JSON 序列化（供运行日志写入）。"""
    import json

    result = apply_safety(
        [100.0] + [0.0] * 6, prev_command=list(ZERO), limits=limits
    )
    payload = json.dumps(result.to_dict(), ensure_ascii=False)
    assert "soft_limit" in payload
    violation = SafetyViolation(reason="soft_limit", gate="G1", detail="x")
    assert violation.to_dict()["joints"] == []


def test_safety_module_has_no_drake_dependency():
    """设备层零 pydrake 依赖（保证无真机 / 无 Drake 环境可测）。"""
    import subprocess

    code = (
        "import sys; sys.path.insert(0, %r);"
        "import rm75_drake.device.safety;"
        "print('pydrake' in sys.modules)" % str(SRC_DIR)
    )
    out = subprocess.run(
        [sys.executable, "-c", code], capture_output=True, text=True, check=True
    )
    assert out.stdout.strip() == "False", out.stdout
