"""安全七闸（S03-T02）：**纯函数**实现，零真机 / 零 SDK 依赖。

为什么是纯函数
--------------
研究 F5：UDP 主动上报的一帧里已经含有软限位判定、增量、加速度、看门狗、
电流、围栏与状态 age 所需的**几乎全部信号**——不需要额外的 TCP 查询
（那会占用指令通道、破坏「单写者」契约）。

于是七闸被写成**纯函数**：

* 输入 = 上一拍指令（+ 上一拍速度）+ 本拍状态 + 阈值配置；
* 输出 = 钳位后指令 + 违规列表。

纯函数 ⇒ 没有真机也能对每一闸做**正例 / 反例**单元测试
（见 :mod:`tests.test_device_safety`），这是 S03「离线可完整验证的代码层」
的基石。

七闸定义
--------
=====  =======  ====================================================
闸     reason   语义
=====  =======  ====================================================
G1     ``soft_limit``    软件软限位（消费 :func:`rm75_drake.limits.load_joint_limits`
                         的 ``software`` 段——硬件限位每侧再收 4°）
G2     ``delta_clamp``   单步增量钳位（每轴 ω_max；官方表 J1/J2 3.1416、
                         J3–J7 3.927 rad/s）
G3     ``accel_clamp``   加速度钳位（每轴 a_max，取设备配置的标定值）
G4     ``watchdog``      心跳看门狗（3 周期告警、>10 周期缓停阈值）
G5     ``current``       每轴电流阈值（mA）
G6     ``fence``         几何围栏（TCP 位置用
                         :func:`rm75_drake.kinematics.mdh.link7_position`
                         在 5 ms 环内算，**零 Drake 依赖**）
G7     ``state_age``     状态帧 age 超时
=====  =======  ====================================================

钳位顺序（**固定**，有单测钉死）
------------------------------
G1 / G2 / G3 三条**位置约束**在每个关节上求**区间交集**后一次性钳位
（见 :func:`_clamp_joint`）：

* 软限位区间 ``[lower, upper]``；
* 增量区间 ``prev ± ω_max · dt``；
* 加速度区间 ``prev + (v_prev ± a_max · dt) · dt``。

区间非空 ⇒ 钳位后**同时**满足 G1/G2/G3（这正是「钳位后仍满足 G1」的实现）；
区间为空（约束互相打架，例如上一拍指令本身就在限位之外）⇒ 冲突时
**软限位优先**（硬件边界不可协商），并在对应闸的违规详情里点名冲突。

G4–G7 是**监视闸**（不直接改写指令），但违规同样进列表并驱动安全态
（:class:`SafetyResult.action`）。

单位纪律
--------
全部 **rad / rad·s⁻¹ / rad·s⁻² / mA / m / s**。
本模块**不实现** rad↔deg 换算（唯一转换点在 :mod:`rm75_drake.device.sdk`）。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

from ..kinematics.mdh import link7_position
from ..limits import load_joint_limits
from ..planning.trajectory import JOINT_COUNT
from .config import DeviceConfig
from .sdk import deg_to_rad

__all__ = [
    # 稳定 reason 标签
    "REASON_SOFT_LIMIT",
    "REASON_DELTA_CLAMP",
    "REASON_ACCEL_CLAMP",
    "REASON_WATCHDOG",
    "REASON_CURRENT",
    "REASON_FENCE",
    "REASON_STATE_AGE",
    "ALL_REASONS",
    # 动作
    "ACTION_PROCEED",
    "ACTION_CLAMP",
    "ACTION_SLOW_STOP",
    "ACTION_STOP",
    # 数据结构
    "SafetyLimits",
    "SafetyState",
    "SafetyViolation",
    "SafetyResult",
    # 闸
    "gate_soft_limit",
    "gate_delta_clamp",
    "gate_accel_clamp",
    "gate_watchdog",
    "gate_current",
    "gate_fence",
    "gate_state_age",
    # 编排
    "apply_safety",
    # 阈值来源
    "safety_limits_from_sources",
    "load_safety_limits",
]


# ── 稳定 reason 标签 ────────────────────────────────────────────────────
REASON_SOFT_LIMIT = "soft_limit"
REASON_DELTA_CLAMP = "delta_clamp"
REASON_ACCEL_CLAMP = "accel_clamp"
REASON_WATCHDOG = "watchdog"
REASON_CURRENT = "current"
REASON_FENCE = "fence"
REASON_STATE_AGE = "state_age"

#: 全部七闸的 reason 标签（顺序即 G1→G7）。
ALL_REASONS: Tuple[str, ...] = (
    REASON_SOFT_LIMIT,
    REASON_DELTA_CLAMP,
    REASON_ACCEL_CLAMP,
    REASON_WATCHDOG,
    REASON_CURRENT,
    REASON_FENCE,
    REASON_STATE_AGE,
)

#: 闸号（与切片计划里的 G1–G7 对应）。
_GATE_SOFT_LIMIT = "G1"
_GATE_DELTA_CLAMP = "G2"
_GATE_ACCEL_CLAMP = "G3"
_GATE_WATCHDOG = "G4"
_GATE_CURRENT = "G5"
_GATE_FENCE = "G6"
_GATE_STATE_AGE = "G7"


# ── 安全态动作（严重度递增）─────────────────────────────────────────────
ACTION_PROCEED = "proceed"
ACTION_CLAMP = "clamp"
ACTION_SLOW_STOP = "slow_stop"
ACTION_STOP = "stop"

_ACTION_SEVERITY = {
    ACTION_PROCEED: 0,
    ACTION_CLAMP: 1,
    ACTION_SLOW_STOP: 2,
    ACTION_STOP: 3,
}

#: 严重度标签。
_SEVERITY_INFO = "info"
_SEVERITY_WARN = "warn"
_SEVERITY_CRITICAL = "critical"


# ── 阈值配置 ────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class SafetyLimits:
    """安全七闸的全部阈值（**不可变**）。

    字段
    ----
    soft_lower_rad / soft_upper_rad:
        软件软限位（rad，7 维）——来自 ``joint_limits.yaml`` 的 ``software`` 段。
    max_velocity_rad_s:
        每轴最大角速度（rad/s，7 维）——官方参数表 J1/J2 3.1416、J3–J7 3.927。
    max_accel_rad_s2:
        每轴最大加速度（rad/s²，7 维）——来自设备配置的标定值。
    current_threshold_ma:
        每轴电流阈值（mA，7 维）。
    table_height_m:
        桌面高度（m）——TCP 低于该高度即越界。
    singularity_radius_m:
        奇异圆柱半径（m）——TCP 距基座竖轴的水平距离小于该值即判越界。
    cycle_period_s:
        状态上报周期（s）——用于把 age 折算成「心跳周期数」。
    watchdog_warn_cycles:
        心跳看门狗告警阈值（周期数）。
    watchdog_slow_stop_cycles:
        心跳看门狗缓停阈值（周期数，**严格大于**该值才缓停）。
    state_age_timeout_s:
        状态帧 age 超时阈值（s）——G7。
    """

    soft_lower_rad: Tuple[float, ...]
    soft_upper_rad: Tuple[float, ...]
    max_velocity_rad_s: Tuple[float, ...]
    max_accel_rad_s2: Tuple[float, ...]
    current_threshold_ma: Tuple[float, ...]
    table_height_m: float
    singularity_radius_m: float
    cycle_period_s: float = 0.005
    watchdog_warn_cycles: int = 3
    watchdog_slow_stop_cycles: int = 10
    state_age_timeout_s: float = 0.05

    def __post_init__(self) -> None:
        for name in (
            "soft_lower_rad",
            "soft_upper_rad",
            "max_velocity_rad_s",
            "max_accel_rad_s2",
            "current_threshold_ma",
        ):
            values = tuple(float(v) for v in getattr(self, name))
            if len(values) != JOINT_COUNT:
                raise ValueError(
                    f"{name} 必须是 {JOINT_COUNT} 维，实际是 {len(values)} 维"
                )
            if not all(math.isfinite(v) for v in values):
                raise ValueError(f"{name} 含有 NaN/Inf：{values}")
            object.__setattr__(self, name, values)
        if not float(self.table_height_m) == self.table_height_m:
            raise ValueError("table_height_m 必须是有限数值")
        if not float(self.singularity_radius_m) > 0.0:
            raise ValueError(
                f"singularity_radius_m 必须为正，实际是 {self.singularity_radius_m}"
            )
        if not float(self.cycle_period_s) > 0.0:
            raise ValueError(f"cycle_period_s 必须为正，实际是 {self.cycle_period_s}")
        if int(self.watchdog_warn_cycles) < 1:
            raise ValueError("watchdog_warn_cycles 必须 ≥ 1")
        if int(self.watchdog_slow_stop_cycles) <= int(self.watchdog_warn_cycles):
            raise ValueError("watchdog_slow_stop_cycles 必须大于 watchdog_warn_cycles")
        if not float(self.state_age_timeout_s) > 0.0:
            raise ValueError("state_age_timeout_s 必须为正")
        # 软限位必须上下界有序且跨过 0
        for i in range(JOINT_COUNT):
            lo = self.soft_lower_rad[i]
            hi = self.soft_upper_rad[i]
            if not lo < hi:
                raise ValueError(f"关节 {i} 的软限位区间非法：[{lo}, {hi}]")
        # 速度 / 加速度 / 电流阈值必须为正
        for name in ("max_velocity_rad_s", "max_accel_rad_s2", "current_threshold_ma"):
            values = getattr(self, name)
            if any(v <= 0.0 for v in values):
                raise ValueError(f"{name} 必须全为正：{values}")

    def to_dict(self) -> Dict[str, Any]:
        """返回可 JSON 序列化的字典（写进运行日志）。"""
        return {
            "soft_lower_rad": list(self.soft_lower_rad),
            "soft_upper_rad": list(self.soft_upper_rad),
            "max_velocity_rad_s": list(self.max_velocity_rad_s),
            "max_accel_rad_s2": list(self.max_accel_rad_s2),
            "current_threshold_ma": list(self.current_threshold_ma),
            "table_height_m": float(self.table_height_m),
            "singularity_radius_m": float(self.singularity_radius_m),
            "cycle_period_s": float(self.cycle_period_s),
            "watchdog_warn_cycles": int(self.watchdog_warn_cycles),
            "watchdog_slow_stop_cycles": int(self.watchdog_slow_stop_cycles),
            "state_age_timeout_s": float(self.state_age_timeout_s),
        }


@dataclass(frozen=True)
class SafetyState:
    """本拍状态（**纯数据**，不含任何真机句柄）。

    字段
    ----
    joint_currents_ma:
        逐轴电流（mA，7 维）——G5 用。
    state_age_s:
        最新状态帧的 age（s）——G4 / G7 用。
    missed_cycles:
        状态源统计的心跳缺失周期数；``None`` ⇒ 由 ``state_age_s / cycle_period_s``
        折算。
    """

    joint_currents_ma: Tuple[float, ...] = (0.0,) * JOINT_COUNT
    state_age_s: float = 0.0
    missed_cycles: Optional[int] = None

    def __post_init__(self) -> None:
        values = tuple(float(v) for v in self.joint_currents_ma)
        if len(values) != JOINT_COUNT:
            raise ValueError(
                f"joint_currents_ma 必须是 {JOINT_COUNT} 维，实际是 {len(values)} 维"
            )
        if not all(math.isfinite(v) for v in values):
            raise ValueError(f"joint_currents_ma 含有 NaN/Inf：{values}")
        object.__setattr__(self, "joint_currents_ma", values)
        age = float(self.state_age_s)
        if not math.isfinite(age) or age < 0.0:
            raise ValueError(f"state_age_s 必须是非负有限数值，实际是 {age}")
        object.__setattr__(self, "state_age_s", age)
        if self.missed_cycles is not None:
            object.__setattr__(self, "missed_cycles", int(self.missed_cycles))


@dataclass(frozen=True)
class SafetyViolation:
    """一条安全闸违规（**不吞掉**：全部进列表，可写进运行日志）。

    字段
    ----
    reason:
        稳定标签（见 :data:`ALL_REASONS`）。
    gate:
        闸号（``G1``…``G7``）。
    detail:
        人类可读描述。
    joints:
        涉及的关节下标（空元组 = 整机级）。
    severity:
        ``info`` / ``warn`` / ``critical``。
    action:
        该违规要求的动作（见 ``ACTION_*``）。
    """

    reason: str
    gate: str
    detail: str
    joints: Tuple[int, ...] = ()
    severity: str = _SEVERITY_WARN
    action: str = ACTION_CLAMP

    def to_dict(self) -> Dict[str, Any]:
        """返回可 JSON 序列化的字典。"""
        return {
            "reason": self.reason,
            "gate": self.gate,
            "detail": self.detail,
            "joints": list(self.joints),
            "severity": self.severity,
            "action": self.action,
        }


@dataclass(frozen=True)
class SafetyResult:
    """七闸一次裁决的结果。

    字段
    ----
    command:
        **钳位后**的指令（rad，7 维）——唯一可下发的版本。
    violations:
        违规列表（**有序**，按闸号 G1→G7）。
    action:
        整体要求的安全态动作（:data:`ACTION_PROCEED` 最轻）。
    """

    command: Tuple[float, ...]
    violations: Tuple[SafetyViolation, ...] = ()
    action: str = ACTION_PROCEED

    @property
    def ok(self) -> bool:
        """是否无任何违规。"""
        return not self.violations

    def reasons(self) -> Tuple[str, ...]:
        """返回本次裁决触发的 reason 标签（去重、保序）。"""
        seen: List[str] = []
        for v in self.violations:
            if v.reason not in seen:
                seen.append(v.reason)
        return tuple(seen)

    def violations_of(self, reason: str) -> Tuple[SafetyViolation, ...]:
        """返回指定 reason 的全部违规。"""
        return tuple(v for v in self.violations if v.reason == reason)

    def to_dict(self) -> Dict[str, Any]:
        """返回可 JSON 序列化的字典（写进运行日志）。"""
        return {
            "command_rad": list(self.command),
            "action": self.action,
            "violations": [v.to_dict() for v in self.violations],
        }


# ── 内部工具 ────────────────────────────────────────────────────────────
def _as_vector(values: Sequence[float], name: str) -> Tuple[float, ...]:
    """校验关节向量：长度 7、元素为有限实数。"""
    if isinstance(values, (str, bytes)) or not isinstance(values, Sequence):
        raise ValueError(f"{name} 必须是 {JOINT_COUNT} 维序列，收到 {type(values).__name__}")
    if len(values) != JOINT_COUNT:
        raise ValueError(f"{name} 必须是 {JOINT_COUNT} 维序列（实际 {len(values)} 维）")
    out: List[float] = []
    for i, v in enumerate(values):
        if isinstance(v, bool) or not isinstance(v, (int, float)):
            raise ValueError(f"{name}[{i}] 必须是数值，收到 {v!r}")
        fv = float(v)
        if not math.isfinite(fv):
            raise ValueError(f"{name}[{i}] 必须是有限数值，收到 {v!r}")
        out.append(fv)
    return tuple(out)


def _worst_action(actions: Sequence[str]) -> str:
    """取最严重的动作。"""
    worst = ACTION_PROCEED
    for action in actions:
        if _ACTION_SEVERITY.get(action, 0) > _ACTION_SEVERITY[worst]:
            worst = action
    return worst


def _clamp(value: float, low: float, high: float) -> float:
    """把 ``value`` 钳位到 ``[low, high]``（``low <= high`` 时才有意义）。"""
    if value < low:
        return low
    if value > high:
        return high
    return value


# ── G1：软件软限位 ──────────────────────────────────────────────────────
def gate_soft_limit(
    command: Sequence[float], limits: SafetyLimits
) -> Tuple[Tuple[float, ...], List[SafetyViolation]]:
    """G1：把指令钳位到软件软限位区间内。

    返回
    ----
    ``(clamped, violations)``
    """
    values = _as_vector(command, "command")
    clamped: List[float] = []
    violated: List[int] = []
    for i in range(JOINT_COUNT):
        lo = limits.soft_lower_rad[i]
        hi = limits.soft_upper_rad[i]
        if values[i] < lo or values[i] > hi:
            violated.append(i)
        clamped.append(_clamp(values[i], lo, hi))
    violations: List[SafetyViolation] = []
    if violated:
        violations.append(
            SafetyViolation(
                reason=REASON_SOFT_LIMIT,
                gate=_GATE_SOFT_LIMIT,
                detail=(
                    "指令超出软件软限位："
                    + "；".join(
                        f"joint_{i + 1} 请求 {values[i]:.6f} rad，"
                        f"限位 [{limits.soft_lower_rad[i]:.6f}, "
                        f"{limits.soft_upper_rad[i]:.6f}]"
                        for i in violated
                    )
                ),
                joints=tuple(violated),
                severity=_SEVERITY_WARN,
                action=ACTION_CLAMP,
            )
        )
    return tuple(clamped), violations


# ── G2：单步增量钳位 ────────────────────────────────────────────────────
def gate_delta_clamp(
    command: Sequence[float],
    prev_command: Sequence[float],
    limits: SafetyLimits,
    dt_s: float,
) -> Tuple[Tuple[float, ...], List[SafetyViolation]]:
    """G2：单步增量钳位（每轴 |Δq| ≤ ω_max · dt）。

    返回
    ----
    ``(clamped, violations)``
    """
    values = _as_vector(command, "command")
    prev = _as_vector(prev_command, "prev_command")
    dt = float(dt_s)
    if not math.isfinite(dt) or dt <= 0.0:
        raise ValueError(f"dt_s 必须是正有限数值，实际是 {dt_s!r}")
    clamped: List[float] = []
    violated: List[int] = []
    for i in range(JOINT_COUNT):
        max_step = limits.max_velocity_rad_s[i] * dt
        delta = values[i] - prev[i]
        if abs(delta) > max_step:
            violated.append(i)
            delta = math.copysign(max_step, delta)
        clamped.append(prev[i] + delta)
    violations: List[SafetyViolation] = []
    if violated:
        violations.append(
            SafetyViolation(
                reason=REASON_DELTA_CLAMP,
                gate=_GATE_DELTA_CLAMP,
                detail=(
                    "单步增量超过 ω_max·dt："
                    + "；".join(
                        f"joint_{i + 1} 请求 Δ={values[i] - prev[i]:.6f} rad，"
                        f"上限 {limits.max_velocity_rad_s[i] * dt:.6f} rad"
                        for i in violated
                    )
                ),
                joints=tuple(violated),
                severity=_SEVERITY_WARN,
                action=ACTION_CLAMP,
            )
        )
    return tuple(clamped), violations


# ── G3：加速度钳位 ──────────────────────────────────────────────────────
def gate_accel_clamp(
    command: Sequence[float],
    prev_command: Sequence[float],
    prev_velocity_rad_s: Sequence[float],
    limits: SafetyLimits,
    dt_s: float,
) -> Tuple[Tuple[float, ...], List[SafetyViolation]]:
    """G3：加速度钳位（每轴 |v − v_prev| ≤ a_max · dt）。

    返回
    ----
    ``(clamped, violations)``
    """
    values = _as_vector(command, "command")
    prev = _as_vector(prev_command, "prev_command")
    prev_v = _as_vector(prev_velocity_rad_s, "prev_velocity_rad_s")
    dt = float(dt_s)
    if not math.isfinite(dt) or dt <= 0.0:
        raise ValueError(f"dt_s 必须是正有限数值，实际是 {dt_s!r}")
    clamped: List[float] = []
    violated: List[int] = []
    for i in range(JOINT_COUNT):
        v = (values[i] - prev[i]) / dt
        max_dv = limits.max_accel_rad_s2[i] * dt
        if abs(v - prev_v[i]) > max_dv:
            violated.append(i)
            v = prev_v[i] + math.copysign(max_dv, v - prev_v[i])
        clamped.append(prev[i] + v * dt)
    violations: List[SafetyViolation] = []
    if violated:
        violations.append(
            SafetyViolation(
                reason=REASON_ACCEL_CLAMP,
                gate=_GATE_ACCEL_CLAMP,
                detail=(
                    "加速度超过 a_max："
                    + "；".join(
                        f"joint_{i + 1} 请求 v={(values[i] - prev[i]) / dt:.6f} rad/s，"
                        f"上一拍 v={prev_v[i]:.6f} rad/s，"
                        f"上限 Δv={limits.max_accel_rad_s2[i] * dt:.6f} rad/s"
                        for i in violated
                    )
                ),
                joints=tuple(violated),
                severity=_SEVERITY_WARN,
                action=ACTION_CLAMP,
            )
        )
    return tuple(clamped), violations


# ── G4：心跳看门狗 ──────────────────────────────────────────────────────
def gate_watchdog(
    state: SafetyState, limits: SafetyLimits
) -> List[SafetyViolation]:
    """G4：心跳看门狗（≥ 告警周期数告警；> 缓停周期数要求缓停）。

    返回
    ----
    ``violations``
    """
    cycles = (
        int(state.missed_cycles)
        if state.missed_cycles is not None
        else int(state.state_age_s / limits.cycle_period_s)
    )
    if cycles < limits.watchdog_warn_cycles:
        return []
    slow = cycles > limits.watchdog_slow_stop_cycles
    return [
        SafetyViolation(
            reason=REASON_WATCHDOG,
            gate=_GATE_WATCHDOG,
            detail=(
                f"心跳缺失 {cycles} 个周期"
                f"（告警阈值 {limits.watchdog_warn_cycles}，"
                f"缓停阈值 >{limits.watchdog_slow_stop_cycles}），"
                f"state_age={state.state_age_s * 1000.0:.3f} ms"
            ),
            severity=_SEVERITY_CRITICAL if slow else _SEVERITY_WARN,
            action=ACTION_SLOW_STOP if slow else ACTION_CLAMP,
        )
    ]


# ── G5：电流阈值 ────────────────────────────────────────────────────────
def gate_current(state: SafetyState, limits: SafetyLimits) -> List[SafetyViolation]:
    """G5：逐轴电流阈值（mA）；越限即要求停机。"""
    violated = [
        i
        for i in range(JOINT_COUNT)
        if abs(state.joint_currents_ma[i]) > limits.current_threshold_ma[i]
    ]
    if not violated:
        return []
    return [
        SafetyViolation(
            reason=REASON_CURRENT,
            gate=_GATE_CURRENT,
            detail=(
                "电流越限："
                + "；".join(
                    f"joint_{i + 1} {state.joint_currents_ma[i]:.1f} mA "
                    f"> 阈值 {limits.current_threshold_ma[i]:.1f} mA"
                    for i in violated
                )
            ),
            joints=tuple(violated),
            severity=_SEVERITY_CRITICAL,
            action=ACTION_STOP,
        )
    ]


# ── G6：几何围栏 ────────────────────────────────────────────────────────
def gate_fence(command: Sequence[float], limits: SafetyLimits) -> List[SafetyViolation]:
    """G6：几何围栏（桌面高度 + 奇异圆柱半径）。

    TCP 位置用 :func:`rm75_drake.kinematics.mdh.link7_position` 计算
    （**零 Drake 依赖**，可在 5 ms 环内跑）。

    判定
    ----
    * TCP 高度 < ``table_height_m`` ⇒ 低于桌面（越界）；
    * TCP 到基座竖轴的水平距离 < ``singularity_radius_m`` ⇒ 进入奇异圆柱
      （越界；注意**零位构型**的 TCP 正在基座轴线上方，属于该区域）。
    """
    values = _as_vector(command, "command")
    position = link7_position(values)
    x, y, z = (float(position[0]), float(position[1]), float(position[2]))
    radial = math.hypot(x, y)
    violations: List[SafetyViolation] = []
    if z < limits.table_height_m:
        violations.append(
            SafetyViolation(
                reason=REASON_FENCE,
                gate=_GATE_FENCE,
                detail=(
                    f"TCP 高度 {z:.4f} m 低于桌面高度 {limits.table_height_m:.4f} m"
                ),
                severity=_SEVERITY_CRITICAL,
                action=ACTION_STOP,
            )
        )
    if radial < limits.singularity_radius_m:
        violations.append(
            SafetyViolation(
                reason=REASON_FENCE,
                gate=_GATE_FENCE,
                detail=(
                    f"TCP 到基座轴线距离 {radial:.4f} m 小于奇异圆柱半径 "
                    f"{limits.singularity_radius_m:.4f} m"
                ),
                severity=_SEVERITY_CRITICAL,
                action=ACTION_STOP,
            )
        )
    return violations


# ── G7：状态帧 age 超时 ─────────────────────────────────────────────────
def gate_state_age(state: SafetyState, limits: SafetyLimits) -> List[SafetyViolation]:
    """G7：状态帧 age 超时（超过 ``state_age_timeout_s`` 即要求停机）。"""
    if state.state_age_s <= limits.state_age_timeout_s:
        return []
    return [
        SafetyViolation(
            reason=REASON_STATE_AGE,
            gate=_GATE_STATE_AGE,
            detail=(
                f"状态帧 age {state.state_age_s * 1000.0:.3f} ms 超过阈值 "
                f"{limits.state_age_timeout_s * 1000.0:.3f} ms"
            ),
            severity=_SEVERITY_CRITICAL,
            action=ACTION_STOP,
        )
    ]


# ── 编排：唯一计算「钳位后指令」的地方 ──────────────────────────────────
def _clamp_joint(
    requested: float,
    prev: float,
    prev_velocity: float,
    limits: SafetyLimits,
    index: int,
    dt_s: float,
) -> Tuple[float, bool, bool]:
    """单关节钳位：软限位 ∩ 增量 ∩ 加速度。

    返回
    ----
    ``(value, soft_violated, conflict)``
        ``conflict=True`` 表示三条约束互相打架（交集为空）——此时
        **软限位优先**，并额外用增量/加速度做尽力收缩。
    """
    lo_soft = limits.soft_lower_rad[index]
    hi_soft = limits.soft_upper_rad[index]
    max_step = limits.max_velocity_rad_s[index] * dt_s
    lo_delta = prev - max_step
    hi_delta = prev + max_step
    max_dv = limits.max_accel_rad_s2[index] * dt_s
    lo_accel = prev + (prev_velocity - max_dv) * dt_s
    hi_accel = prev + (prev_velocity + max_dv) * dt_s

    lo = max(lo_soft, lo_delta, lo_accel)
    hi = min(hi_soft, hi_delta, hi_accel)

    if lo <= hi:
        soft_violated = requested < lo_soft or requested > hi_soft
        return _clamp(requested, lo, hi), soft_violated, False

    # 冲突：软限位优先（硬件边界不可协商）
    conflict = True
    value = _clamp(requested, lo_soft, hi_soft)
    # 在软限位内尽力满足增量约束
    if value < prev - max_step:
        value = prev - max_step
    elif value > prev + max_step:
        value = prev + max_step
    # 再钳回软限位（软限位优先）
    value = _clamp(value, lo_soft, hi_soft)
    soft_violated = requested < lo_soft or requested > hi_soft
    return value, soft_violated, conflict


def apply_safety(
    command: Sequence[float],
    *,
    prev_command: Sequence[float],
    limits: SafetyLimits,
    state: Optional[SafetyState] = None,
    prev_velocity_rad_s: Optional[Sequence[float]] = None,
    dt_s: Optional[float] = None,
) -> SafetyResult:
    """跑完安全七闸，返回**钳位后指令 + 违规列表**（纯函数）。

    参数
    ----
    command:
        本拍期望下发的指令（rad，7 维）。
    prev_command:
        上一拍**实际下发**的指令（rad，7 维）——增量/加速度的基准。
    limits:
        阈值配置。
    state:
        本拍状态（电流 / age）；``None`` ⇒ 视为全零电流、age = 0。
    prev_velocity_rad_s:
        上一拍速度（rad/s，7 维）；``None`` ⇒ 视为 0（起步拍）。
    dt_s:
        本拍周期（s）；``None`` ⇒ 用 ``limits.cycle_period_s``。

    返回
    ----
    :class:`SafetyResult`
        ``command`` 是**唯一可下发**的版本；``violations`` 是全部违规
        （**不吞掉**，按闸号 G1→G7 排序）；``action`` 是整体安全态动作。

    说明
    ----
    * 钳位后的指令**同时**满足 G1/G2/G3（区间交集，见 :func:`_clamp_joint`）；
    * G4–G7 是监视闸：它们不改写指令，但会把 ``action`` 抬到
      ``slow_stop`` / ``stop``。
    """
    values = _as_vector(command, "command")
    prev = _as_vector(prev_command, "prev_command")
    if prev_velocity_rad_s is None:
        prev_v = (0.0,) * JOINT_COUNT
    else:
        prev_v = _as_vector(prev_velocity_rad_s, "prev_velocity_rad_s")
    dt = float(limits.cycle_period_s if dt_s is None else dt_s)
    if not math.isfinite(dt) or dt <= 0.0:
        raise ValueError(f"dt_s 必须是正有限数值，实际是 {dt_s!r}")
    current_state = state if state is not None else SafetyState()

    # ── 位置类三闸：一次算完（G1/G2/G3 区间交集）────────────────────────
    clamped: List[float] = []
    soft_joints: List[int] = []
    delta_joints: List[int] = []
    accel_joints: List[int] = []
    conflict_joints: List[int] = []
    for i in range(JOINT_COUNT):
        value, soft_hit, conflict = _clamp_joint(
            values[i], prev[i], prev_v[i], limits, i, dt
        )
        clamped.append(value)
        if soft_hit:
            soft_joints.append(i)
        if conflict:
            conflict_joints.append(i)
            # 约束冲突 ⇒ 一定报软限位违规（软限位优先，必须留痕）
            if i not in soft_joints:
                soft_joints.append(i)
        # 增量 / 加速度是否被触发（用**请求值**判定，便于解释）
        max_step = limits.max_velocity_rad_s[i] * dt
        if abs(values[i] - prev[i]) > max_step:
            delta_joints.append(i)
        max_dv = limits.max_accel_rad_s2[i] * dt
        if abs((values[i] - prev[i]) / dt - prev_v[i]) > max_dv:
            accel_joints.append(i)

    violations: List[SafetyViolation] = []

    if soft_joints:
        detail = "；".join(
            f"joint_{i + 1} 请求 {values[i]:.6f} rad，"
            f"限位 [{limits.soft_lower_rad[i]:.6f}, {limits.soft_upper_rad[i]:.6f}]"
            for i in soft_joints
        )
        if conflict_joints:
            detail += (
                "；约束冲突（软限位 ∩ 增量 ∩ 加速度 交集为空），已按软限位优先处理："
                + "、".join(f"joint_{i + 1}" for i in conflict_joints)
            )
        violations.append(
            SafetyViolation(
                reason=REASON_SOFT_LIMIT,
                gate=_GATE_SOFT_LIMIT,
                detail=detail,
                joints=tuple(soft_joints),
                severity=_SEVERITY_WARN,
                action=ACTION_CLAMP,
            )
        )

    if delta_joints:
        violations.append(
            SafetyViolation(
                reason=REASON_DELTA_CLAMP,
                gate=_GATE_DELTA_CLAMP,
                detail="单步增量超过 ω_max·dt："
                + "；".join(
                    f"joint_{i + 1} 请求 Δ={values[i] - prev[i]:.6f} rad，"
                    f"上限 {limits.max_velocity_rad_s[i] * dt:.6f} rad"
                    for i in delta_joints
                ),
                joints=tuple(delta_joints),
                severity=_SEVERITY_WARN,
                action=ACTION_CLAMP,
            )
        )

    if accel_joints:
        violations.append(
            SafetyViolation(
                reason=REASON_ACCEL_CLAMP,
                gate=_GATE_ACCEL_CLAMP,
                detail="加速度超过 a_max："
                + "；".join(
                    f"joint_{i + 1} 请求 v={(values[i] - prev[i]) / dt:.6f} rad/s，"
                    f"上一拍 v={prev_v[i]:.6f} rad/s，"
                    f"上限 Δv={limits.max_accel_rad_s2[i] * dt:.6f} rad/s"
                    for i in accel_joints
                ),
                joints=tuple(accel_joints),
                severity=_SEVERITY_WARN,
                action=ACTION_CLAMP,
            )
        )

    # ── 监视类四闸：只报告，不改写指令 ───────────────────────────────────
    violations.extend(gate_watchdog(current_state, limits))
    violations.extend(gate_current(current_state, limits))
    violations.extend(gate_fence(tuple(clamped), limits))
    violations.extend(gate_state_age(current_state, limits))

    action = _worst_action([v.action for v in violations])
    return SafetyResult(
        command=tuple(clamped),
        violations=tuple(violations),
        action=action,
    )


# ── 阈值来源 ────────────────────────────────────────────────────────────
def safety_limits_from_sources(
    joint_limits: Mapping[str, Any], device_config: DeviceConfig
) -> SafetyLimits:
    """从「限位交接件 + 设备配置」构造 :class:`SafetyLimits`。

    * 软限位 / ω_max ← ``joint_limits.yaml`` 的 ``software`` 段；
    * a_max ← ``device_config.accel_limits_rad_s2``（标定值）；
    * 电流阈值 ← ``device_config.current_thresholds_ma``；
    * 围栏 ← ``device_config.fence_*``。

    **有效限位 = 模型软限位 ∩ 控制器限位**
    ------------------------------------------------
    真机实测（RM75-B / 控制器代次 3）：控制器自己的软限位**并不总是**
    比模型的软限位更紧——J1 控制器 ±125.998°（模型 ±174.0°）、
    J3 控制器 ±130.988°（模型 ±174.0°）、J5 控制器 ±123.998°（模型 ±174.0°）。
    只按模型钳位 ⇒ 会下发**控制器不接受**的目标位形。
    ⇒ 设备配置里若提供了 ``controller_joint_limits``（实测读取），
    有效限位取**交集**（更紧者胜）；未提供 ⇒ 退化为模型软限位（保持旧行为）。

    异常
    ----
    ValueError
        限位交接件结构非法（缺 ``joints`` / ``software`` 段 / 维度不对）。
    """
    joints = joint_limits.get("joints")
    if not isinstance(joints, list) or len(joints) != JOINT_COUNT:
        raise ValueError(
            f"joint_limits 的 joints 段必须是 {JOINT_COUNT} 项列表，"
            f"实际是 {joint_limits.get('joints')!r}"
        )
    lower: List[float] = []
    upper: List[float] = []
    velocity: List[float] = []
    for i, item in enumerate(joints):
        if not isinstance(item, Mapping):
            raise ValueError(f"joint_limits.joints[{i}] 必须是映射，实际是 {item!r}")
        software = item.get("software")
        if not isinstance(software, Mapping):
            raise ValueError(f"joint_limits.joints[{i}] 缺少 software 段")
        for key in ("position_lower_rad", "position_upper_rad", "velocity_rad_s"):
            if key not in software:
                raise ValueError(f"joint_limits.joints[{i}].software 缺少 {key}")
        lower.append(float(software["position_lower_rad"]))
        upper.append(float(software["position_upper_rad"]))
        velocity.append(float(software["velocity_rad_s"]))
    # 有效限位 = 模型软限位 ∩ 控制器限位（控制器更紧时取控制器值）
    controller_lower = getattr(device_config, "controller_lower_deg", None)
    controller_upper = getattr(device_config, "controller_upper_deg", None)
    if controller_lower is not None and controller_upper is not None:
        for index in range(JOINT_COUNT):
            lower[index] = max(lower[index], deg_to_rad(float(controller_lower[index])))
            upper[index] = min(upper[index], deg_to_rad(float(controller_upper[index])))

    return SafetyLimits(
        soft_lower_rad=tuple(lower),
        soft_upper_rad=tuple(upper),
        max_velocity_rad_s=tuple(velocity),
        max_accel_rad_s2=tuple(device_config.accel_limits_rad_s2),
        current_threshold_ma=tuple(device_config.current_thresholds_ma),
        table_height_m=float(device_config.fence_table_height_m),
        singularity_radius_m=float(device_config.fence_singularity_radius_m),
    )


def load_safety_limits(
    device_config: Optional[DeviceConfig] = None,
    joint_limits: Optional[Mapping[str, Any]] = None,
) -> SafetyLimits:
    """加载默认的安全阈值（限位交接件 + 默认设备配置）。"""
    from .config import load_device_config

    config = device_config if device_config is not None else load_device_config()
    limits = joint_limits if joint_limits is not None else load_joint_limits()
    return safety_limits_from_sources(limits, config)
