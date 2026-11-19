"""安全判据（**纯函数**）：故障类 → 等级 → 动作 → 恢复方式。

本模块是安全层的**判据内核**：

* **纯函数**：无 I/O、无 SDK、无线程、无时钟 —— 因此可被离线单测逐条钉死，
  也因此「安全判据独立于规划器」（R005）这条红线在代码层成立；
* **阈值不写死在本模块**：:class:`SafetyThresholds` 的四个阈值由调用方
  （执行层，持有 S03 冻结的 ``rm_executor/contract.py`` 常量）传入，
  避免出现第二处常量源；
* **判据不依赖规划器**：输入只是一组「故障信号」（跟随误差、断流时长、
  限值越界、控制器错误码、控制器状态），与任何规划/轨迹数据无关。

故障类 → 等级（**冻结**）
--------------------------

=========================  ==============================  =====  ===================
故障类                     判据                           等级   动作
=========================  ==============================  =====  ===================
``follow_error_light``    ``follow_warn_deg < e ≤ follow_stop_deg``  L1  set_speed_scale
``follow_error_heavy``    ``e > follow_stop_deg``        L2   pause
``limit_exceeded``        运行期位置/速度越过声明限位     L2   pause
``stream_lag_light``      ``stream_warn_ms < age ≤ stream_stop_ms``（**仅当**
                           ``enable_l2_pause_on_stream_lag``）  L2   pause
``stream_lost``           ``age > stream_stop_ms``       L4   stop
``collision``             错误码命中 ``collision`` 类     L3   slow_stop
``unknown_error_code``    错误码不在表内（保守）           L3   slow_stop
``protection_stop``       ``arm_current_status ∈ {9, 10}`` 且非本进程发起  L4  stop
=========================  ==============================  =====  ===================

多故障并发时按 **(等级降序, 故障类优先级)** 取主因；故障类优先级::

    protection_stop > stream_lost > collision > unknown_error_code
      > limit_exceeded > follow_error_heavy > follow_error_light
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Mapping, Optional, Tuple

from rm_safety import levels as _levels

__all__ = [
    "ARM_STATUS_PROTECTION_STATES",
    "Decision",
    "FAULT_KIND_COLLISION",
    "FAULT_KIND_FOLLOW_ERROR_HEAVY",
    "FAULT_KIND_FOLLOW_ERROR_LIGHT",
    "FAULT_KIND_LIMIT_EXCEEDED",
    "FAULT_KIND_PROTECTION_STOP",
    "FAULT_KIND_STREAM_LAG_LIGHT",
    "FAULT_KIND_STREAM_LOST",
    "FAULT_KIND_UNKNOWN_ERROR_CODE",
    "FAULT_KINDS",
    "KIND_PRIORITY",
    "SafetyThresholds",
    "can_auto_recover",
    "evaluate",
]

# ---------------------------------------------------------------------------
# 故障类常量
# ---------------------------------------------------------------------------

FAULT_KIND_FOLLOW_ERROR_LIGHT = "follow_error_light"
FAULT_KIND_FOLLOW_ERROR_HEAVY = "follow_error_heavy"
FAULT_KIND_LIMIT_EXCEEDED = "limit_exceeded"
FAULT_KIND_STREAM_LAG_LIGHT = "stream_lag_light"
FAULT_KIND_STREAM_LOST = "stream_lost"
FAULT_KIND_COLLISION = "collision"
FAULT_KIND_UNKNOWN_ERROR_CODE = "unknown_error_code"
FAULT_KIND_PROTECTION_STOP = "protection_stop"

#: 全部故障类（顺序 = 主因优先级从高到低）
FAULT_KINDS: Tuple[str, ...] = (
    FAULT_KIND_PROTECTION_STOP,
    FAULT_KIND_STREAM_LOST,
    FAULT_KIND_COLLISION,
    FAULT_KIND_UNKNOWN_ERROR_CODE,
    FAULT_KIND_LIMIT_EXCEEDED,
    FAULT_KIND_FOLLOW_ERROR_HEAVY,
    FAULT_KIND_STREAM_LAG_LIGHT,
    FAULT_KIND_FOLLOW_ERROR_LIGHT,
)

#: 故障类主因优先级（数值大 = 优先）
KIND_PRIORITY: Dict[str, int] = {
    FAULT_KIND_PROTECTION_STOP: 70,
    FAULT_KIND_STREAM_LOST: 60,
    FAULT_KIND_COLLISION: 50,
    FAULT_KIND_UNKNOWN_ERROR_CODE: 40,
    FAULT_KIND_LIMIT_EXCEEDED: 30,
    FAULT_KIND_FOLLOW_ERROR_HEAVY: 20,
    FAULT_KIND_STREAM_LAG_LIGHT: 10,
    FAULT_KIND_FOLLOW_ERROR_LIGHT: 5,
}

#: 控制器「已停机」状态码（RM_STOP_E=9 / RM_SLOW_STOP_E=10）
ARM_STATUS_PROTECTION_STATES: Tuple[int, ...] = (9, 10)


# ---------------------------------------------------------------------------
# 阈值与决策
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class SafetyThresholds:
    """安全判据阈值（**由调用方传入**，本模块不写死阈值）。

    四个阈值（``follow_warn_deg`` / ``follow_stop_deg`` / ``stream_warn_ms`` /
    ``stream_stop_ms``）必须由调用方从**唯一真值**传入（执行层持有
    ``rm_executor/contract.py`` 的冻结常量）；本 dataclass **不提供默认值**，
    以免出现第二处常量源。

    三个开关有默认值（它们是**策略开关**而非阈值）：
    ``l1_speed_scale`` / ``enable_l1_slowdown`` /
    ``enable_l2_pause_on_stream_lag``。
    """

    follow_warn_deg: float
    follow_stop_deg: float
    stream_warn_ms: float
    stream_stop_ms: float
    l1_speed_scale: float = 0.5
    enable_l1_slowdown: bool = True
    enable_l2_pause_on_stream_lag: bool = False

    def __post_init__(self) -> None:
        """校验阈值自洽（非法即抛错，不静默纠正）。"""
        for name in ("follow_warn_deg", "follow_stop_deg",
                     "stream_warn_ms", "stream_stop_ms"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError("%s 必须是数值（当前 %r）" % (name, value))
            if not math.isfinite(float(value)) or float(value) <= 0:
                raise ValueError("%s 必须是正有限数（当前 %r）" % (name, value))
        if float(self.follow_warn_deg) >= float(self.follow_stop_deg):
            raise ValueError(
                "follow_warn_deg 必须小于 follow_stop_deg（当前 %r ≥ %r）"
                % (self.follow_warn_deg, self.follow_stop_deg))
        if float(self.stream_warn_ms) >= float(self.stream_stop_ms):
            raise ValueError(
                "stream_warn_ms 必须小于 stream_stop_ms（当前 %r ≥ %r）"
                % (self.stream_warn_ms, self.stream_stop_ms))
        if not (0.0 < float(self.l1_speed_scale) <= 1.0):
            raise ValueError(
                "l1_speed_scale 必须落在 (0, 1]（当前 %r）" % (self.l1_speed_scale,))


@dataclass(frozen=True)
class Decision:
    """一次安全判定的结果（**纯数据**）。

    字段
    ----
    fault_kind
        主因故障类；空串表示**无故障命中**。
    level
        安全等级 ``L1``–``L4``；空串表示无故障。
    severity
        严重度（与等级表一致）。
    exit_code
        进程退出码（L1–L3 → 3，L4 → 4；无故障 → 0）。
    reason
        人可读的原因。
    evidence
        证据元组；每项是 dict，至少含 ``fault_kind`` / ``level`` /
        ``threshold`` / ``observed`` / ``margin`` 五个可核对字段。
    next_action
        **可粘贴**的下一步。
    detected
        本次命中的**全部**故障类（按主因优先级排序）。
    action
        等级对应动作名（无故障时为空串）。
    """

    fault_kind: str = ""
    level: str = ""
    severity: str = ""
    exit_code: int = 0
    reason: str = ""
    evidence: Tuple[Dict[str, Any], ...] = field(default_factory=tuple)
    next_action: str = ""
    detected: Tuple[str, ...] = field(default_factory=tuple)
    action: str = ""


# ---------------------------------------------------------------------------
# 下一步（可粘贴）
# ---------------------------------------------------------------------------
def _next_action(level: str, thresholds: Optional[SafetyThresholds] = None,
                 fault_kind: str = "") -> str:
    """按等级给出**可粘贴**的下一步。"""
    warn = thresholds.follow_warn_deg if thresholds else "告警阈值"
    if level == _levels.LEVEL_L1:
        return (
            "确认跟随误差已回落到 %s° 以内，安全层会自动恢复原速并记录 %s；"
            "若误差持续，请 `rmctl safety status` 查看当前等级后人工处置"
            % (warn, _levels.EVENT_L1_SLOWDOWN_CLEARED)
        )
    if level == _levels.LEVEL_L2:
        return (
            "人工确认现场安全后手动下发 rm_set_arm_continue 恢复运行"
            "（安全层不自动 continue_）；恢复前请查看证据里的阈值与实测值"
        )
    if level == _levels.LEVEL_L3:
        return (
            "人工读取控制器错误码并复位后手动恢复运行"
            "（安全层不自动清错误码）；未知错误码按保守档 L3 处理，"
            "请把证据里的 code 原样贴给操作者"
        )
    if level == _levels.LEVEL_L4:
        return (
            "立即按下硬件急停并人工复位控制器后恢复运行"
            "（安全层不自动复位）；请把本次 evidence 原样贴给操作者"
        )
    return "无故障命中：继续按原计划运行"


# ---------------------------------------------------------------------------
# 内部工具
# ---------------------------------------------------------------------------
def _num(value: Any) -> Optional[float]:
    """转成有限浮点数；``None`` / 非数值 / NaN / inf → ``None``。"""
    if value is None or isinstance(value, bool):
        return None
    if not isinstance(value, (int, float)):
        return None
    number = float(value)
    if not math.isfinite(number):
        return None
    return number


def _ev(fault_kind: str, level: str, threshold: Any, observed: Any,
        margin: Any, **extra: Any) -> Dict[str, Any]:
    """构造一条证据（字段固定，便于离线逐条核对）。"""
    item: Dict[str, Any] = {
        "fault_kind": fault_kind,
        "level": level,
        "threshold": threshold,
        "observed": observed,
        "margin": margin,
    }
    item.update(extra)
    return item


def _violation_margin(violations: Any) -> Optional[float]:
    """从限值越界条目里取最小 margin（若条目带 margin 字段）。"""
    if not isinstance(violations, (list, tuple)):
        return None
    margins: List[float] = []
    for item in violations:
        if isinstance(item, Mapping):
            for key in ("margin", "margin_to_limit", "overrun"):
                value = _num(item.get(key))
                if value is not None:
                    margins.append(abs(value))
                    break
    return min(margins) if margins else None


# ---------------------------------------------------------------------------
# 判据
# ---------------------------------------------------------------------------
def evaluate(signals: Mapping[str, Any], thresholds: SafetyThresholds) -> Decision:
    """评估一组故障信号 → 安全决策（**纯函数**）。

    参数
    ----
    signals
        故障信号字典，键（全部可选，缺失视为「无该信号」）：

        * ``follow_error_deg``（float|None）：跟随误差（度）；
        * ``stream_age_ms``（float|None）：反馈断流时长（毫秒）；
        * ``limit_violations``（list[dict]）：运行期限值越界条目；
        * ``arm_err``（list[int]）：控制器错误码列表；
        * ``arm_current_status``（int|None）：控制器当前状态码；
        * ``error_code_map``（ErrorCodeMap|None）：错误码映射表；
        * ``self_triggered_stop``（bool）：停机是否由本进程发起（保护停判据用）。

    thresholds
        阈值（由调用方从唯一真值传入）。

    返回
    ----
    Decision
        无故障时 ``level=""`` / ``exit_code=0``；有故障时给出主因等级、
        严重度、退出码、证据与可粘贴的下一步。
    """
    if thresholds is None:
        raise ValueError("thresholds 不能为空（阈值必须由调用方从唯一真值传入）")
    if not isinstance(signals, Mapping):
        raise TypeError("signals 必须是映射（当前是 %s）"
                        % type(signals).__name__)

    detected: List[str] = []
    evidences: List[Dict[str, Any]] = []
    levels_by_kind: Dict[str, str] = {}

    # ---------------------------------------------------------- 保护停（L4）
    status = signals.get("arm_current_status")
    status_num = _num(status)
    self_triggered = bool(signals.get("self_triggered_stop", False))
    if status_num is not None and int(status_num) in ARM_STATUS_PROTECTION_STATES:
        if self_triggered:
            evidences.append(_ev(
                "advisory", "",
                "arm_current_status ∈ {9, 10}",
                int(status_num),
                None,
                note="控制器处于停机状态，但该停机由本进程发起 → 不判保护停",
            ))
        else:
            detected.append(FAULT_KIND_PROTECTION_STOP)
            levels_by_kind[FAULT_KIND_PROTECTION_STOP] = _levels.LEVEL_L4
            evidences.append(_ev(
                FAULT_KIND_PROTECTION_STOP, _levels.LEVEL_L4,
                "arm_current_status ∈ {9, 10} 且非本进程发起",
                int(status_num),
                None,
                note="控制器保护停（非本进程发起）",
            ))

    # ------------------------------------------------------------ 断流（L4/L2）
    age = _num(signals.get("stream_age_ms"))
    if age is not None:
        if age > float(thresholds.stream_stop_ms):
            detected.append(FAULT_KIND_STREAM_LOST)
            levels_by_kind[FAULT_KIND_STREAM_LOST] = _levels.LEVEL_L4
            evidences.append(_ev(
                FAULT_KIND_STREAM_LOST, _levels.LEVEL_L4,
                float(thresholds.stream_stop_ms),
                age,
                age - float(thresholds.stream_stop_ms),
                unit="ms",
            ))
        elif age > float(thresholds.stream_warn_ms):
            if thresholds.enable_l2_pause_on_stream_lag:
                detected.append(FAULT_KIND_STREAM_LAG_LIGHT)
                levels_by_kind[FAULT_KIND_STREAM_LAG_LIGHT] = _levels.LEVEL_L2
                evidences.append(_ev(
                    FAULT_KIND_STREAM_LAG_LIGHT, _levels.LEVEL_L2,
                    float(thresholds.stream_warn_ms),
                    age,
                    float(thresholds.stream_stop_ms) - age,
                    unit="ms",
                ))
            else:
                evidences.append(_ev(
                    "advisory", "",
                    float(thresholds.stream_warn_ms),
                    age,
                    float(thresholds.stream_stop_ms) - age,
                    unit="ms",
                    note="断流处于告警区间，但 enable_l2_pause_on_stream_lag=false "
                         "→ 只告警不动作（保持 S03 口径）",
                ))

    # ------------------------------------------------------- 跟随误差（L1/L2）
    follow = _num(signals.get("follow_error_deg"))
    if follow is not None and follow > float(thresholds.follow_warn_deg):
        if follow > float(thresholds.follow_stop_deg):
            detected.append(FAULT_KIND_FOLLOW_ERROR_HEAVY)
            levels_by_kind[FAULT_KIND_FOLLOW_ERROR_HEAVY] = _levels.LEVEL_L2
            evidences.append(_ev(
                FAULT_KIND_FOLLOW_ERROR_HEAVY, _levels.LEVEL_L2,
                float(thresholds.follow_stop_deg),
                follow,
                follow - float(thresholds.follow_stop_deg),
                unit="deg",
            ))
        else:
            if thresholds.enable_l1_slowdown:
                level = _levels.LEVEL_L1
            else:
                # L1 降速被关闭 → 保守升到 L2（不做没有动作的降速）
                level = _levels.LEVEL_L2
            detected.append(FAULT_KIND_FOLLOW_ERROR_LIGHT)
            levels_by_kind[FAULT_KIND_FOLLOW_ERROR_LIGHT] = level
            evidences.append(_ev(
                FAULT_KIND_FOLLOW_ERROR_LIGHT, level,
                float(thresholds.follow_warn_deg),
                follow,
                float(thresholds.follow_stop_deg) - follow,
                unit="deg",
                note=("L1 降速" if thresholds.enable_l1_slowdown
                      else "enable_l1_slowdown=false → L1 降速不可用，保守升到 L2 暂停"),
            ))

    # ------------------------------------------------------------- 超限（L2）
    violations = signals.get("limit_violations")
    if isinstance(violations, (list, tuple)) and len(violations) > 0:
        detected.append(FAULT_KIND_LIMIT_EXCEEDED)
        levels_by_kind[FAULT_KIND_LIMIT_EXCEEDED] = _levels.LEVEL_L2
        evidences.append(_ev(
            FAULT_KIND_LIMIT_EXCEEDED, _levels.LEVEL_L2,
            "声明限位（运行期复检）",
            len(violations),
            _violation_margin(violations),
            violations=[v for v in violations][:20],
        ))

    # ----------------------------------------------- 控制器错误码（L3 保守）
    raw_err = signals.get("arm_err")
    err_codes: List[int] = []
    if isinstance(raw_err, (list, tuple)):
        for item in raw_err:
            value = _num(item)
            if value is not None:
                err_codes.append(int(value))
    # **码 0 = 无错误**（厂商 ``rm_current_arm_state_t.err[]`` 约定）：0 不是故障码，
    # 必须排除在分类之外 —— 否则会把"无错误"当成"未知码"并保守升级到 L3 缓停
    # （2026-11-19 真机实测：真机常态上报 arm_err=[0]，执行层因此被误判为
    # unknown_error_code → L3 缓停，整条轨迹只跑了 2 拍就被停掉）。
    err_codes = [code for code in err_codes if code != 0]
    error_code_map = signals.get("error_code_map")
    if err_codes:
        collision_codes: List[int] = []
        unknown_codes: List[int] = []
        over_limit_codes: List[int] = []
        protection_codes: List[int] = []
        other: List[Dict[str, Any]] = []
        for code in err_codes:
            if error_code_map is not None and error_code_map.known(code):
                code_class = error_code_map.classify(code)
            else:
                code_class = "unknown"
            if code_class == "collision":
                collision_codes.append(code)
            elif code_class == "unknown":
                unknown_codes.append(code)
            elif code_class == "over_limit":
                over_limit_codes.append(code)
            elif code_class == "protection":
                protection_codes.append(code)
            else:
                other.append({"code": code, "class": code_class})

        if collision_codes:
            detected.append(FAULT_KIND_COLLISION)
            levels_by_kind[FAULT_KIND_COLLISION] = _levels.LEVEL_L3
            evidences.append(_ev(
                FAULT_KIND_COLLISION, _levels.LEVEL_L3,
                "error_code_map 中 class=collision",
                collision_codes,
                None,
            ))
        if unknown_codes:
            detected.append(FAULT_KIND_UNKNOWN_ERROR_CODE)
            levels_by_kind[FAULT_KIND_UNKNOWN_ERROR_CODE] = _levels.LEVEL_L3
            evidences.append(_ev(
                FAULT_KIND_UNKNOWN_ERROR_CODE, _levels.LEVEL_L3,
                "error_code_map 覆盖范围",
                unknown_codes,
                None,
                note="未知错误码，按保守档 L3 缓停处理（不自动清错误码）",
            ))
        if over_limit_codes and FAULT_KIND_LIMIT_EXCEEDED not in levels_by_kind:
            detected.append(FAULT_KIND_LIMIT_EXCEEDED)
            levels_by_kind[FAULT_KIND_LIMIT_EXCEEDED] = _levels.LEVEL_L2
            evidences.append(_ev(
                FAULT_KIND_LIMIT_EXCEEDED, _levels.LEVEL_L2,
                "error_code_map 中 class=over_limit",
                over_limit_codes,
                None,
            ))
        if protection_codes and FAULT_KIND_PROTECTION_STOP not in levels_by_kind:
            detected.append(FAULT_KIND_PROTECTION_STOP)
            levels_by_kind[FAULT_KIND_PROTECTION_STOP] = _levels.LEVEL_L4
            evidences.append(_ev(
                FAULT_KIND_PROTECTION_STOP, _levels.LEVEL_L4,
                "error_code_map 中 class=protection",
                protection_codes,
                None,
            ))
        if other:
            evidences.append(_ev(
                "advisory", "",
                "error_code_map 覆盖范围",
                other,
                None,
                note="已知但非安全故障类的错误码：仅记录证据，不触发安全动作",
            ))

    # ------------------------------------------------------------ 主因选择
    if not detected:
        return Decision(
            fault_kind="",
            level="",
            severity="",
            exit_code=0,
            reason="无故障命中（跟随误差/断流/限值/错误码/控制器状态均在阈值内）",
            evidence=tuple(evidences),
            next_action=_next_action(""),
            detected=(),
            action="",
        )

    ordered = sorted(
        set(detected),
        key=lambda kind: (_levels.level_rank(levels_by_kind[kind]),
                          KIND_PRIORITY.get(kind, 0)),
        reverse=True,
    )
    primary = ordered[0]
    level = levels_by_kind[primary]
    severity = _levels.level_severity(level)
    exit_code = _levels.level_exit_code(level)
    action = _levels.level_action(level)
    reason = "%s → %s（%s）：动作=%s，恢复=%s" % (
        primary, level, _levels.LEVELS[level]["title"], action,
        _levels.level_recovery(level))
    return Decision(
        fault_kind=primary,
        level=level,
        severity=severity,
        exit_code=exit_code,
        reason=reason,
        evidence=tuple(evidences),
        next_action=_next_action(level, thresholds, primary),
        detected=tuple(ordered),
        action=action,
    )


def can_auto_recover(level: str, follow_error_deg: Any,
                     thresholds: SafetyThresholds) -> bool:
    """判断是否允许**自动恢复**（仅 L1，且跟随误差已回落到告警阈值内）。

    L2 / L3 / L4 一律返回 ``False``（不自动 continue_ / 不自动清错误码 /
    不自动复位）。
    """
    if level != _levels.LEVEL_L1:
        return False
    value = _num(follow_error_deg)
    if value is None:
        return False
    return value <= float(thresholds.follow_warn_deg)
