"""设备层（S03）：真机连接、配置与传输。

分层
----
* :mod:`rm75_drake.device.config` —— **部署配置**真相源（``devices/*.yaml``）：
  IP / 端口 / UDP 周期 / 零位偏移 / 加速度上限 / 电流阈值 / 围栏；
  校验纪律沿用 ``scenario.py``（未知键 / 缺字段 / 越界 → 点名报错，禁止静默默认值）。
* :mod:`rm75_drake.device.sdk` —— 厂商 SDK 的**懒加载**封装与
  **rad↔deg 唯一转换点**；SDK 缺失时抛显式 ``NotAvailableError``。
* :mod:`rm75_drake.device.transport` —— **传输层边界**：
  :class:`~rm75_drake.device.transport.ArmTransport` 协议 +
  真机实现 :class:`~rm75_drake.device.transport.SdkTransport` +
  离线替身 :class:`~rm75_drake.device.transport.ScriptedTransport`。
* :mod:`rm75_drake.device.safety` —— **安全七闸**（纯函数）：
  输入 = 上一拍指令 + 本拍状态，输出 = 钳位后指令 + 违规列表。
* :mod:`rm75_drake.device.state_source` —— **UDP 状态源**：
  无锁环形缓冲（最近 N 帧）+ 帧完整性校验 + 状态 age。
* :mod:`rm75_drake.device.state_machine` —— **设备状态机**：
  纯转移表 + 守卫条件（9 状态 / 15 事件），非法转移显式报错。
* :mod:`rm75_drake.device.metrics` —— **抖动 / 延迟指标**：
  预分配直方图 + P50 / P99 / P99.9 分位数（唯一口径，单测与真机脚本共用）。
* :mod:`rm75_drake.device.exec_loop` —— **5 ms 实时执行环**：
  固定节拍 + 七闸钳位 + 唯一运动通道；时钟 / 睡眠可注入（确定性虚拟时钟）。

本包**零 pydrake 依赖**（只依赖 numpy + 本项目的轨迹/路径层），
因此安全七闸 / 状态机 / 执行环 / 恢复流程都能在无 SDK、无真机的环境里被测试。
"""

from __future__ import annotations

from .config import (
    ACCEL_LIMIT_KEYS,
    CONSERVATIVE_CURRENT_THRESHOLDS_MA,
    CURRENT_THRESHOLD_KEYS,
    FENCE_KEYS,
    SCHEMA_VERSION,
    TOP_LEVEL_KEYS,
    DeviceConfig,
    DeviceConfigError,
    default_device_config_path,
    device_config_from_dict,
    load_device_config,
)
from .sdk import (
    CONVERSION_FUNCTION_NAMES,
    SDK_CALL_WHITELIST,
    SDK_DIST_NAME,
    SDK_IMPORT_NAME,
    SDK_UNAVAILABLE_REASON,
    NotAvailableError,
    SdkAudit,
    SdkCapabilities,
    SdkProxy,
    assert_single_conversion_point,
    deg_to_rad,
    load_rm_interface,
    load_sdk,
    probe_capabilities,
    rad_to_deg,
    sdk_available,
)
from .exec_loop import (
    PERIOD_S,
    REASON_COMPLETED,
    REASON_SAFE_STOP,
    REASON_TRANSPORT_ERROR,
    Clock,
    ExecLoop,
    ExecLoopConfig,
    ExecLoopResult,
    RtSchedulingReport,
    SystemClock,
    VirtualClock,
    apply_realtime_scheduling,
)
from .metrics import (
    DEFAULT_MAX_SAMPLES,
    PERCENTILE_LABELS,
    JitterHistogram,
    LatencyHistogram,
    MetricHistogram,
    linear_percentile,
)
from .recovery import (
    FAULT_REASONS,
    RECOVERY_STEPS,
    RELAX_MARGIN_RAD,
    SAFE_STATE_BY_REASON,
    STEP_FAILED,
    STEP_OK,
    STEP_SKIPPED,
    FaultDiagnosis,
    RecoveryError,
    RecoveryProcedure,
    RecoveryResult,
    RecoveryStep,
    classify_fault,
    enter_safe_state,
    recover,
)
from .safety import (
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
from .state_source import (
    DEFAULT_CAPACITY,
    MISSED_CYCLES_UNBOUNDED,
    REASON_FRAME_INCOMPLETE,
    REASON_INVALID_FRAME,
    FrameRing,
    StateSample,
    StateSource,
    StateSourceError,
)
from .state_machine import (
    ALL_EVENTS,
    ALL_STATES,
    REASON_ESTOP_STATUS,
    REASON_GUARD_REJECTED,
    REASON_ILLEGAL_TRANSITION,
    REASON_JOINT_ERROR,
    REASON_SYSTEM_ERROR,
    DeviceEvent,
    DeviceState,
    DeviceStateMachine,
    GuardRejectedError,
    IllegalTransitionError,
    StateMachineError,
    Transition,
    detect_fault,
    transition_table,
)
from .transport import (
    REASON_CLOSED,
    REASON_GRIPPER_OFFLINE,
    REASON_INVALID_INPUT,
    REASON_LINK_DOWN,
    REASON_NOT_OPEN,
    REASON_SDK_UNAVAILABLE,
    ArmTransport,
    GripperState,
    ScriptedTransport,
    SdkTransport,
    StateFrame,
    TransportError,
)

__all__ = [
    # config
    "DeviceConfig",
    "DeviceConfigError",
    "load_device_config",
    "device_config_from_dict",
    "default_device_config_path",
    "SCHEMA_VERSION",
    "TOP_LEVEL_KEYS",
    "ACCEL_LIMIT_KEYS",
    "CURRENT_THRESHOLD_KEYS",
    "FENCE_KEYS",
    "CONSERVATIVE_CURRENT_THRESHOLDS_MA",
    # sdk
    "NotAvailableError",
    "SDK_IMPORT_NAME",
    "SDK_DIST_NAME",
    "SDK_UNAVAILABLE_REASON",
    "SDK_CALL_WHITELIST",
    "CONVERSION_FUNCTION_NAMES",
    "deg_to_rad",
    "rad_to_deg",
    "sdk_available",
    "load_sdk",
    "load_rm_interface",
    "SdkCapabilities",
    "probe_capabilities",
    "SdkAudit",
    "SdkProxy",
    "assert_single_conversion_point",
    # transport
    "ArmTransport",
    "SdkTransport",
    "ScriptedTransport",
    "StateFrame",
    "GripperState",
    "TransportError",
    "REASON_NOT_OPEN",
    "REASON_CLOSED",
    "REASON_SDK_UNAVAILABLE",
    "REASON_LINK_DOWN",
    "REASON_INVALID_INPUT",
    "REASON_GRIPPER_OFFLINE",
    # state_source
    "StateSource",
    "StateSourceError",
    "StateSample",
    "FrameRing",
    "DEFAULT_CAPACITY",
    "MISSED_CYCLES_UNBOUNDED",
    "REASON_FRAME_INCOMPLETE",
    "REASON_INVALID_FRAME",
    # state_machine
    "DeviceStateMachine",
    "DeviceState",
    "DeviceEvent",
    "ALL_STATES",
    "ALL_EVENTS",
    "Transition",
    "StateMachineError",
    "IllegalTransitionError",
    "GuardRejectedError",
    "detect_fault",
    "transition_table",
    "REASON_ILLEGAL_TRANSITION",
    "REASON_GUARD_REJECTED",
    "REASON_ESTOP_STATUS",
    "REASON_JOINT_ERROR",
    "REASON_SYSTEM_ERROR",
    # safety
    "SafetyLimits",
    "SafetyState",
    "SafetyViolation",
    "SafetyResult",
    "apply_safety",
    "gate_soft_limit",
    "gate_delta_clamp",
    "gate_accel_clamp",
    "gate_watchdog",
    "gate_current",
    "gate_fence",
    "gate_state_age",
    "safety_limits_from_sources",
    "load_safety_limits",
    "REASON_SOFT_LIMIT",
    "REASON_DELTA_CLAMP",
    "REASON_ACCEL_CLAMP",
    "REASON_WATCHDOG",
    "REASON_CURRENT",
    "REASON_FENCE",
    "REASON_STATE_AGE",
    "ALL_REASONS",
    "ACTION_PROCEED",
    "ACTION_CLAMP",
    "ACTION_SLOW_STOP",
    "ACTION_STOP",
    # metrics
    "MetricHistogram",
    "JitterHistogram",
    "LatencyHistogram",
    "linear_percentile",
    "PERCENTILE_LABELS",
    "DEFAULT_MAX_SAMPLES",
    # exec_loop
    "ExecLoop",
    "ExecLoopConfig",
    "ExecLoopResult",
    "RtSchedulingReport",
    "Clock",
    "SystemClock",
    "VirtualClock",
    "apply_realtime_scheduling",
    "PERIOD_S",
    "REASON_COMPLETED",
    "REASON_SAFE_STOP",
    "REASON_TRANSPORT_ERROR",
    # recovery
    "RecoveryProcedure",
    "RecoveryResult",
    "RecoveryStep",
    "RecoveryError",
    "FaultDiagnosis",
    "classify_fault",
    "enter_safe_state",
    "recover",
    "RECOVERY_STEPS",
    "RELAX_MARGIN_RAD",
    "FAULT_REASONS",
    "SAFE_STATE_BY_REASON",
    "STEP_OK",
    "STEP_FAILED",
    "STEP_SKIPPED",
]
