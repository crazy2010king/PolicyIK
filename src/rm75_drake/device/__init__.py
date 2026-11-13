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
]
