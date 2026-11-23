"""rm_safety —— 失效安全停机层（四级等级表 + 判据纯函数 + 声明式错误码表）。

层定位
------

依赖方向（**不得反向**）::

    hardware(rm_hw_bridge)  <  safety(rm_safety)  <  executor(rm_executor)  <  experience(rmctl)

本包**只依赖标准库 + pyyaml**，**不 import** ``rmctl`` / ``rm_executor`` /
``rm_hw_bridge`` / ``tesseract*`` —— 这条红线由静态审计（T8 注册到
``rm_hw_bridge/contract.yaml`` 的 ``rules.layers``）锁进代码层。

四个模块的职责
--------------

* :mod:`rm_safety.levels` —— **四级等级表（唯一真值）**：动作 / SDK 符号 /
  严重度 / 退出码 / 恢复方式 / 事件名；
* :mod:`rm_safety.policy` —— **判据纯函数**（无 I/O / 无 SDK / 无线程）：
  故障类 → 等级 → 动作 → 恢复方式；
* :mod:`rm_safety.error_codes` —— **声明式错误码表**读取（未知码 → 保守 L3）；
* :mod:`rm_safety.config` —— 安全配置读取（唯一来源 =
  ``workcell/rm_workcell.yaml`` 的 ``safety:`` 段）；
* :mod:`rm_safety.paths` —— 路径解析（**独立实现**，不反向依赖任何上层）。

执行侧的两个模块
----------------

* :mod:`rm_safety.actuator` —— **安全动作执行器**（唯一动作权威的执行侧：
  等级 → 动作 → 调用传输端口；失败留证据并升级到 L4）；
* :mod:`rm_safety.supervisor` —— **安全监督器**（判据 → 动作 → 事件；
  等级单调升级，**仅 L1** 允许自动恢复）。

两个模块同样**零反向依赖**：传输端口由调用方注入（鸭子类型）。

安全闸门
--------

* :mod:`rm_safety.gate` —— **进程外强制停机通道**（Unix domain socket）：
  ``InProcessGate``（进程内共享状态）/ ``UnixSocketGate``（socket 服务端）/
  ``GateServer``（生命周期门面）。闸门**只做停机请求**，不做配置下发。
"""
from __future__ import annotations

from rm_safety.actuator import ActionResult, SafetyActuator
from rm_safety.config import load_safety_config
from rm_safety.error_codes import ErrorCodeMap, load_error_codes
from rm_safety.gate import (
    GateError,
    GateServer,
    InProcessGate,
    UnixSocketGate,
    default_socket_path,
)
from rm_safety.levels import (
    EVENT_L1_SLOWDOWN,
    EVENT_L1_SLOWDOWN_CLEARED,
    EVENT_L2_PAUSE,
    EVENT_L3_SLOW_STOP,
    EVENT_L4_EMERGENCY_STOP,
    FAULT_COLLISION,
    FAULT_FOLLOW_ERROR,
    FAULT_LIMIT_EXCEEDED,
    FAULT_PROTECTION_STOP,
    FAULT_STREAM_LOST,
    LEVELS,
    LEVEL_ORDER,
    LEVEL_L1,
    LEVEL_L2,
    LEVEL_L3,
    LEVEL_L4,
    level_action,
    level_event_type,
    level_exit_code,
    level_rank,
    level_recovery,
    level_sdk_symbol,
    level_severity,
)
from rm_safety.policy import (
    Decision,
    SafetyThresholds,
    can_auto_recover,
    evaluate,
)
from rm_safety.supervisor import SafetySupervisor

__all__ = [
    "ActionResult",
    "Decision",
    "ErrorCodeMap",
    "EVENT_L1_SLOWDOWN",
    "EVENT_L1_SLOWDOWN_CLEARED",
    "EVENT_L2_PAUSE",
    "EVENT_L3_SLOW_STOP",
    "EVENT_L4_EMERGENCY_STOP",
    "FAULT_COLLISION",
    "FAULT_FOLLOW_ERROR",
    "FAULT_LIMIT_EXCEEDED",
    "FAULT_PROTECTION_STOP",
    "FAULT_STREAM_LOST",
    "GateError",
    "GateServer",
    "InProcessGate",
    "UnixSocketGate",
    "LEVELS",
    "LEVEL_ORDER",
    "LEVEL_L1",
    "LEVEL_L2",
    "LEVEL_L3",
    "LEVEL_L4",
    "SafetyActuator",
    "SafetySupervisor",
    "SafetyThresholds",
    "can_auto_recover",
    "evaluate",
    "level_action",
    "level_event_type",
    "level_exit_code",
    "level_rank",
    "level_recovery",
    "level_sdk_symbol",
    "level_severity",
    "load_error_codes",
    "load_safety_config",
    "default_socket_path",
]

__version__ = "0.1.0"
