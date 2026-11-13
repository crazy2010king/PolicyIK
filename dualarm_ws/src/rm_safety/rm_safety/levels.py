"""失效安全停机**四级等级表**（**唯一真值**）。

本模块冻结 L1–L4 四级处置的**全部语义**：动作、SDK 符号、严重度、退出码、
恢复方式与事件名。下游（动作执行器 / 执行器接线 / 注入用例 / 安全闸门 /
文档 / 验收）**一律以本表为准**，不得在别处再写一份等级表。

=====================  ==========  ==============================  ==========  ==========  =========
等级                  动作        SDK 符号                         严重度     退出码     恢复方式
=====================  ==========  ==============================  ==========  ==========  =========
L1 降速               降速       ``rm_set_joint_drive_max_speed``  RECOVERABLE  3        auto
L2 暂停               暂停       ``rm_set_arm_pause``             NEEDS_HUMAN   3        manual
L3 缓停               缓停       ``rm_set_arm_slow_stop``         NEEDS_HUMAN   3        manual
L4 急停               急停       ``rm_set_arm_stop``              FATAL        4        manual_reset
=====================  ==========  ==============================  ==========  ==========  =========

恢复方式语义
------------

* ``auto``：**仅 L1**。跟随误差回落到 ``follow_warn_deg`` 内后自动恢复原速，
  并必须记一条 ``safety_l1_slowdown_cleared`` 事件；
* ``manual``：L2 / L3 —— 人工确认后由人恢复运行（安全层**不自动**
  ``continue_``、不自动清错误码）；
* ``manual_reset``：L4 —— 需人工按硬件急停并复位控制器后恢复。

本模块是**纯数据 + 纯函数**（无 I/O、无 SDK、无线程、无第三方依赖），
因此可被离线单测逐条钉死。
"""
from __future__ import annotations

from typing import Dict, Tuple

__all__ = [
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
    "LEVELS",
    "LEVEL_ORDER",
    "LEVEL_L1",
    "LEVEL_L2",
    "LEVEL_L3",
    "LEVEL_L4",
    "RECOVERY_AUTO",
    "RECOVERY_MANUAL",
    "RECOVERY_MANUAL_RESET",
    "SEVERITY_FATAL",
    "SEVERITY_NEEDS_HUMAN",
    "SEVERITY_RECOVERABLE",
    "level_action",
    "level_event_type",
    "level_exit_code",
    "level_rank",
    "level_recovery",
    "level_sdk_symbol",
    "level_severity",
]

# ---------------------------------------------------------------------------
# 等级标识
# ---------------------------------------------------------------------------

LEVEL_L1 = "L1"
LEVEL_L2 = "L2"
LEVEL_L3 = "L3"
LEVEL_L4 = "L4"

#: 等级顺序（由轻到重）
LEVEL_ORDER: Tuple[str, ...] = (LEVEL_L1, LEVEL_L2, LEVEL_L3, LEVEL_L4)

#: 严重度口径（与 rm_executor.events 的五级事件模型同口径，本层不另立一套）
SEVERITY_RECOVERABLE = "RECOVERABLE"
SEVERITY_NEEDS_HUMAN = "NEEDS_HUMAN"
SEVERITY_FATAL = "FATAL"

#: 恢复方式
RECOVERY_AUTO = "auto"
RECOVERY_MANUAL = "manual"
RECOVERY_MANUAL_RESET = "manual_reset"

# ---------------------------------------------------------------------------
# 事件名（冻结）
# ---------------------------------------------------------------------------

#: L1 降速触发
EVENT_L1_SLOWDOWN = "safety_l1_slowdown"
#: L1 降速恢复（跟随误差回落到告警阈值内）
EVENT_L1_SLOWDOWN_CLEARED = "safety_l1_slowdown_cleared"
#: L2 暂停
EVENT_L2_PAUSE = "safety_l2_pause"
#: L3 缓停
EVENT_L3_SLOW_STOP = "safety_l3_slow_stop"
#: L4 急停
EVENT_L4_EMERGENCY_STOP = "safety_l4_emergency_stop"

#: 故障类事件名（跟随误差 / 超限 / 断流 / 碰撞 / 保护停）
FAULT_FOLLOW_ERROR = "fault_follow_error"
FAULT_LIMIT_EXCEEDED = "fault_limit_exceeded"
FAULT_STREAM_LOST = "fault_stream_lost"
FAULT_COLLISION = "fault_collision"
FAULT_PROTECTION_STOP = "fault_protection_stop"


# ---------------------------------------------------------------------------
# 四级表（唯一真值）
# ---------------------------------------------------------------------------

#: 等级 → 处置定义
LEVELS: Dict[str, Dict[str, object]] = {
    LEVEL_L1: {
        "rank": 1,
        "title": "降速",
        "action": "set_speed_scale",
        "sdk_symbol": "rm_set_joint_drive_max_speed",
        "severity": SEVERITY_RECOVERABLE,
        "exit_code": 3,
        "recovery": RECOVERY_AUTO,
        "event_type": EVENT_L1_SLOWDOWN,
        "recovery_event_type": EVENT_L1_SLOWDOWN_CLEARED,
        "note": "按比例下调关节驱动最大速度；跟随误差回落到告警阈值内自动恢复原速",
    },
    LEVEL_L2: {
        "rank": 2,
        "title": "暂停",
        "action": "pause",
        "sdk_symbol": "rm_set_arm_pause",
        "severity": SEVERITY_NEEDS_HUMAN,
        "exit_code": 3,
        "recovery": RECOVERY_MANUAL,
        "event_type": EVENT_L2_PAUSE,
        "recovery_event_type": "",
        "note": "硬边界越界 / 重跟随误差时暂停；不自动 continue_，等人工确认",
    },
    LEVEL_L3: {
        "rank": 3,
        "title": "缓停",
        "action": "slow_stop",
        "sdk_symbol": "rm_set_arm_slow_stop",
        "severity": SEVERITY_NEEDS_HUMAN,
        "exit_code": 3,
        "recovery": RECOVERY_MANUAL,
        "event_type": EVENT_L3_SLOW_STOP,
        "recovery_event_type": "",
        "note": "碰撞 / 未知错误码时缓停；不自动清错误码，等人工复位",
    },
    LEVEL_L4: {
        "rank": 4,
        "title": "急停",
        "action": "stop",
        "sdk_symbol": "rm_set_arm_stop",
        "severity": SEVERITY_FATAL,
        "exit_code": 4,
        "recovery": RECOVERY_MANUAL_RESET,
        "event_type": EVENT_L4_EMERGENCY_STOP,
        "recovery_event_type": "",
        "note": "断流失联 / 控制器保护停时急停；提示硬件急停 + 人工复位",
    },
}


def _require(level: str) -> Dict[str, object]:
    """取出等级定义；未知等级抛 :class:`ValueError`（不静默兜底）。"""
    try:
        return LEVELS[level]
    except (KeyError, TypeError):
        raise ValueError(
            "未知安全等级 %r（只允许 %s）" % (level, ", ".join(LEVEL_ORDER))
        ) from None


def level_rank(level: str) -> int:
    """等级序号（L1→1 … L4→4）；未知等级抛 :class:`ValueError`。"""
    return int(_require(level)["rank"])


def level_event_type(level: str) -> str:
    """等级对应的事件名。"""
    return str(_require(level)["event_type"])


def level_action(level: str) -> str:
    """等级对应的动作名（``set_speed_scale`` / ``pause`` / ``slow_stop`` / ``stop``）。"""
    return str(_require(level)["action"])


def level_sdk_symbol(level: str) -> str:
    """等级对应的 RM-API2 SDK 符号（白名单内）。"""
    return str(_require(level)["sdk_symbol"])


def level_severity(level: str) -> str:
    """等级对应的严重度。"""
    return str(_require(level)["severity"])


def level_exit_code(level: str) -> int:
    """等级对应的进程退出码（L1–L3 → 3，L4 → 4）。"""
    return int(_require(level)["exit_code"])


def level_recovery(level: str) -> str:
    """等级对应的恢复方式（``auto`` / ``manual`` / ``manual_reset``）。"""
    return str(_require(level)["recovery"])
