"""夹爪**纯域逻辑**：归一化状态、错误位解码、六类判定、恢复阶梯与运动闸门。

本模块是 S05「夹爪接入与状态判定」的**离线可证伪载体**：

* **零 SDK、零 I/O**：不 import ``ctypes`` / ``rm_hw_bridge.generated`` / ``rm_hw_bridge.loader``，
  不碰任何寄存器；所有输入都是纯 Python 数据结构；
* **路径无关的归一化**：三条协议路径（``modbus`` / ``gripper_api`` / ``rm_plus``）的原始值
  先统一成一个 :class:`GripperState`，再由 :func:`judge` 统一判定；
* **两条路径的模式码不等价**：手册状态码（0x0014，1–5）与 SDK ``rm_gripper_state_t.mode``
  （1–6，多一个「闭合中力控停止」）**各自写死映射表**，不合并成一个表。

一手来源（不可改动的事实）：

* 厂商《两指平行夹爪用户使用手册 V1.0》§4.1 寄存器表 / §4.2.3.18 故障码位定义；
* SDK ``rm_define.h: rm_gripper_state_t``（7 个 int 字段，逐位注释见 ``abi_spec.yaml``）。

**换算系数缺失 → 对应字段为 ``None``（不猜、不填 0）**：位置毫米、力牛顿都依赖配置里的
换算系数（唯一来源 = ``workcell/rm_workcell.yaml::gripper``），缺失即 ``None``，
判定随之降级为 ``needs_param``，**绝不**产出 ``grasp_ok`` / ``grasp_empty``。

**红线**：恢复阶梯里**不得**出现「写寄存器 0x0008（清错误码）」这一动作——该动作需重启设备，
本项目禁止自动执行（:func:`recovery_ladder_violations` 供自检与单测断言）。
"""
from __future__ import annotations

from contextlib import contextmanager
from dataclasses import dataclass, field
from typing import Any, Dict, Iterator, List, Optional, Tuple

__all__ = [
    # 错误位
    "ERROR_BIT_STALL",
    "ERROR_BIT_OVERTEMP",
    "ERROR_BIT_OVERCURRENT",
    "ERROR_BIT_DRIVER",
    "ERROR_BIT_COMM",
    "ERROR_BIT_NAMES",
    "ERROR_BIT_LABELS",
    "decode_error_bits",
    "error_bits_text",
    # 模式
    "MODE_OPEN_MAX",
    "MODE_CLOSED_MIN",
    "MODE_STOPPED",
    "MODE_CLOSING",
    "MODE_OPENING",
    "MODE_FORCE_STOP",
    "MANUAL_MODE_MAP",
    "SDK_MODE_MAP",
    "PATH_MODBUS",
    "PATH_GRIPPER_API",
    "PATH_RM_PLUS",
    "mode_for_path",
    "mode_from_manual_code",
    "mode_from_sdk_code",
    # 归一化
    "GripperState",
    "normalize_state",
    # 判定
    "Judgment",
    "judge",
    "SEV_OK",
    "SEV_RECOVERABLE",
    "SEV_NEEDS_PARAM",
    "SEV_NEEDS_HUMAN",
    "RESULT_GRASP_OK",
    "RESULT_GRASP_EMPTY",
    "RESULT_STALL",
    "RESULT_OVERTEMP",
    "RESULT_OVERCURRENT",
    "RESULT_OFFLINE",
    "RESULT_NEEDS_PARAM",
    "RESULT_UNKNOWN",
    "REQUIRED_GRASP_THRESHOLDS",
    "RECOVERY_LADDER",
    "recovery_ladder_violations",
    "FORBIDDEN_ACTION_TOKENS",
    # 运动闸门
    "MotionGate",
    "shared_motion_gate",
    "gripper_action",
]


# --------------------------------------------------------------------------
# 1. 错误位常量与解码（手册 §4.2.3.18，与 SDK rm_gripper_state_t.error 逐位一致）
# --------------------------------------------------------------------------

#: bit0 堵转
ERROR_BIT_STALL = 1 << 0
#: bit1 过温
ERROR_BIT_OVERTEMP = 1 << 1
#: bit2 过流
ERROR_BIT_OVERCURRENT = 1 << 2
#: bit3 驱动器运行故障
ERROR_BIT_DRIVER = 1 << 3
#: bit4 内部通信故障
ERROR_BIT_COMM = 1 << 4

#: bit5–bit7 厂商保留位（解码时忽略，**不得报错**）
ERROR_BIT_RESERVED_MASK = 0b1110_0000

#: 错误位名 → 位掩码
ERROR_BIT_NAMES: Dict[str, int] = {
    "stall": ERROR_BIT_STALL,
    "overtemp": ERROR_BIT_OVERTEMP,
    "overcurrent": ERROR_BIT_OVERCURRENT,
    "driver": ERROR_BIT_DRIVER,
    "comm": ERROR_BIT_COMM,
}

#: 错误位名 → 中文标签（用于 reason 文本）
ERROR_BIT_LABELS: Dict[str, str] = {
    "stall": "堵转",
    "overtemp": "过温",
    "overcurrent": "过流",
    "driver": "驱动器故障",
    "comm": "内部通信故障",
}


def decode_error_bits(error: int) -> Dict[str, bool]:
    """把夹爪错误字解码成「每个错误位是否置位」。

    只看低 8 位；**bit5–bit7 为厂商保留位，忽略但不得报错**（不抛异常、不产生额外键）。

    :param error: 原始错误字（寄存器 0x0013 或 ``rm_gripper_state_t.error``）
    :return: ``{"stall": bool, "overtemp": bool, "overcurrent": bool,
              "driver": bool, "comm": bool}``
    """
    if isinstance(error, bool) or not isinstance(error, int):
        raise TypeError("error 必须是 int（原始错误字），收到 %r" % (type(error).__name__,))
    value = error & 0xFF
    return {name: bool(value & bit) for name, bit in ERROR_BIT_NAMES.items()}


def error_bits_text(error: int) -> str:
    """错误字 → 人类可读文本（保留位不出现）。"""
    bits = decode_error_bits(error)
    hit = [ERROR_BIT_LABELS[name] for name, on in bits.items() if on]
    return "、".join(hit) if hit else "无"


# --------------------------------------------------------------------------
# 2. 模式映射（两条路径不等价，各自写死）
# --------------------------------------------------------------------------

#: 归一化模式：张开到最大且空闲
MODE_OPEN_MAX = "open_max"
#: 归一化模式：闭合到最小且空闲
MODE_CLOSED_MIN = "closed_min"
#: 归一化模式：停止且空闲
MODE_STOPPED = "stopped"
#: 归一化模式：正在闭合
MODE_CLOSING = "closing"
#: 归一化模式：正在张开
MODE_OPENING = "opening"
#: 归一化模式：闭合过程中遇到力控停止（**只有 SDK 结构体有**）
MODE_FORCE_STOP = "force_stop"

#: 手册状态码（寄存器 0x0014）→ 归一化模式
MANUAL_MODE_MAP: Dict[int, str] = {
    1: MODE_OPEN_MAX,
    2: MODE_CLOSED_MIN,
    3: MODE_STOPPED,
    4: MODE_CLOSING,
    5: MODE_OPENING,
}

#: SDK ``rm_gripper_state_t.mode`` → 归一化模式（比手册多一个 6 = 力控停止）
SDK_MODE_MAP: Dict[int, str] = dict(MANUAL_MODE_MAP)
SDK_MODE_MAP[6] = MODE_FORCE_STOP

#: 协议路径标识
PATH_MODBUS = "modbus"
PATH_GRIPPER_API = "gripper_api"
PATH_RM_PLUS = "rm_plus"

#: 路径 → 模式映射表（modbus / rm_plus 走手册状态码；gripper_api 走 SDK mode）
_PATH_MODE_MAPS: Dict[str, Dict[int, str]] = {
    PATH_MODBUS: MANUAL_MODE_MAP,
    PATH_RM_PLUS: MANUAL_MODE_MAP,
    PATH_GRIPPER_API: SDK_MODE_MAP,
}


def mode_from_manual_code(code: Optional[int]) -> Optional[str]:
    """手册状态码（0x0014，1–5）→ 归一化模式；未知码返回 ``None``。"""
    if code is None:
        return None
    return MANUAL_MODE_MAP.get(int(code))


def mode_from_sdk_code(code: Optional[int]) -> Optional[str]:
    """SDK ``rm_gripper_state_t.mode``（1–6）→ 归一化模式；未知码返回 ``None``。"""
    if code is None:
        return None
    return SDK_MODE_MAP.get(int(code))


def mode_for_path(path: str, code: Optional[int]) -> Optional[str]:
    """按协议路径选择映射表解码模式码。

    两条路径**不等价**：官方夹爪 API 的 ``mode=6``（力控停止）在手册状态码里不存在。
    """
    table = _PATH_MODE_MAPS.get(path)
    if table is None:
        raise ValueError("未知协议路径: %r（允许 %s）" % (path, sorted(_PATH_MODE_MAPS)))
    if code is None:
        return None
    return table.get(int(code))


# --------------------------------------------------------------------------
# 3. 归一化快照
# --------------------------------------------------------------------------

#: 协议状态取值
PROTOCOL_STATES = ("unknown", "disable", "offline", "online")


@dataclass
class GripperState:
    """路径无关的夹爪状态快照（判定与留痕的唯一输入）。

    :param path: 协议路径（``modbus`` / ``gripper_api`` / ``rm_plus``）
    :param mode: 归一化模式字符串（见 ``MODE_*``）；无法映射时为 ``None``
    :param mode_raw: 原始模式码（手册状态码 / SDK mode），用于留痕
    :param position_mm: 开口度（mm）；换算系数缺失时为 ``None``
    :param force_norm: 归一化力度（0–1000 标度）；标度不明时为 ``None``
    :param force_N: 力度（N）；换算系数缺失时为 ``None``
    :param temp_c: 温度（℃）
    :param current_mA: 电流（mA）
    :param error_bits: 原始错误字（位图）
    :param online: 设备在线状态（``None`` = 未知）
    :param protocol_state: ``unknown`` / ``disable`` / ``offline`` / ``online``
    :param raw: 原始寄存器 / 结构体字段字典（证据留痕，原样保留）
    """

    path: str
    mode: Optional[str] = None
    mode_raw: Optional[int] = None
    position_mm: Optional[float] = None
    force_norm: Optional[float] = None
    force_N: Optional[float] = None
    temp_c: Optional[float] = None
    current_mA: Optional[float] = None
    error_bits: int = 0
    online: Optional[bool] = None
    protocol_state: str = "unknown"
    raw: Dict[str, Any] = field(default_factory=dict)

    def error_flags(self) -> Dict[str, bool]:
        """本快照的错误位解码（便捷方法）。"""
        return decode_error_bits(self.error_bits)

    def to_dict(self) -> Dict[str, Any]:
        """JSON 友好字典（留痕 / 报告用）。"""
        return {
            "path": self.path,
            "mode": self.mode,
            "mode_raw": self.mode_raw,
            "position_mm": self.position_mm,
            "force_norm": self.force_norm,
            "force_N": self.force_N,
            "temp_c": self.temp_c,
            "current_mA": self.current_mA,
            "error_bits": self.error_bits,
            "error_flags": self.error_flags(),
            "online": self.online,
            "protocol_state": self.protocol_state,
            "raw": dict(self.raw),
        }


def _as_float(value: Any) -> Optional[float]:
    if value is None or isinstance(value, bool):
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def normalize_state(
    path: str,
    raw: Dict[str, Any],
    *,
    position_mm_per_unit: Optional[float] = None,
    force_scale: Optional[float] = None,
) -> GripperState:
    """把某条协议路径的原始值归一化成 :class:`GripperState`。

    原始字典的**规范键**（缺省即 ``None``，不猜、不填 0）：

    ==================  =====================================================
    键                  含义
    ==================  =====================================================
    ``mode``            模式码（手册状态码 / SDK mode，按 path 解码）
    ``position_raw``    原始开口度（源标度）
    ``force_raw``       原始力度（源标度：Modbus 0–1000，SDK 单位 g）
    ``force_norm``      可选，适配层已折算到 0–1000 标度的力度
    ``temp_c``          温度（℃）
    ``current_mA``      电流（mA）
    ``error``           原始错误字（位图）
    ``online``          设备在线（bool）
    ``protocol_state``  ``unknown`` / ``disable`` / ``offline`` / ``online``
    ==================  =====================================================

    :param position_mm_per_unit: 开口度换算系数（mm / 源单位）；缺失 → ``position_mm=None``
    :param force_scale: 力度换算系数（N / 源单位）；缺失 → ``force_N=None``
    """
    raw = dict(raw or {})
    if path not in _PATH_MODE_MAPS:
        raise ValueError("未知协议路径: %r（允许 %s）" % (path, sorted(_PATH_MODE_MAPS)))

    mode_raw = raw.get("mode")
    mode = mode_for_path(path, mode_raw)

    position_raw = _as_float(raw.get("position_raw"))
    position_mm = None
    if position_raw is not None and position_mm_per_unit is not None:
        position_mm = position_raw * float(position_mm_per_unit)

    force_raw = _as_float(raw.get("force_raw"))
    force_norm = _as_float(raw.get("force_norm"))
    if force_norm is None and force_raw is not None and path == PATH_MODBUS:
        # Modbus 0x000E 本身就是 0–1000 标度；其他路径必须由适配层显式折算。
        force_norm = min(max(force_raw, 0.0), 1000.0)

    force_N = None
    if force_raw is not None and force_scale is not None:
        force_N = force_raw * float(force_scale)

    protocol_state = raw.get("protocol_state") or "unknown"
    if protocol_state not in PROTOCOL_STATES:
        raise ValueError(
            "未知 protocol_state: %r（允许 %s）" % (protocol_state, list(PROTOCOL_STATES))
        )

    online = raw.get("online")
    if online is not None:
        online = bool(online)

    error_bits = raw.get("error") or 0
    if isinstance(error_bits, bool) or not isinstance(error_bits, int):
        raise TypeError("raw['error'] 必须是 int（原始错误字）")

    return GripperState(
        path=path,
        mode=mode,
        mode_raw=int(mode_raw) if mode_raw is not None else None,
        position_mm=position_mm,
        force_norm=force_norm,
        force_N=force_N,
        temp_c=_as_float(raw.get("temp_c")),
        current_mA=_as_float(raw.get("current_mA")),
        error_bits=error_bits,
        online=online,
        protocol_state=protocol_state,
        raw=raw,
    )


# --------------------------------------------------------------------------
# 4. 判定
# --------------------------------------------------------------------------

#: 判定结果
SEV_OK = "OK"
SEV_RECOVERABLE = "RECOVERABLE"
SEV_NEEDS_PARAM = "NEEDS_PARAM"
SEV_NEEDS_HUMAN = "NEEDS_HUMAN"

RESULT_GRASP_OK = "grasp_ok"
RESULT_GRASP_EMPTY = "grasp_empty"
RESULT_STALL = "stall"
RESULT_OVERTEMP = "overtemp"
RESULT_OVERCURRENT = "overcurrent"
RESULT_OFFLINE = "offline"
RESULT_NEEDS_PARAM = "needs_param"
RESULT_UNKNOWN = "unknown"

#: 判定「夹到 / 夹空」所必需的阈值键（缺任一 → 只能给 needs_param）
REQUIRED_GRASP_THRESHOLDS = (
    "force_threshold",
    "grasp_position_window",
    "position_mm_per_unit",
    "force_scale",
)

#: 默认阶梯上限（可被 thresholds 覆盖）
DEFAULT_MAX_REGRASP = 2
DEFAULT_MAX_STALL_RETRY = 1
DEFAULT_MAX_OFFLINE_RETRY = 1


@dataclass
class Judgment:
    """判定结果。

    :param result: ``grasp_ok`` / ``grasp_empty`` / ``stall`` / ``overtemp`` /
                   ``overcurrent`` / ``offline`` / ``needs_param`` / ``unknown``
    :param severity: ``OK`` / ``RECOVERABLE`` / ``NEEDS_PARAM`` / ``NEEDS_HUMAN``
    :param reason: 判定依据（写明触发了哪条判据）
    :param evidence: 客观证据（原始快照、错误位、阈值、缺失清单）
    :param next_action: **可粘贴**的下一步
    :param retryable: 是否允许自动重试
    """

    result: str
    severity: str
    reason: str
    evidence: Dict[str, Any] = field(default_factory=dict)
    next_action: str = ""
    retryable: bool = False

    def to_dict(self) -> Dict[str, Any]:
        """JSON 友好字典。"""
        return {
            "result": self.result,
            "severity": self.severity,
            "reason": self.reason,
            "evidence": dict(self.evidence),
            "next_action": self.next_action,
            "retryable": bool(self.retryable),
        }


#: 恢复阶梯（表驱动）：判定结果 → 步骤表。
#: 每步 = ``{"severity": ..., "retryable": ..., "limit": 阈值键或 None, "next_action": 模板}``。
#: 步骤选择：``attempt < limit`` 取第 0 步（可重试），否则取最后一步（升级人工）。
RECOVERY_LADDER: Dict[str, List[Dict[str, Any]]] = {
    RESULT_GRASP_EMPTY: [
        {
            "severity": SEV_RECOVERABLE,
            "retryable": True,
            "limit": "max_regrasp",
            "next_action": "退回到安全开口后重抓（第 {attempt}/{max_regrasp} 次尝试）",
        },
        {
            "severity": SEV_NEEDS_HUMAN,
            "retryable": False,
            "limit": None,
            "next_action": "重抓次数已用尽（{attempt}/{max_regrasp}），需人工确认工件与夹爪状态",
        },
    ],
    RESULT_STALL: [
        {
            "severity": SEV_RECOVERABLE,
            "retryable": True,
            "limit": "max_stall_retry",
            "next_action": "松手后重试闭合（第 {attempt}/{max_stall_retry} 次尝试）",
        },
        {
            "severity": SEV_NEEDS_HUMAN,
            "retryable": False,
            "limit": None,
            "next_action": "松手重试后仍堵转（{attempt}/{max_stall_retry}），需人工检查夹爪机械卡滞",
        },
    ],
    RESULT_OVERTEMP: [
        {
            "severity": SEV_NEEDS_HUMAN,
            "retryable": False,
            "limit": None,
            "next_action": "停机冷却（当前 {temp_c} ℃），禁止自动重试；温度回落后由人工复位",
        },
    ],
    RESULT_OVERCURRENT: [
        {
            "severity": SEV_NEEDS_PARAM,
            "retryable": False,
            "limit": None,
            "next_action": (
                "确认夹爪使用外部 24 V 供电并与法兰共地"
                "（法兰供电上限 1.5 A < 额定 1.9 A），再重新下发夹爪动作"
            ),
        },
    ],
    RESULT_OFFLINE: [
        {
            "severity": SEV_RECOVERABLE,
            "retryable": True,
            "limit": "max_offline_retry",
            "next_action": "夹爪协议已开但设备离线：重连后重试（第 {attempt}/{max_offline_retry} 次）",
        },
        {
            "severity": SEV_NEEDS_HUMAN,
            "retryable": False,
            "limit": None,
            "next_action": "夹爪离线重试已用尽（{attempt}/{max_offline_retry}），需人工检查线缆与外部供电",
        },
    ],
    "offline_disabled": [
        {
            "severity": SEV_NEEDS_PARAM,
            "retryable": False,
            "limit": None,
            "next_action": "夹爪协议未开（disable）：先开启末端 Modbus-RTU 或 RM Plus 协议，再重试",
        },
    ],
    RESULT_NEEDS_PARAM: [
        {
            "severity": SEV_NEEDS_PARAM,
            "retryable": False,
            "limit": None,
            "next_action": "补齐配置项（{missing}）后重跑夹爪判定",
        },
    ],
    RESULT_UNKNOWN: [
        {
            "severity": SEV_NEEDS_HUMAN,
            "retryable": False,
            "limit": None,
            "next_action": "无法归类夹爪状态，需人工确认（原因：{reason}）",
        },
    ],
}

#: 恢复阶梯红线：这些动作**禁止**出现在任何 ``next_action`` 里
#: （写 0x0008 清错误码需重启设备，本项目禁止自动执行）。
FORBIDDEN_ACTION_TOKENS = ("0x0008", "清除错误码", "清错误", "clear_error")


def recovery_ladder_violations() -> List[str]:
    """自检：返回恢复阶梯里违反红线的动作描述（空列表 = 干净）。"""
    violations: List[str] = []
    for result, steps in RECOVERY_LADDER.items():
        for index, step in enumerate(steps):
            action = str(step.get("next_action", ""))
            for token in FORBIDDEN_ACTION_TOKENS:
                if token in action:
                    violations.append("%s[%d] 含有禁止动作 %r：%s" % (result, index, token, action))
    return violations


class _SafeDict(dict):
    """格式化占位符缺失时回落到 ``?``（不抛 KeyError）。"""

    def __missing__(self, key: str) -> str:  # pragma: no cover - 防御分支
        return "?"


def _render(action: str, ctx: Dict[str, Any]) -> str:
    try:
        return action.format_map(_SafeDict(ctx))
    except (ValueError, IndexError):  # pragma: no cover - 防御分支
        return action


def _normalize_thresholds(thresholds: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    """把阈值字典规范化（缺 → ``None``；非法窗口 → ``None``；不猜默认值）。"""
    raw = dict(thresholds or {})
    out: Dict[str, Any] = dict(raw)

    for key in ("force_threshold", "position_mm_per_unit", "force_scale"):
        out[key] = _as_float(raw.get(key))

    window = raw.get("grasp_position_window")
    if window is None or isinstance(window, (str, bytes)) or len(window) != 2:
        out["grasp_position_window"] = None
    else:
        lo = _as_float(window[0])
        hi = _as_float(window[1])
        if lo is None or hi is None or lo > hi:
            out["grasp_position_window"] = None
        else:
            out["grasp_position_window"] = (lo, hi)

    def _int(key: str, default: int) -> int:
        value = raw.get(key, default)
        if value is None:
            return default
        try:
            return int(value)
        except (TypeError, ValueError):
            return default

    out["attempt"] = max(0, _int("attempt", 0))
    out["max_regrasp"] = max(0, _int("max_regrasp", DEFAULT_MAX_REGRASP))
    out["max_stall_retry"] = max(0, _int("max_stall_retry", DEFAULT_MAX_STALL_RETRY))
    out["max_offline_retry"] = max(0, _int("max_offline_retry", DEFAULT_MAX_OFFLINE_RETRY))
    return out


def _ladder_step(result: str, ladder_key: str, th: Dict[str, Any],
                 ctx: Dict[str, Any]) -> Tuple[str, bool, str]:
    """从恢复阶梯里取下一步 → ``(severity, retryable, next_action)``。"""
    steps = RECOVERY_LADDER.get(ladder_key) or RECOVERY_LADDER[result]
    limit_key = steps[0].get("limit")
    if limit_key is None or th.get("attempt", 0) < th.get(limit_key, 0):
        step = steps[0]
    else:
        step = steps[-1]
    return (
        str(step["severity"]),
        bool(step["retryable"]),
        _render(str(step["next_action"]), ctx),
    )


def judge(state: GripperState, thresholds: Optional[Dict[str, Any]] = None) -> Judgment:
    """纯逻辑判定：给定归一化状态与阈值，给出六类判定之一（含 needs_param / unknown）。

    判定优先级（自上而下，先命中先返回）：

    1. ``protocol_state == "disable"`` → ``offline`` / ``NEEDS_PARAM``（协议未开）
    2. ``protocol_state == "offline"`` 或 ``online is False`` → ``offline``（可重试 → 升级人工）
    3. 错误位：过温 → 过流 → 堵转 → 驱动器/内部通信（unknown）
    4. 动作进行中（closing / opening）→ ``unknown``（保守）
    5. 阈值/换算系数缺失 → ``needs_param`` / ``NEEDS_PARAM``（**不得**产出 grasp_ok/grasp_empty）
    6. 力度 ≥ 阈值且位置落在夹持窗口 → ``grasp_ok``；否则力度不足 → ``grasp_empty``

    :param state: :class:`GripperState` 归一化快照
    :param thresholds: 阈值字典；必需的键见 :data:`REQUIRED_GRASP_THRESHOLDS`，
        可选键 ``attempt`` / ``max_regrasp`` / ``max_stall_retry`` / ``max_offline_retry``
    """
    th = _normalize_thresholds(thresholds)
    missing = [key for key in REQUIRED_GRASP_THRESHOLDS if th.get(key) is None]

    bits = state.error_flags()
    ctx: Dict[str, Any] = {
        "attempt": th["attempt"],
        "max_regrasp": th["max_regrasp"],
        "max_stall_retry": th["max_stall_retry"],
        "max_offline_retry": th["max_offline_retry"],
        "temp_c": state.temp_c if state.temp_c is not None else "未知",
        "missing": ", ".join(missing) or "无",
        "reason": "",
    }

    # 位置/力度的有效值：优先用快照里已换算的字段，缺失时用 raw + 阈值换算。
    position_mm = state.position_mm
    if position_mm is None:
        position_raw = _as_float(state.raw.get("position_raw"))
        if position_raw is not None and th.get("position_mm_per_unit") is not None:
            position_mm = position_raw * float(th["position_mm_per_unit"])

    force_norm = state.force_norm
    force_N = state.force_N
    if force_N is None:
        force_raw = _as_float(state.raw.get("force_raw"))
        if force_raw is not None and th.get("force_scale") is not None:
            force_N = force_raw * float(th["force_scale"])

    evidence: Dict[str, Any] = {
        "path": state.path,
        "mode": state.mode,
        "mode_raw": state.mode_raw,
        "error_bits": state.error_bits,
        "error_flags": bits,
        "protocol_state": state.protocol_state,
        "online": state.online,
        "position_mm": position_mm,
        "force_norm": force_norm,
        "force_N": force_N,
        "temp_c": state.temp_c,
        "current_mA": state.current_mA,
        "attempt": th["attempt"],
        "thresholds": {k: v for k, v in th.items() if k != "attempt"},
        "thresholds_missing": missing,
        "raw": dict(state.raw),
    }

    def _finish(result: str, severity: str, reason: str, next_action: str,
                retryable: bool) -> Judgment:
        return Judgment(
            result=result,
            severity=severity,
            reason=reason,
            evidence=evidence,
            next_action=next_action,
            retryable=retryable,
        )

    # 1) 协议未开 → NEEDS_PARAM（与「已开但离线」区分）
    if state.protocol_state == "disable":
        severity, retryable, action = _ladder_step(RESULT_OFFLINE, "offline_disabled", th, ctx)
        return _finish(
            RESULT_OFFLINE,
            severity,
            "夹爪协议未开（protocol_state=disable），无法判定夹爪状态",
            action,
            retryable,
        )

    # 2) 已开但离线
    if state.protocol_state == "offline" or state.online is False:
        severity, retryable, action = _ladder_step(RESULT_OFFLINE, RESULT_OFFLINE, th, ctx)
        return _finish(
            RESULT_OFFLINE,
            severity,
            "夹爪离线（protocol_state=%s, online=%s）" % (state.protocol_state, state.online),
            action,
            retryable,
        )

    # 3) 错误位（优先级：过温 > 过流 > 堵转 > 驱动器/内部通信）
    if bits["overtemp"]:
        severity, retryable, action = _ladder_step(RESULT_OVERTEMP, RESULT_OVERTEMP, th, ctx)
        return _finish(
            RESULT_OVERTEMP,
            severity,
            "过温（error bit1），当前温度 %s ℃" % ctx["temp_c"],
            action,
            retryable,
        )
    if bits["overcurrent"]:
        severity, retryable, action = _ladder_step(
            RESULT_OVERCURRENT, RESULT_OVERCURRENT, th, ctx
        )
        return _finish(
            RESULT_OVERCURRENT,
            severity,
            "过流（error bit2）",
            action,
            retryable,
        )
    if bits["stall"]:
        severity, retryable, action = _ladder_step(RESULT_STALL, RESULT_STALL, th, ctx)
        return _finish(
            RESULT_STALL,
            severity,
            "堵转（error bit0）",
            action,
            retryable,
        )
    if bits["driver"] or bits["comm"]:
        reason = "夹爪错误位异常：%s" % error_bits_text(state.error_bits)
        ctx["reason"] = reason
        severity, retryable, action = _ladder_step(RESULT_UNKNOWN, RESULT_UNKNOWN, th, ctx)
        return _finish(RESULT_UNKNOWN, severity, reason, action, retryable)

    # 4) 动作进行中 → 保守 unknown
    if state.mode in (MODE_CLOSING, MODE_OPENING):
        reason = "夹爪动作进行中（mode=%s），状态未稳定" % state.mode
        ctx["reason"] = reason
        severity, retryable, action = _ladder_step(RESULT_UNKNOWN, RESULT_UNKNOWN, th, ctx)
        return _finish(RESULT_UNKNOWN, severity, reason, action, retryable)

    # 5) 阈值 / 换算系数缺失 → needs_param（绝不产出 grasp_ok / grasp_empty）
    if missing:
        severity, retryable, action = _ladder_step(
            RESULT_NEEDS_PARAM, RESULT_NEEDS_PARAM, th, ctx
        )
        return _finish(
            RESULT_NEEDS_PARAM,
            severity,
            "换算系数或阈值缺失（%s），无法判定夹到/夹空" % ", ".join(missing),
            action,
            retryable,
        )

    if force_norm is None or position_mm is None:
        evidence["force_norm"] = force_norm
        evidence["position_mm"] = position_mm
        reason = "缺少力度或位置原始值（force_norm=%s, position_mm=%s），无法判定夹到/夹空" % (
            force_norm,
            position_mm,
        )
        ctx["reason"] = reason
        severity, retryable, action = _ladder_step(
            RESULT_NEEDS_PARAM, RESULT_NEEDS_PARAM, th, ctx
        )
        return _finish(RESULT_NEEDS_PARAM, severity, reason, action, retryable)

    # 6) 夹到 / 夹空
    lo, hi = th["grasp_position_window"]
    force_threshold = float(th["force_threshold"])
    if force_norm >= force_threshold:
        if lo <= position_mm <= hi:
            return _finish(
                RESULT_GRASP_OK,
                SEV_OK,
                "力度 %s ≥ 阈值 %s 且开口度 %s mm 落在夹持窗口 [%s, %s] mm"
                % (force_norm, force_threshold, position_mm, lo, hi),
                "夹爪已夹持工件，可继续后续动作",
                False,
            )
        reason = "力度 %s ≥ 阈值 %s 但开口度 %s mm 不在夹持窗口 [%s, %s] mm" % (
            force_norm,
            force_threshold,
            position_mm,
            lo,
            hi,
        )
        ctx["reason"] = reason
        severity, retryable, action = _ladder_step(RESULT_UNKNOWN, RESULT_UNKNOWN, th, ctx)
        return _finish(RESULT_UNKNOWN, severity, reason, action, retryable)

    severity, retryable, action = _ladder_step(
        RESULT_GRASP_EMPTY, RESULT_GRASP_EMPTY, th, ctx
    )
    return _finish(
        RESULT_GRASP_EMPTY,
        severity,
        "力度 %s < 阈值 %s（开口度 %s mm），判为夹空" % (force_norm, force_threshold, position_mm),
        action,
        retryable,
    )


# --------------------------------------------------------------------------
# 5. 运动闸门（纯逻辑，零 SDK）
# --------------------------------------------------------------------------

class MotionGate:
    """夹爪 ↔ 运动的互锁闸门（纯逻辑）。

    三态：

    * ``IDLE``：无夹爪动作在途 → ``check()`` 放行（返回 ``None``）；
    * ``GRIPPER_BUSY``：夹爪动作进行中 → **拒绝下发任何运动**；
    * ``GRIPPER_UNJUDGED``：夹爪动作结束但状态未判定 → **拒绝下发任何运动**。
    """

    #: 空闲
    STATE_IDLE = "IDLE"
    #: 夹爪动作进行中
    STATE_GRIPPER_BUSY = "GRIPPER_BUSY"
    #: 夹爪动作结束但未判定
    STATE_GRIPPER_UNJUDGED = "GRIPPER_UNJUDGED"

    #: 三态全集
    STATES = (STATE_IDLE, STATE_GRIPPER_BUSY, STATE_GRIPPER_UNJUDGED)

    #: 各状态的 next_action 文案
    _NEXT_ACTIONS = {
        STATE_IDLE: "",
        STATE_GRIPPER_BUSY: "等待夹爪动作完成后再下发运动",
        STATE_GRIPPER_UNJUDGED: "先完成夹爪状态判定（rmctl gripper judge）确认结果后再下发运动",
    }

    def __init__(self, state: str = STATE_IDLE, reason: str = "") -> None:
        if state not in self.STATES:
            raise ValueError("未知闸门状态: %r（允许 %s）" % (state, list(self.STATES)))
        self._state = state
        self._reason = reason

    # -- 只读属性 ----------------------------------------------------------
    @property
    def state(self) -> str:
        """当前闸门状态（三态之一）。"""
        return self._state

    @property
    def reason(self) -> str:
        """进入当前状态的原因。"""
        return self._reason

    @property
    def is_blocked(self) -> bool:
        """当前是否拒绝下发运动。"""
        return self._state != self.STATE_IDLE

    # -- 状态迁移 ----------------------------------------------------------
    def begin_action(self, reason: str = "") -> str:
        """夹爪动作开始 → ``GRIPPER_BUSY``。"""
        self._state = self.STATE_GRIPPER_BUSY
        self._reason = reason or "夹爪动作进行中"
        return self._state

    def finish_action(self) -> str:
        """夹爪动作结束且已判定 → ``IDLE``。"""
        self._state = self.STATE_IDLE
        self._reason = ""
        return self._state

    def mark_unjudged(self, reason: str = "") -> str:
        """夹爪动作结束但状态未判定 → ``GRIPPER_UNJUDGED``。"""
        self._state = self.STATE_GRIPPER_UNJUDGED
        self._reason = reason or "夹爪状态未判定"
        return self._state

    def reset(self) -> str:
        """无条件复位到 ``IDLE``（人工干预后使用）。"""
        self._state = self.STATE_IDLE
        self._reason = ""
        return self._state

    # -- 闸门检查 ----------------------------------------------------------
    def check(self) -> Optional[Dict[str, Any]]:
        """运动下发前的闸门检查。

        :return: ``None`` = 放行；否则 ``{"state": ..., "reason": ..., "next_action": ...}``
        """
        if self._state == self.STATE_IDLE:
            return None
        return {
            "state": self._state,
            "reason": self._reason,
            "next_action": self._NEXT_ACTIONS[self._state],
        }

    def __repr__(self) -> str:  # pragma: no cover - 调试辅助
        return "MotionGate(state=%r, reason=%r)" % (self._state, self._reason)


# --------------------------------------------------------------------------
# 6. 进程内**共享**的运动闸门
# --------------------------------------------------------------------------

#: 进程内共享的闸门实例（**单例**）——夹爪动作客户端与执行器共用同一个对象，
#: 这样「夹爪动作进行中」的状态对运动下发方立即可见。
_SHARED_MOTION_GATE: Optional[MotionGate] = None


def shared_motion_gate() -> MotionGate:
    """返回**进程内共享**的 :class:`MotionGate` 实例（首次调用时创建）。

    * 同一进程内的**夹爪动作客户端**（``rm_hw_bridge.gripper`` / ``rmctl
      gripper``）与**执行器**（``rm_executor``）拿到的是**同一个对象** →
      夹爪一动作，执行器的运动闸门立刻变为非 IDLE。
    * 纯逻辑、零 SDK、零 I/O（本模块不碰任何寄存器）。
    """
    global _SHARED_MOTION_GATE
    if _SHARED_MOTION_GATE is None:
        _SHARED_MOTION_GATE = MotionGate()
    return _SHARED_MOTION_GATE


@contextmanager
def gripper_action(reason: str = "", gate: Optional[MotionGate] = None
                   ) -> Iterator[MotionGate]:
    """夹爪动作的**闸门护栏**：动作期间闸门 = ``GRIPPER_BUSY``。

    用法（夹爪动作客户端）::

        with gripper_action("夹取中"):
            adapter.gripper_pick(speed=100, force=100, block=True, timeout=2000)

    * 动作期间 → 闸门 ``GRIPPER_BUSY`` → 执行器在阶段 B / C 前**拒绝运动**；
    * 动作**正常结束** → 闸门 ``GRIPPER_UNJUDGED``（动作完成但状态**未判定**）
      → 运动**仍然被拒**（必须先完成判定）；
    * 动作**抛异常** → 同样 ``GRIPPER_UNJUDGED``（异常退出也**不假装判定过**）。

    纯逻辑：``gate`` 缺省用进程内共享实例；不碰任何寄存器。
    """
    g = gate if gate is not None else shared_motion_gate()
    g.begin_action(reason or "夹爪动作进行中")
    try:
        yield g
    finally:
        # 无论正常结束还是抛异常，动作结束后都**未判定** → 保守保持拒绝运动
        g.mark_unjudged(reason or "夹爪动作已结束，状态待判定")
