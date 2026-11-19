"""传输层：真机实现（:class:`SdkTransport`）与离线替身（:class:`ScriptedTransport`）。

为什么要有传输层抽象
--------------------
研究 F1：真机在当前环境**不可达**（两台开发机都 ping 不到机械臂），
但 S03 的安全七闸 / 状态机 / 故障恢复流程**必须**能在无真机条件下被真实验证。
⇒ 把「与机械臂说话」这件事收敛到**一个可替换的边界**：

* :class:`ArmTransport` —— 协议（最小运动与状态接口）；
* :class:`SdkTransport` —— 真机实现（**只用白名单 SDK 符号**）；
* :class:`ScriptedTransport` —— 离线替身（可脚本化逐帧回放故障）。

所有下游逻辑（安全闸 / 状态机 / 执行环 / 恢复流程）只依赖 :class:`ArmTransport`，
因此「拔网线 / 物体掉落 / 夹爪夹空 / 关节超限位 / 供电跌落」五类故障可以成为
**真正的单元测试**。

单位纪律
--------
本模块**对外一律 rad**（弧度）；只有 :class:`SdkTransport` 在**下发前最后一刻**
调用 :func:`rm75_drake.device.sdk.rad_to_deg` 换算成度（SDK 全线用度）——
换算点唯一，源头见 :mod:`rm75_drake.device.sdk`。
"""

from __future__ import annotations

import abc
import math
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Deque, List, Optional, Sequence, Tuple

from ..planning.trajectory import JOINT_COUNT
from .binding import RM_TRIPLE_MODE_E, ArmBinding
from .config import DeviceConfig
from .sdk import NotAvailableError, SdkAudit, deg_to_rad, rad_to_deg

__all__ = [
    "TransportError",
    "REASON_NOT_OPEN",
    "REASON_CLOSED",
    "REASON_SDK_UNAVAILABLE",
    "REASON_LINK_DOWN",
    "REASON_INVALID_INPUT",
    "REASON_GRIPPER_OFFLINE",
    "REASON_UNSUPPORTED",
    "GRIPPER_POSITION_MIN",
    "GRIPPER_POSITION_MAX",
    "GRIPPER_BLOCK_TIMEOUT_S",
    "RM_TRIPLE_MODE_E",
    "FRAME_INCOMPLETE_ERR_CODE",
    "StateFrame",
    "GripperState",
    "ArmTransport",
    "SdkTransport",
    "ScriptedTransport",
]


# ── 失败标签（稳定）─────────────────────────────────────────────────────
REASON_NOT_OPEN = "not_open"
REASON_CLOSED = "closed"
REASON_SDK_UNAVAILABLE = "sdk_unavailable"
REASON_LINK_DOWN = "link_down"
REASON_INVALID_INPUT = "invalid_input"
REASON_GRIPPER_OFFLINE = "gripper_offline"
REASON_UNSUPPORTED = "unsupported"

#: 夹爪位置是**无量纲**的 0–1000 整数（不是角度，不做任何 rad/deg 换算）。
GRIPPER_POSITION_MIN = 0
GRIPPER_POSITION_MAX = 1000

#: 夹爪下发的**阻塞超时**（s）——阻塞模式等控制器返回「夹爪到位」，
#: 超时返回 ``-4``。夹爪单程约 0.4 s，取 5 s 留足裕量。
GRIPPER_BLOCK_TIMEOUT_S = 5

#: 帧级系统错误码里的**特殊值**：``errCode == -3`` 表示「推送数据不完整」
#: ⇒ 该帧必须丢弃（**不是**故障原因）。其余非 0 值一律视为系统错误码。
#: 本常量是这条语义的**唯一**真相源（状态源 / 状态机 / 恢复流程共用）。
FRAME_INCOMPLETE_ERR_CODE = -3


def _cleared_err_code(code: int) -> int:
    """清错后的帧级错误码：系统错误码清零，但**保留**帧不完整标记（``-3``）。

    ``-3`` 不是错误码，而是「这一帧必须丢弃」的标记 ⇒ 清错不能把坏帧
    变成好帧。
    """
    value = int(code)
    if value == FRAME_INCOMPLETE_ERR_CODE:
        return value
    return 0


class TransportError(RuntimeError):
    """传输层错误（带稳定 ``reason`` 标签）。

    属性
    ----
    reason:
        稳定标签（见模块常量）。
    """

    def __init__(self, message: str, *, reason: str = "transport_error") -> None:
        super().__init__(message)
        self.reason = reason


# ── 数据结构 ────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class StateFrame:
    """一帧机械臂状态（等价于 UDP 主动上报的一帧）。

    字段
    ----
    joint_positions_rad:
        关节角（**rad**，7 维）。
    joint_currents_ma:
        逐轴电流（mA，7 维）。
    joint_en_flags:
        逐轴使能标志（7 个 bool）。
    joint_err_codes:
        逐轴错误码（7 个 int，0 = 无错）。
    controller_status:
        控制器状态枚举（``arm_current_status``）。
    gripper_online:
        夹爪是否在线。
    arm_current_status:
        机械臂状态枚举（0 IDLE / 5 角度透传 / 9 急停 / 10 缓停 …）。
    received_at:
        帧到达时刻（**秒**，单调时钟口径）。
    err_code:
        帧完整性错误码（``errCode``；``-3`` = 推送数据不完整 → 帧必须丢弃）。
    """

    joint_positions_rad: Tuple[float, ...] = (0.0,) * JOINT_COUNT
    joint_currents_ma: Tuple[float, ...] = (0.0,) * JOINT_COUNT
    joint_en_flags: Tuple[bool, ...] = (False,) * JOINT_COUNT
    joint_err_codes: Tuple[int, ...] = (0,) * JOINT_COUNT
    controller_status: int = 0
    gripper_online: bool = True
    arm_current_status: int = 0
    received_at: float = 0.0
    err_code: int = 0

    @property
    def is_complete(self) -> bool:
        """帧是否完整（``errCode == -3`` 视为推送数据不完整 → 必须丢弃）。"""
        return int(self.err_code) != FRAME_INCOMPLETE_ERR_CODE

    @property
    def all_enabled(self) -> bool:
        """是否全部 7 轴都已使能（WAIT_ENABLE → READY 的守卫条件）。"""
        return all(bool(flag) for flag in self.joint_en_flags)


@dataclass(frozen=True)
class GripperState:
    """夹爪状态。"""

    online: bool
    position: int = 0

    def __post_init__(self) -> None:
        object.__setattr__(self, "online", bool(self.online))
        object.__setattr__(self, "position", int(self.position))


# ── 协议 ────────────────────────────────────────────────────────────────
class ArmTransport(abc.ABC):
    """与机械臂对话的**最小**协议。

    子类必须实现
    ------------
    :meth:`open`、:meth:`close`、:meth:`move_j_canfd`、:meth:`enable`、
    :meth:`clear_error`、:meth:`read_gripper`、:meth:`read_state`。
    """

    #: 稳定标识（写进运行日志）。
    name: str = "transport"

    def __init__(self) -> None:
        self._open = False
        self._closed = False

    # -- 生命周期 --------------------------------------------------------
    @abc.abstractmethod
    def open(self) -> None:
        """打开连接（幂等）。"""

    @abc.abstractmethod
    def close(self) -> None:
        """关闭连接（幂等）。"""

    @property
    def is_open(self) -> bool:
        """连接是否已打开。"""
        return self._open

    # -- 运动 ------------------------------------------------------------
    @abc.abstractmethod
    def move_j_canfd(self, joint_rad: Sequence[float], *, follow: bool = True) -> None:
        """透传下发关节角（**唯一运动通道**）。

        参数
        ----
        joint_rad:
            目标关节角（**rad**，7 维）。
        follow:
            是否跟随（真机透传必须 ``True``）。
        """

    @abc.abstractmethod
    def enable(self, state: bool) -> None:
        """整臂使能 / 去使能。"""

    @abc.abstractmethod
    def clear_error(self) -> None:
        """清除逐关节错误码与系统错误码。"""

    @abc.abstractmethod
    def emergency_stop(self) -> None:
        """急停（软件侧的兜底；物理急停不可被软件替代）。"""

    @abc.abstractmethod
    def slow_stop(self) -> None:
        """缓停。"""

    @abc.abstractmethod
    def set_joint_limits(
        self, lower_rad: Sequence[float], upper_rad: Sequence[float]
    ) -> None:
        """设置真机限位（**rad**，7 维；恢复流程里的「放宽 / 恢复限位」用）。"""

    # -- 状态读取 --------------------------------------------------------
    @abc.abstractmethod
    def read_state(self) -> Optional[StateFrame]:
        """读取最新一帧状态；无可用帧时返回 ``None``。"""

    @abc.abstractmethod
    def read_gripper(self) -> GripperState:
        """读取夹爪状态。"""

    # -- 夹爪下发 --------------------------------------------------------
    def command_gripper_position(self, position: int, *, speed: int = 100) -> None:
        """下发夹爪位置（**夹爪下发通道**）。

        命名理由
        --------
        :meth:`ScriptedTransport.set_gripper_position` 是**回读值设定器**
        （模拟「夹爪已到位」），被 ``tests/test_device_recovery.py`` 与
        ``scripts/fault_inject.py`` 使用，其语义**不得改动**；因此**下发**
        通道另起名 ``command_gripper_position``，避免同名不同义。

        参数
        ----
        position:
            夹爪位置（**无量纲** 0–1000 整数；不涉及 rad/deg 换算）。
        speed:
            夹爪速度（0–100 整数；仅做入参校验，当前 SDK 白名单符号只接受位置）。

        异常
        ----
        TransportError
            基类默认实现恒抛 ``reason="unsupported"``（当前传输层不支持夹爪下发）。
        """
        raise TransportError(
            "command_gripper_position：当前传输层不支持夹爪下发"
            "（基类默认实现；请由具体传输层覆盖本方法）",
            reason=REASON_UNSUPPORTED,
        )

    # -- 内部工具 --------------------------------------------------------
    def _require_open(self) -> None:
        if not self._open:
            raise TransportError(
                "传输层未打开（先调用 open()）",
                reason=REASON_CLOSED if self._closed else REASON_NOT_OPEN,
            )


def _validate_joint_vector(
    values: Sequence[float], name: str, dof: int = JOINT_COUNT
) -> Tuple[float, ...]:
    """校验关节向量：长度必须是 ``dof``，每个元素是有限实数。"""
    if isinstance(values, (str, bytes)) or not isinstance(values, Sequence):
        raise TransportError(
            f"{name} 必须是 {dof} 维序列，收到 {type(values).__name__}",
            reason=REASON_INVALID_INPUT,
        )
    if len(values) != dof:
        raise TransportError(
            f"{name} 必须是 {dof} 维序列（实际 {len(values)} 维）",
            reason=REASON_INVALID_INPUT,
        )
    out: List[float] = []
    for i, v in enumerate(values):
        if isinstance(v, bool) or not isinstance(v, (int, float)):
            raise TransportError(
                f"{name}[{i}] 必须是数值，收到 {v!r}", reason=REASON_INVALID_INPUT
            )
        fv = float(v)
        if not math.isfinite(fv):
            raise TransportError(
                f"{name}[{i}] 必须是有限数值，收到 {v!r}", reason=REASON_INVALID_INPUT
            )
        out.append(fv)
    return tuple(out)


def _validate_gripper_position(position: int, *, name: str = "position") -> int:
    """校验夹爪位置：必须是 0–1000 的**整数**（bool 不算整数）。

    夹爪位置是**无量纲**的（不是角度）⇒ 这里**不做任何 rad/deg 换算**，
    换算实现只能有一份（在 :mod:`rm75_drake.device.sdk`）。
    """
    if isinstance(position, bool) or not isinstance(position, int):
        raise TransportError(
            f"{name} 必须是 {GRIPPER_POSITION_MIN}–{GRIPPER_POSITION_MAX} 之间的整数，"
            f"收到 {position!r}",
            reason=REASON_INVALID_INPUT,
        )
    if not (GRIPPER_POSITION_MIN <= position <= GRIPPER_POSITION_MAX):
        raise TransportError(
            f"{name} 必须在 {GRIPPER_POSITION_MIN}–{GRIPPER_POSITION_MAX} 之间，"
            f"收到 {position}",
            reason=REASON_INVALID_INPUT,
        )
    return int(position)


def _validate_gripper_speed(speed: int, *, name: str = "speed") -> int:
    """校验夹爪速度：必须是 0–100 的**整数**。"""
    if isinstance(speed, bool) or not isinstance(speed, int):
        raise TransportError(
            f"{name} 必须是 0–100 之间的整数，收到 {speed!r}",
            reason=REASON_INVALID_INPUT,
        )
    if not (0 <= speed <= 100):
        raise TransportError(f"{name} 必须在 0–100 之间，收到 {speed}", reason=REASON_INVALID_INPUT)
    return int(speed)


# ── 真机实现 ────────────────────────────────────────────────────────────
class SdkTransport(ArmTransport):
    """基于官方 SDK 的真机传输层（**只用白名单 SDK 符号**）。

    参数
    ----
    config:
        设备配置（IP / 端口等）。
    sdk:
        已加载的 SDK 命名空间；``None`` → :meth:`open` 时懒加载
        （缺失 → :class:`~rm75_drake.device.sdk.NotAvailableError`）。
    audit:
        调用审计计数器（运行时白名单审计，R008）。
    """

    name = "sdk_transport"

    def __init__(
        self,
        config: DeviceConfig,
        *,
        sdk: Any = None,
        audit: Optional[SdkAudit] = None,
        binding: Optional[ArmBinding] = None,
    ) -> None:
        super().__init__()
        self.config = config
        self.audit = audit if audit is not None else SdkAudit()
        self._sdk = sdk
        self._binding: Optional[ArmBinding] = binding
        self._owns_binding = binding is None
        self._arm: Any = None
        self._state_source: Any = None

    # -- 生命周期 --------------------------------------------------------
    def open(self) -> None:
        """建立连接（幂等）。

        步骤：三线程模式实例 → ``rm_create_robot_arm(ip, port)`` **建连接** →
        校验句柄（详见 :class:`~rm75_drake.device.binding.ArmBinding`）。

        异常
        ----
        :class:`~rm75_drake.device.sdk.NotAvailableError`
            SDK 未安装（**显式**报错，不静默降级）。
        :class:`TransportError`
            SDK 存在但连接失败（``reason="link_down"``）或结构不符
            （``reason="sdk_unavailable"``）。
        """
        if self._open:
            return
        binding = self._binding
        if binding is None:
            binding = ArmBinding(self.config, sdk=self._sdk, audit=self.audit)
        try:
            report = binding.open()
        except NotAvailableError:
            raise
        except Exception as exc:
            raise TransportError(
                f"SDK 初始化失败：{exc!r}", reason=REASON_SDK_UNAVAILABLE
            ) from exc
        if not report.connected:
            # 连接失败（含「替身没有建连接方法」）⇒ 明确报错，不假装已连上。
            reason = (
                REASON_LINK_DOWN
                if "句柄" in report.detail or "连接" in report.detail
                else REASON_SDK_UNAVAILABLE
            )
            raise TransportError(report.detail, reason=reason)
        self._binding = binding
        self._arm = binding.arm
        self._open = True
        self._closed = False

    def close(self) -> None:
        """关闭连接（幂等）。

        **共享绑定**（``binding=...`` 传入的）不会被关闭——它的生命周期归持有者。
        """
        self._open = False
        self._closed = True
        self._arm = None
        binding = self._binding
        if binding is None:
            return
        if self._owns_binding:
            self._binding = None
            binding.close()

    # -- 运动 ------------------------------------------------------------
    def move_j_canfd(self, joint_rad: Sequence[float], *, follow: bool = True) -> None:
        """透传下发关节角（rad → 度 的换算发生在**这一行之前**的唯一转换点）。"""
        self._require_open()
        values = _validate_joint_vector(joint_rad, "joint_rad")
        degrees = [rad_to_deg(v) for v in values]
        self._require_binding().call(
            "rm_movej_canfd", degrees, follow=bool(follow), trajectory_mode=0
        )

    def enable(self, state: bool) -> None:
        """整臂使能 / 去使能。

        真机实测：``rm_set_joint_en_state(joint_num, en_state)`` 是**逐轴**接口
        （``en_state`` 是 int：1 使能 / 0 去使能）⇒ 这里逐轴下发。
        """
        self._require_open()
        binding = self._require_binding()
        for index in range(JOINT_COUNT):
            binding.call(
                "rm_set_joint_en_state", index, 1 if state else 0
            )

    def clear_error(self) -> None:
        """清除逐关节错误码，再清系统错误码（官方顺序）。

        真机实测：``rm_set_joint_clear_err(joint_num)`` 是**逐轴**接口。
        """
        self._require_open()
        binding = self._require_binding()
        for index in range(JOINT_COUNT):
            binding.call("rm_set_joint_clear_err", index)
        binding.call("rm_clear_system_err")

    def emergency_stop(self) -> None:
        """急停。

        真机实测：``rm_set_arm_emergency_stop(state: bool)``。
        """
        self._require_open()
        self._require_binding().call("rm_set_arm_emergency_stop", True)

    def slow_stop(self) -> None:
        """缓停。"""
        self._require_open()
        self._require_binding().call("rm_set_arm_slow_stop")

    def set_joint_limits(
        self, lower_rad: Sequence[float], upper_rad: Sequence[float]
    ) -> None:
        """设置真机限位（rad → 度）。

        真机实测：``rm_set_joint_min_pos(joint_num, min_pos)`` /
        ``rm_set_joint_max_pos(joint_num, max_pos)`` 是**逐轴**接口。
        """
        self._require_open()
        binding = self._require_binding()
        lower = _validate_joint_vector(lower_rad, "lower_rad")
        upper = _validate_joint_vector(upper_rad, "upper_rad")
        for index in range(JOINT_COUNT):
            binding.call(
                "rm_set_joint_min_pos", index, rad_to_deg(lower[index])
            )
            binding.call(
                "rm_set_joint_max_pos", index, rad_to_deg(upper[index])
            )

    # -- 状态读取 --------------------------------------------------------
    def attach_state_source(self, source: Any) -> None:
        """挂载 UDP 状态源（:class:`~rm75_drake.device.state_source.StateSource`）。

        真机上 UDP 主动上报是**唯一**的状态回读通道：没有挂载状态源时
        :meth:`read_state` 一律返回 ``None``（**明确是「没有状态」**，
        不得当成「状态正常」）。
        """
        self._state_source = source

    def read_state(self) -> Optional[StateFrame]:
        """读取最新一帧状态（来自 UDP 回调缓存）。"""
        self._require_open()
        source = self._state_source
        if source is None:
            return None
        return source.latest()

    def read_arm_state(self) -> StateFrame:
        """**一次性只读查询**机械臂状态（``rm_get_current_arm_state``）。

        与 UDP 主动上报互补：UDP 帧里有逐轴电流 / 使能标志 / 错误码，
        而这个接口返回**控制器当前**的关节角与位姿（外参标定与围栏判定用）。

        异常
        ----
        :class:`TransportError`
            查询失败（``reason="link_down"``）或返回结构不符。
        """
        self._require_open()
        binding = self._require_binding()
        ret, raw = binding.call("rm_get_current_arm_state")
        if int(ret) != 0:
            raise TransportError(
                f"rm_get_current_arm_state 返回 {ret}（查询失败）",
                reason=REASON_LINK_DOWN,
            )
        return _state_frame_from_raw(raw)

    def read_joint_currents_ma(self) -> Tuple[float, ...]:
        """读取逐轴电流（mA，7 维）。"""
        self._require_open()
        ret, raw = self._require_binding().call("rm_get_current_joint_current")
        if int(ret) != 0:
            raise TransportError(
                f"rm_get_current_joint_current 返回 {ret}（查询失败）",
                reason=REASON_LINK_DOWN,
            )
        return tuple(float(v) for v in raw)

    def read_gripper(self) -> GripperState:
        """读取夹爪状态（``rm_get_gripper_state``）。"""
        self._require_open()
        raw = self._require_binding().call("rm_get_gripper_state")
        return _gripper_state_from_raw(raw)

    def command_gripper_position(self, position: int, *, speed: int = 100) -> None:
        """下发夹爪位置（``rm_set_gripper_position``）。

        * 夹爪位置是**无量纲** 0–1000 整数 ⇒ **不涉及 rad/deg 换算**；
        * ``rm_set_gripper_position`` 已在 SDK 白名单内（``device/sdk.py``）
          ⇒ **不改** ``sdk.py`` / ``versions.lock``；
        * 真机实测：签名是 ``(position, block, timeout)``——
          阻塞模式会等待控制器返回「夹爪到位」，超时返回 ``-4``。
        """
        self._require_open()
        value = _validate_gripper_position(position)
        _validate_gripper_speed(speed)
        self._require_binding().call(
            "rm_set_gripper_position", value, True, GRIPPER_BLOCK_TIMEOUT_S
        )

    # -- 内部工具 --------------------------------------------------------
    def _require_binding(self) -> ArmBinding:
        if self._binding is None:
            raise TransportError(
                "传输层未初始化 SDK 绑定（先调用 open()）", reason=REASON_NOT_OPEN
            )
        return self._binding


def _state_frame_from_raw(raw: Any) -> StateFrame:
    """把 ``rm_get_current_arm_state`` 的返回翻译成 :class:`StateFrame`。

    真机返回结构（实测）::

        {"joint": [deg, ...], "pose": [x, y, z, rx, ry, rz], "err": {"err_len": n, "err": [...]}}

    * ``joint`` 是**度** ⇒ 这里换算成 rad（唯一转换点 :func:`deg_to_rad`）；
    * ``err.err`` 是逐轴错误码（字符串，``"0"`` = 无错）；
    * UDP 帧才带电流 / 使能标志 ⇒ 这里保持默认（**不猜**）。
    """
    if not isinstance(raw, dict):
        raise TransportError(
            f"rm_get_current_arm_state 返回结构非法：{type(raw).__name__}",
            reason=REASON_INVALID_INPUT,
        )
    joints = raw.get("joint")
    if not isinstance(joints, (list, tuple)) or len(joints) != JOINT_COUNT:
        raise TransportError(
            f"rm_get_current_arm_state 的 joint 维度非法：{joints!r}",
            reason=REASON_INVALID_INPUT,
        )
    positions = tuple(deg_to_rad(float(v)) for v in joints)
    err_codes = [0] * JOINT_COUNT
    err = raw.get("err")
    if isinstance(err, dict):
        entries = err.get("err")
        if isinstance(entries, (list, tuple)):
            for index, item in enumerate(entries[:JOINT_COUNT]):
                try:
                    err_codes[index] = int(item)
                except (TypeError, ValueError):
                    err_codes[index] = -1
    return StateFrame(
        joint_positions_rad=positions,
        joint_err_codes=tuple(err_codes),
        received_at=time.monotonic(),
    )


def _gripper_state_from_raw(raw: Any) -> GripperState:
    """把 SDK 的夹爪状态翻译成 :class:`GripperState`。

    真机实测结构：``rm_get_gripper_state()`` 返回 ``(ret, dict)``，dict 的键是
    ``enable_state / status / error / mode / current_force / temperature / actpos``；
    ``status``：**0 = 离线，1 = 在线**（与 UDP 帧里的 ``rm_plus_state``
    「0 = 在线」语义相反 ⇒ 两处都必须按各自文档解释，**不得混用**）。
    """
    if raw is None:
        return GripperState(online=False, position=0)
    if isinstance(raw, GripperState):
        return raw
    payload: Any = raw
    if isinstance(raw, tuple) and len(raw) == 2 and isinstance(raw[1], dict):
        payload = raw[1]
    if isinstance(payload, dict):
        if "status" in payload:
            online = bool(int(payload.get("status", 0)) == 1)
        else:
            online = bool(int(payload.get("rm_plus_state", 0)) == 0)
        if "actpos" in payload:
            position = int(payload.get("actpos", 0) or 0)
        else:
            position = int(payload.get("gripper_position", 0) or 0)
        return GripperState(online=online, position=position)
    plus_state = getattr(raw, "rm_plus_state", None)
    online = True if plus_state is None else bool(int(plus_state) == 0)
    position = int(getattr(raw, "gripper_position", 0) or 0)
    return GripperState(online=online, position=position)


# ── 离线替身 ────────────────────────────────────────────────────────────
class ScriptedTransport(ArmTransport):
    """**离线替身**：可脚本化逐帧回放的传输层（零硬件）。

    脚本化能力（供安全闸 / 状态机 / 恢复流程做故障注入）：

    * :meth:`set_positions` / :meth:`set_currents` / :meth:`set_enable_flags`
      / :meth:`set_joint_error_codes` / :meth:`set_controller_status`
      / :meth:`set_arm_current_status` —— 改「下一帧模板」；
    * :meth:`push_frame` —— 推入一帧（可指定 ``received_at`` 制造**陈旧帧**）；
    * :attr:`link_down` —— 模拟**拔网线**（不再有状态帧、指令下发失败）；
    * :meth:`set_gripper_offline` —— 模拟**物体掉落 / 夹爪离线**；
    * :meth:`set_frame_incomplete` —— 模拟帧不完整（``errCode=-3``）；
    * :meth:`set_system_error` —— 模拟**帧级系统错误码**（如供电跌落）。
    """

    name = "scripted_transport"

    def __init__(self, frames: Optional[Sequence[StateFrame]] = None) -> None:
        super().__init__()
        self._template = _default_frame()
        self._frames: Deque[StateFrame] = deque()
        self.calls: List[Tuple[str, tuple]] = []
        self.link_down = False
        self.gripper_offline = False
        self._gripper_position = 0
        for frame in frames or ():
            self._frames.append(frame)

    # -- 生命周期 --------------------------------------------------------
    def open(self) -> None:
        """打开（替身无资源，幂等）。"""
        self._open = True
        self._closed = False

    def close(self) -> None:
        """关闭（幂等）。"""
        self._open = False
        self._closed = True

    # -- 脚本化 API ------------------------------------------------------
    def set_positions(self, joint_rad: Sequence[float]) -> None:
        """设置下一帧的关节角（rad）。"""
        self._template = _replace(self._template, joint_positions_rad=_validate_joint_vector(joint_rad, "joint_rad"))

    def set_currents(self, currents_ma: Sequence[float]) -> None:
        """设置下一帧的逐轴电流（mA）。"""
        self._template = _replace(self._template, joint_currents_ma=_validate_joint_vector(currents_ma, "currents_ma"))

    def set_enable_flags(self, flags: Sequence[bool]) -> None:
        """设置下一帧的逐轴使能标志。"""
        if len(flags) != JOINT_COUNT:
            raise TransportError(
                f"flags 必须是 {JOINT_COUNT} 维序列（实际 {len(flags)} 维）",
                reason=REASON_INVALID_INPUT,
            )
        self._template = _replace(self._template, joint_en_flags=tuple(bool(f) for f in flags))

    def set_joint_error_codes(self, codes: Sequence[int]) -> None:
        """设置下一帧的逐轴错误码（0 = 无错）。"""
        if len(codes) != JOINT_COUNT:
            raise TransportError(
                f"codes 必须是 {JOINT_COUNT} 维序列（实际 {len(codes)} 维）",
                reason=REASON_INVALID_INPUT,
            )
        self._template = _replace(self._template, joint_err_codes=tuple(int(c) for c in codes))

    def set_controller_status(self, status: int) -> None:
        """设置下一帧的控制器状态枚举。"""
        self._template = _replace(self._template, controller_status=int(status))

    def set_arm_current_status(self, status: int) -> None:
        """设置下一帧的机械臂状态枚举（0 IDLE / 5 透传 / 9 急停 / 10 缓停）。"""
        self._template = _replace(self._template, arm_current_status=int(status))

    def set_frame_incomplete(self, incomplete: bool = True) -> None:
        """设置下一帧是否**不完整**（``errCode=-3``）。

        交互提示
        --------
        * ``incomplete=True`` ⇒ ``errCode=-3``（帧不完整 → **必须丢弃**，
          绝不作为故障原因）；
        * ``incomplete=False`` ⇒ ``errCode`` **归零**，即**同时清掉**此前
          用 :meth:`set_system_error` 设置的帧级系统错误码（两者共用
          ``errCode`` 这一个字段）⇒ 恢复供电后用它复位是安全的，但会
          **抹掉**系统错误码，调用方须自己记录原始错误码。
        """
        self._template = _replace(
            self._template,
            err_code=FRAME_INCOMPLETE_ERR_CODE if incomplete else 0,
        )

    def set_system_error(self, code: int) -> None:
        """设置下一帧的**帧级系统错误码**（``errCode``）。

        语义
        ----
        * 非 0 且非 ``-3`` ⇒ **系统错误码**（本项目用它承载「供电跌落」
          这类物理故障；厂商错误码 ↔ 具体物理故障的对应关系在硬件可达前
          **未核实**，因此 ``power_supply_drop`` 是本项目自定的稳定 reason
          标签，不代表厂商错误码表）；
        * ``0`` ⇒ 无系统错误；
        * ``-3`` ⇒ 帧不完整标记（等价于 :meth:`set_frame_incomplete(True)`）。

        注意
        ----
        本方法只改「下一帧模板」；替身的 :meth:`read_state` 返回的是
        **最后一帧** ⇒ 必须真的 :meth:`push_frame` 才能让故障被下游看到。
        """
        if isinstance(code, bool) or not isinstance(code, int):
            raise TransportError(
                f"系统错误码必须是整数，收到 {code!r}",
                reason=REASON_INVALID_INPUT,
            )
        self._template = _replace(self._template, err_code=int(code))

    def set_gripper_offline(self, offline: bool = True) -> None:
        """模拟夹爪离线（物体掉落 / 夹爪反馈丢失）。"""
        self.gripper_offline = bool(offline)

    def set_gripper_position(self, position: int) -> None:
        """设置夹爪位置指令回读值（0–1000）。"""
        self._gripper_position = int(position)

    def push_frame(self, frame: Optional[StateFrame] = None) -> StateFrame:
        """推入一帧（``None`` → 用当前模板快照）。

        返回
        ----
        实际推入的帧。
        """
        snapshot = frame if frame is not None else self._template
        self._frames.append(snapshot)
        return snapshot

    def drop_link(self) -> None:
        """模拟**拔网线**：不再有状态帧，指令下发失败。"""
        self.link_down = True

    def restore_link(self) -> None:
        """恢复链路（拔网线故障的「恢复」侧）。"""
        self.link_down = False

    # -- 协议实现 --------------------------------------------------------
    def move_j_canfd(self, joint_rad: Sequence[float], *, follow: bool = True) -> None:
        """记录一次透传下发（断链时抛 :class:`TransportError`）。"""
        self._require_open()
        values = _validate_joint_vector(joint_rad, "joint_rad")
        if self.link_down:
            self.calls.append(("move_j_canfd", values))
            raise TransportError(
                "链路已断开，无法下发透传指令（模拟拔网线）", reason=REASON_LINK_DOWN
            )
        self.calls.append(("move_j_canfd", values))

    def enable(self, state: bool) -> None:
        """记录一次使能 / 去使能，并更新模板里的使能标志。"""
        self._require_open()
        self.calls.append(("enable", (bool(state),)))
        flags = tuple(bool(state) for _ in range(JOINT_COUNT))
        self._template = _replace(self._template, joint_en_flags=flags)

    def clear_error(self) -> None:
        """记录一次清错，并把逐轴错误码与帧级系统错误码**一起**清零。

        清什么 / 不清什么（语义边界）
        ----------------------------
        * **模板**：逐轴错误码与系统错误码都清零（对齐真机
          ``rm_set_joint_clear_err`` → ``rm_clear_system_err``）；
        * **最后一帧**：只清**系统错误码**（``errCode``）——因为真机
          ``rm_clear_system_err`` 作用于控制器，替身必须让 ``read_state()``
          看到「系统错误已被清除」，否则恢复流程的 ``resync_q`` /
          ``reset_ready`` 守卫读到的仍是带错误码的帧 ⇒ 恢复必然失败；
        * **最后一帧的逐轴错误码保持不变**（控制器是否真的清掉了逐轴错误，
          要由 ``resync_q`` 的「错误码未清零」守卫去判定 ⇒ 不得在替身里
          偷偷抹掉它，否则该守卫永远无法生效）；
        * **帧不完整标记（``-3``）永远不被清掉**：它不是错误码，而是
          「这一帧必须丢弃」的标记（清错不能把坏帧变成好帧）。

        因此这里**既清模板、也清最后一帧的系统错误码**。
        """
        self._require_open()
        self.calls.append(("clear_error", ()))
        self._template = _replace(
            self._template,
            joint_err_codes=(0,) * JOINT_COUNT,
            err_code=_cleared_err_code(self._template.err_code),
        )
        if self._frames:
            self._frames[-1] = _replace(
                self._frames[-1],
                err_code=_cleared_err_code(self._frames[-1].err_code),
            )

    def emergency_stop(self) -> None:
        """记录一次急停，并把机械臂状态置为急停（9）。"""
        self._require_open()
        self.calls.append(("emergency_stop", ()))
        self._template = _replace(self._template, arm_current_status=9)

    def slow_stop(self) -> None:
        """记录一次缓停，并把机械臂状态置为缓停（10）。"""
        self._require_open()
        self.calls.append(("slow_stop", ()))
        self._template = _replace(self._template, arm_current_status=10)

    def set_joint_limits(
        self, lower_rad: Sequence[float], upper_rad: Sequence[float]
    ) -> None:
        """记录一次限位写入。"""
        self._require_open()
        lower = _validate_joint_vector(lower_rad, "lower_rad")
        upper = _validate_joint_vector(upper_rad, "upper_rad")
        self.calls.append(("set_joint_limits", (lower, upper)))

    def read_state(self) -> Optional[StateFrame]:
        """返回最新一帧；断链或无帧时返回 ``None``。"""
        self._require_open()
        if self.link_down:
            return None
        if not self._frames:
            return None
        return self._frames[-1]

    def read_gripper(self) -> GripperState:
        """返回夹爪状态（可脚本化离线）。"""
        self._require_open()
        return GripperState(
            online=not self.gripper_offline, position=self._gripper_position
        )

    def command_gripper_position(self, position: int, *, speed: int = 100) -> None:
        """记录一次夹爪下发，并把回读值更新为该位置（离线「夹爪到位」语义）。

        注意：:meth:`set_gripper_position` 是**回读值设定器**（不记录调用），
        语义与本方法（**下发**）不同，不可混用。
        """
        self._require_open()
        value = _validate_gripper_position(position)
        _validate_gripper_speed(speed)
        self.calls.append(("command_gripper_position", (value,)))
        self._gripper_position = value

    # -- 观测 ------------------------------------------------------------
    def called(self, name: str) -> int:
        """返回某个动作被调用的次数（测试断言用）。"""
        return sum(1 for call, _ in self.calls if call == name)

    def call_names(self) -> List[str]:
        """返回全部动作名（按调用顺序）。"""
        return [call for call, _ in self.calls]


def _default_frame() -> StateFrame:
    """默认帧模板（7 维全零、无错、未使能）。"""
    return StateFrame(
        joint_positions_rad=(0.0,) * JOINT_COUNT,
        joint_currents_ma=(0.0,) * JOINT_COUNT,
        joint_en_flags=(False,) * JOINT_COUNT,
        joint_err_codes=(0,) * JOINT_COUNT,
        controller_status=0,
        gripper_online=True,
        arm_current_status=0,
        received_at=0.0,
        err_code=0,
    )


def _replace(frame: StateFrame, **changes: Any) -> StateFrame:
    """基于模板生成新帧（不可变数据类 ⇒ 用 ``dataclasses.replace`` 语义）。"""
    data = {
        "joint_positions_rad": frame.joint_positions_rad,
        "joint_currents_ma": frame.joint_currents_ma,
        "joint_en_flags": frame.joint_en_flags,
        "joint_err_codes": frame.joint_err_codes,
        "controller_status": frame.controller_status,
        "gripper_online": frame.gripper_online,
        "arm_current_status": frame.arm_current_status,
        "received_at": frame.received_at,
        "err_code": frame.err_code,
    }
    data.update(changes)
    return StateFrame(**data)
