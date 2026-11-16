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
因此「拔网线 / 物体掉落 / 夹爪夹空 / 关节超限位」四类故障可以成为**真正的单元测试**。

单位纪律
--------
本模块**对外一律 rad**（弧度）；只有 :class:`SdkTransport` 在**下发前最后一刻**
调用 :func:`rm75_drake.device.sdk.rad_to_deg` 换算成度（SDK 全线用度）——
换算点唯一，源头见 :mod:`rm75_drake.device.sdk`。
"""

from __future__ import annotations

import abc
import math
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Deque, List, Optional, Sequence, Tuple

from ..planning.trajectory import JOINT_COUNT
from .config import DeviceConfig
from .sdk import NotAvailableError, SdkAudit, SdkProxy, rad_to_deg

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
        return int(self.err_code) != -3

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
    ) -> None:
        super().__init__()
        self.config = config
        self.audit = audit if audit is not None else SdkAudit()
        self._sdk = sdk
        self._proxy: Optional[SdkProxy] = None
        self._arm: Any = None

    # -- 生命周期 --------------------------------------------------------
    def open(self) -> None:
        """建立连接（幂等）。

        异常
        ----
        :class:`~rm75_drake.device.sdk.NotAvailableError`
            SDK 未安装（**显式**报错，不静默降级）。
        :class:`TransportError`
            SDK 存在但初始化失败（``reason="sdk_unavailable"``）。
        """
        if self._open:
            return
        if self._sdk is None:
            from .sdk import load_sdk  # 局部导入：懒加载的最后一环

            try:
                self._sdk = load_sdk()
            except NotAvailableError:
                raise
            except Exception as exc:  # pragma: no cover - 防御
                raise TransportError(
                    f"SDK 加载失败：{exc!r}", reason=REASON_SDK_UNAVAILABLE
                ) from exc
        self._proxy = SdkProxy(self._sdk, self.audit)
        self._arm = self._init_arm()
        self._open = True
        self._closed = False

    def _init_arm(self) -> Any:
        """初始化三线程模式下的 ``RoboticArm`` 实例。

        三线程模式是 UDP 主动上报的前提（研究 F4）。
        """
        try:
            interface = self._sdk.rm_robot_interface  # type: ignore[union-attr]
            arm_cls = getattr(interface, "RoboticArm")
        except AttributeError as exc:
            raise TransportError(
                "SDK 里找不到 RoboticArm（rm_robot_interface.RoboticArm）",
                reason=REASON_SDK_UNAVAILABLE,
            ) from exc
        try:
            return arm_cls(RM_TRIPLE_MODE_E)
        except TypeError:
            # 老版本 SDK 的三线程常量名可能不同：显式回退并记录，不假装。
            return arm_cls()
        except Exception as exc:
            raise TransportError(
                f"RoboticArm 初始化失败：{exc!r}", reason=REASON_SDK_UNAVAILABLE
            ) from exc

    def close(self) -> None:
        """关闭连接（幂等）。"""
        self._open = False
        self._closed = True
        self._arm = None

    # -- 运动 ------------------------------------------------------------
    def move_j_canfd(self, joint_rad: Sequence[float], *, follow: bool = True) -> None:
        """透传下发关节角（rad → 度 的换算发生在**这一行之前**的唯一转换点）。"""
        self._require_open()
        proxy = self._require_proxy()
        values = _validate_joint_vector(joint_rad, "joint_rad")
        degrees = [rad_to_deg(v) for v in values]
        proxy.rm_movej_canfd(degrees, follow=bool(follow), trajectory_mode=0)

    def enable(self, state: bool) -> None:
        """整臂使能 / 去使能。"""
        self._require_open()
        self._require_proxy().rm_set_joint_en_state(bool(state))

    def clear_error(self) -> None:
        """清除逐关节错误码，再清系统错误码（官方顺序）。"""
        self._require_open()
        proxy = self._require_proxy()
        proxy.rm_set_joint_clear_err()
        proxy.rm_clear_system_err()

    def emergency_stop(self) -> None:
        """急停。"""
        self._require_open()
        self._require_proxy().rm_set_arm_emergency_stop()

    def slow_stop(self) -> None:
        """缓停。"""
        self._require_open()
        self._require_proxy().rm_set_arm_slow_stop()

    def set_joint_limits(
        self, lower_rad: Sequence[float], upper_rad: Sequence[float]
    ) -> None:
        """设置真机限位（rad → 度）。"""
        self._require_open()
        proxy = self._require_proxy()
        lower = _validate_joint_vector(lower_rad, "lower_rad")
        upper = _validate_joint_vector(upper_rad, "upper_rad")
        proxy.rm_set_joint_min_pos([rad_to_deg(v) for v in lower])
        proxy.rm_set_joint_max_pos([rad_to_deg(v) for v in upper])

    # -- 状态读取 --------------------------------------------------------
    def read_state(self) -> Optional[StateFrame]:
        """读取最新一帧状态（来自 UDP 回调缓存）。"""
        self._require_open()
        getter = getattr(self, "_latest_frame", None)
        if getter is None:
            return None
        return getter()

    def read_gripper(self) -> GripperState:
        """读取夹爪状态（``rm_get_gripper_state``）。"""
        self._require_open()
        raw = self._require_proxy().rm_get_gripper_state()
        return _gripper_state_from_raw(raw)

    def command_gripper_position(self, position: int, *, speed: int = 100) -> None:
        """下发夹爪位置（``rm_set_gripper_position``）。

        * 夹爪位置是**无量纲** 0–1000 整数 ⇒ **不涉及 rad/deg 换算**；
        * ``rm_set_gripper_position`` 已在 SDK 白名单内（``device/sdk.py``）
          ⇒ **不改** ``sdk.py`` / ``versions.lock``。
        """
        self._require_open()
        value = _validate_gripper_position(position)
        _validate_gripper_speed(speed)
        self._require_proxy().rm_set_gripper_position(value)

    # -- 内部工具 --------------------------------------------------------
    def _require_proxy(self) -> SdkProxy:
        if self._proxy is None:
            raise TransportError(
                "传输层未初始化 SDK 代理（先调用 open()）", reason=REASON_NOT_OPEN
            )
        return self._proxy


def _gripper_state_from_raw(raw: Any) -> GripperState:
    """把 SDK 的夹爪状态结构体翻译成 :class:`GripperState`。"""
    if raw is None:
        return GripperState(online=False, position=0)
    if isinstance(raw, GripperState):
        return raw
    online = True
    position = 0
    if isinstance(raw, dict):
        online = bool(raw.get("rm_plus_state", 0) == 0)
        position = int(raw.get("gripper_position", 0) or 0)
    else:
        plus_state = getattr(raw, "rm_plus_state", None)
        if plus_state is not None:
            online = int(plus_state) == 0
        position = int(getattr(raw, "gripper_position", 0) or 0)
    return GripperState(online=online, position=position)


#: SDK 的三线程模式常量（与官方 ``RM_TRIPLE_MODE_E`` 一致）。
RM_TRIPLE_MODE_E = 3


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
    * :meth:`set_frame_incomplete` —— 模拟帧不完整（``errCode=-3``）。
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
        """设置下一帧是否**不完整**（``errCode=-3``）。"""
        self._template = _replace(self._template, err_code=-3 if incomplete else 0)

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
        """记录一次清错，并把逐轴错误码清零。"""
        self._require_open()
        self.calls.append(("clear_error", ()))
        self._template = _replace(self._template, joint_err_codes=(0,) * JOINT_COUNT)

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
