"""UDP 状态主动上报（真机唯一的**状态回读**通道）。

为什么要有这个模块
------------------
设计文档 §八.3 的巨坑清单写明：「状态回读滞后 ⇒ 未开 UDP 主动上报 /
cycle 非 5 的倍数」。真机实测（RM75-B / 192.168.1.18 / 控制器代次 3）：

* ``cycle`` 的单位是 **5 ms 的倍数**（读法 A）：
  ``cycle=1`` ⇒ 实测上报周期 **P50 = 4.998 ms**；``cycle=5`` ⇒ **P50 = 24.954 ms**。
  （读法 B「单位是毫秒」被实测**否证**。）
* 主动上报只在**三线程模式**（``RM_TRIPLE_MODE_E = 2``）下可用；
* 上报目标 IP 必须是**开发机在机械臂网段上的地址**，不是机械臂自己的 IP；
* 帧 ``errCode == -3`` 表示「推送数据不完整」⇒ 该帧**必须丢弃**
  （判定与丢弃的唯一真相源在 :mod:`rm75_drake.device.state_source`）。

职责边界
--------
* 本模块只做「配置主动上报 + 注册回调 + 解码成 :class:`StateFrame`」；
* 收帧后交给 :class:`~rm75_drake.device.state_source.StateSource`
  （环形缓冲 + age + 完整性校验）；
* **不**做任何运动学 / 规划（那是 Drake 层的职责）。

单位纪律
--------
关节角 **rad**（SDK 帧里是度 ⇒ 这里换算，唯一转换点
:func:`rm75_drake.device.sdk.deg_to_rad`）、电流 **mA**、时间 **s**（单调时钟）。
"""

from __future__ import annotations

import socket
import time
from collections import deque
from dataclasses import dataclass
from typing import Any, Deque, Dict, List, Optional, Sequence, Tuple

from ..planning.trajectory import JOINT_COUNT
from .binding import ArmBinding
from .sdk import deg_to_rad
from .state_source import StateSource

__all__ = [
    "REALTIME_PUSH_CYCLE_UNIT_MS",
    "detect_local_ip",
    "RealtimePushSettings",
    "RealtimePeriodStats",
    "decode_realtime_state",
    "RealtimeStateReceiver",
]


#: ``rm_set_realtime_push`` 的 ``cycle`` 单位（ms）。
#:
#: 真机实测标定结论：``cycle`` 的单位是 **5 ms 的倍数**（读法 A）——
#: ``cycle=1`` ⇒ 实测 4.998 ms；``cycle=5`` ⇒ 实测 24.954 ms。
#: ⇒ 期望上报周期（ms）= ``cycle × 5``。
REALTIME_PUSH_CYCLE_UNIT_MS = 5.0


def detect_local_ip(target_ip: str, target_port: int = 8080) -> str:
    """探测本机在 ``target_ip`` 所在网段上的**源地址**。

    做法：建一个 UDP socket 并 ``connect`` 到机械臂（不真正发包），
    再读 ``getsockname()`` ⇒ 内核选出的源地址就是机械臂看到的本机地址。
    这是 UDP 主动上报**必填**的「上报目标 IP」（填机械臂自己的 IP 是经典坑）。

    异常
    ----
    OSError
        探测失败（调用方应显式报错，不静默兜底）。
    """
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect((str(target_ip), int(target_port)))
        return str(probe.getsockname()[0])
    finally:
        probe.close()


@dataclass(frozen=True)
class RealtimePushSettings:
    """UDP 主动上报配置。

    字段
    ----
    ip:
        **上报目标 IP**（开发机在机械臂网段上的地址；**不是**机械臂自己的 IP）。
    port:
        上报端口（开发机本地监听端口）。
    cycle:
        上报周期参数（单位见 :data:`REALTIME_PUSH_CYCLE_UNIT_MS`）。
    enable:
        是否开启主动上报（关闭时用它复位）。
    force_coordinate:
        外受力数据坐标系（无 F/T 传感器时保持 ``-1``）。
    """

    ip: str
    port: int
    cycle: int = 1
    enable: bool = True
    force_coordinate: int = -1

    def expected_period_ms(self) -> float:
        """按实测口径返回期望上报周期（ms）。"""
        return float(self.cycle) * REALTIME_PUSH_CYCLE_UNIT_MS

    def to_dict(self) -> Dict[str, Any]:
        """返回可 JSON 序列化的字典。"""
        return {
            "ip": self.ip,
            "port": int(self.port),
            "cycle": int(self.cycle),
            "enable": bool(self.enable),
            "force_coordinate": int(self.force_coordinate),
            "expected_period_ms": self.expected_period_ms(),
        }


@dataclass(frozen=True)
class RealtimePeriodStats:
    """上报周期分布（ms）。

    字段
    ----
    frames:
        收到的帧数。
    p50 / p99 / p999 / max:
        周期分位数（ms）。
    min:
        最小周期（ms；突发帧会拉低它，**不能**当典型值）。
    """

    frames: int
    p50: float
    p99: float
    p999: float
    max: float
    min: float

    def to_dict(self) -> Dict[str, Any]:
        """返回可 JSON 序列化的字典。"""
        return {
            "frames": int(self.frames),
            "p50_ms": float(self.p50),
            "p99_ms": float(self.p99),
            "p999_ms": float(self.p999),
            "max_ms": float(self.max),
            "min_ms": float(self.min),
        }


def decode_realtime_state(raw: Any) -> "RealtimeFrame":
    """把 SDK 推送结构体解码成中性帧（**不引入 SDK 依赖**）。

    真机实测结构（``rm_realtime_arm_joint_state_t``）::

        errCode: int
        joint_status: {joint_position(deg), joint_current(mA),
                       joint_en_flag(bool), joint_err_code(int)}
        arm_current_status: int
        rm_plus_state: int        # 0 = 末端设备在线

    关节角是**度** ⇒ 这里换算成 rad（唯一转换点）。
    """
    err_code = int(getattr(raw, "errCode", 0))
    joint_status = getattr(raw, "joint_status", None)
    positions: List[float] = []
    currents: List[float] = []
    enabled: List[bool] = []
    err_codes: List[int] = []
    if joint_status is not None:
        for value in _as_list(getattr(joint_status, "joint_position", None)):
            positions.append(deg_to_rad(float(value)))
        for value in _as_list(getattr(joint_status, "joint_current", None)):
            currents.append(float(value))
        for value in _as_list(getattr(joint_status, "joint_en_flag", None)):
            enabled.append(bool(value))
        for value in _as_list(getattr(joint_status, "joint_err_code", None)):
            err_codes.append(int(value))
    plus_state = getattr(raw, "rm_plus_state", None)
    gripper_online = True if plus_state is None else bool(int(plus_state) == 0)
    return RealtimeFrame(
        joint_positions_rad=tuple(positions[:JOINT_COUNT]),
        joint_currents_ma=tuple(currents[:JOINT_COUNT]),
        joint_en_flags=tuple(enabled[:JOINT_COUNT]),
        joint_err_codes=tuple(err_codes[:JOINT_COUNT]),
        arm_current_status=int(getattr(raw, "arm_current_status", 0)),
        gripper_online=gripper_online,
        err_code=err_code,
        received_at=time.monotonic(),
    )


#: **进程级**回调保活表。
#:
#: ⚠️ 真机实测：厂商 SDK 把回调当作**裸函数指针**保存；若 Python 侧的
#: ``CFUNCTYPE`` 对象被 GC，SDK 的 UDP 线程再回调一次就是**野指针**
#: ⇒ 随机 ``Segmentation fault``（崩在任意位置，连报告都写不出来）。
#: ⇒ 回调对象放进这个模块级表里，**直到进程退出都不释放**。
_LIVE_CALLBACKS: List[Any] = []


@dataclass(frozen=True)
class RealtimeFrame:
    """中性化的上报帧（不依赖 SDK 类型）。"""

    joint_positions_rad: Tuple[float, ...]
    joint_currents_ma: Tuple[float, ...]
    joint_en_flags: Tuple[bool, ...]
    joint_err_codes: Tuple[int, ...]
    arm_current_status: int
    gripper_online: bool
    err_code: int
    received_at: float

    def to_state_frame(self) -> Any:
        """转换成 :class:`~rm75_drake.device.transport.StateFrame`。"""
        from .transport import StateFrame

        return StateFrame(
            joint_positions_rad=self.joint_positions_rad,
            joint_currents_ma=self.joint_currents_ma,
            joint_en_flags=self.joint_en_flags,
            joint_err_codes=self.joint_err_codes,
            arm_current_status=self.arm_current_status,
            gripper_online=self.gripper_online,
            received_at=self.received_at,
            err_code=self.err_code,
        )


def _as_list(value: Any) -> Sequence[Any]:
    """把 ctypes 数组 / 列表统一成序列。"""
    if value is None:
        return ()
    if isinstance(value, (list, tuple)):
        return value
    try:
        return list(value)
    except TypeError:  # pragma: no cover - 防御
        return ()


class RealtimeStateReceiver:
    """UDP 主动上报接收器（配置 + 回调 + 解码 + 周期统计）。

    参数
    ----
    binding:
        已连接的 :class:`~rm75_drake.device.binding.ArmBinding`。
    source:
        状态源（:class:`~rm75_drake.device.state_source.StateSource`）；
        收到的帧会推进去。
    settings:
        上报配置（目标 IP / 端口 / 周期）。
    history:
        到达时刻历史长度（默认 4096，用于周期统计）。
    """

    def __init__(
        self,
        binding: ArmBinding,
        source: StateSource,
        settings: RealtimePushSettings,
        *,
        history: int = 4096,
    ) -> None:
        self._binding = binding
        self._source = source
        self._settings = settings
        self._arrivals: Deque[float] = deque(maxlen=int(history))
        self._callback: Any = None
        self._started = False
        self._config_type: Any = None

    # -- 属性 ------------------------------------------------------------
    @property
    def settings(self) -> RealtimePushSettings:
        """当前上报配置。"""
        return self._settings

    @property
    def started(self) -> bool:
        """是否已开启主动上报。"""
        return self._started

    @property
    def arrivals(self) -> Tuple[float, ...]:
        """到达时刻序列（单调时钟）。"""
        return tuple(self._arrivals)

    # -- 生命周期 --------------------------------------------------------
    def start(self) -> None:
        """开启主动上报（注册回调 + 下发配置）。

        异常
        ----
        RuntimeError
            SDK 里缺少必要的类型 / 符号（显式报错，不静默跳过）。
        """
        if self._started:
            return
        rm_interface = self._binding.interface
        callback_type = getattr(rm_interface, "rm_realtime_arm_state_callback_ptr", None)
        state_type = getattr(rm_interface, "rm_realtime_arm_joint_state_t", None)
        config_type = getattr(rm_interface, "rm_realtime_push_config_t", None)
        if callback_type is None or state_type is None or config_type is None:
            raise RuntimeError(
                "SDK 里找不到 UDP 主动上报相关类型"
                "（rm_realtime_arm_state_callback_ptr / rm_realtime_arm_joint_state_t / "
                "rm_realtime_push_config_t）"
            )
        self._config_type = config_type

        def _handler(data: Any) -> None:
            self._on_frame(data)

        self._callback = callback_type(_handler)
        _LIVE_CALLBACKS.append(self._callback)
        rm_interface.rm_realtime_arm_state_call_back(self._callback)
        self._apply_push(enable=True)
        self._started = True

    def stop(self) -> None:
        """关闭主动上报（幂等；失败只记录不抛）。"""
        if not self._started:
            return
        self._started = False
        try:
            self._apply_push(enable=False)
        except Exception:  # pragma: no cover - 收尾尽力而为
            pass
        self._callback = None

    # -- 统计 ------------------------------------------------------------
    def period_stats(self) -> RealtimePeriodStats:
        """返回上报周期分布（ms）。"""
        arrivals = list(self._arrivals)
        if len(arrivals) < 2:
            return RealtimePeriodStats(
                frames=len(arrivals), p50=0.0, p99=0.0, p999=0.0, max=0.0, min=0.0
            )
        deltas = sorted(
            (arrivals[i + 1] - arrivals[i]) * 1000.0
            for i in range(len(arrivals) - 1)
        )
        return RealtimePeriodStats(
            frames=len(arrivals),
            p50=_percentile(deltas, 0.50),
            p99=_percentile(deltas, 0.99),
            p999=_percentile(deltas, 0.999),
            max=deltas[-1],
            min=deltas[0],
        )

    # -- 内部工具 --------------------------------------------------------
    def _apply_push(self, *, enable: bool) -> None:
        """下发主动上报配置。"""
        settings = self._settings
        config = self._config_type(
            cycle=int(settings.cycle),
            enable=bool(enable),
            port=int(settings.port),
            force_coordinate=int(settings.force_coordinate),
            ip=str(settings.ip),
        )
        ret = self._binding.call("rm_set_realtime_push", config)
        if int(ret) != 0:
            raise RuntimeError(f"rm_set_realtime_push 返回 {ret}（配置失败）")

    def _on_frame(self, data: Any) -> None:
        """回调（SDK 的 UDP 接收线程）：只做「解码 + 记时刻 + 入缓冲」。"""
        frame = decode_realtime_state(data)
        self._arrivals.append(frame.received_at)
        try:
            self._source.push(frame.to_state_frame())
        except Exception:  # pragma: no cover - 坏帧不打断接收线程
            pass


def _percentile(sorted_values: Sequence[float], fraction: float) -> float:
    """分位数（线性插值，输入必须已排序）。"""
    if not sorted_values:
        return 0.0
    if len(sorted_values) == 1:
        return float(sorted_values[0])
    position = fraction * (len(sorted_values) - 1)
    lower = int(position)
    upper = min(lower + 1, len(sorted_values) - 1)
    weight = position - lower
    return float(sorted_values[lower] * (1.0 - weight) + sorted_values[upper] * weight)
