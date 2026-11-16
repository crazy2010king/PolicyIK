"""UDP 状态源（S03-T03）：无锁环形缓冲 + 状态 age。

为什么要有状态源
----------------
研究 F5：真机的 **UDP 主动上报**一帧里已经含有状态机与安全闸所需的
**几乎全部信号**（逐轴使能标志、逐轴错误码、逐轴电流、关节角、
控制器状态枚举、夹爪在线标志、帧完整性错误码）。
⇒ 状态源是「UDP 回调线程」与「5 ms 执行环」之间的**唯一交接面**。

职责边界
--------
* 本模块**只**做「收帧 → 校验 → 存最近 N 帧 → 算 age」；
* 帧完整性校验（``errCode == -3`` = 推送数据不完整）→ **丢弃并计数**，
  不进入状态机（研究 F5 的硬要求）；
* 回调线程内**只做 memcpy + 时间戳**：禁止 I/O、禁止日志、禁止阻塞
  （见 :meth:`StateSource.push` 的纪律注释）。

线程模型
--------
* **单写者**：UDP 回调线程（1 个）；
* **多读者**：5 ms 执行环 / 状态机 / 运行日志（可多个）。

实现用**预分配槽位 + 整数游标**的环形缓冲：写入是「一次列表赋值 + 一次
整数赋值」，在 CPython 下由 GIL 保证原子性；读取是「一次整数读取 + 一次
列表读取」。因此**不需要锁**，也不会在 5 ms 环里引入阻塞点。

单位纪律
--------
关节角 **rad**、电流 **mA**、时间 **s**（单调时钟）。
本模块**不实现** rad↔deg 换算（唯一转换点在 :mod:`rm75_drake.device.sdk`）。
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional

from ..planning.trajectory import JOINT_COUNT
from .transport import FRAME_INCOMPLETE_ERR_CODE as _FRAME_INCOMPLETE_ERR_CODE
from .transport import StateFrame

__all__ = [
    "DEFAULT_CAPACITY",
    "REASON_FRAME_INCOMPLETE",
    "REASON_INVALID_FRAME",
    "MISSED_CYCLES_UNBOUNDED",
    "StateSourceError",
    "StateSample",
    "FrameRing",
    "StateSource",
]


#: 环形缓冲默认容量（最近 N 帧）。
DEFAULT_CAPACITY = 64

#: 帧完整性错误码：``errCode == -3`` 表示「推送数据不完整」→ 帧必须丢弃。
#: （真相源在 :mod:`rm75_drake.device.transport`，此处**重导出**保持旧导入路径可用。）
FRAME_INCOMPLETE_ERR_CODE = _FRAME_INCOMPLETE_ERR_CODE

#: 稳定 reason 标签。
REASON_FRAME_INCOMPLETE = "frame_incomplete"
REASON_INVALID_FRAME = "invalid_frame"

#: 「从未收到过帧」时 :meth:`StateSource.missed_cycles` 的返回值
#: （age 为无穷 ⇒ 缺失周期数无界；用一个显式哨兵值代替 ``inf``）。
MISSED_CYCLES_UNBOUNDED = 1_000_000_000


class StateSourceError(ValueError):
    """状态源错误（带稳定 ``reason`` 标签）。

    属性
    ----
    reason:
        稳定标签（``frame_incomplete`` / ``invalid_frame``）。
    """

    def __init__(self, message: str, *, reason: str = REASON_INVALID_FRAME) -> None:
        super().__init__(message)
        self.reason = reason


@dataclass(frozen=True)
class StateSample:
    """一帧状态 + 它**到达**本进程的时刻。

    字段
    ----
    frame:
        状态帧本身（不可变数据类）。
    arrival_time_s:
        到达时刻（**单调时钟**，秒）——``state_age`` 的基准。
    """

    frame: StateFrame
    arrival_time_s: float


class FrameRing:
    """**无锁**环形缓冲（预分配槽位 + 整数游标）。

    参数
    ----
    capacity:
        槽位数（≥ 1）。
    """

    __slots__ = ("_capacity", "_slots", "_write_index", "_count")

    def __init__(self, capacity: int = DEFAULT_CAPACITY) -> None:
        if isinstance(capacity, bool) or not isinstance(capacity, int):
            raise StateSourceError(
                f"capacity 必须是整数，收到 {capacity!r}", reason=REASON_INVALID_FRAME
            )
        if capacity < 1:
            raise StateSourceError(
                f"capacity 必须 ≥ 1，收到 {capacity}", reason=REASON_INVALID_FRAME
            )
        self._capacity = capacity
        self._slots: List[Any] = [None] * capacity
        self._write_index = 0
        self._count = 0

    @property
    def capacity(self) -> int:
        """槽位总数。"""
        return self._capacity

    def __len__(self) -> int:
        """当前存放的样本数（≤ ``capacity``）。"""
        return self._count

    def push(self, item: Any) -> None:
        """写入一个样本（**覆写**最老的那个槽位）。"""
        index = self._write_index + 1
        if index >= self._capacity:
            index = 0
        self._slots[index] = item
        self._write_index = index
        if self._count < self._capacity:
            self._count += 1

    def latest(self) -> Optional[Any]:
        """返回最新样本；空缓冲返回 ``None``。"""
        if self._count == 0:
            return None
        return self._slots[self._write_index]

    def to_list(self) -> List[Any]:
        """返回全部样本，**从最老到最新**（便于测试与诊断）。"""
        out: List[Any] = []
        start = self._write_index - self._count + 1
        for offset in range(self._count):
            index = start + offset
            if index < 0:
                index += self._capacity
            out.append(self._slots[index])
        return out


class StateSource:
    """UDP 状态源：收帧 → 校验 → 环形缓冲 → 状态 age。

    参数
    ----
    capacity:
        环形缓冲容量（最近 N 帧）。
    clock:
        单调时钟（默认 :func:`time.monotonic`）；测试可注入确定性时钟。
    """

    def __init__(
        self,
        capacity: int = DEFAULT_CAPACITY,
        *,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._ring = FrameRing(capacity)
        self._clock = clock
        self._frames_received = 0
        self._frames_rejected_incomplete = 0
        self._frames_rejected_invalid = 0

    # -- 统计 ------------------------------------------------------------
    @property
    def capacity(self) -> int:
        """环形缓冲容量。"""
        return self._ring.capacity

    @property
    def frames_received(self) -> int:
        """**有效**帧计数（通过完整性校验并入缓冲的帧数）。"""
        return self._frames_received

    @property
    def frames_rejected_incomplete(self) -> int:
        """因 ``errCode == -3`` 被丢弃的帧计数。"""
        return self._frames_rejected_incomplete

    @property
    def frames_rejected_invalid(self) -> int:
        """因字段非法被丢弃的帧计数。"""
        return self._frames_rejected_invalid

    def stats(self) -> Dict[str, int]:
        """返回统计快照（写进运行日志）。"""
        return {
            "frames_received": self._frames_received,
            "frames_rejected_incomplete": self._frames_rejected_incomplete,
            "frames_rejected_invalid": self._frames_rejected_invalid,
            "buffered_frames": len(self._ring),
            "capacity": self._ring.capacity,
        }

    # -- 写入（回调线程）------------------------------------------------
    def push(
        self, frame: StateFrame, *, now: Optional[float] = None
    ) -> bool:
        """收一帧（**UDP 回调线程**调用）。

        纪律
        ----
        本方法内**只做 memcpy + 时间戳**：不写日志、不做 I/O、不阻塞。

        返回
        ----
        ``True``
            帧有效并已入缓冲；
        ``False``
            帧被丢弃（``errCode == -3`` 不完整帧）。

        异常
        ----
        :class:`StateSourceError`
            帧字段非法（维度错 / NaN / 类型错）——**显式冒泡**，
            不静默丢弃（避免把坏数据伪装成「没收到帧」）。
        """
        if not isinstance(frame, StateFrame):
            raise StateSourceError(
                f"frame 必须是 StateFrame，收到 {type(frame).__name__}",
                reason=REASON_INVALID_FRAME,
            )
        try:
            _validate_frame(frame)
        except StateSourceError:
            # 坏数据：计数后**显式冒泡**（不静默伪装成「没收到帧」）。
            self._frames_rejected_invalid += 1
            raise

        if int(frame.err_code) == FRAME_INCOMPLETE_ERR_CODE:
            # 推送数据不完整 → 丢弃并计数（**不进入状态机**）。
            self._frames_rejected_incomplete += 1
            return False

        arrival = self._clock() if now is None else float(now)
        # memcpy：把可变序列拷成不可变元组，避免调用方后续改动污染缓冲。
        copied = StateFrame(
            joint_positions_rad=tuple(float(v) for v in frame.joint_positions_rad),
            joint_currents_ma=tuple(float(v) for v in frame.joint_currents_ma),
            joint_en_flags=tuple(bool(v) for v in frame.joint_en_flags),
            joint_err_codes=tuple(int(v) for v in frame.joint_err_codes),
            controller_status=int(frame.controller_status),
            gripper_online=bool(frame.gripper_online),
            arm_current_status=int(frame.arm_current_status),
            received_at=float(arrival),
            err_code=int(frame.err_code),
        )
        self._ring.push(StateSample(frame=copied, arrival_time_s=arrival))
        self._frames_received += 1
        return True

    # -- 读取（执行环 / 状态机）-----------------------------------------
    def latest(self) -> Optional[StateFrame]:
        """返回最新一帧状态；无帧时返回 ``None``。"""
        sample = self._ring.latest()
        return None if sample is None else sample.frame

    def latest_sample(self) -> Optional[StateSample]:
        """返回最新样本（帧 + 到达时刻）。"""
        return self._ring.latest()

    def frames(self) -> List[StateFrame]:
        """返回缓冲里的全部帧（**从最老到最新**）。"""
        return [sample.frame for sample in self._ring.to_list()]

    def state_age_s(self, now: Optional[float] = None) -> float:
        """返回最新帧的 age（s）：``now − 到达时刻``。

        从未收到过帧 ⇒ 返回 ``math.inf``（明确表示「无数据」，
        而不是伪造一个 0）。
        """
        sample = self._ring.latest()
        if sample is None:
            return math.inf
        current = self._clock() if now is None else float(now)
        age = current - sample.arrival_time_s
        return age if age >= 0.0 else 0.0

    def missed_cycles(
        self, cycle_period_s: float = 0.005, *, now: Optional[float] = None
    ) -> int:
        """返回心跳缺失的周期数（``age / cycle_period_s``）。

        从未收到过帧 ⇒ 返回 :data:`MISSED_CYCLES_UNBOUNDED`（哨兵值）。
        """
        period = float(cycle_period_s)
        if not math.isfinite(period) or period <= 0.0:
            raise StateSourceError(
                f"cycle_period_s 必须是正有限数值，实际是 {cycle_period_s!r}",
                reason=REASON_INVALID_FRAME,
            )
        age = self.state_age_s(now=now)
        if math.isinf(age):
            return MISSED_CYCLES_UNBOUNDED
        return int(age / period)

    def is_stale(self, threshold_s: float, *, now: Optional[float] = None) -> bool:
        """判定状态帧是否**超时未更新**（``age > threshold``）。

        ``threshold_s`` 必须为正有限数值（否则显式报错）。
        """
        threshold = float(threshold_s)
        if not math.isfinite(threshold) or threshold <= 0.0:
            raise StateSourceError(
                f"threshold_s 必须是正有限数值，实际是 {threshold_s!r}",
                reason=REASON_INVALID_FRAME,
            )
        return self.state_age_s(now=now) > threshold


def _validate_frame(frame: StateFrame) -> None:
    """校验帧字段（维度 / 有限性）。

    失败一律抛 :class:`StateSourceError`（点名字段）。
    """
    vectors = (
        ("joint_positions_rad", frame.joint_positions_rad),
        ("joint_currents_ma", frame.joint_currents_ma),
    )
    for name, values in vectors:
        if len(values) != JOINT_COUNT:
            raise StateSourceError(
                f"{name} 必须是 {JOINT_COUNT} 维，实际是 {len(values)} 维",
                reason=REASON_INVALID_FRAME,
            )
        for index, value in enumerate(values):
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise StateSourceError(
                    f"{name}[{index}] 必须是数值，收到 {value!r}",
                    reason=REASON_INVALID_FRAME,
                )
            if not math.isfinite(float(value)):
                raise StateSourceError(
                    f"{name}[{index}] 必须是有限数值，收到 {value!r}",
                    reason=REASON_INVALID_FRAME,
                )
    for name, values in (
        ("joint_en_flags", frame.joint_en_flags),
        ("joint_err_codes", frame.joint_err_codes),
    ):
        if len(values) != JOINT_COUNT:
            raise StateSourceError(
                f"{name} 必须是 {JOINT_COUNT} 维，实际是 {len(values)} 维",
                reason=REASON_INVALID_FRAME,
            )
    if not isinstance(frame.controller_status, int) or isinstance(
        frame.controller_status, bool
    ):
        raise StateSourceError(
            f"controller_status 必须是整数，收到 {frame.controller_status!r}",
            reason=REASON_INVALID_FRAME,
        )
    if not isinstance(frame.arm_current_status, int) or isinstance(
        frame.arm_current_status, bool
    ):
        raise StateSourceError(
            f"arm_current_status 必须是整数，收到 {frame.arm_current_status!r}",
            reason=REASON_INVALID_FRAME,
        )
