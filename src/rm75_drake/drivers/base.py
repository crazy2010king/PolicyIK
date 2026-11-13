"""驱动抽象层（S02-T07）：仿真 / 假驱动 / 真机共用的**同一套接口**。

职责边界
--------
本模块定义**驱动协议**（:class:`Driver`）与**状态数据结构**
（:class:`DriverState`），并提供「驱动不得修改轨迹内容」的**自检钩子**。

为什么要有这一层（R007）
-------------------------
「同一 scenario YAML 驱动仿真与真机，切换只发生在驱动实现层」意味着：

* **Planner 侧**（:mod:`rm75_drake.planner`）只产出数据
  （:class:`~rm75_drake.planning.trajectory.Trajectory` + 夹爪调度表）；
* **驱动侧**（本包）负责「把轨迹变成动作」与「读回状态」；
* 仿真驱动、假驱动、真机驱动（``rm_device``，S03）**实现同一组方法**，
  因此上层代码不需要知道自己在跟谁说话。

本模块**零 pydrake 依赖**（只依赖 numpy + 本项目的轨迹/限位层），
因此假驱动可以脱离 Drake 环境单独使用与测试。

公开接口（S03 的 ``rm_device`` 驱动按同一签名实现）
----------------------------------------------------
``open() -> None``
    打开设备（建立连接 / 进入就绪态）。重复调用必须幂等。
``close() -> None``
    关闭设备（释放资源）。重复调用必须幂等。
``submit_trajectory(trajectory) -> None``
    提交一条轨迹（:class:`~rm75_drake.planning.trajectory.Trajectory`）。
    实现**不得修改轨迹内容**（自检钩子 :meth:`Driver.verify_trajectory_unchanged`
    负责比对深拷贝快照）。
``read_state() -> DriverState``
    读回当前状态：关节角 + ``seq`` + 绝对时刻 ``t_abs``。

流式协议（S03-T06）
-------------------
S02 的驱动协议只有上面四个方法，**没有**「流式执行」语义
（FakeDriver 的 ``step`` / ``replay`` / ``phase`` 都不在基类里）。
S03 补齐了 :class:`StreamingDriver` 协议（``step / stop / hold /
state_age / phase``），并给出**默认实现**：

* :meth:`Driver.step` —— 默认抛 :class:`DriverError`
  （``reason="not_streaming"``），**不是** ``abstractmethod``；
* :meth:`Driver.stop` / :meth:`Driver.hold` / :meth:`Driver.resume`
  —— 相位切换（``stopped`` / ``holding`` / ``streaming``）；
* :meth:`Driver.state_age` —— 最近一次状态读回的年龄（``None`` = 未知）。

因此**既有子类不被破坏**（FakeDriver / SimulationDriver 继续可用），
而三者可以遵守**同一套**流式语义（同源断言的前提）。

失败可解释
----------
所有驱动错误都抛 :class:`DriverError`，并带稳定 ``reason`` 标签：

======================  ====================================================
``invalid_input``       入参类型不对（不是轨迹 / 不是数值序列）
``empty``               提交了**空轨迹**（0 个采样点）
``dt_mismatch``         轨迹采样周期与驱动节拍（5 ms）不一致
``not_open``            设备未打开就提交 / 读状态
``no_trajectory``       设备里还没有已提交的轨迹就推进 / 读状态
``integrity``           驱动修改了轨迹内容（自检钩子发现）
``closed``              设备已关闭后再操作
``not_streaming``       驱动未实现流式推进（``step`` 的默认实现）
``held``                驱动处于保持相位（先 ``resume()``）
``stopped``             驱动已停止，不能再推进
======================  ====================================================
"""

from __future__ import annotations

import abc
import copy
import time
from dataclasses import dataclass
from typing import Any, Optional, Protocol, Tuple, runtime_checkable

from ..planning.trajectory import DEFAULT_DT, JOINT_COUNT

__all__ = [
    "DriverError",
    "DriverState",
    "Driver",
    "StreamingDriver",
    "PHASE_IDLE",
    "PHASE_STREAMING",
    "PHASE_DONE",
    "PHASE_HOLDING",
    "PHASE_STOPPED",
    "REASON_INVALID_INPUT",
    "REASON_EMPTY",
    "REASON_DT_MISMATCH",
    "REASON_NOT_OPEN",
    "REASON_NO_TRAJECTORY",
    "REASON_INTEGRITY",
    "REASON_CLOSED",
    "REASON_NOT_STREAMING",
    "REASON_HELD",
    "REASON_STOPPED",
]


# ── 驱动相位（状态机）──────────────────────────────────────────────────
#: 空闲：已打开但没有正在执行的轨迹。
PHASE_IDLE = "idle"
#: 流式：轨迹已提交，正在按节拍下发。
PHASE_STREAMING = "streaming"
#: 完成：轨迹已全部下发完。
PHASE_DONE = "done"
#: 保持：主动保持当前位置（不再推进，但未终止）。
PHASE_HOLDING = "holding"
#: 停止：本次流式执行已终止（不再接受推进）。
PHASE_STOPPED = "stopped"

# ── 失败标签 ────────────────────────────────────────────────────────────
REASON_INVALID_INPUT = "invalid_input"
REASON_EMPTY = "empty"
REASON_DT_MISMATCH = "dt_mismatch"
REASON_NOT_OPEN = "not_open"
REASON_NO_TRAJECTORY = "no_trajectory"
REASON_INTEGRITY = "integrity"
REASON_CLOSED = "closed"
#: 驱动未实现流式推进（S03-T06 新增；**不是**硬性 abstractmethod）。
REASON_NOT_STREAMING = "not_streaming"
#: 驱动处于保持相位（hold 之后必须先 resume）。
REASON_HELD = "held"
#: 驱动已停止（stop 之后不能再推进）。
REASON_STOPPED = "stopped"


class DriverError(RuntimeError):
    """驱动层错误（带稳定 ``reason`` 标签）。

    属性
    ----
    reason:
        稳定标签（见模块 docstring 的失败表）。
    """

    def __init__(self, message: str, *, reason: str = "driver_error") -> None:
        super().__init__(message)
        self.reason = reason


@dataclass(frozen=True)
class DriverState:
    """驱动回读到的状态。

    字段
    ----
    joint_positions:
        关节角（rad，长度 7，顺序 joint_1..joint_7）。
    seq:
        该状态对应的采样点序号（与轨迹里的 ``seq`` 对齐）。
    t_abs:
        该状态的**绝对时刻**（s）。
    """

    joint_positions: Tuple[float, ...]
    seq: int
    t_abs: float

    def __post_init__(self) -> None:
        positions = tuple(float(v) for v in self.joint_positions)
        if len(positions) != JOINT_COUNT:
            raise DriverError(
                f"joint_positions 必须是 {JOINT_COUNT} 维，实际 {len(positions)} 维",
                reason=REASON_INVALID_INPUT,
            )
        object.__setattr__(self, "joint_positions", positions)
        object.__setattr__(self, "seq", int(self.seq))
        object.__setattr__(self, "t_abs", float(self.t_abs))

    def q_array(self):
        """关节角的 numpy 数组（长度 7）。"""
        import numpy as np

        return np.asarray(self.joint_positions, dtype=float)


class Driver(abc.ABC):
    """驱动协议（抽象基类）。

    子类必须实现
    ------------
    :meth:`open`、:meth:`close`、:meth:`read_state`
    以及内部钩子 :meth:`_on_submit`（提交轨迹后的驱动特定动作）。

    自检钩子
    --------
    :meth:`submit_trajectory` 会在接受轨迹时保存一份**深拷贝快照**；
    :meth:`verify_trajectory_unchanged` 随时可比对「当前轨迹内容」与
    「快照」——一旦驱动实现擅自改写了轨迹（例如重采样、改夹爪调度表），
    立刻抛 :class:`DriverError`（``reason="integrity"``）。
    """

    #: 驱动的稳定标识（写进日志 / 录制文件）。
    name: str = "driver"

    def __init__(self, *, dt: float = DEFAULT_DT) -> None:
        self._dt = float(dt)
        self._open = False
        self._closed = False
        self._trajectory: Any = None
        self._snapshot: Any = None
        # 流式协议状态（S03-T06）
        self._phase: str = PHASE_IDLE
        self._held = False
        self._stopped = False
        self._last_state_monotonic: Optional[float] = None

    # -- 生命周期 --------------------------------------------------------
    def open(self) -> None:
        """打开设备（幂等）。"""
        self._closed = False
        self._open = True
        self._phase = PHASE_IDLE
        self._held = False
        self._stopped = False

    def close(self) -> None:
        """关闭设备（幂等；释放已提交的轨迹引用）。"""
        self._open = False
        self._closed = True
        self._trajectory = None
        self._snapshot = None
        self._phase = PHASE_IDLE
        self._held = False
        self._stopped = False

    @property
    def is_open(self) -> bool:
        """设备是否已打开。"""
        return self._open

    @property
    def dt(self) -> float:
        """驱动节拍（s）——默认 5 ms（与真机透传周期一致）。"""
        return self._dt

    @property
    def trajectory(self) -> Any:
        """当前已提交的轨迹（未提交时为 ``None``）。"""
        return self._trajectory

    @property
    def trajectory_snapshot(self) -> Any:
        """提交时的**深拷贝快照**（自检用）。"""
        return self._snapshot

    # -- 流式协议（S03-T06）----------------------------------------------
    @property
    def phase(self) -> str:
        """当前相位（``idle`` / ``streaming`` / ``done`` / ``holding`` / ``stopped``）。"""
        return self._phase

    @property
    def state_age(self) -> Optional[float]:
        """最近一次**状态读回**的年龄（s）。

        ``None`` 表示「尚无状态读回」——是**未知**，不是测量值 0。
        """
        if self._last_state_monotonic is None:
            return None
        return max(0.0, time.monotonic() - self._last_state_monotonic)

    def step(self) -> DriverState:
        """推进**一个**流式节拍并返回该拍状态（流式协议）。

        默认实现显式报错（``reason="not_streaming"``）——
        **不是** ``abstractmethod``，因此既有子类（FakeDriver /
        SimulationDriver）不会因为新增协议而失效。
        """
        raise DriverError(
            "该驱动未实现流式推进（step）", reason=REASON_NOT_STREAMING
        )

    def stop(self) -> None:
        """停止本次流式执行（保留已提交轨迹；之后 :meth:`step` 显式报错）。"""
        self._require_open()
        self._stopped = True
        self._held = False
        self._phase = PHASE_STOPPED

    def hold(self) -> DriverState:
        """保持当前位置：停止推进但**不**终止（先 :meth:`resume` 才能继续）。"""
        self._require_open()
        self._held = True
        self._phase = PHASE_HOLDING
        return self.read_state()

    def resume(self) -> None:
        """从保持相位恢复到流式相位。"""
        self._require_open()
        if self._stopped:
            raise DriverError(
                "驱动已停止，无法恢复", reason=REASON_STOPPED
            )
        self._held = False
        self._phase = PHASE_STREAMING

    # -- 轨迹提交 --------------------------------------------------------
    def submit_trajectory(self, trajectory: Any) -> None:
        """提交一条轨迹。

        参数
        ----
        trajectory:
            :class:`~rm75_drake.planning.trajectory.Trajectory`（或任何暴露
            ``samples`` 序列的同构对象）。

        失败
        ----
        * 设备未打开 → ``reason="not_open"``；
        * 入参不是轨迹 → ``reason="invalid_input"``；
        * 空轨迹（0 个采样点）→ ``reason="empty"``；
        * 轨迹采样周期与驱动节拍不一致 → ``reason="dt_mismatch"``。
        """
        self._require_open()
        samples = getattr(trajectory, "samples", None)
        if samples is None or isinstance(trajectory, (str, bytes, int, float)):
            raise DriverError(
                f"submit_trajectory 需要 Trajectory，实际：{type(trajectory).__name__}",
                reason=REASON_INVALID_INPUT,
            )
        if len(samples) == 0:
            raise DriverError(
                "不能提交空轨迹（0 个采样点）", reason=REASON_EMPTY
            )
        step = float(getattr(trajectory, "dt", self._dt))
        if abs(step - self._dt) > 1e-12:
            raise DriverError(
                f"轨迹采样周期 {step!r} s 与驱动节拍 {self._dt!r} s 不一致"
                "（真机透传周期固定 5 ms）",
                reason=REASON_DT_MISMATCH,
            )
        self._trajectory = trajectory
        self._snapshot = copy.deepcopy(trajectory)
        self._on_submit(trajectory)

    def _on_submit(self, trajectory: Any) -> None:
        """子类钩子：驱动特定的下发准备（默认什么都不做）。"""

    # -- 状态读取 --------------------------------------------------------
    @abc.abstractmethod
    def read_state(self) -> DriverState:
        """读回当前状态（关节角 + seq + 绝对时刻）。"""

    # -- 自检钩子 --------------------------------------------------------
    def verify_trajectory_unchanged(self) -> bool:
        """校验「当前轨迹内容」与「提交时的深拷贝快照」一致。

        返回
        ----
        ``True`` —— 未被修改；
        ``False`` —— 已被修改（**不**抛异常，便于观测）。

        异常
        ----
        :class:`DriverError`（``reason="integrity"``）
            仅当调用 :meth:`assert_trajectory_unchanged` 时。
        """
        if self._trajectory is None or self._snapshot is None:
            return True
        return _same_trajectory(self._trajectory, self._snapshot)

    def assert_trajectory_unchanged(self) -> None:
        """自检钩子：驱动若修改了轨迹内容 → 抛 :class:`DriverError`。

        异常
        ----
        :class:`DriverError`（``reason="integrity"``）
            当前轨迹内容与提交时的快照不一致。
        """
        if not self.verify_trajectory_unchanged():
            raise DriverError(
                "驱动修改了已提交轨迹的内容（违反「驱动不得修改轨迹」契约）",
                reason=REASON_INTEGRITY,
            )

    # -- 内部工具 --------------------------------------------------------
    def _require_open(self) -> None:
        if not self._open:
            raise DriverError(
                "设备未打开（先调用 open()）",
                reason=REASON_CLOSED if self._closed else REASON_NOT_OPEN,
            )

    def _require_streaming(self) -> None:
        """流式推进前的守卫：已停止 / 保持中 ⇒ 显式报错。"""
        if self._stopped:
            raise DriverError(
                "驱动已停止，不能再推进", reason=REASON_STOPPED
            )
        if self._held:
            raise DriverError(
                "驱动处于保持相位（先调用 resume()）", reason=REASON_HELD
            )

    def _mark_state_read(self) -> None:
        """记录「刚刚读回过一次状态」（供 :attr:`state_age` 使用）。"""
        self._last_state_monotonic = time.monotonic()


@runtime_checkable
class StreamingDriver(Protocol):
    """**流式执行协议**（S03-T06）。

    三个驱动（假驱动 / 仿真驱动 / 真机驱动）都遵守这组语义：

    ``step() -> DriverState``
        推进**一个**节拍（默认 5 ms）并返回该拍状态；
    ``stop() -> None``
        终止本次流式执行（保留轨迹；之后 ``step()`` 显式报错）；
    ``hold() -> DriverState``
        保持当前位置（不再推进，**不**终止）；
    ``state_age -> Optional[float]``
        最近一次状态读回的年龄（s）；``None`` = 未知；
    ``phase -> str``
        当前相位（``idle`` / ``streaming`` / ``done`` / ``holding`` / ``stopped``）。

    说明
    ----
    这是一个**独立的协议对象**（``typing.Protocol``），
    **不是** :class:`Driver` 的新 ``abstractmethod``——
    这样既有子类（FakeDriver / SimulationDriver）保持向后兼容。
    """

    def step(self) -> DriverState:
        """推进一个节拍。"""
        ...

    def stop(self) -> None:
        """终止流式执行。"""
        ...

    def hold(self) -> DriverState:
        """保持当前位置。"""
        ...

    @property
    def state_age(self) -> Optional[float]:
        """状态年龄（s）。"""
        ...

    @property
    def phase(self) -> str:
        """当前相位。"""
        ...


def _same_trajectory(left: Any, right: Any) -> bool:
    """比较两条轨迹的内容是否完全一致（逐采样点、逐分量）。"""
    left_samples = list(getattr(left, "samples", ()))
    right_samples = list(getattr(right, "samples", ()))
    if len(left_samples) != len(right_samples):
        return False
    for a, b in zip(left_samples, right_samples):
        if int(getattr(a, "seq", -1)) != int(getattr(b, "seq", -2)):
            return False
        if abs(float(getattr(a, "t_abs", 0.0)) - float(getattr(b, "t_abs", 0.0))) > 1e-12:
            return False
        left_q = tuple(float(v) for v in getattr(a, "q", ()))
        right_q = tuple(float(v) for v in getattr(b, "q", ()))
        if len(left_q) != len(right_q):
            return False
        for x, y in zip(left_q, right_q):
            if x != y:
                return False
    return True
