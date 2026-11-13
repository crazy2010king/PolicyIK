"""假驱动（S02-T07）：不接真机，**按 5 ms 节拍回放** Planner 的输出。

职责
----
:class:`FakeDriver` 消费 :class:`~rm75_drake.planning.trajectory.Trajectory`
（**Planner 的原始输出，不做任何加工**），按 **5 ms 固定节拍**模拟
「下发 → 回读」，并保证：

* **回读内容 == 提交内容**（逐点、逐分量一致，bit 级相同）；
* 相位状态机明确：**IDLE → STREAMING → DONE**；
* **不修改轨迹**（父类 :meth:`~rm75_drake.drivers.base.Driver.submit_trajectory`
  的深拷贝快照自检钩子守着这条契约）。

为什么需要它
------------
* **离线回放**：S07 的 golden-file 回归需要在**没有真机、没有仿真**的情况下
  跑通「规划 → 下发 → 回读」整条链路；
* **同源证明**：与 :class:`~rm75_drake.drivers.sim.SimulationDriver` 对同一
  Planner 输出产出**逐点一致**的关节序列 → 证明「仿真与假驱动同源」；
* **CI 友好**：零 Drake 依赖（本模块只依赖 numpy + 本项目轨迹层），
  可在 fast 档 CI 里跑。

相位语义
--------
============  ==========================================================
``idle``      已打开，**没有**正在执行的轨迹。
``streaming`` 轨迹已提交，正在按节拍下发（当前位置 = 当前采样点）。
``done``      轨迹已全部下发完（``read_state()`` 仍返回最后一个点）。
============  ==========================================================
"""

from __future__ import annotations

from typing import Any, List

from ..planning.trajectory import DEFAULT_DT
from .base import (
    PHASE_DONE,
    PHASE_IDLE,
    PHASE_STREAMING,
    REASON_NO_TRAJECTORY,
    Driver,
    DriverError,
    DriverState,
)

__all__ = ["FakeDriver"]


class FakeDriver(Driver):
    """假驱动（5 ms 节拍回放，零硬件依赖）。

    参数
    ----
    dt:
        驱动节拍（s），默认 5 ms（与真机透传周期一致）。
    name:
        驱动标识（写进日志 / 录制文件）。
    """

    name = "fake"

    def __init__(self, *, dt: float = DEFAULT_DT, name: str = "fake") -> None:
        super().__init__(dt=dt)
        self.name = name
        self._phase = PHASE_IDLE
        self._index = 0

    # -- 生命周期 --------------------------------------------------------
    def open(self) -> None:
        """打开假设备（幂等）；相位回到 ``idle``。"""
        super().open()
        self._phase = PHASE_IDLE
        self._index = 0

    def close(self) -> None:
        """关闭假设备（幂等）。"""
        super().close()
        self._phase = PHASE_IDLE
        self._index = 0

    # -- 状态 ------------------------------------------------------------
    @property
    def phase(self) -> str:
        """当前相位（``idle`` / ``streaming`` / ``done``）。"""
        return self._phase

    @property
    def index(self) -> int:
        """当前采样点下标（已下发的点数 − 1）。"""
        return self._index

    @property
    def num_samples(self) -> int:
        """已提交轨迹的采样点数（未提交时为 0）。"""
        trajectory = self.trajectory
        return 0 if trajectory is None else len(trajectory.samples)

    # -- 轨迹提交 --------------------------------------------------------
    def _on_submit(self, trajectory: Any) -> None:
        """进入 ``streaming`` 相位，节拍复位到第一个采样点。"""
        self._phase = PHASE_STREAMING
        self._index = 0

    # -- 推进与读回 ------------------------------------------------------
    def step(self) -> DriverState:
        """推进**一个** 5 ms 节拍并返回该拍的状态。

        推进到最后一个采样点之后，相位变为 ``done``；
        之后再次调用 :meth:`step` 只会重复返回最后一个状态（**不**越界）。
        """
        self._require_open()
        trajectory = self.trajectory
        if trajectory is None:
            raise DriverError(
                "尚未提交轨迹，无法推进节拍", reason=REASON_NO_TRAJECTORY
            )
        samples = trajectory.samples
        if self._index + 1 < len(samples):
            self._index += 1
        if self._index >= len(samples) - 1:
            self._phase = PHASE_DONE
        return self.read_state()

    def read_state(self) -> DriverState:
        """读回当前状态（**不**推进节拍）。

        失败
        ----
        :class:`~rm75_drake.drivers.base.DriverError`
            设备未打开（``not_open`` / ``closed``）或尚未提交轨迹
            （``no_trajectory``）。
        """
        self._require_open()
        trajectory = self.trajectory
        if trajectory is None:
            raise DriverError(
                "尚未提交轨迹，无法读回状态", reason=REASON_NO_TRAJECTORY
            )
        samples = trajectory.samples
        index = min(self._index, len(samples) - 1)
        sample = samples[index]
        return DriverState(
            joint_positions=tuple(float(v) for v in sample.q),
            seq=int(sample.seq),
            t_abs=float(sample.t_abs),
        )

    def replay(self) -> List[DriverState]:
        """从头回放整条轨迹，返回**逐点状态序列**。

        返回值长度 == 采样点数，且**第一个状态就是第一个采样点**
        （回放是「先读当前点，再推进」）。
        回放结束后相位为 ``done``。
        """
        self._require_open()
        trajectory = self.trajectory
        if trajectory is None:
            raise DriverError(
                "尚未提交轨迹，无法回放", reason=REASON_NO_TRAJECTORY
            )
        states: List[DriverState] = [self.read_state()]
        for _ in range(len(trajectory.samples) - 1):
            states.append(self.step())
        return states

    def joint_sequence(self) -> List[tuple]:
        """回放整条轨迹并返回**关节角序列**（tuple 列表）。

        这是「同源比对」的最小接口：仿真驱动与假驱动对同一条 Planner 输出
        产出的关节序列必须逐点一致。
        """
        return [state.joint_positions for state in self.replay()]
