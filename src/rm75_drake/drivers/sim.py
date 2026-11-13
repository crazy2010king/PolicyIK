"""仿真驱动（S02-T07）：在 Drake 仿真里按同样的 5 ms 节拍回读。

职责
----
:class:`SimulationDriver` 与
:class:`~rm75_drake.drivers.fake.FakeDriver` **实现同一套接口**
（:class:`~rm75_drake.drivers.base.Driver`），差别只在「状态从哪来」：

* **假驱动**：状态直接来自已提交的轨迹（纯回放，零硬件零仿真）；
* **仿真驱动**：状态**真的写进 Drake 的 plant**（``SetPositions``），
  再从 plant **读回**（``GetPositions``）。

两者消费**同一条 Planner 输出**时，必须产出**逐点一致**的关节序列
（容差 1e-12）——这就是「仿真与假驱动同源」的可执行证明。

为什么不直接返回轨迹里的值
--------------------------
仿真驱动刻意**多走一趟 plant**：写进去 → 读回来。这样「回读内容 == 提交内容」
就不再是「同一份数据自我复读」的平凡结论，而是**经过仿真器状态往返**后的
逐位一致结论（Drake 的 plant 存的就是 ``double``，往返是 bit 级无误差的）。
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

__all__ = ["SimulationDriver"]


class SimulationDriver(Driver):
    """仿真驱动（把轨迹写进 Drake plant，再读回状态）。

    参数
    ----
    scene:
        :class:`rm75_drake.planning.scene.SceneBundle`（含已 Finalize 的 plant）。
    dt:
        驱动节拍（s），默认 5 ms。
    name:
        驱动标识（写进日志 / 录制文件）。

    失败
    ----
    :class:`~rm75_drake.drivers.base.DriverError`
        场景缺少 plant（``invalid_input``）、未打开设备（``not_open`` /
        ``closed``）、尚未提交轨迹（``no_trajectory``）。
    """

    name = "simulation"

    def __init__(self, scene: Any, *, dt: float = DEFAULT_DT, name: str = "simulation") -> None:
        super().__init__(dt=dt)
        self.name = name
        self._scene = scene
        self._context = None
        self._phase = PHASE_IDLE
        self._index = 0

    # -- 生命周期 --------------------------------------------------------
    def open(self) -> None:
        """打开仿真驱动：建立**独立**的 plant 上下文（不动场景的共享上下文）。"""
        super().open()
        plant = getattr(self._scene, "plant", None)
        if plant is None:
            raise DriverError(
                "场景缺少 plant，无法建立仿真上下文", reason="invalid_input"
            )
        self._context = plant.CreateDefaultContext()
        self._phase = PHASE_IDLE
        self._index = 0

    def close(self) -> None:
        """关闭仿真驱动（幂等）。"""
        super().close()
        self._context = None
        self._phase = PHASE_IDLE
        self._index = 0

    # -- 状态 ------------------------------------------------------------
    @property
    def phase(self) -> str:
        """当前相位（``idle`` / ``streaming`` / ``done``）。"""
        return self._phase

    @property
    def index(self) -> int:
        """当前采样点下标。"""
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
        """推进**一个** 5 ms 节拍：把该拍位形写进 plant，再读回。"""
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

        读回路径：把当前采样点位形写进 plant，再从 plant 读关节角——
        因此读回值是**仿真器状态**而不是轨迹里的原始数值。
        """
        self._require_open()
        trajectory = self.trajectory
        if trajectory is None:
            raise DriverError(
                "尚未提交轨迹，无法读回状态", reason=REASON_NO_TRAJECTORY
            )
        if self._context is None:
            raise DriverError(
                "仿真上下文未建立（先调用 open()）", reason="invalid_input"
            )
        samples = trajectory.samples
        index = min(self._index, len(samples) - 1)
        sample = samples[index]
        plant = self._scene.plant
        plant.SetPositions(self._context, sample.q)
        positions = plant.GetPositions(self._context)
        return DriverState(
            joint_positions=tuple(float(v) for v in positions[: len(sample.q)]),
            seq=int(sample.seq),
            t_abs=float(sample.t_abs),
        )

    def replay(self) -> List[DriverState]:
        """从头回放整条轨迹，返回**逐点状态序列**。

        返回值长度 == 采样点数，且**第一个状态就是第一个采样点**。
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
        """回放整条轨迹并返回**关节角序列**（tuple 列表）。"""
        return [state.joint_positions for state in self.replay()]
