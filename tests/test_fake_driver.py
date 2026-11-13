"""驱动层测试（S02-T07）：FakeDriver / SimulationDriver 与「同源证明」。

被测对象
--------
:mod:`rm75_drake.drivers.base`（:class:`Driver` 协议 + :class:`DriverState`
+ 「驱动不得修改轨迹」自检钩子）、
:mod:`rm75_drake.drivers.fake`（:class:`FakeDriver`）、
:mod:`rm75_drake.drivers.sim`（:class:`SimulationDriver`）。

覆盖的契约
----------
**正例**
* 假驱动按 **5 ms 节拍**回放已提交轨迹，**回读内容 == 提交内容**（逐点一致）；
* 相位状态机明确：**IDLE → STREAMING → DONE**；
* **同源证明（核心断言）**：同一 Planner 输出被 ``SimulationDriver`` 与
  ``FakeDriver`` 消费时，产出的关节序列**逐点一致**（``np.allclose`` 容差 1e-12）；
* 驱动**不修改**轨迹内容（深拷贝快照自检钩子）。

**负例（可解释失败）**
* 提交**空轨迹** → ``DriverError(reason="empty")``；
* 提交非轨迹对象 → ``DriverError(reason="invalid_input")``；
* 轨迹采样周期与驱动节拍不一致 → ``DriverError(reason="dt_mismatch")``；
* 未打开设备就提交 / 读状态 → ``DriverError(reason="not_open")``；
* 未提交轨迹就推进 / 读回 → ``DriverError(reason="no_trajectory")``；
* 驱动擅自改写轨迹 → ``DriverError(reason="integrity")``。

耗时说明
--------
只有「同源证明」那条用例需要 Planner 的输出（含 IRIS，本机 ~8 s/区域），
因此单独打 ``@pytest.mark.slow``；其余用例走**合成轨迹**（毫秒级）。
"""

from __future__ import annotations

import numpy as np
import pytest

from rm75_drake.drivers import (
    PHASE_DONE,
    PHASE_IDLE,
    PHASE_STREAMING,
    REASON_DT_MISMATCH,
    REASON_EMPTY,
    REASON_INTEGRITY,
    REASON_INVALID_INPUT,
    REASON_NOT_OPEN,
    REASON_NO_TRAJECTORY,
    Driver,
    DriverError,
    DriverState,
    FakeDriver,
    SimulationDriver,
)
from rm75_drake.planning.trajectory import DEFAULT_DT, trajectory_from_arrays


# ── 合成轨迹（毫秒级用例用）─────────────────────────────────────────────
def _synthetic_trajectory(steps: int = 12, dt: float = DEFAULT_DT):
    """一条合法的合成轨迹（缓起步，5 ms 网格，满足软限位 / 速度 / 加速度）。"""
    rows = []
    for index in range(steps):
        ratio = index / (steps - 1)
        rows.append(
            [
                0.02 * ratio,
                -0.6,
                0.0,
                0.9,
                0.0,
                0.5,
                0.0,
            ]
        )
    return trajectory_from_arrays(rows, t0=0.0, dt=dt)


class _MutableSample:
    """可变的「类采样点」对象（仅测试用）。"""

    def __init__(self, sample) -> None:
        self.seq = sample.seq
        self.t_abs = sample.t_abs
        self.q = tuple(sample.q)


class _MutableTrajectory:
    """可变的「类轨迹」对象（用于验证驱动完整性自检钩子）。"""

    def __init__(self, samples, dt: float = DEFAULT_DT) -> None:
        self.samples = [_MutableSample(sample) for sample in samples]
        self.dt = dt


class _TamperingDriver(FakeDriver):
    """故意在提交后改写轨迹内容的驱动（负例用）。"""

    def _on_submit(self, trajectory) -> None:  # pragma: no cover - 仅构造用
        super()._on_submit(trajectory)
        trajectory.samples[0].q = (0.123,) * 7


# ── 正例 ────────────────────────────────────────────────────────────────
def test_fake_driver_replays_submitted_trajectory():
    """正例：回读内容 == 提交内容（逐点一致）。"""
    trajectory = _synthetic_trajectory()
    driver = FakeDriver()
    driver.open()
    driver.submit_trajectory(trajectory)

    assert driver.phase == PHASE_STREAMING
    states = driver.replay()
    assert driver.phase == PHASE_DONE
    assert len(states) == len(trajectory)

    expected = trajectory.q_matrix()
    actual = np.asarray([state.joint_positions for state in states], dtype=float)
    assert np.allclose(actual, expected, atol=0.0, rtol=0.0)
    assert [state.seq for state in states] == [s.seq for s in trajectory.samples]
    times = [state.t_abs for state in states]
    assert np.allclose(np.diff(times), DEFAULT_DT)


def test_fake_driver_state_machine_transitions():
    """正例：相位状态机 IDLE → STREAMING → DONE。"""
    trajectory = _synthetic_trajectory(steps=4)
    driver = FakeDriver()
    assert driver.phase == PHASE_IDLE
    driver.open()
    assert driver.phase == PHASE_IDLE
    driver.submit_trajectory(trajectory)
    assert driver.phase == PHASE_STREAMING
    assert driver.index == 0
    driver.step()
    assert driver.index == 1
    for _ in range(len(trajectory.samples)):
        driver.step()
    assert driver.phase == PHASE_DONE
    # done 之后继续推进不会越界，只重复返回最后一点。
    state = driver.step()
    assert state.seq == trajectory.samples[-1].seq


def test_driver_state_carries_seq_and_absolute_time():
    """正例：DriverState 带 seq 与绝对时刻。"""
    trajectory = _synthetic_trajectory(steps=3)
    driver = FakeDriver()
    driver.open()
    driver.submit_trajectory(trajectory)
    state = driver.read_state()
    assert isinstance(state, DriverState)
    assert state.seq == 0
    assert state.t_abs == trajectory.samples[0].t_abs
    assert len(state.joint_positions) == 7


def test_driver_does_not_modify_submitted_trajectory():
    """正例：驱动不修改轨迹内容（深拷贝快照自检钩子）。"""
    trajectory = _synthetic_trajectory()
    driver = FakeDriver()
    driver.open()
    driver.submit_trajectory(trajectory)
    assert driver.verify_trajectory_unchanged() is True
    driver.assert_trajectory_unchanged()


def test_simulation_driver_reads_back_from_plant(single_arm_scene):
    """正例：仿真驱动的状态真的经过 plant 往返（写进去 → 读回来）。"""
    trajectory = _synthetic_trajectory()
    driver = SimulationDriver(single_arm_scene)
    driver.open()
    driver.submit_trajectory(trajectory)
    state = driver.read_state()
    assert np.allclose(
        np.asarray(state.joint_positions, dtype=float),
        trajectory.samples[0].q,
        atol=0.0,
        rtol=0.0,
    )
    # plant 里的当前位形确实被写成了该采样点。
    positions = single_arm_scene.plant.GetPositions(
        driver._context  # noqa: SLF001 - 测试需要验证「真的写进了 plant」
    )
    assert np.allclose(positions, trajectory.samples[0].q, atol=0.0, rtol=0.0)


@pytest.mark.slow
def test_simulation_and_fake_drivers_are_pointwise_identical(planner_bundle):
    """同源证明：仿真驱动与假驱动消费**同一条** Planner 输出 → 逐点一致。"""
    result = planner_bundle["result"]
    assert result.success, f"规划失败：{result.message}"
    trajectory = result.trajectory
    assert trajectory is not None

    fake = FakeDriver()
    fake.open()
    fake.submit_trajectory(trajectory)
    fake_states = fake.replay()

    simulation = SimulationDriver(planner_bundle["scene"])
    simulation.open()
    simulation.submit_trajectory(trajectory)
    simulation_states = simulation.replay()

    fake_q = np.asarray([s.joint_positions for s in fake_states], dtype=float)
    simulation_q = np.asarray([s.joint_positions for s in simulation_states], dtype=float)
    assert fake_q.shape == simulation_q.shape == trajectory.q_matrix().shape
    assert np.allclose(fake_q, simulation_q, atol=1e-12, rtol=0.0)
    assert np.allclose(fake_q, trajectory.q_matrix(), atol=1e-12, rtol=0.0)
    assert [s.seq for s in fake_states] == [s.seq for s in simulation_states]
    assert np.allclose(
        [s.t_abs for s in fake_states],
        [s.t_abs for s in simulation_states],
        atol=1e-12,
        rtol=0.0,
    )


# ── 负例（可解释失败）───────────────────────────────────────────────────
def test_fake_driver_rejects_empty_trajectory():
    """负例：提交空轨迹 → 显式报错。"""
    driver = FakeDriver()
    driver.open()
    with pytest.raises(DriverError) as excinfo:
        driver.submit_trajectory(_MutableTrajectory([]))
    assert excinfo.value.reason == REASON_EMPTY


def test_fake_driver_rejects_non_trajectory():
    """负例：提交非轨迹对象 → 显式报错。"""
    driver = FakeDriver()
    driver.open()
    with pytest.raises(DriverError) as excinfo:
        driver.submit_trajectory([0.0] * 7)
    assert excinfo.value.reason == REASON_INVALID_INPUT


def test_fake_driver_rejects_dt_mismatch():
    """负例：轨迹采样周期与驱动节拍不一致 → 显式报错。"""
    trajectory = _synthetic_trajectory(steps=6, dt=0.01)
    driver = FakeDriver()
    driver.open()
    with pytest.raises(DriverError) as excinfo:
        driver.submit_trajectory(trajectory)
    assert excinfo.value.reason == REASON_DT_MISMATCH


def test_fake_driver_requires_open_before_use():
    """负例：未打开设备就提交 / 读状态 → 显式报错。"""
    driver = FakeDriver()
    with pytest.raises(DriverError) as excinfo:
        driver.submit_trajectory(_synthetic_trajectory())
    assert excinfo.value.reason == REASON_NOT_OPEN
    with pytest.raises(DriverError) as excinfo:
        driver.read_state()
    assert excinfo.value.reason == REASON_NOT_OPEN


def test_fake_driver_requires_submitted_trajectory():
    """负例：未提交轨迹就推进 / 读回 → 显式报错。"""
    driver = FakeDriver()
    driver.open()
    with pytest.raises(DriverError) as excinfo:
        driver.read_state()
    assert excinfo.value.reason == REASON_NO_TRAJECTORY
    with pytest.raises(DriverError) as excinfo:
        driver.step()
    assert excinfo.value.reason == REASON_NO_TRAJECTORY


def test_integrity_hook_detects_trajectory_tampering():
    """负例：驱动擅自改写轨迹 → 自检钩子报错。"""
    samples = _synthetic_trajectory().samples
    mutable = _MutableTrajectory(samples)
    driver = _TamperingDriver()
    driver.open()
    driver.submit_trajectory(mutable)
    assert driver.verify_trajectory_unchanged() is False
    with pytest.raises(DriverError) as excinfo:
        driver.assert_trajectory_unchanged()
    assert excinfo.value.reason == REASON_INTEGRITY


def test_driver_base_is_abstract():
    """负例：Driver 是抽象基类（不能直接实例化）。"""
    with pytest.raises(TypeError):
        Driver()  # type: ignore[abstract]
