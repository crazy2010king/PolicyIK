"""真机驱动测试（S03-T06）：流式协议 + 同源一致性。

被测对象
--------
:mod:`rm75_drake.drivers.base` 的**流式协议**（:class:`StreamingDriver`：
``step / stop / hold / state_age / phase``）、
:mod:`rm75_drake.drivers.rm_device`（:class:`RmDeviceDriver`），
以及既有 :class:`FakeDriver` / :class:`SimulationDriver` 的**向后兼容**。

覆盖的契约
----------
**正例**
* 真机驱动只经**唯一运动通道** ``move_j_canfd``（真机即 ``rm_movej_canfd``）下发；
* 同一 Planner 输出被 **FakeDriver / SimulationDriver / RmDeviceDriver**
  消费 ⇒ 关节序列**逐点一致**（容差 1e-12，R007 同源）；
* 驱动**不修改**轨迹内容（父类深拷贝快照自检钩子）；
* 三个驱动都满足 :class:`StreamingDriver` 协议；
* 5 ms 节拍对齐（虚拟时钟口径）。

**负例（可解释失败）**
* 未打开就推进 → ``not_open``；未提交轨迹就推进 → ``no_trajectory``；
* 保持中推进 → ``held``；停止后推进 → ``stopped``；
* 断链下发 → ``link_down``（且错误不被吞掉）；
* 读不到状态帧（``require_state=True``）→ 安全闸 ``state_age`` 停机
  （急停被调用），**不**伪装成「正常」；
* 越软限位的指令 → 被钳位到限位内并留痕 ``soft_limit``。
"""

from __future__ import annotations

import numpy as np
import pytest

from rm75_drake.device import sdk as sdk_mod
from rm75_drake.device.exec_loop import VirtualClock
from rm75_drake.device.safety import load_safety_limits
from rm75_drake.device.transport import ScriptedTransport
from rm75_drake.drivers import (
    PHASE_DONE,
    PHASE_HOLDING,
    PHASE_IDLE,
    PHASE_STOPPED,
    PHASE_STREAMING,
    REASON_HELD,
    REASON_NOT_OPEN,
    REASON_NO_TRAJECTORY,
    REASON_STOPPED,
    Driver,
    DriverError,
    FakeDriver,
    RmDeviceDriver,
    RmDeviceDriverConfig,
    SimulationDriver,
    StreamingDriver,
)
from rm75_drake.planning.trajectory import DEFAULT_DT, trajectory_from_arrays

#: 一个**既不撞桌面也不进奇异圆柱**的基准构型（与设备安全闸测试同源）。
BASE_Q = (1.0, -0.5, 0.0, 0.0, 0.0, 0.0, 0.0)

#: 每拍位移（rad）——必须同时满足 G2（ω_max·dt）与 G3（a_max·dt²）。
STEP_RAD = 1e-4


def _trajectory(steps: int = 20):
    """一条**完全合规**的合成轨迹（不触发任何安全闸）。"""
    rows = [
        [BASE_Q[0] + index * STEP_RAD, *BASE_Q[1:]] for index in range(steps)
    ]
    return trajectory_from_arrays(rows, t0=0.0, dt=DEFAULT_DT)


class _StubSample:
    """最小「类采样点」对象（绕开 Trajectory 自身的软限位校验）。"""

    def __init__(self, seq: int, t_abs: float, q) -> None:
        self.seq = seq
        self.t_abs = t_abs
        self.q = tuple(q)


class _StubTrajectory:
    """最小「类轨迹」对象（同源测试里只需要 samples / dt）。"""

    def __init__(self, rows, dt: float = DEFAULT_DT) -> None:
        self.samples = [
            _StubSample(index, index * dt, row) for index, row in enumerate(rows)
        ]
        self.dt = dt


def _driver(
    transport: ScriptedTransport | None = None,
    *,
    require_state: bool = False,
    pace: bool = False,
    clock: VirtualClock | None = None,
):
    """装配一个真机驱动（离线替身传输层 + 仓库真实阈值）。"""
    transport = transport if transport is not None else ScriptedTransport()
    clock = clock if clock is not None else VirtualClock()
    config = RmDeviceDriverConfig(
        require_state=require_state, pace=pace, period_s=DEFAULT_DT
    )
    driver = RmDeviceDriver(
        transport,
        limits=load_safety_limits(),
        config=config,
        clock=clock,
    )
    return driver, transport, clock


# ── 正例 ────────────────────────────────────────────────────────────────
def test_rm_device_driver_dispatches_through_canfd_only():
    """正例：唯一运动通道是 ``move_j_canfd``（真机即 ``rm_movej_canfd``）。"""
    driver, transport, _ = _driver()
    driver.open()
    driver.submit_trajectory(_trajectory())
    driver.step()
    driver.step()

    assert transport.call_names() == ["move_j_canfd", "move_j_canfd"]
    for _, values in transport.calls:
        assert len(values) == 7
        assert all(isinstance(v, float) for v in values)


def test_rm_device_driver_is_pointwise_identical_to_fake_driver():
    """同源证明（核心断言）：真机驱动与假驱动逐点一致（atol 1e-12）。"""
    trajectory = _trajectory()

    fake = FakeDriver()
    fake.open()
    fake.submit_trajectory(trajectory)

    driver, transport, _ = _driver()
    driver.open()
    driver.submit_trajectory(trajectory)

    fake_q = np.asarray(fake.joint_sequence(), dtype=float)
    device_q = np.asarray(driver.joint_sequence(), dtype=float)
    dispatched = np.asarray(
        [values for name, values in transport.calls if name == "move_j_canfd"],
        dtype=float,
    )

    assert fake_q.shape == device_q.shape == dispatched.shape
    assert fake_q.shape[0] == len(trajectory.samples)
    assert np.allclose(device_q, fake_q, atol=1e-12, rtol=0.0)
    assert np.allclose(dispatched, fake_q, atol=1e-12, rtol=0.0)
    assert np.allclose(fake_q, trajectory.q_matrix(), atol=1e-12, rtol=0.0)


def test_rm_device_driver_is_pointwise_identical_to_simulation_driver(
    single_arm_scene,
):
    """同源证明：真机驱动与仿真驱动逐点一致（atol 1e-12）。"""
    trajectory = _trajectory()

    simulation = SimulationDriver(single_arm_scene)
    simulation.open()
    simulation.submit_trajectory(trajectory)

    driver, _, _ = _driver()
    driver.open()
    driver.submit_trajectory(trajectory)

    sim_q = np.asarray(simulation.joint_sequence(), dtype=float)
    device_q = np.asarray(driver.joint_sequence(), dtype=float)
    assert sim_q.shape == device_q.shape
    assert np.allclose(device_q, sim_q, atol=1e-12, rtol=0.0)


def test_rm_device_driver_does_not_modify_submitted_trajectory():
    """正例：驱动不修改轨迹内容（深拷贝快照自检钩子）。"""
    trajectory = _trajectory()
    driver, _, _ = _driver()
    driver.open()
    driver.submit_trajectory(trajectory)
    driver.step()
    assert driver.verify_trajectory_unchanged() is True
    driver.assert_trajectory_unchanged()


def test_streaming_protocol_is_shared_by_three_drivers(single_arm_scene):
    """正例：假驱动 / 仿真驱动 / 真机驱动都满足同一套流式协议。"""
    driver, _, _ = _driver()
    driver.open()
    assert isinstance(driver, StreamingDriver)
    assert isinstance(FakeDriver(), StreamingDriver)
    assert isinstance(SimulationDriver(single_arm_scene), StreamingDriver)


def test_driver_base_stays_backward_compatible():
    """正例：基类新增流式协议**没有**变成硬性 abstractmethod。"""

    class _LegacyDriver(Driver):
        """只实现 S02 契约的旧式驱动（没有 step / hold / stop）。"""

        def read_state(self):  # pragma: no cover - 仅构造契约检查
            raise NotImplementedError

    legacy = _LegacyDriver()
    legacy.open()
    # 新增的流式方法存在且给出**显式**错误（而不是实例化失败）。
    with pytest.raises(DriverError) as excinfo:
        legacy.step()
    assert excinfo.value.reason == "not_streaming"


def test_rm_device_driver_paces_to_5ms_grid():
    """正例：每拍对齐到 5 ms 栅格（虚拟时钟口径，绝对 deadline 不漂移）。"""
    clock = VirtualClock()
    driver, _, _ = _driver(pace=True, clock=clock)
    driver.open()
    driver.submit_trajectory(_trajectory(steps=5))
    for _ in range(5):
        driver.step()
    assert len(clock.sleep_calls) == 5
    assert all(abs(value - DEFAULT_DT) < 1e-12 for value in clock.sleep_calls)
    assert driver.jitter.count == 4  # 首拍没有「上一拍」⇒ 不产生样本


def test_rm_device_driver_phase_transitions():
    """正例：相位 idle → streaming → done。"""
    driver, _, _ = _driver()
    assert driver.phase == PHASE_IDLE
    driver.open()
    driver.submit_trajectory(_trajectory(steps=3))
    assert driver.phase == PHASE_STREAMING
    for _ in range(3):
        driver.step()
    assert driver.phase == PHASE_DONE
    # done 之后不再下发（**不多发**）。
    transport = driver.transport
    before = transport.called("move_j_canfd")
    driver.step()
    assert transport.called("move_j_canfd") == before


def test_rm_device_driver_state_age_is_unknown_without_frames():
    """正例：没有状态帧时 ``state_age`` 是 ``None``（未知，不是 0）。"""
    driver, _, _ = _driver()
    driver.open()
    driver.submit_trajectory(_trajectory(steps=2))
    assert driver.state_age is None
    assert driver.read_device_frame() is None


def test_rm_device_driver_discards_incomplete_state_frame():
    """负例：不完整帧（errCode=-3）被丢弃 ⇒ 等价于「没有状态」⇒ 安全闸停机。"""
    transport = ScriptedTransport()
    transport.set_frame_incomplete(True)
    transport.push_frame()
    driver, _, _ = _driver(transport, require_state=True)
    driver.open()
    driver.submit_trajectory(_trajectory(steps=2))
    assert driver.read_device_frame() is None
    with pytest.raises(DriverError) as excinfo:
        driver.step()
    assert excinfo.value.reason == "state_age"


@pytest.mark.skipif(
    sdk_mod.sdk_available(), reason="本机已安装 robotic-arm，缺失路径不可测"
)
def test_rm_device_driver_surfaces_missing_sdk():
    """负例：SDK 缺失 → 显式 NotAvailableError（不得静默降级成「假连接」）。"""
    from rm75_drake.device.config import load_device_config
    from rm75_drake.device.transport import SdkTransport

    driver = RmDeviceDriver(
        SdkTransport(load_device_config()),
        limits=load_safety_limits(),
        config=RmDeviceDriverConfig(require_state=False, pace=False),
        clock=VirtualClock(),
    )
    with pytest.raises(sdk_mod.NotAvailableError):
        driver.open()


# ── 负例（可解释失败）───────────────────────────────────────────────────
def test_rm_device_driver_requires_open_before_use():
    """负例：未打开设备就推进 → ``not_open``。"""
    driver, _, _ = _driver()
    with pytest.raises(DriverError) as excinfo:
        driver.step()
    assert excinfo.value.reason == REASON_NOT_OPEN


def test_rm_device_driver_requires_submitted_trajectory():
    """负例：未提交轨迹就推进 → ``no_trajectory``。"""
    driver, _, _ = _driver()
    driver.open()
    with pytest.raises(DriverError) as excinfo:
        driver.step()
    assert excinfo.value.reason == REASON_NO_TRAJECTORY


def test_rm_device_driver_hold_blocks_stepping_until_resume():
    """负例：保持中推进 → ``held``；``resume()`` 后恢复。"""
    driver, transport, _ = _driver()
    driver.open()
    driver.submit_trajectory(_trajectory(steps=4))
    driver.step()
    driver.hold()
    assert driver.phase == PHASE_HOLDING
    with pytest.raises(DriverError) as excinfo:
        driver.step()
    assert excinfo.value.reason == REASON_HELD
    driver.resume()
    driver.step()
    assert driver.phase == PHASE_STREAMING


def test_rm_device_driver_stop_blocks_stepping():
    """负例：停止后推进 → ``stopped``；且停机时发出缓停。"""
    driver, transport, _ = _driver()
    driver.open()
    driver.submit_trajectory(_trajectory(steps=4))
    driver.step()
    driver.stop()
    assert driver.phase == PHASE_STOPPED
    assert transport.called("slow_stop") == 1
    with pytest.raises(DriverError) as excinfo:
        driver.step()
    assert excinfo.value.reason == REASON_STOPPED


def test_rm_device_driver_surfaces_link_down():
    """负例：断链时下发失败 → ``link_down``（错误不被吞掉）。"""
    driver, transport, _ = _driver()
    driver.open()
    driver.submit_trajectory(_trajectory(steps=4))
    transport.drop_link()
    with pytest.raises(DriverError) as excinfo:
        driver.step()
    assert excinfo.value.reason == "link_down"
    assert driver.errors and driver.errors[-1]["reason"] == "link_down"


def test_rm_device_driver_missing_state_triggers_safe_stop():
    """负例：读不到状态帧（require_state=True）→ ``state_age`` 停机 + 急停。"""
    driver, transport, _ = _driver(require_state=True)
    driver.open()
    driver.submit_trajectory(_trajectory(steps=4))
    with pytest.raises(DriverError) as excinfo:
        driver.step()
    assert excinfo.value.reason == "state_age"
    assert transport.called("emergency_stop") == 1
    assert driver.phase == PHASE_STOPPED
    assert any(v.reason == "state_age" for v in driver.violations)


def test_rm_device_driver_clamps_out_of_limit_command_and_keeps_evidence():
    """负例：越软限位的指令 → 被钳位到限位内，并留痕 ``soft_limit``。"""
    driver, transport, _ = _driver()
    driver.open()
    # 第 1 点合规，第 2 点越软限位（joint_1 = 3.2 rad > 3.0362 rad）。
    rows = [[BASE_Q[0], *BASE_Q[1:]], [3.2, *BASE_Q[1:]]]
    out_of_limit = _StubTrajectory(rows)
    driver.submit_trajectory(out_of_limit)
    driver.step()
    state = driver.step()
    limits = load_safety_limits()
    assert limits.soft_lower_rad[0] <= state.joint_positions[0] <= limits.soft_upper_rad[0]
    assert any(v.reason == "soft_limit" for v in driver.violations)
