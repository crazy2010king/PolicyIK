"""真机闭环新增能力的回归测试（零硬件）。

覆盖
----
* :mod:`rm75_drake.drivers.rm_device` 的 **G5 电流闸去抖**
  （真机实测：控制器起步瞬间有单拍尖峰 ⇒ 连续 3 拍越限才停机）；
* :mod:`scripts.eval_tracking` 的跟踪误差评估（指令 vs 实测 → 关节误差 + 末端误差）；
* :mod:`scripts.calibrate_currents` 的包络统计（去抖口径：窗口最小值的最大值）；
* CLI 真机执行的**起点一致性守卫**（起点位形不匹配 ⇒ 拒绝执行）。
"""

from __future__ import annotations

import importlib.util
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = REPO_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))


def _load_script(name: str):
    """按路径加载 ``scripts/`` 下的脚本模块（它们不是包的一部分）。"""
    path = REPO_ROOT / "scripts" / f"{name}.py"
    spec = importlib.util.spec_from_file_location(f"_script_{name}", path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


# ── G5 电流闸去抖 ────────────────────────────────────────────────────────
def _driver_with_current(current_ma: float, *, ticks: int = 1):
    from rm75_drake.device.safety import SafetyLimits
    from rm75_drake.device.transport import ScriptedTransport, StateFrame
    from rm75_drake.drivers.rm_device import RmDeviceDriver, RmDeviceDriverConfig

    limits = SafetyLimits(
        soft_lower_rad=(-1.0,) * 7,
        soft_upper_rad=(1.0,) * 7,
        max_velocity_rad_s=(3.14,) * 7,
        max_accel_rad_s2=(8.0,) * 7,
        current_threshold_ma=(1000.0,) * 7,
        table_height_m=0.0,
        singularity_radius_m=0.01,
    )
    transport = ScriptedTransport()
    transport.set_currents([current_ma] * 7)
    transport.set_enable_flags([True] * 7)
    for _ in range(ticks):
        transport.push_frame(
            StateFrame(
                joint_currents_ma=(current_ma,) * 7,
                joint_en_flags=(True,) * 7,
                received_at=__import__("time").monotonic(),
            )
        )
    driver = RmDeviceDriver(
        transport,
        limits=limits,
        config=RmDeviceDriverConfig(require_state=True, current_stop_after_ticks=3),
    )
    return driver, transport


def _trajectory(count: int = 5):
    from rm75_drake.planning.trajectory import Trajectory, TrajectorySample

    # 位形必须**离开围栏**（零位在基座轴线上 ⇒ 会额外触发 G6 围栏闸）
    pose = (0.0, -0.6, 0.0, 0.9, 0.0, 0.5, 0.0)
    samples = tuple(
        TrajectorySample(seq=i, t_abs=i * 0.005, q=pose) for i in range(count)
    )
    return Trajectory(samples=samples, dt=0.005)


def test_single_current_spike_does_not_stop_the_driver():
    """单拍电流尖峰**不停机**（真机实测：起步瞬间 J1 有 -2802 mA 尖峰）。"""
    from rm75_drake.drivers.base import DriverError

    driver, transport = _driver_with_current(5000.0, ticks=6)
    driver.open()
    driver.submit_trajectory(_trajectory())
    driver.step()  # 第一拍：尖峰只被记录
    assert driver.phase != "stopped"
    assert any(v.reason == "current" for v in driver.violations)


def test_persistent_current_overrun_stops_the_driver():
    """连续 3 拍越限 ⇒ 停机（去抖不是「永不保护」）。"""
    from rm75_drake.drivers.base import DriverError

    driver, transport = _driver_with_current(5000.0, ticks=8)
    driver.open()
    driver.submit_trajectory(_trajectory())
    with pytest.raises(DriverError) as excinfo:
        for _ in range(5):
            driver.step()
    assert excinfo.value.reason == "current"


def test_debounce_ticks_must_be_positive():
    from rm75_drake.drivers.rm_device import RmDeviceDriverConfig

    with pytest.raises(ValueError):
        RmDeviceDriverConfig(current_stop_after_ticks=0)


# ── 跟踪误差评估 ────────────────────────────────────────────────────────
def test_eval_tracking_pairs_commands_with_feedback():
    module = _load_script("eval_tracking")
    run_log = {
        "events": [
            {"kind": "command", "payload": {"joint_rad": [0.0] * 7}},
            {
                "kind": "state_frame",
                "payload": {"joint_rad": [0.001] * 7, "currents_ma": [0.0] * 7},
            },
            {"kind": "command", "payload": {"joint_rad": [0.01] * 7}},
            {
                "kind": "state_frame",
                "payload": {"joint_rad": [0.011] * 7, "currents_ma": [0.0] * 7},
            },
        ]
    }
    summary = module.evaluate(run_log)
    assert summary["num_pairs"] == 2
    assert summary["joint_error_deg"]["max"] == pytest.approx(
        math.degrees(0.001), abs=1e-6
    )
    assert summary["tcp_error_mm"]["max"] >= 0.0


def test_eval_tracking_rejects_log_without_pairs():
    module = _load_script("eval_tracking")
    with pytest.raises(ValueError):
        module.evaluate({"events": []})


# ── 电流标定：去抖包络 ───────────────────────────────────────────────────
def test_calibrate_currents_debounced_envelope_ignores_single_spike():
    module = _load_script("calibrate_currents")
    # 20 拍 ~100 mA，第 5 拍一个 9000 mA 的单点尖峰
    values = [100.0] * 20
    values[5] = 9000.0
    envelope = module._debounced_envelope(values, 3)
    assert envelope == pytest.approx(100.0)


def test_calibrate_currents_debounced_envelope_keeps_sustained_level():
    module = _load_script("calibrate_currents")
    values = [100.0] * 10 + [2000.0] * 10
    envelope = module._debounced_envelope(values, 3)
    assert envelope == pytest.approx(2000.0)


# ── 真机互斥锁 ──────────────────────────────────────────────────────────
def test_hardware_lock_is_reentrant_within_a_process():
    """同一进程内可重入（厂商约束是**跨进程**互斥）。"""
    from rm75_drake.device.hardware_lock import hardware_lock

    with hardware_lock("192.168.0.99", holder="test-a"):
        with hardware_lock("192.168.0.99", holder="test-b"):
            pass


def test_hardware_lock_blocks_another_process():
    """另一个进程拿不到锁 ⇒ 明确报错（真机实测：并发会顶掉连接，rm_movej_canfd -2）。"""
    import subprocess
    import textwrap

    from rm75_drake.device.hardware_lock import HardwareLockBusy, hardware_lock

    script = textwrap.dedent(
        """
        import sys
        sys.path.insert(0, %r)
        from rm75_drake.device.hardware_lock import hardware_lock, HardwareLockBusy
        try:
            with hardware_lock("192.168.0.98", holder="child"):
                print("acquired")
        except HardwareLockBusy:
            print("busy")
        """
        % str(SRC_DIR)
    )
    with hardware_lock("192.168.0.98", holder="parent"):
        out = subprocess.run(
            [sys.executable, "-c", script], capture_output=True, text=True, timeout=60
        )
    assert "busy" in out.stdout
    assert issubclass(HardwareLockBusy, RuntimeError)


# ── CLI 起点一致性守卫 ──────────────────────────────────────────────────
def test_cli_wait_settled_returns_true_for_static_frames():
    from rm75_drake.cli import _wait_settled
    from rm75_drake.device.transport import StateFrame

    class _Source:
        def __init__(self) -> None:
            self.frame = StateFrame(joint_positions_rad=(0.0,) * 7)

        def latest(self):
            return self.frame

    assert _wait_settled(_Source(), timeout_s=0.5) is True


def test_cli_wait_settled_times_out_for_moving_frames():
    from rm75_drake.cli import _wait_settled
    from rm75_drake.device.transport import StateFrame

    class _Source:
        def __init__(self) -> None:
            self.counter = 0

        def latest(self):
            self.counter += 1
            return StateFrame(joint_positions_rad=(0.01 * self.counter,) * 7)

    assert _wait_settled(_Source(), timeout_s=0.3) is False
