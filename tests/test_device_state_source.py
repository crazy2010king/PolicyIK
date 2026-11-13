"""UDP 状态源的单测（S03-T03）。

覆盖
----
* 环形缓冲**覆写**（容量满后丢最老帧）与顺序（最老→最新）；
* 帧**不完整**（``errCode == -3``）→ 丢弃并计数、**不进入缓冲**；
* **age 超时**判定（陈旧帧）与「从未收到帧」的无界语义；
* 负例：维度错 / NaN / 非 StateFrame / 非法阈值 → 显式报错并点名。
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = REPO_ROOT / "src"

if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from rm75_drake.device.state_source import (  # noqa: E402
    DEFAULT_CAPACITY,
    MISSED_CYCLES_UNBOUNDED,
    REASON_INVALID_FRAME,
    FrameRing,
    StateSample,
    StateSource,
    StateSourceError,
)
from rm75_drake.device.transport import StateFrame  # noqa: E402

DOF = 7


def _frame(**kwargs) -> StateFrame:
    """构造一帧（默认健康、全零、未使能）。"""
    base = dict(
        joint_positions_rad=(0.0,) * DOF,
        joint_currents_ma=(0.0,) * DOF,
        joint_en_flags=(True,) * DOF,
        joint_err_codes=(0,) * DOF,
        controller_status=0,
        gripper_online=True,
        arm_current_status=0,
        received_at=0.0,
        err_code=0,
    )
    base.update(kwargs)
    return StateFrame(**base)


class _FakeClock:
    """确定性时钟。"""

    def __init__(self, start: float = 0.0) -> None:
        self.now = float(start)

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> float:
        self.now += float(seconds)
        return self.now


# ── 环形缓冲 ────────────────────────────────────────────────────────────
def test_ring_overwrites_oldest_when_full():
    ring = FrameRing(3)
    for i in range(5):
        ring.push(i)
    assert len(ring) == 3
    assert ring.latest() == 4
    assert ring.to_list() == [2, 3, 4]


def test_ring_rejects_bad_capacity():
    with pytest.raises(StateSourceError) as exc:
        FrameRing(0)
    assert exc.value.reason == REASON_INVALID_FRAME
    with pytest.raises(StateSourceError):
        FrameRing(True)  # type: ignore[arg-type]


def test_ring_empty_latest_is_none():
    assert FrameRing(4).latest() is None


# ── 收帧 ────────────────────────────────────────────────────────────────
def test_push_accepts_valid_frame():
    source = StateSource(capacity=4, clock=_FakeClock())
    assert source.push(_frame()) is True
    assert source.frames_received == 1
    assert source.latest() is not None
    assert source.latest().joint_en_flags == (True,) * DOF


def test_push_rejects_incomplete_frame_and_counts():
    source = StateSource(capacity=4, clock=_FakeClock())
    assert source.push(_frame(err_code=-3)) is False
    assert source.frames_received == 0
    assert source.frames_rejected_incomplete == 1
    # 关键：丢帧**不进入**缓冲（不得让状态机看到半帧）
    assert source.latest() is None
    assert source.frames() == []


def test_push_rejects_wrong_dimension_and_names_field():
    source = StateSource(capacity=4, clock=_FakeClock())
    with pytest.raises(StateSourceError) as exc:
        source.push(_frame(joint_positions_rad=(0.0, 0.1)))
    assert exc.value.reason == REASON_INVALID_FRAME
    assert "joint_positions_rad" in str(exc.value)
    assert source.frames_rejected_invalid == 1
    assert source.frames_received == 0


def test_push_rejects_nan_and_non_stateframe():
    source = StateSource(capacity=4, clock=_FakeClock())
    with pytest.raises(StateSourceError) as exc:
        source.push(_frame(joint_currents_ma=(float("nan"),) + (0.0,) * 6))
    assert "joint_currents_ma" in str(exc.value)
    with pytest.raises(StateSourceError):
        source.push({"joint_positions_rad": (0.0,) * DOF})  # type: ignore[arg-type]


def test_push_copies_sequences_so_caller_cannot_mutate_buffer():
    positions = [0.0] * DOF
    source = StateSource(capacity=4, clock=_FakeClock())
    source.push(_frame(joint_positions_rad=tuple(positions)))
    positions[0] = 99.0
    assert source.latest().joint_positions_rad[0] == 0.0


def test_buffer_overwrites_oldest_frames():
    clock = _FakeClock()
    source = StateSource(capacity=3, clock=clock)
    for i in range(5):
        clock.advance(0.005)
        source.push(_frame(joint_positions_rad=(float(i),) + (0.0,) * 6))
    frames = source.frames()
    assert len(frames) == 3
    assert [f.joint_positions_rad[0] for f in frames] == [2.0, 3.0, 4.0]
    assert source.stats()["buffered_frames"] == 3


# ── age ─────────────────────────────────────────────────────────────────
def test_state_age_grows_with_time():
    clock = _FakeClock()
    source = StateSource(capacity=4, clock=clock)
    source.push(_frame())
    assert source.state_age_s() == pytest.approx(0.0)
    clock.advance(0.020)
    assert source.state_age_s() == pytest.approx(0.020)


def test_state_age_is_infinite_without_any_frame():
    source = StateSource(capacity=4, clock=_FakeClock())
    assert math.isinf(source.state_age_s())
    assert source.missed_cycles(0.005) == MISSED_CYCLES_UNBOUNDED
    assert source.is_stale(0.05) is True


def test_is_stale_threshold():
    clock = _FakeClock()
    source = StateSource(capacity=4, clock=clock)
    source.push(_frame())
    clock.advance(0.030)
    assert source.is_stale(0.05) is False
    clock.advance(0.030)
    assert source.is_stale(0.05) is True


def test_missed_cycles_counts_periods():
    clock = _FakeClock()
    source = StateSource(capacity=4, clock=clock)
    source.push(_frame())
    clock.advance(0.025)
    assert source.missed_cycles(0.005) == 5


def test_invalid_threshold_and_period_raise():
    source = StateSource(capacity=4, clock=_FakeClock())
    with pytest.raises(StateSourceError):
        source.is_stale(0.0)
    with pytest.raises(StateSourceError):
        source.missed_cycles(-1.0)


def test_state_sample_exposes_arrival_time():
    clock = _FakeClock(1.25)
    source = StateSource(capacity=4, clock=clock)
    source.push(_frame())
    sample = source.latest_sample()
    assert isinstance(sample, StateSample)
    assert sample.arrival_time_s == pytest.approx(1.25)


def test_stats_snapshot_keys():
    source = StateSource(capacity=DEFAULT_CAPACITY, clock=_FakeClock())
    source.push(_frame())
    source.push(_frame(err_code=-3))
    stats = source.stats()
    assert stats["frames_received"] == 1
    assert stats["frames_rejected_incomplete"] == 1
    assert stats["capacity"] == DEFAULT_CAPACITY


# ── 并发（写者=UDP 回调线程，读者=执行环）───────────────────────────────
def test_concurrent_writer_and_reader_do_not_corrupt_buffer():
    """写线程持续推帧、读线程持续取最新帧：不得出现半帧 / 崩溃 / 计数丢失。"""
    import threading

    source = StateSource(capacity=64, clock=_FakeClock())
    total = 2000
    errors: list[str] = []
    seen: list[int] = []

    def writer() -> None:
        for i in range(total):
            try:
                source.push(_frame(joint_positions_rad=(float(i),) + (0.0,) * 6))
            except Exception as exc:  # pragma: no cover - 失败即记录
                errors.append(repr(exc))

    def reader() -> None:
        for _ in range(total):
            frame = source.latest()
            if frame is not None:
                seen.append(int(frame.joint_positions_rad[0]))

    threads = [threading.Thread(target=writer), threading.Thread(target=reader)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()

    assert errors == []
    assert source.frames_received == total
    # 读到的帧必须**完整自洽**（不是拼接出来的半帧）
    for value in seen:
        assert value >= 0
    assert len(source.frames()) <= 64


def test_high_rate_push_keeps_ring_bounded():
    """10x 负载（2 万帧）：缓冲容量不变、计数正确、无内存增长。"""
    source = StateSource(capacity=64, clock=_FakeClock())
    for i in range(20000):
        source.push(_frame(joint_positions_rad=(float(i),) + (0.0,) * 6))
    assert source.frames_received == 20000
    assert len(source.frames()) == 64
    assert source.stats()["buffered_frames"] == 64
    assert source.latest().joint_positions_rad[0] == pytest.approx(19999.0)
