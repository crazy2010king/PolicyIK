"""双臂同一 tick 同步执行器（S04-T07）单测。

覆盖四类断言（与任务计划一一对应）：

1. **同 tick 次序**：一拍内左臂下发**先于**右臂（次序可配置）。
2. **时间差可测**：虚拟时钟下两臂下发时间差被记录（逻辑时间差 = 0），
   且直方图 ``count == 拍数``；另有一条用例证明时间差**可被测量**
   （左臂发送耗时 200 µs ⇒ 时间差 = 200 000 ns）。
3. **口径唯一**：结果里的直方图对象**就是**
   :class:`rm75_drake.device.metrics.MetricHistogram`（同一类型、同一
   ``summary()`` 字段名）——禁止出现第二套指标口径。
4. **安全闸联动 + 负例**：软限位越限 ⇒ 停发并回报 S03 稳定 reason 标签；
   钳位类违规（G2/G3）不停止；停机类闸（G7）⇒ 两臂同时急停；
   传输错误 ⇒ 显式上报；轨迹维度不一致 / 未知臂名 / 非法 ``max_ticks``
   ⇒ 显式报错并点名。

外加「未验证声明」：结果恒带 ``hardware_required=True`` 与
``unverified_metrics``（含「双臂下发时间差 < 1 ms（真机）」）——真机指标
**不**声称已达标（MEM083 纪律）。
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any, Dict, List, Tuple

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = REPO_ROOT / "src"

if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from rm75_drake.bimanual.sync_exec import (  # noqa: E402
    ARM_LEFT,
    ARM_RIGHT,
    DualSyncExecutor,
    SyncExecError,
    UNVERIFIED_DISPATCH_SKEW_METRIC,
)
from rm75_drake.device.exec_loop import (  # noqa: E402
    REASON_COMPLETED,
    REASON_SAFE_STOP,
    REASON_TRANSPORT_ERROR,
    ExecLoopConfig,
    VirtualClock,
)
from rm75_drake.device.metrics import JitterHistogram, MetricHistogram  # noqa: E402
from rm75_drake.device.safety import (  # noqa: E402
    REASON_ACCEL_CLAMP,
    REASON_SOFT_LIMIT,
    REASON_STATE_AGE,
    SafetyLimits,
    load_safety_limits,
)
from rm75_drake.device.transport import (  # noqa: E402
    ScriptedTransport,
    StateFrame,
    TransportError,
)

#: 一个**既不撞桌面也不进奇异圆柱**的已知构型（与 device/safety 用例同源）。
SAFE_POSE: Tuple[float, ...] = (1.0, -0.5, 0.0, 0.0, 0.0, 0.0, 0.0)

#: 越出 joint_7 软限位（±6.2134 rad）的构型：TCP 位置不变 ⇒ 不触发围栏闸。
OVER_LIMIT_POSE: Tuple[float, ...] = (1.0, -0.5, 0.0, 0.0, 0.0, 0.0, 6.5)


# ── 替身 ────────────────────────────────────────────────────────────────
class RecordingTransport(ScriptedTransport):
    """在 :class:`ScriptedTransport` 之上记录**跨臂统一下发日志**的替身。

    * ``send_cost_s``：模拟一次下发的耗时（推进虚拟时钟）⇒ 时间差可测；
    * ``fail_move``：模拟下发失败（传输层错误）；
    * ``fresh_state``：每拍返回**新鲜**状态帧（``received_at`` = 当前虚拟时刻）。
    """

    def __init__(
        self,
        arm: str,
        log: List[Tuple[str, int, Tuple[float, ...]]],
        clock: VirtualClock,
        *,
        send_cost_s: float = 0.0,
        fail_move: bool = False,
        fresh_state: bool = True,
    ) -> None:
        super().__init__()
        self.arm = arm
        self._log = log
        self._clock = clock
        self.send_cost_s = send_cost_s
        self.fail_move = fail_move
        self.fresh_state = fresh_state
        self.open()

    def read_state(self) -> StateFrame | None:
        """返回一帧**新鲜**状态（陈旧帧 / 无帧另有专门用例）。"""
        if self.fresh_state:
            self.push_frame(
                StateFrame(
                    joint_positions_rad=SAFE_POSE,
                    joint_currents_ma=(0.0,) * 7,
                    joint_en_flags=(True,) * 7,
                    joint_err_codes=(0,) * 7,
                    controller_status=0,
                    gripper_online=True,
                    arm_current_status=5,
                    received_at=self._clock.now_s,
                    err_code=0,
                )
            )
        return super().read_state()

    def move_j_canfd(self, joint_rad: Any, *, follow: bool = True) -> None:
        """记录一次下发（可模拟耗时 / 失败）。"""
        if self.send_cost_s:
            self._clock.advance(self.send_cost_s)
        self._log.append((self.arm, self._clock.now_ns(), tuple(joint_rad)))
        if self.fail_move:
            raise TransportError("模拟下发失败（链路断开）", reason="link_down")
        super().move_j_canfd(joint_rad, follow=follow)


# ── 夹具 ────────────────────────────────────────────────────────────────
@pytest.fixture(scope="module")
def limits() -> SafetyLimits:
    """真实仓库阈值（限位交接件 + 设备配置）。"""
    return load_safety_limits()


def _make_executor(
    limits: SafetyLimits,
    clock: VirtualClock,
    log: List[Tuple[str, int, Tuple[float, ...]]],
    points: List[Tuple[float, ...]],
    *,
    right_points: List[Tuple[float, ...]] | None = None,
    order: Tuple[str, ...] = (ARM_LEFT, ARM_RIGHT),
    left_kwargs: Dict[str, Any] | None = None,
    right_kwargs: Dict[str, Any] | None = None,
) -> Tuple[DualSyncExecutor, RecordingTransport, RecordingTransport]:
    """构造执行器 + 两臂替身（默认：新鲜状态、无发送耗时）。"""
    left = RecordingTransport(ARM_LEFT, log, clock, **(left_kwargs or {}))
    right = RecordingTransport(ARM_RIGHT, log, clock, **(right_kwargs or {}))
    executor = DualSyncExecutor(
        left,
        right,
        left_points=points,
        right_points=right_points if right_points is not None else points,
        limits=limits,
        config=ExecLoopConfig(),
        clock=clock,
        order=order,
    )
    return executor, left, right


# ── 1. 同 tick 次序 ─────────────────────────────────────────────────────
def test_same_tick_dispatch_order_is_left_then_right(limits: SafetyLimits) -> None:
    """一拍内左臂下发**先于**右臂（默认次序 left → right）。"""
    clock = VirtualClock()
    log: List[Tuple[str, int, Tuple[float, ...]]] = []
    points = [SAFE_POSE] * 4
    executor, left, right = _make_executor(limits, clock, log, points)

    result = executor.run()

    arms = [entry[0] for entry in log]
    assert arms == [ARM_LEFT, ARM_RIGHT] * len(points)
    assert result.left_dispatches == len(points)
    assert result.right_dispatches == len(points)
    assert result.ticks == len(points)
    assert result.ok is True
    assert result.stop_reason == REASON_COMPLETED
    assert left.called("move_j_canfd") == len(points)
    assert right.called("move_j_canfd") == len(points)


def test_dispatch_order_is_configurable(limits: SafetyLimits) -> None:
    """``order`` 可配置：反向次序下右臂先下发。"""
    clock = VirtualClock()
    log: List[Tuple[str, int, Tuple[float, ...]]] = []
    points = [SAFE_POSE] * 3
    executor, _, _ = _make_executor(
        limits, clock, log, points, order=(ARM_RIGHT, ARM_LEFT)
    )

    result = executor.run()

    arms = [entry[0] for entry in log]
    assert arms == [ARM_RIGHT, ARM_LEFT] * len(points)
    assert executor.order == (ARM_RIGHT, ARM_LEFT)
    assert result.ticks == len(points)


def test_both_arms_dispatch_in_the_same_tick(limits: SafetyLimits) -> None:
    """同一拍内两臂下发的**虚拟时刻相同**（逻辑时间差 = 0）。"""
    clock = VirtualClock()
    log: List[Tuple[str, int, Tuple[float, ...]]] = []
    points = [SAFE_POSE] * 3
    executor, _, _ = _make_executor(limits, clock, log, points)

    executor.run()

    for index in range(len(points)):
        left_ns = log[2 * index][1]
        right_ns = log[2 * index + 1][1]
        assert left_ns == right_ns


# ── 2. 时间差可测 ───────────────────────────────────────────────────────
def test_dispatch_skew_is_recorded_zero_under_virtual_clock(
    limits: SafetyLimits,
) -> None:
    """虚拟时钟下时间差被记录为 0（逻辑时间差），直方图 count == 拍数。"""
    clock = VirtualClock()
    log: List[Tuple[str, int, Tuple[float, ...]]] = []
    points = [SAFE_POSE] * 5
    executor, _, _ = _make_executor(limits, clock, log, points)

    result = executor.run()

    assert result.dispatch_skew_ns == [0] * len(points)
    assert result.dispatch_skew.count == len(points)
    assert result.dispatch_skew.max == 0.0
    assert result.dispatch_skew.min == 0.0
    assert result.dispatch_skew_ms == 0.0


def test_dispatch_skew_is_measurable(limits: SafetyLimits) -> None:
    """左臂下发耗时 200 µs ⇒ 两臂下发时间差被**测到** = 200 000 ns。"""
    clock = VirtualClock()
    log: List[Tuple[str, int, Tuple[float, ...]]] = []
    points = [SAFE_POSE] * 3
    executor, _, _ = _make_executor(
        limits,
        clock,
        log,
        points,
        left_kwargs={"send_cost_s": 0.0002},
    )

    result = executor.run()

    assert result.dispatch_skew_ns == [200_000] * len(points)
    assert result.dispatch_skew.count == len(points)
    assert result.dispatch_skew.max == pytest.approx(0.0002)
    assert result.dispatch_skew_ms == pytest.approx(0.2)


# ── 3. 口径唯一（禁止第二套指标口径） ───────────────────────────────────
def test_histograms_are_device_metrics_instances(limits: SafetyLimits) -> None:
    """结果里的直方图对象**就是** ``device.metrics`` 的类型（无第二套口径）。"""
    clock = VirtualClock()
    log: List[Tuple[str, int, Tuple[float, ...]]] = []
    executor, _, _ = _make_executor(limits, clock, log, [SAFE_POSE] * 3)

    result = executor.run()

    assert isinstance(result.dispatch_skew, MetricHistogram)
    assert type(result.dispatch_skew) is MetricHistogram
    assert MetricHistogram.__module__ == "rm75_drake.device.metrics"
    assert isinstance(result.jitter, JitterHistogram)
    assert type(result.jitter) is JitterHistogram


def test_histogram_summary_field_names_match_device_metrics(
    limits: SafetyLimits,
) -> None:
    """``summary()`` 字段名与 ``device/metrics.py`` 完全一致。"""
    clock = VirtualClock()
    log: List[Tuple[str, int, Tuple[float, ...]]] = []
    executor, _, _ = _make_executor(limits, clock, log, [SAFE_POSE] * 3)

    result = executor.run()

    reference = MetricHistogram("reference", "s").summary()
    assert set(result.dispatch_skew.summary().keys()) == set(reference.keys())
    for key in ("count", "min", "max", "mean", "p50", "p99", "p999"):
        assert key in result.dispatch_skew.summary()
    assert result.dispatch_skew.summary()["count"] == 3
    # 毫秒口径复用同一个换算点（metrics.summary_ms）。
    assert result.dispatch_skew.summary_ms()["unit"] == "ms"


# ── 4. 安全闸联动 ───────────────────────────────────────────────────────
def test_soft_limit_violation_stops_dispatch(limits: SafetyLimits) -> None:
    """越软限位的轨迹点 ⇒ 停发 + 回报 S03 稳定 reason 标签 ``soft_limit``。"""
    clock = VirtualClock()
    log: List[Tuple[str, int, Tuple[float, ...]]] = []
    points = [SAFE_POSE, SAFE_POSE, OVER_LIMIT_POSE]
    executor, left, right = _make_executor(limits, clock, log, points)

    result = executor.run()

    assert result.violations, "越限必须留痕"
    assert REASON_SOFT_LIMIT in result.reasons
    assert result.stop_reason == REASON_SOFT_LIMIT
    assert result.final_state == REASON_SAFE_STOP
    assert result.ok is False
    # 越限点所在的第 3 拍**没有下发**。
    assert result.ticks == 2
    assert result.left_dispatches == 2
    assert result.right_dispatches == 2
    assert left.called("move_j_canfd") == 2
    assert right.called("move_j_canfd") == 2
    # 违规记录带臂名与拍号（可定位）。
    records = [r for r in result.violation_records if r["reason"] == REASON_SOFT_LIMIT]
    assert records and all(r["arm"] in (ARM_LEFT, ARM_RIGHT) for r in records)


def test_soft_limit_halt_does_not_slam_emergency_stop(
    limits: SafetyLimits,
) -> None:
    """软限位越限是「保持」而非「急停」（急停会引入机械冲击）。"""
    clock = VirtualClock()
    log: List[Tuple[str, int, Tuple[float, ...]]] = []
    points = [SAFE_POSE, OVER_LIMIT_POSE]
    executor, left, right = _make_executor(limits, clock, log, points)

    executor.run()

    assert left.called("emergency_stop") == 0
    assert right.called("emergency_stop") == 0


def test_clamp_only_violation_does_not_stop_execution(limits: SafetyLimits) -> None:
    """钳位类违规（G3 加速度）**只记录不停止**（与 S03 单臂语义一致）。"""
    clock = VirtualClock()
    log: List[Tuple[str, int, Tuple[float, ...]]] = []
    # 每拍步进 0.01 rad：超过 a_max·dt（6.0 × 5 ms = 0.03 rad/s）⇒ G3 触发。
    stepped = (SAFE_POSE[0] + 0.01,) + SAFE_POSE[1:]
    points = [SAFE_POSE, stepped, stepped]
    executor, left, right = _make_executor(limits, clock, log, points)

    result = executor.run()

    assert REASON_ACCEL_CLAMP in result.reasons
    assert result.ok is True
    assert result.stop_reason == REASON_COMPLETED
    assert result.ticks == len(points)
    assert result.left_dispatches == len(points)
    assert result.right_dispatches == len(points)


def test_state_age_gate_stops_both_arms(limits: SafetyLimits) -> None:
    """读不到状态帧 ⇒ G7 停机（reason ``state_age``），两臂同时急停。"""
    clock = VirtualClock()
    log: List[Tuple[str, int, Tuple[float, ...]]] = []
    left = ScriptedTransport()
    right = ScriptedTransport()
    left.open()
    right.open()
    executor = DualSyncExecutor(
        left,
        right,
        left_points=[SAFE_POSE] * 3,
        right_points=[SAFE_POSE] * 3,
        limits=limits,
        config=ExecLoopConfig(),
        clock=clock,
    )

    result = executor.run()

    assert REASON_STATE_AGE in result.reasons
    assert result.stop_reason == REASON_STATE_AGE
    assert result.final_state == REASON_SAFE_STOP
    assert result.ticks == 0
    assert result.left_dispatches == 0
    assert result.right_dispatches == 0
    assert left.called("emergency_stop") == 1
    assert right.called("emergency_stop") == 1


def test_transport_error_is_reported(limits: SafetyLimits) -> None:
    """下发失败 ⇒ 显式上报传输错误（不伪装成正常走完）。"""
    clock = VirtualClock()
    log: List[Tuple[str, int, Tuple[float, ...]]] = []
    executor, _, _ = _make_executor(
        limits, clock, log, [SAFE_POSE] * 3, left_kwargs={"fail_move": True}
    )

    result = executor.run()

    assert result.final_state == REASON_TRANSPORT_ERROR
    assert result.stop_reason == "link_down"
    assert result.errors
    assert result.errors[0]["arm"] == ARM_LEFT
    assert result.ok is False


# ── 负例 ────────────────────────────────────────────────────────────────
def test_trajectory_length_mismatch_is_rejected(limits: SafetyLimits) -> None:
    """左右轨迹点数不一致 ⇒ 显式报错并点名两侧点数。"""
    clock = VirtualClock()
    log: List[Tuple[str, int, Tuple[float, ...]]] = []
    with pytest.raises(SyncExecError) as excinfo:
        _make_executor(
            limits,
            clock,
            log,
            [SAFE_POSE] * 3,
            right_points=[SAFE_POSE] * 2,
        )
    message = str(excinfo.value)
    assert excinfo.value.reason == "trajectory_length_mismatch"
    assert "left" in message and "right" in message
    assert "3" in message and "2" in message


def test_joint_dimension_mismatch_is_rejected(limits: SafetyLimits) -> None:
    """轨迹点不是 7 维 ⇒ 显式报错并点名臂与下标。"""
    clock = VirtualClock()
    log: List[Tuple[str, int, Tuple[float, ...]]] = []
    with pytest.raises(SyncExecError) as excinfo:
        _make_executor(limits, clock, log, [SAFE_POSE[:6]])
    message = str(excinfo.value)
    assert excinfo.value.reason == "joint_dimension_mismatch"
    assert "left_points[0]" in message
    assert "7" in message


def test_non_finite_point_is_rejected(limits: SafetyLimits) -> None:
    """NaN 轨迹点 ⇒ 显式报错（不静默放行）。"""
    clock = VirtualClock()
    log: List[Tuple[str, int, Tuple[float, ...]]] = []
    bad = (float("nan"),) + SAFE_POSE[1:]
    with pytest.raises(SyncExecError) as excinfo:
        _make_executor(limits, clock, log, [bad])
    assert excinfo.value.reason == "joint_dimension_mismatch"


def test_unknown_arm_in_order_is_rejected(limits: SafetyLimits) -> None:
    """``order`` 含未知臂名 ⇒ 显式报错并点名合法臂名。"""
    clock = VirtualClock()
    log: List[Tuple[str, int, Tuple[float, ...]]] = []
    with pytest.raises(SyncExecError) as excinfo:
        _make_executor(
            limits, clock, log, [SAFE_POSE], order=(ARM_LEFT, "middle")
        )
    message = str(excinfo.value)
    assert excinfo.value.reason == "unknown_arm"
    assert "middle" in message
    assert "left" in message and "right" in message


def test_duplicate_arm_in_order_is_rejected(limits: SafetyLimits) -> None:
    """``order`` 里同一臂出现多次 ⇒ 显式报错。"""
    clock = VirtualClock()
    log: List[Tuple[str, int, Tuple[float, ...]]] = []
    with pytest.raises(SyncExecError) as excinfo:
        _make_executor(limits, clock, log, [SAFE_POSE], order=(ARM_LEFT, ARM_LEFT))
    assert excinfo.value.reason == "duplicate_arm"


def test_invalid_max_ticks_is_rejected(limits: SafetyLimits) -> None:
    """非法 ``max_ticks`` ⇒ 显式报错。"""
    clock = VirtualClock()
    log: List[Tuple[str, int, Tuple[float, ...]]] = []
    executor, _, _ = _make_executor(limits, clock, log, [SAFE_POSE] * 2)
    with pytest.raises(SyncExecError) as excinfo:
        executor.run(max_ticks=0)
    assert excinfo.value.reason == "invalid_max_ticks"


def test_max_ticks_limits_the_run(limits: SafetyLimits) -> None:
    """``max_ticks`` 生效（只跑前 N 拍）。"""
    clock = VirtualClock()
    log: List[Tuple[str, int, Tuple[float, ...]]] = []
    executor, _, _ = _make_executor(limits, clock, log, [SAFE_POSE] * 5)

    result = executor.run(max_ticks=2)

    assert result.ticks == 2
    assert result.left_dispatches == 2
    assert len(result.dispatch_skew_ns) == 2


# ── 未验证声明 ──────────────────────────────────────────────────────────
def test_hardware_metric_is_declared_unverified(limits: SafetyLimits) -> None:
    """真机「双臂下发时间差 < 1 ms」**必须**标为未验证。"""
    clock = VirtualClock()
    log: List[Tuple[str, int, Tuple[float, ...]]] = []
    executor, _, _ = _make_executor(limits, clock, log, [SAFE_POSE] * 3)

    result = executor.run()

    assert result.hardware_required is True
    assert UNVERIFIED_DISPATCH_SKEW_METRIC in result.unverified_metrics
    assert "双臂下发时间差 < 1 ms（真机）" in result.unverified_metrics
    payload = result.to_dict()
    assert payload["hardware_required"] is True
    assert "双臂下发时间差 < 1 ms（真机）" in payload["unverified_metrics"]


def test_result_to_dict_exposes_observability(limits: SafetyLimits) -> None:
    """结果自带可观测面（可直接写进运行日志）。"""
    clock = VirtualClock()
    log: List[Tuple[str, int, Tuple[float, ...]]] = []
    executor, _, _ = _make_executor(limits, clock, log, [SAFE_POSE] * 2)

    payload = executor.run().to_dict()

    for key in (
        "ticks",
        "left_dispatches",
        "right_dispatches",
        "dispatch_skew_ns",
        "dispatch_skew",
        "dispatch_skew_ms",
        "jitter",
        "violations",
        "violation_records",
        "errors",
        "stop_reason",
        "final_state",
        "hardware_required",
        "unverified_metrics",
    ):
        assert key in payload
    assert payload["dispatch_skew"]["count"] == 2
    assert payload["dispatch_skew_ms"]["unit"] == "ms"
