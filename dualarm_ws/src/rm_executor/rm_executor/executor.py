"""rm_executor 主循环：预就位 → 10 ms tick 流式透传 → 到位判定 → 收尾。

这是 S03 的**执行内核**。五个阶段（阶段名与切片契约一一对应）：

===========  ==========================================================
阶段          语义
===========  ==========================================================
A 预校验     轨迹 JSON 契约合法 → ``verify.ok == true`` → 时间域限值复检
             （任一不过 → **不下发任何指令**，返回 blocked）
B 预就位     ``transport.preposition(首点, v=pre_position_speed)`` → 等
             ``|q_act − q_goal| < PRE_POSITION_TOL_DEG`` **持续**
             ``PRE_POSITION_HOLD_MS``（超时 → blocked）
C tick 流    按 ``resample.resample_to_grid`` 的 10 ms 栅格，用**绝对截止
             时间**逐 tick ``transport.transmit(...)``
D 到位判定   ``|q_act − q_goal| < ARRIVAL_TOL_DEG`` **且**
             ``|q̇| < ARRIVAL_SPEED_DEG_S`` **持续** ``ARRIVAL_HOLD_MS``
E 收尾       末点保持发送若干拍 → 跟随误差 RMS/峰值 → 限值恢复（无下发则
             记录证据）→ 输出统一报告
===========  ==========================================================

**线程约束（靠代码结构强制）**：tick 循环内**只**做三件事——读反馈快照、
调 ``transport.transmit``、记指标；**不得**做任何 SDK 状态查询
（``current_trajectory_type`` 由反馈线程周期查，见 T5）。

**异常语义**：任一阶段 ``transport`` 返回失败 → 立即 ``transport.stop()``
→ 报告 ``status="error"`` + reason + evidence（FATAL 事件）。

**单位**：内部一律 **rad**，只在调传输层（度）前用 ``rm_hw_bridge`` 的
``RAD2DEG`` **单点**换算（执行层不另开换算处）。

**确定性**：主循环只依赖 ``Clock`` 接口；``ManualClock`` 下可逐 tick 完全
重放（测试断言因此是确定性的），``TickClock`` 下走真实墙钟。

**零 rmctl 依赖、零 tesseract 依赖**。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

from rm_executor import contract
from rm_executor import events as events_mod
from rm_executor import metrics as metrics_mod
from rm_executor import report as report_mod
from rm_executor import resample
from rm_executor.clock import Clock, ManualClock, TickClock

__all__ = [
    "DEFAULT_ARRIVAL_TIMEOUT_S",
    "DEFAULT_POLL_PERIOD_S",
    "DEFAULT_PRE_POSITION_TIMEOUT_S",
    "Executor",
    "ExecutorConfig",
]

#: 反馈轮询周期（秒）—— 预就位 / 到位等待阶段的采样节奏
DEFAULT_POLL_PERIOD_S = 0.010

#: 预就位等待上限（秒）—— 超时 → blocked
DEFAULT_PRE_POSITION_TIMEOUT_S = 30.0

#: 到位等待上限（秒）—— 超时 → blocked
DEFAULT_ARRIVAL_TIMEOUT_S = 2.0


class _BridgeUnavailable(RuntimeError):
    """硬件适配层不可用（取不到 ``RAD2DEG`` 单点换算常量）。"""


@dataclass(frozen=True)
class ExecutorConfig:
    """执行参数（**唯一来源** = ``workcell/rm_workcell.yaml::execution``）。

    本类**只搬运**执行参数，**不定义任何阈值**——反馈闭环阈值全部引用
    :mod:`rm_executor.contract` 的冻结常量，避免第二处真相。
    """

    period_s: float
    follow: bool
    trajectory_mode: int
    radio: int
    pre_position_speed: float

    @classmethod
    def from_execution_config(cls, cfg: Dict[str, Any]) -> "ExecutorConfig":
        """从 ``contract.load_execution_config()`` 的结果构造。"""
        return cls(
            period_s=float(cfg["period_s"]),
            follow=bool(cfg["follow"]),
            trajectory_mode=int(cfg["trajectory_mode"]),
            radio=int(cfg["radio"]),
            pre_position_speed=float(cfg["pre_position_speed"]),
        )

    def to_dict(self) -> Dict[str, Any]:
        return {
            "period_s": self.period_s,
            "follow": self.follow,
            "trajectory_mode": self.trajectory_mode,
            "radio": self.radio,
            "pre_position_speed": self.pre_position_speed,
        }


class Executor:
    """执行器（阶段 A→E）。

    参数
    ----
    transport
        透传通道（:class:`rm_executor.transport.Transport`）——离线
        ``FakeTransport`` 与真机 ``BridgeTransport`` **同一套代码路径**。
    feedback
        反馈读取器：只需暴露 ``latest()`` 返回快照字典。真机用
        ``FeedbackPump``（后台线程采集，tick 线程只读缓存，零 SDK 调用进
        tick 线程）；离线用 ``ScriptedFeedback``（拉取式，索引推进 → 确定性
        重放）。
    config
        执行参数；``None`` → 从工位文件读取（唯一来源），读不到则运行期
        返回 blocked。
    clock
        主时钟；``None`` → 按 mode 选 ``ManualClock``（fake）/``TickClock``
        （live）。
    limits
        限值覆盖（测试可注入）；``None`` → ``rm_hw_bridge.limits``。
    """

    def __init__(self,
                 transport: Any,
                 feedback: Any,
                 config: Optional[ExecutorConfig] = None,
                 clock: Optional[Clock] = None,
                 mode: Optional[str] = None,
                 ws: Optional[str] = None,
                 limits: Optional[Dict[str, Any]] = None,
                 poll_period_s: float = DEFAULT_POLL_PERIOD_S,
                 pre_position_timeout_s: float = DEFAULT_PRE_POSITION_TIMEOUT_S,
                 arrival_timeout_s: float = DEFAULT_ARRIVAL_TIMEOUT_S,
                 stream_clock_factory: Optional[Callable[[float, float], Clock]] = None,
                 ) -> None:
        self.transport = transport
        self.feedback = feedback
        self.ws = ws
        self.limits = limits
        self.poll_period_s = float(poll_period_s)
        self.pre_position_timeout_s = float(pre_position_timeout_s)
        self.arrival_timeout_s = float(arrival_timeout_s)
        self._stream_clock_factory = stream_clock_factory

        self.mode = mode or ("fake" if getattr(transport, "name", "") == "fake"
                             else "live")

        self._config = config
        self._config_error: Optional[Dict[str, Any]] = None
        if self._config is None:
            res = contract.load_execution_config(ws)
            if res.get("status") == contract.STATUS_OK:
                self._config = ExecutorConfig.from_execution_config(res["config"])
            else:
                self._config_error = res

        if clock is None:
            clock = (ManualClock(period_s=0.010) if self.mode == "fake"
                     else TickClock(period_s=0.010))
        self.clock = clock

        #: 流式阶段的时钟（与主时钟共享时间来源，栅格起点 = 阶段 C 起点）
        self.stream_clock: Optional[Clock] = None

        self._events = events_mod.EventLog()
        self._rad2deg_factor: Optional[float] = None
        self._feedback_error_logged = False

        # -- 运行期统计（供报告层读取） --------------------------------
        self._ticks = 0
        self._hold_ticks = 0
        self._jitter_samples: List[float] = []
        self._follow_errors: List[float] = []
        self._pre_position_hold_ms: Optional[float] = None
        self._arrival_err_deg: Optional[float] = None
        self._arrival_hold_ms: Optional[float] = None
        self._planned_s: Optional[float] = None
        self._stream_actual_s: Optional[float] = None

    # ------------------------------------------------------------------ 工具
    def _now(self) -> float:
        """当前时刻（秒，主时钟域）。"""
        return self.clock.now()

    def _advance(self, seconds: float) -> float:
        """把主时钟推进 ``seconds``（等待阶段的节拍）。"""
        deadline = self._now() + float(seconds)
        return self.clock.sleep_until(deadline)

    def _add_event(self, type: str, severity: str, reason: str,
                   evidence: Optional[Sequence[Any]] = None,
                   next_action: str = "", t: Optional[float] = None) -> None:
        """追加一条结构化事件（时间取主时钟）。"""
        self._events.add(
            type=type,
            severity=severity,
            t=self._now() if t is None else float(t),
            reason=reason,
            evidence=list(evidence or []),
            next_action=next_action,
        )

    def _rad2deg(self) -> float:
        """rad → deg 的**单点**换算常量（来自 ``rm_hw_bridge``）。"""
        if self._rad2deg_factor is None:
            try:
                from rm_hw_bridge.bridge import RAD2DEG  # 惰性导入：唯一换算处
            except Exception as exc:  # noqa: BLE001
                raise _BridgeUnavailable(
                    "无法从 rm_hw_bridge.bridge 取得 RAD2DEG（%s）: %s"
                    % (type(exc).__name__, exc)
                ) from exc
            self._rad2deg_factor = float(RAD2DEG)
        return self._rad2deg_factor

    def _to_deg(self, row_rad: Sequence[float], factor: float) -> List[float]:
        """把一行关节角（rad）换算成度（**只此一处**）。"""
        return [float(v) * factor for v in row_rad]

    def _read_feedback(self) -> Optional[Dict[str, Any]]:
        """读一次反馈快照（**纯读**，不在 tick 线程内做 SDK 调用）。

        读失败 → 返回 ``None``（**不返回 0 冒充**）并记一条 RECOVERABLE
        事件（只记第一次，避免刷屏）。
        """
        reader = getattr(self.feedback, "latest", None)
        if reader is None:
            if not self._feedback_error_logged:
                self._feedback_error_logged = True
                self._add_event(
                    "feedback_reader_invalid", "RECOVERABLE",
                    "反馈读取器没有 latest()（当前 %r）" % type(self.feedback).__name__,
                    [], "给 Executor 传入 FeedbackPump 或 ScriptedFeedback 后重跑",
                )
            return None
        try:
            snapshot = reader()
        except Exception as exc:  # noqa: BLE001 - 反馈异常不得打断执行
            if not self._feedback_error_logged:
                self._feedback_error_logged = True
                self._add_event(
                    "feedback_read_failed", "RECOVERABLE",
                    "读取反馈快照异常（%s）: %s" % (type(exc).__name__, exc),
                    [repr(exc)],
                    "检查反馈源（SDK 连接 / UDP 订阅）后重跑",
                )
            return None
        if not isinstance(snapshot, dict):
            if not self._feedback_error_logged:
                self._feedback_error_logged = True
                self._add_event(
                    "feedback_snapshot_invalid", "RECOVERABLE",
                    "反馈快照格式不合法（当前 %r）" % type(snapshot).__name__,
                    [], "检查反馈源返回的快照结构后重跑",
                )
            return None
        return snapshot

    @staticmethod
    def _position_deg(snapshot: Optional[Dict[str, Any]]) -> Optional[List[float]]:
        """快照里的关节角（度）；无数据 → ``None``（不返回 0 冒充）。"""
        if not snapshot:
            return None
        values = snapshot.get("joint_position_deg")
        if not isinstance(values, (list, tuple)) or not values:
            return None
        try:
            return [float(v) for v in values]
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _speed_deg_s(snapshot: Optional[Dict[str, Any]]) -> Optional[float]:
        """快照里的最大关节角速度绝对值（度/秒）；无数据 → ``None``。"""
        if not snapshot:
            return None
        values = snapshot.get("joint_speed_deg_s")
        if not isinstance(values, (list, tuple)) or not values:
            return None
        try:
            return max(abs(float(v)) for v in values)
        except (TypeError, ValueError):
            return None

    @classmethod
    def _position_error(cls, snapshot: Optional[Dict[str, Any]],
                        target_deg: Sequence[float]) -> Optional[float]:
        """``max_j |q_act − q_goal|``（度）；无数据 → ``None``。"""
        actual = cls._position_deg(snapshot)
        if actual is None or len(actual) != len(target_deg):
            return None
        return max(abs(actual[i] - float(target_deg[i])) for i in range(len(actual)))

    def _transport_failure(self, stage: str, result: Dict[str, Any]) -> None:
        """传输失败 → 立即停机 + FATAL 事件（报告 status=error）。"""
        reason = result.get("reason") or "%s 阶段传输失败" % stage
        evidence = list(result.get("evidence") or [])
        evidence.append("transport stage=%s status=%s" % (stage, result.get("status")))
        self._add_event(
            "transport_failed", "FATAL",
            "%s 失败：%s" % (stage, reason),
            evidence,
            result.get("next_action") or "恢复 %s 的下发条件后重跑" % stage,
        )
        stop_result = None
        try:
            stop_result = self.transport.stop()
        except Exception as exc:  # noqa: BLE001 - 停机异常也要留痕
            self._add_event(
                "stop_failed", "FATAL",
                "紧急停止调用异常（%s）: %s" % (type(exc).__name__, exc),
                [repr(exc)],
                "立即人工确认机械臂安全状态并断电",
            )
            return
        if isinstance(stop_result, dict) and stop_result.get("status") != contract.STATUS_OK:
            self._add_event(
                "stop_rejected", "FATAL",
                "紧急停止未成功：%s" % (stop_result.get("reason") or "未知原因"),
                list(stop_result.get("evidence") or []),
                stop_result.get("next_action") or "立即人工确认机械臂安全状态并断电",
            )

    def _make_stream_clock(self, period_s: float, start: float) -> Clock:
        """构造流式阶段的时钟（与主时钟**共享时间来源**，栅格起点 = 阶段 C 起点）。

        共享时间来源是必须的：否则等待阶段推进过虚拟时间后，流式阶段的
        第一个 tick 截止时刻会落在过去（ManualClock 直接抛「虚拟时间不可
        回退」）。
        """
        if self._stream_clock_factory is not None:
            return self._stream_clock_factory(period_s, start)
        if isinstance(self.clock, ManualClock):
            return ManualClock(period_s=period_s, source=self.clock.source, start=start)
        return TickClock(period_s=period_s, start=start)

    # -------------------------------------------------------------- 阶段 A
    def _pre_validate(self, trajectory_path: str, allow_unverified: bool,
                      evidence: List[str]) -> Optional[Dict[str, Any]]:
        """阶段 A：轨迹契约 → verify → 时间域限值复检。

        任一不过 → 记事件并返回 ``None``（**不下发任何指令**）。
        """
        load = contract.load_trajectory(trajectory_path,
                                        allow_unverified=allow_unverified)
        if load.get("status") != contract.STATUS_OK:
            self._add_event(
                "trajectory_contract_rejected", "NEEDS_PARAM",
                load.get("reason") or "轨迹契约校验未通过",
                list(load.get("evidence") or []),
                load.get("next_action") or contract.NEXT_ACTION_PLAN_FIRST,
            )
            return None
        evidence.extend(load.get("evidence") or [])
        if load.get("schema_inferred"):
            evidence.append("schema_inferred=true（units/schema_version 按 rad 推定）")

        trajectory = load["trajectory"]
        times = trajectory["times"]
        points = trajectory["points"]

        limits_result = contract.check_time_domain_limits(
            times, points, ws=self.ws, limits=self.limits
        )
        if limits_result.get("status") != contract.STATUS_OK:
            self._add_event(
                "time_domain_limits_rejected", "NEEDS_PARAM",
                limits_result.get("reason") or "时间域限值复检未通过",
                list(limits_result.get("evidence") or []),
                limits_result.get("next_action") or "",
            )
            return None
        evidence.extend(limits_result.get("evidence") or [])
        return trajectory

    # -------------------------------------------------------------- 阶段 B
    def _wait_pre_position(self, goal_deg: Sequence[float]) -> Tuple[bool, Optional[float], str]:
        """阶段 B：等 ``|q_act − q_goal| < PRE_POSITION_TOL_DEG`` 持续
        ``PRE_POSITION_HOLD_MS``。

        返回 ``(是否到位, 实际保持时长 ms, 未到位原因)``。
        """
        deadline = self._now() + self.pre_position_timeout_s
        hold_start: Optional[float] = None
        while True:
            now = self._now()
            error = self._position_error(self._read_feedback(), goal_deg)
            if error is not None and error < contract.PRE_POSITION_TOL_DEG:
                if hold_start is None:
                    hold_start = now
                elapsed_ms = (now - hold_start) * 1000.0
                if elapsed_ms >= contract.PRE_POSITION_HOLD_MS:
                    return True, elapsed_ms, ""
            else:
                hold_start = None
            if now >= deadline:
                return False, None, (
                    "预就位未到位：%.3f s 内未满足 |q_act − q_goal| < %.1f° 持续 %.0f ms"
                    % (self.pre_position_timeout_s, contract.PRE_POSITION_TOL_DEG,
                       contract.PRE_POSITION_HOLD_MS)
                )
            self._advance(self.poll_period_s)

    # -------------------------------------------------------------- 阶段 C
    def _stream_ticks(self, grid_p: Sequence[Sequence[float]], factor: float
                      ) -> Optional[Dict[str, Any]]:
        """阶段 C：按 10 ms 栅格逐 tick 透传。

        tick 线程内**只**做「调 transport.transmit + 读反馈快照 + 记指标」。
        返回 ``None`` 表示成功；失败时事件已记录。
        """
        cfg = self._config
        assert cfg is not None  # run() 已保证
        stream_start = self._now()
        for row_rad in grid_p:
            # ① 绝对截止时间等待（抖动样本由时钟记录）
            self.stream_clock.tick()  # type: ignore[union-attr]
            # ② 下发本拍指令角（度）
            cmd_deg = self._to_deg(row_rad, factor)
            result = self.transport.transmit(
                cmd_deg, cfg.follow, cfg.trajectory_mode, cfg.radio
            )
            if result.get("status") != contract.STATUS_OK:
                self._transport_failure("transmit", result)
                return None
            # ③ 读反馈快照 + 记跟随误差（纯读，零 SDK 调用进 tick 线程）
            snapshot = self._read_feedback()
            error = self._position_error(snapshot, cmd_deg)
            if error is not None:
                self._follow_errors.append(error)
            self._ticks += 1
        self._stream_actual_s = metrics_mod.duration_s(stream_start, self._now())
        return {}

    # -------------------------------------------------------------- 阶段 D/E
    def _hold_and_arrival(self, goal_deg: Sequence[float]) -> bool:
        """阶段 D+E：末点保持发送 + 到位判定。

        返回 ``True`` = 已判定到位。
        """
        cfg = self._config
        assert cfg is not None
        max_ticks = max(1, int(math.ceil(self.arrival_timeout_s / cfg.period_s)))
        deadline = self._now() + self.arrival_timeout_s
        hold_start: Optional[float] = None
        while self._hold_ticks < max_ticks:
            self.stream_clock.tick()  # type: ignore[union-attr]
            result = self.transport.transmit(
                goal_deg, cfg.follow, cfg.trajectory_mode, cfg.radio
            )
            if result.get("status") != contract.STATUS_OK:
                self._transport_failure("transmit_hold", result)
                return False
            self._hold_ticks += 1
            snapshot = self._read_feedback()
            position_error = self._position_error(snapshot, goal_deg)
            if position_error is not None:
                self._follow_errors.append(position_error)
            now = self._now()
            speed = self._speed_deg_s(snapshot)
            if (position_error is not None and speed is not None
                    and position_error < contract.ARRIVAL_TOL_DEG
                    and speed < contract.ARRIVAL_SPEED_DEG_S):
                if hold_start is None:
                    hold_start = now
                elapsed_ms = (now - hold_start) * 1000.0
                if elapsed_ms >= contract.ARRIVAL_HOLD_MS:
                    self._arrival_err_deg = position_error
                    self._arrival_hold_ms = elapsed_ms
                    return True
            else:
                hold_start = None
            if now >= deadline:
                break
        self._add_event(
            "arrival_not_reached", "NEEDS_HUMAN",
            "到位判定未通过：%.3f s 内未满足 |q_act − q_goal| < %.1f° 且 |q̇| < %.1f°/s 持续 %.0f ms"
            % (self.arrival_timeout_s, contract.ARRIVAL_TOL_DEG,
               contract.ARRIVAL_SPEED_DEG_S, contract.ARRIVAL_HOLD_MS),
            [
                "hold_ticks=%d" % self._hold_ticks,
                "arrival_tol_deg=%.3f" % contract.ARRIVAL_TOL_DEG,
                "arrival_speed_deg_s=%.3f" % contract.ARRIVAL_SPEED_DEG_S,
                "arrival_hold_ms=%.0f" % contract.ARRIVAL_HOLD_MS,
            ],
            "检查机械臂是否实际到达末点（反馈是否可信 / 是否被外部动作打断）后重跑",
        )
        return False

    # -------------------------------------------------------------- 报告
    def _build_metrics(self, t_start: float) -> Dict[str, Any]:
        """按 :mod:`rm_executor.metrics` 的冻结口径装配 15 个指标。"""
        jitter = metrics_mod.jitter_stats(self._jitter_samples)
        follow = metrics_mod.follow_error_stats(self._follow_errors)
        arrival = metrics_mod.arrival_metrics(self._arrival_err_deg,
                                               self._arrival_hold_ms)
        return {
            "ticks": int(self._ticks),
            "duration_s": metrics_mod.duration_s(t_start, self._now()),
            "rt_factor": metrics_mod.rt_factor(self._planned_s, self._stream_actual_s),
            "tick_jitter_p50_ms": jitter["p50_ms"],
            "tick_jitter_p95_ms": jitter["p95_ms"],
            "tick_jitter_p99_ms": jitter["p99_ms"],
            "tick_jitter_max_ms": jitter["max_ms"],
            "follow_err_rms_deg": follow["rms_deg"],
            "follow_err_max_deg": follow["max_deg"],
            "arrival_err_deg": arrival["arrival_err_deg"],
            "arrival_hold_ms": arrival["arrival_hold_ms"],
            "sched_fifo": bool(getattr(self.clock, "sched_fifo", False)),
            "pre_position_hold_ms": self._pre_position_hold_ms,
            "hold_ticks": int(self._hold_ticks),
            "planned_duration_s": self._planned_s,
        }

    def _finalize(self, t_start: float, evidence: List[str],
                  reason: str = "", next_action: str = "") -> Dict[str, Any]:
        """装配统一报告（**fake / live 同一份生成代码**）。"""
        event_list = self._events.to_list()
        status = report_mod.status_from_events(event_list)
        worst = self._events.worst_severity()
        if worst and worst != "INFO":
            # 用最严重事件的 reason / next_action 作为报告顶层说明
            worst_events = self._events.at_least(worst)
            if worst_events:
                reason = reason or worst_events[-1].reason
                next_action = next_action or worst_events[-1].next_action
        if status != contract.STATUS_OK and not reason:
            reason = "执行未完成（见 events）"
        metrics = self._build_metrics(t_start)
        return report_mod.build_report(
            status=status,
            reason=reason,
            next_action=next_action,
            evidence=evidence,
            mode=self.mode,
            metrics=metrics,
            events=event_list,
        )

    # -------------------------------------------------------------- 主入口
    def run(self, trajectory_path: str, allow_unverified: bool = False) -> Dict[str, Any]:
        """执行一条轨迹，返回**统一报告**（离线 fake 与真机 live 同一 schema）。

        参数
        ----
        trajectory_path
            轨迹 JSON 路径。
        allow_unverified
            ``True`` → 允许缺 ``verify`` 放行（**仅供离线假执行**，会写进
            evidence）。
        """
        self._events = events_mod.EventLog()
        self._ticks = 0
        self._hold_ticks = 0
        self._jitter_samples = []
        self._follow_errors = []
        self._pre_position_hold_ms = None
        self._arrival_err_deg = None
        self._arrival_hold_ms = None
        self._planned_s = None
        self._stream_actual_s = None
        self._feedback_error_logged = False

        t_start = self._now()
        evidence: List[str] = [
            "mode=%s transport=%s" % (self.mode, getattr(self.transport, "name", "?")),
        ]

        # ---------------- 阶段 A：预校验 ----------------
        if self._config is None:
            err = self._config_error or {}
            self._add_event(
                "execution_config_invalid", "NEEDS_PARAM",
                err.get("reason") or "执行配置不可用（workcell/rm_workcell.yaml::execution）",
                list(err.get("evidence") or []),
                err.get("next_action") or contract.NEXT_ACTION_FALLBACK,
            )
            return self._finalize(t_start, evidence)
        evidence.append("execution config: %s" % self._config.to_dict())

        trajectory = self._pre_validate(trajectory_path, allow_unverified, evidence)
        if trajectory is None:
            return self._finalize(t_start, evidence)

        try:
            factor = self._rad2deg()
        except _BridgeUnavailable as exc:
            self._add_event(
                "unit_conversion_unavailable", "NEEDS_PARAM", str(exc), [],
                contract.NEXT_ACTION_HW_BRIDGE_PATH,
            )
            return self._finalize(t_start, evidence)

        try:
            grid_t, grid_p = resample.resample_to_grid(
                trajectory["times"], trajectory["points"], self._config.period_s
            )
        except ValueError as exc:
            self._add_event(
                "resample_failed", "NEEDS_PARAM",
                "轨迹重采样到 %.0f ms 栅格失败: %s" % (self._config.period_s * 1000.0, exc),
                [], "修复轨迹时间轴（严格递增、与点位等长）后重跑",
            )
            return self._finalize(t_start, evidence)

        self._planned_s = metrics_mod.duration_s(grid_t[0], grid_t[-1])
        evidence.append("栅格点数=%d period_s=%.4f planned_duration_s=%s"
                        % (len(grid_t), self._config.period_s, self._planned_s))

        # ---------------- 阶段 B：预就位 ----------------
        goal_start_deg = self._to_deg(grid_p[0], factor)
        pre_result = self.transport.preposition(goal_start_deg,
                                                self._config.pre_position_speed)
        if pre_result.get("status") != contract.STATUS_OK:
            self._transport_failure("preposition", pre_result)
            return self._finalize(t_start, evidence)

        reached, hold_ms, reason = self._wait_pre_position(goal_start_deg)
        if not reached:
            self._add_event(
                "pre_position_not_reached", "NEEDS_HUMAN", reason,
                [
                    "pre_position_tol_deg=%.3f" % contract.PRE_POSITION_TOL_DEG,
                    "pre_position_hold_ms=%.0f" % contract.PRE_POSITION_HOLD_MS,
                    "pre_position_timeout_s=%.1f" % self.pre_position_timeout_s,
                ],
                "确认机械臂已上电且可运动、反馈数据可信后重跑"
                "（必要时先用 `python3 -m rmctl.cli connect` 建链）",
            )
            return self._finalize(t_start, evidence)
        self._pre_position_hold_ms = hold_ms
        self._add_event(
            "pre_position_ok", "INFO",
            "预就位到位（|q_act − q_goal| < %.1f° 持续 %.0f ms，实际 %.0f ms）"
            % (contract.PRE_POSITION_TOL_DEG, contract.PRE_POSITION_HOLD_MS,
               hold_ms or 0.0),
            ["pre_position_hold_ms=%.0f" % (hold_ms or 0.0)],
            "",
        )

        # ---------------- 阶段 C：tick 流式透传 ----------------
        self.stream_clock = self._make_stream_clock(self._config.period_s,
                                                     self._now())
        jitter_before = len(self.stream_clock.jitter_samples_ms)
        if self._stream_ticks(grid_p, factor) is None:
            return self._finalize(t_start, evidence)
        self._jitter_samples = list(self.stream_clock.jitter_samples_ms[jitter_before:])
        jitter_p99 = metrics_mod.percentile(self._jitter_samples, 99)
        self._add_event(
            "stream_done", "INFO",
            "栅格流式透传完成：%d 拍，每拍 %.0f ms" % (self._ticks,
                                                     self._config.period_s * 1000.0),
            ["ticks=%d" % self._ticks,
             "tick_jitter_p99_ms=%s" % ("None" if jitter_p99 is None
                                        else "%.3f" % jitter_p99)],
            "",
        )

        # ---------------- 阶段 D/E：到位判定 + 收尾 ----------------
        goal_deg = self._to_deg(grid_p[-1], factor)
        if not self._hold_and_arrival(goal_deg):
            return self._finalize(t_start, evidence)

        self._add_event(
            "arrival_ok", "INFO",
            "到位判定通过：|q_act − q_goal| = %.4f° < %.1f°，|q̇| < %.1f°/s，持续 %.0f ms"
            % (self._arrival_err_deg or 0.0, contract.ARRIVAL_TOL_DEG,
               contract.ARRIVAL_SPEED_DEG_S, self._arrival_hold_ms or 0.0),
            ["arrival_err_deg=%.4f" % (self._arrival_err_deg or 0.0),
             "arrival_hold_ms=%.0f" % (self._arrival_hold_ms or 0.0)],
            "",
        )
        # 阶段 E：限值恢复——本执行器**不下发限值**（限值下发属硬件层职责），
        # 显式记证据，不假装做过。
        evidence.append("限值恢复：执行器不下发限值（限值下发属 rm_hw_bridge 职责），"
                        "无需恢复动作")
        evidence.append("hold_ticks=%d（末点保持发送）" % self._hold_ticks)

        return self._finalize(
            t_start, evidence,
            reason="执行完成：%d 拍 10 ms 栅格透传 + 到位判定通过" % self._ticks,
        )
