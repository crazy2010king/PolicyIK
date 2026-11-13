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
import queue
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

from rm_executor import contract
from rm_executor import events as events_mod
from rm_executor import metrics as metrics_mod
from rm_executor import report as report_mod
from rm_executor import resample
from rm_executor.clock import Clock, ManualClock, TickClock

__all__ = [
    "ControllerWatchdog",
    "DEFAULT_ARRIVAL_TIMEOUT_S",
    "DEFAULT_POLL_PERIOD_S",
    "DEFAULT_PRE_POSITION_TIMEOUT_S",
    "DEFAULT_WATCHDOG_INTERVAL_TICKS",
    "DEFAULT_WATCHDOG_PERIOD_S",
    "DEFAULT_WATCHDOG_WAIT_TIMEOUT_S",
    "EXPECTED_TRAJECTORY_TYPE",
    "Executor",
    "ExecutorConfig",
]

#: 反馈轮询周期（秒）—— 预就位 / 到位等待阶段的采样节奏
DEFAULT_POLL_PERIOD_S = 0.010

#: 预就位等待上限（秒）—— 超时 → blocked
DEFAULT_PRE_POSITION_TIMEOUT_S = 30.0

#: 到位等待上限（秒）—— 超时 → blocked
DEFAULT_ARRIVAL_TIMEOUT_S = 2.0

#: 控制器「无规划」轨迹类型名（抢轨迹判据的唯一期望值）
EXPECTED_TRAJECTORY_TYPE = "RM_NO_PLANNING_E"

#: 控制器看门狗自检周期（秒）—— 落在 10–20 ms 区间（计划 §5.7）
DEFAULT_WATCHDOG_PERIOD_S = 0.015

#: 每多少个 tick 触发一次看门狗检查（2 × 10 ms = 20 ms）
DEFAULT_WATCHDOG_INTERVAL_TICKS = 2

#: fake 模式下等待看门狗完成一次检查的上限（秒）
DEFAULT_WATCHDOG_WAIT_TIMEOUT_S = 2.0


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


class ControllerWatchdog:
    """控制器抢轨迹看门狗——**独立线程**（绝不进 tick 线程）。

    为什么必须在别的线程：``transport.current_trajectory_type()`` 在真机上
    是一次 SDK/TCP 往返，放进 tick 线程会把往返延迟直接打进 10 ms 栅格的
    抖动里（p99 < 2 ms 的红线当场破掉）。因此本类把查询整体搬到一个
    后台线程，tick 线程**只读一个 Event 标志位**。

    线程模型：

    * 后台线程按 ``period_s``（10–20 ms）自转查询；
    * tick 线程可以「请求一次检查」（:meth:`request_check`），但 live 模式下
      **只置事件、不等待**（绝不在 tick 线程里等 SDK 往返）；
    * fake 模式允许等待（:meth:`wait_check`）——换取**确定性重放**，等待
      发生在离线假传输上，不产生真机延迟。

    判据：``type_name`` 不是 :data:`EXPECTED_TRAJECTORY_TYPE`
    → 判为「控制器抢轨迹」→ :attr:`conflict` 置位 + 回调（立即停机）。
    """

    def __init__(self,
                 transport: Any,
                 expected_type_name: str = EXPECTED_TRAJECTORY_TYPE,
                 period_s: float = DEFAULT_WATCHDOG_PERIOD_S,
                 on_conflict: Optional[Callable[[str, Dict[str, Any]], None]] = None,
                 on_query_failed: Optional[Callable[[str, List[str]], None]] = None,
                 on_unknown: Optional[Callable[[Dict[str, Any]], None]] = None,
                 max_checks: int = 4096,
                 ) -> None:
        self.transport = transport
        self.expected_type_name = str(expected_type_name)
        self.period_s = float(period_s)
        self.on_conflict = on_conflict
        self.on_query_failed = on_query_failed
        self.on_unknown = on_unknown

        #: 每次检查的记录（线程名 / 观测到的类型 / 查询状态）——可事后审计
        #: （**有上限**：长跑不无界增长；总数记在 :attr:`total_checks`）
        self.checks: List[Dict[str, Any]] = []
        self.max_checks = max(1, int(max_checks))
        self.total_checks = 0
        #: 抢轨迹已确认（线程安全标志位，tick 线程只读）
        self.conflict = threading.Event()
        self.last_error = ""

        self._stop = threading.Event()
        self._trigger = threading.Event()
        self._lock = threading.Lock()
        self._check_count = 0
        self._thread: Optional[threading.Thread] = None

    # -- 线程生命周期 -----------------------------------------------------
    def start(self) -> None:
        """启动后台看门狗线程（幂等）。"""
        if self._thread is not None:
            return
        self._stop.clear()
        thread = threading.Thread(
            target=self._loop, name="rm-controller-watchdog", daemon=True
        )
        self._thread = thread
        thread.start()

    def stop(self, timeout: float = 1.0) -> None:
        """停止后台线程（幂等；阻塞等待退出，最长 ``timeout`` 秒）。"""
        self._stop.set()
        self._trigger.set()
        thread = self._thread
        if thread is not None:
            thread.join(timeout)
        self._thread = None

    @property
    def running(self) -> bool:
        """后台线程是否在跑。"""
        return self._thread is not None and self._thread.is_alive()

    @property
    def check_count(self) -> int:
        """已完成的检查次数（含已被上限淘汰的记录）。"""
        return self.total_checks

    def request_check(self) -> None:
        """请求一次检查（**不等**）——tick 线程唯一允许做的动作。"""
        self._trigger.set()

    def wait_check(self, target_count: int, timeout: float = DEFAULT_WATCHDOG_WAIT_TIMEOUT_S
                   ) -> bool:
        """等待检查次数达到 ``target_count``；超时返回 ``False``。

        只在 **fake 模式**（离线确定性重放）使用；live 模式禁止在 tick
        线程里等待。
        """
        deadline = time.monotonic() + float(timeout)
        while self._check_count < int(target_count):
            if self._stop.is_set():
                return self._check_count >= int(target_count)
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.0005)
        return True

    # -- 检查逻辑 ---------------------------------------------------------
    def _loop(self) -> None:
        while not self._stop.is_set():
            self._trigger.wait(timeout=self.period_s)
            self._trigger.clear()
            if self._stop.is_set():
                break
            try:
                self.check()
            except Exception as exc:  # noqa: BLE001 - 看门狗线程绝不能死
                self.last_error = "看门狗内部异常（%s）: %s" % (type(exc).__name__, exc)

    def check(self) -> Optional[str]:
        """执行一次查询；返回观测到的轨迹类型名（无 → ``None``）。

        线程安全：所有对 :attr:`checks` 的写入都在锁内。
        """
        thread_name = threading.current_thread().name
        try:
            result = self.transport.current_trajectory_type()
        except Exception as exc:  # noqa: BLE001 - SDK 异常不得打死看门狗
            reason = "current_trajectory_type 调用异常（%s）: %s" % (type(exc).__name__, exc)
            self.last_error = reason
            self._record(thread_name, None, "error", reason)
            if self.on_query_failed:
                self.on_query_failed(reason, [repr(exc)])
            return None

        if not isinstance(result, dict):
            reason = "current_trajectory_type 返回格式不合法（%r）" % type(result).__name__
            self.last_error = reason
            self._record(thread_name, None, "error", reason)
            if self.on_query_failed:
                self.on_query_failed(reason, [repr(result)])
            return None

        status = result.get("status")
        if status != contract.STATUS_OK:
            reason = result.get("reason") or "current_trajectory_type 查询失败"
            self.last_error = reason
            self._record(thread_name, None, str(status), reason)
            if self.on_query_failed:
                self.on_query_failed(reason, list(result.get("evidence") or []))
            return None

        type_name = result.get("type_name", result.get("type"))
        if type_name is None:
            reason = "current_trajectory_type 未返回轨迹类型（无法判定是否抢轨迹）"
            self.last_error = reason
            self._record(thread_name, None, "unknown", reason)
            if self.on_unknown:
                self.on_unknown(result)
            return None

        observed = str(type_name)
        self.last_error = ""
        self._record(thread_name, observed, str(status), "")
        if observed != self.expected_type_name:
            self.conflict.set()
            if self.on_conflict:
                self.on_conflict(observed, result)
            return observed
        return observed

    def _record(self, thread_name: str, type_name: Optional[str],
                status: str, reason: str) -> None:
        with self._lock:
            self.checks.append({
                "thread": thread_name,
                "type_name": type_name,
                "status": status,
                "reason": reason,
            })
            if len(self.checks) > self.max_checks:
                del self.checks[: len(self.checks) - self.max_checks]
            self.total_checks += 1
            self._check_count = self.total_checks


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
                 watchdog_period_s: float = DEFAULT_WATCHDOG_PERIOD_S,
                 watchdog_interval_ticks: int = DEFAULT_WATCHDOG_INTERVAL_TICKS,
                 watchdog_wait_timeout_s: float = DEFAULT_WATCHDOG_WAIT_TIMEOUT_S,
                 expected_trajectory_type: str = EXPECTED_TRAJECTORY_TYPE,
                 ) -> None:
        self.transport = transport
        self.feedback = feedback
        self.ws = ws
        self.limits = limits
        self.poll_period_s = float(poll_period_s)
        self.pre_position_timeout_s = float(pre_position_timeout_s)
        self.arrival_timeout_s = float(arrival_timeout_s)
        self._stream_clock_factory = stream_clock_factory
        self.watchdog_period_s = float(watchdog_period_s)
        self.watchdog_interval_ticks = max(1, int(watchdog_interval_ticks))
        self.watchdog_wait_timeout_s = float(watchdog_wait_timeout_s)
        self.expected_trajectory_type = str(expected_trajectory_type)

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

        # -- 反馈闭环（T5）------------------------------------------------
        #: 控制器看门狗（**独立线程**；tick 线程只读 conflict 标志位）
        self._watchdog: Optional[ControllerWatchdog] = None
        #: 看门狗线程 → tick 线程的事件交接队列（线程安全）
        self._pending_events: "queue.Queue[Dict[str, Any]]" = queue.Queue()
        #: 最近一次观察到的新反馈帧计数（断流判据用）
        self._last_frames: Optional[int] = None
        #: 最近一次观察到新帧的时刻（主时钟域；断流 age 的基准）
        self._last_frame_time: Optional[float] = None
        #: 跟随误差 / 断流的「一次性」闩锁（避免每拍刷屏）
        self._follow_warn_active = False
        self._follow_stop_active = False
        self._stream_lag_active = False
        self._stream_lost_active = False
        #: tick 线程标识（证明 SDK 状态查询不在 tick 线程）
        self._tick_thread_ident: Optional[int] = None

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
        self._track_frame(snapshot)
        return snapshot

    # -------------------------------------------------- 反馈闭环（断流判据）
    def _track_frame(self, snapshot: Dict[str, Any]) -> None:
        """记录「是否来了新帧」（断流 age 的基准）。

        判据用**帧计数**而不是时间戳：帧计数与时钟域无关，ManualClock 下
        也可确定性重放。``_last_frame_time`` 落在**主时钟域**，因此
        ``age = now − last_frame_time`` 在 fake / live 两种模式下口径一致。
        """
        frames = snapshot.get("frames")
        if isinstance(frames, bool) or not isinstance(frames, int):
            return
        if self._last_frames is None or frames != self._last_frames:
            self._last_frames = frames
            self._last_frame_time = self._now()

    def _stream_age_ms(self) -> Optional[float]:
        """距上一次**新帧**的时长（毫秒）；未知 → ``None``。

        口径统一在**主时钟域**：fake 用虚拟时钟，live 用真实墙钟。
        """
        if self._last_frame_time is None:
            return None
        return max(0.0, (self._now() - self._last_frame_time) * 1000.0)

    def _check_stream_health(self) -> bool:
        """断流两档：>50 ms → RECOVERABLE；>200 ms → FATAL + 停机。

        返回 ``True`` = 已触发 FATAL 停机（调用方应立即中止执行）。
        """
        age_ms = self._stream_age_ms()
        if age_ms is None:
            return False
        if age_ms > contract.STREAM_STOP_MS:
            if not self._stream_lost_active:
                self._stream_lost_active = True
                self._add_event(
                    "feedback_stream_lost", "FATAL",
                    "反馈断流：距最后一帧已 %.0f ms（> %.0f ms 停机阈值）"
                    % (age_ms, contract.STREAM_STOP_MS),
                    [
                        "stream_age_ms=%.1f" % age_ms,
                        "stream_warn_ms=%d" % contract.STREAM_WARN_MS,
                        "stream_stop_ms=%d" % contract.STREAM_STOP_MS,
                        "frames=%s" % self._last_frames,
                    ],
                    "检查反馈链路（SDK 连接 / UDP 订阅 / 网线）后重跑；"
                    "执行器已调用 transport.stop()",
                )
                self._stop_transport()
            return True
        if age_ms > contract.STREAM_WARN_MS:
            if not self._stream_lag_active:
                self._stream_lag_active = True
                self._add_event(
                    "feedback_stream_lag", "RECOVERABLE",
                    "反馈滞后：距最后一帧已 %.0f ms（> %.0f ms 预警阈值）"
                    % (age_ms, contract.STREAM_WARN_MS),
                    [
                        "stream_age_ms=%.1f" % age_ms,
                        "stream_warn_ms=%d" % contract.STREAM_WARN_MS,
                        "stream_stop_ms=%d" % contract.STREAM_STOP_MS,
                        "frames=%s" % self._last_frames,
                    ],
                    "观察反馈链路是否恢复；若持续超过 %.0f ms 执行器将停机"
                    % contract.STREAM_STOP_MS,
                )
            return False
        self._stream_lag_active = False
        return False

    # -------------------------------------------------- 反馈闭环（跟随误差）
    def _check_follow_error(self, error: float, cmd_deg: Sequence[float],
                            snapshot: Optional[Dict[str, Any]]) -> bool:
        """跟随误差两档：>2° → RECOVERABLE；>5° → NEEDS_HUMAN + 暂停。

        返回 ``True`` = 已触发暂停（调用方应立即中止执行）。
        S03 **不实现降速策略**（降速 / 缓停处置属 S04 安全链路），
        这里只发事件 + 调 ``transport.pause()``。
        """
        actual = self._position_deg(snapshot)
        evidence = [
            "follow_error_deg=%.4f" % error,
            "follow_warn_deg=%.2f" % contract.FOLLOW_WARN_DEG,
            "follow_stop_deg=%.2f" % contract.FOLLOW_STOP_DEG,
            "cmd_deg=%s" % ["%.4f" % v for v in cmd_deg],
            "act_deg=%s" % (["%.4f" % v for v in actual] if actual else "None"),
        ]
        if error > contract.FOLLOW_STOP_DEG:
            if not self._follow_stop_active:
                self._follow_stop_active = True
                self._add_event(
                    "follow_error_exceeded", "NEEDS_HUMAN",
                    "跟随误差超限：%.3f° > %.1f°（已暂停执行）"
                    % (error, contract.FOLLOW_STOP_DEG),
                    evidence,
                    "立即人工确认机械臂状态（负载 / 碰撞 / 指令可达性）；"
                    "执行器已调用 transport.pause()，排查后重跑",
                )
                self._pause_transport()
            return True
        if error > contract.FOLLOW_WARN_DEG:
            if not self._follow_warn_active:
                self._follow_warn_active = True
                self._add_event(
                    "follow_error_warn", "RECOVERABLE",
                    "跟随误差预警：%.3f° > %.1f°" % (error, contract.FOLLOW_WARN_DEG),
                    evidence,
                    "观察机械臂跟随情况；若持续超过 %.1f° 执行器将暂停"
                    % contract.FOLLOW_STOP_DEG,
                )
            return False
        self._follow_warn_active = False
        return False

    # -------------------------------------------------- 安全动作（停机 / 暂停）
    def _stop_transport(self) -> None:
        """立即停机（硬红线）并留痕（失败也要留证据）。"""
        try:
            result = self.transport.stop()
        except Exception as exc:  # noqa: BLE001 - 停机异常也要留痕
            self._add_event(
                "stop_failed", "FATAL",
                "紧急停止调用异常（%s）: %s" % (type(exc).__name__, exc),
                [repr(exc)],
                "立即人工确认机械臂安全状态并断电",
            )
            return
        if isinstance(result, dict) and result.get("status") != contract.STATUS_OK:
            self._add_event(
                "stop_rejected", "FATAL",
                "紧急停止未成功：%s" % (result.get("reason") or "未知原因"),
                list(result.get("evidence") or []),
                result.get("next_action") or "立即人工确认机械臂安全状态并断电",
            )

    def _pause_transport(self) -> None:
        """暂停执行（跟随误差超限时的安全动作）；失败 → FATAL + 停机。"""
        try:
            result = self.transport.pause()
        except Exception as exc:  # noqa: BLE001
            self._add_event(
                "pause_failed", "FATAL",
                "暂停调用异常（%s）: %s" % (type(exc).__name__, exc),
                [repr(exc)],
                "立即人工确认机械臂安全状态并断电",
            )
            self._stop_transport()
            return
        if isinstance(result, dict) and result.get("status") != contract.STATUS_OK:
            self._add_event(
                "pause_rejected", "FATAL",
                "暂停未成功：%s" % (result.get("reason") or "未知原因"),
                list(result.get("evidence") or []),
                result.get("next_action") or "立即人工确认机械臂安全状态并断电",
            )
            self._stop_transport()

    # -------------------------------------------------- 看门狗（独立线程）
    def _start_watchdog(self) -> None:
        """启动控制器看门狗线程（**SDK 状态查询只在这里发生**）。"""
        if self._watchdog is not None:
            return
        self._watchdog = ControllerWatchdog(
            self.transport,
            expected_type_name=self.expected_trajectory_type,
            period_s=self.watchdog_period_s,
            on_conflict=self._on_controller_conflict,
            on_query_failed=self._on_controller_query_failed,
            on_unknown=self._on_controller_state_unknown,
        )
        self._watchdog.start()

    def _stop_watchdog(self) -> None:
        """停止看门狗线程（幂等；**对象保留**供事后审计检查记录）。"""
        if self._watchdog is not None:
            self._watchdog.stop()

    @property
    def watchdog_checks(self) -> List[Dict[str, Any]]:
        """看门狗的检查记录（线程名 / 观测到的轨迹类型）——事后审计用。"""
        if self._watchdog is None:
            return []
        return list(self._watchdog.checks)

    def _enqueue_event(self, type: str, severity: str, reason: str,
                       evidence: Optional[Sequence[Any]] = None,
                       next_action: str = "") -> None:
        """看门狗线程 → tick 线程的事件交接（**不直接改 EventLog**）。"""
        self._pending_events.put({
            "type": type,
            "severity": severity,
            "t": self._now(),
            "reason": reason,
            "evidence": list(evidence or []),
            "next_action": next_action,
        })

    def _drain_pending_events(self) -> int:
        """把看门狗线程产生的事件并入事件日志（返回并入条数）。"""
        drained = 0
        while True:
            try:
                item = self._pending_events.get_nowait()
            except queue.Empty:
                break
            self._events.add(
                type=item["type"],
                severity=item["severity"],
                t=item["t"],
                reason=item["reason"],
                evidence=item["evidence"],
                next_action=item["next_action"],
            )
            drained += 1
        return drained

    def _on_controller_conflict(self, type_name: str, result: Dict[str, Any]) -> None:
        """看门狗回调：**控制器抢轨迹** → FATAL + 立即停机。

        本回调运行在**看门狗线程**（停机必须立即发生，不能等 tick 线程）。
        """
        evidence = [
            "expected_type_name=%s" % self.expected_trajectory_type,
            "observed_type_name=%s" % type_name,
            "watchdog_thread=%s" % threading.current_thread().name,
            "tick_thread_ident=%s" % self._tick_thread_ident,
        ]
        stop_result = None
        try:
            stop_result = self.transport.stop()
        except Exception as exc:  # noqa: BLE001 - 停机异常也要留痕
            evidence.append("transport.stop 异常（%s）: %s" % (type(exc).__name__, exc))
        else:
            if isinstance(stop_result, dict):
                evidence.append("transport.stop status=%s" % stop_result.get("status"))
                if stop_result.get("status") != contract.STATUS_OK:
                    evidence.append("transport.stop reason=%s"
                                    % stop_result.get("reason"))
        self._enqueue_event(
            "controller_trajectory_conflict", "FATAL",
            "控制器抢轨迹：当前轨迹类型为 %s（期望 %s）"
            % (type_name, self.expected_trajectory_type),
            evidence,
            "立即人工确认控制器状态并停机后重跑（执行器已调用 transport.stop()）",
        )

    def _on_controller_query_failed(self, reason: str,
                                    evidence: List[str]) -> None:
        """看门狗回调：查询失败 → RECOVERABLE（**不伪造类型**）。"""
        self._enqueue_event(
            "controller_trajectory_query_failed", "RECOVERABLE",
            "控制器轨迹类型查询失败：%s" % reason,
            list(evidence or []),
            "检查控制器连接状态后重跑（本次执行无法确认是否抢轨迹）",
        )

    def _on_controller_state_unknown(self, result: Dict[str, Any]) -> None:
        """看门狗回调：查询成功但无类型字段 → RECOVERABLE。"""
        self._enqueue_event(
            "controller_trajectory_state_unknown", "RECOVERABLE",
            "控制器轨迹类型查询未返回类型字段（无法判定是否抢轨迹）",
            [repr(result)],
            "检查 rm_hw_bridge 的 current_trajectory_type 返回值后重跑",
        )

    def _watchdog_checkpoint(self) -> bool:
        """tick 线程里的看门狗检查点。

        live 模式：**只请求、不等待**（绝不在 tick 线程里等 SDK 往返）；
        fake 模式：允许等待一次检查完成，换取**确定性重放**。

        返回 ``False`` = 看门狗失灵（fake 模式等待超时）→ 调用方应停机。
        """
        watchdog = self._watchdog
        if watchdog is None:
            return True
        baseline = watchdog.check_count
        watchdog.request_check()
        if self.mode != "fake":
            return True
        if watchdog.wait_check(baseline + 1, timeout=self.watchdog_wait_timeout_s):
            return True
        self._add_event(
            "controller_watchdog_failed", "FATAL",
            "控制器看门狗未在 %.1f s 内完成检查（线程可能已退出）"
            % self.watchdog_wait_timeout_s,
            ["watchdog_check_count=%d" % watchdog.check_count,
             "watchdog_running=%s" % watchdog.running],
            "立即人工确认机械臂安全状态并断电",
        )
        self._stop_transport()
        return False

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
        for index, row_rad in enumerate(grid_p):
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
                # 跟随误差两档（>2° 预警 / >5° 暂停）
                if self._check_follow_error(error, cmd_deg, snapshot):
                    return None
            # ④ 断流两档（>50 ms 预警 / >200 ms 停机）
            if self._check_stream_health():
                return None
            # ⑤ 看门狗（控制器抢轨迹）：先并入已产生的事件，再看标志位
            self._drain_pending_events()
            if self._watchdog is not None and self._watchdog.conflict.is_set():
                return None
            self._ticks += 1
            # ⑥ 看门狗检查点（查询发生在**看门狗线程**，不在 tick 线程）
            if self._watchdog is not None and (
                    index % self.watchdog_interval_ticks == 0):
                if not self._watchdog_checkpoint():
                    return None
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
                if self._check_follow_error(position_error, goal_deg, snapshot):
                    return False
            if self._check_stream_health():
                return False
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
        watchdog_info = None
        if self._watchdog is not None:
            watchdog_info = (
                "watchdog: checks=%d conflict=%s expected_type_name=%s"
                % (self._watchdog.check_count, self._watchdog.conflict.is_set(),
                   self.expected_trajectory_type)
            )
            self._stop_watchdog()
        # 看门狗线程产生的事件在此并入（保证一条不丢）
        self._drain_pending_events()
        if watchdog_info:
            evidence.append(watchdog_info)
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

        # -- 反馈闭环状态重置（T5）-----------------------------------------
        self._pending_events = queue.Queue()
        self._watchdog = None
        self._last_frames = None
        self._last_frame_time = None
        self._follow_warn_active = False
        self._follow_stop_active = False
        self._stream_lag_active = False
        self._stream_lost_active = False
        self._tick_thread_ident = threading.get_ident()

        t_start = self._now()
        self._last_frame_time = t_start  # 断流 age 的起点
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
        # 控制器看门狗先行启动（**独立线程**：SDK 状态查询不进 tick 线程）
        self._start_watchdog()
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
