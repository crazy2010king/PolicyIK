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

**安全包接线（S04）**
------------------

本执行器**不再自己二分安全动作**：跟随误差 / 断流的**判定**与**动作**统一
交给安全包 ``rm_safety``（依赖方向 ``rm_hw_bridge < rm_safety < rm_executor
< rmctl``）：

* 阈值唯一来源 = :mod:`rm_executor.contract` 的冻结常量；策略开关唯一来源 =
  ``rm_safety.config.load_safety_config(ws)``；
* 动作唯一权威 = ``SafetySupervisor`` → ``SafetyActuator``（L1 降速 / L2 暂停
  / L3 缓停 / L4 急停），**消除双重动作**（不再「既 pause 又被安全线程
  再 stop」）；
* 控制器错误态（``arm_err`` / ``arm_current_status``）由**看门狗线程**采集并
  缓存，tick 线程**只读缓存**（零 SDK 进 tick 线程，红线同 P003 / P007）；
* S03 既有事件类型与退出码口径（0 / 3 / 4）**保持不变**，安全包额外产出
  ``safety_*`` / ``fault_*`` 事件；
* 安全层不可用 → **降级**就地二分并留 ``safety_layer_unavailable`` 事件
  （**不静默**）。
"""
from __future__ import annotations

import json
import math
import os
import queue
import sys
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Mapping, Optional, Sequence, Tuple

from rm_executor import contract
from rm_executor import events as events_mod
from rm_executor import interlock
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

#: 控制器「无规划」轨迹类型名（纯透传）
EXPECTED_TRAJECTORY_TYPE = "RM_NO_PLANNING_E"

#: CANFD 透传阶段**可接受**的控制器轨迹类型集合（抢轨迹判据）。
#:
#: 真机实测（2026-11-19，RM75-B 控制器 192.168.1.18）：透传通道下控制器上报的
#: 轨迹类型**随下发的 trajectory_mode 变化**，而不是恒为“纯透传”：
#:
#:   * ``trajectory_mode=0``（完全透传）→ ``RM_NO_PLANNING_E``
#:   * ``trajectory_mode=1``（曲线拟合，本项目默认）→ ``RM_SPLINE_CURVE_MOTION_PLANNING_E``
#:   * 滤波/混合                        → ``RM_BLEND_TRAJECTORY_PLANNING_E``
#:
#: 若把期望值只写成 RM_NO_PLANNING_E，会把**我们自己下发的透传**误判为“控制器抢
#: 轨迹”（FATAL + 急停，实测连跑三次分别被判成 blend / spline 两种形态）。
#:
#: 真正需要报警的是**别人下发的轨迹**类型（见 :data:`CONFLICT_TRAJECTORY_TYPES`）。
EXPECTED_TRAJECTORY_TYPES = (
    "RM_NO_PLANNING_E",
    "RM_BLEND_TRAJECTORY_PLANNING_E",
    "RM_SPLINE_CURVE_MOTION_PLANNING_E",
    "RM_CARTESIAN_LINEAR_PLANNING_E",
    "RM_CARTESIAN_ARC_PLANNING_E",
)

#: 明确属于「**别人下发的轨迹**」的控制器轨迹类型 → 判为真抢轨迹。
#:
#: * ``RM_JOINT_SPACE_PLANNING_E``：另一客户端用 ``rm_movej`` 下发的关节空间轨迹
#:   （本项目自己的预就位走 rm_movej，但那是**阶段 B**，看门狗那时还没启动）；
#: * ``RM_TRAJECTORY_REPLAY_PLANNING_E``：轨迹回放 —— 本项目从不下发。
#:
#: 其余（纯透传 / 混合 / 样条 / 笛卡尔直线 / 笛卡尔圆弧）都是**我们自己这条
#: CANFD 透传**在曲线拟合（``trajectory_mode=1``）下可能出现的形态 —— 真机实测
#: 同一段透传里就先后出现过 spline 与 cartesian-arc 两种上报（见
#: ``docs/RM75-B真机验证阶梯.md`` §5.2）。把它们当抢轨迹会导致**正常的透传被判 FATAL
#: 并急停**，因此排除在红��之外。
CONFLICT_TRAJECTORY_TYPES = (
    "RM_JOINT_SPACE_PLANNING_E",
    "RM_TRAJECTORY_REPLAY_PLANNING_E",
)

#: 「抢轨迹」判据需要**连续**看到几次外部轨迹类型才成立（抗瞬态）。
#:
#: 真机实测：CANFD 透传期间控制器的轨迹类型会在若干形态之间切换（纯透传/样条/
#: 笛卡尔圆弧…），单次采样不足以判定"被抢"。看门狗周期 15 ms、每 2 拍查一次
#: → 连续 3 次 ≈ 30 ms，足以区分瞬态与持续。
DEFAULT_CONFLICT_CONFIRMATIONS = 3

#: 控制器看门狗自检周期（秒）—— 落在 10–20 ms 区间（计划 §5.7）
DEFAULT_WATCHDOG_PERIOD_S = 0.015

#: 每多少个 tick 触发一次看门狗检查（2 × 10 ms = 20 ms）
DEFAULT_WATCHDOG_INTERVAL_TICKS = 2

#: fake 模式下等待看门狗完成一次检查的上限（秒）
DEFAULT_WATCHDOG_WAIT_TIMEOUT_S = 2.0


class _BridgeUnavailable(RuntimeError):
    """硬件适配层不可用（取不到 ``RAD2DEG`` 单点换算常量）。"""


def _format_evidence(items: Any) -> List[str]:
    """把安全包的证据（dict 列表）格式化成**字符串**列表（事件日志口径）。

    事件日志的 ``evidence`` 口径是字符串列表（见 :mod:`rm_executor.events`）；
    安全包的证据是结构化 dict（字段固定、可离线逐条核对）。这里用
    ``json.dumps(sort_keys=True)`` **保字段**地降级成字符串——不丢字段、
    不改口径。
    """
    out: List[str] = []
    for item in items or []:
        if isinstance(item, str):
            out.append(item)
        elif isinstance(item, Mapping):
            out.append(json.dumps(dict(item), ensure_ascii=False,
                                  sort_keys=True, default=str))
        else:
            out.append(str(item))
    return out


def _import_safety_modules() -> Optional[Dict[str, Any]]:
    """惰性导入安全层 ``rm_safety``（失败 → ``None``，**不抛裸异常**）。

    依赖方向固定为 ``rm_hw_bridge < rm_safety < rm_executor < rmctl``：
    执行层**依赖**安全层，安全层**不反向依赖**执行层。

    本函数是执行层**唯一**触碰 ``rm_safety`` 的入口（便于静态审计
    「安全层不被反向依赖」）。导入失败时按「安全层不可用」降级处理，
    并由调用方**留证据**（不静默）。
    """
    try:
        import rm_safety  # noqa: F401
    except Exception:  # noqa: BLE001 - 兜底：兄弟包目录
        sibling = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "rm_safety",
        )
        if os.path.isdir(sibling) and sibling not in sys.path:
            sys.path.insert(0, sibling)
    try:
        from rm_safety import config as safety_config
        from rm_safety import error_codes as safety_error_codes
        from rm_safety import levels as safety_levels
        from rm_safety import policy as safety_policy
        from rm_safety.supervisor import SafetySupervisor
    except Exception:  # noqa: BLE001 - 安全层不可用 → 降级（不抛裸异常）
        return None
    return {
        "config": safety_config,
        "error_codes": safety_error_codes,
        "levels": safety_levels,
        "policy": safety_policy,
        "supervisor": SafetySupervisor,
    }


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
                 expected_type_names: Optional[Sequence[str]] = None,
                 period_s: float = DEFAULT_WATCHDOG_PERIOD_S,
                 on_conflict: Optional[Callable[[str, Dict[str, Any]], None]] = None,
                 on_query_failed: Optional[Callable[[str, List[str]], None]] = None,
                 on_unknown: Optional[Callable[[Dict[str, Any]], None]] = None,
                 max_checks: int = 4096,
                 conflict_confirmations: int = DEFAULT_CONFLICT_CONFIRMATIONS,
                 ) -> None:
        self.transport = transport
        self.expected_type_name = str(expected_type_name)
        #: 可接受的轨迹类型集合（缺省 = 仅 {expected_type_name}，保持旧语义）
        self.expected_type_names = tuple(expected_type_names) if expected_type_names else (
            (self.expected_type_name,))
        self.period_s = float(period_s)
        self.on_conflict = on_conflict
        self.on_query_failed = on_query_failed
        self.on_unknown = on_unknown

        #: 每次检查的记录（线程名 / 观测到的类型 / 查询状态）——可事后审计
        #: （**有上限**：长跑不无界增长；总数记在 :attr:`total_checks`）
        self.checks: List[Dict[str, Any]] = []
        self.max_checks = max(1, int(max_checks))
        self.total_checks = 0
        #: 「抢轨迹」需要连续确认几次（抗瞬态；1 = 旧行为，立即判）
        self.conflict_confirmations = max(1, int(conflict_confirmations))
        #: 连续看到"外部轨迹类型"的次数
        self.consecutive_conflicts = 0
        #: 抢轨迹已确认（线程安全标志位，tick 线程只读）
        self.conflict = threading.Event()
        self.last_error = ""

        self._stop = threading.Event()
        self._trigger = threading.Event()
        self._lock = threading.Lock()
        self._check_count = 0
        self._thread: Optional[threading.Thread] = None

        # -- 控制器错误态缓存（S04）----------------------------------------
        #: 控制器错误态缓存（**看门狗线程写、tick 线程只读**）
        self._state_lock = threading.Lock()
        self._latest_state: Optional[Dict[str, Any]] = None
        self._state_query_count = 0
        self._state_error = ""

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

    # -- 控制器错误态（只读缓存，tick 线程零 SDK）-----------------------
    @property
    def latest_state(self) -> Optional[Dict[str, Any]]:
        """最近一次控制器错误态快照（**只读拷贝**）。

        tick 线程**只读这里**，绝不调 ``transport.controller_state()``
        （沿用「零 SDK 进 tick 线程」红线，与 ``type_query_threads``
        断言同风格）。
        """
        with self._state_lock:
            return dict(self._latest_state) if self._latest_state else None

    @property
    def state_query_count(self) -> int:
        """控制器错误态采集次数（用于审计「查询确实发生过」）。"""
        with self._state_lock:
            return self._state_query_count

    @property
    def state_error(self) -> str:
        """最近一次控制器错误态采集失败的原因（空 = 无失败）。"""
        with self._state_lock:
            return self._state_error

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
            try:
                self._poll_controller_state()
            except Exception as exc:  # noqa: BLE001 - 采集失败也不得打死看门狗
                with self._state_lock:
                    self._state_error = "控制器错误态采集异常（%s）: %s" % (
                        type(exc).__name__, exc)

    def _poll_controller_state(self) -> None:
        """采集一次控制器错误态（**只在看门狗线程**）。

        结果缓存到 :attr:`latest_state`；失败只留错误串，**不伪造**
        ``arm_err`` / ``arm_current_status``（不返回空错误码冒充「无错误」）。
        """
        try:
            result = self.transport.controller_state()
        except Exception as exc:  # noqa: BLE001 - SDK 异常不得打死看门狗
            with self._state_lock:
                self._state_error = "controller_state 调用异常（%s）: %s" % (
                    type(exc).__name__, exc)
            return
        if not isinstance(result, dict):
            with self._state_lock:
                self._state_error = ("controller_state 返回格式不合法（%s）"
                                     % type(result).__name__)
            return
        with self._state_lock:
            self._latest_state = dict(result)
            self._state_query_count += 1
            if result.get("status") == "ok":
                self._state_error = ""
            else:
                self._state_error = str(result.get("reason") or "控制器错误态查询失败")

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
        if observed not in self.expected_type_names:
            # 外部轨迹类型：需**连续**确认才判为抢轨迹（瞬态只留证据、不停机）
            self.consecutive_conflicts += 1
            self._record(thread_name, observed, str(status),
                         "外部轨迹类型 %s（连续 %d/%d）"
                         % (observed, self.consecutive_conflicts,
                            self.conflict_confirmations))
            if self.consecutive_conflicts >= self.conflict_confirmations:
                self.conflict.set()
                if self.on_conflict:
                    self.on_conflict(observed, result)
            return observed
        self.consecutive_conflicts = 0
        self._record(thread_name, observed, str(status), "")
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
                 expected_trajectory_types: Optional[Sequence[str]] = None,
                 gate: Any = None,
                 motion_gate: Any = None,
                 ) -> None:
        self.transport = transport
        self.feedback = feedback
        self.ws = ws
        self.limits = limits
        #: 安全闸门（进程外强制停机通道；鸭子类型：``stop_requested`` 属性
        #: + ``add_handler`` / ``requested_level``）——``None`` → 不启用
        self.gate = gate
        #: **夹爪运动闸门**（鸭子类型：``check() -> Optional[dict]`` + ``state``
        #: 属性）——``None`` → **不启用互锁**（既有行为完全不变）。
        #: 非 IDLE 时在**阶段 B 之前**与**阶段 C 起流之前**各查一次，
        #: 非 IDLE → **拒绝下发任何运动指令**（并留 ``gripper_interlock_blocked``
        #: 事件证据）。夹爪动作是离散、慢、阻塞的，绝不跑进 10 ms tick 线程，
        #: 因此互锁只落在**阶段边界**上。
        self.motion_gate = motion_gate
        self.poll_period_s = float(poll_period_s)
        self.pre_position_timeout_s = float(pre_position_timeout_s)
        self.arrival_timeout_s = float(arrival_timeout_s)
        self._stream_clock_factory = stream_clock_factory
        self.watchdog_period_s = float(watchdog_period_s)
        self.watchdog_interval_ticks = max(1, int(watchdog_interval_ticks))
        self.watchdog_wait_timeout_s = float(watchdog_wait_timeout_s)
        self.expected_trajectory_type = str(expected_trajectory_type)
        #: 可接受的轨迹类型集合（透传阶段：纯透传 + 混合规划）
        self.expected_trajectory_types = (
            tuple(expected_trajectory_types) if expected_trajectory_types
            else tuple(EXPECTED_TRAJECTORY_TYPES))

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

        # -- 安全闸门（S04/T7）---------------------------------------------
        #: 外部闸门是否已触发停机（**停止标志**：tick 下一拍退出）
        self._external_stop = False
        #: 闸门停机只触发一次（幂等）
        self._external_stop_done = False
        #: 闸门线程与 tick 线程的互斥（等级/事件一致性）
        self._gate_lock = threading.Lock()

        # -- 安全包接线（S04）-------------------------------------------
        self._init_safety()

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

    # ------------------------------------------------------- 夹爪互锁闸门
    def _check_motion_gate(self, stage: str) -> Optional[Dict[str, Any]]:
        """夹爪 ↔ 运动互锁检查（**只在阶段边界调用**，绝不进 tick 线程）。

        返回
        ----
        ``None``
            放行（未启用互锁，或闸门为 IDLE）。
        ``dict``
            被拒绝：``{"state", "reason", "next_action", "evidence"}``。
        """
        gate = self.motion_gate
        if gate is None:
            return None
        try:
            result = gate.check()
        except Exception as exc:  # noqa: BLE001 - 闸门异常一律保守拒绝
            return {
                "state": "UNKNOWN",
                "reason": "夹爪闸门检查异常: %s: %s" % (type(exc).__name__, exc),
                "next_action": "检查夹爪闸门对象后重跑（闸门异常时保守拒绝运动）",
                "evidence": ["stage=%s" % stage,
                             "motion_gate_check_error=%s: %s"
                             % (type(exc).__name__, exc)],
            }
        if result is None:
            return None
        # 鸭子类型：check() 返回 dict（state / reason / next_action）
        state = str(result.get("state") or getattr(gate, "state", "") or "UNKNOWN")
        reason = str(result.get("reason") or getattr(gate, "reason", "") or "")
        next_action = str(result.get("next_action") or "")
        evidence = [
            "stage=%s" % stage,
            "motion_gate_state=%s" % state,
            "motion_gate_reason=%s" % reason,
        ]
        # 闸门对象若带最近夹爪快照字段则一并带上（可选，缺省不编造）
        for attr in ("last_state", "last_snapshot", "snapshot"):
            snap = getattr(gate, attr, None)
            if snap is not None:
                try:
                    import json as _json
                    evidence.append("motion_gate_%s=%s"
                                    % (attr, _json.dumps(snap, ensure_ascii=False,
                                                        default=str)))
                except Exception:  # noqa: BLE001 - 快照序列化失败不致命
                    evidence.append("motion_gate_%s=%r" % (attr, snap))
                break
        if not next_action:
            next_action = ("等待夹爪动作结束并完成判定后重跑"
                           "（或执行 `python3 -m rmctl gripper judge` 确认夹爪状态）")
        return {
            "state": state,
            "reason": reason,
            "next_action": next_action,
            "evidence": evidence,
        }

    def _block_by_motion_gate(self, stage: str) -> Optional[Dict[str, Any]]:
        """闸门非 IDLE → 记 ``gripper_interlock_blocked`` 事件并返回拒绝信息。

        返回 ``None`` 表示放行；否则返回拒绝字典（调用方据此**不下发运动**）。
        """
        blocked = self._check_motion_gate(stage)
        if blocked is None:
            return None
        state = str(blocked.get("state") or "UNKNOWN")
        if state == interlock.STATE_GATE_UNAVAILABLE:
            # **fail-closed**（S12/T4）：闸门/配置不可用 → 不是「夹爪动作期间」
            reason = ("夹爪闸门不可用（%s，stage=%s，闸门状态=%s）"
                      % (interlock.REASON_GATE_UNAVAILABLE, stage, state))
        else:
            reason = "夹爪动作期间禁止运动（stage=%s，闸门状态=%s）" % (stage, state)
        self._add_event(
            "gripper_interlock_blocked", "NEEDS_HUMAN",
            reason,
            blocked["evidence"],
            blocked["next_action"],
        )
        return blocked

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
        """断流两档：>50 ms → RECOVERABLE；>200 ms → FATAL。

        返回 ``True`` = 已判定断流（调用方应立即中止执行）。

        **动作权威（S04）**：本方法**只发事件、不动动作**——急停动作由安全包
        （``SafetySupervisor`` → ``SafetyActuator``）统一发出，避免
        「执行器就地 stop + 安全线程再 stop」的双重动作。仅当安全层不可用
        （降级）时才就地急停（并留证据）。
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
                if self._safety is None:
                    # 降级：安全层不可用 → 就地急停（并留证据）
                    self._add_event(
                        "safety_layer_unavailable_fallback", "FATAL",
                        "安全层不可用 → 执行器就地急停（降级路径）",
                        ["fallback=stop",
                         "safety_error=%s" % (self._safety_error or "未知原因")],
                        "立即人工确认机械臂安全状态并断电",
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
        """跟随误差两档：>2° → RECOVERABLE；>5° → NEEDS_HUMAN。

        返回 ``True`` = 已判定超限（调用方应立即中止执行）。

        **动作权威（S04）**：本方法**只发事件、不动动作**——暂停动作由安全包
        （``SafetySupervisor`` → ``SafetyActuator``）统一发出，避免
        「既 pause 又被安全线程再 stop」的双重动作。仅当安全层不可用
        （降级）时才就地暂停（并留证据）。
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
                if self._safety is None:
                    # 降级：安全层不可用 → 就地暂停（并留证据）
                    self._add_event(
                        "safety_layer_unavailable_fallback", "NEEDS_HUMAN",
                        "安全层不可用 → 执行器就地暂停（降级路径）",
                        ["fallback=pause",
                         "safety_error=%s" % (self._safety_error or "未知原因")],
                        "立即人工确认机械臂安全状态并断电",
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

    # ------------------------------------------------- 安全包接线（S04）
    def _init_safety(self) -> None:
        """构造安全监督器（**唯一动作权威**）。

        * 阈值唯一来源 = :mod:`rm_executor.contract` 的冻结常量
          （``FOLLOW_WARN_DEG`` / ``FOLLOW_STOP_DEG`` /
          ``STREAM_WARN_MS`` / ``STREAM_STOP_MS``）；
        * 策略开关唯一来源 = ``rm_safety.config.load_safety_config(ws)``
          （读 ``workcell/rm_workcell.yaml::safety``）；
        * 声明式错误码表 = ``rm_safety.error_codes.load_error_codes()``。

        安全层不可用 → **不静默**：记 ``safety_layer_unavailable`` 事件并
        退回 S03 的就地二分（降级路径，仍会停机 / 暂停）。
        """
        self._safety_modules = _import_safety_modules()
        self._safety = None
        self._safety_thresholds = None
        self._safety_error = ""
        self._safety_config: Dict[str, Any] = {}
        self._safety_config_evidence: List[str] = []
        self._safety_error_codes = None
        self._safety_error_code_warnings: List[str] = []
        self._self_triggered_stop = False
        #: 控制器停止态的起始时刻（单调时钟；不在停止态时为 None）
        self._stop_state_since_s = None
        #: 当前是否正在下发运动指令（阶段 C 流式 / 阶段 D 保持）
        self._commanding_motion = False
        self._limit_check = "unavailable"
        self._limit_check_reason = ""
        self._limits_cache: Optional[Dict[str, Any]] = None
        self._limits_error = ""

        if self._safety_modules is None:
            self._safety_error = "无法导入安全层 rm_safety（安全动作权威不可用）"
            return
        policy = self._safety_modules["policy"]
        config_mod = self._safety_modules["config"]
        error_codes_mod = self._safety_modules["error_codes"]

        try:
            thresholds = policy.SafetyThresholds(
                follow_warn_deg=contract.FOLLOW_WARN_DEG,
                follow_stop_deg=contract.FOLLOW_STOP_DEG,
                stream_warn_ms=contract.STREAM_WARN_MS,
                stream_stop_ms=contract.STREAM_STOP_MS,
                protection_grace_ms=contract.PROTECTION_GRACE_MS,
            )
        except Exception as exc:  # noqa: BLE001
            self._safety_error = "安全阈值构造失败（%s）: %s" % (type(exc).__name__, exc)
            return

        cfg_res = config_mod.load_safety_config(self.ws)
        self._safety_config_evidence = list(cfg_res.get("evidence") or [])
        if cfg_res.get("status") == config_mod.STATUS_OK:
            cfg = dict(cfg_res.get("config") or {})
            self._safety_config = cfg
            try:
                thresholds = policy.SafetyThresholds(
                    follow_warn_deg=contract.FOLLOW_WARN_DEG,
                    follow_stop_deg=contract.FOLLOW_STOP_DEG,
                    stream_warn_ms=contract.STREAM_WARN_MS,
                    stream_stop_ms=contract.STREAM_STOP_MS,
                    protection_grace_ms=contract.PROTECTION_GRACE_MS,
                    l1_speed_scale=cfg.get("l1_speed_scale", 0.5),
                    enable_l1_slowdown=bool(cfg.get("enable_l1_slowdown", True)),
                    enable_l2_pause_on_stream_lag=bool(
                        cfg.get("enable_l2_pause_on_stream_lag", False)),
                )
            except Exception as exc:  # noqa: BLE001
                self._safety_error = "安全配置非法（%s）: %s" % (type(exc).__name__, exc)
                return
            try:
                table = error_codes_mod.load_error_codes(
                    str(cfg.get("error_codes_path") or ""))
                self._safety_error_codes = table
                self._safety_error_code_warnings = list(table.warnings)
            except Exception as exc:  # noqa: BLE001
                self._safety_error_codes = None
                self._safety_error_code_warnings = [
                    "错误码表读取失败（%s）: %s" % (type(exc).__name__, exc)]
        else:
            # 安全配置不可用 → 用契约冻结阈值 + 默认策略开关（**留证据**）
            self._safety_error = "安全配置不可用：%s" % (cfg_res.get("reason") or "未知原因")
            self._safety_config_evidence.append(
                "safety config blocked → 使用契约冻结阈值 + 默认策略开关"
                "（l1_speed_scale=0.5, enable_l1_slowdown=true, "
                "enable_l2_pause_on_stream_lag=false）")

        self._safety_thresholds = thresholds
        self._safety = self._create_supervisor()

    def _create_supervisor(self) -> Any:
        """新建安全监督器（每次 run 重建：等级单调状态**不跨 run** 泄漏）。"""
        if self._safety_modules is None or self._safety_thresholds is None:
            return None
        supervisor_cls = self._safety_modules["supervisor"]
        return supervisor_cls(self.transport, self._safety_thresholds,
                              on_event=self._on_safety_event)

    # -------------------------------------------------------- 安全信号
    def _watchdog_state(self) -> Optional[Dict[str, Any]]:
        """看门狗缓存的控制器错误态（**只读缓存**，tick 线程零 SDK）。"""
        watchdog = self._watchdog
        if watchdog is None:
            return None
        return watchdog.latest_state

    def _load_limits(self) -> Optional[Dict[str, Any]]:
        """取声明限位（SI）；不可用 → ``None``（**不伪造限值**）。"""
        if self.limits is not None:
            return self.limits
        if self._limits_cache is not None:
            return self._limits_cache
        try:
            from rm_hw_bridge import limits as hw_limits  # 惰性导入：唯一限值源
            self._limits_cache = hw_limits.load_joint_limits(self.ws)
        except Exception as exc:  # noqa: BLE001
            self._limits_error = ("读取声明限值失败（%s）: %s"
                                  % (type(exc).__name__, exc))
            return None
        return self._limits_cache

    def _limit_violations(self, snapshot: Optional[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """运行期限值越界条目（实际关节角（度）vs 声明限位）。

        * 限值唯一来源 = ``rm_hw_bridge.limits.load_joint_limits()``（SI）；
        * SI → 度换算收敛在 :meth:`_rad2deg` **单点**；
        * 限值不可用 → **跳过超限判据**并把 ``limit_check`` 置为
          ``unavailable``（**不静默**：报告 evidence 里会记下原因）。
        """
        actual = self._position_deg(snapshot)
        if actual is None:
            self._limit_check = "unavailable"
            self._limit_check_reason = "反馈快照无关节角数据 → 跳过超限判据"
            return []
        limits = self._load_limits()
        if not limits:
            self._limit_check = "unavailable"
            self._limit_check_reason = self._limits_error or "声明限值不可用"
            return []
        try:
            factor = self._rad2deg()
        except _BridgeUnavailable as exc:
            self._limit_check = "unavailable"
            self._limit_check_reason = str(exc)
            return []
        violations: List[Dict[str, Any]] = []
        for index, (name, item) in enumerate(limits.items()):
            if index >= len(actual):
                break
            if not isinstance(item, Mapping):
                continue
            try:
                lower = float(item["min_pos"]) * factor
                upper = float(item["max_pos"]) * factor
            except (KeyError, TypeError, ValueError):
                continue
            pos = float(actual[index])
            if pos < lower:
                violations.append({"joint": name, "observed_deg": pos,
                                   "limit_deg": lower, "margin": pos - lower})
            elif pos > upper:
                violations.append({"joint": name, "observed_deg": pos,
                                   "limit_deg": upper, "margin": pos - upper})
        self._limit_check = "ok"
        self._limit_check_reason = ""
        return violations

    def _safety_signals(self, follow_error: Optional[float],
                        snapshot: Optional[Dict[str, Any]]) -> Dict[str, Any]:
        """组装安全判据输入（**纯读**：不产生任何 SDK 调用）。"""
        signals: Dict[str, Any] = {
            "follow_error_deg": follow_error,
            "stream_age_ms": self._stream_age_ms(),
            "limit_violations": self._limit_violations(snapshot),
            "self_triggered_stop": bool(self._self_triggered_stop),
            "error_code_map": self._safety_error_codes,
        }
        arm_err: List[int] = []
        arm_status: Optional[int] = None
        state = self._watchdog_state()
        if state and state.get("status") == contract.STATUS_OK:
            raw_err = state.get("arm_err")
            if isinstance(raw_err, (list, tuple)):
                for item in raw_err:
                    if isinstance(item, bool) or not isinstance(item, (int, float)):
                        continue
                    arm_err.append(int(item))
            raw_status = state.get("arm_current_status")
            if isinstance(raw_status, int) and not isinstance(raw_status, bool):
                arm_status = int(raw_status)
        signals["arm_err"] = arm_err
        signals["arm_current_status"] = arm_status
        # 停止态**持续时长**（毫秒）与"是否正在下发运动指令"——
        # 保护停判据需要这两项佐证（普通停止态不算保护停，见 rm_safety.policy）。
        signals["protection_stop_ms"] = self._protection_stop_ms(arm_status)
        signals["commanding_motion"] = bool(self._commanding_motion)
        return signals

    def _protection_states(self) -> Tuple[int, ...]:
        """保护停状态集合（来自安全层唯一真值；取不到时回退到 (9, 10)）。"""
        module = (self._safety_modules or {}).get("policy")
        states = getattr(module, "ARM_STATUS_PROTECTION_STATES", None)
        return tuple(states) if states else (9, 10)

    def _protection_stop_ms(self, arm_status: Optional[int]) -> Optional[float]:
        """控制器停止态已持续多久（毫秒）；不在停止态 → ``None``。

        真机实测（2026-11-19）：``RM_STOP_E(9)`` 是**普通停止态**（空闲 / 预就位刚
        结束 / 透传刚下发时都会短暂出现），单次看到它不能判保护停；只有
        "持续超过宽限 + 我们正在下发指令" 或 "有非零错误码" 才是保护停。
        """
        if arm_status is None:
            self._stop_state_since_s = None
            return None
        if int(arm_status) not in self._protection_states():
            self._stop_state_since_s = None
            return None
        if self._stop_state_since_s is None:
            self._stop_state_since_s = time.monotonic()
            return 0.0
        return (time.monotonic() - self._stop_state_since_s) * 1000.0

    def _on_safety_event(self, type: str, severity: str, reason: str,
                          evidence: Any, next_action: str) -> None:
        """安全包事件回调 → 执行器事件日志（安全层**不碰** EventLog）。"""
        self._add_event(type, severity, reason, _format_evidence(evidence),
                        next_action)

    def _apply_safety(self, follow_error: Optional[float],
                      snapshot: Optional[Dict[str, Any]]) -> bool:
        """把故障信号交给安全包判定（**唯一动作权威**）。

        跟随误差 / 断流的**判定**在安全包里，安全动作也只由安全包发出
        一次（消除「既 pause 又被安全线程再 stop」的双重动作）。

        返回 ``True`` = 需要中止本次执行（L2 / L3 / L4 或动作失败）。
        """
        supervisor = self._safety
        if supervisor is None:
            return False
        signals = self._safety_signals(follow_error, snapshot)
        try:
            decision = supervisor.evaluate(signals)
        except Exception as exc:  # noqa: BLE001 - 判据异常不得假装正常
            self._add_event(
                "safety_evaluation_failed", "FATAL",
                "安全判据执行异常（%s）: %s" % (type(exc).__name__, exc),
                ["fallback=stop"],
                "立即人工确认机械臂安全状态并断电",
            )
            self._self_triggered_stop = True
            self._stop_transport()
            return True
        action = supervisor.last_action
        if action is not None and action.status == "fatal":
            # 动作失败（含 L4 急停失败）→ 执行器兜底停机（**不假装成功**）
            self._add_event(
                "safety_action_failed", "FATAL",
                "安全动作失败：%s" % action.reason,
                _format_evidence(action.evidence),
                action.next_action or "立即人工确认机械臂安全状态并断电",
            )
            self._self_triggered_stop = True
            self._stop_transport()
            return True
        levels = self._safety_modules["levels"]
        if decision.level:
            effective = action.effective_level if action is not None else decision.level
        else:
            effective = ""
        if effective == levels.LEVEL_L4:
            self._self_triggered_stop = True
        return effective in (levels.LEVEL_L2, levels.LEVEL_L3, levels.LEVEL_L4)

    def _safety_summary(self) -> str:
        """安全层状态摘要（报告 evidence 用）。"""
        if self._safety is None:
            return "unavailable（%s）" % (self._safety_error or "未知原因")
        data = self._safety.to_dict()
        return ("current_level=%s peak_level=%s speed_scaled=%s event_count=%s "
                "action_count=%s" % (data["current_level"], data["peak_level"],
                                     data["speed_scaled"], data["event_count"],
                                     data["action_count"]))

    @property
    def safety_status(self) -> Dict[str, Any]:
        """安全层状态（``rmctl safety status`` 与报告共用）。"""
        if self._safety is None:
            return {"status": "unavailable", "reason": self._safety_error}
        out = dict(self._safety.to_dict())
        out["status"] = "ok"
        return out

    # -------------------------------------------------- 安全闸门（进程外停机）
    def _gate_stop_requested(self) -> bool:
        """闸门是否已请求停机（**只读**闸门状态，零 SDK 调用）。"""
        gate = self.gate
        if gate is None:
            return False
        try:
            return bool(gate.stop_requested)
        except Exception:  # noqa: BLE001 - 闸门不可用不得假装正常
            return False

    def _on_gate_stop(self, level: str, reason: str, source: str) -> None:
        """外部闸门请求 → **立即**停机（闸门线程调用，不等 tick）。

        动作仍由安全包（唯一动作权威）发出：``SafetySupervisor.apply_external``
        → ``SafetyActuator`` → ``transport.stop()``（L4）。安全层不可用时
        执行器**兜底**停机（并留 FATAL 事件，绝不假装成功）。
        """
        with self._gate_lock:
            if self._external_stop_done:
                return
            self._external_stop_done = True
        self._external_stop = True
        gate_evidence = {
            "kind": "external_gate_request",
            "level": level,
            "reason": reason,
            "source": source or "socket",
        }
        supervisor = self._safety
        if supervisor is None:
            self._add_event(
                "safety_l4_emergency_stop", "FATAL",
                "外部闸门请求（安全层不可用 → 执行器兜底急停）：%s" % reason,
                [str(gate_evidence)],
                "立即按下硬件急停并人工确认机械臂状态",
            )
            self._self_triggered_stop = True
            self._stop_transport()
            return
        try:
            result = supervisor.apply_external(
                level=level, reason=reason, source=source,
                extra_evidence=[gate_evidence])
        except Exception as exc:  # noqa: BLE001 - 闸门处理异常也要留痕
            self._add_event(
                "safety_l4_emergency_stop", "FATAL",
                "外部闸门请求处理异常（%s）: %s" % (type(exc).__name__, exc),
                [repr(exc)],
                "立即按下硬件急停并人工确认机械臂状态",
            )
            self._self_triggered_stop = True
            self._stop_transport()
            return
        if not result.ok:
            self._add_event(
                "safety_action_failed", "FATAL",
                "外部闸门停机失败：%s" % result.reason,
                _format_evidence(result.evidence),
                result.next_action or "立即按下硬件急停并人工确认机械臂状态",
            )
            self._self_triggered_stop = True
            self._stop_transport()

    def _wire_gate(self) -> None:
        """把执行器挂到闸门上（注册回调 + 处理早到的请求）。"""
        gate = self.gate
        if gate is None:
            return
        try:
            gate.add_handler(self._on_gate_stop)
        except Exception:  # noqa: BLE001 - 闸门不可用不得掀翻执行
            return
        if not self._gate_stop_requested():
            return
        level = getattr(gate, "requested_level", "") or "L4"
        reason = getattr(gate, "reason", "") or "外部闸门请求"
        self._on_gate_stop(level, reason, getattr(gate, "source", "") or "socket")

    # -------------------------------------------------- 安全动作（停机 / 暂停）
    def _stop_transport(self) -> None:
        """立即停机（硬红线）并留痕（失败也要留证据）。

        S04 起本方法**降级为兜底**：正常路径的急停由安全包发出；本方法只在
        「安全动作失败」「判据异常」「控制器抢轨迹」「安全层不可用」时兜底
        调用。
        """
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
        """暂停执行；失败 → FATAL + 停机。

        S04 起本方法**降级为兜底**：正常路径的暂停由安全包发出；本方法只在
        「安全层不可用（降级）」时兜底调用。
        """
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
            expected_type_names=self.expected_trajectory_types,
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
            "expected_type_names=%s" % (",".join(self.expected_trajectory_types),),
            "conflict_confirmations=%d" % self._watchdog.conflict_confirmations
            if self._watchdog else "conflict_confirmations=?",
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
            # ①' 外部闸门：已请求停机 → 不再下发任何指令（tick 下一拍退出）
            if self._external_stop:
                return None
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
            follow_abort = False
            if error is not None:
                self._follow_errors.append(error)
                # 跟随误差两档（>2° 预警 / >5° 暂停）——**只发事件**
                follow_abort = self._check_follow_error(error, cmd_deg, snapshot)
            # ④ 断流两档（>50 ms 预警 / >200 ms 停机）——**只发事件**
            stream_abort = self._check_stream_health()
            # ④' 安全包判定（**唯一动作权威**：动作只由 supervisor 发一次）
            safety_abort = self._apply_safety(error, snapshot)
            if follow_abort or stream_abort or safety_abort or self._external_stop:
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
            if self._external_stop:
                return False
            result = self.transport.transmit(
                goal_deg, cfg.follow, cfg.trajectory_mode, cfg.radio
            )
            if result.get("status") != contract.STATUS_OK:
                self._transport_failure("transmit_hold", result)
                return False
            self._hold_ticks += 1
            snapshot = self._read_feedback()
            position_error = self._position_error(snapshot, goal_deg)
            follow_abort = False
            if position_error is not None:
                self._follow_errors.append(position_error)
                follow_abort = self._check_follow_error(position_error, goal_deg,
                                                       snapshot)
            stream_abort = self._check_stream_health()
            safety_abort = self._apply_safety(position_error, snapshot)
            if follow_abort or stream_abort or safety_abort or self._external_stop:
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
    # ------------------------------------------------------------ 观测接口
    @property
    def jitter_samples_ms(self) -> List[float]:
        """tick 抖动样本（毫秒）——长跑合并统计用（**只读拷贝**）。"""
        return list(self._jitter_samples)

    @property
    def follow_errors_deg(self) -> List[float]:
        """跟随误差样本（度）——长跑合并统计用（**只读拷贝**）。"""
        return list(self._follow_errors)

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
        evidence.append("safety: %s" % self._safety_summary())
        evidence.append("limit_check=%s%s" % (
            self._limit_check,
            ("（%s）" % self._limit_check_reason) if self._limit_check_reason else "",
        ))
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

        # -- 安全包状态重置（S04）-----------------------------------------
        # 监督器每次 run 重建：等级单调状态**不跨 run** 泄漏
        self._self_triggered_stop = False
        self._limit_check = "unavailable"
        self._limit_check_reason = ""
        self._safety = self._create_supervisor()
        if self._safety is None:
            self._add_event(
                "safety_layer_unavailable", "NEEDS_PARAM",
                "安全层 rm_safety 不可用：%s" % (self._safety_error or "未知原因"),
                list(self._safety_config_evidence),
                "确认 dualarm_ws/src/rm_safety 在 sys.path 上"
                "（export PYTHONPATH=<repo>/dualarm_ws/src/rm_safety）后重跑",
            )

        t_start = self._now()
        self._last_frame_time = t_start  # 断流 age 的起点
        # -- 安全闸门接线（S04/T7）：注册回调 + 处理早到的请求 ------------
        self._wire_gate()
        evidence: List[str] = [
            "mode=%s transport=%s" % (self.mode, getattr(self.transport, "name", "?")),
        ]
        # -- 夹爪互锁闸门状态（可观测：报告里能看出互锁是否启用）----------
        if self.motion_gate is None:
            evidence.append("未启用夹爪互锁（未注入 motion_gate）")
        elif getattr(self.motion_gate, "state", "") == interlock.STATE_GATE_UNAVAILABLE:
            evidence.append("夹爪互锁闸门不可用（%s）"
                            % interlock.REASON_GATE_UNAVAILABLE)
        else:
            evidence.append("已启用夹爪互锁（闸门状态=%s）"
                            % getattr(self.motion_gate, "state", "UNKNOWN"))
        for item in self._safety_config_evidence:
            evidence.append("safety config: %s" % item)
        for item in self._safety_error_code_warnings:
            evidence.append("safety error codes: %s" % item)

        # -- 闸门已请求停机（执行开始前就收到）→ 不下发任何运动指令 -------
        if self._external_stop:
            evidence.append("safety gate: 外部闸门已请求停机 → 本次执行未下发运动指令")
            return self._finalize(t_start, evidence)

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
        # 夹爪互锁闸门检查（**阶段边界**，绝不进 tick 线程）：
        # 闸门非 IDLE → 拒绝下发**任何**运动指令（含预就位）。
        blocked = self._block_by_motion_gate("pre_position")
        if blocked is not None:
            evidence.append("夹爪互锁：阶段 B 之前被拒绝（闸门状态=%s），"
                            "未下发任何运动指令" % blocked["state"])
            return self._finalize(t_start, evidence)
        # 阶段 B 期间**不启动控制器看门狗**：预就位走的是 ``rm_movej``
        # （控制器内部规划），此时控制器轨迹类型必然是 RM_JOINT_SPACE_PLANNING_E
        # ——若此时看门狗就绪，会把**我们自己下发的规划运动**误判为“控制器抢轨迹”
        # （2026-11-19 真机实测：V5 阶段因此报 FATAL + 急停）。
        # “抢轨迹”只在**阶段 C（CANFD 透传）**期间有意义：那时控制器应当处于
        # 纯透传态（RM_NO_PLANNING_E），任何其它类型都是真的抢轨迹。
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
        # 夹爪互锁闸门检查（**起流之前**）：闸门非 IDLE → 拒绝下发任何运动
        blocked = self._block_by_motion_gate("stream_start")
        if blocked is not None:
            evidence.append("夹爪互锁：阶段 C 起流前被拒绝（闸门状态=%s），"
                            "未下发任何运动指令" % blocked["state"])
            return self._finalize(t_start, evidence)
        self.stream_clock = self._make_stream_clock(self._config.period_s,
                                                     self._now())
        # 从此刻起我们在**下发运动指令**（保护停判据的"佐证"之一）
        self._commanding_motion = True
        # 控制器看门狗在此刻启动（**独立线程**：SDK 状态查询不进 tick 线程）：
        # 阶段 B 已完成、进入纯透传阶段，此刻起控制器轨迹类型必须是
        # RM_NO_PLANNING_E，否则就是真的“抢轨迹”。
        self._start_watchdog()
        jitter_before = len(self.stream_clock.jitter_samples_ms)
        if self._stream_ticks(grid_p, factor) is None:
            return self._finalize(t_start, evidence)
        self._commanding_motion = False
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
