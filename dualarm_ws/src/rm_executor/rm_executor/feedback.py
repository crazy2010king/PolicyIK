"""rm_executor 反馈层（反馈源抽象 + 后台采集泵）。

**硬约束（靠接口设计强制，不靠注释自觉）**：反馈采集**不得**在 tick 线程内
做 SDK 读调用——TCP 往返会直接打进 10 ms 栅格的抖动里，把 p99 < 2 ms 的红线
自己破坏掉。因此本层的形状是：

* :class:`FeedbackSource` —— 只暴露 ``latest()``（**读最新快照**），不做任何
  主动采集；SDK 读调用封装在具体实现内部；
* :class:`FeedbackPump` —— **后台线程**周期性拉取反馈源并缓存最新快照，
  tick 线程**只读**缓存（零 SDK 调用进 tick 线程）。

快照结构（**拷贝语义**，不透出 SDK 复用结构体的引用）::

    {
      "joint_position_deg": [7] | None,   # 关节角（度）
      "joint_speed_deg_s":  [7] | None,   # 关节角速度（度/秒）
      "last_frame_ts": float | None,      # 最后一帧时间戳（秒）
      "frames": int,                      # 已收到的帧数
    }

**无数据时字段为 ``None``，绝不返回 0 冒充**（0 会被误读成「关节在 0 度」）。

四个实现：

* :class:`ScriptedFeedback` —— 脚本/回放驱动的假反馈（可注入跟随误差、断流、
  静止等场景），T4/T5 的故障注入全靠它；
* :class:`PollFeedback` —— 白名单符号 ``rm_get_joint_degree`` 轮询（独立线程，
  周期 10–20 ms），速度由位置差分得到；
* :class:`UdpFeedback` —— 读 ``rm_hw_bridge.bridge.realtime_state()`` 的实时
  上报（T6 接线前只有帧计数，关节字段为 ``None``）；
* :class:`FeedbackPump` —— 后台采集泵（线程安全的快照缓存 + 断流看门狗输入）。

**零 rmctl 依赖、零 tesseract 依赖**。
"""
from __future__ import annotations

import ctypes
import threading
import time
from abc import ABC, abstractmethod
from typing import Any, Callable, Dict, List, Optional, Sequence

from rm_executor.contract import ARM_DOF

__all__ = [
    "FeedbackPump",
    "FeedbackSource",
    "PollFeedback",
    "ScriptedFeedback",
    "UdpFeedback",
    "empty_snapshot",
]

#: 反馈采集默认周期（秒）—— 10–20 ms 区间的中值
DEFAULT_POLL_PERIOD_S = 0.015


def empty_snapshot() -> Dict[str, Any]:
    """空快照（**None 值字段，不返回 0 冒充**）。"""
    return {
        "joint_position_deg": None,
        "joint_speed_deg_s": None,
        "last_frame_ts": None,
        "frames": 0,
    }


def _copy_snapshot(snapshot: Dict[str, Any]) -> Dict[str, Any]:
    """快照拷贝（列表逐个复制，避免调用方改到缓存内部）。"""
    out: Dict[str, Any] = {}
    for key, value in (snapshot or {}).items():
        if isinstance(value, (list, tuple)):
            out[key] = list(value)
        else:
            out[key] = value
    return out


def _as_float_list(value: Any, size: int = ARM_DOF) -> Optional[List[float]]:
    """把序列转成定长 float 列表；不合法 → ``None``（不静默补 0）。"""
    if value is None or isinstance(value, (str, bytes)):
        return None
    try:
        seq = list(value)
    except TypeError:
        return None
    if len(seq) != size:
        return None
    out: List[float] = []
    for item in seq:
        if isinstance(item, bool) or not isinstance(item, (int, float)):
            return None
        out.append(float(item))
    return out


# ------------------------------------------------------------------- 接口
class FeedbackSource(ABC):
    """反馈源接口：**只暴露「读最新快照」**。

    实现方**自己**决定怎么采集（轮询 SDK / 读 UDP 缓存 / 回放脚本），
    但**不得**把采集放进 tick 线程——采集节奏由 :class:`FeedbackPump`
    的后台线程控制。
    """

    #: 反馈源名（报告 / evidence 用）
    name = "feedback"

    @abstractmethod
    def latest(self) -> Dict[str, Any]:
        """返回最新快照（**拷贝语义**）。

        返回字典至少含 ``joint_position_deg`` / ``joint_speed_deg_s`` /
        ``last_frame_ts`` / ``frames`` 四个键；无数据时前两个为 ``None``。
        """

    def to_dict(self) -> Dict[str, Any]:
        """反馈源自描述（报告用）。"""
        return {"name": self.name}


# ------------------------------------------------------------ 脚本化假反馈
class ScriptedFeedback(FeedbackSource):
    """脚本化反馈（**离线假反馈源**）。

    三种驱动方式（可组合）：

    * ``frames`` —— 帧列表：每项可以是 7 个关节角（度）的序列，或
      ``{"joint_position_deg": [...], "joint_speed_deg_s": [...], "ts": ...}``；
    * ``script`` —— 可调用对象 ``f(index) -> item | None``；返回 ``None``
      表示**断流**（本拍没有新帧，时间戳不变 → 看门狗的 age 增长）；
    * ``static_deg`` —— 静止场景：始终返回同一组关节角。

    ``follow_error_deg`` 用于**注入跟随误差**（在真实位置上加一个恒定偏差），
    ``speed_deg_s`` 用于给缺少速度字段的帧补一个恒定速度。

    语义要点：

    * ``frames`` 计数**只在产出新帧时递增**（断流不递增）；
    * ``latest()`` 每次调用**消耗一帧**（拉取式），因此可确定性重放；
    * 返回值里的列表是**拷贝**，改它不影响内部状态。
    """

    name = "scripted"

    def __init__(
        self,
        frames: Optional[Sequence[Any]] = None,
        script: Optional[Callable[[int], Any]] = None,
        period_s: float = DEFAULT_POLL_PERIOD_S,
        start_ts: float = 0.0,
        follow_error_deg: float = 0.0,
        static_deg: Optional[Sequence[float]] = None,
        speed_deg_s: Optional[Any] = None,
        repeat_last: bool = True,
    ) -> None:
        """
        参数
        ----
        frames
            帧列表（见类文档）。
        script
            ``f(index) -> item | None``；``None`` = 断流。
        period_s
            合成帧的时间步长（秒）——用于生成单调递增的 ``last_frame_ts``。
        start_ts
            合成时间戳起点。
        follow_error_deg
            注入的跟随误差（度，恒定偏置）。
        static_deg
            静止场景的关节角（7 个，度）。
        speed_deg_s
            缺速度时的兜底速度（标量或 7 元序列，度/秒）。
        repeat_last
            ``frames`` 用尽后是否重复最后一帧（``False`` → 之后视作断流）。
        """
        self._frames = list(frames or [])
        self._script = script
        self.period_s = float(period_s)
        self._next_ts = float(start_ts)
        self.follow_error_deg = float(follow_error_deg)
        self._static = _as_float_list(static_deg)
        self._speed_default = speed_deg_s
        self.repeat_last = bool(repeat_last)

        self._index = 0
        self._frames_seen = 0
        self._last_ts: Optional[float] = None
        self._last_position: Optional[List[float]] = None
        self._last_speed: Optional[List[float]] = None
        self.last_error = ""
        self.pull_count = 0

    # -- 内部 -------------------------------------------------------------
    def _next_item(self) -> Any:
        """取下一项（``None`` = 断流）。"""
        if self._script is not None:
            item = self._script(self._index)
            self._index += 1
            return item
        if self._index < len(self._frames):
            item = self._frames[self._index]
            self._index += 1
            return item
        if self._frames and self.repeat_last:
            return self._frames[-1]
        if self._static is not None:
            # 静止场景：每一拍都是一帧（同一组关节角）
            return list(self._static)
        return None

    def _parse(self, item: Any) -> Optional[Dict[str, Any]]:
        """解析一帧 → ``{"position": [...], "speed": [...] | None, "ts": ...}``。"""
        ts = None
        position = None
        speed = None
        if isinstance(item, dict):
            ts = item.get("ts")
            position = item.get("joint_position_deg", item.get("position_deg"))
            speed = item.get("joint_speed_deg_s", item.get("speed_deg_s"))
        elif item is not None:
            position = item
        if position is None and self._static is not None:
            position = self._static
        positions = _as_float_list(position)
        if positions is None:
            return None
        speeds = _as_float_list(speed)
        if speeds is None:
            speeds = _as_float_list(self._speed_default)
            if speeds is None and isinstance(self._speed_default, (int, float)):
                speeds = [float(self._speed_default)] * ARM_DOF
        if self.follow_error_deg:
            positions = [p + self.follow_error_deg for p in positions]
        if ts is None:
            ts = self._next_ts
        return {"position": positions, "speed": speeds, "ts": float(ts)}

    # -- FeedbackSource ---------------------------------------------------
    def latest(self) -> Dict[str, Any]:
        """拉取一帧（**断流时不产出新帧**，时间戳保持不变）。"""
        self.pull_count += 1
        item = self._next_item()
        if item is None:
            if self._last_position is None:
                # 从未有过帧：明确报「暂无数据」，不返回 0 冒充
                self.last_error = "反馈脚本尚未产出任何帧（断流）"
                return empty_snapshot()
            # 断流：保留上一帧内容，但时间戳不变 → age 增长
            self.last_error = "反馈脚本断流（本次无新帧）"
            return {
                "joint_position_deg": list(self._last_position or []),
                "joint_speed_deg_s": list(self._last_speed or []),
                "last_frame_ts": self._last_ts,
                "frames": self._frames_seen,
            }
        parsed = self._parse(item)
        if parsed is None:
            self.last_error = "反馈脚本帧格式不合法（需要 %d 个关节角，度）" % ARM_DOF
            return {
                "joint_position_deg": None,
                "joint_speed_deg_s": None,
                "last_frame_ts": self._last_ts,
                "frames": self._frames_seen,
            }
        self._frames_seen += 1
        self._next_ts = parsed["ts"] + self.period_s
        self._last_ts = parsed["ts"]
        self._last_position = parsed["position"]
        self._last_speed = parsed["speed"]
        self.last_error = ""
        return {
            "joint_position_deg": list(parsed["position"]),
            "joint_speed_deg_s": list(parsed["speed"]) if parsed["speed"] else None,
            "last_frame_ts": self._last_ts,
            "frames": self._frames_seen,
        }

    # -- 便捷视图 ---------------------------------------------------------
    @property
    def frames(self) -> int:
        """已产出的帧数。"""
        return self._frames_seen

    @property
    def exhausted(self) -> bool:
        """脚本是否已用尽（且不重复末帧）。"""
        if self._script is not None:
            return False
        return self._index >= len(self._frames) and not (
            self._frames and self.repeat_last
        )


# --------------------------------------------------------------- 轮询反馈源
class PollFeedback(FeedbackSource):
    """轮询反馈源：白名单符号 ``rm_get_joint_degree``。

    **只允许在反馈线程里调用**（由 :class:`FeedbackPump` 的后台线程驱动）。

    * 关节角来自 SDK（**度**）；
    * 关节角速度由**相邻位置差分**得到（度/秒）——SDK 的
      ``rm_get_joint_degree`` 不返回速度，本实现不额外拉全量状态快照
      （那会引入更多 SDK 往返）；
    * SDK 调用失败（rc≠0 / 符号缺失 / 句柄无效）→ 关节字段为 ``None``，
      ``last_error`` 写明**接口名 + 返回码**，**不返回 0 冒充**。
    """

    name = "poll"

    def __init__(self, link: Any, period_s: float = DEFAULT_POLL_PERIOD_S) -> None:
        """
        参数
        ----
        link
            已建立的 ``rm_hw_bridge.bridge.ArmLink``（可为 ``None`` → 不可用）。
        period_s
            建议的轮询周期（秒，10–20 ms 区间）。
        """
        self._link = link
        self.period_s = float(period_s)
        self._frames = 0
        self._last_ts: Optional[float] = None
        self._prev_position: Optional[List[float]] = None
        self._prev_ts: Optional[float] = None
        self.last_error = ""

    def _snapshot(self, positions: Optional[List[float]],
                  speeds: Optional[List[float]],
                  ts: Optional[float]) -> Dict[str, Any]:
        return {
            "joint_position_deg": list(positions) if positions else None,
            "joint_speed_deg_s": list(speeds) if speeds else None,
            "last_frame_ts": ts,
            "frames": self._frames,
        }

    def latest(self) -> Dict[str, Any]:
        """轮询一次（**只允许在反馈线程调用**）。"""
        link = self._link
        if link is None or not getattr(link, "connected", False):
            self.last_error = "ArmLink 不可用（未连接），无法轮询关节角"
            return self._snapshot(None, None, self._last_ts)
        binding = getattr(getattr(link, "bridge", None), "binding", None)
        fn = getattr(binding, "rm_get_joint_degree", None)
        if fn is None:
            self.last_error = "契约清单缺少符号 rm_get_joint_degree，无法轮询关节角"
            return self._snapshot(None, None, self._last_ts)
        joints = (ctypes.c_float * ARM_DOF)()
        try:
            rc = int(fn(link.handle, joints) or 0)
        except Exception as exc:  # noqa: BLE001 - SDK 异常统一降级
            self.last_error = "rm_get_joint_degree 调用异常（%s）: %s" % (
                type(exc).__name__, exc,
            )
            return self._snapshot(None, None, self._last_ts)
        if rc != 0:
            self.last_error = "rm_get_joint_degree 返回 %d（未连接到控制器 / 通信失败）" % rc
            return self._snapshot(None, None, self._last_ts)

        now = time.monotonic()
        positions = [float(joints[i]) for i in range(ARM_DOF)]
        speeds: List[float] = [0.0] * ARM_DOF
        if self._prev_position is not None and self._prev_ts is not None:
            dt = now - self._prev_ts
            if dt > 0.0:
                speeds = [
                    (positions[i] - self._prev_position[i]) / dt
                    for i in range(ARM_DOF)
                ]
        self._prev_position = positions
        self._prev_ts = now
        self._frames += 1
        self._last_ts = now
        self.last_error = ""
        return self._snapshot(positions, speeds, now)

    @property
    def frames(self) -> int:
        """成功轮询到的帧数。"""
        return self._frames


# --------------------------------------------------------------- UDP 反馈源
class UdpFeedback(FeedbackSource):
    """UDP 实时上报反馈源（读 ``rm_hw_bridge.bridge.realtime_state()``）。

    T6 接线后，``realtime_state()`` 会提供线程安全的只读访问器
    ``latest_joint_state()``（返回拷贝后的关节角/速度）；**T6 之前**该访问器
    不存在，此时本模块**只**返回帧计数与时间戳，关节字段为 ``None``，
    并在 ``last_error`` 里写明原因（**不返回 0 冒充**）。
    """

    name = "udp"

    def __init__(self, realtime_state: Any = None) -> None:
        """
        参数
        ----
        realtime_state
            可选的实时上报状态对象（测试注入假对象用）；
            ``None`` → 惰性取 ``rm_hw_bridge.bridge.realtime_state()``。
        """
        self._state = realtime_state
        self.last_error = ""

    def _resolve(self) -> Any:
        if self._state is not None:
            return self._state
        try:
            from rm_hw_bridge import bridge as hw_bridge  # 惰性导入硬件层
        except Exception as exc:  # noqa: BLE001
            self.last_error = "无法导入 rm_hw_bridge.bridge（%s）: %s" % (
                type(exc).__name__, exc,
            )
            return None
        try:
            self._state = hw_bridge.realtime_state()
        except Exception as exc:  # noqa: BLE001
            self.last_error = "取 realtime_state() 失败（%s）: %s" % (
                type(exc).__name__, exc,
            )
            return None
        return self._state

    def latest(self) -> Dict[str, Any]:
        """读一次实时上报快照（**拷贝语义**）。"""
        state = self._resolve()
        if state is None:
            return empty_snapshot()
        getter = getattr(state, "latest_joint_state", None)
        if callable(getter):
            try:
                data = getter()
            except Exception as exc:  # noqa: BLE001
                self.last_error = "latest_joint_state() 调用异常（%s）: %s" % (
                    type(exc).__name__, exc,
                )
                return empty_snapshot()
            if not isinstance(data, dict):
                self.last_error = "latest_joint_state() 返回值格式不合法（%s）" % (
                    type(data).__name__,
                )
                return empty_snapshot()
            positions = _as_float_list(data.get("joint_position_deg"))
            speeds = _as_float_list(data.get("joint_speed_deg_s"))
            frames = data.get("frames")
            self.last_error = ""
            return {
                "joint_position_deg": positions,
                "joint_speed_deg_s": speeds,
                "last_frame_ts": data.get("last_frame_ts"),
                "frames": int(frames) if isinstance(frames, int) else 0,
            }
        # T6 接线前：只有帧计数
        try:
            data = state.snapshot(None)
        except Exception as exc:  # noqa: BLE001
            self.last_error = "realtime_state().snapshot() 调用异常（%s）: %s" % (
                type(exc).__name__, exc,
            )
            return empty_snapshot()
        data = data if isinstance(data, dict) else {}
        frames = data.get("frames")
        self.last_error = (
            "rm_hw_bridge.bridge.realtime_state() 尚未提供 latest_joint_state()"
            "（T6 接线前只有帧计数，关节字段为 None）"
        )
        return {
            "joint_position_deg": None,
            "joint_speed_deg_s": None,
            "last_frame_ts": data.get("last_ts"),
            "frames": int(frames) if isinstance(frames, int) else 0,
        }


# ------------------------------------------------------------- 后台采集泵
class FeedbackPump:
    """后台采集泵：**反馈采集与 tick 线程彻底解耦**。

    工作方式：

    * 后台线程按 ``period_s``（10–20 ms）周期调用 ``source.latest()``，
      把**最新快照**缓存起来；
    * tick 线程只调用 :meth:`latest`（**纯内存读**，零 SDK 调用）；
    * :meth:`last_age_ms` 给出「最新一帧距今多久」（**毫秒**），
      供 T5 的断流看门狗判据（50 ms 告警 / 200 ms 停机）使用；
    * :meth:`pull_once` 供测试/预热**同步**拉取一次——**tick 线程不得调用**
      （那会把 SDK 往返打进栅格抖动）。

    线程安全：缓存读写受锁保护；返回的快照是**拷贝**（改返回值不影响缓存）。
    """

    def __init__(self, source: FeedbackSource,
                 period_s: float = DEFAULT_POLL_PERIOD_S,
                 time_func: Callable[[], float] = time.monotonic) -> None:
        self.source = source
        self.period_s = float(period_s)
        self._time = time_func
        self._lock = threading.Lock()
        self._snapshot: Dict[str, Any] = empty_snapshot()
        self._frames = 0
        self._last_update: Optional[float] = None
        self._started_at: Optional[float] = None
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self.polls = 0
        self.errors: List[str] = []

    # -- 生命周期 ---------------------------------------------------------
    def start(self) -> None:
        """启动后台采集线程（幂等）。"""
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                return
            self._stop_event.clear()
            now = self._time()
            self._started_at = now
            if self._last_update is None:
                self._last_update = now
            thread = threading.Thread(
                target=self._run, name="rm-feedback-pump", daemon=True
            )
            self._thread = thread
        thread.start()

    def stop(self, timeout: float = 1.0) -> None:
        """停止后台采集线程（幂等）。"""
        self._stop_event.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout)
        self._thread = None

    @property
    def running(self) -> bool:
        """后台线程是否在跑。"""
        thread = self._thread
        return bool(thread is not None and thread.is_alive())

    def _run(self) -> None:
        while not self._stop_event.is_set():
            try:
                self.pull_once()
            except Exception as exc:  # noqa: BLE001 - 采集线程绝不能死掉
                self._record_error("采集线程异常（%s）: %s" % (type(exc).__name__, exc))
            self._stop_event.wait(self.period_s)

    # -- 采集 -------------------------------------------------------------
    def _record_error(self, message: str) -> None:
        with self._lock:
            self.errors.append(message)
            if len(self.errors) > 20:
                del self.errors[:-20]

    def pull_once(self) -> Dict[str, Any]:
        """**同步**拉取一次并更新缓存。

        仅供测试 / 预热使用——**tick 线程不得调用**（会把 SDK 往返打进栅格
        抖动）。返回当前缓存快照（拷贝）。
        """
        try:
            snapshot = self.source.latest()
        except Exception as exc:  # noqa: BLE001
            self._record_error("反馈源拉取异常（%s）: %s" % (type(exc).__name__, exc))
            snapshot = empty_snapshot()
        if not isinstance(snapshot, dict):
            self._record_error("反馈源返回值格式不合法（%s）"
                               % type(snapshot).__name__)
            snapshot = empty_snapshot()
        frames = snapshot.get("frames")
        fresh = isinstance(frames, int) and frames > self._frames
        with self._lock:
            self.polls += 1
            if fresh:
                self._snapshot = _copy_snapshot(snapshot)
                self._frames = frames
                self._last_update = self._time()
        return self.latest()

    # -- 只读视图 ---------------------------------------------------------
    def latest(self) -> Dict[str, Any]:
        """最新快照（**拷贝**，纯内存读）。"""
        with self._lock:
            return _copy_snapshot(self._snapshot)

    @property
    def frames(self) -> int:
        """已缓存的最新帧计数。"""
        with self._lock:
            return self._frames

    def last_age_ms(self) -> float:
        """最新一帧距今的毫秒数（**断流看门狗输入**）。

        从未收到过帧时，从泵启动时刻起算（因此「一直没数据」也会让 age
        增长 → 触发断流判据）。
        """
        with self._lock:
            last = self._last_update
        if last is None:
            return 0.0
        return max(0.0, (self._time() - last) * 1000.0)

    def to_dict(self) -> Dict[str, Any]:
        """泵的自描述（报告用）。"""
        with self._lock:
            return {
                "source": self.source.to_dict() if self.source else None,
                "period_s": self.period_s,
                "frames": self._frames,
                "polls": self.polls,
                "running": self.running,
                "errors": list(self.errors),
            }
