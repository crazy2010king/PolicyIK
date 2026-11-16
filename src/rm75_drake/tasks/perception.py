"""感知适配接口（S06-T06）：**非阻塞离散事件** + **保守场景膨胀**。

职责
----
本模块把「外部感知（相机 / 深度 / 抓取候选）」接入任务层，只做三件事：

1. **非阻塞订阅语义**：:class:`PerceptionBus` 用「回调 + 有界队列 + :meth:`PerceptionBus.poll`」
   承接离散事件——**不引入线程 / asyncio**（本仓库无并发设施），
   :meth:`PerceptionBus.publish` 永不阻塞、永不抛异常（队列满时丢弃最旧并计数）。
2. **保守场景表示**：:class:`PerceptionPolicy` 把「感知延迟」翻译成
   **包围盒膨胀安全裕量**（延迟越大裕量越大 ⇒ 延迟被保守场景吸收）。
3. **场景落地**：:func:`apply_perception_scene` 把膨胀后的包围盒注册进 scene graph，
   **重建碰撞检查器并回填** ``bundle.checker`` ⇒ 保守场景**真的**进入碰撞判定。

关键实测结论（本机 Drake 1.51.1 复现，S06 侦察 §4.2）
----------------------------------------------------
* ``finalize`` 之后 ``RegisterAnchoredGeometry`` + ``AssignRole(ProximityProperties())``
  是**允许**的；
* **旧碰撞检查器看不见新注册的几何**（实测 ``old checker free? True``）
  ⇒ **必须重建 checker**，否则保守场景**静默失效**（最危险的失败模式）；
* ``SceneBundle`` / ``DualSceneBundle`` 都是**可变 dataclass** ⇒ 可以直接回填
  ``bundle.checker``；
* ``RegisterAnchoredGeometry`` 的 ``source_id`` 必须是 ``SourceId`` **对象**
  （传裸整数会抛类型错）；
* ``CollisionCheckerParams.model`` 只接受 ``RobotDiagram``
  （见 :mod:`rm75_drake.drake_compat`）。

生效边界纪律（**必须遵守**）
----------------------------
感知更新**只在规划边界生效**：下一段规划开始**之前**重建场景并重算轨迹。
执行环是**阻塞**的、点位在**构造期固定** ⇒ 本模块**不假装能在线重定向已下发的轨迹**。
换句话说：感知可以让「下一段」更保守，但**不能**把「已经下发的这一段」改掉。

零 pydrake 依赖
--------------
:mod:`rm75_drake.tasks` 整包采用 PEP 562 惰性导入（import 不拉起 pydrake）。
本模块同样**不在模块顶层 import pydrake**：所有 Drake 调用都发生在
:func:`apply_perception_scene` 内部 ⇒ ``import rm75_drake.tasks.perception``
不会把 pydrake 拉进 ``sys.modules``（纯 Python 的策略 / 总线可离线单测）。
"""

from __future__ import annotations

import math
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Callable, Deque, Dict, Iterable, List, Mapping, Optional, Tuple

__all__ = [
    "PerceptionError",
    "PerceptionObservation",
    "PerceptionPolicy",
    "PerceptionSnapshot",
    "PerceptionBus",
    "PerceptionSceneResult",
    "apply_perception_scene",
    "DEFAULT_QUEUE_CAPACITY",
    "DEFAULT_STALE_AFTER_S",
]

#: 有界队列的默认容量（超出即丢最旧）。
DEFAULT_QUEUE_CAPACITY = 64

#: 观测「过期」的默认阈值（s）：年龄超过它 ⇒ 快照 ``stale=True``。
DEFAULT_STALE_AFTER_S = 0.5

#: 长度量纲上限（米）——超出视为「单位写错」（例如把毫米当米填）。
_LENGTH_LIMIT = 100.0

#: 角度量纲上限（rad）——超出视为「角度制写错」。
_RPY_LIMIT = math.pi


class PerceptionError(RuntimeError):
    """感知适配层的显式失败（非法观测 / 非法场景对象 / 场景更新失败）。"""


# ── 内部校验工具 ────────────────────────────────────────────────────────
def _check_name(value: Any, path: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise PerceptionError(f"{path} 必须是非空字符串，收到 {value!r}")
    return value


def _check_finite(value: Any, path: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise PerceptionError(f"{path} 必须是数值，收到 {value!r}")
    fvalue = float(value)
    if not math.isfinite(fvalue):
        raise PerceptionError(f"{path} 必须是有限数值，收到 {value!r}")
    return fvalue


def _check_vec3(
    values: Any,
    path: str,
    *,
    limit: float = _LENGTH_LIMIT,
    unit: str = "长度",
    positive: bool = False,
) -> Tuple[float, float, float]:
    """校验 3 维数值序列（长度 / 有限性 / 可选严格正）。"""
    if isinstance(values, (str, bytes)) or not isinstance(values, (tuple, list)):
        raise PerceptionError(f"{path} 必须是 3 维序列（list / tuple），收到 {type(values).__name__}")
    if len(values) != 3:
        raise PerceptionError(f"{path} 必须是 3 维序列，收到 {len(values)} 个元素")
    out: List[float] = []
    for index, value in enumerate(values):
        fvalue = _check_finite(value, f"{path}[{index}]")
        if abs(fvalue) > limit:
            raise PerceptionError(
                f"{path}[{index}] = {fvalue} 超出{unit}上限 {limit}"
                f"（{'长度一律用米（毫米请先换算）' if unit == '长度' else 'rpy 必须用弧度（rad）'}）"
            )
        if positive and fvalue <= 0.0:
            raise PerceptionError(f"{path}[{index}] = {fvalue} 必须严格为正")
        out.append(fvalue)
    return (out[0], out[1], out[2])


# ── 观测与策略 ──────────────────────────────────────────────────────────
@dataclass(frozen=True)
class PerceptionObservation:
    """一次外部感知观测（**离散事件**的载荷）。

    字段
    ----
    observation_id:
        观测标识（非空）。用于几何命名（``perception_<id>``）与去重。
    position_xyz:
        观测目标在**世界系**的位置（m），3 维。
    rpy:
        观测目标的姿态（**内旋 xyz**，rad），3 维。
    half_size:
        包围盒**半长**（m），3 维且**严格为正**。
        （Drake 的 ``Box`` 参数就是半长；膨胀时每个轴各加一个裕量。）
    timestamp_s:
        观测时刻（s）。与 :meth:`PerceptionPolicy.effective_margin` 的 ``now_s``
        必须**同一个时钟域**（否则裕量计算无意义）。
    source:
        观测来源标签（例如 ``"camera"`` / ``"depth"`` / ``"synthetic"``）。
    """

    observation_id: str
    position_xyz: Tuple[float, float, float]
    rpy: Tuple[float, float, float]
    half_size: Tuple[float, float, float]
    timestamp_s: float
    source: str = "perception"

    def __post_init__(self) -> None:
        object.__setattr__(self, "observation_id", _check_name(self.observation_id, "observation_id"))
        object.__setattr__(self, "position_xyz", _check_vec3(self.position_xyz, "position_xyz"))
        object.__setattr__(
            self,
            "rpy",
            _check_vec3(self.rpy, "rpy", limit=_RPY_LIMIT, unit="角度"),
        )
        object.__setattr__(
            self,
            "half_size",
            _check_vec3(self.half_size, "half_size", positive=True),
        )
        object.__setattr__(self, "timestamp_s", _check_finite(self.timestamp_s, "timestamp_s"))
        object.__setattr__(self, "source", _check_name(self.source, "source"))

    def age_s(self, now_s: float) -> float:
        """观测年龄（s）：``max(0, now_s − timestamp_s)``（未来时刻按 0 计）。"""
        return max(0.0, float(now_s) - float(self.timestamp_s))


@dataclass(frozen=True)
class PerceptionPolicy:
    """感知保守场景策略（**延迟 ⇒ 膨胀裕量**）。

    字段
    ----
    margin:
        基础膨胀裕量（m）。即便零延迟也保留的安全余量。
    latency_margin_rate:
        延迟裕量速率（m/s）：每延迟 1 s，包围盒**每轴**多膨胀这么多米。
    max_margin:
        裕量上限（m）。延迟再大也不超过它（避免把场景膨胀到失去意义）。
    stale_after_s:
        观测过期阈值（s）：年龄超过它 ⇒ 快照标记 ``stale``。

    裕量语义
    --------
    ``effective_margin = min(max_margin, margin + latency_margin_rate × age)``
    ⇒ **延迟越大裕量越大** ⇒ 感知延迟被**保守场景表示**吸收。
    """

    margin: float = 0.02
    latency_margin_rate: float = 0.05
    max_margin: float = 0.15
    stale_after_s: float = DEFAULT_STALE_AFTER_S

    def __post_init__(self) -> None:
        for name in ("margin", "latency_margin_rate", "max_margin", "stale_after_s"):
            value = _check_finite(getattr(self, name), name)
            if value < 0.0:
                raise PerceptionError(f"{name} 必须 ≥ 0，收到 {value!r}")
            object.__setattr__(self, name, value)

    def effective_margin(self, observation: PerceptionObservation, now_s: float) -> float:
        """返回该观测在 ``now_s`` 时刻的**生效膨胀裕量**（m，每轴）。"""
        age = observation.age_s(now_s)
        return min(self.max_margin, self.margin + self.latency_margin_rate * age)

    def is_stale(self, observation: PerceptionObservation, now_s: float) -> bool:
        """观测是否过期（年龄 > ``stale_after_s``）。"""
        return observation.age_s(now_s) > self.stale_after_s


@dataclass(frozen=True)
class PerceptionSnapshot:
    """:meth:`PerceptionBus.poll` 返回的**最新快照**。"""

    observation: PerceptionObservation
    age_s: float
    stale: bool


# ── 非阻塞事件总线 ──────────────────────────────────────────────────────
class PerceptionBus:
    """感知**离散事件**总线（非阻塞订阅语义，**无线程 / 无 asyncio**）。

    语义
    ----
    * :meth:`subscribe` 注册回调（返回句柄，可用于 :meth:`unsubscribe`）；
    * :meth:`publish` **立即返回**：观测进入**有界队列**（容量 ``capacity``），
      队列满时**丢弃最旧**并把 ``dropped`` 计数 +1——**不阻塞、不抛异常**；
      随后依次调用订阅回调，回调自身的异常被**吞掉并计入** ``callback_errors``
      （一个坏订阅者不许拖垮发布方）；
    * :meth:`poll` 返回**最新快照**（含 ``stale`` 标志），并丢弃比它更旧的条目
      （队列里最多保留最新一条）。

    过期阈值
    --------
    ``stale_after_s`` 默认 **0.5 s**，与
    :attr:`rm75_drake.tasks.perception.PerceptionPolicy.stale_after_s` 同值
    （两处口径一致，避免「总线判新鲜、场景判过期」的错位）。
    """

    def __init__(
        self,
        *,
        capacity: int = DEFAULT_QUEUE_CAPACITY,
        stale_after_s: float = DEFAULT_STALE_AFTER_S,
    ) -> None:
        if isinstance(capacity, bool) or not isinstance(capacity, int) or capacity < 1:
            raise PerceptionError(f"capacity 必须是 ≥ 1 的整数，收到 {capacity!r}")
        stale = _check_finite(stale_after_s, "stale_after_s")
        if stale < 0.0:
            raise PerceptionError(f"stale_after_s 必须 ≥ 0，收到 {stale_after_s!r}")
        self._capacity = int(capacity)
        self._stale_after_s = float(stale)
        self._queue: Deque[PerceptionObservation] = deque()
        self._subscribers: List[Callable[[PerceptionObservation], None]] = []
        #: 被丢弃（队列满时挤掉）的观测数。
        self.dropped = 0
        #: 回调抛出的异常数（异常已被吞掉，不影响发布方）。
        self.callback_errors = 0
        #: 累计发布数。
        self.published = 0

    # ── 订阅 ────────────────────────────────────────────────────────────
    def subscribe(self, callback: Callable[[PerceptionObservation], None]) -> int:
        """注册回调，返回句柄（int，可传给 :meth:`unsubscribe`）。"""
        if not callable(callback):
            raise PerceptionError(f"callback 必须可调用，收到 {type(callback).__name__}")
        self._subscribers.append(callback)
        return len(self._subscribers) - 1

    def unsubscribe(self, handle: int) -> bool:
        """注销回调；返回是否真的注销掉了。"""
        if isinstance(handle, bool) or not isinstance(handle, int):
            raise PerceptionError(f"handle 必须是整数，收到 {handle!r}")
        if handle < 0 or handle >= len(self._subscribers):
            return False
        self._subscribers.pop(handle)
        return True

    @property
    def subscriber_count(self) -> int:
        """当前订阅者数量。"""
        return len(self._subscribers)

    # ── 发布（非阻塞）───────────────────────────────────────────────────
    def publish(self, observation: PerceptionObservation) -> None:
        """发布一次观测（**非阻塞**）：入队 → 通知回调 → 立即返回。"""
        if not isinstance(observation, PerceptionObservation):
            raise PerceptionError(
                f"observation 必须是 PerceptionObservation，收到 {type(observation).__name__}"
            )
        self.published += 1
        if len(self._queue) >= self._capacity:
            self._queue.popleft()
            self.dropped += 1
        self._queue.append(observation)
        for callback in list(self._subscribers):
            try:
                callback(observation)
            except Exception:  # 坏订阅者不许拖垮发布方
                self.callback_errors += 1

    # ── 轮询 ────────────────────────────────────────────────────────────
    def poll(self, now_s: float) -> Optional[PerceptionSnapshot]:
        """返回最新观测的快照（含 ``stale`` 标志）；队列为空时返回 ``None``。"""
        now = _check_finite(now_s, "now_s")
        if not self._queue:
            return None
        latest = self._queue[-1]
        while len(self._queue) > 1:
            self._queue.popleft()
        age = latest.age_s(now)
        return PerceptionSnapshot(
            observation=latest,
            age_s=age,
            stale=age > self._stale_after_s,
        )

    @property
    def queued(self) -> int:
        """当前队列长度（≤ ``capacity``）。"""
        return len(self._queue)


# ── 保守场景落地 ────────────────────────────────────────────────────────
@dataclass(frozen=True)
class PerceptionSceneResult:
    """一次感知场景更新的结果。

    字段
    ----
    geometry_ids:
        新注册的感知几何 id（顺序 = 输入观测顺序）。
    effective_margins:
        ``observation_id`` → 生效膨胀裕量（m）。
    checker_rebuilt:
        是否重建了碰撞检查器（**恒为 True**：不重建 ⇒ 保守场景静默失效）。
    stale_observation_ids:
        过期（年龄 > ``stale_after_s``）的观测 id（仍会注册，裕量更大 ⇒ 更保守）。
    dropped_observations:
        本次**未注册**的观测数（同一 ``observation_id`` 重复 ⇒ 跳过，不重复注册）。
    """

    geometry_ids: Tuple[Any, ...] = ()
    effective_margins: Mapping[str, float] = field(default_factory=dict)
    checker_rebuilt: bool = True
    stale_observation_ids: Tuple[str, ...] = ()
    dropped_observations: int = 0


def _bundle_registry(bundle: Any) -> Dict[str, Any]:
    """取出（必要时建立）bundle 上的「已注册感知几何」登记表。"""
    registry = getattr(bundle, "perception_geometry_ids", None)
    if registry is None:
        registry = {}
        try:
            setattr(bundle, "perception_geometry_ids", registry)
        except Exception as exc:  # pragma: no cover - 不可变 bundle
            raise PerceptionError(
                "bundle 不可写：无法登记感知几何（perception_geometry_ids）"
            ) from exc
    return registry


def apply_perception_scene(
    bundle: Any,
    observations: Iterable[PerceptionObservation],
    policy: PerceptionPolicy,
    *,
    now_s: Optional[float] = None,
    edge_step_size: float = 0.01,
) -> PerceptionSceneResult:
    """把感知观测**保守化**后落地到场景，并**重建碰撞检查器**。

    参数
    ----
    bundle:
        :class:`rm75_drake.planning.scene.SceneBundle` 或
        :class:`rm75_drake.bimanual.dual_scene.DualSceneBundle`
        （两者都是**可变 dataclass**，可以回填 ``checker``）。
    observations:
        观测序列（:class:`PerceptionObservation`）。
    policy:
        保守场景策略（裕量 / 延迟速率 / 上限 / 过期阈值）。
    now_s:
        当前时刻（s，**必须与观测的 ``timestamp_s`` 同一时钟域**）。
        ``None`` ⇒ 取观测中最大的 ``timestamp_s``（视作「零延迟」的确定性默认）。
    edge_step_size:
        重建碰撞检查器时的边采样步长（m，> 0）。

    步骤（**顺序不可改**）
    --------------------
    1. 逐观测算生效裕量与**膨胀后半长**（``half_size + effective_margin``）；
    2. ``RegisterAnchoredGeometry(bundle.source_id, GeometryInstance(...))``；
    3. ``AssignRole(bundle.source_id, geometry_id, ProximityProperties())``
       ——**必须显式赋角色**，否则碰撞检查**完全看不见**这个几何；
    4. **重建 checker** 并回填 ``bundle.checker``（旧 checker 看不见新几何）；
    5. 返回 :class:`PerceptionSceneResult`。

    生效边界
    --------
    本函数在**规划边界**调用（下一段规划之前）。执行环阻塞、点位在构造期固定
    ⇒ 本函数**不会**（也不能）在线重定向已经下发的轨迹。
    """
    from pydrake.geometry import Box, GeometryInstance, ProximityProperties
    from pydrake.math import RigidTransform

    from .. import drake_compat

    if not isinstance(policy, PerceptionPolicy):
        raise PerceptionError(f"policy 必须是 PerceptionPolicy，收到 {type(policy).__name__}")
    if bundle is None:
        raise PerceptionError("bundle 不能是 None")
    scene_graph = getattr(bundle, "scene_graph", None)
    source_id = getattr(bundle, "source_id", None)
    robot_diagram = getattr(bundle, "robot_diagram", None)
    model_instances = getattr(bundle, "model_instances", ())
    if scene_graph is None or source_id is None or robot_diagram is None:
        raise PerceptionError(
            "bundle 必须具备 scene_graph / source_id / robot_diagram（"
            "请用 build_scene / build_dual_scene 装配）"
        )

    obs_list: List[PerceptionObservation] = []
    for item in observations:
        if not isinstance(item, PerceptionObservation):
            raise PerceptionError(
                f"observations 只能包含 PerceptionObservation，收到 {type(item).__name__}"
            )
        obs_list.append(item)

    if now_s is None:
        now = max((o.timestamp_s for o in obs_list), default=time.monotonic())
    else:
        now = _check_finite(now_s, "now_s")

    registry = _bundle_registry(bundle)
    geometry_ids: List[Any] = []
    effective_margins: Dict[str, float] = {}
    stale_ids: List[str] = []
    dropped = 0

    for observation in obs_list:
        if observation.observation_id in registry:
            # 同一观测重复到达 ⇒ 不重复注册几何（登记幂等）。
            dropped += 1
            continue
        margin = policy.effective_margin(observation, now)
        half = tuple(float(h) + margin for h in observation.half_size)
        rotation = drake_compat.rotation_from_rpy(observation.rpy)
        pose = RigidTransform(rotation, observation.position_xyz)
        instance = GeometryInstance(
            pose, Box(half[0], half[1], half[2]), f"perception_{observation.observation_id}"
        )
        geometry_id = scene_graph.RegisterAnchoredGeometry(source_id, instance)
        # 没有 proximity 角色 ⇒ 碰撞检查看不见这个几何（实测坑）。
        scene_graph.AssignRole(source_id, geometry_id, ProximityProperties())
        registry[observation.observation_id] = geometry_id
        geometry_ids.append(geometry_id)
        effective_margins[observation.observation_id] = margin
        if policy.is_stale(observation, now):
            stale_ids.append(observation.observation_id)

    # 重建碰撞检查器：旧 checker 看不见新注册的几何 ⇒ 不重建等于保守场景失效。
    robot_instances = list(model_instances) + list(
        getattr(bundle, "gripper_model_instances", ())
    )
    checker = drake_compat.make_collision_checker(
        robot_model_instances=robot_instances,
        robot_diagram=robot_diagram,
        edge_step_size=edge_step_size,
    )
    bundle.checker = checker

    return PerceptionSceneResult(
        geometry_ids=tuple(geometry_ids),
        effective_margins=dict(effective_margins),
        checker_rebuilt=True,
        stale_observation_ids=tuple(stale_ids),
        dropped_observations=dropped,
    )
