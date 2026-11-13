"""Planner API（S02-T07）：**同源**规划入口 + 可解释失败。

职责边界
--------
本模块是 **S02 → S03/S04 的规划面交接件**：把「场景真相源 + 目标」变成
「一条可直接下发的 5 ms 网格轨迹 + 夹爪调度表」。

**同源保证（本模块的第一契约）**
--------------------------------
:func:`Planner.plan_task_space_target` 与 :func:`Planner.plan_joint_target`
**只产出数据**（:class:`~rm75_drake.planning.trajectory.Trajectory` +
:class:`~rm75_drake.gripper.GripperAction` 调度表 + 诊断字典），
**完全不碰任何驱动**：

* 仿真驱动（:class:`rm75_drake.drivers.sim.SimulationDriver`）、
  假驱动（:class:`rm75_drake.drivers.fake.FakeDriver`）、
  未来的真机驱动（``rm_device``，S03）**消费同一条 Planner 输出**，
  差异**只**发生在驱动实现层（R007）。

因此本模块**不得** import :mod:`rm75_drake.drivers`（有测试守着这条边界）。

内部链路
--------
::

    任务空间目标 (RigidTransform)
        → IK（T03）
    关节目标 (7 维)
        → 直接进入下一段（可达性按软件软限位复核）

    → IRIS 凸区域链（T04）      起点 → 目标 之间的种子链
    → 空间 GCS 最短路（T05）   区域图上的连通性验证 + 区域序列
    → Toppra 重定时（T06）      5 ms 网格 + 限值 / C2 连续性复核
    → 碰撞复检（T05）           沿轨迹密采样，逃逸数必须为 0
    → Trajectory（T02）

失败可解释
----------
任一阶段失败都**不抛异常**，而是返回 ``success=False`` 的
:class:`PlanResult`，且：

* :attr:`PlanResult.stage` 指出**失败阶段**（``ik`` / ``regions`` / ``gcs`` /
  ``retime`` / ``collision``）；
* :attr:`PlanResult.reason` 沿用**各阶段自己的失败分类标签**
  （如 :data:`rm75_drake.planning.ik.REASON_UNREACHABLE`、
  :data:`rm75_drake.planning.regions.REASON_SEED_INFEASIBLE`、
  :data:`rm75_drake.planning.gcs.REASON_NO_PATH`）；
* :attr:`PlanResult.message` 同时含阶段、原因与人类可读细节。

几何路径的来源（**必须写清楚，否则会被误读**）
----------------------------------------------
本机（Drake 1.51.1，无 Mosek 许可）**没有混合整数求解器**，GCS 只能走
**凸松弛**模式。实测：凸松弛解出的顶点位置会退化（实测解点为全零向量，
长度为零的"路径"），**不能**直接拿来当几何路径。

因此本模块的分工是：

* **IRIS 区域链**：给出沿直线路径的一串**碰撞自由**凸区域（种子位形即路径节点）；
* **空间 GCS**：在区域图上求最短路，**验证区域链连通**并记录求解模式 / 边数 /
  经过的区域序列；
* **几何路径**：由区域链的**种子位形**稠密插值得到（相邻间距 ≤ 0.01 rad）；
* **碰撞复检**：对最终 5 ms 轨迹密采样，**逃逸数必须为 0**——这是几何路径
  安全性的权威判据（IRIS 认证的是区域，直线段的安全性由复检兜底）。

有 Mosek 的环境（远端基线）里 GCS 会走整数版，届时
``diagnostics["gcs"]["solver_mode"]`` 会记成 ``"mixed_integer"``，
``diagnostics["gcs"]["visited_regions"]`` 会给出整数版真正经过的区域序列。

单臂限定
--------
本切片**只支持单臂**（7 DoF）。多机器人（双臂 / 移动底盘）留给 S04/S05：
届时 Planner 会按机器人实例拆分出多个 :class:`Planner` 或引入多臂目标表达。
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field, replace
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

import numpy as np

from .gripper import GripperAction
from . import limits as _limits
from .planning import gcs as _gcs
from .planning import ik as _ik
from .planning import regions as _regions
from .planning import retime as _retime
from .planning import scene as _scene
from .planning.trajectory import DEFAULT_DT, JOINT_COUNT, Trajectory

__all__ = [
    "PlannerError",
    "PlanOptions",
    "PlanResult",
    "Planner",
    "STAGE_IK",
    "STAGE_REGIONS",
    "STAGE_GCS",
    "STAGE_RETIME",
    "STAGE_COLLISION",
    "STAGES",
    "DEFAULT_REGION_SPACING_RAD",
    "DEFAULT_ADJACENCY_FACTOR",
    "DEFAULT_PATH_STEP_RAD",
    "REASON_LIMITS_ERROR",
]


# ── 阶段标签（失败时写进 PlanResult.stage）─────────────────────────────
STAGE_IK = "ik"
STAGE_REGIONS = "regions"
STAGE_GCS = "gcs"
STAGE_RETIME = "retime"
STAGE_COLLISION = "collision"

#: 全部阶段标签（按链路顺序）。
STAGES: Tuple[str, ...] = (
    STAGE_IK,
    STAGE_REGIONS,
    STAGE_GCS,
    STAGE_RETIME,
    STAGE_COLLISION,
)

#: 区域链种子之间的最大**欧氏**间距（rad）。
DEFAULT_REGION_SPACING_RAD = 0.15

#: 邻接阈值 = 该系数 × 实际相邻种子间距（保证「只连相邻两个区域」）。
DEFAULT_ADJACENCY_FACTOR = 1.5

#: 几何路径的稠密插值步长（rad）。
DEFAULT_PATH_STEP_RAD = 0.01

#: 限位交接件（``models/rm75_drake/joint_limits.yaml``）不可用时的原因标签。
REASON_LIMITS_ERROR = "limits_error"


class PlannerError(RuntimeError):
    """Planner 的用法错误（构造参数非法、目标非法等）。

    属性
    ----
    stage:
        失败阶段标签（见 :data:`STAGES`）；构造期错误为空串。
    reason:
        稳定原因标签；尽量沿用各阶段自己的失败分类。
    """

    def __init__(self, message: str, *, stage: str = "", reason: str = "invalid_input") -> None:
        super().__init__(message)
        self.stage = stage
        self.reason = reason


# ── 选项 ────────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class PlanOptions:
    """一次规划的可调旋钮（全部有默认值，默认值即可跑通常规单臂任务）。

    字段
    ----
    start_configuration:
        起点关节角（7 维，rad）。``None`` → 用场景当前位形
        （``scene.plant_context`` 里的关节角）。
    region_spacing:
        IRIS 区域链种子之间的最大欧氏间距（rad）。
    adjacency_factor:
        GCS 邻接阈值 = 该系数 × 实际相邻种子间距。取 1.5 的效果是
        **只连相邻区域**（隔一个的区域不连边），使最短路必须沿区域链走。
    path_step:
        几何路径的稠密插值步长（rad）。
    dt:
        输出轨迹的采样周期（s），默认 5 ms。
    t0:
        输出轨迹首点的绝对时刻（s）。
    duration_limit:
        可选的总时长上限（s）；超过 → 失败（阶段 ``retime``）。
    gripper_schedule:
        夹爪动作调度表（原样写进输出轨迹）。
    robot_name:
        输出轨迹里的机器人名（写入录制文件供回放校验）。
    ik_timeout / ik_position_tol / ik_orientation_tol:
        IK 阶段参数（语义见 :func:`rm75_drake.planning.ik.solve_ik`）。
    ik_initial_posture:
        IK 的显式种子位形；``None`` → 用 IK 模块的抬肘名义姿态。
    region_options:
        IRIS 旋钮字典（键取自
        :data:`rm75_drake.planning.regions.OVERRIDABLE_OPTIONS`），
        例如 ``{"num_collision_infeasible_samples": 3}``（更快、认证更弱）。
    collision_edge_step_size:
        碰撞复检的密采样步长（rad）。
    """

    start_configuration: Optional[Tuple[float, ...]] = None
    region_spacing: float = DEFAULT_REGION_SPACING_RAD
    adjacency_factor: float = DEFAULT_ADJACENCY_FACTOR
    path_step: float = DEFAULT_PATH_STEP_RAD
    dt: float = DEFAULT_DT
    t0: float = 0.0
    duration_limit: Optional[float] = None
    gripper_schedule: Tuple[GripperAction, ...] = ()
    robot_name: str = "rm75"
    ik_timeout: float = 0.05
    ik_position_tol: float = 1e-4
    ik_orientation_tol: float = 1e-3
    ik_initial_posture: Optional[Tuple[float, ...]] = None
    region_options: Mapping[str, Any] = field(default_factory=dict)
    collision_edge_step_size: float = 0.01

    def __post_init__(self) -> None:
        if not math.isfinite(self.region_spacing) or self.region_spacing <= 0.0:
            raise PlannerError(
                f"region_spacing 必须是正有限值，实际：{self.region_spacing!r}",
                reason="invalid_config",
            )
        if not math.isfinite(self.adjacency_factor) or self.adjacency_factor <= 1.0:
            raise PlannerError(
                "adjacency_factor 必须 > 1（否则非相邻区域也会连边），实际："
                f"{self.adjacency_factor!r}",
                reason="invalid_config",
            )
        if not math.isfinite(self.path_step) or self.path_step <= 0.0:
            raise PlannerError(
                f"path_step 必须是正有限值，实际：{self.path_step!r}",
                reason="invalid_config",
            )
        if not math.isfinite(self.dt) or self.dt <= 0.0:
            raise PlannerError(
                f"dt 必须是正有限值，实际：{self.dt!r}", reason="invalid_config"
            )
        if not math.isfinite(self.t0):
            raise PlannerError(
                f"t0 必须是有限值，实际：{self.t0!r}", reason="invalid_config"
            )
        unknown = set(self.region_options) - set(_regions.OVERRIDABLE_OPTIONS)
        if unknown:
            raise PlannerError(
                f"region_options 含未知 IRIS 旋钮：{sorted(unknown)}"
                f"（允许：{list(_regions.OVERRIDABLE_OPTIONS)}）",
                reason="invalid_config",
            )
        object.__setattr__(self, "gripper_schedule", tuple(self.gripper_schedule))
        if self.start_configuration is not None:
            object.__setattr__(
                self, "start_configuration", tuple(float(v) for v in self.start_configuration)
            )
        if self.ik_initial_posture is not None:
            object.__setattr__(
                self,
                "ik_initial_posture",
                tuple(float(v) for v in self.ik_initial_posture),
            )

    def merged(self, override: Optional["PlanOptions"]) -> "PlanOptions":
        """返回「以本选项为底、用 ``override`` 覆盖非空字段」的新选项。"""
        if override is None:
            return self
        if not isinstance(override, PlanOptions):
            raise PlannerError(
                f"options 必须是 PlanOptions，收到 {type(override).__name__}",
                reason="invalid_input",
            )
        changes: Dict[str, Any] = {}
        for name in (
            "start_configuration",
            "region_spacing",
            "adjacency_factor",
            "path_step",
            "dt",
            "t0",
            "duration_limit",
            "gripper_schedule",
            "robot_name",
            "ik_timeout",
            "ik_position_tol",
            "ik_orientation_tol",
            "ik_initial_posture",
            "region_options",
            "collision_edge_step_size",
        ):
            value = getattr(override, name)
            default = getattr(PlanOptions(), name)
            if value != default:
                changes[name] = value
        return replace(self, **changes) if changes else self


# ── 结果 ────────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class PlanResult:
    """一次规划的结果（**成功与失败都用它表达**，不抛异常）。

    字段
    ----
    success:
        是否规划成功。
    message:
        **始终非空**的可读说明（成功时描述结果；失败时含阶段 + 原因）。
    trajectory:
        成功时的输出轨迹（5 ms 网格）。失败时为 ``None``。
    gripper_schedule:
        夹爪动作调度表（成功时与轨迹内的一致；失败时为空）。
    diagnostics:
        诊断字典（见 :meth:`Planner.plan_joint_target` 的说明）。
    stage:
        失败阶段标签（成功时为空串）。
    reason:
        失败原因标签（沿用各阶段的失败分类；成功时为空串）。
    """

    success: bool
    message: str
    trajectory: Optional[Trajectory] = None
    gripper_schedule: Tuple[GripperAction, ...] = ()
    diagnostics: Mapping[str, Any] = field(default_factory=dict)
    stage: str = ""
    reason: str = ""

    def __bool__(self) -> bool:
        """``bool(result)`` 等价于 ``result.success``。"""
        return bool(self.success)


# ── Planner ─────────────────────────────────────────────────────────────
class Planner:
    """同源规划器（仿真 / 假驱动 / 真机共用同一套 API）。

    构造
    ----
    ``Planner(scenario)``
        由场景真相源装配 Drake 场景（:func:`rm75_drake.planning.scene.build_scene`）。
    ``Planner(scenario, scene=...)``
        复用**已有**的 :class:`~rm75_drake.planning.scene.SceneBundle`
        （避免重复装配，并让区域缓存跨规划复用）。
    ``Planner(scenario, region_cache=...)``
        复用已有的 :class:`~rm75_drake.planning.regions.RegionCache`
        （命中缓存时**不重跑 IRIS**，见 ``diagnostics["regions"]["cache_hit"]``）。

    公开方法
    --------
    :meth:`plan_task_space_target`
        任务空间目标（``pydrake.math.RigidTransform``）→ :class:`PlanResult`。
    :meth:`plan_joint_target`
        关节空间目标（7 维序列）→ :class:`PlanResult`。
    """

    def __init__(
        self,
        scenario: Any = None,
        *,
        scene: Any = None,
        region_cache: Any = None,
        options: Optional[PlanOptions] = None,
    ) -> None:
        """构造 Planner。

        参数
        ----
        scenario:
            :class:`rm75_drake.scenario.Scenario`（场景真相源）。
            ``scene`` 已给出时可为空。
        scene:
            可选的、**已装配好**的
            :class:`~rm75_drake.planning.scene.SceneBundle`。
        region_cache:
            可选的 :class:`~rm75_drake.planning.regions.RegionCache`。
        options:
            默认的 :class:`PlanOptions`（每次调用可再覆盖）。
        """
        if scene is None:
            if scenario is None:
                raise PlannerError(
                    "必须给出 scenario 或 scene 之一（场景真相源不能为空）",
                    reason="invalid_input",
                )
            scene = _scene.build_scene(scenario)
        self._scenario = scenario
        self._scene = scene
        self._options = options if options is not None else PlanOptions()
        if region_cache is None:
            region_cache = _regions.RegionCache(scene)
        self._region_cache = region_cache

    # -- 只读属性 --------------------------------------------------------
    @property
    def scene(self) -> Any:
        """场景装配产物（:class:`~rm75_drake.planning.scene.SceneBundle`）。"""
        return self._scene

    @property
    def region_cache(self) -> Any:
        """IRIS 区域缓存（:class:`~rm75_drake.planning.regions.RegionCache`）。"""
        return self._region_cache

    @property
    def options(self) -> PlanOptions:
        """默认选项。"""
        return self._options

    # -- 规划入口 --------------------------------------------------------
    def plan_task_space_target(
        self, target_pose: Any, *, options: Optional[PlanOptions] = None
    ) -> PlanResult:
        """规划到**任务空间目标**（末端 frame 在世界系下的位姿）。

        参数
        ----
        target_pose:
            目标位姿（``pydrake.math.RigidTransform``，世界系）。
        options:
            可选的选项覆盖（见 :class:`PlanOptions`）。

        返回
        ----
        :class:`PlanResult`：成功时含 5 ms 网格轨迹与夹爪调度表；
        失败时 ``success=False`` 且 ``stage="ik"``（IK 不可达 / 目标非法）。

        失败模式
        --------
        * 目标位姿数值非法（NaN/Inf / 类型不对）→ ``stage="ik"``、
          ``reason="invalid_target"``；
        * IK 不收敛 → ``stage="ik"``、``reason="unreachable"``；
        * IK 超时 → ``stage="ik"``、``reason="timeout"``。
        """
        opts = self._options.merged(options)
        diagnostics: Dict[str, Any] = {}
        started = time.perf_counter()

        # ── IK 阶段 ────────────────────────────────────────────────────
        ik_started = time.perf_counter()
        try:
            ik_result = _ik.solve_ik(
                self._scene,
                target_pose,
                nominal_posture=None,
                position_tol=opts.ik_position_tol,
                orientation_tol=opts.ik_orientation_tol,
                timeout=opts.ik_timeout,
                initial_posture=opts.ik_initial_posture,
            )
        except _ik.IkError as exc:
            diagnostics["ik"] = {
                "duration_s": time.perf_counter() - ik_started,
                "success": False,
                "reason": getattr(exc, "reason", "invalid_target"),
                "message": str(exc),
            }
            diagnostics["total_duration_s"] = time.perf_counter() - started
            return _failure(
                STAGE_IK,
                getattr(exc, "reason", "invalid_target"),
                f"IK 失败：{exc}",
                diagnostics,
            )
        diagnostics["ik"] = {
            "duration_s": time.perf_counter() - ik_started,
            "success": bool(ik_result.success),
            "reason": ik_result.reason,
            "iterations": int(ik_result.iterations),
            "position_error": float(ik_result.position_error),
            "orientation_error": float(ik_result.orientation_error),
            "message": ik_result.message,
        }
        if not ik_result.success:
            diagnostics["total_duration_s"] = time.perf_counter() - started
            return _failure(
                STAGE_IK,
                ik_result.reason or "unreachable",
                f"IK 失败（{ik_result.reason}）：{ik_result.message}",
                diagnostics,
            )

        # IK 之后走与关节目标完全相同的链路（**同源**）。
        return self._plan_joint(
            np.asarray(ik_result.q_array, dtype=float),
            opts,
            diagnostics,
            started,
        )

    def plan_joint_target(
        self, q_target: Sequence[float], *, options: Optional[PlanOptions] = None
    ) -> PlanResult:
        """规划到**关节空间目标**（7 维关节角序列）。

        参数
        ----
        q_target:
            目标关节角（rad，长度 7，顺序 joint_1..joint_7）。
        options:
            可选的选项覆盖（见 :class:`PlanOptions`）。

        返回
        ----
        :class:`PlanResult`。

        失败模式
        --------
        * 目标维度 / 数值非法 → ``stage="ik"``、``reason="invalid_target"``；
        * 目标越出**软件软限位** → ``stage="ik"``、
          ``reason="soft_limit_violation"``；
        * 起点不可行（IRIS 种子碰撞）→ ``stage="regions"``、
          ``reason="seed_infeasible"``；
        * 区域图不连通 → ``stage="gcs"``、``reason="no_path"``；
        * 重定时失败 → ``stage="retime"``、``reason`` 取重定时错误标签；
        * 轨迹碰撞复检有逃逸 → ``stage="collision"``、
          ``reason="collision_escape"``。
        """
        opts = self._options.merged(options)
        diagnostics: Dict[str, Any] = {}
        started = time.perf_counter()

        try:
            target = self._validate_joint_target(q_target)
        except PlannerError as exc:
            diagnostics["ik"] = {
                "duration_s": 0.0,
                "success": False,
                "reason": exc.reason,
                "message": str(exc),
                "skipped": True,
            }
            diagnostics["total_duration_s"] = time.perf_counter() - started
            return _failure(STAGE_IK, exc.reason, str(exc), diagnostics)

        diagnostics["ik"] = {
            "duration_s": 0.0,
            "success": True,
            "reason": "",
            "skipped": True,
            "message": "关节目标：不做 IK，可达性按软件软限位复核",
        }
        return self._plan_joint(target, opts, diagnostics, started)

    # -- 内部实现 --------------------------------------------------------
    def _validate_joint_target(self, q_target: Sequence[float]) -> np.ndarray:
        """校验关节目标（维度 / 有限性 / 软件软限位）。"""
        try:
            target = np.asarray(q_target, dtype=float).reshape(-1)
        except (TypeError, ValueError) as exc:
            raise PlannerError(
                f"关节目标必须是数值序列：{q_target!r}（{exc}）",
                stage=STAGE_IK,
                reason=_ik.REASON_INVALID_TARGET,
            ) from exc
        if target.size != JOINT_COUNT:
            raise PlannerError(
                f"关节目标必须是 {JOINT_COUNT} 维（RM75-B 7 DoF），实际 {target.size} 维",
                stage=STAGE_IK,
                reason=_ik.REASON_INVALID_TARGET,
            )
        if not np.all(np.isfinite(target)):
            raise PlannerError(
                f"关节目标含 NaN/Inf：{list(target)}",
                stage=STAGE_IK,
                reason=_ik.REASON_INVALID_TARGET,
            )
        try:
            names, lower, upper = _ik.software_joint_limits()
        except _limits.LimitsError as exc:
            raise PlannerError(
                f"软件限位交接件不可用：{exc}",
                stage=STAGE_IK,
                reason=REASON_LIMITS_ERROR,
            ) from exc
        violations = []
        for index in range(JOINT_COUNT):
            if target[index] < lower[index] - 1e-9 or target[index] > upper[index] + 1e-9:
                violations.append(
                    f"{names[index]}={target[index]:.6f} 越出软件软限位 "
                    f"[{lower[index]:.6f}, {upper[index]:.6f}]"
                )
        if violations:
            raise PlannerError(
                "关节目标不可达（越出软件软限位）：" + "；".join(violations),
                stage=STAGE_IK,
                reason=_ik.REASON_SOFT_LIMIT_VIOLATION,
            )
        return target

    def _start_configuration(self, opts: PlanOptions) -> np.ndarray:
        """返回起点关节角（选项优先，否则取场景当前位形）。"""
        if opts.start_configuration is not None:
            start = np.asarray(opts.start_configuration, dtype=float).reshape(-1)
            if start.size != JOINT_COUNT or not np.all(np.isfinite(start)):
                raise PlannerError(
                    f"start_configuration 必须是 {JOINT_COUNT} 维有限数值，实际："
                    f"{list(opts.start_configuration)}",
                    stage=STAGE_REGIONS,
                    reason=_regions.REASON_INVALID_SEED,
                )
            return start
        context = getattr(self._scene, "plant_context", None)
        plant = getattr(self._scene, "plant", None)
        if context is None or plant is None:
            raise PlannerError(
                "场景缺少 plant / plant_context，无法确定起点位形",
                stage=STAGE_REGIONS,
                reason=_regions.REASON_INVALID_SEED,
            )
        return np.asarray(plant.GetPositions(context), dtype=float).reshape(-1)[:JOINT_COUNT]

    def _plan_joint(
        self,
        target: np.ndarray,
        opts: PlanOptions,
        diagnostics: Dict[str, Any],
        started: float,
    ) -> PlanResult:
        """关节目标之后的链路：区域 → GCS → 重定时 → 碰撞复检。"""
        start = self._start_configuration(opts)
        diagnostics["start_configuration"] = [float(v) for v in start]
        diagnostics["target_configuration"] = [float(v) for v in target]

        # ── 区域阶段（IRIS 区域链）────────────────────────────────────
        regions_started = time.perf_counter()
        iris_runs_before = int(self._region_cache.iris_runs)
        try:
            seeds = _chain_seeds(start, target, opts.region_spacing)
            specs = [
                _regions.RegionSpec(seed_q=tuple(float(v) for v in seed), **opts.region_options)
                for seed in seeds
            ]
            region_list = [self._region_cache.get(spec) for spec in specs]
        except _regions.RegionError as exc:
            diagnostics["regions"] = {
                "duration_s": time.perf_counter() - regions_started,
                "success": False,
                "reason": exc.reason,
                "message": str(exc),
            }
            diagnostics["total_duration_s"] = time.perf_counter() - started
            return _failure(
                STAGE_REGIONS,
                exc.reason,
                f"IRIS 区域生成失败（{exc.reason}）：{exc}",
                diagnostics,
            )
        iris_runs = int(self._region_cache.iris_runs) - iris_runs_before
        diagnostics["regions"] = {
            "duration_s": time.perf_counter() - regions_started,
            "success": True,
            "num_regions": len(region_list),
            "iris_runs": iris_runs,
            "cache_hits": len(region_list) - iris_runs,
            "cache_hit": iris_runs == 0,
            "region_ids": [region.id for region in region_list],
            "seeds": [[float(v) for v in seed] for seed in seeds],
        }

        # ── GCS 阶段（区域图最短路）───────────────────────────────────
        gcs_started = time.perf_counter()
        threshold = _adjacency_threshold(seeds, opts.adjacency_factor)
        try:
            graph = _gcs.build_graph(region_list, adjacency_threshold=threshold)
            gcs_result = _gcs.solve_path(graph, region_list[0].id, region_list[-1].id)
        except _gcs.GcsError as exc:
            diagnostics["gcs"] = {
                "duration_s": time.perf_counter() - gcs_started,
                "success": False,
                "reason": exc.reason,
                "message": str(exc),
                "num_regions": len(region_list),
                "adjacency_threshold": threshold,
            }
            diagnostics["total_duration_s"] = time.perf_counter() - started
            return _failure(
                STAGE_GCS,
                exc.reason,
                f"空间 GCS 失败（{exc.reason}）：{exc}",
                diagnostics,
            )
        diagnostics["gcs"] = {
            "duration_s": time.perf_counter() - gcs_started,
            "success": bool(gcs_result.success),
            "reason": gcs_result.reason,
            "solver_mode": gcs_result.solver_mode,
            "num_regions": len(region_list),
            "num_edges": int(gcs_result.num_edges),
            "adjacency_threshold": threshold,
            "visited_regions": _visited_regions(gcs_result),
            "message": gcs_result.message,
        }
        if not gcs_result.success:
            diagnostics["total_duration_s"] = time.perf_counter() - started
            return _failure(
                STAGE_GCS,
                gcs_result.reason or _gcs.REASON_NO_PATH,
                f"空间 GCS 失败（{gcs_result.reason}）：{gcs_result.message}",
                diagnostics,
            )

        # ── 几何路径（区域链种子的稠密插值）──────────────────────────
        path = _densify(seeds, opts.path_step)

        # ── 重定时阶段（Toppra → 5 ms 网格）───────────────────────────
        retime_started = time.perf_counter()
        try:
            retimed = _retime.retime(
                path,
                dt=opts.dt,
                t0=opts.t0,
                robot_name=opts.robot_name,
                gripper_schedule=opts.gripper_schedule,
                duration_limit=opts.duration_limit,
            )
        except _retime.RetimeError as exc:
            reason = _extract_tag(str(exc), "parameterization")
            diagnostics["retime"] = {
                "duration_s": time.perf_counter() - retime_started,
                "success": False,
                "reason": reason,
                "message": str(exc),
            }
            diagnostics["total_duration_s"] = time.perf_counter() - started
            return _failure(
                STAGE_RETIME,
                reason,
                f"重定时失败（{reason}）：{exc}",
                diagnostics,
            )
        except _limits.LimitsError as exc:
            diagnostics["retime"] = {
                "duration_s": time.perf_counter() - retime_started,
                "success": False,
                "reason": REASON_LIMITS_ERROR,
                "message": str(exc),
            }
            diagnostics["total_duration_s"] = time.perf_counter() - started
            return _failure(
                STAGE_RETIME,
                REASON_LIMITS_ERROR,
                f"重定时失败（{REASON_LIMITS_ERROR}）：{exc}",
                diagnostics,
            )
        trajectory = retimed.to_trajectory()
        diagnostics["retime"] = {
            "duration_s": time.perf_counter() - retime_started,
            "success": True,
            "reason": "",
            "num_samples": len(trajectory),
            "trajectory_duration_s": float(trajectory.duration),
            "margin": float(retimed.margin),
            "peak_velocity": [float(v) for v in retimed.peak_velocity],
            "peak_acceleration": [float(v) for v in retimed.peak_acceleration],
        }

        # ── 碰撞复检（零逃逸是硬要求）─────────────────────────────────
        collision_started = time.perf_counter()
        try:
            recheck = _gcs.collision_recheck(
                self._scene,
                trajectory.q_matrix(),
                edge_step_size=opts.collision_edge_step_size,
            )
        except _gcs.GcsError as exc:
            diagnostics["collision"] = {
                "duration_s": time.perf_counter() - collision_started,
                "success": False,
                "reason": exc.reason,
                "message": str(exc),
                "num_escapes": -1,
            }
            diagnostics["total_duration_s"] = time.perf_counter() - started
            return _failure(
                STAGE_COLLISION,
                exc.reason,
                f"碰撞复检失败（{exc.reason}）：{exc}",
                diagnostics,
            )
        diagnostics["collision"] = {
            "duration_s": time.perf_counter() - collision_started,
            "success": bool(recheck.ok),
            "reason": "" if recheck.ok else "collision_escape",
            "num_samples": int(recheck.num_samples),
            "num_escapes": int(recheck.num_escapes),
            "escape_indices": list(recheck.escape_indices),
        }
        if not recheck.ok:
            diagnostics["total_duration_s"] = time.perf_counter() - started
            return _failure(
                STAGE_COLLISION,
                "collision_escape",
                (
                    f"碰撞复检发现 {recheck.num_escapes} 处逃逸"
                    f"（共复检 {recheck.num_samples} 个位形）"
                ),
                diagnostics,
            )

        diagnostics["total_duration_s"] = time.perf_counter() - started
        return PlanResult(
            success=True,
            message=(
                f"规划成功：{len(trajectory)} 个采样点（{opts.dt} s 网格），"
                f"时长 {trajectory.duration:.3f} s，区域 {len(region_list)} 个，"
                f"GCS 模式 {gcs_result.solver_mode or 'n/a'}，"
                f"碰撞复检逃逸 {recheck.num_escapes}"
            ),
            trajectory=trajectory,
            gripper_schedule=trajectory.gripper_schedule,
            diagnostics=diagnostics,
        )


# ── 内部工具 ────────────────────────────────────────────────────────────
def _failure(
    stage: str, reason: str, message: str, diagnostics: Mapping[str, Any]
) -> PlanResult:
    """构造失败结果（阶段 + 原因写进 message）。"""
    return PlanResult(
        success=False,
        message=f"[{stage}] 失败（{reason}）：{message}",
        trajectory=None,
        gripper_schedule=(),
        diagnostics=dict(diagnostics),
        stage=stage,
        reason=reason,
    )


def _chain_seeds(
    start: np.ndarray, target: np.ndarray, spacing: float
) -> List[np.ndarray]:
    """把起点 → 目标直线切成**间距 ≤ spacing** 的种子链。

    种子链是 IRIS 区域链的节点，也是几何路径的路径点（见模块 docstring
    「几何路径的来源」）。首尾必须精确等于起点 / 目标（规划必须到位）。
    """
    delta = np.asarray(target, dtype=float) - np.asarray(start, dtype=float)
    distance = float(np.linalg.norm(delta))
    steps = max(1, int(math.ceil(distance / float(spacing)))) if distance > 0 else 1
    seeds = [np.asarray(start, dtype=float) + delta * (index / steps) for index in range(steps + 1)]
    seeds[0] = np.asarray(start, dtype=float)
    seeds[-1] = np.asarray(target, dtype=float)
    return seeds


def _adjacency_threshold(seeds: Sequence[np.ndarray], factor: float) -> float:
    """邻接阈值 = factor × 相邻种子的最大间距（保证只连相邻区域）。"""
    step = 0.0
    for index in range(1, len(seeds)):
        step = max(step, float(np.linalg.norm(seeds[index] - seeds[index - 1])))
    return max(factor * step, 1e-9)


def _visited_regions(gcs_result: Any) -> List[str]:
    """从 GCS 结果里提取经过的区域 id（失败或不可解析时返回空表）。"""
    names: List[str] = []
    for edge in getattr(gcs_result, "edges", ()):  # pragma: no branch
        for endpoint in ("u", "v"):
            vertex = getattr(edge, endpoint, None)
            if vertex is None:
                continue
            name = getattr(vertex, "name", None)
            if callable(name):
                try:
                    name = name()
                except Exception:  # pragma: no cover - Drake 内部异常
                    name = None
            if isinstance(name, str) and name and name not in names:
                names.append(name)
    return names


def _densify(seeds: Sequence[np.ndarray], step: float) -> np.ndarray:
    """把种子链稠密化成相邻点间距 ≤ step 的点列。"""
    if not seeds:
        return np.zeros((0, JOINT_COUNT), dtype=float)
    rows: List[np.ndarray] = [np.asarray(seeds[0], dtype=float)]
    for index in range(1, len(seeds)):
        start = np.asarray(seeds[index - 1], dtype=float)
        end = np.asarray(seeds[index], dtype=float)
        distance = float(np.max(np.abs(end - start)))
        segments = max(1, int(math.ceil(distance / float(step))))
        for k in range(1, segments + 1):
            rows.append(start + (end - start) * (k / segments))
    return np.vstack(rows)


def _extract_tag(message: str, default: str) -> str:
    """从错误信息里抽出 ``[tag]`` 形式的稳定标签（取第一个）。"""
    start = message.find("[")
    if start < 0:
        return default
    end = message.find("]", start)
    if end < 0:
        return default
    return message[start + 1 : end] or default
