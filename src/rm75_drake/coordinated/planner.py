"""**协调规划 API**：参数化 → 区域图 → GCS → 14 维重定时 → 双轨迹 → 零逃逸复检（S05-T05）。

链路
----
1. **参数化**（:mod:`rm75_drake.coordinated.parameterization`）：把「相对位姿
   固定」这条 14 维零测集约束流形降成 **8 维**最小坐标
   （leader 7 关节 + follower ψ 1）。映射 ``x → q14`` 由解析 IK 闭式反解 ⇒
   约束**由构造保证**成立（残差 ~1e-16 m）。
2. **最小坐标路径**（本模块）：leader 关节沿直线推进，**ψ 连续跟踪**
   （每步取「离上一个 ψ 最近且可行」的取值）⇒ 路径上**每一点**都满足
   映射 / 限位 / 碰撞三项判定。
3. **区域图**（:mod:`rm75_drake.coordinated.regions`）：在 8 维最小坐标空间
   沿路径生成凸区域（自研；Drake 的 IRIS 绑定 plant 的完整关节空间，表达
   不了「8 维 + 映射回 14 维判碰撞」的自定义空间）。
4. **GCS**（:mod:`rm75_drake.planning.gcs`）：**复用** ``build_graph`` /
   ``solve_path``（维度无关入口；Mosek 在位走 ``mixed_integer``，不在位自动
   回退 ``convex_relaxation``——沿用既有能力探测，**不另写**）。GCS 在此
   承担**区域图连通性校验**（与 S02 的 ``planner.py`` 同构：几何路径来自
   区域链种子，GCS 负责连通性）。
5. **映射回 14 维**：最小坐标路径**逐点**映射（**逐点**满足相对位姿约束）。
6. **14 维重定时**（:mod:`rm75_drake.planning.retime` 的**维度无关核**
   ``retime_path``；``retime()`` 的 7 维公开签名与行为逐字未动）。
7. **双轨迹**：拆成两条 7 维 ``Trajectory`` 组成
   :class:`~rm75_drake.coordinated.trajectory.DualTrajectory`
   （共享时间轴由构造保证；复用 S02 的全部不变量校验）。
8. **14 维零逃逸复检**：走 **S04 的同一个 plant**（同一套过滤矩阵），
   逃逸数必须为 0（R017 的硬要求）。

阶段标签（沿用 S02 词表 + 新增一档）
------------------------------------
``parameterize`` → ``regions`` → ``gcs`` → ``retime`` → ``collision``
（S02 的 ``ik`` / ``regions`` / ``gcs`` / ``retime`` / ``collision`` 全部保留）。

降级链（显式、可解释，**禁止静默降级**）
----------------------------------------
区域为空 / 区域图无路 ⇒ 走**采样基线**（最小空间 PRM，**同一套 oracle**，
:func:`rm75_drake.coordinated.regions.sample_baseline_path`）+ 局部精修；
降级发生时 ``diagnostics["degraded"] == True`` 且 ``degradation_reason``
说明原因。全部失败 ⇒ ``success=False`` + ``reason`` + ``stage``，
**绝不返回半成品轨迹**。

为什么不直接复用 S02 的 ``plan_q_sequence`` / ``collision_recheck``
--------------------------------------------------------------------
那两处在 S02 里把 **7 维写死**（``JOINT_COUNT = 7``）：本模块需要的是
**14 维**复检，因此复检逻辑写在**本模块**里（``x → q14 → 14 维判定``），
而 GCS 的 ``build_graph`` / ``solve_path``（维度无关的部分）**照常复用**。

夹爪自由度口径
--------------
本模块只产出 **14 维臂关节**。场景装夹爪时 ``plant.num_positions() == 18``
（两夹爪各 2 个 prismatic），**碰撞查询前**必须按
:func:`rm75_drake.coordinated.limits.pad_gripper_positions` 补齐 4 个
prismatic（固定开度）。S04 的 ``include_grippers_in_collision`` 默认
``False``——夹爪默认不进入碰撞判定。

零 pydrake 依赖（模块级）
-------------------------
模块顶层只 import ``numpy`` + 本包内模块 + :mod:`rm75_drake.planning.retime`
（numpy + 惰性 toppra）。Drake 侧依赖（碰撞查询）一律**函数内惰性导入**，
因此 ``import rm75_drake.coordinated.planner`` **不会**把 pydrake 拉进
``sys.modules``。
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

import numpy as np

from .parameterization import (
    ARM_JOINT_COUNT,
    PARAMETER_DIM,
    CoordinatedParameterization,
    CoordinatedParameterizationError,
)
from .regions import (
    MINIMAL_DIM,
    MinimalRegionError,
    MinimalRegionSpec,
    generate_minimal_region,
    make_parameterization_oracle,
    minimal_model_version,
    sample_baseline_path,
)
from .trajectory import DUAL_JOINT_COUNT, DualTrajectory, dual_trajectory_from_arrays

__all__ = [
    "CoordinatedPlanError",
    "CoordinatedPlanOptions",
    "CoordinatedPlanResult",
    "plan_coordinated",
    "STAGE_PARAMETERIZE",
    "STAGE_REGIONS",
    "STAGE_GCS",
    "STAGE_RETIME",
    "STAGE_COLLISION",
    "STAGE_IK",
    "STAGES",
    "REASON_NO_PATH",
    "REASON_EMPTY_REGION",
    "REASON_COLLISION_ESCAPE",
    "REASON_PATH_INFEASIBLE",
    "REASON_INVALID_INPUT",
    "REASON_CONSTRAINT_VIOLATION",
    "IK_FAILURE_REASONS",
]


# ── 阶段标签 ─────────────────────────────────────────────────────────────
#: 参数化阶段（新增的一档；S02 的 ``ik`` 保留给解析 IK 内部失败）。
STAGE_PARAMETERIZE = "parameterize"

#: S02 词表（保留）：解析 IK 阶段。
STAGE_IK = "ik"

#: 区域阶段。
STAGE_REGIONS = "regions"

#: GCS 阶段。
STAGE_GCS = "gcs"

#: 重定时阶段。
STAGE_RETIME = "retime"

#: 碰撞复检阶段。
STAGE_COLLISION = "collision"

#: 全部阶段标签（按链路顺序）。
STAGES: Tuple[str, ...] = (
    STAGE_PARAMETERIZE,
    STAGE_IK,
    STAGE_REGIONS,
    STAGE_GCS,
    STAGE_RETIME,
    STAGE_COLLISION,
)

# ── 失败标签（稳定词表）────────────────────────────────────────────────
REASON_NO_PATH = "no_path"
REASON_EMPTY_REGION = "empty_region"
REASON_COLLISION_ESCAPE = "collision_escape"
REASON_PATH_INFEASIBLE = "path_infeasible"
REASON_INVALID_INPUT = "invalid_input"
REASON_CONSTRAINT_VIOLATION = "constraint_violation"

#: **解析 IK 的稳定失败词表**（沿用
#: :mod:`rm75_drake.kinematics.analytic_ik`）。
#: 参数化阶段只有落在该词表内的 ``reason`` 才会原样透出；其余（场景指派
#: 非法 / 维度错 / 变换非法 …）统一归一到 :data:`REASON_INVALID_INPUT`
#: ——**公开契约**：输入非法 ⇒ ``reason="invalid_input"``。
IK_FAILURE_REASONS = frozenset(
    {
        "unreachable",
        "branch_exhausted",
        "limit_violation",
        "wrist_singular",
        "invalid_target",
    }
)

#: 相对位姿残差阈值（**逐点**口径，m / rad）。
RESIDUAL_TOLERANCE = 1e-6


class CoordinatedPlanError(RuntimeError):
    """协调规划的**用法 / 输入**错误。

    ``stage`` / ``reason`` 是稳定标签，便于调用方分支处理。
    """

    def __init__(
        self,
        message: str,
        *,
        stage: str = STAGE_PARAMETERIZE,
        reason: str = REASON_INVALID_INPUT,
    ) -> None:
        super().__init__(message)
        self.stage = str(stage)
        self.reason = str(reason)


# ── 选项 ────────────────────────────────────────────────────────────────
def _finite_float(value: Any, name: str) -> float:
    """把选项字段规整成**有限浮点数**（非法 ⇒ :class:`CoordinatedPlanError`）。

    NaN / Inf / 非数值一律**显式报错并点名**（不得静默改写默认值）。
    """
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise CoordinatedPlanError(
            f"[options] {name} 必须是数值，实际：{value!r}"
        ) from exc
    if not math.isfinite(number):
        raise CoordinatedPlanError(
            f"[options] {name} 必须是有限值，实际：{value!r}"
        )
    return number


def _positive_int(value: Any, name: str) -> int:
    """把选项字段规整成**正整数**（非整数 / NaN ⇒ 显式报错并点名）。"""
    number = _finite_float(value, name)
    rounded = int(round(number))
    if abs(number - rounded) > 1e-9:
        raise CoordinatedPlanError(
            f"[options] {name} 必须是整数，实际：{value!r}"
        )
    return rounded


@dataclass(frozen=True)
class CoordinatedPlanOptions:
    """一次协调规划的可调旋钮（默认值即可跑常规紧协调任务）。

    字段
    ----
    num_regions:
        沿最小坐标路径生成的区域数（种子取自路径上的等距点）。
    region_step / region_max_expand / region_margin:
        区域膨胀的步长 / 最大探测步数 / 保守余量（语义同
        :class:`rm75_drake.coordinated.regions.MinimalRegionSpec`）。
    region_validation_samples / region_rounds:
        区域的保守性验证采样数 / 收缩轮数上限。
    path_step:
        **leader 关节路径**的推进步长（rad，逐轴上限）。该步长决定 14 维
        路径的相邻点间距 ⇒ 相对位姿残差的量级（重定时样条上的偏差
        ~ O(path_step²)）。
    dt:
        输出轨迹的采样周期（s），默认 5 ms。
    t0:
        输出轨迹首点的绝对时刻（s）。
    adjacency_threshold:
        GCS 图的邻接阈值（最小坐标空间的欧氏距离，rad）。
    collision_edge_step_size:
        14 维碰撞复检的密采样步长（rad）。
    allow_degradation:
        是否允许区域图失败时降级到采样基线（``False`` ⇒ 直接失败）。
    psi_grid:
        ψ 连续跟踪的扫描网格数（``[0, 2π)`` 上等距）。
    psi_min_margin:
        ψ 的**余量门槛**（以网格间距为单位）：要求选中的 ψ 离可行边界至少
        这么远（贴着边界的 ψ 膨胀不出有体积的区域）。
    random_seed:
        区域生成与采样基线的随机种子（**固定 ⇒ 可复现**）。
    """

    num_regions: int = 3
    region_step: float = 0.05
    region_max_expand: int = 8
    region_margin: float = 0.02
    region_validation_samples: int = 24
    region_rounds: int = 6
    path_step: float = 0.02
    dt: float = 0.005
    t0: float = 0.0
    adjacency_threshold: float = 1.0
    collision_edge_step_size: float = 0.02
    allow_degradation: bool = True
    psi_grid: int = 17
    psi_min_margin: float = 1.0
    random_seed: int = 20250516

    def __post_init__(self) -> None:
        if _positive_int(self.num_regions, "num_regions") < 1:
            raise CoordinatedPlanError(
                f"[options] num_regions 必须是正整数，实际：{self.num_regions!r}"
            )
        for name in ("region_step", "path_step", "dt"):
            value = _finite_float(getattr(self, name), name)
            if value <= 0.0:
                raise CoordinatedPlanError(
                    f"[options] {name} 必须是正有限值，实际：{getattr(self, name)!r}"
                )
        if _positive_int(self.region_max_expand, "region_max_expand") < 1:
            raise CoordinatedPlanError(
                "[options] region_max_expand 必须是正整数，"
                f"实际：{self.region_max_expand!r}"
            )
        for name in ("region_validation_samples", "region_rounds"):
            if _positive_int(getattr(self, name), name) < 1:
                raise CoordinatedPlanError(
                    f"[options] {name} 必须是正整数，实际：{getattr(self, name)!r}"
                )
        if not math.isfinite(_finite_float(self.t0, "t0")):
            raise CoordinatedPlanError(
                f"[options] t0 必须是有限值，实际：{self.t0!r}"
            )
        if _finite_float(self.adjacency_threshold, "adjacency_threshold") <= 0.0:
            raise CoordinatedPlanError(
                "[options] adjacency_threshold 必须是正有限值，"
                f"实际：{self.adjacency_threshold!r}"
            )
        if _finite_float(self.collision_edge_step_size, "collision_edge_step_size") <= 0.0:
            raise CoordinatedPlanError(
                "[options] collision_edge_step_size 必须是正有限值，"
                f"实际：{self.collision_edge_step_size!r}"
            )
        if _positive_int(self.psi_grid, "psi_grid") < 1:
            raise CoordinatedPlanError(
                f"[options] psi_grid 必须是正整数，实际：{self.psi_grid!r}"
            )


# ── 结果 ────────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class CoordinatedPlanResult:
    """一次协调规划的结果（**成功与失败都用它表达**，不抛异常）。

    字段
    ----
    success:
        是否规划成功。
    message:
        **始终非空**的可读说明（失败时含阶段 + 原因）。
    reason:
        失败标签（成功时为空串）。
    stage:
        失败阶段标签（成功时为空串）。
    left_trajectory / right_trajectory:
        成功时的两条 7 维轨迹（**共享时间轴**）；失败时为 ``None``。
    duration:
        端到端耗时（s）。
    diagnostics:
        诊断字典（区域数 / 各阶段耗时 / 相对位姿残差 max / 碰撞逃逸数 /
        ψ 与分支切换次数 / 降级标志与原因）。
    """

    success: bool
    message: str
    reason: str = ""
    stage: str = ""
    left_trajectory: Any = None
    right_trajectory: Any = None
    duration: float = 0.0
    diagnostics: Mapping[str, Any] = field(default_factory=dict)

    def __bool__(self) -> bool:
        """``bool(result)`` 等价于 ``result.success``。"""
        return bool(self.success)

    @property
    def dual_trajectory(self) -> Optional[DualTrajectory]:
        """成功时的 :class:`DualTrajectory`（失败时为 ``None``）。"""
        if self.left_trajectory is None or self.right_trajectory is None:
            return None
        return DualTrajectory(left=self.left_trajectory, right=self.right_trajectory)


# ── 主入口 ──────────────────────────────────────────────────────────────
def plan_coordinated(
    bundle: Any,
    *,
    x_lr: Any,
    start_leader_q: Sequence[float],
    goal_leader_q: Sequence[float],
    leader: str = "left",
    follower: str = "right",
    tool_offsets: Optional[Mapping[str, float]] = None,
    psi_start: Optional[float] = None,
    psi_goal: Optional[float] = None,
    options: Optional[CoordinatedPlanOptions] = None,
) -> CoordinatedPlanResult:
    """给定相对位姿约束，产出**两臂共享时间轴**的联合轨迹。

    参数
    ----
    bundle:
        :class:`rm75_drake.bimanual.dual_scene.DualSceneBundle`
        （**同一个 plant**；碰撞判定与 S04 完全一致）。
    x_lr:
        **常数**相对变换（4×4，leader TCP 系 → follower TCP 系）。
    start_leader_q / goal_leader_q:
        leader 臂的 7 维关节角（rad）——起点与终点。
    leader / follower:
        指派（``"left"`` / ``"right"``）。
    tool_offsets:
        两侧各自的工具偏置（m，沿 J7 轴）；``None`` → 裸臂默认值。
    psi_start / psi_goal:
        follower 的肘点方位角 ψ（rad）。``None`` → 连续跟踪挑一个可行值。
    options:
        规划选项（见 :class:`CoordinatedPlanOptions`）。

    返回
    ----
    :class:`CoordinatedPlanResult`：成功时含两条共享时间轴的 7 维轨迹；
    失败时 ``success=False`` 且带 ``stage`` / ``reason``。

    失败模式
    --------
    * 输入非法（维度 / 数值 / leader == follower）→ ``stage="parameterize"``、
      ``reason="invalid_input"``；
    * follower 无解（unreachable / branch_exhausted / limit_violation /
      wrist_singular）→ ``stage="parameterize"``，``reason`` **沿用解析 IK
      的稳定词表**；
    * 路径不可行（某个 leader 位姿找不到可行的 ψ）→
      ``stage="parameterize"``、``reason="path_infeasible"``；
    * 区域为空 → ``stage="regions"``、``reason="empty_region"``；
    * GCS 无路（且采样基线也失败）→ ``stage="gcs"``、``reason="no_path"``；
    * 重定时失败 → ``stage="retime"``、``reason`` 取重定时错误标签；
    * 14 维复检有逃逸 → ``stage="collision"``、``reason="collision_escape"``；
    * 相对位姿残差超阈值 → ``stage="collision"``、
      ``reason="constraint_violation"``。
    """
    started = time.perf_counter()
    opts = options if options is not None else CoordinatedPlanOptions()
    diagnostics: Dict[str, Any] = {
        "degraded": False,
        "degradation_reason": "",
    }

    # ── 参数化阶段 ──────────────────────────────────────────────────────
    param_started = time.perf_counter()
    try:
        parameterization = CoordinatedParameterization(
            bundle,
            x_lr=x_lr,
            leader=leader,
            follower=follower,
            tool_offsets=tool_offsets,
        )
        x_path = _build_minimal_path(
            parameterization,
            start_leader_q,
            goal_leader_q,
            psi_start=psi_start,
            psi_goal=psi_goal,
            opts=opts,
        )
    except CoordinatedParameterizationError as exc:
        reason = _normalize_parameterize_reason(exc.reason)
        diagnostics["parameterize"] = {
            "duration_s": time.perf_counter() - param_started,
            "success": False,
            "reason": reason,
            "message": str(exc),
            "num_points": 0,
        }
        diagnostics["total_duration_s"] = time.perf_counter() - started
        return _failure(
            STAGE_PARAMETERIZE,
            reason,
            f"参数化阶段失败（{reason}）：{exc}",
            diagnostics,
            started,
        )
    except CoordinatedPlanError as exc:
        reason = _normalize_parameterize_reason(exc.reason)
        diagnostics["parameterize"] = {
            "duration_s": time.perf_counter() - param_started,
            "success": False,
            "reason": reason,
            "message": str(exc),
            "num_points": 0,
        }
        diagnostics["total_duration_s"] = time.perf_counter() - started
        return _failure(
            STAGE_PARAMETERIZE,
            reason,
            f"参数化阶段失败（{reason}）：{exc}",
            diagnostics,
            started,
        )

    diagnostics["parameterize"] = {
        "duration_s": time.perf_counter() - param_started,
        "success": True,
        "reason": "",
        "num_points": int(len(x_path)),
        "x_start": [float(v) for v in x_path[0]],
        "x_goal": [float(v) for v in x_path[-1]],
    }

    # ── 区域阶段 ────────────────────────────────────────────────────────
    regions_started = time.perf_counter()
    oracle = make_parameterization_oracle(parameterization)
    try:
        model_version = minimal_model_version(bundle)
    except MinimalRegionError as exc:
        diagnostics["regions"] = {
            "duration_s": 0.0,
            "success": False,
            "reason": exc.reason,
            "message": str(exc),
            "num_regions": 0,
        }
        diagnostics["total_duration_s"] = time.perf_counter() - started
        return _failure(
            STAGE_REGIONS,
            exc.reason,
            f"区域阶段失败（{exc.reason}）：{exc}",
            diagnostics,
            started,
        )

    region_info = _build_regions(
        x_path,
        oracle,
        opts,
        model_version=model_version,
    )
    region_duration = time.perf_counter() - regions_started
    diagnostics["regions"] = {
        "duration_s": region_duration,
        "success": bool(region_info["success"]),
        "reason": region_info["reason"],
        "num_regions": int(region_info["num_regions"]),
        "region_ids": list(region_info["region_ids"]),
        "degraded": bool(region_info["degraded"]),
        "message": region_info["message"],
    }
    if region_info["degraded"]:
        diagnostics["degraded"] = True
        diagnostics["degradation_reason"] = region_info["reason"]
        diagnostics["degradation_message"] = region_info["message"]
        if not bool(opts.allow_degradation):
            # **不允许降级** ⇒ 区域链不完整直接失败（点名 reason），
            # 不得静默地继续或“假装成功”。
            diagnostics["total_duration_s"] = time.perf_counter() - started
            return _failure(
                STAGE_REGIONS,
                region_info["reason"] or REASON_EMPTY_REGION,
                (
                    f"区域阶段失败（{region_info['reason']}，"
                    f"allow_degradation=False）：{region_info['message']}"
                ),
                diagnostics,
                started,
            )
    if not region_info["success"]:
        diagnostics["total_duration_s"] = time.perf_counter() - started
        return _failure(
            STAGE_REGIONS,
            region_info["reason"],
            f"区域阶段失败（{region_info['reason']}）：{region_info['message']}",
            diagnostics,
            started,
        )

    regions = region_info["regions"]

    # ── GCS 阶段 ────────────────────────────────────────────────────────
    gcs_started = time.perf_counter()
    gcs_info = _solve_region_graph(regions, opts)
    gcs_duration = time.perf_counter() - gcs_started
    diagnostics["gcs"] = {
        "duration_s": gcs_duration,
        "success": bool(gcs_info["success"]),
        "reason": gcs_info["reason"],
        "solver_mode": gcs_info["solver_mode"],
        "num_edges": int(gcs_info["num_edges"]),
        "num_regions": len(regions),
        "message": gcs_info["message"],
    }
    if gcs_info["degraded"]:
        diagnostics["degraded"] = True
        diagnostics["degradation_reason"] = gcs_info["reason"]
        diagnostics["degradation_message"] = gcs_info["message"]
    if not gcs_info["success"]:
        diagnostics["total_duration_s"] = time.perf_counter() - started
        return _failure(
            STAGE_GCS,
            gcs_info["reason"] or REASON_NO_PATH,
            f"GCS 阶段失败（{gcs_info['reason']}）：{gcs_info['message']}",
            diagnostics,
            started,
        )

    # 区域图 / GCS 失败 ⇒ 走**采样基线**（最小空间 PRM，同一套 oracle）。
    if gcs_info["degraded"]:
        try:
            baseline = _sample_baseline_path_for(
                x_path[0], x_path[-1], oracle, opts
            )
        except MinimalRegionError as exc:
            diagnostics["total_duration_s"] = time.perf_counter() - started
            return _failure(
                STAGE_GCS,
                exc.reason or REASON_NO_PATH,
                (
                    f"GCS 阶段失败（{gcs_info['reason']}），采样基线亦失败"
                    f"（{exc.reason}）：{exc}"
                ),
                diagnostics,
                started,
            )
        x_path = baseline
        diagnostics["degradation_path_points"] = int(len(baseline))

    # ── 映射阶段（最小坐标 → 14 维）────────────────────────────────────
    map_started = time.perf_counter()
    try:
        q_path = _map_path(parameterization, x_path)
    except CoordinatedParameterizationError as exc:
        diagnostics["map"] = {
            "duration_s": time.perf_counter() - map_started,
            "success": False,
            "reason": exc.reason,
            "message": str(exc),
        }
        diagnostics["total_duration_s"] = time.perf_counter() - started
        return _failure(
            STAGE_PARAMETERIZE,
            exc.reason,
            f"映射回 14 维失败（{exc.reason}）：{exc}",
            diagnostics,
            started,
        )
    map_duration = time.perf_counter() - map_started
    diagnostics["map"] = {
        "duration_s": map_duration,
        "success": True,
        "reason": "",
        "num_points": int(len(q_path)),
    }

    # ── 重定时阶段（14 维，维度无关核）──────────────────────────────────
    retime_started = time.perf_counter()
    dual_limits = parameterization.dual_limits
    try:
        from ..planning.retime import retime_path

        retimed = retime_path(
            q_path,
            velocity_limits=dual_limits.velocity_array(),
            acceleration_limits=dual_limits.acceleration_array(),
            dt=opts.dt,
            t0=opts.t0,
            soft_limits=(dual_limits.lower_array(), dual_limits.upper_array()),
        )
    except Exception as exc:  # RetimeError / TrajectoryError / 其它
        reason = getattr(exc, "reason", "") or _extract_tag(str(exc), "parameterization")
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
            started,
        )
    retime_duration = time.perf_counter() - retime_started

    try:
        dual = dual_trajectory_from_arrays(
            retimed.q,
            t0=retimed.t0,
            dt=retimed.dt,
            left_robot_name=f"{parameterization.leader}_arm",
            right_robot_name=f"{parameterization.follower}_arm",
            acceleration_limits=dual_limits.acceleration_array()[:ARM_JOINT_COUNT],
        )
    except Exception as exc:
        reason = getattr(exc, "reason", "") or _extract_tag(str(exc), "trajectory")
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
            f"轨迹构造失败（{reason}）：{exc}",
            diagnostics,
            started,
        )

    diagnostics["retime"] = {
        "duration_s": retime_duration,
        "success": True,
        "reason": "",
        "num_samples": int(len(dual)),
        "trajectory_duration_s": float(dual.duration),
        "peak_velocity": [float(v) for v in retimed.peak_velocity],
        "peak_acceleration": [float(v) for v in retimed.peak_acceleration],
    }

    # ── 碰撞复检阶段（14 维，零逃逸是硬要求）───────────────────────────
    collision_started = time.perf_counter()
    try:
        recheck = _collision_recheck(
            bundle,
            dual.q_matrix(),
            edge_step_size=opts.collision_edge_step_size,
        )
    except Exception as exc:
        reason = getattr(exc, "reason", "") or REASON_INVALID_INPUT
        diagnostics["collision"] = {
            "duration_s": time.perf_counter() - collision_started,
            "success": False,
            "reason": reason,
            "message": str(exc),
            "num_escapes": -1,
        }
        diagnostics["total_duration_s"] = time.perf_counter() - started
        return _failure(
            STAGE_COLLISION,
            reason,
            f"碰撞复检失败（{reason}）：{exc}",
            diagnostics,
            started,
        )
    collision_duration = time.perf_counter() - collision_started

    # 相对位姿残差（**逐点**，m / rad）——构造性满足的验收口径。
    residuals = _relative_residuals(parameterization, dual.q_matrix())
    max_position = float(max((r[0] for r in residuals), default=0.0))
    max_orientation = float(max((r[1] for r in residuals), default=0.0))

    diagnostics["collision"] = {
        "duration_s": collision_duration,
        "success": bool(recheck["num_escapes"] == 0),
        "reason": "" if recheck["num_escapes"] == 0 else REASON_COLLISION_ESCAPE,
        "num_samples": int(recheck["num_samples"]),
        "num_escapes": int(recheck["num_escapes"]),
        "escape_indices": list(recheck["escape_indices"]),
    }
    diagnostics["relative_pose_residual_max_position_m"] = max_position
    diagnostics["relative_pose_residual_max_orientation_rad"] = max_orientation
    diagnostics["relative_pose_residual_max"] = max_position
    diagnostics["num_escapes"] = int(recheck["num_escapes"])
    diagnostics["num_regions"] = int(region_info["num_regions"])
    diagnostics["region_duration_s"] = float(region_duration)
    diagnostics["gcs_duration_s"] = float(gcs_duration)
    diagnostics["map_duration_s"] = float(map_duration)
    diagnostics["retime_duration_s"] = float(retime_duration)
    diagnostics["collision_duration_s"] = float(collision_duration)
    diagnostics["branch_switches"] = int(parameterization.diagnostics.branch_switches)
    diagnostics["psi_switches"] = int(parameterization.diagnostics.psi_switches)
    diagnostics["total_duration_s"] = time.perf_counter() - started

    if recheck["num_escapes"] > 0:
        return _failure(
            STAGE_COLLISION,
            REASON_COLLISION_ESCAPE,
            (
                f"14 维碰撞复检发现 {recheck['num_escapes']} 处逃逸"
                f"（共复检 {recheck['num_samples']} 个位形）"
            ),
            diagnostics,
            started,
        )
    if (
        max_position > RESIDUAL_TOLERANCE
        or max_orientation > RESIDUAL_TOLERANCE
    ):
        return _failure(
            STAGE_COLLISION,
            REASON_CONSTRAINT_VIOLATION,
            (
                f"相对位姿残差超阈值：位置 {max_position:.3e} m / 姿态 "
                f"{max_orientation:.3e} rad（阈值 {RESIDUAL_TOLERANCE:g}）"
            ),
            diagnostics,
            started,
        )

    return CoordinatedPlanResult(
        success=True,
        message=(
            f"协调规划成功：{len(dual)} 个采样点（{opts.dt} s 网格），"
            f"时长 {dual.duration:.3f} s，区域 {region_info['num_regions']} 个，"
            f"GCS 模式 {gcs_info['solver_mode'] or 'n/a'}，"
            f"碰撞复检逃逸 {recheck['num_escapes']}，"
            f"相对位姿残差 max {max_position:.3e} m / {max_orientation:.3e} rad"
        ),
        left_trajectory=dual.left,
        right_trajectory=dual.right,
        duration=float(diagnostics["total_duration_s"]),
        diagnostics=diagnostics,
    )


# ── 内部工具 ────────────────────────────────────────────────────────────
def _failure(
    stage: str,
    reason: str,
    message: str,
    diagnostics: Mapping[str, Any],
    started: float,
) -> CoordinatedPlanResult:
    """构造失败结果（阶段 + 原因写进 message）。"""
    payload = dict(diagnostics)
    payload.setdefault("total_duration_s", time.perf_counter() - started)
    return CoordinatedPlanResult(
        success=False,
        message=f"[{stage}] 失败（{reason}）：{message}",
        reason=str(reason),
        stage=str(stage),
        left_trajectory=None,
        right_trajectory=None,
        duration=float(payload.get("total_duration_s", 0.0)),
        diagnostics=payload,
    )


def _normalize_parameterize_reason(reason: str) -> str:
    """把参数化阶段的 ``reason`` 归一到**公开词表**。

    * 解析 IK 的稳定词表（:data:`IK_FAILURE_REASONS`）⇒ **原样透出**；
    * ``path_infeasible`` ⇒ 原样透出；
    * 其余（场景指派非法 / 维度错 / 变换非法 …）⇒ 归一到
      ``invalid_input``（**公开契约**：输入非法 ⇒ ``invalid_input``）。
    """
    text = str(reason)
    if text in IK_FAILURE_REASONS or text == REASON_PATH_INFEASIBLE:
        return text
    if text == REASON_INVALID_INPUT:
        return text
    return REASON_INVALID_INPUT


def _extract_tag(message: str, default: str) -> str:
    """从错误信息里抽出 ``[tag]`` 形式的稳定标签（取第一个）。"""
    start = message.find("[")
    if start < 0:
        return default
    end = message.find("]", start)
    if end < 0:
        return default
    return message[start + 1 : end]


def _as_leader_q(q: Sequence[float], label: str) -> np.ndarray:
    """把 leader 关节角规整成 7 维 float 数组（非法 → 显式报错并点名）。"""
    try:
        array = np.asarray(q, dtype=float).reshape(-1)
    except (TypeError, ValueError) as exc:
        raise CoordinatedPlanError(
            f"[input] {label} 的 leader 关节角必须是数值序列：{exc}"
        ) from exc
    if array.size != ARM_JOINT_COUNT:
        raise CoordinatedPlanError(
            f"[dimension] {label} 的 leader 关节角必须是 {ARM_JOINT_COUNT} 维"
            f"，实际：{array.size} 维",
            reason=REASON_INVALID_INPUT,
        )
    if not np.all(np.isfinite(array)):
        raise CoordinatedPlanError(
            f"[input] {label} 的 leader 关节角含 NaN/Inf",
            reason=REASON_INVALID_INPUT,
        )
    return array


def _psi_grid(grid: int) -> np.ndarray:
    """ψ 的均匀扫描网格（``[0, 2π)`` 上的 ``grid`` 个点）。"""
    count = max(2, int(grid))
    return np.linspace(0.0, 2.0 * math.pi, count, endpoint=False)


def _feasible_at(
    parameterization: CoordinatedParameterization,
    leader_q: np.ndarray,
    psi: float,
) -> Optional[np.ndarray]:
    """给定 (leader 关节角, ψ)，返回可行的 14 维位形；不可行返回 ``None``。

    判据 = 映射成功（解析 IK 有解）+ 14 维软件限位内 + 14 维碰撞自由
    （**同一个 plant**）。
    """
    x = np.concatenate([leader_q, np.array([float(psi)], dtype=float)])
    try:
        q14 = parameterization.to_joint_config(x)
    except CoordinatedParameterizationError:
        return None
    if not parameterization.within_limits(q14):
        return None
    if not parameterization.is_collision_free(q14):
        return None
    return q14


def _psi_margins(
    parameterization: CoordinatedParameterization,
    leader_q: np.ndarray,
    grid: np.ndarray,
) -> np.ndarray:
    """返回每个 ψ 网格点的可行性与**余量**（到最近不可行点的角距离，rad）。

    余量为 0 ⇒ 该 ψ 不可行；余量越大 ⇒ 离可行边界越远（**区域膨胀需要
    空间**：贴着边界的种子膨胀不出有体积的区域）。
    """
    count = len(grid)
    flags = np.zeros(count, dtype=bool)
    for index in range(count):
        flags[index] = _feasible_at(parameterization, leader_q, float(grid[index])) is not None
    margins = np.zeros(count, dtype=float)
    spacing = 2.0 * math.pi / count
    for index in range(count):
        if not flags[index]:
            continue
        distance = 0.0
        for offset in range(1, count + 1):
            if not flags[(index - offset) % count] or not flags[(index + offset) % count]:
                distance = (offset - 1) * spacing
                break
        else:  # pragma: no cover - 全可行时余量取半圈
            distance = math.pi
        margins[index] = distance
    return np.where(flags, margins, 0.0)


def _select_psi(
    margins: np.ndarray,
    grid: np.ndarray,
    psi_hint: Optional[float],
    *,
    min_margin: float,
) -> Optional[float]:
    """从可行 ψ 中挑一个：**余量够大**且**离上一个 ψ 最近**。

    * 余量 < ``min_margin`` 的候选被排除（贴着可行边界 ⇒ 膨胀不出区域）；
    * 在剩余候选里取「离 ``psi_hint`` 最近」者 ⇒ ψ 连续、不跳变；
    * ``psi_hint`` 为 ``None`` ⇒ 取余量最大的（最居中）候选。
    """
    usable = np.nonzero(margins >= float(min_margin))[0]
    if usable.size == 0:
        usable = np.nonzero(margins > 0.0)[0]
    if usable.size == 0:
        return None
    if psi_hint is None:
        best = int(usable[np.argmax(margins[usable])])
        return float(grid[best])
    hint = float(psi_hint)
    deltas = np.abs((grid[usable] - hint + math.pi) % (2.0 * math.pi) - math.pi)
    best = int(usable[int(np.argmin(deltas))])
    return float(grid[best])


def _build_minimal_path(
    parameterization: CoordinatedParameterization,
    start_leader_q: Sequence[float],
    goal_leader_q: Sequence[float],
    *,
    psi_start: Optional[float],
    psi_goal: Optional[float],
    opts: CoordinatedPlanOptions,
) -> np.ndarray:
    """构造 8 维最小坐标路径（**逐点可行**，ψ 连续跟踪）。

    leader 关节沿「起点 → 终点」直线按 ``path_step`` 推进；每一步取
    **离上一个 ψ 最近且可行**的 ψ ⇒ 路径上每一点都满足
    映射 / 限位 / 碰撞三项判定，且 ψ 连续（不跳变）。

    失败
    ----
    :class:`CoordinatedParameterizationError`
        ``reason`` ∈ ``invalid_input`` / ``unreachable`` /
        ``branch_exhausted`` / ``limit_violation`` / ``wrist_singular`` /
        ``path_infeasible``。
    """
    q0 = _as_leader_q(start_leader_q, "起点")
    q1 = _as_leader_q(goal_leader_q, "终点")
    step = float(opts.path_step)
    span = float(np.max(np.abs(q1 - q0)))
    segments = max(1, int(math.ceil(span / step))) if span > 0.0 else 1

    grid = _psi_grid(opts.psi_grid)
    spacing = 2.0 * math.pi / len(grid)
    # 余量门槛 = 一个网格间距：贴着可行边界的 ψ 膨胀不出有体积的区域。
    min_margin = float(opts.psi_min_margin) * spacing

    points: List[np.ndarray] = []
    psi_hint = None if psi_start is None else float(psi_start)
    for index in range(segments + 1):
        ratio = index / segments
        leader_q = q0 + (q1 - q0) * ratio
        margins = _psi_margins(parameterization, leader_q, grid)
        chosen = _select_psi(margins, grid, psi_hint, min_margin=min_margin)
        if chosen is None:
            reason, detail = _diagnose_psi_failure(
                parameterization, leader_q, grid, ratio
            )
            raise CoordinatedParameterizationError(
                f"[path] leader 位姿 {list(np.round(leader_q, 4))}（路径进度 "
                f"{ratio:.3f}）找不到可行的 ψ（ψ 网格 {len(grid)} 个候选全部不可行："
                f"{detail}）",
                reason=reason,
            )
        if index == segments and psi_goal is not None:
            # 终点允许调用方显式指定 ψ（但必须可行）。
            if _feasible_at(parameterization, leader_q, float(psi_goal)) is None:
                raise CoordinatedParameterizationError(
                    f"[path] 终点指定的 ψ={float(psi_goal)!r} 不可行",
                    reason=REASON_PATH_INFEASIBLE,
                )
            chosen = float(psi_goal)
        psi_hint = chosen
        points.append(
            np.concatenate([leader_q, np.array([chosen], dtype=float)])
        )

    if len(points) < 2:
        raise CoordinatedParameterizationError(
            "[path] 最小坐标路径至少需要 2 个点",
            reason="invalid_input",
        )
    return np.asarray(points, dtype=float)


def _diagnose_psi_failure(
    parameterization: CoordinatedParameterization,
    leader_q: np.ndarray,
    grid: np.ndarray,
    ratio: float,
) -> Tuple[str, str]:
    """诊断「某个 leader 位姿上**所有** ψ 都不可行」的**根因**。

    逐个 ψ 重试 :meth:`CoordinatedParameterization.to_joint_config`，把失败
    归类：

    * **全部**候选都因**解析 IK** 失败（:data:`IK_FAILURE_REASONS`）⇒ 返回
      该 IK 原因（**点名** ``unreachable`` / ``branch_exhausted`` /
      ``limit_violation`` / ``wrist_singular``）；
    * 其余情形（限位 / 碰撞 / 混合）⇒ ``path_infeasible``。

    返回 ``(reason, detail)``。
    """
    reasons: List[str] = []
    details: List[str] = []
    for psi in grid:
        x = np.concatenate([leader_q, np.array([float(psi)], dtype=float)])
        try:
            q14 = parameterization.to_joint_config(x)
        except CoordinatedParameterizationError as exc:
            reasons.append(str(exc.reason))
            details.append(str(exc))
            continue
        if not parameterization.within_limits(q14):
            reasons.append("limit_violation")
            details.append(f"ψ={float(psi)!r} 超出 14 维软件限位")
            continue
        reasons.append("collision")
        details.append(f"ψ={float(psi)!r} 在同一 plant 里碰撞")
    if reasons and all(item in IK_FAILURE_REASONS for item in reasons):
        counted = {item: reasons.count(item) for item in set(reasons)}
        reason = max(sorted(counted), key=lambda item: counted[item])
        return reason, f"解析 IK 失败（{reason}）：{details[0]}"
    return (
        REASON_PATH_INFEASIBLE,
        "follower 无解 / 限位越界 / 碰撞（原因不唯一）",
    )


def _region_seeds(x_path: np.ndarray, count: int) -> List[np.ndarray]:
    """从最小坐标路径上等距取 ``count`` 个区域种子（含首尾）。"""
    number = max(1, int(count))
    indices = np.unique(
        np.linspace(0, len(x_path) - 1, number).round().astype(int)
    )
    return [np.asarray(x_path[index], dtype=float) for index in indices]


def _build_regions(
    x_path: np.ndarray,
    oracle: Any,
    opts: CoordinatedPlanOptions,
    *,
    model_version: str,
) -> Dict[str, Any]:
    """沿最小坐标路径生成区域链（**失败必须点名**）。

    某个种子区域生成失败 / 为空 ⇒ 记为**降级**（``degraded=True`` +
    ``reason``）并继续（几何路径仍然可行）；全部种子都失败 ⇒ 走**采样基线**
    （同一套 oracle）；采样基线也失败 ⇒ 失败（点名 reason）。
    """
    seeds = _region_seeds(x_path, int(opts.num_regions))
    regions: List[Any] = []
    empty_reason = ""
    empty_message = ""
    for seed in seeds:
        spec = MinimalRegionSpec(
            seed_x=tuple(float(v) for v in seed),
            dim=MINIMAL_DIM,
            random_seed=int(opts.random_seed),
            step=float(opts.region_step),
            max_expand=int(opts.region_max_expand),
            margin=float(opts.region_margin),
            num_validation_samples=int(opts.region_validation_samples),
            max_rounds=int(opts.region_rounds),
        )
        try:
            region = generate_minimal_region(spec, oracle, model_version=model_version)
        except MinimalRegionError as exc:
            empty_reason = exc.reason
            empty_message = (
                f"区域生成失败（种子 {list(np.round(seed, 4))}）：{exc}"
            )
            break
        if region.is_empty:
            empty_reason = REASON_EMPTY_REGION
            empty_message = (
                f"区域为空（种子 {list(np.round(seed, 4))}，reason="
                f"{region.reason or 'empty_region'}）"
            )
            break
        regions.append(region)

    if not empty_reason:
        return {
            "success": True,
            "reason": "",
            "num_regions": len(regions),
            "region_ids": [region.id for region in regions],
            "regions": regions,
            "degraded": False,
            "message": "",
        }

    # 区域链不完整 ⇒ 降级（采样基线在 GCS 阶段兜底）；先记录降级原因。
    return {
        "success": True,
        "reason": empty_reason,
        "num_regions": len(regions),
        "region_ids": [region.id for region in regions],
        "regions": regions,
        "degraded": True,
        "message": empty_message,
    }


def _solve_region_graph(regions: Sequence[Any], opts: CoordinatedPlanOptions) -> Dict[str, Any]:
    """在区域图上求最短路（**连通性校验**；失败 ⇒ 降级或点名失败）。

    * 正常：GCS 求解成功 ⇒ 返回 ``solver_mode`` / 边数；
    * 降级：GCS 无路 ⇒ 走**采样基线**（调用方负责，此处只标注），
      ``degraded=True``；
    * 全败：``success=False`` + ``reason``。
    """
    from ..planning import gcs as _gcs

    if not regions:
        # 区域链为空 ⇒ **降级**（由调用方走采样基线兜底）；不允许降级时
        # 调用方（``allow_degradation=False``）已在区域阶段直接失败。
        return {
            "success": True,
            "reason": REASON_EMPTY_REGION,
            "solver_mode": "",
            "num_edges": 0,
            "degraded": True,
            "message": "区域链为空，已降级到采样基线",
        }
    solver_mode = ""
    num_edges = 0
    try:
        graph = _gcs.build_graph(
            [region.as_graph_entry() for region in regions],
            adjacency_threshold=float(opts.adjacency_threshold),
        )
        result = _gcs.solve_path(graph, regions[0].id, regions[-1].id)
        solver_mode = result.solver_mode
        num_edges = int(result.num_edges)
        if not result.success:
            raise _gcs.GcsError(
                result.message or "区域图上无可行边路径",
                reason=result.reason or _gcs.REASON_NO_PATH,
            )
    except Exception as exc:
        reason = getattr(exc, "reason", "") or REASON_NO_PATH
        message = str(exc)
        if not bool(opts.allow_degradation):
            return {
                "success": False,
                "reason": reason,
                "solver_mode": solver_mode,
                "num_edges": num_edges,
                "degraded": False,
                "message": message,
            }
        return {
            "success": True,
            "reason": reason,
            "solver_mode": solver_mode,
            "num_edges": num_edges,
            "degraded": True,
            "message": f"区域图求解失败（{reason}），已降级到采样基线：{message}",
        }
    return {
        "success": True,
        "reason": "",
        "solver_mode": solver_mode,
        "num_edges": num_edges,
        "degraded": False,
        "message": "",
    }


def _map_path(
    parameterization: CoordinatedParameterization, x_path: np.ndarray
) -> np.ndarray:
    """把最小坐标路径逐点映射成 14 维关节路径（**逐点**满足相对位姿约束）。"""
    rows: List[np.ndarray] = []
    for point in np.asarray(x_path, dtype=float):
        if point.size != PARAMETER_DIM:
            raise CoordinatedParameterizationError(
                f"[dimension] 最小坐标点必须是 {PARAMETER_DIM} 维，实际 {point.size} 维",
                reason="invalid_dimension",
            )
        rows.append(parameterization.to_joint_config(point))
    if len(rows) < 2:
        raise CoordinatedParameterizationError(
            "[path] 映射后的路径至少需要 2 个点",
            reason="invalid_input",
        )
    return np.asarray(rows, dtype=float)


def _collision_recheck(
    bundle: Any, q14: np.ndarray, *, edge_step_size: float
) -> Dict[str, Any]:
    """14 维密采样碰撞复检（**同一个 plant**；逃逸数必须为 0）。"""
    from ..bimanual.dual_scene import check_configs_free
    from .limits import pad_gripper_positions

    matrix = np.asarray(q14, dtype=float)
    if matrix.ndim != 2 or matrix.shape[1] != DUAL_JOINT_COUNT:
        raise CoordinatedPlanError(
            f"[dimension] 复检路径必须是 (N, {DUAL_JOINT_COUNT}) 的矩阵，"
            f"实际形状：{matrix.shape}",
            stage=STAGE_COLLISION,
            reason=REASON_INVALID_INPUT,
        )
    step = float(edge_step_size)
    dense = _densify_q(matrix, step)
    padded = pad_gripper_positions(
        dense,
        gripper_joint_names=getattr(bundle, "gripper_joint_names", ()) or (),
    )
    flags = check_configs_free(bundle, padded)
    escapes = [index for index, flag in enumerate(flags) if not bool(flag)]
    return {
        "num_samples": len(flags),
        "num_escapes": len(escapes),
        "escape_indices": tuple(escapes),
    }


def _densify_q(matrix: np.ndarray, step: float) -> np.ndarray:
    """把 14 维点列稠密化（相邻点无穷范数距离 ≤ ``step``）。"""
    rows: List[np.ndarray] = [np.asarray(matrix[0], dtype=float)]
    for index in range(1, matrix.shape[0]):
        start = np.asarray(matrix[index - 1], dtype=float)
        end = np.asarray(matrix[index], dtype=float)
        distance = float(np.max(np.abs(end - start)))
        segments = max(1, int(math.ceil(distance / step)))
        for k in range(1, segments + 1):
            rows.append(start + (end - start) * (k / segments))
    return np.vstack(rows)


def _relative_residuals(
    parameterization: CoordinatedParameterization, q14: np.ndarray
) -> List[Tuple[float, float]]:
    """逐点相对位姿残差（位置 m / 姿态 rad）。"""
    out: List[Tuple[float, float]] = []
    for row in np.asarray(q14, dtype=float):
        residual = parameterization.relative_pose(row)
        out.append(
            (float(residual.position_error), float(residual.orientation_error))
        )
    return out


def _sample_baseline_path_for(
    x_start: np.ndarray,
    x_goal: np.ndarray,
    oracle: Any,
    opts: CoordinatedPlanOptions,
) -> np.ndarray:
    """采样基线（最小空间 PRM，**同一套 oracle**）——降级路径的公共入口。

    PRM 路径往往只有两个端点；重定时阶段的样条在两个**约束满足点**之间
    插值 ⇒ 相对位姿残差 ~ O(间距²)。因此基线取回后**在最小坐标空间稠密化**
    （只保留 oracle 判为可行的中间点），把残差压回阈值以内。
    """
    baseline = sample_baseline_path(
        x_start,
        x_goal,
        oracle,
        random_seed=int(opts.random_seed),
    )
    return _densify_minimal_path(baseline, oracle, float(opts.path_step))


def _densify_minimal_path(
    path: np.ndarray, oracle: Any, step: float
) -> np.ndarray:
    """在最小坐标空间稠密化一条路径（**同一套 oracle** 判定中间点可行）。"""
    array = np.asarray(path, dtype=float)
    if array.ndim != 2 or array.shape[0] < 2 or not math.isfinite(step) or step <= 0.0:
        return array
    rows: List[np.ndarray] = [np.asarray(array[0], dtype=float)]
    for index in range(1, array.shape[0]):
        start = np.asarray(array[index - 1], dtype=float)
        end = np.asarray(array[index], dtype=float)
        distance = float(np.max(np.abs(end - start)))
        segments = max(1, int(math.ceil(distance / step)))
        for k in range(1, segments):
            candidate = start + (end - start) * (k / segments)
            try:
                flags = oracle(candidate.reshape(1, MINIMAL_DIM))
            except MinimalRegionError:
                continue
            if bool(np.asarray(flags).reshape(-1)[0]):
                rows.append(candidate)
        rows.append(np.asarray(end, dtype=float))
    return np.vstack(rows) if len(rows) > 1 else array
