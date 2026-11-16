"""单臂求解器**快路径**与原语编排（S06-T03）。

为什么走「快路径」
------------------
本机实测：单臂 :class:`rm75_drake.planner.Planner`（IRIS 区域路径）**> 300 s
未返回**一次 ``plan_task_space_target``；而

* ``build_scene_from_path`` ≈ 0.08 s；
* ``collision_recheck``（50 点）≈ 0.011 s；
* ``solve_analytic_ik``（全枚举）≈ 9.6 ms（限分支 ≈ 0.8 ms）。

因此**单臂原语**一律走 **numpy 快路径**：

    目标位姿 → 解析 IK → 关节直线插值 → 碰撞复检 → 重定时（5 ms 网格）

这条链路**不依赖 IRIS 区域**，因此整条原语链可以在**秒级**内离线跑完
（离线门槛必须挂在可秒级完成的链路上，见 R011）。

单一求解器入口
--------------
:func:`solve_task` 是**唯一**的任务级求解入口：按任务文件里 ``primitives``
的顺序逐原语求解，返回逐原语结果（:class:`PrimitiveResult`）与任务级汇总
（:class:`TaskResult`）。状态机（S06-T04）只调这一个入口。

失败语义（稳定词表）
--------------------
``stage`` ∈ ``{schema, ik, path, collision, retime, gripper, verify}``；
``reason`` 是稳定的小写词表：

======================  ====================================================
``stage``               典型 ``reason``
======================  ====================================================
``schema``              ``unknown_primitive`` / ``missing_target`` /
                        ``unknown_object`` / ``unsupported_kind`` /
                        ``missing_scene``
``ik``                  解析 IK 词表：``unreachable`` /
                        ``branch_exhausted`` / ``limit_violation`` /
                        ``wrist_singular`` / ``invalid_target``
``path``                ``interpolation_failed`` / ``tolerance_exceeded``
``collision``           ``escape`` / ``recheck_failed``
``retime``              重定时标签：``parameterization`` / ``continuity`` /
                        ``velocity`` / ``acceleration`` / ``duration`` /
                        ``dependency`` / ``path`` / ``shape`` …
``gripper``             ``transport_error`` / ``unsupported_action``
``verify``              ``gripper_offline`` / ``gripper_not_closed`` /
                        ``transport_error`` / ``no_transport``
======================  ====================================================

纪律
----
* **不改冻结类型**：:class:`rm75_drake.planning.trajectory.Trajectory` /
  :class:`~rm75_drake.planning.trajectory.TrajectorySample` /
  :class:`rm75_drake.gripper.GripperAction` /
  :mod:`rm75_drake.recording` 一律**只读消费**；夹爪动作通过
  :func:`dataclasses.replace` 挂到采样点的 ``gripper_action`` 字段上
  （该字段本来就是为此预留的，:meth:`Trajectory._validate` 只做 isinstance 检查）。
* **零 pydrake 依赖**：本模块 import 时**不**拉起 pydrake（碰撞复检所需的
  场景由调用方装配后传入）。
* **不 import** :mod:`rm75_drake.drivers`（P002 同源分层边界纪律）；
  允许 import :mod:`rm75_drake.device.transport`（夹爪下发通道）。
* 误差测量一律用 :func:`rm75_drake.kinematics.mdh.fk`（numpy-only 的
  **独立被测对照**）：MDH 与 Drake FK 之间固有 ~2.24e-6 m 的模型差异
  （官方 URDF 的 rpy 舍入），**测得值如实报**，不掩盖、不修正。
"""

from __future__ import annotations

import dataclasses
import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

from ..gripper import GripperAction, close_action, open_action
from ..kinematics import analytic_ik
from ..kinematics import mdh
from ..planning import gcs as _gcs
from ..planning import retime as _retime
from ..planning.trajectory import (
    DEFAULT_DT,
    Trajectory,
    TrajectoryError,
    TrajectorySample,
)
from .goals import TaskError
from .primitives import (
    DEFAULT_ORIENTATION_TOLERANCE_RAD,
    DEFAULT_POSITION_TOLERANCE_M,
    PrimitiveSpec,
    primitive_spec,
)
from .schema import TaskSpec

__all__ = [
    "STAGES",
    "REASONS",
    "SolverOptions",
    "SolveState",
    "PrimitiveResult",
    "TaskResult",
    "solve_task",
    "solve_primitive",
]


# ── 失败语义词表 ────────────────────────────────────────────────────────
#: 失败阶段（稳定词表）。
STAGES: Tuple[str, ...] = (
    "schema",
    "ik",
    "path",
    "collision",
    "retime",
    "gripper",
    "verify",
)

#: 失败原因（稳定词表；``ik`` 段的取值直接来自解析 IK 的 reason 词表）。
REASONS: Tuple[str, ...] = (
    # schema
    "unknown_primitive",
    "missing_target",
    "unknown_object",
    "unsupported_kind",
    "missing_scene",
    # ik（与 rm75_drake.kinematics.analytic_ik 的 reason 同名）
    "unreachable",
    "branch_exhausted",
    "limit_violation",
    "wrist_singular",
    "invalid_target",
    # path
    "interpolation_failed",
    "tolerance_exceeded",
    # collision
    "escape",
    "recheck_failed",
    # retime（与 rm75_drake.planning.retime 的标签同名）
    "parameterization",
    "continuity",
    "velocity",
    "acceleration",
    "duration",
    "dependency",
    "path",
    "shape",
    "finite",
    "dt",
    "limits",
    # gripper
    "transport_error",
    "unsupported_action",
    # verify
    "gripper_offline",
    "gripper_not_closed",
    "no_transport",
)

#: 任务级成功时的 ``stage``（不在失败词表内）。
STAGE_DONE = "done"

#: 任务级成功时的 ``reason``。
REASON_COMPLETED = "completed"


# ── 数据结构 ────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class SolverOptions:
    """求解器参数（**全部有默认值**，缺省即可跑离线快路径）。

    字段
    ----
    initial_q:
        起始关节角（rad，7 维）。默认全零（等价于「零位」）。
    dt:
        重定时采样周期（s），默认 5 ms。
    tool_offset:
        TCP 工具偏置（m，沿 J7 轴）。``None`` → :data:`rm75_drake.kinematics.mdh.MDH_D[6]`
        （裸臂 0.144 m，与解析 IK 的默认工具偏置**同一个**）。
    joint_step_rad:
        关节直线插值的**相邻关节角步长上限**（rad），默认 0.02。
    position_tolerance_m:
        覆盖位置误差门槛（m）；``None`` → 原语 / 任务 / 默认三级取值。
    orientation_tolerance_rad:
        覆盖姿态误差门槛（rad）；``None`` → 原语 / 任务 / 默认三级取值。
    verify_expected_position:
        ``verify`` 原语期望的夹爪开口指令（0–1000）；``None`` → 按原语的
        ``action`` 推断（``close`` → 0，``open`` → 1000）。
    verify_position_tolerance:
        ``verify`` 原语允许的开口偏差（指令单位，0–1000）。
    default_hold_seconds:
        夹爪原语缺省保持时长（s）。
    robot_name:
        写入轨迹的机器人名。
    """

    initial_q: Tuple[float, ...] = (0.0,) * 7
    dt: float = DEFAULT_DT
    tool_offset: Optional[float] = None
    joint_step_rad: float = 0.02
    position_tolerance_m: Optional[float] = None
    orientation_tolerance_rad: Optional[float] = None
    verify_expected_position: Optional[int] = None
    verify_position_tolerance: int = 50
    default_hold_seconds: float = 0.5
    robot_name: str = "rm75"

    def __post_init__(self) -> None:
        q = tuple(float(v) for v in self.initial_q)
        if len(q) != mdh.N_JOINTS:
            raise TaskError(
                f"initial_q 必须是 {mdh.N_JOINTS} 维（rad），收到 {len(q)} 维"
            )
        object.__setattr__(self, "initial_q", q)
        if not math.isfinite(float(self.dt)) or float(self.dt) <= 0.0:
            raise TaskError(f"dt 必须是正有限值（s），收到 {self.dt!r}")
        if not math.isfinite(float(self.joint_step_rad)) or float(self.joint_step_rad) <= 0.0:
            raise TaskError(
                f"joint_step_rad 必须是正有限值（rad），收到 {self.joint_step_rad!r}"
            )


@dataclass(frozen=True)
class PrimitiveResult:
    """**单个原语**的求解结果。

    字段
    ----
    name:
        原语名。
    goal_id:
        原子目标 id（来自任务文件，任务内唯一）。
    success:
        是否成功。
    stage:
        失败阶段（成功时为 ``"done"``）；取值见 :data:`STAGES`。
    reason:
        失败原因（稳定词表）；成功时为 ``"completed"``。
    trajectory:
        产出的 5 ms 网格轨迹（夹爪 ``verify`` 原语无轨迹 → ``None``）。
    position_error_m:
        TCP 位置误差（m，numpy 独立对照测得）；不适用 → ``None``。
    orientation_error_rad:
        TCP 姿态误差（rad）；不适用 → ``None``。
    duration_s:
        该原语耗时（s，轨迹时长）。
    command_issued:
        是否**真的下发**了夹爪指令（无 transport ⇒ ``False``，即**未验证**）。
    message:
        人类可读的补充说明（失败原因 / 跳过原因）。
    """

    name: str
    goal_id: str
    success: bool
    stage: str
    reason: str
    trajectory: Optional[Trajectory] = None
    position_error_m: Optional[float] = None
    orientation_error_rad: Optional[float] = None
    duration_s: float = 0.0
    command_issued: bool = False
    message: str = ""

    def as_dict(self) -> Dict[str, Any]:
        """返回可序列化的字典视图（指标 / 审计用；轨迹本身不展开）。"""
        return {
            "name": self.name,
            "goal_id": self.goal_id,
            "success": self.success,
            "stage": self.stage,
            "reason": self.reason,
            "position_error_m": self.position_error_m,
            "orientation_error_rad": self.orientation_error_rad,
            "duration_s": self.duration_s,
            "command_issued": self.command_issued,
            "message": self.message,
        }


@dataclass(frozen=True)
class TaskResult:
    """**整个任务**的求解结果（原语结果的汇总）。

    字段
    ----
    task_name:
        任务名。
    success:
        是否全部原语成功。
    stage:
        首个失败原语的阶段（成功时为 ``"done"``）。
    reason:
        首个失败原语的原因（成功时为 ``"completed"``）。
    results:
        逐原语结果（**按执行顺序**；失败后不再继续执行后续原语）。
    duration_s:
        全部原语耗时之和（s）。
    message:
        人类可读说明。
    """

    task_name: str
    success: bool
    stage: str
    reason: str
    results: Tuple[PrimitiveResult, ...] = ()
    duration_s: float = 0.0
    message: str = ""

    @property
    def primitives(self) -> Tuple[PrimitiveResult, ...]:
        """逐原语结果（别名，便于指标口径按「原语」聚合）。"""
        return self.results

    def as_dict(self) -> Dict[str, Any]:
        """返回可序列化的字典视图。"""
        return {
            "task_name": self.task_name,
            "success": self.success,
            "stage": self.stage,
            "reason": self.reason,
            "duration_s": self.duration_s,
            "message": self.message,
            "results": [item.as_dict() for item in self.results],
        }


# ── 内部状态 ────────────────────────────────────────────────────────────
@dataclass
class SolveState:
    """一次求解过程中的**可变游标**（当前关节角 + 时间轴）。

    它**不是**任务状态（任务状态由 S06-T04 的状态机单一持有），
    只描述「几何 / 时间游标」。
    """

    current_q: np.ndarray
    t: float = 0.0


class _PrimitiveFailure(Exception):
    """内部信号：某个原语失败（携带 stage / reason / message）。"""

    def __init__(self, stage: str, reason: str, message: str) -> None:
        super().__init__(message)
        self.stage = stage
        self.reason = reason
        self.message = message


# ── 几何小工具 ──────────────────────────────────────────────────────────
def _rpy_to_rotation(rpy: Sequence[float]) -> np.ndarray:
    """rpy（rad，**内旋 xyz**）→ 3×3 旋转矩阵。

    约定与 :func:`rm75_drake.drake_compat.rotation_from_rpy` 一致：
    ``R = Rz(yaw)·Ry(pitch)·Rx(roll)``。
    """
    roll, pitch, yaw = (float(v) for v in rpy)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def _pose_matrix(xyz: Sequence[float], rpy: Sequence[float]) -> np.ndarray:
    """世界系位姿 → 4×4 齐次矩阵。"""
    pose = np.eye(4)
    pose[:3, 3] = np.asarray(xyz, dtype=float)
    pose[:3, :3] = _rpy_to_rotation(rpy)
    return pose


def _angle_between(r1: np.ndarray, r2: np.ndarray) -> float:
    """两个旋转矩阵之间的夹角（rad，``atan2`` 口径，数值稳定）。"""
    rel = r1.T @ r2
    skew = 0.5 * (rel - rel.T)
    sin_theta = float(np.linalg.norm(skew))
    cos_theta = (float(np.trace(rel)) - 1.0) * 0.5
    cos_theta = min(1.0, max(-1.0, cos_theta))
    return float(math.atan2(sin_theta, cos_theta))


def _tcp_pose(q: Sequence[float], tool_offset: float) -> Tuple[np.ndarray, np.ndarray]:
    """用 numpy 独立对照（:func:`rm75_drake.kinematics.mdh.fk`）算 TCP 位姿。

    TCP = link_7 原点 + ``(tool_offset − d7)`` 沿 link_7 的 z 轴偏移
    （与解析 IK 的 TCP 口径**完全一致**）。
    """
    frame = mdh.fk(q)
    rot = frame[:3, :3]
    offset = float(tool_offset) - float(mdh.MDH_D[6])
    position = frame[:3, 3] + offset * rot[:, 2]
    return position, rot


def _interpolate(
    q_start: np.ndarray, q_end: np.ndarray, step_rad: float
) -> np.ndarray:
    """关节直线插值（相邻关节角步长 ≤ ``step_rad``）。"""
    delta = np.asarray(q_end, dtype=float) - np.asarray(q_start, dtype=float)
    max_step = float(np.max(np.abs(delta))) if delta.size else 0.0
    count = max(1, int(math.ceil(max_step / float(step_rad))))
    return np.linspace(
        np.asarray(q_start, dtype=float),
        np.asarray(q_end, dtype=float),
        count + 1,
    )


def _stationary_trajectory(q: Sequence[float], t0: float, dt: float) -> Trajectory:
    """零运动轨迹（单采样点，冻结类型直接构造，不动任何既有类型）。"""
    sample = TrajectorySample(seq=0, t_abs=float(t0), q=tuple(float(v) for v in q))
    return Trajectory(samples=(sample,), dt=float(dt))


def _hold_trajectory(
    q: Sequence[float],
    *,
    duration_s: float,
    t0: float,
    dt: float,
    action: GripperAction,
) -> Trajectory:
    """保持段轨迹（常量关节角 + 逐点挂上夹爪动作）。

    夹爪动作通过 :func:`dataclasses.replace` 挂到**每一个**采样点的
    ``gripper_action`` 字段上——这是冻结类型里本来就预留的字段
    （:meth:`Trajectory._validate` 只做 isinstance 检查），
    因此**不需要**改动 :class:`Trajectory` / :class:`TrajectorySample` /
    :class:`GripperAction` / :mod:`rm75_drake.recording`。
    """
    step = float(dt)
    count = max(1, int(round(float(duration_s) / step)))
    samples = []
    for index in range(count + 1):
        sample = TrajectorySample(
            seq=index,
            t_abs=float(t0) + index * step,
            q=tuple(float(v) for v in q),
        )
        samples.append(dataclasses.replace(sample, gripper_action=action))
    return Trajectory(
        samples=tuple(samples),
        dt=step,
        gripper_schedule=(action,),
    )


def _retime_reason(exc: Exception) -> str:
    """从 :class:`rm75_drake.planning.retime.RetimeError` 抽出稳定 reason 词。"""
    text = str(exc)
    start = text.find("[")
    if start >= 0:
        end = text.find("]", start)
        if end > start:
            tag = text[start + 1 : end].strip()
            if tag:
                return tag
    return "parameterization"


# ── 容差解析 ────────────────────────────────────────────────────────────
def _resolve_tolerances(
    primitive, task: TaskSpec, options: SolverOptions
) -> Tuple[float, float]:
    """按「原语 → 任务 → 默认」三级解析误差门槛（m / rad）。"""
    position = options.position_tolerance_m
    orientation = options.orientation_tolerance_rad
    if primitive.tolerance is not None:
        if position is None:
            position = float(primitive.tolerance.position_m)
        if orientation is None:
            orientation = float(primitive.tolerance.orientation_rad)
    if position is None:
        raw = task.thresholds.get("position_m") if task.thresholds else None
        position = float(raw) if raw is not None else DEFAULT_POSITION_TOLERANCE_M
    if orientation is None:
        raw = task.thresholds.get("orientation_rad") if task.thresholds else None
        orientation = (
            float(raw) if raw is not None else DEFAULT_ORIENTATION_TOLERANCE_RAD
        )
    return float(position), float(orientation)


# ── 单原语求解 ──────────────────────────────────────────────────────────
def _resolve_target(primitive, task: TaskSpec) -> Tuple[np.ndarray, np.ndarray]:
    """解析原语的目标位姿（显式 ``target`` 或 ``target_object`` 的物体位姿）。"""
    if primitive.target is not None:
        return (
            np.asarray(primitive.target.xyz, dtype=float),
            np.asarray(primitive.target.rpy, dtype=float),
        )
    if primitive.target_object:
        for obj in task.objects:
            if obj.name == primitive.target_object:
                return (
                    np.asarray(obj.pose.xyz, dtype=float),
                    np.asarray(obj.pose.rpy, dtype=float),
                )
        raise _PrimitiveFailure(
            "schema",
            "unknown_object",
            f"原语 {primitive.name!r} 引用了未声明的物体 {primitive.target_object!r}；"
            f"已声明的物体有 {[o.name for o in task.objects]}",
        )
    raise _PrimitiveFailure(
        "schema",
        "missing_target",
        f"原语 {primitive.name!r} 既没有 target，也没有 target_object，无法确定目标位姿",
    )


def _solve_motion(
    primitive,
    task: TaskSpec,
    *,
    spec: PrimitiveSpec,
    scene: Any,
    options: SolverOptions,
    state: SolveState,
) -> PrimitiveResult:
    """运动类原语：**解析 IK → 关节插值 → 碰撞复检 → 重定时**。"""
    tool_offset = (
        float(mdh.MDH_D[6]) if options.tool_offset is None else float(options.tool_offset)
    )
    target_xyz, target_rpy = _resolve_target(primitive, task)
    pose = _pose_matrix(target_xyz, target_rpy)

    ik = analytic_ik.solve_analytic_ik(
        pose,
        seed=state.current_q,
        branch_policy="nearest",
        limits=analytic_ik.software_limits(),
        tool_offset=tool_offset,
    )
    if not ik.success or ik.q is None:
        raise _PrimitiveFailure(
            "ik",
            ik.reason,
            f"原语 {primitive.name!r} 的解析 IK 失败：{ik.reason}（{ik.message}）",
        )

    q_target = np.asarray(ik.q, dtype=float)
    if float(np.max(np.abs(q_target - state.current_q))) < 1e-12:
        # 已经到位：零运动（重定时拒绝零长度路径，故直接给出静止轨迹）。
        tcp_position, tcp_rotation = _tcp_pose(q_target, tool_offset)
        position_error = float(np.linalg.norm(tcp_position - target_xyz))
        orientation_error = _angle_between(tcp_rotation, _rpy_to_rotation(target_rpy))
        trajectory = _stationary_trajectory(q_target, state.t, float(options.dt))
        state.current_q = q_target
        return PrimitiveResult(
            name=primitive.name,
            goal_id=primitive.goal_id,
            success=True,
            stage=STAGE_DONE,
            reason=REASON_COMPLETED,
            trajectory=trajectory,
            position_error_m=position_error,
            orientation_error_rad=orientation_error,
            duration_s=0.0,
            command_issued=False,
            message="已在目标位形（零运动）",
        )
    try:
        q_seq = _interpolate(state.current_q, q_target, options.joint_step_rad)
    except (ValueError, FloatingPointError) as exc:
        raise _PrimitiveFailure(
            "path",
            "interpolation_failed",
            f"原语 {primitive.name!r} 的关节插值失败：{exc}",
        ) from exc

    if scene is None:
        raise _PrimitiveFailure(
            "schema",
            "missing_scene",
            f"原语 {primitive.name!r} 需要场景做碰撞复检，但没有传入 scene",
        )

    try:
        recheck = _gcs.collision_recheck(scene, q_seq)
    except _gcs.GcsError as exc:
        raise _PrimitiveFailure(
            "collision",
            "recheck_failed",
            f"原语 {primitive.name!r} 的碰撞复检失败：{exc}",
        ) from exc
    if recheck.num_escapes != 0:
        raise _PrimitiveFailure(
            "collision",
            "escape",
            f"原语 {primitive.name!r} 的碰撞复检出现 {recheck.num_escapes} 个逃逸位形"
            f"（首个逃逸索引 {recheck.escape_indices[0] if recheck.escape_indices else -1}）",
        )

    try:
        retimed = _retime.retime(
            q_seq,
            dt=float(options.dt),
            t0=float(state.t),
            robot_name=options.robot_name,
            gripper_schedule=(),
        )
    except (_retime.RetimeError, TrajectoryError) as exc:
        raise _PrimitiveFailure(
            "retime",
            _retime_reason(exc),
            f"原语 {primitive.name!r} 的重定时失败：{exc}",
        ) from exc

    trajectory = retimed.trajectory
    final_q = trajectory.samples[-1].q
    tcp_position, tcp_rotation = _tcp_pose(final_q, tool_offset)
    position_error = float(np.linalg.norm(tcp_position - target_xyz))
    orientation_error = _angle_between(tcp_rotation, _rpy_to_rotation(target_rpy))

    position_tol, orientation_tol = _resolve_tolerances(primitive, task, options)
    if position_error > position_tol or orientation_error > orientation_tol:
        raise _PrimitiveFailure(
            "path",
            "tolerance_exceeded",
            f"原语 {primitive.name!r} 的末端误差超门槛：位置 {position_error!r} m"
            f"（门槛 {position_tol!r} m）、姿态 {orientation_error!r} rad"
            f"（门槛 {orientation_tol!r} rad）",
        )

    state.current_q = q_target
    state.t = trajectory.end_time
    return PrimitiveResult(
        name=primitive.name,
        goal_id=primitive.goal_id,
        success=True,
        stage=STAGE_DONE,
        reason=REASON_COMPLETED,
        trajectory=trajectory,
        position_error_m=position_error,
        orientation_error_rad=orientation_error,
        duration_s=float(trajectory.duration),
        command_issued=False,
        message="",
    )


def _solve_gripper(
    primitive,
    *,
    spec: PrimitiveSpec,
    transport: Any,
    options: SolverOptions,
    state: SolveState,
) -> PrimitiveResult:
    """夹爪类原语：生成夹爪动作 → 经下发通道下发 → 挂到保持段采样点上。"""
    action_kind = spec.gripper_action_kind
    if action_kind is None:
        raise _PrimitiveFailure(
            "gripper",
            "unsupported_action",
            f"原语 {primitive.name!r} 没有定义夹爪动作种类",
        )
    hold = (
        float(primitive.hold_seconds)
        if primitive.hold_seconds is not None
        else float(options.default_hold_seconds)
    )
    if action_kind == "close":
        action = close_action(hold_seconds=hold)
    elif action_kind == "open":
        action = open_action(hold_seconds=hold)
    else:
        raise _PrimitiveFailure(
            "gripper",
            "unsupported_action",
            f"原语 {primitive.name!r} 的夹爪动作 {action_kind!r} 不受支持",
        )

    command_issued = False
    message = ""
    if transport is None:
        message = "没有传输层：夹爪命令**未下发**（本次结果未在真机/替身上验证）"
    else:
        try:
            transport.command_gripper_position(action.position, speed=action.speed)
            command_issued = True
        except Exception as exc:  # 传输层异常原样冒泡为稳定 reason
            raise _PrimitiveFailure(
                "gripper",
                "transport_error",
                f"原语 {primitive.name!r} 的夹爪下发失败：{type(exc).__name__}: {exc}",
            ) from exc

    trajectory = _hold_trajectory(
        state.current_q,
        duration_s=hold,
        t0=state.t,
        dt=float(options.dt),
        action=action,
    )
    state.t = trajectory.end_time
    return PrimitiveResult(
        name=primitive.name,
        goal_id=primitive.goal_id,
        success=True,
        stage=STAGE_DONE,
        reason=REASON_COMPLETED,
        trajectory=trajectory,
        position_error_m=None,
        orientation_error_rad=None,
        duration_s=float(trajectory.duration),
        command_issued=command_issued,
        message=message,
    )


def _solve_verify(
    primitive,
    *,
    spec: PrimitiveSpec,
    transport: Any,
    options: SolverOptions,
    state: SolveState,
) -> PrimitiveResult:
    """``verify`` 原语：回读夹爪状态，判断是否到位。"""
    if transport is None:
        raise _PrimitiveFailure(
            "verify",
            "no_transport",
            f"原语 {primitive.name!r} 需要回读夹爪状态，但没有传入 transport",
        )
    try:
        gripper_state = transport.read_gripper()
    except Exception as exc:
        raise _PrimitiveFailure(
            "verify",
            "transport_error",
            f"原语 {primitive.name!r} 读取夹爪状态失败：{type(exc).__name__}: {exc}",
        ) from exc

    if not bool(getattr(gripper_state, "online", False)):
        raise _PrimitiveFailure(
            "verify",
            "gripper_offline",
            f"原语 {primitive.name!r} 读回夹爪离线（物体掉落 / 反馈丢失）",
        )

    expected = options.verify_expected_position
    if expected is None:
        action_kind = primitive.action
        expected = 0 if action_kind != "open" else 1000
    tolerance = int(options.verify_position_tolerance)
    if abs(int(gripper_state.position) - int(expected)) > tolerance:
        raise _PrimitiveFailure(
            "verify",
            "gripper_not_closed",
            f"原语 {primitive.name!r} 期望夹爪位置 {expected}（±{tolerance}），"
            f"实测 {int(gripper_state.position)}",
        )
    return PrimitiveResult(
        name=primitive.name,
        goal_id=primitive.goal_id,
        success=True,
        stage=STAGE_DONE,
        reason=REASON_COMPLETED,
        trajectory=None,
        position_error_m=None,
        orientation_error_rad=None,
        duration_s=0.0,
        command_issued=False,
        message=f"夹爪在线且位置到位（{int(gripper_state.position)}）",
    )


def solve_primitive(
    primitive,
    *,
    task: TaskSpec,
    scene: Any = None,
    transport: Any = None,
    options: Optional[SolverOptions] = None,
    state: Optional[SolveState] = None,
) -> PrimitiveResult:
    """求解**单个**原语（:func:`solve_task` 的构件；状态机可复用）。

    参数
    ----
    primitive:
        任务文件里的原语条目（:class:`rm75_drake.tasks.schema.PrimitiveSpec`）。
    task:
        所属任务（用于解析物体 / 阈值）。
    scene:
        场景（提供碰撞检查器）；运动类原语**必须**提供。
    transport:
        传输层（夹爪下发 / 状态回读）；``None`` ⇒ 夹爪命令**不下发**
        （结果标记 ``command_issued=False``，即未验证）。
    options:
        求解参数；``None`` → 默认值。
    state:
        求解游标（当前关节角 / 时间轴）；``None`` → 从
        ``options.initial_q`` 新建。

    返回
    ----
    :class:`PrimitiveResult`（失败时 ``success=False``，并带 stage / reason）。
    """
    opts = options or SolverOptions()
    cursor = state if state is not None else SolveState(
        current_q=np.asarray(opts.initial_q, dtype=float)
    )
    try:
        spec = primitive_spec(primitive.name)
    except TaskError as exc:
        return PrimitiveResult(
            name=str(getattr(primitive, "name", "?")),
            goal_id=str(getattr(primitive, "goal_id", "?")),
            success=False,
            stage="schema",
            reason="unknown_primitive",
            message=str(exc),
        )

    try:
        if spec.kind == "motion":
            return _solve_motion(
                primitive, task, spec=spec, scene=scene, options=opts, state=cursor
            )
        if spec.kind == "gripper":
            return _solve_gripper(
                primitive, spec=spec, transport=transport, options=opts, state=cursor
            )
        if spec.kind == "verify":
            return _solve_verify(
                primitive, spec=spec, transport=transport, options=opts, state=cursor
            )
        raise _PrimitiveFailure(
            "schema",
            "unsupported_kind",
            f"原语 {primitive.name!r} 的种类 {spec.kind!r} 不在单臂求解器的实现范围内"
            "（双臂原语由 S06-T05 的协调规划实现）",
        )
    except _PrimitiveFailure as failure:
        return PrimitiveResult(
            name=primitive.name,
            goal_id=primitive.goal_id,
            success=False,
            stage=failure.stage,
            reason=failure.reason,
            message=failure.message,
        )


def solve_task(
    task: TaskSpec,
    *,
    scene: Any = None,
    transport: Any = None,
    options: Optional[SolverOptions] = None,
) -> TaskResult:
    """**单一求解器入口**：按原语顺序求解整个任务。

    参数
    ----
    task:
        任务（:class:`rm75_drake.tasks.schema.TaskSpec`）。
    scene:
        场景（碰撞复检用）；运动类原语必须提供，否则该原语以
        ``stage="schema" / reason="missing_scene"`` 失败。
    transport:
        传输层（夹爪下发 / 状态回读）；``None`` ⇒ 夹爪命令不下发。
    options:
        求解参数；``None`` → 默认值。

    返回
    ----
    :class:`TaskResult`：逐原语结果 + 任务级汇总。
    **首个失败即停止**（后续原语不执行），失败原语的 stage / reason
    透传到任务级字段。
    """
    if not isinstance(task, TaskSpec):
        raise TaskError(f"task 必须是 TaskSpec 实例，收到 {type(task).__name__}")
    opts = options or SolverOptions()
    cursor = SolveState(current_q=np.asarray(opts.initial_q, dtype=float))
    results: List[PrimitiveResult] = []
    for primitive in task.primitives:
        result = solve_primitive(
            primitive,
            task=task,
            scene=scene,
            transport=transport,
            options=opts,
            state=cursor,
        )
        results.append(result)
        if not result.success:
            return TaskResult(
                task_name=task.name,
                success=False,
                stage=result.stage,
                reason=result.reason,
                results=tuple(results),
                duration_s=sum(item.duration_s for item in results),
                message=result.message,
            )
    return TaskResult(
        task_name=task.name,
        success=True,
        stage=STAGE_DONE,
        reason=REASON_COMPLETED,
        results=tuple(results),
        duration_s=sum(item.duration_s for item in results),
        message=f"全部 {len(results)} 个原语求解成功",
    )
