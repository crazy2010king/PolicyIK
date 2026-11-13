"""逆运动学模块（S02-T03）：任务空间目标 → 关节位形，含**可解释失败**。

职责边界
--------
本模块是 S02 规划链路的**第一环**：把「末端目标位姿」解成「7 个关节角」。
下游（IRIS / 空间 GCS / Toppra）全部消费这里产出的关节位形，因此本模块的
硬性要求是：

1. **失败必须可解释**：不返回"看起来能用但其实是坏解"的位形，而是给出
   *失败类别*（:data:`FAILURE_REASONS`）+ *可读消息*（点名越界关节/数值）；
2. **软限位复核**：收敛解必须逐轴落在**软件软限位**内（每侧比硬件再收 4°，
   见 :mod:`rm75_drake.limits`），越界一律判失败；
3. **不得调用厂商解算接口**：本模块只依赖 Drake 的雅可比 / 正运动学
   （``scripts/scan_forbidden_apis.py`` 必须 0 命中）。

名义姿态（nominal_posture）为什么不能是全零
--------------------------------------------
全零位姿在 RM75 上会让 **6 轴机构退化**：多个关节轴共线
（各轴 ``a_i = 0``，见 :mod:`rm75_drake.kinematics.mdh`），雅可比在零位
接近奇异，阻尼最小二乘会给出巨大的关节步长并被限位裁剪，结果既慢又不可复现。
因此本模块固定使用一组"**抬肘**"名义姿态常量 :data:`NOMINAL_POSTURE`：

    joint_1..joint_7 = [0, -0.6, 0, 0.9, 0, 0.5, 0]   (rad)

它把肘部抬离奇异位形，同时整体落在软件软限位内（逐轴复核见
:func:`rm75_drake.limits.check_joint_limits` 的同一真值源）。
调用方可以用 ``nominal_posture=`` 覆盖，但**不允许传全零**（会被显式拒绝，
见 :func:`_validate_posture`），因为全零必然落回退化位形。

失败类别（:data:`FAILURE_REASONS`）
----------------------------------
=========================  ==============================================
``unreachable``            目标不可达：迭代用尽仍未收敛（位置/姿态残差超标）
``timeout``                超时：单次求解耗时超过 ``timeout``（秒）
``seed_out_of_limits``     起点（种子位形）软限位越界，点名越界关节与数值
``soft_limit_violation``   收敛解复核未通过（越界关节名 + 数值）
=========================  ==============================================

另外，**目标位姿数值非法**（含 NaN/Inf 或不是位姿对象）**抛异常**
:class:`IkError`（``reason="invalid_target"``），而不是返回失败结果——
调用方传进来的坏数据必须立刻炸出来，不能伪装成"解不出来"。

关于求解器的实现选型（**与切片计划的一处偏差，已实测钉死**）
------------------------------------------------------------
切片计划原本要求"用 ``pydrake.multibody.inverse_kinematics.
DifferentialInverseKinematicsParameters``（走 ``drake_compat.diff_ik_parameters`` /
``diff_ik_succeeded``）"。实测（Drake **1.51.1** 本机 fallback 档 + **1.55.0**
正式基线 wheel 解包核对）：

* 两个版本的 ``inverse_kinematics`` 模块**都**提供
  ``DifferentialInverseKinematicsParameters``（参数类），
  但**都没有**消费它的旧版求解器类 ``DifferentialInverseKinematics``
  （1.55.0 的 ``inverse_kinematics.so`` 符号表里只有
  ``DifferentialInverseKinematicsSystem`` / ``...Integrator`` /
  ``...Controller`` / ``...Parameters`` / ``...Result`` / ``...Status``）；
* 也就是说"参数类"在，但"能单独吃这个参数类的同步求解器"已经下线，
  计划里那条调用路径**在两个受支持 Drake 版本上都不存在**。

因此本模块改为**自带的阻尼最小二乘（DLS / Levenberg-Marquardt）**：用 Drake 的
雅可比（:func:`rm75_drake.drake_compat.jacobian_6d`）+ 正运动学
（``CalcRelativeTransform``）迭代求解，参数语义与
``DifferentialInverseKinematicsParameters`` 对齐（关节位置限位取软件软限位、
名义关节位置取 ``nominal_posture``、相对位置/姿态容差取 ``position_tol`` /
``orientation_tol``）。这样：

* 容差、迭代上限、超时三类预算**由本模块自己掌控并可观测**（迭代数、
  残差都进 :class:`IkResult`），比旧版 DiffIK 的黑盒更好解释；
* 不引入任何 Drake 版本分支，也不依赖已下线的类。

失败分类与语义不受影响：仍然严格区分"不可达 / 超时 / 起点越界 / 目标非法"。
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any, Iterable, List, Optional, Sequence, Tuple

import numpy as np

from .. import drake_compat, limits as _limits

__all__ = [
    "IkError",
    "IkResult",
    "NOMINAL_POSTURE",
    "EE_FRAME_NAME",
    "FAILURE_REASONS",
    "REASON_NONE",
    "REASON_UNREACHABLE",
    "REASON_TIMEOUT",
    "REASON_SEED_OUT_OF_LIMITS",
    "REASON_SOFT_LIMIT_VIOLATION",
    "REASON_INVALID_TARGET",
    "software_joint_limits",
    "solve_ik",
    "batch_solve_ik",
    "target_pose_from_position",
]


#: 末端执行器 frame 名（URDF 里的 ``link_7``）。
EE_FRAME_NAME = "link_7"

#: **抬肘**名义姿态（rad），顺序 joint_1..joint_7。
#:
#: 为什么不是全零：零位下 6 轴机构退化（轴线共线），雅可比接近奇异；
#: 详见模块 docstring。
NOMINAL_POSTURE: Tuple[float, ...] = (0.0, -0.6, 0.0, 0.9, 0.0, 0.5, 0.0)

#: RM75 的关节数。
JOINT_COUNT = 7

# ── 失败类别 ────────────────────────────────────────────────────────────
REASON_NONE = ""
REASON_UNREACHABLE = "unreachable"
REASON_TIMEOUT = "timeout"
REASON_SEED_OUT_OF_LIMITS = "seed_out_of_limits"
REASON_SOFT_LIMIT_VIOLATION = "soft_limit_violation"
REASON_INVALID_TARGET = "invalid_target"

#: 全部失败类别的稳定取值（供调用方按标签分类，不依赖自然语言措辞）。
FAILURE_REASONS: Tuple[str, ...] = (
    REASON_UNREACHABLE,
    REASON_TIMEOUT,
    REASON_SEED_OUT_OF_LIMITS,
    REASON_SOFT_LIMIT_VIOLATION,
    REASON_INVALID_TARGET,
)

#: 数值比较容差（限位复核用，rad）。
_LIMIT_TOL = 1e-9

#: Levenberg-Marquardt 的单次迭代内最大重试次数（每次重试加大阻尼）。
_MAX_LM_ATTEMPTS = 8

#: LM 阻尼系数下界（步长足够"敢"时用）。
_MIN_DAMPING = 1e-4

#: LM 阻尼系数上界（步长必须保守时用）。
_MAX_DAMPING = 10.0


class IkError(RuntimeError):
    """IK 输入非法（目标位姿数值非法、参数越界、场景缺 frame 等）。

    属性
    ----
    reason:
        稳定标签（见 :data:`FAILURE_REASONS`），默认 ``"invalid_input"``。
    """

    def __init__(self, message: str, *, reason: str = "invalid_input") -> None:
        super().__init__(message)
        self.reason = reason


@dataclass(frozen=True)
class IkResult:
    """一次 IK 求解的结果。

    字段
    ----
    q:
        关节角（rad），长度 7，顺序 joint_1..joint_7。**失败时也有值**
        （种子位形或最后一次迭代的位形），便于调用方复现/诊断。
    success:
        是否收敛到满足容差的解。
    iterations:
        实际执行的迭代次数。
    message:
        **始终非空**的可读说明（成功时给出残差，失败时给出原因与关键数值）。
    reason:
        失败类别（:data:`FAILURE_REASONS`）；成功时为空串。
    position_error:
        位置残差（m）。
    orientation_error:
        姿态残差（rad）。
    """

    q: Tuple[float, ...]
    success: bool
    iterations: int
    message: str
    reason: str = REASON_NONE
    position_error: float = float("nan")
    orientation_error: float = float("nan")

    @property
    def q_array(self) -> np.ndarray:
        """关节角的 numpy 视图（长度 7）。"""
        return np.asarray(self.q, dtype=float)


# ── 软限位（真值来自 limits.py，不在本模块复制）───────────────────────
def software_joint_limits() -> Tuple[Tuple[str, ...], np.ndarray, np.ndarray]:
    """返回软件软限位 ``(关节名, 下限数组, 上限数组)``。

    真值来自 ``models/rm75_drake/joint_limits.yaml`` 的 ``software`` 段
    （硬件限位每侧再收 4°），**不在本模块复制常量**。
    """
    data = _limits.load_joint_limits()
    joints = data.get("joints")
    if not isinstance(joints, list) or not joints:
        raise IkError("限位交接件缺少 joints 段", reason="invalid_limits")
    names: List[str] = []
    lower: List[float] = []
    upper: List[float] = []
    for joint in joints:
        software = joint.get("software")
        if not isinstance(software, dict):
            raise IkError(
                f"限位交接件缺少 {joint.get('name')} 的 software 段",
                reason="invalid_limits",
            )
        names.append(str(joint["name"]))
        lower.append(float(software["position_lower_rad"]))
        upper.append(float(software["position_upper_rad"]))
    if len(names) != JOINT_COUNT:
        raise IkError(
            f"软件限位必须是 {JOINT_COUNT} 维，实际 {len(names)}", reason="invalid_limits"
        )
    return tuple(names), np.asarray(lower, dtype=float), np.asarray(upper, dtype=float)


def _violations(
    q: np.ndarray, names: Sequence[str], lower: np.ndarray, upper: np.ndarray
) -> List[str]:
    """返回逐轴越界描述（``"joint_2=3.5 超出 [-2.199, 2.199] rad"``）。"""
    problems: List[str] = []
    for index, name in enumerate(names):
        value = float(q[index])
        low = float(lower[index])
        high = float(upper[index])
        if value < low - _LIMIT_TOL or value > high + _LIMIT_TOL:
            problems.append(
                f"{name}={value:.6f} 超出软件软限位 [{low:.6f}, {high:.6f}] rad"
            )
    return problems


def _validate_posture(posture: Sequence[float], label: str) -> np.ndarray:
    """校验一组关节角（长度 7、有限），返回 numpy 数组。"""
    arr = np.asarray(posture, dtype=float).reshape(-1)
    if arr.size != JOINT_COUNT:
        raise IkError(
            f"{label} 必须是 {JOINT_COUNT} 个关节角，收到 {arr.size} 个",
            reason="invalid_posture",
        )
    if not np.all(np.isfinite(arr)):
        raise IkError(f"{label} 含 NaN/Inf：{posture!r}", reason="invalid_posture")
    return arr


def _rotation_vector(rotation) -> np.ndarray:
    """把旋转矩阵转成**旋转向量**（轴 × 角，rad）。

    用旋转向量而不是欧拉角：欧拉角在 ``±π`` 附近不连续，会让 DLS 在
    "翻转"方向上抖动。``angle=0`` 时返回零向量。
    """
    matrix = np.asarray(rotation, dtype=float).reshape(3, 3)
    cos_angle = (np.trace(matrix) - 1.0) / 2.0
    cos_angle = float(np.clip(cos_angle, -1.0, 1.0))
    angle = float(np.arccos(cos_angle))
    if angle < 1e-12:
        return np.zeros(3)
    axis = np.array(
        [
            matrix[2, 1] - matrix[1, 2],
            matrix[0, 2] - matrix[2, 0],
            matrix[1, 0] - matrix[0, 1],
        ],
        dtype=float,
    ) / (2.0 * np.sin(angle))
    return axis * angle


def _target_arrays(target_pose: Any) -> Tuple[np.ndarray, np.ndarray]:
    """把目标位姿拆成 ``(位置数组, 旋转矩阵)``，并校验有限性。

    非有限值 → 抛 :class:`IkError`（``reason="invalid_target"``）。
    """
    if target_pose is None:
        raise IkError("目标位姿不能是 None", reason=REASON_INVALID_TARGET)
    try:
        translation = np.asarray(target_pose.translation(), dtype=float).reshape(3)
        rotation = np.asarray(target_pose.rotation().matrix(), dtype=float).reshape(3, 3)
    except AttributeError as exc:
        raise IkError(
            f"目标位姿必须是 RigidTransform（提供 translation()/rotation()），"
            f"收到 {type(target_pose).__name__}",
            reason=REASON_INVALID_TARGET,
        ) from exc
    if not np.all(np.isfinite(translation)):
        raise IkError(
            f"目标位姿位置含非有限值：{translation.tolist()}",
            reason=REASON_INVALID_TARGET,
        )
    if not np.all(np.isfinite(rotation)):
        raise IkError(
            "目标位姿旋转矩阵含非有限值",
            reason=REASON_INVALID_TARGET,
        )
    return translation, rotation


def target_pose_from_position(
    position: Sequence[float], rpy: Sequence[float] = (0.0, 0.0, 0.0)
):
    """便捷构造：位置（m）+ 内旋 rpy（rad）→ ``RigidTransform``。"""
    from pydrake.math import RigidTransform

    xyz = np.asarray(position, dtype=float).reshape(3)
    if not np.all(np.isfinite(xyz)):
        raise IkError(
            f"目标位置含非有限值：{xyz.tolist()}", reason=REASON_INVALID_TARGET
        )
    return RigidTransform(drake_compat.rotation_from_rpy(rpy), xyz)


def solve_ik(
    scene,
    target_pose,
    *,
    nominal_posture: Optional[Sequence[float]] = None,
    position_tol: float = 1e-4,
    orientation_tol: float = 1e-3,
    timeout: float = 0.05,
    max_iterations: int = 100,
    initial_posture: Optional[Sequence[float]] = None,
    frame_name: str = EE_FRAME_NAME,
    damping: float = 0.1,
    nullspace_gain: float = 0.02,
    max_step: float = 0.25,
) -> IkResult:
    """解一次逆运动学。

    参数
    ----
    scene:
        :class:`rm75_drake.planning.scene.SceneBundle`（S02-T01 装配产物）。
    target_pose:
        目标位姿（``RigidTransform``），在**世界系**下表达（末端 frame）。
    nominal_posture:
        名义姿态（rad，7 维）。``None`` → 用 :data:`NOMINAL_POSTURE`。
        **不接受全零**（零位下 6 轴退化，见模块 docstring）。
    position_tol:
        位置收敛容差（m）。
    orientation_tol:
        姿态收敛容差（rad）。
    timeout:
        单次求解的时间预算（s）。超时 → ``reason="timeout"``。
    max_iterations:
        最大迭代次数。迭代用尽仍未收敛 → ``reason="unreachable"``。
    initial_posture:
        显式种子位形（覆盖 ``nominal_posture`` 作为迭代起点）。
        种子越界 → ``reason="seed_out_of_limits"``。
    frame_name:
        末端 frame 名（默认 ``link_7``）。
    damping:
        **初始** LM 阻尼系数（``J Jᵀ + λ²I``），抑制奇异位形下的巨步长；
        迭代中自适应调整（变好则减小、变差则增大）。
    nullspace_gain:
        零空间回中增益（把冗余自由度拉向名义姿态）。
    max_step:
        单步关节位移上限（rad），防止一步跳到奇异/限位另一侧。

    返回
    ----
    :class:`IkResult`

    异常
    ----
    IkError
        目标位姿数值非法（含 NaN/Inf、类型不对）、参数非法、场景缺 frame。
    """
    if not isinstance(position_tol, (int, float)) or position_tol <= 0.0:
        raise IkError(f"position_tol 必须为正，收到 {position_tol!r}")
    if not isinstance(orientation_tol, (int, float)) or orientation_tol <= 0.0:
        raise IkError(f"orientation_tol 必须为正，收到 {orientation_tol!r}")
    if not isinstance(timeout, (int, float)) or timeout <= 0.0:
        raise IkError(f"timeout 必须为正（秒），收到 {timeout!r}")
    if not isinstance(max_iterations, int) or max_iterations <= 0:
        raise IkError(f"max_iterations 必须是正整数，收到 {max_iterations!r}")
    if not isinstance(damping, (int, float)) or damping <= 0.0:
        raise IkError(f"damping 必须为正，收到 {damping!r}")

    target_position, target_rotation = _target_arrays(target_pose)

    posture_source = (
        initial_posture
        if initial_posture is not None
        else (nominal_posture if nominal_posture is not None else NOMINAL_POSTURE)
    )
    seed = _validate_posture(posture_source, "种子位形")
    nominal = _validate_posture(
        nominal_posture if nominal_posture is not None else NOMINAL_POSTURE, "名义姿态"
    )
    if not np.any(np.abs(nominal) > 0.0):
        raise IkError(
            "名义姿态不得为全零：零位下 RM75 的 6 轴机构退化（轴线共线），"
            "雅可比接近奇异，求解既慢又不可复现。请改用抬肘名义姿态。",
            reason="invalid_posture",
        )

    names, lower, upper = software_joint_limits()

    seed_problems = _violations(seed, names, lower, upper)
    if seed_problems:
        return IkResult(
            q=tuple(float(v) for v in seed),
            success=False,
            iterations=0,
            message="起点软限位越界：" + "；".join(seed_problems),
            reason=REASON_SEED_OUT_OF_LIMITS,
        )

    plant = scene.plant
    frame = plant.GetFrameByName(frame_name)
    world = plant.world_frame()
    # 复用场景的上下文求解，但**求解后把关节位形还原**（IK 是纯函数式查询，
    # 不应把场景的当前状态改掉）。
    context = scene.plant_context
    original_positions = plant.GetPositions(context).copy()
    try:
        return _solve_ik_impl(
            plant=plant,
            frame=frame,
            world=world,
            context=context,
            seed=seed,
            nominal=nominal,
            names=names,
            lower=lower,
            upper=upper,
            target_position=target_position,
            target_rotation=target_rotation,
            position_tol=float(position_tol),
            orientation_tol=float(orientation_tol),
            timeout=float(timeout),
            max_iterations=int(max_iterations),
            damping=float(damping),
            nullspace_gain=float(nullspace_gain),
            max_step=float(max_step),
        )
    finally:
        plant.SetPositions(context, original_positions)


def _solve_ik_impl(
    *,
    plant,
    frame,
    world,
    context,
    seed,
    nominal,
    names,
    lower,
    upper,
    target_position,
    target_rotation,
    position_tol,
    orientation_tol,
    timeout,
    max_iterations,
    damping,
    nullspace_gain,
    max_step,
) -> IkResult:
    """DLS 迭代主体（见 :func:`solve_ik` 的参数说明）。"""
    q = seed.astype(float).copy()
    plant.SetPositions(context, q)

    started = time.perf_counter()
    converged = False
    timed_out = False
    stalled = False
    iterations = 0
    position_error = float("nan")
    orientation_error = float("nan")

    def evaluate(joint_positions):
        """返回 ``(位置残差, 姿态残差, 任务误差向量, 度量)``。

        度量把位置/姿态残差按各自容差归一化后相加，用于**比较两次迭代的
        好坏**（否则 m 与 rad 直接相加没有意义）。
        """
        plant.SetPositions(context, joint_positions)
        current = plant.CalcRelativeTransform(context, world, frame)
        current_position = np.asarray(current.translation(), dtype=float)
        current_rotation = np.asarray(current.rotation().matrix(), dtype=float)
        rot_error = _rotation_vector(target_rotation @ current_rotation.T)
        pos_error = target_position - current_position
        pos_norm = float(np.linalg.norm(pos_error))
        rot_norm = float(np.linalg.norm(rot_error))
        metric = pos_norm / position_tol + rot_norm / orientation_tol
        return pos_norm, rot_norm, np.concatenate((rot_error, pos_error)), metric

    position_error, orientation_error, task_error, metric = evaluate(q)

    for iteration in range(1, max_iterations + 1):
        if time.perf_counter() - started > timeout:
            timed_out = True
            break
        iterations = iteration

        if position_error <= position_tol and orientation_error <= orientation_tol:
            converged = True
            break

        jacobian = drake_compat.jacobian_6d(plant, context, frame, world)

        # Levenberg-Marquardt：步长变好就减小阻尼，变差就增大阻尼重试。
        improved = False
        for _ in range(_MAX_LM_ATTEMPTS):
            jjt = jacobian @ jacobian.T + (damping ** 2) * np.eye(6)
            try:
                dq_task = jacobian.T @ np.linalg.solve(jjt, task_error)
            except np.linalg.LinAlgError:  # pragma: no cover - 阻尼后基本不会奇异
                dq_task = jacobian.T @ np.linalg.lstsq(jjt, task_error, rcond=None)[0]
            if nullspace_gain > 0.0:
                # 零空间回中（把冗余自由度拉向名义姿态）
                projector = np.eye(JOINT_COUNT) - jacobian.T @ np.linalg.solve(
                    jjt, jacobian
                )
                dq = dq_task + nullspace_gain * (projector @ (nominal - q))
            else:
                dq = dq_task

            norm = float(np.linalg.norm(dq))
            if norm > max_step:
                dq = dq * (max_step / norm)
            candidate = np.clip(q + dq, lower, upper)

            new_position_error, new_orientation_error, new_task_error, new_metric = (
                evaluate(candidate)
            )
            if new_metric < metric:
                q = candidate
                position_error = new_position_error
                orientation_error = new_orientation_error
                task_error = new_task_error
                metric = new_metric
                damping = max(damping / 3.0, _MIN_DAMPING)
                improved = True
                break
            # 没变好：加大阻尼再试（更保守的步长）。
            damping = min(damping * 3.0, _MAX_DAMPING)
        if not improved:
            # 停滞：既到不了容差，也找不到能降低残差的步长 → 局部最优/不可达。
            stalled = True
            break

    if not converged:
        # 最后一次迭代的残差（可能因为超时而未更新，这里补算一次）。
        position_error, orientation_error, _, _ = evaluate(q)

    if not converged:
        if timed_out:
            reason = REASON_TIMEOUT
            message = (
                f"IK 超时（{timeout} s 内未收敛）：迭代 {iterations} 次，"
                f"位置残差 {position_error:.6f} m，姿态残差 {orientation_error:.6f} rad"
            )
        else:
            reason = REASON_UNREACHABLE
            detail = "迭代停滞于局部最优" if stalled else f"{max_iterations} 次迭代后仍未收敛"
            message = (
                f"目标不可达：{detail}，"
                f"位置残差 {position_error:.6f} m（容差 {position_tol}），"
                f"姿态残差 {orientation_error:.6f} rad（容差 {orientation_tol}）"
            )
        return IkResult(
            q=tuple(float(v) for v in q),
            success=False,
            iterations=iterations,
            message=message,
            reason=reason,
            position_error=position_error,
            orientation_error=orientation_error,
        )

    # 软限位复核（收敛后再查一遍；越界即判失败，点名关节与数值）。
    problems = _violations(q, names, lower, upper)
    if problems:
        return IkResult(
            q=tuple(float(v) for v in q),
            success=False,
            iterations=iterations,
            message="收敛解软限位复核未通过：" + "；".join(problems),
            reason=REASON_SOFT_LIMIT_VIOLATION,
            position_error=position_error,
            orientation_error=orientation_error,
        )

    return IkResult(
        q=tuple(float(v) for v in q),
        success=True,
        iterations=iterations,
        message=(
            f"IK 收敛：迭代 {iterations} 次，位置残差 {position_error:.3e} m，"
            f"姿态残差 {orientation_error:.3e} rad，软限位复核通过"
        ),
        reason=REASON_NONE,
        position_error=position_error,
        orientation_error=orientation_error,
    )


def batch_solve_ik(
    scene,
    targets: Iterable[Any],
    **kwargs,
) -> List[IkResult]:
    """批量求解（逐个目标独立求解，返回与输入等长的结果列表）。

    参数
    ----
    scene:
        同 :func:`solve_ik`。
    targets:
        目标位姿序列（每个是 ``RigidTransform``）。
    **kwargs:
        原样透传给 :func:`solve_ik`。

    返回
    ----
    list[:class:`IkResult`]
        与 ``targets`` 等长、顺序一致。
    """
    return [solve_ik(scene, target, **kwargs) for target in targets]
