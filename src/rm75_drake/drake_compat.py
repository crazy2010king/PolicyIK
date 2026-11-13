"""Drake 版本差异的薄封装（T02：只做探测与占位）。

本模块**不隐藏** Drake 的版本差异，而是把已知差异收敛成"探测 + 统一入口"，
具体能力随 S01（模型加载）/ S02（场景与规划）落地，TODO 标记处届时补齐。

已钉死的差异项（来自 T02 调研）：

1. ``EvalBodyPoseInWorld`` vs ``EvalPoseInWorld``——旧版只有 ``EvalPoseInWorld``。
2. ``ToRollPitchYaw().roll_angle()`` vs ``.roll()``——新版提供 ``roll()``。
3. ``plant.get_joint(plant.GetJointIndices()[i])``——旧版是 ``plant.joint(index)``。
4. ``CollisionCheckerParams.robot_model_instances`` 必须是 ``ModelInstanceIndex``，
   且 ``edge_step_size > 0``（否则 Drake CHECK 失败）。
5. ``DifferentialInverseKinematicsParameters(num_positions, num_velocities)``——
   老版本允许 ``num_velocities=None``，新版本必须显式给出。
6. ``DiffIKResult.status`` 的判定方式随版本变化（新版返回枚举，旧版可能是布尔/字符串）。
"""

from __future__ import annotations

import importlib.metadata as _importlib_metadata
import sys
from typing import Any, List, Optional, Sequence, Tuple

__all__ = [
    "prepare_plant_for_parsing",
    "drake_version",
    "drake_version_tuple",
    "baseline_profile",
    "eval_body_pose_in_world",
    "roll_pitch_yaw",
    "joint_by_index",
    "make_collision_checker_params",
    "diff_ik_parameters",
    "diff_ik_succeeded",
]


def drake_version() -> Optional[str]:
    """返回当前环境实装的 Drake 版本；未安装返回 ``None``。"""
    try:
        return _importlib_metadata.version("drake")
    except _importlib_metadata.PackageNotFoundError:
        return None


def drake_version_tuple() -> Tuple[int, ...]:
    """返回实装 Drake 版本号元组（例如 ``(1, 51, 1)``）；未安装返回 ``()``。"""
    raw = drake_version()
    if not raw:
        return ()
    parts = []
    for chunk in raw.split("."):
        digits = "".join(ch for ch in chunk if ch.isdigit())
        parts.append(int(digits) if digits else 0)
    return tuple(parts)


def baseline_profile() -> str:
    """按当前解释器版本返回 versions.lock 中的档位名（``py312`` / ``py310``）。"""
    return "py312" if sys.version_info >= (3, 12) else "py310"


# ── 差异项 0：解析前的 plant 预备（Drake 1.51.1 的坑）────────────────
def prepare_plant_for_parsing(
    plant,
    slot_name: str = "rm75_drake_reserved_slot",
    slots: int = 8,
):
    """在解析模型**之前**为 plant 预留若干个空的 model instance 槽位。

    为什么需要（Drake 1.51.1 实测，fallback 档）：
    ``Parser.AddModels`` 内部会构造 ``CollisionFilterGroupResolver``，其构造函数里有
    ``DRAKE_DEMAND(minimum_model_instance_index_ > world)``，其中
    ``minimum_model_instance_index_ = plant->num_model_instances()``，
    ``world = plant->world_body().model_instance()``。
    **未 finalize 的 plant 上，world body 的 model instance 不可靠**（Python 侧
    ``plant.world_body().model_instance()`` 恒显示 0，但 C++ 侧读到的值随堆布局变化）。
    于是该 DEMAND 变成**间歇性**失败并直接 abort 进程
    （``detail_collision_filter_group_resolver.cc:27``，
    ``condition 'minimum_model_instance_index_ > world' failed``）。

    本函数把 ``num_model_instances()`` 抬到足够高，**降低**触发概率，但在本机实测
    **不能保证 100% 消除**（pytest 进程内仍会失败）。因此
    ``tests/test_package_map.py`` 的模型加载用例把解析放在**独立子进程**里跑：
    abort 只影响子进程，pytest 会话不会被打挂。

    TODO(S01)：在 Drake 1.55（受支持基线）上复核该 DEMAND 是否仍存在；
    若已修复，应删除本函数并简化调用方。
    """
    add_model_instance = getattr(plant, "AddModelInstance", None)
    if add_model_instance is None:  # pragma: no cover - 极老版本
        return None
    reserved = []
    for i in range(max(0, int(slots))):
        name = slot_name if i == 0 else f"{slot_name}_{i}"
        try:
            reserved.append(add_model_instance(name))
        except Exception:  # 槽位已存在（重复调用）
            continue
    return reserved


# ── 差异项 1：位姿求值 ────────────────────────────────────────────────
def eval_body_pose_in_world(context, body) -> Any:
    """返回 body 在世界系下的位姿。

    新版用 ``plant.EvalBodyPoseInWorld``，旧版只有 ``EvalPoseInWorld``；这里优先调用新版，
    旧版自动回落。TODO(S01)：确认 RM75-B 的 base link 语义后收敛为单一调用。
    """
    plant = body.get_model().GetPlant() if hasattr(body, "get_model") else None
    if plant is None:
        raise TypeError("eval_body_pose_in_world 需要传入 MultibodyPlant 或其 body")
    if hasattr(plant, "EvalBodyPoseInWorld"):
        return plant.EvalBodyPoseInWorld(context, body)
    if hasattr(plant, "EvalPoseInWorld"):  # pragma: no cover - 老版本路径
        return plant.EvalPoseInWorld(context, body)
    raise AttributeError("该 Drake 版本既无 EvalBodyPoseInWorld 也无 EvalPoseInWorld")


# ── 差异项 2：RPY 取值 ────────────────────────────────────────────────
def roll_pitch_yaw(rpy) -> Tuple[float, float, float]:
    """从 ``ToRollPitchYaw()`` 结果取 (roll, pitch, yaw)，兼容 ``roll()`` / ``roll_angle()``。"""
    def _pick(*names: str) -> float:
        for name in names:
            attr = getattr(rpy, name, None)
            if attr is None:
                continue
            return float(attr() if callable(attr) else attr)
        raise AttributeError(f"无法从 {type(rpy).__name__} 取到 {names}")

    return (
        _pick("roll", "roll_angle"),
        _pick("pitch", "pitch_angle"),
        _pick("yaw", "yaw_angle"),
    )


# ── 差异项 3：关节索引取关节 ──────────────────────────────────────────
def joint_by_index(plant, index: int):
    """按**序号**取关节（兼容 ``get_joint(GetJointIndices()[i])`` 与旧的 ``joint(i)``）。"""
    if hasattr(plant, "get_joint"):
        return plant.get_joint(plant.GetJointIndices()[index])
    return plant.joint(index)  # pragma: no cover - 老版本路径


# ── 差异项 4：碰撞检查参数 ────────────────────────────────────────────
def make_collision_checker_params(
    robot_model_instances: Optional[Sequence[Any]] = None,
    edge_step_size: float = 0.01,
    model: Optional[Any] = None,
):
    """构造 ``CollisionCheckerParams``。

    * ``robot_model_instances`` 必须是 ``ModelInstanceIndex``（传入裸整数会被 Drake CHECK 拒绝）；
    * ``edge_step_size`` 必须 > 0，否则 Drake CHECK 失败。
    """
    from pydrake.multibody.tree import ModelInstanceIndex
    from pydrake.planning import CollisionCheckerParams

    params = CollisionCheckerParams()
    params.edge_step_size = edge_step_size
    if model is not None:
        params.model = model
    if robot_model_instances:
        params.robot_model_instances = [
            mi if isinstance(mi, ModelInstanceIndex) else ModelInstanceIndex(int(mi))
            for mi in robot_model_instances
        ]
    return params


# ── 差异项 5/6：DiffIK 参数与结果判定 ────────────────────────────────
def diff_ik_parameters(num_positions: int, num_velocities: Optional[int] = None):
    """构造 ``DifferentialInverseKinematicsParameters``（新旧签名兼容）。"""
    from pydrake.multibody.inverse_kinematics import DifferentialInverseKinematicsParameters

    if num_velocities is None:
        try:
            return DifferentialInverseKinematicsParameters(num_positions)
        except TypeError:  # pragma: no cover - 新版本要求显式 num_velocities
            return DifferentialInverseKinematicsParameters(num_positions, num_positions)
    return DifferentialInverseKinematicsParameters(num_positions, num_velocities)


def diff_ik_succeeded(result) -> bool:
    """判断 DiffIK 结果是否成功（兼容枚举 status / 布尔 status）。"""
    status = getattr(result, "status", None)
    if status is None:
        return False
    if isinstance(status, bool):
        return status
    name = getattr(status, "name", None)
    if name is not None:
        return str(name) == "kSolutionFound"
    return str(status) == "kSolutionFound"
