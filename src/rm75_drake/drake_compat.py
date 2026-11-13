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
7. ``CollisionCheckerParams.model`` 在 1.51/1.55 **只接受 ``RobotDiagram``**：传
   ``MultibodyPlant`` 或 ``ModelInstanceIndex`` 会抛 pybind 的
   ``RuntimeError: Unable to cast Python instance of type ... to C++ type '?'``。
   ``SceneGraphCollisionChecker`` 的构造器在这两档**只接受**
   ``CollisionCheckerParams`` 一个参数（ kwargs 重载虽然存在，但同样受 §7 限制）。
   因此 :func:`make_collision_checker_params` 会把非法 ``model`` 显式转成
   可读的 :class:`TypeError`，并由 :func:`make_collision_checker` 提供工厂入口。

## 反模式（差异项 0 的翻版）：解析前不得预预留 model instance 槽位

T02 早期版本提供过 ``prepare_plant_for_parsing()``，在解析前往 plant 里塞若干空
model instance 槽位，想规避 ``detail_collision_filter_group_resolver.cc:27`` 的
``minimum_model_instance_index_ > world`` abort。**该变通本身就是缺陷**：
``minimum_model_instance_index_`` 取自 ``plant->num_model_instances()``，预预留把
它抬到大于 world 之后，Drake 反而**确定性**触发该 DEMAND。

实测对照（唯一变量 = 是否预预留）：基线 Drake 1.55.0 预预留 0/20 成功、不预预留
20/20 成功；降级 Drake 1.51.1 预预留 0/10 成功、不预预留 10/10 成功。
因此该函数**已删除**，model instance 的分配交还 Drake Parser 管理。
"""

from __future__ import annotations

import importlib.metadata as _importlib_metadata
import sys
from typing import Any, List, Optional, Sequence, Tuple

__all__ = [
    "drake_version",
    "drake_version_tuple",
    "baseline_profile",
    "eval_body_pose_in_world",
    "roll_pitch_yaw",
    "rotation_from_rpy",
    "joint_by_index",
    "make_collision_checker_params",
    "make_collision_checker",
    "diff_ik_parameters",
    "diff_ik_succeeded",
    "jacobian_6d",
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


# ── 差异项 1：位姿求值 ────────────────────────────────────────────────
def eval_body_pose_in_world(context, body, plant=None) -> Any:
    """返回 body 在世界系下的位姿。

    新版用 ``plant.EvalBodyPoseInWorld``，旧版只有 ``EvalPoseInWorld``；这里优先调用新版，
    旧版自动回落。

    ``plant`` 可显式给出（**推荐**：Drake 1.51 的 ``RigidBody`` 只有
    ``model_instance()``，没有 ``get_model()``，无法从 body 反推 plant）；
    未给出时尝试从 body 自身反推（兼容较新版本的 ``body.get_model()``）。
    """
    if plant is None:
        plant = _plant_from_body(body)
    if plant is None:
        raise TypeError(
            "eval_body_pose_in_world 需要 plant（或能提供 plant 的 body）；"
            "Drake 1.51 的 RigidBody 无法反推 plant，请显式传 plant=。"
        )
    if hasattr(plant, "EvalBodyPoseInWorld"):
        return plant.EvalBodyPoseInWorld(context, body)
    if hasattr(plant, "EvalPoseInWorld"):  # pragma: no cover - 老版本路径
        return plant.EvalPoseInWorld(context, body)
    raise AttributeError("该 Drake 版本既无 EvalBodyPoseInWorld 也无 EvalPoseInWorld")


def _plant_from_body(body) -> Any:
    """尝试从 body 反推所属 MultibodyPlant；无法反推时返回 ``None``。

    兼容两种形态：
    * 较新版本：``body.get_model()`` 返回 MultibodyPlant；
    * 旧版本：``body.get_model()`` 返回 ModelInstance（需要 ``.GetPlant()``）。
    """
    getter = getattr(body, "get_model", None)
    if not callable(getter):
        return None
    model = getter()
    if hasattr(model, "GetPlant"):
        return model.GetPlant()
    if hasattr(model, "EvalBodyPoseInWorld"):
        return model
    return None


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


def rotation_from_rpy(rpy):
    """把 ``(roll, pitch, yaw)``（**弧度**）转成 :class:`pydrake.math.RotationMatrix`。

    Drake 1.51/1.55 的 ``pydrake.math.RotationMatrix`` **没有**静态方法
    ``RPY(...)``（只有 ``RotationMatrix(RollPitchYaw(...))``、四元数、AngleAxis
    等构造重载），因此统一走 ``RollPitchYaw``——1.51/1.55/1.58 同构。

    约定：rpy 是**内旋 xyz**（``R = Rz(yaw)·Ry(pitch)·Rx(roll)``），与
    :class:`rm75_drake.scenario.Scenario` 的 rpy 字段一致。
    """
    from pydrake.math import RollPitchYaw, RotationMatrix

    roll, pitch, yaw = (float(v) for v in rpy)
    return RotationMatrix(RollPitchYaw(roll, pitch, yaw))


# ── 差异项 3：关节索引取关节 ──────────────────────────────────────────
def joint_by_index(plant, index: int):
    """按**序号**取关节（兼容 ``get_joint(GetJointIndices()[i])`` 与旧的 ``joint(i)``）。"""
    if hasattr(plant, "get_joint"):
        return plant.get_joint(plant.GetJointIndices()[index])
    return plant.joint(index)  # pragma: no cover - 老版本路径


# ── 差异项 4：碰撞检查参数 ────────────────────────────────────────────
def _coerce_collision_model(model: Any) -> Any:
    """把 ``model`` 参数规范化成 Drake 接受的 ``RobotDiagram``。

    Drake 1.51/1.55 的 ``CollisionCheckerParams.model`` 字段只接受
    ``RobotDiagram``（pybind 里是 C++ ``Model*``）。传入 ``MultibodyPlant`` 或
    ``ModelInstanceIndex`` 会抛不透明的
    ``RuntimeError: Unable to cast Python instance of type ... to C++ type '?'``。

    这里把两类误用转成**可读的** :class:`TypeError`，并指明正确用法：

    * ``MultibodyPlant`` → 请用 ``RobotDiagramBuilder`` 构建 ``RobotDiagram``
      （见 :func:`rm75_drake.planning.scene.build_scene`）；
    * ``ModelInstanceIndex`` → 请用 ``robot_model_instances=[...]`` 表达。
    """
    from pydrake.multibody.plant import MultibodyPlant
    from pydrake.multibody.tree import ModelInstanceIndex
    from pydrake.planning import RobotDiagram

    if isinstance(model, RobotDiagram):
        return model
    if isinstance(model, MultibodyPlant):
        raise TypeError(
            "CollisionCheckerParams.model 只接受 RobotDiagram（Drake 1.51/1.55 实测）："
            "传 MultibodyPlant 会抛 pybind 的 'Unable to cast' RuntimeError。"
            "请改用 RobotDiagramBuilder 构建 RobotDiagram 后传入"
            "（见 rm75_drake.planning.scene.build_scene）。"
        )
    if isinstance(model, ModelInstanceIndex):
        raise TypeError(
            "CollisionCheckerParams.model 不接受 ModelInstanceIndex："
            "机器人模型实例请用 robot_model_instances=[...] 表达。",
        )
    raise TypeError(
        f"model 必须是 RobotDiagram，收到 {type(model).__name__}"
    )


def make_collision_checker_params(
    robot_model_instances: Optional[Sequence[Any]] = None,
    edge_step_size: float = 0.01,
    model: Optional[Any] = None,
):
    """构造 ``CollisionCheckerParams``。

    * ``robot_model_instances`` 必须是 ``ModelInstanceIndex``（传入裸整数会被 Drake CHECK 拒绝）；
    * ``edge_step_size`` 必须 > 0，否则 Drake CHECK 失败（本函数提前抛
      :class:`ValueError`，把不透明的 CHECK 变成可读错误）；
    * ``model`` 只能是 ``RobotDiagram``（``MultibodyPlant`` / ``ModelInstanceIndex``
      会被 :func:`_coerce_collision_model` 转成可读的 :class:`TypeError`）。
      ``None`` 表示**不设置**该字段（保持旧行为）。
    """
    from pydrake.multibody.tree import ModelInstanceIndex
    from pydrake.planning import CollisionCheckerParams

    if not isinstance(edge_step_size, (int, float)) or isinstance(edge_step_size, bool):
        raise ValueError(f"edge_step_size 必须是数值，收到 {edge_step_size!r}")
    if edge_step_size <= 0.0:
        raise ValueError(f"edge_step_size 必须 > 0（m），收到 {edge_step_size!r}")

    params = CollisionCheckerParams()
    params.edge_step_size = edge_step_size
    if model is not None:
        params.model = _coerce_collision_model(model)
    if robot_model_instances:
        params.robot_model_instances = [
            mi if isinstance(mi, ModelInstanceIndex) else ModelInstanceIndex(int(mi))
            for mi in robot_model_instances
        ]
    return params


def make_collision_checker(
    robot_model_instances: Optional[Sequence[Any]] = None,
    edge_step_size: float = 0.01,
    model: Optional[Any] = None,
    robot_diagram: Optional[Any] = None,
):
    """构造 ``SceneGraphCollisionChecker``（Drake 1.51/1.55 的版本差异收口点）。

    参数
    ----
    robot_diagram:
        ``RobotDiagram``（推荐用这个参数名，与 ``model`` 等价但更明确）。
    model:
        ``model`` 的别名（两者给其一即可）；只接受 ``RobotDiagram``。
    robot_model_instances:
        机器人模型实例（``ModelInstanceIndex`` 列表或裸整数列表）。
    edge_step_size:
        边采样步长（m），必须 > 0。

    返回
    ----
    :class:`pydrake.planning.SceneGraphCollisionChecker`
    """
    from pydrake.planning import SceneGraphCollisionChecker

    diagram = robot_diagram if robot_diagram is not None else model
    if diagram is None:
        raise TypeError(
            "make_collision_checker 需要 RobotDiagram（robot_diagram= / model=）："
            "Drake 1.51/1.55 的 CollisionCheckerParams.model 不接受 MultibodyPlant。"
        )
    params = make_collision_checker_params(
        robot_model_instances=robot_model_instances,
        edge_step_size=edge_step_size,
        model=diagram,
    )
    return SceneGraphCollisionChecker(params)


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


# ── 差异项 8：6×N 雅可比（世界系）────────────────────────────────────────
def jacobian_6d(plant, context, frame, reference_frame):
    """返回末端 frame 的 **6×N 雅可比**（numpy 数组，行序：角速度在前、线速度在后）。

    参数
    ----
    plant:
        :class:`MultibodyPlant`。
    context:
        plant 的上下文。
    frame:
        末端 frame（``frame_B``）。
    reference_frame:
        参考系（``frame_A``，通常是世界系）。

    说明
    ----
    Drake **1.51.1** 与 **1.55.0** 都**没有** ``MultibodyPlant.CalcJacobian``，
    只有三个分量的接口（两版签名**同构**，因此这里不需要版本分支）：

    * ``CalcJacobianAngularVelocity(context, kV, frame_B, frame_A, frame_E)``；
    * ``CalcJacobianTranslationalVelocity(context, kV, frame_B, p_BoBi_B,
      frame_A, frame_E)``。

    其中 ``kV`` = :class:`pydrake.multibody.tree.JacobianWrtVariable` 的速度变量，
    ``p_BoBi_B`` 取末端 frame 原点（``[0, 0, 0]``）。

    **注意 ``frame_E`` 必须取 ``reference_frame``（世界系）**：``frame_E`` 指的是
    "求导所依据的速度变量所属的 frame"。取末端 frame 时，Drake 返回的是
    "末端 frame 自身速度"那组变量（实测：joint_7 列全零），与 plant 的
    广义速度对不上；取世界系时列数 = plant 速度变量数（RM75 固定基座后为 7），
    且与**有限差分**逐列吻合（实测最大偏差 2.6e-7）。

    返回
    ----
    numpy.ndarray
        形状 ``(6, n)``：行 0-2 是角速度雅可比，行 3-5 是线速度雅可比，
        都在 ``reference_frame`` 下表达。
    """
    import numpy as np
    from pydrake.multibody.tree import JacobianWrtVariable

    angular = np.asarray(
        plant.CalcJacobianAngularVelocity(
            context, JacobianWrtVariable.kV, frame, reference_frame, reference_frame
        ),
        dtype=float,
    )
    linear = np.asarray(
        plant.CalcJacobianTranslationalVelocity(
            context,
            JacobianWrtVariable.kV,
            frame,
            np.zeros(3),
            reference_frame,
            reference_frame,
        ),
        dtype=float,
    )
    return np.vstack([angular, linear])
