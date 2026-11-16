"""任务 / 物体 / 原语的正式 schema 与严格校验（S06-T01）。

本模块是**任务文件（部署实例数据）的真相源解析器**，纪律与
:mod:`rm75_drake.scenario` / :mod:`rm75_drake.calibration.schema` 完全一致：

* **严格键白名单**：未知顶层键 / 未知子键 → :class:`TaskError`，
  异常信息里**点名该键**；
* **缺必填字段** → :class:`TaskError`（点名缺哪个键）；
* **维度错**（``xyz`` / ``rpy`` / ``half_size`` 不是 3 维）→ :class:`TaskError`；
* **单位错**（``rpy`` 写成角度制、长度写成毫米）→ :class:`TaskError`，并给出换算提示；
* **原语名白名单**：10 个原语名之外的名字 → :class:`TaskError`（点名该名字）。

单位约定（全表 SI，禁止在别处另立单位）
----------------------------------------

========  ==========================================================
量纲       约定
========  ==========================================================
长度      m（米）。``xyz`` / ``half_size`` 一律 m。
角度      **rad（弧度）**。所有 ``rpy`` 一律 rad。
时间      s（秒）。``hold_seconds`` 一律 s。
========  ==========================================================

目录语义（**同名易混，必须分清**）
--------------------------------

* ``src/rm75_drake/tasks/`` → **Python 包**（代码：schema / goals / primitives / ...）；
* ``<repo root>/tasks/``      → **部署实例数据**（``*.yaml`` 任务定义），
  路径常量是 :data:`rm75_drake.paths.task_files_dir`。

任务文件形状（``tasks/pick_place.yaml`` 的形状）::

    schema_version: 1
    name: pick_place
    description: "单臂抓放链"
    robot: left
    scenario: single_arm.yaml
    objects:
      - name: cube_a
        half_size: [0.02, 0.02, 0.02]
        pose: {xyz: [0.45, 0.0, 0.02], rpy: [0.0, 0.0, 0.0]}
    thresholds: {position_m: 0.005, orientation_rad: 0.05}
    primitives:
      - name: approach
        kind: motion
        target: {xyz: [0.45, 0.0, 0.18], rpy: [0.0, 0.0, 0.0]}
        tolerance: {position_m: 0.01, orientation_rad: 0.1}
      - name: close
        kind: gripper
        action: close
        target_object: cube_a
        hold_seconds: 0.5

**本模块零 pydrake 依赖**（只用标准库 + PyYAML），因此可离线单测。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

from .goals import (
    GoalError,
    GoalRegistry,
    ObjectGoal,
    PoseGoal,
    RelativePoseGoal,
    TaskError,
    TaskGoal,
    build_goal_registry,
)

__all__ = [
    "TaskError",
    "GoalError",
    "SCHEMA_VERSION",
    "TOP_LEVEL_KEYS",
    "REQUIRED_TOP_LEVEL_KEYS",
    "PRIMITIVE_NAMES",
    "PRIMITIVE_KINDS",
    "GRIPPER_ACTIONS",
    "OBJECT_KEYS",
    "PRIMITIVE_KEYS",
    "TOLERANCE_KEYS",
    "THRESHOLD_KEYS",
    "PoseSpec",
    "Tolerance",
    "ObjectSpec",
    "PrimitiveSpec",
    "TaskSpec",
    "parse_task",
    "load_task_file",
    "goal_for_primitive",
]

#: 当前正式 schema 版本。
SCHEMA_VERSION = 1

#: 顶层键（顺序即 YAML 里的书写顺序）。
TOP_LEVEL_KEYS: Tuple[str, ...] = (
    "schema_version",
    "name",
    "description",
    "robot",
    "scenario",
    "objects",
    "primitives",
    "thresholds",
)

#: 必填顶层键（其余可缺省或为空）。
REQUIRED_TOP_LEVEL_KEYS: Tuple[str, ...] = (
    "schema_version",
    "name",
    "robot",
    "primitives",
)

#: 10 个任务原语名白名单（roadmap 里的 ``pre-grasp`` 在本仓库统一写作 ``pre_grasp``）。
PRIMITIVE_NAMES: Tuple[str, ...] = (
    "approach",
    "pre_grasp",
    "close",
    "verify",
    "lift",
    "transit",
    "place",
    "open",
    "retreat",
    "handover",
)

#: 原语种类（白名单）。
PRIMITIVE_KINDS: Tuple[str, ...] = ("motion", "gripper", "bimanual_motion")

#: 夹爪动作（白名单）。
GRIPPER_ACTIONS: Tuple[str, ...] = ("close", "open")

#: 物体条目的合法子键（全部必填）。
OBJECT_KEYS: Tuple[str, ...] = ("name", "half_size", "pose")

#: 原语条目的合法子键。
PRIMITIVE_KEYS: Tuple[str, ...] = (
    "name",
    "goal_id",
    "kind",
    "target",
    "target_object",
    "tolerance",
    "action",
    "hold_seconds",
)

#: 容差条目的合法子键（``position_m`` / ``orientation_rad`` 必填）。
TOLERANCE_KEYS: Tuple[str, ...] = ("position_m", "orientation_rad")

#: 任务级默认门槛的合法子键。
THRESHOLD_KEYS: Tuple[str, ...] = ("position_m", "orientation_rad")

#: 位姿条目的合法子键（``xyz`` / ``rpy`` 必填）。
_POSE_KEYS: Tuple[str, ...] = ("xyz", "rpy")

#: rpy 主值区间上限（弧度）——超出视为「角度制写错」。
_RPY_LIMIT = math.pi

#: 长度量纲上限（米）——超出视为「单位写错」（例如把毫米当米填）。
_LENGTH_LIMIT = 100.0


# ── 内部校验工具 ────────────────────────────────────────────────────────
def _require_mapping(value: Any, path: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise TaskError(f"{path} 必须是映射（mapping），收到 {type(value).__name__}")
    for key in value:
        if not isinstance(key, str):
            raise TaskError(f"{path} 的键必须是字符串，收到 {key!r}")
    return value


def _require_sequence(value: Any, path: str) -> Sequence[Any]:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise TaskError(f"{path} 必须是序列（list），收到 {type(value).__name__}")
    return value


def _check_keys(
    mapping: Mapping[str, Any],
    path: str,
    allowed: Sequence[str],
    required: Sequence[str],
) -> None:
    unknown = [k for k in mapping if k not in allowed]
    if unknown:
        raise TaskError(
            f"{path} 出现未知键：{sorted(unknown)}；该处允许的键只有 {list(allowed)}"
        )
    missing = [k for k in required if k not in mapping]
    if missing:
        raise TaskError(
            f"{path} 缺少必填字段：{missing}；该处允许的键只有 {list(allowed)}"
        )


def _as_float(value: Any, path: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TaskError(f"{path} 必须是数值，收到 {value!r}")
    out = float(value)
    if not math.isfinite(out):
        raise TaskError(f"{path} 必须是有限数值，收到 {value!r}")
    return out


def _as_int(value: Any, path: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise TaskError(f"{path} 必须是整数，收到 {value!r}")
    return int(value)


def _require_str(value: Any, path: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise TaskError(f"{path} 必须是非空字符串，收到 {value!r}")
    return value


def _as_float_list(value: Any, path: str, length: int) -> Tuple[float, ...]:
    seq = _require_sequence(value, path)
    if len(seq) != length:
        raise TaskError(f"{path} 必须是 {length} 个数值，收到 {len(seq)} 个")
    return tuple(_as_float(v, f"{path}[{i}]") for i, v in enumerate(seq))


def _check_xyz(values: Tuple[float, ...], path: str) -> Tuple[float, float, float]:
    for i, v in enumerate(values):
        if abs(v) > _LENGTH_LIMIT:
            raise TaskError(
                f"{path}[{i}] = {v} 超出长度上限 {_LENGTH_LIMIT} m；"
                "长度一律用米（若填的是毫米请先换算）"
            )
    return (values[0], values[1], values[2])


def _check_rpy(values: Tuple[float, ...], path: str) -> Tuple[float, float, float]:
    for i, v in enumerate(values):
        if abs(v) > _RPY_LIMIT:
            raise TaskError(
                f"{path}[{i}] = {v} 超出弧度主值区间 [-π, π]；"
                "rpy 必须用弧度（rad），不接收角度制（度）——"
                f"若填的是度，请乘 {math.pi / 180.0:.10f} 换算为弧度"
            )
    return (values[0], values[1], values[2])


# ── 数据结构 ────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class PoseSpec:
    """6D 位姿（xyz: m / rpy: rad）。"""

    xyz: Tuple[float, float, float]
    rpy: Tuple[float, float, float]

    def as_dict(self) -> Dict[str, Any]:
        return {"xyz": list(self.xyz), "rpy": list(self.rpy)}


@dataclass(frozen=True)
class Tolerance:
    """原语容差（位置 m / 姿态 rad）。"""

    position_m: float
    orientation_rad: float

    def as_dict(self) -> Dict[str, Any]:
        return {
            "position_m": self.position_m,
            "orientation_rad": self.orientation_rad,
        }


@dataclass(frozen=True)
class ObjectSpec:
    """任务中的物体条目。"""

    name: str
    half_size: Tuple[float, float, float]
    pose: PoseSpec

    def as_dict(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "half_size": list(self.half_size),
            "pose": self.pose.as_dict(),
        }


@dataclass(frozen=True)
class PrimitiveSpec:
    """任务原语条目（一个原语 = 一个原子目标 + 夹爪动作 + 容差）。"""

    name: str
    goal_id: str
    kind: str
    target: Optional[PoseSpec] = None
    target_object: Optional[str] = None
    tolerance: Optional[Tolerance] = None
    action: Optional[str] = None
    hold_seconds: Optional[float] = None

    def as_dict(self) -> Dict[str, Any]:
        out: Dict[str, Any] = {
            "name": self.name,
            "goal_id": self.goal_id,
            "kind": self.kind,
        }
        if self.target is not None:
            out["target"] = self.target.as_dict()
        if self.target_object is not None:
            out["target_object"] = self.target_object
        if self.tolerance is not None:
            out["tolerance"] = self.tolerance.as_dict()
        if self.action is not None:
            out["action"] = self.action
        if self.hold_seconds is not None:
            out["hold_seconds"] = self.hold_seconds
        return out


@dataclass(frozen=True)
class TaskSpec:
    """任务文件（schema_version 1）。

    字段
    ----
    schema_version:
        正式 schema 版本（当前 :data:`SCHEMA_VERSION`）。
    name:
        任务名（非空字符串）。
    robot:
        执行机器人标识（单臂如 ``"left"``，双臂如 ``"dual_arm"``）。
    primitives:
        原语条目序列（顺序即执行顺序）。
    objects:
        物体条目序列。
    description:
        任务描述（可缺省）。
    scenario:
        关联场景文件名（相对 ``scenarios/``，可缺省）。
    thresholds:
        任务级默认门槛（``position_m`` / ``orientation_rad``）。
    source_path:
        来源文件路径（内存构造时为 ``None``）。
    """

    schema_version: int
    name: str
    robot: str
    primitives: Tuple[PrimitiveSpec, ...]
    objects: Tuple[ObjectSpec, ...] = ()
    description: str = ""
    scenario: Optional[str] = None
    thresholds: Mapping[str, float] = field(default_factory=dict)
    source_path: Optional[str] = None

    # ── 目标视图 ────────────────────────────────────────────────────────
    def goals(self) -> List[TaskGoal]:
        """返回该任务的原子目标列表（与 ``primitives`` 一一对应）。"""
        return [goal_for_primitive(p) for p in self.primitives]

    def goal_ids(self) -> Tuple[str, ...]:
        return tuple(p.goal_id for p in self.primitives)

    def goal_registry(self) -> GoalRegistry:
        """返回目标表（``goal_id`` 重复 ⇒ :class:`GoalError`）。"""
        return build_goal_registry(self.goals())

    def primitive(self, name: str) -> PrimitiveSpec:
        """按原语名取原语（不存在 ⇒ :class:`TaskError` 并点名）。"""
        for prim in self.primitives:
            if prim.name == name:
                return prim
        raise TaskError(
            f"任务 {self.name!r} 中不存在原语 {name!r}；"
            f"已声明的原语有 {[p.name for p in self.primitives]}"
        )

    def object(self, name: str) -> ObjectSpec:
        """按物体名取物体（不存在 ⇒ :class:`TaskError` 并点名）。"""
        for obj in self.objects:
            if obj.name == name:
                return obj
        raise TaskError(
            f"任务 {self.name!r} 中不存在物体 {name!r}；"
            f"已声明的物体有 {[o.name for o in self.objects]}"
        )

    def as_dict(self) -> Dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "name": self.name,
            "description": self.description,
            "robot": self.robot,
            "scenario": self.scenario,
            "objects": [o.as_dict() for o in self.objects],
            "primitives": [p.as_dict() for p in self.primitives],
            "thresholds": dict(self.thresholds),
            "source_path": self.source_path,
        }


def goal_for_primitive(primitive: PrimitiveSpec) -> TaskGoal:
    """把原语条目映射成原子目标。

    * ``motion`` → :class:`PoseGoal`（世界系绝对位姿）；
    * ``bimanual_motion`` → :class:`RelativePoseGoal`（相对 ``target_object`` 的偏移位姿）；
    * ``gripper`` → :class:`ObjectGoal`（作用在 ``target_object`` 上）。
    """
    if primitive.kind == "motion":
        if primitive.target is None:
            raise TaskError(
                f"原语 {primitive.name!r}（kind=motion）缺少 target，无法构造位姿目标"
            )
        return PoseGoal(
            goal_id=primitive.goal_id,
            xyz=primitive.target.xyz,
            rpy=primitive.target.rpy,
        )
    if primitive.kind == "bimanual_motion":
        if primitive.target is None:
            raise TaskError(
                f"原语 {primitive.name!r}（kind=bimanual_motion）缺少 target，无法构造相对位姿目标"
            )
        if not primitive.target_object:
            raise TaskError(
                f"原语 {primitive.name!r}（kind=bimanual_motion）缺少 target_object，"
                "无法确定相对位姿的参照物体"
            )
        return RelativePoseGoal(
            goal_id=primitive.goal_id,
            xyz=primitive.target.xyz,
            rpy=primitive.target.rpy,
            relative_to=primitive.target_object,
        )
    if primitive.kind == "gripper":
        if not primitive.target_object:
            raise TaskError(
                f"原语 {primitive.name!r}（kind=gripper）缺少 target_object，"
                "无法确定作用对象"
            )
        return ObjectGoal(
            goal_id=primitive.goal_id,
            object_name=primitive.target_object,
        )
    raise TaskError(
        f"原语 {primitive.name!r} 的 kind = {primitive.kind!r} 不受支持；"
        f"允许的取值只有 {list(PRIMITIVE_KINDS)}"
    )


# ── 解析 ────────────────────────────────────────────────────────────────
def _parse_pose(entry: Any, path: str) -> PoseSpec:
    mapping = _require_mapping(entry, path)
    _check_keys(mapping, path, _POSE_KEYS, _POSE_KEYS)
    xyz = _check_xyz(_as_float_list(mapping["xyz"], f"{path}.xyz", 3), f"{path}.xyz")
    rpy = _check_rpy(_as_float_list(mapping["rpy"], f"{path}.rpy", 3), f"{path}.rpy")
    return PoseSpec(xyz=xyz, rpy=rpy)


def _parse_objects(entry: Any) -> Tuple[ObjectSpec, ...]:
    seq = _require_sequence(entry, "objects")
    out: List[ObjectSpec] = []
    seen: Dict[str, int] = {}
    for i, raw in enumerate(seq):
        path = f"objects[{i}]"
        mapping = _require_mapping(raw, path)
        _check_keys(mapping, path, OBJECT_KEYS, OBJECT_KEYS)
        name = _require_str(mapping["name"], f"{path}.name")
        if name in seen:
            raise TaskError(
                f"{path}.name = {name!r} 与 objects[{seen[name]}] 重名；物体名在任务内必须唯一"
            )
        seen[name] = i
        half = _as_float_list(mapping["half_size"], f"{path}.half_size", 3)
        for k, v in enumerate(half):
            if v <= 0.0:
                raise TaskError(f"{path}.half_size[{k}] = {v} 必须为正数（半长）")
        pose = _parse_pose(mapping["pose"], f"{path}.pose")
        out.append(ObjectSpec(name=name, half_size=half, pose=pose))
    return tuple(out)


def _parse_tolerance(entry: Any, path: str) -> Tolerance:
    mapping = _require_mapping(entry, path)
    _check_keys(mapping, path, TOLERANCE_KEYS, TOLERANCE_KEYS)
    position_m = _as_float(mapping["position_m"], f"{path}.position_m")
    orientation_rad = _as_float(mapping["orientation_rad"], f"{path}.orientation_rad")
    if position_m < 0.0:
        raise TaskError(f"{path}.position_m = {position_m} 不能为负")
    if orientation_rad < 0.0:
        raise TaskError(f"{path}.orientation_rad = {orientation_rad} 不能为负")
    if position_m > _LENGTH_LIMIT:
        raise TaskError(
            f"{path}.position_m = {position_m} 超出长度上限 {_LENGTH_LIMIT} m；长度一律用米"
        )
    if orientation_rad > _RPY_LIMIT:
        raise TaskError(
            f"{path}.orientation_rad = {orientation_rad} 超出弧度主值区间 [-π, π]；"
            "姿态容差一律用弧度（rad）"
        )
    return Tolerance(position_m=position_m, orientation_rad=orientation_rad)


def _parse_thresholds(entry: Any) -> Dict[str, float]:
    mapping = _require_mapping(entry, "thresholds")
    _check_keys(mapping, "thresholds", THRESHOLD_KEYS, ())
    out: Dict[str, float] = {}
    for key in THRESHOLD_KEYS:
        if key not in mapping:
            continue
        value = _as_float(mapping[key], f"thresholds.{key}")
        if value < 0.0:
            raise TaskError(f"thresholds.{key} = {value} 不能为负")
        if key == "position_m" and value > _LENGTH_LIMIT:
            raise TaskError(
                f"thresholds.position_m = {value} 超出长度上限 {_LENGTH_LIMIT} m；长度一律用米"
            )
        if key == "orientation_rad" and value > _RPY_LIMIT:
            raise TaskError(
                f"thresholds.orientation_rad = {value} 超出弧度主值区间 [-π, π]；"
                "姿态门槛一律用弧度（rad）"
            )
        out[key] = value
    return out


def _parse_primitives(
    entry: Any, objects: Tuple[ObjectSpec, ...]
) -> Tuple[PrimitiveSpec, ...]:
    seq = _require_sequence(entry, "primitives")
    if len(seq) == 0:
        raise TaskError("primitives 不能为空：任务至少要有一个原语")
    object_names = {o.name for o in objects}
    out: List[PrimitiveSpec] = []
    for i, raw in enumerate(seq):
        path = f"primitives[{i}]"
        mapping = _require_mapping(raw, path)
        _check_keys(mapping, path, PRIMITIVE_KEYS, ("name", "kind"))
        name = _require_str(mapping["name"], f"{path}.name")
        if name not in PRIMITIVE_NAMES:
            raise TaskError(
                f"{path}.name = {name!r} 不是合法的原语名；"
                f"原语名白名单只有 {list(PRIMITIVE_NAMES)}"
            )
        kind = _require_str(mapping["kind"], f"{path}.kind")
        if kind not in PRIMITIVE_KINDS:
            raise TaskError(
                f"{path}.kind = {kind!r} 不受支持；允许的取值只有 {list(PRIMITIVE_KINDS)}"
            )
        goal_id = (
            _require_str(mapping["goal_id"], f"{path}.goal_id")
            if "goal_id" in mapping
            else name
        )
        target: Optional[PoseSpec] = None
        if "target" in mapping:
            target = _parse_pose(mapping["target"], f"{path}.target")
        target_object: Optional[str] = None
        if "target_object" in mapping:
            target_object = _require_str(mapping["target_object"], f"{path}.target_object")
            if target_object not in object_names:
                raise TaskError(
                    f"{path}.target_object = {target_object!r} 未在 objects 中声明；"
                    f"已声明的物体有 {sorted(object_names)}"
                )
        tolerance: Optional[Tolerance] = None
        if "tolerance" in mapping:
            tolerance = _parse_tolerance(mapping["tolerance"], f"{path}.tolerance")
        action: Optional[str] = None
        if "action" in mapping:
            action = _require_str(mapping["action"], f"{path}.action")
            if action not in GRIPPER_ACTIONS:
                raise TaskError(
                    f"{path}.action = {action!r} 不受支持；允许的取值只有 {list(GRIPPER_ACTIONS)}"
                )
        hold_seconds: Optional[float] = None
        if "hold_seconds" in mapping:
            hold_seconds = _as_float(mapping["hold_seconds"], f"{path}.hold_seconds")
            if hold_seconds < 0.0:
                raise TaskError(f"{path}.hold_seconds = {hold_seconds} 不能为负")

        # ── kind 与子键的一致性 ────────────────────────────────────────
        if kind == "gripper":
            if action is None:
                raise TaskError(
                    f"{path}（kind=gripper）缺少必填字段：['action']；"
                    f"夹爪原语必须显式声明 action ∈ {list(GRIPPER_ACTIONS)}"
                )
            if not target_object:
                raise TaskError(
                    f"{path}（kind=gripper）缺少必填字段：['target_object']；"
                    "夹爪原语必须声明作用的物体"
                )
        else:
            if action is not None:
                raise TaskError(
                    f"{path}.action 只允许出现在 kind=gripper 的原语上，"
                    f"当前 kind = {kind!r}"
                )
        if kind in ("motion", "bimanual_motion") and target is None:
            raise TaskError(
                f"{path}（kind={kind}）缺少必填字段：['target']；"
                "运动类原语必须声明目标位姿"
            )
        if kind == "bimanual_motion" and not target_object:
            raise TaskError(
                f"{path}（kind=bimanual_motion）缺少必填字段：['target_object']；"
                "双臂原语必须声明参照物体（相对位姿的基准）"
            )

        out.append(
            PrimitiveSpec(
                name=name,
                goal_id=goal_id,
                kind=kind,
                target=target,
                target_object=target_object,
                tolerance=tolerance,
                action=action,
                hold_seconds=hold_seconds,
            )
        )
    return tuple(out)


def parse_task(data: Any, source_path: Optional[str] = None) -> TaskSpec:
    """从已解析的 dict 构造 :class:`TaskSpec`（校验失败抛 :class:`TaskError`）。"""
    mapping = _require_mapping(data, "task")

    unknown = [k for k in mapping if k not in TOP_LEVEL_KEYS]
    if unknown:
        raise TaskError(
            f"task 出现未知顶层键：{sorted(unknown)}；顶层键固定为 {list(TOP_LEVEL_KEYS)}"
        )
    missing = [k for k in REQUIRED_TOP_LEVEL_KEYS if k not in mapping]
    if missing:
        raise TaskError(f"task 缺少必填顶层键：{missing}")

    version = _as_int(mapping["schema_version"], "schema_version")
    if version != SCHEMA_VERSION:
        raise TaskError(
            f"schema_version = {version} 不受支持；当前正式版本是 {SCHEMA_VERSION}"
        )

    name = _require_str(mapping["name"], "name")
    robot = _require_str(mapping["robot"], "robot")
    description = (
        _require_str(mapping["description"], "description")
        if "description" in mapping
        else ""
    )
    scenario = (
        _require_str(mapping["scenario"], "scenario") if "scenario" in mapping else None
    )

    objects = _parse_objects(mapping["objects"]) if "objects" in mapping else ()
    primitives = _parse_primitives(mapping["primitives"], objects)
    thresholds = (
        _parse_thresholds(mapping["thresholds"]) if "thresholds" in mapping else {}
    )

    spec = TaskSpec(
        schema_version=version,
        name=name,
        robot=robot,
        primitives=primitives,
        objects=objects,
        description=description,
        scenario=scenario,
        thresholds=thresholds,
        source_path=source_path,
    )
    # goal_id 唯一性（重复 ⇒ 报错并点名）。
    build_goal_registry(spec.goals())
    return spec


def load_task_file(path) -> TaskSpec:
    """从 YAML 文件加载任务文件。

    * 文件不存在 → :class:`FileNotFoundError`；
    * YAML 语法错误 → :class:`TaskError`；
    * schema 违规 → :class:`TaskError`（点名违规键）。
    """
    import yaml

    file_path = Path(path)
    try:
        text = file_path.read_text(encoding="utf-8")
    except FileNotFoundError as exc:
        raise FileNotFoundError(f"找不到任务文件：{file_path}") from exc
    except OSError as exc:  # pragma: no cover - 权限/IO 异常
        raise TaskError(f"读取任务文件失败：{file_path}（{exc}）") from exc

    try:
        data = yaml.safe_load(text)
    except yaml.YAMLError as exc:
        raise TaskError(f"任务 YAML 解析失败：{file_path}（{exc}）") from exc

    if data is None:
        raise TaskError(f"任务文件为空：{file_path}")

    return parse_task(data, source_path=str(file_path))
