"""任务目标（goal）类型（S06-T01）。

分层
----
本模块是 **S06 任务层最底层的纯数据契约**，只依赖标准库：

* :class:`TaskGoal` —— 目标基类：**原子** ``goal_id``（任务内唯一、非空）
  + ``kind``（``pose`` / ``relative_pose`` / ``object``）；
* :class:`PoseGoal` —— 世界系绝对位姿目标（xyz: m / rpy: rad）；
* :class:`RelativePoseGoal` —— 相对位姿目标（相对某个物体的偏移位姿，
  xyz: m / rpy: rad），用于「双臂共同搬运 / 交接」这类**相对量**表达；
* :class:`ObjectGoal` —— 物体目标（夹爪作用在哪个物体上）；
* :class:`GoalRegistry` —— 任务内的目标表（``goal_id`` → 目标），
  由 :func:`build_goal_registry` 构造，**重复 goal_id 直接报错并点名**。

纪律
----
* 全部是 **frozen dataclass**（不可变），保证「单一主状态源」下目标不会被就地改写；
* 单位一律 **m / rad**；``rpy`` 超出 ``[-π, π]``、长度超出 100 m ⇒
  :class:`GoalError`（点名字段），防止「角度制 / 毫米」写错；
* **零 pydrake 依赖**（只用标准库），因此可离线单测。

错误层级
--------
:class:`TaskError` 是任务层**所有**错误的基类；:class:`GoalError` 是目标相关
错误的子类。:mod:`rm75_drake.tasks.schema` 复用同一个 :class:`TaskError`
（跨模块统一捕获入口）。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Any, Dict, Iterable, Iterator, Mapping, Optional, Tuple

__all__ = [
    "TaskError",
    "GoalError",
    "GOAL_KINDS",
    "TaskGoal",
    "PoseGoal",
    "RelativePoseGoal",
    "ObjectGoal",
    "GoalRegistry",
    "build_goal_registry",
]


class TaskError(ValueError):
    """任务层通用错误（schema 违规 / 目标违规）的基类。"""


class GoalError(TaskError):
    """目标（goal）违规：``goal_id`` 空 / 非法 ``kind`` / 重复 ``goal_id`` / 量纲越界。"""


#: 目标种类（白名单）。
GOAL_KINDS: Tuple[str, ...] = ("pose", "relative_pose", "object")

#: rpy 主值区间上限（弧度）——超出视为「角度制写错」。
_RPY_LIMIT = math.pi

#: 长度量纲上限（米）——超出视为「单位写错」（例如把毫米当米填）。
_LENGTH_LIMIT = 100.0


# ── 内部校验工具 ────────────────────────────────────────────────────────
def _check_goal_id(goal_id: Any, path: str = "goal_id") -> str:
    if not isinstance(goal_id, str) or not goal_id.strip():
        raise GoalError(f"{path} 必须是非空字符串，收到 {goal_id!r}")
    return goal_id


def _check_kind(kind: Any, path: str = "kind") -> str:
    if kind not in GOAL_KINDS:
        raise GoalError(f"{path} = {kind!r} 不受支持；允许的取值只有 {list(GOAL_KINDS)}")
    return str(kind)


def _check_vec3(values: Any, path: str, limit: float, unit: str) -> Tuple[float, float, float]:
    """校验 3 维数值序列（长度 / 角度）。"""
    if isinstance(values, (str, bytes)) or not isinstance(values, (tuple, list)):
        raise GoalError(f"{path} 必须是 3 维序列（list），收到 {type(values).__name__}")
    if len(values) != 3:
        raise GoalError(f"{path} 必须是 3 维序列（list），收到 {len(values)} 个元素")
    out: list = []
    for i, value in enumerate(values):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise GoalError(f"{path}[{i}] 必须是数值，收到 {value!r}")
        fvalue = float(value)
        if not math.isfinite(fvalue):
            raise GoalError(f"{path}[{i}] 必须是有限数值，收到 {value!r}")
        if abs(fvalue) > limit:
            raise GoalError(
                f"{path}[{i}] = {fvalue} 超出{unit}上限 {limit}；"
                f"{'长度一律用米（毫米请先换算）' if unit == '长度' else 'rpy 必须用弧度（rad）'}"
            )
        out.append(fvalue)
    return (out[0], out[1], out[2])


def _check_name(value: Any, path: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise GoalError(f"{path} 必须是非空字符串，收到 {value!r}")
    return value


# ── 目标类型 ────────────────────────────────────────────────────────────
@dataclass(frozen=True, kw_only=True)
class TaskGoal:
    """任务目标基类。

    字段
    ----
    goal_id:
        **原子**目标标识：任务内唯一、非空字符串。它是状态机推进的最小单位
        （S06 的「原子 goal_id」口径）。
    kind:
        目标种类，取值见 :data:`GOAL_KINDS`。
    """

    goal_id: str
    kind: str = "pose"

    def __post_init__(self) -> None:
        object.__setattr__(self, "goal_id", _check_goal_id(self.goal_id))
        object.__setattr__(self, "kind", _check_kind(self.kind))

    def as_dict(self) -> Dict[str, Any]:
        return {"goal_id": self.goal_id, "kind": self.kind}


@dataclass(frozen=True, kw_only=True)
class PoseGoal(TaskGoal):
    """世界系绝对位姿目标（xyz: m / rpy: rad）。"""

    kind: str = "pose"
    xyz: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    rpy: Tuple[float, float, float] = (0.0, 0.0, 0.0)

    def __post_init__(self) -> None:
        super().__post_init__()
        object.__setattr__(self, "xyz", _check_vec3(self.xyz, "xyz", _LENGTH_LIMIT, "长度"))
        object.__setattr__(self, "rpy", _check_vec3(self.rpy, "rpy", _RPY_LIMIT, "弧度"))

    def as_dict(self) -> Dict[str, Any]:
        out = super().as_dict()
        out["xyz"] = list(self.xyz)
        out["rpy"] = list(self.rpy)
        return out


@dataclass(frozen=True, kw_only=True)
class RelativePoseGoal(TaskGoal):
    """相对位姿目标（相对 ``relative_to`` 所指物体的偏移位姿）。

    ``xyz`` / ``rpy`` 表达的是**相对量**（例如「沿 +y 平移 0.10 m」），
    单位仍为 m / rad。
    """

    kind: str = "relative_pose"
    xyz: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    rpy: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    relative_to: str = ""

    def __post_init__(self) -> None:
        super().__post_init__()
        object.__setattr__(self, "xyz", _check_vec3(self.xyz, "xyz", _LENGTH_LIMIT, "长度"))
        object.__setattr__(self, "rpy", _check_vec3(self.rpy, "rpy", _RPY_LIMIT, "弧度"))
        object.__setattr__(self, "relative_to", _check_name(self.relative_to, "relative_to"))

    def as_dict(self) -> Dict[str, Any]:
        out = super().as_dict()
        out["xyz"] = list(self.xyz)
        out["rpy"] = list(self.rpy)
        out["relative_to"] = self.relative_to
        return out


@dataclass(frozen=True, kw_only=True)
class ObjectGoal(TaskGoal):
    """物体目标（夹爪作用在哪个物体上）。"""

    kind: str = "object"
    object_name: str = ""

    def __post_init__(self) -> None:
        super().__post_init__()
        object.__setattr__(self, "object_name", _check_name(self.object_name, "object_name"))

    def as_dict(self) -> Dict[str, Any]:
        out = super().as_dict()
        out["object_name"] = self.object_name
        return out


# ── 目标表 ──────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class GoalRegistry:
    """任务内的目标表（``goal_id`` → :class:`TaskGoal`）。"""

    _goals: Mapping[str, TaskGoal] = field(default_factory=dict)

    def __getitem__(self, goal_id: str) -> TaskGoal:
        return self._goals[goal_id]

    def get(self, goal_id: str, default: Optional[TaskGoal] = None) -> Optional[TaskGoal]:
        return self._goals.get(goal_id, default)

    def __contains__(self, goal_id: object) -> bool:
        return goal_id in self._goals

    def __iter__(self) -> Iterator[str]:
        return iter(self._goals)

    def __len__(self) -> int:
        return len(self._goals)

    def ids(self) -> Tuple[str, ...]:
        return tuple(self._goals.keys())

    def as_dict(self) -> Dict[str, Dict[str, Any]]:
        return {goal_id: goal.as_dict() for goal_id, goal in self._goals.items()}


def build_goal_registry(goals: Iterable[TaskGoal]) -> GoalRegistry:
    """由目标列表构造 :class:`GoalRegistry`（``goal_id`` 重复 ⇒ 报错并点名）。"""
    table: Dict[str, TaskGoal] = {}
    for goal in goals:
        if not isinstance(goal, TaskGoal):
            raise GoalError(f"目标必须是 TaskGoal 实例，收到 {type(goal).__name__}")
        if goal.goal_id in table:
            raise GoalError(
                f"重复的 goal_id：{goal.goal_id!r}（任务内 goal_id 必须唯一）；"
                "请给该原语显式指定不同的 goal_id"
            )
        table[goal.goal_id] = goal
    return GoalRegistry(MappingProxyType(dict(table)))
