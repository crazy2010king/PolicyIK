"""任务原语库（S06-T03）：10 个可组合原语的**静态目录**。

职责边界
--------
本模块只回答两个问题：

1. **有哪些原语**（名字白名单 + 每个原语的种类 / 夹爪动作 / 是否需要物体）；
2. **每个原语的默认误差门槛是多少**。

它**不**负责求解（求解在 :mod:`rm75_drake.tasks.solver`），也**不**负责状态机
（状态机在 S06-T04 的 :mod:`rm75_drake.tasks.state_machine`）——保持
「数据目录 / 执行 / 编排」三层分离。

原语名映射
----------
roadmap 里写的是 ``pre-grasp``（带连字符），本仓库**统一写作**
``pre_grasp``（下划线），与 :mod:`rm75_drake.tasks.schema` 的原语名白名单
一致；``handover`` 在 roadmap 与本仓库写法相同。

默认误差门槛的来历
------------------
:data:`DEFAULT_POSITION_TOLERANCE_M` / :data:`DEFAULT_ORIENTATION_TOLERANCE_RAD`
取 **1e-5**（m / rad）。**理由**：官方 URDF 的关节 rpy 是**舍入值**
（±1.5708 而不是 ±π/2），这带来模型固有的 **~2.24e-6 m** 位置差异
（见 :mod:`rm75_drake.kinematics.mdh` 的模块 docstring 与 MEM041）。
因此门槛必须

* **大于** 2.24e-6 m —— 否则会把「模型舍入残差」误判成真实偏差；
* **远小于** 任何有工程意义的偏差（毫米级）—— 否则会掩盖真实偏差。

1e-5 m 恰好落在两者之间。

零 pydrake 依赖
---------------
本模块只用标准库（:mod:`rm75_drake.tasks.goals` 提供 :class:`TaskError`），
import 不拉起 pydrake。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional, Tuple

from .goals import TaskError
from .schema import PRIMITIVE_NAMES

__all__ = [
    "PRIMITIVE_NAMES",
    "PRIMITIVE_KINDS",
    "GRIPPER_ACTION_KINDS",
    "DEFAULT_POSITION_TOLERANCE_M",
    "DEFAULT_ORIENTATION_TOLERANCE_RAD",
    "PrimitiveSpec",
    "PrimitiveCatalogSpec",
    "PRIMITIVE_SPECS",
    "primitive_spec",
    "primitive_names",
]


#: 原语种类（**原语目录**的种类，与任务文件里 ``kind`` 字段不是同一层概念）。
#:
#: * ``motion`` —— 关节运动类原语（目标位姿 → 解析 IK → 插值 → 复检 → 重定时）；
#: * ``gripper`` —— 夹爪开合类原语（发一条夹爪指令）；
#: * ``verify`` —— 夹爪状态回读类原语（不发指令，只看反馈）；
#: * ``bimanual_motion`` —— 双臂类原语（S06-T05 落地，单臂求解器不做实现）。
PRIMITIVE_KINDS: Tuple[str, ...] = ("motion", "gripper", "verify", "bimanual_motion")

#: 夹爪动作种类（``close`` / ``open``）。
GRIPPER_ACTION_KINDS: Tuple[str, ...] = ("close", "open")

#: 默认位置误差门槛（m）。
DEFAULT_POSITION_TOLERANCE_M: float = 1.0e-5

#: 默认姿态误差门槛（rad）。
DEFAULT_ORIENTATION_TOLERANCE_RAD: float = 1.0e-5


@dataclass(frozen=True)
class PrimitiveSpec:
    """**原语目录**条目（静态元数据，不可变）。

    字段
    ----
    name:
        原语名（:data:`PRIMITIVE_NAMES` 之一）。
    kind:
        原语种类（:data:`PRIMITIVE_KINDS` 之一）。
    gripper_action_kind:
        夹爪动作种类（``"close"`` / ``"open"`` / ``None``）。
        ``None`` 表示该原语**不发**夹爪指令。
    requires_object:
        该原语是否**必须**绑定一个任务物体（``target_object``）。
    description:
        人类可读说明（中文）。
    """

    name: str
    kind: str
    gripper_action_kind: Optional[str]
    requires_object: bool
    description: str

    def as_dict(self) -> Dict[str, object]:
        """返回可序列化的字典视图（审计 / 指标口径用）。"""
        return {
            "name": self.name,
            "kind": self.kind,
            "gripper_action_kind": self.gripper_action_kind,
            "requires_object": self.requires_object,
            "description": self.description,
        }


#: 原语目录（10 个原语）。
PRIMITIVE_SPECS: Dict[str, PrimitiveSpec] = {
    "approach": PrimitiveSpec(
        name="approach",
        kind="motion",
        gripper_action_kind=None,
        requires_object=False,
        description="接近：运动到目标物体上方的接近位姿",
    ),
    "pre_grasp": PrimitiveSpec(
        name="pre_grasp",
        kind="motion",
        gripper_action_kind=None,
        requires_object=False,
        description="预抓取：运动到夹爪可闭合的预备位姿",
    ),
    "close": PrimitiveSpec(
        name="close",
        kind="gripper",
        gripper_action_kind="close",
        requires_object=True,
        description="闭合夹爪，抓取目标物体",
    ),
    "verify": PrimitiveSpec(
        name="verify",
        kind="verify",
        gripper_action_kind=None,
        requires_object=True,
        description="回读夹爪状态，确认抓取是否到位",
    ),
    "lift": PrimitiveSpec(
        name="lift",
        kind="motion",
        gripper_action_kind=None,
        requires_object=False,
        description="抬起：夹持物体离开支撑面",
    ),
    "transit": PrimitiveSpec(
        name="transit",
        kind="motion",
        gripper_action_kind=None,
        requires_object=False,
        description="转运：夹持物体移动到目标投放点上方",
    ),
    "place": PrimitiveSpec(
        name="place",
        kind="motion",
        gripper_action_kind=None,
        requires_object=False,
        description="放置：运动到投放位姿",
    ),
    "open": PrimitiveSpec(
        name="open",
        kind="gripper",
        gripper_action_kind="open",
        requires_object=True,
        description="张开夹爪，释放物体",
    ),
    "retreat": PrimitiveSpec(
        name="retreat",
        kind="motion",
        gripper_action_kind=None,
        requires_object=False,
        description="撤离：从投放位姿退回到安全位姿",
    ),
    "handover": PrimitiveSpec(
        name="handover",
        kind="bimanual_motion",
        gripper_action_kind=None,
        requires_object=True,
        description="双臂交接：两臂在交接点交换承载（常数 X_LR，S06-T05 实现）",
    ),
}


#: :class:`PrimitiveSpec` 的**别名**（供包级导出使用）。
#:
#: 原因：本包的包级 ``PrimitiveSpec`` 沿用
#: :class:`rm75_drake.tasks.schema.PrimitiveSpec`（**任务文件里的原语条目**，
#: S06-T01 已冻结），而本模块的 :class:`PrimitiveSpec` 是**原语目录条目**——
#: 两者字段不同，不能同名混用，故在包级用 ``PrimitiveCatalogSpec`` 指代本模块的类。
PrimitiveCatalogSpec = PrimitiveSpec


def primitive_names() -> Tuple[str, ...]:
    """返回原语名白名单（顺序即 :data:`PRIMITIVE_NAMES`）。"""
    return tuple(PRIMITIVE_NAMES)


def primitive_spec(name: str) -> PrimitiveSpec:
    """按原语名取目录条目。

    参数
    ----
    name:
        原语名。

    返回
    ----
    :class:`PrimitiveSpec`

    异常
    ----
    :class:`rm75_drake.tasks.goals.TaskError`
        原语名不在白名单内（**点名**该名字，并列出允许的名字）。
    """
    if not isinstance(name, str) or name not in PRIMITIVE_SPECS:
        raise TaskError(
            f"原语名 {name!r} 不在白名单内；允许的原语名只有 {list(PRIMITIVE_NAMES)}"
        )
    return PRIMITIVE_SPECS[name]
