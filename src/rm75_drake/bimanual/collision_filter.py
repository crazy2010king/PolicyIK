"""双臂碰撞过滤矩阵（S04-T03）。

为什么必须**程序化**建过滤矩阵
------------------------------
Drake 1.51.1 / 1.55.0 的 URDF 解析器**不实现** ``drake:disable_collision`` /
``drake:disable_collide``（源码里靠默认的父子相邻体过滤生效），因此**不能靠 URDF
标签**表达「左右臂互检」/「夹爪指 vs 被抓物体放行」这类策略。本模块把这条矩阵
落成**可执行的代码**（``pydrake.geometry.CollisionFilterManager`` /
``CollisionFilterDeclaration``），并为每条规则提供**正例 + 反例**断言。

语义轴（两个轴，**不要混**）
---------------------------
``FilterRule.action`` 用 **Drake 语义**（描述对碰撞**候选集 C**的操作）：

==========  ==========================================================
``action``  含义
==========  ==========================================================
``allow``   把几何对**加入**候选集 → 该对**会被检测**（报碰撞）
``exclude`` 把几何对**移出**候选集 → 该对**不会被检测**（不报碰撞）
==========  ==========================================================

``FilterRule.contact`` 用 **物理语义**（这一对**允不允许穿透**）：

============  ========================================================
``contact``   含义
============  ========================================================
``forbid``    禁止接触（必须报碰撞）
``permit``    允许接触（**物理上的「放行」** → Drake 侧是 ``exclude``）
============  ========================================================

所以「夹爪指 vs 被抓物体**放行**」这条规则 = ``action="exclude"`` +
``contact="permit"``：**放行 = 不报碰撞 = 把该对移出候选集**。

四条默认规则（稳定标签，CI / 报告按标签断言）
---------------------------------------------
``adjacent_same_arm``
    同臂父子相邻体**保持过滤**（Drake 默认行为）。实现机制是 plant 级的
    ``set_adjacent_bodies_collision_filters``（**必须在 ``Finalize()`` 之前**
    设置，见 :func:`set_adjacent_filters_enabled`），**不是**场景图声明——
    否则会误伤同臂的非相邻连杆。
``left_right_mutual``
    左右臂**互检保持启用**：不做任何 ``ExcludeBetween(left, right)``，并用
    ``AllowBetween(left, right)`` 把互检**显式钉住**（谁把臂间互检关掉，
    这条规则会把它拉回来）。
``gripper_finger_vs_grasped_object``
    夹爪指 vs 被抓物体**放行**（抓取中不该报碰撞）。
``gripper_finger_vs_own_palm``
    夹爪指 vs 本臂掌部放行（闭链自碰不误报）。

过滤**必须真的作用到场景图**
---------------------------
只把规则记在 Python 侧（例如打个日志标记）**不会**改变碰撞判定。本模块把规则
翻译成 ``CollisionFilterDeclaration`` 并通过 ``CollisionFilterManager.Apply``
施加到 :class:`SceneGraph`。

**关键坑（实测钉死）**：``SceneGraphCollisionChecker`` 的查询走的是
``CollisionCheckerContext`` 里的**上下文副本**的场景图状态，
**不是** ``SceneGraph`` 的默认模型。因此：

* 只对 ``scene_graph.collision_filter_manager()``（默认模型）施加声明 →
  ``SceneGraphInspector.CollisionFiltered`` 会变成 ``True``，
  但**碰撞判定结果不变**（过滤没生效）；
* 必须用 ``checker.PerformOperationAgainstAllModelContexts(...)`` 把声明施加到
  **每个模型上下文**的场景图状态上，碰撞判定才会翻面。

本模块**两条都做**：默认模型 + 检查器的全部模型上下文（后者才是「判定真的变了」
的关键）。调用方若不传 ``checker``，则只作用到默认模型（场景图查询可见）。

相邻体过滤的**装配期**约束
-------------------------
``plant.set_adjacent_bodies_collision_filters(True/False)`` 在 Drake 里
**只能在 ``Finalize()`` 之前调用**（post-finalize 直接抛
``RuntimeError: Post-finalize calls to 'set_adjacent_bodies_collision_filters()'
are not allowed``）。因此本模块提供两个入口：

* :func:`set_adjacent_filters_enabled` —— 装配期调用（转发到 Drake）；
* ``rm75_drake.bimanual.dual_scene.build_dual_scene(
  adjacent_bodies_collision_filters=...)`` —— 场景装配期把开关交给调用方。

post-finalize 再调用 :func:`set_adjacent_filters_enabled` 会抛
:class:`FilterRuleError`（点名「必须装配期设置」），**不静默吞掉**。
"""

from __future__ import annotations

import fnmatch
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

__all__ = [
    "FilterRuleError",
    "RuleTarget",
    "FilterRule",
    "RuleApplication",
    "FilterReport",
    "ACTIONS",
    "SCOPES",
    "MECHANISMS",
    "RULE_LABELS",
    "ROLE_LEFT_ARM",
    "ROLE_RIGHT_ARM",
    "ROLE_GRIPPER_FINGERS",
    "ROLE_GRIPPER_PALMS",
    "ROLE_GRASPED_OBJECTS",
    "default_dual_arm_rules",
    "validate_rule",
    "build_filter_manager",
    "apply_dual_arm_filters",
    "set_adjacent_filters_enabled",
]


# ── 错误类型 ────────────────────────────────────────────────────────────
class FilterRuleError(RuntimeError):
    """过滤规则非法 / 无法解析时抛出（**点名**规则与出错原因）。

    继承 :class:`RuntimeError`（而不是裸 ``ValueError``）是为了让调用方一眼
    区分「Drake 底层报错」与「规则本身写错」。
    """


# ── 词表 ────────────────────────────────────────────────────────────────
#: ``action`` 的合法取值（**Drake 语义**：对碰撞候选集 C 的操作）。
ACTIONS: Tuple[str, ...] = ("allow", "exclude")

#: ``scope`` 的合法取值（Drake 的声明作用域）。
SCOPES: Tuple[str, ...] = ("between", "within")

#: 规则的实现机制。
#:
#: * ``scene_graph`` —— 翻译成 ``CollisionFilterDeclaration`` 施加到场景图；
#: * ``plant_adjacent`` —— 由 plant 级的相邻体过滤开关实现（装配期设置）。
MECHANISMS: Tuple[str, ...] = ("scene_graph", "plant_adjacent")

#: 四条默认规则的**稳定标签**（CI / 报告按标签断言，不得改名）。
RULE_LABELS: Tuple[str, ...] = (
    "adjacent_same_arm",
    "left_right_mutual",
    "gripper_finger_vs_grasped_object",
    "gripper_finger_vs_own_palm",
)

#: 角色名（规则里用角色引用「哪一侧 / 哪一类几何」，由调用方绑定到具体实例）。
ROLE_LEFT_ARM = "left_arm"
ROLE_RIGHT_ARM = "right_arm"
ROLE_GRIPPER_FINGERS = "gripper_fingers"
ROLE_GRIPPER_PALMS = "gripper_palms"
ROLE_GRASPED_OBJECTS = "grasped_objects"

#: 角色 → (绑定来源, 连杆名匹配模式)。
_ROLE_PATTERNS: Dict[str, Tuple[str, str]] = {
    ROLE_LEFT_ARM: ("left", "*"),
    ROLE_RIGHT_ARM: ("right", "*"),
    ROLE_GRIPPER_FINGERS: ("grippers", "*finger*"),
    ROLE_GRIPPER_PALMS: ("grippers", "*gripper_base*"),
    ROLE_GRASPED_OBJECTS: ("grasped", "*"),
}


# ── 数据结构 ────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class RuleTarget:
    """规则的一端：**（model instance, geometry filter）** 描述。

    字段
    ----
    role:
        角色名（见 :data:`ROLE_LEFT_ARM` 等）。非空时**优先**于 ``model``。
    model:
        具体目标：``ModelInstanceIndex`` / model instance 名字 /
        ``GeometryId``（锚定障碍物或单个几何）。``None`` = 不绑定。
    link_pattern:
        连杆名匹配模式（``fnmatch`` 语法，作用在 body / link 名上），
        默认 ``"*"`` = 该 model instance 的全部连杆。
    label:
        报告里显示的名字（默认由 role / model 推出）。
    """

    role: str = ""
    model: Any = None
    link_pattern: str = "*"
    label: str = ""

    def describe(self) -> str:
        """人类可读的描述（点名 role / model / 模式）。"""
        if self.label:
            return self.label
        if self.role:
            return f"{self.role}[{self.link_pattern}]"
        if isinstance(self.model, str):
            return f"{self.model}[{self.link_pattern}]"
        return f"{self.model}[{self.link_pattern}]"


@dataclass(frozen=True)
class FilterRule:
    """一条过滤规则。

    字段
    ----
    name:
        **稳定标签**（默认四条的标签见 :data:`RULE_LABELS`）。
    action:
        ``"allow"`` / ``"exclude"``（**Drake 语义**，见模块 docstring）。
    scope:
        ``"between"`` / ``"within"``（Drake 声明作用域）。
    pairs:
        规则作用的「几何对」列表，每项是 ``(目标A, 目标B)``；
        ``scope="within"`` 时 A 与 B 必须是**同一个目标**。
    rationale:
        这条规则为什么存在（写进报告，便于人审）。
    mechanism:
        ``"scene_graph"`` / ``"plant_adjacent"``（默认 ``"scene_graph"``）。
    contact:
        物理语义：``"forbid"``（禁止接触）/ ``"permit"``（放行）。
    """

    name: str
    action: str
    scope: str
    pairs: Tuple[Tuple[RuleTarget, RuleTarget], ...] = ()
    rationale: str = ""
    mechanism: str = "scene_graph"
    contact: str = "forbid"

    def describe(self) -> str:
        pairs = "; ".join(f"{a.describe()} vs {b.describe()}" for a, b in self.pairs)
        return f"{self.name}[{self.action}/{self.scope}/{self.mechanism}]: {pairs}"


@dataclass(frozen=True)
class RuleApplication:
    """单条规则的施加结果（``applied`` / ``skipped`` + 原因）。"""

    name: str
    status: str
    reason: str
    details: Tuple[str, ...] = ()

    @property
    def applied(self) -> bool:
        return self.status == "applied"

    def as_dict(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "status": self.status,
            "reason": self.reason,
            "details": list(self.details),
        }


@dataclass(frozen=True)
class FilterReport:
    """一次过滤施加的完整报告（逐条规则的结果）。"""

    rules: Tuple[FilterRule, ...] = ()
    applications: Tuple[RuleApplication, ...] = ()
    adjacent_filters_enabled: Optional[bool] = None

    def application(self, name: str) -> RuleApplication:
        """按标签取某条规则的结果（找不到 → 抛 :class:`FilterRuleError`）。"""
        for app in self.applications:
            if app.name == name:
                return app
        raise FilterRuleError(
            f"过滤报告里没有规则 {name!r}；已记录的规则：{[a.name for a in self.applications]}"
        )

    @property
    def labels(self) -> Tuple[str, ...]:
        return tuple(app.name for app in self.applications)

    def as_dict(self) -> Dict[str, Any]:
        return {
            "adjacent_filters_enabled": self.adjacent_filters_enabled,
            "rules": [app.as_dict() for app in self.applications],
        }

    def summary(self) -> str:
        """人类可读的多行摘要（CLI / 日志消费）。"""
        lines = ["碰撞过滤矩阵："]
        for app in self.applications:
            lines.append(f"  - {app.name}: {app.status}（{app.reason}）")
            for detail in app.details:
                lines.append(f"      {detail}")
        adjacent = self.adjacent_filters_enabled
        lines.append(
            "  - 相邻体过滤（plant 级）: "
            + ("开启" if adjacent else ("关闭" if adjacent is False else "未知"))
        )
        return "\n".join(lines)


@dataclass(frozen=True)
class _RoleContext:
    """角色 → 具体实例的绑定（由 :func:`apply_dual_arm_filters` 组装）。"""

    left_instance: Any = None
    right_instance: Any = None
    gripper_instances: Tuple[Any, ...] = ()
    grasped_object_instances: Tuple[Any, ...] = ()


# ── 默认规则 ────────────────────────────────────────────────────────────
def default_dual_arm_rules() -> List[FilterRule]:
    """返回四条默认规则（**稳定标签**，见 :data:`RULE_LABELS`）。

    规则里引用的是**角色**（``left_arm`` / ``right_arm`` / ``gripper_fingers`` /
    ``gripper_palms`` / ``grasped_objects``），具体实例由
    :func:`apply_dual_arm_filters` 绑定。
    """
    return [
        FilterRule(
            name="adjacent_same_arm",
            action="exclude",
            scope="within",
            pairs=(
                (
                    RuleTarget(role=ROLE_LEFT_ARM, link_pattern="*", label="左臂全部连杆"),
                    RuleTarget(role=ROLE_LEFT_ARM, link_pattern="*", label="左臂全部连杆"),
                ),
                (
                    RuleTarget(role=ROLE_RIGHT_ARM, link_pattern="*", label="右臂全部连杆"),
                    RuleTarget(role=ROLE_RIGHT_ARM, link_pattern="*", label="右臂全部连杆"),
                ),
            ),
            rationale=(
                "同臂父子相邻体保持过滤（Drake 默认行为），避免相邻连杆互相误报；"
                "由 plant 级 set_adjacent_bodies_collision_filters 实现（装配期设置），"
                "**不**用场景图声明（否则会误伤同臂非相邻连杆的自碰检测）"
            ),
            mechanism="plant_adjacent",
            contact="forbid",
        ),
        FilterRule(
            name="left_right_mutual",
            action="allow",
            scope="between",
            pairs=(
                (
                    RuleTarget(role=ROLE_LEFT_ARM, link_pattern="*", label="左臂"),
                    RuleTarget(role=ROLE_RIGHT_ARM, link_pattern="*", label="右臂"),
                ),
            ),
            rationale=(
                "左右臂互检必须保持启用（绝不做 ExcludeBetween(left, right)）；"
                "用 AllowBetween 把互检显式钉住，防止别处的过滤把臂间互检关掉"
            ),
            mechanism="scene_graph",
            contact="forbid",
        ),
        FilterRule(
            name="gripper_finger_vs_grasped_object",
            action="exclude",
            scope="between",
            pairs=(
                (
                    RuleTarget(role=ROLE_GRIPPER_FINGERS, link_pattern="*finger*", label="夹爪指"),
                    RuleTarget(role=ROLE_GRASPED_OBJECTS, link_pattern="*", label="被抓物体"),
                ),
            ),
            rationale=(
                "抓取中的物体被夹爪指夹住是**允许接触**（物理放行）："
                "把该对移出碰撞候选集，否则夹爪一夹紧就报碰撞"
            ),
            mechanism="scene_graph",
            contact="permit",
        ),
        FilterRule(
            name="gripper_finger_vs_own_palm",
            action="exclude",
            scope="between",
            pairs=(
                (
                    RuleTarget(role=ROLE_GRIPPER_FINGERS, link_pattern="*finger*", label="夹爪指"),
                    RuleTarget(role=ROLE_GRIPPER_PALMS, link_pattern="*gripper_base*", label="本臂掌部"),
                ),
            ),
            rationale=(
                "夹爪指 vs 本臂掌部放行（闭链自碰不误报）；"
                "Drake 的相邻体过滤已覆盖父子对，这条规则是**纵深防御**"
            ),
            mechanism="scene_graph",
            contact="permit",
        ),
    ]


# ── 校验 ────────────────────────────────────────────────────────────────
def validate_rule(rule: FilterRule) -> None:
    """校验一条规则；非法 → 抛 :class:`FilterRuleError`（点名规则与出错值）。

    校验项：``action`` / ``scope`` / ``mechanism`` / ``contact`` 词表、
    ``pairs`` 的结构（``within`` 必须是自对）、规则名非空且唯一性由调用方保证。
    """
    if not rule.name:
        raise FilterRuleError("过滤规则必须有非空的 name（稳定标签）")
    if rule.action not in ACTIONS:
        raise FilterRuleError(
            f"规则 {rule.name!r} 的 action 非法：{rule.action!r}；"
            f"合法取值 {list(ACTIONS)}（Drake 语义：allow=加入候选集，exclude=移出候选集）"
        )
    if rule.scope not in SCOPES:
        raise FilterRuleError(
            f"规则 {rule.name!r} 的 scope 非法：{rule.scope!r}；合法取值 {list(SCOPES)}"
        )
    if rule.mechanism not in MECHANISMS:
        raise FilterRuleError(
            f"规则 {rule.name!r} 的 mechanism 非法：{rule.mechanism!r}；"
            f"合法取值 {list(MECHANISMS)}"
        )
    if rule.contact not in ("permit", "forbid"):
        raise FilterRuleError(
            f"规则 {rule.name!r} 的 contact 非法：{rule.contact!r}；"
            "合法取值 ['permit', 'forbid']"
        )
    if not rule.pairs:
        raise FilterRuleError(f"规则 {rule.name!r} 没有任何 pairs（几何对）")
    for index, (first, second) in enumerate(rule.pairs):
        for target in (first, second):
            if not target.role and target.model is None:
                raise FilterRuleError(
                    f"规则 {rule.name!r} 的第 {index} 个几何对里有一个目标既没有 role "
                    "也没有 model（无法定位几何）"
                )
            if target.role and target.role not in _ROLE_PATTERNS:
                raise FilterRuleError(
                    f"规则 {rule.name!r} 引用了未知角色 {target.role!r}；"
                    f"已知角色 {sorted(_ROLE_PATTERNS)}"
                )
        if rule.scope == "within" and first is not second and (
            first.role != second.role
            or first.model != second.model
            or first.link_pattern != second.link_pattern
        ):
            raise FilterRuleError(
                f"规则 {rule.name!r} 的 scope='within'，第 {index} 个几何对的两端必须"
                f"是同一个目标，实际是 {first.describe()} vs {second.describe()}"
            )


# ── 几何解析 ────────────────────────────────────────────────────────────
def _model_instance_index(plant, model: Any, *, rule_name: str, target: RuleTarget):
    """把 ``model`` 解析成 ``ModelInstanceIndex``；不存在 → 显式报错并点名。"""
    from pydrake.multibody.tree import ModelInstanceIndex

    if isinstance(model, ModelInstanceIndex):
        index = model
    elif isinstance(model, bool):  # bool 是 int 的子类，先挡掉
        raise FilterRuleError(
            f"规则 {rule_name!r} 的目标 {target.describe()} 的 model 类型非法：{model!r}"
        )
    elif isinstance(model, int):
        index = ModelInstanceIndex(model)
    elif isinstance(model, str):
        if not plant.HasModelInstanceNamed(model):
            raise FilterRuleError(
                f"规则 {rule_name!r} 的目标 {target.describe()} 引用了不存在的 model "
                f"instance 名字 {model!r}；plant 里现有："
                f"{[plant.GetModelInstanceName(ModelInstanceIndex(i)) for i in range(plant.num_model_instances())]}"
            )
        index = plant.GetModelInstanceByName(model)
    else:
        raise FilterRuleError(
            f"规则 {rule_name!r} 的目标 {target.describe()} 的 model 类型非法：{type(model)!r}"
            "（应为 ModelInstanceIndex / model instance 名字 / GeometryId）"
        )
    if not 0 <= int(index) < plant.num_model_instances():
        raise FilterRuleError(
            f"规则 {rule_name!r} 的目标 {target.describe()} 引用了 plant 里不存在的 "
            f"model instance（索引 {int(index)}，plant 只有 "
            f"{plant.num_model_instances()} 个 model instance）"
        )
    if not plant.GetBodyIndices(index):
        raise FilterRuleError(
            f"规则 {rule_name!r} 的目标 {target.describe()} 引用的 model instance "
            f"（索引 {int(index)}）在 plant 里没有任何 body"
        )
    return index


def _is_geometry_id(value: Any) -> bool:
    try:
        from pydrake.geometry import GeometryId
    except Exception:  # pragma: no cover - 无 Drake 环境
        return False
    return isinstance(value, GeometryId)


def _geometry_ids_for_model(plant, inspector, index, pattern: str) -> List[Any]:
    """返回某个 model instance 里匹配 ``pattern`` 的**带 proximity 角色**的几何。"""
    out: List[Any] = []
    for body_index in plant.GetBodyIndices(index):
        name = plant.get_body(body_index).name()
        if not fnmatch.fnmatch(name, pattern):
            continue
        frame_id = plant.GetBodyFrameIdOrThrow(body_index)
        for geometry_id in inspector.GetGeometries(frame_id):
            try:
                # 只有带 proximity 角色的几何才参与碰撞判定
                # （没有该角色时 Drake 抛 RuntimeError）。
                inspector.GetProximityProperties(geometry_id)
            except RuntimeError:
                continue
            out.append(geometry_id)
    return out


def _target_sources(target: RuleTarget, ctx: _RoleContext) -> List[Tuple[str, Any, str]]:
    """把一个目标解析成 ``(kind, payload, pattern)`` 列表。

    ``kind`` 为 ``"model"``（payload = model instance）或 ``"geometry"``
    （payload = GeometryId）。
    """
    if target.role:
        source, pattern = _ROLE_PATTERNS[target.role]
        if source == "left":
            if ctx.left_instance is None:
                return []
            return [("model", ctx.left_instance, pattern)]
        if source == "right":
            if ctx.right_instance is None:
                return []
            return [("model", ctx.right_instance, pattern)]
        if source == "grippers":
            return [("model", instance, pattern) for instance in ctx.gripper_instances]
        if source == "grasped":
            sources: List[Tuple[str, Any, str]] = []
            for item in ctx.grasped_object_instances:
                if _is_geometry_id(item):
                    sources.append(("geometry", item, pattern))
                else:
                    sources.append(("model", item, pattern))
            return sources
        raise FilterRuleError(f"未知角色 {target.role!r}（内部错误）")
    if target.model is None:
        return []
    if _is_geometry_id(target.model):
        return [("geometry", target.model, target.link_pattern)]
    return [("model", target.model, target.link_pattern)]


def _resolve_geometry_ids(
    plant,
    inspector,
    target: RuleTarget,
    ctx: _RoleContext,
    *,
    rule_name: str,
) -> List[Any]:
    """解析目标 → 几何 id 列表（空列表 = 该端没绑到任何几何）。"""
    out: List[Any] = []
    for kind, payload, pattern in _target_sources(target, ctx):
        if kind == "geometry":
            try:
                inspector.GetName(payload)
            except RuntimeError as exc:
                raise FilterRuleError(
                    f"规则 {rule_name!r} 的目标 {target.describe()} 引用了未注册的几何 "
                    f"{payload}：{exc}"
                ) from exc
            out.append(payload)
        else:
            index = _model_instance_index(plant, payload, rule_name=rule_name, target=target)
            out.extend(_geometry_ids_for_model(plant, inspector, index, pattern))
    # 去重且保持顺序（GeometryId 不能转 int，用 repr 做键）
    seen = set()
    unique = []
    for geometry_id in out:
        key = str(geometry_id)
        if key in seen:
            continue
        seen.add(key)
        unique.append(geometry_id)
    return unique


def _geometry_set(geometry_ids: Iterable[Any]):
    from pydrake.geometry import GeometrySet

    geometry_set = GeometrySet()
    for geometry_id in geometry_ids:
        geometry_set.Add(geometry_id)
    return geometry_set


# ── 相邻体过滤（plant 级）──────────────────────────────────────────────
def set_adjacent_filters_enabled(plant, enabled: bool) -> bool:
    """封装 ``plant.set_adjacent_bodies_collision_filters``。

    **Drake 硬约束**：只能在 ``Finalize()`` **之前**调用；post-finalize 调用会抛
    ``RuntimeError``。本函数把它收口成 :class:`FilterRuleError`（点名「必须
    装配期设置」，并给出替代入口
    ``build_dual_scene(adjacent_bodies_collision_filters=...)``）。

    返回当前（设置后）的开关状态。
    """
    try:
        plant.set_adjacent_bodies_collision_filters(bool(enabled))
    except RuntimeError as exc:
        raise FilterRuleError(
            "相邻体过滤开关只能在 plant Finalize() **之前**设置"
            f"（Drake 限制：{exc}）；"
            "请在装配期传入 build_dual_scene(adjacent_bodies_collision_filters=...)"
        ) from exc
    return _adjacent_filters_state(plant)


def _adjacent_filters_state(plant) -> Optional[bool]:
    """读取 plant 的相邻体过滤开关（读不到 → ``None``，不猜）。"""
    getter = getattr(plant, "get_adjacent_bodies_collision_filters", None)
    if getter is None:
        return None
    try:
        return bool(getter())
    except RuntimeError:  # pragma: no cover - 老版本无 getter
        return None


# ── 声明构造与施加 ──────────────────────────────────────────────────────
def _declaration_for_rule(
    rule: FilterRule,
    plant,
    scene_graph,
    ctx: _RoleContext,
):
    """把一条规则翻译成 ``CollisionFilterDeclaration``。

    返回 ``(declaration, reason, details)``；``declaration is None`` 表示这条规则
    **没有可作用的几何**（例如没装夹爪）→ 调用方记 ``skipped``。
    """
    from pydrake.geometry import CollisionFilterDeclaration

    inspector = scene_graph.model_inspector()
    statements = []
    details: List[str] = []
    for first, second in rule.pairs:
        first_ids = _resolve_geometry_ids(plant, inspector, first, ctx, rule_name=rule.name)
        second_ids = _resolve_geometry_ids(plant, inspector, second, ctx, rule_name=rule.name)
        if not first_ids or not second_ids:
            details.append(
                f"{first.describe()} → {len(first_ids)} 个几何 / "
                f"{second.describe()} → {len(second_ids)} 个几何（无可作用几何）"
            )
            continue
        set_a = _geometry_set(first_ids)
        set_b = _geometry_set(second_ids)
        if rule.scope == "within":
            if rule.action == "allow":
                statements.append(("allow_within", set_a))
            else:
                statements.append(("exclude_within", set_a))
        else:
            if rule.action == "allow":
                statements.append(("allow_between", set_a, set_b))
            else:
                statements.append(("exclude_between", set_a, set_b))
        details.append(
            f"{first.describe()}({len(first_ids)} 几何) vs {second.describe()}({len(second_ids)} 几何)"
        )
    if not statements:
        return None, "该规则没有解析到任何可作用的几何", tuple(details)

    declaration = CollisionFilterDeclaration()
    for statement in statements:
        if statement[0] == "allow_between":
            declaration = declaration.AllowBetween(statement[1], statement[2])
        elif statement[0] == "exclude_between":
            declaration = declaration.ExcludeBetween(statement[1], statement[2])
        elif statement[0] == "allow_within":
            declaration = declaration.AllowWithin(statement[1])
        else:
            declaration = declaration.ExcludeWithin(statement[1])
    reason = f"{rule.action}/{rule.scope} 施加于 {len(statements)} 组几何对"
    return declaration, reason, tuple(details)


def _apply_declaration(declaration, scene_graph, checker) -> None:
    """把声明施加到场景图（**默认模型** + **检查器的全部模型上下文**）。

    只施加到默认模型时，检查器的碰撞判定**不会变**（实测钉死，见模块
    docstring）；因此检查器路径必须逐上下文施加。
    """
    default_manager = scene_graph.collision_filter_manager()
    default_manager.Apply(declaration)
    if checker is None:
        return

    def _operation(diagram, model_context) -> None:
        manager = diagram.scene_graph().collision_filter_manager(model_context.scene_graph_context())
        manager.Apply(declaration)

    checker.PerformOperationAgainstAllModelContexts(_operation)


def build_filter_manager(
    plant,
    scene_graph,
    rules: Sequence[FilterRule],
    *,
    checker=None,
    role_context: Optional[_RoleContext] = None,
):
    """把规则翻译成 ``CollisionFilterDeclaration`` 并**施加到场景图**。

    参数
    ----
    plant:
        已装配（**已 Finalize**）的 :class:`MultibodyPlant`。
    scene_graph:
        与 plant 配套的 :class:`SceneGraph`。
    rules:
        规则列表（每条先过 :func:`validate_rule`）。
    checker:
        可选的 :class:`SceneGraphCollisionChecker`。给了就把声明**逐模型上下文**
        施加（否则碰撞判定看不到过滤，见模块 docstring）。
    role_context:
        角色 → 实例的绑定（默认全空 = 角色型规则会被跳过）。

    返回
    ----
    实际使用的 :class:`CollisionFilterManager`（默认模型的那个）。
    """
    ctx = role_context or _RoleContext()
    for rule in rules:
        validate_rule(rule)
    declarations = []
    for rule in rules:
        if rule.mechanism != "scene_graph":
            continue
        declaration, _reason, _details = _declaration_for_rule(rule, plant, scene_graph, ctx)
        if declaration is not None:
            declarations.append(declaration)
    manager = scene_graph.collision_filter_manager()
    for declaration in declarations:
        _apply_declaration(declaration, scene_graph, checker)
    return manager


def apply_dual_arm_filters(
    plant,
    scene_graph,
    *,
    left_instance: Any = None,
    right_instance: Any = None,
    gripper_instances: Optional[Sequence[Any]] = None,
    grasped_object_instances: Optional[Sequence[Any]] = None,
    rules: Optional[Sequence[FilterRule]] = None,
    checker=None,
    adjacent_filters_enabled: Optional[bool] = None,
) -> FilterReport:
    """施加双臂碰撞过滤矩阵，返回**逐条规则**的结果。

    参数
    ----
    plant / scene_graph:
        场景（``plant`` 必须已 Finalize）。
    left_instance / right_instance:
        左右臂的 model instance（``ModelInstanceIndex`` 或名字）。
    gripper_instances:
        夹爪的 model instance 列表（没装夹爪 → 夹爪类规则记 ``skipped``）。
    grasped_object_instances:
        被抓物体的标识列表：可以是 model instance，也可以是 ``GeometryId``
        （锚定障碍物 / 单个几何）。
    rules:
        规则列表（默认 :func:`default_dual_arm_rules`）。
    checker:
        可选碰撞检查器（给了就逐模型上下文施加，碰撞判定才会真的变）。
    adjacent_filters_enabled:
        相邻体过滤的目标状态。``None``（默认）= **保持 plant 当前状态**（只报告）；
        给定 ``True`` / ``False`` 而 plant 当前状态与之不同时 → 抛
        :class:`FilterRuleError`（该开关只能在装配期设置）。

    返回
    ----
    :class:`FilterReport`（逐条规则的 applied / skipped + 原因）。
    """
    ctx = _RoleContext(
        left_instance=left_instance,
        right_instance=right_instance,
        gripper_instances=tuple(gripper_instances or ()),
        grasped_object_instances=tuple(grasped_object_instances or ()),
    )
    rule_list = list(rules) if rules is not None else default_dual_arm_rules()
    applications: List[RuleApplication] = []
    adjacent_state = _adjacent_filters_state(plant)

    for rule in rule_list:
        validate_rule(rule)
        if rule.mechanism == "plant_adjacent":
            if adjacent_state is None:
                applications.append(
                    RuleApplication(
                        name=rule.name,
                        status="skipped",
                        reason=(
                            "当前 Drake 版本读不到相邻体过滤开关状态，"
                            "无法确认该规则是否生效"
                        ),
                    )
                )
                continue
            if (
                adjacent_filters_enabled is not None
                and bool(adjacent_filters_enabled) != adjacent_state
            ):
                # plant 已 Finalize → 改不了；显式报错（不静默）。
                set_adjacent_filters_enabled(plant, adjacent_filters_enabled)
            if adjacent_state:
                applications.append(
                    RuleApplication(
                        name=rule.name,
                        status="applied",
                        reason="相邻体过滤保持开启（plant 级，已在装配期设置）",
                        details=(f"plant 相邻体过滤开关 = {adjacent_state}",),
                    )
                )
            else:
                applications.append(
                    RuleApplication(
                        name=rule.name,
                        status="skipped",
                        reason=(
                            "plant 的相邻体过滤当前是关闭的（装配期设置），"
                            "本规则无法在运行期开启"
                        ),
                        details=(f"plant 相邻体过滤开关 = {adjacent_state}",),
                    )
                )
            continue

        declaration, reason, details = _declaration_for_rule(rule, plant, scene_graph, ctx)
        if declaration is None:
            applications.append(
                RuleApplication(name=rule.name, status="skipped", reason=reason, details=details)
            )
            continue
        try:
            _apply_declaration(declaration, scene_graph, checker)
        except RuntimeError as exc:
            raise FilterRuleError(
                f"规则 {rule.name!r} 施加失败：{exc}"
            ) from exc
        applications.append(
            RuleApplication(name=rule.name, status="applied", reason=reason, details=details)
        )

    return FilterReport(
        rules=tuple(rule_list),
        applications=tuple(applications),
        adjacent_filters_enabled=adjacent_state,
    )
