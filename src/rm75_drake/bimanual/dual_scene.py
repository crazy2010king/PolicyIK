"""双臂**单一** MultibodyPlant 装配器（S04-T02）。

为什么需要独立模块
------------------
S02 的 :func:`rm75_drake.planning.scene.build_scene` 是**单臂**装配器（``primary``
假设：一台机器人、一个 model instance、7 个关节）。R005 要求「两臂共享同一个
:class:`MultibodyPlant`，臂间碰撞 / 自碰撞 / 相邻体过滤在**同一次碰撞查询**中统一
判定」，因此这里需要一个**多机器人**的装配入口。

Drake 的硬约束（实测钉死，本模块的装配顺序不可改）
---------------------------------------------------
1. 同一 plant 里**同一份 URDF 只能装一次**：第二次
   ``parser.AddModels(url=...)`` 抛 ``RuntimeError: model instance names must be
   unique``（Drake 1.51.1 / 1.55.0 两档同构，``AddModels`` 没有 ``simple_name``
   参数）。→ 双臂必须用**每臂一份重命名 URDF 变体**
   （:mod:`rm75_drake.bimanual.model_variants` 生成的
   ``models/rm75_drake/rm{75,g24}_{left,right}.urdf``）。
2. ``parser.AddModels(url=...)`` 的 URL 形式**必须是关键字**（位置参数会被当成
   相对 cwd 的文件名）。
3. ``plant.WeldFrames(...)``（基座 / 夹爪挂载）**必须在 ``Build()`` 之前**：
   ``Build()`` 之后 plant 已 Finalize，再 weld 会抛异常。
4. 障碍物几何必须在 ``Build()`` 之前注册**并显式分配 proximity 角色**，
   否则碰撞检查**看不见**它（S02 实测坑）。
5. ``robot_diagram.plant_context(robot_diagram.CreateDefaultContext())``
   ——**必须传 root context**，无参调用会 ``TypeError``。
6. ``CollisionCheckerParams.model`` **只接受 ``RobotDiagram``**：传裸
   ``MultibodyPlant`` 会抛 pybind 的 ``Unable to cast ...``（统一走
   :func:`rm75_drake.drake_compat.make_collision_checker`）。

q 维度硬约束（防 abort）
------------------------
``CheckConfigCollisionFree`` / ``CheckConfigsCollisionFree`` 内部要求
``q.size() == plant.num_positions()``，维度不符**直接进程 abort**（不是可捕获的
异常）。双臂（无夹爪）是 **14**，双臂 + 双夹爪是 **18**。因此本模块提供
:func:`check_config_free` 包装：先校验维度，不符 → 抛
:class:`~rm75_drake.planning.scene.SceneError`（点名期望值与实际值），
**绝不让错误传导到 abort**。

夹爪语义
--------
* 夹爪是**独立 model instance**（``rmg24_{left,right}.urdf`` 变体）；
* 必须**显式 weld 到各自机械臂的法兰 link**（默认取 scenario 声明的最后一个
  关节的 child link），否则夹爪是浮动基座 → 每个浮动基座额外贡献
  **7 个位置变量**（四元数 qw/qx/qy/qz + x/y/z），双臂 + 双浮动夹爪会把
  ``num_positions`` 抬到 **32**（而不是期望的 18）；
* 夹爪的 4 个 prismatic 自由度**要么纳入 q、要么把夹爪从
  ``robot_model_instances`` 排除**（否则几何不随 q 更新，检不出「手指插进物体」）。
  本模块用 ``include_grippers_in_collision`` 显式表达这个选择。
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

from .. import drake_compat, package_map
from ..planning.scene import OBSTACLE_SOURCE_NAME, SceneError

__all__ = [
    "SceneError",
    "DualSceneBundle",
    "build_dual_scene",
    "build_dual_scene_from_path",
    "check_config_free",
    "check_configs_free",
    "default_gripper_urdfs",
    "GRIPPER_TCP_SUFFIX",
]

#: 夹爪 TCP link 的名字后缀（变体里是 ``left_tcp`` / ``right_tcp``）。
GRIPPER_TCP_SUFFIX = "_tcp"


@dataclass
class DualSceneBundle:
    """一次双臂场景装配的全部产物。

    字段
    ----
    robot_diagram:
        ``RobotDiagram``（``CollisionCheckerParams.model`` 只接受它）。
    plant:
        已 Finalize 的 :class:`MultibodyPlant`（两臂 + 可选两夹爪，**同一个 plant**）。
    scene_graph:
        :class:`SceneGraph`（机器人几何 + 障碍物几何）。
    context:
        plant 的上下文（零位配置）。
    checker:
        :class:`SceneGraphCollisionChecker`（臂间 / 自碰撞在**同一次查询**里判定）。
    model_instances:
        机械臂的 model instance 索引（顺序 = ``scenario.robots`` 顺序）。
    gripper_model_instances:
        夹爪的 model instance 索引（未装夹爪时为空元组）。
    joint_names:
        **有序**机械臂关节名（左 7 + 右 7；顺序 = ``scenario.robots`` 顺序 ×
        各自 ``joint_names`` 顺序）。
    gripper_joint_names:
        夹爪的可动关节名（每个夹爪 2 个 prismatic）。
    base_links:
        机器人名 → 基座 link 名（来自 scenario，**不得**用全局 ``"base_link"``）。
    tip_links:
        机器人名 → 法兰 link 名（机械臂最后一个关节的 child link）。
    tcp_frames:
        机器人名 → 夹爪 TCP frame 名（未装夹爪时为空映射）。
    num_positions:
        plant 的位置自由度总数（双臂 14；含双夹爪 18）。
    """

    robot_diagram: Any
    plant: Any
    scene_graph: Any
    context: Any
    checker: Any
    model_instances: Tuple[Any, ...]
    joint_names: Tuple[str, ...]
    base_links: Mapping[str, str]
    robot_names: Tuple[str, ...]
    tip_links: Mapping[str, str] = field(default_factory=dict)
    gripper_model_instances: Tuple[Any, ...] = ()
    gripper_joint_names: Tuple[str, ...] = ()
    tcp_frames: Mapping[str, str] = field(default_factory=dict)
    source_id: Any = None
    obstacle_geometry_ids: Tuple[Any, ...] = ()
    urdf_paths: Tuple[str, ...] = ()
    gripper_urdf_paths: Tuple[str, ...] = ()

    @property
    def num_positions(self) -> int:
        """plant 的位置自由度总数（q 的合法长度）。"""
        return int(self.plant.num_positions())


# ── URDF 文本辅助（零 Drake 依赖，可离线复用）────────────────────────────
def _urdf_root_link(urdf_path: Path) -> str:
    """返回 URDF 里**没有父关节**的那个 link 名（= 模型的根 link）。

    夹爪的挂载基座用它识别（自研 ``rmg24_*`` 的根 link 是 ``*_gripper_base``）。
    """
    root = ET.parse(str(urdf_path)).getroot()
    children = {
        (joint.find("child").get("link") if joint.find("child") is not None else None)
        for joint in root.findall("joint")
    }
    roots = [link.get("name") for link in root.findall("link") if link.get("name") not in children]
    if len(roots) != 1:
        raise SceneError(
            f"URDF {urdf_path} 的根 link 不唯一：{roots}；无法确定挂载基座"
        )
    return roots[0]


def _urdf_tcp_link(urdf_path: Path) -> Optional[str]:
    """返回夹爪 URDF 里的 TCP link 名（名字以 ``_tcp`` 结尾的那个 link）。

    自研 ``rmg24_*`` 的 TCP 是挂在本体上的固定关节（``<link name="*_tcp"/>``），
    由 T01 的变体生成器统一加了侧别前缀。
    """
    root = ET.parse(str(urdf_path)).getroot()
    candidates = [
        link.get("name")
        for link in root.findall("link")
        if link.get("name", "").endswith(GRIPPER_TCP_SUFFIX)
    ]
    if len(candidates) != 1:
        return None
    return candidates[0]


def _urdf_joint_child_link(urdf_path: Path, joint_name: str) -> str:
    """返回指定关节的 child link 名（机械臂法兰 = 最后一个关节的 child）。"""
    root = ET.parse(str(urdf_path)).getroot()
    for joint in root.findall("joint"):
        if joint.get("name") == joint_name:
            child = joint.find("child")
            if child is None:
                break
            return child.get("link")
    raise SceneError(
        f"URDF {urdf_path} 里没有关节 {joint_name!r}（无法确定末端 link）"
    )


def default_gripper_urdfs(robot_names: Sequence[str]) -> Dict[str, str]:
    """按「机器人名 = 侧别名」的约定给出默认夹爪 URL。

    ``left`` → ``package://rm75_drake/rmg24_left.urdf``，
    ``right`` → ``package://rm75_drake/rmg24_right.urdf``。

    **仅是约定**（与 T01 的变体命名规则一致）：侧别名不是 ``left`` / ``right``
    时必须显式传 ``gripper_urdfs``。
    """
    out: Dict[str, str] = {}
    for name in robot_names:
        if name not in ("left", "right"):
            raise SceneError(
                f"机器人名 {name!r} 不是侧别名（left/right），"
                "无法推断夹爪 URDF；请显式传 gripper_urdfs"
            )
        out[name] = f"package://rm75_drake/rmg24_{name}.urdf"
    return out


# ── 装配 ────────────────────────────────────────────────────────────────
def _extrinsic_transform(extrinsic):
    from pydrake.math import RigidTransform

    rotation = drake_compat.rotation_from_rpy(extrinsic.rpy)
    return RigidTransform(rotation, extrinsic.xyz)


def _obstacle_geometry(obstacle):
    from pydrake.geometry import Box
    from pydrake.math import RigidTransform

    rotation = drake_compat.rotation_from_rpy(obstacle.rpy)
    pose = RigidTransform(rotation, obstacle.xyz)
    return Box(*obstacle.half_size), pose


#: 贡献位置自由度的关节类型（其余类型：weld / quaternion_floating / fixed）。
_MOVABLE_JOINT_TYPES = ("revolute", "continuous", "prismatic")


def _instance_joint_names(plant, model_instance) -> List[str]:
    """返回某个 model instance 的**可动关节名**（按 plant 的关节顺序）。

    用 **plant 的关节表**而不是位置变量名反推：Drake 的位置变量名后缀随关节
    类型变化（revolute → ``_q``、prismatic → ``_x``、quaternion_floating →
    ``_qw/_qx/_qy/_qz`` + ``_x/_y/_z``），靠剥后缀反推关节名不可靠。
    """
    out: List[str] = []
    for index in plant.GetJointIndices():
        joint = plant.get_joint(index)
        if int(joint.model_instance()) != int(model_instance):
            continue
        if joint.type_name() not in _MOVABLE_JOINT_TYPES:
            continue
        out.append(joint.name())
    return out


def build_dual_scene(
    scenario,
    *,
    edge_step_size: float = 0.01,
    gripper_urdfs: Optional[Mapping[str, str]] = None,
    include_grippers_in_collision: bool = False,
    adjacent_bodies_collision_filters: bool = True,
) -> DualSceneBundle:
    """把双臂场景装配成**同一个 plant**。

    参数
    ----
    scenario:
        场景真相源（:func:`rm75_drake.scenario.load_scenario` 的产物）。
    edge_step_size:
        碰撞检查器的边采样步长（m），必须 > 0。
    gripper_urdfs:
        机器人名 → 夹爪 ``package://`` URL。``None`` = **不装夹爪**
        （``num_positions() == 14``）。给了就装（``== 18``）。
    include_grippers_in_collision:
        夹爪是否进入碰撞检查器的 ``robot_model_instances``。
        默认 ``False``（夹爪排除在碰撞判定外）。置 ``True`` 时 **q 必须包含
        夹爪的 4 个 prismatic 自由度**（否则几何不随 q 更新）。
    adjacent_bodies_collision_filters:
        **同臂父子相邻体过滤**开关（Drake 的
        ``plant.set_adjacent_bodies_collision_filters``）。Drake 只允许在
        ``Finalize()`` **之前**设置，因此只能在这里（装配期）给。
        默认 ``True``（保留 Drake 默认：相邻体不过滤会误报）。

    装配顺序（**固定**，违反会出问题）
    ----------------------------------
    1. ``RobotDiagramBuilder`` → ``plant()`` / ``scene_graph()`` / ``parser()``；
    2. ``package_map.build_package_map()`` + ``attach_package_map(parser, pkg_map)``；
    3. ``parser.AddModels(url=...)``（两臂 + 可选两夹爪）；
    4. ``plant.WeldFrames(world, base_link, X)``（各自外参）+ 夹爪 weld 到法兰
       ——**全部必须在 ``Build()`` 之前**；
    5. 注册障碍物几何 + **proximity 角色**（必须在 ``Build()`` 之前）；
    6. ``builder.Build()`` → ``robot_diagram``；
    7. ``robot_diagram.plant_context(robot_diagram.CreateDefaultContext())``；
    8. 构造碰撞检查器（``params.model`` 必须是 ``RobotDiagram``）。
    """
    from pydrake.geometry import GeometryInstance, ProximityProperties
    from pydrake.planning import RobotDiagramBuilder

    if not scenario.robots:
        raise SceneError("场景里没有机器人，无法装配")
    if len(scenario.robots) < 2:
        raise SceneError(
            f"双臂场景至少需要两台机器人，收到 {len(scenario.robots)} 台："
            f"{[r.name for r in scenario.robots]}"
        )

    builder = RobotDiagramBuilder()
    plant = builder.plant()
    scene_graph = builder.scene_graph()
    parser = builder.parser()

    pkg_map = package_map.build_package_map()
    package_map.attach_package_map(parser, pkg_map)

    model_instances: List[Any] = []
    urdf_paths: List[str] = []
    urdf_files: List[Path] = []
    for spec in scenario.robots:
        # url= 必须是关键字（位置参数会被 Drake 当成相对 cwd 的文件名）。
        indices = parser.AddModels(url=spec.urdf)
        if not indices:
            raise SceneError(f"解析 {spec.urdf} 没有得到任何 model instance")
        model_instances.append(indices[0])
        urdf_paths.append(package_map.resolve_package_url(pkg_map, spec.urdf))
        urdf_files.append(Path(urdf_paths[-1]))

    # ── 夹爪（可选）：独立 model instance，必须显式 weld 到各自法兰 ──
    gripper_model_instances: List[Any] = []
    gripper_joint_names: List[str] = []
    gripper_urdf_paths: List[str] = []
    tip_links: Dict[str, str] = {}
    tcp_frames: Dict[str, str] = {}
    gripper_base_links: Dict[str, str] = {}

    for spec, urdf_file in zip(scenario.robots, urdf_files):
        if not spec.joint_names:
            raise SceneError(f"机器人 {spec.name!r} 没有声明关节名，无法确定末端 link")
        tip_links[spec.name] = _urdf_joint_child_link(urdf_file, spec.joint_names[-1])

    if gripper_urdfs:
        for spec in scenario.robots:
            url = gripper_urdfs.get(spec.name)
            if url is None:
                raise SceneError(
                    f"gripper_urdfs 缺少机器人 {spec.name!r} 的夹爪 URL"
                    f"（已给出：{sorted(gripper_urdfs)}）"
                )
            indices = parser.AddModels(url=url)
            if not indices:
                raise SceneError(f"解析夹爪 {url} 没有得到任何 model instance")
            gripper_model_instances.append(indices[0])
            gripper_urdf_paths.append(package_map.resolve_package_url(pkg_map, url))
            gripper_file = Path(gripper_urdf_paths[-1])
            base_link = _urdf_root_link(gripper_file)
            gripper_base_links[spec.name] = base_link
            mount = tip_links[spec.name]
            if not plant.HasFrameNamed(base_link):
                raise SceneError(
                    f"夹爪 {url} 里没有名为 {base_link!r} 的 link，无法挂载到 {spec.name!r}"
                )
            if not plant.HasFrameNamed(mount):
                raise SceneError(
                    f"机器人 {spec.name!r} 没有名为 {mount!r} 的法兰 link，无法挂载夹爪"
                )
            # 夹爪基座 weld 到机械臂法兰（identity，与 models.load_rm75_gripper 同语义）。
            plant.WeldFrames(
                plant.GetFrameByName(mount),
                plant.GetFrameByName(base_link),
            )
            # TCP frame：取夹爪 URDF 里唯一的 "_tcp" link（变体已加侧别前缀）。
            tcp_name = _urdf_tcp_link(gripper_file)
            if tcp_name and plant.HasFrameNamed(tcp_name):
                tcp_frames[spec.name] = tcp_name

    # ── 固定基座：必须在 Build() 之前（build 之后 plant 已 Finalize）──
    base_links: Dict[str, str] = {}
    for spec in scenario.robots:
        if not plant.HasFrameNamed(spec.base_link):
            raise SceneError(
                f"机器人 {spec.name!r} 的模型里没有名为 {spec.base_link!r} 的 frame，无法固定基座"
            )
        extrinsic = scenario.base_extrinsics.get(spec.name)
        if extrinsic is None:
            raise SceneError(f"场景缺少机器人 {spec.name!r} 的基座外参")
        base_links[spec.name] = spec.base_link
        plant.WeldFrames(
            plant.world_frame(),
            plant.GetFrameByName(spec.base_link),
            _extrinsic_transform(extrinsic),
        )

    # ── 相邻体过滤：只能在 Build() 之前设置（post-finalize 会抛）──
    plant.set_adjacent_bodies_collision_filters(bool(adjacent_bodies_collision_filters))

    # ── 障碍物：静态几何挂在世界系上（anchored），并显式分配 proximity 角色 ──
    source_id = scene_graph.RegisterSource(OBSTACLE_SOURCE_NAME)
    geometry_ids: List[Any] = []
    for obstacle in scenario.obstacles:
        shape, pose = _obstacle_geometry(obstacle)
        instance = GeometryInstance(pose, shape, obstacle.name)
        geometry_id = scene_graph.RegisterAnchoredGeometry(source_id, instance)
        scene_graph.AssignRole(source_id, geometry_id, ProximityProperties())
        geometry_ids.append(geometry_id)

    robot_diagram = builder.Build()
    context = robot_diagram.plant_context(robot_diagram.CreateDefaultContext())

    # ── 关节名校验：plant 解析出的每台机器人关节序列必须与 scenario 一致 ──
    joint_names: List[str] = []
    for spec, instance in zip(scenario.robots, model_instances):
        plant_joints = _instance_joint_names(plant, instance)
        if tuple(plant_joints) != tuple(spec.joint_names):
            raise SceneError(
                f"机器人 {spec.name!r} 的关节顺序与 scenario 不一致："
                f"plant 解析出 {plant_joints}，scenario 声明 {list(spec.joint_names)}"
            )
        joint_names.extend(spec.joint_names)

    for instance in gripper_model_instances:
        gripper_joint_names.extend(_instance_joint_names(plant, instance))

    checker_instances = list(model_instances)
    if include_grippers_in_collision:
        checker_instances.extend(gripper_model_instances)

    checker = drake_compat.make_collision_checker(
        robot_model_instances=checker_instances,
        robot_diagram=robot_diagram,
        edge_step_size=edge_step_size,
    )

    return DualSceneBundle(
        robot_diagram=robot_diagram,
        plant=plant,
        scene_graph=scene_graph,
        context=context,
        checker=checker,
        model_instances=tuple(model_instances),
        joint_names=tuple(joint_names),
        base_links=base_links,
        robot_names=tuple(r.name for r in scenario.robots),
        tip_links=tip_links,
        gripper_model_instances=tuple(gripper_model_instances),
        gripper_joint_names=tuple(gripper_joint_names),
        tcp_frames=tcp_frames,
        source_id=source_id,
        obstacle_geometry_ids=tuple(geometry_ids),
        urdf_paths=tuple(urdf_paths),
        gripper_urdf_paths=tuple(gripper_urdf_paths),
    )


def build_dual_scene_from_path(path, **kwargs) -> DualSceneBundle:
    """便捷入口：读 YAML → 装配双臂场景。"""
    from ..scenario import load_scenario

    return build_dual_scene(load_scenario(path), **kwargs)


# ── 碰撞查询（q 维度硬约束在此收口）────────────────────────────────────
def _as_q(q) -> Any:
    import numpy as np

    array = np.asarray(q, dtype=float).reshape(-1)
    return array


def check_config_free(bundle: DualSceneBundle, q) -> bool:
    """判断一个位形是否**碰撞自由**（臂间 / 自碰撞在同一次查询里判定）。

    维度不符时抛 :class:`SceneError`（点名期望值与实际值）——
    **绝不让维度错误传导到 Drake 的 abort**（``CheckConfigCollisionFree``
    内部要求 ``q.size() == plant.num_positions()``，不符直接杀进程）。
    """
    array = _as_q(q)
    expected = bundle.num_positions
    if array.size != expected:
        raise SceneError(
            f"q 维度不符：期望 {expected}（plant.num_positions()），实际 {array.size}；"
            "位形向量长度必须与 plant 的位置自由度严格一致"
        )
    # 同步 plant 上下文（后续读末端位姿时与本次判定一致）。
    bundle.plant.SetPositions(bundle.context, array)
    return bool(bundle.checker.CheckConfigCollisionFree(array))


def check_configs_free(bundle: DualSceneBundle, configs) -> List[bool]:
    """批量判定（逐个位形，维度校验同 :func:`check_config_free`）。"""
    import numpy as np

    array = np.asarray(configs, dtype=float)
    if array.ndim == 1:
        array = array.reshape(1, -1)
    return [check_config_free(bundle, row) for row in array]
