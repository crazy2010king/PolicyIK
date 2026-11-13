"""场景装配层（S02-T01）：scenario → Drake 场景。

职责
----
把 :class:`rm75_drake.scenario.Scenario`（场景真相源）装配成可直接用于
IK / IRIS / GCS / Toppra 的 Drake 场景：

* ``plant`` + ``scene_graph``（含障碍物几何）
* ``plant_context``（默认上下文，已含机器人与障碍物位姿）
* ``checker``（:class:`pydrake.planning.SceneGraphCollisionChecker`）
* ``joint_names``（**有序**，来自 scenario，与规划链路的变量顺序一致）

Drake 版本差异（**本机 Drake 1.51.1 实测**，1.55.0 同构）
--------------------------------------------------------
1. ``RobotDiagramBuilder`` 自带 parser（**不要**用 ``make_parser(plant)``）；
2. ``parser.AddModels(url=...)`` 的 URL 形式**必须是关键字**；
3. 固定基座 ``plant.WeldFrames(...)`` **必须在 ``Build()`` 之前**
   （``Build()`` 之后 plant 已 Finalize，再 weld 会抛异常）；
4. ``robot_diagram.plant_context(robot_diagram.CreateDefaultContext())``
   ——**必须传 root context**，无参调用会 ``TypeError``；
5. ``pydrake.geometry.GeometryInstance`` 在 1.51/1.55 的 pybind 里**只有三参重载**
   ``(pose, shape, name)``（两参会 ``TypeError``）；
   ``RigidTransform`` 从 ``pydrake.math`` 导入（**不在** ``pydrake.geometry``）；
6. ``SceneGraph`` 在 1.51/1.55 **没有** ``AddGeometry``：静态障碍物走
   ``RegisterSource`` + ``RegisterAnchoredGeometry(source_id, GeometryInstance)``；
7. **必须显式给障碍物分配 proximity 角色**
   （``AssignRole(source_id, geometry_id, ProximityProperties())``），
   否则几何虽然进了 scene graph，但**碰撞检查完全看不见它**
   （实测：机器人末端正好穿过障碍物时 ``CheckConfigCollisionFree`` 仍返回 True）；
8. ``CollisionCheckerParams.model`` 只接受 ``RobotDiagram``：传 ``MultibodyPlant``
   或 ``ModelInstanceIndex`` 会抛 pybind 的 ``Unable to cast ... to C++ type '?'``；
   碰撞检查器构造器在 1.51/1.55 **只接受** ``CollisionCheckerParams`` 一个参数。
   这些差异统一收敛在 :mod:`rm75_drake.drake_compat`，本模块不直接碰版本分支。
9. ``pydrake.math.RotationMatrix`` **没有**静态方法 ``RPY(...)``（1.51/1.55 实测）：
   只能 ``RotationMatrix(RollPitchYaw(roll, pitch, yaw))``。统一走
   :func:`rm75_drake.drake_compat.rotation_from_rpy`。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, List, Tuple

from .. import drake_compat, models, package_map

__all__ = [
    "SceneError",
    "SceneBundle",
    "build_scene",
    "build_scene_from_path",
    "OBSTACLE_SOURCE_NAME",
]

#: 障碍物在 scene graph 里的 source 名（便于在 Meshcat / 调试里辨认）。
OBSTACLE_SOURCE_NAME = "scenario_obstacles"


class SceneError(RuntimeError):
    """场景装配失败（模型缺 frame、关节不匹配、check 参数非法等）。"""


@dataclass
class SceneBundle:
    """一次场景装配的全部产物。

    字段
    ----
    robot_diagram:
        ``RobotDiagram``（``CollisionCheckerParams.model`` 只接受它）。
    plant:
        已 Finalize 的 :class:`MultibodyPlant`（DoF = 7，基座已焊到世界系）。
    scene_graph:
        :class:`SceneGraph`（机器人几何 + 障碍物几何）。
    plant_context:
        机器人 plant 的上下文（零位配置）。
    model_instance:
        机器人模型实例索引（规划链路的 robot model instance）。
    joint_names:
        **有序**关节名（= 规划变量顺序）。
    checker:
        :class:`SceneGraphCollisionChecker`。
    source_id:
        障碍物几何所在的 ``SourceId``。
    obstacle_geometry_ids:
        障碍物几何 id 列表（顺序 = ``scenario.obstacles`` 顺序）。
    urdf_paths:
        机器人 URDF 的**本地绝对路径**（顺序 = ``scenario.robots`` 顺序）。
        供 S02-T04 的 IRIS 区域缓存计算**模型版本标识**（URDF 内容 sha256），
        以便缓存文件能拒绝「模型换了还复用旧凸区域」。
    """

    robot_diagram: Any
    plant: Any
    scene_graph: Any
    plant_context: Any
    model_instance: Any
    joint_names: Tuple[str, ...]
    checker: Any
    source_id: Any
    obstacle_geometry_ids: Tuple[Any, ...] = ()
    model_instances: Tuple[Any, ...] = ()
    robot_name: str = ""
    urdf_paths: Tuple[str, ...] = ()


def _robot_joint_names(plant) -> List[str]:
    """返回 plant 里**可动**关节的有序名字（跳过 weld 关节）。"""
    return [item["name"] for item in models.joint_limits_from_plant(plant)]


def _obstacle_geometry(obstacle):
    """把 scenario 障碍物翻译成 Drake 几何（Box，半长由全长换算）。"""
    from pydrake.geometry import Box
    from pydrake.math import RigidTransform

    rotation = drake_compat.rotation_from_rpy(obstacle.rpy)
    pose = RigidTransform(rotation, obstacle.xyz)
    return Box(*obstacle.half_size), pose


def build_scene(scenario, edge_step_size: float = 0.01) -> SceneBundle:
    """把 :class:`~rm75_drake.scenario.Scenario` 装配成 Drake 场景。

    参数
    ----
    scenario:
        场景真相源（:func:`rm75_drake.scenario.load_scenario` 的产物）。
    edge_step_size:
        碰撞检查器的边采样步长（m），必须 > 0（Drake CHECK）。

    装配顺序（**固定**，违反会出问题）
    ----------------------------------
    1. ``RobotDiagramBuilder`` → ``plant()`` / ``scene_graph()`` / ``parser()``；
    2. 合并 ``package://`` 映射 → ``parser.AddModels(url=...)``；
    3. ``plant.WeldFrames(...)`` 固定基座（**必须在 Build() 之前**）；
    4. 注册障碍物几何 + **proximity 角色**（必须在 Build() 之前，
       否则 scene graph 模型变更会使已分配的上下文失效）；
    5. ``builder.Build()`` → ``robot_diagram``；
    6. ``robot_diagram.plant_context(robot_diagram.CreateDefaultContext())``；
    7. 构造碰撞检查器（``params.model`` 必须是 ``RobotDiagram``）。

    **不得**在解析前调用 ``plant.AddModelInstance(...)``（MEM021/MEM024：
    预预留 model instance 槽位会确定性触发 Drake 的 abort）。
    """
    from pydrake.geometry import GeometryInstance, ProximityProperties
    from pydrake.planning import RobotDiagramBuilder

    if not scenario.robots:
        raise SceneError("场景里没有机器人，无法装配")

    builder = RobotDiagramBuilder()
    plant = builder.plant()
    scene_graph = builder.scene_graph()
    parser = builder.parser()

    pkg_map = package_map.build_package_map()
    package_map.attach_package_map(parser, pkg_map)

    model_instances: List[Any] = []
    urdf_paths: List[str] = []
    for spec in scenario.robots:
        # url= 必须是关键字（位置参数会被 Drake 当成相对 cwd 的文件名）。
        indices = parser.AddModels(url=spec.urdf)
        if not indices:
            raise SceneError(f"解析 {spec.urdf} 没有得到任何 model instance")
        model_instances.append(indices[0])
        # 解析出本地路径（供 IRIS 区域缓存算模型版本标识）。
        urdf_paths.append(package_map.resolve_package_url(pkg_map, spec.urdf))

    # 固定基座：必须在 Build() 之前（build 之后 plant 已 Finalize）。
    for spec in scenario.robots:
        if not plant.HasFrameNamed(spec.base_link):
            raise SceneError(f"模型里没有名为 {spec.base_link!r} 的 frame，无法固定基座")
        extrinsic = scenario.base_extrinsics.get(spec.name)
        if extrinsic is None:
            raise SceneError(f"场景缺少机器人 {spec.name!r} 的基座外参")
        plant.WeldFrames(
            plant.world_frame(),
            plant.GetFrameByName(spec.base_link),
            _extrinsic_transform(extrinsic),
        )

    # 障碍物：静态几何挂在世界系上（anchored），并显式分配 proximity 角色。
    source_id = scene_graph.RegisterSource(OBSTACLE_SOURCE_NAME)
    geometry_ids: List[Any] = []
    for obstacle in scenario.obstacles:
        shape, pose = _obstacle_geometry(obstacle)
        instance = GeometryInstance(pose, shape, obstacle.name)
        geometry_id = scene_graph.RegisterAnchoredGeometry(source_id, instance)
        # 没有 proximity 角色 → 碰撞检查看不见这个几何（实测坑，见模块 docstring）。
        scene_graph.AssignRole(source_id, geometry_id, ProximityProperties())
        geometry_ids.append(geometry_id)

    robot_diagram = builder.Build()
    plant_context = robot_diagram.plant_context(robot_diagram.CreateDefaultContext())

    primary = scenario.robots[0]
    plant_joints = _robot_joint_names(plant)
    if tuple(plant_joints) != tuple(primary.joint_names):
        raise SceneError(
            f"关节顺序与 scenario 不一致：plant 解析出 {plant_joints}，"
            f"scenario 声明 {list(primary.joint_names)}"
        )

    checker = drake_compat.make_collision_checker(
        robot_diagram=robot_diagram,
        robot_model_instances=model_instances,
        edge_step_size=edge_step_size,
    )

    return SceneBundle(
        robot_diagram=robot_diagram,
        plant=plant,
        scene_graph=scene_graph,
        plant_context=plant_context,
        model_instance=model_instances[0],
        joint_names=tuple(primary.joint_names),
        checker=checker,
        source_id=source_id,
        obstacle_geometry_ids=tuple(geometry_ids),
        model_instances=tuple(model_instances),
        robot_name=primary.name,
        urdf_paths=tuple(urdf_paths),
    )


def _extrinsic_transform(extrinsic):
    """把 :class:`~rm75_drake.scenario.BaseExtrinsic` 转成 ``RigidTransform``。"""
    from pydrake.math import RigidTransform

    rotation = drake_compat.rotation_from_rpy(extrinsic.rpy)
    return RigidTransform(rotation, extrinsic.xyz)


def build_scene_from_path(path, edge_step_size: float = 0.01) -> SceneBundle:
    """便捷入口：读 YAML → 装配场景。"""
    from ..scenario import load_scenario

    return build_scene(load_scenario(path), edge_step_size=edge_step_size)
