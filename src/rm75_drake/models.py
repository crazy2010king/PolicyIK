"""模型装配层（S01-T02）。

职责边界
--------
* **模型真值**（URDF 文本）放在 ``models/rm75_drake/``，入 git，零外部资产依赖；
* 本模块只做**装配**：把 URDF 解析进 :class:`~pydrake.multibody.plant.MultibodyPlant`，
  在 ``Finalize()`` 之前固定基座，并把 Drake 版本差异收敛到
  :mod:`rm75_drake.drake_compat`。

装配顺序（**固定**，违反会出问题）
------------------------------------
1. ``MultibodyPlant(0.0)``（若调用方自带 plant 则沿用）；
2. 可选：``plant.RegisterAsSourceForSceneGraph(scene_graph)``——**必须早于
   ``Finalize()``**，否则抛 ``RuntimeError``；
3. ``package_map.make_parser(plant)`` → ``parser.AddModels(url=...)``
   （``url=`` 必须是**关键字**；位置参数会被当成相对 cwd 的文件名）；
4. ``plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base_link"))``
   ——**必须早于 ``Finalize()``**，否则抛异常；
5. 调用方自行 ``plant.Finalize()``。

反模式（项目既有约定，见 :mod:`rm75_drake.package_map`）
--------------------------------------------------------
* **不得**在解析前调用 ``plant.AddModelInstance(...)`` 预留 model instance 槽位：
  那会把 ``num_model_instances()`` 抬到大于 world，Drake 的
  ``CollisionFilterGroupResolver`` 随之**确定性 abort**。model instance 的分配
  交还 Drake Parser 管理。
* **不得**在 URDF 里写 ``parent link="world"``：基座固定一律用 ``WeldFrames``。
* **不得**在库内硬编码绝对路径：模型路径走
  :func:`rm75_drake.package_map.build_package_map`（默认取
  :func:`rm75_drake.paths.model_search_paths`），URDF 用 ``package://`` URL 引用。

版本差异
--------
位姿求值统一走 :func:`rm75_drake.drake_compat.eval_body_pose_in_world`
（1.51/1.55 是 ``EvalBodyPoseInWorld``，1.58 起改名 ``EvalPoseInWorld``）。
"""

from __future__ import annotations

from typing import Any, List, Optional, Tuple

from . import drake_compat, package_map

__all__ = [
    "RM75_PACKAGE_NAME",
    "RM75_MODEL_NAME",
    "RM75_URDF_URL",
    "GRIPPER_URDF_URL",
    "RM75_LINK_NAMES",
    "RM75_JOINT_NAMES",
    "RM75_BASE_LINK",
    "RM75_TIP_LINK",
    "GRIPPER_BASE_LINK",
    "GRIPPER_TCP_LINK",
    "GRIPPER_JOINT_NAMES",
    "make_plant",
    "attach_scene_graph",
    "load_rm75",
    "load_rm75_gripper",
    "eval_link_pose_in_world",
    "joint_limits_from_plant",
    "joint_limits_from_urdf",
]

#: PackageMap key（与 models/rm75_drake/package.xml 的 <name> 一致）。
RM75_PACKAGE_NAME = "rm75_drake"

#: 机器人名（Drake 生成的几何名形如 "RM75-B::link_1_collision"）。
RM75_MODEL_NAME = "RM75-B"

#: 改造版 RM75-B URDF 的 ``package://`` URL。
RM75_URDF_URL = "package://rm75_drake/rm75_drake.urdf"

#: 自研两指夹爪 URDF 的 ``package://`` URL（S01-T04 落地）。
GRIPPER_URDF_URL = "package://rm75_drake/rmg24_drake.urdf"

#: 基座 link（被 weld 到 world）。
RM75_BASE_LINK = "base_link"

#: 末端 link（法兰）。
RM75_TIP_LINK = "link_7"

#: 夹爪本体 link（自研 URDF；**不叫** base_link，避免与 RM75-B 同 plant 时 frame 重名）。
GRIPPER_BASE_LINK = "gripper_base"

#: 夹爪 TCP link（两指中点、指尖高度）。
GRIPPER_TCP_LINK = "tcp"

#: 夹爪两个开合关节名（顺序 = 左、右）。
GRIPPER_JOINT_NAMES: Tuple[str, ...] = ("finger_joint_left", "finger_joint_right")

#: 全部 link 名（顺序 = 运动链顺序）。
RM75_LINK_NAMES: Tuple[str, ...] = (
    "base_link",
    "link_1",
    "link_2",
    "link_3",
    "link_4",
    "link_5",
    "link_6",
    "link_7",
)

#: 全部关节名（顺序 = 关节序号）。
RM75_JOINT_NAMES: Tuple[str, ...] = tuple(f"joint_{i}" for i in range(1, 8))


def make_plant(time_step: float = 0.0):
    """创建空的 :class:`MultibodyPlant`（连续时间，零步长 = 纯模型容器）。"""
    from pydrake.multibody.plant import MultibodyPlant

    return MultibodyPlant(time_step)


def attach_scene_graph(plant, scene_graph):
    """把 plant 注册为 scene graph 的 source，返回 ``SourceId``。

    **必须在 ``Finalize()`` 之前**调用，否则 Drake 抛 ``RuntimeError``；
    同一 plant 只能注册一次。
    """
    return plant.RegisterAsSourceForSceneGraph(scene_graph)


def _parse_url(parser, url: str):
    """用 ``package://`` URL 解析模型，返回 model instance 索引列表。"""
    # url= 必须是关键字（位置参数会被 Drake 当成普通文件名，相对 cwd 拼接）。
    return parser.AddModels(url=url)


def _weld_base(plant, link_name: str = RM75_BASE_LINK) -> None:
    """把基座 link 焊接到 world（**必须在 Finalize() 之前**）。"""
    if not plant.HasFrameNamed(link_name):
        raise RuntimeError(f"模型里没有名为 {link_name!r} 的 frame，无法固定基座")
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName(link_name))


def load_rm75(
    plant=None,
    weld: bool = True,
    scene_graph=None,
) -> Tuple[Any, Any, Any]:
    """把改造版 RM75-B 装配进 plant，返回 ``(plant, model_instance_index, parser)``。

    参数
    ----
    plant:
        目标 :class:`MultibodyPlant`；``None`` 时新建一个（零步长）。
    weld:
        是否把基座 ``base_link`` 焊接到 world。默认 ``True``（固定基座，
        DoF=7）；``False`` 时基座自由（DoF=14，仅作对照用）。
    scene_graph:
        可选的 :class:`pydrake.geometry.SceneGraph`；给出时在 ``Finalize()``
        之前把 plant 注册为 source（供碰撞过滤 / 几何查询用）。

    **不**调用 ``plant.Finalize()``——由调用方决定何时定稿（weld 必须在定稿前）。
    """
    if plant is None:
        plant = make_plant()
    if scene_graph is not None:
        attach_scene_graph(plant, scene_graph)

    parser, _pkg_map = package_map.make_parser(plant)
    indices = _parse_url(parser, RM75_URDF_URL)
    if not indices:
        raise RuntimeError(f"解析 {RM75_URDF_URL} 没有得到任何 model instance")
    if weld:
        _weld_base(plant, RM75_BASE_LINK)
    return plant, indices[0], parser


def load_rm75_gripper(
    plant=None,
    weld: bool = True,
    scene_graph=None,
    base_link: str = GRIPPER_BASE_LINK,
    attach_to: Optional[str] = None,
) -> Tuple[Any, Any, Any]:
    """把自研两指夹爪（``rmg24_drake.urdf``）装配进 plant。

    与 :func:`load_rm75` 同构。夹爪本体 link 名为 ``gripper_base``
    （**不叫** ``base_link``）：夹爪与 RM75-B 常被装进**同一个 plant**，
    frame 名必须全局唯一。

    参数
    ----
    weld:
        是否把夹爪本体固定住。默认 ``True``：夹爪基座若自由浮动，Drake 会给它
        6 个自由度 + 2 个四元数（``num_positions`` 从 2 变成 9），与
        “夹爪恰有 2 个 prismatic 位置自由度”的模型契约冲突。
    attach_to:
        指定夹爪挂载的 frame 名（例如机械臂法兰 ``"link_7"``），
        此时夹爪本体被 weld 到该 frame（**优先于** ``weld``），
        可与机械臂同 plant 装配成一条运动链。

    参数 ``base_link`` 可覆盖可焊基座 link 名（默认 :data:`GRIPPER_BASE_LINK`）。
    """
    if plant is None:
        plant = make_plant()
    if scene_graph is not None:
        attach_scene_graph(plant, scene_graph)

    parser, _pkg_map = package_map.make_parser(plant)
    indices = _parse_url(parser, GRIPPER_URDF_URL)
    if not indices:
        raise RuntimeError(f"解析 {GRIPPER_URDF_URL} 没有得到任何 model instance")
    if attach_to is not None:
        if not plant.HasFrameNamed(attach_to):
            raise RuntimeError(f"挂载目标 frame {attach_to!r} 不存在，无法装配夹爪")
        plant.WeldFrames(
            plant.GetFrameByName(attach_to), plant.GetFrameByName(base_link)
        )
    elif weld:
        _weld_base(plant, base_link)
    return plant, indices[0], parser


def eval_link_pose_in_world(plant, context, link_name: str):
    """返回 link 原点在世界系下的位姿（``X_WLink``）。

    统一走 :func:`rm75_drake.drake_compat.eval_body_pose_in_world`，不直接调用
    ``plant.EvalBodyPoseInWorld``（避免 Drake 版本差异外溢）。
    """
    body = plant.GetBodyByName(link_name)
    return drake_compat.eval_body_pose_in_world(context, body, plant=plant)


def joint_limits_from_plant(plant) -> List[dict]:
    """按关节序号返回 **Drake 解析后** 的限位摘要。

    返回的每项形如::

        {"name": "joint_1", "lower": -3.106, "upper": 3.106,
         "velocity": 3.1416}

    数值取自 Drake 解析后的关节限位（不是重新解析 XML），因此可以直接当
    “解析结果真值”用于回归断言。

    **effort 不在这里**：Drake 1.51/1.55 不把 URDF 的 ``<limit effort=...>``
    暴露到 joint/plant API（``plant.GetEffortUpperLimits()`` 返回空向量），
    effort 请用 :func:`joint_limits_from_urdf` 直接从 URDF 读。
    """
    out: List[dict] = []
    for index in plant.GetJointIndices():
        joint = plant.get_joint(index)
        # weld 关节（world–base_link）没有位置限位，直接跳过：
        # GetJointIndices() 同时包含 weld 关节与可动关节。
        if not hasattr(joint, "position_lower_limit"):
            continue
        out.append(
            {
                "name": joint.name(),
                "lower": float(joint.position_lower_limit()),
                "upper": float(joint.position_upper_limit()),
                "velocity": float(joint.velocity_upper_limit()),
            }
        )
    return out


def joint_limits_from_urdf(url: str = RM75_URDF_URL) -> List[dict]:
    """从 URDF XML 直接读限位（含 ``effort``）。

    与 :func:`joint_limits_from_plant` 的区别：这里读的是**文件里的声明值**
    （未经 Drake 变换），因此包含 ``effort``；用于交叉核对与交接件生成。
    """
    import xml.etree.ElementTree as ET

    pkg_map = package_map.build_package_map()
    path = package_map.resolve_package_url(pkg_map, url)
    root = ET.parse(path).getroot()
    out: List[dict] = []
    for joint in root.findall("joint"):
        limit = joint.find("limit")
        if limit is None:
            continue
        out.append(
            {
                "name": joint.get("name"),
                "type": joint.get("type"),
                "lower": float(limit.get("lower")),
                "upper": float(limit.get("upper")),
                "effort": float(limit.get("effort")),
                "velocity": float(limit.get("velocity")),
            }
        )
    return out
