"""scenario 正式 schema 与加载/校验（S02-T01）。

本模块是**全链路的场景真相源**：仿真驱动、假驱动（FakeDriver）、规划器与回放
复检都消费同一个 :class:`Scenario` 对象，保证「同一 scenario YAML 驱动仿真与真机，
切换只发生在驱动实现层」（R007）。

顶层四键（**S02 落地后不得随意改名**，改名会击穿 S07 的 golden-file 回归）::

    schema_version / robots / base_extrinsics / obstacles / calibration

单位约定（**全表 SI**，禁止在别处另立单位）
------------------------------------------------

========  ==========================================================
量纲       约定
========  ==========================================================
长度      m（米）。位置 ``xyz``、障碍物 ``size`` 一律 m。
角度      **rad（弧度）**。所有 ``rpy`` 一律弧度；**不接收度**。
时间      s（秒）。
夹爪开口  u ∈ [0, 1000]（无量纲指令单位）。
========  ==========================================================

校验原则（**禁止静默默认值**）
--------------------------------
* 未知顶层键 / 未知子键 → :class:`ScenarioError`，异常信息里**列出该键**；
* 缺必填字段 → :class:`ScenarioError`；
* 数值维度不对（xyz/rpy 不是 3 维、障碍物 size 不是 3 个正数）→ :class:`ScenarioError`；
* 单位明显写错（rpy 写成角度制）→ :class:`ScenarioError`，并给出换算提示。

`:class:`ScenarioError` 继承 :class:`ValueError`，调用方既可以按具体类型捕获，
也可以用 ``except ValueError`` 兜底。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

__all__ = [
    "ScenarioError",
    "SCHEMA_VERSION",
    "TOP_LEVEL_KEYS",
    "RobotSpec",
    "BaseExtrinsic",
    "Obstacle",
    "Scenario",
    "load_scenario",
    "load_scenario_dict",
    "scenario_from_dict",
]

#: 当前正式 schema 版本（S02 从占位版的 0 提升到 1）。
SCHEMA_VERSION = 1

#: 顶层键（顺序即 YAML 里的书写顺序）。
TOP_LEVEL_KEYS: Tuple[str, ...] = (
    "schema_version",
    "robots",
    "base_extrinsics",
    "obstacles",
    "calibration",
)

#: 机器人条目的合法子键（全部必填）。
_ROBOT_KEYS: Tuple[str, ...] = ("name", "urdf", "base_link", "joint_names")

#: 基座外参条目的合法子键（全部必填）。
_EXTRINSIC_KEYS: Tuple[str, ...] = ("xyz", "rpy")

#: 障碍物条目的合法子键（全部必填）。
_OBSTACLE_KEYS: Tuple[str, ...] = ("name", "type", "size", "pose")

#: 位姿条目的合法子键（全部必填）。
_POSE_KEYS: Tuple[str, ...] = ("xyz", "rpy")

#: 标定条目的合法子键（可整体为空映射）。
#:
#: ``base_extrinsic``（S04-T05 新增，**可选**）—— 形如 ``{file: dual_arm.yaml}``，
#: 相对路径按 :data:`rm75_drake.paths.calibration_files_dir`（仓库根 ``calibration/``）解析；
#: 该标定文件里的基座外参是**唯一真相源**（覆盖场景里手写的 ``base_extrinsics``）。
_CALIBRATION_KEYS: Tuple[str, ...] = (
    "gripper_force",
    "joint_zero_offset",
    "tool_offset",
    "base_extrinsic",
)

#: 目前支持的障碍物类型（其它类型显式报错，不静默降级）。
_SUPPORTED_OBSTACLE_TYPES: Tuple[str, ...] = ("box",)

#: rpy 主值区间上限（弧度）。超出该区间视为"角度制写错"。
_RPY_LIMIT = math.pi

#: xyz / size 的量纲上限（米）。超出视为"单位写错"（例如把毫米当米填）。
_LENGTH_LIMIT = 100.0


class ScenarioError(ValueError):
    """scenario schema 校验失败（未知键 / 缺字段 / 维度或单位错误）。"""


# ── 内部校验工具 ────────────────────────────────────────────────────────
def _require_mapping(value: Any, path: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ScenarioError(f"{path} 必须是映射（mapping），收到 {type(value).__name__}")
    for key in value:
        if not isinstance(key, str):
            raise ScenarioError(f"{path} 的键必须是字符串，收到 {key!r}")
    return value


def _require_sequence(value: Any, path: str) -> Sequence[Any]:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise ScenarioError(f"{path} 必须是序列（list），收到 {type(value).__name__}")
    return value


def _check_keys(
    mapping: Mapping[str, Any],
    path: str,
    allowed: Sequence[str],
    required: Sequence[str],
) -> None:
    """校验键集合：未知键与缺必填键都抛 :class:`ScenarioError`。"""
    unknown = [k for k in mapping if k not in allowed]
    if unknown:
        raise ScenarioError(
            f"{path} 出现未知键：{sorted(unknown)}；"
            f"该处允许的键只有 {list(allowed)}"
        )
    missing = [k for k in required if k not in mapping]
    if missing:
        raise ScenarioError(f"{path} 缺少必填字段：{missing}；该处允许的键只有 {list(allowed)}")


def _as_float(value: Any, path: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ScenarioError(f"{path} 必须是数值，收到 {value!r}")
    out = float(value)
    if not math.isfinite(out):
        raise ScenarioError(f"{path} 必须是有限数值，收到 {value!r}")
    return out


def _as_float_list(value: Any, path: str, length: int) -> Tuple[float, ...]:
    """把序列转成固定长度的浮点元组（维度不对直接报错）。"""
    seq = _require_sequence(value, path)
    if len(seq) != length:
        raise ScenarioError(f"{path} 必须是 {length} 个数值（长度 m / 角度 rad），收到 {len(seq)} 个")
    return tuple(_as_float(v, f"{path}[{i}]") for i, v in enumerate(seq))


def _check_xyz(values: Tuple[float, ...], path: str) -> Tuple[float, ...]:
    """位置量纲检查：数值必须是米（上限 100 m，超出视为单位写错）。"""
    for i, v in enumerate(values):
        if abs(v) > _LENGTH_LIMIT:
            raise ScenarioError(
                f"{path}[{i}] = {v} 超出长度上限 {_LENGTH_LIMIT} m；"
                "长度一律用米（若填的是毫米请先换算）"
            )
    return values


def _check_rpy(values: Tuple[float, ...], path: str) -> Tuple[float, ...]:
    """姿态量纲检查：rpy 一律弧度，且落在主值区间 [-π, π]。"""
    for i, v in enumerate(values):
        if abs(v) > _RPY_LIMIT:
            raise ScenarioError(
                f"{path}[{i}] = {v} 超出弧度主值区间 [-π, π]；"
                "rpy 必须用弧度（rad），不接收角度制（度）——"
                f"若填的是度，请乘 {math.pi / 180.0:.10f} 换算为弧度"
            )
    return values


def _require_str(value: Any, path: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ScenarioError(f"{path} 必须是非空字符串，收到 {value!r}")
    return value


# ── 数据结构 ────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class RobotSpec:
    """场景里的一台机器人。

    字段
    ----
    name:
        机器人名（在 ``base_extrinsics`` / ``calibration`` 里作为键）。
    urdf:
        ``package://`` URL（禁止写绝对路径）。
    base_link:
        被焊接到世界系的基座 link 名。
    joint_names:
        **有序**关节名（规划链路的变量顺序 = 这个顺序）。
    """

    name: str
    urdf: str
    base_link: str
    joint_names: Tuple[str, ...]


@dataclass(frozen=True)
class BaseExtrinsic:
    """机器人基座相对场景世界系的外参（长度 m / 角度 rad）。"""

    xyz: Tuple[float, float, float]
    rpy: Tuple[float, float, float]


@dataclass(frozen=True)
class Obstacle:
    """场景里的一个静态障碍物（目前支持 box）。

    ``size`` 的语义：**外形全长**（m），不是半长；Drake 的 ``Box`` 用半长，
    转换见 :attr:`half_size`。
    """

    name: str
    type: str
    size: Tuple[float, float, float]
    xyz: Tuple[float, float, float]
    rpy: Tuple[float, float, float]

    @property
    def half_size(self) -> Tuple[float, float, float]:
        """Drake ``Box`` 需要的半长（m）。"""
        return tuple(s / 2.0 for s in self.size)  # type: ignore[return-value]


@dataclass(frozen=True)
class Scenario:
    """场景真相源（schema_version 1）。

    单位：长度 m、角度 rad、时间 s、夹爪开口 u ∈ [0, 1000]。
    """

    schema_version: int
    robots: Tuple[RobotSpec, ...]
    base_extrinsics: Mapping[str, BaseExtrinsic]
    obstacles: Tuple[Obstacle, ...]
    calibration: Mapping[str, Any] = field(default_factory=dict)
    source_path: Optional[str] = None

    @property
    def robot_names(self) -> Tuple[str, ...]:
        return tuple(r.name for r in self.robots)

    def robot(self, name: str) -> RobotSpec:
        """按名字取机器人；不存在抛 :class:`ScenarioError`。"""
        for spec in self.robots:
            if spec.name == name:
                return spec
        raise ScenarioError(f"场景里没有名为 {name!r} 的机器人（已有：{list(self.robot_names)}）")


# ── 解析 ────────────────────────────────────────────────────────────────
def _parse_robot(entry: Any, index: int) -> RobotSpec:
    path = f"robots[{index}]"
    mapping = _require_mapping(entry, path)
    _check_keys(mapping, path, _ROBOT_KEYS, _ROBOT_KEYS)

    joints = _require_sequence(mapping["joint_names"], f"{path}.joint_names")
    if not joints:
        raise ScenarioError(f"{path}.joint_names 不能为空")
    joint_names = tuple(_require_str(j, f"{path}.joint_names[{i}]") for i, j in enumerate(joints))
    if len(set(joint_names)) != len(joint_names):
        raise ScenarioError(f"{path}.joint_names 里有重复名字：{list(joint_names)}")

    return RobotSpec(
        name=_require_str(mapping["name"], f"{path}.name"),
        urdf=_require_str(mapping["urdf"], f"{path}.urdf"),
        base_link=_require_str(mapping["base_link"], f"{path}.base_link"),
        joint_names=joint_names,
    )


def _parse_extrinsic(entry: Any, path: str) -> BaseExtrinsic:
    mapping = _require_mapping(entry, path)
    _check_keys(mapping, path, _EXTRINSIC_KEYS, _EXTRINSIC_KEYS)
    xyz = _check_xyz(
        _as_float_list(mapping["xyz"], f"{path}.xyz", 3), f"{path}.xyz"
    )
    rpy = _check_rpy(_as_float_list(mapping["rpy"], f"{path}.rpy", 3), f"{path}.rpy")
    return BaseExtrinsic(xyz=xyz, rpy=rpy)  # type: ignore[arg-type]


def _parse_obstacle(entry: Any, index: int) -> Obstacle:
    path = f"obstacles[{index}]"
    mapping = _require_mapping(entry, path)
    _check_keys(mapping, path, _OBSTACLE_KEYS, _OBSTACLE_KEYS)

    obs_type = _require_str(mapping["type"], f"{path}.type")
    if obs_type not in _SUPPORTED_OBSTACLE_TYPES:
        raise ScenarioError(
            f"{path}.type = {obs_type!r} 不受支持；当前支持：{list(_SUPPORTED_OBSTACLE_TYPES)}"
        )

    size = _as_float_list(mapping["size"], f"{path}.size", 3)
    for i, s in enumerate(size):
        if s <= 0.0:
            raise ScenarioError(f"{path}.size[{i}] = {s} 必须是正数（外形全长，m）")
        if s > _LENGTH_LIMIT:
            raise ScenarioError(
                f"{path}.size[{i}] = {s} 超出长度上限 {_LENGTH_LIMIT} m；长度一律用米"
            )

    pose_path = f"{path}.pose"
    pose = _require_mapping(mapping["pose"], pose_path)
    _check_keys(pose, pose_path, _POSE_KEYS, _POSE_KEYS)
    xyz = _check_xyz(_as_float_list(pose["xyz"], f"{pose_path}.xyz", 3), f"{pose_path}.xyz")
    rpy = _check_rpy(_as_float_list(pose["rpy"], f"{pose_path}.rpy", 3), f"{pose_path}.rpy")

    return Obstacle(
        name=_require_str(mapping["name"], f"{path}.name"),
        type=obs_type,
        size=size,  # type: ignore[arg-type]
        xyz=xyz,  # type: ignore[arg-type]
        rpy=rpy,  # type: ignore[arg-type]
    )


def _load_calibration_base_extrinsics(raw_file: str) -> Tuple[str, Dict[str, BaseExtrinsic]]:
    """加载标定文件里的基座外参（标定文件是外参的**唯一真相源**）。

    返回 ``(标定文件绝对路径, {机器人名: BaseExtrinsic})``。

    * 文件不存在 → :class:`ScenarioError`，消息**点名路径**；
    * 标定文件 schema 非法（未知键 / 缺字段 / 维度或单位错）→ :class:`ScenarioError`，
      消息**同时点名路径与出错键**（标定层的 ``CalibrationError`` 消息里已带键路径）。
    """
    from . import paths
    from .calibration.schema import CalibrationError, load_calibration_file

    candidate = Path(raw_file)
    if not candidate.is_absolute():
        candidate = paths.calibration_files_dir / candidate
    if not candidate.is_file():
        raise ScenarioError(
            f"calibration.base_extrinsic.file 引用的标定文件不存在：{candidate}；"
            f"相对路径按 {paths.calibration_files_dir} 解析"
        )
    try:
        doc = load_calibration_file(candidate)
    except CalibrationError as exc:
        # 标定层已点名出错键，这里补上路径（不静默回退到手写外参）。
        raise ScenarioError(f"标定文件非法：{candidate}（{exc}）") from exc
    except FileNotFoundError as exc:  # pragma: no cover - 竞态删除
        raise ScenarioError(
            f"calibration.base_extrinsic.file 引用的标定文件不存在：{candidate}"
        ) from exc

    resolved: Dict[str, BaseExtrinsic] = {}
    for robot, entry in doc.base_extrinsic.items():
        resolved[robot] = BaseExtrinsic(xyz=entry.xyz, rpy=entry.rpy)
    return str(candidate), resolved


def _parse_calibration(entry: Any) -> Tuple[Dict[str, Any], Dict[str, BaseExtrinsic]]:
    """解析 ``calibration`` 段（可为空映射，但键必须存在）。

    返回 ``(解析结果, 标定文件给出的基座外参)``；第二项仅在存在
    ``calibration.base_extrinsic`` 时非空（此时它**覆盖**场景手写的
    ``base_extrinsics``——标定文件是唯一真相源）。
    """
    from . import paths

    mapping = _require_mapping(entry, "calibration")
    _check_keys(mapping, "calibration", _CALIBRATION_KEYS, ())

    out: Dict[str, Any] = {}

    if "gripper_force" in mapping:
        path = "calibration.gripper_force"
        sub = _require_mapping(mapping["gripper_force"], path)
        _check_keys(sub, path, ("file",), ("file",))
        raw = _require_str(sub["file"], f"{path}.file")
        candidate = Path(raw)
        if not candidate.is_absolute():
            candidate = paths.calibration_dir / candidate
        if not candidate.is_file():
            raise ScenarioError(
                f"{path}.file 引用的标定文件不存在：{candidate}；"
                f"相对路径按 {paths.calibration_dir} 解析"
            )
        out["gripper_force"] = {"file": str(candidate)}

    if "joint_zero_offset" in mapping:
        path = "calibration.joint_zero_offset"
        sub = _require_mapping(mapping["joint_zero_offset"], path)
        offsets: Dict[str, Dict[str, float]] = {}
        for robot_name, joints in sub.items():
            jpath = f"{path}.{robot_name}"
            jmap = _require_mapping(joints, jpath)
            offsets[robot_name] = {}
            for joint_name, value in jmap.items():
                vpath = f"{jpath}.{joint_name}"
                offset = _as_float(value, vpath)
                if abs(offset) > _RPY_LIMIT:
                    raise ScenarioError(
                        f"{vpath} = {offset} 超出弧度主值区间 [-π, π]；"
                        "关节零位偏移一律用弧度（rad）"
                    )
                offsets[robot_name][joint_name] = offset
        out["joint_zero_offset"] = offsets

    if "tool_offset" in mapping:
        path = "calibration.tool_offset"
        sub = _require_mapping(mapping["tool_offset"], path)
        tool: Dict[str, BaseExtrinsic] = {}
        for robot_name, value in sub.items():
            tpath = f"{path}.{robot_name}"
            tool[robot_name] = _parse_extrinsic(value, tpath)
        out["tool_offset"] = tool

    resolved: Dict[str, BaseExtrinsic] = {}
    if "base_extrinsic" in mapping:
        path = "calibration.base_extrinsic"
        sub = _require_mapping(mapping["base_extrinsic"], path)
        _check_keys(sub, path, ("file",), ("file",))
        raw = _require_str(sub["file"], f"{path}.file")
        resolved_path, resolved = _load_calibration_base_extrinsics(raw)
        out["base_extrinsic"] = {"file": resolved_path}
        if not resolved:
            raise ScenarioError(
                f"标定文件里没有 base_extrinsic 段：{resolved_path}；"
                "该场景引用了标定文件作为基座外参真相源，标定文件必须给出外参"
            )

    return out, resolved


def load_scenario_dict(data: Any, source_path: Optional[str] = None) -> Scenario:
    """从已解析的 dict 构造 :class:`Scenario`（校验失败抛 :class:`ScenarioError`）。"""
    mapping = _require_mapping(data, "scenario")

    unknown = [k for k in mapping if k not in TOP_LEVEL_KEYS]
    if unknown:
        raise ScenarioError(
            f"scenario 出现未知顶层键：{sorted(unknown)}；"
            f"顶层键固定为 {list(TOP_LEVEL_KEYS)}（改名会击穿 golden-file 回归）"
        )
    missing = [k for k in TOP_LEVEL_KEYS if k not in mapping]
    if missing:
        raise ScenarioError(f"scenario 缺少必填顶层键：{missing}")

    version = mapping["schema_version"]
    if isinstance(version, bool) or not isinstance(version, int):
        raise ScenarioError(f"schema_version 必须是整数，收到 {version!r}")
    if version != SCHEMA_VERSION:
        raise ScenarioError(
            f"schema_version = {version} 不受支持；当前正式版本是 {SCHEMA_VERSION}"
            "（0 是 S02 之前的占位版本）"
        )

    robots_raw = _require_sequence(mapping["robots"], "robots")
    robots = tuple(_parse_robot(entry, i) for i, entry in enumerate(robots_raw))
    if not robots:
        raise ScenarioError("robots 不能为空：场景至少要有一台机器人")
    names = [r.name for r in robots]
    if len(set(names)) != len(names):
        raise ScenarioError(f"robots 里有重名机器人：{names}")

    extrinsics_raw = _require_mapping(mapping["base_extrinsics"], "base_extrinsics")
    extrinsics: Dict[str, BaseExtrinsic] = {}
    for robot_name, entry in extrinsics_raw.items():
        extrinsics[robot_name] = _parse_extrinsic(entry, f"base_extrinsics.{robot_name}")

    missing_extrinsic = [r.name for r in robots if r.name not in extrinsics]
    if missing_extrinsic:
        raise ScenarioError(
            f"base_extrinsics 缺少机器人外参：{missing_extrinsic}；"
            "禁止静默默认（外参必须是显式声明的米/弧度值）"
        )
    unknown_extrinsic = [k for k in extrinsics if k not in names]
    if unknown_extrinsic:
        raise ScenarioError(
            f"base_extrinsics 里有未在 robots 中声明的机器人：{sorted(unknown_extrinsic)}"
        )

    obstacles_raw = _require_sequence(mapping["obstacles"], "obstacles")
    obstacles = tuple(_parse_obstacle(entry, i) for i, entry in enumerate(obstacles_raw))
    obstacle_names = [o.name for o in obstacles]
    if len(set(obstacle_names)) != len(obstacle_names):
        raise ScenarioError(f"obstacles 里有重名障碍物：{obstacle_names}")

    calibration, calibrated_extrinsics = _parse_calibration(mapping["calibration"])

    if calibrated_extrinsics:
        # 标定文件是基座外参的**唯一真相源**：它给出的值覆盖场景里手写的值；
        # 缺项 / 多项一律显式报错（禁止静默回退到手写值造成两个真相源）。
        unknown_calib = [k for k in calibrated_extrinsics if k not in names]
        if unknown_calib:
            raise ScenarioError(
                f"标定文件里的基座外参含未在 robots 中声明的机器人：{sorted(unknown_calib)}；"
                f"标定文件={calibration.get('base_extrinsic', {}).get('file')}"
            )
        missing_calib = [n for n in names if n not in calibrated_extrinsics]
        if missing_calib:
            raise ScenarioError(
                f"标定文件里缺少机器人 {missing_calib} 的基座外参；"
                f"标定文件={calibration.get('base_extrinsic', {}).get('file')}；"
                "禁止静默回退到手写 base_extrinsics（那会形成两个真相源）"
            )
        extrinsics.update(calibrated_extrinsics)

    return Scenario(
        schema_version=version,
        robots=robots,
        base_extrinsics=extrinsics,
        obstacles=obstacles,
        calibration=calibration,
        source_path=source_path,
    )


def scenario_from_dict(data: Any, source_path: Optional[str] = None) -> Scenario:
    """:func:`load_scenario_dict` 的别名。"""
    return load_scenario_dict(data, source_path=source_path)


def load_scenario(path) -> Scenario:
    """从 YAML 文件加载场景。

    * 文件不存在 → :class:`FileNotFoundError`；
    * YAML 语法错误 → :class:`ScenarioError`；
    * schema 违规 → :class:`ScenarioError`。
    """
    import yaml

    file_path = Path(path)
    try:
        text = file_path.read_text(encoding="utf-8")
    except FileNotFoundError as exc:
        raise FileNotFoundError(f"找不到 scenario 文件：{file_path}") from exc
    except OSError as exc:  # pragma: no cover - 权限/IO 异常
        raise ScenarioError(f"读取 scenario 文件失败：{file_path}（{exc}）") from exc

    try:
        data = yaml.safe_load(text)
    except yaml.YAMLError as exc:
        raise ScenarioError(f"scenario YAML 解析失败：{file_path}（{exc}）") from exc

    if data is None:
        raise ScenarioError(f"scenario 文件为空：{file_path}")

    return load_scenario_dict(data, source_path=str(file_path))
