"""双臂 URDF 变体生成器（S04-T01）。

**为什么需要它（实测钉死，见包 docstring）**

Drake 1.51.1 / 1.55.0 的 ``Parser.AddModels`` 在同一个 plant 里对同一份 URDF
只允许一个 model instance（第二次调用抛 ``RuntimeError: ... Model instance names
must be unique within a given model.``），且两档都**没有** ``simple_name`` 参数，
无法在解析时改名。R005 要求「两臂共享同一个 MultibodyPlant，臂间碰撞 / 自碰撞 /
相邻体过滤在同一次碰撞查询中统一判定」，因此唯一可行路线是
「**每臂一份重命名 URDF 变体**」。

本模块把该路线固化为**确定性生成器 + 入 git 的变体文件**：
变体**只改名**，**不改任何数值**——惯量 / 质心 / 限位 / 碰撞几何与原点
逐字段等于源文件（S01 的官方改造版 URDF 仍是唯一模型真相源）。

**命名规则**

======================  ==================  ==========================
对象                     规则                例（rm75 / left）
======================  ==================  ==========================
``<robot name>``        ``<基名>-<侧别>``    ``RM75-B`` → ``RM75-B-left``
``<link name>``         ``<侧别>_<原名>``    ``link_7`` → ``left_link_7``
``<joint name>``        ``<侧别>_<原名>``    ``joint_1`` → ``left_joint_1``
``<collision name>``    ``<侧别>_<原名>``    ``link_7_collision`` → ``left_link_7_collision``
``parent/child @link``  ``<侧别>_<原名>``    ``base_link`` → ``left_base_link``
======================  ==================  ==========================

侧别前缀固定为 ``left_`` / ``right_``（**不是** ``l_`` / ``r_``，避免歧义）。

**不改动**（数值真值，逐字段保持）：

* ``<inertial>``（质量 / 质心 / 惯量张量）；
* ``<limit>``（lower / upper / velocity / effort）；
* ``<collision>`` 的 ``<origin xyz rpy>`` 与 ``<geometry>``；
* ``<visual>``（本项目模型本就无 visual）；
* 固定关节（夹爪 ``tcp_joint``）的**类型与连接关系**（仅改名）。

**为什么用 ``xml.etree.ElementTree`` 而不是正则**

自闭合标签（``<collision .../>``、``<origin .../>``）会被正则重写打乱，实测报
``XML_ERROR_MISMATCHED_ELEMENT``。ET 走真正的 XML 解析器，**能保留注释**
（``insert_comments=True``）并保证输出仍是合法 XML。

**输出格式的确定性**

* 文件头（XML 声明 + 源文件顶部的说明注释）**原样保留**；
* 根元素之前追加一段**自动生成声明**注释（说明「只改名、勿手改、``--check`` 语义」）；
* ET 序列化结果**逐字节确定**（属性顺序 = 解析顺序，缩进 = 源文件 tail 空白），
  因此生成器**幂等**：重跑后与磁盘上的文件字节级一致（``--check`` 退出码 0）。

本模块**零 pydrake 依赖**。
"""

from __future__ import annotations

import os
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

from .. import paths

__all__ = [
    "KINDS",
    "SIDES",
    "BASE_ROBOT_NAMES",
    "SOURCE_FILES",
    "VariantError",
    "VariantMismatchError",
    "variant_robot_name",
    "variant_path",
    "source_path",
    "build_variant",
    "write_variants",
    "check_variants",
]

#: 支持的模型种类（对应 S01 的两份官方改造版 URDF）。
KINDS: Tuple[str, ...] = ("rm75", "rmg24")

#: 支持的侧别。
SIDES: Tuple[str, ...] = ("left", "right")

#: 源 URDF 的 ``<robot name>`` 基名（与 S01 官方改造版逐字一致）。
BASE_ROBOT_NAMES: Dict[str, str] = {
    "rm75": "RM75-B",
    "rmg24": "RMG24",
}

#: 源 URDF 文件名（位于 ``models/rm75_drake/``）。
SOURCE_FILES: Dict[str, str] = {
    "rm75": "rm75_drake.urdf",
    "rmg24": "rmg24_drake.urdf",
}

#: Drake URDF 里用到的命名空间（``<drake:disable_collision>``）。
_DRAKE_NS = "http://drake.edu"

#: 侧别前缀（link / joint / collision 名统一加此前缀）。
_SIDE_PREFIX = {"left": "left_", "right": "right_"}


class VariantError(Exception):
    """变体生成 / 校验的基类异常。"""


class VariantMismatchError(VariantError):
    """磁盘上的变体文件与生成器输出不一致（``--check`` 用）。

    属性
    ----
    report:
        逐个变体的检查结果（见 :func:`check_variants` 的返回结构）。
    """

    def __init__(self, message: str, report: Optional[dict] = None) -> None:
        super().__init__(message)
        self.report = report or {}


# ``drake:`` 前缀必须注册，否则 ET 会写成 ``ns0:``（语义等价但与源文件不一致）。
ET.register_namespace("drake", _DRAKE_NS)


def _check_kind(kind: str) -> str:
    if kind not in KINDS:
        raise VariantError(
            f"未知的模型种类 {kind!r}；可用：{', '.join(KINDS)}"
        )
    return kind


def _check_side(side: str) -> str:
    if side not in SIDES:
        raise VariantError(
            f"未知的侧别 {side!r}；可用：{', '.join(SIDES)}"
        )
    return side


def variant_robot_name(kind: str, side: str) -> str:
    """返回变体的 ``<robot name>``（例：``RM75-B-left``）。

    参数
    ----
    kind:
        ``"rm75"``（RM75-B 机械臂）或 ``"rmg24"``（RMG24 夹爪）。
    side:
        ``"left"`` 或 ``"right"``。
    """
    kind = _check_kind(kind)
    side = _check_side(side)
    return f"{BASE_ROBOT_NAMES[kind]}-{side}"


def models_package_dir() -> Path:
    """返回模型包目录（``models/rm75_drake/``）。"""
    return paths.models_dir / "rm75_drake"


def source_path(kind: str) -> Path:
    """返回源 URDF 的绝对路径（``models/rm75_drake/rm75_drake.urdf`` 等）。"""
    kind = _check_kind(kind)
    return models_package_dir() / SOURCE_FILES[kind]


def variant_path(side: str, kind: str) -> Path:
    """返回变体 URDF 的绝对路径（``models/rm75_drake/rm75_left.urdf`` 等）。

    参数顺序为 ``(side, kind)``（先侧别、后种类），与文件名 ``{kind}_{side}.urdf``
    的**字典序**相反——刻意如此：调用方通常按「左臂机械臂 / 左臂夹爪 / 右臂…」的
    顺序取用，而文件名排序（rm75_left < rm75_right < rmg24_left < rmg24_right）
    仍是稳定的。
    """
    kind = _check_kind(kind)
    side = _check_side(side)
    return models_package_dir() / f"{kind}_{side}.urdf"


def variant_id(kind: str, side: str) -> str:
    """变体的稳定标识（``rm75_left`` / ``rmg24_right`` …）。"""
    return f"{_check_kind(kind)}_{_check_side(side)}"


def _root_start_index(text: str) -> int:
    """返回根元素起始标签 ``<`` 在 *text* 中的下标。

    跳过 XML 声明（``<?…?>``）与注释（``<!--…-->``），因此**根元素之前的内容**
    （文件头说明注释）可以被原样保留。
    """
    i = 0
    while True:
        i = text.find("<", i)
        if i < 0:
            raise VariantError("URDF 文本里找不到根元素起始标签")
        nxt = text[i + 1 : i + 2]
        if nxt == "?":
            j = text.find("?>", i)
            if j < 0:
                raise VariantError("XML 声明未闭合")
            i = j + 2
            continue
        if nxt == "!":
            j = text.find("-->", i)
            if j < 0:
                raise VariantError("注释未闭合")
            i = j + 3
            continue
        return i


def _parse_with_comments(text: str) -> Tuple[ET.Element, str]:
    """解析 URDF，返回 ``(根元素, 根元素之前的前缀文本)``。

    使用 ``insert_comments=True`` 的 TreeBuilder，**保留文件内注释**；
    ``前缀`` = XML 声明 + 顶部说明注释（逐字保留）。
    """
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    try:
        root = ET.fromstring(text, parser=parser)
    except ET.ParseError as exc:  # pragma: no cover - 由调用方给的坏文件触发
        raise VariantError(f"URDF 解析失败：{exc}") from exc
    prefix = text[: _root_start_index(text)]
    return root, prefix


def _generated_banner(kind: str, side: str) -> str:
    """生成「本文件是自动生成的变体」注释块。"""
    # 注意：XML 注释内**不能**出现连续两个连字符（"--"），否则 XML 解析报
    # "not well-formed (invalid token)"。因此这里不能直接写命令行选项 "--check"。
    return (
        "<!--\n"
        f"  本文件由 tools/gen_dual_arm_models.py 自动生成（{kind} / {side} 臂）。\n"
        "  它是 S01 官方改造版 URDF 的**重命名变体**：只改 robot / link / joint /\n"
        "  collision 的名字，惯量、限位、碰撞几何与原点逐字段与源文件相同。\n"
        "  请勿手改：任何手改都会让变体校验（gen_dual_arm_models.py 的 check 模式）\n"
        "  失败（CI 用它守住「变体 = 源文件改名」这条不变量）。\n"
        "-->\n"
    )


def build_variant(source_path_: os.PathLike | str, *, side: str, kind: str) -> str:
    """从源 URDF 生成一个重命名变体，返回其完整文本。

    参数
    ----
    source_path_:
        源 URDF 路径（通常是 :func:`source_path` 的返回值）。
    side:
        ``"left"`` / ``"right"``。
    kind:
        ``"rm75"`` / ``"rmg24"``（决定 ``<robot name>`` 的基名）。

    异常
    ----
    VariantError:
        侧别 / 种类非法、源文件不存在、源 ``<robot name>`` 与 ``kind`` 不匹配、
        XML 解析失败。
    """
    kind = _check_kind(kind)
    side = _check_side(side)
    prefix_p = _SIDE_PREFIX[side]

    src = Path(source_path_)
    if not src.is_file():
        raise VariantError(f"源 URDF 不存在：{src}")
    text = src.read_text(encoding="utf-8")
    root, head = _parse_with_comments(text)

    expected = BASE_ROBOT_NAMES[kind]
    if root.get("name") != expected:
        raise VariantError(
            f"源 URDF 的 <robot name>={root.get('name')!r} 与 kind={kind!r} "
            f"期望的 {expected!r} 不一致（模型真相源被改动？）"
        )
    root.set("name", variant_robot_name(kind, side))

    # link 名
    for link in root.findall("link"):
        name = link.get("name")
        if name is None:
            raise VariantError("源 URDF 里有无名 <link>，无法生成变体")
        link.set("name", f"{prefix_p}{name}")

    # joint 名 + parent/child 的 link 引用
    for joint in root.findall("joint"):
        name = joint.get("name")
        if name is None:
            raise VariantError("源 URDF 里有无名 <joint>，无法生成变体")
        joint.set("name", f"{prefix_p}{name}")
        for tag in ("parent", "child"):
            ref = joint.find(tag)
            if ref is None:
                raise VariantError(
                    f"joint {name!r} 缺少 <{tag}>，无法生成变体"
                )
            link_name = ref.get("link")
            if link_name is None:
                raise VariantError(
                    f"joint {name!r} 的 <{tag}> 缺少 link 属性，无法生成变体"
                )
            ref.set("link", f"{prefix_p}{link_name}")

    # collision 名（几何与原点不动）
    for collision in root.iter("collision"):
        name = collision.get("name")
        if name is None:
            raise VariantError("源 URDF 里有无名 <collision>，无法生成变体")
        collision.set("name", f"{prefix_p}{name}")

    # <drake:disable_collide link1=… link2=…> 引用的是 link 名，必须同步改名，
    # 否则声明会指向不存在的 link（Drake 当前不实现该标签，但语义不能悬空）。
    for element in root.iter():
        if not isinstance(element.tag, str):
            continue
        if element.tag.rsplit("}", 1)[-1] != "disable_collide":
            continue
        for attr in ("link1", "link2"):
            value = element.get(attr)
            if value is not None:
                element.set(attr, f"{prefix_p}{value}")

    body = ET.tostring(root, encoding="unicode")
    if not body.endswith("\n"):
        body += "\n"
    return head + _generated_banner(kind, side) + body


def _count_structure(text: str) -> dict:
    """统计一个 URDF 文本的 link / joint / collision 数量（用于 CLI 摘要）。"""
    root, _ = _parse_with_comments(text)
    joints = root.findall("joint")
    return {
        "links": len(root.findall("link")),
        "joints": len(joints),
        "revolute": sum(1 for j in joints if j.get("type") == "revolute"),
        "prismatic": sum(1 for j in joints if j.get("type") == "prismatic"),
        "fixed": sum(1 for j in joints if j.get("type") == "fixed"),
        "collisions": sum(1 for _ in root.iter("collision")),
    }


def _variant_pairs() -> List[Tuple[str, str]]:
    """返回全部 ``(kind, side)`` 组合（顺序稳定：rm75_left → rmg24_right）。"""
    return [(kind, side) for kind in KINDS for side in SIDES]


def write_variants(
    check_only: bool = False,
    out_dir: Optional[os.PathLike | str] = None,
) -> Dict[str, dict]:
    """生成（或校验）四个双臂 URDF 变体。

    参数
    ----
    check_only:
        ``True`` 时**只比对不写**：磁盘文件缺失或与生成器输出不一致 → 抛
        :class:`VariantMismatchError`（附逐个变体的差异报告）。
    out_dir:
        输出目录，默认 ``models/rm75_drake/``。

    返回
    ----
    dict：
        键为变体标识（``rm75_left`` …），值为
        ``{"path", "robot_name", "status", "links", "joints", "collisions"}``；
        ``status`` ∈ ``{"written", "ok"}``（``check_only=True`` 时只可能是 ``ok``）。
    """
    out = Path(out_dir) if out_dir is not None else models_package_dir()
    report: Dict[str, dict] = {}
    problems: List[str] = []

    for kind, side in _variant_pairs():
        vid = variant_id(kind, side)
        target = out / f"{kind}_{side}.urdf"
        content = build_variant(source_path(kind), side=side, kind=kind)
        stats = _count_structure(content)
        entry = {
            "path": target,
            "robot_name": variant_robot_name(kind, side),
            "status": "ok",
            **stats,
        }
        if check_only:
            if not target.is_file():
                entry["status"] = "missing"
                problems.append(f"{vid}: 文件缺失 → {target}")
            else:
                on_disk = target.read_text(encoding="utf-8")
                if on_disk != content:
                    entry["status"] = "stale"
                    problems.append(
                        f"{vid}: 磁盘内容与生成器不一致 → {target} "
                        f"(磁盘 {len(on_disk)} 字节 / 生成 {len(content)} 字节)"
                    )
        else:
            out.mkdir(parents=True, exist_ok=True)
            target.write_text(content, encoding="utf-8")
            entry["status"] = "written"
        report[vid] = entry

    if problems:
        raise VariantMismatchError(
            "变体文件与生成器不一致（变体已过期或缺失）：\n  - "
            + "\n  - ".join(problems),
            report=report,
        )
    return report


def check_variants(out_dir: Optional[os.PathLike | str] = None) -> Dict[str, dict]:
    """只校验变体（不写盘），**不抛异常**——返回逐个变体的状态报告。

    状态：``"ok"``（一致）/ ``"missing"``（缺失）/ ``"stale"``（不一致）。
    """
    out = Path(out_dir) if out_dir is not None else models_package_dir()
    report: Dict[str, dict] = {}
    for kind, side in _variant_pairs():
        vid = variant_id(kind, side)
        target = out / f"{kind}_{side}.urdf"
        content = build_variant(source_path(kind), side=side, kind=kind)
        if not target.is_file():
            status = "missing"
        elif target.read_text(encoding="utf-8") != content:
            status = "stale"
        else:
            status = "ok"
        report[vid] = {
            "path": target,
            "robot_name": variant_robot_name(kind, side),
            "status": status,
            **_count_structure(content),
        }
    return report


def iter_variant_ids() -> Iterable[str]:
    """返回全部变体标识（稳定顺序）。"""
    for kind, side in _variant_pairs():
        yield variant_id(kind, side)
