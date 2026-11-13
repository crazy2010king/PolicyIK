"""``package://`` 解析的唯一入口（T02）。

## 实测结论（T02 钉死，后续切片不得再自行验证）

实测环境：受支持基线 Drake 1.55.0（py312）与降级档 Drake 1.51.1（py310），
两档结论一致：

======================  ==========================  =============================
注册方式                 key                        value（``GetPath`` 返回）
======================  ==========================  =============================
``PopulateFromFolder``  package.xml 里的 ``<name>``  该 package 所在**目录**（带尾斜杠）
``AddPackageXml``       同上                        同一目录（**不带**尾斜杠）
======================  ==========================  =============================

两者的 ``ResolveUrl("package://<pkg>/rel/path")`` 结果**完全一致**（同一绝对文件路径），
因此拼接语义本身没有歧义；差异仅存在于 ``GetPath`` 原始 value 的尾斜杠。

**结论**：本模块统一只走 ``PopulateFromFolder``（一次注册、目录树级联、天然支持
``models/<pkg>/package.xml`` 布局），**禁止**在同一张 map 上混用两种注册方式，
以免后续维护者按 ``GetPath`` 的 value 自行拼路径时产生歧义。

## 另一个必须记住的 Drake API 陷阱

``Parser.AddModels`` 的 URL 形式是**关键字参数**：``parser.AddModels(url="package://...")``。
位置参数会被当成普通文件名（"package://..." 相对 cwd 拼接后 XML 解析失败）。

## 反模式：**不得**在解析前预预留 model instance 槽位（T02 返工钉死）

T02 早期版本曾在解析前调用 ``plant.AddModelInstance(...)`` 预留空槽位，试图规避
``detail_collision_filter_group_resolver.cc:27`` 的
``minimum_model_instance_index_ > world`` abort。**该变通是本项目自己引入的缺陷**：
它把 ``num_model_instances()`` 抬高，反而让 Drake 内部的
``minimum_model_instance_index_`` 大于 world，**确定性**触发该 DEMAND。

实测对照（同一台机器、同一份代码，唯一变量是"是否预预留"）：

* 受支持基线 Drake 1.55.0（py312）：预预留 0/20 成功；不预预留 **20/20 成功**；
* 降级档 Drake 1.51.1（py310）：预预留 0/10 成功；不预预留 **10/10 成功**。

结论：model instance 的分配**交还 Drake Parser 管理**，解析前不要动 plant 的
model instance 槽位。原 ``drake_compat.prepare_plant_for_parsing()`` 已删除。

## 生命周期陷阱：Drake 的 ``Parser`` 不保活 plant

C++ 侧 ``Parser`` 只持有 ``MultibodyPlant*`` **裸指针**。若 plant 在某个函数作用域里
创建（典型：``make_parser()`` 内部的局部变量），函数返回后 plant 被 Python GC
销毁，parser 悬垂，后续 ``AddModels()`` 直接 **SIGSEGV**。因此 ``make_parser()``
把 plant 登记到弱引用保活表（见 :data:`_PLANTS_BY_PARSER`），并提供
:func:`plant_for_parser` 供调用方显式持有。
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence
from weakref import WeakKeyDictionary

from . import paths

__all__ = [
    "build_package_map",
    "resolve_package_url",
    "package_names",
    "attach_package_map",
    "add_model_from_url",
    "make_parser",
    "plant_for_parser",
    "PACKAGE_URL_SCHEMES",
]

#: Drake 支持的 URL scheme。
PACKAGE_URL_SCHEMES = ("package://", "model://", "file://")

#: ``parser → plant`` 的保活表（T02 返工新增）。
#:
#: **弱键**（parser）+ **强引用**（plant）：只要 parser 活着，plant 就活着；
#: parser 被回收时条目自动消失（不会泄漏）。
#:
#: Drake 的 C++ ``Parser`` 只持有 ``MultibodyPlant*`` **裸指针**，Python 侧
#: **不会**自动保活 plant。若 plant 是在某个函数作用域里创建的（典型写法：
#: ``make_parser()`` 内部 ``plant = MultibodyPlant(0.0)``），函数返回后 plant 立即
#: 被 Python GC 销毁，parser 里的指针就变成悬垂指针——后续 ``AddModels()`` 会
#: **段错误（SIGSEGV）**，而不是给出可读的异常。
#:
#: 曾经把 plant 藏在 ``make_parser()`` 的局部变量里（调用方拿不到），于是踩到了这个坑。
#: 现在用 :class:`weakref.WeakKeyDictionary` 把 plant 挂在 parser 上：只要 parser 活着，
#: plant 就活着；parser 被回收时条目自动消失（不会泄漏）。
_PLANTS_BY_PARSER: "WeakKeyDictionary[Any, Any]" = WeakKeyDictionary()


def _normalize_dirs(dirs: Iterable[Path]) -> List[Path]:
    """去重并保持顺序（同一目录被注册两次会触发 Drake 的重复包告警）。"""
    seen = set()
    out: List[Path] = []
    for d in dirs:
        key = str(d)
        if key in seen:
            continue
        seen.add(key)
        out.append(d)
    return out


def build_package_map(
    search_paths: Optional[Sequence[Path]] = None,
    include_drake_packages: bool = False,
) -> "object":
    """构造并返回 ``pydrake.multibody.parsing.PackageMap``。

    参数
    ----
    search_paths:
        要扫描的模型根目录列表；``None`` 时用 :func:`rm75_drake.paths.model_search_paths`
        （即环境变量覆盖优先，否则仓库 ``models/``）。目录不存在时**跳过**（不抛错），
        但一个都不存在时抛 :class:`FileNotFoundError`，避免"静默绿灯"。
    include_drake_packages:
        是否把 Drake 自带的 ``drake`` / ``drake_models`` 包合并进来。
        默认关闭——构造 Drake 默认 PackageMap 会触发 drake_models 归档下载，
        在离线/CI 环境下不可控；需要时由调用方显式开启。

    只使用 ``PopulateFromFolder`` 注册（见模块 docstring 的实测结论）。
    """
    from pydrake.multibody.parsing import PackageMap  # 延迟导入：避免 import 本包即拉起 Drake

    try:
        pkg_map = PackageMap.MakeEmpty()
    except AttributeError:  # pragma: no cover - 极老版本没有 MakeEmpty
        pkg_map = PackageMap()

    dirs = _normalize_dirs(
        [Path(p) for p in search_paths]
        if search_paths is not None
        else paths.model_search_paths()
    )
    existing = [d for d in dirs if d.is_dir()]
    if not existing:
        raise FileNotFoundError(
            "没有可用的模型搜索路径：" + ", ".join(str(d) for d in dirs)
        )

    for d in existing:
        pkg_map.PopulateFromFolder(str(d))

    if include_drake_packages:  # pragma: no cover - 依赖网络/缓存
        try:
            pkg_map.AddMap(PackageMap())
        except Exception:
            # Drake 自带包合并失败不应影响本项目包的解析。
            pass

    return pkg_map


def package_names(pkg_map) -> List[str]:
    """返回已注册的包名（有序）。"""
    return list(pkg_map.GetPackageNames())


def resolve_package_url(pkg_map, url: str) -> str:
    """把 ``package://`` / ``model://`` / ``file://`` URL 解析为本地绝对路径。

    未知包或非法 URL 会让 Drake 抛 ``RuntimeError``，本函数**不吞异常**（显式冒泡）。
    """
    if not isinstance(url, str) or not url.startswith(PACKAGE_URL_SCHEMES):
        raise ValueError(
            f"只支持 {', '.join(PACKAGE_URL_SCHEMES)} URL，收到：{url!r}"
        )
    return pkg_map.ResolveUrl(url)


def attach_package_map(parser, pkg_map) -> None:
    """把 ``pkg_map`` 合并进 ``parser`` 自己的 PackageMap。

    ``Parser.package_map`` 属性只读，但 ``parser.package_map()`` 返回**可变引用**，
    因此用 ``AddMap`` 合并（Drake 自带的 drake / drake_models 条目会被保留）。

    **幂等**：若 ``pkg_map`` 里的包名已全部注册在 parser 上，则直接返回，
    避免重复 ``AddMap`` 造成的重复条目与告警。
    """
    existing = set(parser.package_map().GetPackageNames())
    wanted = list(pkg_map.GetPackageNames())
    if wanted and all(name in existing for name in wanted):
        return
    parser.package_map().AddMap(pkg_map)


def add_model_from_url(parser, url: str, pkg_map=None):
    """用 URL（``package://...``）向 ``parser`` 添加模型，返回 ModelInstanceIndex 列表。

    必须使用 ``url=`` 关键字（见模块 docstring 的 Drake API 陷阱说明）。
    """
    if pkg_map is not None:
        attach_package_map(parser, pkg_map)
    return parser.AddModels(url=url)


def make_parser(plant=None, pkg_map=None):
    """创建"已绑定 ``package://`` 且可安全解析"的 Parser。

    返回 ``(parser, pkg_map)``。步骤：

    1. 创建（或沿用）``MultibodyPlant``；
    2. 创建 ``Parser`` 并把 package map 合并进去（Drake 自带的 drake / drake_models
       条目保留）。

    **不做**任何 model instance 槽位预留：预预留会确定性触发 Drake 的
    ``CollisionFilterGroupResolver`` DEMAND（abort），详见模块 docstring 的反模式
    说明。model instance 的分配由 Drake Parser 自行管理。

    **生命周期**：Drake 的 ``Parser`` 只持有 plant 的裸指针，因此本函数把 plant
    登记到 :data:`_PLANTS_BY_PARSER`（弱引用字典）为其**保活**——否则 plant 会在
    本函数返回时被 GC 销毁，parser 悬垂，后续 ``AddModels()`` 直接 SIGSEGV。
    """
    from pydrake.multibody.parsing import Parser
    from pydrake.multibody.plant import MultibodyPlant

    if plant is None:
        plant = MultibodyPlant(0.0)
    parser = Parser(plant)
    _PLANTS_BY_PARSER[parser] = plant
    if pkg_map is None:
        pkg_map = build_package_map()
    attach_package_map(parser, pkg_map)
    return parser, pkg_map


def plant_for_parser(parser):
    """返回 ``make_parser()`` 创建（或登记过）的 plant；未登记时返回 ``None``。

    用途：让调用方显式拿到 plant 并延长其生命周期（Drake 的 Parser 不保活 plant）。
    """
    return _PLANTS_BY_PARSER.get(parser)
