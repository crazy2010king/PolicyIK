"""``package://`` 解析的唯一入口（T02）。

## 实测结论（T02 钉死，后续切片不得再自行验证）

在 Drake 1.51.1（fallback 档，Python 3.10）上实测：

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
"""

from __future__ import annotations

from pathlib import Path
from typing import Iterable, List, Optional, Sequence

from . import paths

__all__ = [
    "build_package_map",
    "resolve_package_url",
    "package_names",
    "attach_package_map",
    "add_model_from_url",
    "make_parser",
    "PACKAGE_URL_SCHEMES",
]

#: Drake 支持的 URL scheme。
PACKAGE_URL_SCHEMES = ("package://", "model://", "file://")


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
    """
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
    2. 调用 :func:`rm75_drake.drake_compat.prepare_plant_for_parsing` 预留 model
       instance 槽位（规避 Drake 1.51.1 的 ``CollisionFilterGroupResolver`` DEMAND
       间歇性 abort，见该函数 docstring）；
    3. 创建 ``Parser`` 并把 package map 合并进去（Drake 自带的 drake / drake_models
       条目保留）。
    """
    from pydrake.multibody.parsing import Parser
    from pydrake.multibody.plant import MultibodyPlant

    from .drake_compat import prepare_plant_for_parsing

    if plant is None:
        plant = MultibodyPlant(0.0)
    prepare_plant_for_parsing(plant)
    parser = Parser(plant)
    if pkg_map is None:
        pkg_map = build_package_map()
    attach_package_map(parser, pkg_map)
    return parser, pkg_map
