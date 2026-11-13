"""rm75_drake —— RM75-D 基于 Drake 的仿真与规划包（T02 骨架）。

版本标识约定
------------
``__version__`` 取自 ``versions.lock`` 的 ``baseline.drake``（当前为 ``"1.55.0"``），
即**项目基线标识**，与唯一版本真相源严格一致；它**不是**当前环境实装的 Drake 版本。
实装版本请用 :func:`rm75_drake.drake_compat.drake_version` 查询
（本机 Python 3.10 fallback 档实装的是 Drake 1.51.1）。

子模块
------
``paths``
    路径层（不依赖 cwd）。
``package_map``
    ``package://`` 解析的唯一入口。
``drake_compat``
    Drake 版本差异的薄封装。
"""

from __future__ import annotations

from . import drake_compat
from . import package_map
from . import paths

__all__ = ["__version__", "drake_compat", "package_map", "paths"]


def _read_baseline_version() -> str:
    """从 versions.lock 读取 baseline.drake 作为项目基线标识。"""
    import yaml

    with paths.versions_lock_path.open("r", encoding="utf-8") as fh:
        lock = yaml.safe_load(fh)
    return str(lock["baseline"]["drake"])


__version__ = _read_baseline_version()
