"""厂商动态库环境兜底（GLIBCXX_3.4.31+）。

背景：厂商 nanobind wheel 自带的 ``libboost_filesystem.so.1.88.0`` 需要
``GLIBCXX_3.4.31+``；本机系统 ``libstdc++.so.6`` 只有 3.4.30 时，
``import tesseract_robotics`` 会直接抛
``ImportError: version `GLIBCXX_3.4.31' not found``。

仓库自带 ``.deps/libstdcxx``（与 ``scripts/rmenv.sh`` 同一来源）。本模块在
import 厂商库**之前**用 ``ctypes`` 以 ``RTLD_GLOBAL`` 预加载它：两者 soname
相同（``libstdc++.so.6``），动态链接器会复用已加载的副本，从而让厂商 wheel
在**没有** ``source scripts/rmenv.sh``（即没有 LD_LIBRARY_PATH）的裸环境下
也能 import —— 这是「同一份代码在裸 shell / CI / 远程容器里行为一致」的
前提，不是绕过厂商依赖。

约定：

* **只读**：只引用仓库里已有的 ``.deps/libstdcxx``，不拷贝、不修改；
* **best-effort**：找不到或加载失败时静默返回 ``None``，由调用方沿用既有
  报错路径（rmctl 的 NEEDS_HUMAN + 可粘贴 next_action），绝不伪造能力。
"""
from __future__ import annotations

import ctypes
import glob
import os

from . import paths

#: 已尝试过预加载的目录 → 已加载的库文件（None 表示该目录无可用副本）
#: 进程内只做一次，避免重复 dlopen。
_loaded = {}


class VendorUnavailable(RuntimeError):
    """厂商规划内核（tesseract_robotics）不可用。

    裸环境缺 GLIBCXX_3.4.31+ 的 libstdc++ 时抛出；调用方应把它转成
    「NEEDS_HUMAN + reason + 可粘贴 next_action」（退出码 3），
    不得吞掉、也不得伪造模型/执行能力。
    """


def _candidate_dirs():
    """候选的 libstdcxx 目录（仓库自带优先，其次同仓库其它 checkout）。"""
    root = paths.workspace_root()
    out = []
    for base in (root,
                 os.path.dirname(root),
                 os.path.dirname(os.path.dirname(root))):
        out.append(os.path.join(base, ".deps", "libstdcxx"))
    seen, uniq = set(), []
    for d in out:
        if d not in seen:
            seen.add(d)
            uniq.append(d)
    return uniq


def ensure_libstdcxx():
    """确保厂商 wheel 所需的 libstdc++ 可用。

    返回：已预加载的库文件路径；无可用副本或加载失败时返回 ``None``。
    """
    for d in _candidate_dirs():
        if d in _loaded:
            return _loaded[d]
        if not os.path.isdir(d):
            continue
        libs = sorted(glob.glob(os.path.join(d, "libstdc++.so.6*")))
        if not libs:
            _loaded[d] = None
            continue
        for lib in libs:
            try:
                ctypes.CDLL(lib, mode=ctypes.RTLD_GLOBAL)
            except OSError:
                continue
            _loaded[d] = lib
            return lib
        _loaded[d] = None
    return None
