"""rm_safety 路径解析（**独立实现**，安全层不得反向依赖任何上层）。

安全层（``rm_safety``）位于 ``rm_hw_bridge``（硬件层）与 ``rm_executor``
（执行层）之间，依赖方向必须是::

    hardware(rm_hw_bridge)  <  safety(rm_safety)  <  executor(rm_executor)  <  experience(rmctl)

因此本模块**独立实现**工作区根定位，**不 import** ``rmctl`` /
``rm_executor`` / ``rm_hw_bridge``（理由同 ``rm_hw_bridge.paths``、
``rm_executor.paths``：各层不得反向依赖；同层之间也不共享路径代码，
避免一处改动同时牵动多层）。

本模块**零第三方依赖**、零 SDK、零 I/O 副作用（除路径判定外）。
"""
from __future__ import annotations

import os

__all__ = [
    "ENV_WS",
    "safety_dir",
    "workcell_config_path",
    "workcell_dir",
    "workspace_root",
]

#: 显式指定工作区根的环境变量
ENV_WS = "RM_WS"

#: 仓库根判定标记（同时含 dualarm_ws/src/rm_description 与 workcell/）
_SRC_MARKER = os.path.join("dualarm_ws", "src", "rm_description")
_WS_MARKER = "workcell"

#: 工位定义文件名
WORKCELL_CONFIG_NAME = "rm_workcell.yaml"


def _is_repo_root(path: str) -> bool:
    """是否为工作区根（含 ``dualarm_ws/src/rm_description`` 与 ``workcell/``）。"""
    return os.path.isdir(os.path.join(path, _SRC_MARKER)) and os.path.isdir(
        os.path.join(path, _WS_MARKER)
    )


def workspace_root(start: str = None) -> str:
    """向上查找工作区（仓库）根。

    优先级：显式 ``start``（须本身是合法仓库根）→ ``$RM_WS``（须是合法
    仓库根）→ 从 ``start`` 向上逐级查找 → 兜底返回包路径上溯四级
    （``dualarm_ws/src/rm_safety/rm_safety`` → 仓库根）。

    兜底**不静默假定**：调用方应检查返回值里是否含 ``workcell/``。
    """
    explicit = os.path.abspath(start) if start else ""
    if explicit and _is_repo_root(explicit):
        return explicit
    env = os.environ.get(ENV_WS)
    if env and _is_repo_root(os.path.abspath(env)):
        return os.path.abspath(env)
    here = explicit or os.path.abspath(os.path.dirname(__file__))
    cur = here
    while True:
        if _is_repo_root(cur):
            return cur
        parent = os.path.dirname(cur)
        if parent == cur:
            break
        cur = parent
    # 兜底：从包路径上溯（dualarm_ws/src/rm_safety/rm_safety → 仓库根）
    return os.path.abspath(os.path.join(here, "..", "..", "..", ".."))


def workcell_dir(ws: str = None) -> str:
    """工位目录 ``<ws>/workcell``。"""
    return os.path.join(workspace_root(ws), _WS_MARKER)


def workcell_config_path(ws: str = None) -> str:
    """工位定义文件 ``<ws>/workcell/rm_workcell.yaml``（安全配置的**唯一来源**）。"""
    return os.path.join(workcell_dir(ws), WORKCELL_CONFIG_NAME)


def safety_dir(ws: str = None) -> str:
    """安全数据目录 ``<ws>/workcell/safety``（错误码表等声明式文件）。"""
    return os.path.join(workcell_dir(ws), "safety")
