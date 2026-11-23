"""rm_executor 路径解析（**独立实现**，执行层不得反向依赖 rmctl 体验层）。

与 ``rm_hw_bridge/paths.py`` 同模式但**代码独立**：执行层（``rm_executor``）
既不允许反向依赖体验层（``rmctl``），也不允许依赖规划内核（``tesseract*``）——
这条边界的契约在 ``rm_hw_bridge/contract.yaml: rules.executor_must_not_import``。

提供三个入口：

* :func:`workspace_root` —— 向上查找仓库根（含 ``dualarm_ws/src/rm_description``
  与 ``workcell/``）；
* :func:`workcell_dir` —— 工位目录 ``<ws>/workcell``；
* :func:`workcell_config_path` —— 工位定义文件 ``<ws>/workcell/rm_workcell.yaml``；
* :func:`hw_bridge_src` —— 硬件适配层源码目录（用于 PYTHONPATH 提示，不自动改环境）。

本模块**零第三方依赖**、零 SDK、零 I/O 副作用（除路径判定外）。
"""
from __future__ import annotations

import os

__all__ = [
    "ENV_WS",
    "hw_bridge_src",
    "workcell_config_path",
    "workcell_dir",
    "workspace_root",
]

#: 显式指定工作区根的环境变量
ENV_WS = "RM_WS"

#: 仓库根判定标记
_SRC_MARKER = os.path.join("dualarm_ws", "src", "rm_description")
_WS_MARKER = "workcell"

#: 工位定义文件名
WORKCELL_CONFIG_NAME = "rm_workcell.yaml"


def _is_repo_root(path: str) -> bool:
    """是否为仓库根（同时含 ``dualarm_ws/src/rm_description`` 与 ``workcell/``）。"""
    return os.path.isdir(os.path.join(path, _SRC_MARKER)) and os.path.isdir(
        os.path.join(path, _WS_MARKER)
    )


def workspace_root(start: str = None) -> str:
    """向上查找工作区（仓库）根。

    优先级：显式 ``start``（且本身是合法仓库根）→ ``$RM_WS``（须是合法仓库根）
    → 从 ``start`` 向上逐级查找 → 兜底返回包路径上溯四级
    （``dualarm_ws/src/rm_executor/rm_executor`` → 仓库根）。
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
    # 兜底：从包路径上溯（dualarm_ws/src/rm_executor/rm_executor → 仓库根）
    return os.path.abspath(os.path.join(here, "..", "..", "..", ".."))


def workcell_dir(ws: str = None) -> str:
    """工位目录：``<ws>/workcell``。"""
    return os.path.join(workspace_root(ws), _WS_MARKER)


def workcell_config_path(ws: str = None) -> str:
    """工位定义文件：``<ws>/workcell/rm_workcell.yaml``。

    **执行配置的唯一来源**（``execution:`` 段），执行层不得在代码里新增第二处常量源。
    """
    return os.path.join(workcell_dir(ws), WORKCELL_CONFIG_NAME)


def hw_bridge_src(ws: str = None) -> str:
    """硬件适配层源码目录：``<ws>/dualarm_ws/src/rm_hw_bridge``。

    仅用于**提示**（next_action 里给出可粘贴的 PYTHONPATH），**不自动改环境**。
    """
    return os.path.join(
        workspace_root(ws), "dualarm_ws", "src", "rm_hw_bridge"
    )
