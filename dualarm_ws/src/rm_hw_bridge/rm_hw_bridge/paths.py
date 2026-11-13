"""rm_hw_bridge 路径解析（**独立实现**，硬件层不得反向依赖 rmctl 体验层）。

与 ``rmctl/paths.py`` 同模式但**代码独立**：硬件层（``rm_hw_bridge``）不允许
反向依赖体验层（``rmctl``）——否则"厂商解耦"这条边界就破了（契约见
``contract.yaml: rules.hardware_must_not_import``）。

提供三个入口：

* :func:`workspace_root` —— 向上查找仓库根（含 ``dualarm_ws/src`` 与 ``workcell/``）；
* :func:`baselines_dir` —— 真机基线目录 ``<ws>/workcell/baselines``；
* :func:`joint_limits_path` —— 关节限值文件（实测优先，回退到 ``joint_limits.yaml``）；
* :func:`workcell_dir` / :func:`workcell_config_path` —— 工位目录与工位定义文件
  （``<ws>/workcell/rm_workcell.yaml``，夹爪配置的**唯一来源**）。
"""
from __future__ import annotations

import os

__all__ = [
    "ENV_WS",
    "WORKCELL_CONFIG_NAME",
    "baselines_dir",
    "joint_limits_path",
    "workcell_config_path",
    "workcell_dir",
    "workspace_root",
]

#: 显式指定工作区根的环境变量
ENV_WS = "RM_WS"

#: 仓库根的判定：同时含 dualarm_ws/src/rm_description 与 workcell/
_SRC_MARKER = os.path.join("dualarm_ws", "src", "rm_description")
_WS_MARKER = "workcell"

#: 工位定义文件名
WORKCELL_CONFIG_NAME = "rm_workcell.yaml"


def _is_repo_root(path: str) -> bool:
    return os.path.isdir(os.path.join(path, _SRC_MARKER)) and os.path.isdir(
        os.path.join(path, _WS_MARKER)
    )


def workspace_root(start: str = None) -> str:
    """向上查找工作区（仓库）根。

    优先级：``$RM_WS``（须是合法仓库根）→ 从 ``start`` 向上逐级查找 → 兜底
    返回 ``start`` 自身（调用方据此判断"未找到"，不静默假定）。
    """
    env = os.environ.get(ENV_WS)
    if env and _is_repo_root(os.path.abspath(env)):
        return os.path.abspath(env)
    here = os.path.abspath(start or os.path.dirname(__file__))
    cur = here
    while True:
        if _is_repo_root(cur):
            return cur
        parent = os.path.dirname(cur)
        if parent == cur:
            break
        cur = parent
    # 兜底：从包路径上溯（dualarm_ws/src/rm_hw_bridge/rm_hw_bridge → 仓库根）
    return os.path.abspath(os.path.join(here, "..", "..", "..", ".."))


def workcell_dir(ws: str = None) -> str:
    """工位目录：``<ws>/workcell``。"""
    return os.path.join(workspace_root(ws), _WS_MARKER)


def workcell_config_path(ws: str = None) -> str:
    """工位定义文件：``<ws>/workcell/rm_workcell.yaml``。

    **夹爪配置的唯一来源**（``gripper:`` 段）——硬件层不得在代码里另立
    一份夹爪常量源。
    """
    return os.path.join(workcell_dir(ws), WORKCELL_CONFIG_NAME)


def baselines_dir(ws: str = None) -> str:
    """真机基线目录：``<ws>/workcell/baselines``。"""
    return os.path.join(workspace_root(ws), _WS_MARKER, "baselines")


def joint_limits_path(ws: str = None) -> str:
    """关节限值文件路径。

    优先 ``<ws>/build/model/joint_limits.yaml``（实测值落盘处），
    回退 ``<ws>/dualarm_ws/src/rm_description/config/joint_limits.yaml``。
    """
    root = workspace_root(ws)
    built = os.path.join(root, "build", "model", "joint_limits.yaml")
    if os.path.exists(built):
        return built
    return os.path.join(
        root, "dualarm_ws", "src", "rm_description", "config", "joint_limits.yaml"
    )
