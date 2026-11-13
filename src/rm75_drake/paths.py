"""路径层（T02）。

约定（S01–S07 全量复用，禁止各切片各写一份路径推导）：

* 所有路径都从**本文件**推导，**不依赖 cwd**（进程在任意工作目录下调用都得到同一组路径）。
* 仓库根 = ``Path(__file__).resolve().parents[2]``（本文件位于 ``<root>/src/rm75_drake/paths.py``）。
* 模型搜索路径可被环境变量 :data:`MODEL_PATH_ENV_VAR` 覆盖；多个路径用 ``:``（``os.pathsep``）分隔。
  未设置时退回仓库内 ``models/``。
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import List

__all__ = [
    "MODEL_PATH_ENV_VAR",
    "REPO_ROOT_ENV_VAR",
    "REPO_ROOT",
    "models_dir",
    "scenarios_dir",
    "vendor_dir",
    "third_party_dir",
    "versions_lock_path",
    "model_search_paths",
    "resolve_repo_root",
]

#: 覆盖模型搜索路径的环境变量（多路径用 ":" 分隔）。
MODEL_PATH_ENV_VAR = "RM75_DRAKE_MODEL_PATH"

#: 当包被安装到 site-packages（不在仓库内）时，用它指明仓库根。
REPO_ROOT_ENV_VAR = "RM75_DRAKE_ROOT"


def resolve_repo_root() -> Path:
    """返回仓库根目录（绝对路径）。

    优先使用源码布局推导（``parents[2]``）；若该位置不像仓库（例如包被 pip 安装到
    site-packages），退回到环境变量 :data:`REPO_ROOT_ENV_VAR`；两者都不可用时抛错，
    绝不静默回退到 cwd（cwd 依赖是本项目明令禁止的）。
    """
    candidate = Path(__file__).resolve().parents[2]
    if (candidate / "src").is_dir() and (candidate / "versions.lock").is_file():
        return candidate

    env_root = os.environ.get(REPO_ROOT_ENV_VAR, "").strip()
    if env_root:
        return Path(env_root).expanduser().resolve()

    raise RuntimeError(
        "无法定位仓库根：既没有 src/ + versions.lock 的源码布局，"
        f"也未设置 {REPO_ROOT_ENV_VAR} 环境变量。"
    )


REPO_ROOT = resolve_repo_root()

#: 仓库内模型根目录（package:// 的默认来源）。
models_dir = REPO_ROOT / "models"

#: 场景描述目录（S02 填充正式 schema）。
scenarios_dir = REPO_ROOT / "scenarios"

#: 本机离线 wheel 镜像目录（不入 git，见 .gitignore）。
vendor_dir = REPO_ROOT / "vendor"

#: 上游资产归档目录（归档文件不入 git，清单入 git）。
third_party_dir = REPO_ROOT / "third_party"

#: 唯一版本真相源。
versions_lock_path = REPO_ROOT / "versions.lock"


def model_search_paths() -> List[Path]:
    """返回模型搜索路径列表（有序）。

    * 若设置了 :data:`MODEL_PATH_ENV_VAR`，则**覆盖**默认路径，按 ":" 顺序返回；
    * 否则返回 ``[models_dir]``。

    每次调用都重新读取环境变量（不在 import 时缓存），便于测试与运行期切换。
    """
    raw = os.environ.get(MODEL_PATH_ENV_VAR, "")
    overrides = [Path(p).expanduser() for p in raw.split(os.pathsep) if p.strip()]
    if overrides:
        return overrides
    return [models_dir]
