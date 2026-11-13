"""rmctl 测试共用夹具（pytest）。

只使用 git 跟踪的仓库文件 + pytest 临时目录，绝不依赖 .gitignore 目录
（build/、.deps/、.gsd/）。
"""
import os
import shutil
import sys

import pytest

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(TESTS_DIR, "..", "..", "..", ".."))
PKG_SRC = os.path.join(REPO_ROOT, "dualarm_ws", "src", "rmctl")

if PKG_SRC not in sys.path:
    sys.path.insert(0, PKG_SRC)


@pytest.fixture(scope="session")
def repo_root():
    """仓库根（workcell/ 与 dualarm_ws/ 都在这一层）。"""
    return REPO_ROOT


@pytest.fixture(scope="session")
def ws(repo_root):
    """工作区根：仓库根本身（与 scripts/rmenv.sh 里 RM_WS 一致）。"""
    return repo_root


@pytest.fixture
def tmp_workspace(tmp_path, repo_root):
    """一个可写的临时工作区副本：含 workcell/baselines/ 与 joint_limits.yaml。

    布局与真实仓库一致，便于直接调用 paths.*(ws) 系列函数。
    """
    wsdir = tmp_path / "ws"
    bdir = wsdir / "workcell" / "baselines"
    bdir.mkdir(parents=True)
    src_b = os.path.join(repo_root, "workcell", "baselines")
    for name in sorted(os.listdir(src_b)):
        if name.endswith(".yaml"):
            shutil.copyfile(os.path.join(src_b, name), str(bdir / name))
    limits_dst = wsdir / "dualarm_ws" / "src" / "rm_description" / "config"
    limits_dst.mkdir(parents=True)
    shutil.copyfile(
        os.path.join(repo_root, "dualarm_ws", "src", "rm_description", "config",
                     "joint_limits.yaml"),
        str(limits_dst / "joint_limits.yaml"))
    return str(wsdir)


@pytest.fixture
def baseline_dir(tmp_workspace):
    """临时工作区里的基线目录。"""
    return os.path.join(tmp_workspace, "workcell", "baselines")
