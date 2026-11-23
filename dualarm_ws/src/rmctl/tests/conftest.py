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
# 硬件适配层 rm_hw_bridge（体验层 rmctl 依赖硬件层，方向合法）
HW_SRC = os.path.join(REPO_ROOT, "dualarm_ws", "src", "rm_hw_bridge")
# 执行层 rm_executor（rmctl run --executor rm_executor / --live 走它，方向合法）
EXEC_SRC = os.path.join(REPO_ROOT, "dualarm_ws", "src", "rm_executor")
# 安全层 rm_safety（rmctl safety stop|status 是闸门客户端，方向合法：
# experience(rmctl) → safety(rm_safety)；安全层**不**反向依赖 rmctl）
SAFETY_SRC = os.path.join(REPO_ROOT, "dualarm_ws", "src", "rm_safety")

if PKG_SRC not in sys.path:
    sys.path.insert(0, PKG_SRC)
if HW_SRC not in sys.path:
    sys.path.insert(0, HW_SRC)
if EXEC_SRC not in sys.path:
    sys.path.insert(0, EXEC_SRC)
if SAFETY_SRC not in sys.path:
    sys.path.insert(0, SAFETY_SRC)

# 厂商 nanobind wheel 需要 GLIBCXX_3.4.31+（本机系统 libstdc++ 只有
# 3.4.30）。pytest 在 import 任何测试模块**之前**加载 conftest，这里提前
# 预加载仓库自带的 .deps/libstdcxx（只读引用，与 scripts/rmenv.sh 同源），
# 保证裸环境（无 LD_LIBRARY_PATH）下厂商规划内核同样可用。
# 预加载失败时静默跳过：rmctl 会沿用「厂商内核不可用 → 退出码 3 + reason +
# 可粘贴 next_action」的诚实降级路径（见 rmctl/vendor_env.py）。
from rmctl.vendor_env import ensure_libstdcxx  # noqa: E402

ensure_libstdcxx()


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
