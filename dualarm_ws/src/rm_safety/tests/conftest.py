"""rm_safety 测试共用夹具（pytest）。

只使用 git 跟踪的仓库文件 + pytest 临时目录；
绝不依赖 .gitignore 目录（``build/``、``.deps/``、``.gsd/``）。
"""
from __future__ import annotations

import os
import sys

import pytest

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_SRC = os.path.abspath(os.path.join(TESTS_DIR, ".."))
REPO_ROOT = os.path.abspath(os.path.join(TESTS_DIR, "..", "..", "..", ".."))

# 安全层包源码在 sys.path 上（**只加本包**，不加 rmctl / rm_executor /
# rm_hw_bridge —— 安全层不得反向依赖，测试环境同样保持这条边界）
if PKG_SRC not in sys.path:
    sys.path.insert(0, PKG_SRC)


@pytest.fixture(scope="session")
def repo_root() -> str:
    """仓库根（workcell/ 与 dualarm_ws/ 都在这一层）。"""
    return REPO_ROOT


@pytest.fixture(scope="session")
def safety_pkg_root() -> str:
    """rm_safety 包根（dualarm_ws/src/rm_safety）。"""
    return PKG_SRC


@pytest.fixture(scope="session")
def error_codes_path(repo_root: str) -> str:
    """声明式错误码表（git 跟踪）。"""
    return os.path.join(repo_root, "workcell", "safety", "error_codes.yaml")


@pytest.fixture(scope="session")
def workcell_config_path(repo_root: str) -> str:
    """工位定义文件（安全配置唯一来源，git 跟踪）。"""
    return os.path.join(repo_root, "workcell", "rm_workcell.yaml")


@pytest.fixture()
def thresholds():
    """判据阈值。

    四个阈值取 S03 冻结值（跟随误差 2°/5°、断流 50 ms/200 ms）—— 与
    ``rm_executor/contract.py`` 一致，但**在此显式写出**，是为了让安全层的
    单测**不 import 执行层**（安全层不得反向依赖执行层）。
    """
    from rm_safety.policy import SafetyThresholds

    return SafetyThresholds(
        follow_warn_deg=2.0,
        follow_stop_deg=5.0,
        stream_warn_ms=50.0,
        stream_stop_ms=200.0,
    )


@pytest.fixture()
def error_code_map(error_codes_path: str):
    """仓库内的错误码表（git 跟踪）。"""
    from rm_safety.error_codes import load_error_codes

    return load_error_codes(error_codes_path)


@pytest.fixture()
def fake_ws(tmp_path):
    """构造一个最小「工作区根」布局（用于配置读取的正负例）。"""
    ws = tmp_path / "ws"
    (ws / "workcell").mkdir(parents=True)
    (ws / "dualarm_ws" / "src" / "rm_description").mkdir(parents=True)
    return ws
