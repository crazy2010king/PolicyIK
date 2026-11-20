"""rm_hw_bridge 测试共用夹具（pytest）。

只使用 git 跟踪的仓库文件 + pytest 临时目录；
绝不依赖 .gitignore 目录（build/、.deps/、.gsd/）。
"""
from __future__ import annotations

import importlib.util
import json
import os
import sys

import pytest

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_SRC = os.path.abspath(os.path.join(TESTS_DIR, ".."))
REPO_ROOT = os.path.abspath(os.path.join(TESTS_DIR, "..", "..", "..", ".."))

if PKG_SRC not in sys.path:
    sys.path.insert(0, PKG_SRC)


def _load_module(path: str, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(autouse=True)
def _reset_realtime_state():
    """每个用例前复位**进程级** realtime 单例（跨包测试隔离）。

    ``rm_hw_bridge.bridge._REALTIME`` 是进程级单例（SDK 回调全局注册）。
    其它包（如 ``rm_executor`` 的硬件接线用例）会直接对它 ``push()`` 且不还原，
    导致本包的 ``frames == 0`` 断言被污染。这里在每个用例前复位，
    使本包断言与 pytest 收集顺序、外部污染无关。
    """
    from rm_hw_bridge import bridge as hw_bridge

    hw_bridge.reset_realtime_state()
    yield
    hw_bridge.reset_realtime_state()


@pytest.fixture(scope="session")
def repo_root() -> str:
    """仓库根（workcell/ 与 dualarm_ws/ 都在这一层）。"""
    return REPO_ROOT


@pytest.fixture(scope="session")
def pkg_root() -> str:
    """rm_hw_bridge 包根（dualarm_ws/src/rm_hw_bridge）。"""
    return PKG_SRC


@pytest.fixture(scope="session")
def spec_path(pkg_root: str) -> str:
    """ABI 唯一真值路径。"""
    return os.path.join(pkg_root, "rm_hw_bridge", "abi_spec.yaml")


@pytest.fixture(scope="session")
def spec(spec_path: str) -> dict:
    """解析后的 ABI 规格。"""
    import rm_hw_bridge.codegen as codegen

    return codegen.load_spec(spec_path)


@pytest.fixture(scope="session")
def generated_module(pkg_root: str):
    """生成的 ctypes 绑定模块。"""
    path = os.path.join(pkg_root, "rm_hw_bridge", "generated", "rm_api2_ctypes.py")
    return _load_module(path, "_rm_hw_bridge_generated_under_test")


@pytest.fixture(scope="session")
def layout_golden(pkg_root: str) -> dict:
    """布局黄金表（生成物）。"""
    path = os.path.join(pkg_root, "rm_hw_bridge", "generated", "layout_golden.json")
    with open(path, "r", encoding="utf-8") as fh:
        return json.load(fh)


@pytest.fixture(scope="session")
def vendor_include_dir(pkg_root: str) -> str:
    """随仓库分发的厂商头文件目录。"""
    return os.path.join(pkg_root, "vendor", "rm_api2", "include")
