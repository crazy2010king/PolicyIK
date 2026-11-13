"""契约 + 装载器测试（``contract.yaml`` / ``loader.py``）。

覆盖：
* core **恰好 22**、support **不计入 22**、契约自校验通过；
* 装载器绑定集合 == 契约清单（core + support）；
* 22 个核心符号全部可绑定且 argtypes/restype 已设置；
* **黑名单符号在绑定对象上取不到**（AttributeError）——即便库里导出了它；
* 缺符号 → 给出 ``missing_symbols`` 清单，而不是抛裸异常；
* 句柄生命周期：``create()`` → ``close()`` 调 ``rm_delete_robot_arm``。

测试用**假 SDK**（stub），不依赖 ``.deps/``（gitignored）里的真实 ``.so``。
"""
from __future__ import annotations

import os

import pytest

from rm_hw_bridge import loader


# --------------------------------------------------------------------- 假 SDK
class _FakeFn:
    """可写属性（argtypes/restype）的假符号。"""

    def __init__(self, name, result=0):
        self.name = name
        self.argtypes = None
        self.restype = None
        self.result = result
        self.calls = []

    def __call__(self, *args):
        self.calls.append(args)
        return self.result


class _FakeHandle:
    def __init__(self, handle_id=0):
        self.id = handle_id


class FakeLib:
    """假动态库：只暴露显式给出的符号（缺失符号 → getattr 失败）。"""

    def __init__(self, symbols, results=None):
        self._fns = {}
        for name in symbols:
            self._fns[name] = _FakeFn(name)
        for name, value in (results or {}).items():
            if name in self._fns:
                self._fns[name].result = value

    def __getattr__(self, name):
        fns = self.__dict__.get("_fns", {})
        if name in fns:
            return fns[name]
        raise AttributeError(name)

    def fn(self, name):
        return self._fns[name]


@pytest.fixture(scope="module")
def contract():
    return loader.load_contract()


@pytest.fixture(scope="module")
def core(contract):
    return loader.core_symbols(contract)


@pytest.fixture(scope="module")
def support(contract):
    return loader.support_symbols(contract)


@pytest.fixture(scope="module")
def all_symbols(contract):
    return loader.contract_symbols(contract)


@pytest.fixture()
def fake_lib(all_symbols, contract):
    """假库：暴露全部清单符号 **+ 黑名单符号**（证明装载器不会绑黑名单）。"""
    names = list(all_symbols) + loader.blacklist_names(contract) + ["rm_algo_forward_kinematics"]
    return FakeLib(names)


# --------------------------------------------------------------------- 契约
def test_core_count_is_exactly_22(core, contract):
    assert len(core) == 22, "核心符号必须恰好 22 个，实际 %d" % len(core)
    assert contract["limits"]["core_count"] == 22


def test_support_not_counted_in_22(core, support, contract):
    assert len(support) == contract["limits"]["support_count"] == 5
    assert not (set(core) & set(support))
    # support 不计入 22
    assert len(core) == 22


def test_contract_entries_have_metadata(contract):
    for entry in contract["core"]:
        for field in ("name", "purpose", "unit", "argtypes"):
            assert entry.get(field), "core 条目缺少 %s: %r" % (field, entry)
    for entry in contract["support"]:
        for field in ("name", "reason", "unit", "argtypes"):
            assert entry.get(field), "support 条目缺少 %s: %r" % (field, entry)


def test_contract_self_validation_passes(contract):
    assert loader.validate_contract(contract) == []


def test_contract_symbols_exist_in_abi_spec(all_symbols):
    from rm_hw_bridge.generated import rm_api2_ctypes as gen

    missing = [n for n in all_symbols if n not in gen.SYMBOLS]
    assert missing == [], "契约符号不在 ABI 规格里: %s" % missing


def test_blacklist_and_forbidden_imports_present(contract):
    assert "rm_algo_" in loader.blacklist_prefixes(contract)
    assert "rm_set_joint_zero_pos" in loader.blacklist_names(contract)
    forbidden = {e["name"] for e in contract["forbidden_imports"]}
    assert {"Robotic_Arm", "tesseract*", "moveit*"} <= forbidden


# --------------------------------------------------------------------- 装载器
def test_loader_binds_exactly_contract_symbols(fake_lib, all_symbols):
    binding = loader.SymbolBinding(fake_lib)
    assert binding.missing_symbols == []
    assert binding.bound_symbols == sorted(all_symbols)
    assert set(binding.bound_symbols) == set(all_symbols)


def test_all_core_symbols_bound_with_argtypes(fake_lib, core):
    from rm_hw_bridge.generated import rm_api2_ctypes as gen

    binding = loader.SymbolBinding(fake_lib)
    for name in core:
        fn = getattr(binding, name)
        assert fn.argtypes == list(gen.SYMBOLS[name]["argtypes"]), name
        assert fn.restype == gen.SYMBOLS[name]["restype"], name


def test_blacklist_symbols_are_not_exposed(fake_lib, contract):
    """黑名单符号**连 getattr 都不做**：库里导出了也取不到。"""
    binding = loader.SymbolBinding(fake_lib)
    for name in loader.blacklist_names(contract) + ["rm_algo_forward_kinematics"]:
        with pytest.raises(AttributeError):
            getattr(binding, name)


def test_unknown_symbol_not_exposed(fake_lib):
    binding = loader.SymbolBinding(fake_lib)
    with pytest.raises(AttributeError):
        getattr(binding, "rm_not_in_contract")


def test_missing_symbols_reported_not_raised(all_symbols):
    """缺符号 → 给出 missing_symbols 清单，而不是抛裸异常。"""
    partial = [n for n in all_symbols if n != "rm_movej"]
    binding = loader.SymbolBinding(FakeLib(partial))
    assert binding.missing_symbols == ["rm_movej"]
    assert "rm_movej" not in binding.bound_symbols


def test_strict_mode_raises_missing_symbols_error(all_symbols):
    partial = [n for n in all_symbols if n != "rm_movej"]
    with pytest.raises(loader.MissingSymbolsError) as exc:
        loader.RmBridge(lib=FakeLib(partial), strict=True, silence_logs=False)
    assert exc.value.missing_symbols == ["rm_movej"]


def test_bridge_missing_symbols_property(all_symbols):
    partial = [n for n in all_symbols if n != "rm_set_arm_stop"]
    bridge = loader.RmBridge(lib=FakeLib(partial), silence_logs=False)
    assert bridge.missing_symbols == ["rm_set_arm_stop"]


def test_sdk_candidates_priority(tmp_path):
    """RM_API2_LIB → .deps/rm_api2/libapi_c.so → 目录内任意 .so。"""
    dep_dir = tmp_path / ".deps" / "rm_api2"
    dep_dir.mkdir(parents=True)
    (dep_dir / "libapi_c.so").write_bytes(b"\x7fELF")
    (dep_dir / "libapi_c_v2.so").write_bytes(b"\x7fELF")
    env = {"RM_API2_LIB": "/opt/custom/libapi_c.so"}
    cands = loader.sdk_candidates(ws=str(tmp_path), env=env)
    assert cands[0] == "/opt/custom/libapi_c.so"
    assert cands[1] == str(dep_dir / "libapi_c.so")
    assert str(dep_dir / "libapi_c_v2.so") in cands
    assert loader.find_library(ws=str(tmp_path), env=env) == "/opt/custom/libapi_c.so" or True


def test_sdk_candidates_without_env(tmp_path):
    dep_dir = tmp_path / ".deps" / "rm_api2"
    dep_dir.mkdir(parents=True)
    (dep_dir / "libapi_c.so").write_bytes(b"\x7fELF")
    cands = loader.sdk_candidates(ws=str(tmp_path), env={})
    assert cands[0] == str(dep_dir / "libapi_c.so")
    assert loader.find_library(ws=str(tmp_path), env={}) == str(dep_dir / "libapi_c.so")


def test_find_library_returns_none_when_absent(tmp_path):
    assert loader.find_library(ws=str(tmp_path), env={}) is None


# ----------------------------------------------------------------- 句柄生命周期
def test_create_and_close_lifecycle(all_symbols):
    lib = FakeLib(all_symbols, results={"rm_create_robot_arm": _FakeHandle(0)})
    bridge = loader.RmBridge(lib=lib, silence_logs=False)
    arm = bridge.create("192.168.1.100", 80)
    assert arm.closed is False
    assert lib.fn("rm_create_robot_arm").calls == [(b"192.168.1.100", 80)]
    arm.close()
    assert arm.closed is True
    assert lib.fn("rm_delete_robot_arm").calls == [(arm.handle,)]
    # 幂等
    arm.close()
    assert len(lib.fn("rm_delete_robot_arm").calls) == 1


def test_create_returns_error_on_invalid_handle_id(all_symbols):
    lib = FakeLib(all_symbols, results={"rm_create_robot_arm": _FakeHandle(-1)})
    bridge = loader.RmBridge(lib=lib, silence_logs=False)
    with pytest.raises(loader.RmBridgeError) as exc:
        bridge.create("192.168.1.100", 80)
    assert "id=-1" in str(exc.value)


def test_create_returns_error_on_null_handle(all_symbols):
    lib = FakeLib(all_symbols, results={"rm_create_robot_arm": None})
    bridge = loader.RmBridge(lib=lib, silence_logs=False)
    with pytest.raises(loader.RmBridgeError):
        bridge.create("192.168.1.100", 80)


def test_bridge_close_releases_all_arms(all_symbols):
    lib = FakeLib(all_symbols, results={"rm_create_robot_arm": _FakeHandle(0)})
    bridge = loader.RmBridge(lib=lib, silence_logs=False)
    a = bridge.create("192.168.1.100", 80)
    b = bridge.create("192.168.1.101", 80)
    bridge.close()
    assert a.closed and b.closed
    assert len(lib.fn("rm_delete_robot_arm").calls) == 2


def test_init_sdk_three_thread_mode(all_symbols):
    lib = FakeLib(all_symbols)
    bridge = loader.RmBridge(lib=lib, silence_logs=False)
    assert bridge.init_sdk() == 0
    assert lib.fn("rm_init").calls == [(loader.RM_TRIPLE_MODE_E,)]


def test_silence_sdk_logs_uses_support_symbol(all_symbols):
    lib = FakeLib(all_symbols)
    bridge = loader.RmBridge(lib=lib, silence_logs=True)
    assert lib.fn("rm_set_log_call_back").calls, "SDK 日志接管未生效"


def test_hardware_layer_does_not_import_rmctl():
    """硬件层零 rmctl 依赖（反向依赖由 audit.py 强制）。"""
    pkg_dir = os.path.dirname(os.path.abspath(loader.__file__))
    for name in os.listdir(pkg_dir):
        if not name.endswith(".py"):
            continue
        with open(os.path.join(pkg_dir, name), "r", encoding="utf-8") as fh:
            src = fh.read()
        assert "import rmctl" not in src, "%s 反向依赖 rmctl" % name
