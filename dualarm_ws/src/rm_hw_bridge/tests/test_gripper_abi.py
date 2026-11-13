"""夹爪 ABI 扩容（S05/T1）的**契约级**测试。

覆盖三组断言：

* **布局** —— ``rm_gripper_state_t``（7 个 int）与 ``rm_peripheral_read_write_params_t``
  （4 个 int）的字段名 / 顺序 / 类型 / 偏移与 ``abi_spec.yaml`` 一致，
  且与 gcc 实测的 C 参考布局一致（见 ``test_codegen_layout.py`` 的 C 编译轨）。
* **原型** —— 16 个新增符号（夹爪快路径 6 + Modbus 主路径 6 + RM Plus 2 + 工具端电压 2）
  的参数与返回**逐字**符合厂商头文件：``bool`` 形参必须是 ``ctypes.c_bool``，
  ``params`` 形参必须**按值**传递（不是指针）。
* **一致性** —— ``contract.yaml`` 里每条 support 条目的 ``argtypes`` 必须与
  ``abi_spec.yaml`` 的原型**逐字段一致**（两处真值不允许漂移）。

外加负例：契约重复符号、契约引用 ABI 里不存在的符号、白黑名单交叉、
畸形 ABI 规格（未知类型）都必须被**明确报错**而不是静默通过。
"""
from __future__ import annotations

import copy
import ctypes
import os

import pytest
import yaml

from rm_hw_bridge import codegen, loader
from rm_hw_bridge.generated import rm_api2_ctypes as gen

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SPEC_PATH = os.path.join(PKG_ROOT, "rm_hw_bridge", "abi_spec.yaml")
CONTRACT_PATH = os.path.join(PKG_ROOT, "rm_hw_bridge", "contract.yaml")

#: 新增的 16 个支持层符号（夹爪快路径 / Modbus 主路径 / RM Plus / 工具端电压）
GRIPPER_FAST_PATH = (
    "rm_set_gripper_route",
    "rm_set_gripper_release",
    "rm_set_gripper_pick",
    "rm_set_gripper_pick_on",
    "rm_set_gripper_position",
    "rm_get_gripper_state",
)
MODBUS_MAIN_PATH = (
    "rm_set_modbus_mode",
    "rm_close_modbus_mode",
    "rm_read_holding_registers",
    "rm_read_input_registers",
    "rm_write_registers",
    "rm_write_single_register",
)
RM_PLUS_PATH = (
    "rm_set_rm_plus_mode",
    "rm_get_rm_plus_mode",
)
TOOL_VOLTAGE_PATH = (
    "rm_set_tool_voltage",
    "rm_get_tool_voltage",
)
NEW_SUPPORT_SYMBOLS = (
    GRIPPER_FAST_PATH + MODBUS_MAIN_PATH + RM_PLUS_PATH + TOOL_VOLTAGE_PATH
)


@pytest.fixture(scope="module")
def spec():
    with open(SPEC_PATH, "r", encoding="utf-8") as fh:
        return yaml.safe_load(fh)


@pytest.fixture(scope="module")
def contract():
    with open(CONTRACT_PATH, "r", encoding="utf-8") as fh:
        return yaml.safe_load(fh)


# ---------------------------------------------------------------------------
# 布局
# ---------------------------------------------------------------------------


def test_gripper_state_struct_matches_vendor(spec):
    """rm_gripper_state_t：7 个 int 字段，名字与顺序逐字来自 rm_define.h。"""
    expected = [
        "enable_state", "status", "error", "mode",
        "current_force", "temperature", "actpos",
    ]
    fields = spec["structs"]["rm_gripper_state_t"]["fields"]
    assert [f["name"] for f in fields] == expected
    assert all(f["type"] == "int" for f in fields)

    cls = gen.rm_gripper_state_t
    names = [name for name, _ in cls._fields_]
    assert names == expected
    assert ctypes.sizeof(cls) == 28
    for index, name in enumerate(expected):
        assert getattr(cls, name).offset == index * 4, "%s 偏移不对" % name


def test_peripheral_read_write_params_struct_matches_vendor(spec):
    """rm_peripheral_read_write_params_t：4 个 int 字段（port/address/device/num）。"""
    expected = ["port", "address", "device", "num"]
    fields = spec["structs"]["rm_peripheral_read_write_params_t"]["fields"]
    assert [f["name"] for f in fields] == expected
    assert all(f["type"] == "int" for f in fields)

    cls = gen.rm_peripheral_read_write_params_t
    assert ctypes.sizeof(cls) == 16
    for index, name in enumerate(expected):
        assert getattr(cls, name).offset == index * 4, "%s 偏移不对" % name


def test_gripper_structs_present_in_layout_golden():
    """黄金表必须覆盖两个新结构体（漏一个 = 漏校验）。"""
    import json

    golden_path = os.path.join(PKG_ROOT, "rm_hw_bridge", "generated", "layout_golden.json")
    with open(golden_path, "r", encoding="utf-8") as fh:
        golden = json.load(fh)
    structs = golden["structs"]
    assert structs["rm_gripper_state_t"]["size"] == 28
    assert structs["rm_peripheral_read_write_params_t"]["size"] == 16
    assert structs["rm_gripper_state_t"]["fields"]["error"] == 8


# ---------------------------------------------------------------------------
# 原型
# ---------------------------------------------------------------------------


def test_new_symbols_present_in_symbol_table():
    """16 个新增符号必须都在生成的符号表里。"""
    for name in NEW_SUPPORT_SYMBOLS:
        assert name in gen.SYMBOLS, "符号表缺少 %s" % name


def test_gripper_block_flag_is_bool():
    """夹爪接口的 block 形参是 bool（不是 int）。"""
    for name, arg_index in (
        ("rm_set_gripper_release", 2),
        ("rm_set_gripper_pick", 3),
        ("rm_set_gripper_pick_on", 3),
        ("rm_set_gripper_position", 2),
    ):
        argtypes = gen.SYMBOLS[name]["argtypes"]
        assert argtypes[0] is ctypes.POINTER(gen.rm_robot_handle)
        assert argtypes[arg_index] is ctypes.c_bool, "%s 的 block 形参不是 bool" % name


def test_gripper_state_is_out_pointer():
    """rm_get_gripper_state 的 state 形参是 rm_gripper_state_t*。"""
    argtypes = gen.SYMBOLS["rm_get_gripper_state"]["argtypes"]
    assert argtypes[0] is ctypes.POINTER(gen.rm_robot_handle)
    assert argtypes[1] is ctypes.POINTER(gen.rm_gripper_state_t)
    assert gen.SYMBOLS["rm_get_gripper_state"]["restype"] is ctypes.c_int


def test_modbus_params_are_passed_by_value():
    """四个 rm_read_*/rm_write_* 的 params 形参**按值**传递（不是指针）。"""
    for name in (
        "rm_read_holding_registers",
        "rm_read_input_registers",
        "rm_write_registers",
        "rm_write_single_register",
    ):
        sig = gen.SYMBOLS[name]
        assert sig["by_value"] == ["params"], "%s 未标记按值传参" % name
        assert sig["argtypes"][1] is gen.rm_peripheral_read_write_params_t, (
            "%s 的 params 是指针而不是按值" % name
        )
    # 读/多写的数据缓冲是 int*；写单个寄存器的数据是**按值** int
    for name in ("rm_read_holding_registers", "rm_read_input_registers", "rm_write_registers"):
        assert gen.SYMBOLS[name]["argtypes"][2] is ctypes.POINTER(ctypes.c_int), name
    assert gen.SYMBOLS["rm_write_single_register"]["argtypes"][2] is ctypes.c_int


def test_rm_plus_and_tool_voltage_shapes():
    """RM Plus 与工具端电压接口：出参是 int*，入参是 int。"""
    assert gen.SYMBOLS["rm_set_rm_plus_mode"]["argtypes"][1] is ctypes.c_int
    assert gen.SYMBOLS["rm_get_rm_plus_mode"]["argtypes"][1] is ctypes.POINTER(ctypes.c_int)
    assert gen.SYMBOLS["rm_set_tool_voltage"]["argtypes"][1] is ctypes.c_int
    assert gen.SYMBOLS["rm_get_tool_voltage"]["argtypes"][1] is ctypes.POINTER(ctypes.c_int)


def test_gripper_symbols_are_not_blacklisted():
    """夹爪/Modbus 符号**不得**与黑名单交叉（黑名单仍 13 个 + rm_algo_ 前缀）。"""
    contract = loader.load_contract()
    blacklist = set(loader.blacklist_names(contract))
    assert blacklist.isdisjoint(set(NEW_SUPPORT_SYMBOLS))
    assert len(blacklist) == 13
    assert loader.blacklist_prefixes(contract) == ["rm_algo_"]


# ---------------------------------------------------------------------------
# 契约 ↔ ABI 一致性
# ---------------------------------------------------------------------------


def _parse_argtypes(text):
    """把契约里的 ``"a: int, b: int* -> int"`` 解析成 (参数列表, 返回类型)。"""
    assert "->" in text, "argtypes 缺少返回类型: %r" % text
    params_text, ret_text = text.split("->", 1)
    params = []
    for chunk in params_text.split(","):
        chunk = chunk.strip()
        if not chunk:
            continue
        assert ":" in chunk, "argtypes 片段缺少类型: %r" % chunk
        name, type_text = chunk.split(":", 1)
        params.append((name.strip(), type_text.strip()))
    return params, ret_text.strip()


@pytest.mark.parametrize("entry_name", list(NEW_SUPPORT_SYMBOLS))
def test_contract_argtypes_match_abi_spec(spec, contract, entry_name):
    """契约里每个新增 support 条目的 argtypes 必须与 ABI 原型逐字段一致。"""
    entries = {e["name"]: e for e in contract["support"]}
    assert entry_name in entries, "契约 support 段缺少 %s" % entry_name
    entry = entries[entry_name]

    params, ret_text = _parse_argtypes(entry["argtypes"])
    fn = spec["functions"][entry_name]
    spec_params = [(a["name"], str(a["type"]).strip()) for a in fn.get("args") or []]
    assert params == spec_params, "%s 的 argtypes 与 ABI 规格不一致" % entry_name
    assert ret_text == str(fn["ret"]).strip(), "%s 的返回类型与 ABI 规格不一致" % entry_name


def test_all_support_entries_have_reason_and_unit(contract):
    """support 条目必须写明理由与单位（21 个全部要写）。"""
    for entry in contract["support"]:
        assert entry.get("name"), "support 条目缺少 name"
        assert entry.get("reason"), "support 条目缺少 reason: %s" % entry.get("name")
        assert entry.get("unit"), "support 条目缺少 unit: %s" % entry.get("name")
        assert entry.get("argtypes"), "support 条目缺少 argtypes: %s" % entry.get("name")


def test_contract_self_validation_is_clean():
    """契约自校验必须返回空问题列表（白名单全部在 ABI 符号表内）。"""
    assert loader.validate_contract() == []


# ---------------------------------------------------------------------------
# 负例
# ---------------------------------------------------------------------------


def test_negative_duplicate_support_symbol_is_reported(contract):
    """负例：support 段出现重复符号必须被契约自校验抓住。"""
    broken = copy.deepcopy(contract)
    broken["support"].append(dict(broken["support"][0]))
    problems = loader.validate_contract(broken)
    assert any("重复" in p for p in problems), problems


def test_negative_unknown_symbol_in_contract_is_reported(contract):
    """负例：契约写了 ABI 里不存在的符号必须被自校验抓住。"""
    broken = copy.deepcopy(contract)
    broken["support"].append({
        "name": "rm_not_exist_symbol",
        "reason": "负例",
        "unit": "-",
        "argtypes": "handle: rm_robot_handle* -> int",
    })
    broken["limits"]["support_count"] = len(broken["support"])
    problems = loader.validate_contract(broken)
    assert any("不在 ABI 规格符号表" in p for p in problems), problems


def test_negative_blacklist_crossing_is_reported(contract):
    """负例：白名单符号落进黑名单必须被自校验抓住（交叉即破损）。"""
    broken = copy.deepcopy(contract)
    broken["blacklist"]["symbols"].append("rm_set_gripper_pick")
    problems = loader.validate_contract(broken)
    assert any("交叉" in p for p in problems), problems


def test_negative_malformed_gripper_spec_is_rejected(spec):
    """负例：夹爪原型的类型写错时，codegen 必须拒绝（不能静默生成错绑定）。"""
    broken = copy.deepcopy(spec)
    broken["functions"]["rm_set_gripper_pick"]["args"][1]["type"] = "rm_not_exist_t"
    with pytest.raises(codegen.SpecError):
        codegen.resolve_type("rm_not_exist_t", broken)


def test_negative_gripper_symbol_not_bound_when_absent():
    """负例：动态库缺夹爪符号时，装载器必须报告 missing_symbols 而不是抛裸异常。"""

    class FakeLib:
        def __init__(self):
            for name in loader.contract_symbols():
                if name in NEW_SUPPORT_SYMBOLS:
                    continue
                setattr(self, name, lambda *a, **k: 0)

    binding = loader.SymbolBinding(FakeLib())
    missing = binding.missing_symbols
    assert set(missing) == set(NEW_SUPPORT_SYMBOLS), missing
    # 夹爪符号既不在绑定表里，也不能通过属性访问拿到
    assert "rm_set_gripper_pick" not in binding.bound_symbols
    with pytest.raises(AttributeError):
        getattr(binding, "rm_set_gripper_pick")
