"""S13-T1：固件/软件版本 SDK 读数路径（``rm_get_arm_software_info``）。

覆盖：

* **(a) 契约名单** —— 新符号在 **support** 段（**不在 core**）；``core_count == 22``
  保持不变；``support_count == len(support)`` 严格一致；
* **(b) loader 护栏** —— 清单外符号（如 ``rm_get_joint_software_version``）
  **仍然抛 AttributeError**（白名单护栏不得被削弱）；
* **(c) ABI 布局** —— ``ctypes.sizeof(rm_arm_software_version_t) == 215``，
  各字段偏移与厂商头文件 ``rm_define.h`` **逐字一致**；
* **(d) 诚实降级** —— 未连接时 ``ArmLink.arm_software_info()`` 返回结构化
  error（**不抛异常、不伪造版本**）；SDK 返回空串时版本字段为 ``None``；
* **(e) 黑名单约束** —— 该符号既不在 core，也不在 13 个黑名单符号与
  ``rm_algo_`` 前缀族里。

全程**离线**（假 SDK），零真机、零运动、零下发。
"""
from __future__ import annotations

import ctypes

import pytest

from rm_hw_bridge import bridge, errors, loader
from rm_hw_bridge.generated import rm_api2_ctypes as gen

#: 新增的只读版本读数符号
SYMBOL = "rm_get_arm_software_info"

#: 厂商头文件 rm_define.h 的 ``rm_arm_software_version_t`` 布局（全 char 数组
#: ⇒ 天然对齐 1，结构体无填充字节；size = 20+10+20+40+5+40+40+40 = 215）
EXPECTED_SIZE = 215
EXPECTED_OFFSETS = {
    "product_version": 0,
    "robot_controller_version": 20,
    "algorithm_info": 30,
    "ctrl_info": 50,
    "dynamic_info": 90,
    "plan_info": 95,
    "com_info": 135,
    "program_info": 175,
}

#: 13 个黑名单符号（契约原文）
EXPECTED_BLACKLIST = {
    "rm_movep_canfd",
    "rm_movep_follow",
    "rm_movel",
    "rm_moves",
    "rm_movec",
    "rm_movej_p",
    "rm_set_pos_step",
    "rm_set_ort_step",
    "rm_set_teach_frame",
    "rm_set_manual_tool_frame",
    "rm_change_tool_frame",
    "rm_update_tool_frame",
    "rm_set_joint_zero_pos",
}


# --------------------------------------------------------------------- 假 SDK
class FakeHandle:
    def __init__(self, handle_id: int = 1) -> None:
        self.id = handle_id


class FakeLib:
    """假动态库：只实现建连 + 版本读数所需的少量符号。"""

    def __init__(self, handle_id: int = 1, rc: int = 0, raises: bool = False,
                 payload=None):
        self.calls = []
        self.handle = FakeHandle(handle_id)
        self.rc = rc
        self.raises = raises
        self.payload = payload if payload is not None else {}
        self._install()

    def _install(self):
        lib = self

        def rm_init(mode):
            lib.calls.append(("rm_init", mode))
            return 0

        def rm_create_robot_arm(ip, port):
            lib.calls.append(("rm_create_robot_arm", ip, port))
            return lib.handle

        def rm_delete_robot_arm(handle):
            lib.calls.append(("rm_delete_robot_arm",))
            return 0

        def rm_set_realtime_push(handle, config):
            lib.calls.append(("rm_set_realtime_push",))
            return 0

        def rm_realtime_arm_state_call_back(cb):
            lib.calls.append(("rm_realtime_arm_state_call_back",))
            return None

        def rm_set_log_call_back(cb, level):
            lib.calls.append(("rm_set_log_call_back",))
            return None

        def rm_set_timeout(timeout):
            lib.calls.append(("rm_set_timeout", timeout))
            return None

        def rm_get_arm_software_info(handle, info_ptr):
            lib.calls.append(("rm_get_arm_software_info",))
            if lib.raises:
                raise RuntimeError("fake rm_get_arm_software_info 失败")
            if lib.rc != 0:
                return lib.rc
            st = info_ptr.contents
            for field, value in lib.payload.items():
                if field == "algorithm_info":
                    st.algorithm_info.version = value
                elif field == "ctrl_info":
                    st.ctrl_info.version = value.get("version", b"")
                    st.ctrl_info.build_time = value.get("build_time", b"")
                elif field == "dynamic_info":
                    st.dynamic_info.model_version = value
                elif field == "plan_info":
                    st.plan_info.version = value.get("version", b"")
                    st.plan_info.build_time = value.get("build_time", b"")
                elif field == "com_info":
                    st.com_info.version = value.get("version", b"")
                elif field == "program_info":
                    st.program_info.version = value.get("version", b"")
                else:
                    setattr(st, field, value)
            return 0

        for name, fn in list(locals().items()):
            if name.startswith("rm_") and callable(fn):
                setattr(self, name, fn)

    def calls_of(self, name):
        return [c for c in self.calls if c[0] == name]


@pytest.fixture()
def fake_lib():
    return FakeLib()


@pytest.fixture()
def link(fake_lib):
    link_obj = bridge.connect("192.168.1.18", 8080, lib=fake_lib)
    assert link_obj.status == errors.STATUS_OK, link_obj.reason
    return link_obj


# ------------------------------------------------------- (a) 契约名单
def test_symbol_is_in_support_and_not_in_core():
    """新符号在 **support** 段（不计入 core 22）。"""
    contract = loader.load_contract()
    core = loader.core_symbols(contract)
    support = loader.support_symbols(contract)

    assert SYMBOL in support, "新符号必须在 support 名单里"
    assert SYMBOL not in core, "新符号不得进入 core 名单"
    assert len(core) == 22, "core 名单被改动：%d" % len(core)
    assert int((contract.get("limits") or {}).get("core_count")) == 22
    assert int((contract.get("limits") or {}).get("support_count")) == len(support)
    assert loader.validate_contract(contract) == []


def test_symbol_is_bound_through_loader(fake_lib):
    """符号可经 loader 绑定（真实读数在 S13-T2 真机窗口完成）。"""
    binding = loader.SymbolBinding(fake_lib)
    assert binding.has(SYMBOL)
    assert SYMBOL in binding.bound_symbols


# ------------------------------------------------------- (b) loader 护栏
def test_negative_off_contract_symbol_still_raises_attribute_error(fake_lib):
    """负例：清单外符号仍抛 AttributeError（护栏不得被削弱）。"""
    # 假库里**故意**多出一个清单外符号
    fake_lib.rm_get_joint_software_version = lambda *a, **k: 0
    binding = loader.SymbolBinding(fake_lib)

    with pytest.raises(AttributeError):
        getattr(binding, "rm_get_joint_software_version")
    with pytest.raises(AttributeError):
        getattr(binding, "rm_set_joint_zero_pos")  # 黑名单符号


# ------------------------------------------------------- (c) ABI 布局
def test_struct_layout_matches_vendor_header():
    """``rm_arm_software_version_t`` 布局与 ``rm_define.h`` 逐字一致（size 215）。"""
    cls = gen.rm_arm_software_version_t
    assert ctypes.sizeof(cls) == EXPECTED_SIZE, "size 与厂商头文件不一致"
    for fname, offset in EXPECTED_OFFSETS.items():
        assert getattr(cls, fname).offset == offset, "%s offset 不一致" % fname


def test_nested_struct_layout_matches_vendor_header():
    """嵌套结构体（算法库/构建信息/动力学）布局同样逐字一致。"""
    assert ctypes.sizeof(gen.rm_algorithm_version_t) == 20
    assert ctypes.sizeof(gen.rm_software_build_info_t) == 40
    assert ctypes.sizeof(gen.rm_dynamic_version_t) == 5
    assert gen.rm_algorithm_version_t.version.offset == 0
    assert gen.rm_software_build_info_t.build_time.offset == 0
    assert gen.rm_software_build_info_t.version.offset == 20
    assert gen.rm_dynamic_version_t.model_version.offset == 0


def test_generated_symbol_signature():
    """生成的符号表里该符号是 ``(handle*, software_info*) -> int``。"""
    sig = gen.SYMBOLS[SYMBOL]
    assert sig["restype"] is ctypes.c_int
    assert sig["argtypes"][0] is ctypes.POINTER(gen.rm_robot_handle)
    assert sig["argtypes"][1] is ctypes.POINTER(gen.rm_arm_software_version_t)


# ------------------------------------------------------- (e) 黑名单约束
def test_symbol_not_in_blacklist_or_algo_prefix():
    """新符号既不在黑名单，也不在 ``rm_algo_`` 前缀族里。"""
    contract = loader.load_contract()
    blacklist = loader.blacklist_names(contract)
    prefixes = loader.blacklist_prefixes(contract)

    assert len(blacklist) == 13, "黑名单符号数变了：%d" % len(blacklist)
    assert set(blacklist) == EXPECTED_BLACKLIST
    assert SYMBOL not in blacklist
    assert not any(SYMBOL.startswith(p) for p in prefixes)
    assert prefixes == ["rm_algo_"]


# ------------------------------------------------------- (d) 诚实降级
def test_not_connected_returns_structured_error_not_exception():
    """未连接 → 结构化 error（**不抛异常**、不伪造版本）。"""
    fake = FakeLib(handle_id=-1)
    link_obj = bridge.connect("192.168.1.18", 8080, lib=fake)
    assert link_obj.status != errors.STATUS_OK

    before = len(fake.calls)
    res = link_obj.arm_software_info()

    assert res["status"] == errors.STATUS_ERROR, res
    assert res["reason"]
    assert res["next_action"]
    assert res.get("sdk_calls") == 0
    for key in ("product_version", "robot_controller_version", "algorithm_version",
                "ctrl_version", "ctrl_build_time", "dynamic_model_version",
                "plan_version", "plan_build_time", "com_version", "program_version"):
        assert res[key] is None, "%s 不得伪造" % key
    assert len(fake.calls) == before, "未连接路径不得调用 SDK"


def test_success_maps_versions(link, fake_lib):
    """成功路径：版本字段逐字映射到结果字典。"""
    fake_lib.payload = {
        "product_version": b"RM75",
        "robot_controller_version": b"4.0",
        "algorithm_info": b"1.2.3",
        "ctrl_info": {"version": b"3.1.0", "build_time": b"2025/01/02 03:04:05"},
        "dynamic_info": b"d2.1",
        "plan_info": {"version": b"7.0.1", "build_time": b"2025/02/03 04:05:06"},
        "com_info": {"version": b"8.0.0"},
        "program_info": {"version": b"9.0.0"},
    }
    res = link.arm_software_info()

    assert res["status"] == errors.STATUS_OK, res
    assert res["product_version"] == "RM75"
    assert res["robot_controller_version"] == "4.0"
    assert res["algorithm_version"] == "1.2.3"
    assert res["ctrl_version"] == "3.1.0"
    assert res["ctrl_build_time"] == "2025/01/02 03:04:05"
    assert res["dynamic_model_version"] == "d2.1"
    assert res["plan_version"] == "7.0.1"
    assert res["plan_build_time"] == "2025/02/03 04:05:06"
    assert res["com_version"] == "8.0.0"
    assert res["program_version"] == "9.0.0"
    assert res.get("sdk_calls") == 1
    assert fake_lib.calls_of("rm_get_arm_software_info")


def test_empty_version_strings_become_none(link, fake_lib):
    """负例：SDK 返回**空串** → 版本字段为 ``None``（不返回 "" 冒充版本）。"""
    fake_lib.payload = {"product_version": b""}
    res = link.arm_software_info()

    assert res["status"] == errors.STATUS_OK, res
    assert res["product_version"] is None
    assert res["robot_controller_version"] is None
    assert res["algorithm_version"] is None
    assert res["ctrl_version"] is None
    assert res["ctrl_build_time"] is None
    assert res["dynamic_model_version"] is None
    assert res["plan_version"] is None
    assert res["plan_build_time"] is None
    assert res["com_version"] is None
    assert res["program_version"] is None


def test_sdk_failure_is_error_without_versions(link, fake_lib):
    """负例：SDK 返回非 0 → error，且**不带任何版本字段**。"""
    fake_lib.rc = -3
    res = link.arm_software_info()

    assert res["status"] == errors.STATUS_ERROR, res
    assert SYMBOL in res["reason"]
    assert "-3" in res["reason"]
    assert res["next_action"]
    assert res["product_version"] is None
    assert res.get("sdk_calls") == 1


def test_sdk_exception_is_error_without_versions(link, fake_lib):
    """负例：SDK 抛异常 → error（不把异常抛给调用方）。"""
    fake_lib.raises = True
    res = link.arm_software_info()

    assert res["status"] == errors.STATUS_ERROR, res
    assert SYMBOL in res["reason"]
    assert res["next_action"]
    assert res["product_version"] is None


def test_decode_version_text_helper():
    """解码助手：空串 → None；bytes/str 正常解码；None → None。"""
    assert bridge._decode_version_text(b"1.2.3") == "1.2.3"
    assert bridge._decode_version_text("1.2.3") == "1.2.3"
    assert bridge._decode_version_text(b"") is None
    assert bridge._decode_version_text(b"   ") is None
    assert bridge._decode_version_text(None) is None
