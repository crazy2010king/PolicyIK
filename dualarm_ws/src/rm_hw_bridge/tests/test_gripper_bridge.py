"""夹爪 SDK 适配层测试（``rm_hw_bridge/gripper.py``）。

覆盖：

* **调用形态**：``rm_set_modbus_mode(port=1, baudrate, timeout)``、读寄存器的
  ``params.port/address/device/num``、写单寄存器的 ``data``、``rm_close_modbus_mode``
  被调用（假 SDK，走 ``loader.SymbolBinding`` 契约绑定）；
* **生命周期**：正常路径开 → 用 → 关；**失败路径（读超时）也必须关闭协议**；
* **地址基数两套取值**与 **FC03 / FC04 双读**都被覆盖；
* **红线负例**：写 0x0008 被拒绝（blocked）且**没有任何 SDK 写调用**产生；
* **FakeGripper → GripperState 归一化**：模式映射、错误位映射、
  position_mm / force 换算在系数存在时正确、系数为 ``None`` 时字段为 ``None``；
* 假夹爪产生的状态一律带 ``mode=fake`` 标注（**不冒充真机结论**）。

测试用**假 SDK**（stub），不依赖 ``.deps/`` 里的真实 ``.so``。
"""
from __future__ import annotations

import pytest

from rm_hw_bridge import gripper as g
from rm_hw_bridge import loader


# --------------------------------------------------------------------- 假 SDK
class _FakeFn:
    """可写属性（argtypes/restype）的假符号；记录调用形态。"""

    def __init__(self, name, result=0, behavior=None):
        self.name = name
        self.argtypes = None
        self.restype = None
        self.result = result
        self.behavior = behavior
        self.calls = []

    def __call__(self, *args):
        self.calls.append(args)
        if self.behavior is not None:
            return self.behavior(*args)
        return self.result


class FakeLib:
    """假动态库：只暴露显式给出的符号（缺失符号 → getattr 失败）。"""

    def __init__(self, symbols, results=None, behaviors=None):
        self._fns = {}
        for name in symbols:
            self._fns[name] = _FakeFn(name)
        for name, value in (results or {}).items():
            if name in self._fns:
                self._fns[name].result = value
        for name, behavior in (behaviors or {}).items():
            if name in self._fns:
                self._fns[name].behavior = behavior

    def __getattr__(self, name):
        fns = self.__dict__.get("_fns", {})
        if name in fns:
            return fns[name]
        raise AttributeError(name)

    def fn(self, name):
        return self._fns[name]


class _FakeHandle:
    def __init__(self, handle_id=1):
        self.id = handle_id


#: 适配层用到的全部夹爪符号
GRIPPER_SYMBOLS = (
    "rm_set_modbus_mode",
    "rm_close_modbus_mode",
    "rm_read_holding_registers",
    "rm_read_input_registers",
    "rm_write_single_register",
    "rm_write_registers",
    "rm_set_gripper_route",
    "rm_set_gripper_release",
    "rm_set_gripper_pick",
    "rm_set_gripper_pick_on",
    "rm_set_gripper_position",
    "rm_get_gripper_state",
    "rm_set_rm_plus_mode",
    "rm_get_rm_plus_mode",
    "rm_get_tool_voltage",
    "rm_set_tool_voltage",
)


def _make_read_behavior(value=123):
    """读寄存器：往缓冲区写入固定值。"""

    def behavior(_handle, _params, buf):
        buf[0] = value
        return 0

    return behavior


@pytest.fixture()
def fake_binding():
    """夹爪符号的契约绑定（走 ``loader.SymbolBinding``，只暴露清单内符号）。"""
    lib = FakeLib(
        GRIPPER_SYMBOLS,
        behaviors={
            "rm_read_holding_registers": _make_read_behavior(321),
            "rm_read_input_registers": _make_read_behavior(654),
        },
    )
    binding = loader.SymbolBinding(lib)
    return binding, lib


@pytest.fixture()
def adapter(fake_binding):
    binding, _lib = fake_binding
    return g.GripperAdapter(g.CtypesSdkPort(binding, _FakeHandle()))


# ----------------------------------------------------------------- 地址基数
def test_register_address_base1_is_manual_value():
    assert g.register_address(g.REG_ACTUAL_FORCE, 1) == 0x000E
    assert g.register_address(g.REG_STATE, 1) == 0x0014


def test_register_address_base0_is_pdU_offset():
    assert g.register_address(g.REG_ACTUAL_FORCE, 0) == 0x000D
    assert g.register_address(g.REG_STATE, 0) == 0x0013


def test_register_address_default_base_is_1():
    assert g.register_address(g.REG_ERROR) == g.register_address(g.REG_ERROR, 1)


def test_register_address_rejects_bad_base():
    with pytest.raises(ValueError):
        g.register_address(g.REG_ERROR, 2)


def test_register_address_rejects_non_int():
    with pytest.raises(TypeError):
        g.register_address("0x0013", 1)


# ----------------------------------------------------------------- 调用形态
def test_open_modbus_call_shape(adapter, fake_binding):
    binding, lib = fake_binding
    res = adapter.open_protocol()
    assert res["status"] == "ok"
    calls = lib.fn("rm_set_modbus_mode").calls
    assert len(calls) == 1
    handle, port, baud, timeout = calls[0]
    assert port == 1
    assert baud == 115200
    assert timeout == 5


def test_read_register_params_shape(adapter, fake_binding):
    _binding, lib = fake_binding
    res = adapter.read_register(g.REG_ACTUAL_POS)
    assert res["status"] == "ok"
    calls = lib.fn("rm_read_holding_registers").calls
    assert len(calls) == 1
    _handle, params, _buf = calls[0]
    assert params.port == 1
    assert params.address == g.REG_ACTUAL_POS
    assert params.device == 1
    assert params.num == 1


def test_read_register_value_roundtrip(adapter, fake_binding):
    _binding, lib = fake_binding
    res = adapter.read_register(g.REG_ACTUAL_POS)
    assert res["value"] == 321
    assert lib.fn("rm_read_input_registers").calls == []


def test_read_input_function_uses_fc04(adapter, fake_binding):
    _binding, lib = fake_binding
    res = adapter.read_register(g.REG_ACTUAL_POS, function="input")
    assert res["status"] == "ok"
    assert lib.fn("rm_read_input_registers").calls
    assert lib.fn("rm_read_holding_registers").calls == []
    assert res["value"] == 654


def test_write_single_register_data_shape(adapter, fake_binding):
    _binding, lib = fake_binding
    res = adapter.write_register(g.REG_TARGET_POS, 500)
    assert res["status"] == "ok"
    calls = lib.fn("rm_write_single_register").calls
    assert len(calls) == 1
    _handle, params, data = calls[0]
    assert params.address == g.REG_TARGET_POS
    assert params.num == 1
    assert data == 500


def test_write_registers_num_and_values(adapter, fake_binding):
    _binding, lib = fake_binding
    res = adapter.write_registers(g.REG_TARGET_POS, [10, 20, 30])
    assert res["status"] == "ok"
    _handle, params, data = lib.fn("rm_write_registers").calls[0]
    assert params.num == 3
    assert list(data) == [10, 20, 30]


def test_close_modbus_is_called(adapter, fake_binding):
    _binding, lib = fake_binding
    adapter.open_protocol()
    res = adapter.close_protocol()
    assert res["status"] == "ok"
    assert lib.fn("rm_close_modbus_mode").calls


# ----------------------------------------------------------------- 生命周期
def test_lifecycle_open_use_close(adapter, fake_binding):
    _binding, lib = fake_binding
    adapter.open_protocol()
    assert adapter.protocol_open is True
    adapter.read_register(g.REG_STATE)
    adapter.close_protocol()
    assert adapter.protocol_open is False
    assert lib.fn("rm_set_modbus_mode").calls
    assert lib.fn("rm_close_modbus_mode").calls


def test_close_is_idempotent(adapter, fake_binding):
    _binding, lib = fake_binding
    adapter.open_protocol()
    adapter.close_protocol()
    adapter.close_protocol()
    assert len(lib.fn("rm_close_modbus_mode").calls) == 1


def test_failure_path_also_closes_protocol(adapter, fake_binding):
    """读超时（rc=-2）也必须恢复协议状态（关闭 Modbus）。"""
    _binding, lib = fake_binding
    lib.fn("rm_read_holding_registers").behavior = lambda *a: g.RC_RECV_TIMEOUT

    res = adapter.with_protocol(lambda ad: ad.read_register(g.REG_STATE))
    assert res["status"] == "error"
    assert "接收失败或超时" in res["reason"]
    assert lib.fn("rm_close_modbus_mode").calls, "失败路径必须调用 rm_close_modbus_mode"
    assert adapter.protocol_open is False


def test_with_protocol_closes_on_success(adapter, fake_binding):
    _binding, lib = fake_binding
    res = adapter.with_protocol(lambda ad: ad.read_register(g.REG_STATE))
    assert res["status"] == "ok"
    assert lib.fn("rm_close_modbus_mode").calls
    assert adapter.protocol_open is False


def test_with_protocol_keeps_preexisting_open_state(adapter, fake_binding):
    """进入前协议已开 → 成功后**保持打开**（恢复到进入前状态）。"""
    _binding, lib = fake_binding
    adapter.open_protocol()
    res = adapter.with_protocol(lambda ad: ad.read_register(g.REG_STATE))
    assert res["status"] == "ok"
    assert adapter.protocol_open is True
    assert lib.fn("rm_close_modbus_mode").calls == []


def test_exception_in_protocol_scope_becomes_error(adapter, fake_binding):
    _binding, lib = fake_binding

    def boom(_ad):
        raise RuntimeError("boom")

    res = adapter.with_protocol(boom)
    assert res["status"] == "error"
    assert "boom" in res["reason"]
    assert lib.fn("rm_close_modbus_mode").calls


def test_open_modbus_failure_is_error(adapter, fake_binding):
    _binding, lib = fake_binding
    lib.fn("rm_set_modbus_mode").result = g.RC_SEND_FAILED
    res = adapter.open_protocol()
    assert res["status"] == "error"
    assert "发送失败" in res["reason"]
    assert adapter.protocol_open is False


def test_open_modbus_minus4_is_blocked(adapter, fake_binding):
    """-4 = 四代控制器不支持该接口 → blocked（能力缺失）。"""
    _binding, lib = fake_binding
    lib.fn("rm_set_modbus_mode").result = g.RC_UNSUPPORTED
    res = adapter.open_protocol()
    assert res["status"] == "blocked"
    assert res["rc"] == -4


# ----------------------------------------------------------------- 红线负例
def test_write_clear_error_is_blocked(adapter, fake_binding):
    _binding, lib = fake_binding
    res = adapter.write_register(g.REG_CLEAR_ERROR, 1)
    assert res["status"] == "blocked"
    assert "0x0008" in res["reason"]
    assert res["sdk_calls"] == 0


def test_write_clear_error_produces_no_sdk_write_call(adapter, fake_binding):
    _binding, lib = fake_binding
    adapter.write_register(g.REG_CLEAR_ERROR, 1)
    assert lib.fn("rm_write_single_register").calls == []
    assert lib.fn("rm_write_registers").calls == []


def test_write_registers_blocking_clear_error_produces_no_write(adapter, fake_binding):
    _binding, lib = fake_binding
    res = adapter.write_registers(g.REG_ESTOP, [1, 1])  # 0x0007 与 0x0008
    assert res["status"] == "blocked"
    assert lib.fn("rm_write_registers").calls == []


def test_write_read_only_register_is_blocked(adapter, fake_binding):
    _binding, lib = fake_binding
    res = adapter.write_register(g.REG_ERROR, 0)
    assert res["status"] == "blocked"
    assert lib.fn("rm_write_single_register").calls == []


def test_write_wire_address_0x0008_is_blocked_under_base0(adapter, fake_binding):
    """基数猜错时，线上 0x0008 也必须拦住（宁可误伤，不冒误清错误码的风险）。"""
    _binding, lib = fake_binding
    # base=0 下 manual 0x0009（目标开口度）→ 线上 0x0008
    res = adapter.write_register(g.REG_TARGET_POS, 500, base=0)
    assert res["status"] == "blocked"
    assert "0x0008" in res["reason"]
    assert lib.fn("rm_write_single_register").calls == []


def test_write_wire_address_0x0008_is_blocked_under_base1(adapter, fake_binding):
    _binding, lib = fake_binding
    res = adapter.write_register(g.REG_TARGET_POS, 500, base=1)
    assert res["status"] == "ok"
    res = adapter.write_register(g.REG_CLEAR_ERROR, 1, base=1)
    assert res["status"] == "blocked"
    assert lib.fn("rm_write_single_register").calls


def test_red_line_never_reaches_sdk_even_after_repeated_calls(adapter, fake_binding):
    _binding, lib = fake_binding
    for _ in range(3):
        adapter.write_register(g.REG_CLEAR_ERROR, 1)
        adapter.write_registers(g.REG_ESTOP, [1, 1])
    assert lib.fn("rm_write_single_register").calls == []
    assert lib.fn("rm_write_registers").calls == []


def test_write_more_than_ten_registers_is_rejected(adapter, fake_binding):
    _binding, lib = fake_binding
    res = adapter.write_registers(g.REG_TARGET_POS, list(range(11)))
    assert res["status"] == "error"
    assert lib.fn("rm_write_registers").calls == []


# ------------------------------------------------------------- 基数与双功能码
def test_both_bases_are_covered(adapter, fake_binding):
    _binding, lib = fake_binding
    res1 = adapter.read_register(g.REG_ACTUAL_POS, base=1)
    res0 = adapter.read_register(g.REG_ACTUAL_POS, base=0)
    assert res1["status"] == "ok" and res0["status"] == "ok"
    assert res1["address"] == g.REG_ACTUAL_POS
    assert res0["address"] == g.REG_ACTUAL_POS - 1
    addresses = [call[1].address for call in lib.fn("rm_read_holding_registers").calls]
    assert addresses == [g.REG_ACTUAL_POS, g.REG_ACTUAL_POS - 1]


def test_both_function_codes_are_covered(adapter, fake_binding):
    _binding, lib = fake_binding
    adapter.read_register(g.REG_STATE, function="holding")
    adapter.read_register(g.REG_STATE, function="input")
    assert lib.fn("rm_read_holding_registers").calls
    assert lib.fn("rm_read_input_registers").calls


def test_unknown_function_is_error(adapter):
    res = adapter.read_register(g.REG_STATE, function="fc05")
    assert res["status"] == "error"


# ------------------------------------------------------- FakeGripper 归一化
def _fake_with_registers(**kw):
    fake = g.FakeGripper(**kw)
    fake.set_register(g.REG_STATE, 2)
    fake.set_register(g.REG_ACTUAL_POS, 500)
    fake.set_register(g.REG_ACTUAL_FORCE, 400)
    fake.set_register(g.REG_ACTUAL_TEMP, 35)
    fake.set_register(g.REG_ACTUAL_CURRENT, 120)
    fake.set_register(g.REG_ERROR, 0)
    return fake


def test_fake_gripper_normalizes_modbus_state():
    fake = _fake_with_registers()
    ad = g.GripperAdapter(fake, path=g.PATH_MODBUS)
    res = ad.read_state()
    assert res["status"] == "ok"
    state = res["state"]
    assert state.mode == "closed_min"      # 手册状态码 2
    assert state.mode_raw == 2
    assert state.position_mm is None        # 系数缺失 → None
    assert state.force_norm == 400          # Modbus 0x000E 本身即 0–1000 标度
    assert state.force_N is None            # 力标度缺失 → None
    assert state.temp_c == 35
    assert state.current_mA == 120
    assert state.error_bits == 0
    assert state.protocol_state == "online"


def test_fake_gripper_position_and_force_converted_with_coefficients():
    fake = _fake_with_registers()
    ad = g.GripperAdapter(
        fake, path=g.PATH_MODBUS,
        position_mm_per_unit=0.065, force_scale=0.039,
    )
    state = ad.read_state()["state"]
    assert state.position_mm == pytest.approx(500 * 0.065)
    assert state.force_N == pytest.approx(400 * 0.039)


def test_fake_gripper_error_bits_decoded():
    fake = _fake_with_registers()
    fake.set_register(g.REG_ERROR, 0b0000_0011)  # bit0 堵转 + bit1 过温
    ad = g.GripperAdapter(fake, path=g.PATH_MODBUS)
    state = ad.read_state()["state"]
    flags = state.error_flags()
    assert flags["stall"] is True
    assert flags["overtemp"] is True
    assert flags["overcurrent"] is False


def test_fake_gripper_mode_mapping_table():
    fake = g.FakeGripper()
    ad = g.GripperAdapter(fake, path=g.PATH_MODBUS)
    for code, mode in ((1, "open_max"), (2, "closed_min"), (3, "stopped"),
                       (4, "closing"), (5, "opening")):
        fake.set_register(g.REG_STATE, code)
        state = ad.read_state()["state"]
        assert state.mode == mode


def test_fake_gripper_gripper_api_mode6_is_force_stop():
    """SDK 结构体 mode=6（力控停止）在手册状态码里不存在。"""
    fake = g.FakeGripper(mode=6, actpos=100, current_force=50, temperature=30)
    ad = g.GripperAdapter(fake, path=g.PATH_GRIPPER_API)
    state = ad.read_state()["state"]
    assert state.mode == "force_stop"
    assert state.mode_raw == 6


def test_fake_gripper_state_is_tagged_fake():
    fake = _fake_with_registers()
    ad = g.GripperAdapter(fake, path=g.PATH_MODBUS)
    res = ad.read_state()
    assert res["mode_source"] == "fake"
    assert res["state"].raw["source"] == "fake"
    assert any("mode=fake" in str(e) for e in res["evidence"])


def test_fake_gripper_offline_maps_to_offline_protocol_state():
    fake = g.FakeGripper(online=False, mode=1)
    ad = g.GripperAdapter(fake, path=g.PATH_GRIPPER_API)
    state = ad.read_state()["state"]
    assert state.protocol_state == "offline"
    assert state.online is False


def test_rm_plus_path_has_no_state_read():
    fake = g.FakeGripper()
    ad = g.GripperAdapter(fake, path=g.PATH_RM_PLUS)
    res = ad.read_state()
    assert res["status"] == "blocked"


def test_rm_plus_mode_disable_vs_enabled():
    fake = g.FakeGripper(rm_plus_mode=0)
    ad = g.GripperAdapter(fake)
    res = ad.rm_plus_mode()
    assert res["protocol_state"] == "disable"
    fake.set_rm_plus_mode(115200)
    res = ad.rm_plus_mode()
    assert res["protocol_state"] == "online"


def test_rm_plus_offline_is_distinguished_from_disable():
    """协议已开但读超时 → offline（而非 disable / unknown）。"""
    fake = g.FakeGripper(rm_plus_mode=115200)
    ad = g.GripperAdapter(fake)
    assert ad.rm_plus_mode()["protocol_state"] == "online"
    fake.set_timeout("get_rm_plus_mode")
    res = ad.rm_plus_mode()
    assert res["status"] == "error"
    assert res["protocol_state"] == "offline"


def test_rm_plus_unknown_when_mode_never_known():
    fake = g.FakeGripper(rm_plus_mode=0)
    ad = g.GripperAdapter(fake)
    fake.set_timeout("get_rm_plus_mode")
    res = ad.rm_plus_mode()
    assert res["protocol_state"] == "unknown"


# ------------------------------------------------------------------- 其它
def test_tool_voltage_is_read_only_by_default():
    fake = g.FakeGripper(tool_voltage=0)
    ad = g.GripperAdapter(fake)
    res = ad.read_tool_voltage()
    assert res["status"] == "ok"
    assert res["written"] is False
    assert fake.calls_for("set_tool_voltage") == []


def test_probe_reports_assumptions_and_closes_protocol():
    fake = _fake_with_registers()
    ad = g.GripperAdapter(fake, path=g.PATH_MODBUS)
    out = ad.probe()
    assert out["status"] == "ok"
    assert out["assumptions"], "探针必须显式标注未实测假设"
    assert out["detected_base"] == 1
    assert fake.calls_for("close_modbus_mode")


def test_probe_marks_fake_mode():
    fake = _fake_with_registers()
    ad = g.GripperAdapter(fake, path=g.PATH_MODBUS)
    out = ad.probe()
    assert any("mode=fake" in str(e) for e in out["evidence"])


def test_gripper_fast_path_calls(adapter, fake_binding):
    _binding, lib = fake_binding
    assert adapter.gripper_route(0, 1000)["status"] == "ok"
    assert adapter.gripper_release(100, True, 2)["status"] == "ok"
    assert adapter.gripper_pick(100, 200, True, 2)["status"] == "ok"
    assert adapter.gripper_pick_on(100, 200, True, 2)["status"] == "ok"
    assert adapter.gripper_position(500, True, 2)["status"] == "ok"
    for name in ("rm_set_gripper_route", "rm_set_gripper_release", "rm_set_gripper_pick",
                 "rm_set_gripper_pick_on", "rm_set_gripper_position"):
        assert lib.fn(name).calls, name


def test_describe_rc_covers_vendor_codes():
    assert "发送失败" in g.describe_rc(-1)
    assert "超时" in g.describe_rc(-2)
    assert "解析失败" in g.describe_rc(-3)
    assert "四代控制器" in g.describe_rc(-4)
    assert g.describe_rc(0) == "成功"


# --------------------------------------------------------------- 负载维度
def test_fake_gripper_call_records_are_bounded_but_counts_survive():
    """长跑（离线证据脚本）时调用记录有界，计数不受淘汰影响。"""
    fake = g.FakeGripper(max_records=10)
    ad = g.GripperAdapter(fake, path=g.PATH_MODBUS)
    for _ in range(50):
        ad.read_register(g.REG_STATE)
    assert len(fake.calls) == 10
    assert fake.count("read_holding_register") == 50
    assert fake.call_count == 50
