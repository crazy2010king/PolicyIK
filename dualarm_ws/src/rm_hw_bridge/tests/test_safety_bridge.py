"""硬件层**安全窄接口**的离线测试（假 SDK）。

覆盖（T2 验收点）：

* ``ArmLink.set_drive_max_speed()`` —— 参数非法 → **0 次 SDK 调用**；
  ``rc != 0`` → ``status="error"`` 且 reason 写明**接口名 + 返回码**；
* ``ArmLink.set_timeout()`` —— 参数非法 → **0 次 SDK 调用**；成功时注明
  「SDK 侧全局生效」（**不伪读回值**）；
* ``ArmLink.controller_error_state()`` —— ``arm_err`` / ``arm_current_status`` /
  ``arm_current_status_name`` 的映射，且**无数据时为 ``None``（不返回 0 冒充）**；
* **建连期接线** —— ``connect()`` 后 ``rm_set_timeout`` 被调用**一次**且传入
  ``1.0 s``（对外秒 → SDK 毫秒，换算只在 ``rm_hw_bridge.bridge`` 一处）；
  超时设置**失败不阻断连接**（但必须留下证据）；
* **22 + 21 名单与黑名单**（直接读 ``contract.yaml`` 断言；core 22 / 黑名单 13 不变）。

只用**注入的假 SDK**，不依赖 ``.deps/``（gitignored）里的真实 ``.so``。
"""
from __future__ import annotations

import ctypes
import math

import pytest

from rm_hw_bridge import bridge, errors, loader
from rm_hw_bridge.generated import rm_api2_ctypes as gen


# --------------------------------------------------------------------- 假 SDK
class FakeHandle:
    """假句柄（RM-API2 失败时仍返回非空指针但 ``id == -1``）。"""

    def __init__(self, handle_id: int = 1) -> None:
        self.id = handle_id


class FakeLib:
    """假动态库：只实现本任务用到的白名单符号（+ 建连期必需的少数几个）。"""

    def __init__(self, handle_id: int = 1, results=None, timeout_raises: bool = False,
                 speed_raises: bool = False, arm_err_codes=(1001, 1002)):
        self.calls = []
        self.handle = FakeHandle(handle_id)
        self.results = dict(results or {})
        self.timeout_raises = timeout_raises
        self.speed_raises = speed_raises
        self.arm_err_codes = list(arm_err_codes)
        self.timeout_args = []
        self._install()

    # -- 安装假符号
    def _install(self):
        lib = self

        def _rc(name, default=0):
            return lib.results.get(name, default)

        def rm_init(mode):
            lib.calls.append(("rm_init", mode))
            return _rc("rm_init")

        def rm_create_robot_arm(ip, port):
            lib.calls.append(("rm_create_robot_arm", ip, port))
            return _rc("rm_create_robot_arm", lib.handle)

        def rm_delete_robot_arm(handle):
            lib.calls.append(("rm_delete_robot_arm",))
            return _rc("rm_delete_robot_arm")

        def rm_set_realtime_push(handle, config):
            lib.calls.append(("rm_set_realtime_push",))
            return _rc("rm_set_realtime_push")

        def rm_realtime_arm_state_call_back(cb):
            lib.calls.append(("rm_realtime_arm_state_call_back",))
            return None

        def rm_set_log_call_back(cb, level):
            lib.calls.append(("rm_set_log_call_back",))
            return None

        def rm_set_timeout(timeout):
            lib.calls.append(("rm_set_timeout", timeout))
            lib.timeout_args.append(timeout)
            if lib.timeout_raises:
                raise RuntimeError("fake rm_set_timeout 失败")
            return None  # SDK 侧返回 void

        def rm_set_joint_drive_max_speed(handle, joint_num, max_speed):
            lib.calls.append(
                ("rm_set_joint_drive_max_speed", joint_num, float(max_speed))
            )
            if lib.speed_raises:
                raise RuntimeError("fake rm_set_joint_drive_max_speed 失败")
            return _rc("rm_set_joint_drive_max_speed")

        def rm_get_current_arm_state(handle, state_ptr):
            lib.calls.append(("rm_get_current_arm_state",))
            rc = _rc("rm_get_current_arm_state")
            if rc == 0:
                st = state_ptr.contents
                st.err.err_len = len(lib.arm_err_codes)
                for i, code in enumerate(lib.arm_err_codes):
                    st.err.err[i] = int(code)
            return rc

        for name, fn in list(locals().items()):
            if name.startswith("rm_") and callable(fn):
                setattr(self, name, fn)

    # -- 工具
    def calls_of(self, name):
        return [c for c in self.calls if c[0] == name]


@pytest.fixture()
def fake_lib():
    return FakeLib()


@pytest.fixture()
def link(fake_lib):
    """已连接的 ArmLink（假 SDK）。"""
    link_obj = bridge.connect("192.168.1.18", 8080, lib=fake_lib)
    assert link_obj.status == errors.STATUS_OK, link_obj.reason
    return link_obj


# ------------------------------------------------------- 契约名单（22 + 21）
EXPECTED_CORE = {
    "rm_init",
    "rm_create_robot_arm",
    "rm_delete_robot_arm",
    "rm_set_realtime_push",
    "rm_get_joint_degree",
    "rm_get_current_arm_state",
    "rm_get_arm_all_state",
    "rm_get_joint_err_flag",
    "rm_get_arm_current_trajectory",
    "rm_movej",
    "rm_movej_canfd",
    "rm_set_delete_current_trajectory",
    "rm_set_arm_stop",
    "rm_set_arm_slow_stop",
    "rm_set_arm_pause",
    "rm_set_arm_continue",
    "rm_set_joint_drive_max_speed",
    "rm_set_joint_drive_max_acc",
    "rm_set_joint_drive_min_pos",
    "rm_set_joint_drive_max_pos",
    "rm_set_collision_state",
    "rm_set_self_collision_enable",
}

EXPECTED_SUPPORT = {
    "rm_set_log_call_back",
    "rm_set_timeout",
    "rm_get_robot_info",
    "rm_get_realtime_push",
    "rm_realtime_arm_state_call_back",
    # --- 夹爪快路径（S05 扩容）
    "rm_set_gripper_route",
    "rm_set_gripper_release",
    "rm_set_gripper_pick",
    "rm_set_gripper_pick_on",
    "rm_set_gripper_position",
    "rm_get_gripper_state",
    # --- Modbus-RTU 主路径
    "rm_set_modbus_mode",
    "rm_close_modbus_mode",
    "rm_read_holding_registers",
    "rm_read_input_registers",
    "rm_write_registers",
    "rm_write_single_register",
    # --- RM Plus 末端生态协议
    "rm_set_rm_plus_mode",
    "rm_get_rm_plus_mode",
    # --- 工具端电压
    "rm_set_tool_voltage",
    "rm_get_tool_voltage",
}

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


def test_contract_core_support_blacklist_unchanged():
    """**22 核心 + 21 支持 + 13 黑名单符号 + rm_algo_ 前缀**（核心/黑名单不变）。"""
    contract = loader.load_contract()
    core = loader.core_symbols(contract)
    support = loader.support_symbols(contract)
    blacklist = loader.blacklist_names(contract)
    prefixes = loader.blacklist_prefixes(contract)

    assert len(core) == 22, "核心符号数变了：%d" % len(core)
    assert set(core) == EXPECTED_CORE, "核心符号名单变了"
    assert len(support) == len(EXPECTED_SUPPORT), "支持符号数变了：%d" % len(support)
    assert set(support) == EXPECTED_SUPPORT, "支持符号名单变了"
    assert len(blacklist) == 13, "黑名单符号数变了：%d" % len(blacklist)
    assert set(blacklist) == EXPECTED_BLACKLIST, "黑名单符号名单变了"
    assert prefixes == ["rm_algo_"], "黑名单前缀变了：%r" % (prefixes,)
    assert int((contract.get("limits") or {}).get("core_count")) == 22
    # 支持层数量**按契约读取**（不再写死数字），且与名单严格一致
    assert int((contract.get("limits") or {}).get("support_count")) == len(support)

    # 三个窄接口用到的符号**必须**在既有名单内（不新增 SDK 符号）
    assert "rm_set_joint_drive_max_speed" in core
    assert "rm_get_current_arm_state" in core
    assert "rm_set_timeout" in support


# ------------------------------------------- set_drive_max_speed：非法参数
@pytest.mark.parametrize(
    "joint_index,deg_s",
    [
        (-1, 10.0),          # 索引越界（负）
        (7, 10.0),           # 索引越界（>= 7）
        (1.5, 10.0),         # 索引非整数
        ("1", 10.0),         # 索引非整数类型
        (True, 10.0),        # bool 不算整数
        (0, 0.0),            # 速度非正
        (0, -5.0),           # 速度非正
        (0, float("nan")),   # 非有限
        (0, float("inf")),   # 非有限
        (0, "10"),           # 速度非数值
    ],
)
def test_set_drive_max_speed_invalid_params_zero_sdk_calls(link, fake_lib, joint_index, deg_s):
    """参数非法 → **0 次 SDK 调用** + error + 可粘贴 next_action。"""
    before = len(fake_lib.calls)
    res = link.set_drive_max_speed(joint_index, deg_s)

    assert res["status"] == errors.STATUS_ERROR, res
    assert res["reason"], "reason 不能为空"
    assert res["next_action"], "next_action 不能为空"
    assert res.get("sdk_calls") == 0
    assert len(fake_lib.calls) == before, "非法参数路径不得调用 SDK"
    assert not fake_lib.calls_of("rm_set_joint_drive_max_speed")


def test_set_drive_max_speed_success_reports_applied(link, fake_lib):
    """成功 → applied 含 joint_index / deg_s / rc（单位 °/s，不换算）。"""
    res = link.set_drive_max_speed(2, 15.5)
    assert res["status"] == errors.STATUS_OK, res
    applied = res["applied"][0]
    assert applied["joint_index"] == 2
    assert applied["deg_s"] == pytest.approx(15.5)
    assert applied["rc"] == 0
    assert applied["unit"] == "deg/s"
    # SDK 的 joint_num 是 1 基（= joint_index + 1）
    assert fake_lib.calls_of("rm_set_joint_drive_max_speed") == [
        ("rm_set_joint_drive_max_speed", 3, pytest.approx(15.5))
    ]


def test_set_drive_max_speed_rc_nonzero_reports_symbol_and_code(link, fake_lib):
    """rc != 0 → error，reason 写明**接口名 + 返回码**。"""
    fake_lib.results["rm_set_joint_drive_max_speed"] = -7
    res = link.set_drive_max_speed(0, 20.0)

    assert res["status"] == errors.STATUS_ERROR, res
    assert "rm_set_joint_drive_max_speed" in res["reason"]
    assert "-7" in res["reason"]
    assert res["next_action"]
    assert res.get("sdk_calls") == 1


def test_set_drive_max_speed_sdk_exception_is_error(fake_lib):
    """SDK 调用抛异常 → error（不抛到调用方）。"""
    fake_lib.speed_raises = True
    link_obj = bridge.connect("192.168.1.18", 8080, lib=fake_lib)
    res = link_obj.set_drive_max_speed(0, 20.0)
    assert res["status"] == errors.STATUS_ERROR, res
    assert "rm_set_joint_drive_max_speed" in res["reason"]
    assert res["next_action"]


# ------------------------------------------------------- set_timeout：超时
@pytest.mark.parametrize("seconds", [0, -1.0, float("nan"), float("inf"), "1.0", None])
def test_set_timeout_invalid_params_zero_sdk_calls(link, fake_lib, seconds):
    """seconds 非法 → **0 次 SDK 调用** + error。"""
    before = len(fake_lib.calls)
    res = link.set_timeout(seconds)

    assert res["status"] == errors.STATUS_ERROR, res
    assert res["reason"]
    assert res["next_action"]
    assert res.get("sdk_calls") == 0
    assert len(fake_lib.calls) == before, "非法参数路径不得调用 SDK"
    # 建连期已调用过一次；非法参数路径**不得**再调用
    assert len(fake_lib.calls_of("rm_set_timeout")) == 1


def test_set_timeout_success_is_global_and_not_read_back(link, fake_lib):
    """成功 → 说明「SDK 侧全局生效」，**不伪读回值**。"""
    before = len(fake_lib.calls_of("rm_set_timeout"))
    res = link.set_timeout(1.0)

    assert res["status"] == errors.STATUS_OK, res
    assert len(fake_lib.calls_of("rm_set_timeout")) == before + 1
    assert res["timeout_s"] == pytest.approx(1.0)
    assert res["sdk_timeout_ms"] == 1000
    assert res["scope"] == "global"
    assert "全局" in res["reason"]


def test_set_timeout_converts_seconds_to_milliseconds(link, fake_lib):
    """**秒 → 毫秒换算只在一处**（厂商头文件：``rm_set_timeout`` 单位 ms）。"""
    res = link.set_timeout(0.5)
    assert res["status"] == errors.STATUS_OK, res
    assert res["sdk_timeout_ms"] == 500
    assert fake_lib.calls_of("rm_set_timeout")[-1][1] == 500


def test_set_timeout_sub_millisecond_rounds_to_minimum(link, fake_lib):
    """小于 1 ms 的请求向上取整到下限（0 = 不超时，绝不发 0）。"""
    res = link.set_timeout(0.0001)
    assert res["status"] == errors.STATUS_OK, res
    assert res["sdk_timeout_ms"] >= 1
    assert fake_lib.calls_of("rm_set_timeout")[-1][1] >= 1


def test_set_timeout_sdk_exception_is_error(link, fake_lib):
    """``rm_set_timeout`` 抛异常 → error（不吞掉）。"""
    fake_lib.timeout_raises = True
    res = link.set_timeout(1.0)
    assert res["status"] == errors.STATUS_ERROR, res
    assert "rm_set_timeout" in res["reason"]
    assert res["next_action"]


# ------------------------------------------- controller_error_state：状态映射
def test_controller_error_state_maps_arm_err(link):
    """arm_err 来自 ``rm_get_current_arm_state.err``。"""
    res = link.controller_error_state()
    assert res["status"] == errors.STATUS_OK, res
    assert res["arm_err"] == [1001, 1002]
    assert res.get("sdk_calls") == 1


def test_controller_error_state_without_realtime_data_is_none(link, monkeypatch):
    """**无实时上报数据时为 None**（不返回 0 冒充）。"""
    monkeypatch.setattr(bridge._REALTIME, "arm_current_status", None)
    res = link.controller_error_state()
    assert res["status"] == errors.STATUS_OK, res
    assert res["arm_current_status"] is None
    assert res["arm_current_status_name"] is None


def test_controller_error_state_maps_realtime_status(link, monkeypatch):
    """实时上报状态 → 枚举名映射（**不额外打 SDK**）。"""
    monkeypatch.setattr(bridge._REALTIME, "arm_current_status", 9)
    res = link.controller_error_state()
    assert res["status"] == errors.STATUS_OK, res
    assert res["arm_current_status"] == 9
    assert res["arm_current_status_name"] == "RM_STOP_E"
    assert res["arm_err"] == [1001, 1002]


def test_controller_error_state_sdk_failure_is_error_without_numbers(link, fake_lib):
    """SDK 失败 → error，三个数据字段全为 ``None``（不伪造）。"""
    fake_lib.results["rm_get_current_arm_state"] = -3
    res = link.controller_error_state()

    assert res["status"] == errors.STATUS_ERROR, res
    assert "rm_get_current_arm_state" in res["reason"]
    assert "-3" in res["reason"]
    assert res["next_action"]
    assert res["arm_err"] is None
    assert res["arm_current_status"] is None
    assert res["arm_current_status_name"] is None


def test_controller_error_state_arm_err_empty_list_when_no_error(fake_lib):
    """err_len = 0 → ``arm_err == []``（有数据、无错误码，不是 None）。"""
    fake_lib.arm_err_codes = []
    link_obj = bridge.connect("192.168.1.18", 8080, lib=fake_lib)
    res = link_obj.controller_error_state()
    assert res["status"] == errors.STATUS_OK, res
    assert res["arm_err"] == []


# ---------------------------------------------------------- 建连期接线
def test_connect_sets_timeout_once_with_one_second(fake_lib):
    """connect 后 ``rm_set_timeout`` 调用**一次**，秒数来自工位配置（1.0 s）。

    **单位**：对外是秒（``workcell`` 的 ``safety.sdk_timeout_s``），SDK 侧是
    **毫秒**（厂商头文件 ``rm_interface.h``：``rm_set_timeout`` 默认 500 ms），
    换算只发生在 ``rm_hw_bridge.bridge`` 一处 → 1.0 s 下发 1000 ms。
    """
    link_obj = bridge.connect("192.168.1.18", 8080, lib=fake_lib)
    assert link_obj.status == errors.STATUS_OK, link_obj.reason

    calls = fake_lib.calls_of("rm_set_timeout")
    assert len(calls) == 1, "rm_set_timeout 应被调用一次，实际 %d 次" % len(calls)
    assert calls[0][1] == 1000, "1.0 s 应换算成 1000 ms 下发"
    # 结果里留痕（请求秒数 1.0 s，SDK 侧实际下发毫秒数）
    assert link_obj.sdk_timeout_result["status"] == errors.STATUS_OK
    assert link_obj.sdk_timeout_result["timeout_s"] == pytest.approx(1.0)
    assert link_obj.sdk_timeout_result["sdk_timeout_ms"] == 1000


def test_connect_timeout_failure_does_not_block_connection():
    """超时设置失败**不阻断连接**，但必须留下证据（不静默）。"""
    fake = FakeLib(timeout_raises=True)
    link_obj = bridge.connect("192.168.1.18", 8080, lib=fake)

    assert link_obj.status == errors.STATUS_OK, link_obj.reason
    assert link_obj.sdk_timeout_result["status"] == errors.STATUS_ERROR
    assert "rm_set_timeout" in link_obj.sdk_timeout_result["reason"]
    assert any("不阻断连接" in e for e in link_obj.evidence), link_obj.evidence


def test_connect_timeout_missing_workcell_uses_default(fake_lib, tmp_path, monkeypatch):
    """缺工位文件 → 用默认 1.0 s，并在 evidence 注明（不静默）。"""
    fake = FakeLib()
    monkeypatch.setenv("RM_WS", str(tmp_path))
    link_obj = bridge.connect("192.168.1.18", 8080, lib=fake, ws=str(tmp_path))

    assert link_obj.status == errors.STATUS_OK, link_obj.reason
    assert fake.calls_of("rm_set_timeout")[0][1] == 1000
    assert any("默认" in e for e in link_obj.evidence), link_obj.evidence


# ---------------------------------------------------------- 未连接路径
def test_narrow_interfaces_blocked_when_not_connected():
    """未连接 → blocked/error + **0 次 SDK 调用**（不伪造任何数值）。"""
    fake = FakeLib(handle_id=-1)
    link_obj = bridge.connect("192.168.1.18", 8080, lib=fake)
    assert link_obj.status == errors.STATUS_ERROR

    before = len(fake.calls)
    res = link_obj.set_drive_max_speed(0, 10.0)
    assert res["status"] == errors.STATUS_ERROR, res
    assert res.get("sdk_calls") == 0

    res = link_obj.set_timeout(1.0)
    assert res["status"] == errors.STATUS_ERROR, res
    assert res.get("sdk_calls") == 0

    res = link_obj.controller_error_state()
    assert res["status"] == errors.STATUS_ERROR, res
    assert res["arm_err"] is None
    assert res["arm_current_status"] is None
    assert res["arm_current_status_name"] is None

    assert len(fake.calls) == before, "未连接路径不得调用 SDK"


def test_gen_structures_are_usable_offline():
    """冒烟：假 SDK 用的结构体在本环境可构造（保证测试自洽）。"""
    state = gen.rm_current_arm_state_t()
    assert state.err.err_len == 0
    assert isinstance(ctypes.sizeof(state), int)
    assert math.isfinite(bridge.DEG2RAD)
