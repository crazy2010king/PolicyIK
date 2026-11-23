"""关节角透传通道 + 停机族语义（T5 验收点）。

覆盖：

* **正例**：``transmit_joint`` 传给 SDK 的 ``joint[7]`` 内容与输入一致（**度**），
  ``follow`` / ``trajectory_mode`` / ``radio`` 正确；
* **缓冲生命周期**：调用返回并 GC 之后，SDK 侧仍能读到**正确数值**
  （防止 GC 提前回收导致静默错值）；
* **停机族映射**：``stop`` / ``slow_stop`` / ``pause`` / ``continue_`` /
  ``delete_current_trajectory`` 分别映射到正确的白名单符号；
* **负例**：非法 ``trajectory_mode`` / 关节数不为 7 / ``radio`` 越界 /
  非数字关节角 → 拒绝且 **0 次 SDK 调用**；
* ``current_trajectory_type()`` 正确返回枚举名（``RM_NO_PLANNING_E`` 等）；
* **黑名单隔离**：透传路径绝不触碰 ``rm_movep_canfd`` / ``rm_movep_follow``
  （位姿透传会触发控制器逆解，违反 R003）。

只用**注入的假 SDK**，不依赖 ``.deps/``（gitignored）里的真 SDK。
"""
from __future__ import annotations

import ctypes
import gc
import os
import re

import pytest

from rm_hw_bridge import bridge, errors

# --------------------------------------------------------------- 假 SDK
TRANSMIT_SYMBOLS = (
    "rm_movej_canfd",
    "rm_movej",
    "rm_set_arm_stop",
    "rm_set_arm_slow_stop",
    "rm_set_arm_pause",
    "rm_set_arm_continue",
    "rm_set_delete_current_trajectory",
    "rm_get_arm_current_trajectory",
)

#: 黑名单符号（只出现在假 SDK 里，用于断言"从未被调用"）
BLACKLIST_PROBES = ("rm_movep_canfd", "rm_movep_follow", "rm_movep", "rm_movej_p")


class FakeHandle:
    """假句柄（``id`` 直接暴露在属性上）。"""

    def __init__(self, handle_id: int = 1):
        self.id = handle_id


class FakeLib:
    """假动态库：实现连接 + 透传/停机族所需的全部白名单符号。"""

    def __init__(self, handle_id: int = 1, results=None, traj_type: int = 0):
        self.calls = []
        self.results = dict(results or {})
        self.handle = FakeHandle(handle_id)
        self.traj_type = int(traj_type)
        # 最近一次 rm_movej_canfd 的原始配置（用于生命周期校验）
        self.last_config = None
        self.last_config_joint = None

    def _rc(self, name, default=0):
        return self.results.get(name, default)

    # -- 连接
    def rm_init(self, mode):
        self.calls.append(("rm_init", mode))
        return self._rc("rm_init")

    def rm_create_robot_arm(self, ip, port):
        self.calls.append(("rm_create_robot_arm", ip, port))
        return self._rc("rm_create_robot_arm", self.handle)

    def rm_delete_robot_arm(self, handle):
        self.calls.append(("rm_delete_robot_arm",))
        return self._rc("rm_delete_robot_arm")

    # -- 透传 / 预就位
    def rm_movej_canfd(self, handle, config):
        # 复制一份 SDK 侧"看到的"数值（模拟 C 侧立即取数）
        joint = [float(config.joint[i]) for i in range(7)]
        self.last_config = config
        self.last_config_joint = joint
        self.calls.append(
            (
                "rm_movej_canfd",
                joint,
                bool(config.follow),
                int(config.trajectory_mode),
                int(config.radio),
                float(config.expand),
            )
        )
        return self._rc("rm_movej_canfd")

    def rm_movej(self, handle, joint, v, r, trajectory_connect, block):
        values = [float(joint[i]) for i in range(7)]
        self.calls.append(
            ("rm_movej", values, int(v), int(r), int(trajectory_connect), int(block))
        )
        return self._rc("rm_movej")

    # -- 停机族
    def rm_set_arm_stop(self, handle):
        self.calls.append(("rm_set_arm_stop",))
        return self._rc("rm_set_arm_stop")

    def rm_set_arm_slow_stop(self, handle):
        self.calls.append(("rm_set_arm_slow_stop",))
        return self._rc("rm_set_arm_slow_stop")

    def rm_set_arm_pause(self, handle):
        self.calls.append(("rm_set_arm_pause",))
        return self._rc("rm_set_arm_pause")

    def rm_set_arm_continue(self, handle):
        self.calls.append(("rm_set_arm_continue",))
        return self._rc("rm_set_arm_continue")

    def rm_set_delete_current_trajectory(self, handle):
        self.calls.append(("rm_set_delete_current_trajectory",))
        return self._rc("rm_set_delete_current_trajectory")

    # -- 规划类型
    def rm_get_arm_current_trajectory(self, handle, type_ptr, data):
        # 兼容 ctypes.pointer()（LP_c_int）与 ctypes.byref()（CArgObject）两种传法
        if hasattr(type_ptr, "_obj"):
            type_ptr._obj.value = self.traj_type
        else:
            type_ptr.contents.value = self.traj_type
        values = [float(data[i]) for i in range(7)]
        self.calls.append(("rm_get_arm_current_trajectory", self.traj_type, values))
        return self._rc("rm_get_arm_current_trajectory")

    # -- 黑名单探针（**绝不应被调用**）
    def rm_movep_canfd(self, handle, config):
        self.calls.append(("rm_movep_canfd",))
        return 0

    def rm_movep_follow(self, handle, pose):
        self.calls.append(("rm_movep_follow",))
        return 0

    def rm_movep(self, handle, pose, v, r, trajectory_connect, block):
        self.calls.append(("rm_movep",))
        return 0

    def rm_movej_p(self, handle, pose, v, r, trajectory_connect, block):
        self.calls.append(("rm_movej_p",))
        return 0


@pytest.fixture
def link_factory():
    """构造已连接的 :class:`ArmLink`（假 SDK）。"""

    def _make(results=None, handle_id=1, traj_type=0):
        lib = FakeLib(handle_id=handle_id, results=results, traj_type=traj_type)
        link = bridge.connect("192.168.1.18", 8080, lib=lib, enable_realtime=False)
        assert link.connected, link.reason
        # 连接过程不算运动类调用
        lib.calls = []
        return link, lib

    return _make


DEG7 = [0.0, 10.0, -20.0, 30.0, -40.0, 50.0, -60.0]


# ------------------------------------------------------------------ 正例
def test_transmit_joint_passes_expected_arguments(link_factory):
    """SDK 侧看到的 joint[7]/follow/trajectory_mode/radio 与入参一致（度）。"""
    link, lib = link_factory()
    res = link.transmit_joint(DEG7, follow=True, trajectory_mode=2, radio=500)

    assert res["status"] == errors.STATUS_OK
    assert res["symbol"] == "rm_movej_canfd"
    assert res["sdk_calls"] == 1
    calls = [c for c in lib.calls if c[0] == "rm_movej_canfd"]
    assert len(calls) == 1
    joint, follow, mode, radio, expand = calls[0][1:]
    assert joint == pytest.approx(DEG7), "关节角必须是原始度值，不做换算"
    assert follow is True
    assert mode == 2
    assert radio == 500
    assert expand == 0.0
    assert res["joint_deg"] == pytest.approx(DEG7)


def test_transmit_joint_default_mode_is_curve_fit(link_factory):
    """默认 trajectory_mode=1（曲线拟合）、radio=30、follow=False。"""
    link, lib = link_factory()
    res = link.transmit_joint(DEG7)
    assert res["status"] == errors.STATUS_OK
    _name, joint, follow, mode, radio, _expand = [
        c for c in lib.calls if c[0] == "rm_movej_canfd"
    ][0]
    assert follow is False
    assert mode == 1
    assert radio == 30
    assert joint == pytest.approx(DEG7)


def test_transmit_joint_buffer_survives_gc(link_factory):
    """缓冲生命周期：调用返回并 GC 之后，SDK 侧仍能读到正确数值。"""
    link, lib = link_factory()
    res = link.transmit_joint(DEG7)
    assert res["status"] == errors.STATUS_OK

    # 强制 GC：若缓冲被提前回收，SDK 侧指针会读到垃圾值
    gc.collect()
    config = lib.last_config
    assert config is not None
    observed = [float(config.joint[i]) for i in range(7)]
    assert observed == pytest.approx(DEG7), "GC 后 SDK 侧读到的关节角必须仍然正确"


def test_transmit_joint_holds_buffer_reference(link_factory):
    """桥接层**显式持有**缓冲引用（不靠 GC 巧合）。"""
    link, lib = link_factory()
    link.transmit_joint(DEG7)
    assert link._last_transmit is not None
    assert link._last_transmit["buffer"] is not None
    assert len(link._joint_buffers) >= 1


def test_transmit_joint_retained_buffers_are_bounded(link_factory):
    """保留的缓冲**有界**（10 ms 调度下不会无限堆积）。"""
    link, lib = link_factory()
    for _ in range(bridge.MAX_RETAINED_JOINT_BUFFERS + 5):
        link.transmit_joint(DEG7)
    assert len(link._joint_buffers) <= bridge.MAX_RETAINED_JOINT_BUFFERS


def test_preposition_movej_calls_rm_movej(link_factory):
    """预就位 → rm_movej(handle, joint, v, r, trajectory_connect, block)。"""
    link, lib = link_factory()
    res = link.preposition_movej(DEG7, speed=20, radius=5, block=0)

    assert res["status"] == errors.STATUS_OK
    assert res["symbol"] == "rm_movej"
    calls = [c for c in lib.calls if c[0] == "rm_movej"]
    assert len(calls) == 1
    _name, joint, v, r, connect, block = calls[0]
    assert joint == pytest.approx(DEG7)
    assert (v, r, connect, block) == (20, 5, 0, 0)


# ------------------------------------------------------------- 停机族映射
def test_stop_family_maps_to_expected_symbols(link_factory):
    """停机族四个方法 + 删除轨迹分别映射到正确的白名单符号。"""
    link, lib = link_factory()

    assert link.stop()["status"] == errors.STATUS_OK
    assert link.slow_stop()["status"] == errors.STATUS_OK
    assert link.pause()["status"] == errors.STATUS_OK
    assert link.continue_()["status"] == errors.STATUS_OK
    assert link.delete_current_trajectory()["status"] == errors.STATUS_OK

    assert [c[0] for c in lib.calls] == [
        "rm_set_arm_stop",
        "rm_set_arm_slow_stop",
        "rm_set_arm_pause",
        "rm_set_arm_continue",
        "rm_set_delete_current_trajectory",
    ]


@pytest.mark.parametrize(
    "method,symbol",
    [
        ("stop", "rm_set_arm_stop"),
        ("slow_stop", "rm_set_arm_slow_stop"),
        ("pause", "rm_set_arm_pause"),
        ("continue_", "rm_set_arm_continue"),
        ("delete_current_trajectory", "rm_set_delete_current_trajectory"),
    ],
)
def test_stop_family_symbol_per_method(link_factory, method, symbol):
    """逐方法断言符号映射（防止方法写错符号）。"""
    link, lib = link_factory()
    res = getattr(link, method)()
    assert res["symbol"] == symbol
    assert [c[0] for c in lib.calls] == [symbol]


@pytest.mark.parametrize(
    "method",
    ["stop", "slow_stop", "pause", "continue_", "delete_current_trajectory"],
)
def test_stop_family_rc_nonzero_reports_symbol_and_code(link_factory, method):
    """停机族返回码非 0 → error + 接口名 + 返回码 + next_action。"""
    link, lib = link_factory(results={"rm_set_arm_stop": -1, "rm_set_arm_slow_stop": -1,
                                      "rm_set_arm_pause": -1, "rm_set_arm_continue": -1,
                                      "rm_set_delete_current_trajectory": -1})
    res = getattr(link, method)()
    assert res["status"] == errors.STATUS_ERROR
    assert errors.exit_code(res) == 4
    assert res["symbol"] in res["reason"]
    assert "-1" in res["reason"]
    assert res["next_action"]


# --------------------------------------------------------------- 负例
def test_invalid_trajectory_mode_rejected_with_zero_calls(link_factory):
    """非法 trajectory_mode → 拒绝且 0 次 SDK 调用。"""
    link, lib = link_factory()
    res = link.transmit_joint(DEG7, trajectory_mode=3)
    assert res["status"] == errors.STATUS_ERROR
    assert "trajectory_mode" in res["reason"]
    assert res["sdk_calls"] == 0
    assert lib.calls == [], "被拒时不得产生任何 SDK 调用"


def test_invalid_joint_count_rejected_with_zero_calls(link_factory):
    """关节数不为 7 → 拒绝且 0 次 SDK 调用。"""
    link, lib = link_factory()
    res = link.transmit_joint([0.0, 1.0, 2.0])
    assert res["status"] == errors.STATUS_ERROR
    assert "7" in res["reason"]
    assert res["sdk_calls"] == 0
    assert lib.calls == []


def test_non_numeric_joint_rejected_with_zero_calls(link_factory):
    """非数字关节角 → 拒绝且 0 次 SDK 调用。"""
    link, lib = link_factory()
    res = link.transmit_joint([0.0, 1.0, 2.0, 3.0, 4.0, 5.0, "六"])
    assert res["status"] == errors.STATUS_ERROR
    assert res["sdk_calls"] == 0
    assert lib.calls == []


@pytest.mark.parametrize(
    "mode,radio",
    [(1, 101), (1, -1), (2, 1001), (2, -1)],
)
def test_radio_out_of_range_rejected(link_factory, mode, radio):
    """radio 越界（曲线拟合 0..100 / 滤波 0..1000）→ 拒绝且 0 次调用。"""
    link, lib = link_factory()
    res = link.transmit_joint(DEG7, trajectory_mode=mode, radio=radio)
    assert res["status"] == errors.STATUS_ERROR
    assert "radio" in res["reason"]
    assert res["sdk_calls"] == 0
    assert lib.calls == []


def test_follow_type_rejected(link_factory):
    """follow 非 bool → 拒绝且 0 次调用。"""
    link, lib = link_factory()
    res = link.transmit_joint(DEG7, follow=1)
    assert res["status"] == errors.STATUS_ERROR
    assert res["sdk_calls"] == 0
    assert lib.calls == []


def test_transmit_rc_nonzero_reports_symbol_and_code(link_factory):
    """rm_movej_canfd 返回非 0 → error + 接口名 + 返回码。"""
    link, lib = link_factory(results={"rm_movej_canfd": -1})
    res = link.transmit_joint(DEG7)
    assert res["status"] == errors.STATUS_ERROR
    assert "rm_movej_canfd" in res["reason"]
    assert "-1" in res["reason"]
    assert res["next_action"]


@pytest.mark.parametrize("speed", [0, 101, -5])
def test_preposition_speed_out_of_range_rejected(link_factory, speed):
    """预就位速度百分比越界 → 拒绝且 0 次调用。"""
    link, lib = link_factory()
    res = link.preposition_movej(DEG7, speed=speed)
    assert res["status"] == errors.STATUS_ERROR
    assert "speed" in res["reason"]
    assert res["sdk_calls"] == 0
    assert lib.calls == []


def test_preposition_rc_nonzero_reports_symbol_and_code(link_factory):
    """rm_movej 返回非 0 → error + 接口名 + 返回码。"""
    link, lib = link_factory(results={"rm_movej": 1})
    res = link.preposition_movej(DEG7)
    assert res["status"] == errors.STATUS_ERROR
    assert "rm_movej" in res["reason"]
    assert "1" in res["reason"]


def test_disconnected_link_refuses_transmit_and_stop():
    """未连接 → 拒绝，且不产生任何 SDK 调用。"""
    lib = FakeLib()
    link = bridge.connect("192.168.1.18", 8080, lib=lib, enable_realtime=False)
    # 人为把状态改成 blocked
    link.status = errors.STATUS_BLOCKED
    link.reason = "假故障"
    res = link.transmit_joint(DEG7)
    assert res["status"] == errors.STATUS_BLOCKED
    assert res["sdk_calls"] == 0
    res2 = link.stop()
    assert res2["status"] == errors.STATUS_BLOCKED
    assert res2["sdk_calls"] == 0


# --------------------------------------------------------- 规划类型查询
def test_current_trajectory_type_returns_enum_name(link_factory):
    """规划类型 → 枚举名（RM_NO_PLANNING_E 等）。"""
    link, lib = link_factory(traj_type=0)
    res = link.current_trajectory_type()
    assert res["status"] == errors.STATUS_OK
    assert res["type"] == 0
    assert res["type_name"] == "RM_NO_PLANNING_E"
    assert res["planning"] is False
    assert len(res["data"]) == 7


def test_current_trajectory_type_detects_planning(link_factory):
    """控制器在规划时 → planning=True（用于检测"偷偷规划"）。"""
    link, lib = link_factory(traj_type=1)
    res = link.current_trajectory_type()
    assert res["type_name"] == "RM_JOINT_SPACE_PLANNING_E"
    assert res["planning"] is True


def test_current_trajectory_type_rc_nonzero(link_factory):
    """查询失败 → error + 接口名 + 返回码。"""
    link, lib = link_factory(results={"rm_get_arm_current_trajectory": -2})
    res = link.current_trajectory_type()
    assert res["status"] == errors.STATUS_ERROR
    assert "rm_get_arm_current_trajectory" in res["reason"]
    assert "-2" in res["reason"]


# ------------------------------------------------------------ 黑名单隔离
def test_transmit_path_never_calls_blacklisted_symbols(link_factory):
    """透传/停机路径绝不触碰位姿透传等黑名单符号。"""
    link, lib = link_factory()
    link.transmit_joint(DEG7)
    link.preposition_movej(DEG7)
    link.stop()
    link.slow_stop()
    link.pause()
    link.continue_()
    link.delete_current_trajectory()
    called = {c[0] for c in lib.calls}
    for probe in BLACKLIST_PROBES:
        assert probe not in called, "黑名单符号 %s 不得被调用" % probe


def test_bridge_source_has_no_blacklisted_call(pkg_root):
    """源码层面不得出现黑名单符号的调用形态。"""
    src = open(
        os.path.join(pkg_root, "rm_hw_bridge", "bridge.py"), "r", encoding="utf-8"
    ).read()
    for sym in ("rm_movep_canfd", "rm_movep_follow", "rm_movej_p", "rm_set_joint_zero_pos"):
        assert not re.search(r"\b%s\s*\(" % sym, src), "禁止调用黑名单符号 %s" % sym


def test_transmit_symbols_are_whitelisted():
    """用到的符号必须在契约白名单内。"""
    from rm_hw_bridge import loader

    allowed = set(loader.contract_symbols())
    for sym in TRANSMIT_SYMBOLS:
        assert sym in allowed
