"""``ArmLink.set_realtime_push_enabled`` 的**离线**守卫测试（假 SDK，不连真机）。

为什么需要这个文件（2026-11-23 真机窗口的实测教训）
-----------------------------------------------------
真机断流演练脚本 ``scripts/rm75_stream_loss_drill.py`` 在真机上调用
``ArmLink.set_realtime_push_enabled(False)`` 时抛出::

    AttributeError: 'ArmLink' object has no attribute '_udp_port'

—— 该方法引用了两个**并不存在**的私有属性（``_udp_port`` / ``_realtime_target_ip``），
而真实属性名是**不带下划线**的 ``udp_port`` / ``realtime_target_ip``（``ArmLink.__init__``）。
异常发生在**线程里**，随后进程以 SIGSEGV（core dump，退出码 139）结束，
真机断流演练因此**无法产出证据**，连带把 S10 的结案工件拖成 blocked。

本文件把「开关主动上报」这条真机路径钉死：

* 端口 / 目标 IP 取自建链时保存的公开属性（不引用不存在的私有属性）；
* ``enable`` 位与 ``cycle``（5 的倍数）正确下发；
* 显式传入 port / target_ip 时以传入值为准；
* 未连接时返回结构化错误（**不抛异常**、不静默通过）；
* ``udp_port`` 为 None 时端口下发 0 而不是抛 AttributeError。
"""
from __future__ import annotations

import types

from rm_hw_bridge import bridge, errors


class _FakeBinding:
    """假 binding：只实现 ``rm_set_realtime_push``，记录调用参数。"""

    def __init__(self, rc: int = 0):
        self.calls = []
        self._rc = rc

    def rm_set_realtime_push(self, handle, config):
        self.calls.append((handle, config))
        return self._rc


def _link(binding=None, connected: bool = True, udp_port=8091,
          target_ip="192.168.1.50", period_ms=5):
    """构造一个**已连接**的 ArmLink（不碰真机 SDK）。"""
    binding = binding or _FakeBinding()
    fake_bridge = types.SimpleNamespace(binding=binding)
    return bridge.ArmLink(
        bridge=fake_bridge,
        handle=object(),
        status=errors.STATUS_OK if connected else errors.STATUS_ERROR,
        realtime_period_ms=period_ms,
        realtime_target_ip=target_ip,
        udp_port=udp_port,
    )


# ------------------------------------------------------------------ 正例
def test_toggle_uses_link_saved_port_and_ip():
    """端口 / 目标 IP 取自建链时保存的公开属性（这条路径曾在真机崩掉）。"""
    binding = _FakeBinding()
    link = _link(binding)
    res = link.set_realtime_push_enabled(False)
    assert res["status"] == "ok", res
    assert len(binding.calls) == 1
    _handle, cfg = binding.calls[0]
    assert cfg.port == 8091, cfg.port
    assert cfg.ip == b"192.168.1.50", cfg.ip
    assert cfg.enable in (0, False), cfg.enable


def test_toggle_enable_bit_reflects_request():
    """enable 位随请求翻转（关上报 = 真机断流演练的关键动作）。"""
    binding = _FakeBinding()
    link = _link(binding)
    link.set_realtime_push_enabled(True)
    link.set_realtime_push_enabled(False)
    assert [c[1].enable for c in binding.calls] == [1, 0]


def test_cycle_is_multiple_of_five_ms():
    """cycle 必须是 5 ms 的倍数（SDK 要求），缺省取建链周期。"""
    binding = _FakeBinding()
    link = _link(binding, period_ms=10)
    link.set_realtime_push_enabled(True)
    assert binding.calls[0][1].cycle == 10
    assert binding.calls[0][1].cycle % 5 == 0


def test_explicit_port_and_ip_override():
    """显式给出 port / target_ip 时以传入值为准。"""
    binding = _FakeBinding()
    link = _link(binding)
    link.set_realtime_push_enabled(True, port=9000, target_ip="10.0.0.7")
    _handle, cfg = binding.calls[0]
    assert cfg.port == 9000
    assert cfg.ip == b"10.0.0.7"


def test_missing_udp_port_does_not_raise():
    """建链时未启用上报（udp_port=None）→ 端口下发 0，**不得**抛 AttributeError。"""
    binding = _FakeBinding()
    link = _link(binding, udp_port=None)
    res = link.set_realtime_push_enabled(False)
    assert res["status"] == "ok", res
    assert binding.calls[0][1].port == 0


# ------------------------------------------------------------------ 负例
def test_not_connected_returns_structured_error():
    """未连接 → 结构化错误（不抛异常、不下发 SDK 调用）。"""
    binding = _FakeBinding()
    link = _link(binding, connected=False)
    res = link.set_realtime_push_enabled(False)
    assert res["status"] != "ok", res
    assert res.get("reason") and res.get("next_action")
    assert binding.calls == [], "未连接时不得调用 SDK"


def test_sdk_nonzero_rc_is_error():
    """SDK 返回非 0 → error 且写明返回码（不伪造成功）。"""
    binding = _FakeBinding(rc=-1)
    link = _link(binding)
    res = link.set_realtime_push_enabled(False)
    assert res["status"] == "error", res
    assert "-1" in str(res.get("reason"))
