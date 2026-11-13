"""夹爪 **SDK 适配层**（硬件层内唯一直接碰夹爪 SDK 的模块）。

本模块把厂商 RM-API2 的**三条夹爪协议路径**收敛到 T2 的纯域逻辑
（:mod:`rm_hw_bridge.gripper_state`）：

============  ====================================================  ===================
路径          SDK 符号                                              用途
============  ====================================================  ===================
``modbus``    ``rm_set_modbus_mode`` / ``rm_read_holding_registers``  **主路径**
              ``rm_read_input_registers`` / ``rm_write_registers``   （三代机通用）
              ``rm_write_single_register`` / ``rm_close_modbus_mode``
``gripper_api``  ``rm_set_gripper_route/release/pick/pick_on/position``  官方**快路径**
              ``rm_get_gripper_state``
``rm_plus``   ``rm_set_rm_plus_mode`` / ``rm_get_rm_plus_mode``      只做
                                                                「协议未开(disable)
                                                                vs 已开但离线(offline)」
                                                                的区分，**不作判定真值**
============  ====================================================  ===================

供电只**回读**：``rm_get_tool_voltage``（法兰供电上限 1.5 A < 夹爪额定 1.9 A
→ 夹爪动力必须外部 24 V + 与法兰共地），``rm_set_tool_voltage`` **默认不写**。

三条纪律（与仓库既有约定一致）：

1. **生命周期强制**：协议「开 → 读写 → 关/恢复」。厂商头文件写明
   ``rm_set_modbus_mode`` 的配置**会被控制器持久化**，不关闭就把工具端 RS485
   留在 RTU 主站态；官方夹爪协议与末端 Modbus **互斥**。
   :meth:`GripperAdapter.with_protocol` 保证任何失败路径都恢复到**进入前**状态。
2. **红线**：寄存器 ``0x0008``（清错误码）**只读不写**（清错误码需重启设备，
   本项目禁止自动执行）。写该寄存器的调用一律 ``blocked`` 且**不产生任何 SDK 写调用**。
3. **不伪造**：任何失败路径返回统一结果字典
   （``status`` / ``reason`` / ``next_action`` / ``evidence``），**绝不**返回编造的
   寄存器值或状态。

**地址基数未定**（真机窗口一次性解决）：:func:`register_address` 同时支持
``base=1``（手册书写值）与 ``base=0``（PDU 0 基偏移）两套取值，默认 ``base=1``，
实际所用基数**必须**写进 evidence。

**只读寄存器在 FC03 还是 FC04 未定**：读路径同时提供 holding（FC03）与
input（FC04）两种读取函数，:meth:`GripperAdapter.probe` 两条都试。

**零真机可测**：:class:`FakeGripper` 是可脚本化的假夹爪（可设置寄存器值、错误位、
模式、是否超时），所有经它产生的状态都带 ``mode=fake`` 标注（**绝不冒充真机结论**）。
"""
from __future__ import annotations

import ctypes
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

from rm_hw_bridge import errors
from rm_hw_bridge.generated import rm_api2_ctypes as gen
from rm_hw_bridge.gripper_state import (
    PATH_GRIPPER_API,
    PATH_MODBUS,
    PATH_RM_PLUS,
    GripperState,
    normalize_state,
)

__all__ = [
    # 寄存器表
    "REG_SAVE",
    "REG_RESTORE_DEFAULT",
    "REG_DEVICE_ID",
    "REG_BAUD",
    "REG_GRAB_MODE",
    "REG_OPEN_MAX",
    "REG_ESTOP",
    "REG_CLEAR_ERROR",
    "REG_TARGET_POS",
    "REG_SPEED",
    "REG_FORCE",
    "REG_MAX_POS",
    "REG_MIN_POS",
    "REG_ACTUAL_FORCE",
    "REG_ACTUAL_POS",
    "REG_ACTUAL_CURRENT",
    "REG_ACTUAL_VOLTAGE",
    "REG_ACTUAL_TEMP",
    "REG_ERROR",
    "REG_STATE",
    "REGISTER_NAMES",
    "READ_ONLY_REGISTERS",
    "FORBIDDEN_WRITE_REGISTERS",
    "register_address",
    # 返回码语义
    "RC_SEND_FAILED",
    "RC_RECV_TIMEOUT",
    "RC_PARSE_FAILED",
    "RC_UNSUPPORTED",
    "RC_SEMANTICS",
    "describe_rc",
    # 端口
    "GripperPort",
    "CtypesSdkPort",
    "FakeGripper",
    # 适配器
    "GripperAdapter",
]


# --------------------------------------------------------------------------
# 1. 寄存器表（厂商《两指平行夹爪用户使用手册 V1.0》§4.1）
# --------------------------------------------------------------------------

#: 保存参数（读写）
REG_SAVE = 0x0001
#: 恢复默认（读写）
REG_RESTORE_DEFAULT = 0x0002
#: 设备地址 / ID（读写，重启生效）
REG_DEVICE_ID = 0x0003
#: 波特率档位（读写，重启生效）
REG_BAUD = 0x0004
#: 抓取模式（0 单次 / 1 持续）
REG_GRAB_MODE = 0x0005
#: 松开到最大开口
REG_OPEN_MAX = 0x0006
#: 急停
REG_ESTOP = 0x0007
#: **清除错误码（需重启设备）→ 本项目禁止写**
REG_CLEAR_ERROR = 0x0008
#: 目标开口度（0–1000）
REG_TARGET_POS = 0x0009
#: 夹取速度（0–1000）
REG_SPEED = 0x000A
#: 夹取力度（0–1000）
REG_FORCE = 0x000B
#: 最大开口度（0–1000）
REG_MAX_POS = 0x000C
#: 最小开口度（0–1000）
REG_MIN_POS = 0x000D
#: 实际力度（只读，0–1000）
REG_ACTUAL_FORCE = 0x000E
#: 实际位置 / 开口度（只读，0–1000）
REG_ACTUAL_POS = 0x000F
#: 实际电流（只读，mA）
REG_ACTUAL_CURRENT = 0x0010
#: 实际电压（只读，V）
REG_ACTUAL_VOLTAGE = 0x0011
#: 实际温度（只读，℃）
REG_ACTUAL_TEMP = 0x0012
#: 夹爪错误码位图（只读）
REG_ERROR = 0x0013
#: 状态码（只读：1 最大空闲 / 2 最小空闲 / 3 停止空闲 / 4 正在闭合 / 5 正在张开）
REG_STATE = 0x0014

#: 寄存器地址 → 中文名（留痕用）
REGISTER_NAMES: Dict[int, str] = {
    REG_SAVE: "保存参数",
    REG_RESTORE_DEFAULT: "恢复默认",
    REG_DEVICE_ID: "设备地址(ID)",
    REG_BAUD: "波特率档位",
    REG_GRAB_MODE: "抓取模式",
    REG_OPEN_MAX: "松开到最大开口",
    REG_ESTOP: "急停",
    REG_CLEAR_ERROR: "清除错误码(禁止写)",
    REG_TARGET_POS: "目标开口度",
    REG_SPEED: "夹取速度",
    REG_FORCE: "夹取力度",
    REG_MAX_POS: "最大开口度",
    REG_MIN_POS: "最小开口度",
    REG_ACTUAL_FORCE: "实际力度",
    REG_ACTUAL_POS: "实际位置/开口度",
    REG_ACTUAL_CURRENT: "实际电流",
    REG_ACTUAL_VOLTAGE: "实际电压",
    REG_ACTUAL_TEMP: "实际温度",
    REG_ERROR: "错误码位图",
    REG_STATE: "状态码",
}

#: 手册标注为**只读**的寄存器（写它们一律拒绝）
READ_ONLY_REGISTERS = frozenset({
    REG_ACTUAL_FORCE,
    REG_ACTUAL_POS,
    REG_ACTUAL_CURRENT,
    REG_ACTUAL_VOLTAGE,
    REG_ACTUAL_TEMP,
    REG_ERROR,
    REG_STATE,
})

#: **禁止写入**的寄存器 → 原因（红线：清错误码需重启设备）
FORBIDDEN_WRITE_REGISTERS: Dict[int, str] = {
    REG_CLEAR_ERROR: "寄存器 0x0008 是「清除错误码」，需重启设备才生效，本项目禁止自动写",
}

#: 探针扫描的地址窗口（含端点）
PROBE_ADDRESS_RANGE = (0x0000, 0x0020)

#: 探针的物理合理性窗口（用于猜地址基数 / 判断读到的是不是夹爪数据）
PROBE_WINDOWS: Dict[str, Tuple[float, float]] = {
    "position": (0.0, 1000.0),
    "temperature": (20.0, 60.0),
    "current": (0.0, 2400.0),
}


def register_address(reg: int, base: int = 1) -> int:
    """寄存器**手册地址** → **线上（PDU）地址**。

    基数语义（真机窗口一次性判定，当前**两种都要支持**）：

    * ``base=1``：线上地址 = 手册地址（手册 §4.1 直接书写 0x000E / 0x0013）；
    * ``base=0``：线上地址 = 手册地址 − 1（PDU 0 基偏移）。

    :param reg: 手册地址（:data:`REG_*` 常量）
    :param base: 地址基数（1 = 手册写法，0 = PDU 0 基）
    :raises ValueError: base 不是 0 / 1，或 reg 不是整数
    """
    if isinstance(reg, bool) or not isinstance(reg, int):
        raise TypeError("reg 必须是 int（手册地址），收到 %r" % type(reg).__name__)
    if base not in (0, 1):
        raise ValueError("base 只能是 0 或 1（地址基数），收到 %r" % (base,))
    return reg if base == 1 else reg - 1


def register_name(reg: int) -> str:
    """手册地址 → 名称（未知地址给十六进制串）。"""
    return REGISTER_NAMES.get(reg, "0x%04X" % reg)


# --------------------------------------------------------------------------
# 2. SDK 返回码语义（厂商 rm_interface.h 注释）
# --------------------------------------------------------------------------

#: 数据发送失败
RC_SEND_FAILED = -1
#: 数据接收失败或控制器超时未返回
RC_RECV_TIMEOUT = -2
#: 返回值解析失败（数据格式不正确或不完整）
RC_PARSE_FAILED = -3
#: 四代控制器不支持该接口
RC_UNSUPPORTED = -4

#: 返回码 → 中文语义
RC_SEMANTICS: Dict[int, str] = {
    RC_SEND_FAILED: "数据发送失败",
    RC_RECV_TIMEOUT: "数据接收失败或超时（控制器未在超时时间内返回）",
    RC_PARSE_FAILED: "返回值解析失败（数据格式不正确或不完整）",
    RC_UNSUPPORTED: "四代控制器不支持该接口",
}


def describe_rc(rc: int) -> str:
    """返回码 → 人类可读描述（未知码给原始值，绝不编造语义）。"""
    try:
        code = int(rc)
    except (TypeError, ValueError):
        return "返回码不可解析: %r" % (rc,)
    if code == 0:
        return "成功"
    return "%d（%s）" % (code, RC_SEMANTICS.get(code, "厂商头文件未定义该返回码"))


# --------------------------------------------------------------------------
# 3. 端口抽象：SDK 调用面
# --------------------------------------------------------------------------

class GripperPort:
    """夹爪 SDK 调用面（**鸭子类型**）。

    两个实现：

    * :class:`CtypesSdkPort` —— 真机：直接调 ``rm_hw_bridge.loader.SymbolBinding``
      绑好的 ctypes 符号；
    * :class:`FakeGripper` —— 离线：可脚本化的假夹爪。

    参数一律用**普通 Python 类型**（port / address / device / num / 值列表），
    由具体实现负责组装 ``rm_peripheral_read_write_params_t``（按值传参）。
    """

    #: 是否为**假**夹爪（经它产生的状态必须带 mode=fake 标注）
    is_fake = False

    # -- 协议开关 ----------------------------------------------------------
    def set_modbus_mode(self, port: int, baudrate: int, timeout_100ms: int) -> int:
        raise NotImplementedError

    def close_modbus_mode(self, port: int) -> int:
        raise NotImplementedError

    def set_rm_plus_mode(self, mode: int) -> int:
        raise NotImplementedError

    def get_rm_plus_mode(self) -> Tuple[int, Optional[int]]:
        raise NotImplementedError

    # -- 寄存器读写 --------------------------------------------------------
    def read_holding_register(self, port: int, address: int, device: int) -> Tuple[int, Optional[int]]:
        raise NotImplementedError

    def read_input_register(self, port: int, address: int, device: int) -> Tuple[int, Optional[int]]:
        raise NotImplementedError

    def write_single_register(self, port: int, address: int, device: int, value: int) -> int:
        raise NotImplementedError

    def write_registers(self, port: int, address: int, device: int, values: Sequence[int]) -> int:
        raise NotImplementedError

    # -- 官方夹爪快路径 ----------------------------------------------------
    def set_gripper_route(self, min_limit: int, max_limit: int) -> int:
        raise NotImplementedError

    def set_gripper_release(self, speed: int, block: bool, timeout: int) -> int:
        raise NotImplementedError

    def set_gripper_pick(self, speed: int, force: int, block: bool, timeout: int) -> int:
        raise NotImplementedError

    def set_gripper_pick_on(self, speed: int, force: int, block: bool, timeout: int) -> int:
        raise NotImplementedError

    def set_gripper_position(self, position: int, block: bool, timeout: int) -> int:
        raise NotImplementedError

    def get_gripper_state(self) -> Tuple[int, Optional[Dict[str, Any]]]:
        raise NotImplementedError

    # -- 工具端供电（只回读） ----------------------------------------------
    def get_tool_voltage(self) -> Tuple[int, Optional[int]]:
        raise NotImplementedError

    def set_tool_voltage(self, voltage_type: int) -> int:
        raise NotImplementedError


class CtypesSdkPort(GripperPort):
    """真机端口：把普通参数翻译成厂商 SDK 的 ctypes 调用形态。

    * ``rm_peripheral_read_write_params_t`` **按值**传参（port / address / device / num）；
    * 读寄存器每次**只读 1 个**（厂商头文件：``rm_read_*_registers`` 一次只能读
      1 个寄存器，``num`` 无需设置）；
    * ``rm_write_registers`` 一次最多 10 个寄存器（``num <= 10``）。

    :param binding: ``loader.SymbolBinding``（只暴露契约清单内符号）
    :param handle: ``rm_robot_handle*``
    """

    def __init__(self, binding: Any, handle: Any) -> None:
        self._binding = binding
        self._handle = handle

    @property
    def binding(self) -> Any:
        return self._binding

    @property
    def handle(self) -> Any:
        return self._handle

    # -- 内部 --------------------------------------------------------------
    def _params(self, port: int, address: int, device: int, num: int):
        return gen.rm_peripheral_read_write_params_t(
            port=int(port), address=int(address), device=int(device), num=int(num)
        )

    # -- 协议开关 ----------------------------------------------------------
    def set_modbus_mode(self, port: int, baudrate: int, timeout_100ms: int) -> int:
        return int(self._binding.rm_set_modbus_mode(
            self._handle, int(port), int(baudrate), int(timeout_100ms)
        ))

    def close_modbus_mode(self, port: int) -> int:
        return int(self._binding.rm_close_modbus_mode(self._handle, int(port)))

    def set_rm_plus_mode(self, mode: int) -> int:
        return int(self._binding.rm_set_rm_plus_mode(self._handle, int(mode)))

    def get_rm_plus_mode(self) -> Tuple[int, Optional[int]]:
        buf = ctypes.c_int(0)
        rc = int(self._binding.rm_get_rm_plus_mode(self._handle, buf))
        return rc, (int(buf.value) if rc == 0 else None)

    # -- 寄存器读写 --------------------------------------------------------
    def read_holding_register(self, port: int, address: int, device: int) -> Tuple[int, Optional[int]]:
        buf = (ctypes.c_int * 1)()
        rc = int(self._binding.rm_read_holding_registers(
            self._handle, self._params(port, address, device, 1), buf
        ))
        return rc, (int(buf[0]) if rc == 0 else None)

    def read_input_register(self, port: int, address: int, device: int) -> Tuple[int, Optional[int]]:
        buf = (ctypes.c_int * 1)()
        rc = int(self._binding.rm_read_input_registers(
            self._handle, self._params(port, address, device, 1), buf
        ))
        return rc, (int(buf[0]) if rc == 0 else None)

    def write_single_register(self, port: int, address: int, device: int, value: int) -> int:
        return int(self._binding.rm_write_single_register(
            self._handle, self._params(port, address, device, 1), int(value)
        ))

    def write_registers(self, port: int, address: int, device: int, values: Sequence[int]) -> int:
        data = (ctypes.c_int * len(values))(*[int(v) for v in values])
        return int(self._binding.rm_write_registers(
            self._handle, self._params(port, address, device, len(values)), data
        ))

    # -- 官方夹爪快路径 ----------------------------------------------------
    def set_gripper_route(self, min_limit: int, max_limit: int) -> int:
        return int(self._binding.rm_set_gripper_route(
            self._handle, int(min_limit), int(max_limit)
        ))

    def set_gripper_release(self, speed: int, block: bool, timeout: int) -> int:
        return int(self._binding.rm_set_gripper_release(
            self._handle, int(speed), bool(block), int(timeout)
        ))

    def set_gripper_pick(self, speed: int, force: int, block: bool, timeout: int) -> int:
        return int(self._binding.rm_set_gripper_pick(
            self._handle, int(speed), int(force), bool(block), int(timeout)
        ))

    def set_gripper_pick_on(self, speed: int, force: int, block: bool, timeout: int) -> int:
        return int(self._binding.rm_set_gripper_pick_on(
            self._handle, int(speed), int(force), bool(block), int(timeout)
        ))

    def set_gripper_position(self, position: int, block: bool, timeout: int) -> int:
        return int(self._binding.rm_set_gripper_position(
            self._handle, int(position), bool(block), int(timeout)
        ))

    def get_gripper_state(self) -> Tuple[int, Optional[Dict[str, Any]]]:
        state = gen.rm_gripper_state_t()
        rc = int(self._binding.rm_get_gripper_state(self._handle, state))
        if rc != 0:
            return rc, None
        return rc, {
            "enable_state": int(state.enable_state),
            "status": int(state.status),
            "error": int(state.error),
            "mode": int(state.mode),
            "current_force": int(state.current_force),
            "temperature": int(state.temperature),
            "actpos": int(state.actpos),
        }

    # -- 工具端供电 --------------------------------------------------------
    def get_tool_voltage(self) -> Tuple[int, Optional[int]]:
        buf = ctypes.c_int(0)
        rc = int(self._binding.rm_get_tool_voltage(self._handle, buf))
        return rc, (int(buf.value) if rc == 0 else None)

    def set_tool_voltage(self, voltage_type: int) -> int:
        return int(self._binding.rm_set_tool_voltage(self._handle, int(voltage_type)))


# --------------------------------------------------------------------------
# 4. 假夹爪
# --------------------------------------------------------------------------

class FakeGripper(GripperPort):
    """可脚本化的**假夹爪**（照抄 ``rm_executor/transport.py::FakeTransport`` 范式）。

    能力：

    * 设置寄存器值（按**手册地址**，与线上基数解耦）；
    * 设置错误位、模式、在线状态；
    * 脚本化**超时**（按操作名或全局开关）；
    * 记录全部调用（``calls``，可查 ``calls_for`` / ``count``）；
    * ``is_fake = True`` —— 经它产生的状态一律带 ``mode=fake`` 标注。

    ``base`` 表示**假设备的线上基数**：默认 1（线上地址 = 手册地址）。
    把它设成 0 可复现「基数猜错」的场景。
    """

    is_fake = True

    def __init__(
        self,
        base: int = 1,
        registers: Optional[Dict[int, int]] = None,
        mode: Optional[int] = None,
        error: int = 0,
        online: bool = True,
        enable_state: int = 1,
        current_force: int = 0,
        temperature: int = 25,
        actpos: int = 0,
        rm_plus_mode: int = 0,
        tool_voltage: int = 0,
        timeout_ops: Optional[Sequence[str]] = None,
        timeout_all: bool = False,
        max_records: Optional[int] = None,
    ) -> None:
        if base not in (0, 1):
            raise ValueError("base 只能是 0 或 1")
        self.base = int(base)
        #: 手册地址 → 值
        self.registers: Dict[int, int] = dict(registers or {})
        self.mode = mode
        self.error = int(error)
        self.online = bool(online)
        self.enable_state = int(enable_state)
        self.current_force = int(current_force)
        self.temperature = int(temperature)
        self.actpos = int(actpos)
        self.rm_plus_mode = int(rm_plus_mode)
        self.tool_voltage = int(tool_voltage)
        self.timeout_all = bool(timeout_all)
        self.timeout_ops = set(timeout_ops or ())
        #: 调用记录的**保留上限**（``None`` = 不限）；计数不受上限影响
        self._max_records = None if max_records is None else int(max_records)
        self.calls: List[Dict[str, Any]] = []
        self._counts: Dict[str, int] = {}

    # -- 脚本化 ------------------------------------------------------------
    def set_register(self, reg: int, value: int) -> "FakeGripper":
        """设置一个寄存器值（手册地址）。"""
        self.registers[int(reg)] = int(value)
        return self

    def set_registers(self, values: Dict[int, int]) -> "FakeGripper":
        """批量设置寄存器值（手册地址）。"""
        for reg, value in values.items():
            self.set_register(reg, value)
        return self

    def set_error_bits(self, bits: int) -> "FakeGripper":
        """设置错误位图。"""
        self.error = int(bits)
        return self

    def set_mode(self, mode: Optional[int]) -> "FakeGripper":
        """设置状态码 / SDK mode。"""
        self.mode = None if mode is None else int(mode)
        return self

    def set_online(self, online: bool) -> "FakeGripper":
        self.online = bool(online)
        return self

    def set_timeout(self, *ops: str) -> "FakeGripper":
        """让指定操作（``read`` / ``write`` / ``modbus_open`` …）返回超时。"""
        self.timeout_ops.update(ops)
        return self

    def clear_timeout(self) -> "FakeGripper":
        self.timeout_ops.clear()
        self.timeout_all = False
        return self

    # -- 调用记录 ----------------------------------------------------------
    def _record(self, op: str, args: Dict[str, Any]) -> None:
        """记录一次调用（记录可设上限，**计数永不受淘汰影响**）。"""
        self.calls.append({"op": op, "args": dict(args)})
        self._counts[op] = self._counts.get(op, 0) + 1
        if self._max_records is not None and len(self.calls) > self._max_records:
            del self.calls[: len(self.calls) - self._max_records]

    @property
    def call_count(self) -> int:
        """已记录的有效调用总数（**含已被淘汰的**）。"""
        return sum(self._counts.values())

    def calls_for(self, op: str) -> List[Dict[str, Any]]:
        """按操作名过滤调用记录（可能已被上限淘汰掉一部分）。"""
        return [c for c in self.calls if c["op"] == op]

    def count(self, op: str) -> int:
        """某操作的调用次数（**永不受 max_records 淘汰影响**）。"""
        return self._counts.get(op, 0)

    def clear_calls(self) -> None:
        """清空调用记录与计数。"""
        self.calls = []
        self._counts = {}

    def _timed_out(self, op: str) -> bool:
        return self.timeout_all or op in self.timeout_ops

    # -- 内部：线上地址 → 手册地址 -----------------------------------------
    def _canonical(self, wire_address: int) -> int:
        """线上地址 → 手册地址（按假设备的基数）。"""
        return int(wire_address) + (1 - self.base)

    def _value(self, address: int) -> int:
        return int(self.registers.get(self._canonical(address), 0))

    # -- 协议开关 ----------------------------------------------------------
    def set_modbus_mode(self, port: int, baudrate: int, timeout_100ms: int) -> int:
        self._record("set_modbus_mode", {
            "port": port, "baudrate": baudrate, "timeout_100ms": timeout_100ms,
        })
        if self._timed_out("set_modbus_mode"):
            return RC_RECV_TIMEOUT
        return 0

    def close_modbus_mode(self, port: int) -> int:
        self._record("close_modbus_mode", {"port": port})
        return 0

    def set_rm_plus_mode(self, mode: int) -> int:
        self._record("set_rm_plus_mode", {"mode": mode})
        self.rm_plus_mode = int(mode)
        return 0

    def get_rm_plus_mode(self) -> Tuple[int, Optional[int]]:
        self._record("get_rm_plus_mode", {})
        if self._timed_out("get_rm_plus_mode"):
            return RC_RECV_TIMEOUT, None
        return 0, int(self.rm_plus_mode)

    # -- 寄存器读写 --------------------------------------------------------
    def read_holding_register(self, port: int, address: int, device: int) -> Tuple[int, Optional[int]]:
        self._record("read_holding_register", {
            "port": port, "address": address, "device": device, "num": 1,
        })
        if self._timed_out("read_holding_register"):
            return RC_RECV_TIMEOUT, None
        return 0, self._value(address)

    def read_input_register(self, port: int, address: int, device: int) -> Tuple[int, Optional[int]]:
        self._record("read_input_register", {
            "port": port, "address": address, "device": device, "num": 1,
        })
        if self._timed_out("read_input_register"):
            return RC_RECV_TIMEOUT, None
        return 0, self._value(address)

    def write_single_register(self, port: int, address: int, device: int, value: int) -> int:
        self._record("write_single_register", {
            "port": port, "address": address, "device": device, "value": int(value),
        })
        if self._timed_out("write_single_register"):
            return RC_RECV_TIMEOUT
        self.registers[self._canonical(address)] = int(value)
        return 0

    def write_registers(self, port: int, address: int, device: int, values: Sequence[int]) -> int:
        self._record("write_registers", {
            "port": port, "address": address, "device": device,
            "values": [int(v) for v in values],
        })
        if self._timed_out("write_registers"):
            return RC_RECV_TIMEOUT
        for offset, value in enumerate(values):
            self.registers[self._canonical(int(address) + offset)] = int(value)
        return 0

    # -- 官方夹爪快路径 ----------------------------------------------------
    def set_gripper_route(self, min_limit: int, max_limit: int) -> int:
        self._record("set_gripper_route", {"min_limit": min_limit, "max_limit": max_limit})
        return 0

    def set_gripper_release(self, speed: int, block: bool, timeout: int) -> int:
        self._record("set_gripper_release", {"speed": speed, "block": block, "timeout": timeout})
        return 0

    def set_gripper_pick(self, speed: int, force: int, block: bool, timeout: int) -> int:
        self._record("set_gripper_pick", {
            "speed": speed, "force": force, "block": block, "timeout": timeout,
        })
        return 0

    def set_gripper_pick_on(self, speed: int, force: int, block: bool, timeout: int) -> int:
        self._record("set_gripper_pick_on", {
            "speed": speed, "force": force, "block": block, "timeout": timeout,
        })
        return 0

    def set_gripper_position(self, position: int, block: bool, timeout: int) -> int:
        self._record("set_gripper_position", {"position": position, "block": block, "timeout": timeout})
        return 0

    def get_gripper_state(self) -> Tuple[int, Optional[Dict[str, Any]]]:
        self._record("get_gripper_state", {})
        if self._timed_out("get_gripper_state"):
            return RC_RECV_TIMEOUT, None
        return 0, {
            "enable_state": int(self.enable_state),
            "status": 1 if self.online else 0,
            "error": int(self.error),
            "mode": 0 if self.mode is None else int(self.mode),
            "current_force": int(self.current_force),
            "temperature": int(self.temperature),
            "actpos": int(self.actpos),
        }

    # -- 工具端供电 --------------------------------------------------------
    def get_tool_voltage(self) -> Tuple[int, Optional[int]]:
        self._record("get_tool_voltage", {})
        if self._timed_out("get_tool_voltage"):
            return RC_RECV_TIMEOUT, None
        return 0, int(self.tool_voltage)

    def set_tool_voltage(self, voltage_type: int) -> int:
        self._record("set_tool_voltage", {"voltage_type": int(voltage_type)})
        self.tool_voltage = int(voltage_type)
        return 0


# --------------------------------------------------------------------------
# 5. 适配器
# --------------------------------------------------------------------------

#: 读寄存器用的功能码标签
FUNCTION_HOLDING = "holding"
FUNCTION_INPUT = "input"
FUNCTIONS = (FUNCTION_HOLDING, FUNCTION_INPUT)


class GripperAdapter:
    """夹爪适配器：把三条协议路径统一成 T2 的 :class:`GripperState`。

    :param port: :class:`GripperPort`（真机 :class:`CtypesSdkPort` / 离线
        :class:`FakeGripper`）
    :param path: 协议路径（``modbus`` / ``gripper_api`` / ``rm_plus``）
    :param device_id: Modbus 从站地址（手册默认 1）
    :param baud: Modbus 波特率（手册默认 115200）
    :param timeout_100ms: ``rm_set_modbus_mode`` 超时（**单位百毫秒**）
    :param base: 寄存器地址基数（1 = 手册写法，0 = PDU 0 基）
    :param position_mm_per_unit: 开口度换算系数（mm / 源单位）；缺失 → ``None``
    :param force_scale: 力度换算系数（N / 源单位）；缺失 → ``None``
    """

    def __init__(
        self,
        port: GripperPort,
        path: str = PATH_MODBUS,
        device_id: int = 1,
        baud: int = 115200,
        timeout_100ms: int = 5,
        base: int = 1,
        position_mm_per_unit: Optional[float] = None,
        force_scale: Optional[float] = None,
    ) -> None:
        if path not in (PATH_MODBUS, PATH_GRIPPER_API, PATH_RM_PLUS):
            raise ValueError("未知协议路径: %r" % (path,))
        if base not in (0, 1):
            raise ValueError("base 只能是 0 或 1（地址基数）")
        self._port = port
        self._path = path
        self._device_id = int(device_id)
        self._baud = int(baud)
        self._timeout_100ms = int(timeout_100ms)
        self._base = int(base)
        self._position_mm_per_unit = position_mm_per_unit
        self._force_scale = force_scale
        self._protocol_open = False
        #: 最近一次已知的 RM Plus 协议开关（用于区分 disable / offline）
        self._rm_plus_mode_last: Optional[int] = None

    # -- 属性 --------------------------------------------------------------
    @property
    def port(self) -> GripperPort:
        return self._port

    @property
    def path(self) -> str:
        return self._path

    @property
    def base(self) -> int:
        """当前使用的地址基数（1 / 0）。"""
        return self._base

    @property
    def protocol_open(self) -> bool:
        """Modbus 协议是否已打开。"""
        return self._protocol_open

    @property
    def is_fake(self) -> bool:
        """底层端口是否为假夹爪（决定状态是否带 ``mode=fake`` 标注）。"""
        return bool(getattr(self._port, "is_fake", False))

    # -- 结果构造 ----------------------------------------------------------
    def _ok(self, reason: str, evidence: Optional[List[Any]] = None, **extra: Any) -> Dict[str, Any]:
        return errors.ok(reason, "", evidence or [], **extra)

    def _rc_failure(self, symbol: str, rc: int, action: str, evidence: List[Any]) -> Dict[str, Any]:
        """按厂商返回码语义构造失败结果。

        ``-4``（四代控制器不支持该接口）→ ``blocked``（能力缺失，需换路径或人工）；
        其余非 0 → ``error``。
        """
        rc = int(rc)
        detail = "%s 返回 %s：%s" % (symbol, rc, describe_rc(rc))
        evidence = list(evidence) + ["%s rc=%d" % (symbol, rc)]
        if rc == RC_UNSUPPORTED:
            return errors.blocked(
                "%s（%s）" % (detail, action),
                "当前控制器代际不支持该接口：改走 Modbus-RTU 主路径，"
                "或在真机窗口确认控制器代际后再重试",
                evidence,
                rc=rc,
                symbol=symbol,
            )
        return errors.error(
            "%s（%s）" % (detail, action),
            "检查控制器与夹爪连线、波特率与从站地址后重跑同一条命令",
            evidence,
            rc=rc,
            symbol=symbol,
        )

    def _fake_tag(self) -> List[Any]:
        return ["mode=fake（离线假夹爪，不代表真机结论）"] if self.is_fake else []

    # -- 生命周期 ----------------------------------------------------------
    def open_protocol(self) -> Dict[str, Any]:
        """打开末端 Modbus-RTU 协议（``rm_set_modbus_mode``）。

        厂商头文件：该配置**会被控制器持久化**，因此用完必须
        :meth:`close_protocol` 关闭。
        """
        if self._protocol_open:
            return self._ok(
                "Modbus 协议已处于打开状态（幂等）",
                self._fake_tag() + ["rm_set_modbus_mode 未重复调用"],
                protocol_open=True,
            )
        rc = self._port.set_modbus_mode(1, self._baud, self._timeout_100ms)
        if rc != 0:
            return self._rc_failure(
                "rm_set_modbus_mode", rc, "打开末端 Modbus-RTU 协议",
                self._fake_tag() + [
                    "port=1 baud=%d timeout=%d(单位:百毫秒)" % (
                        self._baud, self._timeout_100ms),
                ],
            )
        self._protocol_open = True
        return self._ok(
            "末端 Modbus-RTU 协议已打开",
            self._fake_tag() + [
                "rm_set_modbus_mode(port=1, baud=%d, timeout=%d[单位:百毫秒])" % (
                    self._baud, self._timeout_100ms),
            ],
            protocol_open=True,
            port=1,
            baud=self._baud,
            timeout_100ms=self._timeout_100ms,
        )

    def close_protocol(self) -> Dict[str, Any]:
        """关闭末端 Modbus-RTU 协议（``rm_close_modbus_mode``，**幂等**）。"""
        if not self._protocol_open:
            return self._ok(
                "Modbus 协议本就未打开（关闭幂等，未调用 SDK）",
                self._fake_tag() + ["rm_close_modbus_mode 未调用"],
                protocol_open=False,
            )
        rc = self._port.close_modbus_mode(1)
        self._protocol_open = False
        if rc != 0:
            return self._rc_failure(
                "rm_close_modbus_mode", rc, "关闭末端 Modbus-RTU 协议",
                self._fake_tag(),
            )
        return self._ok(
            "末端 Modbus-RTU 协议已关闭（工具端 RS485 已退出 RTU 主站态）",
            self._fake_tag() + ["rm_close_modbus_mode(port=1)"],
            protocol_open=False,
        )

    def with_protocol(self, fn: Callable[["GripperAdapter"], Dict[str, Any]]) -> Dict[str, Any]:
        """在「开协议 → 干活 → 关/恢复」的生命周期里执行 ``fn``。

        * 进入前协议未开 → 本函数打开；
        * 退出时**成功路径**：本函数打开的就关闭（恢复到进入前状态）；
        * 退出时**失败路径**（含抛异常）：协议若处于打开状态**一律关闭**
          （厂商头文件：``rm_set_modbus_mode`` 的配置会被控制器持久化，
          失败时宁可关闭，也不把工具端 RS485 留在 RTU 主站态）；
        * ``fn`` 抛异常 → 关闭协议后把异常转成 ``error`` 结果（**不抛裸异常**）。
        """
        opened_here = False
        if not self._protocol_open:
            opened = self.open_protocol()
            if opened.get("status") != errors.STATUS_OK:
                return opened
            opened_here = True
        try:
            result = fn(self)
        except Exception as exc:  # noqa: BLE001 - 统一转 error 结果
            self.close_protocol()
            return errors.error(
                "夹爪协议操作抛出异常: %s: %s" % (type(exc).__name__, exc),
                "修复上述异常后重跑同一条命令；若为通讯类异常，先检查夹爪接线与波特率",
                self._fake_tag() + [repr(exc), "失败路径已关闭 Modbus 协议"],
            )
        failed = isinstance(result, dict) and result.get("status") != errors.STATUS_OK
        if failed or opened_here:
            closed = self.close_protocol()
            if failed and isinstance(result, dict):
                result = dict(result)
                result["evidence"] = list(result.get("evidence", [])) + [
                    "失败路径已恢复协议状态：%s" % closed.get("reason", ""),
                ]
        return result

    # -- 寄存器读写 --------------------------------------------------------
    def _guard_write(self, reg: int, base: Optional[int] = None) -> Optional[Dict[str, Any]]:
        """写保护检查：红线寄存器 / 只读寄存器 → blocked（**0 次 SDK 调用**）。

        两道闸门：

        1. **手册地址级**：目标手册寄存器是 0x0008（清错误码）→ 拒绝；
        2. **线上地址级**：写出的**线上地址**恰好是 0x0008 → 同样拒绝
           （地址基数未定时，基数猜错可能把「线上 0x0008」当成别的寄存器写下去；
           这一道闸门宁可误伤，也不冒「误清错误码」的风险）。
        """
        reason = FORBIDDEN_WRITE_REGISTERS.get(reg)
        if reason is not None:
            return errors.blocked(
                "拒绝写寄存器 0x%04X（%s）：%s" % (reg, register_name(reg), reason),
                "改用人工复位流程（停机断电重启夹爪）后重跑；本项目不提供自动清错误码动作",
                self._fake_tag() + [
                    "禁止写寄存器: 0x%04X（%s）" % (reg, register_name(reg)),
                    "SDK 写调用次数: 0",
                ],
                register=reg,
                sdk_calls=0,
            )
        use_base = self._base if base is None else int(base)
        if use_base in (0, 1):
            wire = register_address(reg, use_base)
            if wire == REG_CLEAR_ERROR:
                return errors.blocked(
                    "拒绝写线上地址 0x%04X：它在 base=%d 下对应手册寄存器 0x%04X（%s），"
                    "而清错误码需重启设备，本项目禁止"
                    % (wire, use_base, reg, register_name(reg)),
                    "确认寄存器地址基数（base=0/1）后重跑；"
                    "先用 probe 确认基数，本项目不提供自动清错误码动作",
                    self._fake_tag() + [
                        "线上地址 0x%04X 被红线拦截（base=%d）" % (wire, use_base),
                        "SDK 写调用次数: 0",
                    ],
                    register=reg,
                    sdk_calls=0,
                )
        if reg in READ_ONLY_REGISTERS:
            return errors.blocked(
                "拒绝写只读寄存器 0x%04X（%s）" % (reg, register_name(reg)),
                "只读寄存器只能回读；如需改变状态请使用厂商提供的动作接口",
                self._fake_tag() + [
                    "只读寄存器: 0x%04X（%s）" % (reg, register_name(reg)),
                    "SDK 写调用次数: 0",
                ],
                register=reg,
                sdk_calls=0,
            )
        return None

    def read_register(
        self,
        reg: int,
        function: str = FUNCTION_HOLDING,
        base: Optional[int] = None,
    ) -> Dict[str, Any]:
        """读一个寄存器（holding=FC03 / input=FC04）。

        :param reg: 手册地址（:data:`REG_*`）
        :param function: ``holding``（FC03）或 ``input``（FC04）
        :param base: 地址基数（默认用适配器当前基数）
        """
        if function not in FUNCTIONS:
            return errors.error(
                "未知功能码: %r（只允许 %s）" % (function, list(FUNCTIONS)),
                "用 holding（FC03）或 input（FC04）重跑",
                self._fake_tag(),
            )
        use_base = self._base if base is None else int(base)
        if use_base not in (0, 1):
            return errors.error(
                "非法地址基数: %r（只允许 0 或 1）" % (use_base,),
                "改用 base=0 或 base=1 重跑",
                self._fake_tag(),
            )
        address = register_address(reg, use_base)
        reader = (
            self._port.read_holding_register if function == FUNCTION_HOLDING
            else self._port.read_input_register
        )
        rc, value = reader(1, address, self._device_id)
        if rc != 0:
            return self._rc_failure(
                "rm_read_%s_registers" % function, rc,
                "读寄存器 0x%04X（%s）" % (reg, register_name(reg)),
                self._fake_tag() + [
                    "port=1 address=%d(线上) device=%d base=%d function=%s" % (
                        address, self._device_id, use_base, function),
                ],
            )
        return self._ok(
            "读到寄存器 0x%04X（%s）= %d" % (reg, register_name(reg), value),
            self._fake_tag() + [
                "port=1 address=%d(线上) device=%d base=%d function=%s" % (
                    address, self._device_id, use_base, function),
            ],
            register=reg,
            value=int(value),
            address=address,
            base=use_base,
            function=function,
        )

    def read_registers(
        self,
        regs: Sequence[int],
        function: str = FUNCTION_HOLDING,
        base: Optional[int] = None,
    ) -> Dict[str, Any]:
        """读一组寄存器（逐个单读——厂商头文件：一次只能读 1 个）。

        任一寄存器读失败 → 整体 ``error``，并写明**第一个失败**的寄存器。
        """
        values: Dict[int, int] = {}
        evidence: List[Any] = list(self._fake_tag())
        for reg in regs:
            res = self.read_register(reg, function=function, base=base)
            if res.get("status") != errors.STATUS_OK:
                res = dict(res)
                res["evidence"] = list(res.get("evidence", [])) + evidence
                return res
            values[int(reg)] = int(res["value"])
            evidence.append("0x%04X=%d" % (reg, res["value"]))
        return self._ok(
            "读到 %d 个寄存器" % len(values),
            evidence,
            values=values,
        )

    def write_register(self, reg: int, value: int, base: Optional[int] = None) -> Dict[str, Any]:
        """写一个寄存器（``rm_write_single_register``）。

        **红线**：写 0x0008 → blocked 且**不产生任何 SDK 写调用**。
        """
        guard = self._guard_write(reg, base)
        if guard is not None:
            return guard
        use_base = self._base if base is None else int(base)
        if use_base not in (0, 1):
            return errors.error(
                "非法地址基数: %r（只允许 0 或 1）" % (use_base,),
                "改用 base=0 或 base=1 重跑",
                self._fake_tag(),
            )
        address = register_address(reg, use_base)
        rc = self._port.write_single_register(1, address, self._device_id, int(value))
        if rc != 0:
            return self._rc_failure(
                "rm_write_single_register", rc,
                "写寄存器 0x%04X（%s）" % (reg, register_name(reg)),
                self._fake_tag() + [
                    "port=1 address=%d(线上) device=%d base=%d data=%d" % (
                        address, self._device_id, use_base, int(value)),
                ],
            )
        return self._ok(
            "已写寄存器 0x%04X（%s）= %d" % (reg, register_name(reg), int(value)),
            self._fake_tag() + [
                "port=1 address=%d(线上) device=%d base=%d data=%d" % (
                    address, self._device_id, use_base, int(value)),
            ],
            register=reg,
            value=int(value),
            address=address,
            base=use_base,
        )

    def write_registers(
        self,
        reg: int,
        values: Sequence[int],
        base: Optional[int] = None,
    ) -> Dict[str, Any]:
        """写一组连续寄存器（``rm_write_registers``，一次最多 10 个）。"""
        if not values:
            return errors.error(
                "写入值列表为空",
                "给出至少一个待写值后重跑",
                self._fake_tag(),
            )
        if len(values) > 10:
            return errors.error(
                "一次最多写 10 个寄存器（收到 %d 个）" % len(values),
                "分批写入（每批 ≤ 10 个）后重跑",
                self._fake_tag(),
            )
        for offset in range(len(values)):
            guard = self._guard_write(int(reg) + offset, base)
            if guard is not None:
                return guard
        use_base = self._base if base is None else int(base)
        address = register_address(reg, use_base)
        rc = self._port.write_registers(1, address, self._device_id, values)
        if rc != 0:
            return self._rc_failure(
                "rm_write_registers", rc,
                "写寄存器 0x%04X 起的 %d 个寄存器" % (reg, len(values)),
                self._fake_tag() + ["port=1 address=%d(线上) num=%d" % (address, len(values))],
            )
        return self._ok(
            "已写 %d 个寄存器（起始 0x%04X）" % (len(values), reg),
            self._fake_tag() + [
                "port=1 address=%d(线上) device=%d num=%d values=%s" % (
                    address, self._device_id, len(values), list(values)),
            ],
            register=reg,
            values=[int(v) for v in values],
            address=address,
            base=use_base,
        )

    # -- 官方夹爪快路径 ----------------------------------------------------
    def gripper_route(self, min_limit: int, max_limit: int) -> Dict[str, Any]:
        """设置夹爪行程（``rm_set_gripper_route``）。"""
        rc = self._port.set_gripper_route(int(min_limit), int(max_limit))
        if rc != 0:
            return self._rc_failure(
                "rm_set_gripper_route", rc, "设置夹爪行程",
                self._fake_tag() + ["min=%d max=%d" % (min_limit, max_limit)],
            )
        return self._ok(
            "夹爪行程已设置（%d–%d）" % (min_limit, max_limit),
            self._fake_tag() + ["rm_set_gripper_route(min=%d, max=%d)" % (min_limit, max_limit)],
        )

    def gripper_release(self, speed: int, block: bool = True, timeout: int = 0) -> Dict[str, Any]:
        """松开夹爪（``rm_set_gripper_release``）。"""
        rc = self._port.set_gripper_release(int(speed), bool(block), int(timeout))
        if rc != 0:
            return self._rc_failure(
                "rm_set_gripper_release", rc, "松开夹爪",
                self._fake_tag() + ["speed=%d block=%s timeout=%d" % (speed, bool(block), timeout)],
            )
        return self._ok(
            "夹爪松开指令已下发",
            self._fake_tag() + ["rm_set_gripper_release(speed=%d, block=%s, timeout=%d)"
                                % (speed, bool(block), timeout)],
        )

    def gripper_pick(self, speed: int, force: int, block: bool = True, timeout: int = 0) -> Dict[str, Any]:
        """夹取（``rm_set_gripper_pick``）。"""
        rc = self._port.set_gripper_pick(int(speed), int(force), bool(block), int(timeout))
        if rc != 0:
            return self._rc_failure(
                "rm_set_gripper_pick", rc, "夹爪夹取",
                self._fake_tag() + [
                    "speed=%d force=%d block=%s timeout=%d" % (speed, force, bool(block), timeout),
                ],
            )
        return self._ok(
            "夹爪夹取指令已下发",
            self._fake_tag() + [
                "rm_set_gripper_pick(speed=%d, force=%d, block=%s, timeout=%d)"
                % (speed, force, bool(block), timeout),
            ],
        )

    def gripper_pick_on(self, speed: int, force: int, block: bool = True, timeout: int = 0) -> Dict[str, Any]:
        """持续夹取（``rm_set_gripper_pick_on``）。"""
        rc = self._port.set_gripper_pick_on(int(speed), int(force), bool(block), int(timeout))
        if rc != 0:
            return self._rc_failure(
                "rm_set_gripper_pick_on", rc, "夹爪持续夹取",
                self._fake_tag() + [
                    "speed=%d force=%d block=%s timeout=%d" % (speed, force, bool(block), timeout),
                ],
            )
        return self._ok(
            "夹爪持续夹取指令已下发",
            self._fake_tag() + [
                "rm_set_gripper_pick_on(speed=%d, force=%d, block=%s, timeout=%d)"
                % (speed, force, bool(block), timeout),
            ],
        )

    def gripper_position(self, position: int, block: bool = True, timeout: int = 0) -> Dict[str, Any]:
        """设定开口度（``rm_set_gripper_position``）。"""
        rc = self._port.set_gripper_position(int(position), bool(block), int(timeout))
        if rc != 0:
            return self._rc_failure(
                "rm_set_gripper_position", rc, "设定夹爪开口度",
                self._fake_tag() + ["position=%d block=%s timeout=%d"
                                    % (position, bool(block), timeout)],
            )
        return self._ok(
            "夹爪开口度指令已下发（%d）" % position,
            self._fake_tag() + ["rm_set_gripper_position(position=%d, block=%s, timeout=%d)"
                                % (position, bool(block), timeout)],
        )

    # -- RM Plus（只做协议开关区分） ----------------------------------------
    def rm_plus_mode(self) -> Dict[str, Any]:
        """回读 RM Plus 协议模式（``rm_get_rm_plus_mode``）。

        返回的 ``protocol_state``：

        * ``disable`` —— 协议**未开**（mode=0）；
        * ``online`` —— 协议已开且回读成功；
        * ``offline`` —— 协议**已开但设备无响应**（回读失败，且已知协议处于开启态）；
        * ``unknown`` —— 回读失败且协议开关状态未知（无法区分 disable / offline）。

        「disable vs offline」两态的区分正是本接口存在的唯一理由
        （RM Plus 只作协议开关的真值，**不作夹爪判定真值**）。
        """
        rc, mode = self._port.get_rm_plus_mode()
        if rc != 0:
            known = self._rm_plus_mode_last
            state = "offline" if known not in (None, 0) else "unknown"
            res = self._rc_failure(
                "rm_get_rm_plus_mode", rc, "回读 RM Plus 协议模式",
                self._fake_tag() + [
                    "已知 RM Plus 模式 = %s（据此判定协议 %s）"
                    % (known if known is not None else "未知", state),
                ],
            )
            res["protocol_state"] = state
            return res
        self._rm_plus_mode_last = int(mode)
        state = "disable" if int(mode) == 0 else "online"
        return self._ok(
            "RM Plus 协议模式 = %s（%d）" % (state, int(mode)),
            self._fake_tag() + ["rm_get_rm_plus_mode → %d" % int(mode)],
            protocol_state=state,
            mode=int(mode),
        )

    def set_rm_plus_mode(self, mode: int) -> Dict[str, Any]:
        """设置 RM Plus 协议模式（``rm_set_rm_plus_mode``，0 = 禁用）。"""
        rc = self._port.set_rm_plus_mode(int(mode))
        if rc != 0:
            return self._rc_failure(
                "rm_set_rm_plus_mode", rc, "设置 RM Plus 协议模式",
                self._fake_tag() + ["mode=%d" % int(mode)],
            )
        self._rm_plus_mode_last = int(mode)
        return self._ok(
            "RM Plus 协议模式已设置为 %d" % int(mode),
            self._fake_tag() + ["rm_set_rm_plus_mode(mode=%d)" % int(mode)],
            mode=int(mode),
        )

    # -- 工具端供电（只回读） ----------------------------------------------
    def read_tool_voltage(self) -> Dict[str, Any]:
        """回读工具端供电档位（``rm_get_tool_voltage``，**不写**）。

        夹爪额定 1.9 A > 法兰供电上限 1.5 A → 夹爪动力必须外部 24 V + 与法兰共地；
        本函数**只回读**，不改变供电状态。
        """
        rc, voltage = self._port.get_tool_voltage()
        if rc != 0:
            return self._rc_failure(
                "rm_get_tool_voltage", rc, "回读工具端供电档位",
                self._fake_tag(),
            )
        return self._ok(
            "工具端供电档位 = %d（0=0V 1=5V 2=12V 3=24V；只回读，未写入）" % int(voltage),
            self._fake_tag() + ["rm_get_tool_voltage → %d" % int(voltage)],
            voltage_type=int(voltage),
            written=False,
        )

    def set_tool_voltage(self, voltage_type: int) -> Dict[str, Any]:
        """设置工具端供电（``rm_set_tool_voltage``）——**默认不调用**。

        仅在人工明确要求时使用（本项目夹爪动力走外部 24 V，通常**不需要**开法兰供电）。
        """
        rc = self._port.set_tool_voltage(int(voltage_type))
        if rc != 0:
            return self._rc_failure(
                "rm_set_tool_voltage", rc, "设置工具端供电",
                self._fake_tag() + ["voltage_type=%d" % int(voltage_type)],
            )
        return self._ok(
            "工具端供电档位已设置为 %d" % int(voltage_type),
            self._fake_tag() + ["rm_set_tool_voltage(voltage_type=%d)" % int(voltage_type)],
            voltage_type=int(voltage_type),
            written=True,
        )

    # -- 状态归一化 --------------------------------------------------------
    def _raw_modbus(self, base: Optional[int] = None) -> Dict[str, Any]:
        """读一组关键寄存器 → 归一化原始字典（Modbus 路径）。"""
        regs = [
            REG_ERROR, REG_STATE, REG_ACTUAL_POS, REG_ACTUAL_FORCE,
            REG_ACTUAL_TEMP, REG_ACTUAL_CURRENT,
        ]
        res = self.read_registers(regs, function=FUNCTION_HOLDING, base=base)
        if res.get("status") != errors.STATUS_OK:
            return {"__error__": res}
        values = res["values"]
        return {
            "mode": values.get(REG_STATE),
            "position_raw": values.get(REG_ACTUAL_POS),
            "force_raw": values.get(REG_ACTUAL_FORCE),
            "temp_c": values.get(REG_ACTUAL_TEMP),
            "current_mA": values.get(REG_ACTUAL_CURRENT),
            "error": values.get(REG_ERROR, 0),
            "online": True,
            "protocol_state": "online",
        }

    def read_state(self, base: Optional[int] = None) -> Dict[str, Any]:
        """读一次夹爪状态 → 归一化成 :class:`GripperState`。

        * ``modbus`` 路径：读 6 个关键寄存器（错误码 / 状态码 / 位置 / 力度 / 温度 / 电流）；
        * ``gripper_api`` 路径：``rm_get_gripper_state``（SDK 结构体，mode 语义与手册不同）；
        * ``rm_plus`` 路径：**不做判定**（只读协议开关），返回 ``blocked``。
        """
        if self._path == PATH_RM_PLUS:
            return errors.blocked(
                "RM Plus 路径没有可用的夹爪状态读取接口（白名单只有协议开关）",
                "改用 modbus 主路径或 gripper_api 路径读取夹爪状态",
                self._fake_tag() + ["RM Plus 仅用于区分 disable / offline，不作判定真值"],
            )
        if self._path == PATH_GRIPPER_API:
            rc, raw = self._port.get_gripper_state()
            if rc != 0 or raw is None:
                return self._rc_failure(
                    "rm_get_gripper_state", rc if rc is not None else RC_RECV_TIMEOUT,
                    "读夹爪状态",
                    self._fake_tag() + ["注意：厂商头文件要求先控制夹爪，状态才会更新"],
                )
            normalized = {
                "mode": raw.get("mode"),
                "position_raw": raw.get("actpos"),
                "force_raw": raw.get("current_force"),
                "temp_c": raw.get("temperature"),
                "current_mA": None,
                "error": raw.get("error", 0),
                "online": bool(raw.get("status", 0) == 1),
                "protocol_state": "online" if raw.get("status", 0) == 1 else "offline",
            }
        else:
            raw = self._raw_modbus(base=base)
            if "__error__" in raw:
                res = dict(raw["__error__"])
                res["evidence"] = list(res.get("evidence", [])) + self._fake_tag()
                return res
            normalized = raw

        state = normalize_state(
            self._path,
            normalized,
            position_mm_per_unit=self._position_mm_per_unit,
            force_scale=self._force_scale,
        )
        if self.is_fake:
            state.raw["source"] = "fake"
        return self._ok(
            "夹爪状态已归一化（path=%s）" % self._path,
            self._fake_tag() + ["raw=%s" % normalized],
            state=state,
            state_dict=state.to_dict(),
            mode_source="fake" if self.is_fake else "sdk",
        )

    # -- 探针 --------------------------------------------------------------
    def _read_wire(self, address: int, function: str) -> Tuple[int, Optional[int]]:
        """按**线上地址**直接读一个寄存器（不做 manual→wire 换算）。

        探针用：先按线上地址块扫描原始数据，再对两种基数假设分别反推手册寄存器。
        """
        reader = (
            self._port.read_holding_register if function == FUNCTION_HOLDING
            else self._port.read_input_register
        )
        return reader(1, int(address), self._device_id)

    def _scan_wire_block(self, function: str) -> Dict[int, int]:
        """扫描线上地址块 → ``{线上地址: 值}``（读失败即返回已读到的部分）。"""
        start, end = PROBE_ADDRESS_RANGE
        values: Dict[int, int] = {}
        for address in range(start, end + 1):
            rc, value = self._read_wire(address, function)
            if rc != 0 or value is None:
                continue
            values[address] = int(value)
        return values

    def _plausible_hits(self, wire_values: Dict[int, int], hypothesis: int) -> int:
        """按物理合理性窗口给一个基数假设打分（命中项数）。

        ``hypothesis`` 是**假设的设备基数**：设备基数 H 意味着手册寄存器 M 位于
        线上地址 ``M - (1 - H)``（H=1 → 线上地址 = 手册地址；H=0 → 线上地址 = 手册地址 − 1）。
        """
        hits = 0
        checks = (
            (REG_ACTUAL_POS, "position"),
            (REG_ACTUAL_TEMP, "temperature"),
            (REG_ACTUAL_CURRENT, "current"),
        )
        for reg, key in checks:
            wire = reg - (1 - hypothesis)
            value = wire_values.get(wire)
            if value is None:
                continue
            low, high = PROBE_WINDOWS[key]
            if low <= float(value) <= high:
                hits += 1
        return hits

    def probe(self) -> Dict[str, Any]:
        """一次性探针：协议状态 / 波特率 / ID / 寄存器基数 / 供电档位 / 代际线索。

        **显式标注哪些是未实测的假设**（``assumptions`` 列表）。
        """
        evidence: List[Any] = list(self._fake_tag())
        assumptions: List[str] = [
            "寄存器地址基数由启发式（物理合理性窗口）推断，未在真机确认",
            "只读寄存器归属 FC03 还是 FC04 未定，两条都试了",
            "波特率档位 → 实际波特率的映射取自手册（0=9600 3=57600 4=115200 5=460800），未实测",
            "控制器代际线索仅来自 rm_set_modbus_mode 的返回码，未经型号核对",
        ]
        out: Dict[str, Any] = {"assumptions": assumptions}

        # 1) RM Plus 协议开关（disable vs 已开）
        plus = self.rm_plus_mode()
        out["rm_plus"] = {
            "status": plus.get("status"),
            "protocol_state": plus.get("protocol_state"),
            "mode": plus.get("mode"),
            "reason": plus.get("reason"),
        }
        evidence.extend(["rm_plus: %s" % plus.get("reason", "")])

        # 2) Modbus 协议 + 代际线索
        opened = self.open_protocol()
        out["modbus_open"] = {
            "status": opened.get("status"),
            "reason": opened.get("reason"),
            "rc": opened.get("rc"),
        }
        evidence.append("modbus: %s" % opened.get("reason", ""))
        generation = "unknown"
        if opened.get("status") == errors.STATUS_OK:
            generation = "supports_modbus_rtu（rm_set_modbus_mode 返回 0）"
        elif opened.get("rc") == RC_UNSUPPORTED:
            generation = "fourth_gen_unsupported（rm_set_modbus_mode 返回 -4）"
        out["controller_generation_hint"] = generation

        # 3) 寄存器块扫描（线上地址块 × 两种功能码），再对两种基数假设打分
        scan: Dict[str, Any] = {}
        base_scores: Dict[int, int] = {}
        if opened.get("status") == errors.STATUS_OK:
            for function in FUNCTIONS:
                wire_values = self._scan_wire_block(function)
                per_hypothesis: Dict[str, Any] = {}
                for hypothesis in (1, 0):
                    hits = self._plausible_hits(wire_values, hypothesis)
                    base_scores[hypothesis] = base_scores.get(hypothesis, 0) + hits
                    per_hypothesis["base%d" % hypothesis] = {"hits": hits}
                key = "wire_%s" % function
                scan[key] = {
                    "status": "ok" if wire_values else "error",
                    "read_count": len(wire_values),
                    "hypotheses": per_hypothesis,
                    "values": wire_values,
                }
                evidence.append(
                    "%s: 读到 %d 个线上地址；base1 命中 %d 项，base0 命中 %d 项"
                    % (key, len(wire_values),
                       per_hypothesis["base1"]["hits"], per_hypothesis["base0"]["hits"])
                )
        out["register_scan"] = scan
        out["base_scores"] = {str(k): v for k, v in base_scores.items()}
        detected = None
        if base_scores:
            best = max(base_scores.values())
            winners = [b for b, v in base_scores.items() if v == best]
            if best > 0 and len(winners) == 1:
                detected = winners[0]
        out["detected_base"] = detected
        out["detected_base_note"] = (
            "启发式推断（未实测确认）" if detected is not None
            else "无法判定（两种基数得分相同或全不命中）"
        )

        # 4) ID / 波特率（按推断出的基数读 0x0003 / 0x0004）
        id_res = self.read_register(REG_DEVICE_ID, base=detected)
        baud_res = self.read_register(REG_BAUD, base=detected)
        out["device_id"] = id_res.get("value") if id_res.get("status") == errors.STATUS_OK else None
        out["baud_code"] = baud_res.get("value") if baud_res.get("status") == errors.STATUS_OK else None
        evidence.append("device_id=%s baud_code=%s" % (out["device_id"], out["baud_code"]))

        # 5) 工具端供电（只回读）
        voltage = self.read_tool_voltage()
        out["tool_voltage"] = {
            "status": voltage.get("status"),
            "voltage_type": voltage.get("voltage_type"),
            "reason": voltage.get("reason"),
        }
        evidence.append("tool_voltage: %s" % voltage.get("reason", ""))

        # 6) 关闭协议（恢复工具端 RS485）
        closed = self.close_protocol()
        out["modbus_close"] = {"status": closed.get("status"), "reason": closed.get("reason")}

        out["status"] = errors.STATUS_OK
        out["reason"] = "夹爪探针完成（部分结论为启发式推断，见 assumptions）"
        out["next_action"] = ""
        out["evidence"] = evidence
        return out
