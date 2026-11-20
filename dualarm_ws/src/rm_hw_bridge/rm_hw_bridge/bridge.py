"""rm_hw_bridge 连接层 + 状态快照（R004 全字段）。

本模块是**硬件适配层的高层入口**，只依赖白名单符号（T2 契约）与 ABI 规格（T1 真相）。

调用链::

    connect(ip, port, udp_port, enable_realtime, lib)
      → rm_init(RM_TRIPLE_MODE_E)          # 三线程模式，否则收不到 UDP 主动上报
      → rm_create_robot_arm(ip, port)       # **失败仍返回非空指针**
      → 读 handle.id                        # id < 0 即失败（不可达时实测也是非空指针！）
      → rm_set_realtime_push(handle, cfg)   # cycle 必须是 5 的倍数
      → rm_realtime_arm_state_call_back(cb) # **进程级全局注册**

    link.snapshot()
      → rm_get_joint_degree / rm_get_arm_all_state / rm_get_current_arm_state
      → rm_get_joint_err_flag / rm_get_arm_current_trajectory
      → 组装 R004 全字段快照

**诚实降级原则（与 S01 同构）**：

* 缺 SDK → ``status="blocked"``，给出搜索路径 + 可粘贴修复动作；
* 句柄无效 / 任何 SDK 调用返回非 0 → ``status="error"``，写明**接口名 + 返回码**
  + 可粘贴下一步；
* **任何失败路径都不产生任何伪造数值**（失败结果里没有关节角、没有状态、没有"已连通"）。

**全局回调的限制（重要）**：``rm_realtime_arm_state_call_back`` 是**进程级全局注册**，
同一进程**只能接一路臂**；双臂同进程不可行（属 R014/R015，Out of Scope）。
"""
from __future__ import annotations

import ctypes
import math
import os
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

import yaml

from rm_hw_bridge import errors, limits as limits_mod, loader, paths
from rm_hw_bridge.generated import rm_api2_ctypes as gen
from rm_hw_bridge.snapshot import (
    ARM_DOF,
    BRAKE_HARD_JOINTS,
    BRAKE_SOFT_JOINTS,
    TRAJECTORY_ENUM_NAMES,
    UDP_STATUS_ENUM_NAMES,
    StateSnapshot,
    controller_pose_scope,
)

__all__ = [
    "ArmLink",
    "COLLISION_STAGE_MAX",
    "COLLISION_STAGE_MIN",
    "DEG2RAD",
    "DEFAULT_SDK_TIMEOUT_S",
    "JOINT_INDEX_MAX",
    "JOINT_INDEX_MIN",
    "MAX_RETAINED_JOINT_BUFFERS",
    "RAD2DEG",
    "RADIO_RANGE_CURVE_FIT",
    "RADIO_RANGE_FILTER",
    "REALTIME_CYCLE_MULTIPLE_MS",
    "TRAJECTORY_MODES",
    "TRAJECTORY_MODE_CURVE_FIT",
    "TRAJECTORY_MODE_FILTER",
    "TRAJECTORY_MODE_PASSTHROUGH",
    "connect",
    "realtime_state",
    "reset_realtime_state",
]

#: UDP 主动上报周期必须是 5 ms 的倍数（头文件注释 + network_plan 基线）
REALTIME_CYCLE_MULTIPLE_MS = 5

#: 默认 TCP 端口
DEFAULT_TCP_PORT = 8080

# ---------------------------------------------------------------------------
# 单位换算**单点收敛**：本模块内**只允许**这两处换算常量
# （与 rmctl/fk_cross.py 的约定一致：内部一律 SI，调用 SDK 前再换算成度制）。
# ---------------------------------------------------------------------------
#: 度 → 弧度
DEG2RAD = math.pi / 180.0
#: 弧度 → 度
RAD2DEG = 180.0 / math.pi

#: 碰撞检测等级范围（厂商头文件 rm_interface.h：0=不检测，8=最灵敏）
COLLISION_STAGE_MIN = 0
COLLISION_STAGE_MAX = 8

#: 窄接口 ``set_drive_max_speed`` 的**公开关节序号**范围（**0 基**：0..6）
JOINT_INDEX_MIN = 0
JOINT_INDEX_MAX = ARM_DOF - 1

#: SDK 通信超时的**默认秒数**（缺 ``workcell/rm_workcell.yaml::safety.sdk_timeout_s``
#: 或该键非法时使用，并在 evidence 里写明「使用了默认值」，**不静默**）
DEFAULT_SDK_TIMEOUT_S = 1.0

#: ``rm_set_timeout`` 的 **SDK 单位换算**：厂商头文件
#: ``rm_interface.h``（``rm_set_timeout``）写明「多数接口默认超时时间为 500ms，
#: 单位 **ms**」→ 本模块对外用**秒**，调用 SDK 前**在此处**换算成毫秒。
#:
#: **注意**：``rm_hw_bridge/abi_spec.yaml`` 里该符号的 doc 串写的是「秒」，
#: 与厂商头文件矛盾；按仓库的**上游真值链**（vendor 头文件 → abi_spec），
#: 以厂商头文件为准（500 这个默认值只在「毫秒」解释下合理）。
SDK_TIMEOUT_MS_PER_S = 1000
#: 下发给 SDK 的毫秒数下限（避免 0 = 不超时）
SDK_TIMEOUT_MIN_MS = 1

# ---------------------------------------------------------------------------
# 关节角透传（rm_movej_canfd）语义常量
#
# **只用 rm_movej_canfd 透传关节角**：位姿透传（rm_movep_canfd / rm_movep_follow）
# 在厂商黑名单里——控制器会偷偷做逆解，违反 R003 与设计 §1.4。
# ---------------------------------------------------------------------------
#: trajectory_mode = 0：完全透传（原始数据直发关节）
TRAJECTORY_MODE_PASSTHROUGH = 0
#: trajectory_mode = 1：曲线拟合（radio 0..100）
TRAJECTORY_MODE_CURVE_FIT = 1
#: trajectory_mode = 2：滤波（radio 0..1000）
TRAJECTORY_MODE_FILTER = 2
#: 允许的 trajectory_mode 取值
TRAJECTORY_MODES = (
    TRAJECTORY_MODE_PASSTHROUGH,
    TRAJECTORY_MODE_CURVE_FIT,
    TRAJECTORY_MODE_FILTER,
)
#: 曲线拟合模式的 radio 允许范围（厂商头文件 rm_define.h）
RADIO_RANGE_CURVE_FIT = (0, 100)
#: 滤波模式的 radio 允许范围（厂商头文件 rm_define.h）
RADIO_RANGE_FILTER = (0, 1000)

#: 保留的关节角缓冲上限（**有界**：防止 10 ms 调度下缓冲无限堆积）
MAX_RETAINED_JOINT_BUFFERS = 64

#: 下发限值族 →（SDK 符号名, SDK 单位, SI→SDK 换算类型）
#: 换算类型："deg"（位置）、"deg_per_s"（速度）、"deg_per_s2"（加速度）
_LIMIT_DOWNLINK_FIELDS = (
    ("max_speed", "rm_set_joint_drive_max_speed", "deg_per_s"),
    ("max_acc", "rm_set_joint_drive_max_acc", "deg_per_s2"),
    ("min_pos", "rm_set_joint_drive_min_pos", "deg"),
    ("max_pos", "rm_set_joint_drive_max_pos", "deg"),
)

#: 各字段的 SI 单位 / SDK 单位（供 diff 输出与自检）
_LIMIT_UNITS = {
    "max_speed": ("rad/s", "deg/s"),
    "max_acc": ("rad/s^2", "deg/s^2"),
    "min_pos": ("rad", "deg"),
    "max_pos": ("rad", "deg"),
    "jerk": ("rad/s^3", "-"),
}


def _to_sdk_unit(value: float, kind: str) -> float:
    """SI → SDK 单位（**唯一的换算出口**）。"""
    if kind == "deg":
        return float(value) * RAD2DEG
    if kind == "deg_per_s":
        return float(value) * RAD2DEG
    if kind == "deg_per_s2":
        return float(value) * RAD2DEG
    return float(value)


def _joint_index(name: str) -> int:
    """关节名 → 关节序号（1..7）。"""
    digits = "".join(ch for ch in str(name) if ch.isdigit())
    return int(digits) if digits else 0


def _joint_order(values: Dict[str, Any]) -> List[str]:
    """按关节序号排序（joint1..joint7）。"""
    return sorted(values.keys(), key=_joint_index)


def print_diff(diff: List[Dict[str, Any]]) -> None:
    """打印逐关节限值 diff（当前值 vs 目标值，含单位）。

    只打印，不写文件；结构化数据同时在结果字典的 ``diff`` 字段里。
    """
    if not diff:
        print("[limits] 限值 diff 为空（无可下发项）")
        return
    print("[limits] 限值 diff（当前值 → 目标值，含单位）：")
    for item in diff:
        target = item.get("target")
        current = item.get("current")
        unit = item.get("target_unit")
        sdk = item.get("sdk_value")
        sdk_txt = "n/a" if sdk is None else "%.4f %s" % (float(sdk), item.get("sdk_unit"))
        print(
            "  %-7s %-9s %s → %s %s   (SDK: %s)"
            % (
                item.get("joint"),
                item.get("field"),
                "未知" if current is None else ("%.4f" % float(current)),
                "未知" if target is None else ("%.4f" % float(target)),
                unit,
                sdk_txt,
            )
        )


# --------------------------------------------------------------- 实时上报状态
def _copy_joint_array(values: Any, size: int = ARM_DOF) -> Optional[List[float]]:
    """把 SDK 关节数组**拷贝**成定长 float 列表；不合法 → ``None``。

    **不返回 0 冒充**：数量不对 / 含非数值 → ``None``（调用方据此报「暂无数据」）。
    """
    if values is None or isinstance(values, (str, bytes)):
        return None
    try:
        seq = list(values)
    except TypeError:
        return None
    if len(seq) != size:
        return None
    out: List[float] = []
    for item in seq:
        if isinstance(item, bool) or not isinstance(item, (int, float)):
            return None
        out.append(float(item))
    return out


def _copy_joint_arrays(joint_status: Any) -> Tuple[Optional[List[float]], Optional[List[float]]]:
    """从 ``rm_joint_status_t`` 拷贝出 (关节角°, 关节速度°/s)。

    ``joint_status`` 为 ``None`` 或字段缺失/不合法 → 两个 ``None``（无数据）。
    """
    if joint_status is None:
        return None, None
    position = _copy_joint_array(getattr(joint_status, "joint_position", None))
    speed = _copy_joint_array(getattr(joint_status, "joint_speed", None))
    return position, speed


class _RealtimeState:
    """进程级实时上报状态（``rm_realtime_arm_state_call_back`` 是全局注册）。

    **拷贝语义**：``push()`` 收到的 ``rm_joint_status_t`` 是 SDK **复用结构体**
    （下一次回调会原地覆写），因此本类在 push 时就把 ``joint_position``（°）
    与 ``joint_speed``（°/s）**拷贝**成 Python ``list``，只读访问器
    :meth:`latest_joint_state` 只返回拷贝——读到的快照不会撕裂。
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.frames = 0
        self.last_ts: Optional[float] = None
        self.arm_current_status: Optional[int] = None
        self.last_joint_status: Optional[Any] = None
        # 拷贝出来的关节字段（Python list，不保留 SDK 结构体引用）
        self._joint_position: Optional[List[float]] = None
        self._joint_speed: Optional[List[float]] = None

    def push(self, state: Any) -> None:
        with self._lock:
            self.frames += 1
            self.last_ts = time.time()
            self.arm_current_status = int(getattr(state, "arm_current_status", 0) or 0)
            self.last_joint_status = getattr(state, "joint_status", None)
            position, speed = _copy_joint_arrays(self.last_joint_status)
            self._joint_position = position
            self._joint_speed = speed

    def reset(self) -> None:
        """把全部可变字段复位到初始状态（**仅供测试隔离**）。

        复位项：``frames`` / ``last_ts`` / ``arm_current_status`` /
        ``last_joint_status`` 以及拷贝出来的关节字段
        （``_joint_position`` / ``_joint_speed``）。

        用途：``_RealtimeState`` 是**进程级单例**，跨包测试（例如
        ``rm_executor`` 的接线用例对真实单例 ``push()``）会把它留在脏状态，
        导致 ``rm_hw_bridge`` 自己的快照用例读到被污染的帧数。
        运行时**不调用**本方法——正常运行时状态由回调持续累积。
        """
        with self._lock:
            self.frames = 0
            self.last_ts = None
            self.arm_current_status = None
            self.last_joint_status = None
            self._joint_position = None
            self._joint_speed = None

    def snapshot(self, period_ms: Optional[int]) -> Dict[str, Any]:
        with self._lock:
            return {
                "frames": self.frames,
                "last_ts": self.last_ts,
                "period_ms": period_ms,
            }

    def latest_joint_state(self) -> Dict[str, Any]:
        """最新关节状态快照（**拷贝语义**，线程安全只读）。

        返回::

            {
              "joint_position_deg": [7] | None,   # 关节角（度）
              "joint_speed_deg_s":  [7] | None,   # 关节角速度（度/秒）
              "last_frame_ts": float | None,      # 最后一帧时间戳（秒）
              "frames": int,                      # 已收到的帧数
            }

        **无数据时关节字段为 ``None``，绝不返回 0 冒充**（0 会被误读成
        「关节在 0 度」）。返回值里的 list 是拷贝，改它不影响内部缓存。
        """
        with self._lock:
            return {
                "joint_position_deg": (
                    list(self._joint_position) if self._joint_position else None
                ),
                "joint_speed_deg_s": (
                    list(self._joint_speed) if self._joint_speed else None
                ),
                "last_frame_ts": self.last_ts,
                "frames": self.frames,
            }


#: 进程内唯一的实时上报状态（回调是全局注册，天然单例）
_REALTIME = _RealtimeState()
#: 已注册的回调对象（必须保持引用，否则被 GC 后 SDK 调用即崩）
_REALTIME_CB = None


def realtime_state() -> _RealtimeState:
    """进程级实时上报状态（只读访问）。"""
    return _REALTIME


def reset_realtime_state() -> None:
    """复位进程级实时上报状态（**测试隔离入口**，运行时不变行为）。

    ``_REALTIME`` 是进程级单例（SDK 回调全局注册），任何包在测试中对其
    ``push()`` 都会污染其它包的断言（例如 ``rm_executor`` 的接线用例会让
    ``rm_hw_bridge`` 的 ``frames == 0`` 断言失败）。

    本函数把单例复位成「从未收到过帧」的状态，供
    ``rm_hw_bridge/tests/conftest.py`` 的 autouse 夹具在每个用例前调用，
    使本包断言与 pytest 收集顺序、外部污染无关。
    """
    _REALTIME.reset()


# --------------------------------------------------------------------- 工具
def _handle_id(handle: Any) -> Optional[int]:
    """读句柄 id（真实指针在 ``handle.contents.id``；假 SDK 直接暴露 ``.id``）。"""
    return loader._handle_id(handle)


def _tcp_reachable(ip: str, port: int, timeout_s: float = 1.0) -> bool:
    """TCP 层探测：控制器**端口可达**吗？

    用途：把"控制器不可达"与"控制器可达但拒绝创建客户端连接（连接数达上限）"
    这两种**处理方式完全不同**的情况分开：

    * 端口不通 → 控制器离线 / 网线 / IP 错 → 现场处理；
    * 端口通但 ``rm_create_robot_arm`` 失败 → 控制器在线，只是**不接纳新客户端**
      （RM-API2 实测：同一控制器对客户端连接数有上限，达到后返回 NULL / id=-1）。
    """
    import socket as _socket
    try:
        with _socket.create_connection((str(ip), int(port)), timeout=float(timeout_s)):
            return True
    except OSError:
        return False


def _err_flag_to_list(err_flag: int) -> List[int]:
    """关节错误位标志 → 每关节错误码列表（bit i 对应关节 i+1）。"""
    return [(int(err_flag) >> i) & 1 for i in range(ARM_DOF)]


def _brake_to_list(brake_state: int) -> List[bool]:
    """抱闸状态位标志 → 每关节 ``True=抱闸未打开``（头文件：1 代表抱闸未打开）。"""
    return [bool((int(brake_state) >> i) & 1) for i in range(ARM_DOF)]


def _normalize_deg7(deg7: Any) -> Any:
    """关节角输入校验：7 个数字（单位**度**）。

    返回 ``(values, None)`` 或 ``(None, reason)``；``reason`` 非空即表示拒绝，
    且拒绝路径**必须 0 次 SDK 调用**。
    """
    if deg7 is None:
        return None, "关节角输入为 None"
    if isinstance(deg7, (str, bytes)):
        return None, "关节角输入必须是 7 个数字的序列（收到 %s）" % type(deg7).__name__
    try:
        seq = list(deg7)
    except TypeError:
        return None, "关节角输入必须是可迭代序列（收到 %s）" % type(deg7).__name__
    if len(seq) != ARM_DOF:
        return None, "关节角必须是 %d 个（收到 %d 个）" % (ARM_DOF, len(seq))
    values: List[float] = []
    for i, item in enumerate(seq):
        if isinstance(item, bool) or not isinstance(item, (int, float)):
            return None, "第 %d 个关节角不是数字（%r）" % (i + 1, item)
        values.append(float(item))
    return values, None


def _find_sdk(ws: Optional[str] = None, env: Optional[Dict[str, str]] = None):
    """找 SDK；找不到返回 ``(None, 候选路径列表)``。"""
    candidates = loader.sdk_candidates(ws=ws, env=env)
    path = loader.find_library(ws=ws, env=env)
    return path, candidates


# ------------------------------------------------------- 安全窄接口的公共助手
def _is_finite_number(value: Any) -> bool:
    """是否为有限数值（bool 不算数值）。"""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False
    number = float(value)
    return math.isfinite(number)


def _load_sdk_timeout_s(ws: Optional[str] = None) -> Tuple[float, List[str]]:
    """读取 ``workcell/rm_workcell.yaml::safety.sdk_timeout_s``（**唯一来源**）。

    缺文件 / 缺键 / 值非法 / 解析失败 → 使用默认 ``DEFAULT_SDK_TIMEOUT_S``
    （1.0 s），并**逐条**写进 evidence（**不静默**）。

    返回 ``(秒数, evidence 行)``。
    """
    cfg_path = os.path.join(paths.workspace_root(ws), "workcell", "rm_workcell.yaml")
    evidence: List[str] = ["workcell: %s" % cfg_path]
    try:
        with open(cfg_path, "r", encoding="utf-8") as fh:
            data = yaml.safe_load(fh)
    except FileNotFoundError:
        evidence.append(
            "缺少 %s → 使用默认 SDK 超时 %.1f s（不静默）" % (
                cfg_path, DEFAULT_SDK_TIMEOUT_S,
            )
        )
        return DEFAULT_SDK_TIMEOUT_S, evidence
    except Exception as exc:  # noqa: BLE001 - 读取异常统一降级为默认值
        evidence.append(
            "读取 %s 失败（%s）→ 使用默认 SDK 超时 %.1f s（不静默）" % (
                cfg_path, exc, DEFAULT_SDK_TIMEOUT_S,
            )
        )
        return DEFAULT_SDK_TIMEOUT_S, evidence

    section = data.get("safety") if isinstance(data, dict) else None
    raw = section.get("sdk_timeout_s") if isinstance(section, dict) else None
    if raw is None:
        evidence.append(
            "缺少 safety.sdk_timeout_s → 使用默认 SDK 超时 %.1f s（不静默）" % (
                DEFAULT_SDK_TIMEOUT_S,
            )
        )
        return DEFAULT_SDK_TIMEOUT_S, evidence
    if not _is_finite_number(raw) or float(raw) <= 0:
        evidence.append(
            "safety.sdk_timeout_s 非法（%r）→ 使用默认 SDK 超时 %.1f s（不静默）" % (
                raw, DEFAULT_SDK_TIMEOUT_S,
            )
        )
        return DEFAULT_SDK_TIMEOUT_S, evidence
    return float(raw), evidence


def _call_set_timeout(binding: Any, seconds: float) -> Dict[str, Any]:
    """调用 ``rm_set_timeout``（**support 5 内**；SDK 侧**全局**生效）。

    * ``rm_set_timeout`` 返回 ``void``（无返回码）→ 成功即认为已下发，
      **不伪读回值**；
    * **单位**：对外是**秒**（``workcell`` 的 ``safety.sdk_timeout_s``），
      SDK 侧是**毫秒**（厂商头文件：默认 500 ms）→ 在本函数里**一次性换算**
      （本模块唯一换算点）；
    * 符号缺失 / 抛异常 → ``status="error"`` + 可粘贴 next_action。
    """
    sym = "rm_set_timeout"
    fn = getattr(binding, sym, None)
    if fn is None:
        return errors.error(
            "契约清单缺少符号 %s，无法设置 SDK 超时" % sym,
            "确认所用 .so 为 RM-API2 1.1.7（`nm -D <so> | grep %s`），"
            "重新物化 SDK 后重跑" % sym,
            [],
            sdk_calls=0,
            timeout_s=float(seconds),
        )
    sdk_timeout_ms = max(
        SDK_TIMEOUT_MIN_MS,
        int(round(float(seconds) * SDK_TIMEOUT_MS_PER_S)),
    )
    try:
        fn(sdk_timeout_ms)
    except Exception as exc:  # noqa: BLE001 - SDK 调用异常统一转 error
        return errors.error(
            "%s 调用异常：%s" % (sym, exc),
            "确认 .so 与当前架构匹配（file <so>），修复后重跑",
            [repr(exc)],
            sdk_calls=1,
            timeout_s=float(seconds),
            sdk_timeout_ms=sdk_timeout_ms,
        )
    return errors.ok(
        "SDK 超时已下发：%d ms（= %.3f s，SDK 侧全局生效，不伪读回值）" % (
            sdk_timeout_ms, float(seconds),
        ),
        "",
        ["%s(timeout=%d ms)" % (sym, sdk_timeout_ms)],
        sdk_calls=1,
        timeout_s=float(seconds),
        sdk_timeout_ms=sdk_timeout_ms,
        scope="global",
    )


# --------------------------------------------------------------------- 连接
class ArmLink:
    """一次机械臂连接（句柄 + 实时上报状态）。

    ``status`` 属性给出连接结果（``ok`` / ``blocked`` / ``error``）；
    ``snapshot()`` 给出状态快照结果（同一套统一错误模型）。
    """

    def __init__(
        self,
        bridge: loader.RmBridge,
        handle: Any = None,
        status: str = errors.STATUS_OK,
        reason: str = "",
        next_action: str = "",
        evidence: Optional[List[Any]] = None,
        realtime_period_ms: Optional[int] = None,
        endpoint: Optional[Dict[str, Any]] = None,
        sdk_timeout_result: Optional[Dict[str, Any]] = None,
        realtime_target_ip: Optional[str] = None,
        udp_port: Optional[int] = None,
    ) -> None:
        self._bridge = bridge
        self._handle = handle
        self.status = status
        self.reason = reason
        self.next_action = next_action
        self.evidence = list(evidence or [])
        self.realtime_period_ms = realtime_period_ms
        self.realtime_target_ip = realtime_target_ip
        self.udp_port = udp_port
        self.endpoint = dict(endpoint or {})
        self._closed = False
        # 建连期 ``rm_set_timeout`` 的结果（成功/失败都留痕，**不静默**）
        self.sdk_timeout_result = dict(sdk_timeout_result or {})
        # 关节角缓冲的**显式持有**（防止 GC 提前回收）+ **有界**保留
        self._joint_buffers: List[Any] = []
        self._last_transmit: Optional[Dict[str, Any]] = None

    # -- 属性
    @property
    def handle(self) -> Any:
        return self._handle

    @property
    def connected(self) -> bool:
        return self.status == errors.STATUS_OK and self._handle is not None

    @property
    def bridge(self) -> loader.RmBridge:
        return self._bridge

    def to_dict(self) -> Dict[str, Any]:
        """JSON 安全的连接结果（不含 ctypes 对象）。"""
        out = {
            "status": self.status,
            "reason": self.reason,
            "next_action": self.next_action,
            "evidence": list(self.evidence),
        }
        if self.endpoint:
            out["endpoint"] = dict(self.endpoint)
        return out

    # -- 生命周期
    def close(self) -> None:
        """释放句柄（幂等）。"""
        if self._closed:
            return
        self._closed = True
        try:
            self._bridge.close()
        except Exception:  # pragma: no cover - 释放失败不影响主流程
            pass

    def __enter__(self) -> "ArmLink":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    # -- 快照
    def snapshot(self) -> Dict[str, Any]:
        """采集一次状态快照（R004 全字段）。

        任何一步 SDK 返回非 0 → ``status="error"`` + reason（写明**接口名与返回码**）
        + 可粘贴 next_action；**结果里不含任何数值**。
        """
        if not self.connected:
            return errors.result(
                self.status if self.status != errors.STATUS_OK else errors.STATUS_ERROR,
                self.reason or "尚未建立连接，无法采集状态快照",
                self.next_action,
                self.evidence,
            )

        binding = self._bridge.binding
        handle = self._handle
        evidence: List[str] = []

        # 1) 关节角（°）
        joints = (ctypes.c_float * ARM_DOF)()
        rc = int(binding.rm_get_joint_degree(handle, joints) or 0)
        if rc != 0:
            return errors.error(
                "rm_get_joint_degree 返回 %d（未连接到控制器 / 通信失败）" % rc,
                "确认控制器已上电且 %s:%s 可达（ping + TCP 端口检查），"
                "并确认同一 ip:port 没有被其它客户端占用后重跑" % (
                    self.endpoint.get("ip", "?"), self.endpoint.get("port", "?"),
                ),
                evidence + ["rm_get_joint_degree rc=%d" % rc],
            )

        # 2) 关节电流 / 温度 / 电压 / 使能位 / 错误码
        all_state = gen.rm_arm_all_state_t()
        rc = int(binding.rm_get_arm_all_state(handle, ctypes.pointer(all_state)) or 0)
        if rc != 0:
            return errors.error(
                "rm_get_arm_all_state 返回 %d（未连接到控制器 / 通信失败）" % rc,
                "确认控制器在线后重跑 `rmctl connect --arm %s --json`" % (
                    self.endpoint.get("arm", "left"),
                ),
                evidence + ["rm_get_arm_all_state rc=%d" % rc],
            )

        # 3) 当前臂状态（位姿 + 错误码列表）
        cur_state = gen.rm_current_arm_state_t()
        rc = int(binding.rm_get_current_arm_state(handle, ctypes.pointer(cur_state)) or 0)
        if rc != 0:
            return errors.error(
                "rm_get_current_arm_state 返回 %d（未连接到控制器 / 通信失败）" % rc,
                "确认控制器在线后重跑 `rmctl connect --arm %s --json`" % (
                    self.endpoint.get("arm", "left"),
                ),
                evidence + ["rm_get_current_arm_state rc=%d" % rc],
            )

        # 4) 关节错误位标志 + 抱闸状态
        err_flag = ctypes.c_uint16(0)
        brake_state = ctypes.c_uint16(0)
        rc = int(
            binding.rm_get_joint_err_flag(
                handle, ctypes.pointer(err_flag), ctypes.pointer(brake_state)
            ) or 0
        )
        if rc != 0:
            return errors.error(
                "rm_get_joint_err_flag 返回 %d（未连接到控制器 / 通信失败）" % rc,
                "确认控制器在线后重跑 `rmctl connect --arm %s --json`" % (
                    self.endpoint.get("arm", "left"),
                ),
                evidence + ["rm_get_joint_err_flag rc=%d" % rc],
            )

        # 5) 控制器规划类型（+ 轨迹数据）
        traj_type = ctypes.c_int(0)
        traj_data = (ctypes.c_float * ARM_DOF)()
        rc = int(
            binding.rm_get_arm_current_trajectory(
                handle, ctypes.pointer(traj_type), traj_data
            ) or 0
        )
        if rc != 0:
            return errors.error(
                "rm_get_arm_current_trajectory 返回 %d（未连接到控制器 / 通信失败）" % rc,
                "确认控制器在线后重跑 `rmctl connect --arm %s --json`" % (
                    self.endpoint.get("arm", "left"),
                ),
                evidence + ["rm_get_arm_current_trajectory rc=%d" % rc],
            )

        # 6) 组装快照（全部字段都来自真实 SDK 返回）
        pose = cur_state.pose
        realtime = _REALTIME.snapshot(self.realtime_period_ms)
        status_code = _REALTIME.arm_current_status
        snapshot = StateSnapshot(
            joint_deg=[float(v) for v in joints],
            joint_current_mA=[float(v) for v in all_state.joint_current],
            joint_temperature_c=[float(v) for v in all_state.joint_temperature],
            joint_voltage_v=[float(v) for v in all_state.joint_voltage],
            joint_en_flag=[int(v) for v in all_state.joint_en_flag],
            joint_err_code=[int(v) for v in all_state.joint_err_code],
            arm_err=[int(v) for v in cur_state.err.err[: int(cur_state.err.err_len)]],
            brake_state={
                "err_flag": int(err_flag.value),
                "brake_state": int(brake_state.value),
                "joint_err_flags": _err_flag_to_list(err_flag.value),
                "brake_open": _brake_to_list(brake_state.value),
                "hard_brake_joints": list(BRAKE_HARD_JOINTS),
                "soft_brake_joints": list(BRAKE_SOFT_JOINTS),
            },
            controller_trajectory_type=int(traj_type.value),
            controller_trajectory_type_name=TRAJECTORY_ENUM_NAMES.get(
                int(traj_type.value)
            ),
            arm_current_status=status_code,
            arm_current_status_name=(
                UDP_STATUS_ENUM_NAMES.get(status_code) if status_code is not None else None
            ),
            realtime=realtime,
            controller_pose={
                "position": {
                    "x": float(pose.position.x),
                    "y": float(pose.position.y),
                    "z": float(pose.position.z),
                    "unit": "m",
                },
                "quaternion": {
                    "w": float(pose.quaternion.w),
                    "x": float(pose.quaternion.x),
                    "y": float(pose.quaternion.y),
                    "z": float(pose.quaternion.z),
                },
                "euler": {
                    "rx": float(pose.euler.rx),
                    "ry": float(pose.euler.ry),
                    "rz": float(pose.euler.rz),
                    "unit": "rad",
                },
                "scope": controller_pose_scope,
            },
            controller_pose_scope=controller_pose_scope,
        )
        payload = snapshot.to_dict()
        return errors.ok(
            "状态快照采集成功（%d 个关节，实时上报 %d 帧）" % (
                snapshot.joint_count, realtime.get("frames", 0),
            ),
            "",
            evidence,
            snapshot=payload,
        )

    # -- 限值下发（R004）
    def set_joint_limits(self, limits: Dict[str, Any], dry_run: bool = True) -> Dict[str, Any]:
        """下发关节限值（驱动级生效，适合在线降速）。

        参数
        ----
        limits
            :func:`rm_hw_bridge.limits.load_limit_defaults` 的返回值；
            **``status="blocked"`` 时拒绝下发**（基线未实测，不得用占位值糊过去）。
        dry_run
            ``True``（默认）时**只打印 diff、绝不调用 SDK**；``False`` 才真正下发。

        下发面：``rm_set_joint_drive_max_speed`` / ``_max_acc`` / ``_min_pos`` /
        ``_max_pos``（各 7 关节）。单位换算只在本模块发生（SI → SDK 度制）。

        任何 SDK 返回非 0 → ``status="error"`` + reason（**接口名 + 返回码**）
        + next_action，不静默吞掉。
        """
        if not self.connected:
            return errors.result(
                self.status if self.status != errors.STATUS_OK else errors.STATUS_ERROR,
                self.reason or "尚未建立连接，无法下发限值",
                self.next_action,
                self.evidence,
            )

        limits = limits or {}
        limit_status = str(limits.get("status") or "")
        evidence: List[str] = list(limits.get("evidence") or [])
        if limit_status == limits_mod.STATUS_BLOCKED:
            return errors.blocked(
                limits.get("reason") or "限值来源基线未实测，拒绝下发",
                limits.get("next_action") or "",
                evidence,
                source=limits.get("source", ""),
                diff=[],
                applied=[],
                dry_run=bool(dry_run),
                sdk_calls=0,
            )

        values = limits.get("values") or {}
        current = limits.get("current") or {}
        if not values:
            return errors.error(
                "限值内容为空，拒绝下发",
                "确认 limits 由 `rm_hw_bridge.limits.load_limit_defaults()` 生成后重跑",
                evidence,
                diff=[],
                applied=[],
                dry_run=bool(dry_run),
                sdk_calls=0,
            )

        # 1) 组装 diff（当前值 = 声明/生效值，目标值 = 待下发值；均带单位）
        diff: List[Dict[str, Any]] = []
        for joint_name in _joint_order(values):
            idx = _joint_index(joint_name)
            target_item = values.get(joint_name) or {}
            current_item = current.get(joint_name) or {}
            for field, _sym, kind in _LIMIT_DOWNLINK_FIELDS:
                if field not in target_item:
                    continue
                si_unit, sdk_unit = _LIMIT_UNITS[field]
                diff.append({
                    "joint": joint_name,
                    "joint_index": idx,
                    "field": field,
                    "current": current_item.get(field),
                    "target": target_item.get(field),
                    "current_unit": si_unit,
                    "target_unit": si_unit,
                    "sdk_value": _to_sdk_unit(target_item[field], kind),
                    "sdk_unit": sdk_unit,
                })
            # jerk 仅供参考（白名单里没有 jerk 下发符号）
            if "jerk" in target_item:
                diff.append({
                    "joint": joint_name,
                    "joint_index": idx,
                    "field": "jerk",
                    "current": current_item.get("jerk"),
                    "target": target_item.get("jerk"),
                    "current_unit": "rad/s^3",
                    "target_unit": "rad/s^3",
                    "sdk_value": None,
                    "sdk_unit": "-",
                    "note": "白名单无 jerk 下发符号，仅记录不下发",
                })

        print_diff(diff)

        if dry_run:
            return errors.ok(
                "dry-run：已生成 %d 条限值 diff，**未调用任何 SDK**（未下发）" % len(diff),
                "",
                evidence + ["dry_run=True：SDK 调用数 0"],
                source=limits.get("source", ""),
                diff=diff,
                applied=[],
                dry_run=True,
                sdk_calls=0,
            )

        # 2) 真正下发（驱动级生效）
        binding = self._bridge.binding
        handle = self._handle
        applied: List[Dict[str, Any]] = []
        for entry in diff:
            if entry["sdk_value"] is None:
                continue
            field = entry["field"]
            sym, kind = next(
                (s, k) for (f, s, k) in _LIMIT_DOWNLINK_FIELDS if f == field
            )
            fn = getattr(binding, sym, None)
            if fn is None:
                return errors.error(
                    "契约清单缺少符号 %s，无法下发 %s.%s" % (sym, entry["joint"], field),
                    "确认所用 .so 为 RM-API2 1.1.7（`nm -D <so> | grep %s`），"
                    "重新物化 SDK 后重跑" % sym,
                    evidence + applied,
                    source=limits.get("source", ""),
                    diff=diff,
                    applied=applied,
                    dry_run=False,
                    sdk_calls=len(applied),
                )
            try:
                rc = int(fn(handle, int(entry["joint_index"]), float(entry["sdk_value"])) or 0)
            except Exception as exc:
                return errors.error(
                    "%s 调用异常（%s.%s）：%s" % (
                        sym, entry["joint"], field, exc,
                    ),
                    "确认 .so 与当前架构匹配（file <so>），修复后重跑",
                    evidence + applied + [repr(exc)],
                    source=limits.get("source", ""),
                    diff=diff,
                    applied=applied,
                    dry_run=False,
                    sdk_calls=len(applied),
                )
            applied.append({
                "joint": entry["joint"],
                "joint_index": entry["joint_index"],
                "field": field,
                "symbol": sym,
                "sdk_value": entry["sdk_value"],
                "sdk_unit": entry["sdk_unit"],
                "rc": rc,
            })
            if rc != 0:
                return errors.error(
                    "%s 返回 %d（下发 %s.%s 失败）" % (
                        sym, rc, entry["joint"], field,
                    ),
                    "确认控制器在线、句柄有效（同一 ip:port 只允许一个客户端），"
                    "并核对参数范围后重跑",
                    evidence + ["%s rc=%d" % (sym, rc)],
                    source=limits.get("source", ""),
                    diff=diff,
                    applied=applied,
                    dry_run=False,
                    sdk_calls=len(applied),
                )

        return errors.ok(
            "限值下发完成：%d 次 SDK 调用（7 关节 × 4 项）" % len(applied),
            "",
            evidence + ["下发接口: %s" % ", ".join(sorted({a["symbol"] for a in applied}))],
            source=limits.get("source", ""),
            diff=diff,
            applied=applied,
            dry_run=False,
            sdk_calls=len(applied),
        )

    # -- 安全开关下发（R004）
    def set_safety(
        self,
        collision_stage: Optional[int] = None,
        self_collision_enable: Optional[bool] = None,
        dry_run: bool = False,
    ) -> Dict[str, Any]:
        """下发安全开关（碰撞检测等级 / 自碰撞检测使能）。

        * ``collision_stage``：**0–8**（厂商头文件：0=不检测，8=最灵敏）；越界**拒绝**；
        * ``self_collision_enable``：必须是 bool；
        * 任何 SDK 返回非 0 → ``status="error"`` + reason（接口名 + 返回码）。
        """
        if not self.connected:
            return errors.result(
                self.status if self.status != errors.STATUS_OK else errors.STATUS_ERROR,
                self.reason or "尚未建立连接，无法下发安全开关",
                self.next_action,
                self.evidence,
            )

        if collision_stage is None and self_collision_enable is None:
            return errors.error(
                "未指定任何安全开关（collision_stage / self_collision_enable 均为 None）",
                "至少指定一项：collision_stage（0..8）或 self_collision_enable（bool）后重跑",
                [],
                sdk_calls=0,
            )

        planned: List[Dict[str, Any]] = []
        if collision_stage is not None:
            if isinstance(collision_stage, bool) or not isinstance(collision_stage, int):
                return errors.error(
                    "collision_stage 必须是整数（当前 %r）" % (collision_stage,),
                    "传入 0..8 的整数等级后重跑",
                    [],
                    sdk_calls=0,
                )
            if collision_stage < COLLISION_STAGE_MIN or collision_stage > COLLISION_STAGE_MAX:
                return errors.error(
                    "collision_stage=%d 越界（允许范围 %d..%d，0=不检测，8=最灵敏）"
                    % (collision_stage, COLLISION_STAGE_MIN, COLLISION_STAGE_MAX),
                    "把碰撞检测等级改到 %d..%d 后重跑" % (
                        COLLISION_STAGE_MIN, COLLISION_STAGE_MAX,
                    ),
                    [],
                    sdk_calls=0,
                )
            planned.append({
                "symbol": "rm_set_collision_state",
                "arg": int(collision_stage),
                "arg_name": "collision_stage",
            })

        if self_collision_enable is not None:
            if not isinstance(self_collision_enable, bool):
                return errors.error(
                    "self_collision_enable 必须是 bool（当前 %r）" % (self_collision_enable,),
                    "传入 True / False 后重跑",
                    [],
                    sdk_calls=0,
                )
            planned.append({
                "symbol": "rm_set_self_collision_enable",
                "arg": bool(self_collision_enable),
                "arg_name": "self_collision_enable",
            })

        if dry_run:
            return errors.ok(
                "dry-run：已生成 %d 条安全开关计划，**未调用任何 SDK**" % len(planned),
                "",
                [repr(p) for p in planned],
                planned=planned,
                applied=[],
                dry_run=True,
                sdk_calls=0,
            )

        binding = self._bridge.binding
        handle = self._handle
        applied: List[Dict[str, Any]] = []
        for item in planned:
            sym = item["symbol"]
            fn = getattr(binding, sym, None)
            if fn is None:
                return errors.error(
                    "契约清单缺少符号 %s，无法下发安全开关" % sym,
                    "确认所用 .so 为 RM-API2 1.1.7，重新物化 SDK 后重跑",
                    applied,
                    applied=applied,
                    dry_run=False,
                    sdk_calls=len(applied),
                )
            try:
                rc = int(fn(handle, item["arg"]) or 0)
            except Exception as exc:
                return errors.error(
                    "%s 调用异常（%s=%r）：%s" % (sym, item["arg_name"], item["arg"], exc),
                    "确认 .so 与当前架构匹配（file <so>），修复后重跑",
                    applied + [repr(exc)],
                    applied=applied,
                    dry_run=False,
                    sdk_calls=len(applied),
                )
            applied.append(dict(item, rc=rc))
            if rc != 0:
                return errors.error(
                    "%s 返回 %d（下发 %s 失败）" % (sym, rc, item["arg_name"]),
                    "确认控制器在线、句柄有效（同一 ip:port 只允许一个客户端），"
                    "并核对参数范围后重跑",
                    applied + ["%s rc=%d" % (sym, rc)],
                    applied=applied,
                    dry_run=False,
                    sdk_calls=len(applied),
                )

        return errors.ok(
            "安全开关下发完成：%d 次 SDK 调用（%s）" % (
                len(applied), ", ".join(a["symbol"] for a in applied),
            ),
            "",
            [repr(a) for a in applied],
            planned=planned,
            applied=applied,
            dry_run=False,
            sdk_calls=len(applied),
        )

    # -- 安全窄接口（L1 降速 / SDK 超时 / 控制器错误状态）
    def set_drive_max_speed(self, joint_index: Any, deg_s: Any) -> Dict[str, Any]:
        """按比例下调**单个关节**的驱动最大速度（L1 降速的窄接口）。

        * ``joint_index``：**0 基**关节序号（整数，``0..6``；bool 不算）；
          内部换算成 SDK 的 **1 基** ``joint_num``（= ``joint_index + 1``）；
        * ``deg_s``：目标最大速度，单位 **°/s**（SDK 侧单位同为 °/s → **不换算**，
          本模块也不新开换算点）；
        * 非法参数 → **0 次 SDK 调用** + ``status="error"`` + 可粘贴 next_action；
        * SDK 返回非 0 → ``status="error"``，reason 写明**接口名 + 返回码**。

        成功返回的 ``applied`` 含 ``joint_index`` / ``deg_s`` / ``rc``。
        """
        if not self.connected:
            return self._not_connected("set_drive_max_speed")

        if isinstance(joint_index, bool) or not isinstance(joint_index, int):
            return errors.error(
                "joint_index 必须是整数（当前 %r）" % (joint_index,),
                "传入 0..%d 的整数关节序号（0 基）后重跑" % JOINT_INDEX_MAX,
                [],
                sdk_calls=0,
                joint_index=None,
                deg_s=None,
            )
        if joint_index < JOINT_INDEX_MIN or joint_index > JOINT_INDEX_MAX:
            return errors.error(
                "joint_index=%d 越界（允许范围 %d..%d，0 基）" % (
                    joint_index, JOINT_INDEX_MIN, JOINT_INDEX_MAX,
                ),
                "把关节序号改到 %d..%d 后重跑" % (JOINT_INDEX_MIN, JOINT_INDEX_MAX),
                [],
                sdk_calls=0,
                joint_index=joint_index,
                deg_s=None,
            )
        if not _is_finite_number(deg_s):
            return errors.error(
                "deg_s 必须是有限数值（当前 %r）" % (deg_s,),
                "传入正的有限速度（°/s，如 15.0）后重跑",
                [],
                sdk_calls=0,
                joint_index=joint_index,
                deg_s=None,
            )
        speed = float(deg_s)
        if speed <= 0:
            return errors.error(
                "deg_s=%r 非法（必须是正的有限速度，°/s）" % (deg_s,),
                "传入正的有限速度（°/s，如 15.0）后重跑",
                [],
                sdk_calls=0,
                joint_index=joint_index,
                deg_s=None,
            )

        sym = "rm_set_joint_drive_max_speed"
        fn = getattr(self._bridge.binding, sym, None)
        if fn is None:
            return errors.error(
                "契约清单缺少符号 %s，无法下发关节最大速度" % sym,
                "确认所用 .so 为 RM-API2 1.1.7（`nm -D <so> | grep %s`），"
                "重新物化 SDK 后重跑" % sym,
                [],
                sdk_calls=0,
                joint_index=joint_index,
                deg_s=speed,
            )
        sdk_joint_num = joint_index + 1
        try:
            rc = int(fn(self._handle, int(sdk_joint_num), speed) or 0)
        except Exception as exc:  # noqa: BLE001 - SDK 调用异常统一转 error
            return errors.error(
                "%s 调用异常（joint_index=%d, deg_s=%r）：%s" % (
                    sym, joint_index, speed, exc,
                ),
                "确认 .so 与当前架构匹配（file <so>），修复后重跑",
                [repr(exc)],
                sdk_calls=1,
                joint_index=joint_index,
                deg_s=speed,
            )
        applied = {
            "joint_index": joint_index,
            "joint": "joint%d" % sdk_joint_num,
            "sdk_joint_num": sdk_joint_num,
            "deg_s": speed,
            "unit": "deg/s",
            "symbol": sym,
            "rc": rc,
        }
        if rc != 0:
            return errors.error(
                "%s 返回 %d（下发 joint%d 的最大速度 %r °/s 失败）" % (
                    sym, rc, sdk_joint_num, speed,
                ),
                "确认控制器在线、句柄有效（同一 ip:port 只允许一个客户端），"
                "并核对速度范围后重跑",
                ["%s rc=%d" % (sym, rc)],
                sdk_calls=1,
                joint_index=joint_index,
                deg_s=speed,
                applied=[applied],
            )
        return errors.ok(
            "关节最大速度已下发：joint%d = %r °/s" % (sdk_joint_num, speed),
            "",
            ["%s rc=0 joint_num=%d max_speed=%r °/s" % (sym, sdk_joint_num, speed)],
            sdk_calls=1,
            joint_index=joint_index,
            deg_s=speed,
            applied=[applied],
        )

    def set_timeout(self, seconds: Any) -> Dict[str, Any]:
        """设置 SDK 通信超时（**窄接口**，support 符号 ``rm_set_timeout``）。

        * ``seconds``：秒（正的有限数）；非法 → **0 次 SDK 调用** + error；
        * 成功 → 注明「SDK 侧**全局**生效」（**不伪读回值**）；
        * **单位换算**：对外**秒** → SDK 侧**毫秒**（厂商头文件：默认 500 ms），
          换算只在本模块一处（``_call_set_timeout``）。
        """
        if not self.connected:
            return self._not_connected("set_timeout")
        if not _is_finite_number(seconds):
            return errors.error(
                "seconds 必须是数值（秒，当前 %r）" % (seconds,),
                "传入正的有限秒数（如 1.0）后重跑",
                [],
                sdk_calls=0,
                timeout_s=None,
            )
        value = float(seconds)
        if value <= 0:
            return errors.error(
                "seconds=%r 非法（必须是正的有限秒数）" % (seconds,),
                "传入正的有限秒数（如 1.0）后重跑",
                [],
                sdk_calls=0,
                timeout_s=None,
            )
        return _call_set_timeout(self._bridge.binding, value)

    def controller_error_state(self) -> Dict[str, Any]:
        """控制器错误状态（安全判据的输入源）。

        返回::

            {
              "status": "ok" | "error",
              "reason": str, "next_action": str, "evidence": [...],
              "arm_err": list[int] | None,          # rm_get_current_arm_state.err
              "arm_current_status": int | None,      # UDP 实时上报缓存
              "arm_current_status_name": str | None,
            }

        * ``arm_err`` 来源：``rm_get_current_arm_state``（**core 22 内**）；
        * ``arm_current_status`` / ``_name`` 来源：**进程级** UDP 实时上报缓存
          （**不额外打 SDK**）；**无数据时为 ``None``（不返回 0 冒充）**；
        * SDK 失败 → ``status="error"``，三个数据字段全为 ``None``（不伪造）。
        """
        empty = {
            "arm_err": None,
            "arm_current_status": None,
            "arm_current_status_name": None,
        }
        if not self.connected:
            res = self._not_connected("controller_error_state")
            res.update(empty)
            return res

        sym = "rm_get_current_arm_state"
        fn = getattr(self._bridge.binding, sym, None)
        if fn is None:
            res = errors.error(
                "契约清单缺少符号 %s，无法读取控制器错误状态" % sym,
                "确认所用 .so 为 RM-API2 1.1.7（`nm -D <so> | grep %s`），"
                "重新物化 SDK 后重跑" % sym,
                [],
                sdk_calls=0,
            )
            res.update(empty)
            return res

        cur_state = gen.rm_current_arm_state_t()
        try:
            rc = int(fn(self._handle, ctypes.pointer(cur_state)) or 0)
        except Exception as exc:  # noqa: BLE001 - SDK 调用异常统一转 error
            res = errors.error(
                "%s 调用异常：%s" % (sym, exc),
                "确认控制器在线、句柄有效后重跑 `rmctl connect --arm %s --json`" % (
                    self.endpoint.get("arm", "left"),
                ),
                [repr(exc)],
                sdk_calls=1,
            )
            res.update(empty)
            return res
        if rc != 0:
            res = errors.error(
                "%s 返回 %d（未连接到控制器 / 通信失败）" % (sym, rc),
                "确认控制器在线、句柄有效后重跑 `rmctl connect --arm %s --json`" % (
                    self.endpoint.get("arm", "left"),
                ),
                ["%s rc=%d" % (sym, rc)],
                sdk_calls=1,
            )
            res.update(empty)
            return res

        err_len = int(cur_state.err.err_len)
        arm_err = [int(v) for v in cur_state.err.err[:err_len]]
        status_code = _REALTIME.arm_current_status
        status_name = (
            UDP_STATUS_ENUM_NAMES.get(int(status_code))
            if status_code is not None else None
        )
        return errors.ok(
            "控制器错误状态读取成功（臂级错误码 %d 个）" % len(arm_err),
            "",
            [
                "%s rc=0" % sym,
                "arm_err 来源: rm_get_current_arm_state.err（err_len=%d）" % err_len,
                "arm_current_status 来源: UDP 实时上报缓存（frames=%d，不额外打 SDK）"
                % _REALTIME.snapshot(self.realtime_period_ms).get("frames", 0),
            ],
            sdk_calls=1,
            arm_err=arm_err,
            arm_current_status=(int(status_code) if status_code is not None else None),
            arm_current_status_name=status_name,
        )

    # ------------------------------------------------- 主动上报开关（窄接口）
    def set_realtime_push_enabled(self, enabled: bool,
                                  port: Optional[int] = None,
                                  target_ip: Optional[str] = None) -> Dict[str, Any]:
        """开/关 UDP 主动上报（``rm_set_realtime_push`` 的 ``enable`` 位）。

        用途：**真机断流复现**——执行层要求"UDP 断流 200 ms 内停机"必须有真机证据；
        离线只能注入假断流。关掉主动上报后，控制器不再上报，执行层的反馈龄期
        会在 200 ms 内越过阈值并触发 L4 急停 —— 这是**真实**的断流链路。

        * 只用契约白名单内的 ``rm_set_realtime_push``（C 类：纯配置，无解算）；
        * 关闭时把 ``enable=0``、``cycle`` 保持合法值（5 的倍数），避免配置非法；
        * 目标 IP/端口沿用建链时的配置（未显式给出时从实时上报状态推断，
          推断不到则用 ``0.0.0.0`` + 当前端口）。
        """
        if not self.connected:
            return self._not_connected("主动上报开关")
        sym = "rm_set_realtime_push"
        fn = getattr(self._bridge.binding, sym, None)
        if fn is None:
            return errors.error(
                "契约清单缺少符号 %s，无法开关主动上报" % sym,
                "确认所用 .so 为 RM-API2 1.1.7，重新物化 SDK 后重跑",
                [], symbol=sym, sdk_calls=0)
        cfg = gen.rm_realtime_push_config_t()
        cfg.cycle = int(self.realtime_period_ms or REALTIME_CYCLE_MULTIPLE_MS)
        cfg.enable = bool(enabled)
        cfg.port = int(port or self._udp_port or 0)
        cfg.force_coordinate = 0
        cfg.ip = (target_ip or self._realtime_target_ip or "0.0.0.0").encode("utf-8")[:27]
        cfg.custom_config.arm_current_status = 1
        cfg.custom_config.joint_speed = 1
        try:
            rc = int(fn(self._handle, cfg) or 0)
        except Exception as exc:  # noqa: BLE001 - SDK 异常统一转 error
            return errors.error(
                "%s 调用异常：%s" % (sym, exc),
                "确认 .so 与当前架构匹配后重跑", [repr(exc)],
                symbol=sym, rc=None, sdk_calls=1)
        if rc != 0:
            return errors.error(
                "%s 返回 %d（开关主动上报失败）" % (sym, rc),
                "确认控制器在线、目标 IP/端口合法后重跑",
                ["%s rc=%d" % (sym, rc)], symbol=sym, rc=rc, sdk_calls=1)
        return errors.ok(
            "主动上报已%s" % ("开启" if enabled else "关闭"),
            "", ["%s rc=0" % sym],
            symbol=sym, rc=0, sdk_calls=1, enabled=bool(enabled))

    # ------------------------------------------------------- 关节角透传通道
    def _not_connected(self, action: str) -> Dict[str, Any]:
        """未连接时的统一失败结果（**不伪造任何数值**）。"""
        return errors.result(
            self.status if self.status != errors.STATUS_OK else errors.STATUS_ERROR,
            self.reason or ("尚未建立连接，无法执行 %s" % action),
            self.next_action,
            self.evidence,
            sdk_calls=0,
        )

    def _retain_joint_buffer(self, buf: Any, config: Any) -> None:
        """**显式持有**关节角缓冲（防止 GC 提前回收）+ **有界**保留。

        厂商 Python 参考实现是 ``config.joint = ctypes.pointer(arr)``，
        缓冲必须**活到调用结束**；这里额外保留最近若干个，
        保证 SDK 侧异步取用时也不会读到已回收内存。
        """
        self._joint_buffers.append(buf)
        overflow = len(self._joint_buffers) - MAX_RETAINED_JOINT_BUFFERS
        if overflow > 0:
            del self._joint_buffers[:overflow]
        self._last_transmit = {"buffer": buf, "config": config}

    def transmit_joint(
        self,
        deg7: Any,
        follow: bool = False,
        trajectory_mode: int = TRAJECTORY_MODE_CURVE_FIT,
        radio: int = 30,
        expand: float = 0.0,
    ) -> Dict[str, Any]:
        """关节角透传（``rm_movej_canfd``）——**唯一的关节透传通道**。

        参数
        ----
        deg7
            7 个关节角，**单位度**（内部不换算，SDK 也是度制）。
        follow
            ``True`` 高跟随 / ``False`` 低跟随（写入 ``rm_movej_canfd_mode_t.follow``）。
        trajectory_mode
            ``0`` 完全透传 / ``1`` 曲线拟合 / ``2`` 滤波。
        radio
            平滑系数：曲线拟合 ``0..100``、滤波 ``0..1000``（完全透传模式下忽略）。

        **只走关节角透传**：位姿透传（``rm_movep_canfd`` / ``rm_movep_follow``）
        在厂商黑名单里（控制器会偷偷做逆解，违反 R003）。

        非法参数 → ``status="error"`` + reason + next_action，**0 次 SDK 调用**；
        SDK 返回非 0 → ``status="error"`` + reason（**接口名 + 返回码**）+ next_action。
        """
        if not self.connected:
            return self._not_connected("关节角透传")

        values, err = _normalize_deg7(deg7)
        if err:
            return errors.error(
                err,
                "传入 7 个关节角（单位：度）后重跑",
                [],
                symbol="rm_movej_canfd",
                sdk_calls=0,
            )
        if isinstance(trajectory_mode, bool) or not isinstance(trajectory_mode, int):
            return errors.error(
                "trajectory_mode 必须是整数（当前 %r）" % (trajectory_mode,),
                "传入 0（完全透传）/ 1（曲线拟合）/ 2（滤波）后重跑",
                [],
                symbol="rm_movej_canfd",
                sdk_calls=0,
            )
        if trajectory_mode not in TRAJECTORY_MODES:
            return errors.error(
                "trajectory_mode=%d 非法（允许 0=完全透传 / 1=曲线拟合 / 2=滤波）"
                % trajectory_mode,
                "把 trajectory_mode 改成 0 / 1 / 2 后重跑",
                [],
                symbol="rm_movej_canfd",
                sdk_calls=0,
            )
        if isinstance(radio, bool) or not isinstance(radio, int):
            return errors.error(
                "radio 必须是整数（当前 %r）" % (radio,),
                "传入整数平滑系数后重跑",
                [],
                symbol="rm_movej_canfd",
                sdk_calls=0,
            )
        if trajectory_mode == TRAJECTORY_MODE_CURVE_FIT:
            radio_lo, radio_hi = RADIO_RANGE_CURVE_FIT
        elif trajectory_mode == TRAJECTORY_MODE_FILTER:
            radio_lo, radio_hi = RADIO_RANGE_FILTER
        else:
            radio_lo, radio_hi = 0, 1000
        if radio < radio_lo or radio > radio_hi:
            return errors.error(
                "radio=%d 越界（trajectory_mode=%d 允许范围 %d..%d）" % (
                    radio, trajectory_mode, radio_lo, radio_hi,
                ),
                "把 radio 改到 %d..%d 后重跑" % (radio_lo, radio_hi),
                [],
                symbol="rm_movej_canfd",
                sdk_calls=0,
            )
        if not isinstance(follow, bool):
            return errors.error(
                "follow 必须是 bool（当前 %r）" % (follow,),
                "传入 True（高跟随）/ False（低跟随）后重跑",
                [],
                symbol="rm_movej_canfd",
                sdk_calls=0,
            )
        if isinstance(expand, bool) or not isinstance(expand, (int, float)):
            return errors.error(
                "expand 必须是数字（当前 %r）" % (expand,),
                "传入扩展关节角数值（无扩展关节时传 0）后重跑",
                [],
                symbol="rm_movej_canfd",
                sdk_calls=0,
            )

        binding = self._bridge.binding
        fn = getattr(binding, "rm_movej_canfd", None)
        if fn is None:
            return errors.error(
                "契约清单缺少符号 rm_movej_canfd，无法透传关节角",
                "确认所用 .so 为 RM-API2 1.1.7（`nm -D <so> | grep rm_movej_canfd`），"
                "重新物化 SDK 后重跑",
                [],
                symbol="rm_movej_canfd",
                sdk_calls=0,
            )

        # 组装配置（结构体按值传入）；缓冲**显式持有**到调用返回之后
        buf = (ctypes.c_float * ARM_DOF)(*values)
        config = gen.rm_movej_canfd_mode_t()
        config.joint = ctypes.cast(buf, ctypes.POINTER(ctypes.c_float))
        config.expand = float(expand)
        config.follow = bool(follow)
        config.trajectory_mode = int(trajectory_mode)
        config.radio = int(radio)
        self._retain_joint_buffer(buf, config)

        try:
            rc = int(fn(self._handle, config) or 0)
        except Exception as exc:
            return errors.error(
                "rm_movej_canfd 调用异常：%s" % exc,
                "确认 .so 与当前架构匹配（file <so>），修复后重跑",
                [repr(exc)],
                symbol="rm_movej_canfd",
                rc=None,
                sdk_calls=1,
            )
        if rc != 0:
            return errors.error(
                "rm_movej_canfd 返回 %d（关节角透传失败）" % rc,
                "确认控制器在线、句柄有效（同一 ip:port 只允许一个客户端），"
                "并核对关节角范围后重跑",
                ["rm_movej_canfd rc=%d" % rc],
                symbol="rm_movej_canfd",
                rc=rc,
                sdk_calls=1,
                joint_deg=values,
                follow=bool(follow),
                trajectory_mode=int(trajectory_mode),
                radio=int(radio),
            )

        return errors.ok(
            "关节角透传已下发（%d 个关节，trajectory_mode=%d，radio=%d）" % (
                ARM_DOF, trajectory_mode, radio,
            ),
            "",
            ["rm_movej_canfd rc=0"],
            symbol="rm_movej_canfd",
            rc=0,
            sdk_calls=1,
            joint_deg=values,
            follow=bool(follow),
            trajectory_mode=int(trajectory_mode),
            radio=int(radio),
            expand=float(expand),
        )

    def preposition_movej(
        self,
        deg7: Any,
        speed: int = 10,
        radius: int = 0,
        block: int = 0,
        trajectory_connect: int = 0,
    ) -> Dict[str, Any]:
        """预就位 / 归位（``rm_movej``，关节空间**规划**运动）。

        参数
        ----
        deg7
            7 个关节角（单位度）。
        speed
            速度百分比（``v``，允许 1..100）。
        radius
            交融半径百分比（``r``，允许 0..100）。
        block
            ``0`` 非阻塞 / ``1`` 阻塞（**阻塞时本调用会一直等到到位**）。
        trajectory_connect
            ``0`` 立即规划执行 / ``1`` 与下一条轨迹连接。

        与 :meth:`transmit_joint` 的分工：**本接口走控制器规划**（慢、可到位），
        透传接口走 CANFD 直发（快、不规划）。两者都用**关节角**，不用位姿。
        """
        if not self.connected:
            return self._not_connected("预就位运动")

        values, err = _normalize_deg7(deg7)
        if err:
            return errors.error(
                err,
                "传入 7 个关节角（单位：度）后重跑",
                [],
                symbol="rm_movej",
                sdk_calls=0,
            )
        for name, value, lo, hi in (
            ("speed", speed, 1, 100),
            ("radius", radius, 0, 100),
        ):
            if isinstance(value, bool) or not isinstance(value, int):
                return errors.error(
                    "%s 必须是整数（当前 %r）" % (name, value),
                    "传入 %d..%d 的整数后重跑" % (lo, hi),
                    [],
                    symbol="rm_movej",
                    sdk_calls=0,
                )
            if value < lo or value > hi:
                return errors.error(
                    "%s=%d 越界（允许范围 %d..%d）" % (name, value, lo, hi),
                    "把 %s 改到 %d..%d 后重跑" % (name, lo, hi),
                    [],
                    symbol="rm_movej",
                    sdk_calls=0,
                )
        for name, value in (
            ("block", block),
            ("trajectory_connect", trajectory_connect),
        ):
            if isinstance(value, bool) or not isinstance(value, int) or value not in (0, 1):
                return errors.error(
                    "%s 必须是 0 或 1（当前 %r）" % (name, value),
                    "传入 0 或 1 后重跑",
                    [],
                    symbol="rm_movej",
                    sdk_calls=0,
                )

        binding = self._bridge.binding
        fn = getattr(binding, "rm_movej", None)
        if fn is None:
            return errors.error(
                "契约清单缺少符号 rm_movej，无法执行预就位",
                "确认所用 .so 为 RM-API2 1.1.7（`nm -D <so> | grep rm_movej`），"
                "重新物化 SDK 后重跑",
                [],
                symbol="rm_movej",
                sdk_calls=0,
            )

        buf = (ctypes.c_float * ARM_DOF)(*values)
        self._retain_joint_buffer(buf, None)
        try:
            rc = int(
                fn(
                    self._handle,
                    ctypes.cast(buf, ctypes.POINTER(ctypes.c_float)),
                    int(speed),
                    int(radius),
                    int(trajectory_connect),
                    int(block),
                )
                or 0
            )
        except Exception as exc:
            return errors.error(
                "rm_movej 调用异常：%s" % exc,
                "确认 .so 与当前架构匹配（file <so>），修复后重跑",
                [repr(exc)],
                symbol="rm_movej",
                rc=None,
                sdk_calls=1,
            )
        if rc != 0:
            return errors.error(
                "rm_movej 返回 %d（预就位失败）" % rc,
                "确认控制器在线、句柄有效（同一 ip:port 只允许一个客户端），"
                "并核对目标角度与速度参数后重跑",
                ["rm_movej rc=%d" % rc],
                symbol="rm_movej",
                rc=rc,
                sdk_calls=1,
                joint_deg=values,
                speed=int(speed),
                radius=int(radius),
                block=int(block),
                trajectory_connect=int(trajectory_connect),
            )
        return errors.ok(
            "预就位指令已下发（7 个关节，速度 %d%%，交融半径 %d%%，%s）" % (
                speed, radius, "阻塞" if block else "非阻塞",
            ),
            "",
            ["rm_movej rc=0"],
            symbol="rm_movej",
            rc=0,
            sdk_calls=1,
            joint_deg=values,
            speed=int(speed),
            radius=int(radius),
            block=int(block),
            trajectory_connect=int(trajectory_connect),
        )

    # ------------------------------------------------------------ 停机族语义
    def _stop_call(self, symbol: str, label: str) -> Dict[str, Any]:
        """停机族统一实现（符号映射 + 返回码处理）。"""
        if not self.connected:
            return self._not_connected(label)
        binding = self._bridge.binding
        fn = getattr(binding, symbol, None)
        if fn is None:
            return errors.error(
                "契约清单缺少符号 %s，无法执行%s" % (symbol, label),
                "确认所用 .so 为 RM-API2 1.1.7（`nm -D <so> | grep %s`），"
                "重新物化 SDK 后重跑" % symbol,
                [],
                symbol=symbol,
                sdk_calls=0,
            )
        try:
            rc = int(fn(self._handle) or 0)
        except Exception as exc:
            return errors.error(
                "%s 调用异常（%s）：%s" % (symbol, label, exc),
                "确认 .so 与当前架构匹配（file <so>），修复后重跑",
                [repr(exc)],
                symbol=symbol,
                rc=None,
                sdk_calls=1,
            )
        if rc != 0:
            return errors.error(
                "%s 返回 %d（%s失败）" % (symbol, rc, label),
                "确认控制器在线、句柄有效（同一 ip:port 只允许一个客户端）后重跑",
                ["%s rc=%d" % (symbol, rc)],
                symbol=symbol,
                rc=rc,
                sdk_calls=1,
            )
        return errors.ok(
            "%s已下发（%s）" % (label, symbol),
            "",
            ["%s rc=0" % symbol],
            symbol=symbol,
            rc=0,
            sdk_calls=1,
        )

    def stop(self) -> Dict[str, Any]:
        """急停（``rm_set_arm_stop``）。"""
        return self._stop_call("rm_set_arm_stop", "急停")

    def slow_stop(self) -> Dict[str, Any]:
        """缓停（``rm_set_arm_slow_stop``）。"""
        return self._stop_call("rm_set_arm_slow_stop", "缓停")

    def pause(self) -> Dict[str, Any]:
        """暂停（``rm_set_arm_pause``）。"""
        return self._stop_call("rm_set_arm_pause", "暂停")

    def continue_(self) -> Dict[str, Any]:
        """继续（``rm_set_arm_continue``）。"""
        return self._stop_call("rm_set_arm_continue", "继续")

    def delete_current_trajectory(self) -> Dict[str, Any]:
        """删除当前轨迹（``rm_set_delete_current_trajectory``）——规划器崩溃兜底。"""
        return self._stop_call("rm_set_delete_current_trajectory", "删除当前轨迹")

    def current_trajectory_type(self) -> Dict[str, Any]:
        """查询控制器当前规划类型（``rm_get_arm_current_trajectory``）。

        返回 ``type``（整数值）、``type_name``（枚举名，如 ``RM_NO_PLANNING_E``）
        与 ``planning``（是否在规划中）——用于**检测控制器是否在偷偷规划**
        （透传模式下应为 ``RM_NO_PLANNING_E``）。
        """
        if not self.connected:
            return self._not_connected("规划类型查询")
        binding = self._bridge.binding
        fn = getattr(binding, "rm_get_arm_current_trajectory", None)
        if fn is None:
            return errors.error(
                "契约清单缺少符号 rm_get_arm_current_trajectory，无法查询规划类型",
                "确认所用 .so 为 RM-API2 1.1.7，重新物化 SDK 后重跑",
                [],
                symbol="rm_get_arm_current_trajectory",
                sdk_calls=0,
            )
        traj_type = ctypes.c_int(0)
        traj_data = (ctypes.c_float * ARM_DOF)()
        try:
            rc = int(
                fn(self._handle, ctypes.pointer(traj_type), traj_data) or 0
            )
        except Exception as exc:
            return errors.error(
                "rm_get_arm_current_trajectory 调用异常：%s" % exc,
                "确认 .so 与当前架构匹配（file <so>），修复后重跑",
                [repr(exc)],
                symbol="rm_get_arm_current_trajectory",
                rc=None,
                sdk_calls=1,
            )
        if rc != 0:
            return errors.error(
                "rm_get_arm_current_trajectory 返回 %d（查询规划类型失败）" % rc,
                "确认控制器在线、句柄有效后重跑",
                ["rm_get_arm_current_trajectory rc=%d" % rc],
                symbol="rm_get_arm_current_trajectory",
                rc=rc,
                sdk_calls=1,
            )
        value = int(traj_type.value)
        name = TRAJECTORY_ENUM_NAMES.get(value)
        return errors.ok(
            "当前规划类型：%s（%d）" % (name or "未知", value),
            "",
            ["rm_get_arm_current_trajectory rc=0", "trajectory_type=%d" % value],
            symbol="rm_get_arm_current_trajectory",
            rc=0,
            sdk_calls=1,
            type=value,
            type_name=name,
            planning=(value != 0),
            data=[float(v) for v in traj_data],
        )

    def __repr__(self) -> str:
        return "<ArmLink status=%s endpoint=%s>" % (self.status, self.endpoint)


# --------------------------------------------------------------------- 连接入口
def connect(
    ip: str,
    port: int = DEFAULT_TCP_PORT,
    udp_port: Optional[int] = None,
    enable_realtime: bool = True,
    lib: Any = None,
    ws: Optional[str] = None,
    env: Optional[Dict[str, str]] = None,
    realtime_period_ms: int = REALTIME_CYCLE_MULTIPLE_MS,
    realtime_target_ip: Optional[str] = None,
    realtime_target_port: Optional[int] = None,
    arm: str = "left",
) -> ArmLink:
    """连接机械臂并准备状态快照。

    参数
    ----
    ip, port
        控制器地址（TCP）。
    udp_port
        UDP 主动上报端口（默认与 ``port`` 相同）。
    enable_realtime
        是否下发 UDP 主动上报配置并注册回调。
    lib
        注入的 SDK 对象（离线测试用假 SDK）；``None`` 时自动找 ``.so``。
    realtime_period_ms
        上报周期（毫秒），**必须是 5 的倍数**。
    realtime_target_ip / realtime_target_port
        UDP 上报的**目标**地址（SDK 从臂侧把报文发到这里；默认取 ``ip`` / ``udp_port``）。
        注意：基线 ``network_plan.yaml`` 只记了**臂侧**地址，没有记运行本进程的主机地址；
        真机窗口里若 SDK 报 "invaild ip"，应用 ``--realtime-ip <本机可达IP>`` 覆盖。

    **建连期接线**：句柄建立成功后调用一次 ``rm_set_timeout``，秒数取自
    ``workcell/rm_workcell.yaml::safety.sdk_timeout_s``（缺文件/缺键 → 默认 1.0 s
    并写入 evidence）；该调用**失败不阻断连接**（结果留在 ``link.sdk_timeout_result``）。

    返回
    ----
    :class:`ArmLink` —— 其 ``status`` 字段为 ``ok`` / ``blocked`` / ``error``。
    """
    endpoint = {"arm": arm, "ip": ip, "port": int(port), "udp_port": udp_port}

    # 1) SDK 就位检查
    lib_path = None
    if lib is None:
        lib_path, candidates = _find_sdk(ws=ws, env=env)
        if not lib_path:
            return ArmLink(
                bridge=None,
                status=errors.STATUS_BLOCKED,
                reason="找不到 RM-API2 动态库（libapi_c.so），无法连接真机",
                next_action=(
                    "把 RM-API2 SDK 物化后重试，二选一："
                    "① export RM_API2_LIB=/abs/path/libapi_c.so；"
                    "② mkdir -p %s && cp /abs/path/libapi_c.so %s"
                    % (
                        os.path.join(paths.workspace_root(ws), ".deps", "rm_api2"),
                        os.path.join(
                            paths.workspace_root(ws), ".deps", "rm_api2", "libapi_c.so"
                        ),
                    )
                ),
                evidence=[
                    "已搜索: %s" % p for p in candidates
                ] or ["未找到任何候选路径"],
                endpoint=endpoint,
            )
    else:
        candidates = ["<injected>"]

    # 2) 装载 + 绑定（只绑清单符号）
    try:
        bridge = loader.RmBridge(lib=lib, ws=ws, env=env)
    except loader.MissingSymbolsError as exc:
        return ArmLink(
            bridge=None,
            status=errors.STATUS_BLOCKED,
            reason="RM-API2 库缺少契约清单符号：%s" % ", ".join(exc.missing_symbols),
            next_action=(
                "确认所用 .so 为 RM-API2 1.1.7（`nm -D <so> | grep rm_init` 可核对），"
                "必要时重新物化 SDK 后重跑"
            ),
            evidence=["missing: %s" % m for m in exc.missing_symbols],
            endpoint=endpoint,
        )
    except Exception as exc:
        return ArmLink(
            bridge=None,
            status=errors.STATUS_BLOCKED,
            reason="装载 RM-API2 失败：%s" % exc,
            next_action=(
                "确认 %s 可读且架构匹配（file <so>），修复后重跑" % (
                    lib_path or "<lib>"
                )
            ),
            evidence=[repr(exc)],
            endpoint=endpoint,
        )

    binding = bridge.binding
    evidence: List[str] = []
    if lib_path:
        evidence.append("sdk: %s" % lib_path)

    # 3) SDK 初始化（必须三线程模式，否则收不到 UDP 主动上报）
    try:
        rc = int(binding.rm_init(gen.RM_TRIPLE_MODE_E) or 0)
    except Exception as exc:
        return ArmLink(
            bridge=bridge,
            status=errors.STATUS_ERROR,
            reason="rm_init 调用异常：%s" % exc,
            next_action="检查 .so 是否与当前架构匹配（file <so>），修复后重跑",
            evidence=evidence + [repr(exc)],
            endpoint=endpoint,
        )
    if rc != 0:
        return ArmLink(
            bridge=bridge,
            status=errors.STATUS_ERROR,
            reason="rm_init(RM_TRIPLE_MODE_E) 返回 %d（SDK 初始化失败）" % rc,
            next_action="确认没有其它进程占用 SDK 单例（同一 ip:port 只允许一个客户端），"
                        "必要时重启进程后重跑",
            evidence=evidence + ["rm_init rc=%d" % rc],
            endpoint=endpoint,
        )

    # 4) 创建句柄（失败仍返回非空指针 → 必须读 id）
    try:
        handle = binding.rm_create_robot_arm(ip.encode("utf-8"), int(port))
    except Exception as exc:
        return ArmLink(
            bridge=bridge,
            status=errors.STATUS_ERROR,
            reason="rm_create_robot_arm 调用异常：%s" % exc,
            next_action="确认 ip/port 合法（%s:%s）后重跑" % (ip, port),
            evidence=evidence + [repr(exc)],
            endpoint=endpoint,
        )
    if handle is None:
        reachable = _tcp_reachable(ip, port)
        return ArmLink(
            bridge=bridge,
            status=errors.STATUS_BLOCKED,
            reason=("rm_create_robot_arm 返回空指针：控制器 %s:%s %s"
                    % (ip, port,
                       "可达但不接纳新客户端（该控制器的客户端连接数已达上限）"
                       if reachable else "端口不可达（控制器离线 / 网络不通）")),
            next_action=(
                "连接数上限的处置：① **同一进程内建一次链、全程复用**（不要每条命令都重连）；"
                "② 等待控制器释放旧连接后重跑（实测释放很慢）；"
                "③ 必要时重启控制器后重跑。"
                if reachable else
                "确认控制器已上电、网线正常、ip=%s 正确后重跑" % ip),
            evidence=evidence + ["rm_create_robot_arm -> NULL",
                                 "tcp_reachable=%s" % reachable],
            endpoint=endpoint,
        )
    handle_id = _handle_id(handle)
    if handle_id is None or handle_id < 0:
        reachable = _tcp_reachable(ip, port)
        return ArmLink(
            bridge=bridge,
            status=errors.STATUS_BLOCKED if reachable else errors.STATUS_ERROR,
            reason=("rm_create_robot_arm 返回无效句柄 id=%s：控制器 %s:%s %s"
                    % (handle_id, ip, port,
                       "**可达但不接纳新客户端**（客户端连接数已达上限）"
                       if reachable else "不可达（控制器离线 / 网络不通）")),
            next_action=(
                "连接数上限的处置：① **同一进程内建一次链、全程复用**（不要每条命令都重连）；"
                "② 等待控制器释放旧连接后重跑（实测释放很慢）；"
                "③ 必要时重启控制器后重跑。"
                if reachable else
                "1) ping %s 确认控制器在线；"
                "2) 确认 %s:%s 的 TCP 端口可达；"
                "3) 确认同一 ip:port 没有被其它客户端占用（单客户端约束）；"
                "4) 确认已 rm_init(RM_TRIPLE_MODE_E) 后重跑" % (ip, ip, port)),
            evidence=evidence + ["rm_create_robot_arm handle.id=%s" % handle_id,
                                 "tcp_reachable=%s" % reachable],
            endpoint=endpoint,
        )
    evidence.append("handle.id=%d" % handle_id)

    # 4.5) SDK 超时（建连期接线；失败**不阻断**连接，但必须留下证据）
    timeout_s, timeout_evidence = _load_sdk_timeout_s(ws)
    evidence.extend(timeout_evidence)
    timeout_result = _call_set_timeout(binding, timeout_s)
    evidence.extend(timeout_result.get("evidence") or [])
    if timeout_result.get("status") != errors.STATUS_OK:
        evidence.append(
            "SDK 超时设置失败（**不阻断连接**）：%s" % timeout_result.get("reason")
        )

    # 5) UDP 主动上报（下发配置 + 注册全局回调）
    period = int(realtime_period_ms)
    if enable_realtime:
        if period <= 0 or period % REALTIME_CYCLE_MULTIPLE_MS != 0:
            bridge.close()
            return ArmLink(
                bridge=bridge,
                status=errors.STATUS_ERROR,
                reason="实时上报周期 %d ms 不是 5 的倍数（SDK 要求）" % period,
                next_action="把上报周期改成 5 的倍数（如 5 / 10 ms）后重跑",
                evidence=evidence,
                endpoint=endpoint,
            )
        target_ip = realtime_target_ip or ip
        target_port = int(realtime_target_port or udp_port or port)
        cfg = gen.rm_realtime_push_config_t()
        cfg.cycle = period
        cfg.enable = True
        cfg.port = target_port
        cfg.force_coordinate = 0
        cfg.ip = target_ip.encode("utf-8")[:27]
        cfg.custom_config.arm_current_status = 1
        cfg.custom_config.joint_speed = 1
        try:
            rc = int(binding.rm_set_realtime_push(handle, cfg) or 0)
        except Exception as exc:
            bridge.close()
            return ArmLink(
                bridge=bridge,
                status=errors.STATUS_ERROR,
                reason="rm_set_realtime_push 调用异常：%s" % exc,
                next_action="确认上报目标 IP/端口配置正确（%s:%s）后重跑" % (
                    target_ip, target_port,
                ),
                evidence=evidence + [repr(exc)],
                endpoint=endpoint,
            )
        if rc != 0:
            bridge.close()
            return ArmLink(
                bridge=bridge,
                status=errors.STATUS_ERROR,
                reason="rm_set_realtime_push 返回 %d（配置非法 / 臂不可达）" % rc,
                next_action=(
                    "确认上报目标 IP 可达（%s:%s，须是**运行本进程的主机**地址）、"
                    "周期 %d ms 为 5 的倍数，且臂在线后重跑"
                    % (target_ip, target_port, period)
                ),
                evidence=evidence + ["rm_set_realtime_push rc=%d" % rc],
                endpoint=endpoint,
            )
        evidence.append("realtime_push cycle=%dms target=%s:%d" % (
            period, target_ip, target_port,
        ))
        _register_realtime_callback(binding)

    return ArmLink(
        bridge=bridge,
        handle=handle,
        status=errors.STATUS_OK,
        reason="已连接控制器 %s:%s" % (ip, port),
        next_action="",
        evidence=evidence,
        realtime_period_ms=period if enable_realtime else None,
        endpoint=endpoint,
        sdk_timeout_result=timeout_result,
        realtime_target_ip=(realtime_target_ip or ip) if enable_realtime else None,
        udp_port=int(target_port) if enable_realtime else None,
    )


def _register_realtime_callback(binding: Any) -> bool:
    """注册 UDP 实时上报回调（**进程级全局注册**，只接一路臂）。"""
    global _REALTIME_CB
    if _REALTIME_CB is None:
        def _handler(state: Any) -> None:
            try:
                _REALTIME.push(state)
            except Exception:  # pragma: no cover - 回调内不允许抛异常
                pass

        _REALTIME_CB = gen.rm_realtime_arm_state_callback_ptr(_handler)
    try:
        binding.rm_realtime_arm_state_call_back(_REALTIME_CB)
    except Exception:
        return False
    return True
