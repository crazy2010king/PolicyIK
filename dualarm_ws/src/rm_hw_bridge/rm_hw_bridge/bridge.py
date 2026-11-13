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
import os
import threading
import time
from typing import Any, Dict, List, Optional

from rm_hw_bridge import errors, loader, paths
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
    "REALTIME_CYCLE_MULTIPLE_MS",
    "connect",
    "realtime_state",
]

#: UDP 主动上报周期必须是 5 ms 的倍数（头文件注释 + network_plan 基线）
REALTIME_CYCLE_MULTIPLE_MS = 5

#: 默认 TCP 端口
DEFAULT_TCP_PORT = 8080


# --------------------------------------------------------------- 实时上报状态
class _RealtimeState:
    """进程级实时上报状态（``rm_realtime_arm_state_call_back`` 是全局注册）。"""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.frames = 0
        self.last_ts: Optional[float] = None
        self.arm_current_status: Optional[int] = None
        self.last_joint_status: Optional[Any] = None

    def push(self, state: Any) -> None:
        with self._lock:
            self.frames += 1
            self.last_ts = time.time()
            self.arm_current_status = int(getattr(state, "arm_current_status", 0) or 0)
            self.last_joint_status = getattr(state, "joint_status", None)

    def snapshot(self, period_ms: Optional[int]) -> Dict[str, Any]:
        with self._lock:
            return {
                "frames": self.frames,
                "last_ts": self.last_ts,
                "period_ms": period_ms,
            }


#: 进程内唯一的实时上报状态（回调是全局注册，天然单例）
_REALTIME = _RealtimeState()
#: 已注册的回调对象（必须保持引用，否则被 GC 后 SDK 调用即崩）
_REALTIME_CB = None


def realtime_state() -> _RealtimeState:
    """进程级实时上报状态（只读访问）。"""
    return _REALTIME


# --------------------------------------------------------------------- 工具
def _handle_id(handle: Any) -> Optional[int]:
    """读句柄 id（真实指针在 ``handle.contents.id``；假 SDK 直接暴露 ``.id``）。"""
    return loader._handle_id(handle)


def _err_flag_to_list(err_flag: int) -> List[int]:
    """关节错误位标志 → 每关节错误码列表（bit i 对应关节 i+1）。"""
    return [(int(err_flag) >> i) & 1 for i in range(ARM_DOF)]


def _brake_to_list(brake_state: int) -> List[bool]:
    """抱闸状态位标志 → 每关节 ``True=抱闸未打开``（头文件：1 代表抱闸未打开）。"""
    return [bool((int(brake_state) >> i) & 1) for i in range(ARM_DOF)]


def _find_sdk(ws: Optional[str] = None, env: Optional[Dict[str, str]] = None):
    """找 SDK；找不到返回 ``(None, 候选路径列表)``。"""
    candidates = loader.sdk_candidates(ws=ws, env=env)
    path = loader.find_library(ws=ws, env=env)
    return path, candidates


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
    ) -> None:
        self._bridge = bridge
        self._handle = handle
        self.status = status
        self.reason = reason
        self.next_action = next_action
        self.evidence = list(evidence or [])
        self.realtime_period_ms = realtime_period_ms
        self.endpoint = dict(endpoint or {})
        self._closed = False

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
        return ArmLink(
            bridge=bridge,
            status=errors.STATUS_ERROR,
            reason="rm_create_robot_arm 返回空指针（连接数达上限 5 或参数非法）",
            next_action="确认本进程没有残留的旧连接（最多 5 个），"
                        "并核对 ip=%s port=%s 后重跑" % (ip, port),
            evidence=evidence + ["rm_create_robot_arm -> NULL"],
            endpoint=endpoint,
        )
    handle_id = _handle_id(handle)
    if handle_id is None or handle_id < 0:
        return ArmLink(
            bridge=bridge,
            status=errors.STATUS_ERROR,
            reason="rm_create_robot_arm 返回无效句柄 id=%s（控制器不可达）" % handle_id,
            next_action=(
                "1) ping %s 确认控制器在线；"
                "2) 确认 %s:%s 的 TCP 端口可达；"
                "3) 确认同一 ip:port 没有被其它客户端占用（单客户端约束）；"
                "4) 确认已 rm_init(RM_TRIPLE_MODE_E) 后重跑"
                % (ip, ip, port)
            ),
            evidence=evidence + ["rm_create_robot_arm handle.id=%s" % handle_id],
            endpoint=endpoint,
        )
    evidence.append("handle.id=%d" % handle_id)

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
