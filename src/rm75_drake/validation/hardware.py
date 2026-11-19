"""真机阶梯的**执行器**（把每一级落到真实动作上）。

分层纪律
--------
* 本模块是「阶梯 ↔ 真机」的**唯一**执行面：阶梯定义在
  :mod:`rm75_drake.validation.ladder`，判定逻辑在那里，这里只**产出指标**；
* **零 pydrake 依赖**（TCP 位姿 / MDH 正运动学都不需要 Drake）；
* 任何会动真机的动作都在 :meth:`HardwareExecutor.jog` /
  :meth:`HardwareExecutor.jitter` / :meth:`HardwareExecutor.gripper` 里，
  且**必须**由调用方显式 ``allow_motion`` 授权（阶梯的门禁负责拦截）；
* 真机不可达 ⇒ 抛异常（由阶梯记为 FAIL），**不**伪造指标。
"""

from __future__ import annotations

import math
import socket
import time
import urllib.error
import urllib.request
from typing import Any, Dict, List, Optional, Sequence, Tuple

from ..device.binding import ArmBinding
from ..device.config import DeviceConfig
from ..device.realtime import RealtimePushSettings, RealtimeStateReceiver
from ..device.safety import (
    ACTION_PROCEED,
    SafetyLimits,
    SafetyState,
    apply_safety,
    gate_fence,
    safety_limits_from_sources,
)
from ..device.state_source import StateSource
from ..device.sdk import rad_to_deg
from ..device.transport import SdkTransport, TransportError
from ..kinematics.mdh import link7_position
from ..limits import load_joint_limits
from .ladder import LadderContext, Stage

__all__ = [
    "HardwareExecutor",
    "detect_local_ip",
    "probe_tcp",
    "probe_http",
]


#: TCP 探测超时（s）。
TCP_TIMEOUT_S = 2.0

#: 网页探测超时（s）。
HTTP_TIMEOUT_S = 5.0


def detect_local_ip(target_ip: str, target_port: int = 8080) -> str:
    """探测本机在 ``target_ip`` 所在网段上的**源地址**。

    做法：建一个 UDP socket 并 ``connect`` 到机械臂（不真正发包），
    再读 ``getsockname()`` ⇒ 内核选出的源地址就是机械臂看到的本机地址。
    这是 UDP 主动上报**必填**的「上报目标 IP」（填机械臂自己的 IP 是经典坑）。

    异常
    ----
    OSError
        探测失败（调用方应显式报错，不静默兜底）。
    """
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect((str(target_ip), int(target_port)))
        return str(probe.getsockname()[0])
    finally:
        probe.close()


def probe_tcp(host: str, port: int, timeout: float = TCP_TIMEOUT_S) -> Dict[str, Any]:
    """探测 TCP 指令通道（返回 ``connected`` / ``latency_ms`` / ``detail``）。"""
    started = time.monotonic()
    try:
        with socket.create_connection((str(host), int(port)), timeout=float(timeout)):
            pass
    except OSError as exc:
        return {
            "connected": False,
            "latency_ms": None,
            "detail": f"{type(exc).__name__}: {exc}",
        }
    return {
        "connected": True,
        "latency_ms": (time.monotonic() - started) * 1000.0,
        "detail": "",
    }


def probe_http(host: str, port: int = 80, timeout: float = HTTP_TIMEOUT_S) -> Dict[str, Any]:
    """探测厂商网页（返回 ``status`` / ``detail``）。

    失败**不抛异常**（网页不可达不应阻断整条阶梯），但会如实记录原因。
    """
    url = f"http://{host}:{int(port)}/"
    try:
        with urllib.request.urlopen(url, timeout=float(timeout)) as response:
            return {"status": int(response.status), "detail": url}
    except urllib.error.HTTPError as exc:
        return {"status": int(exc.code), "detail": f"{url}（HTTPError）"}
    except Exception as exc:
        return {"status": None, "detail": f"{url}（{type(exc).__name__}: {exc}）"}


class HardwareExecutor:
    """真机执行器：把阶梯的每一级落到**真实动作**上。

    参数
    ----
    context:
        阶梯上下文（设备配置 / 选项 / 跨阶段共享状态）。
    """

    def __init__(self, context: LadderContext) -> None:
        self.context = context
        self._binding: Optional[ArmBinding] = None
        self._source: Optional[StateSource] = None
        self._receiver: Optional[RealtimeStateReceiver] = None
        self._transport: Optional[SdkTransport] = None

    # -- 入口 ------------------------------------------------------------
    def __call__(self, stage: Stage, context: LadderContext) -> Dict[str, Any]:
        """执行一级阶梯，返回**指标字典**（判定由阶梯负责）。"""
        handler = getattr(self, f"_stage_{stage.executor}", None)
        if handler is None:
            raise RuntimeError(f"未知执行器：{stage.executor!r}")
        return handler(context)

    def close(self) -> None:
        """收尾（停上报 + 关连接）。

        ⚠️ 真机实测：厂商 SDK 的 UDP 接收线程在**解释器退出阶段**仍会回调
        Python ⇒ 可能触发 ``Fatal Python error: Segmentation fault``。
        ⇒ 这里先停主动上报并留一点沉降时间，让接收线程自然停下；
        解释器退出期的崩溃由脚本用 ``os._exit`` 规避（见脚本注释）。
        """
        if self._receiver is not None:
            self._receiver.stop()
            self._receiver = None
            time.sleep(0.2)
        if self._transport is not None:
            self._transport.close()
            self._transport = None
        if self._binding is not None:
            self._binding.close()
            self._binding = None

    # -- 共享资源 --------------------------------------------------------
    @property
    def binding(self) -> ArmBinding:
        """已连接的 SDK 绑定（懒连接，跨阶段复用）。"""
        if self._binding is None:
            binding = ArmBinding(self.context.device_config)
            report = binding.open()
            if not report.connected:
                binding.close()
                raise TransportError(report.detail, reason="link_down")
            self._binding = binding
            self.context.state["connect_report"] = report.to_dict()
        return self._binding

    @property
    def source(self) -> StateSource:
        """UDP 状态源（懒创建）。"""
        if self._source is None:
            self._source = StateSource()
        return self._source

    @property
    def transport(self) -> SdkTransport:
        """复用同一绑定的传输层（懒创建）。"""
        if self._transport is None:
            transport = SdkTransport(self.context.device_config, binding=self.binding)
            transport.open()
            transport.attach_state_source(self.source)
            self._transport = transport
        return self._transport

    def limits(self) -> SafetyLimits:
        """构造安全七闸阈值（软限位 + 设备配置）。"""
        return safety_limits_from_sources(
            load_joint_limits(), self.context.device_config
        )

    # -- L0：链路 --------------------------------------------------------
    def _stage_network(self, context: LadderContext) -> Dict[str, Any]:
        """L0：TCP 指令通道 + 厂商网页可达性。"""
        config = context.device_config
        tcp = probe_tcp(config.arm_ip, config.tcp_port)
        http = probe_http(config.arm_ip, int(context.option("http_port", 80)))
        return {
            "tcp_connected": bool(tcp["connected"]),
            "tcp_latency_ms": tcp["latency_ms"],
            "tcp_detail": tcp["detail"],
            "http_status": http["status"],
            "http_detail": http["detail"],
            "arm_ip": config.arm_ip,
            "tcp_port": int(config.tcp_port),
        }

    # -- L1：SDK 与能力 --------------------------------------------------
    def _stage_sdk(self, context: LadderContext) -> Dict[str, Any]:
        """L1：SDK 可用性 + 控制器能力（连接只读，不下发任何指令）。"""
        from ..device.sdk import load_sdk, sdk_available

        available = sdk_available()
        metrics: Dict[str, Any] = {"sdk_available": bool(available)}
        if not available:
            raise RuntimeError(
                "厂商 SDK 不可用（分发名 robotic-arm，导入名 Robotic_Arm）"
            )
        load_sdk()
        report = self.binding.open()
        metrics.update(
            {
                "connected": bool(report.connected),
                "arm_dof": report.arm_dof,
                "controller_version": report.controller_version,
                "connect_detail": report.detail,
            }
        )
        metrics["controller_version"] = report.controller_version
        return metrics

    # -- L2：连接与只读状态 ----------------------------------------------
    def _stage_connect_state(self, context: LadderContext) -> Dict[str, Any]:
        """L2：只读状态快照（关节角 / 位姿 / 电流 / 错误码）。"""
        transport = self.transport
        frame = transport.read_arm_state()
        raw = self.binding.call("rm_get_current_arm_state")[1]
        pose = list(raw.get("pose") or [])
        metrics: Dict[str, Any] = {
            "connected": True,
            "state_query_ok": True,
            "joint_positions_rad": [round(v, 6) for v in frame.joint_positions_rad],
            "joint_positions_deg": [
                round(rad_to_deg(v), 3) for v in frame.joint_positions_rad
            ],
            "joint_err_codes": list(frame.joint_err_codes),
            "joint_error_count": sum(1 for c in frame.joint_err_codes if int(c) != 0),
            "tcp_pose": pose[:6],
        }
        if len(pose) >= 3:
            metrics["tcp_height_m"] = float(pose[2])
            metrics["tcp_radius_m"] = math.hypot(float(pose[0]), float(pose[1]))
        try:
            currents = transport.read_joint_currents_ma()
            metrics["joint_currents_ma"] = [round(v, 1) for v in currents]
        except TransportError as exc:
            metrics["joint_currents_ma"] = None
            metrics["currents_error"] = str(exc)
        context.state["arm_state"] = metrics
        return metrics

    # -- L3：UDP 主动上报 ------------------------------------------------
    def _stage_realtime(self, context: LadderContext) -> Dict[str, Any]:
        """L3：开启主动上报并**实测**周期（定死 cycle 读法）。"""
        config = context.device_config
        target_ip = context.option("push_target_ip") or detect_local_ip(
            config.arm_ip, config.tcp_port
        )
        settings = RealtimePushSettings(
            ip=str(target_ip),
            port=int(config.udp_port),
            cycle=int(config.udp_cycle),
        )
        receiver = RealtimeStateReceiver(self.binding, self.source, settings)
        receiver.start()
        self._receiver = receiver
        duration = float(context.option("realtime_seconds", 3.0))
        time.sleep(duration)
        stats = receiver.period_stats()
        expected = settings.expected_period_ms()
        metrics: Dict[str, Any] = {
            "push_target_ip": str(target_ip),
            "udp_port": int(config.udp_port),
            "udp_cycle": int(config.udp_cycle),
            "expected_period_ms": expected,
            "incomplete_frames": int(self.source.frames_rejected_incomplete),
            "invalid_frames": int(self.source.frames_rejected_invalid),
        }
        metrics.update(stats.to_dict())
        metrics["period_match"] = bool(
            stats.frames >= 2
            and expected > 0
            and abs(stats.p50 - expected) / expected <= float(
                context.option("period_tolerance", 0.10)
            )
        )
        metrics["cycle_reading"] = (
            "A（cycle 单位 = 5 ms 的倍数）"
            if metrics["period_match"]
            else "与期望周期不符（需复核 cycle 读法或网络）"
        )
        context.state["realtime"] = metrics
        return metrics

    # -- L4：使能标志 ----------------------------------------------------
    def _stage_enable_check(self, context: LadderContext) -> Dict[str, Any]:
        """L4：逐轴使能标志 / 错误码（来自 UDP 帧）。"""
        frame = self.source.latest()
        if frame is None:
            return {
                "en_flags_received": False,
                "all_axes_enabled": False,
                "joint_error_count": None,
                "detail": "没有收到 UDP 状态帧（先跑通 L3）",
            }
        flags = [bool(f) for f in frame.joint_en_flags]
        metrics = {
            "en_flags_received": True,
            "joint_en_flags": flags,
            "enabled_axis_count": sum(1 for f in flags if f),
            "all_axes_enabled": bool(len(flags) == 7 and all(flags)),
            "joint_err_codes": list(frame.joint_err_codes),
            "joint_error_count": sum(1 for c in frame.joint_err_codes if int(c) != 0),
            "arm_current_status": int(frame.arm_current_status),
            "gripper_online": bool(frame.gripper_online),
        }
        context.state["enable"] = metrics
        return metrics

    # -- L5：运动前静态安全自检 ------------------------------------------
    def _stage_static_safety(self, context: LadderContext) -> Dict[str, Any]:
        """L5：软限位 + 几何围栏（用关节角算 TCP，零 Drake 依赖）。"""
        config = context.device_config
        limits = self.limits()
        state = context.state.get("arm_state") or {}
        positions = state.get("joint_positions_rad")
        if not positions:
            frame = self.source.latest()
            if frame is None:
                raise RuntimeError("没有状态数据（先跑 L2 / L3）")
            positions = list(frame.joint_positions_rad)
        q = tuple(float(v) for v in positions)
        violations = gate_fence(q, limits)
        lower = limits.soft_lower_rad
        upper = limits.soft_upper_rad
        out_of_range = [
            index
            for index, value in enumerate(q)
            if value < lower[index] - 1e-9 or value > upper[index] + 1e-9
        ]
        position = link7_position(q)
        metrics = {
            "joints_within_soft_limits": not out_of_range,
            "out_of_range_joints": out_of_range,
            "fence_violations": len(violations),
            "fence_details": [v.detail for v in violations],
            "link7_position_m": [round(float(v), 5) for v in position],
            "tcp_height_m": round(float(position[2]), 5),
            "tcp_radius_m": round(math.hypot(float(position[0]), float(position[1])), 5),
            "table_height_m": float(config.fence_table_height_m),
            "singularity_radius_m": float(config.fence_singularity_radius_m),
        }
        # 围栏余量：TCP 离桌面 / 离奇异圆柱还有多少（**余量太小就别动**）。
        metrics["fence_height_margin_m"] = round(
            float(position[2]) - float(config.fence_table_height_m), 5
        )
        metrics["fence_radius_margin_m"] = round(
            math.hypot(float(position[0]), float(position[1]))
            - float(config.fence_singularity_radius_m),
            5,
        )
        context.state["static_safety"] = metrics
        return metrics

    # -- L6：单轴点动 ----------------------------------------------------
    def _stage_jog(self, context: LadderContext) -> Dict[str, Any]:
        """L6：单轴 5° 点动 → 反馈校验 → 回原位（**会让机械臂动**）。"""
        transport = self.transport
        limits = self.limits()
        frame = self.source.latest()
        if frame is None:
            raise RuntimeError("没有状态帧（先跑 L3）")
        origin = tuple(float(v) for v in frame.joint_positions_rad)
        axis = int(context.option("jog_axis", 0))
        delta = float(context.option("jog_delta_rad", rad_to_deg(5.0)))
        hold_s = float(context.option("jog_hold_s", 0.8))
        target = list(origin)
        target[axis] = origin[axis] + delta
        if not (
            limits.soft_lower_rad[axis] <= target[axis] <= limits.soft_upper_rad[axis]
        ):
            raise RuntimeError(
                f"点动目标越界：关节 {axis} 目标 {target[axis]:.4f} rad "
                f"不在软限位 [{limits.soft_lower_rad[axis]:.4f}, "
                f"{limits.soft_upper_rad[axis]:.4f}] 内"
            )
        _hold(transport, tuple(target), hold_s)
        reached = _feedback(transport, self.source)
        jog_error = rad_to_deg(abs(reached[axis] - target[axis]))
        _hold(transport, origin, hold_s)
        back = _feedback(transport, self.source)
        return_error = rad_to_deg(abs(back[axis] - origin[axis]))
        frame = self.source.latest()
        error_count = (
            sum(1 for c in frame.joint_err_codes if int(c) != 0) if frame else None
        )
        return {
            "jog_axis": axis,
            "jog_delta_deg": rad_to_deg(delta),
            "origin_deg": [round(rad_to_deg(v), 3) for v in origin],
            "target_deg": [round(rad_to_deg(v), 3) for v in target],
            "reached_deg": [round(rad_to_deg(v), 3) for v in reached],
            "back_deg": [round(rad_to_deg(v), 3) for v in back],
            "jog_error_deg": jog_error,
            "return_error_deg": return_error,
            "joint_error_count": error_count,
        }

    # -- L7：5 ms 透传抖动 -----------------------------------------------
    def _stage_jitter(self, context: LadderContext) -> Dict[str, Any]:
        """L7：5 ms 节拍保持位姿，统计抖动（口径复用设备层唯一实现）。"""
        from ..device.measure_support import (
            measurement_summary,
            parse_joint_rad,
            run_hold_measurement,
        )

        pose = context.option("joint_rad")
        if isinstance(pose, str):
            pose = parse_joint_rad(pose)
        if not pose:
            raise RuntimeError(
                "真机抖动测量必须显式给出保持位姿（--joint-rad），脚本不自己编造目标"
            )
        pose = tuple(float(v) for v in pose)
        limits = self.limits()
        transport = self.transport
        result = run_hold_measurement(
            transport=transport,
            joint_rad=pose,
            limits=limits,
            period_s=float(context.option("period_s", 0.005)),
            duration_s=float(context.option("jitter_duration_s", 10.0)),
            require_state=False,
        )
        summary = measurement_summary(
            result, joint_rad=pose, period_s=float(context.option("period_s", 0.005))
        )
        jitter = summary["jitter_ms"]
        metrics = {
            "jitter_p50_ms": jitter.get("p50"),
            "jitter_p99_ms": jitter.get("p99"),
            "jitter_p999_ms": jitter.get("p999"),
            "jitter_max_ms": jitter.get("max"),
            "num_violations": summary["num_violations"],
            "num_errors": summary["num_errors"],
            "final_state": summary["final_state"],
            "ticks": summary["ticks"],
        }
        context.state["jitter"] = metrics
        return metrics

    # -- L9：夹爪 --------------------------------------------------------
    def _stage_gripper(self, context: LadderContext) -> Dict[str, Any]:
        """L9：夹爪张开 / 闭合，并回读实际开口（**会让夹爪动**）。"""
        transport = self.transport
        open_target = int(context.option("gripper_open", 1000))
        close_target = int(context.option("gripper_close", 0))
        transport.command_gripper_position(open_target)
        time.sleep(float(context.option("gripper_settle_s", 1.0)))
        open_state = transport.read_gripper()
        transport.command_gripper_position(close_target)
        time.sleep(float(context.option("gripper_settle_s", 1.0)))
        close_state = transport.read_gripper()
        return {
            "gripper_online": bool(open_state.online and close_state.online),
            "open_target": open_target,
            "open_readback": int(open_state.position),
            "open_error": abs(int(open_state.position) - open_target),
            "close_target": close_target,
            "close_readback": int(close_state.position),
            "close_error": abs(int(close_state.position) - close_target),
        }

    # -- L10：安全七闸逐闸验证 -------------------------------------------
    def _stage_safety_gates(self, context: LadderContext) -> Dict[str, Any]:
        """L10：离线替身逐闸注入 + 真机围栏拒绝（**真机部分不下发指令**）。"""
        limits = self.limits()
        results = _inject_gates(limits)
        blocked = sum(1 for item in results.values() if item["blocked"])
        metrics: Dict[str, Any] = {
            "gates_checked": len(results),
            "gates_blocked": blocked,
            "gates": results,
        }
        # 真机部分：构造一个「低于桌面」的姿态，确认安全层会拒绝（不下发）。
        low_pose = _low_pose(limits)
        violations = gate_fence(low_pose, limits)
        verdict = apply_safety(
            low_pose,
            prev_command=low_pose,
            limits=limits,
            state=SafetyState(),
            dt_s=limits.cycle_period_s,
        )
        metrics["real_fence_rejected"] = bool(violations) and (
            verdict.action != ACTION_PROCEED
        )
        metrics["real_fence_detail"] = [v.detail for v in violations]
        context.state["safety_gates"] = metrics
        return metrics

    # -- L13：第二台臂可达性 ---------------------------------------------
    def _stage_dual_arm(self, context: LadderContext) -> Dict[str, Any]:
        """L13：第二台臂可达性探测（**只探测，不动作**）。"""
        second_ip = str(context.option("second_arm_ip", "192.168.1.19"))
        second_port = int(context.option("second_arm_port", 8080))
        probe = probe_tcp(second_ip, second_port)
        return {
            "dual_arm_reachable": bool(probe["connected"]),
            "second_arm_ip": second_ip,
            "second_arm_detail": probe["detail"],
        }


# ── 工具函数 ────────────────────────────────────────────────────────────
def _hold(transport: SdkTransport, target: Sequence[float], seconds: float) -> None:
    """持续下发保持位姿（透传 + follow），持续 ``seconds`` 秒。

    真机的透传指令是「保持到下一个指令」⇒ 连续下发几次可抗丢包；
    这里按 50 ms 间隔重发，节拍远低于 5 ms 执行环（**不与执行环混用**）。
    """
    deadline = time.monotonic() + float(seconds)
    while time.monotonic() < deadline:
        transport.move_j_canfd(tuple(float(v) for v in target), follow=True)
        time.sleep(0.05)


def _feedback(
    transport: SdkTransport, source: StateSource, timeout_s: float = 1.0
) -> Tuple[float, ...]:
    """等待一帧**新**反馈并返回关节角（rad）。

    没有新帧 ⇒ 抛 :class:`TransportError`（**不拿命令回读冒充实测值**）。
    """
    latest = source.latest()
    baseline = latest.received_at if latest is not None else -1.0
    deadline = time.monotonic() + float(timeout_s)
    while time.monotonic() < deadline:
        current = source.latest()
        if current is not None and current.received_at > baseline:
            return tuple(float(v) for v in current.joint_positions_rad)
        time.sleep(0.02)
    raise TransportError(
        "等待 UDP 反馈超时（没有新帧）——不得用命令回读冒充实测值",
        reason="state_unavailable",
    )


def _low_pose(limits: SafetyLimits) -> Tuple[float, ...]:
    """构造一个「低于桌面」的姿态（用于验证围栏闸会拒绝它）。"""
    pose = [0.0] * 7
    pose[1] = 0.9  # 肩关节前伸 → TCP 落到桌面以下
    pose[3] = 0.4
    return tuple(pose)


def _inject_gates(limits: SafetyLimits) -> Dict[str, Dict[str, Any]]:
    """离线逐闸注入（**纯函数**，不需要真机）。

    基线位姿取一个**围栏合法**的位姿（否则每一闸都会额外报 ``fence``，
    注入信号会被噪声淹没）。

    返回 ``{闸号: {"reason": ..., "blocked": bool}}``。
    """
    from ..device.measure_support import default_hold_pose

    base = default_hold_pose(limits)
    dt = limits.cycle_period_s
    out: Dict[str, Dict[str, Any]] = {}

    def _record(gate: str, verdict: Any, expected_reason: str) -> None:
        reasons = [v.reason for v in verdict.violations]
        out[gate] = {
            "expected_reason": expected_reason,
            "reasons": reasons,
            "action": verdict.action,
            "blocked": bool(expected_reason in reasons),
        }

    # G1 软限位：目标超出软限位上限。
    over = list(base)
    over[0] = limits.soft_upper_rad[0] + 0.5
    verdict = apply_safety(
        over, prev_command=base, limits=limits, state=SafetyState(), dt_s=dt
    )
    _record("G1", verdict, "soft_limit")

    # G2 单步增量：一步跳 1 rad（远超 ω_max·dt）。
    jump = list(base)
    jump[0] = base[0] + 1.0
    verdict = apply_safety(
        jump, prev_command=base, limits=limits, state=SafetyState(), dt_s=dt
    )
    _record("G2", verdict, "delta_clamp")

    # G3 加速度：上一拍速度很大，本拍继续加速。
    fast = list(base)
    fast[0] = base[0] + 0.05
    verdict = apply_safety(
        fast,
        prev_command=base,
        limits=limits,
        state=SafetyState(),
        prev_velocity_rad_s=(2.0,) + base[1:],
        dt_s=dt,
    )
    _record("G3", verdict, "accel_clamp")

    # G4 看门狗：状态 age 超过看门狗阈值（按周期数折算）。
    watchdog_s = float(limits.watchdog_warn_cycles + 1) * limits.cycle_period_s
    verdict = apply_safety(
        base,
        prev_command=base,
        limits=limits,
        state=SafetyState(state_age_s=watchdog_s),
        dt_s=dt,
    )
    _record("G4", verdict, "watchdog")

    # G5 电流：逐轴电流超阈值。
    verdict = apply_safety(
        base,
        prev_command=base,
        limits=limits,
        state=SafetyState(
            joint_currents_ma=tuple(
                limit * 2.0 for limit in limits.current_threshold_ma
            )
        ),
        dt_s=dt,
    )
    _record("G5", verdict, "current")

    # G6 围栏：TCP 低于桌面。
    low = _low_pose(limits)
    verdict = apply_safety(
        low, prev_command=low, limits=limits, state=SafetyState(), dt_s=dt
    )
    _record("G6", verdict, "fence")

    # G7 状态 age 超时。
    verdict = apply_safety(
        base,
        prev_command=base,
        limits=limits,
        state=SafetyState(state_age_s=limits.state_age_timeout_s * 4.0),
        dt_s=dt,
    )
    _record("G7", verdict, "state_age")

    return out
