"""真机**单连接、多段执行**会话（执行层）。

设计要点（与切片契约一致）：

* **一条连接、连续多段**：:class:`LiveSession` 持有一条 ``ArmLink`` +
  一个 :class:`~rm_executor.transport.BridgeTransport` + 一个
  :class:`~rm_executor.feedback.FeedbackPump`，可**反复**调用
  :meth:`LiveSession.run_trajectory`（home→approach→lift→place→home…），
  **不重复建链**；
* **验收口径唯一**：``rm_executor/cli.py::_run_live()`` **复用**本模块的
  :meth:`LiveSession.open`，因此 CLI 与工艺层（``rmctl.process``）**共用
  同一套真机建连语义**，不会出现「两套真机真相」；
* **反馈泵修复**：旧 live 路径构造了 ``FeedbackPump`` 却**从未调用**
  ``start()``（真机反馈缓存恒为空）。本模块在 :meth:`LiveSession.open`
  里启动反馈泵、:meth:`LiveSession.close` 里停止它（两者均幂等）；
* **诚实降级**：控制器不可达（硬件层不可用 / 未提供 ip / 建链异常 /
  ``link.connected is False``）→ 返回 ``status="error"`` + reason 含
  「控制器不可达」+ **指标全 None**（零伪造数值）+ 可粘贴 next_action，
  **不抛裸异常**；
* **真机拒绝未校验轨迹**：live 路径 ``allow_unverified=True`` → blocked。

**零 rmctl 依赖、零 tesseract 依赖**（守 ``executor_must_not_import``
红线）；硬件层（``rm_hw_bridge``）只在需要时惰性导入。
"""
from __future__ import annotations

from typing import Any, Dict, List, Optional, Tuple

from rm_executor import contract
from rm_executor import report as report_mod
from rm_executor.clock import ManualClock, TickClock
from rm_executor.executor import Executor, ExecutorConfig
from rm_executor.feedback import ScriptedFeedback
from rm_executor.transport import FakeTransport

__all__ = [
    "DEFAULT_PERIOD_S",
    "FAKE_CALL_RECORD_LIMIT",
    "LIVE_FEEDBACK_PERIOD_S",
    "FakeSession",
    "LiveSession",
    "mirror_feedback",
]

#: 默认栅格周期（秒）—— 10 ms 单时钟
DEFAULT_PERIOD_S = 0.010

#: 真机反馈采集周期（秒）—— 落在计划 §5.7 的 10–20 ms 区间
LIVE_FEEDBACK_PERIOD_S = 0.015

#: 离线假传输的**调用记录保留上限**（长跑时防内存无界增长；计数不受影响）
FAKE_CALL_RECORD_LIMIT = 2000

#: 「硬件层不可用」时的可粘贴下一步（固定文案，便于检索）
NEXT_ACTION_HW_BRIDGE_PATH = (
    "export PYTHONPATH=$PWD/dualarm_ws/src/rm_hw_bridge:$PYTHONPATH  "
    "# 让执行层看到硬件适配层"
)

#: 真机路径拒绝未校验轨迹时的可粘贴下一步
NEXT_ACTION_PLAN_FIRST = (
    "先用 `python3 -m rmctl.cli plan ... --out <traj.json>` 生成带 verify "
    "报告的轨迹，再在真机路径执行"
)


# --------------------------------------------------------------------- 工具
def mirror_feedback(transport: FakeTransport, period_s: float) -> ScriptedFeedback:
    """构造「镜像最近一次下发指令角」的脚本化反馈（理想跟随模型）。

    * 预就位阶段：镜像 ``preposition`` 的首点 → 预就位必然到位；
    * 流式阶段：镜像每一拍的 ``transmit`` 指令 → 跟随误差恒为 0；
    * 速度恒为 0 → 到位判定的速度条件成立。

    **这是离线代理模型，不是真机结论**（与 ``rm_executor.cli`` 的既有
    口径一致，唯一实现在本模块，CLI 与会话共用）。
    """
    def _script(_index: int) -> Optional[List[float]]:
        call = transport.last_call("transmit") or transport.last_call("preposition")
        if call is None:
            return None  # 尚未下发任何指令 → 断流（不伪造快照）
        return list(call["args"]["deg7"])

    return ScriptedFeedback(script=_script, speed_deg_s=0.0, period_s=period_s)


def _failure_report(status: str, reason: str, next_action: str,
                    evidence: Optional[List[str]], mode: str,
                    events: Optional[List[Dict[str, Any]]] = None) -> Dict[str, Any]:
    """构造失败报告（**不伪造任何数值**：指标全空）。"""
    return report_mod.build_report(
        status=status,
        reason=reason,
        next_action=next_action,
        evidence=evidence or [],
        mode=mode,
        events=events or [],
    )


def _unreachable_report(reason: str, next_action: str,
                        evidence: Optional[List[str]]) -> Dict[str, Any]:
    """控制器不可达 → error 报告（退出码 4，**零伪造数值**）。"""
    event = {
        "type": "controller_unreachable",
        "severity": "FATAL",
        "t": 0.0,
        "reason": reason,
        "evidence": list(evidence or []),
        "next_action": next_action,
    }
    return _failure_report(contract.STATUS_ERROR, reason, next_action,
                           evidence, "live", [event])


# --------------------------------------------------------------- 真机会话
class LiveSession:
    """一次建链、连续多段执行的真机会话。

    参数
    ----
    link
        已建立的 ``ArmLink``（可注入假对象做离线测试）。
    transport
        透传通道；``None`` → 由 :meth:`open` 构造 ``BridgeTransport(link)``。
    feedback
        反馈读取器；``None`` → 由 :meth:`open` 构造 ``make_live_feedback()``。
    config
        执行配置；``None`` → 执行器自己按 ``ws`` 读工位文件（唯一来源）。
    ws
        工作区根。
    arm
        选哪条臂（``left`` / ``right``）。
    ip / port / udp_port
        控制器地址（惰性建连时使用）。
    clock
        主时钟；``None`` → ``TickClock``（真机墙钟）。
    """

    def __init__(self,
                 link: Any = None,
                 transport: Any = None,
                 feedback: Any = None,
                 config: Optional[ExecutorConfig] = None,
                 ws: Optional[str] = None,
                 arm: str = "left",
                 ip: Optional[str] = None,
                 port: Optional[int] = None,
                 udp_port: Optional[int] = None,
                 clock: Any = None) -> None:
        self.link = link
        self.transport = transport
        self.feedback = feedback
        self.config = config
        self.ws = ws
        self.arm = arm or "left"
        self.ip = ip
        self.port = port
        self.udp_port = udp_port
        self.clock = clock
        self._closed = False
        #: 夹爪端口构造失败的证据（``gripper_port()`` 返回 None 时说明原因）
        self.gripper_port_evidence: List[str] = []
        #: 反馈泵启动失败的证据（正常为空）
        self.feedback_error: str = ""

    # -- 建立 --------------------------------------------------------------
    @classmethod
    def open(cls,
             arm: str = "left",
             ip: Optional[str] = None,
             port: Optional[int] = None,
             udp_port: Optional[int] = None,
             ws: Optional[str] = None,
             config: Optional[ExecutorConfig] = None,
             link: Any = None,
             transport: Any = None,
             feedback: Any = None,
             clock: Any = None) -> Tuple[Optional["LiveSession"], Optional[Dict[str, Any]]]:
        """建立会话。

        返回 ``(session, error_report)``：

        * 成功 → ``(session, None)``，**反馈泵已启动**；
        * 失败 → ``(None, error_report)``，报告 ``status="error"``、
          reason 含「控制器不可达」（或反馈泵启动失败）、**指标全 None**、
          next_action 可粘贴。

        **不抛裸异常**：所有失败路径都变成可粘贴的错误报告。
        """
        if link is None:
            try:
                from rm_hw_bridge import bridge as hw_bridge
            except Exception as exc:  # noqa: BLE001 - 硬件层不可用 → 诚实降级
                return None, _unreachable_report(
                    "控制器不可达：硬件适配层 rm_hw_bridge 不可用（%s）: %s"
                    % (type(exc).__name__, exc),
                    NEXT_ACTION_HW_BRIDGE_PATH,
                    [repr(exc)])

            if not ip:
                return None, _unreachable_report(
                    "控制器不可达：未提供控制器 ip（--ip 必填）",
                    "加 --ip <控制器IP> --port <TCP端口> 重跑"
                    "（ip 也可从 workcell/baselines/network_plan.yaml 取）",
                    ["未提供 --ip"])

            tcp_port = int(port or getattr(hw_bridge, "DEFAULT_TCP_PORT", 8080))
            try:
                link = hw_bridge.connect(ip, tcp_port,
                                         udp_port=int(udp_port) if udp_port else None,
                                         arm=arm)
            except Exception as exc:  # noqa: BLE001 - 建链异常 → 诚实降级
                return None, _unreachable_report(
                    "控制器不可达：建立连接异常（%s）: %s" % (type(exc).__name__, exc),
                    "确认控制器已上电、网络可达（ip=%s port=%s）后重跑" % (ip, tcp_port),
                    [repr(exc)])

            if not getattr(link, "connected", False):
                reason = getattr(link, "reason", "") or "ArmLink 未连接"
                next_action = getattr(link, "next_action", "") or (
                    "确认控制器已上电、网络可达（ip=%s port=%s）后重跑"
                    % (ip, tcp_port))
                evidence = list(getattr(link, "evidence", []) or [])
                evidence.append("ArmLink.connected = False")
                return None, _unreachable_report("控制器不可达：%s" % reason,
                                                 next_action, evidence)

        if transport is None:
            from rm_executor.transport import BridgeTransport
            transport = BridgeTransport(link=link)
        if feedback is None:
            from rm_executor.feedback import make_live_feedback
            feedback = make_live_feedback(period_s=LIVE_FEEDBACK_PERIOD_S)

        session = cls(link=link, transport=transport, feedback=feedback,
                      config=config, ws=ws, arm=arm, ip=ip, port=port,
                      udp_port=udp_port, clock=clock)
        if not session.start_feedback():
            session.close()
            return None, _failure_report(
                contract.STATUS_ERROR,
                "真机反馈泵启动失败：%s" % session.feedback_error,
                "确认真机环境可创建反馈线程后重跑（查看终端栈信息）",
                list(session.gripper_port_evidence) or [session.feedback_error],
                "live")
        return session, None

    # -- 反馈泵 ------------------------------------------------------------
    def start_feedback(self) -> bool:
        """启动反馈泵（**幂等**）；失败返回 False 并留证据。

        **这是 S03 遗留缺陷的修复点**：旧 live 路径构造了 ``FeedbackPump``
        却从未 ``start()``，真机反馈缓存恒为空（跟随误差/断流判据永远拿不到
        真实快照）。
        """
        pump = self.feedback
        if pump is None:
            self.feedback_error = "反馈泵缺失（feedback=None）"
            return False
        starter = getattr(pump, "start", None)
        if starter is None:
            self.feedback_error = "反馈对象没有 start()（不是 FeedbackPump）"
            return False
        try:
            starter()
        except Exception as exc:  # noqa: BLE001 - 启动失败 → 诚实降级
            self.feedback_error = "%s: %s" % (type(exc).__name__, exc)
            return False
        return True

    # -- 执行 --------------------------------------------------------------
    def run_trajectory(self, trajectory_path: str,
                       allow_unverified: bool = False,
                       gate: Any = None,
                       motion_gate: Any = None) -> Dict[str, Any]:
        """执行一条轨迹，返回**与 fake 同一 schema** 的统一报告。

        * **live 路径拒绝** ``allow_unverified=True`` → blocked + 明确 reason；
        * 会话未建链时**惰性建链**（工艺层直接构造 ``LiveSession`` 的情形），
          建链失败 → 诚实降级报告（**不抛裸异常**）。
        """
        if allow_unverified:
            return _failure_report(
                contract.STATUS_BLOCKED,
                "真机路径不接受未校验轨迹（allow_unverified=True 仅限离线假执行）",
                NEXT_ACTION_PLAN_FIRST,
                ["allow_unverified=True 在真机会话被拒绝"], "live")

        if self.link is None or self.transport is None:
            session, err = LiveSession.open(
                arm=self.arm, ip=self.ip, port=self.port,
                udp_port=self.udp_port, ws=self.ws, config=self.config)
            if err is not None or session is None:
                return err or _failure_report(
                    contract.STATUS_ERROR, "真机会话建立失败",
                    "确认控制器已上电、网络可达后重跑", [], "live")
            self.link = session.link
            self.transport = session.transport
            self.feedback = session.feedback
            self.clock = session.clock

        clock = self.clock or TickClock(period_s=DEFAULT_PERIOD_S)
        executor = Executor(self.transport, self.feedback, config=self.config,
                            clock=clock, mode="live", ws=self.ws, gate=gate,
                            motion_gate=motion_gate)
        try:
            return executor.run(trajectory_path, allow_unverified=False)
        except Exception as exc:  # noqa: BLE001 - 兜底：异常 → error 报告
            return _failure_report(
                contract.STATUS_ERROR,
                "执行器异常终止（%s）: %s" % (type(exc).__name__, exc),
                "查看报告 evidence 与终端栈信息后重跑",
                [repr(exc)], "live")

    # -- 夹爪 --------------------------------------------------------------
    def gripper_port(self) -> Any:
        """构造夹爪真机端口（与机械臂**共用同一条连接**）。

        返回 ``rm_hw_bridge.gripper.CtypesSdkPort(binding, handle)``：
        ``binding`` 取自 ``link.bridge.binding``、``handle`` 取自
        ``link.handle`` → 夹爪与机械臂走**同一条 TCP 连接**。

        硬件层不可用 / 句柄缺失 → 返回 ``None`` 并在
        :attr:`gripper_port_evidence` 写明原因，**不抛裸异常**。
        """
        self.gripper_port_evidence = []
        link = self.link
        if link is None:
            self.gripper_port_evidence.append("会话尚未建链（link=None）")
            return None
        try:
            from rm_hw_bridge import gripper as hw_gripper
        except Exception as exc:  # noqa: BLE001 - 硬件层不可用 → None + 证据
            self.gripper_port_evidence.append(
                "硬件适配层 rm_hw_bridge.gripper 不可用（%s）: %s"
                % (type(exc).__name__, exc))
            return None
        bridge = getattr(link, "bridge", None)
        binding = getattr(bridge, "binding", None)
        handle = getattr(link, "handle", None)
        if binding is None or handle is None:
            self.gripper_port_evidence.append(
                "ArmLink 缺少 binding/handle（binding=%s, handle=%s）"
                % (binding is not None, handle is not None))
            return None
        try:
            return hw_gripper.CtypesSdkPort(binding, handle)
        except Exception as exc:  # noqa: BLE001 - 构造失败 → None + 证据
            self.gripper_port_evidence.append(
                "构造 CtypesSdkPort 失败（%s）: %s" % (type(exc).__name__, exc))
            return None

    # -- 释放 --------------------------------------------------------------
    def close(self) -> None:
        """停反馈泵 + 关连接（**幂等**）。"""
        if self._closed:
            return
        self._closed = True
        pump = self.feedback
        stopper = getattr(pump, "stop", None) if pump is not None else None
        if stopper is not None:
            try:
                stopper()
            except Exception:  # pragma: no cover - 释放失败不影响主流程
                pass
        link = self.link
        if link is not None:
            try:
                link.close()
            except Exception:  # pragma: no cover - 释放失败不影响主流程
                pass

    @property
    def closed(self) -> bool:
        """会话是否已关闭。"""
        return self._closed

    def __enter__(self) -> "LiveSession":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()


# --------------------------------------------------------------- 离线会话
class FakeSession:
    """离线等价会话（**零真机依赖**）。

    与 :class:`LiveSession` **同接口**（``run_trajectory`` / ``gripper_port``
    / ``close``），供工艺层离线跑批与单测使用。
    """

    def __init__(self,
                 config: Optional[ExecutorConfig] = None,
                 ws: Optional[str] = None,
                 realtime: bool = False,
                 allow_unverified: bool = False) -> None:
        self.config = config
        self.ws = ws
        self.realtime = bool(realtime)
        self.allow_unverified = bool(allow_unverified)
        self._closed = False
        self.gripper_port_evidence: List[str] = []

    @classmethod
    def open(cls, **kwargs: Any) -> Tuple["FakeSession", None]:
        """离线会话「建立」永远成功（无硬件依赖）。"""
        return cls(**kwargs), None

    def run_trajectory(self, trajectory_path: str,
                       allow_unverified: Optional[bool] = None,
                       gate: Any = None,
                       motion_gate: Any = None) -> Dict[str, Any]:
        """离线执行一条轨迹，返回**与 live 同一 schema** 的统一报告。"""
        if self._closed:
            return _failure_report(
                contract.STATUS_ERROR, "离线会话已关闭",
                "重新建立离线会话后重跑", [], "fake")
        transport = FakeTransport(max_records=FAKE_CALL_RECORD_LIMIT)
        feedback = mirror_feedback(transport, DEFAULT_PERIOD_S)
        clock = (TickClock(period_s=DEFAULT_PERIOD_S) if self.realtime
                 else ManualClock(period_s=DEFAULT_PERIOD_S))
        executor = Executor(transport, feedback, config=self.config, clock=clock,
                            mode="fake", ws=self.ws, gate=gate,
                            motion_gate=motion_gate)
        allow = self.allow_unverified if allow_unverified is None else bool(allow_unverified)
        return executor.run(trajectory_path, allow_unverified=allow)

    def gripper_port(self) -> Any:
        """离线会话没有真机夹爪端口（返回 None + 证据，不抛裸异常）。"""
        self.gripper_port_evidence = ["离线会话没有真机夹爪端口"]
        return None

    def close(self) -> None:
        """关闭（**幂等**）。"""
        self._closed = True

    @property
    def closed(self) -> bool:
        return self._closed

    def __enter__(self) -> "FakeSession":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()
