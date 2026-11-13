"""离线故障注入（**仅 fake 模式**）——S03 证据复跑用的受控异常。

三类注入与 S03 切片契约的异常判定一一对应：

======================  =====================================  ==================
注入                    触发条件                              期望事件 / 退出码
======================  =====================================  ==================
``follow_error:<deg>``  流式阶段跟随误差恒定偏置 ``<deg>``°  ``follow_error_exceeded``
                                                                （NEEDS_HUMAN +
                                                                ``transport.pause()``）
                                                                → blocked / 3
``feedback_stall:<ms>`` 流式阶段反馈**不再产出新帧** ``<ms>``  ``feedback_stream_lost``
                                                                （FATAL +
                                                                ``transport.stop()``）
                                                                → error / 4
``controller_conflict`` 控制器轨迹类型恒为非期望值            ``controller_trajectory_conflict``
                                                                （FATAL +
                                                                ``transport.stop()``）
                                                                → error / 4
======================  =====================================  ==================

三条硬约束
----------

1. **只对 fake 模式开放**：真机（live）路径拒绝任何注入——故障注入绝不
   能带到真机。
2. **注入从流式阶段（阶段 C）起生效**：预就位阶段（阶段 B）必须正常到位，
   否则注入会退化成「预就位超时」而不是被注入的那条判据。
3. **断流注入时长必须大于执行层停机阈值**（``STREAM_STOP_MS = 200 ms``）：
   注入时长短于阈值时执行层按设计**不会**判 FATAL（断流中途恢复），
   因此这类取值在解析阶段就被拒绝，而不是跑出一个「没有事件」的报告。

**注入是代理**：离线注入得到的事件与停机动作是**离线代理结论**，不代表
真机结论（真机结论见真机窗口）。
"""
from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Sequence, Tuple

from rm_executor import contract

__all__ = [
    "INJECTION_KINDS",
    "CONFLICT_TYPE_NAME",
    "InjectedFeedback",
    "apply_injections",
    "parse_injection",
]

#: 支持的注入种类
INJECTION_KINDS: Tuple[str, ...] = (
    "follow_error",
    "feedback_stall",
    "controller_conflict",
)

#: 控制器抢轨迹注入使用的轨迹类型名（≠ 执行层期望的 RM_NO_PLANNING_E）
CONFLICT_TYPE_NAME = "RM_PLANNING_A"


class _InjectionSpecError(ValueError):
    """注入参数非法（带可粘贴的下一步）。"""


def parse_injection(spec: str) -> Dict[str, Any]:
    """解析一条注入规格 → 规范字典。

    规格格式::

        follow_error:<deg>          跟随误差偏置（度，> 0）
        feedback_stall:<ms>         反馈断流时长（毫秒，> STREAM_STOP_MS）
        controller_conflict         控制器抢轨迹（无参数）

    非法输入抛 :class:`ValueError`（消息含可粘贴的下一步）。
    """
    if not isinstance(spec, str) or not spec.strip():
        raise _InjectionSpecError(
            "注入规格为空：可用格式 follow_error:<deg> / feedback_stall:<ms> / "
            "controller_conflict")
    text = spec.strip()
    kind, _, value = text.partition(":")
    kind = kind.strip()
    if kind not in INJECTION_KINDS:
        raise _InjectionSpecError(
            "未知注入种类 %r：只允许 %s"
            % (kind, " / ".join(INJECTION_KINDS)))
    if kind == "controller_conflict":
        if value.strip():
            raise _InjectionSpecError(
                "controller_conflict 不接受参数（当前 %r）" % value.strip())
        return {"kind": kind, "value": None, "spec": text}
    if not value.strip():
        raise _InjectionSpecError(
            "注入 %s 缺少参数：用法 %s:<值>" % (kind, kind))
    try:
        number = float(value.strip())
    except ValueError:
        raise _InjectionSpecError(
            "注入 %s 的参数不是数值（当前 %r）" % (kind, value.strip())) from None
    if not math.isfinite(number):
        raise _InjectionSpecError("注入 %s 的参数不是有限数" % kind)
    if kind == "follow_error":
        if number <= 0:
            raise _InjectionSpecError(
                "follow_error 的偏置必须 > 0（当前 %r）" % number)
        if number <= contract.FOLLOW_STOP_DEG:
            raise _InjectionSpecError(
                "follow_error 偏置 %.3f° 未超过执行层停机阈值 %.1f°，"
                "执行层按设计不会判 follow_error_exceeded"
                % (number, contract.FOLLOW_STOP_DEG))
    else:  # feedback_stall
        if number <= contract.STREAM_STOP_MS:
            raise _InjectionSpecError(
                "feedback_stall 时长 %.0f ms 未超过执行层断流停机阈值 %.0f ms，"
                "断流中途恢复时执行层按设计不会判 feedback_stream_lost"
                % (number, contract.STREAM_STOP_MS))
    return {"kind": kind, "value": number, "spec": text}


class InjectedFeedback:
    """带注入的反馈源（包一层 :class:`rm_executor.feedback.ScriptedFeedback`）。

    * ``follow_error_deg`` —— 流式阶段起，位置整体偏置该角度（度）；
    * ``stall_ms`` —— 流式阶段起，**冻结帧计数** ``ceil(ms / period_s)`` 拍
      （不产出新帧 → 执行层的断流 age 增长）。
    """

    name = "scripted+injected"

    def __init__(self, base: Any, transport: Any,
                 follow_error_deg: float = 0.0, stall_ms: float = 0.0,
                 period_s: float = 0.010) -> None:
        self.base = base
        self.transport = transport
        self.follow_error_deg = float(follow_error_deg)
        self.stall_ms = float(stall_ms)
        self.period_s = float(period_s)
        self.stall_budget = (int(math.ceil(self.stall_ms / self.period_s))
                             if self.stall_ms > 0 else 0)
        self.stall_used = 0
        self.last_error = ""
        self._frozen_frames = 0

    # -- 内部 -------------------------------------------------------------
    def _streaming(self) -> bool:
        """是否已进入流式阶段（阶段 C）——判据：已经下发过 transmit。"""
        counter = getattr(self.transport, "count", None)
        if not callable(counter):
            return False
        try:
            return int(counter("transmit")) > 0
        except Exception:  # noqa: BLE001 - 判据失败按「未进入流式」处理
            return False

    # -- FeedbackSource ---------------------------------------------------
    def latest(self) -> Dict[str, Any]:
        """拉取一帧（注入语义见类文档）。"""
        snapshot = self.base.latest()
        if not isinstance(snapshot, dict):
            return snapshot
        out = dict(snapshot)
        streaming = self._streaming()
        frames = out.get("frames")
        if streaming and self.stall_budget and self.stall_used < self.stall_budget:
            # 断流：冻结帧计数 → 执行层的 age 增长（不伪造新帧）
            self.stall_used += 1
            out["frames"] = self._frozen_frames
            self.last_error = ("注入：反馈断流第 %d/%d 拍（不产出新帧）"
                               % (self.stall_used, self.stall_budget))
        elif isinstance(frames, int) and not isinstance(frames, bool):
            self._frozen_frames = int(frames)
        if streaming and self.follow_error_deg:
            position = out.get("joint_position_deg")
            if isinstance(position, (list, tuple)) and position:
                out["joint_position_deg"] = [
                    float(v) + self.follow_error_deg for v in position]
                self.last_error = ("注入：跟随误差偏置 %+.3f°"
                                   % self.follow_error_deg)
        return out

    def to_dict(self) -> Dict[str, Any]:
        """反馈源自描述（报告用）。"""
        return {
            "name": self.name,
            "follow_error_deg": self.follow_error_deg,
            "stall_ms": self.stall_ms,
            "stall_budget_ticks": self.stall_budget,
            "stall_used": self.stall_used,
            "last_error": self.last_error,
        }


class _ConflictTransport:
    """包一层传输层：``current_trajectory_type`` 恒返回非期望类型（抢轨迹）。

    其它方法**原样转发**给底层传输层（调用记录仍然留在底层 FakeTransport，
    因此「查询确实发生过」可事后审计）。
    """

    def __init__(self, base: Any, type_name: str = CONFLICT_TYPE_NAME) -> None:
        self.base = base
        self.type_name = str(type_name)

    def __getattr__(self, item: str) -> Any:
        # 只在底层没有同名属性时转发（避免遮蔽本类的 name / base 等）
        return getattr(self.base, item)

    def current_trajectory_type(self) -> Dict[str, Any]:
        """查询控制器轨迹类型（注入为「非期望类型」）。"""
        result = self.base.current_trajectory_type()
        if isinstance(result, dict):
            out = dict(result)
            out["type_name"] = self.type_name
            out["planning"] = True
            out["injected"] = True
            return out
        return result


def apply_injections(transport: Any, feedback: Any,
                     injections: Sequence[Dict[str, Any]],
                     period_s: float = 0.010) -> Tuple[Any, Any]:
    """把注入规格应用到（传输层, 反馈层）上；返回 ``(transport, feedback)``。

    未注入任何东西时**原样返回**（零开销、零行为变化）。
    """
    follow_error_deg = 0.0
    stall_ms = 0.0
    conflict = False
    for item in injections or []:
        kind = item.get("kind")
        if kind == "follow_error":
            follow_error_deg = float(item.get("value") or 0.0)
        elif kind == "feedback_stall":
            stall_ms = float(item.get("value") or 0.0)
        elif kind == "controller_conflict":
            conflict = True
    if conflict:
        transport = _ConflictTransport(transport)
    if follow_error_deg or stall_ms:
        feedback = InjectedFeedback(feedback, transport,
                                    follow_error_deg=follow_error_deg,
                                    stall_ms=stall_ms, period_s=period_s)
    return transport, feedback


def describe(injections: Sequence[Dict[str, Any]]) -> List[str]:
    """把注入规格渲染成可写进报告 evidence 的一行行描述。"""
    out: List[str] = []
    for item in injections or []:
        kind = item.get("kind")
        if kind == "follow_error":
            out.append("injection: follow_error=%+.3f deg（流式阶段起生效，"
                       "离线代理）" % float(item.get("value") or 0.0))
        elif kind == "feedback_stall":
            out.append("injection: feedback_stall=%.0f ms（流式阶段起冻结帧计数，"
                       "离线代理）" % float(item.get("value") or 0.0))
        elif kind == "controller_conflict":
            out.append("injection: controller_conflict（控制器轨迹类型恒为 %s，"
                       "离线代理）" % CONFLICT_TYPE_NAME)
    return out
