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

S04 新增四类注入（**安全层四级链路验收用**）
--------------------------------------------

=============================  ==========================================  ==================
注入                           触发条件                                  预期等级 / 动作
=============================  ==========================================  ==================
``follow_error_light:<deg>``  跟随误差偏置落在 **告警阈值之上、停机阈值**
                              之内（``2.0° < deg ≤ 5.0°``）            L1 降速
                                                                   （``set_speed_scale``）
``limit_exceeded:<deg>``      反馈里 ``joint1`` 位置被抬到「声明限位
                              ``max_pos`` + ``deg``」（``deg > 0``）    L2 暂停
                                                                   （``pause``）
``collision:<code>``          ``transport.controller_state()`` 返回
                              ``arm_err=[code]``（码表内碰撞码 → L3；
                              码表外未知码 → **L3 保守档**）            L3 缓停
                                                                   （``slow_stop``）
``protection_stop``           ``transport.controller_state()`` 返回
                              ``arm_current_status=9``（RM_STOP_E）      L4 急停
                                                                   （``stop``）
=============================  ==========================================  ==================

**诚实声明**：碰撞在软件侧只能靠控制器错误码表达，而 RM-API2 ``err[]`` 的
**厂商错误码表本轮未取得**——因此「碰撞码」用的是
``workcell/safety/error_codes.yaml`` 里的**工程声明映射码**（``class=collision``），
真机窗口必须逐条回填（见 ``docs/rm_safety安全停机四级链路说明.md``）。

四条硬约束
----------

1. **只对 fake 模式开放**：真机（live）路径拒绝任何注入——故障注入绝不
   能带到真机。
2. **注入从流式阶段（阶段 C）起生效**：预就位阶段（阶段 B）必须正常到位，
   否则注入会退化成「预就位超时」而不是被注入的那条判据。
3. **断流注入时长必须大于执行层停机阈值**（``STREAM_STOP_MS = 200 ms``）：
   注入时长短于阈值时执行层按设计**不会**判 FATAL（断流中途恢复），
   因此这类取值在解析阶段就被拒绝，而不是跑出一个「没有事件」的报告。
4. **轻跟随误差 / 超限注入也必须在解析阶段自证**：取值落在阈值之内时
   执行层按设计**不会**判对应的等级，因此这类取值在解析阶段就被拒绝
   （而不是跑出一份「跑了但没动作」的报告）。

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
    "LIMIT_EXCEEDED_JOINT_INDEX",
    "PROTECTION_STOP_STATUS",
    "InjectedFeedback",
    "apply_injections",
    "describe",
    "parse_injection",
]

#: 支持的注入种类
INJECTION_KINDS: Tuple[str, ...] = (
    "follow_error",
    "feedback_stall",
    "controller_conflict",
    # -- S04 新增四类（安全层四级链路验收用）--
    "follow_error_light",
    "limit_exceeded",
    "collision",
    "protection_stop",
)

#: 控制器抢轨迹注入使用的轨迹类型名（≠ 执行层期望的 RM_NO_PLANNING_E）
CONFLICT_TYPE_NAME = "RM_PLANNING_A"

#: ``limit_exceeded`` 注入作用到的关节下标（0 基 = ``joint1``）
LIMIT_EXCEEDED_JOINT_INDEX = 0

#: ``protection_stop`` 注入使用的控制器状态码（``RM_STOP_E``）
PROTECTION_STOP_STATUS = 9

#: 不接受参数的注入种类
_NO_PARAM_KINDS: Tuple[str, ...] = ("controller_conflict", "protection_stop")


class _InjectionSpecError(ValueError):
    """注入参数非法（带可粘贴的下一步）。"""


def parse_injection(spec: str) -> Dict[str, Any]:
    """解析一条注入规格 → 规范字典。

    规格格式::

        follow_error:<deg>          跟随误差偏置（度，> 停机阈值）
        follow_error_light:<deg>    轻跟随误差偏置（度，告警阈值 < deg ≤ 停机阈值）
        feedback_stall:<ms>         反馈断流时长（毫秒，> STREAM_STOP_MS）
        controller_conflict         控制器抢轨迹（无参数）
        limit_exceeded:<deg>        joint1 反馈位置抬到「声明限位 max_pos + deg」
        collision:<code>            控制器错误码 arm_err=[code]（非负整数）
        protection_stop             控制器状态码 arm_current_status=9（RM_STOP_E）

    非法输入抛 :class:`ValueError`（消息含可粘贴的下一步）。
    """
    if not isinstance(spec, str) or not spec.strip():
        raise _InjectionSpecError(
            "注入规格为空：可用格式 follow_error:<deg> / "
            "follow_error_light:<deg> / feedback_stall:<ms> / "
            "controller_conflict / limit_exceeded:<deg> / collision:<code> / "
            "protection_stop")
    text = spec.strip()
    kind, _, value = text.partition(":")
    kind = kind.strip()
    if kind not in INJECTION_KINDS:
        raise _InjectionSpecError(
            "未知注入种类 %r：只允许 %s"
            % (kind, " / ".join(INJECTION_KINDS)))
    if kind in _NO_PARAM_KINDS:
        if value.strip():
            raise _InjectionSpecError(
                "%s 不接受参数（当前 %r）" % (kind, value.strip()))
        return {"kind": kind, "value": None, "spec": text}
    if not value.strip():
        raise _InjectionSpecError(
            "注入 %s 缺少参数：用法 %s:<值>" % (kind, kind))
    if kind == "collision":
        # 错误码必须是**非负整数**（码表外未知码 → 保守 L3）
        raw = value.strip()
        try:
            code = int(raw, 10)
        except ValueError:
            raise _InjectionSpecError(
                "collision 的错误码必须是非负整数（当前 %r）" % raw) from None
        if code < 0:
            raise _InjectionSpecError(
                "collision 的错误码必须是非负整数（当前 %r）" % raw)
        return {"kind": kind, "value": code, "spec": text}
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
    elif kind == "follow_error_light":
        # 轻跟随误差：必须落在**告警阈值之上、停机阈值之内**，
        # 否则执行层按设计不会判 L1（降速）
        if number <= contract.FOLLOW_WARN_DEG:
            raise _InjectionSpecError(
                "follow_error_light 偏置 %.3f° 必须大于告警阈值 %.1f°，"
                "否则执行层按设计不会判 L1（降速）"
                % (number, contract.FOLLOW_WARN_DEG))
        if number > contract.FOLLOW_STOP_DEG:
            raise _InjectionSpecError(
                "follow_error_light 偏置 %.3f° 超过停机阈值 %.1f°，"
                "该场景请用 follow_error:<deg>（预期 L2 暂停）"
                % (number, contract.FOLLOW_STOP_DEG))
    elif kind == "limit_exceeded":
        if number <= 0:
            raise _InjectionSpecError(
                "limit_exceeded 的越界量必须 > 0（当前 %r）" % number)
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


class _ControllerStateTransport:
    """包一层传输层：``controller_state()`` 返回注入的控制器错误态。

    * ``arm_err`` —— 控制器错误码列表（碰撞注入）；
    * ``arm_current_status`` —— 控制器状态码（保护停注入，``RM_STOP_E=9``）。

    其它方法**原样转发**给底层传输层；``controller_state`` 本身**也先调底层**
    （因此调用记录仍留在底层 FakeTransport，便于事后审计「查询确实发生过」）。
    """

    def __init__(self, base: Any, arm_err: Optional[Sequence[int]] = None,
                 arm_current_status: Optional[int] = None) -> None:
        self.base = base
        self.arm_err: List[int] = [int(c) for c in (arm_err or [])]
        self.arm_current_status = (None if arm_current_status is None
                                   else int(arm_current_status))

    def __getattr__(self, item: str) -> Any:
        return getattr(self.base, item)

    def controller_state(self) -> Dict[str, Any]:
        """查询控制器错误态（注入 arm_err / arm_current_status）。"""
        result = self.base.controller_state()
        if not isinstance(result, dict):
            return result
        out = dict(result)
        out["arm_err"] = list(self.arm_err)
        out["arm_current_status"] = self.arm_current_status
        out["injected"] = True
        return out


class _LimitExceededFeedback:
    """包一层反馈源：``joint1`` 位置抬到「声明限位 max_pos + deg」。

    * 生效时机：流式阶段起（预就位阶段必须正常到位）；
    * 限位唯一来源：``rm_hw_bridge.limits``（SI，rad）+ ``RAD2DEG`` 单点换算
      —— 与执行层的超限判据**同一数据源**（不另开第二处限值常量）。
    """

    name = "scripted+limit_exceeded"

    def __init__(self, base: Any, transport: Any,
                 joint_index: int = LIMIT_EXCEEDED_JOINT_INDEX,
                 target_deg: float = 0.0,
                 period_s: float = 0.010) -> None:
        self.base = base
        self.transport = transport
        self.joint_index = int(joint_index)
        self.target_deg = float(target_deg)
        self.period_s = float(period_s)
        self.last_error = ""

    def _streaming(self) -> bool:
        counter = getattr(self.transport, "count", None)
        if not callable(counter):
            return False
        try:
            return int(counter("transmit")) > 0
        except Exception:  # noqa: BLE001 - 判据失败按「未进入流式」处理
            return False

    def latest(self) -> Dict[str, Any]:
        snapshot = self.base.latest()
        if not isinstance(snapshot, dict):
            return snapshot
        out = dict(snapshot)
        if self._streaming():
            position = out.get("joint_position_deg")
            if isinstance(position, (list, tuple)) and len(position) > self.joint_index:
                values = [float(v) for v in position]
                values[self.joint_index] = self.target_deg
                out["joint_position_deg"] = values
                self.last_error = ("注入：joint%d 反馈位置抬到 %.4f°"
                                   "（声明限位 max_pos + 越界量）"
                                   % (self.joint_index + 1, self.target_deg))
        return out

    def to_dict(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "joint_index": self.joint_index,
            "target_deg": self.target_deg,
            "last_error": self.last_error,
        }


def declared_max_pos_deg(joint_index: int = LIMIT_EXCEEDED_JOINT_INDEX,
                         ws: Optional[str] = None) -> float:
    """声明限位某一关节的 ``max_pos``（**度**）。

    限值唯一来源 = ``rm_hw_bridge.limits.load_joint_limits()``（SI，rad），
    rad→deg 换算走 ``rm_hw_bridge.bridge.RAD2DEG`` **单点**（与执行层一致）。
    """
    try:
        from rm_hw_bridge import limits as hw_limits
        from rm_hw_bridge.bridge import RAD2DEG
    except Exception as exc:  # noqa: BLE001 - 硬件层不可用 → 明确报错
        raise _InjectionSpecError(
            "无法读取声明限值（硬件适配层 rm_hw_bridge 不可用：%s: %s）"
            % (type(exc).__name__, exc)) from exc
    try:
        limits = hw_limits.load_joint_limits(ws)
    except Exception as exc:  # noqa: BLE001
        raise _InjectionSpecError(
            "读取声明限值失败（%s: %s）" % (type(exc).__name__, exc)) from exc
    name = "joint%d" % (int(joint_index) + 1)
    item = limits.get(name)
    if not isinstance(item, dict) or "max_pos" not in item:
        raise _InjectionSpecError(
            "声明限值缺少关节 %s 的 max_pos（限值数据不完整）" % name)
    return float(item["max_pos"]) * float(RAD2DEG)


def apply_injections(transport: Any, feedback: Any,
                     injections: Sequence[Dict[str, Any]],
                     period_s: float = 0.010,
                     ws: Optional[str] = None) -> Tuple[Any, Any]:
    """把注入规格应用到（传输层, 反馈层）上；返回 ``(transport, feedback)``。

    未注入任何东西时**原样返回**（零开销、零行为变化）。

    ``ws`` 只影响 ``limit_exceeded``：声明限位从 ``rm_hw_bridge.limits``
    读取（唯一数据源），与执行层的超限判据一致。
    """
    follow_error_deg = 0.0
    stall_ms = 0.0
    conflict = False
    limit_exceeded_deg: Optional[float] = None
    arm_err: Optional[List[int]] = None
    arm_status: Optional[int] = None
    for item in injections or []:
        kind = item.get("kind")
        if kind in ("follow_error", "follow_error_light"):
            follow_error_deg = float(item.get("value") or 0.0)
        elif kind == "feedback_stall":
            stall_ms = float(item.get("value") or 0.0)
        elif kind == "controller_conflict":
            conflict = True
        elif kind == "limit_exceeded":
            limit_exceeded_deg = float(item.get("value") or 0.0)
        elif kind == "collision":
            arm_err = [int(item.get("value"))]
        elif kind == "protection_stop":
            arm_status = PROTECTION_STOP_STATUS
    if conflict:
        transport = _ConflictTransport(transport)
    if arm_err is not None or arm_status is not None:
        transport = _ControllerStateTransport(transport, arm_err=arm_err,
                                              arm_current_status=arm_status)
    if follow_error_deg or stall_ms:
        feedback = InjectedFeedback(feedback, transport,
                                    follow_error_deg=follow_error_deg,
                                    stall_ms=stall_ms, period_s=period_s)
    if limit_exceeded_deg is not None:
        target = declared_max_pos_deg(LIMIT_EXCEEDED_JOINT_INDEX, ws) \
            + limit_exceeded_deg
        feedback = _LimitExceededFeedback(
            feedback, transport, joint_index=LIMIT_EXCEEDED_JOINT_INDEX,
            target_deg=target, period_s=period_s)
    return transport, feedback


def describe(injections: Sequence[Dict[str, Any]]) -> List[str]:
    """把注入规格渲染成可写进报告 evidence 的一行行描述。"""
    out: List[str] = []
    for item in injections or []:
        kind = item.get("kind")
        if kind == "follow_error":
            out.append("injection: follow_error=%+.3f deg（流式阶段起生效，"
                       "离线代理）" % float(item.get("value") or 0.0))
        elif kind == "follow_error_light":
            out.append("injection: follow_error_light=%+.3f deg（流式阶段起生效，"
                       "预期 L1 降速，离线代理）"
                       % float(item.get("value") or 0.0))
        elif kind == "feedback_stall":
            out.append("injection: feedback_stall=%.0f ms（流式阶段起冻结帧计数，"
                       "离线代理）" % float(item.get("value") or 0.0))
        elif kind == "controller_conflict":
            out.append("injection: controller_conflict（控制器轨迹类型恒为 %s，"
                       "离线代理）" % CONFLICT_TYPE_NAME)
        elif kind == "limit_exceeded":
            out.append("injection: limit_exceeded=+%.3f deg（joint1 反馈位置抬到"
                       "声明限位 max_pos 之上，预期 L2 暂停，离线代理）"
                       % float(item.get("value") or 0.0))
        elif kind == "collision":
            out.append("injection: collision=arm_err[%d]（控制器错误码；"
                       "码表为工程声明映射，预期 L3 缓停，离线代理）"
                       % int(item.get("value") or 0))
        elif kind == "protection_stop":
            out.append("injection: protection_stop=arm_current_status=%d"
                       "（RM_STOP_E，预期 L4 急停，离线代理）"
                       % PROTECTION_STOP_STATUS)
    return out
