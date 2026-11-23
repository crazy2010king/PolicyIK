"""安全动作执行器（**唯一动作权威**的执行侧实现）。

本模块回答一个问题：**谁执行安全动作、失败了怎么办**。

职责边界
--------

* 只做「等级 → 动作 → 调用传输端口」这一件事；
* 等级语义（动作名 / SDK 符号 / 严重度 / 恢复方式）**一律来自**
  :mod:`rm_safety.levels`（唯一真值），本模块**不另立一套等级表**；
* **失败必须留证据并升级**：L1/L2/L3 的动作失败 → 立即执行 L4 急停
  （``escalated_to="L4"``）；L4 动作失败 → **只留 FATAL 证据，绝不假装成功**；
* ``restore_speed()`` 是**唯一**的自动恢复入口（L1 回落到告警阈值内后
  恢复原速），且只调用 ``set_speed_scale(1.0)``。

本模块**不**做（红线，写在模块文档与断言里）：

* 不自动清控制器错误码；
* 不自动重新使能关节；
* 不自动重启控制器；
* 不自动 ``continue_``（继续运行）——那是人工动作。

依赖方向
--------

本模块**不 import** ``rm_executor`` / ``rm_hw_bridge`` / ``rmctl`` /
``tesseract*``：传输端口是**结构化鸭子类型**（由调用方注入），安全层因此
保持零反向依赖。

传输端口协议（:class:`TransportPort`）
-------------------------------------

端口必须提供下列方法，全部返回**统一结果字典**
``{"status", "reason", "next_action", "evidence"}``，其中 ``status == "ok"``
表示成功（其余状态一律视为失败）：

====================  ==========================================  ================
方法                  入参                                       用途
====================  ==========================================  ================
``set_speed_scale``   ``scale``（∈ (0, 1]）                      L1 降速 / 恢复
``pause``             无                                          L2 暂停
``slow_stop``         无                                          L3 缓停
``stop``              无                                          L4 急停
====================  ==========================================  ================

端口**不得**抛裸异常；即便抛了，本模块也会捕获并按失败处理（留证据 → 升级）。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, Mapping, Optional, Tuple

from rm_safety import levels as _levels

__all__ = [
    "ACTION_METHODS",
    "ActionResult",
    "SafetyActuator",
    "STATUS_ESCALATED",
    "STATUS_FATAL",
    "STATUS_OK",
]

#: 动作成功
STATUS_OK = "ok"
#: 原动作失败、已升级到 L4 且 L4 停机成功
STATUS_ESCALATED = "escalated"
#: 致命失败（L4 动作本身失败 / 端口异常）——**不假装成功**
STATUS_FATAL = "fatal"

#: 等级 → 传输端口方法名（**唯一动作映射**；等级语义见 rm_safety.levels）
ACTION_METHODS: Dict[str, str] = {
    _levels.LEVEL_L1: "set_speed_scale",
    _levels.LEVEL_L2: "pause",
    _levels.LEVEL_L3: "slow_stop",
    _levels.LEVEL_L4: "stop",
}


# --------------------------------------------------------------------- 结果
@dataclass(frozen=True)
class ActionResult:
    """一次安全动作的执行结果（**纯数据**）。

    字段
    ----
    level
        原始等级（``L1``–``L4``）。
    action
        动作名（``set_speed_scale`` / ``pause`` / ``slow_stop`` / ``stop``）。
    transport_method
        实际调用的传输端口方法名。
    sdk_symbol
        对应的 RM-API2 SDK 符号（取自等级表，用于审计对账）。
    status
        ``ok``（成功）/ ``escalated``（失败后已升级到 L4 且停机成功）/
        ``fatal``（L4 失败，绝不假装成功）。
    reason
        人可读的原因（成功时说明做了什么；失败时写明失败点）。
    evidence
        证据元组（每项 dict，至少含 ``kind`` / ``level`` / ``reason``）。
    escalated_to
        升级到的等级（失败升级时为 ``"L4"``；否则空串）。
    next_action
        **可粘贴**的下一步。
    """

    level: str = ""
    action: str = ""
    transport_method: str = ""
    sdk_symbol: str = ""
    status: str = STATUS_FATAL
    reason: str = ""
    evidence: Tuple[Dict[str, Any], ...] = field(default_factory=tuple)
    escalated_to: str = ""
    next_action: str = ""

    @property
    def ok(self) -> bool:
        """动作是否已成功落地（含「失败后升级到 L4 且停机成功」）。"""
        return self.status in (STATUS_OK, STATUS_ESCALATED)

    @property
    def severity(self) -> str:
        """事件严重度（按**最终生效等级**：升级到 L4 即 FATAL）。"""
        effective = self.escalated_to or self.level
        if not effective:
            return _levels.SEVERITY_FATAL
        return _levels.level_severity(effective)

    @property
    def effective_level(self) -> str:
        """最终生效等级（升级后为 L4）。"""
        return self.escalated_to or self.level

    def to_dict(self) -> Dict[str, Any]:
        """自描述字典（报告 / 事件用）。"""
        return {
            "level": self.level,
            "action": self.action,
            "transport_method": self.transport_method,
            "sdk_symbol": self.sdk_symbol,
            "status": self.status,
            "reason": self.reason,
            "evidence": [dict(item) for item in self.evidence],
            "escalated_to": self.escalated_to,
            "next_action": self.next_action,
            "effective_level": self.effective_level,
        }


# ------------------------------------------------------------------ 执行器
class SafetyActuator:
    """安全动作执行器（把等级翻译成传输端口调用，失败即升级）。

    参数
    ----
    transport
        传输端口（鸭子类型，见模块文档 :class:`TransportPort` 协议）。
    l1_speed_scale
        L1 降速比例（∈ (0, 1]）；**由调用方从唯一真值传入**
        （安全配置 ``safety.l1_speed_scale``）。
    max_records
        :attr:`action_log` 的保留上限（``None`` = 不限）。长跑场景下
        动作序列会随时间增长，设上限可让内存有界（沿用 P005 的做法：
        淘汰旧记录，但计数不受影响）。
    """

    def __init__(self, transport: Any, l1_speed_scale: float = 0.5,
                 max_records: Optional[int] = None) -> None:
        if transport is None:
            raise ValueError("transport 不能为空（安全动作必须有执行端口）")
        scale = float(l1_speed_scale)
        if not (0.0 < scale <= 1.0):
            raise ValueError("l1_speed_scale 必须落在 (0, 1]（当前 %r）"
                             % (l1_speed_scale,))
        self.transport = transport
        self.l1_speed_scale = scale
        self._max_records = None if max_records is None else int(max_records)
        #: 已执行的动作序列（审计用；只增不改）
        self.action_log: list = []
        #: 动作执行总次数（**不受保留上限影响**）
        self.action_count = 0

    def _record(self, outcome: "ActionResult") -> None:
        """追加一条动作记录（保留上限内才留详情，计数永远准确）。"""
        self.action_count += 1
        self.action_log.append(outcome)
        if self._max_records is not None and len(self.action_log) > self._max_records:
            del self.action_log[0]

    # ------------------------------------------------------------ 动作执行
    def apply(self, level: str, context: Optional[Mapping[str, Any]] = None) -> ActionResult:
        """执行某个等级的安全动作；失败 → 留证据并升级到 L4。

        参数
        ----
        level
            ``L1``–``L4``；未知等级抛 :class:`ValueError`（不静默兜底）。
        context
            可选上下文（``reason`` / ``evidence`` / ``next_action``），
            用于把判据证据带进动作证据。

        返回
        ----
        ActionResult
            失败时 ``status`` 为 ``escalated``（已升级停机成功）或 ``fatal``
            （L4 失败）。
        """
        if level not in ACTION_METHODS:
            raise ValueError("未知安全等级 %r（只允许 %s）"
                             % (level, ", ".join(_levels.LEVEL_ORDER)))
        context = dict(context or {})
        method = ACTION_METHODS[level]
        action = _levels.level_action(level)
        sdk_symbol = _levels.level_sdk_symbol(level)

        # L1 需要带下降速比例；其余动作无参
        args = (self.l1_speed_scale,) if level == _levels.LEVEL_L1 else ()
        ok, result, failure = self._call(method, level, args=args)

        if ok:
            evidence = self._evidence(
                "action_ok", level, action, method, sdk_symbol,
                reason=("已执行 %s（%s）" % (action, sdk_symbol)),
                detail=result,
            )
            outcome = ActionResult(
                level=level,
                action=action,
                transport_method=method,
                sdk_symbol=sdk_symbol,
                status=STATUS_OK,
                reason="已执行安全动作 %s（%s）" % (action, sdk_symbol),
                evidence=tuple(evidence),
                escalated_to="",
                next_action=str(context.get("next_action") or ""),
            )
            self._record(outcome)
            return outcome

        # ---- 失败：留证据 ----
        evidence = self._evidence(
            "action_failed", level, action, method, sdk_symbol,
            reason=failure,
            detail=result,
        )
        if level == _levels.LEVEL_L4:
            # L4 失败：只留 FATAL 证据，**不假装成功**，不假装已停机
            evidence.extend(self._evidence(
                "l4_stop_failed", level, action, method, sdk_symbol,
                reason="L4 急停未成功：%s（不假装已停机，需立即人工处置）" % failure,
                detail=result,
            ))
            outcome = ActionResult(
                level=level,
                action=action,
                transport_method=method,
                sdk_symbol=sdk_symbol,
                status=STATUS_FATAL,
                reason="L4 急停失败：%s" % failure,
                evidence=tuple(evidence),
                escalated_to="",
                next_action=("立即按下硬件急停并人工确认机械臂状态；"
                             "把证据原样贴给操作者"),
            )
            self._record(outcome)
            return outcome

        # ---- 失败 → 升级到 L4 急停 ----
        stop_ok, stop_result, stop_failure = self._call(
            ACTION_METHODS[_levels.LEVEL_L4], _levels.LEVEL_L4)
        if stop_ok:
            evidence.extend(self._evidence(
                "escalated_to_l4", level, action, method, sdk_symbol,
                reason="%s 失败 → 已升级执行 L4 急停（%s）"
                       % (action, _levels.level_sdk_symbol(_levels.LEVEL_L4)),
                detail=stop_result,
            ))
            outcome = ActionResult(
                level=level,
                action=action,
                transport_method=method,
                sdk_symbol=sdk_symbol,
                status=STATUS_ESCALATED,
                reason="%s 失败，已升级执行 L4 急停（原因：%s）" % (action, failure),
                evidence=tuple(evidence),
                escalated_to=_levels.LEVEL_L4,
                next_action=("确认机械臂已停机后人工处置；安全层不自动恢复"),
            )
        else:
            evidence.extend(self._evidence(
                "l4_stop_failed", _levels.LEVEL_L4, "stop",
                ACTION_METHODS[_levels.LEVEL_L4],
                _levels.level_sdk_symbol(_levels.LEVEL_L4),
                reason="升级到 L4 急停也失败：%s（不假装已停机）" % stop_failure,
                detail=stop_result,
            ))
            outcome = ActionResult(
                level=level,
                action=action,
                transport_method=method,
                sdk_symbol=sdk_symbol,
                status=STATUS_FATAL,
                reason="%s 失败且 L4 急停失败：%s / %s" % (action, failure, stop_failure),
                evidence=tuple(evidence),
                escalated_to=_levels.LEVEL_L4,
                next_action=("立即按下硬件急停并人工确认机械臂状态；"
                             "把证据原样贴给操作者"),
            )
        self._record(outcome)
        return outcome

    # ------------------------------------------------------------ 自动恢复
    def restore_speed(self) -> Dict[str, Any]:
        """恢复原速（**仅 L1 自动恢复**用）。

        只调用 ``transport.set_speed_scale(1.0)``；失败**不升级**（恢复失败
        意味着机械臂仍处于降速状态，不是更危险的方向），但同样留证据、
        不抛裸异常。
        """
        ok, result, failure = self._call(
            ACTION_METHODS[_levels.LEVEL_L1], _levels.LEVEL_L1,
            args=(1.0,))
        if ok:
            return {
                "status": "ok",
                "reason": "已恢复原速（set_speed_scale=1.0）",
                "next_action": "",
                "evidence": self._evidence(
                    "l1_speed_restored", _levels.LEVEL_L1, "set_speed_scale",
                    ACTION_METHODS[_levels.LEVEL_L1],
                    _levels.level_sdk_symbol(_levels.LEVEL_L1),
                    reason="跟随误差回落到告警阈值内 → 恢复原速",
                    detail=result,
                ),
            }
        return {
            "status": "failed",
            "reason": "恢复原速失败：%s" % failure,
            "next_action": "人工确认机械臂速度状态后重新下发原速",
            "evidence": self._evidence(
                "l1_speed_restore_failed", _levels.LEVEL_L1, "set_speed_scale",
                ACTION_METHODS[_levels.LEVEL_L1],
                _levels.level_sdk_symbol(_levels.LEVEL_L1),
                reason=failure,
                detail=result,
            ),
        }

    # ------------------------------------------------------------ 内部工具
    def _call(self, method: str, level: str,
              args: Tuple[Any, ...] = ()) -> Tuple[bool, Optional[Dict[str, Any]], str]:
        """调用端口方法，返回 ``(ok, result, failure_reason)``。

        端口抛异常 / 返回非字典 / 返回非 ``ok`` 状态 → 一律按失败处理
        （**失败绝不吞掉**，失败原因会进入证据）。
        """
        func = getattr(self.transport, method, None)
        if func is None or not callable(func):
            return False, None, "传输端口缺少方法 %s" % method
        try:
            result = func(*args)
        except Exception as exc:  # noqa: BLE001 - 端口异常也必须留证据
            return False, None, "%s 调用异常（%s）: %s" % (
                method, type(exc).__name__, exc)
        if not isinstance(result, Mapping):
            return False, None, "%s 返回格式不合法（必须是结果字典，当前 %s）" % (
                method, type(result).__name__)
        status = result.get("status")
        if status != "ok":
            reason = result.get("reason") or "状态=%r" % (status,)
            return False, dict(result), "%s 未成功：%s" % (method, reason)
        return True, dict(result), ""

    @staticmethod
    def _evidence(kind: str, level: str, action: str, method: str,
                  sdk_symbol: str, reason: str,
                  detail: Optional[Mapping[str, Any]] = None) -> list:
        """构造一条动作证据（字段固定，便于离线逐条核对）。"""
        item: Dict[str, Any] = {
            "kind": kind,
            "level": level,
            "action": action,
            "transport_method": method,
            "sdk_symbol": sdk_symbol,
            "reason": reason,
        }
        if detail is not None:
            item["transport_status"] = detail.get("status")
            transport_evidence = detail.get("evidence")
            if transport_evidence:
                item["transport_evidence"] = list(transport_evidence)
            if detail.get("next_action"):
                item["transport_next_action"] = detail["next_action"]
        return [item]
