"""安全监督器（**唯一动作权威**的编排层）。

本模块回答两个问题：**什么时候动作**、**什么时候能恢复**。

职责边界
--------

* 判据一律走 :func:`rm_safety.policy.evaluate`（纯函数）—— 本模块**不
  自己写判据**，避免出现第二处真相；
* 动作一律走 :class:`rm_safety.actuator.SafetyActuator`—— 本模块**不直接
  调用传输端口**（除了经由执行器的 ``restore_speed``），因此「谁执行动作」
  只有一处实现；
* 事件通过 ``on_event`` 回调交给宿主（**不碰 EventLog**）—— 安全包因此
  保持零执行层依赖。

等级单调
--------

``current_level`` **只升不降**（``""`` → ``L1`` → ``L2`` → ``L3`` → ``L4``），
**永不回落**：L2 暂停 / L3 缓停 / L4 急停都必须人工处置。

L1 自动恢复成功后 ``current_level`` **仍保持 ``L1``**（等级单调不降级），
恢复状态由 :attr:`speed_scaled` 表达（降速生效 False = 已恢复原速）。
此后若再次发生 L1 故障，会**重新**触发降速（同一等级允许重入）。

明确不做（红线，写在模块文档与单测断言里）
--------------------------------------------

* **不**自动清控制器错误码；
* **不**自动重新使能关节；
* **不**自动重启控制器；
* **不**自动 ``continue_``（继续运行）；
* **不**在 L2/L3/L4 上做任何自动恢复。

自动恢复的唯一入口
------------------

**仅 L1**：跟随误差回落到 ``follow_warn_deg`` 以内后调用
``actuator.restore_speed()``（= ``set_speed_scale(1.0)``），并发出
``safety_l1_slowdown_cleared`` 事件。恢复判定复用
:func:`rm_safety.policy.can_auto_recover`（唯一判据源）。
"""
from __future__ import annotations

from typing import Any, Callable, Dict, List, Mapping, Optional

from rm_safety import levels as _levels
from rm_safety import policy as _policy
from rm_safety.actuator import ActionResult, SafetyActuator

__all__ = [
    "FAULT_EVENT_NAMES",
    "SafetySupervisor",
    "fault_event_name",
]

#: 故障类 → 故障类事件名（**唯一映射**；不在表内的故障类不发故障事件）
FAULT_EVENT_NAMES: Dict[str, str] = {
    _policy.FAULT_KIND_FOLLOW_ERROR_LIGHT: _levels.FAULT_FOLLOW_ERROR,
    _policy.FAULT_KIND_FOLLOW_ERROR_HEAVY: _levels.FAULT_FOLLOW_ERROR,
    _policy.FAULT_KIND_LIMIT_EXCEEDED: _levels.FAULT_LIMIT_EXCEEDED,
    _policy.FAULT_KIND_STREAM_LOST: _levels.FAULT_STREAM_LOST,
    _policy.FAULT_KIND_COLLISION: _levels.FAULT_COLLISION,
    _policy.FAULT_KIND_PROTECTION_STOP: _levels.FAULT_PROTECTION_STOP,
}


def fault_event_name(fault_kind: str) -> str:
    """故障类 → 故障事件名；未登记的故障类返回空串（不发故障事件）。"""
    return FAULT_EVENT_NAMES.get(fault_kind, "")


class SafetySupervisor:
    """安全监督器：判据 → 动作 → 事件（唯一动作权威）。

    参数
    ----
    transport
        传输端口（鸭子类型，见 :mod:`rm_safety.actuator` 的协议说明）。
    thresholds
        判据阈值（由调用方从唯一真值传入，见
        :class:`rm_safety.policy.SafetyThresholds`）。
    on_event
        事件回调 ``f(type, severity, reason, evidence, next_action)``；
        ``None`` 表示不外发（事件仍会记录在 :attr:`events` 里）。
    l1_speed_scale
        L1 降速比例；缺省取 ``thresholds.l1_speed_scale``。
    max_events
        :attr:`events` 的保留上限（``None`` = 不限）。长跑场景下事件数随
        故障次数增长，设上限可让内存有界（P005 的同款做法）。
    """

    def __init__(self, transport: Any,
                 thresholds: _policy.SafetyThresholds,
                 on_event: Optional[Callable[..., Any]] = None,
                 l1_speed_scale: Optional[float] = None,
                 max_events: Optional[int] = None) -> None:
        if thresholds is None:
            raise ValueError("thresholds 不能为空（阈值必须由调用方从唯一真值传入）")
        scale = (thresholds.l1_speed_scale if l1_speed_scale is None
                 else float(l1_speed_scale))
        self.transport = transport
        self.thresholds = thresholds
        self.on_event = on_event
        self.actuator = SafetyActuator(transport, l1_speed_scale=scale)
        #: 当前生效等级（``""`` = 正常；**只升不降**，见模块文档）
        self.current_level: str = ""
        #: 历史上到达过的最高等级（单调，**永不回退**，用于审计）
        self.peak_level: str = ""
        #: 是否处于 L1 降速状态（降速生效 True，恢复后 False）
        self.speed_scaled: bool = False
        #: L1 是否已自动恢复过（审计用）
        self.l1_cleared: bool = False
        #: 已发出的事件（自留副本，便于离线断言与报告）
        self.events: List[Dict[str, Any]] = []
        self._max_events = None if max_events is None else int(max_events)
        #: 事件总数（**不受保留上限影响**）
        self.event_count = 0
        #: 最近一次判据决策
        self.last_decision: Optional[_policy.Decision] = None
        #: 最近一次动作结果
        self.last_action: Optional[ActionResult] = None
        #: 事件回调抛出的异常记录（**不静默**：留痕但不打断安全动作）
        self.event_errors: List[str] = []

    # ------------------------------------------------------------------ 状态
    @property
    def effective_level(self) -> str:
        """当前生效等级（等价于 :attr:`current_level`）。"""
        return self.current_level

    def to_dict(self) -> Dict[str, Any]:
        """自描述字典（``rmctl safety status`` 与报告用）。"""
        return {
            "current_level": self.current_level,
            "peak_level": self.peak_level,
            "speed_scaled": self.speed_scaled,
            "event_count": self.event_count,
            "auto_recovery_allowed": self.auto_recovery_allowed(),
            "action_count": self.actuator.action_count,
        }

    def auto_recovery_allowed(self) -> bool:
        """当前等级是否允许自动恢复（**仅 L1**）。"""
        return self.current_level == _levels.LEVEL_L1

    # ------------------------------------------------------------------ 判定
    def evaluate(self, signals: Mapping[str, Any]) -> _policy.Decision:
        """评估一组故障信号 → 决策（必要时执行安全动作）。

        流程
        ----

        1. :func:`rm_safety.policy.evaluate` 给出判据决策；
        2. 决策等级**高于** ``current_level`` → 执行对应动作并升级
           ``current_level``（动作失败 → 执行器已升级到 L4，此处同步）；
        3. 等级**不高于** ``current_level`` → **不做任何动作**（等级单调，
           L2/L3/L4 不自动回落）；
        4. 当前等级为 L1 且跟随误差回落到告警阈值内 → 自动恢复原速
           （``restore_speed``）并发 ``safety_l1_slowdown_cleared``。

        返回
        ----
        Decision
            判据决策本身（动作结果见 :attr:`last_action`）。
        """
        decision = _policy.evaluate(signals, self.thresholds)
        self.last_decision = decision

        if decision.level:
            if self._should_act(decision.level):
                self._raise_level(decision)
            # 等级不高于当前等级 → 不动作（单调）
            return decision

        # ---- 无故障命中：唯一允许的自动恢复路径（L1） ----
        self._maybe_auto_recover(signals)
        return decision

    # -------------------------------------------------------------- 内部工具
    def _should_act(self, level: str) -> bool:
        """是否需要执行动作。

        * 等级**高于**当前等级 → 执行（升级）；
        * 等级**等于**当前等级且当前是 L1 且降速**已恢复** → 重新执行
          （L1 可重入：恢复后再次发生 L1 故障应再次降速）；
        * 其余（等级不高于当前等级）→ **不动作**（等级单调，不自动回落）。
        """
        if self._rank(level) > self._rank(self.current_level):
            return True
        if (level == _levels.LEVEL_L1 and self.current_level == _levels.LEVEL_L1
                and not self.speed_scaled):
            return True
        return False

    def _rank(self, level: str) -> int:
        """等级序号（空串 = 0）。"""
        if not level:
            return 0
        return _levels.level_rank(level)

    def _raise_level(self, decision: _policy.Decision) -> None:
        """执行动作 + 升级等级 + 发事件。"""
        result = self.actuator.apply(decision.level, context={
            "reason": decision.reason,
            "evidence": decision.evidence,
            "next_action": decision.next_action,
        })
        self.last_action = result

        effective = result.effective_level or decision.level
        self.current_level = effective
        if self._rank(effective) > self._rank(self.peak_level):
            self.peak_level = effective
        if effective == _levels.LEVEL_L1 and result.status == "ok":
            self.speed_scaled = True

        evidence = self._merge_evidence(decision, result)
        severity = _levels.level_severity(effective)
        reason = "%s｜动作=%s｜%s" % (decision.reason, result.action, result.reason)
        next_action = decision.next_action or result.next_action

        # 等级事件（安全动作本身：用**决策等级**的事件名）
        self._emit(_levels.level_event_type(decision.level), severity, reason,
                   evidence, next_action)
        # 故障类事件（主因故障）
        fault_event = fault_event_name(decision.fault_kind)
        if fault_event:
            self._emit(fault_event, severity,
                       "主因故障 %s → %s" % (decision.fault_kind, effective),
                       evidence, next_action)
        # 升级到 L4 时额外记一条 L4 事件（失败升级也必须留痕）
        if result.escalated_to == _levels.LEVEL_L4 and effective == _levels.LEVEL_L4:
            self._emit(_levels.EVENT_L4_EMERGENCY_STOP, _levels.SEVERITY_FATAL,
                       "安全动作失败 → 已升级执行 L4 急停（%s）" % result.reason,
                       evidence, result.next_action or next_action)

    def _maybe_auto_recover(self, signals: Mapping[str, Any]) -> None:
        """L1 自动恢复（**唯一**允许自动恢复的等级）。"""
        if self.current_level != _levels.LEVEL_L1:
            return
        if not self.speed_scaled:
            return
        follow_error = signals.get("follow_error_deg")
        if not _policy.can_auto_recover(_levels.LEVEL_L1, follow_error,
                                        self.thresholds):
            return
        outcome = self.actuator.restore_speed()
        evidence = list(outcome.get("evidence") or [])
        if outcome.get("status") != "ok":
            # 恢复失败 → 留证据，不假装恢复成功；等级仍为 L1（不升级）
            self._emit(_levels.EVENT_L1_SLOWDOWN_CLEARED,
                       _levels.SEVERITY_RECOVERABLE,
                       "L1 自动恢复原速失败：%s" % outcome.get("reason"),
                       evidence, outcome.get("next_action") or "")
            return
        self.speed_scaled = False
        self.l1_cleared = True
        # 等级保持 L1（单调不降级）；恢复状态由 speed_scaled 表达
        self._emit(_levels.EVENT_L1_SLOWDOWN_CLEARED,
                   _levels.SEVERITY_RECOVERABLE,
                   "跟随误差回落到告警阈值内 → 已自动恢复原速（L1 唯一自动恢复）",
                   evidence, outcome.get("next_action") or "")

    @staticmethod
    def _merge_evidence(decision: _policy.Decision,
                        result: ActionResult) -> List[Dict[str, Any]]:
        """合并判据证据与动作证据（统一为 list[dict]）。"""
        evidence: List[Dict[str, Any]] = []
        for item in decision.evidence:
            if isinstance(item, Mapping):
                evidence.append(dict(item))
        for item in result.evidence:
            evidence.append(dict(item))
        return evidence

    def _emit(self, event_type: str, severity: str, reason: str,
              evidence: List[Dict[str, Any]], next_action: str) -> None:
        """发出一条事件（自留副本 + 交给宿主回调）。"""
        record = {
            "type": event_type,
            "severity": severity,
            "reason": reason,
            "evidence": [dict(item) for item in evidence],
            "next_action": next_action,
        }
        self.event_count += 1
        self.events.append(record)
        if self._max_events is not None and len(self.events) > self._max_events:
            del self.events[0]
        if self.on_event is None:
            return
        try:
            self.on_event(event_type, severity, reason,
                          record["evidence"], next_action)
        except Exception as exc:  # noqa: BLE001 - 回调异常不得打断安全动作
            self.event_errors.append("%s 回调异常（%s）: %s"
                                     % (event_type, type(exc).__name__, exc))
