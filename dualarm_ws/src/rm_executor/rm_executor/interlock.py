"""执行层的**夹爪运动互锁**策略（fail-closed 语义）。

背景（S12/T4）：工位文件 ``workcell/rm_workcell.yaml`` 的 ``gripper:`` 段
有两种截然不同的语义，执行层必须**区分**：

* **工位未配置夹爪**（没有 ``gripper:`` 段）→ **不启用互锁**是正确的
  语义，运动照常下发（行为逐字不变）；
* **工位已配置夹爪但闸门不可用**（配置读取失败 / 配置非法 /
  硬件层 ``rm_hw_bridge.gripper_state`` 不可导入）→ **静默降级为「无互锁」
  是不安全降级**，必须 **fail-closed**：拒绝一切运动下发。

判定「工位是否配置夹爪」的**唯一无歧义**判据在硬件层
（``rm_hw_bridge.gripper_config.gripper_declaration`` 的 ``declared`` 布尔，
以及 ``load_gripper_config`` 结果里的 ``gripper_declared`` 字段）——
**不得**靠匹配自由文本 ``reason`` 来分流。

本模块只放**执行层自己**的东西：闸门不可用时的状态名、拒绝原因文案与
一个**永远拒绝运动**的闸门对象（鸭子类型：``check()`` + ``state``）。

状态名与硬件层的运行时互锁态（``GRIPPER_BUSY`` / ``GRIPPER_UNJUDGED``）
**刻意不同**：那两条是「夹爪动作进行中」的运行时互锁态，本模块的
``GATE_UNAVAILABLE`` 是「闸门/配置不可用」的**失效保护**态——两者在报告里
必须可区分。
"""
from __future__ import annotations

from typing import Any, Dict, Iterable, List, Optional

__all__ = [
    "REASON_GATE_UNAVAILABLE",
    "STATE_GATE_UNAVAILABLE",
    "UnavailableMotionGate",
]

#: 闸门不可用时的闸门状态名（**不复用** GRIPPER_BUSY / GRIPPER_UNJUDGED）
STATE_GATE_UNAVAILABLE = "GATE_UNAVAILABLE"

#: 拒绝运动的**固定原因文案**（便于报告检索）
REASON_GATE_UNAVAILABLE = "工位已配置夹爪但闸门不可用 → 拒绝运动"


class UnavailableMotionGate:
    """**永远拒绝运动**的闸门（fail-closed）。

    鸭子类型（与 ``rm_hw_bridge.gripper_state.MotionGate`` 一致）：
    ``check()`` 返回拒绝字典、``state`` 属性给出状态名。

    * ``check()`` **永远**返回拒绝字典（绝不返回 ``None``）→ 执行器在
      阶段 B（预就位）之前就拒绝，**零运动下发**；
    * ``state`` = :data:`STATE_GATE_UNAVAILABLE`（与运行时互锁态可区分）；
    * ``reason`` 指名「工位已配置夹爪但闸门不可用 → 拒绝运动」；
    * ``next_action`` 必须**可粘贴**（不静默丢失）。
    """

    def __init__(self, reason: str = "",
                 next_action: str = "",
                 evidence: Optional[Iterable[Any]] = None) -> None:
        self._reason = reason or REASON_GATE_UNAVAILABLE
        self._next_action = next_action or (
            "修复工位文件 workcell/rm_workcell.yaml::gripper 的配置"
            "（或恢复 rm_hw_bridge.gripper_state 导入）后重跑同一条命令"
        )
        self._evidence: List[Any] = list(evidence or [])

    # -- 只读属性 ----------------------------------------------------------
    @property
    def state(self) -> str:
        """闸门状态名（恒为 :data:`STATE_GATE_UNAVAILABLE`）。"""
        return STATE_GATE_UNAVAILABLE

    @property
    def reason(self) -> str:
        """拒绝原因（指名「闸门不可用」）。"""
        return self._reason

    @property
    def evidence(self) -> List[Any]:
        """支撑证据（配置路径 / 导入错误等）。"""
        return list(self._evidence)

    # -- 闸门检查（永远拒绝） ---------------------------------------------
    def check(self) -> Dict[str, Any]:
        """闸门检查：**永远**返回拒绝字典（不返回 ``None``）。"""
        return {
            "state": STATE_GATE_UNAVAILABLE,
            "reason": self._reason,
            "next_action": self._next_action,
            "evidence": list(self._evidence),
        }

    def __repr__(self) -> str:  # pragma: no cover - 调试辅助
        return "UnavailableMotionGate(reason=%r)" % (self._reason,)
