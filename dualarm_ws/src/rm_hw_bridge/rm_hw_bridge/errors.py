"""rm_hw_bridge 统一错误模型：**分级 + 可粘贴的下一步**。

本模块定义整个硬件适配层唯一的错误表达：

.. code-block:: python

    {
      "status": "ok" | "blocked" | "error",
      "reason": str,        # 发生了什么（必须写明接口名 / 返回码 / 缺失清单）
      "next_action": str,   # **可粘贴**的下一步命令或操作（必须非空）
      "evidence": [ ... ],  # 客观证据：路径、返回码、缺失符号清单……
    }

分级语义（与 S01 的 rmctl 分级约定一致）：

===========  =====  ======================================================
status       退出码  含义
===========  =====  ======================================================
``ok``         0     操作成功，结果可信
``blocked``    3     需要人工 / 外部条件（缺 SDK、缺文件、需真机窗口）
``error``      4     运行错误（臂不可达、SDK 返回非 0、参数非法）
===========  =====  ======================================================

**硬约束**：``blocked`` / ``error`` 的结果**必须**带非空的 ``next_action``，
且**绝不**携带任何伪造的数值（不造关节角、不造状态、不造"已连通"）。
"""
from __future__ import annotations

from typing import Any, Dict, Iterable, List, Optional

__all__ = [
    "EXIT_CODES",
    "STATUS_BLOCKED",
    "STATUS_ERROR",
    "STATUS_OK",
    "blocked",
    "error",
    "exit_code",
    "is_failure",
    "ok",
    "result",
]

#: 成功
STATUS_OK = "ok"
#: 需要人工 / 外部条件（缺 SDK、真机窗口、基线 blocked……）
STATUS_BLOCKED = "blocked"
#: 运行错误（臂不可达、SDK 返回非 0、参数非法……）
STATUS_ERROR = "error"

#: status → 进程退出码（与 rmctl 的 0/2/3/4 分级一致；2 = 输入非法，由调用方判定）
EXIT_CODES: Dict[str, int] = {
    STATUS_OK: 0,
    STATUS_BLOCKED: 3,
    STATUS_ERROR: 4,
}

#: 失败状态集合
FAILURE_STATUSES = (STATUS_BLOCKED, STATUS_ERROR)


def result(
    status: str,
    reason: str = "",
    next_action: str = "",
    evidence: Optional[Iterable[Any]] = None,
    **extra: Any,
) -> Dict[str, Any]:
    """构造统一结果字典。

    ``blocked`` / ``error`` 必须给非空 ``reason`` 与 ``next_action``；
    缺 ``next_action`` 时补一条**可粘贴**的通用兜底动作（不静默丢失）。
    """
    if status not in EXIT_CODES:
        raise ValueError("未知 status: %r（只允许 %s）" % (status, sorted(EXIT_CODES)))
    reason = reason or ""
    next_action = next_action or ""
    if status in FAILURE_STATUSES and not next_action:
        next_action = (
            "查看上条 reason 与 evidence，修复对应条件后重跑同一条命令；"
            "若仍失败，请把 evidence 原样贴给操作者。"
        )
    out: Dict[str, Any] = {
        "status": status,
        "reason": reason,
        "next_action": next_action,
        "evidence": list(evidence or []),
    }
    out.update(extra)
    return out


def ok(reason: str = "", next_action: str = "", evidence: Optional[Iterable[Any]] = None,
       **extra: Any) -> Dict[str, Any]:
    """成功结果。"""
    return result(STATUS_OK, reason, next_action, evidence, **extra)


def blocked(reason: str, next_action: str, evidence: Optional[Iterable[Any]] = None,
            **extra: Any) -> Dict[str, Any]:
    """被阻塞（需要人工 / 外部条件）→ 退出码 3。"""
    return result(STATUS_BLOCKED, reason, next_action, evidence, **extra)


def error(reason: str, next_action: str, evidence: Optional[Iterable[Any]] = None,
          **extra: Any) -> Dict[str, Any]:
    """运行错误 → 退出码 4。"""
    return result(STATUS_ERROR, reason, next_action, evidence, **extra)


def is_failure(res: Dict[str, Any]) -> bool:
    """结果是否为失败（blocked / error）。"""
    return bool(res) and res.get("status") in FAILURE_STATUSES


def exit_code(res: Dict[str, Any]) -> int:
    """结果 → 进程退出码（0 / 3 / 4）。"""
    return EXIT_CODES.get((res or {}).get("status"), EXIT_CODES[STATUS_ERROR])


def evidence_lines(items: Iterable[Any]) -> List[str]:
    """证据条目统一成字符串列表（JSON 友好）。"""
    out: List[str] = []
    for item in items or []:
        out.append(item if isinstance(item, str) else str(item))
    return out
