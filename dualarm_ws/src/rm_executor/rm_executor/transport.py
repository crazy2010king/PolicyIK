"""rm_executor 传输层（透传通道抽象）。

执行器要同时满足两个互相拉扯的要求：

* **零真机可测**（离线代理证据）；
* **能无缝切到真机**（同一套代码路径、同一份报告 schema）。

因此透传通道先抽象成 :class:`Transport` 接口，再分别给出两个实现：

* :class:`FakeTransport` —— 离线假传输，记录全部调用、可脚本化注入故障
  （供 T4/T5/T8 的故障注入复用）；
* :class:`BridgeTransport` —— 真机实现，**只包住** ``rm_hw_bridge.bridge.ArmLink``
  已有的 ``preposition_movej`` / ``transmit_joint`` / ``stop`` / ``slow_stop`` /
  ``pause`` / ``continue_`` / ``current_trajectory_type``，**不重复实现任何
  SDK 调用**（S02 已交付）。

三条硬约束：

1. **单位是度**：接口入参/出参一律度（°）。rad→deg 的换算由**调用方**完成，
   且必须用 ``rm_hw_bridge`` 的 ``RAD2DEG`` **单点**换算——本层**不另开换算处**；
2. **统一结果字典**：所有方法返回 ``{"status", "reason", "next_action",
   "evidence", ...}``（与 ``rm_hw_bridge.errors`` 同构），失败路径**不抛裸异常**、
   **不伪造数值**；
3. **拒绝路径 0 次 SDK 调用**：参数非法 / ArmLink 不可用时**不得**触碰 SDK。

**零 rmctl 依赖、零 tesseract 依赖**（执行层不得反向依赖体验层）。
"""
from __future__ import annotations

import math
from abc import ABC, abstractmethod
from typing import Any, Callable, Dict, List, Optional, Tuple

from rm_executor.contract import (
    ARM_DOF,
    blocked_result,
    error_result,
    ok_result,
    result_dict,
)

__all__ = [
    "ARM_DOF",
    "BridgeTransport",
    "FakeTransport",
    "Transport",
    "make_live_transport",
    "validate_deg7",
]

#: 可粘贴的下一步：先建立机械臂连接
NEXT_ACTION_CONNECT = (
    "先建立机械臂连接再执行："
    "`python3 -m rmctl.cli connect --arm left --json`；"
    "确认控制器上电且 ip:port 可达（同一 ip:port 只允许一个客户端）"
)

#: 可粘贴的下一步：把硬件适配层加入 PYTHONPATH
NEXT_ACTION_HW_BRIDGE_PATH = (
    "export PYTHONPATH=$PWD/dualarm_ws/src/rm_hw_bridge:$PYTHONPATH  "
    "# 真机传输通道由 rm_hw_bridge.bridge.ArmLink 提供"
)


# --------------------------------------------------------------------- 校验
def validate_deg7(deg7: Any) -> Tuple[Optional[List[float]], str]:
    """关节角入参校验（**单位：度**）。

    返回 ``(values, "")`` 或 ``(None, reason)``；``reason`` 非空即拒绝，
    且拒绝路径**必须 0 次 SDK 调用**（不静默、不猜测）。
    """
    if deg7 is None:
        return None, "关节角输入为 None"
    if isinstance(deg7, (str, bytes)):
        return None, "关节角输入必须是 %d 个数字的序列（收到 %s）" % (
            ARM_DOF, type(deg7).__name__,
        )
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
        fval = float(item)
        if not math.isfinite(fval):
            return None, "第 %d 个关节角不是有限数（%r）" % (i + 1, item)
        values.append(fval)
    return values, ""


# ------------------------------------------------------------------- 接口
class Transport(ABC):
    """透传通道接口（**单位：度**）。

    子类必须实现全部方法，且**全部返回统一结果字典**
    （``status`` / ``reason`` / ``next_action`` / ``evidence``）。
    """

    #: 通道名（报告里的 ``mode`` / evidence 用）
    name = "transport"

    # -- 运动 -------------------------------------------------------------
    @abstractmethod
    def preposition(self, deg7: Any, speed: Any) -> Dict[str, Any]:
        """预就位（关节空间**规划**运动）。

        参数
        ----
        deg7
            7 个关节角（**度**）。
        speed
            速度百分比（1..100）。真机实现按 SDK 要求取整（``rm_movej`` 的
            ``v`` 是整数百分比）。
        """

    @abstractmethod
    def transmit(self, deg7: Any, follow: bool = False,
                 trajectory_mode: int = 1, radio: int = 30) -> Dict[str, Any]:
        """关节角透传（CANFD 直发，**每 tick 一次**）。

        参数
        ----
        deg7
            7 个关节角（**度**）。
        follow
            高跟随（True）/ 低跟随（False）。
        trajectory_mode
            0 完全透传 / 1 曲线拟合 / 2 滤波。
        radio
            平滑系数（曲线拟合 0..100、滤波 0..1000）。
        """

    # -- 安全 -------------------------------------------------------------
    @abstractmethod
    def stop(self) -> Dict[str, Any]:
        """急停。"""

    @abstractmethod
    def slow_stop(self) -> Dict[str, Any]:
        """缓停。"""

    @abstractmethod
    def pause(self) -> Dict[str, Any]:
        """暂停。"""

    @abstractmethod
    def continue_(self) -> Dict[str, Any]:
        """继续。"""

    # -- 状态 -------------------------------------------------------------
    @abstractmethod
    def current_trajectory_type(self) -> Dict[str, Any]:
        """查询控制器当前规划类型（**不得**在 tick 线程内调用）。"""

    # -- 共用校验 ---------------------------------------------------------
    def _checked_deg7(self, deg7: Any) -> Tuple[Optional[List[float]], Optional[Dict[str, Any]]]:
        """关节角校验。

        返回 ``(values, None)``（合法）或 ``(None, error_result)``
        （非法；拒绝路径 **0 次 SDK 调用**）。合法时 ``values`` 是**拷贝**，
        因此生成器入参也不会被消费两遍。
        """
        values, reason = validate_deg7(deg7)
        if reason:
            return None, error_result(
                reason,
                "传入 %d 个关节角（单位：度）后重跑" % ARM_DOF,
                [],
                sdk_calls=0,
            )
        return values, None

    def to_dict(self) -> Dict[str, Any]:
        """通道自描述（报告用）。"""
        return {"name": self.name}


# --------------------------------------------------------------- 离线假传输
class FakeTransport(Transport):
    """离线假传输（**零真机依赖**）。

    行为：

    * 记录**全部有效调用**（方法名 + 参数 + 结果）到 :attr:`calls`；
    * 参数非法 → 返回 error 结果且**不记录调用**（拒绝路径 0 次下行）；
    * ``script`` 可脚本化失败，供 T4/T5/T8 注入故障。

    ``script`` 的取值形式（键 = 方法名）：

    * ``int`` —— 简写为「返回码非 0」的失败（rc=该值）；
    * ``dict`` —— 含 ``status`` / ``reason`` / ``next_action`` / ``evidence``
      的结果字典（其余键原样附加）；
    * ``callable`` —— 形如 ``f(method, args) -> dict | None``，返回 ``None``
      表示走默认成功路径。

    默认成功结果：``status="ok"``，evidence 标明「FakeTransport（离线）」。
    """

    name = "fake"

    def __init__(self, script: Optional[Dict[str, Any]] = None,
                 max_records: Optional[int] = None) -> None:
        """
        参数
        ----
        script
            脚本化故障注入表（见类文档）。
        max_records
            调用记录的**保留上限**（``None`` = 不限）。离线长跑（如 600 s
            @100 Hz = 6 万条）时建议给个上限，避免内存无界增长；**计数不受
            影响**（:meth:`count` 用的是独立计数器，永不淘汰）。
        """
        self._script: Dict[str, Any] = dict(script or {})
        self._max_records = None if max_records is None else int(max_records)
        self._counts: Dict[str, int] = {}
        self.calls: List[Dict[str, Any]] = []

    # -- 调用记录 ---------------------------------------------------------
    @property
    def call_count(self) -> int:
        """已记录的有效调用总数（**含已被淘汰的**，计数不受上限影响）。"""
        return sum(self._counts.values())

    def calls_for(self, method: str) -> List[Dict[str, Any]]:
        """按方法名过滤调用记录（可能已被上限淘汰掉一部分）。"""
        return [c for c in self.calls if c["method"] == method]

    def count(self, method: str) -> int:
        """某方法的调用次数（**永不受 max_records 淘汰影响**）。"""
        return self._counts.get(method, 0)

    def last_call(self, method: str = None) -> Optional[Dict[str, Any]]:
        """最后一次（可按方法过滤）调用记录。"""
        items = self.calls if method is None else self.calls_for(method)
        return items[-1] if items else None

    def clear(self) -> None:
        """清空调用记录与计数。"""
        self.calls = []
        self._counts = {}

    # -- 脚本化结果 -------------------------------------------------------
    def _scripted(self, method: str, args: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        spec = self._script.get(method)
        if spec is None:
            return None
        if isinstance(spec, bool):
            return error_result(
                "%s 脚本化失败（布尔脚本值不合法）" % method,
                "把 %s 的脚本值改成 dict / int / callable 后重跑" % method,
                [],
                script_type=type(spec).__name__,
            )
        if isinstance(spec, int):
            return error_result(
                "%s 脚本化失败（rc=%d）" % (method, spec),
                "恢复 %s 的正常下发条件后重跑" % method,
                ["%s rc=%d" % (method, spec)],
                rc=spec,
            )
        if isinstance(spec, dict):
            payload = dict(spec)
            status = payload.pop("status", "error")
            reason = payload.pop("reason", "%s 脚本化失败" % method)
            next_action = payload.pop(
                "next_action", "恢复 %s 的正常下发条件后重跑" % method
            )
            evidence = payload.pop("evidence", [])
            return result_dict(status, reason, next_action, evidence, **payload)
        if callable(spec):
            out = spec(method, dict(args))
            if out is None:
                return None
            if not isinstance(out, dict) or "status" not in out:
                return error_result(
                    "%s 脚本返回的结果格式不合法（必须是含 status 的字典，当前 %r）"
                    % (method, out),
                    "让 %s 的脚本返回含 status/reason/next_action 的字典" % method,
                    [],
                )
            return dict(out)
        return error_result(
            "%s 脚本值类型不支持：%s" % (method, type(spec).__name__),
            "把 %s 的脚本值改成 dict / int / callable 后重跑" % method,
            [],
        )

    def _record(self, method: str, args: Dict[str, Any],
                result: Dict[str, Any]) -> Dict[str, Any]:
        """记录一次有效调用并返回结果（调用记录可设上限，计数不受影响）。"""
        self.calls.append({"method": method, "args": dict(args), "result": result})
        self._counts[method] = self._counts.get(method, 0) + 1
        if self._max_records is not None and len(self.calls) > self._max_records:
            del self.calls[: len(self.calls) - self._max_records]
        return result

    # -- Transport 接口 ---------------------------------------------------
    def preposition(self, deg7: Any, speed: Any) -> Dict[str, Any]:
        values, reject = self._checked_deg7(deg7)
        if reject is not None:
            return reject
        args = {"deg7": list(values), "speed": speed}
        scripted = self._scripted("preposition", args)
        if scripted is not None:
            return self._record("preposition", args, scripted)
        return self._record("preposition", args, ok_result(
            "FakeTransport.preposition 已记录（离线，未触达控制器）",
            "",
            ["fake transport: preposition"],
            deg7=list(values),
            speed=speed,
        ))

    def transmit(self, deg7: Any, follow: bool = False,
                 trajectory_mode: int = 1, radio: int = 30) -> Dict[str, Any]:
        values, reject = self._checked_deg7(deg7)
        if reject is not None:
            return reject
        args = {
            "deg7": list(values),
            "follow": bool(follow),
            "trajectory_mode": trajectory_mode,
            "radio": radio,
        }
        scripted = self._scripted("transmit", args)
        if scripted is not None:
            return self._record("transmit", args, scripted)
        return self._record("transmit", args, ok_result(
            "FakeTransport.transmit 已记录（离线，未触达控制器）",
            "",
            ["fake transport: transmit"],
            deg7=list(values),
            follow=bool(follow),
            trajectory_mode=trajectory_mode,
            radio=radio,
        ))

    def stop(self) -> Dict[str, Any]:
        return self._record("stop", {}, self._scripted("stop", {}) or ok_result(
            "FakeTransport.stop 已记录（离线，未触达控制器）",
            "", ["fake transport: stop"]))

    def slow_stop(self) -> Dict[str, Any]:
        return self._record("slow_stop", {},
                            self._scripted("slow_stop", {}) or ok_result(
            "FakeTransport.slow_stop 已记录（离线，未触达控制器）",
            "", ["fake transport: slow_stop"]))

    def pause(self) -> Dict[str, Any]:
        return self._record("pause", {}, self._scripted("pause", {}) or ok_result(
            "FakeTransport.pause 已记录（离线，未触达控制器）",
            "", ["fake transport: pause"]))

    def continue_(self) -> Dict[str, Any]:
        return self._record("continue_", {},
                            self._scripted("continue_", {}) or ok_result(
            "FakeTransport.continue_ 已记录（离线，未触达控制器）",
            "", ["fake transport: continue_"]))

    def current_trajectory_type(self) -> Dict[str, Any]:
        scripted = self._scripted("current_trajectory_type", {})
        if scripted is not None:
            return self._record("current_trajectory_type", {}, scripted)
        return self._record("current_trajectory_type", {}, ok_result(
            "FakeTransport.current_trajectory_type 已记录（离线，恒为无规划）",
            "", ["fake transport: current_trajectory_type"],
            type=0,
            type_name="RM_NO_PLANNING_E",
            planning=False,
        ))


# --------------------------------------------------------------- 真机传输
def make_live_transport(ip: Optional[str] = None,
                        port: Optional[int] = None,
                        arm: str = "left",
                        link: Any = None,
                        **connect_kwargs: Any) -> BridgeTransport:
    """真机传输通道工厂（**与 fake 同一套代码路径**）。

    * 已给定 ``link`` → 直接包住它；
    * 否则**惰性**建链：首次使用时才 import ``rm_hw_bridge.bridge.connect``
      （建链失败 / 硬件层不可用 → :class:`BridgeTransport` 返回
      ``status="blocked"`` + 可粘贴 next_action，**0 次 SDK 调用**、
      **不抛裸异常**）。

    本函数**不引入第二处 SDK 调用**——建链完全委托给硬件层的
    ``rm_hw_bridge.bridge.connect``。
    """
    if link is not None:
        return BridgeTransport(link=link)

    def _factory() -> Any:
        from rm_hw_bridge import bridge as hw_bridge  # 惰性导入硬件层

        if not ip:
            raise ValueError("未提供控制器 ip（真机路径必须显式给出 ip:port）")
        return hw_bridge.connect(ip, int(port or hw_bridge.DEFAULT_TCP_PORT),
                                 arm=arm, **connect_kwargs)

    return BridgeTransport(link_factory=_factory)


class BridgeTransport(Transport):
    """真机传输通道（**包住** ``rm_hw_bridge.bridge.ArmLink``，零 SDK 实现）。

    本类**不重复实现任何 SDK 调用**：所有动作都转发给 ``ArmLink`` 已有的
    方法（``preposition_movej`` / ``transmit_joint`` / ``stop`` /
    ``slow_stop`` / ``pause`` / ``continue_`` / ``current_trajectory_type``）。

    ArmLink 不可用（未连接 / 构造失败 / import 失败）时：返回
    ``status="blocked"`` + 非空 reason + **可粘贴** next_action，
    **0 次 SDK 调用**，**不抛裸异常**。
    """

    name = "bridge"

    def __init__(self, link: Any = None,
                 link_factory: Optional[Callable[[], Any]] = None) -> None:
        """
        参数
        ----
        link
            已建立的 ``ArmLink``（可为 ``None``）。
        link_factory
            可选的连接工厂（真机入口用它惰性建链）。工厂抛异常时记为
            「不可用」，**不抛裸异常**。
        """
        self._link = link
        self._link_factory = link_factory
        self._link_error = ""

    # -- 连接解析 ---------------------------------------------------------
    @property
    def link(self) -> Any:
        """当前 ArmLink（可能为 None）。"""
        return self._link

    def _resolve_link(self) -> Any:
        """解析可用 ArmLink；失败返回 None（并记录原因）。"""
        if self._link is not None:
            return self._link
        if self._link_factory is None:
            self._link_error = self._link_error or "尚未建立连接（ArmLink 为空）"
            return None
        try:
            link = self._link_factory()
        except Exception as exc:  # noqa: BLE001 - 建链失败统一降级为 blocked
            self._link_error = "建立连接失败（%s）: %s" % (type(exc).__name__, exc)
            return None
        if link is None:
            self._link_error = self._link_error or "连接工厂未返回 ArmLink"
            return None
        self._link = link
        return link

    def _unavailable(self, action: str) -> Dict[str, Any]:
        """ArmLink 不可用 → blocked 结果（**0 次 SDK 调用**）。"""
        reason = "ArmLink 不可用（%s），无法执行 %s" % (
            self._link_error or "未连接", action,
        )
        return blocked_result(
            reason,
            NEXT_ACTION_CONNECT,
            ["ArmLink 不可用：%s" % (self._link_error or "未连接")],
            sdk_calls=0,
            arm_link_available=False,
        )

    def _not_connected(self, link: Any, action: str) -> Dict[str, Any]:
        """ArmLink 存在但未连接 → blocked 结果（**0 次 SDK 调用**）。"""
        reason = getattr(link, "reason", "") or "机械臂未连接，无法执行 %s" % action
        next_action = getattr(link, "next_action", "") or NEXT_ACTION_CONNECT
        evidence = list(getattr(link, "evidence", []) or [])
        evidence.append("ArmLink.connected = False")
        return blocked_result(reason, next_action, evidence, sdk_calls=0,
                              arm_link_available=False)

    @staticmethod
    def _normalize(out: Any, action: str) -> Dict[str, Any]:
        """把 ArmLink 的返回值规范化成统一结果字典（缺字段则补齐）。"""
        if not isinstance(out, dict):
            return error_result(
                "%s 返回值格式不合法（%s）" % (action, type(out).__name__),
                "检查 rm_hw_bridge 版本是否与执行层匹配后重跑",
                [repr(out)],
                sdk_calls=1,
            )
        if "status" not in out:
            return error_result(
                "%s 返回值缺少 status 字段" % action,
                "检查 rm_hw_bridge 版本是否与执行层匹配后重跑",
                [repr(out)],
                sdk_calls=1,
            )
        return result_dict(
            out.get("status", "error"),
            out.get("reason", ""),
            out.get("next_action", ""),
            out.get("evidence", []),
            **{k: v for k, v in out.items()
               if k not in ("status", "reason", "next_action", "evidence")},
        )

    # -- Transport 接口 ---------------------------------------------------
    def preposition(self, deg7: Any, speed: Any) -> Dict[str, Any]:
        values, reject = self._checked_deg7(deg7)
        if reject is not None:
            return reject
        # rm_movej 的 speed 是整数百分比（1..100）
        try:
            speed_num = float(speed)
        except (TypeError, ValueError):
            return error_result(
                "speed 必须是数值（当前 %r）" % (speed,),
                "传入 1..100 的速度百分比后重跑",
                [], sdk_calls=0,
            )
        if not math.isfinite(speed_num) or not (1.0 <= speed_num <= 100.0):
            return error_result(
                "speed=%r 越界（允许 1..100 的速度百分比）" % (speed,),
                "把 pre_position_speed 改到 1..100 后重跑",
                [], sdk_calls=0,
            )
        speed_int = int(round(speed_num))
        link = self._resolve_link()
        if link is None:
            return self._unavailable("预就位运动")
        if not getattr(link, "connected", False):
            return self._not_connected(link, "预就位运动")
        try:
            out = link.preposition_movej(values, speed=speed_int)
        except Exception as exc:  # noqa: BLE001 - SDK 异常统一转 error 结果
            return error_result(
                "preposition_movej 调用异常（%s）: %s" % (type(exc).__name__, exc),
                "确认控制器在线、SDK 与当前架构匹配后重跑",
                [repr(exc)], sdk_calls=1,
            )
        return self._normalize(out, "预就位运动")

    def transmit(self, deg7: Any, follow: bool = False,
                 trajectory_mode: int = 1, radio: int = 30) -> Dict[str, Any]:
        values, reject = self._checked_deg7(deg7)
        if reject is not None:
            return reject
        link = self._resolve_link()
        if link is None:
            return self._unavailable("关节角透传")
        if not getattr(link, "connected", False):
            return self._not_connected(link, "关节角透传")
        try:
            out = link.transmit_joint(
                values,
                follow=bool(follow),
                trajectory_mode=trajectory_mode,
                radio=radio,
            )
        except Exception as exc:  # noqa: BLE001
            return error_result(
                "transmit_joint 调用异常（%s）: %s" % (type(exc).__name__, exc),
                "确认控制器在线、SDK 与当前架构匹配后重跑",
                [repr(exc)], sdk_calls=1,
            )
        return self._normalize(out, "关节角透传")

    def _safety_call(self, method: str, label: str) -> Dict[str, Any]:
        link = self._resolve_link()
        if link is None:
            return self._unavailable(label)
        if not getattr(link, "connected", False):
            return self._not_connected(link, label)
        try:
            out = getattr(link, method)()
        except Exception as exc:  # noqa: BLE001
            return error_result(
                "%s 调用异常（%s）: %s" % (method, type(exc).__name__, exc),
                "确认控制器在线后重跑；必要时先做一次 %s 复位" % method,
                [repr(exc)], sdk_calls=1,
            )
        return self._normalize(out, label)

    def stop(self) -> Dict[str, Any]:
        return self._safety_call("stop", "急停")

    def slow_stop(self) -> Dict[str, Any]:
        return self._safety_call("slow_stop", "缓停")

    def pause(self) -> Dict[str, Any]:
        return self._safety_call("pause", "暂停")

    def continue_(self) -> Dict[str, Any]:
        return self._safety_call("continue_", "继续")

    def current_trajectory_type(self) -> Dict[str, Any]:
        link = self._resolve_link()
        if link is None:
            return self._unavailable("规划类型查询")
        if not getattr(link, "connected", False):
            return self._not_connected(link, "规划类型查询")
        try:
            out = link.current_trajectory_type()
        except Exception as exc:  # noqa: BLE001
            return error_result(
                "current_trajectory_type 调用异常（%s）: %s"
                % (type(exc).__name__, exc),
                "确认控制器在线后重跑",
                [repr(exc)], sdk_calls=1,
            )
        return self._normalize(out, "规划类型查询")
