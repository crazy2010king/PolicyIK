"""工艺客户端抽象层（Planner / MotionClient / GripperClient）+ fake / live 实现。

三条职责红线（本模块的**硬边界**，由 ``tests/test_process_clients.py`` 守卫）
----------------------------------------------------------------------------

1. **零 SDK 直调**：本模块**不** import ``ctypes`` / ``rm_hw_bridge.generated``
   / 厂商 SDK；夹爪动作与状态读取只经 ``rm_hw_bridge.gripper.GripperAdapter``，
   运动执行只经 ``rm_executor``。
2. **零判定逻辑复制**：判定**唯一**实现在
   ``rm_hw_bridge.gripper_state.judge(state, thresholds)``；本模块只调用它，
   **不**重写「夹到 / 夹空 / 堵转 / 过温 / 过流 / 离线」的判据。
3. **零严重度排序实现**：严重度等级与恢复阶梯的唯一真值在
   ``rm_hw_bridge.gripper_state``；本模块**不**比较严重度大小、不排序。
4. **零阈值副本**：判定阈值**唯一**来源 =
   ``rmctl.gripper_proxy.effective_thresholds(...)``；本模块**不**写死任何
   阈值数值，也**不**自行补默认值（缺标定 → 判定自然落到 ``needs_param``）。

规划真值仍在 ``rmctl.planner`` / ``rmctl.verify``（含保路径降速重定时阶梯），
执行真值仍在 ``rm_executor``（统一报告 schema）—— 本模块只做**协议 + 接线**。

三个协议
--------

``Planner``
    ``plan_segment(start_deg, goal_deg=None, goal_pose=None) -> dict``
    产出**符合** ``rm_executor.contract.load_trajectory`` 的轨迹 JSON
    （times 严格递增 / points 每行 7 个有限数 / joint_names=joint1..7 /
    group=rm75_arm / units={"times":"s","points":"rad"} / schema_version=1 /
    verify.ok is True）。``goal_deg`` 是**点到点**关节目标（**度**），
    ``goal_pose`` 是**笛卡尔**目标 ``[x,y,z,r,p,y]``（米 + 弧度），二者互斥。
    度→弧度的换算在本层完成（**唯一换算点**）。
``MotionClient``
    ``run_trajectory(traj_path) -> dict``：返回 ``rm_executor`` 的**统一报告**
    （顶层 7 键：status/reason/next_action/evidence/mode/metrics/events）。
``GripperClient``
    ``act(action, speed=None, force=None) -> dict`` 与 ``judge() -> dict``；
    ``judge()`` 返回 ``rm_hw_bridge.gripper_state.Judgment.to_dict()`` 形态
    （result/severity/reason/evidence/next_action/retryable）。
    **阈值由构造参数注入**（本层不读工位配置、不写阈值）。
"""
from __future__ import annotations

import json
import math
import os
import sys
from typing import Any, Callable, Dict, List, Optional, Sequence

try:  # Python 3.8+
    from typing import Protocol, runtime_checkable
except ImportError:  # pragma: no cover - 老解释器降级
    Protocol = object  # type: ignore

    def runtime_checkable(cls):  # type: ignore
        return cls


from rmctl import paths
from rmctl.gripper_proxy import GRIPPER_PROXY_NOTE

__all__ = [
    "FakeGripperClient",
    "FakeMotionClient",
    "FakePlanner",
    "GripperClient",
    "LiveGripperClient",
    "LiveMotionClient",
    "LivePlanner",
    "MotionClient",
    "Planner",
    "save_trajectory",
    "trajectory_payload",
]


# --------------------------------------------------------------------------
# 惰性依赖（硬件层 / 执行层 / 规划内核）
# --------------------------------------------------------------------------

def _import_hw():
    """惰性导入硬件层（``gripper`` / ``gripper_state``）。

    返回 ``(mods, error_reason)``；``mods`` 为 ``None`` 时 ``error_reason``
    说明原因（**不把 ImportError 堆栈甩给操作者**）。
    """
    def _import():
        from rm_hw_bridge import gripper as _gripper
        from rm_hw_bridge import gripper_state as _gripper_state

        return {"gripper": _gripper, "state": _gripper_state}

    try:
        return _import(), ""
    except Exception as exc:  # noqa: BLE001 - 统一降级
        first = repr(exc)
    root = paths.workspace_root()
    pkg_parent = os.path.join(root, "dualarm_ws", "src", "rm_hw_bridge")
    if os.path.isdir(pkg_parent) and pkg_parent not in sys.path:
        sys.path.insert(0, pkg_parent)
    try:
        return _import(), ""
    except Exception as exc:  # noqa: BLE001
        return None, "%s / 兜底路径 %s: %r" % (first, pkg_parent, exc)


def _import_executor():
    """惰性导入执行层（``rm_executor.cli`` / ``rm_executor.contract``）。"""
    def _import():
        from rm_executor import cli as _exec_cli
        from rm_executor import contract as _contract

        return {"cli": _exec_cli, "contract": _contract}

    try:
        return _import(), ""
    except Exception as exc:  # noqa: BLE001
        first = repr(exc)
    root = paths.workspace_root()
    pkg_parent = os.path.join(root, "dualarm_ws", "src", "rm_executor")
    if os.path.isdir(pkg_parent) and pkg_parent not in sys.path:
        sys.path.insert(0, pkg_parent)
    try:
        return _import(), ""
    except Exception as exc:  # noqa: BLE001
        return None, "%s / 兜底路径 %s: %r" % (first, pkg_parent, exc)


def _executor_contract():
    """取 ``rm_executor.contract`` 模块（不可用 → ``None``）。"""
    mods, _err = _import_executor()
    return mods["contract"] if mods else None


def _py_path_hint(pkg: str) -> str:
    """可粘贴的 PYTHONPATH 修复命令。"""
    return ("export PYTHONPATH=%s:$PYTHONPATH  # 让 rmctl 看到 %s"
            % (os.path.join(paths.workspace_root(), "dualarm_ws", "src", pkg), pkg))


# --------------------------------------------------------------------------
# 结果构造
# --------------------------------------------------------------------------

def _ok(reason: str, evidence: Optional[Sequence[Any]] = None, **extra: Any
        ) -> Dict[str, Any]:
    out = {
        "status": "ok",
        "reason": reason,
        "next_action": "",
        "evidence": list(evidence or []),
    }
    out.update(extra)
    return out


def _blocked(reason: str, next_action: str, evidence: Optional[Sequence[Any]] = None,
             **extra: Any) -> Dict[str, Any]:
    out = {
        "status": "blocked",
        "reason": reason,
        "next_action": next_action or "按上条 reason 修复后重跑同一条命令",
        "evidence": list(evidence or []),
    }
    out.update(extra)
    return out


# --------------------------------------------------------------------------
# 轨迹载荷（契约字段**只从 rm_executor.contract 取**，不复制常量）
# --------------------------------------------------------------------------

def trajectory_payload(times: Sequence[float], points: Sequence[Sequence[float]],
                       verify: Dict[str, Any]) -> Dict[str, Any]:
    """构造符合 ``rm_executor.contract.load_trajectory`` 的轨迹 JSON。

    字段（schema_version / group / joint_names / units）**唯一来源** =
    ``rm_executor.contract``（执行层契约），本模块**不**另写一份常量。

    :raises RuntimeError: 执行层不可用时（调用方转成 blocked）。
    """
    contract = _executor_contract()
    if contract is None:
        raise RuntimeError("rm_executor 不可用")
    return {
        "schema_version": contract.SCHEMA_VERSION,
        "group": contract.JOINT_GROUP,
        "joint_names": list(contract.JOINT_NAMES),
        "units": dict(contract.UNITS),
        "times": [float(t) for t in times],
        "points": [[float(v) for v in row] for row in points],
        "verify": dict(verify),
    }


def save_trajectory(plan_result: Dict[str, Any], path: str) -> str:
    """把规划结果里的轨迹 JSON 落盘（供 ``MotionClient.run_trajectory`` 使用）。

    :return: 绝对路径。
    """
    payload = plan_result.get("trajectory")
    if not isinstance(payload, dict):
        raise ValueError("规划结果里没有轨迹载荷（trajectory）")
    target = os.path.abspath(path)
    parent = os.path.dirname(target)
    if parent and not os.path.isdir(parent):
        os.makedirs(parent, exist_ok=True)
    with open(target, "w", encoding="utf-8") as fh:
        json.dump(payload, fh, ensure_ascii=False, indent=2)
    return target


# --------------------------------------------------------------------------
# 协议
# --------------------------------------------------------------------------

@runtime_checkable
class Planner(Protocol):
    """规划协议：把一段运动目标变成**符合执行层契约**的轨迹。"""

    def plan_segment(self, start_deg: Sequence[float],
                     goal_deg: Optional[Sequence[float]] = None,
                     goal_pose: Optional[Sequence[float]] = None) -> Dict[str, Any]:
        ...


@runtime_checkable
class MotionClient(Protocol):
    """运动协议：执行一条轨迹，返回 ``rm_executor`` 统一报告。"""

    def run_trajectory(self, traj_path: str) -> Dict[str, Any]:
        ...


@runtime_checkable
class GripperClient(Protocol):
    """夹爪协议：动作 + 判定（判定真值在硬件层）。"""

    def act(self, action: str, speed: Optional[int] = None,
            force: Optional[int] = None) -> Dict[str, Any]:
        ...

    def judge(self) -> Dict[str, Any]:
        ...


# --------------------------------------------------------------------------
# FakePlanner（离线确定性轨迹）
# --------------------------------------------------------------------------

class FakePlanner:
    """**确定性线性插值**的离线规划器（不依赖厂商规划内核）。

    * 只支持**点到点**（关节目标）；笛卡尔目标在离线代理下 → ``blocked``；
    * **没有** ``verify_fn`` 时**拒绝出轨迹**（返回 blocked，reason 写明
      「未提供校验函数，不得伪造 verify.ok」）——**绝不**合成 ``verify.ok=True``；
    * ``verify_fn(times, points) -> dict`` 的返回值原样写进轨迹的 ``verify``
      字段（``ok`` 不为 True 时 → blocked，不出轨迹）。

    :param verify_fn: 校验函数（注入；缺省 → 不出轨迹）。
    :param samples: 采样点数（≥2）。
    :param duration_s: 轨迹时长（秒）。
    """

    def __init__(self, verify_fn: Optional[Callable[[Sequence[float],
                                                    Sequence[Sequence[float]]],
                                                   Dict[str, Any]]] = None,
                 samples: int = 11, duration_s: float = 1.0) -> None:
        self._verify_fn = verify_fn
        self._samples = max(2, int(samples))
        self._duration_s = float(duration_s)

    # -- 内部 --------------------------------------------------------------
    @staticmethod
    def _as_vector(value: Any, count: int) -> Optional[List[float]]:
        if isinstance(value, (str, bytes)) or not isinstance(value, (list, tuple)):
            return None
        if len(value) != count:
            return None
        out: List[float] = []
        for item in value:
            if isinstance(item, bool) or not isinstance(item, (int, float)):
                return None
            val = float(item)
            if not math.isfinite(val):
                return None
            out.append(val)
        return out

    def _interpolate(self, start: Sequence[float], goal: Sequence[float]
                     ) -> (List[float], List[List[float]]):
        n = self._samples
        times = [self._duration_s * i / float(n - 1) for i in range(n)]
        points: List[List[float]] = []
        for i in range(n):
            ratio = i / float(n - 1)
            row = []
            for j in range(len(start)):
                a = math.radians(start[j])
                b = math.radians(goal[j])
                row.append(a + (b - a) * ratio)
            points.append(row)
        return times, points

    # -- 协议 --------------------------------------------------------------
    def plan_segment(self, start_deg: Sequence[float],
                     goal_deg: Optional[Sequence[float]] = None,
                     goal_pose: Optional[Sequence[float]] = None
                     ) -> Dict[str, Any]:
        """规划一段运动（见 :class:`Planner`）。"""
        if goal_pose is not None and goal_deg is not None:
            return _blocked(
                "运动目标互斥：goal_deg 与 goal_pose 只能给一个",
                "只保留 goal_joints_deg 或 goal_pose 后重跑",
                evidence=["goal_deg=%r" % (list(goal_deg),),
                          "goal_pose=%r" % (list(goal_pose),)],
                times=[], points=[], verify=None, pipeline=None, cached=False,
                trajectory=None,
            )
        if goal_pose is not None:
            return _blocked(
                "离线代理不支持笛卡尔目标（goal_pose）",
                "在离线跑批里改用 goal_joints_deg（点到点）；"
                "笛卡尔目标请走 LivePlanner（真实 IK + 自由空间规划）",
                evidence=["goal_pose=%r" % (list(goal_pose),)],
                times=[], points=[], verify=None, pipeline=None, cached=False,
                trajectory=None,
            )
        if goal_deg is None:
            return _blocked(
                "缺少运动目标（goal_deg / goal_pose 都没给）",
                "在工艺规格里补 goal_joints_deg 或 goal_pose 后重跑",
                evidence=[],
                times=[], points=[], verify=None, pipeline=None, cached=False,
                trajectory=None,
            )
        if self._verify_fn is None:
            return _blocked(
                "未提供校验函数，不得伪造 verify.ok",
                "构造 FakePlanner 时注入 verify_fn(times, points) -> dict"
                "（真实四道校验报告；本层不合成 verify.ok=True）",
                evidence=["FakePlanner 无 verify_fn → 不产出任何轨迹"],
                times=[], points=[], verify=None, pipeline=None, cached=False,
                trajectory=None,
            )

        contract = _executor_contract()
        if contract is None:
            return _blocked(
                "执行层 rm_executor 不可用（拿不到轨迹契约字段）",
                _py_path_hint("rm_executor"),
                evidence=[],
                times=[], points=[], verify=None, pipeline=None, cached=False,
                trajectory=None,
            )
        dof = int(contract.ARM_DOF)
        start = self._as_vector(start_deg, dof)
        goal = self._as_vector(goal_deg, dof)
        if start is None or goal is None:
            return _blocked(
                "起点/目标非法（必须是 %d 个有限数）" % dof,
                "修正工艺规格里的关节角（%d 个有限数）后重跑" % dof,
                evidence=["start=%r" % (list(start_deg),),
                          "goal=%r" % (list(goal_deg),)],
                times=[], points=[], verify=None, pipeline=None, cached=False,
                trajectory=None,
            )

        times, points = self._interpolate(start, goal)
        report = self._verify_fn(times, points)
        if not isinstance(report, dict) or "ok" not in report:
            return _blocked(
                "校验函数返回值非法（必须是含 ok 字段的字典）",
                "让 verify_fn 返回 rmctl.verify.verify 的报告形态后重跑",
                evidence=[repr(report)],
                times=[], points=[], verify=None, pipeline=None, cached=False,
                trajectory=None,
            )
        if report.get("ok") is not True:
            failed = []
            checks = report.get("checks")
            if isinstance(checks, dict):
                failed = [k for k, v in checks.items()
                          if isinstance(v, dict) and v.get("ok") is False]
            return _blocked(
                "四道校验未通过（verify.ok=%r%s）"
                % (report.get("ok"),
                   ("，失败项: %s" % ", ".join(failed)) if failed else ""),
                "修正轨迹后重新规划（不得绕过校验出轨迹）",
                evidence=[repr(report)],
                times=[], points=[], verify=None, pipeline=None, cached=False,
                trajectory=None,
            )

        try:
            payload = trajectory_payload(times, points, report)
        except RuntimeError as exc:
            return _blocked(
                "构造轨迹载荷失败: %r" % (exc,),
                _py_path_hint("rm_executor"),
                evidence=[repr(exc)],
                times=[], points=[], verify=None, pipeline=None, cached=False,
                trajectory=None,
            )
        return _ok(
            "离线线性插值轨迹已生成（%d 个采样点，%.3f s）"
            % (len(times), self._duration_s),
            evidence=["mode=fake（离线代理，不代表真机结论）",
                      "verify 来自注入的 verify_fn（本层不合成）"],
            times=times, points=points, verify=dict(report),
            pipeline="fake_linear", cached=False, trajectory=payload,
        )


# --------------------------------------------------------------------------
# LivePlanner（真实规划 + 四道校验 + 保路径降速阶梯）
# --------------------------------------------------------------------------

class LivePlanner:
    """真机规划客户端：``rmctl.planner`` + ``rmctl.verify`` + 重定时自愈阶梯。

    * **惰性导入**：``rmctl.planner`` 在模块顶层 import 厂商规划内核，本类
      **不在 import 时拉起**厂商内核（只有真正规划时才 import）；
    * 规划失败 / 四道校验仍不过 → ``blocked`` + reason + **可粘贴** next_action，
      **不伪造** ``verify.ok=True``；
    * 重定时阶梯（``required_scale`` → ``scale_trajectory`` → 重新 verify）
      **与** ``rmctl.cli.cmd_plan`` **同一份实现**
      （``rmctl.verify.verify_with_retime``），本类**不复制**；
    * **可选缓存**：key = (起点, 目标)；命中时返回同一条轨迹并标 ``cached=True``
      （确定性规划，缓存不改变语义）。

    :param ws: 工作区根。
    :param model: 已加载的模型（缺省 → 规划时惰性加载）。
    :param plan_joint: 点到点规划函数（缺省 → ``rmctl.planner.plan_joint``）。
    :param plan_pose: 笛卡尔规划函数（缺省 → ``rmctl.planner.plan_pose``）。
    :param verify_with_retime: 校验+重定时函数（缺省 →
        ``rmctl.verify.verify_with_retime``）。
    :param cache: 是否启用缓存。
    """

    def __init__(self, ws: Optional[str] = None, model: Any = None,
                 plan_joint: Optional[Callable[..., Any]] = None,
                 plan_pose: Optional[Callable[..., Any]] = None,
                 verify_with_retime: Optional[Callable[..., Any]] = None,
                 cache: bool = True) -> None:
        self._ws = ws
        self._model = model
        self._plan_joint = plan_joint
        self._plan_pose = plan_pose
        self._verify_with_retime = verify_with_retime
        self._cache_enabled = bool(cache)
        self._cache: Dict[Any, Dict[str, Any]] = {}

    # -- 惰性依赖 ----------------------------------------------------------
    def _load_planner(self):
        try:
            from rmctl import planner as _planner
        except Exception as exc:  # noqa: BLE001 - 厂商内核不可用 → blocked
            return None, "%s: %r" % (type(exc).__name__, exc)
        return _planner, ""

    def _load_verify(self):
        try:
            from rmctl import verify as _verify
        except Exception as exc:  # noqa: BLE001
            return None, "%s: %r" % (type(exc).__name__, exc)
        return _verify, ""

    def _load_model(self):
        if self._model is not None:
            return self._model, ""
        try:
            from rmctl.model import Model
        except Exception as exc:  # noqa: BLE001
            return None, "%s: %r" % (type(exc).__name__, exc)
        try:
            self._model = Model(ws=self._ws)
        except Exception as exc:  # noqa: BLE001 - VendorUnavailable 等
            return None, "%s: %r" % (type(exc).__name__, exc)
        return self._model, ""

    # -- 协议 --------------------------------------------------------------
    def plan_segment(self, start_deg: Sequence[float],
                     goal_deg: Optional[Sequence[float]] = None,
                     goal_pose: Optional[Sequence[float]] = None
                     ) -> Dict[str, Any]:
        """规划一段运动（见 :class:`Planner`）。"""
        if goal_pose is not None and goal_deg is not None:
            return _blocked(
                "运动目标互斥：goal_deg 与 goal_pose 只能给一个",
                "只保留 goal_joints_deg 或 goal_pose 后重跑",
                evidence=[],
                times=[], points=[], verify=None, pipeline=None, cached=False,
                trajectory=None,
            )
        if goal_deg is None and goal_pose is None:
            return _blocked(
                "缺少运动目标（goal_deg / goal_pose 都没给）",
                "在工艺规格里补 goal_joints_deg 或 goal_pose 后重跑",
                evidence=[],
                times=[], points=[], verify=None, pipeline=None, cached=False,
                trajectory=None,
            )

        contract = _executor_contract()
        if contract is None:
            return _blocked(
                "执行层 rm_executor 不可用（拿不到轨迹契约字段）",
                _py_path_hint("rm_executor"),
                evidence=[],
                times=[], points=[], verify=None, pipeline=None, cached=False,
                trajectory=None,
            )
        dof = int(contract.ARM_DOF)

        planner, err = self._load_planner()
        if planner is None:
            return _blocked(
                "厂商规划内核不可用（rmctl.planner 导入失败）: %s" % err,
                "source scripts/rmenv.sh 后重跑"
                "（或确认 tesseract_robotics 与 .deps/libstdcxx 就位）",
                evidence=[err],
                times=[], points=[], verify=None, pipeline=None, cached=False,
                trajectory=None,
            )
        verify_mod, verr = self._load_verify()
        if verify_mod is None:
            return _blocked(
                "轨迹校验模块不可用（rmctl.verify 导入失败）: %s" % verr,
                "source scripts/rmenv.sh 后重跑",
                evidence=[verr],
                times=[], points=[], verify=None, pipeline=None, cached=False,
                trajectory=None,
            )

        start = self._as_vector(start_deg, dof)
        if start is None:
            return _blocked(
                "起点非法（必须是 %d 个有限数）" % dof,
                "修正工艺规格里的起始关节角后重跑",
                evidence=["start=%r" % (list(start_deg),)],
                times=[], points=[], verify=None, pipeline=None, cached=False,
                trajectory=None,
            )

        # -- 缓存查找（确定性规划，缓存不改变语义）----------------------
        key = None
        if self._cache_enabled:
            if goal_deg is not None:
                goal_key = self._as_vector(goal_deg, dof)
                key = ("deg", tuple(start), tuple(goal_key) if goal_key else None)
            else:
                pose_key = self._as_vector(goal_pose, 6)
                key = ("pose", tuple(start), tuple(pose_key) if pose_key else None)
            if key is not None and key in self._cache:
                cached = dict(self._cache[key])
                cached["cached"] = True
                return cached

        model, merr = self._load_model()
        if model is None:
            return _blocked(
                "模型加载失败: %s" % merr,
                "先执行 `python3 -m rmctl init` 物化 URDF/SRDF 后重跑",
                evidence=[merr],
                times=[], points=[], verify=None, pipeline=None, cached=False,
                trajectory=None,
            )

        start_rad = [math.radians(v) for v in start]
        outcome = None
        if goal_deg is not None:
            goal = self._as_vector(goal_deg, dof)
            if goal is None:
                return _blocked(
                    "目标非法（必须是 %d 个有限数）" % dof,
                    "修正工艺规格里的 goal_joints_deg 后重跑",
                    evidence=["goal=%r" % (list(goal_deg),)],
                    times=[], points=[], verify=None, pipeline=None, cached=False,
                    trajectory=None,
                )
            plan_fn = self._plan_joint or planner.plan_joint
            try:
                outcome = plan_fn(model, start_rad, [math.radians(v) for v in goal])
            except Exception as exc:  # noqa: BLE001 - 规划异常 → blocked
                return _blocked(
                    "规划失败: %s: %r" % (type(exc).__name__, exc),
                    "检查目标可达性与碰撞场景后重跑（必要时先 `rmctl plan` 单段验证）",
                    evidence=[repr(exc)],
                    times=[], points=[], verify=None, pipeline=None, cached=False,
                    trajectory=None,
                )
        else:
            pose = self._as_vector(goal_pose, 6)
            if pose is None:
                return _blocked(
                    "笛卡尔目标非法（必须是 6 个有限数 [x,y,z,r,p,y]）",
                    "修正工艺规格里的 goal_pose 后重跑",
                    evidence=["goal_pose=%r" % (list(goal_pose),)],
                    times=[], points=[], verify=None, pipeline=None, cached=False,
                    trajectory=None,
                )
            plan_fn = self._plan_pose or planner.plan_pose
            try:
                pose_obj = self._make_pose(pose)
                outcome = plan_fn(model, start_rad, pose_obj)
            except Exception as exc:  # noqa: BLE001
                return _blocked(
                    "笛卡尔规划失败: %s: %r" % (type(exc).__name__, exc),
                    "检查目标位姿可达性后重跑（或改用 goal_joints_deg 点到点）",
                    evidence=[repr(exc)],
                    times=[], points=[], verify=None, pipeline=None, cached=False,
                    trajectory=None,
                )

        ok_flag = bool(getattr(outcome, "ok", False))
        message = str(getattr(outcome, "message", "") or "")
        times = list(getattr(outcome, "times", []) or [])
        points = [list(row) for row in (getattr(outcome, "points", []) or [])]
        pipeline = getattr(outcome, "pipeline", None)
        if not ok_flag or not times or not points:
            return _blocked(
                "规划未成功: %s" % (message or "规划器未返回轨迹"),
                "检查目标可达性 / 碰撞场景后重跑"
                "（可先用 `python3 -m rmctl.cli plan --goal ...` 单段复现）",
                evidence=["pipeline=%r" % (pipeline,), "message=%s" % message],
                times=[], points=[], verify=None, pipeline=pipeline, cached=False,
                trajectory=None,
            )

        verify_fn = self._verify_with_retime or getattr(
            verify_mod, "verify_with_retime", None)
        if verify_fn is None:
            return _blocked(
                "rmctl.verify 未提供 verify_with_retime（重定时阶梯的唯一实现）",
                "检查 rmctl.verify 版本后重跑（本层不复制阶梯逻辑）",
                evidence=[],
                times=[], points=[], verify=None, pipeline=pipeline, cached=False,
                trajectory=None,
            )
        try:
            report, times, points, scale = verify_fn(model, points, times)
        except Exception as exc:  # noqa: BLE001
            return _blocked(
                "四道校验异常: %s: %r" % (type(exc).__name__, exc),
                "检查模型与限值配置后重跑",
                evidence=[repr(exc)],
                times=[], points=[], verify=None, pipeline=pipeline, cached=False,
                trajectory=None,
            )
        if not isinstance(report, dict) or report.get("ok") is not True:
            failed = []
            checks = (report or {}).get("checks")
            if isinstance(checks, dict):
                failed = [k for k, v in checks.items()
                          if isinstance(v, dict) and v.get("ok") is False]
            return _blocked(
                "四道校验未通过（verify.ok=%r%s）"
                % ((report or {}).get("ok"),
                   ("，失败项: %s" % ", ".join(failed)) if failed else ""),
                "按失败项修正目标/场景后重跑（保路径降速已尝试仍不过；"
                "本层不伪造 verify.ok=True）",
                evidence=["scale=%r" % (scale,)],
                times=[], points=[], verify=None, pipeline=pipeline, cached=False,
                trajectory=None,
            )

        try:
            payload = trajectory_payload(times, points, report)
        except RuntimeError as exc:
            return _blocked(
                "构造轨迹载荷失败: %r" % (exc,),
                _py_path_hint("rm_executor"),
                evidence=[repr(exc)],
                times=[], points=[], verify=None, pipeline=pipeline, cached=False,
                trajectory=None,
            )
        result = _ok(
            "规划成功：%d 个采样点（管线=%s）" % (len(times), pipeline),
            evidence=["verify: 四道校验全部通过",
                      ("保路径降速重定时 ×%.3f" % scale) if scale else "无需降速重定时"],
            times=times, points=points, verify=dict(report),
            pipeline=pipeline, cached=False, trajectory=payload,
        )
        if self._cache_enabled and key is not None:
            self._cache[key] = result
        return result

    # -- 辅助 --------------------------------------------------------------
    @staticmethod
    def _as_vector(value: Any, count: int) -> Optional[List[float]]:
        if isinstance(value, (str, bytes)) or not isinstance(value, (list, tuple)):
            return None
        if len(value) != count:
            return None
        out: List[float] = []
        for item in value:
            if isinstance(item, bool) or not isinstance(item, (int, float)):
                return None
            val = float(item)
            if not math.isfinite(val):
                return None
            out.append(val)
        return out

    @staticmethod
    def _make_pose(values: Sequence[float]) -> Any:
        """构造厂商位姿对象（``Pose.from_xyz_rpy``）——惰性 import 厂商内核。"""
        from tesseract_robotics.planning import Pose

        return Pose.from_xyz_rpy(*[float(v) for v in values])


# --------------------------------------------------------------------------
# 运动客户端
# --------------------------------------------------------------------------

class FakeMotionClient:
    """离线运动客户端：走 ``rm_executor.cli.execute(mode="fake")``。

    **同一 Executor、同一报告 schema**（不另造一套报告结构）。

    :param motion_gate: 夹爪互锁闸门（与夹爪客户端**共享同一实例**；
        ``None`` → 不启用互锁，执行器会在 evidence 里说明）。
    :param ws: 工作区根（执行配置唯一来源 = ``workcell/rm_workcell.yaml``）。
    :param arm: 选哪条臂。
    """

    def __init__(self, motion_gate: Any = None, ws: Optional[str] = None,
                 arm: str = "left") -> None:
        self._motion_gate = motion_gate
        self._ws = ws
        self._arm = arm

    def run_trajectory(self, traj_path: str) -> Dict[str, Any]:
        """执行一条轨迹，返回 ``rm_executor`` 统一报告。"""
        mods, err = _import_executor()
        if mods is None:
            return _blocked(
                "执行层 rm_executor 不可用: %s" % err,
                _py_path_hint("rm_executor"),
                evidence=[err],
            )
        try:
            return mods["cli"].execute(
                mode="fake",
                trajectory_path=traj_path,
                ws=self._ws,
                arm=self._arm,
                motion_gate=self._motion_gate,
            )
        except Exception as exc:  # noqa: BLE001 - 统一降级，不抛裸异常
            return _blocked(
                "执行异常: %s: %r" % (type(exc).__name__, exc),
                "查看终端栈信息后重跑同一条命令",
                evidence=[repr(exc)],
            )


class LiveMotionClient:
    """真机运动客户端：包装 ``rm_executor.session.LiveSession``。

    T5 交付 ``rm_executor.session``；在此之前本类**按协议调用**并在会话层
    不可用时返回 ``blocked``（可粘贴下一步），**不伪造**执行结论。

    :param session: 已建立的会话（缺省 → 惰性取 ``rm_executor.session``）。
    :param ws: 工作区根。
    :param arm: 选哪条臂。
    """

    def __init__(self, session: Any = None, ws: Optional[str] = None,
                 arm: str = "left") -> None:
        self._session = session
        self._ws = ws
        self._arm = arm

    def _load_session(self):
        if self._session is not None:
            return self._session, ""
        try:
            from rm_executor import session as _session_mod
        except Exception as exc:  # noqa: BLE001 - T5 未落地 → blocked
            return None, "%s: %r" % (type(exc).__name__, exc)
        factory = getattr(_session_mod, "LiveSession", None)
        if factory is None:
            return None, "rm_executor.session 里没有 LiveSession"
        try:
            return factory(ws=self._ws, arm=self._arm), ""
        except Exception as exc:  # noqa: BLE001
            return None, "%s: %r" % (type(exc).__name__, exc)

    def run_trajectory(self, traj_path: str) -> Dict[str, Any]:
        """执行一条轨迹，返回 ``rm_executor`` 统一报告。"""
        session, err = self._load_session()
        if session is None:
            return _blocked(
                "真机会话层不可用（rm_executor.session.LiveSession 未就绪）: %s" % err,
                "确认 dualarm_ws/src/rm_executor 在 PYTHONPATH 上"
                "（S06/T5 落地后接线）后重跑",
                evidence=[err],
            )
        runner = getattr(session, "run_trajectory", None) or getattr(session, "run", None)
        if runner is None:
            return _blocked(
                "真机会话对象没有 run_trajectory/run 方法",
                "检查 rm_executor.session.LiveSession 的接口后重跑",
                evidence=[repr(type(session))],
            )
        try:
            return runner(traj_path)
        except Exception as exc:  # noqa: BLE001
            return _blocked(
                "真机执行异常: %s: %r" % (type(exc).__name__, exc),
                "查看终端栈信息与控制器状态后重跑",
                evidence=[repr(exc)],
            )


# --------------------------------------------------------------------------
# 夹爪客户端
# --------------------------------------------------------------------------

class _BaseGripperClient:
    """夹爪客户端公共部分：动作走 ``gripper_action`` 护栏，判定走 ``judge``。

    * **动作**必须经 ``rm_hw_bridge.gripper_state.gripper_action(reason, gate)``
      上下文：动作期间闸门 = ``GRIPPER_BUSY``，退出后 = ``GRIPPER_UNJUDGED``；
    * **判定**必须经 ``rm_hw_bridge.gripper_state.judge(state, thresholds)``
      （阈值由构造参数注入，**本层不补默认值**）。
    """

    def __init__(self, thresholds: Optional[Dict[str, Any]] = None,
                 gate: Any = None) -> None:
        self._thresholds = dict(thresholds or {})
        self._gate = gate

    # -- 公共 --------------------------------------------------------------
    @property
    def thresholds(self) -> Dict[str, Any]:
        """注入的判定阈值（**只读**）。"""
        return dict(self._thresholds)

    @property
    def gate(self) -> Any:
        """注入的运动闸门（可能为 ``None``）。"""
        return self._gate

    def _state_module(self):
        mods, err = _import_hw()
        if mods is None:
            return None, err
        return mods["state"], ""

    def _judge_result(self, state: Any) -> Dict[str, Any]:
        """调用硬件层判定并回写闸门状态（**不比较严重度、不排序**）。"""
        state_mod, err = self._state_module()
        if state_mod is None:
            return _blocked(
                "硬件适配层 rm_hw_bridge 不可用: %s" % err,
                _py_path_hint("rm_hw_bridge"),
                evidence=[err],
            )
        judgment = state_mod.judge(state, self._thresholds)
        result = judgment.to_dict()
        result["mode"] = "fake" if self._is_fake() else "live"
        result["note"] = GRIPPER_PROXY_NOTE
        # 判定「有结论」→ 解除运动互锁；仍需人工/参数的判定 → 保持保守阻塞。
        if result.get("result") in ("grasp_ok", "grasp_empty") and self._gate is not None:
            self._gate.finish_action()
        return result

    def _is_fake(self) -> bool:
        return False


class FakeGripperClient(_BaseGripperClient):
    """离线假夹爪客户端（``rm_hw_bridge.gripper.FakeGripper`` + 适配器）。

    假夹爪按动作更新寄存器以表达物理语义：

    * ``close`` → 力度 = 2×阈值、位置 = 夹持窗口中心 → 判定 ``grasp_ok``；
    * ``open``  → 力度 = 0 → 判定 ``grasp_empty``。

    阈值缺失（未标定）时**不**伪造力度/位置 → 判定自然落到 ``needs_param``。

    :param thresholds: 判定阈值（**注入**；缺项保持 ``None``）。
    :param gate: 运动闸门（与运动客户端共享）。
    :param path: 协议路径（``modbus`` / ``gripper_api``）。
    """

    def __init__(self, thresholds: Optional[Dict[str, Any]] = None, gate: Any = None,
                 path: str = "modbus", port: Any = None) -> None:
        super().__init__(thresholds=thresholds, gate=gate)
        self._path = path
        self._port = port
        self._adapter = None

    def _is_fake(self) -> bool:
        return True

    def _hw(self):
        mods, err = _import_hw()
        if mods is None:
            return None, err
        return mods, ""

    def _build_adapter(self):
        mods, err = self._hw()
        if mods is None:
            return None, err
        gripper = mods["gripper"]
        if self._adapter is not None:
            return self._adapter, ""
        port = self._port
        if port is None:
            port = gripper.FakeGripper(base=1, mode=2)
        self._adapter = gripper.GripperAdapter(
            port,
            path=self._path,
            base=1,
            position_mm_per_unit=self._thresholds.get("position_mm_per_unit"),
            force_scale=self._thresholds.get("force_scale"),
        )
        return self._adapter, ""

    # -- 协议 --------------------------------------------------------------
    def act(self, action: str, speed: Optional[int] = None,
            force: Optional[int] = None) -> Dict[str, Any]:
        """执行一次夹爪动作（``close`` / ``open``）。"""
        if action not in ("close", "open"):
            return _blocked(
                "未知夹爪动作: %r（只允许 close / open）" % (action,),
                "改成 close 或 open 后重跑",
                evidence=[],
            )
        adapter, err = self._build_adapter()
        if adapter is None:
            return _blocked(
                "硬件适配层 rm_hw_bridge 不可用: %s" % err,
                _py_path_hint("rm_hw_bridge"),
                evidence=[err],
            )
        state_mod, serr = self._state_module()
        if state_mod is None:
            return _blocked(
                "硬件适配层 rm_hw_bridge 不可用: %s" % serr,
                _py_path_hint("rm_hw_bridge"),
                evidence=[serr],
            )

        speed_value = 100 if speed is None else int(speed)
        force_value = 100 if force is None else int(force)
        reason = "夹爪动作 %s（离线代理）" % action
        with state_mod.gripper_action(reason, self._gate):
            if action == "close":
                res = adapter.gripper_pick(speed_value, force_value, True, 0)
            else:
                res = adapter.gripper_release(speed_value, True, 0)
            if res.get("status") != "ok":
                return _blocked(
                    "夹爪动作失败: %s" % (res.get("reason") or "未知原因"),
                    res.get("next_action") or "检查夹爪接线与配置后重跑",
                    evidence=list(res.get("evidence") or []),
                )
            self._apply_physical_semantics(action)
        return _ok(
            "夹爪动作已下发: %s（离线代理）" % action,
            evidence=["mode=fake（离线假夹爪，不代表真机结论）",
                      GRIPPER_PROXY_NOTE],
            action=action, speed=speed_value, force=force_value, mode="fake",
            note=GRIPPER_PROXY_NOTE,
        )

    def _apply_physical_semantics(self, action: str) -> None:
        """把动作的物理语义写进假夹爪寄存器。"""
        mods, _err = self._hw()
        if mods is None:
            return
        gripper = mods["gripper"]
        port = getattr(self._adapter, "port", None)
        if port is None:
            return
        force_threshold = self._thresholds.get("force_threshold")
        window = self._thresholds.get("grasp_position_window")
        per_unit = self._thresholds.get("position_mm_per_unit")
        if action == "close":
            if force_threshold is not None:
                raw = min(1000, int(round(float(force_threshold) * 2)))
                port.set_register(gripper.REG_ACTUAL_FORCE, raw)
            if window is not None and per_unit:
                center = (float(window[0]) + float(window[1])) / 2.0
                port.set_register(gripper.REG_ACTUAL_POS,
                                  int(round(center / float(per_unit))))
        else:
            port.set_register(gripper.REG_ACTUAL_FORCE, 0)
        port.set_register(gripper.REG_STATE, 2)

    def judge(self) -> Dict[str, Any]:
        """读一次夹爪状态并判定（真值 = ``gripper_state.judge``）。"""
        adapter, err = self._build_adapter()
        if adapter is None:
            return _blocked(
                "硬件适配层 rm_hw_bridge 不可用: %s" % err,
                _py_path_hint("rm_hw_bridge"),
                evidence=[err],
            )
        try:
            res = adapter.read_state()
        except Exception as exc:  # noqa: BLE001
            return _blocked(
                "读取夹爪状态异常: %s: %r" % (type(exc).__name__, exc),
                "检查夹爪接线后重跑",
                evidence=[repr(exc)],
            )
        if res.get("status") != "ok":
            return _blocked(
                "读取夹爪状态失败: %s" % (res.get("reason") or "未知原因"),
                res.get("next_action") or "检查夹爪接线与配置后重跑",
                evidence=list(res.get("evidence") or []),
            )
        return self._judge_result(res.get("state"))


class LiveGripperClient(_BaseGripperClient):
    """真机夹爪客户端：包装 ``rm_hw_bridge.gripper.GripperAdapter``。

    T5 会话层给出夹爪端口；在此之前可注入已建立的 ``adapter``。

    :param adapter: 已建立的适配器（缺省 → 惰性构造，端口缺失即 blocked）。
    :param thresholds: 判定阈值（**注入**）。
    :param gate: 运动闸门。
    """

    def __init__(self, adapter: Any = None, thresholds: Optional[Dict[str, Any]] = None,
                 gate: Any = None, port: Any = None, path: str = "modbus") -> None:
        super().__init__(thresholds=thresholds, gate=gate)
        self._adapter = adapter
        self._port = port
        self._path = path

    def _build_adapter(self):
        if self._adapter is not None:
            return self._adapter, ""
        mods, err = _import_hw()
        if mods is None:
            return None, err
        if self._port is None:
            return None, "真机夹爪端口未就绪（S06/T5 会话层提供）"
        self._adapter = mods["gripper"].GripperAdapter(
            self._port,
            path=self._path,
            base=1,
            position_mm_per_unit=self._thresholds.get("position_mm_per_unit"),
            force_scale=self._thresholds.get("force_scale"),
        )
        return self._adapter, ""

    def act(self, action: str, speed: Optional[int] = None,
            force: Optional[int] = None) -> Dict[str, Any]:
        """执行一次夹爪动作（``close`` / ``open``）。"""
        if action not in ("close", "open"):
            return _blocked(
                "未知夹爪动作: %r（只允许 close / open）" % (action,),
                "改成 close 或 open 后重跑",
                evidence=[],
            )
        adapter, err = self._build_adapter()
        if adapter is None:
            return _blocked(
                "夹爪适配器不可用: %s" % err,
                "在真机窗口建立夹爪会话（T5 会话层）后重跑",
                evidence=[err],
            )
        state_mod, serr = self._state_module()
        if state_mod is None:
            return _blocked(
                "硬件适配层 rm_hw_bridge 不可用: %s" % serr,
                _py_path_hint("rm_hw_bridge"),
                evidence=[serr],
            )
        speed_value = 100 if speed is None else int(speed)
        force_value = 100 if force is None else int(force)
        with state_mod.gripper_action("夹爪动作 %s" % action, self._gate):
            if action == "close":
                res = adapter.gripper_pick(speed_value, force_value, True, 0)
            else:
                res = adapter.gripper_release(speed_value, True, 0)
        if res.get("status") != "ok":
            return _blocked(
                "夹爪动作失败: %s" % (res.get("reason") or "未知原因"),
                res.get("next_action") or "检查夹爪接线与配置后重跑",
                evidence=list(res.get("evidence") or []),
            )
        return _ok(
            "夹爪动作已下发: %s" % action,
            evidence=["mode=live"],
            action=action, speed=speed_value, force=force_value, mode="live",
        )

    def judge(self) -> Dict[str, Any]:
        """读一次夹爪状态并判定（真值 = ``gripper_state.judge``）。"""
        adapter, err = self._build_adapter()
        if adapter is None:
            return _blocked(
                "夹爪适配器不可用: %s" % err,
                "在真机窗口建立夹爪会话（T5 会话层）后重跑",
                evidence=[err],
            )
        try:
            res = adapter.read_state()
        except Exception as exc:  # noqa: BLE001
            return _blocked(
                "读取夹爪状态异常: %s: %r" % (type(exc).__name__, exc),
                "检查夹爪接线后重跑",
                evidence=[repr(exc)],
            )
        if res.get("status") != "ok":
            return _blocked(
                "读取夹爪状态失败: %s" % (res.get("reason") or "未知原因"),
                res.get("next_action") or "检查夹爪接线与配置后重跑",
                evidence=list(res.get("evidence") or []),
            )
        return self._judge_result(res.get("state"))
