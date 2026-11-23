"""工艺状态机 runner：段执行 → 夹爪判定 → 恢复阶梯 → 失败分级 → 循环控制。

本模块把「一条轨迹」的执行能力编排成「一条工艺循环」（取放工艺），并且
**失败时给出可粘贴的下一步**（R007），而不是一个布尔失败。

四条硬边界（由 ``tests/test_process_runner.py`` 守卫）
------------------------------------------------------

1. **零严重度定义**：本模块**不**定义任何严重度等级集合，**不**写任何
   严重度排序字典。排序**唯一**来自 ``rm_executor.events.severity_rank``，
   「最严重者」**唯一**来自 ``rm_executor.events.worst_severity``。
   （守护断言：源码里不出现 ``SEVERITY_RANK = {`` 之类的本地字典。）
2. **零判定定义**：夹爪判定的判据**唯一**在
   ``rm_hw_bridge.gripper_state.judge``；本模块只**调用**客户端层的判定
   结果并与段声明的 ``expect`` 比对，不重写任何判据。
3. **恢复阶梯表驱动**：可重试 / 升级人工的裁决**唯一**来自
   ``rm_hw_bridge.gripper_state.RECOVERY_LADDER``（纯数据表），本模块
   **不复制**阶梯逻辑；重试上限由 ``thresholds`` **注入**
   （唯一来源 = ``workcell/rm_workcell.yaml::gripper.max_regrasp`` 经
   ``rmctl.gripper_proxy.effective_thresholds`` 传入），本模块**不写死**任何
   上限数值（未注入 → 不自动重试，直接升级人工）。
4. **零阈值副本**：判定阈值由构造参数注入；本模块**不读**工位配置、
   **不补**默认值。

段语义
------

``motion``
    运动段。执行前**必查**夹爪互锁闸门（``gate.check()``），非 IDLE 即拒绝
    本段（记 ``gripper_interlock_blocked`` 证据，**绝不绕过**）；随后
    ``Planner.plan_segment(self._current_deg, goal_deg=..., goal_pose=...)``
    → 写轨迹 JSON（临时目录，字段齐全符合 ``rm_executor.contract``）→
    ``MotionClient.run_trajectory(path)``。报告 status≠ok → 该段失败。
``gripper``
    夹爪段。``GripperClient.act(action)`` → ``GripperClient.judge()`` →
    与段声明的 ``expect`` 比对。
``judge``
    判定段（只读判定，不动作）→ 与段声明的 ``expect`` 比对。

当前关节状态跟踪
----------------

runner 自持 ``self._current_deg``（初值 = 规格 ``home_joints_deg``）。一次
``motion`` 段**成功**执行后，``self._current_deg`` = **该段轨迹最后一个点的
关节角**（弧度→度，**runner 是度↔弧度的唯一换算点**）：点到点时等价于段
目标，笛卡尔时为 IK 解；**不额外读反馈、不新增 IO**。段失败时**不更新**
（保持失败前的状态，便于诊断）。

闸门复位规则（红线）
--------------------

判定**完成且结果确定**（``grasp_ok`` / ``grasp_empty`` / ``stall`` /
``overtemp`` / ``overcurrent`` / ``offline``）→ 调 ``gate.finish_action()``
把闸门恢复 IDLE，之后才允许运动段；判定为 ``needs_param`` / ``unknown``
→ **不复位**（闸门保持 UNJUDGED → 运动段必被互锁拒绝）并停止循环。

失败分级的唯一出口
------------------

循环级 ``reason`` / ``next_action`` = 所有失败里**最严重者**的
``next_action``（排序直接用 ``rm_executor.events.severity_rank``，
**不新写排序函数**）。

循环控制
--------

``iterations`` 来自规格（可被 ``max_iterations`` 覆盖，**上限 50**，超出 →
参数非法 blocked）；任一不可恢复失败（NEEDS_PARAM / NEEDS_HUMAN / FATAL）→
**停止循环**并给下一步；RECOVERABLE 且阶梯允许重试 → 在本圈内重试；每圈
结束必须回到 home 段（未回到 home → 本圈判失败并停循环）。

trace 出口（duck-type）
-----------------------

runner 内部维护事件列表，并调用可选 sink 的 ``.record(event)``
（T4 的 ``TraceWriter`` 只需实现 ``record``）；无 sink 时事件只留在报告的
``events`` 里（不落盘）。

零 SDK、零 IO：本模块只经三个协议与上游真值交互，**不** import ``ctypes`` /
厂商 SDK / ``rm_hw_bridge.generated``。
"""
from __future__ import annotations

import json
import math
import os
import shutil
import tempfile
import time
from typing import Any, Dict, List, Optional, Sequence, Tuple

from rmctl.process import clients as clients_mod

__all__ = [
    "GATE_RESETTABLE_RESULTS",
    "MAX_ITERATIONS",
    "ProcessRunner",
]


#: 循环次数上限（工艺约束：连续 ≤50 圈后强制人工检查）。
#: 与 ``rmctl.process.spec.MAX_ITERATIONS`` 同一口径（规格层不据此 blocked，
#: runner 在**运行时**据此 blocked —— 参数非法绝不静默截断）。
MAX_ITERATIONS = 50

#: **判定结果确定**的集合（可复位运动闸门）。
#:
#: 口径来自切片契约的「闸门复位规则」：``grasp_ok`` / ``grasp_empty`` /
#: ``stall`` / ``overtemp`` / ``overcurrent`` / ``offline`` 都是**已判定**
#: （结论明确），可恢复运动互锁；``needs_param`` / ``unknown`` **不是**
#: （结论未定），闸门保持 UNJUDGED，运动段必被互锁拒绝。
#:
#: 这**不是**判定定义（判据仍在 ``rm_hw_bridge.gripper_state.judge``），
#: 只是「结论是否确定」的枚举。
GATE_RESETTABLE_RESULTS: Tuple[str, ...] = (
    "grasp_ok",
    "grasp_empty",
    "stall",
    "overtemp",
    "overcurrent",
    "offline",
)

#: 结论**未确定**的判定结果（不复位闸门 → 停循环）
INDETERMINATE_RESULTS: Tuple[str, ...] = ("needs_param", "unknown")


# --------------------------------------------------------------------------
# 小工具
# --------------------------------------------------------------------------

def _render(template: str, ctx: Dict[str, Any]) -> str:
    """渲染阶梯模板（``{attempt}`` / ``{max_regrasp}`` …）。

    只做**字符串占位替换**（不复制任何判定 / 阶梯逻辑）；模板缺占位符时
    原样返回（**不抛异常**）。
    """
    try:
        return str(template).format(**ctx)
    except (KeyError, IndexError, ValueError):
        return str(template)


def _deg_list(values: Optional[Sequence[float]]) -> List[float]:
    """序列 → 浮点列表（非法项跳过）。"""
    out: List[float] = []
    for item in values or []:
        try:
            out.append(float(item))
        except (TypeError, ValueError):
            continue
    return out


class ProcessRunner:
    """工艺状态机：把一份工艺规格跑成「一条工艺循环」。

    :param spec: 工艺规格（``rmctl.process.spec.load_task_spec`` 的
        ``spec`` 字段形态）。
    :param planner: 规划客户端（``rmctl.process.clients.Planner``）。
    :param motion: 运动客户端（``MotionClient``）。
    :param gripper: 夹爪客户端（``GripperClient``）。
    :param trace_sink: 可选 trace 出口（鸭子类型：只需 ``.record(event)``）。
    :param gate: 可选夹爪运动闸门（鸭子类型：``check()`` / ``finish_action()``）。
    :param thresholds: 注入的阈值（判定阈值 + 恢复上限 ``max_regrasp`` /
        ``max_stall_retry`` / ``max_offline_retry``）；**唯一来源** =
        ``rmctl.gripper_proxy.effective_thresholds`` + 工位配置。
    :param max_iterations: 覆盖规格里的 ``iterations``（≤50）。
    """

    def __init__(self, spec: Dict[str, Any], planner: Any, motion: Any,
                 gripper: Any, trace_sink: Any = None, gate: Any = None,
                 thresholds: Optional[Dict[str, Any]] = None,
                 max_iterations: Optional[int] = None) -> None:
        self._spec = dict(spec or {})
        self._planner = planner
        self._motion = motion
        self._gripper = gripper
        self._sink = trace_sink
        self._gate = gate
        self._thresholds = dict(thresholds or {})
        self._max_iterations = max_iterations

        # 当前关节状态（度）—— runner 是度↔弧度的唯一换算点
        self._current_deg: List[float] = _deg_list(
            self._spec.get("home_joints_deg"))

        # 运行期状态
        self._events: List[Dict[str, Any]] = []
        self._judgments: List[Dict[str, Any]] = []
        self._failures: List[Dict[str, Any]] = []
        self._evidence: List[str] = []
        self._segments_executed = 0
        self._iterations_completed = 0
        self._mode = "fake"
        self._attempts: Dict[str, int] = {}
        self._tmpdir: Optional[str] = None

    # ------------------------------------------------------------------
    # 公共入口
    # ------------------------------------------------------------------
    def run(self) -> Dict[str, Any]:
        """跑完一条工艺循环，返回**循环级报告**。

        返回键：``status`` / ``reason`` / ``next_action`` / ``severity`` /
        ``mode`` / ``iterations_planned`` / ``iterations_completed`` /
        ``segments_executed`` / ``judgments`` / ``failures`` / ``evidence`` /
        ``trace_path`` / ``events`` / ``exit_code``。
        """
        mods, err = clients_mod.executor_modules()
        if mods is None:
            return self._blocked_report(
                "执行层 rm_executor 不可用（拿不到严重度排序真值）: %s" % err,
                "export PYTHONPATH=$PWD/dualarm_ws/src/rm_executor:$PYTHONPATH  "
                "# 让 rmctl 看到 rm_executor（严重度排序唯一真值）",
                ["rm_executor 不可用: %s" % err],
            )
        self._events_mod = mods["events"]
        self._contract = mods["contract"]

        planned = self._planned_iterations()
        if planned is None:
            return self._blocked_report(
                "工艺循环次数非法（上限 %d）" % MAX_ITERATIONS,
                "把 iterations / --count 改成 1..%d 后重跑"
                "（超过 %d 圈必须人工检查，runner 不静默截断）"
                % (MAX_ITERATIONS, MAX_ITERATIONS),
                ["iterations=%r, max_iterations=%r"
                 % (self._spec.get("iterations"), self._max_iterations)],
                severity="NEEDS_PARAM",
            )

        self._tmpdir = tempfile.mkdtemp(prefix="rmctl_process_")
        try:
            for iteration in range(1, planned + 1):
                if not self._run_iteration(iteration):
                    break
        finally:
            self._cleanup_tmpdir()

        return self._finalize(planned)

    # ------------------------------------------------------------------
    # 循环控制
    # ------------------------------------------------------------------
    def _planned_iterations(self) -> Optional[int]:
        """本次循环的圈数（规格 ``iterations``，可被 ``max_iterations`` 覆盖）。

        非法（<1 / >50 / 非整数）→ ``None``（调用方转 blocked）。
        """
        value = self._max_iterations
        if value is None:
            value = self._spec.get("iterations")
        if isinstance(value, bool) or not isinstance(value, int):
            return None
        if value < 1 or value > MAX_ITERATIONS:
            return None
        return int(value)

    def _run_iteration(self, iteration: int) -> bool:
        """跑一圈；返回 ``True`` = 本圈完成（可继续下一圈）。"""
        segments = list(self._spec.get("segments") or [])
        if not segments:
            self._fail(iteration, {}, "spec_no_segments", "NEEDS_PARAM",
                       "工艺规格里没有段序列",
                       "在 workcell/tasks/*.yaml 里补齐 segments 后重跑")
            return False

        self._attempts = {}
        self._record(iteration, "loop", "", "ok", "INFO",
                     "第 %d 圈开始（段数 %d）" % (iteration, len(segments)))

        home_ok = False
        for index, seg in enumerate(segments):
            seg_type = seg.get("type")
            if seg_type == "motion":
                ok, stop = self._run_motion(iteration, index, seg)
            elif seg_type == "gripper":
                ok, stop = self._run_gripper(iteration, index, seg)
            elif seg_type == "judge":
                ok, stop = self._run_judge(iteration, index, seg)
            else:
                self._fail(iteration, seg, "segment_type_invalid", "NEEDS_HUMAN",
                           "段类型非法: %r（只允许 motion / gripper / judge）"
                           % (seg_type,),
                           "修正规格里的段类型后重跑")
                return False
            if index == len(segments) - 1 and ok:
                home_ok = True
            if stop:
                return False

        if not home_ok:
            self._fail(iteration, segments[-1], "home_not_reached", "NEEDS_HUMAN",
                       "本圈未回到 home 段（最后一段 %r 未成功执行）"
                       % (segments[-1].get("name"),),
                       "确认机械臂已回到 home 位姿后再继续（人工复位后重跑）")
            return False

        self._iterations_completed += 1
        self._record(iteration, "loop", "", "ok", "INFO",
                     "第 %d 圈完成（累计 %d 圈）" % (iteration, self._iterations_completed))
        return True

    # ------------------------------------------------------------------
    # 段：运动
    # ------------------------------------------------------------------
    def _run_motion(self, iteration: int, index: int,
                    seg: Dict[str, Any]) -> Tuple[bool, bool]:
        """执行一个运动段；返回 ``(段是否成功, 是否停止循环)``。"""
        name = str(seg.get("name") or "segment_%d" % index)

        # -- 红线：任何运动段执行前都查一次闸门 --------------------------
        blocked = self._gate_check()
        if blocked is not None:
            self._fail(iteration, seg, "gripper_interlock_blocked", "NEEDS_HUMAN",
                       "夹爪互锁闸门未放行（state=%s）：%s"
                       % (blocked.get("state"), blocked.get("reason")),
                       blocked.get("next_action")
                       or "先完成夹爪状态判定后再下发运动",
                       evidence=["gate.state=%s" % blocked.get("state")])
            return False, True

        goal_deg = seg.get("goal_joints_deg")
        goal_pose = seg.get("goal_pose")
        try:
            plan = self._planner.plan_segment(
                list(self._current_deg), goal_deg=goal_deg, goal_pose=goal_pose)
        except Exception as exc:  # noqa: BLE001 - 规划异常 → 段失败
            self._fail(iteration, seg, "plan_failed", "RECOVERABLE",
                       "规划异常（%s）: %s" % (type(exc).__name__, exc),
                       "检查目标可达性与碰撞场景后重跑",
                       evidence=[repr(exc)])
            return False, False

        if not isinstance(plan, dict) or plan.get("status") != "ok":
            reason = str((plan or {}).get("reason") or "规划未成功")
            next_action = str((plan or {}).get("next_action")
                              or "修正规划输入后重跑")
            self._fail(iteration, seg, "plan_failed", "RECOVERABLE",
                       "段 %s 规划失败: %s" % (name, reason), next_action,
                       evidence=[str(e) for e in (plan or {}).get("evidence") or []])
            return False, False

        payload = self._trajectory_payload(plan)
        if payload is None:
            self._fail(iteration, seg, "trajectory_invalid", "NEEDS_PARAM",
                       "段 %s 的规划结果里没有可用轨迹" % name,
                       "确认规划器返回符合 rm_executor 契约的轨迹后重跑")
            return False, True

        path = self._write_trajectory(iteration, index, name, payload)
        if path is None:
            self._fail(iteration, seg, "trajectory_write_failed", "NEEDS_PARAM",
                       "段 %s 的轨迹文件写入失败" % name,
                       "检查临时目录写权限后重跑")
            return False, True

        # -- 契约自检（字段齐全符合 rm_executor.contract.load_trajectory）----
        if self._contract is not None:
            checked = self._contract.load_trajectory(path, allow_unverified=True)
            if checked.get("status") != "ok":
                self._fail(iteration, seg, "trajectory_contract_failed",
                           "NEEDS_PARAM",
                           "段 %s 的轨迹不符合执行层契约: %s"
                           % (name, checked.get("reason")),
                           str(checked.get("next_action")
                               or "重新规划该段后重跑"),
                           evidence=[str(e) for e in checked.get("evidence") or []])
                return False, True

        try:
            report = self._motion.run_trajectory(path)
        except Exception as exc:  # noqa: BLE001 - 执行异常 → 段失败
            self._fail(iteration, seg, "motion_failed", "RECOVERABLE",
                       "段 %s 执行异常（%s）: %s"
                       % (name, type(exc).__name__, exc),
                       "查看终端栈信息后重跑同一条命令",
                       evidence=[repr(exc)])
            return False, False

        report = report if isinstance(report, dict) else {}
        mode = report.get("mode")
        if isinstance(mode, str) and mode:
            self._mode = mode

        if report.get("status") != "ok":
            severity = self._report_severity(report)
            self._fail(iteration, seg, "motion_failed", severity,
                       "段 %s 执行失败: %s" % (name, report.get("reason")
                                               or "未知原因"),
                       str(report.get("next_action")
                           or "查看执行报告的 evidence 后重跑"),
                       evidence=[str(e) for e in report.get("evidence") or []])
            # RECOVERABLE = 可恢复异常（续跑即可）；其余一律停循环交人工
            return False, severity not in ("RECOVERABLE",)

        # -- 成功：更新当前关节状态（轨迹最后一点，弧度→度）--------------
        self._current_deg = self._last_point_deg(payload)
        self._segments_executed += 1
        self._record(iteration, "execute", name, "ok", "INFO",
                     "段 %s 执行成功（当前关节 %s）"
                     % (name, _fmt_vector(self._current_deg)))
        return True, False

    # ------------------------------------------------------------------
    # 段：夹爪 / 判定
    # ------------------------------------------------------------------
    def _run_gripper(self, iteration: int, index: int,
                     seg: Dict[str, Any]) -> Tuple[bool, bool]:
        """执行一个夹爪段（动作 → 判定 → 与 expect 比对 → 恢复阶梯）。"""
        name = str(seg.get("name") or "segment_%d" % index)
        action = seg.get("action")
        expect = seg.get("expect")
        speed = seg.get("speed")
        force = seg.get("force")

        attempt = 0
        while True:
            try:
                act_res = self._gripper.act(action, speed=speed, force=force)
            except Exception as exc:  # noqa: BLE001
                self._fail(iteration, seg, "gripper_action_failed", "RECOVERABLE",
                           "段 %s 夹爪动作异常（%s）: %s"
                           % (name, type(exc).__name__, exc),
                           "检查夹爪接线后重跑")
                return False, True
            if not isinstance(act_res, dict) or act_res.get("status") != "ok":
                self._fail(iteration, seg, "gripper_action_failed", "RECOVERABLE",
                           "段 %s 夹爪动作失败: %s"
                           % (name, (act_res or {}).get("reason") or "未知原因"),
                           str((act_res or {}).get("next_action")
                               or "检查夹爪接线与配置后重跑"),
                           evidence=[str(e) for e in (act_res or {}).get("evidence") or []])
                return False, True

            self._record(iteration, "gripper", name, "ok", "INFO",
                         "夹爪动作已下发: %s" % action)

            ok, stop = self._judge_and_compare(iteration, seg, name, expect,
                                                attempt)
            if stop:
                return False, True
            if ok:
                self._segments_executed += 1
                return True, False

            # -- 阶梯允许重试 → 重新做该段（重抓前先 open 再 close）------
            result = str(self._last_result)
            step = self._ladder_step(result, attempt)
            if step is None or not step.get("retryable"):
                return False, True
            attempt += 1
            self._attempts[result] = attempt
            self._record(iteration, "recover", name, "ok", "RECOVERABLE",
                         "按恢复阶梯重试段 %s（attempt=%d）" % (name, attempt),
                         next_action=_render(str(step.get("next_action") or ""),
                                            self._ladder_ctx(attempt)))
            if action == "close":
                # 重抓前先 open 再 close（阶梯口径）
                try:
                    self._gripper.act("open", speed=speed, force=force)
                except Exception as exc:  # noqa: BLE001
                    self._fail(iteration, seg, "gripper_action_failed",
                               "RECOVERABLE",
                               "段 %s 松手失败（%s）: %s"
                               % (name, type(exc).__name__, exc),
                               "检查夹爪接线后重跑")
                    return False, True

    def _run_judge(self, iteration: int, index: int,
                   seg: Dict[str, Any]) -> Tuple[bool, bool]:
        """执行一个判定段（只读判定，不动作）。"""
        name = str(seg.get("name") or "segment_%d" % index)
        expect = seg.get("expect")
        attempt = 0
        while True:
            ok, stop = self._judge_and_compare(iteration, seg, name, expect,
                                                attempt)
            if stop:
                return False, True
            if ok:
                self._segments_executed += 1
                return True, False
            result = str(self._last_result)
            step = self._ladder_step(result, attempt)
            if step is None or not step.get("retryable"):
                return False, True
            attempt += 1
            self._attempts[result] = attempt
            self._record(iteration, "recover", name, "ok", "RECOVERABLE",
                         "按恢复阶梯重新判定段 %s（attempt=%d）"
                         % (name, attempt))
            # 只判定段：再判一次即可（不动作）

    # ------------------------------------------------------------------
    # 判定 → 比对 → 阶梯
    # ------------------------------------------------------------------
    def _judge_and_compare(self, iteration: int, seg: Dict[str, Any], name: str,
                            expect: Optional[str], attempt: int
                            ) -> Tuple[bool, bool]:
        """读一次判定并与 ``expect`` 比对。

        返回 ``(是否匹配, 是否停止循环)``；不匹配且阶梯可重试时返回
        ``(False, False)``（由调用方重做该段）。
        """
        try:
            judgment = self._gripper.judge()
        except Exception as exc:  # noqa: BLE001
            self._fail(iteration, seg, "judge_failed", "RECOVERABLE",
                       "段 %s 判定异常（%s）: %s"
                       % (name, type(exc).__name__, exc),
                       "检查夹爪接线后重跑")
            return False, True
        judgment = judgment if isinstance(judgment, dict) else {}
        result = str(judgment.get("result") or "unknown")
        self._last_result = result

        # -- 闸门复位规则（红线）----------------------------------------
        if result in GATE_RESETTABLE_RESULTS:
            self._gate_finish()
        # needs_param / unknown → **不复位**（闸门保持 UNJUDGED）

        self._judgments.append({
            "iteration": iteration,
            "segment": name,
            "expect": expect,
            "result": result,
            "severity": _norm_severity(judgment.get("severity")),
            "reason": str(judgment.get("reason") or ""),
            "next_action": str(judgment.get("next_action") or ""),
            "retryable": bool(judgment.get("retryable")),
            "attempt": int(attempt),
        })
        self._record(iteration, "judge", name, "ok",
                     _norm_severity(judgment.get("severity")),
                     "段 %s 判定=%s（期望 %s）"
                     % (name, result, expect if expect else "未声明"),
                     next_action=str(judgment.get("next_action") or ""),
                     evidence=_evidence_lines(judgment.get("evidence")))

        if result in INDETERMINATE_RESULTS:
            # 结论未定 → 不复位闸门 → 停循环（运动段必被互锁拒绝）
            self._fail(iteration, seg, "gripper_indeterminate",
                       "NEEDS_PARAM",
                       "段 %s 判定结果未定（%s）: %s"
                       % (name, result, judgment.get("reason") or ""),
                       str(judgment.get("next_action")
                           or "补齐夹爪标定 / 确认夹爪状态后重跑"),
                       evidence=["gate 未复位（保持 UNJUDGED）"],
                       judgment=judgment)
            return False, True

        if expect is None:
            return True, False
        if result == expect:
            return True, False

        # -- 不匹配 → 查恢复阶梯（表驱动）-------------------------------
        step = self._ladder_step(result, attempt)
        if step is None:
            self._fail(iteration, seg, "gripper_mismatch", "NEEDS_HUMAN",
                       "段 %s 判定=%s，与期望 %s 不符（恢复阶梯里没有该结果）"
                       % (name, result, expect),
                       "人工确认夹爪与工件状态后重跑",
                       judgment=judgment)
            return False, True

        if step.get("retryable"):
            # 可重试：交还调用方重做该段（**不**在这里记失败）
            self._evidence.append(
                "段 %s 判定=%s（期望 %s）→ 按阶梯重试"
                % (name, result, expect))
            return False, False

        severity = str(step.get("severity") or "NEEDS_HUMAN")
        self._fail(iteration, seg, "gripper_mismatch", severity,
                   "段 %s 判定=%s，与期望 %s 不符（恢复阶梯已到升级人工步）"
                   % (name, result, expect),
                   _render(str(step.get("next_action") or ""),
                           self._ladder_ctx(attempt)),
                   evidence=["attempt=%d" % attempt],
                   judgment=judgment)
        return False, True

    def _ladder_step(self, result: str, attempt: int) -> Optional[Dict[str, Any]]:
        """按 ``RECOVERY_LADDER``（表驱动）取本步。

        * 第 0 步 = 可重试步（``limit`` 给出上限键）；上限取自**注入的**
          ``thresholds``（唯一来源 = 工位配置）；未注入 → 视为 0（不自动重试）；
        * 超过上限 → 最后一步 = 升级人工步。
        """
        ladder = self._recovery_ladder()
        if not ladder:
            return None
        steps = ladder.get(result)
        if not steps:
            return None
        first = steps[0]
        limit_key = first.get("limit")
        if not limit_key:
            # 不可重试步（过温 / 过流 / needs_param …）
            return first
        limit = self._thresholds.get(str(limit_key))
        try:
            limit_value = int(limit)
        except (TypeError, ValueError):
            limit_value = 0
        if limit_value <= 0:
            self._evidence.append(
                "恢复上限 %s 未注入 → 不自动重试（直接升级人工）" % limit_key)
            return steps[-1]
        if int(attempt) < limit_value:
            return first
        return steps[-1]

    def _ladder_ctx(self, attempt: int) -> Dict[str, Any]:
        """阶梯模板的渲染上下文（上限取自注入的 thresholds）。"""
        ctx: Dict[str, Any] = {"attempt": int(attempt)}
        for key in ("max_regrasp", "max_stall_retry", "max_offline_retry"):
            try:
                ctx[key] = int(self._thresholds.get(key))
            except (TypeError, ValueError):
                ctx[key] = 0
        ctx.setdefault("temp_c", "未知")
        ctx.setdefault("missing", "无")
        return ctx

    def _recovery_ladder(self) -> Dict[str, Any]:
        """取恢复阶梯表（``rm_hw_bridge.gripper_state.RECOVERY_LADDER``）。"""
        if self._ladder_cache is None:
            mods, _err = clients_mod.hardware_modules()
            state = mods["state"] if mods else None
            self._ladder_cache = getattr(state, "RECOVERY_LADDER", {}) or {}
        return self._ladder_cache

    # ------------------------------------------------------------------
    # 闸门
    # ------------------------------------------------------------------
    def _gate_check(self) -> Optional[Dict[str, Any]]:
        """闸门检查（``None`` = 放行）。"""
        gate = self._gate
        if gate is None:
            return None
        checker = getattr(gate, "check", None)
        if checker is None:
            return None
        try:
            return checker()
        except Exception as exc:  # noqa: BLE001 - 闸门异常 → 保守拒绝
            return {"state": "unknown", "reason": "闸门检查异常: %s" % exc,
                    "next_action": "检查夹爪互锁闸门后重跑"}

    def _gate_finish(self) -> None:
        """判定完成 → 复位闸门到 IDLE。"""
        gate = self._gate
        if gate is None:
            return
        finisher = getattr(gate, "finish_action", None)
        if finisher is None:
            return
        try:
            finisher()
        except Exception as exc:  # noqa: BLE001
            self._evidence.append("闸门复位失败: %s" % exc)

    # ------------------------------------------------------------------
    # 轨迹
    # ------------------------------------------------------------------
    def _trajectory_payload(self, plan: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """从规划结果取轨迹载荷（缺则用 times/points/verify 组装）。"""
        payload = plan.get("trajectory")
        if isinstance(payload, dict) and payload.get("points"):
            return payload
        times = plan.get("times")
        points = plan.get("points")
        verify = plan.get("verify")
        if not times or not points or not isinstance(verify, dict):
            return None
        try:
            return clients_mod.trajectory_payload(times, points, verify)
        except Exception:  # noqa: BLE001
            return None

    def _write_trajectory(self, iteration: int, index: int, name: str,
                          payload: Dict[str, Any]) -> Optional[str]:
        """把轨迹写到临时目录（返回路径；失败 → ``None``）。"""
        if self._tmpdir is None:
            self._tmpdir = tempfile.mkdtemp(prefix="rmctl_process_")
        path = os.path.join(self._tmpdir, "it%03d_%02d_%s.json"
                            % (iteration, index, name))
        try:
            with open(path, "w", encoding="utf-8") as fh:
                json.dump(payload, fh, ensure_ascii=False, indent=2)
        except (OSError, TypeError, ValueError) as exc:
            self._evidence.append("轨迹写入失败 %s: %s" % (path, exc))
            return None
        self._evidence.append("trajectory: %s" % path)
        return path

    def _cleanup_tmpdir(self) -> None:
        if self._tmpdir and os.path.isdir(self._tmpdir):
            shutil.rmtree(self._tmpdir, ignore_errors=True)
        self._tmpdir = None

    @staticmethod
    def _last_point_deg(payload: Dict[str, Any]) -> List[float]:
        """轨迹最后一点的关节角（弧度→度，**runner 是唯一换算点**）。"""
        points = payload.get("points") or []
        if not points:
            return []
        last = points[-1]
        out: List[float] = []
        for value in last:
            try:
                out.append(math.degrees(float(value)))
            except (TypeError, ValueError):
                continue
        return out

    # ------------------------------------------------------------------
    # 事件 / 失败 / 报告
    # ------------------------------------------------------------------
    def _record(self, iteration: int, phase: str, segment: str, status: str,
                severity: str, reason: str, next_action: str = "",
                evidence: Optional[Sequence[str]] = None,
                report: Optional[Dict[str, Any]] = None) -> None:
        """记录一条 trace 事件（并交给 sink）。"""
        event = {
            "schema_version": 1,
            "ts": time.time(),
            "iteration": int(iteration),
            "phase": phase,
            "segment": segment,
            "status": status,
            "severity": severity,
            "reason": reason,
            "next_action": next_action,
            "evidence": list(evidence or []),
        }
        if report is not None:
            event["report"] = report
        self._events.append(event)
        sink = self._sink
        if sink is None:
            return
        recorder = getattr(sink, "record", None)
        if recorder is None:
            return
        try:
            recorder(event)
        except Exception as exc:  # noqa: BLE001 - sink 异常不打断工艺
            self._evidence.append("trace sink 记录失败: %s" % exc)

    def _fail(self, iteration: int, seg: Dict[str, Any], kind: str,
              severity: str, reason: str, next_action: str,
              evidence: Optional[Sequence[str]] = None,
              judgment: Optional[Dict[str, Any]] = None) -> None:
        """登记一次失败（并记录 trace 事件）。"""
        severity = _norm_severity(severity)
        failure = {
            "iteration": int(iteration),
            "segment": str(seg.get("name") or ""),
            "type": kind,
            "severity": severity,
            "reason": reason,
            "next_action": next_action,
            "evidence": list(evidence or []),
        }
        if judgment is not None:
            failure["judgment"] = judgment.get("result")
        self._failures.append(failure)
        self._evidence.append("failure[%s] %s" % (kind, reason))
        self._record(iteration, _phase_for(kind), failure["segment"], "blocked",
                     severity, reason, next_action=next_action,
                     evidence=list(evidence or []))

    def _report_severity(self, report: Dict[str, Any]) -> str:
        """运动段报告的严重度（**唯一**来自 rm_executor.events）。"""
        events = report.get("events") or []
        parsed = []
        for item in events:
            if not isinstance(item, dict):
                continue
            try:
                parsed.append(self._events_mod.Event.from_dict(item))
            except Exception:  # noqa: BLE001
                continue
        worst = self._events_mod.worst_severity(parsed)
        if worst:
            return str(worst)
        status = str(report.get("status") or "")
        if status == "error":
            return "FATAL"
        return "RECOVERABLE"

    def _finalize(self, planned: int) -> Dict[str, Any]:
        """汇总循环级报告（最严重者 → reason / next_action）。"""
        severity = "INFO"
        worst: Optional[Dict[str, Any]] = None
        for failure in self._failures:
            candidate = str(failure.get("severity") or "INFO")
            if self._events_mod.severity_rank(candidate) > self._events_mod.severity_rank(severity):
                severity = candidate
                worst = failure
        status = self._events_mod.status_for_severity(severity)
        reason = (worst or {}).get("reason") or (
            "工艺循环完成：%d/%d 圈，%d 段"
            % (self._iterations_completed, planned, self._segments_executed))
        next_action = (worst or {}).get("next_action") or ""
        report = {
            "status": status,
            "reason": reason,
            "next_action": next_action,
            "severity": severity,
            "mode": self._mode,
            "iterations_planned": int(planned),
            "iterations_completed": int(self._iterations_completed),
            "segments_executed": int(self._segments_executed),
            "judgments": list(self._judgments),
            "failures": list(self._failures),
            "evidence": list(self._evidence),
            "trace_path": str(getattr(self._sink, "path", "") or ""),
            "events": list(self._events),
            "exit_code": int(self._events_mod.exit_code_for_severity(severity)),
        }
        return report

    def _blocked_report(self, reason: str, next_action: str,
                        evidence: Optional[Sequence[str]] = None,
                        severity: str = "NEEDS_PARAM") -> Dict[str, Any]:
        """参数非法 / 依赖不可用 → blocked 报告（不抛裸异常）。"""
        severity = _norm_severity(severity)
        return {
            "status": "blocked",
            "reason": reason,
            "next_action": next_action,
            "severity": severity,
            "mode": self._mode,
            "iterations_planned": 0,
            "iterations_completed": 0,
            "segments_executed": 0,
            "judgments": [],
            "failures": [],
            "evidence": list(evidence or []),
            "trace_path": str(getattr(self._sink, "path", "") or ""),
            "events": list(self._events),
            "exit_code": 3,
        }

    # 内部状态槽（避免在 __init__ 里塞太多属性）
    _events_mod = None
    _contract = None
    _ladder_cache = None
    _last_result = "unknown"


# --------------------------------------------------------------------------
# 模块级工具
# --------------------------------------------------------------------------

def _norm_severity(value: Any) -> str:
    """把判定层的严重度名归一到执行层的五级口径。

    ``OK`` → ``INFO``；``RECOVERABLE`` / ``NEEDS_PARAM`` / ``NEEDS_HUMAN`` /
    ``FATAL`` 原样；未知 → ``NEEDS_HUMAN``（保守）。
    """
    text = str(value or "").strip().upper()
    if text == "OK":
        return "INFO"
    if text in ("RECOVERABLE", "NEEDS_PARAM", "NEEDS_HUMAN", "FATAL", "INFO"):
        return text
    return "NEEDS_HUMAN"


def _evidence_lines(value: Any) -> List[str]:
    """证据 → 字符串列表（dict → ``key=value`` 行）。"""
    if value is None:
        return []
    if isinstance(value, dict):
        return ["%s=%r" % (k, value[k]) for k in sorted(value)]
    if isinstance(value, (list, tuple)):
        return [str(item) for item in value]
    return [str(value)]


def _fmt_vector(values: Sequence[float]) -> str:
    """关节角向量的短文本。"""
    return "[" + ", ".join("%.3f" % float(v) for v in values) + "]"


def _phase_for(kind: str) -> str:
    """失败类型 → trace phase。"""
    if kind.startswith("gripper_interlock"):
        return "execute"
    if kind.startswith("gripper"):
        return "gripper"
    if kind.startswith("judge"):
        return "judge"
    if kind.startswith("plan"):
        return "plan"
    return "loop"
