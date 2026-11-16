"""任务状态机（S06-T04）：**单一主状态源** + **原子 goal_id** 推进。

设计契约
--------
1. **单一主状态源**：状态机对外只暴露 :attr:`TaskStateMachine.state`
   一个可变状态入口（内部字段 ``_state`` 就是它的存储），**不存在第二份
   状态副本**（没有 ``status`` / ``phase`` 之类的影子字段）。
   :attr:`TaskStateMachine.history` 只是**只读审计轨迹**，不是真相源。
2. **原子 goal_id 推进**：每次 :meth:`TaskStateMachine.advance` **只消费一个
   goal**；重复推进已经推进过的 ``goal_id`` ⇒ 显式报错并**点名该 goal_id**
   （幂等重入不得静默成功）。
3. **单一求解器入口**：求解只经 :func:`rm75_drake.tasks.solver.solve_task`
   （可注入替身做离线单测）。状态机按 goal 逐个调用它，并把上一原语的
   末端关节角作为下一次调用的 ``initial_q`` 传下去，保证链式原语的位形连续。
4. **失败语义与既有词表一致**：``stage`` ∈
   ``{schema, ik, path, collision, retime, gripper, verify, state}``
   （``state`` 是状态机自身的失败阶段），``reason`` 取稳定小写词表。
5. **零 pydrake 依赖**：本模块 import 时不拉起 pydrake，因此状态机逻辑
   （单一状态源 / 原子推进 / 失败与中止语义）可在无 pydrake 环境离线单测。

术语
----
* **goal** —— 原子目标，由任务文件里每个原语的 ``goal_id`` 标识
  （唯一性由 :mod:`rm75_drake.tasks.goals` 保证）。
* **消费（consume）** —— 一个 goal 被成功推进一次；同一 goal 至多消费一次。
"""

from __future__ import annotations

import dataclasses
from dataclasses import dataclass
from enum import Enum
from typing import Any, Callable, Dict, List, Optional, Tuple

from .goals import TaskError
from .schema import TaskSpec
from .solver import (
    REASON_COMPLETED,
    REASONS,
    STAGE_DONE,
    STAGES,
    PrimitiveResult,
    SolverOptions,
    TaskResult,
    solve_task,
)

__all__ = [
    "TaskState",
    "TERMINAL_STATES",
    "TASK_STAGES",
    "TASK_REASONS",
    "TaskFailure",
    "StateTransition",
    "TaskStateMachine",
]


# ── 状态与失败词表 ──────────────────────────────────────────────────────
class TaskState(str, Enum):
    """任务状态机的状态枚举（取值即审计轨迹里记录的小写词）。"""

    PENDING = "pending"
    RUNNING = "running"
    WAITING = "waiting"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    ABORTED = "aborted"


#: 终态集合（进入后不再推进）。
TERMINAL_STATES = frozenset(
    {TaskState.SUCCEEDED, TaskState.FAILED, TaskState.ABORTED}
)

#: 状态机阶段的失败阶段（``state``）：状态机自身的失败（重复推进 / 顺序错 /
#: 求解器异常等）。
STAGE_STATE = "state"

#: 完整失败阶段词表 = 求解器词表 + ``state``。
TASK_STAGES: Tuple[str, ...] = tuple(STAGES) + (STAGE_STATE,)

#: 状态机自身的失败原因（稳定小写词表）。
STATE_REASONS: Tuple[str, ...] = (
    "duplicate_goal",      # 重复推进已推进过的 goal_id
    "unknown_goal",        # goal_id 不属于本任务
    "goal_out_of_order",   # goal_id 不是下一个待推进的目标
    "no_pending_goal",     # 没有待推进的目标
    "terminal_state",      # 终态下继续推进
    "solver_error",        # 求解器抛异常 / 返回结果不自洽
    "aborted",             # 显式中止
    "wait",                # 等待外部事件
    "resumed",             # 外部事件到达后恢复
    "in_progress",         # 正在推进（审计轨迹用）
)

#: 完整失败原因词表 = 求解器词表 + 状态机词表。
TASK_REASONS: Tuple[str, ...] = tuple(REASONS) + STATE_REASONS


# ── 审计数据结构 ────────────────────────────────────────────────────────
@dataclass(frozen=True)
class TaskFailure:
    """一次失败的稳定描述（``stage`` / ``reason`` / ``message``）。"""

    stage: str
    reason: str
    message: str = ""
    goal_id: Optional[str] = None

    def as_dict(self) -> Dict[str, Any]:
        return {
            "stage": self.stage,
            "reason": self.reason,
            "message": self.message,
            "goal_id": self.goal_id,
        }


@dataclass(frozen=True)
class StateTransition:
    """审计轨迹上的一次状态转移（**只读**，不是真相源）。"""

    index: int
    state: TaskState
    goal_id: Optional[str] = None
    stage: str = STAGE_STATE
    reason: str = ""
    message: str = ""

    def as_dict(self) -> Dict[str, Any]:
        return {
            "index": self.index,
            "state": self.state.value,
            "goal_id": self.goal_id,
            "stage": self.stage,
            "reason": self.reason,
            "message": self.message,
        }


#: 求解器契约：``solver(task, *, scene, transport, options) -> TaskResult``。
Solver = Callable[..., TaskResult]


# ── 状态机 ──────────────────────────────────────────────────────────────
class TaskStateMachine:
    """任务状态机：**单一主状态源** + **原子 goal_id** 推进。

    参数
    ----
    task:
        任务（:class:`rm75_drake.tasks.schema.TaskSpec`）。
    solver:
        求解器入口，默认 :func:`rm75_drake.tasks.solver.solve_task`
        （**唯一**的任务级求解入口）。测试可注入替身（零 Drake 依赖）。
    scene:
        场景（碰撞复检用）；``None`` ⇒ 运动类原语会以
        ``stage="schema" / reason="missing_scene"`` 失败。
    transport:
        传输层（夹爪下发 / 状态回读）；``None`` ⇒ 夹爪命令不下发。
    options:
        求解参数（:class:`rm75_drake.tasks.solver.SolverOptions`）。

    纪律
    ----
    * 状态只有 :attr:`state` 一个对外入口；:attr:`history` 是只读审计轨迹；
    * :meth:`advance` 每次只消费一个 goal；重复推进 ⇒ 报错并点名 goal_id；
    * :meth:`run` 推进到终态；:meth:`abort` 显式中止。
    """

    def __init__(
        self,
        task: TaskSpec,
        *,
        solver: Optional[Solver] = None,
        scene: Any = None,
        transport: Any = None,
        options: Optional[SolverOptions] = None,
    ) -> None:
        if not isinstance(task, TaskSpec):
            raise TaskError(f"task 必须是 TaskSpec 实例，收到 {type(task).__name__}")
        goal_ids = task.goal_ids()
        seen: Dict[str, int] = {}
        for index, goal_id in enumerate(goal_ids):
            if goal_id in seen:
                raise TaskError(
                    f"重复的 goal_id：{goal_id!r}（primitives[{seen[goal_id]}] 与 "
                    f"primitives[{index}]）；任务内 goal_id 必须唯一"
                )
            seen[goal_id] = index

        self._task = task
        self._solver: Solver = solver if solver is not None else solve_task
        self._scene = scene
        self._transport = transport
        self._options = options if options is not None else SolverOptions()

        # ── 唯一的状态存储（单一主状态源）──────────────────────────
        self._state = TaskState.PENDING

        # ── 进度游标（不是状态）────────────────────────────────────
        self._goal_ids: Tuple[str, ...] = tuple(goal_ids)
        self._attempted: List[str] = []
        self._consumed: List[str] = []
        self._cursor: Tuple[float, ...] = tuple(float(v) for v in self._options.initial_q)
        self._current_goal_id: Optional[str] = None

        # ── 输出与审计（只读视图）──────────────────────────────────
        self._results: List[PrimitiveResult] = []
        self._history: List[StateTransition] = []
        self._failure: Optional[TaskFailure] = None

    # ── 状态（唯一真相源）──────────────────────────────────────────────
    @property
    def state(self) -> TaskState:
        """**唯一**的主状态源（外部不得另存一份状态）。"""
        return self._state

    @property
    def is_terminal(self) -> bool:
        """是否已进入终态（``SUCCEEDED`` / ``FAILED`` / ``ABORTED``）。"""
        return self._state in TERMINAL_STATES

    @property
    def failure(self) -> Optional[TaskFailure]:
        """最近一次失败的稳定描述（未失败时为 ``None``）。"""
        return self._failure

    @property
    def history(self) -> Tuple[StateTransition, ...]:
        """只读审计轨迹（**不是**真相源）。"""
        return tuple(self._history)

    @property
    def results(self) -> Tuple[PrimitiveResult, ...]:
        """逐原语结果（按推进顺序）。"""
        return tuple(self._results)

    @property
    def consumed_goal_ids(self) -> Tuple[str, ...]:
        """已成功消费（推进过）的 goal_id 序列。"""
        return tuple(self._consumed)

    @property
    def attempted_goal_ids(self) -> Tuple[str, ...]:
        """已尝试过的 goal_id 序列（含失败的那一个）。"""
        return tuple(self._attempted)

    @property
    def pending_goal_ids(self) -> Tuple[str, ...]:
        """尚未推进的 goal_id 序列。"""
        done = set(self._attempted)
        return tuple(g for g in self._goal_ids if g not in done)

    @property
    def current_goal_id(self) -> Optional[str]:
        """当前（或最近一次）处理的 goal_id。"""
        return self._current_goal_id

    @property
    def task(self) -> TaskSpec:
        """所属任务（只读引用）。"""
        return self._task

    # ── 推进 ───────────────────────────────────────────────────────────
    def advance(self, goal_id: Optional[str] = None) -> PrimitiveResult:
        """推进**一个** goal，返回该 goal 的求解结果。

        参数
        ----
        goal_id:
            目标 goal_id；``None`` ⇒ 推进**下一个**待推进的目标。

        返回
        ----
        :class:`rm75_drake.tasks.solver.PrimitiveResult`
            成功 ``success=True``；失败时状态机转为
            :attr:`TaskState.FAILED`，且 ``stage`` / ``reason`` 透出。

        异常
        ----
        :class:`rm75_drake.tasks.goals.TaskError`
            终态下推进 / 处于 ``WAITING`` / ``goal_id`` 重复 / 未知 /
            顺序不对 —— 均**显式报错并点名**。
        """
        self._require_runnable()
        target = self._resolve_goal(goal_id)
        self._current_goal_id = target
        self._record(
            TaskState.RUNNING,
            goal_id=target,
            stage=STAGE_STATE,
            reason="in_progress",
            message=f"开始推进 goal_id={target!r}",
        )
        result = self._solve_goal(target)
        self._results.append(result)
        self._attempted.append(target)

        if not result.success:
            failure = TaskFailure(
                stage=result.stage or STAGE_STATE,
                reason=result.reason or "solver_error",
                message=result.message,
                goal_id=target,
            )
            self._failure = failure
            self._record(
                TaskState.FAILED,
                goal_id=target,
                stage=failure.stage,
                reason=failure.reason,
                message=failure.message,
            )
            return result

        self._consumed.append(target)
        if self.pending_goal_ids:
            self._record(
                TaskState.RUNNING,
                goal_id=target,
                stage=STAGE_DONE,
                reason=REASON_COMPLETED,
                message=f"goal_id={target!r} 推进成功",
            )
        else:
            self._record(
                TaskState.SUCCEEDED,
                goal_id=target,
                stage=STAGE_DONE,
                reason=REASON_COMPLETED,
                message="全部目标推进完成",
            )
        return result

    def run(self, max_steps: Optional[int] = None) -> Tuple[PrimitiveResult, ...]:
        """循环推进到终态（``SUCCEEDED`` / ``FAILED`` / ``ABORTED``）。

        参数
        ----
        max_steps:
            最多推进多少个 goal（``None`` ⇒ 不限）；到达上限即返回，
            状态机停在 ``RUNNING``。

        返回
        ----
        tuple：本次推进得到的所有 :class:`PrimitiveResult`。
        """
        produced: List[PrimitiveResult] = []
        steps = 0
        while not self.is_terminal:
            if self._state is TaskState.WAITING:
                # 等待外部事件：不是终态，但也不能自行推进。
                break
            if max_steps is not None and steps >= int(max_steps):
                break
            steps += 1
            produced.append(self.advance())
        return tuple(produced)

    def wait(self, reason: str = "wait") -> None:
        """进入 ``WAITING``（等待外部事件，例如感知回调）。"""
        if self.is_terminal:
            raise TaskError(
                f"状态机已进入终态 {self._state.value}，不能进入 WAITING"
            )
        self._record(
            TaskState.WAITING,
            goal_id=self._current_goal_id,
            stage=STAGE_STATE,
            reason=reason,
            message="等待外部事件",
        )

    def resume(self) -> None:
        """从 ``WAITING`` 恢复为 ``RUNNING``。"""
        if self._state is not TaskState.WAITING:
            raise TaskError(
                f"状态机不在 WAITING（当前 {self._state.value}），不能 resume()"
            )
        self._record(
            TaskState.RUNNING,
            goal_id=self._current_goal_id,
            stage=STAGE_STATE,
            reason="resumed",
            message="外部事件已到达，继续推进",
        )

    def abort(self, reason: str = "aborted") -> None:
        """显式中止（进入 :attr:`TaskState.ABORTED`）。"""
        if self.is_terminal:
            raise TaskError(
                f"状态机已进入终态 {self._state.value}，不能再次 abort()"
            )
        self._failure = TaskFailure(
            stage=STAGE_STATE,
            reason=reason,
            message=reason,
            goal_id=self._current_goal_id,
        )
        self._record(
            TaskState.ABORTED,
            goal_id=self._current_goal_id,
            stage=STAGE_STATE,
            reason=reason,
            message=reason,
        )

    # ── 审计视图 ───────────────────────────────────────────────────────
    def as_dict(self) -> Dict[str, Any]:
        """返回可序列化的状态快照（审计 / 指标口径用）。"""
        return {
            "task_name": self._task.name,
            "state": self._state.value,
            "is_terminal": self.is_terminal,
            "goal_ids": list(self._goal_ids),
            "attempted_goal_ids": list(self._attempted),
            "consumed_goal_ids": list(self._consumed),
            "current_goal_id": self._current_goal_id,
            "failure": self._failure.as_dict() if self._failure else None,
            "results": [item.as_dict() for item in self._results],
            "history": [item.as_dict() for item in self._history],
        }

    # ── 内部实现 ───────────────────────────────────────────────────────
    def _require_runnable(self) -> None:
        if self.is_terminal:
            raise TaskError(
                f"状态机已进入终态 {self._state.value}，不能再推进"
                f"（已尝试的 goal_id：{list(self._attempted)}）"
            )
        if self._state is TaskState.WAITING:
            raise TaskError(
                "状态机处于 WAITING（等待外部事件），请先 resume() 再推进"
            )

    def _resolve_goal(self, goal_id: Optional[str]) -> str:
        """解析本次要推进的 goal_id（并挡住重复 / 未知 / 乱序）。"""
        pending = self.pending_goal_ids
        if goal_id is None:
            if not pending:
                raise TaskError(
                    f"任务 {self._task.name!r} 没有待推进的目标（已尝试 {list(self._attempted)}）"
                )
            return pending[0]
        if goal_id in self._attempted:
            raise TaskError(
                f"goal_id {goal_id!r} 已经推进过，不能重复推进"
                "（幂等重入不被接受）；已推进的 goal_id 有 "
                f"{list(self._attempted)}"
            )
        if goal_id not in self._goal_ids:
            raise TaskError(
                f"goal_id {goal_id!r} 不属于任务 {self._task.name!r}；"
                f"任务声明的 goal_id 有 {list(self._goal_ids)}"
            )
        if pending and goal_id != pending[0]:
            raise TaskError(
                f"goal_id {goal_id!r} 不是下一个待推进的目标；"
                f"下一个是 {pending[0]!r}"
            )
        return goal_id

    def _primitive_of(self, goal_id: str):
        for primitive in self._task.primitives:
            if primitive.goal_id == goal_id:
                return primitive
        raise TaskError(
            f"goal_id {goal_id!r} 在任务 {self._task.name!r} 里找不到对应的原语"
        )

    def _solve_goal(self, goal_id: str) -> PrimitiveResult:
        """经**单一求解器入口**求解单个 goal，并把末端位形写回关节游标。"""
        primitive = self._primitive_of(goal_id)
        sub_task = dataclasses.replace(self._task, primitives=(primitive,))
        options = dataclasses.replace(self._options, initial_q=self._cursor)
        try:
            task_result = self._solver(
                sub_task,
                scene=self._scene,
                transport=self._transport,
                options=options,
            )
        except Exception as exc:  # 求解器异常 → 稳定的失败语义
            return PrimitiveResult(
                name=primitive.name,
                goal_id=goal_id,
                success=False,
                stage=STAGE_STATE,
                reason="solver_error",
                message=f"求解器抛出异常：{type(exc).__name__}: {exc}",
            )
        if task_result is None or not getattr(task_result, "results", ()):
            return PrimitiveResult(
                name=primitive.name,
                goal_id=goal_id,
                success=False,
                stage=STAGE_STATE,
                reason="solver_error",
                message="求解器没有返回逐原语结果",
            )
        result = task_result.results[0]
        if result.goal_id != goal_id:
            return PrimitiveResult(
                name=primitive.name,
                goal_id=goal_id,
                success=False,
                stage=STAGE_STATE,
                reason="solver_error",
                message=(
                    f"求解器返回的 goal_id {result.goal_id!r} 与请求的 {goal_id!r} 不一致"
                ),
            )
        trajectory = getattr(result, "trajectory", None)
        samples = getattr(trajectory, "samples", ()) if trajectory is not None else ()
        if samples:
            self._cursor = tuple(float(v) for v in samples[-1].q)
        return result

    def _record(
        self,
        state: TaskState,
        *,
        goal_id: Optional[str] = None,
        stage: str = STAGE_STATE,
        reason: str = "",
        message: str = "",
    ) -> None:
        """登记一次状态转移（审计轨迹）并更新唯一的状态源。"""
        self._state = state
        self._history.append(
            StateTransition(
                index=len(self._history),
                state=state,
                goal_id=goal_id,
                stage=stage,
                reason=reason,
                message=message,
            )
        )
