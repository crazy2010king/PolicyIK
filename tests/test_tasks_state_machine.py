"""任务状态机（S06-T04）的测试：**单一主状态源** + **原子 goal_id**。

覆盖面
------
* 正常链推进到 ``SUCCEEDED``：每个 goal_id **恰好被消费一次**；
* 中途失败 → ``FAILED`` 且 ``stage`` / ``reason`` 透出，后续 goal 不再推进；
* 重复推进同一 goal_id → **报错并点名**（幂等重入不得静默成功）；
* ``abort()`` → ``ABORTED``；
* **状态是单一真相源**：对外只暴露一个可变状态（无第二份状态字段）；
* 求解器异常 → 稳定的 ``stage="state" / reason="solver_error"``；
* ``WAITING`` / ``resume()`` 与 ``run()`` 的终态语义；
* 关节游标连续：上一个原语的末端位形传给下一个原语。

**零 pydrake**：全部用假求解器注入，不 import Drake。
"""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = REPO_ROOT / "src"

from rm75_drake.planning.trajectory import Trajectory, TrajectorySample
from rm75_drake.tasks.goals import TaskError
from rm75_drake.tasks.schema import parse_task
from rm75_drake.tasks.solver import PrimitiveResult, TaskResult
from rm75_drake.tasks.state_machine import (
    STAGE_STATE,
    TASK_REASONS,
    TASK_STAGES,
    TaskState,
    TaskStateMachine,
)

# ── 任务夹具 ────────────────────────────────────────────────────────────
def _make_task(goal_ids: Tuple[str, ...] = ("g_approach", "g_pre_grasp", "g_close")):
    """构造一个 3 原语的任务（goal_id 显式且唯一）。"""
    names = ("approach", "pre_grasp", "close")
    primitives = []
    for name, goal_id in zip(names, goal_ids):
        if name == "close":
            primitives.append(
                {
                    "name": name,
                    "kind": "gripper",
                    "goal_id": goal_id,
                    "action": "close",
                    "target_object": "cube_a",
                }
            )
        else:
            primitives.append(
                {
                    "name": name,
                    "kind": "motion",
                    "goal_id": goal_id,
                    "target": {"xyz": [0.45, 0.0, 0.18], "rpy": [0.0, 0.0, 0.0]},
                }
            )
    return parse_task(
        {
            "schema_version": 1,
            "name": "state_machine_case",
            "robot": "left",
            "objects": [
                {
                    "name": "cube_a",
                    "half_size": [0.02, 0.02, 0.02],
                    "pose": {"xyz": [0.45, 0.0, 0.02], "rpy": [0.0, 0.0, 0.0]},
                }
            ],
            "primitives": primitives,
        }
    )


# ── 假求解器（零 Drake / 零硬件）──────────────────────────────────────
class FakeSolver:
    """假求解器：按 goal_id 返回预设结果。

    签名与 :func:`rm75_drake.tasks.solver.solve_task` 一致
    （``solver(task, *, scene, transport, options) -> TaskResult``），
    因此状态机调用它的方式与真实路径完全相同。
    """

    def __init__(
        self,
        failures: Optional[Dict[str, Tuple[str, str]]] = None,
        raising: Optional[Dict[str, Exception]] = None,
        q_step: float = 0.0,
    ) -> None:
        self.failures = dict(failures or {})
        self.raising = dict(raising or {})
        self.q_step = float(q_step)
        self.calls: List[Dict[str, Any]] = []

    def __call__(
        self,
        task,
        *,
        scene: Any = None,
        transport: Any = None,
        options: Any = None,
    ) -> TaskResult:
        primitive = task.primitives[0]
        goal_id = primitive.goal_id
        initial_q = tuple(options.initial_q) if options is not None else ()
        self.calls.append(
            {
                "task_name": task.name,
                "goal_ids": tuple(p.goal_id for p in task.primitives),
                "initial_q": initial_q,
            }
        )
        if goal_id in self.raising:
            raise self.raising[goal_id]
        if goal_id in self.failures:
            stage, reason = self.failures[goal_id]
            result = PrimitiveResult(
                name=primitive.name,
                goal_id=goal_id,
                success=False,
                stage=stage,
                reason=reason,
                message=f"注入的失败：{stage}/{reason}",
            )
        else:
            result = PrimitiveResult(
                name=primitive.name,
                goal_id=goal_id,
                success=True,
                stage="done",
                reason="completed",
                trajectory=self._trajectory(initial_q),
                duration_s=0.1,
            )
        return TaskResult(
            task_name=task.name,
            success=result.success,
            stage=result.stage,
            reason=result.reason,
            results=(result,),
            duration_s=0.1,
            message=result.message,
        )

    def _trajectory(self, initial_q: Tuple[float, ...]) -> Optional[Trajectory]:
        if not self.q_step or not initial_q:
            return None
        q = list(initial_q)
        q[0] = float(q[0]) + self.q_step
        sample = TrajectorySample(seq=0, t_abs=0.0, q=tuple(q))
        return Trajectory(samples=(sample,), dt=0.005)


# ── 正常链 ──────────────────────────────────────────────────────────────
def test_normal_chain_reaches_succeeded_and_consumes_each_goal_once() -> None:
    """正常链推进到 SUCCEEDED，每个 goal_id 恰好被消费一次。"""
    task = _make_task()
    solver = FakeSolver()
    machine = TaskStateMachine(task, solver=solver)

    assert machine.state is TaskState.PENDING
    results = machine.run()

    assert machine.state is TaskState.SUCCEEDED
    assert machine.is_terminal
    assert len(results) == 3
    assert machine.consumed_goal_ids == ("g_approach", "g_pre_grasp", "g_close")
    assert len(set(machine.consumed_goal_ids)) == 3
    assert machine.attempted_goal_ids == ("g_approach", "g_pre_grasp", "g_close")
    # 每个 goal 恰好一次求解调用。
    assert [call["goal_ids"] for call in solver.calls] == [
        ("g_approach",),
        ("g_pre_grasp",),
        ("g_close",),
    ]
    assert machine.failure is None


def test_advance_consumes_exactly_one_goal_per_call() -> None:
    """每次 advance() 只消费一个 goal（不是整个任务）。"""
    task = _make_task()
    solver = FakeSolver()
    machine = TaskStateMachine(task, solver=solver)

    first = machine.advance()
    assert first.goal_id == "g_approach"
    assert machine.consumed_goal_ids == ("g_approach",)
    assert machine.pending_goal_ids == ("g_pre_grasp", "g_close")
    assert len(solver.calls) == 1

    machine.advance("g_pre_grasp")
    assert machine.consumed_goal_ids == ("g_approach", "g_pre_grasp")
    assert len(solver.calls) == 2


def test_joint_cursor_is_carried_to_the_next_goal() -> None:
    """上一个原语的末端位形传给下一个原语（链式位形连续）。"""
    task = _make_task()
    solver = FakeSolver(q_step=0.1)
    machine = TaskStateMachine(task, solver=solver)

    machine.advance("g_approach")
    machine.advance("g_pre_grasp")

    assert solver.calls[0]["initial_q"][0] == pytest.approx(0.0)
    assert solver.calls[1]["initial_q"][0] == pytest.approx(0.1)


# ── 失败语义 ────────────────────────────────────────────────────────────
def test_failure_midway_sets_failed_and_keeps_stage_reason() -> None:
    """中途失败 → FAILED 且 stage / reason 透出，后续 goal 不再推进。"""
    task = _make_task()
    solver = FakeSolver(failures={"g_pre_grasp": ("ik", "unreachable")})
    machine = TaskStateMachine(task, solver=solver)

    machine.advance("g_approach")
    result = machine.advance("g_pre_grasp")

    assert result.success is False
    assert machine.state is TaskState.FAILED
    assert machine.failure is not None
    assert machine.failure.stage == "ik"
    assert machine.failure.reason == "unreachable"
    assert machine.failure.goal_id == "g_pre_grasp"

    # 后续 goal 不再推进。
    assert machine.pending_goal_ids == ("g_close",)
    with pytest.raises(TaskError):
        machine.advance("g_close")
    assert len(solver.calls) == 2


def test_solver_exception_is_mapped_to_stable_stage_state() -> None:
    """求解器抛异常 → 稳定失败语义（stage=state / reason=solver_error）。"""
    task = _make_task()
    solver = FakeSolver(raising={"g_approach": RuntimeError("boom")})
    machine = TaskStateMachine(task, solver=solver)

    result = machine.advance("g_approach")

    assert result.success is False
    assert result.stage == STAGE_STATE
    assert result.reason == "solver_error"
    assert machine.state is TaskState.FAILED
    assert machine.failure is not None
    assert machine.failure.stage == STAGE_STATE
    assert machine.failure.reason == "solver_error"
    assert "boom" in machine.failure.message


def test_failure_stage_and_reason_are_in_the_stable_vocabulary() -> None:
    """失败阶段 / 原因必须落在既有词表内。"""
    assert "state" in TASK_STAGES
    for reason in ("duplicate_goal", "unknown_goal", "goal_out_of_order",
                   "solver_error", "aborted"):
        assert reason in TASK_REASONS


# ── 原子 goal_id：重复 / 未知 / 乱序 ───────────────────────────────────
def test_duplicate_goal_id_raises_and_names_it() -> None:
    """重复推进同一 goal_id → 报错并点名该 goal_id。"""
    task = _make_task()
    solver = FakeSolver()
    machine = TaskStateMachine(task, solver=solver)
    machine.advance("g_approach")

    with pytest.raises(TaskError) as excinfo:
        machine.advance("g_approach")
    message = str(excinfo.value)
    assert "g_approach" in message
    # 幂等重入不得静默成功：状态未被推进。
    assert machine.consumed_goal_ids == ("g_approach",)
    assert len(solver.calls) == 1


def test_duplicate_goal_id_after_success_still_raises() -> None:
    """整链成功后再次推进已消费的 goal_id → 仍然报错并点名。"""
    task = _make_task()
    solver = FakeSolver()
    machine = TaskStateMachine(task, solver=solver)
    machine.run()
    assert machine.state is TaskState.SUCCEEDED

    with pytest.raises(TaskError) as excinfo:
        machine.advance("g_approach")
    assert "g_approach" in str(excinfo.value)


def test_unknown_goal_id_raises_and_names_it() -> None:
    """未知 goal_id → 报错并点名。"""
    task = _make_task()
    machine = TaskStateMachine(task, solver=FakeSolver())

    with pytest.raises(TaskError) as excinfo:
        machine.advance("g_nonexistent")
    assert "g_nonexistent" in str(excinfo.value)


def test_out_of_order_goal_id_raises() -> None:
    """跳过中间目标 → 报错（原子推进必须按序）。"""
    task = _make_task()
    machine = TaskStateMachine(task, solver=FakeSolver())

    with pytest.raises(TaskError) as excinfo:
        machine.advance("g_close")
    assert "g_close" in str(excinfo.value)
    assert "g_approach" in str(excinfo.value)


def test_duplicate_goal_id_in_task_is_rejected_at_parse_time() -> None:
    """任务内重复 goal_id → 解析阶段即报错并点名。"""
    with pytest.raises(TaskError) as excinfo:
        parse_task(
            {
                "schema_version": 1,
                "name": "dup_goals",
                "robot": "left",
                "primitives": [
                    {
                        "name": "approach",
                        "kind": "motion",
                        "goal_id": "same",
                        "target": {"xyz": [0.4, 0.0, 0.2], "rpy": [0.0, 0.0, 0.0]},
                    },
                    {
                        "name": "lift",
                        "kind": "motion",
                        "goal_id": "same",
                        "target": {"xyz": [0.4, 0.0, 0.3], "rpy": [0.0, 0.0, 0.0]},
                    },
                ],
            }
        )
    assert "same" in str(excinfo.value)


def test_state_machine_itself_rejects_duplicate_goal_ids() -> None:
    """即便绕开解析器直接构造 TaskSpec，状态机自身也拒绝重复 goal_id。"""
    from rm75_drake.tasks.schema import PoseSpec, PrimitiveSpec, TaskSpec

    def _prim(name: str) -> PrimitiveSpec:
        return PrimitiveSpec(
            name=name,
            goal_id="same",
            kind="motion",
            target=PoseSpec(xyz=(0.4, 0.0, 0.2), rpy=(0.0, 0.0, 0.0)),
        )

    spec = TaskSpec(
        schema_version=1,
        name="dup_goals",
        robot="left",
        primitives=(_prim("approach"), _prim("lift")),
    )
    with pytest.raises(TaskError) as excinfo:
        TaskStateMachine(spec, solver=FakeSolver())
    assert "same" in str(excinfo.value)


# ── 中止 / 等待 ────────────────────────────────────────────────────────
def test_abort_sets_aborted() -> None:
    """abort() → ABORTED，且失败原因可查。"""
    task = _make_task()
    machine = TaskStateMachine(task, solver=FakeSolver())
    machine.advance("g_approach")

    machine.abort(reason="operator_stop")

    assert machine.state is TaskState.ABORTED
    assert machine.is_terminal
    assert machine.failure is not None
    assert machine.failure.stage == STAGE_STATE
    assert machine.failure.reason == "operator_stop"


def test_abort_on_terminal_machine_raises() -> None:
    """终态下再 abort → 显式报错。"""
    task = _make_task()
    machine = TaskStateMachine(task, solver=FakeSolver())
    machine.run()
    assert machine.state is TaskState.SUCCEEDED

    with pytest.raises(TaskError):
        machine.abort()


def test_waiting_blocks_advance_until_resume() -> None:
    """WAITING 期间不得推进；resume() 后继续。"""
    task = _make_task()
    machine = TaskStateMachine(task, solver=FakeSolver())
    machine.wait(reason="perception_pending")
    assert machine.state is TaskState.WAITING

    with pytest.raises(TaskError):
        machine.advance("g_approach")

    machine.resume()
    assert machine.state is TaskState.RUNNING
    machine.advance("g_approach")
    assert machine.consumed_goal_ids == ("g_approach",)


def test_run_stops_at_waiting() -> None:
    """run() 遇到 WAITING 即返回（等待外部事件）。"""
    task = _make_task()
    machine = TaskStateMachine(task, solver=FakeSolver())
    machine.wait()
    assert machine.run() == ()
    assert machine.state is TaskState.WAITING


# ── 单一真相源 ──────────────────────────────────────────────────────────
def test_state_is_the_only_mutable_state_source() -> None:
    """状态机对外只暴露一个可变状态（没有第二份状态字段）。"""
    task = _make_task()
    machine = TaskStateMachine(task, solver=FakeSolver())
    machine.advance("g_approach")

    # 实例上只有一个字段持有 TaskState 值。
    holders = [
        name
        for name, value in vars(machine).items()
        if isinstance(value, TaskState)
    ]
    assert holders == ["_state"]

    # 对外只有 state 这一个状态入口（没有 status / phase 之类的副本）。
    public_state_attrs = [
        name
        for name in dir(machine)
        if not name.startswith("_")
        and isinstance(getattr(machine, name, None), TaskState)
    ]
    assert public_state_attrs == ["state"]

    # 历史只是审计轨迹：它记录了状态，但真相源仍然只有 state。
    assert machine.history[-1].state is machine.state
    assert isinstance(machine.history, tuple)


def test_history_is_read_only_audit_trail() -> None:
    """history 是不可变轨迹（不是可写的真相源）。"""
    task = _make_task()
    machine = TaskStateMachine(task, solver=FakeSolver())
    machine.run()

    assert len(machine.history) >= 3
    assert machine.history[0].state is TaskState.RUNNING
    assert machine.history[-1].state is TaskState.SUCCEEDED
    with pytest.raises(AttributeError):  # dataclasses.FrozenInstanceError
        machine.history[0].state = TaskState.FAILED  # type: ignore[misc]


def test_snapshot_dict_is_serializable() -> None:
    """状态快照可序列化（审计 / 指标口径用）。"""
    import json

    task = _make_task()
    machine = TaskStateMachine(task, solver=FakeSolver())
    machine.run()

    payload = machine.as_dict()
    json.dumps(payload, ensure_ascii=False)
    assert payload["state"] == "succeeded"
    assert payload["consumed_goal_ids"] == ["g_approach", "g_pre_grasp", "g_close"]


# ── 惰性导入（零 pydrake 依赖）──────────────────────────────────────────
def test_state_machine_module_import_does_not_load_pydrake() -> None:
    """独立解释器里 import 状态机也不得把 pydrake 拉进 sys.modules。"""
    env = dict(os.environ)
    env["PYTHONPATH"] = str(SRC_DIR) + os.pathsep + env.get("PYTHONPATH", "")
    code = (
        "import sys; import rm75_drake.tasks; "
        "import rm75_drake.tasks.state_machine; "
        "assert 'pydrake' not in sys.modules, sorted(sys.modules); "
        "assert 'rm75_drake.drivers' not in sys.modules; "
        "print('ok')"
    )
    proc = subprocess.run(
        [sys.executable, "-c", code],
        cwd=str(REPO_ROOT),
        env=env,
        capture_output=True,
        text=True,
        timeout=120,
    )
    assert proc.returncode == 0, proc.stderr
    assert "ok" in proc.stdout
