"""任务原语库与单臂求解器快路径的测试（S06-T03）。

覆盖面
------
* 原语目录：10 个原语齐全、未知原语名**点名**报错；
* 单臂快路径：在 ``scenarios/single_arm.yaml`` 场景上整条原语链跑通、
  **零逃逸**、误差在门槛内、**秒级**完成；
* 失败语义：IK 失败 / 碰撞逃逸 / 夹爪下发失败 / 夹爪离线 / 夹爪未到位，
  各自的 ``stage`` 与 ``reason`` 稳定可辨；
* 夹爪动作的**时间锚定**：逐点挂到 ``TrajectorySample.gripper_action``，
  且**不改动**冻结类型（``Trajectory`` / ``TrajectorySample`` / ``GripperAction``）。

零硬件：全部用 :class:`rm75_drake.device.transport.ScriptedTransport` 替身。
"""

from __future__ import annotations

import dataclasses
import os
import subprocess
import sys
import time
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = REPO_ROOT / "src"

from rm75_drake.gripper import GripperAction
from rm75_drake.planning.trajectory import Trajectory, TrajectorySample
from rm75_drake.tasks.goals import TaskError
from rm75_drake.tasks.primitives import (
    DEFAULT_ORIENTATION_TOLERANCE_RAD,
    DEFAULT_POSITION_TOLERANCE_M,
    PRIMITIVE_NAMES,
    PRIMITIVE_SPECS,
    primitive_spec,
)
from rm75_drake.tasks.schema import PoseSpec, parse_task
from rm75_drake.tasks.solver import (
    SolverOptions,
    solve_primitive,
    solve_task,
)


# ── 测试用任务数据 ──────────────────────────────────────────────────────
def _fast_path_task_data() -> dict:
    """单臂快路径任务（目标位姿均**可达**，且直线段碰撞自由）。"""
    return {
        "schema_version": 1,
        "name": "fast_path",
        "robot": "left",
        "objects": [
            {
                "name": "cube_a",
                "half_size": [0.02, 0.02, 0.02],
                "pose": {"xyz": [0.30, 0.0, 0.25], "rpy": [0.0, 0.0, 0.0]},
            }
        ],
        "primitives": [
            {
                "name": "approach",
                "kind": "motion",
                "target": {"xyz": [0.35, 0.0, 0.30], "rpy": [0.0, 0.0, 0.0]},
            },
            {
                "name": "pre_grasp",
                "kind": "motion",
                "target": {"xyz": [0.30, 0.0, 0.25], "rpy": [0.0, 0.0, 0.0]},
            },
            {
                "name": "close",
                "kind": "gripper",
                "action": "close",
                "target_object": "cube_a",
                "hold_seconds": 0.2,
            },
            {
                "name": "verify",
                "kind": "gripper",
                "action": "close",
                "target_object": "cube_a",
            },
            {
                "name": "lift",
                "kind": "motion",
                "target": {"xyz": [0.30, 0.0, 0.35], "rpy": [0.0, 0.0, 0.0]},
            },
        ],
    }


def _motion_task_data(name: str, xyz, rpy=(0.0, 0.0, 0.0)) -> dict:
    """只含一个运动原语的任务。"""
    return {
        "schema_version": 1,
        "name": name,
        "robot": "left",
        "primitives": [
            {"name": "approach", "kind": "motion", "target": {"xyz": list(xyz), "rpy": list(rpy)}}
        ],
    }


class _AlwaysCollidingChecker:
    """测试替身：碰撞检查器**永远**报告碰撞（用于负例）。"""

    def CheckConfigsCollisionFree(self, configs):  # noqa: N802 - Drake 同名接口
        return [0 for _ in configs]


class _AlwaysCollidingScene:
    """测试替身场景（只有 ``checker``，用于负例）。"""

    def __init__(self) -> None:
        self.checker = _AlwaysCollidingChecker()


# ── 原语目录 ────────────────────────────────────────────────────────────
def test_primitive_catalog_has_ten_primitives() -> None:
    """原语白名单是 10 个，且目录条目齐全。"""
    assert set(PRIMITIVE_NAMES) == {
        "approach",
        "pre_grasp",
        "close",
        "verify",
        "lift",
        "transit",
        "place",
        "open",
        "retreat",
        "handover",
    }
    assert set(PRIMITIVE_SPECS) == set(PRIMITIVE_NAMES)
    for name in PRIMITIVE_NAMES:
        spec = PRIMITIVE_SPECS[name]
        assert spec.name == name
        assert spec.kind in ("motion", "gripper", "verify", "bimanual_motion")
        assert spec.gripper_action_kind in (None, "close", "open")
        assert isinstance(spec.requires_object, bool)
        assert spec.description


def test_gripper_primitives_declare_action_and_object() -> None:
    """夹爪类原语必须声明夹爪动作，且需要绑定物体。"""
    assert PRIMITIVE_SPECS["close"].gripper_action_kind == "close"
    assert PRIMITIVE_SPECS["close"].requires_object is True
    assert PRIMITIVE_SPECS["open"].gripper_action_kind == "open"
    assert PRIMITIVE_SPECS["verify"].kind == "verify"
    assert PRIMITIVE_SPECS["handover"].kind == "bimanual_motion"


def test_unknown_primitive_name_is_named() -> None:
    """未知名 → TaskError 并**点名**该名字。"""
    with pytest.raises(TaskError) as exc:
        primitive_spec("fly")
    message = str(exc.value)
    assert "fly" in message
    assert "approach" in message


# ── 单臂快路径 ──────────────────────────────────────────────────────────
def test_fast_path_chain_succeeds_within_two_seconds(single_arm_scene) -> None:
    """整条单臂原语链：跑通、零逃逸、误差在门槛内、秒级完成。"""
    from rm75_drake.device.transport import ScriptedTransport

    task = parse_task(_fast_path_task_data())
    transport = ScriptedTransport()
    transport.open()

    started = time.perf_counter()
    result = solve_task(task, scene=single_arm_scene, transport=transport)
    elapsed = time.perf_counter() - started

    assert result.success, (result.stage, result.reason, result.message)
    assert result.stage == "done"
    assert [item.name for item in result.results] == [
        "approach",
        "pre_grasp",
        "close",
        "verify",
        "lift",
    ]
    for item in result.results:
        assert item.success
        assert item.stage == "done"
        assert item.reason == "completed"
        if item.position_error_m is not None:
            assert item.position_error_m <= DEFAULT_POSITION_TOLERANCE_M
            assert item.orientation_error_rad <= DEFAULT_ORIENTATION_TOLERANCE_RAD
    # 夹爪原语：命令真的下发了（替身记录到调用）
    assert transport.called("command_gripper_position") == 1
    assert transport.read_gripper().position == 0
    # 秒级完成（离线门槛必须挂在可秒级完成的链路上）
    assert elapsed < 2.0, elapsed


def test_motion_primitive_uses_zero_escapes_and_five_ms_grid(single_arm_scene) -> None:
    """运动原语：碰撞复检零逃逸，产出 5 ms 网格轨迹。"""
    task = parse_task(_motion_task_data("one", [0.35, 0.0, 0.30]))
    result = solve_task(task, scene=single_arm_scene)
    assert result.success, result.message
    item = result.results[0]
    assert item.trajectory is not None
    assert item.trajectory.dt == pytest.approx(0.005)
    assert len(item.trajectory.samples) > 1
    # 关节角序列落在软件限位内（Trajectory 构造期已校验）
    assert all(len(sample.q) == 7 for sample in item.trajectory.samples)


def test_motion_primitive_can_target_an_object(single_arm_scene) -> None:
    """``target_object`` 的运动原语用物体位姿作为目标。

    注意：任务**文件**的 schema（S06-T01 冻结）要求运动类原语必须写
    ``target``；这里直接构造 :class:`TaskSpec`，验证求解器侧
    「显式 target 缺失时回落到物体位姿」的那条分支。
    """
    from rm75_drake.tasks.schema import ObjectSpec, PrimitiveSpec, TaskSpec

    task = TaskSpec(
        schema_version=1,
        name="obj_target",
        robot="left",
        primitives=(
            PrimitiveSpec(
                name="approach",
                goal_id="approach",
                kind="motion",
                target_object="cube_a",
            ),
        ),
        objects=(
            ObjectSpec(
                name="cube_a",
                half_size=(0.02, 0.02, 0.02),
                pose=PoseSpec(xyz=(0.30, 0.0, 0.25), rpy=(0.0, 0.0, 0.0)),
            ),
        ),
    )
    result = solve_task(task, scene=single_arm_scene)
    assert result.success, result.message
    assert result.results[0].position_error_m <= DEFAULT_POSITION_TOLERANCE_M


def test_motion_primitive_without_any_target_is_named(single_arm_scene) -> None:
    """既无 target 也无 target_object → stage="schema"、reason="missing_target"。"""
    from rm75_drake.tasks.schema import PrimitiveSpec, TaskSpec

    task = TaskSpec(
        schema_version=1,
        name="no_target",
        robot="left",
        primitives=(
            PrimitiveSpec(name="approach", goal_id="approach", kind="motion"),
        ),
    )
    result = solve_task(task, scene=single_arm_scene)
    assert not result.success
    assert result.stage == "schema"
    assert result.reason == "missing_target"


# ── 失败语义 ────────────────────────────────────────────────────────────
def test_unreachable_target_names_ik_stage_and_reason(single_arm_scene) -> None:
    """目标不可达 → stage="ik"、reason="unreachable"（解析 IK 词表透出）。"""
    task = parse_task(_motion_task_data("far", [1.60, 0.0, 0.60]))
    result = solve_task(task, scene=single_arm_scene)
    assert not result.success
    assert result.stage == "ik"
    assert result.reason == "unreachable"
    assert "unreachable" in result.results[0].message


def test_collision_escape_names_collision_stage(single_arm_scene) -> None:
    """碰撞逃逸 → stage="collision"、reason="escape"。"""
    task = parse_task(_motion_task_data("coll", [0.35, 0.0, 0.30]))
    result = solve_task(task, scene=_AlwaysCollidingScene())
    assert not result.success
    assert result.stage == "collision"
    assert result.reason == "escape"
    assert "逃逸" in result.results[0].message


def test_missing_scene_is_named(single_arm_scene) -> None:
    """运动原语缺场景 → stage="schema"、reason="missing_scene"。"""
    task = parse_task(_motion_task_data("nos", [0.35, 0.0, 0.30]))
    result = solve_task(task, scene=None)
    assert not result.success
    assert result.stage == "schema"
    assert result.reason == "missing_scene"


def test_task_stops_at_first_failure(single_arm_scene) -> None:
    """首个失败即停：后续原语不再执行。"""
    data = _fast_path_task_data()
    data["primitives"][0]["target"] = {"xyz": [1.60, 0.0, 0.60], "rpy": [0.0, 0.0, 0.0]}
    task = parse_task(data)
    result = solve_task(task, scene=single_arm_scene)
    assert not result.success
    assert result.stage == "ik"
    assert len(result.results) == 1


def test_bimanual_primitive_is_reported_as_unsupported(single_arm_scene) -> None:
    """双臂原语在单臂求解器里**显式**报不支持（不假装成功）。"""
    data = {
        "schema_version": 1,
        "name": "ho",
        "robot": "dual_arm",
        "objects": [
            {
                "name": "box_a",
                "half_size": [0.02, 0.02, 0.02],
                "pose": {"xyz": [0.30, 0.0, 0.25], "rpy": [0.0, 0.0, 0.0]},
            }
        ],
        "primitives": [
            {
                "name": "handover",
                "kind": "bimanual_motion",
                "target": {"xyz": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]},
                "target_object": "box_a",
            }
        ],
    }
    task = parse_task(data)
    result = solve_task(task, scene=single_arm_scene)
    assert not result.success
    assert result.stage == "schema"
    assert result.reason == "unsupported_kind"


# ── 夹爪原语 ────────────────────────────────────────────────────────────
def test_gripper_primitive_issues_command_and_reads_back(single_arm_scene) -> None:
    """夹爪原语：命令下发被记录，回读位置到位。"""
    from rm75_drake.device.transport import ScriptedTransport

    task = parse_task(_fast_path_task_data())
    transport = ScriptedTransport()
    transport.open()
    result = solve_task(task, scene=single_arm_scene, transport=transport)

    close = next(item for item in result.results if item.name == "close")
    assert close.success
    assert close.command_issued is True
    assert transport.called("command_gripper_position") == 1
    assert transport.read_gripper().online is True
    assert transport.read_gripper().position == 0


def test_gripper_primitive_without_transport_is_not_verified(single_arm_scene) -> None:
    """无传输层 → 命令**不下发**（command_issued=False，且明说未验证）。"""
    task = parse_task(_fast_path_task_data())
    result = solve_task(task, scene=single_arm_scene, transport=None)
    close = next(item for item in result.results if item.name == "close")
    assert close.success
    assert close.command_issued is False
    assert "未下发" in close.message


def test_verify_fails_when_gripper_is_offline(single_arm_scene) -> None:
    """夹爪离线 → stage="verify"、reason="gripper_offline"。"""
    from rm75_drake.device.transport import ScriptedTransport

    task = parse_task(_fast_path_task_data())
    transport = ScriptedTransport()
    transport.open()
    transport.set_gripper_offline(True)
    result = solve_task(task, scene=single_arm_scene, transport=transport)

    verify = next(item for item in result.results if item.name == "verify")
    assert not verify.success
    assert verify.stage == "verify"
    assert verify.reason == "gripper_offline"
    assert not result.success
    assert result.stage == "verify"


def test_verify_fails_when_gripper_not_closed(single_arm_scene) -> None:
    """夹爪在线但位置没到位 → stage="verify"、reason="gripper_not_closed"。"""
    from rm75_drake.device.transport import ScriptedTransport

    task = parse_task(_fast_path_task_data())
    transport = ScriptedTransport()
    transport.open()
    transport.set_gripper_position(500)
    # 只求解 verify 原语（前面的 close 会把回读位置改成 0）
    verify_primitive = task.primitive("verify")
    verify = solve_primitive(
        verify_primitive, task=task, scene=single_arm_scene, transport=transport
    )
    assert not verify.success
    assert verify.stage == "verify"
    assert verify.reason == "gripper_not_closed"


def test_gripper_action_is_anchored_on_every_sample(single_arm_scene) -> None:
    """夹爪动作**逐点**挂到采样点上（复用冻结类型已有的字段）。"""
    from rm75_drake.device.transport import ScriptedTransport

    task = parse_task(_fast_path_task_data())
    transport = ScriptedTransport()
    transport.open()
    result = solve_task(task, scene=single_arm_scene, transport=transport)

    close = next(item for item in result.results if item.name == "close")
    trajectory = close.trajectory
    assert isinstance(trajectory, Trajectory)
    assert trajectory is not None
    assert len(trajectory.samples) > 1
    for sample in trajectory.samples:
        assert isinstance(sample, TrajectorySample)
        assert isinstance(sample.gripper_action, GripperAction)
        assert sample.gripper_action.kind == "close"
    assert tuple(trajectory.gripper_schedule) == (trajectory.samples[0].gripper_action,)


def test_frozen_types_are_not_modified() -> None:
    """本任务**不得**改动冻结类型（Trajectory / TrajectorySample / GripperAction）。"""
    assert [field.name for field in dataclasses.fields(TrajectorySample)] == [
        "seq",
        "t_abs",
        "q",
        "gripper_action",
    ]
    assert [field.name for field in dataclasses.fields(Trajectory)] == [
        "samples",
        "dt",
        "robot_name",
        "gripper_schedule",
        "acceleration_limits",
    ]
    assert [field.name for field in dataclasses.fields(GripperAction)] == [
        "kind",
        "position",
        "speed",
        "force",
        "hold_seconds",
        "notes",
    ]


# ── 惰性导入（零 pydrake 依赖）──────────────────────────────────────────
def test_solver_module_import_does_not_load_pydrake() -> None:
    """独立解释器里 import 求解器也不得把 pydrake 拉进 sys.modules。"""
    env = dict(os.environ)
    env["PYTHONPATH"] = str(SRC_DIR) + os.pathsep + env.get("PYTHONPATH", "")
    code = (
        "import sys; import rm75_drake.tasks; "
        "import rm75_drake.tasks.primitives; "
        "import rm75_drake.tasks.solver; "
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
