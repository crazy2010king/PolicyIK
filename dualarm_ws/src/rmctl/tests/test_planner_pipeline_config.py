"""性能档位接线（S08/T02）：``max_solutions`` / ``optimize`` 可从配置层控制。

背景（S08 研究实测）：厂商 ``create_freespace_pipeline_profiles`` 的
``planner_range`` / ``max_solutions`` / ``optimize`` 是延迟-内存的最大杠杆
（range 0.01→329 点/1.08 s/4.8 GB；0.2→19 点/0.065 s/403 MB），但此前
**无法从配置层控制**。本任务把三个旋钮接到唯一配置来源
``workcell/planner/pipeline.yaml`` 上，**不补默认值**（缺字段 → blocked）。

覆盖：

(a) 新字段被加载进 spec 且类型正确；
(b) 缺字段 → blocked（reason 指名字段）；
(c) 未知字段 → blocked；
(d) 非法值（``max_solutions=0`` / ``optimize`` 非 bool）→ blocked；
(e) ``_make_profiles`` 确实把 ``optimize`` / ``max_solutions`` 透传给厂商
    （monkeypatch spy，断言调用参数）。

夹具纪律：临时工作区里手写 pipeline.yaml（不读任何 gitignore 目录）。
"""
from __future__ import annotations

import os
import sys

import pytest

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(TESTS_DIR, "..", "..", "..", ".."))
PKG_SRC = os.path.join(REPO_ROOT, "dualarm_ws", "src", "rmctl")

if PKG_SRC not in sys.path:
    sys.path.insert(0, PKG_SRC)

from rmctl.planner_service import config as planner_config  # noqa: E402
from rmctl.planner_service import engine  # noqa: E402


#: 合法基线内容（缺字段 / 非法值用例在此基础上改动）
BASE_PIPELINE = """\
version: 1
pipeline: FreespacePipeline
num_planners: 2
planning_time_s: 5.0
planner_range: 0.01
deterministic: true
max_solutions: 10
optimize: true
verify_resample_step_rad: 0.01
output_resample_step_rad: 0.01
"""


def _write_pipeline(ws, text=BASE_PIPELINE):
    """在临时工作区写 pipeline.yaml，返回工作区根。"""
    planner = os.path.join(str(ws), "workcell", "planner")
    os.makedirs(planner, exist_ok=True)
    with open(os.path.join(planner, "pipeline.yaml"), "w", encoding="utf-8") as fh:
        fh.write(text)
    return str(ws)


# --------------------------------------------------------------------------
# (a) 加载与类型
# --------------------------------------------------------------------------

def test_new_fields_are_loaded_into_spec(tmp_path):
    """max_solutions / optimize 进入 spec 且类型正确。"""
    ws = _write_pipeline(tmp_path)
    result = planner_config.load_pipeline_config(ws=ws)
    assert result["status"] == "ok", result["reason"]
    spec = result["spec"]
    assert spec["max_solutions"] == 10
    assert isinstance(spec["max_solutions"], int)
    assert spec["optimize"] is True
    assert isinstance(spec["optimize"], bool)
    # 既有字段不受影响
    assert spec["planner_range"] == 0.01
    assert spec["num_planners"] == 2


def test_repo_pipeline_yaml_has_the_new_fields():
    """仓库唯一来源 pipeline.yaml 已含新字段（S07 基线取值）。"""
    result = planner_config.load_pipeline_config()
    assert result["status"] == "ok", result["reason"]
    spec = result["spec"]
    assert spec["max_solutions"] == 10
    assert spec["optimize"] is True


# --------------------------------------------------------------------------
# (b) 缺字段 → blocked
# --------------------------------------------------------------------------

@pytest.mark.parametrize("field", ["max_solutions", "optimize"])
def test_missing_field_is_blocked(tmp_path, field):
    """缺字段 → blocked 且 reason 指名该字段（不补默认值）。"""
    text = "".join(
        line for line in BASE_PIPELINE.splitlines(keepends=True)
        if not line.startswith(field + ":")
    )
    ws = _write_pipeline(tmp_path, text)
    result = planner_config.load_pipeline_config(ws=ws)
    assert result["status"] == "blocked"
    assert result["spec"] is None
    assert field in result["reason"]
    assert result["next_action"]


# --------------------------------------------------------------------------
# (c) 未知字段 → blocked
# --------------------------------------------------------------------------

def test_unknown_field_is_blocked(tmp_path):
    """未知字段 → blocked（避免第二处配置源）。"""
    ws = _write_pipeline(tmp_path, BASE_PIPELINE + "foo: 1\n")
    result = planner_config.load_pipeline_config(ws=ws)
    assert result["status"] == "blocked"
    assert "foo" in result["reason"]


# --------------------------------------------------------------------------
# (d) 非法值 → blocked
# --------------------------------------------------------------------------

@pytest.mark.parametrize("bad,field", [
    ("max_solutions: 0\n", "max_solutions"),
    ("max_solutions: 1.5\n", "max_solutions"),
    ("max_solutions: true\n", "max_solutions"),
    ("optimize: 1\n", "optimize"),
    ("optimize: \"true\"\n", "optimize"),
])
def test_invalid_value_is_blocked(tmp_path, bad, field):
    """非法值 → blocked 且 reason 指名该字段。"""
    lines = []
    for line in BASE_PIPELINE.splitlines(keepends=True):
        key = line.split(":", 1)[0]
        if key == field:
            continue
        lines.append(line)
    ws = _write_pipeline(tmp_path, "".join(lines) + bad)
    result = planner_config.load_pipeline_config(ws=ws)
    assert result["status"] == "blocked"
    assert field in result["reason"]


# --------------------------------------------------------------------------
# (e) 透传给厂商
# --------------------------------------------------------------------------

def test_make_profiles_forwards_knobs_to_vendor(monkeypatch):
    """_make_profiles 把 optimize / max_solutions / planner_range 透传给厂商。"""
    calls = []

    def spy(**kwargs):
        calls.append(kwargs)
        return {"spy": True}

    monkeypatch.setattr(engine, "create_freespace_pipeline_profiles", spy)
    result = engine._make_profiles(
        "FreespacePipeline", 2, 5.0, 0.01, optimize=False, max_solutions=1
    )
    assert result == {"spy": True}
    assert len(calls) == 1
    kwargs = calls[0]
    assert kwargs["optimize"] is False
    assert kwargs["max_solutions"] == 1
    assert kwargs["num_planners"] == 2
    assert kwargs["planning_time"] == 5.0
    assert kwargs["planner_range"] == 0.01


def test_make_profiles_defaults_match_vendor_defaults(monkeypatch):
    """默认参数与厂商默认一致（True / 10），不改变 S07 行为。"""
    calls = []
    monkeypatch.setattr(
        engine, "create_freespace_pipeline_profiles",
        lambda **kwargs: calls.append(kwargs) or None,
    )
    engine._make_profiles("FreespacePipeline", 2, 5.0, 0.01)
    assert calls[0]["optimize"] is True
    assert calls[0]["max_solutions"] == 10


class _StopHere(Exception):
    """在 _make_profiles 处中断测试流（避免真跑规划）。"""


class _StubModel(object):
    """最小模型替身：只提供 _plan_inner 在 _make_profiles 之前用到的属性。"""

    joint_names = ["j%d" % i for i in range(7)]
    group = "group"
    tip = "tool"
    env = object()

    def set_state(self, state):
        self.state = state


class _StubProgram(object):
    def __init__(self, group, tcp_frame=None):
        self.group = group

    def set_joint_names(self, names):
        return self

    def move_to(self, target):
        return self

    def to_composite_instruction(self, names, tip):
        return object()


class _StubKeys(object):
    def __init__(self, mapping):
        self._mapping = mapping

    def has(self, key):
        return key in self._mapping

    def get(self, key):
        return self._mapping[key]

    def data(self):
        return dict(self._mapping)


class _StubTask(object):
    def getInputKeys(self):
        return _StubKeys({"planning_input": "input_data"})

    def getOutputKeys(self):
        return _StubKeys({"program": "output_data"})


class _StubFactory(object):
    def createTaskComposerNode(self, pipeline_name):
        return _StubTask()


class _StubComposer(object):
    factory = _StubFactory()


def test_plan_inner_reads_knobs_from_pipeline_spec(monkeypatch):
    """_plan_inner 从 pipeline_spec 取档位并透传给 _make_profiles。"""
    seen = {}

    def fake_make_profiles(pipeline, num_planners, planning_time_s, planner_range,
                           optimize=True, max_solutions=10):
        seen.update(
            pipeline=pipeline, num_planners=num_planners,
            planning_time_s=planning_time_s, planner_range=planner_range,
            optimize=optimize, max_solutions=max_solutions,
        )
        raise _StopHere()

    monkeypatch.setattr(engine, "_make_profiles", fake_make_profiles)
    monkeypatch.setattr(engine, "MotionProgram", _StubProgram)
    monkeypatch.setattr(engine, "assignCurrentStateAsSeed", lambda *a, **k: None)
    monkeypatch.setattr(engine, "apply_scene", lambda *a, **k: [])
    monkeypatch.setattr(engine, "_composer", lambda: _StubComposer())
    monkeypatch.setattr(
        engine, "load_pipeline_config",
        lambda ws=None: {
            "status": "ok",
            "spec": {
                "version": 1, "pipeline": "FreespacePipeline", "num_planners": 2,
                "planning_time_s": 5.0, "planner_range": 0.01,
                "deterministic": True, "max_solutions": 3, "optimize": False,
                "verify_resample_step_rad": 0.01,
                "output_resample_step_rad": 0.01,
            },
        },
    )
    monkeypatch.setattr(
        engine, "load_profiles_config",
        lambda ws=None: {
            "status": "ok",
            "spec": {
                "version": 1, "timeout_ms": 60000, "max_time_scaling": 3.0,
                "collision_margin_m": 0.02, "verify": {"continuous": True},
                "scene_version": 1,
            },
        },
    )

    from rmctl.planner_service.contract import PlanRequest

    request = PlanRequest(
        plan_id="t", profile="default", group="dual_arm",
        start_state=[0.0] * 7, goal={"type": "joint", "values": [0.1] * 7},
        constraints={}, seed=1, timeout_ms=1000, deterministic=False,
        scene_version=1, scene={},
    )
    with pytest.raises(_StopHere):
        engine.plan(request, model=_StubModel(), ws=None, seed_applied=True)
    assert seen["optimize"] is False
    assert seen["max_solutions"] == 3
