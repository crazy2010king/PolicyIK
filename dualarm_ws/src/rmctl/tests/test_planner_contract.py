"""规划服务契约层测试（T1）：PlanRequest / PlanResponse + 声明式配置。

覆盖：
* 合法请求的 ``to_dict`` / ``from_dict`` 往返一致；
* 逐字段指名的非法输入（缺字段 / 类型错 / 非有限数 / 未知 goal 类型）；
* 响应对象的 JSON 可序列化、四道校验键齐全、坏行解析不抛裸异常；
* 轨迹摘要的确定性（同输入同哈希、异输入异哈希）；
* 两份声明式 YAML 的加载（字段齐全 / 缺失 → blocked + 可粘贴 next_action）。

测试只用 **git 跟踪的仓库文件**（``workcell/planner/*.yaml``）与 pytest 临时目录，
绝不依赖 .gitignore 目录。
"""
import copy
import json
import math
import os

import pytest

from rmctl.planner_service import PROCESS_ROLE
from rmctl.planner_service import config as planner_config
from rmctl.planner_service.contract import (
    GOAL_TYPES,
    STATUSES,
    VERIFICATION_KEYS,
    InvalidRequest,
    PlanRequest,
    PlanResponse,
    parse_response_line,
    trajectory_digest,
)


# --------------------------------------------------------------------------
# 夹具
# --------------------------------------------------------------------------

def _valid_request_dict():
    """一份合法的请求字典（字段与设计 §4.3 逐项对齐）。"""
    return {
        "plan_id": "plan-0001",
        "profile": "FREESPACE",
        "group": "left_arm",
        "start_state": [0.0, 0.1, -0.2, 0.3, -0.4, 0.5, -0.6],
        "goal": {"type": "joint", "values": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]},
        "constraints": {"time_scaling": 1.0, "collision_margin": 0.005},
        "seed": 42,
        "timeout_ms": 20000,
        "scene_version": 1,
    }


def _trajectory():
    return {
        "frame": ["j1", "j2", "j3", "j4", "j5", "j6", "j7"],
        "points": [
            {"t": 0.0, "positions": [0.0] * 7},
            {"t": 0.01, "positions": [0.1] * 7},
        ],
        "time_base": 100.0,
        "source_profile": "FREESPACE",
    }


# --------------------------------------------------------------------------
# 包声明
# --------------------------------------------------------------------------

def test_process_role_is_rm_planner():
    """进程角色名与 D004 三进程模型一致。"""
    assert PROCESS_ROLE == "rm_planner"


def test_statuses_match_design():
    """状态集合与设计 §4.3 逐字一致（唯一、无重复）。"""
    assert STATUSES == (
        "SUCCESS",
        "FAILED",
        "TIMEOUT",
        "CANCELLED",
        "INVALID_REQUEST",
    )
    assert len(set(STATUSES)) == len(STATUSES)


# --------------------------------------------------------------------------
# PlanRequest 正向
# --------------------------------------------------------------------------

def test_valid_request_roundtrip():
    """合法请求 to_dict / from_dict 往返一致。"""
    data = _valid_request_dict()
    request = PlanRequest.from_dict(data)
    # to_dict 输出稳定（scene / deterministic 恒在），往返一致
    assert request.to_dict() == dict(data, scene=None, deterministic=True)
    again = PlanRequest.from_dict(request.to_dict())
    assert again.to_dict() == request.to_dict()
    assert again == request


def test_request_to_dict_is_json_serializable():
    """请求 to_dict 可 JSON 序列化（所有 float 有限）。"""
    request = PlanRequest.from_dict(_valid_request_dict())
    text = json.dumps(request.to_dict(), allow_nan=False)
    assert "plan-0001" in text


def test_pose_goal_accepted():
    """笛卡尔目标（6 维）合法。"""
    data = _valid_request_dict()
    data["goal"] = {"type": "pose", "values": [0.1, 0.2, 0.3, 0.0, 0.0, 1.57]}
    request = PlanRequest.from_dict(data)
    assert request.goal["type"] == "pose"
    assert len(request.goal["values"]) == 6
    assert GOAL_TYPES == ("joint", "pose")


def test_scene_obstacles_accepted():
    """场景障碍物（盒子）合法。"""
    data = _valid_request_dict()
    data["scene"] = {
        "obstacles": [
            {"name": "box_a", "box_size": [0.1, 0.2, 0.3], "position": [0.4, 0.0, 0.1]}
        ]
    }
    request = PlanRequest.from_dict(data)
    assert request.to_dict()["scene"]["obstacles"][0]["name"] == "box_a"


# --------------------------------------------------------------------------
# PlanRequest 反向（逐字段指名）
# --------------------------------------------------------------------------

@pytest.mark.parametrize(
    "field", ["plan_id", "seed", "timeout_ms", "goal", "start_state"]
)
def test_missing_required_field_raises_named_invalid_request(field):
    """必填字段缺失 → InvalidRequest，且 reason 含字段名。"""
    data = _valid_request_dict()
    data.pop(field)
    with pytest.raises(InvalidRequest) as excinfo:
        PlanRequest.from_dict(data)
    assert excinfo.value.field == field
    assert field in excinfo.value.reason


def test_unknown_goal_type_raises():
    """未知 goal 类型 → InvalidRequest（field=goal.type）。"""
    data = _valid_request_dict()
    data["goal"] = {"type": "cartesian_path", "values": [0.0] * 6}
    with pytest.raises(InvalidRequest) as excinfo:
        PlanRequest.from_dict(data)
    assert excinfo.value.field == "goal.type"
    assert "cartesian_path" in excinfo.value.reason


def test_start_state_wrong_count_raises():
    """start_state 不是 7 个数 → InvalidRequest。"""
    data = _valid_request_dict()
    data["start_state"] = [0.0] * 6
    with pytest.raises(InvalidRequest) as excinfo:
        PlanRequest.from_dict(data)
    assert excinfo.value.field == "start_state"


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf")])
def test_start_state_non_finite_raises(bad):
    """start_state 含 NaN / Inf → InvalidRequest（有限数纪律）。"""
    data = _valid_request_dict()
    data["start_state"] = [0.0] * 6 + [bad]
    with pytest.raises(InvalidRequest) as excinfo:
        PlanRequest.from_dict(data)
    assert excinfo.value.field.startswith("start_state")
    assert not math.isfinite(bad)  # 确认用例本身有意义


def test_time_scaling_above_profile_limit_raises():
    """constraints.time_scaling 超过 max_time_scaling → InvalidRequest。"""
    data = _valid_request_dict()
    data["constraints"] = {"time_scaling": 2.0}
    with pytest.raises(InvalidRequest) as excinfo:
        PlanRequest.from_dict(data, max_time_scaling=1.5)
    assert excinfo.value.field == "constraints.time_scaling"
    assert "1.5" in excinfo.value.reason


def test_time_scaling_above_profiles_yaml_limit_raises():
    """上限来自 profiles.yaml（唯一来源），不是代码里写死的数。"""
    profiles = planner_config.load_profiles_config()
    assert profiles["status"] == "ok"
    limit = profiles["spec"]["max_time_scaling"]
    data = _valid_request_dict()
    data["constraints"] = {"time_scaling": limit + 0.5}
    with pytest.raises(InvalidRequest) as excinfo:
        PlanRequest.from_dict(data)
    assert excinfo.value.field == "constraints.time_scaling"


def test_negative_seed_raises():
    """seed 为负 → InvalidRequest。"""
    data = _valid_request_dict()
    data["seed"] = -1
    with pytest.raises(InvalidRequest) as excinfo:
        PlanRequest.from_dict(data)
    assert excinfo.value.field == "seed"


def test_non_positive_timeout_raises():
    """timeout_ms <= 0 → InvalidRequest（硬超时必须为正）。"""
    data = _valid_request_dict()
    data["timeout_ms"] = 0
    with pytest.raises(InvalidRequest) as excinfo:
        PlanRequest.from_dict(data)
    assert excinfo.value.field == "timeout_ms"


def test_unknown_request_field_raises():
    """未知请求字段 → InvalidRequest（不静默忽略）。"""
    data = _valid_request_dict()
    data["surprise"] = 1
    with pytest.raises(InvalidRequest) as excinfo:
        PlanRequest.from_dict(data)
    assert excinfo.value.field == "surprise"


# --------------------------------------------------------------------------
# PlanResponse
# --------------------------------------------------------------------------

def test_response_to_dict_json_serializable_with_verification_keys():
    """响应 to_dict 可 JSON 序列化，且四道校验键齐全。"""
    response = PlanResponse(
        plan_id="plan-0001",
        status="SUCCESS",
        trajectory=_trajectory(),
        verification={
            "limits": {"ok": True, "issues": []},
            "discrete_collision": {"ok": True, "issues": []},
            "continuous_collision": {"ok": True, "issues": []},
            "ok": True,
            "issues": [],
        },
        failure_reason="",
        metrics={
            "pipeline": "FreespacePipeline",
            "seed": 42,
            "num_planners": 2,
            "elapsed_total_s": 0.5,
            "nodes": [{"name": "OMPL", "elapsed_s": 0.2, "status": "OK", "message": ""}],
            "verify": {"ok": True},
            "retimed_scale": 1.0,
        },
    )
    data = response.to_dict()
    json.dumps(data, allow_nan=False)
    for key in VERIFICATION_KEYS:
        assert key in data["verification"]
    assert data["status"] == "SUCCESS"


def test_response_default_verification_has_four_keys():
    """默认 verification 骨架自带四道校验键（服务端不漏字段）。"""
    response = PlanResponse(plan_id="p", status="FAILED")
    for key in VERIFICATION_KEYS:
        assert key in response.verification


def test_parse_response_line_roundtrip():
    """一行 JSON ↔ 响应对象往返一致。"""
    response = PlanResponse(
        plan_id="plan-0001",
        status="TIMEOUT",
        failure_reason="服务端硬超时",
        metrics={"seed": 7},
    )
    line = json.dumps(response.to_dict(), allow_nan=False)
    parsed = parse_response_line(line)
    assert parsed.status == "TIMEOUT"
    assert parsed.plan_id == "plan-0001"
    assert parsed.failure_reason == "服务端硬超时"
    assert parsed.to_dict() == response.to_dict()


@pytest.mark.parametrize(
    "line", ["", "   ", "not json", "[1, 2, 3]", '{"status": "WAT"}']
)
def test_parse_bad_line_returns_invalid_request(line):
    """坏响应行 → INVALID_REQUEST 响应对象（不抛裸异常）。"""
    response = parse_response_line(line)
    assert response.status == "INVALID_REQUEST"
    assert response.failure_reason


def test_parse_response_line_non_string():
    """非字符串输入同样走 INVALID_REQUEST 路径。"""
    response = parse_response_line(None)
    assert response.status == "INVALID_REQUEST"


# --------------------------------------------------------------------------
# 确定性摘要
# --------------------------------------------------------------------------

def test_trajectory_digest_is_stable():
    """同输入 → 同哈希。"""
    first = trajectory_digest(_trajectory())
    second = trajectory_digest(copy.deepcopy(_trajectory()))
    assert first == second
    assert len(first) == 64


def test_trajectory_digest_differs_on_change():
    """异输入 → 异哈希。"""
    base = _trajectory()
    other = _trajectory()
    other["points"][1]["positions"][0] = 0.2
    assert trajectory_digest(base) != trajectory_digest(other)


def test_trajectory_digest_ignores_dict_order():
    """摘要不依赖 dict 键顺序（规范化）。"""
    base = _trajectory()
    shuffled = {
        "source_profile": base["source_profile"],
        "time_base": base["time_base"],
        "points": base["points"],
        "frame": base["frame"],
    }
    assert trajectory_digest(base) == trajectory_digest(shuffled)


# --------------------------------------------------------------------------
# 声明式配置
# --------------------------------------------------------------------------

def test_pipeline_yaml_loads_with_expected_fields():
    """pipeline.yaml 加载成功且字段齐全。"""
    result = planner_config.load_pipeline_config()
    assert result["status"] == "ok"
    spec = result["spec"]
    for key in ("version", "pipeline", "num_planners", "planning_time_s",
                "planner_range", "deterministic"):
        assert key in spec
    assert spec["pipeline"] == "FreespacePipeline"
    assert spec["deterministic"] is True


def test_profiles_yaml_loads_with_expected_fields():
    """profiles.yaml 加载成功且字段齐全。"""
    result = planner_config.load_profiles_config()
    assert result["status"] == "ok"
    spec = result["spec"]
    for key in ("version", "timeout_ms", "max_time_scaling",
                "collision_margin_m", "verify", "scene_version"):
        assert key in spec
    assert spec["timeout_ms"] > 0
    assert spec["verify"]["continuous"] is True


def test_missing_yaml_is_blocked_with_next_action(tmp_path):
    """YAML 缺失 → status=blocked 且 next_action 非空（不抛裸异常）。"""
    result = planner_config.load_pipeline_config(ws=str(tmp_path))
    assert result["status"] == "blocked"
    assert result["spec"] is None
    assert result["next_action"]
    assert result["reason"]
    assert result["evidence"]


def test_malformed_yaml_is_blocked(tmp_path):
    """YAML 语法错 → blocked（不抛裸异常）。"""
    planner = tmp_path / "workcell" / "planner"
    planner.mkdir(parents=True)
    (planner / "pipeline.yaml").write_text("version: 1\n  bad: [\n", encoding="utf-8")
    result = planner_config.load_pipeline_config(ws=str(tmp_path))
    assert result["status"] == "blocked"
    assert result["next_action"]


def test_yaml_unknown_field_is_blocked(tmp_path):
    """YAML 出现未知字段 → blocked（避免第二处配置源）。"""
    planner = tmp_path / "workcell" / "planner"
    planner.mkdir(parents=True)
    (planner / "profiles.yaml").write_text(
        "version: 1\ntimeout_ms: 100\nmax_time_scaling: 1.5\n"
        "collision_margin_m: 0.005\nverify:\n  continuous: true\n"
        "scene_version: 1\nsecret_knob: 7\n",
        encoding="utf-8",
    )
    result = planner_config.load_profiles_config(ws=str(tmp_path))
    assert result["status"] == "blocked"
    assert "secret_knob" in result["reason"]


def test_yaml_missing_required_field_is_blocked(tmp_path):
    """YAML 缺必需字段 → blocked 且 reason 指名（不补默认值）。"""
    planner = tmp_path / "workcell" / "planner"
    planner.mkdir(parents=True)
    (planner / "profiles.yaml").write_text("version: 1\n", encoding="utf-8")
    result = planner_config.load_profiles_config(ws=str(tmp_path))
    assert result["status"] == "blocked"
    assert "timeout_ms" in result["reason"]
    assert result["next_action"]


def test_planner_config_paths_are_under_workcell():
    """配置路径挂在 <ws>/workcell/planner/ 下（声明式、可 diff）。"""
    path = planner_config.pipeline_config_path()
    assert path.endswith(os.path.join("workcell", "planner", "pipeline.yaml"))
    assert planner_config.profiles_config_path().endswith(
        os.path.join("workcell", "planner", "profiles.yaml")
    )
