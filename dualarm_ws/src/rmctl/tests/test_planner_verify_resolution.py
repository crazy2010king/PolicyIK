"""四道校验的**分辨率与点数解耦**（S08/T01）。

问题（S08 研究已实测复现）：``rmctl.verify.check_limits`` 的速度/加速度/jerk
是对**原始点**做数值微分，点数少 → 采样间隔大 → **峰值被抹平** → 校验分辨率
被静默削弱：把点数从 329 降到 19 会让 ``limit_violation_jerk`` 场景从 FAILED
翻成 SUCCESS（R002 红线被悄悄破坏）。

解法（本任务交付）：

* ``rmctl.verify.resample_trajectory`` —— 按关节空间最大步长做三次 Hermite
  （Catmull-Rom 切线）稠密化，几何路径不变、只是采样更密；
* ``rmctl.planner_service.engine.verify_and_heal`` 增加两个独立旋钮：
  ``resample_step_rad``（**校验**用稠密轨迹）与 ``output_resample_step_rad``
  （**下发**轨迹密度）→ 校验点数与下发点数彻底解耦；
* ``pipeline.yaml`` 增加 ``verify_resample_step_rad`` / ``output_resample_step_rad``
  （**不补默认值**：缺字段 → blocked，与既有配置纪律一致）。

本文件覆盖四组断言：

(a) 点数无关性 —— 同一条几何轨迹在「原始点数」与「稠密化后」两种输入下，
    四道校验结论**一致**；
(b) 「少点抹平峰值」的反例 —— 少点轨迹的粗校验**通过**，稠密化后仍被判
    limits 未通过（jerk 超限）→ 稠密化**不会**削弱校验；
(c) ``resample_trajectory`` 自身性质 —— 时间严格递增、点数上限生效、
    几何端点不变、关闭时原样返回；
(d) 配置纪律 —— 缺字段 / 未知字段 / 非法值 → blocked（逐字段指名）。

夹具纪律：模型在 pytest 临时目录里用 ``build_model.build_model()`` 物化绝对
路径模型；规划配置从仓库复制进临时工作区。**不读任何 gitignore 目录**。
"""
from __future__ import annotations

import copy
import math
import os
import shutil
import sys

import pytest

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(TESTS_DIR, "..", "..", "..", ".."))
PKG_SRC = os.path.join(REPO_ROOT, "dualarm_ws", "src", "rmctl")
RM_DESC_SRC = os.path.join(REPO_ROOT, "dualarm_ws", "src", "rm_description")
RM_DESC_SCRIPTS = os.path.join(RM_DESC_SRC, "scripts")
PLANNER_CFG_DIR = os.path.join(REPO_ROOT, "workcell", "planner")

for _path in (PKG_SRC, RM_DESC_SCRIPTS):
    if _path not in sys.path:
        sys.path.insert(0, _path)

import build_model  # noqa: E402

from rmctl.planner_service import config as planner_config  # noqa: E402
from rmctl.planner_service import engine  # noqa: E402
from rmctl.planner_service.contract import PlanRequest  # noqa: E402
from rmctl.verify import (  # noqa: E402
    check_limits,
    resample_trajectory,
    verify,
)

#: 稠密化步长（rad）—— 与 ``workcell/planner/pipeline.yaml`` 的取值同量级。
STEP_RAD = 0.01


# --------------------------------------------------------------------------
# 轨迹构造
# --------------------------------------------------------------------------

def _trajectory(times, joint1_values):
    """时间轴 + joint1 的一维取值 → 7 关节点位序列（其余关节不动）。"""
    points = []
    for value in joint1_values:
        point = [0.0] * 7
        point[0] = float(value)
        points.append(point)
    return points, [float(t) for t in times]


def _smooth_trajectory(n_knots=9, amplitude=0.2, omega=1.0, horizon=1.0):
    """光滑正弦轨迹（真实导数远低于限值 → 任何密度下都应通过）。"""
    times = [horizon * i / (n_knots - 1) for i in range(n_knots)]
    values = [amplitude * math.sin(omega * t) for t in times]
    return _trajectory(times, values)


#: 「少点抹平峰值」的反例轨迹：采样不均匀 + 局部曲率大。
#:
#: 粗校验（5 个点）算出的 jerk 是一个**跨整段区间**的差分 → 峰值被抹平 →
#: 通过；稠密化（0.01 rad）后数值微分逼近真实导数 → jerk 峰值暴露 → 不通过。
PEAK_TIMES = [0.0, 0.3488, 0.5544, 0.8298, 1.0]
PEAK_VALUES = [-0.7242, -0.5601, -0.5130, -0.3139, -0.2105]

#: 本文件测的是**校验分辨率**（点数 ↔ 校验强度），与"实测限值口径"无关。
#: S13（2026-11-23）起 joint_limits.yaml 的 acceleration 已按真机实测**逐关节收紧**
#: （1.48~2.44 rad/s^2）→ 上面那条固定的"少点抹平 jerk 峰值"轨迹会在**粗校验**
#: 阶段就撞上 acceleration 上限，前置条件（粗校验通过）不再成立 —— 变的是限值口径，
#: 不是分辨率机制。故在需要该前置条件的用例里把模型限值**钉死**在回填前的占位值，
#: 让断言只检验分辨率机制本身（不改判据、不放宽任何真实限值）。
PINNED_LIMITS = {"acceleration": 5.2359, "jerk": 30.0, "velocity": 3.1416}


def _pinned_model(model):
    """返回一份限值被钉死（回填前占位值）的模型副本。"""
    pinned = copy.copy(model)
    pinned.limits = {joint: dict(PINNED_LIMITS) for joint in model.joint_names}
    return pinned


# --------------------------------------------------------------------------
# 夹具
# --------------------------------------------------------------------------

@pytest.fixture(scope="session")
def planner_ws(tmp_path_factory):
    """临时工作区：物化好的绝对路径模型 + 复制进来的规划配置。"""
    ws = tmp_path_factory.mktemp("planner_verify_resolution_ws")
    build_model.build_model(
        ws=str(ws),
        out=str(ws / "build" / "model"),
        pkg=RM_DESC_SRC,
    )
    cfg_dst = ws / "workcell" / "planner"
    cfg_dst.mkdir(parents=True)
    for name in ("pipeline.yaml", "profiles.yaml"):
        shutil.copyfile(os.path.join(PLANNER_CFG_DIR, name),
                        str(cfg_dst / name))
    return str(ws)


@pytest.fixture
def model(planner_ws):
    """每个用例一份干净的模型（碰撞检查会改环境状态）。"""
    from rmctl.model import Model

    return Model(ws=planner_ws)


def _make_request(planner_ws, **overrides):
    data = {
        "plan_id": "p-res",
        "profile": "FREESPACE",
        "group": "rm75_arm",
        "start_state": [0.0] * 7,
        "goal": {"type": "joint", "values": [0.2, -0.3, 0.1, 0.0, 0.0, 0.0, 0.0]},
        "constraints": {},
        "seed": 7,
        "timeout_ms": 20000,
        "scene_version": 1,
    }
    data.update(overrides)
    return PlanRequest.from_dict(data, ws=planner_ws)


# --------------------------------------------------------------------------
# (c) resample_trajectory 自身性质
# --------------------------------------------------------------------------

def test_resample_trajectory_keeps_endpoints_and_monotonic_time():
    """稠密化：时间严格递增 + 两端点逐点不变（几何路径不变）。"""
    points, times = _smooth_trajectory()
    dense_points, dense_times = resample_trajectory(points, times, STEP_RAD)

    assert len(dense_points) > len(points), "点数必须变多（否则没稠密化）"
    assert all(b > a for a, b in zip(dense_times, dense_times[1:])), \
        "时间戳必须严格递增"
    assert dense_points[0] == pytest.approx(points[0]), "起点必须逐点不变"
    assert dense_points[-1] == pytest.approx(points[-1]), "终点必须逐点不变"
    assert dense_times[0] == pytest.approx(times[0])
    assert dense_times[-1] == pytest.approx(times[-1])
    # 原始节点仍在稠密轨迹里（几何一致，仅点数不同）
    for point, time in zip(points, times):
        assert any(
            dp == pytest.approx(point) and dt == pytest.approx(time)
            for dp, dt in zip(dense_points, dense_times)
        ), "原始节点必须被保留"


def test_resample_trajectory_respects_point_cap():
    """点数上限生效：每段最多细分 ``max_sub`` 份 → 总点数有界。"""
    points, times = _smooth_trajectory(n_knots=9, amplitude=2.0)
    dense_points, _ = resample_trajectory(points, times, STEP_RAD, max_sub=3)
    # 8 段 × 最多 3 份 + 1 个末点
    assert len(dense_points) <= 3 * (len(points) - 1) + 1
    assert len(dense_points) > len(points)


def test_resample_trajectory_disabled_returns_input():
    """``max_step_rad <= 0`` → 原样返回（关闭稠密化的显式语义）。"""
    points, times = _smooth_trajectory()
    same_points, same_times = resample_trajectory(points, times, 0.0)
    assert same_points == points
    assert same_times == times
    same_points, same_times = resample_trajectory(points, times, -1.0)
    assert same_points == points
    assert same_times == times


def test_resample_trajectory_rejects_non_monotonic_time():
    """时间轴非严格递增 → 原样返回（不猜语义，判定仍归四道校验）。"""
    points, times = _smooth_trajectory()
    broken = list(times)
    broken[2] = broken[1]
    same_points, same_times = resample_trajectory(points, broken, STEP_RAD)
    assert same_points == points
    assert same_times == broken


# --------------------------------------------------------------------------
# (a) 点数无关性：同一几何轨迹，两种密度 → 结论一致
# --------------------------------------------------------------------------

def test_verification_conclusion_is_independent_of_point_density(model):
    """同一条光滑轨迹：原始点数与稠密化后，四道校验结论**一致**。"""
    points, times = _smooth_trajectory()
    dense_points, dense_times = resample_trajectory(points, times, STEP_RAD)
    assert len(dense_points) > len(points)

    coarse = verify(model, points, times)
    dense = verify(model, dense_points, dense_times)

    assert coarse["ok"] is True, coarse["issues"]
    assert dense["ok"] is True, dense["issues"]
    for name in ("limits", "discrete_collision", "continuous_collision"):
        assert coarse["checks"][name]["ok"] == dense["checks"][name]["ok"], name
    assert coarse["issues"] == dense["issues"] == []


# --------------------------------------------------------------------------
# (b) 「少点抹平峰值」的反例
# --------------------------------------------------------------------------

def test_few_points_do_not_flatten_jerk_peak(model):
    """少点轨迹：粗校验**通过**，稠密化后仍被判 limits 未通过（jerk 超限）。

    这正是 S08 研究实测的失效模式（点数 329 → 19 让越限场景从 FAILED 翻成
    SUCCESS）：点数少 → 数值微分的峰值被抹平 → 校验被静默削弱。稠密化后
    结论**不被削弱**（反而更严格）。
    """
    model = _pinned_model(model)
    points, times = _trajectory(PEAK_TIMES, PEAK_VALUES)

    coarse_issues, _ = check_limits(model, points, times)
    assert coarse_issues == [], (
        "前置条件：粗校验（少点）应当通过（这正是峰值被抹平的表现）：%s"
        % coarse_issues
    )

    dense_points, dense_times = resample_trajectory(points, times, STEP_RAD)
    dense_issues, _ = check_limits(model, dense_points, dense_times)
    assert dense_issues, "稠密化后必须仍然判 limits 未通过"
    assert any("jerk 超限" in issue for issue in dense_issues), dense_issues

    # 整份四道校验报告的结论：粗 = 通过，稠密 = 不过 → 校验没有被削弱
    assert verify(model, points, times)["ok"] is True
    assert verify(model, dense_points, dense_times)["ok"] is False


def test_dense_verification_never_weakens_the_verdict(model):
    """单调性：粗校验不过 → 稠密校验也不过（不允许「变绿」）。"""
    model = _pinned_model(model)
    # (1) 位置越限：粗/稠密都不过
    over, over_times = _trajectory([0.0, 1.0], [0.0, 3.5])
    dense_over, dense_over_times = resample_trajectory(over, over_times, STEP_RAD)
    assert verify(model, over, over_times)["ok"] is False
    assert verify(model, dense_over, dense_over_times)["ok"] is False

    # (2) 少点峰值轨迹：粗通过、稠密不过（变红，不是变绿）
    points, times = _trajectory(PEAK_TIMES, PEAK_VALUES)
    dense_points, dense_times = resample_trajectory(points, times, STEP_RAD)
    assert verify(model, points, times)["ok"] is True
    assert verify(model, dense_points, dense_times)["ok"] is False


# --------------------------------------------------------------------------
# 引擎层：校验点数与下发点数解耦
# --------------------------------------------------------------------------

def test_verify_and_heal_verifies_dense_but_returns_requested_output(model):
    """``verify_and_heal``：校验用稠密轨迹，下发点数由 output 参数独立决定。"""
    points, times = _smooth_trajectory()

    plain = engine.verify_and_heal(model, points, times)
    assert plain["verified_points"] == len(points)
    assert len(plain["points"]) == len(points)

    dense = engine.verify_and_heal(
        model, points, times,
        resample_step_rad=STEP_RAD,
        output_resample_step_rad=STEP_RAD,
    )
    assert dense["verified_points"] > len(points), "校验必须用稠密轨迹"
    assert len(dense["points"]) > len(points), "下发轨迹按 output 参数稠密化"
    assert dense["report"]["ok"] is True, dense["report"]["issues"]
    # 下发轨迹的时间轴仍严格递增
    assert all(b > a for a, b in zip(dense["times"], dense["times"][1:]))


def test_verify_and_heal_reports_jerk_violation_after_densification(model):
    """引擎层：少点轨迹经稠密校验后必须**判不过**（红线不被削弱）。

    ``max_scale=1.0`` 模拟黄金场景 ``limit_violation_jerk``（请求里
    ``constraints.time_scaling = 1.0`` → 不允许保路径降速）→ 稠密校验暴露的
    jerk 峰值**无处可藏**。
    """
    points, times = _trajectory(PEAK_TIMES, PEAK_VALUES)
    healed = engine.verify_and_heal(
        model, points, times, max_scale=1.0, resample_step_rad=STEP_RAD
    )
    assert healed["report"]["ok"] is False, healed["report"]["issues"]
    limits = healed["report"]["checks"]["limits"]
    assert limits["ok"] is False
    assert any("jerk 超限" in issue for issue in limits["issues"]), limits
    assert healed["scale"] is None, "缩放被上限封顶 → 不降速"
    assert any("人工放宽" in note for note in healed["notes"]), healed["notes"]


def test_verify_and_heal_retime_scales_original_time_axis(model):
    """保路径降速：缩放作用在**原始**时间轴上，几何路径不变。"""
    points = [[0.0] * 7, [0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]
    times = [0.0, 0.015]
    healed = engine.verify_and_heal(
        model, points, times, max_scale=1.5,
        resample_step_rad=STEP_RAD, output_resample_step_rad=STEP_RAD,
    )
    assert healed["report"]["ok"] is True, healed["report"]["issues"]
    assert healed["scale"] is not None and healed["scale"] > 1.001
    assert healed["times"][-1] > times[-1], "降速必须拉长时间轴"
    # 几何不变：起点/终点与原轨迹一致
    assert healed["points"][0] == pytest.approx(points[0])
    assert healed["points"][-1] == pytest.approx(points[1])


# --------------------------------------------------------------------------
# 端到端：metrics 里的点数三兄弟
# --------------------------------------------------------------------------

def test_plan_metrics_expose_point_counts(planner_ws, model):
    """规划响应里可观测：原始点数 / 校验点数 / 下发点数。"""
    response = engine.plan(
        _make_request(planner_ws), ws=planner_ws, model=model
    )
    assert response.status == "SUCCESS", response.failure_reason
    metrics = response.metrics
    assert metrics["trajectory_points"] >= 1
    assert metrics["verified_points"] >= metrics["trajectory_points"], (
        "开启稠密化后校验点数不应少于原始点数"
    )
    assert metrics["output_points"] == len(response.trajectory["points"])
    assert metrics["points"] == metrics["output_points"]
    assert metrics["verify"]["ok"] is True


# --------------------------------------------------------------------------
# (d) 配置纪律：缺字段 / 未知字段 / 非法值 → blocked
# --------------------------------------------------------------------------

_VALID_PIPELINE = (
    "version: 1\n"
    "pipeline: FreespacePipeline\n"
    "num_planners: 2\n"
    "planning_time_s: 5.0\n"
    "planner_range: 0.01\n"
    "deterministic: true\n"
    "max_solutions: 10\n"
    "optimize: true\n"
    "verify_resample_step_rad: 0.01\n"
    "output_resample_step_rad: 0.01\n"
)


def _write_pipeline(tmp_path, text):
    planner = tmp_path / "workcell" / "planner"
    planner.mkdir(parents=True, exist_ok=True)
    (planner / "pipeline.yaml").write_text(text, encoding="utf-8")
    return str(tmp_path)


def test_pipeline_config_exposes_new_fields(tmp_path):
    """新字段被加载进 spec 且类型正确。"""
    ws = _write_pipeline(tmp_path, _VALID_PIPELINE)
    result = planner_config.load_pipeline_config(ws=ws)
    assert result["status"] == "ok", result["reason"]
    spec = result["spec"]
    assert spec["verify_resample_step_rad"] == pytest.approx(0.01)
    assert spec["output_resample_step_rad"] == pytest.approx(0.01)


def test_pipeline_config_blocks_missing_new_field(tmp_path):
    """缺字段 → blocked 且 reason 指名（**不补默认值**）。"""
    text = "\n".join(
        line for line in _VALID_PIPELINE.splitlines()
        if not line.startswith("verify_resample_step_rad")
    ) + "\n"
    ws = _write_pipeline(tmp_path, text)
    result = planner_config.load_pipeline_config(ws=ws)
    assert result["status"] == "blocked"
    assert result["spec"] is None
    assert "verify_resample_step_rad" in result["reason"]
    assert result["next_action"]


def test_pipeline_config_blocks_unknown_field(tmp_path):
    """未知字段 → blocked（避免第二处配置源）。"""
    ws = _write_pipeline(tmp_path, _VALID_PIPELINE + "secret_knob: 7\n")
    result = planner_config.load_pipeline_config(ws=ws)
    assert result["status"] == "blocked"
    assert "secret_knob" in result["reason"]


def test_pipeline_config_blocks_illegal_values(tmp_path):
    """非法值（负数 / 非数值）→ blocked 且 reason 指名字段。"""
    for bad in ("-0.01", "'abc'"):
        text = _VALID_PIPELINE.replace(
            "verify_resample_step_rad: 0.01",
            "verify_resample_step_rad: %s" % bad,
        )
        ws = _write_pipeline(tmp_path, text)
        result = planner_config.load_pipeline_config(ws=ws)
        assert result["status"] == "blocked", bad
        assert "verify_resample_step_rad" in result["reason"], result["reason"]
