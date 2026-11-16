"""规划引擎（``rmctl.planner_service.engine``）的测试。

夹具纪律：

* **模型夹具**在 pytest 临时目录里用 ``build_model.build_model()`` **物化绝对
  路径模型**（``build/model/rm75_clean.urdf`` 的 mesh 路径是绝对路径，Tesseract
  的资源定位器不认源码里的相对路径），因此**不依赖仓库里 gitignore 的
  ``build/model`` 状态**，也不读任何 .gitignore 目录；
* 规划配置（``workcell/planner/*.yaml``）从仓库**复制**进临时工作区，引擎的
  「唯一来源」纪律在测试里同样成立；
* 场景障碍物会**改环境**，所以 ``Model`` 用函数级夹具（每个用例一份干净环境）。

覆盖：简单关节规划成功 / per-node 指标 / 四道校验 / **跨进程确定性** /
种子生效 / 限位不过 / IK 无解 / 非法请求透出 / 保路径降速 / 上限封顶 /
场景增量命令 / 零重复实现守卫 / 配置不可得。
"""
from __future__ import annotations

import os
import shutil
import subprocess
import sys

import pytest

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(TESTS_DIR, "..", "..", "..", ".."))
PKG_SRC = os.path.join(REPO_ROOT, "dualarm_ws", "src", "rmctl")
RM_DESC_SRC = os.path.join(REPO_ROOT, "dualarm_ws", "src", "rm_description")
RM_DESC_SCRIPTS = os.path.join(RM_DESC_SRC, "scripts")

for _path in (PKG_SRC, RM_DESC_SCRIPTS):
    if _path not in sys.path:
        sys.path.insert(0, _path)

import build_model  # noqa: E402

import rmctl.verify as rmctl_verify  # noqa: E402
from rmctl.model import Model  # noqa: E402
from rmctl.planner_service import engine  # noqa: E402
from rmctl.planner_service.contract import (  # noqa: E402
    InvalidRequest,
    PlanRequest,
    trajectory_digest,
)

#: 引擎源码（零重复实现守卫用）
ENGINE_SOURCE = os.path.join(
    PKG_SRC, "rmctl", "planner_service", "engine.py"
)

#: 规划配置（唯一来源）
PLANNER_CFG_DIR = os.path.join(REPO_ROOT, "workcell", "planner")

START = [0.0] * 7
GOAL = [0.2, -0.3, 0.1, 0.0, 0.0, 0.0, 0.0]


# --------------------------------------------------------------------------
# 夹具
# --------------------------------------------------------------------------

@pytest.fixture(scope="session")
def planner_ws(tmp_path_factory):
    """临时工作区：物化好的绝对路径模型 + 复制进来的规划配置。"""
    ws = tmp_path_factory.mktemp("planner_ws")
    build_model.build_model(
        ws=str(ws),
        out=str(ws / "build" / "model"),
        pkg=RM_DESC_SRC,
    )
    cfg_dst = ws / "workcell" / "planner"
    cfg_dst.mkdir(parents=True)
    for name in ("pipeline.yaml", "profiles.yaml"):
        shutil.copyfile(
            os.path.join(PLANNER_CFG_DIR, name), str(cfg_dst / name)
        )
    return str(ws)


@pytest.fixture
def model(planner_ws):
    """每个用例一份干净的模型（场景障碍物会改环境，不能共用）。"""
    return Model(ws=planner_ws)


def make_request(planner_ws, **overrides):
    """构造合法请求（非法用例由契约层负责，这里只做正例底座）。"""
    data = {
        "plan_id": "p-test",
        "profile": "FREESPACE",
        "group": "rm75_arm",
        "start_state": list(START),
        "goal": {"type": "joint", "values": list(GOAL)},
        "constraints": {},
        "seed": 7,
        "timeout_ms": 20000,
        "scene_version": 1,
    }
    data.update(overrides)
    return PlanRequest.from_dict(data, ws=planner_ws)


def _digest_worker(seed, goal=None):
    """子进程里跑一次规划并打印摘要（供跨进程确定性用例调用）。"""
    import tempfile

    from rmctl.vendor_env import ensure_libstdcxx

    ensure_libstdcxx()
    tmp = tempfile.mkdtemp()
    build_model.build_model(
        ws=tmp, out=os.path.join(tmp, "build", "model"), pkg=RM_DESC_SRC
    )
    cfg_dst = os.path.join(tmp, "workcell", "planner")
    os.makedirs(cfg_dst)
    for name in ("pipeline.yaml", "profiles.yaml"):
        shutil.copyfile(os.path.join(PLANNER_CFG_DIR, name),
                        os.path.join(cfg_dst, name))
    local_model = Model(ws=tmp)
    request = PlanRequest.from_dict(
        {
            "plan_id": "p-sub",
            "profile": "FREESPACE",
            "group": "rm75_arm",
            "start_state": list(START),
            "goal": {"type": "joint", "values": list(goal or GOAL)},
            "constraints": {},
            "seed": int(seed),
            "timeout_ms": 20000,
            "scene_version": 1,
        },
        ws=tmp,
    )
    response = engine.plan(request, ws=tmp, model=local_model)
    print("STATUS:%s" % response.status)
    print("DIGEST:%s" % trajectory_digest(response.trajectory))


def _run_in_subprocess(seed, goal=None):
    """在**独立子进程**里跑一次规划，返回 (status, digest)。"""
    code = (
        "import sys\n"
        "sys.path.insert(0, %r)\n"
        "sys.path.insert(0, %r)\n"
        "import test_planner_engine as t\n"
        "t._digest_worker(%d, %r)\n"
    ) % (PKG_SRC, RM_DESC_SCRIPTS, int(seed), goal)
    env = os.environ.copy()
    env["PYTHONPATH"] = os.pathsep.join(
        [p for p in (PKG_SRC, RM_DESC_SCRIPTS, env.get("PYTHONPATH", "")) if p]
    )
    completed = subprocess.run(
        [sys.executable, "-c", code],
        cwd=TESTS_DIR,
        env=env,
        capture_output=True,
        text=True,
        timeout=900,
    )
    assert completed.returncode == 0, (
        "子进程规划失败（exit=%s）:\n%s" % (completed.returncode, completed.stderr[-2000:])
    )
    status = None
    digest = None
    for line in completed.stdout.splitlines():
        if line.startswith("STATUS:"):
            status = line.split(":", 1)[1].strip()
        elif line.startswith("DIGEST:"):
            digest = line.split(":", 1)[1].strip()
    assert status and digest, "子进程没有产出状态/摘要:\n%s" % completed.stdout[-2000:]
    return status, digest


# --------------------------------------------------------------------------
# 正例：关节规划 + 指标 + 四道校验
# --------------------------------------------------------------------------

def test_joint_plan_succeeds_with_trajectory(planner_ws, model):
    response = engine.plan(make_request(planner_ws), ws=planner_ws, model=model)
    assert response.status == "SUCCESS"
    assert response.failure_reason == ""
    trajectory = response.trajectory
    assert trajectory is not None
    assert len(trajectory["points"]) > 1
    for point in trajectory["points"]:
        assert len(point["positions"]) == 7
    times = trajectory["times"]
    assert all(b > a for a, b in zip(times, times[1:])), "时间戳必须严格递增"


def test_metrics_expose_per_node_information(planner_ws, model):
    response = engine.plan(make_request(planner_ws), ws=planner_ws, model=model)
    metrics = response.metrics
    assert metrics["pipeline"] == "FreespacePipeline"
    assert metrics["seed"] == 7
    assert metrics["num_planners"] >= 1
    assert metrics["elapsed_total_s"] > 0.0
    nodes = metrics["nodes"]
    assert nodes, "per-node 指标不能为空"
    names = [node["name"] for node in nodes]
    assert "OMPLMotionPlannerTask" in names
    for node in nodes:
        assert node["elapsed_s"] >= 0.0
        assert node["status"] in ("SUCCESS", "FAILED", "UNKNOWN")
        assert "message" in node


def test_node_timing_is_consistent_with_total(planner_ws, model):
    response = engine.plan(make_request(planner_ws), ws=planner_ws, model=model)
    metrics = response.metrics
    total = float(metrics["elapsed_total_s"])
    nodes = metrics["nodes"]
    # 厂商节点的 elapsed 是**嵌套**的（父节点含子节点耗时），因此
    # 「每个节点耗时 ≤ 总耗时」成立，而「各节点耗时之和 ≤ 总耗时」不成立；
    # 这里按嵌套语义断言：单节点 ≤ 总耗时 + 余量，总和 ≤ 总耗时 × 节点数 + 余量。
    for node in nodes:
        assert node["elapsed_s"] <= total + 0.5
    assert sum(node["elapsed_s"] for node in nodes) <= total * len(nodes) + 0.5


def test_verification_report_has_four_checks(planner_ws, model):
    response = engine.plan(make_request(planner_ws), ws=planner_ws, model=model)
    verification = response.verification
    for key in ("limits", "discrete_collision", "continuous_collision", "ok", "issues"):
        assert key in verification
    assert verification["ok"] is True
    assert verification["issues"] == []


# --------------------------------------------------------------------------
# 确定性
# --------------------------------------------------------------------------

def test_same_seed_same_digest_across_processes():
    status_a, digest_a = _run_in_subprocess(7)
    status_b, digest_b = _run_in_subprocess(7)
    assert status_a == status_b == "SUCCESS"
    assert digest_a == digest_b, "同请求同种子必须逐点一致"


def test_different_seed_still_plans():
    """不同种子只断言不崩（不做反向断言：不同种子可能给出同一条路径）。"""
    status_a, digest_a = _run_in_subprocess(7)
    status_b, digest_b = _run_in_subprocess(1234)
    assert status_a == status_b == "SUCCESS"
    assert digest_a and digest_b


def test_seed_is_set_before_planning(planner_ws, model, monkeypatch):
    """种子必须在任何规划之前设定（OMPL 开始采样后不可再改）。"""
    calls = []
    original = engine.apply_seed

    def spy(seed):
        calls.append(int(seed))
        return original(seed)

    monkeypatch.setattr(engine, "apply_seed", spy)
    response = engine.plan(
        make_request(planner_ws, seed=99), ws=planner_ws, model=model
    )
    assert response.status == "SUCCESS"
    assert calls == [99], "规划前必须先设定种子"


def test_seed_readback_in_fresh_process():
    """**新进程**里种子可读回（OMPL 只在第一次采样前接受新种子）。

    这条同时是服务端的硬前提：一个进程里第二次换种子不生效 → 必须
    **每请求独占一个 worker 子进程**。
    """
    code = (
        "import sys\n"
        "sys.path.insert(0, %r)\n"
        "from rmctl.planner_service import engine\n"
        "print('SEED:%%d' %% engine.apply_seed(4321))\n"
    ) % (PKG_SRC,)
    env = os.environ.copy()
    env["PYTHONPATH"] = os.pathsep.join(
        [p for p in (PKG_SRC, env.get("PYTHONPATH", "")) if p]
    )
    completed = subprocess.run(
        [sys.executable, "-c", code],
        cwd=TESTS_DIR,
        env=env,
        capture_output=True,
        text=True,
        timeout=300,
    )
    assert completed.returncode == 0, completed.stderr[-2000:]
    assert "SEED:4321" in completed.stdout, completed.stdout[-2000:]


# --------------------------------------------------------------------------
# 负例：限位 / IK / 非法请求 / 上限封顶
# --------------------------------------------------------------------------

def test_limit_violation_is_not_success(planner_ws, model):
    """超出 joint_limits 的轨迹 → 校验不过（不是 SUCCESS）。"""
    points = [[0.0] * 7, [4.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]
    times = [0.0, 1.0]
    healed = engine.verify_and_heal(model, points, times, max_scale=1.5)
    assert healed["report"]["ok"] is False
    assert healed["report"]["issues"], "校验不过必须给出问题列表"
    assert healed["scale"] is None


def test_out_of_range_goal_does_not_report_success(planner_ws, model):
    request = make_request(
        planner_ws, goal={"type": "joint", "values": [9.0] * 7}
    )
    response = engine.plan(request, ws=planner_ws, model=model)
    assert response.status != "SUCCESS"


def test_ik_failure_reports_ik(planner_ws, model):
    request = make_request(
        planner_ws, goal={"type": "pose", "values": [5.0, 0.0, 0.0, 0.0, 0.0, 0.0]}
    )
    response = engine.plan(request, ws=planner_ws, model=model)
    assert response.status == "FAILED"
    assert "IK" in response.failure_reason


def test_invalid_request_bubbles_up(planner_ws, model):
    with pytest.raises(InvalidRequest):
        engine.plan(None, ws=planner_ws, model=model)

    request = make_request(planner_ws)
    request.goal = {"type": "cartesian", "values": [0.0] * 6}
    with pytest.raises(InvalidRequest):
        engine.plan(request, ws=planner_ws, model=model)


def test_retime_keeps_geometry_and_stretches_times(planner_ws, model):
    """保路径降速：时间轴被拉长，**点位逐点不变**。"""
    points = [[0.0] * 7, [0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]
    times = [0.0, 0.015]
    healed = engine.verify_and_heal(model, points, times, max_scale=1.5)
    assert healed["report"]["ok"] is True
    assert healed["scale"] is not None and healed["scale"] > 1.001
    assert healed["points"] == points, "降速不得改变几何路径"
    assert healed["times"][-1] > times[-1], "降速必须拉长时间轴"


def test_retime_blocked_by_max_time_scaling(planner_ws, model):
    """需要的缩放系数超过 profiles.yaml 的上限 → 不降速，指名人工处理。"""
    points = [[0.0] * 7, [0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]
    times = [0.0, 0.001]
    healed = engine.verify_and_heal(model, points, times, max_scale=1.5)
    assert healed["scale"] is None
    assert healed["report"]["ok"] is False
    assert any("人工放宽" in note for note in healed["notes"]), healed["notes"]


# --------------------------------------------------------------------------
# 场景（增量命令）
# --------------------------------------------------------------------------

def test_scene_obstacle_is_added_incrementally(planner_ws, model):
    links_before = [link.getName() for link in model.env.getSceneGraph().getLinks()]
    request = make_request(
        planner_ws,
        scene={
            "obstacles": [
                {"name": "obs_far", "box_size": [0.1, 0.1, 0.1],
                 "position": [3.0, 3.0, 3.0]}
            ]
        },
    )
    response = engine.plan(request, ws=planner_ws, model=model)
    links_after = [link.getName() for link in model.env.getSceneGraph().getLinks()]
    assert "obs_far" in links_after
    assert len(links_after) == len(links_before) + 1, "只提交增量命令"
    assert set(links_before).issubset(set(links_after)), "不得替换既有 link"
    assert response.metrics["scene_links"] == ["obs_far"]


def test_blocking_obstacle_fails_naming_real_node(planner_ws, model):
    """障碍物堵住起点 → 规划失败，失败原因指名**真实失败节点**。"""
    request = make_request(
        planner_ws,
        scene={
            "obstacles": [
                {"name": "obs_block", "box_size": [0.4, 0.4, 0.4],
                 "position": [0.0, 0.0, 0.0]}
            ]
        },
    )
    response = engine.plan(request, ws=planner_ws, model=model)
    assert response.status == "FAILED"
    assert "OMPLMotionPlannerTask" in response.failure_reason
    assert "Error (Abort Triggered)" not in response.failure_reason


# --------------------------------------------------------------------------
# 零重复实现守卫 + 配置不可得
# --------------------------------------------------------------------------

def test_engine_uses_shared_verification_implementation(planner_ws, model, monkeypatch):
    """四道校验只有一个实现：engine 必须调 rmctl.verify.verify_with_retime。"""
    calls = []
    original = rmctl_verify.verify_with_retime

    def spy(*args, **kwargs):
        calls.append((args, kwargs))
        return original(*args, **kwargs)

    monkeypatch.setattr(engine, "verify_with_retime", spy)
    response = engine.plan(make_request(planner_ws), ws=planner_ws, model=model)
    assert response.status == "SUCCESS"
    assert calls, "engine 必须调用 rmctl.verify.verify_with_retime（唯一实现）"


def test_engine_source_has_no_second_limit_check():
    """engine.py 里不得出现第二份限位判定文案。"""
    with open(ENGINE_SOURCE, "r", encoding="utf-8") as handle:
        source = handle.read()
    assert "速度超限" not in source
    assert "加速度超限" not in source
    assert "jerk 超限" not in source
    # 唯一实现的判定文案仍在 verify.py 里
    with open(os.path.join(PKG_SRC, "rmctl", "verify.py"), "r", encoding="utf-8") as handle:
        verify_source = handle.read()
    assert "速度超限" in verify_source


def test_missing_planner_config_yields_failed_response(planner_ws, model, tmp_path):
    """配置不可得 → FAILED + 可读原因（不猜默认值、不抛裸异常）。"""
    empty_ws = tmp_path / "empty_ws"
    empty_ws.mkdir()
    response = engine.plan(
        make_request(planner_ws), ws=str(empty_ws), model=model
    )
    assert response.status == "FAILED"
    assert "配置不可用" in response.failure_reason
