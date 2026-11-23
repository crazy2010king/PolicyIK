"""S08/T06 —— R009 的性能口径变成**会失败的测试**（性能门禁 + 红线回归）。

这个文件把「报告里漂亮的口径」变成**可执行断言**：

**性能门禁**（口径与 ``docs/rm_planner规划服务说明.md`` §12 一致）：

1. ``success_only`` 的 **p95 ≤ 1.0 s**（只含 ``status == SUCCESS`` 的场景的
   e2e 墙钟；e2e 墙钟含 worker 冷启动）；
2. **worker 子进程峰值 RSS < 500 MB**（``/proc/<pid>/status`` 的 ``VmRSS``，
   10 ms 轮询 —— **不是**服务端进程 RSS）；
3. **成功率 ≥ 0.95**（``status == expect_status`` 的比例）。

**四道校验红线**（性能优化**不得**削弱校验语义 —— R002）：

4. ``limit_violation_jerk`` 场景在优化后**仍为 FAILED**（且失败原因确实是
   「四道校验未通过」—— 不是超时/规划器放弃）；降点数不得抹平 jerk 峰值；
5. ``joint_small_local`` 场景在优化后**仍为 SUCCESS**（且四道校验全过）；
6. 同请求同 seed 两次规划 → ``trajectory_digest`` 逐点一致（确定性不被
   性能优化破坏）；
7. **校验分辨率与点数无关**：同一条几何轨迹在两种点数下，四道校验结论一致
   （复用 T01 的 ``resample_trajectory``）。

测量逻辑**复用 T03 的脚本**（``scripts/rm_planner_perf.py``，用
``importlib`` 按路径加载，不重复实现分位数/RSS 采样/请求构造）：

* 模型在 **pytest 临时目录**里用 ``build_model.build_model()`` 物化
  （绝对路径 mesh）；规划配置从 git 跟踪的 ``workcell/planner/*.yaml`` 复制
  进临时工作区；
* 报告写进 pytest 临时目录（**不读** ``build/`` 等 gitignore 目录）。

夹具纪律：不依赖 gitignore 目录；不使用 psutil 等新依赖。
"""
from __future__ import annotations

import contextlib
import copy
import importlib.util
import os
import sys
import time

import pytest

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(TESTS_DIR, "..", "..", "..", ".."))
SCRIPT_PATH = os.path.join(REPO_ROOT, "scripts", "rm_planner_perf.py")

# --------------------------------------------------------------------------
# 门禁阈值（与 docs/rm_planner规划服务说明.md §12「性能档位与口径定义」一致）
# --------------------------------------------------------------------------

#: success_only 的 p95 上限（秒）—— 端到端墙钟（含 worker 冷启动）。
GATE_SUCCESS_ONLY_P95_S = 1.0

#: worker 子进程峰值 RSS 上限（MB）—— 规划内存的真正载体。
GATE_WORKER_PEAK_RSS_MB = 500.0

#: 成功率下限（status == expect_status 的比例）。
GATE_SUCCESS_RATE = 0.95

#: 稠密化步长（rad）—— 校验/稠密化用的关节空间最大步长。
STEP_RAD = 0.01


def _load_perf_module():
    """按路径加载 T03 的性能脚本模块（脚本不是包，用 importlib 加载）。"""
    spec = importlib.util.spec_from_file_location("rm_planner_perf_gate",
                                                  SCRIPT_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="session")
def perf_module():
    """T03 的性能脚本模块（分位数 / RSS 采样 / 请求构造的唯一实现）。"""
    return _load_perf_module()


# --------------------------------------------------------------------------
# 夹具：临时工作区 + 真实服务 + 一次全量场景跑
# --------------------------------------------------------------------------

class _PerfContext(object):
    """一次「优化后档位」全量跑的结果（供各条断言消费）。"""

    def __init__(self, perf, ws, client, rows, summary, report_path, scenarios,
                 pipeline):
        self.perf = perf
        self.ws = ws
        self.client = client
        self.rows = rows
        self.summary = summary
        self.report_path = report_path
        self.scenarios = scenarios
        self.pipeline = pipeline

    def row(self, sid):
        """按场景 id 取那一行（缺失 → 明确失败，不静默跳过）。"""
        by_id = {row["id"]: row for row in self.rows}
        assert sid in by_id, (
            "场景集里没有 %s（现有: %s）" % (sid, sorted(by_id)))
        return by_id[sid]

    def scenario(self, sid):
        by_id = {item["id"]: item for item in self.scenarios}
        assert sid in by_id, "场景集里没有 %s" % sid
        return by_id[sid]


@pytest.fixture(scope="session")
def perf_ctx(perf_module, tmp_path_factory):
    """物化工作区 → 起真实服务 → 跑一遍黄金场景集（optimized 档）。"""
    root = tmp_path_factory.mktemp("s08_perf_gate")
    ws = perf_module._materialize_workspace(str(root))
    sock_path = os.path.join(str(root), "rm_planner.sock")
    server = perf_module.PlannerServer(socket_path=sock_path, ws=ws,
                                      worker_pool_size=1)
    server.start()
    client = perf_module.PlannerClient(socket_path=sock_path, ws=ws)
    sampler = perf_module.RssSampler(server)
    sampler.start()
    try:
        scenarios, _meta = perf_module._load_scenarios(False)
        pipeline = perf_module._read_repo_pipeline()
        rows = perf_module.run_profile("optimized", pipeline, ws, scenarios,
                                       client, sampler)
    finally:
        sampler.stop()
    summary = perf_module.summarize(rows)
    report_path = os.path.join(str(root), "s08_perf_gate.json")
    perf_module._write_json(report_path, {
        "version": 1,
        "source": "S08/T06 性能门禁（tests/test_planner_perf.py）",
        "profile": "optimized",
        "pipeline": perf_module._profile_params(pipeline),
        "results": rows,
        "summary": summary,
    })
    ctx = _PerfContext(perf_module, ws, client, rows, summary, report_path,
                       scenarios, pipeline)
    try:
        yield ctx
    finally:
        server.stop()


@pytest.fixture(scope="session")
def model(perf_ctx):
    """规划模型（碰撞查询用；每个进程一份即可）。"""
    from rmctl.model import Model

    return Model(ws=perf_ctx.ws)


# --------------------------------------------------------------------------
# 辅助：临时改写工作区档位（引擎每条请求都重载配置 → 无需重启服务）
# --------------------------------------------------------------------------

@contextlib.contextmanager
def _raw_output_profile(perf_ctx):
    """临时把「下发稠密化」关掉 → 下发的是规划器**原始点**。

    只改**临时工作区**里的 ``workcell/planner/pipeline.yaml``（引擎每条请求
    都重新 ``load_pipeline_config()`` → 换档无需重启服务），退出时恢复原值。
    """
    original = dict(perf_ctx.pipeline)
    patched = dict(original)
    patched["output_resample_step_rad"] = 0.0
    perf_ctx.perf._write_pipeline(perf_ctx.ws, patched)
    try:
        yield
    finally:
        perf_ctx.perf._write_pipeline(perf_ctx.ws, original)


# --------------------------------------------------------------------------
# 门禁 1：success_only 的 p95 ≤ 1.0 s
# --------------------------------------------------------------------------

def test_gate_success_only_p95_within_1s(perf_ctx):
    """端到端延迟门禁：只统计 SUCCESS 场景的 e2e 墙钟 p95。"""
    success = perf_ctx.summary.get("success_only") or {}
    p95 = success.get("p95_s")
    p50 = success.get("p50_s")
    count = success.get("count")
    peak_rss = perf_ctx.summary.get("peak_rss_mb_max")
    points = perf_ctx.summary.get("points_median")
    assert p95 is not None, (
        "没有 SUCCESS 场景，无法判定延迟门禁；summary=%s" % perf_ctx.summary)
    assert count >= 1, perf_ctx.summary
    assert p95 <= GATE_SUCCESS_ONLY_P95_S, (
        "success_only p95 = %.3f s > %.1f s（p50=%.3f s，场景数=%d，"
        "worker 峰值 RSS=%s MB，点数中位数=%s）"
        % (p95, GATE_SUCCESS_ONLY_P95_S, p50, count, peak_rss, points))


# --------------------------------------------------------------------------
# 门禁 2：worker 子进程峰值 RSS < 500 MB
# --------------------------------------------------------------------------

def test_gate_worker_peak_rss_below_500mb(perf_ctx):
    """内存门禁：量的是 **worker 子进程**峰值 RSS（不是服务端进程）。"""
    peak = perf_ctx.summary.get("peak_rss_mb_max")
    assert peak is not None, (
        "没有采到任何 worker RSS（采样失效 → 不能当作通过）：summary=%s"
        % perf_ctx.summary)
    assert peak < GATE_WORKER_PEAK_RSS_MB, (
        "worker 峰值 RSS = %.2f MB ≥ %.0f MB（success_only p95=%s s，"
        "点数中位数=%s）"
        % (peak, GATE_WORKER_PEAK_RSS_MB,
           (perf_ctx.summary.get("success_only") or {}).get("p95_s"),
           perf_ctx.summary.get("points_median")))


def test_gate_rss_is_measured_on_worker_not_on_server_process(perf_ctx):
    """口径自查：被量的是 **worker 子进程**，不是服务端本体。

    判据：采样到的 pid **不等于**服务端进程自身的 pid（否则量的是服务端
    RSS —— 十几 MB 的平凡通过），且采样值是个真实的进程占用。
    """
    server_pid = os.getpid()
    rows = [row for row in perf_ctx.rows
            if row.get("worker_peak_rss_mb") is not None]
    assert rows, "没有任何场景采到 worker RSS"
    for row in rows:
        assert row["worker_pids"], (
            "场景 %s 有 RSS 采样却没有 worker pid（口径错了？）" % row["id"])
        assert server_pid not in row["worker_pids"], (
            "场景 %s 把服务端进程本身当成了 worker（口径错了）" % row["id"])
        assert row["worker_peak_rss_mb"] > 0.0
    # 采样值是真实占用（不是 0 / 不是 None 当 0）
    assert max(row["worker_peak_rss_mb"] for row in rows) > 20.0, (
        "worker 峰值 RSS 小得不像真的规划进程：%s"
        % [row["worker_peak_rss_mb"] for row in rows])


# --------------------------------------------------------------------------
# 门禁 3：成功率 ≥ 0.95
# --------------------------------------------------------------------------

def test_gate_success_rate_at_least_95_percent(perf_ctx):
    """成功率门禁：``status == expect_status`` 的比例。"""
    rate = perf_ctx.summary.get("success_rate")
    assert rate is not None
    assert rate >= GATE_SUCCESS_RATE, (
        "成功率 = %.4f < %.2f（status 不匹配的场景: %s）"
        % (rate, GATE_SUCCESS_RATE,
           [(row["id"], row["status"], row["expect_status"])
            for row in perf_ctx.rows if not row["status_match"]]))


def test_gate_report_is_written_to_tmp_dir(perf_ctx):
    """门禁跑出来的报告落在 pytest 临时目录（不写 build/ 等 gitignore 目录）。"""
    assert os.path.isfile(perf_ctx.report_path), perf_ctx.report_path
    assert perf_ctx.report_path.startswith(
        os.path.realpath(os.sep + "tmp") + os.sep) or \
        "tmp" in perf_ctx.report_path, perf_ctx.report_path
    import json
    with open(perf_ctx.report_path, "r", encoding="utf-8") as fh:
        report = json.load(fh)
    assert report["summary"]["success_only"]["p95_s"] is not None
    assert report["summary"]["peak_rss_mb_max"] is not None


# --------------------------------------------------------------------------
# 红线 1：limit_violation_jerk 仍然 FAILED（降点数不得抹平 jerk 峰值）
# --------------------------------------------------------------------------

def test_redline_jerk_violation_still_failed_after_optimization(perf_ctx):
    """``limit_violation_jerk``：优化后**仍为 FAILED**，且失败原因是校验未过。"""
    row = perf_ctx.row("limit_violation_jerk")
    assert row["status"] == "FAILED", (
        "limit_violation_jerk: status=%s，期望 FAILED（降点数把 jerk 峰值"
        "抹平了？）；failure_reason=%r"
        % (row["status"], row["failure_reason"]))
    assert row["verify_ok"] is False, (
        "limit_violation_jerk: verification.ok=%r，期望 False"
        % (row["verify_ok"],))
    assert "四道校验未通过" in row["failure_reason"], (
        "limit_violation_jerk 的失败原因应指名「四道校验未通过」，实际 %r"
        % (row["failure_reason"],))


# --------------------------------------------------------------------------
# 红线 2：joint_small_local 仍然 SUCCESS
# --------------------------------------------------------------------------

def test_redline_small_local_still_success_after_optimization(perf_ctx):
    """``joint_small_local``：优化后**仍为 SUCCESS** 且四道校验全过。"""
    row = perf_ctx.row("joint_small_local")
    assert row["status"] == "SUCCESS", (
        "joint_small_local: status=%s，期望 SUCCESS；failure_reason=%r"
        % (row["status"], row["failure_reason"]))
    assert row["verify_ok"] is True, (
        "joint_small_local: verification.ok=%r，期望 True" % (row["verify_ok"],))


# --------------------------------------------------------------------------
# 红线 3：确定性（同请求同 seed → 逐点一致）
# --------------------------------------------------------------------------

def test_redline_determinism_survives_optimization(perf_ctx):
    """同请求同 seed 两次规划 → ``trajectory_digest`` 逐点一致。"""
    from rmctl.planner_service.contract import trajectory_digest

    scenario = perf_ctx.scenario("joint_small_local")
    request = perf_ctx.perf.build_request(scenario, "perf-gate-det-a")
    first = perf_ctx.client.plan(request)
    second = perf_ctx.client.plan(
        perf_ctx.perf.build_request(scenario, "perf-gate-det-b"))
    assert first["status"] == "SUCCESS", first.get("failure_reason")
    assert second["status"] == "SUCCESS", second.get("failure_reason")
    assert trajectory_digest(first["trajectory"]) == \
        trajectory_digest(second["trajectory"]), "同种子两次结果不一致"


# --------------------------------------------------------------------------
# 红线 4：校验分辨率与点数无关
# --------------------------------------------------------------------------

def test_redline_verification_conclusion_is_density_independent(perf_ctx, model):
    """同一条几何轨迹，两种点数 → 四道校验结论一致。

    取**同一次规划**的原始点（把临时工作区的 ``output_resample_step_rad``
    置 0 → 下发的就是规划器原始点），再按 0.01 rad 稠密化一次 → 对照两条
    轨迹的四道校验结论：必须一致（点数变化不改变判定 → 性能优化不会削弱
    校验语义）。

    为什么不在「已稠密的输出轨迹」上再稠密一次：那样插值节点变了，Hermite
    曲线会轻微变形，比较的就不再是**同一条几何轨迹**了。
    """
    from rmctl.verify import resample_trajectory, verify

    scenario = perf_ctx.scenario("joint_small_local")
    with _raw_output_profile(perf_ctx):
        response = perf_ctx.client.plan(
            perf_ctx.perf.build_request(scenario, "perf-gate-density"))
    assert response["status"] == "SUCCESS", response.get("failure_reason")
    trajectory = response["trajectory"]
    raw_points = [list(item["positions"]) for item in trajectory["points"]]
    raw_times = [float(t) for t in trajectory["times"]]

    dense_points, dense_times = resample_trajectory(raw_points, raw_times,
                                                    STEP_RAD)
    assert len(dense_points) > len(raw_points), (
        "稠密化必须真的变密（原 %d 点 → %d 点）" % (len(raw_points),
                                                  len(dense_points)))

    coarse_report = verify(model, raw_points, raw_times)
    dense_report = verify(model, dense_points, dense_times)
    assert coarse_report["ok"] is True, coarse_report["issues"]
    assert dense_report["ok"] is True, dense_report["issues"]
    for name in ("limits", "discrete_collision", "continuous_collision"):
        assert coarse_report["checks"][name]["ok"] == \
            dense_report["checks"][name]["ok"], (
                "四道校验的 %s 结论随点数变化了（原点数 %d / 稠密点数 %d）"
                % (name, len(raw_points), len(dense_points)))


def test_redline_density_check_is_not_vacuous(model):
    """**负向对照**：少点轨迹的粗校验通过，但稠密化后必须判不过。

    没有这一条，上一条「两种点数结论一致」可能是**平凡成立**的（例如两边
    都因为数据缺失而跳过校验）。这里构造一条「少点抹平 jerk 峰值」的轨迹：
    粗校验通过、稠密校验不过 —— 证明稠密化确实在提高校验分辨率。

    2026-11-23（S13）起 joint_limits.yaml 的 acceleration 已按真机实测收紧
    （1.48~2.44 rad/s^2）→ 这条固定轨迹会在**粗校验**阶段就撞上 acceleration
    上限，前置条件不再成立 —— 变的是限值口径，不是分辨率机制。故把模型限值
    **钉死**在回填前的占位值（不改判据、不放宽任何真实限值）。
    """
    from rmctl.verify import check_limits, resample_trajectory

    pinned = copy.copy(model)
    pinned.limits = {joint: {"acceleration": 5.2359, "jerk": 30.0, "velocity": 3.1416}
                     for joint in model.joint_names}
    model = pinned

    times = [0.0, 0.3488, 0.5544, 0.8298, 1.0]
    values = [-0.7242, -0.5601, -0.5130, -0.3139, -0.2105]
    points = []
    for value in values:
        point = [0.0] * 7
        point[0] = float(value)
        points.append(point)

    coarse_issues, _ = check_limits(model, points, times)
    assert coarse_issues == [], (
        "前置条件：少点轨迹的粗校验应当通过（峰值被抹平）：%s" % coarse_issues)
    dense_points, dense_times = resample_trajectory(points, times, STEP_RAD)
    dense_issues, _ = check_limits(model, dense_points, dense_times)
    assert dense_issues, "稠密化后必须仍然判 limits 未通过"
    assert any("jerk 超限" in issue for issue in dense_issues), dense_issues


# --------------------------------------------------------------------------
# 门禁口径本身的负向校验（阈值不是空话）
# --------------------------------------------------------------------------

def test_gate_arithmetic_rejects_bad_rows(perf_module):
    """门禁算术的**负向**校验：造出「不达标」的行 → 门禁必须会红。

    证明三条门禁不是恒真的装饰：p95 超阈 / 峰值 RSS 超阈 / status 不匹配
    三种情况都必须被各自的断言抓到。
    """
    def _row(sid, status, expect, wall, rss):
        return {
            "profile": "optimized", "id": sid, "status": status,
            "expect_status": expect, "status_match": status == expect,
            "e2e_wall_s": wall, "planning_s": wall, "points": 30,
            "worker_peak_rss_mb": rss, "observed_profile": {},
        }

    slow = perf_module.summarize([_row("a", "SUCCESS", "SUCCESS", 2.0, 100.0)])
    assert slow["success_only"]["p95_s"] > GATE_SUCCESS_ONLY_P95_S
    assert slow["success_rate"] >= GATE_SUCCESS_RATE

    fat = perf_module.summarize([_row("a", "SUCCESS", "SUCCESS", 0.2, 900.0)])
    assert fat["peak_rss_mb_max"] > GATE_WORKER_PEAK_RSS_MB

    broken = perf_module.summarize([_row("a", "SUCCESS", "FAILED", 0.2, 100.0)])
    assert broken["success_rate"] < GATE_SUCCESS_RATE


# --------------------------------------------------------------------------
# 测量口径的可观测性（供定位问题）
# --------------------------------------------------------------------------

def test_gate_rows_expose_measurement_fields(perf_ctx):
    """每行都带齐定位所需的量：墙钟 / 纯规划耗时 / 点数三兄弟 / 峰值 RSS。"""
    for row in perf_ctx.rows:
        assert row["e2e_wall_s"] >= 0.0, row
        assert "planning_s" in row and "points" in row, row
        assert "trajectory_points" in row, row
        assert "verified_points" in row, row
        assert "output_points" in row, row
        assert "worker_peak_rss_mb" in row, row
        assert row["status"], row
        assert row["expect_status"], row


def test_gate_uses_current_repo_pipeline_profile(perf_ctx):
    """门禁量的是**仓库当前**档位（不是脚本里的冻结基线）。"""
    assert perf_ctx.pipeline["planner_range"] == \
        perf_ctx.perf._read_repo_pipeline()["planner_range"]
    for row in perf_ctx.rows:
        observed = row.get("observed_profile") or {}
        if observed.get("planner_range") is None:
            continue  # 失败/超时场景没有档位 metrics
        assert abs(float(observed["planner_range"]) -
                   float(perf_ctx.pipeline["planner_range"])) < 1e-9, (
            "场景 %s 实际跑的不是仓库档位（planner_range=%r）"
            % (row["id"], observed["planner_range"]))


def test_gate_run_is_reproducible_enough_for_ci(perf_ctx):
    """门禁本身要在 CI 可跑：整轮耗时有限、场景集完整。"""
    assert len(perf_ctx.rows) == len(perf_ctx.scenarios) >= 6
    started = time.time()
    assert started > 0.0
    ids = {row["id"] for row in perf_ctx.rows}
    assert "limit_violation_jerk" in ids
    assert "joint_small_local" in ids
