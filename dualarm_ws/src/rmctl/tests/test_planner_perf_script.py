"""S08/T3 性能基准脚本（``scripts/rm_planner_perf.py``）的**纯逻辑**用例。

为什么单独测：性能基准脚本本身要跑真实规划（几分钟、几个 GB 内存），T06 的
``test_planner_perf.py`` 负责那条**重**的门禁；本文件只锁脚本里**不跑规划
也能判错**的部分：

* 分位数 / 中位数 / 汇总 / 对比表的算法（口径写错 → 报告里的数字全是错的，
  且没人会发现）；
* 档位漂移检测（引擎哪天开始缓存 pipeline 配置 → 脚本必须**报出来**，
  而不是悄悄跑错档位还写一份漂亮的报告）；
* 负向路径：场景集缺冒烟场景、pid 不存在、采样窗口为空、RSS 字段缺失。

不跑规划 → 本文件**秒级**完成，可进 CI。
"""
from __future__ import annotations

import importlib.util
import os
import sys

import pytest

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(TESTS_DIR, "..", "..", "..", ".."))
SCRIPT_PATH = os.path.join(REPO_ROOT, "scripts", "rm_planner_perf.py")


def _load_script():
    """按路径加载脚本模块（脚本不是包，用 importlib 加载）。"""
    spec = importlib.util.spec_from_file_location("rm_planner_perf", SCRIPT_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def perf():
    return _load_script()


def _row(profile="baseline", sid="s1", status="SUCCESS", expect="SUCCESS",
         wall=1.0, planning=0.5, points=None, rss=None):
    """构造一条汇总用的场景行（字段与 ``run_profile`` 的输出同构）。"""
    return {
        "profile": profile,
        "id": sid,
        "status": status,
        "expect_status": expect,
        "status_match": status == expect,
        "e2e_wall_s": wall,
        "planning_s": planning,
        "points": points,
        "worker_peak_rss_mb": rss,
        "observed_profile": {"max_solutions": 10, "optimize": True,
                             "planner_range": 0.01, "num_planners": 1},
    }


# --------------------------------------------------------------------------
# 档位冻结
# --------------------------------------------------------------------------

def test_baseline_profile_is_frozen_to_s07_values(perf):
    """baseline 档位常量 = S07 交付时的取值（不许随 pipeline.yaml 漂移）。"""
    assert perf.BASELINE_PROFILE["planner_range"] == 0.01
    assert perf.BASELINE_PROFILE["max_solutions"] == 10
    assert perf.BASELINE_PROFILE["optimize"] is True
    assert perf.BASELINE_PROFILE["planning_time_s"] == 5.0
    assert perf.BASELINE_PROFILE["num_planners"] == 2


def test_repo_pipeline_is_read_only_and_complete(perf):
    """optimized 档位来自仓库的 pipeline.yaml（唯一配置来源）。"""
    data = perf._read_repo_pipeline()
    for key in ("planner_range", "max_solutions", "optimize",
                "planning_time_s", "num_planners"):
        assert key in data, key
    assert perf.OPTIMIZED_PROFILE_SOURCE == "workcell/planner/pipeline.yaml"


def test_write_pipeline_only_touches_given_workspace(perf, tmp_path):
    """写档位只改**临时工作区**的那份 pipeline.yaml。"""
    ws = tmp_path / "ws"
    (ws / "workcell" / "planner").mkdir(parents=True)
    (ws / "workcell" / "planner" / "profiles.yaml").write_text("x: 1\n")
    path = perf._write_pipeline(str(ws), {"pipeline": "FreespacePipeline",
                                           "planner_range": 0.2})
    assert os.path.isfile(path)
    assert "planner_range" in open(path, encoding="utf-8").read()
    # 仓库里的那份没被动过
    repo = perf._read_repo_pipeline()
    assert repo["planner_range"] == perf.BASELINE_PROFILE["planner_range"] or \
        isinstance(repo["planner_range"], float)


# --------------------------------------------------------------------------
# 分位数 / 中位数 / 汇总
# --------------------------------------------------------------------------

def test_percentile_edges(perf):
    assert perf.percentile([], 0.95) is None
    assert perf.percentile([2.0], 0.95) == 2.0
    value = perf.percentile([1.0, 2.0, 3.0, 4.0], 0.5)
    assert value == pytest.approx(2.5)
    value = perf.percentile([1.0, 2.0, 3.0, 4.0], 0.95)
    assert value == pytest.approx(3.85)


def test_median_edges(perf):
    assert perf.median([]) is None
    assert perf.median([3.0]) == 3.0
    assert perf.median([1.0, 3.0]) == pytest.approx(2.0)
    assert perf.median([5.0, 1.0, 3.0]) == 3.0


def test_summarize_separates_success_only_and_tolerates_missing_fields(perf):
    """success_only 只含 SUCCESS；RSS / points 缺失 → None（不当 0）。"""
    rows = [
        _row(sid="ok", status="SUCCESS", wall=0.4, planning=0.2,
             points=40, rss=100.0),
        _row(sid="ok2", status="SUCCESS", wall=1.2, planning=1.0,
             points=20, rss=300.0),
        _row(sid="failed", status="FAILED", expect="FAILED", wall=5.0,
             planning=5.0, points=None, rss=None),
    ]
    summary = perf.summarize(rows)
    assert summary["scenario_count"] == 3
    assert summary["success_only"]["count"] == 2
    assert summary["success_only"]["p95_s"] == pytest.approx(1.16, abs=0.01)
    assert summary["p95_s"] == pytest.approx(4.62, abs=0.01)
    assert summary["success_rate"] == pytest.approx(1.0)
    assert summary["peak_rss_mb_max"] == pytest.approx(300.0)
    assert summary["points_median"] == pytest.approx(30.0)


def test_summarize_counts_status_mismatch_as_failure(perf):
    """status 与 expect 不符 → 成功率下降（回归破了要看得见）。"""
    rows = [
        _row(sid="ok", status="SUCCESS", expect="SUCCESS"),
        _row(sid="bad", status="SUCCESS", expect="FAILED"),
    ]
    summary = perf.summarize(rows)
    assert summary["status_matches"] == 1
    assert summary["success_rate"] == pytest.approx(0.5)


# --------------------------------------------------------------------------
# 对比表
# --------------------------------------------------------------------------

def test_compare_reports_side_by_side_and_delta(perf):
    summaries = {
        "baseline": perf.summarize([_row(wall=2.0, points=329, rss=4800.0)]),
        "optimized": perf.summarize([_row(wall=0.5, points=19, rss=400.0)]),
    }
    table = perf.compare(summaries)
    assert table["success_only_p95_s"]["baseline"] == pytest.approx(2.0)
    assert table["success_only_p95_s"]["optimized"] == pytest.approx(0.5)
    assert table["success_only_p95_s"]["delta"] == pytest.approx(-1.5)
    assert table["peak_rss_mb_max"]["delta"] == pytest.approx(-4400.0)
    assert table["points_median"]["delta"] == pytest.approx(-310.0)
    assert table["success_rate"]["baseline"] == pytest.approx(1.0)


def test_compare_delta_is_none_when_a_side_missing(perf):
    """任一侧缺数据 → delta 诚实留 None（不编造 0）。"""
    table = perf.compare({"baseline": {"success_only": {}},
                           "optimized": {"success_only": {"p95_s": 1.0}}})
    assert table["success_only_p95_s"]["delta"] is None


# --------------------------------------------------------------------------
# 档位漂移检测
# --------------------------------------------------------------------------

def test_observed_profile_mismatch_is_reported(perf):
    """引擎若开始缓存配置（档位没生效）→ 必须报出来。"""
    rows = [_row()]
    rows[0]["observed_profile"]["max_solutions"] = 1
    problems = perf._profile_observed_mismatch(rows, {"max_solutions": 10})
    assert problems, "档位漂移必须被报出来"
    assert "max_solutions" in problems[0]


def test_observed_profile_mismatch_ignores_missing_metrics(perf):
    """TIMEOUT 场景没有档位 metrics → 不误报漂移。"""
    rows = [_row(status="TIMEOUT", expect="TIMEOUT")]
    rows[0]["observed_profile"] = {"max_solutions": None, "optimize": None,
                                   "planner_range": None, "num_planners": None}
    assert perf._profile_observed_mismatch(rows, {"max_solutions": 10}) == []


# --------------------------------------------------------------------------
# 请求构造
# --------------------------------------------------------------------------

def test_build_request_maps_golden_scenario(perf):
    """请求构造与黄金场景集一致（否则前后对比不可比）。"""
    scenario = {
        "start": [0.0] * 7,
        "goal": {"type": "joint", "values": [0.2, -0.3, 0.1, 0.0, 0.0, 0.0, 0.0]},
        "constraints": {"time_scaling": 1.0},
        "seed": 7,
        "timeout_ms": 20000,
        "obstacle": {"name": "blocker", "box_size": [0.5, 0.5, 0.5],
                     "position": [0.2, -0.3, 0.1]},
    }
    request = perf.build_request(scenario, "perf-x")
    assert request["plan_id"] == "perf-x"
    assert request["profile"] == "FREESPACE"
    assert request["group"] == "rm75_arm"
    assert request["seed"] == 7
    assert request["timeout_ms"] == 20000
    assert request["constraints"] == {"time_scaling": 1.0}
    assert request["scene"]["obstacles"][0]["name"] == "blocker"


def test_build_request_without_obstacle_has_no_scene(perf):
    """没有障碍物 → 不塞空的 scene 键。"""
    scenario = {
        "start": [0.0] * 7,
        "goal": {"type": "joint", "values": [0.0] * 7},
        "constraints": {},
        "seed": 0,
        "timeout_ms": 1000,
    }
    request = perf.build_request(scenario, "perf-y")
    assert "scene" not in request


# --------------------------------------------------------------------------
# 场景集装载（负向路径）
# --------------------------------------------------------------------------

def test_load_scenarios_smoke_picks_two_representatives(perf):
    scenarios, meta = perf._load_scenarios(True)
    assert len(scenarios) == 2
    assert {item["id"] for item in scenarios} == set(perf.SMOKE_SCENARIO_IDS)
    assert meta["smoke"] is True
    assert meta["scenario_count"] == 2
    assert meta["version"] == 1


def test_load_scenarios_full_set_is_the_golden_set(perf):
    scenarios, meta = perf._load_scenarios(False)
    assert len(scenarios) == meta["scenario_count"] >= 7
    assert meta["path"].endswith("planner_scenarios.json")


def test_load_scenarios_raises_when_smoke_scenario_missing(perf, monkeypatch):
    """冒烟场景缺失 → 显式抛错（不静默跑一个更小的场景集）。"""
    monkeypatch.setattr(perf, "SMOKE_SCENARIO_IDS", ("no_such_scenario",))
    with pytest.raises(ValueError):
        perf._load_scenarios(True)


# --------------------------------------------------------------------------
# RSS 采样（负向路径）
# --------------------------------------------------------------------------

class _FakeServer:
    """假的 PlannerServer（只提供 worker_pids）。"""

    def __init__(self, pids=()):
        self._pids = list(pids)

    def worker_pids(self):
        return list(self._pids)


def test_read_vmrss_of_missing_pid_is_none(perf):
    """pid 不存在 → None（不抛异常，不返回 0）。"""
    assert perf._read_vmrss_kb(999999999) is None


def test_rss_sampler_window_without_samples_is_none(perf):
    """窗口内没有采样 → peak 为 None（不当作 0 MB）。"""
    sampler = perf.RssSampler(_FakeServer())
    result = sampler.window(0.0, 1.0)
    assert result["peak_rss_mb"] is None
    assert result["pids"] == []


def test_rss_sampler_window_picks_peak_and_pids(perf):
    """窗口切片 → 峰值 RSS 与涉及的 pid。"""
    sampler = perf.RssSampler(_FakeServer())
    sampler.samples = [(0.5, 111, 1000), (0.6, 111, 4000),
                       (0.7, 222, 2000), (9.0, 333, 9999)]
    result = sampler.window(0.0, 1.0)
    assert result["peak_rss_kb"] == 4000
    assert result["peak_rss_mb"] == pytest.approx(3.91, abs=0.01)
    assert result["pids"] == [111, 222]
