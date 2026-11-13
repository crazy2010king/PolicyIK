"""故障注入 + 样本轨迹生成器测试（T8：离线证据的可复跑输入）。

覆盖：

* ``rm_executor.injection.parse_injection`` 的正负例（非法规格必须被拒）；
* ``rm_executor.sample`` 生成的轨迹**确定性**、过契约、过时间域限值；
* 三类注入的**端到端**行为：事件类型、严重度、安全动作（pause / stop）、
  报告 status 与退出码；
* 真机（live）路径拒绝任何注入。
"""
from __future__ import annotations

import json
import os

import pytest

from rm_executor import cli as cli_mod
from rm_executor import contract
from rm_executor import injection as inj_mod
from rm_executor import sample as sample_mod
from rm_executor.clock import ManualClock
from rm_executor.executor import Executor
from rm_executor.transport import FakeTransport


# ------------------------------------------------------------------ 样本轨迹
def test_sample_trajectory_is_deterministic(tmp_path):
    """同样参数 → 同样字节（证据可复跑）。"""
    a = sample_mod.build_sample_trajectory()
    b = sample_mod.build_sample_trajectory()
    assert json.dumps(a, sort_keys=True) == json.dumps(b, sort_keys=True)
    p1 = sample_mod.write_sample_trajectory(str(tmp_path / "t1.json"))
    p2 = sample_mod.write_sample_trajectory(str(tmp_path / "t2.json"))
    with open(p1, encoding="utf-8") as fh:
        text1 = fh.read()
    with open(p2, encoding="utf-8") as fh:
        text2 = fh.read()
    assert text1 == text2


def test_sample_trajectory_passes_contract(tmp_path):
    """样本轨迹满足轨迹契约（含 verify）。"""
    path = sample_mod.write_sample_trajectory(str(tmp_path / "traj.json"))
    res = contract.load_trajectory(path)
    assert res["status"] == "ok", res.get("reason")
    assert res["schema_inferred"] is False


def test_sample_trajectory_passes_time_domain_limits(tmp_path):
    """样本轨迹通过时间域限值复检（位置/速度/加速度/jerk）。"""
    path = sample_mod.write_sample_trajectory(str(tmp_path / "traj.json"))
    payload = json.loads(open(path, encoding="utf-8").read())
    res = contract.check_time_domain_limits(payload["times"], payload["points"])
    assert res["status"] == "ok", res.get("reason")
    assert not res.get("violations")


def test_sample_trajectory_rejects_bad_args(tmp_path):
    """非法参数 → 明确报错（不静默）。"""
    with pytest.raises(ValueError):
        sample_mod.build_sample_trajectory(period_s=0.0)
    with pytest.raises(ValueError):
        sample_mod.build_sample_trajectory(n_points=1)


# -------------------------------------------------------------- 注入规格解析
def test_parse_injection_accepts_three_kinds():
    specs = [inj_mod.parse_injection(s) for s in
             ("follow_error:6", "feedback_stall:250", "controller_conflict")]
    assert [s["kind"] for s in specs] == list(inj_mod.INJECTION_KINDS)
    assert specs[0]["value"] == 6.0
    assert specs[1]["value"] == 250.0
    assert specs[2]["value"] is None


@pytest.mark.parametrize("spec", [
    "",                      # 空规格
    "follow_error",          # 缺参数
    "follow_error:abc",      # 非数值
    "follow_error:0",        # 必须 > 0
    "follow_error:1",        # 未超过停机阈值 → 不会触发判据
    "feedback_stall:100",    # 未超过断流停机阈值 → 不会触发判据
    "controller_conflict:1", # 不接受参数
    "unknown:1",             # 未知种类
])
def test_parse_injection_rejects_bad_specs(spec):
    """非法注入规格必须被拒（带可粘贴的下一步）。"""
    with pytest.raises(ValueError) as exc:
        inj_mod.parse_injection(spec)
    assert str(exc.value)


# -------------------------------------------------------------- 端到端注入
def _run_with_injection(traj_path, spec_text, ws=None):
    """跑一次离线假执行（带注入）；返回 ``(report, transport)``。"""
    specs = [inj_mod.parse_injection(spec_text)]
    transport = FakeTransport()
    feedback = cli_mod._mirror_feedback(transport, cli_mod.DEFAULT_PERIOD_S)
    transport, feedback = inj_mod.apply_injections(
        transport, feedback, specs, period_s=cli_mod.DEFAULT_PERIOD_S)
    executor = Executor(transport, feedback,
                        clock=ManualClock(period_s=cli_mod.DEFAULT_PERIOD_S),
                        mode="fake", ws=ws)
    report = executor.run(traj_path)
    return report, transport


def test_inject_follow_error_pauses_and_blocks(tmp_path):
    """跟随误差 6° → follow_error_exceeded（NEEDS_HUMAN）+ pause → blocked/3。"""
    path = sample_mod.write_sample_trajectory(str(tmp_path / "traj.json"))
    report, transport = _run_with_injection(path, "follow_error:6")
    types = [(e["type"], e["severity"]) for e in report["events"]]
    assert ("follow_error_exceeded", "NEEDS_HUMAN") in types
    assert report["status"] == "blocked"
    assert contract.EXIT_CODES[report["status"]] == 3
    assert transport.count("pause") == 1
    assert transport.count("stop") == 0  # 未到硬红线，不应停机


def test_inject_feedback_stall_stops_and_errors(tmp_path):
    """反馈断流 250 ms → feedback_stream_lost（FATAL）+ stop → error/4。"""
    path = sample_mod.write_sample_trajectory(str(tmp_path / "traj.json"))
    report, transport = _run_with_injection(path, "feedback_stall:250")
    types = [(e["type"], e["severity"]) for e in report["events"]]
    assert ("feedback_stream_lost", "FATAL") in types
    assert report["status"] == "error"
    assert contract.EXIT_CODES[report["status"]] == 4
    assert transport.count("stop") >= 1
    lost = [e for e in report["events"] if e["type"] == "feedback_stream_lost"]
    assert lost, report["events"]
    # 断流判据要写明实测 age 与阈值
    joined = " ".join(lost[0]["evidence"])
    assert "stream_age_ms" in joined and "stream_stop_ms" in joined


def test_inject_controller_conflict_stops_immediately(tmp_path):
    """控制器抢轨迹 → controller_trajectory_conflict（FATAL）+ stop → error/4。"""
    path = sample_mod.write_sample_trajectory(str(tmp_path / "traj.json"))
    report, transport = _run_with_injection(path, "controller_conflict")
    types = [(e["type"], e["severity"]) for e in report["events"]]
    assert ("controller_trajectory_conflict", "FATAL") in types
    assert report["status"] == "error"
    assert contract.EXIT_CODES[report["status"]] == 4
    assert transport.count("stop") >= 1
    assert transport.count("current_trajectory_type") >= 1


def test_cli_rejects_injection_in_live_mode(tmp_path):
    """真机路径拒绝故障注入（不触达控制器）。"""
    path = sample_mod.write_sample_trajectory(str(tmp_path / "traj.json"))
    report = cli_mod.execute(mode="live", trajectory_path=path,
                             injections=["follow_error:6"])
    assert report["status"] == "blocked"
    assert report["reason"]
    assert report["next_action"]
    assert not report["metrics"]["ticks"]


def test_cli_rejects_bad_injection_spec(tmp_path):
    """非法注入规格 → blocked 报告（不抛裸异常）。"""
    path = sample_mod.write_sample_trajectory(str(tmp_path / "traj.json"))
    report = cli_mod.execute(mode="fake", trajectory_path=path,
                             injections=["follow_error:1"])
    assert report["status"] == "blocked"
    assert report["reason"]
    assert report["next_action"]


def test_cli_injection_writes_report(tmp_path):
    """注入复跑的报告落盘（含事件与注入 evidence）。"""
    path = sample_mod.write_sample_trajectory(str(tmp_path / "traj.json"))
    out = str(tmp_path / "inj.json")
    rc = cli_mod.main(["--traj", path, "--mode", "fake",
                       "--inject", "controller_conflict", "--report", out])
    assert rc == 4
    assert os.path.exists(out)
    with open(out, encoding="utf-8") as fh:
        report = json.load(fh)
    assert report["status"] == "error"
    assert any(e["type"] == "controller_trajectory_conflict"
               for e in report["events"])
    assert any("injection: controller_conflict" in line
               for line in report["evidence"])
