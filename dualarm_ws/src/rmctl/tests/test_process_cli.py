"""``rmctl run`` 的**工艺路径**接线测试（S06/T6 验收证据）。

覆盖：

* **离线工艺**：``rmctl run pick_place --count 2`` → 退出码 0、输出带
  「离线代理」标注、``--report`` 落 JSON（status=ok / iterations_completed=2）、
  trace JSONL 落盘且逐行可解析；
* **参数互斥 / 边界**：缺执行目标 → 2；``--traj`` 与位置参数同给 → 2；
  ``--count`` 越界（0 / 51）→ 2；``--replay`` 与 ``--traj`` 同给 → 2；
  未知工艺任务名 → 2；
* **只读回放**：``--replay <trace>`` → 退出码 0、逐事件时间线、
  **零运动调用**（执行器入口被调用即失败）；
* **规格缺失**：``--task-file`` 指向不存在的路径 → 退出码 3 + reason 指名 +
  可粘贴 next_action；
* **既有行为逐字不变**：``--traj`` + ``--executor rm_executor`` → 退出码 0；
  ``--live`` 控制器不可达 → 退出码 4；legacy 路径仍标注「不作验收证据」。

只用 git 跟踪的仓库文件 + pytest 临时目录，绝不依赖 .gitignore 目录。
"""
from __future__ import annotations

import json
import os

import pytest

from rmctl import cli as rmctl_cli


# --------------------------------------------------------------------------
# 夹具
# --------------------------------------------------------------------------

def _payload(verify=True):
    """一条符合执行层契约的轨迹（与既有接线测试同形）。"""
    return {
        "schema_version": 1,
        "group": "rm75_arm",
        "joint_names": ["joint%d" % i for i in range(1, 8)],
        "units": {"times": "s", "points": "rad"},
        "times": [0.0, 0.05, 0.1],
        "points": [[0.0] * 7, [0.02] * 7, [0.04] * 7],
        "verify": {"ok": bool(verify), "checks": {}},
    }


@pytest.fixture()
def traj_file(tmp_path):
    path = tmp_path / "traj.json"
    path.write_text(json.dumps(_payload(), ensure_ascii=False), encoding="utf-8")
    return str(path)


@pytest.fixture()
def trace_file(tmp_path):
    """一份**手写**的 trace（inline fixture，含一条失败事件与下一步动作）。"""
    path = tmp_path / "trace.jsonl"
    lines = [
        {
            "schema_version": 1, "ts": 1.0, "iteration": 1, "phase": "execute",
            "segment": "approach_pick", "status": "ok", "severity": "INFO",
            "reason": "段执行完成", "next_action": "", "evidence": [],
        },
        {
            "schema_version": 1, "ts": 2.0, "iteration": 1, "phase": "judge",
            "segment": "grasp", "status": "blocked", "severity": "NEEDS_HUMAN",
            "reason": "夹爪夹空且重抓已用尽",
            "next_action": "人工确认工件后重跑 `rmctl run pick_place`",
            "evidence": ["result=grasp_empty"],
        },
    ]
    with open(path, "w", encoding="utf-8") as fh:
        for line in lines:
            fh.write(json.dumps(line, ensure_ascii=False) + "\n")
    return str(path)


def _planner_available(repo_root):
    """真实规划是否可用（厂商内核 + 已物化模型）。"""
    try:
        from rmctl.model import Model

        Model(ws=repo_root)
    except Exception as exc:  # noqa: BLE001 - 裸环境缺厂商内核/模型
        pytest.skip("真实规划不可用（%s: %r）" % (type(exc).__name__, exc))
    return True


# --------------------------------------------------------------------------
# 离线工艺路径
# --------------------------------------------------------------------------

def test_run_pick_place_offline_count_two(repo_root, tmp_path, capsys):
    """``rmctl run pick_place --count 2`` → 退出码 0 + 离线代理标注 + 报告 JSON。"""
    _planner_available(repo_root)
    report_path = str(tmp_path / "report.json")
    trace_path = str(tmp_path / "trace.jsonl")
    rc = rmctl_cli.main(["run", "pick_place", "--count", "2",
                         "--trace", trace_path, "--report", report_path])
    out = capsys.readouterr().out
    assert rc == 0, out
    assert "离线代理" in out
    assert "mode=fake" in out
    assert os.path.isfile(report_path)
    with open(report_path, "r", encoding="utf-8") as fh:
        report = json.load(fh)
    assert report["status"] == "ok"
    assert report["iterations_completed"] == 2
    assert report["iterations_planned"] == 2
    assert report["segments_executed"] == 14  # 2 圈 × 7 段
    # trace 落盘且逐行可解析（schema_version=1）
    assert os.path.isfile(trace_path)
    with open(trace_path, "r", encoding="utf-8") as fh:
        rows = [json.loads(line) for line in fh if line.strip()]
    assert rows
    assert all(row["schema_version"] == 1 for row in rows)


def test_run_pick_place_default_task_file_and_trace_path(repo_root, tmp_path, capsys):
    """默认规格路径 = workcell/tasks/pick_place.yaml；--trace 落盘位置被尊重。"""
    _planner_available(repo_root)
    trace_path = str(tmp_path / "custom_trace.jsonl")
    rc = rmctl_cli.main(["run", "pick_place", "--count", "1", "--trace", trace_path])
    out = capsys.readouterr().out
    assert rc == 0, out
    assert os.path.isfile(trace_path)
    assert rmctl_cli.PROCESS_DEFAULT_TASK_FILE == os.path.join(
        "workcell", "tasks", "pick_place.yaml")
    assert rmctl_cli.PROCESS_DEFAULT_TRACE == os.path.join(
        "build", "run", "pick_place_trace.jsonl")


# --------------------------------------------------------------------------
# 参数互斥 / 边界
# --------------------------------------------------------------------------

def test_run_without_target_is_usage_error(capsys):
    """既无 --traj 也无工艺任务 → 退出码 2（不再 _load_traj(None) 崩溃）。"""
    rc = rmctl_cli.main(["run"])
    out = capsys.readouterr().out
    assert rc == 2, out
    assert "rmctl run pick_place" in out
    assert "--traj" in out


def test_run_task_and_traj_are_mutually_exclusive(capsys):
    """位置参数与 --traj 同给 → 退出码 2。"""
    rc = rmctl_cli.main(["run", "pick_place", "--traj", "x.json"])
    out = capsys.readouterr().out
    assert rc == 2, out
    assert "互斥" in out


@pytest.mark.parametrize("count", [0, 51, -1])
def test_run_pick_place_count_out_of_range(count, capsys):
    """--count 越界（0 / 51 / 负数）→ 退出码 2。"""
    rc = rmctl_cli.main(["run", "pick_place", "--count", str(count)])
    out = capsys.readouterr().out
    assert rc == 2, out
    assert "越界" in out


def test_run_unknown_task_name(capsys):
    """未知工艺任务名 → 退出码 2 + 可粘贴提示。"""
    rc = rmctl_cli.main(["run", "pick_and_place"])
    out = capsys.readouterr().out
    assert rc == 2, out
    assert "pick_place" in out


def test_replay_with_traj_is_mutually_exclusive(traj_file, capsys):
    """--replay 与 --traj 同给 → 退出码 2。"""
    rc = rmctl_cli.main(["run", "pick_place", "--replay", "t.jsonl",
                         "--traj", traj_file])
    out = capsys.readouterr().out
    assert rc == 2, out
    assert "互斥" in out


# --------------------------------------------------------------------------
# 只读回放
# --------------------------------------------------------------------------

def test_run_replay_is_read_only(trace_file, capsys, monkeypatch):
    """--replay：退出码 0 + 逐事件时间线 + **零运动调用**。"""
    from rm_executor import cli as exec_cli

    calls = []

    def _boom(*_args, **_kwargs):
        calls.append(1)
        raise AssertionError("回放路径绝不允许调用执行器")

    monkeypatch.setattr(exec_cli, "execute", _boom)
    rc = rmctl_cli.main(["run", "pick_place", "--replay", trace_file])
    out = capsys.readouterr().out
    assert rc == 0, out
    assert calls == []
    assert "approach_pick" in out
    assert "grasp" in out
    assert "下一步" in out
    assert "人工确认工件后重跑" in out


def test_run_replay_missing_file_is_blocked(tmp_path, capsys):
    """回放文件不存在 → 退出码 3 + 指名 + 可粘贴下一步（不抛裸异常）。"""
    missing = str(tmp_path / "nope.jsonl")
    rc = rmctl_cli.main(["run", "pick_place", "--replay", missing])
    out = capsys.readouterr().out
    assert rc == 3, out
    assert "nope.jsonl" in out
    assert "下一步" in out


# --------------------------------------------------------------------------
# 规格缺失
# --------------------------------------------------------------------------

def test_run_missing_task_spec_is_blocked(tmp_path, capsys):
    """--task-file 指向不存在的路径 → 退出码 3 + reason 指名 + next_action。"""
    missing = str(tmp_path / "missing.yaml")
    rc = rmctl_cli.main(["run", "pick_place", "--task-file", missing])
    out = capsys.readouterr().out
    assert rc == 3, out
    assert "missing.yaml" in out
    assert "下一步" in out


# --------------------------------------------------------------------------
# 既有行为逐字不变
# --------------------------------------------------------------------------

def test_run_with_rm_executor_still_works(traj_file, tmp_path, capsys):
    """``--traj`` + ``--executor rm_executor`` → 退出码 0（既有行为不变）。"""
    out_path = str(tmp_path / "report.json")
    rc = rmctl_cli.main(["run", "--traj", traj_file, "--executor", "rm_executor",
                         "--report", out_path])
    assert rc == 0, capsys.readouterr().out
    with open(out_path, "r", encoding="utf-8") as fh:
        rep = json.load(fh)
    assert rep["status"] == "ok"
    assert rep["mode"] == "fake"


def test_run_legacy_path_still_marked(traj_file, capsys):
    """legacy 路径仍标注「不作验收证据」（既有行为不变）。"""
    rc = rmctl_cli.main(["run", "--traj", traj_file, "--fast"])
    out = capsys.readouterr().out
    assert "legacy" in out
    assert "不作验收证据" in out
    assert rc in (0, 3), out


def test_run_live_unreachable_controller_is_exit_4(capsys):
    """``rmctl run --live``（控制器不可达）→ 退出码 4（诚实降级）。"""
    rc = rmctl_cli.main(["run", "--live", "--ip", "127.0.0.1", "--port", "59999"])
    out = capsys.readouterr().out
    assert rc == 4, out
    assert "控制器不可达" in out or "真机会话不可用" in out
