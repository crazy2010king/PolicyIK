"""CLI 族测试（S02-T08）：``single`` / ``pick`` / ``bimanual`` / ``replay``。

被测对象
--------
:mod:`rm75_drake.cli` —— ``python3 -m rm75_drake.cli <subcommand>``。

覆盖的契约
----------
**正例**
* 四个子命令的 ``--help`` **退出码 0**；
* ``single`` 走完整链路并产出**录制文件**，该文件可被 :func:`rm75_drake.recording.load_recording`
  读回、可被 ``replay`` 复检（逃逸数 = 0）；
* ``pick`` 产出录制文件，且**含开爪动作调度**（``kind="open"``，开口 u=1000）；
* ``replay`` 对正常录制 → 退出码 0、逃逸数 0；
* ``--json`` 输出可被 ``json.loads`` 解析。

**负例（可解释失败）**
* ``bimanual`` → **非 0 退出码**，且消息说明归属 **S05**（不静默成功、不假装规划）；
* ``replay`` 对**人为篡改**的录制（插入一个碰撞构型）→ 逃逸数 > 0 且**非 0 退出码**；
* ``single`` 同时给 ``--target-xyz`` 与 ``--target-joints`` → 用法错误（退出码 2）；
* ``single`` 不给任何目标 → 用法错误（退出码 2）；
* ``replay`` 指向不存在的文件 → 用法错误（退出码 2）。

耗时说明
--------
``single`` / ``pick`` 的完整链路含 IRIS 区域生成（本机单区域 ~7 s），
因此这两个用例打 ``@pytest.mark.slow``；CI fast 档用 ``-m "not slow"`` 跳过。
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = REPO_ROOT / "src"
SCENARIO_PATH = REPO_ROOT / "scenarios" / "single_arm.yaml"

if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from rm75_drake import cli as cli_mod  # noqa: E402

#: 抬肘名义姿态（碰撞自由）。
START_Q = (0.0, -0.6, 0.0, 0.9, 0.0, 0.5, 0.0)
#: 目标位形（沿 joint_1 偏移 0.3 rad；直线段碰撞自由）。
TARGET_Q = (0.3, -0.6, 0.0, 0.9, 0.0, 0.5, 0.0)

#: 人为构造的**碰撞构型**（本机实测与工作台 box 相撞）。
COLLIDING_Q = (0.0, 1.2, 0.0, 1.5, 0.0, 0.0, 0.0)


def _run_module(*args: str, timeout: int = 120) -> subprocess.CompletedProcess:
    """以子进程方式运行 ``python3 -m rm75_drake.cli``（真实 CLI 行为）。"""
    env = dict(os.environ)
    env["PYTHONPATH"] = str(SRC_DIR) + os.pathsep + env.get("PYTHONPATH", "")
    return subprocess.run(
        [sys.executable, "-m", "rm75_drake.cli", *args],
        cwd=str(REPO_ROOT),
        capture_output=True,
        text=True,
        timeout=timeout,
        env=env,
    )


def _write_recording(path: Path, samples, *, scenario_path: Path) -> Path:
    """写一条最小录制文件（用于 replay 用例）。"""
    payload = {
        "schema_version": 1,
        "robot_name": "rm75",
        "dt": 0.005,
        "samples": [
            {"seq": index, "t_abs": index * 0.005, "q": list(q)}
            for index, q in enumerate(samples)
        ],
        "gripper_schedule": [],
        "created_at": "2026-01-01T00:00:00Z",
        "meta": {
            "urdf_sha256": "",
            "scenario_path": str(scenario_path),
        },
    }
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    return path


# ── --help ──────────────────────────────────────────────────────────────
@pytest.mark.parametrize("command", ["single", "pick", "bimanual", "replay"])
def test_subcommand_help_exits_zero(command):
    """正例：四个子命令的 --help 退出码 0。"""
    result = _run_module(command, "--help")
    assert result.returncode == 0, result.stdout + result.stderr
    assert result.stdout.strip()


def test_top_level_help_exits_zero():
    """正例：顶层 --help 退出码 0。"""
    result = _run_module("--help")
    assert result.returncode == 0, result.stdout + result.stderr
    for command in ("single", "pick", "bimanual", "replay"):
        assert command in result.stdout


# ── bimanual（未实现）────────────────────────────────────────────────────
def test_bimanual_reports_s05_and_returns_nonzero(capsys):
    """负例：bimanual 未实现 → 非 0 退出码 + 明确说明归属 S05。"""
    code = cli_mod.main(["bimanual"])
    captured = capsys.readouterr()
    combined = captured.out + captured.err
    assert code != 0
    assert "S05" in combined
    assert "未实现" in combined


# ── replay ──────────────────────────────────────────────────────────────
def test_replay_ok_recording_exits_zero(tmp_path, capsys):
    """正例：正常录制回放 → 退出码 0、逃逸数 0。"""
    recording = _write_recording(
        tmp_path / "ok.json",
        [START_Q, list(START_Q)],
        scenario_path=SCENARIO_PATH,
    )
    code = cli_mod.main(["replay", "--recording", str(recording)])
    captured = capsys.readouterr()
    assert code == 0, captured.out + captured.err
    assert "逃逸数：0" in captured.out


def test_replay_tampered_recording_reports_escapes(tmp_path, capsys):
    """负例：篡改过的录制 → 逃逸数 > 0 且退出码非 0。"""
    recording = _write_recording(
        tmp_path / "bad.json",
        [START_Q, list(COLLIDING_Q), list(START_Q)],
        scenario_path=SCENARIO_PATH,
    )
    code = cli_mod.main(["replay", "--recording", str(recording)])
    captured = capsys.readouterr()
    combined = captured.out + captured.err
    assert code != 0
    assert "逃逸数：" in combined
    escapes_line = [
        line for line in combined.splitlines() if "逃逸数：" in line
    ][0]
    count = int(escapes_line.split("逃逸数：")[1].strip().split()[0])
    assert count > 0


def test_replay_missing_file_is_usage_error(tmp_path, capsys):
    """负例：录制文件不存在 → 用法错误（退出码 2）。"""
    code = cli_mod.main(["replay", "--recording", str(tmp_path / "nope.json")])
    capsys.readouterr()
    assert code == 2


def test_replay_json_output_is_parseable(tmp_path, capsys):
    """正例：--json 输出可被解析。"""
    recording = _write_recording(
        tmp_path / "ok.json", [START_Q, list(START_Q)], scenario_path=SCENARIO_PATH
    )
    code = cli_mod.main(["replay", "--recording", str(recording), "--json"])
    captured = capsys.readouterr()
    assert code == 0
    payload = json.loads(captured.out)
    assert payload["num_escapes"] == 0
    assert payload["success"] is True


# ── 用法错误 ────────────────────────────────────────────────────────────
def test_single_without_target_is_usage_error(capsys):
    """负例：不给目标 → 用法错误（退出码 2），不是静默成功。"""
    code = cli_mod.main(["single", "--scenario", str(SCENARIO_PATH)])
    captured = capsys.readouterr()
    assert code == 2
    assert "target" in (captured.out + captured.err)


def test_single_with_conflicting_targets_is_usage_error(capsys):
    """负例：同时给 --target-xyz 与 --target-joints → 用法错误。"""
    code = cli_mod.main(
        [
            "single",
            "--scenario",
            str(SCENARIO_PATH),
            "--target-xyz",
            "0.35",
            "0.0",
            "0.55",
            "--target-joints",
            *["0.0"] * 7,
        ]
    )
    capsys.readouterr()
    assert code == 2


def test_pick_without_pre_grasp_is_usage_error(capsys):
    """负例：pick 不给预抓位置 → 用法错误。"""
    code = cli_mod.main(["pick", "--scenario", str(SCENARIO_PATH)])
    captured = capsys.readouterr()
    assert code == 2
    assert "pre-grasp" in (captured.out + captured.err)


def test_single_with_missing_scenario_is_usage_error(capsys):
    """负例：场景文件不存在 → 用法错误（退出码 2）。"""
    code = cli_mod.main(
        [
            "single",
            "--scenario",
            str(REPO_ROOT / "scenarios" / "no-such-scenario.yaml"),
            "--target-joints",
            *["0.0"] * 7,
        ]
    )
    capsys.readouterr()
    assert code == 2


# ── 完整链路（重负载）───────────────────────────────────────────────────
@pytest.mark.slow
def test_single_produces_loadable_recording(tmp_path, capsys):
    """正例：single 产出录制文件，可被读回、可被回放复检（逃逸 0）。"""
    from rm75_drake.recording import load_recording, replay_check
    from rm75_drake.scenario import load_scenario
    from rm75_drake.planning.scene import build_scene

    output = tmp_path / "single.json"
    code = cli_mod.main(
        [
            "single",
            "--scenario",
            str(SCENARIO_PATH),
            "--target-joints",
            *[str(v) for v in TARGET_Q],
            "--start-joints",
            *[str(v) for v in START_Q],
            "--region-spacing",
            "0.5",
            "--iris-samples",
            "3",
            "--output",
            str(output),
        ]
    )
    captured = capsys.readouterr()
    assert code == 0, captured.out + captured.err
    assert output.is_file()

    recording = load_recording(output)
    assert recording.robot_name == "rm75"
    assert len(recording.samples) >= 2
    assert recording.meta["scenario_path"]

    scene = build_scene(load_scenario(SCENARIO_PATH))
    result = replay_check(recording, scene)
    assert result.num_escapes == 0


@pytest.mark.slow
def test_pick_includes_open_gripper_action(tmp_path, capsys):
    """正例：pick 的结果含开爪动作调度（u=1000），且录制可被读回。"""
    from rm75_drake.recording import load_recording

    output = tmp_path / "pick.json"
    code = cli_mod.main(
        [
            "pick",
            "--scenario",
            str(SCENARIO_PATH),
            "--pre-grasp-xyz",
            "0.30",
            "0.0",
            "0.60",
            "--pre-grasp-rpy",
            "0.0",
            "0.0",
            "0.0",
            "--start-joints",
            *[str(v) for v in START_Q],
            "--region-spacing",
            "0.5",
            "--iris-samples",
            "3",
            "--output",
            str(output),
        ]
    )
    captured = capsys.readouterr()
    assert code == 0, captured.out + captured.err
    assert output.is_file()

    recording = load_recording(output)
    assert len(recording.gripper_schedule) >= 1
    action = recording.gripper_schedule[0]
    assert action.kind == "open"
    assert action.position == 1000
