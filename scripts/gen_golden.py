#!/usr/bin/env python3
"""golden-file 基线生成器 / 校验器（S07-T02）。

用途
----
本脚本是 **``tests/data/golden/`` 的唯一写入入口**，做两件事：

* **生成**：跑一遍「任务文件 → 场景 → 求解 → 录制 → 会话产物」整条链，
  把结果压成一份 **golden 基线**（关节样本 + 末端点 + 夹爪调度 + 三枚指纹）
  写到 ``tests/data/golden/single_arm_pick_place.json``；
* **校验**（``--check``）：**不写盘**，只比对「现有基线」是否仍与**当前代码**
  的产出一致；不一致时**退出码 1** 并打印差异摘要。

为什么要有 ``--check``
---------------------
CI 的 fast 档只跑「比对」，不重新生成基线——否则任何人都能在改坏代码后
「重新生成一份基线」把红灯变绿。基线是**回归契约**，只能由人显式更新。

口径纪律
--------
* 末端位置用 **:mod:`rm75_drake.kinematics.mdh`**（纯 numpy、零 pydrake）的
  **理论 MDH 正解**复算，**不**与 Drake 实体 FK 混在同一口径里比；
* 三枚指纹（模型 URDF / 场景 YAML / 任务 YAML）绑定**当前仓库文件**，
  模型漂移会让比较器报「基线失效」。

用法
----
::

    python3 scripts/gen_golden.py                # 生成基线（写盘）
    python3 scripts/gen_golden.py --check        # 只比对现有基线（不写盘）
    python3 scripts/gen_golden.py --out other.json --check

退出码
------
``0``  生成成功 / 比对通过；
``1``  比对不一致（基线失效或逐点漂移）、环境不可用（无 pydrake）、任务求解失败；
``2``  用法错误（路径不存在等）。
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = REPO_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

#: 默认 golden 基线路径（**唯一**基线入口）。
DEFAULT_OUT = REPO_ROOT / "tests" / "data" / "golden" / "single_arm_pick_place.json"

#: 默认任务文件（部署实例数据）。
DEFAULT_TASK = REPO_ROOT / "tasks" / "pick_place.yaml"

#: 默认场景真相源。
DEFAULT_SCENARIO = REPO_ROOT / "scenarios" / "single_arm.yaml"


class GoldenGenerationError(RuntimeError):
    """基线生成失败（求解失败 / 环境不可用 / 拼接非法）。"""


def _relative(path: Path) -> str:
    """把绝对路径转成**相对仓库根**的路径（写进产物里的路径字段）。"""
    try:
        return str(Path(path).resolve().relative_to(REPO_ROOT))
    except ValueError:
        return str(path)


def concatenate_trajectories(trajectories: Sequence[Any]) -> Tuple[List[Any], List[Any]]:
    """把各原语轨迹拼成**一条时间连续的关节序列**。

    拼接规则
    --------
    每个原语轨迹都从**上一段结束时刻**开始（``state.t`` 语义），因此拼接时
    **丢掉后续轨迹的首个采样点**（它与上一段的末点同刻同位形），
    否则会出现 ``t_abs`` 重复 ⇒ 违反 ``Recording`` 的「严格递增」不变量。

    返回
    ----
    ``(samples, gripper_actions)``：采样点列表 + 夹爪动作列表（按时间顺序）。
    """
    samples: List[Any] = []
    actions: List[Any] = []
    for index, trajectory in enumerate(trajectories):
        if trajectory is None:
            continue
        items = list(trajectory.samples)
        if index > 0 and items:
            items = items[1:]
        samples.extend(items)
        actions.extend(list(trajectory.gripper_schedule))
    return samples, actions


def build_chain_session(
    task_path: Path = DEFAULT_TASK,
    scenario_path: Path = DEFAULT_SCENARIO,
) -> Any:
    """跑完整条原语链，产出 :class:`~rm75_drake.regression.session.Session`。

    流程
    ----
    ``load_task_file`` → ``load_scenario`` → ``build_scene`` →
    ``solve_task``（无 jitter ⇒ 确定性）→ 轨迹拼接 → ``Recording`` +
    ``build_run_log`` → ``build_session``。

    失败模式
    --------
    * 环境无 pydrake → :class:`ImportError`（由调用方决定如何呈现）；
    * 任务求解失败 → :class:`GoldenGenerationError`（带 stage / reason）；
    * 轨迹拼接后时刻不严格递增 → :class:`GoldenGenerationError`。
    """
    from rm75_drake.device.run_log import RunLogEvent, build_run_log
    from rm75_drake.device.transport import ScriptedTransport
    from rm75_drake.planning.scene import build_scene
    from rm75_drake.recording import Recording, RecordingSample, urdf_sha256
    from rm75_drake.regression.golden import repo_fingerprints
    from rm75_drake.regression.session import build_session
    from rm75_drake.scenario import load_scenario
    from rm75_drake.tasks.schema import load_task_file
    from rm75_drake.tasks.solver import SolverOptions, solve_task

    task = load_task_file(task_path)
    scenario = load_scenario(scenario_path)
    scene = build_scene(scenario)

    # 离线替身传输层（零硬件）：夹爪下发后回读值同步更新 ⇒ ``verify`` 原语可判「到位」。
    transport = ScriptedTransport()
    transport.open()

    options = SolverOptions()
    result = solve_task(task, scene=scene, transport=transport, options=options)
    if not result.success:
        raise GoldenGenerationError(
            "任务求解失败："
            f"stage={result.stage} reason={result.reason} message={result.message}"
        )

    trajectories = [item.trajectory for item in result.results]
    samples, actions = concatenate_trajectories(trajectories)
    if not samples:
        raise GoldenGenerationError("原语链没有产出任何采样点")
    for index in range(1, len(samples)):
        if samples[index].t_abs <= samples[index - 1].t_abs:
            raise GoldenGenerationError(
                f"拼接后的时刻必须严格递增："
                f"samples[{index}].t_abs={samples[index].t_abs} "
                f"≤ samples[{index - 1}].t_abs={samples[index - 1].t_abs}"
            )

    recording = Recording(
        robot_name=options.robot_name,
        dt=float(options.dt),
        samples=tuple(
            RecordingSample(seq=index, t_abs=float(sample.t_abs), q=tuple(sample.q))
            for index, sample in enumerate(samples)
        ),
        gripper_schedule=tuple(actions),
        meta={
            "urdf_sha256": urdf_sha256(),
            "scenario_path": _relative(scenario_path),
        },
    )

    fingerprints = repo_fingerprints()
    run_log = build_run_log(
        [
            RunLogEvent(
                seq=0,
                t_mono_ns=0,
                kind="note",
                payload={"text": f"golden 基线生成：任务 {task.name} 求解成功"},
            )
        ],
        meta={
            "command": "gen_golden",
            "mode": "offline_replay",
            "scenario_path": _relative(scenario_path),
            "robot_name": options.robot_name,
            "dry_run": True,
        },
        summary={"status": "ok", "reason": "completed"},
    )

    return build_session(
        recording,
        run_log,
        meta={
            "robot_name": options.robot_name,
            "task_path": _relative(task_path),
            "scenario_path": _relative(scenario_path),
            "urdf_sha256": fingerprints["urdf_sha256"],
            "task_sha256": fingerprints["task_sha256"],
            "scenario_sha256": fingerprints["scenario_sha256"],
        },
    )


def main(argv: Optional[Sequence[str]] = None) -> int:
    """脚本入口（返回退出码）。"""
    parser = argparse.ArgumentParser(
        description="生成 / 校验 golden-file 基线（S07-T02）"
    )
    parser.add_argument(
        "--out",
        default=str(DEFAULT_OUT),
        help="基线文件路径（默认 tests/data/golden/single_arm_pick_place.json）",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="只比对现有基线是否仍与当前代码产出一致（不写盘）；不一致退出码 1",
    )
    parser.add_argument("--task", default=str(DEFAULT_TASK), help="任务文件路径")
    parser.add_argument("--scenario", default=str(DEFAULT_SCENARIO), help="场景文件路径")
    parser.add_argument("--json", action="store_true", help="以 JSON 输出比对结论")
    args = parser.parse_args(list(argv) if argv is not None else None)

    out_path = Path(args.out)
    task_path = Path(args.task)
    scenario_path = Path(args.scenario)
    for label, path in (("--task", task_path), ("--scenario", scenario_path)):
        if not path.is_file():
            print(f"gen_golden: {label} 指向的文件不存在：{path}", file=sys.stderr)
            return 2

    try:
        from rm75_drake.regression.golden import (
            build_golden,
            compare_golden,
            load_golden,
            repo_fingerprints,
            tcp_positions_from_joints,
            write_golden,
        )
    except ImportError as exc:  # pragma: no cover - 依赖缺失
        print(f"gen_golden: 环境不可用：{exc}", file=sys.stderr)
        return 1

    try:
        session = build_chain_session(task_path, scenario_path)
    except ImportError as exc:
        print(
            f"gen_golden: 环境不可用（需要 Drake）：{exc}"
            "（--check 需要真机无关的离线求解环境）",
            file=sys.stderr,
        )
        return 1
    except GoldenGenerationError as exc:
        print(f"gen_golden: 生成失败：{exc}", file=sys.stderr)
        return 1

    fingerprints = repo_fingerprints()
    joints = [[float(v) for v in sample.q] for sample in session.recording.samples]
    tcp_positions = tcp_positions_from_joints(joints)

    if args.check:
        if not out_path.is_file():
            print(
                f"gen_golden: 基线文件不存在：{out_path}（先跑一次生成）",
                file=sys.stderr,
            )
            return 1
        try:
            baseline = load_golden(out_path)
        except Exception as exc:
            print(f"gen_golden: 基线文件非法：{exc}", file=sys.stderr)
            return 1
        comparison = compare_golden(baseline, session)
        if args.json:
            print(
                json.dumps(
                    {
                        "fingerprints_match": comparison.fingerprints_match,
                        "fingerprint_details": dict(comparison.fingerprint_details),
                        "num_points": comparison.num_points,
                        "max_tcp_diff_m": comparison.max_tcp_diff_m,
                        "num_tcp_mismatches": comparison.num_tcp_mismatches,
                        "pointwise_identical": comparison.pointwise_identical,
                        "gripper_match": comparison.gripper_match,
                        "joint_max_diff_rad": comparison.joint_max_diff_rad,
                        "ok": comparison.ok,
                    },
                    ensure_ascii=False,
                    indent=2,
                )
            )
        else:
            print(f"gen_golden: 比对 {out_path}")
            print(f"gen_golden: {comparison.summary()}")
            if not comparison.fingerprints_match:
                print(
                    "gen_golden: **基线失效**（指纹与当前仓库文件不一致）"
                    " ⇒ 需重新生成基线并复盘差异：",
                    file=sys.stderr,
                )
                for key, detail in comparison.fingerprint_details.items():
                    if detail["baseline"] != detail["current"]:
                        print(
                            f"  - {key}: baseline={detail['baseline']} "
                            f"current={detail['current']}",
                            file=sys.stderr,
                        )
        return 0 if comparison.ok else 1

    baseline = build_golden(
        session, fingerprints=fingerprints, tcp_positions=tcp_positions
    )
    write_golden(baseline, out_path)
    print(
        f"gen_golden: 已写出基线 {out_path}"
        f"（{len(baseline.joint_samples)} 个采样点，"
        f"{len(baseline.tcp_positions)} 个末端点，"
        f"checksum={baseline.checksum[:12]}…）"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
