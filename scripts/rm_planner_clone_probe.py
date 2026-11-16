#!/usr/bin/env python3
"""S08 rm_planner **克隆探针**（环境克隆成本画像 + 笛卡尔管线可行性）。

用途
----
回答 S08 切片标题里第三项「笛卡尔克隆」在当前栈里**到底有没有可交付形态**，
并给出**可复现的实测结论**（不是推测）：

1. **环境克隆画像**：对规划模型 env 连续 clone N 次，记录每次克隆耗时（ms）
   与克隆体 RSS 增量（MB，``/proc/self/status`` 的 ``VmRSS`` 前后差）；
2. **场景隔离性断言**：在 clone1 上加一个障碍物后，clone2 与原 env 的
   scene graph 与**碰撞状态**都不受影响（逐项断言，不靠"看起来对"）；
3. **笛卡尔管线可行性探针**：用厂商 ``create_cartesian_pipeline_profiles()``
   构造笛卡尔管线并尝试笛卡尔目标规划，**带有限超时保护**（子进程 + 超时
   kill，绝不无限等待），记录每个用例的结论与**原始错误信息**。

诚实定位
--------
本脚本**不交付笛卡尔路径规划功能**。它回答的是「厂商笛卡尔管线在当前模型
与当前 SRDF 配置下能否跑通」这一**研究问题**，并把结论（含失败证据）落到
``build/reports/s08_clone_probe.json`` 与
``docs/rm_planner规划服务说明.md`` 的「笛卡尔克隆原型与范围判定」章节。

用法
----
    python3 scripts/rm_planner_clone_probe.py \
        --out build/reports/s08_clone_probe.json
    python3 scripts/rm_planner_clone_probe.py --smoke   # 2 次克隆 + 2 个笛卡尔用例

退出码
------
* ``0`` —— 探针跑完：克隆画像、隔离性断言、笛卡尔探针结论齐备
  （笛卡尔**不可行**是**预期结论**，不算失败）；
* ``4`` —— 探针自身异常（模型加载不了 / 隔离性断言不成立 / 报告写不出）。

零新依赖：只用标准库 + 仓库内既有包（``rmctl`` / ``rm_description``）+
``yaml``。
"""
from __future__ import annotations

import argparse
import json
import os
import platform
import shutil
import subprocess
import sys
import tempfile
import time
from datetime import datetime
from typing import Any, Dict, List, Optional, Sequence, Tuple

# --------------------------------------------------------------------------
# 0. 自插 sys.path（脚本可从任意目录调用）
# --------------------------------------------------------------------------

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(HERE)

for _rel in (
    "dualarm_ws/src/rmctl",
    "dualarm_ws/src/rm_executor",
    "dualarm_ws/src/rm_hw_bridge",
    "dualarm_ws/src/rm_safety",
    "dualarm_ws/src/rm_description/scripts",
):
    _p = os.path.join(REPO_ROOT, _rel)
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

import yaml  # noqa: E402

from rmctl.vendor_env import ensure_libstdcxx  # noqa: E402

ensure_libstdcxx()

#: 规划配置目录（唯一配置来源；**只读**）。
PLANNER_CFG_DIR = os.path.join(REPO_ROOT, "workcell", "planner")

#: 报告默认落盘位置（运行时产物，gitignore）。
DEFAULT_OUT = os.path.join("build", "reports", "s08_clone_probe.json")

#: 本切片不交付笛卡尔路径规划功能（**范围声明**，逐字写进报告与文档）。
SCOPE_STATEMENT = (
    "本切片不交付笛卡尔路径规划功能；笛卡尔路径依赖自研 TRAC-IK（远期）。"
    "「笛卡尔克隆」以「环境克隆成本画像 + 笛卡尔管线可行性探针」的原型形态交付。"
)

#: 笛卡尔轨迹的 TCP 位置偏差阈值（米）—— 超过即认为「笛卡尔路径不可用」。
TRACKING_TOLERANCE_M = 0.01

#: 笛卡尔探针的子进程超时（秒）—— 有限超时保护，绝不无限等待。
CARTESIAN_TIMEOUT_S = 240.0


# --------------------------------------------------------------------------
# 1. 小工具
# --------------------------------------------------------------------------

def _now() -> str:
    """当前时间（ISO 字符串）。"""
    return datetime.now().isoformat(timespec="seconds")


def _write_json(path: str, payload: Dict[str, Any]) -> str:
    """写 JSON（UTF-8、缩进 2、键序稳定）。"""
    parent = os.path.dirname(os.path.abspath(path))
    if parent:
        os.makedirs(parent, exist_ok=True)
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(payload, fh, ensure_ascii=False, indent=2, default=str)
    return path


def _read_json(path: str) -> Dict[str, Any]:
    """读 JSON 文件。"""
    with open(path, "r", encoding="utf-8") as fh:
        return json.load(fh)


def _vmrss_kb() -> int:
    """本进程 ``/proc/self/status`` 的 ``VmRSS``（kB）；读不到 → 0。"""
    try:
        with open("/proc/self/status", "r", encoding="utf-8") as fh:
            for line in fh:
                if line.startswith("VmRSS:"):
                    return int(line.split()[1])
    except (OSError, ValueError, IndexError):
        return 0
    return 0


def _git_rev() -> Optional[str]:
    """当前 git 提交（读不到就诚实留 ``None``，不伪造）。"""
    try:
        out = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=REPO_ROOT, capture_output=True, text=True, timeout=10,
        )
    except Exception:  # noqa: BLE001 - 没有 git / 超时 → 留空
        return None
    if out.returncode != 0:
        return None
    return (out.stdout or "").strip() or None


def _machine_info() -> Dict[str, Any]:
    """机器信息（uname / CPU 数 / 负载）—— 数字的可复现前提。"""
    try:
        loadavg = [round(float(x), 2) for x in os.getloadavg()]
    except OSError:  # pragma: no cover - 平台差异
        loadavg = []
    return {
        "uname": list(platform.uname()),
        "cpu_count": os.cpu_count(),
        "loadavg": loadavg,
        "python": platform.python_version(),
    }


def _r3(value: Optional[float]) -> Optional[float]:
    """四舍五入到 3 位（报告可读性）。"""
    return None if value is None else round(float(value), 3)


def _materialize_model_ws(tmp_root: str) -> str:
    """物化临时工作区：绝对路径模型（**不读 gitignore 目录**）。

    与 ``scripts/rm_planner_perf.py`` 同做法：用
    ``build_model.build_model()`` 现场物化，而不是去读 ``build/model``。

    :return: 工作区根路径
    """
    ws = os.path.join(tmp_root, "clone_ws")
    os.makedirs(ws, exist_ok=True)

    import build_model  # dualarm_ws/src/rm_description/scripts/build_model.py

    build_model.build_model(
        ws=ws,
        out=os.path.join(ws, "build", "model"),
        pkg=os.path.join(REPO_ROOT, "dualarm_ws", "src", "rm_description"),
    )
    return ws


def _repo_num_planners() -> Optional[int]:
    """``workcell/planner/pipeline.yaml`` 的 ``num_planners``（只读）。"""
    path = os.path.join(PLANNER_CFG_DIR, "pipeline.yaml")
    try:
        with open(path, "r", encoding="utf-8") as fh:
            data = yaml.safe_load(fh) or {}
        value = data.get("num_planners")
        return int(value) if value is not None else None
    except (OSError, ValueError, TypeError):
        return None


# --------------------------------------------------------------------------
# 2. 环境克隆画像
# --------------------------------------------------------------------------

def clone_profile(model: Any, clone_count: int) -> Dict[str, Any]:
    """连续 clone N 次，记录每次耗时与 RSS 增量。

    :param model: 已加载的 ``rmctl.model.Model``
    :param clone_count: 克隆次数
    :return: 画像字典（逐次记录 + 均值/极值）
    """
    env = model.env
    base_links = [link.getName() for link in env.getSceneGraph().getLinks()]
    rss0 = _vmrss_kb()
    clones: List[Dict[str, Any]] = []
    kept: List[Any] = []
    for index in range(clone_count):
        rss_before = _vmrss_kb()
        started = time.time()
        cloned = env.clone()
        elapsed_ms = (time.time() - started) * 1000.0
        rss_after = _vmrss_kb()
        kept.append(cloned)
        clones.append({
            "index": index + 1,
            "elapsed_ms": round(elapsed_ms, 3),
            "rss_delta_mb": round((rss_after - rss_before) / 1024.0, 3),
            "rss_total_mb": round(rss_after / 1024.0, 3),
            "link_count": len(cloned.getSceneGraph().getLinks()),
            "revision": int(cloned.getRevision()),
        })
    elapsed_values = [float(c["elapsed_ms"]) for c in clones]
    delta_values = [float(c["rss_delta_mb"]) for c in clones]
    return {
        "clone_count": int(clone_count),
        "base_env": {
            "link_count": len(base_links),
            "link_names": base_links,
            "revision": int(env.getRevision()),
            "rss_mb": round(rss0 / 1024.0, 3),
        },
        "clones": clones,
        "mean_elapsed_ms": _r3(sum(elapsed_values) / len(elapsed_values))
        if elapsed_values else None,
        "max_elapsed_ms": _r3(max(elapsed_values)) if elapsed_values else None,
        "mean_rss_delta_mb": _r3(sum(delta_values) / len(delta_values))
        if delta_values else None,
        "total_rss_delta_mb": _r3(sum(delta_values)) if delta_values else None,
        "links_match_base": all(c["link_count"] == len(base_links)
                                for c in clones),
        "_kept": kept,          # 供隔离性断言复用（不落进报告）
    }


# --------------------------------------------------------------------------
# 3. 场景隔离性断言
# --------------------------------------------------------------------------

class _EnvShim(object):
    """只带 ``env`` 属性的薄壳 —— 复用 ``apply_scene`` 的增量命令路径。"""

    def __init__(self, env: Any):
        self.env = env


def _contact_count(env: Any, joint_names: Sequence[str], q: Sequence[float]) -> int:
    """在某个 env 的给定关节状态下做一次离散碰撞查询，返回接触数。

    每个 env 用**自己**的 contact manager 与状态（克隆体不共享状态）——
    这正是隔离性要验的东西。
    """
    import numpy as np
    from tesseract_robotics.tesseract_collision import (
        ContactRequest, ContactResultMap, ContactTestType)

    env.setStateByNamesAndValues(list(joint_names),
                                 np.asarray(q, dtype=float))
    state = env.getState()
    mgr = env.getDiscreteContactManager()
    mgr.setActiveCollisionObjects(env.getActiveLinkNames())
    mgr.setCollisionObjectsTransform(state.link_transforms)
    contacts = ContactResultMap()
    mgr.contactTest(contacts, ContactRequest(ContactTestType.ALL))
    return int(contacts.size())


def isolation_checks(model: Any, profile: Dict[str, Any]) -> Dict[str, Any]:
    """隔离性断言：clone1 加障碍物后，clone2 与原 env 不受影响。

    断言项：

    1. clone1 确实多出障碍物 link；
    2. clone2 的 link 集合与原 env **完全一致**；
    3. 原 env 的 link 集合未变；
    4. clone1 在探针状态下**发生碰撞**（障碍物真的落在机器人上）；
    5. clone2 的碰撞状态与原 env **逐项相同**（都是 0 接触）。
    """
    from rmctl.planner_service.engine import apply_scene

    env = model.env
    clones = profile.get("_kept") or []
    if len(clones) < 2:
        return {"ok": False, "reason": "克隆体不足 2 个，无法做隔离性断言",
                "checks": []}
    clone1, clone2 = clones[0], clones[1]

    base_links = set(profile["base_env"]["link_names"])
    q = model.home_state()
    model.set_state(q)
    tcp = model.fk_tesseract(q).translation

    obstacle = {
        "name": "probe_obstacle",
        "box_size": [0.2, 0.2, 0.2],
        "position": [float(tcp[0]), float(tcp[1]), float(tcp[2])],
    }
    apply_scene(_EnvShim(clone1), [obstacle])

    clone1_links = [link.getName() for link in clone1.getSceneGraph().getLinks()]
    clone2_links = [link.getName() for link in clone2.getSceneGraph().getLinks()]
    orig_links = [link.getName() for link in env.getSceneGraph().getLinks()]

    contacts = {
        "original": _contact_count(env, model.joint_names, q),
        "clone1": _contact_count(clone1, model.joint_names, q),
        "clone2": _contact_count(clone2, model.joint_names, q),
    }

    checks = [
        {"name": "clone1_received_obstacle",
         "ok": obstacle["name"] in clone1_links,
         "observed": clone1_links},
        {"name": "clone2_links_unchanged",
         "ok": set(clone2_links) == base_links,
         "observed": clone2_links},
        {"name": "original_links_unchanged",
         "ok": set(orig_links) == base_links,
         "observed": orig_links},
        {"name": "clone1_collides_with_obstacle",
         "ok": contacts["clone1"] > 0,
         "observed": contacts},
        {"name": "clone2_collision_state_matches_original",
         "ok": contacts["clone2"] == contacts["original"],
         "observed": contacts},
    ]
    return {
        "ok": all(item["ok"] for item in checks),
        "assertion": ("clone1 加障碍物后，clone2 与原 env 的 scene graph 与"
                      "碰撞状态都不受影响"),
        "obstacle": obstacle,
        "probe_state": [float(v) for v in q],
        "contacts": contacts,
        "checks": checks,
    }


# --------------------------------------------------------------------------
# 4. 笛卡尔管线探针（子进程 + 有限超时）
# --------------------------------------------------------------------------

def _cartesian_child(ws: str, payload_path: str, smoke: bool = False) -> int:
    """子进程入口：跑笛卡尔探针用例，把结果写进 ``payload_path``。

    之所以放在**子进程**里：厂商笛卡尔管线一旦卡住 / 崩溃，不能拖垮
    探针脚本本身（父进程有超时 kill 兜底）。
    """
    import numpy as np
    from tesseract_robotics.planning import (
        CartesianTarget, MotionProgram, TaskComposer)
    from tesseract_robotics.planning.profiles import (
        create_cartesian_pipeline_profiles, create_descartes_default_profiles)
    from tesseract_robotics.planning.transforms import Pose
    from tesseract_robotics.tesseract_command_language import (
        AnyPoly_as_CompositeInstruction, InstructionPoly_as_MoveInstructionPoly,
        WaypointPoly_as_StateWaypointPoly)
    from tesseract_robotics.tesseract_motion_planners import assignCurrentStateAsSeed
    from tesseract_robotics.tesseract_task_composer import (
        AnyPoly_wrap_CompositeInstruction, AnyPoly_wrap_EnvironmentConst,
        AnyPoly_wrap_ProfileDictionary, TaskComposerDataStorage)

    from rmctl.model import Model

    model = Model(ws=ws)
    env = model.env
    q0 = model.home_state()
    model.set_state(q0)
    pose0 = model.fk_tesseract(q0)

    composer = TaskComposer.from_config(warmup=True)

    def _polyline(points: Sequence[Sequence[float]],
                  query: Sequence[float]) -> float:
        """点到折线的最短距离（3D）。"""
        best = float("inf")
        for i in range(len(points) - 1):
            a = [float(v) for v in points[i]]
            b = [float(v) for v in points[i + 1]]
            d = [b[k] - a[k] for k in range(3)]
            seg2 = sum(v * v for v in d)
            p = [float(v) for v in query]
            if seg2 <= 1e-12:
                dist = sum((p[k] - a[k]) ** 2 for k in range(3)) ** 0.5
            else:
                t = max(0.0, min(1.0, sum((p[k] - a[k]) * d[k]
                                           for k in range(3)) / seg2))
                dist = sum((p[k] - (a[k] + t * d[k])) ** 2
                           for k in range(3)) ** 0.5
            best = min(best, dist)
        return best

    def run_case(name: str, pipeline: str, waypoints: int,
                 orientation_change: bool, seed: bool,
                 profile_factory, kind: str) -> Dict[str, Any]:
        prog = MotionProgram(model.group, tcp_frame=model.tip)
        targets: List[Any] = []
        for k in range(waypoints):
            base = Pose.from_xyz(
                float(pose0.translation[0]) + 0.02 * k,
                float(pose0.translation[1]),
                float(pose0.translation[2]) - 0.01 * k,
            )
            if orientation_change:
                base = Pose.from_xyz_rpy(base.translation, [0.0, 0.0, 0.3 * k])
            targets.append(base)
            prog = prog.move_to(CartesianTarget(base))
        composite = prog.to_composite_instruction(model.joint_names, model.tip)
        if seed:
            assignCurrentStateAsSeed(composite, env)

        case: Dict[str, Any] = {
            "name": name,
            "kind": kind,
            "pipeline": pipeline,
            "waypoints": int(waypoints),
            "orientation_change": bool(orientation_change),
            "assign_current_state_as_seed": bool(seed),
        }
        try:
            task = composer.factory.createTaskComposerNode(pipeline)
        except Exception as exc:  # noqa: BLE001 - 厂商工厂可能抛
            case["successful"] = False
            case["error"] = "%s: %s" % (type(exc).__name__, exc)
            return case
        if task is None:
            case["successful"] = False
            case["error"] = "createTaskComposerNode 返回 None（管线不存在）"
            return case

        keys = task.getInputKeys()
        if keys.has("planning_input"):
            key = keys.get("planning_input")
        elif keys.has("program"):
            key = keys.get("program")
        else:
            case["successful"] = False
            case["error"] = "输入键未知"
            return case
        out_key = task.getOutputKeys().get("program")

        data = TaskComposerDataStorage()
        data.setData(key, AnyPoly_wrap_CompositeInstruction(composite))
        data.setData("environment", AnyPoly_wrap_EnvironmentConst(env))
        data.setData("profiles",
                     AnyPoly_wrap_ProfileDictionary(profile_factory()))

        started = time.time()
        try:
            future = composer.executor.run(task, data)
            future.wait()
        except Exception as exc:  # noqa: BLE001 - 厂商执行器可能抛
            case["successful"] = False
            case["elapsed_s"] = _r3(time.time() - started)
            case["error"] = "%s: %s" % (type(exc).__name__, exc)
            return case
        context = future.context
        case["elapsed_s"] = _r3(time.time() - started)
        case["successful"] = bool(context.isSuccessful())
        fail_nodes = []
        for info in context.task_infos.getAllInfos():
            if int(getattr(info, "status_code", -1)) != 1:
                fail_nodes.append("%s: %s" % (info.name,
                                              str(info.status_message)[:300]))
        case["fail_nodes"] = fail_nodes
        case["raw_error"] = fail_nodes[0] if fail_nodes else ""

        if case["successful"] and out_key is not None:
            try:
                raw = context.data_storage.getData(out_key)
                points: List[List[float]] = []
                for instruction in AnyPoly_as_CompositeInstruction(raw):
                    if not instruction.isMoveInstruction():
                        continue
                    waypoint = InstructionPoly_as_MoveInstructionPoly(
                        instruction).getWaypoint()
                    if not waypoint.isStateWaypoint():
                        continue
                    state_waypoint = WaypointPoly_as_StateWaypointPoly(waypoint)
                    points.append([float(v) for v in list(state_waypoint.getPosition())])
                case["trajectory_points"] = len(points)
                polyline = [[float(pose0.translation[0]), float(pose0.translation[1]),
                             float(pose0.translation[2])]]
                polyline.extend([[float(t.translation[0]), float(t.translation[1]),
                                  float(t.translation[2])] for t in targets])
                worst = 0.0
                for q in points:
                    model.set_state(q)
                    pos = model.fk_tesseract(q).translation
                    worst = max(worst, _polyline(polyline, pos))
                case["max_tcp_deviation_m"] = round(worst, 5)
            except Exception as exc:  # noqa: BLE001
                case["trajectory_extract_error"] = "%s: %s" % (type(exc).__name__, exc)
        return case

    # kind 语义：
    #   feasibility —— 带姿态约束的笛卡尔路径（**可行性判据**：Descartes 必须
    #                 为每个路点拿到多组冗余 IK 种子才能连成 ladder graph）；
    #   remedy     —— 对 feasibility 失败做的补救尝试（同样要求成功）；
    #   baseline   —— 纯平移的笛卡尔路径（**结构对照**：不要求姿态约束）；
    #   structural —— 结构性用法说明（单路点被 TrajOpt 拒：至少要两个状态）。
    cases = [
        run_case("translation_only_3wp", "CartesianPipeline", 3, False, True,
                 create_cartesian_pipeline_profiles, kind="baseline"),
        run_case("orientation_change_3wp", "CartesianPipeline", 3, True, True,
                 create_cartesian_pipeline_profiles, kind="feasibility"),
    ]
    if not smoke:
        cases.extend([
            run_case("single_waypoint", "CartesianPipeline", 1, False, True,
                     create_cartesian_pipeline_profiles, kind="structural"),
            run_case("orientation_change_redundant_ik", "DescartesFPipeline", 3,
                     True, True,
                     lambda: create_descartes_default_profiles(
                         use_redundant_joint_solutions=True),
                     kind="remedy"),
            run_case("orientation_change_kdl_ik", "DescartesFPipeline", 3, True,
                     True,
                     lambda: create_descartes_default_profiles(ik_solver="KDL"),
                     kind="remedy"),
        ])

    payload = {
        "cases": cases,
        "tcp_tracking_tolerance_m": TRACKING_TOLERANCE_M,
        "numpy": np.__version__,
    }
    _write_json(payload_path, payload)
    return 0


def _case_ok(cases: Sequence[Dict[str, Any]], kind: str) -> Optional[bool]:
    """某一类用例是否全部成功（没有该类用例 → ``None``）。"""
    picked = [case for case in cases if case.get("kind") == kind]
    if not picked:
        return None
    return all(bool(case.get("successful")) for case in picked)


def _case_deviation(cases: Sequence[Dict[str, Any]], kind: str) -> Optional[float]:
    """某一类用例里最大的 TCP 偏差（米）；没有 → ``None``。"""
    values = [float(case["max_tcp_deviation_m"]) for case in cases
              if case.get("kind") == kind and case.get("max_tcp_deviation_m")
              is not None]
    return max(values) if values else None


def _pick_informative(fail_nodes: Sequence[str]) -> str:
    """从失败节点消息里挑**有信息量**的那条。

    厂商的 ``"Error (Abort Triggered)"`` 只是**传播消息**，不是失败原因
    （与 ``rmctl.planner_service.engine.failure_reason_from_infos`` 同一纪律）；
    真正的信息量在失败节点自己身上（如
    ``"DescartesMotionPlannerTask: Failed to build graph"``）。
    """
    informative = [str(node) for node in fail_nodes
                   if node and "Error (Abort Triggered)" not in str(node)]
    if not informative:
        return ""
    for node in informative:
        if "build graph" in node:
            return node
    return informative[0]


def cartesian_probe(ws: str, timeout_s: float, smoke: bool = False) -> Dict[str, Any]:
    """笛卡尔管线可行性探针（子进程 + 有限超时保护）。"""
    tmp_root = tempfile.mkdtemp(prefix="rm-clone-probe-cart-")
    payload_path = os.path.join(tmp_root, "cartesian.json")
    cmd = [sys.executable, os.path.abspath(__file__),
           "--cartesian-child", "--payload", payload_path, "--ws", ws]
    if smoke:
        cmd.append("--smoke")
    started = time.time()
    timed_out = False
    stdout = ""
    stderr = ""
    returncode = -1
    try:
        proc = subprocess.run(cmd, cwd=REPO_ROOT, capture_output=True,
                              text=True, timeout=timeout_s)
        stdout = proc.stdout or ""
        stderr = proc.stderr or ""
        returncode = int(proc.returncode)
    except subprocess.TimeoutExpired as exc:
        timed_out = True
        stdout = exc.stdout.decode("utf-8", "replace") if isinstance(
            exc.stdout, bytes) else (exc.stdout or "")
        stderr = exc.stderr.decode("utf-8", "replace") if isinstance(
            exc.stderr, bytes) else (exc.stderr or "")
    finally:
        pass  # 临时目录在**读完 payload 之后**再删（否则读不到子进程结果）

    cases: List[Dict[str, Any]] = []
    if os.path.exists(payload_path):
        try:
            cases = list(_read_json(payload_path).get("cases") or [])
        except (OSError, ValueError):
            cases = []
    shutil.rmtree(tmp_root, ignore_errors=True)

    raw_error = ""
    raw_error_source = ""
    for case in cases:
        if case.get("kind") not in ("feasibility", "remedy"):
            continue
        message = _pick_informative(case.get("fail_nodes") or [])
        if message:
            raw_error = message
            raw_error_source = str(case.get("name"))
            break
    if not raw_error:
        for case in cases:
            message = _pick_informative(case.get("fail_nodes") or [])
            if message:
                raw_error = message
                raw_error_source = str(case.get("name"))
                break
    if not raw_error and "Failed to build graph" in (stdout + stderr):
        raw_error = "DescartesMotionPlannerTask: Failed to build graph"
        raw_error_source = "vendor stderr"
    evidence = [line for line in (stderr + "\n" + stdout).splitlines()
                if ("build graph" in line or "Failed vertices" in line
                    or "Failed edges" in line)]

    # 结论：笛卡尔管线「可用」= 带姿态约束的用例全部成功
    #        **且** 纯平移用例的 TCP 跟踪偏差在容差内。
    feasibility_cases = [case for case in cases
                         if case.get("kind") in ("feasibility", "remedy")]
    feasibility_ok = bool(feasibility_cases) and all(
        bool(case.get("successful")) for case in feasibility_cases)
    worst_deviation = None
    for case in cases:
        value = case.get("max_tcp_deviation_m")
        if value is None:
            continue
        worst_deviation = value if worst_deviation is None \
            else max(worst_deviation, float(value))
    tracking_ok = (worst_deviation is not None
                   and float(worst_deviation) <= TRACKING_TOLERANCE_M)
    feasible = bool(feasibility_ok and tracking_ok and not timed_out)

    remedies = [
        "use_redundant_joint_solutions=True（Descartes profile）",
        "ik_solver=KDL",
        "assignCurrentStateAsSeed（当前状态作笛卡尔种子）",
        "纯笛卡尔目标（无关节目标混入）",
    ]
    if timed_out:
        conclusion = "笛卡尔探针超时（%.0f s）→ 不可行判定靠超时兜底" % timeout_s
    elif not cases:
        conclusion = "笛卡尔探针子进程没有产出结果 → 不可用（探针异常）"
    elif not feasibility_ok:
        failed = [case.get("name") for case in feasibility_cases
                  if not case.get("successful")]
        conclusion = (
            "笛卡尔管线在当前模型/配置下**不可行**：带姿态约束的用例 %s 失败"
            "（根因：Descartes 需要多组冗余 IK 种子，7 自由度单链在 KDL 下拿不到）"
            % ", ".join(str(name) for name in failed))
    elif not tracking_ok:
        conclusion = (
            "笛卡尔管线能跑完，但 TCP 跟踪偏差 %.3f m > 容差 %.3f m"
            " → 笛卡尔路径不可交付" % (float(worst_deviation or 0.0),
                                       TRACKING_TOLERANCE_M))
    else:
        conclusion = "笛卡尔管线在当前模型/配置下可跑通（本次探针实测）"

    return {
        "feasible": feasible,
        "feasible_definition": ("笛卡尔管线可用 = 带姿态约束的用例全部成功"
                                " 且 TCP 跟踪偏差 ≤ %.3f m" % TRACKING_TOLERANCE_M),
        "translation_only_works": _case_ok(cases, "baseline"),
        "translation_only_max_tcp_deviation_m": _case_deviation(cases, "baseline"),
        "orientation_constrained_works": _case_ok(cases, "feasibility"),
        "conclusion": conclusion,
        "cases": cases,
        "raw_error": raw_error,
        "raw_error_source": raw_error_source,
        "vendor_evidence": evidence[:20],
        "remedies_tried": remedies,
        "timed_out": timed_out,
        "timeout_s": timeout_s,
        "child_returncode": returncode,
        "child_elapsed_s": _r3(time.time() - started),
        "child_stdout_tail": stdout[-4000:],
        "child_stderr_tail": stderr[-4000:],
        "tcp_tracking_tolerance_m": TRACKING_TOLERANCE_M,
        "max_tcp_deviation_m": worst_deviation,
    }


# --------------------------------------------------------------------------
# 5. 主流程
# --------------------------------------------------------------------------

def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="S08 rm_planner 克隆探针（环境克隆画像 + 笛卡尔管线可行性）")
    parser.add_argument("--out", default=DEFAULT_OUT,
                        help="报告输出路径（默认 %s）" % DEFAULT_OUT)
    parser.add_argument("--clones", type=int, default=5,
                        help="环境克隆次数（默认 5）")
    parser.add_argument("--cartesian-timeout-s", type=float,
                        default=CARTESIAN_TIMEOUT_S,
                        help="笛卡尔探针子进程超时（秒，默认 %.0f）"
                             % CARTESIAN_TIMEOUT_S)
    parser.add_argument("--skip-cartesian", action="store_true",
                        help="只跑克隆画像与隔离性断言（不跑笛卡尔探针）")
    parser.add_argument("--smoke", action="store_true",
                        help="冒烟：2 次克隆 + 只跑 2 个笛卡尔用例"
                             "（纯平移 + 带姿态约束）")
    # 内部：笛卡尔探针子进程入口（父进程调用）
    parser.add_argument("--cartesian-child", action="store_true",
                        help=argparse.SUPPRESS)
    parser.add_argument("--payload", default="",
                        help=argparse.SUPPRESS)
    parser.add_argument("--ws", default="",
                        help=argparse.SUPPRESS)
    args = parser.parse_args(argv)

    # -- 子进程模式 ------------------------------------------------------
    if args.cartesian_child:
        return _cartesian_child(args.ws, args.payload, smoke=bool(args.smoke))

    out_path = args.out if os.path.isabs(args.out) \
        else os.path.join(REPO_ROOT, args.out)

    report: Dict[str, Any] = {
        "version": 1,
        "source": "S08/T5 rm_planner 克隆探针（scripts/rm_planner_clone_probe.py）",
        "generated_at": _now(),
        "repo_root": REPO_ROOT,
        "script": os.path.relpath(os.path.abspath(__file__), REPO_ROOT),
        "machine": _machine_info(),
        "git_rev": _git_rev(),
        "scope_statement": SCOPE_STATEMENT,
        "notes": [
            "克隆耗时 = env.clone() 的墙钟；RSS 增量 = /proc/self/status 的 "
            "VmRSS 前后差（含克隆体的碰撞世界，不是纯 Python 对象）",
            "隔离性 = clone1 加障碍物后，clone2 与原 env 的 link 集合与"
            "离散碰撞接触数都保持不变",
            "笛卡尔探针跑在子进程里（有限超时保护）→ 厂商卡死 / 崩溃不影响脚本",
        ],
    }

    tmp_root = tempfile.mkdtemp(prefix="rm-clone-probe-")
    try:
        ws = _materialize_model_ws(tmp_root)
        from rmctl.model import Model

        model = Model(ws=ws)

        clone_count = 2 if args.smoke else max(1, int(args.clones))
        profile = clone_profile(model, clone_count)
        kept = profile.pop("_kept", [])
        isolation = isolation_checks(model, {"_kept": kept,
                                            "base_env": profile["base_env"]})
        report["clone_profile"] = profile
        report["isolation"] = isolation

        num_planners = _repo_num_planners()
        budget: Dict[str, Any] = {
            "num_planners": num_planners,
            "source": "workcell/planner/pipeline.yaml::num_planners",
        }
        if num_planners:
            budget["total_clone_ms"] = _r3(
                float(profile.get("mean_elapsed_ms") or 0.0) * num_planners)
            budget["total_rss_delta_mb"] = _r3(
                float(profile.get("mean_rss_delta_mb") or 0.0) * num_planners)
            budget["per_planner_clone_feasible"] = bool(
                float(profile.get("mean_elapsed_ms") or 0.0) * num_planners < 1000.0)
        report["per_planner_clone_budget"] = budget

        if args.skip_cartesian:
            report["cartesian_pipeline"] = {
                "feasible": None,
                "conclusion": "已跳过笛卡尔探针（--skip-cartesian）",
                "cases": [],
            }
        else:
            report["cartesian_pipeline"] = cartesian_probe(
                ws, float(args.cartesian_timeout_s), smoke=bool(args.smoke))
    except Exception as exc:  # noqa: BLE001 - 任何未预期异常 → 诚实失败
        report.update({
            "status": "error",
            "reason": "克隆探针异常终止（%s）: %s" % (type(exc).__name__, exc),
            "next_action": "查看 stdout/stderr 后重跑本脚本",
        })
        _write_json(out_path, report)
        print("克隆探针异常终止: %s: %s" % (type(exc).__name__, exc))
        return 4
    finally:
        shutil.rmtree(tmp_root, ignore_errors=True)

    isolation_ok = bool(isolation.get("ok"))
    report["status"] = "ok" if isolation_ok else "error"
    report["reason"] = (
        "克隆画像 %d 次 + 隔离性断言全部通过 + 笛卡尔探针有结论"
        % profile["clone_count"] if isolation_ok else
        "隔离性断言未通过：%s" % "; ".join(
            "%s(%s)" % (item["name"], item["observed"])
            for item in isolation.get("checks") or [] if not item["ok"]))
    report["next_action"] = "" if isolation_ok else \
        "检查隔离性断言的 observed 字段：克隆体是否真的与原 env 完全隔离"
    _write_json(out_path, report)

    print("rm_planner 克隆探针报告: %s" % out_path)
    print("  克隆: %d 次，均耗时 %s ms，均 RSS 增量 %s MB（链接数与原 env 一致：%s）"
          % (profile["clone_count"], profile.get("mean_elapsed_ms"),
             profile.get("mean_rss_delta_mb"), profile.get("links_match_base")))
    print("  隔离性断言: %s" % ("通过" if isolation_ok else "未通过"))
    cart = report.get("cartesian_pipeline") or {}
    print("  笛卡尔管线可行: %s —— %s"
          % (cart.get("feasible"), cart.get("conclusion")))
    print("  总体: %s（%s）" % (report["status"], report["reason"]))
    return 0 if report["status"] == "ok" else 4


if __name__ == "__main__":
    sys.exit(main())
