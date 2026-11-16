"""规划引擎：低层 Task Composer 四步 + per-node 指标 + 四道校验 + 保路径降速自愈 + 确定性种子。

为什么不用厂商的 ``TaskComposer.plan()``：
厂商高层 ``plan()`` 内部 ``del future``，调用方**拿不到** per-node 的
``task_infos``，因此 R008「指标与诊断内置」无法满足。本模块自己走低层四步：

1. ``factory.createTaskComposerNode(pipeline)``  —— 建管线节点；
2. ``TaskComposerDataStorage.setData(...)``       —— 喂 input/environment/profiles；
3. ``executor.run(task, data)`` + ``future.wait()`` —— 跑管线（Taskflow 执行器）；
4. ``future.context.task_infos.getAllInfos()``    —— 取 per-node 指标。

输出键**不硬编码**：厂商 FreespacePipeline 的输出端口名是 ``program``，
实际数据键名是 ``output_data``（见 ``getOutputKeys().get("program")``），
因此这里一律按 task 的**实际**输出键取结果。

纪律：

* **四道校验唯一实现**：本模块**只**调 ``rmctl.verify.verify_with_retime``，
  不复制任何限位 / 碰撞判定（守卫测试会断言源码里没有第二份判定文案）。
* **确定性前提**：OMPL 一旦开始采样就**不可再改种子**（厂商会打印
  ``"Random number generation already started. Changing seed now will not lead
  to deterministic sampling."``），因此种子必须在**任何规划之前**设定
  （``RNG_setSeed``）。这意味着**一个进程里第二次换种子不生效** → 服务端必须
  **每请求独占一个 worker 子进程**（本切片的服务架构正是如此）。此外
  ``deterministic=True`` 时强制 ``num_planners=1``（并行多规划器的胜者取决于
  线程调度 → 不确定）。
* **场景只提交增量命令**：障碍物用 ``AddLinkCommand`` 逐个加到环境里，
  不做全量 scene graph diff / 替换。
* **单位**：时间秒（s）、角度弧度（rad）、长度米（m）。

来源参照（**复制逻辑而非 import 私有函数**）：
``tesseract_robotics/planning/composer.py::_failure_message`` —— 失败原因必须
指名**真实失败节点**（厂商 ``ErrorTask`` 的 ``"Error (Abort Triggered)"`` 只是
传播消息，不是失败原因）。
"""
from __future__ import annotations

import time
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

from rmctl import paths
from rmctl.vendor_env import ensure_libstdcxx

# 厂商 nanobind wheel 需要 GLIBCXX_3.4.31+：先预加载仓库自带的
# .deps/libstdcxx（只读引用），再 import 厂商库。
ensure_libstdcxx()

from rmctl.planner_service.config import load_pipeline_config, load_profiles_config
from rmctl.planner_service.contract import InvalidRequest, PlanRequest, PlanResponse
from rmctl.retime import required_scale, scale_trajectory
from rmctl.verify import resample_trajectory, verify_with_retime

from tesseract_robotics.planning import JointTarget, MotionProgram, TaskComposer
from tesseract_robotics.planning.profiles import (
    create_cartesian_pipeline_profiles,
    create_freespace_pipeline_profiles,
    create_ompl_default_profiles,
    create_trajopt_default_profiles,
)
from tesseract_robotics.tesseract_command_language import (
    AnyPoly_as_CompositeInstruction,
    InstructionPoly_as_MoveInstructionPoly,
    WaypointPoly_as_StateWaypointPoly,
)
from tesseract_robotics.tesseract_environment import AddLinkCommand
from tesseract_robotics.tesseract_geometry import Box
from tesseract_robotics.tesseract_motion_planners import assignCurrentStateAsSeed
from tesseract_robotics.tesseract_motion_planners_ompl import RNG_getSeed, RNG_setSeed
from tesseract_robotics.tesseract_scene_graph import (
    Collision,
    Joint,
    JointType,
    Link,
)
from tesseract_robotics.tesseract_task_composer import (
    AnyPoly_wrap_CompositeInstruction,
    AnyPoly_wrap_EnvironmentConst,
    AnyPoly_wrap_ProfileDictionary,
    TaskComposerDataStorage,
)

__all__ = [
    "ABORT_PROPAGATION_MESSAGE",
    "FAILED_STATUS_CODE",
    "apply_seed",
    "failure_reason_from_infos",
    "node_metrics",
    "plan",
    "verify_and_heal",
]

#: 厂商失败传播消息（**不是**失败原因本身）。
ABORT_PROPAGATION_MESSAGE = "Error (Abort Triggered)"

#: 厂商 ``TaskComposerNodeInfo.status_code``：失败分支取 0。
FAILED_STATUS_CODE = 0

#: 进程内复用的 TaskComposer（加载插件只付一次代价）。
_COMPOSER = None


def _composer():
    """进程内单例的 TaskComposer（只拿 factory / executor，不走高层 plan）。"""
    global _COMPOSER
    if _COMPOSER is None:
        _COMPOSER = TaskComposer.from_config(warmup=True)
    return _COMPOSER


# --------------------------------------------------------------------------
# 确定性种子
# --------------------------------------------------------------------------

def apply_seed(seed: int) -> int:
    """在任何规划**之前**设定随机种子，返回设定后的读回值。

    OMPL 一旦开始采样就不可再改种子——这是「同请求同种子结果逐点一致」的
    关键前提，所以种子必须在**规划前**设定（本函数由 :func:`plan` 在最前面
    调用）。

    **进程级约束**：OMPL 的全局 RNG 只在**第一次采样前**接受新种子；同一进程
    里第二次 ``RNG_setSeed`` 不再产生确定性采样。因此服务端必须**每请求独占
    一个 worker 子进程**（本切片的进程模型）。

    :param seed: 非负整数种子
    :return: ``RNG_getSeed()`` 读回值（用于自检）
    """
    RNG_setSeed(int(seed))
    return int(RNG_getSeed())


# --------------------------------------------------------------------------
# 失败原因 / per-node 指标
# --------------------------------------------------------------------------

def failure_reason_from_infos(infos: Sequence[Any], aborting: Any = None) -> str:
    """从 per-node 信息里指名**真实失败节点**（不是传播消息）。

    逻辑**复制自**厂商 ``tesseract_robotics/planning/composer.py`` 的
    ``_failure_message``（不 import 私有函数）：厂商的 ``ErrorTask``、管线节点
    与途经的 composite 都会报 ``"Error (Abort Triggered)"``，真正有信息量的
    消息在**失败节点自己**身上。

    :param infos: ``TaskComposerNodeInfo`` 序列（name / status_code / status_message）
    :param aborting: ``getAbortingNodeInfo()``（没有节点带具体消息时用它）
    :return: ``"<节点>: <消息>"``（多个用 ``"; "`` 连接）
    """
    specific = sorted(
        "%s: %s" % (info.name, info.status_message)
        for info in infos
        if info.status_code == FAILED_STATUS_CODE
        and info.status_message
        and info.status_message != ABORT_PROPAGATION_MESSAGE
    )
    if specific:
        return "; ".join(specific)
    if aborting is not None and aborting.status_message:
        return "%s: %s" % (aborting.name, aborting.status_message)
    return "Planning failed"


def node_metrics(infos: Sequence[Any]) -> List[Dict[str, Any]]:
    """per-node 指标：``[{name, elapsed_s, status, message}]``。"""
    out: List[Dict[str, Any]] = []
    for info in infos:
        code = int(getattr(info, "status_code", -1))
        if code == FAILED_STATUS_CODE:
            status = "FAILED"
        elif code == 1:
            status = "SUCCESS"
        else:
            status = "UNKNOWN"
        out.append(
            {
                "name": str(getattr(info, "name", "")),
                "elapsed_s": float(getattr(info, "elapsed_time", 0.0) or 0.0),
                "status": status,
                "message": str(getattr(info, "status_message", "") or ""),
            }
        )
    return out


# --------------------------------------------------------------------------
# 四道校验 + 保路径降速自愈
# --------------------------------------------------------------------------

def verify_and_heal(
    model: Any,
    points: Sequence[Sequence[float]],
    times: Sequence[float],
    do_continuous: bool = True,
    max_scale: Optional[float] = None,
    resample_step_rad: Optional[float] = None,
    output_resample_step_rad: Optional[float] = None,
) -> Dict[str, Any]:
    """四道校验 + 保路径降速自愈（阶梯第 1 级）——**不复制判定逻辑**。

    全部判定都来自 ``rmctl.verify.verify_with_retime``（唯一实现）；本函数
    只负责**编排**与**上限约束**：

    * 先做一次「不重定时」的校验；
    * 不过 → 需要的缩放系数 ``need``（``rmctl.retime.required_scale``）；
    * ``need > 1.001`` 且 ``need`` 不超过 ``max_scale`` → 交给
      ``verify_with_retime(retime=True)`` 做保路径降速（几何路径不变）；
    * ``need`` 超过 ``max_scale`` → **不降速**（上限来自
      ``workcell/planner/profiles.yaml::max_time_scaling``），并在 notes 里
      写明「需要人工放宽 / 换路径」。

    **校验分辨率与点数解耦**（``resample_step_rad``）：四道校验的速度/加速度/
    jerk 是对采样点做数值微分，点数少 = 峰值被抹平 = 校验被静默削弱。开启
    ``resample_step_rad``（rad，关节空间最大步长）后，**校验用稠密轨迹**，
    而**下发轨迹**的点数由 ``output_resample_step_rad`` 独立决定（几何一致，
    仅点数不同）。降速缩放系数作用在**原始**时间轴上，稠密轨迹按同一系数
    缩放后再校验 → 「降速后仍过」。

    :return: ``{"report", "times", "points", "scale", "notes",
        "verified_points", "original_points"}``
    """
    points = [list(map(float, p)) for p in points]
    times = [float(t) for t in times]
    notes: List[str] = []

    # 校验用轨迹：按 resample_step_rad 稠密化（校验分辨率与点数解耦）。
    verify_points, verify_times = points, times
    if resample_step_rad is not None and float(resample_step_rad) > 0:
        verify_points, verify_times = resample_trajectory(
            points, times, float(resample_step_rad)
        )

    report, _, _, scale = verify_with_retime(
        model, verify_points, verify_times, do_continuous=do_continuous,
        retime=False,
    )
    if not report.get("ok"):
        need = float(required_scale(model, verify_times, verify_points))
        if need > 1.001:
            if max_scale is not None and need > float(max_scale) + 1e-9:
                notes.append(
                    "需要的时间缩放系数 %.3f 超过上限 %.3f：按上限不降速，"
                    "需要人工放宽 max_time_scaling 或换路径" % (need, float(max_scale))
                )
            else:
                report, _, _, scale = verify_with_retime(
                    model, verify_points, verify_times,
                    do_continuous=do_continuous, retime=True,
                )
                if scale is None:
                    notes.append(
                        "保路径降速（系数 %.3f）后四道校验仍未通过" % need
                    )

    # 下发用轨迹：几何不变；降速缩放作用在**原始**时间轴上；
    # 再按 output_resample_step_rad 稠密化（可选）。
    out_points, out_times = points, times
    if scale is not None:
        out_times, out_points = scale_trajectory(times, points, scale)
    if output_resample_step_rad is not None and float(output_resample_step_rad) > 0:
        out_points, out_times = resample_trajectory(
            out_points, out_times, float(output_resample_step_rad)
        )

    return {
        "report": report,
        "times": out_times,
        "points": out_points,
        "scale": scale,
        "notes": notes,
        "verified_points": len(verify_points),
        "original_points": len(points),
    }


def _verification_from_report(report: Dict[str, Any]) -> Dict[str, Any]:
    """把 ``rmctl.verify`` 的报告映射到契约层的四道校验键。"""
    checks = report.get("checks") or {}
    limits = checks.get("limits") or {"ok": False, "issues": []}
    discrete = checks.get("discrete_collision") or {"ok": False, "issues": []}
    continuous = checks.get("continuous_collision") or {"ok": False, "issues": []}
    issues = list(report.get("issues") or [])
    for name in ("limits", "discrete_collision", "continuous_collision"):
        res = checks.get(name) or {}
        for issue in res.get("issues") or []:
            if issue not in issues:
                issues.append(str(issue))
    return {
        "limits": limits,
        "discrete_collision": discrete,
        "continuous_collision": continuous,
        "ok": bool(report.get("ok")),
        "issues": issues,
    }


# --------------------------------------------------------------------------
# 场景（增量命令）
# --------------------------------------------------------------------------

def apply_scene(model: Any, obstacles: Optional[Sequence[Dict[str, Any]]]) -> List[str]:
    """把请求里的盒子障碍物加到环境里（**只提交增量命令**）。

    每个障碍物 = 在 ``base_link`` 上固定一个 Box 碰撞 link，用
    ``tesseract_environment.AddLinkCommand`` 提交（**不做**全量 scene graph
    diff / 替换）。

    :return: 实际加入的 link 名列表
    """
    if not obstacles:
        return []

    from tesseract_robotics.tesseract_common import Isometry3d as _Iso

    env = model.env
    existing = {link.getName() for link in env.getSceneGraph().getLinks()}
    added: List[str] = []
    for index, obstacle in enumerate(obstacles):
        name = str(obstacle["name"])
        if name in existing:
            raise InvalidRequest(
                "scene.obstacles[%d].name" % index,
                "链接名 %r 在场景里已存在（不覆盖既有 link）" % name,
            )
        size = obstacle["box_size"]
        position = obstacle["position"]
        link = Link(name)
        collision = Collision()
        collision.geometry = Box(float(size[0]), float(size[1]), float(size[2]))
        link.addCollision(collision)
        joint = Joint("joint_%s" % name)
        joint.type = JointType.FIXED
        joint.parent_link_name = "base_link"
        joint.child_link_name = name
        joint.parent_to_joint_origin_transform = _Iso.Identity().translate(
            np.asarray([float(position[0]), float(position[1]), float(position[2])])
        )
        command = AddLinkCommand(link, joint)
        if not env.applyCommand(command):
            raise InvalidRequest(
                "scene.obstacles[%d]" % index,
                "环境拒绝了障碍物 %r 的 AddLinkCommand" % name,
            )
        added.append(name)
    return added


# --------------------------------------------------------------------------
# 规划主流程
# --------------------------------------------------------------------------

def _make_profiles(
    pipeline: str,
    num_planners: int,
    planning_time_s: float,
    planner_range: float,
):
    """按管线名构造 profile（管线名来自 ``pipeline.yaml``，唯一来源）。"""
    if "Freespace" in pipeline:
        return create_freespace_pipeline_profiles(
            num_planners=num_planners,
            planning_time=planning_time_s,
            planner_range=planner_range,
        )
    if "Cartesian" in pipeline:
        return create_cartesian_pipeline_profiles()
    if "TrajOpt" in pipeline:
        return create_trajopt_default_profiles()
    if "OMPL" in pipeline:
        return create_ompl_default_profiles()
    from tesseract_robotics.tesseract_command_language import ProfileDictionary

    return ProfileDictionary()


def _resolve_goal(model: Any, request: PlanRequest) -> Tuple[Optional[List[float]], str]:
    """目标解析：``joint`` 直接用；``pose`` 先 IK。"""
    goal_type = request.goal.get("type")
    values = list(request.goal.get("values") or [])
    if goal_type == "joint":
        return [float(v) for v in values], ""
    if goal_type != "pose":
        raise InvalidRequest(
            "goal.type", "未知目标类型 %r（本引擎只支持 joint / pose）" % goal_type
        )
    from tesseract_robotics.planning.transforms import Pose

    # goal.values = [x, y, z, roll, pitch, yaw]（米 / 弧度）
    pose = Pose.from_xyz_rpy(
        [float(values[0]), float(values[1]), float(values[2])],
        [float(values[3]), float(values[4]), float(values[5])],
    )
    solution = model.ik(pose, seed=request.start_state)
    if solution is None:
        return None, (
            "IK 无解：目标位姿 %s 在 seed=%s 下不可达"
            % ([round(float(v), 4) for v in values],
               [round(float(v), 4) for v in request.start_state])
        )
    return [float(v) for v in list(solution)], ""


def _extract_trajectory(composite: Any) -> Tuple[List[List[float]], List[float]]:
    """从输出 composite 里逐点取关节值与时间戳。

    注意是 ``getPosition()``（**不是** ``getPositions()``）。
    """
    points: List[List[float]] = []
    times: List[float] = []
    for instruction in composite:
        if not instruction.isMoveInstruction():
            continue
        waypoint = InstructionPoly_as_MoveInstructionPoly(instruction).getWaypoint()
        if not waypoint.isStateWaypoint():
            continue
        state_waypoint = WaypointPoly_as_StateWaypointPoly(waypoint)
        positions = state_waypoint.getPosition()
        points.append([float(v) for v in list(positions)])
        times.append(float(state_waypoint.getTime()))
    return points, times


def _emit_stage(on_stage: Optional[Any], stage: str) -> None:
    """发阶段心跳（**回调异常绝不掀翻规划**）。

    阶段名：``"plan"``（开始规划）/ ``"verify"``（开始四道校验）/
    ``"done"``（结束）。服务端据此在超时 / 取消响应里说明「在哪个阶段被
    中断」。
    """
    if on_stage is None:
        return
    try:
        on_stage(stage)
    except Exception:  # noqa: BLE001 - 心跳失败不影响规划本身
        pass


def plan(
    request: PlanRequest,
    ws: Optional[str] = None,
    model: Any = None,
    on_stage: Optional[Any] = None,
    seed_applied: bool = False,
) -> PlanResponse:
    """执行一次规划，返回契约层的 :class:`PlanResponse`。

    :param request: :class:`PlanRequest`（非法请求由构造层逐字段指名地抛出）
    :param ws: 工作区根（决定 ``workcell/planner/*.yaml`` 与模型解析位置）
    :param model: 已加载的 ``rmctl.model.Model``（``None`` → 按 ``ws`` 加载）
    :param on_stage: 阶段心跳回调（可选）：``on_stage("plan"|"verify"|"done")``
    :param seed_applied: 进程级种子**已由调用方设定**（worker 子进程在启动
        时就调了 :func:`apply_seed`）→ 本函数不再重复设定
    :raises InvalidRequest: 请求非法时**透出**（引擎不吞异常）
    """
    try:
        return _plan_inner(
            request, ws=ws, model=model, on_stage=on_stage,
            seed_applied=seed_applied,
        )
    finally:
        _emit_stage(on_stage, "done")


def _plan_inner(
    request: PlanRequest,
    ws: Optional[str] = None,
    model: Any = None,
    on_stage: Optional[Any] = None,
    seed_applied: bool = False,
) -> PlanResponse:
    """:func:`plan` 的实现（外层负责发 ``done`` 心跳）。"""
    if not isinstance(request, PlanRequest):
        raise InvalidRequest(
            "<request>", "必须是 PlanRequest 实例，当前是 %s" % type(request).__name__
        )
    started = time.time()
    ws = ws or paths.workspace_root()

    pipeline_cfg = load_pipeline_config(ws)
    if pipeline_cfg.get("status") != "ok":
        return _failed_response(
            request,
            "管线配置不可用: %s" % pipeline_cfg.get("reason"),
            started,
        )
    profiles_cfg = load_profiles_config(ws)
    if profiles_cfg.get("status") != "ok":
        return _failed_response(
            request,
            "profile 配置不可用: %s" % profiles_cfg.get("reason"),
            started,
        )
    pipeline_spec = pipeline_cfg["spec"]
    profiles_spec = profiles_cfg["spec"]

    pipeline_name = str(pipeline_spec["pipeline"])
    num_planners = int(pipeline_spec["num_planners"])
    planning_time_s = float(pipeline_spec["planning_time_s"])
    planner_range = float(pipeline_spec["planner_range"])
    # 校验分辨率 / 下发密度：两个独立旋钮（校验点数与下发点数解耦）。
    verify_resample_step_rad = float(pipeline_spec["verify_resample_step_rad"])
    output_resample_step_rad = float(pipeline_spec["output_resample_step_rad"])
    deterministic = bool(request.deterministic)
    if deterministic:
        # 并行多规划器的胜者取决于线程调度 → 不确定；确定性模式强制单规划器。
        num_planners = 1
    do_continuous = bool((profiles_spec.get("verify") or {}).get("continuous", True))

    # 时间缩放上限：请求约束与 profile 上限取更严的一个（唯一来源 profiles.yaml）。
    max_scale = float(profiles_spec["max_time_scaling"])
    requested_scale = request.constraints.get("time_scaling")
    if requested_scale is not None:
        max_scale = min(max_scale, float(requested_scale))

    # 种子必须在**任何规划之前**设定（OMPL 开始采样后不可再改）。
    # seed_applied=True：调用方（worker 子进程）已在进程启动时设定过种子。
    if not seed_applied:
        apply_seed(request.seed)

    if model is None:
        from rmctl.model import Model

        model = Model(ws=ws)

    start = [float(v) for v in request.start_state]
    if len(start) != len(model.joint_names):
        raise InvalidRequest(
            "start_state",
            "关节数不匹配：请求 %d 个，模型 %d 个"
            % (len(start), len(model.joint_names)),
        )

    goal_joints, ik_reason = _resolve_goal(model, request)
    if goal_joints is None:
        return _failed_response(request, ik_reason, started, goal_type=request.goal.get("type"))
    if len(goal_joints) != len(model.joint_names):
        raise InvalidRequest(
            "goal.values",
            "目标关节数不匹配：%d 个，模型 %d 个"
            % (len(goal_joints), len(model.joint_names)),
        )

    scene_links = apply_scene(model, (request.scene or {}).get("obstacles") or [])

    model.set_state(start)
    program = (
        MotionProgram(model.group, tcp_frame=model.tip)
        .set_joint_names(model.joint_names)
        # start **必须**作为第一个路点，否则 TrajOpt 拿到单状态路径会报错。
        .move_to(JointTarget(start))
        .move_to(JointTarget(goal_joints))
    )
    composite = program.to_composite_instruction(model.joint_names, model.tip)
    assignCurrentStateAsSeed(composite, model.env)

    composer = _composer()
    task = composer.factory.createTaskComposerNode(pipeline_name)
    if task is None:
        return _failed_response(
            request, "管线 %r 不存在（工厂创建失败）" % pipeline_name, started
        )

    input_keys = task.getInputKeys()
    if input_keys.has("planning_input"):
        input_key = input_keys.get("planning_input")
    elif input_keys.has("program"):
        input_key = input_keys.get("program")
    else:
        return _failed_response(
            request, "管线 %r 的输入键未知" % pipeline_name, started
        )

    # 输出键按 task 的**实际**键名取（端口名 program → 实际键 output_data）。
    output_key = task.getOutputKeys().get("program")
    if output_key is None:
        available = list(task.getOutputKeys().data().values())
        if len(available) != 1:
            return _failed_response(
                request,
                "管线 %r 的输出键无法确定（候选 %s）" % (pipeline_name, available),
                started,
            )
        output_key = available[0]

    profiles = _make_profiles(
        pipeline_name, num_planners, planning_time_s, planner_range
    )
    data = TaskComposerDataStorage()
    data.setData(input_key, AnyPoly_wrap_CompositeInstruction(composite))
    data.setData("environment", AnyPoly_wrap_EnvironmentConst(model.env))
    data.setData("profiles", AnyPoly_wrap_ProfileDictionary(profiles))

    _emit_stage(on_stage, "plan")
    plan_started = time.time()
    future = composer.executor.run(task, data)
    future.wait()
    planning_s = time.time() - plan_started

    context = future.context
    infos = context.task_infos.getAllInfos()
    aborting = context.task_infos.getAbortingNodeInfo()
    nodes = node_metrics(infos)
    metrics: Dict[str, Any] = {
        "pipeline": pipeline_name,
        "seed": int(request.seed),
        "num_planners": int(num_planners),
        "deterministic": bool(deterministic),
        "elapsed_total_s": float(time.time() - started),
        "planning_s": float(planning_s),
        "nodes": nodes,
        "scene_links": scene_links,
        "goal_type": str(request.goal.get("type")),
    }

    if not context.isSuccessful():
        reason = failure_reason_from_infos(infos, aborting)
        metrics["verify"] = {"ok": False, "issues": []}
        metrics["retimed_scale"] = None
        return _failed_response(
            request, "规划失败: %s" % reason, started, metrics=metrics
        )

    raw = context.data_storage.getData(output_key)
    points, times = _extract_trajectory(AnyPoly_as_CompositeInstruction(raw))
    if not points:
        return _failed_response(
            request, "规划成功但输出为空（键 %s）" % output_key, started, metrics=metrics
        )

    _emit_stage(on_stage, "verify")
    healed = verify_and_heal(
        model, points, times, do_continuous=do_continuous, max_scale=max_scale,
        resample_step_rad=verify_resample_step_rad,
        output_resample_step_rad=output_resample_step_rad,
    )
    report = healed["report"]
    verification = _verification_from_report(report)
    metrics["verify"] = {
        "ok": bool(report.get("ok")),
        "issues": list(report.get("issues") or []),
    }
    metrics["retimed_scale"] = healed["scale"]

    if not verification["ok"]:
        reason = "四道校验未通过: %s" % "; ".join(verification["issues"]) or "未知"
        for note in healed["notes"]:
            reason = "%s；%s" % (reason, note)
        return _failed_response(
            request, reason, started, metrics=metrics, verification=verification
        )

    trajectory = {
        "joint_names": list(model.joint_names),
        "times": [float(t) for t in healed["times"]],
        "points": [
            {"t": float(t), "positions": [float(v) for v in p]}
            for t, p in zip(healed["times"], healed["points"])
        ],
    }
    metrics["points"] = len(trajectory["points"])
    # 点数三兄弟（可观测）：原始点数 / 校验点数 / 下发点数。
    metrics["trajectory_points"] = int(healed["original_points"])
    metrics["verified_points"] = int(healed["verified_points"])
    metrics["output_points"] = len(trajectory["points"])
    if healed["notes"]:
        metrics["notes"] = list(healed["notes"])
    return PlanResponse(
        plan_id=request.plan_id,
        status="SUCCESS",
        trajectory=trajectory,
        verification=verification,
        failure_reason="",
        metrics=metrics,
    )


def _failed_response(
    request: PlanRequest,
    reason: str,
    started: float,
    metrics: Optional[Dict[str, Any]] = None,
    verification: Optional[Dict[str, Any]] = None,
    goal_type: Optional[str] = None,
) -> PlanResponse:
    """构造 FAILED 响应（指标齐全，便于诊断）。"""
    base: Dict[str, Any] = {
        "pipeline": "",
        "seed": int(request.seed),
        "num_planners": 0,
        "deterministic": bool(request.deterministic),
        "elapsed_total_s": float(time.time() - started),
        "nodes": [],
        "verify": {"ok": False, "issues": []},
        "retimed_scale": None,
    }
    if goal_type:
        base["goal_type"] = goal_type
    if metrics:
        base.update(metrics)
    return PlanResponse(
        plan_id=request.plan_id,
        status="FAILED",
        trajectory=None,
        verification=verification if verification is not None else _verification_from_report(
            {"ok": False, "issues": [], "checks": {}}
        ),
        failure_reason=reason,
        metrics=base,
    )
