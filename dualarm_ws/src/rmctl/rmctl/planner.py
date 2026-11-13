"""规划层：Task Composer 单例 + 管线封装。

对应设计 §5.13.3：composer 只建一次（加载插件 0.3–1.0 s，复用后每请求只花毫秒级）。
"""
import time

import numpy as np

from tesseract_robotics.planning import (JointTarget, CartesianTarget, MotionProgram, TaskComposer)
from tesseract_robotics.planning.profiles import create_freespace_pipeline_profiles

_COMPOSER = None


def composer():
    global _COMPOSER
    if _COMPOSER is None:
        _COMPOSER = TaskComposer.from_config(warmup=True)
    return _COMPOSER


def freespace_profiles(num_planners=2, planning_time=5.0, planner_range=0.01):
    return create_freespace_pipeline_profiles(num_planners=num_planners,
                                              planning_time=planning_time,
                                              planner_range=planner_range)


class PlanOutcome(object):
    def __init__(self, ok, message, times, points, elapsed, pipeline):
        self.ok = ok
        self.message = message
        self.times = times          # list[float] 秒
        self.points = points        # list[list[float]] rad
        self.elapsed = elapsed
        self.pipeline = pipeline

    def as_dict(self):
        return {"ok": self.ok, "message": self.message, "elapsed_s": self.elapsed,
                "pipeline": self.pipeline, "times": self.times, "points": self.points}


def _extract(result):
    traj = list(result.trajectory or [])
    times, points = [], []
    for pt in traj:
        times.append(float(getattr(pt, "time", 0.0)))
        pos = getattr(pt, "positions", None)
        if pos is None:
            pos = getattr(pt, "joint_positions", None)
        points.append([float(x) for x in np.asarray(pos, dtype=float).reshape(-1)])
    return times, points


def plan_joint(model, start, goal, pipeline="FreespacePipeline", profiles=None, tip=None):
    """关节空间自由空间规划。start 必须作为第一个路点（否则 TrajOpt 会拿到单状态路径）。"""
    start = np.asarray(start, dtype=float).reshape(-1)
    goal = np.asarray(goal, dtype=float).reshape(-1)
    model.set_state(start)
    prof = profiles if profiles is not None else freespace_profiles()
    program = (MotionProgram(model.group, tcp_frame=tip or model.tip)
               .set_joint_names(model.joint_names)
               .move_to(JointTarget(start))
               .move_to(JointTarget(goal)))
    t0 = time.time()
    result = composer().plan(model.robot, program, pipeline=pipeline, profiles=prof)
    dt = time.time() - t0
    times, points = _extract(result)
    return PlanOutcome(bool(result.successful), result.message or "", times, points, dt, pipeline)


def plan_pose(model, start, pose, pipeline="FreespacePipeline", profiles=None, tip=None):
    """位姿目标：先求 IK，再走自由空间规划到该关节解。"""
    start = np.asarray(start, dtype=float).reshape(-1)
    model.set_state(start)
    sol = model.ik(pose, seed=start, tip=tip or model.tip)
    if sol is None:
        return PlanOutcome(False, "IK 无解（seed=%s）" % np.round(start, 3).tolist(),
                           [], [], 0.0, pipeline)
    return plan_joint(model, start, sol, pipeline=pipeline, profiles=profiles, tip=tip)
