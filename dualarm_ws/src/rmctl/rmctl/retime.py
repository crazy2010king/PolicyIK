"""保路径重定时（自愈阶梯第 1 级）。

规划器给出的轨迹可能违反我们自己的速度/加速度/jerk 限值。解决办法不是重规划
（会换路径、可能换到更差的解），而是**时间缩放**：
    v' = v / s ,  a' = a / s² ,  j' = j / s³
几何路径完全不变（碰撞结论仍然有效），只是走得更慢更平滑。对 10ms 关节透传而言，
平滑度直接决定控制器跟随质量与保护停概率。
"""
import numpy as np


def derivatives(times, points):
    t = np.asarray(times, dtype=float)
    q = np.asarray(points, dtype=float)
    dt = np.diff(t)
    dt[dt <= 0] = 1e-9
    vel = np.diff(q, axis=0) / dt[:, None]
    if vel.shape[0] < 2:
        return vel, np.zeros((0, q.shape[1])), np.zeros((0, q.shape[1]))
    acc = np.diff(vel, axis=0) / dt[:-1][:, None]
    if acc.shape[0] < 2:
        return vel, acc, np.zeros((0, q.shape[1]))
    jerk = np.diff(acc, axis=0) / dt[:-2][:, None]
    return vel, acc, jerk


def required_scale(model, times, points):
    """返回需要的最小时间缩放系数 s（>=1）。"""
    vel, acc, jerk = derivatives(times, points)
    s = 1.0
    if vel.size:
        for i, jn in enumerate(model.joint_names):
            lim = model.limit(jn)
            if lim.get("velocity"):
                s = max(s, float(np.abs(vel[:, i]).max()) / lim["velocity"])
            if lim.get("acceleration") and acc.size:
                s = max(s, float(np.abs(acc[:, i]).max()) / lim["acceleration"])
            if lim.get("jerk") and jerk.size:
                s = max(s, (float(np.abs(jerk[:, i]).max()) / lim["jerk"]) ** (1.0 / 3.0))
    return s


def scale_trajectory(times, points, scale):
    t = np.asarray(times, dtype=float)
    q = np.asarray(points, dtype=float)
    return (t[0] + (t - t[0]) * scale).tolist(), q.tolist()
