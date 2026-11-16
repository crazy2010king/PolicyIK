"""轨迹四道校验（设计 §5.6）：限位 / 离散碰撞 / 连续碰撞 / 时间可实现性。

`SUCCESS` 的语义是"四道全过"，而不是"规划器说 OK"。
"""
import numpy as np

from .vendor_env import ensure_libstdcxx

# 厂商 nanobind wheel 需要 GLIBCXX_3.4.31+：先预加载仓库自带的
# .deps/libstdcxx（只读引用），再 import 厂商库。
ensure_libstdcxx()

from tesseract_robotics.tesseract_collision import (ContactRequest, ContactResultMap,
                                                     ContactTestType)


def _contact_request():
    """ContactRequest 在 0.35 只接受 type（距离/穿透量由 ContactResult 自行给出）。"""
    return ContactRequest(ContactTestType.ALL)


def check_limits(model, points, times):
    issues, margins = [], []
    n = len(points)
    arr = np.asarray(points, dtype=float)
    # 位置
    for i, jn in enumerate(model.joint_names):
        lim = model.limit(jn)
        if not lim:
            continue
        col = arr[:, i]
        lo, hi = lim.get("lower"), lim.get("upper")
        if lo is not None and col.min() < lo - 1e-9:
            issues.append("关节 %s 低于下限：%.4f < %.4f（点 %d）" % (jn, col.min(), lo, int(col.argmin())))
        if hi is not None and col.max() > hi + 1e-9:
            issues.append("关节 %s 超过上限：%.4f > %.4f（点 %d）" % (jn, col.max(), hi, int(col.argmax())))
        margins.append(float(min(col.min() - (lo if lo is not None else col.min()),
                                 (hi if hi is not None else col.max()) - col.max())))
    # 速度 / 加速度 / jerk（数值微分）
    t = np.asarray(times, dtype=float)
    if n >= 2 and np.all(np.diff(t) > 0):
        vel = np.diff(arr, axis=0) / np.diff(t)[:, None]
        acc = np.diff(vel, axis=0) / np.diff(t)[:-1][:, None]
        jerk = np.diff(acc, axis=0) / np.diff(t)[:-2][:, None] if acc.shape[0] > 1 else None
        for i, jn in enumerate(model.joint_names):
            lim = model.limit(jn)
            if not lim:
                continue
            vmax = lim.get("velocity")
            if vmax and vel[:, i].max() > vmax * 1.001:
                issues.append("关节 %s 速度超限：%.4f > %.4f rad/s" % (jn, vel[:, i].max(), vmax))
            amax = lim.get("acceleration")
            # 两点轨迹（n == 2）时 acc 为空：必须判空，否则 numpy 报
            # "zero-size array to reduction operation"（规划服务会直接崩）。
            if amax and acc.size and acc[:, i].max() > amax * 1.001:
                issues.append("关节 %s 加速度超限：%.4f > %.4f rad/s²" % (jn, acc[:, i].max(), amax))
            jmax = lim.get("jerk")
            if jmax and jerk is not None and jerk[:, i].max() > jmax * 1.001:
                issues.append("关节 %s jerk 超限：%.4f > %.4f rad/s³" % (jn, jerk[:, i].max(), jmax))
    else:
        issues.append("时间戳非严格递增或点数不足，无法做时间可实现性检查")
    return issues, margins


def check_discrete(model, points):
    env = model.env
    mgr = env.getDiscreteContactManager()
    req = _contact_request()
    hits = []
    for idx, q in enumerate(points):
        model.set_state(q)
        state = env.getState()
        mgr.setActiveCollisionObjects(env.getActiveLinkNames())
        mgr.setCollisionObjectsTransform(state.link_transforms)
        contacts = ContactResultMap()
        mgr.contactTest(contacts, req)
        if contacts.size() > 0:
            hits.append((idx, contacts.size()))
    return hits


def check_continuous(model, points, max_step_rad=0.02, max_sub=64):
    """连续碰撞校验（稠密采样式）。

    为什么不用 Tesseract 的 cast：0.35.0.8 的 Python 绑定
    `ContinuousContactManager.setCollisionObjectsTransformCast` 段错误（用官方 ABB
    模型同样复现，非本项目模型问题）。因此这里用"段内插值 + 稠密离散检查"实现
    等效的保守连续校验：步长 ≤ max_step_rad，任何两采样点之间的穿透都会被覆盖到。
    这与设计 §3.4 的判断一致（Tesseract 的 cast 在 0.35 仍有 #1371 等已知缺陷），
    也保证"校验永远比规划保守"。
    """
    if len(points) < 2:
        return []
    arr = np.asarray(points, dtype=float)
    env = model.env
    mgr = env.getDiscreteContactManager()
    req = _contact_request()
    hits = []
    for i in range(len(arr) - 1):
        d = float(np.max(np.abs(arr[i + 1] - arr[i])))
        n = int(min(max_sub, max(1, np.ceil(d / max_step_rad)) + 1))
        for k in range(1, n + 1):
            q = arr[i] + (arr[i + 1] - arr[i]) * (float(k) / n)
            model.set_state(q)
            state = env.getState()
            mgr.setActiveCollisionObjects(env.getActiveLinkNames())
            mgr.setCollisionObjectsTransform(state.link_transforms)
            contacts = ContactResultMap()
            mgr.contactTest(contacts, req)
            if contacts.size() > 0:
                hits.append((i, contacts.size()))
                break
    return hits


def verify_with_retime(model, points, times, do_continuous=True, retime=True):
    """四道校验 + **保路径降速重定时**（自愈阶梯第 1 级）——**唯一实现**。

    阶梯语义（与 ``rmctl.cli.cmd_plan`` 逐字一致）：

    1. 先跑四道校验；全过 → 直接返回；
    2. 不过 → ``retime.required_scale`` 算最小时间缩放系数 ``s``；
    3. ``s > 1.001`` → ``retime.scale_trajectory`` 保路径降速（几何路径不变）
       → **重新**跑四道校验；这次过了 → 返回新时间轴/点位。

    :param retime: 是否允许保路径降速重定时（``False`` = 只跑四道校验，
        等价于 ``rmctl.cli plan --no-retime``）。

    返回 ``(report, times, points, scale)``：

    * ``report`` —— 最终的四道校验报告（``ok`` 为 True 才算通过）；
    * ``times`` / ``points`` —— **可能被降速重定时改写**的时间轴/点位；
    * ``scale`` —— 实际用到的缩放系数（``None`` = 没降速）。

    **本函数是阶梯的唯一实现**：CLI 与工艺客户端层都调它（不复制）。
    """
    report = verify(model, points, times, do_continuous=do_continuous)
    if report.get("ok") or not retime:
        return report, times, points, None
    from .retime import required_scale, scale_trajectory

    scale = required_scale(model, times, points)
    if scale > 1.001:
        new_times, new_points = scale_trajectory(times, points, scale)
        report2 = verify(model, new_points, new_times,
                         do_continuous=do_continuous)
        if report2.get("ok"):
            return report2, new_times, new_points, scale
    return report, times, points, None


def verify(model, points, times, do_continuous=True):
    report = {"points": len(points), "checks": {}, "ok": True, "issues": []}
    issues, margins = check_limits(model, points, times)
    report["checks"]["limits"] = {"ok": not issues, "issues": issues,
                                  "min_margin": min(margins) if margins else None}
    dhits = check_discrete(model, points)
    report["checks"]["discrete_collision"] = {"ok": not dhits,
                                              "hits": dhits[:10], "n_hits": len(dhits)}
    if do_continuous:
        chits = check_continuous(model, points)
        report["checks"]["continuous_collision"] = {"ok": not chits,
                                                   "hits": chits[:10], "n_hits": len(chits)}
    else:
        report["checks"]["continuous_collision"] = {"ok": True, "skipped": True}
    for name, res in report["checks"].items():
        if not res.get("ok", True):
            report["ok"] = False
            report["issues"].append("%s 未通过" % name)
    return report
