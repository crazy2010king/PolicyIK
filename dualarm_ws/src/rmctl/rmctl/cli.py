#!/usr/bin/env python3
"""rmctl —— 双臂工程的操作入口（设计 §6.5）。

同一实现、两种入口：每个子命令既能一次性执行（脚本/CI），也能在 repl 里连续用（交互）。
"""
import argparse
import json
import os
import sys
import time

import numpy as np
import yaml

from . import baseline as bl
from . import paths
from .executor import MockExecutor
from .model import Model

# 注意：planner / verify 在模块顶层 import tesseract_robotics，而 nanobind wheel
# 需要 GLIBCXX_3.4.31+。为了让 `rmctl baseline` / `rmctl doctor` 在没有厂商库
# 的机器上也能跑（基线校验只依赖 yaml+numpy），这里改成按需惰性 import。


# ---------------------------------------------------------------- helpers
def _fmt_vec(v, n=4):
    return "[" + ", ".join("%.*f" % (n, x) for x in np.asarray(v).reshape(-1)) + "]"


def _parse_floats(text):
    if text is None:
        return None
    return [float(x) for x in text.replace(";", ",").split(",") if x.strip()]


def _load_model(args):
    return Model(ws=getattr(args, "ws", None), group=getattr(args, "group", "rm75_arm"))


def _save_traj(path, outcome, model, extra=None):
    data = {"group": model.group, "joint_names": model.joint_names,
            "times": outcome.times, "points": outcome.points,
            "pipeline": outcome.pipeline, "elapsed_s": outcome.elapsed}
    if extra:
        data.update(extra)
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(data, fh, ensure_ascii=False, indent=2)
    return path


def _load_traj(path):
    with open(path, "r", encoding="utf-8") as fh:
        return json.load(fh)


# ---------------------------------------------------------------- commands
def cmd_init(args):
    """生成工位骨架 + 体检清单（设计 §6.6）。"""
    ws = args.ws or paths.workspace_root()
    wc = paths.workcell_dir(ws)
    os.makedirs(os.path.join(wc, "scenes"), exist_ok=True)
    os.makedirs(os.path.join(wc, "tasks"), exist_ok=True)
    os.makedirs(os.path.join(wc, "waypoints"), exist_ok=True)
    yml = os.path.join(wc, "rm_workcell.yaml")
    if not os.path.exists(yml) or args.force:
        with open(yml, "w", encoding="utf-8") as fh:
            fh.write(WORKCELL_TEMPLATE)
    # 物化模型（绝对路径版本），这是"零配置"的关键一步
    try:
        import sys as _sys
        _sys.path.insert(0, os.path.join(paths.description_dir(ws), "scripts"))
        import build_model
        man = build_model.build_model(ws=ws)
        print("模型已物化: %s" % man["urdf"])
    except Exception as exc:
        print("模型物化失败: %r" % exc)
    print("工位已就绪: %s" % wc)
    for p in (yml, paths.urdf_path(ws), paths.srdf_path(ws), paths.joint_limits_path(ws)):
        print("  %-6s %s" % ("OK" if os.path.exists(p) else "MISS", p))
    print("\n下一步: rmctl doctor   # 一条命令体检")
    return 0


def cmd_doctor(args):
    """一条命令体检（设计 §6.5 / §6.8）。"""
    checks = []

    def add(name, ok, detail, action=None, level="RECOVERABLE"):
        checks.append({"name": name, "ok": bool(ok), "detail": detail,
                       "action": action, "level": "INFO" if ok else level})

    ws = args.ws or paths.workspace_root()
    add("workspace", paths._find_src(ws) is not None,
        "工作区: %s" % ws, "检查 RM_WS 或目录结构")
    for label, p in (("urdf", paths.urdf_path(ws)), ("srdf", paths.srdf_path(ws)),
                     ("joint_limits", paths.joint_limits_path(ws))):
        add("model:" + label, os.path.exists(p), p, "rmctl init / 重新生成模型")
    try:
        import tesseract_robotics  # noqa
        add("dep:nanobind", True, "tesseract_robotics %s" % tesseract_robotics.__version__)
    except Exception as exc:
        add("dep:nanobind", False, str(exc), "pip install tesseract-robotics-nanobind==0.35.0.8", "NEEDS_HUMAN")

    model = None
    try:
        model = _load_model(args)
        add("model:load", True, "组=%s 关节=%d 连杆=%d" %
            (model.group, len(model.joint_names), len(model.link_names)))
    except Exception as exc:
        add("model:load", False, repr(exc), "检查 URDF/SRDF 与插件配置", "NEEDS_HUMAN")

    if model is not None:
        # 自研 FK vs Tesseract FK
        rng = np.random.default_rng(0)
        worst = 0.0
        for _ in range(20):
            q = np.array([rng.uniform(model.limit(j).get("lower", -3.0),
                                       model.limit(j).get("upper", 3.0))
                           for j in model.joint_names])
            T = model.fk_self(q)
            P = model.fk_tesseract(q)
            d = float(np.linalg.norm(T[:3, 3] - np.asarray([P.x, P.y, P.z])))
            worst = max(worst, d)
        add("fk:self_vs_tesseract", worst < 1e-6,
            "20 组随机位形最大位置偏差 %.3e m" % worst,
            "检查 URDF 清洗结果（clean_urdf.py）")
        # 零位自碰撞
        hits = model.urdf.collisions(np.zeros(len(model.joint_names)))
        add("collision:home", not hits, "零位自碰撞对: %s" % (hits or "无"),
            "检查 ACM（scripts/gen_srdf.py）")
        # 限值一致性
        mismatch = []
        for j in model.joint_names:
            ul = model.urdf.joints[[x["name"] for x in model.urdf.rev].index(j)]
            lim = model.limit(j)
            if lim.get("upper") is not None and abs(ul["upper"] - lim["upper"]) > 1e-4:
                mismatch.append("%s urdf=%.4f yaml=%.4f" % (j, ul["upper"], lim["upper"]))
            if lim.get("lower") is not None and abs(ul["lower"] - lim["lower"]) > 1e-4:
                mismatch.append("%s urdf=%.4f yaml=%.4f" % (j, ul["lower"], lim["lower"]))
        add("limits:consistent", not mismatch, "URDF vs joint_limits.yaml: %s" % (mismatch or "一致"),
            "以 joint_limits.yaml 为准，必要时重新清洗 URDF")
        # 硬件
        add("hardware:connected", False, "未检测到真机连接（原型阶段为模拟执行）",
            "rmctl connect --left <ip> --right <ip>", "INFO")

    # 基线齐备度（基线校验只依赖 yaml+numpy，与厂商库无关）
    try:
        report = bl.validate_all(ws)
        missing = report["missing"]
        present = len(bl.KINDS) - len(missing)
        add("baselines:present", not missing,
            "四类基线 %d/%d 齐备%s" % (present, len(bl.KINDS),
                                       ("（缺: %s）" % ", ".join(missing)) if missing else ""),
            "python3 -m rmctl baseline scaffold")
        ml = report["kinds"].get("measured_limits", {})
        ml_status = ml.get("status")
        add("limits:measured", ml_status == "measured",
            "measured_limits.status=%s（acc/jerk 实测值未回填）" % (ml_status or "缺失"),
            "真机窗口执行 python3 -m rmctl.hwcollect ladder_acc_jerk "
            "并回填 workcell/baselines/measured_limits.yaml（见 docs/code_doc/真机基线与实测限值归档说明.md）",
            "INFO")
    except Exception as exc:
        add("baselines:present", False, repr(exc),
            "python3 -m rmctl baseline scaffold")

    ok = sum(1 for c in checks if c["ok"])
    print("体检结果: %d/%d 通过\n" % (ok, len(checks)))
    for c in checks:
        print("  [%s] %-26s %s" % ("PASS" if c["ok"] else c["level"], c["name"], c["detail"]))
        if not c["ok"] and c["action"]:
            print("        → 下一步: %s" % c["action"])
    bad = [c for c in checks if not c["ok"] and c["level"] != "INFO"]
    print("\n%s" % ("全部通过，可以开始规划。" if not bad else "存在 %d 项需处理。" % len(bad)))
    return 0 if not bad else 1


# ---------------------------------------------------------------- 基线子命令
def _baseline_report(ws):
    """基线结论的唯一来源：baseline.validate_all（不另写一套判定逻辑）。"""
    return bl.validate_all(ws)


def _baseline_kind_rows(report):
    """把 validate_all 报告拆成逐 kind 的 (kind, level, detail) 行。

    level: OK（齐备）/ PENDING（未实测，不阻塞）/ BLOCKED（被阻塞，不阻塞）/ FAIL（阻塞项）。
    """
    rows = []
    for kind in bl.KINDS:
        entry = report["kinds"].get(kind, {})
        issues = entry.get("issues", [])
        errors = [i for i in issues if i["level"] == "error"]
        warns = [i for i in issues if i["level"] == "warn"]
        infos = [i for i in issues if i["level"] == "info"]
        if not entry.get("present"):
            level = "FAIL"
        elif errors:
            level = "FAIL"
        elif warns:
            level = "WARN"
        elif entry.get("status") == "blocked":
            level = "BLOCKED"
        elif infos:
            level = "PENDING"
        else:
            level = "OK"
        detail = []
        if not entry.get("present"):
            detail.append("文件缺失")
        else:
            detail.append("version=%s" % entry.get("version"))
            detail.append("status=%s" % (entry.get("status") or "?"))
            if entry.get("status") == "blocked":
                reason = ""
                for issue in issues:
                    if issue["code"] == "blocked":
                        reason = issue["message"].split("：", 1)[-1]
                        break
                if reason:
                    detail.append("原因=%s" % reason)
        if errors:
            detail.append("%d 项错误" % len(errors))
        elif warns:
            detail.append("%d 项警告" % len(warns))
        rows.append((kind, level, " ".join(detail), issues))
    return rows


def _workcell_seam(ws):
    """校验 workcell/rm_workcell.yaml 与 baselines/ 的接缝。

    返回 (issues, notes)：issues 是阻塞项（声明路径与实际文件不一致），
    notes 是说明性信息（如 workcell 文件缺失）。
    """
    issues, notes = [], []
    wc = paths.workcell_dir(ws)
    yml = os.path.join(wc, "rm_workcell.yaml")
    if not os.path.exists(yml):
        notes.append("workcell 定义缺失: %s（跳过接缝校验）" % yml)
        return issues, notes
    try:
        with open(yml, "r", encoding="utf-8") as fh:
            data = yaml.safe_load(fh) or {}
    except Exception as exc:
        issues.append("workcell/rm_workcell.yaml 解析失败: %r" % exc)
        return issues, notes
    robot = data.get("robot") or {}
    declared_dir = robot.get("baselines_dir")
    actual_dir = os.path.abspath(paths.baselines_dir(ws))
    if not declared_dir:
        issues.append("workcell/rm_workcell.yaml 缺 robot.baselines_dir（基线目录入口）")
    else:
        resolved = os.path.abspath(os.path.join(wc, declared_dir))
        if resolved != actual_dir:
            issues.append("robot.baselines_dir=%r 解析到 %s，与 paths.baselines_dir=%s 不一致"
                          % (declared_dir, resolved, actual_dir))
    calibration = data.get("calibration") or {}
    for kind in bl.KINDS:
        rel = calibration.get(kind)
        if not rel:
            issues.append("workcell/rm_workcell.yaml 缺 calibration.%s（基线声明路径）" % kind)
            continue
        resolved = os.path.abspath(os.path.join(wc, rel))
        expected = os.path.abspath(os.path.join(actual_dir, "%s.yaml" % kind))
        if resolved != expected:
            issues.append("calibration.%s=%r 解析到 %s，与实际基线文件 %s 不一致"
                          % (kind, rel, resolved, expected))
        elif not os.path.exists(resolved):
            issues.append("calibration.%s=%r 指向的基线文件不存在: %s" % (kind, rel, resolved))
    return issues, notes


def cmd_baseline(args):
    """`rmctl baseline [action]`：action = check（默认）/ list / show / scaffold。"""
    action = getattr(args, "action", None) or "check"
    if action == "check":
        return cmd_baseline_check(args)
    if action == "list":
        return cmd_baseline_list(args)
    if action == "show":
        if not args.kind:
            print("show 需要基线类型：rmctl baseline show <%s>" % "|".join(bl.KINDS))
            return 2
        return cmd_baseline_show(args)
    if action == "scaffold":
        return cmd_baseline_scaffold(args)
    print("未知 action: %s" % action)
    return 2


def cmd_baseline_check(args):
    """`rmctl baseline check`：逐 kind 输出存在性/版本/一致性结论与分级 level。"""
    ws = args.ws or paths.workspace_root()
    report = _baseline_report(ws)
    rows = _baseline_kind_rows(report)
    print("基线检查: %s" % report["baselines_dir"])
    blocking = []
    for kind, level, detail, issues in rows:
        print("  [%-7s] %-16s %s" % (level, kind, detail))
        for issue in issues:
            if issue["level"] in ("error", "warn"):
                loc = ("[%s] " % issue["path"]) if issue.get("path") else ""
                print("        %s %s%s: %s" % (issue["level"].upper(), loc,
                                              issue["code"], issue["message"]))
        if level == "FAIL":
            blocking.append(kind)
    # workcell 接缝（声明路径 vs 实际文件）
    seam_issues, seam_notes = _workcell_seam(ws)
    for note in seam_notes:
        print("  [NOTE  ] %s" % note)
    for issue in seam_issues:
        print("  [FAIL  ] workcell 接缝: %s" % issue)
        blocking.append("workcell")
    print("")
    if blocking:
        print("阻塞项 %d 个（%s）。" % (len(blocking), ", ".join(blocking)))
        print("→ 修复: python3 -m rmctl baseline scaffold   # 生成 status=pending 占位基线")
        return 3
    pending = [k for k, lv, _, _ in rows if lv == "PENDING"]
    blocked = [k for k, lv, _, _ in rows if lv == "BLOCKED"]
    if blocked:
        print("基线被阻塞（status=blocked，已附阻塞原因与已尝试动作）: %s" % ", ".join(blocked))
        print("→ 这些条目**没有**任何实测数值；真机窗口可用后按 "
              "docs/code_doc/真机基线与实测限值归档说明.md 重新执行并回填")
    if pending:
        print("基线结构齐备，未实测: %s（不阻塞；实测回填后 status 改为 measured）"
              % ", ".join(pending))
    if not pending and not blocked:
        print("基线齐备且均已实测。")
    return 0


def cmd_baseline_list(args):
    """`rmctl baseline list`：列出四类基线与 status。"""
    ws = args.ws or paths.workspace_root()
    report = _baseline_report(ws)
    print("%-16s %-10s %-8s %s" % ("kind", "status", "version", "path"))
    for kind in bl.KINDS:
        entry = report["kinds"].get(kind, {})
        path = os.path.join(report["baselines_dir"], "%s.yaml" % kind)
        print("%-16s %-10s %-8s %s" % (
            kind, entry.get("status") or "-", entry.get("version") or "-",
            path if entry.get("present") else path + "  (缺失)"))
    return 0


def cmd_baseline_show(args):
    """`rmctl baseline show <kind>`：打印单份基线内容。"""
    ws = args.ws or paths.workspace_root()
    kind = args.kind
    if kind not in bl.KINDS:
        print("未知基线类型: %s（可选: %s）" % (kind, ", ".join(bl.KINDS)))
        return 2
    try:
        data = bl.load_baseline(kind, ws)
    except bl.BaselineError as exc:
        print(str(exc))
        return 3
    print(yaml.safe_dump(data, allow_unicode=True, sort_keys=False))
    return 0


def _scaffold_payload(kind):
    """占位基线的最小合法内容（status=pending，未实测项一律 null）。

    结构对齐 baseline schema：生成后必须能通过 baseline.validate_all，
    避免"占位文件本身就是坏基线"。
    """
    if kind == "zero_baseline":
        return {
            "kind": kind, "schema_version": 1, "version": 1, "status": "pending",
            "captured_at": None, "arm": "left", "firmware": None,
            "teachpendant_version": None,
            "joints": [{"name": n, "zero_deg": None, "unit": bl.ANGLE_UNIT,
                        "groove_photo": None} for n in bl.JOINT_NAMES],
            "groove_photos": [],
            "fk_recheck": {"method": None, "max_pos_dev_mm": None,
                           "max_ori_dev_deg": None, "conclusion": None,
                           "controller_pose_usage": bl.CONTROLLER_POSE_USAGE},
            "recalibration_triggers": ["换电池", "拆装后", "机械冲击后", "固件升级后"],
        }
    if kind == "measured_limits":
        def item(unit):
            return {"value": None, "unit": unit, "source": None, "method": None,
                    "samples": None}
        return {
            "kind": kind, "schema_version": 1, "version": 1, "status": "pending",
            "captured_at": None, "arm": "left", "udp_period_ms": bl.UDP_PERIOD_MS,
            "joints": [{"name": n, "acceleration": item(bl.ACCELERATION_UNIT),
                        "jerk": item(bl.JERK_UNIT)} for n in bl.JOINT_NAMES],
            "tcp_repeatability": {"value_mm": None, "unit": bl.LENGTH_UNIT,
                                  "source": None, "method": None, "samples": None},
        }
    if kind == "version_matrix":
        return {
            "kind": kind, "schema_version": 1, "version": 1, "status": "pending",
            "captured_at": None, "controller_firmware": None, "teachpendant": None,
            "rm_api2_sdk": None, "ros2_rm_robot_tag": None,
            "tesseract_robotics_nanobind": None, "libstdcxx": None, "python": None,
            "os": None, "container_image": None,
        }
    if kind == "network_plan":
        return {
            "kind": kind, "schema_version": 1, "version": 1, "status": "pending",
            "captured_at": None,
            "arms": {"left": {"ip": "192.168.1.18", "tcp_port": 8080, "udp_port": 8091},
                     "right": {"ip": "192.168.1.19", "tcp_port": 8080, "udp_port": 8092}},
            "realtime_report": {"period_ms": bl.UDP_PERIOD_MS,
                                "cycle_multiple_ms": bl.UDP_PERIOD_MS,
                                "mode": "triple_thread"},
            "constraints": {"single_client_per_endpoint": True,
                            "triple_thread_mode": True, "wired_direct": True},
        }
    raise ValueError("未知基线类型: %s" % kind)


def cmd_baseline_scaffold(args):
    """`rmctl baseline scaffold`：缺失时生成 status=pending 占位基线。"""
    ws = args.ws or paths.workspace_root()
    bdir = paths.baselines_dir(ws)
    os.makedirs(bdir, exist_ok=True)
    created, skipped = [], []
    for kind in bl.KINDS:
        path = os.path.join(bdir, "%s.yaml" % kind)
        if os.path.exists(path) and not args.force:
            skipped.append((kind, path))
            continue
        payload = _scaffold_payload(kind)
        # 自检：占位文件本身必须是合法基线（否则 scaffold 产出的是坏文件）
        issues = bl.validate_baseline(kind, payload, ws=ws)
        errors = [i for i in issues if i["level"] == "error"]
        if errors:
            print("占位模板自检失败（%s）: %s" % (kind, bl.format_issues(errors)))
            return 3
        header = ("# %s（占位骨架，由 `rmctl baseline scaffold` 生成）\n"
                  "# status=pending：未实测项一律 null，禁止填写占位数值。\n"
                  "# 实测回填后把 status 改为 measured 并补齐 source/method/samples。\n"
                  % kind)
        with open(path, "w", encoding="utf-8") as fh:
            fh.write(header)
            yaml.safe_dump(payload, fh, allow_unicode=True, sort_keys=False,
                           default_flow_style=False)
        created.append((kind, path))
    for kind, path in created:
        print("  [新建] %-16s %s" % (kind, path))
    for kind, path in skipped:
        print("  [已存在] %-16s %s（加 --force 覆盖）" % (kind, path))
    print("")
    print("已生成 %d 份占位基线（status=pending），实测后回填并改 status=measured。"
          % len(created))
    return 0


def cmd_fk(args):
    model = _load_model(args)
    q = np.asarray(_parse_floats(args.joints) or model.home_state())
    T = model.fk_self(q, args.tip or model.tip)
    P = model.fk_tesseract(q, args.tip or model.tip)
    p_self = T[:3, 3]
    p_tess = np.array([P.x, P.y, P.z])
    print("关节(rad) : %s" % _fmt_vec(q))
    print("自研 FK    : pos=%s" % _fmt_vec(p_self, 6))
    print("Tesseract : pos=%s" % _fmt_vec(p_tess, 6))
    print("偏差      : %.3e m" % float(np.linalg.norm(p_self - p_tess)))
    return 0


def cmd_ik(args):
    from tesseract_robotics.planning import Pose
    model = _load_model(args)
    vals = _parse_floats(args.pose)
    if len(vals) == 6:
        pose = Pose.from_xyz_rpy(*vals)
    elif len(vals) == 7:
        pose = Pose.from_xyz_quat(*vals)
    else:
        print("pose 需要 6 个数(x,y,z,rx,ry,rz) 或 7 个数(x,y,z,qx,qy,qz,qw)")
        return 2
    seed = np.asarray(_parse_floats(args.seed)) if args.seed else model.home_state()
    sol = model.ik(pose, seed=seed, tip=args.tip or model.tip)
    if sol is None:
        print("IK 无解（seed=%s）" % _fmt_vec(seed, 3))
        return 1
    print("IK 解(rad): %s" % _fmt_vec(sol))
    T = model.fk_self(sol, args.tip or model.tip)
    print("回代 FK   : pos=%s" % _fmt_vec(T[:3, 3], 6))
    return 0


def cmd_plan(args):
    from .planner import plan_joint, plan_pose, freespace_profiles
    from .verify import verify
    model = _load_model(args)
    start = np.asarray(_parse_floats(args.start) or model.home_state())
    goal = _parse_floats(args.goal)
    if goal is None:
        print("需要 --goal（关节角，逗号分隔）")
        return 2
    prof = freespace_profiles(num_planners=args.num_planners, planning_time=args.planning_time)
    if args.goal_pose:
        from tesseract_robotics.planning import Pose
        vals = _parse_floats(args.goal_pose)
        pose = Pose.from_xyz_rpy(*vals) if len(vals) == 6 else Pose.from_xyz_quat(*vals)
        out = plan_pose(model, start, pose, pipeline=args.pipeline, profiles=prof)
    else:
        out = plan_joint(model, start, goal, pipeline=args.pipeline, profiles=prof)
    print("管线 %s | 成功=%s | 耗时 %.3fs | 点数 %d" %
          (out.pipeline, out.ok, out.elapsed, len(out.points)))
    if not out.ok:
        print("失败原因: %s" % (out.message or "未知"))
        return 1
    report = None
    if not args.no_verify:
        from .retime import required_scale, scale_trajectory
        report = verify(model, out.points, out.times, do_continuous=not args.no_continuous)
        # 自愈阶梯第 1 级：保路径降速重定时（几何路径不变 → 碰撞结论仍然有效）
        if not report["ok"] and not args.no_retime:
            s = required_scale(model, out.times, out.points)
            if s > 1.001:
                new_times, new_points = scale_trajectory(out.times, out.points, s)
                rep2 = verify(model, new_points, new_times, do_continuous=not args.no_continuous)
                if rep2["ok"]:
                    print("\n自动降速重定时 ×%.3f（路径不变）→ 四道校验全部通过" % s)
                    out.times, out.points = new_times, new_points
                    report = rep2
        print("\n四道校验: %s" % ("全部通过" if report["ok"] else "未通过"))
        for name, res in report["checks"].items():
            print("  [%s] %s" % ("PASS" if res.get("ok") else "FAIL", name))
            for issue in res.get("issues", [])[:5]:
                print("        %s" % issue)
    if args.out:
        _save_traj(args.out, out, model, {"verify": report})
        print("\n轨迹已保存: %s" % args.out)
    return 0 if (report is None or report["ok"]) else 3


def cmd_check(args):
    from .verify import verify
    model = _load_model(args)
    data = _load_traj(args.traj)
    report = verify(model, data["points"], data["times"])
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["ok"] else 3


def cmd_run(args):
    """模拟执行：10 ms 时钟回放（单臂先动起来）。"""
    model = _load_model(args)
    data = _load_traj(args.traj)
    ex = MockExecutor(model, period=args.period)
    print("开始回放 %d 点 / 计划时长 %.2fs（模拟执行器）" %
          (len(data["times"]), data["times"][-1] - data["times"][0]))
    res = ex.run(data["times"], data["points"], realtime=not args.fast)
    print("执行完成: %s" % json.dumps(res.stats, ensure_ascii=False))
    return 0 if res.ok else 1


def cmd_ui(args):
    """浏览器可视化 + 按 10 ms 栅格播放轨迹。"""
    model = _load_model(args)
    from .viewer import serve_background, animate
    v = serve_background(model, port=args.port)
    print("浏览器可视化已启动: http://127.0.0.1:%d" % args.port)
    if args.traj:
        data = _load_traj(args.traj)
        print("播放轨迹 %d 点 / %.2fs（10 ms 栅格）…" % (len(data["times"]),
                                                   data["times"][-1] - data["times"][0]))
        try:
            animate(v, model, data["times"], data["points"], realtime=not args.fast)
        except KeyboardInterrupt:
            print("\n停止播放。")
    print("按 Ctrl-C 退出。")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n退出。")
    return 0


def cmd_bench(args):
    """规划性能基准（自建场景集 S1 的一部分）。"""
    from .planner import plan_joint, freespace_profiles
    model = _load_model(args)
    from .verify import check_discrete
    rng = np.random.default_rng(args.seed)
    times, oks, skipped = [], 0, 0

    def collision_free(q):
        return not check_discrete(model, [q])

    for i in range(args.n):
        start = np.array([rng.uniform(model.limit(j).get("lower", -1.0),
                                      model.limit(j).get("upper", 1.0))
                          for j in model.joint_names])
        goal = np.array([rng.uniform(model.limit(j).get("lower", -1.0),
                                     model.limit(j).get("upper", 1.0))
                         for j in model.joint_names])
        if args.rejection:
            # 拒绝采样：起止位形都必须无碰撞（否则 OMPL 的目标树/起始树根本建不出来，
            # 这是"随机目标规划失败"最常见的误判来源）
            def sample_free():
                for _ in range(60):
                    q = np.array([rng.uniform(model.limit(j).get("lower", -1.0),
                                              model.limit(j).get("upper", 1.0))
                                  for j in model.joint_names])
                    if collision_free(q):
                        return q
                return None
            goal = sample_free()
            if goal is None:
                skipped += 1
                print("  [%2d/%d] 跳过：采样不到无碰撞目标" % (i + 1, args.n))
                continue
            start = sample_free()
            if start is None:
                skipped += 1
                print("  [%2d/%d] 跳过：采样不到无碰撞起点" % (i + 1, args.n))
                continue
        out = plan_joint(model, start, goal, profiles=freespace_profiles(
            num_planners=args.num_planners, planning_time=args.planning_time))
        times.append(out.elapsed)
        oks += 1 if out.ok else 0
        print("  [%2d/%d] ok=%-5s %.3fs pts=%d" % (i + 1, args.n, out.ok, out.elapsed, len(out.points)))
    t = np.asarray(times)
    print("\n成功率 %d/%d（跳过 %d） | 延迟 p50 %.3fs p95 %.3fs max %.3fs" %
          (oks, len(times), skipped, np.percentile(t, 50) if len(t) else 0,
           np.percentile(t, 95) if len(t) else 0, t.max() if len(t) else 0))
    return 0


def cmd_repl(args):
    """交互式会话：提示符里直接 fk / ik / plan / exec。"""
    from .planner import plan_joint, freespace_profiles
    from .verify import verify
    model = _load_model(args)
    print("rmctl 会话 —— 关节(%s)" % ",".join(model.joint_names))
    print("命令: fk <q> | ik <x y z r p y> | plan <q...> | run <q...> | home | q | help | quit")
    cur = model.home_state()
    while True:
        try:
            line = input("rm> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not line:
            continue
        parts = line.split()
        cmd = parts[0]
        try:
            if cmd in ("quit", "exit", "q"):
                break
            elif cmd == "help":
                print("fk <q...> | ik <x y z r p y> | plan <q...> | run <q...> | home | quit")
            elif cmd == "home":
                cur = model.home_state()
                print(_fmt_vec(cur))
            elif cmd == "fk":
                cur = np.asarray(_parse_floats(" ".join(parts[1:])))
                T = model.fk_self(cur, args.tip or model.tip)
                print("pos=%s" % _fmt_vec(T[:3, 3], 6))
            elif cmd == "ik":
                from tesseract_robotics.planning import Pose
                vals = [float(x) for x in parts[1:]]
                sol = model.ik(Pose.from_xyz_rpy(*vals), seed=cur)
                print("IK: %s" % ("无解" if sol is None else _fmt_vec(sol)))
                if sol is not None:
                    cur = sol
            elif cmd in ("plan", "run"):
                goal = np.asarray([float(x) for x in parts[1:]])
                out = plan_joint(model, cur, goal, profiles=freespace_profiles())
                print("ok=%s %.3fs pts=%d %s" % (out.ok, out.elapsed, len(out.points),
                                                 (out.message or "")[:60]))
                if out.ok:
                    rep = verify(model, out.points, out.times)
                    print("校验: %s" % ("PASS" if rep["ok"] else "FAIL " + str(rep["issues"])))
                    if cmd == "run":
                        MockExecutor(model).run(out.times, out.points)
                        print("回放完成")
                    cur = goal
            else:
                print("未知命令: %s（help 看用法）" % cmd)
        except Exception as exc:
            print("错误: %r" % exc)
    return 0


WORKCELL_TEMPLATE = """# 双臂工位定义（rm_workcell.yaml）
# 所有可配置项集中在这里；换机型 = 换这份文件 + 重新标定，不改代码。
version: 1
robot:
  description: rm_description
  urdf: urdf/rm75_clean.urdf
  srdf: rm75.srdf
  joint_limits: config/joint_limits.yaml
  baselines_dir: baselines   # 四类真机基线目录（相对本文件所在目录）
  groups:
    - name: rm75_arm
      base: base_link
      tip: Link7
      dof: 7
execution:
  period_s: 0.010          # 10 ms 单时钟
  follow: false            # 低跟随
  trajectory_mode: 1       # 曲线拟合
  radio: 30
  pre_position_speed: 10   # rm_movej 预就位速度百分比
safety:
  collision_margin_m: 0.005
  self_collision: true
  joint_limit_margin_deg: 1.0
planning:
  num_planners: 2
  planning_time_s: 5.0
  planner_range: 0.01
gripper:                   # 待接：型号、供电、协议
  model: RMG22-65
  power: external_24v     # 法兰供电上限 1.5A < 额定 1.9A，必须外部供电
  protocol: modbus_rtu
calibration:
  tcp: null               # 未标定：不得伪造
  base_to_base: null      # 未标定：不得伪造
  # 四类真机基线（相对 workcell/ 目录；文件本体在 baselines/）
  zero_baseline: baselines/zero_baseline.yaml
  measured_limits: baselines/measured_limits.yaml
  version_matrix: baselines/version_matrix.yaml
  network_plan: baselines/network_plan.yaml
"""


def build_parser():
    p = argparse.ArgumentParser(prog="rmctl", description="双臂工程操作入口")
    p.add_argument("--ws", default=None, help="工作区根目录（默认自动发现或 $RM_WS）")
    p.add_argument("--group", default="rm75_arm")
    p.add_argument("--tip", default=None)
    sub = p.add_subparsers(dest="cmd")

    s = sub.add_parser("init", help="生成工位骨架")
    s.add_argument("--force", action="store_true")
    s.set_defaults(func=cmd_init)

    s = sub.add_parser("doctor", help="一条命令体检")
    s.set_defaults(func=cmd_doctor)

    s = sub.add_parser("baseline", help="真机基线：check / list / show / scaffold")
    s.add_argument("action", nargs="?", default="check",
                   choices=["check", "list", "show", "scaffold"],
                   help="默认 check")
    s.add_argument("kind", nargs="?", default=None,
                   help="show 用的基线类型（%s）" % "/".join(bl.KINDS))
    s.add_argument("--force", action="store_true", help="scaffold 覆盖已存在文件")
    s.set_defaults(func=cmd_baseline)

    s = sub.add_parser("fk", help="正解（自研 FK vs Tesseract 交叉校验）")
    s.add_argument("--joints", default=None)
    s.set_defaults(func=cmd_fk)

    s = sub.add_parser("ik", help="逆解")
    s.add_argument("--pose", required=True, help="x,y,z,rx,ry,rz 或 x,y,z,qx,qy,qz,qw")
    s.add_argument("--seed", default=None)
    s.set_defaults(func=cmd_ik)

    s = sub.add_parser("plan", help="规划并校验")
    s.add_argument("--start", default=None)
    s.add_argument("--goal", default=None, help="目标关节角")
    s.add_argument("--goal-pose", dest="goal_pose", default=None)
    s.add_argument("--pipeline", default="FreespacePipeline")
    s.add_argument("--num-planners", dest="num_planners", type=int, default=2)
    s.add_argument("--planning-time", dest="planning_time", type=float, default=5.0)
    s.add_argument("--out", default=None, help="保存轨迹 JSON")
    s.add_argument("--no-verify", action="store_true")
    s.add_argument("--no-continuous", action="store_true")
    s.add_argument("--no-retime", action="store_true", help="禁止自动降速重定时")
    s.set_defaults(func=cmd_plan)

    s = sub.add_parser("check", help="对轨迹文件做四道校验")
    s.add_argument("--traj", required=True)
    s.set_defaults(func=cmd_check)

    s = sub.add_parser("run", help="模拟执行回放")
    s.add_argument("--traj", required=True)
    s.add_argument("--period", type=float, default=0.010)
    s.add_argument("--fast", action="store_true", help="不按真实时钟等待")
    s.set_defaults(func=cmd_run)

    s = sub.add_parser("ui", help="浏览器可视化（并可播放轨迹）")
    s.add_argument("--traj", default=None)
    s.add_argument("--port", type=int, default=8000)
    s.add_argument("--fast", action="store_true")
    s.set_defaults(func=cmd_ui)

    s = sub.add_parser("bench", help="规划性能基准")
    s.add_argument("--n", type=int, default=20)
    s.add_argument("--seed", type=int, default=7)
    s.add_argument("--num-planners", dest="num_planners", type=int, default=2)
    s.add_argument("--planning-time", dest="planning_time", type=float, default=5.0)
    s.add_argument("--rejection", action="store_true", default=True,
                   help="拒绝采样：只测无碰撞目标（默认开）")
    s.add_argument("--no-rejection", dest="rejection", action="store_false")
    s.set_defaults(func=cmd_bench)

    s = sub.add_parser("repl", help="交互式会话")
    s.set_defaults(func=cmd_repl)
    return p


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    if not getattr(args, "func", None):
        parser.print_help()
        return 0
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
