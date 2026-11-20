#!/usr/bin/env python3
"""rmctl —— 双臂工程的操作入口（设计 §6.5）。

同一实现、两种入口：每个子命令既能一次性执行（脚本/CI），也能在 repl 里连续用（交互）。
"""
import argparse
import contextlib
import ipaddress
import json
import os
import socket
import sys
import tempfile
import time

import numpy as np
import yaml

from . import baseline as bl
from . import doctor
from . import paths
from .executor import MockExecutor
from .model import Model
from .vendor_env import VendorUnavailable

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


# ------------------------------------------------------- 硬件适配层检查项（hw:*）
#
# R010：doctor 的每条检查项都要有分级 level + **可粘贴的修复命令**。
# 本切片（S02）新增的硬件面——SDK 在位 / 22 核心符号 / 白黑名单契约 / 限值来源——
# 全部收敛到下面 5 条 hw:* 检查项；判定逻辑一律复用硬件适配层（rm_hw_bridge），
# **不在 rmctl 侧另写一套**（清单与真值都在 rm_hw_bridge 里）。

#: 硬件适配层的 5 条检查项名字（顺序固定）
HW_CHECK_NAMES = ("hw:sdk", "hw:symbols", "hw:contract", "hw:limits_source",
                  "hw:gripper")

#: 硬件适配层不可导入时的修复动作（可直接粘贴）
_HW_PYTHONPATH_ACTION = (
    "export PYTHONPATH=$PWD/dualarm_ws/src/rm_hw_bridge:$PYTHONPATH  "
    "# 让 rmctl 看到硬件适配层")


def _hw_import(ws=None):
    """惰性导入硬件适配层的三个模块（loader / audit / limits）。

    返回 ``(mods, error)``：``mods`` 为 ``None`` 时 ``error`` 说明原因
    （**不把 ImportError 堆栈直接甩给操作者**）。
    """
    loader = None
    first = ""
    try:
        from rm_hw_bridge import loader as _loader
    except Exception as exc:
        first = repr(exc)
    else:
        loader = _loader
    if loader is None:
        root = ws or paths.workspace_root()
        pkg_parent = os.path.join(root, "dualarm_ws", "src", "rm_hw_bridge")
        if os.path.isdir(pkg_parent) and pkg_parent not in sys.path:
            sys.path.insert(0, pkg_parent)
        try:
            from rm_hw_bridge import loader as _loader
        except Exception as exc:
            return None, "%s / 兜底路径 %s: %r" % (first, pkg_parent, exc)
        loader = _loader
    try:
        from rm_hw_bridge import audit as _audit
        from rm_hw_bridge import limits as _limits
    except Exception as exc:
        return None, "导入 rm_hw_bridge.audit/limits 失败: %r" % exc
    return {"loader": loader, "audit": _audit, "limits": _limits}, ""


def _hw_check(name, ok, detail, action, level="INFO"):
    """构造一条 hw:* 检查项（统一字段：name / ok / detail / action / level）。"""
    return {"name": name, "ok": bool(ok), "detail": detail,
            "action": action, "level": level}


def _hw_sdk_check(ws):
    """hw:sdk —— RM-API2 动态库是否在位（.so 路径 + 契约声明版本）。

    缺失是**环境未就位**（不是代码缺陷），因此 level=INFO（不阻塞退出码），
    但必须给出可粘贴的修复动作。
    """
    mods, err = _hw_import(ws)
    deps_so = os.path.join(ws or "", ".deps", "rm_api2", "libapi_c.so")
    action = ("export RM_API2_LIB=/abs/path/to/libapi_c.so  "
              "# 或把 SDK 物化到 %s" % deps_so)
    if mods is None:
        return _hw_check("hw:sdk", False,
                         "硬件适配层 rm_hw_bridge 不可用: %s" % err,
                         _HW_PYTHONPATH_ACTION)
    loader = mods["loader"]
    lib = loader.find_library(ws=ws)
    if not lib:
        cands = ", ".join(c for c in loader.sdk_candidates(ws=ws) if c)
        return _hw_check("hw:sdk", False,
                         "未找到 RM-API2 动态库（候选: %s）" % (cands or "无"),
                         action)
    try:
        version = str((loader.load_contract().get("sdk") or {}).get("version") or "?")
    except Exception:
        version = "?"
    return _hw_check("hw:sdk", True,
                     "RM-API2 动态库在位: %s（契约声明版本 %s）" % (lib, version),
                     "python3 -m rmctl connect --arm left --ip <ip>  "
                     "# 用该 .so 连真机")


def _hw_symbols_check(ws):
    """hw:symbols —— 22 个核心符号能否在 .so 里解析（用 rm_hw_bridge.loader 绑定）。"""
    mods, err = _hw_import(ws)
    if mods is None:
        return _hw_check("hw:symbols", False,
                         "硬件适配层 rm_hw_bridge 不可用: %s" % err,
                         _HW_PYTHONPATH_ACTION)
    loader = mods["loader"]
    lib = loader.find_library(ws=ws)
    if not lib:
        return _hw_check("hw:symbols", False,
                         "SDK 未就位，无法校验 22 个核心符号",
                         "export RM_API2_LIB=/abs/path/to/libapi_c.so  "
                         "# 先按 hw:sdk 修复 SDK")
    try:
        import ctypes

        binding = loader.SymbolBinding(ctypes.CDLL(lib))
    except Exception as exc:
        return _hw_check("hw:symbols", False,
                         "加载 RM-API2 动态库失败: %r" % exc,
                         "ldd %s  # 检查依赖库是否齐全" % lib,
                         level="FAIL")
    core = loader.core_symbols()
    support = loader.support_symbols()
    missing = set(binding.missing_symbols)
    missing_core = [n for n in core if n in missing]
    missing_support = [n for n in support if n in missing]
    if missing_core:
        return _hw_check("hw:symbols", False,
                         "22 核心符号缺 %d 个: %s"
                         % (len(missing_core), ", ".join(missing_core)),
                         "nm -D --defined-only %s | grep rm_  "
                         "# 核对 .so 导出符号（版本不匹配需换正确 SDK）" % lib,
                         level="FAIL")
    detail = "22 核心符号全部可解析（%s）；支持层 %d/%d 可解析" % (
        lib, len(support) - len(missing_support), len(support))
    if missing_support:
        detail += "；缺: %s" % ", ".join(missing_support)
    return _hw_check("hw:symbols", True, detail,
                     "python3 -m rmctl connect --arm left --ip <ip>  "
                     "# 符号齐备，可直接连真机")


def _hw_contract_check(root):
    """hw:contract —— 白名单/黑名单静态审计（跑 rm_hw_bridge.audit 的仓库扫描）。

    有违规 = **阻塞**（FAIL）：黑名单调用会绕过 R003 的厂商解耦红线。
    """
    mods, err = _hw_import(root)
    if mods is None:
        return _hw_check("hw:contract", False,
                         "静态审计不可用: %s" % err,
                         _HW_PYTHONPATH_ACTION, level="FAIL")
    audit = mods["audit"]
    try:
        report = audit.audit_repo(root=root)
    except Exception as exc:
        return _hw_check("hw:contract", False,
                         "静态审计执行失败: %r" % exc,
                         "python3 -m rm_hw_bridge.audit --repo  # 手动跑审计看具体错误",
                         level="FAIL")
    violations = list(report.get("violations") or [])
    if not violations:
        return _hw_check("hw:contract", True,
                         "白/黑名单静态审计零违规（扫描 %d 个 .py）"
                         % int(report.get("files_scanned") or 0),
                         "python3 -m rm_hw_bridge.audit --repo  # 复核（退出码 0 = 零违规）")
    items = []
    for v in violations[:5]:
        items.append("%s:%s [%s] %s" % (v.get("path"), v.get("line"),
                                         v.get("kind"), v.get("message")))
    return _hw_check("hw:contract", False,
                     "静态审计发现 %d 项违规: %s" % (len(violations), "; ".join(items)),
                     "python3 -m rm_hw_bridge.audit --repo  "
                     "# 逐条打印违规（文件:行: [kind] 说明），修复后重跑 rmctl doctor",
                     level="FAIL")


def _hw_limits_source_check(ws):
    """hw:limits_source —— 下发限值的默认值来源是不是**实测**基线。

    口径复用 rm_hw_bridge.limits.load_limit_defaults（硬件层的唯一判定口）：
    status=measured 才算就绪；blocked/placeholder 都**拒绝下发**。
    未实测只是「真机窗口还没到」，level=INFO，不阻塞退出码。
    """
    mods, err = _hw_import(ws)
    if mods is None:
        return _hw_check("hw:limits_source", False,
                         "硬件适配层 rm_hw_bridge 不可用: %s" % err,
                         _HW_PYTHONPATH_ACTION)
    limits = mods["limits"]
    try:
        rep = limits.load_limit_defaults(ws=ws)
    except Exception as exc:
        return _hw_check("hw:limits_source", False,
                         "读取限值默认来源失败: %r" % exc,
                         "python3 -m rmctl baseline check  # 先看基线齐备度")
    status = str(rep.get("status") or "unknown")
    source = str(rep.get("source") or "")
    detail = "限值默认来源 status=%s source=%s" % (status, source or "<空>")
    ok = status == "measured"
    if not ok:
        detail += "（%s）" % (rep.get("reason") or "基线未实测")
    return _hw_check("hw:limits_source", ok, detail,
                     "python3 dualarm_ws/src/rm_description/scripts/apply_measured_limits.py "
                     "--dry-run  # 真机窗口回填 measured_limits.yaml 后把 status 改为 measured")


def _hw_checks(ws):
    """5 条 hw:* 检查项（顺序固定：sdk → symbols → contract → limits_source → gripper）。"""
    return [
        _hw_sdk_check(ws),
        _hw_symbols_check(ws),
        _hw_contract_check(ws),
        _hw_limits_source_check(ws),
        _hw_gripper_check(ws),
    ]


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


def _print_doctor_text(checks, matrix):
    """doctor 的人读输出（**逐字沿用既有格式**，只多一行自检覆盖小结）。"""
    ok = sum(1 for c in checks if c["ok"])
    print("体检结果: %d/%d 通过\n" % (ok, len(checks)))
    for c in checks:
        print("  [%s] %-26s %s" % ("PASS" if c["ok"] else c["level"], c["name"], c["detail"]))
        if not c["ok"] and c["action"]:
            print("        → 下一步: %s" % c["action"])
    covered = [m for m in matrix if m.get("covered")]
    uncovered = [m for m in matrix if not m.get("covered")]
    print("\n自检覆盖: %d/%d 类现场问题%s"
          % (len(covered), len(matrix),
             ("（未覆盖: %s）" % ", ".join(str(m.get("label")) for m in uncovered))
             if uncovered else ""))
    bad = [c for c in checks if not c["ok"] and c["level"] != "INFO"]
    print("%s" % ("全部通过，可以开始规划。" if not bad else "存在 %d 项需处理。" % len(bad)))


def cmd_doctor(args):
    """一条命令体检（设计 §6.5 / §6.8）。

    输出形态：默认**人读文本**；``--json`` 时 stdout 输出**机器可读 JSON**
    （含 ``checks[]`` 与 ``coverage_matrix[]``）；``--report <path>`` 把**同一份
    JSON** 落盘（与 stdout 的 JSON 等价）。

    退出码（全项目统一 0/3/4，与 ``rm_executor.contract.EXIT_CODES`` 同口径）：

    * ``0`` —— 全部检查项 PASS（INFO 级未通过**不算**失败）；
    * ``3`` —— 存在需人工 / 外部条件的项（blocked）；
    * ``4`` —— 存在硬红线项（error）。

    检查项构造的唯一实现 = :mod:`rmctl.doctor`（本函数只是**薄封装**）。
    """
    ws = args.ws or paths.workspace_root()
    checks = doctor.collect_checks(ws, group=getattr(args, "group", "rm75_arm"))
    matrix = doctor.coverage_matrix(checks)
    report = doctor.build_report(ws, checks, matrix)

    as_json = bool(getattr(args, "json", False))
    report_path = getattr(args, "report", None)
    if as_json:
        print(json.dumps(report, ensure_ascii=False, indent=2))
    else:
        _print_doctor_text(checks, matrix)

    if report_path:
        path = os.path.abspath(report_path)
        parent = os.path.dirname(path)
        try:
            if parent:
                os.makedirs(parent, exist_ok=True)
            with open(path, "w", encoding="utf-8") as fh:
                json.dump(report, fh, ensure_ascii=False, indent=2)
        except OSError as exc:
            # 报告写不出去是**操作错误**（路径不可写 / 权限不足）→ 明确报错，
            # 不静默丢失证据（JSON 已打印到 stdout 时仍可用）。
            print("体检报告写入失败: %s (%s)" % (path, exc), file=sys.stderr)
            print("→ 修复: 确认目录可写后重跑，例如 --report build/reports/doctor.json",
                  file=sys.stderr)
            return doctor.EXIT_ERROR
        if not as_json:
            print("\n体检报告已写入: %s" % path)
    return report["exit_code"]


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
    """``rmctl plan``：规划并做四道校验。

    **两种模式**（默认行为逐字不变）：

    * **进程内直调**（默认）：规划与四道校验都在 rmctl 进程里完成
      （离线对照 / 回退用），不需要先起任何服务。
    * **规划服务**（``--via-service``）：规划交给独立的 ``rm_planner`` 服务
      （Unix socket 控制面 + 每请求独占 worker 子进程），规划进程可以
      独立崩溃 / 重启。**前置条件**：先起服务
      ``rm-planner --ws <仓库根> --socket <ws>/build/run/rm_planner.sock``。

    两种模式输出**同构**（管线 / 成功与否 / 耗时 / 点数 / 四道校验逐项
    PASS-FAIL / 失败原因）；服务模式额外打印服务端返回的 per-node 指标。

    **退出码**（两种模式一致）：

    * ``0`` —— 成功（四道校验全过）
    * ``1`` —— 规划失败
    * ``2`` —— 参数错误
    * ``3`` —— 四道校验未通过
    * ``4`` —— 规划超时（**服务端强制中断** worker，属于"需要人看"的类别，
      与 ``3``（校验未过）区分开）
    """
    if getattr(args, "via_service", False):
        return _cmd_plan_via_service(args)
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
        # 自愈阶梯第 1 级（保路径降速重定时）的**唯一实现**：
        # rmctl.verify.verify_with_retime —— 工艺客户端层（rmctl.process.clients）
        # 也调它，两边**不复制**阶梯逻辑。
        from .verify import verify_with_retime
        report, new_times, new_points, scale = verify_with_retime(
            model, out.points, out.times,
            do_continuous=not args.no_continuous, retime=not args.no_retime)
        if scale:
            print("\n自动降速重定时 ×%.3f（路径不变）→ 四道校验全部通过" % scale)
            out.times, out.points = new_times, new_points
        print("\n四道校验: %s" % ("全部通过" if report["ok"] else "未通过"))
        for name, res in report["checks"].items():
            print("  [%s] %s" % ("PASS" if res.get("ok") else "FAIL", name))
            for issue in res.get("issues", [])[:5]:
                print("        %s" % issue)
    if args.out:
        _save_traj(args.out, out, model, {"verify": report})
        print("\n轨迹已保存: %s" % args.out)
    return 0 if (report is None or report["ok"]) else 3


# --------------------------------------------------------------------------
# `rmctl plan --via-service`：走 rm_planner 规划服务（薄客户端）
# --------------------------------------------------------------------------
#
# 两种模式的输出**同构**：管线 / 成功与否 / 耗时 / 点数 / 四道校验逐项
# PASS-FAIL / 失败原因 —— 服务模式额外打印服务端返回的 per-node 指标
# （各节点 name + 耗时），便于运维诊断。
#
# 退出码沿用既有约定：SUCCESS=0、FAILED=1、参数错误=2、校验未过=3、
# TIMEOUT=4（超时是"需要人看"的类别，与校验未过的 3 区分开）。

#: 服务模式下四道校验的**逐项**结论（与进程内模式的 ``report["checks"]``
#: 键名一致 → 输出同构）。
_PLAN_CHECK_NAMES = ("limits", "discrete_collision", "continuous_collision")


def _service_goal(args):
    """把 CLI 目标参数翻译成契约层的 ``goal`` 字典。

    :return: ``{"type": ..., "values": [...]}``；缺目标 → ``None``
    """
    if getattr(args, "goal_pose", None):
        vals = _parse_floats(args.goal_pose)
        if vals is None or len(vals) not in (6, 7):
            return None
        return {"type": "pose", "values": [float(v) for v in vals]}
    goal = _parse_floats(args.goal)
    if goal is None:
        return None
    return {"type": "joint", "values": [float(v) for v in goal]}


def _service_exit_code(response):
    """服务模式响应 → 退出码（见 ``cmd_plan`` 的 docstring）。"""
    status = str(response.get("status") or "")
    if status == "SUCCESS":
        return 0
    if status == "INVALID_REQUEST":
        return 2
    if status == "TIMEOUT":
        return 4
    if status == "CANCELLED":
        return 1
    verification = response.get("verification")
    verification = verification if isinstance(verification, dict) else {}
    issues = list(verification.get("issues") or [])
    reason = str(response.get("failure_reason") or "")
    if issues or reason.startswith("四道校验未通过"):
        return 3
    return 1


def _print_service_nodes(metrics):
    """打印服务端返回的 per-node 指标（name + 耗时 + 状态）。"""
    nodes = metrics.get("nodes")
    if not isinstance(nodes, list) or not nodes:
        return
    print("\n节点耗时:")
    for node in nodes:
        if not isinstance(node, dict):
            continue
        name = str(node.get("name") or "")
        elapsed = float(node.get("elapsed_s") or 0.0)
        status = str(node.get("status") or "")
        message = str(node.get("message") or "")
        print("  %-34s %8.3fs  %s" % (name, elapsed, status))
        if message and status != "SUCCESS":
            print("        %s" % message)


def _cmd_plan_via_service(args):
    """``rmctl plan --via-service``：把规划交给 rm_planner 服务。

    客户端只做三件事：**翻译请求**（CLI 参数 → 契约层字典）、**发一行 JSON**、
    **渲染响应**。规划本身在服务端进程里跑（可独立崩溃 / 重启）。
    """
    from .planner_service.client import PlannerClient
    from .planner_service.contract import JOINT_COUNT
    from .planner_service.config import load_profiles_config

    ws = getattr(args, "ws", None)
    goal = _service_goal(args)
    if goal is None:
        print("需要 --goal（关节角，逗号分隔）或 --goal-pose")
        return 2
    start = _parse_floats(getattr(args, "start", None))
    if start is None:
        # 缺省起始位形 = 零位（与进程内模式的 model.home_state() 一致；
        # 关节数取契约层 JOINT_COUNT → 客户端**不需要**加载厂商内核）。
        start = [0.0] * JOINT_COUNT

    # timeout_ms / scene_version 的**唯一来源**：profiles.yaml。
    profiles = load_profiles_config(ws)
    spec = profiles.get("spec") or {}
    timeout_ms = getattr(args, "timeout_ms", None) or spec.get("timeout_ms")
    if not timeout_ms:
        print("规划服务请求缺少 timeout_ms（profiles.yaml 不可用: %s）"
              % (profiles.get("reason") or "未知"))
        return 2
    scene_version = spec.get("scene_version", 1)

    client = PlannerClient(
        socket_path=getattr(args, "socket", None),
        ws=ws,
        timeout_ms=int(timeout_ms),
    )
    request = {
        "profile": "FREESPACE",
        "group": getattr(args, "group", None) or "rm75_arm",
        "start_state": [float(v) for v in start],
        "goal": goal,
        "constraints": {},
        "seed": int(getattr(args, "seed", 0) or 0),
        "timeout_ms": int(timeout_ms),
        "scene_version": int(scene_version),
        "deterministic": True,
    }
    response = client.plan(request)
    return _render_service_plan(response, args)


def _render_service_plan(response, args):
    """渲染服务响应（输出与进程内模式同构）+ 返回退出码。"""
    metrics = response.get("metrics")
    metrics = metrics if isinstance(metrics, dict) else {}
    verification = response.get("verification")
    verification = verification if isinstance(verification, dict) else {}
    trajectory = response.get("trajectory")
    trajectory = trajectory if isinstance(trajectory, dict) else {}
    status = str(response.get("status") or "")
    ok = status == "SUCCESS"
    pipeline = str(metrics.get("pipeline") or "")
    elapsed = float(metrics.get("elapsed_total_s") or 0.0)
    points = trajectory.get("points") or []

    print("管线 %s | 成功=%s | 耗时 %.3fs | 点数 %d"
          % (pipeline, ok, elapsed, len(points)))
    if not ok:
        print("失败原因: %s" % (response.get("failure_reason") or "未知"))

    scale = metrics.get("retimed_scale")
    if scale:
        print("\n自动降速重定时 ×%.3f（路径不变）→ 四道校验全部通过" % float(scale))

    issues = list(verification.get("issues") or [])
    has_verification = bool(verification.get("ok")) or bool(issues)
    if has_verification:
        print("\n四道校验: %s" % ("全部通过" if verification.get("ok") else "未通过"))
        for name in _PLAN_CHECK_NAMES:
            res = verification.get(name)
            res = res if isinstance(res, dict) else {}
            print("  [%s] %s" % ("PASS" if res.get("ok") else "FAIL", name))
            for issue in (res.get("issues") or [])[:5]:
                print("        %s" % issue)

    _print_service_nodes(metrics)

    next_action = str(metrics.get("next_action") or "")
    if next_action:
        print("\n下一步: %s" % next_action)

    if ok and getattr(args, "out", None):
        _save_service_traj(args.out, response, metrics)
        print("\n轨迹已保存: %s" % args.out)

    return _service_exit_code(response)


def _save_service_traj(path, response, metrics):
    """把服务返回的轨迹落盘（形状与进程内模式的 ``--out`` 一致）。"""
    from .planner import PlanOutcome

    trajectory = response.get("trajectory") or {}
    raw_points = trajectory.get("points") or []
    times = [float(t) for t in (trajectory.get("times") or [])]
    points = []
    for index, item in enumerate(raw_points):
        if isinstance(item, dict):
            positions = item.get("positions") or item.get("q") or []
            if index < len(times):
                times[index] = float(item.get("t", times[index]))
        else:
            positions = item
        points.append([float(v) for v in positions])

    class _TrajModel(object):
        """最小模型替身（服务模式不需要加载厂商内核）。"""

        def __init__(self, group, joint_names):
            self.group = group
            self.joint_names = list(joint_names)

    model = _TrajModel(
        str(metrics.get("group") or "rm75_arm"),
        trajectory.get("joint_names") or [],
    )
    outcome = PlanOutcome(
        True,
        "",
        times,
        points,
        float(metrics.get("elapsed_total_s") or 0.0),
        str(metrics.get("pipeline") or ""),
    )
    _save_traj(path, outcome, model, {"verify": response.get("verification")})
    return path


def cmd_check(args):
    from .verify import verify
    model = _load_model(args)
    data = _load_traj(args.traj)
    report = verify(model, data["points"], data["times"])
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["ok"] else 3


def _run_with_rm_executor(args, live=False):
    """走**新执行器**（rm_executor）：离线 fake 或真机 live。

    与 rm_executor 的 cli 是**同一份代码路径**（不复制实现）：只做参数转译。
    """
    try:
        from rm_executor import cli as exec_cli
    except ImportError as exc:
        print("执行层 rm_executor 不可用: %s" % exc, file=sys.stderr)
        print("→ 下一步: export PYTHONPATH=$PWD/dualarm_ws/src/rm_executor:$PYTHONPATH",
              file=sys.stderr)
        return 3
    argv = ["--traj", args.traj, "--mode", "live" if live else "fake"]
    ws = getattr(args, "ws", None)
    if ws:
        argv += ["--ws", ws]
    arm = getattr(args, "arm", None) or "left"
    argv += ["--arm", arm]
    if getattr(args, "report", None):
        argv += ["--report", args.report]
    if live:
        ip = getattr(args, "ip", None)
        port = getattr(args, "port", None)
        udp_port = getattr(args, "udp_port", None)
        plan = _network_plan(ws or paths.workspace_root())
        arm_plan = ((plan or {}).get("arms") or {}).get(arm) or {}
        ip = ip or arm_plan.get("ip")
        port = port or arm_plan.get("tcp_port")
        udp_port = udp_port or arm_plan.get("udp_port")
        if ip:
            argv += ["--ip", str(ip)]
        if port:
            argv += ["--port", str(int(port))]
        if udp_port:
            argv += ["--udp-port", str(int(udp_port))]
    return exec_cli.main(argv)


# ---------------------------------------------------------------- 工艺路径
#: 当前支持的声明式工艺任务（`rmctl run <task>` 的**唯一**取值集合）
PROCESS_TASKS = ("pick_place",)

#: 工艺循环次数上限（>50 必须人工检查；与 runner 的上限一致）
PROCESS_MAX_ITERATIONS = 50

#: 任务规格默认路径（相对工作区根）
PROCESS_DEFAULT_TASK_FILE = os.path.join("workcell", "tasks", "pick_place.yaml")

#: trace 默认落盘路径（相对工作区根）
PROCESS_DEFAULT_TRACE = os.path.join("build", "run", "pick_place_trace.jsonl")

#: 离线口径声明（**必须**出现在离线模式的所有输出里）
PROCESS_FAKE_NOTE = "离线代理，不代表真机结论"

#: 工作区同层包（工艺路径的依赖：执行层 / 硬件层 / 安全层）
PROCESS_WS_PACKAGES = ("rm_executor", "rm_hw_bridge", "rm_safety")


def _process_root(args):
    """工艺路径的工作区根（``--ws`` 或自动发现）。"""
    return getattr(args, "ws", None) or paths.workspace_root()


def _process_path(root, path):
    """相对路径 → 工作区根下的绝对路径（绝对路径原样保留）。"""
    if os.path.isabs(path):
        return path
    return os.path.join(root, path)


def _ensure_ws_packages(root):
    """把同层的执行层/硬件层/安全层加入 ``sys.path``（只读引用，不拷贝）。

    与 ``scripts/rmenv.sh`` 同源：让 ``rmctl run pick_place`` 在没有手动
    export PYTHONPATH 时也能看到依赖包；**已存在则不重复添加**。
    """
    added = []
    for name in PROCESS_WS_PACKAGES:
        pkg_dir = os.path.join(root, "dualarm_ws", "src", name)
        if os.path.isdir(pkg_dir) and pkg_dir not in sys.path:
            sys.path.append(pkg_dir)
            added.append(pkg_dir)
    return added


def _process_arg_error(reason, next_action):
    """参数互斥 / 越界 → ``(2, reason, next_action)``（可粘贴提示）。"""
    return 2, reason, next_action


def _validate_run_args(args):
    """``rmctl run`` 的参数互斥 / 边界校验（**先校验再分发**）。

    返回 ``(rc, reason, next_action)``；``rc == 0`` 表示可以分发。
    """
    task = getattr(args, "task", None)
    traj = getattr(args, "traj", None)
    replay = getattr(args, "replay", None)
    count = getattr(args, "count", None)
    task_file = getattr(args, "task_file", None)
    trace = getattr(args, "trace", None)
    live = bool(getattr(args, "live", False))
    report = getattr(args, "report", None)

    if task is not None and task not in PROCESS_TASKS:
        return _process_arg_error(
            "未知工艺任务: %r（当前只支持 %s）"
            % (task, ", ".join(PROCESS_TASKS)),
            "改成 `rmctl run %s` 后重跑" % PROCESS_TASKS[0])
    if count is not None and (count < 1 or count > PROCESS_MAX_ITERATIONS):
        return _process_arg_error(
            "--count 越界: %d（允许 1..%d 圈；超过 %d 圈必须人工检查）"
            % (count, PROCESS_MAX_ITERATIONS, PROCESS_MAX_ITERATIONS),
            "把 --count 改成 1..%d 后重跑" % PROCESS_MAX_ITERATIONS)
    if replay is not None:
        conflicts = []
        if traj is not None:
            conflicts.append("--traj")
        if live:
            conflicts.append("--live")
        if count is not None:
            conflicts.append("--count")
        if task_file is not None:
            conflicts.append("--task-file")
        if trace is not None:
            conflicts.append("--trace")
        if report is not None:
            conflicts.append("--report")
        if conflicts:
            return _process_arg_error(
                "--replay 与 %s 互斥（回放只读，绝不下发运动）"
                % ", ".join(conflicts),
                "只保留 `--replay <trace.jsonl>` 后重跑")
    if task is not None and traj is not None:
        return _process_arg_error(
            "--traj 与位置参数 %s 互斥（一条命令只跑一种东西）" % task,
            "要么 `rmctl run --traj <path>`，要么 `rmctl run %s`" % task)
    if task is None and traj is None and replay is None and not live:
        return _process_arg_error(
            "缺少执行目标（既没有 --traj，也没有工艺任务）",
            "执行一条轨迹：`rmctl run --traj <path>`；"
            "执行一条取放工艺：`rmctl run %s`" % PROCESS_TASKS[0])
    return 0, "", ""


def _process_thresholds(ws):
    """判定阈值 + 恢复阶梯上限（**唯一来源链**）。

    * 判定阈值四键 → ``rmctl.gripper_proxy.effective_thresholds``
      （工位配置优先，缺失项用代理值补齐并标注来源）；
    * 恢复上限 → ``workcell/rm_workcell.yaml::gripper.max_regrasp`` +
      ``rm_hw_bridge.gripper_state`` 的 stall/offline 默认上限。

    :return: ``(thresholds, proxy_used_keys, source, config_result)``；
        ``thresholds is None`` → 工位配置不可用（调用方按 blocked 处理）。
    """
    from .gripper_proxy import load_effective_thresholds

    thresholds, proxy_used, source, cfg_result = load_effective_thresholds(ws)
    if thresholds is None:
        return None, proxy_used, source, cfg_result
    config = cfg_result.get("config") or {}
    try:
        from rm_hw_bridge import gripper_state as state_mod

        thresholds["max_regrasp"] = int(config.get("max_regrasp") or 0)
        thresholds["max_stall_retry"] = int(state_mod.DEFAULT_MAX_STALL_RETRY)
        thresholds["max_offline_retry"] = int(state_mod.DEFAULT_MAX_OFFLINE_RETRY)
    except Exception as exc:  # noqa: BLE001 - 恢复上限取不到 → 证据留痕
        evidence = cfg_result.setdefault("evidence", [])
        evidence.append("恢复阶梯上限注入失败: %r" % exc)
    return thresholds, proxy_used, source, cfg_result


def _process_exit_code(report):
    """循环级报告 → 进程退出码（沿用 ``rm_executor.contract.EXIT_CODES``）。"""
    code = report.get("exit_code")
    if isinstance(code, int) and not isinstance(code, bool):
        return int(code)
    status = str(report.get("status") or "")
    try:
        from rm_executor import contract as contract_mod

        return int(contract_mod.EXIT_CODES.get(status, 3))
    except Exception:  # noqa: BLE001 - 执行层不可用 → 保守 blocked
        return 3


def _write_json_report(path, payload):
    """报告 JSON 落盘（父目录自动创建）。"""
    parent = os.path.dirname(os.path.abspath(path))
    if parent and not os.path.isdir(parent):
        os.makedirs(parent, exist_ok=True)
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(payload, fh, ensure_ascii=False, indent=2)
    return path


def _cmd_run_replay(args, path):
    """回放模式：只读 trace → 逐事件时间线（**绝不下发运动**）。"""
    root = _process_root(args)
    trace_path = _process_path(root, path)
    if not os.path.isfile(trace_path):
        print("【回放】找不到 trace 文件: %s" % trace_path)
        print("        下一步：先跑一次 `rmctl run %s` 生成 trace 后再回放"
              % PROCESS_TASKS[0])
        return 3
    from .process.trace import replay_trace

    try:
        result = replay_trace(trace_path)
    except Exception as exc:  # noqa: BLE001 - 坏文件 → 诚实降级
        print("【回放】回放失败: %s: %r" % (type(exc).__name__, exc))
        print("        下一步：检查 trace 文件是否损坏后重跑")
        return 3

    print("【回放】%s（只读回放：零运动 / 零 SDK / 零网络）" % trace_path)
    for line in result.get("lines") or []:
        print("  %s" % line)
    summary = result.get("summary") or {}
    print("")
    print("  事件数=%s  失败事件=%s  已解析=%s  跳过=%s"
          % (summary.get("events"), summary.get("failures"),
             summary.get("parsed"), summary.get("skipped")))
    last_action = str(summary.get("last_next_action") or "")
    if last_action:
        print("  下一步（最后一条非空）: %s" % last_action)
    return 0


def _cmd_run_process(args, task):
    """工艺路径：声明式规格 → 真实规划 → 执行 → 判定 → trace → 报告。

    CLI 只做**薄分发**：状态机在 ``rmctl.process.runner``、判定在
    ``rm_hw_bridge.gripper_state``、trace 在 ``rmctl.process.trace``，
    本函数**不复制**任何一份。
    """
    root = _process_root(args)
    _ensure_ws_packages(root)

    from .process.spec import load_task_spec

    task_file = getattr(args, "task_file", None) or PROCESS_DEFAULT_TASK_FILE
    task_path = _process_path(root, task_file)
    spec_result = load_task_spec(path=task_path, ws=root)
    if spec_result.get("status") != "ok":
        print("【工艺】任务规格不可用：%s" % spec_result.get("reason"))
        print("        下一步：%s" % spec_result.get("next_action"))
        return 3
    spec = spec_result["spec"]

    thresholds, proxy_used, source, cfg_result = _process_thresholds(root)
    if thresholds is None:
        print("【工艺】夹爪配置不可用：%s"
              % (cfg_result.get("reason") or "未知原因"))
        print("        下一步：%s" % (cfg_result.get("next_action") or ""))
        return 3

    try:
        from .process import clients as clients_mod
        from .process.runner import ProcessRunner
        from .process.trace import TraceWriter
        from rm_hw_bridge import gripper_state
    except Exception as exc:  # noqa: BLE001 - 依赖不可用 → 诚实降级
        print("【工艺】工艺层依赖不可用: %s: %r" % (type(exc).__name__, exc))
        print("        下一步：export PYTHONPATH=$PWD/dualarm_ws/src/"
              "rm_executor:$PWD/dualarm_ws/src/rm_hw_bridge:$PYTHONPATH 后重跑")
        return 3

    live = bool(getattr(args, "live", False))
    mode = "live" if live else "fake"
    arm = getattr(args, "arm", None) or "left"
    gate = gripper_state.shared_motion_gate()

    print("【工艺】%s | mode=%s | 规格=%s" % (task, mode, task_path))
    if mode == "fake":
        print("        ⚠ %s（本结果**不代表真机结论**）" % PROCESS_FAKE_NOTE)
    if proxy_used:
        print("        判定阈值来源=%s（代理补齐: %s）"
              % (source, ", ".join(proxy_used)))

    session = None
    try:
        if live:
            try:
                from rm_executor import session as session_mod
            except Exception as exc:  # noqa: BLE001
                print("【工艺】执行层 rm_executor 不可用: %s" % exc)
                print("        下一步：export PYTHONPATH=$PWD/dualarm_ws/src/"
                      "rm_executor:$PYTHONPATH 后重跑")
                return 3
            ip = getattr(args, "ip", None)
            port = getattr(args, "port", None)
            udp_port = getattr(args, "udp_port", None)
            if not ip:
                plan = _network_plan(root)
                arm_plan = ((plan or {}).get("arms") or {}).get(arm) or {}
                ip = arm_plan.get("ip")
                port = port or arm_plan.get("tcp_port")
                udp_port = udp_port or arm_plan.get("udp_port")
            session, err = session_mod.LiveSession.open(
                arm=arm, ip=ip, port=port, udp_port=udp_port, ws=root)
            if session is None:
                err = err or {}
                print("【工艺】真机会话不可用：%s"
                      % (err.get("reason") or "真机会话建立失败"))
                print("        下一步：%s" % (err.get("next_action") or ""))
                return 4
            motion = clients_mod.LiveMotionClient(session=session, ws=root, arm=arm)
            gripper = clients_mod.LiveGripperClient(
                port=session.gripper_port(), thresholds=thresholds, gate=gate)
        else:
            motion = clients_mod.FakeMotionClient(motion_gate=gate, ws=root, arm=arm)
            gripper = clients_mod.FakeGripperClient(thresholds=thresholds, gate=gate)

        trace_rel = getattr(args, "trace", None) or PROCESS_DEFAULT_TRACE
        trace_path = _process_path(root, trace_rel)
        parent = os.path.dirname(os.path.abspath(trace_path))
        if parent and not os.path.isdir(parent):
            os.makedirs(parent, exist_ok=True)
        sink = TraceWriter(trace_path)
        try:
            runner = ProcessRunner(
                spec,
                clients_mod.LivePlanner(ws=root),
                motion,
                gripper,
                trace_sink=sink,
                gate=gate,
                thresholds=thresholds,
                max_iterations=getattr(args, "count", None),
            )
            report = runner.run()
        finally:
            sink.close()
    except Exception as exc:  # noqa: BLE001 - 统一降级，不抛裸异常
        print("【工艺】工艺执行异常: %s: %r" % (type(exc).__name__, exc))
        print("        下一步：查看终端栈信息后重跑同一条命令")
        return 4
    finally:
        if session is not None:
            try:
                session.close()
            except Exception:  # pragma: no cover - 释放失败不影响主流程
                pass

    print("")
    print("【工艺】循环级报告")
    print("  status=%s  severity=%s  mode=%s"
          % (report.get("status"), report.get("severity"), mode))
    print("  圈次: %s/%s  段数: %s"
          % (report.get("iterations_completed"), report.get("iterations_planned"),
             report.get("segments_executed")))
    print("  原因: %s" % report.get("reason"))
    next_action = str(report.get("next_action") or "")
    if next_action:
        print("  下一步: %s" % next_action)
    print("  trace: %s" % (report.get("trace_path") or trace_path))
    if mode == "fake":
        print("  ⚠ %s（mode=fake；本结果**不代表真机结论**）" % PROCESS_FAKE_NOTE)

    report_out = getattr(args, "report", None)
    if report_out:
        out_path = _write_json_report(_process_path(root, report_out), report)
        print("  报告: %s" % out_path)

    return _process_exit_code(report)


def cmd_run(args):
    """执行一条轨迹，或执行一条声明式工艺。

    三条轨迹路径（**验收口径唯一**，行为逐字不变）：

    * 默认 ``--executor legacy``：**legacy** MockExecutor 回放（保留兼容，
      **不作验收证据**）；
    * ``--executor rm_executor``：新 10 ms 执行器的**离线 fake** 路径
      （与真机同一 Executor、同一报告 schema）；
    * ``--live``：新执行器的**真机**路径。

    外加两条**工艺路径**（S06/T6）：``rmctl run pick_place``（声明式工艺
    编排）与 ``rmctl run --replay <trace>``（只读回放）。
    """
    rc, reason, next_action = _validate_run_args(args)
    if rc:
        print("【参数】%s" % reason)
        print("        下一步：%s" % next_action)
        return rc

    task = getattr(args, "task", None)
    traj = getattr(args, "traj", None)
    replay = getattr(args, "replay", None)
    live = bool(getattr(args, "live", False))

    if replay is not None:
        return _cmd_run_replay(args, replay)
    if task is not None:
        return _cmd_run_process(args, task)
    if live and traj is None:
        # `rmctl run --live`（未给 --traj）→ 真机工艺路径（默认 pick_place）：
        # 控制器不可达 → 退出码 4（与既有 live 语义一致）。
        return _cmd_run_process(args, PROCESS_TASKS[0])

    kind = getattr(args, "executor", "legacy") or "legacy"
    if getattr(args, "live", False):
        kind = "rm_executor"
    if kind == "rm_executor":
        return _run_with_rm_executor(args, live=bool(getattr(args, "live", False)))

    print("【legacy】MockExecutor 回放路径：仅供开发自测，**不作验收证据**。")
    print("        验收请用 `--executor rm_executor`（离线）或 `--live`（真机）。")
    try:
        model = _load_model(args)
    except VendorUnavailable as exc:
        # 厂商规划内核不可用（裸环境缺 GLIBCXX_3.4.31+）：
        # 诚实降级 —— 退出码 3 + reason + 可粘贴 next_action，
        # 绝不抛裸异常、绝不伪造回放统计。
        print("【legacy】无法加载模型：%s" % exc)
        print("        下一步：python3 -m rmctl init   "
              "# 物化 build/model 绝对路径模型（mesh 相对路径解析失败时）")
        print("        下一步：source scripts/rmenv.sh   "
              "# 提供 GLIBCXX_3.4.31+ 的 libstdc++ 后重试")
        return 3
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


# ---------------------------------------------------------------- connect
def _import_rm_hw_bridge(ws=None):
    """惰性导入 ``rm_hw_bridge``（硬件适配层），失败时按仓库布局兜底。

    返回 ``(module, error_reason)``：``module`` 为 ``None`` 时 ``error_reason``
    说明为什么不可用（**不允许把 ImportError 堆栈直接甩给操作者**）。

    兜底顺序：
    1. ``import rm_hw_bridge``（已在 PYTHONPATH 里：``scripts/rmenv.sh`` 已加）；
    2. 把 ``<repo>/dualarm_ws/src/rm_hw_bridge`` 插入 ``sys.path`` 再试。
    """
    try:
        import rm_hw_bridge  # noqa: F401
        import rm_hw_bridge.bridge  # noqa: F401
        return rm_hw_bridge, ""
    except Exception as exc:
        first = repr(exc)
    root = ws or paths.workspace_root()
    pkg_parent = os.path.join(root, "dualarm_ws", "src", "rm_hw_bridge")
    if os.path.isdir(pkg_parent) and pkg_parent not in sys.path:
        sys.path.insert(0, pkg_parent)
        try:
            import rm_hw_bridge  # noqa: F401
            import rm_hw_bridge.bridge  # noqa: F401
            return rm_hw_bridge, ""
        except Exception as exc:
            first = "%s / 兜底路径 %s: %r" % (first, pkg_parent, exc)
    return None, first


@contextlib.contextmanager
def _quiet_stdout_fd():
    """把 **fd 1** 重定向到临时文件。

    RM-API2 的噪声是 **C 层 printf**（``[rm_create_robot_arm] socket connect err!``），
    Python 层的 ``contextlib.redirect_stdout`` 拦不住；只有重定向文件描述符才行。
    ``rmctl connect --json`` 的输出必须是**单个可 json.loads 的对象**，
    任何一行 SDK 噪声都会让它解析失败。
    """
    sys.stdout.flush()
    saved = os.dup(1)
    tmp = tempfile.TemporaryFile()
    try:
        os.dup2(tmp.fileno(), 1)
        yield tmp
    finally:
        sys.stdout.flush()
        os.dup2(saved, 1)
        os.close(saved)


def _valid_ip(ip):
    try:
        ipaddress.ip_address(str(ip))
    except ValueError:
        return False
    return True


def _network_plan(ws):
    """读网络规划基线（缺失 → ``None``）。只读 yaml，不做校验。"""
    path = os.path.join(paths.baselines_dir(ws), "network_plan.yaml")
    if not os.path.exists(path):
        return None
    try:
        with open(path, "r", encoding="utf-8") as fh:
            return yaml.safe_load(fh) or {}
    except Exception:
        return None


def cmd_connect(args):
    """`rmctl connect`：连真机并输出可信状态快照（R004 全字段）。

    退出码：0 成功 / 2 输入非法 / 3 blocked（缺 SDK 等）/ 4 error（臂不可达等）。
    """
    ws = args.ws or paths.workspace_root()
    arm = getattr(args, "arm", "left") or "left"
    plan = _network_plan(ws)
    arms = (plan or {}).get("arms") or {}
    arm_plan = arms.get(arm) or {}

    # ---- 参数解析与校验（非法 → 2）
    ip = getattr(args, "ip", None) or arm_plan.get("ip")
    port = getattr(args, "port", None)
    port = int(port) if port else int(arm_plan.get("tcp_port") or 8080)
    udp_port = getattr(args, "udp_port", None)
    udp_port = int(udp_port) if udp_port else int(arm_plan.get("udp_port") or 0)
    period_ms = int(((plan or {}).get("realtime_report") or {}).get("period_ms") or 5)

    invalid = []
    if not ip:
        invalid.append("--ip 未指定，且 network_plan.yaml 里没有 arms.%s.ip" % arm)
    elif not _valid_ip(ip):
        invalid.append("非法 IP: %r" % ip)
    if not (0 < port < 65536):
        invalid.append("非法端口: %r" % port)
    if udp_port and not (0 < udp_port < 65536):
        invalid.append("非法 UDP 端口: %r" % udp_port)
    if invalid:
        for msg in invalid:
            print("参数非法: %s" % msg, file=sys.stderr)
        print("→ 修复: rmctl connect --arm %s --ip <ip> --port 8080 --udp-port <port>"
              % arm, file=sys.stderr)
        return 2

    ip_source = "参数 --ip" if getattr(args, "ip", None) else (
        "workcell/baselines/network_plan.yaml（arms.%s.ip，status=%s，**未实测连通**）"
        % (arm, (plan or {}).get("status") or "unknown"))
    baseline_status = (plan or {}).get("status") or "unknown"

    # ---- 惰性导入硬件层（不可用 → 3，可粘贴修复动作）
    hw, import_error = _import_rm_hw_bridge(ws)
    if hw is None:
        payload = {
            "status": "blocked",
            "reason": "硬件适配层 rm_hw_bridge 不可用：%s" % import_error,
            "next_action": (
                "把硬件适配层加入 PYTHONPATH 后重跑："
                "export PYTHONPATH=%s:$PYTHONPATH" % os.path.join(
                    ws, "dualarm_ws", "src", "rm_hw_bridge")),
            "evidence": [import_error],
            "mode": "connect",
            "arm": arm,
            "ip": ip,
            "port": port,
            "udp_port": udp_port,
            "ip_source": ip_source,
            "baseline_status": baseline_status,
        }
        if getattr(args, "json", False):
            print(json.dumps(payload, ensure_ascii=False, indent=2))
        else:
            print("被阻塞: %s" % payload["reason"], file=sys.stderr)
            print("→ 下一步: %s" % payload["next_action"], file=sys.stderr)
        return 3

    # ---- dry-run：走通参数/基线/路径解析，但**不连真机**
    if getattr(args, "dry_run", False):
        lib_path = hw.loader.find_library(ws=ws)
        payload = {
            "status": "ok",
            "reason": "dry-run：已解析连接参数与基线，**未连接真机**（无任何实测数值）",
            "next_action": "去掉 --dry-run 即尝试真实连接（真机窗口内执行）",
            "evidence": [
                "sdk: %s" % (lib_path or "未找到 RM-API2 动态库（真机路径会 blocked）"),
                "network_plan.status=%s" % baseline_status,
            ],
            "mode": "dry_run",
            "arm": arm,
            "ip": ip,
            "port": port,
            "udp_port": udp_port,
            "realtime_period_ms": period_ms,
            "ip_source": ip_source,
            "baseline_status": baseline_status,
        }
        if getattr(args, "json", False):
            print(json.dumps(payload, ensure_ascii=False, indent=2))
        else:
            print("dry-run（未连真机）")
            print("  臂      : %s" % arm)
            print("  控制器  : %s:%d（TCP） / UDP %d" % (ip, port, udp_port))
            print("  上报周期: %d ms" % period_ms)
            print("  IP 来源 : %s" % ip_source)
            print("  基线    : status=%s" % baseline_status)
            print("  SDK     : %s" % (lib_path or "未找到"))
            print("→ 下一步: rmctl connect --arm %s --json" % arm)
        return 0

    # ---- 真机路径（SDK 的 C 层噪声必须隔离在 fd 1 之外）
    with _quiet_stdout_fd():
        link = hw.bridge.connect(
            ip=ip, port=port, udp_port=udp_port or None,
            enable_realtime=True, ws=ws, arm=arm,
            realtime_period_ms=period_ms,
            realtime_target_port=udp_port or None,
        )
        result = link.to_dict()
        if link.status == "ok":
            snap = link.snapshot()
            result = dict(result)
            result.update({
                "status": snap["status"],
                "reason": snap["reason"],
                "next_action": snap["next_action"],
                "evidence": list(result.get("evidence") or []) + list(snap.get("evidence") or []),
            })
            if snap.get("snapshot") is not None:
                result["snapshot"] = snap["snapshot"]
            else:
                result["snapshot"] = None
        link.close()

    result.update({
        "mode": "connect",
        "arm": arm,
        "ip": ip,
        "port": port,
        "udp_port": udp_port,
        "realtime_period_ms": period_ms,
        "ip_source": ip_source,
        "baseline_status": baseline_status,
    })
    result.setdefault("snapshot", None)
    code = {"ok": 0, "blocked": 3, "error": 4}.get(result.get("status"), 4)

    if getattr(args, "json", False):
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return code

    if code == 0:
        snap = result.get("snapshot") or {}
        print("已连接 %s:%s（%s）" % (ip, port, arm))
        print("  IP 来源 : %s" % ip_source)
        print("  关节角  : %s" % _fmt_vec(snap.get("joint_deg") or [], 3))
        print("  电流mA  : %s" % _fmt_vec(snap.get("joint_current_mA") or [], 1))
        print("  温度℃   : %s" % _fmt_vec(snap.get("joint_temperature_c") or [], 1))
        print("  电压V   : %s" % _fmt_vec(snap.get("joint_voltage_v") or [], 2))
        print("  使能位  : %s" % (snap.get("joint_en_flag")))
        print("  错误码  : %s" % (snap.get("joint_err_code")))
        print("  臂错误  : %s" % (snap.get("arm_err")))
        print("  抱闸    : %s" % ((snap.get("brake_state") or {}).get("brake_open")))
        print("  规划类型: %s (%s)" % (snap.get("controller_trajectory_type"),
                                      snap.get("controller_trajectory_type_name")))
        print("  控制器态: %s (%s)" % (snap.get("arm_current_status"),
                                      snap.get("arm_current_status_name")))
        print("  实时上报: %s" % (snap.get("realtime")))
        print("  控制器位姿: %s（scope=%s，仅供 FK 交叉校验）"
              % (snap.get("controller_pose"), snap.get("controller_pose_scope")))
        return 0
    print("连接失败[%s]: %s" % (result.get("status"), result.get("reason")), file=sys.stderr)
    for ev in result.get("evidence") or []:
        print("  · %s" % ev, file=sys.stderr)
    print("→ 下一步: %s" % result.get("next_action"), file=sys.stderr)
    return code


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
  error_codes: safety/error_codes.yaml   # 声明式错误码表（相对 workcell/ 目录）
  l1_speed_scale: 0.5                    # L1 降速比例
  enable_l1_slowdown: true               # 是否启用 L1 降速
  enable_l2_pause_on_stream_lag: false   # 断流告警区间是否升到 L2 暂停（默认只告警）
  sdk_timeout_s: 1.0                     # SDK 调用超时（秒）
planning:
  num_planners: 2
  planning_time_s: 5.0
  planner_range: 0.01
gripper:                   # 夹爪唯一配置来源（换机型 = 换这份文件 + 重新标定，不改代码）
  model: RMG22-65
  power: external_24v     # 法兰供电上限 1.5A < 额定 1.9A，必须外部供电
  protocol: modbus_rtu    # 线路协议：modbus_rtu（RTU 帧）/ rm_plus（官方 RM Plus）/ gripper_api（官方夹爪快路径）
  path: modbus            # 驱动路径：modbus | gripper_api | rm_plus
  device_id: 1            # Modbus 从站地址（1 起）
  baud: 115200            # Modbus 波特率
  timeout_ms: 500         # 单次寄存器读写超时（毫秒）
  poll_interval_ms: 50    # 状态轮询周期（毫秒）
  action_timeout_ms: 2000 # 一次开/合动作的等待上限（毫秒）
  max_regrasp: 2          # 夹空/堵转后的最大重夹次数
  # ---- 标定类字段：真机实测之前一律留 null（不得写入猜测值）----
  position_mm_per_unit: null   # 位置换算系数 mm/unit（来源：厂商手册标度 + 真机实测回填）
  force_scale: null            # 力度换算系数 N/源力度单位（来源：厂商手册标度 + 真机实测回填）
  force_threshold: null        # 夹到判定阈值（源力度单位，来源：真机实测回填）
  grasp_position_window: null  # 夹持位置窗口 [lo, hi]（mm，来源：真机实测回填）
calibration:
  # ---- 标定偏置的结构化 schema（详见 rmctl.calibration.OFFSET_FIELDS）----
  # 两处**未标定时一律写 null**，禁止伪造数值。字段口径（坐标系不可互换）：
  #
  #   frame:  "tip_link7"（tcp 专用）| "base"（base_to_base 专用）
  #   x/y/z:  平移（m）
  #   rpy:    [rx, ry, rz]（rad，Rz*Ry*Rx）
  #   method: 标定方法（人类可读）
  #   source: 数据来源（工件路径 / 现场记录）
  #   captured_at: 采集时间（ISO8601）
  #   uncertainty_mm: 不确定度（mm）
  #
  # tcp = 法兰→工具偏置，在 **tip(Link7) 系**表达
  #      （控制器 TCP 保持默认法兰 → 控制器上报的是法兰系位姿）
  #      施加：T_corrected = T_link7 · T_tool（位置与姿态同时修正）
  # base_to_base = 基座/世界系刚体偏置，在 **base 系**表达
  #      施加：T_corrected = T_base · T_link7
  tcp: null               # 未标定：不得伪造
  base_to_base: null      # 未标定：不得伪造
  # 四类真机基线（相对 workcell/ 目录；文件本体在 baselines/）
  zero_baseline: baselines/zero_baseline.yaml
  measured_limits: baselines/measured_limits.yaml
  version_matrix: baselines/version_matrix.yaml
  network_plan: baselines/network_plan.yaml
"""


# ------------------------------------------------------- 安全闸门（客户端）
#
# 方向：**rmctl → rm_runtime**（rmctl 是客户端，rm_runtime 是服务端）。
# 这**不违反**「硬件层不反向依赖体验层」的红线：rmctl 依赖安全层 rm_safety，
# 而 rm_safety / rm_executor 都**不** import rmctl。


def _safety_socket_path(args):
    """闸门 socket 路径（``--socket`` 优先，否则用安全层默认路径）。"""
    path = getattr(args, "socket_path", None)
    if path:
        return os.path.abspath(os.path.expanduser(path))
    try:
        from rm_safety.gate import default_socket_path
    except Exception:  # noqa: BLE001 - 安全层不可用 → 后面报错
        return ""
    try:
        return default_socket_path(getattr(args, "ws", None))
    except Exception:  # noqa: BLE001
        return ""


def _safety_blocked(reason, next_action, evidence=None):
    """构造 blocked 结果（**带可粘贴下一步**）。"""
    return {
        "status": "blocked",
        "ok": False,
        "reason": reason,
        "next_action": next_action,
        "evidence": list(evidence or []),
    }


def _gate_client_request(path, payload, timeout=2.0):
    """向闸门发一行 JSON → 读一行 JSON 响应（失败也返回结果字典）。

    失败路径全部**诚实映射**为 blocked（带 reason + 可粘贴 next_action）：
    socket 不存在 / 无人监听 / 超时 / 返回非 JSON / 服务端拒绝。
    """
    if not path:
        return _safety_blocked(
            "无法确定安全闸门 socket 路径",
            "用 --socket <path> 显式指定安全闸门路径")
    if not hasattr(socket, "AF_UNIX"):
        return _safety_blocked(
            "当前平台不支持 Unix domain socket",
            "在 Linux 上使用 `rmctl safety stop`")
    request = (json.dumps(payload, ensure_ascii=False) + "\n").encode("utf-8")
    client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    client.settimeout(timeout)
    try:
        client.connect(path)
    except FileNotFoundError:
        client.close()
        return _safety_blocked(
            "安全闸门 socket 不存在：%s" % path,
            "先启动 rm_runtime（它会创建该 socket）："
            "python3 -m rm_executor.runtime --traj <traj.json>")
    except ConnectionRefusedError:
        client.close()
        return _safety_blocked(
            "安全闸门 socket 存在但无人监听：%s（rm_runtime 可能已退出）" % path,
            "删除残留 socket 后重试：rm -f %s" % path)
    except OSError as exc:
        client.close()
        return _safety_blocked(
            "连接安全闸门失败（%s）: %s" % (type(exc).__name__, exc),
            "确认 rm_runtime 正在运行且 socket 路径一致")
    try:
        client.sendall(request)
        data = client.recv(65536)
    except socket.timeout:
        return _safety_blocked(
            "安全闸门无响应（超时 %.1f s）" % timeout,
            "确认 rm_runtime 仍在运行；必要时用 --socket 指向正确的路径")
    except OSError as exc:
        return _safety_blocked(
            "与安全闸门通信失败（%s）: %s" % (type(exc).__name__, exc),
            "确认 rm_runtime 仍在运行")
    finally:
        try:
            client.close()
        except OSError:  # pragma: no cover
            pass
    text = data.decode("utf-8", errors="replace").strip()
    if not text:
        return _safety_blocked("安全闸门返回空响应", "确认闸门服务线程正常")
    try:
        resp = json.loads(text)
    except ValueError as exc:
        return _safety_blocked(
            "安全闸门返回非 JSON（%s）: %s" % (type(exc).__name__, exc),
            "查看闸门端日志")
    if not isinstance(resp, dict):
        return _safety_blocked(
            "安全闸门返回格式非法：%s" % type(resp).__name__, "查看闸门端日志")
    if resp.get("status") != "ok":
        return _safety_blocked(
            resp.get("reason") or "安全闸门拒绝了该请求",
            resp.get("next_action") or "按闸门协议重发请求",
            resp.get("evidence") or [])
    out = dict(resp)
    out["status"] = "ok"
    out["ok"] = True
    out.setdefault("reason", "")
    out.setdefault("next_action", "")
    out.setdefault("evidence", [])
    return out


def cmd_safety(args):
    """``rmctl safety stop|status`` —— 安全闸门客户端（stdio JSON 输出）。

    退出码沿用仓库口径：``0`` ok / ``3`` blocked（socket 不存在、超时、
    服务端拒绝、非法等级）/ ``4`` error（未预期异常）。
    """
    action = getattr(args, "action", "status")
    path = _safety_socket_path(args)
    if action == "stop":
        level = str(getattr(args, "level", None) or "L4").strip().upper()
        try:
            from rm_safety.levels import LEVEL_ORDER
        except Exception as exc:  # noqa: BLE001 - 安全层不可用 → blocked
            result = _safety_blocked(
                "安全层 rm_safety 不可用：%s: %s" % (type(exc).__name__, exc),
                "export PYTHONPATH=$PWD/dualarm_ws/src/rm_safety:$PYTHONPATH  "
                "# 让 rmctl 看到安全层（等级唯一真值）")
            print(json.dumps(result, ensure_ascii=False, indent=2))
            return 3
        if level not in LEVEL_ORDER:
            result = _safety_blocked(
                "非法安全等级 %r（只允许 %s）" % (level, ", ".join(LEVEL_ORDER)),
                "用 --level L1|L2|L3|L4 重发（缺省 L4 急停）")
            print(json.dumps(result, ensure_ascii=False, indent=2))
            return 3
        result = _gate_client_request(path, {"cmd": "stop", "level": level})
    else:
        result = _gate_client_request(path, {"cmd": "status"})
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result.get("status") == "ok" else 3


# ------------------------------------------------------- 夹爪体验层（S05/T5）
#
# 纪律（与仓库既有约定一致）：
# * 夹爪配置的**唯一来源** = workcell/rm_workcell.yaml::gripper（由硬件层读取器校验）；
# * 夹爪判定的**唯一真值** = rm_hw_bridge.gripper_state.judge（CLI 不另写一套判定）；
# * 离线一律走硬件层的 FakeGripper **代理**，输出必须带 mode=fake 与「代理模型，不代表真机结论」；
# * 真机不可达 → status=blocked + 可粘贴 next_action（退出码 3），**绝不伪造结论**；
# * 退出码沿用既有口径：0 ok / 2 参数非法 / 3 blocked / 4 error。

#: 六类判定（结果名与 rm_hw_bridge.gripper_state 的 RESULT_* 一致）
GRIPPER_FAULT_CLASSES = ("grasp_ok", "grasp_empty", "stall", "overtemp",
                         "overcurrent", "offline")

#: 六类判定的中文标签
GRIPPER_FAULT_LABELS = {
    "grasp_ok": "夹到",
    "grasp_empty": "夹空",
    "stall": "堵转",
    "overtemp": "过温",
    "overcurrent": "过流",
    "offline": "离线",
}

#: 严重度排序（取「最严重等级」用）
GRIPPER_SEVERITY_ORDER = ("OK", "RECOVERABLE", "NEEDS_PARAM", "NEEDS_HUMAN")

# 代理阈值的**唯一来源** = rmctl.gripper_proxy（见该模块 docstring）。
# 这里只做**再导出**（保持 `rmctl.cli.GRIPPER_PROXY_THRESHOLDS` 这个名字可见，
# 避免破坏既有引用），**不再持有第二份副本**。
from .gripper_proxy import (GRIPPER_PROXY_NOTE,  # noqa: E402  (再导出)
                            GRIPPER_PROXY_THRESHOLDS, effective_thresholds)


def _gripper_hw(ws=None):
    """惰性导入夹爪相关的硬件层模块（gripper / gripper_config / gripper_state / loader）。

    返回 ``(mods, error_reason)``；``mods`` 为 ``None`` 时 ``error_reason``
    说明原因（**不把 ImportError 堆栈直接甩给操作者**）。
    """
    def _import():
        from rm_hw_bridge import gripper as _gripper
        from rm_hw_bridge import gripper_config as _gripper_config
        from rm_hw_bridge import gripper_state as _gripper_state
        from rm_hw_bridge import loader as _loader
        return {"gripper": _gripper, "config": _gripper_config,
                "state": _gripper_state, "loader": _loader}

    try:
        return _import(), ""
    except Exception as exc:  # noqa: BLE001 - 统一降级
        first = repr(exc)
    root = ws or paths.workspace_root()
    pkg_parent = os.path.join(root, "dualarm_ws", "src", "rm_hw_bridge")
    if os.path.isdir(pkg_parent) and pkg_parent not in sys.path:
        sys.path.insert(0, pkg_parent)
    try:
        return _import(), ""
    except Exception as exc:  # noqa: BLE001
        return None, "%s / 兜底路径 %s: %r" % (first, pkg_parent, exc)


def _gripper_config_result(hw, ws):
    """读夹爪配置（唯一来源 = workcell/rm_workcell.yaml::gripper）。"""
    try:
        return hw["config"].load_gripper_config(ws)
    except Exception as exc:  # noqa: BLE001
        return {"status": "error",
                "reason": "读取夹爪配置失败: %r" % (exc,),
                "next_action": "检查 workcell/rm_workcell.yaml::gripper 段后重跑",
                "evidence": [repr(exc)],
                "config": None}


def _gripper_thresholds(cfg):
    """判定阈值：优先用工位配置；缺失项用**代理阈值**补齐（并标注来源）。

    **薄封装**：实现已收敛到 :mod:`rmctl.gripper_proxy`（代理阈值的唯一来源），
    本函数只做转调 —— 键集合、补齐规则与历史实现**逐字一致**，返回的仍是
    历史形态 ``(thresholds, proxy_used)``（2 元组，**签名不变**）。
    需要 ``source``（``workcell_config`` / ``proxy_default``）时直接调
    :func:`rmctl.gripper_proxy.effective_thresholds`。
    """
    thresholds, proxy_used, _source = effective_thresholds(cfg)
    return thresholds, proxy_used


def _gripper_emit(args, payload, lines=None):
    """统一输出（--json → 单个 JSON 对象；否则人读摘要）+ 退出码。"""
    if getattr(args, "json", False):
        print(json.dumps(payload, ensure_ascii=False, indent=2, default=str))
    else:
        for line in (lines or []):
            print(line)
        if payload.get("status") != "ok":
            print("[%s] %s" % (payload.get("status"), payload.get("reason")),
                  file=sys.stderr)
            print("→ 下一步: %s" % payload.get("next_action"), file=sys.stderr)
    return {"ok": 0, "blocked": 3, "error": 4}.get(payload.get("status"), 4)


def _gripper_real_adapter(hw, ws, arm, cfg):
    """真机路径：找 SDK → 连臂 → 构造夹爪适配器。

    返回 ``(adapter, handles, error_payload)``：``error_payload`` 非 ``None`` 时
    **不返回任何伪造结论**（blocked = 等真机窗口，error = 运行错误）。
    """
    loader = hw["loader"]
    lib = loader.find_library(ws=ws)
    if not lib:
        return None, None, {
            "status": "blocked",
            "reason": "未找到 RM-API2 动态库（libapi_c.so），无法连真机夹爪",
            "next_action": ("在真机窗口执行：export RM_API2_LIB=/abs/path/libapi_c.so "
                            "&& python3 -m rmctl gripper probe --arm %s" % arm),
            "evidence": ["已搜索: %s" % p for p in loader.sdk_candidates(ws=ws)],
            "mode": "live",
            "arm": arm,
        }
    plan = _network_plan(ws) or {}
    arm_plan = ((plan.get("arms") or {}).get(arm) or {})
    ip = arm_plan.get("ip")
    if not ip:
        return None, None, {
            "status": "blocked",
            "reason": "network_plan.yaml 里没有 arms.%s.ip（没有控制器地址）" % arm,
            "next_action": ("在 workcell/baselines/network_plan.yaml 补 arms.%s.ip 后重跑"
                            % arm),
            "evidence": ["network_plan.status=%s" % (plan.get("status") or "unknown")],
            "mode": "live",
            "arm": arm,
        }
    port_no = int(arm_plan.get("tcp_port") or 8080)
    try:
        bridge = loader.RmBridge(ws=ws)
    except Exception as exc:  # noqa: BLE001
        return None, None, {
            "status": "blocked",
            "reason": "加载 RM-API2 动态库失败: %r" % (exc,),
            "next_action": "确认 SDK 版本与依赖库齐全（ldd %s）后重跑" % lib,
            "evidence": [repr(exc), "lib=%s" % lib],
            "mode": "live",
            "arm": arm,
        }
    try:
        arm_obj = bridge.create(str(ip), port_no)
    except Exception as exc:  # noqa: BLE001
        try:
            bridge.close()
        except Exception:  # pragma: no cover - 释放失败不影响主流程
            pass
        return None, None, {
            "status": "error",
            "reason": "连接控制器失败（%s:%d）: %r" % (ip, port_no, exc),
            "next_action": "确认控制器已上电、IP 可达后重跑同一条命令",
            "evidence": [repr(exc), "ip=%s port=%d" % (ip, port_no)],
            "mode": "live",
            "arm": arm,
        }
    G = hw["gripper"]
    adapter = G.GripperAdapter(
        G.CtypesSdkPort(bridge.binding, arm_obj.handle),
        path=(cfg or {}).get("path") or "modbus",
        device_id=int((cfg or {}).get("device_id") or 1),
        baud=int((cfg or {}).get("baud") or 115200),
        timeout_100ms=max(1, int((cfg or {}).get("timeout_ms") or 500) // 100),
        base=1,
        position_mm_per_unit=(cfg or {}).get("position_mm_per_unit"),
        force_scale=(cfg or {}).get("force_scale"),
    )
    return adapter, (bridge, arm_obj), None


def _gripper_apply(adapter, G, path, action, speed, force, timeout_ms):
    """下发一次夹爪动作（open = 张开 / close = 夹取）。

    * ``gripper_api`` 路径 → 官方快路径（``rm_set_gripper_release`` / ``rm_set_gripper_pick``）；
    * ``modbus`` 路径 → 手册寄存器（0x0006 松开到最大开口 / 0x0009 目标开口度 + 0x000A 速度
      + 0x000B 力度）。

    返回硬件层的统一结果字典（``status`` / ``reason`` / ``next_action`` / ``evidence``）。
    """
    if path == "gripper_api":
        if action == "open":
            return adapter.gripper_release(speed, True, timeout_ms)
        return adapter.gripper_pick(speed, force, True, timeout_ms)
    if action == "open":
        return adapter.write_register(G.REG_OPEN_MAX, 1)
    for reg, value in ((G.REG_SPEED, int(speed)), (G.REG_FORCE, int(force)),
                       (G.REG_TARGET_POS, 0)):
        res = adapter.write_register(reg, value)
        if res.get("status") != "ok":
            return res
    return {"status": "ok",
            "reason": "夹爪闭合动作已下发（目标开口度=0，速度=%d，力度=%d）" % (speed, force),
            "next_action": "",
            "evidence": ["写寄存器 0x%04X=0（目标开口度）" % G.REG_TARGET_POS]}


def _gripper_fixture_thresholds(effective):
    """确保夹具能落在阈值窗口内（否则退回代理阈值并标注）。"""
    window = list(effective.get("grasp_position_window") or [])
    try:
        ppu = float(effective.get("position_mm_per_unit") or 0.0)
    except (TypeError, ValueError):
        ppu = 0.0
    if len(window) == 2 and ppu > 0:
        center = (float(window[0]) + float(window[1])) / 2.0
        raw = center / ppu
        if 0.0 <= raw <= 1000.0:
            return effective, False
    return dict(GRIPPER_PROXY_THRESHOLDS), True


def _gripper_fixture(hw, fault, thresholds):
    """构造一个能触发指定判定的假夹爪（返回 ``(port, path)``）。

    * 夹到 / 夹空 / 堵转 / 过温 / 过流 → Modbus 路径（力度 0x000E 本身即 0–1000 标度）；
    * 离线 → 官方夹爪快路径（Modbus 路径下「读不到」只能是通讯错误，**不是离线判定**）。
    """
    G = hw["gripper"]
    window = list(thresholds.get("grasp_position_window") or [10.0, 30.0])
    center = (float(window[0]) + float(window[1])) / 2.0
    ppu = float(thresholds.get("position_mm_per_unit") or 0.05)
    pos_raw = int(round(center / ppu)) if ppu else 400
    force_threshold = float(thresholds.get("force_threshold") or 100.0)
    error_bits = 0
    force_raw = min(1000, int(round(force_threshold * 2)))
    temp = 25
    path = "modbus"
    online = True
    if fault == "grasp_empty":
        force_raw = max(0, int(force_threshold // 2))
    elif fault == "stall":
        error_bits = 1
    elif fault == "overtemp":
        error_bits = 2
        temp = 85
    elif fault == "overcurrent":
        error_bits = 4
    elif fault == "offline":
        path = "gripper_api"
        online = False
    registers = {
        G.REG_ACTUAL_POS: pos_raw,
        G.REG_ACTUAL_FORCE: force_raw,
        G.REG_ACTUAL_TEMP: temp,
        G.REG_ACTUAL_CURRENT: 120,
        G.REG_ERROR: error_bits,
        G.REG_STATE: 2,
    }
    port = G.FakeGripper(base=1, registers=registers, online=online, mode=2)
    return port, path


def _gripper_judge(hw, cfg, faults, count, thresholds):
    """注入故障并用纯逻辑判定层复核（返回 ``(payload, lines)``）。"""
    G = hw["gripper"]
    S = hw["state"]
    effective, fell_back = _gripper_fixture_thresholds(thresholds)
    rows = []
    mismatches_total = 0
    for fault in faults:
        correct = 0
        mismatches = []
        worst = "OK"
        for index in range(count):
            port, path = _gripper_fixture(hw, fault, effective)
            adapter = G.GripperAdapter(
                port,
                path=path,
                device_id=int((cfg or {}).get("device_id") or 1),
                baud=int((cfg or {}).get("baud") or 115200),
                timeout_100ms=max(1, int((cfg or {}).get("timeout_ms") or 500) // 100),
                base=1,
                position_mm_per_unit=effective.get("position_mm_per_unit"),
                force_scale=effective.get("force_scale"),
            )
            state_res = adapter.read_state()
            if state_res.get("status") != "ok":
                mismatches.append({"index": index, "expected": fault,
                                   "actual": "read_error",
                                   "reason": state_res.get("reason")})
                continue
            judgment = S.judge(state_res.get("state"), effective)
            if judgment.result == fault:
                correct += 1
            else:
                mismatches.append({
                    "index": index,
                    "expected": fault,
                    "actual": judgment.result,
                    "reason": judgment.reason,
                    "evidence": judgment.evidence.get("error_flags"),
                })
            if GRIPPER_SEVERITY_ORDER.index(judgment.severity) > \
                    GRIPPER_SEVERITY_ORDER.index(worst):
                worst = judgment.severity
        mismatches_total += len(mismatches)
        rows.append({
            "fault": fault,
            "label": GRIPPER_FAULT_LABELS.get(fault, fault),
            "injected": count,
            "correct": correct,
            "accuracy": (correct / float(count)) if count else 0.0,
            "worst_severity": worst,
            "mismatches": mismatches[:10],
        })
    ok = mismatches_total == 0
    if ok:
        # 判定全部正确 → 夹爪状态**已判定** → 进程内共享闸门回到 IDLE
        # （运动互锁解除；此前夹爪动作后闸门处于 GRIPPER_UNJUDGED）
        try:
            hw["state"].shared_motion_gate().finish_action()
        except Exception:  # noqa: BLE001 - 闸门复位失败不影响判定结论
            pass
    payload = {
        "status": "ok" if ok else "error",
        "reason": ("六类判定全部正确（%d 类 × %d 次注入）" % (len(faults), count)
                   if ok else
                   "判定不符 %d 处（%d 类 × %d 次注入）" % (mismatches_total, len(faults), count)),
        "next_action": ("" if ok else
                        "对照 rm_hw_bridge.gripper_state 的判定优先级复核注入夹具与阈值后重跑"),
        "evidence": [
            "mode=fake（离线假夹爪，不代表真机结论）",
            "判定真值: rm_hw_bridge.gripper_state.judge（CLI 不另写判定）",
            "阈值来源: %s" % ("代理阈值（真机标定未回填）" if fell_back else "workcell/rm_workcell.yaml::gripper"),
            "注入: %s" % ", ".join("%s×%d" % (f, count) for f in faults),
        ],
        "mode": "fake",
        "action": "judge",
        "note": GRIPPER_PROXY_NOTE,
        "faults": rows,
        "mismatches": mismatches_total,
        "thresholds": {
            key: (list(value) if isinstance(value, tuple) else value)
            for key, value in effective.items()
        },
        "thresholds_source": "proxy_default" if fell_back else "workcell_config",
    }
    lines = ["夹爪判定自检（%s）" % GRIPPER_PROXY_NOTE]
    for row in rows:
        lines.append("  %-12s %-4s 正确 %d/%d（%.0f%%）最严重 %s" % (
            row["label"], row["fault"], row["correct"], row["injected"],
            row["accuracy"] * 100.0, row["worst_severity"]))
    lines.append("→ %s" % payload["reason"])
    return payload, lines


def _gripper_cycle(hw, cfg, count, thresholds):
    """代理模式开合 N 次，统计超时与失败（返回 ``(payload, lines)``）。"""
    G = hw["gripper"]
    path = (cfg or {}).get("path") or "modbus"
    port = G.FakeGripper(base=1, registers={
        G.REG_STATE: 2, G.REG_ACTUAL_TEMP: 25, G.REG_ACTUAL_CURRENT: 120,
    }, max_records=1000)
    adapter = G.GripperAdapter(
        port,
        path=path,
        device_id=int((cfg or {}).get("device_id") or 1),
        baud=int((cfg or {}).get("baud") or 115200),
        timeout_100ms=max(1, int((cfg or {}).get("timeout_ms") or 500) // 100),
        base=1,
        position_mm_per_unit=(cfg or {}).get("position_mm_per_unit"),
        force_scale=(cfg or {}).get("force_scale"),
    )
    timeout_ms = int((cfg or {}).get("action_timeout_ms") or 2000)
    timeouts = 0
    failures = []
    actions = 0
    opened = adapter.open_protocol()
    try:
        for index in range(count):
            for act in ("open", "close"):
                res = _gripper_apply(adapter, G, path, act, 100, 100, timeout_ms)
                actions += 1
                if res.get("status") != "ok":
                    if res.get("rc") == G.RC_RECV_TIMEOUT:
                        timeouts += 1
                    failures.append({"cycle": index, "action": act,
                                     "status": res.get("status"),
                                     "reason": res.get("reason")})
    finally:
        closed = adapter.close_protocol()
    ok = not failures
    payload = {
        "status": "ok" if ok else "error",
        "reason": ("代理模式完成 %d 次开合（%d 次动作，%d 次超时，%d 次失败）"
                   % (count, actions, timeouts, len(failures))),
        "next_action": ("在真机窗口用 rmctl gripper open / close 复核真实动作"
                        "（本次为%s）" % GRIPPER_PROXY_NOTE),
        "evidence": [
            "mode=fake（离线假夹爪，不代表真机结论）",
            "协议生命周期: %s → %s" % (opened.get("reason"), closed.get("reason")),
            "配置来源: %s" % ((cfg or {}).get("source") or "?"),
        ],
        "mode": "fake",
        "action": "cycle",
        "note": GRIPPER_PROXY_NOTE,
        "cycles": count,
        "actions": actions,
        "timeouts": timeouts,
        "failures": failures[:20],
        "failure_count": len(failures),
    }
    lines = ["夹爪开合自检（%s）" % GRIPPER_PROXY_NOTE,
             "  次数: %d（动作 %d 次）" % (count, actions),
             "  超时: %d 次" % timeouts,
             "  失败: %d 次" % len(failures)]
    return payload, lines


def cmd_gripper(args):
    """`rmctl gripper`：夹爪体验层（probe / open / close / cycle / judge）。

    退出码：0 ok / 2 参数非法 / 3 blocked（缺配置、缺 SDK、需真机窗口）/ 4 error。
    """
    action = getattr(args, "action", None)
    if action not in ("probe", "open", "close", "cycle", "judge"):
        print("rmctl gripper 需要一个子命令：probe / open / close / cycle / judge",
              file=sys.stderr)
        return 2
    ws = getattr(args, "ws", None) or paths.workspace_root()
    arm = getattr(args, "arm", "left") or "left"

    hw, err = _gripper_hw(ws)
    if hw is None:
        payload = {
            "status": "blocked",
            "reason": "硬件适配层 rm_hw_bridge 不可用: %s" % err,
            "next_action": ("export PYTHONPATH=%s:$PYTHONPATH  "
                            "# 让 rmctl 看到硬件适配层"
                            % os.path.join(ws, "dualarm_ws", "src", "rm_hw_bridge")),
            "evidence": [err],
            "mode": "gripper",
            "action": action,
            "arm": arm,
        }
        return _gripper_emit(args, payload, [])

    cfg_res = _gripper_config_result(hw, ws)
    if cfg_res.get("status") != "ok":
        payload = {
            "status": "blocked",
            "reason": cfg_res.get("reason") or "夹爪配置不可用",
            "next_action": cfg_res.get("next_action") or (
                "补齐 workcell/rm_workcell.yaml::gripper 段后重跑"),
            "evidence": list(cfg_res.get("evidence") or []),
            "mode": "gripper",
            "action": action,
            "arm": arm,
        }
        return _gripper_emit(args, payload, [])
    cfg = cfg_res.get("config") or {}
    thresholds, proxy_used = _gripper_thresholds(cfg)

    # ---- 参数校验（非法 → 2） ----
    if action in ("open", "close"):
        speed = int(getattr(args, "speed", 100))
        force = int(getattr(args, "force", 100))
        invalid = []
        if not (0 <= speed <= 1000):
            invalid.append("--speed 必须在 0–1000 之间（收到 %d）" % speed)
        if not (0 <= force <= 1000):
            invalid.append("--force 必须在 0–1000 之间（收到 %d）" % force)
        if invalid:
            for msg in invalid:
                print("参数非法: %s" % msg, file=sys.stderr)
            print("→ 修复: rmctl gripper %s --arm %s --speed 100 --force 100"
                  % (action, arm), file=sys.stderr)
            return 2
    if action == "cycle":
        count = int(getattr(args, "count", 10))
        if count < 1 or count > 10000:
            print("参数非法: --count 必须在 1–10000 之间（收到 %d）" % count,
                  file=sys.stderr)
            print("→ 修复: rmctl gripper cycle --count 100", file=sys.stderr)
            return 2
    if action == "judge":
        count = int(getattr(args, "count", 1))
        if count < 1 or count > 1000:
            print("参数非法: --count 必须在 1–1000 之间（收到 %d）" % count,
                  file=sys.stderr)
            print("→ 修复: rmctl gripper judge --fault each --count 10", file=sys.stderr)
            return 2

    # ------------------------------------------------------------- probe
    if action == "probe":
        G = hw["gripper"]
        if getattr(args, "dry_run", False):
            lib = None
            try:
                lib = hw["loader"].find_library(ws=ws)
            except Exception:  # noqa: BLE001 - dry-run 不因探测失败而中断
                lib = None
            payload = {
                "status": "ok",
                "reason": "dry-run：已解析夹爪配置与 SDK 路径，**未连接真机**（无任何实测数值）",
                "next_action": "去掉 --dry-run 即在真机窗口执行探针",
                "evidence": [
                    "配置来源: %s" % cfg.get("source"),
                    "protocol=%s path=%s device_id=%s baud=%s timeout_ms=%s" % (
                        cfg.get("protocol"), cfg.get("path"), cfg.get("device_id"),
                        cfg.get("baud"), cfg.get("timeout_ms")),
                    "SDK: %s" % (lib or "未找到 RM-API2 动态库（真机路径会 blocked）"),
                    "未实测假设: 寄存器地址基数、只读寄存器归属 FC03/FC04、"
                    "波特率档位映射、控制器代际",
                ],
                "mode": "dry_run",
                "action": "probe",
                "arm": arm,
                "config": cfg,
            }
            lines = ["夹爪探针（dry-run，未连接真机）",
                     "  配置来源: %s" % cfg.get("source"),
                     "  协议    : %s（path=%s）" % (cfg.get("protocol"), cfg.get("path")),
                     "  从站/波特: device_id=%s baud=%s" % (cfg.get("device_id"),
                                                          cfg.get("baud")),
                     "  SDK     : %s" % (lib or "未找到"),
                     "→ 下一步: rmctl gripper probe --arm %s" % arm]
            return _gripper_emit(args, payload, lines)
        adapter, handles, err_payload = _gripper_real_adapter(hw, ws, arm, cfg)
        if err_payload is not None:
            err_payload.update({"action": "probe"})
            return _gripper_emit(args, err_payload, [])
        bridge, arm_obj = handles
        try:
            probe = adapter.probe()
        except Exception as exc:  # noqa: BLE001
            payload = {"status": "error", "reason": "夹爪探针异常: %r" % (exc,),
                       "next_action": "检查夹爪接线与控制器状态后重跑同一条命令",
                       "evidence": [repr(exc)], "mode": "live",
                       "action": "probe", "arm": arm}
            return _gripper_emit(args, payload, [])
        finally:
            try:
                arm_obj.close()
                bridge.close()
            except Exception:  # pragma: no cover - 释放失败不影响结论
                pass
        payload = {
            "status": probe.get("status") or "ok",
            "reason": probe.get("reason") or "",
            "next_action": probe.get("next_action") or "",
            "evidence": list(probe.get("evidence") or []),
            "mode": "live",
            "action": "probe",
            "arm": arm,
            "probe": probe,
            "config": cfg,
        }
        lines = ["夹爪探针（真机）",
                 "  协议状态: rm_plus=%s modbus=%s" % (
                     (probe.get("rm_plus") or {}).get("protocol_state"),
                     (probe.get("modbus_open") or {}).get("status")),
                 "  波特率档: %s   设备 ID: %s" % (probe.get("baud_code"),
                                              probe.get("device_id")),
                 "  基数探测: detected_base=%s（%s）" % (
                     probe.get("detected_base"), probe.get("detected_base_note")),
                 "  工具端供电: %s" % (probe.get("tool_voltage") or {}).get("voltage_type"),
                 "  代际线索: %s" % probe.get("controller_generation_hint"),
                 "  未实测假设: %s" % "; ".join(probe.get("assumptions") or [])]
        return _gripper_emit(args, payload, lines)

    # --------------------------------------------------------- open / close
    if action in ("open", "close"):
        speed = int(getattr(args, "speed", 100))
        force = int(getattr(args, "force", 100))
        path = cfg.get("path") or "modbus"
        if path == "rm_plus":
            payload = {
                "status": "blocked",
                "reason": "RM Plus 路径只作协议开关区分（disable / offline），没有夹爪动作接口",
                "next_action": ("把 workcell/rm_workcell.yaml::gripper.path 改成 modbus "
                                "（主路径）后重跑"),
                "evidence": ["RM Plus 不提供夹爪动作接口（白名单只有协议开关）"],
                "mode": "gripper",
                "action": action,
                "arm": arm,
            }
            return _gripper_emit(args, payload, [])
        if getattr(args, "dry_run", False):
            planned = ("rm_set_gripper_release(speed=%d)" % speed if action == "open" and
                       path == "gripper_api" else
                       "rm_set_gripper_pick(speed=%d, force=%d)" % (speed, force))
            if path == "modbus":
                planned = ("写寄存器 0x0006=1（松开到最大开口）" if action == "open" else
                           "写寄存器 0x000A=%d / 0x000B=%d / 0x0009=0（速度/力度/目标开口度）"
                           % (speed, force))
            payload = {
                "status": "ok",
                "reason": "dry-run：已解析参数与配置，**未下发任何动作**（零 SDK 调用）",
                "next_action": "去掉 --dry-run 即在真机窗口下发该动作",
                "evidence": [
                    "计划动作: %s" % planned,
                    "path=%s device_id=%s baud=%s" % (path, cfg.get("device_id"),
                                                     cfg.get("baud")),
                    "配置来源: %s" % cfg.get("source"),
                    "SDK 写调用次数: 0",
                ],
                "mode": "dry_run",
                "action": action,
                "arm": arm,
                "speed": speed,
                "force": force,
                "config": cfg,
            }
            lines = ["夹爪 %s（dry-run，未下发任何动作）" % action,
                     "  计划动作: %s" % planned,
                     "  路径    : %s（device_id=%s baud=%s）" % (path, cfg.get("device_id"),
                                                             cfg.get("baud"))]
            return _gripper_emit(args, payload, lines)
        adapter, handles, err_payload = _gripper_real_adapter(hw, ws, arm, cfg)
        if err_payload is not None:
            err_payload.update({"action": action})
            return _gripper_emit(args, err_payload, [])
        bridge, arm_obj = handles
        G = hw["gripper"]
        timeout_ms = int(cfg.get("action_timeout_ms") or 2000)
        try:
            # **运动互锁护栏**：真机夹爪动作期间把进程内共享闸门置为
            # GRIPPER_BUSY（动作结束 → GRIPPER_UNJUDGED，未判定前仍拒运动）。
            # 与 rm_executor 共用同一个 MotionGate 实例（同进程）。
            with hw["state"].gripper_action("夹爪 %s 动作中" % action):
                res = adapter.with_protocol(
                    lambda ad: _gripper_apply(ad, G, path, action, speed, force,
                                              timeout_ms))
        finally:
            try:
                arm_obj.close()
                bridge.close()
            except Exception:  # pragma: no cover
                pass
        payload = {
            "status": res.get("status") or "ok",
            "reason": res.get("reason") or "",
            "next_action": res.get("next_action") or "",
            "evidence": list(res.get("evidence") or []),
            "mode": "live",
            "action": action,
            "arm": arm,
            "speed": speed,
            "force": force,
            "path": path,
        }
        lines = ["夹爪 %s（真机）" % action,
                 "  结果: %s" % payload["reason"],
                 "  路径: %s  速度=%s 力度=%s" % (path, speed, force)]
        return _gripper_emit(args, payload, lines)

    # ------------------------------------------------------------- cycle
    if action == "cycle":
        path = cfg.get("path") or "modbus"
        if path == "rm_plus":
            payload = {
                "status": "blocked",
                "reason": "RM Plus 路径没有夹爪动作接口，无法做开合循环",
                "next_action": ("把 workcell/rm_workcell.yaml::gripper.path 改成 modbus "
                                "（主路径）后重跑"),
                "evidence": ["RM Plus 不提供夹爪动作接口（白名单只有协议开关）"],
                "mode": "gripper",
                "action": action,
                "arm": arm,
            }
            return _gripper_emit(args, payload, [])
        payload, lines = _gripper_cycle(hw, cfg, count, thresholds)
        payload.update({"action": action, "arm": arm})
        return _gripper_emit(args, payload, lines)

    # ------------------------------------------------------------- judge
    fault = getattr(args, "fault", "each") or "each"
    faults = list(GRIPPER_FAULT_CLASSES) if fault == "each" else [fault]
    payload, lines = _gripper_judge(hw, cfg, faults, count, thresholds)
    payload.update({"action": action, "arm": arm, "fault": fault})
    return _gripper_emit(args, payload, lines)


def _hw_gripper_check(ws):
    """hw:gripper —— 夹爪配置完整性与标定系数是否已回填。

    * 必需字段缺失 / 类型错 → **FAIL（阻塞）**，reason 逐字段指名（来自硬件层读取器）；
    * 标定字段为 null → **INFO（不阻塞）**：判定层据此走 NEEDS_PARAM 属**预期行为**，
      但必须给出「在真机窗口回填标定系数」的可粘贴动作。
    """
    mods, err = _gripper_hw(ws)
    if mods is None:
        return _hw_check("hw:gripper", False,
                         "硬件适配层 rm_hw_bridge 不可用: %s" % err,
                         _HW_PYTHONPATH_ACTION)
    res = _gripper_config_result(mods, ws)
    if res.get("status") != "ok":
        return _hw_check("hw:gripper", False,
                         res.get("reason") or "夹爪配置不可用",
                         res.get("next_action") or (
                             "补齐 workcell/rm_workcell.yaml::gripper 段后重跑 rmctl doctor"),
                         level="FAIL")
    cfg = res.get("config") or {}
    pending = list(cfg.get("calibration_pending") or [])
    if pending:
        return _hw_check(
            "hw:gripper", False,
            "夹爪配置齐备（%s），但标定系数未回填: %s"
            "（判定层据此走 needs_param，属预期行为，不阻塞）"
            % (cfg.get("source"), ", ".join(pending)),
            "在真机窗口回填标定系数（%s）：编辑 workcell/rm_workcell.yaml::gripper 的 %s"
            % (", ".join(pending), ", ".join(pending)),
            level="INFO")
    return _hw_check("hw:gripper", True,
                     "夹爪配置齐备且标定系数已回填（%s）" % cfg.get("source"),
                     "python3 -m rmctl gripper probe --arm left  # 用探针复核真机夹爪")


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
    s.add_argument("--json", action="store_true",
                   help="stdout 输出机器可读 JSON（含 checks[] 与 coverage_matrix[]）")
    s.add_argument("--report", default=None, metavar="PATH",
                   help="把体检 JSON 报告写到文件（与 --json 的 stdout 内容等价）")
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

    s = sub.add_parser(
        "plan",
        help="规划并校验",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description=(
            "规划并做四道校验。\n"
            "\n"
            "两种模式（默认行为逐字不变）：\n"
            "  1) 进程内直调（默认）：规划与四道校验都在 rmctl 进程内完成，\n"
            "     适合离线对照与回退；**不需要**先起任何服务。\n"
            "  2) 规划服务（--via-service）：规划交给独立的 rm_planner 服务\n"
            "     （Unix socket + 每请求独占 worker 子进程），规划进程可以\n"
            "     独立崩溃 / 重启。\n"
            "     前置条件：先启动服务 ——\n"
            "       rm-planner --ws <仓库根> --socket <仓库根>/build/run/rm_planner.sock\n"
            "\n"
            "两种模式输出同构：管线 / 成功与否 / 耗时 / 点数 / 四道校验逐项\n"
            "PASS-FAIL / 失败原因；服务模式额外打印服务端 per-node 指标\n"
            "（各节点 name + 耗时）。\n"
            "\n"
            "退出码（两种模式一致）：\n"
            "  0 成功（四道校验全过）\n"
            "  1 规划失败\n"
            "  2 参数错误\n"
            "  3 四道校验未通过\n"
            "  4 规划超时（服务端强制中断 worker，属于“需要人看”的类别，\n"
            "    与 3（校验未过）区分开）\n"
            "\n"
            "注：--pipeline / --num-planners / --planning-time / --no-verify /\n"
            "--no-continuous / --no-retime 只作用于进程内模式；服务模式的管线与\n"
            "校验参数由服务端的 workcell/planner/*.yaml 决定（唯一来源）。\n"
        ),
    )
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
    s.add_argument("--via-service", dest="via_service", action="store_true",
                   help="走 rm_planner 规划服务（默认：进程内直调）")
    s.add_argument("--socket", default=None,
                   help="[--via-service] 规划服务 socket 路径"
                        "（默认 <ws>/build/run/rm_planner.sock）")
    s.add_argument("--seed", dest="seed", type=int, default=0,
                   help="[--via-service] 规划种子（同种子 → 同结果，默认 0）")
    s.add_argument("--timeout-ms", dest="timeout_ms", type=int, default=None,
                   help="[--via-service] 服务端硬超时（毫秒，"
                        "默认取 workcell/planner/profiles.yaml）")
    s.set_defaults(func=cmd_plan)

    s = sub.add_parser("check", help="对轨迹文件做四道校验")
    s.add_argument("--traj", required=True)
    s.set_defaults(func=cmd_check)

    s = sub.add_parser("run",
                       help="执行一条轨迹（默认 legacy 回放；"
                            "--executor rm_executor / --live 走新执行器）"
                            "或执行一条声明式工艺（`rmctl run pick_place`）")
    s.add_argument("task", nargs="?", default=None,
                   help="工艺任务名（当前只支持 pick_place；与 --traj 互斥）")
    s.add_argument("--traj", default=None,
                   help="要执行的轨迹文件（与工艺任务互斥；缺省且无工艺任务 → 退出码 2）")
    s.add_argument("--task-file", dest="task_file", default=None,
                   help="[工艺] 任务规格文件（默认 workcell/tasks/pick_place.yaml）")
    s.add_argument("--count", type=int, default=None,
                   help="[工艺] 覆盖规格里的循环次数（1..50，越界 → 退出码 2）")
    s.add_argument("--trace", default=None,
                   help="[工艺] trace JSONL 落盘路径"
                        "（默认 build/run/pick_place_trace.jsonl）")
    s.add_argument("--replay", default=None,
                   help="[工艺] 只读回放一份 trace（零运动 / 零 SDK / 零网络）")
    s.add_argument("--period", type=float, default=0.010,
                   help="[legacy] 回放周期（秒）")
    s.add_argument("--fast", action="store_true",
                   help="[legacy] 不按真实时钟等待")
    s.add_argument("--executor", choices=["legacy", "rm_executor"],
                   default="legacy",
                   help="执行器实现：legacy=旧 MockExecutor 回放（默认，"
                        "**不作验收证据**）；rm_executor=新 10 ms 执行器")
    s.add_argument("--live", action="store_true",
                   help="走新执行器的真机路径（等价 --executor rm_executor + live）")
    s.add_argument("--report", default=None,
                   help="[rm_executor] 统一报告 JSON 落盘路径")
    s.add_argument("--arm", choices=["left", "right"], default="left",
                   help="[rm_executor] 选哪条臂")
    s.add_argument("--ip", default=None, help="[rm_executor --live] 控制器 IP")
    s.add_argument("--port", type=int, default=None,
                   help="[rm_executor --live] 控制器 TCP 端口")
    s.add_argument("--udp-port", dest="udp_port", type=int, default=None,
                   help="[rm_executor --live] 控制器 UDP 主动上报端口")
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

    s = sub.add_parser("connect", help="连真机并输出状态快照（R004 全字段）")
    s.add_argument("--arm", choices=["left", "right"], default="left",
                   help="选哪条臂（默认 left）")
    s.add_argument("--ip", default=None, help="覆盖基线里的控制器 IP")
    s.add_argument("--port", type=int, default=None, help="TCP 端口（默认取基线 8080）")
    s.add_argument("--udp-port", dest="udp_port", type=int, default=None,
                   help="UDP 主动上报端口（默认取基线）")
    s.add_argument("--json", action="store_true", help="机器可读输出（单个 JSON 对象）")
    s.add_argument("--dry-run", dest="dry_run", action="store_true",
                   help="走通全路径但不连真机")
    s.set_defaults(func=cmd_connect)

    s = sub.add_parser("safety",
                       help="安全闸门：外部强制停机 / 状态查询"
                            "（客户端，方向 rmctl → rm_runtime）")
    s.add_argument("action", nargs="?", default="status",
                   choices=["stop", "status"],
                   help="stop=强制停机（外部进程）；status=查询闸门状态")
    s.add_argument("--socket", dest="socket_path", default=None,
                   help="闸门 socket 路径"
                        "（默认 <工作区根>/build/run/rm_runtime.sock）")
    s.add_argument("--level", default="L4",
                   help="停机等级 L1/L2/L3/L4（默认 L4 急停）")
    s.set_defaults(func=cmd_safety)

    s = sub.add_parser("repl", help="交互式会话")
    s.set_defaults(func=cmd_repl)

    # ---- 夹爪体验层（S05）----
    g = sub.add_parser("gripper", help="夹爪：probe / open / close / cycle / judge")
    g.set_defaults(func=cmd_gripper)
    gsub = g.add_subparsers(dest="action")
    for _name, _help in (
        ("probe", "探针：协议/波特率/ID/寄存器基数/供电/代际线索"),
        ("open", "张开夹爪（松开）"),
        ("close", "夹爪夹取（闭合）"),
        ("cycle", "开合 N 次并统计超时（离线代理）"),
        ("judge", "注入六类故障并复核判定（离线代理）"),
    ):
        sp = gsub.add_parser(_name, help=_help)
        sp.add_argument("--arm", choices=["left", "right"], default="left",
                        help="选哪条臂（默认 left）")
        sp.add_argument("--json", action="store_true", help="机器可读输出（单个 JSON 对象）")
        sp.add_argument("--dry-run", dest="dry_run", action="store_true",
                        help="走通参数/配置解析但不下发任何动作")
        sp.add_argument("--ws", dest="ws", default=argparse.SUPPRESS,
                        help="工作区根目录（也可写在子命令之前）")
        if _name in ("open", "close"):
            sp.add_argument("--speed", type=int, default=100,
                            help="夹取速度（0–1000 标度，默认 100）")
            sp.add_argument("--force", type=int, default=100,
                            help="夹取力度（0–1000 标度，默认 100）")
        if _name == "cycle":
            sp.add_argument("--count", type=int, default=10, help="开合次数（默认 10）")
        if _name == "judge":
            sp.add_argument("--fault", default="each",
                            choices=["each"] + list(GRIPPER_FAULT_CLASSES),
                            help="注入哪类故障（默认 each = 六类各来一次）")
            sp.add_argument("--count", type=int, default=1,
                            help="每类注入次数（默认 1）")
        sp.set_defaults(func=cmd_gripper)
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
