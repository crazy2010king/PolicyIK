"""真机基线 schema 与校验器（设计 §5.8 / D001）。

四类基线文件（工作区 ``workcell/baselines/`` 下，git 跟踪、可 diff 可评审）：

===================== ====================================================
zero_baseline         零位基线（固件/示教器版本/每关节零位/凹槽照片/FK 复测）
measured_limits       实测限值（每关节 acceleration+jerk、TCP 重复性）
version_matrix        版本矩阵（控制器/示教器/SDK/工具链/系统）
network_plan          网络与端口规划（双臂 IP、TCP/UDP 端口、上报周期）
===================== ====================================================

设计约束：

* 本模块只依赖 ``yaml`` + ``numpy``，**禁止 import tesseract**——保证在没有
  GLIBCXX_3.4.31 的环境里也能被 pytest 覆盖（基线校验不应依赖厂商解算库）。
* 未实测项一律写 ``null`` 且 ``status`` 为 ``pending``/``blocked``，
  **禁止填写占位数值**；校验器对此显式报错。
* ``status`` 语义：``pending``（待实测）/``measured``（已实测且带证据）/
  ``blocked``（被外部条件阻塞，附阻塞原因）。
* 控制器位姿（厂商运动学输出）只允许出现在 FK 交叉校验字段里，
  且必须显式标注"仅用于 FK 交叉校验"（R003）。
"""
import os

import numpy as np
import yaml

from . import paths

KINDS = ("zero_baseline", "measured_limits", "version_matrix", "network_plan")
SCHEMA_VERSION = 1
STATUSES = ("pending", "measured", "blocked")
JOINT_NAMES = tuple("joint%d" % i for i in range(1, 8))

ACCELERATION_UNIT = "rad/s^2"
JERK_UNIT = "rad/s^3"
ANGLE_UNIT = "deg"
LENGTH_UNIT = "mm"

UDP_PERIOD_MS = 5          # 实时上报周期必须是 5 ms 的倍数（设计 §5.1）
MIN_SAMPLES = 3            # 实测样本数下限（单次极值不算证据，设计 §5.2）
FK_TOL_MM = 1.0            # FK 交叉校验位置判定阈值（mm，设计 §5.3）
CONTROLLER_POSE_USAGE = "仅用于 FK 交叉校验，不得作为规划真值"

_COMMON_REQUIRED = ("kind", "schema_version", "version", "status", "captured_at")


class BaselineError(Exception):
    """基线文件缺失 / 解析失败（消息面向操作者，可直接粘贴）。"""


# ------------------------------------------------------------------ helpers
def _issue(level, code, message, path=""):
    return {"level": level, "code": code, "path": path, "message": message}


def _err(code, message, path=""):
    return _issue("error", code, message, path)


def _warn(code, message, path=""):
    return _issue("warn", code, message, path)


def _is_num(v):
    return isinstance(v, (int, float, np.integer, np.floating)) and not isinstance(v, bool)


def _is_int(v):
    return isinstance(v, (int, np.integer)) and not isinstance(v, bool)


def _is_str(v):
    return isinstance(v, str)


def _blank(v):
    return v is None or (isinstance(v, str) and not v.strip())


def baseline_path(kind, ws=None):
    """某类基线文件的绝对路径。"""
    return os.path.join(paths.baselines_dir(ws), "%s.yaml" % kind)


def baselines_dir(ws=None):
    """基线目录（转发到 paths，保持单一真值）。"""
    return paths.baselines_dir(ws)


def load_baseline(kind, ws=None):
    """读取一份基线 YAML；缺失或非法时抛出 BaselineError（含明确修复动作）。"""
    if kind not in KINDS:
        raise BaselineError("未知基线类型: %s（可选: %s）" % (kind, ", ".join(KINDS)))
    path = baseline_path(kind, ws)
    if not os.path.exists(path):
        raise BaselineError(
            "基线文件缺失: %s\n→ 请执行: python3 -m rmctl baseline scaffold" % path)
    try:
        with open(path, "r", encoding="utf-8") as fh:
            data = yaml.safe_load(fh)
    except yaml.YAMLError as exc:
        raise BaselineError("基线 YAML 解析失败: %s (%s)" % (path, exc))
    if data is None:
        raise BaselineError("基线文件为空: %s" % path)
    return data


def load_joint_limits(ws=None):
    """读取 joint_limits.yaml（唯一真值），返回 {joint_name: {...}}；缺失返回 None。"""
    path = paths.joint_limits_path(ws) if ws else paths.joint_limits_path()
    if not path or not os.path.exists(path):
        return None
    try:
        with open(path, "r", encoding="utf-8") as fh:
            raw = yaml.safe_load(fh) or {}
    except yaml.YAMLError:
        return None
    group = raw.get("rm75_arm")
    if not isinstance(group, dict):
        return None
    return group


# ------------------------------------------------------------------ 公共校验
def _validate_common(kind, data):
    issues = []
    if data.get("kind") != kind:
        issues.append(_err("kind_mismatch",
                           "kind 字段应为 '%s'，实际为 %r" % (kind, data.get("kind")),
                           "kind"))
    sv = data.get("schema_version")
    if not _is_int(sv) or sv != SCHEMA_VERSION:
        issues.append(_err("schema_version_unsupported",
                           "schema_version 必须为整数 %d，实际为 %r" % (SCHEMA_VERSION, sv),
                           "schema_version"))
    ver = data.get("version")
    if not _is_int(ver) or ver < 1:
        issues.append(_err("version_invalid",
                           "version 必须是 >=1 的整数（基线修订号，单调递增），实际为 %r" % ver,
                           "version"))
    status = data.get("status")
    if status not in STATUSES:
        issues.append(_err("status_invalid",
                           "status 必须是 %s 之一，实际为 %r" % ("/".join(STATUSES), status),
                           "status"))
    if status == "blocked":
        # blocked 必须自证：写明阻塞原因 + 已尝试的门禁动作（禁止空口说被阻塞）
        blocked = data.get("blocked")
        if not isinstance(blocked, dict):
            issues.append(_err("blocked_missing",
                               "status=blocked 时必须有 blocked 段（reason + attempted_actions）",
                               "blocked"))
        else:
            if _blank(blocked.get("reason")):
                issues.append(_err("blocked_reason_missing",
                                   "status=blocked 时 blocked.reason 必须写明阻塞原因",
                                   "blocked.reason"))
            actions = blocked.get("attempted_actions")
            if not isinstance(actions, list) or not actions:
                issues.append(_err("blocked_actions_missing",
                                   "status=blocked 时 blocked.attempted_actions 必须列出"
                                   "已尝试的门禁动作（至少一条）", "blocked.attempted_actions"))
    if "captured_at" not in data:
        issues.append(_err("captured_at_missing", "缺少 captured_at 字段（未实测写 null）",
                           "captured_at"))
    elif status == "measured" and _blank(data.get("captured_at")):
        issues.append(_err("captured_at_required",
                           "status=measured 时 captured_at 必须填写采集时间", "captured_at"))
    return issues


def check_version_monotonic(previous, current, kind=""):
    """版本单调：同一类基线的修订号不得倒退。"""
    issues = []
    if not isinstance(previous, dict) or not isinstance(current, dict):
        return issues
    prev = previous.get("version")
    cur = current.get("version")
    if not _is_int(prev) or not _is_int(cur):
        return issues
    if cur <= prev:
        issues.append(_err("version_not_monotonic",
                           "版本必须单调递增: %s 的 version %r 不大于上一版 %r"
                           % (kind or "基线", cur, prev), "version"))
    return issues


# ------------------------------------------------------------------ 各类型校验
def _require_keys(data, keys, prefix=""):
    issues = []
    for k in keys:
        if k not in data:
            issues.append(_err("missing_field", "缺少必填字段 '%s%s'" % (prefix, k),
                               prefix + k))
    return issues


def _check_joint_list(data, field="joints"):
    """校验 7 关节条目结构，返回 (issues, {name: entry})。"""
    issues = []
    entries = {}
    joints = data.get(field)
    if not isinstance(joints, list):
        issues.append(_err("invalid_joints", "%s 必须是列表（7 个关节条目）" % field, field))
        return issues, entries
    if len(joints) != len(JOINT_NAMES):
        issues.append(_err("joint_count", "%s 必须恰好 %d 个关节条目，实际 %d"
                           % (field, len(JOINT_NAMES), len(joints)), field))
    for idx, entry in enumerate(joints):
        path = "%s[%d]" % (field, idx)
        if not isinstance(entry, dict):
            issues.append(_err("invalid_joint_entry", "%s 必须是映射" % path, path))
            continue
        name = entry.get("name")
        if name not in JOINT_NAMES:
            issues.append(_err("unknown_joint",
                               "%s 的 name=%r 不是合法关节名（%s）"
                               % (path, name, ", ".join(JOINT_NAMES)), path + ".name"))
        elif name in entries:
            issues.append(_err("duplicate_joint", "%s 关节 %s 重复" % (path, name),
                               path + ".name"))
        else:
            entries[name] = entry
    missing = [n for n in JOINT_NAMES if n not in entries]
    if missing:
        issues.append(_err("missing_joints", "缺少关节条目: %s" % ", ".join(missing), field))
    return issues, entries


def _validate_zero_baseline(data, ws=None):
    issues = []
    issues += _require_keys(data, ("arm", "firmware", "teachpendant_version", "joints",
                                    "groove_photos", "fk_recheck", "recalibration_triggers"))
    status = data.get("status")
    issues += _require_keys(data, ("arm",), prefix="")
    if not _is_str(data.get("arm")) or not data.get("arm"):
        issues.append(_err("invalid_arm", "arm 必须是非空字符串（left/right）", "arm"))

    jissues, entries = _check_joint_list(data, "joints")
    issues += jissues
    for name, entry in entries.items():
        p = "joints.%s" % name
        unit = entry.get("unit")
        if unit != ANGLE_UNIT:
            issues.append(_err("unit_mismatch",
                               "%s.unit 必须为 '%s'（关节读数用度），实际为 %r"
                               % (p, ANGLE_UNIT, unit), p + ".unit"))
        if "zero_deg" not in entry:
            issues.append(_err("missing_field", "%s 缺少 zero_deg（未实测写 null）" % p,
                               p + ".zero_deg"))
        elif status == "measured" and not _is_num(entry.get("zero_deg")):
            issues.append(_err("missing_measurement",
                               "%s.zero_deg 在 status=measured 时必须是数值" % p,
                               p + ".zero_deg"))
        if "groove_photo" not in entry:
            issues.append(_err("missing_field", "%s 缺少 groove_photo（照片引用，未实测写 null）" % p,
                               p + ".groove_photo"))

    photos = data.get("groove_photos")
    if not isinstance(photos, list):
        issues.append(_err("invalid_groove_photos", "groove_photos 必须是列表", "groove_photos"))
    elif status == "measured" and not photos:
        issues.append(_err("missing_evidence",
                           "status=measured 时 groove_photos 至少要有一条凹槽对齐照片引用",
                           "groove_photos"))

    fk = data.get("fk_recheck")
    if not isinstance(fk, dict):
        issues.append(_err("invalid_fk_recheck", "fk_recheck 必须是映射", "fk_recheck"))
    else:
        usage = fk.get("controller_pose_usage")
        if not _is_str(usage) or "仅用于 FK 交叉校验" not in usage:
            issues.append(_err("controller_pose_usage_missing",
                               "fk_recheck.controller_pose_usage 必须显式标注"
                               "'仅用于 FK 交叉校验，不得作为规划真值'（R003）",
                               "fk_recheck.controller_pose_usage"))
        for key in ("method", "max_pos_dev_mm", "max_ori_dev_deg", "conclusion"):
            if key not in fk:
                issues.append(_err("missing_field", "fk_recheck 缺少 '%s'（未实测写 null）" % key,
                                   "fk_recheck." + key))
        dev = fk.get("max_pos_dev_mm")
        if status == "measured":
            if not _is_num(dev):
                issues.append(_err("missing_measurement",
                                   "fk_recheck.max_pos_dev_mm 在 status=measured 时必须是数值",
                                   "fk_recheck.max_pos_dev_mm"))
            elif dev >= FK_TOL_MM:
                issues.append(_err("fk_deviation_exceeded",
                                   "FK 交叉校验位置偏差 %r mm 超过阈值 %r mm（设计 §5.3）"
                                   % (dev, FK_TOL_MM), "fk_recheck.max_pos_dev_mm"))

    triggers = data.get("recalibration_triggers")
    if not isinstance(triggers, list) or not triggers:
        issues.append(_err("invalid_triggers",
                           "recalibration_triggers 必须是非空列表（重标触发条件表）",
                           "recalibration_triggers"))
    elif status == "measured":
        if _blank(data.get("firmware")):
            issues.append(_err("missing_measurement",
                               "status=measured 时必须填写控制器固件版本", "firmware"))
        if _blank(data.get("teachpendant_version")):
            issues.append(_err("missing_measurement",
                               "status=measured 时必须填写示教器版本", "teachpendant_version"))
    return issues


def _validate_measured_limits(data, ws=None):
    issues = []
    issues += _require_keys(data, ("arm", "joints", "tcp_repeatability", "udp_period_ms"))
    status = data.get("status")
    jissues, entries = _check_joint_list(data, "joints")
    issues += jissues

    limits = load_joint_limits(ws) if ws is not None else load_joint_limits()

    for name, entry in entries.items():
        p = "joints.%s" % name
        for field, unit in (("acceleration", ACCELERATION_UNIT), ("jerk", JERK_UNIT)):
            item = entry.get(field)
            fp = "%s.%s" % (p, field)
            if not isinstance(item, dict):
                issues.append(_err("missing_field", "%s 必须是映射（value/unit/source/method/samples）"
                                   % fp, fp))
                continue
            for key in ("value", "unit", "source", "method", "samples"):
                if key not in item:
                    issues.append(_err("missing_field", "%s 缺少 '%s'（未实测写 null）"
                                       % (fp, key), fp + "." + key))
            got_unit = item.get("unit")
            if got_unit != unit:
                issues.append(_err("unit_mismatch",
                                   "%s.unit 必须为 '%s'，实际为 %r" % (fp, unit, got_unit),
                                   fp + ".unit"))
            value = item.get("value")
            if status == "measured":
                if not _is_num(value) or value <= 0:
                    issues.append(_err("missing_measurement",
                                       "%s.value 在 status=measured 时必须是正数" % fp,
                                       fp + ".value"))
                if _blank(item.get("source")):
                    issues.append(_err("missing_source",
                                       "%s.source 在 status=measured 时必须写明来源（可追溯）" % fp,
                                       fp + ".source"))
                if _blank(item.get("method")):
                    issues.append(_err("missing_source",
                                       "%s.method 在 status=measured 时必须写明采集方法" % fp,
                                       fp + ".method"))
                samples = item.get("samples")
                if not _is_int(samples) or samples < MIN_SAMPLES:
                    issues.append(_err("insufficient_samples",
                                       "%s.samples 至少 %d（单次极值不算证据），实际为 %r"
                                       % (fp, MIN_SAMPLES, samples), fp + ".samples"))
                elif limits is None:
                    issues.append(_err("joint_limits_unavailable",
                                       "读不到 joint_limits.yaml，无法校验 %s 是否比已生效限值更宽松"
                                       % fp, fp + ".value"))
                elif name not in limits:
                    issues.append(_err("joint_limits_missing_joint",
                                       "joint_limits.yaml 中没有关节 %s，无法做保守性校验" % name,
                                       fp + ".value"))
                else:
                    ref = limits[name].get(field)
                    if _is_num(ref) and _is_num(value) and value > ref:
                        issues.append(_err("limit_above_joint_limits",
                                           "%s.value=%r 高于 joint_limits.yaml 的 %r"
                                           "（基线不得比已生效限值更宽松）" % (fp, value, ref),
                                           fp + ".value"))
            else:
                if value is not None:
                    issues.append(_err("unmeasured_value_present",
                                       "%s.value 在 status=%s 时必须为 null（禁止占位数值）"
                                       % (fp, status), fp + ".value"))

    tcp = data.get("tcp_repeatability")
    if not isinstance(tcp, dict):
        issues.append(_err("missing_field", "tcp_repeatability 必须是映射", "tcp_repeatability"))
    else:
        for key in ("value_mm", "unit", "source", "method", "samples"):
            if key not in tcp:
                issues.append(_err("missing_field", "tcp_repeatability 缺少 '%s'（未实测写 null）" % key,
                                   "tcp_repeatability." + key))
        if tcp.get("unit") != LENGTH_UNIT:
            issues.append(_err("unit_mismatch",
                               "tcp_repeatability.unit 必须为 '%s'，实际为 %r"
                               % (LENGTH_UNIT, tcp.get("unit")),
                               "tcp_repeatability.unit"))
        val = tcp.get("value_mm")
        if status == "measured":
            if not _is_num(val) or val < 0:
                issues.append(_err("missing_measurement",
                                   "tcp_repeatability.value_mm 在 status=measured 时必须是非负数",
                                   "tcp_repeatability.value_mm"))
            samples = tcp.get("samples")
            if not _is_int(samples) or samples < MIN_SAMPLES:
                issues.append(_err("insufficient_samples",
                                   "tcp_repeatability.samples 至少 %d，实际为 %r"
                                   % (MIN_SAMPLES, samples), "tcp_repeatability.samples"))
        elif val is not None:
            issues.append(_err("unmeasured_value_present",
                               "tcp_repeatability.value_mm 在 status=%s 时必须为 null"
                               % status, "tcp_repeatability.value_mm"))

    period = data.get("udp_period_ms")
    if not _is_int(period) or period <= 0:
        issues.append(_err("invalid_udp_period",
                           "udp_period_ms 必须是正整数（实时上报周期），实际为 %r" % period,
                           "udp_period_ms"))
    elif period % UDP_PERIOD_MS != 0:
        issues.append(_err("udp_period_not_multiple",
                           "udp_period_ms=%r 必须是 %d ms 的倍数（设计 §5.1）"
                           % (period, UDP_PERIOD_MS), "udp_period_ms"))
    return issues


def _validate_version_matrix(data, ws=None):
    issues = []
    fields = ("controller_firmware", "teachpendant", "rm_api2_sdk", "ros2_rm_robot_tag",
              "tesseract_robotics_nanobind", "libstdcxx", "python", "os", "container_image")
    issues += _require_keys(data, fields)
    status = data.get("status")
    for f in fields:
        val = data.get(f)
        if val is None:
            if status == "measured":
                issues.append(_err("missing_measurement",
                                   "status=measured 时版本矩阵条目 '%s' 必须填写" % f, f))
            continue
        if not _is_str(val):
            issues.append(_err("invalid_version_entry",
                               "版本矩阵条目 '%s' 必须是字符串或 null，实际为 %r" % (f, val), f))
    return issues


def _valid_ip(ip):
    if not _is_str(ip):
        return False
    parts = ip.split(".")
    if len(parts) != 4:
        return False
    for p in parts:
        if not p.isdigit() or not 0 <= int(p) <= 255:
            return False
    return True


def _validate_network_plan(data, ws=None):
    issues = []
    issues += _require_keys(data, ("arms", "realtime_report", "constraints"))
    arms = data.get("arms")
    if not isinstance(arms, dict):
        issues.append(_err("invalid_arms", "arms 必须是映射（left/right）", "arms"))
    else:
        for side in ("left", "right"):
            arm = arms.get(side)
            p = "arms.%s" % side
            if not isinstance(arm, dict):
                issues.append(_err("missing_field", "%s 必须是映射" % p, p))
                continue
            for key in ("ip", "tcp_port", "udp_port"):
                if key not in arm:
                    issues.append(_err("missing_field", "%s 缺少 '%s'" % (p, key),
                                       "%s.%s" % (p, key)))
            if not _valid_ip(arm.get("ip")):
                issues.append(_err("invalid_ip", "%s.ip 必须是合法 IPv4，实际为 %r"
                                   % (p, arm.get("ip")), p + ".ip"))
            for key in ("tcp_port", "udp_port"):
                port = arm.get(key)
                if key in arm and (not _is_int(port) or not 1 <= port <= 65535):
                    issues.append(_err("invalid_port", "%s.%s 必须是 1..65535 的整数，实际为 %r"
                                       % (p, key, port), "%s.%s" % (p, key)))
        left, right = arms.get("left"), arms.get("right")
        if isinstance(left, dict) and isinstance(right, dict):
            if left.get("udp_port") is not None and left.get("udp_port") == right.get("udp_port"):
                issues.append(_err("duplicate_udp_port",
                                   "左右臂 UDP 端口必须不同（设计 §5.1），实际都是 %r"
                                   % left.get("udp_port"), "arms.right.udp_port"))

    rep = data.get("realtime_report")
    if not isinstance(rep, dict):
        issues.append(_err("missing_field", "realtime_report 必须是映射", "realtime_report"))
    else:
        for key in ("period_ms", "cycle_multiple_ms", "mode"):
            if key not in rep:
                issues.append(_err("missing_field", "realtime_report 缺少 '%s'" % key,
                                   "realtime_report." + key))
        period = rep.get("period_ms")
        mult = rep.get("cycle_multiple_ms")
        if _is_int(period) and _is_int(mult) and mult > 0 and period % mult != 0:
            issues.append(_err("udp_period_not_multiple",
                               "realtime_report.period_ms=%r 必须是 cycle_multiple_ms=%r 的倍数"
                               % (period, mult), "realtime_report.period_ms"))
        if rep.get("mode") != "triple_thread":
            issues.append(_err("invalid_report_mode",
                               "realtime_report.mode 必须为 'triple_thread'"
                               "（不开三线程收不到 UDP 实时上报）", "realtime_report.mode"))

    con = data.get("constraints")
    if not isinstance(con, dict):
        issues.append(_err("missing_field", "constraints 必须是映射", "constraints"))
    else:
        for key in ("single_client_per_endpoint", "triple_thread_mode", "wired_direct"):
            if key not in con:
                issues.append(_err("missing_field", "constraints 缺少 '%s'" % key,
                                   "constraints." + key))
        for key in ("single_client_per_endpoint", "triple_thread_mode"):
            if key in con and con.get(key) is not True:
                issues.append(_err("constraint_violated",
                                   "constraints.%s 必须为 true（硬约束）" % key,
                                   "constraints." + key))
    return issues


_VALIDATORS = {
    "zero_baseline": _validate_zero_baseline,
    "measured_limits": _validate_measured_limits,
    "version_matrix": _validate_version_matrix,
    "network_plan": _validate_network_plan,
}


# ------------------------------------------------------------------ 对外接口
def validate_baseline(kind, data, ws=None, previous=None):
    """校验一份基线数据，返回 issues 列表。

    每条 issue 是 ``{level, code, path, message}``；level=error 表示阻塞。
    ``status=pending`` 不阻塞校验（未实测是合法状态），但会带上 info 级说明。
    """
    if kind not in KINDS:
        return [_err("unknown_kind", "未知基线类型: %s（可选: %s）" % (kind, ", ".join(KINDS)))]
    if not isinstance(data, dict):
        return [_err("not_mapping", "基线文件顶层必须是映射（dict）")]
    issues = _validate_common(kind, data)
    issues += _VALIDATORS[kind](data, ws)
    if previous is not None:
        issues += check_version_monotonic(previous, data, kind)
    if data.get("status") == "pending":
        issues.append(_issue("info", "pending",
                             "%s 尚未实测（status=pending），实测回填后需把 status 改为 measured "
                             "并补齐 source/method/samples" % kind))
    elif data.get("status") == "blocked":
        blocked = data.get("blocked") or {}
        issues.append(_issue("info", "blocked",
                             "%s 被阻塞（status=blocked）：%s" % (kind, blocked.get("reason") or "")))
    return issues


def validate_all(ws=None, previous=None):
    """校验全部四类基线，返回汇总报告。

    报告结构::

        {"ok": bool, "ws": str, "baselines_dir": str,
         "kinds": {kind: {"present": bool, "status": str|None, "version": int|None,
                          "issues": [...], "error_count": int}},
         "issues": [...], "missing": [...], "error_count": int}
    """
    ws = ws or paths.workspace_root()
    bdir = paths.baselines_dir(ws)
    report = {"ok": True, "ws": os.path.abspath(ws), "baselines_dir": bdir,
              "kinds": {}, "issues": [], "missing": [], "error_count": 0}
    all_issues = []
    for kind in KINDS:
        entry = {"present": False, "status": None, "version": None, "issues": [],
                 "error_count": 0}
        path = os.path.join(bdir, "%s.yaml" % kind)
        if not os.path.exists(path):
            entry["issues"] = [_err("file_missing",
                                    "基线文件缺失: %s\n→ 修复: python3 -m rmctl baseline scaffold"
                                    % path)]
            all_issues += entry["issues"]
            entry["error_count"] = len(entry["issues"])
            report["kinds"][kind] = entry
            report["missing"].append(kind)
            continue
        entry["present"] = True
        try:
            data = load_baseline(kind, ws)
        except BaselineError as exc:
            entry["issues"] = [_err("unreadable", str(exc))]
        else:
            entry["status"] = data.get("status")
            entry["version"] = data.get("version")
            prev = (previous or {}).get(kind)
            entry["issues"] = validate_baseline(kind, data, ws=ws, previous=prev)
        entry["error_count"] = sum(1 for i in entry["issues"] if i["level"] == "error")
        all_issues += entry["issues"]
        report["kinds"][kind] = entry
    report["issues"] = all_issues
    report["error_count"] = sum(1 for i in all_issues if i["level"] == "error")
    report["ok"] = report["error_count"] == 0 and not report["missing"]
    return report


def format_issues(issues, kind=""):
    """把 issues 渲染成人类可读文本（供 CLI / 文档复用）。"""
    lines = []
    for i in issues:
        loc = ("[%s] " % i["path"]) if i.get("path") else ""
        lines.append("%s%s %s: %s" % (i["level"].upper(), loc, i["code"], i["message"]))
    return "\n".join(lines)
