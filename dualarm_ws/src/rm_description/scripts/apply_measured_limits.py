#!/usr/bin/env python3
"""把真机实测限值回填进 joint_limits.yaml（设计 §5.2 / D001）。

背景
----
``dualarm_ws/src/rm_description/config/joint_limits.yaml`` 里的 acceleration/jerk
是**保守占位值**（文件头明说"实测后更新"）：官方从未给出 RM75 的加速度数值，
只有 RM65 的 600°/s² ≈ 10.47 rad/s² 可参考。实测数据到手后必须能"一次回填、
全链路自动生效"，否则基线与校验口径会长期漂移。

为什么默认保守
--------------
研究记录过一个教训：**jerk 实测超"声明值" 2.75 倍**——单次极值污染了测量。
因此本工具默认 ``mode=conservative``：新值取 ``min(实测值, 现值)``，保证
只可能更保守，不会因为一次采样把保护阈值放大；``mode=measured`` 必须显式
开启（``--mode measured``），并打印逐关节差异摘要。

拒绝回填的三条红线（Q5 失效模式）
---------------------------------
1. 实测值被单次极值污染 → ``samples`` 必须 ≥ 3（``MIN_SAMPLES``）；
2. 缺来源 → 每条必须有非空 ``source``（可追溯）；
3. 缺采集方法 → 每条必须有非空 ``method``。

另外，基线整体 ``status`` 必须是 ``measured``；未实测（pending/blocked）一律
拒绝回填，绝不写入任何数值。

写盘约定
--------
* 只做**行内数值替换**：joint_limits.yaml 原有注释（文件头说明、每个关节行尾的
  "±178°, 180°/s" 等）逐字保留；
* 文件尾追加一段"实测限值回填记录"注释（回填日期、基线来源与版本、逐关节
  source/method/samples），使改动可 diff、可评审（D001）；
* 若存在机器本地的 ``build/model/joint_limits.yaml``（rmctl init 物化的副本，
  gitignore），同步刷新它——``rmctl.paths.joint_limits_path`` 优先读这份副本，
  不刷新的话"回填"不会真正生效。

用法
----
    # 干跑（只打印 diff，不写盘）
    python3 dualarm_ws/src/rm_description/scripts/apply_measured_limits.py --dry-run

    # 保守回填（默认）
    python3 dualarm_ws/src/rm_description/scripts/apply_measured_limits.py

    # 显式采用实测值（会打印逐关节差异摘要）
    python3 dualarm_ws/src/rm_description/scripts/apply_measured_limits.py --mode measured

退出码：0 成功 / 2 输入非法（文件缺失、YAML 解析失败）/ 3 拒绝回填（校验不通过）。
"""
import argparse
import datetime
import json
import os
import re
import shutil
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)                       # .../rm_description
WS_DEFAULT = os.path.abspath(os.path.join(PKG, "..", "..", ".."))

GROUP = "rm75_arm"
JOINT_NAMES = tuple("joint%d" % i for i in range(1, 8))
FIELDS = ("acceleration", "jerk")
ACC_UNIT = "rad/s^2"
JERK_UNIT = "rad/s^3"
MIN_SAMPLES = 3

# 物理合理性上限（绝对封顶）：RM75 官方没有任何加速度数值，官方 URDF 统一
# velocity=3.14 rad/s；把 acceleration 放到 100 rad/s²（≈5700°/s²）以上已经
# 不可能是本机的真实上限，jerk 超过 1e4 rad/s³ 同理。超过上限 → 拒绝回填，
# 而不是照单全收把保护阈值写成荒谬值。
SANE_MAX_ACCELERATION = 100.0     # rad/s^2
SANE_MAX_JERK = 10000.0           # rad/s^3

# 关节行：`  joint1: {lower: ..., acceleration: 5.2359, jerk: 30.0}   # 注释`
_JOINT_LINE_RE = re.compile(r"^(?P<head>\s*joint(?P<idx>[1-7])\s*:\s*\{)(?P<body>.*)$")
_FIELD_RE = re.compile(r"(?P<key>acceleration|jerk)(?P<sp>\s*:\s*)(?P<val>[-+0-9.eE]+)")

# 回填记录块标记：每次回填**替换**旧块（而不是无限追加），避免同一文件反复
# 回填时尾部注释块线性膨胀。
PROVENANCE_MARKER = "# ── 实测限值回填记录"


class BackfillError(Exception):
    """输入非法（文件缺失 / YAML 解析失败）——对应退出码 2。"""


# ------------------------------------------------------------------ 小工具
def _issue(level, code, message, path=""):
    return {"level": level, "code": code, "path": path, "message": message}


def _err(code, message, path=""):
    return _issue("error", code, message, path)


def _warn(code, message, path=""):
    return _issue("warn", code, message, path)


def _is_num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def _is_int(v):
    return isinstance(v, int) and not isinstance(v, bool)


def _blank(v):
    return v is None or (isinstance(v, str) and not v.strip())


def _fmt_num(v):
    """数值 → 稳定的 YAML 文本（保留浮点身份，不写成 int）。"""
    f = float(v)
    if f == int(f) and abs(f) < 1e15:
        return "%.1f" % f
    return repr(round(f, 9))


def _one_line(text):
    return " ".join(str(text).split())


def load_yaml(path, what):
    if not os.path.exists(path):
        raise BackfillError("%s 不存在: %s" % (what, path))
    try:
        with open(path, "r", encoding="utf-8") as fh:
            return yaml.safe_load(fh)
    except yaml.YAMLError as exc:
        raise BackfillError("%s 解析失败: %s (%s)" % (what, path, exc))


def default_paths(ws):
    ws = os.path.abspath(ws)
    return {
        "baseline": os.path.join(ws, "workcell", "baselines", "measured_limits.yaml"),
        "limits": os.path.join(ws, "dualarm_ws", "src", "rm_description", "config",
                               "joint_limits.yaml"),
        "build_copy": os.path.join(ws, "build", "model", "joint_limits.yaml"),
    }


# ------------------------------------------------------------------ 校验 / 计划
def plan_backfill(baseline, limits, mode="conservative"):
    """算出回填计划（不写盘）。

    参数
    ----
    baseline : measured_limits.yaml 解析后的 dict
    limits   : joint_limits.yaml 解析后的 dict（顶层含 rm75_arm 组）
    mode     : ``conservative``（默认，取 min）或 ``measured``（直接采用实测值）

    返回
    ----
    ``{"ok": bool, "issues": [...], "updates": {joint: {field: {...}}}, "changed": [...]}``
    """
    issues = []
    updates = {}

    if mode not in ("conservative", "measured"):
        return {"ok": False, "issues": [_err("bad_mode", "未知模式: %r" % mode)],
                "updates": {}, "changed": []}

    if not isinstance(baseline, dict):
        return {"ok": False, "issues": [_err("baseline_not_mapping", "基线顶层必须是映射")],
                "updates": {}, "changed": []}
    if not isinstance(limits, dict):
        return {"ok": False, "issues": [_err("limits_not_mapping", "限值文件顶层必须是映射")],
                "updates": {}, "changed": []}

    group = limits.get(GROUP)
    if not isinstance(group, dict):
        issues.append(_err("limits_group_missing",
                           "限值文件缺少 %s 组（无法定位 7 个关节）" % GROUP))
        group = {}

    status = baseline.get("status")
    if status != "measured":
        issues.append(_err(
            "baseline_not_measured",
            "基线 status=%r，未实测不得回填（只允许 status=measured 且带证据的值）"
            % status, "status"))

    joints = baseline.get("joints")
    if not isinstance(joints, list):
        issues.append(_err("invalid_joints", "基线 joints 必须是列表", "joints"))
        joints = []
    entries = {}
    for entry in joints:
        if not isinstance(entry, dict):
            issues.append(_err("invalid_joint_entry", "joints 条目必须是映射", "joints"))
            continue
        name = entry.get("name")
        if name in entries:
            issues.append(_err("duplicate_joint", "关节 %r 重复" % name, "joints"))
            continue
        entries[name] = entry
    for name in JOINT_NAMES:
        if name not in entries:
            issues.append(_err("missing_joint", "基线缺少关节条目: %s" % name, "joints"))

    changed = []
    for name in JOINT_NAMES:
        entry = entries.get(name)
        if entry is None:
            continue
        if name not in group:
            issues.append(_err("joint_limits_missing_joint",
                               "joint_limits.yaml 中没有关节 %s，无法回填" % name, name))
            continue
        cur = group.get(name) or {}
        for field in FIELDS:
            unit = ACC_UNIT if field == "acceleration" else JERK_UNIT
            cap = SANE_MAX_ACCELERATION if field == "acceleration" else SANE_MAX_JERK
            path = "%s.%s" % (name, field)
            item = entry.get(field)
            if not isinstance(item, dict):
                issues.append(_err("missing_field",
                                   "%s 必须是映射（value/unit/source/method/samples）" % path, path))
                continue
            value = item.get("value")
            src = item.get("source")
            method = item.get("method")
            samples = item.get("samples")
            got_unit = item.get("unit")

            # 单位口径
            if got_unit != unit:
                issues.append(_err("unit_mismatch",
                                   "%s.unit 必须为 '%s'，实际为 %r" % (path, unit, got_unit),
                                   path + ".unit"))
                continue
            # 数值
            if not _is_num(value):
                issues.append(_err("missing_measurement",
                                   "%s.value 必须是数值（未实测不得回填）" % path,
                                   path + ".value"))
                continue
            if not (value > 0):
                issues.append(_err("non_positive_value",
                                   "%s.value=%r 必须为正数（负值/零不具物理意义）"
                                   % (path, value), path + ".value"))
                continue
            if value > cap:
                issues.append(_err("implausible_value",
                                   "%s.value=%r 超出物理合理性上限 %r（单次极值污染？拒绝回填）"
                                   % (path, value, cap), path + ".value"))
                continue
            # 来源可追溯
            if _blank(src):
                issues.append(_err("missing_source",
                                   "%s.source 必须写明来源（可追溯），否则拒绝回填" % path,
                                   path + ".source"))
                continue
            if _blank(method):
                issues.append(_err("missing_source",
                                   "%s.method 必须写明采集方法，否则拒绝回填" % path,
                                   path + ".method"))
                continue
            # 样本数
            if not _is_int(samples) or samples < MIN_SAMPLES:
                issues.append(_err("insufficient_samples",
                                   "%s.samples 至少 %d（单次极值不算证据），实际为 %r"
                                   % (path, MIN_SAMPLES, samples), path + ".samples"))
                continue

            cur_val = cur.get(field)
            if not _is_num(cur_val):
                issues.append(_err("limits_field_missing",
                                   "joint_limits.yaml 中 %s.%s 不是数值，无法回填"
                                   % (name, field), path))
                continue
            if mode == "conservative":
                new_val = min(float(value), float(cur_val))
                reason = "conservative: min(实测, 现值)"
            else:
                new_val = float(value)
                reason = "measured: 直接采用实测值"
            if new_val > float(cur_val) + 1e-12:
                # conservative 模式下不可能发生；measured 模式下是有意放宽
                issues.append(_warn("limit_loosened",
                                    "%s 由 %r 放宽到 %r（measured 模式，允许但需评审）"
                                    % (path, cur_val, new_val), path))
            rec = {"current": float(cur_val), "measured": float(value),
                   "new": new_val, "unit": unit, "source": _one_line(src),
                   "method": _one_line(method), "samples": samples,
                   "reason": reason}
            updates.setdefault(name, {})[field] = rec
            if abs(new_val - float(cur_val)) > 1e-12:
                changed.append(path)

    ok = not any(i["level"] == "error" for i in issues)
    return {"ok": ok, "issues": issues, "updates": updates, "changed": changed}


def format_diff(report):
    """把回填计划渲染成逐关节差异摘要（人可读）。"""
    lines = []
    for name in JOINT_NAMES:
        fields = report["updates"].get(name)
        if not fields:
            continue
        for field in FIELDS:
            rec = fields.get(field)
            if not rec:
                continue
            lines.append("  %-8s %-12s %s → %s   (实测 %s %s, %s)"
                         % (name, field, _fmt_num(rec["current"]), _fmt_num(rec["new"]),
                            _fmt_num(rec["measured"]), rec["unit"], rec["reason"]))
    return "\n".join(lines)


# ------------------------------------------------------------------ 写盘
def _strip_existing_provenance(text):
    """去掉上一次回填留下的记录块（记录块总是在文件尾）。"""
    lines = text.splitlines()
    for i, line in enumerate(lines):
        if line.startswith(PROVENANCE_MARKER):
            return "\n".join(lines[:i]).rstrip("\n")
    return text


def render_limits_text(text, updates, provenance_lines):
    """按行替换数值，**保留原有全部注释**，并在文件尾追加来源记录。"""
    text = _strip_existing_provenance(text)
    out_lines = []
    for line in text.splitlines():
        m = _JOINT_LINE_RE.match(line)
        if m:
            name = "joint%s" % m.group("idx")
            fields = updates.get(name)
            if fields:
                body = m.group("body")

                def _sub(fm):
                    rec = fields.get(fm.group("key"))
                    if not rec:
                        return fm.group(0)
                    return "%s%s%s" % (fm.group("key"), fm.group("sp"), _fmt_num(rec["new"]))

                body = _FIELD_RE.sub(_sub, body)
                line = m.group("head") + body
        out_lines.append(line)
    result = "\n".join(out_lines)
    if not result.endswith("\n"):
        result += "\n"
    if provenance_lines:
        result += "\n" + "\n".join(provenance_lines) + "\n"
    return result


def provenance_lines(baseline, report, mode, date_str, baseline_path):
    """生成"实测限值回填记录"注释块（追加在文件尾）。"""
    lines = [
        PROVENANCE_MARKER + "（apply_measured_limits.py 自动追加，勿手改）──",
        "# 回填日期: %s" % date_str,
        "# 基线来源: %s（version=%s, status=%s, captured_at=%s）"
        % (baseline_path, baseline.get("version"), baseline.get("status"),
           baseline.get("captured_at")),
        "# 回填模式: %s" % mode,
        "# 逐关节来源（source | method | samples）:",
    ]
    for name in JOINT_NAMES:
        fields = report["updates"].get(name)
        if not fields:
            continue
        for field in FIELDS:
            rec = fields.get(field)
            if not rec:
                continue
            lines.append("#   %-8s %-12s source=%s | method=%s | samples=%s"
                         % (name, field, rec["source"], rec["method"], rec["samples"]))
    return lines


def write_limits(path, text):
    """原子写盘（先写临时文件再替换），避免中途失败把限值文件写坏。"""
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as fh:
        fh.write(text)
    os.replace(tmp, path)


# ------------------------------------------------------------------ CLI
def main(argv=None):
    ap = argparse.ArgumentParser(
        prog="apply_measured_limits",
        description="把 measured_limits.yaml 的实测 acc/jerk 回填进 joint_limits.yaml")
    ap.add_argument("--ws", default=WS_DEFAULT, help="工作区根（默认按脚本位置推断）")
    ap.add_argument("--baseline", default=None, help="measured_limits.yaml 路径")
    ap.add_argument("--limits", default=None, help="joint_limits.yaml 路径")
    ap.add_argument("--mode", default="conservative", choices=["conservative", "measured"],
                    help="conservative=取 min(实测,现值)（默认）；measured=直接采用实测值")
    ap.add_argument("--dry-run", action="store_true", help="只打印 diff，不写盘")
    ap.add_argument("--date", default=None, help="回填日期（默认今天，YYYY-MM-DD）")
    ap.add_argument("--no-build-copy", action="store_true",
                    help="不刷新 build/model/joint_limits.yaml（默认刷新）")
    ap.add_argument("--json", dest="as_json", action="store_true", help="以 JSON 输出报告")
    args = ap.parse_args(argv)

    paths = default_paths(args.ws)
    baseline_path = args.baseline or paths["baseline"]
    limits_path = args.limits or paths["limits"]
    date_str = args.date or datetime.date.today().isoformat()

    try:
        baseline = load_yaml(baseline_path, "实测限值基线")
        limits = load_yaml(limits_path, "关节限值文件")
    except BackfillError as exc:
        print("[拒绝] %s" % exc)
        return 2

    report = plan_backfill(baseline, limits, mode=args.mode)
    errors = [i for i in report["issues"] if i["level"] == "error"]
    warns = [i for i in report["issues"] if i["level"] == "warn"]

    if args.as_json:
        print(json.dumps({"ok": report["ok"], "issues": report["issues"],
                          "changed": report["changed"], "mode": args.mode,
                          "dry_run": bool(args.dry_run)},
                         ensure_ascii=False, indent=2))
    else:
        print("基线: %s" % baseline_path)
        print("限值: %s" % limits_path)
        print("模式: %s%s" % (args.mode, "（dry-run，不写盘）" if args.dry_run else ""))
        print("")
        if errors:
            print("拒绝回填（%d 项）：" % len(errors))
            for i in errors:
                loc = ("[%s] " % i["path"]) if i["path"] else ""
                print("  ERROR %s%s: %s" % (loc, i["code"], i["message"]))
        for i in warns:
            loc = ("[%s] " % i["path"]) if i["path"] else ""
            print("  WARN  %s%s: %s" % (loc, i["code"], i["message"]))
        diff = format_diff(report)
        if diff:
            print("\n逐关节差异:")
            print(diff)
        if not report["changed"] and not errors:
            print("\n无差异：实测值与现值一致（或更宽松，保守模式不改动）。")

    if errors:
        return 3
    if args.dry_run:
        return 0
    if not report["updates"]:
        return 0

    with open(limits_path, "r", encoding="utf-8") as fh:
        original = fh.read()
    prov = provenance_lines(baseline, report, args.mode, date_str, baseline_path)
    new_text = render_limits_text(original, report["updates"], prov)
    write_limits(limits_path, new_text)
    if not args.as_json:
        print("\n已回填 %s（%d 项数值）" % (limits_path, len(report["changed"])))

    # 机器本地的 build/model 副本（rmctl init 物化、gitignore）：不刷新则不生效
    build_copy = paths["build_copy"]
    if (not args.no_build_copy) and os.path.exists(build_copy):
        shutil.copyfile(limits_path, build_copy)
        if not args.as_json:
            print("已同步刷新 build/model/joint_limits.yaml（rmctl 实际读取这一份）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
