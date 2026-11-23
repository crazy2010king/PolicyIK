"""体验基准 E1–E4 的**纯函数小工具**（S09/T5）：限值读取 / 代码差异 / doctor 解析。

与 :mod:`rmctl.regression.experience` 的分工：

* ``experience`` —— 场景声明装载 + E1–E4 的**测量实现**（跑命令、读 trace）；
* ``experience_report`` —— 指标构造 / 纪律校验 / 报告落盘；
* 本模块 —— **零 I/O 副作用**的小工具（读文件、解析文本、数行数）。

这里的四个函数都是**纯函数**（同样的输入 → 同样的输出），可单独测；
它们**不**知道指标口径，也不 import 厂商内核 / 硬件适配层。
"""
from __future__ import annotations

import difflib
import os
import re
import xml.etree.ElementTree as ET
from typing import Any, Dict, List, Optional, Tuple

__all__ = [
    "LIMIT_TOL",
    "diff_py_lines",
    "parse_doctor",
    "urdf_limits",
    "yaml_limits",
]

#: 限值比较容差（与 doctor 的 ``limits:consistent`` 同口径）
LIMIT_TOL = 1e-4

#: doctor 输出里的检查项行（``  [PASS] name  detail``）
DOCTOR_LINE = re.compile(r"^\s*\[([A-Z_]+)\]\s+(\S+)\s*(.*)$")


def urdf_limits(urdf_path: str) -> Dict[str, Tuple[float, float]]:
    """读 URDF 的关节限值 → ``{joint: (lower, upper)}``（读不到 → {}）。"""
    try:
        tree = ET.parse(urdf_path)
    except (OSError, ET.ParseError):
        return {}
    out: Dict[str, Tuple[float, float]] = {}
    for joint in tree.getroot().iter("joint"):
        name = joint.get("name")
        limit = joint.find("limit")
        if not name or limit is None:
            continue
        try:
            lower = float(limit.get("lower"))
            upper = float(limit.get("upper"))
        except (TypeError, ValueError):
            continue
        out[name] = (lower, upper)
    return out


def yaml_limits(limits_path: str) -> Dict[str, Tuple[float, float]]:
    """读 ``joint_limits.yaml`` → ``{joint: (lower, upper)}``。"""
    import yaml

    try:
        with open(limits_path, "r", encoding="utf-8") as fh:
            data = yaml.safe_load(fh)
    except (OSError, ValueError):
        return {}
    if not isinstance(data, dict):
        return {}
    out: Dict[str, Tuple[float, float]] = {}
    for group in data.values():
        if not isinstance(group, dict):
            continue
        for joint, spec in group.items():
            if not isinstance(spec, dict):
                continue
            lower = spec.get("lower")
            upper = spec.get("upper")
            if lower is None or upper is None:
                continue
            try:
                out[str(joint)] = (float(lower), float(upper))
            except (TypeError, ValueError):
                continue
    return out


def diff_py_lines(dir_a: str, dir_b: str) -> Tuple[int, List[str]]:
    """两个目录树里 ``.py`` 文件的逐行差异行数（新增 + 删除）。

    一侧缺失的文件按**实际行数**计入（新增 / 删除），不是「1 行」。
    """
    def _collect(root: str) -> Dict[str, str]:
        out: Dict[str, str] = {}
        for base, _dirs, files in os.walk(root):
            if "__pycache__" in base:
                continue
            for name in files:
                if not name.endswith(".py"):
                    continue
                full = os.path.join(base, name)
                out[os.path.relpath(full, root)] = full
        return out

    files_a = _collect(dir_a)
    files_b = _collect(dir_b)
    changed = 0
    evidence: List[str] = []
    for rel in sorted(set(files_a) | set(files_b)):
        path_a = files_a.get(rel)
        path_b = files_b.get(rel)
        if path_a is None or path_b is None:
            only = path_b or path_a
            try:
                with open(only, "r", encoding="utf-8") as fh:
                    lines = fh.readlines()
            except OSError:
                lines = []
            changed += len(lines)
            evidence.append("%s: %s（%d 行）"
                            % (rel, "新增" if path_a is None else "删除",
                               len(lines)))
            continue
        try:
            with open(path_a, "r", encoding="utf-8") as fh:
                lines_a = fh.readlines()
            with open(path_b, "r", encoding="utf-8") as fh:
                lines_b = fh.readlines()
        except OSError:
            continue
        if lines_a == lines_b:
            continue
        diff = list(difflib.unified_diff(lines_a, lines_b, fromfile=rel,
                                         tofile=rel, n=0))
        delta = sum(1 for line in diff
                    if (line.startswith("+") or line.startswith("-"))
                    and not line.startswith(("+++", "---")))
        changed += delta
        evidence.append("%s: %d 行差异" % (rel, delta))
    return changed, evidence


def parse_doctor(stdout: str) -> Dict[str, Any]:
    """解析 doctor 输出 → ``{checks, errors, non_pass, names, error_names}``。

    ``errors`` = 非 PASS 且非 INFO 的检查项条数（与 doctor 自己的「需处理」
    口径一致）；``non_pass`` = 全部非 PASS 条数；``error_names`` = 计入
    ``errors`` 的检查项**名字**（供调用方按检查项粒度做口径过滤）。
    """
    checks = 0
    errors = 0
    non_pass = 0
    names: List[str] = []
    error_names: List[str] = []
    for line in (stdout or "").splitlines():
        matched = DOCTOR_LINE.match(line)
        if not matched:
            continue
        level = matched.group(1)
        name = matched.group(2)
        checks += 1
        names.append(name)
        if level == "PASS":
            continue
        non_pass += 1
        if level != "INFO":
            errors += 1
            error_names.append(name)
    return {"checks": checks, "errors": errors, "non_pass": non_pass,
            "names": names, "error_names": error_names}
