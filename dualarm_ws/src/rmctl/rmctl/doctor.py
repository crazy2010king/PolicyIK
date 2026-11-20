"""doctor 体检内核（S09/T6）：检查项构造 + 自检覆盖矩阵 + 机器可读输出 + 退出码口径。

``rmctl doctor`` 的人读输出与**既有检查项集合/文案保持不变**；本模块把
「检查项构造」从 CLI 里搬出来（:func:`collect_checks`），让同一份逻辑可被
三处复用：

* CLI ``rmctl doctor``（人读文本 + ``--json`` / ``--report``）；
* 一键回归编排器的 ``doctor`` 段（读 JSON 报告 → 段级结果）；
* 单测（构造检查项集合 → 断言退出码映射）。

退出码口径（全项目统一 **0 / 3 / 4**，与 ``rm_executor.contract.EXIT_CODES``
同值；一致性由 ``tests/test_doctor_json.py`` 守住）：

======  ==========================================================
退出码   含义
======  ==========================================================
0       全部检查项 PASS（**INFO 级未通过不算失败**，与既有
        「limits:measured 是 INFO 不阻塞」语义一致）
3       存在需人工 / 外部条件的项（RECOVERABLE / NEEDS_HUMAN /
        BLOCKED）——「环境没准备好」，不是代码回归
4       存在硬红线项（FAIL：黑名单调用、SDK 核心符号缺失、夹爪配置非法）
======  ==========================================================

自检覆盖矩阵（:data:`COVERAGE_CATEGORIES`）
-------------------------------------------

设计 §6.2「自检覆盖」点名 **网络 / 供电 / 夹爪 / 零位 / FK / 限值 /
控制器状态**，本模块再补 **依赖与模型 / 碰撞 / 硬件适配层** 三类，共
**10 类 ≥ 8**。每类映射到**具体 check name**：

* 映射得到的类别 → ``covered=true``，并给出该类当前的 ok / blocked / fail 状态；
* **映射不到**（该类检查项因上游失败被级联跳过、或根本不存在）的类别 →
  **显式** 标 ``covered=false`` + 缺口说明 + 可粘贴 next_action。
  **绝不**新增「一切正常」的水条目凑数。

典型级联：厂商库不可导入 → ``model:load`` 失败 → FK / 碰撞 / 限值一致性
三条检查被**跳过**（不是通过）→ 覆盖矩阵里这几类必须标 ``covered=false``。

真机 / 外部条件类检查项
-----------------------

:data:`EXTERNAL_CONDITION_CHECKS` 列出「阻塞原因天然在真机窗口/外部条件」
的检查项（供电 / 控制器状态 / 网络连通 / 零位采集）。它们**不阻塞**一键回归
的判定（doctor 段本身执行成功即 pass），但在「接入新机型」这类**离线**体验
度量里也不算「接入报错」（详见 ``rmctl.regression.experience``）。

安全约束（Q3）
--------------

本模块输出会被脚本消费，因此：

* 字段名**稳定**（``checks[]`` / ``coverage_matrix[]`` / ``status`` /
  ``verdict`` / ``exit_code``），只**增不改**；
* JSON **不含**任何凭据 / 内网口令（只输出检查项结论与可粘贴命令）。
"""
from __future__ import annotations

import os
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np
import yaml

from . import baseline as bl
from . import paths
from .model import Model

__all__ = [
    "COVERAGE_CATEGORIES",
    "EXIT_BLOCKED",
    "EXIT_ERROR",
    "EXIT_OK",
    "EXTERNAL_CONDITION_CHECKS",
    "LEVELS",
    "build_report",
    "collect_checks",
    "coverage_matrix",
    "exit_code_for",
    "status_for",
]

# ---------------------------------------------------------------- 级别与退出码
#: 检查项 level 词表。**通过的检查项 level 恒为 INFO**（既有行为：
#: ``level`` 表达的是「未通过时的严重度」），未通过时取下列值之一。
LEVELS = ("INFO", "RECOVERABLE", "NEEDS_HUMAN", "BLOCKED", "FAIL")

#: 硬红线级别（唯一会让退出码变成 4 的级别）
FAIL_LEVEL = "FAIL"

#: 需人工 / 外部条件的级别（退出码 3）
BLOCKED_LEVELS = ("RECOVERABLE", "NEEDS_HUMAN", "BLOCKED")

#: 退出码（与 ``rm_executor.contract.EXIT_CODES`` 同值：ok=0 / blocked=3 /
#: error=4）。此处**不**依赖执行层包（doctor 必须能在最小环境里跑），
#: 一致性由单测断言。
EXIT_OK = 0
EXIT_BLOCKED = 3
EXIT_ERROR = 4

#: 阻塞原因天然在真机窗口 / 外部条件的检查项（不计入「接入新机型」报错数）
EXTERNAL_CONDITION_CHECKS = (
    "network:plan",
    "zero:baseline",
    "power:supply",
    "controller:state",
)

#: 自检覆盖矩阵（设计 §6.2 点名 7 类 + 本模块补 3 类 = 10 类 ≥ 8）
COVERAGE_CATEGORIES: Tuple[Dict[str, Any], ...] = (
    {"category": "network", "label": "网络连通",
     "checks": ("network:plan",)},
    {"category": "power", "label": "供电与共地",
     "checks": ("power:supply",)},
    {"category": "gripper", "label": "夹爪",
     "checks": ("hw:gripper",)},
    {"category": "zero", "label": "零位基线",
     "checks": ("zero:baseline",)},
    {"category": "fk", "label": "FK 一致性",
     "checks": ("fk:self_vs_tesseract",)},
    {"category": "limits", "label": "限值",
     "checks": ("limits:consistent", "limits:measured")},
    {"category": "controller", "label": "控制器状态",
     "checks": ("controller:state",)},
    {"category": "deps_model", "label": "依赖与模型",
     "checks": ("dep:nanobind", "model:load", "model:urdf", "model:srdf",
                "model:joint_limits")},
    {"category": "collision", "label": "碰撞（零位自碰撞）",
     "checks": ("collision:home",)},
    {"category": "hardware_layer", "label": "硬件适配层（SDK/符号/白黑名单/限值源）",
     "checks": ("hw:sdk", "hw:symbols", "hw:contract", "hw:limits_source")},
)


# ---------------------------------------------------------------- 小工具
def _check(name: str, ok: bool, detail: str, action: Optional[str] = None,
           level: str = "RECOVERABLE") -> Dict[str, Any]:
    """构造一条检查项（字段与既有 doctor 完全一致）。

    **通过的检查项 level 恒为 INFO**（既有 ``cmd_doctor.add`` 的语义）。
    """
    return {"name": name, "ok": bool(ok), "detail": detail,
            "action": action, "level": "INFO" if ok else level}


def _hw_checks(ws: str) -> List[Dict[str, Any]]:
    """5 条 ``hw:*`` 检查项。

    **唯一实现仍在** :mod:`rmctl.cli`（硬件适配层判定与硬件层强相关，
    不在这里复制一份）；此处**延迟导入**以避开 ``cli ↔ doctor`` 的循环导入。
    """
    from . import cli  # 延迟导入：cli 顶层 import 本模块

    return list(cli._hw_checks(ws))


def _load_baseline_yaml(ws: str, kind: str) -> Tuple[Optional[Dict[str, Any]],
                                                      str, str]:
    """读 ``workcell/baselines/<kind>.yaml``。

    返回 ``(data, path, error)``：``data`` 为 ``None`` 且 ``error`` 为空
    表示**文件缺失**；``error`` 非空表示**解析失败**。
    """
    path = os.path.join(paths.baselines_dir(ws), "%s.yaml" % kind)
    if not os.path.exists(path):
        return None, path, ""
    try:
        with open(path, "r", encoding="utf-8") as fh:
            data = yaml.safe_load(fh) or {}
    except Exception as exc:  # noqa: BLE001 - 解析失败要如实报告
        return None, path, "%s: %s" % (type(exc).__name__, exc)
    if not isinstance(data, dict):
        return None, path, "顶层不是映射"
    return data, path, ""


# ---------------------------------------------------------------- 新增一等检查项
def _network_check(ws: str) -> Dict[str, Any]:
    """``network:plan`` —— 网络连通性（依据 ``network_plan.yaml`` 的 status）。"""
    data, path, err = _load_baseline_yaml(ws, "network_plan")
    if err:
        return _check("network:plan", False,
                      "网络基线解析失败（%s）: %s" % (err, path),
                      "修复 workcell/baselines/network_plan.yaml 后重跑 rmctl doctor",
                      "RECOVERABLE")
    if data is None:
        return _check("network:plan", False,
                      "缺网络基线文件: %s" % path,
                      "python3 -m rmctl baseline scaffold  # 生成 network_plan.yaml",
                      "RECOVERABLE")
    status = str(data.get("status") or "unknown")
    if status == "measured":
        return _check("network:plan", True,
                      "网络基线 status=measured（连通性已实测确认）: %s" % path,
                      "python3 -m rmctl connect --left <ip> --right <ip>")
    blocked = data.get("blocked") or {}
    reason = str(blocked.get("reason") or "").strip()
    action = str(blocked.get("next_action") or "").strip()
    return _check(
        "network:plan", False,
        "网络基线 status=%s（连通性未实测确认）%s"
        % (status, ("：" + reason) if reason else ""),
        action or ("真机窗口内 ping 192.168.1.18 / TCP 8080 连通性检查通过后，"
                   "把 network_plan.yaml 的 status 改为 measured"),
        "NEEDS_HUMAN")


def _zero_check(ws: str) -> Dict[str, Any]:
    """``zero:baseline`` —— 真机零位基线（依据 ``zero_baseline.yaml`` 的 status）。"""
    data, path, err = _load_baseline_yaml(ws, "zero_baseline")
    if err:
        return _check("zero:baseline", False,
                      "零位基线解析失败（%s）: %s" % (err, path),
                      "修复 workcell/baselines/zero_baseline.yaml 后重跑 rmctl doctor",
                      "RECOVERABLE")
    if data is None:
        return _check("zero:baseline", False,
                      "缺零位基线文件: %s" % path,
                      "python3 -m rmctl baseline scaffold  # 生成 zero_baseline.yaml",
                      "RECOVERABLE")
    status = str(data.get("status") or "unknown")
    if status == "measured":
        return _check("zero:baseline", True,
                      "零位基线 status=measured（零位已采集并复测）: %s" % path,
                      "python3 -m rmctl.fk_cross  # 复核 FK 一致性")
    blocked = data.get("blocked") or {}
    reason = str(blocked.get("reason") or "").strip()
    action = str(blocked.get("next_action") or "").strip()
    return _check(
        "zero:baseline", False,
        "零位基线 status=%s（零位未采集）%s" % (status, ("：" + reason) if reason else ""),
        action or ("真机窗口按 docs/真机基线与实测限值归档说明.md 第 3 节"
                   "采集零位后，把 zero_baseline.yaml 的 status 改为 measured"),
        "NEEDS_HUMAN")


def _power_check(hw_checks: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    """``power:supply`` —— 供电与共地（离线无法自动确认 → 需人工）。

    处置口径与 S06 的 ``needs_param`` 一致：**无真机信号时**给
    ``level=NEEDS_HUMAN`` + 指名「检查外部供电与共地」的可粘贴动作，
    **绝不**伪造「供电正常」的结论。
    """
    sdk_ok = any(c.get("name") == "hw:sdk" and c.get("ok") for c in hw_checks)
    if sdk_ok:
        return _check("power:supply", True,
                      "真机 SDK 在位（供电/共地可在真机窗口在线确认）",
                      "python3 -m rmctl connect --left <ip> --right <ip>  "
                      "# 真机窗口确认供电与共地")
    return _check(
        "power:supply", False,
        "无真机信号：离线环境无法自动确认供电与共地"
        "（按 S06 needs_param 口径需人工确认，不伪造结论）",
        "检查外部供电与共地（真机窗口确认上电 / 急停 / 共地），"
        "再执行 python3 -m rmctl connect --left <ip> --right <ip>",
        "NEEDS_HUMAN")


def _controller_check(hw_checks: Sequence[Dict[str, Any]],
                      net_status: str) -> Dict[str, Any]:
    """``controller:state`` —— 控制器状态（真机未连接 → blocked）。

    「已连接」的判据 = **真机 SDK 在位** 且 **网络基线已实测（measured）**；
    两者缺一即视为「未连接」，动作指名 ``rmctl connect``。
    """
    sdk_ok = any(c.get("name") == "hw:sdk" and c.get("ok") for c in hw_checks)
    if sdk_ok and net_status == "measured":
        return _check("controller:state", True,
                      "控制器在位（SDK 已装载、网络基线 status=measured）",
                      "python3 -m rmctl connect --left <ip> --right <ip>  # 复核握手")
    return _check(
        "controller:state", False,
        "真机未连接（SDK 在位=%s，网络基线 status=%s）"
        % ("是" if sdk_ok else "否", net_status),
        "python3 -m rmctl connect --left <ip> --right <ip>  # 连真机并做握手自检",
        "BLOCKED")


# ---------------------------------------------------------------- 检查项构造
def collect_checks(ws: Optional[str] = None,
                   group: str = "rm75_arm") -> List[Dict[str, Any]]:
    """构造 doctor 的**全部**检查项（**唯一实现**，CLI 与回归段共用）。

    返回 ``[{name, ok, detail, action, level}, ...]``；顺序 = 展示顺序。
    检查项集合与文案与既有 ``cmd_doctor`` **逐字一致**（只新增 4 条一等
    检查项：``network:plan`` / ``zero:baseline`` / ``power:supply`` /
    ``controller:state``）。
    """
    ws = ws or paths.workspace_root()
    checks: List[Dict[str, Any]] = []

    checks.append(_check("workspace", paths._find_src(ws) is not None,
                         "工作区: %s" % ws, "检查 RM_WS 或目录结构"))
    for label, p in (("urdf", paths.urdf_path(ws)), ("srdf", paths.srdf_path(ws)),
                     ("joint_limits", paths.joint_limits_path(ws))):
        checks.append(_check("model:" + label, os.path.exists(p), p,
                             "rmctl init / 重新生成模型"))
    try:
        import tesseract_robotics  # noqa: F401
        checks.append(_check("dep:nanobind", True,
                             "tesseract_robotics %s" % tesseract_robotics.__version__))
    except Exception as exc:  # noqa: BLE001
        checks.append(_check("dep:nanobind", False, str(exc),
                             "pip install tesseract-robotics-nanobind==0.35.0.8",
                             "NEEDS_HUMAN"))

    model = None
    try:
        model = Model(ws=ws, group=group)
        checks.append(_check("model:load", True, "组=%s 关节=%d 连杆=%d" %
                             (model.group, len(model.joint_names),
                              len(model.link_names))))
    except Exception as exc:  # noqa: BLE001
        checks.append(_check("model:load", False, repr(exc),
                             "检查 URDF/SRDF 与插件配置", "NEEDS_HUMAN"))

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
        checks.append(_check("fk:self_vs_tesseract", worst < 1e-6,
                             "20 组随机位形最大位置偏差 %.3e m" % worst,
                             "检查 URDF 清洗结果（clean_urdf.py）"))
        # 零位自碰撞
        hits = model.urdf.collisions(np.zeros(len(model.joint_names)))
        checks.append(_check("collision:home", not hits,
                             "零位自碰撞对: %s" % (hits or "无"),
                             "检查 ACM（scripts/gen_srdf.py）"))
        # 限值一致性
        mismatch = []
        for j in model.joint_names:
            ul = model.urdf.joints[[x["name"] for x in model.urdf.rev].index(j)]
            lim = model.limit(j)
            if lim.get("upper") is not None and abs(ul["upper"] - lim["upper"]) > 1e-4:
                mismatch.append("%s urdf=%.4f yaml=%.4f" % (j, ul["upper"], lim["upper"]))
            if lim.get("lower") is not None and abs(ul["lower"] - lim["lower"]) > 1e-4:
                mismatch.append("%s urdf=%.4f yaml=%.4f" % (j, ul["lower"], lim["lower"]))
        checks.append(_check("limits:consistent", not mismatch,
                             "URDF vs joint_limits.yaml: %s" % (mismatch or "一致"),
                             "以 joint_limits.yaml 为准，必要时重新清洗 URDF"))
        # 硬件
        checks.append(_check("hardware:connected", False,
                             "未检测到真机连接（原型阶段为模拟执行）",
                             "rmctl connect --left <ip> --right <ip>", "INFO"))

    # 基线齐备度（基线校验只依赖 yaml+numpy，与厂商库无关）
    try:
        report = bl.validate_all(ws)
        missing = report["missing"]
        present = len(bl.KINDS) - len(missing)
        checks.append(_check("baselines:present", not missing,
                             "四类基线 %d/%d 齐备%s"
                             % (present, len(bl.KINDS),
                                ("（缺: %s）" % ", ".join(missing)) if missing else ""),
                             "python3 -m rmctl baseline scaffold"))
        ml = report["kinds"].get("measured_limits", {})
        ml_status = ml.get("status")
        checks.append(_check("limits:measured", ml_status == "measured",
                             "measured_limits.status=%s（acc/jerk 实测值未回填）"
                             % (ml_status or "缺失"),
                             "真机窗口执行 python3 -m rmctl.hwcollect ladder_acc_jerk "
                             "并回填 workcell/baselines/measured_limits.yaml"
                             "（见 docs/真机基线与实测限值归档说明.md）",
                             "INFO"))
    except Exception as exc:  # noqa: BLE001
        checks.append(_check("baselines:present", False, repr(exc),
                             "python3 -m rmctl baseline scaffold"))

    # 硬件适配层（rm_hw_bridge）5 条：SDK 在位 / 22 符号 / 契约静态审计 /
    # 限值来源 / 夹爪配置
    hw_checks = _hw_checks(ws)
    for chk in hw_checks:
        checks.append(_check(chk["name"], chk["ok"], chk["detail"],
                             chk["action"], chk["level"]))

    # ---- 新增一等检查项（网络 / 零位 / 供电 / 控制器状态）----------------
    net = _network_check(ws)
    checks.append(net)
    checks.append(_zero_check(ws))
    checks.append(_power_check(hw_checks))
    net_data, _net_path, _net_err = _load_baseline_yaml(ws, "network_plan")
    net_status = str((net_data or {}).get("status") or "unknown")
    checks.append(_controller_check(hw_checks, net_status))

    return checks


# ---------------------------------------------------------------- 退出码映射
def exit_code_for(checks: Sequence[Dict[str, Any]]) -> int:
    """检查项集合 → 退出码（**纯函数**，0 / 3 / 4）。

    * 有 ``level == FAIL`` 的未通过项 → **4**（硬红线）；
    * 否则有未通过且 level 不是 INFO 的项 → **3**（需人工 / 外部条件）；
    * 否则 → **0**（全 PASS；INFO 级未通过不算失败）。
    """
    failed = [c for c in checks if not c.get("ok")]
    if any(str(c.get("level")) == FAIL_LEVEL for c in failed):
        return EXIT_ERROR
    if any(str(c.get("level")) != "INFO" for c in failed):
        return EXIT_BLOCKED
    return EXIT_OK


def status_for(exit_code: int) -> str:
    """退出码 → 执行层 status 词表（``ok`` / ``blocked`` / ``error``）。

    词表**优先**取 ``rm_executor.contract``（唯一来源）；取不到时用等值
    字面量兜底（doctor 必须能在最小环境里跑，不因此崩掉）。
    """
    vocab = {"ok": "ok", "blocked": "blocked", "error": "error"}
    try:
        from .regression import segments as _seg

        contract = _seg.contract()
        vocab = {"ok": contract.STATUS_OK,
                 "blocked": contract.STATUS_BLOCKED,
                 "error": contract.STATUS_ERROR}
    except Exception:  # noqa: BLE001 - 兜底：等值字面量
        pass
    if exit_code == EXIT_OK:
        return vocab["ok"]
    if exit_code == EXIT_BLOCKED:
        return vocab["blocked"]
    return vocab["error"]


# ---------------------------------------------------------------- 覆盖矩阵
def _missing_action(missing: Sequence[str]) -> str:
    """缺检查项时的**可粘贴**下一步（按缺失项类型给不同的修复命令）。"""
    names = set(missing)
    if names & {"fk:self_vs_tesseract", "collision:home", "limits:consistent"}:
        return ("先让模型可加载（python3 -m rmctl init 物化 build/model；"
                "厂商库缺失时 pip install tesseract-robotics-nanobind==0.35.0.8）"
                "后重跑 rmctl doctor")
    if names & {"hw:sdk", "hw:symbols", "hw:contract", "hw:limits_source",
                "hw:gripper"}:
        return ("先修复硬件适配层（export PYTHONPATH=$PWD/dualarm_ws/src/"
                "rm_hw_bridge:$PYTHONPATH）后重跑 rmctl doctor")
    return "先补齐上述检查项的执行条件后重跑 rmctl doctor"


def coverage_matrix(checks: Sequence[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """检查项集合 → 自检覆盖矩阵（设计 §6.2「自检覆盖 ≥ 8 类」）。

    每个类别一条记录::

        {"category", "label", "checks", "covered", "status", "detail",
         "gap", "next_action"}

    * ``covered=true`` —— 该类映射到的检查项**全部出现**（跑了，可能没过）；
      ``status`` 给出该类当前状态（``ok`` / ``blocked`` / ``fail``）；
    * ``covered=false`` —— 至少一个检查项**没出现**（被级联跳过或根本不存在）：
      ``gap`` 说明缺口，``next_action`` 给可粘贴的修复命令。

    **绝不**用「一切正常」的水条目凑数。
    """
    by_name = {str(c.get("name")): c for c in checks}
    matrix: List[Dict[str, Any]] = []
    for cat in COVERAGE_CATEGORIES:
        names = list(cat["checks"])
        present = [n for n in names if n in by_name]
        missing = [n for n in names if n not in by_name]
        entry: Dict[str, Any] = {
            "category": cat["category"],
            "label": cat["label"],
            "checks": names,
            "covered": not missing,
            "status": "ok",
            "detail": "",
            "gap": "",
            "next_action": "",
        }
        if missing:
            entry["covered"] = False
            entry["status"] = "missing"
            entry["gap"] = ("缺检查项: %s（未执行，**不是通过**）" % ", ".join(missing))
            entry["next_action"] = _missing_action(missing)
            matrix.append(entry)
            continue
        failed = [by_name[n] for n in present if not by_name[n].get("ok")]
        if not failed:
            entry["status"] = "ok"
            entry["detail"] = "%d 项检查全部通过" % len(present)
        else:
            hard = [c for c in failed if str(c.get("level")) == FAIL_LEVEL]
            entry["status"] = "fail" if hard else "blocked"
            entry["detail"] = ("%d/%d 项未通过: %s"
                               % (len(failed), len(present),
                                  ", ".join(str(c.get("name")) for c in failed)))
            entry["gap"] = ("检查项未通过: %s"
                            % ", ".join(str(c.get("name")) for c in failed))
            entry["next_action"] = next((str(c.get("action")) for c in failed
                                         if c.get("action")), "") or \
                "按 doctor 输出里的「下一步」逐条处理后重跑 rmctl doctor"
        matrix.append(entry)
    return matrix


# ---------------------------------------------------------------- 报告
def build_report(ws: str, checks: Sequence[Dict[str, Any]],
                 matrix: Optional[Sequence[Dict[str, Any]]] = None
                 ) -> Dict[str, Any]:
    """检查项 + 覆盖矩阵 → **机器可读报告**（``--json`` / ``--report``）。

    字段稳定（只增不改）：``schema_version`` / ``tool`` / ``workspace`` /
    ``status`` / ``verdict`` / ``exit_code`` / ``reason`` / ``next_action`` /
    ``summary`` / ``checks[]`` / ``coverage_matrix[]``。
    """
    matrix = list(matrix if matrix is not None else coverage_matrix(checks))
    exit_code = exit_code_for(checks)
    status = status_for(exit_code)
    verdict = {EXIT_OK: "pass", EXIT_BLOCKED: "blocked",
               EXIT_ERROR: "fail"}.get(exit_code, "fail")

    failed = [c for c in checks if not c.get("ok")]
    hard = [c for c in failed if str(c.get("level")) == FAIL_LEVEL]
    blocked_items = [c for c in failed if str(c.get("level")) != "INFO"]
    covered = [m for m in matrix if m.get("covered")]
    uncovered = [m for m in matrix if not m.get("covered")]

    if exit_code == EXIT_OK:
        reason = "全部检查项通过（共 %d 条，覆盖 %d/%d 类现场问题）" % (
            len(checks), len(covered), len(matrix))
        next_action = ""
    elif exit_code == EXIT_ERROR:
        reason = "有 %d 项硬红线未通过（共 %d 条）：%s" % (
            len(hard), len(checks),
            ", ".join(str(c.get("name")) for c in hard))
        next_action = next((str(c.get("action")) for c in hard
                            if c.get("action")), "")
    else:
        reason = "有 %d 项需人工 / 外部条件（共 %d 条）：%s" % (
            len(blocked_items), len(checks),
            ", ".join(str(c.get("name")) for c in blocked_items))
        next_action = next((str(c.get("action")) for c in blocked_items
                            if c.get("action")), "")

    return {
        "schema_version": 1,
        "tool": "rmctl doctor",
        "workspace": ws,
        "status": status,
        "verdict": verdict,
        "exit_code": exit_code,
        "reason": reason,
        "next_action": next_action,
        "summary": {
            "total": len(checks),
            "passed": len(checks) - len(failed),
            "failed": len(failed),
            "hard_failed": len(hard),
            "needs_human": len(blocked_items),
            "categories": len(matrix),
            "covered_categories": len(covered),
            "uncovered_categories": len(uncovered),
            "uncovered": [str(m.get("category")) for m in uncovered],
        },
        "checks": [dict(c) for c in checks],
        "coverage_matrix": matrix,
    }
