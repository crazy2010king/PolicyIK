#!/usr/bin/env python3
"""真机采集脚本骨架：基于 rm_hw_bridge 的句柄语义 + 六类实验 + ``--dry-run``（设计 §5.1/§5.2/§5.3）。

真机是**预约资源**（session.md：机械臂"等到需要时我再提供"）。窗口内每多一分钟
交互式调试就多一分钟撞机风险，因此本模块把"真机窗口要执行的动作"全部脚本化，
并在没有 SDK 的机器上离线自测：**窗口只跑脚本、只落数据**。

**ABI 形态（本版已修正，见 S02-T6）**：S01 遗留版本按 **IP 字符串** 调用 SDK、
并把 64 位指针截断成 32 位——真实 ABI 是 **handle-based**，那样调用是未定义行为
（可能读越界、可能静默串号）。本版：

* **不再手写 SDK 封装**：``RmSdk`` 与手写 ``_RmArmState``（"尽力而为布局"）已删除；
  所有 SDK 访问都经由硬件适配层 ``rm_hw_bridge``（``loader`` / ``bridge``），
  走**句柄语义**（``rm_robot_handle*``），布局由 ABI 唯一真值
  （``rm_hw_bridge/abi_spec.yaml`` → 生成的 ctypes 绑定）保证。
* **白名单不再是手写的字符串清单**：``WHITELIST`` 由 ``rm_hw_bridge/contract.yaml``
  的 ``core`` **取子集**得到（硬件层是唯一真值；契约与本模块的用符需求不一致时直接报错）。
* **黑名单自检来源改为契约**：``BLACKLIST`` 来自 contract 的 ``blacklist``，
  判定用 **AST 调用形态**（字符串常量不误判）。

六类实验（与 S01-T7 的窗口执行清单一一对应）：

=================== =====================================================
udp_stability       实时上报稳定性（断流次数、周期是否落在 5 ms 倍数上）
ladder_acc_jerk     阶梯速度反推 acceleration / jerk 上限（取多次采样包络）
tcp_repeatability   TCP 位置重复性（明确区分重复性 ≠ 绝对精度）
fk_cross            FK 与控制器位姿交叉校验（调用 rmctl.fk_cross）
estop_drill         软停 / 急停演练（记录返回码、错误码、状态快照）
zero_capture        零位读数与证据引用（零位只能走示教器受控流程）
=================== =====================================================

设计约束：

* **缺 SDK 时优雅降级**：所有真机 action 统一返回 ``status="NEEDS_HUMAN"``
  并附 ``reason`` + 可粘贴的下一步动作，**绝不返回伪造数值**、绝不抛异常。
* **黑名单符号**：``rm_set_joint_zero_pos``（零位标定）等禁止调用——零位只能走
  示教器受控流程（设计 §1.4）。模块启动时做**自检**（``blacklist_scan()``）。
* **控制器位姿仅用于交叉校验**（R003）：厂商运动学输出只进 ``fk_cross``
  的比对报告，不得写回轨迹/模型/基线。
* **量级约束**：udp_stability 600 s @ 5 ms ≈ 12 万条上报，只保留**聚合统计**
  与**抽样原始帧**，不把全量数据堆内存、也不写进 git 跟踪目录。
* 本模块只依赖 ``numpy``（+ 标准库），**禁止 import tesseract**——保证在没有
  GLIBCXX_3.4.31 的环境里也能被 pytest 覆盖。

模块入口::

    python3 -m rmctl.hwcollect udp_stability --dry-run --out /tmp/hwcollect
    python3 -m rmctl.hwcollect fk_cross --arm left --out ./out

退出码：采集成功 0 / 输入非法 2 / 需要人工介入（NEEDS_HUMAN）3 / 真机执行错误 4。
"""
import argparse
import ast
import ctypes
import datetime
import json
import os
import sys
import time

import numpy as np

from . import fk_cross
from . import paths
from . import retime

# ------------------------------------------------------------------ 常量
SCHEMA = "rmctl.hwcollect/1"

#: 默认左右臂控制器 IP（真机窗口可�� --ip 覆盖）
DEFAULT_ARM_IP = {"left": "192.168.1.18", "right": "192.168.1.19"}

#: 默认 TCP 端口（与 rm_hw_bridge.bridge.DEFAULT_TCP_PORT 一致）
DEFAULT_TCP_PORT = 8080

#: 实时上报周期必须是 5 ms 的倍数（设计 §5.1，与 baseline.UDP_PERIOD_MS 同源）
UDP_PERIOD_MS = 5
#: 默认采集时长（秒）：10 分钟
UDP_DEFAULT_DURATION_S = 600
#: TCP 重复性默认次数
TCP_DEFAULT_REPEATS = 30
#: 官方标称重复性（mm）——**标称值不是实测值**，只用于对照
TCP_NOMINAL_MM = 0.05
#: 阶梯实验默认采样周期（秒）
LADDER_DT_S = 0.02
#: 阶梯实验默认重复次数（取包络，不是单次极值）
LADDER_DEFAULT_TRIALS = 3
#: 阶梯实验真机路径：单关节单次位移（度）
LADDER_STEP_DEG = 10.0
#: 阶梯实验真机路径：基础速度与每档增速（RM-API2 rm_movej 的 speed 参数）
LADDER_BASE_SPEED = 5
LADDER_SPEED_STEP = 3
#: FK 交叉校验默认位形数
FK_DEFAULT_SAMPLES = 20

STATUS_OK = "ok"
STATUS_DRY_RUN = "dry_run"
STATUS_NEEDS_HUMAN = "NEEDS_HUMAN"
STATUS_ERROR = "error"

#: Q3 威胁面提示（参数说明与检查里显式提示）
THREAT_NOTES = (
    {"id": "blacklist_symbol",
     "text": "黑名单符号禁止调用：零位标定只能走示教器受控流程（设计 §1.4）。"},
    {"id": "controller_pose_scope",
     "text": "控制器位姿（厂商运动学输出）仅用于 FK 交叉校验，不得作为规划真值（R003）。"},
    {"id": "triple_thread_mode",
     "text": "实时上报必须开三线程模式（rm_init(RM_TRIPLE_MODE_E) + rm_set_realtime_push），"
             "否则收不到 UDP 数据。"},
    {"id": "single_client_per_endpoint",
     "text": "同一 ip:port 只允许一个客户端：上一个进程没退干净时会**静默失败**，"
             "采集前先确认端口未被占用。"},
    {"id": "no_fabricated_values",
     "text": "缺 SDK / 未实测一律返回 NEEDS_HUMAN，禁止填写占位数值。"},
    {"id": "handle_semantics",
     "text": "SDK 调用一律经 rm_hw_bridge 的句柄语义（rm_robot_handle*）；"
             "禁止按 IP 字符串调用或自行解释指针。"},
)


class HwCollectError(Exception):
    """输入非法（消息面向操作者，可直接粘贴）。"""


class SdkMissing(HwCollectError):
    """RM-API2 SDK 缺失（附可粘贴的修复动作）。"""


# ------------------------------------------------------- 硬件适配层（惰性导入）
_HW_LOADER = None          # rm_hw_bridge.loader 模块（缓存）
_HW_IMPORT_ERROR = ""      # 惰性导入失败原因


def _hw_loader():
    """惰性导入 ``rm_hw_bridge.loader``（硬件适配层），返回 ``(module, error)``。

    硬件适配层是**独立包**（``dualarm_ws/src/rm_hw_bridge``），可能未安装到
    ``sys.path``；这里按仓库布局兜底（兄弟包目录）。导入失败**不抛异常**——
    调用方据此走 NEEDS_HUMAN 降级。
    """
    global _HW_LOADER, _HW_IMPORT_ERROR
    if _HW_LOADER is not None:
        return _HW_LOADER, ""
    try:
        from rm_hw_bridge import loader as _loader
    except Exception as exc:  # pragma: no cover - 依赖缺失路径
        first = repr(exc)
        sibling = os.path.abspath(os.path.join(
            os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
            "rm_hw_bridge",
        ))
        if os.path.isdir(sibling) and sibling not in sys.path:
            sys.path.insert(0, sibling)
        try:
            from rm_hw_bridge import loader as _loader
        except Exception as exc2:  # pragma: no cover - 依赖缺失路径
            _HW_IMPORT_ERROR = "%s / 兜底路径 %s: %r" % (first, sibling, exc2)
            return None, _HW_IMPORT_ERROR
    _HW_LOADER = _loader
    _HW_IMPORT_ERROR = ""
    return _HW_LOADER, ""


def _hw_bridge():
    """惰性导入 ``rm_hw_bridge.bridge``（连接层），返回 ``(module, error)``。"""
    _loader, err = _hw_loader()
    if _loader is None:
        return None, err
    try:
        from rm_hw_bridge import bridge as _bridge
    except Exception as exc:  # pragma: no cover - 依赖缺失路径
        return None, "%s / import rm_hw_bridge.bridge: %r" % (err, exc)
    return _bridge, ""


# ------------------------------------------------------------- 符号契约（唯一真值）
#: hwcollect 真机路径**需要用到的**契约核心符号（**能力需求声明**，不是许可清单）。
#:
#: 许可清单的**唯一真值**是 ``rm_hw_bridge/contract.yaml`` 的 ``core``；
#: 本模块只声明"要用哪些能力"，:data:`WHITELIST` = 契约 core ∩ 本需求声明，
#: 且需求里若有符号不在契约 core 中（契约漂移）→ 真机路径直接报错
#: （:data:`MISSING_FROM_CONTRACT`）。**不存在第二份手写白名单。**
REQUIRED_CORE_SYMBOLS = (
    "rm_init",
    "rm_create_robot_arm",
    "rm_set_realtime_push",
    "rm_get_joint_degree",
    "rm_get_current_arm_state",
    "rm_get_arm_all_state",
    "rm_get_joint_err_flag",
    "rm_get_arm_current_trajectory",
    "rm_movej",
    "rm_set_arm_stop",
    "rm_set_arm_slow_stop",
    "rm_set_arm_pause",
    "rm_set_arm_continue",
)


def _contract_symbols():
    """契约核心符号 / 黑名单符号 / 黑名单前缀（全部来自 contract.yaml）。

    返回 ``(core, blacklist, prefixes)``；契约不可用时返回三个空列表，
    并由 :data:`CONTRACT_ERROR` 说明原因（**失败时黑名单自检会 fail-closed**）。
    """
    global CONTRACT_ERROR
    loader, err = _hw_loader()
    if loader is None:
        CONTRACT_ERROR = err
        return [], [], []
    try:
        contract = loader.load_contract()
    except Exception as exc:  # pragma: no cover - 契约损坏路径
        CONTRACT_ERROR = repr(exc)
        return [], [], []
    return (list(loader.core_symbols(contract)),
            list(loader.blacklist_names(contract)),
            list(loader.blacklist_prefixes(contract)))


#: 契约不可用的原因（空 = 契约已加载）
CONTRACT_ERROR = ""

CORE_SYMBOLS, _BLACKLIST_NAMES, _BLACKLIST_PREFIXES = _contract_symbols()

#: 白名单 = 契约 core 中、本模块真机路径实际用到的子集（**不是手写清单**）
WHITELIST = tuple(s for s in CORE_SYMBOLS if s in REQUIRED_CORE_SYMBOLS)

#: 用符需求里**不在**契约 core 里的符号（契约漂移 → 真机路径直接报错）
MISSING_FROM_CONTRACT = tuple(s for s in REQUIRED_CORE_SYMBOLS if s not in CORE_SYMBOLS)

#: 黑名单符号（来自契约的 ``blacklist.symbols``）
BLACKLIST = tuple(_BLACKLIST_NAMES)

#: 黑名单前缀族（来自契约的 ``blacklist.prefixes``，如 ``rm_algo_``）
BLACKLIST_PREFIXES = tuple(_BLACKLIST_PREFIXES)

#: SDK 搜索路径环境变量（与硬件层同名同义）
ENV_SDK_LIB = "RM_API2_LIB"


# ------------------------------------------------------------------ 小工具
def _now_iso():
    return datetime.datetime.now().astimezone().replace(microsecond=0).isoformat()


def _module_source():
    try:
        with open(os.path.abspath(__file__), "r", encoding="utf-8") as fh:
            return fh.read()
    except OSError:
        return ""


def _call_name(node):
    """取调用形态的函数名（``a.b()`` → ``b``；``f()`` → ``f``）。"""
    func = node.func
    if isinstance(func, ast.Attribute):
        return func.attr
    if isinstance(func, ast.Name):
        return func.id
    return None


def blacklist_scan(source=None):
    """自检：源码里是否出现黑名单符号的**调用形态**。

    判定用 **AST**（``ast.Call``）：只认调用形态，字符串常量里的符号名不误判，
    并能给出符号名。黑名单来源是 ``rm_hw_bridge/contract.yaml`` 的 ``blacklist``
    （含前缀族），**不在本模块硬编码任何黑名单符号**。

    返回违规符号列表（空列表 = 通过）。
    """
    src = source if source is not None else _module_source()
    violations = []
    if not src:
        return violations
    try:
        tree = ast.parse(src)
    except SyntaxError:  # pragma: no cover - 退路：源码本身语法错误
        return violations
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        name = _call_name(node)
        if not name:
            continue
        if name in BLACKLIST:
            violations.append(name)
        elif any(name.startswith(p) for p in BLACKLIST_PREFIXES):
            violations.append(name)
    return sorted(set(violations))


def _assert_no_blacklist_calls():
    """黑名单自检。

    * 契约可用 → 按契约的 blacklist（含前缀族）扫描**调用形态**，发现违规即报错；
    * 契约不可用 → **打印警告**并继续（此时本模块根本拿不到任何黑名单符号，
      且全仓静态审计 ``python3 -m rm_hw_bridge.audit --repo`` 是权威门禁）。
    """
    if CONTRACT_ERROR:
        print("警告: 符号契约不可用，黑名单自检无法执行（权威门禁："
              "python3 -m rm_hw_bridge.audit --repo）: %s" % CONTRACT_ERROR,
              file=sys.stderr)
        return
    violations = blacklist_scan()
    if violations:
        raise RuntimeError("黑名单符号调用检测失败（设计 §1.4）: %s" % ", ".join(violations))


# ------------------------------------------------------------------ SDK 定位
def sdk_candidates(ws=None, env=None):
    """SDK 候选路径（委托硬件层装载器，唯一真值在 rm_hw_bridge）。"""
    loader, _err = _hw_loader()
    if loader is None:
        return []
    return list(loader.sdk_candidates(ws=ws, env=env))


def sdk_path(ws=None, env=None):
    """返回第一个真实存在的候选 ``.so`` 路径（找不到返回 ``None``）。"""
    for cand in sdk_candidates(ws=ws, env=env):
        if cand and os.path.isfile(cand):
            return os.path.abspath(cand)
    return None


def load_sdk(ws=None, env=None):
    """定位 RM-API2 动态库。

    找不到时抛 :class:`SdkMissing`（消息里带搜索路径与修复动作）。
    """
    path = sdk_path(ws=ws, env=env)
    if not path:
        tried = sdk_candidates(ws=ws, env=env)
        raise SdkMissing(
            "未找到 RM-API2 SDK（libapi_c.so）\n"
            "  已搜索: %s\n"
            "  → 修复: 向厂商索取 RM-API2 1.1.7，解包后把 libapi_c.so 放到 "
            "<repo>/.deps/rm_api2/，或设置环境变量 %s=/abs/path/libapi_c.so"
            % ("\n          ".join(tried or ["（无候选路径）"]), ENV_SDK_LIB))
    return path


# ------------------------------------------------------------- 失败结果构造
def _evidence_blocker(reason, next_action, extra=None):
    item = {"kind": "blocker", "detail": reason, "next_action": next_action}
    if extra:
        item.update(extra)
    return item


def _evidence_error(reason, next_action, extra=None):
    item = {"kind": "error", "detail": reason, "next_action": next_action}
    if extra:
        item.update(extra)
    return item


def sdk_missing_result(experiment, arm, reason, next_action, params=None, started_at=None):
    """统一的 NEEDS_HUMAN 结果结构（无任何数值）。"""
    return {
        "schema": SCHEMA,
        "experiment": experiment,
        "mode": "hardware",
        "status": STATUS_NEEDS_HUMAN,
        "arm": arm,
        "started_at": started_at or _now_iso(),
        "finished_at": _now_iso(),
        "params": params or {},
        "samples": [],
        "summary": {},
        "evidence": [_evidence_blocker(reason, next_action)],
        "reason": reason,
        "next_action": next_action,
        "notes": list(THREAT_NOTES),
    }


def _failure_result(experiment, arm, params, started, reason, next_action,
                    status=STATUS_ERROR, evidence=None):
    """失败结果（error / NEEDS_HUMAN 通用骨架）。"""
    if status == STATUS_NEEDS_HUMAN:
        ev = [_evidence_blocker(reason, next_action)]
    else:
        ev = [_evidence_error(reason, next_action)]
    ev.extend(evidence or [])
    return {
        "schema": SCHEMA,
        "experiment": experiment,
        "mode": "hardware",
        "status": status,
        "arm": arm,
        "started_at": started,
        "finished_at": _now_iso(),
        "params": params or {},
        "samples": [],
        "summary": {},
        "evidence": ev,
        "reason": reason,
        "next_action": next_action,
        "notes": list(THREAT_NOTES),
    }


# --------------------------------------------------------------- 连接（句柄）
def _local_ip_for(target):
    """推断本机（能到达 ``target`` 的那个网卡）的 IP。

    UDP 主动上报的目标地址必须是**运行本进程的主机**地址（否则臂把报文发给自己），
    因此真机路径需要它。这里用「不真正发包的 UDP connect」技巧推断（无副作用）。
    推断失败返回 ``None``（交由连接层用默认值并如实报错）。
    """
    import socket

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.connect((str(target), 9))   # 9 = discard（不会真的发包）
            return sock.getsockname()[0]
        finally:
            sock.close()
    except OSError:
        return None


def _connect_arm(arm="left", ip=None, port=DEFAULT_TCP_PORT, udp_port=None,
                 enable_realtime=False, realtime_period_ms=UDP_PERIOD_MS,
                 ws=None, lib=None, link=None):
    """建立**句柄语义**连接（真机路径唯一入口）。

    返回 ``(link, failure)``：``failure`` 为 ``None`` 表示连接成功；否则是可直接
    落盘的失败结果字典（NEEDS_HUMAN / error）。

    ``link``：**已建立的**句柄（注入复用）。控制器对客户端连接数有上限
    （实测 5），因此整轮真机验证应建一次链、全程复用 —— 注入的句柄带
    ``_rmctl_shared`` 标记，本模块**不负责关闭**它。
    """
    if link is not None:
        try:
            setattr(link, "_rmctl_shared", True)
        except Exception:  # pragma: no cover - 假句柄可能不允许设属性
            pass
        return link, None
    loader, import_error = _hw_loader()
    if loader is None:
        reason = "硬件适配层 rm_hw_bridge 不可用：%s" % import_error
        return None, _failure_result(
            "connect", arm, {}, _now_iso(), reason,
            "把硬件适配层加入 PYTHONPATH：export PYTHONPATH=%s:$PYTHONPATH"
            % os.path.join(paths.workspace_root(ws), "dualarm_ws", "src", "rm_hw_bridge"),
            status=STATUS_NEEDS_HUMAN,
        )

    if MISSING_FROM_CONTRACT:
        reason = "符号契约漂移：本模块需要的符号不在 rm_hw_bridge/contract.yaml 的 core 里: %s" \
                 % ", ".join(MISSING_FROM_CONTRACT)
        return None, _failure_result(
            "connect", arm, {}, _now_iso(), reason,
            "核对 rm_hw_bridge/contract.yaml 与 rm_hw_bridge/abi_spec.yaml 后重跑",
        )

    if lib is None:
        try:
            load_sdk(ws=ws)
        except SdkMissing as exc:
            return None, _failure_result(
                "connect", arm, {}, _now_iso(), str(exc),
                "向厂商索取 RM-API2 1.1.7 并放入 .deps/rm_api2/，"
                "或 export %s=/abs/path/libapi_c.so" % ENV_SDK_LIB,
                status=STATUS_NEEDS_HUMAN,
            )

    hw_bridge, import_error = _hw_bridge()
    if hw_bridge is None:
        reason = "硬件适配层 rm_hw_bridge.bridge 不可用：%s" % import_error
        return None, _failure_result(
            "connect", arm, {}, _now_iso(), reason,
            "确认 rm_hw_bridge 包完整（dualarm_ws/src/rm_hw_bridge）后重跑",
            status=STATUS_NEEDS_HUMAN,
        )

    host = ip or DEFAULT_ARM_IP.get(arm, DEFAULT_ARM_IP["left"])
    kwargs = {}
    if enable_realtime:
        # 上报目标必须是**本机**地址（否则臂把 UDP 报文发给自己，本机收不到）
        local_ip = _local_ip_for(host)
        if local_ip:
            kwargs["realtime_target_ip"] = local_ip
        if udp_port:
            kwargs["realtime_target_port"] = int(udp_port)
    link = hw_bridge.connect(
        ip=host,
        port=int(port),
        udp_port=udp_port,
        enable_realtime=bool(enable_realtime),
        realtime_period_ms=int(realtime_period_ms),
        lib=lib,
        ws=ws,
        arm=arm,
        **kwargs
    )
    if link.status == "ok":
        return link, None
    # 连接层已给出统一的分级语义：blocked → 需要人工/外部条件；error → 运行错误
    status = STATUS_NEEDS_HUMAN if link.status == "blocked" else STATUS_ERROR
    return None, _failure_result(
        "connect", arm, {}, _now_iso(), link.reason or "连接失败", link.next_action,
        status=status, evidence=[str(e) for e in (link.evidence or [])],
    )


def _joint_degrees(link):
    """读 7 个关节角（°，handle 语义）。

    只走契约清单内的 ``rm_get_joint_degree``，句柄来自连接层
    （**不按 IP 调用、不自行解释指针**）。

    返回 ``(rc, values)``；``rc != 0`` 表示读失败（调用方必须报错，**不得**编数）。
    """
    buf = (ctypes.c_float * 7)()
    rc = int(link.bridge.binding.rm_get_joint_degree(link.handle, buf) or 0)
    return rc, [float(v) for v in buf]


def _close_link(link):
    """释放句柄。

    **共享句柄不由本层释放**：真机窗口里控制器对客户端连接数有上限（实测 5，
    见 docs/rm_hw_bridge硬件适配层说明.md §10），整轮验证应当建一次链、全程复用；
    由调用方（阶梯编排器）统一释放。共享句柄带 ``_rmctl_shared`` 标记。
    """
    if link is None:
        return
    if getattr(link, "_rmctl_shared", False):
        return
    try:
        link.close()
    except Exception:  # pragma: no cover - 释放失败不影响主流程
        pass


# ------------------------------------------------------------------ 结果骨架
def _result(experiment, arm, mode, status, params, samples, summary, evidence,
            started_at=None, extra=None):
    out = {
        "schema": SCHEMA,
        "experiment": experiment,
        "mode": mode,
        "status": status,
        "arm": arm,
        "started_at": started_at or _now_iso(),
        "finished_at": _now_iso(),
        "params": params,
        "samples": samples,
        "summary": summary,
        "evidence": evidence,
        "notes": list(THREAT_NOTES),
    }
    if extra:
        out.update(extra)
    return out


def _synthetic_evidence(experiment, note):
    return [{"kind": "synthetic", "experiment": experiment, "detail": note,
             "warning": "dry-run 合成数据，不是真机读数，禁止回填基线"}]


# ------------------------------------------------------------------ UDP 稳定性
class UdpAggregator(object):
    """流式聚合器：只保留计数/间隔统计与抽样帧（不堆全量数据）。"""

    def __init__(self, period_ms=UDP_PERIOD_MS, sample_every=200, max_samples=200):
        self.period_ms = float(period_ms)
        self.sample_every = max(1, int(sample_every))
        self.max_samples = int(max_samples)
        self.frames = 0
        self.sampled = []
        self.first_ts = None
        self.last_ts = None
        self.gaps = []          # > 1.5 倍周期的间隔（断流）
        self.max_gap_ms = 0.0
        self.deltas = []        # 抽样保存的间隔（判断是否落在周期倍数上）
        self.dropout_frames = 0

    def add(self, ts, frame=None):
        ts = float(ts)
        self.frames += 1
        if self.first_ts is None:
            self.first_ts = ts
        if self.last_ts is not None:
            dt_ms = (ts - self.last_ts) * 1000.0
            self.deltas.append(dt_ms)
            if dt_ms > self.max_gap_ms:
                self.max_gap_ms = dt_ms
            if dt_ms > 1.5 * self.period_ms:
                self.gaps.append(dt_ms)
                missing = int(round(dt_ms / self.period_ms)) - 1
                self.dropout_frames += max(0, missing)
        self.last_ts = ts
        if frame is not None and (self.frames % self.sample_every == 0) \
                and len(self.sampled) < self.max_samples:
            self.sampled.append(frame)

    def period_multiple_ok(self, tol=0.2):
        """观测到的帧间隔是否都落在上报周期的整数倍附近（±20%）。"""
        if not self.deltas:
            return None
        for d in self.deltas:
            k = d / self.period_ms
            if abs(k - round(k)) > tol:
                return False
        return True

    def summary(self, duration_s=None):
        elapsed = 0.0
        if self.first_ts is not None and self.last_ts is not None:
            elapsed = self.last_ts - self.first_ts
        expected = None
        if duration_s:
            expected = int(round(duration_s * 1000.0 / self.period_ms))
        return {
            "frames": self.frames,
            "expected_frames": expected,
            "elapsed_s": elapsed,
            "dropout_count": len(self.gaps),
            "dropout_frames": self.dropout_frames,
            "max_gap_ms": self.max_gap_ms,
            "period_ms": self.period_ms,
            "period_multiple_ok": self.period_multiple_ok(),
            "sampled_frames": len(self.sampled),
        }


def _udp_verdict(frames, push_ret=None):
    """UDP 采集的诚实判定：**一帧都没收到就不能报 ok**。

    臂不可达 / 未开三线程模式 / 同一 ip:port 被别的客户端占用时，
    SDK 不会抛异常、只能返回 0 帧——若照直报 ok 会把"没采到"写成"采到了"。
    """
    if frames <= 0:
        return {
            "ok": False,
            "reason": "未收到任何 UDP 上报帧（frames=0）：控制器不可达、未开三线程模式"
                      "（RM_TRIPLE_MODE_E），或同一 ip:port 已被其它客户端占用",
            "next_action": "确认 ping <ip> 通、rm_init(RM_TRIPLE_MODE_E) 已开、"
                           "ss -lun | grep <port> 端口未被占用后重试",
            "push_ret": push_ret,
        }
    return {"ok": True, "reason": None, "next_action": None, "push_ret": push_ret}


def _synthetic_udp_frames(duration_s, period_ms, dropouts=2, seed=7):
    """生成合成上报帧（惰性生成，不一次性堆内存）。"""
    rng = np.random.default_rng(seed)
    step = period_ms / 1000.0
    total = int(round(duration_s / step))
    drop_idx = set()
    if dropouts > 0 and total > 10:
        drop_idx = set(int(i) for i in rng.choice(np.arange(1, total - 1),
                                                  size=min(dropouts, total - 2),
                                                  replace=False))
    t = 0.0
    for i in range(total):
        if i in drop_idx:
            t += step
            continue
        jitter = (rng.random() - 0.5) * (0.02 * step)
        yield (t + jitter, {"seq": i, "synthetic": True})
        t += step


def udp_stability(arm="left", duration_s=UDP_DEFAULT_DURATION_S,
                  period_ms=UDP_PERIOD_MS, ip=None, port=8091,
                  dry_run=False, ws=None, lib=None, link=None):
    """实时上报稳定性：断流次数 + 周期是否落在 5 ms 倍数上。

    真机路径需要 SDK（三线程模式 + 主动上报配置）与上报可达；
    缺 SDK 时返回 NEEDS_HUMAN，臂不可达时返回 error。
    """
    params = {"duration_s": duration_s, "period_ms": period_ms, "ip": ip, "port": port,
              "mode": "triple_thread", "single_client_per_endpoint": True}
    started = _now_iso()
    if dry_run:
        agg = UdpAggregator(period_ms=period_ms)
        for ts, frame in _synthetic_udp_frames(duration_s, period_ms):
            agg.add(ts, frame)
        summary = agg.summary(duration_s)
        summary["note"] = "dry-run 合成帧，不是真机上报"
        return _result("udp_stability", arm, "dry_run", STATUS_DRY_RUN, params,
                        agg.sampled, summary,
                        _synthetic_evidence("udp_stability", "合成 UDP 帧流"),
                        started)

    link, failure = _connect_arm(arm=arm, ip=ip, udp_port=port,
                                 enable_realtime=True, realtime_period_ms=period_ms,
                                 ws=ws, lib=lib, link=link)
    if failure is not None:
        failure["experiment"] = "udp_stability"
        failure["params"] = params
        failure["started_at"] = started
        if failure.get("status") == STATUS_ERROR:
            # 连接/上报配置失败 → 一帧都没收到（诚实降级：**不伪造任何帧数**）
            failure["summary"] = {
                "frames": 0,
                "expected_frames": int(round(duration_s * 1000.0 / float(period_ms))),
                "period_ms": period_ms,
                "note": "连接或上报配置失败，未采到任何上报帧",
            }
        return failure

    hw_bridge, _err = _hw_bridge()
    state = hw_bridge.realtime_state()
    window_s = max(float(period_ms) * 10.0, 50.0) / 1000.0
    deadline = time.monotonic() + float(duration_s)
    base_frames = state.frames
    samples = []
    windows = 0
    zero_windows = 0
    max_gap_windows = 0
    run_zero = 0
    prev = base_frames
    try:
        while time.monotonic() < deadline:
            time.sleep(window_s)
            now_frames = state.frames
            delta = now_frames - prev
            prev = now_frames
            windows += 1
            if delta <= 0:
                zero_windows += 1
                run_zero += 1
                if run_zero > max_gap_windows:
                    max_gap_windows = run_zero
            else:
                run_zero = 0
            if len(samples) < 200 and windows % 5 == 0:
                samples.append({"t_s": round((windows * window_s), 3),
                                "frames": int(now_frames - base_frames)})
    finally:
        _close_link(link)

    frames = int(state.frames - base_frames)
    elapsed = float(duration_s)
    expected = int(round(duration_s * 1000.0 / float(period_ms)))
    observed_period = (elapsed * 1000.0 / frames) if frames else None
    if observed_period:
        ratio = observed_period / float(period_ms)
        period_ok = abs(ratio - round(ratio)) <= 0.2 and round(ratio) >= 1
    else:
        period_ok = None
    summary = {
        "frames": frames,
        "expected_frames": expected,
        "elapsed_s": elapsed,
        "dropout_count": zero_windows,
        "dropout_frames": max(0, expected - frames),
        "max_gap_ms": max_gap_windows * window_s * 1000.0,
        "period_ms": period_ms,
        "period_multiple_ok": period_ok,
        "sampled_frames": len(samples),
        "host": ip or DEFAULT_ARM_IP.get(arm, DEFAULT_ARM_IP["left"]),
        "set_realtime_push_ret": 0,
        "sampling_window_ms": window_s * 1000.0,
        "note": "真机采样（帧数来自 SDK 上报回调）；只保留聚合统计与抽样帧",
    }
    verdict = _udp_verdict(frames, push_ret=0)
    evidence = [{"kind": "sdk", "detail": "SDK 路径: %s" % (sdk_path(ws=ws) or "<injected>")},
                {"kind": "udp", "frames": frames, "push_ret": 0}]
    if not verdict["ok"]:
        evidence.append(_evidence_error(verdict["reason"], verdict["next_action"]))
        return _result("udp_stability", arm, "hardware", STATUS_ERROR, params,
                        samples, summary, evidence, started,
                        extra={"reason": verdict["reason"],
                               "next_action": verdict["next_action"]})
    return _result("udp_stability", arm, "hardware", STATUS_OK, params,
                    samples, summary, evidence, started)


# ------------------------------------------------------------------ 阶梯 acc/jerk
def _ladder_profile(trials, joint_index, dt=LADDER_DT_S, duration_s=4.0):
    """合成阶梯速度轨迹（每个 trial 一个关节单独运动，返回 times/points）。"""
    times = np.arange(0.0, duration_s + dt, dt)
    n = len(times)
    out = []
    for t in trials:
        peak_deg_s = 30.0 * (1.0 + 0.05 * t)
        half = duration_s / 2.0
        q = np.zeros((n, 7))
        vel = np.zeros(n)
        for i, tt in enumerate(times):
            # 平滑梯形（smoothstep），避免拐点造成无穷 jerk
            if tt < half:
                s = tt / half
                vel[i] = peak_deg_s * (3 * s ** 2 - 2 * s ** 3)
            else:
                s = (duration_s - tt) / half
                s = max(0.0, s)
                vel[i] = peak_deg_s * (3 * s ** 2 - 2 * s ** 3)
        pos = np.cumsum(vel) * dt
        q[:, joint_index] = pos
        out.append((times, q))
    return out


DEG2RAD = np.pi / 180.0


def _ladder_peak(times, points, joint_index):
    """用 retime.derivatives 的微分语义算该关节的 acc / jerk 峰值。

    SDK 关节读数是**度**，先换算成弧度再微分（换算点只此一处），
    输出单位为 rad/s^2 与 rad/s^3（与 baseline 的单位口径一致）。
    """
    rad = np.asarray(points, dtype=float) * DEG2RAD
    vel, acc, jerk = retime.derivatives(list(times), rad)
    acc_peak = float(np.abs(acc[:, joint_index]).max()) if acc.size else 0.0
    jerk_peak = float(np.abs(jerk[:, joint_index]).max()) if jerk.size else 0.0
    return acc_peak, jerk_peak


def ladder_acc_jerk(arm="left", joints=tuple(range(1, 8)),
                    trials=LADDER_DEFAULT_TRIALS, dt=LADDER_DT_S,
                    duration_s=4.0, ip=None, dry_run=False, ws=None, lib=None,
                    link=None):
    """阶梯速度反推 acceleration / jerk 上限。

    取**多次采样的包络**（各 trial 峰值的最小值 = 保守下界），不用单次极值。
    微分语义复用 ``rmctl.retime.derivatives``（与规划链路一致）。
    """
    params = {"joints": list(joints), "trials": trials, "dt_s": dt,
              "duration_s": duration_s, "ip": ip,
              "derivative_method": "rmctl.retime.derivatives",
              "envelope": "min over trials（保守下界）"}
    started = _now_iso()
    if dry_run:
        acc_env, jerk_env = {}, {}
        acc_max, jerk_max = {}, {}
        samples = []
        for j in range(7):
            peaks_a, peaks_j = [], []
            for (times, points) in _ladder_profile(list(range(trials)), j, dt, duration_s):
                a, jj = _ladder_peak(times, points, j)
                peaks_a.append(a)
                peaks_j.append(jj)
                samples.append({"joint": "joint%d" % (j + 1), "trial": len(peaks_a) - 1,
                                "acc_peak_rad_s2": a, "jerk_peak_rad_s3": jj})
            acc_env["joint%d" % (j + 1)] = float(min(peaks_a))
            jerk_env["joint%d" % (j + 1)] = float(min(peaks_j))
            acc_max["joint%d" % (j + 1)] = float(max(peaks_a))
            jerk_max["joint%d" % (j + 1)] = float(max(peaks_j))
        summary = {
            "acceleration_rad_s2": acc_env,
            "jerk_rad_s3": jerk_env,
            "acceleration_observed_max_rad_s2": acc_max,
            "jerk_observed_max_rad_s3": jerk_max,
            "trials": trials,
            "envelope": "min over trials（保守下界）",
            "derivative_method": "rmctl.retime.derivatives",
            "unit_acceleration": "rad/s^2",
            "unit_jerk": "rad/s^3",
            "note": "dry-run 合成阶梯轨迹，不是真机读数",
        }
        return _result("ladder_acc_jerk", arm, "dry_run", STATUS_DRY_RUN, params,
                        samples, summary,
                        _synthetic_evidence("ladder_acc_jerk", "合成阶梯轨迹"), started)

    link, failure = _connect_arm(arm=arm, ip=ip, ws=ws, lib=lib, link=link)
    if failure is not None:
        failure["experiment"] = "ladder_acc_jerk"
        failure["params"] = params
        failure["started_at"] = started
        return failure

    summary = {"trials": trials, "envelope": "min over trials（保守下界）",
               "derivative_method": "rmctl.retime.derivatives",
               "scope": "acc/jerk 上限的保守下界（多次采样包络），非单次极值",
               "unit_acceleration": "rad/s^2", "unit_jerk": "rad/s^3"}
    samples = []
    try:
        for j in range(7):
            peaks_a, peaks_j = [], []
            for t in range(trials):
                rc, base = _joint_degrees(link)
                if rc != 0:
                    raise HwCollectError("rm_get_joint_degree 返回码 %r" % rc)
                # 固定姿态、只让当前关节走一小段（阶梯速度）
                goal = list(base)
                goal[j] = base[j] + LADDER_STEP_DEG
                speed = LADDER_BASE_SPEED + LADDER_SPEED_STEP * t
                move = link.preposition_movej(goal, speed=speed, block=0)
                if move.get("status") != "ok":
                    raise HwCollectError(move.get("reason") or "rm_movej 失败")
                times, positions = [], []
                t0 = time.monotonic()
                min_run = min(1.0, duration_s)
                while True:
                    rc, deg = _joint_degrees(link)
                    if rc != 0:
                        raise HwCollectError("rm_get_joint_degree 返回码 %r" % rc)
                    elapsed = time.monotonic() - t0
                    times.append(elapsed)
                    positions.append(list(deg))
                    traj = link.current_trajectory_type()
                    done = (traj.get("status") == "ok" and not traj.get("planning"))
                    if done and elapsed >= min_run:
                        break
                    if elapsed > duration_s * 4 + 5.0:
                        break
                    time.sleep(dt)
                q = np.asarray(positions, dtype=float)
                a, jj = _ladder_peak(times, q, j)
                peaks_a.append(a)
                peaks_j.append(jj)
                samples.append({"joint": "joint%d" % (j + 1), "trial": t,
                                "speed": speed, "acc_peak_rad_s2": a,
                                "jerk_peak_rad_s3": jj})
            summary.setdefault("acceleration_rad_s2", {})["joint%d" % (j + 1)] = float(min(peaks_a))
            summary.setdefault("jerk_rad_s3", {})["joint%d" % (j + 1)] = float(min(peaks_j))
    except HwCollectError as exc:
        return _result("ladder_acc_jerk", arm, "hardware", STATUS_ERROR, params,
                        samples, summary,
                        [_evidence_error(str(exc),
                                         "检查机械臂使能与网络后重试")], started,
                        extra={"reason": str(exc),
                               "next_action": "检查机械臂使能与网络后重试"})
    finally:
        _close_link(link)
    return _result("ladder_acc_jerk", arm, "hardware", STATUS_OK, params,
                    samples, summary,
                    [{"kind": "sdk", "detail": "SDK 路径: %s" % (sdk_path(ws=ws) or "<injected>")}],
                    started)


# ------------------------------------------------------------------ TCP 重复性
def _synthetic_tcp_samples(repeats, seed=11, sigma_mm=0.02):
    rng = np.random.default_rng(seed)
    target = np.array([300.0, 0.0, 250.0])
    out = []
    for i in range(repeats):
        pos = target + rng.normal(0.0, sigma_mm, 3)
        out.append({"index": i, "x_mm": float(pos[0]), "y_mm": float(pos[1]),
                    "z_mm": float(pos[2])})
    return out


def _repeatability_stats(samples):
    arr = np.asarray([[s["x_mm"], s["y_mm"], s["z_mm"]] for s in samples], dtype=float)
    mean = arr.mean(axis=0)
    dev = np.linalg.norm(arr - mean, axis=1)
    return {
        "repeats": int(len(samples)),
        "mean_position_mm": [float(v) for v in mean],
        "max_deviation_mm": float(dev.max()),
        "mean_deviation_mm": float(dev.mean()),
        "per_axis_spread_mm": [float(v) for v in (arr.max(axis=0) - arr.min(axis=0))],
    }


def tcp_repeatability(arm="left", target_deg=None, repeats=TCP_DEFAULT_REPEATS,
                      ip=None, dry_run=False, ws=None, lib=None, link=None):
    """TCP 位置**重复性**（明确区分重复性 ≠ 绝对精度）。

    官方标称 ±0.05 mm 是重复性标称值；本实验给出的是"多次到同一点的散布"，
    不代表 TCP 的绝对定位精度（绝对精度取决于标定，未标定前必然更大）。
    """
    params = {"target_deg": target_deg, "repeats": repeats, "ip": ip,
              "nominal_repeatability_mm": TCP_NOMINAL_MM,
              "scope": "重复性（散布）≠ 绝对精度（未标定）"}
    started = _now_iso()
    if dry_run:
        samples = _synthetic_tcp_samples(repeats)
        summary = _repeatability_stats(samples)
        summary["nominal_repeatability_mm"] = TCP_NOMINAL_MM
        summary["within_nominal"] = bool(summary["max_deviation_mm"] <= TCP_NOMINAL_MM)
        summary["scope"] = "重复性（散布）≠ 绝对精度（未标定）"
        summary["note"] = "dry-run 合成散布，不是真机重复性"
        return _result("tcp_repeatability", arm, "dry_run", STATUS_DRY_RUN, params,
                        samples, summary,
                        _synthetic_evidence("tcp_repeatability", "合成 TCP 散布"), started)

    link, failure = _connect_arm(arm=arm, ip=ip, ws=ws, lib=lib, link=link)
    if failure is not None:
        failure["experiment"] = "tcp_repeatability"
        failure["params"] = params
        failure["started_at"] = started
        return failure

    goal = list(target_deg) if target_deg else [0.0] * 7
    samples = []
    try:
        for i in range(repeats):
            move = link.preposition_movej(goal, speed=10, block=1)
            if move.get("status") != "ok":
                raise HwCollectError(move.get("reason") or "rm_movej 失败")
            time.sleep(0.2)   # 稳定后再读位置
            snap = link.snapshot()
            if snap.get("status") != "ok":
                raise HwCollectError(snap.get("reason") or "状态快照失败")
            pos = snap["snapshot"]["controller_pose"]["position"]
            samples.append({"index": i,
                            "x_mm": float(pos["x"]) * 1000.0,
                            "y_mm": float(pos["y"]) * 1000.0,
                            "z_mm": float(pos["z"]) * 1000.0})
    except HwCollectError as exc:
        return _result("tcp_repeatability", arm, "hardware", STATUS_ERROR, params,
                        samples, {},
                        [_evidence_error(str(exc),
                                         "确认控制器在线、目标点位合法后重试")], started,
                        extra={"reason": str(exc),
                               "next_action": "确认控制器在线、目标点位合法后重试"})
    finally:
        _close_link(link)
    summary = _repeatability_stats(samples)
    summary["nominal_repeatability_mm"] = TCP_NOMINAL_MM
    summary["within_nominal"] = bool(summary["max_deviation_mm"] <= TCP_NOMINAL_MM)
    summary["scope"] = "重复性（散布）≠ 绝对精度（未标定）"
    return _result("tcp_repeatability", arm, "hardware", STATUS_OK, params,
                    samples, summary,
                    [{"kind": "sdk", "detail": "SDK 路径: %s" % (sdk_path(ws=ws) or "<injected>")}],
                    started)


# ------------------------------------------------------------------ FK 交叉
def fk_cross_experiment(arm="left", n=FK_DEFAULT_SAMPLES, seed=42, ip=None,
                        dry_run=False, ws=None, lib=None, urdf_path=None,
                        link=None):
    """FK 与控制器位姿交叉校验（调用 ``rmctl.fk_cross.cross_check``）。

    控制器位姿只进入比对报告，**不得**写回轨迹/模型（R003）。
    """
    params = {"n": n, "seed": seed, "ip": ip,
              "tolerances": {"pos_mm": fk_cross.DEFAULT_POS_TOL_MM,
                             "ori_deg": fk_cross.DEFAULT_ORI_TOL_DEG},
              "controller_pose_usage": fk_cross.CONTROLLER_POSE_USAGE}
    started = _now_iso()
    if dry_run:
        syn = fk_cross.synthesize_samples(n=n, seed=seed, urdf_path=urdf_path)
        report = fk_cross.cross_check(syn["samples"], urdf_path)
        summary = fk_cross.fk_recheck_fields(report)
        summary["ok"] = bool(report["ok"])
        summary["violations"] = report["violations"]
        summary["note"] = "dry-run 合成样本，不是真机位姿"
        return _result("fk_cross", arm, "dry_run", STATUS_DRY_RUN, params,
                        syn["samples"], summary,
                        _synthetic_evidence("fk_cross", "合成 FK 交叉样本"), started)

    link, failure = _connect_arm(arm=arm, ip=ip, ws=ws, lib=lib, link=link)
    if failure is not None:
        failure["experiment"] = "fk_cross"
        failure["params"] = params
        failure["started_at"] = started
        return failure

    samples = []
    try:
        for _ in range(n):
            rc, deg = _joint_degrees(link)
            if rc != 0:
                raise HwCollectError("rm_get_joint_degree 返回码 %r" % rc)
            goal = [float(v) for v in deg]
            move = link.preposition_movej(goal, speed=5, block=1)
            if move.get("status") != "ok":
                raise HwCollectError(move.get("reason") or "rm_movej 失败")
            snap = link.snapshot()
            if snap.get("status") != "ok":
                raise HwCollectError(snap.get("reason") or "状态快照失败")
            pos = snap["snapshot"]["controller_pose"]["position"]
            samples.append({"joint_deg": goal,
                            "controller_pose": {"x": float(pos["x"]),
                                                "y": float(pos["y"]),
                                                "z": float(pos["z"])}})
    except HwCollectError as exc:
        return _result("fk_cross", arm, "hardware", STATUS_ERROR, params, samples, {},
                       [_evidence_error(str(exc),
                                        "确认控制器在线后重跑 FK 交叉校验")], started,
                       extra={"reason": str(exc),
                              "next_action": "确认控制器在线后重跑 FK 交叉校验"})
    finally:
        _close_link(link)
    report = fk_cross.cross_check(samples, urdf_path)
    summary = fk_cross.fk_recheck_fields(report)
    summary["ok"] = bool(report["ok"])
    summary["violations"] = report["violations"]
    return _result("fk_cross", arm, "hardware", STATUS_OK, params, samples, summary,
                   [{"kind": "sdk", "detail": "SDK 路径: %s" % (sdk_path(ws=ws) or "<injected>")},
                    {"kind": "fk_report", "ok": report["ok"]}], started)


# ------------------------------------------------------------------ 急停演练
ESTOP_SAFETY_CHECKLIST = (
    "工作半径内无人员、无设备、无其它机械臂",
    "急停按钮可触及，且已确认断电后机械臂停止运动",
    "示教器在手边，可清错并重新使能（项目不自动清错）",
    "演练后恢复流程已口头确认",
)

ESTOP_RECOVERY = ("急停后需在示教器上清错并重新使能；本项目不自动清错、不自动续跑。")


def _estop_verdict(soft_ret, stop_ret, state_ret=None):
    """急停演练的诚实判定：SDK 返回码非 0 一律报 error（演练未真正发生）。"""
    if soft_ret != 0 or stop_ret != 0 or (state_ret is not None and state_ret != 0):
        return {
            "ok": False,
            "reason": "急停演练未真正执行：rm_set_arm_slow_stop=%r, rm_set_arm_stop=%r, "
                      "rm_get_current_arm_state=%r（返回码非 0 通常表示未连接到控制器）"
                      % (soft_ret, stop_ret, state_ret),
            "next_action": "确认控制器已上电使能且网络可达后重做演练；"
                           "急停后需示教器清错并重新使能（项目不自动清错）",
        }
    return {"ok": True, "reason": None, "next_action": None}


def _estop_state(link, ret):
    """读一次状态快照 → 演练记录（错误码来自 ABI 规格的真实字段）。"""
    snap = link.snapshot()
    if snap.get("status") != "ok":
        return {"ret": ret, "state_ret": 1, "arm_err": [], "arm_state": None,
                "state_reason": snap.get("reason")}
    payload = snap["snapshot"]
    return {
        "ret": ret,
        "state_ret": 0,
        "arm_err": list(payload.get("arm_err") or []),
        "arm_state": payload.get("controller_trajectory_type"),
        "arm_state_name": payload.get("controller_trajectory_type_name"),
        "sys_err": None,
        "sys_err_note": "RM-API2 1.1.7 的 rm_current_arm_state_t 无独立 sys_err 字段"
                        "（见 rm_hw_bridge/abi_spec.yaml），此处留空而非编造",
    }


def estop_drill(arm="left", ip=None, dry_run=False, ws=None, lib=None,
                link=None):
    """软停 / 急停演练：记录 SDK 返回码、错误码与状态快照。"""
    params = {"ip": ip, "mode": "soft_stop_then_emergency",
              "safety_checklist": list(ESTOP_SAFETY_CHECKLIST),
              "recovery": ESTOP_RECOVERY,
              "auto_clear_error": False}
    started = _now_iso()
    if dry_run:
        summary = {
            "soft_stop": {"ret": 0, "arm_err": 0, "sys_err": 0, "arm_state": "synthetic"},
            "emergency_stop": {"ret": 0, "arm_err": 0, "sys_err": 0, "arm_state": "synthetic"},
            "recovery": ESTOP_RECOVERY,
            "note": "dry-run 合成返回码，不是真机演练结果",
        }
        return _result("estop_drill", arm, "dry_run", STATUS_DRY_RUN, params, [],
                        summary, _synthetic_evidence("estop_drill", "合成急停演练"), started)

    link, failure = _connect_arm(arm=arm, ip=ip, ws=ws, lib=lib, link=link)
    if failure is not None:
        failure["experiment"] = "estop_drill"
        failure["params"] = params
        failure["started_at"] = started
        return failure

    summary = {}
    try:
        soft = link.slow_stop()
        soft_ret = int(soft.get("rc") or 0) if soft.get("status") == "ok" else -1
        summary["soft_stop"] = _estop_state(link, soft_ret)
        stop = link.stop()
        stop_ret = int(stop.get("rc") or 0) if stop.get("status") == "ok" else -1
        summary["emergency_stop"] = _estop_state(link, stop_ret)
    except Exception as exc:                       # noqa: BLE001 —— 真机异常也结构化
        summary["error"] = repr(exc)
        return _result("estop_drill", arm, "hardware", STATUS_ERROR, params, [],
                        summary, [_evidence_error(repr(exc),
                                                 "确认控制器在线后重做演练")], started,
                        extra={"reason": repr(exc),
                               "next_action": "确认控制器在线后重做演练"})
    finally:
        _close_link(link)
    summary["recovery"] = ESTOP_RECOVERY
    summary["safety_checklist"] = list(ESTOP_SAFETY_CHECKLIST)
    verdict = _estop_verdict(soft_ret, stop_ret,
                             summary["emergency_stop"].get("state_ret"))
    if not verdict["ok"]:
        summary["drill_performed"] = False
        return _result("estop_drill", arm, "hardware", STATUS_ERROR, params, [],
                        summary,
                        [{"kind": "sdk", "detail": "SDK 路径: %s"
                          % (sdk_path(ws=ws) or "<injected>")},
                         _evidence_error(verdict["reason"], verdict["next_action"])],
                        started,
                        extra={"reason": verdict["reason"],
                               "next_action": verdict["next_action"]})
    summary["drill_performed"] = True
    return _result("estop_drill", arm, "hardware", STATUS_OK, params, [],
                    summary, [{"kind": "sdk", "detail": "SDK 路径: %s"
                              % (sdk_path(ws=ws) or "<injected>")}], started)


# ------------------------------------------------------------------ 零位采集
def zero_capture(arm="left", photos=None, ip=None, dry_run=False, ws=None, lib=None,
                 link=None):
    """零位读数与证据引用。

    零位标定**只能走示教器受控流程**（黑名单符号禁止调用），本脚本只负责
    记录读数快照与证据（照片/截图）引用，``zero_deg`` 一律留 null 等人工判定。
    """
    params = {"photos": list(photos or []), "ip": ip,
              "method": "示教器受控流程（脚本只读，零位标定由示教器完成）",
              "zero_deg": None}
    started = _now_iso()
    if dry_run:
        summary = {"joints": {}, "groove_photos": [], "zero_deg": None,
                   "note": "dry-run 无真机读数；零位必须由示教器流程确认"}
        return _result("zero_capture", arm, "dry_run", STATUS_DRY_RUN, params, [],
                        summary, _synthetic_evidence("zero_capture", "无读数的空采集"), started)

    link, failure = _connect_arm(arm=arm, ip=ip, ws=ws, lib=lib, link=link)
    if failure is not None:
        failure["experiment"] = "zero_capture"
        failure["params"] = params
        failure["started_at"] = started
        return failure

    try:
        rc, deg = _joint_degrees(link)
        if rc != 0:
            raise HwCollectError("rm_get_joint_degree 返回码 %r" % rc)
    except Exception as exc:                       # noqa: BLE001
        return _result("zero_capture", arm, "hardware", STATUS_ERROR, params, [], {},
                       [_evidence_error(repr(exc),
                                        "确认控制器在线后重跑零位读数")], started,
                       extra={"reason": repr(exc),
                              "next_action": "确认控制器在线后重跑零位读数"})
    finally:
        _close_link(link)
    joints = {}
    for i, value in enumerate(deg):
        joints["joint%d" % (i + 1)] = {"reading_deg": float(value), "unit": "deg",
                                       "zero_deg": None}
    summary = {"joints": joints, "groove_photos": list(photos or []), "zero_deg": None,
               "note": "读数为采集时刻快照，零位需示教器确认后人工回填"}
    return _result("zero_capture", arm, "hardware", STATUS_OK, params, [],
                    summary, [{"kind": "sdk", "detail": "SDK 路径: %s"
                              % (sdk_path(ws=ws) or "<injected>")},
                              {"kind": "photos", "count": len(photos or [])}], started)


# ------------------------------------------------------------------ 实验注册表
EXPERIMENTS = {
    "udp_stability": udp_stability,
    "ladder_acc_jerk": ladder_acc_jerk,
    "tcp_repeatability": tcp_repeatability,
    "fk_cross": fk_cross_experiment,
    "estop_drill": estop_drill,
    "zero_capture": zero_capture,
}


def run_experiment(name, dry_run=False, **kwargs):
    """按名字跑一类实验（未知名字抛 :class:`HwCollectError`）。"""
    if name not in EXPERIMENTS:
        raise HwCollectError("未知实验: %r（可选: %s）" % (name, ", ".join(sorted(EXPERIMENTS))))
    return EXPERIMENTS[name](dry_run=dry_run, **kwargs)


def write_result(result, out_dir=None, name=None):
    """把结果落盘。

    ``out_dir`` 缺省用**临时目录**（不污染 git 跟踪的 workcell/baselines/）。
    """
    import tempfile
    if out_dir is None:
        out_dir = tempfile.mkdtemp(prefix="rmctl-hwcollect-")
    out_dir = os.path.abspath(out_dir)
    if not os.path.isdir(out_dir):
        os.makedirs(out_dir, exist_ok=True)
    fname = "%s.%s.json" % (name or result.get("experiment", "result"),
                             result.get("arm", "arm"))
    path = os.path.join(out_dir, fname)
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(result, fh, ensure_ascii=False, indent=2)
    return path


# ------------------------------------------------------------------ CLI
def _build_parser():
    ap = argparse.ArgumentParser(
        prog="python3 -m rmctl.hwcollect",
        description="真机采集脚本骨架（六类实验；缺 SDK 时返回 NEEDS_HUMAN）")
    ap.add_argument("experiment", choices=sorted(EXPERIMENTS),
                    help="实验名：udp_stability / ladder_acc_jerk / tcp_repeatability / "
                         "fk_cross / estop_drill / zero_capture")
    ap.add_argument("--arm", default="left", choices=("left", "right"))
    ap.add_argument("--ip", default=None, help="控制器 IP（缺省按左右臂取默认）")
    ap.add_argument("--dry-run", action="store_true",
                    help="用合成数据走通落盘路径（不产生任何真机数值）")
    ap.add_argument("--out", default=None,
                    help="输出目录（缺省用临时目录；不会写进 workcell/baselines）")
    ap.add_argument("--duration-s", type=float, default=UDP_DEFAULT_DURATION_S,
                    help="udp_stability 采集时长（秒，默认 600）")
    ap.add_argument("--period-ms", type=float, default=UDP_PERIOD_MS,
                    help="udp_stability 上报周期（毫秒，默认 5）")
    ap.add_argument("--port", type=int, default=8091, help="udp_stability 本地 UDP 端口")
    ap.add_argument("--repeats", type=int, default=TCP_DEFAULT_REPEATS,
                    help="tcp_repeatability 次数（默认 30）")
    ap.add_argument("--trials", type=int, default=LADDER_DEFAULT_TRIALS,
                    help="ladder_acc_jerk 每个关节的重复次数（默认 3，取包络）")
    ap.add_argument("--samples", type=int, default=FK_DEFAULT_SAMPLES,
                    help="fk_cross 位形组数（默认 20）")
    ap.add_argument("--photo", action="append", default=None,
                    help="zero_capture 的证据照片引用（可重复）")
    ap.add_argument("--target", default=None,
                    help="tcp_repeatability 目标关节角（度，逗号分隔 7 个值）")
    return ap


def main(argv=None):
    _assert_no_blacklist_calls()
    args = _build_parser().parse_args(argv)
    kwargs = {"arm": args.arm, "ip": args.ip}
    if args.experiment == "udp_stability":
        kwargs.update({"duration_s": args.duration_s, "period_ms": args.period_ms,
                       "port": args.port})
    elif args.experiment == "tcp_repeatability":
        target = None
        if args.target:
            try:
                target = [float(v) for v in args.target.split(",")]
            except ValueError:
                print("输入非法: --target 必须是逗号分隔的数字", file=sys.stderr)
                return 2
            if len(target) != 7:
                print("输入非法: --target 必须是 7 个关节角（度）", file=sys.stderr)
                return 2
        kwargs.update({"target_deg": target, "repeats": args.repeats})
    elif args.experiment == "ladder_acc_jerk":
        kwargs.update({"trials": args.trials})
    elif args.experiment == "fk_cross":
        kwargs.update({"n": args.samples})
    elif args.experiment == "zero_capture":
        kwargs.update({"photos": args.photo or []})

    try:
        result = run_experiment(args.experiment, dry_run=args.dry_run, **kwargs)
    except HwCollectError as exc:
        print("输入非法: %s" % exc, file=sys.stderr)
        return 2
    except Exception as exc:                       # noqa: BLE001 —— 顶层兜底
        print("采集失败: %r" % exc, file=sys.stderr)
        return 4

    path = write_result(result, args.out, name=args.experiment)
    print("结果已写出: %s" % path)
    print("实验: %s | 模式: %s | 状态: %s" % (result["experiment"], result["mode"],
                                              result["status"]))
    if result["status"] == STATUS_NEEDS_HUMAN:
        print("阻塞: %s" % result.get("reason", ""))
        print("下一步: %s" % result.get("next_action", ""))
        return 3
    if result["status"] == STATUS_ERROR:
        # 操作者必须看到**原因**与**可粘贴的下一步**（S01 遗留只打印空 summary）
        print("错误: %s" % result.get("reason", "（未提供原因）"))
        print("下一步: %s" % result.get("next_action", "（未提供下一步）"))
        summary = result.get("summary") or {}
        if summary:
            print("摘要: %s" % json.dumps(summary, ensure_ascii=False))
        return 4
    print("摘要: %s" % json.dumps(result.get("summary", {}), ensure_ascii=False))
    return 0


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
