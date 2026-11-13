#!/usr/bin/env python3
"""真机采集脚本骨架：最小只读 SDK 路径 + 六类实验 + ``--dry-run``（设计 §5.1/§5.2/§5.3）。

真机是**预约资源**（session.md：机械臂"等到需要时我再提供"）。窗口内每多一分钟
交互式调试就多一分钟撞机风险，因此本模块把"真机窗口要执行的动作"全部脚本化，
并在没有 SDK 的机器上离线自测：**窗口只跑脚本、只落数据**。

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
* **黑名单符号**：``rm_set_joint_zero_pos``（零位标定）禁止调用——零位只能走
  示教器受控流程（设计 §1.4）。模块启动时做**自检**（``blacklist_scan()``），
  一旦源码里出现黑名单符号的调用形态就抛错。
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
import ctypes
import datetime
import json
import os
import re
import socket
import sys
import threading
import time

import numpy as np

from . import fk_cross
from . import paths
from . import retime

# ------------------------------------------------------------------ 常量
SCHEMA = "rmctl.hwcollect/1"

ENV_SDK_LIB = "RM_API2_LIB"
DEPS_REL = os.path.join(".deps", "rm_api2", "libapi_c.so")

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
#: 阶梯实验真机路径：基础速度与每档增速（RM-API2 rm_movej 的 speed 单位）
LADDER_BASE_SPEED = 5
LADDER_SPEED_STEP = 3
#: FK 交叉校验默认位形数
FK_DEFAULT_SAMPLES = 20

#: SDK 白名单（最小只读集 + 预就位移动 + 停机族）
WHITELIST = (
    "rm_init",
    "rm_create_robot_arm",
    "rm_set_realtime_push",
    "rm_get_joint_degree",
    "rm_get_current_arm_state",
    "rm_get_arm_all_state",
    "rm_movej",
    "rm_set_arm_stop",
    "rm_set_arm_slow_stop",
    "rm_set_arm_pause",
    "rm_set_arm_continue",
)

#: 黑名单符号：**禁止调用**。零位标定只能走示教器受控流程（设计 §1.4）。
BLACKLIST = ("rm_set_joint_zero_pos",)

#: RM-API2 三线程模式（不开三线程收不到 UDP 实时上报）
RM_TRIPLE_MODE_E = 0x00000001

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
)


class HwCollectError(Exception):
    """输入非法（消息面向操作者，可直接粘贴）。"""


class SdkMissing(HwCollectError):
    """RM-API2 SDK 缺失（附可粘贴的修复动作）。"""


# ------------------------------------------------------------------ 小工具
def _now_iso():
    return datetime.datetime.now().astimezone().replace(microsecond=0).isoformat()


def _module_source():
    try:
        with open(os.path.abspath(__file__), "r", encoding="utf-8") as fh:
            return fh.read()
    except OSError:
        return ""


def blacklist_scan(source=None):
    """自检：源码里是否出现黑名单符号的**调用形态**。

    返回违规符号列表（空列表 = 通过）。扫描的是"符号 + 左括号"的调用形态，
    因此黑名单常量本身（字符串字面量）不会误报。
    """
    src = source if source is not None else _module_source()
    violations = []
    for name in BLACKLIST:
        pat = re.compile(r"%s\s*\(" % re.escape(name))
        if pat.search(src):
            violations.append(name)
    return violations


def _assert_no_blacklist_calls():
    violations = blacklist_scan()
    if violations:
        raise RuntimeError("黑名单符号调用检测失败（设计 §1.4）: %s" % ", ".join(violations))


# ------------------------------------------------------------------ SDK 装载
def sdk_candidates(ws=None, env=None):
    """SDK 候选路径（按优先级）：环境变量 → ``.deps/rm_api2/libapi_c.so`` → 目录内任意 .so。"""
    env = os.environ if env is None else env
    out = []
    var = env.get(ENV_SDK_LIB)
    if var:
        out.append(var)
    root = ws or paths.workspace_root()
    out.append(os.path.join(root, DEPS_REL))
    dep_dir = os.path.join(root, ".deps", "rm_api2")
    if os.path.isdir(dep_dir):
        for name in sorted(os.listdir(dep_dir)):
            if ".so" in name:
                out.append(os.path.join(dep_dir, name))
    return out


def load_sdk(ws=None, env=None):
    """装载 RM-API2 动态库。

    找不到时抛 :class:`SdkMissing`（消息里带搜索路径与修复动作）。
    """
    candidates = sdk_candidates(ws=ws, env=env)
    tried = []
    for path in candidates:
        tried.append(path)
        if not path or not os.path.exists(path):
            continue
        try:
            lib = ctypes.CDLL(path)
        except OSError as exc:
            raise SdkMissing("加载 RM-API2 失败: %s (%s)" % (path, exc))
        return lib, path
    raise SdkMissing(
        "未找到 RM-API2 SDK（libapi_c.so）\n"
        "  已搜索: %s\n"
        "  → 修复: 向厂商索取 RM-API2 1.1.7，解包后把 libapi_c.so 放到 "
        "<repo>/.deps/rm_api2/，或设置环境变量 %s=/abs/path/libapi_c.so"
        % ("\n          ".join(tried or ["（无候选路径）"]), ENV_SDK_LIB))


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
        "evidence": [{"kind": "blocker", "detail": reason, "next_action": next_action}],
        "reason": reason,
        "next_action": next_action,
        "notes": list(THREAT_NOTES),
    }


# ------------------------------------------------------------------ SDK 封装
class _RmArmState(ctypes.Structure):
    """RM-API2 ``rm_arm_current_state_t`` 的**尽力而为**布局。

    字段顺序按 RM-API2 1.1.7 头文件整理；真机窗口首次运行若布局不符，
    以厂商头文件为准修正本结构——**不要**为了跑通而编造读数。
    """

    _fields_ = [
        ("arm_state", ctypes.c_uint16),
        ("joint", ctypes.c_float * 7),
        ("joint_2", ctypes.c_float * 7),
        ("tcp_pose", ctypes.c_float * 6),
        ("arm_err", ctypes.c_int32),
        ("arm_err_code", ctypes.c_int32),
        ("sys_err", ctypes.c_int32),
        ("sys_err_code", ctypes.c_int32),
    ]


class RmSdk(object):
    """RM-API2 的最小只读封装（只暴露白名单符号）。"""

    def __init__(self, lib, path):
        self.lib = lib
        self.path = path
        self.missing_symbols = [s for s in WHITELIST if not hasattr(lib, s)]

    # -- 生命周期
    def init(self):
        return int(self.lib.rm_init(RM_TRIPLE_MODE_E))

    def create_robot_arm(self, ip, port=8080):
        return int(self.lib.rm_create_robot_arm(ip.encode("utf-8"), int(port)))

    def set_realtime_push(self, ip, port=8080, push=True):
        return int(self.lib.rm_set_realtime_push(ip.encode("utf-8"), int(port), bool(push)))

    # -- 只读
    def get_joint_degree(self, ip):
        buf = (ctypes.c_float * 7)()
        ret = int(self.lib.rm_get_joint_degree(ip.encode("utf-8"), buf))
        return ret, [float(v) for v in buf]

    def get_current_arm_state(self, ip):
        state = _RmArmState()
        ret = int(self.lib.rm_get_current_arm_state(ip.encode("utf-8"), ctypes.byref(state)))
        return ret, state

    def get_arm_all_state(self, ip):
        state = _RmArmState()
        ret = int(self.lib.rm_get_arm_all_state(ip.encode("utf-8"), ctypes.byref(state)))
        return ret, state

    # -- 运动（预就位 / 停机族）
    def movej(self, ip, joints_deg, speed=10):
        buf = (ctypes.c_float * 7)(*[float(v) for v in joints_deg])
        return int(self.lib.rm_movej(ip.encode("utf-8"), buf, int(speed)))

    def set_arm_stop(self, ip):
        return int(self.lib.rm_set_arm_stop(ip.encode("utf-8")))

    def set_arm_slow_stop(self, ip):
        return int(self.lib.rm_set_arm_slow_stop(ip.encode("utf-8")))

    def set_arm_pause(self, ip):
        return int(self.lib.rm_set_arm_pause(ip.encode("utf-8")))

    def set_arm_continue(self, ip):
        return int(self.lib.rm_set_arm_continue(ip.encode("utf-8")))


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
                  dry_run=False, ws=None, sdk=None):
    """实时上报稳定性：断流次数 + 周期是否落在 5 ms 倍数上。

    真机路径需要 SDK（``rm_set_realtime_push`` + 三线程模式）与 UDP 端口可达；
    缺 SDK 时返回 NEEDS_HUMAN。
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

    try:
        lib, path = load_sdk(ws=ws)
    except SdkMissing as exc:
        return sdk_missing_result("udp_stability", arm, str(exc),
                                   "向厂商索取 RM-API2 1.1.7 并放入 .deps/rm_api2/",
                                   params, started)
    sdkw = RmSdk(lib, path)
    host = ip or ("192.168.1.18" if arm == "left" else "192.168.1.19")
    agg = UdpAggregator(period_ms=period_ms)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    push_ret = None
    push_error = None
    try:
        sock.bind(("0.0.0.0", int(port)))
        sock.settimeout(1.0)
        try:
            push_ret = sdkw.set_realtime_push(host, port, True)
        except Exception as exc:                       # noqa: BLE001 —— 记证据不崩
            push_error = repr(exc)
        deadline = time.monotonic() + float(duration_s)
        while time.monotonic() < deadline:
            try:
                data, _addr = sock.recvfrom(4096)
            except socket.timeout:
                continue
            agg.add(time.monotonic(), {"bytes": len(data)})
    finally:
        sock.close()
    summary = agg.summary(duration_s)
    summary["host"] = host
    summary["set_realtime_push_ret"] = push_ret
    summary["note"] = "真机采样；只保留聚合统计与抽样帧"
    verdict = _udp_verdict(agg.frames, push_ret)
    evidence = [{"kind": "sdk", "path": path, "missing_symbols": sdkw.missing_symbols},
                {"kind": "udp", "frames": agg.frames, "push_ret": push_ret}]
    if push_error:
        evidence.append({"kind": "error", "detail": "set_realtime_push 异常: %s" % push_error})
    if not verdict["ok"]:
        evidence.append({"kind": "error", "detail": verdict["reason"],
                         "next_action": verdict["next_action"]})
        return _result("udp_stability", arm, "hardware", STATUS_ERROR, params,
                        agg.sampled, summary, evidence, started,
                        extra={"reason": verdict["reason"],
                               "next_action": verdict["next_action"]})
    return _result("udp_stability", arm, "hardware", STATUS_OK, params,
                    agg.sampled, summary, evidence, started)


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
                    duration_s=4.0, ip=None, dry_run=False, ws=None, sdk=None):
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

    try:
        lib, path = load_sdk(ws=ws)
    except SdkMissing as exc:
        return sdk_missing_result("ladder_acc_jerk", arm, str(exc),
                                   "向厂商索取 RM-API2 1.1.7 并放入 .deps/rm_api2/",
                                   params, started)
    sdkw = RmSdk(lib, path)
    host = ip or ("192.168.1.18" if arm == "left" else "192.168.1.19")
    summary = {"trials": trials, "envelope": "min over trials（保守下界）",
               "derivative_method": "rmctl.retime.derivatives",
               "scope": "acc/jerk 上限的保守下界（多次采样包络），非单次极值",
               "unit_acceleration": "rad/s^2", "unit_jerk": "rad/s^3"}
    samples = []
    try:
        for j in range(7):
            peaks_a, peaks_j = [], []
            for t in range(trials):
                ret, base = sdkw.get_joint_degree(host)
                if ret != 0:
                    raise HwCollectError("rm_get_joint_degree 返回码 %r（IP %s）" % (ret, host))
                # 固定姿态、只让当前关节走一小段（阶梯速度）
                goal = list(base)
                goal[j] = base[j] + LADDER_STEP_DEG
                speed = LADDER_BASE_SPEED + LADDER_SPEED_STEP * t
                times, positions = [], []
                move_result = {}

                def _move():
                    move_result["ret"] = sdkw.movej(host, goal, speed)

                worker = threading.Thread(target=_move, name="rm-movej")
                worker.daemon = True
                worker.start()
                t0 = time.monotonic()
                while worker.is_alive() or not times:
                    ret, deg = sdkw.get_joint_degree(host)
                    if ret != 0:
                        raise HwCollectError("rm_get_joint_degree 返回码 %r" % ret)
                    times.append(time.monotonic() - t0)
                    positions.append(list(deg))
                    time.sleep(dt)
                    if time.monotonic() - t0 > duration_s * 4 + 5.0:
                        break
                worker.join(timeout=1.0)
                if move_result.get("ret") not in (0, None):
                    raise HwCollectError("rm_movej 返回码 %r" % move_result["ret"])
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
                        [{"kind": "error", "detail": str(exc),
                          "next_action": "检查机械臂使能与网络后重试"}], started)
    return _result("ladder_acc_jerk", arm, "hardware", STATUS_OK, params,
                    samples, summary,
                    [{"kind": "sdk", "path": path}], started)


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
                      ip=None, dry_run=False, ws=None, sdk=None):
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

    try:
        lib, path = load_sdk(ws=ws)
    except SdkMissing as exc:
        return sdk_missing_result("tcp_repeatability", arm, str(exc),
                                   "向厂商索取 RM-API2 1.1.7 并放入 .deps/rm_api2/",
                                   params, started)
    sdkw = RmSdk(lib, path)
    host = ip or ("192.168.1.18" if arm == "left" else "192.168.1.19")
    goal = list(target_deg) if target_deg else [0.0] * 7
    samples = []
    try:
        for i in range(repeats):
            ret = sdkw.movej(host, goal)
            if ret != 0:
                raise HwCollectError("rm_movej 返回码 %r" % ret)
            time.sleep(1.0)   # 等待到位（保守等待，真机窗口按需调整）
            ret, state = sdkw.get_current_arm_state(host)
            if ret != 0:
                raise HwCollectError("rm_get_current_arm_state 返回码 %r" % ret)
            pose = [float(v) for v in state.tcp_pose]
            samples.append({"index": i, "x_mm": pose[0], "y_mm": pose[1], "z_mm": pose[2]})
    except HwCollectError as exc:
        return _result("tcp_repeatability", arm, "hardware", STATUS_ERROR, params,
                        samples, {}, [{"kind": "error", "detail": str(exc)}], started)
    summary = _repeatability_stats(samples)
    summary["nominal_repeatability_mm"] = TCP_NOMINAL_MM
    summary["within_nominal"] = bool(summary["max_deviation_mm"] <= TCP_NOMINAL_MM)
    summary["scope"] = "重复性（散布）≠ 绝对精度（未标定）"
    return _result("tcp_repeatability", arm, "hardware", STATUS_OK, params,
                    samples, summary, [{"kind": "sdk", "path": path}], started)


# ------------------------------------------------------------------ FK 交叉
def fk_cross_experiment(arm="left", n=FK_DEFAULT_SAMPLES, seed=42, ip=None,
                        dry_run=False, ws=None, sdk=None, urdf_path=None):
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

    try:
        lib, path = load_sdk(ws=ws)
    except SdkMissing as exc:
        return sdk_missing_result("fk_cross", arm, str(exc),
                                   "向厂商索取 RM-API2 1.1.7 并放入 .deps/rm_api2/",
                                   params, started)
    sdkw = RmSdk(lib, path)
    host = ip or ("192.168.1.18" if arm == "left" else "192.168.1.19")
    samples = []
    try:
        for _ in range(n):
            ret, deg = sdkw.get_joint_degree(host)
            if ret != 0:
                raise HwCollectError("rm_get_joint_degree 返回码 %r" % ret)
            goal = [float(v) for v in deg]
            ret = sdkw.movej(host, goal, speed=5)
            if ret != 0:
                raise HwCollectError("rm_movej 返回码 %r" % ret)
            time.sleep(1.0)
            ret, state = sdkw.get_current_arm_state(host)
            if ret != 0:
                raise HwCollectError("rm_get_current_arm_state 返回码 %r" % ret)
            pose = [float(v) for v in state.tcp_pose]
            samples.append({"joint_deg": goal,
                            "controller_pose": {"x": pose[0] / 1000.0, "y": pose[1] / 1000.0,
                                                "z": pose[2] / 1000.0}})
    except HwCollectError as exc:
        return _result("fk_cross", arm, "hardware", STATUS_ERROR, params, samples, {},
                       [{"kind": "error", "detail": str(exc)}], started)
    report = fk_cross.cross_check(samples, urdf_path)
    summary = fk_cross.fk_recheck_fields(report)
    summary["ok"] = bool(report["ok"])
    summary["violations"] = report["violations"]
    return _result("fk_cross", arm, "hardware", STATUS_OK, params, samples, summary,
                   [{"kind": "sdk", "path": path},
                    {"kind": "fk_report", "ok": report["ok"]}], started)


# ------------------------------------------------------------------ 急停演练
ESTOP_SAFETY_CHECKLIST = (
    "工作半径内无人员、无设备、无其它机械臂",
    "急停按钮可触及，且已确认断电后机械臂停止运动",
    "示教器在手边，可清错并重新使能（项目不自动清错）",
    "演练后恢复流程已口头确认",
)

ESTOP_RECOVERY = ("急停后需在示教器上清错并重新使能；本项目不自动清错、不自动续跑。")


def estop_drill(arm="left", ip=None, dry_run=False, ws=None, sdk=None):
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

    try:
        lib, path = load_sdk(ws=ws)
    except SdkMissing as exc:
        return sdk_missing_result("estop_drill", arm, str(exc),
                                   "向厂商索取 RM-API2 1.1.7 并放入 .deps/rm_api2/",
                                   params, started)
    sdkw = RmSdk(lib, path)
    host = ip or ("192.168.1.18" if arm == "left" else "192.168.1.19")
    summary = {}
    try:
        ret_slow = sdkw.set_arm_slow_stop(host)
        ret_state, state = sdkw.get_current_arm_state(host)
        summary["soft_stop"] = {"ret": ret_slow, "state_ret": ret_state,
                                "arm_err": int(state.arm_err),
                                "sys_err": int(state.sys_err),
                                "arm_state": int(state.arm_state)}
        ret_stop = sdkw.set_arm_stop(host)
        ret_state2, state2 = sdkw.get_current_arm_state(host)
        summary["emergency_stop"] = {"ret": ret_stop, "state_ret": ret_state2,
                                     "arm_err": int(state2.arm_err),
                                     "sys_err": int(state2.sys_err),
                                     "arm_state": int(state2.arm_state)}
    except Exception as exc:                       # noqa: BLE001 —— 真机异常也结构化
        summary["error"] = repr(exc)
        return _result("estop_drill", arm, "hardware", STATUS_ERROR, params, [],
                        summary, [{"kind": "error", "detail": repr(exc)}], started)
    summary["recovery"] = ESTOP_RECOVERY
    summary["safety_checklist"] = list(ESTOP_SAFETY_CHECKLIST)
    verdict = _estop_verdict(ret_slow, ret_stop, ret_state2)
    if not verdict["ok"]:
        summary["drill_performed"] = False
        return _result("estop_drill", arm, "hardware", STATUS_ERROR, params, [],
                        summary,
                        [{"kind": "sdk", "path": path},
                         {"kind": "error", "detail": verdict["reason"],
                          "next_action": verdict["next_action"]}],
                        started,
                        extra={"reason": verdict["reason"],
                               "next_action": verdict["next_action"]})
    summary["drill_performed"] = True
    return _result("estop_drill", arm, "hardware", STATUS_OK, params, [],
                    summary, [{"kind": "sdk", "path": path}], started)


# ------------------------------------------------------------------ 零位采集
def zero_capture(arm="left", photos=None, ip=None, dry_run=False, ws=None, sdk=None):
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

    try:
        lib, path = load_sdk(ws=ws)
    except SdkMissing as exc:
        return sdk_missing_result("zero_capture", arm, str(exc),
                                   "向厂商索取 RM-API2 1.1.7 并放入 .deps/rm_api2/",
                                   params, started)
    sdkw = RmSdk(lib, path)
    host = ip or ("192.168.1.18" if arm == "left" else "192.168.1.19")
    try:
        ret, deg = sdkw.get_joint_degree(host)
        if ret != 0:
            raise HwCollectError("rm_get_joint_degree 返回码 %r" % ret)
    except Exception as exc:                       # noqa: BLE001
        return _result("zero_capture", arm, "hardware", STATUS_ERROR, params, [], {},
                       [{"kind": "error", "detail": repr(exc)}], started)
    joints = {}
    for i, value in enumerate(deg):
        joints["joint%d" % (i + 1)] = {"reading_deg": float(value), "unit": "deg",
                                       "zero_deg": None}
    summary = {"joints": joints, "groove_photos": list(photos or []), "zero_deg": None,
               "note": "读数为采集时刻快照，零位需示教器确认后人工回填"}
    return _result("zero_capture", arm, "hardware", STATUS_OK, params, [],
                    summary, [{"kind": "sdk", "path": path},
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
        print("错误: %s" % json.dumps(result.get("summary", {}), ensure_ascii=False))
        return 4
    print("摘要: %s" % json.dumps(result.get("summary", {}), ensure_ascii=False))
    return 0


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
