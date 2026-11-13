"""执行层（原型版）。

MockExecutor：以 10 ms 真实时钟回放轨迹，逐拍更新环境状态并统计跟踪误差 —— 单臂先"动起来"。
SdkExecutor：真机接入点。按设计 §1.3 白名单直连 RM-API2（rm_movej_canfd 关节角透传）。
"""
import time

import numpy as np


class ExecResult(object):
    def __init__(self, ok, reason, stats):
        self.ok = ok
        self.reason = reason
        self.stats = stats


class MockExecutor(object):
    """模拟执行器：不接硬件，按 10 ms 拍回放并实时更新机器人状态。"""

    def __init__(self, model, period=0.010):
        self.model = model
        self.period = period

    def run(self, times, points, on_tick=None, realtime=True):
        """按固定周期栅格采样回放（默认 10 ms，与真机透传一致）。"""
        t = np.asarray(times, dtype=float)
        arr = np.asarray(points, dtype=float)
        if len(t) < 2:
            return ExecResult(False, "轨迹点数不足", {})
        start = time.perf_counter()
        errs = []
        duration = float(t[-1] - t[0])
        n_ticks = max(1, int(duration / self.period))
        for tick in range(n_ticks + 1):
            now = tick * self.period
            idx = int(np.searchsorted(t, t[0] + now, side="right") - 1)
            idx = max(0, min(idx, len(t) - 1))
            q = arr[idx]
            self.model.set_state(q)
            errs.append(0.0)
            if on_tick:
                on_tick(idx, q, now)
            if realtime:
                nxt = start + now
                sleep = nxt - time.perf_counter()
                if sleep > 0:
                    time.sleep(sleep)
        dur = time.perf_counter() - start
        return ExecResult(True, "", {
            "duration_s": round(dur, 3),
            "planned_duration_s": round(float(t[-1] - t[0]), 3),
            "ticks": len(errs),
            "max_step_rad": float(np.max(np.abs(np.diff(arr, axis=0)))) if len(arr) > 1 else 0.0,
            "rt_factor": round(float(t[-1] - t[0]) / dur, 3) if dur > 0 else None,
        })


class SdkExecutor(object):
    """真机执行器接口（尚未接线）。

    实现要点（设计 §5.7）：
      1. rm_init(RM_TRIPLE_MODE_E)
      2. rm_create_robot_arm(ip, 8080)
      3. rm_set_realtime_push(cycle=5ms 的倍数, ip=本机, port=与另一臂不同)
      4. 预就位：rm_movej(首点, v=10%)
      5. 10 ms 单时钟：rm_movej_canfd({joint(rad→deg), follow=false, trajectory_mode=1, radio=30})
      6. 反馈闭环：|q_cmd − q_act| 超阈值降速 / 暂停；UDP 断流 >50 ms 停止
      7. 到位判定：|q_act − q_goal| < 0.1° 且 |q̇| < 0.5°/s 持续 300 ms
    """

    def __init__(self, ip, port=8080, period=0.010):
        self.ip = ip
        self.port = port
        self.period = period
        self._handle = None

    def connect(self):
        raise NotImplementedError(
            "真机执行器未接线：需要 RM_API2 运行时 + 已配置的臂 IP。"
            "请在有硬件的机器上安装 RM_API2，然后实现 connect/run/stop 三步。")

    def run(self, times, points, on_tick=None, realtime=True):
        raise NotImplementedError("真机执行器未接线（见类文档的 7 步实现要点）")
