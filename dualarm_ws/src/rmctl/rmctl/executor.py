"""执行层（**legacy 原型**）。

.. warning::

   **本模块是 legacy 回放路径，不作验收证据。**

   * :class:`MockExecutor` —— 离线按 10 ms 栅格**回放**轨迹（不接硬件）。
     保留它是为了兼容既有脚本与 ``rmctl repl``；**任何验收结论都不得引用
     它的统计**（它没有限值复检、没有反馈闭环、没有真机下行）。
   * :class:`SdkExecutor` —— **真机路径已迁至** ``rm_executor``（进程角色
     ``rm_runtime``）。本类不再实现任何真机逻辑，只保留迁移指引，
     以免形成「两套真相」。

   验收口径唯一：``rm_executor.cli`` / ``rm_executor.runtime`` 产出的
   **统一报告**（离线 fake 与真机 live 同一 schema）。
"""
import time

import numpy as np


class ExecResult(object):
    def __init__(self, ok, reason, stats):
        self.ok = ok
        self.reason = reason
        self.stats = stats


class MockExecutor(object):
    """**【legacy】** 模拟执行器：不接硬件，按 10 ms 拍回放并实时更新机器人状态。

    .. warning::
       **legacy 路径，不作验收证据**（无四道校验、无限值复检、无反馈闭环）。
       验收请用 ``rmctl run --executor rm_executor``（离线）或
       ``rmctl run --live``（真机），报告 schema 见
       ``docs/rm_executor执行线程契约.md``。
    """

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
    """**【已迁出】** 真机执行器 —— 真机路径已迁至 ``rm_executor``。

    迁移说明（避免两套真相）：

    * 真机执行请用 ``python3 -m rm_executor.cli --traj <traj.json> --mode live
      --ip <ip> --port 8080 --report <report.json>``；
    * 真机窗口的角色入口是 ``python3 -m rm_executor.runtime --arm left
      --traj <traj.json> --report <report.json>``（进程角色 ``rm_runtime``）；
    * 本类**不再实现**任何真机逻辑（``connect`` / ``run`` 只给迁移指引），
      以免与 ``rm_executor`` 形成两套真机真相。

    原设计要点（设计 §5.7，已由 ``rm_executor`` 承担）：
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
            "真机执行路径已迁至 rm_executor：\n"
            "  python3 -m rm_executor.cli --traj <traj.json> --mode live "
            "--ip <ip> --port 8080 --report <report.json>\n"
            "  python3 -m rm_executor.runtime --arm left --traj <traj.json> "
            "--report <report.json>    # 真机窗口入口（进程角色 rm_runtime）")

    def run(self, times, points, on_tick=None, realtime=True):
        raise NotImplementedError(
            "真机执行路径已迁至 rm_executor（见 connect() 的迁移指引）")
