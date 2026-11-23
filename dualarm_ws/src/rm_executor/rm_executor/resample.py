"""rm_executor 重采样：把任意轨迹重采样到**严格 10 ms 栅格**。

栅格语义（与 :mod:`rm_executor.clock` 的绝对截止时间栅格一致）：

* 起点 = 轨迹首采样时刻 ``t0``，步长**恒为** ``period_s``（不按采样间隔变步长）；
* 末端**保持到末点**：最后一格取 ``t_end``，其值取轨迹末点（**不外推**）；
* 单位**保持 rad 不变**——本模块不引入第二处单位换算（契约层已保证 units=rad）。

输入非法（点数 < 2、时间非严格递增、列数不齐、period 非正）时**抛错**，
由调用方（执行入口）转成 ``status="blocked"`` 的结构化结果，
本模块自身不吞异常、不返回伪造数据。
"""
from __future__ import annotations

from typing import Any, List, Sequence, Tuple

__all__ = [
    "grid_times",
    "resample_to_grid",
]

#: 默认栅格周期（秒）
DEFAULT_PERIOD_S = 0.010


def _as_float_list(values: Sequence[Any], name: str) -> List[float]:
    """转成 float 列表（逐项校验，非法即抛错）。"""
    out: List[float] = []
    for idx, item in enumerate(values):
        if isinstance(item, bool) or not isinstance(item, (int, float)):
            raise ValueError("%s[%d] 不是数值（%r）" % (name, idx, item))
        out.append(float(item))
    return out


def grid_times(t0: float, t_end: float, period_s: float = DEFAULT_PERIOD_S) -> List[float]:
    """生成栅格时间轴：从 ``t0`` 起、步长恒为 ``period_s``，**末格覆盖 ``t_end``**。

    * 若 ``t_end - t0`` 不是 ``period_s`` 的整数倍，最后一格被**截到 t_end**
      （保证末点一定被送到，不外推）。
    * ``t_end < t0`` → 抛错；``t_end == t0`` → 只返回 ``[t0]``。
    """
    t0 = float(t0)
    t_end = float(t_end)
    period = float(period_s)
    if not (period > 0.0):
        raise ValueError("period_s 必须为正数（当前 %r）" % (period_s,))
    if t_end < t0:
        raise ValueError("t_end 不可小于 t0（t0=%r, t_end=%r）" % (t0, t_end))
    times = [t0]
    if t_end == t0:
        return times
    k = 1
    while True:
        t = t0 + k * period
        if t >= t_end:
            break
        times.append(t)
        k += 1
    times.append(t_end)
    return times


def resample_to_grid(
    times: Sequence[float],
    points: Sequence[Sequence[float]],
    period_s: float = DEFAULT_PERIOD_S,
) -> Tuple[List[float], List[List[float]]]:
    """把轨迹线性插值重采样到严格 10 ms 栅格。

    参数
    ----
    times
        原始采样时刻（秒），**严格递增**，长度 ≥ 2。
    points
        对应的关节位置（**rad**，保持不变），每行维度一致（关节数一致）。
    period_s
        栅格周期（秒），默认 10 ms。

    返回
    ----
    (grid_t, grid_p)
        ``grid_t`` 栅格时间轴（首点 = times[0]，末点 = times[-1]）；
        ``grid_p`` 对应插值后的关节位置。

    异常
    ----
    ValueError
        点数 < 2、时间非严格递增、points 行数与 times 不符、关节维度不齐、
        period_s 非正——**由调用方转 blocked**。
    """
    ts = _as_float_list(times, "times")
    if len(ts) < 2:
        raise ValueError("轨迹点数不足 2（当前 %d），无法插值" % len(ts))
    for i in range(1, len(ts)):
        if ts[i] <= ts[i - 1]:
            raise ValueError(
                "times 必须严格递增（index=%d: %r -> %r）" % (i, ts[i - 1], ts[i])
            )
    if len(points) != len(ts):
        raise ValueError(
            "points 行数与 times 不一致（points=%d, times=%d）" % (len(points), len(ts))
        )
    n_joints = len(points[0])
    if n_joints == 0:
        raise ValueError("points 每行不得为空")
    rows: List[List[float]] = []
    for idx, row in enumerate(points):
        vals = _as_float_list(row, "points[%d]" % idx)
        if len(vals) != n_joints:
            raise ValueError(
                "points[%d] 维度不一致（%d != %d）" % (idx, len(vals), n_joints)
            )
        rows.append(vals)

    t0, t_end = ts[0], ts[-1]
    grid_t = grid_times(t0, t_end, period_s)
    grid_p = [_interpolate_at(ts, rows, t) for t in grid_t]
    return grid_t, grid_p


def _interpolate_at(
    ts: Sequence[float], rows: Sequence[Sequence[float]], t: float
) -> List[float]:
    """在时刻 ``t`` 处线性插值（``t`` 落在采样区间内；末端取末点值，不外推）。"""
    if t <= ts[0]:
        return list(rows[0])
    if t >= ts[-1]:
        return list(rows[-1])
    # 线性扫描定位区间（栅格单调递增，O(n) 足够；也可换 bisect）
    lo = 0
    hi = len(ts) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if ts[mid] <= t:
            lo = mid
        else:
            hi = mid
    t0, t1 = ts[lo], ts[hi]
    span = t1 - t0
    if span <= 0.0:
        return list(rows[lo])
    alpha = (t - t0) / span
    a, b = rows[lo], rows[hi]
    return [a[j] + alpha * (b[j] - a[j]) for j in range(len(a))]
