"""浏览器可视化（nanobind + viser，零编译、零 ROS 依赖）。

0.35.0.8 的 API 与 0.36 有差异：
  * 构造签名是 TesseractViewer(server_address=("127.0.0.1", 8000))
  * update_environment(env, origin_offset=[0,0,0], trajectory=None)
  * update_joint_positions(joint_names, joint_positions) —— 我们用它按 10ms 逐拍驱动手臂
"""
import time

from .vendor_env import ensure_libstdcxx

# 厂商 nanobind wheel 需要 GLIBCXX_3.4.31+：先预加载仓库自带的
# .deps/libstdcxx（只读引用），再 import 厂商库。
ensure_libstdcxx()


class BrowserViewer(object):
    def __init__(self, host="127.0.0.1", port=8000):
        self.host = host
        self.port = port
        self._viewer = None

    def __enter__(self):
        from tesseract_robotics.viewer import TesseractViewer
        self._viewer = TesseractViewer(server_address=(self.host, self.port))
        return self._viewer

    def __exit__(self, *a):
        try:
            if self._viewer:
                self._viewer.close()
        except Exception:
            pass
        return False

    @property
    def url(self):
        return "http://%s:%d" % (self.host, self.port)


def serve_background(model, host="127.0.0.1", port=8000):
    """起浏览器可视化（后台），返回 viewer 对象。"""
    from tesseract_robotics.viewer import TesseractViewer
    v = TesseractViewer(server_address=(host, port))
    v.update_environment(model.env)
    v.start_serve_background()
    return v


def animate(viewer, model, times, points, period=0.010, realtime=True):
    """按 10 ms 栅格驱动浏览器里的手臂（与执行器同一条轨迹）。"""
    import numpy as np
    t = np.asarray(times, dtype=float)
    q = np.asarray(points, dtype=float)
    if len(t) < 2:
        return
    start = time.perf_counter()
    n_ticks = int((t[-1] - t[0]) / period)
    shown = []
    for tick in range(n_ticks + 1):
        now = tick * period
        idx = max(0, min(int(np.searchsorted(t, t[0] + now, side="right") - 1), len(t) - 1))
        # 0.35 的 update_joint_positions 接受"单个位形"(float64 扁平数组)，逐帧刷新即成动画
        viewer.update_joint_positions(model.joint_names, np.asarray(q[idx], dtype=np.float64))
        if realtime:
            sleep = start + now - time.perf_counter()
            if sleep > 0:
                time.sleep(sleep)
