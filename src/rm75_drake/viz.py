"""可视化抽象层（S02-T08）。

职责边界
--------
本模块只提供**可视化后端的抽象与两种实现**：

* :class:`Visualizer` —— 协议（``show_trajectory`` / ``show_scene`` / ``save``）；
* :class:`MeshcatBackend` —— 浏览器三维可视化（**探测式实现**）；
* :class:`FallbackBackend` —— matplotlib **2D 投影**兜底。

**Meshcat 可用性（本机实测，必须显式记录，不得美化）**
--------------------------------------------------------
本机（Drake 1.51.1 wheel / Python 3.10 / 离线环境）**没有 meshcat 服务端**：

1. ``import meshcat`` → ``ModuleNotFoundError: No module named 'meshcat'``；
2. ``from pydrake import meshcat`` → ``ImportError: cannot import name 'meshcat'``
   ——Drake wheel 内**只有 meshcat.js 前端资源，没有 Python 服务端**；
3. PyPI 在本机**不可达**（离线镜像不含 meshcat wheel），因此装不上。

因此 :class:`MeshcatBackend` 在构造时**主动探测**，不可用就抛
:class:`NotAvailableError` 并把上述原因写进消息——**绝不假装已实现浏览器三维
可视化**。切片目标里「被 Meshcat 呈现」一句在当前环境只能由 2D 兜底承担，
这一点在 CLI 的输出里同样如实打印。

matplotlib 兜底的坑（本机实测）
------------------------------
本机 matplotlib 可用，但 ``mpl_toolkits.mplot3d``（Axes3D）因多版本混装
**导入失败**。因此 :class:`FallbackBackend` **只用 2D**（``matplotlib`` 的普通
``Axes``），不依赖 ``mpl_toolkits.mplot3d``；matplotlib 整体缺失时抛
:class:`NotAvailableError`（不崩溃）。
"""

from __future__ import annotations

import importlib
from pathlib import Path
from typing import Any, List, Optional, Tuple

import numpy as np

__all__ = [
    "NotAvailableError",
    "Visualizer",
    "MeshcatBackend",
    "FallbackBackend",
    "make_visualizer",
    "backend_status",
    "MESHCAT_UNAVAILABLE_REASON",
]


#: Meshcat 不可用的**标准原因说明**（探测失败时写进异常消息）。
MESHCAT_UNAVAILABLE_REASON = (
    "Meshcat 服务端不可用：(1) 本机 PyPI 不可达（离线镜像不含 meshcat wheel）；"
    "(2) 未安装 meshcat 包（import meshcat 失败）；"
    "(3) Drake wheel 内只有 meshcat.js 前端资源、没有 Python 服务端"
    "（from pydrake import meshcat 失败）。"
    "**本项目不提供伪三维浏览器可视化**：需要三维呈现时请改用 meshcat 官方镜像，"
    "或用 2D 兜底（FallbackBackend）。"
)


class NotAvailableError(RuntimeError):
    """某个可视化后端在本机**不可用**。

    消息必须说明**为什么**不可用（缺哪个包 / 缺哪个能力）。
    """


class Visualizer:
    """可视化后端协议（结构化子类）。

    实现者必须提供：

    * ``name``：后端名（``"meshcat"`` / ``"fallback"``）；
    * :meth:`show_trajectory` —— 呈现一条轨迹；
    * :meth:`show_scene` —— 呈现场景；
    * :meth:`save` —— 把当前图像写到文件。
    """

    name = "abstract"

    def show_trajectory(self, scene: Any, trajectory: Any) -> None:
        """呈现一条轨迹（``scene`` = 场景装配产物，``trajectory`` = Trajectory）。"""
        raise NotImplementedError

    def show_scene(self, scene: Any, scenario: Any = None) -> None:
        """呈现场景（``scenario`` 可选：给了就一并画障碍物）。"""
        raise NotImplementedError

    def save(self, path: Any) -> Path:
        """把当前图像写到 ``path``（PNG），返回写出的路径。"""
        raise NotImplementedError


# ── Meshcat 后端（探测式）──────────────────────────────────────────────
def _probe_meshcat() -> Tuple[Optional[Any], str]:
    """探测 meshcat 服务端。

    返回 ``(module, reason)``：``module`` 非 ``None`` → 可用；
    ``module`` 为 ``None`` → ``reason`` 说明**为什么**不可用。
    """
    try:
        module = importlib.import_module("meshcat")
        return module, ""
    except Exception as exc:  # ImportError 或其子类的其它形态
        first = f"import meshcat 失败：{exc}"
    try:
        module = importlib.import_module("pydrake.meshcat")
        return module, ""
    except Exception as exc:
        second = f"from pydrake import meshcat 失败：{exc}"
    return None, f"{MESHCAT_UNAVAILABLE_REASON}（探测细节：{first}；{second}）"


class MeshcatBackend(Visualizer):
    """浏览器三维可视化后端（**探测式**：不可用即显式报错）。

    构造
    ----
    ``MeshcatBackend()``
        探测 meshcat；不可用 → :class:`NotAvailableError`（消息含原因）。

    注意
    ----
    * 真正的 meshcat 可用时，本后端会**打开浏览器窗口**（交互式），
      因此 CLI 在 headless 环境下**默认不启用**它；
    * 本机不可用，**没有**任何"假装能渲染"的降级路径——降级只发生在
      :func:`make_visualizer` 的**显式选择**上（选 fallback）。
    """

    name = "meshcat"

    def __init__(self, *, url: Optional[str] = None, open_browser: bool = True) -> None:
        module, reason = _probe_meshcat()
        if module is None:
            raise NotAvailableError(reason)
        self._module = module
        self._url = url
        self._open_browser = open_browser
        self._visualizer = None
        try:
            self._visualizer = module.Visualizer()
        except Exception as exc:  # 服务端起不来（端口占用 / 缺 zmq 等）
            raise NotAvailableError(
                f"meshcat 服务端启动失败：{exc}（{MESHCAT_UNAVAILABLE_REASON}）"
            ) from exc

    # -- 呈现 ------------------------------------------------------------
    def show_trajectory(self, scene: Any, trajectory: Any) -> None:
        """把轨迹画成一条折线（meshcat 的 ``lines`` 路径）。"""
        if self._visualizer is None:  # pragma: no cover - 构造失败即抛错
            raise NotAvailableError("meshcat 后端尚未初始化")
        points = _end_effector_points(scene, trajectory)
        if points.size == 0:
            return
        path = self._module.geometry.Trajectory(
            points[:, 0], points[:, 1], points[:, 2]
        )
        self._visualizer["trajectory"].set_object(
            self._module.geometry.PointsGeometry(
                points[:, 0], points[:, 1], points[:, 2]
            )
        )
        self._visualizer["trajectory_line"].set_object(
            self._module.geometry.Line(
                self._module.geometry.PointsGeometry(
                    points[:, 0], points[:, 1], points[:, 2]
                )
            )
        )
        del path  # 仅为可读性保留变量（不依赖 meshcat 的 Trajectory 几何）

    def show_scene(self, scene: Any, scenario: Any = None) -> None:
        """呈现场景（本机不可用时构造期已报错；这里只做最小绘制）。"""
        if self._visualizer is None:  # pragma: no cover
            raise NotAvailableError("meshcat 后端尚未初始化")
        for obstacle in getattr(scenario, "obstacles", ()) or ():
            size = np.asarray(obstacle.size, dtype=float)
            center = np.asarray(obstacle.xyz, dtype=float)
            self._visualizer[f"obstacle/{obstacle.name}"].set_object(
                self._module.geometry.Box(size[0], size[1], size[2]).translate(
                    center
                )
            )

    def save(self, path: Any) -> Path:
        """meshcat 是交互式的，**没有**"保存成图片"的语义。

        这里显式抛 :class:`NotAvailableError`（而不是悄悄写一个空文件）。
        """
        raise NotAvailableError(
            "meshcat 后端是交互式的，不支持保存为静态图片；"
            "需要落盘请用 FallbackBackend（matplotlib 2D）。"
        )


# ── matplotlib 2D 兜底 ─────────────────────────────────────────────────
def _probe_matplotlib() -> Tuple[Optional[Any], str]:
    """探测 matplotlib（返回 ``(module, reason)``）。"""
    try:
        module = importlib.import_module("matplotlib")
        return module, ""
    except Exception as exc:
        return None, f"matplotlib 不可用：{exc}（无法提供 2D 兜底可视化）"


class FallbackBackend(Visualizer):
    """matplotlib **2D** 投影兜底后端。

    呈现内容
    --------
    * **x–z 视图** 与 **y–z 视图**：末端轨迹的 2D 投影；
    * **关节角–时间** 视图：7 条关节曲线。

    约束
    ----
    * **不使用** ``mpl_toolkits.mplot3d``（本机 Axes3D 导入失败）；
    * 默认 matplotlib 后端是 ``Agg``（**headless 安全**：不弹窗、不连显示器）；
    * matplotlib 缺失 → :class:`NotAvailableError`（不崩溃）。
    """

    name = "fallback"

    def __init__(self, *, matplotlib_backend: str = "Agg") -> None:
        module, reason = _probe_matplotlib()
        if module is None:
            raise NotAvailableError(reason)
        self._matplotlib = module
        try:
            module.use(matplotlib_backend, force=True)
        except Exception:  # pragma: no cover - 后端名非法时退回默认
            pass
        self._pyplot = importlib.import_module("matplotlib.pyplot")
        self._figure = None

    # -- 呈现 ------------------------------------------------------------
    def show_trajectory(self, scene: Any, trajectory: Any) -> None:
        """画末端轨迹的两个 2D 视图 + 关节角–时间视图。"""
        figure = self._new_figure()
        points = _end_effector_points(scene, trajectory)
        axes = figure.subplots(2, 2)
        titles = ("x–z 视图", "y–z 视图")
        for index, (xs, ys, title) in enumerate(
            (
                (points[:, 0], points[:, 2], titles[0]),
                (points[:, 1], points[:, 2], titles[1]),
            )
        ):
            axis = axes[0][index]
            if points.size:
                axis.plot(xs, ys, linewidth=1.5)
                axis.scatter(xs[0], ys[0], marker="o", label="起点")
                axis.scatter(xs[-1], ys[-1], marker="x", label="终点")
                axis.legend(loc="best", fontsize="small")
            axis.set_title(f"末端轨迹 {title}")
            axis.set_xlabel("x (m)" if index == 0 else "y (m)")
            axis.set_ylabel("z (m)")
            axis.grid(True, alpha=0.3)

        axis = axes[1][0]
        matrix = np.asarray([s.q for s in trajectory.samples], dtype=float)
        times = np.asarray([s.t_abs for s in trajectory.samples], dtype=float)
        for joint_index in range(matrix.shape[1]):
            axis.plot(times, matrix[:, joint_index], linewidth=1.0, label=f"joint_{joint_index + 1}")
        axis.set_title("关节角随时间")
        axis.set_xlabel("t (s)")
        axis.set_ylabel("q (rad)")
        axis.grid(True, alpha=0.3)
        axis.legend(loc="best", fontsize="x-small")
        axes[1][1].axis("off")
        figure.tight_layout()

    def show_scene(self, scene: Any, scenario: Any = None) -> None:
        """画场景：世界系坐标轴 + 障碍物俯视/侧视投影。"""
        figure = self._new_figure()
        axes = figure.subplots(1, 2)
        for index, (axis, title) in enumerate(zip(axes, ("x–y 俯视", "x–z 侧视"))):
            axis.set_title(title)
            axis.set_xlabel("x (m)")
            axis.set_ylabel("y (m)" if index == 0 else "z (m)")
            axis.grid(True, alpha=0.3)
            axis.axhline(0.0, color="0.6", linewidth=0.8)
            axis.axvline(0.0, color="0.6", linewidth=0.8)
        for obstacle in getattr(scenario, "obstacles", ()) or ():
            size = np.asarray(obstacle.size, dtype=float)
            center = np.asarray(obstacle.xyz, dtype=float)
            axes[0].add_patch(
                self._pyplot.Rectangle(
                    (center[0] - size[0] / 2.0, center[1] - size[1] / 2.0),
                    size[0],
                    size[1],
                    fill=False,
                    linewidth=1.2,
                    label=obstacle.name,
                )
            )
            axes[1].add_patch(
                self._pyplot.Rectangle(
                    (center[0] - size[0] / 2.0, center[2] - size[2] / 2.0),
                    size[0],
                    size[2],
                    fill=False,
                    linewidth=1.2,
                    label=obstacle.name,
                )
            )
        for axis in axes:
            handles, labels = axis.get_legend_handles_labels()
            if handles:
                axis.legend(loc="best", fontsize="small")
        figure.tight_layout()

    def save(self, path: Any) -> Path:
        """把当前图像写到 ``path``（PNG）。"""
        if self._figure is None:
            # 没有内容时也产出一张空图（**非空文件**），便于流水线检查。
            self._new_figure()
        target = Path(path)
        if target.parent and not target.parent.exists():
            target.parent.mkdir(parents=True, exist_ok=True)
        self._figure.savefig(target, dpi=120)
        return target

    # -- 内部 ------------------------------------------------------------
    def _new_figure(self):
        """新建一张画布（并关闭上一张）。"""
        if self._figure is not None:
            try:
                self._pyplot.close(self._figure)
            except Exception:  # pragma: no cover - 关闭失败不影响出图
                pass
        self._figure = self._pyplot.figure(figsize=(9.0, 7.0))
        return self._figure


# ── 内部工具 ────────────────────────────────────────────────────────────
#: 末端 frame 名（RM75-B 的 TCP 近似：最后一个活动关节的子 frame）。
_END_EFFECTOR_FRAME = "link_7"

#: 2D 兜底最多画多少个末端点（避免几千点把 PNG 撑爆）。
_MAX_POINTS = 200


def _end_effector_points(scene: Any, trajectory: Any) -> np.ndarray:
    """返回轨迹的末端位置点列 ``(N, 3)``（世界系，米）。

    场景没有 plant / 上下文或 frame 不存在时返回**空数组**
    （绘图层据此只画坐标轴，不崩溃）。
    """
    samples = list(getattr(trajectory, "samples", ()) or ())
    if not samples:
        return np.zeros((0, 3), dtype=float)
    plant = getattr(scene, "plant", None)
    context = getattr(scene, "plant_context", None)
    if plant is None or context is None:
        return np.zeros((0, 3), dtype=float)
    try:
        frame = plant.GetFrameByName(_END_EFFECTOR_FRAME)
    except Exception:
        return np.zeros((0, 3), dtype=float)
    # **不复用场景的 plant_context**：改位形会污染规划链路的起点位形。
    context = plant.CreateDefaultContext()
    stride = max(1, len(samples) // _MAX_POINTS)
    rows: List[np.ndarray] = []
    for sample in samples[::stride]:
        plant.SetPositions(context, np.asarray(sample.q, dtype=float))
        pose = frame.CalcPoseInWorld(context)
        rows.append(np.asarray(pose.translation(), dtype=float))
    if not rows:
        return np.zeros((0, 3), dtype=float)
    return np.vstack(rows)


# ── 后端选择 ────────────────────────────────────────────────────────────
def backend_status() -> List[dict]:
    """返回各后端的可用性状态（诊断用）。

    每个条目：``{"name": ..., "available": bool, "reason": str}``。
    """
    status: List[dict] = []
    _, meshcat_reason = _probe_meshcat()
    status.append(
        {
            "name": "meshcat",
            "available": not meshcat_reason,
            "reason": meshcat_reason,
        }
    )
    _, mpl_reason = _probe_matplotlib()
    status.append(
        {
            "name": "fallback",
            "available": not mpl_reason,
            "reason": mpl_reason,
        }
    )
    return status


def make_visualizer(prefer: str = "auto") -> Visualizer:
    """构造一个可用后端。

    参数
    ----
    prefer:
        ``"auto"``（优先 meshcat，不可用则退回 2D 兜底）、
        ``"meshcat"``（**只**用 meshcat，不可用 → :class:`NotAvailableError`）、
        ``"fallback"``（**只**用 matplotlib 2D 兜底）。

    失败模式
    --------
    * ``prefer`` 不是这三个值 → :class:`ValueError`；
    * 指定的后端不可用 → :class:`NotAvailableError`（消息含原因）；
    * ``auto`` 下两个后端都不可用 → :class:`NotAvailableError`。
    """
    if prefer not in ("auto", "meshcat", "fallback"):
        raise ValueError(f"prefer 必须是 auto/meshcat/fallback，实际：{prefer!r}")
    if prefer in ("auto", "meshcat"):
        try:
            return MeshcatBackend()
        except NotAvailableError:
            if prefer == "meshcat":
                raise
    return FallbackBackend()
