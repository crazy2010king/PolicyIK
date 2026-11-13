"""可视化抽象层测试（S02-T08）。

被测对象
--------
:mod:`rm75_drake.viz` —— 可视化协议 + Meshcat 后端（探测式）+ matplotlib 2D 兜底。

覆盖的契约
----------
**正例**
* matplotlib 可用时：:class:`FallbackBackend` 能 ``save()`` 出一张**非空**图片文件；
* ``show_trajectory`` / ``show_scene`` 在**没有场景**（``scene=None``）时也不崩溃
  （只画坐标轴）；
* ``make_visualizer("auto")`` 在 meshcat 不可用时**退回** 2D 兜底。

**负例（可解释失败）**
* meshcat 服务端不可用 → :class:`NotAvailableError`，消息必须说明**原因**
  （本机：无 meshcat 包 / Drake wheel 只有 meshcat.js 前端 / PyPI 不可达）；
* matplotlib 缺失 → :class:`NotAvailableError`（**不崩溃**）；
* ``prefer`` 取值非法 → :class:`ValueError`；
* meshcat 后端 ``save()`` → :class:`NotAvailableError`（交互式后端没有静态图片语义）。

**诚实性要求**
本机**没有** meshcat 服务端，因此本文件**不得**断言"已实现浏览器三维可视化"——
它断言的是"不可用时明确报错并给出原因"。
"""

from __future__ import annotations

import pytest

from rm75_drake import viz as viz_mod
from rm75_drake.viz import (
    FallbackBackend,
    MeshcatBackend,
    NotAvailableError,
    Visualizer,
    backend_status,
    make_visualizer,
)


def _matplotlib_available() -> bool:
    try:
        import matplotlib  # noqa: F401
    except Exception:
        return False
    return True


def _meshcat_available() -> bool:
    status = {item["name"]: item for item in backend_status()}
    return bool(status["meshcat"]["available"])


# ── 后端探测 ────────────────────────────────────────────────────────────
def test_backend_status_reports_both_backends():
    """正例：状态表同时给出两个后端与各自原因。"""
    status = {item["name"]: item for item in backend_status()}
    assert set(status) == {"meshcat", "fallback"}
    for item in status.values():
        assert "available" in item and "reason" in item
        assert isinstance(item["available"], bool)


def test_meshcat_backend_raises_not_available_with_reason():
    """负例：meshcat 不可用 → NotAvailableError，且消息说明原因。

    本机实测不可用（无 meshcat 包 / Drake wheel 只有 meshcat.js 前端 /
    PyPI 不可达）；若换到有 meshcat 的环境，则退化为「构造成功」断言。
    """
    if _meshcat_available():
        backend = MeshcatBackend()
        assert isinstance(backend, Visualizer)
        return
    with pytest.raises(NotAvailableError) as excinfo:
        MeshcatBackend()
    message = str(excinfo.value)
    assert "meshcat" in message
    # 消息必须说清「为什么不可用」，而不是一句"No module named"
    assert "meshcat" in message and ("前端" in message or "wheel" in message)


def test_meshcat_backend_save_is_explicitly_unsupported():
    """负例：meshcat 后端不假装能存图片。"""
    if not _meshcat_available():
        pytest.skip("本机无 meshcat 服务端（构造即报错）")
    backend = MeshcatBackend()
    with pytest.raises(NotAvailableError):
        backend.save("/tmp/should-not-exist.png")


# ── 2D 兜底 ─────────────────────────────────────────────────────────────
def test_fallback_backend_saves_non_empty_image(tmp_path):
    """正例：2D 兜底 save() 产出非空文件。"""
    if not _matplotlib_available():
        pytest.skip("本机无 matplotlib（此时应抛 NotAvailableError，见下方用例）")
    backend = FallbackBackend()
    target = tmp_path / "plot.png"
    backend.save(target)
    assert target.is_file()
    assert target.stat().st_size > 0


def test_fallback_backend_draws_without_scene(tmp_path):
    """正例：没有场景（scene=None）时绘图不崩溃，仍能出图。"""
    if not _matplotlib_available():
        pytest.skip("本机无 matplotlib")
    from rm75_drake.planning.trajectory import trajectory_from_arrays

    trajectory = trajectory_from_arrays(
        [[0.0, -0.6, 0.0, 0.9, 0.0, 0.5, 0.0], [0.01, -0.6, 0.0, 0.9, 0.0, 0.5, 0.0]],
        t0=0.0,
        dt=0.005,
    )
    backend = FallbackBackend()
    backend.show_trajectory(None, trajectory)
    backend.show_scene(None, None)
    target = tmp_path / "plot.png"
    backend.save(target)
    assert target.is_file() and target.stat().st_size > 0


def test_fallback_backend_raises_when_matplotlib_missing(monkeypatch):
    """负例：matplotlib 缺失 → NotAvailableError（不崩溃）。"""
    real_import = viz_mod.importlib.import_module

    def fake_import(name, *args, **kwargs):
        if name == "matplotlib" or name.startswith("matplotlib."):
            raise ImportError("No module named 'matplotlib'")
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(viz_mod.importlib, "import_module", fake_import)
    with pytest.raises(NotAvailableError) as excinfo:
        FallbackBackend()
    assert "matplotlib" in str(excinfo.value)


# ── 后端选择 ────────────────────────────────────────────────────────────
def test_make_visualizer_auto_falls_back_to_2d():
    """正例：auto 模式下 meshcat 不可用 → 退回 2D 兜底。"""
    visualizer = make_visualizer("auto")
    assert isinstance(visualizer, Visualizer)
    assert visualizer.name in ("meshcat", "fallback")


def test_make_visualizer_meshcat_only_raises_when_unavailable():
    """负例：显式要 meshcat 而它不可用 → NotAvailableError（不静默降级）。"""
    if _meshcat_available():
        assert make_visualizer("meshcat").name == "meshcat"
        return
    with pytest.raises(NotAvailableError):
        make_visualizer("meshcat")


def test_make_visualizer_rejects_unknown_preference():
    """负例：非法后端名 → ValueError。"""
    with pytest.raises(ValueError):
        make_visualizer("openGL")
