#!/usr/bin/env python3
"""视觉网格链路（官方归档 → 解包 → STL→OBJ 转换 → CI fast 项）的正向/反向契约测试

背景（S09-T02）
---------------
归档物化后，``bash scripts/ci.sh fast`` 的 ``assets/visual-mesh`` 项必须由
``[SKIP]``（无官方 STL 源目录）转为 ``[PASS]``。本文件锁住三件事：

1. **正向**：官方 STL 已物化时，``scripts/convert_meshes.py --print-boxes`` 退出码 0
   并打印包围盒表（归档物化后的真实链路）。
2. **反向（缺输入）**：STL 源目录不存在时退出码 **2**（环境未就绪），
   不得以 0 伪装成功，也不得以 1 伪装业务失败。
3. **反向（防假绿灯）**：``scripts/ci.sh`` 里的视觉网格项必须走
   ``run_check_skippable``（缺失 → 显式 SKIP），不得被改成无条件 PASS。

本文件只跑轻量脚本（无 Drake 依赖、无规划负载），耗时 < 5 s。
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
CONVERT = REPO_ROOT / "scripts" / "convert_meshes.py"
CI_SCRIPT = REPO_ROOT / "scripts" / "ci.sh"
#: 官方 STL 源目录（由 scripts/extract_upstream_assets.py 物化，不入 git）。
OFFICIAL_MESH_DIR = (
    REPO_ROOT / "third_party" / ".work" / "rm75b" / "RM75" / "urdf" / "RM75-B" / "meshes"
)


def run(cmd: list[str]) -> subprocess.CompletedProcess:
    return subprocess.run(
        cmd, cwd=REPO_ROOT, capture_output=True, text=True, timeout=300
    )


def test_convert_meshes_missing_src_dir_exits_two(tmp_path):
    """缺 STL 源目录 → 退出码 2（环境未就绪），不是 0 也不是 1。"""
    missing = tmp_path / "no-such-meshes"
    proc = run(
        [
            sys.executable,
            str(CONVERT),
            "--src",
            str(missing),
            "--dst",
            str(tmp_path / "out"),
            "--print-boxes",
        ]
    )
    assert proc.returncode == 2, proc.stderr
    assert "STL 源目录不存在" in proc.stderr


def test_convert_meshes_empty_src_dir_exits_two(tmp_path):
    """空目录（无任何 .STL）→ 退出码 2，不得把「没有网格」当成功。"""
    empty = tmp_path / "empty-meshes"
    empty.mkdir()
    proc = run(
        [
            sys.executable,
            str(CONVERT),
            "--src",
            str(empty),
            "--dst",
            str(tmp_path / "out"),
            "--print-boxes",
        ]
    )
    assert proc.returncode == 2, proc.stdout + proc.stderr
    assert "没有找到任何 .STL" in proc.stderr


@pytest.mark.skipif(
    not OFFICIAL_MESH_DIR.is_dir(),
    reason="上游官方 STL 未物化（先运行 tools/fetch_upstream.py + scripts/extract_upstream_assets.py）",
)
def test_convert_meshes_on_official_stl_exits_zero(tmp_path):
    """官方 STL 已物化 → 退出码 0 且打印包围盒表（含 base_link 等 8 个 link）。"""
    proc = run(
        [
            sys.executable,
            str(CONVERT),
            "--src",
            str(OFFICIAL_MESH_DIR),
            "--dst",
            str(tmp_path / "meshes"),
            "--print-boxes",
        ]
    )
    assert proc.returncode == 0, proc.stdout + proc.stderr
    assert "base_link" in proc.stdout
    assert "extent(x,y,z)" in proc.stdout
    assert "hull_vol(m^3)" in proc.stdout


def test_ci_visual_mesh_item_keeps_skippable_semantics():
    """防假绿灯：CI 的视觉网格项必须保留 run_check_skippable（缺失即显式 SKIP）。"""
    text = CI_SCRIPT.read_text(encoding="utf-8")
    assert 'run_check_skippable "assets/visual-mesh"' in text
    # 该项不得被换成无条件 run_check（否则无归档时会被伪装成 PASS）。
    assert 'run_check "assets/visual-mesh"' not in text


def test_rel_to_repo_handles_outside_path(tmp_path):
    """仓库外的 --dst 不得让日志渲染抛 ValueError（否则一次成功转换被变成失败）。"""
    sys.path.insert(0, str(REPO_ROOT / "scripts"))
    try:
        import convert_meshes  # type: ignore
    finally:
        sys.path.pop(0)
    inside = REPO_ROOT / "models" / "rm75_drake" / "meshes" / "base_link.obj"
    assert convert_meshes.rel_to_repo(inside) == "models/rm75_drake/meshes/base_link.obj"
    outside = tmp_path / "base_link.obj"
    rendered = convert_meshes.rel_to_repo(outside)
    assert rendered.startswith("..")
