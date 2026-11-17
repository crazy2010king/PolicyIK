#!/usr/bin/env python3
"""scripts/convert_meshes.py —— 官方 RM75-B STL → OBJ 批量转换 + 包围盒表

为什么需要（T01）
----------------
Drake 1.51.1 / 1.55.0 都不解析 ``.stl``（``MakeConvexHull`` 只吃 .obj/.vtk/.gltf），
而官方 RM75-B URDF 的 collision 引用的是 ``package://rm_models/.../meshes/*.STL``。
本脚本把上游 STL 转成 OBJ 并**打印每个 link 的轴对齐包围盒**，作为 S01 改造版 URDF
把 collision 换成 ``<box>`` 原始几何体时的尺寸/中心真值来源。

用法
----
    # 1) 先物化上游资产（归档 → third_party/.work/rm75b/）
    python3 scripts/extract_upstream_assets.py

    # 2) 批量转换 + 打印包围盒表
    python3 scripts/convert_meshes.py
    python3 scripts/convert_meshes.py --src <stl目录> --dst <obj目录> --print-boxes

退出码：0 成功；1 输入/转换错误；2 环境或参数错误（目录不存在等）。
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "tools"))

from mesh_tools import MeshError, load_stl, mesh_stats, write_obj  # noqa: E402

#: 上游提取目录（由 scripts/extract_upstream_assets.py 物化）。
DEFAULT_SRC = REPO_ROOT / "third_party" / ".work" / "rm75b" / "RM75" / "urdf" / "RM75-B" / "meshes"
#: 派生网格输出目录（.gitignore 已排除 models/**/meshes/）。
DEFAULT_DST = REPO_ROOT / "models" / "rm75_drake" / "meshes"

EXIT_OK = 0
EXIT_FAIL = 1
EXIT_ENV = 2


def rel_to_repo(path: Path) -> str:
    """把路径渲染成可读形式：仓库内用仓库相对路径，仓库外用 ``../`` 相对形式。

    与 tools/fetch_upstream.py 的同名函数同一约定：**绝不**因为路径在仓库外
    就抛 ValueError（``Path.relative_to`` 会抛），否则一次本已成功的转换
    会被日志渲染变成失败。
    """
    try:
        return str(path.relative_to(REPO_ROOT))
    except ValueError:
        return os.path.relpath(path, REPO_ROOT)


def find_stl_files(src: Path) -> list[Path]:
    """返回目录下全部 STL 文件（大小写不敏感，按文件名排序 → 输出顺序确定）。"""
    return sorted(
        (p for p in src.iterdir() if p.is_file() and p.suffix.lower() == ".stl"),
        key=lambda p: p.name,
    )


def print_boxes(rows: list[dict]) -> None:
    """打印包围盒表（每行一个 link）。"""
    header = (
        f"{'link':<12} {'min(x,y,z)':<34} {'max(x,y,z)':<34} "
        f"{'extent(x,y,z)':<30} {'center(x,y,z)':<34}"
    )
    print(header)
    print("-" * len(header))
    for r in rows:
        def fmt(vals) -> str:
            return "(" + ", ".join(f"{v: .6f}" for v in vals) + ")"

        print(
            f"{r['name']:<12} {fmt(r['min']):<34} {fmt(r['max']):<34} "
            f"{fmt(r['extent']):<30} {fmt(r['center']):<34}"
        )
    print()
    print(f"{'link':<12} {'verts':>8} {'faces':>8} {'volume(m^3)':>14} "
          f"{'hull_vtx':>10} {'hull_vol(m^3)':>15}")
    print("-" * 70)
    for r in rows:
        print(
            f"{r['name']:<12} {r['vertices']:>8} {r['faces']:>8} "
            f"{r['volume']:>14.8f} {r['hull_vertices']:>10} {r['hull_volume']:>15.8f}"
        )


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        prog="convert_meshes.py",
        description="官方 RM75-B STL → OBJ 批量转换并打印包围盒表",
    )
    ap.add_argument("--src", default=str(DEFAULT_SRC), help="STL 输入目录")
    ap.add_argument("--dst", default=str(DEFAULT_DST), help="OBJ 输出目录")
    ap.add_argument(
        "--print-boxes", action="store_true", help="打印每个 link 的包围盒表"
    )
    ap.add_argument(
        "--no-convert", action="store_true", help="只测量打印，不写 OBJ"
    )
    args = ap.parse_args(argv)

    src = Path(args.src).expanduser()
    dst = Path(args.dst).expanduser()

    if not src.is_dir():
        print(
            f"[convert_meshes][ERROR] STL 源目录不存在：{src}\n"
            "  先运行 `python3 scripts/extract_upstream_assets.py` 物化上游资产。",
            file=sys.stderr,
        )
        return EXIT_ENV

    stl_files = find_stl_files(src)
    if not stl_files:
        print(f"[convert_meshes][ERROR] {src} 下没有找到任何 .STL 文件", file=sys.stderr)
        return EXIT_ENV

    rows: list[dict] = []
    for stl_path in stl_files:
        try:
            mesh = load_stl(stl_path)
        except MeshError as exc:
            print(f"[convert_meshes][ERROR] 解析失败：{stl_path.name}（{exc}）", file=sys.stderr)
            return EXIT_FAIL
        stats = mesh_stats(mesh)
        stats["name"] = stl_path.stem
        rows.append(stats)
        if not args.no_convert:
            out = dst / f"{stl_path.stem}.obj"
            try:
                write_obj(mesh, out, source=stl_path.name)
            except (OSError, MeshError) as exc:
                print(
                    f"[convert_meshes][ERROR] 写出失败：{out}（{exc}）", file=sys.stderr
                )
                return EXIT_FAIL
            print(f"[convert_meshes] {stl_path.name} -> {rel_to_repo(out)}")

    # 包围盒表是本脚本的主要产物（改造版 URDF 的 collision box 真值来源），始终打印；
    # --print-boxes 保留为显式开关（与计划里的 CLI 约定一致），不改变默认行为。
    print_boxes(rows)
    return EXIT_OK


if __name__ == "__main__":
    sys.exit(main())
