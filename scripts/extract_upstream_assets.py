#!/usr/bin/env python3
"""scripts/extract_upstream_assets.py —— 把上游归档解包到本机工作目录

用途（T01）
-----------
``tools/fetch_upstream.py`` 产出的是**归档**（tar.gz，不入 git）。
本脚本把 ``rm_models`` 归档里的 ``RM75/urdf/RM75-B/`` 子集解到
``third_party/.work/rm75b/``（.gitignore 已排除），供：

* ``scripts/convert_meshes.py`` 做 STL → OBJ 转换与包围盒测量；
* S01-T02 写改造版 URDF 时核对官方惯量 / 限位 / 碰撞尺寸。

真相源 = ``versions.lock:upstream.rm_models.archive.filename``（脚本不硬编码文件名）。

退出码：0 成功；1 归档缺失 / 解包失败；2 环境或参数错误。
"""

from __future__ import annotations

import argparse
import sys
import tarfile
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))

from rm75_drake.paths import third_party_dir, versions_lock_path  # noqa: E402

DEFAULT_DEST = third_party_dir / ".work" / "rm75b"
#: 归档内需要解出的子集前缀（与 versions.lock 的 subset 一致）。
RM75B_PREFIX = "RM75/urdf/RM75-B/"


def load_lock_asset(name: str) -> dict:
    import yaml

    with versions_lock_path.open("r", encoding="utf-8") as fh:
        lock = yaml.safe_load(fh) or {}
    upstream = lock.get("upstream") or {}
    spec = upstream.get(name)
    if not isinstance(spec, dict):
        raise SystemExit(f"[extract] versions.lock:upstream 中没有资产 {name}")
    return spec


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        prog="extract_upstream_assets.py",
        description="把上游归档解包到 third_party/.work/（不入 git）",
    )
    ap.add_argument("--asset", default="rm_models", help="versions.lock:upstream 里的资产名")
    ap.add_argument("--dest", default=str(DEFAULT_DEST), help="解包目标目录")
    ap.add_argument(
        "--prefix",
        default=RM75B_PREFIX,
        help=f"只解出该前缀下的条目（默认 {RM75B_PREFIX}）",
    )
    args = ap.parse_args(argv)

    try:
        spec = load_lock_asset(args.asset)
    except SystemExit:
        raise
    except Exception as exc:  # pragma: no cover - 环境异常
        print(f"[extract][ERROR] 读取 versions.lock 失败：{exc}", file=sys.stderr)
        return 2

    archive_name = ((spec.get("archive") or {}) or {}).get("filename")
    if not archive_name:
        print(
            f"[extract][ERROR] versions.lock:upstream.{args.asset}.archive.filename 缺失"
            "（先运行 `python3 tools/fetch_upstream.py`）",
            file=sys.stderr,
        )
        return 2

    archive_path = third_party_dir / "archives" / archive_name
    if not archive_path.is_file():
        print(
            f"[extract][ERROR] 归档不存在：{archive_path}\n"
            "  先运行 `python3 tools/fetch_upstream.py` 物化上游归档。",
            file=sys.stderr,
        )
        return 1

    dest = Path(args.dest).expanduser()
    dest.mkdir(parents=True, exist_ok=True)

    extracted = 0
    try:
        with tarfile.open(archive_path, "r:gz") as tar:
            for member in tar.getmembers():
                name = member.name
                if not name.startswith(args.prefix):
                    continue
                if not member.isfile():
                    continue
                # 防御路径穿越（tar 条目名带 ../ 或绝对路径）
                rel = Path(name)
                if rel.is_absolute() or ".." in rel.parts:
                    print(f"[extract][ERROR] 归档条目路径非法：{name}", file=sys.stderr)
                    return 1
                target = dest / rel
                target.parent.mkdir(parents=True, exist_ok=True)
                src = tar.extractfile(member)
                if src is None:
                    continue
                target.write_bytes(src.read())
                extracted += 1
    except (tarfile.TarError, OSError) as exc:
        print(f"[extract][ERROR] 解包失败：{archive_path}（{exc}）", file=sys.stderr)
        return 1

    print(f"[extract] {archive_name} → {dest}（{extracted} 个文件，前缀 {args.prefix}）")
    stl_dir = dest / args.prefix / "meshes"
    if stl_dir.is_dir():
        stls = sorted(stl_dir.glob("*.STL"))
        print(f"[extract] STL 网格 {len(stls)} 个：{[p.stem for p in stls]}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
