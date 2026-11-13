#!/usr/bin/env python3
"""双臂 URDF 变体生成器 CLI（S04-T01）。

用途
----
从 S01 的官方改造版 URDF 生成「每臂一份重命名变体」，供双臂在同一
``MultibodyPlant`` 里装配（R005）：

    models/rm75_drake/rm75_left.urdf
    models/rm75_drake/rm75_right.urdf
    models/rm75_drake/rmg24_left.urdf
    models/rm75_drake/rmg24_right.urdf

变体**只改名**（robot / link / joint / collision），**不改任何数值**——
惯量、限位、碰撞几何与原点逐字段等于源文件（S01 的 URDF 仍是唯一模型真相源）。

用法
----
```bash
# 生成（默认写到 models/rm75_drake/）
python3 tools/gen_dual_arm_models.py

# CI 校验：变体缺失或与生成器不一致 → 打印差异并以退出码 1 结束
python3 tools/gen_dual_arm_models.py --check
```

退出码
------
* ``0`` —— 生成成功 / 校验通过（四个变体与生成器**字节级**一致）；
* ``1`` —— 校验失败（变体缺失或已过期；``--check`` 专用）；
* ``2`` —— 用法或环境错误（参数非法、源 URDF 不可读等）。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

# ── 自引导 sys.path（与 tools/verify_env.py 同一做法）────────────────────
# 仓库根 = <root>/src/rm75_drake/... → 仓库根 = parents[2]
REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = REPO_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from rm75_drake.bimanual.model_variants import (  # noqa: E402
    VariantError,
    VariantMismatchError,
    check_variants,
    models_package_dir,
    write_variants,
)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="gen_dual_arm_models.py",
        description="生成（或校验）双臂 URDF 重命名变体。",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="只校验变体是否与生成器一致（不写盘）；不一致则退出码 1。",
    )
    parser.add_argument(
        "--out-dir",
        default=None,
        help="输出目录（默认 models/rm75_drake/）。",
    )
    return parser


def _print_report(report: dict) -> None:
    for vid, entry in report.items():
        print(
            f"  {vid:<12} robot={entry['robot_name']:<14} "
            f"links={entry['links']:<3} joints={entry['joints']:<3} "
            f"collisions={entry['collisions']:<3} status={entry['status']}"
        )


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    out_dir = Path(args.out_dir) if args.out_dir else models_package_dir()

    if args.check:
        report = check_variants(out_dir)
        print(f"[CHECK] 双臂 URDF 变体校验目录：{out_dir}")
        _print_report(report)
        bad = {
            vid: entry
            for vid, entry in report.items()
            if entry["status"] != "ok"
        }
        if bad:
            print("[FAIL] 以下变体与生成器不一致（缺失或已过期）：")
            for vid, entry in bad.items():
                print(f"  - {vid}: status={entry['status']} → {entry['path']}")
            print(
                "[HINT] 重新生成：python3 tools/gen_dual_arm_models.py"
            )
            return 1
        print("[PASS] 四个变体均与生成器字节级一致。")
        return 0

    try:
        report = write_variants(check_only=False, out_dir=out_dir)
    except VariantMismatchError as exc:  # pragma: no cover - 写盘路径不产生此错
        print(f"[FAIL] {exc}", file=sys.stderr)
        return 1
    except VariantError as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 2
    except OSError as exc:
        print(f"[ERROR] 写入失败：{exc}", file=sys.stderr)
        return 2

    print(f"[WRITE] 双臂 URDF 变体输出目录：{out_dir}")
    _print_report(report)
    print("[OK] 已生成四个变体（只改名，数值与源 URDF 逐字段相同）。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
