#!/usr/bin/env python3
"""离线 wheel 镜像（vendor/wheels）的清单生成、校验与 versions.lock 回写（T03）。

本模块是 ``tools/lock_deps.sh`` 的实现内核，同时被 ``tests/test_wheels_archive.py``
直接 import 做负例断言（格式错误、缺件、hash 不符都必须显式报错，禁止"永远绿灯"）。

职责边界
--------
* 归档目录 ``vendor/wheels/`` 是**本机离线镜像**，已被 .gitignore 忽略（``vendor/``）。
  入 git 的是本模块产出的**校验清单**与 ``versions.lock:wheels`` 映射，而不是二进制。
* Mosek wheel 受商业许可限制再分发，因此**整个归档都不入 git**，
  只保留 "文件名 → sha256" 的可审计证据（见 licenses/registry.yaml）。

子命令
------
    sha256sums   生成/刷新 SHA256SUMS（sha256sum 标准格式）
    verify       逐文件重算 sha256 并与 SHA256SUMS 比对
    write-lock   把 SHA256SUMS 的映射回写 versions.lock 的 wheels 段（保留注释）
    archive-info 生成 ARCHIVE_INFO.md（本机镜像说明 + 实测耗时/体积）

退出码：0 成功 / 1 校验失败 / 2 运行期错误（目录缺失、格式非法等）。
"""

from __future__ import annotations

import argparse
import hashlib
import re
import sys
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_LOCK = REPO_ROOT / "versions.lock"
DEFAULT_WHEELS_DIR = REPO_ROOT / "vendor" / "wheels"
DEFAULT_CONSTRAINTS_DIR = REPO_ROOT / "constraints"

SHA256SUMS_NAME = "SHA256SUMS"
ARCHIVE_INFO_NAME = "ARCHIVE_INFO.md"

#: 归档里除 wheel/sdist 之外的管理文件（不参与 sha256 清单）
MANAGEMENT_FILES = {SHA256SUMS_NAME, ARCHIVE_INFO_NAME}

SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
SHA256SUMS_LINE_RE = re.compile(r"^(?P<digest>[0-9a-f]{64}) [ *](?P<name>\S.*)$")


class ArchiveError(RuntimeError):
    """归档结构非法 / 缺件 / hash 不符。"""


# --------------------------------------------------------------------------- #
# 基础工具
# --------------------------------------------------------------------------- #
def compute_sha256(path: Path, chunk_size: int = 1 << 20) -> str:
    """流式计算文件 sha256（避免把几百 MB 的 wheel 读进内存）。"""
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        while True:
            chunk = handle.read(chunk_size)
            if not chunk:
                break
            digest.update(chunk)
    return digest.hexdigest()


def iter_archive_files(wheels_dir: Path) -> List[Path]:
    """列出归档内的发行文件（排除 SHA256SUMS / ARCHIVE_INFO.md）。"""
    if not wheels_dir.is_dir():
        raise ArchiveError(f"归档目录不存在: {wheels_dir}")
    files = [
        path
        for path in wheels_dir.iterdir()
        if path.is_file() and path.name not in MANAGEMENT_FILES
    ]
    return sorted(files, key=lambda p: p.name)


def build_manifest(wheels_dir: Path) -> Dict[str, str]:
    """重算归档内每个文件的 sha256 → {文件名: sha256}。"""
    return {path.name: compute_sha256(path) for path in iter_archive_files(wheels_dir)}


# --------------------------------------------------------------------------- #
# SHA256SUMS
# --------------------------------------------------------------------------- #
def format_sha256sums(manifest: Dict[str, str]) -> str:
    """渲染标准 sha256sum 文本（"<hash>  <name>"，按文件名排序）。"""
    return "".join(f"{digest}  {name}\n" for name, digest in sorted(manifest.items()))


def parse_sha256sums(text: str) -> Dict[str, str]:
    """解析 sha256sum 文本；任何格式错误都显式抛 :class:`ArchiveError`。"""
    manifest: Dict[str, str] = {}
    for lineno, raw in enumerate(text.splitlines(), start=1):
        line = raw.rstrip("\n")
        if not line.strip():
            continue
        match = SHA256SUMS_LINE_RE.match(line)
        if not match:
            raise ArchiveError(
                f"SHA256SUMS 第 {lineno} 行格式非法（期望 '<64位sha256>  <文件名>'）: {line!r}"
            )
        digest = match.group("digest")
        name = match.group("name").strip()
        if not SHA256_RE.match(digest):
            raise ArchiveError(f"SHA256SUMS 第 {lineno} 行 sha256 非法: {digest!r}")
        if name in manifest:
            raise ArchiveError(f"SHA256SUMS 中出现重复条目: {name}")
        manifest[name] = digest
    if not manifest:
        raise ArchiveError("SHA256SUMS 为空：归档里没有任何发行文件")
    return manifest


def write_sha256sums(wheels_dir: Path) -> Tuple[Path, Dict[str, str]]:
    """生成/刷新 SHA256SUMS，返回 (路径, 映射)。"""
    manifest = build_manifest(wheels_dir)
    target = wheels_dir / SHA256SUMS_NAME
    target.write_text(format_sha256sums(manifest), encoding="utf-8")
    return target, manifest


def verify_archive(wheels_dir: Path) -> List[str]:
    """校验归档与 SHA256SUMS 是否一致，返回问题列表（空列表 = 一致）。"""
    sums_file = wheels_dir / SHA256SUMS_NAME
    if not sums_file.is_file():
        raise ArchiveError(f"缺少校验清单: {sums_file}")
    expected = parse_sha256sums(sums_file.read_text(encoding="utf-8"))
    actual = build_manifest(wheels_dir)

    problems: List[str] = []
    for name, digest in expected.items():
        path = wheels_dir / name
        if not path.is_file():
            problems.append(f"缺失文件: {name}")
        elif actual.get(name) != digest:
            problems.append(f"sha256 不符: {name}（清单 {digest}，实际 {actual.get(name)}）")
    for name in sorted(set(actual) - set(expected)):
        problems.append(f"归档中多出未登记文件: {name}")
    return problems


# --------------------------------------------------------------------------- #
# versions.lock:wheels 回写（保留原文件的注释与排版）
# --------------------------------------------------------------------------- #
def render_wheels_block(manifest: Dict[str, str]) -> str:
    """渲染 versions.lock 的 wheels 段（2 空格缩进的映射）。"""
    if not manifest:
        return "wheels: {}\n"
    lines = ["wheels:"]
    for name in sorted(manifest):
        lines.append(f"  {name}: {manifest[name]}")
    return "\n".join(lines) + "\n"


def replace_wheels_block(lock_text: str, manifest: Dict[str, str]) -> str:
    """把 lock 文本里的 wheels 段整体替换为新映射（其余内容逐字保留）。"""
    lines = lock_text.splitlines(keepends=True)
    start = None
    for index, line in enumerate(lines):
        if line.rstrip("\n").startswith("wheels:"):
            start = index
            break
    if start is None:
        raise ArchiveError("versions.lock 中找不到 wheels: 段")

    end = start + 1
    while end < len(lines):
        candidate = lines[end]
        stripped = candidate.strip()
        if stripped and not candidate.startswith((" ", "\t")):
            break  # 下一个顶层键
        end += 1

    new_lines = lines[:start] + [render_wheels_block(manifest)] + lines[end:]
    return "".join(new_lines)


def write_lock(lock_path: Path, manifest: Dict[str, str]) -> None:
    """回写 versions.lock:wheels。"""
    text = lock_path.read_text(encoding="utf-8")
    lock_path.write_text(replace_wheels_block(text, manifest), encoding="utf-8")


# --------------------------------------------------------------------------- #
# 锁文件读取（复用 tools/gen_constraints.py 的版本解析，保证唯一真相源）
# --------------------------------------------------------------------------- #
def _load_gen_constraints():
    if str(REPO_ROOT) not in sys.path:
        sys.path.insert(0, str(REPO_ROOT))
    from tools import gen_constraints  # noqa: WPS433（延迟导入，避免包初始化副作用）

    return gen_constraints


def load_profile_info(lock_path: Path, profile: str) -> Dict[str, object]:
    """从 versions.lock 取出某 profile 的解析/下载参数。"""
    gen_constraints = _load_gen_constraints()
    lock = gen_constraints.load_lock(lock_path)
    gen_constraints.validate_lock(lock)
    if profile not in ("baseline", "fallback"):
        raise ArchiveError(f"未知 profile: {profile}（可选 baseline/fallback）")
    block = lock[profile]
    platforms = [lock["platform"]["platform_tag"]]
    platforms.extend(lock["platform"].get("extra_platform_tags", []) or [])
    requirements = [
        f"{name}=={version}"
        for name, version in sorted(gen_constraints.resolve_packages(lock, profile).items())
    ]
    return {
        "profile": block["profile"],
        "python": str(block["python"]),
        "python_tag": block["python_tag"],
        "platforms": platforms,
        "requirements": requirements,
        "supported": bool(block.get("supported", False)),
    }


# --------------------------------------------------------------------------- #
# ARCHIVE_INFO.md
# --------------------------------------------------------------------------- #
ARCHIVE_INFO_TEMPLATE = """# vendor/wheels —— 本机离线 wheel 镜像

> 本目录**不入 git**（`.gitignore` 的 `vendor/` 规则）。
> 入 git 的是：`versions.lock:wheels`（文件名 → sha256 映射）与本说明文档。

## 1. 为什么不入 git

* `drake` 依赖商业求解器 **Mosek 11.1.2**，其 wheel 受商业许可限制**再分发**；
* 官方 `robotic_arm`（RM_API2）上游许可未明示，同样不得再分发；
* 单是 Drake + Mosek 的体积就远超仓库可接受的二进制存量。

因此正确的交付形态是 **"锁 + 校验清单 + 归档脚本"**：
仓库里保留可复现的依赖锁（`versions.lock` + `constraints/`）、
可审计的校验证据（`versions.lock:wheels` → sha256）与可重建的脚本
（`tools/lock_deps.sh`），而二进制只落在各自机器的 `vendor/wheels/`。

## 2. 本次归档实测

| 项 | 值 |
|----|----|
| profile | `{profile}`（python {python}，tag {python_tag}） |
| 目标平台标签 | {platforms} |
| 生成时间 | {timestamp} |
| 下载耗时 | {duration} s |
| 文件数 | {file_count} |
| 总体积 | {total_size} |
| pip 版本 | {pip_version} |

包清单（{package_count} 个，含传递依赖）：

{packages}

耗时说明：

{notes}

## 3. 逐文件 sha256

见同目录 `SHA256SUMS`（标准 `sha256sum` 格式）。同一份映射已回写进
`versions.lock:wheels`，因此本镜像被移动/改名后仍可被校验。

## 4. 如何重建

```bash
# 联网重建（会重新下载并刷新 SHA256SUMS / ARCHIVE_INFO.md）
bash tools/lock_deps.sh --download --profile {profile}

# 离线安装（不触网）
python3 -m pip install --no-index --find-links vendor/wheels \\
    -c constraints/{profile}.txt --require-hashes -r <requirements.txt>
```

## 5. 如何校验

```bash
bash tools/lock_deps.sh --verify        # 全量 sha256 比对
python3 tools/verify_env.py --wheels    # 同一套校验的 CLI 入口（T04）
```

校验失败时脚本以非 0 退出码报错并逐条列出差异文件。
"""


def write_archive_info(
    wheels_dir: Path,
    profile_info: Dict[str, object],
    duration: float,
    timestamp: str,
    pip_version: str,
    notes: Sequence[str] = (),
) -> Path:
    """写 ARCHIVE_INFO.md（把实测数据填进模板）。"""
    manifest = build_manifest(wheels_dir)
    total = sum((wheels_dir / name).stat().st_size for name in manifest)
    packages = "\n".join(f"- `{req}`" for req in profile_info["requirements"])
    note_lines = "\n".join(f"- {note}" for note in notes) or "- （无）"
    content = ARCHIVE_INFO_TEMPLATE.format(
        profile=profile_info["profile"],
        python=profile_info["python"],
        python_tag=profile_info["python_tag"],
        platforms="、".join(profile_info["platforms"]),
        timestamp=timestamp,
        duration=f"{duration:.1f}",
        file_count=len(manifest),
        total_size=_human_size(total),
        pip_version=pip_version,
        package_count=len(profile_info["requirements"]),
        packages=packages,
        notes=note_lines,
    )
    target = wheels_dir / ARCHIVE_INFO_NAME
    target.write_text(content, encoding="utf-8")
    return target


def _human_size(num: int) -> str:
    value = float(num)
    for unit in ("B", "KiB", "MiB", "GiB"):
        if value < 1024 or unit == "GiB":
            return f"{value:.1f} {unit}（{num} 字节）"
        value /= 1024
    return f"{num} 字节"


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #
def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="vendor/wheels 归档工具内核")
    parser.add_argument("--lock", type=Path, default=DEFAULT_LOCK)
    parser.add_argument("--dir", dest="wheels_dir", type=Path, default=DEFAULT_WHEELS_DIR)
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("sha256sums", help="生成/刷新 SHA256SUMS")
    sub.add_parser("verify", help="校验归档与 SHA256SUMS 一致性")

    lock_cmd = sub.add_parser("write-lock", help="把 SHA256SUMS 回写 versions.lock:wheels")
    lock_cmd.add_argument("--manifest", type=Path, default=None, help="SHA256SUMS 路径")

    profile_cmd = sub.add_parser("profile-info", help="输出 profile 的下载参数（shell 可 source）")
    profile_cmd.add_argument("--profile", default="baseline")

    info_cmd = sub.add_parser("archive-info", help="生成 ARCHIVE_INFO.md")
    info_cmd.add_argument("--profile", default="baseline")
    info_cmd.add_argument("--duration", type=float, default=0.0)
    info_cmd.add_argument("--timestamp", default="")
    info_cmd.add_argument("--pip-version", default="unknown")
    info_cmd.add_argument("--note", action="append", default=[], help="追加一条耗时/环境说明")

    args = parser.parse_args(list(argv) if argv is not None else None)

    try:
        if args.command == "sha256sums":
            path, manifest = write_sha256sums(args.wheels_dir)
            print(f"wheels_lock: 已生成 {path}（{len(manifest)} 个文件）")
            return 0

        if args.command == "verify":
            problems = verify_archive(args.wheels_dir)
            if problems:
                for problem in problems:
                    print(f"wheels_lock: {problem}", file=sys.stderr)
                print(f"wheels_lock: 归档校验失败，共 {len(problems)} 处差异", file=sys.stderr)
                return 1
            print(f"wheels_lock: 归档校验通过（{args.wheels_dir}）")
            return 0

        if args.command == "write-lock":
            manifest_path = args.manifest or (args.wheels_dir / SHA256SUMS_NAME)
            if not manifest_path.is_file():
                raise ArchiveError(f"缺少校验清单: {manifest_path}")
            manifest = parse_sha256sums(manifest_path.read_text(encoding="utf-8"))
            write_lock(args.lock, manifest)
            print(f"wheels_lock: 已回写 {args.lock} 的 wheels 段（{len(manifest)} 条）")
            return 0

        if args.command == "profile-info":
            info = load_profile_info(args.lock, args.profile)
            print(f"PROFILE={info['profile']}")
            print(f"PYTHON_VERSION={info['python']}")
            print(f"PYTHON_TAG={info['python_tag']}")
            print(f"PLATFORMS=({' '.join(info['platforms'])})")
            print("REQUIREMENTS=(" + " ".join(f"'{req}'" for req in info["requirements"]) + ")")
            return 0

        if args.command == "archive-info":
            info = load_profile_info(args.lock, args.profile)
            path = write_archive_info(
                args.wheels_dir,
                info,
                duration=args.duration,
                timestamp=args.timestamp or "(未记录)",
                pip_version=args.pip_version,
                notes=args.note,
            )
            print(f"wheels_lock: 已生成 {path}")
            return 0
    except ArchiveError as exc:
        print(f"wheels_lock: {exc}", file=sys.stderr)
        return 2
    except Exception as exc:  # pragma: no cover - 兜底：显式失败而不是静默通过
        print(f"wheels_lock: 未预期的错误: {exc}", file=sys.stderr)
        return 2
    return 2


if __name__ == "__main__":
    sys.exit(main())
