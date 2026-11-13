#!/usr/bin/env python3
"""厂商解算接口禁用名单静态扫描器（T05 / R008 / D003）。

为什么需要它
------------
本项目的主链路必须**零调用**厂商自带的运动学/轨迹求解接口（否则等于把
行为交给一个不可审计的黑盒）。"零调用"这件事不能靠口头承诺，必须能被
CI 反复执行并以退出码表达。

唯一真相源
----------
禁用名单**只**来自 ``versions.lock`` 的 ``forbidden_apis`` 段；白名单来自
``allowed_apis`` 段。本脚本**不硬编码任何接口名**——否则名单会出现第二份
真相源，迟早与锁文件漂移。

匹配规则
--------
只把"调用点"算作违规：``NAME`` 前后不构成标识符、后面紧跟（可含空白）
左括号。形如 ``NAME(`` 的调用会被命中；而 ``NAME`` 作为更长标识符的前缀
（例如白名单里的那个 CANFD 变体）因为后面不是左括号，**不会被前缀误伤**。
命中后再用 ``allowed_apis`` 白名单复核一遍（名单若出现冲突，以白名单为准）。

扫描范围
--------
默认 ``src/``、``tools/``、``scripts/`` 下的代码文件
（``.py/.cpp/.cc/.cxx/.h/.hpp/.hxx``）。``.md`` 等文档默认不扫——
文档里提到接口名不算调用。

退出码
------
0 = 无命中；1 = 有命中；2 = 运行期错误（锁文件不可读等）。

用法::

    python3 scripts/scan_forbidden_apis.py             # 扫默认目录
    python3 scripts/scan_forbidden_apis.py src/        # 扫指定路径
    python3 scripts/scan_forbidden_apis.py --json      # 机器可读输出
    python3 scripts/scan_forbidden_apis.py --paths src tools --lock versions.lock
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_LOCK = REPO_ROOT / "versions.lock"

#: 默认扫描的目录（相对仓库根）。
DEFAULT_SCAN_DIRS: Tuple[str, ...] = ("src", "tools", "scripts")

#: 参与扫描的代码扩展名；文档/数据文件（.md/.yaml/.txt/.json）不算调用点。
CODE_EXTENSIONS: Tuple[str, ...] = (
    ".py",
    ".cpp",
    ".cc",
    ".cxx",
    ".h",
    ".hpp",
    ".hxx",
)

#: 扫描时跳过的目录名。
SKIP_DIR_NAMES = {
    ".git",
    ".hg",
    ".svn",
    "__pycache__",
    ".pytest_cache",
    ".mypy_cache",
    ".ruff_cache",
    "vendor",
    ".venv",
    "venv",
    "node_modules",
    "build",
    "dist",
    ".eggs",
}


class ScannerError(RuntimeError):
    """运行期错误（锁文件不可读 / 名单缺失等）。"""


# --------------------------------------------------------------------------- #
# 名单加载（唯一真相源 = versions.lock）
# --------------------------------------------------------------------------- #
def load_names(lock_path: Path = DEFAULT_LOCK) -> Tuple[List[str], List[str]]:
    """从锁文件读出 (forbidden_apis, allowed_apis)。"""
    if not lock_path.is_file():
        raise ScannerError(f"锁文件不存在: {lock_path}")
    try:
        import yaml
    except ImportError as exc:  # pragma: no cover - 环境缺依赖
        raise ScannerError("需要 PyYAML（pip install pyyaml）") from exc
    try:
        data: Any = yaml.safe_load(lock_path.read_text(encoding="utf-8"))
    except Exception as exc:
        raise ScannerError(f"锁文件解析失败: {lock_path}（{exc}）") from exc
    if not isinstance(data, dict):
        raise ScannerError(f"锁文件结构非法（应为映射）: {lock_path}")

    forbidden = data.get("forbidden_apis")
    allowed = data.get("allowed_apis")
    if not isinstance(forbidden, list) or not forbidden:
        raise ScannerError(f"{lock_path}:forbidden_apis 缺失或为空")
    if not isinstance(allowed, list):
        allowed = []

    forbidden_names = [str(name) for name in forbidden]
    allowed_names = [str(name) for name in allowed]
    overlap = sorted(set(forbidden_names) & set(allowed_names))
    if overlap:
        raise ScannerError(
            "同一接口同时出现在 forbidden_apis 与 allowed_apis: " + ", ".join(overlap)
        )
    return forbidden_names, allowed_names


def build_patterns(
    forbidden: Sequence[str], allowed: Sequence[str]
) -> List[Tuple[str, "re.Pattern[str]"]]:
    """构造 (接口名, 匹配调用点的正则) 列表。

    正则语义：接口名前面不能是标识符字符（避免 ``x_NAME(`` 误判），
    后面必须紧跟（可含空白）左括号（只算调用点，不算文档提及）。
    """
    allowed_set = set(allowed)
    patterns: List[Tuple[str, "re.Pattern[str]"]] = []
    for name in forbidden:
        if name in allowed_set:
            continue  # 白名单优先
        patterns.append(
            (name, re.compile(r"(?<![A-Za-z0-9_])" + re.escape(name) + r"\s*\("))
        )
    return patterns


# --------------------------------------------------------------------------- #
# 文件遍历
# --------------------------------------------------------------------------- #
def iter_code_files(paths: Iterable[Path]) -> List[Path]:
    """展开给定路径下的代码文件（目录递归，文件直接收录）。"""
    found: List[Path] = []
    for raw in paths:
        if raw.is_file():
            if raw.suffix in CODE_EXTENSIONS:
                found.append(raw)
            continue
        if not raw.is_dir():
            continue
        for candidate in sorted(raw.rglob("*")):
            if not candidate.is_file():
                continue
            if candidate.suffix not in CODE_EXTENSIONS:
                continue
            if any(part in SKIP_DIR_NAMES for part in candidate.parts):
                continue
            found.append(candidate)
    # 去重并保持稳定顺序
    unique: List[Path] = []
    seen = set()
    for item in found:
        key = str(item)
        if key in seen:
            continue
        seen.add(key)
        unique.append(item)
    return unique


# --------------------------------------------------------------------------- #
# 扫描
# --------------------------------------------------------------------------- #
def scan_text(
    text: str,
    patterns: Sequence[Tuple[str, "re.Pattern[str]"]],
    allowed: Sequence[str],
) -> List[Dict[str, Any]]:
    """逐行扫描文本，返回命中列表。"""
    allowed_set = set(allowed)
    hits: List[Dict[str, Any]] = []
    for lineno, line in enumerate(text.splitlines(), start=1):
        stripped = line.strip()
        for name, pattern in patterns:
            if name in allowed_set:
                continue
            if pattern.search(line):
                hits.append(
                    {"line": lineno, "api": name, "content": stripped, "raw": line}
                )
                break  # 一行只报一次（命中多个接口时取第一个）
    return hits


def scan_paths(
    paths: Sequence[Path],
    forbidden: Sequence[str],
    allowed: Sequence[str],
) -> Tuple[List[Dict[str, Any]], List[Path], List[str]]:
    """扫描给定路径，返回 (命中列表, 扫描过的文件列表, 读取失败的文件列表)。

    读取失败的文件被单独返回而不是静默跳过——否则"文件读不了"会变成
    一盏假绿灯（该文件里的禁用调用就永远查不出来）。
    """
    patterns = build_patterns(forbidden, allowed)
    files = iter_code_files(paths)
    hits: List[Dict[str, Any]] = []
    unreadable: List[str] = []
    for file_path in files:
        try:
            text = file_path.read_text(encoding="utf-8", errors="replace")
        except OSError as exc:
            unreadable.append(f"{file_path}（{exc}）")
            continue
        for hit in scan_text(text, patterns, allowed):
            record = dict(hit)
            record["file"] = str(file_path)
            hits.append(record)
    return hits, files, unreadable


def resolve_scan_paths(
    raw: Sequence[str], repo_root: Path = REPO_ROOT
) -> List[Path]:
    """把命令行给的路径解析为绝对路径（相对路径按 cwd 解释，其次按仓库根）。"""
    resolved: List[Path] = []
    for item in raw:
        candidate = Path(item)
        if candidate.is_absolute():
            resolved.append(candidate)
            continue
        from_cwd = (Path.cwd() / candidate).resolve()
        if from_cwd.exists():
            resolved.append(from_cwd)
            continue
        from_root = (repo_root / candidate).resolve()
        resolved.append(from_root)
    return resolved


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #
def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="厂商解算接口禁用名单静态扫描（名单来源：versions.lock:forbidden_apis）"
    )
    parser.add_argument(
        "paths",
        nargs="*",
        help=f"要扫描的路径（默认：{' '.join(DEFAULT_SCAN_DIRS)}）",
    )
    parser.add_argument(
        "--paths",
        nargs="+",
        dest="paths_opt",
        default=None,
        help="与位置参数等价的写法：显式指定要扫描的路径",
    )
    parser.add_argument(
        "--lock",
        type=Path,
        default=DEFAULT_LOCK,
        help="锁文件路径（默认 versions.lock）",
    )
    parser.add_argument("--json", action="store_true", help="以 JSON 输出结果")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(list(argv) if argv is not None else None)

    raw_paths: List[str] = list(args.paths)
    if args.paths_opt:
        raw_paths.extend(args.paths_opt)
    if not raw_paths:
        raw_paths = list(DEFAULT_SCAN_DIRS)

    try:
        forbidden, allowed = load_names(args.lock)
    except ScannerError as exc:
        print(f"scan_forbidden_apis: {exc}", file=sys.stderr)
        return 2

    scan_paths_resolved = resolve_scan_paths(raw_paths)
    missing = [str(p) for p in scan_paths_resolved if not p.exists()]
    if missing:
        print(
            "scan_forbidden_apis: 扫描路径不存在: " + ", ".join(missing),
            file=sys.stderr,
        )
        return 2

    hits, files, unreadable = scan_paths(scan_paths_resolved, forbidden, allowed)
    if unreadable:
        for item in unreadable:
            print(f"scan_forbidden_apis: 文件不可读: {item}", file=sys.stderr)
        return 2

    if args.json:
        print(
            json.dumps(
                {
                    "lock": str(args.lock),
                    "scanned_paths": [str(p) for p in scan_paths_resolved],
                    "scanned_files": len(files),
                    "forbidden_apis": forbidden,
                    "allowed_apis": allowed,
                    "hits": hits,
                    "ok": not hits,
                },
                ensure_ascii=False,
                indent=2,
            )
        )
    else:
        for hit in hits:
            print(f"{hit['file']}:{hit['line']}: {hit['content']}")
        if hits:
            print(
                f"scan_forbidden_apis: 命中 {len(hits)} 处禁用接口调用"
                f"（扫描 {len(files)} 个代码文件）"
            )
        else:
            print(
                f"scan_forbidden_apis: 未命中禁用接口"
                f"（扫描 {len(files)} 个代码文件，名单 {len(forbidden)} 项）"
            )
    return 1 if hits else 0


if __name__ == "__main__":
    sys.exit(main())
