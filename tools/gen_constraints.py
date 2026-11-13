#!/usr/bin/env python3
"""从 versions.lock 生成 pip constraints 文件（每行带 --hash=sha256:...）。

设计要点
--------
* 唯一真相源是仓库根的 ``versions.lock``；本脚本不硬编码任何版本号。
* 只访问 PyPI 的 JSON 元数据接口，**不下载任何 wheel/sdist 大文件**
  （真正的下载归档由 T03 的 ``tools/lock_deps.sh`` 负责）。
* 生成的 constraints 供 ``pip install --require-hashes -c constraints/py312.txt`` 使用，
  因此每个包的 sha256 必须与 PyPI 元数据逐字一致。

用法
----
    python3 tools/gen_constraints.py                # 生成/刷新两份 constraints
    python3 tools/gen_constraints.py --check        # 只校验现有文件是否与 lock 一致（CI 用）
    python3 tools/gen_constraints.py --offline      # 只用本地缓存（无网络时）

退出码：0 成功 / 1 校验失败（--check 发现漂移）/ 2 运行期错误（网络、格式等）。
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any, Dict, Iterable, List, Sequence, Tuple

try:  # PyYAML 是硬依赖，但错误信息要友好
    import yaml
except ImportError:  # pragma: no cover - 环境缺依赖时的显式失败
    print("gen_constraints: 需要 PyYAML（pip install pyyaml）", file=sys.stderr)
    raise

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_LOCK = REPO_ROOT / "versions.lock"
DEFAULT_OUT_DIR = REPO_ROOT / "constraints"
DEFAULT_CACHE_DIR = REPO_ROOT / "vendor" / ".pypi-cache"
PYPI_JSON = "https://pypi.org/pypi/{name}/{version}/json"

REQUIRED_TOP_LEVEL_KEYS = (
    "schema_version",
    "baseline",
    "fallback",
    "packages",
    "upstream",
    "platform",
    "forbidden_apis",
    "allowed_apis",
    "wheels",
)

# 锁文件里的键名 → PyPI 项目名（PyPI 自身大小写不敏感，这里只为可读性）
PYPI_NAME_OVERRIDES = {
    "mosek": "mosek",
    "pyyaml": "PyYAML",
}


class LockError(ValueError):
    """versions.lock 结构非法。"""


class FetchError(RuntimeError):
    """PyPI 元数据获取失败。"""


# --------------------------------------------------------------------------- #
# 锁文件读取与校验
# --------------------------------------------------------------------------- #
def load_lock(path: Path) -> Dict[str, Any]:
    """读取并解析 versions.lock（YAML）。"""
    if not path.is_file():
        raise LockError(f"找不到锁文件: {path}")
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:  # pragma: no cover - 依赖具体 yaml 版本
        raise LockError(f"锁文件 YAML 解析失败: {exc}") from exc
    if not isinstance(data, dict):
        raise LockError("锁文件顶层必须是映射（dict）")
    return data


def validate_lock(lock: Dict[str, Any]) -> None:
    """校验锁文件必填字段；不合法时抛出 :class:`LockError`。"""
    missing = [k for k in REQUIRED_TOP_LEVEL_KEYS if k not in lock]
    if missing:
        raise LockError(f"锁文件缺少必填字段: {', '.join(missing)}")

    for section in ("baseline", "fallback"):
        block = lock.get(section)
        if not isinstance(block, dict):
            raise LockError(f"锁文件字段 {section} 必须是映射")
        for key in ("profile", "python", "python_tag", "drake"):
            if not block.get(key):
                raise LockError(f"锁文件字段 {section}.{key} 缺失或为空")
        if not isinstance(block.get("supported", False), bool):
            raise LockError(f"锁文件字段 {section}.supported 必须是布尔值")

    packages = lock.get("packages")
    if not isinstance(packages, dict) or not packages:
        raise LockError("锁文件字段 packages 必须是非空映射")
    for name, spec in packages.items():
        if not isinstance(spec, dict):
            raise LockError(f"packages.{name} 必须是映射")
        if not spec.get("version") and not (spec.get("baseline") and spec.get("fallback")):
            raise LockError(f"packages.{name} 缺少 version（或 baseline+fallback）")

    upstream = lock.get("upstream")
    if not isinstance(upstream, dict) or not upstream:
        raise LockError("锁文件字段 upstream 必须是非空映射")
    for name, spec in upstream.items():
        commit = (spec or {}).get("commit", "")
        if not (isinstance(commit, str) and len(commit) == 40 and all(c in "0123456789abcdef" for c in commit)):
            raise LockError(f"upstream.{name}.commit 必须是 40 位十六进制 sha")

    for key in ("forbidden_apis", "allowed_apis"):
        value = lock.get(key)
        if not isinstance(value, list) or not value:
            raise LockError(f"锁文件字段 {key} 必须是非空列表")

    if not isinstance(lock.get("wheels"), dict):
        raise LockError("锁文件字段 wheels 必须是映射（文件名 → sha256）")


def profile_names(lock: Dict[str, Any]) -> List[str]:
    """返回可用 profile 名（baseline / fallback）。"""
    return [name for name in ("baseline", "fallback") if name in lock]


def resolve_packages(lock: Dict[str, Any], profile: str) -> Dict[str, str]:
    """给出某 profile 下每个包的最终版本号（name → version）。

    * Drake 的版本来自 ``baseline.drake`` / ``fallback.drake``；
    * 其它包来自 ``packages.<name>.version``，若条目带 profile 专属覆盖则以覆盖为准。
    """
    if profile not in profile_names(lock):
        raise LockError(f"未知 profile: {profile}")
    resolved: Dict[str, str] = {"drake": str(lock[profile]["drake"])}
    for name, spec in lock["packages"].items():
        version = spec.get(profile, spec.get("version"))
        if not version:
            raise LockError(f"packages.{name} 缺少 {profile} 档版本")
        resolved[name] = str(version)
    return resolved


# --------------------------------------------------------------------------- #
# PyPI 元数据
# --------------------------------------------------------------------------- #
def _cache_path(cache_dir: Path, name: str, version: str) -> Path:
    return cache_dir / name / f"{version}.json"


def fetch_release(
    name: str,
    version: str,
    cache_dir: Path = DEFAULT_CACHE_DIR,
    offline: bool = False,
    timeout: float = 30.0,
    retries: int = 3,
) -> Dict[str, Any]:
    """取某个包某个版本的 PyPI JSON 元数据（带本地缓存与重试）。"""
    cache_file = _cache_path(cache_dir, name, version)
    if cache_file.is_file():
        try:
            return json.loads(cache_file.read_text(encoding="utf-8"))
        except json.JSONDecodeError:
            cache_file.unlink(missing_ok=True)  # 缓存损坏则重新抓取

    if offline:
        raise FetchError(f"离线模式且无缓存: {name} {version}（缓存路径 {cache_file}）")

    url = PYPI_JSON.format(name=PYPI_NAME_OVERRIDES.get(name.lower(), name), version=version)
    last_error: Exception | None = None
    for attempt in range(1, retries + 1):
        try:
            request = urllib.request.Request(url, headers={"User-Agent": "rm75-drake-lockgen/1.0"})
            with urllib.request.urlopen(request, timeout=timeout) as response:
                payload = response.read()
            data = json.loads(payload.decode("utf-8"))
            cache_file.parent.mkdir(parents=True, exist_ok=True)
            cache_file.write_text(json.dumps(data), encoding="utf-8")
            return data
        except urllib.error.HTTPError as exc:
            last_error = exc
            if exc.code == 404:
                break  # 版本不存在，重试无意义
        except (urllib.error.URLError, TimeoutError, OSError, json.JSONDecodeError) as exc:
            last_error = exc
        if attempt < retries:
            time.sleep(min(2 ** attempt, 8))
    raise FetchError(f"获取 PyPI 元数据失败: {name} {version} → {url}（{last_error}）")


def _wheel_python_tag(filename: str) -> Tuple[str, str]:
    """从 wheel 文件名解析 (python tag, abi tag)。"""
    stem = filename[:-4] if filename.endswith(".whl") else filename
    parts = stem.split("-")
    if len(parts) < 5:
        return ("", "")
    return (parts[-3], parts[-2])


def _python_tag_compatible(python_tag: str, abi_tag: str, target: str) -> bool:
    """判断 wheel 的 Python tag 是否与目标解释器兼容。"""
    if python_tag in ("py3", "py2.py3", "py2"):
        return True
    if abi_tag == "abi3":
        # 例：cp39-abi3 —— 3.9 及以上的 CPython 都可安装
        digits = python_tag[2:] if python_tag.startswith("cp") else ""
        if not digits.isdigit():
            return False
        return int(target[2:]) >= int(digits)
    return python_tag == target


def _platform_compatible(platform_tag: str) -> bool:
    """只保留 Linux x86_64（manylinux）与纯 Python 发行文件。"""
    if platform_tag.endswith("any"):
        return True
    if "musllinux" in platform_tag or "aarch64" in platform_tag:
        return False
    if "manylinux" in platform_tag or "linux_x86_64" in platform_tag:
        return "x86_64" in platform_tag
    return False


def select_files(files: Sequence[Dict[str, Any]], python_tag: str) -> List[Dict[str, Any]]:
    """从 PyPI 元数据的 files 中筛出与 (linux x86_64, 目标 python) 兼容的发行文件。"""
    selected: List[Dict[str, Any]] = []
    for entry in files:
        packagetype = entry.get("packagetype")
        filename = entry.get("filename", "")
        digest = (entry.get("digests") or {}).get("sha256")
        if not filename or not digest:
            continue
        if packagetype == "sdist":
            selected.append(entry)
            continue
        if packagetype != "bdist_wheel":
            continue
        wheel_py, wheel_abi = _wheel_python_tag(filename)
        wheel_plat = filename[:-4].split("-")[-1]
        if not _python_tag_compatible(wheel_py, wheel_abi, python_tag):
            continue
        if not _platform_compatible(wheel_plat):
            continue
        selected.append(entry)
    return sorted(selected, key=lambda e: e["filename"])


# --------------------------------------------------------------------------- #
# 渲染
# --------------------------------------------------------------------------- #
def render_constraints(
    lock: Dict[str, Any],
    profile: str,
    files_by_package: Dict[str, List[Dict[str, Any]]],
) -> str:
    """渲染一份 constraints 文本。"""
    block = lock[profile]
    header = [
        f"# constraints/{block['profile']}.txt",
        "#",
        "# 由 tools/gen_constraints.py 从 versions.lock 生成 —— 请勿手工编辑。",
        f"# profile      : {block['profile']}（{profile}）",
        f"# python       : {block['python']}（tag {block['python_tag']}）",
        f"# drake        : {block['drake']}",
        f"# supported    : {bool(block.get('supported', False))}",
    ]
    if not block.get("supported", False):
        header += [
            "#",
            "# !! 非受支持基线，仅 fast 档 !!",
            "# 本档仅用于本机 Python 3.10 的快速自检，正式交付以 baseline 为准。",
        ]
    header += [
        "#",
        "# 每个包列出该版本在 PyPI 上与 linux/x86_64 + 当前 Python tag 兼容的所有发行文件",
        "# 的 sha256，供 pip install --require-hashes 校验。",
        "# 重新生成： python3 tools/gen_constraints.py",
        "",
    ]
    lines: List[str] = []
    for name, version in sorted(resolve_packages(lock, profile).items()):
        entries = files_by_package.get(name, [])
        if not entries:
            raise FetchError(f"{name} {version}：没有可用的发行文件（sha256 缺失）")
        hashes = " ".join(f"--hash=sha256:{(e['digests']['sha256'])}" for e in entries)
        lines.append(f"{name}=={version} {hashes}")
    return "\n".join(header + lines) + "\n"


def generate(
    lock: Dict[str, Any],
    profile: str,
    out_dir: Path,
    cache_dir: Path = DEFAULT_CACHE_DIR,
    offline: bool = False,
) -> Path:
    """生成单个 profile 的 constraints 文件，返回写入路径。"""
    block = lock[profile]
    packages = resolve_packages(lock, profile)
    files_by_package: Dict[str, List[Dict[str, Any]]] = {}
    for name, version in packages.items():
        meta = fetch_release(name, version, cache_dir=cache_dir, offline=offline)
        entries = select_files(meta.get("urls", []), block["python_tag"])
        if not entries:
            raise FetchError(
                f"{name} {version}：没有与 {block['python_tag']} + linux/x86_64 兼容的发行文件"
            )
        files_by_package[name] = entries
    out_dir.mkdir(parents=True, exist_ok=True)
    target = out_dir / f"{block['profile']}.txt"
    target.write_text(render_constraints(lock, profile, files_by_package), encoding="utf-8")
    return target


def check(
    lock: Dict[str, Any],
    profile: str,
    out_dir: Path,
    cache_dir: Path = DEFAULT_CACHE_DIR,
    offline: bool = False,
) -> List[str]:
    """校验现有 constraints 文件与 lock 一致；返回差异描述列表（空表示一致）。"""
    block = lock[profile]
    target = out_dir / f"{block['profile']}.txt"
    if not target.is_file():
        return [f"{target} 不存在"]
    packages = resolve_packages(lock, profile)
    files_by_package: Dict[str, List[Dict[str, Any]]] = {}
    for name, version in packages.items():
        meta = fetch_release(name, version, cache_dir=cache_dir, offline=offline)
        files_by_package[name] = select_files(meta.get("urls", []), block["python_tag"])
    expected = render_constraints(lock, profile, files_by_package)
    actual = target.read_text(encoding="utf-8")
    if expected == actual:
        return []
    return [f"{target} 与 versions.lock / PyPI 元数据不一致（需重新生成）"]


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #
def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="从 versions.lock 生成 pip constraints（含 sha256）")
    parser.add_argument("--lock", type=Path, default=DEFAULT_LOCK, help="锁文件路径（默认 versions.lock）")
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR, help="constraints 输出目录")
    parser.add_argument("--cache-dir", type=Path, default=DEFAULT_CACHE_DIR, help="PyPI 元数据缓存目录")
    parser.add_argument("--profiles", nargs="*", default=None, help="只处理指定 profile（baseline/fallback）")
    parser.add_argument("--offline", action="store_true", help="只用本地缓存，不访问网络")
    parser.add_argument("--check", action="store_true", help="只校验现有文件是否最新（漂移则退出码 1）")
    args = parser.parse_args(list(argv) if argv is not None else None)

    try:
        lock = load_lock(args.lock)
        validate_lock(lock)
    except LockError as exc:
        print(f"gen_constraints: 锁文件非法: {exc}", file=sys.stderr)
        return 2

    profiles = args.profiles or profile_names(lock)
    unknown = [p for p in profiles if p not in profile_names(lock)]
    if unknown:
        print(f"gen_constraints: 未知 profile {unknown}", file=sys.stderr)
        return 2

    try:
        if args.check:
            drift: List[str] = []
            for profile in profiles:
                drift.extend(check(lock, profile, args.out_dir, args.cache_dir, args.offline))
            if drift:
                for line in drift:
                    print(f"gen_constraints: {line}", file=sys.stderr)
                return 1
            print("gen_constraints: constraints 与 versions.lock / PyPI 元数据一致")
            return 0

        for profile in profiles:
            path = generate(lock, profile, args.out_dir, args.cache_dir, args.offline)
            print(f"gen_constraints: 已生成 {path}")
        return 0
    except (FetchError, LockError) as exc:
        print(f"gen_constraints: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
