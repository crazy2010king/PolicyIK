#!/usr/bin/env python3
"""tools/fetch_upstream.py —— 上游资产 commit 锁定与离线归档（rm_models / Two-Finger-Gripper）

设计要点
--------
1. **唯一真相源**：repo / ref / subset / commit 全部从 ``versions.lock:upstream`` 读取，
   脚本内不硬编码第二份（与 scripts/scan_forbidden_apis.py 同一条约定）。
   许可名从 ``licenses/registry.yaml`` 读取。
2. **只归档子集**：整仓归档不可接受（rm_models zip 248 MB、Two-Finger-Gripper 44 MB），
   因此 rm_models 只取 ``RM75/urdf/RM75-B/``（URDF + STL + csv + launch），
   Two-Finger-Gripper 只取元数据（仓库文件清单 + 许可文件），不做大体积二进制归档。
3. **可复现**：归档一律由 ``git archive --format=tar.gz`` 产生（mtime 取 commit 时间、
   无 gzip 时间戳），同一 commit 反复抓取得到**字节一致**的归档 → sha256 可写进
   versions.lock 并长期有效。
4. **二进制不入 git**：归档落在 ``third_party/archives/``（已 .gitignore），
   入 git 的只是 commit + sha256 清单（third_party/MANIFEST.yaml）。

子命令
------
  fetch（默认）  解析 commit → 稀疏拉取 → 产归档 → 写 MANIFEST.yaml → 回写 versions.lock
  --verify       按 MANIFEST 重算 sha256 比对；缺失/不匹配即非零退出
  --list-remote  只做 git ls-remote（打印各资产 ref → commit）

退出码：0 成功；1 校验失败/归档缺失；2 环境或参数错误（git 缺失、远端不可达等）。
"""

from __future__ import annotations

import argparse
import hashlib
import os
import re
import shutil
import subprocess
import sys
import tarfile
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_LOCK = REPO_ROOT / "versions.lock"
DEFAULT_LICENSES = REPO_ROOT / "licenses" / "registry.yaml"
DEFAULT_ARCHIVE_DIR = REPO_ROOT / "third_party" / "archives"
DEFAULT_MANIFEST = REPO_ROOT / "third_party" / "MANIFEST.yaml"
DEFAULT_WORKDIR = REPO_ROOT / "third_party" / ".work"

HEX40 = re.compile(r"^[0-9a-f]{40}$")
HEX64 = re.compile(r"^[0-9a-f]{64}$")

EXIT_OK = 0
EXIT_FAIL = 1
EXIT_ENV = 2


# ── 小工具 ────────────────────────────────────────────────────────────────
def log(msg: str) -> None:
    print(f"[fetch_upstream] {msg}", flush=True)


def die(msg: str, code: int = EXIT_ENV) -> "NoReturn":  # type: ignore[name-defined]
    print(f"[fetch_upstream][ERROR] {msg}", file=sys.stderr, flush=True)
    raise SystemExit(code)


def rel_to_repo(path: Path) -> str:
    """把路径渲染成「相对仓库根」的短字符串。

    仓库内路径 → ``third_party/MANIFEST.yaml``；
    仓库外路径（如 ``--manifest /tmp/xxx.yaml``）→ ``../tmp/xxx.yaml`` 形式的相对路径，
    **不得**因为 ``Path.relative_to`` 抛 ValueError 而让整条命令失败。
    """
    try:
        return str(Path(path).resolve().relative_to(REPO_ROOT))
    except ValueError:
        return os.path.relpath(str(Path(path).resolve()), str(REPO_ROOT))


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def run_git(args, cwd=None, check=True, capture=True, timeout=3600):
    """执行 git 命令；返回 CompletedProcess。失败时抛 SystemExit(2)。"""
    cmd = ["git"] + list(args)
    try:
        proc = subprocess.run(
            cmd,
            cwd=str(cwd) if cwd else None,
            stdout=subprocess.PIPE if capture else None,
            stderr=subprocess.PIPE if capture else None,
            text=True,
            timeout=timeout,
        )
    except FileNotFoundError:
        die("未找到 git 可执行文件；本工具依赖 git（稀疏拉取 + git archive）")
    except subprocess.TimeoutExpired:
        die(f"git 命令超时（{timeout}s）：{' '.join(cmd)}")
    if check and proc.returncode != 0:
        stderr = (proc.stderr or "").strip()
        die(f"git 命令失败（exit={proc.returncode}）：{' '.join(cmd)}\n{stderr}")
    return proc


def load_yaml(path: Path) -> dict:
    try:
        import yaml  # type: ignore
    except ImportError:  # pragma: no cover
        die("缺少 PyYAML；请用 `pip install pyyaml` 或走 constraints 安装")
    with path.open("r", encoding="utf-8") as fh:
        return yaml.safe_load(fh) or {}


def dump_yaml(path: Path, data: dict) -> None:
    import yaml  # type: ignore

    path.parent.mkdir(parents=True, exist_ok=True)
    header = (
        "# third_party/MANIFEST.yaml —— 上游资产离线归档清单（由 tools/fetch_upstream.py 生成）\n"
        "#\n"
        "# 入 git 的只有这份清单（commit + sha256）。归档本体在 third_party/archives/ 下，\n"
        "# 因上游许可（rm_models 未明示 / 官方网格）限制再分发，已被 .gitignore 排除。\n"
        "# 复现：`python3 tools/fetch_upstream.py`；校验：`python3 tools/fetch_upstream.py --verify`\n"
        "#\n"
        "# 字段：name / repo / ref / commit / subset / mode / license / redistributable /\n"
        "#       archive / sha256 / file_count / total_bytes / fetched_at\n"
    )
    with path.open("w", encoding="utf-8") as fh:
        fh.write(header)
        yaml.safe_dump(data, fh, allow_unicode=True, sort_keys=False, default_flow_style=False)


# ── 资产定义（来自 versions.lock / licenses/registry.yaml）─────────────────
def _norm_license_key(name: str) -> str:
    """归一化许可登记表的资产名：Two-Finger-Gripper ↔ two_finger_gripper。"""
    return "".join(ch for ch in str(name).lower() if ch.isalnum())


def collect_assets(lock: dict, licenses: dict, ref_override: str | None,
                   only: list[str] | None) -> list[dict]:
    upstream = lock.get("upstream")
    if not isinstance(upstream, dict) or not upstream:
        die("versions.lock:upstream 缺失或为空，无法锁定上游资产")
    lic_by_name = {}
    for entry in (licenses.get("entries") or []):
        if isinstance(entry, dict) and entry.get("name"):
            lic_by_name[_norm_license_key(entry["name"])] = entry

    assets = []
    for name, spec in upstream.items():
        if not isinstance(spec, dict):
            die(f"versions.lock:upstream.{name} 不是映射")
        if only and name not in only:
            continue
        repo = spec.get("repo")
        subset = spec.get("subset")
        if not repo or not subset:
            die(f"versions.lock:upstream.{name} 缺少 repo/subset")
        ref = ref_override or spec.get("ref") or "main"
        lic = lic_by_name.get(_norm_license_key(name)) or {}
        assets.append(
            {
                "name": name,
                "repo": str(repo),
                "ref": str(ref),
                "commit": str(spec.get("commit") or ""),
                "subset": str(subset),
                "mode": "metadata" if str(subset) == "metadata-only" else "subset",
                "license": lic.get("license", "未明示"),
                "redistributable": bool(lic.get("redistributable", False)),
                "source_name": lic.get("name", name),
            }
        )
    if not assets:
        die("没有可处理的资产（--asset 过滤后为空）")
    return assets


def resolve_commit(asset: dict) -> str:
    """用 git ls-remote 解析 ref → 40 位 hex commit。"""
    ref = asset["ref"]
    if HEX40.match(ref):
        return ref
    proc = run_git(["ls-remote", asset["repo"], ref], timeout=180)
    for line in proc.stdout.splitlines():
        parts = line.split()
        if len(parts) == 2 and HEX40.match(parts[0]):
            return parts[0]
    die(f"无法解析 {asset['name']} 的 ref={ref!r}（ls-remote 无 40 位 hex 结果）")


def clone_partial(asset: dict, workdir: Path) -> Path:
    """浅克隆 + blob:none 过滤（不 checkout，稀疏内容用 git archive 取）。"""
    repo_dir = workdir / asset["name"]
    reuse = repo_dir.exists() and run_git(
        ["rev-parse", "--git-dir"], cwd=repo_dir, check=False
    ).returncode == 0
    if reuse:
        log(f"{asset['name']}: 复用已有浅克隆 {repo_dir}")
    else:
        if repo_dir.exists():
            shutil.rmtree(repo_dir)
        repo_dir.parent.mkdir(parents=True, exist_ok=True)
        log(f"{asset['name']}: 浅克隆 {asset['repo']}（--filter=blob:none --depth 1）")
        run_git(
            [
                "clone",
                "--filter=blob:none",
                "--no-checkout",
                "--depth",
                "1",
                asset["repo"],
                str(repo_dir),
            ],
            timeout=1800,
        )
    return repo_dir


def ensure_commit(repo_dir: Path, commit: str) -> None:
    """确保目标 commit 在浅克隆里（ref 指向非默认分支/标签时按需 fetch）。"""
    proc = run_git(["cat-file", "-e", f"{commit}^{{commit}}"], cwd=repo_dir, check=False)
    if proc.returncode == 0:
        return
    log(f"浅克隆中无 {commit[:12]}，按需 fetch")
    run_git(["fetch", "--depth", "1", "origin", commit], cwd=repo_dir, timeout=1800)


def ls_tree_long(
    repo_dir: Path, commit: str, pathspec: str | None = None
) -> list[tuple[str, int, str]]:
    """列出树条目：(路径, 大小, blob sha)。

    大小为 0 **可能**意味着“大小未知”（部分克隆里 blob 尚未取回，见
    :func:`resolve_blob_sizes`），不一定是空文件。
    """
    args = ["ls-tree", "-r", "--long", commit]
    if pathspec:
        args += ["--", pathspec]
    proc = run_git(args, cwd=repo_dir)
    entries = []
    for line in proc.stdout.splitlines():
        parts = line.split(None, 4)
        if len(parts) < 5:
            continue
        try:
            size = int(parts[3])
        except ValueError:
            size = 0
        entries.append((parts[4], size, parts[2]))
    return entries


def resolve_blob_sizes(repo_dir: Path, shas: list[str], retries: int = 3) -> dict[str, int]:
    """解析一批 blob 的真实字节数。

    部分克隆（``--filter=blob:none``）下 ``git ls-tree --long`` 对尚未取回的 blob
    会报 **大小未知**（本实现里落成 0）。若把这个 0 写进元数据归档，归档字节就
    取决于“当时哪些 blob 恰好在本地”，**同一 commit 反复抓取 sha256 不一致**，
    写进 versions.lock 的 sha256 也就失去可复现性。

    因此：大小未知的 blob 一律用 ``git cat-file --batch-check`` 取真实大小
    （该命令会触发 promisor 懒加载）；仍取不到就 **硬失败**，绝不静默写 0。
    """
    if not shas:
        return {}
    sizes: dict[str, int] = {}
    pending = list(dict.fromkeys(shas))
    for attempt in range(1, retries + 1):
        proc = subprocess.run(
            ["git", "cat-file", "--batch-check"],
            cwd=str(repo_dir),
            input="\n".join(pending) + "\n",
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=1800,
        )
        still_missing: list[str] = []
        for line in proc.stdout.splitlines():
            parts = line.split()
            if len(parts) < 2:
                continue
            sha, rest = parts[0], parts[1:]
            if rest and rest[0] == "missing":
                still_missing.append(sha)
                continue
            if len(rest) < 2:
                still_missing.append(sha)
                continue
            kind, size = rest[0], rest[1]
            if kind == "missing" or size == "-":
                still_missing.append(sha)
                continue
            try:
                sizes[sha] = int(size)
            except ValueError:
                still_missing.append(sha)
        pending = still_missing
        if not pending:
            return sizes
        log(
            f"blob 大小未取回（尝试 {attempt}/{retries}）：{len(pending)} 个，"
            "5 秒后重试（网络抖动）"
        )
        time.sleep(5)
    die(
        f"无法取得 {len(pending)} 个 blob 的真实大小（{pending[:3]}…）。"
        "归档将不可复现（sha256 会随本地 blob 状态漂移），因此中止。"
        "可先在该浅克隆里执行 `git fetch --refetch --filter=blob:none` 后重试。"
    )
    return {}  # pragma: no cover - die() 必然抛错


def build_subset_archive(repo_dir: Path, commit: str, subset: str, out_path: Path) -> tuple[int, int]:
    """用 git archive 产生子集归档（可复现）。返回 (文件数, 未压缩总字节)。"""
    entries = ls_tree_long(repo_dir, commit, subset)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    tmp = out_path.with_suffix(out_path.suffix + ".part")
    with tmp.open("wb") as fh:
        proc = subprocess.run(
            ["git", "archive", "--format=tar.gz", commit, "--", subset],
            cwd=str(repo_dir),
            stdout=fh,
            stderr=subprocess.PIPE,
            timeout=3600,
        )
    if proc.returncode != 0:
        tmp.unlink(missing_ok=True)
        die(f"git archive 失败：{(proc.stderr or b'').decode(errors='replace').strip()}")
    tmp.replace(out_path)
    return len(entries), sum(size for _, size, _ in entries)


def build_metadata_archive(repo_dir: Path, commit: str, out_path: Path) -> tuple[int, int]:
    """元数据归档：仓库全量文件清单（ls-tree --long）+ 许可文件。不含大体积二进制。"""
    entries = ls_tree_long(repo_dir, commit)
    # 大小未知的 blob 必须取回真实大小，否则归档字节不可复现（见 resolve_blob_sizes）。
    unknown = [(path, sha) for path, size, sha in entries if size == 0]
    if unknown:
        resolved = resolve_blob_sizes(repo_dir, [sha for _, sha in unknown])
        entries = [
            (path, resolved.get(sha, size), sha) for path, size, sha in entries
        ]
    listing = "".join(f"{size:>12}  {path}\n" for path, size, _ in entries)
    files_txt = (
        f"# Two-Finger-Gripper 元数据清单（由 tools/fetch_upstream.py 生成）\n"
        f"# commit: {commit}\n"
        f"# 文件总数: {len(entries)}\n"
        f"# 仅记录文件名与字节大小，不归档二进制内容。\n"
        f"{listing}"
    ).encode("utf-8")

    lic_proc = run_git(["show", f"{commit}:License"], cwd=repo_dir, check=False)
    license_bytes = lic_proc.stdout.encode("utf-8") if lic_proc.returncode == 0 else b""
    if not license_bytes:
        log("警告：未找到 License 文件，元数据归档将不含许可正文")

    # 固定 mtime（commit 时间）+ gzip 无时间戳 → 可复现
    commit_time = int(
        run_git(["show", "-s", "--format=%ct", commit], cwd=repo_dir).stdout.strip() or 0
    )
    out_path.parent.mkdir(parents=True, exist_ok=True)
    tmp = out_path.with_suffix(out_path.suffix + ".part")
    import gzip

    with tmp.open("wb") as raw:
        with gzip.GzipFile(filename="", mode="wb", fileobj=raw, mtime=0) as gz:
            with tarfile.open(fileobj=gz, mode="w", format=tarfile.PAX_FORMAT) as tar:
                for name, payload in (
                    ("FILES.txt", files_txt),
                    ("License", license_bytes),
                ):
                    if not payload:
                        continue
                    info = tarfile.TarInfo(name)
                    info.size = len(payload)
                    info.mtime = commit_time
                    info.mode = 0o644
                    info.uid = info.gid = 0
                    info.uname = info.gname = ""
                    tar.addfile(info, __import__("io").BytesIO(payload))
    tmp.replace(out_path)
    return len(entries) + 1, len(files_txt) + len(license_bytes)


# ── versions.lock 回写（行级编辑，保留注释与排版）────────────────────────
def update_lock(lock_path: Path, results: list[dict]) -> None:
    lines = lock_path.read_text(encoding="utf-8").splitlines()
    try:
        start = next(i for i, ln in enumerate(lines) if ln.rstrip() == "upstream:")
    except StopIteration:
        die("versions.lock 中找不到 upstream: 段")

    end = len(lines)
    for i in range(start + 1, len(lines)):
        ln = lines[i]
        if ln.strip() and not ln.startswith((" ", "#")):
            end = i
            break

    for res in results:
        name = res["name"]
        # 定位该资产块
        try:
            bstart = next(
                i for i in range(start, end) if lines[i].startswith(f"  {name}:")
            )
        except StopIteration:
            die(f"versions.lock:upstream 中找不到资产 {name}")
        bend = end
        for i in range(bstart + 1, end):
            ln = lines[i]
            if ln.startswith("  ") and not ln.startswith("   ") and ln.strip():
                bend = i
                break
        block = lines[bstart:bend]

        # 移除旧的 archive: 子块
        new_block = []
        i = 0
        while i < len(block):
            ln = block[i]
            if ln.strip() == "archive:":
                i += 1
                while i < len(block) and (block[i].startswith("      ") or not block[i].strip()):
                    i += 1
                continue
            new_block.append(ln)
            i += 1
        block = new_block

        # 更新 commit（若上游已变）
        for i, ln in enumerate(block):
            if ln.strip().startswith("commit:"):
                block[i] = f'    commit: "{res["commit"]}"'
                break

        archive_lines = [
            "    archive:",
            f'      filename: {res["archive"]}',
            f'      sha256: {res["sha256"]}',
            f'      file_count: {res["file_count"]}',
            f'      total_bytes: {res["total_bytes"]}',
        ]
        # 插到 note: 之前（保持 note 在块尾），没有 note 则放块尾
        insert_at = len(block)
        for i, ln in enumerate(block):
            if ln.strip().startswith("note:"):
                insert_at = i
                break
        block = block[:insert_at] + archive_lines + block[insert_at:]
        lines[bstart:bend] = block
        # 行号发生偏移，后续资产需重新扫描
        start = next(i for i, ln in enumerate(lines) if ln.rstrip() == "upstream:")
        end = len(lines)
        for i in range(start + 1, len(lines)):
            ln = lines[i]
            if ln.strip() and not ln.startswith((" ", "#")):
                end = i
                break

    lock_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


# ── fetch / verify ───────────────────────────────────────────────────────
def cmd_fetch(args) -> int:
    lock_path = Path(args.lock)
    licenses_path = Path(args.licenses)
    lock = load_yaml(lock_path)
    licenses = load_yaml(licenses_path) if licenses_path.exists() else {"entries": []}
    assets = collect_assets(lock, licenses, args.ref, args.asset)

    if args.list_remote:
        for asset in assets:
            sha = resolve_commit(asset)
            print(f"{asset['name']}\t{asset['ref']}\t{sha}")
        return EXIT_OK

    archive_dir = Path(args.archive_dir)
    manifest_path = Path(args.manifest)
    workdir = Path(args.workdir)

    results = []
    for asset in assets:
        t0 = time.time()
        log(f"{asset['name']}: 解析 ref={asset['ref']}")
        commit = resolve_commit(asset)
        log(f"{asset['name']}: commit={commit}")

        repo_dir = clone_partial(asset, workdir)
        ensure_commit(repo_dir, commit)
        short = commit[:12]
        if asset["mode"] == "subset":
            filename = f"{asset['name']}-{short}.tar.gz"
        else:
            filename = f"{asset['name']}-{short}-metadata.tar.gz"
        out_path = archive_dir / filename

        if asset["mode"] == "subset":
            file_count, total_bytes = build_subset_archive(
                repo_dir, commit, asset["subset"], out_path
            )
        else:
            file_count, total_bytes = build_metadata_archive(
                repo_dir, commit, out_path
            )
        sha = sha256_file(out_path)
        elapsed = time.time() - t0
        log(
            f"{asset['name']}: 归档 {filename} "
            f"({file_count} 文件 / {total_bytes} B / sha256={sha[:12]}… / {elapsed:.1f}s)"
        )

        results.append(
            {
                "name": asset["name"],
                "repo": asset["repo"],
                "ref": asset["ref"],
                "commit": commit,
                "subset": asset["subset"],
                "mode": asset["mode"],
                "license": asset["license"],
                "redistributable": asset["redistributable"],
                "archive": filename,
                "sha256": sha,
                "file_count": file_count,
                "total_bytes": total_bytes,
                "fetched_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            }
        )

    manifest = {
        "schema_version": 1,
        "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "assets": results,
    }
    dump_yaml(manifest_path, manifest)
    log(f"已写清单 {rel_to_repo(manifest_path)}")

    if not args.no_lock_update:
        update_lock(lock_path, results)
        log(f"已回写 {rel_to_repo(lock_path)}:upstream.*.archive")

    # 立即自校验
    ok = verify(manifest_path, archive_dir, lock_path, quiet=False)
    return EXIT_OK if ok else EXIT_FAIL


def verify(manifest_path: Path, archive_dir: Path, lock_path: Path, quiet: bool = False) -> bool:
    problems: list[str] = []
    if not manifest_path.exists():
        print(
            f"[fetch_upstream][ERROR] 清单缺失：{manifest_path}（先运行 `python3 tools/fetch_upstream.py`）",
            file=sys.stderr,
        )
        return False
    manifest = load_yaml(manifest_path)
    assets = manifest.get("assets") or []
    if not assets:
        print("[fetch_upstream][ERROR] 清单里没有任何资产条目", file=sys.stderr)
        return False

    lock = load_yaml(lock_path) if lock_path and Path(lock_path).exists() else {}
    lock_upstream = lock.get("upstream") or {}

    for asset in assets:
        name = asset.get("name", "?")
        expected = str(asset.get("sha256") or "")
        filename = asset.get("archive")
        if not filename:
            problems.append(f"{name}: 清单缺少 archive 文件名")
            continue
        if not expected:
            problems.append(f"{name}: 清单缺少 sha256")
            continue
        if not HEX64.match(expected):
            problems.append(f"{name}: 清单 sha256 格式错误: {expected}")
            continue
        path = Path(archive_dir) / filename
        if not path.exists():
            problems.append(
                f"{name}: 归档缺失 {path}（二进制归档不入 git，需在本机重建）"
            )
            continue
        actual = sha256_file(path)
        if actual != expected:
            problems.append(f"{name}: sha256 不匹配 期望={expected} 实际={actual}")
            continue
        lspec = lock_upstream.get(name)
        if isinstance(lspec, dict):
            if str(lspec.get("commit")) != str(asset.get("commit")):
                problems.append(
                    f"{name}: versions.lock commit({lspec.get('commit')}) 与清单 commit({asset.get('commit')}) 不一致"
                )
            larch = lspec.get("archive") or {}
            if isinstance(larch, dict) and larch.get("sha256") and larch["sha256"] != expected:
                problems.append(
                    f"{name}: versions.lock archive.sha256({larch['sha256']}) 与清单 sha256({expected}) 不一致"
                )
        if not quiet:
            print(f"[fetch_upstream] OK  {name}: {filename} sha256={actual}")

    if problems:
        for p in problems:
            print(f"[fetch_upstream][ERROR] {p}", file=sys.stderr)
        return False
    if not quiet:
        print(f"[fetch_upstream] 校验通过：{len(assets)} 个资产")
    return True


def cmd_verify(args) -> int:
    ok = verify(Path(args.manifest), Path(args.archive_dir), Path(args.lock))
    return EXIT_OK if ok else EXIT_FAIL


# ── CLI ───────────────────────────────────────────────────────────────────
def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="fetch_upstream.py",
        description="上游资产 commit 锁定与离线归档（唯一真相源 = versions.lock:upstream）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "示例：\n"
            "  python3 tools/fetch_upstream.py                 # 抓取全部资产并回写 versions.lock\n"
            "  python3 tools/fetch_upstream.py --verify        # 按 MANIFEST 重算 sha256\n"
            "  python3 tools/fetch_upstream.py --list-remote   # 只看 ref → commit\n"
            "  python3 tools/fetch_upstream.py --asset rm_models --ref v1.0\n"
        ),
    )
    p.add_argument("--verify", action="store_true", help="按 MANIFEST 校验归档 sha256")
    p.add_argument("--list-remote", action="store_true", help="只解析 ref → commit")
    p.add_argument("--ref", default=None, help="覆盖 versions.lock 中的 ref（分支/标签/sha）")
    p.add_argument(
        "--asset", action="append", default=None, help="只处理指定资产（可重复）"
    )
    p.add_argument("--lock", default=str(DEFAULT_LOCK), help="versions.lock 路径")
    p.add_argument("--licenses", default=str(DEFAULT_LICENSES), help="licenses/registry.yaml 路径")
    p.add_argument("--manifest", default=str(DEFAULT_MANIFEST), help="MANIFEST.yaml 路径")
    p.add_argument("--archive-dir", default=str(DEFAULT_ARCHIVE_DIR), help="归档目录")
    p.add_argument("--workdir", default=str(DEFAULT_WORKDIR), help="git 临时工作目录")
    p.add_argument(
        "--no-lock-update", action="store_true", help="不回写 versions.lock（只出归档与清单）"
    )
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    if args.verify:
        return cmd_verify(args)
    return cmd_fetch(args)


if __name__ == "__main__":
    sys.exit(main())
