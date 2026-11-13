"""tests/test_fetch_upstream.py —— T06：上游资产 commit 锁定与离线归档

覆盖（正例）
  - versions.lock:upstream 两个 commit 为 40 位 hex，且与 third_party/MANIFEST.yaml 一致
  - MANIFEST 每个条目 sha256 为 64 位 hex、文件名字段齐全
  - versions.lock:upstream.*.archive.sha256 与 MANIFEST 一致
  - 归档存在时，`tools/fetch_upstream.py --verify` 退出码 0

覆盖（负例）
  - 归档缺失时 `--verify` 必须非零退出并给出明确失败信息（不得静默通过）
  - 归档被篡改（内容改一个字节）时 `--verify` 必须报 sha256 不匹配并非零退出
  - MANIFEST 缺失 / 条目缺 sha256 → 校验器必须失败
"""

from __future__ import annotations

import hashlib
import re
import shutil
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
LOCK_PATH = REPO_ROOT / "versions.lock"
MANIFEST_PATH = REPO_ROOT / "third_party" / "MANIFEST.yaml"
ARCHIVE_DIR = REPO_ROOT / "third_party" / "archives"
TOOL = REPO_ROOT / "tools" / "fetch_upstream.py"

HEX40 = re.compile(r"^[0-9a-f]{40}$")
HEX64 = re.compile(r"^[0-9a-f]{64}$")

EXPECTED_ASSETS = {"rm_models", "two_finger_gripper"}


# ── fixtures ─────────────────────────────────────────────────────────────
@pytest.fixture(scope="module")
def lock() -> dict:
    with LOCK_PATH.open("r", encoding="utf-8") as fh:
        return yaml.safe_load(fh)


@pytest.fixture(scope="module")
def manifest() -> dict:
    assert MANIFEST_PATH.exists(), f"缺少清单 {MANIFEST_PATH}（需运行 tools/fetch_upstream.py）"
    with MANIFEST_PATH.open("r", encoding="utf-8") as fh:
        return yaml.safe_load(fh)


@pytest.fixture(scope="module")
def manifest_assets(manifest) -> dict:
    return {a["name"]: a for a in manifest["assets"]}


def run_tool(*args, cwd=None):
    return subprocess.run(
        [sys.executable, str(TOOL), *args],
        cwd=str(cwd or REPO_ROOT),
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        timeout=300,
    )


# ── versions.lock ↔ MANIFEST 一致性 ───────────────────────────────────────
def test_upstream_commits_are_40_hex(lock):
    upstream = lock["upstream"]
    assert set(upstream) == EXPECTED_ASSETS
    for name, spec in upstream.items():
        assert HEX40.match(spec["commit"]), f"{name}: commit 不是 40 位 hex: {spec['commit']}"


def test_manifest_asset_names_match_lock(lock, manifest_assets):
    assert set(manifest_assets) == set(lock["upstream"])


def test_manifest_commits_match_lock(lock, manifest_assets):
    for name, spec in lock["upstream"].items():
        assert manifest_assets[name]["commit"] == spec["commit"], (
            f"{name}: MANIFEST commit({manifest_assets[name]['commit']}) "
            f"与 versions.lock commit({spec['commit']}) 不一致"
        )


def test_manifest_entries_have_wellformed_sha256(manifest_assets):
    for name, asset in manifest_assets.items():
        sha = asset.get("sha256")
        assert sha, f"{name}: 清单条目缺少 sha256"
        assert HEX64.match(str(sha)), f"{name}: sha256 格式错误: {sha}"
        assert asset.get("archive"), f"{name}: 清单条目缺少 archive 文件名"
        assert isinstance(asset.get("file_count"), int), f"{name}: file_count 必须是整数"
        assert isinstance(asset.get("total_bytes"), int), f"{name}: total_bytes 必须是整数"
        assert asset.get("repo", "").startswith("http"), f"{name}: repo 必须是 http(s) 地址"
        assert asset.get("license"), f"{name}: 清单条目缺少 license 字段"


def test_lock_archive_block_matches_manifest(lock, manifest_assets):
    for name, asset in manifest_assets.items():
        arch = lock["upstream"][name].get("archive")
        assert isinstance(arch, dict), f"{name}: versions.lock 缺少 upstream.*.archive 段"
        assert arch["sha256"] == asset["sha256"], f"{name}: lock/manifest 的 sha256 不一致"
        assert arch["filename"] == asset["archive"], f"{name}: lock/manifest 的归档文件名不一致"


# ── 校验器正例 ───────────────────────────────────────────────────────────
def test_verify_succeeds_when_archives_present():
    if not ARCHIVE_DIR.exists() or not any(ARCHIVE_DIR.glob("*.tar.gz")):
        pytest.skip("归档不存在（本机镜像未建立），跳过正向校验")
    proc = run_tool("--verify")
    assert proc.returncode == 0, f"--verify 失败:\n{proc.stdout}\n{proc.stderr}"
    assert "校验通过" in proc.stdout


# ── 校验器负例 ───────────────────────────────────────────────────────────
def test_verify_fails_when_archive_missing(tmp_path):
    """归档缺失 → 必须非零退出并明确报出缺失路径（不得静默通过）。"""
    empty = tmp_path / "archives"
    empty.mkdir()
    proc = run_tool("--verify", "--archive-dir", str(empty))
    assert proc.returncode != 0, "归档缺失时 --verify 竟返回 0（假绿灯）"
    combined = proc.stdout + proc.stderr
    assert "归档缺失" in combined, f"缺少明确的缺失提示:\n{combined}"
    assert "archives" in combined


def test_verify_detects_tampered_archive(tmp_path):
    """归档内容被改一个字节 → sha256 不匹配 → 非零退出。"""
    if not MANIFEST_PATH.exists():
        pytest.skip("缺少清单")
    tampered_dir = tmp_path / "archives"
    tampered_dir.mkdir()
    manifest = yaml.safe_load(MANIFEST_PATH.read_text(encoding="utf-8"))
    for asset in manifest["assets"]:
        src = ARCHIVE_DIR / asset["archive"]
        if not src.exists():
            pytest.skip("归档不存在，无法构造篡改负例")
        dst = tampered_dir / asset["archive"]
        shutil.copyfile(src, dst)
        data = bytearray(dst.read_bytes())
        data[-1] ^= 0xFF
        dst.write_bytes(bytes(data))
    proc = run_tool("--verify", "--archive-dir", str(tampered_dir))
    assert proc.returncode != 0, "归档被篡改时 --verify 竟返回 0（假绿灯）"
    combined = proc.stdout + proc.stderr
    assert "sha256 不匹配" in combined, f"缺少 sha256 不匹配提示:\n{combined}"


def test_verify_fails_when_manifest_missing(tmp_path):
    proc = run_tool(
        "--verify",
        "--manifest",
        str(tmp_path / "NO_SUCH_MANIFEST.yaml"),
        "--archive-dir",
        str(tmp_path),
    )
    assert proc.returncode != 0, "清单缺失时 --verify 竟返回 0（假绿灯）"
    assert "清单缺失" in (proc.stdout + proc.stderr)


def test_verify_fails_on_malformed_manifest_entry(tmp_path):
    """清单条目缺 sha256 → 校验器必须失败，而不是跳过该项。"""
    bad = tmp_path / "MANIFEST.yaml"
    bad.write_text(
        yaml.safe_dump(
            {"schema_version": 1, "assets": [{"name": "rm_models", "archive": "x.tar.gz"}]},
            allow_unicode=True,
        ),
        encoding="utf-8",
    )
    proc = run_tool("--verify", "--manifest", str(bad), "--archive-dir", str(tmp_path))
    assert proc.returncode != 0, "清单缺 sha256 时 --verify 竟返回 0（假绿灯）"
    assert "缺少 sha256" in (proc.stdout + proc.stderr)


# ── 工具自身约束 ─────────────────────────────────────────────────────────
def test_tool_does_not_hardcode_commits():
    """commit / repo / subset 的唯一真相源是 versions.lock，脚本里不得出现第二份。"""
    source = TOOL.read_text(encoding="utf-8")
    for sha in ("cff49bb40f5ed0728505c03bff0e521a0dc21acd", "91f587b1d4efaab235f3d227ee614b28f35060c9"):
        assert sha not in source, f"脚本里硬编码了 commit {sha}（应从 versions.lock 读）"
    assert "RealManRobot" not in source, "脚本里硬编码了上游 repo 地址（应从 versions.lock 读）"


def test_tool_help_mentions_verify_and_ref():
    proc = run_tool("--help")
    assert proc.returncode == 0
    assert "--verify" in proc.stdout
    assert "--ref" in proc.stdout


def test_gitignore_excludes_archives_and_meshes():
    gi = (REPO_ROOT / ".gitignore").read_text(encoding="utf-8")
    # 只看真正的忽略规则（注释行不算）
    rules = {
        ln.strip()
        for ln in gi.splitlines()
        if ln.strip() and not ln.strip().startswith("#")
    }
    assert "third_party/archives/" in rules
    assert "models/**/meshes/" in rules
    # 清单必须入 git（不能被忽略）
    assert "third_party/MANIFEST.yaml" not in rules
    assert "third_party/" not in rules


def test_manifest_yaml_is_in_git_tracked_scope():
    """MANIFEST.yaml 必须存在且可被 yaml 解析（它是入 git 的凭据）。"""
    assert MANIFEST_PATH.exists()
    data = yaml.safe_load(MANIFEST_PATH.read_text(encoding="utf-8"))
    assert data["assets"]
    # 归档本体本身不入 git
    assert "third_party/archives/" in (REPO_ROOT / ".gitignore").read_text(encoding="utf-8")


def test_archive_sha256_matches_actual_file_when_present(manifest_assets):
    for name, asset in manifest_assets.items():
        path = ARCHIVE_DIR / asset["archive"]
        if not path.exists():
            pytest.skip(f"{name}: 归档不在本机，跳过逐字节校验")
        h = hashlib.sha256()
        with path.open("rb") as fh:
            for chunk in iter(lambda: fh.read(1 << 20), b""):
                h.update(chunk)
        assert h.hexdigest() == asset["sha256"], f"{name}: 归档实际 sha256 与清单不符"
