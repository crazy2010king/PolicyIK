"""vendor/wheels 离线镜像与 versions.lock:wheels 的一致性测试（T03）。

这些断言守着 S00 的核心交付形态：
* 仓库里只留「锁 + 校验清单」，二进制镜像落在本机 vendor/wheels/；
* versions.lock:wheels 与 SHA256SUMS、与归档里文件的实际 sha256 三者必须一致；
* Mosek / robotic_arm 受再分发限制，其 wheel 只能落本机镜像，不得入 git。

负例同样重要：校验器必须在「清单缺失 / 格式错 / 文件被改 / 文件被删 /
多出未登记文件」时**显式报错**，禁止出现「永远返回 0」的假绿灯。
"""

from __future__ import annotations

import hashlib
import sys
from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tools import wheels_lock  # noqa: E402

LOCK_PATH = REPO_ROOT / "versions.lock"
REGISTRY_PATH = REPO_ROOT / "licenses" / "registry.yaml"
WHEELS_DIR = REPO_ROOT / "vendor" / "wheels"
SHA256SUMS = WHEELS_DIR / "SHA256SUMS"


@pytest.fixture(scope="module")
def lock():
    return yaml.safe_load(LOCK_PATH.read_text(encoding="utf-8"))


@pytest.fixture(scope="module")
def registry():
    return yaml.safe_load(REGISTRY_PATH.read_text(encoding="utf-8"))


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


# --------------------------------------------------------------------------- #
# versions.lock:wheels 段
# --------------------------------------------------------------------------- #
def test_lock_wheels_section_is_non_empty_mapping(lock):
    wheels = lock["wheels"]
    assert isinstance(wheels, dict)
    assert wheels, "versions.lock:wheels 不允许为空（T03 已回写归档 sha256 映射）"
    for name, digest in wheels.items():
        assert isinstance(name, str) and name.endswith((".whl", ".tar.gz", ".zip"))
        assert len(digest) == 64 and all(c in "0123456789abcdef" for c in digest), (
            f"{name} 的 sha256 格式非法: {digest}"
        )


def test_lock_wheels_covers_direct_dependencies(lock):
    """归档必须覆盖直接依赖（否则离线重建缺件）。"""
    names = " ".join(lock["wheels"]).lower()
    for package in ("drake-", "mosek-", "robotic_arm-", "toppra-", "numpy-", "scipy-"):
        assert package in names, f"versions.lock:wheels 缺少 {package} 的发行文件"


def test_lock_wheels_matches_sha256sums_manifest(lock):
    """锁里的映射与 SHA256SUMS 的键集合必须完全一致。"""
    if not SHA256SUMS.is_file():
        pytest.skip("本机无 vendor/wheels 镜像（不入 git），跳过清单对照")
    manifest = wheels_lock.parse_sha256sums(SHA256SUMS.read_text(encoding="utf-8"))
    assert set(manifest) == set(lock["wheels"]), (
        "versions.lock:wheels 与 vendor/wheels/SHA256SUMS 的键集合不一致"
    )
    for name, digest in manifest.items():
        assert lock["wheels"][name] == digest, f"{name} 的 sha256 在锁与清单之间不一致"


def test_archive_files_match_recorded_hashes(lock):
    """归档里每个文件的实际 sha256 必须与记录一致。"""
    if not WHEELS_DIR.is_dir():
        pytest.skip("本机无 vendor/wheels 镜像（不入 git），跳过逐文件校验")
    problems = wheels_lock.verify_archive(WHEELS_DIR)
    assert not problems, "归档校验失败: " + "; ".join(problems)


def test_archive_is_not_tracked_by_git(lock):
    """vendor/ 必须在 .gitignore 里——商业求解器 wheel 绝不能进 git。"""
    gitignore = (REPO_ROOT / ".gitignore").read_text(encoding="utf-8")
    rules = {line.strip() for line in gitignore.splitlines()}
    assert "vendor/" in rules, ".gitignore 必须忽略 vendor/（Mosek wheel 不得入 git）"


# --------------------------------------------------------------------------- #
# 许可：Mosek 不可再分发
# --------------------------------------------------------------------------- #
def test_mosek_marked_not_redistributable(registry):
    entries = {e["name"].lower(): e for e in registry["entries"]}
    assert "mosek" in entries
    assert entries["mosek"]["redistributable"] is False
    assert entries["mosek"]["version"].startswith("11.1.2")


def test_robotic_arm_marked_not_redistributable(registry):
    entries = {e["name"].lower(): e for e in registry["entries"]}
    assert entries["robotic_arm"]["redistributable"] is False


def test_mosek_wheel_is_tracked_only_as_hash(lock):
    """Mosek wheel 只以 sha256 形式被登记（清单在仓库、文件在本机）。"""
    mosek_entries = [name for name in lock["wheels"] if name.lower().startswith("mosek-")]
    assert mosek_entries, "versions.lock:wheels 应登记 Mosek wheel 的 sha256"
    for name in mosek_entries:
        assert not (REPO_ROOT / "vendor" / "wheels" / name).exists() or WHEELS_DIR.is_dir()


# --------------------------------------------------------------------------- #
# 负例：校验器不得「永远绿灯」
# --------------------------------------------------------------------------- #
def test_parse_sha256sums_rejects_malformed_line():
    with pytest.raises(wheels_lock.ArchiveError):
        wheels_lock.parse_sha256sums("deadbeef  broken.whl\n")


def test_parse_sha256sums_rejects_missing_hash():
    with pytest.raises(wheels_lock.ArchiveError):
        wheels_lock.parse_sha256sums("some-file.whl\n")


def test_parse_sha256sums_rejects_duplicate_entry():
    digest = "a" * 64
    with pytest.raises(wheels_lock.ArchiveError):
        wheels_lock.parse_sha256sums(f"{digest}  x.whl\n{digest}  x.whl\n")


def test_parse_sha256sums_rejects_empty_manifest():
    with pytest.raises(wheels_lock.ArchiveError):
        wheels_lock.parse_sha256sums("\n\n")


def test_verify_archive_fails_when_manifest_missing(tmp_path):
    (tmp_path / "pkg-1.0-py3-none-any.whl").write_bytes(b"x")
    with pytest.raises(wheels_lock.ArchiveError):
        wheels_lock.verify_archive(tmp_path)


def test_verify_archive_fails_on_missing_directory(tmp_path):
    with pytest.raises(wheels_lock.ArchiveError):
        wheels_lock.verify_archive(tmp_path / "nope")


def test_verify_archive_detects_modified_file(tmp_path):
    """文件被改 → 必须报出 sha256 不符。"""
    wheel = tmp_path / "pkg-1.0-py3-none-any.whl"
    wheel.write_bytes(b"original")
    manifest = wheels_lock.build_manifest(tmp_path)
    (tmp_path / "SHA256SUMS").write_text(wheels_lock.format_sha256sums(manifest), encoding="utf-8")
    assert wheels_lock.verify_archive(tmp_path) == []

    wheel.write_bytes(b"tampered")
    problems = wheels_lock.verify_archive(tmp_path)
    assert problems and "sha256 不符" in problems[0]


def test_verify_archive_detects_missing_file(tmp_path):
    (tmp_path / "pkg-1.0-py3-none-any.whl").write_bytes(b"x")
    manifest = wheels_lock.build_manifest(tmp_path)
    (tmp_path / "SHA256SUMS").write_text(wheels_lock.format_sha256sums(manifest), encoding="utf-8")
    (tmp_path / "pkg-1.0-py3-none-any.whl").unlink()
    problems = wheels_lock.verify_archive(tmp_path)
    assert problems and "缺失文件" in problems[0]


def test_verify_archive_detects_unregistered_file(tmp_path):
    (tmp_path / "pkg-1.0-py3-none-any.whl").write_bytes(b"x")
    manifest = wheels_lock.build_manifest(tmp_path)
    (tmp_path / "SHA256SUMS").write_text(wheels_lock.format_sha256sums(manifest), encoding="utf-8")
    (tmp_path / "extra-2.0-py3-none-any.whl").write_bytes(b"y")
    problems = wheels_lock.verify_archive(tmp_path)
    assert problems and "多出未登记文件" in problems[0]


def test_replace_wheels_block_requires_wheels_section():
    with pytest.raises(wheels_lock.ArchiveError):
        wheels_lock.replace_wheels_block("schema_version: 1\n", {"a.whl": "b" * 64})


def test_replace_wheels_block_preserves_rest_of_file(lock):
    text = LOCK_PATH.read_text(encoding="utf-8")
    updated = wheels_lock.replace_wheels_block(text, {"pkg-1.0-py3-none-any.whl": "c" * 64})
    assert "schema_version: 1" in updated
    assert "# versions.lock" in updated  # 注释不得被 YAML 往返冲掉
    assert "pkg-1.0-py3-none-any.whl: " + "c" * 64 in updated
    assert updated.count("wheels:") == 1


def test_render_wheels_block_is_valid_yaml():
    rendered = wheels_lock.render_wheels_block({"pkg-1.0-py3-none-any.whl": "d" * 64})
    data = yaml.safe_load(rendered)
    assert data == {"wheels": {"pkg-1.0-py3-none-any.whl": "d" * 64}}
