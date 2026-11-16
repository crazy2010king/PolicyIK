"""tests/test_asset_license_gate.py —— 资产与许可门禁（S07-T04）

这些断言守住的契约：

* ``tools/verify_env.py --upstream --licenses`` 在真实仓库上必须退出 0；
* ``licenses/registry.yaml`` 的包集必须**包含** ``versions.lock:packages`` 的全部包
  （缺一条必须失败并**点名**是哪一条），且每条 entry 必填字段齐全；
* 上游资产类条目必须显式声明 commit；``commit: null`` 只允许配 ``locked: false``
  （两个已知例外必须被计数列出，且**不得编造 commit**）；
* 仓库根 ``NOTICE`` 必须是 ``licenses/registry.yaml`` 的派生物（逐字一致）；
* ``scripts/extract_upstream_assets.py`` 解包前必须校验归档 sha256；
* MANIFEST 与 versions.lock 的 commit 不一致时门禁必须失败。

负例一律用**临时目录替身**（不碰仓库里的真实资产），因此本文件可在无归档、
无网络的机器上全量运行。
"""

from __future__ import annotations

import copy
import hashlib
import json
import subprocess
import sys
import tarfile
from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tools import verify_env  # noqa: E402

VERIFY = REPO_ROOT / "tools" / "verify_env.py"
GEN_NOTICE = REPO_ROOT / "tools" / "gen_notice.py"
EXTRACT = REPO_ROOT / "scripts" / "extract_upstream_assets.py"
LOCK_PATH = REPO_ROOT / "versions.lock"
REGISTRY_PATH = REPO_ROOT / "licenses" / "registry.yaml"
MANIFEST_PATH = REPO_ROOT / "third_party" / "MANIFEST.yaml"
NOTICE_PATH = REPO_ROOT / "NOTICE"


# --------------------------------------------------------------------------- #
# fixtures
# --------------------------------------------------------------------------- #
@pytest.fixture(scope="module")
def lock() -> dict:
    return yaml.safe_load(LOCK_PATH.read_text(encoding="utf-8"))


@pytest.fixture(scope="module")
def registry() -> dict:
    return yaml.safe_load(REGISTRY_PATH.read_text(encoding="utf-8"))


def run(cmd: list[str], **kwargs) -> subprocess.CompletedProcess:
    return subprocess.run(
        cmd,
        cwd=str(REPO_ROOT),
        capture_output=True,
        text=True,
        timeout=300,
        **kwargs,
    )


def write_yaml(path: Path, data: dict) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(data, allow_unicode=True, sort_keys=False), encoding="utf-8")
    return path


# --------------------------------------------------------------------------- #
# 1) 门禁本身必须绿
# --------------------------------------------------------------------------- #
def test_verify_env_upstream_licenses_exit_zero():
    """真实仓库上 --upstream --licenses 必须退出 0。"""
    proc = run([sys.executable, str(VERIFY), "--upstream", "--licenses"])
    assert proc.returncode == 0, f"stdout={proc.stdout}\nstderr={proc.stderr}"


def test_verify_env_upstream_licenses_json_reports_both_checks():
    """--json 必须同时给出 upstream / licenses 两个子检查，且均为 PASS。"""
    proc = run([sys.executable, str(VERIFY), "--upstream", "--licenses", "--json"])
    assert proc.returncode == 0, proc.stdout + proc.stderr
    report = json.loads(proc.stdout)
    assert set(report["checks"]) == {"upstream", "licenses"}
    for name in ("upstream", "licenses"):
        assert report["checks"][name]["status"] == "PASS", report["checks"][name]


# --------------------------------------------------------------------------- #
# 2) registry 包集必须覆盖 lock 的 17 个包
# --------------------------------------------------------------------------- #
def test_registry_covers_every_lock_package(lock, registry):
    """registry 的包集必须包含 versions.lock:packages 的全部包（归一化后比较）。"""
    index = {verify_env.normalize_component_name(e["name"]): e for e in registry["entries"]}
    missing = [
        name
        for name in lock["packages"]
        if verify_env.normalize_component_name(name) not in index
    ]
    assert not missing, f"registry 缺少 lock 中的包: {missing}"
    assert len(lock["packages"]) == 17, "lock 的包集应为 17 个（含 9 个传递依赖）"


def test_missing_lock_package_is_named(tmp_path, lock, registry):
    """删掉 registry 里的一条包 → 门禁失败并**点名**缺哪一条（负例）。"""
    broken = copy.deepcopy(registry)
    broken["entries"] = [
        e for e in broken["entries"] if e["name"] != "tzdata"
    ]
    broken_path = write_yaml(tmp_path / "registry.yaml", broken)
    status, problems, summary = verify_env.check_licenses(lock, broken_path)
    assert status == "FAIL"
    assert summary["missing_packages"] == ["tzdata"]
    assert any("tzdata" in p for p in problems), problems


def test_registry_entries_have_all_required_fields(registry):
    """每条 entry 的必填字段齐全，redistributable 必须是布尔。"""
    for entry in registry["entries"]:
        for field in verify_env.REQUIRED_REGISTRY_FIELDS:
            assert field in entry, f"registry 条目 {entry.get('name')} 缺少字段 {field}"
        assert isinstance(entry["redistributable"], bool)


def test_unrestricted_license_must_not_be_redistributable(lock, tmp_path, registry):
    """许可未明示的条目必须禁再分发（负例：改成 true 必须失败）。"""
    broken = copy.deepcopy(registry)
    for entry in broken["entries"]:
        if entry["name"] == "rm_models":
            entry["redistributable"] = True
    broken_path = write_yaml(tmp_path / "registry.yaml", broken)
    status, problems, _ = verify_env.check_licenses(lock, broken_path)
    assert status == "FAIL"
    assert any("rm_models" in p and "未明示" in p for p in problems), problems


# --------------------------------------------------------------------------- #
# 3) 上游资产类条目必须显式声明 commit
# --------------------------------------------------------------------------- #
def test_two_unlocked_entries_have_null_commit_and_locked_false(registry):
    """两个未锁定条目必须 commit: null + locked: false（不得编造 commit）。"""
    expected = {"constrained-bimanual-planning-example", "manipulation"}
    index = {e["name"]: e for e in registry["entries"]}
    for name in expected:
        entry = index[name]
        assert entry["commit"] is None, f"{name} 的 commit 必须为 null（不得编造）"
        assert entry["locked"] is False, f"{name} 的 locked 必须为 False"
        assert "commit" in entry and "locked" in entry


def test_locked_upstream_assets_declare_real_commit(lock, registry):
    """锁文件里的上游资产，其 registry 条目必须显式声明与 lock 一致的 commit。"""
    index = {
        verify_env.normalize_component_name(e["name"]): e for e in registry["entries"]
    }
    for name, spec in lock["upstream"].items():
        entry = index[verify_env.normalize_component_name(name)]
        assert "commit" in entry, f"{name} 的 registry 条目必须显式声明 commit"
        assert entry["commit"] == spec["commit"]
        assert entry["locked"] is True


def test_unlocked_exception_is_counted_and_listed(lock, registry):
    """未锁定例外必须被计数并逐条列出。"""
    status, problems, summary = verify_env.check_licenses(lock, REGISTRY_PATH)
    assert status == "PASS", problems
    assert summary["unlocked_exception_count"] == 2
    assert set(summary["unlocked_exceptions"]) == {
        "constrained-bimanual-planning-example",
        "manipulation",
    }


def test_null_commit_with_locked_true_fails(lock, tmp_path, registry):
    """commit: null 却 locked: true → 必须失败（负例）。"""
    broken = copy.deepcopy(registry)
    for entry in broken["entries"]:
        if entry["name"] == "manipulation":
            entry["locked"] = True
    broken_path = write_yaml(tmp_path / "registry.yaml", broken)
    status, problems, _ = verify_env.check_licenses(lock, broken_path)
    assert status == "FAIL"
    assert any("manipulation" in p for p in problems), problems


# --------------------------------------------------------------------------- #
# 4) NOTICE 必须是 registry 的派生物
# --------------------------------------------------------------------------- #
def test_notice_matches_regenerated_output():
    """仓库根 NOTICE 必须与重新生成的结果逐字一致。"""
    assert NOTICE_PATH.is_file(), "仓库根 NOTICE 不存在"
    registry = yaml.safe_load(REGISTRY_PATH.read_text(encoding="utf-8"))
    from tools import gen_notice

    assert NOTICE_PATH.read_text(encoding="utf-8") == gen_notice.render_notice(registry)


def test_gen_notice_check_mode_passes(tmp_path):
    """--check 模式对仓库里的 NOTICE 必须退出 0。"""
    proc = run([sys.executable, str(GEN_NOTICE), "--check"])
    assert proc.returncode == 0, proc.stdout + proc.stderr


def test_gen_notice_detects_stale_notice(tmp_path, registry):
    """NOTICE 与登记表不一致时 --check 必须失败（负例）。"""
    stale = tmp_path / "NOTICE"
    stale.write_text("# 过期\n", encoding="utf-8")
    proc = run([sys.executable, str(GEN_NOTICE), "--check", "--out", str(stale)])
    assert proc.returncode == 1
    assert "不一致" in (proc.stdout + proc.stderr)


def test_notice_has_three_sections():
    """NOTICE 必须分三段（允许再分发 / 受限禁止再分发 / commit 未锁定例外）。"""
    text = NOTICE_PATH.read_text(encoding="utf-8")
    assert "## 1. 允许再分发" in text
    assert "## 2. 受限、禁止再分发" in text
    assert "## 3. commit 未锁定例外" in text
    # 未锁定例外必须出现在第三段（而不是被当作已锁定组件）
    third = text.split("## 3. commit 未锁定例外", 1)[1]
    assert "constrained-bimanual-planning-example" in third
    assert "manipulation" in third


# --------------------------------------------------------------------------- #
# 5) 解包前的 sha256 门禁（负例：篡改归档）
# --------------------------------------------------------------------------- #
def _make_lock_with_sha(tmp_path: Path, payload: bytes) -> tuple[Path, str]:
    """造一个含 upstream.rm_models 的临时 versions.lock（sha256 由 payload 决定）。"""
    sha = hashlib.sha256(payload).hexdigest()
    lock_data = {
        "schema_version": 1,
        "upstream": {
            "rm_models": {
                "repo": "https://example.invalid/rm_models",
                "ref": "main",
                "commit": "0" * 40,
                "subset": "RM75/urdf/RM75-B/",
                "archive": {
                    "filename": "rm_models-test.tar.gz",
                    "sha256": sha,
                    "file_count": 1,
                    "total_bytes": len(payload),
                },
            }
        },
    }
    lock_path = write_yaml(tmp_path / "versions.lock", lock_data)
    return lock_path, sha


def _make_archive(tmp_path: Path, payload_name: str = "RM75/urdf/RM75-B/x.urdf") -> Path:
    """造一个最小 tar.gz 归档（内容确定）。"""
    archive_dir = tmp_path / "archives"
    archive_dir.mkdir(parents=True, exist_ok=True)
    archive_path = archive_dir / "rm_models-test.tar.gz"
    with tarfile.open(archive_path, "w:gz") as tar:
        data = b"<robot/>"
        info = tarfile.TarInfo(payload_name)
        info.size = len(data)
        import io

        tar.addfile(info, io.BytesIO(data))
    return archive_path


def test_extract_rejects_tampered_archive(tmp_path):
    """篡改归档 → 解包脚本必须退出 1，并打印期望值与实测值。"""
    _make_archive(tmp_path)
    archive_path = tmp_path / "archives" / "rm_models-test.tar.gz"
    # 篡改字节
    archive_path.write_bytes(archive_path.read_bytes() + b"tampered")

    lock_path, expected_sha = _make_lock_with_sha(tmp_path, b"whatever")

    proc = run(
        [
            sys.executable,
            str(EXTRACT),
            "--asset",
            "rm_models",
            "--lock",
            str(lock_path),
            "--archive-dir",
            str(tmp_path / "archives"),
            "--dest",
            str(tmp_path / "dest"),
        ]
    )
    assert proc.returncode == 1, proc.stdout + proc.stderr
    combined = proc.stdout + proc.stderr
    assert "sha256" in combined
    assert expected_sha in combined  # 期望值
    # 实测值也必须打印（= 被篡改后文件的 sha256）
    actual = hashlib.sha256(archive_path.read_bytes()).hexdigest()
    assert actual in combined
    dest = tmp_path / "dest"
    assert not dest.exists() or not any(dest.rglob("*")), "sha256 不符时不得写出任何文件"


def test_extract_missing_archive_still_exit_one(tmp_path):
    """缺归档仍是退出码 1（语义不变，负例）。"""
    lock_path, _ = _make_lock_with_sha(tmp_path, b"whatever")
    proc = run(
        [
            sys.executable,
            str(EXTRACT),
            "--asset",
            "rm_models",
            "--lock",
            str(lock_path),
            "--archive-dir",
            str(tmp_path / "archives"),
            "--dest",
            str(tmp_path / "dest"),
        ]
    )
    assert proc.returncode == 1
    assert "归档不存在" in (proc.stdout + proc.stderr)


# --------------------------------------------------------------------------- #
# 6) MANIFEST ↔ versions.lock 的 commit 不一致必须失败（负例）
# --------------------------------------------------------------------------- #
def test_manifest_commit_mismatch_fails(tmp_path, lock, registry):
    """把 MANIFEST 里的 commit 改掉 → 门禁必须失败并点名。"""
    manifest = yaml.safe_load(MANIFEST_PATH.read_text(encoding="utf-8"))
    manifest = copy.deepcopy(manifest)
    manifest["assets"][0]["commit"] = "f" * 40
    manifest_path = write_yaml(tmp_path / "MANIFEST.yaml", manifest)
    status, problems, _ = verify_env.check_upstream(lock, manifest_path, REGISTRY_PATH)
    assert status == "FAIL"
    assert any("commit" in p for p in problems), problems


def test_manifest_sha256_mismatch_fails(tmp_path, lock, registry):
    """MANIFEST 的 sha256 与 lock 不一致 → 必须失败。"""
    manifest = copy.deepcopy(yaml.safe_load(MANIFEST_PATH.read_text(encoding="utf-8")))
    manifest["assets"][0]["sha256"] = "0" * 64
    manifest_path = write_yaml(tmp_path / "MANIFEST.yaml", manifest)
    status, problems, _ = verify_env.check_upstream(lock, manifest_path, REGISTRY_PATH)
    assert status == "FAIL"
    assert any("sha256" in p for p in problems), problems


def test_registry_license_mismatch_fails(tmp_path, lock):
    """registry 的 license 与 MANIFEST 不一致 → 必须失败（负例）。"""
    registry_data = copy.deepcopy(yaml.safe_load(REGISTRY_PATH.read_text(encoding="utf-8")))
    for entry in registry_data["entries"]:
        if entry["name"] == "rm_models":
            entry["license"] = "MIT（臆造）"
    registry_path = write_yaml(tmp_path / "registry.yaml", registry_data)
    status, problems, _ = verify_env.check_upstream(lock, MANIFEST_PATH, registry_path)
    assert status == "FAIL"
    assert any("license" in p for p in problems), problems


def test_upstream_check_does_not_require_archive_entity(tmp_path, lock, registry):
    """归档缺失只如实报告，**不判为不一致**（归档不入 git）。"""
    status, problems, summary = verify_env.check_upstream(
        lock, MANIFEST_PATH, REGISTRY_PATH, archive_dir=tmp_path / "no-archives"
    )
    assert status == "PASS", problems
    assert summary["notes"], "缺归档必须被如实报告"
