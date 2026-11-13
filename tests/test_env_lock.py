"""versions.lock / constraints / licenses 三件套的一致性测试（T01）。

这些断言是整个里程碑的"版本真相源"守卫：
锁文件一旦与 constraints 或 PyPI 元数据漂移，golden-file 回归（S07）就会静默失真。
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tools import gen_constraints  # noqa: E402  （需先注入 sys.path）

LOCK_PATH = REPO_ROOT / "versions.lock"
REGISTRY_PATH = REPO_ROOT / "licenses" / "registry.yaml"
CONSTRAINTS_DIR = REPO_ROOT / "constraints"

REQUIRED_FORBIDDEN_APIS = [
    "rm_movej",
    "rm_movep",
    "rm_movec",
    "rm_moves",
    "rm_movej_follow",
    "rm_movep_canfd",
    "rm_algo_inverse_kinematics",
    "rm_algo_set_7dof_q3_track_angle",
    "rm_algo_enable_q3_tracker",
    "rm_algo_set_dq_weight",
    "rm_algo_set_joint_limit_angle",
]
REQUIRED_ALLOWED_APIS = ["rm_movej_canfd", "rm_get_current_joint_current"]

SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
COMMIT_RE = re.compile(r"^[0-9a-f]{40}$")


# --------------------------------------------------------------------------- #
# fixtures
# --------------------------------------------------------------------------- #
@pytest.fixture(scope="module")
def lock():
    data = yaml.safe_load(LOCK_PATH.read_text(encoding="utf-8"))
    assert isinstance(data, dict)
    return data


@pytest.fixture(scope="module")
def registry():
    data = yaml.safe_load(REGISTRY_PATH.read_text(encoding="utf-8"))
    assert isinstance(data, dict)
    return data


def parse_constraints(path: Path) -> dict:
    """解析 constraints 文件 → {包名: (版本, [sha256, ...])}。"""
    entries = {}
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        spec = parts[0]
        assert "==" in spec, f"constraints 行必须用 == 固定版本: {line}"
        name, version = spec.split("==", 1)
        hashes = [p.split("sha256:", 1)[1] for p in parts[1:] if p.startswith("--hash=sha256:")]
        assert hashes, f"{name} 缺少 --hash"
        entries[name.lower()] = (version, hashes)
    return entries


# --------------------------------------------------------------------------- #
# 锁文件结构
# --------------------------------------------------------------------------- #
def test_lock_is_parsable_and_has_required_keys(lock):
    gen_constraints.validate_lock(lock)  # 不抛异常即通过
    for key in gen_constraints.REQUIRED_TOP_LEVEL_KEYS:
        assert key in lock, f"锁文件缺少必填字段 {key}"


def test_baseline_and_fallback_shape(lock):
    baseline, fallback = lock["baseline"], lock["fallback"]
    assert baseline["python"] == "3.12"
    assert str(baseline["drake"]) == "1.55.0"
    assert baseline["supported"] is True
    assert fallback["python"] == "3.10"
    assert str(fallback["drake"]) == "1.51.1"
    assert fallback["supported"] is False
    assert "非受支持" in str(fallback.get("note", ""))


def test_packages_are_pinned(lock):
    packages = lock["packages"]
    assert str(packages["robotic_arm"]["version"]) == "1.1.7"
    assert str(packages["toppra"]["version"]) == "0.6.11"
    assert str(packages["mosek"]["version"]) == "11.1.2"
    for name in ("numpy", "scipy", "matplotlib", "pydot", "pyyaml"):
        assert str(packages[name]["version"])


def test_upstream_commits_are_40_hex(lock):
    upstream = lock["upstream"]
    assert upstream["rm_models"]["commit"] == "cff49bb40f5ed0728505c03bff0e521a0dc21acd"
    assert upstream["two_finger_gripper"]["commit"] == "91f587b1d4efaab235f3d227ee614b28f35060c9"
    for name, spec in upstream.items():
        commit = spec["commit"]
        assert len(commit) == 40, f"{name} commit 不是 40 位"
        assert COMMIT_RE.match(commit), f"{name} commit 不是 40 位十六进制"


def test_platform_section(lock):
    platform = lock["platform"]
    assert "ubuntu 22.04" in platform["os"]
    assert "ubuntu 24.04" in platform["os"]
    assert platform["platform_tag"] == "manylinux_2_34_x86_64"
    assert "cp312" in platform["python_tags"]


# --------------------------------------------------------------------------- #
# 禁用 / 允许名单
# --------------------------------------------------------------------------- #
def test_forbidden_apis_cover_required_list(lock):
    forbidden = set(lock["forbidden_apis"])
    missing = [api for api in REQUIRED_FORBIDDEN_APIS if api not in forbidden]
    assert not missing, f"forbidden_apis 缺少: {missing}"


def test_allowed_apis_not_in_forbidden(lock):
    forbidden = set(lock["forbidden_apis"])
    allowed = set(lock["allowed_apis"])
    assert "rm_movej_canfd" in allowed
    assert "rm_get_current_joint_current" in allowed
    assert not (forbidden & allowed), "allowed_apis 与 forbidden_apis 不应有交集"


def test_rm_movej_canfd_is_not_forbidden(lock):
    """rm_movej_canfd 是允许的透传接口，禁止名单里不得出现（含前缀误伤）。"""
    assert "rm_movej_canfd" not in lock["forbidden_apis"]


# --------------------------------------------------------------------------- #
# constraints 与锁文件一致
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("profile", ["baseline", "fallback"])
def test_constraints_match_lock(lock, profile):
    block = lock[profile]
    path = CONSTRAINTS_DIR / f"{block['profile']}.txt"
    assert path.is_file(), f"缺少 constraints/{block['profile']}.txt"
    entries = parse_constraints(path)

    expected = gen_constraints.resolve_packages(lock, profile)
    assert set(entries) == {name.lower() for name in expected}, "constraints 的包集合与 lock 不一致"
    for name, version in expected.items():
        got_version, hashes = entries[name.lower()]
        assert got_version == version, f"{name} 版本不一致: {got_version} != {version}"
        assert hashes, f"{name} 没有 hash"
        for digest in hashes:
            assert SHA256_RE.match(digest), f"{name} hash 格式错误: {digest}"


def test_py310_constraints_marked_as_unsupported():
    text = (CONSTRAINTS_DIR / "py310.txt").read_text(encoding="utf-8")
    assert "非受支持基线" in text
    assert "fast" in text


def test_constraints_cover_required_packages():
    for filename in ("py312.txt", "py310.txt"):
        entries = parse_constraints(CONSTRAINTS_DIR / filename)
        for required in ("drake", "mosek", "robotic_arm", "toppra", "numpy", "scipy", "matplotlib", "pydot", "pyyaml"):
            assert required in entries, f"{filename} 缺少 {required}"


# --------------------------------------------------------------------------- #
# 许可登记表
# --------------------------------------------------------------------------- #
def test_registry_has_required_entries(registry):
    entries = {e["name"].lower(): e for e in registry["entries"]}
    assert "mosek" in entries
    assert "two-finger-gripper" in entries
    for name, entry in entries.items():
        for field in ("name", "version", "license", "source", "redistributable", "usage_note"):
            assert field in entry, f"registry 条目 {name} 缺少字段 {field}"
        assert isinstance(entry["redistributable"], bool)


def test_mosek_is_not_redistributable(registry):
    entries = {e["name"].lower(): e for e in registry["entries"]}
    assert entries["mosek"]["redistributable"] is False
    assert entries["mosek"]["version"].startswith("11.1.2")


def test_robotic_arm_license_not_copied_from_pypi(registry):
    """上游未明示许可 → 不得照抄 PyPI 的 MIT 字段。"""
    entries = {e["name"].lower(): e for e in registry["entries"]}
    assert entries["robotic_arm"]["redistributable"] is False
    assert "MIT" not in entries["robotic_arm"]["license"]


# --------------------------------------------------------------------------- #
# 负例：锁文件非法时必须报错（禁止"永远绿灯"）
# --------------------------------------------------------------------------- #
def test_validate_lock_rejects_missing_sections(lock):
    for key in ("baseline", "packages", "forbidden_apis", "upstream"):
        broken = dict(lock)
        broken.pop(key)
        with pytest.raises(gen_constraints.LockError):
            gen_constraints.validate_lock(broken)


def test_validate_lock_rejects_bad_commit(lock):
    broken = dict(lock)
    broken["upstream"] = dict(lock["upstream"])
    broken["upstream"]["rm_models"] = dict(lock["upstream"]["rm_models"])
    broken["upstream"]["rm_models"]["commit"] = "not-a-sha"
    with pytest.raises(gen_constraints.LockError):
        gen_constraints.validate_lock(broken)


def test_resolve_packages_rejects_unknown_profile(lock):
    with pytest.raises(gen_constraints.LockError):
        gen_constraints.resolve_packages(lock, "py313")


def test_select_files_filters_platform_and_python():
    files = [
        {"filename": "x-1.0-cp312-cp312-manylinux_2_34_x86_64.whl", "packagetype": "bdist_wheel",
         "digests": {"sha256": "a" * 64}},
        {"filename": "x-1.0-cp310-cp310-manylinux_2_34_x86_64.whl", "packagetype": "bdist_wheel",
         "digests": {"sha256": "b" * 64}},
        {"filename": "x-1.0-cp312-cp312-macosx_11_0_arm64.whl", "packagetype": "bdist_wheel",
         "digests": {"sha256": "c" * 64}},
        {"filename": "x-1.0-cp312-cp312-musllinux_1_2_x86_64.whl", "packagetype": "bdist_wheel",
         "digests": {"sha256": "d" * 64}},
        {"filename": "x-1.0-py3-none-any.whl", "packagetype": "bdist_wheel",
         "digests": {"sha256": "e" * 64}},
        {"filename": "x-1.0.tar.gz", "packagetype": "sdist", "digests": {"sha256": "f" * 64}},
    ]
    picked = {e["filename"] for e in gen_constraints.select_files(files, "cp312")}
    assert picked == {
        "x-1.0-cp312-cp312-manylinux_2_34_x86_64.whl",
        "x-1.0-py3-none-any.whl",
        "x-1.0.tar.gz",
    }


def test_python_tag_compatibility():
    assert gen_constraints._python_tag_compatible("py3", "none", "cp312")
    assert gen_constraints._python_tag_compatible("cp312", "cp312", "cp312")
    assert not gen_constraints._python_tag_compatible("cp310", "cp310", "cp312")
    assert gen_constraints._python_tag_compatible("cp39", "abi3", "cp312")
    assert not gen_constraints._python_tag_compatible("cp39", "abi3", "cp38")


# --------------------------------------------------------------------------- #
# constraints 的 sha256 必须与 PyPI 元数据一致（需要网络，无网络则跳过）
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("profile", ["baseline", "fallback"])
def test_constraint_hashes_match_pypi_metadata(lock, profile):
    block = lock[profile]
    entries = parse_constraints(CONSTRAINTS_DIR / f"{block['profile']}.txt")
    packages = gen_constraints.resolve_packages(lock, profile)
    for name, version in packages.items():
        try:
            meta = gen_constraints.fetch_release(name, version, offline=False)
        except gen_constraints.FetchError as exc:
            pytest.skip(f"无法访问 PyPI，跳过 sha256 对照校验: {exc}")
        remote = {
            entry["digests"]["sha256"]
            for entry in meta.get("urls", [])
            if entry.get("digests", {}).get("sha256")
        }
        local = set(entries[name.lower()][1])
        assert local <= remote, f"{name} {version} 的 sha256 与 PyPI 元数据不一致"
