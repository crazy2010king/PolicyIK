"""tools/verify_env.py 与 tools/setup_env.sh 的行为测试（T04）。

这些断言守住的契约：

* ``--lock-only`` 在真实锁文件上必须退出 0（锁自洽）；
* ``--check`` 的 profile 选择必须跟随当前解释器（3.12 → baseline，3.10 → fallback），
  且**实装版本与声明不符时必须非 0**（禁止"永远返回 0"的假绿灯）；
* ``--forbidden`` 必须确认禁用名单来自 versions.lock 这一唯一真相源；
* ``setup_env.sh`` 必须语法正确、帮助齐全、``--dry-run`` 不产生任何副作用。

负例同样重要：伪造缺字段的锁、错配版本、缺包、削减禁用名单都必须被显式报错。
"""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tools import verify_env  # noqa: E402

VERIFY = REPO_ROOT / "tools" / "verify_env.py"
SETUP = REPO_ROOT / "tools" / "setup_env.sh"
LOCK_PATH = REPO_ROOT / "versions.lock"
LIMITS_PATH = REPO_ROOT / "models" / "rm75_drake" / "joint_limits.yaml"


def run_verify(*args: str) -> subprocess.CompletedProcess:
    """以子进程方式调用 verify_env.py（真实 CLI 行为）。"""
    return subprocess.run(
        [sys.executable, str(VERIFY), *args],
        cwd=str(REPO_ROOT),
        capture_output=True,
        text=True,
        timeout=300,
    )


def run_setup(*args: str, env: dict | None = None) -> subprocess.CompletedProcess:
    """以子进程方式调用 setup_env.sh。"""
    import os

    full_env = dict(os.environ)
    if env:
        full_env.update(env)
    return subprocess.run(
        ["bash", str(SETUP), *args],
        cwd=str(REPO_ROOT),
        capture_output=True,
        text=True,
        timeout=1800,
        env=full_env,
    )


@pytest.fixture(scope="module")
def lock() -> dict:
    return yaml.safe_load(LOCK_PATH.read_text(encoding="utf-8"))


def _write_lock(tmp_path: Path, data: dict, name: str = "versions.lock") -> Path:
    target = tmp_path / name
    target.write_text(yaml.safe_dump(data, allow_unicode=True, sort_keys=False), encoding="utf-8")
    return target


# --------------------------------------------------------------------------- #
# --lock-only
# --------------------------------------------------------------------------- #
def test_lock_only_exits_zero():
    result = run_verify("--lock-only")
    assert result.returncode == 0, result.stdout + result.stderr
    assert "lock-only" in result.stdout


def test_lock_only_json_output_is_parseable():
    result = run_verify("--lock-only", "--json")
    assert result.returncode == 0, result.stdout + result.stderr
    payload = json.loads(result.stdout)
    assert payload["ok"] is True
    assert payload["checks"]["lock-only"]["status"] == "PASS"


def test_lock_only_rejects_lock_missing_required_field(tmp_path, lock):
    """负例：缺字段的锁必须非 0（禁止假绿灯）。"""
    broken = dict(lock)
    broken.pop("forbidden_apis")
    target = _write_lock(tmp_path, broken)
    result = run_verify("--lock-only", "--lock", str(target))
    assert result.returncode != 0
    assert "forbidden_apis" in (result.stdout + result.stderr)


def test_lock_only_rejects_unparsable_lock(tmp_path):
    target = tmp_path / "broken.lock"
    target.write_text("schema_version: 1\npackages: [oops\n", encoding="utf-8")
    result = run_verify("--lock-only", "--lock", str(target))
    assert result.returncode == 2  # 运行期错误
    assert "锁文件" in (result.stdout + result.stderr)


def test_lock_only_rejects_missing_lock_file(tmp_path):
    result = run_verify("--lock-only", "--lock", str(tmp_path / "nope.lock"))
    assert result.returncode == 2


def test_check_lock_only_detects_constraint_drift(tmp_path, lock):
    """负例：constraints 与锁的钉版不一致时 --lock-only 必须失败。"""
    constraints = tmp_path / "constraints"
    constraints.mkdir()
    for name in ("py312.txt", "py310.txt"):
        text = (REPO_ROOT / "constraints" / name).read_text(encoding="utf-8")
        (constraints / name).write_text(
            text.replace("numpy==1.26.4", "numpy==1.26.3"), encoding="utf-8"
        )
    result = run_verify("--lock-only", "--constraints-dir", str(constraints))
    assert result.returncode != 0
    assert "numpy" in (result.stdout + result.stderr)


# --------------------------------------------------------------------------- #
# --check：profile 选择与版本对照
# --------------------------------------------------------------------------- #
def test_select_profile_maps_python_versions_to_lock_sections(lock):
    assert verify_env.select_profile(lock, "3.12")[0] == "baseline"
    assert verify_env.select_profile(lock, "3.10")[0] == "fallback"


def test_select_profile_rejects_unsupported_python(lock):
    with pytest.raises(verify_env.VerifyError):
        verify_env.select_profile(lock, "3.9")


def test_check_reports_selected_profile_for_current_interpreter():
    """当前解释器对应的 profile 必须在输出里明示。"""
    result = run_verify("--check")
    expected = "baseline" if sys.version_info[:2] == (3, 12) else "fallback"
    assert f"profile: {expected}" in result.stdout
    # 运行期错误（2）与检查失败（1）都算"跑起来了"，但绝不能是崩溃
    assert result.returncode in (0, 1)


def test_check_fails_on_mismatched_declared_version(tmp_path, lock):
    """负例：锁里声明的版本与实装不符 → 必须非 0 并点名该包。"""
    broken = dict(lock)
    packages = dict(lock["packages"])
    packages["numpy"] = {"version": "0.0.1-not-installed", "note": "负例"}
    broken["packages"] = packages
    target = _write_lock(tmp_path, broken)
    result = run_verify("--check", "--lock", str(target))
    assert result.returncode == 1
    assert "numpy" in result.stdout


def test_check_fails_on_missing_package(tmp_path, lock):
    """负例：锁里声明了未安装的包 → 必须报"未安装"并非 0。"""
    broken = dict(lock)
    packages = dict(lock["packages"])
    packages["totally-missing-pkg"] = {"version": "1.2.3", "note": "负例"}
    broken["packages"] = packages
    target = _write_lock(tmp_path, broken)
    result = run_verify("--check", "--lock", str(target))
    assert result.returncode == 1
    assert "未安装" in result.stdout
    assert "totally-missing-pkg" in result.stdout


def test_check_profile_override_is_honoured(tmp_path, lock):
    """显式 --profile 时必须用指定档位，而不是解释器档位。"""
    target = _write_lock(tmp_path, lock)
    result = run_verify("--check", "--lock", str(target), "--profile", "baseline")
    assert "profile: baseline" in result.stdout
    assert result.returncode in (0, 1)


def test_check_returns_runtime_error_for_unknown_profile_lock(tmp_path):
    """锁里没有 baseline/fallback 段 → 运行期错误（退出码 2），不静默通过。"""
    broken = {"schema_version": 1, "packages": {}, "wheels": {}}
    target = _write_lock(tmp_path, broken)
    result = run_verify("--check", "--lock", str(target))
    assert result.returncode == 2


# --------------------------------------------------------------------------- #
# --forbidden
# --------------------------------------------------------------------------- #
def test_forbidden_exits_zero():
    result = run_verify("--forbidden")
    assert result.returncode == 0, result.stdout + result.stderr


def test_forbidden_detects_missing_core_api(tmp_path, lock):
    """负例：禁用名单被削减 → 必须非 0。"""
    broken = dict(lock)
    broken["forbidden_apis"] = ["rm_movej"]
    target = _write_lock(tmp_path, broken)
    result = run_verify("--forbidden", "--lock", str(target))
    assert result.returncode == 1
    assert "rm_movep" in (result.stdout + result.stderr)


def test_forbidden_detects_conflicting_allowlist(tmp_path, lock):
    """负例：同一接口同时在禁用与白名单 → 必须非 0。"""
    broken = dict(lock)
    broken["allowed_apis"] = list(lock["allowed_apis"]) + ["rm_movej"]
    target = _write_lock(tmp_path, broken)
    result = run_verify("--forbidden", "--lock", str(target))
    assert result.returncode == 1
    assert "rm_movej" in (result.stdout + result.stderr)


# --------------------------------------------------------------------------- #
# --wheels / --package-map
# --------------------------------------------------------------------------- #
def test_wheels_missing_archive_reports_skip_not_crash(tmp_path):
    """归档目录不存在时给出明确提示，而不是崩溃。"""
    result = run_verify("--wheels", "--wheels-dir", str(tmp_path / "no-archive"))
    assert result.returncode == 0
    assert "SKIP" in result.stdout
    assert "lock_deps.sh" in result.stdout


def test_wheels_detects_tampered_archive(tmp_path):
    """负例：归档文件被改 → 必须非 0。"""
    archive = tmp_path / "wheels"
    archive.mkdir()
    wheel = archive / "pkg-1.0-py3-none-any.whl"
    wheel.write_bytes(b"original")
    import hashlib

    sha = hashlib.sha256(b"original").hexdigest()
    (archive / "SHA256SUMS").write_text(f"{sha}  {wheel.name}\n", encoding="utf-8")
    wheel.write_bytes(b"tampered")
    result = run_verify("--wheels", "--wheels-dir", str(archive))
    assert result.returncode == 1
    assert "sha256" in (result.stdout + result.stderr)


def test_package_map_smoke():
    pytest.importorskip("pydrake")
    result = run_verify("--package-map")
    assert result.returncode == 0, result.stdout + result.stderr
    assert "package://rm75_drake/placeholder.urdf" in result.stdout


# --------------------------------------------------------------------------- #
# --model（模型自检，S01-T06）
# --------------------------------------------------------------------------- #
def test_model_check_exits_zero():
    """模型自检在真实模型上必须退出 0，并打印 DoF / 质量 / 零位高度 / 7 条限位。"""
    pytest.importorskip("pydrake")
    result = run_verify("--model")
    assert result.returncode == 0, result.stdout + result.stderr
    assert "DoF=7" in result.stdout
    assert "7.776" in result.stdout
    assert "0.8505" in result.stdout
    for index in range(1, 8):
        assert f"joint_{index}" in result.stdout


def test_model_check_json_summary_is_parseable():
    pytest.importorskip("pydrake")
    result = run_verify("--model", "--json")
    assert result.returncode == 0, result.stdout + result.stderr
    payload = json.loads(result.stdout)
    assert payload["ok"] is True
    check = payload["checks"]["model"]
    assert check["status"] == "PASS"
    assert check["summary"]["dof"] == 7
    assert len(check["summary"]["joint_limits"]) == 7


def test_model_check_detects_tampered_limits_handoff(tmp_path):
    """负例：限位交接件被改坏 → --model 必须失败（退出码 1），不得假绿灯。"""
    pytest.importorskip("pydrake")
    data = yaml.safe_load(LIMITS_PATH.read_text(encoding="utf-8"))
    data["joints"][0]["software"]["position_upper_rad"] = data["joints"][0]["hardware"][
        "position_upper_rad"
    ]
    target = tmp_path / "joint_limits.yaml"
    target.write_text(yaml.safe_dump(data, allow_unicode=True), encoding="utf-8")
    result = run_verify("--model", "--joint-limits", str(target))
    assert result.returncode == 1
    assert "position_upper_rad" in result.stdout


def test_model_check_reports_environment_error_when_drake_is_missing():
    """负例：Drake 不可用 → 状态 ERROR（环境错误，退出码 2），不得伪装成通过。"""
    def boom():
        raise ImportError("No module named 'pydrake'")

    status, problems, _summary = verify_env.check_model(loader=boom)
    assert status == "ERROR"
    assert problems


def test_model_check_reports_failure_when_model_cannot_be_loaded():
    """负例：模型加载失败 → 状态 FAIL（检查失败），不得静默通过。"""
    def boom():
        raise RuntimeError("URDF 解析失败")

    status, problems, _summary = verify_env.check_model(loader=boom)
    assert status == "FAIL"
    assert problems


def test_model_check_detects_drifted_joint_limits(tmp_path):
    """负例：URDF 限位与交接件漂移 → 必须判红。"""
    pytest.importorskip("pydrake")
    data = yaml.safe_load(LIMITS_PATH.read_text(encoding="utf-8"))
    data["joints"][1]["hardware"]["velocity_rad_s"] = 3.14
    target = tmp_path / "joint_limits.yaml"
    target.write_text(yaml.safe_dump(data, allow_unicode=True), encoding="utf-8")
    result = run_verify("--model", "--joint-limits", str(target))
    assert result.returncode == 1
    assert "joint_2" in result.stdout


# --------------------------------------------------------------------------- #
# setup_env.sh
# --------------------------------------------------------------------------- #
def test_setup_env_shell_syntax_ok():
    result = subprocess.run(["bash", "-n", str(SETUP)], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr


def test_setup_env_help_lists_all_options():
    result = run_setup("--help")
    assert result.returncode == 0
    for option in ("--offline", "--venv", "--profile", "--dry-run"):
        assert option in result.stdout


def test_setup_env_dry_run_has_no_side_effects(tmp_path):
    """--dry-run 不得创建 venv，也不得安装任何东西。"""
    venv_dir = tmp_path / "venv"
    profile = "py312" if sys.version_info[:2] == (3, 12) else "py310"
    result = run_setup("--dry-run", "--venv", str(venv_dir), "--profile", profile)
    assert result.returncode == 0, result.stdout + result.stderr
    assert not venv_dir.exists(), "--dry-run 不允许创建 venv"
    # 计划中的安装命令必须被打印出来（命令形态随「venv 是否自带 pip」而变）
    assert "pip" in result.stdout
    assert "install" in result.stdout
    assert "--requirement" in result.stdout


def test_setup_env_offline_without_archive_fails_clearly(tmp_path):
    """负例：离线模式但镜像缺失 → 明确报错，不静默继续。"""
    result = run_setup(
        "--offline",
        "--venv",
        str(tmp_path / "venv"),
        "--profile",
        "py310",
        env={"WHEELS_DIR": str(tmp_path / "missing-archive")},
    )
    assert result.returncode == 2
    assert "lock_deps.sh" in (result.stdout + result.stderr)


def test_setup_env_rejects_unknown_profile():
    result = run_setup("--profile", "py999")
    assert result.returncode == 2
    assert "py312" in (result.stdout + result.stderr)
