"""厂商解算接口禁用名单静态扫描器（T05 / R008 / D003）的行为测试。

这些断言守住的契约：

* **正例**：当前仓库的 ``src/`` 不含任何禁用接口调用（扫描器退出码 0）；
* **负例**：临时写一个含禁用调用（``NAME(`` 形式）的文件，扫描器必须退出码 1
  并报出 ``文件:行号``——证明扫描器不是"永远绿灯"的假检查；
* **防前缀误伤**：``allowed_apis`` 里的白名单接口（形如 ``NAME_CANFD(``）必须
  **不**被判违规，否则这条规则一旦上线就会立刻被绕过或误杀；
* **唯一真相源**：扫描器不硬编码任何接口名，名单只来自 ``versions.lock``。

负例同样重要：缺名单的锁、名单与白名单冲突的锁都必须显式失败（退出码 2）。
"""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
SCANNER = REPO_ROOT / "scripts" / "scan_forbidden_apis.py"
LOCK_PATH = REPO_ROOT / "versions.lock"

#: 用于负例的"禁用接口调用"写法（名字取自锁文件，不在测试里另立名单）。
FORBIDDEN_CALL = "rm_movej("
#: 白名单接口的调用写法（必须在锁文件的 allowed_apis 里）。
ALLOWED_CALL = "rm_movej_canfd("
#: **变体名**调用写法（S07-T03）：静态扫描器按「精确名 + 左括号」匹配 ⇒
#: 变体名必须在锁文件里显式登记，否则静态这一证会漏检。
VARIANT_CALL = "rm_algo_inverse_kinematics_new("


def run_scanner(*args: str) -> subprocess.CompletedProcess:
    """以子进程方式调用扫描器（真实 CLI 行为）。"""
    return subprocess.run(
        [sys.executable, str(SCANNER), *args],
        cwd=str(REPO_ROOT),
        capture_output=True,
        text=True,
        timeout=120,
    )


@pytest.fixture(scope="module")
def lock() -> dict:
    data = yaml.safe_load(LOCK_PATH.read_text(encoding="utf-8"))
    assert isinstance(data, dict)
    return data


# --------------------------------------------------------------------------- #
# 正例
# --------------------------------------------------------------------------- #
def test_scanner_on_src_exits_zero():
    """正例：src/ 无禁用接口调用 → 退出码 0。"""
    result = run_scanner("src")
    assert result.returncode == 0, result.stdout + result.stderr


def test_scanner_default_paths_cover_src_tools_scripts():
    """默认扫描范围必须包含 src/、tools/、scripts/ 三个目录。"""
    result = run_scanner("--json")
    assert result.returncode == 0, result.stdout + result.stderr
    report = json.loads(result.stdout)
    joined = " ".join(report["scanned_paths"])
    for expected in ("src", "tools", "scripts"):
        assert expected in joined
    assert report["scanned_files"] > 0


def test_scanner_does_not_hardcode_api_names(lock):
    """名单唯一真相源：扫描器源码里不得出现锁文件中的接口名。"""
    text = SCANNER.read_text(encoding="utf-8")
    hardcoded = [
        name for name in lock["forbidden_apis"] if name in text
    ]
    assert not hardcoded, f"扫描器疑似硬编码禁用名单: {hardcoded}"


def test_scanner_reads_names_from_lock(lock):
    """扫描器读到的名单必须与锁文件一致（--json 输出可核对）。"""
    result = run_scanner("--json")
    report = json.loads(result.stdout)
    assert report["forbidden_apis"] == list(lock["forbidden_apis"])
    assert report["allowed_apis"] == list(lock["allowed_apis"])


# --------------------------------------------------------------------------- #
# 负例：证明扫描器不是"永远绿灯"
# --------------------------------------------------------------------------- #
def test_forbidden_call_is_reported_with_file_and_line(tmp_path):
    """负例：含禁用调用的临时文件必须被命中，且报出文件与行号。"""
    bad = tmp_path / "bad_usage.py"
    bad.write_text(
        "def drive():\n"          # 第 1 行
        "    x = 1\n"             # 第 2 行
        f"    {FORBIDDEN_CALL}arg)\n"  # 第 3 行 ← 违规
        "    return x\n",
        encoding="utf-8",
    )
    result = run_scanner(str(tmp_path))
    assert result.returncode == 1, result.stdout + result.stderr
    assert str(bad) in result.stdout
    assert ":3:" in result.stdout


def test_allowed_api_is_not_flagged(tmp_path):
    """负例（防前缀误伤）：白名单接口的调用不得被判违规。"""
    good = tmp_path / "good_usage.py"
    good.write_text(f"value = {ALLOWED_CALL}arg)\n", encoding="utf-8")
    result = run_scanner(str(tmp_path))
    assert result.returncode == 0, result.stdout + result.stderr
    assert "命中" not in result.stdout or "未命中禁用接口" in result.stdout


def test_longer_identifier_with_forbidden_prefix_is_not_flagged(tmp_path):
    """负例：`x_rm_movej(` 这类更长标识符不是调用点，不应误判。"""
    good = tmp_path / "prefixed.py"
    good.write_text("x_rm_movej(arg)\n", encoding="utf-8")
    result = run_scanner(str(tmp_path))
    assert result.returncode == 0, result.stdout + result.stderr


def test_mention_without_call_is_not_flagged(tmp_path):
    """负例：文档式提及（没有左括号）不算调用。"""
    doc = tmp_path / "notes.py"
    doc.write_text("# 曾经用过 rm_movej 这个接口，后来移除了\n", encoding="utf-8")
    result = run_scanner(str(tmp_path))
    assert result.returncode == 0, result.stdout + result.stderr


def test_markdown_files_are_not_scanned(tmp_path):
    """负例：.md 里的提及不算调用（文档提到接口名很常见）。"""
    doc = tmp_path / "README.md"
    doc.write_text(f"调用方式：{FORBIDDEN_CALL}arg)\n", encoding="utf-8")
    result = run_scanner(str(tmp_path))
    assert result.returncode == 0, result.stdout + result.stderr


def test_json_output_reports_hit_location(tmp_path):
    """负例：--json 输出必须带文件与行号，便于机器消费。"""
    bad = tmp_path / "bad_json.py"
    bad.write_text(f"{FORBIDDEN_CALL}arg)\n", encoding="utf-8")
    result = run_scanner("--json", str(tmp_path))
    assert result.returncode == 1
    report = json.loads(result.stdout)
    assert report["ok"] is False
    assert len(report["hits"]) == 1
    hit = report["hits"][0]
    assert hit["file"] == str(bad)
    assert hit["line"] == 1
    assert hit["api"] == FORBIDDEN_CALL.rstrip("(")


# --------------------------------------------------------------------------- #
# 负例：环境/输入错误必须显式失败
# --------------------------------------------------------------------------- #
def test_missing_lock_exits_two(tmp_path):
    """负例：锁文件不存在 → 退出码 2（运行期错误），不得静默通过。"""
    result = run_scanner("--lock", str(tmp_path / "nope.lock"), "src")
    assert result.returncode == 2
    assert "锁文件" in (result.stdout + result.stderr)


# --------------------------------------------------------------------------- #
# 负例 / 正例：变体名必须显式登记（否则静态扫描漏检）
# --------------------------------------------------------------------------- #
def test_variant_names_are_registered_in_lock(lock):
    """变体名必须在 forbidden_apis 里显式登记。"""
    assert VARIANT_CALL.rstrip("(") in lock["forbidden_apis"]


def test_registered_variant_call_is_reported(tmp_path):
    """负例：已登记的变体名调用必须被命中（静态这一证不漏检）。"""
    bad = tmp_path / "variant_usage.py"
    bad.write_text(f"value = {VARIANT_CALL}arg)\n", encoding="utf-8")
    result = run_scanner(str(tmp_path))
    assert result.returncode == 1, result.stdout + result.stderr
    assert str(bad) in result.stdout


def test_lock_without_forbidden_apis_exits_two(tmp_path):
    """负例：锁文件缺少 forbidden_apis → 退出码 2。"""
    bad_lock = tmp_path / "bad.lock"
    bad_lock.write_text(
        yaml.safe_dump({"schema_version": 1, "allowed_apis": ["x"]}),
        encoding="utf-8",
    )
    result = run_scanner("--lock", str(bad_lock), "src")
    assert result.returncode == 2
    assert "forbidden_apis" in (result.stdout + result.stderr)


def test_conflicting_forbidden_and_allowed_exits_two(tmp_path, lock):
    """负例：同一接口既禁用又允许 → 退出码 2（名单自相矛盾必须显式报错）。"""
    conflicting = dict(lock)
    conflicting["allowed_apis"] = list(lock["allowed_apis"]) + [
        lock["forbidden_apis"][0]
    ]
    bad_lock = tmp_path / "conflict.lock"
    bad_lock.write_text(yaml.safe_dump(conflicting), encoding="utf-8")
    result = run_scanner("--lock", str(bad_lock), "src")
    assert result.returncode == 2
    assert "allowed_apis" in (result.stdout + result.stderr)
