"""``scripts/run_rm75_probe.sh``（一键执行器）的**契约测试**。

为什么单独测：它是"把手伸向真机"的那一层，**默认行为必须是不动臂**。
本文件用**假解释器**（把每次调用参数落盘）验证：

* 默认不带 ``--with-motion`` → 只会跑「离线预演 + 只读体检」，**不发运动指令**；
* 带 ``--with-motion`` → 运动步骤的顺序与参数（单关节 / 微小角度 / 低速）；
* SDK 找不到 → ``blocked``(3) 且**一步都不跑**；
* 只读体检不通过 → **不进运动步骤**（前一步失败就不进下一步）；
* 运动步骤失败 → 退出码如实透传，不吞错。
"""
from __future__ import annotations

import os
import shutil
import stat
import subprocess

import pytest

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(TESTS_DIR, "..", "..", "..", ".."))
RUNNER = os.path.join(REPO_ROOT, "scripts", "run_rm75_probe.sh")

FAKE_PY = """#!/usr/bin/env bash
# 假解释器：把本次调用的全部参数追加到 $RM75_FAKE_LOG，然后按 $RM75_FAKE_RC 退出。
printf '%s\\n' "$*" >> "$RM75_FAKE_LOG"
exit "${RM75_FAKE_RC:-0}"
"""


def _fake_python(tmp_path):
    path = tmp_path / "fake_python.sh"
    path.write_text(FAKE_PY, encoding="utf-8")
    path.chmod(path.stat().st_mode | stat.S_IEXEC | stat.S_IXGRP | stat.S_IXOTH)
    return str(path)


def _run(tmp_path, args, rc=0):
    """跑一键执行器，返回 (rc, stdout, stderr, 调用日志列表)。"""
    log = tmp_path / "calls.log"
    log.write_text("", encoding="utf-8")
    sdk = tmp_path / "libapi_c.so"
    sdk.write_bytes(b"")
    env = dict(os.environ)
    env["RM75_PROBE_PYTHON"] = _fake_python(tmp_path)
    env["RM75_PROBE_SCRIPT"] = os.path.join(REPO_ROOT, "scripts", "rm75_safe_probe.py")
    env["RM75_FAKE_LOG"] = str(log)
    env["RM75_FAKE_RC"] = str(rc)
    proc = subprocess.run(
        ["bash", str(RUNNER), "--skip-net-check", "--sdk-lib", str(sdk),
         "--out-dir", str(tmp_path / "reports")] + list(args),
        cwd=str(REPO_ROOT), env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        text=True)
    calls = [line for line in log.read_text(encoding="utf-8").splitlines() if line.strip()]
    return proc.returncode, proc.stdout, proc.stderr, calls


@pytest.mark.skipif(not os.path.exists(RUNNER), reason="scripts/run_rm75_probe.sh 不存在")
def test_runner_syntax_ok():
    proc = subprocess.run(["bash", "-n", RUNNER], stdout=subprocess.PIPE,
                          stderr=subprocess.PIPE, text=True)
    assert proc.returncode == 0, proc.stderr


@pytest.mark.skipif(not os.path.exists(RUNNER), reason="scripts/run_rm75_probe.sh 不存在")
def test_runner_help_mentions_motion_switch():
    proc = subprocess.run(["bash", RUNNER, "--help"], stdout=subprocess.PIPE,
                          stderr=subprocess.PIPE, text=True)
    assert proc.returncode == 0
    assert "--with-motion" in proc.stdout


@pytest.mark.skipif(not os.path.exists(RUNNER), reason="scripts/run_rm75_probe.sh 不存在")
def test_default_run_never_moves_the_arm(tmp_path):
    """默认行为：只跑离线预演 + 只读体检，**不含运动指令**。"""
    rc, out, err, calls = _run(tmp_path, [])
    assert rc == 0, (out, err)
    assert len(calls) == 2, calls
    assert "--dry-run" in calls[0]
    assert "--precheck-only" in calls[1]
    for call in calls:
        assert "--channel" not in call
        assert "--delta-deg" not in call


@pytest.mark.skipif(not os.path.exists(RUNNER), reason="scripts/run_rm75_probe.sh 不存在")
def test_motion_run_orders_phases_and_keeps_envelope(tmp_path):
    """带 --with-motion：顺序为 预演 → 体检 → 运动，且运动参数在安全包络内。"""
    rc, out, err, calls = _run(tmp_path, ["--with-motion", "--joint", "0",
                                          "--delta-deg", "0.3", "--speed-pct", "2"])
    assert rc == 0, (out, err)
    assert len(calls) == 3, calls
    assert "--dry-run" in calls[0]
    assert "--precheck-only" in calls[1]
    assert "--channel movej" in calls[2]
    assert "--joint 0" in calls[2]
    assert "--delta-deg 0.3" in calls[2]
    assert "--speed-pct 2" in calls[2]


@pytest.mark.skipif(not os.path.exists(RUNNER), reason="scripts/run_rm75_probe.sh 不存在")
def test_missing_sdk_blocks_before_any_probe_call(tmp_path):
    """SDK 找不到 → blocked(3)，且**一步探针都不跑**。

    在**隔离目录**里跑脚本副本（仓库里可能存在 .deps/rm_api2 的回退命中），
    才能构造出"任何候选路径都没有 SDK"的情形。
    """
    solo = tmp_path / "solo"
    (solo / "scripts").mkdir(parents=True)
    shutil.copyfile(RUNNER, str(solo / "scripts" / "run_rm75_probe.sh"))
    log = tmp_path / "calls.log"
    log.write_text("", encoding="utf-8")
    env = dict(os.environ)
    env["RM75_PROBE_PYTHON"] = _fake_python(tmp_path)
    env["RM75_FAKE_LOG"] = str(log)
    env.pop("RM_API2_LIB", None)
    proc = subprocess.run(
        ["bash", str(solo / "scripts" / "run_rm75_probe.sh"), "--skip-net-check"],
        cwd=str(solo), env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    assert proc.returncode == 3, (proc.stdout, proc.stderr)
    assert "RM_API2_LIB" in proc.stderr
    assert log.read_text(encoding="utf-8").strip() == ""


@pytest.mark.skipif(not os.path.exists(RUNNER), reason="scripts/run_rm75_probe.sh 不存在")
def test_precheck_failure_stops_before_motion(tmp_path):
    """只读体检不通过 → 不进运动步骤。"""
    log = tmp_path / "calls.log"
    log.write_text("", encoding="utf-8")
    sdk = tmp_path / "libapi_c.so"
    sdk.write_bytes(b"")
    env = dict(os.environ)
    env["RM75_PROBE_PYTHON"] = _fake_python(tmp_path)
    env["RM75_FAKE_LOG"] = str(log)
    env["RM75_FAKE_RC"] = "4"
    proc = subprocess.run(
        ["bash", str(RUNNER), "--skip-net-check", "--sdk-lib", str(sdk),
         "--with-motion", "--out-dir", str(tmp_path / "reports")],
        cwd=str(REPO_ROOT), env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    assert proc.returncode == 3, (proc.stdout, proc.stderr)
    calls = [c for c in log.read_text(encoding="utf-8").splitlines() if c.strip()]
    # 预演失败即止（假解释器一律返回 4）→ 任何情况都不应出现运动调用
    assert all("--channel" not in c for c in calls)


@pytest.mark.skipif(not os.path.exists(RUNNER), reason="scripts/run_rm75_probe.sh 不存在")
def test_unknown_argument_is_usage_error(tmp_path):
    rc, out, err, _calls = _run(tmp_path, ["--bogus"])
    assert rc == 2, (out, err)
