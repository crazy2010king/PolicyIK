"""scripts/rmenv.sh 的环境自举契约测试。

覆盖三条路径：
  1) 本 checkout 自带 .deps/libstdcxx → 直接采用；
  2) 本地缺失但同一仓库其它 checkout 有（worktree 布局 ../../ 或 ../）→ 只读回退；
  3) 全部缺失 → 打印明确警告而不是静默失败（且不使 shell 退出失败）。
"""
import os
import shutil
import subprocess

import pytest

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
RMENV = os.path.join(REPO_ROOT, "scripts", "rmenv.sh")


def _source(script_path, cwd=None):
    """在一个干净的 bash 里 source 脚本，返回 (rc, stdout, stderr)。"""
    env = dict(os.environ)
    env.pop("LD_LIBRARY_PATH", None)
    proc = subprocess.run(
        ["bash", "-c", "source '%s' >/dev/null" % script_path],
        cwd=cwd or REPO_ROOT, env=env,
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
    )
    return proc.returncode, proc.stdout, proc.stderr


def _probe(script_path, cwd):
    """source 之后把关键变量打印出来。"""
    cmd = ("source '" + script_path + "' >/dev/null 2>&1; "
           "printf 'DIR=%s\\n' \"$RM_LIBSTDCXX_DIR\"; "
           "printf 'LD=%s\\n' \"$LD_LIBRARY_PATH\"")
    proc = subprocess.run(["bash", "-c", cmd], cwd=cwd, stdout=subprocess.PIPE,
                          stderr=subprocess.PIPE, text=True)
    out = dict(line.split("=", 1) for line in proc.stdout.splitlines() if "=" in line)
    return proc.returncode, out.get("DIR", ""), out.get("LD", "")


def _make_deps(base):
    """造一个假的 .deps/libstdcxx（含 GLIBCXX_3.4.31 标记串，便于 strings 校验）。"""
    d = os.path.join(base, ".deps", "libstdcxx")
    os.makedirs(d)
    with open(os.path.join(d, "libstdc++.so.6"), "w", encoding="utf-8") as fh:
        fh.write("GLIBCXX_3.4.31\n")
    return d


@pytest.mark.skipif(not os.path.exists(RMENV), reason="scripts/rmenv.sh 不存在")
def test_rmenv_sources_cleanly():
    rc, _, stderr = _source(RMENV)
    assert rc == 0, stderr
    assert "警告" not in stderr, "本机不应出现 libstdcxx 警告: %s" % stderr


@pytest.mark.skipif(not os.path.exists(RMENV), reason="scripts/rmenv.sh 不存在")
def test_rmenv_exports_usable_libstdcxx():
    rc, lib_dir, ld = _probe(RMENV, REPO_ROOT)
    assert rc == 0
    if lib_dir:
        assert os.path.isdir(lib_dir)
        assert any(n.startswith("libstdc++.so.6") for n in os.listdir(lib_dir))
        assert lib_dir in ld
    else:
        # 探测不到时必须有明确警告（不能静默失败）
        rc2, _, stderr = _source(RMENV)
        assert rc2 == 0
        assert "警告" in stderr and "libstdcxx" in stderr


@pytest.mark.skipif(not os.path.exists(RMENV), reason="scripts/rmenv.sh 不存在")
def test_rmenv_falls_back_to_sibling_checkout(tmp_path):
    """worktree 布局：<root>/.gsd-worktrees/<id>/scripts/rmenv.sh → 命中 ../../.deps。"""
    wt = tmp_path / "repo" / ".gsd-worktrees" / "M001-test"
    (wt / "scripts").mkdir(parents=True)
    shutil.copyfile(RMENV, str(wt / "scripts" / "rmenv.sh"))
    deps = _make_deps(str(tmp_path / "repo"))

    rc, lib_dir, ld = _probe(str(wt / "scripts" / "rmenv.sh"), str(wt))
    assert rc == 0
    assert os.path.normpath(os.path.abspath(lib_dir)) == os.path.normpath(os.path.abspath(deps))
    ld_entries = [os.path.normpath(p) for p in ld.split(":") if p]
    assert os.path.normpath(deps) in ld_entries


@pytest.mark.skipif(not os.path.exists(RMENV), reason="scripts/rmenv.sh 不存在")
def test_rmenv_warns_when_no_libstdcxx_found(tmp_path):
    """负例：任何候选都找不到时 → 警告 + 退出码 0，不静默失败。"""
    wt = tmp_path / "solo" / "ws"
    (wt / "scripts").mkdir(parents=True)
    shutil.copyfile(RMENV, str(wt / "scripts" / "rmenv.sh"))

    rc, lib_dir, _ = _probe(str(wt / "scripts" / "rmenv.sh"), str(wt))
    assert rc == 0
    assert lib_dir == ""

    proc = subprocess.run(["bash", "-c", "source '%s/scripts/rmenv.sh'" % str(wt)],
                          cwd=str(wt), stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    assert proc.returncode == 0
    assert "警告" in proc.stderr
    assert "libstdcxx" in proc.stderr
    assert "GLIBCXX_3.4.31" in proc.stderr
