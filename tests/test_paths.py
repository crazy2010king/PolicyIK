"""路径层测试（T02）。

覆盖：
* 路径不依赖 cwd（切目录后仍正确）；
* ``RM75_DRAKE_MODEL_PATH`` 覆盖生效（单路径 / 多路径 / 非法值忽略）；
* 路径常量指向真实存在的目录；
* versions.lock 可读且 baseline 存在。
"""

from __future__ import annotations

import os
from pathlib import Path

import pytest
import yaml

from rm75_drake import paths


def test_repo_root_contains_known_markers():
    assert paths.REPO_ROOT.is_dir()
    assert (paths.REPO_ROOT / "src").is_dir()
    assert paths.versions_lock_path.is_file()
    assert paths.versions_lock_path == paths.REPO_ROOT / "versions.lock"


def test_known_dirs_exist():
    assert paths.models_dir.is_dir()
    assert paths.scenarios_dir.is_dir()
    # vendor/ 与 third_party/ 由 T03/T06 创建，此处只断言推导位置正确
    assert paths.vendor_dir == paths.REPO_ROOT / "vendor"
    assert paths.third_party_dir == paths.REPO_ROOT / "third_party"


def test_device_dirs_are_derived_from_repo_root():
    """设备层目录必须由 paths 统一推导（S03：禁止各模块各写一份路径推导）。"""
    assert paths.devices_dir == paths.REPO_ROOT / "devices"
    assert paths.devices_dir.is_dir()
    assert (paths.devices_dir / "single_arm.yaml").is_file()
    assert paths.run_logs_dir == paths.REPO_ROOT / "run_logs"
    # run_logs 目录由写日志时创建，这里只断言推导位置正确
    assert paths.run_logs_dir.parent == paths.REPO_ROOT


def test_paths_are_cwd_independent(tmp_path, monkeypatch):
    before = {
        "root": paths.REPO_ROOT,
        "models": paths.models_dir,
        "scenarios": paths.scenarios_dir,
        "devices": paths.devices_dir,
        "run_logs": paths.run_logs_dir,
        "lock": paths.versions_lock_path,
    }
    monkeypatch.chdir(tmp_path)
    assert Path.cwd() == tmp_path
    assert paths.REPO_ROOT == before["root"]
    assert paths.models_dir == before["models"]
    assert paths.scenarios_dir == before["scenarios"]
    assert paths.devices_dir == before["devices"]
    assert paths.run_logs_dir == before["run_logs"]
    assert paths.versions_lock_path == before["lock"]
    # 解析结果也必须与 cwd 无关
    assert paths.model_search_paths() == [before["models"]]


def test_model_path_env_override_single(monkeypatch, tmp_path):
    monkeypatch.setenv(paths.MODEL_PATH_ENV_VAR, str(tmp_path))
    assert paths.model_search_paths() == [tmp_path]


def test_model_path_env_override_multiple(monkeypatch, tmp_path):
    a = tmp_path / "a"
    b = tmp_path / "b"
    a.mkdir()
    b.mkdir()
    monkeypatch.setenv(paths.MODEL_PATH_ENV_VAR, f"{a}:{b}")
    assert paths.model_search_paths() == [a, b]


def test_model_path_env_override_ignores_empty_entries(monkeypatch, tmp_path):
    monkeypatch.setenv(paths.MODEL_PATH_ENV_VAR, f":{tmp_path}::")
    assert paths.model_search_paths() == [tmp_path]


def test_model_path_env_override_empty_falls_back(monkeypatch):
    monkeypatch.setenv(paths.MODEL_PATH_ENV_VAR, "")
    assert paths.model_search_paths() == [paths.models_dir]


def test_model_path_env_unset_falls_back(monkeypatch):
    monkeypatch.delenv(paths.MODEL_PATH_ENV_VAR, raising=False)
    assert paths.model_search_paths() == [paths.models_dir]


def test_versions_lock_parses_and_has_baseline():
    with paths.versions_lock_path.open("r", encoding="utf-8") as fh:
        lock = yaml.safe_load(fh)
    assert lock["baseline"]["drake"]
    assert lock["fallback"]["drake"]
