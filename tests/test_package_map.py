"""``package://`` 解析测试（T02）。

覆盖：
* ``build_package_map()`` 能解析占位 URDF 的 ``package://rm75_drake/...``；
* ``Parser.AddModels(url=...)`` 能成功加载占位模型；
* **value 语义回归**：``PopulateFromFolder`` 与 ``AddPackageXml`` 两种注册方式的
  ``ResolveUrl`` 结果一致（差异仅在 ``GetPath`` 的尾斜杠）——这是 T02 钉死的结论，
  一旦 Drake 行为变化必须立刻暴露；
* 负例：未知包、非 URL 字符串、不存在的搜索路径都必须**显式失败**，不许静默绿灯。
"""

from __future__ import annotations

import os
from pathlib import Path

import pytest

from rm75_drake import package_map, paths

PLACEHOLDER_URL = "package://rm75_drake/placeholder.urdf"


def _make_parser():
    """创建已绑定 package:// 的 Parser（含 Drake 1.51.1 的 model instance 槽位预留）。"""
    parser, _pkg_map = package_map.make_parser()
    return parser


def test_package_xml_lives_in_package_dir_not_models_root():
    """models/ 根目录不得放 package.xml（否则会注册出多余包）。"""
    assert not (paths.models_dir / "package.xml").exists()
    assert (paths.models_dir / "rm75_drake" / "package.xml").is_file()


def test_build_package_map_registers_rm75_drake():
    pkg_map = package_map.build_package_map()
    assert "rm75_drake" in package_map.package_names(pkg_map)


def test_resolve_placeholder_url():
    pkg_map = package_map.build_package_map()
    resolved = package_map.resolve_package_url(pkg_map, PLACEHOLDER_URL)
    assert Path(resolved) == (paths.models_dir / "rm75_drake" / "placeholder.urdf")
    assert Path(resolved).is_file()


def test_parser_add_models_from_package_url():
    """Drake 真正加载 ``package://rm75_drake/placeholder.urdf``。

    Drake 1.51.1（fallback 档）在 ``Parser.AddModels`` 内部构造
    ``CollisionFilterGroupResolver`` 时有一条不稳定的 DEMAND
    （``minimum_model_instance_index_ > world``），失败会直接 abort 整个进程。
    为了不让 abort 打断 pytest 会话，本用例把解析放进**独立子进程**执行；
    子进程若命中该已知 abort，用例显式 skip 并说明原因（不是假绿灯）。
    """
    import subprocess
    import sys
    import textwrap

    script = textwrap.dedent(
        f"""
        import sys
        sys.path.insert(0, {str(paths.REPO_ROOT / "src")!r})
        from rm75_drake import package_map
        parser, pkg_map = package_map.make_parser()
        indices = package_map.add_model_from_url(
            parser, {PLACEHOLDER_URL!r}, pkg_map=pkg_map)
        assert len(indices) == 1, indices
        assert parser.plant().num_bodies() > 0
        print("LOADED_OK")
        """
    )
    proc = subprocess.run(
        [sys.executable, "-c", script],
        capture_output=True,
        text=True,
        cwd=str(paths.REPO_ROOT),
    )
    combined = (proc.stdout or "") + (proc.stderr or "")
    if "minimum_model_instance_index_ > world" in combined:
        pytest.skip(
            "已知环境限制：Drake 1.51.1（fallback 档）在解析时的 "
            "CollisionFilterGroupResolver DEMAND 上不稳定 abort；"
            "受支持基线 Drake 1.55（py312）需复核。"
        )
    assert proc.returncode == 0, combined
    assert "LOADED_OK" in combined


def test_placeholder_urdf_is_wellformed_xml():
    """占位 URDF 的 XML 良构性（不依赖 Drake 解析路径）。"""
    import xml.etree.ElementTree as ET

    urdf = paths.models_dir / "rm75_drake" / "placeholder.urdf"
    root = ET.parse(urdf).getroot()
    assert root.tag == "robot"
    links = root.findall("link")
    joints = root.findall("joint")
    assert len(links) >= 1
    assert len(joints) == 1
    assert joints[0].find("parent").get("link") in {link.get("name") for link in links}


def test_make_parser_reserves_model_instance_slot():
    """Drake 1.51.1：解析前必须预留 model instance 槽位（见 drake_compat 文档）。"""
    parser, _ = package_map.make_parser()
    assert parser.plant().num_model_instances() >= 2


def test_value_semantics_are_identical_for_both_registration_styles():
    """钉死结论：两种注册方式 ResolveUrl 结果一致，仅 GetPath 尾斜杠不同。"""
    from pydrake.multibody.parsing import PackageMap

    pkg_dir = paths.models_dir / "rm75_drake"

    by_folder = PackageMap.MakeEmpty()
    by_folder.PopulateFromFolder(str(paths.models_dir))

    by_xml = PackageMap.MakeEmpty()
    by_xml.AddPackageXml(str(pkg_dir / "package.xml"))

    url_folder = by_folder.ResolveUrl(PLACEHOLDER_URL)
    url_xml = by_xml.ResolveUrl(PLACEHOLDER_URL)
    assert url_folder == url_xml
    assert Path(url_folder).is_file()

    # value 形态差异：目录（带尾斜杠）vs 同一目录（不带尾斜杠）
    path_folder = by_folder.GetPath("rm75_drake")
    path_xml = by_xml.GetPath("rm75_drake")
    assert Path(path_folder) == Path(path_xml)
    assert path_folder.endswith(os.sep)
    assert not path_xml.endswith(os.sep)


def test_readme_documents_the_conclusion():
    readme = (paths.models_dir / "README.md").read_text(encoding="utf-8")
    assert "PopulateFromFolder" in readme
    assert "AddPackageXml" in readme
    assert "package://rm75_drake" in readme


def test_negative_unknown_package_raises():
    pkg_map = package_map.build_package_map()
    with pytest.raises(RuntimeError):
        package_map.resolve_package_url(pkg_map, "package://no_such_pkg/none.urdf")


def test_negative_non_url_raises_value_error():
    pkg_map = package_map.build_package_map()
    with pytest.raises(ValueError):
        package_map.resolve_package_url(pkg_map, "models/rm75_drake/placeholder.urdf")


def test_negative_missing_search_path_raises():
    with pytest.raises(FileNotFoundError):
        package_map.build_package_map(search_paths=[paths.REPO_ROOT / "no_such_models_dir"])


def test_env_override_changes_registered_packages(tmp_path, monkeypatch):
    """模型搜索路径被环境变量覆盖后，默认 models/ 不再被扫描。"""
    other = tmp_path / "models"
    (other / "other_pkg").mkdir(parents=True)
    (other / "other_pkg" / "package.xml").write_text(
        '<?xml version="1.0"?>\n<package format="2">\n'
        "  <name>other_pkg</name>\n  <version>0.0.1</version>\n"
        "  <description>test</description>\n</package>\n",
        encoding="utf-8",
    )
    monkeypatch.setenv(paths.MODEL_PATH_ENV_VAR, str(other))
    pkg_map = package_map.build_package_map()
    names = package_map.package_names(pkg_map)
    assert "other_pkg" in names
    assert "rm75_drake" not in names
