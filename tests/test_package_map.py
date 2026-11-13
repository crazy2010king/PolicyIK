"""``package://`` 解析测试（T02）。

覆盖：
* ``build_package_map()`` 能解析占位 URDF 的 ``package://rm75_drake/...``；
* ``Parser.AddModels(url=...)`` 能**真实加载**占位模型——加载失败一律判红，
  **不允许**用 ``pytest.skip`` 把 abort/失败藏起来（T02 返工硬性要求）；
* **不预预留** model instance 槽位（回归守卫：预预留会确定性触发 Drake 的
  ``CollisionFilterGroupResolver`` DEMAND abort）；
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
    """创建已绑定 package:// 的 Parser（不预预留 model instance 槽位）。"""
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
    """Drake 真正加载 ``package://rm75_drake/placeholder.urdf``（失败即判红）。

    加载放在**独立子进程**里执行：Drake 的 ``CollisionFilterGroupResolver``
    DEMAND 失败时会直接 abort 进程，放在子进程里可以给出可读的失败信息，
    而不是打挂整个 pytest 会话。

    **T02 返工硬性要求**：本用例**不得**在加载失败时 ``pytest.skip``——
    失败必须让用例变红（基线 Drake 1.55.0 与降级 Drake 1.51.1 均实测通过）。
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
    assert proc.returncode == 0, (
        f"package:// 加载失败（退出码 {proc.returncode}）：\n{combined}"
    )
    assert "LOADED_OK" in combined, combined


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


def test_make_parser_does_not_prereserve_model_instances():
    """回归守卫：解析前**不得**预预留 model instance 槽位。

    预预留（``plant.AddModelInstance(...)``）会把 ``num_model_instances()`` 抬高，
    反而**确定性**触发 Drake 的 ``CollisionFilterGroupResolver`` DEMAND
    （``minimum_model_instance_index_ > world``）并 abort 进程。
    因此 ``make_parser()`` 创建的 plant 上不得出现任何本项目自造的槽位，
    实例数必须与新建的空 plant 一致（world + default 两个）。
    """
    from pydrake.multibody.plant import MultibodyPlant

    parser, _ = package_map.make_parser()
    plant = parser.plant()
    baseline = MultibodyPlant(0.0).num_model_instances()
    assert plant.num_model_instances() == baseline, (
        f"解析前不应预预留 model instance，实际 {plant.num_model_instances()}"
        f"（新建空 plant 为 {baseline}）"
    )


def test_make_parser_keeps_plant_alive():
    """回归守卫：Drake ``Parser`` 不保活 plant，``make_parser()`` 必须代为保活。

    Drake 的 C++ ``Parser`` 只持有 ``MultibodyPlant*`` 裸指针。若 plant 在
    ``make_parser()`` 的局部作用域里被 GC 销毁，parser 就悬垂，后续
    ``AddModels()`` 会直接 SIGSEGV（而不是抛可读的异常）。
    """
    import gc

    parser, _ = package_map.make_parser()
    gc.collect()
    plant = package_map.plant_for_parser(parser)
    assert plant is not None, "make_parser() 未登记 plant（会导致 parser 悬垂）"
    assert parser.plant() is not None
    # plant 必须仍然可用：加入一个 model instance 不应崩溃
    plant.AddModelInstance("rm75_drake_test_slot")
    assert plant.num_model_instances() >= 3


def test_package_map_module_does_not_expose_prepare_plant_for_parsing():
    """``prepare_plant_for_parsing`` 已删除（它是自引入缺陷的变通）。"""
    from rm75_drake import drake_compat

    assert not hasattr(drake_compat, "prepare_plant_for_parsing")
    assert "prepare_plant_for_parsing" not in drake_compat.__all__


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
