"""构建入口守卫测试（只读）。

本仓库的硬性纪律：**零编译、规划内核无 ROS 依赖**。

- 仓库里不存在任何 ROS/colcon 构建入口（``package.xml`` / ``CMakeLists.txt``
  / ``setup.py`` / ``*.repos`` / ``COLCON_IGNORE``）——四包全部是纯 Python
  ``setuptools`` 打包，装依赖即可跑，不存在「编译才能用」的路径。
- 导入规划入口（``rmctl`` / ``rmctl.regression.*``）不会把 ``rclpy`` /
  ``ros2`` / ``ament_*`` / ``colcon`` 带进 ``sys.modules``——规划内核与 ROS
  运行时零耦合。

这两条纪律此前只写在文档里，没有任何测试守住它：将来有人加回 ``package.xml``
或让 ``rmctl`` 入口 import ``rclpy``，不会有任何用例报警。本文件把纪律变成
可执行断言。

设计约束（与本仓库既有测试一致）：

- **只读**：不写文件、不 import 触发厂商 SDK 装载、不连真机、不执行任何构建。
  仓库里不存在的构建方式被断言为「必须不存在」，而不是「去构建一次」。
- **不跟随符号链接**：``docs/code_doc`` 是指向仓库外的符号链接，扫描时必须
  跳过，否则会扫到仓库外的文件。
- **排除运行时/规划目录**：``.git`` / ``build`` / ``.deps`` / ``.gsd`` /
  ``.gsd-worktrees`` / ``.pytest_cache`` / ``__pycache__`` 是 gitignore 或
  运行时目录（``.gsd`` 下是规划投影文档，不是产品源码），不得被当成产品源码
  扫描。
- **路径从测试文件位置推导**：不硬编码仓库绝对路径，也不硬编码解释器路径。
"""
import json
import os
import subprocess
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(TESTS_DIR, "..", "..", "..", ".."))
WS_SRC = os.path.join(REPO_ROOT, "dualarm_ws", "src")

# 扫描时必须跳过的目录名（gitignore / 运行时 / 规划投影目录）。
EXCLUDED_DIRS = frozenset(
    {
        ".git",
        "build",
        ".deps",
        ".gsd",
        ".gsd-worktrees",
        ".pytest_cache",
        "__pycache__",
    }
)

# 任何一种出现都说明「ROS/colcon 构建入口」被加回来了。
FORBIDDEN_BUILD_ENTRIES = ("package.xml", "CMakeLists.txt", "setup.py", "COLCON_IGNORE")

# 导入规划入口后，``sys.modules`` 里不允许出现的 ROS 相关模块前缀。
FORBIDDEN_ROS_MODULE_PREFIXES = (
    "rclpy",
    "rclcpp",
    "ros2",
    "rosidl",
    "rospy",
    "rosgraph",
    "roslaunch",
    "ament_",
    "colcon",
    "catkin",
    "std_msgs",
    "geometry_msgs",
    "sensor_msgs",
    "builtin_interfaces",
    "trajectory_msgs",
    "action_msgs",
    "moveit",
    "tf2",
    "launch",
)

# ``pyproject.toml`` 里不允许出现的 ROS 构建后端/依赖键。
FORBIDDEN_PYPROJECT_KEYS = ("ament", "colcon", "catkin", "ros2", "rclpy")

# 纯 Python 打包后端（仓库唯一的构建/打包方式）。
EXPECTED_BUILD_BACKEND = "setuptools.build_meta"


def _is_forbidden_entry(name):
    """文件名是否是禁止的构建入口。"""
    if name in FORBIDDEN_BUILD_ENTRIES:
        return True
    return name.endswith(".repos")


def find_build_entries(root):
    """扫描 ``root``，返回其中所有禁止的构建入口（相对路径，排序）。

    - 跳过 ``EXCLUDED_DIRS`` 里的目录；
    - **不进入符号链接目录**（``docs/code_doc`` 指向仓库外）；
    - 符号链接**文件**若名字命中仍然报告（它确实出现在工作区里）。
    """
    found = []
    for dirpath, dirnames, filenames in os.walk(root, followlinks=False):
        # 就地裁剪：既跳过排除目录，也跳过指向仓库外的符号链接目录。
        kept = []
        for d in dirnames:
            if d in EXCLUDED_DIRS:
                continue
            full = os.path.join(dirpath, d)
            if os.path.islink(full):
                # 符号链接目录：不跟随（防止扫到仓库外）。
                continue
            kept.append(d)
        dirnames[:] = kept
        for fn in filenames:
            if _is_forbidden_entry(fn):
                found.append(os.path.relpath(os.path.join(dirpath, fn), root))
    return sorted(found)


def python_packages_under_ws_src(ws_src):
    """列出 ``ws_src`` 下所有「含可导入 Python 包」的目录名（排序）。"""
    names = []
    for entry in sorted(os.listdir(ws_src)):
        pkg_dir = os.path.join(ws_src, entry)
        if not os.path.isdir(pkg_dir) or os.path.islink(pkg_dir):
            continue
        for child in sorted(os.listdir(pkg_dir)):
            child_path = os.path.join(pkg_dir, child)
            if os.path.isdir(child_path) and os.path.isfile(
                os.path.join(child_path, "__init__.py")
            ):
                names.append(entry)
                break
    return names


# --------------------------------------------------------------------------
# a. 无 ROS/colcon 构建入口
# --------------------------------------------------------------------------


def test_repo_has_no_ros_or_colcon_build_entry():
    """仓库工作区内不得存在 package.xml / CMakeLists.txt / setup.py / *.repos / COLCON_IGNORE。"""
    offenders = find_build_entries(REPO_ROOT)
    assert offenders == [], (
        "仓库不允许出现 ROS/colcon 构建入口（零编译纪律）；发现：%s" % offenders
    )


def test_guard_detects_injected_package_xml(tmp_path):
    """负例：临时目录里造一个 package.xml，守卫必须抓到（不在仓库里留文件）。"""
    (tmp_path / "package.xml").write_text("<package/>\n", encoding="utf-8")
    offenders = find_build_entries(str(tmp_path))
    assert any(p.endswith("package.xml") for p in offenders), offenders


def test_guard_detects_all_forbidden_entry_names(tmp_path):
    """负例：CMakeLists.txt / setup.py / *.repos / COLCON_IGNORE 同样必须被抓到。"""
    for name in ("CMakeLists.txt", "setup.py", "my.repos", "COLCON_IGNORE"):
        (tmp_path / name).write_text("x\n", encoding="utf-8")
    offenders = find_build_entries(str(tmp_path))
    assert len(offenders) == 4, offenders
    for name in ("CMakeLists.txt", "setup.py", "my.repos", "COLCON_IGNORE"):
        assert any(p.endswith(name) for p in offenders), (name, offenders)


def test_guard_skips_excluded_and_symlinked_dirs(tmp_path, tmp_path_factory):
    """负例：排除目录与符号链接目录里的构建入口不得被误报。"""
    (tmp_path / "build").mkdir()
    (tmp_path / "build" / "package.xml").write_text("<package/>\n", encoding="utf-8")
    (tmp_path / ".gsd").mkdir()
    (tmp_path / ".gsd" / "CMakeLists.txt").write_text("x\n", encoding="utf-8")

    # 「仓库外」目录：必须在被扫描根之外（否则它本身就在扫描范围内）。
    outside = tmp_path_factory.mktemp("outside")
    (outside / "setup.py").write_text("x\n", encoding="utf-8")
    link = tmp_path / "code_doc"
    try:
        os.symlink(str(outside), str(link))
    except (OSError, NotImplementedError):  # pragma: no cover - 无符号链接权限的平台
        return

    assert find_build_entries(str(tmp_path)) == []


# --------------------------------------------------------------------------
# b. 零 ROS 运行时依赖
# --------------------------------------------------------------------------

_PROBE_SOURCE = r"""
import json
import os
import sys

for path in sys.argv[2:]:
    if path not in sys.path:
        sys.path.insert(0, path)

import rmctl
import rmctl.regression.segments
import rmctl.regression.orchestrator

# 负例探针：模拟「有人给规划入口加了 import rclpy」的坏情形。
extra = os.environ.get("PROBE_EXTRA_IMPORT", "")
if extra:
    __import__(extra)

prefixes = tuple(sys.argv[1].split("|")) if sys.argv[1] else ()
offenders = sorted(
    name
    for name in sys.modules
    if any(name == p or name.startswith(p + ".") for p in prefixes)
)
print("PROBE_RESULT " + json.dumps({"offenders": offenders}))
"""


def _run_import_probe(extra_import=None, extra_pythonpath=None):
    """在子进程里导入规划入口，返回被带入 ``sys.modules`` 的 ROS 模块名列表。

    子进程用 ``sys.executable``、``cwd`` 设为仓库根、``env`` 继承当前环境；
    ``sys.path`` 引导方式与 ``tests/conftest.py`` 一致（把四包源码目录加进去），
    不依赖当前解释器恰好装了什么。

    ``extra_import`` / ``extra_pythonpath`` 只供负例探针使用（模拟「有人给
    规划入口加了 import rclpy」的坏情形）。
    """
    src_dirs = [
        os.path.join(WS_SRC, name)
        for name in ("rmctl", "rm_hw_bridge", "rm_executor", "rm_safety")
    ]
    env = dict(os.environ)
    env.pop("PYTHONPATH", None)
    pypath_parts = list(src_dirs)
    if extra_pythonpath:
        pypath_parts.append(extra_pythonpath)
    env["PYTHONPATH"] = os.pathsep.join(pypath_parts)
    if extra_import:
        env["PROBE_EXTRA_IMPORT"] = extra_import
    proc = subprocess.run(
        [
            sys.executable,
            "-c",
            _PROBE_SOURCE,
            "|".join(FORBIDDEN_ROS_MODULE_PREFIXES),
        ]
        + src_dirs,
        cwd=REPO_ROOT,
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        universal_newlines=True,
        timeout=300,
    )
    if proc.returncode != 0:
        raise AssertionError(
            "导入探测子进程失败（rc=%s）\nstdout:\n%s\nstderr:\n%s"
            % (proc.returncode, proc.stdout, proc.stderr)
        )
    for line in proc.stdout.splitlines():
        if line.startswith("PROBE_RESULT "):
            return json.loads(line[len("PROBE_RESULT "):])["offenders"]
    raise AssertionError(
        "导入探测子进程没有产出 PROBE_RESULT 行\nstdout:\n%s\nstderr:\n%s"
        % (proc.stdout, proc.stderr)
    )


def test_importing_planning_entry_does_not_load_ros_modules():
    """import rmctl / rmctl.regression.* 不得把任何 ROS 模块带进 sys.modules。"""
    offenders = _run_import_probe()
    assert offenders == [], "规划入口引入了 ROS 相关模块：%s" % offenders


def test_import_probe_would_catch_an_injected_ros_import(tmp_path):
    """负例：探针本身不得是「永远通过」的空断言。

    造一个假的 ``rclpy`` 模块并让子进程导入它，探针必须把它报出来。
    """
    fake = tmp_path / "rclpy.py"
    fake.write_text("# fake ROS module for guard negative test\n", encoding="utf-8")
    offenders = _run_import_probe(extra_import="rclpy", extra_pythonpath=str(tmp_path))
    assert "rclpy" in offenders, offenders


# --------------------------------------------------------------------------
# c. 构建入口仍是纯 Python 打包
# --------------------------------------------------------------------------


def test_every_python_package_has_pyproject():
    """``dualarm_ws/src`` 下每个 Python 包都必须有 pyproject.toml。"""
    packages = python_packages_under_ws_src(WS_SRC)
    assert packages, "未发现任何 Python 包，扫描路径可能不对：%s" % WS_SRC
    missing = [
        name for name in packages if not os.path.isfile(os.path.join(WS_SRC, name, "pyproject.toml"))
    ]
    assert missing == [], "缺少 pyproject.toml 的包：%s" % missing


def test_pyproject_has_no_ros_build_backend_or_dependency():
    """pyproject.toml 不得出现 ament / colcon / catkin 等 ROS 构建后端或依赖键。"""
    packages = python_packages_under_ws_src(WS_SRC)
    offenders = {}
    for name in packages:
        path = os.path.join(WS_SRC, name, "pyproject.toml")
        text = open(path, encoding="utf-8").read()
        lowered = text.lower()
        hits = [k for k in FORBIDDEN_PYPROJECT_KEYS if k in lowered]
        if hits:
            offenders[name] = hits
    assert offenders == {}, "pyproject.toml 出现 ROS 相关键：%s" % offenders


def test_pyproject_build_backend_is_plain_setuptools():
    """唯一允许的构建后端是 setuptools（纯 Python，零编译）。"""
    packages = python_packages_under_ws_src(WS_SRC)
    bad = {}
    for name in packages:
        path = os.path.join(WS_SRC, name, "pyproject.toml")
        text = open(path, encoding="utf-8").read()
        backend = None
        for line in text.splitlines():
            if line.strip().startswith("build-backend"):
                backend = line.split("=", 1)[1].strip().strip('"').strip("'")
                break
        if backend != EXPECTED_BUILD_BACKEND:
            bad[name] = backend
    assert bad == {}, "非纯 Python 打包后端：%s" % bad
