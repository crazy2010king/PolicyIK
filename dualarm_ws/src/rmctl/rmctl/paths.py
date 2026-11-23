"""工作区路径解析（rmctl 的"零配置"基础）。"""
import os

ENV_WS = "RM_WS"


def _find_src(ws):
    """在给定目录下找 package 源码根（含 rm_description）。"""
    for cand in (os.path.join(ws, "src"), os.path.join(ws, "dualarm_ws", "src")):
        if os.path.isdir(os.path.join(cand, "rm_description")):
            return cand
    return None


def _is_repo_root(d):
    """仓库根布局：同时含 dualarm_ws/src/rm_description 与 workcell/。

    工位（workcell/）与 build/model 都挂在仓库根，不在 dualarm_ws/ 下——
    路径解析必须认这一层，否则 workcell_dir() 会落到 dualarm_ws/workcell。
    """
    return (os.path.isdir(os.path.join(d, "dualarm_ws", "src", "rm_description"))
            and os.path.isdir(os.path.join(d, "workcell")))


def workspace_root(start=None):
    """向上查找工作区根；也可用 RM_WS 环境变量直接指定。"""
    env = os.environ.get(ENV_WS)
    if env and _find_src(env):
        return os.path.abspath(env)
    here = os.path.abspath(start or os.path.dirname(__file__))
    # 优先仓库根（workcell/ 与 build/ 都在这一层）
    cur = here
    while True:
        if _is_repo_root(cur):
            return cur
        parent = os.path.dirname(cur)
        if parent == cur:
            break
        cur = parent
    # 回退：把 dualarm_ws/ 本身当作工作区根（旧布局）
    cur = here
    while True:
        if _find_src(cur):
            return cur
        parent = os.path.dirname(cur)
        if parent == cur:
            break
        cur = parent
    return os.path.abspath(os.path.join(here, "..", "..", ".."))


def package_src(ws=None):
    ws = ws or workspace_root()
    src = _find_src(ws)
    return src or os.path.join(ws, "src")


def description_dir(ws=None):
    return os.path.join(package_src(ws), "rm_description")


def build_model_dir(ws=None):
    return os.path.join(ws or workspace_root(), "build", "model")


def urdf_path(ws=None):
    """加载用 URDF：优先 build/model（绝对路径，可直接被 Tesseract 加载）。"""
    built = os.path.join(build_model_dir(ws), "rm75_clean.urdf")
    if os.path.exists(built):
        return built
    return os.path.join(description_dir(ws), "urdf", "rm75_clean.urdf")


def srdf_path(ws=None):
    built = os.path.join(build_model_dir(ws), "rm75.srdf")
    if os.path.exists(built):
        return built
    return os.path.join(description_dir(ws), "srdf", "rm75.srdf")


def joint_limits_path(ws=None):
    built = os.path.join(build_model_dir(ws), "joint_limits.yaml")
    if os.path.exists(built):
        return built
    return os.path.join(description_dir(ws), "config", "joint_limits.yaml")


def workcell_dir(ws=None):
    return os.path.join(ws or workspace_root(), "workcell")


def baselines_dir(ws=None):
    """四类真机基线目录：<ws>/workcell/baselines（git 跟踪，模式同 workcell_dir）。"""
    return os.path.join(workcell_dir(ws), "baselines")
