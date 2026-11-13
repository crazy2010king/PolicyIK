"""工作区路径解析（rmctl 的"零配置"基础）。"""
import os

ENV_WS = "RM_WS"


def _find_src(ws):
    """在给定目录下找 package 源码根（含 rm_description）。"""
    for cand in (os.path.join(ws, "src"), os.path.join(ws, "dualarm_ws", "src")):
        if os.path.isdir(os.path.join(cand, "rm_description")):
            return cand
    return None


def workspace_root(start=None):
    """向上查找工作区根；也可用 RM_WS 环境变量直接指定。"""
    env = os.environ.get(ENV_WS)
    if env and _find_src(env):
        return os.path.abspath(env)
    here = os.path.abspath(start or os.path.dirname(__file__))
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
