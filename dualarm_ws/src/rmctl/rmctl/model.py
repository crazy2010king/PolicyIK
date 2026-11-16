"""模型层：加载自研 RM75 模型，提供 FK/IK/限值/状态接口。"""
import os

import numpy as np
import yaml

from . import paths
from .selfkin import UrdfModel
from .vendor_env import VendorUnavailable, ensure_libstdcxx

# 厂商 nanobind wheel 需要 GLIBCXX_3.4.31+：先预加载仓库自带的
# .deps/libstdcxx（只读引用），再 import 厂商库。
ensure_libstdcxx()


def _locator():
    """Tesseract 0.35 的 GeneralResourceLocator 不支持自定义搜索路径；
    我们的模型在 build/model 里已把 mesh 路径物化成绝对路径，因此用默认 locator 即可。"""
    from tesseract_robotics.tesseract_common import GeneralResourceLocator
    return GeneralResourceLocator()


class Model(object):
    """双臂工程里的"单臂模型"。后续加第二臂时改为 MultiArmModel。"""

    def __init__(self, ws=None, group="rm75_arm", tip="Link7", tcp="flange"):
        self.ws = ws or paths.workspace_root()
        self.group = group
        self.tip = tip
        self.tcp = tcp
        self.urdf_path = paths.urdf_path(self.ws)
        self.srdf_path = paths.srdf_path(self.ws)
        self.urdf = UrdfModel(self.urdf_path)
        try:
            from tesseract_robotics.planning import Robot
        except ImportError as exc:
            # 厂商规划内核不可用（如裸环境缺 GLIBCXX_3.4.31+）：
            # 抛结构化异常，交由 CLI 走「NEEDS_HUMAN + 可粘贴 next_action」，
            # 绝不吞异常、更不伪造模型能力。
            raise VendorUnavailable(
                "厂商规划内核不可用：%s（需要 GLIBCXX_3.4.31+ 的 libstdc++，"
                "仓库自带 .deps/libstdcxx 可用）" % exc) from exc
        try:
            self.robot = Robot.from_files(self.urdf_path, self.srdf_path, _locator())
        except Exception as exc:
            # 厂商内核能 import，但模型加载失败（最常见：新 worktree 还没跑
            # `rmctl init` 物化 build/model 的**绝对路径** URDF，Tesseract 的
            # 资源定位器不认源码里的相对 mesh 路径）。
            # 这里**不抛裸异常**：统一转成 VendorUnavailable，让 CLI 走
            # 「退出码 3 + reason + 可粘贴 next_action」的诚实降级路径，
            # 绝不伪造模型/执行能力。
            raise VendorUnavailable(
                "厂商规划内核加载模型失败：%s（若提示 mesh/URDF 解析失败，"
                "先执行 `python3 -m rmctl init` 物化 build/model 绝对路径模型；"
                "若提示 GLIBCXX，先 `source scripts/rmenv.sh`）" % exc) from exc
        self.joint_names = list(self.robot.get_joint_names(group))
        self.link_names = list(self.robot.get_link_names())
        self.limits = self._load_limits()

    # ---------- 限值 ----------
    def _load_limits(self):
        p = paths.joint_limits_path(self.ws)
        if not os.path.exists(p):
            return {}
        with open(p, "r", encoding="utf-8") as fh:
            data = yaml.safe_load(fh) or {}
        return data.get(self.group, {})

    def limit(self, joint):
        return self.limits.get(joint, {})

    # ---------- 状态 ----------
    def set_state(self, q):
        q = np.asarray(q, dtype=float).reshape(-1)
        self.robot.set_joints(dict(zip(self.joint_names, [float(x) for x in q])), joint_names=self.joint_names)
        return q

    def get_state(self):
        st = self.robot.get_state()
        return np.asarray(st.joint_positions, dtype=float)

    # ---------- 运动学 ----------
    def fk_self(self, q, tip=None):
        """自研 FK（numpy）→ 4x4"""
        return self.urdf.fk(np.asarray(q, dtype=float).reshape(-1), tip or self.tip)

    def fk_tesseract(self, q, tip=None):
        """Tesseract FK → (position, quaternion)"""
        q = np.asarray(q, dtype=float).reshape(-1)
        pose = self.robot.fk(self.group, q, tip or self.tip)
        return pose

    def ik(self, pose, seed=None, tip=None):
        seed = self.get_state() if seed is None else np.asarray(seed, dtype=float)
        sol = self.robot.ik(self.group, pose, seed=seed, tip_link=tip or self.tip)
        return None if sol is None else np.asarray(sol, dtype=float)

    # ---------- 便捷 ----------
    @property
    def env(self):
        return self.robot.env

    def home_state(self):
        return np.zeros(len(self.joint_names))
