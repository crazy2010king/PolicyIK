"""自研运动学（numpy）：不依赖任何厂商解算接口。

对应设计文档 §5.3：模型自主 —— FK 由我们自己算，并与控制器/规划内核交叉校验。
"""
import math
import xml.etree.ElementTree as ET

import numpy as np


def rpy_to_matrix(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])


def axis_angle_matrix(axis, angle):
    axis = np.asarray(axis, dtype=float)
    n = np.linalg.norm(axis)
    if n < 1e-12:
        return np.eye(3)
    axis = axis / n
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * (K @ K)


def _f(*vals):
    return [float(v) for v in vals]


class UrdfModel(object):
    """极简 URDF 解析：只取 FK 与碰撞盒需要的字段。"""

    def __init__(self, path):
        self.path = path
        root = ET.parse(path).getroot()
        self.name = root.get("name")
        self.joints = []
        self.links = {}
        for j in root.findall("joint"):
            org = j.find("origin")
            ax = j.find("axis")
            lim = j.find("limit")
            self.joints.append({
                "name": j.get("name"), "type": j.get("type"),
                "parent": j.find("parent").get("link"), "child": j.find("child").get("link"),
                "xyz": _f(*(org.get("xyz").split() if org is not None and org.get("xyz") else [0, 0, 0])),
                "rpy": _f(*(org.get("rpy").split() if org is not None and org.get("rpy") else [0, 0, 0])),
                "axis": _f(*(ax.get("xyz").split() if ax is not None else [1, 0, 0])),
                "lower": float(lim.get("lower")) if lim is not None and lim.get("lower") else -math.pi,
                "upper": float(lim.get("upper")) if lim is not None and lim.get("upper") else math.pi,
                "velocity": float(lim.get("velocity")) if lim is not None and lim.get("velocity") else 3.14,
            })
        for l in root.findall("link"):
            box = None
            col = l.find("collision")
            if col is not None:
                geo = col.find("geometry")
                b = geo.find("box") if geo is not None else None
                if b is not None:
                    org = col.find("origin")
                    box = {
                        "xyz": _f(*(org.get("xyz").split() if org is not None and org.get("xyz") else [0, 0, 0])),
                        "rpy": _f(*(org.get("rpy").split() if org is not None and org.get("rpy") else [0, 0, 0])),
                        "size": _f(*b.get("size").split()),
                    }
            self.links[l.get("name")] = {"name": l.get("name"), "box": box}
        self.rev = [j for j in self.joints if j["type"] in ("revolute", "continuous")]
        self.parent_of = {j["child"]: j["parent"] for j in self.joints}

    def link_poses(self, q):
        """q: 按 self.rev 顺序的关节角（rad）→ {link: 4x4}"""
        qmap = {j["name"]: q[i] for i, j in enumerate(self.rev)}
        poses = {self.root_link(): np.eye(4)}
        pending = list(self.joints)
        for _ in range(len(self.joints) + 2):
            nxt = []
            for j in pending:
                if j["parent"] not in poses:
                    nxt.append(j)
                    continue
                T = np.eye(4)
                T[:3, :3] = rpy_to_matrix(j["rpy"])
                T[:3, 3] = j["xyz"]
                R = np.eye(4)
                if j["type"] in ("revolute", "continuous"):
                    R[:3, :3] = axis_angle_matrix(j["axis"], qmap[j["name"]])
                poses[j["child"]] = poses[j["parent"]] @ T @ R
            if not nxt:
                break
            pending = nxt
        return poses

    def root_link(self):
        children = {j["child"] for j in self.joints}
        for l in self.links:
            if l not in children:
                return l
        return "base_link"

    def fk(self, q, tip):
        poses = self.link_poses(q)
        return poses.get(tip, poses[self.root_link()])

    def world_aabb(self, q, link):
        poses = self.link_poses(q)
        box = self.links[link]["box"]
        if box is None:
            return None
        R = rpy_to_matrix(box["rpy"])
        h = [s / 2.0 for s in box["size"]]
        corners = np.array([[dx * h[0], dy * h[1], dz * h[2]]
                            for dx in (-1, 1) for dy in (-1, 1) for dz in (-1, 1)])
        pts = corners @ R.T + np.asarray(box["xyz"])
        P = poses[link]
        pts = pts @ P[:3, :3].T + P[:3, 3]
        return pts.min(axis=0), pts.max(axis=0)

    def collisions(self, q, margin=0.0):
        """自研保守碰撞检测：世界系 AABB 相交（少报不报 → 安全侧）"""
        poses = self.link_poses(q)
        boxes = {}
        for name, info in self.links.items():
            if info["box"] is None or name not in poses:
                continue
            boxes[name] = self.world_aabb(q, name)
        names = sorted(boxes)
        hits = []
        for i in range(len(names)):
            for k in range(i + 1, len(names)):
                a, b = names[i], names[k]
                if self._adjacent(a, b):
                    continue
                if aabb_overlap(boxes[a], boxes[b], margin):
                    hits.append((a, b))
        return hits

    def _adjacent(self, a, b):
        if self.parent_of.get(a) == b or self.parent_of.get(b) == a:
            return True
        # 祖父-孙子 也算结构相邻（永不碰撞）
        for x, y in ((a, b), (b, a)):
            p = self.parent_of.get(x)
            if p and self.parent_of.get(p) == y:
                return True
        return False


def aabb_overlap(a, b, margin=0.0):
    if a is None or b is None:
        return False
    return bool(np.all(a[1] + margin > b[0]) and np.all(b[1] + margin > a[0]))
