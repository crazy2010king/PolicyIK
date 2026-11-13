#!/usr/bin/env python3
"""生成 Tesseract 方言的 SRDF（含自研 ACM 采样）。

要点（对应设计 §1.3 / §5.2 / §5.13.3）：
  * 自研 FK（numpy），不依赖任何厂商解算
  * ACM 由采样生成：相邻连杆禁用；采样中始终不相交的连杆对禁用（保守判定：世界系 AABB，
    只会少禁用不会误禁用 —— 少禁用 = 更安全）
  * group / group_state / group_tcp / 插件配置一次写全
"""
import argparse
import json
import math
import os
import xml.etree.ElementTree as ET

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)


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


def load_urdf(path):
    root = ET.parse(path).getroot()
    joints, links = [], {}
    for j in root.findall("joint"):
        org = j.find("origin")
        xyz = [float(x) for x in (org.get("xyz").split() if org is not None and org.get("xyz") else [0, 0, 0])]
        rpy = [float(x) for x in (org.get("rpy").split() if org is not None and org.get("rpy") else [0, 0, 0])]
        ax = j.find("axis")
        lim = j.find("limit")
        joints.append({
            "name": j.get("name"), "type": j.get("type"),
            "parent": j.find("parent").get("link"), "child": j.find("child").get("link"),
            "xyz": xyz, "rpy": rpy,
            "axis": [float(x) for x in (ax.get("xyz").split() if ax is not None else [1, 0, 0])],
            "lower": float(lim.get("lower")) if lim is not None else -math.pi,
            "upper": float(lim.get("upper")) if lim is not None else math.pi,
        })
    for l in root.findall("link"):
        col = l.find("collision")
        box = None
        if col is not None:
            geo = col.find("geometry")
            b = geo.find("box") if geo is not None else None
            if b is not None:
                org = col.find("origin")
                xyz = [float(x) for x in (org.get("xyz").split() if org is not None and org.get("xyz") else [0, 0, 0])]
                rpy = [float(x) for x in (org.get("rpy").split() if org is not None and org.get("rpy") else [0, 0, 0])]
                size = [float(x) for x in b.get("size").split()]
                box = {"xyz": xyz, "rpy": rpy, "size": size}
        links[l.get("name")] = {"name": l.get("name"), "box": box}
    return root.get("name"), joints, links


def fk(joints, links, q):
    """返回 {link_name: 4x4 位姿}，世界系 = base_link。"""
    qmap = {j["name"]: q[i] for i, j in enumerate(joints)}
    poses = {}
    child2parent = {j["child"]: j for j in joints}
    # base_link 为根
    poses["base_link"] = np.eye(4)
    # 按拓扑顺序迭代
    pending = list(joints)
    for _ in range(len(pending) + 2):
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


def link_world_aabb(pose, box):
    """碰撞 box 在世界系的保守 AABB（8 角点）。"""
    if box is None:
        return None
    R = rpy_to_matrix(box["rpy"])
    sx, sy, sz = [s / 2.0 for s in box["size"]]
    corners = np.array([[dx * sx, dy * sy, dz * sz]
                         for dx in (-1, 1) for dy in (-1, 1) for dz in (-1, 1)])
    pts = corners @ R.T + np.asarray(box["xyz"])
    pts = pts @ pose[:3, :3].T + pose[:3, 3]
    return pts.min(axis=0), pts.max(axis=0)


def aabb_overlap(a, b, margin=0.0):
    if a is None or b is None:
        return False
    return bool(np.all(a[1] + margin > b[0]) and np.all(b[1] + margin > a[0]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", default=os.path.join(PKG, "urdf", "rm75_clean.urdf"))
    ap.add_argument("--srdf", default=os.path.join(PKG, "srdf", "rm75.srdf"))
    ap.add_argument("--group", default="rm75_arm")
    ap.add_argument("--base", default="base_link")
    ap.add_argument("--tip", default="Link7")
    ap.add_argument("--samples", type=int, default=3000)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--report", default=os.path.join(PKG, "srdf", "acm_report.json"))
    args = ap.parse_args()

    name, joints, links = load_urdf(args.urdf)
    rev = [j for j in joints if j["type"] in ("revolute", "continuous")]
    rng = np.random.default_rng(args.seed)

    link_names = [l for l in links if links[l]["box"] is not None]
    # 相邻关系
    parent_of = {j["child"]: j["parent"] for j in joints}
    adjacency = set()
    for a in link_names:
        p = parent_of.get(a)
        if p:
            adjacency.add(frozenset((a, p)))
        # 祖父也是"相邻"性质上不碰，但基座与远端连杆一般永不碰撞，由采样决定

    ever_overlap = set()
    for _ in range(args.samples):
        q = np.array([rng.uniform(j["lower"], j["upper"]) for j in rev])
        poses = fk(joints, links, q)
        aabbs = {ln: link_world_aabb(poses[ln], links[ln]["box"]) for ln in link_names}
        for i in range(len(link_names)):
            for k in range(i + 1, len(link_names)):
                a, b = link_names[i], link_names[k]
                if aabb_overlap(aabbs[a], aabbs[b]):
                    ever_overlap.add(frozenset((a, b)))

    disables = []
    for a in link_names:
        for b in link_names:
            if a >= b:
                continue
            pair = frozenset((a, b))
            if pair in adjacency:
                disables.append((a, b, "Adjacent"))
            elif pair not in ever_overlap:
                disables.append((a, b, "Never"))

    # ---- 写 SRDF ----
    srdf = ET.Element("robot", {"name": name})
    g = ET.SubElement(srdf, "group", {"name": args.group})
    ET.SubElement(g, "chain", {"base_link": args.base, "tip_link": args.tip})
    gs = ET.SubElement(srdf, "group_state", {"name": "home", "group": args.group})
    for j in rev:
        ET.SubElement(gs, "joint", {"name": j["name"], "value": "0"})
    tcps = ET.SubElement(srdf, "group_tcps", {"group": args.group})
    ET.SubElement(tcps, "tcp", {"name": "flange", "xyz": "0 0 0", "rpy": "0 0 0"})
    ET.SubElement(srdf, "kinematics_plugin_config",
                  {"filename": "package://rm_description/config/kinematics_plugins.yaml"})
    ET.SubElement(srdf, "contact_managers_plugin_config",
                  {"filename": "package://rm_description/config/contact_manager_plugins.yaml"})
    ET.SubElement(srdf, "virtual_joint", {"name": "world_joint", "type": "fixed",
                                          "parent_frame": "world", "child_link": args.base})
    for a, b, reason in disables:
        ET.SubElement(srdf, "disable_collisions", {"link1": a, "link2": b, "reason": reason})
    os.makedirs(os.path.dirname(args.srdf), exist_ok=True)
    ET.ElementTree(srdf).write(args.srdf, encoding="utf-8", xml_declaration=True)

    report = {
        "urdf": os.path.basename(args.urdf),
        "group": args.group, "base": args.base, "tip": args.tip,
        "samples": args.samples, "seed": args.seed,
        "links": link_names,
        "pairs_ever_overlap": sorted(sorted(p) for p in ever_overlap),
        "disabled": [{"a": a, "b": b, "reason": r} for a, b, r in disables],
        "enabled_pairs": [sorted(p) for p in (set(frozenset(c) for c in
                          [(x, y) for i, x in enumerate(link_names) for y in link_names[i + 1:]])
                          - set(frozenset((a, b)) for a, b, _ in disables))],
    }
    with open(args.report, "w", encoding="utf-8") as fh:
        json.dump(report, fh, ensure_ascii=False, indent=2)
    print("[srdf] group=%s  links=%d  disabled=%d  enabled=%d -> %s" %
          (args.group, len(link_names), len(disables), len(report["enabled_pairs"]), args.srdf))
    print("[srdf] 仍然启用的连杆对(需真机/精细模型复核):", report["enabled_pairs"])


if __name__ == "__main__":
    main()
