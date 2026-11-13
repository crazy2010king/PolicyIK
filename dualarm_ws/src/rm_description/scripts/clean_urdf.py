#!/usr/bin/env python3
"""清洗官方 RM75 URDF。

做四件事（对应设计文档 §5.2 清洗清单）：
  1. 规整 SolidWorks 导出噪声：axis / rpy / origin 小数
  2. collision 与 visual 分离：碰撞体改成由网格 AABB 推出的 box primitive（§5.13.3 "primitive 优先"）
  3. 关节 7 规划域收窄到 ±180°（设计 M5）
  4. 机器人名改写为独立模型名，并保留原始 URDF 不动

用法: python3 clean_urdf.py [--src rm_75_official.urdf] [--dst rm75_clean.urdf]
"""
import argparse
import math
import os
import struct
import xml.etree.ElementTree as ET

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)

# 官方 MDH 参数（用于交叉核对，不参与 URDF 解析）
MDH_A = [0.2405, 0.0, 0.256, 0.0, 0.210, 0.0, 0.144]
MDH_ALPHA = [0.0, -math.pi / 2, math.pi / 2, -math.pi / 2, math.pi / 2, -math.pi / 2]
MDH_OFFSET = [0.0, -math.pi / 2, math.pi / 2, -math.pi / 2, math.pi / 2, -math.pi / 2]


def snap(v, tol=1e-4):
    """把接近 0 / ±1 的值规整到精确值；把接近 pi/2 的角度规整到精确 pi/2。"""
    if abs(v) < tol:
        return 0.0
    if abs(v - 1.0) < tol:
        return 1.0
    if abs(v + 1.0) < tol:
        return -1.0
    for target in (math.pi / 2, -math.pi / 2, math.pi, -math.pi, 0.0):
        if abs(v - target) < 1e-3:
            return target
    return v


def snap_triple(values):
    return [snap(float(v)) for v in values]


def read_stl(path):
    """返回 (N,3) 顶点数组。兼容二进制与 ASCII STL。"""
    with open(path, "rb") as fh:
        head = fh.read(84)
        if len(head) < 84:
            raise ValueError("STL too small: %s" % path)
        n = struct.unpack("<I", head[80:84])[0]
        expected = 84 + n * 50
        import os as _os
        if _os.path.getsize(path) >= expected:
            data = fh.read(n * 50)
            tris = np.frombuffer(data, dtype=np.uint8).reshape(n, 50)
            verts = tris[:, 12:48].copy().view("<f4").reshape(n, 3, 3).astype(np.float64)
            return verts.reshape(-1, 3)
    # ASCII fallback
    pts = []
    with open(path, "r", errors="ignore") as fh:
        for line in fh:
            parts = line.split()
            if parts and parts[0] == "vertex":
                pts.append([float(x) for x in parts[1:4]])
    return np.asarray(pts, dtype=np.float64).reshape(-1, 3)


def rpy_to_matrix(rpy):
    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def resolve_mesh(pkg_root, filename):
    """package://rm_description/meshes/... -> 本地绝对路径"""
    if filename.startswith("package://"):
        rel = filename[len("package://"):]
        parts = rel.split("/", 1)
        if len(parts) == 2:
            return os.path.join(pkg_root, parts[1])
    if os.path.isabs(filename):
        return filename
    return os.path.normpath(os.path.join(pkg_root, filename))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", default=os.path.join(PKG, "urdf", "rm_75_official.urdf"))
    ap.add_argument("--dst", default=os.path.join(PKG, "urdf", "rm75_clean.urdf"))
    ap.add_argument("--robot-name", default="rm75_clean")
    ap.add_argument("--j7-limit", type=float, default=math.pi,
                    help="joint7 规划域限位(rad)，0 表示保持官方 ±6.28")
    ap.add_argument("--report", default=os.path.join(PKG, "urdf", "clean_report.json"))
    args = ap.parse_args()

    tree = ET.parse(args.src)
    root = tree.getroot()
    root.set("name", args.robot_name)
    # Tesseract 0.35 要求 <robot> 上带 tesseract:make_convex（我们用 primitive 碰撞体，取 false）
    root.set("xmlns:tesseract", "https://github.com/tesseract-robotics/tesseract")
    root.set("tesseract:make_convex", "false")

    report = {"source": os.path.basename(args.src), "joints": [], "links": [], "changes": []}

    # ---- 1) 关节：axis / origin 规整，J7 限位收窄 ----
    for j in root.findall("joint"):
        name = j.get("name")
        axis = j.find("axis")
        if axis is not None:
            before = [float(x) for x in axis.get("xyz").split()]
            after = snap_triple(before)
            if before != after:
                report["changes"].append({"joint": name, "field": "axis",
                                          "before": before, "after": after})
            axis.set("xyz", " ".join(repr(v) for v in after))
        org = j.find("origin")
        if org is not None:
            for attr in ("xyz", "rpy"):
                if org.get(attr):
                    before = [float(x) for x in org.get(attr).split()]
                    after = snap_triple(before)
                    if before != after:
                        report["changes"].append({"joint": name, "field": "origin." + attr,
                                                  "before": before, "after": after})
                    org.set(attr, " ".join(repr(v) for v in after))
        lim = j.find("limit")
        if lim is not None and name == "joint7" and args.j7_limit > 0:
            before = (lim.get("lower"), lim.get("upper"))
            lim.set("lower", repr(-args.j7_limit))
            lim.set("upper", repr(args.j7_limit))
            report["changes"].append({"joint": name, "field": "limit", "before": before,
                                      "after": (-args.j7_limit, args.j7_limit)})
        report["joints"].append({
            "name": name,
            "type": j.get("type"),
            "parent": j.find("parent").get("link"),
            "child": j.find("child").get("link"),
            "origin_xyz": [float(x) for x in (org.get("xyz").split() if org is not None and org.get("xyz") else [0, 0, 0])],
            "origin_rpy": [float(x) for x in (org.get("rpy").split() if org is not None and org.get("rpy") else [0, 0, 0])],
            "axis": [float(x) for x in (axis.get("xyz").split() if axis is not None else [1, 0, 0])],
            "limit": dict(lim.attrib) if lim is not None else {},
        })

    # ---- 2) 碰撞体 primitive 化 ----
    for link in root.findall("link"):
        lname = link.get("name")
        col = link.find("collision")
        if col is None:
            continue
        geo = col.find("geometry")
        mesh = geo.find("mesh") if geo is not None else None
        if mesh is None:
            continue
        mesh_path = resolve_mesh(PKG, mesh.get("filename"))
        if not os.path.exists(mesh_path):
            report["links"].append({"link": lname, "status": "mesh_missing", "file": mesh.get("filename")})
            continue
        verts = read_stl(mesh_path)
        corg = col.find("origin")
        cxyz = [float(x) for x in (corg.get("xyz").split() if corg is not None and corg.get("xyz") else [0, 0, 0])]
        crpy = [float(x) for x in (corg.get("rpy").split() if corg is not None and corg.get("rpy") else [0, 0, 0])]
        R = rpy_to_matrix(crpy)
        pts = verts @ R.T + np.asarray(cxyz)
        lo, hi = pts.min(axis=0), pts.max(axis=0)
        size = np.maximum(hi - lo, 1e-4)
        center = (hi + lo) / 2.0
        # 用 box 替换 mesh
        geo.clear()
        box = ET.SubElement(geo, "box")
        box.set("size", " ".join(repr(float(v)) for v in size))
        corg = col.find("origin")
        if corg is None:
            corg = ET.SubElement(col, "origin")
        corg.set("xyz", " ".join(repr(float(v)) for v in center))
        corg.set("rpy", "0 0 0")
        report["links"].append({
            "link": lname, "status": "box", "mesh": os.path.basename(mesh_path),
            "mesh_triangles": int(verts.shape[0] / 3),
            "box_size_mm": [round(float(v) * 1000, 1) for v in size],
            "box_center_mm": [round(float(v) * 1000, 1) for v in center],
        })

    # ---- 3) Tesseract 0.35 解析器要求每个 <mesh> 带 tesseract:make_convex 属性；并把
    #         package:// 路径改写为相对 URDF 的相对路径（保证离线可移植）
    for mesh in root.iter("mesh"):
        mesh.set("tesseract:make_convex", "false")
        fn = mesh.get("filename") or ""
        if fn.startswith("package://rm_description/"):
            rel = fn[len("package://rm_description/"):]
            mesh.set("filename", "../" + rel)

    # ElementTree 会把带前缀属性改写成 ns0:，而 Tesseract 解析器要求字面量 "tesseract:" 前缀
    xml = ET.tostring(root, encoding="unicode")
    xml = xml.replace('xmlns:ns0="https://github.com/tesseract-robotics/tesseract"',
                      'xmlns:tesseract="https://github.com/tesseract-robotics/tesseract"')
    xml = xml.replace('ns0:make_convex', 'tesseract:make_convex')
    with open(args.dst, "w", encoding="utf-8") as fh:
        fh.write('<?xml version="1.0" encoding="utf-8"?>\n' + xml)
    import json
    with open(args.report, "w", encoding="utf-8") as fh:
        json.dump(report, fh, ensure_ascii=False, indent=2)
    print("[clean] joints=%d links=%d changes=%d -> %s" %
          (len(report["joints"]), len(report["links"]), len(report["changes"]), args.dst))


if __name__ == "__main__":
    main()
