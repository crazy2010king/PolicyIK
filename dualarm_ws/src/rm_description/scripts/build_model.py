#!/usr/bin/env python3
"""把"可移植的相对路径 URDF"物化成"可直接加载的绝对路径模型"。

为什么需要这一步：Tesseract 0.35 的资源定位器不支持相对路径，也不接受自建 package://
（官方 RM75 URDF 里是 package://rm_description/...）。于是：
  · 仓库里保存相对路径版本（可 git、可移植）
  · 每台机器上生成一份绝对路径版本到 <ws>/build/model/（机器相关，gitignore）
这正是 `rmctl init` 要做的"零配置"动作。
"""
import argparse
import json
import os
import re
import shutil

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)


def build_model(ws=None, pkg=PKG, out=None):
    ws = ws or os.path.abspath(os.path.join(PKG, "..", ".."))
    out = out or os.path.join(ws, "build", "model")
    os.makedirs(out, exist_ok=True)
    src_urdf = os.path.join(pkg, "urdf", "rm75_clean.urdf")
    src_srdf = os.path.join(pkg, "srdf", "rm75.srdf")
    src_lim = os.path.join(pkg, "config", "joint_limits.yaml")

    # 用文本替换而非 XML round-trip：ElementTree 会破坏 tesseract: 命名空间前缀属性
    dst_urdf = os.path.join(out, "rm75_clean.urdf")
    text = open(src_urdf, "r", encoding="utf-8").read()
    base = os.path.dirname(src_urdf)
    n_mesh = 0

    def _abs(m):
        nonlocal n_mesh
        n_mesh += 1
        return 'filename="%s"' % os.path.normpath(os.path.join(base, m.group(1)))

    text = re.sub(r'filename="(\.\./[^"]+)"', _abs, text)
    with open(dst_urdf, "w", encoding="utf-8") as fh:
        fh.write(text)
    # SRDF 里的插件配置也是 package:// —— 一并物化（复制到 build/model/config 并改绝对路径）
    cfg_out = os.path.join(out, "config")
    os.makedirs(cfg_out, exist_ok=True)
    for fn in ("kinematics_plugins.yaml", "contact_manager_plugins.yaml"):
        src_cfg = os.path.join(pkg, "config", fn)
        if os.path.exists(src_cfg):
            shutil.copyfile(src_cfg, os.path.join(cfg_out, fn))
    srdf_text = open(src_srdf, "r", encoding="utf-8").read()
    srdf_text = srdf_text.replace("package://rm_description/config/",
                                  os.path.join(cfg_out, "") .replace("\\", "/") + "/")
    with open(os.path.join(out, "rm75.srdf"), "w", encoding="utf-8") as fh:
        fh.write(srdf_text)
    shutil.copyfile(src_lim, os.path.join(out, "joint_limits.yaml"))
    manifest = {
        "built_from": src_urdf, "urdf": dst_urdf,
        "srdf": os.path.join(out, "rm75.srdf"),
        "joint_limits": os.path.join(out, "joint_limits.yaml"),
        "mesh_paths_rewritten": n_mesh,
    }
    with open(os.path.join(out, "manifest.json"), "w", encoding="utf-8") as fh:
        json.dump(manifest, fh, ensure_ascii=False, indent=2)
    return manifest


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ws", default=None)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    m = build_model(args.ws, out=args.out)
    print("[model] %s (改写 %d 个 mesh 路径)" % (m["urdf"], m["mesh_paths_rewritten"]))


if __name__ == "__main__":
    main()
