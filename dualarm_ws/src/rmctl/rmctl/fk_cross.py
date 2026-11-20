#!/usr/bin/env python3
"""FK 与控制器位姿交叉校验台架（设计 §5.3）。

目的：让"自研 FK"能自证清白——20 组随机位形，比较**自研 FK 位姿**与**控制器
（厂商运动学）上报位姿**的偏差，判定阈值 位置 < 1 mm / 姿态 < 0.05°。

设计约束（D001 / R003 / R011）：

* **只使用自研 FK**（``rmctl.selfkin``，纯 numpy + xml），**禁止 import tesseract**
  或任何厂商解算接口——否则在没有 GLIBCXX_3.4.31 的环境里无法被 pytest 覆盖。
* 控制器位姿是**厂商运动学输出**，只允许进入本模块的交叉校验，**不得作为规划真值**
  （设计 §1.4）。本模块**不提供**任何把 ``controller_pose`` 写回轨迹/模型/基线的
  路径；出口只有一份只读的比对报告。
* 关节读数单位是「度」（SDK 原生单位），度 → 弧度的换算**只在一处**
  （``_to_rad``）发生。
* 偏差大时按设计 §5.3 的顺序排查：**零位 → 工具系 → MDH 偏置**，不靠调参掩盖。
  报告给出逐样本偏差向量，用于识别"整体平移（零位漂移）"与"分散误差（单关节/MDH）"。

模块入口::

    python3 -m rmctl.fk_cross --samples samples.json --out report.json

退出码：判定通过 0 / 偏差超限 3 / 输入非法 2。
"""
import argparse
import json
import math
import os
import sys

import numpy as np

from . import calibration, paths
from .selfkin import UrdfModel

# 判定阈值（设计 §5.3）
DEFAULT_POS_TOL_MM = 1.0
DEFAULT_ORI_TOL_DEG = 0.05

# 控制器位姿用途声明（R003）
CONTROLLER_POSE_USAGE = "仅用于 FK 交叉校验，不得作为规划真值"

# 设计 §5.3 的排查序
TROUBLESHOOT_ORDER = ("zero_offset", "tool_frame", "mdh_offset")

_TROUBLESHOOT_TEXT = {
    "zero_offset": "查零位：示教器凹槽对齐照片、joint1-7 zero_deg 读数、"
                   "换电池/拆装/冲击/固件升级后是否漂移",
    "tool_frame": "查工具系：TCP/法兰偏置（workcell calibration.tcp 目前为 null，"
                  "未标定前工具系误差必然存在）",
    "mdh_offset": "查 MDH 偏置：官方 MDH 表 a/α/θ 偏置与清洗 URDF 逐项核对",
}

DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi


class FkCrossError(Exception):
    """输入非法 / 依赖缺失（消息面向操作者，可直接粘贴）。"""


# ------------------------------------------------------------------ 换算
def _to_rad(values, unit="deg"):
    """关节读数 → 弧度。**全模块唯一的角度换算点**。

    ``unit`` 默认 ``deg``（SDK 原生单位）；允许 ``rad`` 以便对比验证
    （用度构造与用弧度构造的样本应给出同一结论）。
    """
    arr = np.asarray(values, dtype=float).reshape(-1)
    if unit == "deg":
        return arr * DEG2RAD
    if unit == "rad":
        return arr
    raise FkCrossError("未知的关节角单位 %r（只支持 deg / rad）" % (unit,))


def _quat_to_matrix(quat):
    """四元数 (x, y, z, w) → 3x3 旋转矩阵。"""
    x, y, z, w = [float(v) for v in quat]
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        raise FkCrossError("四元数模长为零")
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _rpy_to_matrix(rpy):
    """rpy (rad) → 3x3 旋转矩阵（roll-pitch-yaw，Rz*Ry*Rx）。"""
    r, p, y = [float(v) for v in rpy]
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def _rotation_angle_deg(R_a, R_b):
    """两个旋转矩阵之间的夹角（deg）。"""
    R = R_a.T @ R_b
    cos_t = (np.trace(R) - 1.0) / 2.0
    cos_t = max(-1.0, min(1.0, float(cos_t)))
    return math.acos(cos_t) * RAD2DEG


# ------------------------------------------------------------------ 样本解析
def _sample_joints(sample, index, joint_names):
    """取样本的关节角（弧度），并做形状/数值校验。"""
    if "joint_deg" in sample:
        unit = sample.get("joint_unit", "deg")
        values = sample["joint_deg"]
    elif "joint_rad" in sample:
        unit = "rad"
        values = sample["joint_rad"]
    else:
        raise FkCrossError("样本 #%d 缺 joint_deg（关节读数，度）" % index)
    if isinstance(values, dict):
        missing = [n for n in joint_names if n not in values]
        if missing:
            raise FkCrossError("样本 #%d 的 joint_deg 缺关节 %s" % (index, ", ".join(missing)))
        values = [values[n] for n in joint_names]
    try:
        arr = _to_rad(values, unit)
    except (TypeError, ValueError) as exc:
        raise FkCrossError("样本 #%d 的关节角无法解析: %r" % (index, exc))
    if arr.size != len(joint_names):
        raise FkCrossError("样本 #%d 关节数 %d ≠ 模型关节数 %d"
                           % (index, arr.size, len(joint_names)))
    if not np.all(np.isfinite(arr)):
        raise FkCrossError("样本 #%d 的关节角含 NaN/Inf" % index)
    return arr


def _sample_pose(sample, index):
    """取样本的控制器位姿 → (位置 xyz[m], 旋转矩阵或 None, 姿态字段名)。"""
    pose = sample.get("controller_pose")
    if pose is None:
        raise FkCrossError("样本 #%d 缺 controller_pose（控制器上报位姿）" % index)
    if not isinstance(pose, dict):
        raise FkCrossError("样本 #%d 的 controller_pose 必须是对象" % index)
    missing = [k for k in ("x", "y", "z") if pose.get(k) is None]
    if missing:
        raise FkCrossError("样本 #%d 的 controller_pose 缺位置字段 %s"
                           % (index, ", ".join(missing)))
    try:
        pos = np.array([float(pose["x"]), float(pose["y"]), float(pose["z"])], dtype=float)
    except (TypeError, ValueError) as exc:
        raise FkCrossError("样本 #%d 的 controller_pose 位置无法解析: %r" % (index, exc))
    if not np.all(np.isfinite(pos)):
        raise FkCrossError("样本 #%d 的 controller_pose 位置含 NaN/Inf" % index)

    rot, kind = None, None
    if pose.get("rpy") is not None:
        try:
            rot = _rpy_to_matrix(pose["rpy"])
            kind = "rpy"
        except (TypeError, ValueError) as exc:
            raise FkCrossError("样本 #%d 的 controller_pose.rpy 无法解析: %r" % (index, exc))
    elif pose.get("quat") is not None:
        try:
            rot = _quat_to_matrix(pose["quat"])
            kind = "quat"
        except (TypeError, ValueError) as exc:
            raise FkCrossError("样本 #%d 的 controller_pose.quat 无法解析: %r" % (index, exc))
    return pos, rot, kind


# ------------------------------------------------------------------ 归因
def _attribution(devs, tol_mm):
    """按设计 §5.3 给出初步归因提示。

    devs: 逐样本偏差向量（mm，fk - controller）。
    """
    if not devs:
        return {"pattern": "no_samples", "consistency": None,
                "mean_vector_mm": None, "hints": [], "next_checks": []}
    arr = np.asarray(devs, dtype=float)
    mean_vec = arr.mean(axis=0)
    mags = np.linalg.norm(arr, axis=1)
    mean_mag = float(mags.mean())
    max_mag = float(mags.max())
    # consistency = |平均偏差向量| / 最大偏差：
    #   ≈1 → 所有样本偏差方向一致（整体平移，典型零位漂移）
    #   ≈0 → 偏差互相抵消（个别样本/个别关节异常）
    consistency = float(np.linalg.norm(mean_vec) / max_mag) if max_mag > 1e-12 else 0.0
    if max_mag <= tol_mm:
        pattern = "within_tolerance"
    elif consistency >= 0.8:
        pattern = "systematic_translation"
    else:
        pattern = "scattered"
    hints = []
    if pattern == "within_tolerance":
        hints.append("全部样本在阈值内：自研 FK 与控制器位姿一致，无需排查")
        next_checks = []
    else:
        if pattern == "systematic_translation":
            hints.append("偏差方向高度一致（consistency=%.2f）：疑似**整体平移**，"
                         "典型原因是零位漂移" % consistency)
        else:
            hints.append("偏差方向分散（consistency=%.2f）：疑似**单关节/建模参数**误差"
                         % consistency)
        hints.append("按设计 §5.3 顺序排查：零位 → 工具系 → MDH 偏置")
        next_checks = list(TROUBLESHOOT_ORDER)
    return {
        "pattern": pattern,
        "consistency": consistency,
        "mean_vector_mm": [float(v) for v in mean_vec],
        "mean_dev_mm": mean_mag,
        "max_dev_mm": max_mag,
        "hints": hints,
        "next_checks": next_checks,
        "next_check_detail": [_TROUBLESHOOT_TEXT[k] for k in next_checks],
    }


# ------------------------------------------------------------------ 偏置
def _prepare_offset(spec, kind, name):
    """把外部给的偏置规范成内部结构；非法输入抛 FkCrossError。"""
    if spec is None:
        return None
    try:
        return calibration.normalize_offset(spec, kind)
    except calibration.CalibrationError as exc:
        raise FkCrossError("%s 非法: %s" % (name, exc))


def _offset_public(offset):
    """偏置的公开回显（可 JSON 序列化）。"""
    if offset is None:
        return None
    return {
        "kind": offset["kind"],
        "frame": offset["frame"],
        "x": offset["x"],
        "y": offset["y"],
        "z": offset["z"],
        "rpy": list(offset["rpy"]),
        "method": offset["method"],
        "source": offset["source"],
        "captured_at": offset["captured_at"],
        "uncertainty_mm": offset["uncertainty_mm"],
    }


# ------------------------------------------------------------------ 主流程
def cross_check(samples, urdf_path=None, pos_tol_mm=DEFAULT_POS_TOL_MM,
                ori_tol_deg=DEFAULT_ORI_TOL_DEG, tip="Link7",
                tool_offset=None, base_offset=None):
    """自研 FK vs 控制器位姿交叉校验。

    参数
    ----
    samples : list[dict]
        每条含 ``joint_deg``（SDK 关节读数，度；可选 ``joint_unit: rad``）与
        ``controller_pose``（``x/y/z`` 必填，单位 m；可选 ``rpy``（rad）或
        ``quat``（x,y,z,w）用于姿态偏差）。
    urdf_path : str
        URDF 路径；``None`` 时用 ``paths.urdf_path()``。
    pos_tol_mm / ori_tol_deg : float
        判定阈值（位置 mm / 姿态 deg）。
    tip : str
        FK 末端 link（默认 ``Link7``）。
    tool_offset : dict | None
        **法兰→工具偏置，在 tip(Link7) 系表达**（结构见
        ``rmctl.calibration.OFFSET_FIELDS``）。施加 ``T = T_link7 · T_tool``：
        位置与姿态**同时**被修正。
    base_offset : dict | None
        **基座/世界系刚体偏置，在 base 系表达**。施加 ``T = T_base · T_link7``。

    返回
    ----
    dict 报告（可 JSON 序列化）。``ok`` 为最终判定；控制器未提供姿态时姿态项
    记 ``not_provided`` 且**不参与判定**。

    异常
    ----
    FkCrossError：输入非法（空样本、缺字段、关节数不符等）。

    判定口径
    --------
    ``position`` / ``orientation`` / ``ok`` 始终是**未校正（raw）**的量，
    即"自研 FK vs 控制器位姿"的原始偏差；施加偏置后的量放在
    ``corrections.corrected`` 与顶层 ``ok_corrected``。两者都摆在报告里，
    便于一眼看出偏置是否真的把残差压下去了。
    """
    tool_offset = _prepare_offset(tool_offset, "tool", "tool_offset（tip 系偏置）")
    base_offset = _prepare_offset(base_offset, "base", "base_offset（base 系偏置）")
    T_tool = calibration.offset_transform(tool_offset)
    T_base = calibration.offset_transform(base_offset)
    if samples is None:
        raise FkCrossError("样本列表为 None")
    if isinstance(samples, dict):
        samples = samples.get("samples")
    if not isinstance(samples, (list, tuple)):
        raise FkCrossError("样本必须是 JSON 数组（list）")
    if len(samples) == 0:
        raise FkCrossError("样本列表为空：至少需要 1 组位形才能交叉校验")

    urdf_path = urdf_path or paths.urdf_path()
    if not os.path.exists(urdf_path):
        raise FkCrossError("URDF 不存在: %s（先执行 python3 -m rmctl init 物化模型）"
                           % urdf_path)
    try:
        urdf = UrdfModel(urdf_path)
    except Exception as exc:
        raise FkCrossError("URDF 解析失败: %r" % exc)
    joint_names = [j["name"] for j in urdf.rev]
    if tip not in urdf.links:
        raise FkCrossError("URDF 里没有末端 link %r（可选: %s）"
                           % (tip, ", ".join(sorted(urdf.links))))

    details = []
    devs = []
    dev_corrs = []
    ori_devs = []
    ori_corr_devs = []
    ori_provided = False
    for i, sample in enumerate(samples):
        if not isinstance(sample, dict):
            raise FkCrossError("样本 #%d 必须是对象" % i)
        q = _sample_joints(sample, i, joint_names)
        ctrl_pos, ctrl_rot, ctrl_kind = _sample_pose(sample, i)
        try:
            T = urdf.fk(q, tip)
        except Exception as exc:
            raise FkCrossError("样本 #%d 的 FK 计算失败: %r" % (i, exc))
        fk_pos = np.asarray(T[:3, 3], dtype=float)
        dev = (fk_pos - ctrl_pos) * 1000.0            # mm
        dev_mm = float(np.linalg.norm(dev))
        devs.append(dev)

        ori_deg, ori_state = None, "not_provided"
        if ctrl_rot is not None:
            ori_provided = True
            ori_deg = _rotation_angle_deg(np.asarray(T[:3, :3], dtype=float), ctrl_rot)
            ori_devs.append(ori_deg)
            ori_state = "provided"

        # ---- 施加标定偏置（raw → corrected）----
        #   tool_offset: 法兰→工具在 **tip(Link7) 系** → T' = T_link7 · T_tool
        #   base_offset: 基座/世界系在 **base 系** → T' = T_base · T_link7
        # 左乘与右乘互可交换，同时施加时无先后歧义。
        T_corr = np.asarray(T, dtype=float)
        if T_base is not None:
            T_corr = T_base @ T_corr
        if T_tool is not None:
            T_corr = T_corr @ T_tool
        fk_corr_pos = np.asarray(T_corr[:3, 3], dtype=float)
        dev_corr = (fk_corr_pos - ctrl_pos) * 1000.0
        dev_corr_mm = float(np.linalg.norm(dev_corr))
        dev_corrs.append(dev_corr)

        ori_corr_deg = None
        if ctrl_rot is not None:
            ori_corr_deg = _rotation_angle_deg(np.asarray(T_corr[:3, :3], dtype=float),
                                                ctrl_rot)
            ori_corr_devs.append(ori_corr_deg)

        entry = {
            "index": i,
            "joint_deg": [float(v) for v in q * RAD2DEG],
            "fk_position_m": [float(v) for v in fk_pos],
            "controller_position_m": [float(v) for v in ctrl_pos],
            "dev_mm": dev_mm,
            "dev_vector_mm": [float(v) for v in dev],
            "dev_direction": [float(v) for v in (dev / dev_mm if dev_mm > 1e-12 else dev * 0.0)],
            "dominant_axis": ("x", "y", "z")[int(np.argmax(np.abs(dev)))] if dev_mm > 1e-12 else None,
            "pos_ok": bool(dev_mm <= pos_tol_mm),
            "corrected_dev_mm": dev_corr_mm,
            "corrected_dev_vector_mm": [float(v) for v in dev_corr],
            "corrected_orientation_dev_deg": ori_corr_deg,
            "orientation": {"state": ori_state, "source": ctrl_kind,
                            "dev_deg": ori_deg,
                            "ok": (None if ori_deg is None else bool(ori_deg <= ori_tol_deg))},
        }
        details.append(entry)

    pos_mags = [d["dev_mm"] for d in details]
    max_pos = max(pos_mags)
    mean_pos = float(np.mean(pos_mags))
    violations = [d["index"] for d in details if not d["pos_ok"]]

    ori_violations = []
    if ori_provided:
        max_ori = max(ori_devs)
        mean_ori = float(np.mean(ori_devs))
        ori_violations = [d["index"] for d in details
                          if d["orientation"]["dev_deg"] is not None
                          and not d["orientation"]["ok"]]
        ori_status = "provided"
    else:
        max_ori, mean_ori = None, None
        ori_status = "not_provided"

    ok = (not violations) and (not ori_violations)

    # ---- 校正后（corrected）指标 ----
    corr_mags = [d["corrected_dev_mm"] for d in details]
    corr_max = max(corr_mags)
    corr_mean = float(np.mean(corr_mags))
    corr_violations = [d["index"] for d in details
                       if d["corrected_dev_mm"] > pos_tol_mm]
    if ori_provided:
        corr_max_ori = max(ori_corr_devs)
        corr_mean_ori = float(np.mean(ori_corr_devs))
        corr_ori_violations = [d["index"] for d in details
                               if d["corrected_dev_mm"] > pos_tol_mm
                               or (d["corrected_orientation_dev_deg"] is not None
                                   and d["corrected_orientation_dev_deg"] > ori_tol_deg)]
    else:
        corr_max_ori, corr_mean_ori = None, None
        corr_ori_violations = []
    ok_corrected = (not corr_violations) and (not corr_ori_violations)

    corrections = {
        "applied": {
            "tool_offset": _offset_public(tool_offset),
            "base_offset": _offset_public(base_offset),
        },
        "raw": {
            "max_pos_dev_mm": float(max_pos),
            "mean_pos_dev_mm": float(mean_pos),
            "max_ori_dev_deg": (None if max_ori is None else float(max_ori)),
            "ok": bool(ok),
        },
        "corrected": {
            "max_pos_dev_mm": float(corr_max),
            "mean_pos_dev_mm": float(corr_mean),
            "max_ori_dev_deg": (None if corr_max_ori is None else float(corr_max_ori)),
            "ok": bool(ok_corrected),
            "violations": corr_violations,
            "orientation_violations": corr_ori_violations,
        },
        "improved": bool(corr_max < max_pos - 1e-12),
        "any_applied": bool(tool_offset is not None or base_offset is not None),
    }

    report = {
        "schema": "rmctl.fk_cross/1",
        "urdf": os.path.abspath(urdf_path),
        "tip": tip,
        "joint_names": joint_names,
        "samples": len(details),
        "tolerances": {"pos_mm": float(pos_tol_mm), "ori_deg": float(ori_tol_deg)},
        "position": {
            "max_mm": max_pos,
            "mean_mm": mean_pos,
            "min_mm": float(min(pos_mags)),
            "tol_mm": float(pos_tol_mm),
            "status": "fail" if violations else "pass",
        },
        "orientation": {
            "status": ori_status,
            "max_deg": max_ori,
            "mean_deg": mean_ori,
            "tol_deg": float(ori_tol_deg),
            "note": ("控制器未提供姿态，姿态项不参与判定" if not ori_provided
                     else CONTROLLER_POSE_USAGE),
        },
        "ok": bool(ok),
        "ok_corrected": bool(ok_corrected),
        "corrections": corrections,
        "violations": violations,
        "orientation_violations": ori_violations,
        "samples_detail": details,
        "attribution": _attribution(devs, pos_tol_mm),
        "controller_pose_usage": CONTROLLER_POSE_USAGE,
        "troubleshoot_order": list(TROUBLESHOOT_ORDER),
    }
    return report


# ------------------------------------------------------------------ 合成样本
def synthesize_samples(n=20, seed=42, offset_mm=0.0, offset_index=0,
                       urdf_path=None, tip="Link7", with_orientation=True,
                       joint_limit_margin=0.05):
    """生成合成样本（用于离线自测台架，不产生任何真机数值）。

    合成样本 = 随机位形下"自研 FK 位姿 + 注入偏移"，因此理想情况（offset_mm=0）
    偏差应在数值噪声量级（~1e-12 m）。``offset_mm`` 只注入到第 ``offset_index``
    个样本，用来模拟"单个位形/单个关节"的误差。

    返回 ``{"samples": [...], "meta": {...}}``。
    """
    if n <= 0:
        raise FkCrossError("样本数必须为正: %r" % (n,))
    urdf_path = urdf_path or paths.urdf_path()
    if not os.path.exists(urdf_path):
        raise FkCrossError("URDF 不存在: %s" % urdf_path)
    urdf = UrdfModel(urdf_path)
    joint_names = [j["name"] for j in urdf.rev]
    rng = np.random.default_rng(seed)
    out = []
    for i in range(int(n)):
        q = []
        for j in urdf.rev:
            lo, hi = j["lower"], j["upper"]
            span = max(hi - lo, 1e-6)
            q.append(float(rng.uniform(lo + joint_limit_margin * span,
                                       hi - joint_limit_margin * span)))
        q = np.asarray(q, dtype=float)
        T = urdf.fk(q, tip)
        pos = np.asarray(T[:3, 3], dtype=float)
        if offset_mm and i == offset_index:
            pos = pos + np.array([offset_mm / 1000.0, 0.0, 0.0])
        pose = {"x": float(pos[0]), "y": float(pos[1]), "z": float(pos[2])}
        if with_orientation:
            R = np.asarray(T[:3, :3], dtype=float)
            pose["quat"] = _matrix_to_quat(R)
        out.append({"joint_deg": [float(v) for v in q * RAD2DEG],
                    "controller_pose": pose})
    meta = {"n": int(n), "seed": int(seed), "offset_mm": float(offset_mm),
            "offset_index": int(offset_index), "urdf": os.path.abspath(urdf_path),
            "tip": tip, "source": "synthetic（自研 FK，非真机数据）"}
    return {"samples": out, "meta": meta}


def _matrix_to_quat(R):
    """3x3 旋转矩阵 → 四元数 (x, y, z, w)。"""
    tr = float(np.trace(R))
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return [float(x), float(y), float(z), float(w)]


def fk_recheck_fields(report):
    """把交叉校验报告压成 ``zero_baseline.yaml`` 的 ``fk_recheck`` 字段。

    字段口径（与基线 schema 一致）：

    * ``method``：复测方法与样本组数
    * ``samples``：样本组数
    * ``max_pos_dev_mm``：最大位置偏差（mm，判定 < 1）
    * ``max_ori_dev_deg``：最大姿态偏差（deg，判定 < 0.05；未提供姿态时为 null）
    * ``conclusion``：结论文本
    * ``corrections``：本次使用的标定偏置与 raw/corrected 对比（新增键）

    只做**格式转换**，不写盘、不改状态位（status 的 measured/blocked 判定属真机窗口）。
    """
    pos = report["position"]
    ori = report["orientation"]
    if report["ok"]:
        conclusion = ("PASS：%d 组位形，自研 FK 与控制器位姿最大位置偏差 %.4f mm"
                      "（≤ %.4f mm）" % (report["samples"], pos["max_mm"],
                                       report["tolerances"]["pos_mm"]))
    else:
        conclusion = ("FAIL：最大位置偏差 %.4f mm（阈值 %.4f mm），越界样本 %s"
                      % (pos["max_mm"], report["tolerances"]["pos_mm"],
                         report["violations"] or "无"))
    max_ori = ori["max_deg"]
    corr = report.get("corrections") or {}
    fields = {
        "method": ("rmctl.fk_cross（自研 FK vs 控制器位姿交叉校验，%d 组位形）"
                   % report["samples"]),
        "samples": int(report["samples"]),
        "max_pos_dev_mm": float(pos["max_mm"]),
        "max_ori_dev_deg": (None if max_ori is None else float(max_ori)),
        "conclusion": conclusion,
        "controller_pose_usage": CONTROLLER_POSE_USAGE,
    }
    if corr:
        applied = corr.get("applied") or {}
        fields["corrections"] = {
            "tool_offset": applied.get("tool_offset"),
            "base_offset": applied.get("base_offset"),
            "raw_max_pos_dev_mm": (corr.get("raw") or {}).get("max_pos_dev_mm"),
            "corrected_max_pos_dev_mm": (corr.get("corrected") or {}).get("max_pos_dev_mm"),
            "raw_max_ori_dev_deg": (corr.get("raw") or {}).get("max_ori_dev_deg"),
            "corrected_max_ori_dev_deg": (corr.get("corrected") or {}).get("max_ori_dev_deg"),
            "improved": bool(corr.get("improved")),
            "ok_corrected": bool(report.get("ok_corrected", report["ok"])),
        }
    return fields


# ------------------------------------------------------------------ CLI
def _load_samples(path):
    if not os.path.exists(path):
        raise FkCrossError("样本文件不存在: %s" % path)
    try:
        with open(path, "r", encoding="utf-8") as fh:
            data = json.load(fh)
    except Exception as exc:
        raise FkCrossError("样本文件解析失败 (%s): %r" % (path, exc))
    if isinstance(data, dict):
        data = data.get("samples")
    return data


def main(argv=None):
    ap = argparse.ArgumentParser(
        prog="python3 -m rmctl.fk_cross",
        description="自研 FK 与控制器位姿交叉校验（判定 <1 mm / 0.05°）")
    ap.add_argument("--samples", required=True, help="样本 JSON 文件（数组或 {samples:[...]}）")
    ap.add_argument("--out", default=None, help="报告输出路径（JSON）；缺省打印到 stdout")
    ap.add_argument("--urdf", default=None, help="URDF 路径（缺省用 rmctl 的模型路径）")
    ap.add_argument("--pos-tol-mm", type=float, default=DEFAULT_POS_TOL_MM)
    ap.add_argument("--ori-tol-deg", type=float, default=DEFAULT_ORI_TOL_DEG)
    ap.add_argument("--synthesize", type=int, default=None,
                    help="不读文件：合成 N 组样本（离线自测）")
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--offset-mm", type=float, default=0.0)
    ap.add_argument("--tool-offset", default=None,
                    help="法兰→工具偏置（tip/Link7 系）：JSON 文件路径或 "
                         "\"x,y,z[,rx,ry,rz]\"（m / rad）")
    ap.add_argument("--base-offset", default=None,
                    help="基座/世界系刚体偏置（base 系）：JSON 文件路径或 "
                         "\"x,y,z[,rx,ry,rz]\"（m / rad）")
    ap.add_argument("--calibration", default=None,
                    help="workcell yaml 路径：直接读其 calibration 段（tcp / base_to_base）")
    args = ap.parse_args(argv)

    try:
        tool_offset = None
        base_offset = None
        if args.calibration:
            cal = calibration.load_workcell_calibration(args.calibration)
            tool_offset = cal.get("tcp")
            base_offset = cal.get("base_to_base")
        if args.tool_offset:
            tool_offset = calibration.parse_offset_spec(
                args.tool_offset, "tool", source="cli:--tool-offset")
        if args.base_offset:
            base_offset = calibration.parse_offset_spec(
                args.base_offset, "base", source="cli:--base-offset")
        if args.synthesize is not None:
            samples = synthesize_samples(n=args.synthesize, seed=args.seed,
                                         offset_mm=args.offset_mm,
                                         urdf_path=args.urdf)["samples"]
        else:
            samples = _load_samples(args.samples)
        report = cross_check(samples, args.urdf, args.pos_tol_mm, args.ori_tol_deg,
                             tool_offset=tool_offset, base_offset=base_offset)
    except (FkCrossError, calibration.CalibrationError) as exc:
        print("输入非法: %s" % exc, file=sys.stderr)
        return 2
    except Exception as exc:  # noqa: BLE001 —— 顶层兜底，给出可读错误
        print("交叉校验失败: %r" % exc, file=sys.stderr)
        return 2

    text = json.dumps(report, ensure_ascii=False, indent=2)
    if args.out:
        d = os.path.dirname(os.path.abspath(args.out))
        if d and not os.path.isdir(d):
            os.makedirs(d, exist_ok=True)
        with open(args.out, "w", encoding="utf-8") as fh:
            fh.write(text)
        print("报告已写出: %s" % args.out)
    else:
        print(text)
    print("判定: %s（位置 max=%.4g mm %s；姿态 %s）"
          % ("PASS" if report["ok"] else "FAIL",
             report["position"]["max_mm"],
             "≤%.4g" % report["tolerances"]["pos_mm"],
             ("max=%s deg" % report["orientation"]["max_deg"])
             if report["orientation"]["status"] == "provided" else "not_provided"))
    if report["violations"]:
        print("越界样本: %s" % ", ".join(str(i) for i in report["violations"]))
    corr = report.get("corrections") or {}
    if corr.get("any_applied"):
        print("已施加标定偏置: tip=%s base=%s" % (
            "有" if (corr["applied"] or {}).get("tool_offset") else "无",
            "有" if (corr["applied"] or {}).get("base_offset") else "无"))
        print("位置偏差 raw max=%.4g mm → corrected max=%.4g mm（ok_corrected=%s）"
              % (corr["raw"]["max_pos_dev_mm"], corr["corrected"]["max_pos_dev_mm"],
                 report["ok_corrected"]))
    for hint in report["attribution"]["hints"]:
        print("提示: %s" % hint)
    return 0 if report["ok"] else 3


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
