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

# ------------------------------------------------------------------ 归因判别器
# 残差性质分类（残差是什么"性质"，而不是"多大"）
RESIDUAL_KINDS = (
    "within_tolerance",       # 残差在阈值内
    "base_frame_offset",      # 恒定的 base 系平移（基座/世界系刚体偏置）
    "tool_frame_offset",      # 恒定的 tip(Link7) 系偏置（法兰→工具）
    "orientation_dependent",  # 随位形变化的残差（零位 / MDH / 单关节）
    "insufficient_diversity",  # 样本多样性不足 → 不能对偏置性质下结论
)

# 偏差方向一致性阈值：|平均偏差向量| / 最大偏差 ≥ 该值 → 该系下方向一致
CONSISTENCY_THRESHOLD = 0.8

# 样本多样性门槛（任一项不满足 → insufficient_diversity）
#
# 背景：2026-11-19 那次 75.0314 mm 被判成"系统性平移"，根因是 20 个样本
# **其实是同一位形重复**（dev_spread_mm = 0.0468 mm）。重复同一位形既不能
# 证明工具偏置，也不能排除基座系偏置或建模误差 —— 因此多样性是**先决条件**。
DIVERSITY_MIN_DISTINCT_CONFIGS = 3
DIVERSITY_MIN_POSITION_SPREAD_MM = 1.0
DIVERSITY_MIN_ORIENTATION_SPREAD_DEG = 0.5

# 多样性展布统计的样本上限（展布统计是 O(n^2)，样本过多时等距抽样以控制耗时）
DIVERSITY_SPREAD_SAMPLE_CAP = 512

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
def _consistency(vectors):
    """偏差方向一致性 = |平均偏差向量| / 最大偏差模长。

    ≈1 → 所有样本偏差方向一致（该坐标系下是恒定偏置）
    ≈0 → 偏差互相抵消（个别样本/个别关节异常）
    """
    if vectors is None or len(vectors) == 0:
        return None
    arr = np.asarray(vectors, dtype=float).reshape(-1, 3)
    mags = np.linalg.norm(arr, axis=1)
    max_mag = float(mags.max())
    if max_mag <= 1e-12:
        return 0.0
    return float(np.linalg.norm(arr.mean(axis=0)) / max_mag)


def _sample_diversity(joint_configs, positions, rotations):
    """样本多样性统计（distinct 位形数 / 位置展布 / 姿态展布）。

    多样性不足时**不能**对残差性质（工具偏置 vs 基座偏置 vs 建模误差）下结论。

    展布统计是 O(n^2)；样本数超过 ``DIVERSITY_SPREAD_SAMPLE_CAP`` 时按等距
    抽样统计（distinct 位形数仍按全量统计），避免样本数放大带来的耗时平方。
    """
    n = len(positions)
    distinct = 0
    if joint_configs:
        distinct = len({tuple(np.round(np.asarray(c, dtype=float), 6))
                        for c in joint_configs})
    idx = np.arange(n)
    if n > DIVERSITY_SPREAD_SAMPLE_CAP:
        idx = np.unique(np.linspace(0, n - 1,
                                    DIVERSITY_SPREAD_SAMPLE_CAP).astype(int))
    P = np.asarray([np.asarray(positions[i], dtype=float) for i in idx],
                   dtype=float).reshape(-1, 3)
    if len(P) > 1:
        sq = np.sum(P * P, axis=1)
        d2 = np.maximum(sq[:, None] + sq[None, :] - 2.0 * (P @ P.T), 0.0)
        pos_spread = float(np.sqrt(d2.max())) * 1000.0     # m → mm
    else:
        pos_spread = 0.0
    ori_spread = 0.0
    Rs = [rotations[i] for i in idx
          if rotations and rotations[i] is not None]
    if len(Rs) > 1:
        Rf = np.asarray([np.asarray(r, dtype=float) for r in Rs])
        # tr(R_i^T R_j) = <R_i, R_j>_F（逐元素内积）
        tr = np.einsum("ijk,ijk->ij", Rf, Rf)
        cos_t = np.clip((tr - 1.0) / 2.0, -1.0, 1.0)
        ori_spread = float(np.arccos(cos_t).max()) * RAD2DEG
    reasons = []
    if distinct < DIVERSITY_MIN_DISTINCT_CONFIGS:
        reasons.append("distinct 位形数 %d < %d" % (distinct, DIVERSITY_MIN_DISTINCT_CONFIGS))
    if pos_spread < DIVERSITY_MIN_POSITION_SPREAD_MM:
        reasons.append("位置展布 %.4f mm < %.4f mm"
                       % (pos_spread, DIVERSITY_MIN_POSITION_SPREAD_MM))
    if ori_spread < DIVERSITY_MIN_ORIENTATION_SPREAD_DEG:
        reasons.append("姿态展布 %.4f° < %.4f°"
                       % (ori_spread, DIVERSITY_MIN_ORIENTATION_SPREAD_DEG))
    return {
        "samples": int(n),
        "distinct_configs": int(distinct),
        "position_spread_mm": float(pos_spread),
        "orientation_spread_deg": float(ori_spread),
        "sufficient": bool(not reasons),
        "reasons": reasons,
        "spread_computed_on": int(len(idx)),
        "requirements": {
            "min_distinct_configs": DIVERSITY_MIN_DISTINCT_CONFIGS,
            "min_position_spread_mm": DIVERSITY_MIN_POSITION_SPREAD_MM,
            "min_orientation_spread_deg": DIVERSITY_MIN_ORIENTATION_SPREAD_DEG,
            "spread_sample_cap": DIVERSITY_SPREAD_SAMPLE_CAP,
        },
    }


def _offset_estimate(vectors, frame):
    """偏置估计：某坐标系下的平均偏移 + 样本标准差（不确定度）。

    符号口径：``vectors`` 是偏差 ``fk - controller``（mm）；本函数返回的是
    **待施加的修正量** ``controller - fk``（= -偏差均值），因此把它直接写进
    ``calibration.tcp``（tip 系）/ ``calibration.base_to_base``（base 系）后
    再跑一次交叉校验，残差即被压下去。
    """
    arr = np.asarray(vectors, dtype=float).reshape(-1, 3)
    correction = -arr.mean(axis=0)
    if arr.shape[0] > 1:
        std = arr.std(axis=0, ddof=1)
    else:
        std = np.zeros(3)
    return {
        "frame": frame,
        "mean_mm": [float(v) for v in correction],
        "std_mm": [float(v) for v in std],
        "magnitude_mm": float(np.linalg.norm(correction)),
        "uncertainty_mm": float(np.linalg.norm(std)),
        "suggested_offset": {
            "x": float(correction[0]) / 1000.0,
            "y": float(correction[1]) / 1000.0,
            "z": float(correction[2]) / 1000.0,
            "frame": frame,
        },
        "note": ("修正量 = controller - fk 的系内均值（可直接写入 calibration.%s）；"
                 "仅估计平移分量，姿态偏置需另做姿态标定"
                 % ("tcp" if frame == "tip_link7" else "base_to_base")),
    }


def _attribution(devs, tol_mm, tip_devs=None, positions=None, rotations=None,
                 joint_configs=None):
    """按设计 §5.3 给出归因提示，并**判定残差性质**（residual_kind）。

    参数
    ----
    devs : list[np.ndarray]
        逐样本偏差向量（mm，base 系，``fk - controller``）。
    tol_mm : float
        位置判定阈值（mm）。
    tip_devs : list[np.ndarray] | None
        逐样本偏差向量在 **tip(Link7) 系** 的表达（mm），
        由 ``R_link7^T · dev_base`` 得到。
    positions / rotations / joint_configs : list
        自研 FK 的末端位置（m）、末端旋转矩阵、关节向量（rad）——用于多样性统计。

    返回
    ----
    dict：保留既有键（``pattern`` / ``consistency`` / ``mean_vector_mm`` /
    ``mean_dev_mm`` / ``max_dev_mm`` / ``hints`` / ``next_checks`` /
    ``next_check_detail``，**向后兼容**），并新增：

    * ``residual_kind``：残差性质（见 ``RESIDUAL_KINDS``）
    * ``consistency_base`` / ``consistency_tip``：两个坐标系下的方向一致性
    * ``diversity``：样本多样性统计与门槛判定
    * ``offset_estimates``：建议偏置估计（tip / base 系平均偏移 + 不确定度）
    """
    if not devs:
        return {"pattern": "no_samples", "consistency": None,
                "consistency_base": None, "consistency_tip": None,
                "residual_kind": "insufficient_diversity",
                "mean_vector_mm": None, "hints": [], "next_checks": [],
                "next_check_detail": [], "diversity": None,
                "offset_estimates": None}
    arr = np.asarray(devs, dtype=float).reshape(-1, 3)
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

    # ---- base 系 / tip 系 双口径一致性 ----
    consistency_base = _consistency(arr)
    consistency_tip = _consistency(tip_devs)

    # ---- 样本多样性 ----
    diversity = _sample_diversity(joint_configs or [], positions or [],
                                   rotations or [])
    if not positions:
        diversity["sufficient"] = False
        diversity["reasons"] = list(diversity["reasons"]) or ["缺少 FK 位姿样本"]

    # ---- 残差性质判定 ----
    if not diversity["sufficient"]:
        residual_kind = "insufficient_diversity"
    elif max_mag <= tol_mm:
        residual_kind = "within_tolerance"
    elif consistency_tip is None:
        residual_kind = "orientation_dependent"
    elif (consistency_tip >= CONSISTENCY_THRESHOLD
          and consistency_base < CONSISTENCY_THRESHOLD):
        residual_kind = "tool_frame_offset"
    elif (consistency_base >= CONSISTENCY_THRESHOLD
          and consistency_tip < CONSISTENCY_THRESHOLD):
        residual_kind = "base_frame_offset"
    else:
        residual_kind = "orientation_dependent"

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

    # ---- 残差性质提示（判别器口径）----
    if residual_kind == "within_tolerance":
        hints.append("residual_kind=within_tolerance：残差在阈值内，无需标定")
    elif residual_kind == "insufficient_diversity":
        hints.append("residual_kind=insufficient_diversity：样本多样性不足"
                     "（%s）；**同一位形重复采样不能判定偏置性质**，"
                     "请补采有界多样位形后复测"
                     % ("；".join(diversity["reasons"]) or "位形/展布未达门槛"))
    elif residual_kind == "base_frame_offset":
        hints.append("residual_kind=base_frame_offset：偏差在 base 系方向一致"
                     "（consistency_base=%.2f）而在 tip 系不一致"
                     "（consistency_tip=%.2f）→ 基座/世界系刚体偏置："
                     "查 URDF 清洗、基座安装，或改标 calibration.base_to_base"
                     % (consistency_base, consistency_tip))
    elif residual_kind == "tool_frame_offset":
        hints.append("residual_kind=tool_frame_offset：偏差在 tip(Link7) 系方向一致"
                     "（consistency_tip=%.2f）而在 base 系不一致"
                     "（consistency_base=%.2f）→ 法兰→工具偏置："
                     "标定 calibration.tcp 后复测"
                     % (consistency_tip, consistency_base))
    else:
        hints.append("residual_kind=orientation_dependent：偏差随位形变化"
                     "（consistency_base=%.2f / consistency_tip=%.2f）→ "
                     "查零位与 MDH 偏置" % (consistency_base, consistency_tip))
        if (consistency_base >= CONSISTENCY_THRESHOLD
                and consistency_tip >= CONSISTENCY_THRESHOLD):
            hints.append("base 与 tip 系一致性同时很高：位形变化不足以区分坐标系，"
                         "结论只能取 orientation_dependent（不写任何偏置）")

    offset_estimates = None
    if tip_devs is not None and len(tip_devs):
        offset_estimates = {
            "tip_frame": _offset_estimate(tip_devs, "tip_link7"),
            "base_frame": _offset_estimate(arr, "base"),
        }

    return {
        "pattern": pattern,
        "consistency": consistency,
        "consistency_base": consistency_base,
        "consistency_tip": consistency_tip,
        "residual_kind": residual_kind,
        "mean_vector_mm": [float(v) for v in mean_vec],
        "mean_dev_mm": mean_mag,
        "max_dev_mm": max_mag,
        "diversity": diversity,
        "offset_estimates": offset_estimates,
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
    tip_devs = []
    fk_positions = []
    fk_rotations = []
    joint_configs = []
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
        fk_rot = np.asarray(T[:3, :3], dtype=float)
        dev = (fk_pos - ctrl_pos) * 1000.0            # mm
        dev_mm = float(np.linalg.norm(dev))
        devs.append(dev)
        # tip(Link7) 系偏差：R_link7^T · dev_base（把 base 系偏差转到法兰系）
        tip_devs.append(fk_rot.T @ dev)
        fk_positions.append(fk_pos)
        fk_rotations.append(fk_rot)
        joint_configs.append(q)

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
        "attribution": _attribution(devs, pos_tol_mm, tip_devs=tip_devs,
                                    positions=fk_positions, rotations=fk_rotations,
                                    joint_configs=joint_configs),
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
    * ``residual_kind``：残差性质（within_tolerance / base_frame_offset /
      tool_frame_offset / orientation_dependent / insufficient_diversity）
    * ``diversity``：样本多样性统计（distinct 位形数 / 位置展布 / 姿态展布）
    * ``offset_estimates``：建议偏置估计（tip / base 系平均偏移 + 不确定度）

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
    attribution = report.get("attribution") or {}
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
    if attribution:
        fields["residual_kind"] = attribution.get("residual_kind")
        fields["attribution_pattern"] = attribution.get("pattern")
        fields["consistency_base"] = attribution.get("consistency_base")
        fields["consistency_tip"] = attribution.get("consistency_tip")
        diversity = attribution.get("diversity")
        if diversity:
            fields["diversity"] = {
                "distinct_configs": diversity.get("distinct_configs"),
                "position_spread_mm": diversity.get("position_spread_mm"),
                "orientation_spread_deg": diversity.get("orientation_spread_deg"),
                "sufficient": diversity.get("sufficient"),
            }
        estimates = attribution.get("offset_estimates")
        if estimates:
            fields["offset_estimates"] = estimates
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
