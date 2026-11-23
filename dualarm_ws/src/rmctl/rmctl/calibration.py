"""标定偏置的结构化 schema 与解析（FK 坐标系口径的唯一落点）。

## 为什么要有这个模块

``workcell/rm_workcell.yaml::calibration`` 里的 ``tcp`` / ``base_to_base``
此前**既没有 schema 也没有消费者**：填了也不会影响任何数值计算。本模块把
"填什么形状、怎么读、怎么施加"三件事一次定死，``rmctl.fk_cross`` 与
``rmctl.hwcollect`` 共用同一套口径。

## 坐标系口径（不可互换）

| 字段 | 含义 | 表达所在系 | 施加方式 |
|------|------|------------|----------|
| ``tcp`` | 法兰→工具偏置 | **tip(Link7) 系** | ``T_corrected = T_link7 · T_tool`` |
| ``base_to_base`` | 基座/世界系刚体偏置 | **base 系** | ``T_corrected = T_base · T_link7`` |

两者都**同时**作用于位置与姿态（姿态比较用变换后的旋转矩阵）。左乘
（base）与右乘（tip）互可交换，因此同时施加时无先后歧义。

前提（来自《睿尔曼RM75-B底层接口双臂运动规划软件整体规划设计》§黑名单表）：
**控制器 TCP 保持默认法兰**，即控制器上报的是法兰系位姿；"工具偏置"因此
表达在法兰（tip）系里，而不是 base 系里。

## 未标定一律 null

没有实测就没有偏置。解析到 ``None`` 就是"未标定"，调用方必须显式处理，
**禁止用默认值/猜测值顶替**。

本模块只依赖标准库 + PyYAML + numpy，不 import 任何厂商库。
"""
import json
import math
import os

import numpy as np
import yaml

# 偏置对象的字段（结构化 schema；对应 workcell/rm_workcell.yaml 的注释）
OFFSET_FIELDS = (
    "frame",        # "tip_link7"（tcp）| "base"（base_to_base）
    "x", "y", "z",  # 平移（m）
    "rpy",          # 姿态 [rx, ry, rz]（rad，Rz*Ry*Rx）
    "method",       # 标定方法（人类可读）
    "source",       # 数据来源（工件路径/现场记录…）
    "captured_at",  # 采集时间（ISO8601）
    "uncertainty_mm",  # 不确定度（mm）
)

# 各偏置允许的 frame 标识
FRAME_ALIASES = {
    "tool": ("tip_link7", "tip", "link7", "link7_tip"),
    "base": ("base", "base_link", "world"),
}

DEFAULT_FRAMES = {"tool": "tip_link7", "base": "base"}


class CalibrationError(Exception):
    """标定偏置非法（消息面向操作者，可直接粘贴）。"""


def _num(value, what):
    try:
        v = float(value)
    except (TypeError, ValueError):
        raise CalibrationError("%s 无法解析为数值: %r" % (what, value))
    if not math.isfinite(v):
        raise CalibrationError("%s 含 NaN/Inf: %r" % (what, value))
    return v


def _parse_rpy(value):
    """rpy → [rx, ry, rz]（rad）。缺省为全零；允许 {"r","p","y"} 字典。"""
    if value is None:
        return [0.0, 0.0, 0.0]
    if isinstance(value, dict):
        try:
            return [_num(value["r"], "rpy.r"), _num(value["p"], "rpy.p"),
                    _num(value["y"], "rpy.y")]
        except KeyError as exc:
            raise CalibrationError("rpy 字典缺字段 %s" % exc)
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise CalibrationError("rpy 必须是 3 元列表（rad）: %r" % (value,))
    return [_num(value[0], "rpy[0]"), _num(value[1], "rpy[1]"), _num(value[2], "rpy[2]")]


def normalize_offset(data, kind, source=None):
    """把任意来源的偏置数据规范成统一结构（返回 ``None`` 表示未标定）。

    ``kind``：``"tool"``（tip 系偏置）或 ``"base"``（base 系偏置）。
    ``data``：dict（含 x/y/z 或 xyz），或 None。
    """
    if kind not in FRAME_ALIASES:
        raise CalibrationError("未知偏置类型 %r（只支持 tool / base）" % (kind,))
    if data is None:
        return None
    if not isinstance(data, dict):
        raise CalibrationError("偏置必须是对象（dict），收到 %s" % type(data).__name__)

    d = dict(data)
    xyz = d.get("xyz")
    if xyz is None:
        missing = [k for k in ("x", "y", "z") if d.get(k) is None]
        if missing:
            raise CalibrationError("偏置缺平移字段 %s" % ", ".join(missing))
        xyz = [d["x"], d["y"], d["z"]]
    if not isinstance(xyz, (list, tuple)) or len(xyz) != 3:
        raise CalibrationError("偏置平移必须是 3 元（m）: %r" % (xyz,))
    x, y, z = [_num(v, "平移") for v in xyz]

    rpy = _parse_rpy(d.get("rpy"))

    frame = d.get("frame")
    if frame is None:
        frame = DEFAULT_FRAMES[kind]
    else:
        frame = str(frame).strip().lower()
        if frame not in FRAME_ALIASES[kind]:
            raise CalibrationError(
                "偏置 frame=%r 与类型 %s 不匹配（允许: %s）"
                % (d.get("frame"), kind, ", ".join(FRAME_ALIASES[kind])))

    return {
        "kind": kind,
        "frame": frame,
        "x": x,
        "y": y,
        "z": z,
        "rpy": rpy,
        "method": d.get("method"),
        "source": d.get("source") or source,
        "captured_at": d.get("captured_at"),
        "uncertainty_mm": (None if d.get("uncertainty_mm") is None
                           else _num(d.get("uncertainty_mm"), "uncertainty_mm")),
    }


def offset_transform(offset):
    """偏置 → 4x4 齐次变换矩阵（None → None）。"""
    if offset is None:
        return None
    r, p, y = offset["rpy"]
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    T = np.eye(4)
    T[:3, :3] = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])
    T[:3, 3] = [offset["x"], offset["y"], offset["z"]]
    return T


def _load_structured_file(path):
    """读 JSON / YAML 文件里的偏置对象（dict 或 3/6 元列表）。"""
    if not os.path.exists(path):
        raise CalibrationError("偏置文件不存在: %s" % path)
    try:
        with open(path, "r", encoding="utf-8") as fh:
            text = fh.read()
    except OSError as exc:
        raise CalibrationError("偏置文件读取失败 (%s): %r" % (path, exc))
    try:
        data = json.loads(text)
    except ValueError:
        try:
            data = yaml.safe_load(text)
        except yaml.YAMLError as exc:
            raise CalibrationError("偏置文件解析失败 (%s): %r" % (path, exc))
    if isinstance(data, dict) and "offset" in data and len(data) == 1:
        data = data["offset"]
    if isinstance(data, (list, tuple)):
        vals = [_num(v, "偏置分量") for v in data]
        if len(vals) == 3:
            return {"x": vals[0], "y": vals[1], "z": vals[2]}
        if len(vals) == 6:
            return {"x": vals[0], "y": vals[1], "z": vals[2],
                    "rpy": vals[3:6]}
        raise CalibrationError("偏置列表长度必须是 3 或 6: %d" % len(vals))
    if not isinstance(data, dict):
        raise CalibrationError("偏置文件内容必须是对象或列表: %s" % path)
    return data


def parse_offset_spec(spec, kind, source=None):
    """解析命令行给的偏置：文件路径 | ``"x,y,z[,rx,ry,rz]"`` | dict | None。

    返回规范化后的偏置 dict；``spec`` 为 None 时返回 None（未标定）。
    """
    if spec is None:
        return None
    if isinstance(spec, dict):
        return normalize_offset(spec, kind, source=source)
    if not isinstance(spec, str):
        raise CalibrationError("偏置输入必须是路径/字符串/dict，收到 %s"
                               % type(spec).__name__)
    text = spec.strip()
    if not text:
        raise CalibrationError("偏置输入为空串")
    if os.path.exists(text):
        data = _load_structured_file(text)
        return normalize_offset(data, kind, source=source or ("file:%s" % text))
    parts = [p.strip() for p in text.split(",") if p.strip() != ""]
    if len(parts) not in (3, 6):
        raise CalibrationError(
            "偏置字符串格式应为 'x,y,z' 或 'x,y,z,rx,ry,rz'（m / rad），收到 %r" % spec)
    vals = []
    for i, p in enumerate(parts):
        try:
            vals.append(float(p))
        except ValueError:
            raise CalibrationError("偏置第 %d 个分量无法解析为数值: %r" % (i + 1, p))
    data = {"x": vals[0], "y": vals[1], "z": vals[2]}
    if len(vals) == 6:
        data["rpy"] = vals[3:6]
    return normalize_offset(data, kind, source=source or "inline")


def load_workcell_calibration(path, source=None):
    """读 workcell yaml 的 ``calibration`` 段 → ``{"tcp":…, "base_to_base":…}``。

    ``None`` 表示该字段未标定（yaml 里写 null）——**不得**用默认值顶替。
    """
    if not os.path.exists(path):
        raise CalibrationError("workcell 文件不存在: %s" % path)
    try:
        with open(path, "r", encoding="utf-8") as fh:
            data = yaml.safe_load(fh)
    except (OSError, yaml.YAMLError) as exc:
        raise CalibrationError("workcell 解析失败 (%s): %r" % (path, exc))
    if not isinstance(data, dict):
        raise CalibrationError("workcell 文件内容必须是映射: %s" % path)
    cal = data.get("calibration")
    if cal is None:
        raise CalibrationError("workcell 文件缺 calibration 段: %s" % path)
    if not isinstance(cal, dict):
        raise CalibrationError("calibration 段必须是映射: %s" % path)
    return {
        "tcp": normalize_offset(cal.get("tcp"), "tool",
                                source=source or "workcell:calibration.tcp"),
        "base_to_base": normalize_offset(cal.get("base_to_base"), "base",
                                         source=source or "workcell:calibration.base_to_base"),
    }
