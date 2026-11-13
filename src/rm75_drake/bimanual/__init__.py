"""双臂层（S04）：双臂几何装配与标定链。

分层
----
* :mod:`rm75_drake.bimanual.model_variants` —— **双臂几何变体生成器**：
  从 S01 的官方改造版 URDF 派生出「每臂一份重命名 URDF 变体」
  （``models/rm75_drake/{rm75,rmg24}_{left,right}.urdf``）。
  变体**只改名**（robot / link / joint / collision），**不改任何数值**
  （惯量、限位、碰撞几何与原点逐字段等于源文件）。

**为什么必须走「变体」路线（实测钉死，S04-T01）**

Drake 1.51.1 与 1.55.0 的 ``Parser.AddModels`` 在**同一个 plant** 里对同一份 URDF
只允许一个 model instance；第二次调用抛::

    RuntimeError: This model already contains a model instance named 'RM75-B'.
    Model instance names must be unique within a given model.

而两档 Drake 的 ``AddModels`` 都**没有** ``simple_name`` 参数（签名只有
``file_name`` / ``url`` / ``file_contents`` + ``file_type``），因此**无法在解析时改名**。
R005 要求「两臂共享同一个 MultibodyPlant，臂间碰撞 / 自碰撞 / 相邻体过滤在
同一次碰撞查询中统一判定」，所以唯一可行路线就是「每臂一份重命名 URDF 变体」。

本子包**零 pydrake 依赖**（只依赖标准库 ``xml.etree.ElementTree``），
因此变体生成与校验可在无 Drake 的环境里全量单测。
"""

from __future__ import annotations

from .model_variants import (
    KINDS,
    SIDES,
    VariantError,
    VariantMismatchError,
    build_variant,
    check_variants,
    source_path,
    variant_path,
    variant_robot_name,
    write_variants,
)

__all__ = [
    "KINDS",
    "SIDES",
    "VariantError",
    "VariantMismatchError",
    "build_variant",
    "check_variants",
    "source_path",
    "variant_path",
    "variant_robot_name",
    "write_variants",
]
