"""双臂层（S04）：双臂几何装配与标定链。

分层
----
* :mod:`rm75_drake.bimanual.model_variants` —— **双臂几何变体生成器**：
  从 S01 的官方改造版 URDF 派生出「每臂一份重命名 URDF 变体」
  （``models/rm75_drake/{rm75,rmg24}_{left,right}.urdf``）。
  变体**只改名**（robot / link / joint / collision），**不改任何数值**
  （惯量、限位、碰撞几何与原点逐字段等于源文件）。
* :mod:`rm75_drake.bimanual.dual_scene` —— **双臂单一 MultibodyPlant 装配器**：
  把两臂（+ 可选两夹爪）装进**同一个 plant**，臂间碰撞 / 自碰撞 / 相邻体过滤
  在**同一次碰撞查询**中统一判定（R005）；并把「q 维度必须等于
  ``plant.num_positions()``（否则 Drake 直接 abort）」收口成可捕获的
  :class:`SceneError`。
* :mod:`rm75_drake.bimanual.collision_filter` —— **双臂碰撞过滤矩阵**：
  程序化地把「同臂相邻过滤 / 左右臂互检 / 夹爪指 vs 被抓物体放行 / 夹爪指 vs
  本臂掌部放行」四条规则翻译成 Drake 的
  ``CollisionFilterDeclaration`` 并**真正施加到场景图**（碰撞判定结果会变，
  不是只记日志）。
* :mod:`rm75_drake.bimanual.consistency` —— **共同点一致性判定器**：
  取两臂 TCP 在 world 系的位姿，给出「位置偏差 < 阈值（默认 0.5 mm，ROADMAP
  口径）」的 PASS / FAIL 结论；标定未实测时**只能**给「未验证」（绝不冒充
  物理达标）。

**为什么必须走「变体」路线（实测钉死，S04-T01）**

Drake 1.51.1 与 1.55.0 的 ``Parser.AddModels`` 在**同一个 plant** 里对同一份 URDF
只允许一个 model instance；第二次调用抛::

    RuntimeError: This model already contains a model instance named 'RM75-B'.
    Model instance names must be unique within a given model.

而两档 Drake 的 ``AddModels`` 都**没有** ``simple_name`` 参数（签名只有
``file_name`` / ``url`` / ``file_contents`` + ``file_type``），因此**无法在解析时改名**。
R005 要求「两臂共享同一个 MultibodyPlant，臂间碰撞 / 自碰撞 / 相邻体过滤在
同一次碰撞查询中统一判定」，所以唯一可行路线就是「每臂一份重命名 URDF 变体」。

本子包的**变体生成器**部分**零 pydrake 依赖**（只依赖标准库
``xml.etree.ElementTree``），因此变体生成与校验可在无 Drake 的环境里全量单测；
``dual_scene`` 依赖 pydrake，**惰性导入**（PEP 562），import 本包不会拉起 Drake。
"""

from __future__ import annotations

from typing import Any

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
    "dual_scene",
    "collision_filter",
    "consistency",
]


def __getattr__(name: str) -> Any:
    """PEP 562 惰性子模块导入（``dual_scene`` 依赖 pydrake，import 时不拉起）。"""
    if name in ("dual_scene", "collision_filter", "consistency"):
        import importlib

        module = importlib.import_module(f"{__name__}.{name}")
        globals()[name] = module
        return module
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
