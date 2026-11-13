"""声明式错误码表（控制器 ``arm_err`` → 故障类 / 安全等级）。

错误码映射**不硬编码在 Python 里**（与「契约即唯一真值」一致）：真值文件是
``workcell/safety/error_codes.yaml``，由 :func:`load_error_codes` 读取。

保守默认（**不静默**）
--------------------

* 未知码 → ``class="unknown"``；
* 未知码 → 等级按 **L3 缓停**（保守：可人工复位，但不自动清错误码）；
* 文件缺失 / YAML 格式错 → 返回**空表 + 保守默认 + 明确 warning**，
  **绝不抛裸异常**、也**绝不静默**（warning 会随结果一起上抛给调用方）。

每条码都必须带 ``source``（来源可追溯）；缺 ``source`` 的条目会保留但
产生 warning（不静默丢码），缺 ``class`` 的条目按 ``unknown`` 处理。
"""
from __future__ import annotations

import os
from typing import Any, Dict, Iterable, List, Optional, Tuple

import yaml

__all__ = [
    "DEFAULT_UNKNOWN_CLASS",
    "DEFAULT_UNKNOWN_LEVEL",
    "ErrorCodeMap",
    "load_error_codes",
]

#: 未知错误码的默认故障类
DEFAULT_UNKNOWN_CLASS = "unknown"

#: 未知错误码的默认等级（保守：L3 缓停）
DEFAULT_UNKNOWN_LEVEL = "L3"


def _as_int(code: Any) -> Optional[int]:
    """把错误码转成 int；无法转换返回 ``None``（调用方记 warning）。"""
    if isinstance(code, bool):
        return None
    if isinstance(code, int):
        return code
    if isinstance(code, float) and float(code).is_integer():
        return int(code)
    if isinstance(code, str):
        text = code.strip()
        try:
            return int(text, 10)
        except ValueError:
            return None
    return None


class ErrorCodeMap:
    """错误码 → 故障类 / 等级的**不可变**映射表。"""

    def __init__(self, codes: Optional[Dict[int, Dict[str, Any]]] = None,
                 default_class: str = DEFAULT_UNKNOWN_CLASS,
                 default_level: str = DEFAULT_UNKNOWN_LEVEL,
                 warnings: Optional[Iterable[str]] = None,
                 source: str = "",
                 status: str = "") -> None:
        self._codes: Dict[int, Dict[str, Any]] = dict(codes or {})
        self.default_class = str(default_class or DEFAULT_UNKNOWN_CLASS)
        self.default_level = str(default_level or DEFAULT_UNKNOWN_LEVEL)
        self.warnings: Tuple[str, ...] = tuple(warnings or ())
        self.source = str(source or "")
        self.status = str(status or "")

    # ------------------------------------------------------------------ 构造
    @classmethod
    def from_dict(cls, data: Any) -> "ErrorCodeMap":
        """从已解析的 YAML 字典构造映射表（**不抛异常**，问题记 warning）。

        支持两种 ``codes`` 形态：

        * 列表形态（**首选**）：``codes: [{code: 1, class: collision, ...}]``；
        * 字典形态（兼容）：``codes: {1: {class: collision, ...}}``。
        """
        warnings: List[str] = []
        if data is None:
            return cls(warnings=["错误码表为空（文件内容为 null）"])
        if not isinstance(data, dict):
            return cls(warnings=[
                "错误码表格式错误（顶层必须是映射，当前是 %s）" % type(data).__name__
            ])

        defaults = data.get("defaults")
        default_class = DEFAULT_UNKNOWN_CLASS
        default_level = DEFAULT_UNKNOWN_LEVEL
        if isinstance(defaults, dict):
            default_class = str(defaults.get("unknown_class") or default_class)
            default_level = str(defaults.get("unknown_level") or default_level)
        elif defaults is not None:
            warnings.append("defaults 段格式错误（必须是映射），已用保守默认值")

        raw_codes = data.get("codes")
        entries: List[Any] = []
        if isinstance(raw_codes, list):
            entries = list(raw_codes)
        elif isinstance(raw_codes, dict):
            for key, value in raw_codes.items():
                item = dict(value) if isinstance(value, dict) else {}
                item.setdefault("code", key)
                entries.append(item)
        elif raw_codes is None:
            warnings.append("错误码表缺少 codes 段（已用保守默认值：未知码 → %s）"
                            % default_level)
        else:
            warnings.append("错误码表 codes 段格式错误（当前是 %s），已忽略"
                            % type(raw_codes).__name__)

        codes: Dict[int, Dict[str, Any]] = {}
        for entry in entries:
            if not isinstance(entry, dict):
                warnings.append("错误码条目格式错误（当前是 %s），已忽略"
                                % type(entry).__name__)
                continue
            code = _as_int(entry.get("code"))
            if code is None:
                warnings.append("错误码条目缺少合法 code（当前 %r），已忽略"
                                % (entry.get("code"),))
                continue
            if code in codes:
                warnings.append("错误码 %d 重复定义，后一条覆盖前一条" % code)
            code_class = str(entry.get("class") or "").strip()
            if not code_class:
                code_class = default_class
                warnings.append("错误码 %d 缺少 class，按默认类 %r 处理"
                                % (code, default_class))
            level_hint = str(entry.get("level_hint") or "").strip()
            if not level_hint:
                level_hint = default_level
                warnings.append("错误码 %d 缺少 level_hint，按保守等级 %r 处理"
                                % (code, default_level))
            source = str(entry.get("source") or "").strip()
            if not source:
                warnings.append("错误码 %d 缺少 source（来源不可追溯，保留但需补齐）"
                                % code)
            codes[code] = {
                "code": code,
                "class": code_class,
                "level_hint": level_hint,
                "note": str(entry.get("note") or ""),
                "source": source,
            }

        return cls(
            codes=codes,
            default_class=default_class,
            default_level=default_level,
            warnings=warnings,
            source=str(data.get("source") or ""),
            status=str(data.get("status") or ""),
        )

    # ------------------------------------------------------------------ 查询
    @property
    def codes(self) -> Dict[int, Dict[str, Any]]:
        """码表本体（只读副本）。"""
        return dict(self._codes)

    def lookup(self, code: Any) -> Optional[Dict[str, Any]]:
        """查一条码的定义；未知码返回 ``None``。"""
        key = _as_int(code)
        if key is None:
            return None
        item = self._codes.get(key)
        return dict(item) if item else None

    def classify(self, code: Any) -> str:
        """码 → 故障类；未知码返回默认类（``unknown``）。"""
        item = self.lookup(code)
        if item is None:
            return self.default_class
        return str(item.get("class") or self.default_class)

    def level_for(self, code: Any) -> str:
        """码 → 等级；未知码返回默认等级（``L3``，保守）。"""
        item = self.lookup(code)
        if item is None:
            return self.default_level
        return str(item.get("level_hint") or self.default_level)

    def known(self, code: Any) -> bool:
        """该码是否在表内。"""
        return self.lookup(code) is not None

    def __len__(self) -> int:
        return len(self._codes)

    def __repr__(self) -> str:  # pragma: no cover - 调试用
        return "<ErrorCodeMap codes=%d default_level=%s warnings=%d>" % (
            len(self._codes), self.default_level, len(self.warnings))


def load_error_codes(path: str) -> ErrorCodeMap:
    """读取声明式错误码表（**不抛裸异常**）。

    参数
    ----
    path
        YAML 文件路径。

    返回
    ----
    ErrorCodeMap
        读成功：表内容 + ``status`` / ``source``；读失败（缺文件 / 格式错）：
        **空表 + 保守默认**（未知码 → ``unknown`` / ``L3``）+ ``warnings``
        明确说明失败原因（**不静默**）。
    """
    path = os.path.abspath(path)
    if not os.path.isfile(path):
        return ErrorCodeMap(
            warnings=[
                "错误码表不存在: %s（已用保守默认：未知码 → class=unknown，"
                "level=L3）" % path
            ],
            source=path,
        )
    try:
        with open(path, "r", encoding="utf-8") as fh:
            data = yaml.safe_load(fh)
    except Exception as exc:  # noqa: BLE001 - 任何解析异常统一转保守默认
        return ErrorCodeMap(
            warnings=[
                "错误码表无法解析（%s: %s）: %s（已用保守默认：未知码 → "
                "class=unknown，level=L3）"
                % (type(exc).__name__, exc, path)
            ],
            source=path,
        )
    table = ErrorCodeMap.from_dict(data)
    if not table.source:
        table.source = path
    table.warnings = tuple(
        list(table.warnings) + ["错误码表来源文件: %s" % path]
    )
    return table
