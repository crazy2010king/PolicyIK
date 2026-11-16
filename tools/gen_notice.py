#!/usr/bin/env python3
"""tools/gen_notice.py —— 由 licenses/registry.yaml 派生仓库根 NOTICE（S07-T04）

设计要点
--------
1. **NOTICE 只能是 registry 的派生物**：本脚本是 NOTICE 的唯一生成器，
   ``NOTICE`` 文件头写明来源；``--check`` 用来在 CI 里证明"仓库里的 NOTICE 与
   登记表仍然一致"（防止手工改 NOTICE 或改 registry 后忘记重新生成）。
2. **分类口径唯一**：同一套分类函数既供 NOTICE 渲染，也供
   ``tools/verify_env.py --licenses`` 做门禁校验 ⇒ NOTICE 段落与门禁口径
   永远一致（不会出现"NOTICE 里写着允许再分发、门禁却判为受限"的分裂）。
3. **确定性输出**：条目按归一化名字排序，不含时间戳 ⇒ 同样输入必然逐字节相同，
   ``--check`` 才能做逐字比较。

分类（三段，互斥且全覆盖）
--------------------------
    1) 允许再分发          —— ``redistributable: true`` 且 commit 已锁定
    2) 受限、禁止再分发    —— ``redistributable: false``
    3) commit 未锁定例外   —— ``commit: null`` 且 ``locked: false``（方法论/范式
                              参考类资产；**不得编造 commit**）

退出码：0 通过（--check 一致 / 写盘成功）；1 不一致或写盘失败；2 参数错误。
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_REGISTRY = REPO_ROOT / "licenses" / "registry.yaml"
DEFAULT_NOTICE = REPO_ROOT / "NOTICE"

#: 登记表每条 entry 的必填字段（与 tests/test_env_lock.py 的口径一致）。
REQUIRED_ENTRY_FIELDS: Tuple[str, ...] = (
    "name",
    "version",
    "license",
    "source",
    "redistributable",
    "usage_note",
)

HEX40_RE = re.compile(r"^[0-9a-f]{40}$")


# --------------------------------------------------------------------------- #
# 登记表语义（唯一实现；verify_env 也复用这里的函数）
# --------------------------------------------------------------------------- #
def normalize_component_name(name: Any) -> str:
    """组件名归一化：小写 + 连字符/下划线等价。

    ⇒ ``Two-Finger-Gripper``、``two_finger_gripper``、``Two_Finger_Gripper``
    归一化为同一个键；``python-dateutil`` 与 ``python_dateutil`` 同理。
    """
    raw = str(name or "").strip().lower()
    return re.sub(r"[-_]+", "_", raw)


def load_registry(path: Path) -> Dict[str, Any]:
    """读取并解析 ``licenses/registry.yaml``（缺 PyYAML 时给明确错误）。"""
    try:
        import yaml
    except ImportError as exc:  # pragma: no cover - 环境缺依赖
        raise RuntimeError("需要 PyYAML（pip install pyyaml）") from exc
    if not Path(path).is_file():
        raise RuntimeError(f"许可登记表不存在：{path}")
    with Path(path).open("r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh) or {}
    if not isinstance(data, dict):
        raise RuntimeError(f"许可登记表格式非法（应为映射）：{path}")
    return data


def registry_entries(registry: Dict[str, Any]) -> List[Dict[str, Any]]:
    """取出 entries 列表（只保留映射型条目）。"""
    entries = registry.get("entries")
    if not isinstance(entries, list):
        return []
    return [entry for entry in entries if isinstance(entry, dict)]


def registry_index(registry: Dict[str, Any]) -> Dict[str, Dict[str, Any]]:
    """按归一化名建立索引；同名（归一化后）条目后者覆盖前者（重复由门禁点名）。"""
    index: Dict[str, Dict[str, Any]] = {}
    for entry in registry_entries(registry):
        name = entry.get("name")
        if not name:
            continue
        index[normalize_component_name(name)] = entry
    return index


def duplicate_names(registry: Dict[str, Any]) -> List[str]:
    """归一化后重名的条目名（返回原始名列表）。"""
    seen: Dict[str, str] = {}
    dups: List[str] = []
    for entry in registry_entries(registry):
        name = entry.get("name")
        if not name:
            continue
        key = normalize_component_name(name)
        if key in seen:
            dups.append(str(name))
        else:
            seen[key] = str(name)
    return dups


def is_unlocked_exception(entry: Dict[str, Any]) -> bool:
    """是否为「commit 未锁定例外」（声明了 commit 且值为 null）。"""
    return "commit" in entry and entry.get("commit") is None


def classify_entries(
    registry: Dict[str, Any],
) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]], List[Dict[str, Any]]]:
    """把登记表条目分成三段。

    返回 ``(redistributable, restricted, unlocked)``：

    * ``redistributable`` —— 允许再分发（``redistributable: true`` 且 commit 已锁定）；
    * ``restricted`` —— 受限、禁止再分发（``redistributable: false``）；
    * ``unlocked`` —— commit 未锁定例外（``commit: null`` 且 ``locked: false``）。

    未锁定例外优先于 redistributable 判定（它单列一段），因此三段互斥。
    """
    redistributable: List[Dict[str, Any]] = []
    restricted: List[Dict[str, Any]] = []
    unlocked: List[Dict[str, Any]] = []
    for entry in registry_entries(registry):
        if is_unlocked_exception(entry):
            unlocked.append(entry)
        elif bool(entry.get("redistributable")):
            redistributable.append(entry)
        else:
            restricted.append(entry)
    key = lambda e: normalize_component_name(e.get("name"))  # noqa: E731
    redistributable.sort(key=key)
    restricted.sort(key=key)
    unlocked.sort(key=key)
    return redistributable, restricted, unlocked


def _one_line(text: Any) -> str:
    """把 YAML 折叠块压成单行（确定性渲染用）。"""
    return " ".join(str(text or "").split())


def _entry_block(entry: Dict[str, Any], bullet: str = "-") -> List[str]:
    """渲染单条目（逐字稳定的文本块）。"""
    lines = [f"{bullet} **{entry.get('name')}** — 版本 {_one_line(entry.get('version'))}"]
    lines.append(f"  - 许可：{_one_line(entry.get('license'))}")
    lines.append(f"  - 来源：{_one_line(entry.get('source'))}")
    if "commit" in entry:
        commit = entry.get("commit")
        lines.append(
            f"  - 上游 commit：{commit if commit else '未锁定（commit: null）'}"
            f"（locked: {str(bool(entry.get('locked'))).lower()}）"
        )
    lines.append(f"  - 用途说明：{_one_line(entry.get('usage_note'))}")
    return lines


def render_notice(registry: Dict[str, Any]) -> str:
    """由登记表渲染 NOTICE 全文（确定性、逐字可复现）。"""
    redistributable, restricted, unlocked = classify_entries(registry)
    lines: List[str] = []
    lines.append("# NOTICE —— 第三方组件与上游资产声明")
    lines.append("#")
    lines.append("# 本文件由 `tools/gen_notice.py` 从 `licenses/registry.yaml` **自动生成**，")
    lines.append("# 请勿手工编辑；改登记表后请重新生成：")
    lines.append("#")
    lines.append("#     python3 tools/gen_notice.py --out NOTICE")
    lines.append("#")
    lines.append("# CI 校验（逐字比较）：")
    lines.append("#")
    lines.append("#     python3 tools/gen_notice.py --check")
    lines.append("#")
    lines.append("# 分类口径（与 licenses/registry.yaml、tools/verify_env.py --licenses 一致）：")
    lines.append("#")
    lines.append("#   1) 允许再分发        —— `redistributable: true` 且 commit 已锁定")
    lines.append("#   2) 受限、禁止再分发  —— `redistributable: false`")
    lines.append("#   3) commit 未锁定例外 —— `commit: null` 且 `locked: false`")
    lines.append("")
    lines.append("## 1. 允许再分发（redistributable: true）")
    lines.append("")
    if redistributable:
        for entry in redistributable:
            lines.extend(_entry_block(entry))
    else:
        lines.append("（无）")
    lines.append("")
    lines.append("## 2. 受限、禁止再分发（redistributable: false）")
    lines.append("")
    if restricted:
        for entry in restricted:
            lines.extend(_entry_block(entry))
    else:
        lines.append("（无）")
    lines.append("")
    lines.append("## 3. commit 未锁定例外（locked: false）")
    lines.append("")
    lines.append("以下条目仅作方法论 / 范式参考，未锁定上游 commit（**不得编造 commit**）；")
    lines.append("若后续直接搬运其代码，必须先补登记真实 commit 并重新生成本文件。")
    lines.append("")
    if unlocked:
        for entry in unlocked:
            lines.extend(_entry_block(entry))
    else:
        lines.append("（无）")
    lines.append("")
    return "\n".join(lines)


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #
def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        prog="gen_notice.py",
        description="由 licenses/registry.yaml 生成 / 校验仓库根 NOTICE",
    )
    parser.add_argument("--registry", type=Path, default=DEFAULT_REGISTRY, help="许可登记表路径")
    parser.add_argument("--out", type=Path, default=None, help="NOTICE 输出路径（默认打印到 stdout）")
    parser.add_argument(
        "--check",
        action="store_true",
        help="只校验现有 NOTICE 与登记表派生结果是否逐字一致（不一致退出码 1）",
    )
    args = parser.parse_args(list(argv) if argv is not None else None)

    try:
        registry = load_registry(args.registry)
    except Exception as exc:
        print(f"[gen_notice][ERROR] 读取许可登记表失败：{exc}", file=sys.stderr)
        return 2

    content = render_notice(registry)
    target = args.out or (args.check and DEFAULT_NOTICE) or None

    if args.check:
        notice_path = Path(args.out) if args.out else DEFAULT_NOTICE
        if not notice_path.is_file():
            print(
                f"[gen_notice][FAIL] NOTICE 不存在：{notice_path}"
                "（请执行 python3 tools/gen_notice.py --out NOTICE）",
                file=sys.stderr,
            )
            return 1
        existing = notice_path.read_text(encoding="utf-8")
        if existing == content:
            print(f"[gen_notice] OK  {notice_path} 与 licenses/registry.yaml 一致")
            return 0
        print(
            f"[gen_notice][FAIL] {notice_path} 与 licenses/registry.yaml 派生结果不一致"
            "（请执行 python3 tools/gen_notice.py --out NOTICE）",
            file=sys.stderr,
        )
        return 1

    if target is not None:
        Path(target).parent.mkdir(parents=True, exist_ok=True)
        Path(target).write_text(content, encoding="utf-8")
        print(f"[gen_notice] 已写 {target}")
    else:
        sys.stdout.write(content)
    return 0


if __name__ == "__main__":
    sys.exit(main())
