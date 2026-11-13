"""静态审计器：把「厂商解耦」与「黑名单禁令」从文档变成**代码层强制**。

三类违规（全��从 ``rm_hw_bridge/contract.yaml`` 读取，不硬编码符号名）：

(a) **黑名单符号调用形态** —— ``rm_algo_*`` 族（按前缀）+ 明确列出的禁用符号。
    只认「调用形态」（``name(``），**不认字符串常量**：
    用 AST 精确判定 ``Call`` 节点，无法解析的语法错误文件退回正则扫描。
(b) **禁止的 import** —— ``Robotic_Arm`` / ``tesseract*`` / ``moveit*``。
    scope=repo 全仓禁止；scope=hardware **仅在硬件层包内**禁止
    （rmctl 侧允许 import ``tesseract_robotics``，那是规划内核，合规）；
    scope 也可写 ``contract.yaml`` 的 ``rules.layers`` 里声明的**层名**
    （如 ``executor``）——**按层判定**，不再硬编码硬件层根目录。
(c) **层反向依赖** —— 按 ``rules.<层>_must_not_import`` 判定：
    硬件层不得反向依赖体验层（rmctl）；**安全层**不得反向依赖体验层（rmctl）、
    执行层（rm_executor）与规划内核（tesseract*）；执行层不得反向依赖体验层
    （rmctl）与规划内核（tesseract*）。

    层的增删**只改 ``contract.yaml`` 的 ``rules.layers``**，本审计器零改动
    即可覆盖新层（``layer_roots`` / ``layer_of`` / ``_layer_reverse_targets``
    均为声明式读取）。

CLI::

    python3 -m rm_hw_bridge.audit --repo                          # 全仓扫描
    python3 -m rm_hw_bridge.audit --symbols --lib <path/to.so>   # 核对 22 核心符号
    python3 -m rm_hw_bridge.audit --paths <file.py> [...]        # 指定路径

退出码：``0`` 通过 / ``3`` 存在违规（与 rmctl 的分级约定一致）。
"""
from __future__ import annotations

import argparse
import ast
import fnmatch
import os
import re
import sys
from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

__all__ = [
    "HARDWARE_ROOT",
    "Violation",
    "audit_repo",
    "check_symbols",
    "layer_roots",
    "layer_of",
    "main",
    "scan_paths",
]

_PACKAGE_DIR = os.path.dirname(os.path.abspath(__file__))

#: 硬件层包根（``dualarm_ws/src/rm_hw_bridge``）—— 含包体与 tests/
HARDWARE_ROOT = os.path.abspath(os.path.join(_PACKAGE_DIR, ".."))

#: 工作区源码根（``dualarm_ws/src``）—— 各层包根的公共父目录
SRC_ROOT = os.path.abspath(os.path.join(_PACKAGE_DIR, "..", ".."))

#: 本文件自身（审计器里必然出现黑名单字面量）——扫描时排除
_SELF = os.path.abspath(__file__)

#: 扫描时跳过的目录名（不扫 .gsd/ / .deps/ / docs/ 等）
SKIP_DIR_NAMES = {
    ".git",
    ".gsd",
    ".deps",
    ".planning",
    ".audits",
    "docs",
    "build",
    "install",
    "log",
    "__pycache__",
    ".pytest_cache",
    ".mypy_cache",
    ".eggs",
    "node_modules",
}

#: 全仓扫描的源码根（相对仓库根）
REPO_SCAN_ROOTS = (os.path.join("dualarm_ws", "src"), "workcell")

#: 违规类型
KIND_BLACKLIST_CALL = "blacklist-call"
KIND_FORBIDDEN_IMPORT = "forbidden-import"
KIND_REVERSE_DEPENDENCY = "reverse-dependency"


@dataclass
class Violation:
    """一条审计违规。"""

    kind: str
    path: str
    line: int
    detail: str
    message: str = ""

    def __post_init__(self) -> None:
        if not self.message:
            self.message = self.detail

    def as_dict(self) -> Dict[str, Any]:
        return {
            "kind": self.kind,
            "path": self.path,
            "line": self.line,
            "detail": self.detail,
            "message": self.message,
        }

    def __str__(self) -> str:
        return "%s:%d: [%s] %s" % (self.path, self.line, self.kind, self.message)


# --------------------------------------------------------------------- 契约
def _load_contract(contract: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    if contract is not None:
        return contract
    from rm_hw_bridge import loader

    return loader.load_contract()


def _names(entries: Iterable[Any]) -> List[str]:
    out = []
    for item in entries or []:
        name = item.get("name") if isinstance(item, dict) else item
        if name:
            out.append(str(name))
    return out


def _blacklist(contract: Dict[str, Any]) -> Tuple[List[str], List[str]]:
    bl = contract.get("blacklist") or {}
    return _names(bl.get("symbols")), _names(bl.get("prefixes"))


def _forbidden_imports(contract: Dict[str, Any]) -> List[Dict[str, str]]:
    out = []
    for item in contract.get("forbidden_imports") or []:
        if isinstance(item, dict):
            name = item.get("name")
            if name:
                out.append(
                    {
                        "name": str(name),
                        "scope": str(item.get("scope") or "repo"),
                        "reason": str(item.get("reason") or ""),
                    }
                )
        elif item:
            out.append({"name": str(item), "scope": "repo", "reason": ""})
    return out


def _reverse_dep_targets(contract: Dict[str, Any],
                         roots: Optional[Dict[str, str]] = None) -> Dict[str, List[str]]:
    """层 → 该层禁止 import 的模块清单（**声明式**，按 ``rules.layers`` 泛化）。

    早期版本只读 ``rules.hardware_must_not_import``（单层硬编码）；现在改为
    按 ``rules.layers`` 声明的**每个层**读 ``rules.<层>_must_not_import``
    ——新增一层（如 ``safety``）只需改 ``contract.yaml``，审计器零改动。

    命中即报 ``reverse-dependency`` 违规（见 :func:`_scan_file` 的 (c) 段）。
    """
    if roots is None:
        roots = layer_roots(contract)
    return _layer_reverse_targets(contract, roots)


# ------------------------------------------------------------------ 分层
def layer_roots(contract: Optional[Dict[str, Any]] = None) -> Dict[str, str]:
    """层 → 层根目录（**声明式**，来自 ``contract.yaml`` 的 ``rules.layers``）。

    ``rules.layers`` 形如 ``hardware: rm_hw_bridge`` —— 层名 → 包根名；
    包根目录取「工作区源码根」下的同名目录（``dualarm_ws/src/<包根>``）。

    * 契约未声明分层时**退化为**只有硬件层（保持 S02 的行为不变）；
    * 返回的字典是**新拷贝**，调用方改动不影响契约。
    """
    contract = _load_contract(contract)
    rules = contract.get("rules") or {}
    layers = rules.get("layers") or {}
    roots: Dict[str, str] = {}
    for layer, pkg_root in (layers or {}).items():
        if not layer or not pkg_root:
            continue
        root = os.path.abspath(os.path.join(SRC_ROOT, str(pkg_root)))
        roots[str(layer)] = root
    # 硬件层是审计器自身的层：契约没声明时也保留（行为不变）
    roots.setdefault("hardware", HARDWARE_ROOT)
    return roots


def layer_of(path: str, roots: Optional[Dict[str, str]] = None) -> Optional[str]:
    """文件属于哪一层（``None`` = 不属于任何已声明层，如 workcell/）。

    命中多个层时取**根目录最长**的那个（更具体的层优先）。
    """
    if roots is None:
        roots = layer_roots()
    ap = os.path.abspath(path)
    best: Optional[str] = None
    best_len = -1
    for layer, root in (roots or {}).items():
        try:
            if os.path.commonpath([ap, os.path.abspath(root)]) == os.path.abspath(root):
                if len(root) > best_len:
                    best = layer
                    best_len = len(root)
        except ValueError:  # pragma: no cover - 跨盘（Windows）
            continue
    return best


def _layer_reverse_targets(contract: Dict[str, Any],
                           roots: Dict[str, str]) -> Dict[str, List[str]]:
    """层 → 该层禁止 import 的模块清单（``rules.<层>_must_not_import``）。"""
    rules = contract.get("rules") or {}
    out: Dict[str, List[str]] = {}
    for layer in roots or {}:
        key = "%s_must_not_import" % layer
        if key in rules:
            out[layer] = _names(rules.get(key))
    return out


def _matches_layer_target(module: str, target: str) -> bool:
    """模块是否命中该层的禁止清单。

    * 无通配符 → **顶层包名精确匹配**（``rmctl`` 命中 ``rmctl.hwcollect``，
      不命中 ``rmctlx``）；
    * 带通配符 → ``fnmatch``（``tesseract*`` 命中 ``tesseract_robotics``）。
    """
    if not module or not target:
        return False
    root = module.split(".")[0]
    if any(ch in target for ch in "*?["):
        return fnmatch.fnmatch(module, target) or fnmatch.fnmatch(root, target)
    return root == target or module == target


# ------------------------------------------------------------------ 工具函数
def _rel(path: str, base: Optional[str] = None) -> str:
    if base:
        try:
            return os.path.relpath(path, base)
        except ValueError:  # pragma: no cover - 跨盘（Windows）
            return path
    return path


def _is_hardware(path: str) -> bool:
    """文件是否位于硬件层包内（含 tests/）。"""
    try:
        return os.path.commonpath([os.path.abspath(path), HARDWARE_ROOT]) == HARDWARE_ROOT
    except ValueError:  # pragma: no cover
        return False


def _call_name(node: ast.Call) -> Optional[str]:
    """取调用目标的名字（``foo(...)`` → ``foo``；``a.foo(...)`` → ``foo``）。"""
    func = node.func
    if isinstance(func, ast.Name):
        return func.id
    if isinstance(func, ast.Attribute):
        return func.attr
    return None


def _imported_names(tree: ast.AST) -> List[Tuple[str, int]]:
    """收集源码里 import 的模块名（含行号）。"""
    out: List[Tuple[str, int]] = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                out.append((alias.name, getattr(node, "lineno", 0)))
        elif isinstance(node, ast.ImportFrom):
            if node.level and node.level > 0:
                # 相对 import：模块名就是包内名字
                out.append((node.module or "", getattr(node, "lineno", 0)))
            elif node.module:
                out.append((node.module, getattr(node, "lineno", 0)))
    return out


def _matches_forbidden(module: str, patterns: Sequence[str]) -> Optional[str]:
    for pat in patterns:
        if fnmatch.fnmatch(module, pat):
            return pat
    return None


def _scope_applies(scope: str, layer: Optional[str], path: str,
                   roots: Optional[Dict[str, str]] = None) -> bool:
    """``forbidden_imports.scope`` 是否适用于该文件（**按层判定**）。

    * ``repo`` —— 全仓适用（调用方直接跳过本函数）；
    * ``<层名>`` —— 仅该层内的文件适用（如 ``hardware`` / ``executor``）；
    * 契约未声明的层名 → 保守按**全仓**适用（不静默放过）。
    """
    if scope == "repo":
        return True
    if roots is None:
        roots = layer_roots()
    if scope in (roots or {}):
        return layer == scope
    if scope == "hardware":
        # 兼容：契约未声明 layers 时仍按硬件层包根判定（行为不变）
        return _is_hardware(path)
    return True


def _matches_blacklist(name: str, symbols: Sequence[str], prefixes: Sequence[str]) -> Optional[str]:
    if name in symbols:
        return name
    for pref in prefixes:
        if name.startswith(pref):
            return pref + "*"
    return None


# ------------------------------------------------------------------ 扫描主体
def _scan_file(
    path: str,
    contract: Dict[str, Any],
    base: Optional[str] = None,
) -> List[Violation]:
    rel = _rel(path, base)
    symbols, prefixes = _blacklist(contract)
    forbidden = _forbidden_imports(contract)
    roots = layer_roots(contract)
    layer = layer_of(path, roots)
    layer_targets = _layer_reverse_targets(contract, roots)
    violations: List[Violation] = []

    try:
        with open(path, "r", encoding="utf-8") as fh:
            src = fh.read()
    except (OSError, UnicodeDecodeError):
        return violations

    try:
        tree = ast.parse(src, filename=path)
    except SyntaxError:
        # 语法错误文件：退回正则（调用形态），并报告解析失败
        violations.append(
            Violation(
                kind="parse-error",
                path=rel,
                line=0,
                detail="无法解析源码，已退回正则扫描",
                message="无法解析源码（SyntaxError），已退回正则扫描: %s" % rel,
            )
        )
        for name, pattern in list(symbols) + [(p + "*", None) for p in prefixes]:
            if pattern is None:
                regex = re.compile(r"\b%s\w*\s*\(" % re.escape(name))
            else:
                regex = re.compile(r"\b%s\w*\s*\(" % re.escape(name))
            for m in regex.finditer(src):
                line = src.count("\n", 0, m.start()) + 1
                violations.append(
                    Violation(
                        kind=KIND_BLACKLIST_CALL,
                        path=rel,
                        line=line,
                        detail=name,
                        message="调用黑名单符号 %s（禁止）" % name,
                    )
                )
        return violations

    # (a) 黑名单调用形态
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        name = _call_name(node)
        if not name:
            continue
        hit = _matches_blacklist(name, symbols, prefixes)
        if hit:
            violations.append(
                Violation(
                    kind=KIND_BLACKLIST_CALL,
                    path=rel,
                    line=getattr(node, "lineno", 0),
                    detail=name,
                    message="调用黑名单符号 %s（禁止）" % name,
                )
            )

    # (b) 禁止的 import（scope 按**层**判定：repo 全仓 / <层名> 仅该层）
    for module, line in _imported_names(tree):
        if not module:
            continue
        for entry in forbidden:
            scope = entry["scope"]
            if scope != "repo" and not _scope_applies(scope, layer, path, roots):
                continue
            if _matches_forbidden(module, [entry["name"]]):
                violations.append(
                    Violation(
                        kind=KIND_FORBIDDEN_IMPORT,
                        path=rel,
                        line=line,
                        detail=module,
                        message="禁止 import %s（%s）"
                        % (module, entry.get("reason") or entry["name"]),
                    )
                )
                break

    # (c) 层反向依赖（rules.<层>_must_not_import）
    if layer and layer in layer_targets:
        for module, line in _imported_names(tree):
            if not module:
                continue
            for target in layer_targets[layer]:
                if _matches_layer_target(module, target):
                    violations.append(
                        Violation(
                            kind=KIND_REVERSE_DEPENDENCY,
                            path=rel,
                            line=line,
                            detail=module,
                            message="%s 层不得 import %s（层反向依赖禁令 %s）"
                            % (layer, module, target),
                        )
                    )
                    break
    return violations


def scan_paths(
    paths: Iterable[str],
    contract: Optional[Dict[str, Any]] = None,
    base: Optional[str] = None,
) -> List[Violation]:
    """静态扫描给定的 ``.py`` 路径（文件或目录），返回违规列表。

    * 目录会递归展开，``SKIP_DIR_NAMES`` 里的目录跳过；
    * ``rm_hw_bridge/audit.py`` 自身跳过（审计器必然含黑名单字面量）。
    """
    contract = _load_contract(contract)
    violations: List[Violation] = []
    seen = set()

    def _handle(file_path: str) -> None:
        ap = os.path.abspath(file_path)
        if ap in seen:
            return
        seen.add(ap)
        if ap == _SELF:
            return
        if not ap.endswith(".py"):
            return
        violations.extend(_scan_file(ap, contract, base=base))

    for raw in paths or []:
        p = os.path.abspath(str(raw))
        if os.path.isdir(p):
            for dirpath, dirnames, filenames in os.walk(p):
                dirnames[:] = sorted(d for d in dirnames if d not in SKIP_DIR_NAMES)
                for fn in sorted(filenames):
                    _handle(os.path.join(dirpath, fn))
        elif os.path.isfile(p):
            _handle(p)
    return violations


def repo_scan_paths(root: Optional[str] = None) -> List[str]:
    """全仓扫描路径（``dualarm_ws/src`` + ``workcell``）。"""
    root = os.path.abspath(root or _repo_root())
    out = []
    for rel in REPO_SCAN_ROOTS:
        p = os.path.join(root, rel)
        if os.path.isdir(p):
            out.append(p)
    return out


def _repo_root() -> str:
    from rm_hw_bridge import loader

    return loader.repo_root()


def audit_repo(
    root: Optional[str] = None,
    contract: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    """扫描全仓，返回结构化报告。"""
    contract = _load_contract(contract)
    root = os.path.abspath(root or _repo_root())
    paths = repo_scan_paths(root)
    violations = scan_paths(paths, contract=contract, base=root)
    scanned = 0
    for p in paths:
        for dirpath, dirnames, filenames in os.walk(p):
            dirnames[:] = [d for d in dirnames if d not in SKIP_DIR_NAMES]
            scanned += sum(1 for f in filenames if f.endswith(".py"))
    return {
        "root": root,
        "paths": paths,
        "files_scanned": scanned,
        "violations": [v.as_dict() for v in violations],
        "violation_objects": violations,
        "ok": not violations,
    }


def check_symbols(
    lib_path: str,
    contract: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    """核对清单符号在动态库中是否存在且可解析。"""
    import ctypes

    from rm_hw_bridge import loader

    contract = _load_contract(contract)
    core = loader.core_symbols(contract)
    support = loader.support_symbols(contract)
    try:
        lib = ctypes.CDLL(lib_path)
    except OSError as exc:
        return {
            "lib": lib_path,
            "ok": False,
            "error": str(exc),
            "missing_core": list(core),
            "missing_support": list(support),
            "found_core": [],
            "found_support": [],
        }
    missing_core = [n for n in core if not hasattr(lib, n)]
    missing_support = [n for n in support if not hasattr(lib, n)]
    return {
        "lib": lib_path,
        "ok": not missing_core and not missing_support,
        "error": None,
        "missing_core": missing_core,
        "missing_support": missing_support,
        "found_core": [n for n in core if hasattr(lib, n)],
        "found_support": [n for n in support if hasattr(lib, n)],
    }


# --------------------------------------------------------------------- CLI
def _print_violations(violations: Sequence[Violation]) -> None:
    for v in violations:
        print(str(v), file=sys.stderr)


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        prog="rm_hw_bridge.audit",
        description="rm_hw_bridge 静态审计：黑名单调用 / 禁止 import / 反向依赖",
    )
    parser.add_argument("--repo", action="store_true", help="扫描全仓（dualarm_ws/src + workcell）")
    parser.add_argument(
        "--paths", nargs="*", default=None, help="扫描指定文件/目录（可多个）"
    )
    parser.add_argument(
        "--symbols", action="store_true", help="核对清单符号在动态库中是否存在"
    )
    parser.add_argument("--lib", default=None, help="动态库路径（配合 --symbols）")
    parser.add_argument("--json", action="store_true", help="以 JSON 输出报告")
    args = parser.parse_args(argv)

    contract = _load_contract()
    violations: List[Violation] = []
    reports: Dict[str, Any] = {}

    do_scan = args.repo or bool(args.paths)
    if do_scan:
        paths = []
        if args.repo:
            paths.extend(repo_scan_paths())
        if args.paths:
            paths.extend(args.paths)
        violations.extend(scan_paths(paths, contract=contract, base=_repo_root()))
        reports["scan"] = {
            "paths": paths,
            "violations": [v.as_dict() for v in violations],
        }

    if args.symbols:
        lib_path = args.lib
        if not lib_path:
            from rm_hw_bridge import loader

            lib_path = loader.find_library()
        if not lib_path:
            print("找不到 RM-API2 动态库（用 --lib 指定路径，或设置 RM_API2_LIB）", file=sys.stderr)
            return 3
        sym_report = check_symbols(lib_path, contract=contract)
        reports["symbols"] = sym_report
        if not sym_report["ok"]:
            for name in sym_report["missing_core"]:
                violations.append(
                    Violation(
                        kind="missing-symbol",
                        path=lib_path,
                        line=0,
                        detail=name,
                        message="动态库缺少核心符号 %s" % name,
                    )
                )
            for name in sym_report["missing_support"]:
                violations.append(
                    Violation(
                        kind="missing-symbol",
                        path=lib_path,
                        line=0,
                        detail=name,
                        message="动态库缺少支持符号 %s" % name,
                    )
                )

    if not do_scan and not args.symbols:
        parser.print_help()
        return 0

    if args.json:
        print(
            __import__("json").dumps(
                {
                    "ok": not violations,
                    "violations": [v.as_dict() for v in violations],
                    "reports": reports,
                },
                ensure_ascii=False,
                indent=2,
            )
        )
    else:
        if violations:
            _print_violations(violations)
            print("审计失败：%d 项违规" % len(violations), file=sys.stderr)
        else:
            print("审计通过：0 项违规")

    return 3 if violations else 0


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
