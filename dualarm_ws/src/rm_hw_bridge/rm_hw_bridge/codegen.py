"""RM-API2 ctypes 绑定生成器（codegen）。

从 ``abi_spec.yaml``（唯一真值 / single source of truth）生成：

* ``rm_api2_ctypes.py`` —— ctypes ``Structure`` 定义、回调类型、符号表
  （每个符号的 ``argtypes`` / ``restype``）以及 ``bind_library()`` 绑定函数。
* ``layout_golden.json`` —— 每个结构体的 ``size`` 与每个字段的 ``offset``。

用法::

    python3 -m rm_hw_bridge.codegen --spec <abi_spec.yaml> --out <generated 目录> [--check]

``--check``：把生成物重新生成到临时目录并与磁盘内容**逐字节**比对；一致退出 0，
不一致则打印差异并以非 0 退出（生成必须幂等）。

本模块只依赖标准库 + PyYAML；**禁止** import 厂商封装（tesseract / Robotic_Arm）。
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
import sys
import tempfile
from typing import Any, Dict, List, Tuple

import yaml

__all__ = [
    "load_spec",
    "sha256_file",
    "render_ctypes",
    "render_golden",
    "generate",
    "main",
]

# ---------------------------------------------------------------------------
# 类型系统
# ---------------------------------------------------------------------------

#: 基元类型 → ctypes 类型表达式
PRIMITIVES: Dict[str, str] = {
    "int": "ctypes.c_int",
    "int8": "ctypes.c_int8",
    "int16": "ctypes.c_int16",
    "int32": "ctypes.c_int32",
    "int64": "ctypes.c_int64",
    "uint8": "ctypes.c_uint8",
    "uint16": "ctypes.c_uint16",
    "uint32": "ctypes.c_uint32",
    "uint64": "ctypes.c_uint64",
    "float": "ctypes.c_float",
    "double": "ctypes.c_double",
    "bool": "ctypes.c_bool",
    "char": "ctypes.c_char",
    "void": "None",
}

#: 枚举在 C 里是 int 尺寸
ENUM_EXPR = "ctypes.c_int"

ABI_RULES = (
    "rm_create_robot_arm 失败时仍返回非空指针但 handle.id == -1；必须读 id 判失败。",
    "rm_set_realtime_push / rm_movej_canfd 的结构体按值传递（非指针）。",
    "rm_movej_canfd_mode_t.joint 是 float*；调用方必须让缓冲存活到调用结束。",
    "rm_realtime_arm_state_call_back 是进程级全局注册；同进程无法同时收两路臂。",
    "rm_realtime_push_config_t.ip 是 char[28]（ctypes.c_char * 28），赋值必须用 bytes。",
    "单位：关节角 °；rm_set_joint_drive_max_speed °/s；_max_acc °/s²；_min_pos/_max_pos °。",
)


class SpecError(ValueError):
    """ABI 规格自身不合法（类型未知 / 循环依赖 / 字段缺失）。"""


# ---------------------------------------------------------------------------
# 规格读取
# ---------------------------------------------------------------------------


def load_spec(spec_path: str) -> Dict[str, Any]:
    """读取并做最小校验的 ABI 规格。"""
    with open(spec_path, "r", encoding="utf-8") as fh:
        spec = yaml.safe_load(fh)
    if not isinstance(spec, dict):
        raise SpecError("ABI 规格必须是 YAML 映射：%s" % spec_path)
    for key in ("structs", "functions"):
        if key not in spec:
            raise SpecError("ABI 规格缺少顶层键 %r" % key)
    return spec


def sha256_file(path: str) -> str:
    """文件 sha256（十六进制），用于把生成物钉在唯一真值上。"""
    digest = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(65536), b""):
            digest.update(chunk)
    return digest.hexdigest()


# ---------------------------------------------------------------------------
# 类型解析
# ---------------------------------------------------------------------------


def _known_types(spec: Dict[str, Any]) -> Dict[str, str]:
    known = dict(PRIMITIVES)
    for name in spec.get("structs", {}):
        known[name] = "struct"
    for name in spec.get("enums", {}):
        known[name] = "enum"
    for name in spec.get("callback_types", {}):
        known[name] = "callback"
    return known


def resolve_type(type_str: str, spec: Dict[str, Any]) -> str:
    """把规格里的类型串翻译成 ctypes 类型表达式。

    支持：基元、结构体名、枚举名、回调类型名、指针（``float*`` / ``const char*``）。
    """
    if not isinstance(type_str, str) or not type_str.strip():
        raise SpecError("非法类型：%r" % (type_str,))
    raw = type_str.strip()
    known = _known_types(spec)

    if raw.endswith("*"):
        base = raw[:-1].strip()
        const = False
        if base.startswith("const "):
            const = True
            base = base[len("const "):].strip()
        if base == "void":
            return "ctypes.c_void_p"
        if base == "char":
            return "ctypes.c_char_p" if const else "ctypes.POINTER(ctypes.c_char)"
        if base in PRIMITIVES:
            return "ctypes.POINTER(%s)" % PRIMITIVES[base]
        if base in known and known[base] == "enum":
            return "ctypes.POINTER(%s)" % ENUM_EXPR
        if base in known and known[base] == "struct":
            return "ctypes.POINTER(%s)" % base
        if base in known and known[base] == "callback":
            return base
        raise SpecError("未知指针基类型 %r（来自 %r）" % (base, type_str))

    if raw in PRIMITIVES:
        return PRIMITIVES[raw]
    if raw in known and known[raw] == "enum":
        return ENUM_EXPR
    if raw in known and known[raw] == "struct":
        return raw
    if raw in known and known[raw] == "callback":
        return raw
    raise SpecError("未知类型 %r" % raw)


def _field_dependencies(spec: Dict[str, Any]) -> Dict[str, List[str]]:
    """结构体 → 其直接依赖的结构体名。"""
    deps: Dict[str, List[str]] = {}
    struct_names = set(spec["structs"])
    for sname, sdef in spec["structs"].items():
        need: List[str] = []
        for field in sdef.get("fields", []):
            ftype = str(field["type"]).strip()
            if ftype.endswith("*"):
                ftype = ftype[:-1].strip()
                if ftype.startswith("const "):
                    ftype = ftype[len("const "):].strip()
            if ftype in struct_names and ftype not in need:
                need.append(ftype)
        deps[sname] = need
    return deps


def struct_order(spec: Dict[str, Any]) -> List[str]:
    """按依赖拓扑排序结构体（被依赖者在前）。"""
    deps = _field_dependencies(spec)
    ordered: List[str] = []
    visiting: List[str] = []

    def visit(name: str) -> None:
        if name in ordered:
            return
        if name in visiting:
            raise SpecError("结构体循环依赖：%s" % " -> ".join(visiting + [name]))
        visiting.append(name)
        for dep in deps[name]:
            visit(dep)
        visiting.pop()
        ordered.append(name)

    for name in spec["structs"]:
        visit(name)
    return ordered


# ---------------------------------------------------------------------------
# 渲染
# ---------------------------------------------------------------------------


def _doc_lines(doc: Any, indent: str = "    ") -> List[str]:
    if not doc:
        return []
    text = str(doc).replace("\n", " ").strip()
    return ["%s\"\"\"%s" % (indent, text), "%s\"\"\"" % indent]


def render_ctypes(spec: Dict[str, Any], spec_sha: str) -> str:
    """生成 ctypes 绑定模块文本（确定性输出，不含时间戳）。"""
    sdk = spec.get("sdk", {}) or {}
    out: List[str] = []
    out.append('"""RM-API2 ctypes 绑定（**自动生成，请勿手改**）。')
    out.append("")
    out.append("源头: rm_hw_bridge/abi_spec.yaml")
    out.append("abi_spec.yaml sha256: %s" % spec_sha)
    out.append("SDK: %s %s" % (sdk.get("name", "RM-API2"), sdk.get("version", "?")))
    out.append("")
    out.append("重新生成: python3 -m rm_hw_bridge.codegen --spec <abi_spec.yaml> --out <dir>")
    out.append("幂等校验: python3 -m rm_hw_bridge.codegen --spec <abi_spec.yaml> --out <dir> --check")
    out.append("")
    out.append("ABI 关键规则:")
    for rule in ABI_RULES:
        out.append("* %s" % rule)
    out.append('"""')
    out.append("from __future__ import annotations")
    out.append("")
    out.append("import ctypes")
    out.append("")
    out.append('SPEC_SHA256 = "%s"' % spec_sha)
    out.append('SDK_NAME = "%s"' % sdk.get("name", "RM-API2"))
    out.append('SDK_VERSION = "%s"' % sdk.get("version", ""))
    out.append("")

    # 常量
    constants = spec.get("constants", {}) or {}
    if constants:
        out.append("# ---------------------------------------------------------------- 常量")
        for name in sorted(constants):
            cdef = constants[name] or {}
            doc = cdef.get("doc")
            if doc:
                out.append("# %s" % doc)
            out.append("%s = %r" % (name, cdef.get("value")))
        out.append("")

    # 枚举
    enums = spec.get("enums", {}) or {}
    if enums:
        out.append("# ---------------------------------------------------------------- 枚举")
        for ename in sorted(enums):
            edef = enums[ename] or {}
            if edef.get("doc"):
                out.append("# %s" % edef["doc"])
            for key in sorted(edef.get("values", {})):
                out.append("%s = %d" % (key, edef["values"][key]))
            out.append("")
        out.append("# 枚举在 C ABI 中是 int 尺寸（不改变内存布局）")
        out.append("")

    # 结构体
    out.append("# ---------------------------------------------------------------- 结构体")
    for sname in struct_order(spec):
        sdef = spec["structs"][sname] or {}
        out.append("")
        out.append("class %s(ctypes.Structure):" % sname)
        doc = sdef.get("doc")
        if doc:
            out.append('    """%s"""' % str(doc).replace("\n", " "))
        out.append("")
        fields = sdef.get("fields", []) or []
        if not fields:
            raise SpecError("结构体 %s 没有任何字段" % sname)
        out.append("    _fields_ = [")
        for field in fields:
            fname = field["name"]
            ftype = str(field["type"]).strip()
            count = field.get("count")
            expr = resolve_type(ftype, spec)
            if count:
                expr = "(%s * %d)" % (expr, int(count))
            comment = field.get("doc")
            if comment:
                out.append('        ("%s", %s),  # %s' % (fname, expr, comment))
            else:
                out.append('        ("%s", %s),' % (fname, expr))
        out.append("    ]")
    out.append("")

    # 回调类型
    callbacks = spec.get("callback_types", {}) or {}
    if callbacks:
        out.append("# ---------------------------------------------------------------- 回调类型")
        for cname in sorted(callbacks):
            cdef = callbacks[cname] or {}
            if cdef.get("doc"):
                out.append("# %s" % cdef["doc"])
            ret = resolve_type(str(cdef.get("ret", "void")), spec)
            args = []
            for arg in cdef.get("args", []) or []:
                args.append(resolve_type(str(arg["type"]), spec))
            out.append("%s = ctypes.CFUNCTYPE(%s%s)" % (
                cname, ret, (", " + ", ".join(args)) if args else ""))
        out.append("")

    # 符号表
    out.append("# ---------------------------------------------------------------- 符号表")
    out.append("#: 符号名 → {\"argtypes\": [...], \"restype\": ...}（可直接喂给 ctypes）")
    out.append("SYMBOLS = {")
    for fname in sorted(spec["functions"]):
        fdef = spec["functions"][fname] or {}
        args = [resolve_type(str(a["type"]), spec) for a in (fdef.get("args") or [])]
        ret = resolve_type(str(fdef.get("ret", "int")), spec)
        out.append('    "%s": {' % fname)
        out.append('        "argtypes": [%s],' % ", ".join(args))
        out.append('        "restype": %s,' % ret)
        by_value = [
            a["name"] for a in (fdef.get("args") or [])
            if a.get("by_value")
        ]
        if by_value:
            out.append('        "by_value": %r,' % (by_value,))
        out.append('    },')
    out.append("}")
    out.append("")
    out.append("#: 全部符号名（白名单基线；装载层另加黑名单静态审计）")
    out.append("SYMBOL_NAMES = tuple(sorted(SYMBOLS))")
    out.append("")
    out.append("")
    out.append("def bind_library(lib, strict: bool = True):")
    out.append('    """把 argtypes / restype 绑定到已加载的 CDLL 上。')
    out.append("")
    out.append("    strict=True 时，库中缺少任何规格符号都会抛 RuntimeError（避免静默半可用）。")
    out.append('    """')
    out.append("    missing = []")
    out.append("    for name, sig in SYMBOLS.items():")
    out.append("        fn = getattr(lib, name, None)")
    out.append("        if fn is None:")
    out.append("            missing.append(name)")
    out.append("            continue")
    out.append("        fn.argtypes = list(sig[\"argtypes\"])")
    out.append("        fn.restype = sig[\"restype\"]")
    out.append("    if missing and strict:")
    out.append("        raise RuntimeError(")
    out.append("            \"RM-API2 库缺少规格符号: %s\" % \", \".join(sorted(missing)))")
    out.append("    if missing:")
    out.append("        return lib, missing")
    out.append("    return lib")
    out.append("")
    return "\n".join(out)


def render_golden(spec: Dict[str, Any], spec_sha: str, layout: Dict[str, Any]) -> str:
    """生成布局黄金表文本（确定性 JSON）。"""
    payload = {
        "sdk": {
            "name": (spec.get("sdk", {}) or {}).get("name", "RM-API2"),
            "version": (spec.get("sdk", {}) or {}).get("version", ""),
        },
        "abi_spec_sha256": spec_sha,
        "structs": layout,
    }
    return json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n"


# ---------------------------------------------------------------------------
# 布局测量
# ---------------------------------------------------------------------------


def measure_layout(module: Any, spec: Dict[str, Any]) -> Dict[str, Any]:
    """用运行期 ctypes 测量每个结构体的 size 与字段 offset。"""
    import ctypes  # 局部导入：仅测量阶段需要

    layout: Dict[str, Any] = {}
    for sname in struct_order(spec):
        cls = getattr(module, sname)
        fields: Dict[str, int] = {}
        for field in (spec["structs"][sname].get("fields") or []):
            fname = field["name"]
            descriptor = getattr(cls, fname)
            offset = getattr(descriptor, "offset", None)
            if offset is None:
                raise SpecError("无法取得字段偏移：%s.%s" % (sname, fname))
            fields[fname] = int(offset)
        layout[sname] = {
            "size": int(ctypes.sizeof(cls)),
            "fields": fields,
        }
    return layout


def _import_generated(path: str, module_name: str) -> Any:
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:  # pragma: no cover - 防御
        raise RuntimeError("无法加载生成模块：%s" % path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def generate(spec_path: str, out_dir: str) -> Dict[str, str]:
    """生成两个产物，返回 ``{文件名: 内容}``。"""
    spec = load_spec(spec_path)
    spec_sha = sha256_file(spec_path)
    source = render_ctypes(spec, spec_sha)

    os.makedirs(out_dir, exist_ok=True)
    tmp_dir = tempfile.mkdtemp(prefix="rm_hw_bridge_codegen_")
    try:
        tmp_py = os.path.join(tmp_dir, "rm_api2_ctypes.py")
        with open(tmp_py, "w", encoding="utf-8") as fh:
            fh.write(source)
        module = _import_generated(tmp_py, "_rm_hw_bridge_generated_probe")
        layout = measure_layout(module, spec)
    finally:
        try:
            os.remove(os.path.join(tmp_dir, "rm_api2_ctypes.py"))
            os.rmdir(tmp_dir)
        except OSError:  # pragma: no cover - 清理失败不影响结果
            pass

    golden = render_golden(spec, spec_sha, layout)
    return {
        "rm_api2_ctypes.py": source,
        "layout_golden.json": golden,
    }


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def _default_spec_path() -> str:
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), "abi_spec.yaml")


def _default_out_dir() -> str:
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), "generated")


def main(argv: List[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="python3 -m rm_hw_bridge.codegen",
        description="从 abi_spec.yaml 生成 RM-API2 ctypes 绑定与布局黄金表",
    )
    parser.add_argument("--spec", default=_default_spec_path(), help="ABI 规格 YAML 路径")
    parser.add_argument("--out", default=_default_out_dir(), help="生成物输出目录")
    parser.add_argument(
        "--check",
        action="store_true",
        help="只校验：重新生成并与磁盘内容逐字节比对（幂等校验）",
    )
    args = parser.parse_args(argv)

    try:
        artifacts = generate(args.spec, args.out)
    except SpecError as exc:
        print("[codegen] ABI 规格错误: %s" % exc, file=sys.stderr)
        return 2

    if not args.check:
        os.makedirs(args.out, exist_ok=True)
        for name, content in artifacts.items():
            path = os.path.join(args.out, name)
            with open(path, "w", encoding="utf-8") as fh:
                fh.write(content)
            print("[codegen] 写入 %s" % path)
        return 0

    drift = []
    for name, content in sorted(artifacts.items()):
        path = os.path.join(args.out, name)
        if not os.path.exists(path):
            drift.append("%s: 缺失（应重新生成）" % name)
            continue
        with open(path, "r", encoding="utf-8") as fh:
            on_disk = fh.read()
        if on_disk != content:
            drift.append("%s: 与重新生成结果不一致" % name)
    if drift:
        print("[codegen] 生成物漂移:", file=sys.stderr)
        for item in drift:
            print("  - %s" % item, file=sys.stderr)
        return 1
    print("[codegen] 生成物与 abi_spec.yaml 一致（幂等）")
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
