# `models/` —— 模型包与 `package://` 约定（T02 钉死）

本目录是本项目**唯一**的模型根目录（默认模型搜索路径）。下游切片（S01 模型改造、
S02 场景构建、S07 golden-file 回归）必须通过 `rm75_drake.package_map` 解析
`package://` URL，**禁止**各自手写 `PackageMap.Add(...)` 或自行拼路径。

## 目录布局

```
models/
├── README.md              ← 本文件
└── rm75_drake/            ← PackageMap key = rm75_drake
    ├── package.xml        ← <name>rm75_drake</name>，注册用
    └── placeholder.urdf   ← T02 占位模型（官方 RM75-B 属 S01）
```

**硬约束**

* `models/` **根目录下不得放 `package.xml`**：`PackageMap.PopulateFromFolder` 会把
  每个含 `package.xml` 的目录注册成一个包，根目录放 package.xml 会凭空多出一个
  无意义包。
* 每个模型子目录必须自带 `package.xml`，其 `<name>` 即 PackageMap 的 key。

## key 命名规则

* 只允许**小写字母**与**下划线**（`^[a-z0-9_]+$`），不得出现大写、连字符或点。
* key 必须与对应的 Python 包名一致（本项目只有 `rm75_drake` 一个包）。
* 官方资产改造后仍归入本项目包（`rm75_drake`），**不**另建包名，以免 `package://`
  前缀分叉。

## `package://` 解析语义（实测结论，T02 钉死）

实测环境：受支持基线 Drake 1.55.0（py312）+ 降级档 Drake 1.51.1（py310），
两档结论一致。

| 注册方式 | key | value（`GetPath` 返回） |
|---|---|---|
| `PackageMap.PopulateFromFolder(dir)` | `package.xml` 里的 `<name>` | 该 package 所在**目录**（带尾斜杠） |
| `PackageMap.AddPackageXml(package.xml)` | 同上 | 同一目录（**不带**尾斜杠） |

* 两种注册方式下 `ResolveUrl("package://<pkg>/rel/path")` 的结果**完全一致**
  （同一个绝对文件路径），即**拼接语义本身没有歧义**；
* 差异只存在于 `GetPath` 原始 value 的尾斜杠，因此**不要**用 `GetPath` 的返回值
  自行做字符串拼接来解析 URL，一律走 `ResolveUrl`。
* 本项目统一**只走 `PopulateFromFolder`**（`rm75_drake.package_map.build_package_map()`），
  **禁止**在同一张 map 上混用两种注册方式。

### 另四个必须记住的 Drake 陷阱

1. `Parser.AddModels` 的 URL 形式是**关键字参数**：
   `parser.AddModels(url="package://rm75_drake/placeholder.urdf")`。
   写成位置参数会被当成普通文件名，相对 cwd 拼接后报 `XML_ERROR_FILE_NOT_FOUND`。
2. **不得**在解析前预预留 model instance 槽位（**反模式**，T02 返工钉死）。
   曾经的变通是 `plant.AddModelInstance(...)` 先塞若干空槽位，想规避
   `detail_collision_filter_group_resolver.cc:27` 的
   `'minimum_model_instance_index_ > world' failed` abort。
   **结论反转**：该 abort 正是这个变通自己引入的——
   `minimum_model_instance_index_` 取自 `plant->num_model_instances()`，
   预预留把 `num_model_instances()` 抬到大于 world 之后，Drake 反而**确定性** abort。
   唯一变量对照（同一台机器、同一份代码）：
   - 基线 Drake 1.55.0：预预留 **0/20** 成功；不预预留 **20/20** 成功；
   - 降级 Drake 1.51.1：预预留 **0/10** 成功；不预预留 **10/10** 成功。

   因此：model instance 的分配**交还 Drake Parser 管理**，解析前不要动 plant 的
   槽位；`rm75_drake.package_map.make_parser()` 内部**不做**任何预留
   （原 `drake_compat.prepare_plant_for_parsing()` 已删除）。
3. URDF 的关节 `parent link` 写成 `world` 并**不会**触发上面的 abort（已实测：
   挂到 world 的模型加载成功）。本项目仍约定 URDF **自包含**（关节 parent 用
   本 URDF 内的 link），而“挂到世界”的位姿用场景层的 `base_extrinsics` 表达
   （S02 填充）——这是**项目约定**，不是 Drake 的硬性限制。
   同理，`tests/test_package_map.py` 的模型加载用例不再用 `pytest.skip`
   掩盖失败：加载失败一律判红。
4. **Drake 的 `Parser` 不保活 plant**（C++ 侧只持有 `MultibodyPlant*` 裸指针）。
   若 plant 是在某个函数作用域里创建的（例如 `make_parser()` 的局部变量），
   函数返回后 plant 立刻被 Python GC 销毁，parser 就变成悬垂指针——后续
   `AddModels()` 直接 **SIGSEGV**（段错误，而不是可读的异常）。
   本项目的做法：`rm75_drake.package_map.make_parser()` 把 plant 登记到
   `plant_for_parser(parser)` 可查的弱引用保活表里（parser 活着 → plant 活着），
   调用方也可以用 `plant_for_parser(parser)` 显式持有 plant。
   **回归守卫**：`tests/test_package_map.py::test_make_parser_keeps_plant_alive`。

## 复现本文档结论的最小实验

```bash
python3 -m pytest tests/test_package_map.py -q
```
