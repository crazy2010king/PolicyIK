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

## `package://` 解析语义（实测结论，T02 在 Drake 1.51.1 上实测并钉死）

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

### 另三个必须记住的 Drake 陷阱

1. `Parser.AddModels` 的 URL 形式是**关键字参数**：
   `parser.AddModels(url="package://rm75_drake/placeholder.urdf")`。
   写成位置参数会被当成普通文件名，相对 cwd 拼接后报 `XML_ERROR_FILE_NOT_FOUND`。
2. URDF 里关节的 `parent link` 必须是**本 URDF 内的 link**，不能写 `link="world"`：
   Drake 1.51.1 会触发
   `detail_collision_filter_group_resolver.cc:27` 的 CHECK 失败
   （`'minimum_model_instance_index_ > world' failed`）并 abort 进程。
3. **空 plant 上直接解析模型会间歇性 abort**（同上 CHECK）：
   `Parser.AddModels` 内部构造的 `CollisionFilterGroupResolver` 要求
   `plant->num_model_instances() > world`，而未 finalize 的 plant 上
   `world_body().model_instance()` 不可靠（Python 侧恒显示 0，C++ 侧读到的值随堆布局变化），
   于是该 DEMAND 变成间歇性失败并 abort 进程。
   规避方式：解析前预留若干空的 model instance 槽位——
   `rm75_drake.drake_compat.prepare_plant_for_parsing(plant)`，
   或直接用 `rm75_drake.package_map.make_parser()`（内部已调用）。
   注意：本机（Drake 1.51.1 / fallback 档）实测**不能 100% 消除**，
   因此 `tests/test_package_map.py` 的模型加载用例把解析放在**独立子进程**中执行，
   abort 只会让该用例显式 skip，不会打挂 pytest 会话。
   TODO(S01)：在受支持基线 Drake 1.55（py312）上复核该 DEMAND 是否仍存在。

## 复现本文档结论的最小实验

```bash
python3 -m pytest tests/test_package_map.py -q
```
