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

---

## S01 改造版模型：`rm75_drake/rm75_drake.urdf`（S01-T02 落地）

上游基线：官方 RM75-B URDF
（`https://github.com/RealManRobot/rm_models`，commit `cff49bb40f5ed0728505c03bff0e521a0dc21acd`，
路径 `RM75/urdf/RM75-B/urdf/RM75-B.urdf`；归档 sha256 见 `third_party/MANIFEST.yaml`）。

### 官方 → 改造差异清单

| # | 项 | 官方 | 改造版 | 理由 |
|---|----|------|--------|------|
| 1 | 速度限位 | 7 轴一律 `3.14` rad/s | J1/J2 `3.1416`，J3–J7 `3.927` rad/s | 官方参数表（J1/J2 180°/s，J3–J7 225°/s）。**本切片唯一的数值修正项** |
| 2 | joint_7 位置限位 | `±6.28` | `±6.283185307179586`（±360°） | 官方参数表；官方 URDF 是十进制截断 |
| 3 | collision 几何 | `.STL` 网格 | `<box>` 原始几何体（8 个 link 全部） | Drake 1.51/1.55 不解析 `.stl`（只吃 .obj/.vtk/.gltf），且凸包化会扭曲形状 |
| 4 | collision 尺寸/原点 | 网格文件 | 官方 STL 实测轴对齐包围盒（容差 1e-4 m） | 见下方包围盒表 |
| 5 | `<visual>` | 8 个网格引用 | **全部删除** | 官方网格许可未明示、不可再分发；入 git 的模型必须零外部资产依赖。视觉版由 `python3 scripts/convert_meshes.py` 在本机生成到 `models/rm75_drake/meshes/`（`.gitignore` 已排除） |
| 6 | collision 命名 | 无 | 每个 collision 加 `name="<link>_collision"` | Drake 据此生成唯一几何名 `RM75-B::link_N_collision`，供自碰撞断言映射 |
| 7 | 基座固定 | 无（DoF=14） | 装配层 `plant.WeldFrames(world, base_link)`（`Finalize()` 之前） | URDF 里写 `parent link="world"` 被项目禁止（见上文陷阱 2/3） |
| 8 | 自碰撞过滤声明 | 无 | `<drake:disable_collision>` + 7 组相邻 link 对 | 见下方「碰撞过滤策略」 |
| 9 | 惯量 / 质量 / COM / joint rpy / axis | 官方值 | **逐字沿用** | 官方 URDF 是唯一真值来源；总质量 7.776 kg |

### collision box 尺寸与原点（官方 STL 实测，`tools/mesh_tools.py`）

| link | size (m) | origin (m) |
|------|----------|------------|
| base_link | 0.1143 × 0.1100 × 0.1385 | -0.00215, 0.0, 0.06925 |
| link_1 | 0.0820 × 0.1090 × 0.1425 | 0.0, -0.0135, -0.03075 |
| link_2 | 0.0810 × 0.1930 × 0.0820 | 0.0, -0.0560, 0.0060 |
| link_3 | 0.0700 × 0.1020 × 0.1375 | 0.0, -0.0160, -0.03475 |
| link_4 | 0.0680 × 0.1625 × 0.0740 | 0.0, -0.04725, 0.0080 |
| link_5 | 0.0580 × 0.0830 × 0.1100 | 0.0, -0.0125, -0.0265 |
| link_6 | 0.0572 × 0.1435 × 0.0685 | 0.0, -0.04325, 0.00575 |
| link_7 | 0.0680 × 0.0733 × 0.0290 | 0.0050, 0.0, -0.0145 |

* 测量方法：`python3 scripts/convert_meshes.py --print-boxes`（binary STL 解析 +
  轴对齐包围盒）；复测容差 1e-4 m。
* 注：`link_7` 的网格 z 上界为 0（不是 z 厚度为 0）——实测 z 向**厚度为 0.029 m**，
  因此不需要人为加厚；若后续实测出现真退化尺寸，至少取 0.005 m。

### 碰撞过滤策略（重要，勿删）

**Drake 1.51.1 与 1.55.0 的 URDF 解析器都不实现 `drake:disable_collision` /
`drake:disable_collide` 标签**（已逐字节扫描两版 `libdrake.so`：字符串
`disable_collision` / `default_collide` 出现 **0** 次；两版都只实现了
`drake:collision_filter_group`）。因此：

* URDF 里的 `<drake:disable_collision>` 块是**声明性意图**，不是生效机制；
* 真正生效的是 **Drake 的默认父子相邻体过滤**：由 joint 直接相连的两个 body
  默认被过滤——本模型的 7 组相邻 link 对（`base_link–link_1` … `link_6–link_7`）
  由此被过滤；非相邻对（`base_link–link_3`、`link_1–link_3` …）默认不过滤；
* 该行为已被 `tests/test_model_load.py::test_adjacent_pairs_are_collision_filtered`
  与 `::test_non_adjacent_pairs_are_not_filtered` 锁死（S01-T05 会再补回归套件）。

**为什么写 `default_collide="true"` 而不是 `"false"`**：若将来某版本 Drake 真的实现
该标签，`"false"` 意味着"除这 7 组外全部过滤"，等于**静默关掉自碰撞检测**——
那是安全相关的失效模式。`"true"`（默认碰撞 + 显式禁用这 7 组）与当前实际行为
一致，且对未来实现是安全的。

### 零位高度锚点

零位末端高度 `0.8505 m` = `d1 + d3 + d5 + d7` = 240.5 + 256 + 210 + 144 mm
（官方 MDH）。实测 Drake FK：`[0.0, 2.24e-06, 0.8505]`——y 的 2.24e-6 偏差来自
官方 URDF 的 `rpy = 1.5708` 舍入（不是模型错误），断言用 `|y| < 1e-5`。

### 复现本节结论

```bash
python3 -m pytest tests/test_model_load.py -q
```

---

## S01 自研夹爪模型：`rm75_drake/rmg24_drake.urdf`（S01-T04 落地）

官方仓 `https://github.com/RealManRobot/Two-Finger-Gripper`（commit
`91f587b1d4efaab235f3d227ee614b28f35060c9`，许可 OpenAtom Open Hardware 1.0，
见 `third_party/MANIFEST.yaml`）**没有 URDF、也没有网格**：只有固件、PCB、
`Hardware/CAD/RMG24 Claw 3D Model.stp`（11.9 MB STEP）、手册 PDF 与上位机软件。
因此夹爪模型是**完全自研**的原始几何体模型（原始几何体 = box，零外部资产依赖）。

### 模型结构

| 项 | 值 |
|----|----|
| 本体 link | `gripper_base`（**不叫** `base_link`：夹爪与机械臂常同 plant，frame 名必须全局唯一） |
| 手指 link | `finger_left` / `finger_right` |
| 开合关节 | `finger_joint_left`（axis `0 1 0`）、`finger_joint_right`（axis `0 -1 0`），均 `prismatic`，限位 `[0, 0.0325] m` |
| TCP | link `tcp`，经 fixed 关节 `tcp_joint` 挂在 `gripper_base`（`xyz = 0 0 0.06`），位于两指中点、指尖高度，z 朝外 |
| collision | 3 个 box，名字 `gripper_base_collision` / `finger_left_collision` / `finger_right_collision` |

**关键约束：禁止 `<mimic>`**——实测 Drake 1.51.1 / 1.55.0 解析 mimic 后仍把两个
prismatic 各自计入 DoF（dof=9），存在双驱动风险。双指对称由
`src/rm75_drake/gripper.py` 的映射层实现（两关节 `q` 都取单指偏移量，
方向差异由 URDF 的 axis 符号承担）。

### 夹爪真值来源

| 真值 | 来源 | 原文 / 数值 |
|------|------|-------------|
| 开口指令量纲与范围 | 官方手册 `Related Files/Two-Finger Parallel Gripper User Manual V1.0.pdf` | 开口参数 **0–1000，无量纲**，其中 **1000 对应标准夹指 65 mm** |
| 动作语义 | 同上手册 | 打开/闭合（最大速度）、力矩夹取（速度 + 力矩，力矩超限停止）三种动作 |
| SDK 接口 | `robotic_arm` 1.1.7（本地 wheel 已解包核验） | `rm_set_gripper_position(position:int)` 绝对开口 0–1000；`rm_set_gripper_pick(speed:int, force:int)` 力控夹取；`rm_set_gripper_release(speed, block, timeout)`；`rm_get_gripper_state()` 开口反馈（判"夹空/夹住"） |
| 单指最大行程 | 由 65 mm 推导 | `65 mm / 2 = 32.5 mm`（`GRIPPER_FINGER_TRAVEL_MAX_M = 0.0325`） |
| 几何尺寸 | **自研**（无官方网格/URDF） | box 近似，尺寸以"标准夹指 65 mm"为锚；官方 STEP（`Hardware/CAD/RMG24 Claw 3D Model.stp`）本项目未解析（无 CAD 内核依赖） |
| u→N 力曲线 | **官方未给出** | 落地为 `src/rm75_drake/calibration/gripper_force.yaml`（`calibrated: false`）；`force_from_position()` 显式抛"未标定"，**禁止伪造牛顿数值** |

### 上游夹爪仓可取得的信息（来自元数据归档）

归档：`third_party/archives/two_finger_gripper-91f587b1d4ef-metadata.tar.gz`
（sha256 `975b5474afbac9de17bf2ea2601d6eacd0267ecdc54c0f46640115ac64d154a5`，
480 个文件条目，仅记录**文件名与字节大小**，不含二进制内容）。

从该归档取得的固件文件名 / 版本信息：

* 应用固件：`Software/gripper_app/Keil/MDK-ARM/board/gripper_app.bin`（58 692 B）、
  `gripper_app.hex`（161 476 B）、`app.bin`（65 612 B）、
  `board.hex`（180 500 B）、`jiazhua_D103.bin`（49 842 B）、
  `RD03_one_loop_formal_D101_H.bin`（49 842 B）、`RD10(1).bin`（45 150 B）；
* 引导固件：`Software/gripper_boot/Keil/MDK-ARM/board/gripper_boot.bin`（18 616 B）、
  `gripper_boot.hex`（51 248 B）、`boot.bin`（18 636 B）、`tool.bin`（20 284 B）、
  `board.hex`（51 300 B）；
* 上位机：`Software/Two-Finger Parallel Claw Master Control v1.0.3/gripper_tools.exe`；
* 机械/电气资料：`Hardware/CAD/RMG24 Claw 3D Model.stp`、
  `Hardware/BOM/Claw Interface Board BOM.xls`、Gerber 与 PCB 文件。

**R019 剩余项：夹爪固件哈希未取得**——本项目的归档策略是 metadata-only
（只归档文件名与大小，不归档二进制内容），因此固件二进制的 sha256 **未取得**，
本节**不写任何伪造哈希**。需要固化固件哈希时，应改为完整归档并把 sha256 写入
`third_party/MANIFEST.yaml`。

### 装配要点（`src/rm75_drake/models.py`）

* `load_rm75_gripper()` 默认把 `gripper_base` **weld 到 world**：若夹爪基座自由浮动，
  Drake 会给它 6 个自由度 + 2 个四元数（`num_positions` 从 2 变 9），与"夹爪恰有
  2 个 prismatic 位置自由度"的模型契约冲突；
* `load_rm75_gripper(plant=..., attach_to="link_7")` 把夹爪挂到机械臂法兰，
  与机械臂同 plant 装配成一条运动链（DoF = 7 + 2）。

### 复现本节结论

```bash
python3 -m pytest tests/test_gripper.py -q
```
