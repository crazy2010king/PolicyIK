# FK 验收门槛口径（FK Tolerance Doctrine）

> **状态**：已接受（accepted）—— 里程碑 M001-acm1jg / 切片 S09 / 任务 S09-T04 落定。
> **适用范围**：官方 RM75-B URDF 与独立 MDH 正解（`src/rm75_drake/kinematics/mdh.py`）之间的 FK 一致性验收。
> **唯一真相源**：`src/rm75_drake/kinematics/mdh.py` 的模块级常量（见文末清单）。
> **本文件取代（覆盖）**：`01-01-PLAN.md:11`、`01-01-PLAN.md:86`、`01-CONTEXT.md:225` 中
> 「位置误差 < 1e-6 m」的旧表述（详见「口径覆盖声明」一节）。

---

## 1. 残差来源：官方 URDF 的 rpy 是四舍五入值

官方 RM75-B URDF（`models/rm75_drake/rm75_drake.urdf`）在每个关节的 `<origin rpy=...>`
上使用**官方舍入值 `1.5708`**，而不是精确的 ±π/2：

```
|1.5708 − π/2| ≈ 3.6732e-6 rad
```

经典（改进）D-H 链把连杆偏移沿「关节转过之后的 z 轴」平移
（`T = ∏ Rotz(q_i) · Transz(d_i) · Rotx(alpha_i)`），而 URDF 链把同一偏移放在
**父系的 y 方向**上。这两种写法只有在 ±90° **精确成立**时才完全重合；
一旦 URDF 侧用舍入值 `1.5708`，两条链之间就出现**固有的位置残差**：

* 零位（q = 0）link_7 的 y 偏差实测 **2.24065511e-06 m**，即期望值
  **2.2407e-6 m**（带宽 ±1e-8 m）；
* 该残差 ≈ 3.6732e-6 rad × 约 0.6 m 的力臂，与上面的量级推导一致。

**结论**：Drake 载入的（官方舍入 URDF）FK 相对**精确**模型本身就偏约 2.24e-06 m。
任何独立的解析模型都不可能在严格门槛以内与它对齐——这是**模型来源决定的已知量**，
不是被测实现的缺陷。

## 2. 接受口径（两条互不等价的门槛）

| 判据对象 | 位置门槛 | 常量 | 含义 |
|---|---|---|---|
| **真实官方 URDF**（rpy = 1.5708 舍入） | **< 5e-6 m** | `FK_POSITION_TOLERANCE_URDF_M` | 上界由官方舍入的固有残差决定，故放宽 |
| **精确 ±π/2 的 URDF 变体**（去掉官方舍入） | **< 1e-6 m** | `FK_POSITION_TOLERANCE_EXACT_PI2_M` | 去掉舍入后 D-H 约定与 URDF 链完全等价，可用严格门槛判定 |
| 姿态（旋转矩阵逐元素差） | < 1e-9 rad | `FK_ROTATION_TOLERANCE_RAD` | 舍入对姿态无影响（实测 < 1e-15 rad），故保持严格 |

配套的已知量锚点：

* 零位 y 残差期望值 **2.2407e-6 m**（`ZERO_Y_ROUNDING_BIAS_M`），带宽 ±1e-8 m
  （`ZERO_Y_ROUNDING_BIAS_TOLERANCE_M`）——把残差**钉成已知量**，而不是未知误差；
* 鉴别力门槛 **1e-3 m**（`WRONG_TABLE_DETECTION_FLOOR_M`）——错误 alpha/d 表与真值的差
  必须**超过**该值。

**为什么放宽不是「悄悄放宽」**：真实 URDF 门槛 5e-6 m 的下界被
`test_zero_pose_rounding_bias` 以 2.2407e-6 ± 1e-8 单独钉死，因此
「门槛被放宽」与「残差被解释」是两件可分别判定的事。

## 3. 鉴别力证据（门槛有区分能力，不是恒真）

| 对照 | 实测量级 | 判据 |
|---|---|---|
| 正确 alpha 表 vs 真实 URDF（随机 100 组位形） | 位置误差 < 5e-6 m（实测量级 ~2.3e-6 m） | PASS（正例） |
| 精确 ±π/2 变体 vs 精确 ±π/2 MDH | 位置误差 < 1e-6 m | PASS（正例，证明约定等价） |
| **错误 alpha 表（全 0）** vs 真实 URDF | 差 **> 1e-3 m** | PASS（负例，鉴别力成立） |
| **错误 d 表（末项 0.144 → 0.3）** vs 真实 URDF | 差 **> 1e-3 m** | PASS（负例，鉴别力成立） |

即：门槛 5e-6 m 与错误实现的差（> 1e-3 m）之间隔着两个数量级以上，
因此该门槛既能放过官方舍入残差，又能抓住真正的建模错误。

复算命令（仓库根目录执行）：

```bash
python3 -m pytest -q tests/test_mdh_fk.py
```

## 4. 口径覆盖声明（覆盖 .gsd 规划产物的旧表述）

以下 .gsd 规划产物中出现「位置误差 < 1e-6 m」的旧表述，**自本文件起失效**，
以本文第 2 节的「两条门槛」为准：

| 文件与位置 | 旧表述 | 现行口径 |
|---|---|---|
| `01-01-PLAN.md:11` | 「100 组随机构型下 Drake FK 与官方 MDH 独立正解位置误差 < 1e-6 m、姿态误差 < 1e-6 rad」 | 真实 URDF 位置误差 < **5e-6 m**；姿态误差 < **1e-9 rad**（官方舍入不影响姿态） |
| `01-01-PLAN.md:86` | 「与 Drake FK 比对：位置误差 < 1e-6 m、姿态角误差 < 1e-6 rad」 | 同上；「去掉舍入后 < 1e-6 m」的严格判据保留为**第二条**断言 |
| `01-CONTEXT.md:225` | 「FK 与官方 MDH 在 100 组随机构型下位置误差 < 1e-6 m」 | 同上 |

说明：`.gsd/` 下的规划产物是**只读的历史上下文**，本任务不直接改写它们；
本文件承担「口径变更的正式声明」职责，里程碑/需求层引用时一律以本文件为准。

## 5. 常量清单（唯一真相源）

全部定义在 `src/rm75_drake/kinematics/mdh.py`（模块级，**只此一处**）：

| 常量 | 值 | 用途 |
|---|---|---|
| `FK_POSITION_TOLERANCE_URDF_M` | 5e-6 | 真实官方 URDF 的 FK 位置门槛 |
| `FK_POSITION_TOLERANCE_EXACT_PI2_M` | 1e-6 | 精确 ±π/2 变体的 FK 位置门槛 |
| `FK_ROTATION_TOLERANCE_RAD` | 1e-9 | FK 姿态误差门槛 |
| `ZERO_Y_ROUNDING_BIAS_M` | 2.2407e-6 | 零位 y 残差期望值（已知量） |
| `ZERO_Y_ROUNDING_BIAS_TOLERANCE_M` | 1e-8 | 零位 y 残差期望值带宽 |
| `WRONG_TABLE_DETECTION_FLOOR_M` | 1e-3 | 鉴别力门槛（错误表必须超过） |
| `ZERO_HEIGHT_TOLERANCE_M` | 1e-9 | 零位高度锚点带宽 |
| `ZERO_X_TOLERANCE_M` | 1e-12 | 零位 x 锚点带宽 |
| `ZERO_Z_TOLERANCE_M` | 1e-6 | 零位 z 对齐 Drake 的带宽 |
| `ZERO_ROTATION_TOLERANCE_M` | 1e-12 | 零位姿态锚点带宽 |
| `MDH_HEIGHT_SUM_TOLERANCE_M` | 1e-12 | 高度锚点与 d 表求和的一致性带宽 |
| `ROUNDING_ALPHA_DELTA_RAD` | 3.6732e-6 | 官方舍入与精确 π/2 的差值（残差来源量化） |
| `ROUNDING_ALPHA_DELTA_TOLERANCE_RAD` | 1e-9 | 上述差值的带宽 |
| `FK_TOLERANCE_DOC` | `docs/architecture/fk-tolerance.md` | 本文档路径（供代码/测试引用） |

守护机制（`tests/test_mdh_fk.py`）：

* `test_fk_threshold_constants_are_single_source`：每个门槛常量在整个 `src/` 下
  只允许**定义一次**，且必须定义在 `mdh.py`；
* `test_no_bare_threshold_literal_in_this_file`：FK 测试文件不得出现裸的科学计数法
  阈值字面量，一律引用上述常量；
* `test_fk_tolerance_doc_exists_and_states_thresholds`：本文件必须存在且写明两条门槛、
  残差期望值与旧口径覆盖声明。

## 6. 目标 Proof / Notes 措辞草案（供切片关闭阶段回填需求库）

### R002（模型与 FK 基准）

**目标 Proof（validation 文本草案）**：

> 独立 MDH 正解（`src/rm75_drake/kinematics/mdh.py`，零 Drake 依赖）与改造后官方
> RM75-B URDF 的 Drake FK 在 100 组随机构型下位置误差 < **5e-6 m**、姿态误差
> < **1e-9 rad**；门槛口径见 `docs/architecture/fk-tolerance.md` 与
> `rm75_drake.kinematics.mdh` 的门槛常量。真实 URDF 门槛取 5e-6 m 而非 1e-6 m，
> 原因是官方 URDF 的 joint rpy = 1.5708 为四舍五入值，导致端点存在固有残差
> **≈ 2.2407e-6 m**（零位 link_7 的 y 偏差实测 2.24065511e-06 m，已钉死为已知量）；
> 去掉官方舍入的精确 ±π/2 变体上位置误差 < **1e-6 m**，证明 D-H 约定与 URDF 链
> 等价；错误 alpha/d 表的差 > **1e-3 m**，证明该比对具备鉴别力。

**目标 Notes 草案**：

> FK 位置门槛的唯一真相源是 `rm75_drake.kinematics.mdh` 的模块常量，测试层不再
> 散落裸字面量（由 `tests/test_mdh_fk.py` 的口径守护用例强制）。旧口径
> 「位置误差 < 1e-6 m」（见 `01-01-PLAN.md:11`、`01-01-PLAN.md:86`、
> `01-CONTEXT.md:225`）已由 `docs/architecture/fk-tolerance.md` 显式覆盖。

### R007（scenario 单一真相源与回归可复现，必要时同步）

**目标 Notes 草案（补充句）**：

> FK 残差口径已统一到 `docs/architecture/fk-tolerance.md`（真实 URDF < 5e-6 m、
> 精确 ±π/2 变体 < 1e-6 m），golden 指纹与录制回放的基线均在该口径下产生，
> 不受旧 1e-6 m 表述影响。
