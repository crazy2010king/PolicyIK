# rm_hw_bridge 硬件适配层说明

> 面向"没参与过本任务的人"：照着本文，可以在真机窗口内从零把 RM-API2 接进本项目，
> 并知道**哪些能改、哪些绝对不能碰**。
>
> 关联切片：M001-3kzz40 / S02（厂商解耦的硬件适配层 rm_hw_bridge）；
> 关联需求：R003（厂商解耦）、R004（状态快照全字段）、R010（doctor 分级检查项）；
> 关联文档：`docs/code_doc/真机基线与实测限值归档说明.md`（基线采集口径）。

---

## 0. 一分钟索引

| 我要做的事 | 命令 / 文件 |
|---|---|
| 看硬件面体检结果（5 条 `hw:*`） | `python3 -m rmctl doctor` |
| 连真机看状态快照（R004 全字段） | `python3 -m rmctl connect --arm left --ip <ip> --json` |
| 探夹爪（协议/波特率/ID/寄存器基数/供电/代际） | `python3 -m rmctl gripper probe --arm left --json` |
| 夹爪开合自检 + 六类判定自检（离线代理） | `python3 -m rmctl gripper cycle --count 100 --json` / `python3 -m rmctl gripper judge --fault each --count 10 --json` |
| 看夹爪接入手册（接线 / 协议 / 寄存器表 / 判定表 / 恢复阶梯 / 互锁 / 真机回填） | `docs/rm_gripper接入与状态判定.md` |
| 跑夹爪离线证据链（三段断言，mode=fake） | `python3 scripts/gripper_offline_evidence.py --out build/reports/s05_offline_gripper.json` |
| 复核白/黑名单静态审计 | `python3 -m rm_hw_bridge.audit --repo`（退出码 0 = 零违规，3 = 有违规） |
| 看 ABI 唯一真值 | `dualarm_ws/src/rm_hw_bridge/rm_hw_bridge/abi_spec.yaml` |
| 看符号契约唯一真值（22 核心 + 21 支持 + 黑名单） | `dualarm_ws/src/rm_hw_bridge/rm_hw_bridge/contract.yaml` |
| 看生成物（ctypes 绑定 / 布局黄金表） | `dualarm_ws/src/rm_hw_bridge/rm_hw_bridge/generated/` |
| 重新生成绑定（改了 abi_spec.yaml 之后） | `python3 -m rm_hw_bridge.codegen --spec dualarm_ws/src/rm_hw_bridge/rm_hw_bridge/abi_spec.yaml --out dualarm_ws/src/rm_hw_bridge/rm_hw_bridge/generated` |
| 检查生成物是否漂移 | 同上加 `--check`（漂移时 stderr 打印「生成物漂移:」清单） |

---

## 1. 分层与边界（厂商解耦 R003）

```
rmctl（体验层：规划 / 基线 / CLI）
  └── rm_hw_bridge（硬件适配层：ABI 规格 → ctypes 绑定 → 句柄语义 → 快照 / 限值 / 停机族）
        └── RM-API2 1.1.7（libapi_c.so，厂商 C 库）
```

* **依赖方向是单向的**：rmctl → rm_hw_bridge → 厂商 `.so`。
  硬件层**不得** import rmctl（契约 `rules.hardware_must_not_import`），
  也不得 import `Robotic_Arm` / `tesseract*` / `moveit*`
  （契约 `forbidden_imports`），否则"厂商解耦"这条边界就破了。
* **真值只有三处**，其余全部是它们的派生物：

  | 真值 | 文件 | 派生物 |
  |---|---|---|
  | ABI 规格（结构体布局 / 符号签名 / 单位） | `rm_hw_bridge/abi_spec.yaml` | `generated/rm_api2_ctypes.py`、`generated/layout_golden.json` |
  | 符号契约（谁允许被调用 / 谁禁止） | `rm_hw_bridge/contract.yaml` | `loader.py`（只绑清单符号）、`audit.py`（静态审计） |
  | 实测限值基线 | `workcell/baselines/measured_limits.yaml` | `limits.py` 的限值默认来源 |

* **清单不写在代码里**：白名单/黑名单符号名在 `contract.yaml`；代码里不硬编码，
  装载器与审计器都只读它（杜绝"清单写在代码里、注释写在别处"的双真值漂移）。

---

## 2. ABI 真相表（RM-API2 1.1.7）

> 口径来自 `abi_spec.yaml`（字段名/顺序/类型逐字抄自厂商头文件
> `rm_define.h` / `rm_interface.h` / `rm_version.h`，
> 来源与 sha256 见 `vendor/rm_api2/PROVENANCE.md`）。

### 2.1 七条硬规则

| # | 规则 | 踩坑后果 |
|---|---|---|
| 1 | **句柄语义**：`rm_create_robot_arm` 失败时**仍返回非空指针**，但 `handle.id == -1`；必须读 `id` 判失败（达到最大连接数 5 时才返回 NULL） | 只判 NULL → 把"臂不可达"当成连上了，后续全是脏数据 |
| 2 | **结构体按值传**：`rm_set_realtime_push` / `rm_movej_canfd` 的配置结构体**按值**传（不是指针） | 传指针 → 布局错位，控制器读到垃圾周期 |
| 3 | **`float*` 缓冲生命周期**：`rm_movej_canfd_mode_t.joint` 是 `float*`，调用方必须保证缓冲在**调用返回前存活** | 缓冲被 GC → 静默错值（本层用显式持有 + 有界保留队列，见 §4.3） |
| 4 | **全局回调**：`rm_realtime_arm_state_call_back` 是**进程级全局注册**（单例） | 同进程接两路臂 → 第二路收不到上报（见 §8 已知限制） |
| 5 | **`char` 数组**：`rm_realtime_push_config_t.ip` 是 `char[28]`，赋值必须用 **bytes** | 传 str → 段错误或读出乱码 IP |
| 6 | **单位**：关节角 **°**；`rm_set_joint_drive_max_speed` **°/s**；`rm_set_joint_drive_max_acc` **°/s²**；`rm_set_joint_drive_min_pos` / `max_pos` **°** | 单位错 → 限值下发成数量级错误的速度 |
| 7 | **`rm_movej` 是阻塞式调用** | 单线程里"边运动边采样"会卡死（采样须放后台线程） |

> 内部一律 SI（rad / rad/s / rad/s² / rad/s³），**调用 SDK 前**才换算成度制；
> 换算常量收敛在 `rm_hw_bridge/bridge.py` 的 `DEG2RAD` / `RAD2DEG`（单点）。

### 2.2 关键结构体（节选，完整表见 `abi_spec.yaml`）

| 结构体 | 关键字段 | 传参方式 |
|---|---|---|
| `rm_robot_handle` | `id`（int，失败为 -1） | 返回 `rm_robot_handle*` |
| `rm_realtime_push_config_t` | `ip`（char[28]）、`port`（int）、`cycle`（int，**5 的倍数**） | **按值** |
| `rm_movej_canfd_mode_t` | `joint`（**float\***）、`trajectory_mode`、`pos`/`vel`/`acc` 等 | **按值**（`joint` 指向的缓冲须存活） |
| `rm_current_arm_state_t` | `err`（int[]）、`current_arm_state`、`move_status` | 出参指针 |
| `rm_arm_all_state_t` | 关节角 / 电流 / 温度 / 电压 / 使能位 / 错误码 | 出参指针 |

`cycle` 必须是 **5 ms 的整数倍**（厂商头文件注释 + `network_plan` 基线），
否则控制器不下发主动上报。

---

## 3. 符号清单（22 核心 + 21 支持）

> **以 `rm_hw_bridge/contract.yaml` 为唯一真值**（下表是它的人类可读快照；
> 代码里不硬编码任何符号名，装载器与审计器都只读契约文件）。
> 契约自带自校验：`limits.core_count = 22`、`limits.support_count = 21`。
> **core 22 个与黑名单 13 个一字未动**（R003 结论原样成立）；本次扩容只动
> **支持层**（5 → 21）：夹爪快路径 6 + Modbus 主路径 6 + RM Plus 2 + 工具端电压 2。

### 3.1 核心符号（恰好 22 个，缺一即契约破损）

| # | 符号 | 用途 | 单位 |
|---|---|---|---|
| 1 | `rm_init` | 初始化 SDK（**三线程模式**，否则收不到 UDP 主动上报） | - |
| 2 | `rm_create_robot_arm` | 创建机械臂实例（**必须读 id 判失败**，见 §2.1 规则 1） | - |
| 3 | `rm_delete_robot_arm` | 删除实例（句柄释放的唯一入口） | - |
| 4 | `rm_set_realtime_push` | 设置 UDP 主动上报（配置**按值**传递） | 周期 ms |
| 5 | `rm_get_joint_degree` | 获取关节角度 | ° |
| 6 | `rm_get_current_arm_state` | 获取机械臂当前状态（工作/拖动/停止） | - |
| 7 | `rm_get_arm_all_state` | 获取所有状态（**R004 快照的数据源**） | - |
| 8 | `rm_get_joint_err_flag` | 获取关节错误标志与抱闸状态 | 位标志 |
| 9 | `rm_get_arm_current_trajectory` | 获取当前规划类型与轨迹数据 | ° |
| 10 | `rm_movej` | 关节空间运动（**阻塞式**） | ° / °·s⁻¹ |
| 11 | `rm_movej_canfd` | **关节角透传**（配置按值；`joint` 缓冲须存活） | ° |
| 12 | `rm_set_delete_current_trajectory` | 删除当前轨迹 | - |
| 13 | `rm_set_arm_stop` | 急停 | - |
| 14 | `rm_set_arm_slow_stop` | 缓停 | - |
| 15 | `rm_set_arm_pause` | 暂停 | - |
| 16 | `rm_set_arm_continue` | 继续 | - |
| 17 | `rm_set_joint_drive_max_speed` | 下发关节最大速度 | °/s |
| 18 | `rm_set_joint_drive_max_acc` | 下发关节最大加速度 | °/s² |
| 19 | `rm_set_joint_drive_min_pos` | 下发关节最小限位 | ° |
| 20 | `rm_set_joint_drive_max_pos` | 下发关节最大限位 | ° |
| 21 | `rm_set_collision_state` | 碰撞检测分级（**安全开关**，0=不检测，8=最灵敏） | 0..8 |
| 22 | `rm_set_self_collision_enable` | 自碰撞检测使能（**安全开关**） | bool |

### 3.2 支持层符号（21 个，不计入 22，但同样受白名单约束）

| 符号 | 为什么必须允许 |
|---|---|
| `rm_set_log_call_back` | 接管 SDK 的 printf 噪声，避免污染 `rmctl` 输出与日志 |
| `rm_set_timeout` | SDK 通信超时：真机不可达时**可控地快速失败**，而不是无限挂住 |
| `rm_get_robot_info` | 版本矩阵 / 机型（判定控制器固件与机型是否匹配） |
| `rm_get_realtime_push` | 主动上报配置回读（下发后确认真的生效） |
| `rm_realtime_arm_state_call_back` | UDP 主动上报回调注册（**进程级全局单例**，见 §8） |
| **夹爪快路径** | |
| `rm_set_gripper_route` | 设定夹爪开口行程上下限（0~1000，无量纲） |
| `rm_set_gripper_release` | 松开夹爪（快路径，block/timeout 阻塞语义） |
| `rm_set_gripper_pick` | 力控夹取（快路径，夹到判定的主下发接口） |
| `rm_set_gripper_pick_on` | 持续力控夹取（保持夹持力） |
| `rm_set_gripper_position` | 夹爪开到指定开口（1~1000，无量纲） |
| `rm_get_gripper_state` | 读夹爪状态（六类判定的**唯一数据源**） |
| **Modbus-RTU 主路径** | |
| `rm_set_modbus_mode` | 打开端口 Modbus RTU 模式（主路径前置，启动后必须先调） |
| `rm_close_modbus_mode` | 关闭端口 Modbus RTU 模式（关闭后自动切回机械臂控制模式） |
| `rm_read_holding_registers` | 读保持寄存器（`params` **按值**传递） |
| `rm_read_input_registers` | 读输入寄存器（`params` **按值**传递） |
| `rm_write_registers` | 写多个寄存器（`num` ≤ 10，`params` **按值**传递） |
| `rm_write_single_register` | 写单个寄存器（`params` **按值**传递） |
| **RM Plus 末端生态协议** | |
| `rm_set_rm_plus_mode` | 设置末端生态协议模式（0 禁用 / 9600 / 115200 / 256000 / 460800） |
| `rm_get_rm_plus_mode` | 查询末端生态协议模式（回读，确认真的生效） |
| **工具端电压** | |
| `rm_set_tool_voltage` | 设置工具端供电档位（0V / 5V / 12V / 24V） |
| `rm_get_tool_voltage` | 获取工具端供电档位（回读，确认夹爪供电正常） |

> **两条互斥路径**：厂商手爪控制协议与末端 Modbus 功能**互斥**（同一时刻只能开一条）。
> 快路径（`rm_set_gripper_*`）走控制器内置手爪协议；通用夹爪走 Modbus-RTU 主路径
> （`rm_set_modbus_mode` → 读写寄存器 → `rm_close_modbus_mode`）。
> 两组参数的 `rm_peripheral_read_write_params_t` 都是**按值**传递（与
> `rm_set_realtime_push` / `rm_movej_canfd` 同写法，见 §2.1 规则 2）。

### 3.3 黑名单（禁止调用，静态审计按"调用形态"拦截）

* 位姿透传类：`rm_movep_canfd`、`rm_movep_follow`、`rm_movel`、`rm_moves`、`rm_movec`、`rm_movej_p`
* 坐标系 / 零位类：`rm_set_pos_step`、`rm_set_ort_step`、`rm_set_teach_frame`、
  `rm_set_manual_tool_frame`、`rm_change_tool_frame`、`rm_update_tool_frame`、`rm_set_joint_zero_pos`
* 前缀类：`rm_algo_*`（厂商解算 / 规划内核；`.so` 实测导出 60 个 `rm_algo_*` 符号）

**为什么位姿透传在黑名单**：控制器会**偷偷做逆解**，把"我给的位姿"变成"我算的关节角"，
违反 R003（厂商解耦）与"关节角透传"的设计约束。关节角透传只走 `rm_movej_canfd`。

**拦截方式**：AST 级 `ast.Call` 判定（不是正则）——字符串常量里的 `"rm_algo_xxx("`
不会被误判成调用，但真调用会带**行号**报到。
`python3 -m rm_hw_bridge.audit --repo` 逐条打印，形如：

```
dualarm_ws/src/xxx/yyy.py:42: [blacklist-call] 调用黑名单符号 rm_algo_xxx（禁止）
```

---

## 4. 状态快照（R004 全字段）

### 4.1 字段表

`rm_hw_bridge.snapshot.StateSnapshot.to_dict()` 的字段（schema `rm_hw_bridge.snapshot/1`）：

| 字段 | 含义 | 单位 | 来源 |
|---|---|---|---|
| `joint_deg` | 7 个关节角 | ° | `rm_get_joint_degree` / `rm_get_arm_all_state` |
| `joint_current_mA` | 关节电流 | mA | `rm_get_arm_all_state` |
| `joint_temperature_c` | 关节温度 | ℃ | `rm_get_arm_all_state` |
| `joint_voltage_v` | 关节电压 | V | `rm_get_arm_all_state` |
| `joint_en_flag` | 关节使能位（1 使能 / 0 掉使能） | - | `rm_get_arm_all_state` |
| `joint_err_code` | 关节错误码 | - | `rm_get_arm_all_state` |
| `arm_err` | 臂级错误码列表 | - | `rm_current_arm_state_t.err` |
| `brake_state` | 抱闸状态：`err_flag` / `brake_state` / `brake_open[7]` / `hard_brake_joints[1..4]` / `soft_brake_joints[5..7]` | - | `rm_get_joint_err_flag` |
| `controller_trajectory_type` | 控制器规划类型（int） | - | `rm_get_arm_current_trajectory` |
| `controller_trajectory_type_name` | 枚举名（`RM_NO_PLANNING_E` 表示**纯透传**） | - | 同上 |
| `arm_current_status` | 控制器状态机（UDP 实时上报） | - | UDP 主动上报 |
| `arm_current_status_name` | 状态枚举名（`RM_MOVE_J_E` / `RM_STOP_E` …） | - | 同上 |
| `realtime` | 上报统计：`frames` / `last_ts` / `period_ms` | ms | UDP 主动上报 |
| `controller_pose` | 厂商运动学输出（**仅交叉校验**） | m / deg | `rm_get_arm_all_state` |
| `controller_pose_scope` | 恒为 `"cross_check_only"` | - | 本层常量 |

### 4.2 边界声明（`controller_pose_scope = "cross_check_only"`）

**厂商运动学输出只用于 FK 交叉校验，不得作为规划真值。**
本项目的位姿真值是**自研 FK**（`rmctl/fk_cross.py` 交叉校验：位置偏差 < 1 mm、
姿态偏差 < 0.05°）。快照里带厂商位姿只是为了**核对**两套算法是否一致，
谁把它当规划真值，谁就在偷偷依赖厂商解算内核——那正是 R003 要切断的依赖。

### 4.3 诚实降级（失败时绝不伪造数值）

| 情形 | 结果 |
|---|---|
| 缺 SDK（`.so` 找不到） | `status="blocked"`（退出码 3）+ 搜索路径 + 可粘贴修复动作 |
| `handle.id == -1` / 任何 SDK 调用返回非 0 | `status="error"`（退出码 4），写明**接口名 + 返回码** + 可粘贴下一步 |
| 关节角透传 | 桥接层**显式持有** ctypes 缓冲（`buf` + `config` 引用），并有界保留（`MAX_RETAINED_JOINT_BUFFERS = 64`），杜绝 GC 提前回收导致静默错值、也不会在 10 ms 调度下无限堆积 |

**失败路径不产生任何伪造数值**：失败结果里没有关节角、没有状态、没有"已连通"。

---

## 5. 限值口径（默认值的唯一来源）

| 事项 | 口径 |
|---|---|
| 默认值来源 | `workcell/baselines/measured_limits.yaml`（实测基线） |
| 基线 `status != measured` | **拒绝下发**（`status="blocked"`，`values` 为空）——**绝不拿占位值糊过去** |
| 占位回退 | 必须调用方**显式** `allow_placeholder=True`；此时 `source` 固定标注 `"joint_limits.yaml (placeholder, 未实测)"` |
| 实测后的组合 | acc / jerk 取**实测值**；速度 / 位置取 `joint_limits.yaml` 的**声明值** |
| 单位换算 | 内部 SI（rad/s、rad/s²、rad/s³、rad），调用 SDK 前换算为 °/s、°/s²、° |

下发前先看 diff：`link.set_joint_limits(defaults)` **默认 dry-run**（零 SDK 调用），
stdout 打印「限值 diff（当前值 → 目标值，含单位）」，结果字典含 `diff` / `applied` / `sdk_calls`。
任何 SDK 返回码非 0 → 报错并写明接口名与返回码。

---

## 6. doctor 的 5 条硬件面检查项（`hw:*`）

`python3 -m rmctl doctor` 现在共 **21 条**检查项，其中 5 条是硬件适配层：

| 检查项 | 判定 | 分级 | 修复动作（可粘贴） |
|---|---|---|---|
| `hw:sdk` | RM-API2 `.so` 在位（`RM_API2_LIB` → `.deps/rm_api2/libapi_c.so` → 目录内任意 `.so`） | 缺失 = **INFO**（不阻塞退出码） | `export RM_API2_LIB=/abs/path/to/libapi_c.so`（或物化到 `.deps/rm_api2/`） |
| `hw:symbols` | 22 个核心符号全部可解析（用 `rm_hw_bridge.loader` 绑定校验） | SDK 缺失 = INFO；**符号缺失 = FAIL（阻塞）** | `nm -D --defined-only <so> \| grep rm_` |
| `hw:contract` | 白/黑名单静态审计**零违规**（跑 `rm_hw_bridge.audit` 的仓库扫描） | 有违规 = **FAIL（阻塞）**，列出违规文件与符号 | `python3 -m rm_hw_bridge.audit --repo` |
| `hw:limits_source` | `measured_limits` 基线 `status` 是否为 `measured` | 非 measured = **INFO**（不阻塞） | `python3 dualarm_ws/src/rm_description/scripts/apply_measured_limits.py --dry-run` |
| `hw:gripper` | `workcell/rm_workcell.yaml::gripper` 段必需字段齐全？标定系数是否已回填？ | 缺必需字段 = **FAIL（阻塞）**，reason 逐字段指名；标定系数为 null = **INFO**（不阻塞，判定层据此走 `needs_param` 属预期） | 缺字段 → 补齐 `gripper` 段；标定 → 在真机窗口回填 `position_mm_per_unit` / `force_scale` / `force_threshold` / `grasp_position_window` |

`hw:contract` 之所以是**阻塞**项：黑名单调用会绕过 R003 的厂商解耦红线，
属于代码缺陷而不是"环境没就位"。

### 6.1 机器可读输出与退出码口径（S09/T6）

```bash
python3 -m rmctl doctor                                  # 人读文本
python3 -m rmctl doctor --json                           # stdout = 机器可读 JSON
python3 -m rmctl doctor --json --report build/reports/s09_doctor.json
```

JSON 顶层含 `status` / `verdict` / `exit_code` / `reason` / `next_action` /
`summary` / `checks[]` / `coverage_matrix[]`；`--report` 落盘内容与 `--json`
的 stdout **等价**。退出码与全项目一致：

| 退出码 | 含义 |
|---|---|
| 0 | 全部检查项 PASS（INFO 级未通过**不算**失败） |
| 3 | 存在需人工 / 外部条件的项（RECOVERABLE / NEEDS_HUMAN / BLOCKED） |
| 4 | 存在硬红线项（FAIL：黑名单调用、SDK 核心符号缺失、夹爪配置非法） |

`coverage_matrix[]` 是**自检覆盖矩阵**（设计 §6.2「自检覆盖 ≥ 8 类」）：
网络 / 供电 / 夹爪 / 零位 / FK / 限值 / 控制器状态 / 依赖与模型 / 碰撞 /
硬件适配层，共 10 类；每类映射到具体 check name，**映射不到**（例如厂商库
不可导入导致 `model:load` 失败、FK/碰撞/限值一致性被级联跳过）的类别显式
标 `covered=false` + 缺口说明 + 可粘贴的修复命令——**不会**被标成「通过」。

---

## 7. 真机窗口操作手册

### 7.1 环境

```bash
source scripts/rmenv.sh     # 设好 RM_WS / PYTHONPATH / LD_LIBRARY_PATH
# RM-API2 动态库放法（二选一）：
#   export RM_API2_LIB=/abs/path/to/libapi_c.so
#   # 或物化到 <repo>/.deps/rm_api2/libapi_c.so（.deps/ 是 gitignored，不入库）
```

### 7.2 体检

```bash
python3 -m rmctl doctor
# 关注 5 条 hw:*：hw:sdk / hw:symbols 必须 PASS；
# hw:limits_source 在真机窗口之前是 INFO（基线未实测），窗口回填后转 PASS；
# hw:gripper 在标定系数回填前是 INFO（配置齐备即合法），缺必需字段才 FAIL。
```

### 7.3 连真机看快照

```bash
python3 -m rmctl connect --arm left --ip <左臂IP> --json
# 失败时：status=blocked / error + reason + 可粘贴的 next_action（**不伪造数值**）
```

### 7.4 六类实验（只跑脚本、只落数据，不交互式调试）

```bash
python3 -m rmctl.hwcollect udp_stability     --arm left --duration-s 600 --out <dir>
python3 -m rmctl.hwcollect ladder_acc_jerk   --arm left --trials 3 --out <dir>
python3 -m rmctl.hwcollect tcp_repeatability --arm left --repeats 30 --out <dir>
python3 -m rmctl.hwcollect fk_cross          --arm left --samples 20 --out <dir>
python3 -m rmctl.hwcollect zero_capture      --arm left --photo <照片引用> --out <dir>
python3 -m rmctl.hwcollect estop_drill       --arm left --out <dir>
# 离线预演（不碰真机）：加 --dry-run
```

各实验产出哪类基线字段、字段含义与单位，详见
`docs/code_doc/真机基线与实测限值归档说明.md`。

### 7.5 回填限值并把基线从 blocked 改成 measured

```bash
# 1) 先看回填预览（不写盘）
python3 dualarm_ws/src/rm_description/scripts/apply_measured_limits.py --dry-run
# 2) 真正写盘（去掉 --dry-run）
python3 dualarm_ws/src/rm_description/scripts/apply_measured_limits.py
# 3) 把 workcell/baselines/measured_limits.yaml 的 status 从 blocked 改成 measured
#    （并填齐每关节 acceleration/jerk 的 value/unit/source/method/samples，samples ≥ 3）
# 4) 复检：hw:limits_source 应转 PASS，且 limits.load_limit_defaults() 返回 status=measured
python3 -c "from rm_hw_bridge import limits; print(limits.load_limit_defaults()['status'])"
```

> **红线**：`status=measured` 时数值不得高于 `joint_limits.yaml` 的已生效限值；
> 未实测项一律写 `null`，**禁止填占位数值**。

### 7.6 夹爪体验层（`rmctl gripper`）

```bash
# 1) 探针：协议状态 / 波特率档 / 设备 ID / 寄存器基数 / 工具端供电 / 控制器代际线索
#    （“未实测假设”逐条列出；真机不可达时 status=blocked + 可粘贴 next_action，退出码 3）
python3 -m rmctl gripper probe --arm left --json

# 2) 开 / 合（--dry-run 只走参数与配置解析，**不下发任何动作**）
python3 -m rmctl gripper open  --arm left --dry-run --json
python3 -m rmctl gripper close --arm left --speed 100 --force 100 --json

# 3) 开合 100 次（**离线代理**：mode=fake，输出带“代理模型，不代表真机结论”）
python3 -m rmctl gripper cycle --count 100 --json

# 4) 六类判定自检（夹到/夹空/堵转/过温/过流/离线，各 10 次；任一不符 → 退出码非 0）
python3 -m rmctl gripper judge --fault each --count 10 --json
```

* 退出码沿用既有口径：`0 ok / 2 参数非法 / 3 blocked / 4 error`；
* 配置（`gripper:` 段）缺必需字段 → `status=blocked` + 退出码 3，reason **指名**缺哪个字段；
* `cycle` / `judge` **永远走离线代理**（硬件层 `FakeGripper`），
  输出必须带 `mode=fake`；真机结论一律报 blocked 并给出回填入口，**绝不以离线代理冒充真机结论**；
* 判定真值只在 `rm_hw_bridge.gripper_state.judge`，CLI **不另写一套判定**；
* 标定系数（`position_mm_per_unit` / `force_scale` / `force_threshold` /
  `grasp_position_window`）未回填时，判定层走 `needs_param`（属预期，不阻塞）。

---

## 8. 已知限制

| 限制 | 原因 | 现状 / 绕行 |
|---|---|---|
| **双臂同进程不可行** | `rm_realtime_arm_state_call_back` 是**进程级全局注册**（单例），同进程只能接一路臂的主动上报 | 左右臂分进程跑；单进程内不得并行接两臂（R014/R015，Out of Scope） |
| **SDK 会向 stdout 打印噪声** | RM-API2 的噪声是 **C 层 printf**（如 `[rm_create_robot_arm] socket connect err!`），Python 层重定向拦不住 | 已用 `rm_set_log_call_back` 接管 SDK 日志；`rmctl connect --json` 另在 **fd 1** 层面重定向，保证输出是**单个可 `json.loads` 的对象** |
| **`.so` 单客户端占用约束** | 同一 `ip:port` 只允许一个客户端（见 `network_plan.yaml` 的 `constraints.single_client_per_endpoint`） | 采集脚本独占连接；不要同时开 `connect` 与 `hwcollect` |
| 回调被 GC 会段错误 | ctypes 回调对象被回收后，SDK 持有的是悬空函数指针 | 桥接层**显式持有**回调与缓冲引用（`_log_callbacks` / `_joint_buffers`） |
| SDK 版本必须与契约一致 | 契约里的符号以 RM-API2 **1.1.7** 为准 | `hw:symbols` 检查会报缺失符号清单；换 SDK 需重新核对 `abi_spec.yaml` |

---

## 9. 排障速查

| 现象 | 查什么 |
|---|---|
| `hw:sdk` INFO | `.so` 是否在 `RM_API2_LIB` / `.deps/rm_api2/`；`ldd <so>` 看依赖是否齐全 |
| `hw:symbols` FAIL | 缺哪些符号在 detail 里；多半是 SDK 版本不对（契约按 1.1.7） |
| `hw:contract` FAIL | `python3 -m rm_hw_bridge.audit --repo` 逐条看 `文件:行: [kind] 说明` |
| `hw:limits_source` INFO | 基线 `status` 还是 blocked/pending → 走 §7.5 回填流程 |
| 快照里 `controller_trajectory_type_name` 不是 `RM_NO_PLANNING_E` | 控制器在**偷偷规划**（透传失效），停下来查是谁下发了位姿类指令 |
| `rm_get_joint_degree` 返回 -1 / "机械臂已断开" | 控制器不可达或已被别的客户端占用（见 §8） |
