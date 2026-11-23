# M001-acm1jg: 双臂规划与操作软件基线

**Vision:** 在 2 套睿尔曼 RM75-B 上建成一套只依赖厂商最底层接口（关节角完全透传 + UDP 主动上报状态）、不使用厂商任何解算与运动学能力、以 Drake 为规划内核的双臂运动规划与操作软件；把既有 9 份设计文档的六个架构冲突收敛为可施工裁决，并让"体验优于 MoveIt Pro"落到可复算指标上：双臂相对位姿约束规划成功率 ≥ 95%（P95 < 3 s）、5 ms 透传周期 P99.9 抖动 < 0.5 ms、末端跟踪误差 < 3 mm、主链路厂商解算接口调用为 0。

## Success Criteria

- 真实 RM75-B 上 Drake 规划的关节轨迹经自研实时层透传执行完毕，末端跟踪误差 < 3 mm，100 Hz 连续 30 min 无故障
- 两台 RM75-B 在同一 world 下共同抓持并搬运一个物体，全程保持末端相对位姿，规划成功率 ≥ 95%、P95 求解 < 3 s
- 一次真机运行的全部消息被录制，离线重放后末端轨迹与原始记录逐点一致（golden-file 比对通过）
- 注入拔网线、物体掉落、夹爪夹空、关节超限位四种故障，系统均进入安全态且按文档流程恢复成功
- 厂商解算接口调用次数在主链路中为 0（CI 静态检查 + 运行时审计双证）
- 新机器 30 分钟内一条命令重建完整环境，且不依赖 ROS2 发行版

## Slices

- [x] **S00: 环境与骨架** `risk:low` `depends:[]`
  > After this: 新机器上 30 分钟内一条命令重建完整环境，CI 三档可运行。

- [x] **S01: 模型与夹爪** `risk:medium` `depends:[S00]`
  > After this: 官方 URDF 改造后加载成功，惯量/FK/限位/质量/自碰撞回归测试全绿，夹爪可独立建模与开口量映射。

- [x] **S02: 规划内核与同源框架** `risk:medium` `depends:[S01]`
  > After this: 单臂 IK→IRIS→GCS 产出可行轨迹并被 Meshcat 呈现；仿真与假驱动共用同一 Planner API；规划结果回放仿真碰撞复检零逃逸。

- [x] **S03: 单臂真机执行与安全** `risk:high` `depends:[S02]`
  > After this: 真机上 5 ms 透传执行完整规划轨迹，末端跟踪误差 < 3 mm；安全七闸与四类故障注入全部进入安全态并恢复。

- [x] **S04: 双臂几何与标定** `risk:medium` `depends:[S03]`
  > After this: 两臂同 plant 同时运动不互撞，两臂 TCP 在共同工作区同一点偏差 < 0.5 mm。

- [x] **S05: 双臂紧协调规划** `risk:high` `depends:[S04]`
  > After this: 两臂保持末端相对位姿固定共同搬运物体，约束构造性满足，成功率 ≥ 95%、P95 < 3 s。

- [x] **S06: 任务原语与感知接入** `risk:medium` `depends:[S05]`
  > After this: 抓放/搬运/交接原语可组合跑通并达到成功率门槛；感知以非阻塞离散事件接入。

- [x] **S07: 工程化与产品化** `risk:low` `depends:[S06]`
  > After this: 全链路录制回放 + golden-file 回归通过；8 h 长稳达标；资产与许可锁定完成。

- [x] **S08: 验证契约与故障覆盖闭合** `risk:medium` `depends:[S01,S03]`
  > After this: python3 tools/verify_env.py --model --joint-limits /nonexistent/path.yaml 的退出码与「01-01-UAT.md:47」契约逐字一致；新增负例测试钉死该退出码语义；第五类故障「供电跌落」在替身传输层注入后进入安全态并恢复到 READY，正例 + 反例单测全绿；runbooks/s03_rm_device_bringup.md 含该类故障的真机复验步骤。

- [x] **S09: 证据闭合与需求口径校准** `risk:low` `depends:[S00,S02]`
  > After this: tools/fetch_upstream.py --verify 在归档物化后 exit 0（或书面豁免口径落定并同步 R019 Proof）；S02 的 runtime-executable 全量复跑逐条证据留存（pytest / verify_env --plan / CLI single→replay 零逃逸 / plan_stats 成功率与 GCS P95 / 禁用接口扫描 0 命中 / CI fast 口径）；FK 门槛偏差（官方 URDF rpy 舍入致 ~2.2407e-6 m 固有残差，断言放宽为 <5e-6 m）在计划/需求层显式接受；REQUIREMENTS.md 中 R007 / R012 / R014 的 Proof 从 unmapped 回填，R011 Proof 反映 task CLI 双臂场景缺口。

## Boundary Map

### S00 → S01

Produces:

- `versions.lock`：Drake 版本、RM_API2 wheel 版本、rm_models commit、Two-Finger-Gripper commit 的锁定记录
- `models/` 目录与 `package.xml` / `PackageMap` 约定（Drake 通过 `package://` 解析网格）
- 依赖锁文件（pip constraints）与离线 wheel 归档目录
- CI 三档定义与许可登记表骨架

Consumes:

- nothing (first slice)

### S01 → S02

Produces:

- `rm75b_drake.urdf` / `rm75b_drake.obj.urdf`：修正速度限位、snap 轴向量、capsule/box 碰撞几何、hydroelastic 属性、显式 weld 固定基座
- `rmg24_drake.urdf`：夹爪本体 + 两指 + 开合关节 + 碰撞几何 + `tcp` frame
- `gripper_mapping`：开口量 (mm) ↔ 0–1000 无量纲转换层与 u→N 标定曲线
- `model_regression` pytest 套件：惯量/FK/限位/质量/自碰撞过滤断言

Consumes:

- S00 的 `versions.lock` 与 `package://` 解析约定

### S01 → S03

Produces:

- 关节限位表（软件侧，比硬件再收 3–5°）与速度/加速度限值表
- 电流阈值基线（`rm_get_current_joint_current`, mA）

Consumes:

- S01 的模型与限位真值

### S02 → S03

Produces:

- `Planner` API：输入任务空间目标 + 场景描述，输出 5 ms 网格关节角序列 + 时间戳 + 夹爪动作调度表
- `Trajectory` 不变量：每点带单调 seq 与绝对时刻；重定时后逐轴速度/加速度满足真机限值
- `scenario.yaml` schema：机器人实例、基座外参、场景障碍、标定文件引用

Consumes:

- S01 的模型；S02 内建的 IRIS/GCS/Toppra 链路

### S02 → S04

Produces:

- `FakeDriver` 实现：与真机 `rm_device` 同接口，仿真与真机共用同一 Planner API

Consumes:

- S02 的 Planner API 与 scenario schema

### S02 → S05

Produces:

- 单臂凸区域缓存结构（IrisNp2/IrisZo 配置与种子约定）

Consumes:

- S02 的规划链路

### S03 → S04

Produces:

- `rm_device` 独占连接层：TCP 8080 指令 + UDP 5 ms 状态回调，每臂一个实例
- `rm_exec` 实时层：5 ms 采样器、双臂时间对齐、安全七闸、看门狗、状态 age 检查
- 状态机：DISCONNECTED → CONNECTED → WAIT_ENABLE → READY → STREAMING → HOLDING → FAULT → RECOVER → ESTOP
- 故障恢复流程：错误码清除、限位回位、断线后 q 重新同步
- 录制/回放格式（全链路消息，含时间戳）

Consumes:

- S01 的模型与限位；S02 的 Planner 输出契约

### S04 → S05

Produces:

- 双臂单一 `MultibodyPlant` scenario（两臂 + 两夹爪 + 场景）
- 标定文件：基座间外参 X_LR、关节零位偏差表、Link7→法兰→夹爪→TCP 6D 变换
- 双臂碰撞过滤矩阵（同臂相邻过滤、左右臂互检、夹爪指与被抓物体放行）

Consumes:

- S01 的模型；S03 的实时层与安全七闸；S04 自身的标定链

### S04 → S06

Produces:

- 任务空间目标表达：笛卡尔位姿 + 相对位姿约束（两臂）

Consumes:

- S04 的双臂 scenario 与标定文件

### S05 → S06

Produces:

- 最小坐标参数化接口：给定末端相对位姿约束，返回参数化空间的位形
- 参数化空间凸区域缓存（C++ 下沉版本）
- 协调规划 API：输出两臂共享时间轴的联合轨迹

Consumes:

- S05 的参数化与凸区域；S04 的双臂 plant

### S06 → S07

Produces:

- 任务状态机与原语库：approach / pre-grasp / close / verify / lift / transit / place / open / retreat / handover
- 原语签名：原子 goal_id、单一主状态源、单一求解器入口
- 感知适配接口：非阻塞订阅语义 + 保守场景表示（包围盒膨胀裕量）

Consumes:

- S06 的原语库；S05 的协调规划 API；S03 的实时层

### S03 → S07

Produces:

- golden-file 基线格式：规划结果回放仿真碰撞复检用例集
- 厂商解算接口零调用审计：CI 静态扫描 + 运行时调用计数

Consumes:

- S03 的录制/回放格式
<!-- gsd:state-version=789:1 -->
