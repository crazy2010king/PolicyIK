# 真机硬件卡：睿尔曼 RM75-B + WHJ03 两指夹爪

> **状态**：**现场在役**（不是模拟器）。本卡是「这台具体设备」的环境契约，
> 与仓库内其它文档的分工：
> - 本卡 = **这台设备是什么、怎么连、怎么用、哪些红线不能碰**；
> - `docs/validation/RM75-B_真机验证阶梯.md` = **验证阶梯怎么走**（L0–L14）；
> - `docs/evidence/rm75-b-real-arm-ladder.md` = **已经测出什么数字**；
> - `runbooks/s03_rm_device_bringup.md` = **设备层怎么接**（bring-up 手册）；
> - `devices/rm75_b_192_168_1_18.yaml` = **部署参数与标定值的唯一真相源**。
>
> ⚠️ 本卡里的每个数字都是**真机实测**（2026-11-19 / 2026-11-20），不是厂商文档抄录。
> 设备变更（换机、换夹爪、改装末端工具）后必须**重新标定**并更新本卡。

---

## 1. 硬件身份

| 项 | 值 | 来源 |
|---|---|---|
| 机械臂 | 睿尔曼 **RM75-B**（7-DoF 仿人冗余构型） | 厂商 |
| 夹爪 | 睿尔曼 **WHJ03** 两指平行夹爪（65 mm 开口，RS485/Modbus RTU） | 厂商 |
| 机械臂 IP | `192.168.1.18` | 厂商出厂默认（网页/指令通道同址） |
| 指令通道 | TCP `192.168.1.18:8080`（`rm_*` SDK） | 实测 |
| 状态主动上报 | UDP → **开发机** `192.168.1.198:8080`（目标 IP 填**开发机**，不是机械臂） | 实测 |
| 厂商示教器网页 | `http://192.168.1.18/`（HTTP 200） | 实测 |
| 控制器代次 | **3**（`rm_get_robot_info().robot_controller_version`） | 实测 |
| 自由度 | `arm_dof = 7` | 实测 |
| SDK | `robotic-arm==1.1.7`（**导入名 `Robotic_Arm`，大写 R/A**；分发名小写连字符） | 实测 |
| 夹爪在线语义（两套，**相反**） | `rm_get_gripper_state` 的 `status`：**1 = 在线**；UDP 帧的 `rm_plus_state`：**0 = 在线** | 实测 |

## 2. 夹爪（用户重点）

| 项 | 实测 |
|---|---|
| 张开 | 指令 1000 → 回读 `actpos` **1000**（误差 0） |
| 闭合 | 指令 0 → 回读 `actpos` **12**（误差 12；空夹时也会读到 -1） |
| 在线 | `status = 1`（在线）；供电与协议正常，设计文档 §八.3 的四坑（末端电压 / 协议 / 供电 / 0–1000 换算）**均未出现** |
| 位置量纲 | **0–1000 无量纲**（不是 mm、不是角度）；65 mm 开口 ⇒ 开口量 = `u/1000 × 65 mm` |
| 故障判据 | 位置回读 ≠ 目标 ⇒ `gripper_not_at_target`（疑似夹空）——**真机已验证整条链**：注入 → 检测 → `holding` → 恢复 → `ready` |

夹爪动作由 `rm_set_gripper_position(position, block, timeout)` 下发（**三参数**，不是单参数）。

## 3. 实测事实（踩过的坑都在这）

| # | 事实 | 后果 / 纪律 |
|---|---|---|
| 1 | `RM_TRIPLE_MODE_E = **2**`（`RM_SINGLE=0 / RM_DUAL=1 / RM_TRIPLE=2`） | 传 3 ⇒ SDK 打印 `Invalid thread mode` 且拿不到句柄 |
| 2 | 必须显式 `rm_create_robot_arm(ip, port)` 才会建连接 | 只 `RoboticArm(...)` ⇒ 根本没有链路 |
| 3 | 厂商 API 是**实例方法**（`arm.rm_movej_canfd(...)`），模块级同名符号是 ctypes 原函数、签名不同 | 直接调模块级函数必失败 |
| 4 | 逐关节接口（使能/清错/限位）是**逐轴**调用，且 **`joint_num` 是 1-based（1~7）** | 传 0 ⇒ 控制器返回 1（失败）；按 0-based 调 ⇒ 恢复流程**打错轴**（假绿灯） |
| 5 | 控制器拒绝请求时**返回 1 且不抛异常**（SDK 打印 `...: false`） | 必须检查返回码，否则失败被当成功 |
| 6 | `cycle` 单位 = **5 ms 的倍数**（`cycle=1` ⇒ 实测 P50 **4.998 ms**；`cycle=5` ⇒ 24.954 ms） | 设计文档「cycle=5 = 5 ms」的读法被实测**否证** |
| 7 | 控制器出厂关节软限位（实测）：J1 **±125.998°**、J2 ±130.0°、J3 **±130.988°**、J4 ±135.0°、J5 **±123.998°**、J6 ±128.0°、J7 ±360.0° | **J1/J3/J5/J6 比模型软限位更紧** ⇒ 有效限位必须取**交集**（`controller_joint_limits` 段） |
| 8 | **软件急停无效**：`rm_set_arm_emergency_stop(True)` 返回 **-4**（控制器超时拒绝） | 急停必须用**物理急停**；且一次被拒调用会让同连接的配置通道**不再返回** ⇒ SDK 调用必须带超时 |
| 9 | 同一 IP+port **只能一个进程**占用 | 并发第二个控制进程 ⇒ 前一个 `rm_movej_canfd` 立刻 `-2`（设备层已加按 IP 的 `flock`） |
| 10 | 控制器在轨迹起步瞬间有**单拍电流尖峰**（J1 实测 -900 / -2802 mA，个别采样 10 A 量级） | G5 电流闸已改「**连续 3 拍**越限才停机」 |
| 11 | 静态重力保持电流 **J2 ≈ -2100 mA**（另一时刻 -1604） | 兜底阈值 1500 mA 必然误停机 ⇒ 已实测标定回写 |
| 12 | 末端跟踪误差与**加速度上限**强相关 | 6.0/8.0 ⇒ 6.51 mm；2.0/2.5 ⇒ 3.73 mm；**1.0/1.2 ⇒ 2.66 mm**（≤ 3 mm 达标） |
| 13 | 上一条轨迹未停稳就下发新轨迹 ⇒ **控制器忽略新指令**，指令-反馈错位 30 mm | `cli exec` 增加停稳等待 + 起点一致性守卫（容差 2°） |

## 4. 设备配置（部署真相源）

`devices/rm75_b_192_168_1_18.yaml`：

| 段 | 内容 | 状态 |
|---|---|---|
| `arm_ip` / `tcp_port` / `udp_port` / `udp_cycle` | `192.168.1.18` / 8080 / 8080 / **1** | 实测 |
| `accel_limits` | J1/J2 **1.0**、J3–J7 **1.2** rad/s² | `measured: true`（末端跟踪误差扫测得出） |
| `current_thresholds` | J1 3200 / J2 3154.5 / J3 1576.5 / J4 2005.5 / J5 600 / J6 681 / J7 600 mA | `calibrated: true`（静态 ∪ 动态去抖包络 ×1.5） |
| `controller_joint_limits` | 实测控制器出厂限位（度） | `measured: true` |
| `fence` | `table_height_m` / `singularity_radius_m` | ⚠️ **仍是占位值**（实测 TCP 高度余量仅 ~4 mm） |

## 5. 可用工具（真机入口）

| 目的 | 命令 |
|---|---|
| 阶梯（只读级别） | `python3 scripts/real_machine_ladder.py --to L5 --device-config devices/rm75_b_192_168_1_18.yaml --json` |
| 阶梯（运动级别） | `… --allow-motion --joint-rad "<7 个 rad>"`（**必须**显式给位姿） |
| 夹爪 / 七闸 / 只读 | `… --stage L9 --allow-motion`、`… --stage L10` |
| 轨迹执行 | `python3 -m rm75_drake.cli exec --device-config devices/rm75_b_192_168_1_18.yaml --recording <plan.json> --log-out <log.json>` |
| 跟踪误差 | `python3 scripts/eval_tracking.py --run-log <log.json> --json` |
| 电流标定 | `python3 scripts/calibrate_currents.py --device-config devices/rm75_b_192_168_1_18.yaml --duration-s 6 --allow-motion --jog-deg 5 --recording <plan.json> --write --json` |
| 故障链（可全自动） | `python3 scripts/fault_chain_test.py --device-config … --case gripper_not_at_target --json` |
| 故障注入（需人手） | `python3 scripts/fault_inject.py --hardware --json`（逐步提示并等回车） |
| 长稳 | `python3 scripts/soak_test.py --device-config … --joint-rad "…" --hz 100 --duration-s 1800 --json` |

## 6. 安全红线（不可协商）

1. **物理急停**必须在手边；软件急停在这台设备上**不可用**（返回 -4）。
2. 会动真机的入口**必须**显式 `--allow-motion`，且**必须**显式给出目标位姿（脚本绝不自己编造下发目标）。
3. 同一时刻**只允许一个**真机程序（设备层已用 `flock` 强制）。
4. 任何会动真机的操作前，**先跑 L0**（TCP + 网页）确认链路——上一轮网卡掉线就是靠它 1 秒识别的。
5. 写控制器限位必须「**先读 → 改 → 写 → 读回校验**」四步，且 `joint_num` 用 1-based。
6. 围栏参数（桌面高度 / 奇异圆柱）**标定前不要做大范围运动**。

## 7. 运行环境（真机侧）

| 项 | 值 |
|---|---|
| 工作站 | `172.16.119.199:2224`（用户 `zy`，SSH 密钥免密），主机名 `zy-Legion-Y9000P-IRX9`，**Ubuntu 24.04.3 LTS** |
| 到机械臂网段的网卡 | `enx98fc84e3177a`（**USB 网卡**），`192.168.1.198/24` —— ⚠️ 曾出现 NO-CARRIER（物理掉线） |
| 容器 | `ros_dev`，镜像 `172.16.119.20:5000/rwthika/ros2-cuda:humble-desktop-full-v25.08_conda_sonnic_20261110_1124`，**网络模式 host**（容器与宿主共享网络 ⇒ 机械臂可达性取决于宿主网卡） |
| 规划环境（容器） | `conda activate drake_dev` ⇒ Python 3.12.13 + **Drake 1.55.0** + `toppra 0.6.11` |
| 执行环境（宿主） | 独立 venv（如 `/tmp/rmvenv`）⇒ Python 3.12.3 + `robotic-arm 1.1.7`（**设备层不需要 Drake**） |
| 代码位置（远端） | `/home/zy/nfs/test_cc_dev/git_source/groot_nvidia/PolicyIK_drake/.gsd-worktrees/M001-acm1jg`（本地改动经 NFS 同一份文件） |

> **规划与执行分离**：Drake 规划在容器里做（需要 Drake + toppra），
> 真机执行在宿主 venv 里做（只要 SDK）——因为容器虽是 host 网络，
> 但 **UDP 上报目标 IP 必须填宿主在 192.168.1.x 上的地址**。

## 8. 已验证 / 未验证（截至 2026-11-20）

| 已验证 ✅ | 未验证 ❌（**不得当作通过**） |
|---|---|
| L0–L10 全阶梯（PASS=11/FAIL=0）、L11 夹爪链、L12 30 min 长稳、L14 golden、末端跟踪误差 2.66 mm、电流/加速度/控制器限位标定 | L11 其余四类物理故障（需人手）、**急停态（软件急停无效）**、L13 双臂（无第二台臂）、8 h 长稳、末端误差外部测量复核、围栏参数现场标定 |
