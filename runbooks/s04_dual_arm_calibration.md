# S04 双臂几何与标定作业手册（RM75-D 双臂）

> **适用范围**：把「双臂几何（单 plant 装配 / 碰撞过滤 / 共同点一致性）」与
> 「标定链（关节零位 / 工具 TCP 6D / 双臂基座外参 X_LR）」从**离线可验证的
> 工具链**推进到**有实测数据的标定产出**的逐步作业手册。
>
> **诚实前提**：本仓库当前**没有真机、没有实测标定数据**。
> 因此所有真实物理指标——真实标定精度、真实共同点 < 0.5 mm、双臂同一 tick
> 下发时间差 < 1 ms、100 次双臂同空间无碰撞——**一律未验证**。
> 工具链在无实测数据时输出 `SKIP(reason=calibration_not_measured)` 并以
> **退出码 0** 结束，同时**逐条列出未验证项**：
> 绝不拿合成数据的残差冒充实测精度，也绝不把「没测」伪装成失败。

---

## 0. 真相源一览

| 内容 | 真相源 |
|---|---|
| 标定产出文件（部署实例） | `calibration/dual_arm.yaml`（schema：`src/rm75_drake/calibration/schema.py`） |
| 采样点集（演练 / 实测） | `calibration/samples/*.yaml`（schema：`src/rm75_drake/calibration/samples.py`） |
| 场景语义 | `scenarios/dual_arm.yaml`（**引用**标定文件，不内联外参） |
| 标定求解器（纯 numpy） | `src/rm75_drake/calibration/solvers.py` |
| 作业编排（CLI 与脚本共用） | `src/rm75_drake/calibration/pipeline.py` |
| CLI 入口 | `python3 -m rm75_drake.cli calibrate` |
| 脚本入口（自引导 `sys.path`） | `python3 scripts/calibrate_dual_arm.py` |
| 双臂 URDF 变体 | `models/rm75_drake/{rm75,rmg24}_{left,right}.urdf`（生成器 `tools/gen_dual_arm_models.py`） |
| 双臂单 plant 装配器 | `src/rm75_drake/bimanual/dual_scene.py` |
| 碰撞过滤矩阵 | `src/rm75_drake/bimanual/collision_filter.py` |
| 共同点一致性判定器 | `src/rm75_drake/bimanual/consistency.py` |
| 双臂同一 tick 同步执行器 | `src/rm75_drake/bimanual/sync_exec.py` |

**唯一真相源纪律**：双臂基座外参**只有** `calibration/dual_arm.yaml` 一处；
场景通过 `calibration.base_extrinsic.file` 引用它。标定文件缺失 / 非法 /
机器人不匹配 ⇒ `ScenarioError` **点名路径与出错键**，**禁止**静默回退到
场景里手写的外参（那会形成两个真相源）。

---

## 1. 离线自证（零硬件，先跑通链路）

```bash
# 1) 解算演练（合成采样 + 未实测标定文件）⇒ SKIP + 逐条未验证 + 退出码 0
PYTHONPATH=src python3 -m rm75_drake.cli calibrate --json

# 2) 显式接受合成数据（仅演练；结论仍**不构成实测精度**）⇒ status=ok + 残差
PYTHONPATH=src python3 -m rm75_drake.cli calibrate --json --accept-synthetic

# 3) 脚本入口（同一份结论，自引导 sys.path）
python3 scripts/calibrate_dual_arm.py --json

# 4) 常驻回归（CI fast 档恒跑，失败即整体失败）
bash scripts/ci.sh fast
```

退出码：`0` 成功 / SKIP；`1` 业务失败（残差超阈值、数据非法）；`2` 用法错误
（路径缺失 / 参数非法）。

---

## 2. 实测采样作业（尖对尖 15–20 组）

### 2.1 关节零位（逐关节 +10° 对比）

1. 把机械臂置于**出厂零位**（或记录编码器读数），记录该关节 `q_nominal`；
2. 该关节**单独** +10°（= 0.17453292519943295 rad），记录 `q_offset`；
3. 两条位形都用**同一手段**观测末端位姿（机器人自身 FK / 外部测量），
   填 `pose_nominal` / `pose_offset`；
4. 逐关节重复（RM75-D 是 7 自由度）。

> ⚠️ **可辨识性**：若 FK 对某关节不敏感（例如腕部三关节只在特定姿态下可辨识），
> 求解器会显式报「**不可辨识**」，并要求改用**编码器读数或外部测量**。
> 这不是 bug，是物理事实——**不得**用「随便给个数」填平。

### 2.2 工具 TCP 6D（四点法 / 六点法）

* 在夹爪上固定标定针（TCP 标定件），采 **≥ 4 组**（推荐 6 组）「局部点 ↔ 世界点」；
* 点必须**非退化**（不能共线 / 共面且对称），否则 Kabsch 解不可信；
* 局部点 = TCP 相对夹爪法兰的坐标；世界点 = 外部测量（激光跟踪仪 / 相机）。

### 2.3 双臂基座外参 `X_LR`（尖对尖，15–20 组）

1. 两臂各装一支尖对尖标定针；
2. 让两针**尖端重合**（接触或光学对准），依次采 **15–20 组**：
   左针尖在**左臂基座系**的坐标 `left_point` + 右针尖在**右臂基座系**的坐标
   `right_point`；
3. 点组必须覆盖**不同姿态**（绕基座转几圈），否则解退化；
4. 求解 `X_LR`（右臂基座系 → 左臂基座系），并做**第三方交叉验证**。

### 2.4 采样文件格式

每条采样**必须**带 `synthetic`（true/false）与 `source`（来源说明）。
实测数据写 `synthetic: false`，`source` 写清测量手段与日期。
未知键 / 缺字段 / 维度错 ⇒ `CalibrationError`（**点名该键**）。

---

## 3. 解算与第三方交叉验证

```bash
# 用实测采样解算（--accept-synthetic 只影响「合成数据」的判定，不影响实测数据）
PYTHONPATH=src python3 -m rm75_drake.cli calibrate \
    --samples calibration/samples/dual_arm_2026-11-13.yaml \
    --calibration calibration/dual_arm.yaml \
    --out /tmp/calib_result.json
```

摘要里看三件事：

| 字段 | 含义 |
|---|---|
| `residuals.<段>.rms` / `.max` | 逐点残差的 RMS / 最大值（单位 m） |
| `residuals.<段>.within_threshold` | 是否在阈值内（超阈值 ⇒ 整体 `failed` + 退出码 1） |
| `sections.<段>.status` | `ok` / `partial`（部分关节不可辨识）/ `failed` / `absent`（无采样） |
| `repeatability` | 留一折重解的离散度（同一批数据的自洽性） |
| `cross_validation` | 连续两半数据各解一次的差异（**自洽性**，**不是**第三方精度） |

**第三方交叉验证**（相机 / 激光跟踪仪）需要**另一套独立解**：
把第三方的解写进同一份标定文件的不同 `method`，或离线比对两份解的差值；
本工具的 `cross_validation` 段**只反映同一批数据的两半自洽性**，
**不得**被引用为第三方精度结论。

**重复性**：同一工装、同一批姿态**重复标定 ≥ 3 次**，比较三次解的离散度；
该结论需写进标定文件的 `method` / `date` 备注。

---

## 4. 写回 `calibration/dual_arm.yaml`

把解出的数值写回标定产出文件，并**同时**把元信息改对：

```yaml
schema_version: 1
measured: true        # 实测数据才允许 true
calibrated: true      # measured=false 时必须 false（schema 强制自洽）
method: "tip-to-tip(15-20)+Kabsch+third-party-cross-check"
date: "2026-11-13"    # YYYY-MM-DD
base_extrinsic:
  left:  {xyz: [...], rpy: [...], residual: {rms: ..., max: ..., samples: 15}}
  right: {xyz: [...], rpy: [...], residual: {rms: ..., max: ..., samples: 15}}
joint_zero_offset:
  left:  {left_joint_1: 0.0, ...}   # 每臂恰好 7 个关节
  right: {right_joint_1: 0.0, ...}
tool_offset:
  left:  {xyz: [...], rpy: [...], residual: {...}}
  right: {xyz: [...], rpy: [...], residual: {...}}
```

纪律：

* **长度一律 m、角度一律 rad**（schema 会拒绝角度制并给出换算提示）；
* `residual` 必须填**真实**的 rms / max / samples（`samples: 0` = 没有实测样本）；
* 标定文件是**唯一**真相源：不要把外参再抄进 `scenarios/dual_arm.yaml`，
  场景只写 `calibration: {base_extrinsic: {file: dual_arm.yaml}}`。

---

## 5. 场景消费与几何自检

```bash
# 场景加载后外参必须等于标定文件（唯一真相源生效）
PYTHONPATH=src python3 -c "
from rm75_drake.scenario import load_scenario
s = load_scenario('scenarios/dual_arm.yaml')
print(s.calibration['base_extrinsic']['file'])
print(s.base_extrinsics)
"
```

---

## 6. 双臂同一 tick 下发观测

```bash
# 离线（零硬件）：双臂同一拍内按固定次序（默认 left→right）依次下发，
# 记录两臂下发时间差（ns + 直方图）
python3 -m pytest -q tests/test_bimanual_sync_exec.py
```

真机上用 `src/rm75_drake/bimanual/sync_exec.py` 驱动两臂，读取其
**下发时间差**指标；口径与 S03 的 `device/exec_loop.py` / `device/metrics.py`
一致（同一拍、同一时钟口径、同样的直方图字段）。

---

## 7. 当前**未验证**的物理指标与解锁条件

| 未验证指标 | 目标 | 解锁条件 | 当前状态 |
|---|---|---|---|
| 真实标定精度：关节零位偏差 | < 0.1° | §2.1 实测采样 + §3 解算（FK 不敏感的关节需编码器读数） | ❌ 未验证（无实测数据） |
| 真实标定精度：工具 TCP 6D | < 0.5 mm | §2.2 四点/六点法实测 + 第三方交叉验证 | ❌ 未验证 |
| 真实标定精度：双臂基座外参 `X_LR` | < 0.5 mm / 0.2° | §2.3 尖对尖 15–20 组 + 第三方交叉验证 | ❌ 未验证 |
| 双臂共同工作区 TCP 同一点偏差 | < 0.5 mm | 上三条达成 + `bimanual/consistency.py` 在真实标定下判定 | ❌ 未验证（`measured=false` ⇒ 判定恒为 `unverified`） |
| 双臂同一 tick 两臂下发时间差 | < 1 ms | 真机 + `sync_exec.py` 的下发时间差直方图 | ❌ 未验证（无真机） |
| 100 次双臂同空间运动无碰撞 | 0 次碰撞 | 真机 / 高保真仿真里跑 100 组同空间轨迹 | ❌ 未验证（无真机） |
| 标定重复性 | 同一工装重复标定的离散度 | 同一批姿态重复标定 ≥ 3 次 | ❌ 未验证 |

> **红线**：以上任何一项在**没有对应实测数据**时，工具链一律报
> `status=skipped` / `reason=calibration_not_measured`（或一致性判定器的
> `unverified`），**不得**在任何报告、演示或提交说明里写成「达标」。

---

## 8. 常见故障速查

| 现象 | 原因 | 处理 |
|---|---|---|
| `CalibrationError: … 不可辨识` | FK 对该关节不敏感 | 换编码器读数或外部测量手段；不要硬填 |
| `CalibrationError: … 残差超阈值` | 采样质量差 / 工装松动 / 点集退化 | 复查采样点分布；确需放宽阈值前先确认数据可信 |
| `CalibrationError: 出现未知顶层键` | 采样或标定文件多了键 | 按报错点名的键删改（schema 严格白名单） |
| `ScenarioError: 标定文件非法 …` | 场景引用的标定文件有问题 | 按消息里的路径与出错键修；**不要**改场景去绕过 |
| `status=skipped` | 无实测数据（**预期行为**） | 想看残差就加 `--accept-synthetic`（结论仍非实测） |
| `exit code 2` | 用法错误（路径缺失 / 参数非法） | 核对 `--samples` / `--calibration` 路径 |
