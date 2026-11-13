# rm_executor 离线证据（S03 / T8）

> **口径声明（先读这一段）**
>
> 本文件记录的是 **离线代理证据**：它们证明「契约成立、时序成立、异常判定
> 路径成立」，**不是真机结论**。
>
> S03 的三条真机验收——① 真机连续 10 分钟无保护停；② **真机** tick 抖动
> p99 < 2 ms；③ **真机**到位判定误差 < 0.1°——**在本轮未实证**（真机为
> 预约资源，本轮不可达）。真机窗口用**同一条代码路径**
> （`python3 -m rm_executor.runtime`，与离线 fake **同一个 Executor、同一份
> 报告 schema**）回填数字，回填后本文件追加「真机窗口回填」小节。
>
> 离线长跑里的抖动 / 跟随误差是**虚拟时钟 + 理想跟随模型**的代理值，报告
> `evidence` 里也固定带这句提示，请勿当成真机指标引用。

---

## 0. 环境准备（复跑前必做）

| 步骤 | 命令 | 说明 |
|------|------|------|
| 0.1 | `source scripts/rmenv.sh` | 注入 `PYTHONPATH`（rmctl / rm_hw_bridge / rm_executor）与 `LD_LIBRARY_PATH` |
| 0.2 | 把 conda-forge 的 `libstdc++`（**需导出 GLIBCXX_3.4.31+**）放到 `<repo>/.deps/libstdcxx/` | 厂商 nanobind wheel（`tesseract_robotics`）需要它；`rmenv.sh` 会自动优先取本 checkout 的 `.deps/libstdcxx`，缺失时只读引用同仓库其它 checkout |
| 0.3 | `python3 -m rmctl.cli init` | 物化 `build/model/rm75_clean.urdf`（**绝对路径 mesh**）；Tesseract 的资源定位器不认相对路径，缺这一步 legacy 路径会报 `Failed to parse URDF` |

> 0.2 / 0.3 产出物都在 **gitignore** 目录（`.deps/`、`build/`）内，是**每台机器的环境准备**，不是仓库内容。

---

## 1. 全量回归（三包合跑）

```bash
source scripts/rmenv.sh
python3 -m pytest dualarm_ws/src/rmctl/tests dualarm_ws/src/rm_hw_bridge/tests \
    dualarm_ws/src/rm_executor/tests -q
```

| 项 | 数值 |
|----|------|
| 退出码 | **0** |
| 结果 | **540 passed, 1 skipped, 0 failed**（共 541 条） |
| 基线对照 | S02 三包基线 344 条 → 本轮 541 条（净增 **197** 条：rm_executor 契约/时钟/传输/反馈/执行/事件/入口/接线/**注入/样本轨迹**测试） |
| 证据 | `.gsd/exec/1ca00f83-b49c-43cf-8e59-4337e61b5e55.stdout`（最新一轮，退出码 0）、`.gsd/exec/4a8d253d-c081-477c-8e9a-e29c3c93d22c.stdout`（首轮） |

> **复跑命令（不依赖 `source scripts/rmenv.sh`）**：主机验证门禁用的是**裸环境**
> （只有 `PYTHONPATH`，没有 `LD_LIBRARY_PATH`）。本轮已把厂商动态库兜底做进代码
> （见 §1.1），因此下面这条命令在裸 shell 里也能 exit 0：
>
> ```bash
> PYTHONPATH=dualarm_ws/src/rmctl:dualarm_ws/src/rm_hw_bridge:dualarm_ws/src/rm_executor \
>     python3 -m pytest dualarm_ws/src/rmctl/tests dualarm_ws/src/rm_hw_bridge/tests \
>     dualarm_ws/src/rm_executor/tests -q
> ```

### 1.1 本轮修掉的环境级缺陷（裸环境下 rmctl legacy 路径 ImportError）

**现象**：主机验证门禁（裸环境，只注入 `PYTHONPATH`）跑全量回归时，
`rmctl/tests/test_run_executor_wiring.py::test_run_default_is_marked_legacy` 失败：

```
ImportError: /usr/lib/x86_64-linux-gnu/libstdc++.so.6: version `GLIBCXX_3.4.31' not found
  (required by .../tesseract_robotics/.../libboost_filesystem.so.1.88.0)
```

**根因**（两层）：

1. 厂商 nanobind wheel 依赖 `GLIBCXX_3.4.31+`，本机系统 `libstdc++` 只有 3.4.30；
   仓库自带 `.deps/libstdcxx`（`scripts/rmenv.sh` 的同一来源）能补上，但此前
   **只有 `source scripts/rmenv.sh` 才生效**（靠 `LD_LIBRARY_PATH`），裸环境失效。
2. 即便用 `ctypes` 预加载，也**必须在 numpy 之前**：numpy 会先拉起系统
   `libstdc++.so.6`（soname 相同），此后厂商 wheel 的版本检查仍落在旧库上。
   证据：`import numpy` → 预加载 → `import tesseract_robotics` **失败**；
   预加载 → `import numpy` → `import tesseract_robotics` **成功**。

**修法**（代码级，零环境假设）：

* 新增 `dualarm_ws/src/rmctl/rmctl/vendor_env.py`：按 `.deps/libstdcxx` →
  `../.deps/libstdcxx` → `../../.deps/libstdcxx` 顺序**只读**引用，用
  `ctypes.CDLL(..., mode=RTLD_GLOBAL)` 预加载；
* 在 `rmctl/__init__.py`（**任何 numpy / 厂商库 import 之前**）、`planner.py`、
  `verify.py`、`viewer.py`、`model.py` 各调一次；`rmctl/tests/conftest.py` 再兜一次底
  （pytest 会先于任何测试模块加载 conftest）；
* 预加载失败**不吞异常**：`Model` 抛 `rmctl.vendor_env.VendorUnavailable`，
  `rmctl run`（legacy 路径）转成**诚实降级** —— 退出码 **3** + reason + 可粘贴
  next_action（`source scripts/rmenv.sh`），**不伪造回放统计**；
* 对应测试改为断言两条**契约**分支：`rc == 0`（厂商内核可用，完成回放）或
  `rc == 3`（不可用 → 降级且带 next_action），**两条分支都不许抛裸异常**。

**修后**：全量回归 **540 passed, 1 skipped**（退出码 0）。

**本轮修掉的一条真实缺陷（三包合跑才暴露）**：
`rm_executor/tests/test_hardware_wiring.py::test_executor_does_not_import_rmctl_or_tesseract`
原先在**当前 pytest 进程**里断言 `"rmctl" not in sys.modules`。三包合跑时 rmctl
的用例已经把 `rmctl` import 进同一会话 → **假失败**（判的是会话状态，不是执行层
依赖）。已改为**干净子进程**里 import `rm_executor` 后再查 `sys.modules`，并另加一条
**静态扫描**用例（执行层源码不得出现 `import rmctl` / `import tesseract`），
判据变强且不再受会话污染。

---

## 2. 厂商解耦红线审计

```bash
python3 -m rm_hw_bridge.audit --repo
```

| 项 | 数值 |
|----|------|
| 退出码 | **0** |
| 输出 | `审计通过：0 项违规`（含新增 executor 层） |
| 证据 | `.gsd/exec/924785f6-3d23-4abb-ba60-eefe0aeb1345.stdout`（最新一轮）、`.gsd/exec/c8347e3b-4132-4603-9539-afa042efe126.stdout`（首轮） |

---

## 3. ABI 生成物幂等

```bash
python3 -m rm_hw_bridge.codegen \
  --spec dualarm_ws/src/rm_hw_bridge/rm_hw_bridge/abi_spec.yaml \
  --out dualarm_ws/src/rm_hw_bridge/rm_hw_bridge/generated --check
```

| 项 | 数值 |
|----|------|
| 退出码 | **0** |
| 输出 | `[codegen] 生成物与 abi_spec.yaml 一致（幂等）`（S03 未改 ABI 规格） |
| 证据 | `.gsd/exec/090e6722-ba11-451f-9881-3361cc27c2da.stdout`（最新一轮）、`.gsd/exec/bec2efd9-9ba6-4423-acbd-c08f0309e933.stdout`（首轮） |

---

## 4. 离线 600 s 等效长跑

### 4.1 轨迹来源（可复跑）

规划内核 `rmctl plan` 产出真机轨迹；离线环境里我们用**仓库内的确定性最小轨迹**
（7 关节小幅平滑运动，41 点 × 0.05 s = 2.0 s），生成器在执行层包内：

```bash
python3 -m rm_executor.sample --out build/reports/s03_sample_traj.json
```

* 生成器：`dualarm_ws/src/rm_executor/rm_executor/sample.py`（闭式公式 → **同样参数
  必然同样字节**，证据可复跑）；
* 该轨迹**过执行层四道校验**：契约校验（`group/joint_names/units/schema_version/
  times/points`）、`verify.ok`、时间域限值复检（位置 / 速度 / 加速度 / jerk，
  唯一数据源 `rm_hw_bridge.limits`）；
* **为什么是「平滑」而不是带拐点的折线**：限值复检用中心差分逐阶求导，
  **分段线性的拐点会产生真实的加速度 / jerk 尖峰**（按越界处理），折线会被
  如实判越界；
* **诚实声明**：这是**合成**轨迹，不是规划器产物；`verify.ok=true` 只表示「满足执行层
  轨迹契约」，时间域限值由执行器阶段 A 独立复检。

### 4.2 长跑命令与实测数字

```bash
python3 -m rm_executor.cli --traj build/reports/s03_sample_traj.json \
    --mode fake --duration 600 --report build/reports/s03_offline_600s.json
```

| 项 | 数值 |
|----|------|
| 退出码 | **0** |
| 报告 | `build/reports/s03_offline_600s.json` |
| 轮次 | `legs: 300`（300 轮 × 2.0 s 轨迹 = 600 s 计划时长；每轮都是完整「预就位 → 流式 → 到位」，不靠延展轨迹绕过限值） |
| `ticks` | **60300** |
| `planned_duration_s` | **600.0** |
| `duration_s` | **756.0**（含每轮预就位 200 ms + 末点保持 + 到位保持 300 ms 的**等效**执行时长） |
| `rt_factor` | **1.26** |
| `tick_jitter_p50_ms` | **0.0** |
| `tick_jitter_p95_ms` | **0.0** |
| **`tick_jitter_p99_ms`** | **0.0**（红线 < 2.0 → **通过**；虚拟时钟下抖动恒为 0，是**代理值**） |
| `tick_jitter_max_ms` | **0.0** |
| `follow_err_rms_deg` | **0.0**（理想跟随模型 → 代理值） |
| `follow_err_max_deg` | **0.0** |
| `arrival_err_deg` | **0.0** |
| `arrival_hold_ms` | **310.0** |
| `pre_position_hold_ms` | **200.0** |
| `hold_ticks` | **9600** |
| `events` | **900**（每轮 3 条 INFO：pre_position_ok / stream_done / arrival_ok） |
| 墙钟耗时 | **≈ 21 s**（60 300 拍虚拟执行） |
| 证据 | `.gsd/exec/4eb0b9cb-7b41-4b97-9212-be5e9fe74778.stdout`（最新一轮长跑）、`.gsd/exec/71342199-28da-490f-8181-ae7907bfb3c7.stdout`（指标断言）、`.gsd/exec/6d5ff4e7-a54c-4a39-a2cb-b544bdc958dc.stdout`、`.gsd/exec/2727e28a-a4c3-4480-a411-063ac2372825.stdout`（首轮） |

**断言**：`tick_jitter_p99_ms < 2.0` → `ASSERT OK`（证据文件 `71342199-*.stdout`）。

> **为什么 fake 下的抖动恒为 0**：离线用确定性虚拟时钟（`ManualClock`），栅格是
> **绝对截止时间**（第 k tick = 起点 + k×10 ms），因此没有真实调度抖动可测。
> 这条数字证明的是「**栅格与截止时间计算正确**」，**不能**替代真机的 p99 < 2 ms。

---

## 5. 三类故障注入复跑

注入入口：执行层 CLI 的 `--inject`（**只对离线 fake 开放**，真机路径拒绝任何注入）。
注入从**流式阶段（阶段 C）**起生效——预就位阶段必须正常到位，否则会退化成
「预就位超时」而不是被注入的那条判据。

| 注入 | 事件 | 严重度 | 安全动作 | 报告 status | **进程退出码** |
|------|------|--------|----------|-------------|----------------|
| `--inject follow_error:6` | `follow_error_exceeded`（跟随误差 6.000° > 5.0°） | NEEDS_HUMAN | `transport.pause()` ×1（**不停机**） | `blocked` | **3** |
| `--inject feedback_stall:250` | `feedback_stream_lag` → **`feedback_stream_lost`**（距最后一帧 **210 ms** > 200 ms 停机阈值） | RECOVERABLE → **FATAL** | `transport.stop()` | `error` | **4** |
| `--inject controller_conflict` | **`controller_trajectory_conflict`**（观测类型 `RM_PLANNING_A` ≠ 期望 `RM_NO_PLANNING_E`） | **FATAL** | `transport.stop()`（看门狗线程内立即停机） | `error` | **4** |

复跑命令（每条一份独立报告）：

```bash
python3 -m rm_executor.cli --traj build/reports/s03_sample_traj.json --mode fake \
    --inject follow_error:6      --report build/reports/s03_inject_follow_error_6.json
python3 -m rm_executor.cli --traj build/reports/s03_sample_traj.json --mode fake \
    --inject feedback_stall:250  --report build/reports/s03_inject_feedback_stall_250.json
python3 -m rm_executor.cli --traj build/reports/s03_sample_traj.json --mode fake \
    --inject controller_conflict --report build/reports/s03_inject_controller_conflict.json
```

证据（最新一轮，三条注入各自的**进程退出码**即命令退出码）：

| 注入 | 命令退出码 | 证据 |
|------|-----------|------|
| `follow_error:6` | **3** | `.gsd/exec/db3fb1f0-ccb8-47a8-8522-8d119dab25a3.stdout` |
| `feedback_stall:250` | **4** | `.gsd/exec/2f030eb9-e110-4bf1-bad5-cad5eb1d5e77.stdout` |
| `controller_conflict` | **4** | `.gsd/exec/b2c52445-e840-497e-8469-e4274717e68d.stdout` |
| （首轮三条合并跑） | — | `.gsd/exec/03945573-2b48-4688-848b-39cbb030bbae.stdout` |

报告里的事件类型（从报告 JSON 读回核对）：
`follow_error_exceeded`(NEEDS_HUMAN) / `feedback_stream_lag`(RECOVERABLE) →
`feedback_stream_lost`(FATAL) / `controller_trajectory_conflict`(FATAL)。

**口径说明（诚实标注）**

* `feedback_stall:250` 的**注入目标**是断流 250 ms，执行层的判据是
  `STREAM_STOP_MS = 200 ms` 硬红线 → 实测在 **age = 210 ms** 处即判 FATAL 并停机。
  **早于目标**是**安全方向**（更早停机），不是漏检。
* 注入参数在**解析阶段**就被校验：`follow_error` 必须 > 停机阈值 5°、
  `feedback_stall` 必须 > 断流停机阈值 200 ms，否则执行层按设计**不会**产生那条
  事件（写一个「看起来跑了但没事件」的报告是自欺）。
* 三条注入得到的事件与停机动作都是**离线代理结论**，不代表真机行为。

---

## 6. 真机窗口回填（本轮未实证）

真机窗口用**同一条代码路径**回填数字：

```bash
python3 -m rm_executor.runtime --traj <真机轨迹.json> --ip <控制器IP> --port <TCP端口> \
    --report build/reports/s03_live_10min.json
```

| S03 真机验收 | 本轮状态 | 回填方式 |
|---------------|----------|----------|
| 真机连续 10 分钟无保护停 | **未在本轮实证**（真机为预约资源，本轮不可达） | 真机窗口重复执行本命令 10 分钟，回填 `status` 与 `events` |
| 真机 tick 抖动 p99 < 2 ms | **未在本轮实证**（离线为虚拟时钟，p99 恒 0，是代理值） | 回填真机报告的 `metrics.tick_jitter_p99_ms` |
| 真机到位判定误差 < 0.1° | **未在本轮实证**（离线为理想跟随模型，误差恒 0） | 回填真机报告的 `metrics.arrival_err_deg` |

真机回填后本文件追加一节「真机窗口回填」，并把 `mode` 从 `fake` 改成 `live`
（**报告 schema 完全相同**，便于逐项对比）。

---

## 7. 证据文件索引

| 类型 | 路径 |
|------|------|
| 600 s 长跑报告 | `build/reports/s03_offline_600s.json` |
| 样本轨迹（生成物） | `build/reports/s03_sample_traj.json` |
| 注入报告（×3） | `build/reports/s03_inject_follow_error_6.json`、`s03_inject_feedback_stall_250.json`、`s03_inject_controller_conflict.json` |
| 命令证据（stdout） | `1ca00f83-*.stdout`（回归 540 passed）、`924785f6-*.stdout`（审计）、`090e6722-*.stdout`（codegen）、`4eb0b9cb-*.stdout` + `71342199-*.stdout`（长跑与断言）、`db3fb1f0-*.stdout` / `2f030eb9-*.stdout` / `b2c52445-*.stdout`（三类注入）、`b72b38dc-*.stdout`（缺 PYTHONPATH 的降级路径）、`35fd1df6-*.stdout`（注入/样本轨迹测试 19 passed） |
| 代码 | `dualarm_ws/src/rm_executor/rm_executor/sample.py`、`injection.py`、`cli.py`（`--inject`）、`tests/test_injection.py` |
| 厂商动态库兜底 | `dualarm_ws/src/rmctl/rmctl/vendor_env.py`（预加载 `.deps/libstdcxx`）、`rmctl/__init__.py`、`planner.py`、`verify.py`、`viewer.py`、`model.py`、`tests/conftest.py` |

---

## 8. 已知限制

1. **离线代理 ≠ 真机**：本文件所有数字都来自虚拟时钟 + 理想跟随模型；真机三条
   验收未实证（见 §6）。
2. **fake 下的 `rt_factor = 1.26`**：长跑的等效时长包含每轮的预就位 / 到位阶段，
   不是「600 s 纯栅格流式」；真机长跑的 rt_factor 需在真机窗口重测。
3. **环境依赖（已修）**：`rmctl` 的 legacy 路径需要厂商规划内核（GLIBCXX_3.4.31+）
   与 `build/model` 物化模型。GLIBCXX 问题已在**代码层**兜底（§1.1：裸环境自动
   预加载仓库自带 `.deps/libstdcxx`；实在拿不到时 legacy 路径**诚实降级**为
   退出码 3 + reason + 可粘贴 next_action，不再抛裸异常、不再伪造回放统计）。
   `build/model` 仍需 `python3 -m rmctl.cli init` 物化（缺失时报
   `Failed to parse URDF`，属**环境准备**而非代码缺陷）。
4. **注入只覆盖三类**：跟随误差、断流、控制器抢轨迹；其它异常（传输失败、配置
   缺失、契约拒绝）由既有单测覆盖，未做成 CLI 注入项。
