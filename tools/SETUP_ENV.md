# SETUP_ENV.md —— 一条命令重建环境（T04）

> 对应验收句（R010）：**新机器 30 分钟内一条命令重建完整环境**。
> 本文档记录**实测**的重建过程、耗时与结果；未跑通的部分明确写明原因与复现命令，
> **不含任何伪造的耗时数据**。

## 1. 入口

```bash
# 联网重建受支持基线（Python 3.12 + Drake 1.55.0）
bash tools/setup_env.sh --profile py312

# 用本机离线镜像重建（不触网；需先有 vendor/wheels 镜像）
bash tools/lock_deps.sh --download --profile py312   # 一次性准备镜像
bash tools/setup_env.sh --offline --profile py312

# 只看将要执行什么（不产生任何副作用）
bash tools/setup_env.sh --dry-run --profile py312
```

参数：

| 参数 | 含义 |
|------|------|
| `--offline` | `pip install --no-index --find-links vendor/wheels`（离线） |
| `--venv DIR` | venv 目录（默认 `<仓库根>/.venv`；已存在则复用） |
| `--profile py312\|py310` | 目标 profile：py312 = baseline（受支持）/ py310 = fallback（非受支持降级档） |
| `--dry-run` | 只打印将要执行的命令，不做任何改动 |

流程：选 profile → 检测/创建 venv → 升级 pip（离线跳过）→ 按 constraints 安装完整依赖闭包
（`--require-hashes`）→ 调 `tools/verify_env.py --check` → 打印 Drake / robotic_arm 实装版本与
`versions.lock` 对照结果 → 输出 wall time。退出码：0 成功 / 1 安装或自检失败 / 2 参数或环境错误。

实现要点：

* **隔离**：所有安装都发生在目标 venv 内；不写系统 site-packages、不用 `--user`。
  在 conda 环境内（`CONDA_PREFIX` 非空）同样只在 venv 内安装，并打印提示。
* **hash 钉死**：pip 的 `--require-hashes` 要求**每条 requirement 自身**带 `--hash`，
  只在 constraints 文件里写 hash 会被 pip 判为「hash 缺失」。
  本项目的 `constraints/*.txt` 每行都是 `name==version --hash=sha256:...`，
  因此它同时就是一份 hash 钉死的 requirements 文件，脚本用 `-r constraints/<profile>.txt --require-hashes` 传入。
* **venv 内无 pip 的兜底**：Debian/Ubuntu 未装 `python3-venv`（`ensurepip` 缺失）时，
  脚本退化为 `python -m venv --without-pip`，再用宿主 pip 的 `--python` 能力（pip ≥ 22.3）
  向该 venv 注入依赖；宿主 pip 过旧则明确报错。

## 2. 自检工具 `tools/verify_env.py`

| 子检查 | 作用 | 缺依赖时的行为 |
|--------|------|----------------|
| `--lock-only` | 只校验 `versions.lock` 自洽性（必填字段 / constraints 钉版一致 / wheels 映射格式） | **不依赖已安装包**，新机也能跑 |
| `--check` | 逐包比对 lock 声明版本 vs `importlib.metadata` 实装版本 | 未安装/版本不符 → 打印差异表并以退出码 1 结束 |
| `--wheels` | 校验 `vendor/wheels` 的 SHA256SUMS | 镜像缺失 → `SKIP` + 明确提示，不崩溃；镜像被改 → 退出码 1 |
| `--forbidden` | 确认禁用名单唯一真相源是 `versions.lock:forbidden_apis` | 名单被削减 / 与白名单冲突 → 退出码 1 |
| `--package-map` | `package://` 解析冒烟（复用 `rm75_drake.package_map`） | Drake 不可导入 → 退出码 2 并说明原因 |

profile 选择：按**当前解释器**自动选（3.12 → `baseline`，3.10 → `fallback`）；
解释器版本不在锁里声明的两档中 → 退出码 2（显式失败，不静默退回某一档）。
`--json` 输出机器可读结果（CI 可直接消费）。

## 3. 实测记录（2026-11-13）

### 3.1 受支持基线（py312 / Drake 1.55.0）——**已完成**

* 环境：远程 docker 容器 `ros_dev`，conda env `py312`（Python 3.12.13），Linux x86_64。
* 离线镜像：仓库 `vendor/wheels/`（18 个 wheel，140 MB；由 `bash tools/lock_deps.sh --download --profile py312` 取得，
  `sha256sum -c SHA256SUMS` 全量 OK）。
* 命令（在容器内，仓库副本置于 `/tmp/rm75-env-test/src`）：

  ```bash
  bash tools/setup_env.sh --offline --profile py312 --venv /tmp/rm75-env-test/venv
  ```

* 结果：**成功**，退出码 0。
  * venv 从零创建 → 18 个包全部从本地镜像按 `--require-hashes` 安装成功；
  * `tools/verify_env.py --check` 逐包对照 **18/18 全部 OK**（drake 1.55.0、mosek 11.1.2、
    robotic_arm 1.1.7、numpy 1.26.4、scipy 1.11.4、toppra 0.6.11 …）；
  * 关键包版本打印正常；
  * **wall time = 7 s**（含创建 venv + 安装 + 自检；目标 30 min，余量充足）。

  > 说明：7 s 是**离线**重建的耗时。联网首跑还要叠加镜像下载耗时
  > （本机实测 `lock_deps.sh --download --profile py312` 用时约 170 s，140 MB，
  > 其中 pip HTTP 缓存冷热影响很大；镜像下载属于"一次性准备"，不属于每次重建）。

### 3.2 降级档（py310 / Drake 1.51.1）——**未跑通，原因是锁文件本身，不是本任务**

本机（开发机）只有 Python 3.10，按理走 fallback 档。实测：

```bash
bash tools/lock_deps.sh --download --profile py310     # 解析失败
```

pip 报错：

```
ERROR: Cannot install fonttools==4.68.0 because these package versions have conflicting dependencies.
Additionally, some packages in these conflicts have no matching distributions available for your environment: fonttools
```

* 根因（已用 PyPI 元数据核对）：`fonttools 4.68.0` 的 `requires_python` 是 **`>=3.11`**，
  在 Python 3.10 上**不存在任何兼容发行文件**（wheel 与 sdist 均要求 ≥3.11）。
* 该版本是 T01/T03 钉死的传递依赖（matplotlib 3.8.4 → fonttools），
  `tools/gen_constraints.py` 选文件时**没有考虑 `requires_python`**，
  因此 `constraints/py310.txt` 里出现了 cp310 装不上的 fonttools 版本。
* 影响面：`fallback` 档（py310）当前**装不上**，只有 `baseline` 档（py312）可离线重建。
  fallback 本就是"非受支持基线，仅 fast 档"，不影响正式交付路径。
* 修复建议（属 T01/T03 的锁维护，不在 T04 范围内）：
  把 `versions.lock:packages.fonttools` 降到支持 py3.10 的版本（例如 `4.55.x`），
  并让 `tools/gen_constraints.py` 在选文件时按 `requires_python` 过滤，
  再重跑 `gen_constraints.py` 与 `lock_deps.sh --download --profile py310`。

### 3.3 本机（开发机）环境的两个限制

1. **无 `ensurepip`**：开发机 Python 3.10 未装 `python3-venv`，`python3 -m venv` 会失败。
   `setup_env.sh` 已按上文兜底（`--without-pip` + 宿主 pip `--python`），因此脚本本身可用；
   但开发机只有 Python 3.10，跑不了 baseline 档（需要 3.12）。
2. **baseline 档实测因此放在远程容器**：`ros_dev` 容器内 `conda activate py312`
   即 Python 3.12.13，与 baseline 档一致，故离线重建实测在该容器内完成。

## 4. 复现命令汇总

```bash
# 0) 准备离线镜像（一次性）
bash tools/lock_deps.sh --download --profile py312
bash tools/lock_deps.sh --verify

# 1) 一条命令重建 + 自检
bash tools/setup_env.sh --offline --profile py312 --venv /tmp/rm75-env-test/venv

# 2) 单独跑自检（CI 复用）
python3 tools/verify_env.py --lock-only
python3 tools/verify_env.py --check
python3 tools/verify_env.py --wheels
python3 tools/verify_env.py --forbidden
python3 tools/verify_env.py --package-map
python3 tools/verify_env.py --check --json
```

退出码语义：`0` 通过 / `1` 检查失败 / `2` 运行期错误（锁不可读、镜像缺失、解释器不受支持等）。
