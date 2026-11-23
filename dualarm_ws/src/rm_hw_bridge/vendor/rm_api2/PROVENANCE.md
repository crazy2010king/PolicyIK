# RM-API2 头文件来源与许可（PROVENANCE）

本目录下的头文件是 **睿尔曼 RM-API2（Robotic_Arm）C ABI 的厂商头文件**，
用于在 CI/本地做 **C 参考编译校验**（把 `sizeof` / `offsetof` 与
`rm_hw_bridge/generated/layout_golden.json` 逐字段比对），
从而证明 ctypes 生成物与厂商 ABI **逐字段一致**。

## 版本

| 项 | 值 |
|----|----|
| SDK | RM-API2（Robotic_Arm） |
| SDK 版本 | **1.1.7**（取自 `rm_version.h` 的 `#define SDK_VERSION ("1.1.7")`） |
| 头文件来源包 | `rm_driver`（ROS2 驱动包，README 标题 "RealMan Robot rm_driver User Manual V1.8"） |
| 本机来源路径 | `/tmp/rmrepo/rm_driver/include/rm_driver/`（另有 `/tmp/rmros/`、`/tmp/rc/ros2_rm_robot-humble/` 等**内容完全相同**的副本，md5 一致） |
| 采集日期 | 2026-11-13 |
| 关联二进制 | `.deps/rm_api2/libapi_c.so`（同机 `rm_api_version()` 返回 `1.1.7`）；该 `.so` 来自 wheel `robotic_arm-1.1.7-py3-none-any.whl` |

## 文件清单与 sha256

| 文件 | sha256 |
|------|--------|
| `include/rm_define.h` | `5175bc5f4e714d623c428a243c80a6ea594e4eb1630a96617994be41a412b684` |
| `include/rm_interface.h` | `29ad630fff1f06526dceb0f7e8bb8f6dc2a921ce5e16b6af061012158a852b60` |
| `include/rm_interface_global.h` | `1dc9365be8e70a1f7c70c2835498a0952f6b2ee45a5fd03ea86ff6bf4f2b0c42` |
| `include/rm_version.h` | `e80157fd0000aeb25e5576de53417cb4e71b2f2812904790e52314dbd59710f4` |

> 任务书只点名了 `rm_interface.h` / `rm_define.h` / `rm_version.h` 三个文件；
> `rm_interface.h` 自身 `#include "rm_interface_global.h"`，**缺少它 C 参考编译会失败**，
> 因此一并收录（内容与上游逐字一致，sha256 如上）。

## 许可声明

- 上游 `rm_driver` / RM-API2 SDK 由 **RealMan Intelligent Technology (Beijing) Co., Ltd.**（睿尔曼）发布，
  任务书记录其许可为 **Apache-2.0**。
- **诚实声明**：本机 checkout（`/tmp/rmrepo/rm_driver`）中**未找到 LICENSE 文件**，
  且当前环境**无外网**（`raw.githubusercontent.com` 连接被重置），因此**无法在本地核验许可文本**。
  本目录仅保留**接口声明头文件**（无厂商实现代码），用于 ABI 一致性校验；
  正式发布前请人工补齐 LICENSE 原文或改为**不随仓库分发**（改为在测试中按环境变量
  `RM_API2_INCLUDE` 指向本机 SDK 头文件目录）。

## 平台说明（重要）

`layout_golden.json` 与 C 参考编译校验都是 **x86-64 SysV ABI** 下的布局
（指针 8 字节，如 `rm_movej_canfd_mode_t` = 24 字节）。
RM-API2 同时提供 `libs/linux_arm/libapi_c.so`（ARM32，指针 4 字节 ⇒
`rm_movej_canfd_mode_t` 等含指针结构体尺寸会变小）。
**若要在 ARM32 上运行，必须用该平台的编译器重新生成黄金表**，不得直接复用本表。

## 与 `abi_spec.yaml` 的关系

- `rm_hw_bridge/abi_spec.yaml` 是本项目的**唯一真值**，字段名 / 字段顺序 / 类型
  逐字来自本目录的头文件。
- 生成物 `rm_hw_bridge/generated/rm_api2_ctypes.py` 头部记录
  `abi_spec.yaml` 的 sha256 与 SDK 版本号。
- 校验方式：`dualarm_ws/src/rm_hw_bridge/tests/test_codegen_layout.py::test_c_reference_layout_matches_golden`
  用 gcc 编译一个 C 参考程序打印 `sizeof` / `offsetof` 与黄金表比对；
  若找不到头文件或 gcc，该用例会 **显式 skip 并写明原因**（绝不静默通过）。
