# ESP-EXO

ESP-EXO 是基于 ESP32-S3 的双侧髋关节外骨骼控制器工程，使用 ESP-IDF 开发。

当前仓库已完成阶段 5“系统状态机和 START/STOP 执行器”，主要包括：

- ESP32-S3；
- 1 Mbps TWAI/CAN 总线；
- 两台 RS01 执行器，当前 CAN ID 为 `0x10` 和 `0x20`；
- 两台执行器独立反馈快照、约 100 Hz 主动上报及 100 ms 在线超时；
- 左/右髋关节对象、显式 A/B 映射、安装方向换算和 RAM 软件零位；
- 一个上电只创建一次、静态分配并永久运行的 100 Hz 控制任务；
- LEFT/RIGHT 同周期读取、零等待实时运动发送和按需时序诊断；
- 六态整机状态机、派生启动就绪条件及异步 START/STOP 执行器；
- 唯一周期运动输出门，以及调试维护命令与启停序列的互斥；
- NimBLE BLE 串口控制台；
- 100 Hz 双电机正弦参考与阻抗控制示例。

> 当前代码仍处于分阶段重构期，不代表最终产品架构。公共类型、硬件配置、RS01 驱动、髋关节对象、唯一 100 Hz 控制任务和整机启停状态机已经集中；产品 BLE 二进制协议和步态控制仍将按后续阶段实现。左右映射与方向已经通过两侧独立运动采集确定，固件不会依据 CAN ID 猜测人体侧别。

阶段 0 的软件基线、构建证据和硬件验收边界见
[阶段 0 基线](docs/baseline/PHASE_0_BASELINE.md)。连接开发板后的逐项验证见
[硬件冒烟测试](docs/baseline/SMOKE_TEST.md)。

阶段 0 已于 2026-08-06 完成，正式基线标签为 `baseline-esp-can-v0`。

阶段 1 已于 2026-08-06 完成，正式标签为 `phase1-common-types-config`。设计边界见
[阶段 1 设计](docs/phase1/PHASE_1_DESIGN.md)，构建、烧录和实机结果见
[阶段 1 验收](docs/phase1/PHASE_1_ACCEPTANCE.md)。

阶段 2 已于 2026-08-06 完成，正式标签为 `phase2-rs01-driver-feedback`。协议和
对象边界见 [阶段 2 设计](docs/phase2/PHASE_2_DESIGN.md)，构建、主动反馈、BLE
及双电机运动结果见 [阶段 2 验收](docs/phase2/PHASE_2_ACCEPTANCE.md)。

阶段 3 已于 2026-08-06 完成，正式标签为 `phase3-hip-joint-software-zero`。对象、
坐标和软件零位边界见 [阶段 3 设计](docs/phase3/PHASE_3_DESIGN.md)，构建、标定、
BLE 及运动回归结果见 [阶段 3 验收](docs/phase3/PHASE_3_ACCEPTANCE.md)。

阶段 4 已于 2026-08-06 完成，正式标签为 `phase4-single-100hz-control-task`。唯一
周期源、实时发送和任务边界见 [阶段 4 设计](docs/phase4/PHASE_4_DESIGN.md)，静态
审计、5 分钟空闲、BLE 重连及 3x10 秒运动结果见
[阶段 4 验收](docs/phase4/PHASE_4_ACCEPTANCE.md)。

阶段 5 已于 2026-08-07 完成，正式标签为 `phase5-system-state-start-stop`。状态模型、
请求语义和执行序列见 [阶段 5 设计](docs/phase5/PHASE_5_DESIGN.md)；静态、构建、
无运动门控、BLE 归零、状态竞争及 3 轮运动证据见
[阶段 5 验收](docs/phase5/PHASE_5_ACCEPTANCE.md)。

## 目录

```text
ESP-EXO/
├── main/                 # ESP-IDF 主组件及当前固件源码
├── docs/baseline/        # 阶段 0 合同、报告和硬件证据
├── docs/phase1/          # 阶段 1 公共类型、配置设计和验收记录
├── docs/phase2/          # 阶段 2 RS01 驱动、主动反馈设计和验收记录
├── docs/phase3/          # 阶段 3 髋关节对象、软件零位设计和验收记录
├── docs/phase4/          # 阶段 4 唯一 100 Hz 控制任务设计和验收记录
├── docs/phase5/          # 阶段 5 状态机、启停执行器设计和验收记录
├── tools/                # 阶段 0 构建验证脚本
├── CMakeLists.txt        # 工程构建入口
├── sdkconfig             # 当前 ESP32-S3 工程配置
└── CONSOLE_COMMANDS.md   # 当前串口/BLE 控制台命令说明
```

## 当前硬件配置

| 项目 | 当前值 |
|---|---:|
| 芯片 | ESP32-S3 |
| TWAI 波特率 | 1 Mbps |
| TWAI TX | GPIO 4 |
| TWAI RX | GPIO 2 |
| 电机 A | CAN ID `0x10` |
| 电机 B | CAN ID `0x20` |
| 左髋 | 电机 B，方向 `+1`（屈曲为正） |
| 右髋 | 电机 A，方向 `-1`（屈曲为正） |

GPIO 和电机左右安装关系来自 2026-08-06 实机标定；RAM 软件零位仍需每次启动后在中立姿态显式建立。

## 构建

当前工程锁定 ESP-IDF v5.5（内部版本 5.5.0）。在 PowerShell 中激活环境后构建：

```powershell
. 'C:\Espressif\tools\Microsoft.v5.5.PowerShell_profile.ps1'
cd D:\ESP32\esp_can
idf.py build
```

执行阶段 0 的干净构建、尺寸检查和产物检查：

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\verify_phase0.ps1
```

烧录并打开串口监视器：

```powershell
idf.py -p COMx flash monitor
```

当前控制台命令见 [CONSOLE_COMMANDS.md](CONSOLE_COMMANDS.md)。

## 重构顺序

0. 固化现有基线（已完成：`baseline-esp-can-v0`）；
1. 建立公共类型和配置（已完成）；
2. 重构 RS01 驱动及主动反馈（已完成）；
3. 建立髋关节对象和软件零位（已完成）；
4. 建立唯一 100 Hz 控制任务（已完成：`phase4-single-100hz-control-task`）；
5. 建立状态机及 START/STOP 执行器（已完成：`phase5-system-state-start-stop`）；
6. 建立产品 BLE 协议；
7. 迁移正弦参考和阻抗控制；
8. 加入五元命令和基础模式；
9. 加入传感快照和步态估计；
10. 加入相位曲线和遥测；
11. 完成产品清理和整体回归。
