# ESP-EXO

ESP-EXO 是基于 ESP32-S3 的双侧髋关节外骨骼控制器工程，使用 ESP-IDF 开发。

当前仓库已完成阶段 9 的软件实现与 `0.9.0` 工程功能真机验收。主要包括：

- ESP32-S3；
- 1 Mbps TWAI/CAN 总线；
- 两台 RS01 执行器，当前 CAN ID 为 `0x10` 和 `0x20`；
- 两台执行器独立反馈快照、约 100 Hz 主动上报及 100 ms 在线超时；
- 左/右髋关节对象、显式 A/B 映射、安装方向换算和 RAM 软件零位；
- 一个上电只创建一次、静态分配并永久运行的 100 Hz 控制任务；
- LEFT/RIGHT 同周期读取、零等待实时运动发送和按需时序诊断；
- 六态整机状态机、派生启动就绪条件及异步 START/STOP 执行器；
- 唯一周期运动输出门，以及调试维护命令与启停序列的互斥；
- 产品 BLE v1 二进制协议，以及 UART-only 工程调试台；
- 独立的纯 C 正弦参考生成器，以及 LEFT/RIGHT 关节语义的 100 Hz 阻抗控制示例；
- 统一的 LEFT/RIGHT 五元关节命令、六种基础模式和通用控制器；
- LEFT/RIGHT 五元目标到 A/B 执行器坐标的集中转换，保持原 A-/B+ 电机行为；
- BLE v1 的 IDLE 静态控制配置扩展和网页模式/五元参数读写，不使用 BLE 100 Hz 命令流。
- 人体关节、参考轨迹、UART 和手机显示统一使用度，RS01/BLE v1 线格式在边界显式换算；
- 每周期统一传感快照，显式记录来源序号/时间、新鲜度、左右 skew、能力/有效 mask 和 reset epoch；
- 可回放的 HIP_EVENT_TBE 髋角代理步态估计，提供连续相位、周期、步频、置信度和质量位；
- 面向 ESP32 的编译期静态助力策略 ops、固定 64-byte 最大工作区和传感/步态依赖声明；
- 阶段 9 仅生成绑定输入序号和 START 代次的 SHADOW 建议，真实助力输出恒关闭；
- BLE/网页与 UART 可读取步态/助力状态，手机仍不参与 100 Hz 实时控制。

> 当前代码仍处于分阶段重构期，不代表最终产品架构。传感快照和第一版髋角代理步态估计已经集中，但 phase profile、AAN residual 仍是 `implemented=false` 的接口占位；相位曲线真实助力和连续遥测属于阶段 10。左右映射与方向已经通过两侧独立运动采集确定，固件不会依据 CAN ID 猜测人体侧别。

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

阶段 6 已于 2026-08-07 完成，正式标签为 `phase6-product-ble-protocol`。协议、任务边界和
手机流程见 [阶段 6 设计](docs/phase6/PHASE_6_DESIGN.md) 与
[BLE 协议 v1](docs/phase6/BLE_PROTOCOL_V1.md)；构建、烧录、畸形帧、归零、幂等、断连语义
及 START/STOP 运动结果见 [阶段 6 验收](docs/phase6/PHASE_6_ACCEPTANCE.md)。

阶段 7 已于 2026-08-07 完成，正式标签为 `phase7-sine-impedance-migration`。纯参考模块、
LEFT/RIGHT 坐标边界和等价迁移见 [阶段 7 设计](docs/phase7/PHASE_7_DESIGN.md)；C 数值回放、
干净构建、BLE 归零、三轮运动、0.5 Hz 同相关节反馈、BLE 重连及 100 Hz/TWAI 结果见
[阶段 7 验收](docs/phase7/PHASE_7_ACCEPTANCE.md)。

2026-08-10 在不改变阶段 7 物理波形的前提下完成阶段 8 软件迁移，当前固件版本为
`0.8.0`：

- 人体髋关节采样、RAM 软件零位、正弦幅值和控制中心使用 `deg`、`deg/s`；
- UART 角度类命令参数和状态输出使用 `deg`、`deg/s`、`deg/s²`；
- RS01 驱动继续使用手册规定的 `rad`、`rad/s`、`rad/s²`；
- 已发布的 BLE v1 载荷继续使用 `mrad`、`mrad/s`，避免破坏旧客户端；验收客户端和
  手机界面将其换算为 `deg`、`deg/s` 后显示；
- 新增 `HOLD/POSITION/VELOCITY/TORQUE/DAMPING/IMPEDANCE` 六种产品模式，底层
  仍统一使用 RS01 Motion 五元帧；
- 模式和五元参数只允许在 IDLE 设置，START 时锁存，RUNNING 中拒绝修改；
- 默认 `IMPEDANCE + SINE` 继续保持阶段 7 的数值行为。

阶段 8 的控制合同见 [阶段 8 设计](docs/phase8/PHASE_8_DESIGN.md)，软件和真机结果见
[阶段 8 验收](docs/phase8/PHASE_8_ACCEPTANCE.md)。

阶段 9 固件版本为 `0.9.0`。架构、文献算法映射和验收边界分别见
[阶段 9 设计](docs/phase9/PHASE_9_DESIGN.md)、
[文献映射](docs/phase9/PHASE_9_LITERATURE_MAP.md) 与
[阶段 9 验收](docs/phase9/PHASE_9_ACCEPTANCE.md)。主机真实 C 回放入口为
`tools/phase9_replay.ps1`；阶段 9 BLE 默认只读/可恢复配置验收入口为
`tools/phase9_ble_acceptance.py`。阶段 9 已于 2026-08-11 完成正常人体左右交替步态、变速、停止退锁和
静止补测，工程功能结论为 COMPLETE / PASS；没有外部 gait event 真值，因此不宣称绝对相位精度。

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
├── docs/phase6/          # 阶段 6 产品 BLE 协议、设计和验收记录
├── docs/phase7/          # 阶段 7 正弦参考、关节阻抗迁移和验收记录
├── docs/phase8/          # 阶段 8 五元命令、基础模式设计和验收记录
├── docs/phase9/          # 阶段 9 传感、步态、SHADOW 助力设计和验收记录
├── tools/                # 构建、BLE 验收和阶段 9 真实 C 回放脚本
├── CMakeLists.txt        # 工程构建入口
├── sdkconfig             # 当前 ESP32-S3 工程配置
└── CONSOLE_COMMANDS.md   # UART-only 工程调试命令说明
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
. .\tools\activate_esp_idf.ps1
idf.py build
```

激活脚本会优先使用当前 `IDF_TOOLS_PATH`，否则自动寻找 Windows EIM 默认安装目录，
并检查 `idf.py`、`IDF_PATH` 和 Python 虚拟环境是否真实可用。脚本必须使用点调用，
这样环境变量和 `idf.py` 命令才会保留在当前 PowerShell 会话中。

本机采用 EIM 的版本化目录布局，正确入口是 EIM 生成的
`C:\Espressif\tools\Microsoft.v5.5.PowerShell_profile.ps1`（仓库脚本会自动调用它）。
不要直接运行 ESP-IDF 源码目录中的 `export.ps1`：该脚本会按传统
`%USERPROFILE%\.espressif` 布局寻找 Python 和工具链，从而产生“环境缺失”的假象。

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
6. 建立产品 BLE 协议（已完成：`phase6-product-ble-protocol`）；
7. 迁移正弦参考和阻抗控制（已完成：`phase7-sine-impedance-migration`）；
8. 加入五元命令和基础模式（已完成）；
9. 加入传感快照和步态估计（已完成：工程与功能范围 COMPLETE / PASS）；
10. 加入相位曲线和遥测；
11. 完成产品清理和整体回归。
