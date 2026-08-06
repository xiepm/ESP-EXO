# ESP-EXO

ESP-EXO 是基于 ESP32-S3 的双侧髋关节外骨骼控制器工程，使用 ESP-IDF 开发。

当前仓库保存改造前的原型基线，主要包括：

- ESP32-S3；
- 1 Mbps TWAI/CAN 总线；
- 两台 RS01 执行器，当前 CAN ID 为 `0x10` 和 `0x20`；
- NimBLE BLE 串口控制台；
- 100 Hz 双电机正弦参考与阻抗控制示例。

> 当前代码仍属于 `esp_can` 原型基线，不代表最终产品架构。后续将按照公共类型、RS01 驱动、髋关节对象、单一 100 Hz 控制任务、状态机、BLE 产品协议和步态控制等阶段逐步重构。

阶段 0 的软件基线、构建证据和硬件验收边界见
[阶段 0 基线](docs/baseline/PHASE_0_BASELINE.md)。连接开发板后的逐项验证见
[硬件冒烟测试](docs/baseline/SMOKE_TEST.md)。

## 目录

```text
ESP-EXO/
├── main/                 # ESP-IDF 主组件及当前固件源码
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

GPIO、电机左右安装关系和软件零位应以实际设备标定结果为准。

## 构建

本基线锁定 ESP-IDF v5.5（内部版本 5.5.0）。在 PowerShell 中激活环境后构建：

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

0. 固化现有基线；
1. 建立公共类型和配置；
2. 重构 RS01 驱动及主动反馈；
3. 建立髋关节对象和软件零位；
4. 建立唯一 100 Hz 控制任务；
5. 建立状态机及 START/STOP 执行器；
6. 建立产品 BLE 协议；
7. 迁移正弦参考和阻抗控制；
8. 加入五元命令和基础模式；
9. 加入传感快照和步态估计；
10. 加入相位曲线和遥测；
11. 完成产品清理和整体回归。
