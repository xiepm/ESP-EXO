# 阶段 0：esp_can 原型基线

## 结论

本文件是阶段 0 的唯一基线事实源。当前主机构建基线已经通过；由于 2026-08-06
检测时没有连接 ESP32-S3，烧录、启动、BLE、CAN 和双电机运行证据仍为
`PENDING`。在这些硬件项目通过前，阶段 0 的总体状态是
`BLOCKED_BY_HARDWARE_CONNECTION`，不能标记为最终完成。

软件基线使用临时标签 `baseline-esp-can-v0.1`。全部硬件项目通过后，才创建
最终标签 `baseline-esp-can-v0`。

## 基线身份

| 项目 | 值 |
|---|---|
| 主仓库 | `https://github.com/xiepm/ESP-EXO.git` |
| 改造前上游 | `https://github.com/saltedfish12138/esp_can.git` |
| 上游提交 | `52a70d9` (`add imp control`) |
| ESP-EXO 基线父提交 | `0946666e00b48e4263c38edb86b100b7fd0e0611` |
| 软件基线标签 | `baseline-esp-can-v0.1` |
| 目标芯片 | ESP32-S3 |
| ESP-IDF | v5.5（内部版本 5.5.0） |

此基线只冻结改造前原型的可观察行为，不把当前实现认定为产品架构，也不在阶段
0 修复双电机反馈归属、控制任务模型或 BLE 产品协议。

## 固化的当前配置

| 项目 | 当前值 |
|---|---:|
| Flash | 2 MB，DIO，40 MHz |
| 串口控制台 | UART0，115200 baud |
| TWAI 模式 | Normal，接收全部帧 |
| TWAI 波特率 | 1 Mbps |
| TWAI TX/RX | GPIO4 / GPIO2 |
| TWAI RX 队列 | 32 帧 |
| 控制器 Master ID | `0xFD` |
| 默认目标电机 | `0x10` |
| 双电机 ID | `0x10`、`0x20` |
| 电机编译类型 | `MOTOR_TYPE=5` |
| BLE 名称 | `RobStride-CAN` |
| BLE 服务 | Nordic UART Service 兼容文本通道 |
| 阻抗基线周期 | 10 ms（100 Hz `esp_timer` 回调） |

GPIO、电机左右安装关系和机械零位尚未形成产品配置；阶段 0 只记录当前事实。

## 构建环境

| 工具 | 已验证版本或路径 |
|---|---|
| ESP-IDF | `C:\Espressif\v5.5\esp-idf` |
| PowerShell 激活脚本 | `C:\Espressif\tools\Microsoft.v5.5.PowerShell_profile.ps1` |
| Python | `C:\Espressif\tools\python\v5.5\venv\Scripts\python.exe` |
| CMake | 3.30.2 |
| Ninja | 1.12.1 |
| Xtensa GCC | 14.2.0 (`esp-14.2.0_20241119`) |
| esptool | 4.12.0 |

可重复验收命令：

```powershell
cd D:\ESP32\esp_can
powershell -ExecutionPolicy Bypass -File .\tools\verify_phase0.ps1
```

脚本默认执行 `idf.py fullclean`、`idf.py build`、`idf.py size`，校验四个必要
产物并输出 SHA-256。只有显式传入 `-Flash -Port COMx` 才会烧录。

## 已取得的软件证据

2026-08-06 在 ESP-IDF v5.5 上完成 1180 个构建步骤，未发现编译 warning、
error、链接错误或分区溢出。

| 产物 | 字节数 | SHA-256 |
|---|---:|---|
| `twai_network_master.bin` | 608352 | `177EC8765B1FA1421283CEF1184C69E6D8D1FAB9E9A927321C4968076B7CB228` |
| `twai_network_master.elf` | 7364532 | `96A78C79A86B49071D961CF0C669DE6E8CBE72DC243D7D9BCF1B3D262E66CA89` |
| `bootloader.bin` | 21088 | `86BE90009EA998ED3299791C921912045FC72589F7A1F4A69B1CA0F035F990F4` |
| `partition-table.bin` | 3072 | `7F00B6C042A89B15B0CAC534F82ED988CAF29278FF5700B0C511EB1B5BB7C820` |

应用镜像占用 608233 字节；1 MiB 应用分区剩余 `0x6b7a0` 字节（约 42%）。
上述哈希用于追溯本次真实构建，不作为跨时间的逐字节复现指标，因为固件包含
Git 版本和构建时间信息。

## 验收状态

| 验收项 | 状态 | 证据或阻塞原因 |
|---|---|---|
| ESP-IDF v5.5 环境可激活 | PASS | `idf.py --version`、工具版本已核验 |
| 干净编译 | PASS | 1180 个步骤成功 |
| 尺寸与必要产物 | PASS | 见上表，应用分区未溢出 |
| 源码行为未在阶段 0 重构 | PASS | 仅配置迁移、文档和验收工具变更 |
| ESP32-S3 烧录和连续启动 | PENDING | 当前无 ESP32/USB-UART 在线端口 |
| UART 控制台 | PENDING | 需要开发板 |
| BLE 广播、连接和命令 | PENDING | 需要开发板/手机 |
| 两台 RS01 逐台反馈 | PENDING | 需要开发板、收发器和电机 |
| 双电机 `imp_start/imp_stop` | PENDING | 需要完整硬件 |

当前 Windows 只在线枚举到 COM3、COM4，二者都是 Bluetooth RFCOMM，禁止用于
烧录。历史记录中存在已断开的 CH343 `COM5`；重新插板后必须重新枚举，不能仅
凭历史端口号烧录。

## 已知原型限制

- `status` 显示的电机编号来自当前选择，但反馈内容来自最后收到的任一电机。
- BLE 自动反馈文本不含源电机 ID；双电机同时上报时不能可靠归属。
- 串口反馈打印的 200 ms 节流由两台电机共用，可能只持续看到其中一台。
- `imp_start` 在缺少反馈时仍可能以中心位置 0 启动并返回成功。
- 双电机状态存在无锁读写；100 Hz 控制位于 `esp_timer` 回调中。

这些限制是后续阶段的输入，阶段 0 不修改它们。硬件验收必须按
[SMOKE_TEST.md](SMOKE_TEST.md) 逐台验证，不能以一次 `status` 输出代替。

## 阶段 0 最终关闭条件

同时满足以下条件后，才把本文件状态改为 `COMPLETE` 并创建
`baseline-esp-can-v0`：

1. 对固化提交重新执行主机构建脚本并通过；
2. 完成一次真实烧录，连续运行 60 秒无 panic、assert、看门狗或重启；
3. UART 和 BLE 命令通道通过；
4. CAN 控制器保持 RUNNING，逐台识别 `0x10`、`0x20` 的真实反馈；
5. 双电机阻抗基线连续运行至少 10 秒，并成功停止和失能；
6. 保存设备、端口、供电、收发器、电机映射和完整测试结果。
