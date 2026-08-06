# 阶段 0：esp_can 原型基线

## 结论

本文件是阶段 0 的唯一基线事实源。2026-08-06 已完成主机构建、真实烧录、
61 秒连续启动、UART、BLE NUS、双 RS01 各 30 秒反馈和三次双电机阻抗启停验证。
阶段 0 状态为 `COMPLETE`，验证结论为 `verified_match_with_recorded_caveats`。

`baseline-esp-can-v0.1` 保留为软件基线；包含本次硬件证据的最终标签为
`baseline-esp-can-v0`。结构化结果见
[hardware_smoke_20260806.json](evidence/hardware_smoke_20260806.json)，严格波形重放
见 [impedance_analysis_20260806.json](evidence/impedance_analysis_20260806.json)。

## 基线身份

| 项目 | 值 |
|---|---|
| 主仓库 | `https://github.com/xiepm/ESP-EXO.git` |
| 改造前上游 | `https://github.com/saltedfish12138/esp_can.git` |
| 上游提交 | `52a70d9` (`add imp control`) |
| ESP-EXO 基线父提交 | `0946666e00b48e4263c38edb86b100b7fd0e0611` |
| 软件基线标签 | `baseline-esp-can-v0.1` |
| 硬件验证固件源码提交 | `234088602ed3f5f0ff266b00a61e40f95660d950` |
| 最终基线标签 | `baseline-esp-can-v0` |
| 目标芯片 | ESP32-S3 |
| ESP-IDF | v5.5（内部版本 5.5.0） |

此基线只冻结改造前原型的可观察行为，不把当前实现认定为产品架构，也不在阶段
0 修复双电机反馈归属、控制任务模型或 BLE 产品协议。

## 固化的当前配置

| 项目 | 当前值 |
|---|---:|
| 固件 Flash 配置 | 2 MB，DIO，40 MHz |
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

## 实测硬件身份

| 项目 | 实测值 |
|---|---|
| 串口 | CH343 `COM5`，`VID_1A86&PID_55D3` |
| USB-UART 序列号 | `5C94152132` |
| CH343 驱动 | wch.cn 2.1.2025.7 |
| 芯片 | ESP32-S3 QFN56 revision v0.2 |
| 晶振 | 40 MHz |
| Wi-Fi MAC | `44:1B:F6:FE:22:C0` |
| BLE MAC | `44:1B:F6:FE:22:C2` |
| 芯片特性 | Wi-Fi、BLE、8 MB embedded PSRAM |
| 实测物理 Flash | 16 MB |

物理 Flash 为 16 MB，但本原型镜像头和分区仍按 2 MB 配置。固件能够正常运行，
只是暂未使用其余容量；这是阶段 0 记录的配置偏差，不在本阶段修改。

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
| `twai_network_master.bin` | 608352 | `0EC1EAA7965F18901E10EC8F2A35D04F0B164E14DA1E3CDCF614B7CE27170A70` |
| `twai_network_master.elf` | 7364532 | `464C893E0CCD675D734711650F9D273CBE1690D96A509CCEE3531F2B29E11DDF` |
| `bootloader.bin` | 21088 | `21A98DCF0DD82575C13ECC9B8796D44C226DF8CC58C1E61D907588DEBAAB0446` |
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
| 源码行为未在阶段 0 重构 | PASS | 仅配置迁移、构建脚本、文档和证据变更 |
| ESP32-S3 烧录和连续启动 | PASS | COM5 烧录并校验；61 秒无异常重启 |
| UART 控制台 | PASS | `help`、`motor_type`、`can_status` 均通过 |
| BLE 广播、连接和命令 | PASS | 同一连接连续三条命令；3.250 秒内重新发现并重连 |
| 两台 RS01 逐台反馈 | PASS | 每台 30 秒各解析 150 个有效样本，5 秒内均出现反馈 |
| 双电机 `imp_start/imp_stop` | PASS | 三轮各运行 10 秒；一轮双侧连续周期拟合及跨轮长覆盖通过 |

整个硬件窗口内 TWAI 始终为 RUNNING；TX/RX error、TX failed、bus error、
arbitration lost 和 RX missed 均为 0。三轮控制中没有缺反馈、BUS_OFF、发送
失败、panic、看门狗或复位。三轮角度跨度均超过 0.4 rad；第 2 轮中 `0x10`
连续段为 2.6 秒/14 点、`0x20` 为 4.8 秒/25 点，共享正弦拟合频率为
0.498 Hz、周期 2.008 秒、左右相位差 178.4°。跨三轮检查还要求每台电机至少
有一轮总覆盖超过 8 秒，结果均通过。最终清理再次收到两台电机
`report 0` 和 `disable` 的命令确认。当前协议没有独立的“电机已失能状态”回读，
因此这里只证明停止及失能命令已发出，不作更强声明。

## 已知原型限制

- `status` 显示的电机编号来自当前选择，但反馈内容来自最后收到的任一电机。
- BLE 自动反馈文本不含源电机 ID；双电机同时上报时不能可靠归属。
- 串口反馈打印的 200 ms 节流由两台电机共用，可能只持续看到其中一台。
- 因上述节流，波形验收采用共享频率拟合，要求至少一轮的两侧均连续覆盖一个
  完整周期，并要求每台电机跨三轮至少有一轮总覆盖 8 秒；不声称每轮、每台
  电机都完成了独立频谱估计。
- `imp_start` 在缺少反馈时仍可能以中心位置 0 启动并返回成功。
- 双电机状态存在无锁读写；100 Hz 控制位于 `esp_timer` 回调中。
- 实测物理 Flash 为 16 MB，当前镜像和分区只配置 2 MB。
- 电机物理左右侧、机械零位和产品安装方向尚未定义；阶段 0 仅以 CAN ID 标识。

这些限制是后续阶段的输入，阶段 0 不修改它们。硬件验收必须按
[SMOKE_TEST.md](SMOKE_TEST.md) 逐台验证，不能以一次 `status` 输出代替。

## 阶段 0 关闭记录

以下关闭条件已于 2026-08-06 全部完成：

1. 对固化提交重新执行主机构建脚本并通过；
2. 完成一次真实烧录，连续运行 60 秒无 panic、assert、看门狗或重启；
3. UART 和 BLE 命令通道通过；
4. CAN 控制器保持 RUNNING，逐台识别 `0x10`、`0x20` 的真实反馈；
5. 双电机阻抗基线连续运行至少 10 秒，成功停止并确认失能命令已发送；
6. 保存可电子读取的设备身份、端口、固件、原始日志哈希和结构化测试结果。

外部供电型号、CAN 收发器型号、终端电阻实物值、电机序列号及左右侧不能由
当前协议电子读取，记为已知元数据缺口；CAN 零错误运行和两台 ID 的真实反馈已
证明当前连接栈可作为阶段 1 之前的功能比较基线。
