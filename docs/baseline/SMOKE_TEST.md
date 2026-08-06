# 阶段 0 硬件冒烟测试

本清单只复现当前 `esp_can` 原型。阶段 0 的四份冒烟日志、Flash 原始输出、
严格阻抗分析和汇总均已固化在 `docs/baseline/evidence/`。一次性 Python 采集脚本
不属于产品固件，也不在阶段 0 关闭后继续保留；需要重新验收时按本清单操作即可。
不要把自动生成的 `build/` 提交到 Git。

## 1. 端口确认和烧录

重新插入 ESP32-S3 后，先在设备管理器确认新出现的 USB/JTAG 或 USB-UART
端口。COM3、COM4 是本机蓝牙虚拟串口，不能使用。

```powershell
. 'C:\Espressif\tools\Microsoft.v5.5.PowerShell_profile.ps1'
cd D:\ESP32\esp_can
idf.py -p COMx flash monitor
```

启动日志必须包含以下事实，任务调度导致的相邻行顺序差异可以忽略：

```text
RobStride Motor Control - TWAI Master
TX: GPIO4, RX: GPIO2, Baud: 1Mbps
TWAI driver installed (1Mbps)
TWAI driver started
TWAI RX task started
GATT services registered OK
NimBLE host task started
NimBLE host synced, starting advertising
Advertising started as 'RobStride-CAN'
BLE and programming UART consoles initialized
```

连续观察 60 秒。没有 panic、assert、watchdog、反复复位或持续错误日志才通过。

## 2. UART 控制台

依次执行：

```text
help
motor_type
can_status
```

通过标准：帮助完整；`MOTOR_TYPE=5`；TWAI 状态为 RUNNING；未发送 CAN 时错误
计数不持续增长。

## 3. BLE 文本通道

1. 手机发现并连接 `RobStride-CAN`；
2. 订阅 NUS TX 特征通知；
3. 在同一连接中向 NUS RX 依次写入 `help\r\n`、`motor_type\r\n` 和
   `can_status\r\n`；
4. 主动断开，5 秒内重新发现并连接，再执行一次 `can_status\r\n`。

通过标准：两次连接均收到欢迎信息；NUS 服务、RX Write 和 TX Notify 属性正确；
第一次连接连续三条命令均完整返回；断开后 5 秒内可重新发现、连接并完成命令。

## 4. 两台 RS01 逐台反馈

先只验证 `0x10`：

```text
motor 0x10
report 1
can_status
report 0
```

再只验证 `0x20`：

```text
motor 0x20
report 1
can_status
report 0
```

每台观察至少 30 秒。通过标准：5 秒内出现对应 `id=16` 或 `id=32` 的反馈；
角度、速度、力矩和温度是有限合理值；TWAI 保持 RUNNING；观察窗口内 TX/RX
error、TX failed、bus error 和 RX missed 不持续增加；不进入 BUS_OFF。

必须逐台测试。当前原型的 `status` 快照和 BLE 自动反馈无法在双上报时可靠区分
反馈来源。

## 5. 双电机阻抗基线

两台电机均已逐台通过后执行：

```text
motor 0x10
report 1
motor 0x20
report 1
imp_start
can_status
imp_stop
```

通过标准：

- 日志中没有 `No feedback from motor`；
- 打印两台电机的中心位置以及 `Impedance control running @ 100 Hz`；
- 连续运行至少 10 秒，两侧为反相、周期约 2 秒；
- 没有 TX failed、BUS_OFF 或持续增长的错误计数；
- `imp_stop` 后出现停止日志并收到命令确认；当前测试没有用外部 CAN 分析仪独立
  验证总线控制帧停止或执行器物理失能状态；
- 完整启停重复三次均成功。

自动判定还要求：每轮两个 CAN ID 都有带时间戳反馈且角度跨度至少 0.1 rad；
至少一轮的每侧均有至少 10 点、最长连续段至少 2.2 秒，共享频率拟合为
0.45–0.55 Hz、周期 1.8–2.2 秒、振幅至少 0.1 rad、拟合 `R²` 至少 0.65，
左右相位差至少 140°；跨三轮还要求每台电机至少有一轮总时间覆盖 8 秒。
这是为了适配原型固件对双电机共用 200 ms 日志节流；它验证共享参考下的反相
响应，不宣称每侧都独立完成频谱辨识。最终以严格重放分析 JSON 为准。

## 6. 测试记录

| 字段 | 记录 |
|---|---|
| 日期/测试人 | 2026-08-06 / Codex 自动验收 |
| Git 提交/标签 | 固件源码 `2340886` / `baseline-esp-can-v0` |
| 开发板型号/序列号 | ESP32-S3 QFN56 rev0.2；USB-UART S/N `5C94152132` |
| 烧录端口/硬件 ID | COM5 / `USB\VID_1A86&PID_55D3\5C94152132` |
| CAN 收发器 | 型号无法由当前硬件电子读取；1 Mbps 实测通过 |
| 供电/终端电阻 | 实物元数据未提供；全测试窗口 CAN 错误为 0 |
| `0x10` 电机序列号及左右侧 | 序列号/左右侧未定义；CAN ID `0x10` 验证通过 |
| `0x20` 电机序列号及左右侧 | 序列号/左右侧未定义；CAN ID `0x20` 验证通过 |
| 启动 60 秒 | PASS；实际观察 61 秒，无异常重启 |
| UART | PASS；`help`、`motor_type`、`can_status` |
| BLE | PASS；连续三条命令，3.250 秒内重新发现并重连 |
| RS01 `0x10` | PASS；30 秒解析 150 个有效样本 |
| RS01 `0x20` | PASS；30 秒解析 150 个有效样本 |
| 双电机阻抗控制 | PASS；三轮各 10 秒；一轮双侧完整周期拟合及跨轮长覆盖通过；停止/失能命令已确认 |
| 完整结果 | `evidence/hardware_smoke_20260806.json`、`evidence/impedance_analysis_20260806.json` |
| 总体结论 | `verified_match_with_recorded_caveats` |
