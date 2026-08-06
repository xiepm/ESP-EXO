# 阶段 0 硬件冒烟测试

本清单只复现当前 `esp_can` 原型。测试结果填入文末记录表，并将完整串口输出
保存为同一基线提交的外部测试附件；不要把自动生成的 `build/` 提交到 Git。

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
3. 向 NUS RX 特征分别写入 `help\r\n` 和 `can_status\r\n`；
4. 断开后等待重新广播，再重复连接。

通过标准：手机收到欢迎信息；连续三次命令无丢失或乱码；串口出现连接、订阅和
断开日志；断开后 5 秒内可重新发现并连接。

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
- `imp_stop` 后出现 stopped/motors disabled，控制帧停止；
- 完整启停重复三次均成功。

## 6. 测试记录

| 字段 | 记录 |
|---|---|
| 日期/测试人 | PENDING |
| Git 提交/标签 | PENDING |
| 开发板型号/序列号 | PENDING |
| 烧录端口/硬件 ID | PENDING |
| CAN 收发器 | PENDING |
| 供电/终端电阻 | PENDING |
| `0x10` 电机序列号及左右侧 | PENDING |
| `0x20` 电机序列号及左右侧 | PENDING |
| 启动 60 秒 | PENDING |
| UART | PENDING |
| BLE | PENDING |
| RS01 `0x10` | PENDING |
| RS01 `0x20` | PENDING |
| 双电机阻抗控制 | PENDING |
| 完整日志位置 | PENDING |
| 总体结论 | PENDING |
