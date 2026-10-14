# 2026-10-13 助力中断只读诊断

状态：`CURRENT TARGET CAN_ERROR_PASSIVE STOP CONFIRMED; PHYSICAL ROOT CAUSE NOT CONFIRMED`。
这是一条失败/故障记录，不是阶段10真人助力验收通过记录。

## 本轮操作边界

操作者报告行走过程中先有助力、随后助力消失，并提供角度仍更新、建议与实际附加
助力曲线中断且最新值为“—”的截图。最初Windows仅有COM3/COM4；操作者本轮接上USB后，
重新枚举到COM6（FTDI 0403:6001/A5069RR4A）。

串口open前DTR/RTS=false，115200波特率，仅执行system_status/control_status/
assist_status/sensor_status/gait_status/can_status/ble_status/joint_status。
两轮均finally关闭串口。未发送START/STOP、使能、参数更改、归零、恢复CAN、重启或烧录。
网页BLE连接保持原会话，没有抢占或断开它。

## 当前目标板证据

原始证据目录：`output/phase10/20261013_assist_stops/`。
2026-10-14另将两轮日志按原始字节归档到
[GitHub证据目录](evidence/20261013_assist_stops/)，原文件保留；SHA256见
[归档清单](evidence/ARCHIVE_MANIFEST.json)。

| 字段 | 两轮读取结果/故障瞬间锁存值 |
|---|---|
| MCU Boot ID | 0x7C9317A5，两轮一致 |
| 系统 | ERROR，run_requested=no，output_allowed=no |
| 当前Boot运行故障累计 | 3；仅保留最近一次原因，不能推断前三次均同因 |
| 最近运行故障 | CAN_ERROR_PASSIVE，ESP_ERR_INVALID_STATE |
| 故障时刻 | MCU启动后1202820636 us |
| 故障CAN样本 | TWAI state=1(RUNNING)，TEC=0，REC=133 |
| 故障时双侧反馈年龄 | LEFT 2986 us，RIGHT 4821 us |
| 控制器 | 未armed；运动输出计数31867，两轮不再增加；控制器发送错误0 |
| 助力配置 | selected ANGLE_THRESHOLD/ACTIVE，左右100%，最大助力15.000 N·m |
| 实际活动助力 | NONE/DISABLED，NOT_ARMED，建议无效 |
| 当前CAN | RUNNING，TX4/RX1，1 Mbit/s；TEC=36，REC=0，累计bus_error=855 |
| CAN比较 | 两轮约200秒间隔及每轮两次读取均为bus_error=855、TEC/REC=36/0 |
| 当前反馈 | 双侧valid/online，反馈计数约200秒内各增加约19990，持续约100 Hz |
| 100 Hz任务 | 存活，deadline_miss=0，skipped_cycle=0；算法错误0 |
| BLE/遥测 | 已连接，响应/遥测已订阅，10 Hz，mask=0x007B |

首次诊断控制周期时间为1441009136 us，比锁存故障时间晚约238秒；本轮接USB后主控
没有复位，故障记录仍在。同Boot两轮START/STOP请求仍为5/4，故障累计仍为3。
本轮读取时BLE遥测Notify累计失败继续增加，但不能据此将CAN错误归因于BLE。

最近部署记录确认烧录版本0.10.13。本轮策略元数据version=4与该实现吻合；串口未
直接输出固件版本，BLE被网页占用，因此本轮没有重新GET_INFO，不把先前版本读回
当作此次直接读取。

## 已定位的直接原因

`main/exo_system.c` 的runtime_health检查当前TEC或REC>=128即触发
CAN_ERROR_PASSIVE。latch_runtime_fault立即关闭run_requested/output_allowed，
保存故障瞬间CAN样本，并交给STOP执行器完成清理，最终锁存在ERROR。

因此本次持续停止助力有目标板证据支持：接收错误计数REC=133越过保护门限，导致
整个运动输出关闭。当前反馈和遥测恢复不能自动解除ERROR，也不能自动恢复助力。
这是角度仍更新而助力显示无效的直接解释；不是正常的一次脉冲结束后等待复位。
ANGLE_THRESHOLD策略不要求步态相位/置信度有效，不能仅凭截图相位间断归因于步态门禁。

REC是CAN控制器错误计数，不能把133解释为准确收到133个错误帧。当前bus_error累计855
包含启动以来历史；本轮未增长不代表故障时总线正常，也不代表运动负载下已修复。

## 假设与下一项最小检查

| 假设 | 支持/反证 | 下一项最小检查 | 状态 |
|---|---|---|---|
| 正常脉冲结束后没有重新触发 | 当前系统ERROR且锁存CAN_ERROR_PASSIVE；正常等待复位不会关闭整个系统 | 只有排除CAN故障后的新记录才能评估其他事件问题 | 本次持续中断直接原因已排除 |
| 实时控制任务停摆 | 两轮控制/反馈计数持续增加，100 Hz且无deadline miss | 当前不需要改变任务或控制频率 | 无证据支持 |
| CAN物理链路瞬态错误 | 故障REC=133；故障前后仍有新反馈；停止运动后计数稳定 | 断电后核对线束接头、CAN H/L双绞/端接及共地；再做待机接收记录 | 待确认 |
| 外部供电/地参考或接收逻辑电平问题 | 用户此前报告USB插入后更稳定；本轮没有电压/波形证据 | 测外部主控/CAN模块供电及RXD高电平；确认模块型号和电平适配 | 待确认 |
| 动力工作时电磁干扰 | 运动中故障、无运动诊断期间CAN错误不增长 | 在供电/线束检查完成后，另行安排受控负载记录与波形检查 | 待确认 |

上述检查没有在本轮实际执行。不能直接认定TJ1050模块损坏，也不能通过提高CAN错误
门限或在ERROR后自动重新START来宣称问题已解决。

下一步先保留故障状态排查物理链路和供电，再确认待机反馈持续稳定；重新启动助力前
需要明确的当前现场条件。阶段10整体、真人分级ACTIVE和阶段11仍未完成。
