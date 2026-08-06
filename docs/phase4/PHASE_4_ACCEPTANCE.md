# 阶段 4 验收：唯一 100 Hz 控制任务

## 最终结论

阶段 4 于 2026-08-06 完成，结论为 **PASS**。阶段基线为
`phase3-hip-joint-software-zero`（`7e6f96d0a9e7af7dd2504f40aeec308dee3c2dd1`），
最终实现代码提交为 `fb7678a29bff8fa5a0612edb27e5676eb343d23c`，正式标签为
`phase4-single-100hz-control-task`。

本阶段只收敛实时调度归属：系统状态机和产品 START/STOP 属于阶段 5，产品 BLE
协议属于阶段 6，正弦参考和阻抗控制的 LEFT/RIGHT 正式迁移属于阶段 7。

## 验收矩阵

| 验收项 | 状态 | 证据 |
|---|---|---|
| 唯一静态 100 Hz 控制任务 | PASS | 静态任务、4096-byte 静态栈、Core 1、优先级 9；任务身份始终为 `0x3fc9ff94` |
| LEFT/RIGHT 同周期读取 | PASS | 一个控制周期内顺序读取；两侧反馈计数和时间戳持续更新 |
| 旧周期 `esp_timer` 移除 | PASS | `impedance_control.c` 无周期定时器；唯一周期调用来自控制任务 |
| 开机默认无运动输出 | PASS | 烧录启动后任务持续运行，旧阻抗为 inactive，电机未被自动使能 |
| 空闲时序 | PASS | 隔离 BLE/UART 后连续观察超过 5 分钟，33,291 个周期样本全部位于 9-11 ms |
| 旧阻抗时序及 3x10 s 回归 | PASS | 三轮均完成 START、至少 10 秒运动和 STOP，运动期间实时计数无错误 |
| BLE/UART/TWAI/主动反馈 | PASS | BLE 首连、断开和重连通过；TWAI RUNNING，错误和 RX missed 均为 0 |
| ESP-IDF 构建、尺寸和烧录 | PASS | ESP-IDF v5.5 干净构建和 COM5 烧录通过，产物哈希见下表 |

## 时序标准解释

两个工况使用相同的周期质量门槛，但观察时长按阶段 4 设计和功能回归标准分别为：

- 空闲：连续观察至少 5 分钟；
- 旧阻抗运动：连续 3 轮，每轮至少 10 秒，每轮独立 START/STOP。

| 指标 | 通过标准 |
|---|---:|
| 平均周期 | `10.0 ms +/- 0.1 ms` |
| 目标范围内周期 | 至少 99% 位于 `9-11 ms` |
| 最大周期误差（包含 BLE 建链/重连） | 不超过 `2 ms` |
| 最大单周期执行时间 | 小于 `10 ms` |
| Deadline miss | `0` |
| 快速补跑/CAN 突发 | 不出现 |
| 静态任务栈最低余量 | 不低于初始栈的 25% |

阶段 4 没有放宽周期误差、执行时间或错误计数门槛。`control_status` 是按需诊断，
不属于阶段 10 连续遥测。LEFT/RIGHT 各自保留反馈来源时间戳，本阶段不声称两台
电机是严格同步采样。

## 静态架构审计

- `exo_control_loop.c` 保存一个静态 TCB 和一块 4096-byte 静态栈，只调用一次
  `xTaskCreateStaticPinnedToCore()`；
- 唯一应用级周期等待是控制任务中的 `xTaskDelayUntil()`；控制任务上电后永久运行；
- 每周期调用链为
  `exo_control_loop -> impedance_control_process_tick -> rs01_try_send_motion`；
- `impedance_control.c` 不再创建或启动周期 `esp_timer`，`imp_start/imp_stop` 不创建、
  删除或重启控制任务；
- 新控制层只表达 LEFT/RIGHT，不包含 `EXO_ACTUATOR_A/B`、CAN ID 或直接 CAN 发送；
- 实时运动发送的 TX mutex 和 `twai_transmit()` 都是零等待，失败只计入诊断且不在
  实时路径打印日志；A/B 两帧始终分别尝试；
- 控制周期没有日志、动态分配、阻塞延时、BLE Notify、模式切换或软件归零；
- 控制诊断由任务私有状态形成完整快照，再在短临界区一次发布；
- 控制任务固定 Core 1 / priority 9；事件驱动 TWAI RX 固定 Core 1 / priority 8；
  NimBLE、BLE 命令和 UART 命令固定 Core 0；
- `app_main()` 只按顺序初始化 RS01、髋关节、TWAI、主动上报、控制台和控制任务，
  然后返回；
- LEFT 保持执行器 B / `+1`，RIGHT 保持执行器 A / `-1`；髋关节对象、反馈解析和
  RAM 软件零位实现未改变；
- 未提前加入阶段 5-10 的产品状态机、产品 BLE 协议、五元命令、步态估计、相位
  曲线或连续遥测。

实时路径和阶段范围的独立静态审计结论均为 PASS，无必须修复项。保留边界是两帧
发送并非事务原子，阶段 5 由状态机禁止运行时维护命令插入。

## 实现收敛记录

| 提交 | 内容 |
|---|---|
| `30bf823` | 建立唯一静态 100 Hz 控制任务并移除旧周期定时器 |
| `53ff58c` | 把控制任务创建移到所有重量级子系统初始化之后 |
| `a5afbfa` | 使 TWAI 主动反馈在 BLE 活动期间保持响应 |
| `495f7b3` | 把 RTOS tick 从 100 Hz 提高到 1 kHz，控制频率仍为 100 Hz |
| `fb7678a` | 将 BLE/UART 通信固定到 Core 0，与 Core 1 控制和反馈路径隔离 |

早期实测分别发现过约 5 ms 唤醒相位误差、BLE 第二次重连造成约 126 ms 控制暂停，
以及控制台日志压力下 `RX missed=10`。上述问题分别由 1 kHz RTOS tick、控制台
Core 0 固定，以及 TWAI RX Core 1 固定并移除正常反馈热路径日志解决。最终验收没有
沿用这些失败数据，也没有放宽 2 ms 周期误差上限。

## 构建与烧录证据

最终实机固件由提交 `fb7678a` 在 ESP-IDF v5.5（内部版本 5.5.0）下干净构建，
随后烧录到 ESP32-S3（CH343，COM5）。应用分区余量约 41%。

| 产物 | 大小 | SHA-256 |
|---|---:|---|
| `twai_network_master.bin` | 620,240 B | `D1BA25EEEE30755E8596F6D141E5C8D867BC8CFCE934FB57F43DCED3B6B85A97` |
| `twai_network_master.elf` | 7,454,220 B | `9411F39AC5D43F2B1A69C9533102C878186337B599A2AE1F31BAE69F77FDCBEB` |
| `bootloader.bin` | 21,088 B | `08653DE915BD8EBE9299E991A0B87F927DEE224768E92B8014F6555CE55E58F2` |
| `partition-table.bin` | 3,072 B | `7F00B6C042A89B15B0CAC534F82ED988CAF29278FF5700B0C511EB1B5BB7C820` |

## 空闲、BLE 和 TWAI 实机证据

烧录后首先在软件零位无效且旧阻抗 inactive 的状态下观察，控制任务没有自动使能
电机或产生运动。最终空闲窗口中 BLE 和 UART 均断开，随后再分别通过 UART 和 BLE
读取诊断：

| 指标 | 5 分钟空闲结果 | BLE 重连后结果 |
|---|---:|---:|
| 任务身份 | `0x3fc9ff94` | `0x3fc9ff94` |
| 周期样本 | 33,291 | 34,608 |
| 平均周期 | 10,000 us（100.000 Hz） | 10,000 us（100.000 Hz） |
| 最小 / 最大周期 | 9,501 / 10,051 us | 9,501 / 10,051 us |
| 最大周期误差 | 499 us | 499 us |
| 超出 9-11 ms | 0 | 0 |
| 最大执行时间 | 407 us | 407 us |
| Deadline miss / 跳周期 / 输出错误 | `0 / 0 / 0` | `0 / 0 / 0` |
| 最低栈余量 | 2,780 B | 2,780 B |

BLE 首次连接、断开、隔离空闲和再次连接期间任务身份不变，周期计数持续增加。最终
TWAI 始终为 RUNNING；TX/RX error counter、TX failed、bus error、arbitration lost 和
RX missed 均为 0。A/B 主动反馈有效、在线且约为 100 Hz。运动回归脚本的 Windows
GATT 服务发现曾在发送命令前返回 `Unreachable`，重新扫描后第二次连接成功；该过程
没有向设备发送 START，也没有改变控制任务身份。这属于阶段 6 产品 BLE 前的主机侧
重试边界，不作为一次运动失败记录。

## BLE 软件归零

烧录会清除阶段 3 的 RAM 软件零位。双髋回到中立位后，经 BLE RX 特征依次执行
`joint_zero left` 和 `joint_zero right`：

| 关节 | 软件零位 | 零位来源 | 归零后关节角 |
|---|---:|---|---:|
| LEFT（执行器 B，`+1`） | `+5.292 rad` | feedback count 51,317；timestamp 513,589,593 us | 约 `0 rad` |
| RIGHT（执行器 A，`-1`） | `-6.310 rad` | feedback count 51,452；timestamp 514,967,299 us | 约 `0 rad` |

两侧 `Joint valid` 均为 YES。BLE 连接和归零后周期样本 51,893，最大周期误差
604 us，deadline miss、跳周期、输出错误和全部 TWAI 错误仍为 0。

## 3x10 s 旧阻抗回归

每轮通过 BLE 独立执行 `imp_start`，运动至少 10 秒后执行 `imp_stop`。运动期间交替
读取左右髋关节状态；由于这是按需文本诊断，采样用于确认运动范围，不作为阶段 10
连续遥测。三轮结果为：

| 轮次 | 运动时间 | LEFT 最小 / 最大 / 范围 | RIGHT 最小 / 最大 / 范围 | START/STOP |
|---|---:|---:|---:|---|
| 1 | 10.016 s | `-0.511 / +0.432 / 0.943 rad` | `-0.405 / +0.392 / 0.797 rad` | PASS |
| 2 | 10.078 s | `-0.340 / +0.534 / 0.874 rad` | `-0.285 / +0.517 / 0.802 rad` | PASS |
| 3 | 10.359 s | `-0.711 / +0.198 / 0.909 rad` | `-0.583 / +0.259 / 0.842 rad` | PASS |

三轮结束后的累计控制诊断：

| 指标 | 最终值 |
|---|---:|
| 任务身份 | `0x3fc9ff94` |
| 周期样本 | 81,874 |
| 平均周期 | 10,000 us（100.000 Hz） |
| 最小 / 最大周期 | 9,375 / 10,579 us |
| 最大周期误差 | 625 us |
| 超出 9-11 ms | 0 |
| 最大执行时间 | 1,272 us |
| Deadline miss / 跳周期 / 输出错误 | `0 / 0 / 0` |
| 最低栈余量 | 2,604 B（63.6%） |

每轮停止命令均返回成功；最终两台电机都是 `Enabled: no (requested)`、`REST`，
fault raw 和 warning raw 均为 0。最终温度为 31.9-32.6 C，LEFT/RIGHT 软件零位继续
有效。TWAI 仍为 RUNNING，全部错误计数和 RX missed 均为 0；没有 panic、assert、
watchdog、brownout 或异常重启。

## 阶段边界

- 软件零位只保存在 RAM；复位、掉电或重新烧录后必须在双髋中立位重新归零；
- 旧阻抗兼容模块内部暂时仍用 A/B，阶段 7 才迁移到 LEFT/RIGHT 关节命令；
- 阶段 4 的 `imp_start/imp_stop` 是调试控制台兼容命令，不是阶段 5 产品状态机接口；
- `web-bluetooth-console/` 是用户未跟踪目录，不属于阶段 4，也没有纳入提交。
