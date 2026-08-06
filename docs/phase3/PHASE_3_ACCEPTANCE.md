# 阶段 3 验收：髋关节对象和软件零位

## 当前结论

阶段 3 的代码实现、左右物理映射与方向辨识、实测配置构建烧录、双髋中立位软件归零、BLE 和运动回归均已完成，当前结论为 **PASS**。正式阶段标签为 `phase3-hip-joint-software-zero`。

截至 2026-08-06：

| 验收项 | 状态 | 说明 |
|---|---|---|
| 髋关节类型、配置和静态对象 | PASS | 已实现 LEFT/RIGHT 层及两个静态对象 |
| 软件零位 API 和调试命令 | PASS | 真实左右配置下完成双髋中立位归零，原始角度未改变 |
| ESP-IDF 增量构建 | PASS | 当前工作区执行增量 `idf.py build` 成功 |
| ESP-IDF `fullclean` 构建 | PASS | ESP-IDF v5.5；ESP32-S3；应用分区剩余 41% |
| 当前候选固件 SHA-256 | PASS | `.bin`：`EC975A84496178FCBBEC875E1F8D78BCCD9B6841EEE3D0436A0E3E558BD1DB2F` |
| 左髋物理映射与方向 | PASS | LEFT → B（CAN `0x20`）；屈曲使原始角度增加，方向 `+1` |
| 右髋物理映射与方向 | PASS | RIGHT → A（CAN `0x10`）；屈曲使原始角度减小，方向 `-1` |
| 两个独立对象的软件零位 | PASS | 最终配置下归零与隔离通过；离线拒绝、清除和重启失效此前也已实机验证 |
| UART/BLE/TWAI 与阻抗回归 | PASS | UART、BLE 连接/重连、TWAI 和最终映射固件 3×10 秒阻抗回归通过 |
| 阶段 Git 身份 | PASS | `phase3-hip-joint-software-zero` |

## 当前实现范围

代码已加入：

- `exo_hip_joint_id_t`：LEFT、RIGHT 和固定数量 2；
- 独立 `exo_hip_joint_config_t`；
- `hip_joint.c/.h` 两个静态关节对象；
- 原始执行器量与关节量并存的快照；
- RAM 软件零位建立、清除和零位来源记录；
- `joint_status`、`joint_zero`、`joint_zero_clear` 控制台命令；
- `app_main()` 中的 `hip_joint_init()`；
- 构建清单中的 `hip_joint.c`。

当前 `exo_config.c` 已写入实测映射和方向，软件零位仍保持启动时无效：

```text
LEFT  -> B (CAN 0x20), direction +1, zero INVALID at boot
RIGHT -> A (CAN 0x10), direction -1, zero INVALID at boot
```

上述映射只来自两侧独立运动采集，不由 CAN ID、静止角度或代码顺序猜测。`joint_valid` 仍须等待本次启动后的显式软件归零。

## 静态架构检查

| 检查 | 结果 | 证据或说明 |
|---|---|---|
| A/B 与 LEFT/RIGHT 分层 | PASS | RS01 驱动仍只接收 `exo_actuator_id_t`；人体侧只在髋关节配置和对象中出现 |
| 固定大小对象 | PASS | `s_joints[EXO_HIP_JOINT_COUNT]`，没有动态分配 |
| 配置一致性检查 | PASS | 初始化检查 side、映射、方向、有限零位和左右执行器重复 |
| 坐标变换 | PASS | `q=d(qm-z)`、`dq=d*dqm`、`tau=d*taum`，温度原样传递 |
| 最终有效门控 | PASS | 映射、方向、零位、反馈、在线和有限值必须全部成立 |
| 一致反馈来源 | PASS | 每次关节快照通过一次 `rs01_get_feedback()` 取得完整执行器快照 |
| 软件零位来源可追溯 | PASS | 保存反馈时间戳和反馈计数 |
| 不自动置零 | PASS | 初始化不读取反馈，首帧不会改变零位 |
| 不进行角度包络 | PASS | 直接计算电机角度与软件偏置之差 |
| 软件零位不发 CAN | PASS | `hip_joint.c` 只读取 RS01 反馈，没有调用发送或机械置零接口 |
| RAM-only | PASS | 髋关节模块没有 NVS 或参数保存调用 |
| 左右隔离 | PASS（代码） | 建立或清除零位时只修改指定数组元素 |
| 阶段边界 | PASS | 没有增加控制任务、状态机、产品 BLE 协议或新控制模式 |

当前实现新增一个静态 `portMUX` 保护关节状态，没有新增任务、软件定时器、队列或显式动态内存分配。

## 构建与静态审计

2026-08-06 对写入实测映射 `LEFT→B,+1 / RIGHT→A,-1` 的当前源代码执行了 ESP-IDF v5.5 `fullclean -> build -> size`，结果为 **PASS**。编译和链接过程没有 warning 或 error 诊断。

| 产物 | 字节数 | SHA-256 |
|---|---:|---|
| `build/twai_network_master.bin` | 617,376 | `EC975A84496178FCBBEC875E1F8D78BCCD9B6841EEE3D0436A0E3E558BD1DB2F` |
| `build/twai_network_master.elf` | 7,435,520 | `A4FBAF51738156A23C3E2E3A63A0FD3B6641AC0F2BC51BAD802BB007145E2451` |
| `build/bootloader/bootloader.bin` | 21,088 | `FF55AF1983E71ED41E8C81E2241025415BB0D21F527DF08D5D74904B7EA9B40F` |
| `build/partition_table/partition-table.bin` | 3,072 | `7F00B6C042A89B15B0CAC534F82ED988CAF29278FF5700B0C511EB1B5BB7C820` |

应用固件占用 `0x96ba0` 字节，1 MiB 应用分区剩余 `0x69460` 字节（41%）。内存摘要为 Flash Code 381,742 字节、Flash Data 127,932 字节、DIRAM 99,719 字节。

与阶段 2 基线提交 `614928b7029e37e5504c2afa4465dbffa5f9ef4c` 比较，资源创建调用计数保持不变：

| 调用 | 阶段 2 | 当前 |
|---|---:|---:|
| `xTaskCreate` | 3 | 3 |
| `esp_timer_create` | 1 | 1 |
| `xTimerCreate` | 0 | 0 |
| `xQueueCreate` | 0 | 0 |
| `xSemaphoreCreate*`（各类信号量/互斥量） | 3 | 3 |
| `xStreamBufferCreate` | 1 | 1 |
| 显式堆分配调用 | 0 | 0 |

`hip_joint.c.obj` 的未解析外部符号只有公共配置、`memcpy/memset`、`rs01_get_feedback` 和临界区原语；没有 TWAI 发送、RS01 控制、任务、定时器、队列或堆分配符号。

以上哈希对应完成实物验收时使用的阶段 3 实测映射固件；中立位软件归零、BLE、最终运动回归和阶段 Git 身份均已记录在本文档中。

同一 `.bin` 已成功烧录至 COM5 上的 ESP32-S3，esptool 对 bootloader、应用固件和分区表的写入后哈希校验全部通过。烧录后的映射验收结果为：

- LEFT 显示执行器 B、方向 `+1`，RIGHT 显示执行器 A、方向 `-1`，两者映射和方向均为 valid；
- 两侧反馈均为 valid/online，LEFT 原始位置约 `5.508 rad`，RIGHT 原始位置约 `-6.693 rad`；
- 两侧启动时软件零位均为 `N/A/INVALID`，关节量为 `N/A`、`joint_valid=no`，证明初始化不会自动捕获零位；
- 执行器 A（CAN `0x10`）与 B（CAN `0x20`）均为 `Valid=YES`、`Online=YES`、`Reporting=ON`、`Enabled=no`；
- A 的主动反馈实测 `99.92 Hz`，B 实测 `99.97 Hz`，两者无效帧均为 0；
- TWAI 为 RUNNING，TX/RX error counter、TX failed、bus error、arbitration lost 和 RX missed 均为 0。

## 首次左髋映射采集

已经进行了第一次“移动左髋、同时观察 A/B 原始反馈”的采集尝试：

| 通道 | 有效样本 | 采集窗角度范围 |
|---|---:|---:|
| 执行器 A | 约 150 | `0.000 rad` |
| 执行器 B | 约 150 | `0.000 rad` |

两侧都有持续反馈样本，说明采集期间通信链路存在；但 A/B 的角度范围都为 `0.000 rad`，测试窗中没有可用于辨识映射的运动信号。

因此本次采集的正确结论是：

- **不能判定左髋对应 A 还是 B**；
- 不能由静止角度、CAN ID 或样本数量推断左右；
- 不能判定左髋安装方向；
- 不修改当前 `UNMAPPED/UNKNOWN` 配置；
- 不进行软件零位验收。

本次结果不是左右映射失败，也不证明左髋没有连接到任一执行器；它只说明本次采集缺少足够的物理角度变化。首次采集状态记为 **待重新采集**。

## 第二次左髋映射采集

2026-08-06 在两台电机确认失能、主动反馈开启、右髋保持不动的条件下，进行了 20 秒左髋采集。动作约定为从中立位开始，至少执行 3 次“屈曲→伸展”。

| 通道 | 有效样本 | 基线角度 | 最小值 | 最大值 | 角度范围 |
|---|---:|---:|---:|---:|---:|
| 执行器 A（CAN `0x10`） | 100 | `-8.631 rad` | `-8.631 rad` | `-8.631 rad` | `0.000 rad` |
| 执行器 B（CAN `0x20`） | 100 | `9.025 rad` | `7.202 rad` | `11.339 rad` | `4.137 rad` |

B 的运动范围大于 `0.15 rad`，且远大于 A 的静止范围，满足映射判据。B 第一次超过 `0.10 rad` 的有效偏移为 `+0.629 rad`；按照采集前约定的第一段动作是髋屈曲，可判定屈曲对应 B 原始角度增加。

因此左髋结论为：

```text
LEFT -> EXO_ACTUATOR_B (CAN 0x20)
LEFT direction -> EXO_DIRECTION_POSITIVE (+1)
```

采集结束时 TWAI 保持 RUNNING，TX/RX error、TX failed、bus error、arbitration lost 和 RX missed 均为 0。本次只形成左髋证据；完成右髋独立采集前没有写入半套配置。

## 右髋映射采集

第一次 20 秒窗口中 A、B 各获得 100 个样本，但两者范围均为 `0.000 rad`，因此该窗口判定无效，没有据此推断映射。随后取消前置命令等待，立即执行第二次 30 秒采集；左髋保持不动，右髋从中立位开始完成多次“屈曲→伸展”。

| 通道 | 有效样本 | 基线角度 | 最小值 | 最大值 | 角度范围 |
|---|---:|---:|---:|---:|---:|
| 执行器 A（CAN `0x10`） | 147 | `-5.300 rad` | `-7.139 rad` | `-2.584 rad` | `4.555 rad` |
| 执行器 B（CAN `0x20`） | 147 | `7.204 rad` | `7.204 rad` | `7.204 rad` | `0.000 rad` |

A 的运动范围大于 `0.15 rad`，且远大于 B 的静止范围，满足映射判据。A 第一次超过 `0.10 rad` 的有效偏移为 `-1.1835 rad`；按照采集前约定的第一段动作是髋屈曲，可判定屈曲对应 A 原始角度减小。

因此右髋结论为：

```text
RIGHT -> EXO_ACTUATOR_A (CAN 0x10)
RIGHT direction -> EXO_DIRECTION_NEGATIVE (-1)
```

左右结果互补且一一对应。第二次采集结束时 TWAI 保持 RUNNING，TX/RX error、TX failed、bus error、arbitration lost 和 RX missed 均为 0。完成该证据后，实测映射和方向已写入 `exo_config.c`。

## 映射验收方法

### 采集前状态

1. 两台 RS01 保持主动反馈；
2. 分别选择 A、B 并确认反馈有效、在线；
3. 两台电机保持失能，不发送 `enable`、`move`、`imp_start`；
4. 记录静止噪声范围和 `can_status`；
5. 验收命令中禁止使用 `set_zero` 和 `mit_set_zero`。

### 左髋采集

保持右髋不动，使左髋完成至少 3 次肉眼可见的屈伸。A/B 同时记录原始角度。建议通过标准为：

```text
被移动通道范围 >= max(0.15 rad, 10 * 静止噪声)
被移动通道范围 >= 5 * 另一通道范围
```

若两个通道再次都没有显著变化，继续记为待验，不能猜测映射。

### 右髋采集

保持左髋不动，对右髋重复同样操作。右髋判定结果必须与左髋互补，使 LEFT 和 RIGHT 分别映射到不同执行器。

### 方向

映射确定后，以髋屈曲为正：

- 从中立位到屈曲位，关节角增量应为正；
- 建议有效增量不小于 `max(0.10 rad, 5 * 静止噪声)`；
- 伸展方向应使关节角减小；
- 左右两侧分别验证后才能把 `direction_valid` 设为 true。

上述采集已经完成，真实映射和方向已写入 `exo_config.c`，并已完成重新构建、烧录和软件零位验收。

## 软件零位机制实机测试

为验证代码机制，曾临时使用 `LEFT=A, direction=+1`、`RIGHT=B, direction=-1` 构建测试固件。该配置只是人为构造两个互不相同的关节对象，**不构成真实左右映射或安装方向证据**。机制测试结束后曾恢复为 `UNMAPPED/UNKNOWN`；此后才根据上述左右独立采集写入真实配置。

实机测试结果：

1. 初始两对象均有在线反馈，但 `zero_valid=false`、`joint_valid=false`；
2. A 原始位置约 `-8.908 rad`，B 原始位置约 `+9.142 rad`；
3. `joint_zero left` 后 LEFT 的 `q_joint=0.000 rad`、零位约 `-8.908 rad`、`joint_valid=true`，RIGHT 保持零位无效；
4. `joint_zero right` 后 RIGHT 的 `q_joint≈0.000 rad`、零位约 `+9.142 rad`、`joint_valid=true`；
5. 两侧零位来源的反馈计数和时间戳分别保存，互不覆盖；
6. `direction=-1` 的测试对象对速度和转矩正确反号，温度不反号；
7. `joint_zero_clear left` 只使 LEFT 的零位和关节值失效，RIGHT 仍有效；
8. 重新捕获 LEFT 零位后停止 A 主动上报，超过在线超时再执行 `joint_zero left`，返回 `ESP_ERR_INVALID_STATE`，旧零位及其来源没有被覆盖；
9. 恢复主动上报后 TWAI 保持 RUNNING，TX/RX error、TX failed、bus error、arbitration lost 和 RX missed 均为 0；
10. 重启 ESP32 后，两侧运行时零位均恢复为无效，证明零位是 RAM-only。

这组测试通过了软件零位机制、左右对象隔离、离线拒绝、坐标反号和重启失效。随后又在真实 LEFT/RIGHT 映射和方向配置下完成了中立位归零及完整回归。

## 最终物理配置软件零位验收

对最终映射后的左右两侧分别执行：

1. 零位建立前，反馈在线，但 `zero_valid=false`、`joint_valid=false`；
2. 在定义好的中立姿态记录 raw actuator angle；
3. 执行 `joint_zero left` 或 `joint_zero right`；
4. 要求 `zero_valid=true`、`joint_valid=true` 且 `|q_joint| <= 0.01 rad`；
5. 软件置零前后 raw actuator angle 应连续，变化不超过 `max(0.01 rad, 3 * 静止噪声)`；
6. 一侧置零不得改变另一侧的软件零位、有效标志或原始反馈；
7. `joint_zero_clear` 只使目标侧零位和 `joint_valid` 失效；
8. 关闭目标侧上报并等待超过 100 ms 在线超时后，`joint_zero` 必须拒绝且不能覆盖旧零位；
9. 重新启动 ESP32 后，运行时建立的 RAM 零位应失效。

最终配置烧录后，在双髋中立并保持不动的状态下完成了本轮归零：

| 关节 | 映射 | 归零前 raw | RAM 软件零位 | 归零后 raw | raw 变化 | 最终关节角 | `joint_valid` | 零位来源反馈计数 |
|---|---|---:|---:|---:|---:|---:|---|---:|
| LEFT | B / `+1` | `5.508 rad` | `5.508 rad` | `5.508 rad` | `0.000 rad` | `0.000 rad` | true | 114 |
| RIGHT | A / `-1` | `-6.693 rad` | `-6.693 rad` | `-6.693 rad` | `0.000 rad` | `-0.000 rad` | true | 260 |

LEFT 零位建立后、RIGHT 零位建立前，RIGHT 仍为 `zero_valid=false`、`joint_valid=false`，证明一侧操作没有改变另一侧。LEFT 和 RIGHT 的零位来源时间戳分别为 `1186562 us` 和 `2648521 us`。两侧最终关节角均满足 `|q_joint| <= 0.01 rad`，反馈均 valid/online；TWAI 保持 RUNNING，所有错误计数为 0。

清除、离线拒绝和重启失效已经在临时逻辑映射机制测试中覆盖。随后为执行运动回归重新打开 COM5，USB 串口控制线使 ESP32 重启，LEFT/RIGHT 零位均恢复为无效；静态审计确认阻抗模块没有访问髋关节零位。这再次验证了 RAM-only 重启失效语义，不属于阻抗控制清除零位。所有回归完成后已经通过 BLE 执行下述最终中立位归零，并避免再次复位设备。

所有烧录、串口和运动测试结束后，设备重新回到双髋中立位，并通过 BLE 执行最终运行归零：

| 关节 | 最终 raw | 最终 RAM 软件零位 | 关节角 | `joint_valid` | 零位来源反馈计数 | 零位来源时间戳 |
|---|---:|---:|---:|---|---:|---:|
| LEFT | `5.292 rad` | `5.292 rad` | `0.000 rad` | true | 53,571 | `505002846 us` |
| RIGHT | `-6.310 rad` | `-6.310 rad` | `-0.000 rad` | true | 53,822 | `507646483 us` |

归零前后两侧 raw 变化均为 `0.000 rad`。BLE 主动断开后重新发现并连接设备，两侧零位数值、来源计数和来源时间戳均保持不变，关节角仍满足 `|q_joint| <= 0.01 rad`；TWAI 为 RUNNING、`RX missed=0`。Windows 第一次立即重连曾返回一次 GATT `Unreachable`，第二次自动重试成功，设备没有复位且零位没有丢失。

当前实机保留上述 RAM 零位；后续若复位、断电、烧录或通过会触发复位的串口方式重新连接，必须在中立位再次执行软件归零。

### 不修改机械零位的证据

最终验收必须同时满足：

- 静态调用审计确认 `joint_zero` 只到达 `hip_joint_set_zero_from_current()`；
- `hip_joint.c` 中不存在 `rs01_set_mechanical_zero()`、`rs01_mit_set_zero()` 或 CAN 发送调用；
- 测试命令记录中没有 `set_zero`、`mit_set_zero` 或 `save`；
- 软件置零前后 RS01 raw actuator angle 没有跳变，只有关节坐标归零。

## 整体回归进度

当前实测映射候选固件以及此前未映射调试固件已经完成：

1. ESP-IDF v5.5 `fullclean -> build -> size`；
2. COM5 烧录及写入后哈希校验；
3. 连续运行和测试超过 60 秒，无 panic、assert、watchdog、Brownout 或异常重启；
4. UART：A/B `status`、左右 `joint_status`、左右 `joint_zero` 和 `can_status`；
5. A/B 均保持约 100 Hz 主动反馈且无效帧为 0；
6. 原有 3×10 秒双电机正弦阻抗启停；
7. 三轮结束后 A/B 均为失能状态；
8. TWAI 保持 RUNNING，TX/RX error、TX failed、bus error、arbitration lost 和 RX missed 均为 0。

最终实测映射固件的 3×10 秒回归日志角度范围如下：

| 轮次 | A 样本数 | A 角度范围 | B 样本数 | B 角度范围 |
|---:|---:|---:|---:|---:|
| 1 | 60 | `0.909 rad` | 60 | `0.936 rad` |
| 2 | 61 | `0.888 rad` | 61 | `0.953 rad` |
| 3 | 61 | `0.832 rad` | 61 | `0.934 rad` |

每轮均观察到正常的启动和停止响应，三轮均未出现致命错误标记，结束后 A/B 均为失能状态。该回归仍检查阶段 2 的 A/B 原始执行器路径，不把 RAM 软件零位当作启动阻抗控制的前置条件。

BLE NUS 最终回归结果：

- 扫描发现 `RobStride-CAN`，成功连接并发现 NUS 服务；
- 通过 RX 特征发送 `joint_status left`、`joint_status right` 和 `can_status`，TX Notify 返回完整响应；
- LEFT 映射 B / `+1`、RIGHT 映射 A / `-1` 均读回正确；
- TWAI 为 RUNNING，`RX missed=0`；
- 主动断开后重新扫描、连接并再次执行 `joint_status left` 成功，随后正常断开。

阶段 3 收口时已完成最终差异检查、明确范围的 Git 提交、阶段标签和推送；未把独立的 `web-bluetooth-console/` 工作区纳入 ESP32 阶段提交。

## 关闭条件

阶段 3 只有在以下项目全部通过后才能改为 **PASS**：

- 最终干净构建、尺寸、哈希和提交身份已记录；
- LEFT/RIGHT 与 A/B 的真实一对一映射有有效运动证据；
- 左右两侧髋屈曲都转换为正方向；
- 两侧软件零位、清除、隔离、离线拒绝和重启失效通过；
- 证明软件零位没有改变 RS01 机械零位；
- UART、BLE、TWAI、主动反馈和 3×10 秒阻抗回归通过；
- 验收文档填写最终实测值和已知边界。

上述关闭条件全部满足，阶段 3 状态为 **PASS**。
