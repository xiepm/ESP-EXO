# 阶段 5 验收：系统状态机和 START/STOP 执行器

## 当前结论

**IN PROGRESS**

阶段基线为 `phase4-single-100hz-control-task`
（`10007b07327984d266142eaa2b8eb74cf999cc29`）。只有在静态审计、ESP-IDF v5.5
干净构建、烧录、无运动验收及经用户确认后的运动验收全部完成，并把实际数据填写到
本文后，才可以将结论改为 `PASS`。

| 记录项 | 值 |
|---|---|
| 阶段 5 实现提交 | `c152b02c50534314c05d0afb8d69fbffab633355` |
| 最终验收提交 | 待填写 |
| 正式标签 | 计划为 `phase5-system-state-start-stop` |
| ESP-IDF 版本 | `v5.5` |
| 目标芯片 | ESP32-S3 |
| 烧录端口/设备 | `COM5`，USB-Enhanced-SERIAL CH343，序列号 `5C94152132` |
| 实机验收日期 | 2026-08-06（静态、构建和无运动部分） |
| 运动前中立位确认 | **尚未确认；未确认前禁止开始运动验收** |

## 验收矩阵

`PENDING` 表示尚无足够证据，不得按设计预期提前填写为通过。

| 类别 | 验收项 | 状态 | 证据/实际结果 |
|---|---|---|---|
| 静态 | 六态模型唯一，READY 为派生值 | PASS | `exo_system.h:13-20`；就绪值由 `exo_system.c:812-875` 派生，不是第七态 |
| 静态 | 唯一永久 100 Hz 控制任务保持不变 | PASS | 全仓只有 `exo_control_loop.c:159` 一个 `xTaskDelayUntil()` |
| 静态 | 执行器为 4096-byte 静态栈、Core 0 / priority 6 事件任务 | PASS | `exo_system.c:16-18,33-38,565-640`；ELF 栈符号为 `0x1000` bytes |
| 静态 | START/STOP 请求无 CAN、延时和反馈等待 | PASS | 请求入口 `exo_system.c:687-800` 只读快照、改状态并通知执行器 |
| 静态 | 只有系统 RUNNING 且输出许可开启可发送周期运动帧 | PASS | `exo_control_loop.c:170 -> exo_system.c:803-809 -> impedance_control.c:60-102` |
| 静态 | STOP 在请求入口立即撤销输出许可 | PASS | `exo_system.c:772-788` 在通知执行器前清除目标和输出门 |
| 静态 | STARTING 可被 STOP 取消 | PASS | STOP 原子转入 STOPPING；启动等待均可由 notification 唤醒并复核目标 |
| 静态 | 维护操作和执行器使用同一操作所有权 | PASS | 同一静态 mutex 用于执行器和 `exo_system_try_begin_maintenance()` |
| 静态 | BLE/UART 不再直接启停阻抗控制器 | PASS | 四个启停文本命令均只调用 `exo_system_request_start/stop()` |
| 静态 | 阻抗模块只保留 arm/disarm/tick，算法不变 | PASS | 公开接口已收缩；KP/KD、频率、幅值、渐入和 A/B 相位未改 |
| 静态 | 系统快照一次一致性复制 | PASS | 私有状态在同一临界区更新，`exo_system_get_snapshot()` 在临界区一次复制 |
| 边界 | 未提前加入阶段 6-10 功能 | PASS | 无产品 BLE 帧、五元命令、步态估计、相位曲线或连续遥测 |
| 构建 | `git diff --check` 通过 | PASS | 命令退出码 0，无 whitespace error |
| 构建 | ESP-IDF v5.5 `fullclean build` 通过 | PASS | 连续两次干净构建通过；`.bin` 626,384 bytes，应用分区余 40% |
| 无运动 | 上电后 INIT 收敛至 IDLE，输出关闭 | PASS | `Transitions=1`，稳定态 IDLE，run/output 均 no；双电机 REST 且未使能 |
| 无运动 | 未归零时 START 被拒绝且不操作电机 | PASS | `LEFT_ZERO_INVALID`、`REJECTED_NOT_READY`；成功/失败/取消及转换计数不变 |
| 无运动 | 只归零一侧时 START 仍被拒绝 | PASS | 临时归零 LEFT 后为 `RIGHT_ZERO_INVALID`，请求仍被拒绝，随后已清除零位 |
| 无运动 | 双侧归零后 `ready_to_start=true` | PENDING | 待填写软件零位和系统快照 |
| 无运动 | IDLE 中重复 STOP 不执行硬件序列 | PASS | STOP request `0 -> 2`，success/failure 均 0，transition 保持 1 |
| 无运动 | BLE 断开重连不创建任务、不改变目标 | PASS | 两次连接后任务 identity、IDLE、run/output 和 transition 均不变 |
| 运动 | `IDLE -> STARTING -> RUNNING` 顺序正确 | PENDING | 需用户中立位确认后填写 |
| 运动 | 连续 START 只执行一次启动序列 | PENDING | 待填写计数和轨迹连续性 |
| 运动 | RUNNING 中 START 不重置轨迹 | PENDING | 待填写开始时间/中心/计数证据 |
| 运动 | RUNNING 中维护命令被拒绝 | PENDING | 待填写代表性命令及响应 |
| 运动 | STARTING 中 STOP 最终不进入 RUNNING | PENDING | 待填写状态序列和最终电机状态 |
| 运动 | STOP 最迟下一控制周期关闭输出 | PENDING | 待填写请求时间和控制周期证据 |
| 运动 | `RUNNING -> STOPPING -> IDLE` 顺序正确 | PENDING | 待填写状态序列 |
| 运动 | 三轮独立运行，每轮至少 10 秒 | PENDING | 待填写三轮数据表 |
| 运动 | 全程控制任务身份稳定 | PENDING | 待填写验收前后 task identity |
| 运动 | 最大周期误差不超过 2 ms | PENDING | 待填写 `control_status` |
| 运动 | deadline/skipped/output error 均为 0 | PENDING | 待填写累计计数 |
| 运动 | TWAI 错误和 RX missed 不增长 | PENDING | 待填写前后 `can_status` |
| 运动 | 最终双电机 REST、失能、零故障 | PENDING | 待填写两侧最终状态 |
| 运动 | LEFT/RIGHT RAM 软件零位仍有效 | PENDING | 待填写最终 `joint_status` |

## 静态审计清单

### 状态和请求

- [x] 仅存在 `INIT/IDLE/STARTING/RUNNING/STOPPING/ERROR` 六个整机状态；
- [x] `ready_to_start` 只在 `IDLE` 且无维护操作时，从控制任务、TWAI、双侧
  `joint_valid` 和电机故障状态派生；
- [x] 系统 `RUNNING` 没有与控制任务存活、TWAI RUNNING、RS01 模式或阻抗内部状态
  混用；
- [x] 请求返回值能够区分接受、已满足、已等待、未就绪、忙、错误和内部错误；
- [x] 请求接受与执行完成分别记录；
- [x] STOPPING 中 START 被拒绝，不保存自动重启请求；
- [x] ERROR 中 START 被拒绝，STOP 可以重新尝试清理；
- [x] 重复 START/STOP 不重复执行硬件序列或成功计数。

### 任务和实时路径

- [x] `exo_control_loop` 仍是唯一应用级 100 Hz 周期源；
- [x] 控制任务仍为静态 Core 1 / priority 9 且上电只创建一次；
- [x] 系统执行器使用 4096-byte 静态栈、Core 0 / priority 6，且上电只创建一次；
- [x] 执行器无请求时阻塞等待 notification，不创建周期 `esp_timer` 或进行空闲轮询；
- [x] START/STOP 不创建、删除、暂停或重启控制任务；
- [x] 控制 tick 不等待操作 mutex，不含日志、BLE Notify、动态分配、阻塞 CAN、模式
  切换或软件归零；
- [x] 周期调用链只有
  `exo_control_loop -> exo_system_process_control_tick -> impedance tick -> try send`；
- [x] STOP 请求先关闭 `output_allowed`，再通知执行器清理；
- [x] disarm 能等待已经进入的 tick 退出；
- [x] 系统私有状态在同一临界区更新，诊断读取在该临界区一次复制完整快照。

### 操作隔离和阶段边界

- [x] 执行器和维护命令使用同一个静态操作 mutex/所有权；
- [x] `maintenance_active` 与状态在同一临界区检查，不能发生“先检查、后插入 CAN”；
- [x] 使能、失能、运动、电流、模式、参数写入、机械零位和 RAM 软件零位命令均被
  操作作用域保护；
- [x] 活动状态只允许只读诊断或无硬件副作用的本地命令；
- [x] `start/stop` 和兼容 `imp_start/imp_stop` 都只调用系统请求接口；
- [x] 阻抗算法的频率、幅值、渐入、KP/KD 和相位关系未改变；
- [x] LEFT 仍为 B / `+1`，RIGHT 仍为 A / `-1`；
- [x] 没有产品 BLE 帧、五元命令、新控制模式、步态估计、相位曲线或连续遥测。

## 构建和产物证据

使用 ESP-IDF v5.5 执行：

```powershell
. 'C:\Espressif\tools\Microsoft.v5.5.PowerShell_profile.ps1'
cd D:\ESP32\esp_can
idf.py fullclean
idf.py build
```

| 项目 | 实际值 |
|---|---|
| `git diff --check` | PASS（退出码 0） |
| 构建结论 | ESP-IDF v5.5 `fullclean -> build -> size` PASS |
| 应用分区使用/余量 | `0x98ed0 / 0x100000`；余 `0x67130` bytes（40%） |
| warnings/errors | 编译 warning 0，error 0 |

| 产物 | 大小 | SHA-256 |
|---|---:|---|
| 应用 `.bin` | 626,384 bytes | `31A5F96C4976250278BAD0FA14B65470C9A193A7C46BBE21A532A9FD947A4C83` |
| 应用 `.elf` | 7,498,544 bytes | `ADE352A6F3E10A6500C278B4E7C679DB6B2C57618ADE1E1F2FE0DB4628553C10` |
| `bootloader.bin` | 21,088 bytes | `8CD6BB7AC8D5A0820E87FC58877DB5326989D534F9BF57696A7503D65F89A1C9` |
| `partition-table.bin` | 3,072 bytes | `7F00B6C042A89B15B0CAC534F82ED988CAF29278FF5700B0C511EB1B5BB7C820` |

## 无运动实机验收

无运动测试可以在烧录后进行，但不得执行 START 成功路径。

### 上电和任务身份

| 指标 | 实际值 |
|---|---|
| 初始/稳定系统状态 | `INIT -> IDLE`，`Transitions=1`，active step `NONE` |
| `run_requested` / `output_allowed` | `no / no` |
| 控制任务 identity | `0x3fc9ffa4`，BLE 重连前后不变 |
| 执行器任务 identity | `0x3fca11cc`，BLE 重连前后不变 |
| 双电机使能/模式 | A/B 均 `Enabled: no`、`Mode status: 0 (REST)`、fault `0x00000000` |

期望结果：初始化从 `INIT` 收敛至 `IDLE`；输出许可保持关闭；两台电机不因系统模块
初始化而自动使能或运动。

### START 前置条件

| 工况 | `ready_to_start` | START 预期 | 实际结果 | 电机状态是否未变 |
|---|---:|---|---|---|
| LEFT/RIGHT 均未归零 | false | `REJECTED_NOT_READY` | 实际 `LEFT_ZERO_INVALID`，请求被拒绝 | 是，A/B 均保持 REST/未使能 |
| 只归零 LEFT | false | `REJECTED_NOT_READY` | 实际 `RIGHT_ZERO_INVALID`，请求被拒绝 | 是，随后已清除临时 LEFT 零位 |
| LEFT/RIGHT 均归零且其他条件正常 | true | 本项只确认就绪，不在此处发送 START | 待填写 | 是 |

软件归零值和来源：

| 关节 | 软件零位 | feedback count | timestamp | `joint_valid` |
|---|---:|---:|---:|---|
| LEFT（B，`+1`） | 临时门控值 `4.700 rad`，已清除 | `850` | `8603071 us` | 临时为 YES，最终为 no |
| RIGHT（A，`-1`） | 待填写 | 待填写 | 待填写 | 待填写 |

LEFT 的值只用于验证单侧门控，不代表中立位标定；测试后已执行 `joint_zero_clear left`。
最终 LEFT/RIGHT 均保持零位无效，等待用户确认双髋中立位后重新正式归零。

### 幂等、维护和 BLE 重连

| 测试 | 请求/状态计数（前） | 操作 | 请求/状态计数（后） | 结论 |
|---|---|---|---|---|
| IDLE 重复 STOP | stop req/success/failure=`0/0/0`，transition=1 | 连续发送 STOP | `2/0/0`，transition=1 | PASS，两次均 `ALREADY_SATISFIED` |
| IDLE 维护作用域 | LEFT zero invalid | 临时 `joint_zero left`，随后 clear | 维护结束后 inactive，LEFT 再次 invalid | PASS，操作作用域正常释放 |
| BLE 断开重连 | control/executor=`0x3fc9ffa4/0x3fca11cc`，IDLE，transition=1 | 两次连接 `44:1B:F6:FE:22:C2` 并查询状态 | identity/state/target/transition 均不变；cycle `1078 -> 3714` | PASS |

BLE 重连前后必须比较控制任务 identity、执行器任务 identity、`run_requested`、
`output_allowed` 和状态转换计数，不能只检查命令仍能响应。

最终无运动会话中，START request/rejected=`2/2`，start success/failure/cancel=`0/0/0`；
TWAI 始终 RUNNING，TX/RX error、TX failed、bus error、arbitration lost 和 RX missed
均为 0。最终控制周期均值 9,999 us（100.010 Hz），最大周期误差 499 us，deadline/skipped/output error
均为 0。最终 A/B 仍为 REST、未使能、零故障，LEFT/RIGHT RAM 软件零位均无效。

已知调试边界：IDLE 下的维护控制台仍允许人工执行 `enable/move` 等底层命令，因此
逻辑 IDLE 本身不能替代电机 REST 反馈确认。阶段 6 产品协议只能映射系统请求接口，
不得复用这条调试旁路。

## 运动验收门禁

真正开始任何可能使能电机的测试前，必须满足并记录：

- [ ] 用户已明确确认“双髋已在中立位，可以归零/运动”；
- [ ] LEFT/RIGHT 已在本次烧录后重新建立 RAM 软件零位；
- [ ] `system_status` 显示 `IDLE`、`ready_to_start=true`；
- [ ] `control_status` 无既有 deadline/skipped/output error；
- [ ] `can_status` 为 RUNNING，错误计数已记录为运动前基准；
- [ ] 双侧反馈有效、在线且无当前故障；
- [ ] 测试脚本或人工流程具有 `finally -> stop` 清理路径。

用户中立位确认原文和时间：**待填写**。

在上述门禁完成前，本节所有项目保持 `PENDING`，不得以静态分析替代实机运动证据。

## 状态与竞争回归

| 编号 | 操作 | 预期状态/结果 | 实际状态序列和计数 | 结论 |
|---|---|---|---|---|
| S1 | IDLE 中发送 START | `IDLE -> STARTING -> RUNNING` | 待填写 | 待填写 |
| S2 | STARTING 中再次 START | `ALREADY_PENDING`，启动序列只执行一次 | 待填写 | 待填写 |
| S3 | RUNNING 中再次 START | `ALREADY_SATISFIED`，轨迹不重置 | 待填写 | 待填写 |
| S4 | RUNNING 中发送维护运动/模式命令 | 命令被拒绝，无维护 CAN 插入 | 待填写 | 待填写 |
| S5 | RUNNING 中发送 STOP | 立即关门，`RUNNING -> STOPPING -> IDLE` | 待填写 | 待填写 |
| S6 | STARTING 后立即发送 STOP | 最终 `IDLE`，过程中不得发布 RUNNING | 待填写 | 待填写 |
| S7 | STOPPING 中再次 STOP | `ALREADY_PENDING`，停止序列只执行一次 | 待填写 | 待填写 |
| S8 | STOPPING 中发送 START | `REJECTED_BUSY`，停止后不自动重启 | 待填写 | 待填写 |

S6 应在独立一轮、确认电机处于中立位后执行。记录状态转换历史、启动成功/取消/停止
计数以及最终两台电机状态，不能只依据最后看到 `IDLE` 判定通过。

## 三轮 START/RUN/STOP 回归

每轮都从 `IDLE + ready_to_start=true` 开始，独立发送 START，确认进入 RUNNING 后
运行至少 10 秒，再发送 STOP 并等待回到 IDLE。调试文本关节采样只用于确认产生了
合理运动，不冒充阶段 10 连续遥测。

| 轮次 | START 结果 | RUNNING 持续时间 | LEFT 最小/最大/范围 | RIGHT 最小/最大/范围 | STOP 结果 | 最终状态 |
|---|---|---:|---:|---:|---|---|
| 1 | 待填写 | 待填写 | 待填写 | 待填写 | 待填写 | 待填写 |
| 2 | 待填写 | 待填写 | 待填写 | 待填写 | 待填写 | 待填写 |
| 3 | 待填写 | 待填写 | 待填写 | 待填写 | 待填写 | 待填写 |

至少在一轮 RUNNING 中执行重复 START 和代表性的维护命令拒绝测试。不要通过修改旧
阻抗参数、缩短 10 秒要求或放宽时序门槛来使测试通过。

## 实时和总线判据

阶段 5 沿用阶段 4 的实时质量门槛：

| 指标 | 通过标准 | 运动前 | 三轮结束后 | 增量/结论 |
|---|---:|---:|---:|---|
| 控制频率/平均周期 | 100 Hz / `10.0 ms +/- 0.1 ms` | 待填写 | 待填写 | 待填写 |
| 最大周期误差 | `<= 2 ms` | 待填写 | 待填写 | 待填写 |
| 最大单周期执行时间 | `< 10 ms` | 待填写 | 待填写 | 待填写 |
| Deadline miss | `0` | 待填写 | 待填写 | 待填写 |
| Skipped cycle | `0` | 待填写 | 待填写 | 待填写 |
| Output error | `0` | 待填写 | 待填写 | 待填写 |
| 控制任务 identity | 全程不变 | 待填写 | 待填写 | 待填写 |
| 执行器任务 identity | 全程不变 | 待填写 | 待填写 | 待填写 |
| TWAI TX/RX error counter | 不增长 | 待填写 | 待填写 | 待填写 |
| TWAI TX failed/bus error/arbitration lost | 不增长 | 待填写 | 待填写 | 待填写 |
| TWAI RX missed | 不增长 | 待填写 | 待填写 | 待填写 |

如果任何门槛失败，保留原始数据并诊断实现或测试环境；不得为了获得 PASS 而放宽
`2 ms`、`0` 错误计数或每轮 10 秒的标准。

## 最终收敛检查

三轮测试和竞争回归后记录：

| 项目 | LEFT | RIGHT |
|---|---|---|
| 反馈有效/在线 | 待填写 | 待填写 |
| RAM 软件零位有效 | 待填写 | 待填写 |
| 最终关节角 | 待填写 | 待填写 |
| 最终 RS01 模式 | 待填写 | 待填写 |
| 最终使能状态 | 待填写 | 待填写 |
| fault/warning raw | 待填写 | 待填写 |
| 最终温度 | 待填写 | 待填写 |

最终系统必须为 `IDLE`、`run_requested=false`、`output_allowed=false`，两台电机均为
REST、失能且无故障。LEFT/RIGHT 软件零位应继续有效，但复位、掉电或重新烧录后仍
会按阶段 3 设计失效，必须重新在中立位归零。

## 完成记录

仅在全部矩阵项完成后填写：

| 项目 | 值 |
|---|---|
| 最终结论 | 待填写 |
| 未通过项/保留边界 | 待填写 |
| 最终实现提交 | 待填写 |
| 最终验收提交 | 待填写 |
| 标签 | `phase5-system-state-start-stop`（待创建） |
| 未纳入范围的用户文件 | `web-bluetooth-console/`，不得修改或提交 |
