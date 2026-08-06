# 阶段 5：系统状态机和 START/STOP 执行器

## 目标

阶段 5 在阶段 4 的唯一永久 100 Hz 控制任务之上，建立一个与 UART、BLE 和具体
控制算法无关的整机生命周期对象。所有整机 START/STOP 请求必须经过同一个系统
接口；通信任务只提交目标，电机准备、使能、停止和失能由后台执行器完成。

本阶段解决的是“谁有权允许运动、请求如何落地以及启停如何串行化”。它不增加新的
控制模式，不改变阶段 4 已验收的实时调度、LEFT/RIGHT 映射、RS01 主动反馈或 RAM
软件零位。

## 阶段边界

本阶段包括：

- 六态整机状态机；
- 非阻塞、幂等的 START/STOP 请求接口；
- 一个静态、事件驱动的 START/STOP 执行任务；
- 唯一的周期运动输出许可；
- START/STOP 与调试维护 CAN 操作的互斥；
- 一致的系统诊断快照；
- 把现有正弦阻抗模块收缩为 `arm/disarm/process_tick`；
- UART/BLE 调试文本命令 `start`、`stop` 和 `system_status`。

本阶段明确不包括：

- 阶段 6 的产品 BLE 二进制协议、协议版本、opcode、sequence、request ID、CRC 或
  配置版本检查；
- 阶段 7 的 LEFT/RIGHT 正式参考生成和控制律迁移；
- 阶段 8 的五元关节命令及新增基础控制模式；
- 阶段 9 的统一传感快照和步态估计；
- 阶段 10 的相位曲线和连续遥测；
- 完整的产品安全状态机、故障分级或持久化零位。

阶段 5 继续把阶段 4 的旧正弦阻抗控制作为回归负载。其 `KP=0.2`、`KD=0.10`、
`+/-pi/2` 幅值、`0.5 Hz`、3 秒五次渐入及两台电机的既有相位关系均保持不变。

## 六态模型

系统只保存以下六个生命周期状态：

| 状态 | 含义 | 是否允许周期运动输出 |
|---|---|---:|
| `INIT` | 子系统初始化尚未全部结束 | 否 |
| `IDLE` | 整机停止，等待请求 | 否 |
| `STARTING` | 已接受 START，执行器正在准备双电机 | 否 |
| `RUNNING` | 启动序列完成，整机允许运动 | 是 |
| `STOPPING` | 输出许可已撤销，执行器正在停止和失能 | 否 |
| `ERROR` | START/STOP 或其清理失败，无法确认生命周期收敛 | 否 |

BLE 是否连接、单侧反馈是否在线、软件零位是否有效、RS01 当前模式和阻抗控制器是否
已 arm 都不是新的系统状态。这些信息是快照属性或 START 前置条件，避免状态数量随
属性组合膨胀。

### `RUNNING` 的唯一含义

系统 `RUNNING` 是产品层运动许可，不等于以下任何一个局部状态：

- 永久 100 Hz 控制任务存活；
- TWAI 驱动处于 `TWAI_STATE_RUNNING`；
- RS01 报告为使能或处于某个运行模式；
- 旧阻抗控制器内部已 arm；
- BLE 已连接。

只有系统状态为 `RUNNING` 且 `output_allowed=true` 时，控制任务才可以调用周期运动
发送路径。两者作为一个一致状态发布；任一条件不满足都不得产生周期运动帧。

## 派生就绪条件

`READY/NOT_READY` 不作为第七个状态保存。`ready_to_start` 每次从当前一致快照派生：

```text
ready_to_start =
    state == IDLE
    && !maintenance_active
    && 100 Hz 控制任务存活
    && TWAI 处于 RUNNING
    && LEFT joint_valid
    && RIGHT joint_valid
    && LEFT/RIGHT 均无当前电机故障
```

`joint_valid` 已包含映射、安装方向、RAM 软件零位、反馈有效和反馈在线要求。START
执行器仍会在真正操作电机前重新读取 LEFT/RIGHT、反馈计数和故障状态，不能把请求
时的 `ready_to_start=true` 当成随后一直成立的保证。

系统快照应同时给出 `start_block_reason`，使调试端能区分控制任务、TWAI、LEFT、
RIGHT、反馈或电机故障等阻断原因。阶段 6 可直接复用该派生信息，无需重新实现
就绪判定。

## 状态转换和请求语义

| 当前状态 | START | STOP |
|---|---|---|
| `INIT` | `REJECTED_NOT_READY` | `ALREADY_SATISFIED` |
| `IDLE` | 就绪且无维护操作时进入 `STARTING`；否则拒绝 | `ALREADY_SATISFIED` |
| `STARTING` | `ALREADY_PENDING`，不重复排队 | 取消启动、立即关门并进入 `STOPPING` |
| `RUNNING` | `ALREADY_SATISFIED`，不得重置轨迹 | 立即关门并进入 `STOPPING` |
| `STOPPING` | `REJECTED_BUSY`，不保存自动重启 | `ALREADY_PENDING` |
| `ERROR` | `REJECTED_ERROR` | 接受再次清理；成功回 `IDLE`，失败留在 `ERROR` |

请求返回值至少区分：

```text
ACCEPTED
ALREADY_SATISFIED
ALREADY_PENDING
REJECTED_NOT_READY
REJECTED_BUSY
REJECTED_ERROR
INTERNAL_ERROR
```

请求被接受只表示目标和状态已经发布，不表示电机序列已经完成。调用方必须通过系统
快照判断最终进入 `RUNNING`、`IDLE` 或 `ERROR`。

`run_requested` 保存最新的整机运行目标：START 接受时设为 `true`；STOP 接受时设为
`false`。STOPPING 中的 START 被明确拒绝，因此本阶段没有“停止完成后自动重启”的
隐式排队语义。

## 公共接口

系统模块位于 `main/exo_system.c` 和 `main/include/exo_system.h`，核心操作接口为：

```c
esp_err_t exo_system_init(void);
esp_err_t exo_system_finish_initialization(void);

exo_request_result_t exo_system_request_start(void);
exo_request_result_t exo_system_request_stop(void);

esp_err_t exo_system_process_control_tick(int64_t now_us);
bool exo_system_get_snapshot(exo_system_snapshot_t *snapshot);

bool exo_system_try_begin_maintenance(void);
void exo_system_end_maintenance(void);
```

头文件还提供状态、步骤、阻断原因和请求结果的名称转换函数，仅用于按需诊断输出。

START/STOP 请求接口必须为非阻塞路径：不发送 CAN、不调用 `vTaskDelay()`、不等待
反馈，也不执行电机模式切换。它们只在短临界区更新状态、计数和输出门，然后通知
执行器任务。

`exo_system_get_snapshot()` 一次复制已发布的完整副本，不允许调用方分别读取多个
可变字段后自行拼接状态。

## 执行模型

系统模块只新增一个执行任务，固定资源为：

- 一个 `StaticTask_t`；
- 一块 4096-byte 静态任务栈；
- Core 0；
- 优先级 6；
- 上电只创建一次；
- 无请求时永久阻塞等待 task notification。

执行器不是周期控制任务，不创建周期 `esp_timer`，不使用 `xTaskDelayUntil()`、动态
内存、无界队列或空闲轮询。它只用 `esp_timer_get_time()` 取得单调时间戳，并在启停
序列中以 10 ms 间隔进行有 500 ms 上限的反馈确认。阶段 4 的 Core 1 / priority 9
永久 100 Hz 控制任务保持唯一周期源，任务 handle 和身份不因 START/STOP 改变。

执行过程中的反馈等待使用有限超时，并能够被新的 STOP notification 提前唤醒；不把
启动序列写成一串无法取消的长 `vTaskDelay()`。执行器允许调用会等待的 RS01 服务
接口，但这些操作永远不在请求入口或 100 Hz 控制路径中执行。

## START 执行序列

执行器在独占系统操作权后按以下顺序处理 START：

1. 重新检查 `run_requested=true`，确认启动尚未被 STOP 取消；
2. 检查永久控制任务存活及 TWAI 状态；
3. 检查 LEFT/RIGHT 映射、方向、RAM 零位、反馈、在线状态、`joint_valid` 和电机故障；
4. 双侧失能；
5. 双侧设置阶段 4 已使用的 Motion 模式；
6. 双侧使能；
7. 重新读取两侧当前位置并记录当前反馈计数；
8. 在各自当前位置发送零速度、零力矩、零 `KP/KD` 帧；
9. 等待相对于步骤 7 的两侧反馈计数均增长，并重新检查反馈和故障；
10. 捕获旧正弦阻抗控制所需的两侧中心位置；
11. arm 阻抗控制器；
12. 在一个短临界区同时发布 `state=RUNNING` 和 `output_allowed=true`。

每个可取消步骤和每次有限等待后都重新检查 `run_requested`。如果 STARTING 中收到
STOP，执行器不得发布 `RUNNING`，而是转入统一停止清理序列。

任一步骤失败时，先撤销输出许可并尝试双侧停止、失能和反馈确认。清理成功则回到
`IDLE`，记录“START 失败但清理成功”；只有不能确认生命周期已经收敛时才进入
`ERROR`。

## STOP 执行序列

STOP 的首要动作发生在请求入口，而不是等待执行器调度：

1. 原子设置 `run_requested=false`；
2. 原子设置 `output_allowed=false`，使下一次 100 Hz tick 不再进入运动发送路径；
3. 将活动状态转为 `STOPPING` 并通知执行器；
4. 执行器 disarm 阻抗控制器，并等待已经进入的控制 tick 退出；
5. 读取两侧当前位置；
6. 发送当前位置、零速度、零力矩、零 `KP/KD` 帧；
7. 双侧失能；
8. 等待两侧产生新反馈，并检查 REST/失能结果；
9. 成功时进入 `IDLE`，失败时进入 `ERROR`。

`enabled_requested=false` 只说明失能命令已被请求或入队，不能单独证明电机已经确认
失能。最终判断必须结合失能后的新反馈和运行模式。重复 STOP 不得重复执行硬件
序列或重复增加停止成功计数。

## 100 Hz 控制路径

阶段 5 的周期调用链为：

```text
exo_control_loop
  -> exo_system_process_control_tick(now_us)
  -> 读取一次一致的 state/output_allowed
  -> 仅当 RUNNING && output_allowed
  -> impedance_control_process_tick(now_us)
  -> rs01_try_send_motion()
```

控制路径不获取可能阻塞的操作 mutex，不打印日志、不发送 BLE Notify、不动态分配、
不切换模式、不归零，也不执行阻塞 CAN。STOP 在请求入口关门后，最迟下一次 100 Hz
周期不再开始新的运动帧尝试；已经进入的单个 tick 由 disarm/退出握手收敛。

## 维护操作互斥

状态检查和维护 CAN 发送不能是两个分离动作，否则可能出现“检查时仍为 IDLE，发送
时已进入 STARTING”的竞争。系统模块提供一个静态操作 mutex，并配合状态临界区中的
`maintenance_active` 标志：

1. `exo_system_try_begin_maintenance()` 以零等待方式尝试获取操作 mutex；
2. 在状态临界区确认系统仍为 `IDLE`、没有运行请求且执行器不忙；
3. 设置 `maintenance_active=true`，调用方在整个维护操作期间持有操作权；
4. `exo_system_end_maintenance()` 清除标志并释放 mutex。

START 请求在同一状态临界区检查 `maintenance_active`；如果维护已开始则返回
`REJECTED_BUSY`。如果 START 已先进入 `STARTING`，随后进入的维护操作会在重新检查
状态时被拒绝。执行器也通过同一操作 mutex 串行化电机启停序列。

所有会发送维护 CAN 或改变运行前提的调试命令都必须使用该作用域，包括使能、失能、
运动、电流、模式/ID/波特率/参数写入、机械零位以及 RAM 软件零位的建立和清除。
`status`、`joint_status`、`control_status`、`can_status`、`system_status`、`motor_type`
和 `help` 等只读命令可以继续执行。

## 阻抗模块职责收缩

`impedance_control` 不再拥有产品生命周期，也不再直接执行以下工作：

- 电机失能或使能；
- RS01 模式切换；
- 阻塞等待反馈；
- 创建、停止或重启控制任务。

它只保留：

- `arm`：接收已确认的中心位置和开始时间，准备既有算法状态；
- `disarm`：阻止新 tick，并与已经进入的 tick 完成退出握手；
- `process_tick`：在唯一 100 Hz 任务中运行既有正弦参考、渐入和阻抗计算。

算法公式、参数和 A/B 兼容实现本阶段不迁移；A/B 到 LEFT/RIGHT 的正式迁移属于
阶段 7。

## 系统快照

`exo_system_snapshot_t` 至少包含：

- `state`、`active_step`；
- `ready_to_start`、`start_block_reason`；
- `run_requested`、`output_allowed`、`maintenance_active`；
- 执行器任务是否存活及任务身份；
- 状态转换计数、START/STOP 请求计数；
- 启动成功、停止成功、失败、取消和拒绝计数；
- 最近请求结果、最近执行错误和最近失败步骤；
- 状态进入、最近请求和最近完成的微秒时间戳。

状态、输出门、计数和错误由模块私有状态持有，并只在同一个短临界区内更新；读取方
也在该临界区一次复制完整快照。派生就绪条件在硬件读取前后核对状态和维护代次，若
发生竞争则保守返回未就绪，避免拼接两个转换的字段。`system_status` 只按需打印该
副本，不属于阶段 10 连续遥测。

## 调试命令边界

UART 与 BLE NUS 继续共用阶段 3/4 的文本调试控制台，并新增：

```text
start
stop
system_status
```

`imp_start`/`imp_stop` 可以保留为兼容别名，但只能调用系统请求接口，不得绕过状态机
直接调用阻抗控制器。通信任务不得等待完整启停序列；命令响应应区分“请求已接受”与
“状态转换已完成”。

这仍不是手机产品协议。阶段 6 只需把产品命令映射到同一 START/STOP 请求和快照
接口，不再接触 RS01 启停细节。

## 初始化顺序

推荐的 `app_main()` 初始化顺序为：

```text
RS01 driver
-> hip joint objects
-> TWAI driver
-> event-driven TWAI RX task
-> RS01 proactive reporting
-> exo_system_init()                 # INIT，创建后立即阻塞的执行器
-> BLE/UART debug consoles
-> permanent 100 Hz control loop
-> exo_system_finish_initialization() # 检查完成后发布 IDLE
-> app_main return
```

系统模块必须先于控制任务初始化，因为控制任务每周期调用系统门；完成函数必须晚于
控制任务创建，使 `IDLE` 不会在永久控制任务尚未存活时被发布。NimBLE 等重量级初始化
仍位于控制任务创建之前，延续阶段 4 的时序隔离。

上电默认 `INIT`、`run_requested=false`、`output_allowed=false`。完成初始化只进入
`IDLE`，不得自动使能电机，也不得因双侧零位有效而自动 START。

## 并发不变量

实现必须始终满足：

1. 全项目只有一个应用级 100 Hz 周期源；
2. START/STOP 永不创建、删除、暂停或重启该控制任务；
3. 只有 `RUNNING && output_allowed` 能到达周期运动发送；
4. STOP 请求入口先关闭输出门，再等待任何硬件清理；
5. `STARTING` 中 STOP 最终不得短暂发布 `RUNNING`；
6. 执行器和维护命令不能同时操作 RS01/TWAI；
7. 活动状态下维护命令不能通过“先检查后发送”的竞争插入；
8. 重复 START/STOP 不重复执行硬件序列；
9. 控制路径只读取一致快照，不等待操作 mutex；
10. 状态快照不会混合两个状态转换的字段；
11. BLE 连接、断开和重连不改变 `run_requested` 或创建任务；
12. RAM 软件零位在正常多轮启停后保持有效，复位后仍按阶段 3 规则失效。

## 文件范围

新增：

```text
main/exo_system.c
main/include/exo_system.h
docs/phase5/PHASE_5_DESIGN.md
docs/phase5/PHASE_5_ACCEPTANCE.md
```

修改：

```text
main/app_main.c
main/exo_control_loop.c
main/impedance_control.c
main/impedance_control.h
main/ble_console.c
main/CMakeLists.txt
CONSOLE_COMMANDS.md
README.md
```

除非实现审计给出必要证据，本阶段不修改 `rs01_driver`、`hip_joint`、`exo_config`、
`exo_types` 或 `sdkconfig`。LEFT 保持执行器 B / `+1`，RIGHT 保持执行器 A / `-1`。

## 完成标准

1. 六态生命周期和派生 `ready_to_start` 的语义唯一；
2. 一个 4096-byte 静态栈、Core 0 / priority 6 的事件执行器完成全部启停硬件序列；
3. START/STOP 请求非阻塞、幂等、可取消且请求结果与完成状态分离；
4. STOP 请求最迟使下一次 100 Hz tick 不再进入运动发送；
5. 唯一永久控制任务、100 Hz、任务身份和阶段 4 实时门槛保持不变；
6. START/STOP 与所有维护操作通过同一操作所有权串行化；
7. 阻抗模块只保留 arm/disarm/tick，算法公式和参数不变；
8. 调试控制台不能绕过系统状态机；
9. 静态、构建、无运动和三轮运动验收全部有可追溯证据；
10. 未提前实现阶段 6-10 内容。
