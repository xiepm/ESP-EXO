# ESP-EXO 产品 BLE 协议 v1

## 1. 适用范围

本协议是阶段 6 的手机/上位机产品接口。它只负责：

- 查询协议与固件信息；
- 查询整机状态；
- 查询左、右髋关节快照；
- 请求整机 START/STOP；
- 捕获或清除单侧髋关节 RAM 软件零位；
- 阶段 8 扩展：在 IDLE 设置/读取控制模式、参考源和单侧五元命令；
- 阶段 9 扩展：按需读取步态估计和高层助力配置，并在 IDLE 设置 SHADOW 策略。

它不接受 UART 调试台的 ASCII 命令，不提供原始 RS01 命令，也不在阶段 6 连续推送
100 Hz 遥测。阶段 8～9 只增加低频状态/静态配置；ESP32 仍在本地唯一 100 Hz 控制任务中生成
和发送关节命令，手机不能通过 BLE 连续流式驱动电机。连续遥测属于阶段 10。

## 2. GATT 接口

广播名称：`ESP-EXO`

| 对象 | UUID | 属性 |
|---|---|---|
| Product Service | `F7A10001-6B5A-4D92-9E7C-3A1C8B640001` | Primary Service |
| Command | `F7A10002-6B5A-4D92-9E7C-3A1C8B640001` | Write |
| Response | `F7A10003-6B5A-4D92-9E7C-3A1C8B640001` | Notify |

手机连接后必须先订阅 Response Notify，再向 Command 写入完整的一帧。App 应使用带响应
写入（例如 Web Bluetooth 的 `writeValueWithResponse()`）；服务端仍会按同一幂等规则处理
客户端手工发出的 Write Command。

服务端要求“一次 GATT 写入就是一帧”，不跨多次写入拼接。所有 v1 请求和响应不超过
20 字节，因此不依赖 MTU 协商。

## 3. 公共帧头

```text
offset  size  含义
0       1     version：固定 0x01
1       1     opcode
2       1     request_id：手机请求使用 0x01..0xFF
3       1     payload_len：0..16
4       N     payload
```

总长度必须严格等于 `4 + payload_len`。多一个字节、少一个字节、超过 20 字节都会被
拒绝。多字节整数均为 little-endian。

响应 opcode 等于请求 opcode 与 `0x80` 按位或。Response 通知中的 `request_id` 与请求
相同，手机据此关联异步通知，不依赖“写入 Promise 和通知谁先到”。

`request_id` 不是长文本 sequence。手机通常维护一个 1 字节计数器，在 1..255 间递增
并跳过 0 即可。例如请求号为 8 的 START 整帧只有 4 字节：

```text
01 10 08 00
```

同一连接会话内，最近 8 个请求及其响应会被缓存：

- 相同 `request_id`、相同完整请求：不重复执行，原样重发缓存响应；
- 相同 `request_id`、不同请求：返回 `REQUEST_ID_CONFLICT`；
- 断开再连接会进入新会话，旧缓存不参与新会话判定。

App 在一个会话内应顺序递增请求号，不要主动复用最近的请求号。协议 v1 不携带
`expected_config_revision`。阶段 8 配置只允许在 IDLE 同步写入并立即读回确认，不引入
配置版本号、NVS 或 RUNNING 热切换。

## 4. 请求

左右侧在线码中，`0x00` 表示左髋，`0x01` 表示右髋；它们不是内部 C enum，也不是
电机 A/B 或 CAN ID。

| opcode | 名称 | payload | 完整请求示例（request_id=8） |
|---:|---|---|---|
| `0x01` | GET_INFO | 无 | `01 01 08 00` |
| `0x02` | GET_SYSTEM_STATUS | 无 | `01 02 08 00` |
| `0x03` | GET_JOINT_STATUS | `side:u8` | 左：`01 03 08 01 00` |
| `0x04` | GET_CONTROL_CONFIG | 无 | `01 04 08 00` |
| `0x05` | GET_JOINT_COMMAND | `side:u8` | 左：`01 05 08 01 00` |
| `0x06` | GET_GAIT_STATUS | 无 | `01 06 08 00` |
| `0x07` | GET_ASSIST_CONFIG | 无 | `01 07 08 00` |
| `0x10` | START | 无 | `01 10 08 00` |
| `0x11` | STOP | 无 | `01 11 08 00` |
| `0x12` | CAPTURE_ZERO | `side:u8` | 右：`01 12 08 01 01` |
| `0x13` | CLEAR_ZERO | `side:u8` | 左：`01 13 08 01 00` |
| `0x14` | SET_CONTROL_CONFIG | `selected_mode:u8, selected_reference:u8` | IMPEDANCE+SINE：`01 14 08 02 06 01` |
| `0x15` | SET_JOINT_COMMAND | 单侧 15-byte 五元命令 | 左侧零目标、KP=0.2、KD=0.1：`01 15 08 0F 00 00 00 00 00 00 00 00 00 00 00 14 00 64 00` |
| `0x16` | SET_ASSIST_CONFIG | 8-byte 高层策略配置 | 透明 SHADOW：`01 16 08 08 01 01 01 01 00 00 00 00` |

CAPTURE_ZERO/CLEAR_ZERO 修改的是 ESP32 RAM 内的单侧髋关节软件零位，不修改 RS01
机械零位，也不发 CAN。两侧是两个独立请求，不宣称原子双侧归零。

SET_CONTROL_CONFIG 和 SET_JOINT_COMMAND 只在整机 IDLE 接受。RUNNING 中返回
`REJECTED_BUSY`，手机应执行 `STOP -> 等待 IDLE -> SET -> GET 读回 -> START`，而不是
在运动过程中热切换。SET 请求只修改 selected 配置；START 时固件把 selected 一次性
锁存为 active，本轮 RUNNING 期间 active 保持不变。

SET_ASSIST_CONFIG 的 8-byte payload 为：

```text
schema_version:u8, selected_mode:u8, requested_execution:u8, strategy_id:u8,
profile_id:u8, left_level_percent:u8, right_level_percent:u8, reserved_flags:u8
```

阶段 9 只实际接受 `DISABLED(0)` 和 `SHADOW(1)`；`ACTIVE(2)` 是可识别线码，但固件
返回 `REJECTED_NOT_READY`，ACK detail=`ACTIVE_NOT_ALLOWED(3)`。这使客户端能区分
“请求格式损坏”和“当前固件明确不允许真实助力”。普通产品界面不应提供 ACTIVE 入口。

## 5. 响应

### 5.1 ACK（总长 8 字节）

用于 START、STOP、CAPTURE_ZERO、CLEAR_ZERO、三个 SET 命令以及可编码的协议错误。

任意 GET 命令若发生内部快照/编码错误，也可能以相同
响应 opcode 返回 4 字节 ACK，而不是正常 INFO/STATUS payload。手机应先依据 payload_len
识别 ACK，再解析正常响应，不要只按 opcode 强制解释固定结构。

```text
payload[0] result
payload[1] system_state
payload[2] system_flags
payload[3] detail
```

START 被拒绝时，`detail` 是 `start_block_reason`；START 成功、已满足或已挂起时为
`NONE(0)`。SET_ASSIST_CONFIG 的 ACTIVE 拒绝使用 detail=3；其余 ACK 当前为 0。
`ACCEPTED` 只表示异步请求已被状态机接收，不表示启停
序列已经完成。App 收到 START/STOP ACK 后仍须轮询系统状态，
直到 `RUNNING`、`IDLE` 或 `ERROR`。

### 5.2 INFO（opcode `0x81`，payload 12 字节）

```text
protocol_min:u8, protocol_max:u8,
firmware_major:u8, firmware_minor:u8, firmware_patch:u8,
capabilities:u16_le, max_frame_size:u8, boot_id:u32_le
```

阶段 6 基线固件版本字段为 `0.6.0`；后续阶段可以在保持协议 v1 的同时更新该版本字段。
capabilities：

| bit | 含义 |
|---:|---|
| 0 | 整机 START/STOP |
| 1 | 关节 RAM 软件零位 |
| 2 | 按需状态查询 |
| 3 | 阶段 8 控制模式、参考源和五元命令静态配置 |
| 4 | 阶段 9 步态状态查询 |
| 5 | 阶段 9 高层助力配置查询/设置 |

`boot_id` 每次 ESP32 启动重新生成，可用于识别连接期间设备是否重启。

### 5.3 SYSTEM_STATUS（opcode `0x82`，payload 10 字节）

```text
state:u8, active_step:u8, system_flags:u8,
start_block_reason:u8, last_request_result:u8,
last_failed_step:u8, transition_count:u32_le
```

状态线码：

| 值 | 状态 |
|---:|---|
| 0 | INIT |
| 1 | IDLE |
| 2 | STARTING |
| 3 | RUNNING |
| 4 | STOPPING |
| 5 | ERROR |
| 255 | UNKNOWN |

system_flags：bit0 initialized，bit1 ready_to_start，bit2 run_requested，bit3
output_allowed，bit4 maintenance_active，bit5 executor_alive。

active_step/last_failed_step：0 NONE，1 CHECK_PRECONDITIONS，2 DISABLE_MOTORS，
3 SET_MOTION_MODE，4 ENABLE_MOTORS，5 SEND_ZERO_OUTPUT，6 WAIT_FRESH_FEEDBACK，
7 ARM_CONTROLLER，8 DISARM_CONTROLLER，9 RELEASE_OUTPUT，10 VERIFY_DISABLED，
255 UNKNOWN。

start_block_reason：0 NONE，1 INITIALIZING，2 SYSTEM_NOT_IDLE，3 OPERATION_BUSY，
4 CONTROL_TASK_NOT_READY，5 TWAI_NOT_RUNNING，6/7 LEFT/RIGHT_MAPPING_INVALID，
8/9 LEFT/RIGHT_DIRECTION_INVALID，10/11 LEFT/RIGHT_ZERO_INVALID，
12/13 LEFT/RIGHT_FEEDBACK_INVALID，14/15 LEFT/RIGHT_OFFLINE，
16/17 LEFT/RIGHT_MOTOR_FAULT，18 SYSTEM_ERROR，255 UNKNOWN。

### 5.4 JOINT_STATUS（opcode `0x83`，payload 13 字节）

```text
side:u8, joint_flags:u8, direction:u8,
position_mrad:i32_le, velocity_mrad_s:i32_le,
feedback_age_ms:u16_le
```

`mrad`、`mrad/s` 是 v1 已发布的定点**传输格式**，不是手机界面的显示单位。当前
产品界面统一显示度，客户端必须换算：

```text
position_deg   = position_mrad   * 180 / (pi * 1000)
velocity_deg_s = velocity_mrad_s * 180 / (pi * 1000)
```

固件不能在保持协议版本为 v1 时把这两个整数静默改成毫度，否则旧客户端会产生
约 57.3 倍的解释错误。若未来要改变线格式，必须发布新的协议版本。

joint_flags：bit0 mapping_valid，bit1 direction_valid，bit2 zero_valid，bit3
feedback_valid，bit4 online，bit5 joint_valid。只有 joint_valid 为 1 时，位置和速度才是
有效的髋关节量；否则固件发送 0。`feedback_age_ms=65535` 表示没有反馈时间戳或已饱和。

direction：0 UNKNOWN，1 POSITIVE，2 NEGATIVE。

### 5.5 CONTROL_CONFIG（opcode `0x84`，payload 5 字节）

```text
selected_mode:u8, active_mode:u8,
selected_reference:u8, active_reference:u8,
config_flags:u8
```

控制模式线码：0 NONE（只允许出现在无 active/selected 的响应槽位），1 HOLD，
2 POSITION，3 VELOCITY，4 TORQUE，5 DAMPING，6 IMPEDANCE。SET 请求中的
`selected_mode` 必须是 1..6，不能写 NONE。

参考源线码：0 CONSTANT，1 SINE。SINE 是参考源而不是控制模式；阶段 8 只允许
`IMPEDANCE + SINE`，其他五种模式必须搭配 CONSTANT，否则返回 INVALID_PAYLOAD。

`config_flags`：bit0 selected_valid，bit1 active_valid，其他位必须为 0。对应 valid 位为 0
时，其 mode 必须是 NONE。正常响应刻意使用 5 字节：现有通用 ACK 是 4 字节，如果这里
也使用 4 字节，则 HOLD/POSITION/VELOCITY 等首字节会与 ACK result 线码发生不可判定歧义。

### 5.6 JOINT_COMMAND（opcode `0x85`，payload 15 字节）

GET 正常响应与 SET 请求使用完全相同的单侧五元布局：

```text
side:u8,
position_mrad:i32_le,
velocity_mrad_s:i32_le,
torque_mNm:i16_le,
kp_x100:u16_le,
kd_x1000:u16_le
```

网页显示使用 `deg、deg/s、N·m、KP、KD`；线协议定点换算为：

```text
position_mrad   = round(position_deg * pi * 1000 / 180)
velocity_mrad_s = round(velocity_deg_s * pi * 1000 / 180)
torque_mNm      = round(torque_Nm * 1000)
kp_x100         = round(KP * 100)
kd_x1000        = round(KD * 1000)
```

线格式范围：position `-12570..12570 mrad`，velocity `-44000..44000 mrad/s`，
torque `-17000..17000 mN·m`，KP `0..500`，KD `0..5`。格式层对越界值返回
`INVALID_PAYLOAD`，不得静默限幅；业务层还可以实施更窄的产品配置范围。KP/KD 是
RS01 Motion 协议增益，本协议不虚构未经手册证明的物理单位。

### 5.7 GAIT_STATUS（opcode `0x86`，payload 16 字节）

```text
estimator_id:u8, estimator_state:u8, gait_flags:u8, phase_origin:u8,
quality_flags:u16_le, left_phase_x10000:u16_le, right_phase_x10000:u16_le,
cadence_x100:u16_le, confidence_x10000:u16_le, estimate_age_ms:u16_le
```

- estimator：0 NONE，1 HIP_EVENT_TBE；state：0 RESET，1 STANDING，2 ACQUIRING，
  3 LOCKED，4 DEGRADED；
- phase origin：0 UNKNOWN，1 HIP_FLEXION_PEAK，2 HIP_EXTENSION_PEAK；当前值是髋角代理
  事件，不是 heel strike/toe off；
- gait_flags：bit0 overall valid，bit1/2 左/右相位有效，bit3检测到运动，bit4/5
  左/右本帧相位回绕；允许单侧相位有效，但 overall 只有双侧均有效时置位；
- 相位范围 `0..9999` 对应 `[0,1)`，置信度 `0..10000`，步频单位 `0.01 step/min`；
- 任一无效定点字段使用 `0xFFFF`，客户端不得显示成 0；
- quality bit0..14 依次表示左/右输入无效、低运动、左/右等待事件、左/右周期越界、
  周期不稳、左右源时间偏斜、时间/序号回退、坐标复位、左/右相位超时、仅单侧、
  双侧周期/相位不一致。未知高位必须保留并显示，不能导致整个握手失败。

### 5.8 ASSIST_CONFIG（opcode `0x87`，payload 16 字节）

```text
schema_version:u8, selected_mode:u8, active_mode:u8, execution_states:u8,
strategy_id:u8, strategy_version:u8, profile_id:u8,
left_level_percent:u8, right_level_percent:u8, assist_flags:u8,
unavailable_reason:u8, config_generation:u8,
required_sensor_mask:u16_le, available_sensor_mask:u16_le
```

`execution_states` 在不增加帧长的前提下同时保留两种语义：低 4 位是 selected 的
`requested_execution`，高 4 位是本次生命周期当前真正生效的 `effective_execution`。
每个半字节只能为 0 DISABLED、1 SHADOW、2 ACTIVE，其余保留位必须为 0。例如 IDLE
下已选择 SHADOW、尚未 START 时该字节为 `0x01`；START 锁存并执行 SHADOW 后为
`0x11`；STOP 后仍保留 selected 时回到 `0x01`。

高层模式：0 NONE，1 TRANSPARENT，2 ACTIVE_PHASE，3 ASSIST_AS_NEEDED；策略：0 NONE，
1 TRANSPARENT，2 PHASE_PROFILE，3 AAN_RESIDUAL。高层模式描述“如何助力”，与阶段 8
POSITION/TORQUE/IMPEDANCE 等底层执行模式是两个不同层级。

assist_flags：bit0 selected valid，bit1 active valid，bit2 config locked，bit3 strategy
ready，bit4 required sensors ready，bit5 suggestion valid，bit6 real output enabled。
阶段 9 的 bit6 必须始终为 0。

unavailable_reason：0 NONE，1 NOT_INITIALIZED，2 CONFIG_INVALID，3 ACTIVE_NOT_ALLOWED，
4 STRATEGY_NOT_IMPLEMENTED，5 REQUIRED_SENSOR_MISSING，6 NOT_ARMED，
7 REQUIRED_GAIT_INVALID。`requested_execution=SHADOW` 但策略尚未实现时，响应中的
effective_execution 可以是 DISABLED，客户端应同时显示 requested/effective 与原因，
SET 后的读回校验应比较 requested 而不是 effective。

## 6. result 线码

| 值 | 名称 |
|---:|---|
| `0x00` | OK |
| `0x01` | ACCEPTED |
| `0x02` | ALREADY_SATISFIED |
| `0x03` | ALREADY_PENDING |
| `0x10` | REJECTED_NOT_READY |
| `0x11` | REJECTED_BUSY |
| `0x12` | REJECTED_ERROR |
| `0x13` | INTERNAL_ERROR |
| `0x20` | UNSUPPORTED_VERSION |
| `0x21` | INVALID_LENGTH |
| `0x22` | UNKNOWN_OPCODE |
| `0x23` | INVALID_REQUEST_ID |
| `0x24` | INVALID_PAYLOAD |
| `0x25` | REQUEST_ID_CONFLICT |
| `0x7F` | NONE（只用于尚无请求的状态快照） |

## 7. 手机完整流程

1. 扫描并连接 `ESP-EXO`，发现 Product Service。
2. 订阅 Response Notify；订阅完成前不发送命令。
3. 发送 GET_INFO，核对协议范围包含 v1、最大帧长为 20，并记录 `boot_id`。
4. 查询 SYSTEM_STATUS 和左右 JOINT_STATUS，显示整机、在线状态和软件零位状态。
5. 若 INFO capability bit3=1，再读取 CONTROL_CONFIG 和左右 JOINT_COMMAND；bit3=0 时
   隐藏配置入口，不向旧固件发送未知 opcode。
6. capability bit4/5=1 时按低频轮询读取 GAIT_STATUS/ASSIST_CONFIG；缺位时隐藏阶段 9
   卡片，不向旧固件发送新 opcode。
7. 若两侧 `zero_valid=0`，操作者将双髋置于中立位后，分别发送左右 CAPTURE_ZERO。
8. 再查两侧 JOINT_STATUS，确认 `zero_valid` 和 `joint_valid`。
9. 调整控制配置时，先 STOP 并等待 IDLE；发送 SET_CONTROL_CONFIG 和左右
   SET_JOINT_COMMAND，随后逐项 GET，只有读回完全一致才允许继续。
10. 若设置高层策略，只允许 IDLE 下配置 DISABLED/SHADOW，并在 GET_ASSIST_CONFIG
    读回一致后继续；手机状态查询不是 100 Hz 控制流。
11. 发送 START；若 ACK 为 ACCEPTED/ALREADY_PENDING，轮询 SYSTEM_STATUS 至 RUNNING；若被
   拒绝，显示 `detail` 对应的 start_block_reason。
12. 发送 STOP 后轮询 SYSTEM_STATUS 至 IDLE；ERROR 时显示失败步骤。

断开 BLE 不会隐式 STOP，也不会改变阶段 5 已接收的运行请求。App 若希望停机，必须先
显式发送 STOP、确认状态，再断开。这是阶段 6 对阶段 5 请求语义的延续，并非连接租约。
