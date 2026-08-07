# ESP-EXO 产品 BLE 协议 v1

## 1. 适用范围

本协议是阶段 6 的手机/上位机产品接口。它只负责：

- 查询协议与固件信息；
- 查询整机状态；
- 查询左、右髋关节快照；
- 请求整机 START/STOP；
- 捕获或清除单侧髋关节 RAM 软件零位。

它不接受 UART 调试台的 ASCII 命令，不提供原始 RS01 命令，也不在阶段 6 连续推送
100 Hz 遥测。控制参数和五元关节命令属于阶段 8，连续遥测属于阶段 10。

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
`expected_config_revision`：当前硬件配置是固件编译期只读配置，尚不存在运行时配置提交。

## 4. 请求

左右侧在线码中，`0x00` 表示左髋，`0x01` 表示右髋；它们不是内部 C enum，也不是
电机 A/B 或 CAN ID。

| opcode | 名称 | payload | 完整请求示例（request_id=8） |
|---:|---|---|---|
| `0x01` | GET_INFO | 无 | `01 01 08 00` |
| `0x02` | GET_SYSTEM_STATUS | 无 | `01 02 08 00` |
| `0x03` | GET_JOINT_STATUS | `side:u8` | 左：`01 03 08 01 00` |
| `0x10` | START | 无 | `01 10 08 00` |
| `0x11` | STOP | 无 | `01 11 08 00` |
| `0x12` | CAPTURE_ZERO | `side:u8` | 右：`01 12 08 01 01` |
| `0x13` | CLEAR_ZERO | `side:u8` | 左：`01 13 08 01 00` |

CAPTURE_ZERO/CLEAR_ZERO 修改的是 ESP32 RAM 内的单侧髋关节软件零位，不修改 RS01
机械零位，也不发 CAN。两侧是两个独立请求，不宣称原子双侧归零。

## 5. 响应

### 5.1 ACK（总长 8 字节）

用于 START、STOP、CAPTURE_ZERO、CLEAR_ZERO 以及可编码的协议错误。

GET_INFO、GET_SYSTEM_STATUS 或 GET_JOINT_STATUS 若发生内部快照/编码错误，也可能以相同
响应 opcode 返回 4 字节 ACK，而不是正常 INFO/STATUS payload。手机应先依据 payload_len
识别 ACK，再解析正常响应，不要只按 opcode 强制解释固定结构。

```text
payload[0] result
payload[1] system_state
payload[2] system_flags
payload[3] detail
```

START 被拒绝时，`detail` 是 `start_block_reason`；START 成功、已满足或已挂起时为
`NONE(0)`，其余 ACK 当前也为 0。`ACCEPTED` 只表示异步请求已被状态机接收，不表示启停
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

joint_flags：bit0 mapping_valid，bit1 direction_valid，bit2 zero_valid，bit3
feedback_valid，bit4 online，bit5 joint_valid。只有 joint_valid 为 1 时，位置和速度才是
有效的髋关节量；否则固件发送 0。`feedback_age_ms=65535` 表示没有反馈时间戳或已饱和。

direction：0 UNKNOWN，1 POSITIVE，2 NEGATIVE。

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
5. 若两侧 `zero_valid=0`，操作者将双髋置于中立位后，分别发送左右 CAPTURE_ZERO。
6. 再查两侧 JOINT_STATUS，确认 `zero_valid` 和 `joint_valid`。
7. 发送 START；若 ACK 为 ACCEPTED/ALREADY_PENDING，轮询 SYSTEM_STATUS 至 RUNNING；若被
   拒绝，显示 `detail` 对应的 start_block_reason。
8. 阶段 6 没有“调节助力参数”命令；手机只显示状态并可发送 STOP。
9. 发送 STOP 后轮询 SYSTEM_STATUS 至 IDLE；ERROR 时显示失败步骤。

断开 BLE 不会隐式 STOP，也不会改变阶段 5 已接收的运行请求。App 若希望停机，必须先
显式发送 STOP、确认状态，再断开。这是阶段 6 对阶段 5 请求语义的延续，并非连接租约。
