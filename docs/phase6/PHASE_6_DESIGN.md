# 阶段 6：产品 BLE 协议设计

## 1. 阶段目标

阶段 6 把原先“手机连接后直接输入工程文本命令”的 BLE 调试通道改成产品接口：手机只
发送有版本、操作码、请求号和长度的短二进制帧；ESP32 在非实时工作任务中验证并执行，
再以 Notify 返回稳定线码。UART 继续保留工程维护命令。

本阶段不迁移正弦阻抗算法、不增加五元关节设定、不做步态估计、不连续推送遥测。这些分别
属于阶段 7、8、9、10。

## 2. 模块与职责

```text
手机 App
  │  Command Write / Response Notify（<=20 B）
  ▼
exo_ble.c
  ├─ NimBLE/GATT/GAP 唯一所有者
  ├─ GATT 回调：复制、边界检查、零等待入队
  ├─ BLE worker：解码、会话去重、业务分发、编码、Notify
  ├─ 查询 exo_system / hip_joint 快照
  └─ 只调用 exo_system_request_start/stop 和软件零位接口
          │
          ├────────► exo_system.c（阶段 5 状态机/异步执行器）
          └────────► hip_joint.c（阶段 3 软件零位/只读快照）

exo_ble_protocol.c
  └─ 纯字节编解码；不依赖 NimBLE、FreeRTOS、CAN 或 C 结构体布局

debug_console.c
  └─ UART-only 工程命令；不拥有 NimBLE，不向手机暴露原始电机命令

exo_control_loop.c
  └─ 全工程唯一 100 Hz 周期源和周期运动输出入口（阶段 4 边界不变）
```

## 3. 文件落点

| 文件 | 职责 |
|---|---|
| `main/include/exo_ble_protocol.h` | v1 常量、稳定线码、协议 DTO 和纯函数 API |
| `main/exo_ble_protocol.c` | 严格请求解码和显式 little-endian 响应编码 |
| `main/include/exo_ble.h` | 产品 BLE 初始化及只读诊断快照 |
| `main/exo_ble.c` | GATT、会话、队列、去重、业务适配和 Notify |
| `main/include/debug_console.h` | UART 调试台初始化接口 |
| `main/debug_console.c` | 原 42 条工程命令及新增 `ble_status` |
| `main/app_main.c` | 先初始化系统、UART 和产品 BLE，再创建唯一控制任务 |
| `main/exo_config.c` | 广播名称 `ESP-EXO` |
| `CONSOLE_COMMANDS.md` | UART-only 工程命令说明 |
| `docs/phase6/BLE_PROTOCOL_V1.md` | 手机端可直接实现的 wire contract |

旧 `ble_console.c/.h` 被移除。它们的工程命令完整迁移到 UART，但 NUS 服务、文本帧、BLE
输出缓存及其 NimBLE 初始化不再进入产品固件。

## 4. 任务和实时边界

阶段 6 只增加一个静态 BLE 命令 worker：Core 0、优先级 5、4096-byte 栈、深度 8 的静态
队列。NimBLE Host 使用 ESP-IDF 自身任务。

GATT access callback 不执行 START/STOP 序列、不归零、不读取 CAN、不延时等待，只做：

1. 确认来自当前连接并已订阅 Response Notify；
2. 检查整帧不超过 20 字节；
3. 使用 `ble_hs_mbuf_to_flat()` 复制完整 mbuf chain；
4. 用零等待方式放入命令队列。

worker 才负责协议解码和业务调用。START/STOP 仍只提交阶段 5 的异步请求；BLE 不复制
启停执行步骤，也不成为新的周期输出源。因此工程中仍只有 `exo_control_loop.c` 的一个
`xTaskDelayUntil()` 负责 100 Hz。

## 5. wire contract 原则

- 不把 C enum、bool、float、指针、结构体 padding 或 `esp_err_t` 直接发给手机；
- 所有字段显式逐字节编码，多字节整数统一 little-endian；
- 关节位置使用 `i32 mrad`，速度使用 `i32 mrad/s`，反馈年龄使用 `u16 ms`；
- v1 最大帧长固定为 20，不要求手机先协商更大 MTU；
- 左/右侧线码固定为 0/1，与内部 enum、A/B 和 CAN ID 解耦；
- `request_id` 固定 1 字节，0 保留；帧中不发送字段名或十进制文本。

完整字节定义见 `BLE_PROTOCOL_V1.md`。

## 6. 请求、幂等和会话

手机必须先订阅通知，再写命令。每次成功连接使 `session_generation` 增加；队列项携带连接
handle 和 generation，worker 丢弃断开连接遗留的过期项。

worker 第一次确认队列项仍属于当前已订阅会话时，该请求取得执行线性化点：若断连事件先于
该检查发生，队列项会被丢弃；若检查先发生，该请求视为已被设备接收，之后即使连接断开也
可继续按“断连不自动 STOP”的语义执行。Notify 的每次重试仍会重新检查 generation、
handle 和订阅状态，绝不向已经失效或被快速复用的新会话发送旧响应。

worker 保存当前会话最近 8 个合法请求的完整请求字节与完整响应字节。精确重复时只重发
缓存响应，避免因无线重试重复触发 START、STOP 或归零；相同 request_id 但字节不同则
返回冲突。响应先编码并缓存，再调用 Notify，因此第一次 Notify 失败后客户端仍可用同一帧
重试并取得同一响应。

断开连接不自动请求 STOP。本阶段沿用阶段 5 的显式请求语义：BLE 是命令传输通道，不是
运行租约。断连会使会话失效、清订阅状态并恢复广播，但已经被系统状态机接收的命令继续按
状态机语义执行。

## 7. 状态和零位适配

产品 BLE 不重新推导 START 就绪条件，而是读取 `exo_system_get_snapshot()` 的
`ready_to_start` 与 `start_block_reason`，再显式映射为稳定线码。

START/STOP 仅调用：

```c
exo_system_request_start();
exo_system_request_stop();
```

ACK 的 ACCEPTED 表示请求已进入异步状态机，不等价于 RUNNING/IDLE。手机随后必须轮询
SYSTEM_STATUS。

软件零位命令在 BLE worker 中先用 `exo_system_try_begin_maintenance()` 取得维护互斥，再
调用单侧 `hip_joint_set_zero_from_current()` 或 `hip_joint_clear_zero()`，所有成功取得维护
权的路径恰好调用一次 `exo_system_end_maintenance()`。它不发送 RS01 机械归零帧。

## 8. 手机应显示的信息

阶段 6 建议至少显示：

- BLE 连接/通知订阅状态；
- 固件版本、协议版本、boot_id；
- 整机 INIT/IDLE/STARTING/RUNNING/STOPPING/ERROR；
- ready_to_start、run_requested、output_allowed、当前步骤和阻塞原因；
- 左/右髋映射、方向、软件零位、反馈有效、在线、joint_valid；
- 左/右髋角度、速度和反馈年龄；
- 最近请求结果、失败步骤和状态迁移计数。

阶段 6 没有参数调整界面：App 可以提供连接、刷新、左右归零、START、STOP；五元目标、
控制模式、增益和助力曲线 UI 等阶段 8/10 协议稳定后再加入。

## 9. 诊断与日志

UART `ble_status` 按需打印：初始化、Host sync、连接、订阅、handle、MTU、会话代际、boot
ID、worker 身份与栈余量，以及 rx/queued/processed、malformed、duplicate、conflict、
queue_full、stale_drop、Notify 同步提交成功/失败及异步 TX 失败计数。

产品 BLE 不把 ESP-IDF 日志文本转发到手机，也不在 100 Hz 循环打印。阶段 10 会另行定义
定频、可控带宽的结构化遥测。

## 10. 阶段 6 完成条件

1. 产品 BLE 是源码中唯一 NimBLE owner，旧 NUS/文本 BLE 不再编译；
2. 纯协议 self-test、畸形帧、重复请求和 ID 冲突测试通过；
3. ESP-IDF v5.5 干净构建通过，帧长静态断言不超过 20；
4. 实机可扫描 `ESP-EXO`，UUID/订阅/GET_INFO/状态查询正确；
5. 未归零时 START 被拒绝，BLE 归零后状态反映正确；
6. START/STOP 各自只进入一次阶段 5 执行序列，重复帧不重复执行；
7. 唯一 100 Hz 控制任务身份不变，BLE 操作不创建第二个周期输出源；
8. 构建、无运动、归零和运动证据写入阶段 6 验收记录后，才可提交、打标签并推送。
