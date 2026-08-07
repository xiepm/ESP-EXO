# 阶段 6 产品 BLE 验收记录

验收日期：2026-08-07
当前结论：**软件、构建、烧录、BLE 归零、幂等、断连语义及 START/STOP 运动验收全部通过，阶段 6 可以封版。**

## 1. 验收边界

阶段 6 从阶段 5 标签 `phase5-system-state-start-stop` 开始。阶段 5 标签和起始 HEAD 均解析为：

```text
0ffe8ab10275abb6480763fd9e897ddd0ba5cee9
```

用户自有未跟踪目录 `web-bluetooth-console/` 和 `.codex_tmp/` 不属于本阶段修改范围，未编辑、
未删除，也不会纳入后续提交。

## 2. 已实现内容

- 产品 GATT Service、Command Write 和 Response Notify；
- v1 4 字节头、最大 20 字节的纯二进制协议；
- GET_INFO、整机/左右髋状态、START、STOP、单侧捕获/清除 RAM 软件零位；
- 1 字节 request_id、最近 8 个请求的精确重放和 ID 冲突拒绝；
- 连接 generation 和过期队列项丢弃；
- UART 工程调试台与产品 BLE 分离；
- 原 42 条工程命令完整迁移到 UART，新增只读 `ble_status`；
- 旧 NUS 文本 BLE 源文件移除，产品 BLE 成为唯一 NimBLE owner；
- 一个静态 BLE worker 和静态深度 8 队列；
- 手机验收工具 `tools/phase6_ble_acceptance.py`，默认无运动，并为归零/启停提供三重确认、
  预期拒绝、危险帧重复和紧急只执行目标命令等显式选项。

## 3. 静态与离线验收

### 3.1 边界扫描

| 检查 | 结果 |
|---|---:|
| `xTaskDelayUntil()` 数量 | 1，仅 `exo_control_loop.c` |
| `nimble_port_init()` 数量 | 1，仅 `exo_ble.c` |
| 产品源码中的旧 NUS UUID/`RobStride-CAN` | 0 |
| `debug_console` 中 NimBLE/GATT 依赖 | 0 |
| `exo_ble` 中 RS01/TWAI/阻抗低层调用 | 0 |
| 纯协议模块平台依赖 | 0（仅注释提及 `esp_err_t`） |
| 旧/新 UART 命令比对 | 原 42 条全部一致，另加 `ble_status` |
| clang-format `--Werror` | PASS |
| `git diff --check` | PASS |

协议模块逐字段编码，不发送 C struct、enum、bool、float、padding、指针或 `esp_err_t`。

### 3.2 离线协议自测

命令：

```powershell
python tools/phase6_ble_acceptance.py --self-test
python -m py_compile tools/phase6_ble_acceptance.py
```

结果：9/9 PASS，退出码 0。覆盖请求黄金帧、INFO/SYSTEM/有符号 JOINT 响应、冲突 ACK、
严格长度拒绝、默认计划不含运动/归零 opcode，以及危险命令确认门。

### 3.3 独立代码审计

最终审计未发现 P0/P1/P2 阻断问题，确认：

- mbuf chain 完整复制，Notify 每次重试重新创建 mbuf；
- Notify 重试重新检查 generation、handle 和订阅状态；
- START 成功类 ACK 的 detail 为 NONE，拒绝时才给阻塞原因；
- response-bit 已置位的畸形 opcode 仍保持 `response=request|0x80`；
- 软件归零成功取得维护权后，所有路径恰好调用一次 `exo_system_end_maintenance()`；
- 左右线码 0/1 显式映射，不直接 cast 内部 enum；
- 断连和 Host reset 均不隐式 STOP。

保留的 P3 硬化项：若未来需要 `exo_ble_init()` 失败后原地重试，应补 NimBLE/队列/任务回滚；
当前唯一调用由 `ESP_ERROR_CHECK()` 包裹，失败即重启，且 `ble_status.host_synced` 与实机扫描
会暴露 Host task 未启动，因此不阻断本阶段。

## 4. 构建与烧录

环境：

```text
ESP-IDF v5.5
xtensa-esp-elf-gcc 14.2.0 (esp-14.2.0_20241119)
target: esp32s3
```

`idf.py fullclean` 后完成全量编译；最终协议修正又触发 `exo_ble.c`、链接和镜像重建，结果 PASS。

| 产物 | 大小 | SHA-256 |
|---|---:|---|
| `build/twai_network_master.bin` | 627,472 bytes (`0x99310`) | `3B24D9A90C86308AF225CFE37009ECE6134C2292BE4C8FAE75D2BE5B7A2E073E` |
| `build/twai_network_master.elf` | 7,508,452 bytes | `0BFE5257193DB25172EDA3323483A5628EE141B3ADEA32050C80BE8B0C9622F4` |

最小 app 分区为 `0x100000`，剩余 `0x66CF0`（40%）。Bootloader 剩余 36%。

最终镜像通过 COM5 写入 ESP32-S3（MAC `44:1b:f6:fe:22:c0`），bootloader、partition table
和 app 三个区域均完成写入并通过 esptool SHA 校验，随后硬复位进入应用。

## 5. 无运动 UART 验收

烧录后的 RAM 软件零位按设计无效。查询 `ble_status`、`system_status`、左右 `joint_status`、
`control_status` 和 `can_status`，确认：

| 项目 | 实测 |
|---|---|
| 系统状态 | IDLE |
| Ready to start | no |
| Start block | LEFT_ZERO_INVALID |
| Run requested / Output allowed | no / no |
| 左/右映射 | B/+1、A/-1，均 valid |
| 左/右反馈 | valid、online |
| 左/右软件零位 | INVALID |
| 左/右 joint_valid | no |
| 控制任务 | 单一 identity `0x3fc9f844` |
| 控制频率 | 100.000 Hz（目标 100 Hz） |
| 周期超出 ±1000 us | 0 |
| Deadline miss / skipped / output error | 0 / 0 / 0 |
| TWAI | RUNNING |
| TX/RX error、TX fail、bus error、arbitration lost、RX missed | 全部 0 |

这证明烧录后不会沿用旧 RAM 零位，控制任务继续运行但没有开放运动输出。

## 6. 产品 BLE 无运动验收

使用 Windows Bluetooth + Bleak 3.0.2，命令：

```powershell
python tools/phase6_ble_acceptance.py `
  --name ESP-EXO `
  --scan-timeout 15 `
  --connect-timeout 15 `
  --response-timeout 5
```

最终默认套件 15/15 PASS：

- 发现 `ESP-EXO`，地址 `44:1B:F6:FE:22:C2`；
- 连接并成功订阅 Response Notify；
- GET_INFO：v1，固件 `0.6.0`，max frame 20，capabilities `0x0007`；
- GET_SYSTEM_STATUS：IDLE，flags `0x21`，transition count 1；
- 左/右 GET_JOINT_STATUS：flags `0x1B`，反馈在线但零位/joint 尚无效；
- 完全相同请求返回逐字节相同的缓存响应；
- 相同 request_id 的不同请求返回 `0x25 REQUEST_ID_CONFLICT`；
- unknown opcode、response-bit opcode、错误版本、声明长度不符、非法 side、request_id=0
  六类畸形帧均返回正确 ACK。

最终一次默认套件后的设备诊断与 14 个实际写入帧完全对应：

```text
RX/queue/process       14/14/14
malformed/duplicate/id 6/1/1
queue full/stale       0/0
notify submit ok/fail  14/0
notify TX failures     0
```

### 6.1 未归零 START 门控

显式使用危险命令三重确认，并要求结果必须是 NOT_READY：

```powershell
python tools/phase6_ble_acceptance.py `
  --name ESP-EXO `
  --allow-dangerous `
  --dangerous-command start `
  --confirm-dangerous "ESP-EXO STAGE6 DANGEROUS" `
  --expect-rejected-not-ready
```

结果：16/16 PASS，START ACK 为：

```text
result = 0x10 REJECTED_NOT_READY
state  = 1 IDLE
detail = 10 LEFT_ZERO_INVALID
```

首次门控调用时设备已经正确拒绝，但旧版脚本只把 START 成功结果视为 PASS，因而工具层退出
为 1；增加“必须被 NOT_READY 拒绝”的显式选项后重新执行通过。这解释了最终 UART 中
`START requests=2`、`Rejected requests=2`。两次请求均未进入启停执行器：系统 transition
count 始终为 1，状态始终为 IDLE，run/output 始终为 no，控制输出错误与 CAN 错误保持 0。

## 7. BLE 归零验收

操作者在本次 boot ID `0x1E5B580B` 下明确确认双髋处于中立位后，分别通过产品 BLE 执行
CAPTURE_ZERO left/right。每次都带 `--repeat-dangerous`，工具在首次 ACK 后原样重发同一
request_id 和完整帧，并要求缓存响应逐字节相同。

| 项目 | 左髋 | 右髋 |
|---|---:|---:|
| CAPTURE_ZERO result | `0x00 OK` | `0x00 OK` |
| 软件零位 | `5.380 rad` | `-6.176 rad` |
| 零位来源反馈计数 | 138616 | 142249 |
| 归零后关节角 | 约 `0.000 rad` | 约 `0.000 rad` |
| `joint_valid` | YES | YES |
| 重复响应 | byte-exact PASS | byte-exact PASS |

左髋归零后系统阻塞原因从 LEFT_ZERO_INVALID 正确切换为 RIGHT_ZERO_INVALID；右髋归零后：

```text
State          IDLE
Ready to start YES
Start block    NONE
Run requested  no
Output allowed no
```

两侧零位来源反馈计数在后续全部测试中保持不变；BLE duplicate 计数分别增加，证明重复帧
命中响应缓存而未再次进入归零业务。

## 8. BLE START、断连和 STOP 运动验收

### 8.1 START 与重复帧

BLE START 首次响应：

```text
result = 0x01 ACCEPTED
state  = 2 STARTING
detail = 0 NONE
```

同一 START 帧立即重发，返回逐字节相同的缓存响应。系统最终为 RUNNING：

```text
Run requested  YES
Output allowed YES
START requests 3  （其中 2 次为归零前门控测试，仅 1 次进入执行器）
START success  1
```

因此 START 重复帧没有把系统请求数从 3 增加到 4。

### 8.2 断连保持运行

START 客户端主动断开后，UART 仍观测到：

```text
BLE connected   no
System state    RUNNING
Output allowed  YES
Left position   0.392 rad
Right position  0.388 rad
```

这直接验证阶段 6 的连接语义：BLE 是显式命令通道，不是运行租约，断连不隐式 STOP。

RUNNING 到 STOP 请求的状态机时间戳差约 65.5 秒，超过原计划约 10 秒。原因是 Windows BLE
扫描/连接取证以及一次主机端 UART 采样命令引号错误延长了取证时间；该主机命令在解析阶段
失败，没有向设备发送内容。延长期间设备未报告 deadline miss、skipped cycle、output error
或 CAN error。

运动中采样：

| 项目 | 结果 |
|---|---:|
| 控制任务 identity | `0x3fc9f844`（与运动前一致） |
| 周期均值 | 100.000 Hz |
| 周期超出 ±1000 us | 0 |
| Deadline miss / skipped / output error | 0 / 0 / 0 |
| TWAI/CAN 全部错误计数 | 0 |
| BLE queue full / stale / Notify failure | 0 / 0 / 0 |

### 8.3 STOP 与重复帧

为缩短已开始的运动，工具使用三重确认加 `--dangerous-only`，跳过默认查询套件并在重连后
直接 STOP。首次响应：

```text
result = 0x01 ACCEPTED
state  = 4 STOPPING
detail = 0
```

相同 STOP 帧的重复响应逐字节一致。最终 UART 和 BLE 均确认：

```text
State           IDLE
Ready to start  YES
Run requested   no
Output allowed  no
Transitions     5
START requests  3, success 1
STOP requests   1, success 1
```

INIT→IDLE、IDLE→STARTING、STARTING→RUNNING、RUNNING→STOPPING、STOPPING→IDLE 共 5 次
迁移与预期一致。重复 START/STOP 均未增加系统请求数。

停止后的最终 BLE 默认套件再次 15/15 PASS：系统为 IDLE/flags `0x23`/transitions 5，
左右 joint flags 均为 `0x3F`。最终设备诊断：

```text
RX/queue/process       108/108/108
malformed/duplicate/id 42/11/7
queue full/stale       0/0
notify submit ok/fail  108/0
notify TX failures     0

control period mean    100.000 Hz
period outside limit   0
deadline/skipped/error 0/0/0
CAN error counters     all 0
```

## 9. 最终判定

## 8. 当前判定

| 验收组 | 状态 |
|---|---|
| 架构/静态边界 | PASS |
| 离线协议与危险门 | PASS |
| ESP-IDF 构建 | PASS |
| COM5 烧录与镜像校验 | PASS |
| BLE 发现/GATT/查询 | PASS |
| 去重/冲突/畸形帧 | PASS |
| 未归零 START 门控 | PASS |
| BLE 软件归零及重复帧 | PASS |
| BLE START 及重复帧 | PASS |
| BLE 断连保持运行 | PASS |
| BLE STOP 及重复帧 | PASS |
| 停止后 BLE/控制/CAN 回归 | PASS |
| 提交/标签/推送 | READY：使用 `phase6-product-ble-protocol` 封版 |
