# 阶段 10 Product BLE v1 遥测扩展

阶段 10 保持既有 Product Service、Command characteristic、Response Notify 和 4-byte 命令帧头不变。新增功能通过 capability、可选 characteristic 和新 opcode 向后兼容；不支持遥测的 `0.9.0` 客户端/设备仍可继续使用原命令。

## 1. GATT 与能力位

| 项目 | UUID/值 |
| --- | --- |
| Product Service | `f7a10001-6b5a-4d92-9e7c-3a1c8b640001` |
| Command Write | `f7a10002-6b5a-4d92-9e7c-3a1c8b640001` |
| Response Notify | `f7a10003-6b5a-4d92-9e7c-3a1c8b640001` |
| Telemetry Notify | `f7a10004-6b5a-4d92-9e7c-3a1c8b640001` |
| `TELEMETRY_STREAM` capability | bit 6，`0x0040` |
| `0.10.0` 基线 capability mask | `0x007F` |
| `0.10.1` 产品/工程 capability mask | `0x007F` / `0x00FF` |

客户端只有在 INFO 声明 bit 6 且发现/订阅 Telemetry characteristic 后，才能启动连续遥测。缺少任一条件时应继续使用阶段 9 的低频 GET 状态，不得无限重连。

## 2. 新增命令

所有命令继续使用既有 4-byte 帧头：`version, opcode, request_id, payload_len`，响应仍为 `opcode | 0x80`，ACK/错误仍通过 request ID 关联。

| opcode | 名称 | 请求 payload |
| --- | --- | --- |
| `0x08` | `GET_TELEMETRY_STATUS` | 0 byte |
| `0x17` | `START_TELEMETRY` | 3 bytes：`rate_hz u8, channel_mask u16 LE` |
| `0x18` | `STOP_TELEMETRY` | 0 byte |

`rate_hz` 只允许 `5/10/20`；`channel_mask` 必须非零且不能包含未知位。START/STOP 只控制当前 BLE 连接的遥测会话，不改变外骨骼系统状态。

## 3. `GET_TELEMETRY_STATUS` 响应

响应 payload 固定 16 bytes，因此连同 4-byte 头恰好20 bytes。

| payload offset | 类型 | 字段 |
| --- | --- | --- |
| 0 | u8 | `streaming`，只能0或1 |
| 1 | u8 | `rate_hz`，停止时可为0 |
| 2..3 | u16 LE | `channel_mask` |
| 4..5 | u16 LE | `stream_sequence` |
| 6..9 | u32 LE | `notify_success_count`，本流已成功提交给 NimBLE 的遥测帧数 |
| 10..13 | u32 LE | `dropped_sample_count` |
| 14 | u8 | `queued_samples` |
| 15 | u8 | `queue_capacity` |

`notify_success_count` 的单位是 Notify 帧，不是采样点；一个采样点选择4个通道时，
完整发送会增加4。`queued_samples` 不得大于 `queue_capacity`，当前固定容量为32。
每次重新 START 都建立新流、清零本流计数并使用新的非零 `stream_sequence`；STOP 后
保留最近一次 rate、mask、stream 和计数供诊断。

## 4. Telemetry Notify 固定包

Telemetry characteristic 不套用命令的4-byte帧头，也不携带 request ID。每个 Notify 固定20 bytes：

| offset | 类型 | 字段 |
| --- | --- | --- |
| 0 | u8 | `schema_version=1` |
| 1 | u8 | `packet_type=1 (SAMPLE)` |
| 2 | u8 | 单比特 `channel` |
| 3 | u8 | flags |
| 4..5 | u16 LE | `stream_sequence` |
| 6..9 | u32 LE | `sample_sequence` |
| 10..13 | u32 LE | `timestamp_ms`，设备单调时间低32位 |
| 14..15 | u16 LE | `activation_epoch_low16`，本次 START 助力合同代次低16位 |
| 16..17 | i16/u16 LE | LEFT value |
| 18..19 | i16/u16 LE | RIGHT value |

同一采样时刻的多个通道拥有完全相同的 stream、sample、timestamp、activation
epoch 和状态标志。一个采样点会产生“所选通道数”个 Notify，因此上位机应按
`stream_sequence + sample_sequence` 组合，而不能假定每次只收到一个包。

### 4.1 flags

| bit | 含义 |
| --- | --- |
| 0 | LEFT value valid |
| 1 | RIGHT value valid |
| 2 | 自上一已发送样本后发生设备端丢弃 |
| 3..5 | 当前整机状态线码：INIT/IDLE/STARTING/RUNNING/STOPPING/ERROR |
| 6 | 本周期整机输出门已打开 |
| 7 | 本周期相位助力确实加入最终命令并成功发送 |

有符号通道的无效哨兵为 `INT16_MIN (-32768)`；无符号通道为 `0xFFFF`。valid flag 与哨兵必须一致。

### 4.2 通道与量化

| channel bit | 名称 | 线值 | 上位机单位 |
| --- | --- | --- | --- |
| `0x01` | POSITION | i16 mrad | deg |
| `0x02` | VELOCITY | i16 centi-rad/s | deg/s |
| `0x04` | MOTOR_TORQUE | i16 mN·m | N·m |
| `0x08` | GAIT_PHASE | u16 phase ×10000 | `[0,1)`或% |
| `0x10` | ASSIST_SUGGESTION | i16 mN·m | N·m |
| `0x20` | GAIT_CONFIDENCE | u16 confidence ×10000；阶段10双侧填同一总体值 | `[0,1]`或% |
| `0x40` | APPLIED_TORQUE | i16 实际加入最终命令的助力增量 mN·m | N·m |

已知通道 mask 为 `0x007F`。`MOTOR_TORQUE` 是电机反馈映射到人体关节坐标后的估计力矩；`APPLIED_TORQUE` 不是电机反馈，也不是最终总 `torque_ff`，而是命令合成器本周期实际加入的助力增量。

固件默认 `rate=20 Hz, mask=0x0078`，即相位、建议助力、总体置信度和实际助力，
估算负载为 `20 × 4 = 80 Notify/s`。全通道 `0x007F` 只用于按需诊断和压力测试。

## 5. 会话与错误行为

- 未订阅 Telemetry characteristic 时，START 返回 not-ready/invalid-state，不在后台积累一个无人消费的流；
- 重复 START 明确重启会话、递增 stream sequence并清空该流队列/计数；
- STOP 幂等，不改变外骨骼 RUNNING/IDLE；
- BLE 断开立即停止该流并清理待发送样本，不能自动 STOP 外骨骼；
- 瞬时mbuf不足由BLE低优先级任务保留当前通道并等待恢复，不计为样本丢弃；环形缓冲覆盖、1秒重试预算耗尽或其他最终放弃才进入可查询的样本丢弃计数；
- 异步Notify TX失败保留在UART帧级诊断中；由于回调不携带稳定的stream/sample键，不得把延迟到达的旧会话失败错误归因到新流；
- 未知 packet type/channel 由网页记录原始帧并忽略，不能与命令 ACK 混淆；
- `sample_sequence` 回绕按无符号32位处理；stream sequence 变化表示新会话，网页必须清空旧包重组状态。

## 6. 吞吐量说明

实际 Notify 频率为：

```text
notify_per_second = sample_rate_hz * selected_channel_count
```

例如20 Hz选择5个通道时是100 Notify/s。网页必须显示该估算值；固件默认 mask 应选择完成常用观察所需的最小集合，全 `0x007F` 用于按需诊断/压力测试。若链路拥塞，允许丢遥测但不能影响100 Hz控制任务。

## 7. `0.10.1` 工程参考源协议

`0.10.1` 保持4-byte命令头、既有opcode和Telemetry帧不变，仅在工程构建中扩展：

- `ALTERNATING_SINE_TEST`参考源线码为`2`；仍通过现有
  `SET_CONTROL_CONFIG (0x14)`的第二个payload字节选择；
- `ENGINEERING_GAIT_STIMULUS` capability为bit 7（`0x0080`）；
- 工程开关开启时INFO capability mask为`0x00FF`，关闭时仍为`0x007F`。

手机只有在INFO声明bit 7后才能显示该选项。工程轨迹固定为单侧10 deg、0.5 Hz、
3 s渐入和左右反相，手机无权通过BLE修改这些参数，也不持续发送轨迹点。

协议能解析参考源不等于运行合同允许。`ALTERNATING_SINE_TEST + ACTIVE`必须在
START前映射为`REJECTED_NOT_READY`，不得先使能电机。测试验收只能配置
`DISABLED`或`SHADOW`。BLE断开和
`STOP_TELEMETRY`都不等于整机STOP；自动化工具
必须在`finally`中显式发送整机`STOP (0x11)`。

助力不可用原因新增稳定线码`WAITING_NEW_GAIT_EVENTS=8`。它表示步态可以有效，但本轮
activation尚未分别观察到LEFT/RIGHT新的髋屈曲峰；这不是传感器故障。此状态下建议
和实际附加助力均为0，客户端不得把START前遗留的LOCKED相位显示成已经开始助力。
