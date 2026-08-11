# 阶段 9：统一传感快照、步态估计与高层助力策略框架

## 1. 文档范围

适用固件版本：`0.9.0`，目标平台：ESP32-S3 + ESP-IDF v5.5。

阶段 9 的目标不是直接产生真实助力，而是先把后续论文算法都需要的三层接口固定下来：

1. 把一个 100 Hz 周期看到的左右髋数据整理成一份有时间、有来源、有质量标志的统一传感快照；
2. 用当前硬件能够提供的双髋角度/速度，实现一个轻量的连续步态相位代理 `HIP_EVENT_TBE`；
3. 建立高层助力模式与策略的静态框架，在 `SHADOW` 状态下计算和观察建议，但不改变阶段 8 已验收的真实 CAN 输出。

本文只描述当前源代码已经存在的行为。自适应振荡器、足底时序/HMM、关节 LSTM、
相位助力曲线和按需辅助残差控制都是后续可替换方向，不是阶段 9 已实现算法，详见
[PHASE_9_LITERATURE_MAP.md](PHASE_9_LITERATURE_MAP.md)。

## 2. 总体数据流

全工程仍只有一个永久 100 Hz 控制任务。每个 10 ms 周期按固定顺序执行：

```text
xTaskDelayUntil（绝对节拍）
        |
        v
读取 LEFT/RIGHT hip_joint 完整快照
        |
        v
exo_sensor_snapshot_build()
  - 统一 sequence / captured_us / reset_epoch
  - 计算新鲜度、左右来源时间差、valid/quality mask
        |
        v
exo_gait_estimator_update()
  - 当前后端：HIP_EVENT_TBE
  - 输出左右 phase、period、cadence、confidence、quality
        |
        v
exo_assist_process_tick()
  - 读取同一周期 sensor + gait
  - 最多生成一对 SHADOW 五元建议
  - 阶段 9 结果不进入真实输出链
        |
        v
exo_system_process_control_tick()
  - 继续走阶段 8 已验收的 controller/joint_command/RS01 路径
        |
        v
发布双缓冲诊断快照
```

算法链放在系统输出门之前，因此设备处于 `IDLE` 时仍可观察传感质量、站立和起步。
这不等于 `IDLE` 可以产生电机输出：真实输出许可仍完全归 `exo_system` 所有。

### 2.1 模块所有权

| 模块 | 主要职责 | 明确不负责的事情 |
|---|---|---|
| `exo_sensor_snapshot.*` | 整理本周期输入、判定数据新鲜度与来源一致性 | 不读取硬件、不创建任务、不估计步态 |
| `gait_estimator.*` | 从统一快照计算步态状态、相位和置信度 | 不读取计时器、不发 CAN、不切换整机状态 |
| `exo_assist.*` | selected/active 配置、静态策略表、SHADOW 建议和诊断 | 不拥有 START/STOP、不发 BLE/CAN、不打开真实输出 |
| `exo_control_loop.*` | 唯一 100 Hz 调度、调用顺序、耗时与完整诊断发布 | 不接受手机实时逐点控制 |
| `exo_system.c` | START/STOP 生命周期、输出门和 arm/disarm 边界 | 不实现步态算法公式 |
| `exo_ble_protocol.*` | 稳定的字节线格式 | 不直接暴露内部 C 结构布局 |
| `debug_console.c` | 面向开发者的按需诊断和 IDLE 配置 | 不创建新的周期采样源 |

## 3. 统一传感快照

### 3.1 为什么不能把“最近两份反馈”直接交给算法

TWAI 接收任务与 100 Hz 控制任务异步运行。控制任务在同一次唤醒中复制到的左右反馈，
不一定是硬件同时采样的数据。阶段 9 因此把它们称为“同一控制周期使用的一对最新完整
反馈”，而不是“同步采样”。快照同时保留左右 `source_timestamp_us` 和
`pair_skew_us`，避免上层算法误判。

### 3.2 快照核心字段

`exo_sensor_snapshot_t` 的重要字段如下：

| 字段 | 含义 |
|---|---|
| `sequence` | 每次构建递增，`0` 保留为“尚无数据” |
| `captured_us` | 唯一 100 Hz 任务本周期的统一时间 |
| `reset_epoch` | 坐标签名、反馈来源或统一时间基准发生复位时递增 |
| `available_sensor_mask` | 本固件/硬件声明可以提供的传感能力 |
| `valid_sensor_mask` | 当前这个周期实际满足有效条件的能力 |
| `quality_flags` | stale、unavailable、时间异常、坐标重置等原因位，可同时置多个 |
| `pair_skew_us` | 左右反馈来源时间戳的绝对差，不代表硬件同步误差已被补偿 |
| `left/right` | 人体髋关节坐标下的角度、角速度、RS01 电机估算力矩及来源元数据 |

当前能力位只声明双髋运动量和左右 RS01 电机估算力矩：

```text
bit 0  LEFT_HIP_KINEMATICS
bit 1  RIGHT_HIP_KINEMATICS
bit 2  LEFT_MOTOR_TORQUE_ESTIMATE
bit 3  RIGHT_MOTOR_TORQUE_ESTIMATE
bit 4  LOWER_LIMB_IMU             （预留，当前无样本字段）
bit 5  FOOT_CONTACT               （预留，当前无样本字段）
bit 6  INTERACTION_TORQUE          （预留，当前无样本字段）
bit 7  SEMG                        （预留，当前无样本字段）
```

重要区别：RS01 返回的电机估算力矩包含电机、减速器、摩擦和外骨骼动力学影响，不能改名为
“人体主动关节力矩”或“人机交互力矩”。未来 AAN 若要求交互力矩，必须接入独立且完成标定的
传感通道，不能用 bit 2/3 冒充 bit 6。

### 3.3 当前有效条件和固定阈值

某侧样本只有同时满足以下条件，才会把该侧运动量/估算力矩加入 `valid_sensor_mask`：

- `snapshot_available`、`joint_valid`、`feedback_valid`、`online` 均为真；
- 角度、角速度、估算力矩和温度均为有限浮点数；
- 来源时间不为 0、不晚于 `captured_us`；
- 数据年龄不超过 `50,000 us`。

左右来源时间差超过 `20,000 us` 时置 `PAIR_SKEW_HIGH`。该标志用于降权和诊断，
不会伪造一份插值后的“同步样本”。

### 3.4 `reset_epoch` 的用途

以下事件会推进 `reset_epoch`，提示有历史状态的算法清空旧周期：

- RS01 连续位置 `position_epoch` 改变；
- RAM 软件零位来源、零位绑定 epoch 或零位有效性改变；
- 某侧反馈计数/来源时间戳回退；
- 统一 `captured_us` 首次进入停滞或回退状态。

普通的反馈暂缺或单侧时间戳为 0 只会让该周期数据无效，不会每 10 ms 制造一次复位风暴。

## 4. 当前步态估计器：`HIP_EVENT_TBE`

### 4.1 输入和事件语义

当前估计器只消费统一快照中的左右髋角度和角速度。人体坐标约定仍是“髋屈曲为正”：

- 滤波速度由正转负，且区间摆幅/时间门槛满足时，报告 `HIP_FLEXION_PEAK`；
- 滤波速度由负转正时，报告 `HIP_EXTENSION_PEAK`；
- 同侧相邻两个有效屈曲峰之间的时间作为该侧候选步态周期；
- 当前连续相位原点固定为 `HIP_FLEXION_PEAK`。

这些事件是编码器波形的代理事件，**不是**足跟着地 `HEEL_STRIKE`，也不是足尖离地
`TOE_OFF`。BLE/UART 不得给它们换成更强的生物力学名称。

### 4.2 计算过程

1. 只有来源时间戳严格前进的新反馈才进入一阶低通，避免同一反馈在多个 100 Hz 周期中被
   重复滤波；
2. 速度趋势使用 `+/-5 deg/s` 滞环，运动检测阈值为 `4 deg/s`；
3. 峰值事件至少需要 `6 deg` 区间摆幅，且距上次事件至少 `0.15 s`；
4. 同侧周期只接受 `0.5~3.0 s`；
5. 周期按 `alpha=0.25` 做 EWMA；相邻周期相对误差超过 `35%` 时标记不稳定并重新累计；
6. 连续接受 2 个稳定周期后，该侧相位才可用；
7. 相位按“距最近屈曲峰的时间 / 当前周期”推进，在等待迟到事件时最多保持在接近 1；
8. 超过“预计周期 + 0.3 s”仍无新原点事件，该侧相位超时失效；
9. 双侧周期差超过 `25%` 或循环相位距离小于 `0.15` 时保留结果、置双侧不一致质量位，
   并降低置信度，而不是强制把左右相位改成相差 0.5；
10. 连续 `1.5 s` 未检测到运动时进入 `STANDING` 并清空旧周期历史。

当前 cadence 使用每侧同类髋峰周期换算步频，双侧有效时取平均；confidence 是基于周期稳定、
锁定次数、单/双侧状态和时间差质量位的工程置信度，不是经过临床标定的概率。

### 4.3 估计器状态

| 状态 | 含义 |
|---|---|
| `RESET` | 刚初始化或收到新 `reset_epoch`，没有可继承历史 |
| `STANDING` | 双侧输入有效，但运动量长期低于阈值；相位无效 |
| `ACQUIRING` | 正在积累峰值和稳定周期；允许出现单侧阶段性结果 |
| `LOCKED` | 左右相位均有效，`estimate.valid=true` |
| `DEGRADED` | 至少一侧输入无效，清空对外有效量，不沿用陈旧相位 |

`valid_flags` 分别描述 LEFT phase、RIGHT phase、cadence、confidence 是否可用；调用者不能只看
浮点字段是否非零。单侧 phase 可用时会置 `SINGLE_SIDE_ONLY`，但整体 `valid` 仍为假。

## 5. 高层助力策略框架

### 5.1 高层模式与阶段 8 基础模式不是同一层

阶段 8 的 HOLD/POSITION/VELOCITY/TORQUE/DAMPING/IMPEDANCE 描述“RS01 五元命令怎样
执行”。阶段 9 的 TRANSPARENT/ACTIVE_PHASE/ASSIST_AS_NEEDED 描述“高层策略为什么生成
一对建议”。两组枚举不能合并。

当前高层模式及策略 ID：

| 高层模式 | 策略 ID | 当前实现 | 声明的输入依赖 |
|---|---|---|---|
| `TRANSPARENT` | `TRANSPARENT` | 已实现，仅 SHADOW | 左右髋运动量；不要求步态 |
| `ACTIVE_PHASE` | `PHASE_PROFILE` | 未实现占位 | 左右髋运动量、左右 phase、confidence>=0.40 |
| `ASSIST_AS_NEEDED` | `AAN_RESIDUAL` | 未实现占位 | 上述步态量 + 人机交互力矩、confidence>=0.60 |

透明策略当前建议为“位置跟随本周期髋角，速度/前馈力矩/KP/KD 为零”。它的用途是验证策略接口、
输入序号绑定和 SHADOW 诊断，不代表已经实现低阻抗透明控制效果。

### 5.2 静态 `ops` 表和 64-byte 工作区

ESP32 固件不在运行时装载插件。可替换策略由编译期常量表描述：

```c
typedef struct {
    esp_err_t (*reset)(...);
    esp_err_t (*step)(...);
} exo_assist_policy_ops_t;
```

每个描述符固定声明策略 ID、版本、支持模式、所需 sensor mask、所需 gait valid flags、最低置信度、
工作区字节数和 `implemented`。运行期只有一个静态对齐的 64-byte union 工作区：

- 透明策略声明 0 byte 历史状态；
- 相位曲线预留最多 32 byte 可变状态；曲线常量本身应放只读 Flash，不复制到工作区；
- AAN residual 预留最多 64 byte 可变状态；若未来模型状态超过该上限，必须通过资源评审后显式修改，
  不能静默使用堆内存。

策略 `reset/step` 的实时合同是：无动态内存、无阻塞等待、无日志、无 BLE、无 CAN、无任务创建。
它们只接收本周期 sensor/gait，并返回绑定 `source_sensor_sequence`、`source_gait_sequence`、
`activation_epoch` 的结果。

### 5.3 selected/active 与 START/STOP

- `IDLE`：BLE/UART 只能修改 `selected`；
- START：`exo_system` 先 arm 阶段 8 控制器，再调用 `exo_assist_arm()`，将 selected 锁存为 active；
- RUNNING：配置锁定，旧的 IDLE 写请求返回 BUSY；每个周期最多生成一份当前 activation epoch 的建议；
- STOP：先 `exo_assist_disarm()` 使建议失效，再 disarm 底层控制器；selected 留在 RAM 供下次使用；
- 重启：恢复编译期默认 `TRANSPARENT + DISABLED + 0%/0%`，不写 NVS。

如果 STOP/下一次 START 恰好发生在策略锁外计算期间，发布端会比较 `activation_epoch`，丢弃跨边界的
旧结果，避免刚被 STOP 清除的 suggestion 再次变为有效。

## 6. `SHADOW` 边界

阶段 9 的硬边界如下：

- `EXO_ASSIST_EXEC_ACTIVE` 可以被协议表达，但配置入口返回 `ESP_ERR_NOT_SUPPORTED`；
- `real_output_enabled` 在当前实现中始终为 `false`；
- 相位曲线和 AAN 策略的描述符为 `implemented=false`、`ops=NULL`；
- `exo_assist_process_tick()` 的返回值只写入控制诊断快照；
- 真实 CAN 路径仍是 `exo_system -> exo_controller -> exo_joint_command -> rs01_driver`；
- BLE 断开不会改变输出许可，也不会把 SHADOW 建议接入控制器。

因此，网页上“助力等级”在阶段 9 只是版本化配置和界面占位，不是实际电机助力百分比。阶段 10 若要
打开真实输出，必须新增明确的连接点、幅值/变化率边界和独立验收，不能只把布尔值改成真。

## 7. BLE 产品接口

协议仍使用 v1 的 `version/opcode/request_id/payload_len` 四字节帧头，最大完整帧 20 byte；整数均按
little-endian 编码。阶段 9 新增能力位：

- bit 4：`GAIT_STATUS`；
- bit 5：`ASSIST_CONFIGURATION`；
- 当前 `GET_INFO.capabilities` 为 `0x003F`。

### 7.1 新操作码

| 请求 | opcode | 请求 payload | 响应 |
|---|---:|---:|---|
| `GET_GAIT_STATUS` | `0x06` | 0 byte | `0x86` + 16-byte gait payload |
| `GET_ASSIST_CONFIG` | `0x07` | 0 byte | `0x87` + 16-byte assist payload |
| `SET_ASSIST_CONFIG` | `0x16` | 8 byte | `0x96` + 4-byte通用 ACK |

16-byte gait payload 包含 estimator/state/flags/origin、quality、左右 `phase_x10000`、
`cadence_x100`、`confidence_x10000` 和估计年龄。无效定点量使用 `0xFFFF`，不能解释成 655.35 或
100% 以上的真实值。

16-byte assist payload 包含 selected/active mode、同一字节低/高半字节编码的
requested/effective execution、strategy/version/profile、左右等级、
状态 flags、不可用原因、config generation 低 8 位、required/available sensor mask。

8-byte SET payload 是：

```text
schema_version, selected_mode, requested_execution_state, strategy_id,
profile_id, left_level_percent, right_level_percent, reserved_flags
```

阶段 9 要求 schema=1、等级 0~100、reserved=0、mode 与 strategy 匹配；维护门只允许在 IDLE 写入。
手机应先 GET_INFO 检查能力，再 GET/SET/GET 读回，不能假定所有 `0.9.x` 设备拥有相同可选策略。

## 8. UART 开发接口

阶段 9 新增四个命令：

```text
sensor_status
gait_status
assist_status
assist_config <mode> <disabled|shadow> <strategy> <profile> <left%> <right%>
```

示例：

```text
assist_config transparent shadow transparent 0 0 0
assist_status
```

`sensor_status` 和 `gait_status` 可随时只读；`assist_config` 经过整机 maintenance 门，只允许 IDLE。
UART 解析器故意不提供 `active` 参数，开发者不能绕过产品业务层打开阶段 9 未授权输出。

## 9. 实时与并发约束

- `exo_sensor_snapshot_builder_t` 和 `exo_gait_estimator_t` 只由唯一 100 Hz 任务持有，不需要跨任务锁；
- 助力 selected/active 与诊断用短临界区复制，策略 `step()` 在锁外执行；
- 控制诊断用双缓冲发布，BLE/UART 读取最近一份完整快照，不会看到半新半旧字段；
- 每个算法阶段记录 last/max 耗时，另有 `algorithm_error_count` 和最后错误码；
- 控制任务静态栈为 4096 byte，不因阶段 9 创建第二个周期任务或动态算法对象；
- 步态算法内部不调用 `esp_timer`，回放时间完全来自快照，所以同一输入序列应得到确定输出。

## 10. 文件索引与维护规则

| 文件 | 阶段 9 维护重点 |
|---|---|
| `main/include/exo_sensor_snapshot.h` / `main/exo_sensor_snapshot.c` | 传感能力、质量位、统一快照构建 |
| `main/include/gait_estimator.h` / `main/gait_estimator.c` | 后端 ID、状态、相位输出合同和 HIP_EVENT_TBE |
| `main/include/exo_assist.h` / `main/exo_assist.c` | 高层模式、静态策略描述符、64-byte 工作区、SHADOW 生命周期 |
| `main/exo_control_loop.c` | 唯一 100 Hz 调用顺序和诊断发布 |
| `main/exo_config.c` | 编译期传感/步态阈值 |
| `main/exo_system.c` | START/STOP 的 assist arm/disarm 边界 |
| `main/include/exo_ble_protocol.h` / `main/exo_ble_protocol.c` | 稳定线码和逐字节编解码 |
| `main/exo_ble.c` | 线 DTO 与内部对象适配、IDLE 业务门 |
| `main/debug_console.c` | UART 只读诊断和维护配置 |

后续替换算法时，应先保持 `exo_sensor_snapshot_t -> exo_gait_estimate_t -> exo_assist_result_t` 三层合同，
再新增静态后端。不要让论文算法直接读取 RS01 全局变量、直接发 CAN，或自行创建另一条周期控制链。
