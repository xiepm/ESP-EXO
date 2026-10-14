# 阶段 10：相位助力曲线、唯一命令合成与产品遥测

当前适用固件版本：`0.10.1`；`0.10.0`保留为阶段10初始基线。目标平台仍是
ESP32-S3 + ESP-IDF v5.5。

## 1. 本阶段交付边界

阶段 9 已经把同一 100 Hz 周期的左右髋反馈整理为传感快照，并产生左右独立的连续步态相位。阶段 10 只在这个稳定接口之后完成两件事：

1. `PHASE_PROFILE` 根据左右相位产生人体髋关节坐标下的附加前馈力矩；
2. 最终命令发送后生成结构化遥测，由低优先级传输路径上报给网页。

本阶段不实现 `ASSIST_AS_NEEDED/AAN_RESIDUAL`，不增加实验条件、实验轮次或实验调度模块，也不创建第二个周期控制任务。手机只配置低频参数和遥测流，不参与 100 Hz 命令生成。

## 2. 唯一实时数据链

```text
100 Hz control task
  sensor snapshot
    -> gait estimator
    -> PHASE_PROFILE suggestion
    -> base controller evaluation
    -> command composer (base torque_ff + assist torque_ff)
    -> one pair validation and one pair CAN send
    -> telemetry sample push

low-priority I/O task
  telemetry ring pop -> BLE/UART transport
```

实时路径禁止动态内存、日志、Flash/NVS 写入、BLE Notify 和无界等待。策略模块不能调用 CAN；BLE 回调不能成为新的控制源。

## 3. 相位曲线合同

### 3.1 输入、输出与单位

每一侧 `s` 的助力力矩定义为：

```text
tau_assist_s = ramp * level_s / 100 * tau_peak_s * curve_s(phase_s)
```

- `phase_s`：该侧自己的相位，范围 `[0, 1)`；
- `curve_s()`：只读 Flash 中的周期归一化曲线，输出范围 `[-1, 1]`；
- `level_s`：IDLE 中配置并在 START 时锁存的 `0..100 %`；
- `tau_peak_s`：曲线元数据声明的峰值，单位 `N·m`；
- `ramp`：START 后的有界渐入因子；
- 输出使用人体髋关节坐标，屈曲为正，不使用电机 A/B 原始方向。

相位策略只建议 `torque_ff_nm`。最终命令中的 `position_deg`、`velocity_deg_s`、`kp` 和 `kd` 必须保持基础控制器结果，左右映射和电机符号继续只由 `hip_joint`/`exo_joint_command` 处理。

### 3.2 相位原点

当前估计器的 `phase_origin` 是每侧各自的髋屈曲峰值。因此 LEFT 必须消费 `left_phase_01`，RIGHT 必须消费 `right_phase_01`，禁止用 `right = left + 0.5` 代替右侧估计。

若论文曲线以足跟着地、脚尖离地或其他事件为零相位，导入前必须保存论文原点并做显式偏移换算。没有原点证据时只能登记为工程测试曲线，不能声称已经复现论文。

### 3.3 曲线实现

- 当前使用16个只读 `float` 归一化采样点，数组位于 Flash；后续文献曲线若改用
  定点表，必须升级 profile version 并重新执行量化误差验收；
- 使用周期线性插值，计算量为 O(1)；
- 相位、曲线值、缩放值和最终力矩都必须检查 `NaN/Inf`；
- 低置信度、无效相位、错误 START 代次或过期输入不得进入真实输出；
- 步态暂时失效时立即使本周期建议无效并清除策略历史；重新有效后从0重新渐入，
  STOP 则立即关闭整机输出门；
- 每条曲线必须声明 ID、版本、来源、原点、方向、峰值、采样点数和是否允许 ACTIVE。

仓库随阶段 10 提供两条工程曲线：`profile 0 ZERO_DIAGNOSTIC_V1` 恒为0，
`profile 1 ENGINEERING_TEST_V1` 是16点正弦形工程波，100%峰值仅0.5 N·m。
两者都明确标记 `literature_profile=false`，不得描述为论文复现结果。策略从首次有效
步态开始执行1秒五次多项式渐入，并限制每10 ms单侧变化不超过0.01 N·m。

## 4. SHADOW、ACTIVE 与基础控制模式

| 执行态 | 行为 |
| --- | --- |
| `DISABLED` | 不运行策略，不产生可用建议，不改变基础输出 |
| `SHADOW` | 计算、记录并上报建议力矩，但最终 CAN 命令保持基础控制器结果 |
| `ACTIVE` | 只有本周期建议通过代次、序号、相位、置信度和数值检查后，命令合成器才加入前馈力矩 |

阶段 10 第一版只允许 `ACTIVE_PHASE + PHASE_PROFILE + ACTIVE` 与 `DAMPING + CONSTANT` 组合。`IMPEDANCE + SINE` 仍用于阶段 7/8 回归，不能叠加主动相位力矩。

`real_output_enabled` 只能由最终命令合成/发送路径更新，策略计算出建议并不等于真实输出已经发生。

## 5. 命令合成器合同

基础控制器不再自行发送 CAN。控制周期必须按以下顺序完成：

1. 生成一对基础五元命令；
2. 验证本周期 `exo_assist_result_t` 的 `activation_epoch`、传感序号、步态序号和有效标志；
3. SHADOW 直接保留基础命令；ACTIVE 只叠加左右 `torque_ff_nm`；
4. 对最终左右命令做全量有限值、产品限幅和变化率检查；
5. 仅调用一次 `exo_joint_command_try_send_pair()`。

在完全相同的回放输入下，SHADOW 和 ACTIVE 0% 的最终命令必须与阶段 9 基线一致。不得为了加入助力而在一个周期发送第二对 CAN 命令。

## 6. 遥测结构

### 6.1 生产与传输分离

100 Hz 控制任务只在最终命令结果已知后构造固定大小记录并写入静态环形缓冲。默认每 5 个控制周期取样一次，即 20 Hz；产品协议允许选择 5/10/20 Hz。

BLE 发送由低优先级任务完成。100 Hz生产者永远不等待发送空间；环形缓冲满时覆盖最旧样本并增加计数。BLE消费者从环形缓冲取出样本后，会保留当前通道并等待瞬时mbuf背压恢复，最长1秒；这段等待只阻塞BLE低优先级任务，不阻塞控制任务。当前NimBLE MSYS1使用24个静态块，遥测任务栈4096 bytes，默认4通道帧以10 ms间隔摊开发送。设备不持续写 NVS/Flash，长期记录由上位机完成。

### 6.2 上报内容

连续遥测的第一版固定7个可选通道：

- 左右人体髋角度、角速度和 RS01 映射后的电机估算力矩；
- 左右独立相位和总体步态置信度；
- 左右建议助力，以及确实加入最终命令并成功发送的助力增量；
- 每包携带流 ID、样本序号、单调时间、START activation epoch低16位、整机状态、
  输出门和真实助力标志；
- GET 遥测状态另提供成功 Notify 帧数、设备端丢弃数和当前队列深度。

profile ID、左右 level、步频/质量位、基础控制配置和状态机详细步骤继续通过现有
`GET_ASSIST_CONFIG`、`GET_GAIT_STATUS`、`GET_CONTROL_CONFIG` 和
`GET_SYSTEM_STATUS` 按需读取；100 Hz 控制时序/错误/栈指标继续由 UART 工程诊断
快照读取。这样不会为了把所有低频元数据塞进连续流而增加每个采样点的 Notify 数。

Product BLE v1 单帧仍不超过20 bytes，因此一个样本拆为若干带相同
`stream_sequence + sample_sequence` 的单通道包。命令 ACK 继续走 Response
Notify；连续遥测走独立 Telemetry Notify 特征。

## 7. BLE 生命周期

1. 客户端连接 Product Service；
2. 订阅 Response Notify 和可选 Telemetry Notify；
3. 读取 INFO、系统状态和遥测状态；
4. 发送遥测配置/START；
5. 设备按配置自动 Notify；
6. 客户端 STOP 或断开时停止遥测。

BLE 断开只终止该连接的遥测会话，不自动 STOP 已经运行的外骨骼。重新连接后必须建立新的 `stream_id`，不能把旧会话尾包解释为新样本。

## 8. ESP32 资源约束

- 唯一控制周期：100 Hz；
- 阶段 10 目标控制路径最大执行时间：`< 5 ms`；
- 控制任务最小剩余栈：`>= 1024 bytes`；
- 算法状态和遥测队列均为固定大小；
- 正常 20 Hz 遥测不丢样本；拥塞时只增加遥测丢弃计数，控制周期统计不得恶化；
- 所有新增模块必须使用明确的中文注释解释单位、状态和并发所有权。

## 9. `0.10.1` 隔离工程验证激励

`0.10.0` 的产品助力因果方向保持不变：人体先运动，编码器和步态估计器得到有效
相位后，`ACTIVE_PHASE` 才能生成助力。该链的软件实现已经存在，但真人
`ACTIVE 0%/5%/10%` HIL仍未完成，不能写成整机验收通过。

`0.10.1` 另行加入 `IMPEDANCE + ALTERNATING_SINE_TEST`，用于空载或受控台架上让
两侧按左右反相的固定工程轨迹运动，进而验证反馈、步态估计、SHADOW建议和遥测。
固定参数为单侧10 deg、0.5 Hz、3 s五次渐入；它不是产品助力模式、不是人体意图
估计器，也不是文献复现曲线。

该参考源由 `CONFIG_EXO_ENGINEERING_GAIT_STIMULUS` 编译开关隔离，默认关闭；只允许
`DISABLED/SHADOW`，与 `ACTIVE` 组合时必须在START前拒绝。产品默认初始化仍为
`DAMPING + CONSTANT`。完整边界见 `PHASE_10_ENGINEERING_STIMULUS.md`。
