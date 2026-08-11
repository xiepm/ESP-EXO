# 阶段 8：统一五元关节命令与基础模式验收

## 1. 当前结论

当前结论：**PASS / COMPLETE**。

软件实现、主机/构建验收、六种基础模式、产品 BLE 和真机运动均已通过。第一次
`IMPEDANCE + SINE` 因左侧原始坐标接近 RS01 周期边界且旧驱动未展开周期反馈而
失败；修复连续位置/epoch 后，操作者授权在中立位对 A/B 写入机械零位，再捕获
双侧 RAM 零位。随后一轮短时门控及三轮完整正弦均通过，未再出现约 `1440 deg`
假跳变。最终系统为 IDLE，A/B 为 REST，控制与 TWAI 错误计数均为 0。

## 2. 软件验收合同

| 项目 | 门槛 |
|---|---|
| 五元单位 | 关节层 `deg/deg/s/N·m`；驱动层 `rad/rad/s/N·m` |
| 左右方向 | q/dq/tau 随 direction；KP/KD 不翻转 |
| 双侧发送 | 预检失败零发送；成功后 A→B；A 失败仍尝试 B |
| 跨接口范围 | UART 可存入的五元配置均能由 BLE 定点格式合法编码并按协议分辨率读回 |
| 模式 | HOLD/POSITION/VELOCITY/TORQUE/DAMPING/IMPEDANCE 六种确定输出 |
| 阶段 7 回归 | 0～60 s、10 ms、6001 点，最大误差 `<=1e-4 rad` |
| 实时结构 | 全工程恰好一个 `xTaskDelayUntil()`；无新周期源/动态内存 |
| BLE | 旧 v1 行为兼容；新帧 `<=20 B`；只允许 IDLE 静态配置 |
| 构建 | ESP-IDF v5.5 fullclean/build/size，0 error/warning |

## 3. 已执行的软件测试

| 测试 | 结果 |
|---|---|
| 六模式实际 C 向量、selected/active 生命周期 | PASS |
| 阶段 7 实际 C 回放 6001 点 | PASS，最大误差 `2.75304283e-05 rad` |
| 双侧预检、固定 A→B、A 失败仍发 B | PASS |
| 实际 `hip_joint.c` LEFT/RIGHT q/dq/tau/KP/KD 映射 | PASS |
| C BLE v1旧请求与阶段8配置/五元编解码 | PASS |
| 阶段6 Python BLE离线回归 | PASS，10/10 |
| 网页构建及协议/连接/日志/兼容测试 | PASS，31/31 |
| 网页 ESLint | PASS |

主机测试使用临时 stub 和可执行文件直接编译仓库中的实际 C 源；结果记录后删除临时
文件，不把一次性 `phase8_*.py`、二进制或缓存加入仓库。

## 4. 最终固件构建

2026-08-10 使用 ESP-IDF v5.5 对最终源代码执行
`idf.py fullclean -> idf.py build -> idf.py size`：

| 项目 | 结果 |
|---|---|
| 芯片目标 | ESP32-S3 |
| ESP-IDF 应用版本 | `0.8.0`，与 BLE GET_INFO 一致 |
| 编译/链接 | PASS，0 error，0 compiler warning |
| 应用镜像 | `0x9bd60` bytes |
| 1 MiB 应用分区剩余 | `0x642a0` bytes，39% |
| `esp-idf-size` 总镜像 | 638189 bytes |

## 5. 静态结构检查

- 周期调用链只有
  `exo_control_loop -> exo_system -> exo_controller -> exo_joint_command -> RS01`；
- `exo_controller` 和 `exo_joint_command` 不创建任务、队列、定时器或堆对象；
- BLE worker 只处理低频配置，不直接发周期 CAN；
- 网页只在 IMPEDANCE 下开放 SINE；离开该模式会恢复 CONSTANT；
- `app_main.c` 只初始化对象；
- 整机枚举仍为六态；模式枚举不属于整机状态机；
- 阶段 9 的传感统一快照/步态估计和阶段 10 的相位曲线/连续遥测均未提前加入。
- 独立实时/协议审查未发现 P0/P1；发现的 SINE 网页组合约束和 UART/BLE 位置
  范围不一致均已修复并重新构建。

## 6. 真机验收计划

前置条件：

1. 烧录本次 `0.8.0` 固件并确认启动为 IDLE；
2. 确认 LEFT/RIGHT 反馈在线、无 fault，记录 control/system/TWAI 基线；
3. 操作者明确确认双髋处于中立位；
4. 本轮重新捕获 LEFT/RIGHT RAM 软件零位并读回。

每种模式独立执行 `STOP -> IDLE -> SET -> GET -> START -> RUNNING -> STOP -> IDLE`：

1. HOLD；
2. DAMPING；
3. 小幅 POSITION；
4. 小量 VELOCITY；
5. 小量 TORQUE；
6. IMPEDANCE + SINE，至少三轮，每轮不少于 10 s。

沿用阶段 7 门槛：平均周期 `10.0 ms +/-0.1 ms`，至少 99% 周期位于 `9～11 ms`，
最大周期误差 `<=2 ms`，最大执行时间 `<10 ms`，栈余量 `>=25%`，deadline、
skipped、output error 不增长；TWAI 各错误计数以本轮开始值为基线且不得增长。
最终 STOP 后两台电机必须为 REST 且未请求使能。

## 7. 2026-08-10 真机执行记录（进行中）

### 7.1 烧录、启动和归零

| 项目 | 结果 |
|---|---|
| 串口 | `COM5`，USB-Enhanced-SERIAL CH343 |
| 烧录 | PASS；ESP32-S3，bootloader/partition/app SHA 均验证通过 |
| 应用版本/镜像 | `0.8.0` / `0x9b680` bytes |
| 启动状态 | IDLE；executor/control/BLE worker 均存活 |
| A/B | 在线、REST、未请求使能、主动反馈约 100 Hz、无 fault |
| TWAI 初始值 | RUNNING；TX/RX error、TX failed、bus error、arbitration lost、RX missed 均为 0 |
| LEFT 零位 | 执行器 B/+1，`66.367 deg`，RAM valid |
| RIGHT 零位 | 执行器 A/-1，`-26.166 deg`，RAM valid |
| 归零后系统 | `Ready to start=YES`，`Start block=NONE` |

### 7.2 前五种模式

每轮都独立完成 `SET -> readback -> START -> RUNNING约10 s -> STOP -> IDLE`：

| 模式 | 运行中五元输出证据 | 本轮结果 |
|---|---|---|
| HOLD | 两侧保持 START 中心，`kp=0.2,kd=0.1` | PASS |
| DAMPING | 两侧 `dq=0,tau=0,kp=0,kd=0.1` | PASS |
| POSITION | 两侧 `q=5.000 deg,dq=0,tau=0,kp=0.2,kd=0.1` | PASS |
| VELOCITY | 两侧 `dq=2.000 deg/s,tau=0,kp=0,kd=0.1` | PASS |
| TORQUE | 两侧人体命令 `tau=0.200 N·m,kp=kd=0` | 命令/状态机 PASS；低负载持续转动见下文 |

五轮之后累计实时证据：平均周期 `10000 us`，最小/最大 `9500/10500 us`，
最大周期误差 `500 us`，`outside +/-1000 us=0`，最大执行时间 `1359 us`，控制
任务最小剩余栈 `2444 B`；deadline、skipped、output error 均为 0。五轮期间及
最终 STOP 后 TWAI 所有错误计数保持 0。每轮 STOP 后控制器均 disarm，A/B 均
REST、未请求使能且无 fault；START/STOP 当前均为 `5/5 success`、0 failure。

### 7.3 TORQUE 模式观察和暂停点

`TORQUE` 是 `kp=kd=0` 的持续开环前馈力矩，不负责位置或速度约束。在当前低负载
装置上运行约 10 s 后：

- 运行中 LEFT 曾到 `q=226.277 deg`、`dq=805.486 deg/s`；
- STOP 后 LEFT 原始执行器位置为 `571.452 deg`，按旧零位计算为 `505.085 deg`；
- STOP 后 RIGHT 按旧零位计算为 `9.583 deg`；
- STOP 成功，系统 IDLE、A/B REST、TWAI/控制计数正常，无固件故障。

这不是五元映射错误，而是开环力矩在低阻负载下的预期积分运动；它也说明“用固定
小力矩运行 10 s”不是合适的产品级力矩模式台架门槛。由于原中立姿态已经失效，
本轮在此停止，未直接启动 `IMPEDANCE + SINE`。待操作者重新把双髋置于中立位后，
应重新捕获 LEFT/RIGHT RAM 零位，再执行三轮正弦阻抗。

### 7.4 产品 BLE 阶段 8 扩展

第二次中立位确认后，未重启固件；覆盖 RAM 零位为 LEFT/B `571.452 deg`、
RIGHT/A `-35.749 deg`。随后先执行真机 BLE 验证：

| 项目 | 结果 |
|---|---|
| 阶段 6 只读默认套件 | PASS，15/15；固件 0.8.0、boot ID `0x758B2156` |
| GET_INFO | capability `0x000F`、max frame `20 B` |
| SET/GET_CONTROL_CONFIG | PASS；`IMPEDANCE + SINE` 逐字节读回一致 |
| LEFT SET/GET_JOINT_COMMAND | PASS；`[0 deg,0 deg/s,0 N·m,0.2,0.1]` |
| RIGHT SET/GET_JOINT_COMMAND | PASS；`[0 deg,0 deg/s,0 N·m,0.2,0.1]` |

### 7.5 IMPEDANCE + SINE 第一次尝试

第 1 轮经 BLE 完成 START、RUNNING `12.203 s`、STOP、IDLE；左右各取得 31 个
实际反馈样本，最大 feedback age `10 ms`。排除前 `3.5 s` 后搜索共享频率：

| 指标 | LEFT | RIGHT |
|---|---:|---:|
| 共享频率 | `0.503 Hz` | `0.503 Hz` |
| 拟合振幅 | `103.40 deg` | `23.87 deg` |
| R² | `0.113` | `0.936` |
| 实际跨度 | `1106.04 deg` | `46.29 deg` |
| 双侧拟合相位差 | `-65.49 deg` | - |

RIGHT 复现了约 0.5 Hz 局部正弦；LEFT 出现超过 1100 deg 的跨度，未形成局部正弦，
因此波形和相位门槛 FAIL。运行器先完成 STOP/IDLE，再阻止第 2、3 轮。STOP 后：

- 系统 IDLE、控制器 disarm，A/B 在线、REST、未请求使能、无 fault；
- LEFT/B 原始位置 `558.198 deg`，相对本次零位 `-13.254 deg`；
- RIGHT/A 原始位置 `-20.012 deg`，相对本次零位 `-15.737 deg`；
- 100 Hz 均值 `10000 us`、最大周期误差 `500 us`、最大执行 `1934 us`、栈余量
  `2444 B`，deadline/skipped/output error 均为 0；
- TWAI 全部错误计数仍为 0，BLE notify failure 为 0。

最可能的待验证假设是：LEFT 软件零位 `571.452 deg` 已接近 RS01 正位置边界，
虽然命令包络 `571.452 +/- 90 deg` 仍在 `+720 deg` 内，但实际跟踪的惯性超调可能
越过反馈边界并发生多圈/回绕；这与最初阶段 7 的 LEFT 零位 `66.367 deg` 条件不
等价。当前证据还不足以把它写成已确认根因。继续试验前，应先把执行器 B 的原始
多圈位置恢复到远离边界的区间（优先接近原始 `66 deg`），再以机械中立位重新
捕获 RAM 零位；不得仅在 `558~571 deg` 附近反复覆盖软件零位后继续正弦。

### 7.6 RS01 周期反馈修复及重新烧录

查阅 RS01 手册 4.2.2 后确认：Type 2 位置反馈以 `-4pi~+4pi` 为一个周期，主机应
根据相邻反馈累计周期数。旧驱动把这个周期坐标直接当成连续位置，因此跨过边界时
会在人髋关节层形成约 `8pi`（约 `1440 deg`）的假跳变。这个协议语义与第 1 轮
LEFT 的异常跨度一致，但第 1 轮没有保存逐个 10 ms 原始帧，所以不能倒推它是否是
唯一因素。

本轮在不改变线上原始坐标语义的前提下完成以下修复：

- `sample.position_rad` 继续保存可直接回发给电机的周期坐标；
- 驱动按 A/B 独立累计周期，另行发布连续位置、周期计数和坐标 epoch；
- 反馈长时间中断、精确半周期歧义、计数溢出或电机机械置零都会创建新 epoch；
- 髋关节软件零位绑定连续位置及其 epoch，旧 epoch 的零位自动失效；
- 下行位置仍执行严格范围检查，不对 Motion 目标做未经手册证明的自动取模；
- 机械置零后的连续坐标需连续 3 帧接近 `0 rad` 才重新有效，避免捕获排队中的旧帧。

直接编译仓库实际 `rs01_driver.c` 和 `hip_joint.c` 的主机测试覆盖正/反向跨界、
A/B 独立性、超时重建、半周期歧义、计数溢出、机械置零旧帧、三帧确认、软件零位
epoch 失效及严格下行范围拒绝，结果均为 PASS。最终执行
`fullclean -> build -> size`，0 error、0 compiler warning；应用镜像为
`0x9bd60` bytes，应用分区剩余 39%。

2026-08-10 已把修复版重新烧录至 COM5，并只读确认：

| 项目 | 重新烧录后的基线 |
|---|---|
| 系统/输出 | `IDLE`，输出未许可，控制器未 armed |
| A（RIGHT） | raw/continuous `-20.012 deg`，periods=0，epoch=1，REST，无 fault |
| B（LEFT） | raw/continuous `558.220 deg`，periods=0，epoch=1，REST，无 fault |
| RAM 零位 | 重启后两侧均 INVALID，START 被 `LEFT_ZERO_INVALID` 阻止 |
| 主动反馈 | A/B 约 100 Hz，连续位置有效 |
| 100 Hz 任务 | 均值 `10000 us`，最大误差 `499 us`，deadline/skipped/output error 均为 0 |
| TWAI | RUNNING；所有错误计数均为 0 |

当前不能把“在中立位重新捕获 RAM 零位”误当成“把电机周期坐标移到安全区”。若
在 B=`558.220 deg` 处只做 RAM 归零，`+90 deg` 目标为约 `648.220 deg`，到
`+720.21 deg` 边界只剩约 `72 deg`。继续三轮之前，需要操作者明确选择：在当前
中立位写入两台 RS01 的持久机械零位，并确认反馈连续 3 帧接近 0；或者用其他方式
先把 B 的原始周期坐标恢复到远离边界的位置。随后还要重新捕获双侧 RAM 零位，并
先通过一轮短时低风险正弦门，再执行三轮完整正弦。

### 7.7 A/B 机械置零、RAM 归零和短时门控

操作者明确授权原文：

> 授权对 A、B 执行 RS01 机械置零

执行前再次确认系统为 IDLE、输出未许可、A/B 均 REST 且无 fault，TWAI 六项错误
为 0。此时中立位原始坐标为 A `-155.295 deg`、B `598.025 deg`。按 A 后 B 的
顺序执行机械置零，每台都先失能，再发送 RS01 Type 6 置零命令，并等待驱动的三帧
确认：

| 项目 | A / RIGHT | B / LEFT |
|---|---:|---:|
| 置零后 raw/continuous | `-0.011 deg` | `-0.011 deg` |
| position epoch | `1 -> 2` | `1 -> 2` |
| rebase count | `1` | `1` |
| period count | `0` | `0` |
| 最终电机状态 | REST、未请求使能、无 fault | REST、未请求使能、无 fault |

随后捕获双侧 RAM 软件零位：LEFT/B 为 `-0.011 deg`，来源 feedback count
`120430`、epoch 2；RIGHT/A 为 `-0.011 deg`，来源 feedback count `120482`、
epoch 2。系统由 `LEFT_ZERO_INVALID` 变为 `Ready to start=YES / block=NONE`。

正式三轮前先执行一轮 `8.2 s` 短门控；任何反馈无效、系统离开 RUNNING、关节绝对
角超过 `180 deg` 或相邻样本跳变超过 `120 deg` 都会立即 STOP。本轮结果：

| 指标 | LEFT | RIGHT |
|---|---:|---:|
| 样本数 | 41 | 41 |
| 实际跨度 | `52.729 deg` | `50.640 deg` |
| 拟合振幅 | `28.434 deg` | `26.432 deg` |
| R² | `0.9914` | `0.9881` |
| 共享频率 | `0.500 Hz` | `0.500 Hz` |
| 关节相位差 | `2.043 deg` | - |

短门控全程两侧 period count 为 0、epoch 为 2；STOP 后系统回到 IDLE，A/B REST，
控制错误和 TWAI 错误均为 0，因此放行三轮完整测试。

### 7.8 三轮 IMPEDANCE + SINE 和最终收敛

每轮从 `IDLE + ready` 独立 START，确认 RUNNING 后运行不少于 10 秒，再 STOP 并
确认 IDLE。以反馈时间轴排除前 `3.5 s` 渐入，在 `0.30~0.70 Hz` 搜索双侧共享
频率并分别拟合正弦：

| 轮次 | RUNNING | 样本 L/R | 跨度 L/R | 共享频率 | 振幅 L/R | R² L/R | 关节相位差 |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 1 | `12.219 s` | 61 / 61 | `54.531 / 48.729 deg` | `0.499 Hz` | `28.507 / 25.234 deg` | `0.9902 / 0.9871` | `-0.841 deg` |
| 2 | `12.219 s` | 61 / 61 | `55.234 / 50.487 deg` | `0.500 Hz` | `27.934 / 24.762 deg` | `0.9915 / 0.9872` | `1.591 deg` |
| 3 | `12.218 s` | 61 / 61 | `54.289 / 50.311 deg` | `0.500 Hz` | `28.079 / 25.086 deg` | `0.9905 / 0.9804` | `0.995 deg` |

三轮所有采样均保持 period count=0、epoch=2；没有反馈失效、状态异常、坐标回绕或
提前 STOP。包括短门控在内，状态机最终 START/STOP 均为 `4/4 success`、0 failure，
transition count 为 17，符合每轮四次迁移。

最终只读快照：

| 项目 | 最终结果 |
|---|---|
| 系统 | IDLE、ready YES、run/output false、executor identity 不变 |
| RAM 零位 | 左右 feedback count、timestamp、epoch 与捕获时完全相同 |
| 100 Hz | mean `10000 us`，min/max `9490/10499 us`，范围外周期 0 |
| 实时上限 | 最大周期误差 `510 us`；最大执行 `1573 us`；栈余量 `2348 B` |
| 控制错误 | deadline/skipped/output error=`0/0/0` |
| A / RIGHT | raw/continuous `0.385 deg`，period 0、epoch 2，REST，无 fault/warning |
| B / LEFT | raw/continuous `8.297 deg`，period 0、epoch 2，REST，无 fault/warning |
| TWAI | RUNNING；TX/RX/TX failed/bus/arbitration/RX missed 均为 0 |
| BLE 最终回归 | 只读默认套件 PASS，15/15；固件 `0.8.0`、boot ID `0x0E785C98` |

最终 BLE 套件只执行 GET、重复请求重放、request-id 冲突和畸形帧拒绝，不执行归零、
START 或 STOP。GET_SYSTEM_STATUS 返回 IDLE；左右 GET_JOINT_STATUS flags 均为
`0x3F`。至此阶段 8 的软件、构建、六模式、BLE 和三轮真机门槛全部闭环。

## 8. 最终完成记录

| 项目 | 结果 |
|---|---|
| 五元命令、左右映射和六种基础模式 | PASS |
| 阶段 7 数值回放、主机边界测试 | PASS |
| RS01 周期反馈展开和 epoch 失效机制 | PASS |
| ESP-IDF v5.5 fullclean/build/size | PASS |
| HOLD/DAMPING/POSITION/VELOCITY/TORQUE 真机链 | PASS |
| IMPEDANCE + SINE 短门控及三轮完整运动 | PASS |
| 100 Hz 实时指标、STOP/REST 和 TWAI | PASS |
| 产品 BLE 配置链及最终只读回归 | PASS（15/15） |
| 阶段结论 | **PASS / COMPLETE** |
