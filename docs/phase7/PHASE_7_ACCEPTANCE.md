# 阶段 7：正弦参考与阻抗控制迁移验收

## 1. 当前结论

阶段 7 于 2026-08-07 完成，最终结论为 **PASS**。

代码迁移、实际 C 数值回放、静态审计、ESP-IDF v5.5 干净构建、烧录、未归零门控、BLE v1 回归、BLE 软件归零和三轮实机运动全部通过。三轮实际 RUNNING 时间分别为 `12.063 s`、`12.360 s` 和 `18.829 s`；共享频率为 `0.5022 Hz`、`0.5000 Hz` 和 `0.4952 Hz`，LEFT/RIGHT 实际关节反馈保持同相。阶段 7 正式标签为 `phase7-sine-impedance-migration`。

## 2. 运行与验收合同

| 项目 | 合同 |
|---|---|
| 基线 | `813117d / phase6-product-ble-protocol` |
| 迁移类型 | 行为等价的软件架构迁移和实机回归 |
| 数值比较 | 旧正弦偏移、新正弦偏移及新旧 A/B 电机目标 |
| 数值区间 | `0~60 s`，步长 `10 ms`，共 6001 点 |
| 最大允许误差 | `1e-4 rad` |
| 长时间检查 | 24 小时等效时间采样，全部 finite 且幅值有界 |
| 波形门槛 | 每轮双侧跨度 `>=0.1 rad`；共享频率 `0.45~0.55 Hz`；拟合振幅 `>=0.1 rad`；R² `>=0.65`；关节相位差 `<=40°` |
| 实时门槛 | 平均周期 `10.0 ms +/-0.1 ms`；至少 99% 位于 `9~11 ms`；最大周期误差 `<=2 ms`；最大执行 `<10 ms`；栈余量 `>=25%`；deadline/skipped/output error 均为 0 |
| 总线门槛 | TWAI TX/RX error、TX failed、bus error、arbitration lost、RX missed 不增长 |
| 停止条件 | 数值不等价、出现第二周期源、构建失败、未确认中立位或任一实机门槛失败 |

阶段 7 保留原参数：幅值 `pi/2 rad`、频率 `0.5 Hz`、五次渐入 `3 s`、`kp=0.2`、`kd=0.10`。LEFT 与 RIGHT 使用同相关节偏移；当前映射为 `LEFT -> B / +1`、`RIGHT -> A / -1`，所以映射后的电机目标继续呈现旧实现的 A 负偏移、B 正偏移。发送顺序仍为 A 后 B，A 失败时仍尝试 B。

## 3. 代码变化检查表

- [x] 正弦和五次渐入拆到纯数学模块 `sine_reference`；
- [x] 固定参数迁入只读控制配置，不混入硬件配置；
- [x] arm 中心由 A/B 改为具名 LEFT/RIGHT 结构；
- [x] 增加“关节位置 -> 配置执行器位置”的窄转换接口；
- [x] START 在 fresh feedback 后以最新 LEFT/RIGHT 关节快照 arm；
- [x] LEFT/RIGHT 参考保持同相，映射后保持旧 A-/B+ 行为；
- [x] A/B 发送顺序和“两侧均尝试”语义保留；
- [x] tick 所有计算/发送结果共用 `s_tick_active` 清理出口；
- [x] `kp/kd` 同时验证 RS01 实际上下限；
- [x] BLE 协议 v1、六态状态机、STOP 输出门和唯一 100 Hz 任务未改变；
- [x] 未提前增加五元命令、基础模式、步态估计或遥测。

说明：arm 使用 fresh-feedback 等待完成后的最新关节快照，和阶段 6 的原始采样可能相差最多一个反馈等待窗口。运动验收从中立位重新归零后开始，因此该差异不改变本阶段目标行为。

## 4. 数值回放

实际使用 WSL Ubuntu gcc 11.4.0 编译并直接链接当前 `main/sine_reference.c`：

```text
gcc -std=c11 -O2 -Wall -Wextra -Werror \
    -Imain/include main/sine_reference.c <temporary_test.c> -lm
```

临时测试源和可执行文件已在记录结果后删除，不进入仓库。

| 指标 | 门槛 | 结果 | 状态 |
|---|---:|---:|---|
| `0~60 s` 样本数 | 6001 | 6001 | PASS |
| 新旧参考最大误差 | `<=1e-4 rad` | `2.75187194e-05 rad` | PASS |
| 电机 A 目标最大误差 | `<=1e-4 rad` | `2.76565552e-05 rad` | PASS |
| 电机 B 目标最大误差 | `<=1e-4 rad` | `2.76565552e-05 rad` | PASS |
| `0~3 s` ramp | `[0,1]` 且单调 | 通过 | PASS |
| 非正 elapsed | offset/ramp 均为 0 | 通过 | PASS |
| `3.5 s` 检查点 | `-pi/2 rad` | 误差 `<1e-6 rad` | PASS |
| 24 h 等效采样 | finite、有界 | 最大绝对值 `1.57079637 rad` | PASS |
| 空指针/无效配置 | 拒绝 | 通过 | PASS |

RS01 位置范围为 `[-12.57, 12.57] rad`。运动前还必须用本次软件零位确认 `software_zero_motor_rad +/- pi/2` 不进入位置编码限幅。

## 5. 静态结构验收

| 检查 | 门槛 | 结果 |
|---|---|---|
| `xTaskDelayUntil()` | 全工程恰好 1 个 | PASS，仍位于 `exo_control_loop.c` |
| 正弦模块任务/定时器 | 0 | PASS |
| 正弦模块 RTOS/RS01/TWAI/BLE/日志/堆依赖 | 0 | PASS |
| `impedance_control` 硬编码 A/B 中心或侧别 | 0 | PASS |
| `exo_ble_protocol.c/.h` 变化 | 0 | PASS |
| 新整机状态 | 0 | PASS |
| 新公共五元命令/模式 | 0 | PASS |
| `git diff --check` | 退出码 0 | PASS |

独立代码审查未发现 P0/P1 阻断项。双精度相位折返加入热路径后的实际开销已由第 11 节实测确认：最终最大执行时间为 `1370 us`，低于 `10 ms` 门槛。

## 6. 构建与固件产物

环境：Windows PowerShell、ESP-IDF v5.5、目标 `esp32s3`。

| 项目 | 结果 |
|---|---|
| `idf.py fullclean -> build -> size` | PASS |
| warnings/errors | 0 / 0 |
| 应用大小 | `0x99830` = 628,784 bytes |
| 最小应用分区余量 | `0x667d0` bytes，40% |
| size 工具 image size | 628,673 bytes |

| 产物 | 大小 | SHA-256 |
|---|---:|---|
| `twai_network_master.bin` | 628,784 B | `1E9466CB450DC292AB3D47FF18347FD9A59519B4603B3347F8606EAB7B3E3CD3` |
| `twai_network_master.elf` | 7,522,532 B | `7FA5434834BBD03F0BCD6C26CAF31225BE7213DB80EA410476F34B31A3BA850E` |
| `bootloader.bin` | 21,088 B | `204AE2E2E8B475C0A1462FED51317B2C4CFC05E9C53E997C978A6133EB7C72F0` |
| `partition-table.bin` | 3,072 B | `7F00B6C042A89B15B0CAC534F82ED988CAF29278FF5700B0C511EB1B5BB7C820` |

## 7. 烧录与无运动验收

状态：**PASS**。

烧录端口 `COM5`，USB `VID_1A86&PID_55D3`；ESP32-S3 MAC `44:1b:f6:fe:22:c0`，BLE MAC `44:1b:f6:fe:22:c2`。最终烧录成功，烧录后固件哈希与上表一致。

| 检查 | 结果 |
|---|---|
| 启动状态 | `IDLE`，`run_requested=false`，`output_allowed=false` |
| 软件零位 | LEFT/RIGHT 均 `zero_valid=false` |
| 就绪门 | `ready_to_start=false`，block=`LEFT_ZERO_INVALID` |
| 电机 | A/B feedback valid/online，REST，未使能，fault/warning=0，约 32.6 °C |
| UART START | `REJECTED_NOT_READY`，状态保持 IDLE |
| BLE START | `result=0x10, state=1, detail=10`，即零位无效而拒绝；4/4 PASS |
| 控制任务 | identity `0x3fc9f844`；100 Hz；mean `10000 us`；最大周期误差 `499 us` |
| 控制错误 | deadline/skipped/output error 均为 0 |
| TWAI | TX/RX/TX failed/bus/arbitration/RX missed 均为 0 |

阶段 6 BLE v1 只读回归为 **15 passed / 0 failed**，GET_INFO 返回协议 v1、固件 `0.7.0`。同一启动周期的 BLE 连接/断开前后：

- system 始终为 IDLE，transition count=1，executor identity 始终为 `0x3fca0a6c`；
- control identity 始终为 `0x3fc9f844`，周期由 306 增至 1745，最大执行时间 `331 us`，无超期、跳周期或输出错误；
- BLE worker identity 始终为 `0x3fca1db8`，session generation 从 0 增至 2，没有新建控制任务；
- BLE 统计 RX/queued/processed=`14/14/14`，queue full/stale=`0/0`，notify failure=0；
- TWAI 所有错误计数保持为 0。

## 8. 中立位确认、BLE 归零和运动前门禁

状态：**PASS**。

本次用户确认原文：

> 双髋已处于中立位，可以归零并开始阶段 7 运动验收。

串口采集器在归零前打开并一直持有 COM5。打开前后的 BLE boot ID 均为 `0xD48CE6D4`，因此没有发生复位，也不需要复用旧阶段的软件零位。随后通过产品 BLE 分别执行 LEFT、RIGHT `CAPTURE_ZERO`，两次首次响应均为 `0x00 OK`，相同帧重复响应逐字节一致。

| 项目 | LEFT | RIGHT |
|---|---:|---:|
| 物理映射 | B / `+1` | A / `-1` |
| RAM 软件零位 | `5.387 rad` | `-6.072 rad` |
| 零位来源 feedback count | 178457 | 178456 |
| 零位来源 timestamp | 1785682241 us | 1785844004 us |
| 归零后关节角 | `0.000 rad` | `0.000 rad` |
| `joint_valid` | YES | YES |
| 基础 `zero +/- pi/2` 端点 | `[3.816, 6.958] rad` | `[-7.643, -4.501] rad` |

归零后系统为 `IDLE`、ready YES、block NONE、run/output 均为 false。每轮 START 前还用当时的实际关节中心重新计算：

```text
motor_center = software_zero_motor + direction * current_joint_center
端点 = motor_center +/- pi/2
```

三轮的最窄端点范围仍为 LEFT `[3.596, 6.958] rad`、RIGHT `[-7.643, -4.269] rad`，并留有 10 mrad 文本量化裕量，未接近 RS01 `[-12.57, 12.57] rad` 限幅。

运动前基线：

- system transition count=1，executor identity=`0x3fca0a6c`；
- control identity=`0x3fc9f844`，mean=`10000 us`，最大周期误差 `500 us`，最大执行 `355 us`；
- deadline/skipped/output error=`0/0/0`，栈余量 `2780 B`；
- TWAI 为 RUNNING，六项错误计数均为 0；
- A/B 均 feedback valid/online、REST、未使能、fault/warning=0，温度 `32.6 °C`。

## 9. 三轮 START/RUNNING/STOP 和波形

状态：**PASS**。

每轮均从 `IDLE + ready` 独立 START，主机确认 RUNNING 后计时，运行不少于 10 秒，再 STOP 并由 BLE 与 UART 共同确认 IDLE、REST 和失能。实际关节反馈使用各自的主机接收时间减 BLE feedback age 形成时间轴；排除前 `3.5 s` 渐入后，在 `0.30~0.70 Hz` 范围搜索共享频率并分别拟合两侧正弦。

| 轮次 | RUNNING | 样本 L/R | 跨度 L/R | 共享频率 / 周期 | 振幅 L/R | R² L/R | 关节相位差 | 映射后电机相位差 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1 | `12.063 s` | 31 / 31 | `0.898 / 0.831 rad` | `0.5022 Hz / 1.9912 s` | `0.317 / 0.426 rad` | `0.7132 / 0.9722` | `2.7°` | `177.3°` |
| 2 | `12.360 s` | 30 / 30 | `0.810 / 0.770 rad` | `0.5000 Hz / 2.0000 s` | `0.386 / 0.441 rad` | `0.9274 / 0.9695` | `11.3°` | `168.7°` |
| 3 | `18.829 s` | 35 / 35 | `0.814 / 0.873 rad` | `0.4952 Hz / 2.0194 s` | `0.390 / 0.405 rad` | `0.9472 / 0.9593` | `2.0°` | `178.0°` |

共保存并复核 192 个实际反馈样本，单样本最大 feedback age 为 `9 ms`。独立读取 CSV 重新拟合得到与运行器 JSON 完全相同的频率、振幅、R² 和相位；每轮每侧总覆盖均超过 11.8 秒。按相邻样本间隔 `<=0.6 s` 重新计算最长连续段，第 3 轮 LEFT/RIGHT 分别为 `2.469 s / 2.468 s`，满足至少一轮双侧连续覆盖 `>=2.2 s` 的基线合同。

“映射后电机相位差”由实际 LEFT/RIGHT 关节反馈和已经实机确认的 `LEFT=B/+1、RIGHT=A/-1` 坐标关系推导；它不是外部 CAN 分析仪对 A/B 控制目标的独立测量。精确目标等价性由第 4 节的 C 数值回放和静态调用链共同证明。

## 10. 幂等、维护隔离与 BLE 断连语义

状态：**PASS**。

- 第 2 轮同一 START 帧重复响应 byte-exact，未重复进入系统请求层；
- RUNNING 中用新 request ID 再次 START，返回 `0x02 ALREADY_SATISFIED`，轨迹未重启；
- RUNNING 中 UART `enable` 被维护门拒绝，LEFT 软件零位和零位来源未改变；
- 第 2 轮 STOP 重复帧同样 byte-exact；
- 三轮结束后 transition count 从 1 精确增加到 13，即每轮 4 次迁移；
- START requests/success 从 `1/0` 变为 `5/3`：三次正常 START 加一次 RUNNING 新请求，缓存重放不计业务请求；
- STOP requests/success 从 `0/0` 变为 `3/3`；START/STOP failure 和 START cancel 始终为 0，系统 rejected count 保持 1。

第 3 轮在 RUNNING 中主动断开 BLE。断连前后：

| 项目 | 断连前 | 重连后 |
|---|---:|---:|
| system | RUNNING / output YES | RUNNING / output YES |
| transition count | 11 | 11 |
| executor identity | `0x3fca0a6c` | `0x3fca0a6c` |
| control identity | `0x3fc9f844` | `0x3fc9f844` |
| control cycle | 183201 | 183809 |
| BLE worker identity | `0x3fca1db8` | `0x3fca1db8` |
| BLE session generation | 11 | 13 |

这证明 BLE 断连不是运行租约，不会隐式 STOP；重连也不创建新的控制、执行器或 BLE worker 任务。第 3 轮重连期间 UART watchdog 持续检查 system、control、TWAI 和两台电机。

## 11. 实时、总线和最终收敛

状态：**PASS**。

| 指标 | 运动前 | 最终 | 增量/判定 |
|---|---:|---:|---|
| control identity | `0x3fc9f844` | `0x3fc9f844` | 不变 |
| cycle / period samples | 178610 / 178609 | 185418 / 185417 | `+6808 / +6808` |
| mean period | `10000 us` | `9999 us` | PASS |
| min / max period | `9500 / 10499 us` | `9410 / 10580 us` | 全部在 9~11 ms |
| period outside `+/-1000 us` | 0 | 0 | 新增比例 0% |
| max absolute period error | `500 us` | `590 us` | `<2 ms` |
| max execution | `355 us` | `1370 us` | `<10 ms` |
| deadline / skipped / output error | `0 / 0 / 0` | `0 / 0 / 0` | PASS |
| stack free min | `2780 B` | `2508 B` | 61.2%，高于 25% |
| TWAI 六项错误 | 全 0 | 全 0 | 不增长 |

最终系统为 IDLE、ready YES、run/output false；executor identity 不变。软件零位及其 feedback count/timestamp 完全未变，最终关节位置为 LEFT `0.153 rad`、RIGHT `0.163 rad`。

| 项目 | Motor A / RIGHT | Motor B / LEFT |
|---|---:|---:|
| feedback | valid / online | valid / online |
| 最终模式 | REST | REST |
| enabled requested | no | no |
| fault / warning | `0 / 0` | `0 / 0` |
| 最终温度 | `33.3 °C` | `33.3 °C` |
| 最终原始位置 | `-6.235 rad` | `5.540 rad` |

最终阶段 6 BLE v1 默认套件再次 **15 passed / 0 failed**：固件 `0.7.0`、boot ID 仍为 `0xD48CE6D4`、system flags=`0x23`、左右 joint flags=`0x3F`。BLE worker identity=`0x3fca1db8`，session generation=15，RX/queue/process=`265/265/265`，queue full/stale、Notify submit failure、Notify TX failure 均为 0。完整 UART 观察中没有 panic、assert、watchdog、brownout、BUS_OFF 或异常重启；唯一以 `ERROR:` 开头的行是预期的 RUNNING 维护命令拒绝。

## 12. 最终完成记录

| 项目 | 最终值 |
|---|---|
| 数值等价 | PASS |
| 静态架构 | PASS |
| 增量构建 | PASS |
| fullclean 构建 | PASS |
| 烧录 | PASS |
| 无运动门控 | PASS |
| BLE 归零及幂等 | PASS |
| 三轮运动与波形 | PASS |
| BLE 断连/重连 | PASS |
| 100 Hz / TWAI / 最终电机状态 | PASS |
| BLE v1 最终回归 | PASS（15/15） |
| 阶段结论 | **PASS / COMPLETE** |
| 正式标签 | `phase7-sine-impedance-migration` |

阶段 7 只证明保留正弦阻抗示例已经等价迁移到人体关节语义；公共五元命令、基础模式、步态估计、左右论文相位曲线和连续产品遥测仍属于阶段 8~10。

## 13. 2026-08-10 度制单位整理回归

正式标签 `phase7-sine-impedance-migration` 的原始验收记录保持上文的弧度制原始
测量值，不回写或改造历史证据。当前固件 `0.7.1` 在该标签之后整理了单位边界：

| 层级 | 当前单位 | 说明 |
|---|---|---|
| 人体关节采样、软件零位 | `deg`、`deg/s` | LEFT/RIGHT 产品对象 |
| 正弦幅值、偏移、启动中心 | `deg` | `90 deg` 等价于原 `pi/2 rad` |
| UART 具名运动命令和状态输出 | `deg`、`deg/s`、`deg/s²` | 裸寄存器读写仍透传手册原始值 |
| RS01 原始反馈和驱动 API | `rad`、`rad/s`、`rad/s²` | 保持电机手册协议单位 |
| BLE v1 线格式 | `mrad`、`mrad/s` | 保持协议兼容，手机换算成度显示 |

单位整理后使用当前 `sine_reference.c` 和公共转换函数重新编译了临时 C 回放程序；
测试程序完成后删除，没有把一次性工具留在仓库。结果如下：

| 检查 | 结果 | 状态 |
|---|---:|---|
| 0～60 s、10 ms 步长 | 6001 点 | PASS |
| 新度制偏移转换回 rad 与 `pi/2 rad` 实现的最大差 | `1.1920929e-07 rad` | PASS |
| LEFT 电机目标最大差 | `4.76837158e-07 rad` | PASS |
| RIGHT 电机目标最大差 | `9.53674316e-07 rad` | PASS |
| 3.5 s 检查点 | `-90 deg` | PASS |
| 24 h 检查点 | finite 且绝对值不超过 `90.0001 deg` | PASS |
| 本次 6001 点目标的 mrad 量化 | 与原目标一致 | PASS |
| BLE v1 离线黄金帧、度显示换算和版本参数 | 10/10 | PASS |
| ESP-IDF v5.5 `fullclean -> build -> size` | `0x999a0`（629,152 B），分区剩余 40% | PASS |

本次固件 SHA-256 为
`579DAAA4BCDB1E630960A3E563E045F1273DA4AA0FDC484A2ED4620080F5701D`。

本次只验证单位换算和既有行为等价，没有重新烧录或驱动电机。阶段 7 的三轮实机
运动、100 Hz 和 TWAI 结论仍由正式标签对应的第 7～12 节证据支持；度制整理的
电机目标误差小于 `1e-6 rad`，没有引入新的状态、任务、周期源或 BLE v1 字段。
