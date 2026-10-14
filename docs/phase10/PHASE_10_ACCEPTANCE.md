# 阶段 10 验收计划与证据记录

本文件同时给出可重复的验收合同和实际证据栏。没有执行过的项目必须标记为 `NOT RUN`，不能用设计结论代替真机结果。

2026-10-14归档更新：0.10.13源码、网页及操作说明整理上传；软件构建/回放的重新验证见
[上传记录](GITHUB_BACKUP_20261014.md)。2026-10-13新增当前运动失败证据：主控ERROR，
最近故障CAN_ERROR_PASSIVE，锁存TEC/REC=0/133，运动输出关闭而双侧新反馈继续。
两轮只读检查没有复位或重新启动；累计故障3仅保留最近一次原因。
见[助力中断诊断](PHASE_10_ASSIST_INTERRUPTION_20261013.md)及其原始串口证据。
这不增加真人SHADOW/ACTIVE通过项，阶段10整体和阶段11仍未完成；以下早先部署状态均为历史记录。

2026-10-13 0.10.13本轮动力断开确认后已烧录，233文件复核/三处哈希/实际BLE0.10.13与策略version4/15与3000参数边界读回PASS，最终关闭/双侧0%/0.500 N·m/600 ms，Boot0x215B41F2、IDLE/运动输出0。新速率20 N·m/s，3秒15申请软件回放到达15；短脉冲与退出有效。脉冲17/17/原集成4/4、O2 ASan/UBSan、故障8/8、网页123/123、TS/ESLint与IDF构建PASS。电机上电反馈、网页按钮E2E、硬件/人体新速率HIL NOT RUN，阶段10整体未完成。见[速率证据](PHASE_10_RISE_20_20261013.md)和[本次部署](PHASE_10_0_10_13_DEPLOYMENT_20261013.md)。

2026-10-13 0.10.12已在本轮动力断开确认后烧录，233文件复核/三处哈希/实际BLE INFO与15上限/3000 ms参数写入读回PASS，超界拒绝；最终恢复关闭/双侧0%/0.500 N·m/600 ms。Boot0x5163F213、IDLE、运动输出0；电机上电后的反馈恢复NOT RUN。FIXED设置上限3000 ms、事件脉冲申请/START上限15.000 N·m，10 N·m/s使3秒实际峰值≤9.5493。真实脉冲16/16及原集成4/4、O2 ASan/UBSan、曲线7/7、故障8/8、成对零位8/8/沿用10/10/方向11/11、网页121/121与IDF构建PASS。包含成对零位与INFO/浮点修复；网页按钮E2E、运动/人体HIL NOT RUN，阶段10整体仍未完成。见[新增范围证据](PHASE_10_EXTENDED_PULSE_20261013.md)及[本次部署](PHASE_10_0_10_12_DEPLOYMENT_20261013.md)。

2026-10-12双髋零位候选0.10.11：产品“重新校准”改为单次BLE成对采集当前位置，整对RAM提交；网页必须核对两侧人体角与原始角/偏置均接近0°才确认成功。真实C新回放8/8、原保存沿用10/10、原方向11/11、网页115/115、TS/ESLint与ESP-IDF 5.5构建PASS。尚未烧录，网页实机双髋校准HIL NOT RUN；阶段10整体仍未完成。见[双髋成对零位证据](PHASE_10_PAIR_ZERO_CAPTURE_20261012.md)。

2026-10-12实时示波器网页：空图原因确认为未开启遥测/通道未选；新增自动采集、主控时间横轴、量程、左右开关、同步游标、暂停及断流提示。网页106测试/构建/TS/ESLint与真实BLE界面操作通过，1389时刻/138.8秒为10.0 Hz、无缺口；暂停时继续收包且图形冻结，手动停止后不自动重启。当前是待机有效角度/速度数据，相位未锁定、助力无有效样本；行走与ACTIVE、全7通道20 Hz及完整断连/重连E2E仍NOT RUN，阶段10整体未完成。见[示波器证据](PHASE_10_REALTIME_SCOPE_20261012.md)。

2026-10-12已部署0.10.10：策略4从持续阈值改为单侧新事件sin²脉冲；两种时长、本侧周期估计、复位门禁、冻结A/Ta、反向/停止/窗口/反馈退出已实现，软件测试与IDF构建通过。当前动力断开确认后224个文件复核及烧录三处哈希校验通过，网页GET_INFO0.10.10/0xFF7F与新脉冲参数查询通过；Boot ID=0x2BD55D97、IDLE/门关闭/START0/运动输出0、RAM零位无效。随后上电无复位/无手动恢复，约33.46秒跨度内左右新反馈约99.95 Hz、REST/故障0、CAN错误0，无运动恢复一轮通过；参数写入E2E及新策略真人SHADOW/ACTIVE NOT RUN。见[软件证据](PHASE_10_SWING_EVENT_PULSE_20261012.md)及[本次部署](PHASE_10_0_10_10_DEPLOYMENT_20261012.md)。阶段10整体仍未完成。

2026-10-10新增并烧录0.10.9：可选沿用穿戴零位，NVS候选不自动启用；关节/候选服务/协议主机10/10、原方向11/11、角度10/10、原相位集成4/4、网页86/86及IDF 5.5构建通过。新界面离线显示通过；用户确认本轮动力断开后，197个冻结文件复核、烧录及三处哈希校验通过，启动IDLE/START请求0/输出0、方向NVS保留、RAM零位无效。断电后原始反馈0；网页INFO待重连核对，网页保存/复位/沿用HIL NOT RUN。见[可选沿用零位证据](PHASE_10_SAVED_WEAR_ZERO_20261010.md)和[0.10.9部署](PHASE_10_0_10_9_DEPLOYMENT_20261010.md)。阶段10整体未完成。

2026-10-10新增并部署0.10.8：ANGLE_THRESHOLD峰值5.000 N·m、最大上升速率10 N·m/s，真实C回放10/10、原相位集成4/4、网页81/81、IDF构建通过。用户确认当前断电后已从冻结产物烧录，三处哈希校验及IDLE/输出0检查通过；网页INFO确认0.10.8/0x3F7F，实际按钮写入并读回5.000 N·m/10.00°/双侧0%/DISABLED，UART确认，START和运动输出仍0。0.10.8上电反馈恢复、运动中SHADOW/ACTIVE HIL NOT RUN。见[0.10.8部署](PHASE_10_0_10_8_DEPLOYMENT_20261010.md)；阶段10整体仍未完成。见[5 N·m软件证据](PHASE_10_5NM_ASSIST_20261010.md)。

2026-10-10最新0.10.7部署：操作者明确授权且动力断电后，冻结版本烧录和三处哈希校验通过，网页读回0.10.7；新策略通过网页按钮写入与读回非默认12.50°阈值和0.100 N·m。最终保存10°、0.100 N·m、双侧0%、DISABLED，方向NVS保留。随后上电，同一boot ID下91.63秒采样跨度内双侧新增反馈约99.96/99.97 Hz、REST/故障0、CAN错误0、运动输出0；一轮后上电无复位/无手动恢复通过。操作者确认当前穿戴中立静止后，双髋RAM零位网页写入/读回通过；人体角约0°、有效输入mask=0x0000000F、START门禁就绪，但仍IDLE/START请求0/运动输出0。新策略运动中SHADOW/ACTIVE NOT RUN；完整网页START/STOP E2E尚未通过。见[0.10.7部署证据](PHASE_10_0_10_7_DEPLOYMENT_20261010.md)。阶段10整体仍未完成。

2026-10-10新增0.10.7软件策略：单向角度阈值助力与运动页策略选择。阈值1–60°、附加力矩0–0.500 N·m，1°退出回差、触发渐入、重新启动/失效后须新前抬。真实C回放7/7、原集成4/4、网页78/78及IDF构建通过。见[新策略软件证据](PHASE_10_ANGLE_THRESHOLD_ASSIST_20261010.md)。

2026-10-10 14:22最新：操作者授权并断开动力后，0.10.6冻结版本烧录成功，三处哈希校验通过，
网页实际读回0.10.6。断电观察窗口内IDLE/输出0、RX低、发送队列0、CAN错误0；
14:27～14:28操作者上电后同一boot ID自动恢复双侧新REST反馈约100 Hz、CAN错误0、输出0；
一轮后上电无复位恢复通过，采样跨度97.65秒，多轮/耐久及人体助力未通过。
见 [0.10.6部署记录](PHASE_10_0_10_6_DEPLOYMENT_20261010.md)。
下方0.10.5操作记录是历史证据，阶段10整体仍未完成。

2026-10-10最新：0.10.5已烧录并由网页读回；动力后上电首次恢复失败（TEC=144），
确认EN复位后双侧100 Hz REST反馈恢复。单侧报告中断的自动恢复和网页按钮手动恢复通过，
全程IDLE、运动输出0；网页还观察到一次写入超时。0.10.6新增RX/待发队列等待条件，
回放、网页74项及IDF构建通过，尚未烧录，后上电HIL待做。阶段10整体未完成。
13:35最新采样boot ID再次变为0x97E2D738、双侧反馈0、TEC/REC=152/0；IDLE/输出0，
复位原因待确认。先前恢复通过仅代表该次操作，当前CAN故障尚未解决。
13:43操作者再次重新连接/上电，新boot ID为0x8E6D6A32、双侧反馈0、TEC/REC=144/0、输出0；网页BLE重连通过，CAN恢复仍失败。
见 [0.10.5部署与0.10.6候选](PHASE_10_POST_FLASH_HIL_20261010.md)。

2026-10-10烧录前历史反馈再次中断：当时0.10.2在IDLE参数应答可达，显式report1恢复双侧约100 Hz REST反馈；
12:59复核TEC/REC=0/0，历史累计错误未增长。0.10.5增加有限待机恢复、SS维护帧与网页CAN诊断，
当时软件回放/构建通过但尚未烧录；之后部署与HIL结果见上方最新记录。电气根因仍待测，阶段10整体未完成。
见 [本次反馈复发与修复候选](PHASE_10_FEEDBACK_RECURRENCE_20261010.md)。

2026-10-09新增现场故障：操作者在0.10.2网页选择ACTIVE 10%并启动后约8秒双电机反馈失效，
后续UART证实CAN BUS_OFF；网页STOP关闭主控输出，但电机失能确认失败并进入ERROR。
这是一次未通过的ACTIVE尝试，没有有效双侧步态或实际附加助力通过证据。
详见 [ACTIVE/CAN故障记录](PHASE_10_ACTIVE_BUS_OFF_20261009.md)。阶段10整体仍未完成，
不能把下方2026年8月的工程验收或此前“真人未执行”历史状态当成本次现场正常状态。

同日新增0.10.3软件候选：校准故障提示、最大附加助力力矩配置及网页助力门禁低频读回。
软件构建/回放已通过；候选尚未烧录，新功能目标板E2E未执行，CAN故障未解除。
详见 [力矩配置软件证据](PHASE_10_ASSIST_TORQUE_LIMIT_20261009.md)。这不增加真人验收通过项。

同日后续CAN复核：断电、操作者再次上电两次只读均为ERROR/BUS_OFF，错误计数不变。
0.10.3候选新增运行期CAN/双髋健康门、首因锁存和一次异步清理，主机回放7/7与构建通过；
尚未烧录、恢复总线或执行新HIL。详见 [CAN排查与保护证据](PHASE_10_CAN_DIAGNOSIS_20261009.md)。
操作者随后自行重启，Agent仅做参数读取及重新请求report1：当前0.10.2回IDLE/RUNNING，
0x10/0x20参数应答和100 Hz双反馈恢复，电机新反馈均REST。约37秒无运动采样未新增总线错误。
无新START/使能或人体运动验收；原ACTIVE后的BUS_OFF根因仍待测，不以恢复替代新保护HIL。

## 1. 基线

阶段 10 开始提交：`9225bb300dde0f722d0f0ad0a30443a30b90c8bc`。

2026-08-11 阶段 9 基线复核：

| 项目 | 结果 |
| --- | --- |
| `idf.py build` | PASS；应用 654096 bytes，app partition 剩余 38% |
| `tools/phase9_replay.ps1` | PASS，8/8 |
| `tools/phase9_ble_acceptance.py --self-test` | PASS，9/9 |
| `npm test` | PASS，42/42，并包含生产构建 |

2026-08-11 阶段 10 软件验收证据：

| 项目 | 结果 |
| --- | --- |
| `idf.py build` | PASS；应用 663600 bytes（`0xa2030`），app partition 剩余 37%；较阶段 9 增加 9504 bytes |
| `tools/phase9_replay.ps1` | PASS，8/8 |
| `tools/phase9_ble_acceptance.py --self-test` | PASS，9/9 |
| `tools/phase10_replay.ps1` | PASS，6/6 |
| `tools/phase10_ble_acceptance.py --self-test` | PASS，8/8 |
| `npm test` | PASS，50/50，并包含生产构建 |
| `npx tsc --noEmit` | PASS |
| `npm run lint` | PASS，0 error、0 warning |
| `git diff --check` | PASS；仅 Git 的 LF/CRLF 工作区提示，无 whitespace error |

2026-08-12 `0.10.1` 最终软件与工程真机增量证据：

| 项目 | 结果 |
| --- | --- |
| `idf.py build` | PASS；应用 665104 bytes（`0xa2610`），app partition 剩余 37% |
| 阶段9/阶段10 C回放 | PASS，8/8、6/6；真实模块助力集成回放3/3 |
| 阶段9/阶段10 BLE自测 | PASS，10/10、9/9；运动门禁自测3/3 |
| 网页测试/构建 | PASS，53/53；TypeScript与ESLint均PASS |
| `0.10.1`烧录 | PASS；ESP32-S3、COM5、Flash写入与哈希校验通过 |
| 工程轨迹真机 | PASS；153个完整样本，详见8.3与第10节 |

## 2. 分层测试矩阵

| 层级 | 测试对象 | 必测行为 | 通过标准 |
| --- | --- | --- | --- |
| 主机纯 C | 相位 LUT | 节点、区间中点、0/1 回绕、非法 profile、NaN/Inf | 与黄金向量误差不超过一个定点/CAN量化 LSB |
| 主机纯 C | level/峰值 | 0%、1%、100%，左右不同 level | 0%严格为0；缩放公式一致；结果有限 |
| 主机纯 C | 失效退化 | 低置信度、无效相位、旧序号、错误 epoch | 不产生可用于 ACTIVE 的旧建议；输出按合同有界归零 |
| 主机纯 C | 命令合成 | DISABLED、SHADOW、ACTIVE 0%、ACTIVE 非零 | 前三者保持基础命令；ACTIVE只改变 `torque_ff_nm` |
| 主机协议 | BLE v1 | 新 opcodes、长度、保留位、错误码、遥测包 | 命令响应仍与 request ID 严格关联；遥测每包不超过20 B |
| 目标板 | 资源与并发 | 固定内存、任务优先级、队列满、断连 | 控制路径不阻塞；断连停止流但不改变整机 RUN/STOP |
| 网页 | 兼容与显示 | 0.9无遥测、0.10遥测、丢包、暂停、导出 | 旧设备可用；序号跳变可见；暂停不停止采集；导出单位正确 |
| 真机 | SHADOW | 正常步态下曲线建议 | 建议周期和左右符号可见，最终 CAN 命令不受影响 |
| 真机 | ACTIVE 0% | 完整真实输出链 | 与相同基础配置一致；实际助力为0；每周期只有一对发送 |
| 真机 | ACTIVE低幅 | 5%和10%分级 | 实际力矩与公式/限幅相符；STOP后IDLE、REST、助力清零 |

## 3. 静态代码审计

必须逐项确认：

- [x] 只有 `exo_control_loop` 创建永久 100 Hz 运动任务；
- [x] 相位策略不调用 CAN、BLE、日志、NVS/Flash 或动态内存；
- [x] 控制器计算与最终发送已拆开；
- [x] 每周期只有一个 `exo_joint_command_try_send_pair()` 调用路径；
- [x] 助力结果在合成前校验 activation epoch、传感序号和步态序号；
- [x] LEFT/RIGHT 使用各自相位，没有固定 `+0.5`；
- [x] `ASSIST_AS_NEEDED` 仍明确返回未实现；
- [x] ACTIVE 只允许 `DAMPING + CONSTANT`；
- [x] 遥测 push 为固定时间、零等待；
- [x] BLE 发送发生在低优先级路径；
- [x] 所有产品帧均不超过20 B；
- [x] 网页未知 capability/packet type 不会导致失控或无限重连；
- [x] 人体关节角度显示为 deg，力矩为 N·m，相位为 `[0,1)`/百分比。

## 4. 自动化命令

```powershell
# ESP-IDF
. .\tools\activate_esp_idf.ps1
idf.py build

# 阶段9回归
.\tools\phase9_replay.ps1
python .\tools\phase9_ble_acceptance.py --self-test

# 阶段10回放与协议自测（实现完成后）
.\tools\phase10_replay.ps1
python .\tools\phase10_ble_acceptance.py --self-test

# 网页
Set-Location .\web-bluetooth-console
npm test
npx tsc --noEmit
```

所有命令必须保留退出码和 PASS/FAIL 计数。固件构建还要记录应用大小、剩余分区以及与阶段 9 的增量。

## 5. 真机验收顺序

真机运动前先确认 A/B 主动反馈有效、左右映射和软件零位仍为本次装配的有效值。按照下列顺序逐级放开，任何一级失败都不得跳到后一级：

1. 烧录当前 `0.10.1` 工程构建，读取 INFO、`0x00FF` capability 和遥测状态；
2. 双髋中立位重新执行软件归零；
3. `DAMPING + CONSTANT`、`ACTIVE_PHASE + PHASE_PROFILE + SHADOW`，运行60秒；
4. 确认建议存在、实际助力为0且控制统计正常；
5. 配置 ACTIVE 但左右 level=0%，运行60秒；
6. 确认实际输出链已进入但附加力矩严格为0；
7. STOP，确认 IDLE、A/B REST、旧建议失效；
8. 经操作人员明确确认后，依次执行5%和10%低幅验收；
9. 最终 STOP，重新读取系统、关节、助力、遥测和100 Hz诊断。

## 6. 实时与耐久标准

| 指标 | 标准 | 实际结果 |
| --- | --- | --- |
| 控制频率 | 100 Hz，mean period 约10000 us | PASS；mean 10000 us（100.000 Hz），min/max 9210/10763 us |
| deadline/skipped/output/algorithm error | 全部0 | PASS；0/0/0/0，算法最后结果 `ESP_OK` |
| 最大控制周期执行时间 | `< 5000 us` | PASS；1653 us |
| 控制任务最小剩余栈 | `>= 1024 bytes` | PASS；1460 bytes；BLE command/telemetry 任务分别为1600/1528 bytes |
| 正常遥测速率 | 配置20 Hz时实测18～22 Hz | PASS；600 s内12000样本，20.00 Hz |
| 正常遥测缓冲丢弃 | 连续10分钟为0 | PASS；12000个样本序号连续且通道完整，运行中 dropped=0，最终队列0/32 |
| 瞬时拥塞恢复 | 重试耗尽为0，控制指标不恶化 | PASS；mbuf分配压力71次、38帧经重试恢复、exhausted=0、submit/TX fail=0 |
| 持续过载行为 | 最终允许显式丢遥测，但控制指标不恶化 | NOT RUN；全通道20 Hz压力工况留待独立压力测试 |
| STOP终态 | IDLE；A/B REST；实际助力0；旧epoch失效 | PASS；2026-08-12 SHADOW 无有效动作尝试结束后已核对 |

## 7. 无运动真机证据与缺陷闭环

2026-08-11 在 `COM5` 烧录 `0.10.0` 后执行了不会归零、不会发送整机START/STOP、不会产生电机运动的 BLE 验收：

1. GET INFO 返回固件 `0.10.0`、capabilities `0x007F`；默认遥测为20 Hz、mask `0x0078`；
2. 第一版10分钟测试虽然平均19.96 Hz，但设备 dropped=81，判定失败并保留证据；
3. UART定位为 NimBLE `BLE_HS_ENOMEM`，不是100 Hz超期或无线TX失败；
4. 修复包括：NimBLE MSYS1静态块12增至24、遥测任务栈3072增至4096 bytes、默认4通道按10 ms摊开发送、低优先级任务保留当前样本并在1秒预算内等待mbuf释放；
5. 最终严格10分钟复测 PASS：`frames=47998`、`samples=12000`、`complete=12000`、`20.00 Hz`、运行中 `dropped=0`、9/9；
6. 复测后 UART：alloc-fail=71、retry-ok=38、exhausted=0、submit-fail=0、TX-fail=0，控制任务100.000 Hz且无deadline/skipped/output/algorithm error；
7. 整个无运动验收期间系统保持IDLE，START请求数为0，输出门关闭，真实助力为DISABLED。

## 8. 2026-08-12 运动验收尝试

本轮开始前，操作者已明确确认“双髋已处于中立位，可以重新归零并开始阶段 10
运动验收”。在 `COM5` 上执行只读预检后，A/B 主动反馈均约为 100 Hz、控制模式为
REST、无电机故障，TWAI 状态为 RUNNING 且发送/接收错误计数均为 0。

### 8.1 软件归零与起动前配置

| 项目 | 实际结果 |
| --- | --- |
| LEFT 软件零位 | 捕获原始角 `-18.210 deg`，归零后 `0.000 deg` |
| RIGHT 软件零位 | 捕获原始角 `+4.824 deg`，归零后 `-0.022 deg` |
| 传感快照 | `available=valid=0x0000000F`，双侧在线且反馈新鲜 |
| 系统门禁 | IDLE，`Ready to start=YES` |
| 基础控制 | `DAMPING + CONSTANT`，左右 `q/dq/tau/kp=0`、`kd=0.1` |
| 高层助力 | `ACTIVE_PHASE + PHASE_PROFILE + SHADOW`，profile 1，左右 10% |

START 后系统正常进入 RUNNING，输出许可打开；60 秒默认 20 Hz 遥测收到
`4797` 帧、`1199` 个完整样本，设备丢样为 0。所有样本均保持 RUNNING，SHADOW
期间 `real_output_enabled=false`，实际附加助力 `1199/1199` 严格为 0，证明 SHADOW
隔离与遥测链路工作正常。

但是该 60 秒内没有形成有效步态：左右相位有效样本均为 0、有效置信度样本为 0、
非零曲线建议样本为 0。结束前诊断显示 LEFT/RIGHT 分别约为 `-0.154/0.000 deg`，
估计器始终为 STANDING、`motion=no`、quality=`0x00000004 (LOW_MOTION)`。
因此该轮记作“无有效人体动作输入”，不能冒充 SHADOW 曲线验收 PASS，也不能据此
判定相位策略算法失败。

随后执行 STOP，系统返回 IDLE，A/B 均为 REST、无电机故障、TWAI 错误计数仍为 0；
100 Hz 路径 deadline/skipped/output/algorithm error 均为 0，最大执行时间 `3265 us`
（仍小于 5 ms），控制任务最小剩余栈 `1092 bytes`（仍大于 1024 bytes）。

为排除提示时序问题，又在电机保持失能的 IDLE 状态执行了 5 秒倒计时加 30 秒步态
预检。全程仍为 STANDING，最终 LEFT/RIGHT 为 `-0.132/-0.022 deg`，速度仅
`1.808/0.885 deg/s`，未达到估计器的运动与周期锁定门槛。下一轮必须由操作者在听到
电脑提示音后实际执行连续交替髋运动；预检锁定前不得进入 ACTIVE 0%、5%或10%。

操作者随后确认已就位且能够听到电脑提示音。重新连接编程 UART 时 ESP32 发生正常
自动复位，因此该次在 INIT 门禁处中止、没有播放开始音，也没有发送运动命令。等待
启动完成后重新捕获 LEFT/RIGHT RAM 零位为 `-18.342/+4.824 deg`；归零后系统为
IDLE、Ready=YES、输出关闭、传感 `available=valid=0x0000000F`。正式 30 秒失能预检
记录 50 个状态样本，左右抽样摆幅均仅 `0.022 deg`，motion/valid/LOCKED 均从未出现。

为进一步排除 UART 复位及采样通道影响，又在不复位设备的 BLE 只读连接上执行第三轮
5 秒倒计时加 30 秒预检，共读取 49 组 gait/joint 状态。系统门禁始终为 IDLE、
Ready=YES、输出关闭且双侧 joint valid；左右观测摆幅均为 `0.00 deg`，状态集合仅为
STANDING/ACQUIRING，motion、overall valid、LOCKED和有效置信度仍全部为 0。该结果
证明测试窗口内编码器没有观测到关节转动；下一步应先做“单侧转动并保持”的静态反馈
检查，确认人体机构确实带动对应 RS01 输出轴后再重复周期步态验收。

随后分别执行静态联动检查：LEFT 保持时读数稳定为 `-11.058 deg`，RIGHT 保持时
读数稳定为 `-13.235 deg`；两侧 joint flags 均为 `0x3F`，反馈年龄均不超过
`10 ms`，系统始终 IDLE 且输出关闭。因此左右机械关节、映射和编码器静态联动均
为 PASS，前述零摆幅不是反馈链损坏。

### 8.2 历史结果与双轨需求澄清

上述失败发生在`0.10.0`：操作者当时期待设备主动产生左右交替运动，但该版本只有
“人体先带动关节 -> 编码器估计相位 -> 按相位叠加助力力矩”的产品反馈链。静止时
estimator进入STANDING并清除相位，因此该轮没有有效测试输入，不是人体助力算法的
通过证据，也不是算法失败证据。

随后需求被明确为两条并存且不能互相替代的链路：产品必须使用人体主动运动驱动
`ACTIVE_PHASE`；另提供受严格隔离的主动轨迹，只为自动制造可重复的工程测试输入。
因此`0.10.1`新增`ALTERNATING_SINE_TEST`，固定左右反相轨迹并且只允许
`DISABLED/SHADOW`。它不会取代产品反馈链，也不会把上述历史失败改写成PASS。

### 8.3 `0.10.1` 隔离工程轨迹真机结果

2026-08-12烧录开启工程能力的`0.10.1`（INFO capabilities `0x00FF`），在双髋
软件归零且A/B主动反馈有效后执行`IMPEDANCE + ALTERNATING_SINE_TEST`、左右
`kp=5.0/kd=0.1`和`ACTIVE_PHASE + PHASE_PROFILE + SHADOW`。最终15秒严格复测：

- 153个遥测样本通道完整且序号连续；
- LEFT/RIGHT反馈峰峰值为`14.6/15.4 deg`，相关系数`-0.989`；
- 有效相位样本`62%`，非零SHADOW建议样本`43%`，其余置信度门槛通过；
- 所有样本的实际附加助力均为0，`real_output_enabled`始终关闭；
- 工具在LEFT/RIGHT为`0.52/1.89 deg`的中立交叉处发送STOP；系统回到IDLE，
  随后逐项读回`DAMPING + CONSTANT`、左右`kp=0/kd=0.1`和助力DISABLED。

另在目标板故意配置`ALTERNATING_SINE_TEST + ACTIVE`：`Ready to start=no`，
阻止原因`RUN_CONTRACT_INVALID`，START同步返回`REJECTED_NOT_READY`；A/B的
`Enabled requested=no`且模式均为REST，证明拒绝发生在电机使能之前。

随后在不使用工程轨迹的产品配置`DAMPING + CONSTANT + ACTIVE 10%`下保持设备
静止3秒：估计器为`STANDING / motion=no`，建议无效且左右建议力矩为`0/0 N·m`，
控制器`accepted/applied=0/0`，真实输出标志关闭。STOP后系统回到IDLE、A/B均为
REST。这形成了“没有人体步态就没有助力”的目标板证据；它仍不能替代真人迈步后
ACTIVE低幅输出的下一阶段验收。

## 9. 当前验收结论

当前状态：`0.10.1 ENGINEERING MOTION HIL PASS; HUMAN ACTIVE HIL NOT RUN`。

| 独立验收轨道 | 软件状态 | 真机状态 |
| --- | --- | --- |
| 工程主动激励 -> 真实反馈 -> 步态 -> SHADOW -> 遥测 | 已实现并完成主机真实C集成回放 | 空载/台架SHADOW HIL：PASS |
| 人体主动运动 -> 步态 -> ACTIVE_PHASE -> 唯一命令合成 | 已实现并完成主机真实C集成回放 | 真人SHADOW及ACTIVE 0%/5%/10% HIL：NOT RUN |

`0.10.0`的软件、网页、无运动BLE遥测耐久以及双侧静态反馈联动已有证据；`0.10.1`
新增工程激励的主机门禁、真实模块集成测试和真机SHADOW轨道已有证据。尚未执行的
真人运动HIL必须继续标记为`NOT RUN`。工程主动激励通过只完成第一行，不能自动把
第二行标为PASS。

阶段10只有在两条轨道各自要求的真机证据均留下记录后才能整体标记完成。若当前没有
条件执行某一级，交付结论仍必须写成“代码/设计完成，硬件验收未完成”。

## 10. `0.10.1` 主动工程激励验收边界

最终产品必须保留“人体先运动、外骨骼根据实时步态再助力”的链路。当前软件已经
实现该反馈驱动链，但真人`ACTIVE` HIL仍为`NOT RUN`，不能标记为整机验收通过。

`ALTERNATING_SINE_TEST`只是隔离工程输入：固定单侧10 deg、0.5 Hz、3 s渐入、
左右反相。它可替代人工扳动来稳定激发编码器和步态估计，但只能搭配`SHADOW`，
`APPLIED_TORQUE`必须严格为0。

| 项目 | 通过标准 | 当前状态 |
| --- | --- | --- |
| 主机运动门禁自测 | 缺任一道授权时连接BLE前拒绝；默认只读 | PASS，3/3，2026-08-12 |
| 主机轨迹数学 | 10 deg、0.5 Hz、3 s渐入、左右和为0、稳态相关系数-1 | PASS，2026-08-12 |
| 真实C模块集成 | 直接链接snapshot、gait、assist、profile、composer、controller | PASS，3/3，2026-08-12 |
| `TEST + ACTIVE`门禁 | START前拒绝；controller preflight/arm均拒绝 | PASS；目标板返回`RUN_CONTRACT_INVALID`，A/B保持REST |
| 新activation事件门 | 旧LOCKED相位全零等待；左右新峰到齐后从0渐入 | PASS，主机真实C；目标板静止ACTIVE实际施加0次 |
| 空载/台架SHADOW | 左右反馈峰峰值10..30 deg、相关系数<=-0.70 | PASS；14.6/15.4 deg，相关系数-0.989 |
| 步态与建议 | 有效相位>=50%、置信度>=0.4样本>=25%、非零建议>=25% | PASS；相位62%，置信度与建议均满足门槛，建议43% |
| SHADOW隔离 | 全样本实际附加助力0，real-output位始终0 | PASS；153个完整样本均为0/关闭 |
| finally STOP | 无论成功、异常或超时，最终IDLE且A/B REST | PASS；中立交叉0.52/1.89 deg后STOP |
| 配置恢复 | STOP后读回产品DAMPING+CONSTANT、kd=0.1、助力DISABLED | PASS，逐项读回一致 |
| 真人主动运动SHADOW | 穿戴者主动迈步后相位和建议有效 | NOT RUN |
| 真人ACTIVE 0%/5%/10% | 独立分级验收并保留原始遥测 | NOT RUN |

主机门禁命令：

```powershell
python .\tools\phase10_motion_acceptance.py --self-test
.\tools\phase10_assist_integration_replay.ps1
```

真实C集成回放覆盖：人体反相髋反馈经过真实步态估计后，SHADOW只建议不施加，ACTIVE
在有效步态下进入唯一命令合成；无效步态清零；旧epoch、传感序号、步态序号和时间戳
逐项拒绝；A/B部分发送如实进入诊断。host fake只替代RTOS临界区和最终CAN发送。

主动工程激励即使全部PASS，也只能关闭“自动产生可重复反馈输入”的测试缺口，不能
把最后两项真人HIL自动改成PASS。完整执行入口见
`PHASE_10_ENGINEERING_STIMULUS.md`。

### 2026-10-09当前板卡接续与真实网页驱动

本轮已确认ESP32-S3 rev0.2、16 MB Flash/8 MB内置PSRAM，TX4/RX1；备份旧镜像
`83b384a / IDF v5.3.1-dirty`后升级为0.10.1工程构建。经操作者本轮台架/中立位确认，
真实网页完成RAM软件零位、TEST+SHADOW配置及START，213个完整运动样本左右峰峰值
17.074°/15.986°、相关系数−0.988、实际附加助力全部0。第一轮由串口20秒保护先STOP，
不算该工程轨迹的网页STOP通过；另在恢复产品DAMPING配置后完成真实网页START/STOP补测，
保护未介入，最终IDLE/A-B REST/CAN错误0。

七通道20 Hz静止短测已出现drop408和满队列，未通过；本轮未做真人SHADOW/ACTIVE、
遥测断开/重连、关闭工程开关后的目标板验收或Git收口。板上目前仍声明工程capability0x00FF，
根sdkconfig开关关闭不能代替板上关闭验证。阶段10总体完成状态不变。
详见 `PHASE_10_WEB_MOTION_20261009.md` 及其原始证据清单，不将8月历史状态混作当前状态。

### 2026-10-09 0.10.2穿戴产品标定接续

0.10.2新增只读出厂映射/连续电机角/RAM零位读回，网页提供左右现场核对和穿戴中立位保存。
本轮已构建并烧录工程开关关闭的产品候选，实际网页读回0.10.2/cap0x017F/Boot0x3503C3B7。
操作者确认本次左右与方向正确、静止中立位后，真实网页保存双髋RAM零位并读回髋角约0°。
此时IDLE、双电机REST、主动反馈约100 Hz，未执行机械置零或START。
主机协议5/5、网页55/55、TypeScript/ESLint均通过。只证明当前静止标定链，
真人运动SHADOW及ACTIVE分级仍需后续独立验收，阶段10整体完成状态不变。
详见 `PHASE_10_PRODUCT_CALIBRATION_20261009.md`。

### 2026-10-10 0.10.4出厂方向与穿戴零位软件接续

0.10.4候选将出厂方向配置与穿戴零位分开：手动前抬示教确定左右正方向，NVS保存出厂方向，
每次穿戴仅采集RAM零位。角度、速度、力矩和下行五元命令统一使用该方向；首次无有效
出厂记录时不复用历史方向作为已标定状态。网页重连读回出厂方向，未连接或不支持时锁定新命令。
真实C/存储回放11/11、网页68/68、TypeScript/ESLint及ESP-IDF 5.5目标构建通过。
本轮未烧录、未执行方向真机写入或硬件断电恢复、未校准穿戴者、未START。
阶段10整体和阶段11完成状态不变；此前ACTIVE CAN故障仍需独立复验。
详见 `PHASE_10_DIRECTION_TEACHING_20261010.md`。
