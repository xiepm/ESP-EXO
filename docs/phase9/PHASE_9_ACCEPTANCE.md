# 阶段 9：统一传感、步态估计与 SHADOW 助力框架验收

## 1. 当前结论

当前阶段结论：**COMPLETE / PASS（阶段 9 工程与功能范围）**。正常人体左右交替步态、变速响应、停止
退锁、静止清历史和全程 SHADOW/真实输出隔离均已完成真机验收。外部 gait event 真值仍为
**NOT MEASURED / OUT OF SCOPE**，因此本阶段不宣称 heel strike/toe off 或绝对相位误差精度。

2026-08-11 的有效人体试次中，正常段 `LOCKED=19`、`valid=19`，左右圆周相位距离中位数为
`0.4288`，步频中位数为 `91.63 spm`；加速段 `LOCKED=12`、`valid=12`，相位距离中位数为
`0.4750`，步频中位数增至 `124.47 spm`。停止后回到 STANDING；针对原始基线受准备动作污染的问题，
另做 8 s 静止补测，20/20 样本均为 STANDING、quality=`0x0004`。详细证据见第 9.5 节。

2026-08-10 已完成主机回放、ESP-IDF v5.5 最终构建、网页测试、BLE 主机/真机协议测试，以及 UART、
100 Hz、算法耗时、TWAI 和双电机在线检查。收到用户“双髋已处于中立位”的明确确认后，已完成左右 RAM
软件归零、静止观察、透明 SHADOW 下 START/RUNNING/STOP 生命周期，以及既有低层
`IMPEDANCE + SINE` 的 12.2 s 自动运动回归。自动刺激期间估计器能够进入 LOCKED，但左右电机由同相正弦
驱动，质量位 `0x4000` 正确指出双侧不一致并降低置信度；该结果不能替代正常人体左右交替步态、变速响应
或有外部真值的相位准确率验收。详细证据见第 8 节和第 9.2 节。

2026-08-10 自动同相刺激停止点 LEFT `18.221 deg`、RIGHT `13.254 deg` 的历史状态已由用户手动回中立位
并在 2026-08-11 重新捕获 RAM 软件零位。最新试次结束后设备为 IDLE、A/B 反馈有效、assist
`real_output_enabled=false`；仍不能把“电机已 REST”理解为“关节会自动回零”。

阶段 8 的历史结果只能作为回归基线，不能当作阶段 9 固件的验收结果。

## 2. 验收范围

阶段 9 只验收以下能力：

- 唯一 100 Hz 周期构建统一 sensor snapshot；
- `HIP_EVENT_TBE` 在回放和真机上给出可解释的状态、相位、步频、置信度和质量位；
- 静止、起步、速度变化、掉线、时间/坐标复位时不沿用错误历史；
- 高层助力配置可通过 UART/BLE 设置和读回；
- `SHADOW` 建议可观察，但不会改变阶段 8 真实 CAN 输出；
- 网页可以显示 gait/assist 状态并只提供阶段 9 允许的配置；
- 新算法没有破坏 100 Hz 实时性、栈余量、TWAI 和 STOP/REST 回归。

不在本阶段验收：论文级 heel strike/toe off 精度、自适应振荡器/HMM/LSTM 的复现、相位曲线真实助力、
AAN 效果、代谢消耗或临床指标。

## 3. 软件验收合同

| 项目 | 通过门槛 |
|---|---|
| 输入一致性 | 每个算法结果绑定本周期 sensor/gait sequence；重复的异步反馈不被重复滤波 |
| 时间和坐标复位 | 时间/序号回退、连续坐标或 RAM 零位改变后推进 `reset_epoch` 并清除旧步态历史 |
| 新鲜度 | 超过 50 ms 的反馈不进入 valid mask；左右来源差超过 20 ms 置质量位 |
| 非有限数 | NaN/Inf 不进入滤波历史，结果无效且有质量原因 |
| 静止 | 连续 1.5 s 低运动进入 STANDING，phase/cadence/confidence 不再宣称有效 |
| 锁定 | 每侧至少两个稳定的同类屈曲峰周期后才允许 phase ready |
| 相位 | 有效区间 `[0,1)`；同一周期内不倒退；只在新屈曲峰报告 wrap |
| 异常周期 | 周期在 0.5~3.0 s 之外不锁定；变化超过 35% 标记不稳定并重新累计 |
| 双侧语义 | 单侧有效不等于 overall valid；双侧不一致只降权/置位，不伪造固定 0.5 相差 |
| 助力策略 | 编译期静态表；最大 64-byte 工作区；step/reset 无动态内存、阻塞、日志、BLE、CAN |
| SHADOW | `ACTIVE` 配置被拒绝；`real_output_enabled=false`；assist result 不进入真实控制发送链 |
| 生命周期 | IDLE 修改 selected；START 锁存 active；RUNNING 配置 BUSY；STOP 清 active/建议、保留 selected |
| BLE | 新完整响应不超过 20 byte；无效定点值为 `0xFFFF`；线枚举与内部枚举显式转换 |
| 实时结构 | 全工程仍只有一个 `xTaskDelayUntil()` 周期源，阶段 9 不创建额外控制任务 |

## 4. 主机回放验收

### 4.1 必测数据集

主机回放应直接编译/调用仓库中的 `exo_sensor_snapshot.c` 和 `gait_estimator.c`，不能用另一份 Python
公式替代真实 C 实现。建议至少覆盖：

| 场景 | 输入 | 预期结果 |
|---|---|---|
| 静止 | 双侧角度/速度近零，持续超过 1.5 s | STANDING；无有效 phase/cadence/confidence；无假事件 |
| 正常周期 | 双侧近似正弦、相位大致交替，周期稳定 | 先 ACQUIRING 后 LOCKED；左右 phase 均在 `[0,1)`；cadence 有限 |
| 速度变化 | 周期分段改变但仍在 0.5~3.0 s | 不产生 NaN/倒退；必要时标记不稳定并重新积累 |
| 小幅噪声 | 摆幅小于 6 deg 或速度未过滞环 | 不因噪声锁定 |
| 周期越界 | 小于 0.5 s 或大于 3.0 s | 对应 OUT_OF_RANGE 质量位；旧周期不继续外推 |
| 单侧暂失 | 一侧输入有效、另一侧无效/陈旧 | DEGRADED 对外清除有效量，不把旧双侧结果保留 |
| 左右时间差 | 来源时间差超过 20 ms | snapshot 与 gait 均保留质量证据，置信度按实现降权 |
| 数据陈旧 | 来源年龄超过 50 ms | 相应 sensor valid 位清除 |
| 坐标/零位变化 | position epoch 或零位签名变化 | reset epoch 递增；旧滤波、周期和相位清除 |
| 时间/序号回退 | captured time 或 source sequence/time 回退 | 质量位置位，估计器重置且不发布陈旧相位 |
| 非有限输入 | 角度/速度/力矩/温度含 NaN/Inf | valid 位清除，滤波状态不被污染 |
| 确定性 | 相同输入文件重复执行至少两次 | 状态、事件、定点输出和统计逐项一致 |

### 4.2 主机通过判据

- 测试进程返回码为 0；
- 所有断言均 PASS，没有跳过异常/复位用例；
- phase、period、cadence、confidence 全程无 NaN/Inf；
- 每次 wrap 前同侧 phase 单调不下降，wrap 后从新原点重新开始；
- 静止、无效输入或 reset 后不继续输出上一个周期的有效相位；
- 测试日志记录实际锁定时间、wrap 数、最终 cadence 和所有失败用例名称。

当前仓库的执行入口为：

```powershell
powershell -ExecutionPolicy Bypass -File tools/phase9_replay.ps1
```

该脚本通过 WSL/GCC 直接编译仓库中的 snapshot、gait 和 BLE protocol C 源，再运行
`tools/phase9_gait_replay.c`；因此主机需要已安装 WSL 和 GCC。现有测试覆盖静止、重复来源、正常反相锁定、
可疑同相降权、速度突变、左右 skew/单侧/坐标复位、缺失来源不造成 reset storm，以及阶段 9 BLE 向量。
本轮脚本实际结果为 **8/8 PASS**。表 4.1 中未被现有用例单独覆盖的非有限输入、周期上下界和重复运行
逐项一致性仍属于后续测试增强项；本轮不把脚本返回 0 扩大解释成这些细分场景已经全部覆盖。

## 5. 构建、静态结构与网页验收

### 5.1 ESP-IDF 最终构建

在已加载 ESP-IDF v5.5 PowerShell 环境后执行：

```powershell
idf.py fullclean
idf.py build
idf.py size
```

通过门槛：

- target 为 ESP32-S3；
- `PROJECT_VER` 和 BLE GET_INFO 均为 `0.9.0`；
- 0 error、0 compiler warning；
- 应用镜像能放入当前 app partition；
- 记录应用镜像字节数、分区剩余字节/百分比和 SHA-256（若流程提供）。

### 5.2 静态结构检查

需要保存 `rg` 或代码审查证据，确认：

- 唯一周期调用链为 `exo_control_loop`；
- `exo_assist.c` 不包含 `rs01_send*`、TWAI 发送、BLE notify 或任务创建；
- `assist_result` 只进入诊断快照，没有传给 `exo_controller` 或 `exo_joint_command`；
- 相位曲线/AAN 描述符为 `implemented=false` 且 `ops=NULL`；
- 运行期没有策略 malloc/calloc/realloc/free；
- 新 BLE 线 DTO 逐字段编码，不对内部 C 结构直接 `memcpy`；
- `app_main.c` 只做一次 `exo_assist_init()` 和既有控制任务初始化。

### 5.3 网页测试

在 `web-bluetooth-console` 目录执行仓库真实脚本：

```powershell
npm test
npx tsc --noEmit
npm run lint
```

通过门槛：

- 协议解析覆盖 capability bit 4/5、`0x86/0x87/0x96`、16-byte 状态和 `0xFFFF` 哨兵；
- 设备兼容检查接受固件 `0.9.0` 且不破坏旧功能；
- 页面能显示 estimator state、左右相位、cadence、confidence、quality、selected/active、unavailable reason；
- 页面只能设置 DISABLED/SHADOW，不能发送 ACTIVE；
- 自动轮询不会让事务日志强制抢回底部，用户向上滚动时可稳定阅读；
- test、TypeScript 和 ESLint 全部返回 0。

## 6. BLE/UART 功能验收

### 6.1 UART

依次记录：

```text
sensor_status
gait_status
assist_status
assist_config transparent shadow transparent 0 0 0
assist_status
```

通过门槛：

- 三个 status 均来自最新 100 Hz 发布快照，不创建新的算法任务；
- sensor sequence 持续递增，双髋在线时 valid mask 与当前硬件能力一致；
- gait 输出明确显示 `HIP_FLEXION_PEAK (not heel strike)` 语义；
- IDLE 配置透明 SHADOW 后 GET/状态读回一致；
- RUNNING 时同一配置命令被维护门拒绝；
- STOP 后 active/locked/suggestion 清除，selected 保留。

### 6.2 BLE

推荐先运行可重复的验收客户端；第一条纯主机测试，第二条连接真机但不发送
START/STOP/归零，配置检查结束后会恢复进入测试前的 selected 配置：

```powershell
python tools/phase9_ble_acceptance.py --self-test
python tools/phase9_ble_acceptance.py --name ESP-EXO --verify-config
```

通过网页或验收客户端执行：

1. GET_INFO，确认固件 `0.9.0`、capability `0x003F`、max frame 20；
2. GET_GAIT_STATUS，核对 request_id 和 16-byte payload；
3. GET_ASSIST_CONFIG，核对默认 selected=`TRANSPARENT`、effective execution=`DISABLED`；
4. IDLE 下 SET `TRANSPARENT + SHADOW + TRANSPARENT`，收到 OK 后再次 GET 并逐字段读回；
5. 尝试 ACTIVE，必须得到明确拒绝且 `real_output_enabled` 仍为假；
6. START 后确认 active/locked，且执行状态打包字节为 `0x11`（低半字节 requested=`SHADOW`，
   高半字节 effective=`SHADOW`）；RUNNING 写配置必须 BUSY；
7. STOP 后确认 active 清除、selected 保留；
8. 重复旧阶段 6/8 只读与配置回归，旧 opcode 行为不变。

BLE 自动轮询建议不高于网页当前设定频率；BLE 状态查询不是 100 Hz 实时控制链，也不能用手机连续写帧
代替本地控制任务。

## 7. 真机验收

### 7.1 前置记录

烧录 `0.9.0` 后先记录：

- COM 口、芯片、boot ID、镜像版本/大小；
- 系统为 IDLE、输出未许可；
- A/B 在线、REST、无 fault/warning；
- LEFT/RIGHT 映射和 RAM 软件零位有效；
- 100 Hz task identity、TWAI 初始计数、control/algorithm error 初始计数；
- sensor valid/available mask、反馈年龄和左右 pair skew。

### 7.2 步态观察流程

阶段 9 不需要打开真实助力即可观察估计器。建议按以下顺序采集：

1. 静止站立至少 3 s：应进入 STANDING，无有效相位；
2. 开始规律原地髋屈伸或步行：应由 ACQUIRING 进入 LOCKED；
3. 稳定运动至少 20 个 gait cycle：记录左右 wrap、phase 范围、period、cadence、confidence 和 quality；
4. 改变速度：记录是否先降权/重新积累，确认没有 NaN、突跳到超范围或长期陈旧锁定；
5. 停止至少 3 s：应退回 STANDING 并清除有效相位；
6. 在 IDLE 重新捕获一次 RAM 零位或执行受控 reset 测试：`reset_epoch` 应变化，旧周期历史清除；
7. 若需要验证 START 生命周期，只配置透明 SHADOW；START/RUNNING/STOP 全程确认真实输出仍由阶段 8
   selected 控制，而不是 assist suggestion。

当前髋峰代理没有外部 gait event 真值，所以真机锁定只能证明算法运行和状态行为，不能证明论文级相位
准确率。若要给出相位误差，必须同步保存力平台、足底开关、动作捕捉或人工标注的参考事件。

### 7.3 实时和回归门槛

沿用阶段 8 门槛，并新增分项算法耗时记录：

| 指标 | 门槛 |
|---|---|
| 控制均值周期 | `10.0 ms +/-0.1 ms` |
| 9~11 ms 周期占比 | `>=99%` |
| 最大周期误差 | `<=2 ms` |
| 最大完整周期执行时间 | `<10 ms` |
| 控制任务最小剩余栈 | `>=1024 B`（4096-byte 静态栈的 25%） |
| snapshot/gait/assist 分项耗时 | 全部记录 last/max；不得使完整周期越过 10 ms |
| deadline/skipped/output error | 相对本轮基线不增长 |
| algorithm error | 0，最后错误为 `ESP_OK` |
| TWAI TX/RX/TX failed/bus/arbitration/RX missed | 相对本轮基线不增长 |
| STOP 后状态 | 系统 IDLE；A/B REST、未请求使能；assist active/suggestion 清除 |
| assist 真实输出 | `real_output_enabled=false`，全程不改变 |

### 7.4 阶段 8 历史基线（仅供比较）

阶段 8 最终记录为：mean `10000 us`、min/max `9490/10499 us`、最大执行 `1573 us`、栈余量
`2348 B`、deadline/skipped/output error=`0/0/0`、TWAI 错误 0。上述数字来自 `0.8.0`，只能用于比较
阶段 9 增量开销，不能填入 `0.9.0` 结果栏。

## 8. PASS / PENDING 总表

| 验收项 | 当前状态 | 当前证据/待补内容 |
|---|---|---|
| 统一 sensor snapshot 接口存在 | PASS（代码事实） | `exo_sensor_snapshot.*`；含 sequence、reset epoch、mask、quality、skew |
| HIP_EVENT_TBE 接口与状态存在 | PASS（代码事实） | `gait_estimator.*`；当前后端 ID=1，髋屈曲峰为 phase origin |
| 静态 assist ops / 64-byte workspace | PASS（代码事实） | `exo_assist.*`；编译期表和静态 union，无动态插件 |
| SHADOW 不接真实输出 | PASS（代码与真机诊断） | `real_output_enabled=false`；assist result 只进入控制诊断；真机读回同样为 false |
| 主机 snapshot/gait 回放 | PASS | `tools/phase9_replay.ps1`：8/8 PASS，返回码 0；未单独覆盖的细分场景见第 4.2 节 |
| BLE C 编解码/边界测试 | PASS | 阶段 6 自测 10/10 PASS；阶段 9 BLE 工具自测 9/9 PASS；回放中的阶段 9 BLE 向量 PASS |
| ESP-IDF v5.5 fullclean/build/size | PASS | ESP32-S3；0 warning/error；bin 654096 B（`0x9FB10`），app 余 `0x604F0`（38%），size total 653985 B；SHA-256 见第 9 节 |
| 网页 test/TypeScript/ESLint | PASS | `npm test` 42/42 PASS；`npx tsc --noEmit` PASS；`npm run lint` PASS |
| BLE 真机只读、配置与旧协议回归 | PASS | capability `0x003F`；历史 boot ID `0xCF6636D9` 上阶段 6 真机 15/15、阶段 9 真机 10/10 PASS；2026-08-11 重启后 boot ID `0xBA4031D7`，人体试次的 BLE 读取、归零和全程状态轮询均通过 |
| BLE START/STOP 生命周期 | PASS | 透明 SHADOW：IDLE 打包字节 `0x01`；START ACK=`0x01`，0.297 s 进入 RUNNING；RUNNING 为 `0x11`、flags=`0x3F`、`real_output=false`；RUNNING SET 返回 BUSY=`0x11` 且 generation 不变；STOP ACK=`0x01`，0.172 s 回 IDLE |
| RAM 软件归零与静止识别 | PASS | 最新 boot ID `0xBA4031D7` 上左右归零 ACK 均为 OK；随后 joint flags 均为 `0x3F`、角度均为 `0.000 deg`，system flags=`0x23`/ready；8 s 静止补测 20/20 为 STANDING、flags=`0`、quality=`0x0004` |
| 自动同相周期刺激与估计器状态行为 | PASS（限定范围） | 既有低层 `IMPEDANCE + SINE` 运行 12.2 s；20 个 gait 样本中 8 个 LOCKED/8 个 valid，首个 LOCKED 为 7.281 s；锁定 quality=`0x4000`，符合“同相双侧不一致”预期降权。仅证明自动刺激下的状态行为，不代表正常人体步态或相位准确率 |
| 正常人体左右交替步态与变速 | PASS（功能范围） | 有效试次：正常段 LOCKED/valid=`19/19`、相位距离中位数 `0.4288`、步频 `91.63 spm`；加速段 `12/12`、`0.4750`、`124.47 spm`；锁定样本 quality 均为 `0x0000`，停止后回 STANDING。外部真值未测，不宣称绝对相位准确率 |
| UART 真机 IDLE 诊断 | PASS | 运动停止且不复位后：system IDLE/ready=YES，START/STOP 各 `1/1` 成功；sensor available/valid=`0xF/0xF`、pair skew=`332 us`；assist active=NONE、suggestion invalid、`real_output=false` |
| 100 Hz/WCET/栈/错误计数（运动后 IDLE） | PASS | 195264 cycles：period min/max/mean=`9500/10499/10000 us`，max error=`500 us`，execution max=`2773 us`，stack min=`1484 B`；deadline/skipped/output/algorithm error 均 0；snapshot/gait/assist max=`303/385/220 us` |
| 电机在线、REST 与 TWAI 回归 | PASS | A/B 均为 REST、requested disabled、无 fault/warning，主动反馈约 100 Hz；TWAI 全部错误计数为 0 |
| START 后 STOP/REST 回归 | PASS | STOP 后 system IDLE；A/B REST 且 requested disabled；assist 回到打包字节 `0x01`、flags=`0x09`，active=NONE、suggestion invalid、`real_output=false`，并已恢复测试前 selected=DISABLED |
| 阶段 9 总结论 | COMPLETE / PASS（工程与功能范围） | 阶段 9 验收范围内的软件、协议、实时性、归零、SHADOW 隔离、自动同相负向刺激、正常人体交替步态、变速和停止回归均通过；论文级事件与绝对相位精度属于本阶段明确排除项，保持 NOT MEASURED |

## 9. 最终记录

### 9.1 2026-08-10：软件与未归零真机基础验收

```text
日期：2026-08-10
Git：本轮验收开始时基线 HEAD 为 2ee7e98；阶段 8/9 源码、工具、网页与文档在验收后统一收口提交
ESP-IDF / target：ESP-IDF v5.5 / esp32s3
构建：fullclean/build/size PASS，0 compiler warning，0 error
应用：0.9.0，bin 654096 bytes（0x9FB10），app partition 余 0x604F0（38%）
size total：653985 bytes
SHA-256：FF2EF201FD382C092FB3954F97A1FB5857366BE3EDD6DBB99523F87025DCF576

主机回放：PASS，8/8
阶段 6 BLE 主机自测：PASS，10/10
阶段 9 BLE 工具自测：PASS，9/9
网页：PASS，test 42/42，TypeScript PASS，ESLint PASS

真机：COM5 / ESP32-S3 rev0.2 / boot ID 0xCF6636D9 / capability 0x003F
阶段 6 BLE 真机回归：PASS，15/15
阶段 9 BLE 真机协议：PASS，10/10
  - GET gait/assist 正常
  - SET SHADOW 后打包执行字节读回 0x01（requested=SHADOW，effective=DISABLED）
  - ACTIVE 返回 ACK result=0x10、detail=3，原配置不变
  - 测试结束已恢复 selected execution=DISABLED

UART IDLE（RAM 零位无效）：
  sensor available=0xF，valid=0
  gait DEGRADED，quality=0x0003
  assist transparent / disabled / not armed / real_output=false
说明：未归零输入无效时 DEGRADED 符合设计，不能记为失败。

100 Hz：cycles=392，mean/min/max=10000/9509/10491 us，max error=491 us
完整周期：max execution=725 us，最小剩余栈=1868 B
算法分项 max：snapshot/gait/assist=141/112/87 us
错误计数：algorithm/output/deadline/skipped=0/0/0/0
总线与电机：TWAI 全错误计数 0；A/B 在线、主动反馈约 100 Hz、均为 REST

未执行：RAM 归零、静止站立、起步/锁定/变速/停走、START/RUNNING/STOP。
待执行原因：等待用户明确确认双髋处于中立位；本轮没有把当前机械位置擅自记录为软件零位。
阶段结论：PENDING（不是 FAIL）
```

### 9.2 2026-08-10：中立位归零、自动同相刺激与 BLE 生命周期

本轮开始前收到用户“双髋已处于中立位，可以归零并继续阶段 9 运动验收”的明确确认。设备未重新启动，
boot ID 仍为 `0xCF6636D9`。

```text
RAM 软件归零：PASS
  LEFT ZERO ACK：OK
  RIGHT ZERO ACK：OK
  归零后 LEFT/RIGHT joint flags：0x3F / 0x3F
  归零后 LEFT/RIGHT 角度：0.000 / 0.000 deg
  system flags：0x23，ready=YES

归零后静止观察：PASS
  gait state：STANDING
  gait flags：0
  gait quality：0x0004

运动配置：既有低层 IMPEDANCE + SINE；高层 TRANSPARENT + SHADOW
  IDLE assist 打包执行字节：0x01（requested=SHADOW，effective=DISABLED）
  START ACK：0x01
  START -> RUNNING：0.297 s
  RUNNING assist 打包执行字节：0x11（requested=SHADOW，effective=SHADOW）
  RUNNING assist flags：0x3F
  RUNNING real_output_enabled：false
  RUNNING SET_ASSIST：BUSY=0x11，generation 不变

自动运动持续时间：12.2 s
采样：20 个 gait 状态样本，其中 LOCKED=8、valid=8
首个 LOCKED：START 后 7.281 s
quality OR：0x401C
LOCKED quality：0x4000（左右同相导致双侧不一致，符合预期降权）
LEFT phase 范围：0.004 .. 0.701
RIGHT phase 范围：0.037 .. 0.680
cadence 范围：59.92 .. 60.64 spm
confidence 范围：0.318 .. 0.338
LEFT 关节角范围：-25.50 .. 25.78 deg
RIGHT 关节角范围：-22.46 .. 25.21 deg

STOP ACK：0x01
STOP -> IDLE：0.172 s
STOP 后 assist：打包执行字节 0x01，flags=0x09
STOP 后 active：NONE
STOP 后 suggestion：invalid
STOP 后 real_output_enabled：false
测试结束：已恢复测试前 selected execution=DISABLED
停止 3.2 s 后 gait：STANDING，quality=0x0004

UART（停止后、未复位）：
  system：IDLE，ready=YES
  START/STOP：各 1/1 成功
  A/B：REST，requested disabled，无 fault/warning，主动反馈约 100 Hz
  sensor available/valid mask：0xF / 0xF
  sensor pair skew：332 us
  control cycles：195264
  period min/max/mean：9500 / 10499 / 10000 us
  max period error：500 us
  max execution：2773 us
  control task min free stack：1484 B
  snapshot/gait/assist max：303 / 385 / 220 us
  deadline/skipped/output error：0 / 0 / 0
  algorithm error：0
  TWAI：全部错误计数为 0
  assist：active=NONE，suggestion invalid，real_output_enabled=false

停止点：LEFT=18.221 deg，RIGHT=13.254 deg
后续动作：用户需手动将双髋回到中立位；REST 不等于关节自动回零。
```

本轮可判定为 **归零、静止状态、自动同相周期刺激下的估计器状态行为、BLE 生命周期及 STOP/REST
回归 PASS**。但是，低层 SINE 同相驱动不是正常人体左右交替步态，且本轮没有力平台、足底开关、动作捕捉
或人工 gait event 标注作为外部真值。因此相位准确率仍为“未测”，正常人体交替步态与变速验收仍为
**PENDING**，因此截至该轮，阶段 9 总结论仍为 **PENDING**；该历史结论已由第 9.5 节的有效人体试次更新。

### 9.3 后续人体步态验收待记录项

```text
人体左右交替步态：PASS/FAIL（运动时长、周期数、状态迁移、左右相位关系）
人体变速：PASS/FAIL（速度区段、退锁/重锁时间、质量位、是否出现陈旧锁定）
外部 gait event 真值：有/无（设备、同步方法、事件数、相位误差统计）
停止回归：system / A / B / assist active / suggestion / real_output_enabled
错误增量：algorithm / output / deadline / skipped / TWAI
阶段结论：PASS / COMPLETE 或 PENDING / 待补人体与真值验收
```

### 9.4 2026-08-10：第一次人体交替步态与变速监测（无有效动作试次）

用户确认双髋已经回到 0° 附近并断开其他 BLE 客户端后，验收端通过产品 BLE 建立独占连接。整个试次
只发送状态读取命令，没有发送 START、STOP、归零、控制配置或关节命令。声音提示定义为：静止基线
5 s、正常交替动作 25 s、加快动作 15 s、停止后静止 15 s。

```text
固件 / boot ID / capability：0.9.0 / 0xCF6636D9 / 0x003F
开始前系统：IDLE，flags=0x23
开始前关节：LEFT=0.000 deg，RIGHT=-1.662 deg，flags=0x3F/0x3F
开始前助力：effective=DISABLED，active_valid=false，real_output_enabled=false

总采样：120 个 gait 状态样本，周期 0.5 s
静止基线：10/10 STANDING，valid=0，quality=0x0004
正常速度段：50 个样本；STANDING=48，ACQUIRING=2，LOCKED=0，valid=0
加速段：30 个样本；STANDING=27，ACQUIRING=3，LOCKED=0，valid=0
停止静止段：30 个样本；STANDING=27，ACQUIRING=3，LOCKED=0，valid=0
结束时：STANDING，gait flags=0x00

正常段 quality：0x0004×48，0x0018×2
加速段 quality：0x0004×27，0x0018×3
正常/加速段有效步频、相位与置信度：均无有效值

全程 system：保持 IDLE、flags=0x23
全程 assist：保持被动，real_output_enabled=false
结束关节：LEFT=0.000 deg，RIGHT=-1.662 deg
```

本试次的设备身份、软件零位、静止识别、IDLE 状态和助力隔离均 **PASS**；人体交替动作与变速判据
**未通过**。关节观察点按 2.0 s 间隔抽样，可能与周期动作发生采样混叠，因此“抽样角度范围为 0”不能
单独证明人体完全没有动作；但估计器运行在 100 Hz，绝大多数样本仍报告 LOW_MOTION，仅短暂进入
ACQUIRING，说明固件确实没有接收到足以形成周期的连续髋关节运动。该结果记录为“无有效动作试次”，
不改写为算法 FAIL，也不降低既有自动周期刺激的 PASS 结论。

下一次重测需采用与常见步频不同步的关节采样节奏，并在开始前确认操作者能够听到提示音、双侧电机
能够随髋关节自由回驱。因此截至第一次人体试次，阶段 9 仍为 **PENDING**；该历史结论已由第 9.5 节更新。

### 9.5 2026-08-11：有效人体交替步态、变速与静止补测

#### 9.5.1 电机重新上电后的恢复与重新归零

第一次重测前，A/B 电机曾单独掉电，而 ESP32 没有重启。BLE 连续五次读取均为 joint flags=`0x0F`、
反馈年龄 `65535 ms`，system flags=`0x21`、block=`14 (LEFT_OFFLINE)`，步态估计器为 DEGRADED、
quality=`0x0003`。该状态下没有播放动作提示音，也没有开始采集。

代码审计确认主动上报请求目前只在 `app_main()` 启动时发送；RS01 单独重新上电后，ESP32 不会自动重发
type-24 上报开关。用户保持双髋中立并复位 ESP32 后，新 boot ID 为 `0xBA4031D7`，A/B 主动反馈恢复。
归零前连续五组反馈年龄为 `1..10 ms`，两侧 flags=`0x1B`，system block=`10 (LEFT_ZERO_INVALID)`，
符合“反馈在线但 RAM 零位尚未捕获”的预期状态。

```text
LEFT CAPTURE_ZERO ACK：0x00
RIGHT CAPTURE_ZERO ACK：0x00
归零后 LEFT / RIGHT：0.000 / 0.000 deg
归零后 joint flags：0x3F / 0x3F
归零后 system：IDLE，flags=0x23，block=0
归零后 assist：active_valid=false，effective=DISABLED，real_output_enabled=false
```

“电机单独重启后不自动恢复主动上报”是本轮发现的产品恢复性缺口，建议在阶段 11 增加离线检测后的
限速重配或显式恢复执行器；它不影响本次重启后得到的阶段 9 有效数据，但不能被当前 PASS 结论隐藏。

#### 9.5.2 60 秒人体试次

本轮在 system IDLE、A/B 主动反馈有效、左右 RAM 零位有效且真实助力输出关闭的条件下，只通过 BLE
读取状态。采样间隔采用 `0.47/0.61/0.73/0.52/0.67 s` 循环，并在每个样本交替读取 LEFT/RIGHT，避免
与常见步态周期同步混叠。声音提示依次为静止 5 s、正常交替 25 s、加速交替 15 s、停止静止 15 s。

```text
总 gait 样本：101

原始基线：9 样本
  STANDING=7，ACQUIRING=2，valid=0
  quality：0x0004×7，0x0018×2
  说明：末段准备动作污染，原始 baseline_tail 断言 FAIL；保留该失败并执行定向补测

正常交替段：42 样本
  STANDING=13，ACQUIRING=10，LOCKED=19，valid=19
  LOCKED/valid 样本 quality：0x0000×19
  左右圆周相位距离中位数：0.4288
  步频中位数：91.63 spm
  置信度中位数：0.9801
  LEFT 角度范围：-8.652 .. 20.168 deg（excursion 28.820 deg）
  RIGHT 角度范围：-24.809 .. 6.818 deg（excursion 31.627 deg）

加速交替段：25 样本
  STANDING=8，ACQUIRING=5，LOCKED=12，valid=12
  LOCKED/valid 样本 quality：0x0000×12
  左右圆周相位距离中位数：0.4750
  步频中位数：124.47 spm（较正常段增加 35.84%）
  置信度中位数：0.96085
  LEFT 角度范围：-4.985 .. 18.277 deg（excursion 23.262 deg）
  RIGHT 角度范围：-20.970 .. 19.939 deg（excursion 40.909 deg）

停止段：25 样本
  最终回到 STANDING，末尾 phase/cadence/confidence 均无效
  post-tail 静止断言 PASS

全程回归：
  system 始终 IDLE，flags=0x23，block=0
  joint flags 始终 0x3F，反馈年龄正常
  assist 始终 active_valid=false、effective=DISABLED、real_output_enabled=false
```

稳定 LOCKED 样本没有任何质量异常，也没有出现自动同相刺激中的 `0x4000` 双侧不一致。少量
`0x2010/0x2808` 是变速边界的单侧等待/超时，`0x1818/0x3010` 是停止阶段的相位超时和退锁；它们均
能自行恢复或进入 STANDING，不是阻塞性错误。

#### 9.5.3 静止定向补测与最终结论

为闭合原始基线污染，操作者在两声准备提示后等待 3 s，再执行 8 s 完全静止补测：

```text
样本：20/20 STANDING
gait flags：20/20 为 0x00
quality：20/20 为 0x0004 (LOW_MOTION)
LEFT 静止范围：-2.750 .. -2.693 deg，excursion 0.057 deg
RIGHT 静止范围：4.068 .. 4.068 deg，excursion 0.000 deg
结束 system：IDLE，flags=0x23
结束 assist：real_output_enabled=false
```

原始基线失败没有删除或放宽断言，而是由独立、门槛更明确的静止补测闭合。结合正常交替段、加速段和
停止段，可将人体左右交替步态识别、速度响应、退锁和静止清历史判为 **PASS（功能范围）**。阶段 9
在既定工程与功能范围内判为 **COMPLETE / PASS**。

本轮没有同步力平台、足底开关、动作捕捉或人工 gait event 标注，因此不能由上述相位距离和高置信度
推导出 heel strike/toe off 精度、绝对相位误差或临床有效性；这些指标保持
**NOT MEASURED / OUT OF SCOPE**。
