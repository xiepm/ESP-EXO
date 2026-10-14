# ESP-EXO

当前仓库源码版本：**0.10.13**，网页入口：**http://localhost:4173/exo**。
对应固件使用ESP-IDF 5.5，ESP32-S3-N16R8、CAN TX4/RX1、1 Mbit/s，工程激励默认关闭。

- [网页操作说明（连接、设置、运动、实时数据、数据记录）](docs/web/WEB_OPERATION_GUIDE.md)
- [网页安装与测试](web-bluetooth-console/README.md)
- [2026-10-14 GitHub上传与验证范围](docs/phase10/GITHUB_BACKUP_20261014.md)
- [阶段10验收记录](docs/phase10/PHASE_10_ACCEPTANCE.md)

**当前未解决问题：** 2026-10-13目标板记录到运动中CAN_ERROR_PASSIVE停机，故障瞬间
TEC/REC=0/133，输出关闭后反馈仍持续。物理根因尚未证实，见
[助力中断诊断](docs/phase10/PHASE_10_ASSIST_INTERRUPTION_20261013.md)。
阶段0–9已完成；阶段10软件及历史工程SHADOW有通过证据，但真人助力整体未通过，阶段11未完成。

下面的带日期记录描述各次操作时刻；Boot ID、待机/反馈和烧录成功不能当作当前现场状态。

2026-10-13：0.10.13已在本轮动力断开确认后烧录，233文件复核/三处哈希/BLE0.10.13/策略version4及15 N·m/3000 ms参数读回PASS；最终恢复关闭/双侧0%/默认0.500 N·m/600 ms。Boot0x215B41F2、IDLE、运动输出0。事件脉冲速率20 N·m/s，3秒15峰值、短脉冲限幅与峰值STOP软件回放通过；网页按INFO区分旧10与新20。真实C17项/原集成4项、O2 ASan/UBSan、故障8项、网页123项与IDF构建PASS。真人及上电反馈未验，见[速率证据](docs/phase10/PHASE_10_RISE_20_20261013.md)和[本次部署](docs/phase10/PHASE_10_0_10_13_DEPLOYMENT_20261013.md)。

2026-10-13：0.10.12已在用户本轮断电确认后烧录，三处哈希、BLE INFO/15上限与3000 ms参数写入读回PASS；已恢复关闭/双侧0%/默认0.500 N·m/600 ms。Boot0x5163F213、IDLE、运动输出0。事件脉冲FIXED可设至3000 ms、申请/START上限15.000 N·m；保留10 N·m/s使3秒实际峰值≤9.55。包含双髋成对零位、INFO版本与15上限换算修复。网页121项及真实C回放通过，见[参数证据](docs/phase10/PHASE_10_EXTENDED_PULSE_20261013.md)及[本次部署](docs/phase10/PHASE_10_0_10_12_DEPLOYMENT_20261013.md)。电机上电反馈与运动验收仍待做。

2026-10-12：0.10.11为已构建、未烧录的双髋成对零位修复候选。选择“重新校准”将当前左右位置一次采集并同时写为RAM中立位，网页读回两侧均接近0°才确认成功；“沿用上次零位”仍保留上次偏置。真实C回放及网页115项测试通过，实机校准待验。见[修复与验收边界](docs/phase10/PHASE_10_PAIR_ZERO_CAPTURE_20261012.md)。

ESP-EXO 是基于 ESP32-S3 的双侧髋关节外骨骼控制器工程，使用 ESP-IDF 开发。

2026-10-12上位机实时页已改为示波器，默认自动10 Hz六通道采集，提供2–30秒时间窗、量程、左右开关、同步游标和暂停查看。网页106项测试、构建、TS/ESLint及实际BLE采集交互通过；138.8秒原始数据段为10.0 Hz/无序号缺口。当前记录为IDLE/站立数据，未验证行走有效相位或真实助力波形；板上仍0.10.10，未另行烧录或START。见[实时示波器证据](docs/phase10/PHASE_10_REALTIME_SCOPE_20261012.md)。

2026-10-12已部署`0.10.10`：策略4为本侧新摆腿事件触发的一次sin²脉冲，左右独立，支持FIXED/PERIOD_SCALED，A/Ta在脉冲内冻结，保留5 N·m上限和10 N·m/s约束。软件测试与构建通过；224个冻结文件复核、三处烧录哈希校验通过，网页实际读回0.10.10/0xFF7F和新脉冲参数。新Boot ID=0x2BD55D97、IDLE/门关闭/START0/运动输出0；出厂方向和上次零位候选保留，当前RAM零位未启用，助力默认关闭/双侧0%。随后上电无需主控复位/手动恢复，约33.46秒采样跨度内左右新反馈99.95/99.95 Hz、REST/故障0、CAN错误0，无运动恢复一轮通过。新策略参数写入E2E及SHADOW/ACTIVE HIL未执行，阶段10整体未完成。见[事件脉冲软件证据](docs/phase10/PHASE_10_SWING_EVENT_PULSE_20261012.md)及[本次部署](docs/phase10/PHASE_10_0_10_10_DEPLOYMENT_20261012.md)。

2026-10-10最新`0.10.9`：设置页可选择“沿用上次零位 / 重新校准”，主控成对保存候选，重启不自动启用；确认本次中立静止和反馈门禁后才恢复RAM。零位主机回放10/10、原方向11/11、角度10/10、网页86/86及IDF 5.5构建通过。用户确认当前动力断电后，197个冻结文件复核及烧录三处哈希校验通过；启动IDLE/START请求0/运动输出0，方向NVS保留、RAM零位无效。网页INFO及候选状态待重连核对，断电沿用真机HIL未执行。见[可选沿用零位证据](docs/phase10/PHASE_10_SAVED_WEAR_ZERO_20261010.md)和[0.10.9部署](docs/phase10/PHASE_10_0_10_9_DEPLOYMENT_20261010.md)。

2026-10-10最新`0.10.8`：角度阈值助力支持5.000 N·m峰值和10 N·m/s最大上升速率，按实际周期收紧增量，最终出口受START锁存峰值限制。相位工程曲线仍0.500 N·m；默认双侧0%/DISABLED。真实C角度回放10/10、原集成4/4、网页81/81及IDF 5.5构建通过。用户确认当前动力断电后，冻结版本烧录与三处哈希校验通过，主控IDLE/START请求0/运动输出0；出厂方向保留，RAM零位清除。网页INFO确认0.10.8/0x3F7F，网页按钮写入并读回ANGLE_THRESHOLD、5.000 N·m、10.00°、双侧0%/DISABLED，UART独立确认；本版本动力上电反馈恢复及真机运动/人体助力未执行。见[5 N·m软件证据](docs/phase10/PHASE_10_5NM_ASSIST_20261010.md)和[0.10.8部署](docs/phase10/PHASE_10_0_10_8_DEPLOYMENT_20261010.md)。

此前2026-10-10部署`0.10.7`：用户确认动力断电并授权后，从冻结产物烧录，三处哈希校验通过，网页实际读回0.10.7。运动页已验证角度阈值12.50°及0.100 N·m写入/读回，最终保存10°、0.100 N·m、双侧0%、DISABLED；既有出厂方向保留。随后上电无需复位或手动恢复，同一boot ID下91.63秒采样跨度内双侧新增反馈约99.96/99.97 Hz、CAN错误0、REST/故障0。本次无运动恢复一轮通过；操作者确认当前穿戴中立静止后，双髋RAM零位经网页写入/读回通过，人体角约0°，输入有效且START门禁就绪，仍IDLE/运动输出0。新策略运动中SHADOW/ACTIVE未执行。见[0.10.7部署与网页验证](docs/phase10/PHASE_10_0_10_7_DEPLOYMENT_20261010.md)和[策略软件证据](docs/phase10/PHASE_10_ANGLE_THRESHOLD_ASSIST_20261010.md)。

此前0.10.6部署由网页读回版本，已从冻结产物烧录并通过写入哈希校验。其功能继续包含在0.10.7中：待机有界反馈恢复、网页CAN诊断、最大附加助力力矩配置、运行期CAN/双髋故障停机保护，
以及分开的“出厂方向配置”和“穿戴后零位校准”。前抬屈髋为正；出厂方向保存到NVS，
断电保留；本次穿戴零位只保存在RAM，每次穿戴重新采集。
0.10.5无运动自动/网页手动恢复通过；动力后上电曾进入错误被动（TEC=144），经EN复位恢复双侧100 Hz REST反馈，运动输出0。
14:27～14:28最新0.10.6后上电检查：boot ID仍0x90E80400，未重启主控，双侧新增REST反馈约100 Hz；97.65秒采样跨度内CAN错误0、IDLE/运动输出0。本次后上电恢复一轮通过，多轮和长时耐久仍待验。
0.10.6针对RX低/待发队列积压增加发送前等待，真实C回放10/10、驱动帧6/6、协议3/3、方向11/11、运行保护8/8、网页74/74及ESP-IDF 5.5构建通过；网页已支持0.10.6及0.10.7候选；该次部署检查时左右出厂方向和穿戴零位尚未标定，不能把当时记录当作当前标定状态。
网页曾出现一次3秒写入超时，新增有限握手超时与日志渲染限额，持续连接仍需重验。
最新证据见 [0.10.6部署与上电验证](docs/phase10/PHASE_10_0_10_6_DEPLOYMENT_20261010.md)；0.10.5历史操作见 [先前烧录后通信验收](docs/phase10/PHASE_10_POST_FLASH_HIL_20261010.md)。
2026-10-10 12:44烧录前重新请求上报后恢复当时0.10.2双侧约100 Hz REST反馈，12:59复核仍在线且CAN收发错误计数为0；
上报中断和巨额历史总线错误的电气根因尚未证实。见 [本次反馈复发与候选证据](docs/phase10/PHASE_10_FEEDBACK_RECURRENCE_20261010.md)。
历史目标板 `0.10.2` 于2026-10-09 ACTIVE 10%尝试发生CAN BUS_OFF，STOP失能确认失败，
主控进入ERROR；该尝试不通过。零位校准仍需IDLE和有效双髋反馈，不能解除故障门禁。
详见 `docs/phase10/PHASE_10_ASSIST_TORQUE_LIMIT_20261009.md`。
CAN现场只读复核及保护候选见 `docs/phase10/PHASE_10_CAN_DIAGNOSIS_20261009.md`；
当时仅重新给电机上电仍BUS_OFF；操作者随后重启并重新请求上报后，IDLE参数往返和100 Hz
双反馈恢复。原ACTIVE后的电气根因待测，保护已随0.10.5烧录，但运行期故障清理HIL仍未完成，不把无运动通信恢复称为运动验收通过。

2026-10-10重新接入COM6后，IDLE只读/反馈请求复核确认双电机REST、约100 Hz反馈且CAN错误计数为0，
详见 `docs/phase10/PHASE_10_FEEDBACK_RESTORE_20261010.md`，仅代表该次采样时的通信状态。
0.10.4首次升级没有出厂方向记录时不会套用历史正负号，方向与穿戴零位须实际标定后才具备启动条件。
网页先记录指定腿的静止原始角，操作者手动向前抬该腿约5～15°并停稳，再保存方向；另一腿保持原位。
左右分别完成后站稳回中立位，仅采集RAM零位。流程和软件证据见
`docs/phase10/PHASE_10_DIRECTION_TEACHING_20261010.md`。0.10.5已包含此代码，但本轮尚未通过网页执行方向NVS标定；未START。

阶段9工程功能真机验收已完成；阶段10相位曲线、唯一
命令合成、BLE连续遥测和网页显示已完成软件集成，隔离的主动工程轨迹也已通过真机
SHADOW验收。真人主动运动SHADOW及ACTIVE 0%/5%仍无新增通过证据，10%失败后需要重验。
阶段10整体未完成，阶段11未完成。主要包括：

2026-10-09新增网页出厂映射只读核对、连续电机角/中立位偏置显示和穿戴RAM软件归零。
当前穿戴目标板使用TX4/RX1、16 MB Flash，工程激励开关关闭；零位采集不会使能电机，
重启后需要重新标定。详见 `docs/phase10/PHASE_10_PRODUCT_CALIBRATION_20261009.md`。

- ESP32-S3；
- 1 Mbps TWAI/CAN 总线；
- 两台 RS01 执行器，当前 CAN ID 为 `0x10` 和 `0x20`；
- 两台执行器独立反馈快照、约 100 Hz 主动上报及 100 ms 在线超时；
- 左/右髋关节对象、显式 A/B 映射、安装方向换算和 RAM 软件零位；
- 一个上电只创建一次、静态分配并永久运行的 100 Hz 控制任务；
- LEFT/RIGHT 同周期读取、零等待实时运动发送和按需时序诊断；
- 六态整机状态机、派生启动就绪条件及异步 START/STOP 执行器；
- 唯一周期运动输出门，以及调试维护命令与启停序列的互斥；
- 产品 BLE v1 二进制协议，以及 UART-only 工程调试台；
- 独立的纯 C 正弦参考生成器，以及 LEFT/RIGHT 关节语义的 100 Hz 阻抗控制示例；
- 统一的 LEFT/RIGHT 五元关节命令、六种基础模式和通用控制器；
- LEFT/RIGHT五元目标到A/B执行器坐标的集中转换，统一使用已保存的出厂方向；
- BLE v1 的 IDLE 静态控制配置扩展和网页模式/五元参数读写，不使用 BLE 100 Hz 命令流。
- 人体关节、参考轨迹、UART 和手机显示统一使用度，RS01/BLE v1 线格式在边界显式换算；
- 每周期统一传感快照，显式记录来源序号/时间、新鲜度、左右 skew、能力/有效 mask 和 reset epoch；
- 可回放的 HIP_EVENT_TBE 髋角代理步态估计，提供连续相位、周期、步频、置信度和质量位；
- 面向 ESP32 的编译期静态助力策略 ops、固定256-byte最大工作区和传感/步态依赖声明；
- 编译进 Flash 的版本化相位 LUT，左右各自消费本侧相位；随仓库曲线仅为0/低幅
  工程验收曲线，不冒充论文复现数据；
- DISABLED/SHADOW/ACTIVE 分层执行，ACTIVE 仅允许 `DAMPING + CONSTANT`，并只向
  基础五元命令叠加有界 `torque_ff`；
- activation epoch、传感序号、步态序号和同周期时间戳的完整校验，以及单一最终
  左右命令发送出口；
- 固定32样本环形缓冲、5/10/20 Hz分频、独立低优先级 BLE Telemetry Notify、
  设备/网页丢包统计和 CSV/JSONL 导出；
- BLE/网页与 UART 可读取步态、助力和遥测状态，手机仍不参与 100 Hz 实时控制。

> 当前代码仍处于分阶段重构期，不代表最终产品架构。`PHASE_PROFILE` 已实现，但
> `AAN_RESIDUAL` 仍明确为未实现；`ENGINEERING_TEST_V1` 不是文献助力曲线。阶段10
> 工程SHADOW已通过；真人SHADOW、ACTIVE 0%和低幅人体运动验收完成前，不能称为
> 人体助力整机验收完成。历史左右映射曾通过两侧独立运动采集确定；当前装配仍需
> 现场核对，并保存本次电机安装方向。固件不会依据CAN ID猜测人体侧别。

阶段 0 的软件基线、构建证据和硬件验收边界见
[阶段 0 基线](docs/baseline/PHASE_0_BASELINE.md)。连接开发板后的逐项验证见
[硬件冒烟测试](docs/baseline/SMOKE_TEST.md)。

阶段 0 已于 2026-08-06 完成，正式基线标签为 `baseline-esp-can-v0`。

阶段 1 已于 2026-08-06 完成，正式标签为 `phase1-common-types-config`。设计边界见
[阶段 1 设计](docs/phase1/PHASE_1_DESIGN.md)，构建、烧录和实机结果见
[阶段 1 验收](docs/phase1/PHASE_1_ACCEPTANCE.md)。

阶段 2 已于 2026-08-06 完成，正式标签为 `phase2-rs01-driver-feedback`。协议和
对象边界见 [阶段 2 设计](docs/phase2/PHASE_2_DESIGN.md)，构建、主动反馈、BLE
及双电机运动结果见 [阶段 2 验收](docs/phase2/PHASE_2_ACCEPTANCE.md)。

阶段 3 已于 2026-08-06 完成，正式标签为 `phase3-hip-joint-software-zero`。对象、
坐标和软件零位边界见 [阶段 3 设计](docs/phase3/PHASE_3_DESIGN.md)，构建、标定、
BLE 及运动回归结果见 [阶段 3 验收](docs/phase3/PHASE_3_ACCEPTANCE.md)。

阶段 4 已于 2026-08-06 完成，正式标签为 `phase4-single-100hz-control-task`。唯一
周期源、实时发送和任务边界见 [阶段 4 设计](docs/phase4/PHASE_4_DESIGN.md)，静态
审计、5 分钟空闲、BLE 重连及 3x10 秒运动结果见
[阶段 4 验收](docs/phase4/PHASE_4_ACCEPTANCE.md)。

阶段 5 已于 2026-08-07 完成，正式标签为 `phase5-system-state-start-stop`。状态模型、
请求语义和执行序列见 [阶段 5 设计](docs/phase5/PHASE_5_DESIGN.md)；静态、构建、
无运动门控、BLE 归零、状态竞争及 3 轮运动证据见
[阶段 5 验收](docs/phase5/PHASE_5_ACCEPTANCE.md)。

阶段 6 已于 2026-08-07 完成，正式标签为 `phase6-product-ble-protocol`。协议、任务边界和
手机流程见 [阶段 6 设计](docs/phase6/PHASE_6_DESIGN.md) 与
[BLE 协议 v1](docs/phase6/BLE_PROTOCOL_V1.md)；构建、烧录、畸形帧、归零、幂等、断连语义
及 START/STOP 运动结果见 [阶段 6 验收](docs/phase6/PHASE_6_ACCEPTANCE.md)。

阶段 7 已于 2026-08-07 完成，正式标签为 `phase7-sine-impedance-migration`。纯参考模块、
LEFT/RIGHT 坐标边界和等价迁移见 [阶段 7 设计](docs/phase7/PHASE_7_DESIGN.md)；C 数值回放、
干净构建、BLE 归零、三轮运动、0.5 Hz 同相关节反馈、BLE 重连及 100 Hz/TWAI 结果见
[阶段 7 验收](docs/phase7/PHASE_7_ACCEPTANCE.md)。

2026-08-10 在不改变阶段 7 物理波形的前提下完成阶段 8 软件迁移，当前固件版本为
`0.8.0`：

- 人体髋关节采样、RAM 软件零位、正弦幅值和控制中心使用 `deg`、`deg/s`；
- UART 角度类命令参数和状态输出使用 `deg`、`deg/s`、`deg/s²`；
- RS01 驱动继续使用手册规定的 `rad`、`rad/s`、`rad/s²`；
- 已发布的 BLE v1 载荷继续使用 `mrad`、`mrad/s`，避免破坏旧客户端；验收客户端和
  手机界面将其换算为 `deg`、`deg/s` 后显示；
- 新增 `HOLD/POSITION/VELOCITY/TORQUE/DAMPING/IMPEDANCE` 六种产品模式，底层
  仍统一使用 RS01 Motion 五元帧；
- 模式和五元参数只允许在 IDLE 设置，START 时锁存，RUNNING 中拒绝修改；
- `IMPEDANCE + SINE` 继续保持阶段 7 的数值行为，但当前整机上电默认已改为
  `DAMPING + CONSTANT`、左右 `q/dq/tau/kp=0, kd=0.1`，不会默认生成位置轨迹。

阶段 8 的控制合同见 [阶段 8 设计](docs/phase8/PHASE_8_DESIGN.md)，软件和真机结果见
[阶段 8 验收](docs/phase8/PHASE_8_ACCEPTANCE.md)。

阶段 9 固件版本为 `0.9.0`。架构、文献算法映射和验收边界分别见
[阶段 9 设计](docs/phase9/PHASE_9_DESIGN.md)、
[文献映射](docs/phase9/PHASE_9_LITERATURE_MAP.md) 与
[阶段 9 验收](docs/phase9/PHASE_9_ACCEPTANCE.md)。主机真实 C 回放入口为
`tools/phase9_replay.ps1`；阶段 9 BLE 默认只读/可恢复配置验收入口为
`tools/phase9_ble_acceptance.py`。阶段 9 已于 2026-08-11 完成正常人体左右交替步态、变速、停止退锁和
静止补测，工程功能结论为 COMPLETE / PASS；没有外部 gait event 真值，因此不宣称绝对相位精度。

阶段10的 `0.10.0` 建立了相位曲线/合成/遥测基线；历史 `0.10.1` 又加入了受编译开关隔离的
交替步态工程激励，专门用于空载/台架 SHADOW 链路验证。产品 `ACTIVE`不下发位置步态轨迹：每次
START 之后必须先由人体带动左右髋，固件在本轮分别观测到两侧新的髋屈曲峰、且相位和置信度达标后，
才从 `0 N·m` 开始渐入助力。相位曲线/合成/遥测架构、曲线来源、BLE字节协议和
验收边界分别见 [阶段10设计](docs/phase10/PHASE_10_DESIGN.md)、
[曲线来源记录](docs/phase10/PHASE_10_PROFILE_PROVENANCE.md)、
[遥测协议](docs/phase10/PHASE_10_PROTOCOL.md) 与
[阶段10验收](docs/phase10/PHASE_10_ACCEPTANCE.md)。主机真实 C 曲线/合成回放入口为
`tools/phase10_replay.ps1`；不发外骨骼 START/STOP 的 BLE 遥测验收入口为
`tools/phase10_ble_acceptance.py`。

## 目录

```text
ESP-EXO/
├── main/                 # ESP-IDF 主组件及当前固件源码
├── docs/baseline/        # 阶段 0 合同、报告和硬件证据
├── docs/phase1/          # 阶段 1 公共类型、配置设计和验收记录
├── docs/phase2/          # 阶段 2 RS01 驱动、主动反馈设计和验收记录
├── docs/phase3/          # 阶段 3 髋关节对象、软件零位设计和验收记录
├── docs/phase4/          # 阶段 4 唯一 100 Hz 控制任务设计和验收记录
├── docs/phase5/          # 阶段 5 状态机、启停执行器设计和验收记录
├── docs/phase6/          # 阶段 6 产品 BLE 协议、设计和验收记录
├── docs/phase7/          # 阶段 7 正弦参考、关节阻抗迁移和验收记录
├── docs/phase8/          # 阶段 8 五元命令、基础模式设计和验收记录
├── docs/phase9/          # 阶段 9 传感、步态、SHADOW 助力设计和验收记录
├── docs/phase10/         # 阶段10相位曲线、遥测协议、来源和验收记录
├── tools/                # 构建、分阶段主机回放和 BLE 验收脚本
├── CMakeLists.txt        # 工程构建入口
├── sdkconfig             # 当前 ESP32-S3 工程配置
└── CONSOLE_COMMANDS.md   # UART-only 工程调试命令说明
```

## 当前硬件配置

| 项目 | 当前值 |
|---|---:|
| 主控板 | ESP32-S3-N16R8（2026-10-09 操作者确认） |
| Flash / PSRAM | 16 MB Flash；8 MB PSRAM 硬件，当前固件不启用 PSRAM |
| TWAI 波特率 | 1 Mbps |
| TWAI TX | GPIO 4（2026-10-09 操作者确认当前接线） |
| TWAI RX | GPIO 1（2026-10-09 操作者确认当前接线） |
| 电机 A | CAN ID `0x10` |
| 电机 B | CAN ID `0x20` |
| 左髋 | 电机 B，方向 `+1`（屈曲为正） |
| 右髋 | 电机 A，方向 `-1`（屈曲为正） |

2026-10-09 已按操作者确认将当前配置改为 RX GPIO1、16 MB Flash，并关闭工程激励。
TX GPIO4 也已由操作者确认；左右安装关系来自 2026-08-06 历史标定，
尚未复核当前装配。RAM 软件零位需每次启动后在中立姿态显式建立。
配置和编译成功不代表当前板已烧录或已通过硬件验收，接续记录见
[2026-10-09 核对记录](docs/phase10/PHASE_10_RECONNECT_20261009.md)。

## 构建

当前工程锁定 ESP-IDF v5.5（内部版本 5.5.0）。在 PowerShell 中激活环境后构建：

```powershell
. .\tools\activate_esp_idf.ps1
idf.py build
```

激活脚本会优先使用当前 `IDF_TOOLS_PATH`，否则自动寻找 Windows EIM 默认安装目录，
并检查 `idf.py`、`IDF_PATH` 和 Python 虚拟环境是否真实可用。脚本必须使用点调用，
这样环境变量和 `idf.py` 命令才会保留在当前 PowerShell 会话中。

本机采用 EIM 的版本化目录布局，正确入口是 EIM 生成的
`C:\Espressif\tools\Microsoft.v5.5.PowerShell_profile.ps1`（仓库脚本会自动调用它）。
不要直接运行 ESP-IDF 源码目录中的 `export.ps1`：该脚本会按传统
`%USERPROFILE%\.espressif` 布局寻找 Python 和工具链，从而产生“环境缺失”的假象。

执行阶段 0 的干净构建、尺寸检查和产物检查：

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\verify_phase0.ps1
```

烧录并打开串口监视器：

```powershell
idf.py -p COMx flash monitor
```

当前控制台命令见 [CONSOLE_COMMANDS.md](CONSOLE_COMMANDS.md)。

## 重构顺序

0. 固化现有基线（已完成：`baseline-esp-can-v0`）；
1. 建立公共类型和配置（已完成）；
2. 重构 RS01 驱动及主动反馈（已完成）；
3. 建立髋关节对象和软件零位（已完成）；
4. 建立唯一 100 Hz 控制任务（已完成：`phase4-single-100hz-control-task`）；
5. 建立状态机及 START/STOP 执行器（已完成：`phase5-system-state-start-stop`）；
6. 建立产品 BLE 协议（已完成：`phase6-product-ble-protocol`）；
7. 迁移正弦参考和阻抗控制（已完成：`phase7-sine-impedance-migration`）；
8. 加入五元命令和基础模式（已完成）；
9. 加入传感快照和步态估计（已完成：工程与功能范围 COMPLETE / PASS）；
10. 加入相位曲线和遥测（软件集成完成，真机分级验收中）；
11. 完成产品清理和整体回归。
