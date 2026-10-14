# 2026-10-09 CAN现场复核与运行期保护候选

当前无运动通信已在操作者重启后恢复：参数往返与重新请求主动上报均有真机证据；
原先ACTIVE后的BUS_OFF电气根因尚未定位，不能标记整机运动验收通过。
操作者报告模块型号为`tj1050`、
由外部电源供电；本文件对TJA1050的接口说明以型号确为TJA1050为前提。
不能把“模块没问题”的判断替代当前供电、共地、端接和波形证据。

## 当前现场证据

| 北京时间 | 操作者电机供电说明 | 串口只读实测 |
| --- | --- | --- |
| 19:10:37 | 已切断两台电机动力电源 | COM6正常响应，ERROR / BUS_OFF，RX GPIO1单次读数0 |
| 19:16:59 | 操作者已重新打开电机动力电源，CAN模块外部供电 | ERROR / BUS_OFF，RX GPIO1单次读数1 |
| 19:27:10 | 操作者随后确认已重启 | IDLE / RUNNING，START/STOP均0，RAM零位无效；TEC124、bus_error912，双电机反馈count0 |
| 19:28:47 | 保持IDLE，只读0x7005 | A0x10/B0x20均实际参数应答status0/raw0，bus_error仍912，TEC124→123→122 |
| 19:30:11 | 保持IDLE，读取0x7026并重新请求report1 | 间隔寄存器均raw0；A131/B132个有效反馈，100.06/99.96 Hz，年龄2253/736 us，均REST、fault_raw0；bus_error仍912，TEC降至116 |
| 19:31:37起 | 无运动，仅UART读快照 | 四次采样跨度约37.2秒，A count8003→11716、B7208→10920；均约100 Hz/REST，反馈年龄<10 ms；bus_error912、TEC116、REC0、RX missed0均未增加 |

重启前两次读回START请求/成功均为1，STOP请求1/失败1，run_requested与output_allowed均为no，
失败步骤VERIFY_DISABLED。TEC128、REC0、bus_error1687、arbitration_lost147、RX missed0，
两次计数一致；100 Hz任务仍存活、deadline/skipped0、历史输出错误14324。
这些累计错误已经在电机断电之前形成；不能把它们归因于本次主动断电。

原始记录：`output/phase10/20261009_can_diagnosis/20261009_191037_uart.bin/.txt`、
`20261009_191659_uart.bin/.txt`及各自metadata。串口115200、open前设置DTR/RTS=false，
每次finally关闭。元数据区分操作者说明与物理反馈；第二次采样的供电字段已更正为on，
不沿用第一次断电确认。Agent没有新START、使能、运动、归零、复位、恢复API或烧录；
之后操作者自行重启，Agent仅发送各侧0x7005/0x7026读取和`report 1`状态上报开关。
后续查询原始证据另存`*_parameter_probe.bin/.txt/.json`与`*_passive.bin/.txt/.json`。

RXD是逻辑侧输入的单次数字采样，不是CAN_H/CAN_L差分波形；它既不能证明有ACK，
也不能独立证明模块供电或总线被拉低。两次高低变化与操作者电机供电变化同场发生，
尚不能确定因果。19:16主控仍BUS_OFF，说明仅重新给电机上电当时没有恢复通信。
操作者随后重启使CAN控制器回RUNNING；参数查询证实两台节点可达，但反馈count仍0。
仅重新请求状态上报即产生新鲜100 Hz反馈，证实当时主动上报未实际工作，不能把
`Reporting ON (requested)`解释成电机已经上报。启动配置与电机上电顺序是待复现假设。
无运动观察没有新增bus_error；这不覆盖运动负载、线束移动或BLE遥测压力测试。

## 软件核对与已修候选

仓库仍为main / 9225bb3，保留既有修改；本轮未提交/推送。
本机ESP-IDF5.5，现有工程ESP32-S3、16 MB Flash、TX4/RX1，工程激励关闭。
当前故障板仍是此前验证过的0.10.2；本轮没有重启读取芯片ROM信息或用新ELF解释旧固件地址。

原实现运行中`exo_system_process_control_tick`仅返回控制器错误，永久控制循环只记输出错误，
所以双电机反馈中断后仍曾保持RUNNING直到网页STOP。这是故障处置缺口，不等于CAN电气根因。
旧固件没有CAN恢复命令，也没有调用`twai_initiate_recovery`，不能通过反复网页START清除BUS_OFF。

0.10.3候选在已有最大附加力矩功能上增加：

- 唯一100 Hz输出门发送前读取TWAI短快照：状态不可读、非RUNNING、TEC或REC>=128均触发故障。
- 使用本周期已经生成的统一传感快照，检查时间/序号、左右髋有效位与在线状态、同源电机故障位。
  反馈新鲜门槛仍为已有50 ms；没有第二个采样任务，不重新采集一套不同时间的关节数据。
- 任一控制输出失败（含仅一台入队成功）关闭后续周期输出。锁存首个原因与时间、CAN状态/计数、
  左右反馈年龄，通知原有慢速执行器做一次释力/失能/新REST确认。
- 自动故障清理即使得到新REST也保持ERROR。明确的后续STOP并确认新REST后才允许IDLE；
  不自动CAN恢复、START或重新使能。失能无法确认仍报告VERIFY_DISABLED，并保留原始故障原因。
- UART `system_status`新增历史运行故障诊断；`can_status`显示TX/RX/位率，并将GPIO读数标为
  digital high/low，避免把未供电或浮动RXD解释成真实CAN总线状态。
- RX驱动异常返回时限速日志并让出CPU，防止高优先级错误循环空转。此项是保护，不声称本轮发生过该空转。

CAN累计bus_error和arbitration_lost不单独触发故障；正常仲裁竞争也可能增加后者。
正常等待人体产生有效步态不会触发系统故障，策略仍按门禁将附加助力保持0。
关闭周期输出门不是“电机已失能”的物理证明，CAN故障下仍需动力电源切断手段。

## 软件测试证据

| 检查 | 结果 | 边界 |
| --- | --- | --- |
| `tools/phase10_runtime_fault_replay.ps1` | PASS 7/7 | 真实exo_system.c +真实sensor builder；RTOS/控制器/CAN以明确桩替代，验证BUS_OFF/STOPPED/RECOVERING、128错误阈值、50,000/50,001 us新鲜边界、双侧故障、部分发送、一次清理、ERROR锁定、STOP竞争和原始故障保留 |
| `tools/phase9_replay.ps1` | PASS 8/8 | 统一传感/步态/协议回归 |
| `tools/phase10_assist_integration_replay.ps1` | PASS 4/4 | 真实传感→步态→助力→控制器，替代最终CAN；含最大力矩锁存/限幅 |
| `tools/phase10_replay.ps1` | PASS 7/7 | 真实相位曲线及五元合成 |
| 现有工程 `idf.py build` | PASS | ESP-IDF5.5；最终binary/ELF/map、SHA与源码摘要见manifest |
| `git diff --check` | PASS | 无空白错误，原有工作区改动保留 |
| 新保护目标板时序、故障注入、CAN恢复与双电机反馈 | NOT RUN | 本轮未烧录；主机桩不能证明实际失能或双核RTOS竞争时序 |

证据及构建冻结在`output/phase10/20261009_can_diagnosis/`。此前最大力矩候选冻结于
`20261009_assist_torque_limit/`，保留其原始binary/manifest；两者同属未烧录0.10.3候选，
必须按各自SHA区分，不能将旧软件候选的构建摘要当成这次新保护源码的对应ELF。

## 假设与最小实验

| 项目 | 当前证据 | 下一项检查 | 状态 |
| --- | --- | --- | --- |
| MCU/控制任务停摆 | 串口响应、100 Hz存活、deadline0 | 新固件后复测保护开销和周期 | 当前证据不支持 |
| 网页轮询导致旧数据 | 原BLE响应持续到达但源年龄增长 | 继续区分源反馈与网页查询时间 | 已排除为CAN中断主因 |
| 主控BUS_OFF锁定 | 重启前两次直接读回；旧代码无恢复过程 | 后续独立验证受控恢复，不能自动使能 | 操作者重启后回RUNNING；Agent未调用恢复API |
| TJ1050模块本体损坏 | 操作者认为正常；未测电压/波形 | 先核对准确型号与模块电路，不直接换模块 | 未证实 |
| 供电/逻辑电平不匹配 | 外部供电电压未知 | 测模块VCC、RXD高电平及ESP输入侧，核对RXD是否有电平转换 | 未证实 |
| 共地、端接或运动使线束接触改变 | 原故障发生在启动约8秒，双方同时失联 | 断电检查共地/线束，测H-L端接，台架逐变量测试并保存波形 | 未证实 |
| 正常100 Hz即总线超载 | 没有总线利用率或波形证据，RX missed0；1 Mbps与RS01默认手册一致 | 先无运动测100 Hz双反馈，再增加周期运动帧 | 未证实，不能直接降低控制频率掩盖错误 |
| 主动反馈配置与上电顺序 | 重启后Reporting requested为ON但反馈0；重发report1即恢复两侧100 Hz | 台架复现主控先上电、双电机后上电，不使能运动 | 本次反馈缺失/恢复已证实；启动顺序因果待复现 |

## 接口依据与测量顺序

[NXP TJA1050数据手册](https://www.nxp.com/docs/en/data-sheet/TJA1050.pdf)：供电4.75～5.25 V；
TXD输入兼容3.3 V不意味着RXD输出自动限在3.3 V。RXD接口须按模块实际电路/实测判断。
[ESP32-S3数据手册](https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf)
DC Characteristics给出输入高电平范围上限VDD+0.3 V；不能默认可直接承受5 V逻辑。

本地RS01手册PDF第13/14页给出默认1 Mbps与电源+/−、CAN_L、CAN_H引脚；第41页说明私有协议
为1 Mbps扩展帧。相关页文本保存在`rs01_manual_relevant_pages.txt`，完整原件路径为
`D:\GITHUB\motor\product-information\Product_Information\Product Literature\RS01\RS01User Manual260713.pdf`。
接线图仍需在实物连接器方向下核对，不能只凭文本编号插拔。

现有工程使用`driver/twai.h`经典驱动，核对的是本机IDF5.5头文件和实现：
`twai_transmit`成功仅表示入队/启动发送，不能证明ACK；`twai_initiate_recovery`清发送队列，
恢复完成进入STOPPED，`twai_start`清TX/RX队列后才进入RUNNING。
依据为[同版本官方头文件](https://github.com/espressif/esp-idf/blob/v5.5/components/driver/twai/include/driver/twai.h)。
不套用IDF5.5文档中新TWAI node驱动的自动重新连接行为。

1. 当前先保持扶稳，不增加助力或START。补充外部供电实际电压、准确型号、是否有万用表。
2. 如测电阻，需整条CAN总线及收发器、主控均断电；在标准两端各120Ω配置下H-L约60Ω，
   40Ω可能提示三只120Ω并联，120Ω可能提示仅一端，均须结合实际拓扑确认，不能盲目移除端接。
3. 测逻辑电平时保持电机动力断电；若RXD可能直出5V，先全部断电并断开RXD到GPIO1的连接，
   再仅为模块逻辑供电测其RXD，避免把未知5V继续直接接ESP32输入。
4. 保护候选已构建，尚未获得本轮升级烧录的明确授权，且最新操作者说明电机动力已开启。
   烧录/复位之前需重新切断电机动力并确认；升级会丢失RAM零位，不能自动沿用旧标定。
5. 当前0.10.2的参数往返与100 Hz上报已恢复；升级后仍需独立目标板IDLE无使能验收：
   新固件身份/配置、CAN受控恢复、0x10/0x20参数读回与100 Hz主动反馈，保存错误计数与时序。
   本轮启动后曾有912次错误，后续读取未新增；不将累计计数不为0当作正在持续出错。
   恢复API成功只证明控制器状态，不能证明电气问题已修复。
6. 再台架验证运行故障关闭门、两侧新REST确认、断线/复连；通过之后重新穿戴校准，先SHADOW，
   再0%/5%/10%分级验收。当前ACTIVE 10%仍为FAIL，阶段10整体和阶段11均未完成。
