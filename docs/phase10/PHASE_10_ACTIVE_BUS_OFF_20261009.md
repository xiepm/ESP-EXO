# 2026-10-09 ACTIVE 10%启动后的CAN故障

结论：本轮ACTIVE尝试不通过验收。当时CAN为BUS_OFF；网页STOP关闭主控周期输出，
但VERIFY_DISABLED失败，不能确认两台电机已收到失能命令。
不能把过期反馈中的RUNNING、无故障位解释成当前电机状态。
已要求操作者扶稳并切断电机动力电源；在收到实际断电确认前不恢复CAN、不使能、不重新START。

## 真实证据

- 同一0.10.2固件，Boot ID0x3503C3B7。操作者通过网页自行归零、选择ACTIVE 10%并START。
- BLE事务：北京时间18:13:41.046/18:13:42.486发出LEFT/RIGHT CAPTURE_ZERO，
  18:13:57.227读回ACTIVE_PHASE/ACTIVE/profile1/左右10%。
- 18:14:01.746 START，18:14:02.478有效执行态ACTIVE，但真实输出关闭、REQUIRED_GAIT_INVALID。
- 左侧最后新鲜关节响应18:14:09.906，右侧18:14:10.122。
  首次无效响应分别18:14:10.626/18:14:10.746，年龄576/688 ms。
  因此启动后约8秒反馈失效；该时间线不包含TWAI进入BUS_OFF的精确瞬间。
- 启动到STOP之间每侧165次关节响应，仅11次新鲜有效。有效角跨度LEFT23.033°、RIGHT0.057°。
  165次步态响应整体有效均为0，没有可用的置信度读数；不能认定已形成双侧周期步态。
- 日期记录142点，有效步态0点、两侧各7点有效角度，其他角度保持null。
- Agent通过网页于18:16:33.123 STOP，网页报告ERROR/VERIFY_DISABLED。
- 同步UART：START requests=1/success=1，STOP requests=1/failure=1；state ERROR，
  run_requested=no、output_allowed=no；CAN BUS_OFF，TX error128、Bus error1687、Arbitration lost147、RX missed0。
  电机反馈已过期约156秒，最后模式为RUNNING且使能请求仍为YES，这些不是当前电机失能证据。
- 100 Hz任务存活，deadline/skipped均0；输出错误14324。
  Assist suggestion/applied累计均0，最终增量0，主控助力执行DISABLED。
  BLE连接与响应正常，Notify失败计数0；独立遥测流为STOPPED。

Agent没有再次归零、提高助力、START、烧录或进行CAN恢复。串口只读程序关闭COM6。
原始证据目录：`output/phase10/20261009_active_feedback_debug/`，包括
`active_transactions.jsonl`、`failed_active_gait_record.json`、`initial_uart.txt/bin`、
`before-stop-dom.txt`、`after-stop-dom.txt`、`timeline.json`与离线分析脚本。

## 当前助力策略

底层为DAMPING+CONSTANT，q/dq/torque_ff/Kp为0，Kd=0.1。
高层ACTIVE_PHASE+PHASE_PROFILE，profile1=ENGINEERING_TEST_V1：16点归一化双向周期LUT，
左右分别使用本侧髋屈曲峰代理相位插值。100%曲线峰值0.5 N·m，10%峰值0.05 N·m。
该曲线仍是工程验收曲线，没有通过真人时机与效果验证，不是论文曲线。

需要双髋新鲜反馈、有效左右相位、置信度至少40%，本次activation内左右各一个新屈曲峰。
通过门禁后1秒五次渐入，每10 ms力矩变化限幅0.01 N·m，仅叠加torque_ff；
数据/相位/epoch失效时附加助力清零并重新等待。
步态基础门槛还包括6°最小运动跨度、0.5～3秒周期及双侧一致性。
本轮首先没有双侧有效步态，随后CAN反馈整体丢失，最终没有实际附加助力。

## 根因边界与待修项

| 假设/问题 | 支持或反证 | 下一项检查 | 状态 |
| --- | --- | --- | --- |
| 网页没有查询设备 | BLE响应持续返回，反馈年龄不断增长 | 保留查询时序证据 | 排除为反馈中断的主因 |
| CPU/100 Hz任务停摆 | 任务存活、周期10000 us、deadline/skipped0 | 不用网页刷新率替代控制任务计数 | 当前证据不支持 |
| CAN链路错误使控制器BUS_OFF | TWAI直接读回BUS_OFF与1687总线错误，两侧同一时段失去反馈 | 断电后核对CAN接线、端接、接地与电源，必要时捕获总线波形 | 直接原因已证实；电气底层原因未定位 |
| 符合正常人体助力条件 | 双侧步态全程无效，右髋有效跨度很小，实际增量累计0 | CAN恢复后先做观察测试，禁止用提高等级绕过门禁 | 当前不满足 |
| 固件运行期故障处置不足 | output错误累计14324，失联期间仍RUNNING；exo_system_process_control_tick只回传错误，循环只计数 | 增加运行期总线/双髋健康与发送错误触发撤销输出、异步清理；失败仍保留ERROR，禁止自动再使能 | 已定位软件缺口，尚未修复/烧录 |
| 网页助力门禁显示滞后 | pollOnce不周期GET_ASSIST_CONFIG，失败STOP提前结束也未刷新assist | 对动态执行态/门禁原因低频读回，区分保存的配置与当前真实输出 | 已定位显示缺口，尚未修复 |

本次记录不能标记真人ACTIVE 10%通过；0%/5%分级验收与真人SHADOW仍没有新增通过证据。
阶段10整体未完成，阶段11未完成。没有提交或推送GitHub。

后续0.10.3软件候选已增加网页动态助力门禁读回和STOP失败后的读回，但尚未做目标板E2E。
运行期CAN/发送失败自动触发状态机清理的缺口仍未修复；本轮再次只读查询仍为ERROR/BUS_OFF。
校准锁定与最大附加助力配置详见 [软件证据](PHASE_10_ASSIST_TORQUE_LIMIT_20261009.md)。

后续CAN排查已在未烧录0.10.3候选补上运行期故障门与一次清理，回放7/7及构建通过。
19:10断电、19:16操作者再上电时只读仍ERROR/BUS_OFF；电气根因及新保护HIL未完成。
上文“尚未修复”对应较早快照，最新软件与当前硬件边界见 [CAN复核](PHASE_10_CAN_DIAGNOSIS_20261009.md)。
操作者后续重启主控，Agent仅读参数并重新请求主动反馈；0.10.2在IDLE恢复两侧100 Hz/REST反馈。
这更新当前通信状态，不改变上述ACTIVE失败和电气根因未定位结论。
