# 2026-10-09 0.10.2 产品标定与当前穿戴目标板

本次在既有工作区增量开发，没有重建工程、reset工作区、机械置零或默认启动运动。
操作者报告人体已穿戴、电机已供电，并明确要求先烧录当前程序，再调试网页零位与左右电机核对。

## 当前版本与新增行为

- 固件与网页首选版本0.10.2；旧0.7.x–0.10.1仍按已声明能力读取兼容功能。
- 新增只读`GET_CALIBRATION_STATUS=0x09`（请求带LEFT/RIGHT侧别），capability bit8。
  请求读取同一个髋关节快照与静态配置，不发送CAN、不创建任务、不改变零位或电机状态。
- 回应opcode0x89，payload16 B、总长20 B，保持BLE v1和默认ATT MTU兼容。
  byte4 side、5 actuator(A=0/B=1)、6 CAN ID、7 direction、8 joint_flags、9 reserved=0，
  byte10–13连续电机角i32 LE mrad、14–17软件零位i32 LE mrad、18–19反馈年龄u16 LE ms。
  无效数值使用INT32_MIN；网页转为度，并显示“无有效数据/尚未标定”，不能冒充0°。
- 出厂面板读回实际固件映射；它是只读核对界面，没有运行期更改安装映射或NVS出厂写入接口。
  人工左右/方向核对必须由现场操作者完成，页面不会自动摆腿辨识。
- 标定面板分别显示电机连续角、中立位电机偏置和人体髋角，使用现有CAPTURE_ZERO保存RAM偏置。
  断连、重启或反馈丢失会撤销本次网页人工核对记录；RAM零位重启失效。
- 人体穿戴入口默认启用；START前重新读取基础配置，仅允许DAMPING+CONSTANT、
  左右q/dq/tau/Kp=0、Kd≤0.1。未核对左右腿/方向或未建立有效零位时按钮保持锁定。
  该门禁是网页操作约束，不宣称替代固件状态机门禁或现场停止措施。

## 构建、烧录与证据

证据目录：`output/phase10/20261009T163003_product_calibration_0102/`。

- 本机ESP-IDF v5.5，目标ESP32-S3 rev0.2，MAC `14:c1:9f:c5:26:80`，Flash16 MB/PSRAM8 MB。
- 隔离`sdkconfig.product`：TX4/RX1、DIO40 MHz，工程激励关闭，PSRAM未使用。
- app665552 B，1 MiB分区剩余37%；SHA256
  `338dd315e1b6cb14152e0d6d41da54a68ca0afc2d8e14a07211016870d27c1f0`。
- 烧录前又读到旧版数字CAN ID控制台；芯片MAC匹配，应用头为83b384a/IDFv5.3.1-dirty，
  与上午完整备份的应用头逐字节一致。这里只证明应用头匹配，不宣称再次比对了整片16 MB。
- esptool烧录bootloader/分区表/app且逐项Hash verified，未整片擦除。
- 网页实际读回0.10.2、capabilities0x017F、Boot ID0x3503C3B7，和UART一致。
  bit7未声明，工程激励入口不可用；不能沿用上午capability0x00FF的工程构建状态。
- 主机直接链接固件纯C协议：5/5 PASS；网页含构建55/55 PASS；TypeScript及ESLint退出码0。
- Python BLE验收工具的离线自检9/9 PASS，已区分0.10.2的标定能力位；本次没有运行它的真机动作。

## 真实网页标定结果

操作者通过本轮问题明确回复“两侧对应正确，现在已静止于中立位”。
随后在真实网页勾选本次左右核对和当前中立位，点击“保存双髋中立位（软件归零）”。
BLE ACK及重新查询均成功；没有用UART归零或Python BLE代替网页按钮。

| 本轮项目 | 证据与结果 |
| --- | --- |
| 映射配置读回 | LEFT→B/0x20/+1，RIGHT→A/0x10/−1 |
| 装配左右/方向核对 | 操作者本轮现场确认；不是软件自动识别证据 |
| 中立位软件零位 | 网页读回LEFT约−1.776°、RIGHT约−22.918°（电机坐标偏置，mrad定点显示） |
| 标定后人体髋角 | 网页两侧约0.000°、RAM零位有效、关节最终有效 |
| 主动反馈 | A/B约100 Hz、新鲜有效、REST、无使能请求，故障/警告0 |
| CAN与100 Hz | 烧录后首次UART CAN错误全0；控制任务mean10000 us，最大执行816 us，deadline/skipped/output/algorithm error均0 |
| 软件归零后的状态 | IDLE、输出关闭、Ready可启动；未START、未机械置零 |

网页当时准备了DAMPING+CONSTANT、ACTIVE_PHASE+PHASE_PROFILE+SHADOW、profile1、左右10%，
五通道10 Hz（mask0x0079）仅用于观察。IDLE中SHADOW尚未arm，有效执行态DISABLED/NOT_ARMED属正常。
后续真人SHADOW需要穿戴者主动动作，不能把本次静止归零或静止遥测当作其通过证据。
真人ACTIVE 0%/5%/10%、遥测重连与七通道20 Hz持续负载仍未完成，阶段10不能整体完成。

## 标定后只读复核与网页断连问题

`product_calibrated_idle_uart.*`再次读回同一Boot ID0x3503C3B7：
IDLE、START requests=0、output_allowed=no；双侧REST且未请求使能，CAN错误全0。
LEFT软件零位−1.791°、RIGHT−22.935°，双髋人体角0°且零位epoch有效。
与网页的−1.776°/−22.918°之差来自BLE的mrad定点量化；不能把电机偏置当作人体髋角。
100 Hz累计约13万周期，deadline/skipped/output/algorithm error仍为0。

本次只读复核发现BLE实际已断开、遥测已停止，但网页曾保留旧的“已连接”与旧快照；
网页刷新报“BLE尚未连接”。网页开发热更新的清理过程可释放GATT却保留界面状态，
这是从代码得出的可能原因，不宣称已定位这次断连的全部底层原因。
新增1 Hz只读GATT会话检查及每笔事务发送前检查；失效后清除快照、人工确认并锁定START。
修改后真实网页显示尚未连接、旧关节/标定读数清除、START禁用，保存在
`web_stale_connection_fixed.txt`。这是断连失效显示的验证，不等于遥测断开/重连E2E已通过。
重新配对只查询当前状态；不得重新归零或自动START。

网页5000条事务日志已导出为`web_calibration_transactions.jsonl`，其中保留本次LEFT/RIGHT
CAPTURE_ZERO及关联ACK/标定读回。静止五通道10 Hz期间UART累计成功Notify16909、
设备丢样本0；这只支持该次IDLE推流的计数观察，不支持运动耐久或七通道20 Hz通过。

## 后续服务发现失败与一次网页重连

操作者截图对应北京时间17:06的失败：网页17:06:20.823记录GATT连接建立，
17:06:21.174记录断连事件，随后getPrimaryService报告GATT Server is disconnected。
因此“已建立GATT，但没有发现Product Service”的通用错误提示不准确，
此证据支持服务发现期间连接中断，不能据此判定固件缺少服务。

17:08通过真实网页“重试已选设备”复用同一ESP-EXO，产品服务、响应Notify、
遥测Notify订阅及设备初始状态查询成功。未新增固件烧录、软件/机械归零或START。
网页再读回0.10.2/capability0x017F/Boot ID0x3503C3B7，IDLE、双髋0°、RAM零位有效。
并行UART只读记录确认同一Boot ID、START requests=0、响应/遥测订阅有效。
串口观察程序已关闭端口。

该次重试恢复支持产品服务实际存在、当前网页连接可用；具体底层断开原因仍未确定，
不能把一次状态查询重连恢复扩展成遥测持续推流断开/重连E2E整体通过。
本轮没有修改网页连接实现，避免在穿戴调试中再次触发开发热更新断连。
通用错误文案仍需在后续网页清理中按断连/服务缺失分别处理。

证据：`ble_service_failure_before_retry.txt`、`ble_service_failure_uart.*`、
`ble_reconnect_observed_uart.*`、`ble_service_retry_transactions.jsonl`、
`ble_service_retry_recovered.txt`及`ble_service_recovered.png`。

`candidate_manifest.json`保存构建/源码身份；`flash.txt`、`preflash_*`保存烧录/芯片证据；
`product_0102_after_flash_uart.*`、`web_zero_readback.txt`、`web_shadow_prepared.txt`及
`web_calibration_zero.png`保存当前目标板与网页证据。工作区未提交或推送GitHub。
