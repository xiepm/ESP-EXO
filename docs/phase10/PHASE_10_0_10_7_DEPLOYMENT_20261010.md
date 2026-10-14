# 0.10.7 部署与网页参数读回记录

状态：`FLASH PASS; WEB ANGLE CONFIG READBACK PASS; SINGLE LATE-POWER-ON FEEDBACK WINDOW PASS; WEB WEAR RAM ZERO PASS; HUMAN ANGLE SHADOW/ACTIVE NOT RUN`。

本记录对应2026-10-10本次操作：断电部署窗口最后UART采样时间16:18:42（UTC+8），随后上电检查见第5节。
用户明确确认“已断电，同意升级0.10.7”；第1–5节只执行烧录、只读诊断和待机参数写入/读回。
之后用户另行确认当前已穿戴且双髋中立静止，第6节执行本次穿戴RAM零位采集。
全程没有START、使能、主动运动命令或RS01机械置零。

## 1. 烧录来源与目标

- 仓库HEAD仍为`9225bb300dde0f722d0f0ad0a30443a30b90c8bc`，main上原有修改全部保留。未提交或推送GitHub。
- 冻结来源：`output/phase10/20261010_angle_threshold_assist/candidate-0.10.7/`。
- 131个冻结源码、构建产物和测试日志的大小与SHA-256均与manifest一致。
- App：672944 bytes；SHA-256：`d809a7282b1506760e3b37827a5e7037a148db2a890503005915d82f0817de5c`。
- COM6：FTDI VID:PID 0403:6001，序列号A5069RR4A。
- esptool 4.12.0实测ESP32-S3 QFN56 revision v0.2，嵌入8 MB PSRAM；构建与烧录使用16 MB Flash配置。
- 冻结配置：TX GPIO4 / RX GPIO1，1 Mbps；工程激励编译开关关闭。
- 从冻结build目录按flash_args写入bootloader 0x0、partition table 0x8000和app 0x10000，三处均显示`Hash of data verified`，退出码0。
- 未执行整片擦除，NVS不在此次写入范围内。

冻结manifest中的`SOFTWARE_PASS_NOT_FLASHED`描述冻结时的状态，保持原样；实际部署结果以本记录为准。

## 2. 断电部署窗口的目标板与网页证据

本节记录烧录后、操作者恢复动力之前的状态；上电后的新反馈结果另见第5节，不混用两个窗口。

网页通过“重试已选设备”重新建立Product Service与Notify，实际读回固件`0.10.7`。
新策略选为“角度阈值助力 · 单向前抬”后，ACTIVE选项和保存按钮可用；没有选择ACTIVE。
本次验证确实使用网页按钮，未以Python BLE脚本代替网页参数E2E。

| 操作 | 主控读回/观测 | 判定 |
| --- | --- | --- |
| 新策略保存：T=10°、M=0.100 N·m、左右0%、SHADOW | ANGLE_THRESHOLD、10.00°、0.100 N·m | PASS，配置验证 |
| 修改为非默认T=12.50°，仍SHADOW/双侧0% | 页面设备保存值12.50° | PASS，阈值写入/读回 |
| 验证后保存T=10°、M=0.100 N·m、双侧0%、DISABLED | 页面10.00°；UART ANGLE_THRESHOLD/DISABLED、gen=4、M=0.100、左右0% | PASS，最终配置 |
| 主控生命周期 | IDLE；run_requested=no；output_allowed=no；START requests=0 | PASS，待机检查 |
| 唯一控制循环 | 100 Hz；deadline/skipped/output errors=0；运动输出0/0 | PASS，仅该采样窗口 |
| 出厂方向重启后读回 | LEFT→B/−1(valid)，RIGHT→A/+1(valid) | PASS，既有NVS记录保留 |
| 穿戴RAM零位 | 双侧INVALID，START block=LEFT_ZERO_INVALID | 符合重启失效约定，未重新校准 |
| 动力断电后的CAN读数 | RUNNING，TEC/REC=0/0，总线错误0，队列0；RX采样为低 | 仅主控状态，不能证明上电通信正常 |
| 电机反馈 | 两侧count=0、INVALID/OFFLINE | 上电后反馈验收PENDING |
| 新策略运动中SHADOW/ACTIVE | 未START、未运动 | NOT RUN |

两个UART烧录后窗口的boot ID均为`0xE166E107`；没有为参数验证再次复位主控。
烧录前旧固件的只读窗口曾出现RX低、REC=134、累计总线错误29566589及离线旧反馈；
这项证据发生在用户已确认电机断电之后，不能据此确定电气根因，也不能因烧录后计数归零就宣布CAN问题彻底解决。

## 3. 原始证据

目录：`output/phase10/20261010_angle_threshold_assist/deployment-0.10.7-20261010/`。

- `flash.log`：芯片识别、写入地址、三处哈希校验。
- `before-flash-serial.log`：旧固件烧录前只读状态。
- `after-flash-serial.log`：新固件启动后只读状态。
- `after-web-write-serial.log`：网页配置验证后IDLE、gen=4、输出0、方向、零位及CAN快照。
- `web-threshold-12_50-shadow-zero.jpg`：非默认阈值网页读回截图。
- `web-motion-final-state.txt`、`web-motion-final.jpg`：最终页面与已保存配置。
- `read_only_serial.py`：仅查询状态，串口open前设置DTR/RTS=False，finally及时关闭。

## 4. 后续验收

操作者随后确认“已上电”，第5节已完成本次只读反馈窗口。
操作者后来确认当前穿戴中立静止姿态，本次RAM零位已通过第6节；SHADOW跨阈值记录尚未执行。
网页配置可用不等于实际助力有效；真人SHADOW/ACTIVE及完整START/STOP网页E2E仍需独立执行。
阶段10整体仍未完成，阶段11未完成。

## 5. 本次后上电无运动检查

操作者确认“已上电”后，两次只读采样之间的统一传感快照时间跨度为91.629772秒。
没有复位主控，也没有按网页“恢复电机反馈”；两次boot ID仍为`0xE166E107`。
因此本次证据支持一轮动力后上电自动恢复，不能代替多轮掉电或长期耐久验收。

| 项目 | 本次真实读数 | 判定 |
| --- | --- | --- |
| LEFT/B新增反馈 | 9159帧；接收源时间跨度91.626586秒；平均99.960 Hz | PASS |
| RIGHT/A新增反馈 | 9160帧；接收源时间跨度91.631748秒；平均99.965 Hz | PASS |
| 末次统一快照反馈年龄 | LEFT 3467 μs；RIGHT 5900 μs；双侧online=YES | PASS |
| 双电机状态读取 | A/B均REST；fault/warning=0；invalid frames=0；enabled_requested=no | PASS，仅本次读数 |
| CAN两个端点 | RUNNING；TEC/REC=0/0；bus errors=0；TX/RX queue=0 | PASS，本次窗口无新增错误 |
| 主控/输出 | IDLE；START requests=0；output_allowed=no；运动输出0/0；助力DISABLED | PASS |
| 网页 | 0.10.7；A/B反馈在线；设置页反馈年龄约1/6 ms | PASS，仅本次页面 |
| 零位与人体角 | 双侧RAM零位INVALID；人体角显示“—”；统一快照valid mask=0 | 预期未校准，不能据此认定CAN无反馈 |
| 新策略运动中SHADOW/ACTIVE | 未执行 | NOT RUN |

追加原始证据为`power-on-serial-1.log`、`power-on-serial-2.log`、`power-on-summary.json`、
`analyze_power_on.py`和`web-power-on-settings.jpg`/`web-power-on-settings-state.txt`。
解析脚本校验同一启动、两侧接收计数/时间增长、约100 Hz、故障位、REST、IDLE和输出0；只解析日志，不连接串口。
`deployment-summary.json`仍是上电前的部署快照；上电后的状态以`power-on-summary.json`和本节为准。
第5节结束时尚未采集零位；后来收到当前中立静止确认并完成第6节，始终未START。

## 6. 本次穿戴软件零位网页E2E

操作者明确回复“已穿戴，双髋中立且静止”后，在网页设置页勾选本次穿戴确认，
点击“校准并写入双髋零位”。页面16:33:44显示“双髋软件零位已写入并读回确认”，
确认勾选随后由页面复位，校准标记为“本次校准已写入”。

16:34:07前后UART独立复核：

| 项目 | 当前证据 |
| --- | --- |
| LEFT软件零位 | 电机连续坐标7.418°，RAM valid，采集源count=35710、timestamp=1211899493 μs、epoch=1(current) |
| RIGHT软件零位 | 电机连续坐标−24.364°，RAM valid，采集源count=35856、timestamp=1213338429 μs、epoch=1(current) |
| 人体髋角 | UART LEFT −0.000°、RIGHT 0.000°；网页双侧0.00° |
| 统一输入 | 双侧joint valid=YES，sensor valid mask=0x0000000F、quality=0、reset_epoch=4 |
| 系统门禁 | ready_to_start=YES，start_block=NONE；当前仍IDLE，START requests=0、output_allowed=no |
| 电机/CAN | A/B REST、故障/警告0；RUNNING、TEC/REC=0/0、bus errors=0 |
| 输出与配置 | 运动输出0/0；ANGLE_THRESHOLD/DISABLED，M=0.100 N·m，双侧0% |
| 网页实时数据 | 双侧“有效”，反馈年龄毫秒级；自动轮询中请求ID、速度、年龄变化；未开启连续遥测 |

本项只证明本次软件零位写入/读回及当前有效显示；不证明人体步态或角度阈值助力通过。
原始证据为`after-wear-zero-serial.log`、`wear-zero-summary.json`、
`web-wear-zero-success.jpg`/`web-wear-zero-success-state.txt`和`web-after-zero-live-state.txt`。
零位仍只保存在RAM，重启或重新穿戴需重做；方向NVS没有重新标定，机械零位没有改写。
