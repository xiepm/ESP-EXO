# 2026-10-09 接续：当前板卡升级与真实网页验收

本记录只描述 2026-10-09 本轮证据。2026 年 8 月工程运动验收保留在
`PHASE_10_ACCEPTANCE.md`，不能代表本轮连接硬件的状态。

## 授权和现场条件

操作者明确授权：“请继续，配合网页上位机，驱动外骨骼，给你授权”。
本轮按该授权准备 Flash 备份、现有工程升级、网页 BLE 联接及低幅工程 SHADOW 验证。
软件零位是把当前反馈锁存在 ESP32 RAM；不执行 RS01 机械置零。
在操作者确认本轮空载/台架、中立位、运动空间及能立即断电前，不捕获软件零位、不 START。

## 当前身份与接线

- 操作者确认模块为 ESP32-S3-N16R8，CAN TXD 接 GPIO4、RXD 接 GPIO1。
- 本轮 esptool `flash_id` 读出 ESP32-S3 QFN56 rev0.2、内置 8 MB PSRAM (AP_3v3)、
  Flash 16 MB、MAC `14:c1:9f:c5:26:80`。
- 当前编程串口 COM6，FTDI `0403:6001`、序列号 `A5069RR4A`。
- 旧镜像应用描述：项目 `twai_network_master`、版本 `83b384a`、编译日期
  `Oct 8 2026 17:35:23`、IDF `v5.3.1-dirty`，ELF SHA-256
  `3ced017743c6a496950b50ec376b8b80fd41a03631cb8e73a0d06bd0a6692373`。
- 旧应用仅有数字 CAN ID 的工程控制台，不能当作产品 BLE 0.10.1。
  新版本使用 RS01 专用协议边界、LEFT/RIGHT 对象、RAM 零位、100 Hz 唯一控制任务、
  产品状态机/START/STOP、产品 BLE 及遥测；实际运行必须在升级后另行验证。
- 仓库配置仍是 LEFT→B/+1、RIGHT→A/−1；这证明配置值，当前装配的左右和方向仍需现场核对。

## 备份与构建

证据目录：`output/phase10/20261009T100926_web_motion/`。

旧镜像整片读取 `read_flash 0 ALL` 两次返回错误，未生成有效完整备份，也未写 Flash。
改用每块 1 MiB 的独立读取；每块由 esptool stub MD5 校验，合并后记录长度及 SHA-256。
完整性以 `original_full_flash_metadata.json` 和 `original_full_flash_16mb.bin` 为准。
不能把两次失败日志当作备份 PASS。

新固件使用本机 ESP-IDF v5.5，从现有工作树构建，不重建工程、不覆盖阶段10修改。
隔离配置 `sdkconfig.engineering`：TX4/RX1、16 MB Flash、DIO/40 MHz、PSRAM关闭，
仅该候选启用 `CONFIG_EXO_ENGINEERING_GAIT_STIMULUS`。根目录 sdkconfig 仍关闭该选项。
应用 665104 B，工厂应用分区仍 1 MiB。

工程候选应用 SHA-256：
`b2378d00f0d09720c9ea74ae08b933c38791168e3bfcc7cac7e4d70ea67b7233`。
详细产物身份见 `engineering_candidate_manifest.json`。

运动计划使用真实 `/exo` 网页按钮写入并读回工程配置：
`IMPEDANCE + ALTERNATING_SINE_TEST`、左右 q/dq/τ=0、Kp=5/Kd=0.1，
`ACTIVE_PHASE + PHASE_PROFILE + SHADOW`、profile1、左右10%。
基础工程轨迹单侧10°、0.5 Hz、3秒渐入；SHADOW只保证附加助力增量为0，基础轨迹会驱动电机。
目标测试窗口约15秒，串口仅用于诊断和20秒运行截止保护；若串口保护执行 STOP，
该轮不得记为网页 STOP E2E PASS。不会执行 Python BLE 运动验收来代替网页。

## 本轮现场确认与升级

操作者在本轮明确答复“当前条件已确认，可以进行低幅测试”，对应问题包含空载/台架、
双髋中立位、两侧±10°运动范围无阻挡及能随时断电。此确认只用于本轮，未复用8月中立位。
浏览器原生设备选择器无法由当前网页自动化接口操作，操作者现场选择 ESP-EXO 并配对，
随后网页 Product Service、Response/Telemetry Notify 和初始状态读回均成功。

- 旧 Flash 完整备份：16777216 B；每块 esptool MD5 验证成功，合并 SHA-256 为
  `b3957aed0ed20daedc5d8660933aae3852f6db7e88cf6c8b03f6f3bb13e46eff`。
- 工程候选烧录成功，bootloader、应用、分区表均由 esptool 验证；未执行整片擦除。
- 网页读回协议 v1、固件0.10.1、capabilities0x00FF、最大帧长20 B，
  Boot ID `0xB471CB40` 与 UART 一致。
- 升级后第一次串口打开时有乱码前缀，已原样保存在二进制/文本记录中；
  结论取命令响应中的完整诊断块，后续串口读回正常，不将乱码前缀当作有效状态。
- 首次 IDLE 读回 A/B 均 valid/online、约100 Hz Type24主动上报、REST、失能请求，
  故障/警告为0。零位未建立时 START 阻止为 LEFT_ZERO_INVALID，符合门禁。
- 网页“一键准备 TEST + SHADOW”逐项读回配置成功，尚未自动 START。
- 经本轮现场确认后，网页“双髋依次归零”捕获 RAM 零位 LEFT=23.485°、RIGHT=−29.529°。
  两侧最终有效，传感 available=valid=0x0F，系统 IDLE/Ready=YES/输出关闭。

## 本轮实测结果

| 项目 | 本轮结果及范围 |
| --- | --- |
| 当前 TX4/RX1 双电机通信/主动反馈 | PASS；A=0x10、B=0x20，约100 Hz、有效、新鲜、无故障 |
| 网页配置与RAM零位 | PASS；真实网页按钮及BLE逐项读回，未执行机械置零 |
| 网页启动工程交替轨迹 | 已证实真实运动；RUNNING、反馈反相、步态与SHADOW建议有效 |
| 第一轮工程轨迹停止 | 串口20秒保护先STOP并确认IDLE；网页随后STOP返回ALREADY_SATISFIED，不能记为工程轨迹网页STOP PASS |
| 产品DAMPING网页启停补测 | PASS；真实网页START→RUNNING→STOP→IDLE，保护未发送STOP，A/B最终REST |
| 产品配置恢复 | PASS；真实网页恢复并读回DAMPING+CONSTANT、左右q/dq/τ/Kp=0/Kd=0.1、TRANSPARENT+DISABLED |
| 五通道10 Hz运动遥测 | 本轮213个RUNNING样本全部通道完整；序号无缺口、实际附加助力全0 |
| 七通道20 Hz静止遥测 | 未通过无丢样本要求；队列32/32，停止后设备drop=408；不是持续耐久PASS |
| 遥测断开/重连E2E | 本轮NOT RUN；首次连接及遥测启停/导出有证据，不能代替重连 |
| 关闭工程开关的目标板验收 | 本轮NOT RUN；根sdkconfig关闭，但板上目前仍是声明0x00FF的工程构建 |
| 真人SHADOW/ACTIVE 0%/5%/10% | 本轮NOT RUN；本轮只做台架工程输入和产品基础阻尼启停 |

### 工程运动原始数据统计

第一轮用网页 START，五通道10 Hz、mask0x0079。网页原始导出保留了静止stream1和运动stream2；
统计严格筛选 stream2、systemState=RUNNING、outputAllowed=true，未把旧流或IDLE数据混入。

- RUNNING完整样本213个，设备时间跨度21.2 s、activation epoch低16位=2。
- 左右反馈峰峰值17.074°/15.986°，范围分别−8.824～8.251°、−8.251～7.735°。
- 左右相关系数−0.988074；双侧相位有效135/213（63.38%），非零SHADOW建议114/213（53.52%）。
- 213个样本的左右实际附加助力均为0，realOutputEnabled均false；全stream2序号缺口为0。
- 运行20秒截止保护发送串口STOP，停止并确认IDLE耗时约21.75秒（从保护检测RUNNING起）。
  网页STOP提交晚于保护，因此第一轮只证明运动/SHADOW/保护停止，不证明工程轨迹网页STOP。

补测先通过网页恢复产品基础阻尼，避免在停止后的偏置姿态再次施加交替位置轨迹。
网页事务日志记录：UTC02:33:09.339 START、02:33:14.931 STOP、02:33:15.332 STOP完成IDLE。
`damping_guard.json` 为 `webpage_stop_idle_verified`、`emergency_stop_sent=false`；
保护检测RUNNING到确认IDLE约4.94 s。此补测不算真人ACTIVE 0%：助力执行态是DISABLED。

最终UART证据：START requests/success=2/2、failure=0；STOP requests=3、success=2、failure=0。
多出的STOP请求是第一轮保护停止后网页发出的ALREADY_SATISFIED。
100 Hz任务mean=10000 us、周期范围9190～10808 us、最大执行4162 us、deadline/skipped/output/algorithm error均0，
任务最小剩余栈1092 B；CAN TX/RX错误、发送失败、bus error、arbitration lost、RX missed均0。
A/B均REST、enabled_requested=no、fault/warning=0。最终关节角约LEFT−6.594°/RIGHT4.901°，
这是停止后的实际姿态，不宣称自动回到机械中立位；没有重捕零位掩盖该偏置。

### 七通道20 Hz观察与限制

首次静止stream1，网页启动至提交停止约44 s。网页最后计数471个采样时刻、3292个Notify、
网页序号缺口375，设备最终提交3330帧、drop408。UART观察到频繁的
`NimBLE: GATT procedure initiated: notify`/`att_handle` INFO日志，
mbuf分配失败、提交失败和满32样本队列。这些逐包串口打印可能加重吞吐瓶颈，属于待验证原因，
不能仅凭同时出现就断言是唯一根因。
五通道10 Hz流的设备drop=0、网页序号缺口=0，但BLE底层仍有累计重试/分配及异步TX失败，
因此“完整收到本轮运动样本”不等于“底层从未拥塞”，也不等于所有遥测配置通过。
本轮未修改日志策略或传输协议，七通道20 Hz持续负载仍需修复并独立回归。

## 证据清单与当前留置状态

- 芯片/旧镜像：chip_flash_probe.txt、original_firmware_metadata.json、
  original_full_flash_metadata.json、original_full_flash_16mb.bin和逐块备份日志。
- 新固件：engineering_candidate_manifest.json、engineering_build.txt、engineering_flash.txt。
- UART：engineering_after_flash、engineering_idle_second、engineering_7channel_20hz_idle、
  engineering_zero_prestart、after_web_damping_stop各自的_uart.bin/_uart.txt。
- 保护：motion_guard.json/_uart.bin（实际保护STOP）、damping_guard.json/_uart.bin（未介入）。
- 网页：web_7channel_20hz_idle.csv、web_motion_first.csv/.jsonl、web_motion_running_only.csv、
  web_motion_metrics.json、web_transactions_before_restore.jsonl、web_transactions_final.jsonl、
  web_*各操作DOM快照、web_running_full.png、web_final_idle.png。

最终板上是工程能力开启的0.10.1，但运行配置已恢复产品待机、系统IDLE、输出关闭、A/B REST。
RAM软件零位仍有效，若重启/烧录就失效；网页服务保留在 `http://localhost:4173/exo`。
所有串口已关闭；未reset工作树、未清理原有文件、未提交/推送Git。

阶段10总体状态保持：`0.10.1 ENGINEERING MOTION HIL PASS; HUMAN ACTIVE HIL NOT RUN`。
本轮新增真实网页驱动和产品基础启停证据，未完成真人SHADOW/ACTIVE、工程轨迹网页STOP单轮合同、
遥测重连、七通道20 Hz持续负载或关闭工程开关的目标板验收，阶段10不能整体完成，阶段11未完成。
