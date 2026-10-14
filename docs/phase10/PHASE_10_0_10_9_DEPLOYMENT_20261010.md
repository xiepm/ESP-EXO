# 0.10.9 部署与可选沿用零位验收（2026-10-10）

当前状态：`FLASH PASS; IDLE / MOTION OUTPUT 0; WEB INFO PENDING; SAVED ZERO HIL NOT RUN`。
阶段10整体及阶段11仍未完成；本记录不代表人体助力或5 N·m真实输出通过。

## 授权、产物与烧录

操作者本轮明确回复“已断电，请继续”，确认两台电机动力断开并继续已提出的0.10.9升级。
从既有工程冻结产物烧录，不重新搭建工程、不整片擦除、不改动方向NVS。

- 冻结目录：`output/phase10/20261010_saved_wear_zero/candidate-0.10.9/`。
- 烧录前197个文件大小和SHA256全部一致。
- 应用677776字节，SHA256：`90c8823f2cd8a63c61d00849c12bb9a62ac49992b50f4dbb85e0ff15f1dec76e`。
- COM6为FTDI `0403:6001 / A5069RR4A`；esptool 4.12.0识别ESP32-S3 revision v0.2、8 MB嵌入式PSRAM、MAC `14:c1:9f:c5:26:80`。
- 写入bootloader `0x0`、partition `0x8000`、app `0x10000`，DIO / 40 MHz / 16 MB；三处写入哈希校验通过，退出码0。
- 冻结manifest保留当时`SOFTWARE_PASS_NOT_FLASHED`状态；部署结果独立记录，不回写冻结证据。

## 本轮只读现场结果

烧录前：Boot ID `0xE32D814D`，IDLE、START请求0、输出门关闭、运动输出0。
双侧反馈已离线约160秒，RX GPIO1为低；CAN RUNNING但REC=128、累计总线错误19848303。
这些是在断电后的新采样，不能将早前在线/错误0的记录当作当时状态，也不能仅凭RX数字电平确定电气根因。

烧录后稳定采样：Boot ID `0x47FCCEF7`，IDLE、START请求0、输出门关闭、控制器未armed、运动输出0。
100 Hz循环mean=10000 us、min/max=9993/10007 us，deadline/skipped/output error均0（该次短时观察）。
LEFT→B/−1、RIGHT→A/+1方向记录保留；双侧RAM零位无效，原始反馈计数0，电机动力按操作者确认保持断开。
助力恢复启动默认TRANSPARENT / DISABLED、双侧0%、最大附加力矩0.500 N·m；未自动恢复升级前用户选择的ACTIVE / 100% / 5 N·m。
CAN TX4 / RX1 / 1 Mbps，RX低、发送队列0、TEC/REC=0/0、总线错误0；断电状态的计数0不等于上电通信恢复通过。

烧录后首次串口读取遇到GBK输出编码异常，后续UTF-8读取包含大量非文本字节；原始记录保留。
再次只读采样已获得完整正常状态、同一Boot ID，串口均在finally中释放，DTR/RTS在open前设为false。
尚未将该次非文本字节归因为固件异常。

## 网页与零位验收

已在原`http://localhost:4173/exo`页面发起ESP-EXO选择；浏览器原生设备选择器由操作者完成。
INFO版本/能力、主控候选读取暂待连接后核对。

目标板候选保存、复位后保留但RAM不自动启用、显式沿用、放弃沿用与姿态拒绝：`NOT RUN`。
首次使用需当前穿戴中立静止确认及双侧新REST反馈，校准并保存一次；旧固件未持久保存的RAM偏置不能事后恢复。
上述校准操作不START、不使能、不发送运动或机械置零。

## 原始证据

`output/phase10/20261010_saved_wear_zero/before-flash-0.10.9-read-only.log`
及`output/phase10/20261010_saved_wear_zero/deployment-0.10.9-20261010/`内：

- `candidate-verification-before-flash.json`
- `flash.log` / `flash-exit-code.txt`
- `after-flash-read-only.log` / `after-flash-read-only-utf8.log`（首次读取异常记录）
- `after-flash-stable-read-only.log` / `after-flash-stable-console.txt`（有效状态记录）
- `deployment-summary.json`（部署状态）
- `web-pairing-pending-state.txt` / `web-pairing-pending.jpg`（原页面等待原生配对，尚非INFO验证）

后续只将实际执行的验收更新为PASS；人体运动SHADOW/ACTIVE仍`NOT RUN`。
