# 0.10.8烧录与5 N·m网页配置

当前状态：`FLASH PASS; POST FLASH IDLE/OUTPUT ZERO PASS; WEB INFO/5NM CONFIG READBACK PASS; HUMAN SHADOW/ACTIVE NOT RUN`。

## 授权和冻结产物

用户最终要求：ANGLE_THRESHOLD单向前抬助力峰值5 N·m，最大上升速率10 N·m/s。用户随后明确“已断电，请继续”。烧录前只读COM6确认IDLE、START请求0、运动输出0、双电机反馈已离线。

冻结产物：`output/phase10/20261010_angle_assist_5nm/candidate-0.10.8/`。烧录前138个源码、产物和证据文件重新校验哈希通过。冻结清单的NOT FLASHED是冻结时状态，保持不变；部署证据单独存放。

应用文件673232字节，SHA256：`28977f12451562c1b426a5f8a258ff6dd262278811072cba1e6a76c348d53af3`。

## 实际烧录

2026-10-10约17:07（北京时间），COM6，FTDI 0403:6001/A5069RR4A；esptool4.12.0检测ESP32-S3 QFN56 v0.2、8 MB PSRAM。使用冻结的flash_args：DIO、40 MHz、16 MB Flash。

写入bootloader 0x0、应用0x10000、分区表0x8000；三次`Hash of data verified`通过，随后硬件复位。没有整片擦除，NVS出厂方向保留。没有发送START、使能、软件归零或机械置零。

## 烧录后只读检查

`after-flash-serial.log`和`flash-summary.json`记录：

| 项目 | 观察结果 |
| --- | --- |
| Boot ID | 0xE32D814D，与烧录前0xE166E107不同 |
| 整机 | IDLE、run_requested=no、output_allowed=no |
| START / STOP请求 | 均0 |
| 控制器运动输出/错误 | 0/0，未armed |
| 默认基础 | DAMPING+CONSTANT、左右Kd=0.100 |
| 默认助力 | TRANSPARENT、DISABLED、双侧0%、峰值0.500 N·m |
| 出厂方向 | LEFT→B/−1，RIGHT→A/+1，均valid |
| RAM穿戴零位 | 两侧INVALID，复位后已清除 |
| CAN | RUNNING、TX4/RX1、1 Mbps，TEC/REC=0/0，总线错误0 |
| 电机反馈 | 均0/INVALID/OFFLINE，操作者保持动力断电 |
| BLE | 初始未连接；操作者重新选择并配对后，Product Service、Response Notify及初始状态验证通过 |

电机无反馈时status中的角度/力矩零值不是实测有效状态。烧录前断电窗口曾观察到RX低、REC134及大量累计总线错误；烧录后只读窗口RX仍低而错误0，尚未完成本版本动力上电恢复检查，不把计数复位视为电气故障根因解决。

## 网页实际写入与读回

配对完成后，网页INFO显示固件0.10.8、CAPABILITIES=0x3F7F（包含bit13）、BOOT ID=0xE32D814D，与串口当前启动一致。网页连接状态显示Product Service、Response Notify与初始状态均已验证。

通过运动页“保存助力设置并读回”按钮实际写入ANGLE_THRESHOLD、M=5.000 N·m、T=10.00°、左右等级0%、DISABLED。最终页面分别显示“设备已保存策略：ANGLE_THRESHOLD”“设备已保存：5.000 N·m”“设备已保存：10.00°”、双侧已保存0%；工程上限5.000 N·m，上升速率说明10 N·m/s。这是设备读回，非未保存草稿。

17:18的`after-web-config-serial.log`独立确认：策略版本2、selected=5.000 N·m（RAM）、双侧0%/DISABLED；BLE已连接且Response Notify已订阅；IDLE、START/STOP请求0、输出门关闭、运动输出/错误0/0。100 Hz控制任务67183周期，周期9991–10011 us，截止时间违约和跳周期均0。记录时间以日志为准，不把这个断电空载窗口称为运动负载验收。

网页原始状态与截图：`web-info-state.txt`、`web-5nm-saved-state.txt`、`web-5nm-saved.jpg`。此前旧对象重连失败与被动扫描无结果保留为历史过程；后续实际连接及参数读回已经通过。

## 尚待完成

需要动力上电后的真实反馈恢复证据；穿戴零位须在操作者确认当前中立静止后重新采集。未START，未执行本策略运动中SHADOW/ACTIVE，未测实际5 N·m输出。阶段10整体未完成，阶段11未完成。

原始证据目录：`output/phase10/20261010_angle_assist_5nm/deployment-0.10.8-20261010/`。
