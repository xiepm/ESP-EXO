# RobStride 电机控制台命令手册

## 命令通道

同一套命令同时支持：

1. **烧录串口（推荐调试方式）**：115200 baud，发送 ASCII 命令并以 `\r` 或
   `\n` 结束。可直接在 ESP-IDF monitor 中输入命令后按 Enter。
2. **BLE NUS**：配置见下节。

烧录串口示例：

```powershell
. 'C:\Espressif\tools\Microsoft.v5.5.PowerShell_profile.ps1'
idf.py -p COMx flash monitor
```

进入 monitor 后输入：

```text
help
motor 0x10
status
```

> 实际端口号以设备管理器或 `idf.py -p <PORT>` 使用的烧录端口为准。

## BLE NUS 配置

| 参数 | 值 |
|------|-----|
| 广播名称 | `RobStride-CAN` |
| Service UUID | `6E400001-B5A3-F393-E0A9-E50E24DCCA9E` |
| RX UUID（客户端写入） | `6E400002-B5A3-F393-E0A9-E50E24DCCA9E` |
| TX UUID（客户端订阅 Notify） | `6E400003-B5A3-F393-E0A9-E50E24DCCA9E` |
| 命令编码 | UTF-8 / ASCII 文本 |
| 命令结束符 | `\r` 或 `\n` |

使用支持 Nordic UART Service 的 BLE 客户端连接 `RobStride-CAN`，先订阅 TX
特征通知，再向 RX 特征写入命令。例如发送 `help\r\n`。

烧录串口同时用于固件烧录、ESP-IDF 日志和命令控制：

```powershell
. 'C:\Espressif\tools\Microsoft.v5.5.PowerShell_profile.ps1'
idf.py -p COMx monitor
```

> **CAN 总线**: TX=GPIO4, RX=GPIO2, 1Mbps（可在 Kconfig 中修改）。

---

## 基本用法

连接并订阅 TX 通知后会收到欢迎信息。所有命令通过 RX 特征写入，使用回车
或换行执行。运动控制命令按一次写入发送一次，不会在后台周期重发。

未接电机时可安全验证 `help`、`motor 0x01`、`motor_type` 和 `status`。

### 选择目标电机

```
motor <can_id>
```

设置当前要控制的电机 CAN ID。后续所有命令都发往这个电机。默认值: `0x10` (16)。

示例：
```
motor 0x01     # 选择 CAN ID = 1 的电机
motor 0x10     # 选择 CAN ID = 16 的电机
```

---

## 私有协议命令

### enable — 电机使能
```
enable
```
发送使能命令（通信类型 0x03）。电机上电。

### disable — 电机失能
```
disable
```
发送失能命令（通信类型 0x04）。电机下电。

### move — 运控模式（混合控制）
```
move <torque> <position> <speed> <kp> <kd>
```
| 参数 | 范围 | 单位 | 说明 |
|------|------|------|------|
| torque | T_MIN ~ T_MAX | Nm | 目标力矩 |
| position | -12.57 ~ 12.57 | rad | 目标位置 |
| speed | V_MIN ~ V_MAX | rad/s | 目标速度 |
| kp | KP_MIN ~ KP_MAX | - | 比例增益 |
| kd | KD_MIN ~ KD_MAX | - | 微分增益 |

示例：`move 1 0 2 0.5 0.1`

### speed — 速度控制模式
```
speed <cur_limit> <acc> <speed>
```
| 参数 | 范围 | 单位 | 说明 |
|------|------|------|------|
| cur_limit | 0 ~ 23 | A | 电流限制 |
| acc | - | rad/s² | 加速度 |
| speed | V_MIN ~ V_MAX | rad/s | 目标速度 |

示例：`speed 1 1 2`

### pos_pp — PP 位置控制模式
```
pos_pp <speed> <acc> <angle>
```
| 参数 | 范围 | 单位 | 说明 |
|------|------|------|------|
| speed | V_MIN ~ V_MAX | rad/s | 目标速度 |
| acc | - | rad/s² | 加速度 |
| angle | -12.57 ~ 12.57 | rad | 目标角度 |

示例：`pos_pp 6 1 2`

### pos_csp — CSP 位置控制模式
```
pos_csp <speed> <angle>
```
| 参数 | 范围 | 单位 | 说明 |
|------|------|------|------|
| speed | 0 ~ 44 | rad/s | 速度限制 |
| angle | -12.57 ~ 12.57 | rad | 目标角度 |

示例：`pos_csp 4 1`

### current — 电流（力矩）控制模式
```
current <current>
```
| 参数 | 范围 | 单位 | 说明 |
|------|------|------|------|
| current | -23 ~ 23 | A | 目标电流 |

示例：`current 0.5`

### set_zero — 设置机械零位
```
set_zero
```
将当前电机位置设为零点。

### set_id — 修改电机 CAN ID
```
set_id <new_id>
```
修改当前电机的 CAN ID（0~127）。修改后需要用新 ID 重新选择电机。示例：`set_id 1`

### save — 保存参数到 Flash
```
save
```
将当前电机参数写入内部 Flash，断电不丢失。

### set_baud — 修改 CAN 波特率
```
set_baud <rate>
```
| rate | 波特率 |
|------|--------|
| 1 | 1 Mbps |
| 2 | 500 Kbps |
| 3 | 250 Kbps |
| 4 | 125 Kbps |

示例：`set_baud 1`

### report — 主动上报开关
```
report <0|1>
```
- `0` — 关闭主动上报
- `1` — 开启主动上报（默认间隔 10ms）

### read_param — 读参数
```
read_param <reg_addr>
```
读取电机寄存器（私有协议）。地址为十六进制。示例：`read_param 0x7005`

### switch_mit — 切换到 MIT 协议
```
switch_mit
```
选择 MIT 协议模式。电机必须断电重启后才生效。

### switch_private — 切换回私有协议
```
switch_private
```
选择私有协议模式。电机必须断电重启后才生效。

---

## MIT 协议命令

### mit_enable — MIT 电机使能
```
mit_enable
```

### mit_disable — MIT 电机失能
```
mit_disable
```

### mit_control — MIT 运控模式
```
mit_control <angle> <speed> <kp> <kd> <torque>
```
| 参数 | 范围 | 说明 |
|------|------|------|
| angle | -12.57 ~ 12.57 rad | 目标角度 |
| speed | V_MIN ~ V_MAX rad/s | 目标速度 |
| kp | KP_MIN ~ KP_MAX | 比例增益 |
| kd | KD_MIN ~ KD_MAX | 微分增益 |
| torque | T_MIN ~ T_MAX Nm | 目标力矩 |

示例：`mit_control 1 0 1 0.1 0.1`

### mit_pos — MIT 位置模式
```
mit_pos <position> <speed>
```
| 参数 | 单位 | 说明 |
|------|------|------|
| position | rad | 目标位置 |
| speed | rad/s | 目标速度 |

示例：`mit_pos 0 1`

### mit_speed — MIT 速度模式
```
mit_speed <speed> <current_limit>
```
| 参数 | 单位 | 说明 |
|------|------|------|
| speed | rad/s | 目标速度 |
| current_limit | A | 电流限制 |

示例：`mit_speed 10 2`

### mit_set_zero — MIT 设置零位
```
mit_set_zero
```

### mit_set_type — MIT 设置电机类型
```
mit_set_type <type>
```
| type | 模式 |
|------|------|
| 0 | MIT 默认模式 |
| 1 | 位置模式 |
| 2 | 速度模式 |

### mit_set_id — MIT 设置电机 CAN ID
```
mit_set_id <new_id>
```

### mit_save — MIT 保存参数
```
mit_save
```

### mit_report — MIT 主动上报开关
```
mit_report <0|1>
```

### mit_read — MIT 读参数
```
mit_read <reg_addr>
```
标准帧 MIT 协议未定义运行参数读命令。固件会拒绝该命令，不发送 CAN 帧。

### mit_write — MIT 写参数
```
mit_write <reg_addr> <value>
```
标准帧 MIT 协议未定义运行参数写命令。固件会拒绝该命令，不发送 CAN 帧。

### mit_error — MIT 查询或清除错误
```
mit_error <value>
```
- `mit_error 0xFF`：清除当前错误。
- 其他 `0~255` 数值：按参考协议发送错误状态查询命令。

---

## 双电机正弦阻抗基线

### imp_start — 启动双电机基线控制

```text
imp_start
```

读取 CAN ID `0x10` 和 `0x20` 的当前位置作为中心点，然后启动当前原型的
100 Hz 正弦参考与阻抗控制。若任一电机没有有效反馈，固件仍可能用零位继续，
因此阶段 0 验收时必须同时检查日志中没有 `No feedback from motor`。

### imp_stop — 停止双电机基线控制

```text
imp_stop
```

停止 100 Hz 定时器控制并向两台电机发送失能命令。

---

## 工具命令

### status — 显示电机反馈
```
status
```
输出当前电机反馈数据：角度、速度、力矩、温度、运行状态、故障标志。

### can_status — 显示 CAN 控制器状态
```
can_status
```
输出 TWAI 运行状态、TX/RX 队列、错误计数、发送失败、总线错误和仲裁丢失
计数。若 `TX error counter`/`Bus error count` 持续增加或进入
`BUS_OFF`，应检查收发器使能、CAN_H/CAN_L、共地、终端电阻和波特率。

### motor_type — 查看电机型号参数
```
motor_type
```
显示编译时选择的 `MOTOR_TYPE` 及对应的 V/KP/KD/T 范围。

### help — 显示帮助
```
help
```
列出所有可用命令。

---

## 当前电机型号参数 (MOTOR_TYPE=5)

| 参数 | 最小值 | 最大值 |
|------|--------|--------|
| 位置 (P) | -12.57 rad | 12.57 rad |
| 速度 (V) | -50.0 rad/s | 50.0 rad/s |
| KP | 0.0 | 500.0 |
| KD | 0.0 | 5.0 |
| 力矩 (T) | -5.5 Nm | 5.5 Nm |

> 修改 `MOTOR_TYPE` 宏（0~6）可切换不同的电机型号参数。编译时生效。

---

## 典型操作流程

```
# 1. 选择电机
motor 0x10

# 2. 使能电机
enable

# 3. 运控模式控制
move 1 0 2 0.5 0.1

# 4. 查看反馈
status

# 5. 切换到 MIT 协议
switch_mit
mit_enable

# 6. MIT 控制
mit_control 1 0 1 0.1 0.1

# 7. 失能电机
mit_disable
```
