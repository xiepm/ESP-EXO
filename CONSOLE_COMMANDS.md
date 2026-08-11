# ESP-EXO RS01 与髋关节 UART 调试控制台命令手册

> 这是仅通过烧录 UART 使用的工程调试接口，不是阶段 6 的手机产品协议。固件上电后会为
> A/B 两台 RS01 写入 10 ms 上报周期并请求开启主动反馈；该操作不会使能电机。手机 BLE
> 接口见 `docs/phase6/BLE_PROTOCOL_V1.md`，不接受本页的 ASCII 命令。

本控制台从固件 `0.8.0` 起，运动控制命令和状态快照中的角度、角速度、角加速度
输入/显示统一使用 `deg`、`deg/s`、`deg/s²`。RS01 驱动内部仍按电机手册使用
弧度，并在 UART 边界后立即显式换算；旧的弧度制运动参数不能直接复用。裸寄存器
读写（`read_param`、`mit_read`、`mit_write`）仍透传手册原始值，不做单位推断或换算。

## 命令通道

调试命令只支持**烧录串口**：115200 baud，发送 ASCII 命令并以 `\r` 或
`\n` 结束。可直接在 ESP-IDF monitor 中输入命令后按 Enter。

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

## 与产品 BLE 的边界

- UART 保留全部工程维护、原始 RS01 和诊断命令；
- 产品 BLE 开放 `GET_INFO`、系统/关节/步态/助力状态、静态配置、`START`、`STOP`
  和髋关节 RAM 软件零位命令；
- 产品 BLE 不支持 Nordic UART Service，也不传输 ASCII 文本；
- 阶段 6 仅按请求返回快照，不连续推送 100 Hz 反馈；连续遥测在阶段 10 加入。

烧录串口同时用于固件烧录、ESP-IDF 日志和命令控制：

```powershell
. 'C:\Espressif\tools\Microsoft.v5.5.PowerShell_profile.ps1'
idf.py -p COMx monitor
```

> **CAN 总线**：TX=GPIO4，RX=GPIO2，1 Mbps；当前值集中定义在
> `g_exo_hardware_config`，不是通过 Kconfig 修改。

---

## 基本用法

连接烧录串口后输入命令并按 Enter。运动控制命令按一次输入发送一次，不会在后台
周期重发。

未接电机时可验证 `help`、`motor A`、`motor_type`、`status` 和
`can_status`；此时 `status` 应显示 `Valid: no`、`Online: no`，而
`system_status` 会说明 START 当前被阻止的原因。

调试命令分为三类：

- 系统请求：`start`、`stop` 及其兼容别名，只提交异步请求；
- 只读或本地命令：`motor`、各类 `status` 和 `help`，始终可用；
- 维护命令：所有会发送调试 CAN 帧、切换模式或修改 RAM 软件零位的命令。

维护命令只有在系统处于 `IDLE` 且执行器没有占用电机操作权时才会执行。系统处于
`STARTING`、`RUNNING`、`STOPPING` 或 `ERROR` 时，统一分发器会在调用具体处理函数
之前拒绝它，因此不会出现“先检查状态、随后又插入 CAN”的竞争窗口。先发送 `stop`，
再用 `system_status` 确认回到 `IDLE`，才能继续维护操作。

### 选择目标电机

```
motor <can_id>
```

选择当前要控制的逻辑执行器。后续命令只接受逻辑槽 A/B；为了兼容原调试流程，
也可输入这两个槽在硬件配置中的 CAN ID。A/B 只表示逻辑槽，不表示人体左右侧。
默认选择 A（当前 CAN ID `0x10`）。

示例：
```
motor A        # 选择逻辑槽 A（当前 CAN ID 0x10）
motor B        # 选择逻辑槽 B（当前 CAN ID 0x20）
motor 0x10     # 兼容写法，映射到 A
motor 0x20     # 兼容写法，映射到 B
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
| torque | -17 ~ 17 | Nm | 目标力矩 |
| position | -720.21 ~ 720.21 | deg | 目标位置 |
| speed | -2521.01 ~ 2521.01 | deg/s | 目标速度 |
| kp | 0 ~ 500 | - | 比例增益 |
| kd | 0 ~ 5 | - | 微分增益 |

示例：`move 1 0 2 0.5 0.1`

### speed — 速度控制模式
```
speed <cur_limit> <acc> <speed>
```
| 参数 | 范围 | 单位 | 说明 |
|------|------|------|------|
| cur_limit | 0 ~ 23 | A | 电流限制 |
| acc | - | deg/s² | 加速度 |
| speed | -2521.01 ~ 2521.01 | deg/s | 目标速度 |

示例：`speed 1 1 2`

### pos_pp — PP 位置控制模式
```
pos_pp <speed> <acc> <angle>
```
| 参数 | 范围 | 单位 | 说明 |
|------|------|------|------|
| speed | -2521.01 ~ 2521.01 | deg/s | 目标速度 |
| acc | - | deg/s² | 加速度 |
| angle | -720.21 ~ 720.21 | deg | 目标角度 |

示例：`pos_pp 180 60 30`

### pos_csp — CSP 位置控制模式
```
pos_csp <speed> <angle>
```
| 参数 | 范围 | 单位 | 说明 |
|------|------|------|------|
| speed | 0 ~ 2521.01 | deg/s | 速度限制 |
| angle | -720.21 ~ 720.21 | deg | 目标角度 |

示例：`pos_csp 180 30`

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
向当前 RS01 发送私有协议的电机机械置零命令。它会改变电机侧零点，不是髋关节
RAM 软件零位；阶段 3 标定应使用 `joint_zero left/right`，不要使用本命令。

### set_id — 修改电机 CAN ID
```
set_id <new_id>
```
修改当前电机的 CAN ID（0~127）。发送成功后，当前运行中的逻辑槽会立即切换到
新 ID，并拒绝与另一个槽重复的 ID。若希望下次上电仍使用新 ID，还必须同步修改
`g_exo_hardware_config`。示例：`set_id 1`。

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
- `1` — 开启主动上报（启动时已将间隔配置为 10 ms）

`report` 只改变当前所选 A/B。命令返回成功表示 CAN 帧已发送；是否真正收到数据，
应以 `status` 的 `Online`、`Feedback count` 和 `Feedback age` 为准。
当前产品固件实测需写入 `0x7026=0` 才得到约 10 ms；手册所写的值 1 在当前
两台电机上约为 15 ms。驱动已封装该差异，控制台无需直接写寄存器。

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
| angle | -720.21 ~ 720.21 deg | 目标角度 |
| speed | -2521.01 ~ 2521.01 deg/s | 目标速度 |
| kp | 0 ~ 500 | 比例增益 |
| kd | 0 ~ 5 | 微分增益 |
| torque | -17 ~ 17 Nm | 目标力矩 |

示例：`mit_control 30 0 1 0.1 0.1`

### mit_pos — MIT 位置模式
```
mit_pos <position> <speed>
```
| 参数 | 单位 | 说明 |
|------|------|------|
| position | deg | 目标位置 |
| speed | deg/s | 目标速度 |

示例：`mit_pos 0 60`

### mit_speed — MIT 速度模式
```
mit_speed <speed> <current_limit>
```
| 参数 | 单位 | 说明 |
|------|------|------|
| speed | deg/s | 目标速度 |
| current_limit | A | 电流限制 |

示例：`mit_speed 600 2`

### mit_set_zero — MIT 设置零位
```
mit_set_zero
```
向当前 RS01 发送 MIT 协议的电机机械置零命令。它不是髋关节 RAM 软件零位；
阶段 3 标定不得使用本命令。

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
发送标准帧参数读命令，标识符为 `(3 << 8) | CAN_ID`。

### mit_write — MIT 写参数
```
mit_write <reg_addr> <value>
```
发送标准帧参数写命令，标识符为 `(4 << 8) | CAN_ID`，值按
little-endian `float32` 编码。

### mit_error — MIT 查询或清除错误
```
mit_error <value>
```
- `mit_error 0xFF`：清除当前错误。
- 其他 `0~255` 数值：按参考协议发送错误状态查询命令。

---

## 系统状态与 START/STOP

阶段 5 的唯一整机状态为：

- `INIT`：系统初始化未完成；
- `IDLE`：输出关闭，等待操作；
- `STARTING`：异步执行器正在准备两侧电机；
- `RUNNING`：允许唯一 100 Hz 控制任务产生运动输出；
- `STOPPING`：输出许可已撤销，异步执行器正在失能两侧电机；
- `ERROR`：START/STOP 转换或清理失败。

`ready_to_start` 是由控制任务、TWAI、左右关节零位/反馈及故障状态派生的就绪值，
不是第七个系统状态。

### start — 请求启动

```text
start
```

该命令只调用非阻塞请求接口。返回 `ACCEPTED` 表示请求已受理且系统进入
`STARTING`，不表示电机准备已经完成；随后应查询 `system_status`，直到状态成为
`RUNNING` 或返回 `IDLE`/`ERROR`。在 `RUNNING` 中重复发送不会重新使能电机、
重新捕获中心点或重置正弦相位。

### stop — 请求停止

```text
stop
```

该命令立即撤销运动输出许可并提交异步清理请求。返回 `ACCEPTED` 不表示双电机
已经确认失能；随后应查询 `system_status`，直到状态成为 `IDLE` 或 `ERROR`。
在 `IDLE` 中重复发送是幂等操作。

### imp_start / imp_stop — 阶段 4 兼容别名

`imp_start` 完全等价于 `start`，`imp_stop` 完全等价于 `stop`。两者不再直接调用
阻抗控制器，也不绕过整机状态机。常驻 100 Hz 控制任务不会因这些命令而创建、
停止、删除或重启。

### system_status — 显示整机生命周期快照

```text
system_status
```

输出一次一致性快照，包括当前六态、活动转换步骤、`ready_to_start` 及阻止原因、
运行请求与输出许可、执行器任务身份/存活状态、状态转换和启停计数、拒绝/失败
计数、最近请求结果和错误，以及进入当前状态和最近完成转换的时间戳。

---

## 工具命令

### status — 显示电机反馈
```
status
```
输出所选 A/B 的独立反馈快照：有效/在线状态、上报和使能请求、有效/无效帧计数、
type 2/type 24 分项计数、
最近帧间隔和估算频率、反馈年龄、角度、速度、力矩、温度、运行模式及故障标志。
`Online` 只在最近 100 ms 内收到该执行器的有效反馈时为 `YES`。
角度和角速度显示为 `deg`、`deg/s`；这是显示层换算，不改变 RS01 原始帧编码。

### joint_status — 显示髋关节快照

```text
joint_status <left|right>
```

输出左/右髋到执行器 A/B 的映射、安装方向、RAM 软件零位、原始执行器采样、换算后的关节采样、
反馈时间戳和最终 `Joint valid`。只有映射、方向、软件零位、反馈有效和在线状态全部成立时，
`Joint valid` 才为 `YES`。
软件零位、原始执行器角和人体关节角均以 `deg` 显示。

### joint_zero — 捕获软件零位

```text
joint_zero <left|right>
```

把当前 RS01 输出位置保存为所选髋关节的 RAM 偏置。该命令不发送 CAN 帧、不使能电机，也不会修改
RS01 机械零位；重启 ESP32 后该运行时零位丢失。它与电机维护命令 `set_zero`、`mit_set_zero` 完全不同。

### joint_zero_clear — 清除软件零位

```text
joint_zero_clear <left|right>
```

只清除所选髋关节的 RAM 软件零位，不影响另一侧，也不修改电机反馈或机械零位。

### motion_mode — 选择基础控制模式

```text
motion_mode <hold|position|velocity|torque|damping|impedance> [constant|sine]
```

只修改下一次 START 要锁存的 `selected` 配置，不使能电机、不发送周期 CAN。参数只
能在整机 `IDLE` 时修改；RUNNING 中应先 `stop` 并等待回到 IDLE。`sine` 当前只允许
与 `impedance` 搭配，例如复现阶段 7：

```text
motion_mode impedance sine
```

六种产品模式底层仍全部使用 RS01 Motion 五元帧，不会在 100 Hz 路径切换 PP、
CSP、速度或电流模式。SINE 是参考源，不是第七种产品模式。

### joint_command — 设置单侧五元命令

```text
joint_command <left|right> <position_deg> <velocity_deg_s> <torque_nm> <kp> <kd>
```

在 IDLE 中写入下一次 START 使用的一侧五元参数。位置、速度和前馈力矩采用人体
关节坐标，正方向为髋屈曲；左右安装方向和软件零位由 `hip_joint` 统一映射。KP/KD
是 RS01 Motion 协议增益，不随右腿方向翻转，也不宣称是已经标定的物理刚度。

该命令不会直接发 CAN。推荐流程是分别设置 LEFT/RIGHT 后先执行 `control_status`
读回，确认无误再执行 `start`。

### control_status — 显示唯一控制循环状态

```text
control_status
```

按需输出常驻 100 Hz 控制任务的任务身份、周期数、目标及实测周期、执行时间、
超期和跳周期数、输出错误、栈最低余量，以及 LEFT/RIGHT 本周期反馈来源与
`joint_valid`。阶段 8 还会显示 selected/active 模式、参考源、左右 selected 五元
参数和最近实际输出。该命令只是 UART 文本诊断，不是连续遥测，也不会启动、停止
或修改控制任务。

### sensor_status — 显示统一传感快照

```text
sensor_status
```

读取唯一 100 Hz 任务最近发布的 sensor snapshot，包括 sequence、captured time、
reset epoch、available/valid mask、质量位、左右来源时间差，以及 LEFT/RIGHT 的人体
髋角（deg）、角速度（deg/s）和 RS01 电机估算力矩。它不会重新采样，也不会创建任务。
电机估算力矩不能当作已经标定的人机交互力矩。

### gait_status — 显示步态估计

```text
gait_status
```

显示 HIP_EVENT_TBE 的 RESET/STANDING/ACQUIRING/LOCKED/DEGRADED 状态、左右连续
相位、周期、步频、置信度、事件和质量位，以及 snapshot/gait/assist 分项 last/max
耗时。当前相位原点明确是 `HIP_FLEXION_PEAK (not heel strike)`；没有足底或外部真值
时，只能把它称为髋角代理相位，不能声称足跟着地检测或论文级误差。

### assist_status — 显示高层助力策略状态

```text
assist_status
```

显示 selected/requested 与 active/effective 配置、策略版本、START activation epoch、
所需传感器/步态条件、SHADOW 结果绑定的 sensor/gait sequence，以及不可用原因。
阶段 9 的 `REAL OUTPUT` 必须始终为 `DISABLED`。

### assist_config — 设置下一次 START 的高层策略

```text
assist_config <transparent|active_phase|aan> <disabled|shadow> \
              <transparent|phase_profile|aan_residual> \
              <profile 0..255> <left 0..100> <right 0..100>
```

只允许整机 IDLE 时修改。阶段 9 只有 `transparent + transparent` 已实现；
`phase_profile` 与 `aan_residual` 是为后续算法保留的静态接口，当前会明确显示
`STRATEGY_NOT_IMPLEMENTED`。UART 不提供 ACTIVE 参数，SHADOW 建议也不会进入 CAN。

示例：

```text
assist_config transparent shadow transparent 0 0 0
assist_status
```

### can_status — 显示 CAN 控制器状态
```
can_status
```
输出 TWAI 运行状态、TX/RX 队列、错误计数、发送失败、总线错误和仲裁丢失
计数。若 `TX error counter`/`Bus error count` 持续增加或进入
`BUS_OFF`，应检查收发器使能、CAN_H/CAN_L、共地、终端电阻和波特率。

### ble_status — 显示产品 BLE 传输状态

```text
ble_status
```

输出产品 BLE 是否初始化、Host 是否同步、手机是否连接并订阅 Response Notify、连接
代际、协商 MTU、BLE 命令任务栈余量，以及接收、排队、畸形帧、重复请求、请求号冲突、
队列满、过期请求、通知同步提交成功/失败和异步 TX 失败计数。该命令只读，不发送产品 BLE
命令，也不影响电机。

### motor_type — 查看电机型号参数
```
motor_type
```
显示阶段 2 RS01 驱动实际使用的 P/V/KP/KD/T 量程。

### help — 显示帮助
```
help
```
列出所有可用命令。

---

## 当前 RS01 量程

| 参数 | 最小值 | 最大值 |
|------|--------|--------|
| 位置 (P) | -720.21 deg | 720.21 deg |
| 速度 (V) | -2521.01 deg/s | 2521.01 deg/s |
| KP | 0.0 | 500.0 |
| KD | 0.0 | 5.0 |
| 力矩 (T) | -17.0 Nm | 17.0 Nm |

这些显示值由 RS01 手册的 `±12.57 rad`、`±44 rad/s` 换算而来；驱动内部仍保留
手册原单位并封装在 `rs01_driver.c`，不再使用通用 `MOTOR_TYPE` 宏。

---

## 典型操作流程

```text
# 1. 确认系统空闲，并查看 START 的阻止原因
system_status

# 2. 双髋保持中立位时，在 IDLE 中捕获 RAM 软件零位
joint_zero left
joint_zero right
system_status

# 3. 提交异步启动请求
start

# 4. 查询直到状态成为 RUNNING；可同时查看反馈和控制周期
system_status
joint_status left
joint_status right
control_status

# 5. 提交异步停止请求（输出许可立即撤销）
stop

# 6. 查询直到 IDLE，再确认电机反馈和 CAN 状态
system_status
motor A
status
motor B
status
can_status
```

若 `system_status` 显示 `ERROR`，不要使用低层维护命令绕过状态机；再次发送 `stop`
请求清理，并以最终系统状态和两侧新反馈为准。
