# 阶段 3：髋关节对象和软件零位

## 目标

阶段 3 在阶段 2 的双 RS01 执行器驱动之上建立最小的人体髋关节层，使后续控制代码能够使用 `LEFT/RIGHT`，同时保持 CAN、RS01 协议和维护命令继续使用逻辑执行器 `A/B`。

本阶段完成以下内容：

- 定义左右髋关节标识；
- 显式配置 `LEFT/RIGHT -> A/B` 映射和安装方向；
- 建立两个固定大小的静态髋关节对象；
- 将 RS01 输出侧反馈转换为关节坐标；
- 提供显式、RAM-only 的软件零位建立和清除接口；
- 提供 UART/BLE 共用的髋关节调试命令；
- 保持阶段 2 的驱动、主动反馈和正弦阻抗行为不变。

阶段 3 不猜测真实左右映射。工程最初保持 `UNMAPPED/UNKNOWN`，随后通过两侧独立屈伸采集确认物理关系。当前配置采用实测结果：LEFT 映射到 B（CAN `0x20`）且方向 `+1`，RIGHT 映射到 A（CAN `0x10`）且方向 `-1`；软件零位仍必须在每次启动后显式建立。

## 分层和命名

### 执行器层

`EXO_ACTUATOR_A` 和 `EXO_ACTUATOR_B` 是 ESP32 内部的固定逻辑槽，目前分别使用 CAN ID `0x10` 和 `0x20`。A/B 不代表人体左侧或右侧。

RS01 驱动的所有接口继续接收 `exo_actuator_id_t`。CAN ID、RS01 功能码、帧布局、主动上报和机械置零都只属于驱动层。

### 髋关节层

`EXO_HIP_JOINT_LEFT` 和 `EXO_HIP_JOINT_RIGHT` 是人体域对象。髋关节层通过配置显式选择其数据来源是执行器 A 还是 B，不根据 CAN ID 大小或代码顺序推断左右。

关节正方向约定为髋屈曲为正。实际安装方向必须通过手动移动对应髋关节并观察原始反馈确定。

## 公共类型和配置

`exo_types.h` 增加：

```c
typedef enum {
    EXO_HIP_JOINT_LEFT = 0,
    EXO_HIP_JOINT_RIGHT = 1,
    EXO_HIP_JOINT_COUNT = 2,
} exo_hip_joint_id_t;
```

执行器配置只保留执行器自身属性：

```c
typedef struct {
    exo_can_node_id_t can_id;
} exo_actuator_config_t;
```

人体侧属性进入独立的髋关节配置：

```c
typedef struct {
    exo_side_t side;
    exo_actuator_id_t actuator;
    exo_direction_t direction;
    float software_zero_rad;
    bool mapping_valid;
    bool direction_valid;
    bool software_zero_valid;
} exo_hip_joint_config_t;
```

当前实物标定配置为：

| 关节 | 执行器 | 映射 | 方向 | 软件零位 |
|---|---|---|---|---|
| LEFT | `EXO_ACTUATOR_B`（CAN `0x20`） | 有效 | `+1` | 启动时无效，RAM 捕获后有效 |
| RIGHT | `EXO_ACTUATOR_A`（CAN `0x10`） | 有效 | `-1` | 启动时无效，RAM 捕获后有效 |

使用哨兵值 `EXO_ACTUATOR_COUNT` 表示未映射。配置中的有效标志不能与字段内容矛盾：有效映射必须选择 A 或 B；有效方向必须是 `+1` 或 `-1`；两侧均有效时不能指向同一执行器。

## 静态对象模型

`hip_joint.c` 内部只保存两个静态对象：

```c
static hip_joint_t s_joints[EXO_HIP_JOINT_COUNT];
```

每个对象保存：

- 关节标识和人体侧；
- 映射的执行器槽；
- 安装方向；
- 当前软件零位；
- 建立零位时所用反馈的时间戳和计数；
- 映射、方向和零位的独立有效标志。

对象状态通过一个静态 `portMUX` 保护。阶段 3 不创建新任务、软件定时器、队列或动态内存对象。

`hip_joint_init()` 从唯一的 `g_exo_hardware_config` 初始化对象，并检查：

- LEFT/RIGHT 条目的 `side` 与数组索引一致；
- 映射有效标志与执行器值一致；
- 方向有效标志与方向值一致；
- 软件零位是有限浮点数；
- 有效软件零位不能属于未映射关节；
- 两个有效关节不能映射到同一个执行器。

初始化只读取配置，不读取实时反馈，也不会自动建立零位。

## 反馈快照和坐标变换

`hip_joint_get_snapshot()` 先复制关节状态，再通过一次 `rs01_get_feedback()` 取得一致的执行器快照。对外快照同时保留：

- 原始 RS01 输出侧位置、速度、估算转矩和温度；
- 换算后的关节位置、速度和转矩；
- 反馈时间戳和反馈计数；
- 建立零位时的来源时间戳和来源计数；
- 映射、方向、零位、反馈、在线和最终关节有效状态。

坐标变换为：

```text
q_joint   = direction * (q_motor - software_zero_motor)
dq_joint  = direction * dq_motor
tau_joint = direction * tau_motor
```

温度不改变符号。位置不执行 `2π` 包络，避免隐藏 RS01 多圈输出和软件零位之间的真实差值。

RS01 的转矩是电机输出侧估算值，不等同于人体净髋关节力矩。阶段 3 只统一符号，不进行人体动力学补偿。

只有以下条件全部成立时 `joint_valid=true`：

```text
mapping_valid
&& direction_valid
&& zero_valid
&& feedback_valid
&& online
&& raw_sample_finite
```

当反馈已经离线时，快照中可能仍保留最后一次转换值；控制层必须检查 `joint_valid`，不能只读取数值字段。

## 软件零位

`hip_joint_set_zero_from_current()` 只执行以下动作：

1. 检查关节已经完成有效映射；
2. 取得对应执行器的一次反馈快照；
3. 要求反馈有效、在线且位置为有限值；
4. 将当前 `position_rad` 保存为该关节的 RAM 软件偏置；
5. 同时记录来源反馈的时间戳和计数并置 `zero_valid=true`。

建立软件零位不要求方向已经有效，但在方向有效之前 `joint_valid` 仍为 false。

`hip_joint_clear_zero()` 只清除指定关节的 RAM 偏置、来源信息和有效标志，不影响另一侧。

软件零位不执行以下操作：

- 不调用 `rs01_set_mechanical_zero()` 或 `rs01_mit_set_zero()`；
- 不发送 CAN 帧；
- 不使能、失能或移动电机；
- 不写入 RS01 参数或电机 Flash；
- 不写入 ESP32 NVS。

因此本阶段运行时建立的零位在 ESP32 重启后失效。持久化及其版本管理不属于阶段 3。

## 调试接口

UART 和 BLE NUS 使用同一命令表：

```text
joint_status <left|right>
joint_zero <left|right>
joint_zero_clear <left|right>
```

也接受 `l/r` 简写。`joint_status` 输出映射、方向、软件零位、反馈状态、原始执行器量、关节量以及零位来源。

必须区分：

- `joint_zero left/right`：只修改 ESP32 RAM 中的软件偏置；
- `set_zero`：向当前 RS01 发送私有协议机械置零命令；
- `mit_set_zero`：向当前 RS01 发送 MIT 机械置零命令。

控制台使用完整命令名精确匹配，`joint_zero` 不会匹配到 `set_zero`。阶段 3 的标定和验收流程不得使用后两条机械置零命令。

## 启动和运行方式

主程序保持原文件名和阶段 2 的整体结构，只在 RS01 驱动初始化之后增加：

```c
ESP_ERROR_CHECK(hip_joint_init());
```

髋关节对象采用按需读取，不建立“关节更新任务”。TWAI RX 任务仍只把 CAN 帧交给 RS01 驱动；阶段 4 才建立唯一的 100 Hz 控制任务并让控制层消费 LEFT/RIGHT 快照。

原有 `impedance_control.c` 继续直接使用 A/B 原始执行器坐标，以便阶段 3 仍可对阶段 2 行为做等价回归。

## 实物映射流程

完整标定需要两次固件配置循环：

1. 使用当前未映射固件，在两台电机失能且主动反馈开启时采集 A/B 原始角度；
2. 单独手动移动左髋，判断哪一个执行器反馈发生显著变化；
3. 单独手动移动右髋，确认结果与左侧互补；
4. 以髋屈曲为正，分别确定两侧方向符号；
5. 把实际映射、方向和有效标志写入 `exo_config.c`；
6. 重新构建、烧录并用 `joint_status` 验证；
7. 在定义好的中立姿态下分别执行 `joint_zero left` 和 `joint_zero right`。

CAN ID、静止角度和电机安装外观都不能单独证明人体左右关系。

## 文件范围

```text
main/
|- include/exo_types.h
|- include/exo_config.h
|- exo_config.c
|- include/hip_joint.h
|- hip_joint.c
|- ble_console.c
|- twai_network_example_master_main.c
`- CMakeLists.txt

docs/phase3/
|- PHASE_3_DESIGN.md
`- PHASE_3_ACCEPTANCE.md
```

阶段 3 不修改 `sdkconfig`、Kconfig、分区表、RS01 协议实现或现有阻抗参数。

## 完成标准

1. 驱动层始终使用 A/B，髋关节层始终使用 LEFT/RIGHT，映射只来自统一配置。
2. 两个静态关节对象能提供一致的原始量、关节量、时间戳和有效状态。
3. 坐标变换、方向和软件零位定义明确，无角度包络。
4. 软件零位只能由显式命令建立，离线或无映射时必须拒绝。
5. 软件零位不发送 CAN、不修改机械零位、不持久化，左右侧互不影响。
6. 不新增任务、软件定时器、队列或动态内存分配。
7. ESP-IDF v5.5 干净构建无 warning/error，镜像不超分区。
8. 真实左右映射、方向、两侧软件零位和原始角度连续性获得实机证据。
9. UART、BLE、TWAI、主动反馈和 3×10 s 阻抗回归通过。

只有以上条件全部满足，阶段 3 才能关闭并创建阶段标签。
