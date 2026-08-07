# 阶段 7：正弦参考与阻抗控制迁移设计

## 1. 目标和基线

阶段 7 基于 `813117d / phase6-product-ble-protocol`，把阶段 6 中直接使用
执行器 A/B 原始坐标的正弦阻抗示例，等价迁移为 LEFT/RIGHT 人体髋关节语义。

本阶段只改变算法组织和坐标边界，不改变运动波形、状态机、产品 BLE v1、RS01
协议或 100 Hz 调度方式。固件版本字段更新为 `0.7.0`，BLE 协议版本仍为 v1。

工程问题是：能否在不改变既有电机目标的前提下，把参考生成、关节坐标和硬件
发送职责分开？

- H0：迁移后的 A/B 目标与旧算法偏差超过 `1e-4 rad`，或引入新的周期源、状态、
  阻塞 I/O；
- H1：0～60 秒回放误差不超过 `1e-4 rad`，且阶段 4～6 的任务、状态和 BLE 合同
  保持不变。

## 2. 行为兼容合同

阶段 6 固定参数保持不变：

| 参数 | 值 |
|---|---:|
| 幅值 | `pi/2 rad` |
| 频率 | `0.5 Hz` |
| 五次渐入时间 | `3.0 s` |
| RS01 Motion KP | `0.2` |
| RS01 Motion KD | `0.10` |
| 目标速度 | `0 rad/s` |
| 前馈转矩 | `0 Nm` |

参考偏移为：

```text
s(t) = clamp(t / 3.0, 0, 1)
r(t) = 10s^3 - 15s^4 + 6s^5
delta(t) = (pi/2) * r(t) * sin(2*pi*0.5*t)
```

当前物理映射为 LEFT→B/`+1`、RIGHT→A/`-1`。因此关节空间和电机空间的
等价关系是：

```text
LEFT_ref  = LEFT_center  + delta
RIGHT_ref = RIGHT_center + delta

B_target = B_center + delta
A_target = A_center - delta
```

也就是说，LEFT/RIGHT 人体关节参考同相，A/B 原始电机目标反相。阶段 7 不把它
改成步态反相；左右相位曲线属于阶段 10。

## 3. 模块和对象

```text
唯一 100 Hz exo_control_loop
          │ now_us
          ▼
exo_system 输出门
          │ RUNNING && run_requested && output_allowed
          ▼
impedance_control
  ├─ arm/disarm/tick 生命周期和退出握手
  ├─ 保存 LEFT/RIGHT 启动中心
  ├─ 调用 sine_reference 取得 delta
  └─ 生成 LEFT/RIGHT 目标
          │
          ▼
hip_joint_to_actuator_position
  └─ software_zero + direction * joint_position
          │
          ▼
按 actuator A、B 顺序调用 rs01_try_send_motion
```

### 3.1 `sine_reference`

`sine_reference.c/.h` 是纯标准 C 数学模块：

- 输入只包含固定配置和有符号 `elapsed_us`；
- 输出位置偏移和当前 ramp；
- `elapsed_us <= 0` 输出零偏移；
- 在调用 `sinf()` 前折返完整周期，避免长时间运行时相位数不断增大；
- 不依赖 ESP-IDF、FreeRTOS、RS01、TWAI、BLE、日志或动态内存。

相位折返与旧公式数学等价，但单精度舍入顺序不同，因此必须用 0～60 秒、10 ms
步长回放证明误差小于合同阈值。

### 3.2 `hip_joint`

阶段 7 新增的窄接口只执行坐标转换：

```c
esp_err_t hip_joint_to_actuator_position(
    exo_hip_joint_id_t joint,
    float joint_position_rad,
    exo_actuator_id_t *actuator,
    float *actuator_position_rad);
```

它在关节锁内只复制静态对象状态，解锁后计算；要求映射、方向和 RAM 软件零位
有效。它不读取反馈、不发送 CAN，也不接收 KP/KD 或五元命令。

### 3.3 `impedance_control`

控制器使用具名中心结构，避免两个位置参数被交换：

```c
typedef struct {
    float center_left_rad;
    float center_right_rad;
} impedance_control_centers_t;
```

每个 tick 先生成两个关节目标，再通过髋关节对象解析到执行器数组，最后按 A、B
顺序发送。即使 A 发送失败，B 仍会尝试；坐标解析失败时不发送不完整的单侧目标。

`s_tick_active` 在计算或发送结果成功、失败时都通过同一个清理出口复位，保证 STOP
中的 disarm 不会永久等待。

## 4. START、周期和 STOP

START 硬件序列保持阶段 5 的顺序。取得 fresh A/B 反馈后，系统重新获取有效的
LEFT/RIGHT 快照，以关节位置建立控制中心，再 arm 控制器。随后仍由状态机发布
`RUNNING + output_allowed`。

这里有意使用 freshness 门通过后的最新关节快照，而不是把此前返回的 A/B 原始样本
重新复制到算法层。若设备在两次读取之间运动，中心采样相对阶段 6 最多偏移约一个反馈
更新窗口；数值等价合同是在相同中心条件下比较。运动验收在中立位建立软件零位并记录
实际 arm 后波形，避免把这一采样时刻差异误判成坐标错误。

周期路径仍然只有：

```text
exo_control_loop
→ exo_system_process_control_tick
→ impedance_control_process_tick
→ rs01_try_send_motion
```

STOP 仍在请求线性化点先清除 `run_requested/output_allowed`，执行器随后 disarm、
发送零增益 release、失能并确认 fresh REST 反馈。本阶段不改变这些锁和状态顺序。

## 5. 配置边界

固定控制配置独立于 `exo_hardware_config_t`，但与硬件配置一起由 `exo_config.c`
提供只读全局对象。阶段 7 不增加：

- BLE 参数修改命令；
- NVS 持久化；
- 配置 revision；
- 运行时模式切换。

RS01 所谓阻抗控制仍指电机 Motion 模式：ESP32 发送位置、速度、前馈转矩、KP、KD，
闭环由 RS01 执行；本阶段不宣称 ESP32 显式计算关节阻抗转矩。

## 6. 文件变化

| 文件 | 变化 |
|---|---|
| `main/include/sine_reference.h` | 新增纯参考类型和 API |
| `main/sine_reference.c` | 新增五次渐入和周期折返正弦 |
| `main/include/exo_config.h`、`main/exo_config.c` | 新增只读阶段 7 控制配置 |
| `main/include/hip_joint.h`、`main/hip_joint.c` | 新增关节位置反向转换 |
| `main/impedance_control.h/.c` | 迁移到 LEFT/RIGHT，并保留 A/B 发送顺序 |
| `main/exo_system.c` | START 使用 LEFT/RIGHT 快照 arm |
| `main/exo_ble.c` | 固件版本字段更新为 `0.7.0` |
| `main/CMakeLists.txt` | 注册纯参考源文件 |

`app_main.c`、`exo_control_loop.c`、`exo_ble_protocol.c/.h` 和 `rs01_driver.c/.h`
不需要改变。

## 7. 实时和资源约束

- 全工程仍只能有一个 `xTaskDelayUntil()`；
- 不创建新的任务、队列、定时器或动态对象；
- 周期发送继续使用零等待 `rs01_try_send_motion()`；
- 参考模块不日志、不阻塞；
- double 相位折返带来的实际执行时间必须通过实机 100 Hz 指标验证，不能只用构建
  通过代替实时验收。

## 8. 阶段边界

以下内容不属于阶段 7：

- 公共五元关节命令和基础控制模式（阶段 8）；
- 统一传感快照和步态估计（阶段 9）；
- 左右相位曲线、论文轨迹和连续遥测（阶段 10）；
- 产品清理和整体回归（阶段 11）。

阶段 7 的完成条件不是“代码能够编译”，而是数值等价、静态边界、干净构建、
无运动门控和三轮实机 START/RUNNING/STOP 全部通过。
