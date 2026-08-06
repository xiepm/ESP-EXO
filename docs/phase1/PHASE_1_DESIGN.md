# 阶段 1：公共类型和配置

## 目标

阶段 1 只建立后续重构都能依赖的底层语言：

- 用带单位的公共类型表达执行器采样；
- 用逻辑槽 `A/B` 管理当前两台执行器；
- 用唯一的只读对象 `g_exo_hardware_config` 保存当前硬件事实；
- 让主程序、RS01 私有协议、BLE 控制台和现有阻抗示例都读取同一配置。

本阶段不建立髋关节对象、软件零位流程、状态机、产品 BLE 协议、统一控制任务或新控制模式。

## 依赖方向

```text
exo_types.h
    ↑
exo_config.h + exo_config.c
    ↑
main / robstride_control / impedance_control / ble_console
```

`exo_config` 不依赖任何上层业务模块，也不创建任务、定时器、锁、队列或动态内存。

## 公共类型

公共类型位于 `main/include/exo_types.h`：

| 类型 | 含义 |
|---|---|
| `exo_can_node_id_t` | CAN 节点 ID 的固件内部表示 |
| `exo_side_t` | `UNKNOWN/LEFT/RIGHT` 物理侧 |
| `exo_actuator_id_t` | 固件逻辑槽 `A/B`，可直接作为两元素数组下标 |
| `exo_direction_t` | 电机正方向到未来关节正方向的符号关系 |
| `exo_motion_sample_t` | 单位明确的 `rad`、`rad/s`、`N·m`、`°C` 采样 |

这些类型是固件内部 ABI，不是 BLE/CAN 线格式，禁止直接 `memcpy` 或 `packed` 后发送。

## 唯一配置对象

配置声明位于 `main/include/exo_config.h`，唯一实例位于 `main/exo_config.c`。

| 配置项 | 阶段 1 值 |
|---|---:|
| TWAI TX/RX | GPIO4 / GPIO2 |
| TWAI 速率 | 1,000,000 bit/s |
| TWAI RX 队列 | 32 帧 |
| 控制器 Master ID | `0xFD` |
| 执行器 A | CAN ID `0x10` |
| 执行器 B | CAN ID `0x20` |
| 默认控制台目标 | 执行器 A |
| 现有控制频率 | 100 Hz |
| BLE 名称 | `RobStride-CAN` |

GPIO 的构建值仍由原有 `CONFIG_EXAMPLE_TX_GPIO_NUM` 和
`CONFIG_EXAMPLE_RX_GPIO_NUM` 提供，以保持阶段 0 的 `sdkconfig` 不变；只有
`exo_config.c` 可以读取这两个 Kconfig 符号，其余模块只读公共配置对象。

当前不能从代码或电子反馈推断 `0x10/0x20` 对应人体左侧还是右侧，也不能推断机械安装正方向。因此两台执行器都保持：

- `physical_side = EXO_SIDE_UNKNOWN`；
- `direction = EXO_DIRECTION_UNKNOWN`；
- `software_zero_valid = false`。

阻抗示例启动时捕获的中心位置只是本次运行的参考中心，不是产品软件零位。

## 协议常量边界

`robstride_control.c` 内私有扩展帧的 Master ID 已改为读取公共配置。MIT 标准帧末字节中的 `0xFD` 是 RS01 协议命令字，语义不同，必须保留。

`MOTOR_TYPE=5`、RS01 量化范围和寄存器协议仍属于当前驱动实现，将在阶段 2 随 RS01 驱动重构处理。阻抗参数 `KP/KD/幅值/频率/渐入时间` 仍属于现有演示控制器，将在阶段 7 处理。

## 文件范围

```text
main/
├─ include/
│  ├─ exo_types.h
│  └─ exo_config.h
├─ exo_config.c
├─ twai_network_example_master_main.c
├─ robstride_control.c/.h
├─ impedance_control.c/.h
├─ ble_console.c
└─ CMakeLists.txt
```

阶段 1 不修改 `sdkconfig`、分区表、Kconfig 选项，也不新增 ESP-IDF 组件依赖。由于公共配置头公开使用 GPIO/TWAI 类型，已有的 `driver` 依赖从私有改为公共可见。

## 完成标准

1. 配置值只在 `exo_config.c` 定义，业务模块不再直接读取 GPIO Kconfig 或定义 A/B、Master ID、BLE 名称和 100 Hz 常量。
2. A/B 不被命名成 LEFT/RIGHT，方向和软件零位不被猜测。
3. 相对 `baseline-esp-can-v0` 不新增任务、定时器、队列、同步对象或动态分配调用点。
4. ESP-IDF v5.5 干净构建无 `warning:` 和 `error:`，镜像不超出 1 MiB 应用分区。
5. 烧录后完成 UART、BLE、TWAI、双执行器反馈和一次 10 秒阻抗启停回归。

实际验收证据记录在 `PHASE_1_ACCEPTANCE.md`。
