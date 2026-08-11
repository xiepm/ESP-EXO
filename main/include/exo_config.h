#ifndef EXO_CONFIG_H
#define EXO_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/twai.h"
#include "exo_sensor_snapshot.h"
#include "exo_types.h"
#include "gait_estimator.h"
#include "sine_reference.h"

/*
 * 本文件只声明“配置长什么样”，具体取值统一放在 exo_config.c。
 * 这样驱动、关节、控制器和 BLE 都读取同一个只读配置源，不会出现某个文件认为
 * 左腿是 A、另一个文件却认为左腿是 B 的情况。
 */

typedef struct {
	/* TWAI 就是 ESP32 的 CAN 控制器；引脚连接的是外部 CAN 收发器逻辑侧。 */
	gpio_num_t tx_gpio;
	gpio_num_t rx_gpio;
	/* 位率单位为 bit/s；timing 是 ESP-IDF 按相同位率生成的底层时序参数。 */
	uint32_t bitrate_bps;
	twai_timing_config_t timing;
	/* 软件接收队列可暂存的 CAN 帧数量，不是单帧字节数。 */
	uint32_t rx_queue_length;
	/* ESP32 作为主站发送私有协议命令时使用的节点身份。 */
	exo_can_node_id_t master_id;
} exo_twai_config_t;

typedef struct {
	/* 执行器槽位 A/B 对应的 RS01 CAN 节点号；这里只描述电机，不描述左右腿。 */
	exo_can_node_id_t can_id;
} exo_actuator_config_t;

/*
 * 一侧人体髋关节的静态安装配置。
 * software_zero_deg 是电机输出轴的 RAM 软件零位，采用度，仅在明确标记有效时
 * 才能使用；它不是写入电机的机械零位。
 */
typedef struct {
	/* 人体侧别通常与数组下标一致，保留字段是为了让配置能被完整校验。 */
	exo_side_t side;
	/* 该侧髋关节实际连接哪个逻辑执行器槽位。 */
	exo_actuator_id_t actuator;
	/* 电机坐标到“髋屈曲为正”人体坐标的 +1/-1 符号。 */
	exo_direction_t direction;
	/* 电机输出轴在人体中立位时的位置，单位 deg。 */
	float software_zero_deg;
	/* 三个 valid 标志让“值是 0”和“值尚未标定”成为不同状态。 */
	bool mapping_valid;
	bool direction_valid;
	bool software_zero_valid;
} exo_hip_joint_config_t;

typedef struct {
	/* 所有产品硬件参数的唯一只读聚合对象。时间单位在字段名中显式标出。 */
	exo_twai_config_t twai;
	/* 两个执行器槽位和两个人体髋关节分别建表，再通过 actuator 字段建立映射。 */
	exo_actuator_config_t actuator[EXO_ACTUATOR_COUNT];
	exo_hip_joint_config_t hip_joint[EXO_HIP_JOINT_COUNT];
	/* 工程串口上电后默认选中的电机，仅影响手工调试命令。 */
	exo_actuator_id_t default_actuator;
	/* 控制频率为 Hz；主动上报和超时为 ms，字段名即单位。 */
	uint32_t control_frequency_hz;
	uint32_t rs01_report_interval_ms;
	uint32_t rs01_feedback_timeout_ms;
	const char *ble_device_name;
} exo_hardware_config_t;

typedef struct {
	/* reference 只描述轨迹，kp/kd 描述电机 Motion 模式的阻抗参数。 */
	sine_reference_config_t reference;
	float kp;
	float kd;
} exo_sine_impedance_config_t;

/*
 * 固件运行期唯一可读的硬件配置对象。const 表示业务模块只读；若要修改产品
 * 硬件映射，应改 exo_config.c 后重新编译，不能在运行期偷偷覆盖。
 */
extern const exo_hardware_config_t g_exo_hardware_config;

/*
 * 阶段 7 正弦参考及阶段 8 默认 KP/KD。阶段 8 的 selected 五元可在 IDLE 通过
 * BLE/UART 修改，但正弦幅值/频率/ramp 仍是编译期只读值，且不写入 NVS。
 */
extern const exo_sine_impedance_config_t g_exo_sine_impedance_config;

/*
 * 阶段 9 的传感时效和步态算法参数仍采用编译期只读配置。这样主机回放、真机固件
 * 和验收文档能使用同一组有版本的数值；手机不能在未验证的情况下在线改滤波器或
 * 事件阈值。助力等级属于产品运行配置，由 exo_assist 单独管理。
 */
extern const exo_sensor_snapshot_config_t g_exo_sensor_snapshot_config;
extern const exo_gait_estimator_config_t g_exo_gait_estimator_config;

#endif /* EXO_CONFIG_H */
