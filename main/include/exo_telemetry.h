#ifndef EXO_TELEMETRY_H
#define EXO_TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>

#include "exo_types.h"

/*
 * 阶段 10 实时遥测环形队列
 * ========================
 *
 * 唯一 100 Hz 控制任务只调用 exo_telemetry_push()：函数只做有界的
 * 定点量化和一次静态环形队列写入，不分配内存、不等待队列、不调用 BLE，
 * 也不记日志。BLE 低优先级任务通过 exo_telemetry_pop() 消费。
 *
 * 队列满时覆盖最旧样本，以保证实时曲线尽量接近“现在”；被覆盖样本和
 * BLE 已取出但未能完整发送的样本都计入 dropped_sample_count。
 */

#ifdef __cplusplus
extern "C" {
#endif

#define EXO_TELEMETRY_SOURCE_FREQUENCY_HZ 100U
#define EXO_TELEMETRY_DEFAULT_RATE_HZ 20U
#define EXO_TELEMETRY_RING_CAPACITY 32U

/* 一个 Notify 只携带一个单 bit 通道的左右值；同样本的多通道共用序号。 */
typedef enum {
	EXO_TELEMETRY_CHANNEL_JOINT_POSITION = 1U << 0,
	EXO_TELEMETRY_CHANNEL_JOINT_VELOCITY = 1U << 1,
	EXO_TELEMETRY_CHANNEL_MOTOR_TORQUE = 1U << 2,
	EXO_TELEMETRY_CHANNEL_GAIT_PHASE = 1U << 3,
	EXO_TELEMETRY_CHANNEL_ASSIST_SUGGESTION = 1U << 4,
	EXO_TELEMETRY_CHANNEL_GAIT_CONFIDENCE = 1U << 5,
	EXO_TELEMETRY_CHANNEL_APPLIED_TORQUE = 1U << 6,
} exo_telemetry_channel_t;

#define EXO_TELEMETRY_CHANNEL_MASK_ALL 0x007FU
#define EXO_TELEMETRY_DEFAULT_CHANNEL_MASK                                     \
	((uint16_t)(EXO_TELEMETRY_CHANNEL_GAIT_PHASE |                             \
				EXO_TELEMETRY_CHANNEL_ASSIST_SUGGESTION |                      \
				EXO_TELEMETRY_CHANNEL_GAIT_CONFIDENCE |                        \
				EXO_TELEMETRY_CHANNEL_APPLIED_TORQUE))

enum {
	EXO_TELEMETRY_VALUE_LEFT_VALID = 1U << 0,
	EXO_TELEMETRY_VALUE_RIGHT_VALID = 1U << 1,
};

/* input.sample_flags 只接收这两个同周期终态位，其余线位由编码器生成。 */
enum {
	EXO_TELEMETRY_SAMPLE_FLAG_OUTPUT_ALLOWED = 1U << 2,
	EXO_TELEMETRY_SAMPLE_FLAG_REAL_OUTPUT_ENABLED = 1U << 3,
	EXO_TELEMETRY_SAMPLE_INPUT_FLAGS_MASK = 0x0CU,
};

/* 控制链使用人体关节物理单位填入，push 在模块内统一量化。 */
typedef struct {
	float left;
	float right;
	bool left_valid;
	bool right_valid;
} exo_telemetry_pair_input_t;

typedef struct {
	/* 启动后单调时钟，单位 us；线上发送其 ms 低 32 位。 */
	int64_t timestamp_us;
	/* 不在遥测模块内重读状态机；由同一控制周期一次性填入终态。 */
	uint8_t system_state;
	uint8_t sample_flags;
	uint32_t activation_epoch;
	/* deg / deg/s / N·m / [0,1] / N·m / N·m。 */
	exo_telemetry_pair_input_t joint_position_deg;
	exo_telemetry_pair_input_t joint_velocity_deg_s;
	exo_telemetry_pair_input_t motor_torque_nm;
	exo_telemetry_pair_input_t gait_phase_01;
	exo_telemetry_pair_input_t assist_suggestion_nm;
	float gait_confidence_01;
	bool gait_confidence_valid;
	/* 实际交给输出门的最终五元命令，用于与同周期遥测绑定。 */
	exo_joint_command_pair_t final_command;
	bool final_left_valid;
	bool final_right_valid;
	/* 策略实际加入最终命令的助力增量，不能用 final torque_ff 冒充。 */
	exo_telemetry_pair_input_t applied_assist_nm;
} exo_telemetry_input_t;

/* 环形队列中保存的定点样本；不是 C 结构体线格式，禁止直接 memcpy 到 BLE。 */
typedef struct {
	uint16_t stream_sequence;
	uint32_t sample_sequence;
	uint32_t timestamp_ms;
	uint8_t system_state;
	uint8_t sample_flags;
	uint32_t activation_epoch;
	uint16_t left_valid_channel_mask;
	uint16_t right_valid_channel_mask;
	int16_t left_position_mrad;
	int16_t right_position_mrad;
	int16_t left_velocity_x100_rad_s;
	int16_t right_velocity_x100_rad_s;
	int16_t left_motor_torque_mnm;
	int16_t right_motor_torque_mnm;
	uint16_t left_phase_x10000;
	uint16_t right_phase_x10000;
	int16_t left_suggestion_mnm;
	int16_t right_suggestion_mnm;
	uint16_t confidence_x10000;
	int16_t left_applied_mnm;
	int16_t right_applied_mnm;
	/* 保留同周期最终命令，但不把整个 C 结构直接序列化到 BLE。 */
	exo_joint_command_pair_t final_command;
	bool final_left_valid;
	bool final_right_valid;
} exo_telemetry_sample_t;

typedef struct {
	bool initialized;
	bool streaming;
	uint8_t rate_hz;
	uint16_t channel_mask;
	uint16_t stream_sequence;
	uint32_t last_sample_sequence;
	uint32_t notify_success_count;
	uint32_t dropped_sample_count;
	uint8_t queued_samples;
	uint8_t queue_capacity;
} exo_telemetry_status_t;

/* 幂等纯内存初始化；不创建任务、不启动 BLE。 */
bool exo_telemetry_init(void);

/* 仅接受 5/10/20 Hz 及非空的已知通道掩码。每次成功 START 开启新 stream。 */
bool exo_telemetry_start(uint8_t rate_hz, uint16_t channel_mask,
						 uint16_t *stream_sequence);

/* 返回调用前是否正在流式输出；STOP 会丢弃队列中未发样本并计数。 */
bool exo_telemetry_stop(void);

/* 100 Hz 路径的唯一入口；未 START 或未到分频点时立即返回。 */
void exo_telemetry_push(const exo_telemetry_input_t *input);

/* BLE 低优先级任务的非阻塞消费入口。 */
bool exo_telemetry_pop(exo_telemetry_sample_t *sample);

/* 记录指定 stream 中已出队但没有完整送达的一个样本。 */
void exo_telemetry_note_transport_drop(uint16_t stream_sequence);

/* 每个被 NimBLE 接收的 Telemetry Notify 调用一次，计数单位是帧而非样本。 */
void exo_telemetry_note_notify_success(uint16_t stream_sequence);

/* 复制自洽诊断快照，不改变流状态。 */
bool exo_telemetry_get_status(exo_telemetry_status_t *status);

/* 为单通道 Notify 取出左右 16-bit 线值和有效位。 */
bool exo_telemetry_get_channel_values(const exo_telemetry_sample_t *sample,
									  uint16_t channel, uint16_t *left_value,
									  uint16_t *right_value,
									  uint8_t *valid_flags);

#ifdef __cplusplus
}
#endif

#endif /* EXO_TELEMETRY_H */
