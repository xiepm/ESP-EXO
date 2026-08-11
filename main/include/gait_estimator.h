#ifndef GAIT_ESTIMATOR_H
#define GAIT_ESTIMATOR_H

#include <stdbool.h>
#include <stdint.h>

#include "exo_sensor_snapshot.h"

/*
 * 阶段 9 步态估计器
 * =================
 *
 * 当前实现是适合 ESP32 的 HIP_EVENT_TBE：从左右髋角度/速度检测髋屈曲和伸展
 * 极值，再用同侧重复事件之间的时间估计周期，并在两个事件之间单调推进连续相位。
 * 它借鉴文献中的“连续相位、周期边界复位、锁定/退化状态机”，但不冒充论文中的
 * 膝角 LSTM、足底着地检测或人体运动模式分类器。
 *
 * 所有时间都由调用者的快照传入，算法内部不读 esp_timer；因此同一回放输入必然
 * 得到同一输出。上下文固定大小、无动态内存，也不创建任何任务。
 */

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	EXO_GAIT_ESTIMATOR_NONE = 0,
	EXO_GAIT_ESTIMATOR_HIP_EVENT_TBE = 1,
	/* 以下ID只用于未来兼容，不代表当前固件已经实现对应算法。 */
	EXO_GAIT_ESTIMATOR_ADAPTIVE_OSCILLATOR = 2,
	EXO_GAIT_ESTIMATOR_PLANTAR_DURATION_HMM = 3,
	EXO_GAIT_ESTIMATOR_JOINT_LSTM = 4,
} exo_gait_estimator_id_t;

/* 这是估计器自己的生命周期，不应加入整机 INIT/IDLE/RUNNING 状态机。 */
typedef enum {
	EXO_GAIT_STATE_RESET = 0,
	EXO_GAIT_STATE_STANDING = 1,
	EXO_GAIT_STATE_ACQUIRING = 2,
	EXO_GAIT_STATE_LOCKED = 3,
	EXO_GAIT_STATE_DEGRADED = 4,
} exo_gait_estimator_state_t;

/* 当前编码器可直接观测的事件。HEEL_STRIKE/TOE_OFF故意不在本阶段定义。 */
typedef enum {
	EXO_GAIT_EVENT_NONE = 0,
	EXO_GAIT_EVENT_HIP_FLEXION_PEAK = 1U << 0,
	EXO_GAIT_EVENT_HIP_EXTENSION_PEAK = 1U << 1,
	EXO_GAIT_EVENT_PHASE_WRAP = 1U << 2,
} exo_gait_event_flag_t;

typedef enum {
	EXO_GAIT_PHASE_ORIGIN_UNKNOWN = 0,
	EXO_GAIT_PHASE_ORIGIN_HIP_FLEXION_PEAK = 1,
	EXO_GAIT_PHASE_ORIGIN_HIP_EXTENSION_PEAK = 2,
} exo_gait_phase_origin_t;

/* 左右相位和全局量分别带有效位，避免用0或NaN表达“没有结果”。 */
typedef enum {
	EXO_GAIT_VALID_NONE = 0,
	EXO_GAIT_VALID_LEFT_PHASE = 1U << 0,
	EXO_GAIT_VALID_RIGHT_PHASE = 1U << 1,
	EXO_GAIT_VALID_CADENCE = 1U << 2,
	EXO_GAIT_VALID_CONFIDENCE = 1U << 3,
} exo_gait_valid_flag_t;

/* 16位质量位会被BLE原样编码；新增位只能从保留高位向后扩展。 */
typedef enum {
	EXO_GAIT_QUALITY_NONE = 0,
	EXO_GAIT_QUALITY_LEFT_INPUT_INVALID = 1U << 0,
	EXO_GAIT_QUALITY_RIGHT_INPUT_INVALID = 1U << 1,
	EXO_GAIT_QUALITY_LOW_MOTION = 1U << 2,
	EXO_GAIT_QUALITY_LEFT_WAITING_EVENT = 1U << 3,
	EXO_GAIT_QUALITY_RIGHT_WAITING_EVENT = 1U << 4,
	EXO_GAIT_QUALITY_LEFT_PERIOD_OUT_OF_RANGE = 1U << 5,
	EXO_GAIT_QUALITY_RIGHT_PERIOD_OUT_OF_RANGE = 1U << 6,
	EXO_GAIT_QUALITY_PERIOD_UNSTABLE = 1U << 7,
	EXO_GAIT_QUALITY_PAIR_SKEW_HIGH = 1U << 8,
	EXO_GAIT_QUALITY_TIME_OR_SEQUENCE_ROLLBACK = 1U << 9,
	EXO_GAIT_QUALITY_COORDINATE_RESET = 1U << 10,
	EXO_GAIT_QUALITY_LEFT_PHASE_TIMEOUT = 1U << 11,
	EXO_GAIT_QUALITY_RIGHT_PHASE_TIMEOUT = 1U << 12,
	EXO_GAIT_QUALITY_SINGLE_SIDE_ONLY = 1U << 13,
	/* 两侧虽各自锁定，但周期差过大或相位长期接近同相；只降权，不硬编码0.5。 */
	EXO_GAIT_QUALITY_BILATERAL_INCONSISTENT = 1U << 14,
} exo_gait_quality_flag_t;

typedef struct {
	exo_gait_estimator_id_t estimator_id;
	exo_gait_estimator_state_t state;
	exo_gait_phase_origin_t phase_origin;
	uint32_t sequence;
	uint32_t source_snapshot_sequence;
	int64_t timestamp_us;
	uint32_t source_reset_epoch;
	uint32_t valid_flags;
	uint32_t quality_flags;
	uint16_t left_event_flags;
	uint16_t right_event_flags;
	float left_phase_01;
	float right_phase_01;
	float left_period_s;
	float right_period_s;
	float cadence_spm;
	float confidence_01;
	bool motion_detected;
	bool valid;
} exo_gait_estimate_t;

/* 参数全部带物理单位；当前为编译期只读配置，不通过BLE直接写入。 */
typedef struct {
	float filter_alpha;
	float trend_velocity_threshold_deg_s;
	float motion_velocity_threshold_deg_s;
	float minimum_excursion_deg;
	float minimum_period_s;
	float maximum_period_s;
	float minimum_event_interval_s;
	float period_ewma_alpha;
	float unstable_period_fraction;
	/* 左右周期相对差大于此值时降低置信度。 */
	float bilateral_period_tolerance_fraction;
	/* 左右循环相位距离小于此值时视作可疑同相，范围(0,0.5)。 */
	float minimum_bilateral_phase_separation;
	uint32_t standing_timeout_us;
	uint32_t phase_timeout_margin_us;
	uint8_t lock_period_count;
} exo_gait_estimator_config_t;

/* 单侧固定内存状态；字段公开只为静态分配和真实C代码主机回放。 */
typedef struct {
	bool filter_initialized;
	float filtered_position_deg;
	float filtered_velocity_deg_s;
	int8_t trend;
	float interval_min_deg;
	float interval_max_deg;
	int64_t last_event_us;
	int64_t last_origin_us;
	float period_s;
	float last_period_error_fraction;
	uint8_t accepted_period_count;
	bool phase_ready;
	/* 用反馈源签名去重；同一个异步反馈不得在多个100 Hz周期重复滤波。 */
	uint64_t last_source_timestamp_us;
	uint32_t last_source_feedback_count;
} exo_gait_side_state_t;

typedef struct {
	exo_gait_estimator_config_t config;
	exo_gait_side_state_t left;
	exo_gait_side_state_t right;
	uint32_t output_sequence;
	uint32_t last_snapshot_sequence;
	uint32_t last_reset_epoch;
	int64_t last_timestamp_us;
	int64_t last_motion_us;
	exo_gait_estimator_state_t state;
	bool initialized;
} exo_gait_estimator_t;

bool exo_gait_estimator_init(exo_gait_estimator_t *estimator,
				 const exo_gait_estimator_config_t *config);

/* 主动复位时保留配置，只清除滤波、事件、周期、相位和锁定历史。 */
void exo_gait_estimator_reset(exo_gait_estimator_t *estimator,
				  uint32_t source_reset_epoch);

/*
 * 单周期纯计算入口。返回false只代表参数/初始化错误；输入暂时无效仍返回true，并在
 * estimate中明确给出DEGRADED、valid=false和质量位。
 */
bool exo_gait_estimator_update(exo_gait_estimator_t *estimator,
				   const exo_sensor_snapshot_t *snapshot,
				   exo_gait_estimate_t *estimate);

const char *exo_gait_estimator_state_name(exo_gait_estimator_state_t state);
const char *exo_gait_phase_origin_name(exo_gait_phase_origin_t origin);

#ifdef __cplusplus
}
#endif

#endif /* GAIT_ESTIMATOR_H */
