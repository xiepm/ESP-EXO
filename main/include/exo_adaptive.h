#ifndef EXO_ADAPTIVE_H
#define EXO_ADAPTIVE_H
#include <stdbool.h>
#include <stdint.h>
/* 纯算法接口：人体屈髋为正，角度deg、速度deg/s、力矩N·m。
 * 不读时钟、不依赖RTOS/CAN，不分配内存。下标0=LEFT、1=RIGHT。 */
#define EXO_ADAPTIVE_SCHEMA 1U
#define EXO_AFO_HARMONICS 3U
#define EXO_ADAPTIVE_CONFIG_WORDS 49U
#define EXO_ADAPTIVE_CONFIG_BYTES (EXO_ADAPTIVE_CONFIG_WORDS * 4U)
#define EXO_ADAPTIVE_PI 3.14159265358979323846f

/* 只读锁相判据位；与故障/重新授权位分开，不改变任何保护阈值。 */
typedef enum {
    EXO_ADAPTIVE_LOCK_INPUT=1U<<0, EXO_ADAPTIVE_LOCK_TIME=1U<<1,
    EXO_ADAPTIVE_LOCK_STOPPED=1U<<2, EXO_ADAPTIVE_LOCK_INTERVALS=1U<<3,
    EXO_ADAPTIVE_LOCK_EVENT_AGE=1U<<4, EXO_ADAPTIVE_LOCK_FREQUENCY=1U<<5,
    EXO_ADAPTIVE_LOCK_PHASE_ERROR=1U<<6, EXO_ADAPTIVE_LOCK_RESIDUAL=1U<<7,
    EXO_ADAPTIVE_LOCK_EMERGENCY=1U<<8
} exo_adaptive_lock_reason_t;

typedef struct {
    float minimum_hz, maximum_hz, initial_hz;
    float phase_gain, frequency_gain, amplitude_gain, input_scale_deg;
    float maximum_dt_s, velocity_filter_tau_s;
    float event_velocity_deg_s, event_excursion_deg, correction_rate_rad_s;
} exo_afo_config_t;
typedef struct {
    float phase_rad, omega_rad_s, cosine[4], sine[4], residual_ewma;
    bool initialized;
} exo_afo_t;
typedef struct {
    float filtered_velocity, low_deg, peak_deg, period_s, confidence;
    float target_offset_rad, offset_rad, phase_before, phase_after;
    float last_peak_deg, last_low_deg;
    float reference_phase_error_rad;
    int64_t peak_us, previous_event_us, last_event_us;
    uint32_t intervals, rejected, events;
    /* phase_error取本周期offset更新前误差、最近峰值核对误差、
     * 校正相位相对同侧周期时钟误差中绝对值最大的有符号值。 */
    uint32_t lock_reasons, lock_intervals;
    float lock_phase_error_rad, lock_frequency_error, lock_event_age_s;
    int trend;
    bool initialized, event, locked;
    /* Walking observer: causal angle derivative, no future samples. */
    float previous_angle_deg;
    int64_t penultimate_event_us;
    /* Pending trough is not a phase reference until its peak is accepted. */
    float candidate_low_deg;
} exo_adaptive_event_t;
typedef struct {
    /* enabled严格为0或1；全局强度与左右强度均为0..1。以下全是配置快照值。 */
    float enabled, strength;
    float flex_amplitude_nm, flex_peak_phase, flex_width;
    float ext_amplitude_nm, ext_peak_phase, ext_width;
    float torque_limit_nm, slew_nm_s, positive_power_w, negative_power_w;
    float minimum_angle_deg, maximum_angle_deg, maximum_velocity_deg_s;
    float fade_s, transition_s;
} exo_adaptive_leg_config_t;
typedef struct {
    uint32_t schema;
    float overall_strength;
    exo_afo_config_t afo;
    exo_adaptive_leg_config_t leg[2];
} exo_adaptive_config_t;
typedef enum {
    EXO_ADAPTIVE_FAULT_NONE=0,
    EXO_ADAPTIVE_FAULT_INPUT=1U<<0, EXO_ADAPTIVE_FAULT_TIME=1U<<1,
    EXO_ADAPTIVE_FAULT_RANGE=1U<<2, EXO_ADAPTIVE_FAULT_PHASE=1U<<3,
    EXO_ADAPTIVE_FAULT_STOP=1U<<4, EXO_ADAPTIVE_FAULT_REARM=1U<<5,
    EXO_ADAPTIVE_FAULT_CONFIG=1U<<6
} exo_adaptive_fault_t;
typedef struct {
    int64_t timestamp_us;
    uint32_t sequence, coordinate_epoch;
    uint64_t source_timestamp_us[2]; /* 0表示主机输入与采样时刻相同；真机保留异步源时间。 */
    float position_deg[2], velocity_deg_s[2], feedback_torque_nm[2];
    bool valid[2], feedback_torque_valid[2], emergency;
    /* 0禁止；只有显式START/调试授权可改变为新的非零代次，重锁定不改变此值。 */
    uint32_t permit_epoch;
} exo_adaptive_input_t;
typedef struct {
    float raw_phase, corrected_phase, frequency_hz, period_s, confidence;
    float flex_nm, ext_nm, desired_nm, limited_nm, lambda;
    float profile_nm; /* Read-only diagnostic before walking direction gating. */
    float event_phase_before, event_phase_after, phase_offset_rad;
    uint32_t faults, accepted_events, rejected_events;
    uint32_t lock_reasons, lock_intervals;
    float lock_phase_error_rad, lock_frequency_error, lock_event_age_s, afo_residual;
    int64_t event_timestamp_us;
    bool event, locked, rearm_required;
} exo_adaptive_leg_output_t;
typedef struct {
    int64_t timestamp_us;
    uint32_t sequence, config_generation;
    exo_adaptive_leg_output_t leg[2];
} exo_adaptive_output_t;
typedef struct {
    exo_afo_t afo[2];
    exo_adaptive_event_t event[2];
    exo_adaptive_config_t config;
    float previous_torque[2], lambda[2], previous_raw_phase[2];
    float transition_from[2], transition_elapsed[2], previous_desired[2];
    int64_t last_motion_us[2];
    uint32_t arm_event_counts[2];
    uint64_t source_timestamp_us[2];
    float source_position_deg[2], source_velocity_deg_s[2];
    bool was_locked[2], rearm_required[2], initialized;
    bool acquisition_seeded[2];
    /* Set once by the product adapter after init; never a wire parameter.
     * Phase readiness and an operating-window pause may reacquire within the
     * existing permit. Input/time/emergency/configuration faults never do. */
    bool walking_mode;
    int64_t velocity_pause_since_us[2];
    int64_t phase_pause_since_us[2];
    uint32_t permit_epoch, coordinate_epoch, sequence, config_generation;
    int64_t timestamp_us;
} exo_adaptive_engine_t;

exo_adaptive_config_t exo_adaptive_default_config(void);
exo_adaptive_config_t exo_adaptive_walking_preset(void);
exo_adaptive_config_t exo_adaptive_walking_preset_for_peak(float peak_nm);
/* Explicit profile 1 for the bounded manual trial; never applied at boot. */
exo_adaptive_config_t exo_adaptive_trial_preset(void);
bool exo_adaptive_trial_preset_matches(const exo_adaptive_config_t *c);
bool exo_adaptive_config_valid(const exo_adaptive_config_t *c);
bool exo_adaptive_config_encode(const exo_adaptive_config_t *c, uint8_t *b, uint32_t size);
bool exo_adaptive_config_decode(exo_adaptive_config_t *c, const uint8_t *b, uint32_t size);
void exo_afo_reset(exo_afo_t *s, const exo_afo_config_t *c, float q_deg);
bool exo_afo_update(exo_afo_t *s, const exo_afo_config_t *c, float q_deg, float dt_s);
void exo_adaptive_event_reset(exo_adaptive_event_t *s);
void exo_adaptive_event_update(exo_adaptive_event_t *s, const exo_afo_config_t *c,
    float q_deg, float velocity_deg_s, float raw_phase_rad, float omega_rad_s,
    float dt_s, int64_t timestamp_us);
void exo_adaptive_walking_event_update(exo_adaptive_event_t *s,
    const exo_afo_config_t *c, float q_deg, float dt_s, int64_t timestamp_us);
float exo_adaptive_wrap(float radians);
float exo_adaptive_circular_error(float radians);
/* von Mises形式的周期高斯：exp((cos(2π(s-μ))-1)/(2πσ)^2)。
 * s=0和1处函数及所有阶导数一致，μ处峰值1，σ以周期比例表示。 */
float exo_adaptive_gaussian(float phase, float peak, float width);
bool exo_adaptive_profile(const exo_adaptive_leg_config_t *c, float phase,
    float strength, float lambda, float *flex_nm, float *ext_nm, float *desired_nm);
float exo_adaptive_limit(const exo_adaptive_leg_config_t *c, float desired_nm,
    float previous_nm, float q_deg, float velocity_deg_s, float dt_s, uint32_t *faults);
bool exo_adaptive_engine_init(exo_adaptive_engine_t *s, const exo_adaptive_config_t *c);
/* 只由算法持有者更新整份配置。线程安全发布由平台adapter完成。 */
bool exo_adaptive_engine_configure(exo_adaptive_engine_t *s, const exo_adaptive_config_t *c, uint32_t generation);
bool exo_adaptive_engine_update(exo_adaptive_engine_t *s, const exo_adaptive_input_t *in, exo_adaptive_output_t *out);
#endif
