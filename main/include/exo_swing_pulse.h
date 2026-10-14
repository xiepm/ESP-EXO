#ifndef EXO_SWING_PULSE_H
#define EXO_SWING_PULSE_H

#include <stdbool.h>
#include <stdint.h>

/* 单侧事件脉冲。纯算法，无RTOS/CAN/堆；由唯一100 Hz任务调用。
 * 人体前抬为正，返回人体坐标下的正向力矩。电机方向s由原关节出口乘一次。
 * u是这一次脉冲进度，绝不是完整步态相位。 */
typedef enum { EXO_PULSE_FIXED = 0, EXO_PULSE_PERIOD_SCALED = 1 } exo_pulse_duration_mode_t;
typedef enum {
    EXO_PULSE_WAIT_RESET = 0, EXO_PULSE_READY = 1,
    EXO_PULSE_CONFIRM_EVENT = 2, EXO_PULSE_ACTIVE = 3,
    EXO_PULSE_WAIT_NEXT_RESET = 4,
} exo_pulse_state_t;
typedef enum {
    EXO_PULSE_EXIT_NONE = 0, EXO_PULSE_EXIT_COMPLETE = 1,
    EXO_PULSE_EXIT_REVERSE = 2, EXO_PULSE_EXIT_WINDOW = 3,
    EXO_PULSE_EXIT_EVENT_REJECTED = 4, EXO_PULSE_EXIT_PERIOD_UNTRUSTED = 5,
    EXO_PULSE_EXIT_FEEDBACK = 6,
    EXO_PULSE_EXIT_MOTION_STOPPED = 7,
} exo_pulse_exit_t;

/* 两个独立、短小的配置页面；timing可原子更新，新事件才消费。
 * 阈值仍位于exo_assist_config_t（0.01°），只在IDLE改变。
 * peak_mnm是100%等级的申请峰值，不能越过本轮START锁存最大力矩。 */
typedef struct {
    uint8_t schema_version;
    uint8_t duration_mode;
    uint16_t peak_mnm;
    uint16_t fixed_ms;
    uint16_t ratio_permille;
    uint16_t minimum_ms;
    uint16_t maximum_ms;
    uint16_t period_minimum_ms;
    uint16_t period_maximum_ms;
} exo_pulse_timing_t;

/* 0.10.12：申请峰值上限15 N·m，FIXED上限3秒；比例时长仍保持原2秒上限。
 * 申请峰值不等于实际输出；A还受本轮START上限、等级及上升速率约束。 */
#define EXO_PULSE_PEAK_HARD_MAX_MNM 15000U
#define EXO_PULSE_FIXED_MAX_MS 3000U
#define EXO_PULSE_SCALED_MAX_MS 2000U
typedef struct {
    int16_t reset_cdeg;
    uint16_t window_maximum_cdeg;
} exo_pulse_window_t;

#define EXO_PULSE_EVENT_SPEED_DEG_S 5.0f
#define EXO_PULSE_REVERSE_SPEED_DEG_S (-2.0f)
#define EXO_PULSE_STATIONARY_SPEED_DEG_S 2.0f
#define EXO_PULSE_STATIONARY_DWELL_US 100000
#define EXO_PULSE_RESET_DWELL_US 80000
#define EXO_PULSE_EVENT_CONFIRM_US 30000
#define EXO_PULSE_EVENT_TIMEOUT_US 150000
#define EXO_PULSE_FRESHNESS_US 50000U
/* 0.10.13：20 N·m/s使15 N·m、3秒sin²脉冲不再被速率削低。
 * 实际最大正斜率πA/Ta；15/3时约15.708 N·m/s。短脉冲仍限制A。
 * 这只约束新脉冲的冻结峰值，不允许反向/反馈失效时强行完成旧脉冲。 */
#define EXO_PULSE_MAX_RISE_NM_S 20.0f

typedef struct {
    int64_t last_now_us;
    int64_t stationary_started_us;
    int64_t pulse_started_us;
    int64_t last_valid_event_us;
    int64_t reset_started_us;
    int64_t confirmation_started_us;
    uint64_t last_source_us;
    uint32_t last_feedback_count;
    uint32_t frozen_duration_us;
    int16_t frozen_reset_cdeg;
    uint16_t frozen_window_maximum_cdeg;
    int16_t observed_reset_cdeg;
    uint32_t event_count;
    uint32_t pulse_count;
    float estimated_period_us;
    float frozen_peak_nm;
    float previous_angle_deg;
    float crossing_angle_deg;
    float progress_01;
    uint8_t state;
    uint8_t exit_reason;
    uint8_t valid_period_intervals;
    uint8_t confirmation_samples;
    uint8_t reset_samples;
    uint8_t stationary_samples;
    bool have_sample;
    bool peak_rate_limited;
    bool have_window;
} exo_swing_pulse_leg_t;

typedef struct {
    int64_t now_us;
    uint64_t source_us;
    uint32_t feedback_count;
    float angle_deg;
    float velocity_deg_s;
    bool valid;
} exo_swing_pulse_input_t;

exo_pulse_timing_t exo_swing_pulse_default_timing(uint16_t peak_mnm);
exo_pulse_window_t exo_swing_pulse_default_window(uint16_t trigger_cdeg);
bool exo_swing_pulse_timing_valid(const exo_pulse_timing_t *timing);
bool exo_swing_pulse_window_valid(const exo_pulse_window_t *window, uint16_t trigger_cdeg);
void exo_swing_pulse_reset(exo_swing_pulse_leg_t *leg);
/* 只读预检供双侧框架先核对整对；任何一侧失效都不能留下另一侧旧脉冲。 */
bool exo_swing_pulse_input_valid(const exo_swing_pulse_leg_t *leg,
    const exo_swing_pulse_input_t *input);
/* 无新反馈时不检测事件/复位，但有效、未过期的当前脉冲可按时钟推进。
 * 任何提前退出立即输出0，消耗本次事件，必须重新完成复位；从不按时钟发新脉冲。 */
float exo_swing_pulse_step(exo_swing_pulse_leg_t *leg,
    const exo_pulse_timing_t *timing, const exo_pulse_window_t *window,
    uint16_t trigger_cdeg, uint8_t level_percent,
    const exo_swing_pulse_input_t *input);

#endif
