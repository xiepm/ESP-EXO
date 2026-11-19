#ifndef EXO_ADAPTIVE_RUNTIME_H
#define EXO_ADAPTIVE_RUNTIME_H
#include "esp_err.h"
#include "exo_adaptive.h"
#include "exo_sensor_snapshot.h"
#include "exo_assist.h"
#include "exo_controller.h"
#define EXO_ADAPTIVE_LOG_CAPACITY 32U
#define EXO_ADAPTIVE_LOCK_LOG_CAPACITY 16U
typedef struct {
    int64_t timestamp_us;
    uint32_t sequence, side, system_state, permit_epoch, generation, dropped;
    float position_deg, velocity_deg_s, command_torque_nm;
    bool command_valid;
    exo_adaptive_leg_output_t output;
} exo_adaptive_lock_record_t;
typedef struct {
    exo_adaptive_input_t input;
    exo_adaptive_output_t output;
    uint32_t system_state, execution_us, maximum_execution_us, dropped_logs;
    /* 完整周期统计滞后一周期，避免把仅算法耗时伪称为整个控制周期。 */
    uint32_t previous_control_execution_us;
    bool assist_enabled[2];
    float command_torque_nm[2]; /* 最终五元命令FF，不是实测人体力矩。 */
    bool command_valid[2];
} exo_adaptive_log_t;
/* First hard failure per side in the latest START epoch, retained through
 * STOP and ring-buffer overflow. Read-only, bounded storage, no control role. */
typedef struct {
    exo_adaptive_log_t tick;
    uint32_t side, age_us, pair_skew_us, source_interval_us;
    bool present;
} exo_adaptive_hard_fault_t;
bool exo_adaptive_runtime_first_fault(unsigned side, exo_adaptive_hard_fault_t *fault);
/* Every authorized control tick, not a downsampled telemetry estimate.
 * Categories 1..7 partition total (until uint32 saturation). Read-only. */
enum { EXO_AFO_COUNT_TOTAL, EXO_AFO_COUNT_NONZERO, EXO_AFO_COUNT_HARD,
    EXO_AFO_COUNT_RANGE, EXO_AFO_COUNT_ACQUIRING, EXO_AFO_COUNT_STOP,
    EXO_AFO_COUNT_DOWNSTREAM_ZERO, EXO_AFO_COUNT_PROFILE_ZERO, EXO_AFO_COUNT_SIZE };
typedef struct {
    uint32_t permit_epoch, ticks[EXO_AFO_COUNT_SIZE], zero_run, longest_zero_run;
} exo_adaptive_output_counts_t;
bool exo_adaptive_runtime_output_counts(unsigned side, exo_adaptive_output_counts_t *counts);
/* USB diagnostic schema 1. Fixed 128 paired control-tick slots (~1.28 s).
 * Read by cursor without consuming; first hard fault is retained independently.
 * No formatting, allocation, transport IO or control action in the producer. */
#define EXO_AFO_TRACE_CAPACITY 128U
typedef struct {
    uint64_t source_us, previous_source_us;
    uint32_t source_count, age_us, source_interval_us, sensor_flags;
    float previous_q, previous_v, q, v, motor_estimate_nm, temperature_c, filtered_v;
    float phase, period_s, lambda, profile_nm, desired_nm, limited_nm, suggestion_nm, final_nm;
    uint32_t flags, faults, reasons, intervals, category;
    float event_age_s;
} exo_afo_trace_leg_t;
typedef struct {
    uint32_t id, sequence, permit, coordinate_epoch, generation, system_state;
    uint32_t valid_mask, quality, skew_us, previous_execution_us, dt_us;
    int64_t time_us;
    int32_t system_result;
    uint32_t gate_flags, reject_reason;
    exo_afo_trace_leg_t leg[2];
} exo_afo_trace_t;
void exo_adaptive_runtime_trace_finish(const exo_sensor_snapshot_t *sensor,
    const exo_assist_result_t *assist, const exo_controller_tick_result_t *control, esp_err_t result);
void exo_adaptive_runtime_trace_bounds(uint32_t *oldest, uint32_t *latest);
bool exo_adaptive_runtime_trace_after(uint32_t cursor, uint32_t through, exo_afo_trace_t *row);
bool exo_adaptive_runtime_trace_first(unsigned side, exo_afo_trace_t *row);
typedef struct {
    int64_t timestamp_us;
    uint32_t generation;
    exo_adaptive_config_t before, after;
} exo_adaptive_config_change_t;
/* 初始化/保存/加载只在启动或IDLE维护任务调用，永不在100Hz任务写NVS。 */
esp_err_t exo_adaptive_runtime_init(void);
bool exo_adaptive_runtime_config(exo_adaptive_config_t *c, uint32_t *generation);
esp_err_t exo_adaptive_runtime_set(const exo_adaptive_config_t *c);
esp_err_t exo_adaptive_runtime_prepare_trial(void);
bool exo_adaptive_runtime_trial_ready(void);
/* START锁存内部配置，STOP后才能修改；锁内只做标志切换。 */
esp_err_t exo_adaptive_runtime_lock_active_config(void);
void exo_adaptive_runtime_unlock_config(void);
uint32_t exo_adaptive_runtime_permit(const exo_assist_snapshot_t *assist, bool output_allowed);
esp_err_t exo_adaptive_runtime_save(void);
esp_err_t exo_adaptive_runtime_load(void);
bool exo_adaptive_runtime_last_change(exo_adaptive_config_change_t *change);
void exo_adaptive_runtime_update(const exo_sensor_snapshot_t *s, uint32_t permit_epoch,
    uint32_t system_state, bool emergency);
/* 当周期最终出口完成后调用，静态32槽环按10Hz入队，满时丢最旧并计数。 */
void exo_adaptive_runtime_finish(float left_command, float right_command, bool left_valid, bool right_valid,
    uint32_t previous_control_execution_us);
bool exo_adaptive_runtime_snapshot(exo_adaptive_log_t *log);
bool exo_adaptive_runtime_pop(exo_adaptive_log_t *log);
/* 100Hz锁相下降沿独立入环；UART消费，不提高旧ADAPT CSV速率。 */
bool exo_adaptive_runtime_pop_lock_loss(exo_adaptive_lock_record_t *record);
bool exo_adaptive_runtime_preview(uint32_t sequence, int64_t time_us,
    exo_joint_command_pair_t *suggestion);
/* Same control-tick diagnostics only; does not change permit, latch or torque. */
exo_assist_unavailable_reason_t exo_adaptive_runtime_unavailable_reason(
    uint32_t sequence, int64_t time_us, bool left_enabled, bool right_enabled);
#endif
