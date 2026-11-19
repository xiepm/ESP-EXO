#include "exo_adaptive_runtime.h"
#include <math.h>
#include <string.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "nvs.h"
static portMUX_TYPE s_lock=portMUX_INITIALIZER_UNLOCKED;
static exo_adaptive_engine_t s_engine;
static exo_adaptive_config_t s_selected;
static exo_adaptive_config_change_t s_change;
static uint32_t s_generation;
static bool s_initialized;
static bool s_active_config_locked;
static exo_adaptive_log_t s_current,s_published,s_logs[EXO_ADAPTIVE_LOG_CAPACITY];
static uint32_t s_head,s_count,s_dropped;
static int64_t s_last_log_us;
static uint64_t s_source_time[2];
static uint32_t s_source_count[2];
static exo_adaptive_lock_record_t s_lock_logs[EXO_ADAPTIVE_LOCK_LOG_CAPACITY];
static uint32_t s_lock_head,s_lock_count,s_lock_dropped;
static bool s_previous_locked[2];
static exo_adaptive_hard_fault_t s_first_fault[2];
static uint32_t s_fault_epoch;
static exo_adaptive_output_counts_t s_output_counts[2];
static void saturating_increment(uint32_t *value) { if(*value<UINT32_MAX)++*value; }
static uint32_t s_current_age[2],s_current_skew;
static uint32_t s_current_source_interval[2];
static exo_afo_trace_t s_trace[EXO_AFO_TRACE_CAPACITY],s_trace_first[2];
static uint32_t s_trace_head,s_trace_count,s_trace_id,s_trace_epoch;
static bool s_trace_first_present[2];
static uint64_t s_previous_source[2];
static float s_previous_q[2],s_previous_v[2];
static uint32_t s_trace_dt,s_current_category[2];
static uint32_t trace_reject_code(exo_controller_assist_reject_reason_t reason) {
    /* Schema1 codes are explicit; do not expose an internal enum ABI. */
    switch(reason) {
    case EXO_CONTROLLER_ASSIST_REJECT_NONE:return 0;
    case EXO_CONTROLLER_ASSIST_REJECT_NOT_REQUESTED:return 1;
    case EXO_CONTROLLER_ASSIST_REJECT_SHADOW_ONLY:return 2;
    case EXO_CONTROLLER_ASSIST_REJECT_RESULT_INVALID:return 3;
    case EXO_CONTROLLER_ASSIST_REJECT_EXECUTION_MISMATCH:return 4;
    case EXO_CONTROLLER_ASSIST_REJECT_EPOCH_MISMATCH:return 5;
    case EXO_CONTROLLER_ASSIST_REJECT_SENSOR_SEQUENCE_MISMATCH:return 6;
    case EXO_CONTROLLER_ASSIST_REJECT_GAIT_SEQUENCE_MISMATCH:return 7;
    case EXO_CONTROLLER_ASSIST_REJECT_TIMESTAMP_MISMATCH:return 8;
    case EXO_CONTROLLER_ASSIST_REJECT_OUTPUT_KIND:return 9;
    case EXO_CONTROLLER_ASSIST_REJECT_COMPOSE_INVALID:return 10;
    default:return 255;
    }
}
esp_err_t exo_adaptive_runtime_init(void) {
    exo_adaptive_config_t c=exo_adaptive_default_config();
    if(!exo_adaptive_engine_init(&s_engine,&c)) return ESP_ERR_INVALID_ARG;
    s_engine.walking_mode=EXO_ADAPTIVE_ASSIST_MODE;
    portENTER_CRITICAL(&s_lock);
    s_selected=c;s_generation=1;s_initialized=true;s_active_config_locked=false;
    memset(&s_current,0,sizeof(s_current));memset(&s_published,0,sizeof(s_published));
    memset(&s_change,0,sizeof(s_change));s_head=0;s_count=0;s_dropped=0;s_last_log_us=0;
    memset(s_source_time,0,sizeof(s_source_time));memset(s_source_count,0,sizeof(s_source_count));
    s_lock_head=0;s_lock_count=0;s_lock_dropped=0;
    memset(s_previous_locked,0,sizeof(s_previous_locked));
    memset(s_first_fault,0,sizeof(s_first_fault));s_fault_epoch=0;
    memset(s_output_counts,0,sizeof(s_output_counts));
    s_trace_head=0;s_trace_count=0;s_trace_id=0;s_trace_epoch=0;
    memset(s_trace_first_present,0,sizeof(s_trace_first_present));
    portEXIT_CRITICAL(&s_lock);
    /* 不自动加载NVS配置。即使显式load，仍须通过独立的START/部署边界。 */
    return ESP_OK;
}
bool exo_adaptive_runtime_config(exo_adaptive_config_t *c,uint32_t *generation) {
    if(!c) return false;
    portENTER_CRITICAL(&s_lock);
    bool ready=s_initialized;
    if(ready) { *c=s_selected;if(generation)*generation=s_generation; }
    portEXIT_CRITICAL(&s_lock);return ready;
}
esp_err_t exo_adaptive_runtime_set(const exo_adaptive_config_t *c) {
    if(!exo_adaptive_config_valid(c)||c->afo.maximum_dt_s>0.03f) return ESP_ERR_INVALID_ARG;
    int64_t now=esp_timer_get_time();
    portENTER_CRITICAL(&s_lock);
    if(!s_initialized||s_active_config_locked) { portEXIT_CRITICAL(&s_lock);return ESP_ERR_INVALID_STATE; }
    s_change.before=s_selected;s_selected=*c;
    if(++s_generation==0) s_generation=1;
    s_change.after=*c;s_change.generation=s_generation;s_change.timestamp_us=now;
    portEXIT_CRITICAL(&s_lock);return ESP_OK;
}
esp_err_t exo_adaptive_runtime_lock_active_config(void) {
    portENTER_CRITICAL(&s_lock);
    if(!s_initialized||s_active_config_locked) {
        portEXIT_CRITICAL(&s_lock);return ESP_ERR_INVALID_STATE;
    }
    const float cap=(float)EXO_ADAPTIVE_CONFIG_MAX_MNM/1000.0f;
    if(EXO_ADAPTIVE_TRIAL_ALLOWED&&!exo_adaptive_trial_preset_matches(&s_selected)) {
        portEXIT_CRITICAL(&s_lock);return ESP_ERR_INVALID_STATE;
    }
    for(unsigned i=0;i<2;i++) {
        if(s_selected.leg[i].torque_limit_nm>cap||s_selected.leg[i].flex_amplitude_nm>cap||
           s_selected.leg[i].ext_amplitude_nm>cap) {
            portEXIT_CRITICAL(&s_lock);return ESP_ERR_INVALID_ARG;
        }
    }
    s_active_config_locked=true;
    portEXIT_CRITICAL(&s_lock);return ESP_OK;
}
esp_err_t exo_adaptive_runtime_prepare_trial(void) {
    if(!EXO_ADAPTIVE_TRIAL_ALLOWED)return ESP_ERR_INVALID_STATE;
    exo_adaptive_config_t c=exo_adaptive_trial_preset();
    return exo_adaptive_runtime_set(&c);
}
bool exo_adaptive_runtime_trial_ready(void) {
    exo_adaptive_config_t c;
    return EXO_ADAPTIVE_TRIAL_ALLOWED&&exo_adaptive_runtime_config(&c,NULL)&&
        exo_adaptive_trial_preset_matches(&c);
}
void exo_adaptive_runtime_unlock_config(void) {
    portENTER_CRITICAL(&s_lock);s_active_config_locked=false;portEXIT_CRITICAL(&s_lock);
}
uint32_t exo_adaptive_runtime_permit(const exo_assist_snapshot_t *assist, bool output_allowed) {
    if(!assist||!output_allowed||!assist->active_valid||
       assist->active.strategy_id!=EXO_ASSIST_STRATEGY_ADAPTIVE_GAUSSIAN) return 0U;
    return (assist->execution_state==EXO_ASSIST_EXEC_SHADOW||
        (EXO_ADAPTIVE_ACTIVE_ALLOWED&&assist->execution_state==EXO_ASSIST_EXEC_ACTIVE))
        ?assist->activation_epoch:0U;
}
bool exo_adaptive_runtime_last_change(exo_adaptive_config_change_t *change) {
    if(!change) return false;
    portENTER_CRITICAL(&s_lock);*change=s_change;portEXIT_CRITICAL(&s_lock);
    return change->generation!=0;
}
esp_err_t exo_adaptive_runtime_save(void) {
    exo_adaptive_config_t c;uint8_t bytes[EXO_ADAPTIVE_CONFIG_BYTES];
    if(!exo_adaptive_runtime_config(&c,NULL)||!exo_adaptive_config_encode(&c,bytes,sizeof(bytes))) return ESP_ERR_INVALID_STATE;
    nvs_handle_t handle;esp_err_t err=nvs_open("exo_adaptive",NVS_READWRITE,&handle);
    if(err!=ESP_OK)return err;
    err=nvs_set_blob(handle,"config_v1",bytes,sizeof(bytes));
    if(err==ESP_OK)err=nvs_commit(handle);
    nvs_close(handle);return err;
}
esp_err_t exo_adaptive_runtime_load(void) {
    uint8_t bytes[EXO_ADAPTIVE_CONFIG_BYTES];size_t size=sizeof(bytes);
    nvs_handle_t handle;esp_err_t err=nvs_open("exo_adaptive",NVS_READONLY,&handle);
    if(err!=ESP_OK)return err;
    err=nvs_get_blob(handle,"config_v1",bytes,&size);nvs_close(handle);
    if(err!=ESP_OK)return err;
    exo_adaptive_config_t c;
    if(!exo_adaptive_config_decode(&c,bytes,(uint32_t)size))return ESP_ERR_INVALID_ARG;
    return exo_adaptive_runtime_set(&c);
}
void exo_adaptive_runtime_update(const exo_sensor_snapshot_t *s,uint32_t epoch,uint32_t state,bool emergency) {
    if(!s||!s_initialized)return;
    int64_t begin=esp_timer_get_time();
    exo_adaptive_config_t c;uint32_t generation=0;
    if(!exo_adaptive_runtime_config(&c,&generation))return;
    if(generation!=s_engine.config_generation) (void)exo_adaptive_engine_configure(&s_engine,&c,generation);
    exo_adaptive_input_t in={.timestamp_us=s->captured_us,.sequence=s->sequence,
        .coordinate_epoch=s->reset_epoch,.permit_epoch=epoch,.emergency=emergency};
    uint64_t delta=s_engine.timestamp_us>0&&s->captured_us>s_engine.timestamp_us?
        (uint64_t)s->captured_us-(uint64_t)s_engine.timestamp_us:0;
    s_trace_dt=delta>UINT32_MAX?UINT32_MAX:(uint32_t)delta;
    for(unsigned i=0;i<2;i++) {
        s_previous_source[i]=s_engine.source_timestamp_us[i];
        s_previous_q[i]=s_engine.source_position_deg[i];s_previous_v[i]=s_engine.source_velocity_deg_s[i];
        const exo_sensor_joint_snapshot_t *j=i?&s->right:&s->left;
        s_current_age[i]=j->age_us;
        const uint64_t previous=s_source_time[i],source=j->source_timestamp_us;
        const uint64_t interval=previous&&source>=previous?source-previous:0U;
        s_current_source_interval[i]=interval>UINT32_MAX?UINT32_MAX:(uint32_t)interval;
        in.position_deg[i]=j->joint_sample.position_deg;in.velocity_deg_s[i]=j->joint_sample.velocity_deg_s;
        in.feedback_torque_nm[i]=j->joint_sample.torque_nm;
        in.source_timestamp_us[i]=j->source_timestamp_us;
        /* 异步100Hz反馈允许短暂重复，纯算法按源时间去重；回退/过期立即失效。 */
        in.valid[i]=j->joint_valid&&j->online&&!j->motor_fault&&j->age_us<=50000U&&
            j->source_timestamp_us>=s_source_time[i]&&j->source_feedback_count>=s_source_count[i]&&
            s->pair_skew_us<=20000U&&s->captured_us>0&&
            j->source_timestamp_us<=(uint64_t)s->captured_us;
        in.feedback_torque_valid[i]=in.valid[i]&&isfinite(in.feedback_torque_nm[i]);
        s_source_time[i]=j->source_timestamp_us;s_source_count[i]=j->source_feedback_count;
    }
    s_current_skew=s->pair_skew_us;
    s_current.input=in;s_current.system_state=state;
    for(unsigned i=0;i<2;i++)s_current.assist_enabled[i]=c.leg[i].enabled>0;
    if(!exo_adaptive_engine_update(&s_engine,&in,&s_current.output))
        memset(&s_current.output,0,sizeof(s_current.output));
    int64_t elapsed=esp_timer_get_time()-begin;
    s_current.execution_us=elapsed>0?(uint32_t)elapsed:0;
    if(s_current.execution_us>s_current.maximum_execution_us)s_current.maximum_execution_us=s_current.execution_us;
}
void exo_adaptive_runtime_finish(float left,float right,bool lv,bool rv,uint32_t previous_control_execution_us) {
    s_current.command_torque_nm[0]=left;s_current.command_torque_nm[1]=right;
    s_current.command_valid[0]=lv;s_current.command_valid[1]=rv;
    s_current.previous_control_execution_us=previous_control_execution_us;
    portENTER_CRITICAL(&s_lock);
    if(s_current.input.permit_epoch&&s_current.input.permit_epoch!=s_fault_epoch) {
        memset(s_first_fault,0,sizeof(s_first_fault));
        memset(s_output_counts,0,sizeof(s_output_counts));
        s_fault_epoch=s_current.input.permit_epoch;
    }
    for(unsigned i=0;i<2;i++) {
        const exo_adaptive_leg_output_t *o=&s_current.output.leg[i];
        s_current_category[i]=EXO_AFO_COUNT_STOP;
        if(s_current.input.permit_epoch) {
            exo_adaptive_output_counts_t *counts=&s_output_counts[i];
            counts->permit_epoch=s_current.input.permit_epoch;
            unsigned category;
            if(s_current.command_valid[i]&&fabsf(s_current.command_torque_nm[i])>1e-6f)
                category=EXO_AFO_COUNT_NONZERO;
            else if(o->rearm_required||(o->faults&(EXO_ADAPTIVE_FAULT_INPUT|EXO_ADAPTIVE_FAULT_TIME|EXO_ADAPTIVE_FAULT_CONFIG)))
                category=EXO_AFO_COUNT_HARD;
            else if(o->faults&EXO_ADAPTIVE_FAULT_RANGE)category=EXO_AFO_COUNT_RANGE;
            else if(o->faults&EXO_ADAPTIVE_FAULT_STOP)category=EXO_AFO_COUNT_STOP;
            else if(o->faults&EXO_ADAPTIVE_FAULT_PHASE)category=EXO_AFO_COUNT_ACQUIRING;
            else if(!s_current.command_valid[i]||fabsf(o->limited_nm)>1e-6f)
                category=EXO_AFO_COUNT_DOWNSTREAM_ZERO;
            else category=EXO_AFO_COUNT_PROFILE_ZERO;
            s_current_category[i]=category;
            saturating_increment(&counts->ticks[EXO_AFO_COUNT_TOTAL]);
            saturating_increment(&counts->ticks[category]);
            if(category==EXO_AFO_COUNT_NONZERO)counts->zero_run=0;
            else saturating_increment(&counts->zero_run);
            if(counts->zero_run>counts->longest_zero_run)counts->longest_zero_run=counts->zero_run;
        }
        if(s_current.input.permit_epoch&&o->rearm_required&&!s_first_fault[i].present)
            s_first_fault[i]=(exo_adaptive_hard_fault_t){.tick=s_current,.side=i,
                .age_us=s_current_age[i],.pair_skew_us=s_current_skew,
                .source_interval_us=s_current_source_interval[i],.present=true};
        if(s_previous_locked[i]&&!o->locked) {
            if(s_lock_count==EXO_ADAPTIVE_LOCK_LOG_CAPACITY) {
                s_lock_head=(s_lock_head+1)%EXO_ADAPTIVE_LOCK_LOG_CAPACITY;s_lock_count--;s_lock_dropped++;
            }
            exo_adaptive_lock_record_t *r=&s_lock_logs[(s_lock_head+s_lock_count)%EXO_ADAPTIVE_LOCK_LOG_CAPACITY];
            *r=(exo_adaptive_lock_record_t){.timestamp_us=s_current.input.timestamp_us,
                .sequence=s_current.input.sequence,.side=i,.system_state=s_current.system_state,
                .permit_epoch=s_current.input.permit_epoch,.generation=s_current.output.config_generation,
                .dropped=s_lock_dropped,.position_deg=s_current.input.position_deg[i],
                .velocity_deg_s=s_current.input.velocity_deg_s[i],.command_torque_nm=s_current.command_torque_nm[i],
                .command_valid=s_current.command_valid[i],.output=*o};
            s_lock_count++;
        }
        s_previous_locked[i]=o->locked;
    }
    if(s_current.input.timestamp_us-s_last_log_us>=100000) {
        if(s_count==EXO_ADAPTIVE_LOG_CAPACITY) {
            s_head=(s_head+1)%EXO_ADAPTIVE_LOG_CAPACITY;s_count--;s_dropped++;
        }
        s_current.dropped_logs=s_dropped;
        s_logs[(s_head+s_count)%EXO_ADAPTIVE_LOG_CAPACITY]=s_current;s_count++;
        s_last_log_us=s_current.input.timestamp_us;
    }
    s_current.dropped_logs=s_dropped;s_published=s_current;
    portEXIT_CRITICAL(&s_lock);
}
bool exo_adaptive_runtime_snapshot(exo_adaptive_log_t *log) {
    if(!log)return false;
    portENTER_CRITICAL(&s_lock);*log=s_published;bool ready=s_initialized;portEXIT_CRITICAL(&s_lock);return ready;
}
void exo_adaptive_runtime_trace_finish(const exo_sensor_snapshot_t *sensor,
    const exo_assist_result_t *assist,const exo_controller_tick_result_t *control,esp_err_t result) {
    if(!sensor||!assist||!control||!s_initialized)return;
    exo_afo_trace_t row={.sequence=sensor->sequence,.time_us=sensor->captured_us,
        .permit=s_current.input.permit_epoch,.coordinate_epoch=sensor->reset_epoch,
        .generation=s_current.output.config_generation,.system_state=s_current.system_state,
        .valid_mask=sensor->valid_sensor_mask,.quality=sensor->quality_flags,.skew_us=sensor->pair_skew_us,
        .previous_execution_us=s_current.previous_control_execution_us,.dt_us=s_trace_dt,
        .system_result=result,.reject_reason=trace_reject_code(control->assist_reject_reason),
        .gate_flags=(uint32_t)control->output_gate_open|((uint32_t)control->base_generated<<1)|
            ((uint32_t)control->assist_suggested<<2)|((uint32_t)control->assist_accepted<<3)|
            ((uint32_t)control->assist_applied<<4)|((uint32_t)control->send_attempted<<5)|
            ((uint32_t)control->send_succeeded<<6)|((uint32_t)control->send_partially_succeeded<<7)};
    for(unsigned i=0;i<2;i++) {
        const exo_sensor_joint_snapshot_t *j=i?&sensor->right:&sensor->left;
        const exo_adaptive_leg_output_t *o=&s_current.output.leg[i];
        row.leg[i]=(exo_afo_trace_leg_t){.source_us=j->source_timestamp_us,.source_count=j->source_feedback_count,
            .previous_source_us=s_previous_source[i],.previous_q=s_previous_q[i],.previous_v=s_previous_v[i],
            .age_us=j->age_us,.source_interval_us=s_current_source_interval[i],
            .sensor_flags=(uint32_t)j->snapshot_available|((uint32_t)j->mapping_valid<<1)|
                ((uint32_t)j->direction_valid<<2)|((uint32_t)j->zero_valid<<3)|((uint32_t)j->feedback_valid<<4)|
                ((uint32_t)j->online<<5)|((uint32_t)j->motor_fault<<6)|((uint32_t)j->joint_valid<<7),
            .q=j->joint_sample.position_deg,.v=j->joint_sample.velocity_deg_s,.motor_estimate_nm=j->joint_sample.torque_nm,
            .temperature_c=j->joint_sample.temperature_c,.profile_nm=o->profile_nm,
            .filtered_v=s_engine.event[i].filtered_velocity,.phase=o->corrected_phase,.period_s=o->period_s,
            .lambda=o->lambda,.desired_nm=o->desired_nm,.limited_nm=o->limited_nm,
            .suggestion_nm=i?assist->suggestion.right.torque_ff_nm:assist->suggestion.left.torque_ff_nm,
            .final_nm=s_current.command_torque_nm[i],.faults=o->faults,.reasons=o->lock_reasons,
            .intervals=o->lock_intervals,.event_age_s=o->lock_event_age_s,.category=s_current_category[i],
            .flags=(uint32_t)s_current.input.valid[i]|((uint32_t)o->locked<<1)|((uint32_t)o->rearm_required<<2)|
                ((uint32_t)s_current.command_valid[i]<<3)|((uint32_t)assist->valid<<4)};
    }
    portENTER_CRITICAL(&s_lock);
    if(++s_trace_id==0){s_trace_id=1;s_trace_count=0;s_trace_head=0;}
    row.id=s_trace_id;
    if(row.permit&&row.permit!=s_trace_epoch) {
        s_trace_epoch=row.permit;memset(s_trace_first_present,0,sizeof(s_trace_first_present));
    }
    for(unsigned i=0;i<2;i++)if(row.permit&&(row.leg[i].flags&4U)&&!s_trace_first_present[i]) {
        s_trace_first[i]=row;s_trace_first_present[i]=true;
    }
    if(s_trace_count==EXO_AFO_TRACE_CAPACITY){s_trace_head=(s_trace_head+1)%EXO_AFO_TRACE_CAPACITY;s_trace_count--;}
    s_trace[(s_trace_head+s_trace_count)%EXO_AFO_TRACE_CAPACITY]=row;s_trace_count++;
    portEXIT_CRITICAL(&s_lock);
}
void exo_adaptive_runtime_trace_bounds(uint32_t *oldest,uint32_t *latest) {
    portENTER_CRITICAL(&s_lock);
    if(oldest)*oldest=s_trace_count?s_trace[s_trace_head].id:0;
    if(latest)*latest=s_trace_id;
    portEXIT_CRITICAL(&s_lock);
}
bool exo_adaptive_runtime_trace_after(uint32_t cursor,uint32_t through,exo_afo_trace_t *row) {
    if(!row)return false;
    bool found=false;portENTER_CRITICAL(&s_lock);
    if(s_trace_count&&cursor<s_trace_id) {
        uint32_t first=s_trace[s_trace_head].id;
        uint32_t offset=cursor<first?0:cursor-first+1;
        if(offset<s_trace_count) {
            const exo_afo_trace_t *r=&s_trace[(s_trace_head+offset)%EXO_AFO_TRACE_CAPACITY];
            if(r->id<=through){*row=*r;found=true;}
        }
    }
    portEXIT_CRITICAL(&s_lock);return found;
}
bool exo_adaptive_runtime_trace_first(unsigned side,exo_afo_trace_t *row) {
    if(side>1||!row)return false;
    portENTER_CRITICAL(&s_lock);bool found=s_trace_first_present[side];
    if(found)*row=s_trace_first[side];
    portEXIT_CRITICAL(&s_lock);return found;
}
bool exo_adaptive_runtime_output_counts(unsigned side,exo_adaptive_output_counts_t *counts) {
    if(side>1||!counts)return false;
    portENTER_CRITICAL(&s_lock);*counts=s_output_counts[side];
    bool ready=s_initialized;portEXIT_CRITICAL(&s_lock);return ready;
}
bool exo_adaptive_runtime_first_fault(unsigned side,exo_adaptive_hard_fault_t *fault) {
    if(side>1||!fault)return false;
    portENTER_CRITICAL(&s_lock);*fault=s_first_fault[side];portEXIT_CRITICAL(&s_lock);
    return fault->present;
}
bool exo_adaptive_runtime_pop(exo_adaptive_log_t *log) {
    if(!log)return false;
    portENTER_CRITICAL(&s_lock);
    bool ready=s_count>0;
    if(ready) { *log=s_logs[s_head];s_head=(s_head+1)%EXO_ADAPTIVE_LOG_CAPACITY;s_count--; }
    portEXIT_CRITICAL(&s_lock);return ready;
}
bool exo_adaptive_runtime_preview(uint32_t sequence,int64_t time,exo_joint_command_pair_t *p) {
    if(!p)return false;
    memset(p,0,sizeof(*p));
    /* 只由同一100Hz任务调用，读取本周期未发布副本，无跨周期/跨任务拼接。 */
    if(s_current.output.sequence!=sequence||s_current.output.timestamp_us!=time)return false;
    p->left.torque_ff_nm=s_current.output.leg[0].limited_nm;
    p->right.torque_ff_nm=s_current.output.leg[1].limited_nm;
    return true;
}
exo_assist_unavailable_reason_t exo_adaptive_runtime_unavailable_reason(
    uint32_t sequence,int64_t time,bool left_enabled,bool right_enabled) {
    if(s_current.output.sequence!=sequence||s_current.output.timestamp_us!=time)
        return EXO_ASSIST_UNAVAILABLE_CONFIG_INVALID;
    const bool enabled[2]={left_enabled,right_enabled};
    bool rearm=false,fault=false,acquiring=false;
    for(unsigned i=0;i<2;i++) {
        if(!enabled[i])continue;
        const exo_adaptive_leg_output_t *o=&s_current.output.leg[i];
        rearm|=o->rearm_required;
        const uint32_t recoverable=EXO_ADAPTIVE_FAULT_PHASE|
            (EXO_ADAPTIVE_ASSIST_MODE?EXO_ADAPTIVE_FAULT_RANGE:0U);
        fault|=(o->faults&~(recoverable|EXO_ADAPTIVE_FAULT_REARM))!=0;
        acquiring|=!o->locked||(o->faults&recoverable)!=0;
    }
    if(rearm)return EXO_ASSIST_UNAVAILABLE_AFO_REARM_REQUIRED;
    if(fault)return EXO_ASSIST_UNAVAILABLE_AFO_INPUT_FAULT;
    return acquiring?EXO_ASSIST_UNAVAILABLE_AFO_ACQUIRING:EXO_ASSIST_UNAVAILABLE_NONE;
}
bool exo_adaptive_runtime_pop_lock_loss(exo_adaptive_lock_record_t *record) {
    if(!record)return false;
    portENTER_CRITICAL(&s_lock);
    const bool ready=s_lock_count>0;
    if(ready) {
        *record=s_lock_logs[s_lock_head];record->dropped=s_lock_dropped;
        s_lock_head=(s_lock_head+1)%EXO_ADAPTIVE_LOCK_LOG_CAPACITY;s_lock_count--;
    }
    portEXIT_CRITICAL(&s_lock);return ready;
}
