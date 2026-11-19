#include "exo_adaptive.h"
#include <math.h>
#include <string.h>
/* A valid asynchronous source may skip a control tick. Integrate the
 * observer in bounded substeps up to the existing 50 ms feedback freshness
 * boundary; do not confuse source spacing with the control-task deadline.
 * No synthetic events: the event detector still receives ONE measured sample. */
static bool walking_source_update(exo_afo_t *afo,const exo_afo_config_t *c,
    float previous_q,float q,float sample_dt) {
    if(!isfinite(sample_dt)||sample_dt<=0||sample_dt>0.05f)return false;
    const unsigned steps=(unsigned)ceilf(sample_dt/c->maximum_dt_s);
    if(steps==0||steps>5)return false;
    for(unsigned k=1;k<=steps;k++) {
        float sample=previous_q+(q-previous_q)*(float)k/(float)steps;
        if(!exo_afo_update(afo,c,sample,sample_dt/(float)steps))return false;
    }
    return true;
}
bool exo_adaptive_engine_init(exo_adaptive_engine_t *s,const exo_adaptive_config_t *c) {
    if(!s||!exo_adaptive_config_valid(c)) return false;
    memset(s,0,sizeof(*s));s->config=*c;s->config_generation=1;s->initialized=true;return true;
}
bool exo_adaptive_engine_configure(exo_adaptive_engine_t *s,const exo_adaptive_config_t *c,uint32_t gen) {
    if(!s||!s->initialized||!exo_adaptive_config_valid(c)||gen==0) return false;
    bool reset=memcmp(&s->config.afo,&c->afo,sizeof(c->afo))!=0;
    for(unsigned i=0;i<2;i++) {
        s->transition_from[i]=s->previous_desired[i];s->transition_elapsed[i]=0;
        if(reset) {
            memset(&s->afo[i],0,sizeof(s->afo[i]));
            s->acquisition_seeded[i]=false;
            exo_adaptive_event_reset(&s->event[i]);
            if(s->permit_epoch) s->rearm_required[i]=true;
        }
    }
    s->config=*c;s->config_generation=gen;return true;
}
bool exo_adaptive_engine_update(exo_adaptive_engine_t *s,const exo_adaptive_input_t *in,exo_adaptive_output_t *out) {
    if(!s||!s->initialized||!in||!out) return false;
    memset(out,0,sizeof(*out));out->timestamp_us=in->timestamp_us;
    out->sequence=in->sequence;out->config_generation=s->config_generation;
    /* 先比较再相减，避免INT64_MIN等坏时间戳导致有符号溢出。 */
    float dt=s->timestamp_us>0?(in->timestamp_us>s->timestamp_us
        ?(float)(in->timestamp_us-s->timestamp_us)*1e-6f:0.0f):0.01f;
    bool time_ok=in->timestamp_us>0&&isfinite(dt)&&dt>0&&dt<=s->config.afo.maximum_dt_s&&
        (s->timestamp_us==0||in->sequence>s->sequence);
    bool reset=s->timestamp_us>0&&in->coordinate_epoch!=s->coordinate_epoch;
    if(in->permit_epoch!=s->permit_epoch) {
        s->permit_epoch=in->permit_epoch;
        for(unsigned i=0;i<2;i++) {
            s->previous_torque[i]=0;s->lambda[i]=0;s->was_locked[i]=false;
            s->rearm_required[i]=false;s->arm_event_counts[i]=s->event[i].events;
            s->velocity_pause_since_us[i]=0;
            s->phase_pause_since_us[i]=0;
        }
    }
    for(unsigned i=0;i<2;i++) {
        exo_adaptive_leg_output_t *o=&out->leg[i];
        const exo_adaptive_leg_config_t *c=&s->config.leg[i];
        bool valid=in->valid[i]&&isfinite(in->position_deg[i])&&isfinite(in->velocity_deg_s[i])&&
            fabsf(in->position_deg[i])<=180&&fabsf(in->velocity_deg_s[i])<=1000;
        uint64_t source=in->source_timestamp_us[i]?in->source_timestamp_us[i]:(uint64_t)in->timestamp_us;
        bool fresh=source!=s->source_timestamp_us[i];
        float sample_dt=s->source_timestamp_us[i]&&source>s->source_timestamp_us[i]
            ?(float)(source-s->source_timestamp_us[i])*1e-6f:dt;
        if(source<s->source_timestamp_us[i]||source>(uint64_t)in->timestamp_us)valid=false;
        /* Check before the operating-window branch as well: an excursion must
         * never conceal an expired source interval and turn it into recovery. */
        if(s->walking_mode&&fresh&&s->source_timestamp_us[i]&&sample_dt>0.05f)valid=false;
        if(fresh&&s->source_timestamp_us[i]&&valid) {
            float motion_bound=fmaxf(fabsf(in->velocity_deg_s[i]),fabsf(s->source_velocity_deg_s[i]))*sample_dt+3.0f;
            if(fabsf(in->position_deg[i]-s->source_position_deg[i])>motion_bound)valid=false;
        }
        if(!time_ok) o->faults|=EXO_ADAPTIVE_FAULT_TIME;
        if(!valid||reset) o->faults|=EXO_ADAPTIVE_FAULT_INPUT;
        if(in->emergency) o->faults|=EXO_ADAPTIVE_FAULT_STOP;
        /* Raw operating-window limits always close torque immediately.
         * For <=100 ms speed-only excursions, keep observing valid positions:
         * pausing output is not evidence that the measured stride disappeared.
         * Angle excursions, sustained overspeed and hard faults still reset.
         * This does not clear a latch or bypass the final raw-speed limiter. */
        if(s->walking_mode&&s->permit_epoch&&valid&&c->enabled>0&&
           (in->position_deg[i]<c->minimum_angle_deg||in->position_deg[i]>c->maximum_angle_deg||
            fabsf(in->velocity_deg_s[i])>c->maximum_velocity_deg_s))
            o->faults|=EXO_ADAPTIVE_FAULT_RANGE;
        bool speed_only=s->walking_mode&&o->faults==EXO_ADAPTIVE_FAULT_RANGE&&
            in->position_deg[i]>=c->minimum_angle_deg&&in->position_deg[i]<=c->maximum_angle_deg;
        if(speed_only) {
            if(!s->velocity_pause_since_us[i])s->velocity_pause_since_us[i]=(int64_t)source;
        } else s->velocity_pause_since_us[i]=0;
        /* Re-reading a still-fresh source does not measure a longer speed
         * excursion. Use source time for the <=100 ms observation window;
         * torque remains zero until a new in-range sample arrives. A held
         * sample gets at most the existing 50 ms freshness allowance. Runtime
         * stale/invalid input and new-source spacing checks remain independent. */
        bool brief_speed_pause=speed_only&&
            (int64_t)source-s->velocity_pause_since_us[i]<=100000&&
            in->timestamp_us-(int64_t)source<=50000;
        if(o->faults&&!brief_speed_pause) {
            if(o->faults&EXO_ADAPTIVE_FAULT_INPUT)o->lock_reasons|=EXO_ADAPTIVE_LOCK_INPUT;
            if(o->faults&EXO_ADAPTIVE_FAULT_TIME)o->lock_reasons|=EXO_ADAPTIVE_LOCK_TIME;
            if(in->emergency)o->lock_reasons|=EXO_ADAPTIVE_LOCK_EMERGENCY;
            memset(&s->afo[i],0,sizeof(s->afo[i]));exo_adaptive_event_reset(&s->event[i]);
            s->acquisition_seeded[i]=false;
            s->source_timestamp_us[i]=0;
            const bool operating_pause=s->walking_mode&&o->faults==EXO_ADAPTIVE_FAULT_RANGE;
            if(s->permit_epoch&&!operating_pause) s->rearm_required[i]=true;
            if(operating_pause) {
                s->was_locked[i]=false;
                /* Keep sensor continuity checks across the range boundary. */
                s->source_timestamp_us[i]=source;
                s->source_position_deg[i]=in->position_deg[i];
                s->source_velocity_deg_s[i]=in->velocity_deg_s[i];
            }
            s->arm_event_counts[i]=0;
        } else {
            float q=in->position_deg[i],v=in->velocity_deg_s[i];
            s->event[i].event=false;
            if(fresh) {
                bool integrated=s->walking_mode
                    ?walking_source_update(&s->afo[i],&s->config.afo,
                        s->source_timestamp_us[i]?s->source_position_deg[i]:q,q,sample_dt)
                    :exo_afo_update(&s->afo[i],&s->config.afo,q,sample_dt);
                if(!integrated) {
                    /* 循环dt正常不代表源反馈dt正常。重复帧后若新样本间隔过大，
                     * 振荡器拒绝积分也必须撤销本次授权，不能恢复后沿用旧epoch。 */
                    o->faults|=EXO_ADAPTIVE_FAULT_INPUT;
                    o->lock_reasons|=EXO_ADAPTIVE_LOCK_INPUT;
                    memset(&s->afo[i],0,sizeof(s->afo[i]));
                    s->acquisition_seeded[i]=false;
                    exo_adaptive_event_reset(&s->event[i]);
                    if(s->permit_epoch)s->rearm_required[i]=true;
                }
                else if(s->walking_mode)exo_adaptive_walking_event_update(
                    &s->event[i],&s->config.afo,q,sample_dt,(int64_t)source);
                else exo_adaptive_event_update(&s->event[i],&s->config.afo,q,v,
                    s->afo[i].phase_rad,s->afo[i].omega_rad_s,sample_dt,(int64_t)source);
                /* Capture only: the first measured same-side period gives an
                 * observable initial frequency, phase, DC and excursion. Seed
                 * while still unlocked/zero, never jump an active oscillator.
                 * Walking torque uses the independent measured phase; this
                 * seeds the AFO diagnostic. Both paths need two intervals. */
                exo_adaptive_event_t *event=&s->event[i];
                if(!s->acquisition_seeded[i]&&!s->was_locked[i]&&event->event&&
                   event->intervals==1U&&event->period_s>0) {
                    exo_afo_reset(&s->afo[i],&s->config.afo,q);
                    exo_afo_t *afo=&s->afo[i];
                    afo->omega_rad_s=2*EXO_ADAPTIVE_PI/event->period_s;
                    afo->phase_rad=exo_adaptive_wrap(afo->omega_rad_s*
                        (float)((int64_t)source-event->last_event_us)*1e-6f);
                    afo->cosine[0]=.5f*(event->last_peak_deg+event->last_low_deg)/s->config.afo.input_scale_deg;
                    afo->cosine[1]=.5f*(event->last_peak_deg-event->last_low_deg)/s->config.afo.input_scale_deg;
                    event->offset_rad=0;event->target_offset_rad=0;
                    event->reference_phase_error_rad=0;
                    event->phase_before=afo->phase_rad/(2*EXO_ADAPTIVE_PI);
                    if(!s->walking_mode)event->phase_after=event->phase_before;
                    s->acquisition_seeded[i]=true;
                }
                /* 可信同侧周期帮助排除谐波/半频捕获，不能用另一侧间隔代替。
                 * 频率以0.5/s小增益校正，最大2rad/s²；相位仍由AFO积分，绝不硬复位。 */
                if(s->event[i].intervals>=2 && s->event[i].period_s>0 &&
                   (float)(in->timestamp_us-s->event[i].last_event_us)*1e-6f<1.6f*s->event[i].period_s) {
                    float frequency_error=2*EXO_ADAPTIVE_PI/s->event[i].period_s-s->afo[i].omega_rad_s;
                    s->afo[i].omega_rad_s+=sample_dt*fmaxf(-2.0f,fminf(2.0f,0.5f*frequency_error));
                }
                s->source_timestamp_us[i]=source;s->source_position_deg[i]=q;s->source_velocity_deg_s[i]=v;
            }
            o->raw_phase=s->afo[i].phase_rad/(2*EXO_ADAPTIVE_PI);
            o->corrected_phase=s->event[i].phase_after;
            o->frequency_hz=s->afo[i].omega_rad_s/(2*EXO_ADAPTIVE_PI);
            o->period_s=s->event[i].period_s;
            o->confidence=s->event[i].confidence;
            if(fabsf(s->walking_mode?s->event[i].filtered_velocity:v)>s->config.afo.event_velocity_deg_s) s->last_motion_us[i]=in->timestamp_us;
            /* A stance dwell can exceed the trial's fixed 300 ms. Bound the
             * product timeout by the observed same-side cycle (0.6..1 s).
             * No fresh events still closes the separate event-age gate. */
            float stop_s=s->walking_mode?(s->event[i].period_s>0?
                fmaxf(.6f,fminf(1.0f,.6f*s->event[i].period_s)):1.0f):.3f;
            bool stopped=(float)(in->timestamp_us-s->last_motion_us[i])*1e-6f>stop_s;
            if(s->event[i].last_event_us && (float)(in->timestamp_us-s->event[i].last_event_us)*1e-6f>1.6f*s->event[i].period_s)
                s->event[i].locked=false;
            o->locked=s->event[i].locked&&!stopped&&(s->walking_mode||s->afo[i].residual_ewma<0.5f);
            o->lock_reasons|=s->event[i].lock_reasons;
            /* 重复源帧时event没有更新，但外层仍按当前控制时刻检查过期。 */
            if(s->event[i].last_event_us && (float)(in->timestamp_us-s->event[i].last_event_us)*1e-6f>1.6f*s->event[i].period_s)
                o->lock_reasons|=EXO_ADAPTIVE_LOCK_EVENT_AGE;
            if(stopped)o->lock_reasons|=EXO_ADAPTIVE_LOCK_STOPPED;
            if(!s->walking_mode&&!(s->afo[i].residual_ewma<0.5f))o->lock_reasons|=EXO_ADAPTIVE_LOCK_RESIDUAL;
            o->lock_intervals=s->event[i].lock_intervals;
            /* Keep real observer disagreement visible, even though walking
             * readiness is based on measured motion rather than harmonic fit. */
            o->lock_phase_error_rad=s->walking_mode?exo_adaptive_circular_error(
                2*EXO_ADAPTIVE_PI*(o->raw_phase-o->corrected_phase)):
                s->event[i].lock_phase_error_rad;
            o->lock_frequency_error=s->walking_mode&&o->period_s>0?
                fabsf(o->frequency_hz*o->period_s-1):s->event[i].lock_frequency_error;
            o->lock_event_age_s=s->event[i].last_event_us?(float)(in->timestamp_us-s->event[i].last_event_us)*1e-6f:1000;
            o->afo_residual=s->afo[i].residual_ewma;
            if(!o->locked)o->confidence=0;
            if(stopped) {
                s->event[i].locked=false;s->event[i].intervals=0;
                /* 静止输入不允许振荡器继续学习虚假频率；下一次运动重新捕获。 */
                exo_afo_reset(&s->afo[i],&s->config.afo,q);
                s->acquisition_seeded[i]=false;
                if(s->walking_mode) {
                    uint32_t events=s->event[i].events,rejected=s->event[i].rejected;
                    exo_adaptive_event_reset(&s->event[i]);
                    s->event[i].events=events;s->event[i].rejected=rejected;
                    s->event[i].initialized=true;s->event[i].previous_angle_deg=q;
                    s->event[i].low_deg=q;s->event[i].peak_deg=q;
                    s->event[i].peak_us=(int64_t)source;
                }
            }
            o->event=s->event[i].event;o->event_timestamp_us=s->event[i].last_event_us;
            o->accepted_events=s->event[i].events;o->rejected_events=s->event[i].rejected;
            o->event_phase_before=s->walking_mode?o->raw_phase:s->event[i].phase_before;
            o->event_phase_after=s->event[i].phase_after;
            o->phase_offset_rad=s->walking_mode?exo_adaptive_circular_error(
                2*EXO_ADAPTIVE_PI*(o->corrected_phase-o->raw_phase)):s->event[i].offset_rad;
            if(s->was_locked[i]&&!o->locked&&s->permit_epoch) {
                if(s->walking_mode) {
                    /* The measured observer owns period validity. Stop and
                     * event-age expiry already invalidate its intervals.
                     * Do not impose an additional new-peak embargo or clear
                     * its history a second time after a brief reference loss.
                     * Hard latches remain independent of measured readiness. */
                    s->was_locked[i]=false;
                } else s->rearm_required[i]=true;
            }
            if(o->locked) s->was_locked[i]=true;
            if(!o->locked) o->faults|=EXO_ADAPTIVE_FAULT_PHASE;
        }
        if(!s->permit_epoch||in->emergency) o->faults|=EXO_ADAPTIVE_FAULT_STOP;
        if(s->event[i].events<=s->arm_event_counts[i]) o->faults|=EXO_ADAPTIVE_FAULT_PHASE;
        if(s->rearm_required[i]) o->faults|=EXO_ADAPTIVE_FAULT_REARM;
        o->rearm_required=s->rearm_required[i];
        bool phase_only=s->walking_mode&&s->permit_epoch&&
            o->faults==EXO_ADAPTIVE_FAULT_PHASE&&
            o->lock_reasons==EXO_ADAPTIVE_LOCK_PHASE_ERROR;
        if(phase_only) {
            if(!s->phase_pause_since_us[i])s->phase_pause_since_us[i]=in->timestamp_us;
        } else s->phase_pause_since_us[i]=0;
        bool brief_phase_pause=phase_only&&
            in->timestamp_us-s->phase_pause_since_us[i]<=100000;
        if(o->faults) {
            /* Preserve acquired fade only while a trusted measured stride
             * survives a bounded speed pause. Recovery starts from zero torque
             * through the existing slew/power limits, never a torque step. */
            if(!((brief_speed_pause&&o->faults==EXO_ADAPTIVE_FAULT_RANGE&&o->locked)||brief_phase_pause))
                s->lambda[i]=0;
            s->previous_torque[i]=0;s->previous_desired[i]=0;
        }
        else {
            float target=c->enabled>0?1.0f:0.0f;
            s->lambda[i]+=fmaxf(-dt/c->fade_s,fminf(dt/c->fade_s,target-s->lambda[i]));
            if(!exo_adaptive_profile(c,o->corrected_phase,s->config.overall_strength,
                    s->lambda[i],&o->flex_nm,&o->ext_nm,&o->desired_nm)) o->faults|=EXO_ADAPTIVE_FAULT_CONFIG;
            s->transition_elapsed[i]+=dt;
            float blend=fminf(1.0f,s->transition_elapsed[i]/c->transition_s);
            o->desired_nm=(1-blend)*s->transition_from[i]+blend*o->desired_nm;
            o->profile_nm=o->desired_nm;
            /* Measured motion must agree with the Gaussian sign. A dwell or
             * reversal is zero assist, without restarting the acquisition fade.
             * Both angle-derived motion and motor velocity must agree. */
            bool motion_agrees=!s->walking_mode||
                (o->desired_nm*s->event[i].filtered_velocity>0&&
                 o->desired_nm*in->velocity_deg_s[i]>0&&
                 fabsf(s->event[i].filtered_velocity)>s->config.afo.event_velocity_deg_s&&
                 fabsf(in->velocity_deg_s[i])>s->config.afo.event_velocity_deg_s);
            if(!motion_agrees){o->desired_nm=0;s->previous_torque[i]=0;}
            s->previous_desired[i]=o->desired_nm;
            /* 对禁用侧不要求尚未标定的工作窗口；正常禁用通过slew退至0。 */
            if(c->torque_limit_nm>0) o->limited_nm=exo_adaptive_limit(c,o->desired_nm,
                s->previous_torque[i],in->position_deg[i],in->velocity_deg_s[i],dt,&o->faults);
            else o->limited_nm=0;
            if(o->faults) { o->limited_nm=0;s->rearm_required[i]=true;o->rearm_required=true; }
            if(s->walking_mode&&o->limited_nm*in->velocity_deg_s[i]<=0)o->limited_nm=0;
            s->previous_torque[i]=o->limited_nm;
        }
        o->lambda=s->lambda[i];
    }
    s->timestamp_us=in->timestamp_us;s->sequence=in->sequence;s->coordinate_epoch=in->coordinate_epoch;
    return true;
}
