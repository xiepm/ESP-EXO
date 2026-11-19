#include "exo_adaptive.h"
#include <math.h>
#include <string.h>
void exo_adaptive_event_reset(exo_adaptive_event_t *s) { memset(s,0,sizeof(*s)); }
/* Walking phase is observed from displacement on the current half-cycle.
 * The AFO remains a cadence/waveform observer; its harmonic fit is not proof
 * that a non-sinusoidal hip has stopped. Two measured ipsilateral intervals
 * and a full excursion are required. No extrapolated phase can drive torque.
 * 0=max flexion, .5=min flexion; existing Gaussian peaks remain .25/.75.
 * Peak/trough confirmation uses angular hysteresis, not noisy velocity zeros.
 * Bench/trial event tracking below is deliberately unchanged. */
void exo_adaptive_walking_event_update(exo_adaptive_event_t *s,
    const exo_afo_config_t *c,float q,float dt,int64_t now) {
    s->event=false;
    if(!s->initialized) {
        s->initialized=true;s->previous_angle_deg=q;
        s->low_deg=q;s->peak_deg=q;s->peak_us=now;
        s->lock_reasons=EXO_ADAPTIVE_LOCK_INTERVALS|EXO_ADAPTIVE_LOCK_EVENT_AGE;
        return;
    }
    float derivative=(q-s->previous_angle_deg)/dt;
    s->previous_angle_deg=q;
    s->filtered_velocity+=dt/(c->velocity_filter_tau_s+dt)*
        (derivative-s->filtered_velocity);
    s->low_deg=fminf(s->low_deg,q);
    if(q>=s->peak_deg){s->peak_deg=q;s->peak_us=now;}
    if(s->trend!=1&&q-s->low_deg>=c->event_excursion_deg&&
       s->filtered_velocity>c->event_velocity_deg_s) {
        s->trend=1;s->candidate_low_deg=s->low_deg;
        s->peak_deg=q;s->peak_us=now;
    }
    if(s->trend==1&&s->peak_deg-q>=c->event_excursion_deg&&
       s->filtered_velocity < -c->event_velocity_deg_s) {
        s->trend=-1;
        float period=s->previous_event_us>0?
            (float)(s->peak_us-s->previous_event_us)*1e-6f:0;
        bool first=s->previous_event_us==0;
        /* A local rebound before 55% of the last measured stride is not a
         * second gait cycle. Keep the previous full-cycle reference instead
         * of letting that half-cycle erase established cadence history. */
        bool reasonable=period>=1/c->maximum_hz&&period<=1/c->minimum_hz&&
            (s->period_s<=0||period>=.55f*s->period_s);
        if(first||reasonable) {
            float change=s->period_s>0?fabsf(period-s->period_s)/s->period_s:0;
            if(reasonable) {
                if(change>.45f)s->intervals=0;
                if(s->intervals<1000U)s->intervals++;
                s->period_s=s->period_s>0?.75f*s->period_s+.25f*period:period;
            }
            s->event=true;s->events++;s->last_event_us=s->peak_us;
            s->penultimate_event_us=s->previous_event_us;
            s->previous_event_us=s->peak_us;s->last_peak_deg=s->peak_deg;
            s->last_low_deg=s->candidate_low_deg;
        } else if(period>0&&s->period_s>0&&period<.55f*s->period_s&&
                  s->peak_deg>s->last_peak_deg+c->event_excursion_deg) {
            /* A larger peak shortly after a local maximum refines the SAME
             * stride. Do not count a second event/interval or unlock torque.
             * This is causal: the refinement happens only after its measured
             * reversal. Otherwise ignoring it makes the next real stride look
             * artificially late and triggers a false event-age timeout. */
            float full=s->penultimate_event_us>0?
                (float)(s->peak_us-s->penultimate_event_us)*1e-6f:0;
            if(full>=1/c->maximum_hz&&full<=1/c->minimum_hz)
                s->period_s=.75f*s->period_s+.25f*full;
            s->last_event_us=s->peak_us;s->previous_event_us=s->peak_us;
            s->last_peak_deg=s->peak_deg;
            s->last_low_deg=fminf(s->last_low_deg,s->candidate_low_deg);
        } else {
            s->rejected++;
            if(period>1/c->minimum_hz) {
                s->previous_event_us=s->peak_us;s->intervals=0;
            }
        }
        s->low_deg=q;
    }
    float span=s->last_peak_deg-s->last_low_deg;
    float age=s->last_event_us>0?(float)(now-s->last_event_us)*1e-6f:1000;
    s->lock_reasons=0;
    if(s->intervals<2U)s->lock_reasons|=EXO_ADAPTIVE_LOCK_INTERVALS;
    if(s->period_s<=0||age>1.6f*s->period_s)s->lock_reasons|=EXO_ADAPTIVE_LOCK_EVENT_AGE;
    if(span<c->event_excursion_deg)s->lock_reasons|=EXO_ADAPTIVE_LOCK_PHASE_ERROR;
    s->phase_before=s->phase_after;
    if(span>=c->event_excursion_deg) {
        float x=fmaxf(-1,fminf(1,2*(q-s->last_low_deg)/span-1));
        float half=acosf(x)/(2*EXO_ADAPTIVE_PI);
        s->phase_after=s->trend==1?1-half:half;
        if(s->phase_after>=1)s->phase_after=0;
    }
    s->locked=s->lock_reasons==0;s->confidence=s->locked?1:0;
    s->lock_intervals=s->intervals;s->lock_event_age_s=age;
    /* Harmonic phase/frequency errors belong to the AFO diagnostic, not this
     * directly measured phase. Never report fabricated agreement with AFO. */
    s->lock_phase_error_rad=0;s->lock_frequency_error=0;
    if(s->period_s>0&&age>1.6f*s->period_s)s->intervals=0;
}
void exo_adaptive_event_update(exo_adaptive_event_t *s,const exo_afo_config_t *c,
    float q,float v,float phase,float omega,float dt,int64_t now) {
    s->event=false;
    if(!s->initialized) {
        s->initialized=true;s->filtered_velocity=v;s->low_deg=q;s->peak_deg=q;
        s->lock_reasons=EXO_ADAPTIVE_LOCK_INTERVALS|EXO_ADAPTIVE_LOCK_EVENT_AGE;
        s->lock_event_age_s=1000;s->lock_frequency_error=1;
        s->peak_us=now;return;
    }
    /* 一阶因果速度滤波；极值时间取原角峰值，不把过零确认延迟当事件发生时刻。 */
    float a=dt/(c->velocity_filter_tau_s+dt);
    s->filtered_velocity+=a*(v-s->filtered_velocity);
    s->low_deg=fminf(s->low_deg,q);
    if(q>=s->peak_deg) { s->peak_deg=q;s->peak_us=now; }
    if(s->filtered_velocity>c->event_velocity_deg_s&&s->trend!=1) {
        s->trend=1;s->peak_deg=q;s->peak_us=now;
    }
    if(s->trend==1&&s->filtered_velocity < -c->event_velocity_deg_s) {
        s->trend=-1;
        if(s->peak_deg-s->low_deg >= c->event_excursion_deg) {
            int64_t event_us=s->peak_us;
            float period=s->previous_event_us>0?(float)(event_us-s->previous_event_us)*1e-6f:0;
            bool first=s->previous_event_us==0;
            bool reasonable=period>=1.0f/c->maximum_hz&&period<=1.0f/c->minimum_hz;
            if(first||reasonable) {
                /* 固定参考：同侧最大屈髋。没有左右交替/50%假设。 */
                s->event=true;s->events++;s->last_event_us=event_us;
                s->last_peak_deg=s->peak_deg;s->last_low_deg=s->low_deg;
                if(reasonable) {
                    float relative=s->period_s>0?fabsf(period-s->period_s)/s->period_s:0;
                    if(relative>0.45f) s->intervals=0;
                    s->period_s=s->period_s>0?0.75f*s->period_s+0.25f*period:period;
                    if(s->intervals<1000U) s->intervals++;
                }
                s->previous_event_us=event_us;
                /* 将延迟的事件相位前推到now，再取圆周误差；只更新目标offset，
                 * offset本身按rad/s渐变，事件不能硬跳到0%。 */
                float expected=omega*(float)(now-event_us)*1e-6f;
                s->target_offset_rad=exo_adaptive_circular_error(expected-phase);
                s->reference_phase_error_rad=exo_adaptive_circular_error(
                    s->target_offset_rad-s->offset_rad);
            } else {
                s->rejected++;
                /* 过短事件不污染上个有效事件；过长意味着重新捕获，当前事件作新基准。 */
                if(period>1.0f/c->minimum_hz) {
                    s->previous_event_us=event_us;s->intervals=0;s->locked=false;
                }
            }
        } else s->rejected++;
        s->low_deg=q;s->peak_deg=q;s->peak_us=now;
    }
    s->phase_before=exo_adaptive_wrap(phase+s->offset_rad)/(2*EXO_ADAPTIVE_PI);
    float err=exo_adaptive_circular_error(s->target_offset_rad-s->offset_rad);
    float step=c->correction_rate_rad_s*dt;
    s->offset_rad=exo_adaptive_circular_error(s->offset_rad+fmaxf(-step,fminf(step,err)));
    s->phase_after=exo_adaptive_wrap(phase+s->offset_rad)/(2*EXO_ADAPTIVE_PI);
    float age=s->last_event_us>0?(float)(now-s->last_event_us)*1e-6f:1000;
    /* An offset reaching its own target is not a new phase observation.
     * Keep a rejected peak rejected until a new one, and check drift between
     * peaks against the last measured same-side period (not another leg).
     * Use the existing 0.6 rad bound for every phase check. */
    float checked_err=fabsf(s->reference_phase_error_rad)>fabsf(err)
        ?s->reference_phase_error_rad:err;
    if(s->period_s>0 && s->last_event_us>0) {
        float clock_err=exo_adaptive_circular_error(
            2*EXO_ADAPTIVE_PI*(s->phase_after-age/s->period_s));
        if(fabsf(clock_err)>fabsf(checked_err))checked_err=clock_err;
    }
    float frequency_error=s->period_s>0?fabsf(omega*s->period_s/(2*EXO_ADAPTIVE_PI)-1):1;
    s->locked=s->intervals>=2U && age<=1.6f*s->period_s &&
        frequency_error<0.25f && fabsf(checked_err)<0.6f;
    /* 使用与上面相同的比较，记录确切判定周期，而非从10Hz快照反推。 */
    s->lock_reasons=0;s->lock_intervals=s->intervals;
    s->lock_phase_error_rad=checked_err;s->lock_frequency_error=frequency_error;s->lock_event_age_s=age;
    if(!(s->intervals>=2U))s->lock_reasons|=EXO_ADAPTIVE_LOCK_INTERVALS;
    if(!(age<=1.6f*s->period_s))s->lock_reasons|=EXO_ADAPTIVE_LOCK_EVENT_AGE;
    if(!(frequency_error<0.25f))s->lock_reasons|=EXO_ADAPTIVE_LOCK_FREQUENCY;
    if(!(fabsf(checked_err)<0.6f))s->lock_reasons|=EXO_ADAPTIVE_LOCK_PHASE_ERROR;
    s->confidence=s->locked?fmaxf(0,1-frequency_error):0;
    if(s->period_s>0 && age>1.6f*s->period_s) { s->intervals=0;s->locked=false; }
}
