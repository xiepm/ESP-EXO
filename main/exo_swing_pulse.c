#include "exo_swing_pulse.h"
#include <math.h>
#include <string.h>

#define PULSE_PI 3.14159265358979323846f

exo_pulse_timing_t exo_swing_pulse_default_timing(uint16_t peak_mnm) {
    return (exo_pulse_timing_t){.schema_version=1, .duration_mode=EXO_PULSE_FIXED,
        .peak_mnm=peak_mnm, .fixed_ms=600, .ratio_permille=300,
        .minimum_ms=150, .maximum_ms=1500,
        .period_minimum_ms=400, .period_maximum_ms=3000};
}
exo_pulse_window_t exo_swing_pulse_default_window(uint16_t trigger_cdeg) {
    return (exo_pulse_window_t){.reset_cdeg=(int16_t)((int)trigger_cdeg-200),
        .window_maximum_cdeg=(uint16_t)(trigger_cdeg+2000)};
}
bool exo_swing_pulse_timing_valid(const exo_pulse_timing_t *t) {
    return t && t->schema_version==1 && t->duration_mode<=EXO_PULSE_PERIOD_SCALED &&
        t->peak_mnm<=EXO_PULSE_PEAK_HARD_MAX_MNM && t->fixed_ms>=100 && t->fixed_ms<=EXO_PULSE_FIXED_MAX_MS &&
        t->ratio_permille>=100 && t->ratio_permille<=600 &&
        t->minimum_ms>=100 && t->maximum_ms<=EXO_PULSE_SCALED_MAX_MS && t->minimum_ms<=t->maximum_ms &&
        t->period_minimum_ms>=300 && t->period_maximum_ms<=5000 &&
        t->period_minimum_ms<t->period_maximum_ms;
}
bool exo_swing_pulse_window_valid(const exo_pulse_window_t *w, uint16_t threshold) {
    return w && threshold>=100 && threshold<=6000 && w->reset_cdeg>=-3000 &&
        (int)w->reset_cdeg<=(int)threshold-100 &&
        w->window_maximum_cdeg>=threshold+200 && w->window_maximum_cdeg<=9000;
}
void exo_swing_pulse_reset(exo_swing_pulse_leg_t *leg) {
    if (leg) memset(leg,0,sizeof(*leg));
}
static float exit_pulse(exo_swing_pulse_leg_t *leg, exo_pulse_exit_t reason) {
    leg->state=EXO_PULSE_WAIT_NEXT_RESET;
    leg->exit_reason=(uint8_t)reason;
    leg->reset_started_us=0;
    leg->reset_samples=0;
    leg->progress_01=0;
    return 0;
}
/* 仅使用本侧经过复位/正向确认的新事件。短间隔是假事件，不移动有效锚点；
 * 长间隔/节律突变使周期失去可信度，重新采集两个合理间隔。
 * 有效间隔做0.25一阶平滑；两侧状态彼此完全独立。 */
static bool accept_event_period(exo_swing_pulse_leg_t *leg,
    const exo_pulse_timing_t *t, int64_t now) {
    const int64_t previous=leg->last_valid_event_us;
    if (previous==0) {
        leg->last_valid_event_us=now;
        ++leg->event_count;
        return true;
    }
    const int64_t interval=now-previous;
    if (interval<(int64_t)t->period_minimum_ms*1000) {
        leg->valid_period_intervals=0;
        leg->estimated_period_us=0;
        return false;
    }
    leg->last_valid_event_us=now;
    ++leg->event_count;
    if (interval>(int64_t)t->period_maximum_ms*1000) {
        leg->valid_period_intervals=0;
        leg->estimated_period_us=0;
        return true;
    }
    const float sample=(float)interval;
    if (leg->estimated_period_us>0 &&
        fabsf(sample-leg->estimated_period_us)>leg->estimated_period_us*0.5f) {
        leg->estimated_period_us=sample;
        leg->valid_period_intervals=1;
    } else {
        leg->estimated_period_us=leg->estimated_period_us>0 ?
            leg->estimated_period_us+0.25f*(sample-leg->estimated_period_us) : sample;
        if (leg->valid_period_intervals<2) ++leg->valid_period_intervals;
    }
    return true;
}
static float begin_event(exo_swing_pulse_leg_t *leg, const exo_pulse_timing_t *t,
    const exo_pulse_window_t *window, uint8_t level, int64_t now) {
    if (!accept_event_period(leg,t,now))
        return exit_pulse(leg,EXO_PULSE_EXIT_EVENT_REJECTED);
    float duration_us=(float)t->fixed_ms*1000;
    if (t->duration_mode==EXO_PULSE_PERIOD_SCALED) {
        if (leg->valid_period_intervals<2 || !isfinite(leg->estimated_period_us) ||
            leg->estimated_period_us<(float)t->period_minimum_ms*1000 ||
            leg->estimated_period_us>(float)t->period_maximum_ms*1000)
            return exit_pulse(leg,EXO_PULSE_EXIT_PERIOD_UNTRUSTED);
        duration_us=fminf((float)t->maximum_ms*1000,
            fmaxf((float)t->minimum_ms*1000,
                (float)t->ratio_permille*0.001f*leg->estimated_period_us));
    }
    leg->frozen_duration_us=(uint32_t)lroundf(duration_us);
    const float requested=((float)t->peak_mnm/1000.0f)*(float)level/100.0f;
    /* sin²的最大正斜率=pi*A/Ta。直接冻结受约束的A而不在波形外追加滞后滤波，
     * 因而脉冲严格在Ta结束，不为追赶申请峰值而延长或保持力矩。
     * 20 N·m/s下，FIXED=3秒、申请15 N·m时A可完整冻结为15。
     * 小于2357 ms的15 N·m申请仍可能受速率边界削低。 */
    const float rate_peak=EXO_PULSE_MAX_RISE_NM_S*(float)leg->frozen_duration_us*0.000001f/PULSE_PI;
    leg->frozen_peak_nm=fminf(requested,rate_peak);
    leg->peak_rate_limited=requested>rate_peak;
    leg->pulse_started_us=now;
    leg->stationary_started_us=0;leg->stationary_samples=0;
    leg->frozen_reset_cdeg=window->reset_cdeg;
    leg->frozen_window_maximum_cdeg=window->window_maximum_cdeg;
    leg->progress_01=0;
    leg->state=EXO_PULSE_ACTIVE;
    leg->exit_reason=EXO_PULSE_EXIT_NONE;
    ++leg->pulse_count;
    return 0; /* u=0，始端力矩与导数都是0。 */
}

bool exo_swing_pulse_input_valid(const exo_swing_pulse_leg_t *leg,
    const exo_swing_pulse_input_t *in) {
    if(!leg || !in || !in->valid || !isfinite(in->angle_deg) || !isfinite(in->velocity_deg_s) ||
        in->now_us<=0 || in->source_us==0 || in->feedback_count==0 ||
        (leg->last_now_us!=0 && in->now_us<=leg->last_now_us) ||
        in->source_us>(uint64_t)in->now_us ||
        (uint64_t)in->now_us-in->source_us>EXO_PULSE_FRESHNESS_US) return false;
    if(leg->have_sample && (in->source_us<leg->last_source_us ||
        in->source_us-leg->last_source_us>EXO_PULSE_FRESHNESS_US ||
        (int32_t)(in->feedback_count-leg->last_feedback_count)<0 ||
        ((in->source_us!=leg->last_source_us)!=(in->feedback_count!=leg->last_feedback_count))))
        return false;
    return true;
}
float exo_swing_pulse_step(exo_swing_pulse_leg_t *leg,
    const exo_pulse_timing_t *t, const exo_pulse_window_t *window,
    uint16_t threshold_cdeg, uint8_t level,
    const exo_swing_pulse_input_t *in) {
    if (!leg) return 0;
    if (!exo_swing_pulse_timing_valid(t) ||
        !exo_swing_pulse_window_valid(window,threshold_cdeg) || level>100 ||
        !exo_swing_pulse_input_valid(leg,in)) {
        /* 失效恢复不能复用旧周期；必须重新复位并收集新事件。 */
        exo_swing_pulse_reset(leg);
        leg->exit_reason=EXO_PULSE_EXIT_FEEDBACK;
        return 0;
    }
    const bool fresh=!leg->have_sample ||
        (in->source_us>leg->last_source_us && in->feedback_count!=leg->last_feedback_count);
    leg->last_now_us=in->now_us;
    /* 复位角改变后不能沿用旧READY/候选事件或旧周期可信度。
     * 当前脉冲仍保留冻结A/Ta及更严格的窗口，下一事件须按新规则重新复位。 */
    if(leg->have_window && leg->observed_reset_cdeg!=window->reset_cdeg) {
        leg->valid_period_intervals=0;leg->estimated_period_us=0;leg->last_valid_event_us=0;
        if(leg->state!=EXO_PULSE_ACTIVE) {
            leg->state=EXO_PULSE_WAIT_RESET;leg->reset_started_us=0;leg->reset_samples=0;
            leg->confirmation_started_us=0;leg->confirmation_samples=0;
        }
    }
    leg->observed_reset_cdeg=window->reset_cdeg;leg->have_window=true;
    /* 超时只使旧周期不再可信，不生成任何事件或下一脉冲。 */
    if(leg->last_valid_event_us!=0 && in->now_us-leg->last_valid_event_us>(int64_t)t->period_maximum_ms*1000) {
        leg->valid_period_intervals=0;
        leg->estimated_period_us=0;
    }
    const float angle=in->angle_deg;
    const float trigger=(float)threshold_cdeg*0.01f;
    const float reset=(float)window->reset_cdeg*0.01f;
    const float maximum=(float)window->window_maximum_cdeg*0.01f;
    float output=0;
    if (leg->state==EXO_PULSE_ACTIVE) {
        /* 提前退出优先于Ta；动作反向/窗口失效时不能强行放完脉冲。 */
        const float pulse_reset=fmaxf(reset,(float)leg->frozen_reset_cdeg*0.01f);
        const float pulse_maximum=fminf(maximum,(float)leg->frozen_window_maximum_cdeg*0.01f);
        if (angle<pulse_reset || angle>pulse_maximum) output=exit_pulse(leg,EXO_PULSE_EXIT_WINDOW);
        else if (in->velocity_deg_s<EXO_PULSE_REVERSE_SPEED_DEG_S ||
            (fresh && leg->have_sample && angle<leg->previous_angle_deg-0.25f))
            output=exit_pulse(leg,EXO_PULSE_EXIT_REVERSE);
        else {
            /* 真实新反馈确认停止/低速保持100 ms便提前退出；重复帧不推进确认。
             * 停止检测只负责退出，绝不作为重复助力的定时器。 */
            if(fresh) {
                if(fabsf(in->velocity_deg_s)<=EXO_PULSE_STATIONARY_SPEED_DEG_S &&
                    fabsf(angle-leg->previous_angle_deg)<=0.05f) {
                    if(leg->stationary_started_us==0) leg->stationary_started_us=(int64_t)in->source_us;
                    if(leg->stationary_samples<255) ++leg->stationary_samples;
                } else {leg->stationary_started_us=0;leg->stationary_samples=0;}
            }
            const int64_t elapsed=in->now_us-leg->pulse_started_us;
            if(leg->stationary_started_us!=0 && leg->stationary_samples>=5 &&
                (int64_t)in->source_us-leg->stationary_started_us>=EXO_PULSE_STATIONARY_DWELL_US)
                output=exit_pulse(leg,EXO_PULSE_EXIT_MOTION_STOPPED);
            else if (elapsed<0) output=exit_pulse(leg,EXO_PULSE_EXIT_FEEDBACK);
            else if (elapsed>=(int64_t)leg->frozen_duration_us)
                output=exit_pulse(leg,EXO_PULSE_EXIT_COMPLETE);
            else {
                leg->progress_01=(float)elapsed/(float)leg->frozen_duration_us;
                const float sine=sinf(PULSE_PI*leg->progress_01);
                output=leg->frozen_peak_nm*sine*sine;
            }
        }
    } else if (fresh) {
        if (leg->state==EXO_PULSE_WAIT_RESET || leg->state==EXO_PULSE_WAIT_NEXT_RESET) {
            /* 复位要求真实反馈连续位于低角区80 ms；时钟/重复快照不能替代返回动作。 */
            if (angle<=reset && angle>=-30 && in->velocity_deg_s<=EXO_PULSE_EVENT_SPEED_DEG_S) {
                if (leg->reset_started_us==0) {leg->reset_started_us=(int64_t)in->source_us;leg->reset_samples=0;}
                if(leg->reset_samples<255) ++leg->reset_samples;
                if ((int64_t)in->source_us-leg->reset_started_us>=EXO_PULSE_RESET_DWELL_US && leg->reset_samples>=5) {
                    leg->state=EXO_PULSE_READY;
                    leg->exit_reason=EXO_PULSE_EXIT_NONE;
                    leg->reset_started_us=0;
                    leg->reset_samples=0;
                }
            } else {leg->reset_started_us=0;leg->reset_samples=0;}
        } else if (leg->state==EXO_PULSE_READY) {
            if (angle>maximum || angle<-30) output=exit_pulse(leg,EXO_PULSE_EXIT_WINDOW);
            else if (leg->have_sample && leg->previous_angle_deg<=trigger && angle>trigger) {
                if (in->velocity_deg_s<EXO_PULSE_EVENT_SPEED_DEG_S)
                    output=exit_pulse(leg,EXO_PULSE_EXIT_EVENT_REJECTED);
                else {
                    leg->state=EXO_PULSE_CONFIRM_EVENT;
                    leg->confirmation_started_us=(int64_t)in->source_us;
                    leg->crossing_angle_deg=angle;
                    leg->confirmation_samples=1;
                }
            }
        } else if (leg->state==EXO_PULSE_CONFIRM_EVENT) {
            if(leg->confirmation_samples<255) ++leg->confirmation_samples;
            const int64_t confirmed_us=(int64_t)in->source_us-leg->confirmation_started_us;
            if (angle<=trigger || angle>maximum ||
                in->velocity_deg_s<EXO_PULSE_EVENT_SPEED_DEG_S ||
                angle<leg->previous_angle_deg-0.05f ||
                confirmed_us>EXO_PULSE_EVENT_TIMEOUT_US)
                output=exit_pulse(leg,EXO_PULSE_EXIT_EVENT_REJECTED);
            else if (confirmed_us>=EXO_PULSE_EVENT_CONFIRM_US && leg->confirmation_samples>=4 &&
                angle-leg->crossing_angle_deg>=0.5f)
                output=begin_event(leg,t,window,level,in->now_us);
        }
    }
    if (fresh) {
        leg->previous_angle_deg=angle;
        leg->last_source_us=in->source_us;
        leg->last_feedback_count=in->feedback_count;
        leg->have_sample=true;
    }
    return output;
}
