#include "exo_bench_peak.h"
#include <math.h>
#include <string.h>
/* Gently take up/release possible fixture backlash without weakening guards.
 * The profile limits commanded slew; it does not establish fixture rigidity. */
static float rising_magnitude(float seconds, float peak) {
    float magnitude = seconds <= 0.5f ? 2.0f * seconds
                                     : 1.0f + 20.0f * (seconds - 0.5f);
    return fmaxf(0.0f, fminf(peak, magnitude));
}
bool exo_bench_peak_init(exo_bench_peak_t *s,unsigned side,float peak,float l,float r,int64_t start) {
    if(!s||side>1||!isfinite(peak)||fabsf(peak)<0.1f||fabsf(peak)>15.0f||
       !isfinite(l)||!isfinite(r)||start<=0)return false;
    memset(s,0,sizeof(*s));s->initialized=true;s->side=side;s->requested_nm=peak;
    s->centers_deg[0]=l;s->centers_deg[1]=r;s->start_us=start;return true;
}
bool exo_bench_peak_step(exo_bench_peak_t *s,const exo_sensor_snapshot_t *in,
    int64_t now,uint32_t sequence,float *out) {
    if(!out)return false;
    *out=0;
    if(!s||!s->initialized)return false;
    if(s->fault)return false;
    if(s->complete)return true;
    if(!in||now<s->start_us||in->captured_us!=now||in->sequence!=sequence||
       (s->last_us&&(now<=s->last_us||now-s->last_us>30000||sequence<=s->last_sequence))) {
        s->fault=EXO_BENCH_PEAK_TIME;return false;
    }
    if(s->samples&&in->reset_epoch!=s->coordinate_epoch) {
        s->fault=EXO_BENCH_PEAK_COORDINATES;return false;
    }
    const exo_sensor_joint_snapshot_t *j[2]={&in->left,&in->right};
    for(unsigned i=0;i<2;i++) {
        const exo_joint_sample_t *p=&j[i]->joint_sample;
        if(!j[i]->joint_valid||!j[i]->online||j[i]->motor_fault||j[i]->age_us>50000||
           !j[i]->source_timestamp_us||j[i]->source_timestamp_us>(uint64_t)now||
           (uint64_t)now-j[i]->source_timestamp_us>50000||
           (s->last_source_us[i]&&j[i]->source_timestamp_us<s->last_source_us[i])||
           !isfinite(p->position_deg)||!isfinite(p->velocity_deg_s)||
           !isfinite(p->torque_nm)||!isfinite(p->temperature_c)||in->pair_skew_us>20000) {
            s->fault=EXO_BENCH_PEAK_FEEDBACK;return false;
        }
        float displacement=fabsf(p->position_deg-s->centers_deg[i]);
        s->maximum_displacement_by_side[i]=fmaxf(s->maximum_displacement_by_side[i],displacement);
        s->maximum_speed_by_side[i]=fmaxf(s->maximum_speed_by_side[i],fabsf(p->velocity_deg_s));
        s->maximum_displacement_deg=fmaxf(s->maximum_displacement_deg,displacement);
        s->maximum_speed_deg_s=fmaxf(s->maximum_speed_deg_s,fabsf(p->velocity_deg_s));
        s->maximum_temperature_c=fmaxf(s->maximum_temperature_c,p->temperature_c);
        if(displacement>2.0f){s->fault=EXO_BENCH_PEAK_MOVED;return false;}
        if(fabsf(p->velocity_deg_s)>20.0f){s->fault=EXO_BENCH_PEAK_SPEED;return false;}
        if(p->temperature_c<0||p->temperature_c>50.0f){s->fault=EXO_BENCH_PEAK_TEMPERATURE;return false;}
    }
    float sign=s->requested_nm>0?1.0f:-1.0f,peak=fabsf(s->requested_nm);
    float estimate=sign*j[s->side]->joint_sample.torque_nm;
    if(estimate < -fmaxf(0.5f,peak*.25f)||estimate>peak*1.25f+.2f||
       fabsf(j[1-s->side]->joint_sample.torque_nm)>0.75f) {
        s->fault=EXO_BENCH_PEAK_ESTIMATE;return false;
    }
    bool new_feedback=j[s->side]->source_timestamp_us!=s->last_source_us[s->side];
    s->coordinate_epoch=in->reset_epoch;s->last_us=now;s->last_sequence=sequence;s->samples++;
    for(unsigned i=0;i<2;i++)s->last_source_us[i]=j[i]->source_timestamp_us;
    float elapsed=(float)(now-s->start_us)*1e-6f-0.5f;
    float rise=fminf(peak,1.0f)/2.0f+fmaxf(peak-1.0f,0.0f)/20.0f, command=0;
    if(elapsed>=0&&elapsed<rise)command=rising_magnitude(elapsed,peak);
    else if(elapsed>=rise&&elapsed<rise+.1f)command=peak;
    else if(elapsed>=rise+.1f&&elapsed<2*rise+.1f)command=rising_magnitude(2*rise+.1f-elapsed,peak);
    else if(elapsed>=2*rise+.1f)s->complete=true;
    command=fmaxf(0,fminf(peak,command));
    if(new_feedback) {
        s->estimate_peak_nm=fmaxf(s->estimate_peak_nm,estimate);
        /* Discard first 20 ms of the plateau to allow previous-cycle feedback. */
        if(elapsed>=rise+.02f&&elapsed<rise+.1f) {
            s->plateau_sum_nm+=estimate;s->plateau_samples++;
        }
    }
    s->command_peak_nm=fmaxf(s->command_peak_nm,command);*out=sign*command;return true;
}
