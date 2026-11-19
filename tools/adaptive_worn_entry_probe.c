/* Host-only observer/curve/limiter probe. Synthetic kinematics, no hardware,
 * plant model, runtime adapter, CAN, or deployable configuration export.
 * 5/10 Nm are numerical requested-amplitude cases, NOT human authorization.
 * Usage: entry_probe OUTPUT.csv [5|10] (default 10, refuses existing output). */
#include "exo_adaptive.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define REQUIRE(x) do { if (!(x)) { fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x); return 1; } } while (0)

/* Maximum flexion at phase zero. Smooth half-cosine descent/ascent with a
 * bottom dwell. This invented waveform is not reconstructed patient motion. */
static void motion(double t, unsigned shape, float *q, float *v) {
    const double period=1.4, pi=3.14159265358979323846;
    double p=fmod(t/period,1.0);
    if(shape==0) {
        *q=(float)(12.5*(1+cos(2*pi*p)));
        *v=(float)(-12.5*2*pi/period*sin(2*pi*p));
    } else if(p<0.32) {
        *q=(float)(12.5*(1+cos(pi*p/0.32)));
        *v=(float)(-12.5*pi/(period*0.32)*sin(pi*p/0.32));
    } else if(p<0.72) {
        *q=0; *v=0;
    } else {
        *q=(float)(12.5*(1-cos(pi*(p-0.72)/0.28)));
        *v=(float)(12.5*pi/(period*0.28)*sin(pi*(p-0.72)/0.28));
    }
}

int main(int argc,char **argv) {
    REQUIRE(argc==2||argc==3);
    REQUIRE(argc==2||strcmp(argv[2],"5")==0||strcmp(argv[2],"10")==0);
    const float amplitude=argc==3&&strcmp(argv[2],"5")==0?5.0f:10.0f;
    FILE *trace=fopen(argv[1],"wx");
    REQUIRE(trace!=NULL);
    fprintf(trace,"case,timestamp_us,permit,segment,q_deg,true_velocity_deg_s,reported_velocity_deg_s,phase,locked,rearm,reasons,faults,lambda,desired_nm,limited_nm,accepted_events,losses_in_epoch,raw_phase,omega_rad_s,offset_rad,target_offset_rad,lock_phase_error_rad,lock_frequency_error,period_s,event_us,event,intervals,residual,cosine1,cosine2,sine2,cosine3,sine3,true_phase\n");
    for(unsigned scenario=0;scenario<3;scenario++) {
        exo_adaptive_config_t c=exo_adaptive_default_config(); c.overall_strength=1;
        /* Host fixture only. These broad bounds are NOT a wearable test plan. */
        for(unsigned side=0;side<2;side++) c.leg[side]=(exo_adaptive_leg_config_t){
            .enabled=1,.strength=1,.flex_amplitude_nm=amplitude,.ext_amplitude_nm=amplitude,
            .flex_peak_phase=.75f,.ext_peak_phase=.25f,.flex_width=.1f,.ext_width=.1f,
            .torque_limit_nm=amplitude,.slew_nm_s=20,.positive_power_w=100,.negative_power_w=100,
            .minimum_angle_deg=-40,.maximum_angle_deg=60,.maximum_velocity_deg_s=350,
            .fade_s=1,.transition_s=1};
        REQUIRE(c.afo.maximum_dt_s==.03f && exo_adaptive_config_valid(&c));
        exo_adaptive_engine_t engine; exo_adaptive_output_t output;
        REQUIRE(exo_adaptive_engine_init(&engine,&c));
        unsigned previous_permit=0,losses=0,nonzero=0,high=0,opposed=0,desired_opposed=0;
        unsigned prior_permit_relocked=0,prior_permit_nonzero=0;
        bool previous_locked=false;
        for(unsigned n=0;n<6000;n++) {
            double t=n*.01; float q,v;
            /* 14 cycles, five seconds held at peak, eight cycles with the old
             * permit, then a modeled explicit new permit. No automatic rearm. */
            unsigned segment=t<19.6?0:t<24.6?1:t<35.8?2:3;
            if(segment==1) {q=25;v=0;}
            else motion(segment==0?t:t-24.6,scenario==0?0:1,&q,&v);
            float measured_v=v;
            if(scenario==2) measured_v+=5*sinf((float)(2*3.14159265358979323846*13*t));
            unsigned permit=segment<3?1:2;
            if(permit!=previous_permit) {losses=0;previous_locked=false;previous_permit=permit;}
            exo_adaptive_input_t input={.timestamp_us=10000+(int64_t)n*10000,
                .sequence=n+1,.coordinate_epoch=1,.permit_epoch=permit,.valid={true,true},
                .position_deg={q,q},.velocity_deg_s={measured_v,measured_v}};
            REQUIRE(exo_adaptive_engine_update(&engine,&input,&output));
            const exo_adaptive_leg_output_t *o=&output.leg[0];
            REQUIRE(!(o->faults&(EXO_ADAPTIVE_FAULT_TIME|EXO_ADAPTIVE_FAULT_INPUT)));
            REQUIRE(isfinite(o->limited_nm)&&fabsf(o->limited_nm)<=amplitude+0.00001f);
            if(previous_locked&&!o->locked) losses++;
            previous_locked=o->locked;
            if(losses) REQUIRE(o->rearm_required&&o->limited_nm==0);
            if(segment==2) {
                prior_permit_relocked+=o->locked;
                prior_permit_nonzero+=fabsf(o->limited_nm)>1e-5f;
            }
            nonzero+=fabsf(o->limited_nm)>1e-5f;
            if(fabsf(v)>=3 && fabsf(o->limited_nm)>=amplitude*.5f) {
                high++;
                opposed+=o->limited_nm*v<0;
            }
            desired_opposed+=fabsf(v)>=3 && fabsf(o->desired_nm)>=amplitude*.5f && o->desired_nm*v<0;
            const exo_afo_t *afo=&engine.afo[0];
            const exo_adaptive_event_t *event=&engine.event[0];
            double true_phase=segment==1?0:fmod((segment==0?t:t-24.6)/1.4,1.0);
            fprintf(trace,"%u,%lld,%u,%u,%.8f,%.8f,%.8f,%.8f,%u,%u,%u,%u,%.8f,%.8f,%.8f,%u,%u,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%lld,%u,%u,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f\n",
                scenario,(long long)input.timestamp_us,permit,segment,(double)q,(double)v,(double)measured_v,
                (double)o->corrected_phase,(unsigned)o->locked,(unsigned)o->rearm_required,
                o->lock_reasons,o->faults,(double)o->lambda,(double)o->desired_nm,(double)o->limited_nm,
                o->accepted_events,losses,(double)o->raw_phase,(double)afo->omega_rad_s,
                (double)event->offset_rad,(double)event->target_offset_rad,
                (double)o->lock_phase_error_rad,(double)o->lock_frequency_error,
                (double)o->period_s,(long long)o->event_timestamp_us,(unsigned)o->event,
                o->lock_intervals,(double)o->afo_residual,(double)afo->cosine[1],
                (double)afo->cosine[2],(double)afo->sine[2],(double)afo->cosine[3],
                (double)afo->sine[3],true_phase);
        }
        REQUIRE(prior_permit_nonzero==0);
        printf("case=%u amplitude_nm=%.0f nonzero_ticks=%u high_output_moving_ticks=%u opposed_limited_ticks=%u opposed_desired_ticks=%u old_permit_relocked=%u old_permit_nonzero=%u\n",
            scenario,(double)amplitude,nonzero,high,opposed,desired_opposed,prior_permit_relocked,prior_permit_nonzero);
    }
    REQUIRE(fclose(trace)==0);
    puts("HOST ONLY: 100 Hz synthetic input; numerical guards checked. Direction counts are observations, NOT wearable acceptance. No deployment/CAN access.");
    return 0;
}
