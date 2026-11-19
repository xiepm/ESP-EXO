/* Diagnose the current regular-mode preset with synthetic kinematics.
 * These signals are test cases, not reconstructed user telemetry. No I/O. */
#include "exo_adaptive.h"
#include <math.h>
#include <stdio.h>

static int scenario(const char *name,float center,float amplitude,float hz,bool expect_range) {
    exo_adaptive_config_t c=exo_adaptive_trial_preset();
    for(unsigned side=0;side<2;side++) {
        c.leg[side].flex_amplitude_nm=.5f;c.leg[side].ext_amplitude_nm=.5f;
    }
    exo_adaptive_engine_t engine;exo_adaptive_output_t out;
    if(!exo_adaptive_engine_init(&engine,&c))return 1;
    unsigned nonzero=0,locked=0,range=0,rearm=0;
    bool range_seen=false;
    float first_q=0,first_v=0;
    for(unsigned n=0;n<3000;n++) {
        float angle=2*EXO_ADAPTIVE_PI*hz*n*.01f;
        float q=center+amplitude*cosf(angle),v=-amplitude*2*EXO_ADAPTIVE_PI*hz*sinf(angle);
        exo_adaptive_input_t in={.timestamp_us=10000+(int64_t)n*10000,.sequence=n+1,
            .coordinate_epoch=1,.permit_epoch=1,.valid={true,true},
            .position_deg={q,q},.velocity_deg_s={v,v}};
        if(!exo_adaptive_engine_update(&engine,&in,&out))return 1;
        const exo_adaptive_leg_output_t *o=&out.leg[0];
        if(o->faults&EXO_ADAPTIVE_FAULT_RANGE) {
            if(!range_seen){first_q=q;first_v=v;}
            range_seen=true;range++;
        }
        if(range_seen&&(!o->rearm_required||o->limited_nm!=0))return 1;
        nonzero+=fabsf(o->limited_nm)>.001f;locked+=o->locked;rearm+=o->rearm_required;
    }
    printf("%s: nonzero=%u locked=%u range_ticks=%u rearm_ticks=%u first_range_q=%.3f first_range_v=%.3f\n",
        name,nonzero,locked,range,rearm,(double)first_q,(double)first_v);
    if(expect_range)return range_seen&&rearm>1000?0:1;
    return !range_seen&&nonzero>1000?0:1;
}
int main(void) {
    return scenario("small_slow_reference",0,8,.7f,false)||
        scenario("angle_over_20_only",15,8,.7f,true)||
        scenario("speed_over_60_only",0,8,1.5f,true)||
        scenario("larger_faster_motion",10,17,1.2f,true);
}
