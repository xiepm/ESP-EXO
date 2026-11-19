#include "exo_adaptive.h"
#include <math.h>
#include <stdio.h>
#define REQUIRE(x) do {if(!(x)){fprintf(stderr,"FAIL %d: %s\n",__LINE__,#x);return 1;}}while(0)
int main(void) {
    exo_adaptive_config_t c=exo_adaptive_walking_preset_for_peak(15);
    REQUIRE(c.leg[0].minimum_angle_deg==-40&&c.leg[0].maximum_angle_deg==90&&
        c.leg[0].maximum_velocity_deg_s==400);
    /* Both the early walking guard and final torque limiter include the
     * boundary itself, and reject the first representable test point beyond. */
    const float cases[][3]={{-40,0,0},{90,0,0},{0,400,0},{0,-400,0},
        {-40.01f,0,1},{90.01f,0,1},{0,400.01f,1},{0,-400.01f,1}};
    for(unsigned k=0;k<sizeof(cases)/sizeof(cases[0]);k++) {
        exo_adaptive_engine_t boundary;exo_adaptive_output_t result;
        REQUIRE(exo_adaptive_engine_init(&boundary,&c));boundary.walking_mode=true;
        exo_adaptive_input_t input={.timestamp_us=10000,.sequence=1,
            .coordinate_epoch=1,.permit_epoch=1,.valid={true,true},
            .position_deg={cases[k][0],cases[k][0]},
            .velocity_deg_s={cases[k][1],cases[k][1]}};
        REQUIRE(exo_adaptive_engine_update(&boundary,&input,&result));
        REQUIRE(((result.leg[0].faults&EXO_ADAPTIVE_FAULT_RANGE)!=0)==(cases[k][2]!=0));
        uint32_t faults=0;
        (void)exo_adaptive_limit(&c.leg[0],1,0,cases[k][0],cases[k][1],.01f,&faults);
        REQUIRE(((faults&EXO_ADAPTIVE_FAULT_RANGE)!=0)==(cases[k][2]!=0));
    }
    uint32_t power_fault=0;
    float power_limited=exo_adaptive_limit(&c.leg[0],15,15,0,400,.01f,&power_fault);
    REQUIRE(power_fault==0&&power_limited>11.45f&&power_limited<11.47f);
    exo_adaptive_event_t event={.initialized=true,.previous_angle_deg=40,
        .trend=1,.peak_deg=40,.peak_us=1400000,.previous_event_us=1000000,
        .penultimate_event_us=100000,.last_event_us=1000000,.period_s=1,
        .last_peak_deg=20,.last_low_deg=0,.low_deg=0,.intervals=3,.events=4};
    exo_adaptive_walking_event_update(&event,&c.afo,34,.01f,1450000);
    REQUIRE(event.last_event_us==1400000&&event.events==4&&!event.event);
    REQUIRE(event.locked&&event.intervals==3);
    exo_adaptive_engine_t e;exo_adaptive_output_t out;
    REQUIRE(exo_adaptive_engine_init(&e,&c));e.walking_mode=true;
    unsigned nonzero=0,resumed=0;
    for(unsigned n=0;n<4000;n++) {
        float t=n*.01f,q=10+20*cosf(2*EXO_ADAPTIVE_PI*t);
        float v=-40*EXO_ADAPTIVE_PI*sinf(2*EXO_ADAPTIVE_PI*t);
        if(n>=1500&&n<2000){q=30;v=8*sinf(13*t);}
        exo_adaptive_input_t in={.timestamp_us=10000+(int64_t)n*10000,
            .sequence=n+1,.permit_epoch=1,.coordinate_epoch=1,
            .valid={true,true},.position_deg={q,q},.velocity_deg_s={v,v}};
        /* Bad harmonic fit must not claim a bad measured gait phase. */
        if(n==1225)e.afo[0].residual_ewma=2;
        REQUIRE(exo_adaptive_engine_update(&e,&in,&out));
        for(unsigned side=0;side<2;side++) {
            float tau=out.leg[side].limited_nm;
            REQUIRE(isfinite(tau)&&fabsf(tau)<=15.00001f);
            REQUIRE(tau*v>=-1e-6f);
            if(n>1600&&n<2000)REQUIRE(tau==0);
            if(n<1500)nonzero+=fabsf(tau)>.1f;
            if(n>2500)resumed+=fabsf(tau)>.1f;
        }
        if(n==1225)REQUIRE(fabsf(out.leg[0].limited_nm)>1);
    }
    REQUIRE(nonzero>1000&&resumed>1000);
    puts("PASS walking observer: larger-peak refinement, geometric phase despite harmonic residual, no reverse command, stationary angle with noisy velocity, resume");
    return 0;
}
