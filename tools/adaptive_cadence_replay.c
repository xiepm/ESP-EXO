#include "exo_adaptive.h"
#include <math.h>
#include <stdio.h>
static int fixed(float period) {
    exo_adaptive_config_t c=exo_adaptive_default_config();exo_adaptive_engine_t s;exo_adaptive_output_t out;
    if(!exo_adaptive_engine_init(&s,&c))return 1;
    unsigned locked=0;float maximum_peak_error=0;
    for(unsigned n=0;n<9000;n++) {
        float phase=2*EXO_ADAPTIVE_PI*(float)n*.01f/period;
        exo_adaptive_input_t in={.timestamp_us=10000+(int64_t)n*10000,.sequence=n+1,
            .coordinate_epoch=1,.valid={true,true},.permit_epoch=0};
        for(unsigned side=0;side<2;side++) {
            in.position_deg[side]=15+12*cosf(phase);in.velocity_deg_s[side]=-24*EXO_ADAPTIVE_PI/period*sinf(phase);
        }
        if(!exo_adaptive_engine_update(&s,&in,&out))return 1;
        if(n>=6000) {
            locked+=out.leg[0].locked;
            if(out.leg[0].event) {
                float age=(float)(in.timestamp_us-out.leg[0].event_timestamp_us)*1e-6f;
                float error=exo_adaptive_circular_error(out.leg[0].corrected_phase*2*EXO_ADAPTIVE_PI-age*2*EXO_ADAPTIVE_PI/period);
                maximum_peak_error=fmaxf(maximum_peak_error,fabsf(error));
            }
        }
        if(out.leg[0].limited_nm!=0||out.leg[1].limited_nm!=0)return 1;
    }
    printf("fixed_period_s=%.3f last30s_locked=%u/3000 maximum_event_phase_error_rad=%.6f\n",period,locked,maximum_peak_error);
    return locked<2970||maximum_peak_error>.6f;
}
static int variable(void) {
    const float periods[]={.65f,.96f,.79f,.66f,.70f,.69f,.56f,1.68f,.72f,.67f,.70f,1.30f,.84f,.86f,.94f,.98f,1.73f,.81f,.99f,.92f};
    exo_adaptive_config_t c=exo_adaptive_default_config();exo_adaptive_engine_t s;exo_adaptive_output_t out;
    if(!exo_adaptive_engine_init(&s,&c))return 1;
    float phase=0;unsigned cycle=0,losses=0,settled=0;bool previous=false;
    for(unsigned n=0;n<15000;n++) {
        float period=n<3000?1.0f:cycle<sizeof(periods)/sizeof(periods[0])?periods[cycle]:1.0f;
        phase+=2*EXO_ADAPTIVE_PI*.01f/period;
        if(phase>=2*EXO_ADAPTIVE_PI){phase-=2*EXO_ADAPTIVE_PI;if(n>=3000)cycle++;}
        exo_adaptive_input_t in={.timestamp_us=10000+(int64_t)n*10000,.sequence=n+1,
            .coordinate_epoch=1,.valid={true,true},.permit_epoch=0};
        for(unsigned side=0;side<2;side++) {
            in.position_deg[side]=15+12*cosf(phase);
            in.velocity_deg_s[side]=-24*EXO_ADAPTIVE_PI/period*sinf(phase);
        }
        if(!exo_adaptive_engine_update(&s,&in,&out))return 1;
        if(n>=3000&&n<8000&&previous&&!out.leg[0].locked)losses++;
        previous=out.leg[0].locked;
        if(n>=12000)settled+=out.leg[0].locked;
    }
    printf("variable_cadence_synthetic losses=%u after_return_to_1Hz_last30s_locked=%u/3000\n",losses,settled);
    return losses==0||settled<2970;
}
int main(void) {
    float periods[]={.56f,.7f,.9f,1.0f,1.3f,1.73f};
    for(unsigned i=0;i<sizeof(periods)/sizeof(periods[0]);i++)if(fixed(periods[i]))return 1;
    if(variable())return 1;
    puts("PASS fixed/variable-cadence real C observer; synthetic inputs only, not hardware lock certification; no output permit");
    return 0;
}
