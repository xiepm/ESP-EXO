/* Offline interpolation probe, NOT a reconstruction of unrecorded CAN samples. */
#include "exo_adaptive.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
int main(int argc,char **argv) {
    if(argc!=3)return 2;
    FILE *input=fopen(argv[1],"r"),*trace=fopen(argv[2],"w");
    if(!input||!trace)return 2;
    char line[1024];if(!fgets(line,sizeof(line),input))return 2;
    fprintf(trace,"trial,time_us,side,q,v,locked,rearm,faults,reasons,residual,phase_error,frequency_error,events,intervals,lambda,torque,phase\n");
    exo_adaptive_config_t config=exo_adaptive_walking_preset_for_peak(15);
    exo_adaptive_engine_t e;exo_adaptive_output_t out;
    unsigned current=999,n=0;
    while(fgets(line,sizeof(line),input)) {
        unsigned trial;long long us;float ql,vl,qr,vr;
        if(sscanf(line,"%u,%lld,%f,%f,%f,%f",&trial,&us,&ql,&vl,&qr,&vr)!=6)return 3;
        if(trial!=current) {
            if(!exo_adaptive_engine_init(&e,&config))return 4;
            e.walking_mode=true;current=trial;n=0;
        }
        exo_adaptive_input_t in={.timestamp_us=us,.sequence=++n,.coordinate_epoch=1,.permit_epoch=1,
            .valid={true,true},.position_deg={ql,qr},.velocity_deg_s={vl,vr}};
        if(!exo_adaptive_engine_update(&e,&in,&out))return 5;
        for(unsigned i=0;i<2;i++) {
            const exo_adaptive_leg_output_t *o=&out.leg[i];
            if(!isfinite(o->limited_nm)||fabsf(o->limited_nm)>15.0001f)return 6;
            if(o->faults&&o->limited_nm!=0)return 7;
            fprintf(trace,"%u,%lld,%u,%.5f,%.5f,%u,%u,%u,%u,%.5f,%.5f,%.5f,%u,%u,%.5f,%.5f,%.5f\n",
                trial,us,i,(double)in.position_deg[i],(double)in.velocity_deg_s[i],o->locked,o->rearm_required,
                o->faults,o->lock_reasons,(double)o->afo_residual,(double)o->lock_phase_error_rad,
                (double)o->lock_frequency_error,o->accepted_events,o->lock_intervals,(double)o->lambda,
                (double)o->limited_nm,(double)o->corrected_phase);
        }
    }
    fclose(input);fclose(trace);return 0;
}
