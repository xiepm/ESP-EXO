#include "exo_afo_trace_wire.h"
#include <stdio.h>
int exo_afo_trace_format(char *buffer,size_t size,const exo_afo_trace_t *r,unsigned side,bool first) {
    if(!buffer||!size||!r||side>1)return -1;
    const exo_afo_trace_leg_t *l=&r->leg[side];
    int n=snprintf(buffer,size,
        "%s,1,%lu,%lld,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%ld,%lu,%lu,%u,"
        "%llu,%lu,%lu,%lu,%lu,%llu,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%lu,%lu,%lu,%lu,%.7g,%lu",
        first?"AFOFIRST":"AFOROW",(unsigned long)r->id,(long long)r->time_us,
        (unsigned long)r->sequence,(unsigned long)r->permit,(unsigned long)r->coordinate_epoch,
        (unsigned long)r->generation,(unsigned long)r->system_state,(unsigned long)r->valid_mask,
        (unsigned long)r->quality,(unsigned long)r->skew_us,(unsigned long)r->previous_execution_us,
        (unsigned long)r->dt_us,(long)r->system_result,(unsigned long)r->gate_flags,(unsigned long)r->reject_reason,side,
        (unsigned long long)l->source_us,(unsigned long)l->source_count,(unsigned long)l->age_us,
        (unsigned long)l->source_interval_us,(unsigned long)l->sensor_flags,(unsigned long long)l->previous_source_us,
        (double)l->previous_q,(double)l->previous_v,(double)l->q,(double)l->v,(double)l->motor_estimate_nm,(double)l->temperature_c,
        (double)l->filtered_v,(double)l->phase,(double)l->period_s,(double)l->lambda,(double)l->profile_nm,(double)l->desired_nm,
        (double)l->limited_nm,(double)l->suggestion_nm,(double)l->final_nm,(unsigned long)l->flags,
        (unsigned long)l->faults,(unsigned long)l->reasons,(unsigned long)l->intervals,(double)l->event_age_s,
        (unsigned long)l->category);
    if(n<0||(size_t)n>=size){buffer[0]='\0';return -1;}
    return n;
}
