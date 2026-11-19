/* Exact supplied configuration/timestamps. No built-in demo, no hardware I/O. */
#include "exo_adaptive.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

int main(int argc,char **argv) {
    if(argc!=4)return 2;
    FILE *config=fopen(argv[1],"rb");if(!config)return 3;
    uint8_t blob[EXO_ADAPTIVE_CONFIG_BYTES+1];size_t size=fread(blob,1,sizeof(blob),config);fclose(config);
    exo_adaptive_config_t c;exo_adaptive_engine_t engine;
    if(!exo_adaptive_config_decode(&c,blob,(uint32_t)size)||!exo_adaptive_engine_init(&engine,&c))return 4;
    FILE *src=fopen(argv[2],"r"),*dst=fopen(argv[3],"w");if(!src||!dst)return 5;
    char line[1024];unsigned count=0;long long previous=0;
    fprintf(dst,"timestamp_us,side,locked,phase,desired_nm,limited_nm,faults,historical_phase,current_profile_at_historical_phase_nm\n");
    while(fgets(line,sizeof(line),src)) {
        long long us;float ql,qr,vl,vr,pl,pr;unsigned valid_l,valid_r;
        if(count==0&&line[0]=='t')continue;
        if(sscanf(line,"%lld,%f,%f,%f,%f,%u,%u,%f,%f",&us,&ql,&qr,&vl,&vr,&valid_l,&valid_r,&pl,&pr)!=9||us<=previous)return 6;
        previous=us;
        exo_adaptive_input_t in={.timestamp_us=us,.sequence=++count,.coordinate_epoch=1,
            .position_deg={ql,qr},.velocity_deg_s={vl,vr},.valid={valid_l!=0,valid_r!=0},
            .permit_epoch=0}; /* Actual current selection is TRANSPARENT/DISABLED. */
        exo_adaptive_output_t out;if(!exo_adaptive_engine_update(&engine,&in,&out))return 7;
        float phase[2]={pl,pr};
        for(unsigned side=0;side<2;side++) {
            float flex,ext,desired;
            /* Curve-only calculation: lambda=1 is explicit, not a simulated lock. */
            if(!exo_adaptive_profile(&c.leg[side],phase[side],c.overall_strength,1,&flex,&ext,&desired))return 8;
            const exo_adaptive_leg_output_t *o=&out.leg[side];
            fprintf(dst,"%lld,%u,%u,%.8f,%.8f,%.8f,%u,%.8f,%.8f\n",us,side,
                (unsigned)o->locked,(double)o->corrected_phase,(double)o->desired_nm,
                (double)o->limited_nm,o->faults,(double)phase[side],(double)desired);
        }
    }
    fclose(src);fclose(dst);printf("Replayed %u original paired timestamps; maximum_dt_s=%.6f permit=0; no hardware I/O\n",count,(double)c.afo.maximum_dt_s);
    return count?0:9;
}
