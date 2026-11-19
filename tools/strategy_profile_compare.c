/* Offline curve projection only. Calls production profile code, no engine/I/O. */
#include "exo_adaptive.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
int main(int argc,char **argv) {
    if(argc!=7)return 2;
    unsigned side=(unsigned)strtoul(argv[3],NULL,10);
    float flex=strtof(argv[4],NULL),ext=strtof(argv[5],NULL),amplitude=strtof(argv[6],NULL);
    if(side>1||!isfinite(flex)||!isfinite(ext)||!isfinite(amplitude)||flex<0||flex>1||ext<0||ext>1||amplitude<0||amplitude>15)return 3;
    exo_adaptive_leg_config_t c={.enabled=1,.strength=1,.flex_amplitude_nm=amplitude,
        .ext_amplitude_nm=amplitude,.flex_peak_phase=flex,.ext_peak_phase=ext,.flex_width=.1f,.ext_width=.1f};
    FILE *src=fopen(argv[1],"r"),*dst=fopen(argv[2],"w");if(!src||!dst)return 4;
    char line[1024];unsigned count=0;
    fprintf(dst,"timestamp_us,phase,velocity_deg_s,valid,projected_flex_nm,projected_ext_nm,projected_desired_nm\n");
    while(fgets(line,sizeof(line),src)) {
        if(line[0]=='t')continue;
        long long us;float ql,qr,vl,vr,pl,pr;unsigned valid_l,valid_r;
        if(sscanf(line,"%lld,%f,%f,%f,%f,%u,%u,%f,%f",&us,&ql,&qr,&vl,&vr,&valid_l,&valid_r,&pl,&pr)!=9)return 5;
        float a,b,t,phase=side?pr:pl;
        if(!exo_adaptive_profile(&c,phase,1,1,&a,&b,&t))return 6;
        fprintf(dst,"%lld,%.8f,%.8f,%u,%.8f,%.8f,%.8f\n",us,(double)phase,(double)(side?vr:vl),side?valid_r:valid_l,(double)a,(double)b,(double)t);count++;
    }
    fclose(src);fclose(dst);return count?0:7;
}
