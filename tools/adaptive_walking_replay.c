/* 100 Hz synthetic walking regression; no device, CAN, or measured gait claim. */
#include "exo_adaptive.h"
#include <math.h>
#include <stdio.h>
#define CHECK(x) do { if(!(x)) {fprintf(stderr,"FAIL %s:%d: %s\n",__FILE__,__LINE__,#x);return false;} } while(0)
static void motion(float p,float hz,unsigned shape,float *q,float *v) {
    p-=floorf(p);
    if(shape==0) {
        *q=10+20*cosf(2*EXO_ADAPTIVE_PI*p);
        *v=-40*EXO_ADAPTIVE_PI*hz*sinf(2*EXO_ADAPTIVE_PI*p);
    } else if(p<.32f) {
        *q=15*(1+cosf(EXO_ADAPTIVE_PI*p/.32f));
        *v=-15*EXO_ADAPTIVE_PI*hz/.32f*sinf(EXO_ADAPTIVE_PI*p/.32f);
    } else if(p<.72f) { *q=0;*v=0; }
    else {
        *q=15*(1-cosf(EXO_ADAPTIVE_PI*(p-.72f)/.28f));
        *v=15*EXO_ADAPTIVE_PI*hz/.28f*sinf(EXO_ADAPTIVE_PI*(p-.72f)/.28f);
    }
}
static bool scenario(unsigned shape,float initial_hz,bool noise,float peak) {
    exo_adaptive_config_t c=exo_adaptive_walking_preset_for_peak(peak);
    exo_adaptive_engine_t e;exo_adaptive_output_t out;
    CHECK(exo_adaptive_engine_init(&e,&c));e.walking_mode=true;
    unsigned nz[4][2]={{0}},range=0,rearm=0,cycle[2]={0},seen[2]={0},verified[2]={0};float phase[2]={0,.5f},maximum[2]={0};
    for(unsigned n=0;n<12000;n++) {
        float t=n*.01f;unsigned segment=t<30?0:t<60?1:t<65?2:3;
        float hz=t<30?initial_hz:t<40?initial_hz+(t-30)*(1.4f-initial_hz)/10:1.4f;
        exo_adaptive_input_t in={.timestamp_us=10000+(int64_t)n*10000,.sequence=n+1,
            .coordinate_epoch=1,.permit_epoch=1,.valid={true,true}};
        for(unsigned i=0;i<2;i++) {
            float f=hz*(i?.95f:1);
            motion(phase[i],f,shape,&in.position_deg[i],&in.velocity_deg_s[i]);
            if(segment==2) in.velocity_deg_s[i]=0;
            else phase[i]+=f*.01f;
            if(noise)in.velocity_deg_s[i]+=2*sinf(2*EXO_ADAPTIVE_PI*13*t);
        }
        CHECK(exo_adaptive_engine_update(&e,&in,&out));
        for(unsigned i=0;i<2;i++) {
            const exo_adaptive_leg_output_t *o=&out.leg[i];
            CHECK(isfinite(o->limited_nm)&&fabsf(o->limited_nm)<=peak+.00001f);
            maximum[i]=fmaxf(maximum[i],fabsf(o->limited_nm));
            unsigned current=(unsigned)phase[i];
            if(current!=cycle[i]) {
                if((segment==0&&t>15)||(segment==3&&t>75)) {
                    CHECK(seen[i]==3U);verified[i]++;
                }
                cycle[i]=current;seen[i]=0;
            }
            if(o->limited_nm>.001f)seen[i]|=1;
            if(o->limited_nm<-.001f)seen[i]|=2;
            CHECK(o->limited_nm*in.velocity_deg_s[i]>=-1e-6f);
            if(in.velocity_deg_s[i]==0)CHECK(o->limited_nm==0);
            range+=(o->faults&EXO_ADAPTIVE_FAULT_RANGE)!=0;rearm+=o->rearm_required;
            nz[segment][i]+=fabsf(o->limited_nm)>.01f;
            if(segment==2&&t>61.1f)CHECK(o->limited_nm==0);
            if(o->faults||!o->locked)CHECK(o->limited_nm==0);
        }
    }
    printf("walking shape=%u initial_hz=%.2f noise=%u nonzero L/R steady=%u/%u cadence=%u/%u resumed=%u/%u range=%u rearm=%u\n",
        shape,initial_hz,noise,nz[0][0],nz[0][1],nz[1][0],nz[1][1],nz[3][0],nz[3][1],range,rearm);
    CHECK(range==0&&rearm==0);
    printf("command peak request=%.2f actual L/R=%.4f/%.4f Nm\n",peak,maximum[0],maximum[1]);
    if(peak==15)CHECK(maximum[0]>14.8f&&maximum[1]>14.8f);
    /* Cycle coverage replaces nonzero-time quota: stance dwell must be zero. */
    for(unsigned i=0;i<2;i++)CHECK(verified[i]>20&&nz[0][i]>0&&nz[1][i]>0&&nz[3][i]>0);
    return true;
}
static bool geometric_phase_ignores_harmonic_residual(void) {
    exo_adaptive_config_t c=exo_adaptive_walking_preset_for_peak(15);
    exo_adaptive_engine_t e;exo_adaptive_output_t out;
    CHECK(exo_adaptive_engine_init(&e,&c));e.walking_mode=true;
    for(unsigned n=0;n<3500;n++) {
        exo_adaptive_input_t in={.timestamp_us=10000+(int64_t)n*10000,
            .sequence=n+1,.coordinate_epoch=1,.permit_epoch=1,.valid={true,true}};
        for(unsigned i=0;i<2;i++)motion(n*.01f,1,0,&in.position_deg[i],&in.velocity_deg_s[i]);
        if(n==3025)e.afo[0].residual_ewma=2;
        CHECK(exo_adaptive_engine_update(&e,&in,&out));
        CHECK(fabsf(out.leg[0].limited_nm-out.leg[1].limited_nm)<1e-6f);
        if(n==3025)CHECK(fabsf(out.leg[0].limited_nm)>10);
        CHECK(!out.leg[0].rearm_required);
    }
    puts("PASS measured walking phase independent of harmonic reconstruction residual");
    return true;
}
static bool operating_window(bool invalid_during_pause) {
    exo_adaptive_config_t c=exo_adaptive_walking_preset_for_peak(15);
    exo_adaptive_engine_t e;exo_adaptive_output_t out;
    CHECK(exo_adaptive_engine_init(&e,&c));e.walking_mode=true;
    unsigned pauses=0,resumed=0;
    for(unsigned n=0;n<2500;n++) {
        float t=n*.01f;
        exo_adaptive_input_t in={.timestamp_us=10000+(int64_t)n*10000,.sequence=n+1,
            .coordinate_epoch=1,.permit_epoch=1,.valid={true,true}};
        for(unsigned i=0;i<2;i++) {
            motion(t,1,0,&in.position_deg[i],&in.velocity_deg_s[i]);
            if(t>10&&t<12) {
                float x=EXO_ADAPTIVE_PI*(t-10)/2;
                in.position_deg[i]+=80*sinf(x)*sinf(x);
                in.velocity_deg_s[i]+=40*EXO_ADAPTIVE_PI*sinf(2*x);
            }
        }
        if(invalid_during_pause&&n==1100)in.valid[0]=in.valid[1]=false;
        CHECK(exo_adaptive_engine_update(&e,&in,&out));
        for(unsigned i=0;i<2;i++) {
            bool outside=in.position_deg[i]>c.leg[i].maximum_angle_deg||
                in.position_deg[i]<c.leg[i].minimum_angle_deg||
                fabsf(in.velocity_deg_s[i])>c.leg[i].maximum_velocity_deg_s;
            if(outside){CHECK(out.leg[i].limited_nm==0);pauses++;}
            if(invalid_during_pause&&n>=1100)CHECK(out.leg[i].rearm_required&&out.leg[i].limited_nm==0);
            else CHECK(!out.leg[i].rearm_required);
            if(n>1500)resumed+=fabsf(out.leg[i].limited_nm)>.1f;
        }
    }
    CHECK(pauses>0);CHECK(invalid_during_pause?resumed==0:resumed>1500);
    printf("PASS angle-window pause/reacquisition invalid=%u paused=%u resumed=%u\n",invalid_during_pause,pauses,resumed);return true;
}
int main(void) {
    if(!operating_window(false)||!operating_window(true))return 1;
    if(!geometric_phase_ignores_harmonic_residual())return 1;
    for(unsigned shape=0;shape<2;shape++)
        for(unsigned hz=0;hz<3;hz++)
            for(unsigned level=0;level<2;level++)
                if(!scenario(shape,.5f+.2f*hz,hz==2,level?15.f:.5f))return 1;
    return 0;
}
