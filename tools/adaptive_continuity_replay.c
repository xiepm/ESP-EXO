/* Adversarial walking continuity tests; host-only, no actuator access. */
#include "exo_adaptive.h"
#include <math.h>
#include <stdio.h>
#define CHECK(x) do { if(!(x)){fprintf(stderr,"FAIL %d: %s\n",__LINE__,#x);return 1;} } while(0)
static exo_adaptive_input_t sample(unsigned n) {
    float phase=2*EXO_ADAPTIVE_PI*(float)n*.01f;
    float q=10+20*cosf(phase),v=-40*EXO_ADAPTIVE_PI*sinf(phase);
    return (exo_adaptive_input_t){.timestamp_us=10000+(int64_t)n*10000,
        .sequence=n+1,.permit_epoch=1,.coordinate_epoch=1,.valid={true,true},
        .position_deg={q,q},.velocity_deg_s={v,v}};
}
static int phase_pause_case(bool prolonged,bool invalid) {
    exo_adaptive_config_t c=exo_adaptive_walking_preset_for_peak(15);
    exo_adaptive_engine_t e;exo_adaptive_output_t out;
    CHECK(exo_adaptive_engine_init(&e,&c));e.walking_mode=true;
    float previous_low=0;unsigned events=0,end=prolonged?1040:1026;
    for(unsigned n=0;n<1050;n++) {
        exo_adaptive_input_t in=sample(n);
        if(n==1025){previous_low=e.event[0].last_low_deg;events=e.event[0].events;}
        if(n>=1025&&n<end)e.event[0].last_low_deg=e.event[0].last_peak_deg;
        if(n==end)e.event[0].last_low_deg=previous_low;
        if(n==1025&&invalid)in.valid[0]=false;
        CHECK(exo_adaptive_engine_update(&e,&in,&out));
        if(n>=1025&&n<end)CHECK(out.leg[0].limited_nm==0);
        if(invalid&&n>=1025)CHECK(out.leg[0].rearm_required&&out.leg[0].limited_nm==0);
        if(n==end&&!invalid) {
            CHECK(out.leg[0].locked&&out.leg[0].faults==0&&e.event[0].events==events);
            CHECK(fabsf(out.leg[0].limited_nm)<=2.5001f);
            CHECK(prolonged?out.leg[0].lambda<.01f:out.leg[0].lambda>.99f);
            CHECK(fabsf(out.leg[0].limited_nm)>0);
        }
    }
    return 0;
}
static int asynchronous_speed_pause_case(bool prolonged,bool invalid) {
    exo_adaptive_config_t c=exo_adaptive_walking_preset_for_peak(15);
    exo_adaptive_engine_t e;exo_adaptive_output_t out;
    CHECK(exo_adaptive_engine_init(&e,&c));e.walking_mode=true;
    exo_adaptive_input_t held={0};
    for(unsigned n=0;n<1020;n++) {
        exo_adaptive_input_t in=sample(n);
        if(n>=1000) {
            if((n-1000)%3==0) {
                held=in;
                if(n<(prolonged?1015U:1010U))held.velocity_deg_s[0]=-450;
            }
            in.position_deg[0]=held.position_deg[0];
            in.velocity_deg_s[0]=held.velocity_deg_s[0];
            in.source_timestamp_us[0]=(uint64_t)held.timestamp_us;
            if(invalid&&n==1012)in.valid[0]=false;
        }
        CHECK(exo_adaptive_engine_update(&e,&in,&out));
        if(n>=1000&&n<=1011) {
            CHECK(out.leg[0].limited_nm==0);
            CHECK(out.leg[0].locked&&out.leg[0].lambda>.99f&&!out.leg[0].rearm_required);
        }
        if(n==1012&&!invalid) {
            if(prolonged)CHECK(out.leg[0].limited_nm==0&&out.leg[0].lambda==0&&!out.leg[0].locked);
            else CHECK(out.leg[0].locked&&out.leg[0].lambda>.99f&&fabsf(out.leg[0].limited_nm)<=2.5001f);
        }
        if(invalid&&n>=1012)CHECK(out.leg[0].rearm_required&&out.leg[0].limited_nm==0);
    }
    return 0;
}
int main(void) {
    CHECK(asynchronous_speed_pause_case(false,false)==0);
    CHECK(asynchronous_speed_pause_case(true,false)==0);
    CHECK(asynchronous_speed_pause_case(false,true)==0);
    CHECK(phase_pause_case(false,false)==0);
    CHECK(phase_pause_case(true,false)==0);
    CHECK(phase_pause_case(false,true)==0);
    exo_adaptive_config_t c=exo_adaptive_walking_preset_for_peak(15);
    /* A small rebound must not pair the previous peak with the new high trough. */
    exo_adaptive_event_t event={.initialized=true,.previous_angle_deg=25,
        .trend=-1,.low_deg=25,.peak_deg=25.5f,.last_peak_deg=25.5f,.last_low_deg=0,
        .previous_event_us=1000000,.last_event_us=1000000,.period_s=1,
        .intervals=3,.events=4};
    exo_adaptive_walking_event_update(&event,&c.afo,31,.01f,1400000);
    CHECK(event.locked&&event.last_low_deg==0&&event.candidate_low_deg==25);
    exo_adaptive_engine_t e;exo_adaptive_output_t out;
    CHECK(exo_adaptive_engine_init(&e,&c));e.walking_mode=true;
    unsigned positive[30]={0},negative[30]={0};
    for(unsigned n=0;n<3000;n++) {
        exo_adaptive_input_t in=sample(n);
        bool spike=n>=1000&&n%100>=20&&n%100<25;
        if(spike)in.velocity_deg_s[0]=-450;
        CHECK(exo_adaptive_engine_update(&e,&in,&out));
        if(n>=1000) {
            CHECK(!out.leg[0].rearm_required&&out.leg[0].locked);
            CHECK(out.leg[0].lambda>.99f);
            if(spike)CHECK(out.leg[0].limited_nm==0&&(out.leg[0].faults&EXO_ADAPTIVE_FAULT_RANGE));
            positive[n/100]+=out.leg[0].limited_nm>.2f;
            negative[n/100]+=out.leg[0].limited_nm<-.2f;
            if(n%100==25)CHECK(fabsf(out.leg[0].limited_nm)<=2.5001f);
        }
        CHECK(out.leg[0].limited_nm*in.velocity_deg_s[0]>=-1e-6f);
        CHECK(fabsf(out.leg[0].limited_nm)<=15.0001f);
    }
    for(unsigned k=10;k<30;k++)CHECK(positive[k]>0&&negative[k]>0);
    /* Sustained overspeed still drops capture and fade; never commands outside. */
    for(unsigned n=3000;n<3020;n++) {
        exo_adaptive_input_t in=sample(n);in.velocity_deg_s[0]=450;
        CHECK(exo_adaptive_engine_update(&e,&in,&out));CHECK(out.leg[0].limited_nm==0);
    }
    CHECK(out.leg[0].lambda==0&&!out.leg[0].locked);
    for(unsigned n=3020;n<4000;n++){exo_adaptive_input_t in=sample(n);CHECK(exo_adaptive_engine_update(&e,&in,&out));}
    CHECK(out.leg[0].lambda>.99f);
    /* Missing feedback DURING a short speed pause must remain a hard latch. */
    for(unsigned n=4000;n<4300;n++) {
        exo_adaptive_input_t in=sample(n);
        if(n==4000)in.velocity_deg_s[0]=450;
        if(n==4001){in.valid[0]=false;in.velocity_deg_s[0]=450;}
        CHECK(exo_adaptive_engine_update(&e,&in,&out));
        if(n>=4001)CHECK(out.leg[0].rearm_required&&out.leg[0].limited_nm==0);
    }
    puts("PASS continuity: atomic phase references; 20/20 bilateral-sign cycles despite speed spikes; zero outside; bounded restart slew; sustained overspeed resets; invalid feedback during pause stays latched");
    return 0;
}
