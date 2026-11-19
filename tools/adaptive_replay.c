#include "exo_adaptive.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
static unsigned checks;
#define CHECK(x) do { if(!(x)) { fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x);exit(1); } checks++; } while(0)
static exo_adaptive_config_t demo(void) {
    exo_adaptive_config_t c=exo_adaptive_default_config();c.overall_strength=1;
    for(unsigned i=0;i<2;i++) c.leg[i]=(exo_adaptive_leg_config_t){
        .enabled=1,.strength=1,.flex_amplitude_nm=0.2f,.flex_peak_phase=0.75f,.flex_width=0.1f,
        .ext_amplitude_nm=0.15f,.ext_peak_phase=0.15f,.ext_width=0.12f,.torque_limit_nm=0.25f,
        .slew_nm_s=2,.positive_power_w=1,.negative_power_w=0.8f,
        .minimum_angle_deg=-40,.maximum_angle_deg=60,.maximum_velocity_deg_s=350,
        .fade_s=1,.transition_s=0.5f};
    return c;
}
static exo_adaptive_input_t sample(unsigned n,float left_hz,float right_hz) {
    float t=n*0.01f;exo_adaptive_input_t in={.timestamp_us=10000+(int64_t)n*10000,
        .sequence=n+1,.coordinate_epoch=1,.valid={true,true},.permit_epoch=1};
    float f[2]={left_hz,right_hz};
    for(unsigned i=0;i<2;i++) {
        float p=2*EXO_ADAPTIVE_PI*f[i]*t+(i?0.7f:0.0f);
        in.position_deg[i]=7+18*cosf(p)+2*cosf(2*p+0.2f);
        in.velocity_deg_s[i]=-18*2*EXO_ADAPTIVE_PI*f[i]*sinf(p)-4*2*EXO_ADAPTIVE_PI*f[i]*sinf(2*p+0.2f);
    }return in;
}
static void header(FILE *f) {
    fprintf(f,"timestamp_us,side,position_deg,velocity_deg_s,feedback_torque_nm,feedback_valid,raw_phase,corrected_phase,frequency_hz,period_s,confidence,event,locked,flex_nm,ext_nm,desired_nm,limited_nm,faults,rearm,config_generation\n");
}
static void row(FILE *f,const exo_adaptive_input_t *in,const exo_adaptive_output_t *out) {
    if(!f) return;
    for(unsigned i=0;i<2;i++) {
        const exo_adaptive_leg_output_t *o=&out->leg[i];
        fprintf(f,"%lld,%u,%.6f,%.6f,%.6f,%u,%.6f,%.6f,%.6f,%.6f,%.6f,%u,%u,%.6f,%.6f,%.6f,%.6f,%u,%u,%u\n",
            (long long)in->timestamp_us,i,in->position_deg[i],in->velocity_deg_s[i],in->feedback_torque_nm[i],
            in->feedback_torque_valid[i],o->raw_phase,o->corrected_phase,o->frequency_hz,o->period_s,
            o->confidence,o->event,o->locked,o->flex_nm,o->ext_nm,o->desired_nm,o->limited_nm,
            o->faults,o->rearm_required,out->config_generation);
    }
}
static void tests(const char *output) {
    exo_adaptive_config_t c=demo();CHECK(exo_adaptive_config_valid(&c));
    uint8_t blob[EXO_ADAPTIVE_CONFIG_BYTES];exo_adaptive_config_t decoded={0};
    CHECK(exo_adaptive_config_encode(&c,blob,sizeof(blob)));
    CHECK(exo_adaptive_config_decode(&decoded,blob,sizeof(blob)));
    CHECK(memcmp(&c,&decoded,sizeof(c))==0);
    blob[20]^=1;CHECK(!exo_adaptive_config_decode(&decoded,blob,sizeof(blob)));
    c.leg[0].flex_width=NAN;CHECK(!exo_adaptive_config_valid(&c));c=demo();
    c.afo.maximum_dt_s=1;CHECK(!exo_adaptive_config_valid(&c));c=demo();
    float maximum_wrap_error=0;
    for(unsigned n=0;n<100;n++) {
        float mu=n/100.0f,eps=1e-5f;
        float l=exo_adaptive_gaussian(-eps,mu,.1f),r=exo_adaptive_gaussian(1-eps,mu,.1f);
        maximum_wrap_error=fmaxf(maximum_wrap_error,fabsf(l-r));CHECK(fabsf(l-r)<2e-6f);
    }
    printf("METRIC periodic_boundary_error %.9f\n",maximum_wrap_error);
    uint32_t flags=0;
    CHECK(fabsf(exo_adaptive_limit(&c.leg[0],10,0,0,180,.01f,&flags)-.02f)<1e-6f);
    CHECK(flags==0);
    float cap=exo_adaptive_limit(&c.leg[0],10,10,0,350,.01f,&flags);
    CHECK(cap*350*EXO_ADAPTIVE_PI/180<=1.00001f);
    flags=EXO_ADAPTIVE_FAULT_STOP;CHECK(exo_adaptive_limit(&c.leg[0],1,1,0,0,.01f,&flags)==0);
    flags=0;CHECK(exo_adaptive_limit(&c.leg[0],1,1,100,0,.01f,&flags)==0);
    CHECK(flags&EXO_ADAPTIVE_FAULT_RANGE);
    exo_adaptive_engine_t e;exo_adaptive_output_t o;
    CHECK(exo_adaptive_engine_init(&e,&c));
    FILE *f=output?fopen(output,"w"):NULL;if(output) CHECK(f!=NULL);if(f) header(f);
    unsigned locked[2]={0},nonzero[2]={0};float errors[2]={0};
    clock_t begin=clock();
    for(unsigned n=0;n<6000;n++) {
        exo_adaptive_input_t in=sample(n,.85f,1.2f);
        CHECK(exo_adaptive_engine_update(&e,&in,&o));row(f,&in,&o);
        for(unsigned i=0;i<2;i++) {
            if(n>3000) {locked[i]+=o.leg[i].locked;errors[i]+=fabsf(o.leg[i].frequency_hz-(i?1.2f:.85f));}
            if(fabsf(o.leg[i].limited_nm)>1e-4f) nonzero[i]++;
            CHECK(isfinite(o.leg[i].limited_nm)&&fabsf(o.leg[i].limited_nm)<=.25001f);
        }
    }
    if(f) fclose(f);
    printf("METRIC asymmetric locked %u %u frequency_mae %.6f %.6f nonzero %u %u host_cpu_us_per_tick %.3f\n",
        locked[0],locked[1],errors[0]/2999,errors[1]/2999,nonzero[0],nonzero[1],
        1e6*(double)(clock()-begin)/CLOCKS_PER_SEC/6000);
    CHECK(locked[0]>2500&&locked[1]>2500);
    CHECK(errors[0]/2999<.08f&&errors[1]/2999<.08f);
    CHECK(nonzero[0]>100&&nonzero[1]>100);
    exo_adaptive_input_t in=sample(6000,.85f,1.2f);in.valid[0]=false;
    CHECK(exo_adaptive_engine_update(&e,&in,&o));CHECK(o.leg[0].limited_nm==0);
    CHECK(o.leg[0].rearm_required);
    for(unsigned n=6001;n<8000;n++) { in=sample(n,.85f,1.2f);CHECK(exo_adaptive_engine_update(&e,&in,&o));CHECK(o.leg[0].limited_nm==0); }
    CHECK(o.leg[0].locked&&o.leg[0].rearm_required);
    in=sample(8000,.85f,1.2f);in.permit_epoch=2;CHECK(exo_adaptive_engine_update(&e,&in,&o));
    CHECK(!o.leg[0].rearm_required&&o.leg[0].limited_nm==0);
    in.timestamp_us--;CHECK(exo_adaptive_engine_update(&e,&in,&o));CHECK(o.leg[0].faults&EXO_ADAPTIVE_FAULT_TIME);
    CHECK(o.leg[0].limited_nm==0);
    printf("PASS adaptive initial suite assertions=%u\n",checks);
}

static void extended_tests(void) {
    exo_adaptive_config_t c=demo();exo_adaptive_engine_t e;exo_adaptive_output_t out;
    CHECK(exo_adaptive_engine_init(&e,&c));
    float phase[2]={0,1.2f},freq_error=0,phase_error=0;
    unsigned counted=0,locked=0;uint32_t noise=12345U;
    for(unsigned n=0;n<14000;n++) {
        float t=n*.01f, hz=t<20?.65f:(t<70?.65f+.015f*(t-20):(t<120?1.4f-.015f*(t-70):.65f));
        exo_adaptive_input_t in={.timestamp_us=10000+(int64_t)n*10000,.sequence=n+1,
            .coordinate_epoch=1,.valid={true,true},.permit_epoch=1};
        for(unsigned i=0;i<2;i++) {
            float f=hz*(i?1.12f:1),amplitude=16+6*sinf(t*.15f+(float)i);
            noise=noise*1664525U+1013904223U;
            float jitter=((noise>>8)%1000)/1000.0f-.5f;
            in.position_deg[i]=4+amplitude*cosf(phase[i])+.2f*jitter;
            in.velocity_deg_s[i]=-amplitude*2*EXO_ADAPTIVE_PI*f*sinf(phase[i]);
            phase[i]=exo_adaptive_wrap(phase[i]+2*EXO_ADAPTIVE_PI*f*.01f);
        }
        CHECK(exo_adaptive_engine_update(&e,&in,&out));
        if(n>2000) {
            freq_error+=fabsf(out.leg[0].frequency_hz-hz);counted++;locked+=out.leg[0].locked;
            phase_error+=fabsf(exo_adaptive_circular_error(out.leg[0].corrected_phase*2*EXO_ADAPTIVE_PI-phase[0]))/(2*EXO_ADAPTIVE_PI);
        }
        CHECK(isfinite(out.leg[0].limited_nm)&&fabsf(out.leg[0].limited_nm)<=.25001f);
    }
    printf("METRIC speed_ramp_noise frequency_mae=%.6f phase_mae_cycles=%.6f locked=%u/%u\n",
        freq_error/counted,phase_error/counted,locked,counted);
    CHECK(freq_error/counted<.12f);CHECK(phase_error/counted<.10f);CHECK(locked>counted*.85f);
    /* 重新建立稳定步态，观察参数突变、普通禁用和紧急切断。 */
    CHECK(exo_adaptive_engine_init(&e,&c));
    exo_adaptive_input_t in={0};
    for(unsigned n=0;n<3000;n++) {in=sample(n,1,1);CHECK(exo_adaptive_engine_update(&e,&in,&out));}
    float prev=out.leg[0].limited_nm,max_delta=0;
    c.leg[0].flex_peak_phase=.1f;c.leg[0].flex_amplitude_nm=.25f;
    CHECK(exo_adaptive_engine_configure(&e,&c,2));
    for(unsigned n=3000;n<3300;n++) {
        in=sample(n,1,1);CHECK(exo_adaptive_engine_update(&e,&in,&out));
        max_delta=fmaxf(max_delta,fabsf(out.leg[0].limited_nm-prev));
        CHECK(out.config_generation==2&&fabsf(out.leg[0].limited_nm-prev)<=.02001f);
        prev=out.leg[0].limited_nm;
    }
    CHECK(max_delta<=.02001f);printf("METRIC parameter_transition maximum_delta_nm=%.6f\n",max_delta);
    c.leg[0].enabled=0;CHECK(exo_adaptive_engine_configure(&e,&c,3));
    for(unsigned n=3300;n<3500;n++) {in=sample(n,1,1);CHECK(exo_adaptive_engine_update(&e,&in,&out));}
    CHECK(out.leg[0].limited_nm==0);
    in=sample(3500,1,1);in.emergency=true;CHECK(exo_adaptive_engine_update(&e,&in,&out));
    CHECK(out.leg[1].limited_nm==0&&(out.leg[1].faults&EXO_ADAPTIVE_FAULT_STOP));
    /* 停步：超过300ms后的首个控制周期失锁；重捕获不能自动恢复授权。 */
    c=demo();CHECK(exo_adaptive_engine_init(&e,&c));
    for(unsigned n=0;n<3000;n++) {in=sample(n,1,1);CHECK(exo_adaptive_engine_update(&e,&in,&out));}
    float still[2]={in.position_deg[0],in.position_deg[1]};
    for(unsigned n=3000;n<3100;n++) {
        in=sample(n,1,1);for(unsigned i=0;i<2;i++){in.position_deg[i]=still[i];in.velocity_deg_s[i]=0;}
        CHECK(exo_adaptive_engine_update(&e,&in,&out));
    }
    CHECK(!out.leg[0].locked&&out.leg[0].limited_nm==0&&out.leg[0].rearm_required);
    for(unsigned n=3100;n<6000;n++){in=sample(n,1,1);CHECK(exo_adaptive_engine_update(&e,&in,&out));CHECK(out.leg[0].limited_nm==0);}
    printf("METRIC restart locked=%u hz=%.4f period=%.4f events=%u rejected=%u residual=%.4f offset=%.4f target=%.4f intervals=%u\n",out.leg[0].locked,out.leg[0].frequency_hz,out.leg[0].period_s,out.leg[0].accepted_events,out.leg[0].rejected_events,e.afo[0].residual_ewma,e.event[0].offset_rad,e.event[0].target_offset_rad,e.event[0].intervals);
    CHECK(out.leg[0].locked);
    for(unsigned n=6000;n<6200;n++){in=sample(n,1,1);in.permit_epoch=2;CHECK(exo_adaptive_engine_update(&e,&in,&out));}
    CHECK(!out.leg[0].rearm_required);
    /* 重复源反馈不重复积分/记事件，只有超时/回退才失效。 */
    in=sample(6200,1,1);in.permit_epoch=2;in.source_timestamp_us[0]=(uint64_t)in.timestamp_us;
    CHECK(exo_adaptive_engine_update(&e,&in,&out));float raw=out.leg[0].raw_phase;uint32_t events=out.leg[0].accepted_events;
    in.timestamp_us+=10000;in.sequence++;CHECK(exo_adaptive_engine_update(&e,&in,&out));
    CHECK(out.leg[0].raw_phase==raw&&!out.leg[0].event&&out.leg[0].accepted_events==events);
    /* 坐标复位、NaN、源时钟回退、编码器跳变。 */
    in.coordinate_epoch++;in.timestamp_us+=10000;in.sequence++;
    CHECK(exo_adaptive_engine_update(&e,&in,&out));CHECK(out.leg[0].limited_nm==0&&out.leg[0].rearm_required);
    in.position_deg[0]=NAN;in.timestamp_us+=10000;in.sequence++;
    CHECK(exo_adaptive_engine_update(&e,&in,&out));CHECK(out.leg[0].faults&EXO_ADAPTIVE_FAULT_INPUT);
    /* 有效事件太短不更新周期/相位目标；太长必须重新捕获。 */
    exo_adaptive_event_t event={.initialized=true,.trend=1,.filtered_velocity=-100,
        .peak_deg=20,.low_deg=0,.peak_us=1100000,.previous_event_us=1000000,
        .last_event_us=1000000,.period_s=1,.intervals=3,.locked=true};
    exo_adaptive_event_update(&event,&c.afo,15,-100,0,2*EXO_ADAPTIVE_PI,.01f,1120000);
    CHECK(!event.event&&event.rejected==1&&event.previous_event_us==1000000);
    event.trend=1;event.peak_deg=20;event.low_deg=0;event.peak_us=5000000;
    exo_adaptive_event_update(&event,&c.afo,15,-100,0,2*EXO_ADAPTIVE_PI,.01f,5020000);
    CHECK(!event.locked&&event.intervals==0&&event.rejected==2);
    /* 圆周跨界校正每步受限，没有瞬时相位清零。 */
    event=(exo_adaptive_event_t){.initialized=true,.offset_rad=3.13f,.target_offset_rad=-3.13f};
    exo_adaptive_event_update(&event,&c.afo,0,0,0,2*EXO_ADAPTIVE_PI,.01f,100000);
    CHECK(fabsf(exo_adaptive_circular_error(event.offset_rad-3.13f))<=c.afo.correction_rate_rad_s*.01f+1e-6f);
    float flex,ext,total;exo_adaptive_leg_config_t leg=c.leg[0];
    leg.flex_amplitude_nm=leg.ext_amplitude_nm=.2f;leg.flex_peak_phase=leg.ext_peak_phase=.5f;leg.flex_width=leg.ext_width=.1f;
    CHECK(exo_adaptive_profile(&leg,.5f,1,1,&flex,&ext,&total));CHECK(fabsf(total)<1e-7f);
    leg.flex_amplitude_nm=leg.ext_amplitude_nm=0;CHECK(exo_adaptive_profile(&leg,.1f,1,1,&flex,&ext,&total)&&total==0);
    CHECK(isnan(exo_adaptive_gaussian(.5f,.2f,0)));
    uint32_t flags=0;float negative=exo_adaptive_limit(&c.leg[0],-1,-1,0,180,.01f,&flags);
    CHECK(-negative*EXO_ADAPTIVE_PI<=c.leg[0].negative_power_w+1e-6f);
    in.timestamp_us=INT64_MIN;in.sequence++;
    CHECK(exo_adaptive_engine_update(&e,&in,&out));
    CHECK(out.leg[0].limited_nm==0&&(out.leg[0].faults&EXO_ADAPTIVE_FAULT_TIME));
    /* 控制循环仍10ms，但重复反馈导致新源间隔40ms；AFO拒绝后必须重新授权。 */
    CHECK(exo_adaptive_engine_init(&e,&c));
    for(unsigned n=0;n<3000;n++){in=sample(n,1,1);CHECK(exo_adaptive_engine_update(&e,&in,&out));}
    uint64_t source_time=(uint64_t)in.timestamp_us;
    for(unsigned n=0;n<4;n++) {
        in.timestamp_us+=10000;in.sequence++;
        in.source_timestamp_us[0]=n==3?(uint64_t)in.timestamp_us:source_time;
        CHECK(exo_adaptive_engine_update(&e,&in,&out));
    }
    CHECK((out.leg[0].faults&EXO_ADAPTIVE_FAULT_INPUT)&&out.leg[0].rearm_required);
    CHECK(!out.leg[0].locked&&out.leg[0].limited_nm==0);
    printf("PASS extended: acceleration/deceleration noise amplitude stop/restart events wrap parameter fade emergency NaN epoch duplicate-source power\n");
}
static int replay(const char *input,const char *output) {
    FILE *src=fopen(input,"r"),*dst=fopen(output,"w");if(!src||!dst) return 2;
    exo_adaptive_config_t c=demo();
    /* 原始历史遥测为10/20Hz，按真实时间回放，内部仍使用<=2ms子步；不伪造100Hz证据。 */
    c.afo.maximum_dt_s=.15f;
    exo_adaptive_engine_t e;exo_adaptive_output_t out;exo_adaptive_engine_init(&e,&c);
    char line[1024];unsigned rows=0,locked=0;header(dst);
    while(fgets(line,sizeof(line),src)) {
        long long us;float ql,qr,vl,vr;unsigned valid_l,valid_r;
        if(sscanf(line,"%lld,%f,%f,%f,%f,%u,%u",&us,&ql,&qr,&vl,&vr,&valid_l,&valid_r)!=7) continue;
        exo_adaptive_input_t in={.timestamp_us=us,.sequence=++rows,.coordinate_epoch=1,
            .position_deg={ql,qr},.velocity_deg_s={vl,vr},.valid={valid_l!=0,valid_r!=0},
            .permit_epoch=1};
        if(!exo_adaptive_engine_update(&e,&in,&out)) return 3;
        locked+=out.leg[0].locked||out.leg[1].locked;row(dst,&in,&out);
    }
    fclose(src);fclose(dst);printf("PASS real_log_replay rows=%u locked_rows=%u (not human efficacy)\n",rows,locked);
    return rows?0:4;
}
int main(int argc,char **argv) {
    if(argc==3&&strcmp(argv[1],"--validate-config")==0) {
        FILE *f=fopen(argv[2],"rb");uint8_t blob[EXO_ADAPTIVE_CONFIG_BYTES+1];
        if(!f)return 2;
        size_t n=fread(blob,1,sizeof(blob),f);fclose(f);exo_adaptive_config_t c;
        if(!exo_adaptive_config_decode(&c,blob,(uint32_t)n))return 3;
        printf("PASS firmware config decoder: schema=%u enabled L/R=%.0f/%.0f strength=%.3f; no hardware action\n",c.schema,c.leg[0].enabled,c.leg[1].enabled,c.overall_strength);return 0;
    }
    if(argc==4&&strcmp(argv[1],"--replay")==0) return replay(argv[2],argv[3]);
    tests(argc>1?argv[1]:NULL);extended_tests();printf("PASS total assertions=%u\n",checks);return 0;
}
