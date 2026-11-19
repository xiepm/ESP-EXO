#include "exo_adaptive.h"
#include <math.h>
#include <string.h>
#define AFO_FIELDS(X) X(minimum_hz) X(maximum_hz) X(initial_hz) X(phase_gain) X(frequency_gain) X(amplitude_gain) X(input_scale_deg) X(maximum_dt_s) X(velocity_filter_tau_s) X(event_velocity_deg_s) X(event_excursion_deg) X(correction_rate_rad_s)
#define LEG_FIELDS(X) X(enabled) X(strength) X(flex_amplitude_nm) X(flex_peak_phase) X(flex_width) X(ext_amplitude_nm) X(ext_peak_phase) X(ext_width) X(torque_limit_nm) X(slew_nm_s) X(positive_power_w) X(negative_power_w) X(minimum_angle_deg) X(maximum_angle_deg) X(maximum_velocity_deg_s) X(fade_s) X(transition_s)
static bool range(float x,float lo,float hi) { return isfinite(x)&&x>=lo&&x<=hi; }
exo_adaptive_config_t exo_adaptive_default_config(void) {
    exo_adaptive_config_t c={.schema=EXO_ADAPTIVE_SCHEMA,.overall_strength=0};
    c.afo=(exo_afo_config_t){.minimum_hz=0.3f,.maximum_hz=2.5f,.initial_hz=1.0f,
        .phase_gain=4,.frequency_gain=8,.amplitude_gain=3,.input_scale_deg=20,
        .maximum_dt_s=0.03f,.velocity_filter_tau_s=0.025f,
        .event_velocity_deg_s=3,.event_excursion_deg=5,.correction_rate_rad_s=0.8f};
    for(unsigned i=0;i<2;i++) c.leg[i]=(exo_adaptive_leg_config_t){
        .flex_width=0.1f,.ext_width=0.1f,.fade_s=1,.transition_s=1};
    /* 幅值/使能/功率/角速窗口全零：没有现场参数时不能产生助力。
     * AFO默认值仅为信号处理捕获设置，不是已经验证的人体助力处方。 */
    return c;
}
bool exo_adaptive_config_valid(const exo_adaptive_config_t *c) {
    if(!c||c->schema!=EXO_ADAPTIVE_SCHEMA||!range(c->overall_strength,0,1)) return false;
    const exo_afo_config_t *a=&c->afo;
    if(!range(a->minimum_hz,0.2f,2)||!range(a->maximum_hz,0.4f,3)||
        a->minimum_hz>=a->maximum_hz||!range(a->initial_hz,a->minimum_hz,a->maximum_hz)||
        !range(a->phase_gain,0.1f,20)||!range(a->frequency_gain,0.1f,30)||
        !range(a->amplitude_gain,0.1f,10)||!range(a->input_scale_deg,5,100)||
        !range(a->maximum_dt_s,0.01f,0.15f)||!range(a->velocity_filter_tau_s,0,0.1f)||
        !range(a->event_velocity_deg_s,1,30)||!range(a->event_excursion_deg,1,30)||
        !range(a->correction_rate_rad_s,0.05f,2)) return false;
    for(unsigned i=0;i<2;i++) {
        const exo_adaptive_leg_config_t *l=&c->leg[i];
#define FINITE(f) if(!isfinite(l->f)) return false;
        LEG_FIELDS(FINITE)
#undef FINITE
        if((l->enabled!=0&&l->enabled!=1)||!range(l->strength,0,1)||
           !range(l->flex_amplitude_nm,0,15)||!range(l->ext_amplitude_nm,0,15)||
           !range(l->flex_peak_phase,0,1)||!range(l->ext_peak_phase,0,1)||
           !range(l->flex_width,0.02f,0.3f)||!range(l->ext_width,0.02f,0.3f)||
           !range(l->torque_limit_nm,0,15)||!range(l->slew_nm_s,0,250)||
           !range(l->positive_power_w,0,100)||!range(l->negative_power_w,0,100)||
           !range(l->minimum_angle_deg,-90,90)||!range(l->maximum_angle_deg,-90,120)||
           !range(l->maximum_velocity_deg_s,0,500)||!range(l->fade_s,0.1f,10)||
           !range(l->transition_s,0.1f,10)) return false;
        if(l->enabled && (l->torque_limit_nm<=0||l->slew_nm_s<=0||
            l->positive_power_w<=0||l->negative_power_w<=0||
            l->minimum_angle_deg>=l->maximum_angle_deg||l->maximum_velocity_deg_s<=0)) return false;
    }
    return true;
}
exo_adaptive_config_t exo_adaptive_walking_preset(void) {
    return exo_adaptive_walking_preset_for_peak(.5f);
}
exo_adaptive_config_t exo_adaptive_walking_preset_for_peak(float peak_nm) {
    exo_adaptive_config_t c=exo_adaptive_default_config();c.overall_strength=1;
    /* A stance plateau is waveform shape, not an abrupt cadence change.
     * Keep shape adaptation, reduce phase/frequency chasing of that residual. */
    c.afo.phase_gain=1;c.afo.frequency_gain=2;
    /* User-requested walking envelope (2026-11-18): -40..90 deg,
     * |velocity|<=400 deg/s. Historical bench/trial presets are independent.
     * These are command guards, not a validated range for every wearer. */
    for(unsigned i=0;i<2;i++)c.leg[i]=(exo_adaptive_leg_config_t){
        .enabled=1,.strength=1,.flex_amplitude_nm=peak_nm,.flex_peak_phase=.75f,.flex_width=.1f,
        .ext_amplitude_nm=peak_nm,.ext_peak_phase=.25f,.ext_width=.1f,
        .torque_limit_nm=fmaxf(.5f,peak_nm),
        /* Gaussian sigma=.1 at <=2.5 Hz needs <228 Nm/s at 15 Nm.
         * Retain the existing 80 W ceiling when expanding the velocity window:
         * at 400 deg/s it limits torque to ~11.46 Nm, not 15 Nm.
         * Scale with the user's peak; zero amplitude cannot generate torque. */
        .slew_nm_s=fmaxf(1,peak_nm/15.0f*250.0f),
        .positive_power_w=fmaxf(1,peak_nm/15.0f*80.0f),
        .negative_power_w=fmaxf(1,peak_nm/15.0f*80.0f),
        .minimum_angle_deg=-40,.maximum_angle_deg=90,.maximum_velocity_deg_s=400,
        .fade_s=3,.transition_s=3};
    return c;
}
exo_adaptive_config_t exo_adaptive_trial_preset(void) {
    exo_adaptive_config_t c=exo_adaptive_default_config();c.overall_strength=1;
    for(unsigned i=0;i<2;i++)c.leg[i]=(exo_adaptive_leg_config_t){
        .enabled=1,.strength=1,.flex_amplitude_nm=5,.flex_peak_phase=.75f,.flex_width=.1f,
        .ext_amplitude_nm=5,.ext_peak_phase=.25f,.ext_width=.1f,.torque_limit_nm=5,
        .slew_nm_s=1,.positive_power_w=2,.negative_power_w=1,
        .minimum_angle_deg=-20,.maximum_angle_deg=20,.maximum_velocity_deg_s=60,
        .fade_s=3,.transition_s=3};
    return c;
}
bool exo_adaptive_trial_preset_matches(const exo_adaptive_config_t *c) {
    uint8_t actual[EXO_ADAPTIVE_CONFIG_BYTES],expected[EXO_ADAPTIVE_CONFIG_BYTES];
    exo_adaptive_config_t preset=exo_adaptive_trial_preset();
    return exo_adaptive_config_encode(c,actual,sizeof(actual))&&
        exo_adaptive_config_encode(&preset,expected,sizeof(expected))&&
        memcmp(actual,expected,sizeof(actual))==0;
}
static uint32_t crc32(const uint8_t *b,unsigned n) {
    uint32_t v=0xffffffffU;
    for(unsigned i=0;i<n;i++) {
        v^=b[i];
        for(unsigned j=0;j<8;j++) v=(v>>1)^((v&1U)?0xedb88320U:0U);
    }
    return ~v;
}
static void put(uint8_t *b,uint32_t v) {
    for(unsigned j=0;j<4;j++) b[j]=(uint8_t)(v>>(8*j));
}
static uint32_t get(const uint8_t *b) {
    return (uint32_t)b[0]|((uint32_t)b[1]<<8)|((uint32_t)b[2]<<16)|((uint32_t)b[3]<<24);
}
bool exo_adaptive_config_encode(const exo_adaptive_config_t *c,uint8_t *b,uint32_t size) {
    if(!b||size!=EXO_ADAPTIVE_CONFIG_BYTES||!exo_adaptive_config_valid(c)) return false;
    unsigned index=4;put(b,c->schema);
#define WRITE_FLOAT(v) do { uint32_t bits;float value=(v);memcpy(&bits,&value,4);put(b+index,bits);index+=4; } while(0)
    WRITE_FLOAT(c->overall_strength);
#define WRITE_AFO(f) WRITE_FLOAT(c->afo.f);
    AFO_FIELDS(WRITE_AFO)
#undef WRITE_AFO
    for(unsigned i=0;i<2;i++) {
#define WRITE_LEG(f) WRITE_FLOAT(c->leg[i].f);
        LEG_FIELDS(WRITE_LEG)
#undef WRITE_LEG
    }
#undef WRITE_FLOAT
    if(index!=size-4) return false;
    put(b+index,crc32(b,index));return true;
}
bool exo_adaptive_config_decode(exo_adaptive_config_t *c,const uint8_t *b,uint32_t size) {
    if(!c||!b||size!=EXO_ADAPTIVE_CONFIG_BYTES||get(b+size-4)!=crc32(b,size-4)) return false;
    exo_adaptive_config_t candidate={.schema=get(b)};unsigned index=4;
#define READ_FLOAT(v) do { uint32_t bits=get(b+index);memcpy(&(v),&bits,4);index+=4; } while(0)
    READ_FLOAT(candidate.overall_strength);
#define READ_AFO(f) READ_FLOAT(candidate.afo.f);
    AFO_FIELDS(READ_AFO)
#undef READ_AFO
    for(unsigned i=0;i<2;i++) {
#define READ_LEG(f) READ_FLOAT(candidate.leg[i].f);
        LEG_FIELDS(READ_LEG)
#undef READ_LEG
    }
#undef READ_FLOAT
    if(index!=size-4||!exo_adaptive_config_valid(&candidate)) return false;
    *c=candidate;return true;
}
