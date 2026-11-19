#include "exo_adaptive.h"
#include <math.h>
#include <string.h>
static float bound(float x,float lo,float hi) { return fminf(hi,fmaxf(lo,x)); }
float exo_adaptive_wrap(float x) {
    x=fmodf(x,2.0f*EXO_ADAPTIVE_PI);
    return x<0.0f ? x+2.0f*EXO_ADAPTIVE_PI : x;
}
float exo_adaptive_circular_error(float x) {
    return exo_adaptive_wrap(x+EXO_ADAPTIVE_PI)-EXO_ADAPTIVE_PI;
}
void exo_afo_reset(exo_afo_t *s,const exo_afo_config_t *c,float q) {
    if(!s||!c)return;
    memset(s,0,sizeof(*s));
    s->omega_rad_s=2.0f*EXO_ADAPTIVE_PI*c->initial_hz;
    s->cosine[0]=q/c->input_scale_deg;
    s->cosine[1]=0.1f;
    s->initialized=true;
}
bool exo_afo_update(exo_afo_t *s,const exo_afo_config_t *c,float q,float dt) {
    if(!s||!c||!isfinite(q)||!isfinite(dt)||dt<=0.0f||dt>c->maximum_dt_s) return false;
    if(!isfinite(c->input_scale_deg)||c->input_scale_deg<=0||
       !isfinite(c->minimum_hz)||!isfinite(c->maximum_hz)||
       c->minimum_hz<=0||c->maximum_hz<=c->minimum_hz||
       !isfinite(c->phase_gain)||!isfinite(c->frequency_gain)||!isfinite(c->amplitude_gain))return false;
    if(!s->initialized) exo_afo_reset(s,c,q);
    if(!isfinite(s->phase_rad)||!isfinite(s->omega_rad_s)||!isfinite(s->residual_ewma)) return false;
    for(unsigned j=0;j<=EXO_AFO_HARMONICS;j++)
        if(!isfinite(s->cosine[j])||!isfinite(s->sine[j])) return false;
    /* Petric形式：B1故意不存在，避免基频正弦项把相位误差全吸收到幅值。
     * 文献方程(2)-(7)，frequency_gain单独调节；输入按deg尺度归一化。
     * 旧状态同时计算导数，再Euler更新；最大2ms子步抑制10ms离散误差。 */
    unsigned n=(unsigned)ceilf(dt/0.002f);
    if(n==0U||n>75U) return false;
    float h=dt/(float)n;
    for(unsigned k=0;k<n;k++) {
        float cs[4]={1,0,0,0},sn[4]={0},estimate=s->cosine[0];
        for(unsigned j=1;j<=EXO_AFO_HARMONICS;j++) {
            cs[j]=cosf(j*s->phase_rad); sn[j]=sinf(j*s->phase_rad);
            estimate+=s->cosine[j]*cs[j]+(j>=2U?s->sine[j]*sn[j]:0.0f);
        }
        float e=bound(q/c->input_scale_deg-estimate,-4.0f,4.0f);
        float phase_dot=s->omega_rad_s-c->phase_gain*e*sn[1];
        float frequency_dot=-c->frequency_gain*e*sn[1];
        s->phase_rad=exo_adaptive_wrap(s->phase_rad+h*phase_dot);
        s->omega_rad_s=bound(s->omega_rad_s+h*frequency_dot,
            2*EXO_ADAPTIVE_PI*c->minimum_hz,2*EXO_ADAPTIVE_PI*c->maximum_hz);
        for(unsigned j=0;j<=EXO_AFO_HARMONICS;j++) {
            s->cosine[j]=bound(s->cosine[j]+h*c->amplitude_gain*e*cs[j],j==1?0.001f:-8.0f,8.0f);
            if(j>=2U) s->sine[j]=bound(s->sine[j]+h*c->amplitude_gain*e*sn[j],-8.0f,8.0f);
        }
        s->residual_ewma+=h*2.0f*(fabsf(e)-s->residual_ewma);
    }
    return isfinite(s->phase_rad)&&isfinite(s->omega_rad_s);
}
