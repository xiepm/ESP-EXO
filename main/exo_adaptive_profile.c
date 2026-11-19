#include "exo_adaptive.h"
#include <math.h>
float exo_adaptive_gaussian(float s,float mu,float sigma) {
    if(!isfinite(s)||!isfinite(mu)||!isfinite(sigma)||sigma<0.02f||sigma>0.30f) return NAN;
    float d=2*EXO_ADAPTIVE_PI*(s-mu),w=2*EXO_ADAPTIVE_PI*sigma;
    return expf((cosf(d)-1)/(w*w));
}
bool exo_adaptive_profile(const exo_adaptive_leg_config_t *c,float phase,
    float strength,float lambda,float *flex,float *ext,float *desired) {
    if(!c||!flex||!ext||!desired) return false;
    *flex=0;*ext=0;*desired=0;
    if(!isfinite(phase)||!isfinite(strength)||!isfinite(lambda)||
        strength<0||strength>1||lambda<0||lambda>1) return false;
    if(c->enabled==0) return true;
    *flex=lambda*strength*c->strength*c->flex_amplitude_nm*
        exo_adaptive_gaussian(phase,c->flex_peak_phase,c->flex_width);
    *ext=-lambda*strength*c->strength*c->ext_amplitude_nm*
        exo_adaptive_gaussian(phase,c->ext_peak_phase,c->ext_width);
    *desired=*flex+*ext;
    return isfinite(*desired);
}
