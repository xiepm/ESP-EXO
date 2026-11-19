#include "exo_adaptive.h"
#include <math.h>
float exo_adaptive_limit(const exo_adaptive_leg_config_t *c,float desired,float previous,
    float q,float velocity,float dt,uint32_t *faults) {
    if(!c||!faults) return 0;
    if(!isfinite(c->torque_limit_nm)||c->torque_limit_nm<0||c->torque_limit_nm>15||
       !isfinite(c->slew_nm_s)||c->slew_nm_s<0||c->slew_nm_s>250||
       !isfinite(c->positive_power_w)||c->positive_power_w<0||
       !isfinite(c->negative_power_w)||c->negative_power_w<0||
       !isfinite(c->minimum_angle_deg)||!isfinite(c->maximum_angle_deg)||
       c->minimum_angle_deg>=c->maximum_angle_deg||
       !isfinite(c->maximum_velocity_deg_s)||c->maximum_velocity_deg_s<=0) {
        *faults|=EXO_ADAPTIVE_FAULT_CONFIG;return 0;
    }
    if(!isfinite(desired)||!isfinite(previous)||!isfinite(q)||!isfinite(velocity)||
       !isfinite(dt)||dt<=0) { *faults|=EXO_ADAPTIVE_FAULT_INPUT;return 0; }
    if(q<c->minimum_angle_deg||q>c->maximum_angle_deg||
       fabsf(velocity)>c->maximum_velocity_deg_s) {
        *faults|=EXO_ADAPTIVE_FAULT_RANGE;return 0;
    }
    if(*faults) return 0; /* 故障优先于普通渐变，不能拖延保护。 */
    float limit=c->torque_limit_nm,step=c->slew_nm_s*dt;
    float target=fmaxf(-limit,fminf(limit,desired));
    float result=previous+fmaxf(-step,fminf(step,target-previous));
    /* P=τ·ω，度/秒只在这里显式转rad/s。吸收功率与输出功率分别限制。
     * 安全窗口收紧/功率超限优先于变化率；切除不得被旧力矩惯性延迟。 */
    float omega=velocity*EXO_ADAPTIVE_PI/180.0f;
    if(fabsf(omega)>0.0001f) {
        float pmax=result*omega>=0?c->positive_power_w:c->negative_power_w;
        limit=fminf(limit,pmax/fabsf(omega));
    }
    return fmaxf(-limit,fminf(limit,result));
}
