/* 历史策略回归仅隔离新增观察器；新策略另用真实runtime/NVS fake做集成测试。 */
#include "exo_adaptive_runtime.h"
#include <string.h>
bool exo_adaptive_runtime_preview(uint32_t sequence,int64_t time,exo_joint_command_pair_t *p) {
    (void)sequence;(void)time;if(p)memset(p,0,sizeof(*p));return false;
}
exo_assist_unavailable_reason_t exo_adaptive_runtime_unavailable_reason(
    uint32_t sequence,int64_t time,bool left,bool right) {
    (void)sequence;(void)time;(void)left;(void)right;
    return EXO_ASSIST_UNAVAILABLE_NONE;
}
