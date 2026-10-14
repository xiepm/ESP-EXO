#ifndef EXO_FEEDBACK_RECOVERY_H
#define EXO_FEEDBACK_RECOVERY_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "exo_types.h"

/* 待机反馈恢复不是运动控制。由现有慢速执行器调用，不创建任务/定时器。
 * 每次失联最多3次请求，观察到连续1秒真实反馈才重新武装下一次失联预算。
 * 自动预算耗尽后由网页显式重试；BUS_OFF/错误被动不自动复位。
 * 已有待发帧时等待；双侧都无新反馈时还需RX离散高采样持续1秒。
 * 另一台真实在线时不把正常CAN低位误当作断电，单侧按原有限流程恢复。 */
typedef struct {
    uint8_t attempts[EXO_ACTUATOR_COUNT];
    bool exhausted[EXO_ACTUATOR_COUNT];
} exo_feedback_recovery_snapshot_t;

void exo_feedback_recovery_poll(void);
esp_err_t exo_feedback_recovery_request(void);
void exo_feedback_recovery_get_snapshot(exo_feedback_recovery_snapshot_t *out);

#endif
