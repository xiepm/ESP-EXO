#include "exo_feedback_recovery.h"

#include <string.h>
#include "driver/twai.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "exo_config.h"
#include "exo_system.h"
#include "freertos/FreeRTOS.h"
#include "rs01_driver.h"

#define RECOVERY_ATTEMPTS 3U
#define RECOVERY_GRACE_US 1000000ULL
#define RECOVERY_MANUAL_COOLDOWN_US 10000000ULL

typedef struct {
    uint8_t attempts;
    uint64_t offline_since_us;
    uint64_t healthy_since_us;
    uint64_t next_attempt_us;
} recovery_node_t;

static recovery_node_t s_nodes[EXO_ACTUATOR_COUNT];
static uint64_t s_last_manual_us;
static bool s_manual_requested;
/* 仅由既有执行器任务读写；BLE只重置节点预算，不操作这个物理等待状态。 */
static uint64_t s_bus_ready_since_us;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

static bool can_usable(void) {
    twai_status_info_t can;
    return twai_get_status_info(&can) == ESP_OK &&
        can.state == TWAI_STATE_RUNNING &&
        can.tx_error_counter < 128U && can.rx_error_counter < 128U;
}

/* 0.10.5真机后上电失败：RX持续低时TWAI仍可显示RUNNING/TEC=0，
 * 入队帧会等到供电恢复后集中发送，且预算已先被消耗。
 * 因此先等待“健康计数、没有待发帧、RX采样为高”持续1秒再尝试。
 * 这里是200 ms离散观察，不是示波器的总线空闲证明；RX高也不能证明
 * 模块电压、终端电阻或电机ACK正常，所以仍保留SS、有限预算和真实反馈门。
 * 等待期间不消耗节点预算、不清错误、不复位主控或CAN控制器。 */
static bool bus_settled(uint64_t now, bool has_live_feedback) {
    twai_status_info_t can;
    if (twai_get_status_info(&can) != ESP_OK || can.state != TWAI_STATE_RUNNING ||
        can.tx_error_counter >= 128U || can.rx_error_counter >= 128U ||
        can.msgs_to_tx != 0U) {
        s_bus_ready_since_us = 0U;
        return false;
    }
    /* 另一台的真实在线反馈已证明接收链正在工作。正常CAN位期间RX也会为低，
     * 不能因此永久挡住单侧恢复；双侧都无新反馈时才使用上电等待条件。 */
    if (has_live_feedback) {
        s_bus_ready_since_us = now;
        return true;
    }
    if (gpio_get_level(g_exo_hardware_config.twai.rx_gpio) != 1) {
        s_bus_ready_since_us = 0U;
        return false;
    }
    if (s_bus_ready_since_us == 0U) s_bus_ready_since_us = now;
    return now - s_bus_ready_since_us >= RECOVERY_GRACE_US;
}

/* 只有真实、仍在线的反馈才算恢复，reporting_requested只是发送意图。
 * online从接收时间戳动态计算；旧REST/旧计数不能重置重试预算。 */
static bool observe_node(exo_actuator_id_t actuator, uint64_t now) {
    rs01_feedback_snapshot_t feedback;
    const bool online = rs01_get_feedback(actuator, &feedback) &&
        feedback.valid && feedback.online;
    portENTER_CRITICAL(&s_lock);
    recovery_node_t *node = &s_nodes[actuator];
    if (online) {
        node->offline_since_us = 0U;
        if (node->healthy_since_us == 0U) node->healthy_since_us = now;
        if (now - node->healthy_since_us >= RECOVERY_GRACE_US) {
            node->attempts = 0U;
            node->next_attempt_us = 0U;
        }
    } else {
        node->healthy_since_us = 0U;
        if (node->offline_since_us == 0U) node->offline_since_us = now;
    }
    portEXIT_CRITICAL(&s_lock);
    return online;
}

void exo_feedback_recovery_poll(void) {
    const uint64_t now = (uint64_t)esp_timer_get_time();
    bool online[EXO_ACTUATOR_COUNT];
    for (unsigned i = 0; i < EXO_ACTUATOR_COUNT; ++i)
        online[i] = observe_node((exo_actuator_id_t)i, now);
    const bool settled = bus_settled(now, online[0] || online[1]);

    /* 没有到期请求时不占维护锁，避免正常在线期间的5 Hz观察与START就绪检查竞争。 */
    bool pending = false;
    portENTER_CRITICAL(&s_lock);
    for (unsigned i = 0; i < EXO_ACTUATOR_COUNT; ++i) {
        const recovery_node_t *node = &s_nodes[i];
        pending = pending || (!online[i] && node->attempts < RECOVERY_ATTEMPTS &&
            now - node->offline_since_us >= RECOVERY_GRACE_US && now >= node->next_attempt_us);
    }
    portEXIT_CRITICAL(&s_lock);
    if (!pending || !settled) return;

    /* 与START/STOP/归零共用维护锁；运行/故障态不会发送恢复帧。
     * 每次轮询最多处理一个槽位，CAN不可用时不消耗预算，也不清错误计数。 */
    if (!exo_system_try_begin_maintenance()) return;
    if (can_usable()) {
        for (unsigned i = 0; i < EXO_ACTUATOR_COUNT; ++i) {
            if (online[i]) continue;
            portENTER_CRITICAL(&s_lock);
            recovery_node_t *node = &s_nodes[i];
            const bool due = node->attempts < RECOVERY_ATTEMPTS &&
                now - node->offline_since_us >= RECOVERY_GRACE_US &&
                now >= node->next_attempt_us;
            if (due) {
                ++node->attempts;
                node->next_attempt_us = now +
                    (RECOVERY_GRACE_US << (node->attempts - 1U));
            }
            portEXIT_CRITICAL(&s_lock);
            if (due) {
                /* 入队失败也消耗一次预算，防止故障环境无限刷CAN。
                 * 两帧只写10 ms周期和上报开关，不写模式/使能/运动/机械零位。 */
                (void)rs01_request_feedback_once((exo_actuator_id_t)i, 10U);
                break;
            }
        }
    }
    exo_system_end_maintenance();
}

esp_err_t exo_feedback_recovery_request(void) {
    if (!exo_system_try_begin_maintenance()) return ESP_ERR_INVALID_STATE;
    esp_err_t result = ESP_ERR_INVALID_STATE;
    if (can_usable()) {
        const uint64_t now = (uint64_t)esp_timer_get_time();
        portENTER_CRITICAL(&s_lock);
        if (!s_manual_requested || now - s_last_manual_us >= RECOVERY_MANUAL_COOLDOWN_US) {
            /* 仅重置有限预算，BLE worker不阻塞等待反馈；执行器稍后处理。
             * 在线节点下一轮会被observe跳过，不能重复干扰正常数据流。 */
            memset(s_nodes, 0, sizeof(s_nodes));
            s_last_manual_us = now;
            s_manual_requested = true;
            result = ESP_OK;
        } else result = ESP_ERR_TIMEOUT;
        portEXIT_CRITICAL(&s_lock);
    }
    exo_system_end_maintenance();
    return result;
}

void exo_feedback_recovery_get_snapshot(exo_feedback_recovery_snapshot_t *out) {
    if (out == NULL) return;
    portENTER_CRITICAL(&s_lock);
    for (unsigned i = 0; i < EXO_ACTUATOR_COUNT; ++i) {
        out->attempts[i] = s_nodes[i].attempts;
        out->exhausted[i] = s_nodes[i].attempts >= RECOVERY_ATTEMPTS;
    }
    portEXIT_CRITICAL(&s_lock);
}
