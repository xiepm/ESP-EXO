/* 真实恢复模块 + 可控制的RTOS/CAN/反馈桩。不打开串口、不操作电机。
 * 验证预算与门禁；不能代替电机后上电/断开重连的目标板验收。 */
#include <assert.h>
#include <stdio.h>
#include "driver/gpio.h"
#define EXO_CONFIG_H
static const struct { struct { gpio_num_t rx_gpio; } twai; }
    g_exo_hardware_config = {{1}};
#include "../main/exo_feedback_recovery.c"

static uint64_t fake_us;
static twai_status_info_t fake_can;
static rs01_feedback_snapshot_t motors[2];
static bool idle, maintenance_locked;
static int fake_rx_level;
static unsigned requests[2], maintenance_ends;
int64_t esp_timer_get_time(void) { return (int64_t)fake_us; }
int gpio_get_level(gpio_num_t gpio) { assert(gpio == 1); return fake_rx_level; }
esp_err_t twai_get_status_info(twai_status_info_t *out) { *out = fake_can; return ESP_OK; }
bool rs01_get_feedback(exo_actuator_id_t actuator, rs01_feedback_snapshot_t *out) {
    assert(actuator < 2); *out = motors[actuator]; return true;
}
bool exo_system_try_begin_maintenance(void) {
    if (!idle || maintenance_locked) return false;
    maintenance_locked = true; return true;
}
void exo_system_end_maintenance(void) { assert(maintenance_locked); maintenance_locked = false; ++maintenance_ends; }
esp_err_t rs01_request_feedback_once(exo_actuator_id_t actuator, uint32_t interval_ms) {
    assert(maintenance_locked && idle && interval_ms == 10); ++requests[actuator]; return ESP_OK;
}
static void reset(void) {
    memset(s_nodes, 0, sizeof(s_nodes)); memset(motors, 0, sizeof(motors));
    memset(requests, 0, sizeof(requests)); fake_us = 1; idle = true;
    fake_can = (twai_status_info_t){.state = TWAI_STATE_RUNNING};
    maintenance_locked = s_manual_requested = false; s_last_manual_us = 0;
    s_bus_ready_since_us = 0; fake_rx_level = 1;
    maintenance_ends = 0;
}
static void advance(unsigned milliseconds) { fake_us += milliseconds * 1000ULL; exo_feedback_recovery_poll(); }
static void online(unsigned i, bool value) {
    motors[i].online = value; motors[i].valid = true;
    motors[i].reporting_requested = true;
}
int main(void) {
    reset(); exo_feedback_recovery_poll();
    assert(requests[0] == 0 && requests[1] == 0);
    for (unsigned i = 0; i < 500; ++i) advance(200);
    assert(requests[0] == 3 && requests[1] == 3);
    exo_feedback_recovery_snapshot_t out; exo_feedback_recovery_get_snapshot(&out);
    assert(out.exhausted[0] && out.exhausted[1]);
    puts("PASS absent motors: grace, backoff, finite per-node budget");

    /* 请求上报 + 曾有效仍不算在线。短暂一帧不能重新获得无限预算。 */
    online(0, true); advance(200); online(0, false); advance(2000);
    assert(requests[0] == 3); motors[0].reporting_requested = true;
    advance(2000); assert(requests[0] == 3);
    puts("PASS stale/requested/transient feedback cannot rearm budget");

    online(0, true); advance(200); advance(1200);
    exo_feedback_recovery_get_snapshot(&out); assert(!out.exhausted[0]);
    online(0, false); advance(200); advance(1000);
    assert(requests[0] == 4 && requests[1] == 3);
    puts("PASS sustained real feedback rearms only recovered node");

    reset(); online(0, true); exo_feedback_recovery_poll();
    for (unsigned i = 0; i < 100; ++i) advance(200);
    assert(requests[0] == 0 && requests[1] == 3);
    puts("PASS online motor never receives recovery frames");

    for (unsigned state = 0; state < 4; ++state) {
        if (state == TWAI_STATE_RUNNING) continue;
        reset(); fake_can.state = (twai_state_t)state;
        for (unsigned i = 0; i < 30; ++i) advance(200);
        assert(requests[0] == 0 && requests[1] == 0);
        assert(exo_feedback_recovery_request() == ESP_ERR_INVALID_STATE);
    }
    reset(); fake_can.rx_error_counter = 128; advance(2000); advance(2000);
    assert(requests[0] == 0 && exo_feedback_recovery_request() == ESP_ERR_INVALID_STATE);
    reset(); fake_can.tx_error_counter = 128; advance(2000); advance(2000);
    assert(requests[0] == 0);
    puts("PASS stopped/BUS_OFF/recovering/error-passive CAN blocked");

    reset(); idle = false; for (unsigned i = 0; i < 30; ++i) advance(200);
    assert(requests[0] == 0 && requests[1] == 0);
    assert(exo_feedback_recovery_request() == ESP_ERR_INVALID_STATE);
    idle = true; maintenance_locked = true; advance(2000); assert(requests[0] == 0);
    puts("PASS non-IDLE and concurrent maintenance cannot send");

    reset(); assert(exo_feedback_recovery_request() == ESP_OK);
    assert(requests[0] == 0 && requests[1] == 0);
    assert(exo_feedback_recovery_request() == ESP_ERR_TIMEOUT);
    advance(10000); assert(exo_feedback_recovery_request() == ESP_OK);
    puts("PASS manual retry asynchronous and limited to one per 10 seconds");
    reset(); fake_rx_level = 0; exo_feedback_recovery_poll();
    for (unsigned i = 0; i < 300; ++i) advance(200);
    exo_feedback_recovery_get_snapshot(&out);
    assert(requests[0] == 0 && requests[1] == 0 && !out.exhausted[0] && !out.exhausted[1]);
    fake_rx_level = 1; advance(200); advance(800);
    assert(requests[0] == 0 && requests[1] == 0);
    advance(200); assert(requests[0] == 1 && requests[1] == 0);
    puts("PASS late power: persistent RX low queues no frames/consumes no budget; high needs grace");

    reset(); fake_can.msgs_to_tx = 6; exo_feedback_recovery_poll();
    for (unsigned i = 0; i < 100; ++i) advance(200);
    assert(requests[0] == 0 && requests[1] == 0);
    fake_can.msgs_to_tx = 0; advance(200); advance(800);
    fake_rx_level = 0; advance(200); fake_rx_level = 1;
    advance(200); advance(800); assert(requests[0] == 0 && requests[1] == 0);
    advance(200); assert(requests[0] == 1);
    puts("PASS pending TX or new low sample restarts bus settle grace without spending budget");
    reset(); fake_rx_level = 0; online(0, true); exo_feedback_recovery_poll();
    for (unsigned i = 0; i < 100; ++i) advance(200);
    assert(requests[0] == 0 && requests[1] == 3);
    puts("PASS real online peer permits single-side recovery during ordinary low CAN bits");
    puts("RESULT 10/10 PASS");
}
