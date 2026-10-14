/* 编译真实RS01驱动，捕获实际构造的帧。不会打开UART/BLE/CAN硬件。 */
#include <assert.h>
#include <stdio.h>
#include "exo_types.h"
#define EXO_CONFIG_H
static const struct {
    struct { exo_can_node_id_t master_id; } twai;
    struct { exo_can_node_id_t can_id; } actuator[2];
    uint32_t rs01_feedback_timeout_ms;
} g_exo_hardware_config = {{0xFD}, {{0x10}, {0x20}}, 50};
#include "../main/rs01_driver.c"

static twai_status_info_t fake_can;
static twai_message_t sent[8];
static unsigned sent_count;
static bool fail_tx, bus_off_after_first;
int64_t esp_timer_get_time(void) { return 1000000; }
const char *esp_err_to_name(int error) { (void)error; return "fake"; }
SemaphoreHandle_t xSemaphoreCreateRecursiveMutexStatic(StaticSemaphore_t *buf) { return buf; }
BaseType_t xSemaphoreTakeRecursive(SemaphoreHandle_t mutex, TickType_t ticks) {
    assert(mutex != NULL); assert(ticks == 0 || ticks == portMAX_DELAY); return pdTRUE;
}
BaseType_t xSemaphoreGiveRecursive(SemaphoreHandle_t mutex) { assert(mutex != NULL); return pdTRUE; }
void vTaskDelay(TickType_t ticks) { (void)ticks; }
esp_err_t twai_get_status_info(twai_status_info_t *out) { *out = fake_can; return ESP_OK; }
esp_err_t twai_transmit(const twai_message_t *message, unsigned ticks) {
    assert(ticks == 0 && sent_count < 8); sent[sent_count++] = *message;
    if (bus_off_after_first) fake_can.state = TWAI_STATE_BUS_OFF;
    return fail_tx ? ESP_ERR_TIMEOUT : ESP_OK;
}
static void reset(void) {
    fake_can = (twai_status_info_t){.state = TWAI_STATE_RUNNING};
    sent_count = 0; fail_tx = bus_off_after_first = false;
    assert(rs01_driver_init() == ESP_OK);
    s_motors[0].feedback.reporting_requested = false;
}
int main(void) {
    reset(); assert(rs01_request_feedback_once(EXO_ACTUATOR_A, 10) == ESP_OK);
    assert(sent_count == 2);
    assert(sent[0].identifier == 0x1200FD10 && sent[1].identifier == 0x1800FD10);
    const uint8_t interval[8] = {0x26, 0x70, 0, 0, 0, 0, 0, 0};
    const uint8_t report[8] = {1, 2, 3, 4, 5, 6, 1, 8};
    for (unsigned i = 0; i < 2; ++i) assert(sent[i].flags == (TWAI_MSG_FLAG_EXTD | TWAI_MSG_FLAG_SS) && sent[i].data_length_code == 8);
    assert(memcmp(sent[0].data, interval, 8) == 0 && memcmp(sent[1].data, report, 8) == 0);
    rs01_feedback_snapshot_t feedback; assert(rs01_get_feedback(EXO_ACTUATOR_A, &feedback));
    assert(feedback.reporting_requested && !feedback.online && !feedback.valid && !feedback.enabled_requested);
    puts("PASS exact two SS maintenance frames; enqueue is not online/enable");

    reset(); assert(rs01_request_feedback_once(EXO_ACTUATOR_B, 15) == ESP_OK);
    assert(sent[0].identifier == 0x1200FD20 && sent[1].identifier == 0x1800FD20 && sent[0].data[4] == 1);
    puts("PASS slot B and report period encoding");
    reset(); fail_tx = true;
    assert(rs01_request_feedback_once(EXO_ACTUATOR_A, 10) == ESP_ERR_TIMEOUT && sent_count == 1);
    assert(rs01_get_feedback(EXO_ACTUATOR_A, &feedback) && !feedback.reporting_requested);
    puts("PASS failed first frame cannot mark reporting requested");
    reset(); bus_off_after_first = true;
    assert(rs01_request_feedback_once(EXO_ACTUATOR_A, 10) == ESP_ERR_INVALID_STATE && sent_count == 1);
    puts("PASS CAN health rechecked between frames");
    reset(); fake_can.rx_error_counter = 128;
    assert(rs01_request_feedback_once(EXO_ACTUATOR_A, 10) == ESP_ERR_INVALID_STATE && sent_count == 0);
    fake_can.rx_error_counter = 0; fake_can.tx_error_counter = 128;
    assert(rs01_request_feedback_once(EXO_ACTUATOR_A, 10) == ESP_ERR_INVALID_STATE && sent_count == 0);
    puts("PASS passive error threshold blocks TX");
    reset(); assert(rs01_request_feedback_once(EXO_ACTUATOR_COUNT, 10) == ESP_ERR_INVALID_ARG);
    assert(rs01_request_feedback_once(EXO_ACTUATOR_A, 9) == ESP_ERR_INVALID_ARG);
    assert(rs01_request_feedback_once(EXO_ACTUATOR_A, 11) == ESP_ERR_INVALID_ARG && sent_count == 0);
    puts("PASS invalid slot/interval rejected");
    puts("RESULT 6/6 PASS");
}
