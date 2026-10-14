#ifndef HOST_SYSTEM_TWAI_H
#define HOST_SYSTEM_TWAI_H
#include <stdint.h>
#include "esp_err.h"
typedef enum { TWAI_STATE_STOPPED, TWAI_STATE_RUNNING, TWAI_STATE_BUS_OFF,
               TWAI_STATE_RECOVERING } twai_state_t;
typedef struct {
    twai_state_t state;
    uint32_t msgs_to_tx, msgs_to_rx, tx_error_counter, rx_error_counter;
    uint32_t tx_failed_count, rx_missed_count, rx_overrun_count;
    uint32_t arb_lost_count, bus_error_count;
} twai_status_info_t;
typedef struct { uint32_t flags, identifier; uint8_t data_length_code, data[8]; } twai_message_t;
#define TWAI_MSG_FLAG_NONE 0U
#define TWAI_MSG_FLAG_EXTD 1U
#define TWAI_MSG_FLAG_RTR 2U
#define TWAI_MSG_FLAG_SS 8U
esp_err_t twai_transmit(const twai_message_t *message, unsigned ticks);
esp_err_t twai_get_status_info(twai_status_info_t *status);
#endif
