/*
 * SPDX-FileCopyrightText: 2010-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * RobStride Motor Control via TWAI (CAN bus)
 * Ported from STM32F1 RobStrideSPC project.
 *
 * Features:
 *  - Dual protocol: Lingzu private protocol + MIT protocol
 *  - Interactive BLE console for motor control commands
 *  - CAN bus feedback parsing (angle, speed, torque, temperature, faults)
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/twai.h"
#include "driver/gpio.h"
#include "robstride_control.h"
#include "ble_console.h"

/* ---- TWAI Config ---- */
#define TWAI_TX_GPIO  CONFIG_EXAMPLE_TX_GPIO_NUM
#define TWAI_RX_GPIO  CONFIG_EXAMPLE_RX_GPIO_NUM

static const char *TAG = "main";

/* ================================================================
 *  TWAI RX Task
 *  Blocks on twai_receive(), then parses motor feedback.
 * ================================================================ */
static void twai_rx_task(void *arg)
{
    twai_message_t rx_msg;
    TickType_t last_proactive_log = 0;
    ESP_LOGI(TAG, "TWAI RX task started");

    while (1) {
        esp_err_t err = twai_receive(&rx_msg, portMAX_DELAY);
        if (err == ESP_OK) {
            if ((rx_msg.flags & TWAI_MSG_FLAG_RTR) != 0) {
                ESP_LOGW(TAG, "Ignoring unsupported CAN frame: dlc=%u flags=0x%lx",
                         rx_msg.data_length_code, (unsigned long)rx_msg.flags);
                continue;
            }

            bool is_extended = (rx_msg.flags & TWAI_MSG_FLAG_EXTD) != 0;
            if (!is_extended && rx_msg.data_length_code == 5) {
                uint32_t fault_value = (uint32_t)rx_msg.data[1] |
                                       ((uint32_t)rx_msg.data[2] << 8) |
                                       ((uint32_t)rx_msg.data[3] << 16) |
                                       ((uint32_t)rx_msg.data[4] << 24);
                robstride_update_mit_faults(fault_value);
                ESP_LOGI(TAG, "MIT FAULT: id=%u raw=0x%08lX",
                         rx_msg.data[0], (unsigned long)fault_value);
                continue;
            }

            if (rx_msg.data_length_code != 8) {
                ESP_LOGW(TAG, "Ignoring unsupported CAN frame: dlc=%u flags=0x%lx",
                         rx_msg.data_length_code, (unsigned long)rx_msg.flags);
                continue;
            }

            /* Save raw data to global buffer for inspection */
            memcpy(g_rx_data, rx_msg.data, 8);

            /* Determine frame type and parse */
            uint32_t ext_id = is_extended ? rx_msg.identifier : 0;
            uint32_t std_id = is_extended ? 0 : rx_msg.identifier;
            uint8_t frame_mode = is_extended ? ext_id_get_mode(ext_id) : 0;

            bool log_frame = true;
            if (frame_mode == MOTOR_Proactive) {
                TickType_t now = xTaskGetTickCount();
                if ((now - last_proactive_log) < pdMS_TO_TICKS(100)) {
                    log_frame = false;
                } else {
                    last_proactive_log = now;
                }
            }

            if (log_frame) {
                ESP_LOGI(TAG, "RX: ext=0x%08lX std=0x%03lX dlc=%d "
                         "data=%02X%02X%02X%02X%02X%02X%02X%02X",
                         ext_id, std_id, rx_msg.data_length_code,
                         rx_msg.data[0], rx_msg.data[1], rx_msg.data[2], rx_msg.data[3],
                         rx_msg.data[4], rx_msg.data[5], rx_msg.data[6], rx_msg.data[7]);
            }

            bool is_motor_feedback = (!is_extended && std_id == MASTER_ID) ||
                                     frame_mode == MOTOR_FEEDBACK ||
                                     frame_mode == MOTOR_Proactive;

            if (is_motor_feedback) {
                parse_motor_feedback(is_extended, rx_msg.identifier, rx_msg.data);
                motor_feedback_t feedback;
                robstride_get_feedback_snapshot(&feedback);
                ble_console_push_feedback(&feedback);

                /* In private feedback frames the source motor ID is in the
                 * low byte of the extended-ID data field; the low ID byte is
                 * the destination master ID. */
                uint8_t source_id = is_extended
                    ? (uint8_t)(ext_id_get_data(ext_id) & 0xFF)
                    : rx_msg.data[0];
                if (log_frame) {
                    ESP_LOGI(TAG, "FB: id=%d mode=%d status=%d ang=%.3f spd=%.3f tq=%.3f tmp=%.1f",
                             (int)source_id,
                             (int)frame_mode,
                             (int)feedback.mms,
                             (double)feedback.angle,
                             (double)feedback.speed,
                             (double)feedback.torque,
                             (double)feedback.temperature);
                }
            } else if (frame_mode == MOTOR_PARAREAD) {
                uint16_t index = (uint16_t)rx_msg.data[0] |
                                 ((uint16_t)rx_msg.data[1] << 8);
                uint32_t raw_value = (uint32_t)rx_msg.data[4] |
                                     ((uint32_t)rx_msg.data[5] << 8) |
                                     ((uint32_t)rx_msg.data[6] << 16) |
                                     ((uint32_t)rx_msg.data[7] << 24);
                float float_value;
                memcpy(&float_value, &raw_value, sizeof(float_value));
                ESP_LOGI(TAG,
                         "PARAM: index=0x%04X raw=0x%08lX uint=%lu float=%g",
                         index, (unsigned long)raw_value,
                         (unsigned long)raw_value, (double)float_value);
            } else if (!is_extended) {
                ESP_LOGI(TAG,
                         "MIT DEVICE RESPONSE: std=0x%03lX data=%02X%02X%02X%02X%02X%02X%02X%02X",
                         (unsigned long)std_id,
                         rx_msg.data[0], rx_msg.data[1], rx_msg.data[2],
                         rx_msg.data[3], rx_msg.data[4], rx_msg.data[5],
                         rx_msg.data[6], rx_msg.data[7]);
            } else {
                ESP_LOGI(TAG, "Ignoring non-feedback private frame mode=0x%02X",
                         frame_mode);
            }
        } else {
            ESP_LOGW(TAG, "RX error: %s", esp_err_to_name(err));
        }
    }
}

/* ================================================================
 *  TWAI Initialization
 * ================================================================ */
static void twai_init(void)
{
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        TWAI_TX_GPIO, TWAI_RX_GPIO, TWAI_MODE_NORMAL);
    /* Compound control commands generate several immediate replies while
     * proactive reporting can add 100 Hz feedback. The default five-frame
     * queue can overflow during that short burst. */
    g_config.rx_queue_len = 32;
    twai_timing_config_t t_config  = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_config  = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI(TAG, "TWAI driver installed (1Mbps)");

    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(TAG, "TWAI driver started");
}

/* ================================================================
 *  app_main
 * ================================================================ */
void app_main(void)
{
    ESP_LOGI(TAG, "RobStride Motor Control - TWAI Master");
    ESP_LOGI(TAG, "TX: GPIO%d, RX: GPIO%d, Baud: 1Mbps",
             TWAI_TX_GPIO, TWAI_RX_GPIO);

    ESP_ERROR_CHECK(robstride_control_init());

    /* Initialize TWAI */
    twai_init();

    /* Create TWAI receive task */
    xTaskCreate(twai_rx_task, "twai_rx", 4096, NULL, 8, NULL);

    /* Initialize BLE console (creates its own task) */
    ESP_ERROR_CHECK(ble_console_init());

    /* Wait forever */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    /* Unreachable, but for completeness */
    twai_stop();
    twai_driver_uninstall();
}
