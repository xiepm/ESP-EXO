/*
 * SPDX-FileCopyrightText: 2010-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * ESP-EXO application entry point.
 *
 * Features:
 *  - Dual protocol: Lingzu private protocol + MIT protocol
 *  - One application-wide 100 Hz control loop
 *  - Interactive BLE/UART console for development commands
 *  - CAN bus feedback parsing (angle, speed, torque, temperature, faults)
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/twai.h"
#include "exo_control_loop.h"
#include "exo_config.h"
#include "hip_joint.h"
#include "rs01_driver.h"
#include "ble_console.h"

static const char *TAG = "main";

/* ================================================================
 *  TWAI RX Task
 *  Blocks on twai_receive(), then parses motor feedback.
 * ================================================================ */
static void twai_rx_task(void *arg)
{
    twai_message_t rx_msg;
    TickType_t last_feedback_log[EXO_ACTUATOR_COUNT] = {0};
    TickType_t last_invalid_log = 0;
    TickType_t last_unhandled_log = 0;
    ESP_LOGI(TAG, "TWAI RX task started");

    while (1) {
        esp_err_t err = twai_receive(&rx_msg, portMAX_DELAY);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "RX error: %s", esp_err_to_name(err));
            continue;
        }

        exo_actuator_id_t actuator = EXO_ACTUATOR_COUNT;
        rs01_rx_event_t event = rs01_process_can_frame(&rx_msg, &actuator);

        if (event == RS01_RX_EVENT_FEEDBACK &&
            actuator < EXO_ACTUATOR_COUNT) {
            rs01_feedback_snapshot_t feedback;
            if (!rs01_get_feedback(actuator, &feedback)) {
                continue;
            }

            TickType_t now = xTaskGetTickCount();
            if ((now - last_feedback_log[actuator]) >= pdMS_TO_TICKS(200)) {
                last_feedback_log[actuator] = now;
                exo_can_node_id_t can_id = 0;
                (void)rs01_get_can_id(actuator, &can_id);
                ESP_LOGI(TAG,
                         "FB: id=%d mode=%d status=%d ang=%.3f spd=%.3f tq=%.3f tmp=%.1f",
                         (int)can_id,
                         (int)event,
                         (int)feedback.mode,
                         (double)feedback.sample.position_rad,
                         (double)feedback.sample.velocity_rad_s,
                         (double)feedback.sample.torque_nm,
                         (double)feedback.sample.temperature_c);
            }
        } else if (event == RS01_RX_EVENT_FAULT &&
                   actuator < EXO_ACTUATOR_COUNT) {
            rs01_feedback_snapshot_t feedback;
            exo_can_node_id_t can_id = 0;
            if (rs01_get_feedback(actuator, &feedback) &&
                rs01_get_can_id(actuator, &can_id)) {
                ESP_LOGW(TAG,
                         "RS01 fault: id=%u fault=0x%08lX warning=0x%08lX",
                         can_id,
                         (unsigned long)feedback.fault_raw,
                         (unsigned long)feedback.warning_raw);
            }
        } else if (event == RS01_RX_EVENT_INVALID) {
            TickType_t now = xTaskGetTickCount();
            if ((now - last_invalid_log) >= pdMS_TO_TICKS(1000)) {
                last_invalid_log = now;
                ESP_LOGW(TAG,
                         "Invalid RS01 frame: id=0x%08lX dlc=%u flags=0x%lx",
                         (unsigned long)rx_msg.identifier,
                         rx_msg.data_length_code,
                         (unsigned long)rx_msg.flags);
            }
        } else if (event == RS01_RX_EVENT_NONE) {
            TickType_t now = xTaskGetTickCount();
            if ((now - last_unhandled_log) >= pdMS_TO_TICKS(1000)) {
                last_unhandled_log = now;
                ESP_LOGI(TAG,
                         "Unhandled CAN frame: id=0x%08lX dlc=%u flags=0x%lx",
                         (unsigned long)rx_msg.identifier,
                         rx_msg.data_length_code,
                         (unsigned long)rx_msg.flags);
            }
        }
    }
}

/* ================================================================
 *  TWAI Initialization
 * ================================================================ */
static void twai_init(void)
{
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        g_exo_hardware_config.twai.tx_gpio,
        g_exo_hardware_config.twai.rx_gpio,
        TWAI_MODE_NORMAL);
    /* Compound control commands generate several immediate replies while
     * proactive reporting can add 100 Hz feedback. The default five-frame
     * queue can overflow during that short burst. */
    g_config.rx_queue_len = g_exo_hardware_config.twai.rx_queue_length;
    twai_timing_config_t t_config  = g_exo_hardware_config.twai.timing;
    twai_filter_config_t f_config  = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI(TAG, "TWAI driver installed (%luMbps)",
             (unsigned long)(g_exo_hardware_config.twai.bitrate_bps /
                             1000000U));

    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(TAG, "TWAI driver started");
}

/* ================================================================
 *  app_main
 * ================================================================ */
void app_main(void)
{
    ESP_LOGI(TAG, "RobStride Motor Control - TWAI Master");
    ESP_LOGI(TAG, "TX: GPIO%d, RX: GPIO%d, Baud: %luMbps",
             g_exo_hardware_config.twai.tx_gpio,
             g_exo_hardware_config.twai.rx_gpio,
             (unsigned long)(g_exo_hardware_config.twai.bitrate_bps /
                             1000000U));

    ESP_ERROR_CHECK(rs01_driver_init());
    ESP_ERROR_CHECK(hip_joint_init());

    /* Initialize TWAI */
    twai_init();

    /* Create TWAI receive task */
    BaseType_t rx_task_result =
        xTaskCreate(twai_rx_task, "twai_rx", 4096, NULL, 8, NULL);
    ESP_ERROR_CHECK(rx_task_result == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);

    /* The RS01 report switch does not enable motor torque. A successful TX
     * records only that reporting was requested; online becomes true after
     * the driver receives a valid frame from the configured actuator. */
    for (exo_actuator_id_t actuator = EXO_ACTUATOR_A;
         actuator < EXO_ACTUATOR_COUNT;
         actuator = (exo_actuator_id_t)(actuator + 1)) {
        esp_err_t err = rs01_set_report_interval_ms(
            actuator, g_exo_hardware_config.rs01_report_interval_ms);
        if (err == ESP_OK) {
            err = rs01_start_reporting(actuator);
        }
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Actuator %c active reporting request failed: %s",
                     actuator == EXO_ACTUATOR_A ? 'A' : 'B',
                     esp_err_to_name(err));
        }
    }

    /* Finish the heavyweight NimBLE/NVS initialization before starting the
     * timing diagnostics of the permanent control task. The legacy imp_start
     * command explicitly rejects requests until that task reports alive. */
    ESP_ERROR_CHECK(ble_console_init());

    /* Create the permanent control task after all heavyweight subsystem
     * initialization. app_main then returns and performs no periodic work. */
    ESP_ERROR_CHECK(exo_control_loop_init());
}
