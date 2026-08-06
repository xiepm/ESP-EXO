#include "ble_console.h"
#include "exo_control_loop.h"
#include "exo_config.h"
#include "hip_joint.h"
#include "rs01_driver.h"
#include "impedance_control.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "driver/gpio.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_att.h"
#include "host/ble_gap.h"
#include "host/util/util.h"
#include "services/gatt/ble_svc_gatt.h"
#include "services/gap/ble_svc_gap.h"
#include <string.h>
#include <strings.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <errno.h>
#include <math.h>

static const char *TAG = "ble_console";

/* ---- NUS UUIDs (128-bit, BLE byte order) ----
 * Service:  6E400001-B5A3-F393-E0A9-E50E24DCCA9E
 * RX Char:  6E400002-B5A3-F393-E0A9-E50E24DCCA9E  (phone -> ESP32, Write)
 * TX Char:  6E400003-B5A3-F393-E0A9-E50E24DCCA9E  (ESP32 -> phone, Notify)
 */
static const ble_uuid128_t s_nus_svc_uuid =
    BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
                     0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E);
static const ble_uuid128_t s_nus_rx_uuid =
    BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
                     0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E);
static const ble_uuid128_t s_nus_tx_uuid =
    BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
                     0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E);

/* ---- BLE state ---- */
static uint16_t s_conn_handle       = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_tx_attr_handle    = 0;
static bool     s_notify_enabled    = false;
static bool     s_welcome_pending   = false;
static uint8_t  s_own_addr_type     = BLE_OWN_ADDR_PUBLIC;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static SemaphoreHandle_t s_tx_mutex;
static SemaphoreHandle_t s_cmd_mutex;

/* ---- RX line buffer ---- */
#define LINE_BUF_SIZE  256
#define EXO_CONSOLE_TASK_CORE 0
static StreamBufferHandle_t s_rx_streambuf;

/* ---- Current target logical actuator; A/B do not imply left/right. ---- */
static exo_actuator_id_t s_selected_actuator;

/* ---- Forward declarations ---- */
static int  ble_gap_event_handler(struct ble_gap_event *event, void *arg);
static void ble_start_advertising(void);
static void ble_console_task(void *arg);
static void uart_console_task(void *arg);
static void console_process_line(char *line);
static void ble_vprintf(const char *fmt, ...);
static bool parse_long_arg(const char *text, long min_value, long max_value,
                           long *value);
static bool parse_float_arg(const char *text, float *value);
static bool parse_joint_arg(const char *text, exo_hip_joint_id_t *joint);
static exo_can_node_id_t selected_can_id(void);
static bool driver_command_succeeded(esp_err_t err);

/* ---- Command handler type ---- */
typedef void (*cmd_handler_t)(int argc, char *argv[]);

typedef struct {
    const char    *name;
    const char    *help;
    int            min_args;
    cmd_handler_t  handler;
} console_cmd_t;

/* ---- Command handler forward declarations ---- */
static void cmd_motor(int argc, char *argv[]);
static void cmd_enable(int argc, char *argv[]);
static void cmd_disable(int argc, char *argv[]);
static void cmd_move(int argc, char *argv[]);
static void cmd_speed(int argc, char *argv[]);
static void cmd_pos_pp(int argc, char *argv[]);
static void cmd_pos_csp(int argc, char *argv[]);
static void cmd_current(int argc, char *argv[]);
static void cmd_set_zero(int argc, char *argv[]);
static void cmd_set_id(int argc, char *argv[]);
static void cmd_save(int argc, char *argv[]);
static void cmd_set_baud(int argc, char *argv[]);
static void cmd_report(int argc, char *argv[]);
static void cmd_switch_mit(int argc, char *argv[]);
static void cmd_switch_private(int argc, char *argv[]);
static void cmd_read_param(int argc, char *argv[]);
static void cmd_mit_enable(int argc, char *argv[]);
static void cmd_mit_disable(int argc, char *argv[]);
static void cmd_mit_control(int argc, char *argv[]);
static void cmd_mit_pos(int argc, char *argv[]);
static void cmd_mit_speed(int argc, char *argv[]);
static void cmd_mit_set_zero(int argc, char *argv[]);
static void cmd_mit_set_type(int argc, char *argv[]);
static void cmd_mit_set_id(int argc, char *argv[]);
static void cmd_mit_save(int argc, char *argv[]);
static void cmd_mit_report(int argc, char *argv[]);
static void cmd_mit_read(int argc, char *argv[]);
static void cmd_mit_write(int argc, char *argv[]);
static void cmd_mit_error(int argc, char *argv[]);
static void cmd_motor_type(int argc, char *argv[]);
static void cmd_status(int argc, char *argv[]);
static void cmd_joint_status(int argc, char *argv[]);
static void cmd_joint_zero(int argc, char *argv[]);
static void cmd_joint_zero_clear(int argc, char *argv[]);
static void cmd_control_status(int argc, char *argv[]);
static void cmd_can_status(int argc, char *argv[]);
static void cmd_impedance_start(int argc, char *argv[]);
static void cmd_impedance_stop(int argc, char *argv[]);
static void cmd_help(int argc, char *argv[]);

/* ---- Command table ---- */
static const console_cmd_t s_cmd_table[] = {
    {"motor",      "Select target: <A|B|configured_can_id>",        1, cmd_motor},
    {"enable",     "Motor enable (private protocol)",               0, cmd_enable},
    {"disable",    "Motor disable/reset (private protocol)",        0, cmd_disable},
    {"move",       "Motion control: <t> <pos> <spd> <kp> <kd>",    5, cmd_move},
    {"speed",      "Speed control: <cur_limit> <acc> <speed>",      3, cmd_speed},
    {"pos_pp",     "PP position: <speed> <acc> <angle>",            3, cmd_pos_pp},
    {"pos_csp",    "CSP position: <speed> <angle>",                 2, cmd_pos_csp},
    {"current",    "Current control: <current>",                    1, cmd_current},
    {"set_zero",   "Set RS01 motor mechanical zero (CAN)",         0, cmd_set_zero},
    {"set_id",     "Set motor CAN ID: <new_id>",                    1, cmd_set_id},
    {"save",       "Save parameters to flash",                      0, cmd_save},
    {"set_baud",   "Change baud: <1=1M|2=500K|3=250K|4=125K>",     1, cmd_set_baud},
    {"report",     "Proactive report: <0=off|1=on>",                1, cmd_report},
    {"switch_mit", "Switch protocol to MIT",                        0, cmd_switch_mit},
    {"switch_private", "Switch protocol to private",                 0, cmd_switch_private},
    {"read_param", "Read parameter (private): <reg_addr>",          1, cmd_read_param},
    {"mit_enable",   "MIT motor enable",                            0, cmd_mit_enable},
    {"mit_disable",  "MIT motor disable",                           0, cmd_mit_disable},
    {"mit_control",  "MIT control: <ang> <spd> <kp> <kd> <t>",     5, cmd_mit_control},
    {"mit_pos",      "MIT position: <pos> <spd>",                   2, cmd_mit_pos},
    {"mit_speed",    "MIT speed: <spd> <cur>",                      2, cmd_mit_speed},
    {"mit_set_zero", "Set MIT motor mechanical zero (CAN)",        0, cmd_mit_set_zero},
    {"mit_set_type", "MIT set motor type: <0-2>",                   1, cmd_mit_set_type},
    {"mit_set_id",   "MIT set motor ID: <new_id>",                  1, cmd_mit_set_id},
    {"mit_save",     "MIT save to flash",                           0, cmd_mit_save},
    {"mit_report",   "MIT proactive report: <0=off|1=on>",          1, cmd_mit_report},
    {"mit_read",     "MIT parameter read: <reg_addr>",               1, cmd_mit_read},
    {"mit_write",    "MIT parameter write: <reg_addr> <value>",      2, cmd_mit_write},
    {"mit_error",    "MIT error query/clear: <0-255|0xFF=clear>",    1, cmd_mit_error},
    {"motor_type",   "Show RS01 driver limits",                     0, cmd_motor_type},
    {"status",       "Show motor feedback status",                  0, cmd_status},
    {"joint_status", "Show hip joint: <left|right>",                1, cmd_joint_status},
    {"joint_zero",   "Capture RAM software zero: <left|right>",    1, cmd_joint_zero},
    {"joint_zero_clear", "Clear RAM software zero: <left|right>", 1, cmd_joint_zero_clear},
    {"control_status", "Show the permanent 100 Hz control loop",  0, cmd_control_status},
    {"can_status",   "Show TWAI controller and error counters",     0, cmd_can_status},
    {"imp_start",   "Start dual-motor impedance control",            0, cmd_impedance_start},
    {"imp_stop",    "Stop dual-motor impedance control",             0, cmd_impedance_stop},
    {"help",         "Show this help",                              0, cmd_help},
    {NULL, NULL, 0, NULL},
};

/* ================================================================
 *  BLE TX helper — send notification to connected client
 * ================================================================ */
static void ble_tx_send(const char *data, size_t len)
{
    if (data == NULL || len == 0 || s_tx_mutex == NULL) {
        return;
    }

    xSemaphoreTake(s_tx_mutex, portMAX_DELAY);

    portENTER_CRITICAL(&s_lock);
    bool can_send = s_notify_enabled &&
                    (s_conn_handle != BLE_HS_CONN_HANDLE_NONE);
    uint16_t conn = s_conn_handle;
    uint16_t attr = s_tx_attr_handle;
    portEXIT_CRITICAL(&s_lock);

    if (!can_send) {
        xSemaphoreGive(s_tx_mutex);
        return;
    }

    uint16_t mtu = ble_att_mtu(conn);
    size_t chunk_size = mtu > 3 ? (size_t)(mtu - 3) : 20;
    size_t offset = 0;

    while (offset < len) {
        size_t chunk_len = len - offset;
        if (chunk_len > chunk_size) {
            chunk_len = chunk_size;
        }

        int rc = BLE_HS_ENOMEM;
        for (int attempt = 0; attempt < 3; ++attempt) {
            struct os_mbuf *om =
                ble_hs_mbuf_from_flat(data + offset, (uint16_t)chunk_len);
            if (om != NULL) {
                rc = ble_gatts_notify_custom(conn, attr, om);
            }
            if (rc == 0) {
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        if (rc != 0) {
            ESP_LOGW(TAG, "Notify failed at offset %u: %d",
                     (unsigned)offset, rc);
            break;
        }
        offset += chunk_len;
    }

    xSemaphoreGive(s_tx_mutex);
}

/* ---- printf-style output via BLE ---- */
static void ble_vprintf(const char *fmt, ...)
{
    char buf[LINE_BUF_SIZE];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (len <= 0) return;
    if (len >= sizeof(buf)) len = sizeof(buf) - 1;

    buf[len] = '\0';

    /* The programming console is also a command channel. Keep responses as
     * plain text so they are easy to use from idf.py monitor or a terminal. */
    fwrite(buf, 1, (size_t)len, stdout);
    fflush(stdout);

    /* Send via BLE */
    ble_tx_send(buf, (size_t)len);
}

/* ================================================================
 *  Connection status (public API)
 * ================================================================ */
bool ble_console_is_connected(void)
{
    portENTER_CRITICAL(&s_lock);
    bool connected = s_notify_enabled &&
                     (s_conn_handle != BLE_HS_CONN_HANDLE_NONE);
    portEXIT_CRITICAL(&s_lock);
    return connected;
}

/* ================================================================
 *  GATT RX callback — called when phone writes to RX characteristic
 * ================================================================ */
static int ble_gatt_rx_cb(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR && ctxt->om != NULL) {
        /* Accumulate received bytes into stream buffer */
        struct os_mbuf *om = ctxt->om;
        while (om != NULL) {
            if (om->om_len > 0) {
                size_t sent = xStreamBufferSend(s_rx_streambuf,
                                                om->om_data, om->om_len, 0);
                if (sent != om->om_len) {
                    ESP_LOGW(TAG, "RX stream buffer full, dropped %d bytes",
                             (int)(om->om_len - sent));
                }
            }
            om = SLIST_NEXT(om, om_next);
        }
    }
    return 0;
}

/* ---- GATT TX callback (notify-only, no read/write from client) ---- */
static int ble_gatt_tx_cb(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    /* TX characteristic is notify-only — reject any client access */
    return BLE_ATT_ERR_UNLIKELY;
}

/* ================================================================
 *  GAP event handler
 * ================================================================ */
static int ble_gap_event_handler(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {

    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            ESP_LOGI(TAG, "BLE client connected");
            portENTER_CRITICAL(&s_lock);
            s_conn_handle = event->connect.conn_handle;
            s_notify_enabled = false;
            portEXIT_CRITICAL(&s_lock);
        } else {
            ESP_LOGW(TAG, "Connection failed: %d", event->connect.status);
            ble_start_advertising();
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "BLE client disconnected (reason=%d)",
                 event->disconnect.reason);
        portENTER_CRITICAL(&s_lock);
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        s_notify_enabled = false;
        s_welcome_pending = false;
        portEXIT_CRITICAL(&s_lock);
        ble_start_advertising();
        break;

    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == s_tx_attr_handle) {
            bool enabled = event->subscribe.cur_notify > 0;
            ESP_LOGI(TAG, "TX notifications %s",
                     enabled ? "subscribed" : "unsubscribed");
            portENTER_CRITICAL(&s_lock);
            s_notify_enabled = enabled;
            s_welcome_pending = enabled;
            portEXIT_CRITICAL(&s_lock);
        }
        break;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU updated: %d", event->mtu.value);
        break;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI(TAG, "Advertising completed (reason=%d), restarting",
                 event->adv_complete.reason);
        ble_start_advertising();
        break;

    default:
        break;
    }
    return 0;
}

/* ---- NUS service definitions (file scope, because NimBLE stores pointer, not copy) ---- */
static const struct ble_gatt_svc_def s_nus_svc_defs[] = {
    {
        .type   = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid   = &s_nus_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid       = &s_nus_rx_uuid.u,
                .access_cb  = ble_gatt_rx_cb,
                .flags      = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
                .val_handle = NULL,
            },
            {
                .uuid       = &s_nus_tx_uuid.u,
                .access_cb  = ble_gatt_tx_cb,
                .flags      = BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &s_tx_attr_handle,
            },
            {
                0, /* No more characteristics */
            }
        },
    },
    {
        0, /* No more services */
    },
};

/* ================================================================
 *  NimBLE GATT service registration
 * ================================================================ */
static int ble_gatt_svr_init(void)
{
    int rc;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    /* Register the NUS service + RX + TX characteristics */
    rc = ble_gatts_count_cfg((struct ble_gatt_svc_def *)s_nus_svc_defs);
    if (rc != 0) {
        ESP_LOGE(TAG, "count_cfg svcs failed: %d", rc);
        return rc;
    }

    rc = ble_gatts_add_svcs(s_nus_svc_defs);
    if (rc != 0) {
        ESP_LOGE(TAG, "add_svcs failed: %d", rc);
        return rc;
    }

    ESP_LOGI(TAG, "GATT services registered OK");
    return 0;
}

/* ================================================================
 *  Advertising
 * ================================================================ */
static void ble_start_advertising(void)
{
    struct ble_gap_adv_params adv_params = {0};
    struct ble_hs_adv_fields fields = {0};
    int rc;

    /* Configure advertising data: flags + TX power + device name */
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = 0;
    fields.name = (const uint8_t *)g_exo_hardware_config.ble_device_name;
    fields.name_len = strlen(g_exo_hardware_config.ble_device_name);
    fields.name_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to set advertising fields: %d", rc);
        return;
    }

    adv_params.conn_mode     = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode     = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, ble_gap_event_handler, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to start advertising: %d", rc);
    } else {
        ESP_LOGI(TAG, "Advertising started as '%s'",
                 g_exo_hardware_config.ble_device_name);
    }
}

/* ================================================================
 *  NimBLE host task callbacks
 * ================================================================ */
static void ble_on_reset(int reason)
{
    ESP_LOGE(TAG, "NimBLE host reset (reason=%d)", reason);
    portENTER_CRITICAL(&s_lock);
    s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
    s_notify_enabled = false;
    s_welcome_pending = false;
    portEXIT_CRITICAL(&s_lock);
}

static void ble_on_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "No usable BLE identity address: %d", rc);
        return;
    }

    rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to infer BLE address type: %d", rc);
        return;
    }

    ESP_LOGI(TAG, "NimBLE host synced, starting advertising");
    ble_start_advertising();
}

static void ble_host_task(void *arg)
{
    ESP_LOGI(TAG, "NimBLE host task started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static bool parse_long_arg(const char *text, long min_value, long max_value,
                           long *value)
{
    if (text == NULL || value == NULL) {
        return false;
    }

    errno = 0;
    char *end = NULL;
    long parsed = strtol(text, &end, 0);
    if (errno != 0 || end == text || *end != '\0' ||
        parsed < min_value || parsed > max_value) {
        return false;
    }

    *value = parsed;
    return true;
}

static bool parse_float_arg(const char *text, float *value)
{
    if (text == NULL || value == NULL) {
        return false;
    }

    errno = 0;
    char *end = NULL;
    float parsed = strtof(text, &end);
    if (errno != 0 || end == text || *end != '\0' || !isfinite(parsed)) {
        return false;
    }

    *value = parsed;
    return true;
}

static bool parse_joint_arg(const char *text, exo_hip_joint_id_t *joint)
{
    if (text == NULL || joint == NULL) {
        return false;
    }
    if (strcasecmp(text, "left") == 0 || strcasecmp(text, "l") == 0) {
        *joint = EXO_HIP_JOINT_LEFT;
        return true;
    }
    if (strcasecmp(text, "right") == 0 || strcasecmp(text, "r") == 0) {
        *joint = EXO_HIP_JOINT_RIGHT;
        return true;
    }
    return false;
}

static bool parse_float_args(char *argv[], int first, int count, float values[])
{
    for (int i = 0; i < count; ++i) {
        if (!parse_float_arg(argv[first + i], &values[i])) {
            return false;
        }
    }
    return true;
}

static exo_can_node_id_t selected_can_id(void)
{
    exo_can_node_id_t can_id = 0;
    (void)rs01_get_can_id(s_selected_actuator, &can_id);
    return can_id;
}

static bool driver_command_succeeded(esp_err_t err)
{
    if (err == ESP_OK) {
        return true;
    }

    ble_vprintf("ERROR: RS01 command failed: %s\r\n", esp_err_to_name(err));
    return false;
}

/* ================================================================
 *  Command handlers (identical to console.c, printf -> ble_vprintf)
 * ================================================================ */

static void cmd_motor(int argc, char *argv[])
{
    exo_actuator_id_t actuator = EXO_ACTUATOR_COUNT;
    if (strcmp(argv[1], "A") == 0 || strcmp(argv[1], "a") == 0) {
        actuator = EXO_ACTUATOR_A;
    } else if (strcmp(argv[1], "B") == 0 || strcmp(argv[1], "b") == 0) {
        actuator = EXO_ACTUATOR_B;
    } else {
        long id;
        if (!parse_long_arg(argv[1], 0, 0x7F, &id) ||
            !rs01_find_actuator((exo_can_node_id_t)id, &actuator)) {
            ble_vprintf("ERROR: motor must be A, B, or a configured CAN ID\r\n");
            return;
        }
    }

    s_selected_actuator = actuator;
    exo_can_node_id_t can_id = selected_can_id();
    ble_vprintf("OK: target motor set to 0x%02X (%d), actuator %c\r\n",
                can_id, can_id,
                actuator == EXO_ACTUATOR_A ? 'A' : 'B');
}

static void cmd_enable(int argc, char *argv[])
{
    if (driver_command_succeeded(rs01_enable(s_selected_actuator))) {
        ble_vprintf("OK: motor 0x%02X enable sent\r\n", selected_can_id());
    }
}

static void cmd_disable(int argc, char *argv[])
{
    if (driver_command_succeeded(rs01_disable(s_selected_actuator))) {
        ble_vprintf("OK: motor 0x%02X disable sent\r\n", selected_can_id());
    }
}

static void cmd_move(int argc, char *argv[])
{
    float values[5];
    if (!parse_float_args(argv, 1, 5, values)) {
        ble_vprintf("ERROR: move arguments must be finite numbers\r\n");
        return;
    }
    float t = values[0], pos = values[1], spd = values[2];
    float kp = values[3], kd = values[4];
    if (driver_command_succeeded(rs01_move(
            s_selected_actuator, t, pos, spd, kp, kd))) {
        ble_vprintf("OK: motor 0x%02X move t=%.2f pos=%.2f spd=%.2f kp=%.2f kd=%.2f\r\n",
                    selected_can_id(), (double)t, (double)pos, (double)spd,
                    (double)kp, (double)kd);
    }
}

static void cmd_speed(int argc, char *argv[])
{
    float values[3];
    if (!parse_float_args(argv, 1, 3, values)) {
        ble_vprintf("ERROR: speed arguments must be finite numbers\r\n");
        return;
    }
    float cur = values[0], acc = values[1], spd = values[2];
    if (driver_command_succeeded(rs01_speed_control(
            s_selected_actuator, cur, acc, spd))) {
        ble_vprintf("OK: motor 0x%02X speed cur=%.2f acc=%.2f spd=%.2f\r\n",
                    selected_can_id(), (double)cur, (double)acc, (double)spd);
    }
}

static void cmd_pos_pp(int argc, char *argv[])
{
    float values[3];
    if (!parse_float_args(argv, 1, 3, values)) {
        ble_vprintf("ERROR: pos_pp arguments must be finite numbers\r\n");
        return;
    }
    float spd = values[0], acc = values[1], ang = values[2];
    if (driver_command_succeeded(rs01_position_pp_control(
            s_selected_actuator, spd, acc, ang))) {
        ble_vprintf("OK: motor 0x%02X PP pos spd=%.2f acc=%.2f ang=%.2f\r\n",
                    selected_can_id(), (double)spd, (double)acc, (double)ang);
    }
}

static void cmd_pos_csp(int argc, char *argv[])
{
    float values[2];
    if (!parse_float_args(argv, 1, 2, values)) {
        ble_vprintf("ERROR: pos_csp arguments must be finite numbers\r\n");
        return;
    }
    float spd = values[0], ang = values[1];
    if (driver_command_succeeded(rs01_position_csp_control(
            s_selected_actuator, spd, ang))) {
        ble_vprintf("OK: motor 0x%02X CSP pos spd=%.2f ang=%.2f\r\n",
                    selected_can_id(), (double)spd, (double)ang);
    }
}

static void cmd_current(int argc, char *argv[])
{
    float cur;
    if (!parse_float_arg(argv[1], &cur)) {
        ble_vprintf("ERROR: current must be a finite number\r\n");
        return;
    }
    if (driver_command_succeeded(rs01_current_control(
            s_selected_actuator, cur))) {
        ble_vprintf("OK: motor 0x%02X current cur=%.2f\r\n",
                    selected_can_id(), (double)cur);
    }
}

static void cmd_set_zero(int argc, char *argv[])
{
    if (driver_command_succeeded(rs01_set_mechanical_zero(
            s_selected_actuator))) {
        ble_vprintf("OK: motor 0x%02X set zero position sent\r\n",
                    selected_can_id());
    }
}

static void cmd_set_id(int argc, char *argv[])
{
    long parsed;
    if (!parse_long_arg(argv[1], 0, 0x7F, &parsed)) {
        ble_vprintf("ERROR: new motor ID must be 0-127\r\n");
        return;
    }
    uint8_t new_id = (uint8_t)parsed;
    exo_can_node_id_t old_id = selected_can_id();
    if (driver_command_succeeded(rs01_set_can_id(
            s_selected_actuator, new_id))) {
        ble_vprintf("OK: motor 0x%02X change ID to %d\r\n", old_id, new_id);
    }
}

static void cmd_save(int argc, char *argv[])
{
    if (driver_command_succeeded(rs01_save_parameters(s_selected_actuator))) {
        ble_vprintf("OK: motor 0x%02X save to flash sent\r\n",
                    selected_can_id());
    }
}

static void cmd_set_baud(int argc, char *argv[])
{
    long parsed;
    if (!parse_long_arg(argv[1], 1, 4, &parsed)) {
        ble_vprintf("ERROR: baud rate selector must be 1-4\r\n");
        return;
    }
    uint8_t rate = (uint8_t)parsed;
    if (driver_command_succeeded(rs01_set_baud_rate(
            s_selected_actuator, rate))) {
        ble_vprintf("OK: motor 0x%02X baud rate change sent (rate=%d)\r\n",
                    selected_can_id(), rate);
    }
}

static void cmd_report(int argc, char *argv[])
{
    long parsed;
    if (!parse_long_arg(argv[1], 0, 1, &parsed)) {
        ble_vprintf("ERROR: report must be 0 or 1\r\n");
        return;
    }
    uint8_t on = (uint8_t)parsed;
    esp_err_t err = on ? rs01_start_reporting(s_selected_actuator)
                       : rs01_stop_reporting(s_selected_actuator);
    if (driver_command_succeeded(err)) {
        ble_vprintf("OK: motor 0x%02X proactive report %s\r\n",
                    selected_can_id(), on ? "ON" : "OFF");
    }
}

static void cmd_switch_mit(int argc, char *argv[])
{
    if (driver_command_succeeded(rs01_set_protocol(
            s_selected_actuator, RS01_PROTOCOL_MIT))) {
        ble_vprintf("OK: motor 0x%02X MIT protocol selected; power-cycle motor to apply\r\n",
                    selected_can_id());
    }
}

static void cmd_switch_private(int argc, char *argv[])
{
    if (driver_command_succeeded(rs01_mit_set_protocol(
            s_selected_actuator, RS01_PROTOCOL_PRIVATE))) {
        ble_vprintf("OK: motor 0x%02X private protocol selected; power-cycle motor to apply\r\n",
                    selected_can_id());
    }
}

static void cmd_read_param(int argc, char *argv[])
{
    long parsed;
    if (!parse_long_arg(argv[1], 0, UINT16_MAX, &parsed)) {
        ble_vprintf("ERROR: register address must be 0-0xFFFF\r\n");
        return;
    }
    uint16_t reg = (uint16_t)parsed;
    if (driver_command_succeeded(rs01_read_parameter(
            s_selected_actuator, reg))) {
        ble_vprintf("OK: motor 0x%02X read param 0x%04X sent\r\n",
                    selected_can_id(), reg);
    }
}

static void cmd_mit_enable(int argc, char *argv[])
{
    if (driver_command_succeeded(rs01_mit_enable(s_selected_actuator))) {
        ble_vprintf("OK: motor 0x%02X MIT enable sent\r\n", selected_can_id());
    }
}

static void cmd_mit_disable(int argc, char *argv[])
{
    if (driver_command_succeeded(rs01_mit_disable(s_selected_actuator))) {
        ble_vprintf("OK: motor 0x%02X MIT disable sent\r\n", selected_can_id());
    }
}

static void cmd_mit_control(int argc, char *argv[])
{
    float values[5];
    if (!parse_float_args(argv, 1, 5, values)) {
        ble_vprintf("ERROR: mit_control arguments must be finite numbers\r\n");
        return;
    }
    float ang = values[0], spd = values[1], kp = values[2];
    float kd = values[3], t = values[4];
    if (driver_command_succeeded(rs01_mit_send_motion(
            s_selected_actuator, ang, spd, kp, kd, t))) {
        ble_vprintf("OK: motor 0x%02X MIT control ang=%.2f spd=%.2f kp=%.2f kd=%.2f t=%.2f\r\n",
                    selected_can_id(), (double)ang, (double)spd,
                    (double)kp, (double)kd, (double)t);
    }
}

static void cmd_mit_pos(int argc, char *argv[])
{
    float values[2];
    if (!parse_float_args(argv, 1, 2, values)) {
        ble_vprintf("ERROR: mit_pos arguments must be finite numbers\r\n");
        return;
    }
    float pos = values[0], spd = values[1];
    if (driver_command_succeeded(rs01_mit_position_control(
            s_selected_actuator, pos, spd))) {
        ble_vprintf("OK: motor 0x%02X MIT position pos=%.2f spd=%.2f\r\n",
                    selected_can_id(), (double)pos, (double)spd);
    }
}

static void cmd_mit_speed(int argc, char *argv[])
{
    float values[2];
    if (!parse_float_args(argv, 1, 2, values)) {
        ble_vprintf("ERROR: mit_speed arguments must be finite numbers\r\n");
        return;
    }
    float spd = values[0], cur = values[1];
    if (driver_command_succeeded(rs01_mit_speed_control(
            s_selected_actuator, spd, cur))) {
        ble_vprintf("OK: motor 0x%02X MIT speed spd=%.2f cur=%.2f\r\n",
                    selected_can_id(), (double)spd, (double)cur);
    }
}

static void cmd_mit_set_zero(int argc, char *argv[])
{
    if (driver_command_succeeded(rs01_mit_set_zero(s_selected_actuator))) {
        ble_vprintf("OK: motor 0x%02X MIT set zero sent\r\n", selected_can_id());
    }
}

static void cmd_mit_set_type(int argc, char *argv[])
{
    long parsed;
    if (!parse_long_arg(argv[1], 0, 2, &parsed)) {
        ble_vprintf("ERROR: MIT motor type must be 0-2\r\n");
        return;
    }
    uint8_t type = (uint8_t)parsed;
    if (driver_command_succeeded(rs01_mit_set_operation_mode(
            s_selected_actuator, type))) {
        ble_vprintf("OK: motor 0x%02X MIT set motor type to %d\r\n",
                    selected_can_id(), type);
    }
}

static void cmd_mit_set_id(int argc, char *argv[])
{
    long parsed;
    if (!parse_long_arg(argv[1], 0, 0x7F, &parsed)) {
        ble_vprintf("ERROR: new MIT motor ID must be 0-127\r\n");
        return;
    }
    uint8_t new_id = (uint8_t)parsed;
    if (driver_command_succeeded(rs01_mit_set_can_id(
            s_selected_actuator, new_id))) {
        ble_vprintf("OK: motor 0x%02X MIT set motor ID to %d\r\n",
                    selected_can_id(), new_id);
    }
}

static void cmd_mit_save(int argc, char *argv[])
{
    if (driver_command_succeeded(rs01_mit_save_parameters(
            s_selected_actuator))) {
        ble_vprintf("OK: motor 0x%02X MIT save sent\r\n", selected_can_id());
    }
}

static void cmd_mit_report(int argc, char *argv[])
{
    long parsed;
    if (!parse_long_arg(argv[1], 0, 1, &parsed)) {
        ble_vprintf("ERROR: mit_report must be 0 or 1\r\n");
        return;
    }
    uint8_t on = (uint8_t)parsed;
    esp_err_t err = on ? rs01_mit_start_reporting(s_selected_actuator)
                       : rs01_mit_stop_reporting(s_selected_actuator);
    if (driver_command_succeeded(err)) {
        ble_vprintf("OK: motor 0x%02X MIT proactive report %s\r\n",
                    selected_can_id(), on ? "ON" : "OFF");
    }
}

static void cmd_mit_read(int argc, char *argv[])
{
    long parsed;
    if (!parse_long_arg(argv[1], 0, UINT16_MAX, &parsed)) {
        ble_vprintf("ERROR: MIT register address must be 0-0xFFFF\r\n");
        return;
    }
    if (driver_command_succeeded(rs01_mit_read_parameter(
            s_selected_actuator, (uint16_t)parsed))) {
        ble_vprintf("OK: motor 0x%02X MIT read param 0x%04lX sent\r\n",
                    selected_can_id(), parsed);
    }
}

static void cmd_mit_write(int argc, char *argv[])
{
    long parsed;
    float value;
    if (!parse_long_arg(argv[1], 0, UINT16_MAX, &parsed) ||
        !parse_float_arg(argv[2], &value)) {
        ble_vprintf("ERROR: MIT write requires <reg_addr> <finite_value>\r\n");
        return;
    }
    if (driver_command_succeeded(rs01_mit_write_parameter_float(
            s_selected_actuator, (uint16_t)parsed, value))) {
        ble_vprintf("OK: motor 0x%02X MIT write param 0x%04lX sent\r\n",
                    selected_can_id(), parsed);
    }
}

static void cmd_mit_error(int argc, char *argv[])
{
    long parsed;
    if (!parse_long_arg(argv[1], 0, UINT8_MAX, &parsed)) {
        ble_vprintf("ERROR: MIT error command must be 0-255 (0xFF clears)\r\n");
        return;
    }
    if (driver_command_succeeded(rs01_mit_clear_or_read_fault(
            s_selected_actuator, (uint8_t)parsed))) {
        ble_vprintf("OK: motor 0x%02X MIT error command 0x%02lX sent\r\n",
                    selected_can_id(), parsed);
    }
}

static void cmd_motor_type(int argc, char *argv[])
{
    const rs01_limits_t *limits = rs01_get_limits();
    ble_vprintf("Motor driver: RS01\r\n");
    ble_vprintf("  P:  [%.2f, %.2f] rad\r\n",
                (double)limits->position_min_rad,
                (double)limits->position_max_rad);
    ble_vprintf("  V:  [%.2f, %.2f] rad/s\r\n",
                (double)limits->velocity_min_rad_s,
                (double)limits->velocity_max_rad_s);
    ble_vprintf("  KP: [%.2f, %.2f]\r\n",
                (double)limits->kp_min, (double)limits->kp_max);
    ble_vprintf("  KD: [%.2f, %.2f]\r\n",
                (double)limits->kd_min, (double)limits->kd_max);
    ble_vprintf("  T:  [%.2f, %.2f] Nm\r\n",
                (double)limits->torque_min_nm,
                (double)limits->torque_max_nm);
}

static void cmd_status(int argc, char *argv[])
{
    rs01_feedback_snapshot_t snapshot;
    if (!rs01_get_feedback(s_selected_actuator, &snapshot)) {
        ble_vprintf("ERROR: cannot read RS01 feedback snapshot\r\n");
        return;
    }

    exo_can_node_id_t can_id = selected_can_id();
    uint64_t age_us = snapshot.valid
        ? (uint64_t)esp_timer_get_time() - snapshot.timestamp_us
        : 0U;
    double rate_hz = snapshot.last_interval_us > 0U
        ? 1000000.0 / (double)snapshot.last_interval_us
        : 0.0;

    ble_vprintf("===== Motor 0x%02X Feedback =====\r\n", can_id);
    ble_vprintf("  Actuator:    %c\r\n",
                s_selected_actuator == EXO_ACTUATOR_A ? 'A' : 'B');
    ble_vprintf("  Valid:       %s\r\n", snapshot.valid ? "YES" : "no");
    ble_vprintf("  Online:      %s\r\n", snapshot.online ? "YES" : "no");
    ble_vprintf("  Reporting:   %s (requested)\r\n",
                snapshot.reporting_requested ? "ON" : "OFF");
    ble_vprintf("  Enabled:     %s (requested)\r\n",
                snapshot.enabled_requested ? "YES" : "no");
    ble_vprintf("  Feedback count: %lu\r\n",
                (unsigned long)snapshot.feedback_count);
    ble_vprintf("    Type 2:        %lu\r\n",
                (unsigned long)snapshot.type2_feedback_count);
    ble_vprintf("    Type 24:       %lu\r\n",
                (unsigned long)snapshot.type24_feedback_count);
    ble_vprintf("  Invalid frames: %lu\r\n",
                (unsigned long)snapshot.invalid_frame_count);
    ble_vprintf("  Last interval: %llu us (%.2f Hz)\r\n",
                (unsigned long long)snapshot.last_interval_us, rate_hz);
    ble_vprintf("  Feedback age: %llu us\r\n",
                (unsigned long long)age_us);
    ble_vprintf("  Angle:       %.3f rad\r\n",
                (double)snapshot.sample.position_rad);
    ble_vprintf("  Speed:       %.3f rad/s\r\n",
                (double)snapshot.sample.velocity_rad_s);
    ble_vprintf("  Torque:      %.3f Nm\r\n",
                (double)snapshot.sample.torque_nm);
    ble_vprintf("  Temperature: %.1f C\r\n",
                (double)snapshot.sample.temperature_c);
    ble_vprintf("  Mode status: %d ", (int)snapshot.mode);
    switch (snapshot.mode) {
        case RS01_MODE_STATUS_RESET:       ble_vprintf("(REST)\r\n"); break;
        case RS01_MODE_STATUS_CALIBRATION: ble_vprintf("(CALI)\r\n"); break;
        case RS01_MODE_STATUS_RUNNING:     ble_vprintf("(RUNNING)\r\n"); break;
        default:      ble_vprintf("(UNKNOWN)\r\n"); break;
    }
    ble_vprintf("  Motor mode:  %d\r\n", (int)snapshot.control_mode);
    ble_vprintf("  Fault raw:   0x%08lX\r\n",
                (unsigned long)snapshot.fault_raw);
    ble_vprintf("  Warning raw: 0x%08lX\r\n",
                (unsigned long)snapshot.warning_raw);
    ble_vprintf("  Faults:\r\n");
    ble_vprintf("    Uncalibrated:         %s\r\n", snapshot.faults.uncalibrated ? "YES" : "no");
    ble_vprintf("    Stall/Overload:       %s\r\n", snapshot.faults.stall_overload ? "YES" : "no");
    ble_vprintf("    Magnetic Encoder:     %s\r\n", snapshot.faults.magnetic_encoder ? "YES" : "no");
    ble_vprintf("    Over Temperature:     %s\r\n", snapshot.faults.over_temperature ? "YES" : "no");
    ble_vprintf("    Drive/Phase Current:  %s\r\n",
                (snapshot.faults.driver_chip ||
                 snapshot.faults.phase_over_current) ? "YES" : "no");
    ble_vprintf("    Under Voltage:        %s\r\n", snapshot.faults.under_voltage ? "YES" : "no");
}

static const char *joint_name(exo_hip_joint_id_t joint)
{
    return joint == EXO_HIP_JOINT_LEFT ? "LEFT" : "RIGHT";
}

static const char *joint_direction_name(exo_direction_t direction)
{
    if (direction == EXO_DIRECTION_POSITIVE) {
        return "+1";
    }
    if (direction == EXO_DIRECTION_NEGATIVE) {
        return "-1";
    }
    return "UNKNOWN";
}

static void print_joint_status(exo_hip_joint_id_t joint)
{
    hip_joint_snapshot_t snapshot;
    if (!hip_joint_get_snapshot(joint, &snapshot)) {
        ble_vprintf("ERROR: cannot read hip joint snapshot\r\n");
        return;
    }

    uint64_t age_us = 0U;
    const uint64_t now_us = (uint64_t)esp_timer_get_time();
    if (snapshot.feedback_valid && now_us >= snapshot.feedback_timestamp_us) {
        age_us = now_us - snapshot.feedback_timestamp_us;
    }
    const bool raw_available = snapshot.mapping_valid &&
        snapshot.feedback_valid &&
        isfinite(snapshot.raw_actuator_sample.position_rad) &&
        isfinite(snapshot.raw_actuator_sample.velocity_rad_s) &&
        isfinite(snapshot.raw_actuator_sample.torque_nm) &&
        isfinite(snapshot.raw_actuator_sample.temperature_c);
    const bool joint_sample_available = raw_available &&
        snapshot.direction_valid && snapshot.zero_valid;

    ble_vprintf("===== Hip Joint %s =====\r\n", joint_name(joint));
    if (snapshot.mapping_valid) {
        ble_vprintf("  Mapping:       actuator %c (valid)\r\n",
                    snapshot.actuator == EXO_ACTUATOR_A ? 'A' : 'B');
    } else {
        ble_vprintf("  Mapping:       UNMAPPED\r\n");
    }
    ble_vprintf("  Direction:     %s (%s)\r\n",
                joint_direction_name(snapshot.direction),
                snapshot.direction_valid ? "valid" : "INVALID");
    if (snapshot.zero_valid) {
        ble_vprintf("  Software zero: %.3f rad (valid, RAM only)\r\n",
                    (double)snapshot.software_zero_motor_rad);
        ble_vprintf("  Zero source:   feedback count=%lu timestamp=%llu us\r\n",
                    (unsigned long)snapshot.zero_source_feedback_count,
                    (unsigned long long)snapshot.zero_source_timestamp_us);
    } else {
        ble_vprintf("  Software zero: N/A (INVALID, RAM only)\r\n");
        ble_vprintf("  Zero source:   none\r\n");
    }
    if (!snapshot.mapping_valid) {
        ble_vprintf("  Feedback:      N/A (joint unmapped)\r\n");
    } else if (!snapshot.feedback_valid) {
        ble_vprintf("  Feedback:      INVALID / OFFLINE, count=%lu, age=N/A\r\n",
                    (unsigned long)snapshot.source_feedback_count);
    } else {
        ble_vprintf("  Feedback:      valid / %s, count=%lu, age=%llu us\r\n",
                    snapshot.online ? "online" : "OFFLINE",
                    (unsigned long)snapshot.source_feedback_count,
                    (unsigned long long)age_us);
    }
    if (raw_available) {
        ble_vprintf("  Raw actuator:  q=%.3f rad dq=%.3f rad/s tau=%.3f Nm temp=%.1f C\r\n",
                    (double)snapshot.raw_actuator_sample.position_rad,
                    (double)snapshot.raw_actuator_sample.velocity_rad_s,
                    (double)snapshot.raw_actuator_sample.torque_nm,
                    (double)snapshot.raw_actuator_sample.temperature_c);
    } else {
        ble_vprintf("  Raw actuator:  N/A\r\n");
    }
    if (joint_sample_available) {
        ble_vprintf("  Joint:         q=%.3f rad dq=%.3f rad/s tau=%.3f Nm\r\n",
                    (double)snapshot.joint_sample.position_rad,
                    (double)snapshot.joint_sample.velocity_rad_s,
                    (double)snapshot.joint_sample.torque_nm);
    } else {
        ble_vprintf("  Joint:         N/A\r\n");
    }
    ble_vprintf("  Joint valid:   %s\r\n",
                snapshot.joint_valid ? "YES" : "no");
}

static bool command_joint_from_arg(char *argv[], exo_hip_joint_id_t *joint)
{
    if (parse_joint_arg(argv[1], joint)) {
        return true;
    }
    ble_vprintf("ERROR: hip joint must be left or right\r\n");
    return false;
}

static void cmd_joint_status(int argc, char *argv[])
{
    exo_hip_joint_id_t joint;
    if (command_joint_from_arg(argv, &joint)) {
        print_joint_status(joint);
    }
}

static void cmd_joint_zero(int argc, char *argv[])
{
    exo_hip_joint_id_t joint;
    if (!command_joint_from_arg(argv, &joint)) {
        return;
    }

    esp_err_t err = hip_joint_set_zero_from_current(joint);
    if (err != ESP_OK) {
        ble_vprintf("ERROR: %s hip software zero not captured: %s\r\n",
                    joint_name(joint), esp_err_to_name(err));
        return;
    }
    ble_vprintf("OK: %s hip RAM software zero captured; motor zero unchanged\r\n",
                joint_name(joint));
    print_joint_status(joint);
}

static void cmd_joint_zero_clear(int argc, char *argv[])
{
    exo_hip_joint_id_t joint;
    if (!command_joint_from_arg(argv, &joint)) {
        return;
    }

    esp_err_t err = hip_joint_clear_zero(joint);
    if (err != ESP_OK) {
        ble_vprintf("ERROR: %s hip software zero not cleared: %s\r\n",
                    joint_name(joint), esp_err_to_name(err));
        return;
    }
    ble_vprintf("OK: %s hip RAM software zero cleared; motor zero unchanged\r\n",
                joint_name(joint));
}

static void cmd_control_status(int argc, char *argv[])
{
    exo_control_loop_snapshot_t snapshot;
    if (!exo_control_loop_get_snapshot(&snapshot)) {
        ble_vprintf("ERROR: control loop is not initialized\r\n");
        return;
    }

    const double measured_hz = snapshot.mean_period_us > 0U
        ? 1000000.0 / (double)snapshot.mean_period_us
        : 0.0;

    ble_vprintf("===== 100 Hz Control Loop =====\r\n");
    ble_vprintf("  Initialized:       %s\r\n",
                snapshot.initialized ? "YES" : "no");
    ble_vprintf("  Task alive:        %s\r\n",
                snapshot.task_alive ? "YES" : "no");
    ble_vprintf("  Task identity:     %p\r\n",
                (void *)snapshot.task_handle_identity);
    ble_vprintf("  Configured rate:   %lu Hz (%lu us)\r\n",
                (unsigned long)snapshot.configured_frequency_hz,
                (unsigned long)snapshot.target_period_us);
    ble_vprintf("  Cycle count:       %llu\r\n",
                (unsigned long long)snapshot.cycle_count);
    ble_vprintf("  Period samples:    %llu; outside +/- %lu us: %llu\r\n",
                (unsigned long long)snapshot.period_sample_count,
                (unsigned long)snapshot.period_tolerance_us,
                (unsigned long long)snapshot.period_out_of_tolerance_count);
    ble_vprintf("  Last cycle:        %llu us\r\n",
                (unsigned long long)snapshot.last_cycle_start_us);
    ble_vprintf("  Period us:         last=%lu min=%lu max=%lu mean=%lu (%.3f Hz)\r\n",
                (unsigned long)snapshot.last_period_us,
                (unsigned long)snapshot.min_period_us,
                (unsigned long)snapshot.max_period_us,
                (unsigned long)snapshot.mean_period_us,
                measured_hz);
    ble_vprintf("  Period error us:   last=%ld max_abs=%lu\r\n",
                (long)snapshot.last_period_error_us,
                (unsigned long)snapshot.max_abs_period_error_us);
    ble_vprintf("  Execution us:      last=%lu max=%lu\r\n",
                (unsigned long)snapshot.last_execution_us,
                (unsigned long)snapshot.max_execution_us);
    ble_vprintf("  Deadline misses:   %llu\r\n",
                (unsigned long long)snapshot.deadline_miss_count);
    ble_vprintf("  Skipped cycles:    %llu\r\n",
                (unsigned long long)snapshot.skipped_cycle_count);
    ble_vprintf("  Output errors:     %llu (last=%s)\r\n",
                (unsigned long long)snapshot.output_error_count,
                esp_err_to_name(snapshot.last_output_error));
    ble_vprintf("  Stack free min:    %lu bytes\r\n",
                (unsigned long)snapshot.stack_high_water_mark_bytes);
    ble_vprintf("  LEFT input:        valid=%s count=%lu timestamp=%llu us\r\n",
                snapshot.left.joint_valid ? "YES" : "no",
                (unsigned long)snapshot.left.feedback_count,
                (unsigned long long)snapshot.left.feedback_timestamp_us);
    ble_vprintf("  RIGHT input:       valid=%s count=%lu timestamp=%llu us\r\n",
                snapshot.right.joint_valid ? "YES" : "no",
                (unsigned long)snapshot.right.feedback_count,
                (unsigned long long)snapshot.right.feedback_timestamp_us);
    ble_vprintf("  Legacy impedance: %s\r\n",
                impedance_control_is_running() ? "RUNNING" : "inactive");
}

static void cmd_can_status(int argc, char *argv[])
{
    twai_status_info_t status;
    esp_err_t err = twai_get_status_info(&status);
    if (err != ESP_OK) {
        ble_vprintf("ERROR: cannot read TWAI status: %s\r\n",
                    esp_err_to_name(err));
        return;
    }

    const char *state = "UNKNOWN";
    switch (status.state) {
        case TWAI_STATE_STOPPED:    state = "STOPPED"; break;
        case TWAI_STATE_RUNNING:    state = "RUNNING"; break;
        case TWAI_STATE_BUS_OFF:    state = "BUS_OFF"; break;
        case TWAI_STATE_RECOVERING: state = "RECOVERING"; break;
        default: break;
    }

    ble_vprintf("===== TWAI Status =====\r\n");
    ble_vprintf("  State:             %s\r\n", state);
    ble_vprintf("  RX GPIO%d level:   %d (%s)\r\n",
                g_exo_hardware_config.twai.rx_gpio,
                gpio_get_level(g_exo_hardware_config.twai.rx_gpio),
                gpio_get_level(g_exo_hardware_config.twai.rx_gpio)
                    ? "recessive/idle"
                    : "dominant/busy");
    ble_vprintf("  TX queued:         %lu\r\n",
                (unsigned long)status.msgs_to_tx);
    ble_vprintf("  RX queued:         %lu\r\n",
                (unsigned long)status.msgs_to_rx);
    ble_vprintf("  TX error counter:  %lu\r\n",
                (unsigned long)status.tx_error_counter);
    ble_vprintf("  RX error counter:  %lu\r\n",
                (unsigned long)status.rx_error_counter);
    ble_vprintf("  TX failed count:   %lu\r\n",
                (unsigned long)status.tx_failed_count);
    ble_vprintf("  Bus error count:   %lu\r\n",
                (unsigned long)status.bus_error_count);
    ble_vprintf("  Arbitration lost:  %lu\r\n",
                (unsigned long)status.arb_lost_count);
    ble_vprintf("  RX missed:         %lu\r\n",
                (unsigned long)status.rx_missed_count);
}

static void cmd_impedance_start(int argc, char *argv[])
{
    exo_control_loop_snapshot_t control_snapshot;
    if (!exo_control_loop_get_snapshot(&control_snapshot) ||
        !control_snapshot.task_alive) {
        ble_vprintf("ERROR: control loop is not ready; try again\r\n");
        return;
    }
    if (impedance_control_is_running()) {
        ble_vprintf("ERROR: impedance control already running. Use imp_stop first.\r\n");
        return;
    }
    esp_err_t err = impedance_control_start();
    if (err == ESP_OK) {
        ble_vprintf("OK: dual-motor impedance control started\r\n");
    } else {
        ble_vprintf("ERROR: failed to start impedance control: %s\r\n",
                    esp_err_to_name(err));
    }
}

static void cmd_impedance_stop(int argc, char *argv[])
{
    if (!impedance_control_is_running()) {
        ble_vprintf("ERROR: impedance control not running.\r\n");
        return;
    }
    esp_err_t err = impedance_control_stop();
    if (err == ESP_OK) {
        ble_vprintf("OK: impedance control stopped, motors disabled\r\n");
    } else {
        ble_vprintf("ERROR: failed to stop impedance control: %s\r\n",
                    esp_err_to_name(err));
    }
}

static void cmd_help(int argc, char *argv[])
{
    ble_vprintf("\r\nRobStride Motor Control Console\r\n");
    exo_can_node_id_t can_id = selected_can_id();
    ble_vprintf("Current motor: 0x%02X (%d), actuator %c\r\n",
                can_id, can_id,
                s_selected_actuator == EXO_ACTUATOR_A ? 'A' : 'B');
    ble_vprintf("================================\r\n");
    for (const console_cmd_t *c = s_cmd_table; c->name != NULL; c++) {
        ble_vprintf("  %-14s %s\r\n", c->name, c->help);
    }
    ble_vprintf("\r\n");
}

/* ================================================================
 *  Command parser (same logic as console.c)
 * ================================================================ */
#define CMD_MAX_ARGS 8

static void console_process_line(char *line)
{
    if (line == NULL || s_cmd_mutex == NULL) {
        return;
    }

    xSemaphoreTake(s_cmd_mutex, portMAX_DELAY);

    size_t len = strlen(line);
    while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r')) {
        line[--len] = '\0';
    }
    if (len == 0) {
        xSemaphoreGive(s_cmd_mutex);
        return;
    }

    int argc = 0;
    char *argv[CMD_MAX_ARGS];
    char *saveptr = NULL;
    char *token = strtok_r(line, " \t", &saveptr);
    while (token != NULL && argc < CMD_MAX_ARGS) {
        argv[argc++] = token;
        token = strtok_r(NULL, " \t", &saveptr);
    }
    if (argc == 0) {
        xSemaphoreGive(s_cmd_mutex);
        return;
    }
    if (token != NULL) {
        ble_vprintf("ERROR: too many command arguments (max %d)\r\n",
                    CMD_MAX_ARGS - 1);
        xSemaphoreGive(s_cmd_mutex);
        return;
    }

    for (const console_cmd_t *c = s_cmd_table; c->name != NULL; c++) {
        if (strcmp(argv[0], c->name) == 0) {
            if (argc - 1 < c->min_args) {
                ble_vprintf("Usage: %s (needs %d args, got %d)\r\n",
                       c->help, c->min_args, argc - 1);
                xSemaphoreGive(s_cmd_mutex);
                return;
            }
            c->handler(argc, argv);
            xSemaphoreGive(s_cmd_mutex);
            return;
        }
    }
    ble_vprintf("Unknown command: '%s'. Type 'help' for list.\r\n", argv[0]);
    xSemaphoreGive(s_cmd_mutex);
}

/* ================================================================
 *  BLE Console task — reads from RX stream buffer, parses commands
 * ================================================================ */
static void ble_console_task(void *arg)
{
    char line[LINE_BUF_SIZE];
    size_t pos = 0;
    uint8_t ch;
    bool line_overflow = false;

    while (1) {
        portENTER_CRITICAL(&s_lock);
        bool send_welcome = s_welcome_pending;
        s_welcome_pending = false;
        portEXIT_CRITICAL(&s_lock);
        if (send_welcome) {
            ble_vprintf("\r\nRobStride Motor Control - BLE Console\r\n");
            ble_vprintf("Connected! Type 'help' for command list.\r\n");
        }

        /* Read one byte at a time from the stream buffer */
        size_t received = xStreamBufferReceive(s_rx_streambuf, &ch, 1,
                                               pdMS_TO_TICKS(20));
        if (received == 1) {
            if (ch == '\n' || ch == '\r') {
                if (line_overflow) {
                    ble_vprintf("ERROR: command line exceeds %d bytes\r\n",
                                LINE_BUF_SIZE - 1);
                    line_overflow = false;
                    pos = 0;
                } else if (pos > 0) {
                    line[pos] = '\0';
                    console_process_line(line);
                    pos = 0;
                }
            } else if (pos < sizeof(line) - 1) {
                line[pos++] = ch;
            } else {
                line_overflow = true;
            }
        }
    }
}

/* ================================================================
 *  Programming UART console task
 *  The ESP-IDF primary/secondary console VFS supplies stdin, so this works
 *  through the same COM port used for flashing and monitoring.
 * ================================================================ */
static void uart_console_task(void *arg)
{
    char line[LINE_BUF_SIZE];
    size_t pos = 0;
    bool line_overflow = false;

    ble_vprintf("\r\nRobStride Motor Control - Programming Port Console\r\n");
    ble_vprintf("Type 'help' and press Enter for the command list.\r\n");

    while (1) {
        int input = fgetc(stdin);
        if (input == EOF) {
            clearerr(stdin);
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        uint8_t ch = (uint8_t)input;
        if (ch == '\n' || ch == '\r') {
            if (line_overflow) {
                ble_vprintf("ERROR: command line exceeds %d bytes\r\n",
                            LINE_BUF_SIZE - 1);
                line_overflow = false;
                pos = 0;
            } else if (pos > 0) {
                line[pos] = '\0';
                console_process_line(line);
                pos = 0;
            }
        } else if (pos < sizeof(line) - 1) {
            line[pos++] = (char)ch;
        } else {
            line_overflow = true;
        }
    }
}

/* ================================================================
 *  Public API — initialize BLE console
 *  Host sync → GATT registration → advertising happens in ble_on_sync()
 * ================================================================ */
esp_err_t ble_console_init(void)
{
    s_selected_actuator = g_exo_hardware_config.default_actuator;

    /* Create stream buffer for RX data from BLE */
    s_rx_streambuf = xStreamBufferCreate(1024, 1);
    if (s_rx_streambuf == NULL) {
        ESP_LOGE(TAG, "Failed to create RX stream buffer");
        return ESP_ERR_NO_MEM;
    }

    s_tx_mutex = xSemaphoreCreateMutex();
    if (s_tx_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create TX mutex");
        return ESP_ERR_NO_MEM;
    }

    s_cmd_mutex = xSemaphoreCreateMutex();
    if (s_cmd_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create command mutex");
        return ESP_ERR_NO_MEM;
    }

    /* Initialize NVS (required by NimBLE) */
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES ||
        nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_err = nvs_flash_erase();
        if (nvs_err == ESP_OK) {
            nvs_err = nvs_flash_init();
        }
    }
    if (nvs_err != ESP_OK) {
        ESP_LOGE(TAG, "NVS init failed: %s", esp_err_to_name(nvs_err));
        return nvs_err;
    }

    /* Initialize NimBLE (controller + HCI + port layer) */
    esp_err_t err = nimble_port_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NimBLE init failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Configure the host after nimble_port_init(), as required by IDF 5.3. */
    ble_hs_cfg.reset_cb = ble_on_reset;
    ble_hs_cfg.sync_cb = ble_on_sync;

    /* Register GATT services BEFORE starting host task */
    int gatt_rc = ble_gatt_svr_init();
    if (gatt_rc != 0) {
        ESP_LOGE(TAG, "GATT server init failed: %d", gatt_rc);
        return ESP_FAIL;
    }

    /* Set device name before advertising */
    int rc = ble_svc_gap_device_name_set(
        g_exo_hardware_config.ble_device_name);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to set BLE device name: %d", rc);
        return ESP_FAIL;
    }

    /* Start NimBLE host task; ble_on_sync() starts advertising. */
    nimble_port_freertos_init(ble_host_task);

    /* Create console command processing task */
    if (xTaskCreatePinnedToCore(ble_console_task, "ble_console", 4096, NULL, 5,
                                NULL, EXO_CONSOLE_TASK_CORE) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create BLE console task");
        return ESP_ERR_NO_MEM;
    }

    if (xTaskCreatePinnedToCore(uart_console_task, "uart_console", 4096, NULL,
                                5, NULL, EXO_CONSOLE_TASK_CORE) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create programming UART console task");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "BLE and programming UART consoles initialized");
    return ESP_OK;
}
