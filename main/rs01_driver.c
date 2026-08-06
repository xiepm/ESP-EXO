#include "rs01_driver.h"

#include <math.h>
#include <string.h>

#include "exo_config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "rs01";

enum {
    RS01_PRIVATE_TYPE_CONTROL = 0x01,
    RS01_PRIVATE_TYPE_FEEDBACK = 0x02,
    RS01_PRIVATE_TYPE_ENABLE = 0x03,
    RS01_PRIVATE_TYPE_STOP = 0x04,
    RS01_PRIVATE_TYPE_ZERO = 0x06,
    RS01_PRIVATE_TYPE_SET_CAN_ID = 0x07,
    RS01_PRIVATE_TYPE_PARAMETER_READ = 0x11,
    RS01_PRIVATE_TYPE_PARAMETER_WRITE = 0x12,
    RS01_PRIVATE_TYPE_FAULT = 0x15,
    RS01_PRIVATE_TYPE_SAVE = 0x16,
    RS01_PRIVATE_TYPE_BAUD = 0x17,
    RS01_PRIVATE_TYPE_REPORT = 0x18,
    RS01_PRIVATE_TYPE_PROTOCOL = 0x19,

    RS01_PARAMETER_RUN_MODE = 0x7005,
    RS01_PARAMETER_CURRENT_REFERENCE = 0x7006,
    RS01_PARAMETER_SPEED_REFERENCE = 0x700A,
    RS01_PARAMETER_POSITION_REFERENCE = 0x7016,
    RS01_PARAMETER_CSP_SPEED_LIMIT = 0x7017,
    RS01_PARAMETER_CURRENT_LIMIT = 0x7018,
    RS01_PARAMETER_ACCELERATION = 0x7022,
    RS01_PARAMETER_PP_SPEED = 0x7024,
    RS01_PARAMETER_PP_ACCELERATION = 0x7025,
    RS01_PARAMETER_REPORT_INTERVAL = 0x7026,

    RS01_MAX_CAN_ID = 0x7F,
    RS01_FRAME_DATA_LENGTH = 8,
    RS01_MIT_MANUAL_FAULT_RESPONSE_LENGTH = 8,
    RS01_MIT_LEGACY_FAULT_RESPONSE_LENGTH = 5,
    RS01_MIT_FAULT_RESPONSE_TIMEOUT_MS = 100,
    RS01_TX_TIMEOUT_MS = 10,
};

static const rs01_limits_t s_limits = {
    .position_min_rad = -12.57f,
    .position_max_rad = 12.57f,
    .velocity_min_rad_s = -44.0f,
    .velocity_max_rad_s = 44.0f,
    .torque_min_nm = -17.0f,
    .torque_max_nm = 17.0f,
    .kp_min = 0.0f,
    .kp_max = 500.0f,
    .kd_min = 0.0f,
    .kd_max = 5.0f,
};

typedef struct {
    exo_actuator_id_t actuator;
    exo_can_node_id_t can_id;
    rs01_feedback_snapshot_t feedback;
    rs01_fault_flags_t summary_faults;
    bool mit_fault_response_pending;
    uint64_t mit_fault_query_timestamp_us;
} rs01_motor_t;

static rs01_motor_t s_motors[EXO_ACTUATOR_COUNT];
static StaticSemaphore_t s_tx_mutex_storage;
static SemaphoreHandle_t s_tx_mutex;
static portMUX_TYPE s_feedback_lock = portMUX_INITIALIZER_UNLOCKED;

static exo_can_node_id_t current_can_id(exo_actuator_id_t actuator)
{
    portENTER_CRITICAL(&s_feedback_lock);
    const exo_can_node_id_t can_id = s_motors[actuator].can_id;
    portEXIT_CRITICAL(&s_feedback_lock);
    return can_id;
}

static bool can_id_conflicts(exo_actuator_id_t actuator,
                             exo_can_node_id_t can_id)
{
    bool conflict = false;
    portENTER_CRITICAL(&s_feedback_lock);
    for (unsigned i = 0; i < (unsigned)EXO_ACTUATOR_COUNT; ++i) {
        if (i != (unsigned)actuator && s_motors[i].can_id == can_id) {
            conflict = true;
            break;
        }
    }
    portEXIT_CRITICAL(&s_feedback_lock);
    return conflict;
}

static void set_current_can_id(exo_actuator_id_t actuator,
                               exo_can_node_id_t can_id)
{
    portENTER_CRITICAL(&s_feedback_lock);
    s_motors[actuator].can_id = can_id;
    portEXIT_CRITICAL(&s_feedback_lock);
}

static bool actuator_is_valid(exo_actuator_id_t actuator)
{
    return (unsigned)actuator < (unsigned)EXO_ACTUATOR_COUNT;
}

static const char *actuator_name(exo_actuator_id_t actuator)
{
    return actuator == EXO_ACTUATOR_A ? "A" : "B";
}

static uint32_t make_extended_id(uint8_t destination,
                                 uint16_t data,
                                 uint8_t type)
{
    return (uint32_t)destination |
           ((uint32_t)data << 8) |
           (((uint32_t)type & 0x1FU) << 24);
}

static uint8_t extended_id_destination(uint32_t identifier)
{
    return (uint8_t)(identifier & 0xFFU);
}

static uint16_t extended_id_data(uint32_t identifier)
{
    return (uint16_t)((identifier >> 8) & 0xFFFFU);
}

static uint8_t extended_id_type(uint32_t identifier)
{
    return (uint8_t)((identifier >> 24) & 0x1FU);
}

static uint16_t read_u16_be(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
}

static uint32_t read_u32_le(const uint8_t *data)
{
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[3] << 24);
}

static void write_u16_be(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value >> 8);
    data[1] = (uint8_t)value;
}

static void write_u16_le(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
}

static void write_u32_le(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
    data[2] = (uint8_t)(value >> 16);
    data[3] = (uint8_t)(value >> 24);
}

static uint32_t float_to_raw(float value,
                             float minimum,
                             float maximum,
                             unsigned bits)
{
    if (value > maximum) {
        value = maximum;
    } else if (value < minimum) {
        value = minimum;
    }

    const float span = maximum - minimum;
    const uint32_t raw_max = (1UL << bits) - 1UL;
    return (uint32_t)((value - minimum) * (float)raw_max / span);
}

static float raw_to_float(uint32_t value,
                          float minimum,
                          float maximum,
                          uint32_t raw_max)
{
    return ((float)value / (float)raw_max) * (maximum - minimum) + minimum;
}

static bool motion_values_are_finite(float position_rad,
                                     float velocity_rad_s,
                                     float torque_nm,
                                     float kp,
                                     float kd)
{
    return isfinite(position_rad) && isfinite(velocity_rad_s) &&
           isfinite(torque_nm) && isfinite(kp) && isfinite(kd);
}

static esp_err_t take_tx_lock(void)
{
    if (s_tx_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    return xSemaphoreTakeRecursive(s_tx_mutex, portMAX_DELAY) == pdTRUE
               ? ESP_OK
               : ESP_ERR_TIMEOUT;
}

static void give_tx_lock(void)
{
    xSemaphoreGiveRecursive(s_tx_mutex);
}

static esp_err_t transmit_frame(bool extended,
                                uint32_t identifier,
                                const uint8_t payload[RS01_FRAME_DATA_LENGTH])
{
    esp_err_t err = take_tx_lock();
    if (err != ESP_OK) {
        return err;
    }

    twai_message_t message = {0};
    message.identifier = identifier;
    message.data_length_code = RS01_FRAME_DATA_LENGTH;
    message.flags = extended ? TWAI_MSG_FLAG_EXTD : TWAI_MSG_FLAG_NONE;
    memcpy(message.data, payload, RS01_FRAME_DATA_LENGTH);

    err = twai_transmit(&message, pdMS_TO_TICKS(RS01_TX_TIMEOUT_MS));
    give_tx_lock();

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "%s TX id=0x%08lX failed: %s",
                 extended ? "extended" : "standard",
                 (unsigned long)identifier,
                 esp_err_to_name(err));
    }
    return err;
}

static esp_err_t transmit_private(exo_actuator_id_t actuator,
                                  uint8_t type,
                                  uint16_t data_field,
                                  const uint8_t payload[RS01_FRAME_DATA_LENGTH])
{
    if (!actuator_is_valid(actuator)) {
        return ESP_ERR_INVALID_ARG;
    }
    return transmit_frame(
        true,
        make_extended_id(current_can_id(actuator), data_field, type),
        payload);
}

static esp_err_t transmit_standard(exo_actuator_id_t actuator,
                                   uint16_t identifier,
                                   const uint8_t payload[RS01_FRAME_DATA_LENGTH])
{
    if (!actuator_is_valid(actuator)) {
        return ESP_ERR_INVALID_ARG;
    }
    return transmit_frame(false, identifier, payload);
}

static void set_enabled_requested(exo_actuator_id_t actuator, bool requested)
{
    portENTER_CRITICAL(&s_feedback_lock);
    s_motors[actuator].feedback.enabled_requested = requested;
    portEXIT_CRITICAL(&s_feedback_lock);
}

static void set_reporting_requested(exo_actuator_id_t actuator, bool requested)
{
    portENTER_CRITICAL(&s_feedback_lock);
    s_motors[actuator].feedback.reporting_requested = requested;
    portEXIT_CRITICAL(&s_feedback_lock);
}

static void set_control_mode_state(exo_actuator_id_t actuator,
                                   rs01_control_mode_t mode)
{
    portENTER_CRITICAL(&s_feedback_lock);
    s_motors[actuator].feedback.control_mode = mode;
    portEXIT_CRITICAL(&s_feedback_lock);
}

static void increment_invalid_frame(exo_actuator_id_t actuator)
{
    if (!actuator_is_valid(actuator)) {
        return;
    }
    portENTER_CRITICAL(&s_feedback_lock);
    ++s_motors[actuator].feedback.invalid_frame_count;
    portEXIT_CRITICAL(&s_feedback_lock);
}

static void set_mit_fault_response_pending(exo_actuator_id_t actuator,
                                           bool pending)
{
    const uint64_t now_us = (uint64_t)esp_timer_get_time();
    portENTER_CRITICAL(&s_feedback_lock);
    s_motors[actuator].mit_fault_response_pending = pending;
    s_motors[actuator].mit_fault_query_timestamp_us = pending ? now_us : 0U;
    portEXIT_CRITICAL(&s_feedback_lock);
}

static bool mit_fault_response_is_pending(exo_actuator_id_t actuator)
{
    const uint64_t now_us = (uint64_t)esp_timer_get_time();
    const uint64_t timeout_us =
        (uint64_t)RS01_MIT_FAULT_RESPONSE_TIMEOUT_MS * 1000ULL;
    portENTER_CRITICAL(&s_feedback_lock);
    const uint64_t requested_us =
        s_motors[actuator].mit_fault_query_timestamp_us;
    const bool pending = s_motors[actuator].mit_fault_response_pending &&
                         now_us >= requested_us &&
                         (now_us - requested_us) <= timeout_us;
    if (!pending) {
        s_motors[actuator].mit_fault_response_pending = false;
        s_motors[actuator].mit_fault_query_timestamp_us = 0U;
    }
    portEXIT_CRITICAL(&s_feedback_lock);
    return pending;
}

static rs01_fault_flags_t private_summary_faults(uint8_t raw)
{
    rs01_fault_flags_t faults = {0};
    faults.present = raw != 0U;
    faults.uncalibrated = (raw & (1U << 5)) != 0U;
    faults.stall_overload = (raw & (1U << 4)) != 0U;
    faults.magnetic_encoder = (raw & (1U << 3)) != 0U;
    faults.over_temperature = (raw & (1U << 2)) != 0U;
    faults.phase_over_current = (raw & (1U << 1)) != 0U;
    faults.under_voltage = (raw & (1U << 0)) != 0U;
    return faults;
}

static rs01_fault_flags_t detailed_faults(uint32_t fault,
                                          uint32_t warning)
{
    rs01_fault_flags_t flags = {0};
    flags.present = fault != 0U;
    flags.warning_present = warning != 0U;
    flags.uncalibrated = (fault & (1UL << 7)) != 0U;
    flags.stall_overload = (fault & (1UL << 14)) != 0U;
    flags.over_temperature = (fault & (1UL << 0)) != 0U;
    flags.phase_over_current =
        (fault & ((1UL << 16) | (1UL << 5) | (1UL << 4))) != 0U;
    flags.under_voltage = (fault & (1UL << 2)) != 0U;
    flags.over_voltage = (fault & (1UL << 3)) != 0U;
    flags.driver_chip = (fault & (1UL << 1)) != 0U;
    flags.position_initialization = (fault & (1UL << 9)) != 0U;
    flags.hardware_identification = (fault & (1UL << 8)) != 0U;
    flags.over_temperature_warning = (warning & 1UL) != 0U;
    return flags;
}

static rs01_fault_flags_t merge_faults(rs01_fault_flags_t first,
                                       rs01_fault_flags_t second)
{
    rs01_fault_flags_t merged = {
        .present = first.present || second.present,
        .warning_present = first.warning_present || second.warning_present,
        .uncalibrated = first.uncalibrated || second.uncalibrated,
        .stall_overload = first.stall_overload || second.stall_overload,
        .magnetic_encoder = first.magnetic_encoder || second.magnetic_encoder,
        .over_temperature = first.over_temperature || second.over_temperature,
        .phase_over_current =
            first.phase_over_current || second.phase_over_current,
        .under_voltage = first.under_voltage || second.under_voltage,
        .over_voltage = first.over_voltage || second.over_voltage,
        .driver_chip = first.driver_chip || second.driver_chip,
        .position_initialization =
            first.position_initialization || second.position_initialization,
        .hardware_identification =
            first.hardware_identification || second.hardware_identification,
        .over_temperature_warning =
            first.over_temperature_warning || second.over_temperature_warning,
    };
    return merged;
}

static rs01_mode_status_t decode_mode(uint8_t raw)
{
    return raw <= (uint8_t)RS01_MODE_STATUS_RUNNING
               ? (rs01_mode_status_t)raw
               : RS01_MODE_STATUS_UNKNOWN;
}

static void update_feedback(exo_actuator_id_t actuator,
                            const exo_motion_sample_t *sample,
                            rs01_mode_status_t mode,
                            const rs01_fault_flags_t *faults,
                            uint8_t feedback_fault_raw,
                            uint8_t private_frame_type)
{
    const uint64_t now_us = (uint64_t)esp_timer_get_time();

    portENTER_CRITICAL(&s_feedback_lock);
    rs01_feedback_snapshot_t *feedback = &s_motors[actuator].feedback;
    if (feedback->feedback_count == 0U) {
        feedback->first_timestamp_us = now_us;
        feedback->last_interval_us = 0U;
    } else {
        feedback->last_interval_us = now_us - feedback->timestamp_us;
    }
    feedback->sample = *sample;
    feedback->mode = mode;
    s_motors[actuator].summary_faults = *faults;
    feedback->faults = merge_faults(
        *faults, detailed_faults(feedback->fault_raw,
                                 feedback->warning_raw));
    feedback->feedback_fault_raw = feedback_fault_raw;
    feedback->timestamp_us = now_us;
    ++feedback->feedback_count;
    if (private_frame_type == RS01_PRIVATE_TYPE_FEEDBACK) {
        ++feedback->type2_feedback_count;
    } else if (private_frame_type == RS01_PRIVATE_TYPE_REPORT) {
        ++feedback->type24_feedback_count;
    }
    feedback->valid = true;
    portEXIT_CRITICAL(&s_feedback_lock);
}

static void update_fault(exo_actuator_id_t actuator,
                         uint32_t fault,
                         uint32_t warning)
{
    const rs01_fault_flags_t flags = detailed_faults(fault, warning);
    portENTER_CRITICAL(&s_feedback_lock);
    rs01_feedback_snapshot_t *feedback = &s_motors[actuator].feedback;
    feedback->fault_raw = fault;
    feedback->warning_raw = warning;
    feedback->faults = merge_faults(s_motors[actuator].summary_faults,
                                    flags);
    portEXIT_CRITICAL(&s_feedback_lock);
}

esp_err_t rs01_driver_init(void)
{
    if (s_tx_mutex != NULL) {
        return ESP_OK;
    }

    SemaphoreHandle_t mutex =
        xSemaphoreCreateRecursiveMutexStatic(&s_tx_mutex_storage);
    if (mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }

    portENTER_CRITICAL(&s_feedback_lock);
    memset(s_motors, 0, sizeof(s_motors));
    for (unsigned i = 0; i < (unsigned)EXO_ACTUATOR_COUNT; ++i) {
        s_motors[i].actuator = (exo_actuator_id_t)i;
        s_motors[i].can_id = g_exo_hardware_config.actuator[i].can_id;
        s_motors[i].feedback.mode = RS01_MODE_STATUS_UNKNOWN;
        s_motors[i].feedback.control_mode = RS01_CONTROL_MODE_UNKNOWN;
    }
    portEXIT_CRITICAL(&s_feedback_lock);

    s_tx_mutex = mutex;
    return ESP_OK;
}

const rs01_limits_t *rs01_get_limits(void)
{
    return &s_limits;
}

bool rs01_get_can_id(exo_actuator_id_t actuator,
                     exo_can_node_id_t *can_id)
{
    if (!actuator_is_valid(actuator) || can_id == NULL) {
        return false;
    }
    *can_id = current_can_id(actuator);
    return true;
}

bool rs01_find_actuator(exo_can_node_id_t can_id,
                        exo_actuator_id_t *actuator)
{
    if (actuator == NULL) {
        return false;
    }
    bool found = false;
    portENTER_CRITICAL(&s_feedback_lock);
    for (unsigned i = 0; i < (unsigned)EXO_ACTUATOR_COUNT; ++i) {
        if (s_motors[i].can_id == can_id) {
            *actuator = (exo_actuator_id_t)i;
            found = true;
            break;
        }
    }
    portEXIT_CRITICAL(&s_feedback_lock);
    return found;
}

esp_err_t rs01_enable(exo_actuator_id_t actuator)
{
    uint8_t payload[RS01_FRAME_DATA_LENGTH] = {0};
    esp_err_t err = transmit_private(
        actuator,
        RS01_PRIVATE_TYPE_ENABLE,
        g_exo_hardware_config.twai.master_id,
        payload);
    if (err == ESP_OK) {
        set_enabled_requested(actuator, true);
    }
    return err;
}

static esp_err_t send_stop(exo_actuator_id_t actuator, bool clear_fault)
{
    uint8_t payload[RS01_FRAME_DATA_LENGTH] = {0};
    payload[0] = clear_fault ? 1U : 0U;
    esp_err_t err = transmit_private(
        actuator,
        RS01_PRIVATE_TYPE_STOP,
        g_exo_hardware_config.twai.master_id,
        payload);
    if (err == ESP_OK && !clear_fault) {
        set_enabled_requested(actuator, false);
    }
    return err;
}

esp_err_t rs01_disable(exo_actuator_id_t actuator)
{
    return send_stop(actuator, false);
}

esp_err_t rs01_clear_faults(exo_actuator_id_t actuator)
{
    return send_stop(actuator, true);
}

esp_err_t rs01_send_motion(exo_actuator_id_t actuator,
                           float position_rad,
                           float velocity_rad_s,
                           float torque_nm,
                           float kp,
                           float kd)
{
    if (!actuator_is_valid(actuator) ||
        !motion_values_are_finite(position_rad, velocity_rad_s,
                                  torque_nm, kp, kd)) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t payload[RS01_FRAME_DATA_LENGTH] = {0};
    write_u16_be(&payload[0],
                 (uint16_t)float_to_raw(position_rad,
                                        s_limits.position_min_rad,
                                        s_limits.position_max_rad, 16));
    write_u16_be(&payload[2],
                 (uint16_t)float_to_raw(velocity_rad_s,
                                        s_limits.velocity_min_rad_s,
                                        s_limits.velocity_max_rad_s, 16));
    write_u16_be(&payload[4],
                 (uint16_t)float_to_raw(kp, s_limits.kp_min,
                                        s_limits.kp_max, 16));
    write_u16_be(&payload[6],
                 (uint16_t)float_to_raw(kd, s_limits.kd_min,
                                        s_limits.kd_max, 16));
    const uint16_t torque_raw =
        (uint16_t)float_to_raw(torque_nm,
                               s_limits.torque_min_nm,
                               s_limits.torque_max_nm, 16);
    return transmit_private(actuator, RS01_PRIVATE_TYPE_CONTROL,
                            torque_raw, payload);
}

static esp_err_t send_parameter(exo_actuator_id_t actuator,
                                uint16_t index,
                                const uint8_t value[4])
{
    if (!actuator_is_valid(actuator)) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = take_tx_lock();
    if (err != ESP_OK) {
        return err;
    }

    uint8_t payload[RS01_FRAME_DATA_LENGTH] = {0};
    write_u16_le(&payload[0], index);
    memcpy(&payload[4], value, 4);
    err = transmit_private(
        actuator,
        RS01_PRIVATE_TYPE_PARAMETER_WRITE,
        g_exo_hardware_config.twai.master_id,
        payload);
    if (err == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    give_tx_lock();
    return err;
}

esp_err_t rs01_read_parameter(exo_actuator_id_t actuator, uint16_t index)
{
    uint8_t payload[RS01_FRAME_DATA_LENGTH] = {0};
    write_u16_le(&payload[0], index);
    return transmit_private(
        actuator,
        RS01_PRIVATE_TYPE_PARAMETER_READ,
        g_exo_hardware_config.twai.master_id,
        payload);
}

esp_err_t rs01_write_parameter_float(exo_actuator_id_t actuator,
                                      uint16_t index,
                                      float value)
{
    if (!isfinite(value)) {
        return ESP_ERR_INVALID_ARG;
    }
    uint32_t raw;
    memcpy(&raw, &value, sizeof(raw));
    uint8_t encoded[4];
    write_u32_le(encoded, raw);
    return send_parameter(actuator, index, encoded);
}

esp_err_t rs01_write_parameter_u8(exo_actuator_id_t actuator,
                                   uint16_t index,
                                   uint8_t value)
{
    const uint8_t encoded[4] = {value, 0, 0, 0};
    esp_err_t err = send_parameter(actuator, index, encoded);
    if (err == ESP_OK && index == RS01_PARAMETER_RUN_MODE) {
        rs01_control_mode_t mode = RS01_CONTROL_MODE_UNKNOWN;
        if (value == (uint8_t)RS01_CONTROL_MODE_MOTION ||
            value == (uint8_t)RS01_CONTROL_MODE_POSITION_PP ||
            value == (uint8_t)RS01_CONTROL_MODE_VELOCITY ||
            value == (uint8_t)RS01_CONTROL_MODE_CURRENT ||
            value == (uint8_t)RS01_CONTROL_MODE_POSITION_CSP) {
            mode = (rs01_control_mode_t)value;
        }
        set_control_mode_state(actuator, mode);
    }
    return err;
}

esp_err_t rs01_write_parameter_u16(exo_actuator_id_t actuator,
                                    uint16_t index,
                                    uint16_t value)
{
    uint8_t encoded[4] = {0};
    write_u16_le(encoded, value);
    return send_parameter(actuator, index, encoded);
}

static bool control_mode_is_valid(rs01_control_mode_t mode)
{
    return mode == RS01_CONTROL_MODE_MOTION ||
           mode == RS01_CONTROL_MODE_POSITION_PP ||
           mode == RS01_CONTROL_MODE_VELOCITY ||
           mode == RS01_CONTROL_MODE_CURRENT ||
           mode == RS01_CONTROL_MODE_POSITION_CSP;
}

esp_err_t rs01_set_control_mode(exo_actuator_id_t actuator,
                                rs01_control_mode_t mode)
{
    if (!control_mode_is_valid(mode)) {
        return ESP_ERR_INVALID_ARG;
    }
    return rs01_write_parameter_u8(actuator, RS01_PARAMETER_RUN_MODE,
                                   (uint8_t)mode);
}

static esp_err_t prepare_control_mode(exo_actuator_id_t actuator,
                                      rs01_control_mode_t mode)
{
    rs01_feedback_snapshot_t feedback;
    if (rs01_get_feedback(actuator, &feedback) && feedback.valid &&
        feedback.mode == RS01_MODE_STATUS_RUNNING) {
        esp_err_t err = rs01_disable(actuator);
        if (err != ESP_OK) {
            return err;
        }
    }

    esp_err_t err = rs01_set_control_mode(actuator, mode);
    if (err != ESP_OK) {
        return err;
    }
    return rs01_enable(actuator);
}

esp_err_t rs01_move(exo_actuator_id_t actuator,
                    float torque_nm,
                    float position_rad,
                    float velocity_rad_s,
                    float kp,
                    float kd)
{
    if (!motion_values_are_finite(position_rad, velocity_rad_s,
                                  torque_nm, kp, kd)) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = take_tx_lock();
    if (err != ESP_OK) {
        return err;
    }
    err = prepare_control_mode(actuator, RS01_CONTROL_MODE_MOTION);
    if (err == ESP_OK) {
        err = rs01_send_motion(actuator, position_rad, velocity_rad_s,
                               torque_nm, kp, kd);
    }
    give_tx_lock();
    return err;
}

esp_err_t rs01_speed_control(exo_actuator_id_t actuator,
                             float current_limit_a,
                             float acceleration_rad_s2,
                             float velocity_rad_s)
{
    if (!isfinite(current_limit_a) || !isfinite(acceleration_rad_s2) ||
        !isfinite(velocity_rad_s)) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = take_tx_lock();
    if (err != ESP_OK) {
        return err;
    }
    err = prepare_control_mode(actuator, RS01_CONTROL_MODE_VELOCITY);
    if (err == ESP_OK) {
        err = rs01_write_parameter_float(actuator,
                                         RS01_PARAMETER_CURRENT_LIMIT,
                                         current_limit_a);
    }
    if (err == ESP_OK) {
        err = rs01_write_parameter_float(actuator,
                                         RS01_PARAMETER_ACCELERATION,
                                         acceleration_rad_s2);
    }
    if (err == ESP_OK) {
        err = rs01_write_parameter_float(actuator,
                                         RS01_PARAMETER_SPEED_REFERENCE,
                                         velocity_rad_s);
    }
    give_tx_lock();
    return err;
}

esp_err_t rs01_position_pp_control(exo_actuator_id_t actuator,
                                   float velocity_rad_s,
                                   float acceleration_rad_s2,
                                   float position_rad)
{
    if (!isfinite(velocity_rad_s) || !isfinite(acceleration_rad_s2) ||
        !isfinite(position_rad)) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = take_tx_lock();
    if (err != ESP_OK) {
        return err;
    }
    err = prepare_control_mode(actuator, RS01_CONTROL_MODE_POSITION_PP);
    if (err == ESP_OK) {
        err = rs01_write_parameter_float(actuator,
                                         RS01_PARAMETER_PP_SPEED,
                                         velocity_rad_s);
    }
    if (err == ESP_OK) {
        err = rs01_write_parameter_float(actuator,
                                         RS01_PARAMETER_PP_ACCELERATION,
                                         acceleration_rad_s2);
    }
    if (err == ESP_OK) {
        err = rs01_write_parameter_float(actuator,
                                         RS01_PARAMETER_POSITION_REFERENCE,
                                         position_rad);
    }
    give_tx_lock();
    return err;
}

esp_err_t rs01_position_csp_control(exo_actuator_id_t actuator,
                                    float velocity_limit_rad_s,
                                    float position_rad)
{
    if (!isfinite(velocity_limit_rad_s) || !isfinite(position_rad)) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = take_tx_lock();
    if (err != ESP_OK) {
        return err;
    }
    err = prepare_control_mode(actuator, RS01_CONTROL_MODE_POSITION_CSP);
    if (err == ESP_OK) {
        err = rs01_write_parameter_float(actuator,
                                         RS01_PARAMETER_CSP_SPEED_LIMIT,
                                         velocity_limit_rad_s);
    }
    if (err == ESP_OK) {
        err = rs01_write_parameter_float(actuator,
                                         RS01_PARAMETER_POSITION_REFERENCE,
                                         position_rad);
    }
    give_tx_lock();
    return err;
}

esp_err_t rs01_current_control(exo_actuator_id_t actuator,
                               float current_a)
{
    if (!isfinite(current_a)) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = take_tx_lock();
    if (err != ESP_OK) {
        return err;
    }
    err = prepare_control_mode(actuator, RS01_CONTROL_MODE_CURRENT);
    if (err == ESP_OK) {
        err = rs01_write_parameter_float(actuator,
                                         RS01_PARAMETER_CURRENT_REFERENCE,
                                         current_a);
    }
    give_tx_lock();
    return err;
}

esp_err_t rs01_set_report_interval_ms(exo_actuator_id_t actuator,
                                      uint32_t interval_ms)
{
    if (interval_ms < 10U || ((interval_ms - 10U) % 5U) != 0U) {
        return ESP_ERR_INVALID_ARG;
    }
    const uint32_t encoded = 1U + ((interval_ms - 10U) / 5U);
    if (encoded > UINT16_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    return rs01_write_parameter_u16(actuator,
                                    RS01_PARAMETER_REPORT_INTERVAL,
                                    (uint16_t)encoded);
}

static void fill_private_maintenance_payload(uint8_t payload[8],
                                             uint8_t command)
{
    for (unsigned i = 0; i < 6U; ++i) {
        payload[i] = (uint8_t)(i + 1U);
    }
    payload[6] = command;
    payload[7] = 0x08;
}

static esp_err_t set_private_reporting(exo_actuator_id_t actuator, bool enabled)
{
    uint8_t payload[RS01_FRAME_DATA_LENGTH];
    /*
     * The manual table lists 01..06,F_CMD but omits the eighth byte. The
     * verified phase-0/1 RS01 baseline uses a trailing 0x08; retain it until
     * firmware documentation or an acknowledged response proves otherwise.
     */
    fill_private_maintenance_payload(payload, enabled ? 1U : 0U);
    esp_err_t err = transmit_private(
        actuator,
        RS01_PRIVATE_TYPE_REPORT,
        g_exo_hardware_config.twai.master_id,
        payload);
    if (err == ESP_OK) {
        set_reporting_requested(actuator, enabled);
    }
    return err;
}

esp_err_t rs01_start_reporting(exo_actuator_id_t actuator)
{
    return set_private_reporting(actuator, true);
}

esp_err_t rs01_stop_reporting(exo_actuator_id_t actuator)
{
    return set_private_reporting(actuator, false);
}

esp_err_t rs01_set_mechanical_zero(exo_actuator_id_t actuator)
{
    esp_err_t err = take_tx_lock();
    if (err != ESP_OK) {
        return err;
    }
    err = rs01_disable(actuator);
    if (err == ESP_OK) {
        uint8_t payload[RS01_FRAME_DATA_LENGTH] = {0};
        payload[0] = 1U;
        err = transmit_private(
            actuator,
            RS01_PRIVATE_TYPE_ZERO,
            g_exo_hardware_config.twai.master_id,
            payload);
    }
    give_tx_lock();
    return err;
}

esp_err_t rs01_set_can_id(exo_actuator_id_t actuator,
                          exo_can_node_id_t new_can_id)
{
    if (!actuator_is_valid(actuator) || new_can_id > RS01_MAX_CAN_ID) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = take_tx_lock();
    if (err != ESP_OK) {
        return err;
    }
    if (can_id_conflicts(actuator, new_can_id)) {
        give_tx_lock();
        return ESP_ERR_INVALID_ARG;
    }
    err = rs01_disable(actuator);
    if (err == ESP_OK) {
        uint8_t payload[RS01_FRAME_DATA_LENGTH] = {0};
        const uint16_t data_field =
            ((uint16_t)new_can_id << 8) |
            g_exo_hardware_config.twai.master_id;
        err = transmit_private(actuator, RS01_PRIVATE_TYPE_SET_CAN_ID,
                               data_field, payload);
    }
    if (err == ESP_OK) {
        set_current_can_id(actuator, new_can_id);
    }
    give_tx_lock();
    return err;
}

esp_err_t rs01_save_parameters(exo_actuator_id_t actuator)
{
    uint8_t payload[RS01_FRAME_DATA_LENGTH];
    for (unsigned i = 0; i < RS01_FRAME_DATA_LENGTH; ++i) {
        payload[i] = (uint8_t)(i + 1U);
    }
    return transmit_private(
        actuator,
        RS01_PRIVATE_TYPE_SAVE,
        g_exo_hardware_config.twai.master_id,
        payload);
}

esp_err_t rs01_set_baud_rate(exo_actuator_id_t actuator,
                             uint8_t baud_selector)
{
    if (baud_selector < 1U || baud_selector > 4U) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t payload[RS01_FRAME_DATA_LENGTH];
    fill_private_maintenance_payload(payload, baud_selector);
    return transmit_private(
        actuator,
        RS01_PRIVATE_TYPE_BAUD,
        g_exo_hardware_config.twai.master_id,
        payload);
}

esp_err_t rs01_set_protocol(exo_actuator_id_t actuator,
                            rs01_protocol_t protocol)
{
    if ((unsigned)protocol > (unsigned)RS01_PROTOCOL_MIT) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t payload[RS01_FRAME_DATA_LENGTH];
    fill_private_maintenance_payload(payload, (uint8_t)protocol);
    return transmit_private(
        actuator,
        RS01_PRIVATE_TYPE_PROTOCOL,
        g_exo_hardware_config.twai.master_id,
        payload);
}

static esp_err_t send_mit_terminal_command(exo_actuator_id_t actuator,
                                            uint8_t terminal)
{
    uint8_t payload[RS01_FRAME_DATA_LENGTH];
    memset(payload, 0xFF, sizeof(payload));
    payload[7] = terminal;
    return transmit_standard(actuator, current_can_id(actuator), payload);
}

static esp_err_t send_mit_command_value(exo_actuator_id_t actuator,
                                        uint8_t value,
                                        uint8_t terminal)
{
    uint8_t payload[RS01_FRAME_DATA_LENGTH];
    memset(payload, 0xFF, 6);
    payload[6] = value;
    payload[7] = terminal;
    return transmit_standard(actuator, current_can_id(actuator), payload);
}

esp_err_t rs01_mit_enable(exo_actuator_id_t actuator)
{
    if (!actuator_is_valid(actuator)) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = send_mit_terminal_command(actuator, 0xFC);
    if (err == ESP_OK) {
        set_enabled_requested(actuator, true);
    }
    return err;
}

esp_err_t rs01_mit_disable(exo_actuator_id_t actuator)
{
    if (!actuator_is_valid(actuator)) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = send_mit_terminal_command(actuator, 0xFD);
    if (err == ESP_OK) {
        set_enabled_requested(actuator, false);
    }
    return err;
}

esp_err_t rs01_mit_clear_or_read_fault(exo_actuator_id_t actuator,
                                        uint8_t command)
{
    if (!actuator_is_valid(actuator)) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = send_mit_command_value(actuator, command, 0xFB);
    if (err == ESP_OK) {
        /* 0xFF clears and returns normal feedback; other values query fault. */
        set_mit_fault_response_pending(actuator, command != 0xFFU);
    }
    return err;
}

esp_err_t rs01_mit_set_operation_mode(exo_actuator_id_t actuator,
                                       uint8_t mode)
{
    if (!actuator_is_valid(actuator) || mode > 2U) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = send_mit_command_value(actuator, mode, 0xFC);
    if (err == ESP_OK) {
        const rs01_control_mode_t mapped[] = {
            RS01_CONTROL_MODE_MOTION,
            RS01_CONTROL_MODE_POSITION_CSP,
            RS01_CONTROL_MODE_VELOCITY,
        };
        set_control_mode_state(actuator, mapped[mode]);
    }
    return err;
}

esp_err_t rs01_mit_set_can_id(exo_actuator_id_t actuator,
                              exo_can_node_id_t new_can_id)
{
    if (!actuator_is_valid(actuator) || new_can_id > RS01_MAX_CAN_ID) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = take_tx_lock();
    if (err != ESP_OK) {
        return err;
    }
    if (can_id_conflicts(actuator, new_can_id)) {
        give_tx_lock();
        return ESP_ERR_INVALID_ARG;
    }
    err = send_mit_command_value(actuator, new_can_id, 0xFA);
    if (err == ESP_OK) {
        set_current_can_id(actuator, new_can_id);
    }
    give_tx_lock();
    return err;
}

esp_err_t rs01_mit_set_protocol(exo_actuator_id_t actuator,
                                rs01_protocol_t protocol)
{
    if (!actuator_is_valid(actuator) ||
        (unsigned)protocol > (unsigned)RS01_PROTOCOL_MIT) {
        return ESP_ERR_INVALID_ARG;
    }
    /* 0xFD here is the MIT command byte, not the configured master ID. */
    return send_mit_command_value(actuator, (uint8_t)protocol, 0xFD);
}

esp_err_t rs01_mit_send_motion(exo_actuator_id_t actuator,
                               float position_rad,
                               float velocity_rad_s,
                               float kp,
                               float kd,
                               float torque_nm)
{
    if (!actuator_is_valid(actuator) ||
        !motion_values_are_finite(position_rad, velocity_rad_s,
                                  torque_nm, kp, kd)) {
        return ESP_ERR_INVALID_ARG;
    }

    const uint16_t position =
        (uint16_t)float_to_raw(position_rad,
                               s_limits.position_min_rad,
                               s_limits.position_max_rad, 16);
    const uint16_t velocity =
        (uint16_t)float_to_raw(velocity_rad_s,
                               s_limits.velocity_min_rad_s,
                               s_limits.velocity_max_rad_s, 12);
    const uint16_t kp_raw =
        (uint16_t)float_to_raw(kp, s_limits.kp_min,
                               s_limits.kp_max, 12);
    const uint16_t kd_raw =
        (uint16_t)float_to_raw(kd, s_limits.kd_min,
                               s_limits.kd_max, 12);
    const uint16_t torque =
        (uint16_t)float_to_raw(torque_nm,
                               s_limits.torque_min_nm,
                               s_limits.torque_max_nm, 12);

    uint8_t payload[RS01_FRAME_DATA_LENGTH];
    payload[0] = (uint8_t)(position >> 8);
    payload[1] = (uint8_t)position;
    payload[2] = (uint8_t)(velocity >> 4);
    payload[3] = (uint8_t)((velocity << 4) | (kp_raw >> 8));
    payload[4] = (uint8_t)kp_raw;
    payload[5] = (uint8_t)(kd_raw >> 4);
    payload[6] = (uint8_t)((kd_raw << 4) | (torque >> 8));
    payload[7] = (uint8_t)torque;
    return transmit_standard(actuator, current_can_id(actuator), payload);
}

static esp_err_t send_two_floats(exo_actuator_id_t actuator,
                                 uint16_t identifier,
                                 float first,
                                 float second)
{
    if (!actuator_is_valid(actuator) ||
        !isfinite(first) || !isfinite(second)) {
        return ESP_ERR_INVALID_ARG;
    }
    uint32_t first_raw;
    uint32_t second_raw;
    memcpy(&first_raw, &first, sizeof(first_raw));
    memcpy(&second_raw, &second, sizeof(second_raw));
    uint8_t payload[RS01_FRAME_DATA_LENGTH];
    write_u32_le(&payload[0], first_raw);
    write_u32_le(&payload[4], second_raw);
    return transmit_standard(actuator, identifier, payload);
}

esp_err_t rs01_mit_position_control(exo_actuator_id_t actuator,
                                    float position_rad,
                                    float velocity_rad_s)
{
    if (!actuator_is_valid(actuator)) {
        return ESP_ERR_INVALID_ARG;
    }
    const uint16_t identifier =
        (uint16_t)((1U << 8) | current_can_id(actuator));
    return send_two_floats(actuator, identifier,
                           position_rad, velocity_rad_s);
}

esp_err_t rs01_mit_speed_control(exo_actuator_id_t actuator,
                                 float velocity_rad_s,
                                 float current_limit_a)
{
    if (!actuator_is_valid(actuator)) {
        return ESP_ERR_INVALID_ARG;
    }
    const uint16_t identifier =
        (uint16_t)((2U << 8) | current_can_id(actuator));
    return send_two_floats(actuator, identifier,
                           velocity_rad_s, current_limit_a);
}

esp_err_t rs01_mit_set_zero(exo_actuator_id_t actuator)
{
    if (!actuator_is_valid(actuator)) {
        return ESP_ERR_INVALID_ARG;
    }
    return send_mit_terminal_command(actuator, 0xFE);
}

esp_err_t rs01_mit_save_parameters(exo_actuator_id_t actuator)
{
    if (!actuator_is_valid(actuator)) {
        return ESP_ERR_INVALID_ARG;
    }
    return send_mit_terminal_command(actuator, 0xF8);
}

static esp_err_t set_mit_reporting(exo_actuator_id_t actuator, bool enabled)
{
    if (!actuator_is_valid(actuator)) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = send_mit_command_value(actuator, enabled ? 1U : 0U,
                                            0xF9);
    if (err == ESP_OK) {
        set_reporting_requested(actuator, enabled);
    }
    return err;
}

esp_err_t rs01_mit_start_reporting(exo_actuator_id_t actuator)
{
    return set_mit_reporting(actuator, true);
}

esp_err_t rs01_mit_stop_reporting(exo_actuator_id_t actuator)
{
    return set_mit_reporting(actuator, false);
}

esp_err_t rs01_mit_read_parameter(exo_actuator_id_t actuator,
                                  uint16_t index)
{
    if (!actuator_is_valid(actuator)) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t payload[RS01_FRAME_DATA_LENGTH] = {0};
    write_u16_le(&payload[0], index);
    const uint16_t identifier =
        (uint16_t)((3U << 8) | current_can_id(actuator));
    return transmit_standard(actuator, identifier, payload);
}

esp_err_t rs01_mit_write_parameter_float(exo_actuator_id_t actuator,
                                          uint16_t index,
                                          float value)
{
    if (!actuator_is_valid(actuator) || !isfinite(value)) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t payload[RS01_FRAME_DATA_LENGTH] = {0};
    write_u16_le(&payload[0], index);
    uint32_t raw;
    memcpy(&raw, &value, sizeof(raw));
    write_u32_le(&payload[4], raw);
    const uint16_t identifier =
        (uint16_t)((4U << 8) | current_can_id(actuator));
    return transmit_standard(actuator, identifier, payload);
}

bool rs01_get_feedback(exo_actuator_id_t actuator,
                       rs01_feedback_snapshot_t *snapshot)
{
    if (!actuator_is_valid(actuator) || snapshot == NULL) {
        return false;
    }

    portENTER_CRITICAL(&s_feedback_lock);
    *snapshot = s_motors[actuator].feedback;
    portEXIT_CRITICAL(&s_feedback_lock);

    const uint64_t now_us = (uint64_t)esp_timer_get_time();
    const uint64_t timeout_us =
        (uint64_t)g_exo_hardware_config.rs01_feedback_timeout_ms * 1000ULL;
    snapshot->online = snapshot->valid && timeout_us > 0U &&
                       now_us >= snapshot->timestamp_us &&
                       (now_us - snapshot->timestamp_us) <= timeout_us;
    return true;
}

static rs01_rx_event_t invalid_event(exo_actuator_id_t actuator,
                                     exo_actuator_id_t *updated_actuator)
{
    if (actuator_is_valid(actuator)) {
        increment_invalid_frame(actuator);
        if (updated_actuator != NULL) {
            *updated_actuator = actuator;
        }
    }
    return RS01_RX_EVENT_INVALID;
}

static rs01_rx_event_t process_private_feedback(
    const twai_message_t *frame,
    exo_actuator_id_t *updated_actuator)
{
    const uint16_t data_field = extended_id_data(frame->identifier);
    exo_actuator_id_t actuator = EXO_ACTUATOR_COUNT;
    const bool source_known =
        rs01_find_actuator((uint8_t)data_field, &actuator);

    if (frame->data_length_code != RS01_FRAME_DATA_LENGTH ||
        extended_id_destination(frame->identifier) !=
            g_exo_hardware_config.twai.master_id ||
        !source_known) {
        return invalid_event(actuator, updated_actuator);
    }

    const uint8_t status = (uint8_t)(data_field >> 8);
    const uint8_t feedback_fault_raw = status & 0x3FU;
    const rs01_fault_flags_t faults =
        private_summary_faults(feedback_fault_raw);
    exo_motion_sample_t sample = {
        .position_rad = raw_to_float(read_u16_be(&frame->data[0]),
                                     s_limits.position_min_rad,
                                     s_limits.position_max_rad, UINT16_MAX),
        .velocity_rad_s = raw_to_float(read_u16_be(&frame->data[2]),
                                       s_limits.velocity_min_rad_s,
                                       s_limits.velocity_max_rad_s, UINT16_MAX),
        .torque_nm = raw_to_float(read_u16_be(&frame->data[4]),
                                  s_limits.torque_min_nm,
                                  s_limits.torque_max_nm, UINT16_MAX),
        .temperature_c = (float)read_u16_be(&frame->data[6]) / 10.0f,
    };
    update_feedback(actuator, &sample, decode_mode(status >> 6),
                    &faults, feedback_fault_raw,
                    extended_id_type(frame->identifier));
    if (updated_actuator != NULL) {
        *updated_actuator = actuator;
    }
    return RS01_RX_EVENT_FEEDBACK;
}

static rs01_rx_event_t process_private_fault(
    const twai_message_t *frame,
    exo_actuator_id_t *updated_actuator)
{
    const uint16_t data_field = extended_id_data(frame->identifier);
    exo_actuator_id_t actuator = EXO_ACTUATOR_COUNT;
    const bool source_known =
        rs01_find_actuator((uint8_t)data_field, &actuator);
    if (frame->data_length_code != RS01_FRAME_DATA_LENGTH ||
        extended_id_destination(frame->identifier) !=
            g_exo_hardware_config.twai.master_id ||
        !source_known) {
        return invalid_event(actuator, updated_actuator);
    }

    /*
     * Type-21 documents two uint32 values but omits their byte order. Use
     * the protocol's parameter-data little-endian convention; hardware fault
     * injection/capture must confirm this before detailed bits are certified.
     */
    const uint32_t fault = read_u32_le(&frame->data[0]);
    const uint32_t warning = read_u32_le(&frame->data[4]);
    update_fault(actuator, fault, warning);
    if (updated_actuator != NULL) {
        *updated_actuator = actuator;
    }
    ESP_LOGW(TAG, "actuator %s fault=0x%08lX warning=0x%08lX",
             actuator_name(actuator),
             (unsigned long)fault,
             (unsigned long)warning);
    return RS01_RX_EVENT_FAULT;
}

static rs01_rx_event_t process_private_parameter(
    const twai_message_t *frame,
    exo_actuator_id_t *updated_actuator)
{
    const uint16_t data_field = extended_id_data(frame->identifier);
    exo_actuator_id_t actuator = EXO_ACTUATOR_COUNT;
    const bool source_known =
        rs01_find_actuator((uint8_t)data_field, &actuator);
    const uint8_t response_status = (uint8_t)(data_field >> 8);
    if (frame->data_length_code != RS01_FRAME_DATA_LENGTH ||
        extended_id_destination(frame->identifier) !=
            g_exo_hardware_config.twai.master_id ||
        !source_known || response_status > 1U) {
        return invalid_event(actuator, updated_actuator);
    }

    const uint16_t index =
        (uint16_t)(frame->data[0] | ((uint16_t)frame->data[1] << 8));
    const uint32_t raw = read_u32_le(&frame->data[4]);
    float value;
    memcpy(&value, &raw, sizeof(value));
    ESP_LOGI(TAG,
             "actuator %s parameter index=0x%04X status=%u "
             "raw=0x%08lX uint=%lu float=%g",
             actuator_name(actuator), index, response_status,
             (unsigned long)raw, (unsigned long)raw, (double)value);
    if (updated_actuator != NULL) {
        *updated_actuator = actuator;
    }
    return RS01_RX_EVENT_PARAMETER;
}

static rs01_rx_event_t process_private_frame(
    const twai_message_t *frame,
    exo_actuator_id_t *updated_actuator)
{
    switch (extended_id_type(frame->identifier)) {
        case RS01_PRIVATE_TYPE_FEEDBACK:
        case RS01_PRIVATE_TYPE_REPORT:
            return process_private_feedback(frame, updated_actuator);
        case RS01_PRIVATE_TYPE_FAULT:
            return process_private_fault(frame, updated_actuator);
        case RS01_PRIVATE_TYPE_PARAMETER_READ:
        case RS01_PRIVATE_TYPE_PARAMETER_WRITE:
            return process_private_parameter(frame, updated_actuator);
        default:
            return RS01_RX_EVENT_NONE;
    }
}

static rs01_rx_event_t process_mit_feedback(
    const twai_message_t *frame,
    exo_actuator_id_t *updated_actuator)
{
    exo_actuator_id_t actuator = EXO_ACTUATOR_COUNT;
    const bool source_known = frame->data_length_code > 0U &&
                              rs01_find_actuator(frame->data[0], &actuator);
    if (frame->data_length_code != RS01_FRAME_DATA_LENGTH || !source_known) {
        return invalid_event(actuator, updated_actuator);
    }

    const uint16_t position_raw = read_u16_be(&frame->data[1]);
    const uint16_t velocity_raw =
        (uint16_t)(((uint16_t)frame->data[3] << 4) |
                   (frame->data[4] >> 4));
    const uint16_t torque_raw =
        (uint16_t)((((uint16_t)frame->data[4] & 0x0FU) << 8) |
                   frame->data[5]);
    const uint16_t temperature_raw =
        (uint16_t)((((uint16_t)frame->data[6] & 0x0FU) << 8) |
                   frame->data[7]);
    const uint8_t status = frame->data[6];

    exo_motion_sample_t sample = {
        .position_rad = raw_to_float(position_raw,
                                     s_limits.position_min_rad,
                                     s_limits.position_max_rad, UINT16_MAX),
        .velocity_rad_s = raw_to_float(velocity_raw,
                                       s_limits.velocity_min_rad_s,
                                       s_limits.velocity_max_rad_s, 0x0FFFU),
        .torque_nm = raw_to_float(torque_raw,
                                  s_limits.torque_min_nm,
                                  s_limits.torque_max_nm, 0x0FFFU),
        .temperature_c = (float)temperature_raw / 10.0f,
    };
    rs01_fault_flags_t faults = {0};
    faults.present = (status & (1U << 5)) != 0U;
    faults.warning_present = (status & (1U << 4)) != 0U;
    update_feedback(actuator, &sample, decode_mode(status >> 6),
                    &faults, 0U, 0U);
    if (updated_actuator != NULL) {
        *updated_actuator = actuator;
    }
    return RS01_RX_EVENT_FEEDBACK;
}

static rs01_rx_event_t process_mit_fault(
    const twai_message_t *frame,
    exo_actuator_id_t *updated_actuator)
{
    exo_actuator_id_t actuator = EXO_ACTUATOR_COUNT;
    const bool source_known = frame->data_length_code > 0U &&
                              rs01_find_actuator(frame->data[0], &actuator);
    if ((frame->data_length_code != RS01_MIT_MANUAL_FAULT_RESPONSE_LENGTH &&
         frame->data_length_code != RS01_MIT_LEGACY_FAULT_RESPONSE_LENGTH) ||
        !source_known) {
        return invalid_event(actuator, updated_actuator);
    }
    const uint32_t fault = read_u32_le(&frame->data[1]);
    update_fault(actuator, fault, 0U);
    set_mit_fault_response_pending(actuator, false);
    if (updated_actuator != NULL) {
        *updated_actuator = actuator;
    }
    ESP_LOGW(TAG, "actuator %s MIT fault=0x%08lX",
             actuator_name(actuator), (unsigned long)fault);
    return RS01_RX_EVENT_FAULT;
}

static rs01_rx_event_t process_mit_parameter(
    const twai_message_t *frame,
    exo_actuator_id_t actuator,
    uint8_t operation,
    exo_actuator_id_t *updated_actuator)
{
    if (frame->data_length_code != RS01_FRAME_DATA_LENGTH) {
        return invalid_event(actuator, updated_actuator);
    }
    const uint16_t index =
        (uint16_t)(frame->data[0] | ((uint16_t)frame->data[1] << 8));
    const uint32_t raw = read_u32_le(&frame->data[4]);
    float value;
    memcpy(&value, &raw, sizeof(value));
    ESP_LOGI(TAG,
             "actuator %s MIT parameter %s index=0x%04X "
             "raw=0x%08lX uint=%lu float=%g",
             actuator_name(actuator), operation == 3U ? "read" : "write",
             index, (unsigned long)raw, (unsigned long)raw, (double)value);
    if (updated_actuator != NULL) {
        *updated_actuator = actuator;
    }
    return RS01_RX_EVENT_PARAMETER;
}

static rs01_rx_event_t process_standard_frame(
    const twai_message_t *frame,
    exo_actuator_id_t *updated_actuator)
{
    if (frame->identifier == g_exo_hardware_config.twai.master_id) {
        if (frame->data_length_code == RS01_FRAME_DATA_LENGTH) {
            exo_actuator_id_t actuator = EXO_ACTUATOR_COUNT;
            if (rs01_find_actuator(frame->data[0], &actuator) &&
                mit_fault_response_is_pending(actuator) &&
                frame->data[5] == 0U && frame->data[6] == 0U &&
                frame->data[7] == 0U) {
                /*
                 * The manual defines an eight-byte fault response with the
                 * same host ID and source byte as normal feedback. A pending
                 * query plus its three documented-unused zero bytes supplies
                 * the otherwise absent discriminator without consuming an
                 * interleaved proactive feedback frame.
                 */
                return process_mit_fault(frame, updated_actuator);
            }
            return process_mit_feedback(frame, updated_actuator);
        }
        if (frame->data_length_code ==
            RS01_MIT_LEGACY_FAULT_RESPONSE_LENGTH) {
            /* Retain compatibility with firmware observed by the old driver. */
            return process_mit_fault(frame, updated_actuator);
        }
        exo_actuator_id_t actuator = EXO_ACTUATOR_COUNT;
        if (frame->data_length_code > 0U) {
            (void)rs01_find_actuator(frame->data[0], &actuator);
        }
        return invalid_event(actuator, updated_actuator);
    }

    const uint8_t operation = (uint8_t)((frame->identifier >> 8) & 0x07U);
    if (operation == 3U || operation == 4U) {
        exo_actuator_id_t actuator = EXO_ACTUATOR_COUNT;
        if (!rs01_find_actuator((uint8_t)frame->identifier, &actuator)) {
            return RS01_RX_EVENT_INVALID;
        }
        return process_mit_parameter(frame, actuator, operation,
                                     updated_actuator);
    }
    return RS01_RX_EVENT_NONE;
}

rs01_rx_event_t rs01_process_can_frame(
    const twai_message_t *frame,
    exo_actuator_id_t *updated_actuator)
{
    if (updated_actuator != NULL) {
        *updated_actuator = EXO_ACTUATOR_COUNT;
    }
    if (frame == NULL) {
        return RS01_RX_EVENT_INVALID;
    }

    if ((frame->flags & TWAI_MSG_FLAG_RTR) != 0U ||
        frame->data_length_code > RS01_FRAME_DATA_LENGTH) {
        exo_actuator_id_t actuator = EXO_ACTUATOR_COUNT;
        if ((frame->flags & TWAI_MSG_FLAG_EXTD) != 0U) {
            (void)rs01_find_actuator(
                (uint8_t)extended_id_data(frame->identifier), &actuator);
        } else if (frame->identifier ==
                       g_exo_hardware_config.twai.master_id &&
                   frame->data_length_code > 0U) {
            (void)rs01_find_actuator(frame->data[0], &actuator);
        } else {
            (void)rs01_find_actuator((uint8_t)frame->identifier,
                                     &actuator);
        }
        return invalid_event(actuator, updated_actuator);
    }

    if ((frame->flags & TWAI_MSG_FLAG_EXTD) != 0U) {
        return process_private_frame(frame, updated_actuator);
    }
    return process_standard_frame(frame, updated_actuator);
}
