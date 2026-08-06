#ifndef RS01_DRIVER_H
#define RS01_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

#include "driver/twai.h"
#include "esp_err.h"
#include "exo_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    RS01_MODE_STATUS_RESET = 0,
    RS01_MODE_STATUS_CALIBRATION = 1,
    RS01_MODE_STATUS_RUNNING = 2,
    RS01_MODE_STATUS_UNKNOWN = 3,
} rs01_mode_status_t;

typedef enum {
    RS01_CONTROL_MODE_MOTION = 0,
    RS01_CONTROL_MODE_POSITION_PP = 1,
    RS01_CONTROL_MODE_VELOCITY = 2,
    RS01_CONTROL_MODE_CURRENT = 3,
    RS01_CONTROL_MODE_POSITION_CSP = 5,
    RS01_CONTROL_MODE_UNKNOWN = 0xFF,
} rs01_control_mode_t;

typedef enum {
    RS01_PROTOCOL_PRIVATE = 0,
    RS01_PROTOCOL_CANOPEN = 1,
    RS01_PROTOCOL_MIT = 2,
} rs01_protocol_t;

typedef struct {
    bool present;
    bool warning_present;
    bool uncalibrated;
    bool stall_overload;
    bool magnetic_encoder;
    bool over_temperature;
    bool phase_over_current;
    bool under_voltage;
    bool over_voltage;
    bool driver_chip;
    bool position_initialization;
    bool hardware_identification;
    bool over_temperature_warning;
} rs01_fault_flags_t;

typedef struct {
    exo_motion_sample_t sample;
    rs01_mode_status_t mode;
    rs01_fault_flags_t faults;

    /* Type-21/MIT fault response values use the manual's native bit layout. */
    uint32_t fault_raw;
    uint32_t warning_raw;
    /* Type-2/type-24 feedback carries a separate six-bit fault summary. */
    uint8_t feedback_fault_raw;

    uint64_t timestamp_us;
    uint64_t first_timestamp_us;
    uint64_t last_interval_us;
    uint32_t feedback_count;
    uint32_t type2_feedback_count;
    uint32_t type24_feedback_count;
    uint32_t invalid_frame_count;

    bool valid;
    bool online;
    bool enabled_requested;
    bool reporting_requested;
    rs01_control_mode_t control_mode;
} rs01_feedback_snapshot_t;

typedef struct {
    float position_min_rad;
    float position_max_rad;
    float velocity_min_rad_s;
    float velocity_max_rad_s;
    float torque_min_nm;
    float torque_max_nm;
    float kp_min;
    float kp_max;
    float kd_min;
    float kd_max;
} rs01_limits_t;

typedef enum {
    RS01_RX_EVENT_NONE = 0,
    RS01_RX_EVENT_FEEDBACK,
    RS01_RX_EVENT_FAULT,
    RS01_RX_EVENT_PARAMETER,
    RS01_RX_EVENT_INVALID,
} rs01_rx_event_t;

esp_err_t rs01_driver_init(void);

const rs01_limits_t *rs01_get_limits(void);
bool rs01_get_can_id(exo_actuator_id_t actuator,
                     exo_can_node_id_t *can_id);
bool rs01_find_actuator(exo_can_node_id_t can_id,
                        exo_actuator_id_t *actuator);

esp_err_t rs01_enable(exo_actuator_id_t actuator);
esp_err_t rs01_disable(exo_actuator_id_t actuator);
esp_err_t rs01_clear_faults(exo_actuator_id_t actuator);

/* Sends one private-protocol type-1 frame without changing mode or enable. */
esp_err_t rs01_send_motion(exo_actuator_id_t actuator,
                           float position_rad,
                           float velocity_rad_s,
                           float torque_nm,
                           float kp,
                           float kd);

/* Preserves the legacy console's disable/mode/enable/motion compound action. */
esp_err_t rs01_move(exo_actuator_id_t actuator,
                    float torque_nm,
                    float position_rad,
                    float velocity_rad_s,
                    float kp,
                    float kd);

esp_err_t rs01_set_control_mode(exo_actuator_id_t actuator,
                                rs01_control_mode_t mode);
esp_err_t rs01_speed_control(exo_actuator_id_t actuator,
                             float current_limit_a,
                             float acceleration_rad_s2,
                             float velocity_rad_s);
esp_err_t rs01_position_pp_control(exo_actuator_id_t actuator,
                                   float velocity_rad_s,
                                   float acceleration_rad_s2,
                                   float position_rad);
esp_err_t rs01_position_csp_control(exo_actuator_id_t actuator,
                                    float velocity_limit_rad_s,
                                    float position_rad);
esp_err_t rs01_current_control(exo_actuator_id_t actuator,
                               float current_a);

esp_err_t rs01_read_parameter(exo_actuator_id_t actuator, uint16_t index);
esp_err_t rs01_write_parameter_float(exo_actuator_id_t actuator,
                                      uint16_t index,
                                      float value);
esp_err_t rs01_write_parameter_u8(exo_actuator_id_t actuator,
                                   uint16_t index,
                                   uint8_t value);
esp_err_t rs01_write_parameter_u16(exo_actuator_id_t actuator,
                                    uint16_t index,
                                    uint16_t value);

/* 10 ms maps to 1; each additional 5 ms increments the 0x7026 value by 1. */
esp_err_t rs01_set_report_interval_ms(exo_actuator_id_t actuator,
                                      uint32_t interval_ms);
/* These only send the type-24 switch; they do not write the interval. */
esp_err_t rs01_start_reporting(exo_actuator_id_t actuator);
esp_err_t rs01_stop_reporting(exo_actuator_id_t actuator);

esp_err_t rs01_set_mechanical_zero(exo_actuator_id_t actuator);
esp_err_t rs01_set_can_id(exo_actuator_id_t actuator,
                          exo_can_node_id_t new_can_id);
esp_err_t rs01_save_parameters(exo_actuator_id_t actuator);
esp_err_t rs01_set_baud_rate(exo_actuator_id_t actuator,
                             uint8_t baud_selector);
esp_err_t rs01_set_protocol(exo_actuator_id_t actuator,
                            rs01_protocol_t protocol);

esp_err_t rs01_mit_enable(exo_actuator_id_t actuator);
esp_err_t rs01_mit_disable(exo_actuator_id_t actuator);
esp_err_t rs01_mit_clear_or_read_fault(exo_actuator_id_t actuator,
                                        uint8_t command);
esp_err_t rs01_mit_set_operation_mode(exo_actuator_id_t actuator,
                                       uint8_t mode);
esp_err_t rs01_mit_set_can_id(exo_actuator_id_t actuator,
                              exo_can_node_id_t new_can_id);
esp_err_t rs01_mit_set_protocol(exo_actuator_id_t actuator,
                                rs01_protocol_t protocol);
esp_err_t rs01_mit_send_motion(exo_actuator_id_t actuator,
                               float position_rad,
                               float velocity_rad_s,
                               float kp,
                               float kd,
                               float torque_nm);
esp_err_t rs01_mit_position_control(exo_actuator_id_t actuator,
                                    float position_rad,
                                    float velocity_rad_s);
esp_err_t rs01_mit_speed_control(exo_actuator_id_t actuator,
                                 float velocity_rad_s,
                                 float current_limit_a);
esp_err_t rs01_mit_set_zero(exo_actuator_id_t actuator);
esp_err_t rs01_mit_save_parameters(exo_actuator_id_t actuator);
esp_err_t rs01_mit_start_reporting(exo_actuator_id_t actuator);
esp_err_t rs01_mit_stop_reporting(exo_actuator_id_t actuator);
esp_err_t rs01_mit_read_parameter(exo_actuator_id_t actuator,
                                  uint16_t index);
esp_err_t rs01_mit_write_parameter_float(exo_actuator_id_t actuator,
                                          uint16_t index,
                                          float value);

bool rs01_get_feedback(exo_actuator_id_t actuator,
                       rs01_feedback_snapshot_t *snapshot);

/*
 * Processes one complete TWAI frame. On events attributed to a configured
 * motor, updated_actuator receives A or B; otherwise it receives COUNT.
 */
rs01_rx_event_t rs01_process_can_frame(
    const twai_message_t *frame,
    exo_actuator_id_t *updated_actuator);

#ifdef __cplusplus
}
#endif

#endif /* RS01_DRIVER_H */
