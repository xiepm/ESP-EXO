#ifndef EXO_CONFIG_H
#define EXO_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/twai.h"
#include "exo_types.h"

typedef struct {
    gpio_num_t tx_gpio;
    gpio_num_t rx_gpio;
    uint32_t bitrate_bps;
    twai_timing_config_t timing;
    uint32_t rx_queue_length;
    exo_can_node_id_t master_id;
} exo_twai_config_t;

typedef struct {
    exo_can_node_id_t can_id;
} exo_actuator_config_t;

typedef struct {
    exo_side_t side;
    exo_actuator_id_t actuator;
    exo_direction_t direction;
    float software_zero_rad;
    bool mapping_valid;
    bool direction_valid;
    bool software_zero_valid;
} exo_hip_joint_config_t;

typedef struct {
    exo_twai_config_t twai;
    exo_actuator_config_t actuator[EXO_ACTUATOR_COUNT];
    exo_hip_joint_config_t hip_joint[EXO_HIP_JOINT_COUNT];
    exo_actuator_id_t default_actuator;
    uint32_t control_frequency_hz;
    uint32_t rs01_report_interval_ms;
    uint32_t rs01_feedback_timeout_ms;
    const char *ble_device_name;
} exo_hardware_config_t;

/* The only runtime-readable hardware configuration object. */
extern const exo_hardware_config_t g_exo_hardware_config;

#endif /* EXO_CONFIG_H */
