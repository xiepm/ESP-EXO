#include "exo_config.h"

#include "sdkconfig.h"

#define EXO_CONFIG_BLE_DEVICE_NAME "RobStride-CAN"
#define EXO_CONFIG_TWAI_BITRATE_BPS 1000000U
#define EXO_CONFIG_DEFAULT_ACTUATOR EXO_ACTUATOR_A

#if EXO_CONFIG_TWAI_BITRATE_BPS == 1000000U
#define EXO_CONFIG_TWAI_TIMING() TWAI_TIMING_CONFIG_1MBITS()
#else
#error "Add an ESP-IDF TWAI timing mapping for the configured bitrate"
#endif

enum {
    EXO_CONFIG_TWAI_RX_QUEUE_LENGTH = 32U,
    EXO_CONFIG_TWAI_MASTER_ID = 0xFD,
    EXO_CONFIG_ACTUATOR_A_CAN_ID = 0x10,
    EXO_CONFIG_ACTUATOR_B_CAN_ID = 0x20,
    EXO_CONFIG_CONTROL_FREQUENCY_HZ = 100U,
    EXO_CONFIG_MAX_MOTOR_CAN_ID = 0x7F,
    EXO_CONFIG_LEGACY_ADV_NAME_MAX_LENGTH = 23U,
};

_Static_assert(GPIO_IS_VALID_OUTPUT_GPIO(CONFIG_EXAMPLE_TX_GPIO_NUM),
               "TWAI TX GPIO must support output");
_Static_assert(GPIO_IS_VALID_GPIO(CONFIG_EXAMPLE_RX_GPIO_NUM),
               "TWAI RX GPIO must be valid");
_Static_assert(CONFIG_EXAMPLE_TX_GPIO_NUM != CONFIG_EXAMPLE_RX_GPIO_NUM,
               "TWAI TX and RX GPIOs must differ");
_Static_assert(EXO_CONFIG_ACTUATOR_A_CAN_ID != EXO_CONFIG_ACTUATOR_B_CAN_ID,
               "actuator CAN IDs must differ");
_Static_assert(EXO_CONFIG_ACTUATOR_A_CAN_ID <= EXO_CONFIG_MAX_MOTOR_CAN_ID,
               "actuator A CAN ID exceeds the state table");
_Static_assert(EXO_CONFIG_ACTUATOR_B_CAN_ID <= EXO_CONFIG_MAX_MOTOR_CAN_ID,
               "actuator B CAN ID exceeds the state table");
_Static_assert(EXO_CONFIG_DEFAULT_ACTUATOR >= EXO_ACTUATOR_A &&
                   EXO_CONFIG_DEFAULT_ACTUATOR < EXO_ACTUATOR_COUNT,
               "default actuator must index the actuator array");
_Static_assert(EXO_CONFIG_CONTROL_FREQUENCY_HZ > 0U,
               "control frequency must be positive");
_Static_assert((1000000U % EXO_CONFIG_CONTROL_FREQUENCY_HZ) == 0U,
               "control frequency must divide one second in microseconds");
_Static_assert((sizeof(EXO_CONFIG_BLE_DEVICE_NAME) - 1U) <=
                   EXO_CONFIG_LEGACY_ADV_NAME_MAX_LENGTH,
               "BLE device name does not fit legacy advertising data");

const exo_hardware_config_t g_exo_hardware_config = {
    .twai = {
        .tx_gpio = (gpio_num_t)CONFIG_EXAMPLE_TX_GPIO_NUM,
        .rx_gpio = (gpio_num_t)CONFIG_EXAMPLE_RX_GPIO_NUM,
        .bitrate_bps = EXO_CONFIG_TWAI_BITRATE_BPS,
        .timing = EXO_CONFIG_TWAI_TIMING(),
        .rx_queue_length = EXO_CONFIG_TWAI_RX_QUEUE_LENGTH,
        .master_id = EXO_CONFIG_TWAI_MASTER_ID,
    },
    .actuator = {
        [EXO_ACTUATOR_A] = {
            .can_id = EXO_CONFIG_ACTUATOR_A_CAN_ID,
            .physical_side = EXO_SIDE_UNKNOWN,
            .direction = EXO_DIRECTION_UNKNOWN,
            .software_zero_rad = 0.0f,
            .software_zero_valid = false,
        },
        [EXO_ACTUATOR_B] = {
            .can_id = EXO_CONFIG_ACTUATOR_B_CAN_ID,
            .physical_side = EXO_SIDE_UNKNOWN,
            .direction = EXO_DIRECTION_UNKNOWN,
            .software_zero_rad = 0.0f,
            .software_zero_valid = false,
        },
    },
    .default_actuator = EXO_CONFIG_DEFAULT_ACTUATOR,
    .control_frequency_hz = EXO_CONFIG_CONTROL_FREQUENCY_HZ,
    .ble_device_name = EXO_CONFIG_BLE_DEVICE_NAME,
};
