#ifndef EXO_TYPES_H
#define EXO_TYPES_H

#include <stdint.h>

/*
 * Firmware-domain types only. These types are not BLE/CAN wire formats and
 * must not be serialized with memcpy or compiler-specific packing.
 */

typedef uint8_t exo_can_node_id_t;

typedef enum {
    EXO_SIDE_UNKNOWN = 0,
    EXO_SIDE_LEFT = 1,
    EXO_SIDE_RIGHT = 2,
} exo_side_t;

/* Logical array slots. A/B do not imply a physical left/right assignment. */
typedef enum {
    EXO_ACTUATOR_A = 0,
    EXO_ACTUATOR_B = 1,
    EXO_ACTUATOR_COUNT = 2,
} exo_actuator_id_t;

/* Human-domain hip joints. Mapping to actuator A/B is explicit in config. */
typedef enum {
    EXO_HIP_JOINT_LEFT = 0,
    EXO_HIP_JOINT_RIGHT = 1,
    EXO_HIP_JOINT_COUNT = 2,
} exo_hip_joint_id_t;

/* Sign between motor-positive rotation and the future joint convention. */
typedef enum {
    EXO_DIRECTION_UNKNOWN = 0,
    EXO_DIRECTION_POSITIVE = 1,
    EXO_DIRECTION_NEGATIVE = -1,
} exo_direction_t;

/* Unit-explicit sample shared by future driver and joint layers. */
typedef struct {
    float position_rad;
    float velocity_rad_s;
    float torque_nm;
    float temperature_c;
} exo_motion_sample_t;

#if defined(__cplusplus)
static_assert(EXO_ACTUATOR_A == 0, "actuator A must index element 0");
static_assert(EXO_ACTUATOR_B == 1, "actuator B must index element 1");
static_assert(EXO_ACTUATOR_COUNT == 2, "phase 1 supports exactly two actuators");
static_assert(EXO_HIP_JOINT_LEFT == 0, "left hip must index element 0");
static_assert(EXO_HIP_JOINT_RIGHT == 1, "right hip must index element 1");
static_assert(EXO_HIP_JOINT_COUNT == 2, "exactly two hip joints are supported");
#else
_Static_assert(EXO_ACTUATOR_A == 0, "actuator A must index element 0");
_Static_assert(EXO_ACTUATOR_B == 1, "actuator B must index element 1");
_Static_assert(EXO_ACTUATOR_COUNT == 2, "phase 1 supports exactly two actuators");
_Static_assert(EXO_HIP_JOINT_LEFT == 0, "left hip must index element 0");
_Static_assert(EXO_HIP_JOINT_RIGHT == 1, "right hip must index element 1");
_Static_assert(EXO_HIP_JOINT_COUNT == 2, "exactly two hip joints are supported");
#endif

#endif /* EXO_TYPES_H */
