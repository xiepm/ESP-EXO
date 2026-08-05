#ifndef __ROBSTRIDE_CONTROL_H__
#define __ROBSTRIDE_CONTROL_H__

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/* ---- Default IDs (commissioned motor uses CAN ID 0x10) ---- */
#define DEFAULT_CAN_ID    0x10
#define MASTER_ID         0xFD

/* ---- Private Protocol Command Codes (mode field in ExtId) ---- */
#define ANNOUNCE_DEVID        0
#define MOTOR_CTRL            0x01
#define MOTOR_FEEDBACK        0x02
#define MOTOR_IN              0x03
#define MOTOR_RESET           0x04
#define MOTOR_CALI            0x05
#define MOTOR_ZERO            0x06
#define MOTOR_ID              0x07
#define MOTOR_PARAREAD        0x11
#define MOTOR_PARAWRITE       0x12
#define MOTOR_ErrorFeedback   0x15
#define MOTOR_DataSave        0x16
#define MOTOR_BAUD            0x17
#define MOTOR_Proactive       0x18
#define MOTOR_MODESTE         0x19

/* ---- Motor Type Selection ---- */
#define MOTOR_TYPE 5

#define P_MIN  -12.57f
#define P_MAX   12.57f

#if MOTOR_TYPE == 0
    #define V_MIN  -33.0f
    #define V_MAX   33.0f
    #define KP_MIN   0.0f
    #define KP_MAX 500.0f
    #define KD_MIN   0.0f
    #define KD_MAX   5.0f
    #define T_MIN  -14.0f
    #define T_MAX   14.0f
#elif MOTOR_TYPE == 1
    #define V_MIN  -44.0f
    #define V_MAX   44.0f
    #define KP_MIN   0.0f
    #define KP_MAX 500.0f
    #define KD_MIN   0.0f
    #define KD_MAX   5.0f
    #define T_MIN  -17.0f
    #define T_MAX   17.0f
#elif MOTOR_TYPE == 2
    #define V_MIN  -44.0f
    #define V_MAX   44.0f
    #define KP_MIN   0.0f
    #define KP_MAX 500.0f
    #define KD_MIN   0.0f
    #define KD_MAX   5.0f
    #define T_MIN  -17.0f
    #define T_MAX   17.0f
#elif MOTOR_TYPE == 3
    #define V_MIN  -20.0f
    #define V_MAX   20.0f
    #define KP_MIN   0.0f
    #define KP_MAX 5000.0f
    #define KD_MIN   0.0f
    #define KD_MAX 100.0f
    #define T_MIN  -60.0f
    #define T_MAX   60.0f
#elif MOTOR_TYPE == 4
    #define V_MIN  -15.0f
    #define V_MAX   15.0f
    #define KP_MIN   0.0f
    #define KP_MAX 5000.0f
    #define KD_MIN   0.0f
    #define KD_MAX 100.0f
    #define T_MIN -120.0f
    #define T_MAX  120.0f
#elif MOTOR_TYPE == 5
    #define V_MIN  -50.0f
    #define V_MAX   50.0f
    #define KP_MIN   0.0f
    #define KP_MAX 500.0f
    #define KD_MIN   0.0f
    #define KD_MAX   5.0f
    #define T_MIN  -5.5f
    #define T_MAX   5.5f
#elif MOTOR_TYPE == 6
    #define V_MIN  -50.0f
    #define V_MAX   50.0f
    #define KP_MIN   0.0f
    #define KP_MAX 5000.0f
    #define KD_MIN   0.0f
    #define KD_MAX 100.0f
    #define T_MIN  -36.0f
    #define T_MAX   36.0f
#endif

#define Set_mode       'j'
#define Set_parameter  'p'

#define MOVE_CONTROL_MODE       0
#define POS_PP_CONTROL_MODE     1
#define SPEED_CONTROL_MODE      2
#define ELECT_CONTROL_MODE      3
#define SET_ZERO_MODE           4
#define POS_CSP_CONTROL_MODE    5

enum ModeStatus {
    REST = 0,
    CALI,
    RUNNING,
};

typedef struct {
    uint8_t uncalibrated;
    uint8_t StallOverloadFault;
    uint8_t MagneticEncoderFault;
    uint8_t OverTemperatureFault;
    uint8_t DriveFault;
    uint8_t UnderVoltageFault;
    enum ModeStatus mms;
    float angle;
    float speed;
    float torque;
    float temperature;
    uint8_t motorMode;
} motor_feedback_t;

extern motor_feedback_t g_motor_feedback;
extern uint8_t g_tx_data[8];
extern uint8_t g_rx_data[8];

/* ---- Per-motor state for dual-motor control ---- */
#define MAX_MOTOR_ID    0x7F

typedef struct {
    bool  valid;         /* true after at least one feedback received */
    float position;      /* rad */
    float velocity;      /* rad/s */
    float torque;        /* N·m */
    float temperature;   /* ℃ */
} motor_state_simple_t;

extern motor_state_simple_t g_motor_states[MAX_MOTOR_ID + 1];

void robstride_update_motor_state(uint8_t motor_id, float position,
                                  float velocity, float torque,
                                  float temperature);
void robstride_clear_motor_states(void);

esp_err_t robstride_control_init(void);
void robstride_get_feedback_snapshot(motor_feedback_t *feedback);

/* ---- Helper: construct 29-bit Extended ID from fields ---- */
static inline uint32_t make_ext_id(uint8_t id, uint16_t data, uint8_t mode)
{
    return ((uint32_t)(id & 0xFF))
         | (((uint32_t)(data & 0xFFFF)) << 8)
         | (((uint32_t)(mode & 0x1F)) << 24);
}

/* ---- Helper: extract fields from received Extended ID ---- */
static inline uint8_t  ext_id_get_id(uint32_t eid)   { return (uint8_t)(eid & 0xFF); }
static inline uint16_t ext_id_get_data(uint32_t eid) { return (uint16_t)((eid >> 8) & 0xFFFF); }
static inline uint8_t  ext_id_get_mode(uint32_t eid) { return (uint8_t)((eid >> 24) & 0x1F); }

/* ================================================================
 *  Private Protocol API  (motor_id = target motor's CAN ID)
 * ================================================================ */

void RobStride_motor_enable(uint8_t motor_id);
void RobStride_motor_reset(uint8_t motor_id);
void Set_RobStride_Motor_parameter(uint8_t motor_id, uint16_t Index, float Value, char Value_mode);
void Get_RobStride_Motor_parameter(uint8_t motor_id, uint16_t Index);
void RobStrideMotor_move_control(uint8_t motor_id, float torque, float MechPosition, float speed, float kp, float kd);
void RobStride_Impedance_SendFrame(uint8_t motor_id, float MechPosition, float speed, float kp, float kd, float torque);
void RobStride_Motor_Speed_control(uint8_t motor_id, float limit_cur, float Speed_acc, float Speed);
void RobStride_Motor_Pos_PP_control(uint8_t motor_id, float Speed, float Speed_acc, float Angle);
void RobStride_Motor_Pos_CSP_control(uint8_t motor_id, float Speed, float Angle);
void RobStride_Motor_current_control(uint8_t motor_id, float cuttent);
void RobStride_Set_ZreoPos(uint8_t motor_id);
void RobStride_Set_CAN_ID(uint8_t motor_id, uint8_t new_id);
void RobStride_Motor_MotorDataSave(uint8_t motor_id);
void RobStride_Motor_BaudRateChange(uint8_t motor_id, uint8_t F_CMD);
void RobStride_Motor_ProtactiveEscalationSet(uint8_t motor_id, uint8_t F_CMD);
void RobStride_Motor_MIT_ModeSet(uint8_t motor_id, uint8_t F_CMD);

/* ================================================================
 *  MIT Protocol API  (motor_id = target motor's CAN ID)
 * ================================================================ */

void RobStride_Motor_MIT_enable(uint8_t motor_id);
void RobStride_Motor_MIT_reset(uint8_t motor_id);
void RobStride_Motor_MIT_ClearOrCheckError(uint8_t motor_id, uint8_t F_CMD);
void RobStride_Motor_MIT_SetMotorType(uint8_t motor_id, uint8_t F_CMD);
void RobStride_Motor_MIT_SetMotorId(uint8_t motor_id, uint8_t new_id);
void RobStride_Motor_MIT_Control(uint8_t motor_id, float Angle, float Speed, float Kp, float Kd, float Torque);
void RobStride_Motor_MIT_PositionControl(uint8_t motor_id, float position_rad, float speed_rad_per_s);
void RobStride_Motor_MIT_SpeedControl(uint8_t motor_id, float speed_rad_per_s, float current_limit);
void RobStride_Motor_MIT_SetZeroPos(uint8_t motor_id);
void RobStride_Motor_MIT_MotorDataSave(uint8_t motor_id);
void RobStride_Motor_MIT_ProtactiveEscalationSet(uint8_t motor_id, uint8_t F_CMD);
void Get_RobStride_Motor_MIT_parameter(uint8_t motor_id, uint16_t Index);
void Set_RobStride_Motor_MIT_parameter(uint8_t motor_id, uint16_t Index, float Value);

/* ---- Feedback Parser ---- */
void parse_motor_feedback(bool is_extended, uint32_t identifier, const uint8_t rx_data[8]);
void robstride_update_mit_faults(uint32_t fault_value);

#endif /* __ROBSTRIDE_CONTROL_H__ */
