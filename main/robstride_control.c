#include "robstride_control.h"
#include "driver/twai.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "robstride";

/* ---- Global motor feedback ---- */
motor_feedback_t g_motor_feedback;

/* ---- Global TX/RX data buffers ---- */
uint8_t g_tx_data[8];
uint8_t g_rx_data[8];

/* ---- These globals carry the identifiers for the next extended/std frame ---- */
static uint32_t s_ext_id = 0;
static uint32_t s_std_id = 0;
static StaticSemaphore_t s_tx_mutex_storage;
static SemaphoreHandle_t s_tx_mutex;
static portMUX_TYPE s_feedback_lock = portMUX_INITIALIZER_UNLOCKED;

#define TX_LOCK()                                                               \
    do {                                                                        \
        if (s_tx_mutex != NULL) {                                               \
            xSemaphoreTakeRecursive(s_tx_mutex, portMAX_DELAY);                 \
        }                                                                       \
    } while (0)

#define TX_UNLOCK()                                                             \
    do {                                                                        \
        if (s_tx_mutex != NULL) {                                               \
            xSemaphoreGiveRecursive(s_tx_mutex);                                \
        }                                                                       \
    } while (0)

/* ================================================================
 *  Internal helpers
 * ================================================================ */

static int float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    if (x > x_max) x = x_max;
    else if (x < x_min) x = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

static float uint16_to_float(uint16_t x, float min, float max)
{
    return ((float)x) / 65535.0f * (max - min) + min;
}

static float uint12_to_float(uint16_t x, float min, float max)
{
    return ((float)(x & 0x0FFF)) / 4095.0f * (max - min) + min;
}

esp_err_t robstride_control_init(void)
{
    if (s_tx_mutex == NULL) {
        s_tx_mutex = xSemaphoreCreateRecursiveMutexStatic(&s_tx_mutex_storage);
    }
    return s_tx_mutex != NULL ? ESP_OK : ESP_ERR_NO_MEM;
}

void robstride_get_feedback_snapshot(motor_feedback_t *feedback)
{
    if (feedback == NULL) {
        return;
    }

    portENTER_CRITICAL(&s_feedback_lock);
    *feedback = g_motor_feedback;
    portEXIT_CRITICAL(&s_feedback_lock);
}

static void twai_send_ext(void)
{
    twai_message_t msg = {0};
    msg.flags = TWAI_MSG_FLAG_EXTD;
    msg.identifier = s_ext_id;
    msg.data_length_code = 8;
    memcpy(msg.data, g_tx_data, 8);
    esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(10));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "ext TX failed: %s", esp_err_to_name(err));
    }
}

static void twai_send_std(void)
{
    twai_message_t msg = {0};
    msg.flags = TWAI_MSG_FLAG_NONE;
    msg.identifier = s_std_id;
    msg.data_length_code = 8;
    memcpy(msg.data, g_tx_data, 8);
    esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(10));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "std TX failed: %s", esp_err_to_name(err));
    }
}

/* ================================================================
 *  Private Protocol Implementation
 * ================================================================ */

void RobStride_motor_enable(uint8_t motor_id)
{
    TX_LOCK();
    s_ext_id = make_ext_id(motor_id, MASTER_ID, MOTOR_IN);
    memset(g_tx_data, 0, 8);
    twai_send_ext();
    TX_UNLOCK();
}

void RobStride_motor_reset(uint8_t motor_id)
{
    TX_LOCK();
    s_ext_id = make_ext_id(motor_id, MASTER_ID, MOTOR_RESET);
    memset(g_tx_data, 0, 8);
    twai_send_ext();
    TX_UNLOCK();
}

void Set_RobStride_Motor_parameter(uint8_t motor_id, uint16_t Index, float Value, char Value_mode)
{
    TX_LOCK();
    s_ext_id = make_ext_id(motor_id, MASTER_ID, MOTOR_PARAWRITE);
    memset(g_tx_data, 0, 8);
    g_tx_data[0] = Index & 0xFF;
    g_tx_data[1] = (Index >> 8) & 0xFF;

    if (Value_mode == 'p') {
        memcpy(&g_tx_data[4], &Value, 4);
    } else if (Value_mode == 'j') {
        portENTER_CRITICAL(&s_feedback_lock);
        g_motor_feedback.motorMode = MOVE_CONTROL_MODE;
        portEXIT_CRITICAL(&s_feedback_lock);
        g_tx_data[4] = (uint8_t)Value;
    } else {
        ESP_LOGE(TAG, "invalid parameter value mode: %d", Value_mode);
        TX_UNLOCK();
        return;
    }
    twai_send_ext();
    vTaskDelay(pdMS_TO_TICKS(1));
    TX_UNLOCK();
}

void Get_RobStride_Motor_parameter(uint8_t motor_id, uint16_t Index)
{
    TX_LOCK();
    s_ext_id = make_ext_id(motor_id, MASTER_ID, MOTOR_PARAREAD);
    memset(g_tx_data, 0, 8);
    g_tx_data[0] = Index & 0xFF;
    g_tx_data[1] = (Index >> 8) & 0xFF;
    twai_send_ext();
    TX_UNLOCK();
}

void RobStrideMotor_move_control(uint8_t motor_id, float torque, float MechPosition, float speed, float kp, float kd)
{
    TX_LOCK();
    motor_feedback_t feedback;
    robstride_get_feedback_snapshot(&feedback);
    if (feedback.mms == RUNNING) {
        RobStride_motor_reset(motor_id);
    }
    Set_RobStride_Motor_parameter(motor_id, 0x7005, MOVE_CONTROL_MODE, Set_mode);
    RobStride_motor_enable(motor_id);

    s_ext_id = make_ext_id(motor_id,
        (uint16_t)float_to_uint(torque, T_MIN, T_MAX, 16), MOTOR_CTRL);

    g_tx_data[0] = float_to_uint(MechPosition, P_MIN, P_MAX, 16) >> 8;
    g_tx_data[1] = float_to_uint(MechPosition, P_MIN, P_MAX, 16) & 0xFF;
    g_tx_data[2] = float_to_uint(speed, V_MIN, V_MAX, 16) >> 8;
    g_tx_data[3] = float_to_uint(speed, V_MIN, V_MAX, 16) & 0xFF;
    g_tx_data[4] = float_to_uint(kp, KP_MIN, KP_MAX, 16) >> 8;
    g_tx_data[5] = float_to_uint(kp, KP_MIN, KP_MAX, 16) & 0xFF;
    g_tx_data[6] = float_to_uint(kd, KD_MIN, KD_MAX, 16) >> 8;
    g_tx_data[7] = float_to_uint(kd, KD_MIN, KD_MAX, 16) & 0xFF;
    twai_send_ext();
    TX_UNLOCK();
}

void RobStride_Motor_Speed_control(uint8_t motor_id, float limit_cur, float Speed_acc, float Speed)
{
    TX_LOCK();
    motor_feedback_t feedback;
    robstride_get_feedback_snapshot(&feedback);
    if (feedback.mms == RUNNING) {
        RobStride_motor_reset(motor_id);
    }
    Set_RobStride_Motor_parameter(motor_id, 0x7005, SPEED_CONTROL_MODE, Set_mode);
    RobStride_motor_enable(motor_id);

    Set_RobStride_Motor_parameter(motor_id, 0x7018, limit_cur, Set_parameter);
    Set_RobStride_Motor_parameter(motor_id, 0x7022, Speed_acc, Set_parameter);
    Set_RobStride_Motor_parameter(motor_id, 0x700A, Speed, Set_parameter);
    TX_UNLOCK();
}

void RobStride_Motor_Pos_PP_control(uint8_t motor_id, float Speed, float Speed_acc, float Angle)
{
    TX_LOCK();
    motor_feedback_t feedback;
    robstride_get_feedback_snapshot(&feedback);
    if (feedback.mms == RUNNING) {
        RobStride_motor_reset(motor_id);
    }
    Set_RobStride_Motor_parameter(motor_id, 0x7005, POS_PP_CONTROL_MODE, Set_mode);
    RobStride_motor_enable(motor_id);

    Set_RobStride_Motor_parameter(motor_id, 0x7024, Speed, Set_parameter);
    Set_RobStride_Motor_parameter(motor_id, 0x7025, Speed_acc, Set_parameter);
    Set_RobStride_Motor_parameter(motor_id, 0x7016, Angle, Set_parameter);
    TX_UNLOCK();
}

void RobStride_Motor_Pos_CSP_control(uint8_t motor_id, float Speed, float Angle)
{
    TX_LOCK();
    motor_feedback_t feedback;
    robstride_get_feedback_snapshot(&feedback);
    if (feedback.mms == RUNNING) {
        RobStride_motor_reset(motor_id);
    }
    Set_RobStride_Motor_parameter(motor_id, 0x7005, POS_CSP_CONTROL_MODE, Set_mode);
    RobStride_motor_enable(motor_id);

    Set_RobStride_Motor_parameter(motor_id, 0x7017, Speed, Set_parameter);
    Set_RobStride_Motor_parameter(motor_id, 0x7016, Angle, Set_parameter);
    TX_UNLOCK();
}

void RobStride_Motor_current_control(uint8_t motor_id, float cuttent)
{
    TX_LOCK();
    motor_feedback_t feedback;
    robstride_get_feedback_snapshot(&feedback);
    if (feedback.mms == RUNNING) {
        RobStride_motor_reset(motor_id);
    }
    Set_RobStride_Motor_parameter(motor_id, 0x7005, ELECT_CONTROL_MODE, Set_mode);
    RobStride_motor_enable(motor_id);

    Set_RobStride_Motor_parameter(motor_id, 0x7006, cuttent, Set_parameter);
    TX_UNLOCK();
}

void RobStride_Set_ZreoPos(uint8_t motor_id)
{
    TX_LOCK();
    RobStride_motor_reset(motor_id);
    s_ext_id = make_ext_id(motor_id, MASTER_ID, MOTOR_ZERO);
    memset(g_tx_data, 0, 8);
    g_tx_data[0] = 1;
    twai_send_ext();
    TX_UNLOCK();
}

void RobStride_Set_CAN_ID(uint8_t motor_id, uint8_t new_id)
{
    TX_LOCK();
    RobStride_motor_reset(motor_id);
    s_ext_id = make_ext_id(motor_id, (uint16_t)(new_id << 8) | MASTER_ID, MOTOR_ID);
    memset(g_tx_data, 0, 8);
    twai_send_ext();
    TX_UNLOCK();
}

void RobStride_Motor_MotorDataSave(uint8_t motor_id)
{
    TX_LOCK();
    s_ext_id = make_ext_id(motor_id, MASTER_ID, MOTOR_DataSave);
    g_tx_data[0] = 0x01;
    g_tx_data[1] = 0x02;
    g_tx_data[2] = 0x03;
    g_tx_data[3] = 0x04;
    g_tx_data[4] = 0x05;
    g_tx_data[5] = 0x06;
    g_tx_data[6] = 0x07;
    g_tx_data[7] = 0x08;
    twai_send_ext();
    TX_UNLOCK();
}

void RobStride_Motor_BaudRateChange(uint8_t motor_id, uint8_t F_CMD)
{
    TX_LOCK();
    s_ext_id = make_ext_id(motor_id, MASTER_ID, MOTOR_BAUD);
    g_tx_data[0] = 0x01;
    g_tx_data[1] = 0x02;
    g_tx_data[2] = 0x03;
    g_tx_data[3] = 0x04;
    g_tx_data[4] = 0x05;
    g_tx_data[5] = 0x06;
    g_tx_data[6] = F_CMD;
    g_tx_data[7] = 0x08;
    twai_send_ext();
    TX_UNLOCK();
}

void RobStride_Motor_ProtactiveEscalationSet(uint8_t motor_id, uint8_t F_CMD)
{
    TX_LOCK();
    s_ext_id = make_ext_id(motor_id, MASTER_ID, MOTOR_Proactive);
    g_tx_data[0] = 0x01;
    g_tx_data[1] = 0x02;
    g_tx_data[2] = 0x03;
    g_tx_data[3] = 0x04;
    g_tx_data[4] = 0x05;
    g_tx_data[5] = 0x06;
    g_tx_data[6] = F_CMD;
    g_tx_data[7] = 0x08;
    twai_send_ext();
    TX_UNLOCK();
}

void RobStride_Motor_MIT_ModeSet(uint8_t motor_id, uint8_t F_CMD)
{
    TX_LOCK();
    if (F_CMD == 0) {
        /* MIT → proprietary: send standard frame with 0xFF...0xFF FD */
        s_std_id = motor_id;
        memset(g_tx_data, 0xFF, 6);
        g_tx_data[6] = F_CMD;
        g_tx_data[7] = 0xFD;
        twai_send_std();
    } else if (F_CMD == 2) {
        /* Proprietary → MIT: send extended frame mode MOTOR_MODESTE */
        s_ext_id = make_ext_id(motor_id, MASTER_ID, MOTOR_MODESTE);
        g_tx_data[0] = 0x01;
        g_tx_data[1] = 0x02;
        g_tx_data[2] = 0x03;
        g_tx_data[3] = 0x04;
        g_tx_data[4] = 0x05;
        g_tx_data[5] = 0x06;
        g_tx_data[6] = F_CMD;
        g_tx_data[7] = 0x08;
        twai_send_ext();
    } else {
        ESP_LOGE(TAG, "unsupported protocol mode: %u", F_CMD);
    }
    TX_UNLOCK();
}

/* ================================================================
 *  MIT Protocol Implementation
 * ================================================================ */

void RobStride_Motor_MIT_enable(uint8_t motor_id)
{
    TX_LOCK();
    s_std_id = motor_id;
    memset(g_tx_data, 0xFF, 7);
    g_tx_data[7] = 0xFC;
    twai_send_std();
    portENTER_CRITICAL(&s_feedback_lock);
    g_motor_feedback.mms = RUNNING;
    portEXIT_CRITICAL(&s_feedback_lock);
    TX_UNLOCK();
}

void RobStride_Motor_MIT_reset(uint8_t motor_id)
{
    TX_LOCK();
    s_std_id = motor_id;
    memset(g_tx_data, 0xFF, 7);
    g_tx_data[7] = 0xFD;
    twai_send_std();
    portENTER_CRITICAL(&s_feedback_lock);
    g_motor_feedback.mms = REST;
    portEXIT_CRITICAL(&s_feedback_lock);
    TX_UNLOCK();
}

void RobStride_Motor_MIT_ClearOrCheckError(uint8_t motor_id, uint8_t F_CMD)
{
    TX_LOCK();
    s_std_id = motor_id;
    memset(g_tx_data, 0xFF, 6);
    g_tx_data[6] = F_CMD;
    g_tx_data[7] = 0xFB;
    twai_send_std();
    TX_UNLOCK();
}

void RobStride_Motor_MIT_SetMotorType(uint8_t motor_id, uint8_t F_CMD)
{
    TX_LOCK();
    s_std_id = motor_id;
    memset(g_tx_data, 0xFF, 6);
    g_tx_data[6] = F_CMD;
    g_tx_data[7] = 0xFC;
    twai_send_std();
    TX_UNLOCK();
}

void RobStride_Motor_MIT_SetMotorId(uint8_t motor_id, uint8_t new_id)
{
    TX_LOCK();
    s_std_id = motor_id;
    memset(g_tx_data, 0xFF, 6);
    g_tx_data[6] = new_id;
    g_tx_data[7] = 0xFA;
    twai_send_std();
    TX_UNLOCK();
}

void RobStride_Motor_MIT_Control(uint8_t motor_id, float Angle, float Speed, float Kp, float Kd, float Torque)
{
    TX_LOCK();
    s_std_id = motor_id;
    g_tx_data[0] = float_to_uint(Angle, P_MIN, P_MAX, 16) >> 8;
    g_tx_data[1] = float_to_uint(Angle, P_MIN, P_MAX, 16) & 0xFF;
    g_tx_data[2] = float_to_uint(Speed, V_MIN, V_MAX, 12) >> 4;
    g_tx_data[3] = ((float_to_uint(Speed, V_MIN, V_MAX, 12) << 4) & 0xF0)
                 | (float_to_uint(Kp, KP_MIN, KP_MAX, 12) >> 8);
    g_tx_data[4] = float_to_uint(Kp, KP_MIN, KP_MAX, 12) & 0xFF;
    g_tx_data[5] = float_to_uint(Kd, KD_MIN, KD_MAX, 12) >> 4;
    g_tx_data[6] = ((float_to_uint(Kd, KD_MIN, KD_MAX, 12) << 4) & 0xF0)
                 | (float_to_uint(Torque, T_MIN, T_MAX, 12) >> 8);
    g_tx_data[7] = float_to_uint(Torque, T_MIN, T_MAX, 12) & 0xFF;
    twai_send_std();
    TX_UNLOCK();
}

void RobStride_Motor_MIT_PositionControl(uint8_t motor_id, float position_rad, float speed_rad_per_s)
{
    TX_LOCK();
    s_std_id = (1 << 8) | motor_id;
    memcpy(&g_tx_data[0], &position_rad, 4);
    memcpy(&g_tx_data[4], &speed_rad_per_s, 4);
    twai_send_std();
    TX_UNLOCK();
}

void RobStride_Motor_MIT_SpeedControl(uint8_t motor_id, float speed_rad_per_s, float current_limit)
{
    TX_LOCK();
    s_std_id = (2 << 8) | motor_id;
    memcpy(&g_tx_data[0], &speed_rad_per_s, 4);
    memcpy(&g_tx_data[4], &current_limit, 4);
    twai_send_std();
    TX_UNLOCK();
}

void RobStride_Motor_MIT_SetZeroPos(uint8_t motor_id)
{
    TX_LOCK();
    s_std_id = motor_id;
    memset(g_tx_data, 0xFF, 7);
    g_tx_data[7] = 0xFE;
    twai_send_std();
    TX_UNLOCK();
}

void RobStride_Motor_MIT_MotorDataSave(uint8_t motor_id)
{
    TX_LOCK();
    s_std_id = motor_id;
    memset(g_tx_data, 0xFF, 7);
    g_tx_data[7] = 0xF8;
    twai_send_std();
    TX_UNLOCK();
}

void RobStride_Motor_MIT_ProtactiveEscalationSet(uint8_t motor_id, uint8_t F_CMD)
{
    TX_LOCK();
    s_std_id = motor_id;
    memset(g_tx_data, 0xFF, 6);
    g_tx_data[6] = F_CMD;
    g_tx_data[7] = 0xF9;
    twai_send_std();
    TX_UNLOCK();
}

void Get_RobStride_Motor_MIT_parameter(uint8_t motor_id, uint16_t Index)
{
    TX_LOCK();
    s_std_id = motor_id;
    memset(g_tx_data, 0, 8);
    g_tx_data[0] = Index & 0xFF;
    g_tx_data[1] = (Index >> 8) & 0xFF;
    twai_send_std();
    TX_UNLOCK();
}

void Set_RobStride_Motor_MIT_parameter(uint8_t motor_id, uint16_t Index, float Value)
{
    TX_LOCK();
    s_std_id = motor_id;
    memset(g_tx_data, 0, 8);
    g_tx_data[0] = Index & 0xFF;
    g_tx_data[1] = (Index >> 8) & 0xFF;
    memcpy(&g_tx_data[4], &Value, 4);
    twai_send_std();
    TX_UNLOCK();
}

/* ================================================================
 *  Feedback Parser
 * ================================================================ */

void parse_motor_feedback(bool is_extended, uint32_t identifier, const uint8_t rx_data[8])
{
    motor_feedback_t feedback;
    robstride_get_feedback_snapshot(&feedback);

    if (is_extended) {
        uint16_t data_field = ext_id_get_data(identifier);

        feedback.uncalibrated         = ((data_field >> 13) & 0x01) ? 1 : 0;
        feedback.StallOverloadFault   = ((data_field >> 12) & 0x01) ? 1 : 0;
        feedback.MagneticEncoderFault = ((data_field >> 11) & 0x01) ? 1 : 0;
        feedback.OverTemperatureFault = ((data_field >> 10) & 0x01) ? 1 : 0;
        feedback.DriveFault           = ((data_field >> 9)  & 0x01) ? 1 : 0;
        feedback.UnderVoltageFault    = ((data_field >> 8)  & 0x01) ? 1 : 0;

        uint8_t mode_tmp = (uint8_t)((data_field >> 14) & 0x03);
        if (mode_tmp > RUNNING) mode_tmp = REST;
        feedback.mms = (enum ModeStatus)mode_tmp;

        uint16_t angle_raw  = ((uint16_t)rx_data[0] << 8) | rx_data[1];
        uint16_t speed_raw  = ((uint16_t)rx_data[2] << 8) | rx_data[3];
        uint16_t torque_raw = ((uint16_t)rx_data[4] << 8) | rx_data[5];
        uint16_t temp_raw   = ((uint16_t)rx_data[6] << 8) | rx_data[7];

        feedback.angle       = uint16_to_float(angle_raw, P_MIN, P_MAX);
        feedback.speed       = uint16_to_float(speed_raw, V_MIN, V_MAX);
        feedback.torque      = uint16_to_float(torque_raw, T_MIN, T_MAX);
        feedback.temperature = temp_raw / 10.0f;
    } else {
        uint16_t angle_raw  = ((uint16_t)rx_data[1] << 8) | rx_data[2];
        uint16_t speed_raw  = ((uint16_t)rx_data[3] << 4) |
                              ((uint16_t)rx_data[4] >> 4);
        uint16_t torque_raw = ((uint16_t)(rx_data[4] & 0x0F) << 8) | rx_data[5];
        uint16_t temp_raw   = ((uint16_t)rx_data[6] << 8) | rx_data[7];

        feedback.angle       = uint16_to_float(angle_raw, P_MIN, P_MAX);
        feedback.speed       = uint12_to_float(speed_raw, V_MIN, V_MAX);
        feedback.torque      = uint12_to_float(torque_raw, T_MIN, T_MAX);
        feedback.temperature = temp_raw / 10.0f;
    }

    portENTER_CRITICAL(&s_feedback_lock);
    g_motor_feedback = feedback;
    portEXIT_CRITICAL(&s_feedback_lock);
}

void robstride_update_mit_faults(uint32_t fault_value)
{
    portENTER_CRITICAL(&s_feedback_lock);
    g_motor_feedback.uncalibrated       = (fault_value & (1U << 7)) != 0;
    g_motor_feedback.StallOverloadFault = (fault_value & (1U << 14)) != 0;
    g_motor_feedback.OverTemperatureFault = (fault_value & (1U << 0)) != 0;
    g_motor_feedback.DriveFault         = (fault_value & (1U << 1)) != 0;
    g_motor_feedback.UnderVoltageFault  = (fault_value & (1U << 2)) != 0;
    portEXIT_CRITICAL(&s_feedback_lock);
}
