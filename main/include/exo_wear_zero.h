#ifndef EXO_WEAR_ZERO_H
#define EXO_WEAR_ZERO_H

#include "hip_joint.h"

/*
 * 持久化穿戴零位：开机加载记录，新鲜REST反馈到达后在IDLE恢复原偏置。
 * 恢复不采集新零点、不使能、不发送运动命令。
 * 所有写/恢复操作由调用者取得整机IDLE维护权后执行；100 Hz任务不访问NVS。
 */
#define EXO_WEAR_ZERO_REUSE_MAX_DEVIATION_DEG 5.0f
typedef enum {
    EXO_WEAR_ZERO_OK = 0,
    EXO_WEAR_ZERO_NO_RECORD,
    EXO_WEAR_ZERO_BINDING_CHANGED,
    EXO_WEAR_ZERO_FEEDBACK_INVALID,
    EXO_WEAR_ZERO_MOTOR_NOT_REST,
    EXO_WEAR_ZERO_MOTOR_FAULT,
    EXO_WEAR_ZERO_NOT_STILL,
    EXO_WEAR_ZERO_POSTURE_MISMATCH,
    EXO_WEAR_ZERO_STORAGE_FAILED,
    EXO_WEAR_ZERO_DIRECTION_TEACHING,
    EXO_WEAR_ZERO_CURRENT_ZERO_INVALID,
    EXO_WEAR_ZERO_NOT_CONFIRMED,
    EXO_WEAR_ZERO_EPOCH_CHANGED,
    EXO_WEAR_ZERO_BAD_RECORD,
} exo_wear_zero_result_t;
typedef enum {
    EXO_WEAR_ZERO_SAVE_CURRENT = 0,
    EXO_WEAR_ZERO_RESTORE = 1,
    EXO_WEAR_ZERO_FORGET = 2,
    /* 0.10.11：重新采集当前双侧位置，一次提交到RAM；不复用旧零位。 */
    EXO_WEAR_ZERO_CAPTURE_PAIR = 3,
    /* 0.11.9：重新采集并保存整对零位；ACK成功即已提交并读回NVS。 */
    EXO_WEAR_ZERO_CAPTURE_SAVE_PAIR = 4,
} exo_wear_zero_action_t;
typedef struct {
    exo_actuator_id_t actuator;
    exo_direction_t direction;
    exo_can_node_id_t can_id;
    float saved_periodic_motor_deg;
    float preview_joint_deg;
    bool stored;
    bool binding_matches;
    bool preview_valid;
    bool reuse_ready;
    exo_wear_zero_result_t reason;
} exo_wear_zero_status_t;

esp_err_t exo_wear_zero_init(void);
bool exo_wear_zero_auto_restore_pending(void);
/* Caller holds system IDLE maintenance; bounded cached-record read, no NVS I/O.
 * Restores after master/motor power loss; never runs in RUNNING or ERROR. */
void exo_wear_zero_restore_saved_if_needed(void);
/* Explicit manual RAM changes supersede automatic restore until save/reboot. */
void exo_wear_zero_cancel_auto_restore(void);
bool exo_wear_zero_get_status(exo_hip_joint_id_t joint, exo_wear_zero_status_t *out);
/* SAVE/RESTORE/CAPTURE_PAIR要求明确确认穿戴中立静止；FORGET只删候选。 */
esp_err_t exo_wear_zero_execute(exo_wear_zero_action_t action, bool confirmed,
    exo_wear_zero_result_t *reason);
/* 产品方向标定入口：BEGIN先撤销旧候选，安装调整后即使符号相同也不能复用。 */
esp_err_t exo_wear_zero_teach_direction(exo_hip_joint_id_t joint,
    hip_direction_teach_action_t action, uint8_t token,
    hip_direction_teach_result_t *reason, exo_direction_t *direction);

#endif
