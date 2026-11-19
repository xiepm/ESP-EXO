#include "exo_wear_zero.h"

#include <math.h>
#include <string.h>
#include "exo_config.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "nvs.h"

/* 双侧同一blob原子提交，避免断电后得到“新左零位+旧右零位”。 */
#define ZERO_NAMESPACE "exo_wear"
#define ZERO_KEY "pair_v1"
#define ZERO_MAGIC 0x5A455231U
#define ZERO_FRESH_US 50000ULL
#define ZERO_PERIOD_DEG 1440.0f /* RS01线上位置为-4π..+4π rad，完整周期8π。 */
static uint32_t s_record[5];
static bool s_wear_initialized, s_stored, s_bad_record;
static bool s_auto_restore_enabled;
static portMUX_TYPE s_record_lock = portMUX_INITIALIZER_UNLOCKED;

static uint32_t binding(const hip_joint_snapshot_t *joint) {
    return ((uint32_t)g_exo_hardware_config.actuator[joint->actuator].can_id << 16) |
        ((uint32_t)joint->actuator << 8) |
        (joint->direction == EXO_DIRECTION_POSITIVE ? 1U : 2U);
}
static float record_zero(const uint32_t record[5], unsigned side) {
    float value;
    memcpy(&value, &record[3U + side], sizeof(value));
    return value;
}
static bool record_valid(const uint32_t record[5]) {
    if (record[0] != ZERO_MAGIC || record[1] == record[2]) return false;
    for (unsigned i = 0; i < 2U; ++i) {
        const uint32_t b = record[1U + i];
        const float zero = record_zero(record, i);
        if ((b & 0xFF80FEFCU) || (b & 3U) == 0U || (b & 3U) == 3U ||
            !isfinite(zero) || fabsf(zero) > 720.0f) return false;
    }
    return ((record[1] >> 8) & 1U) != ((record[2] >> 8) & 1U);
}
esp_err_t exo_wear_zero_init(void) {
    uint32_t record[5] = {0};
    nvs_handle_t handle;
    esp_err_t error = nvs_open(ZERO_NAMESPACE, NVS_READONLY, &handle);
    bool stored = false, bad = false;
    if (error == ESP_OK) {
        size_t size = sizeof(record);
        error = nvs_get_blob(handle, ZERO_KEY, record, &size);
        nvs_close(handle);
        if (error == ESP_OK) {
            stored = size == sizeof(record) && record_valid(record);
            bad = !stored;
        } else if (error == ESP_ERR_NVS_INVALID_LENGTH) {
            bad = true;
            error = ESP_OK;
        }
    }
    if (error == ESP_ERR_NVS_NOT_FOUND) error = ESP_OK;
    if (error != ESP_OK) return error;
    portENTER_CRITICAL(&s_record_lock);
    memcpy(s_record, record, sizeof(record));
    s_stored = stored;
    s_bad_record = bad;
    s_wear_initialized = true;
    s_auto_restore_enabled = stored;
    portEXIT_CRITICAL(&s_record_lock);
    /* CAN尚未就绪；稍后仅在IDLE用新反馈建立本次连续坐标。 */
    return ESP_OK;
}
static bool copy_record(uint32_t record[5], bool *bad) {
    portENTER_CRITICAL(&s_record_lock);
    memcpy(record, s_record, sizeof(s_record));
    const bool stored = s_wear_initialized && s_stored;
    *bad = s_bad_record;
    portEXIT_CRITICAL(&s_record_lock);
    return stored;
}
static exo_wear_zero_result_t read_pair(hip_joint_snapshot_t joints[2],
    rs01_feedback_snapshot_t motors[2]) {
    for (unsigned i = 0; i < 2U; ++i) {
        if (!hip_joint_get_snapshot((exo_hip_joint_id_t)i, &joints[i]) ||
            !joints[i].mapping_valid || !joints[i].direction_valid ||
            !rs01_get_feedback(joints[i].actuator, &motors[i]))
            return EXO_WEAR_ZERO_BINDING_CHANGED;
    }
    /* 在复制双反馈之后取时间，避免恰好到达的新帧被误判为“来自未来”。 */
    const int64_t now = esp_timer_get_time();
    if (now <= 0) return EXO_WEAR_ZERO_FEEDBACK_INVALID;
    for (unsigned i = 0; i < 2U; ++i) {
        const rs01_feedback_snapshot_t *m = &motors[i];
        if (!m->valid || !m->online || !m->continuous_position_valid ||
            !m->timestamp_us || m->timestamp_us > (uint64_t)now ||
            (uint64_t)now - m->timestamp_us > ZERO_FRESH_US || !m->position_epoch ||
            !m->feedback_count || !isfinite(m->sample.position_rad) ||
            !isfinite(m->continuous_position_rad) || !isfinite(m->sample.velocity_rad_s) ||
            !isfinite(m->sample.torque_nm) || !isfinite(m->sample.temperature_c) ||
            !isfinite(exo_radians_to_degrees(m->continuous_position_rad)) ||
            fabsf(exo_radians_to_degrees(m->sample.position_rad)) > 720.001f)
            return EXO_WEAR_ZERO_FEEDBACK_INVALID;
        if (m->faults.present) return EXO_WEAR_ZERO_MOTOR_FAULT;
        if (m->mode != RS01_MODE_STATUS_RESET) return EXO_WEAR_ZERO_MOTOR_NOT_REST;
        if (fabsf(exo_radians_to_degrees(m->sample.velocity_rad_s)) > 5.0f)
            return EXO_WEAR_ZERO_NOT_STILL;
    }
    return joints[0].actuator != joints[1].actuator ? EXO_WEAR_ZERO_OK :
        EXO_WEAR_ZERO_BINDING_CHANGED;
}
static exo_wear_zero_result_t preview_pair(const uint32_t record[5],
    hip_joint_snapshot_t joints[2], rs01_feedback_snapshot_t motors[2], float preview[2]) {
    for (unsigned i = 0; i < 2U; ++i) {
        if (!hip_joint_get_snapshot((exo_hip_joint_id_t)i, &joints[i]) ||
            !joints[i].mapping_valid || !joints[i].direction_valid ||
            binding(&joints[i]) != record[1U + i]) return EXO_WEAR_ZERO_BINDING_CHANGED;
    }
    const exo_wear_zero_result_t ready = read_pair(joints, motors);
    if (ready != EXO_WEAR_ZERO_OK) return ready;
    for (unsigned i = 0; i < 2U; ++i) {
        if (binding(&joints[i]) != record[1U + i]) return EXO_WEAR_ZERO_BINDING_CHANGED;
    }
    bool mismatch = false;
    for (unsigned i = 0; i < 2U; ++i) {
        /* 旧启动的连续圈数/epoch不可恢复。用周期差预览当前人体角，再由关节层
         * 把候选零位投影到本次连续坐标；回绕边界附近不会产生1440度假偏差。 */
        preview[i] = (float)joints[i].direction * remainderf(
            exo_radians_to_degrees(motors[i].sample.position_rad) - record_zero(record, i),
            ZERO_PERIOD_DEG);
        if (!isfinite(preview[i]) || fabsf(preview[i]) > EXO_WEAR_ZERO_REUSE_MAX_DEVIATION_DEG)
            mismatch = true;
    }
    return mismatch ? EXO_WEAR_ZERO_POSTURE_MISMATCH : EXO_WEAR_ZERO_OK;
}
bool exo_wear_zero_get_status(exo_hip_joint_id_t joint, exo_wear_zero_status_t *out) {
    if (!out || (unsigned)joint >= 2U) return false;
    memset(out, 0, sizeof(*out));
    out->actuator = EXO_ACTUATOR_COUNT;
    out->saved_periodic_motor_deg = out->preview_joint_deg = NAN;
    hip_joint_snapshot_t current;
    if (hip_joint_get_snapshot(joint, &current) && current.mapping_valid) {
        out->actuator = current.actuator;
        out->can_id = g_exo_hardware_config.actuator[current.actuator].can_id;
        out->direction = current.direction;
    }
    uint32_t record[5];
    bool bad;
    out->stored = copy_record(record, &bad);
    out->reason = bad ? EXO_WEAR_ZERO_BAD_RECORD : EXO_WEAR_ZERO_NO_RECORD;
    if (!out->stored) return true;
    out->saved_periodic_motor_deg = record_zero(record, (unsigned)joint);
    hip_joint_snapshot_t joints[2];
    rs01_feedback_snapshot_t motors[2];
    float preview[2] = {NAN, NAN};
    out->reason = preview_pair(record, joints, motors, preview);
    out->binding_matches = true;
    for (unsigned i = 0; i < 2U; ++i) {
        hip_joint_snapshot_t mapped;
        if (!hip_joint_get_snapshot((exo_hip_joint_id_t)i, &mapped) || !mapped.mapping_valid ||
            !mapped.direction_valid || binding(&mapped) != record[1U + i]) out->binding_matches = false;
    }
    out->preview_valid = isfinite(preview[joint]);
    out->preview_joint_deg = preview[joint];
    out->reuse_ready = out->reason == EXO_WEAR_ZERO_OK;
    return true;
}
static esp_err_t write_record(const uint32_t *record) {
    nvs_handle_t handle;
    esp_err_t error = nvs_open(ZERO_NAMESPACE, NVS_READWRITE, &handle);
    if (error != ESP_OK) return error;
    if (record) error = nvs_set_blob(handle, ZERO_KEY, record, sizeof(s_record));
    else {
        error = nvs_erase_key(handle, ZERO_KEY);
        if (error == ESP_ERR_NVS_NOT_FOUND) error = ESP_OK;
    }
    if (error == ESP_OK) error = nvs_commit(handle);
    uint32_t verified[5];
    size_t size = sizeof(verified);
    if (error == ESP_OK && record) {
        error = nvs_get_blob(handle, ZERO_KEY, verified, &size);
        if (error == ESP_OK && (size != sizeof(verified) ||
            memcmp(record, verified, sizeof(verified)))) error = ESP_FAIL;
    } else if (error == ESP_OK) {
        const esp_err_t missing = nvs_get_blob(handle, ZERO_KEY, verified, &size);
        if (missing != ESP_ERR_NVS_NOT_FOUND) error = ESP_FAIL;
    }
    nvs_close(handle);
    if (error != ESP_OK) return error;
    portENTER_CRITICAL(&s_record_lock);
    if (record) memcpy(s_record, record, sizeof(s_record));
    else memset(s_record, 0, sizeof(s_record));
    s_stored = record != NULL;
    s_auto_restore_enabled = record != NULL;
    s_bad_record = false;
    portEXIT_CRITICAL(&s_record_lock);
    return ESP_OK;
}
bool exo_wear_zero_auto_restore_pending(void) {
    portENTER_CRITICAL(&s_record_lock);
    const bool pending=s_wear_initialized&&s_auto_restore_enabled;
    portEXIT_CRITICAL(&s_record_lock);
    if(!pending)return false;
    hip_joint_snapshot_t joints[2];
    return !hip_joint_get_snapshot(EXO_HIP_JOINT_LEFT,&joints[0]) ||
        !hip_joint_get_snapshot(EXO_HIP_JOINT_RIGHT,&joints[1]) ||
        !joints[0].zero_valid || !joints[1].zero_valid;
}
void exo_wear_zero_cancel_auto_restore(void) {
    portENTER_CRITICAL(&s_record_lock);s_auto_restore_enabled=false;
    portEXIT_CRITICAL(&s_record_lock);
}
void exo_wear_zero_restore_saved_if_needed(void) {
    if(!exo_wear_zero_auto_restore_pending())return;
    uint32_t record[5];bool bad;
    if(!copy_record(record,&bad)) {exo_wear_zero_cancel_auto_restore();return;}
    hip_joint_snapshot_t joints[2];rs01_feedback_snapshot_t motors[2];
    for(unsigned i=0;i<2;i++) {
        if(!hip_joint_get_snapshot((exo_hip_joint_id_t)i,&joints[i])||
           !joints[i].mapping_valid||!joints[i].direction_valid||
           binding(&joints[i])!=record[1U+i]) {
            exo_wear_zero_cancel_auto_restore();return;
        }
    }
    if(read_pair(joints,motors)!=EXO_WEAR_ZERO_OK)return;
    const float zeros[2]={record_zero(record,0),record_zero(record,1)};
    /* Also covers motor-only power cycles, but the caller's IDLE maintenance
     * excludes RUNNING/ERROR. Rebinding cannot open the motion output gate. */
    (void)hip_joint_restore_persistent_zero_pair(zeros,motors);
}
static void reject_persistent_capture(exo_wear_zero_action_t action) {
    if(action!=EXO_WEAR_ZERO_CAPTURE_SAVE_PAIR)return;
    (void)hip_joint_clear_zero(EXO_HIP_JOINT_LEFT);
    (void)hip_joint_clear_zero(EXO_HIP_JOINT_RIGHT);
}
esp_err_t exo_wear_zero_execute(exo_wear_zero_action_t action, bool confirmed,
    exo_wear_zero_result_t *reason) {
    if (!reason || (unsigned)action > EXO_WEAR_ZERO_CAPTURE_SAVE_PAIR) return ESP_ERR_INVALID_ARG;
    *reason = EXO_WEAR_ZERO_NOT_CONFIRMED;
    if (!confirmed) return ESP_ERR_INVALID_STATE;
    if (!s_wear_initialized) return ESP_ERR_INVALID_STATE;
    if (action == EXO_WEAR_ZERO_FORGET) {
        *reason = write_record(NULL) == ESP_OK ? EXO_WEAR_ZERO_OK : EXO_WEAR_ZERO_STORAGE_FAILED;
        return *reason == EXO_WEAR_ZERO_OK ? ESP_OK : ESP_FAIL;
    }
    uint32_t record[5];
    bool bad;
    hip_joint_snapshot_t joints[2];
    rs01_feedback_snapshot_t motors[2];
    if (action == EXO_WEAR_ZERO_CAPTURE_PAIR || action == EXO_WEAR_ZERO_CAPTURE_SAVE_PAIR) {
        *reason = read_pair(joints, motors);
        if (*reason != EXO_WEAR_ZERO_OK) return ESP_ERR_INVALID_STATE;
        /* 成对采集与沿用是不同动作：这里不读取NVS、不受旧零位偏差限制。
         * 当前姿态由操作者确认；新反馈决定左右各自的零位，不命令电机回零。 */
        if (hip_joint_capture_zero_pair(joints, motors) != ESP_OK) {
            *reason = EXO_WEAR_ZERO_EPOCH_CHANGED;
            return ESP_ERR_INVALID_STATE;
        }
        exo_wear_zero_cancel_auto_restore();
        if(action==EXO_WEAR_ZERO_CAPTURE_PAIR)return ESP_OK;
    }
    if (action == EXO_WEAR_ZERO_RESTORE) {
        if (!copy_record(record, &bad)) {
            *reason = bad ? EXO_WEAR_ZERO_BAD_RECORD : EXO_WEAR_ZERO_NO_RECORD;
            return ESP_ERR_INVALID_STATE;
        }
        float preview[2];
        *reason = preview_pair(record, joints, motors, preview);
        if (*reason != EXO_WEAR_ZERO_OK) return ESP_ERR_INVALID_STATE;
        const float zeros[2] = {record_zero(record, 0), record_zero(record, 1)};
        if (hip_joint_restore_zero_pair(zeros, motors) != ESP_OK) {
            *reason = EXO_WEAR_ZERO_EPOCH_CHANGED;
            return ESP_ERR_INVALID_STATE;
        }
        portENTER_CRITICAL(&s_record_lock);s_auto_restore_enabled=true;
        portEXIT_CRITICAL(&s_record_lock);
        return ESP_OK;
    }
    *reason = read_pair(joints, motors);
    if (*reason != EXO_WEAR_ZERO_OK) {
        reject_persistent_capture(action);return ESP_ERR_INVALID_STATE;
    }
    record[0] = ZERO_MAGIC;
    for (unsigned i = 0; i < 2U; ++i) {
        if (!joints[i].zero_valid || !joints[i].joint_valid ||
            joints[i].zero_source_position_epoch != motors[i].position_epoch ||
            !isfinite(joints[i].joint_sample.position_deg) ||
            fabsf(exo_radians_to_degrees(motors[i].continuous_position_rad) -
                joints[i].software_zero_motor_deg) > EXO_WEAR_ZERO_REUSE_MAX_DEVIATION_DEG) {
            *reason = EXO_WEAR_ZERO_CURRENT_ZERO_INVALID;
            reject_persistent_capture(action);
            return ESP_ERR_INVALID_STATE;
        }
        record[1U + i] = binding(&joints[i]);
        memcpy(&record[3U + i], &joints[i].software_zero_periodic_motor_deg, sizeof(float));
    }
    if (!record_valid(record) || write_record(record) != ESP_OK) {
        /* Never enable START on coordinates that failed to persist. */
        reject_persistent_capture(action);
        *reason = EXO_WEAR_ZERO_STORAGE_FAILED;
        return ESP_FAIL;
    }
    return ESP_OK;
}
esp_err_t exo_wear_zero_teach_direction(exo_hip_joint_id_t joint,
    hip_direction_teach_action_t action, uint8_t token,
    hip_direction_teach_result_t *reason, exo_direction_t *direction) {
    if (!reason || !direction || (unsigned)joint >= 2U || !token ||
        (unsigned)action > HIP_DIRECTION_TEACH_CANCEL) return ESP_ERR_INVALID_ARG;
    /* A rejected BEGIN (offline, moving, busy) must not erase a valid saved
     * calibration. Invalidate only after the joint layer accepted the session. */
    esp_err_t error=hip_joint_teach_direction(joint,action,token,reason,direction);
    if(error!=ESP_OK)return error;
    if(action==HIP_DIRECTION_TEACH_BEGIN)exo_wear_zero_cancel_auto_restore();
    if (action == HIP_DIRECTION_TEACH_BEGIN && write_record(NULL) != ESP_OK) {
        hip_direction_teach_result_t ignored;exo_direction_t unused;
        (void)hip_joint_teach_direction(joint,HIP_DIRECTION_TEACH_CANCEL,token,&ignored,&unused);
        *reason = HIP_DIRECTION_TEACH_STORAGE_FAILED;
        *direction = EXO_DIRECTION_UNKNOWN;
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}
