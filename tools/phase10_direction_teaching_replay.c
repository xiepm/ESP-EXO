/*
 * 主机测试直接编译真实hip_joint.c和BLE纯字节编解码器。
 * 仅替代RS01反馈、单调时钟、临界区与NVS设备；没有串口、BLE或CAN实物操作。
 */
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "hip_joint.h"
#include "exo_ble_protocol.h"

/* 只替代硬件配置容器，使用实际LEFT->B / RIGHT->A映射；运行方向可在RAM更改。 */
#define EXO_CONFIG_H
typedef struct {
    exo_side_t side;
    exo_actuator_id_t actuator;
    exo_direction_t direction;
    float software_zero_deg;
    bool mapping_valid, direction_valid, software_zero_valid;
} exo_hip_joint_config_t;
static const struct {
    exo_hip_joint_config_t hip_joint[2];
    struct { exo_can_node_id_t can_id; } actuator[2];
} g_exo_hardware_config = {
    .hip_joint = {
        {.side=EXO_SIDE_LEFT, .actuator=EXO_ACTUATOR_B, .direction=EXO_DIRECTION_POSITIVE,
         .mapping_valid=true, .direction_valid=true},
        {.side=EXO_SIDE_RIGHT, .actuator=EXO_ACTUATOR_A, .direction=EXO_DIRECTION_NEGATIVE,
         .mapping_valid=true, .direction_valid=true},
    },
    .actuator = {{.can_id=0x10}, {.can_id=0x20}},
};
#include "../main/exo_factory_direction.c"
#include "../main/hip_joint.c"

/* Fake仅模拟NVS API；真实记录编码、映射校验与提交顺序在被测模块内执行。 */
static bool flash_valid[2], pending_valid[2];
static uint32_t flash_records[2], pending_records[2];
static esp_err_t storage_init_error, storage_open_error, storage_commit_error;
static unsigned storage_save_calls;
esp_err_t nvs_flash_init(void) { return storage_init_error; }
esp_err_t nvs_open(const char *name, nvs_open_mode_t mode, nvs_handle_t *out) {
    assert(strcmp(name,"exo_factory")==0);
    if (storage_open_error) return storage_open_error;
    if (mode==NVS_READONLY && !flash_valid[0] && !flash_valid[1]) return ESP_ERR_NVS_NOT_FOUND;
    *out=1; return ESP_OK;
}
static unsigned key_slot(const char *key) {
    assert(strcmp(key,"left_dir_v1")==0 || strcmp(key,"right_dir_v1")==0);
    return key[0]=='l'?0U:1U;
}
esp_err_t nvs_get_u32(nvs_handle_t handle, const char *key, uint32_t *value) {
    assert(handle==1); const unsigned slot=key_slot(key);
    if (!flash_valid[slot]) return ESP_ERR_NVS_NOT_FOUND;
    *value=flash_records[slot]; return ESP_OK;
}
esp_err_t nvs_set_u32(nvs_handle_t handle, const char *key, uint32_t value) {
    assert(handle==1); const unsigned slot=key_slot(key);
    pending_valid[slot]=true; pending_records[slot]=value; ++storage_save_calls;
    return ESP_OK;
}
esp_err_t nvs_commit(nvs_handle_t handle) {
    assert(handle==1); if (storage_commit_error) return storage_commit_error;
    for (unsigned i=0;i<2;++i) if (pending_valid[i]) {
        flash_records[i]=pending_records[i]; flash_valid[i]=true;
    }
    return ESP_OK;
}
void nvs_close(nvs_handle_t handle) {
    assert(handle==1); memset(pending_valid,0,sizeof(pending_valid));
}

static int64_t clock_us;
static rs01_feedback_snapshot_t motors[2];
static unsigned window_motion, window_step;
static bool freeze_feedback;
void vTaskDelay(TickType_t ticks) {
    clock_us += (int64_t)ticks * 1000;
    ++window_step;
    if (freeze_feedback) return;
    for (unsigned i = 0; i < 2U; ++i) {
        if (window_motion == 1 && i == EXO_ACTUATOR_B) {
            const float t = (float)window_step * .01f;
            const float prev = (float)(window_step - 1U) * .01f;
            const float phase = 2.0f * 3.14159265f * 8.0f;
            motors[i].continuous_position_rad += exo_degrees_to_radians(
                .6f * (sinf(phase*t) - sinf(phase*prev)));
            motors[i].sample.velocity_rad_s = exo_degrees_to_radians(.6f*phase*cosf(phase*t));
        }
        if (window_motion == 2 && i == EXO_ACTUATOR_B)
            motors[i].continuous_position_rad += exo_degrees_to_radians(15.0f*(float)ticks*.001f);
        motors[i].timestamp_us = (uint64_t)clock_us - 1000U;
        ++motors[i].feedback_count;
    }
    if (window_motion == 3 && window_step == 10) motors[0].online = false;
    if (window_motion == 4 && window_step == 10) ++motors[0].position_epoch;
}
int64_t esp_timer_get_time(void) { return clock_us; }
bool rs01_get_feedback(exo_actuator_id_t actuator, rs01_feedback_snapshot_t *out) {
    if ((unsigned)actuator >= 2U || !out) return false;
    *out = motors[actuator];
    return true;
}
/* 这里只测坐标符号；最终CAN编码量程仍由既有驱动测试/目标板检查覆盖。 */
bool rs01_motion_command_is_valid(const rs01_motion_command_t *c) {
    return c && isfinite(c->position_rad) && isfinite(c->velocity_rad_s) &&
        isfinite(c->torque_nm) && c->kp >= 0 && c->kd >= 0;
}
static void pose(float left_deg, float right_deg) {
    clock_us += 10000;
    const float positions[2] = {right_deg, left_deg};
    for (unsigned i=0; i<2U; ++i) {
        motors[i].sample.position_rad = exo_degrees_to_radians(positions[i]);
        motors[i].continuous_position_rad = motors[i].sample.position_rad;
        motors[i].timestamp_us = (uint64_t)clock_us - 1000U;
        ++motors[i].feedback_count;
    }
}
static void reset(void) {
    window_motion = window_step = 0; freeze_feedback = false;
    memset(flash_valid,0,sizeof(flash_valid));
    memset(pending_valid,0,sizeof(pending_valid));
    storage_init_error=storage_open_error=storage_commit_error=0;
    storage_save_calls=0;
    assert(exo_factory_direction_storage_init()==ESP_OK);
    assert(exo_factory_direction_save(EXO_HIP_JOINT_LEFT,EXO_ACTUATOR_B,0x20,EXO_DIRECTION_POSITIVE)==ESP_OK);
    assert(exo_factory_direction_save(EXO_HIP_JOINT_RIGHT,EXO_ACTUATOR_A,0x10,EXO_DIRECTION_NEGATIVE)==ESP_OK);
    memset(s_joints, 0, sizeof(s_joints));
    memset(&s_direction_teach, 0, sizeof(s_direction_teach));
    s_initialized = false;
    clock_us = 1000000;
    memset(motors, 0, sizeof(motors));
    for (unsigned i=0; i<2U; ++i) {
        motors[i].valid = motors[i].online = motors[i].continuous_position_valid = true;
        motors[i].position_epoch = 1;
        motors[i].mode = RS01_MODE_STATUS_RESET;
    }
    pose(0, 0);
    assert(hip_joint_init() == ESP_OK);
}
static hip_direction_teach_result_t reason;
static exo_direction_t learned;
static esp_err_t teach(exo_hip_joint_id_t joint, hip_direction_teach_action_t action, uint8_t token) {
    window_step = 0;
    return hip_joint_teach_direction(joint, action, token, &reason, &learned);
}
static void close_session(void) {
    assert(teach(EXO_HIP_JOINT_LEFT, HIP_DIRECTION_TEACH_CANCEL, 255) == ESP_OK);
}
static void expect_failure(exo_hip_joint_id_t joint, hip_direction_teach_result_t expected) {
    assert(teach(joint, HIP_DIRECTION_TEACH_FINISH, 7) != ESP_OK);
    assert(reason == expected);
    assert(learned == EXO_DIRECTION_UNKNOWN);
}

int main(void) {
    reset(); window_motion=1;
    assert(teach(EXO_HIP_JOINT_LEFT,HIP_DIRECTION_TEACH_BEGIN,7)==ESP_OK);
    pose(-10,0);
    const unsigned before_save=storage_save_calls;
    assert(teach(EXO_HIP_JOINT_LEFT,HIP_DIRECTION_TEACH_FINISH,7)==ESP_OK);
    assert(learned==EXO_DIRECTION_NEGATIVE && storage_save_calls==before_save+1);
    puts("PASS 8Hz +/-0.6deg suspended-leg tremor (~30deg/s peak) accepted using mean position");
    for (unsigned bad=2;bad<=4;++bad) {
        reset();
        assert(teach(EXO_HIP_JOINT_LEFT,HIP_DIRECTION_TEACH_BEGIN,7)==ESP_OK);
        pose(10,0); window_motion=bad;
        const unsigned saved=storage_save_calls;
        expect_failure(EXO_HIP_JOINT_LEFT,bad==2 ? HIP_DIRECTION_TEACH_NOT_STILL :
            bad==3 ? HIP_DIRECTION_TEACH_FEEDBACK_INVALID : HIP_DIRECTION_TEACH_EPOCH_CHANGED);
        assert(storage_save_calls==saved);
    }
    puts("PASS sustained movement, feedback dropout and mid-window epoch changes do not save");
    reset();
    assert(teach(EXO_HIP_JOINT_LEFT,HIP_DIRECTION_TEACH_BEGIN,7)==ESP_OK);
    clock_us=(int64_t)s_direction_teach.started_us+59900000;
    pose(10,0);
    const unsigned expiry_saves=storage_save_calls;
    expect_failure(EXO_HIP_JOINT_LEFT,HIP_DIRECTION_TEACH_EXPIRED);
    assert(storage_save_calls==expiry_saves && !s_direction_teach.active);
    puts("PASS session expiring during pose window cannot write factory direction");
    reset();
    assert(hip_joint_set_zero_from_current(EXO_HIP_JOINT_LEFT) == ESP_OK);
    assert(teach(EXO_HIP_JOINT_LEFT, HIP_DIRECTION_TEACH_BEGIN, 7) == ESP_OK);
    hip_joint_snapshot_t snap;
    assert(hip_joint_get_snapshot(EXO_HIP_JOINT_LEFT, &snap));
    assert(!snap.zero_valid && !snap.joint_valid);
    assert(hip_joint_set_zero_from_current(EXO_HIP_JOINT_LEFT) == ESP_ERR_INVALID_STATE);
    pose(-10, 0);
    assert(teach(EXO_HIP_JOINT_LEFT, HIP_DIRECTION_TEACH_FINISH, 7) == ESP_OK);
    assert(learned == EXO_DIRECTION_NEGATIVE);
    assert(hip_joint_get_snapshot(EXO_HIP_JOINT_LEFT, &snap) && !snap.zero_valid);
    pose(0, 0);
    assert(hip_joint_set_zero_from_current(EXO_HIP_JOINT_LEFT) == ESP_OK);
    pose(-10, 0);
    motors[EXO_ACTUATOR_B].sample.velocity_rad_s = exo_degrees_to_radians(-2);
    motors[EXO_ACTUATOR_B].sample.torque_nm = -0.1f;
    assert(hip_joint_get_snapshot(EXO_HIP_JOINT_LEFT, &snap));
    assert(snap.joint_valid && fabsf(snap.joint_sample.position_deg-10)<0.001f);
    assert(fabsf(snap.joint_sample.velocity_deg_s-2)<0.001f);
    assert(fabsf(snap.joint_sample.torque_nm-0.1f)<0.001f);
    const exo_joint_command_t cmd = {.position_deg=5, .velocity_deg_s=3,
        .torque_ff_nm=0.2f, .kp=2, .kd=0.1f};
    rs01_motion_command_t motor_cmd;
    exo_actuator_id_t slot;
    assert(hip_joint_to_actuator_command(EXO_HIP_JOINT_LEFT, &cmd, &slot, &motor_cmd)==ESP_OK);
    assert(slot==EXO_ACTUATOR_B && fabsf(motor_cmd.position_rad-exo_degrees_to_radians(-5))<1e-6f);
    assert(fabsf(motor_cmd.velocity_rad_s-exo_degrees_to_radians(-3))<1e-6f);
    assert(motor_cmd.torque_nm==-0.2f && motor_cmd.kp==2 && motor_cmd.kd==0.1f);
    puts("PASS left reversed installation: q/dq/torque feedback and five-element output share learned sign");

    reset();
    assert(teach(EXO_HIP_JOINT_RIGHT, HIP_DIRECTION_TEACH_BEGIN, 7)==ESP_OK);
    pose(0, 10);
    assert(teach(EXO_HIP_JOINT_RIGHT, HIP_DIRECTION_TEACH_FINISH, 7)==ESP_OK);
    assert(learned==EXO_DIRECTION_POSITIVE);
    assert(hip_joint_get_snapshot(EXO_HIP_JOINT_RIGHT,&snap) && snap.actuator==EXO_ACTUATOR_A);
    puts("PASS right direction independent of left and CAN slot");

    reset();pose(8,-30);
    assert(teach(EXO_HIP_JOINT_RIGHT,HIP_DIRECTION_TEACH_BEGIN,9)==ESP_OK);
    pose(15,-50); /* user lifts before the first browser preview arrives */
    hip_direction_teach_preview_t preview;
    const unsigned preview_saves=storage_save_calls;
    assert(hip_joint_direction_preview(EXO_HIP_JOINT_RIGHT,&preview));
    assert(preview.result==HIP_DIRECTION_TEACH_OK && preview.token==9 && preview.actuator==EXO_ACTUATOR_A);
    assert(fabsf(preview.baseline_deg+30)<.001f && fabsf(preview.current_deg+50)<.001f);
    assert(fabsf(preview.current_deg-preview.baseline_deg+20)<.001f);
    pose(20,-60); assert(hip_joint_direction_preview(EXO_HIP_JOINT_RIGHT,&preview));
    assert(fabsf(preview.baseline_deg+30)<.001f && fabsf(preview.current_deg+60)<.001f);
    assert(storage_save_calls==preview_saves); /* reads cannot recapture or save */
    assert(hip_joint_direction_preview(EXO_HIP_JOINT_LEFT,&preview) && preview.result==HIP_DIRECTION_TEACH_NO_SESSION);
    motors[EXO_ACTUATOR_A].timestamp_us=(uint64_t)clock_us-50001;
    assert(hip_joint_direction_preview(EXO_HIP_JOINT_RIGHT,&preview) && preview.result==HIP_DIRECTION_TEACH_FEEDBACK_INVALID && isnan(preview.current_deg));
    pose(20,-60);++motors[EXO_ACTUATOR_A].position_epoch;
    assert(hip_joint_direction_preview(EXO_HIP_JOINT_RIGHT,&preview) && preview.result==HIP_DIRECTION_TEACH_EPOCH_CHANGED);
    clock_us=(int64_t)s_direction_teach.started_us+60000000;
    assert(hip_joint_direction_preview(EXO_HIP_JOINT_RIGHT,&preview) && preview.result==HIP_DIRECTION_TEACH_EXPIRED);
    puts("PASS actual BEGIN mean survives delayed first browser read; stale/epoch/expiry/wrong-side rejected without writes");

    reset();
    assert(teach(EXO_HIP_JOINT_LEFT,HIP_DIRECTION_TEACH_BEGIN,7)==ESP_OK);
    pose(12,-8); /* opposite mechanism follows; selected LEFT raw increases */
    assert(teach(EXO_HIP_JOINT_LEFT,HIP_DIRECTION_TEACH_FINISH,7)==ESP_OK);
    pose(0,0);
    assert(teach(EXO_HIP_JOINT_RIGHT,HIP_DIRECTION_TEACH_BEGIN,8)==ESP_OK);
    pose(9,-12); /* selected RIGHT raw decreases during forward flexion */
    assert(teach(EXO_HIP_JOINT_RIGHT,HIP_DIRECTION_TEACH_FINISH,8)==ESP_OK);
    pose(0,0);
    for(unsigned side=0;side<2;side++) assert(hip_joint_set_zero_from_current(side)==ESP_OK);
    pose(12,-12);
    for(unsigned side=0;side<2;side++) {
        assert(hip_joint_get_snapshot(side,&snap) && snap.joint_valid);
        assert(fabsf(snap.joint_sample.position_deg-12)<.001f);
        exo_joint_command_t peak_cmd={.torque_ff_nm=15};
        assert(hip_joint_to_actuator_command(side,&peak_cmd,&slot,&motor_cmd)==ESP_OK);
        assert(motor_cmd.torque_nm==(side==0?15.f:-15.f));
    }
    pose(-8,8);
    for(unsigned side=0;side<2;side++) {
        assert(hip_joint_get_snapshot(side,&snap) && snap.joint_valid);
        assert(fabsf(snap.joint_sample.position_deg+8)<.001f);
    }
    puts("PASS coupled teaching: both forward angles positive, backward negative, +/-15Nm wire direction follows calibration");

    reset();
    assert(teach(EXO_HIP_JOINT_LEFT, HIP_DIRECTION_TEACH_BEGIN, 7)==ESP_OK);
    pose(.99f, 0); expect_failure(EXO_HIP_JOINT_LEFT,HIP_DIRECTION_TEACH_TOO_SMALL);
    pose(0, 10); expect_failure(EXO_HIP_JOINT_LEFT,HIP_DIRECTION_TEACH_TOO_SMALL);
    pose(1.5f, 4);
    assert(teach(EXO_HIP_JOINT_LEFT,HIP_DIRECTION_TEACH_FINISH,7)==ESP_OK);
    assert(learned==EXO_DIRECTION_POSITIVE);
    reset(); assert(teach(EXO_HIP_JOINT_RIGHT,HIP_DIRECTION_TEACH_BEGIN,7)==ESP_OK);
    pose(10,-40); window_motion=2; /* moving LEFT does not veto teaching RIGHT */
    assert(teach(EXO_HIP_JOINT_RIGHT,HIP_DIRECTION_TEACH_FINISH,7)==ESP_OK);
    assert(learned==EXO_DIRECTION_NEGATIVE);
    puts("PASS selected-leg sign at1.5deg and40deg; coupled/moving opposite leg allowed; sub1deg/wrong-only motion rejected");

    reset();
    motors[0].timestamp_us=(uint64_t)clock_us-50000;
    assert(teach(EXO_HIP_JOINT_LEFT,HIP_DIRECTION_TEACH_BEGIN,7)==ESP_OK);
    close_session(); motors[0].timestamp_us=(uint64_t)clock_us-50001;
    assert(teach(EXO_HIP_JOINT_LEFT,HIP_DIRECTION_TEACH_BEGIN,7)!=ESP_OK);
    assert(reason==HIP_DIRECTION_TEACH_FEEDBACK_INVALID);
    reset(); assert(teach(EXO_HIP_JOINT_LEFT,HIP_DIRECTION_TEACH_BEGIN,7)==ESP_OK);
    freeze_feedback=true;
    expect_failure(EXO_HIP_JOINT_LEFT,HIP_DIRECTION_TEACH_FEEDBACK_INVALID);
    freeze_feedback=false;
    pose(10, 0); ++motors[0].position_epoch;
    expect_failure(EXO_HIP_JOINT_LEFT,HIP_DIRECTION_TEACH_EPOCH_CHANGED);
    puts("PASS 50ms boundary, unchanged sample and either-side epoch mismatch");

    reset(); assert(teach(EXO_HIP_JOINT_LEFT,HIP_DIRECTION_TEACH_BEGIN,7)==ESP_OK);
    clock_us+=60000001; pose(10,0);
    expect_failure(EXO_HIP_JOINT_LEFT,HIP_DIRECTION_TEACH_EXPIRED);
    assert(!s_direction_teach.active);
    assert(teach(EXO_HIP_JOINT_LEFT,HIP_DIRECTION_TEACH_BEGIN,8)==ESP_OK);
    pose(20,0);
    assert(teach(EXO_HIP_JOINT_LEFT,HIP_DIRECTION_TEACH_FINISH,7)!=ESP_OK);
    assert(reason==HIP_DIRECTION_TEACH_TOKEN_MISMATCH);
    assert(teach(EXO_HIP_JOINT_RIGHT,HIP_DIRECTION_TEACH_FINISH,8)!=ESP_OK);
    assert(reason==HIP_DIRECTION_TEACH_TOKEN_MISMATCH);
    close_session();
    assert(hip_joint_get_snapshot(EXO_HIP_JOINT_LEFT,&snap) && !snap.zero_valid);
    puts("PASS expiry, token/side mismatch and cancel without restoring old zero");

    reset();
    motors[0].mode=RS01_MODE_STATUS_RUNNING;
    assert(teach(EXO_HIP_JOINT_LEFT,HIP_DIRECTION_TEACH_BEGIN,7)!=ESP_OK);
    assert(reason==HIP_DIRECTION_TEACH_MOTOR_NOT_REST);
    motors[0].mode=RS01_MODE_STATUS_RESET; motors[1].faults.present=true;
    assert(teach(EXO_HIP_JOINT_LEFT,HIP_DIRECTION_TEACH_BEGIN,7)!=ESP_OK);
    assert(reason==HIP_DIRECTION_TEACH_MOTOR_FAULT);
    motors[1].faults.present=false; window_motion=2;
    assert(teach(EXO_HIP_JOINT_LEFT,HIP_DIRECTION_TEACH_BEGIN,7)!=ESP_OK);
    assert(reason==HIP_DIRECTION_TEACH_NOT_STILL);
    window_motion=0; motors[0].sample.velocity_rad_s=0; motors[1].continuous_position_rad=NAN;
    assert(teach(EXO_HIP_JOINT_LEFT,HIP_DIRECTION_TEACH_BEGIN,7)!=ESP_OK);
    assert(reason==HIP_DIRECTION_TEACH_FEEDBACK_INVALID);
    motors[1].continuous_position_rad=FLT_MAX;
    assert(teach(EXO_HIP_JOINT_LEFT,HIP_DIRECTION_TEACH_BEGIN,7)!=ESP_OK);
    assert(reason==HIP_DIRECTION_TEACH_FEEDBACK_INVALID);
    puts("PASS running motor, fault, moving pose and non-finite feedback rejected");

    reset(); pose(719,0);
    assert(teach(EXO_HIP_JOINT_LEFT,HIP_DIRECTION_TEACH_BEGIN,7)==ESP_OK);
    pose(729,0); motors[1].sample.position_rad=exo_degrees_to_radians(-711);
    assert(teach(EXO_HIP_JOINT_LEFT,HIP_DIRECTION_TEACH_FINISH,7)==ESP_OK);
    assert(learned==EXO_DIRECTION_POSITIVE);
    puts("PASS direction uses continuous angle across RS01 periodic wrap");

    exo_ble_request_t req;
    const uint8_t begin[]={1,0x1a,3,3,0,0,7};
    assert(exo_ble_protocol_decode_request(begin,sizeof(begin),&req)==EXO_BLE_RESULT_OK);
    assert(req.side==EXO_BLE_SIDE_LEFT && req.direction_teach_action==0 && req.direction_teach_token==7);
    for (unsigned i=0;i<3U;++i) {
        uint8_t bad[7]; memcpy(bad,begin,sizeof(bad));
        bad[4+i]=i==0?2:i==1?3:0;
        assert(exo_ble_protocol_decode_request(bad,sizeof(bad),&req)==EXO_BLE_RESULT_INVALID_PAYLOAD);
    }
    assert(exo_ble_protocol_decode_request(begin,sizeof(begin)-1,&req)==EXO_BLE_RESULT_INVALID_LENGTH);
    const exo_ble_ack_t ack={.result=EXO_BLE_RESULT_OK,.state=EXO_BLE_STATE_IDLE,
        .system_flags=0x21,.detail=EXO_BLE_DIRECTION_NEGATIVE};
    uint8_t frame[20]; size_t length=0;
    assert(exo_ble_protocol_encode_ack(0x1a,3,&ack,frame,sizeof(frame),&length));
    const uint8_t expected[]={1,0x9a,3,4,0,1,0x21,2};
    assert(length==8 && memcmp(frame,expected,8)==0);
    puts("PASS byte protocol: golden request/ACK, malformed side/action/token/length");

    reset();
    assert(teach(EXO_HIP_JOINT_LEFT,HIP_DIRECTION_TEACH_BEGIN,7)==ESP_OK);
    pose(-10,0); assert(teach(EXO_HIP_JOINT_LEFT,HIP_DIRECTION_TEACH_FINISH,7)==ESP_OK);
    pose(0,0); assert(hip_joint_set_zero_from_current(EXO_HIP_JOINT_LEFT)==ESP_OK);
    assert(teach(EXO_HIP_JOINT_RIGHT,HIP_DIRECTION_TEACH_BEGIN,8)==ESP_OK);
    pose(0,10); assert(teach(EXO_HIP_JOINT_RIGHT,HIP_DIRECTION_TEACH_FINISH,8)==ESP_OK);
    pose(0,0); assert(hip_joint_set_zero_from_current(EXO_HIP_JOINT_RIGHT)==ESP_OK);
    const unsigned writes_before_reboot=storage_save_calls;
    memset(s_joints,0,sizeof(s_joints)); s_initialized=false;
    memset(&s_direction_teach,0,sizeof(s_direction_teach));
    assert(hip_joint_init()==ESP_OK);
    assert(hip_joint_get_snapshot(EXO_HIP_JOINT_LEFT,&snap) && snap.direction_valid &&
        snap.direction==EXO_DIRECTION_NEGATIVE && !snap.zero_valid);
    assert(hip_joint_get_snapshot(EXO_HIP_JOINT_RIGHT,&snap) && snap.direction_valid &&
        snap.direction==EXO_DIRECTION_POSITIVE && !snap.zero_valid);
    assert(storage_save_calls==writes_before_reboot);
    puts("PASS factory directions restore after simulated restart; wearing zero never persists");

    reset(); assert(teach(EXO_HIP_JOINT_LEFT,HIP_DIRECTION_TEACH_BEGIN,7)==ESP_OK);
    pose(-10,0); storage_commit_error=ESP_FAIL;
    expect_failure(EXO_HIP_JOINT_LEFT,HIP_DIRECTION_TEACH_STORAGE_FAILED);
    assert(hip_joint_get_snapshot(EXO_HIP_JOINT_LEFT,&snap) && !snap.direction_valid && !snap.zero_valid);
    assert(hip_joint_set_zero_from_current(EXO_HIP_JOINT_LEFT)!=ESP_OK);
    storage_commit_error=0;
    assert(teach(EXO_HIP_JOINT_LEFT,HIP_DIRECTION_TEACH_FINISH,7)==ESP_OK);
    storage_init_error=ESP_FAIL;
    assert(exo_factory_direction_storage_init()==ESP_FAIL);
    storage_init_error=0;
    puts("PASS NVS errors are reported without publishing a ready direction; retry can succeed");

    exo_direction_t loaded;
    assert(exo_factory_direction_load(EXO_HIP_JOINT_LEFT,EXO_ACTUATOR_A,0x20,&loaded)==ESP_OK && loaded==EXO_DIRECTION_UNKNOWN);
    assert(exo_factory_direction_load(EXO_HIP_JOINT_LEFT,EXO_ACTUATOR_B,0x21,&loaded)==ESP_OK && loaded==EXO_DIRECTION_UNKNOWN);
    flash_records[0]^=0x01000000U;
    assert(exo_factory_direction_load(EXO_HIP_JOINT_LEFT,EXO_ACTUATOR_B,0x20,&loaded)==ESP_OK && loaded==EXO_DIRECTION_UNKNOWN);
    memset(flash_valid,0,sizeof(flash_valid));
    memset(s_joints,0,sizeof(s_joints)); s_initialized=false;
    memset(&s_direction_teach,0,sizeof(s_direction_teach));
    assert(hip_joint_init()==ESP_OK);
    assert(hip_joint_get_snapshot(EXO_HIP_JOINT_LEFT,&snap) && !snap.direction_valid && !snap.zero_valid);
    assert(hip_joint_set_zero_from_current(EXO_HIP_JOINT_LEFT)!=ESP_OK);
    assert(teach(EXO_HIP_JOINT_LEFT,HIP_DIRECTION_TEACH_BEGIN,7)==ESP_OK);
    pose(-20,0); assert(teach(EXO_HIP_JOINT_LEFT,HIP_DIRECTION_TEACH_FINISH,7)==ESP_OK);
    puts("PASS missing, wrong-version and mismatched mapping records fail closed; first teaching works");
    puts("RESULT 14/14 PASS");
    return 0;
}
