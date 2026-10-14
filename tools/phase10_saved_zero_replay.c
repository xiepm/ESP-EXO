/* 使用真实关节/NVS服务/协议，仅替换Flash、反馈及RTOS。不连接串口或电机。 */
#define main original_direction_main
#define nvs_open direction_nvs_open
#define nvs_commit direction_nvs_commit
#define nvs_close direction_nvs_close
#include "phase10_direction_teaching_replay.c"
#undef main
#undef nvs_open
#undef nvs_commit
#undef nvs_close

static bool zero_flash_valid, zero_pending_valid, zero_erase_pending;
static uint32_t zero_flash[5], zero_pending[5];
static esp_err_t zero_commit_error;
static bool zero_readback_corrupt;
static unsigned zero_writes;
esp_err_t nvs_open(const char *name, nvs_open_mode_t mode, nvs_handle_t *out) {
    if (strcmp(name,"exo_factory")==0) return direction_nvs_open(name,mode,out);
    assert(strcmp(name,"exo_wear")==0);
    if (mode==NVS_READONLY && !zero_flash_valid) return ESP_ERR_NVS_NOT_FOUND;
    *out=2; return ESP_OK;
}
esp_err_t nvs_get_blob(nvs_handle_t h,const char *key,void *data,size_t *size) {
    assert(h==2 && strcmp(key,"pair_v1")==0);
    if (!zero_flash_valid) return ESP_ERR_NVS_NOT_FOUND;
    if (*size<sizeof(zero_flash)) return ESP_ERR_NVS_INVALID_LENGTH;
    memcpy(data,zero_flash,sizeof(zero_flash)); *size=sizeof(zero_flash);
    if (zero_readback_corrupt) ((uint32_t *)data)[3]^=1U;
    return ESP_OK;
}
esp_err_t nvs_set_blob(nvs_handle_t h,const char *key,const void *data,size_t size) {
    assert(h==2 && strcmp(key,"pair_v1")==0 && size==sizeof(zero_pending));
    memcpy(zero_pending,data,size); zero_pending_valid=true; ++zero_writes; return ESP_OK;
}
esp_err_t nvs_erase_key(nvs_handle_t h,const char *key) {
    assert(h==2 && strcmp(key,"pair_v1")==0); zero_erase_pending=true;
    return zero_flash_valid?ESP_OK:ESP_ERR_NVS_NOT_FOUND;
}
esp_err_t nvs_commit(nvs_handle_t h) {
    if (h==1) return direction_nvs_commit(h);
    assert(h==2); if(zero_commit_error) return zero_commit_error;
    if(zero_erase_pending) zero_flash_valid=false;
    if(zero_pending_valid) {memcpy(zero_flash,zero_pending,sizeof(zero_flash)); zero_flash_valid=true;}
    return ESP_OK;
}
void nvs_close(nvs_handle_t h) {
    if(h==1) {direction_nvs_close(h); return;}
    assert(h==2); zero_pending_valid=zero_erase_pending=false;
}
#include "../main/exo_wear_zero.c"
static exo_wear_zero_result_t zero_reason;
static void zero_reset(void) {
    zero_flash_valid=zero_pending_valid=zero_erase_pending=zero_readback_corrupt=false;
    zero_commit_error=0; zero_writes=0;
    reset(); assert(exo_wear_zero_init()==ESP_OK);
}
static void capture_save(float left,float right) {
    pose(left,right);
    for(unsigned i=0;i<2;++i) assert(hip_joint_set_zero_from_current((exo_hip_joint_id_t)i)==ESP_OK);
    assert(exo_wear_zero_execute(EXO_WEAR_ZERO_SAVE_CURRENT,true,&zero_reason)==ESP_OK);
}
static void reboot_joints(void) {
    memset(s_joints,0,sizeof(s_joints)); s_initialized=false;
    memset(&s_direction_teach,0,sizeof(s_direction_teach));
    for(unsigned i=0;i<2;++i) ++motors[i].position_epoch;
    assert(hip_joint_init()==ESP_OK); assert(exo_wear_zero_init()==ESP_OK);
}
static void assert_zeros(bool expected) {
    for(unsigned i=0;i<2;++i) {hip_joint_snapshot_t s;
        assert(hip_joint_get_snapshot((exo_hip_joint_id_t)i,&s)); assert(s.zero_valid==expected);}
}
static void restore_reject(exo_wear_zero_result_t why) {
    assert(exo_wear_zero_execute(EXO_WEAR_ZERO_RESTORE,true,&zero_reason)!=ESP_OK);
    assert(zero_reason==why); assert_zeros(false);
}
#ifndef PHASE10_SAVED_ZERO_MAIN
#define PHASE10_SAVED_ZERO_MAIN main
#endif
int PHASE10_SAVED_ZERO_MAIN(void) {
    assert(original_direction_main()==0);
    zero_reset(); assert_zeros(false);
    restore_reject(EXO_WEAR_ZERO_NO_RECORD);
    assert(exo_wear_zero_execute(EXO_WEAR_ZERO_SAVE_CURRENT,true,&zero_reason)!=ESP_OK);
    assert(zero_reason==EXO_WEAR_ZERO_CURRENT_ZERO_INVALID);
    puts("PASS saved-zero 1: missing record and invalid current zeros rejected");

    capture_save(7,-24); assert(zero_writes==1); reboot_joints(); assert_zeros(false);
    assert(exo_wear_zero_execute(EXO_WEAR_ZERO_RESTORE,false,&zero_reason)!=ESP_OK);
    assert(zero_reason==EXO_WEAR_ZERO_NOT_CONFIRMED); assert_zeros(false);
    pose(8,-23);
    assert(exo_wear_zero_execute(EXO_WEAR_ZERO_RESTORE,true,&zero_reason)==ESP_OK);
    hip_joint_snapshot_t l,r;
    assert(hip_joint_get_snapshot(EXO_HIP_JOINT_LEFT,&l) && hip_joint_get_snapshot(EXO_HIP_JOINT_RIGHT,&r));
    assert(l.joint_valid && r.joint_valid && fabsf(l.joint_sample.position_deg-1)<0.001f);
    assert(fabsf(r.joint_sample.position_deg+1)<0.001f);
    assert(l.zero_source_position_epoch==2 && r.zero_source_position_epoch==2);
    assert(zero_writes==1);
    puts("PASS saved-zero 2: Flash survives reboot; no auto activation; confirmed restore binds new epoch");

    reboot_joints(); pose(13,-24); restore_reject(EXO_WEAR_ZERO_POSTURE_MISMATCH);
    exo_wear_zero_status_t preview;
    assert(exo_wear_zero_get_status(EXO_HIP_JOINT_LEFT,&preview));
    assert(preview.stored && preview.preview_valid && !preview.reuse_ready && preview.preview_joint_deg>5);
    pose(7,-30); restore_reject(EXO_WEAR_ZERO_POSTURE_MISMATCH);
    puts("PASS saved-zero 3: either-side neutral deviation rejects whole pair; preview remains informative");

    pose(7,-24); motors[EXO_ACTUATOR_B].online=false;
    restore_reject(EXO_WEAR_ZERO_FEEDBACK_INVALID); motors[EXO_ACTUATOR_B].online=true;
    clock_us+=60000; restore_reject(EXO_WEAR_ZERO_FEEDBACK_INVALID); pose(7,-24);
    motors[EXO_ACTUATOR_A].sample.velocity_rad_s=exo_degrees_to_radians(6);
    restore_reject(EXO_WEAR_ZERO_NOT_STILL); motors[EXO_ACTUATOR_A].sample.velocity_rad_s=0;
    motors[EXO_ACTUATOR_A].faults.present=true; restore_reject(EXO_WEAR_ZERO_MOTOR_FAULT);
    motors[EXO_ACTUATOR_A].faults.present=false; motors[EXO_ACTUATOR_A].mode=RS01_MODE_STATUS_RUNNING;
    restore_reject(EXO_WEAR_ZERO_MOTOR_NOT_REST); motors[EXO_ACTUATOR_A].mode=RS01_MODE_STATUS_RESET;
    motors[EXO_ACTUATOR_A].sample.position_rad=NAN; restore_reject(EXO_WEAR_ZERO_FEEDBACK_INVALID);
    pose(7,-24);
    puts("PASS saved-zero 4: offline/stale/moving/fault/enabled/nonfinite feedback reject without partial restore");

    s_joints[0].direction=EXO_DIRECTION_NEGATIVE;
    restore_reject(EXO_WEAR_ZERO_BINDING_CHANGED); s_joints[0].direction=EXO_DIRECTION_POSITIVE;
    zero_commit_error=ESP_FAIL;
    hip_direction_teach_result_t why; exo_direction_t direction;
    assert(exo_wear_zero_teach_direction(EXO_HIP_JOINT_LEFT,HIP_DIRECTION_TEACH_BEGIN,7,&why,&direction)!=ESP_OK);
    assert(why==HIP_DIRECTION_TEACH_STORAGE_FAILED && !s_direction_teach.active);
    zero_commit_error=0;
    assert(exo_wear_zero_teach_direction(EXO_HIP_JOINT_LEFT,HIP_DIRECTION_TEACH_BEGIN,7,&why,&direction)==ESP_OK);
    assert(!zero_flash_valid); close_session();
    puts("PASS saved-zero 5: mapping/sign mismatch and direction teaching invalidate old candidates");

    zero_reset(); capture_save(719,-719); reboot_joints();
    pose(-719,719);
    motors[EXO_ACTUATOR_B].continuous_position_rad=exo_degrees_to_radians(721);
    motors[EXO_ACTUATOR_A].continuous_position_rad=exo_degrees_to_radians(-721);
    assert(exo_wear_zero_execute(EXO_WEAR_ZERO_RESTORE,true,&zero_reason)==ESP_OK);
    assert(hip_joint_get_snapshot(EXO_HIP_JOINT_LEFT,&l) && hip_joint_get_snapshot(EXO_HIP_JOINT_RIGHT,&r));
    assert(fabsf(l.software_zero_motor_deg-719)<0.002f && fabsf(r.software_zero_motor_deg+719)<0.002f);
    assert(fabsf(l.joint_sample.position_deg-2)<0.002f && fabsf(r.joint_sample.position_deg-2)<0.002f);
    puts("PASS saved-zero 6: periodic boundary restored into current continuous coordinates, correct signs");

    zero_reset(); capture_save(7,-24); pose(8,-23);
    assert(hip_joint_set_zero_from_current(EXO_HIP_JOINT_LEFT)==ESP_OK);
    assert(hip_joint_set_zero_from_current(EXO_HIP_JOINT_RIGHT)==ESP_OK);
    zero_commit_error=ESP_FAIL;
    assert(exo_wear_zero_execute(EXO_WEAR_ZERO_SAVE_CURRENT,true,&zero_reason)!=ESP_OK);
    assert(zero_reason==EXO_WEAR_ZERO_STORAGE_FAILED);
    assert(exo_wear_zero_get_status(EXO_HIP_JOINT_LEFT,&preview) && fabsf(preview.saved_periodic_motor_deg-7)<0.001f);
    zero_commit_error=0; zero_readback_corrupt=true;
    assert(exo_wear_zero_execute(EXO_WEAR_ZERO_SAVE_CURRENT,true,&zero_reason)!=ESP_OK);
    assert(zero_reason==EXO_WEAR_ZERO_STORAGE_FAILED); zero_readback_corrupt=false;
    zero_flash[0]=0; reboot_joints(); restore_reject(EXO_WEAR_ZERO_BAD_RECORD);
    puts("PASS saved-zero 7: failed commit/readback not published; malformed records do not activate zeros");

    zero_reset(); capture_save(7,-24);
    assert(exo_wear_zero_execute(EXO_WEAR_ZERO_FORGET,true,&zero_reason)==ESP_OK);
    assert_zeros(true); assert(!zero_flash_valid); reboot_joints(); assert_zeros(false);
    puts("PASS saved-zero 8: forget removes candidate, preserves current RAM and factory directions");

    uint8_t frame[20]; size_t size;
    exo_ble_saved_zero_status_t status={.side=EXO_BLE_SIDE_LEFT,.actuator=1,.can_id=0x20,
        .direction=EXO_BLE_DIRECTION_POSITIVE,.flags=15,.reason=0,.saved_zero_mdeg=7000,.preview_joint_mdeg=1000};
    assert(exo_ble_protocol_encode_saved_zero(0x42,&status,frame,sizeof(frame),&size) && size==20);
    const uint8_t golden[20]={1,0x9e,0x42,16,0,1,0x20,1,15,0,0,0,0x58,0x1b,0,0,0xe8,3,0,0};
    assert(memcmp(frame,golden,20)==0);
    status.preview_joint_mdeg=5001; assert(!exo_ble_protocol_encode_saved_zero(0x42,&status,frame,sizeof(frame),&size));
    exo_ble_request_t request;
    uint8_t action[]={1,0x1f,0x43,2,1,1};
    assert(exo_ble_protocol_decode_request(action,6,&request)==EXO_BLE_RESULT_OK && request.saved_zero_action==1);
    action[5]=0; assert(exo_ble_protocol_decode_request(action,6,&request)==EXO_BLE_RESULT_INVALID_PAYLOAD);
    action[5]=1; action[4]=3;
    assert(exo_ble_protocol_decode_request(action,6,&request)==EXO_BLE_RESULT_OK && request.saved_zero_action==3);
    action[4]=4; assert(exo_ble_protocol_decode_request(action,6,&request)==EXO_BLE_RESULT_INVALID_PAYLOAD);
    uint8_t query[]={1,0x1e,0x44,1,1};
    assert(exo_ble_protocol_decode_request(query,5,&request)==EXO_BLE_RESULT_OK && request.side==EXO_BLE_SIDE_RIGHT);
    query[4]=2; assert(exo_ble_protocol_decode_request(query,5,&request)==EXO_BLE_RESULT_INVALID_PAYLOAD);
    puts("PASS saved-zero 9: C golden vectors; strict flags/sentinels/side/action/confirmation and 20-byte limit");
    zero_reset(); capture_save(7,-24); reboot_joints(); pose(7,-24);
    rs01_feedback_snapshot_t expected[2]={motors[EXO_ACTUATOR_B],motors[EXO_ACTUATOR_A]};
    const float zeros[2]={7,-24};
    ++motors[EXO_ACTUATOR_A].position_epoch;
    assert(hip_joint_restore_zero_pair(zeros,expected)==ESP_ERR_INVALID_STATE); assert_zeros(false);
    expected[1]=motors[EXO_ACTUATOR_A];
    pose(8.5f,-24);
    assert(hip_joint_restore_zero_pair(zeros,expected)==ESP_ERR_INVALID_STATE); assert_zeros(false);
    puts("PASS saved-zero 10: between-preview epoch/motion change rejects atomically, no half-pair");
    puts("saved-zero replay: 10/10 PASS (physical persistence/motion HIL NOT RUN)");
    return 0;
}
