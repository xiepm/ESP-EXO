/* 无硬件主机回放：编译真实关节/穿戴服务/协议，复用已有NVS和反馈Fake。 */
#define PHASE10_SAVED_ZERO_MAIN original_saved_main
#include "phase10_saved_zero_replay.c"

static void current_pair(hip_joint_snapshot_t joints[2], rs01_feedback_snapshot_t feedback[2]) {
    assert(read_pair(joints, feedback) == EXO_WEAR_ZERO_OK);
}
static void unchanged_capture_reject(exo_wear_zero_result_t expected) {
    hip_joint_t before[2]; memcpy(before, s_joints, sizeof(before));
    assert(exo_wear_zero_execute(EXO_WEAR_ZERO_CAPTURE_PAIR, true, &zero_reason) != ESP_OK);
    assert(zero_reason == expected && memcmp(before, s_joints, sizeof(before)) == 0);
}
static void unchanged_commit_reject(const hip_joint_snapshot_t joints[2],
    const rs01_feedback_snapshot_t feedback[2]) {
    hip_joint_t before[2]; memcpy(before, s_joints, sizeof(before));
    assert(hip_joint_capture_zero_pair(joints, feedback) == ESP_ERR_INVALID_STATE);
    assert(memcmp(before, s_joints, sizeof(before)) == 0);
}
static void assert_neutral_pair(void) {
    for (unsigned i=0; i<2U; ++i) {
        hip_joint_snapshot_t joint;
        assert(hip_joint_get_snapshot((exo_hip_joint_id_t)i, &joint));
        const rs01_feedback_snapshot_t *motor = &motors[joint.actuator];
        assert(joint.joint_valid && joint.zero_valid && fabsf(joint.joint_sample.position_deg)<0.002f);
        assert(fabsf(joint.software_zero_motor_deg-exo_radians_to_degrees(motor->continuous_position_rad))<0.002f);
        assert(fabsf(joint.software_zero_periodic_motor_deg-exo_radians_to_degrees(motor->sample.position_rad))<0.002f);
        assert(joint.zero_source_timestamp_us==motor->timestamp_us);
        assert(joint.zero_source_feedback_count==motor->feedback_count);
        assert(joint.zero_source_position_epoch==motor->position_epoch);
    }
}
int main(void) {
    /* 原有沿用/NVS及方向用例一起回归，action=3不能改变沿用语义。 */
    assert(original_saved_main()==0);
    zero_reset(); s_joints[0].direction=EXO_DIRECTION_NEGATIVE;
    s_joints[1].direction=EXO_DIRECTION_POSITIVE;
    capture_save(-1.49f, -26.76f); pose(-1.49f,-6.25f);
    hip_joint_snapshot_t joint;
    assert(hip_joint_get_snapshot(EXO_HIP_JOINT_RIGHT,&joint));
    assert(fabsf(joint.joint_sample.position_deg-20.51f)<0.002f);
    /* 两侧已有有效旧零位，仍必须一起覆盖，而不是仅填补zero_valid=false。 */
    assert(exo_wear_zero_execute(EXO_WEAR_ZERO_CAPTURE_PAIR,true,&zero_reason)==ESP_OK);
    assert(zero_reason==EXO_WEAR_ZERO_OK); assert_neutral_pair(); assert(zero_writes==1);
    assert(fabsf(record_zero(zero_flash,1)+26.76f)<0.002f); /* 未勾选保存时旧候选不变。 */
    puts("PASS pair-zero 1: reproduces valid left0/right20.51; both CURRENT positions become neutral, old candidate untouched");

    zero_reset(); pose(8,-24);
    assert(exo_wear_zero_execute(EXO_WEAR_ZERO_CAPTURE_PAIR,false,&zero_reason)==ESP_ERR_INVALID_STATE);
    assert(zero_reason==EXO_WEAR_ZERO_NOT_CONFIRMED); assert_zeros(false);
    assert(exo_wear_zero_execute(EXO_WEAR_ZERO_CAPTURE_PAIR,true,&zero_reason)==ESP_OK);
    assert_neutral_pair(); assert(zero_writes==0);
    assert(exo_wear_zero_execute(EXO_WEAR_ZERO_SAVE_CURRENT,true,&zero_reason)==ESP_OK);
    assert(zero_writes==1); reboot_joints(); assert_zeros(false); pose(9,-24);
    assert(exo_wear_zero_execute(EXO_WEAR_ZERO_RESTORE,true,&zero_reason)==ESP_OK);
    assert(hip_joint_get_snapshot(EXO_HIP_JOINT_LEFT,&joint) && fabsf(joint.joint_sample.position_deg-1)<0.002f);
    puts("PASS pair-zero 2: initial capture requires confirmation; optional save and later reuse preserve offset");

    for(unsigned side=0; side<2U; ++side) {
        zero_reset(); capture_save(7,-24); pose(9,-22);
        const exo_actuator_id_t actuator=side==0?EXO_ACTUATOR_B:EXO_ACTUATOR_A;
        motors[actuator].online=false; unchanged_capture_reject(EXO_WEAR_ZERO_FEEDBACK_INVALID);
        motors[actuator].online=true; motors[actuator].timestamp_us=(uint64_t)clock_us-50001;
        unchanged_capture_reject(EXO_WEAR_ZERO_FEEDBACK_INVALID); pose(9,-22);
        motors[actuator].sample.velocity_rad_s=exo_degrees_to_radians(6);
        unchanged_capture_reject(EXO_WEAR_ZERO_NOT_STILL); motors[actuator].sample.velocity_rad_s=0;
        motors[actuator].faults.present=true; unchanged_capture_reject(EXO_WEAR_ZERO_MOTOR_FAULT);
        motors[actuator].faults.present=false; motors[actuator].mode=RS01_MODE_STATUS_RUNNING;
        unchanged_capture_reject(EXO_WEAR_ZERO_MOTOR_NOT_REST); motors[actuator].mode=RS01_MODE_STATUS_RESET;
        motors[actuator].sample.position_rad=NAN; unchanged_capture_reject(EXO_WEAR_ZERO_FEEDBACK_INVALID);
    }
    puts("PASS pair-zero 3: either-side offline/stale/moving/fault/enabled/NaN rejects without updating either old zero");

    zero_reset(); capture_save(7,-24); pose(9,-22);
    s_joints[1].direction_valid=false; unchanged_capture_reject(EXO_WEAR_ZERO_BINDING_CHANGED);
    s_joints[1].direction_valid=true; s_direction_teach.active=true;
    s_direction_teach.started_us=(uint64_t)clock_us;
    unchanged_capture_reject(EXO_WEAR_ZERO_EPOCH_CHANGED); s_direction_teach.active=false;
    s_joints[0].direction=EXO_DIRECTION_NEGATIVE; s_joints[1].direction=EXO_DIRECTION_POSITIVE;
    assert(exo_wear_zero_execute(EXO_WEAR_ZERO_CAPTURE_PAIR,true,&zero_reason)==ESP_OK);
    assert_neutral_pair(); pose(8,-21);
    assert(hip_joint_get_snapshot(EXO_HIP_JOINT_LEFT,&joint) && fabsf(joint.joint_sample.position_deg-1)<0.002f);
    assert(hip_joint_get_snapshot(EXO_HIP_JOINT_RIGHT,&joint) && fabsf(joint.joint_sample.position_deg-1)<0.002f);
    puts("PASS pair-zero 4: direction teaching blocks; persisted per-side signs determine subsequent positive human angles");

    zero_reset(); capture_save(7,-24); pose(9,-22);
    hip_joint_snapshot_t joints[2]; rs01_feedback_snapshot_t feedback[2];
    current_pair(joints,feedback); pose(9.2f,-21.8f);
    assert(hip_joint_capture_zero_pair(joints,feedback)==ESP_OK); assert_neutral_pair();
    puts("PASS pair-zero 5: commit uses latest validated current pair and matching source epoch/count/time, not earlier preview");

    current_pair(joints,feedback); ++motors[EXO_ACTUATOR_A].position_epoch;
    unchanged_commit_reject(joints,feedback); --motors[EXO_ACTUATOR_A].position_epoch;
    --motors[EXO_ACTUATOR_A].feedback_count; unchanged_commit_reject(joints,feedback);
    ++motors[EXO_ACTUATOR_A].feedback_count;
    --motors[EXO_ACTUATOR_A].timestamp_us; unchanged_commit_reject(joints,feedback);
    ++motors[EXO_ACTUATOR_A].timestamp_us;
    s_joints[1].direction=EXO_DIRECTION_POSITIVE; unchanged_commit_reject(joints,feedback);
    s_joints[1].direction=EXO_DIRECTION_NEGATIVE;
    pose(10.3f,-21.8f); unchanged_commit_reject(joints,feedback);
    puts("PASS pair-zero 6: epoch/count/time/sign change or over1deg between precheck and capture rejects whole pair");

    zero_reset(); capture_save(7,-24); pose(9,-22);
    current_pair(joints,feedback);
    motors[EXO_ACTUATOR_A].timestamp_us-=20001;
    unchanged_capture_reject(EXO_WEAR_ZERO_EPOCH_CHANGED); pose(9,-22);
    current_pair(joints,feedback); motors[EXO_ACTUATOR_A].sample.position_rad=exo_degrees_to_radians(721);
    unchanged_commit_reject(joints,feedback);
    puts("PASS pair-zero 7: over20ms pair skew or out-of-protocol position rejects, preserving both zeros");

    zero_reset(); pose(-719,719);
    motors[EXO_ACTUATOR_B].continuous_position_rad=exo_degrees_to_radians(721);
    motors[EXO_ACTUATOR_A].continuous_position_rad=exo_degrees_to_radians(-721);
    assert(exo_wear_zero_execute(EXO_WEAR_ZERO_CAPTURE_PAIR,true,&zero_reason)==ESP_OK);
    assert_neutral_pair(); assert(zero_writes==0);
    uint8_t frame[]={1,0x1f,0x42,2,3,1}; exo_ble_request_t request;
    assert(exo_ble_protocol_decode_request(frame,sizeof(frame),&request)==EXO_BLE_RESULT_OK && request.saved_zero_action==3);
    frame[5]=0; assert(exo_ble_protocol_decode_request(frame,sizeof(frame),&request)==EXO_BLE_RESULT_INVALID_PAYLOAD);
    puts("PASS pair-zero 8: continuous/periodic coordinates remain distinct at wrap; BLE single request requires confirmation");
    puts("pair-zero replay: 8/8 PASS (target calibration HIL NOT RUN)");
    return 0;
}
