/* Real NVS encoding, joint layer and service; synthetic feedback only. */
#define PHASE10_SAVED_ZERO_MAIN historical_saved_main
#include "phase10_saved_zero_replay.c"
int main(void) {
    assert(historical_saved_main()==0);
    zero_reset();capture_save(7,-24);reboot_joints();
    assert(exo_wear_zero_auto_restore_pending());assert_zeros(false);
    motors[0].online=false;
    assert(exo_wear_zero_execute(EXO_WEAR_ZERO_CAPTURE_SAVE_PAIR,true,&zero_reason)!=ESP_OK);
    assert(exo_wear_zero_auto_restore_pending());
    assert(exo_wear_zero_execute(EXO_WEAR_ZERO_RESTORE,true,&zero_reason)!=ESP_OK);
    assert(exo_wear_zero_auto_restore_pending());
    for(unsigned i=0;i<10;i++)exo_wear_zero_restore_saved_if_needed();
    assert_zeros(false);assert(exo_wear_zero_auto_restore_pending()&&zero_writes==1);
    motors[0].online=true;pose(32,-44);
    exo_wear_zero_restore_saved_if_needed();assert_zeros(true);
    hip_joint_snapshot_t l,r;
    assert(hip_joint_get_snapshot(0,&l)&&hip_joint_get_snapshot(1,&r));
    assert(fabsf(l.joint_sample.position_deg-25)<.001f&&fabsf(r.joint_sample.position_deg-20)<.001f);
    assert(!exo_wear_zero_auto_restore_pending()&&zero_writes==1);
    for(unsigned i=0;i<5;i++) {
        reboot_joints();pose(32,-44);exo_wear_zero_restore_saved_if_needed();assert_zeros(true);
        assert(zero_writes==1);
    }
    puts("PASS persistent 1: delayed feedback, raised-leg boot, five power cycles retain offsets/directions without NVS rewrites");
    ++motors[0].position_epoch;
    assert(hip_joint_get_snapshot(1,&r)&&!r.zero_valid);
    pose(32,-44);exo_wear_zero_restore_saved_if_needed();assert_zeros(true);
    assert(zero_writes==1);
    puts("PASS persistent 2: motor-only power cycle can rebind the same reference during IDLE maintenance without saving a new zero");

    reboot_joints();exo_wear_zero_cancel_auto_restore();pose(7,-24);
    exo_wear_zero_restore_saved_if_needed();assert_zeros(false);
    zero_reset();capture_save(7,-24);reboot_joints();
    s_joints[0].direction=EXO_DIRECTION_NEGATIVE;
    exo_wear_zero_restore_saved_if_needed();assert_zeros(false);assert(!exo_wear_zero_auto_restore_pending());
    zero_reset();capture_save(7,-24);zero_flash[0]=0;reboot_joints();
    exo_wear_zero_restore_saved_if_needed();assert_zeros(false);assert(!exo_wear_zero_auto_restore_pending());
    puts("PASS persistent 3: explicit cancellation, wrong binding and damaged record never restore");

    zero_reset();pose(15,-33);
    assert(exo_wear_zero_execute(EXO_WEAR_ZERO_CAPTURE_SAVE_PAIR,true,&zero_reason)==ESP_OK);
    assert(zero_flash_valid&&zero_writes==1);assert_zeros(true);
    reboot_joints();pose(16,-34);exo_wear_zero_restore_saved_if_needed();assert_zeros(true);
    assert(hip_joint_get_snapshot(0,&l)&&fabsf(l.joint_sample.position_deg-1)<.001f);
    pose(17,-35);zero_commit_error=ESP_FAIL;
    assert(exo_wear_zero_execute(EXO_WEAR_ZERO_CAPTURE_SAVE_PAIR,true,&zero_reason)!=ESP_OK);
    assert(zero_reason==EXO_WEAR_ZERO_STORAGE_FAILED);assert_zeros(false);
    assert(fabsf(record_zero(zero_flash,0)-15)<.001f);
    puts("PASS persistent 4: capture/save/readback in one request; failed commit cannot enable START or corrupt previous pair");

    zero_reset();capture_save(7,-24);motors[0].online=false;
    hip_direction_teach_result_t why;exo_direction_t direction;
    assert(exo_wear_zero_teach_direction(0,HIP_DIRECTION_TEACH_BEGIN,7,&why,&direction)!=ESP_OK);
    assert(zero_flash_valid&&why==HIP_DIRECTION_TEACH_FEEDBACK_INVALID);
    motors[0].online=true;reboot_joints();pose(7,-24);exo_wear_zero_restore_saved_if_needed();assert_zeros(true);
    puts("PASS persistent 5: rejected direction teaching preserves existing factory calibration and saved zero");
    return 0;
}
