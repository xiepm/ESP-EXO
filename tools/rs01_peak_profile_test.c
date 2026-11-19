/* Actual driver encoder, no hardware access. Independent +/-15 Nm vectors. */
#define main feedback_frames_main
#include "phase10_feedback_frames_replay.c"
#undef main
int main(void) {
    assert(feedback_frames_main()==0);
    reset();
    assert(EXO_MOTOR_WIRE_TORQUE_MAX==17.f && EXO_MOTOR_WIRE_SPEED_MAX==44.f);
    rs01_motion_command_t cmd={.velocity_rad_s=44.f,.torque_nm=15.f};
    uint8_t payload[8];uint16_t torque;
    assert(encode_motion_command(EXO_ACTUATOR_A,&cmd,payload,&torque)==ESP_OK);
    assert(torque==61680 && payload[2]==255 && payload[3]==255);
    cmd.torque_nm=-15.f;
    assert(encode_motion_command(EXO_ACTUATOR_B,&cmd,payload,&torque)==ESP_OK);
    assert(torque==3855);
    cmd.torque_nm=17.01f;
    assert(encode_motion_command(EXO_ACTUATOR_A,&cmd,payload,&torque)==ESP_ERR_INVALID_ARG);
    puts("PASS RS01 +/-15 Nm wire vectors,44 rad/s and range rejection");
    return 0;
}
