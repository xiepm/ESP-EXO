/* Actual driver encoding/decoding; no hardware or motor motion. */
#define CONFIG_EXO_MOTOR_RS05 1
#define main feedback_frames_main
#include "phase10_feedback_frames_replay.c"
#undef main
int main(void) {
    feedback_frames_main();
    reset();
    rs01_motion_command_t cmd={.velocity_rad_s=50.f,.torque_nm=5.5f};
    uint8_t payload[8]; uint16_t torque;
    assert(encode_motion_command(EXO_ACTUATOR_A,&cmd,payload,&torque)==ESP_OK);
    assert(torque==65535 && payload[2]==255 && payload[3]==255);
    cmd.torque_nm=.5f;
    assert(encode_motion_command(EXO_ACTUATOR_A,&cmd,payload,&torque)==ESP_OK);
    assert(torque==35746); /* independent RS05 vector: (0.5+5.5)/11*65535 */
    cmd.torque_nm=5.6f;
    assert(encode_motion_command(EXO_ACTUATOR_A,&cmd,payload,&torque)==ESP_ERR_INVALID_ARG);
    twai_message_t frame={.identifier=0x180020fd,.flags=TWAI_MSG_FLAG_EXTD,
        .data_length_code=8,.data={0x82,0x0b,0x7f,0xfb,0x7f,0xff,0x01,0x03}};
    exo_actuator_id_t slot=EXO_ACTUATOR_COUNT;
    assert(rs01_process_can_frame(&frame,&slot)==RS01_RX_EVENT_FEEDBACK);
    assert(slot==EXO_ACTUATOR_B);
    rs01_feedback_snapshot_t f;assert(rs01_get_feedback(slot,&f));
    assert(f.valid && f.online && !f.enabled_requested);
    assert(fabsf(f.sample.velocity_rad_s - (-0.00686656f)) < .00001f);
    assert(fabsf(f.sample.torque_nm - (-0.0000839246f)) < .000001f);
    puts("PASS RS05 0.5Nm wire scale, speed/torque endpoints, range rejection and recorded report frame");
}
