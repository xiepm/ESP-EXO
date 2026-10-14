/* 使用真实脉冲算法/助力/控制器/协议。借用原集成夹具仅替代RTOS和CAN。
 * 不打开串口、不连接BLE、不驱动真实电机。时间和反馈序号由测试控制。 */
#define main previous_phase_suite
#include "phase10_assist_integration_replay.c"
#undef main
#include "exo_ble_protocol.h"

static exo_swing_pulse_leg_t leg;
static exo_pulse_timing_t timing;
static exo_pulse_window_t window;
static int64_t now_us;
static uint32_t count;
static void fixture(uint8_t mode) {
    exo_swing_pulse_reset(&leg);
    timing=exo_swing_pulse_default_timing(200);
    timing.duration_mode=mode;
    window=exo_swing_pulse_default_window(1000);
    now_us=1000000;count=0;
}
static float sample(float q,float v) {
    now_us+=10000;
    const exo_swing_pulse_input_t in={.now_us=now_us,.source_us=(uint64_t)now_us,
        .feedback_count=++count,.angle_deg=q,.velocity_deg_s=v,.valid=true};
    return exo_swing_pulse_step(&leg,&timing,&window,1000,100,&in);
}
static bool reset_leg(void) {
    for(int i=0;i<10;i++) ASSERT_TRUE(sample(0,0)==0,"reset has no torque");
    ASSERT_TRUE(leg.state==EXO_PULSE_READY,"80 ms new-feedback reset confirmation");
    return true;
}
static bool new_swing(void) {
    ASSERT_TRUE(sample(10.5f,20)==0,"crossing alone is not a confirmed event");
    ASSERT_TRUE(sample(11,20)==0,"confirmation 10 ms");
    ASSERT_TRUE(sample(11.5f,20)==0,"confirmation 20 ms");
    ASSERT_TRUE(sample(12,20)==0,"new event pulse begins with u=0");
    return true;
}
static bool event_at(int64_t event_time) {
    while(now_us<event_time-40000) sample(0,0);
    ASSERT_TRUE(leg.state==EXO_PULSE_READY,"fresh reset before timed physical event");
    return new_swing();
}
static bool test_fixed_shape_and_no_repeat(void) {
    fixture(EXO_PULSE_FIXED);
    ASSERT_TRUE(reset_leg() && new_swing(),"first event");
    const int64_t start=now_us;
    float previous=0;
    for(int i=1;i<=60;i++) {
        const float actual=sample(12+0.06f*(float)i,6);
        const float u=(float)(now_us-start)/600000.0f;
        const float expected=i==60?0:0.2f*powf(sinf(PI_F*u),2);
        ASSERT_TRUE(fabsf(actual-expected)<1e-6f,"exact frozen sin squared profile");
        ASSERT_TRUE(actual-previous<=0.20001f,"20 N m/s rise bound");
        previous=actual;
    }
    ASSERT_TRUE(leg.state==EXO_PULSE_WAIT_NEXT_RESET && leg.pulse_count==1,"bounded pulse complete");
    for(int i=0;i<1000;i++) ASSERT_TRUE(sample(12,20)==0,"no timer-generated pulse while holding leg");
    ASSERT_TRUE(leg.pulse_count==1,"one pulse despite ten seconds over threshold");
    ASSERT_TRUE(reset_leg() && new_swing() && leg.pulse_count==2,"only new cycle allows second pulse");
    return true;
}
static bool test_start_high_noise_and_wrong_direction(void) {
    fixture(EXO_PULSE_FIXED);
    for(int i=0;i<100;i++) ASSERT_TRUE(sample(12,20)==0,"start above threshold is not a new event");
    ASSERT_TRUE(reset_leg(),"reset");
    sample(10.05f,20);sample(9.95f,-20);
    for(int i=0;i<100;i++) sample(i%2?10.1f:9.9f,20);
    ASSERT_TRUE(leg.pulse_count==0,"threshold jitter cannot fabricate cycle");
    ASSERT_TRUE(reset_leg(),"reset after noise");
    sample(10.5f,-20);sample(11,-20);sample(12,-20);
    ASSERT_TRUE(leg.pulse_count==0,"reverse motion is not a forward swing");
    ASSERT_TRUE(reset_leg() && new_swing() && leg.pulse_count==1,"real new forward event");
    return true;
}
static bool test_early_exits_and_feedback(void) {
    for(int scenario=0;scenario<5;scenario++) {
        fixture(EXO_PULSE_FIXED);
        ASSERT_TRUE(reset_leg() && new_swing(),"pulse");
        for(int i=0;i<15;i++) sample(12+0.06f*(float)i,6);
        ASSERT_TRUE(leg.state==EXO_PULSE_ACTIVE,"before early exit");
        if(scenario==0) ASSERT_TRUE(sample(11.8f,-20)==0 && leg.exit_reason==EXO_PULSE_EXIT_REVERSE,"reverse abort");
        if(scenario==1) ASSERT_TRUE(sample(31,20)==0 && leg.exit_reason==EXO_PULSE_EXIT_WINDOW,"angle window abort");
        if(scenario>=2) {
            exo_swing_pulse_input_t in={.now_us=now_us+10000,.source_us=(uint64_t)now_us+10000,
                .feedback_count=count+1,.angle_deg=12,.velocity_deg_s=0,.valid=true};
            if(scenario==2) in.valid=false;
            if(scenario==3) in.angle_deg=NAN;
            if(scenario==4) in.source_us-=60000;
            ASSERT_TRUE(exo_swing_pulse_step(&leg,&timing,&window,1000,100,&in)==0 &&
                leg.exit_reason==EXO_PULSE_EXIT_FEEDBACK,"invalid/stale/nonfinite feedback immediately zero");
        }
        for(int i=0;i<90;i++) ASSERT_TRUE(sample(12,20)==0,"abort cannot resume old pulse");
        ASSERT_TRUE(reset_leg() && new_swing(),"recovery requires reset and new event");
    }
    return true;
}
static bool test_duplicates_gap_and_time(void) {
    fixture(EXO_PULSE_FIXED);
    exo_swing_pulse_input_t in={.now_us=now_us,.source_us=(uint64_t)now_us,
        .feedback_count=1,.angle_deg=0,.velocity_deg_s=0,.valid=true};
    for(int i=0;i<5;i++) {
        in.now_us+=10000;
        ASSERT_TRUE(exo_swing_pulse_step(&leg,&timing,&window,1000,100,&in)==0,"duplicate no output");
    }
    ASSERT_TRUE(leg.state==EXO_PULSE_WAIT_RESET,"duplicated feedback cannot confirm reset");
    fixture(EXO_PULSE_FIXED);ASSERT_TRUE(reset_leg() && new_swing(),"pulse");
    now_us+=100000;
    ASSERT_TRUE(sample(12,0)==0 && leg.exit_reason==EXO_PULSE_EXIT_FEEDBACK,"sampling gap cancels pulse and history");
    ASSERT_TRUE(leg.last_valid_event_us==0,"invalid feedback clears period anchor");
    return true;
}
static bool test_period_scaled_and_invalid_periods(void) {
    fixture(EXO_PULSE_PERIOD_SCALED);
    const int64_t first=2000000;
    ASSERT_TRUE(event_at(first) && leg.pulse_count==0,"first event warms up period");
    ASSERT_TRUE(event_at(first+1000000) && leg.pulse_count==0,"one interval is not trusted");
    ASSERT_TRUE(event_at(first+2000000) && leg.pulse_count==1,"two same-side valid intervals enable pulse");
    ASSERT_TRUE(leg.frozen_duration_us==300000 && leg.estimated_period_us==1000000,"Ta=r times T");
    for(int i=0;i<40;i++) sample(12,0);
    ASSERT_TRUE(event_at(first+3200000) && fabsf(leg.estimated_period_us-1050000)<1,
        "period smoothing alpha 0.25");
    ASSERT_TRUE(leg.frozen_duration_us==315000,"scaled smoothed duration");
    for(int i=0;i<40;i++) sample(12,0);
    ASSERT_TRUE(event_at(first+7000000) && leg.state==EXO_PULSE_WAIT_NEXT_RESET &&
        leg.valid_period_intervals==0 && leg.exit_reason==EXO_PULSE_EXIT_PERIOD_UNTRUSTED,
        "out-of-range long period never creates scaled pulse");
    fixture(EXO_PULSE_PERIOD_SCALED);
    ASSERT_TRUE(event_at(first) && event_at(first+200000) && leg.event_count==1 &&
        leg.exit_reason==EXO_PULSE_EXIT_EVENT_REJECTED,"short event interval rejected without moving valid anchor");
    return true;
}
static bool test_duration_clamps_and_independent_sides(void) {
    for(int scenario=0;scenario<2;scenario++) {
        fixture(EXO_PULSE_PERIOD_SCALED);
        timing.minimum_ms=scenario?100:500;timing.maximum_ms=scenario?200:1500;
        ASSERT_TRUE(event_at(2000000) && event_at(3000000) && event_at(4000000),"three real events");
        ASSERT_TRUE(leg.frozen_duration_us==(scenario?200000U:500000U),"Ta min/max clamps");
    }
    fixture(EXO_PULSE_FIXED);
    ASSERT_TRUE(reset_leg() && new_swing(),"left event");
    exo_swing_pulse_leg_t right={0};
    for(int i=0;i<70;i++) {
        sample(12,0);
        const exo_swing_pulse_input_t in={.now_us=now_us,.source_us=(uint64_t)now_us,
            .feedback_count=count,.angle_deg=0,.velocity_deg_s=0,.valid=true};
        ASSERT_TRUE(exo_swing_pulse_step(&right,&timing,&window,1000,100,&in)==0,"stationary right does not assist");
    }
    ASSERT_TRUE(leg.pulse_count==1 && right.pulse_count==0,"left does not require right half-cycle");
    return true;
}
static bool test_independent_periods_and_jitter(void) {
    exo_swing_pulse_leg_t legs[2]={{0},{0}};
    const exo_pulse_timing_t t={.schema_version=1,.duration_mode=1,.peak_mnm=200,.fixed_ms=600,
        .ratio_permille=300,.minimum_ms=150,.maximum_ms=1500,.period_minimum_ms=400,.period_maximum_ms=3000};
    const exo_pulse_window_t w=exo_swing_pulse_default_window(1000);
    for(int i=0;i<650;i++) for(int side=0;side<2;side++) {
        const int phase=(i+(side?13:0))%(side?130:80);
        const float q=phase<4?10.5f+0.5f*(float)phase:phase<(side?55:40)?12:0;
        const exo_swing_pulse_input_t in={.now_us=1000000+(int64_t)i*10000,
            .source_us=1000000+(uint64_t)i*10000,.feedback_count=(uint32_t)i+1,
            .angle_deg=q,.velocity_deg_s=phase<4?20:0,.valid=true};
        exo_swing_pulse_step(&legs[side],&t,&w,1000,100,&in);
    }
    ASSERT_TRUE(legs[0].valid_period_intervals==2 && legs[1].valid_period_intervals==2 &&
        fabsf(legs[0].estimated_period_us-800000)<1 && fabsf(legs[1].estimated_period_us-1300000)<1,
        "left 800 ms and right 1300 ms use only their own events");
    ASSERT_TRUE(legs[0].pulse_count>legs[1].pulse_count && legs[1].pulse_count>0,"no strict alternation or half-cycle requirement");
    fixture(EXO_PULSE_FIXED);timing.peak_mnm=5000;
    ASSERT_TRUE(reset_leg() && new_swing(),"jitter pulse");
    const int dt[]={1000,5000,10000,15000};float previous=0;
    for(int i=0;i<80;i++) {
        now_us+=dt[i%4];
        const exo_swing_pulse_input_t in={.now_us=now_us,.source_us=(uint64_t)now_us,
            .feedback_count=++count,.angle_deg=12+6.0f*(float)(now_us-leg.pulse_started_us)*0.000001f,.velocity_deg_s=6,.valid=true};
        const float output=exo_swing_pulse_step(&leg,&timing,&window,1000,100,&in);
        ASSERT_TRUE(output-previous<=20.0f*(float)dt[i%4]*0.000001f+1e-5f,"20 Nm/s bound uses actual elapsed time");
        previous=output;
    }
    ASSERT_TRUE(leg.state==EXO_PULSE_WAIT_NEXT_RESET,"jitter does not stretch bounded pulse");
    return true;
}
static bool test_freeze_updates_window_and_rate(void) {
    fixture(EXO_PULSE_FIXED);
    ASSERT_TRUE(reset_leg() && new_swing(),"pulse");
    timing.fixed_ms=800;timing.peak_mnm=100;
    for(int i=0;i<30;i++) sample(12+0.06f*(float)i,6);
    ASSERT_TRUE(leg.frozen_duration_us==600000 && fabsf(leg.frozen_peak_nm-0.2f)<1e-6f,
        "current A and Ta frozen despite parameter update");
    for(int i=0;i<35;i++) sample(13.8f+0.06f*(float)i,6);
    ASSERT_TRUE(reset_leg() && new_swing() && leg.frozen_duration_us==800000 &&
        fabsf(leg.frozen_peak_nm-0.1f)<1e-6f,"next event consumes update");
    window.window_maximum_cdeg=1200;
    ASSERT_TRUE(sample(12.1f,20)==0 && leg.exit_reason==EXO_PULSE_EXIT_WINDOW,"window tightening immediate");
    fixture(EXO_PULSE_FIXED);timing.peak_mnm=5000;
    ASSERT_TRUE(reset_leg() && new_swing() && leg.peak_rate_limited &&
        fabsf(leg.frozen_peak_nm-12.0f/PI_F)<1e-5f,"short duration caps A for 20 N m/s");
    float previous=0;
    for(int i=0;i<70;i++) {
        float output=sample(12+0.06f*(float)i,6);
        ASSERT_TRUE(output-previous<=0.20001f,"bounded positive slope at short duration");previous=output;
    }
    ASSERT_TRUE(reset_leg(),"reset for full 5 Nm");timing.fixed_ms=1800;
    ASSERT_TRUE(new_swing() && !leg.peak_rate_limited && fabsf(leg.frozen_peak_nm-5)<1e-6f,"long pulse can reach requested 5 Nm");
    float peak=0;
    for(int i=1;i<=90;i++) peak=sample(12+0.06f*(float)i,6);
    ASSERT_TRUE(fabsf(peak-5)<1e-5f,"actual sin2 calculation reaches 5 Nm at midpoint");
    return true;
}
static bool test_physical_stop_and_window_freeze(void) {
    fixture(EXO_PULSE_FIXED);ASSERT_TRUE(reset_leg() && new_swing(),"pulse");
    for(int i=0;i<15;i++) sample(12,0);
    ASSERT_TRUE(leg.state==EXO_PULSE_WAIT_NEXT_RESET && leg.exit_reason==EXO_PULSE_EXIT_MOTION_STOPPED,
        "stopped movement exits before fixed Ta, not force-completed");
    for(int i=0;i<100;i++) ASSERT_TRUE(sample(12,20)==0,"cannot resume stopped pulse");
    ASSERT_TRUE(reset_leg() && new_swing(),"fresh physical event");
    window.window_maximum_cdeg=9000;
    ASSERT_TRUE(sample(31,20)==0 && leg.exit_reason==EXO_PULSE_EXIT_WINDOW,
        "live widening cannot weaken this pulse frozen safety window");
    fixture(EXO_PULSE_FIXED);ASSERT_TRUE(reset_leg(),"old reset ready");
    window.reset_cdeg=-200;
    for(int i=0;i<4;i++) sample(10.5f+0.5f*i,20);
    ASSERT_TRUE(leg.state==EXO_PULSE_WAIT_RESET && leg.pulse_count==0,"new reset must not reuse old READY");
    for(int i=0;i<10;i++) sample(-3,0);
    ASSERT_TRUE(leg.state==EXO_PULSE_READY && new_swing() && leg.pulse_count==1,"new reset rules require new confirmation");
    return true;
}
static bool test_protocol_vectors_and_limits(void) {
    uint8_t frame[]={1,0x21,0x42,16,1,0,200,0,88,2,44,1,150,0,220,5,144,1,184,11};
    exo_ble_request_t request;
    ASSERT_TRUE(exo_ble_protocol_decode_request(frame,sizeof(frame),&request)==EXO_BLE_RESULT_OK &&
        request.pulse_timing.fixed_ms==600 && request.pulse_timing.peak_mnm==200,"timing golden bytes");
    uint8_t response[20];size_t size=0;
    ASSERT_TRUE(exo_ble_protocol_encode_pulse_timing(0x42,&request.pulse_timing,response,sizeof(response),&size) &&
        size==20 && response[1]==0xa0 && memcmp(response+4,frame+4,16)==0,"same C/TS response vector");
    frame[5]=2;ASSERT_TRUE(exo_ble_protocol_decode_request(frame,20,&request)==EXO_BLE_RESULT_INVALID_PAYLOAD,"bad mode rejected");
    frame[5]=0;frame[8]=99;frame[9]=0;
    ASSERT_TRUE(exo_ble_protocol_decode_request(frame,20,&request)==EXO_BLE_RESULT_INVALID_PAYLOAD,"bad fixed duration");
    uint8_t w[]={1,0x23,0x43,4,32,3,184,11};
    ASSERT_TRUE(exo_ble_protocol_decode_request(w,sizeof(w),&request)==EXO_BLE_RESULT_OK &&
        request.pulse_window.reset_cdeg==800,"window golden bytes");
    ASSERT_TRUE(exo_ble_protocol_encode_pulse_window(0x43,&request.pulse_window,response,sizeof(response),&size) &&
        size==12 && response[1]==0xa2,"window readback");
    const exo_ble_wire_pulse_status_t status={.side=0,.state=3,.flags=7,.event_count=3,
        .pulse_count=1,.duration_ms=600,.period_ms=1000,.peak_mnm=200,.progress_x10000=5000};
    ASSERT_TRUE(exo_ble_protocol_encode_pulse_status(0x44,&status,response,sizeof(response),&size) &&
        size==20 && response[1]==0xa4,"observable independent event state");
    ASSERT_TRUE(!exo_ble_protocol_encode_pulse_status(0x44,&status,response,19,&size) && size==0,"capacity fail has no partial response");
    return true;
}

static exo_sensor_snapshot_t sensor;
static exo_gait_estimate_t gait;
static exo_assist_result_t result;
static exo_controller_tick_result_t tick;
static uint16_t integration_maximum=200;
static uint16_t integration_fixed=600;
static bool integrated_sample(float l,float r,float velocity) {
    sensor.captured_us+=10000;++sensor.sequence;++gait.sequence;
    for(unsigned side=0;side<2;side++) {
        exo_sensor_joint_snapshot_t *j=side?&sensor.right:&sensor.left;
        j->source_timestamp_us=(uint64_t)sensor.captured_us;++j->source_feedback_count;
        j->joint_sample.position_deg=side?r:l;j->joint_sample.velocity_deg_s=velocity;
        j->joint_valid=j->mapping_valid=j->direction_valid=j->zero_valid=j->feedback_valid=j->online=true;
    }
    ASSERT_TRUE(exo_assist_process_tick(&sensor,&gait,&result)==ESP_OK,"real assistance tick");
    ASSERT_TRUE(exo_controller_process_tick(sensor.captured_us,&result,sensor.sequence,gait.sequence,&tick)==ESP_OK,
        "unique same-cycle controller output");
    exo_assist_report_output(&result,tick.assist_applied);
    return true;
}
static bool integrated_arm(exo_assist_execution_state_t execution) {
    ASSERT_TRUE(set_product_base(),"product damping base");
    const exo_assist_config_t config={.schema_version=1,.mode=4,.requested_execution_state=execution,
        .strategy_id=4,.left_level_percent=50,.right_level_percent=100,
        .maximum_assist_torque_mnm=integration_maximum,.angle_threshold_cdeg=1000};
    ASSERT_TRUE(exo_assist_set_config(&config)==ESP_OK,"pulse config");
    exo_pulse_timing_t t=exo_swing_pulse_default_timing(integration_maximum);t.fixed_ms=integration_fixed;
    ASSERT_TRUE(exo_assist_update_pulse_timing(&t)==ESP_OK && exo_assist_arm()==ESP_OK,"pulse arm");
    exo_assist_snapshot_t snapshot;exo_assist_get_snapshot(&snapshot);
    const exo_controller_assist_contract_t contract={.execution_state=snapshot.execution_state,
        .activation_epoch=snapshot.activation_epoch,.strategy_id=4,.maximum_assist_torque_mnm=integration_maximum};
    const exo_controller_centers_t centers={0};
    ASSERT_TRUE(exo_controller_arm(&centers,1000000,&contract)==ESP_OK,"controller arm");
    memset(&sensor,0,sizeof(sensor));memset(&gait,0,sizeof(gait));
    sensor.captured_us=1000000;sensor.reset_epoch=1;sensor.valid_sensor_mask=3;
    for(int i=0;i<10;i++) ASSERT_TRUE(integrated_sample(0,0,0),"reset samples");
    for(int i=0;i<4;i++) ASSERT_TRUE(integrated_sample(10.5f+0.5f*i,10.5f+0.5f*i,20),"simultaneous new events");
    return true;
}
static bool test_real_integration_shadow_active_stop_updates(void) {
    for(int execution=1;execution<=2;execution++) {
        ASSERT_TRUE(integrated_arm((exo_assist_execution_state_t)execution),"run");
        for(int i=1;i<=30;i++) ASSERT_TRUE(integrated_sample(12+0.06f*i,12+0.06f*i,6),"pulse samples");
        ASSERT_TRUE(fabsf(result.suggestion.left.torque_ff_nm-0.1f)<1e-6f &&
            fabsf(result.suggestion.right.torque_ff_nm-0.2f)<1e-6f,"each side scales frozen A");
        ASSERT_TRUE(fabsf(tick.final_command.right.torque_ff_nm-(execution==2?0.2f:0))<1e-6f &&
            fabsf(s_last_sent.right.torque_ff_nm-(execution==2?0.2f:0))<1e-6f,
            "SHADOW zero actual/ACTIVE same-cycle delta");
        exo_pulse_timing_t updated=exo_swing_pulse_default_timing(100);updated.fixed_ms=800;
        ASSERT_TRUE(exo_assist_update_pulse_timing(&updated)==ESP_OK,"live atomic update");
        updated.peak_mnm=201;
        ASSERT_TRUE(exo_assist_update_pulse_timing(&updated)!=ESP_OK,"cannot raise START torque ceiling");
        ASSERT_TRUE(integrated_sample(13.86f,13.86f,6),"frozen pulse continues");
        exo_assist_snapshot_t snapshot;exo_assist_get_snapshot(&snapshot);
        ASSERT_TRUE(snapshot.pulse_right.frozen_duration_us==600000 && snapshot.pulse_right.pulse_count==1,
            "diagnostic snapshot reports frozen Ta and single pulse");
        const exo_assist_result_t stale=result;
        ASSERT_TRUE(exo_assist_disarm()==ESP_OK,"STOP disarms assistance");
        ASSERT_TRUE(exo_assist_process_tick(&sensor,&gait,&result)==ESP_OK && !result.valid &&
            result.suggestion.right.torque_ff_nm==0,"STOP cannot force pulse completion");
        exo_assist_report_output(&stale,true);exo_assist_get_snapshot(&snapshot);
        ASSERT_TRUE(!snapshot.real_output_enabled,"old event cannot revive stopped output");
        ASSERT_TRUE(disarm_run(),"cleanup");
    }
    return true;
}
static bool test_real_integration_invalid_epoch_and_sensors(void) {
    for(int scenario=0;scenario<6;scenario++) {
        ASSERT_TRUE(integrated_arm(EXO_ASSIST_EXEC_ACTIVE),"run");
        for(int i=0;i<15;i++) ASSERT_TRUE(integrated_sample(12+0.06f*i,12+0.06f*i,6),"nonzero pulse");
        if(scenario==0) sensor.valid_sensor_mask=1;
        if(scenario==1) sensor.reset_epoch++;
        if(scenario==2) sensor.left.motor_fault=true;
        if(scenario==3) sensor.left.online=false;
        if(scenario==4) sensor.left.source_feedback_count--;
        if(scenario==5) sensor.left.source_timestamp_us+=100000;
        sensor.captured_us+=10000;++sensor.sequence;++gait.sequence;
        const esp_err_t error=exo_assist_process_tick(&sensor,&gait,&result);
        ASSERT_TRUE((scenario==0?error==ESP_OK:error==ESP_ERR_INVALID_STATE) &&
            result.suggestion.left.torque_ff_nm==0 && result.suggestion.right.torque_ff_nm==0 && !result.valid,
            "invalid pair/epoch/fault clears whole added assistance immediately");
        ASSERT_TRUE(disarm_run(),"cleanup");
    }
    return true;
}
static bool test_real_peak_and_zero_limits(void) {
    for(int scenario=0;scenario<2;scenario++) {
        integration_maximum=scenario?0:5000;integration_fixed=1800;
        ASSERT_TRUE(integrated_arm(EXO_ASSIST_EXEC_ACTIVE),"5 Nm or zero ceiling run");
        for(int i=1;i<=90;i++) ASSERT_TRUE(integrated_sample(12+0.06f*i,12+0.06f*i,6),"moving waveform samples");
        ASSERT_TRUE(fabsf(tick.final_command.right.torque_ff_nm-(scenario?0:5))<1e-5f &&
            fabsf(s_last_sent.right.torque_ff_nm-(scenario?0:5))<1e-5f,"actual unique fake CAN output obeys 5 or zero bound");
        ASSERT_TRUE(disarm_run(),"cleanup");
    }
    integration_maximum=200;integration_fixed=600;
    return true;
}
static bool test_legacy_angle_protocol(void) {
    exo_ble_request_t request;
    uint8_t frame[] = {1,0x1d,0x32,12,1,4,2,4,0,50,100,0,200,0,232,3};
    ASSERT_TRUE(exo_ble_protocol_decode_request(frame, sizeof(frame), &request) == EXO_BLE_RESULT_OK &&
        request.angle_threshold_cdeg == 1000 && request.maximum_assist_torque_mnm == 200, "atomic golden vector");
    frame[14] = 99; frame[15] = 0;
    ASSERT_TRUE(exo_ble_protocol_decode_request(frame, sizeof(frame), &request) == EXO_BLE_RESULT_INVALID_PAYLOAD, "threshold lower bound");
    frame[14] = 100;
    ASSERT_TRUE(exo_ble_protocol_decode_request(frame, sizeof(frame), &request) == EXO_BLE_RESULT_OK, "minimum threshold");
    frame[14] = 112; frame[15] = 23;
    ASSERT_TRUE(exo_ble_protocol_decode_request(frame, sizeof(frame), &request) == EXO_BLE_RESULT_OK, "maximum threshold");
    frame[14] = 113;
    ASSERT_TRUE(exo_ble_protocol_decode_request(frame, sizeof(frame), &request) == EXO_BLE_RESULT_INVALID_PAYLOAD, "threshold upper bound");
    frame[14] = 232; frame[15] = 3; frame[12] = 0x88; frame[13] = 0x13;
    ASSERT_TRUE(exo_ble_protocol_decode_request(frame, sizeof(frame), &request) == EXO_BLE_RESULT_OK &&
        request.maximum_assist_torque_mnm == 5000, "5 N m atomic golden vector matches web");
    frame[12] = 0x98; frame[13] = 0x3a;
    ASSERT_TRUE(exo_ble_protocol_decode_request(frame, sizeof(frame), &request) == EXO_BLE_RESULT_OK &&
        request.maximum_assist_torque_mnm == 15000, "15 N m ceiling accepted");
    frame[12] = 0x99;
    ASSERT_TRUE(exo_ble_protocol_decode_request(frame, sizeof(frame), &request) == EXO_BLE_RESULT_INVALID_PAYLOAD, "torque hard cap");
    frame[12] = 200; frame[13] = 0; frame[8] = 1;
    ASSERT_TRUE(exo_ble_protocol_decode_request(frame, sizeof(frame), &request) == EXO_BLE_RESULT_INVALID_PAYLOAD, "no phase profile");
    frame[8] = 0; frame[1] = 0x19; frame[3] = 10;
    ASSERT_TRUE(exo_ble_protocol_decode_request(frame, 14, &request) == EXO_BLE_RESULT_INVALID_PAYLOAD, "legacy opcode cannot select new strategy");
    uint8_t get[] = {1,0x0b,0x31,0};
    ASSERT_TRUE(exo_ble_protocol_decode_request(get, sizeof(get), &request) == EXO_BLE_RESULT_OK, "read opcode");
    const exo_ble_wire_angle_assist_t c = {.flags=3,.generation=7,.selected_cdeg=1000,.active_cdeg=1000};
    uint8_t out[20]; size_t length;
    const uint8_t golden[] = {1,0x8b,0x31,12,1,3,232,3,232,3,100,0,200,0,7,0};
    ASSERT_TRUE(exo_ble_protocol_encode_angle_assist(0x31,&c,out,sizeof(out),&length) &&
        length == sizeof(golden) && memcmp(out,golden,length)==0, "readback golden vector matches web");
    ASSERT_TRUE(!exo_ble_protocol_encode_angle_assist(0x31,&c,out,15,&length) && length==0, "capacity checked");
    exo_ble_wire_assist_torque_limit_t limit = {
        .flags=1, .selected_maximum_mnm=5000, .hard_maximum_mnm=15000};
    const uint8_t limit_golden[] = {1,0x8a,0x31,8,1,1,0x88,0x13,0,0,0x98,0x3a};
    ASSERT_TRUE(exo_ble_protocol_encode_assist_torque_limit(0x31,&limit,out,sizeof(out),&length) &&
        length==sizeof(limit_golden) && memcmp(out,limit_golden,length)==0, "5 N m limit readback golden vector");
    limit.hard_maximum_mnm=5001;
    ASSERT_TRUE(!exo_ble_protocol_encode_assist_torque_limit(0x31,&limit,out,sizeof(out),&length), "unknown hard bound rejected");
    return true;
}

static bool test_extended_fixed_limits(void) {
    fixture(EXO_PULSE_FIXED); timing.peak_mnm=15000; timing.fixed_ms=3000;
    ASSERT_TRUE(exo_swing_pulse_timing_valid(&timing),"15 Nm / 3000 ms inclusive bounds");
    timing.peak_mnm=15001; ASSERT_TRUE(!exo_swing_pulse_timing_valid(&timing),"15001 mNm rejected");
    timing.peak_mnm=15000; timing.fixed_ms=3001;
    ASSERT_TRUE(!exo_swing_pulse_timing_valid(&timing),"3001 ms rejected"); timing.fixed_ms=3000;
    ASSERT_TRUE(reset_leg() && new_swing(),"new actual side event");
    const int64_t start=now_us; float previous=0, maximum=0;
    ASSERT_TRUE(!leg.peak_rate_limited && fabsf(leg.frozen_peak_nm-15)<1e-5f,"15 Nm / 3 s fully admitted at20 Nm/s");
    for(int i=1;i<=300;i++) {
        const float output=sample(12+0.03f*i,3);
        const float u=(float)(now_us-start)/3000000.0f;
        const float expected=i==300?0:15*powf(sinf(PI_F*u),2);
        ASSERT_TRUE(fabsf(output-expected)<2e-5f && output-previous<=0.20001f,"full15 Nm sin2 and20 Nm/s bound for3 s");
        if(output>maximum) maximum=output;
        previous=output;
    }
    ASSERT_TRUE(fabsf(maximum-15)<1e-5f && leg.state==EXO_PULSE_WAIT_NEXT_RESET,"actual15 Nm peak and finite3 s end");
    for(int i=0;i<500;i++) ASSERT_TRUE(sample(21,3)==0,"no automatic pulse after 3 seconds");
    ASSERT_TRUE(leg.pulse_count==1,"one pulse in confirmed side cycle");
    uint8_t frame[]={1,0x21,0x42,16,1,0,0x98,0x3a,0xb8,0x0b,44,1,150,0,220,5,144,1,184,11};
    exo_ble_request_t request; uint8_t out[20]; size_t length;
    ASSERT_TRUE(exo_ble_protocol_decode_request(frame,20,&request)==EXO_BLE_RESULT_OK &&
        request.pulse_timing.peak_mnm==15000 && request.pulse_timing.fixed_ms==3000,"new C/web timing golden");
    ASSERT_TRUE(exo_ble_protocol_encode_pulse_timing(0x42,&request.pulse_timing,out,20,&length) &&
        length==20 && memcmp(out+4,frame+4,16)==0,"new timing readback fits20B");
    frame[8]=0xb9; ASSERT_TRUE(exo_ble_protocol_decode_request(frame,20,&request)==EXO_BLE_RESULT_INVALID_PAYLOAD,"wire duration over bound rejected");
    frame[8]=0xb8; frame[6]=0x99;
    ASSERT_TRUE(exo_ble_protocol_decode_request(frame,20,&request)==EXO_BLE_RESULT_INVALID_PAYLOAD,"wire torque over bound rejected");
    exo_ble_wire_pulse_status_t status={.side=0,.state=3,.flags=5,.duration_ms=3000,.peak_mnm=15000,.progress_x10000=5000};
    ASSERT_TRUE(exo_ble_protocol_encode_pulse_status(0x43,&status,out,20,&length),"actual long pulse status encodable");
    status.duration_ms=3001; ASSERT_TRUE(!exo_ble_protocol_encode_pulse_status(0x43,&status,out,20,&length),"invalid status duration rejected");
    return true;
}
static bool test_extended_real_integration(void) {
    integration_maximum=15000; integration_fixed=3000;
    ASSERT_TRUE(integrated_arm(EXO_ASSIST_EXEC_ACTIVE),"15 Nm START ceiling admitted");
    for(int i=1;i<=150;i++) ASSERT_TRUE(integrated_sample(12+0.03f*i,12+0.03f*i,3),"three-second live samples");
    ASSERT_TRUE(fabsf(result.suggestion.left.torque_ff_nm-7.5f)<1e-4f &&
        fabsf(result.suggestion.right.torque_ff_nm-15)<1e-4f,"independent50/100 percent actual peaks7.5/15 Nm");
    ASSERT_TRUE(fabsf(s_last_sent.right.torque_ff_nm-15)<1e-4f,"unique final command carries full15 Nm simulated output");
    exo_assist_snapshot_t snapshot; exo_assist_get_snapshot(&snapshot);
    ASSERT_TRUE(snapshot.strategy_version==4 && !snapshot.pulse_right.peak_rate_limited,"new policy version and no3 s rate limiting");
    exo_pulse_timing_t updated=exo_swing_pulse_default_timing(15000); updated.fixed_ms=3001;
    ASSERT_TRUE(exo_assist_update_pulse_timing(&updated)!=ESP_OK,"invalid update cannot replace frozen current pulse");
    for(int i=151;i<=300;i++) ASSERT_TRUE(integrated_sample(12+0.03f*i,12+0.03f*i,3),"remaining real pulse samples");
    ASSERT_TRUE(s_last_sent.left.torque_ff_nm==0 && s_last_sent.right.torque_ff_nm==0,"end zeros at composer output");
    ASSERT_TRUE(disarm_run(),"cleanup");
    integration_maximum=200; integration_fixed=600;
    return true;
}

static bool test_rise20_threshold_and_peak_stop(void) {
    for(unsigned i=0;i<2;++i) {
        fixture(EXO_PULSE_FIXED);timing.peak_mnm=15000;timing.fixed_ms=i?2357:2356;
        ASSERT_TRUE(reset_leg() && new_swing(),"fresh event for threshold boundary");
        ASSERT_TRUE(leg.peak_rate_limited==(!i),"2356ms limited,2357ms can freeze15Nm");
        ASSERT_TRUE(i?fabsf(leg.frozen_peak_nm-15)<1e-6f:leg.frozen_peak_nm<15,"minimum full15Nm duration");
    }
    fixture(EXO_PULSE_FIXED);timing.peak_mnm=15000;timing.fixed_ms=600;
    ASSERT_TRUE(reset_leg() && new_swing() && leg.peak_rate_limited &&
        fabsf(leg.frozen_peak_nm-12/PI_F)<1e-5f,"600ms still limited3.8197Nm");
    integration_maximum=15000;integration_fixed=3000;
    ASSERT_TRUE(integrated_arm(EXO_ASSIST_EXEC_ACTIVE),"new high ceiling run");
    for(int i=1;i<=150;i++) ASSERT_TRUE(integrated_sample(12+0.03f*i,12+0.03f*i,3),"up to15Nm simulated peak");
    const exo_assist_result_t stale=result;
    ASSERT_TRUE(exo_assist_disarm()==ESP_OK && exo_controller_disarm()==ESP_OK,"STOP closes both contracts at peak");
    ASSERT_TRUE(integrated_sample(16.6f,16.6f,3),"next cycle after STOP");
    ASSERT_TRUE(!result.valid && result.suggestion.right.torque_ff_nm==0 && !tick.send_attempted,
        "no completion of remaining high pulse and no periodic send after STOP");
    exo_assist_report_output(&stale,true);
    exo_assist_snapshot_t snapshot;exo_assist_get_snapshot(&snapshot);
    ASSERT_TRUE(!snapshot.real_output_enabled,"stale15Nm result cannot revive stopped output");
    integration_maximum=200;integration_fixed=600;
    return true;
}
/* 直接回放真实100 Hz纯算法及最终合成器；不向实物发送命令。 */
int main(void) {
    if(previous_phase_suite()!=0) return 1;
    struct {const char *name;bool(*run)(void);} tests[]={
        {"legacy angle/torque byte protocol retained",test_legacy_angle_protocol},
        {"FIXED sin2 / no hold or timer repetition",test_fixed_shape_and_no_repeat},
        {"high startup / jitter / wrong direction",test_start_high_noise_and_wrong_direction},
        {"early reverse/window/feedback exits",test_early_exits_and_feedback},
        {"duplicate samples / feedback gap",test_duplicates_gap_and_time},
        {"same-side trusted period / smoothing / bad intervals",test_period_scaled_and_invalid_periods},
        {"duration clamp / unilateral independence",test_duration_clamps_and_independent_sides},
        {"two different same-side periods / actual time jitter",test_independent_periods_and_jitter},
        {"frozen A Ta / next update / rise limit",test_freeze_updates_window_and_rate},
        {"physical stop / frozen safety window",test_physical_stop_and_window_freeze},
        {"C and TS timing/window/status golden vectors",test_protocol_vectors_and_limits},
        {"real SHADOW/ACTIVE / simultaneous events / STOP / updates",test_real_integration_shadow_active_stop_updates},
        {"real pair invalid / epoch change / motor fault",test_real_integration_invalid_epoch_and_sensors},
        {"real 5 Nm and zero START ceilings",test_real_peak_and_zero_limits},
        {"15 Nm request / 3-second shape / protocol / no repeat",test_extended_fixed_limits},
        {"15 Nm START ceiling / levels / unique composer output",test_extended_real_integration},
        {"20 Nm/s duration boundary / short pulse / STOP at15Nm",test_rise20_threshold_and_peak_stop},
    };
    unsigned passed=0;
    for(unsigned i=0;i<sizeof(tests)/sizeof(tests[0]);i++) {
        if(!tests[i].run()) {fprintf(stderr,"[FAIL] %s\n",tests[i].name);return 1;}
        printf("[PASS] %s\n",tests[i].name);passed++;
    }
    printf("[PASS] swing pulse: %u/%u; leg state=%zu bytes; workspace=%u bytes\n",
        passed,(unsigned)(sizeof(tests)/sizeof(tests[0])),sizeof(exo_swing_pulse_leg_t),EXO_ASSIST_POLICY_WORKSPACE_BYTES);
    return 0;
}
