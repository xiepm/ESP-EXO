/* 真实AFO/runtime/assist/controller/composer；最终CAN和RTOS/NVS为主机桩。
 * 运行两种编译边界：首级台架500 mNm和不可部署的主机15000 mNm。 */
#define EXO_ADAPTIVE_REUSE_FIXTURE
#include "adaptive_integration_replay.c"

static unsigned index_now;
static exo_assist_snapshot_t current_assist;
static exo_adaptive_config_t active_parameters;
static exo_assist_result_t last_policy;
static exo_sensor_snapshot_t last_sensors;
static exo_controller_tick_result_t last_tick;

static exo_assist_config_t active_selection(exo_assist_execution_state_t state) {
    return (exo_assist_config_t){.schema_version=1,.mode=EXO_ASSIST_MODE_ADAPTIVE_GAUSSIAN,
        .strategy_id=EXO_ASSIST_STRATEGY_ADAPTIVE_GAUSSIAN,.requested_execution_state=state,
        .left_level_percent=100,.right_level_percent=100,
        .maximum_assist_torque_mnm=EXO_ADAPTIVE_CONFIG_MAX_MNM};
}
static bool begin_case(bool parameters, exo_assist_execution_state_t state) {
    (void)exo_assist_disarm();(void)exo_controller_disarm();
    ASSERT_TRUE(exo_adaptive_runtime_init()==ESP_OK,"runtime reset");
    ASSERT_TRUE(exo_assist_init()==ESP_OK&&exo_controller_init()==ESP_OK,"module init");
    index_now=0;mock_clock=0;
    s_send_error[0]=s_send_error[1]=ESP_OK;
    active_parameters=exo_adaptive_default_config();
    if(parameters) {
        active_parameters.overall_strength=1;
        for(unsigned i=0;i<2;i++) active_parameters.leg[i]=(exo_adaptive_leg_config_t){
            .enabled=1,.strength=1,.flex_amplitude_nm=.2f,.flex_peak_phase=.75f,.flex_width=.1f,
            .ext_amplitude_nm=.15f,.ext_peak_phase=.2f,.ext_width=.1f,.torque_limit_nm=.25f,
            .slew_nm_s=2,.positive_power_w=1,.negative_power_w=1,.minimum_angle_deg=-40,
            .maximum_angle_deg=60,.maximum_velocity_deg_s=350,.fade_s=1,.transition_s=.5f};
    }
    ASSERT_TRUE(exo_adaptive_runtime_set(&active_parameters)==ESP_OK,"initial parameters");
    const exo_joint_command_t zero={0};
    ASSERT_TRUE(exo_controller_set_mode(EXO_CONTROL_MODE_DAMPING,EXO_REFERENCE_CONSTANT)==ESP_OK,"base mode");
    ASSERT_TRUE(exo_controller_set_joint_command(EXO_HIP_JOINT_LEFT,&zero)==ESP_OK&&
        exo_controller_set_joint_command(EXO_HIP_JOINT_RIGHT,&zero)==ESP_OK,"zero base gains");
    exo_assist_config_t cfg=active_selection(state);
    ASSERT_TRUE(exo_assist_set_config(&cfg)==ESP_OK,"new strategy configuration");
    exo_assist_run_contract_t pre;
    ASSERT_TRUE(exo_assist_preview_run_contract(&pre)==ESP_OK&&
        exo_controller_validate_run_contract(&pre)==ESP_OK,"preflight same contract");
    ASSERT_TRUE(exo_assist_arm()==ESP_OK,"arm config");
    ASSERT_TRUE(exo_assist_get_snapshot(&current_assist),"arm snapshot");
    const exo_controller_assist_contract_t contract={.execution_state=state,
        .activation_epoch=current_assist.activation_epoch,.strategy_id=cfg.strategy_id,
        .maximum_assist_torque_mnm=cfg.maximum_assist_torque_mnm};
    const exo_controller_centers_t centers={0};
    ASSERT_TRUE(exo_controller_arm(&centers,10000,&contract)==ESP_OK,"controller arm");
    return true;
}

/* fault: invalid feedback / NaN / future source / stale age / coordinates /
 * emergency / source counter rollback / stopped input / range / time gap. */
static bool step(unsigned fault, bool gate) {
    unsigned n=index_now++;
    int64_t us=10000+(int64_t)n*10000;
    if(fault==10) us+=30000;
    float t=(float)n*.01f;
    exo_sensor_snapshot_t sensor={.captured_us=us,.sequence=n+1,.reset_epoch=1,
        .valid_sensor_mask=EXO_SENSOR_LEFT_HIP_KINEMATICS|EXO_SENSOR_RIGHT_HIP_KINEMATICS};
    sensor.left=make_joint(18*cosf(2*PI_F*t),-36*PI_F*sinf(2*PI_F*t),(uint64_t)us,n+1);
    sensor.right=make_joint(14*cosf(2*PI_F*1.2f*t+.7f),-28*PI_F*1.2f*sinf(2*PI_F*1.2f*t+.7f),(uint64_t)us,n+1);
    if(fault==1) sensor.left.joint_valid=sensor.right.joint_valid=false;
    if(fault==2) sensor.left.joint_sample.position_deg=sensor.right.joint_sample.position_deg=NAN;
    if(fault==3) sensor.left.source_timestamp_us=sensor.right.source_timestamp_us=(uint64_t)us+1;
    if(fault==4) sensor.left.age_us=sensor.right.age_us=50001;
    if(fault==5) sensor.reset_epoch=2;
    if(fault==7) sensor.left.source_feedback_count=sensor.right.source_feedback_count=1;
    if(fault==8) {
        sensor.left.joint_sample.position_deg=last_sensors.left.joint_sample.position_deg;
        sensor.right.joint_sample.position_deg=last_sensors.right.joint_sample.position_deg;
        sensor.left.joint_sample.velocity_deg_s=sensor.right.joint_sample.velocity_deg_s=0;
    }
    if(fault==9) sensor.left.joint_sample.velocity_deg_s=sensor.right.joint_sample.velocity_deg_s=351;
    mock_clock=us;
    ASSERT_TRUE(exo_assist_get_snapshot(&current_assist),"same-cycle assist state");
    const uint32_t permit=exo_adaptive_runtime_permit(&current_assist,gate);
    ASSERT_TRUE(gate ? permit!=0 : permit==0,"shared runtime permit path");
    exo_adaptive_runtime_update(&sensor,permit,3,fault==6);
    exo_gait_estimate_t gait={.sequence=n+1};
    ASSERT_TRUE(exo_assist_process_tick(&sensor,&gait,&last_policy)==ESP_OK,"AFO policy");
    const esp_err_t sent=exo_controller_process_tick(us,&last_policy,sensor.sequence,gait.sequence,&last_tick);
    ASSERT_TRUE(sent==(s_send_error[0]!=ESP_OK?s_send_error[0]:s_send_error[1]),"CAN error propagation");
    ASSERT_TRUE(s_last_sent.left.kp==0&&s_last_sent.left.kd==0&&s_last_sent.right.kp==0&&s_last_sent.right.kd==0,"no hidden base torque gains");
    const float cap=(float)EXO_ADAPTIVE_CONFIG_MAX_MNM/1000.0f;
    ASSERT_TRUE(fabsf(s_last_sent.left.torque_ff_nm)<=cap&&fabsf(s_last_sent.right.torque_ff_nm)<=cap,"final bounded pair");
    exo_adaptive_runtime_finish(last_tick.final_command.left.torque_ff_nm,last_tick.final_command.right.torque_ff_nm,
        last_tick.actuator_b_send_succeeded,last_tick.actuator_a_send_succeeded,100);
    last_sensors=sensor;
    return true;
}
static bool zero_pair(void) {
    return s_last_sent.left.torque_ff_nm==0&&s_last_sent.right.torque_ff_nm==0;
}
static bool warm_nonzero(void) {
    for(unsigned n=0;n<5000;n++) {
        ASSERT_TRUE(step(0,true),"warmup");
        if(fabsf(s_last_sent.left.torque_ff_nm)>.02f&&fabsf(s_last_sent.right.torque_ff_nm)>.02f) return true;
    }
    ASSERT_TRUE(false,"AFO did not produce bilateral nonzero output");
}
static bool test_defaults_and_shadow(void) {
    ASSERT_TRUE(begin_case(false,EXO_ASSIST_EXEC_ACTIVE),"zero default run");
    for(unsigned n=0;n<4000;n++) { ASSERT_TRUE(step(0,true),"default tick");ASSERT_TRUE(zero_pair(),"default never adds torque"); }
    ASSERT_TRUE(begin_case(true,EXO_ASSIST_EXEC_SHADOW),"shadow run");
    unsigned nonzero=0;
    for(unsigned n=0;n<4000;n++) {
        ASSERT_TRUE(step(0,true),"shadow tick");
        nonzero+=fabsf(last_policy.suggestion.left.torque_ff_nm)>1e-4f;
        ASSERT_TRUE(zero_pair()&&!last_tick.assist_applied,"SHADOW remains output-free");
    }
    ASSERT_TRUE(nonzero>3000,"SHADOW had real nonzero suggestions");
    return true;
}
static bool test_active_and_boundaries(void) {
    ASSERT_TRUE(begin_case(true,EXO_ASSIST_EXEC_ACTIVE),"active case");
    ASSERT_TRUE(warm_nonzero(),"nonzero real chain");
    ASSERT_TRUE(last_tick.assist_accepted&&last_tick.assist_applied&&last_tick.send_succeeded,"actual composition and paired send");
    exo_adaptive_config_t changed=active_parameters;changed.overall_strength=.1f;
    ASSERT_TRUE(exo_adaptive_runtime_set(&changed)==ESP_ERR_INVALID_STATE,"ACTIVE parameters frozen");
    exo_adaptive_config_t readback;uint32_t gen;
    ASSERT_TRUE(exo_adaptive_runtime_config(&readback,&gen)&&readback.overall_strength==1,"failed update leaves config");
    exo_assist_config_t cfg=active_selection(EXO_ASSIST_EXEC_ACTIVE);cfg.left_level_percent=0;
    ASSERT_TRUE(exo_assist_set_config(&cfg)==ESP_ERR_INVALID_STATE,"outer configuration frozen");
    exo_assist_result_t boundary=last_policy;
    float cap=(float)EXO_ADAPTIVE_CONFIG_MAX_MNM/1000.0f;
    boundary.suggestion.left.torque_ff_nm=cap;boundary.suggestion.right.torque_ff_nm=-cap;
    ASSERT_TRUE(exo_controller_process_tick(last_sensors.captured_us,&boundary,last_sensors.sequence,last_sensors.sequence,&last_tick)==ESP_OK,"signed exact boundary");
    ASSERT_TRUE(last_tick.assist_applied&&s_last_sent.left.torque_ff_nm==cap&&s_last_sent.right.torque_ff_nm==-cap,"signed cap preserved");
    boundary.suggestion.left.torque_ff_nm=nextafterf(cap,INFINITY);
    ASSERT_TRUE(exo_controller_process_tick(last_sensors.captured_us,&boundary,last_sensors.sequence,last_sensors.sequence,&last_tick)==ESP_OK&&!last_tick.assist_applied&&zero_pair(),"one float beyond cap rejects pair");
    boundary=last_policy;boundary.suggestion.left.torque_ff_nm=NAN;
    ASSERT_TRUE(exo_controller_process_tick(last_sensors.captured_us,&boundary,last_sensors.sequence,last_sensors.sequence,&last_tick)==ESP_OK&&zero_pair(),"NaN final suggestion rejects pair");
    boundary=last_policy;boundary.source_sensor_sequence--;
    ASSERT_TRUE(exo_controller_process_tick(last_sensors.captured_us,&boundary,last_sensors.sequence,last_sensors.sequence,&last_tick)==ESP_OK&&zero_pair(),"stale source rejects pair");
    boundary=last_policy;boundary.activation_epoch--;
    ASSERT_TRUE(exo_controller_process_tick(last_sensors.captured_us,&boundary,last_sensors.sequence,last_sensors.sequence,&last_tick)==ESP_OK&&zero_pair(),"stale activation rejects pair");
    const unsigned sends=s_send_count[0]+s_send_count[1];
    ASSERT_TRUE(exo_assist_disarm()==ESP_OK&&exo_controller_disarm()==ESP_OK,"explicit STOP");
    ASSERT_TRUE(exo_controller_process_tick(last_sensors.captured_us+10000,&last_policy,last_sensors.sequence,last_sensors.sequence,&last_tick)==ESP_OK,"post STOP old result");
    ASSERT_TRUE(s_send_count[0]+s_send_count[1]==sends,"STOP suppresses sends");
    ASSERT_TRUE(exo_adaptive_runtime_set(&changed)==ESP_OK,"STOP unlocks parameter editing");
    cfg=active_selection(EXO_ASSIST_EXEC_ACTIVE);cfg.maximum_assist_torque_mnm++;
    ASSERT_TRUE(exo_assist_set_config(&cfg)==ESP_ERR_INVALID_ARG,"reject cap+1 configuration");
    cfg=active_selection(EXO_ASSIST_EXEC_ACTIVE);ASSERT_TRUE(exo_assist_set_config(&cfg)==ESP_OK,"restore selected");
    const exo_joint_command_t damping={.kd=.1f};
    ASSERT_TRUE(exo_controller_set_joint_command(EXO_HIP_JOINT_LEFT,&damping)==ESP_OK,"prepare gain conflict");
    exo_assist_run_contract_t pre;ASSERT_TRUE(exo_assist_preview_run_contract(&pre)==ESP_OK,"preflight conflict");
    ASSERT_TRUE(exo_controller_validate_run_contract(&pre)==ESP_ERR_INVALID_STATE,"base damping rejected before START");
    const exo_controller_centers_t centers={0};
    const exo_controller_assist_contract_t contract={.execution_state=EXO_ASSIST_EXEC_ACTIVE,.activation_epoch=1,
        .strategy_id=EXO_ASSIST_STRATEGY_ADAPTIVE_GAUSSIAN,.maximum_assist_torque_mnm=EXO_ADAPTIVE_CONFIG_MAX_MNM};
    ASSERT_TRUE(exo_controller_arm(&centers,10000,&contract)==ESP_ERR_INVALID_STATE,"arm independently rejects damping");
    return true;
}
static bool test_faults(void) {
    for(unsigned fault=1;fault<=10;fault++) {
        ASSERT_TRUE(begin_case(true,EXO_ASSIST_EXEC_ACTIVE)&&warm_nonzero(),"nonzero before each fault");
        if(fault==8) {
            /* 有效、连续的静止反馈；不得用跳角INPUT故障冒充停步超时。 */
            for(unsigned n=0;n<31;n++) ASSERT_TRUE(step(fault,true),"valid stationary input");
            ASSERT_TRUE(zero_pair(),"stop timeout removes final nonzero pair");
        } else {
            ASSERT_TRUE(step(fault,true)&&zero_pair(),"fault removes final nonzero pair");
        }
        for(unsigned n=0;n<2000;n++) { ASSERT_TRUE(step(0,true),"healthy input returns");ASSERT_TRUE(zero_pair(),"old permit cannot resume after fault"); }
    }
    ASSERT_TRUE(begin_case(true,EXO_ASSIST_EXEC_ACTIVE)&&warm_nonzero(),"gate case");
    ASSERT_TRUE(step(0,false)&&zero_pair(),"closed gate withdraws adaptive permit");
    ASSERT_TRUE(begin_case(true,EXO_ASSIST_EXEC_ACTIVE)&&warm_nonzero(),"partial CAN case");
    s_send_error[1]=ESP_FAIL;
    ASSERT_TRUE(step(0,true)&&last_tick.send_partially_succeeded&&!last_tick.send_succeeded,"partial send reaches lifecycle fault input");
    ASSERT_TRUE(exo_assist_disarm()==ESP_OK&&exo_controller_disarm()==ESP_OK,"fault cleanup");
    return true;
}
#ifndef EXO_ADAPTIVE_ACTIVE_MAIN
#define EXO_ADAPTIVE_ACTIVE_MAIN main
#endif
int EXO_ADAPTIVE_ACTIVE_MAIN(void) {
    if(!test_defaults_and_shadow()||!test_active_and_boundaries()||!test_faults()) return 1;
    printf("PASS ACTIVE real-C chain cap=%u mNm: defaults/SHADOW zero; nonzero AFO-to-final-pair; signed cap/overflow/NaN/stale/epoch; frozen config; STOP; 10 fault/rearm cases; partial CAN propagation\n",(unsigned)EXO_ADAPTIVE_CONFIG_MAX_MNM);
    return 0;
}
