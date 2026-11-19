/* Regular selection -> START -> bounded paired commands, without UART setup. */
#define EXO_ADAPTIVE_ACTIVE_MAIN historical_active_main
#include "adaptive_active_replay.c"

static bool mode_begin(exo_assist_execution_state_t state, uint16_t maximum,
                       uint8_t left, uint8_t right) {
    (void)exo_assist_disarm();(void)exo_controller_disarm();
    ASSERT_TRUE(exo_adaptive_runtime_init()==ESP_OK,"boot runtime");
    ASSERT_TRUE(exo_assist_init()==ESP_OK&&exo_controller_init()==ESP_OK,"boot modules");
    exo_assist_config_t cfg=active_selection(state);
    cfg.maximum_assist_torque_mnm=maximum;
    cfg.left_level_percent=left;cfg.right_level_percent=right;
    const exo_joint_command_t base={.kd=.1f};
    ASSERT_TRUE(exo_controller_set_mode(EXO_CONTROL_MODE_DAMPING,EXO_REFERENCE_CONSTANT)==ESP_OK,"product base");
    ASSERT_TRUE(exo_controller_set_joint_command(EXO_HIP_JOINT_LEFT,&base)==ESP_OK&&
        exo_controller_set_joint_command(EXO_HIP_JOINT_RIGHT,&base)==ESP_OK,"existing product damping");
    ASSERT_TRUE(exo_assist_set_config(&cfg)==ESP_OK,"direct selection from transparent");
    exo_adaptive_config_t parameters;
    ASSERT_TRUE(exo_adaptive_runtime_config(&parameters,NULL)&&parameters.overall_strength==0,"save alone does not enable profile");
    exo_assist_run_contract_t preview;
    ASSERT_TRUE(exo_assist_preview_run_contract(&preview)==ESP_OK&&exo_controller_validate_run_contract(&preview)==ESP_OK,"ordinary preflight");
    ASSERT_TRUE(exo_assist_arm()==ESP_OK,"ordinary START without prepare");
    ASSERT_TRUE(exo_assist_get_snapshot(&current_assist)&&current_assist.strategy_version==12,"regular mode identity");
    if(state!=EXO_ASSIST_EXEC_DISABLED) {
        ASSERT_TRUE(exo_adaptive_runtime_config(&parameters,NULL)&&parameters.overall_strength==1&&
            parameters.leg[0].flex_amplitude_nm==(float)maximum/1000.0f&&
            parameters.leg[1].ext_amplitude_nm==(float)maximum/1000.0f,"selected maximum scales profile without clipping shape");
    }
    const exo_controller_assist_contract_t contract={.execution_state=state,
        .activation_epoch=current_assist.activation_epoch,.strategy_id=5,.maximum_assist_torque_mnm=maximum};
    const exo_controller_centers_t centers={0};
    ASSERT_TRUE(exo_controller_arm(&centers,10000,&contract)==ESP_OK,"ordinary controller arm");
    return true;
}
static bool mode_tick(unsigned n,unsigned fault) {
    int64_t us=10000+(int64_t)n*10000;float t=n*.01f;
    if(fault==10)us+=30000;
    exo_sensor_snapshot_t sensor={.captured_us=us,.sequence=n+1,.reset_epoch=1,
        .valid_sensor_mask=EXO_SENSOR_LEFT_HIP_KINEMATICS|EXO_SENSOR_RIGHT_HIP_KINEMATICS};
    sensor.left=make_joint(10+20*cosf(2*PI_F*.7f*t),-40*PI_F*.7f*sinf(2*PI_F*.7f*t),(uint64_t)us,n+1);
    sensor.right=make_joint(10+20*cosf(2*PI_F*1.2f*t+.7f),-40*PI_F*1.2f*sinf(2*PI_F*1.2f*t+.7f),(uint64_t)us,n+1);
    if(fault==1)sensor.left.joint_valid=sensor.right.joint_valid=false;
    if(fault==2)sensor.left.joint_sample.position_deg=sensor.right.joint_sample.position_deg=NAN;
    if(fault==3)sensor.left.source_timestamp_us=sensor.right.source_timestamp_us=(uint64_t)us+1;
    if(fault==4)sensor.left.age_us=sensor.right.age_us=50001;
    if(fault==5)sensor.reset_epoch=2;
    if(fault==7)sensor.left.source_feedback_count=sensor.right.source_feedback_count=1;
    if(fault==8)sensor.left.joint_sample.velocity_deg_s=sensor.right.joint_sample.velocity_deg_s=401;
    if(fault==9)sensor.left.joint_sample.position_deg=sensor.right.joint_sample.position_deg=91;
    mock_clock=us;
    ASSERT_TRUE(exo_assist_get_snapshot(&current_assist),"contract snapshot");
    exo_adaptive_runtime_update(&sensor,exo_adaptive_runtime_permit(&current_assist,true),3,fault==6);
    exo_gait_estimate_t gait={.sequence=n+1};
    ASSERT_TRUE(exo_assist_process_tick(&sensor,&gait,&last_policy)==ESP_OK,"policy tick");
    ASSERT_TRUE(exo_controller_process_tick(us,&last_policy,sensor.sequence,gait.sequence,&last_tick)==ESP_OK,"paired final commands");
    exo_adaptive_runtime_finish(last_tick.final_command.left.torque_ff_nm,last_tick.final_command.right.torque_ff_nm,
        last_tick.actuator_b_send_succeeded,last_tick.actuator_a_send_succeeded,100);
    exo_assist_report_output(&last_policy,last_tick.assist_applied);
    exo_assist_snapshot_t reported;
    ASSERT_TRUE(exo_assist_get_snapshot(&reported),"read ordinary assistance status");
    const bool nonzero=fabsf(last_policy.suggestion.left.torque_ff_nm)>1e-6f||
        fabsf(last_policy.suggestion.right.torque_ff_nm)>1e-6f;
    ASSERT_TRUE(reported.real_output_enabled==(last_tick.assist_applied&&nonzero),
        "zero-command delivery is not nonzero assistance");
    float maximum=current_assist.active.maximum_assist_torque_mnm/1000.0f;
    ASSERT_TRUE(fabsf(s_last_sent.left.torque_ff_nm)<=maximum*current_assist.active.left_level_percent/100.0f+1e-6f&&
        fabsf(s_last_sent.right.torque_ff_nm)<=maximum*current_assist.active.right_level_percent/100.0f+1e-6f,"independent level and torque bounds");
    ASSERT_TRUE(s_last_sent.left.kd==last_tick.base_command.left.kd&&
        s_last_sent.right.kd==last_tick.base_command.right.kd&&
        s_last_sent.left.kd>=0&&s_last_sent.left.kd<=.1f,"product damping and startup ramp retained");
    return true;
}
static bool regular_mode(void) {
    ASSERT_TRUE(EXO_ADAPTIVE_ASSIST_MODE&&EXO_ADAPTIVE_CONFIG_MAX_MNM==15000,"regular build");
    ASSERT_TRUE(mode_begin(EXO_ASSIST_EXEC_ACTIVE,500,25,75),"regular mode entry");
    unsigned nz=0;
    for(unsigned n=0;n<3000;n++) {
        ASSERT_TRUE(mode_tick(n,false),"active tick");
        if(n==0) {
            exo_assist_snapshot_t reported;ASSERT_TRUE(exo_assist_get_snapshot(&reported),"initial status");
            ASSERT_TRUE(reported.unavailable_reason==EXO_ASSIST_UNAVAILABLE_AFO_ACQUIRING&&
                !reported.real_output_enabled,"initial acquisition is visible with zero output");
        }
        nz+=fabsf(s_last_sent.left.torque_ff_nm)>.001f&&fabsf(s_last_sent.right.torque_ff_nm)>.001f;
    }
    ASSERT_TRUE(nz>1000&&last_tick.assist_applied,"nonzero bilateral assistance reaches final commands");
    exo_adaptive_config_t parameters=exo_adaptive_trial_preset();
    ASSERT_TRUE(exo_adaptive_runtime_set(&parameters)==ESP_ERR_INVALID_STATE,"ACTIVE parameters locked");
    ASSERT_TRUE(mode_tick(3000,true)&&zero_pair(),"invalid feedback clears same tick");
    for(unsigned n=3001;n<4500;n++) {
        ASSERT_TRUE(mode_tick(n,false)&&zero_pair(),"fault does not silently rearm");
        exo_assist_snapshot_t reported;ASSERT_TRUE(exo_assist_get_snapshot(&reported),"latched fault status");
        ASSERT_TRUE(reported.unavailable_reason==EXO_ASSIST_UNAVAILABLE_AFO_REARM_REQUIRED,
            "ordinary status explains zero output after feedback recovers");
    }
    ASSERT_TRUE(exo_assist_disarm()==ESP_OK&&exo_controller_disarm()==ESP_OK,"STOP");
    ASSERT_TRUE(exo_assist_get_snapshot(&current_assist)&&!current_assist.active_valid&&
        exo_adaptive_runtime_permit(&current_assist,true)==0,"STOP revokes permit");
    /* A real STOP/START keeps the modules and engine allocated. Verify that a
     * fresh activation reacquires after a hard latch without rebooting them. */
    ASSERT_TRUE(exo_assist_arm()==ESP_OK,"restart same initialized modules");
    ASSERT_TRUE(exo_assist_get_snapshot(&current_assist),"new activation");
    const exo_controller_assist_contract_t restart_contract={.execution_state=EXO_ASSIST_EXEC_ACTIVE,
        .activation_epoch=current_assist.activation_epoch,.strategy_id=5,.maximum_assist_torque_mnm=500};
    const exo_controller_centers_t restart_centers={0};
    ASSERT_TRUE(exo_controller_arm(&restart_centers,45010000,&restart_contract)==ESP_OK,"restart controller");
    unsigned resumed=0;
    for(unsigned n=4500;n<7500;n++) {ASSERT_TRUE(mode_tick(n,0),"restarted tick");resumed+=!zero_pair();}
    ASSERT_TRUE(resumed>1000,"STOP/START clears the old permit latch and reacquires without reboot");
    ASSERT_TRUE(mode_begin(EXO_ASSIST_EXEC_ACTIVE,500,0,100),"restart with left off");
    unsigned right_nz=0;
    for(unsigned n=0;n<3000;n++) {
        ASSERT_TRUE(mode_tick(n,false)&&s_last_sent.left.torque_ff_nm==0,"zero left level");
        right_nz+=fabsf(s_last_sent.right.torque_ff_nm)>.001f;
    }
    ASSERT_TRUE(right_nz>1000,"right still assists after restart");
    ASSERT_TRUE(mode_begin(EXO_ASSIST_EXEC_SHADOW,500,100,100),"optional observation");
    unsigned suggestions=0;
    for(unsigned n=0;n<3000;n++) {
        ASSERT_TRUE(mode_tick(n,false)&&zero_pair(),"observation adds no torque");
        suggestions+=fabsf(last_policy.suggestion.left.torque_ff_nm)>.001f;
    }
    ASSERT_TRUE(suggestions>1000,"observation computes same profile");
    ASSERT_TRUE(mode_begin(EXO_ASSIST_EXEC_ACTIVE,0,100,100),"zero maximum");
    for(unsigned n=0;n<3000;n++)ASSERT_TRUE(mode_tick(n,false)&&zero_pair(),"zero maximum stays zero");
    for(unsigned fault=1;fault<=10;fault++) {
        if(fault==8)continue; /* valid operating-speed excursion tested separately */
        ASSERT_TRUE(mode_begin(EXO_ASSIST_EXEC_ACTIVE,500,100,100),"fault case entry");
        unsigned before=0;
        for(unsigned n=0;n<1500;n++) {
            ASSERT_TRUE(mode_tick(n,0),"warmup");
            before+=!zero_pair();
        }
        ASSERT_TRUE(before>500,"fault test starts from producing assistance");
        ASSERT_TRUE(mode_tick(1500,fault)&&zero_pair(),"hard fault zeros both commands immediately");
        for(unsigned n=1501;n<3000;n++)ASSERT_TRUE(mode_tick(n,0)&&zero_pair(),"hard fault remains latched despite phase recovery");
    }
    /* Test the real ordinary START profile, runtime, policy and composer at
     * 100 Hz; merely accepting 15000 in the configuration is insufficient. */
    ASSERT_TRUE(mode_begin(EXO_ASSIST_EXEC_ACTIVE,15000,100,100),"15 Nm ordinary selection");
    float peak[2]={0},previous[2]={0};
    for(unsigned n=0;n<6000;n++) {
        ASSERT_TRUE(mode_tick(n,0),"15 Nm full command path");
        const float sent_tau[2]={s_last_sent.left.torque_ff_nm,s_last_sent.right.torque_ff_nm};
        for(unsigned i=0;i<2;i++) {
            peak[i]=fmaxf(peak[i],fabsf(sent_tau[i]));
            ASSERT_TRUE(fabsf(sent_tau[i])<=15.00001f,"15 Nm hard bound");
            ASSERT_TRUE(fabsf(sent_tau[i]-previous[i])<=2.5001f,"250 Nm/s ordinary waveform bound");
            previous[i]=sent_tau[i];
        }
    }
    printf("METRIC ordinary RS01 AFO command peak L/R %.4f / %.4f Nm\n",peak[0],peak[1]);
    ASSERT_TRUE(peak[0]>14.8f&&peak[1]>14.8f,"both legs reach approximately15 Nm, not just a configured ceiling");
    ASSERT_TRUE(mode_tick(6000,1)&&zero_pair(),"15 Nm feedback fault immediately zeros both");
    printf("PASS regular AFO mode: direct selection/START, paired nonzero ticks=%u, levels, zero maximum, damping, SHADOW, fault and STOP/restart\n",nz);
    return true;
}
/* CAN RX preempts the control task between wakeup and snapshot copy.
 * Compare the old wakeup clock against the actual capture-complete clock
 * using the real builder, runtime, policy and final command composer. */
static bool capture_race(bool old_clock) {
    ASSERT_TRUE(mode_begin(EXO_ASSIST_EXEC_ACTIVE,15000,100,100),"race entry");
    exo_sensor_snapshot_builder_t builder;
    const exo_sensor_snapshot_config_t cfg={.feedback_freshness_us=50000,
        .pair_skew_warning_us=20000,.available_sensor_mask=EXO_SENSOR_CURRENT_HARDWARE_MASK};
    ASSERT_TRUE(exo_sensor_snapshot_builder_init(&builder,&cfg),"real builder");
    unsigned nonzero=0,cycle_seen[2]={0},verified_cycles=0;
    for(unsigned n=0;n<6000;n++) {
        const int64_t wake=10000+(int64_t)n*10000,capture=wake+40;
        const uint64_t source=(uint64_t)(wake+(n==1500?20:-100));
        float t=n*.01f;
        exo_sensor_joint_snapshot_t left=make_joint(10+20*cosf(2*PI_F*t),
            -40*PI_F*sinf(2*PI_F*t),source,n+1);
        exo_sensor_joint_snapshot_t right=make_joint(10+20*cosf(2*PI_F*t+PI_F),
            -40*PI_F*sinf(2*PI_F*t+PI_F),source,n+1);
        exo_sensor_snapshot_t sensor;
        const int64_t now=old_clock?wake:capture;
        ASSERT_TRUE(exo_sensor_snapshot_build(&builder,now,&left,&right,&sensor),"build race snapshot");
        mock_clock=capture;
        exo_adaptive_runtime_update(&sensor,current_assist.activation_epoch,3,false);
        exo_gait_estimate_t gait={.sequence=sensor.sequence};
        ASSERT_TRUE(exo_assist_process_tick(&sensor,&gait,&last_policy)==ESP_OK,"race policy");
        ASSERT_TRUE(exo_controller_process_tick(now,&last_policy,sensor.sequence,gait.sequence,&last_tick)==ESP_OK,"same capture identity");
        exo_adaptive_runtime_finish(s_last_sent.left.torque_ff_nm,s_last_sent.right.torque_ff_nm,true,true,100);
        exo_adaptive_log_t log;ASSERT_TRUE(exo_adaptive_runtime_snapshot(&log),"race diagnostics");
        if(n>=1500) {
            ASSERT_TRUE(log.output.leg[0].rearm_required==old_clock,"only wakeup clock falsely latches");
            if(old_clock)ASSERT_TRUE(zero_pair(),"reproduce permanent zero after one race");
            else {
                nonzero+=!zero_pair();
                float tau[2]={s_last_sent.left.torque_ff_nm,s_last_sent.right.torque_ff_nm};
                for(unsigned side=0;side<2;side++) {
                    if(tau[side]>.01f)cycle_seen[side]|=1;
                    if(tau[side]<-.01f)cycle_seen[side]|=2;
                }
                if(n%100==99) {
                    ASSERT_TRUE(cycle_seen[0]==3&&cycle_seen[1]==3,"each post-race stride has flexion and extension commands");
                    verified_cycles++;cycle_seen[0]=cycle_seen[1]=0;
                }
            }
        }
    }
    if(!old_clock)ASSERT_TRUE(verified_cycles==45&&nonzero>0,"capture-complete clock sustains all 45 bilateral strides");
    exo_adaptive_hard_fault_t first;
    ASSERT_TRUE(exo_adaptive_runtime_first_fault(0,&first)==old_clock,"sticky first fault recorded");
    if(old_clock) {
        ASSERT_TRUE(first.tick.input.sequence==1501&&first.age_us==UINT32_MAX,
            "first cause not overwritten by later recovery");
        exo_adaptive_lock_record_t r;while(exo_adaptive_runtime_pop_lock_loss(&r)) { }
        exo_sensor_snapshot_t stopped={.captured_us=60020000,.sequence=6001};
        exo_adaptive_runtime_update(&stopped,0,1,false);
        exo_adaptive_runtime_finish(0,0,false,false,100);
        ASSERT_TRUE(exo_adaptive_runtime_first_fault(0,&first)&&first.tick.input.sequence==1501,
            "STOP and draining diagnostics preserve first failure");
        stopped.captured_us+=10000;stopped.sequence++;
        exo_adaptive_runtime_update(&stopped,99,3,false);
        exo_adaptive_runtime_finish(0,0,false,false,100);
        ASSERT_TRUE(exo_adaptive_runtime_first_fault(0,&first)&&first.tick.input.permit_epoch==99,
            "next START owns a new first failure record");
    }
    printf("PASS RX-during-copy old_clock=%u nonzero_post_race=%u\n",old_clock,nonzero);
    return true;
}
static bool source_spacing(unsigned interval_us,bool outside_window) {
    ASSERT_TRUE(mode_begin(EXO_ASSIST_EXEC_ACTIVE,15000,100,100),"asynchronous source entry");
    unsigned resumed=0;uint32_t source_count=0;uint64_t previous=0;
    for(unsigned n=0;n<3000;n++) {
        const int64_t now=10000+(int64_t)n*10000;
        const uint64_t anchor=15010000;
        uint64_t source=(uint64_t)now;
        if(n>1500&&n<1500+(interval_us+9999)/10000)source=anchor;
        if(n==1500+(interval_us+9999)/10000)source=anchor+interval_us;
        if(source!=previous)source_count++;
        previous=source;
        float t=(float)(source-10000)*1e-6f;
        exo_sensor_snapshot_t s={.captured_us=now,.sequence=n+1,.reset_epoch=1,
            .valid_sensor_mask=EXO_SENSOR_LEFT_HIP_KINEMATICS|EXO_SENSOR_RIGHT_HIP_KINEMATICS};
        s.left=make_joint(10+20*cosf(2*PI_F*t),-40*PI_F*sinf(2*PI_F*t),source,source_count);
        s.right=make_joint(10+20*cosf(2*PI_F*t+PI_F),-40*PI_F*sinf(2*PI_F*t+PI_F),source,source_count);
        if(outside_window&&n==1500+(interval_us+9999)/10000)
            s.left.joint_sample.velocity_deg_s=401;
        s.left.age_us=s.right.age_us=(uint32_t)(now-(int64_t)source);
        mock_clock=now;
        exo_adaptive_runtime_update(&s,current_assist.activation_epoch,3,false);
        exo_gait_estimate_t gait={.sequence=s.sequence};
        ASSERT_TRUE(exo_assist_process_tick(&s,&gait,&last_policy)==ESP_OK,"spacing policy");
        ASSERT_TRUE(exo_controller_process_tick(now,&last_policy,s.sequence,gait.sequence,&last_tick)==ESP_OK,"spacing final commands");
        exo_adaptive_runtime_finish(s_last_sent.left.torque_ff_nm,s_last_sent.right.torque_ff_nm,true,true,100);
        exo_adaptive_log_t log;ASSERT_TRUE(exo_adaptive_runtime_snapshot(&log),"spacing diagnostic");
        if(n>=1510) {
            ASSERT_TRUE(log.output.leg[0].rearm_required==(interval_us>50000),"source spacing boundary");
            if(interval_us>50000)ASSERT_TRUE(zero_pair(),"expired integration cannot auto-rearm");
            else resumed+=!zero_pair();
        }
    }
    if(interval_us<=50000)ASSERT_TRUE(resumed>1000,"valid source spacing resumes paired torque");
    if(interval_us>50000) {
        exo_adaptive_hard_fault_t first;
        ASSERT_TRUE(exo_adaptive_runtime_first_fault(0,&first)&&first.source_interval_us==interval_us,
            "first failure retains the real source interval");
    }
    printf("PASS source spacing %u us outside=%u resumed=%u\n",interval_us,outside_window,resumed);return true;
}
static bool speed_window_resume(void) {
    ASSERT_TRUE(mode_begin(EXO_ASSIST_EXEC_ACTIVE,15000,100,100),"speed excursion entry");
    for(unsigned n=0;n<1500;n++)ASSERT_TRUE(mode_tick(n,0),"speed warmup");
    ASSERT_TRUE(mode_tick(1500,8)&&zero_pair(),"operating speed limit immediately zeros commands");
    exo_adaptive_log_t log;ASSERT_TRUE(exo_adaptive_runtime_snapshot(&log)&&
        !log.output.leg[0].rearm_required&&(log.output.leg[0].faults&EXO_ADAPTIVE_FAULT_RANGE),"operating pause has no hard latch");
    unsigned resumed=0;
    for(unsigned n=1501;n<3000;n++){ASSERT_TRUE(mode_tick(n,0),"speed recovery");resumed+=!zero_pair();}
    ASSERT_TRUE(resumed>900,"fresh periods and fade recover without START");
    return true;
}
#ifndef EXO_ADAPTIVE_MODE_MAIN
#define EXO_ADAPTIVE_MODE_MAIN main
#endif
int EXO_ADAPTIVE_MODE_MAIN(void) {return regular_mode()&&capture_race(true)&&capture_race(false)&&speed_window_resume()&&
    source_spacing(30500,false)&&source_spacing(40000,false)&&source_spacing(50000,false)&&
    source_spacing(51000,false)&&source_spacing(51000,true)?0:1;}
