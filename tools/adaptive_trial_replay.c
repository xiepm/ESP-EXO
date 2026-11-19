/* Host-only bounded trial contract and actual C command-composition test. */
#define EXO_ADAPTIVE_ACTIVE_MAIN active_existing_main
#include "adaptive_active_replay.c"

static bool trial_tick(unsigned n,bool invalid) {
    int64_t us=10000+(int64_t)n*10000;float t=n*.01f;
    exo_sensor_snapshot_t sensor={.captured_us=us,.sequence=n+1,.reset_epoch=1,
        .valid_sensor_mask=EXO_SENSOR_LEFT_HIP_KINEMATICS|EXO_SENSOR_RIGHT_HIP_KINEMATICS};
    sensor.left=make_joint(8*cosf(2*PI_F*.7f*t),-16*PI_F*.7f*sinf(2*PI_F*.7f*t),(uint64_t)us,n+1);
    sensor.right=make_joint(8*cosf(2*PI_F*.9f*t+.7f),-16*PI_F*.9f*sinf(2*PI_F*.9f*t+.7f),(uint64_t)us,n+1);
    if(invalid)sensor.left.joint_valid=sensor.right.joint_valid=false;
    mock_clock=us;
    ASSERT_TRUE(exo_assist_get_snapshot(&current_assist),"current contract");
    exo_adaptive_runtime_update(&sensor,exo_adaptive_runtime_permit(&current_assist,true),3,false);
    exo_gait_estimate_t gait={.sequence=n+1};
    ASSERT_TRUE(exo_assist_process_tick(&sensor,&gait,&last_policy)==ESP_OK,"trial policy");
    ASSERT_TRUE(exo_controller_process_tick(us,&last_policy,sensor.sequence,gait.sequence,&last_tick)==ESP_OK,"paired command composition");
    exo_adaptive_runtime_finish(last_tick.final_command.left.torque_ff_nm,last_tick.final_command.right.torque_ff_nm,
        last_tick.actuator_b_send_succeeded,last_tick.actuator_a_send_succeeded,100);
    float torque[2]={s_last_sent.left.torque_ff_nm,s_last_sent.right.torque_ff_nm};
    float velocity[2]={sensor.left.joint_sample.velocity_deg_s,sensor.right.joint_sample.velocity_deg_s};
    for(unsigned i=0;i<2;i++) {
        float power=torque[i]*velocity[i]*EXO_ADAPTIVE_PI/180;
        ASSERT_TRUE(fabsf(torque[i])<=5&&power<=2.00001f&&power>=-1.00001f,"5 Nm and directional power caps at final pair");
    }
    ASSERT_TRUE(s_last_sent.left.kp==0&&s_last_sent.left.kd==0&&s_last_sent.right.kp==0&&s_last_sent.right.kd==0,"zero base gains");
    return true;
}
static bool trial_contract(void) {
    ASSERT_TRUE(EXO_ADAPTIVE_TRIAL_ALLOWED&&EXO_ADAPTIVE_CONFIG_MAX_MNM==5000,"explicit trial identity");
    ASSERT_TRUE(begin_case(false,EXO_ASSIST_EXEC_SHADOW),"default shadow fixture");
    ASSERT_TRUE(!exo_adaptive_runtime_trial_ready(),"no automatic preset at boot");
    ASSERT_TRUE(exo_assist_disarm()==ESP_OK&&exo_controller_disarm()==ESP_OK,"idle fixture");
    ASSERT_TRUE(exo_adaptive_runtime_lock_active_config()==ESP_ERR_INVALID_STATE,"zero/default config cannot arm trial ACTIVE");
    ASSERT_TRUE(exo_adaptive_runtime_prepare_trial()==ESP_OK&&exo_adaptive_runtime_trial_ready(),"explicit complete preset");
    exo_assist_config_t cfg=active_selection(EXO_ASSIST_EXEC_ACTIVE);cfg.maximum_assist_torque_mnm=5001;
    ASSERT_TRUE(exo_assist_set_config(&cfg)==ESP_ERR_INVALID_ARG,"reject above 5 Nm");
    cfg.maximum_assist_torque_mnm=5000;
    ASSERT_TRUE(exo_assist_set_config(&cfg)==ESP_OK&&exo_assist_arm()==ESP_OK,"manual trial arm");
    ASSERT_TRUE(exo_assist_get_snapshot(&current_assist)&&current_assist.strategy_version==5,"distinct strategy v5");
    ASSERT_TRUE(exo_adaptive_runtime_prepare_trial()==ESP_ERR_INVALID_STATE,"cannot reconfigure while armed");
    const exo_controller_assist_contract_t contract={.execution_state=EXO_ASSIST_EXEC_ACTIVE,
        .activation_epoch=current_assist.activation_epoch,.strategy_id=5,.maximum_assist_torque_mnm=5000};
    const exo_controller_centers_t centers={0};
    ASSERT_TRUE(exo_controller_arm(&centers,10000,&contract)==ESP_OK,"manual controller arm");
    unsigned nz=0;
    for(unsigned n=0;n<3000;n++) {
        ASSERT_TRUE(trial_tick(n,false),"bounded trial tick");
        nz+=fabsf(s_last_sent.left.torque_ff_nm)>.001f&&fabsf(s_last_sent.right.torque_ff_nm)>.001f;
    }
    ASSERT_TRUE(nz>1000,"nonzero paired assistance, not a zero-only pass");
    ASSERT_TRUE(trial_tick(3000,true)&&zero_pair(),"bad input clears on same tick");
    for(unsigned n=3001;n<4500;n++)ASSERT_TRUE(trial_tick(n,false)&&zero_pair(),"no automatic resume after relock");
    printf("PASS bounded trial: profile explicit/full match, strategy5, cap5000, positive/negative power, nonzero_pair_ticks=%u, same-tick fault zero and old-permit rearm\n",nz);
    return true;
}
int main(int argc,char **argv) {
    if(!trial_contract())return 1;
    if(argc==2) {
        uint8_t b[EXO_ADAPTIVE_CONFIG_BYTES];exo_adaptive_config_t c=exo_adaptive_trial_preset();
        if(!exo_adaptive_config_encode(&c,b,sizeof(b)))return 1;
        FILE *f=fopen(argv[1],"wbx");if(!f)return 1;
        size_t written=fwrite(b,1,sizeof(b),f);int closed=fclose(f);
        if(written!=sizeof(b)||closed)return 1;
    }
    return 0;
}
