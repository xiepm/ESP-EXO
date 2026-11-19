#define EXO_ADAPTIVE_REUSE_FIXTURE
#include "adaptive_integration_replay.c"
#include "exo_bench_peak.h"

static exo_sensor_snapshot_t input(unsigned n,float torque,unsigned side) {
    int64_t now=10000+(int64_t)n*10000;
    exo_sensor_snapshot_t in={.captured_us=now,.sequence=n+1,.reset_epoch=1};
    in.left=make_joint(0,0,(uint64_t)now,n+1);in.right=in.left;
    if(side==0)in.left.joint_sample.torque_nm=torque;else in.right.joint_sample.torque_nm=torque;
    return in;
}
static bool pure_guards(void) {
    for(unsigned kind=0;kind<9;kind++) {
        exo_bench_peak_t s;float torque=0;
        ASSERT_TRUE(exo_bench_peak_init(&s,0,15,0,0,10000),"init");
        exo_sensor_snapshot_t in=input(0,0,0);
        ASSERT_TRUE(exo_bench_peak_step(&s,&in,in.captured_us,in.sequence,&torque),"first sample");
        in=input(1,0,0);
        if(kind==0)in.left.joint_sample.position_deg=2.01f;
        if(kind==1)in.left.joint_sample.velocity_deg_s=20.01f;
        if(kind==2)in.left.joint_sample.temperature_c=50.01f;
        if(kind==3)in.left.joint_valid=false;
        if(kind==4)in.left.joint_sample.torque_nm=NAN;
        if(kind==5)in.reset_epoch=2;
        if(kind==6)in.captured_us+=40000;
        if(kind==7)in.left.joint_sample.torque_nm=-4;
        if(kind==8)in.right.joint_sample.torque_nm=1;
        torque=15;
        ASSERT_TRUE(!exo_bench_peak_step(&s,&in,in.captured_us,in.sequence,&torque)&&torque==0&&s.fault,"guard clears in same tick");
        if(kind==0)ASSERT_TRUE(s.maximum_displacement_by_side[0]>2&&s.maximum_displacement_by_side[1]==0,"identify moving side");
        if(kind==1)ASSERT_TRUE(s.maximum_speed_by_side[0]>20&&s.maximum_speed_by_side[1]==0,"identify overspeed side");
        in=input(10,0,0);
        ASSERT_TRUE(!exo_bench_peak_step(&s,&in,in.captured_us,in.sequence,&torque)&&torque==0,"fault cannot resume");
    }
    return true;
}
static bool chain(unsigned side,float peak) {
    ASSERT_TRUE(exo_controller_init()==ESP_OK&&exo_assist_init()==ESP_OK,"init chain");
    ASSERT_TRUE(exo_controller_disarm()==ESP_OK&&exo_assist_disarm()==ESP_OK,"idle");
    const exo_assist_config_t cfg={.schema_version=1,.mode=EXO_ASSIST_MODE_TRANSPARENT,
        .strategy_id=EXO_ASSIST_STRATEGY_TRANSPARENT,.requested_execution_state=EXO_ASSIST_EXEC_DISABLED,
        .maximum_assist_torque_mnm=500};
    ASSERT_TRUE(exo_assist_set_config(&cfg)==ESP_OK,"disabled assistance");
    ASSERT_TRUE(exo_controller_prepare_bench_peak(side,peak)==ESP_OK,"prepare one shot");
    exo_assist_run_contract_t pre;ASSERT_TRUE(exo_assist_preview_run_contract(&pre)==ESP_OK&&
        exo_controller_validate_run_contract(&pre)==ESP_OK,"preflight");
    const exo_controller_centers_t centers={0};
    const exo_controller_assist_contract_t contract={.execution_state=EXO_ASSIST_EXEC_DISABLED,
        .strategy_id=EXO_ASSIST_STRATEGY_TRANSPARENT,.maximum_assist_torque_mnm=500};
    ASSERT_TRUE(exo_controller_arm(&centers,10000,&contract)==ESP_OK,"arm");
    exo_controller_snapshot_t snapshot;ASSERT_TRUE(exo_controller_get_snapshot(&snapshot),"snapshot");
    ASSERT_TRUE(snapshot.selected.mode==EXO_CONTROL_MODE_DAMPING&&snapshot.selected.command.left.torque_ff_nm==0&&
        snapshot.selected.command.right.torque_ff_nm==0,"one shot consumed at START");
    float previous=0;
    for(unsigned n=0;n<600;n++) {
        exo_sensor_snapshot_t in=input(n,previous,side);exo_controller_tick_result_t tick;
        ASSERT_TRUE(exo_controller_process_sensor_tick(in.captured_us,&in,NULL,in.sequence,in.sequence,&tick)==ESP_OK,"real composition and send");
        const exo_joint_command_t *active=side==0?&tick.final_command.left:&tick.final_command.right;
        const exo_joint_command_t *other=side==0?&tick.final_command.right:&tick.final_command.left;
        ASSERT_TRUE(active->kp==0&&active->kd==0&&other->torque_ff_nm==0&&tick.send_succeeded,"zero gains and inactive side");
        ASSERT_TRUE(fabsf(active->torque_ff_nm)<=fabsf(peak)&&fabsf(active->torque_ff_nm-previous)<=.201f,"cap and rise/fall rate");
        if(fmaxf(fabsf(previous),fabsf(active->torque_ff_nm))<=1.0f)
            ASSERT_TRUE(fabsf(active->torque_ff_nm-previous)<=.0201f,"gentle low-torque slew both directions");
        if(n>=310)ASSERT_TRUE(active->torque_ff_nm==0,"MCU ends pulse without host STOP");
        previous=active->torque_ff_nm;
    }
    exo_bench_peak_t status;ASSERT_TRUE(exo_controller_get_bench_peak(&status),"retained result");
    ASSERT_TRUE(status.complete&&!status.fault&&status.sent_peak_nm==fabsf(peak)&&status.plateau_samples>=6,"verified paired-send peak and fresh plateau");
    const float duration=.6f+2.0f*(fminf(fabsf(peak),1.0f)/2.0f+fmaxf(fabsf(peak)-1.0f,0.0f)/20.0f);
    ASSERT_TRUE(fabsf((float)(status.last_us-status.start_us)*1e-6f-duration)<.011f,"profile completion within one control tick");
    exo_bench_trace_row_t row;unsigned count=0,dropped=0;
    ASSERT_TRUE(!exo_controller_get_bench_trace(0,&row,&count,&dropped),"trace refused while armed");
    ASSERT_TRUE(exo_controller_disarm()==ESP_OK,"stop");
    ASSERT_TRUE(exo_controller_get_bench_trace(0,&row,&count,&dropped)&&count==status.samples&&dropped==0,"trace includes complete pulse and freezes");
    ASSERT_TRUE(exo_controller_get_bench_trace(count-1,&row,&count,&dropped)&&row.command[0]==0&&row.command[1]==0,"trace terminal zero");
    return true;
}
static bool integrated_guard(void) {
    ASSERT_TRUE(exo_controller_prepare_bench_peak(0,15)==ESP_OK,"prepare guard test");
    const exo_controller_centers_t centers={0};
    const exo_controller_assist_contract_t contract={.execution_state=EXO_ASSIST_EXEC_DISABLED,
        .strategy_id=EXO_ASSIST_STRATEGY_TRANSPARENT,.maximum_assist_torque_mnm=500};
    ASSERT_TRUE(exo_controller_arm(&centers,10000,&contract)==ESP_OK,"arm guard test");
    float previous=0;
    for(unsigned n=0;n<=130;n++) {
        exo_sensor_snapshot_t in=input(n,previous,0);exo_controller_tick_result_t tick;
        if(n==130)in.left.joint_sample.position_deg=2.01f;
        esp_err_t error=exo_controller_process_sensor_tick(in.captured_us,&in,NULL,in.sequence,in.sequence,&tick);
        if(n<130) {
            ASSERT_TRUE(error==ESP_OK,"guard test rising torque");
            previous=tick.final_command.left.torque_ff_nm;
        } else {
            ASSERT_TRUE(previous>5&&error==ESP_ERR_INVALID_STATE&&tick.send_succeeded&&
                tick.final_command.left.torque_ff_nm==0&&tick.final_command.right.torque_ff_nm==0,
                "real controller sends zero pair then reports failure to lifecycle");
        }
    }
    ASSERT_TRUE(exo_controller_disarm()==ESP_OK,"guard stop");
    exo_bench_trace_row_t row;unsigned count=0,dropped=0;
    ASSERT_TRUE(exo_controller_get_bench_trace(130,&row,&count,&dropped)&&count==131&&dropped==0,"retain fault cycle and reset per pulse");
    ASSERT_TRUE(row.fault==EXO_BENCH_PEAK_MOVED&&row.q[0]>2&&row.command[0]==0&&row.sent[0],"trace fault input and successful zero pair");
    return true;
}
int main(void) {
    if(!pure_guards())return 1;
    for(unsigned side=0;side<2;side++)for(unsigned sign=0;sign<2;sign++) {
        if(!chain(side,sign?-0.5f:0.5f)||!chain(side,sign?-15.0f:15.0f))return 1;
    }
    if(!integrated_guard())return 1;
    puts("PASS new MCU-timed bench path: signed 0.5/15 both sides; real controller/send fixture; bounded rate; one-shot consumed; zero after deadline; 9 guard latches; powered-trajectory fault sends zero and propagates lifecycle error. HOST ONLY, no measured motor torque.");
    return 0;
}
