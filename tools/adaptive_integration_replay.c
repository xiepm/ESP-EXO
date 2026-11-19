/* 复用原集成夹具，只替代NVS/RTOS/最终CAN；AFO到五元命令合成全是真实C代码。 */
#define main historical_phase10_main
#include "phase10_assist_integration_replay.c"
#undef main
#include "exo_adaptive_runtime.h"
#include "nvs.h"
#include <stdlib.h>
static uint8_t nvs_bytes[EXO_ADAPTIVE_CONFIG_BYTES];
static bool have_nvs;
static int64_t mock_clock;
int64_t esp_timer_get_time(void) { return ++mock_clock; }
esp_err_t nvs_open(const char *ns,nvs_open_mode_t mode,nvs_handle_t *h) {
    (void)ns;(void)mode;*h=1;return ESP_OK;
}
esp_err_t nvs_get_blob(nvs_handle_t h,const char *key,void *data,size_t *size) {
    (void)h;(void)key;if(!have_nvs)return ESP_ERR_NVS_NOT_FOUND;
    if(*size<sizeof(nvs_bytes))return ESP_ERR_NVS_INVALID_LENGTH;
    memcpy(data,nvs_bytes,sizeof(nvs_bytes));*size=sizeof(nvs_bytes);return ESP_OK;
}
esp_err_t nvs_set_blob(nvs_handle_t h,const char *key,const void *data,size_t size) {
    (void)h;(void)key;if(size!=sizeof(nvs_bytes))return ESP_ERR_INVALID_ARG;
    memcpy(nvs_bytes,data,size);have_nvs=true;return ESP_OK;
}
esp_err_t nvs_commit(nvs_handle_t h) { (void)h;return ESP_OK; }
void nvs_close(nvs_handle_t h) { (void)h; }
#ifndef EXO_ADAPTIVE_REUSE_FIXTURE
static bool integrated(void) {
    ASSERT_TRUE(exo_adaptive_runtime_init()==ESP_OK,"runtime init");
    exo_adaptive_config_t c;uint32_t generation;
    ASSERT_TRUE(exo_adaptive_runtime_config(&c,&generation)&&c.overall_strength==0,"safe boot default");
    ASSERT_TRUE(exo_adaptive_runtime_load()==ESP_ERR_NVS_NOT_FOUND,"missing record explicit");
    c.overall_strength=1;
    for(unsigned i=0;i<2;i++)c.leg[i]=(exo_adaptive_leg_config_t){
        .enabled=1,.strength=1,.flex_amplitude_nm=.2f,.flex_peak_phase=.75f,.flex_width=.1f,
        .ext_amplitude_nm=.15f,.ext_peak_phase=.2f,.ext_width=.1f,.torque_limit_nm=.25f,
        .slew_nm_s=2,.positive_power_w=1,.negative_power_w=1,.minimum_angle_deg=-40,
        .maximum_angle_deg=60,.maximum_velocity_deg_s=350,.fade_s=1,.transition_s=.5f};
    ASSERT_TRUE(exo_adaptive_runtime_set(&c)==ESP_OK,"publish config");
    ASSERT_TRUE(exo_adaptive_runtime_save()==ESP_OK,"save NVS");
    exo_adaptive_config_t defaults=exo_adaptive_default_config();
    ASSERT_TRUE(exo_adaptive_runtime_set(&defaults)==ESP_OK,"safe defaults");
    ASSERT_TRUE(exo_adaptive_runtime_load()==ESP_OK,"restore config");
    nvs_bytes[30]^=1;ASSERT_TRUE(exo_adaptive_runtime_load()!=ESP_OK,"reject bad CRC");
    ASSERT_TRUE(exo_adaptive_runtime_config(&defaults,&generation)&&defaults.overall_strength==1,"invalid load does not mutate");
    ASSERT_TRUE(exo_assist_init()==ESP_OK&&exo_controller_init()==ESP_OK,"existing init");
    exo_assist_config_t selected={.schema_version=1,.mode=EXO_ASSIST_MODE_ADAPTIVE_GAUSSIAN,
        .strategy_id=EXO_ASSIST_STRATEGY_ADAPTIVE_GAUSSIAN,.requested_execution_state=EXO_ASSIST_EXEC_ACTIVE,
        .left_level_percent=100,.right_level_percent=100,.maximum_assist_torque_mnm=500};
    ASSERT_TRUE(exo_assist_set_config(&selected)==ESP_ERR_NOT_SUPPORTED,"ACTIVE must remain blocked");
    selected.requested_execution_state=EXO_ASSIST_EXEC_SHADOW;
    ASSERT_TRUE(exo_assist_set_config(&selected)==ESP_OK&&exo_assist_arm()==ESP_OK,"shadow arm in mock only");
    exo_assist_snapshot_t assistance;exo_assist_get_snapshot(&assistance);
    exo_controller_assist_contract_t contract={.execution_state=EXO_ASSIST_EXEC_ACTIVE,
        .activation_epoch=assistance.activation_epoch,.strategy_id=EXO_ASSIST_STRATEGY_ADAPTIVE_GAUSSIAN,
        .maximum_assist_torque_mnm=500};
    exo_controller_centers_t centers={0};
    ASSERT_TRUE(exo_controller_arm(&centers,10000,&contract)==ESP_ERR_INVALID_ARG,"controller also rejects ACTIVE");
    contract.execution_state=EXO_ASSIST_EXEC_SHADOW;
    ASSERT_TRUE(exo_controller_arm(&centers,10000,&contract)==ESP_OK,"existing controller shadow");
    unsigned suggestions=0;
    for(unsigned n=0;n<4000;n++) {
        int64_t us=10000+(int64_t)n*10000;
        exo_sensor_snapshot_t sensors={.captured_us=us,.sequence=n+1,.reset_epoch=1,
            .valid_sensor_mask=EXO_SENSOR_LEFT_HIP_KINEMATICS|EXO_SENSOR_RIGHT_HIP_KINEMATICS};
        float t=n*.01f;
        sensors.left=make_joint(18*cosf(2*PI_F*t),-36*PI_F*sinf(2*PI_F*t),(uint64_t)us,n+1);
        sensors.right=make_joint(14*cosf(2*PI_F*1.2f*t+.7f),-28*PI_F*1.2f*sinf(2*PI_F*1.2f*t+.7f),(uint64_t)us,n+1);
        mock_clock=us;
        exo_adaptive_runtime_update(&sensors,assistance.activation_epoch,3,false);
        exo_gait_estimate_t gait={.sequence=n+1};exo_assist_result_t result;
        ASSERT_TRUE(exo_assist_process_tick(&sensors,&gait,&result)==ESP_OK,"policy step");
        suggestions+=fabsf(result.suggestion.left.torque_ff_nm)>1e-4f;
        exo_controller_tick_result_t tick;
        ASSERT_TRUE(exo_controller_process_tick(us,&result,sensors.sequence,gait.sequence,&tick)==ESP_OK,"existing final exit");
        ASSERT_TRUE(!tick.assist_applied&&s_last_sent.left.torque_ff_nm==0&&s_last_sent.right.torque_ff_nm==0,"shadow never adds torque");
        exo_adaptive_runtime_finish(tick.final_command.left.torque_ff_nm,tick.final_command.right.torque_ff_nm,true,true,123);
        if(n==3000) { c.leg[0].flex_peak_phase=.1f;ASSERT_TRUE(exo_adaptive_runtime_set(&c)==ESP_OK,"runtime parameter snapshot"); }
    }
    ASSERT_TRUE(suggestions>3000,"actual nonzero planner reached policy");
    exo_adaptive_log_t log;ASSERT_TRUE(exo_adaptive_runtime_snapshot(&log),"snapshot");
    ASSERT_TRUE(log.dropped_logs>0&&log.output.config_generation==generation+1,"ring overflow and atomic version");
    unsigned popped=0;while(exo_adaptive_runtime_pop(&log))popped++;
    ASSERT_TRUE(popped==EXO_ADAPTIVE_LOG_CAPACITY,"bounded queue");
    exo_joint_command_pair_t old;
    ASSERT_TRUE(!exo_adaptive_runtime_preview(1,1,&old)&&old.left.torque_ff_nm==0,"stale preview rejected");
    ASSERT_TRUE(exo_assist_disarm()==ESP_OK&&exo_controller_disarm()==ESP_OK,"mock STOP");
    printf("PASS runtime integration: NVS CRC defaults snapshots; %u nonzero previews; all 4000 final pairs contain zero added assist; ACTIVE rejected twice; ring32/drop\n",suggestions);
    return true;
}
int main(void) {
    if(historical_phase10_main()!=0)return 1;
    return integrated()?0:1;
}
#endif
