/* Production runtime + output path, with RTOS/CAN fakes. No device IO. */
#define EXO_ADAPTIVE_MODE_MAIN usb_unused_mode_main
#include "adaptive_mode_replay.c"
#include "exo_afo_trace_wire.h"
/* Included legacy helpers return bool; this executable must fail nonzero. */
#undef ASSERT_TRUE
#define ASSERT_TRUE(condition,message) do { if(!(condition)) { fprintf(stderr,"FAIL %d: %s\n",__LINE__,message);return 1; } } while(0)
int main(int argc,char **argv) {
    if(argc!=2)return 2;
    ASSERT_TRUE(mode_begin(EXO_ASSIST_EXEC_ACTIVE,15000,100,100),"regular AFO");
    exo_sensor_snapshot_t sensor={0};
    for(unsigned n=0;n<600;n++) {
        int64_t us=10000+(int64_t)n*10000;mock_clock=us;
        float phase=2*PI_F*(float)n*.01f;
        sensor=(exo_sensor_snapshot_t){.captured_us=us,.sequence=n+1,.reset_epoch=1,.valid_sensor_mask=15,
            .left=make_joint(10+20*cosf(phase),-40*PI_F*sinf(phase),(uint64_t)us,n+1),
            .right=make_joint(10-20*cosf(phase),40*PI_F*sinf(phase),(uint64_t)us,n+1)};
        if(n==420)sensor.left.joint_valid=false;
        ASSERT_TRUE(exo_assist_get_snapshot(&current_assist),"contract");
        exo_adaptive_runtime_update(&sensor,exo_adaptive_runtime_permit(&current_assist,true),3,false);
        exo_gait_estimate_t gait={.sequence=n+1};
        ASSERT_TRUE(exo_assist_process_tick(&sensor,&gait,&last_policy)==ESP_OK,"policy");
        ASSERT_TRUE(exo_controller_process_tick(us,&last_policy,n+1,n+1,&last_tick)==ESP_OK,"controller");
        exo_adaptive_runtime_finish(last_tick.final_command.left.torque_ff_nm,last_tick.final_command.right.torque_ff_nm,
            last_tick.actuator_b_send_succeeded,last_tick.actuator_a_send_succeeded,123);
        exo_adaptive_runtime_trace_finish(&sensor,&last_policy,&last_tick,ESP_OK);
    }
    uint32_t oldest=0,latest=0;exo_adaptive_runtime_trace_bounds(&oldest,&latest);
    ASSERT_TRUE(oldest==473&&latest==600,"bounded ring overwritten, sequence exposes loss");
    exo_afo_trace_t first,row,again;
    ASSERT_TRUE(exo_adaptive_runtime_trace_first(0,&first)&&first.id==421,"first hard fault retained past ring overflow");
    ASSERT_TRUE(first.leg[0].faults&EXO_ADAPTIVE_FAULT_INPUT,"first input fault");
    ASSERT_TRUE(first.leg[0].previous_source_us==4200000&&first.leg[0].source_us==4210000,"exact preceding source retained");
    ASSERT_TRUE(!exo_adaptive_runtime_trace_first(1,&row),"healthy peer not falsely marked");
    ASSERT_TRUE(exo_adaptive_runtime_trace_after(0,latest,&row)&&row.id==oldest,"old cursor recovers oldest available");
    ASSERT_TRUE(exo_adaptive_runtime_trace_after(0,latest,&again)&&memcmp(&row,&again,sizeof(row))==0,"read is nondestructive");
    ASSERT_TRUE(!exo_adaptive_runtime_trace_after(UINT32_MAX,latest,&row),"cursor overflow fails closed");
    ASSERT_TRUE(!exo_adaptive_runtime_trace_after(latest,latest,&row),"no future data");
    ASSERT_TRUE(exo_adaptive_runtime_trace_after(latest-1,latest,&row),"last row");
    ASSERT_TRUE(row.dt_us==10000&&row.previous_execution_us==123,"actual timing fields");
    ASSERT_TRUE(row.leg[0].category==EXO_AFO_COUNT_HARD&&row.leg[0].final_nm==0,"hard latch classified");
    ASSERT_TRUE(row.leg[1].source_count==600&&row.leg[1].source_us==6000000,"source identity");
    char line[1024],tiny[1];
    ASSERT_TRUE(exo_afo_trace_format(tiny,sizeof(tiny),&row,0,false)==-1,"truncation is not a valid row");
    ASSERT_TRUE(exo_afo_trace_format(line,sizeof(line),&row,2,false)==-1,"invalid side rejected");
    FILE *f=fopen(argv[1],"wb");ASSERT_TRUE(f!=NULL,"fixture output");
    fprintf(f,"AFOMETA,1,42,0.11.23,0000000000000000000000000000000000000000000000000000000000000000,473,600,128\n");
    exo_adaptive_config_t config;uint32_t generation=0;uint8_t bytes[EXO_ADAPTIVE_CONFIG_BYTES];
    ASSERT_TRUE(exo_adaptive_runtime_config(&config,&generation)&&exo_adaptive_config_encode(&config,bytes,sizeof(bytes)),"actual config snapshot");
    fprintf(f,"AFOCFG,1,%lu,",(unsigned long)generation);
    for(unsigned i=0;i<sizeof(bytes);i++)fprintf(f,"%02x",bytes[i]);
    fprintf(f,"\n");
    for(unsigned i=0;i<2;i++) {
        ASSERT_TRUE(exo_afo_trace_format(line,sizeof(line),&row,i,false)>0,"wire row");fprintf(f,"%s\n",line);
    }
    ASSERT_TRUE(exo_afo_trace_format(line,sizeof(line),&first,0,true)>0,"sticky wire");fprintf(f,"%s\n",line);
    fprintf(f,"AFOEND,1,600,600,1\n");fclose(f);
    /* STOP keeps first-fault evidence; read operations cannot authorize output. */
    sensor.captured_us+=10000;sensor.sequence++;
    exo_adaptive_runtime_update(&sensor,0,1,false);
    exo_adaptive_runtime_finish(0,0,false,false,100);
    exo_adaptive_runtime_trace_finish(&sensor,&last_policy,&last_tick,ESP_OK);
    ASSERT_TRUE(exo_adaptive_runtime_trace_first(0,&again)&&again.id==421,"STOP retains first fault");
    sensor.captured_us+=10000;sensor.sequence++;
    exo_adaptive_runtime_update(&sensor,77,3,false);
    exo_adaptive_runtime_finish(0,0,false,false,100);
    exo_adaptive_runtime_trace_finish(&sensor,&last_policy,&last_tick,ESP_OK);
    ASSERT_TRUE(!exo_adaptive_runtime_trace_first(0,&again),"new authorized epoch clears old first fault");
    ASSERT_TRUE(exo_adaptive_runtime_init()==ESP_OK,"new boot");
    exo_adaptive_runtime_trace_bounds(&oldest,&latest);
    ASSERT_TRUE(oldest==0&&latest==0&&!exo_adaptive_runtime_trace_first(0,&again),"boot resets diagnostic storage");
    puts("PASS USB trace: bounded cursor ring, immutable reads, sticky first fault + previous source, timing, categories, wire limits, STOP retention, boot reset");
    return 0;
}
