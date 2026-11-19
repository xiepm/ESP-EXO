/* Complete product computation path; RTOS/NVS/CAN transport are host fakes.
 * Input is SYNTHETIC 100 Hz interpolation, not original motor feedback. */
#define EXO_ADAPTIVE_MODE_MAIN recorded_unused_regression_main
#include "adaptive_mode_replay.c"
#include <stdlib.h>
int main(int argc,char **argv) {
    if(argc!=3&&argc!=4)return 2;
    const char *scenario=argc==4?argv[3]:"fresh";
    bool asynchronous=strcmp(scenario,"async")==0;
    bool quantized=strcmp(scenario,"quantized")==0;
    if(!asynchronous&&!quantized&&strcmp(scenario,"fresh")!=0)return 2;
    FILE *input=fopen(argv[1],"r"),*trace=fopen(argv[2],"w");
    if(!input||!trace)return 2;
    char line[1024];if(!fgets(line,sizeof(line),input))return 3;
    fprintf(trace,"segment,time_us,side,q,v,torque,locked,rearm,faults,lambda,policy_torque,send_succeeded,age_us,pair_skew_us,valid_mask,lock_reasons,intervals,event_age_s,phase,desired_nm\n");
    bool initialized=false;unsigned previous=0,n=0;
    exo_sensor_snapshot_builder_t builder;
    exo_sensor_joint_snapshot_t raw_left={0},raw_right={0};
    while(fgets(line,sizeof(line),input)) {
        unsigned segment;long long us;float ql,vl,qr,vr;
        if(sscanf(line,"%u,%lld,%f,%f,%f,%f",&segment,&us,&ql,&vl,&qr,&vr)!=6)return 3;
        if(!initialized||segment!=previous) {
            if(!mode_begin(EXO_ASSIST_EXEC_ACTIVE,15000,100,100))return 4;
            if(!exo_sensor_snapshot_builder_init(&builder,&k_sensor_config))return 4;
            previous=segment;initialized=true;n=0;
        }
        ++n;mock_clock=us;
        if(quantized) {
            /* Sensitivity scenario: exported 1 mrad angular quantization,
             * not a claim about the underlying motor encoder resolution. */
            const float step=180.0f/EXO_ADAPTIVE_PI*.001f;
            ql=roundf(ql/step)*step;qr=roundf(qr/step)*step;
        }
        if(n==1||!asynchronous||n%2==0)raw_left=make_joint(ql,vl,(uint64_t)us,n);
        if(n==1||!asynchronous||n%3==0)raw_right=make_joint(qr,vr,(uint64_t)us,n);
        exo_sensor_snapshot_t sensor;
        if(!exo_sensor_snapshot_build(&builder,us,&raw_left,&raw_right,&sensor))return 5;
        if(!exo_assist_get_snapshot(&current_assist))return 5;
        exo_adaptive_runtime_update(&sensor,exo_adaptive_runtime_permit(&current_assist,true),3,false);
        exo_gait_estimate_t gait={.sequence=n};
        if(exo_assist_process_tick(&sensor,&gait,&last_policy)!=ESP_OK)return 5;
        if(exo_controller_process_tick(us,&last_policy,n,n,&last_tick)!=ESP_OK)return 6;
        exo_adaptive_runtime_finish(last_tick.final_command.left.torque_ff_nm,last_tick.final_command.right.torque_ff_nm,
            last_tick.actuator_b_send_succeeded,last_tick.actuator_a_send_succeeded,100);
        exo_adaptive_runtime_trace_finish(&sensor,&last_policy,&last_tick,ESP_OK);
        exo_assist_report_output(&last_policy,last_tick.assist_applied);
        exo_adaptive_log_t log;if(!exo_adaptive_runtime_snapshot(&log))return 7;
        for(unsigned side=0;side<2;side++) {
            const exo_adaptive_leg_output_t *o=&log.output.leg[side];
            float sent=side?s_last_sent.right.torque_ff_nm:s_last_sent.left.torque_ff_nm;
            float policy=side?last_policy.suggestion.right.torque_ff_nm:last_policy.suggestion.left.torque_ff_nm;
            const exo_sensor_joint_snapshot_t *joint=side?&sensor.right:&sensor.left;
            float q=joint->joint_sample.position_deg,v=joint->joint_sample.velocity_deg_s;
            if(!isfinite(sent)||fabsf(sent)>15.0001f||sent*v < -1e-5f)return 8;
            if((q < -40||q>90||fabsf(v)>400||o->faults)&&sent!=0)return 8;
            fprintf(trace,"%u,%lld,%u,%.9g,%.9g,%.9g,%u,%u,%u,%.9g,%.9g,%u,%lu,%lu,%lu,%lu,%lu,%.9g,%.9g,%.9g\n",segment,us,side,
                (double)q,(double)v,(double)sent,o->locked,o->rearm_required,o->faults,(double)o->lambda,
                (double)policy,last_tick.send_succeeded,(unsigned long)joint->age_us,
                (unsigned long)sensor.pair_skew_us,(unsigned long)sensor.valid_sensor_mask,
                (unsigned long)o->lock_reasons,(unsigned long)o->lock_intervals,
                (double)o->lock_event_age_s,(double)o->corrected_phase,(double)o->desired_nm);
        }
    }
    for(unsigned side=0;side<2;side++) {
        exo_adaptive_output_counts_t counts;
        if(!exo_adaptive_runtime_output_counts(side,&counts))return 9;
        unsigned sum=0;for(unsigned k=1;k<EXO_AFO_COUNT_SIZE;k++)sum+=counts.ticks[k];
        if(counts.ticks[EXO_AFO_COUNT_TOTAL]!=n||sum!=n)return 9;
    }
    fclose(input);fclose(trace);return 0;
}
