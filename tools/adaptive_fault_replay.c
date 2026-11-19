/* Host-only fault injection. Reuse the existing synthetic fixture, never hardware. */
#define main adaptive_existing_main
#include "adaptive_replay.c"
#undef main

enum fault_case {
    INVALID_FEEDBACK, NAN_POSITION, FUTURE_SOURCE, BACKWARD_TIME, LONG_TICK,
    COORDINATE_CHANGE, VELOCITY_WINDOW, ANGLE_WINDOW, EMERGENCY, PERMIT_REMOVED
};

static unsigned establish_nonzero(exo_adaptive_engine_t *engine,
                                  exo_adaptive_output_t *output) {
    exo_adaptive_config_t config=demo();
    CHECK(exo_adaptive_engine_init(engine,&config));
    for(unsigned n=0;n<6000;n++) {
        exo_adaptive_input_t input=sample(n,1,1);
        CHECK(exo_adaptive_engine_update(engine,&input,output));
        if(n>3000 && output->leg[0].locked && output->leg[1].locked &&
           fabsf(output->leg[0].limited_nm)>.03f &&
           fabsf(input.velocity_deg_s[0])>15 && input.position_deg[0]>0) return n;
    }
    CHECK(false);return 0;
}

static void immediate_faults(void) {
    static const char *names[]={"invalid_feedback","nan_position","future_source",
        "backward_time","long_tick","coordinate_change","velocity_window",
        "angle_window","emergency","permit_removed"};
    static const uint32_t expected[]={EXO_ADAPTIVE_FAULT_INPUT,EXO_ADAPTIVE_FAULT_INPUT,
        EXO_ADAPTIVE_FAULT_INPUT,EXO_ADAPTIVE_FAULT_TIME,EXO_ADAPTIVE_FAULT_TIME,
        EXO_ADAPTIVE_FAULT_INPUT,EXO_ADAPTIVE_FAULT_RANGE,EXO_ADAPTIVE_FAULT_RANGE,
        EXO_ADAPTIVE_FAULT_STOP,EXO_ADAPTIVE_FAULT_STOP};
    exo_adaptive_engine_t ready;exo_adaptive_output_t output;
    unsigned last=establish_nonzero(&ready,&output);
    const float before=output.leg[0].limited_nm;
    for(unsigned which=0;which<sizeof(names)/sizeof(names[0]);which++) {
        exo_adaptive_engine_t engine=ready;
        exo_adaptive_config_t config=demo();
        exo_adaptive_input_t input=sample(last+1,1,1);
        switch((enum fault_case)which) {
        case INVALID_FEEDBACK: input.valid[0]=false;break;
        case NAN_POSITION: input.position_deg[0]=NAN;break;
        case FUTURE_SOURCE: input.source_timestamp_us[0]=(uint64_t)input.timestamp_us+1;break;
        case BACKWARD_TIME: input.timestamp_us=ready.timestamp_us-1;break;
        case LONG_TICK: input.timestamp_us+=30000;break;
        case COORDINATE_CHANGE: input.coordinate_epoch++;break;
        case VELOCITY_WINDOW:
            config.leg[0].maximum_velocity_deg_s=1;
            CHECK(exo_adaptive_engine_configure(&engine,&config,2));break;
        case ANGLE_WINDOW:
            config.leg[0].maximum_angle_deg=input.position_deg[0]-.5f;
            CHECK(exo_adaptive_engine_configure(&engine,&config,2));break;
        case EMERGENCY: input.emergency=true;break;
        case PERMIT_REMOVED: input.permit_epoch=0;break;
        }
        CHECK(exo_adaptive_engine_update(&engine,&input,&output));
        CHECK(output.leg[0].faults & expected[which]);
        CHECK(output.leg[0].limited_nm==0);
        if(which!=PERMIT_REMOVED) CHECK(output.leg[0].rearm_required);
        config=demo();CHECK(exo_adaptive_engine_configure(&engine,&config,3));
        unsigned relocked=0;
        for(unsigned n=last+2;n<last+2002;n++) {
            input=sample(n,1,1);
            if(which==PERMIT_REMOVED) input.permit_epoch=0;
            CHECK(exo_adaptive_engine_update(&engine,&input,&output));
            CHECK(output.leg[0].limited_nm==0);
            if(which!=PERMIT_REMOVED) CHECK(output.leg[0].rearm_required);
            relocked+=output.leg[0].locked;
        }
        CHECK(relocked>0); /* Zero is preserved even after valid motion re-locks. */
        unsigned restarted_nonzero=0;
        for(unsigned n=last+2002;n<last+2302;n++) {
            input=sample(n,1,1);input.permit_epoch=2;
            CHECK(exo_adaptive_engine_update(&engine,&input,&output));
            CHECK(!output.leg[0].rearm_required);
            restarted_nonzero+=fabsf(output.leg[0].limited_nm)>1e-4f;
        }
        CHECK(restarted_nonzero>0);
        printf("PASS immediate_fault=%s before_nm=%.6f fault_cycle_nm=0 old_permit_relocked=%u old_permit_nonzero=0 new_permit_nonzero=%u\n",
               names[which],before,relocked,restarted_nonzero);
    }
}

static void stop_motion(void) {
    exo_adaptive_engine_t engine;exo_adaptive_output_t output;
    unsigned last=establish_nonzero(&engine,&output),unlock_tick=0;
    exo_adaptive_input_t still=sample(last,1,1);
    for(unsigned tick=1;tick<=35;tick++) {
        exo_adaptive_input_t input=sample(last+tick,1,1);
        for(unsigned side=0;side<2;side++) {
            input.position_deg[side]=still.position_deg[side];
            input.velocity_deg_s[side]=0;
        }
        CHECK(exo_adaptive_engine_update(&engine,&input,&output));
        if(!output.leg[0].locked) {
            if(!unlock_tick) unlock_tick=tick;
            CHECK(output.leg[0].limited_nm==0);
            CHECK(output.leg[0].rearm_required);
        }
    }
    CHECK(unlock_tick>0 && unlock_tick<=31);
    for(unsigned n=last+36;n<last+2036;n++) {
        exo_adaptive_input_t input=sample(n,1,1);
        CHECK(exo_adaptive_engine_update(&engine,&input,&output));
        CHECK(output.leg[0].limited_nm==0 && output.leg[0].rearm_required);
    }
    CHECK(output.leg[0].locked);
    printf("PASS stop_motion first_unlock_ms=%u zero_on_first_unlocked_tick=YES relocked_without_resume=YES\n",unlock_tick*10);
}

int main(void) {
    immediate_faults();stop_motion();
    printf("PASS host-only fault matrix: 11 scenarios, %u assertions; no serial/BLE/actuator access\n",checks);
    return 0;
}
