/* Diagnostic-only validation: real C decisions and 100Hz runtime queue, no devices. */
#define EXO_ADAPTIVE_ACTIVE_MAIN active_existing_main
#include "adaptive_active_replay.c"

static exo_adaptive_event_t event_case(unsigned intervals,float age,float frequency,float phase_error) {
    exo_adaptive_config_t config=exo_adaptive_default_config();
    exo_adaptive_event_t event={.initialized=true,.filtered_velocity=-10,.trend=-1,
        .period_s=1,.intervals=intervals,.last_event_us=1000000,.previous_event_us=1000000,
        .target_offset_rad=phase_error,.offset_rad=0};
    exo_adaptive_event_update(&event,&config.afo,0,-10,
        exo_adaptive_wrap(2*EXO_ADAPTIVE_PI*age),2*EXO_ADAPTIVE_PI*frequency,.01f,
        1000000+(int64_t)(age*1000000));
    return event;
}
static bool reasons_are_exact(void) {
    exo_adaptive_event_t e=event_case(2,.2f,1,0);
    ASSERT_TRUE(e.locked&&e.lock_reasons==0,"healthy event decisions");
    e=event_case(1,.2f,1,0);
    ASSERT_TRUE(!e.locked&&e.lock_reasons==EXO_ADAPTIVE_LOCK_INTERVALS,"not enough intervals");
    e=event_case(2,1.7f,1,0);
    ASSERT_TRUE(!e.locked&&e.lock_reasons==EXO_ADAPTIVE_LOCK_EVENT_AGE&&e.lock_intervals==2&&e.intervals==0,"retain pre-reset decision count");
    e=event_case(2,.2f,1.3f,0);
    ASSERT_TRUE(!e.locked&&e.lock_reasons==EXO_ADAPTIVE_LOCK_FREQUENCY,"frequency mismatch");
    e=event_case(2,.2f,1,.6f);
    ASSERT_TRUE(!e.locked&&e.lock_reasons==EXO_ADAPTIVE_LOCK_PHASE_ERROR,"exact phase boundary");
    ASSERT_TRUE(e.lock_phase_error_rad>=.6f&&fabsf(e.lock_phase_error_rad-.6f)<1e-6f&&
        e.target_offset_rad-e.offset_rad<.6f,"use phase error before smooth correction (float wrap tolerance)");
    e=event_case(1,1.7f,1.3f,.7f);
    ASSERT_TRUE(e.lock_reasons==(EXO_ADAPTIVE_LOCK_INTERVALS|EXO_ADAPTIVE_LOCK_EVENT_AGE|
        EXO_ADAPTIVE_LOCK_FREQUENCY|EXO_ADAPTIVE_LOCK_PHASE_ERROR),"simultaneous reasons retained");
    return true;
}
static bool reference_validation(void) {
    exo_adaptive_config_t c=exo_adaptive_default_config();
    exo_adaptive_event_t e={.initialized=true,.filtered_velocity=-10,.trend=1,
        .low_deg=0,.peak_deg=10,.peak_us=2000000,.period_s=1,.intervals=2,
        .previous_event_us=1000000,.last_event_us=1000000};
    const float omega=2*EXO_ADAPTIVE_PI;
    exo_adaptive_event_update(&e,&c.afo,0,-10,1.02f,omega,.01f,2010000);
    ASSERT_TRUE(e.event&&!e.locked,"bad measured reference");
    for(unsigned n=1;n<=90;n++) {
        exo_adaptive_event_update(&e,&c.afo,0,-10,
            exo_adaptive_wrap(1.02f+omega*.01f*n),omega,.01f,2010000+(int64_t)n*10000);
        ASSERT_TRUE(!e.event&&!e.locked&&(e.lock_reasons&EXO_ADAPTIVE_LOCK_PHASE_ERROR),
            "offset self-convergence cannot validate a rejected reference");
    }
    ASSERT_TRUE(fabsf(exo_adaptive_circular_error(e.target_offset_rad-e.offset_rad))<.6f,
        "internal correction converged but reference remains rejected");
    e.trend=1;e.peak_deg=10;e.peak_us=3000000;e.low_deg=0;e.filtered_velocity=-10;
    exo_adaptive_event_update(&e,&c.afo,0,-10,
        exo_adaptive_wrap(omega*.01f-e.offset_rad),omega,.01f,3010000);
    ASSERT_TRUE(e.event&&e.locked&&e.lock_reasons==0,"new matching peak validates reference");
    e=(exo_adaptive_event_t){.initialized=true,.filtered_velocity=-10,.trend=-1,
        .period_s=1,.intervals=2,.last_event_us=1000000,.previous_event_us=1000000};
    exo_adaptive_event_update(&e,&c.afo,0,-10,omega*.2f+.8f,omega,.01f,1200000);
    ASSERT_TRUE(!e.locked&&e.lock_reasons==EXO_ADAPTIVE_LOCK_PHASE_ERROR&&
        fabsf(e.lock_phase_error_rad-.8f)<1e-5f,"between-peak drift caught despite zero offset error");
    exo_adaptive_engine_t engine;exo_adaptive_output_t out;
    ASSERT_TRUE(exo_adaptive_engine_init(&engine,&c),"capture seed fixture");
    bool seeded=false;
    for(unsigned n=0;n<330;n++) {
        float t=n*.01f,phase=omega*t/1.4f;
        exo_adaptive_input_t in={.timestamp_us=10000+(int64_t)n*10000,.sequence=n+1,
            .coordinate_epoch=1,.valid={true,true},.position_deg={0,0},.velocity_deg_s={0,0}};
        in.position_deg[0]=in.position_deg[1]=12.5f*(1+cosf(phase));
        in.velocity_deg_s[0]=in.velocity_deg_s[1]=-12.5f*omega/1.4f*sinf(phase);
        bool before=engine.acquisition_seeded[0];
        ASSERT_TRUE(exo_adaptive_engine_update(&engine,&in,&out),"capture sample");
        if(!before&&engine.acquisition_seeded[0]) {
            ASSERT_TRUE(!out.leg[0].locked&&out.leg[0].limited_nm==0&&
                out.leg[0].lock_intervals==1,"initialization occurs while unlocked/zero");
            ASSERT_TRUE(fabsf(engine.afo[0].omega_rad_s-omega/1.4f)<.001f&&
                fabsf(engine.afo[0].cosine[0]-.625f)<.001f&&
                fabsf(engine.afo[0].cosine[1]-.625f)<.001f,
                "measured period and excursion initialize frequency/DC/amplitude");
            seeded=true;
        }
    }
    ASSERT_TRUE(seeded,"initialization was exercised");
    puts("PASS phase capture: measured-period seed only while zero/unlocked; rejected reference latch; between-peak drift");
    return true;
}
static bool wait_for_pair_lock(void) {
    for(unsigned n=0;n<5000;n++) {
        ASSERT_TRUE(step(0,true),"healthy 100Hz input");
        exo_adaptive_log_t log;
        ASSERT_TRUE(exo_adaptive_runtime_snapshot(&log),"runtime snapshot");
        for(unsigned side=0;side<2;side++)
            ASSERT_TRUE(!log.output.leg[side].locked||log.output.leg[side].lock_reasons==0,"locked cannot carry rejection reasons");
        if(log.output.leg[0].locked&&log.output.leg[1].locked)return true;
    }
    ASSERT_TRUE(false,"pair did not lock");
}
static bool runtime_loss_records(void) {
    ASSERT_TRUE(begin_case(true,EXO_ASSIST_EXEC_ACTIVE)&&warm_nonzero(),"nonzero fixture");
    exo_adaptive_lock_record_t r;
    while(exo_adaptive_runtime_pop_lock_loss(&r)) {}
    ASSERT_TRUE(step(1,true),"invalid input on an exact 10ms cycle");
    unsigned count=0;
    while(exo_adaptive_runtime_pop_lock_loss(&r)) {
        ASSERT_TRUE(r.timestamp_us==last_sensors.captured_us&&r.sequence==last_sensors.sequence,"loss is same cycle, not 10Hz time");
        ASSERT_TRUE(r.side==count&&r.output.lock_reasons&EXO_ADAPTIVE_LOCK_INPUT,"both side reasons");
        ASSERT_TRUE(!r.output.locked&&r.output.rearm_required&&r.command_torque_nm==0,"record final zero and rearm");
        count++;
    }
    ASSERT_TRUE(count==2,"two exact losses");
    for(unsigned n=0;n<10;n++)ASSERT_TRUE(step(1,true),"remaining unlocked");
    ASSERT_TRUE(!exo_adaptive_runtime_pop_lock_loss(&r),"no duplicate events while unlocked");
    ASSERT_TRUE(begin_case(true,EXO_ASSIST_EXEC_ACTIVE),"fresh ring");
    for(unsigned n=0;n<11;n++) {
        ASSERT_TRUE(wait_for_pair_lock()&&step(1,true),"repeated lock-loss cycles");
    }
    count=0;uint32_t dropped=0;int64_t previous_time=0;
    while(exo_adaptive_runtime_pop_lock_loss(&r)) {
        ASSERT_TRUE(r.timestamp_us>=previous_time,"chronological queue");
        previous_time=r.timestamp_us;dropped=r.dropped;count++;
    }
    ASSERT_TRUE(count==EXO_ADAPTIVE_LOCK_LOG_CAPACITY&&dropped>=6,"bounded ring with visible drops");
    ASSERT_TRUE(exo_adaptive_runtime_init()==ESP_OK&&!exo_adaptive_runtime_pop_lock_loss(&r),"reset clears old records");
    printf("PASS lock diagnostics: exact reasons/pre-correction boundary; same-cycle final zero; no duplicates; ring16 overflow/reset\n");
    return true;
}
int main(void) {
    return reasons_are_exact()&&reference_validation()&&runtime_loss_records()?0:1;
}
