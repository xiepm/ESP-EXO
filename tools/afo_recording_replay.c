/* Offline bridge to the production algorithm. No ESP/CAN/BLE dependencies. */
#include "exo_adaptive.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

int main(int argc, char **argv) {
    if (argc != 7) return 2;
    float peak = strtof(argv[4], NULL);
    float levels[2] = {strtof(argv[5], NULL), strtof(argv[6], NULL)};
    if (!isfinite(peak) || peak < 0 || peak > 15) return 2;
    for (unsigned i = 0; i < 2; ++i)
        if (!isfinite(levels[i]) || levels[i] < 0 || levels[i] > 100) return 2;
    FILE *input = fopen(argv[1], "r"), *trace = fopen(argv[2], "w");
    FILE *meta = fopen(argv[3], "w");
    if (!input || !trace || !meta) return 2;
    exo_adaptive_config_t config = exo_adaptive_walking_preset_for_peak(peak);
    fprintf(meta, "{\"minimum_angle_deg\":%.9g,\"maximum_angle_deg\":%.9g,"
        "\"maximum_velocity_deg_s\":%.9g,\"power_w\":%.9g,\"fade_s\":%.9g,"
        "\"control_maximum_dt_s\":%.9g,\"requested_peak_nm\":%.9g}\n",
        (double)config.leg[0].minimum_angle_deg, (double)config.leg[0].maximum_angle_deg,
        (double)config.leg[0].maximum_velocity_deg_s, (double)config.leg[0].positive_power_w,
        (double)config.leg[0].fade_s, (double)config.afo.maximum_dt_s, (double)peak);
    fclose(meta);
    char line[1024];
    if (!fgets(line, sizeof(line), input)) return 3;
    fprintf(trace, "segment,time_us,side,q,v,locked,rearm,faults,reasons,events,intervals,lambda,torque,phase,raw_torque\n");
    exo_adaptive_engine_t engine;
    exo_adaptive_output_t out;
    unsigned previous = 0, sequence = 0;
    bool initialized = false;
    while (fgets(line, sizeof(line), input)) {
        unsigned segment; long long us; float ql, vl, qr, vr;
        if (sscanf(line, "%u,%lld,%f,%f,%f,%f", &segment, &us, &ql, &vl, &qr, &vr) != 6) return 3;
        if (!initialized || segment != previous) {
            if (!exo_adaptive_engine_init(&engine, &config)) return 4;
            engine.walking_mode = true;
            previous = segment; sequence = 0; initialized = true;
        }
        exo_adaptive_input_t in = {.timestamp_us=us, .sequence=++sequence,
            .coordinate_epoch=1, .permit_epoch=1, .valid={true,true},
            .position_deg={ql,qr}, .velocity_deg_s={vl,vr}};
        if (!exo_adaptive_engine_update(&engine, &in, &out)) return 5;
        for (unsigned i = 0; i < 2; ++i) {
            const exo_adaptive_leg_output_t *o = &out.leg[i];
            /* Product adaptive_step applies the user level after the engine. */
            float torque = fmaxf(-peak, fminf(peak, o->limited_nm)) * levels[i] / 100;
            if (!isfinite(torque) || fabsf(torque) > peak + .0001f) return 6;
            if (o->faults && torque != 0) return 7;
            fprintf(trace, "%u,%lld,%u,%.9g,%.9g,%u,%u,%u,%u,%u,%u,%.9g,%.9g,%.9g,%.9g\n",
                segment, us, i, (double)in.position_deg[i], (double)in.velocity_deg_s[i],
                o->locked, o->rearm_required, o->faults, o->lock_reasons,
                o->accepted_events, o->lock_intervals, (double)o->lambda,
                (double)torque, (double)o->corrected_phase, (double)o->limited_nm);
        }
    }
    if (ferror(input) || ferror(trace)) return 8;
    fclose(input); fclose(trace); return 0;
}
