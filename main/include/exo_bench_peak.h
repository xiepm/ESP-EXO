#ifndef EXO_BENCH_PEAK_H
#define EXO_BENCH_PEAK_H
#include "exo_sensor_snapshot.h"

/* Secured, mechanically loaded bench only. No actuator I/O in this module.
 * One side, zero gains, 2 Nm/s below 1 Nm, 20 Nm/s above it, symmetric
 * rise/fall, 100 ms plateau, then permanent zero (15 Nm: at most 3 seconds).
 * The timer is owned by the MCU, never by the host serial connection. */
#define EXO_BENCH_PEAK_PROFILE_VERSION 2U
typedef enum {
    EXO_BENCH_PEAK_OK=0, EXO_BENCH_PEAK_TIME, EXO_BENCH_PEAK_FEEDBACK,
    EXO_BENCH_PEAK_MOVED, EXO_BENCH_PEAK_SPEED, EXO_BENCH_PEAK_TEMPERATURE,
    EXO_BENCH_PEAK_ESTIMATE, EXO_BENCH_PEAK_COORDINATES
} exo_bench_peak_fault_t;
typedef struct {
    bool initialized, complete;
    unsigned side;
    float requested_nm, centers_deg[2];
    int64_t start_us, last_us;
    uint32_t last_sequence, coordinate_epoch, samples, plateau_samples;
    uint64_t last_source_us[2];
    exo_bench_peak_fault_t fault;
    float command_peak_nm, sent_peak_nm, estimate_peak_nm, plateau_sum_nm;
    float maximum_displacement_deg, maximum_speed_deg_s, maximum_temperature_c;
    float maximum_displacement_by_side[2], maximum_speed_by_side[2];
} exo_bench_peak_t;
bool exo_bench_peak_init(exo_bench_peak_t *state,unsigned side,float signed_peak_nm,
    float left_center_deg,float right_center_deg,int64_t start_us);
bool exo_bench_peak_step(exo_bench_peak_t *state,const exo_sensor_snapshot_t *input,
    int64_t now_us,uint32_t expected_sequence,float *signed_torque_nm);
#endif
