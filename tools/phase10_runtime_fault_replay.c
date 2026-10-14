/*
 * 编译真实exo_system.c：只用桩替代RTOS调度/CAN/控制器，验证运行保护和执行器边界。
 * 单线程回放不证明双核竞争时序，也不证明实际电机失能；台架验收仍需新REST反馈。
 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../main/exo_system.c"

static int64_t fake_us;
static TickType_t fake_ticks;
static twai_status_info_t fake_can;
static esp_err_t fake_can_result, fake_output_result;
static unsigned notifications, control_calls, can_reads, release_calls, disable_calls;
static unsigned assist_disarms, controller_disarms;
static bool disabled[2], confirm_rest, fake_partial, stop_during_output;

int64_t esp_timer_get_time(void) { return fake_us; }
TickType_t xTaskGetTickCount(void) { return fake_ticks; }
uint32_t ulTaskNotifyTake(BaseType_t clear, TickType_t ticks) {
    (void)clear; assert(ticks != portMAX_DELAY); fake_ticks += ticks;
    fake_us += (int64_t)ticks * 1000; return 0;
}
void xTaskNotifyGive(TaskHandle_t task) { assert(task != NULL); ++notifications; }
TaskHandle_t xTaskGetCurrentTaskHandle(void) { return &s_executor_task_buffer; }
TaskHandle_t xTaskCreateStaticPinnedToCore(void (*fn)(void *), const char *name,
    uint32_t size, void *arg, unsigned prio, StackType_t *stack, StaticTask_t *buf,
    BaseType_t core) {
    (void)fn; (void)name; (void)size; (void)arg; (void)prio; (void)stack; (void)core;
    return buf;
}
SemaphoreHandle_t xSemaphoreCreateMutexStatic(StaticSemaphore_t *buf) { return buf; }
BaseType_t xSemaphoreTake(SemaphoreHandle_t mutex, TickType_t ticks) {
    (void)ticks; assert(mutex != NULL); return pdTRUE;
}
BaseType_t xSemaphoreGive(SemaphoreHandle_t mutex) { assert(mutex != NULL); return pdTRUE; }
esp_err_t twai_get_status_info(twai_status_info_t *status) {
    ++can_reads; if (fake_can_result == ESP_OK) *status = fake_can; return fake_can_result;
}
bool rs01_get_feedback(exo_actuator_id_t actuator, rs01_feedback_snapshot_t *out) {
    assert(actuator < 2); memset(out, 0, sizeof(*out));
    out->valid = out->online = true;
    out->enabled_requested = !disabled[actuator];
    const bool stopped = disabled[0] && disabled[1] && confirm_rest;
    out->feedback_count = stopped ? 50 + disable_calls : 50;
    out->mode = stopped ? RS01_MODE_STATUS_RESET : RS01_MODE_STATUS_RUNNING;
    return true;
}
esp_err_t rs01_disable(exo_actuator_id_t actuator) {
    assert(actuator < 2); ++disable_calls; disabled[actuator] = true;
    return fake_can.state == TWAI_STATE_BUS_OFF ? ESP_ERR_INVALID_STATE : ESP_OK;
}
esp_err_t rs01_send_motion(exo_actuator_id_t actuator, float q, float dq,
    float tau, float kp, float kd) {
    assert(actuator < 2); assert(q == 0 && dq == 0 && tau == 0 && kp == 0 && kd == 0);
    ++release_calls; return ESP_OK;
}
esp_err_t exo_assist_disarm(void) { ++assist_disarms; return ESP_OK; }
esp_err_t exo_controller_disarm(void) { ++controller_disarms; return ESP_OK; }
void exo_assist_report_output(const exo_assist_result_t *assist, bool applied) {
    (void)assist; (void)applied;
}
esp_err_t exo_controller_process_tick(int64_t now, const exo_assist_result_t *assist,
    uint32_t sensor_seq, uint32_t gait_seq, exo_controller_tick_result_t *result) {
    (void)now; (void)assist; (void)sensor_seq; (void)gait_seq;
    ++control_calls;
    result->send_attempted = result->output_gate_open = true;
    result->send_result = fake_output_result;
    result->send_partially_succeeded = fake_partial;
    result->actuator_a_send_succeeded = fake_output_result == ESP_OK || fake_partial;
    result->actuator_b_send_succeeded = fake_output_result == ESP_OK;
    if (stop_during_output) assert(exo_system_request_stop() == EXO_REQUEST_ACCEPTED);
    return fake_output_result;
}
bool exo_control_loop_get_snapshot(exo_control_loop_snapshot_t *out) {
    memset(out, 0, sizeof(*out)); out->initialized = out->task_alive = true; return true;
}
bool hip_joint_get_snapshot(exo_hip_joint_id_t joint, hip_joint_snapshot_t *out) {
    memset(out, 0, sizeof(*out)); out->actuator = joint == EXO_HIP_JOINT_LEFT ? EXO_ACTUATOR_B : EXO_ACTUATOR_A;
    out->mapping_valid = out->direction_valid = out->zero_valid = true;
    out->feedback_valid = out->online = out->joint_valid = true; return true;
}
esp_err_t exo_assist_preview_run_contract(exo_assist_run_contract_t *out) {
    memset(out, 0, sizeof(*out)); return ESP_OK;
}
esp_err_t exo_controller_validate_run_contract(const exo_assist_run_contract_t *contract) {
    (void)contract; return ESP_OK;
}

static exo_sensor_snapshot_t reset_running(void) {
    fake_us = 1000000; fake_ticks = 0;
    notifications = control_calls = can_reads = release_calls = disable_calls = 0;
    assist_disarms = controller_disarms = 0;
    disabled[0] = disabled[1] = false; confirm_rest = true;
    fake_partial = stop_during_output = false;
    fake_can_result = fake_output_result = ESP_OK;
    fake_can = (twai_status_info_t){.state = TWAI_STATE_RUNNING};
    s_initialized = true; s_runtime_stop_pending = false;
    s_executor_task = &s_executor_task_buffer; s_operation_mutex = &s_operation_mutex_buffer;
    memset(&s_snapshot, 0, sizeof(s_snapshot));
    s_snapshot.initialized = s_snapshot.run_requested = s_snapshot.output_allowed = true;
    s_snapshot.state = EXO_SYSTEM_STATE_RUNNING;
    exo_sensor_snapshot_t sensor = {
        .sequence = 7, .captured_us = fake_us,
        .valid_sensor_mask = EXO_SENSOR_LEFT_HIP_KINEMATICS | EXO_SENSOR_RIGHT_HIP_KINEMATICS,
        .left = {.joint_valid = true, .online = true, .age_us = 1000},
        .right = {.joint_valid = true, .online = true, .age_us = 2000},
    };
    return sensor;
}
static esp_err_t tick(exo_sensor_snapshot_t *sensor, exo_controller_tick_result_t *out) {
    return exo_system_process_control_tick(fake_us, sensor, NULL, 7, 8, out);
}
static void assert_trip(exo_runtime_fault_t reason) {
    assert(s_snapshot.state == EXO_SYSTEM_STATE_STOPPING);
    assert(!s_snapshot.output_allowed && !s_snapshot.run_requested);
    assert(s_snapshot.runtime_fault_count == 1 && s_snapshot.last_runtime_fault == reason);
    assert(notifications == 1);
    exo_controller_tick_result_t out;
    const unsigned prior = control_calls;
    for (unsigned i = 0; i < 100; ++i) assert(tick(NULL, &out) == ESP_OK);
    assert(control_calls == prior && notifications == 1 && s_snapshot.runtime_fault_count == 1);
}

static void test_healthy_and_gait_not_ready(void) {
    exo_sensor_snapshot_t sensor = reset_running(); exo_controller_tick_result_t out;
    fake_can.bus_error_count = 1687; fake_can.arb_lost_count = 147;
    assert(tick(&sensor, &out) == ESP_OK); // 尚无有效助力建议不阻断基础透明/阻尼运行。
    assert(control_calls == 1 && notifications == 0 && out.send_attempted);
    fake_can.tx_error_counter = 127; assert(tick(&sensor, &out) == ESP_OK);
}
static void test_can_state_and_counters(void) {
    const twai_state_t states[] = {TWAI_STATE_BUS_OFF, TWAI_STATE_STOPPED, TWAI_STATE_RECOVERING};
    for (unsigned i = 0; i < sizeof(states)/sizeof(states[0]); ++i) {
        exo_sensor_snapshot_t sensor = reset_running(); exo_controller_tick_result_t out;
        fake_can.state = states[i]; fake_can.tx_error_counter = 128;
        assert(tick(&sensor, &out) != ESP_OK && control_calls == 0);
        assert_trip(EXO_RUNTIME_FAULT_CAN_NOT_RUNNING);
        assert(s_snapshot.runtime_can_state == (uint32_t)states[i]);
    }
    for (unsigned side = 0; side < 2; ++side) {
        exo_sensor_snapshot_t sensor = reset_running(); exo_controller_tick_result_t out;
        if (side == 0) fake_can.tx_error_counter = 128; else fake_can.rx_error_counter = 128;
        assert(tick(&sensor, &out) != ESP_OK && control_calls == 0);
        assert_trip(EXO_RUNTIME_FAULT_CAN_ERROR_PASSIVE);
    }
    exo_sensor_snapshot_t sensor = reset_running(); exo_controller_tick_result_t out;
    fake_can_result = ESP_ERR_INVALID_STATE;
    assert(tick(&sensor, &out) != ESP_OK && control_calls == 0);
    assert_trip(EXO_RUNTIME_FAULT_CAN_STATUS_UNAVAILABLE);
    assert(!s_snapshot.runtime_can_status_valid);
}
static void test_feedback_cycle_and_motor_fault(void) {
    for (unsigned side = 0; side < 2; ++side) {
        exo_sensor_snapshot_t sensor = reset_running(); exo_controller_tick_result_t out;
        sensor.valid_sensor_mask &= ~(side == 0 ? EXO_SENSOR_LEFT_HIP_KINEMATICS : EXO_SENSOR_RIGHT_HIP_KINEMATICS);
        if (side == 0) sensor.left.age_us = 50001; else sensor.right.age_us = 50001;
        assert(tick(&sensor, &out) != ESP_OK && control_calls == 0);
        assert_trip(side == 0 ? EXO_RUNTIME_FAULT_LEFT_FEEDBACK_INVALID : EXO_RUNTIME_FAULT_RIGHT_FEEDBACK_INVALID);
    }
    for (unsigned kind = 0; kind < 3; ++kind) {
        exo_sensor_snapshot_t sensor = reset_running(); exo_controller_tick_result_t out;
        if (kind == 0) --sensor.sequence; else if (kind == 1) --sensor.captured_us;
        assert(tick(kind == 2 ? NULL : &sensor, &out) != ESP_OK && control_calls == 0);
        assert_trip(EXO_RUNTIME_FAULT_SENSOR_CYCLE_INVALID);
    }
    for (unsigned side = 0; side < 2; ++side) {
        exo_sensor_snapshot_t sensor = reset_running(); exo_controller_tick_result_t out;
        if (side == 0) sensor.left.motor_fault = true; else sensor.right.motor_fault = true;
        assert(tick(&sensor, &out) != ESP_OK && control_calls == 0);
        assert_trip(side == 0 ? EXO_RUNTIME_FAULT_LEFT_MOTOR : EXO_RUNTIME_FAULT_RIGHT_MOTOR);
    }
}
static void test_real_snapshot_freshness_boundary(void) {
    for (unsigned age = 50000; age <= 50001; ++age) {
        (void)reset_running();
        const exo_sensor_snapshot_config_t config = {
            .feedback_freshness_us = 50000, .pair_skew_warning_us = 10000,
            .available_sensor_mask = EXO_SENSOR_CURRENT_HARDWARE_MASK,
        };
        exo_sensor_snapshot_builder_t builder;
        assert(exo_sensor_snapshot_builder_init(&builder, &config));
        exo_sensor_joint_snapshot_t input = {
            .source_timestamp_us = (uint64_t)fake_us - age, .source_feedback_count = 50,
            .snapshot_available = true, .mapping_valid = true, .direction_valid = true,
            .zero_valid = true, .feedback_valid = true, .online = true, .joint_valid = true,
        };
        exo_sensor_snapshot_t sensor; exo_controller_tick_result_t out;
        assert(exo_sensor_snapshot_build(&builder, fake_us, &input, &input, &sensor));
        const esp_err_t err = exo_system_process_control_tick(fake_us, &sensor, NULL,
            sensor.sequence, 8, &out);
        if (age == 50000) {
            assert(err == ESP_OK && control_calls == 1 && notifications == 0);
        } else {
            assert(err != ESP_OK && control_calls == 0);
            assert_trip(EXO_RUNTIME_FAULT_LEFT_FEEDBACK_INVALID);
            assert(s_snapshot.runtime_left_age_us == age);
        }
    }
}
static void test_partial_send_and_cleanup_latch(void) {
    exo_sensor_snapshot_t sensor = reset_running(); exo_controller_tick_result_t out;
    fake_output_result = ESP_ERR_TIMEOUT; fake_partial = true;
    assert(tick(&sensor, &out) == ESP_ERR_TIMEOUT && out.send_partially_succeeded);
    assert_trip(EXO_RUNTIME_FAULT_CONTROL_OUTPUT);
    handle_stop(); // 新REST确认成功也必须锁ERROR，而不是自动回到可START的IDLE。
    assert(s_snapshot.state == EXO_SYSTEM_STATE_ERROR && !s_snapshot.output_allowed);
    assert(disable_calls == 2 && release_calls == 2 && assist_disarms == 1 && controller_disarms == 1);
    assert(s_snapshot.stop_request_count == 0 && s_snapshot.stop_success_count == 0);
    assert(exo_system_request_start() == EXO_REQUEST_REJECTED_ERROR);
    assert(exo_system_request_stop() == EXO_REQUEST_ACCEPTED);
    handle_stop();
    assert(s_snapshot.state == EXO_SYSTEM_STATE_IDLE && !s_snapshot.output_allowed);
    assert(s_snapshot.last_runtime_fault == EXO_RUNTIME_FAULT_CONTROL_OUTPUT); // 首因作为历史保留。
}
static void test_failed_cleanup_keeps_first_cause(void) {
    exo_sensor_snapshot_t sensor = reset_running(); exo_controller_tick_result_t out;
    fake_can.state = TWAI_STATE_BUS_OFF; confirm_rest = false;
    assert(tick(&sensor, &out) != ESP_OK);
    handle_stop();
    assert(s_snapshot.state == EXO_SYSTEM_STATE_ERROR && !s_snapshot.output_allowed);
    assert(s_snapshot.last_failed_step == EXO_SYSTEM_STEP_VERIFY_DISABLED);
    assert(s_snapshot.last_runtime_fault == EXO_RUNTIME_FAULT_CAN_NOT_RUNNING);
    assert(s_snapshot.runtime_fault_us == 1000000);
    assert(disable_calls == 2 && s_snapshot.stop_failure_count == 1);
    assert(exo_system_request_start() == EXO_REQUEST_REJECTED_ERROR);
}
static void test_stop_wins_and_closed_states_no_io(void) {
    exo_sensor_snapshot_t sensor = reset_running(); exo_controller_tick_result_t out;
    stop_during_output = true; fake_output_result = ESP_ERR_TIMEOUT;
    assert(tick(&sensor, &out) != ESP_OK);
    assert(s_snapshot.state == EXO_SYSTEM_STATE_STOPPING);
    assert(s_snapshot.runtime_fault_count == 0 && notifications == 1); // 人工STOP先关闭门。
    const exo_system_state_t states[] = {EXO_SYSTEM_STATE_IDLE, EXO_SYSTEM_STATE_STARTING,
        EXO_SYSTEM_STATE_STOPPING, EXO_SYSTEM_STATE_ERROR};
    for (unsigned i = 0; i < sizeof(states)/sizeof(states[0]); ++i) {
        sensor = reset_running(); s_snapshot.state = states[i]; s_snapshot.output_allowed = false;
        assert(tick(NULL, &out) == ESP_OK && control_calls == 0 && can_reads == 0 && notifications == 0);
    }
}
int main(void) {
    /* 上报恢复不能让错误被动CAN先进入STARTING；128边界在请求入口拒绝。 */
    for (unsigned side = 0; side < 2; ++side) {
        (void)reset_running(); s_snapshot.state = EXO_SYSTEM_STATE_IDLE;
        s_snapshot.run_requested = s_snapshot.output_allowed = false;
        if (side == 0) fake_can.tx_error_counter = 128;
        else fake_can.rx_error_counter = 128;
        assert(exo_system_request_start() == EXO_REQUEST_REJECTED_NOT_READY);
        assert(s_snapshot.state == EXO_SYSTEM_STATE_IDLE && !s_snapshot.output_allowed);
        assert(disable_calls == 0 && release_calls == 0 && control_calls == 0);
    }
    puts("PASS START rejects error-passive CAN before hardware operations");
    test_healthy_and_gait_not_ready(); puts("PASS healthy inputs / gait waiting / historical CAN counters");
    test_can_state_and_counters(); puts("PASS CAN states / error passive boundary / unreadable status");
    test_feedback_cycle_and_motor_fault(); puts("PASS left/right stale / cycle mismatch / motor fault");
    test_real_snapshot_freshness_boundary(); puts("PASS real sensor builder 50000/50001 us boundary");
    test_partial_send_and_cleanup_latch(); puts("PASS partial send / auto cleanup ERROR latch / explicit STOP");
    test_failed_cleanup_keeps_first_cause(); puts("PASS failed REST verification retains original CAN fault");
    test_stop_wins_and_closed_states_no_io(); puts("PASS STOP wins / closed states emit no periodic IO");
    puts("Phase10 runtime fault replay PASS: 8/8"); return 0;
}
