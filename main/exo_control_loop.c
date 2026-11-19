/*
 * 文件导读：本文件只创建一个永久周期任务，为整个产品提供唯一 100 Hz 时间基准。
 * 控制任务每次用 FreeRTOS 的绝对节拍延时 API 按计划时刻唤醒，读取左右髋关节
 * 快照，通过
 * exo_system_process_control_tick()
 * 的状态输出门决定是否执行控制，最后发布诊断。
 *
 * FreeRTOS 初学者需要区分：任务是可被调度的长期执行单元；临界区只用于保护很短
 * 的内存复制，期间不能阻塞；StaticTask_t 和静态栈表示创建任务时不使用运行期堆。
 * s_published_snapshot 只是给 UART/BLE 看的诊断副本，不参与下一周期的控制计算。
 */

#include "exo_control_loop.h"

#include <limits.h>
#include <math.h>
#include <string.h>

#include "esp_timer.h"
#include "exo_assist.h"
#include "exo_adaptive_runtime.h"
#include "exo_config.h"
#include "exo_sensor_snapshot.h"
#include "exo_system.h"
#include "exo_telemetry.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gait_estimator.h"
#include "hip_joint.h"

#define EXO_CONTROL_TASK_STACK_BYTES 6144U
#define EXO_CONTROL_TASK_PRIORITY 9U
#define EXO_CONTROL_TASK_CORE 1
#define EXO_CONTROL_PERIOD_TOLERANCE_US 1000U

/* 防止两个调用者同时初始化；INITIALIZING 表示第一个调用者正在创建静态任务。 */
typedef enum {
	CONTROL_LOOP_NOT_INITIALIZED = 0,
	CONTROL_LOOP_INITIALIZING,
	CONTROL_LOOP_INITIALIZED,
} control_loop_init_state_t;

/* 仅由控制任务持有的累计量，周期结束时才整理成可公开的完整快照。 */
typedef struct {
	exo_control_loop_snapshot_t snapshot;
	uint64_t period_total_us;
	uint64_t period_sample_count;
	int64_t previous_cycle_start_us;
} control_loop_diagnostics_t;

/*
 * 任务控制块和栈均为静态存储。s_diagnostics_lock
 * 只保护初始化状态及完整快照复制， 不包围关节读取、控制计算或 CAN
 * 发送，因此不会把 100 Hz 工作放进长临界区。
 */
static StaticTask_t s_control_task_buffer;
static StackType_t
	s_control_task_stack[EXO_CONTROL_TASK_STACK_BYTES / sizeof(StackType_t)];
static control_loop_init_state_t s_init_state;
/*
 * 控制任务把完整快照先复制到非活动缓冲区，最后只在临界区切换1-byte索引。
 * 这样BLE/UART读取大结构时不会与控制任务在同一块内存上互相撕裂。
 */
static exo_control_loop_snapshot_t s_published_snapshots[2];
static uint8_t s_active_snapshot_index;
static portMUX_TYPE s_diagnostics_lock = portMUX_INITIALIZER_UNLOCKED;
/* 下面两个上下文只由永久控制任务读写，静态分配且不需要运行期互斥。 */
static exo_sensor_snapshot_builder_t s_sensor_builder;
static exo_gait_estimator_t s_gait_estimator;

/* 诊断字段宽度有限；极端或异常时间值采用饱和表示，避免转换后回绕。 */
static uint32_t clamp_duration_us(int64_t duration_us) {
	if (duration_us <= 0) {
		return 0U;
	}
	if ((uint64_t)duration_us > UINT32_MAX) {
		return UINT32_MAX;
	}
	return (uint32_t)duration_us;
}

/* 周期误差保留正负号，并在超出 int32_t 诊断范围时饱和。 */
static int32_t clamp_period_error_us(int64_t error_us) {
	if (error_us > INT32_MAX) {
		return INT32_MAX;
	}
	if (error_us < INT32_MIN) {
		return INT32_MIN;
	}
	return (int32_t)error_us;
}

/* 从完整关节快照中只提取诊断所需的来源计数、时间戳和最终有效位。 */
static exo_control_loop_joint_status_t
joint_status_from_snapshot(const hip_joint_snapshot_t *snapshot,
						   bool snapshot_available) {
	exo_control_loop_joint_status_t status = {0};
	if (snapshot_available) {
		status.feedback_count = snapshot->source_feedback_count;
		status.feedback_timestamp_us = snapshot->feedback_timestamp_us;
		status.joint_valid = snapshot->joint_valid;
	}
	return status;
}

/*
 * 把阶段3髋关节对象复制成阶段9通用传感输入。这里不重新读取RS01：调用者传入的
 * hip快照正是本控制周期在同一位置取得的那一份。不存在的快照用available=false
 * 表示，后续算法不会把全零结构误当成真实0度。
 */
static exo_sensor_joint_snapshot_t
sensor_joint_from_hip(const hip_joint_snapshot_t *snapshot,
					 bool snapshot_available) {
	exo_sensor_joint_snapshot_t sensor = {0};
	sensor.age_us = UINT32_MAX;
	sensor.snapshot_available = snapshot_available;
	if (!snapshot_available || snapshot == NULL) {
		return sensor;
	}
	sensor.joint_sample = snapshot->joint_sample;
	sensor.source_timestamp_us = snapshot->feedback_timestamp_us;
	sensor.source_feedback_count = snapshot->source_feedback_count;
	sensor.position_epoch = snapshot->actuator_position_epoch;
	sensor.zero_source_timestamp_us = snapshot->zero_source_timestamp_us;
	sensor.zero_source_feedback_count = snapshot->zero_source_feedback_count;
	sensor.zero_source_position_epoch = snapshot->zero_source_position_epoch;
	sensor.mapping_valid = snapshot->mapping_valid;
	sensor.direction_valid = snapshot->direction_valid;
	sensor.zero_valid = snapshot->zero_valid && snapshot->zero_epoch_matches;
	sensor.feedback_valid = snapshot->feedback_valid &&
						 snapshot->continuous_position_valid;
	sensor.online = snapshot->online;
	sensor.motor_fault = snapshot->motor_fault;
	sensor.joint_valid = snapshot->joint_valid;
	return sensor;
}

/*
 * 把“本次 100 Hz 周期”已经算好的结果整理成阶段 10 遥测输入。
 *
 * 这里不重新读取关节、步态或控制器全局快照，避免把不同周期的数据拼到一起：
 * - 位置/速度/电机估算力矩来自本周期 sensor_snapshot；
 * - 相位和置信度来自由该快照立即更新出的 gait_estimate；
 * - 建议助力来自同一周期 assist_result；
 * - 实际助力只有在最终命令发送成功时才被标为有效。
 *
 * exo_telemetry_push() 内部只做有界定点量化和静态环形队列写入，不发送 BLE，
 * 因此可以安全地留在唯一控制任务中。真正的 Notify 由低优先级 BLE 任务完成。
 */
static void push_phase10_telemetry(
	int64_t cycle_start_us, const exo_sensor_snapshot_t *sensor_snapshot,
	const exo_gait_estimate_t *gait_estimate,
	const exo_assist_result_t *assist_result,
	const exo_controller_tick_result_t *control_result) {
	if (sensor_snapshot == NULL || gait_estimate == NULL ||
		assist_result == NULL || control_result == NULL) {
		return;
	}

	exo_system_control_state_t system_state = {0};
	(void)exo_system_get_control_state(&system_state);

	const bool left_kinematics_valid =
		(sensor_snapshot->valid_sensor_mask &
		 EXO_SENSOR_LEFT_HIP_KINEMATICS) != 0U;
	const bool right_kinematics_valid =
		(sensor_snapshot->valid_sensor_mask &
		 EXO_SENSOR_RIGHT_HIP_KINEMATICS) != 0U;
	const bool left_motor_torque_valid =
		(sensor_snapshot->valid_sensor_mask &
		 EXO_SENSOR_LEFT_MOTOR_TORQUE_ESTIMATE) != 0U;
	const bool right_motor_torque_valid =
		(sensor_snapshot->valid_sensor_mask &
		 EXO_SENSOR_RIGHT_MOTOR_TORQUE_ESTIMATE) != 0U;
	const bool left_phase_valid =
		(gait_estimate->valid_flags & EXO_GAIT_VALID_LEFT_PHASE) != 0U;
	const bool right_phase_valid =
		(gait_estimate->valid_flags & EXO_GAIT_VALID_RIGHT_PHASE) != 0U;
	const bool confidence_valid =
		(gait_estimate->valid_flags & EXO_GAIT_VALID_CONFIDENCE) != 0U;
	const bool torque_suggestion_valid =
		assist_result->valid &&
		assist_result->output_kind == EXO_ASSIST_OUTPUT_TORQUE_FF_DELTA;
	/*
	 * SHADOW 下最终基础命令发送成功时，实际助力是“已知的 0 N·m”，所以仍标
	 * 有效；只有发送失败/输出门关闭时才不能声称某个值已实际施加。
	 */
	/*
	 * 一对 Motion 命令是两帧 CAN，不具备总线原子性。若其中一帧失败，已经成功
	 * 入队的另一侧仍可能产生真实输出，所以遥测必须按人体 LEFT/RIGHT 分别标记，
	 * 不能用合并返回值把物理上的单侧输出抹成两个无效值。
	 */
	const bool left_send_succeeded =
		g_exo_hardware_config.hip_joint[EXO_HIP_JOINT_LEFT].actuator ==
			EXO_ACTUATOR_A
			? control_result->actuator_a_send_succeeded
			: control_result->actuator_b_send_succeeded;
	const bool right_send_succeeded =
		g_exo_hardware_config.hip_joint[EXO_HIP_JOINT_RIGHT].actuator ==
			EXO_ACTUATOR_A
			? control_result->actuator_a_send_succeeded
			: control_result->actuator_b_send_succeeded;
	const bool left_assist_applied =
		control_result->assist_accepted && left_send_succeeded;
	const bool right_assist_applied =
		control_result->assist_accepted && right_send_succeeded;
	const float applied_left_nm =
		left_assist_applied
			? control_result->assist_delta.left.torque_ff_nm
			: 0.0f;
	const float applied_right_nm =
		right_assist_applied
			? control_result->assist_delta.right.torque_ff_nm
			: 0.0f;

	exo_telemetry_input_t input = {
		.timestamp_us = cycle_start_us,
		.system_state = (uint8_t)system_state.state,
		.sample_flags =
			(uint8_t)((control_result->output_gate_open
					   ? EXO_TELEMETRY_SAMPLE_FLAG_OUTPUT_ALLOWED
					   : 0U) |
				  (exo_telemetry_has_applied_assist(applied_left_nm,applied_right_nm)
					   ? EXO_TELEMETRY_SAMPLE_FLAG_REAL_OUTPUT_ENABLED
					   : 0U)),
		.activation_epoch = assist_result->activation_epoch,
		.joint_position_deg = {
			.left = sensor_snapshot->left.joint_sample.position_deg,
			.right = sensor_snapshot->right.joint_sample.position_deg,
			.left_valid = left_kinematics_valid,
			.right_valid = right_kinematics_valid,
		},
		.joint_velocity_deg_s = {
			.left = sensor_snapshot->left.joint_sample.velocity_deg_s,
			.right = sensor_snapshot->right.joint_sample.velocity_deg_s,
			.left_valid = left_kinematics_valid,
			.right_valid = right_kinematics_valid,
		},
		.motor_torque_nm = {
			.left = sensor_snapshot->left.joint_sample.torque_nm,
			.right = sensor_snapshot->right.joint_sample.torque_nm,
			.left_valid = left_motor_torque_valid,
			.right_valid = right_motor_torque_valid,
		},
		.gait_phase_01 = {
			.left = gait_estimate->left_phase_01,
			.right = gait_estimate->right_phase_01,
			.left_valid = left_phase_valid,
			.right_valid = right_phase_valid,
		},
		.assist_suggestion_nm = {
			.left = assist_result->suggestion.left.torque_ff_nm,
			.right = assist_result->suggestion.right.torque_ff_nm,
			.left_valid = torque_suggestion_valid,
			.right_valid = torque_suggestion_valid,
		},
		.gait_confidence_01 = gait_estimate->confidence_01,
		.gait_confidence_valid = confidence_valid,
		.final_command = control_result->final_command,
		.final_left_valid = left_send_succeeded,
		.final_right_valid = right_send_succeeded,
		.applied_assist_nm = {
			.left = applied_left_nm,
			.right = applied_right_nm,
			.left_valid = left_send_succeeded,
			.right_valid = right_send_succeeded,
		},
	};
	exo_telemetry_push(&input);
}

/*
 * 更新“周期从何时开始、输入来自哪里、输出是否报错”等主体统计。
 * period 是相邻两次实际唤醒时刻之差；第一次循环没有前一时刻，因此不产生样本。
 * 本函数只改任务私有副本，尚未让其他任务看到半成品。
 */
static void update_cycle_diagnostics(
	control_loop_diagnostics_t *diagnostics, int64_t cycle_start_us,
	esp_err_t output_result, const hip_joint_snapshot_t *left_snapshot,
	bool left_available, const hip_joint_snapshot_t *right_snapshot,
	bool right_available, const exo_sensor_snapshot_t *sensor_snapshot,
	const exo_gait_estimate_t *gait_estimate, uint32_t snapshot_build_us,
	uint32_t gait_update_us, uint32_t assist_update_us,
	const exo_assist_result_t *assist_result,
	const exo_controller_tick_result_t *control_tick_result,
	esp_err_t algorithm_result,
	bool stack_sample_available,
	uint32_t stack_high_water_mark_bytes) {
	exo_control_loop_snapshot_t *snapshot = &diagnostics->snapshot;
	if (diagnostics->previous_cycle_start_us > 0) {
		const uint32_t period_us = clamp_duration_us(
			cycle_start_us - diagnostics->previous_cycle_start_us);
		const int32_t period_error_us = clamp_period_error_us(
			(int64_t)period_us - snapshot->target_period_us);
		const uint32_t abs_period_error_us =
			period_error_us < 0 ? (uint32_t)(-(int64_t)period_error_us)
								: (uint32_t)period_error_us;
		diagnostics->period_total_us += period_us;
		++diagnostics->period_sample_count;
		snapshot->period_sample_count = diagnostics->period_sample_count;
		if (abs_period_error_us > snapshot->period_tolerance_us) {
			++snapshot->period_out_of_tolerance_count;
		}
		snapshot->last_period_us = period_us;
		if (snapshot->min_period_us == 0U ||
			period_us < snapshot->min_period_us) {
			snapshot->min_period_us = period_us;
		}
		if (period_us > snapshot->max_period_us) {
			snapshot->max_period_us = period_us;
		}
		snapshot->mean_period_us = (uint32_t)(diagnostics->period_total_us /
											  diagnostics->period_sample_count);
		snapshot->last_period_error_us = period_error_us;
		if (abs_period_error_us > snapshot->max_abs_period_error_us) {
			snapshot->max_abs_period_error_us = abs_period_error_us;
		}
	}

	diagnostics->previous_cycle_start_us = cycle_start_us;
	snapshot->last_cycle_start_us = (uint64_t)cycle_start_us;
	++snapshot->cycle_count;
	snapshot->last_output_error = output_result;
	if (output_result != ESP_OK) {
		++snapshot->output_error_count;
	}
	snapshot->left = joint_status_from_snapshot(left_snapshot, left_available);
	snapshot->right =
		joint_status_from_snapshot(right_snapshot, right_available);
	if (sensor_snapshot != NULL) {
		snapshot->sensor_snapshot = *sensor_snapshot;
	}
	if (gait_estimate != NULL) {
		snapshot->gait_estimate = *gait_estimate;
	}
	if (assist_result != NULL) {
		snapshot->assist_result = *assist_result;
	}
	if (control_tick_result != NULL) {
		snapshot->control_tick_result = *control_tick_result;
	}
	snapshot->last_snapshot_build_us = snapshot_build_us;
	if (snapshot_build_us > snapshot->max_snapshot_build_us) {
		snapshot->max_snapshot_build_us = snapshot_build_us;
	}
	snapshot->last_gait_update_us = gait_update_us;
	if (gait_update_us > snapshot->max_gait_update_us) {
		snapshot->max_gait_update_us = gait_update_us;
	}
	snapshot->last_assist_update_us = assist_update_us;
	if (assist_update_us > snapshot->max_assist_update_us) {
		snapshot->max_assist_update_us = assist_update_us;
	}
	snapshot->last_algorithm_error = algorithm_result;
	if (algorithm_result != ESP_OK) {
		++snapshot->algorithm_error_count;
	}
	if (stack_sample_available) {
		snapshot->stack_high_water_mark_bytes = stack_high_water_mark_bytes;
	}
}

/*
 * 补齐本周期执行时间及超期统计，然后一次性发布完整副本。临界区内只有结构体复制，
 * 所以 UART/BLE 读取诊断不会看到撕裂字段，也不会长时间阻塞控制任务。
 */
static void update_timing_fields(control_loop_diagnostics_t *diagnostics,
								 uint32_t execution_us,
								 uint64_t deadline_misses,
								 uint64_t skipped_cycles) {
	exo_control_loop_snapshot_t *snapshot = &diagnostics->snapshot;
	snapshot->last_execution_us = execution_us;
	if (execution_us > snapshot->max_execution_us) {
		snapshot->max_execution_us = execution_us;
	}
	snapshot->deadline_miss_count += deadline_misses;
	snapshot->skipped_cycle_count += skipped_cycles;
}

/*
 * 发布成本必须属于本轮execution/deadline：先把主体复制到非活动缓冲区，再取得
 * 最终切换锁；等待读者、完整结构复制和时间统计都发生在cycle_end采样之前。
 */
static void publish_cycle_snapshot(control_loop_diagnostics_t *diagnostics,
								   int64_t cycle_start_us,
								   TickType_t period_ticks,
								   TickType_t *last_wake_tick) {
	uint8_t inactive_index;
	portENTER_CRITICAL(&s_diagnostics_lock);
	inactive_index = (uint8_t)(s_active_snapshot_index ^ 1U);
	portEXIT_CRITICAL(&s_diagnostics_lock);

	/* 此时inactive不被读取者使用，较大的结构复制无需关中断。 */
	s_published_snapshots[inactive_index] = diagnostics->snapshot;

	/*
	 * 取得切换锁的等待也应计时；持锁后只计算几个整数并切换索引，不做算法/CAN。
	 * 读取者只能看到上一个完整缓冲区，或本轮已经补齐timing字段的新缓冲区。
	 */
	portENTER_CRITICAL(&s_diagnostics_lock);
	const int64_t cycle_end_us = esp_timer_get_time();
	const TickType_t after_cycle_tick = xTaskGetTickCount();
	const uint32_t execution_us =
		clamp_duration_us(cycle_end_us - cycle_start_us);
	const TickType_t lag_ticks = after_cycle_tick - *last_wake_tick;
	uint64_t deadline_misses = 0U;
	uint64_t skipped_cycles = 0U;
	if (lag_ticks >= period_ticks) {
		deadline_misses = 1U;
		skipped_cycles = (uint64_t)(lag_ticks / period_ticks);
		*last_wake_tick = after_cycle_tick;
	}
	update_timing_fields(diagnostics, execution_us, deadline_misses,
						 skipped_cycles);

	/* 主体已复制，只补写刚刚测得的少量周期末字段。 */
	s_published_snapshots[inactive_index].last_execution_us =
		diagnostics->snapshot.last_execution_us;
	s_published_snapshots[inactive_index].max_execution_us =
		diagnostics->snapshot.max_execution_us;
	s_published_snapshots[inactive_index].deadline_miss_count =
		diagnostics->snapshot.deadline_miss_count;
	s_published_snapshots[inactive_index].skipped_cycle_count =
		diagnostics->snapshot.skipped_cycle_count;
	s_active_snapshot_index = inactive_index;
	portEXIT_CRITICAL(&s_diagnostics_lock);
}

/*
 * 唯一控制任务入口，创建后永不退出。绝对节拍延时使用上一次计划唤醒点计算
 * 下一点，避免普通“执行完再延时”造成周期随计算时间不断漂移。系统处于 IDLE 时
 * 循环仍运行；是否产生控制 I/O 完全由 exo_system 的输出门决定。
 */
static void control_task(void *arg) {
	const TickType_t period_ticks = (TickType_t)(uintptr_t)arg;
	const uint32_t stack_sample_period_cycles =
		(uint32_t)configTICK_RATE_HZ / (uint32_t)period_ticks;
	TickType_t last_wake_tick = xTaskGetTickCount();
	const TaskHandle_t current_task = xTaskGetCurrentTaskHandle();
	uint32_t stack_sample_countdown = 0U;
	uint32_t stack_high_water_mark_bytes = 0U;
	control_loop_diagnostics_t diagnostics = {0};

	portENTER_CRITICAL(&s_diagnostics_lock);
	diagnostics.snapshot = s_published_snapshots[s_active_snapshot_index];
	portEXIT_CRITICAL(&s_diagnostics_lock);
	diagnostics.snapshot.task_alive = true;
	diagnostics.snapshot.task_handle_identity = (uintptr_t)current_task;

	/*
	 * 每个周期固定执行四步：定时唤醒 -> 读取左右关节快照 -> 经过系统输出门
	 * 执行一次控制 -> 更新诊断。工程中不允许再创建第二个周期运动任务。
	 */
	for (;;) {
		xTaskDelayUntil(&last_wake_tick, period_ticks);
		const int64_t cycle_start_us = esp_timer_get_time();

		hip_joint_snapshot_t left_snapshot = {0};
		hip_joint_snapshot_t right_snapshot = {0};
		const bool left_available =
			hip_joint_get_snapshot(EXO_HIP_JOINT_LEFT, &left_snapshot);
		const bool right_available =
			hip_joint_get_snapshot(EXO_HIP_JOINT_RIGHT, &right_snapshot);
		/* RX can publish between task wakeup and either snapshot copy. The
		 * capture clock must follow both copies, otherwise a valid new frame
		 * appears to come from the future and latches the AFO input fault. */
		const int64_t sensor_capture_us = esp_timer_get_time();

		/*
		 * 阶段9算法链在系统输出门之前运行，所以即使整机仍在IDLE，也能观察站立、
		 * 起步和步态锁定；但它只产生诊断与SHADOW建议，不拥有真实输出许可。
		 */
		exo_sensor_joint_snapshot_t left_sensor =
			sensor_joint_from_hip(&left_snapshot, left_available);
		exo_sensor_joint_snapshot_t right_sensor =
			sensor_joint_from_hip(&right_snapshot, right_available);
		exo_sensor_snapshot_t sensor_snapshot = {0};
		exo_gait_estimate_t gait_estimate = {0};
		exo_assist_result_t assist_tick_result = {0};
		exo_controller_tick_result_t control_tick_result = {0};
		esp_err_t algorithm_result = ESP_OK;

		int64_t stage_start_us = esp_timer_get_time();
		if (!exo_sensor_snapshot_build(
				&s_sensor_builder, sensor_capture_us, &left_sensor, &right_sensor,
				&sensor_snapshot)) {
			algorithm_result = ESP_FAIL;
		}
		const uint32_t snapshot_build_us = clamp_duration_us(
			esp_timer_get_time() - stage_start_us);

		stage_start_us = esp_timer_get_time();
		if (algorithm_result == ESP_OK &&
			!exo_gait_estimator_update(&s_gait_estimator, &sensor_snapshot,
								   &gait_estimate)) {
			algorithm_result = ESP_FAIL;
		}
		const uint32_t gait_update_us = clamp_duration_us(
			esp_timer_get_time() - stage_start_us);

		stage_start_us = esp_timer_get_time();
		/* AFO观察器沿用同一控制任务与快照；无第二运动任务，无NVS/BLE I/O。
		 * 非新策略、未START或故障时permit=0，只估计/记录而不能产生建议。 */
        exo_system_control_state_t adaptive_system={0};
        static exo_assist_snapshot_t adaptive_contract;
        (void)exo_system_get_control_state(&adaptive_system);
        (void)exo_assist_get_snapshot(&adaptive_contract);
        uint32_t adaptive_permit=exo_adaptive_runtime_permit(&adaptive_contract,adaptive_system.output_allowed);
        exo_adaptive_runtime_update(&sensor_snapshot,adaptive_permit,(uint32_t)adaptive_system.state,
            adaptive_system.state==EXO_SYSTEM_STATE_ERROR);
		if (algorithm_result == ESP_OK) {
			const esp_err_t assist_result =
				exo_assist_process_tick(&sensor_snapshot, &gait_estimate,
									&assist_tick_result);
			if (assist_result != ESP_OK) {
				algorithm_result = assist_result;
			}
		}
		const uint32_t assist_update_us = clamp_duration_us(
			esp_timer_get_time() - stage_start_us);

		/*
		 * 把同一周期的 assist result 及其来源序号直接交给输出门。controller 会
		 * 再核对 epoch/sequence/timestamp，最终纯值结果供本周期诊断与遥测使用。
		 */
		const esp_err_t output_result = exo_system_process_control_tick(
			sensor_capture_us, &sensor_snapshot, &assist_tick_result, sensor_snapshot.sequence,
			gait_estimate.sequence, &control_tick_result);

        const bool adaptive_left_sent=g_exo_hardware_config.hip_joint[EXO_HIP_JOINT_LEFT].actuator==EXO_ACTUATOR_A
            ? control_tick_result.actuator_a_send_succeeded : control_tick_result.actuator_b_send_succeeded;
        const bool adaptive_right_sent=g_exo_hardware_config.hip_joint[EXO_HIP_JOINT_RIGHT].actuator==EXO_ACTUATOR_A
            ? control_tick_result.actuator_a_send_succeeded : control_tick_result.actuator_b_send_succeeded;
        exo_adaptive_runtime_finish(control_tick_result.final_command.left.torque_ff_nm,
            control_tick_result.final_command.right.torque_ff_nm,adaptive_left_sent,adaptive_right_sent,
            diagnostics.snapshot.last_execution_us);
        exo_adaptive_runtime_trace_finish(&sensor_snapshot,&assist_tick_result,&control_tick_result,output_result);

		/*
		 * 控制输出结果确定后再采样，才能区分“策略建议”与“确实加入并发送的助力”。
		 * 此调用不做 BLE I/O；未通过 START_TELEMETRY 开流时会立即返回。
		 */
		push_phase10_telemetry(sensor_capture_us, &sensor_snapshot,
						   &gait_estimate, &assist_tick_result,
						   &control_tick_result);
		const bool sample_stack = stack_sample_countdown == 0U;
		if (sample_stack) {
			stack_high_water_mark_bytes =
				(uint32_t)uxTaskGetStackHighWaterMark(NULL);
			stack_sample_countdown = stack_sample_period_cycles - 1U;
		} else {
			--stack_sample_countdown;
		}

		/*
		 * 执行时间覆盖关节读取、控制输出、低频栈检查和主体诊断更新，不能只测算法
		 * 公式而漏掉实际周期负担。最后一次快照发布在计时结束后单独完成。
		 */
		update_cycle_diagnostics(
			&diagnostics, cycle_start_us, output_result, &left_snapshot,
			left_available, &right_snapshot, right_available, &sensor_snapshot,
			&gait_estimate, snapshot_build_us, gait_update_us,
			assist_update_us, &assist_tick_result, &control_tick_result,
			algorithm_result, sample_stack,
			stack_high_water_mark_bytes);

		/* 最后发布双缓冲；它自身的复制和锁等待也计入本周期execution。 */
		publish_cycle_snapshot(&diagnostics, cycle_start_us, period_ticks,
							   &last_wake_tick);
	}
}

/*
 * 验证配置可由 FreeRTOS tick 精确整除，初始化公开快照，再用静态内存创建任务。
 * 初始化状态在短临界区内变化；若另一个调用者看见 INITIALIZING，它不会再次创建
 * 任务，真正的启动就绪仍由 task_alive 快照字段判断。
 */
esp_err_t exo_control_loop_init(void) {
	const uint32_t frequency_hz = g_exo_hardware_config.control_frequency_hz;
	if (frequency_hz == 0U || frequency_hz > (uint32_t)configTICK_RATE_HZ ||
		((uint32_t)configTICK_RATE_HZ % frequency_hz) != 0U) {
		return ESP_ERR_INVALID_ARG;
	}

	portENTER_CRITICAL(&s_diagnostics_lock);
	if (s_init_state == CONTROL_LOOP_INITIALIZED) {
		portEXIT_CRITICAL(&s_diagnostics_lock);
		return ESP_OK;
	}
	if (s_init_state == CONTROL_LOOP_INITIALIZING) {
		portEXIT_CRITICAL(&s_diagnostics_lock);
		return ESP_OK;
	}
	s_init_state = CONTROL_LOOP_INITIALIZING;
	memset(s_published_snapshots, 0, sizeof(s_published_snapshots));
	s_active_snapshot_index = 0U;
	s_published_snapshots[0].initialized = true;
	s_published_snapshots[0].configured_frequency_hz = frequency_hz;
	s_published_snapshots[0].target_period_us = 1000000U / frequency_hz;
	s_published_snapshots[0].period_tolerance_us =
		EXO_CONTROL_PERIOD_TOLERANCE_US;
	s_published_snapshots[0].last_output_error = ESP_OK;
	s_published_snapshots[0].last_algorithm_error = ESP_OK;
	s_published_snapshots[1] = s_published_snapshots[0];
	portEXIT_CRITICAL(&s_diagnostics_lock);

	/*
	 * 纯内存算法对象必须在任务创建前一次性校验。若静态配置无效，拒绝启动控制
	 * 任务，比让一个永久任务每10 ms重复报错更容易定位，也不会留下半初始化对象。
	 */
	if (!exo_sensor_snapshot_builder_init(&s_sensor_builder,
									   &g_exo_sensor_snapshot_config) ||
		!exo_gait_estimator_init(&s_gait_estimator,
							 &g_exo_gait_estimator_config)) {
		portENTER_CRITICAL(&s_diagnostics_lock);
		s_init_state = CONTROL_LOOP_NOT_INITIALIZED;
		memset(s_published_snapshots, 0, sizeof(s_published_snapshots));
		portEXIT_CRITICAL(&s_diagnostics_lock);
		return ESP_ERR_INVALID_ARG;
	}

	const TickType_t period_ticks =
		(TickType_t)((uint32_t)configTICK_RATE_HZ / frequency_hz);
	TaskHandle_t task_handle = xTaskCreateStaticPinnedToCore(
		control_task, "exo_control", EXO_CONTROL_TASK_STACK_BYTES,
		(void *)(uintptr_t)period_ticks, EXO_CONTROL_TASK_PRIORITY,
		s_control_task_stack, &s_control_task_buffer, EXO_CONTROL_TASK_CORE);
	if (task_handle == NULL) {
		portENTER_CRITICAL(&s_diagnostics_lock);
		s_init_state = CONTROL_LOOP_NOT_INITIALIZED;
		memset(s_published_snapshots, 0, sizeof(s_published_snapshots));
		portEXIT_CRITICAL(&s_diagnostics_lock);
		return ESP_ERR_NO_MEM;
	}

	portENTER_CRITICAL(&s_diagnostics_lock);
	s_init_state = CONTROL_LOOP_INITIALIZED;
	s_published_snapshots[0].task_handle_identity = (uintptr_t)task_handle;
	s_published_snapshots[1].task_handle_identity = (uintptr_t)task_handle;
	portEXIT_CRITICAL(&s_diagnostics_lock);
	return ESP_OK;
}

/* 在短临界区内复制最近完整发布的快照；未完成初始化时清零输出并返回 false。 */
bool exo_control_loop_get_snapshot(exo_control_loop_snapshot_t *snapshot) {
	if (snapshot == NULL) {
		return false;
	}

	portENTER_CRITICAL(&s_diagnostics_lock);
	if (s_init_state != CONTROL_LOOP_INITIALIZED) {
		portEXIT_CRITICAL(&s_diagnostics_lock);
		memset(snapshot, 0, sizeof(*snapshot));
		return false;
	}
	*snapshot = s_published_snapshots[s_active_snapshot_index];
	portEXIT_CRITICAL(&s_diagnostics_lock);
	return true;
}
