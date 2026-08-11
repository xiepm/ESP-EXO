/*
 * 文件导读：本文件拥有整机六态状态机和慢速 START/STOP 执行器，但不拥有 100 Hz
 * 调度，也不实现控制律。系统中有三个并发入口：
 *
 * 1. BLE/UART 调用请求接口，只做快速检查、发布意图并通知后台；
 * 2. exo_executor 后台任务串行执行含延时、CAN 和反馈等待的硬件序列；
 * 3. 唯一 100 Hz 控制任务调用
 * process_control_tick()，只在输出门打开时运行控制器。
 *
 * FreeRTOS 初学者需要区分两种同步手段：portMUX 临界区只保护很短的共享内存访问，
 * 期间不能阻塞；operation mutex 可以跨延时和 CAN 操作持有，用来串行化整段序列。
 * 任务通知只是“请醒来重新检查状态”的信号，状态快照才是请求的事实来源。
 */

#include "exo_system.h"

#include <math.h>
#include <string.h>

#include "driver/twai.h"
#include "esp_timer.h"
#include "exo_assist.h"
#include "exo_control_loop.h"
#include "exo_controller.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "hip_joint.h"
#include "rs01_driver.h"

#define EXO_EXECUTOR_TASK_STACK_BYTES 4096U
#define EXO_EXECUTOR_TASK_PRIORITY 6U
#define EXO_EXECUTOR_TASK_CORE 0
#define EXO_START_DISABLE_DELAY_MS 50U
#define EXO_START_MODE_DELAY_MS 20U
#define EXO_START_ENABLE_DELAY_MS 50U
#define EXO_RELEASE_DELAY_MS 20U
#define EXO_DISABLE_DELAY_MS 50U
#define EXO_FEEDBACK_POLL_MS 10U
#define EXO_FEEDBACK_WAIT_MS 500U

/* START 序列同时区分失败、用户取消和具体前置阻止原因，便于最终正确记账。 */
typedef struct {
	esp_err_t error;
	bool canceled;
	exo_start_block_reason_t block_reason;
} start_sequence_result_t;

/*
 * 执行器任务和 operation mutex 均使用静态内存。s_state_lock 保护 s_snapshot、
 * 初始化标志和维护代数；s_operation_mutex 则保护完整
 * START/STOP/维护操作互不交叉。 s_maintenance_generation
 * 每次维护开始/结束递增，用于识别锁外就绪检查期间的竞争。
 */
static StaticTask_t s_executor_task_buffer;
static StackType_t
	s_executor_task_stack[EXO_EXECUTOR_TASK_STACK_BYTES / sizeof(StackType_t)];
static StaticSemaphore_t s_operation_mutex_buffer;
static SemaphoreHandle_t s_operation_mutex;
static TaskHandle_t s_executor_task;
static exo_system_snapshot_t s_snapshot;
static bool s_initialized;
static uint32_t s_maintenance_generation;
static portMUX_TYPE s_state_lock = portMUX_INITIALIZER_UNLOCKED;

/* ESP 定时器提供上电后的单调微秒时间，不受日历时间调整影响。 */
static uint64_t now_us(void) { return (uint64_t)esp_timer_get_time(); }

/* 调用者必须已持有 s_state_lock；重复写入同一状态不计作新转换。 */
static void set_state_locked(exo_system_state_t state, uint64_t timestamp_us) {
	if (s_snapshot.state == state) {
		return;
	}
	s_snapshot.state = state;
	s_snapshot.state_entered_us = timestamp_us;
	++s_snapshot.transition_count;
}

/* 只有真正拒绝/内部失败的即时请求结果才累计 rejected_request_count。 */
static bool request_is_rejected(exo_request_result_t result) {
	return result == EXO_REQUEST_REJECTED_NOT_READY ||
		   result == EXO_REQUEST_REJECTED_BUSY ||
		   result == EXO_REQUEST_REJECTED_ERROR ||
		   result == EXO_REQUEST_INTERNAL_ERROR;
}

/* 调用者必须已持有 s_state_lock；统一记录最近请求及其时间。 */
static void record_request_locked(exo_request_result_t result,
								  uint64_t timestamp_us) {
	s_snapshot.last_request_result = result;
	s_snapshot.last_request_us = timestamp_us;
	if (request_is_rejected(result)) {
		++s_snapshot.rejected_request_count;
	}
}

/*
 * active_step 和阻止原因会被 UART/BLE 并发读取，因此通过短临界区单独发布；
 * 这里不执行任何驱动或等待操作。
 */
static void set_active_step(exo_system_step_t step) {
	portENTER_CRITICAL(&s_state_lock);
	s_snapshot.active_step = step;
	portEXIT_CRITICAL(&s_state_lock);
}

/* 以同样的短临界区方式发布最近 START 阻止原因。 */
static void set_start_block_reason(exo_start_block_reason_t reason) {
	portENTER_CRITICAL(&s_state_lock);
	s_snapshot.start_block_reason = reason;
	portEXIT_CRITICAL(&s_state_lock);
}

/* 逐侧检查关节对象，再检查其映射到的 RS01 快照和电机故障。返回首个明确原因。 */
static exo_start_block_reason_t
validate_joint(exo_hip_joint_id_t joint, const hip_joint_snapshot_t *snapshot) {
	const bool left = joint == EXO_HIP_JOINT_LEFT;
	if (!snapshot->mapping_valid) {
		return left ? EXO_START_BLOCK_LEFT_MAPPING_INVALID
					: EXO_START_BLOCK_RIGHT_MAPPING_INVALID;
	}
	if (!snapshot->direction_valid) {
		return left ? EXO_START_BLOCK_LEFT_DIRECTION_INVALID
					: EXO_START_BLOCK_RIGHT_DIRECTION_INVALID;
	}
	if (!snapshot->zero_valid) {
		return left ? EXO_START_BLOCK_LEFT_ZERO_INVALID
					: EXO_START_BLOCK_RIGHT_ZERO_INVALID;
	}
	if (!snapshot->feedback_valid) {
		return left ? EXO_START_BLOCK_LEFT_FEEDBACK_INVALID
					: EXO_START_BLOCK_RIGHT_FEEDBACK_INVALID;
	}
	if (!snapshot->online) {
		return left ? EXO_START_BLOCK_LEFT_OFFLINE
					: EXO_START_BLOCK_RIGHT_OFFLINE;
	}
	if (!snapshot->joint_valid) {
		return left ? EXO_START_BLOCK_LEFT_FEEDBACK_INVALID
					: EXO_START_BLOCK_RIGHT_FEEDBACK_INVALID;
	}

	rs01_feedback_snapshot_t motor;
	if (!rs01_get_feedback(snapshot->actuator, &motor) || !motor.valid) {
		return left ? EXO_START_BLOCK_LEFT_FEEDBACK_INVALID
					: EXO_START_BLOCK_RIGHT_FEEDBACK_INVALID;
	}
	if (!motor.online) {
		return left ? EXO_START_BLOCK_LEFT_OFFLINE
					: EXO_START_BLOCK_RIGHT_OFFLINE;
	}
	if (motor.faults.present) {
		return left ? EXO_START_BLOCK_LEFT_MOTOR_FAULT
					: EXO_START_BLOCK_RIGHT_MOTOR_FAULT;
	}
	return EXO_START_BLOCK_NONE;
}

/*
 * 汇总 START 的当前硬件门槛：永久控制任务存活、TWAI 运行、左右关节均已映射、
 * 定向、归零、在线且无电机故障。本函数只读取快照，不使能电机、不发送 CAN。
 */
static exo_start_block_reason_t hardware_start_block_reason(void) {
	exo_control_loop_snapshot_t control;
	if (!exo_control_loop_get_snapshot(&control) || !control.initialized ||
		!control.task_alive) {
		return EXO_START_BLOCK_CONTROL_TASK_NOT_READY;
	}

	twai_status_info_t twai_status;
	if (twai_get_status_info(&twai_status) != ESP_OK ||
		twai_status.state != TWAI_STATE_RUNNING) {
		return EXO_START_BLOCK_TWAI_NOT_RUNNING;
	}

	hip_joint_snapshot_t left;
	hip_joint_snapshot_t right;
	if (!hip_joint_get_snapshot(EXO_HIP_JOINT_LEFT, &left)) {
		return EXO_START_BLOCK_LEFT_MAPPING_INVALID;
	}
	if (!hip_joint_get_snapshot(EXO_HIP_JOINT_RIGHT, &right)) {
		return EXO_START_BLOCK_RIGHT_MAPPING_INVALID;
	}

	exo_start_block_reason_t reason = validate_joint(EXO_HIP_JOINT_LEFT, &left);
	if (reason != EXO_START_BLOCK_NONE) {
		return reason;
	}
	return validate_joint(EXO_HIP_JOINT_RIGHT, &right);
}

/* START 序列在每个可取消边界调用它；STOP 会在状态锁内撤销该条件。 */
static bool start_still_requested(void) {
	portENTER_CRITICAL(&s_state_lock);
	const bool requested = s_snapshot.state == EXO_SYSTEM_STATE_STARTING &&
						   s_snapshot.run_requested;
	portEXIT_CRITICAL(&s_state_lock);
	return requested;
}

/*
 * START 专用可取消延时。ulTaskNotifyTake() 既能等待剩余时间，也会被 STOP 的通知
 * 提前唤醒；醒来后必须重新读状态，不能把“收到通知”本身解释成继续 START。
 */
static bool wait_start_delay_ms(uint32_t delay_ms) {
	TickType_t total_ticks = pdMS_TO_TICKS(delay_ms);
	if (delay_ms > 0U && total_ticks == 0U) {
		total_ticks = 1U;
	}
	const TickType_t start_tick = xTaskGetTickCount();
	while ((xTaskGetTickCount() - start_tick) < total_ticks) {
		const TickType_t elapsed = xTaskGetTickCount() - start_tick;
		(void)ulTaskNotifyTake(pdTRUE, total_ticks - elapsed);
		if (!start_still_requested()) {
			return false;
		}
	}
	return start_still_requested();
}

/*
 * STOP 清理使用的不可取消延时。通知可以提前唤醒一次，但循环会补足剩余时长；
 * STOP 一旦开始就必须继续走到失能确认，不能被另一条请求截断。
 */
static void executor_delay_ms(uint32_t delay_ms) {
	TickType_t total_ticks = pdMS_TO_TICKS(delay_ms);
	if (delay_ms > 0U && total_ticks == 0U) {
		total_ticks = 1U;
	}
	const TickType_t start_tick = xTaskGetTickCount();
	while ((xTaskGetTickCount() - start_tick) < total_ticks) {
		const TickType_t elapsed = xTaskGetTickCount() - start_tick;
		(void)ulTaskNotifyTake(pdTRUE, total_ticks - elapsed);
	}
}

/* 同一次判断读取 A/B 两个驱动快照；任一对象尚未可读即返回 false。 */
static bool get_motor_pair(rs01_feedback_snapshot_t *feedback_a,
						   rs01_feedback_snapshot_t *feedback_b) {
	return rs01_get_feedback(EXO_ACTUATOR_A, feedback_a) &&
		   rs01_get_feedback(EXO_ACTUATOR_B, feedback_b);
}

/*
 * “已运行/已停止”必须由命令后的新反馈证明：feedback_count 要不同于发命令前的
 * 基准值。这样不会把缓存中的旧 RUNNING 或 REST 状态误认为本次序列的确认。
 */
static bool
running_feedback_is_confirmed(uint32_t count_a, uint32_t count_b,
							  rs01_feedback_snapshot_t *feedback_a,
							  rs01_feedback_snapshot_t *feedback_b) {
	if (!get_motor_pair(feedback_a, feedback_b)) {
		return false;
	}
	return feedback_a->valid && feedback_b->valid && feedback_a->online &&
		   feedback_b->online && feedback_a->feedback_count != count_a &&
		   feedback_b->feedback_count != count_b &&
		   feedback_a->enabled_requested && feedback_b->enabled_requested &&
		   feedback_a->control_mode == RS01_CONTROL_MODE_MOTION &&
		   feedback_b->control_mode == RS01_CONTROL_MODE_MOTION &&
		   feedback_a->mode == RS01_MODE_STATUS_RUNNING &&
		   feedback_b->mode == RS01_MODE_STATUS_RUNNING &&
		   !feedback_a->faults.present && !feedback_b->faults.present;
}

/* STOP 的确认条件是两侧均产生新、有效、在线、未请求使能且模式为 REST 的反馈。
 */
static bool stopped_feedback_is_confirmed(uint32_t count_a, uint32_t count_b) {
	rs01_feedback_snapshot_t feedback_a;
	rs01_feedback_snapshot_t feedback_b;
	if (!get_motor_pair(&feedback_a, &feedback_b)) {
		return false;
	}
	return feedback_a.valid && feedback_b.valid && feedback_a.online &&
		   feedback_b.online && feedback_a.feedback_count != count_a &&
		   feedback_b.feedback_count != count_b &&
		   !feedback_a.enabled_requested && !feedback_b.enabled_requested &&
		   feedback_a.mode == RS01_MODE_STATUS_RESET &&
		   feedback_b.mode == RS01_MODE_STATUS_RESET;
}

/*
 * 轮询新 RUNNING 反馈，最长等待 EXO_FEEDBACK_WAIT_MS；等待期间 STOP 通知会唤醒
 * 执行器，并通过 start_still_requested() 把结果分类为取消而不是启动失败。
 */
static esp_err_t
wait_for_running_feedback(uint32_t count_a, uint32_t count_b,
						  rs01_feedback_snapshot_t *feedback_a,
						  rs01_feedback_snapshot_t *feedback_b) {
	const TickType_t timeout_ticks = pdMS_TO_TICKS(EXO_FEEDBACK_WAIT_MS);
	const TickType_t poll_ticks = pdMS_TO_TICKS(EXO_FEEDBACK_POLL_MS);
	const TickType_t start_tick = xTaskGetTickCount();
	for (;;) {
		if (!start_still_requested()) {
			return ESP_ERR_INVALID_STATE;
		}
		if (running_feedback_is_confirmed(count_a, count_b, feedback_a,
										  feedback_b)) {
			return ESP_OK;
		}
		const TickType_t elapsed = xTaskGetTickCount() - start_tick;
		if (elapsed >= timeout_ticks) {
			return ESP_ERR_TIMEOUT;
		}
		const TickType_t remaining = timeout_ticks - elapsed;
		(void)ulTaskNotifyTake(pdTRUE,
							   remaining < poll_ticks ? remaining : poll_ticks);
	}
}

/* 轮询两侧新 REST 反馈直至成功或超时；STOP 清理不接受取消。 */
static esp_err_t wait_for_stopped_feedback(uint32_t count_a, uint32_t count_b) {
	const TickType_t timeout_ticks = pdMS_TO_TICKS(EXO_FEEDBACK_WAIT_MS);
	const TickType_t poll_ticks = pdMS_TO_TICKS(EXO_FEEDBACK_POLL_MS);
	const TickType_t start_tick = xTaskGetTickCount();
	for (;;) {
		if (stopped_feedback_is_confirmed(count_a, count_b)) {
			return ESP_OK;
		}
		const TickType_t elapsed = xTaskGetTickCount() - start_tick;
		if (elapsed >= timeout_ticks) {
			return ESP_ERR_TIMEOUT;
		}
		const TickType_t remaining = timeout_ticks - elapsed;
		(void)ulTaskNotifyTake(pdTRUE,
							   remaining < poll_ticks ? remaining : poll_ticks);
	}
}

/* 以下三个小构造器统一 START 返回值，避免错误和“用户取消”被混为一谈。 */
static start_sequence_result_t start_result(esp_err_t error, bool canceled,
											exo_start_block_reason_t reason) {
	const start_sequence_result_t result = {
		.error = error,
		.canceled = canceled,
		.block_reason = reason,
	};
	return result;
}

static start_sequence_result_t canceled_start(void) {
	return start_result(ESP_OK, true, EXO_START_BLOCK_NONE);
}

static start_sequence_result_t failed_start(esp_err_t error) {
	return start_result(error, false, EXO_START_BLOCK_NONE);
}

/*
 * 执行 START 硬件序列，但不自行发布 RUNNING：
 * 1. 再次检查全部前置条件；
 * 2. 依次失能 A/B，设置 Motion 模式，再依次使能 A/B；
 * 3. 以当前电机位置发送速度、力矩、KP、KD 全为零的过渡帧；
 * 4. 等待两侧命令后的新 RUNNING 反馈，排除旧缓存；
 * 5. 读取 LEFT/RIGHT 人体关节中心并 arm 控制器。
 *
 * 每个必要延时后都会检查 STOP 是否已经取消 START。只有 handle_start() 在确认
 * 状态竞争仍由 START 获胜后，才会同时发布 RUNNING 和 output_allowed=true。
 */
static start_sequence_result_t execute_start_sequence(void) {
	set_active_step(EXO_SYSTEM_STEP_CHECK_PRECONDITIONS);
	if (!start_still_requested()) {
		return canceled_start();
	}
	const exo_start_block_reason_t reason = hardware_start_block_reason();
	if (reason != EXO_START_BLOCK_NONE) {
		set_start_block_reason(reason);
		return start_result(ESP_ERR_INVALID_STATE, false, reason);
	}

	rs01_feedback_snapshot_t feedback_a;
	rs01_feedback_snapshot_t feedback_b;
	if (!get_motor_pair(&feedback_a, &feedback_b)) {
		return failed_start(ESP_ERR_INVALID_STATE);
	}

	set_active_step(EXO_SYSTEM_STEP_DISABLE_MOTORS);
	esp_err_t err = rs01_disable(EXO_ACTUATOR_A);
	if (err != ESP_OK) {
		return failed_start(err);
	}
	if (!wait_start_delay_ms(EXO_START_DISABLE_DELAY_MS)) {
		return canceled_start();
	}
	err = rs01_disable(EXO_ACTUATOR_B);
	if (err != ESP_OK) {
		return failed_start(err);
	}
	if (!wait_start_delay_ms(EXO_START_DISABLE_DELAY_MS)) {
		return canceled_start();
	}

	set_active_step(EXO_SYSTEM_STEP_SET_MOTION_MODE);
	err = rs01_set_control_mode(EXO_ACTUATOR_A, RS01_CONTROL_MODE_MOTION);
	if (err != ESP_OK) {
		return failed_start(err);
	}
	if (!wait_start_delay_ms(EXO_START_MODE_DELAY_MS)) {
		return canceled_start();
	}
	err = rs01_set_control_mode(EXO_ACTUATOR_B, RS01_CONTROL_MODE_MOTION);
	if (err != ESP_OK) {
		return failed_start(err);
	}
	if (!wait_start_delay_ms(EXO_START_MODE_DELAY_MS)) {
		return canceled_start();
	}

	set_active_step(EXO_SYSTEM_STEP_ENABLE_MOTORS);
	err = rs01_enable(EXO_ACTUATOR_A);
	if (err != ESP_OK) {
		return failed_start(err);
	}
	if (!wait_start_delay_ms(EXO_START_ENABLE_DELAY_MS)) {
		return canceled_start();
	}
	err = rs01_enable(EXO_ACTUATOR_B);
	if (err != ESP_OK) {
		return failed_start(err);
	}
	if (!wait_start_delay_ms(EXO_START_ENABLE_DELAY_MS)) {
		return canceled_start();
	}

	if (!get_motor_pair(&feedback_a, &feedback_b) || !feedback_a.valid ||
		!feedback_b.valid || !feedback_a.online || !feedback_b.online ||
		!isfinite(feedback_a.sample.position_rad) ||
		!isfinite(feedback_b.sample.position_rad)) {
		return failed_start(ESP_ERR_INVALID_STATE);
	}
	const uint32_t count_a = feedback_a.feedback_count;
	const uint32_t count_b = feedback_b.feedback_count;

	set_active_step(EXO_SYSTEM_STEP_SEND_ZERO_OUTPUT);
	err = rs01_send_motion(EXO_ACTUATOR_A, feedback_a.sample.position_rad, 0.0f,
						   0.0f, 0.0f, 0.0f);
	if (err != ESP_OK) {
		return failed_start(err);
	}
	if (!start_still_requested()) {
		return canceled_start();
	}
	err = rs01_send_motion(EXO_ACTUATOR_B, feedback_b.sample.position_rad, 0.0f,
						   0.0f, 0.0f, 0.0f);
	if (err != ESP_OK) {
		return failed_start(err);
	}

	set_active_step(EXO_SYSTEM_STEP_WAIT_FRESH_FEEDBACK);
	err = wait_for_running_feedback(count_a, count_b, &feedback_a, &feedback_b);
	if (err != ESP_OK) {
		return start_still_requested() ? failed_start(err) : canceled_start();
	}
	if (!isfinite(feedback_a.sample.position_rad) ||
		!isfinite(feedback_b.sample.position_rad)) {
		return failed_start(ESP_ERR_INVALID_STATE);
	}

	set_active_step(EXO_SYSTEM_STEP_ARM_CONTROLLER);
	hip_joint_snapshot_t left;
	hip_joint_snapshot_t right;
	/*
	 * 这里锁存的是通过新鲜度门检查后的 LEFT/RIGHT 人体关节角（deg），不是
	 * A/B 原始电机角。控制器随后只处理人体坐标，发送前再统一映射到 RS01。
	 */
	if (!hip_joint_get_snapshot(EXO_HIP_JOINT_LEFT, &left) ||
		!hip_joint_get_snapshot(EXO_HIP_JOINT_RIGHT, &right) ||
		!left.joint_valid || !right.joint_valid ||
		!isfinite(left.joint_sample.position_deg) ||
		!isfinite(right.joint_sample.position_deg)) {
		return failed_start(ESP_ERR_INVALID_STATE);
	}
	const exo_controller_centers_t centers = {
		.left_deg = left.joint_sample.position_deg,
		.right_deg = right.joint_sample.position_deg,
	};
	err = exo_controller_arm(&centers, (int64_t)now_us());
	if (err != ESP_OK) {
		return failed_start(err);
	}
	/*
	 * 高层助力配置与阶段8底层控制选择在同一个START边界锁存。阶段9即使选择
	 * ACTIVE_PHASE或AAN也只能锁存SHADOW，exo_assist不会打开真实输出。
	 */
	err = exo_assist_arm();
	if (err != ESP_OK) {
		return failed_start(err);
	}
	if (!start_still_requested()) {
		return canceled_start();
	}
	return start_result(ESP_OK, false, EXO_START_BLOCK_NONE);
}

/* 清理过程中继续尝试后续步骤，同时保留最早出现的命令错误供最终诊断。 */
static esp_err_t first_error(esp_err_t current, esp_err_t candidate) {
	return current == ESP_OK ? candidate : current;
}

/*
 * 执行 STOP 清理序列。请求入口或 START
 * 失败路径在调用本函数前已经关闭周期输出门：
 * 1. disarm 控制器，并等待已经进入的 100 Hz tick 退出；
 * 2. 向两侧当前反馈位置发送零速度、零力矩、零增益的释力帧；
 * 3. 依次失能 A/B；
 * 4. 等待命令后的新 REST 反馈确认实际停止。
 *
 * 某一发送步骤失败不会跳过另一侧或后续失能。最终新 REST 反馈是权威结果：只要
 * 两侧确已停止即可回到 IDLE；无法确认停止才进入 ERROR。
 */
static esp_err_t execute_stop_sequence(void) {
	set_active_step(EXO_SYSTEM_STEP_DISARM_CONTROLLER);
	/* 先让高层策略停止发布建议，再有界等待可能正在发送的底层控制tick退出。 */
	esp_err_t command_error = exo_assist_disarm();
	command_error = first_error(command_error, exo_controller_disarm());

	rs01_feedback_snapshot_t feedback_a;
	rs01_feedback_snapshot_t feedback_b;
	const bool have_feedback = get_motor_pair(&feedback_a, &feedback_b);

	set_active_step(EXO_SYSTEM_STEP_RELEASE_OUTPUT);
	if (have_feedback && feedback_a.valid && feedback_a.online &&
		isfinite(feedback_a.sample.position_rad)) {
		command_error = first_error(
			command_error,
			rs01_send_motion(EXO_ACTUATOR_A, feedback_a.sample.position_rad,
							 0.0f, 0.0f, 0.0f, 0.0f));
	} else {
		command_error = first_error(command_error, ESP_ERR_INVALID_STATE);
	}
	executor_delay_ms(EXO_RELEASE_DELAY_MS);
	if (have_feedback && feedback_b.valid && feedback_b.online &&
		isfinite(feedback_b.sample.position_rad)) {
		command_error = first_error(
			command_error,
			rs01_send_motion(EXO_ACTUATOR_B, feedback_b.sample.position_rad,
							 0.0f, 0.0f, 0.0f, 0.0f));
	} else {
		command_error = first_error(command_error, ESP_ERR_INVALID_STATE);
	}
	executor_delay_ms(EXO_RELEASE_DELAY_MS);

	uint32_t count_a = 0U;
	uint32_t count_b = 0U;
	if (get_motor_pair(&feedback_a, &feedback_b)) {
		count_a = feedback_a.feedback_count;
		count_b = feedback_b.feedback_count;
	} else {
		command_error = first_error(command_error, ESP_ERR_INVALID_STATE);
	}

	set_active_step(EXO_SYSTEM_STEP_DISABLE_MOTORS);
	command_error = first_error(command_error, rs01_disable(EXO_ACTUATOR_A));
	executor_delay_ms(EXO_DISABLE_DELAY_MS);
	command_error = first_error(command_error, rs01_disable(EXO_ACTUATOR_B));
	executor_delay_ms(EXO_DISABLE_DELAY_MS);

	set_active_step(EXO_SYSTEM_STEP_VERIFY_DISABLED);
	const esp_err_t confirmation_error =
		wait_for_stopped_feedback(count_a, count_b);
	/* 新 REST 反馈优先于先前入队错误；ERROR 只保留给无法确认最终失能的清理。 */
	return confirmation_error == ESP_OK
			   ? ESP_OK
			   : first_error(command_error, confirmation_error);
}

/*
 * 把 STOP 的最终结果一次性写回状态机。成功发布 IDLE；失败发布 ERROR 并保留失败
 * 子步骤。preserved_error 用于记录触发清理的 START 错误，但不改变已确认的
 * IDLE。
 */
static void finalize_stop(esp_err_t stop_error, esp_err_t preserved_error,
						  bool count_explicit_stop) {
	const uint64_t timestamp_us = now_us();
	portENTER_CRITICAL(&s_state_lock);
	const exo_system_step_t failed_step = s_snapshot.active_step;
	s_snapshot.run_requested = false;
	s_snapshot.output_allowed = false;
	s_snapshot.active_step = EXO_SYSTEM_STEP_NONE;
	s_snapshot.last_completed_us = timestamp_us;
	if (stop_error == ESP_OK) {
		set_state_locked(EXO_SYSTEM_STATE_IDLE, timestamp_us);
		if (count_explicit_stop) {
			++s_snapshot.stop_success_count;
		}
		s_snapshot.last_error = preserved_error;
	} else {
		set_state_locked(EXO_SYSTEM_STATE_ERROR, timestamp_us);
		++s_snapshot.stop_failure_count;
		s_snapshot.last_error = stop_error;
		s_snapshot.last_failed_step = failed_step;
		s_snapshot.last_failure_us = timestamp_us;
		s_snapshot.start_block_reason = EXO_START_BLOCK_SYSTEM_ERROR;
	}
	portEXIT_CRITICAL(&s_state_lock);
}

/*
 * 执行器侧处理 START。operation mutex 覆盖整个慢速序列，保证 UART 维护和 STOP
 * 清理不会并行操作电机。arm 完成后仍须在状态锁内确认 STOP 没有抢先发布
 * STOPPING；只有 START 获胜才打开 output_allowed 并进入 RUNNING。
 */
static void handle_start(void) {
	if (xSemaphoreTake(s_operation_mutex, portMAX_DELAY) != pdTRUE) {
		return;
	}

	if (!start_still_requested()) {
		const esp_err_t stop_error = execute_stop_sequence();
		xSemaphoreGive(s_operation_mutex);
		finalize_stop(stop_error, ESP_OK, true);
		return;
	}

	const start_sequence_result_t result = execute_start_sequence();
	bool canceled = result.canceled;
	if (result.error == ESP_OK && !result.canceled) {
		const uint64_t timestamp_us = now_us();
		bool published = false;
		portENTER_CRITICAL(&s_state_lock);
		if (s_snapshot.state == EXO_SYSTEM_STATE_STARTING &&
			s_snapshot.run_requested) {
			s_snapshot.output_allowed = true;
			s_snapshot.active_step = EXO_SYSTEM_STEP_NONE;
			s_snapshot.start_block_reason = EXO_START_BLOCK_NONE;
			s_snapshot.last_error = ESP_OK;
			s_snapshot.last_completed_us = timestamp_us;
			++s_snapshot.start_success_count;
			set_state_locked(EXO_SYSTEM_STATE_RUNNING, timestamp_us);
			published = true;
		}
		portEXIT_CRITICAL(&s_state_lock);
		if (published) {
			xSemaphoreGive(s_operation_mutex);
			return;
		}
		/* arm 后、RUNNING 发布前由 STOP 抢到状态锁：本次 START 按取消收口。 */
		canceled = true;
	}

	const uint64_t timestamp_us = now_us();
	portENTER_CRITICAL(&s_state_lock);
	const exo_system_step_t failed_step = s_snapshot.active_step;
	/* 即使驱动调用同时返回错误，已先发布 STOPPING 的 STOP 仍是这次竞争的赢家。
	 */
	if (s_snapshot.state == EXO_SYSTEM_STATE_STOPPING) {
		canceled = true;
	}
	s_snapshot.output_allowed = false;
	s_snapshot.run_requested = false;
	if (s_snapshot.state != EXO_SYSTEM_STATE_STOPPING) {
		set_state_locked(EXO_SYSTEM_STATE_STOPPING, timestamp_us);
	}
	if (!canceled) {
		++s_snapshot.start_failure_count;
		s_snapshot.last_error = result.error;
		s_snapshot.last_failed_step = failed_step;
		s_snapshot.last_failure_us = timestamp_us;
		if (result.block_reason != EXO_START_BLOCK_NONE) {
			s_snapshot.start_block_reason = result.block_reason;
		}
	}
	portEXIT_CRITICAL(&s_state_lock);

	const esp_err_t stop_error = execute_stop_sequence();
	xSemaphoreGive(s_operation_mutex);
	finalize_stop(stop_error, canceled ? ESP_OK : result.error, canceled);
}

/* STOP 的执行器入口：独占慢速操作、完成清理，再统一发布 IDLE 或 ERROR。 */
static void handle_stop(void) {
	if (xSemaphoreTake(s_operation_mutex, portMAX_DELAY) != pdTRUE) {
		return;
	}
	const esp_err_t error = execute_stop_sequence();
	xSemaphoreGive(s_operation_mutex);
	finalize_stop(error, ESP_OK, true);
}

/*
 * 事件驱动执行器任务，不是周期任务。任务通知只负责把它从 portMAX_DELAY 唤醒；
 * 每次醒来都以锁内 state/run_requested 为准，并循环处理到稳定态。因此多个通知
 * 可以合并，START 期间到达的 STOP 也不会因没有传统消息队列而丢失。
 */
static void executor_task(void *arg) {
	(void)arg;
	const TaskHandle_t current_task = xTaskGetCurrentTaskHandle();
	portENTER_CRITICAL(&s_state_lock);
	s_snapshot.executor_task_alive = true;
	s_snapshot.executor_task_identity = (uintptr_t)current_task;
	portEXIT_CRITICAL(&s_state_lock);

	for (;;) {
		(void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		for (;;) {
			portENTER_CRITICAL(&s_state_lock);
			const exo_system_state_t state = s_snapshot.state;
			const bool run_requested = s_snapshot.run_requested;
			portEXIT_CRITICAL(&s_state_lock);

			if (state == EXO_SYSTEM_STATE_STARTING && run_requested) {
				handle_start();
				continue;
			}
			if (state == EXO_SYSTEM_STATE_STOPPING) {
				handle_stop();
				continue;
			}
			break;
		}
	}
}

/*
 * 初始化整机状态存储、静态 operation mutex 和静态执行器任务。此时保持 INIT，
 * 等 app_main 完成其他子系统后再显式 finish；初始化本身不发送电机命令。
 */
esp_err_t exo_system_init(void) {
	const uint64_t timestamp_us = now_us();
	portENTER_CRITICAL(&s_state_lock);
	if (s_initialized) {
		portEXIT_CRITICAL(&s_state_lock);
		return ESP_OK;
	}
	portEXIT_CRITICAL(&s_state_lock);

	SemaphoreHandle_t mutex =
		xSemaphoreCreateMutexStatic(&s_operation_mutex_buffer);
	if (mutex == NULL) {
		return ESP_ERR_NO_MEM;
	}

	portENTER_CRITICAL(&s_state_lock);
	memset(&s_snapshot, 0, sizeof(s_snapshot));
	s_snapshot.initialized = true;
	s_snapshot.state = EXO_SYSTEM_STATE_INIT;
	s_snapshot.active_step = EXO_SYSTEM_STEP_NONE;
	s_snapshot.start_block_reason = EXO_START_BLOCK_INITIALIZING;
	s_snapshot.last_request_result = EXO_REQUEST_NONE;
	s_snapshot.last_error = ESP_OK;
	s_snapshot.state_entered_us = timestamp_us;
	s_maintenance_generation = 0U;
	s_operation_mutex = mutex;
	s_initialized = true;
	portEXIT_CRITICAL(&s_state_lock);

	TaskHandle_t task = xTaskCreateStaticPinnedToCore(
		executor_task, "exo_executor", EXO_EXECUTOR_TASK_STACK_BYTES, NULL,
		EXO_EXECUTOR_TASK_PRIORITY, s_executor_task_stack,
		&s_executor_task_buffer, EXO_EXECUTOR_TASK_CORE);
	if (task == NULL) {
		portENTER_CRITICAL(&s_state_lock);
		s_initialized = false;
		s_operation_mutex = NULL;
		memset(&s_snapshot, 0, sizeof(s_snapshot));
		portEXIT_CRITICAL(&s_state_lock);
		return ESP_ERR_NO_MEM;
	}

	portENTER_CRITICAL(&s_state_lock);
	s_executor_task = task;
	s_snapshot.executor_task_identity = (uintptr_t)task;
	portEXIT_CRITICAL(&s_state_lock);
	return ESP_OK;
}

/* 仅允许 INIT -> IDLE；重复完成初始化是幂等的，其他状态调用视为编程错误。 */
esp_err_t exo_system_finish_initialization(void) {
	const uint64_t timestamp_us = now_us();
	portENTER_CRITICAL(&s_state_lock);
	if (!s_initialized) {
		portEXIT_CRITICAL(&s_state_lock);
		return ESP_ERR_INVALID_STATE;
	}
	if (s_snapshot.state == EXO_SYSTEM_STATE_IDLE) {
		portEXIT_CRITICAL(&s_state_lock);
		return ESP_OK;
	}
	if (s_snapshot.state != EXO_SYSTEM_STATE_INIT) {
		portEXIT_CRITICAL(&s_state_lock);
		return ESP_ERR_INVALID_STATE;
	}
	s_snapshot.run_requested = false;
	s_snapshot.output_allowed = false;
	s_snapshot.start_block_reason = EXO_START_BLOCK_NONE;
	set_state_locked(EXO_SYSTEM_STATE_IDLE, timestamp_us);
	portEXIT_CRITICAL(&s_state_lock);
	return ESP_OK;
}

/*
 * 调用者必须持有 s_state_lock。把当前状态转换为 START 的即时幂等/拒绝结果；
 * 返回 NONE 表示仍需继续做 mutex 和硬件前置检查，不是对外结果。
 */
static exo_request_result_t start_result_for_state_locked(void) {
	switch (s_snapshot.state) {
	case EXO_SYSTEM_STATE_INIT:
		s_snapshot.start_block_reason = EXO_START_BLOCK_INITIALIZING;
		return EXO_REQUEST_REJECTED_NOT_READY;
	case EXO_SYSTEM_STATE_STARTING:
		return EXO_REQUEST_ALREADY_PENDING;
	case EXO_SYSTEM_STATE_RUNNING:
		return EXO_REQUEST_ALREADY_SATISFIED;
	case EXO_SYSTEM_STATE_STOPPING:
		s_snapshot.start_block_reason = EXO_START_BLOCK_SYSTEM_NOT_IDLE;
		return EXO_REQUEST_REJECTED_BUSY;
	case EXO_SYSTEM_STATE_ERROR:
		s_snapshot.start_block_reason = EXO_START_BLOCK_SYSTEM_ERROR;
		return EXO_REQUEST_REJECTED_ERROR;
	case EXO_SYSTEM_STATE_IDLE:
	default:
		return EXO_REQUEST_NONE;
	}
}

/*
 * 非阻塞 START 请求入口。先做状态快检，再以零等待方式取得 operation mutex；持锁
 * 完成硬件前置检查后发布 STARTING，释放 mutex，最后通知执行器。两次状态检查用于
 * 封住快检与取得 mutex 之间的竞争窗口。返回 ACCEPTED 时尚未执行任何慢速步骤。
 */
exo_request_result_t exo_system_request_start(void) {
	const uint64_t request_timestamp_us = now_us();
	portENTER_CRITICAL(&s_state_lock);
	++s_snapshot.start_request_count;
	if (!s_initialized || s_executor_task == NULL) {
		record_request_locked(EXO_REQUEST_INTERNAL_ERROR, request_timestamp_us);
		portEXIT_CRITICAL(&s_state_lock);
		return EXO_REQUEST_INTERNAL_ERROR;
	}
	exo_request_result_t result = start_result_for_state_locked();
	if (result != EXO_REQUEST_NONE) {
		record_request_locked(result, request_timestamp_us);
		portEXIT_CRITICAL(&s_state_lock);
		return result;
	}
	portEXIT_CRITICAL(&s_state_lock);

	if (xSemaphoreTake(s_operation_mutex, 0) != pdTRUE) {
		portENTER_CRITICAL(&s_state_lock);
		result = start_result_for_state_locked();
		if (result == EXO_REQUEST_NONE) {
			result = EXO_REQUEST_REJECTED_BUSY;
			s_snapshot.start_block_reason = EXO_START_BLOCK_OPERATION_BUSY;
		}
		record_request_locked(result, request_timestamp_us);
		portEXIT_CRITICAL(&s_state_lock);
		return result;
	}

	portENTER_CRITICAL(&s_state_lock);
	result = start_result_for_state_locked();
	portEXIT_CRITICAL(&s_state_lock);
	if (result != EXO_REQUEST_NONE) {
		portENTER_CRITICAL(&s_state_lock);
		record_request_locked(result, request_timestamp_us);
		portEXIT_CRITICAL(&s_state_lock);
		xSemaphoreGive(s_operation_mutex);
		return result;
	}

	const exo_start_block_reason_t reason = hardware_start_block_reason();
	const uint64_t timestamp_us = now_us();
	portENTER_CRITICAL(&s_state_lock);
	if (reason != EXO_START_BLOCK_NONE) {
		s_snapshot.start_block_reason = reason;
		result = EXO_REQUEST_REJECTED_NOT_READY;
	} else {
		s_snapshot.run_requested = true;
		s_snapshot.output_allowed = false;
		s_snapshot.active_step = EXO_SYSTEM_STEP_NONE;
		s_snapshot.start_block_reason = EXO_START_BLOCK_NONE;
		set_state_locked(EXO_SYSTEM_STATE_STARTING, timestamp_us);
		result = EXO_REQUEST_ACCEPTED;
	}
	record_request_locked(result, request_timestamp_us);
	const TaskHandle_t task = s_executor_task;
	portEXIT_CRITICAL(&s_state_lock);
	xSemaphoreGive(s_operation_mutex);

	if (result == EXO_REQUEST_ACCEPTED) {
		xTaskNotifyGive(task);
	}
	return result;
}

/*
 * 非阻塞 STOP 请求入口。关键动作在状态锁内立即完成：撤销运行意图、关闭周期输出
 * 门并发布 STOPPING；解锁后才通知执行器做释力/失能。因而“STOP 已接受”不代表
 * 电机已 REST，但保证后续 100 Hz 周期不会再进入控制器。
 */
exo_request_result_t exo_system_request_stop(void) {
	const uint64_t timestamp_us = now_us();
	TaskHandle_t task = NULL;
	exo_request_result_t result;
	portENTER_CRITICAL(&s_state_lock);
	++s_snapshot.stop_request_count;
	if (!s_initialized || s_executor_task == NULL) {
		record_request_locked(EXO_REQUEST_INTERNAL_ERROR, timestamp_us);
		portEXIT_CRITICAL(&s_state_lock);
		return EXO_REQUEST_INTERNAL_ERROR;
	}

	/*
	 * STOP 的线性化点位于状态锁内：先清 run_requested/output_allowed 并发布
	 * STOPPING，再解锁通知执行器任务。这样与 START 竞争时，100 Hz 路径会先
	 * 看见输出门关闭，不必等待较慢的失能与 REST 反馈确认序列完成。
	 */
	switch (s_snapshot.state) {
	case EXO_SYSTEM_STATE_INIT:
	case EXO_SYSTEM_STATE_IDLE:
		result = EXO_REQUEST_ALREADY_SATISFIED;
		break;
	case EXO_SYSTEM_STATE_STOPPING:
		result = EXO_REQUEST_ALREADY_PENDING;
		break;
	case EXO_SYSTEM_STATE_STARTING:
		/* 即使执行器尚未开始 START 序列，也在同一线性化点记录取消。 */
		++s_snapshot.start_cancel_count;
		s_snapshot.run_requested = false;
		s_snapshot.output_allowed = false;
		set_state_locked(EXO_SYSTEM_STATE_STOPPING, timestamp_us);
		result = EXO_REQUEST_ACCEPTED;
		task = s_executor_task;
		break;
	case EXO_SYSTEM_STATE_RUNNING:
	case EXO_SYSTEM_STATE_ERROR:
		s_snapshot.run_requested = false;
		s_snapshot.output_allowed = false;
		set_state_locked(EXO_SYSTEM_STATE_STOPPING, timestamp_us);
		result = EXO_REQUEST_ACCEPTED;
		task = s_executor_task;
		break;
	default:
		result = EXO_REQUEST_INTERNAL_ERROR;
		break;
	}
	record_request_locked(result, timestamp_us);
	portEXIT_CRITICAL(&s_state_lock);

	if (task != NULL) {
		xTaskNotifyGive(task);
	}
	return result;
}

/*
 * 100 Hz 路径的最终门。只在短临界区内复制三个布尔条件，随后解锁再调用控制器；
 * 绝不能在 s_state_lock 内做正弦计算或 CAN 发送。门关闭时返回 ESP_OK
 * 表示“本周期 按状态无需输出”，不是漏执行或错误。
 */
esp_err_t exo_system_process_control_tick(int64_t now) {
	portENTER_CRITICAL(&s_state_lock);
	const bool output_allowed = s_snapshot.state == EXO_SYSTEM_STATE_RUNNING &&
								s_snapshot.run_requested &&
								s_snapshot.output_allowed;
	portEXIT_CRITICAL(&s_state_lock);
	return output_allowed ? exo_controller_process_tick(now) : ESP_OK;
}

/*
 * 生成供 UART/BLE 使用的一致快照。先在锁内复制纯状态，再在锁外执行较慢的硬件
 * 就绪检查，最后用 transition_count 和 maintenance_generation 验证检查期间状态
 * 未变化；若发生竞争，就重新复制最新状态并保守地报告尚不可启动。
 */
bool exo_system_get_snapshot(exo_system_snapshot_t *snapshot) {
	if (snapshot == NULL) {
		return false;
	}

	portENTER_CRITICAL(&s_state_lock);
	if (!s_initialized) {
		portEXIT_CRITICAL(&s_state_lock);
		memset(snapshot, 0, sizeof(*snapshot));
		return false;
	}
	*snapshot = s_snapshot;
	const uint32_t maintenance_generation = s_maintenance_generation;
	portEXIT_CRITICAL(&s_state_lock);

	snapshot->ready_to_start = false;
	if (snapshot->state == EXO_SYSTEM_STATE_INIT) {
		snapshot->start_block_reason = EXO_START_BLOCK_INITIALIZING;
		return true;
	}
	if (snapshot->state == EXO_SYSTEM_STATE_ERROR) {
		snapshot->start_block_reason = EXO_START_BLOCK_SYSTEM_ERROR;
		return true;
	}
	if (snapshot->state != EXO_SYSTEM_STATE_IDLE) {
		snapshot->start_block_reason = EXO_START_BLOCK_SYSTEM_NOT_IDLE;
		return true;
	}
	if (snapshot->maintenance_active) {
		snapshot->start_block_reason = EXO_START_BLOCK_OPERATION_BUSY;
		return true;
	}
	const exo_start_block_reason_t reason = hardware_start_block_reason();

	portENTER_CRITICAL(&s_state_lock);
	const bool unchanged =
		s_snapshot.state == EXO_SYSTEM_STATE_IDLE &&
		s_snapshot.transition_count == snapshot->transition_count &&
		!s_snapshot.run_requested && !s_snapshot.output_allowed &&
		!s_snapshot.maintenance_active &&
		s_maintenance_generation == maintenance_generation;
	portEXIT_CRITICAL(&s_state_lock);
	if (unchanged) {
		snapshot->start_block_reason = reason;
		snapshot->ready_to_start = reason == EXO_START_BLOCK_NONE;
	} else {
		/* 不拼接竞争前的 IDLE
		 * 与竞争后的硬件字段；改为返回一份更新且保守的快照。 */
		portENTER_CRITICAL(&s_state_lock);
		*snapshot = s_snapshot;
		portEXIT_CRITICAL(&s_state_lock);
		snapshot->ready_to_start = false;
		if (snapshot->state == EXO_SYSTEM_STATE_INIT) {
			snapshot->start_block_reason = EXO_START_BLOCK_INITIALIZING;
		} else if (snapshot->state == EXO_SYSTEM_STATE_ERROR) {
			snapshot->start_block_reason = EXO_START_BLOCK_SYSTEM_ERROR;
		} else if (snapshot->state != EXO_SYSTEM_STATE_IDLE) {
			snapshot->start_block_reason = EXO_START_BLOCK_SYSTEM_NOT_IDLE;
		} else {
			snapshot->start_block_reason = EXO_START_BLOCK_OPERATION_BUSY;
		}
	}
	return true;
}

/*
 * UART 原始维护命令的入口门。以零等待取得 operation mutex，所以不会卡住命令
 * 分发任务；只有稳定 IDLE 且输出门关闭时才发布 maintenance_active。成功后 mutex
 * 继续由调用者持有，直到配对调用 end_maintenance()。
 */
bool exo_system_try_begin_maintenance(void) {
	if (s_operation_mutex == NULL ||
		xSemaphoreTake(s_operation_mutex, 0) != pdTRUE) {
		return false;
	}

	portENTER_CRITICAL(&s_state_lock);
	const bool allowed =
		s_initialized && s_snapshot.state == EXO_SYSTEM_STATE_IDLE &&
		!s_snapshot.run_requested && !s_snapshot.output_allowed;
	if (allowed) {
		s_snapshot.maintenance_active = true;
		++s_maintenance_generation;
	}
	portEXIT_CRITICAL(&s_state_lock);
	if (!allowed) {
		xSemaphoreGive(s_operation_mutex);
	}
	return allowed;
}

/* 清除维护标志、递增代数并释放 operation mutex；只能与成功的 begin 配对使用。
 */
void exo_system_end_maintenance(void) {
	if (s_operation_mutex != NULL) {
		portENTER_CRITICAL(&s_state_lock);
		s_snapshot.maintenance_active = false;
		++s_maintenance_generation;
		portEXIT_CRITICAL(&s_state_lock);
		(void)xSemaphoreGive(s_operation_mutex);
	}
}

/* 以下名称函数只做枚举到稳定文本的转换，不读取或改变状态机。 */
const char *exo_system_state_name(exo_system_state_t state) {
	switch (state) {
	case EXO_SYSTEM_STATE_INIT:
		return "INIT";
	case EXO_SYSTEM_STATE_IDLE:
		return "IDLE";
	case EXO_SYSTEM_STATE_STARTING:
		return "STARTING";
	case EXO_SYSTEM_STATE_RUNNING:
		return "RUNNING";
	case EXO_SYSTEM_STATE_STOPPING:
		return "STOPPING";
	case EXO_SYSTEM_STATE_ERROR:
		return "ERROR";
	default:
		return "UNKNOWN";
	}
}

const char *exo_system_step_name(exo_system_step_t step) {
	switch (step) {
	case EXO_SYSTEM_STEP_NONE:
		return "NONE";
	case EXO_SYSTEM_STEP_CHECK_PRECONDITIONS:
		return "CHECK_PRECONDITIONS";
	case EXO_SYSTEM_STEP_DISABLE_MOTORS:
		return "DISABLE_MOTORS";
	case EXO_SYSTEM_STEP_SET_MOTION_MODE:
		return "SET_MOTION_MODE";
	case EXO_SYSTEM_STEP_ENABLE_MOTORS:
		return "ENABLE_MOTORS";
	case EXO_SYSTEM_STEP_SEND_ZERO_OUTPUT:
		return "SEND_ZERO_OUTPUT";
	case EXO_SYSTEM_STEP_WAIT_FRESH_FEEDBACK:
		return "WAIT_FRESH_FEEDBACK";
	case EXO_SYSTEM_STEP_ARM_CONTROLLER:
		return "ARM_CONTROLLER";
	case EXO_SYSTEM_STEP_DISARM_CONTROLLER:
		return "DISARM_CONTROLLER";
	case EXO_SYSTEM_STEP_RELEASE_OUTPUT:
		return "RELEASE_OUTPUT";
	case EXO_SYSTEM_STEP_VERIFY_DISABLED:
		return "VERIFY_DISABLED";
	default:
		return "UNKNOWN";
	}
}

const char *
exo_system_start_block_reason_name(exo_start_block_reason_t reason) {
	switch (reason) {
	case EXO_START_BLOCK_NONE:
		return "NONE";
	case EXO_START_BLOCK_INITIALIZING:
		return "INITIALIZING";
	case EXO_START_BLOCK_SYSTEM_NOT_IDLE:
		return "SYSTEM_NOT_IDLE";
	case EXO_START_BLOCK_OPERATION_BUSY:
		return "OPERATION_BUSY";
	case EXO_START_BLOCK_CONTROL_TASK_NOT_READY:
		return "CONTROL_TASK_NOT_READY";
	case EXO_START_BLOCK_TWAI_NOT_RUNNING:
		return "TWAI_NOT_RUNNING";
	case EXO_START_BLOCK_LEFT_MAPPING_INVALID:
		return "LEFT_MAPPING_INVALID";
	case EXO_START_BLOCK_RIGHT_MAPPING_INVALID:
		return "RIGHT_MAPPING_INVALID";
	case EXO_START_BLOCK_LEFT_DIRECTION_INVALID:
		return "LEFT_DIRECTION_INVALID";
	case EXO_START_BLOCK_RIGHT_DIRECTION_INVALID:
		return "RIGHT_DIRECTION_INVALID";
	case EXO_START_BLOCK_LEFT_ZERO_INVALID:
		return "LEFT_ZERO_INVALID";
	case EXO_START_BLOCK_RIGHT_ZERO_INVALID:
		return "RIGHT_ZERO_INVALID";
	case EXO_START_BLOCK_LEFT_FEEDBACK_INVALID:
		return "LEFT_FEEDBACK_INVALID";
	case EXO_START_BLOCK_RIGHT_FEEDBACK_INVALID:
		return "RIGHT_FEEDBACK_INVALID";
	case EXO_START_BLOCK_LEFT_OFFLINE:
		return "LEFT_OFFLINE";
	case EXO_START_BLOCK_RIGHT_OFFLINE:
		return "RIGHT_OFFLINE";
	case EXO_START_BLOCK_LEFT_MOTOR_FAULT:
		return "LEFT_MOTOR_FAULT";
	case EXO_START_BLOCK_RIGHT_MOTOR_FAULT:
		return "RIGHT_MOTOR_FAULT";
	case EXO_START_BLOCK_SYSTEM_ERROR:
		return "SYSTEM_ERROR";
	default:
		return "UNKNOWN";
	}
}

const char *exo_system_request_result_name(exo_request_result_t result) {
	switch (result) {
	case EXO_REQUEST_NONE:
		return "NONE";
	case EXO_REQUEST_ACCEPTED:
		return "ACCEPTED";
	case EXO_REQUEST_ALREADY_SATISFIED:
		return "ALREADY_SATISFIED";
	case EXO_REQUEST_ALREADY_PENDING:
		return "ALREADY_PENDING";
	case EXO_REQUEST_REJECTED_NOT_READY:
		return "REJECTED_NOT_READY";
	case EXO_REQUEST_REJECTED_BUSY:
		return "REJECTED_BUSY";
	case EXO_REQUEST_REJECTED_ERROR:
		return "REJECTED_ERROR";
	case EXO_REQUEST_INTERNAL_ERROR:
		return "INTERNAL_ERROR";
	default:
		return "UNKNOWN";
	}
}
