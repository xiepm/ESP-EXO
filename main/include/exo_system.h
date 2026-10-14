#ifndef EXO_SYSTEM_H
#define EXO_SYSTEM_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "exo_controller.h"
#include "exo_sensor_snapshot.h"

/*
 * 整机状态机的公开接口。
 *
 * 手机、UART 等调用者只负责“提出 START/STOP 请求”；真正会等待、发送 CAN 和
 * 检查反馈的硬件序列由后台执行器任务完成。唯一 100 Hz 控制任务则通过
 * exo_system_process_control_tick() 查询输出门。三者职责分开，避免 BLE/UART
 * 回调被慢速电机操作阻塞，也避免出现第二个周期控制源。
 *
 * 常见误区：
 * 1. EXO_REQUEST_ACCEPTED 不等于已经 RUNNING/IDLE，调用者仍要轮询状态快照；
 * 2. run_requested 表示用户意图，output_allowed 才是周期控制能否发帧的最终门；
 * 3. 电机“已使能”不等于系统 RUNNING，START 还必须完成新鲜反馈和控制器 arm；
 * 4. ERROR 不是另一种运行模式，STOP 会尝试执行清理并返回 IDLE。
 */

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 六态含义和主路径：INIT -> IDLE -> STARTING -> RUNNING -> STOPPING -> IDLE。
 *
 * INIT：对象和任务仍在初始化，拒绝 START；
 * IDLE：不允许周期运动输出，可接受 START 或工程维护命令；
 * STARTING：后台正在执行失能、切模式、使能、反馈确认和控制器 arm；
 * RUNNING：START 全部成功，且 100 Hz 输出门已经打开；
 * STOPPING：输出门已先关闭，后台继续释力、失能并确认 REST 反馈；
 * ERROR：失能状态无法确认，或运行故障清理后保持锁定；只能明确请求STOP尝试清理。
 */
typedef enum {
	EXO_SYSTEM_STATE_INIT = 0,
	EXO_SYSTEM_STATE_IDLE,
	EXO_SYSTEM_STATE_STARTING,
	EXO_SYSTEM_STATE_RUNNING,
	EXO_SYSTEM_STATE_STOPPING,
	EXO_SYSTEM_STATE_ERROR,
} exo_system_state_t;

/*
 * 100 Hz 控制链和同周期遥测只需要下面三个字段，不需要为了读取系统状态而构造
 * 完整 exo_system_snapshot_t。尤其不能在实时周期里调用会继续检查
 * TWAI、关节映射、 零位和电机反馈的重型就绪判断。
 *
 * 这里保留 run_requested 与 output_allowed
 * 的原始值，而不是预先合并成一个布尔量，
 * 这样日志可以区分“用户没有请求运行”和“用户请求运行但输出门尚未打开”。
 */
typedef struct {
	exo_system_state_t state;
	bool run_requested;
	bool output_allowed;
} exo_system_control_state_t;

/*
 * STARTING/STOPPING 内部正在执行的硬件子步骤，用于日志和手机诊断。
 * 它不是第七种系统状态；稳定态或没有序列运行时为 NONE。
 */
typedef enum {
	EXO_SYSTEM_STEP_NONE = 0,
	EXO_SYSTEM_STEP_CHECK_PRECONDITIONS,
	EXO_SYSTEM_STEP_DISABLE_MOTORS,
	EXO_SYSTEM_STEP_SET_MOTION_MODE,
	EXO_SYSTEM_STEP_ENABLE_MOTORS,
	EXO_SYSTEM_STEP_SEND_ZERO_OUTPUT,
	EXO_SYSTEM_STEP_WAIT_FRESH_FEEDBACK,
	EXO_SYSTEM_STEP_ARM_CONTROLLER,
	EXO_SYSTEM_STEP_DISARM_CONTROLLER,
	EXO_SYSTEM_STEP_RELEASE_OUTPUT,
	EXO_SYSTEM_STEP_VERIFY_DISABLED,
} exo_system_step_t;

/*
 * START 就绪检查的首个失败原因。左右映射、方向、零位、反馈和电机故障分别报告，
 * 便于上位机给出可操作提示，而不是只显示一个笼统的“不能启动”。
 */
typedef enum {
	EXO_START_BLOCK_NONE = 0,
	EXO_START_BLOCK_INITIALIZING,
	EXO_START_BLOCK_SYSTEM_NOT_IDLE,
	EXO_START_BLOCK_OPERATION_BUSY,
	EXO_START_BLOCK_CONTROL_TASK_NOT_READY,
	EXO_START_BLOCK_TWAI_NOT_RUNNING,
	EXO_START_BLOCK_LEFT_MAPPING_INVALID,
	EXO_START_BLOCK_RIGHT_MAPPING_INVALID,
	EXO_START_BLOCK_LEFT_DIRECTION_INVALID,
	EXO_START_BLOCK_RIGHT_DIRECTION_INVALID,
	EXO_START_BLOCK_LEFT_ZERO_INVALID,
	EXO_START_BLOCK_RIGHT_ZERO_INVALID,
	EXO_START_BLOCK_LEFT_FEEDBACK_INVALID,
	EXO_START_BLOCK_RIGHT_FEEDBACK_INVALID,
	EXO_START_BLOCK_LEFT_OFFLINE,
	EXO_START_BLOCK_RIGHT_OFFLINE,
	EXO_START_BLOCK_LEFT_MOTOR_FAULT,
	EXO_START_BLOCK_RIGHT_MOTOR_FAULT,
	EXO_START_BLOCK_SYSTEM_ERROR,
	/* 控制器参考源与高层助力执行状态不能组成获准的运行合同。 */
	EXO_START_BLOCK_RUN_CONTRACT_INVALID,
} exo_start_block_reason_t;

/*
 * 请求接口的即时结果。ACCEPTED/ALREADY_* 都不是硬件序列错误；真正的执行结果
 * 通过 state、last_error、last_failed_step 和完成/失败计数异步观察。
 */
typedef enum {
	EXO_REQUEST_NONE = 0,
	EXO_REQUEST_ACCEPTED,
	EXO_REQUEST_ALREADY_SATISFIED,
	EXO_REQUEST_ALREADY_PENDING,
	EXO_REQUEST_REJECTED_NOT_READY,
	EXO_REQUEST_REJECTED_BUSY,
	EXO_REQUEST_REJECTED_ERROR,
	EXO_REQUEST_INTERNAL_ERROR,
} exo_request_result_t;

/* 运行期首个停机原因，属于内部诊断枚举，不直接作为BLE线码发送。 */
typedef enum {
	EXO_RUNTIME_FAULT_NONE = 0,
	EXO_RUNTIME_FAULT_CAN_STATUS_UNAVAILABLE,
	EXO_RUNTIME_FAULT_CAN_NOT_RUNNING,
	EXO_RUNTIME_FAULT_CAN_ERROR_PASSIVE,
	EXO_RUNTIME_FAULT_SENSOR_CYCLE_INVALID,
	EXO_RUNTIME_FAULT_LEFT_FEEDBACK_INVALID,
	EXO_RUNTIME_FAULT_RIGHT_FEEDBACK_INVALID,
	EXO_RUNTIME_FAULT_LEFT_MOTOR,
	EXO_RUNTIME_FAULT_RIGHT_MOTOR,
	EXO_RUNTIME_FAULT_CONTROL_OUTPUT,
} exo_runtime_fault_t;

/*
 * 面向 UART/BLE 的整机一致快照。结构体是“某一时刻的副本”，调用者可以在返回后
 * 无锁读取；它不会随着后台任务自动更新，需要再次调用 get_snapshot()。
 */
typedef struct {
	/* 当前状态与启动门；ready_to_start 是每次读取快照时重新推导的结果。 */
	bool initialized;
	exo_system_state_t state;
	exo_system_step_t active_step;
	bool ready_to_start;
	exo_start_block_reason_t start_block_reason;
	/* run_requested 是目标状态；output_allowed 才是 100 Hz 路径最终输出门。 */
	bool run_requested;
	bool output_allowed;
	bool maintenance_active;

	bool executor_task_alive;
	uintptr_t executor_task_identity;

	/* 以下计数器自本次上电累计，用于验证异步请求没有丢失或重复执行。 */
	uint32_t transition_count;
	uint32_t start_request_count;
	uint32_t stop_request_count;
	uint32_t start_success_count;
	uint32_t start_failure_count;
	uint32_t start_cancel_count;
	uint32_t stop_success_count;
	uint32_t stop_failure_count;
	uint32_t rejected_request_count;

	/* 最近一次请求/失败以及关键事件时间；所有时间戳均为上电后的单调 us。 */
	exo_request_result_t last_request_result;
	esp_err_t last_error;
	exo_system_step_t last_failed_step;
	uint64_t state_entered_us;
	uint64_t last_request_us;
	uint64_t last_failure_us;
	uint64_t last_completed_us;
	/* 历史首因与STOP清理失败分开保存，VERIFY_DISABLED不能覆盖原始CAN原因。 */
	uint32_t runtime_fault_count;
	exo_runtime_fault_t last_runtime_fault;
	esp_err_t runtime_fault_error;
	uint64_t runtime_fault_us;
	bool runtime_can_status_valid;
	uint32_t runtime_can_state;
	uint32_t runtime_can_tx_errors;
	uint32_t runtime_can_rx_errors;
	uint32_t runtime_left_age_us;
	uint32_t runtime_right_age_us;
} exo_system_snapshot_t;

/*
 * 创建互斥量和静态后台执行器任务；初始化完成前系统保持 INIT。本函数不会创建
 * 100 Hz 控制任务，也不会使能电机。
 */
esp_err_t exo_system_init(void);

/* 所有子系统就绪后，由 app_main 最后一次调用以发布 IDLE。 */
esp_err_t exo_system_finish_initialization(void);

/*
 * 请求接口非阻塞，不发送 CAN、不延时、不等待反馈。START 会同步检查前置条件并
 * 发布 STARTING；STOP 会立即撤销周期输出许可，硬件序列仍由执行器异步完成。
 * ACCEPTED 只表示请求已接收，不表示序列完成；手机应继续查询 SYSTEM_STATUS。
 */
exo_request_result_t exo_system_request_start(void);
exo_request_result_t exo_system_request_stop(void);

/*
 * 产品唯一周期控制输出门，只允许永久100 Hz控制任务调用。运行时在发送前检查
 * CAN短快照和本周期统一双髋输入；首个故障关闭门并通知现有执行器，禁止自动重启。
 * 仅当state、run_requested、output_allowed同时允许时才进入控制器；否则无I/O返回。
 * START/STOP 序列中的零输出和释力帧属于生命周期操作，不由此周期接口产生。
 */
esp_err_t exo_system_process_control_tick(
	int64_t now_us, const exo_sensor_snapshot_t *sensor_snapshot,
	const exo_assist_result_t *assist_result,
	uint32_t expected_sensor_sequence, uint32_t expected_gait_sequence,
	exo_controller_tick_result_t *tick_result);

/*
 * 面向 100 Hz 控制链的轻量只读接口。函数只在一个很短的临界区内复制 state、
 * run_requested 和 output_allowed，不读取硬件，也不调用
 * hardware_start_block_reason()。 系统尚未初始化或参数为空时返回
 * false；失败时输出结构被清零（state 即 INIT）。
 */
bool exo_system_get_control_state(exo_system_control_state_t *control_state);

/*
 * 在短临界区内复制一致快照，再在锁外读取硬件就绪条件；若期间发生状态竞争，
 * 返回更新后的保守快照并保持 ready_to_start=false，不拼接新旧两次状态。
 */
bool exo_system_get_snapshot(exo_system_snapshot_t *snapshot);

/*
 * 串行化 UART 维护命令与 START/STOP 硬件序列。这里使用可跨函数持有的互斥量，
 * 不是禁止调度的临界区。begin 成功后必须配对调用 end；RUNNING 时会被拒绝。
 */
bool exo_system_try_begin_maintenance(void);
void exo_system_end_maintenance(void);

/* 枚举到稳定英文协议/日志名称的只读转换；未知值统一返回 "UNKNOWN"。 */
const char *exo_system_state_name(exo_system_state_t state);
const char *exo_system_runtime_fault_name(exo_runtime_fault_t fault);
const char *exo_system_step_name(exo_system_step_t step);
const char *exo_system_start_block_reason_name(exo_start_block_reason_t reason);
const char *exo_system_request_result_name(exo_request_result_t result);

#ifdef __cplusplus
}
#endif

#endif /* EXO_SYSTEM_H */
