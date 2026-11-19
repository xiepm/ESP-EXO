#ifndef EXO_CONTROLLER_H
#define EXO_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "exo_assist.h"
#include "exo_types.h"
#include "exo_bench_peak.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 阶段 8 通用髋关节控制器
 * ======================
 *
 * 本模块只负责把“选中的基础模式 + 本地参考源”变成 LEFT/RIGHT 五元关节命令。
 * 它不创建任务，不切换 RS01 的 PP/CSP/电流模式，也不拥有整机 START/STOP 状态。
 * 唯一 100 Hz 任务仍经 exo_system 的输出门调用 process_tick()；最终坐标换算和
 * A/B 发送统一交给 exo_joint_command 模块。
 */

/* 产品控制模式与 RS01 底层 control mode 是两个概念；六种模式都使用 Motion 帧。
 */
typedef enum {
	/* NONE 只表示当前没有已 arm 的活动控制器，不能作为用户选中模式。 */
	EXO_CONTROL_MODE_NONE = 0,
	EXO_CONTROL_MODE_HOLD = 1,
	EXO_CONTROL_MODE_POSITION = 2,
	EXO_CONTROL_MODE_VELOCITY = 3,
	EXO_CONTROL_MODE_TORQUE = 4,
	EXO_CONTROL_MODE_DAMPING = 5,
	EXO_CONTROL_MODE_IMPEDANCE = 6,
} exo_control_mode_t;

/*
 * 参考源只决定五元目标如何随时间变化；SINE 不是第七种电机模式。
 * ALTERNATING_SINE_TEST 是编译期开关控制的工程验收激励，不属于产品助力策略。
 */
typedef enum {
	EXO_REFERENCE_CONSTANT = 0,
	EXO_REFERENCE_SINE = 1,
	EXO_REFERENCE_ALTERNATING_SINE_TEST = 2,
    EXO_REFERENCE_BENCH_PEAK = 3,
} exo_reference_source_t;

esp_err_t exo_controller_prepare_bench_peak(unsigned side,float signed_peak_nm);
bool exo_controller_get_bench_peak(exo_bench_peak_t *status);
#define EXO_BENCH_TRACE_CAPACITY 384U
typedef struct {
    int64_t timestamp_us;
    uint32_t sequence, fault;
    uint64_t source_us[2];
    float q[2], v[2], estimate[2], command[2];
    uint32_t age_us[2];
    bool valid[2], sent[2];
} exo_bench_trace_row_t;
/* Reads are refused while armed/ticking. Trace freezes at completion/fault. */
bool exo_controller_get_bench_trace(unsigned index,exo_bench_trace_row_t *row,
    unsigned *count,unsigned *dropped);

/* START 在新鲜反馈通过后锁存的左右人体髋关节起点，单位均为 deg。 */
typedef struct {
	float left_deg;
	float right_deg;
} exo_controller_centers_t;

/*
 * IDLE 中可修改的完整控制选择。START 会一次性复制为 active_selection；RUNNING
 * 期间修改会被拒绝，因而不需要配置 revision、运行中双缓冲或热切换状态机。
 */
typedef struct {
	exo_control_mode_t mode;
	exo_reference_source_t reference;
	exo_joint_command_pair_t command;
} exo_controller_selection_t;

/* START 时与底层 selected 一起锁存的高层执行合同。 */
typedef struct {
	exo_assist_execution_state_t execution_state;
	uint32_t activation_epoch;
    /* 与执行状态一起由START锁存，最终合成器独立检查配置峰值。
     * 所有调用者都必须提供实际策略/峰值，不允许缺省绕过本轮边界。 */
    exo_assist_strategy_id_t strategy_id;
    uint16_t maximum_assist_torque_mnm;
} exo_controller_assist_contract_t;

/*
 * 本周期为什么没有接纳助力。它是内部诊断枚举，不是 BLE 稳定线码；遥测适配层
 * 若要发送必须逐值映射，不能直接 memcpy。
 */
typedef enum {
	EXO_CONTROLLER_ASSIST_REJECT_NONE = 0,
	EXO_CONTROLLER_ASSIST_REJECT_NOT_REQUESTED,
	EXO_CONTROLLER_ASSIST_REJECT_SHADOW_ONLY,
	EXO_CONTROLLER_ASSIST_REJECT_RESULT_INVALID,
	EXO_CONTROLLER_ASSIST_REJECT_EXECUTION_MISMATCH,
	EXO_CONTROLLER_ASSIST_REJECT_EPOCH_MISMATCH,
	EXO_CONTROLLER_ASSIST_REJECT_SENSOR_SEQUENCE_MISMATCH,
	EXO_CONTROLLER_ASSIST_REJECT_GAIT_SEQUENCE_MISMATCH,
	EXO_CONTROLLER_ASSIST_REJECT_TIMESTAMP_MISMATCH,
	EXO_CONTROLLER_ASSIST_REJECT_OUTPUT_KIND,
	EXO_CONTROLLER_ASSIST_REJECT_COMPOSE_INVALID,
} exo_controller_assist_reject_reason_t;

/*
 * 单次 100 Hz 输出的纯值结果。控制任务在最终发送结果已知后得到这份结构，可直接
 * 交给遥测采样器；它不指向控制器内部内存，也不会随下一周期自动变化。
 */
typedef struct {
	bool output_gate_open;
	bool base_generated;
	bool assist_suggested;
	bool assist_accepted;
	bool assist_applied;
	bool send_attempted;
	bool send_succeeded;
	/*
	 * CAN 是两帧事务，以下标志记录每个逻辑执行器是否真实入队成功。正常路径两者
	 * 应同时为 true；只有一个为 true 表示部分发送，必须进入错误统计而不能伪装成
	 * “整对均未输出”。LEFT/RIGHT 的对应关系由硬件配置在遥测边界显式换算。
	 */
	bool actuator_a_send_succeeded;
	bool actuator_b_send_succeeded;
	bool send_partially_succeeded;
	exo_controller_assist_reject_reason_t assist_reject_reason;
	exo_joint_command_pair_t base_command;
	/* 只有 torque_ff_nm 可非零；SHADOW 也会保留建议供遥测对照。 */
	exo_joint_command_pair_t assist_delta;
	exo_joint_command_pair_t final_command;
	esp_err_t send_result;
} exo_controller_tick_result_t;

esp_err_t exo_controller_process_sensor_tick(int64_t now_us,
    const exo_sensor_snapshot_t *sensors,const exo_assist_result_t *assist_result,
    uint32_t expected_sensor_sequence,uint32_t expected_gait_sequence,
    exo_controller_tick_result_t *tick_result);

/* UART/BLE 只读诊断快照；它不是连续遥测，也不是 BLE 线格式。 */
typedef struct {
	bool initialized;
	bool armed;
	bool tick_active;
	exo_controller_selection_t selected;
	exo_controller_selection_t active;
	exo_controller_centers_t centers;
	exo_joint_command_pair_t last_output;
	exo_controller_assist_contract_t assist_contract;
	exo_controller_tick_result_t last_tick_result;
	uint32_t selected_generation;
	uint32_t active_generation;
	uint64_t output_count;
	uint64_t rejected_config_count;
	uint64_t output_error_count;
	uint64_t partial_send_count;
	uint64_t assist_accepted_count;
	uint64_t assist_applied_count;
	uint64_t assist_rejected_count;
	int64_t start_us;
	int64_t last_output_us;
	esp_err_t last_error;
} exo_controller_snapshot_t;

/*
 * 建立产品默认选择：DAMPING + CONSTANT，左右五元均为
 * q/dq/tau/kp=0、kd=0.1。初始化只写软件状态，不使能电机、不发送 CAN。
 */
esp_err_t exo_controller_init(void);

/*
 * 原子修改 selected 模式和参考源。两种正弦参考只允许搭配 IMPEDANCE；NONE
 * 不能被选中。工程参考未编入时返回 ESP_ERR_NOT_SUPPORTED；armed/tick_active
 * 时返回 ESP_ERR_INVALID_STATE。
 */
esp_err_t exo_controller_set_mode(exo_control_mode_t mode,
								  exo_reference_source_t reference);

/* 原子修改一侧 selected 五元参数；另一侧保持不变。只允许未 arm 时调用。 */
esp_err_t exo_controller_set_joint_command(exo_hip_joint_id_t joint,
										   const exo_joint_command_t *command);

/*
 * START 前的纯只读运行合同预检。它只复制 selected 并核对高层助力的有效执行
 * 状态，不 arm 控制器、不锁配置、不读取硬件、不发送 CAN。工程交替激励只允许
 * DISABLED/SHADOW；与 ACTIVE 组合时返回 ESP_ERR_NOT_SUPPORTED。
 */
esp_err_t exo_controller_validate_run_contract(
	const exo_assist_run_contract_t *assist_contract);

/* 复制 selected/active、最近输出和计数器；读取不会改变控制状态。 */
bool exo_controller_get_snapshot(exo_controller_snapshot_t *snapshot);

/*
 * 纯计算入口，也是控制器与主机向量测试共用的唯一模式定义。elapsed_us<=0 视为
 * START 时刻。CONSTANT 模式使用同一五次启动渐入；SINE 复用阶段 7 参考生成器。
 * 本函数不读锁、不发 CAN、不调用 RTOS。
 */
bool exo_controller_evaluate(const exo_controller_selection_t *selection,
							 const exo_controller_centers_t *centers,
							 int64_t elapsed_us,
							 exo_joint_command_pair_t *output);

/* START 生命周期入口：锁存 selected、左右起点和时间，但不使能电机、不发 CAN。
 */
esp_err_t
exo_controller_arm(const exo_controller_centers_t *centers, int64_t start_us,
				   const exo_controller_assist_contract_t *assist_contract);

/*
 * 由唯一 100 Hz 任务调用：先生成基础五元命令，再核对本周期 assist result，最后
 * 只调用一次统一关节命令出口。expected_* 必须来自同一周期的 sensor/gait。
 */
esp_err_t exo_controller_process_tick(
	int64_t now_us, const exo_assist_result_t *assist_result,
	uint32_t expected_sensor_sequence, uint32_t expected_gait_sequence,
	exo_controller_tick_result_t *tick_result);

/*
 * STOP 生命周期入口：阻止新 tick，并有界等待已经进入的一个 tick 完整退出。
 * 超时返回 ESP_ERR_TIMEOUT；整机状态机仍须继续释力、失能和 REST 确认。
 */
esp_err_t exo_controller_disarm(void);

const char *exo_control_mode_name(exo_control_mode_t mode);
const char *exo_reference_source_name(exo_reference_source_t reference);

#ifdef __cplusplus
}
#endif

#endif /* EXO_CONTROLLER_H */
