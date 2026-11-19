#ifndef EXO_ASSIST_H
#define EXO_ASSIST_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "exo_sensor_snapshot.h"
#include "gait_estimator.h"
#include "exo_swing_pulse.h"
#include "exo_adaptive_deployment.h"

/*
 * 阶段 9/10 高层助力策略框架
 * ========================
 *
 * “主动助力/按需辅助”是产品策略；“力矩/阻抗/位置”是阶段 8 已有的底层五元命令
 * 执行方式。二者不能放进同一个枚举。策略只读取传感快照和步态估计，并提出一对
 * exo_joint_command_t；它无权切换系统状态、发送 BLE 或调用 RS01/CAN。
 *
 * ESP32 不支持运行时装载动态插件。这里的“可替换”指编译期静态策略描述表、固定
 * 工作区和版本化接口。阶段10实现透明预览、低幅 PHASE_PROFILE 和单向角度阈值；只有 ACTIVE
 * 结果通过同周期合同并由唯一最终出口发送成功，才会记录真实助力输出。
 */

#ifdef __cplusplus
extern "C" {
#endif

#define EXO_ASSIST_CONFIG_SCHEMA_VERSION 1U
/* 双侧事件/周期历史共用固定256-byte预算（不是新增任务或动态框架）。 */
#define EXO_ASSIST_POLICY_WORKSPACE_BYTES 256U
#define EXO_ASSIST_ANGLE_MIN_CDEG 100U
#define EXO_ASSIST_ANGLE_MAX_CDEG 6000U
#define EXO_ASSIST_ANGLE_DEFAULT_CDEG 1000U
#define EXO_ASSIST_ANGLE_HYSTERESIS_CDEG 100U
#define EXO_ASSIST_ANGLE_RAMP_US 200000U
/* 事件脉冲策略的可配置申请峰值，15000 mN·m = 15 N·m。
 * 相位工程曲线继续使用其独立的500 mN·m边界；默认参数不自动升高。 */
#define EXO_ASSIST_ANGLE_TORQUE_HARD_MAX_MNM EXO_PULSE_PEAK_HARD_MAX_MNM
/* 事件脉冲与助力层共用同一上升速率边界，通过πA/Ta冻结本次A。
 * 提前退出仍立即清零，不能为了满足上升速率而续放旧脉冲。 */
#define EXO_ASSIST_ANGLE_MAX_RISE_NM_PER_S EXO_PULSE_MAX_RISE_NM_S

typedef enum {
	EXO_ASSIST_MODE_NONE = 0,
	EXO_ASSIST_MODE_TRANSPARENT = 1,
	EXO_ASSIST_MODE_ACTIVE_PHASE = 2,
	EXO_ASSIST_MODE_ASSIST_AS_NEEDED = 3,
	EXO_ASSIST_MODE_ANGLE_THRESHOLD = 4,
	EXO_ASSIST_MODE_ADAPTIVE_GAUSSIAN = 5,
} exo_assist_mode_t;

typedef enum {
	EXO_ASSIST_EXEC_DISABLED = 0,
	EXO_ASSIST_EXEC_SHADOW = 1,
	EXO_ASSIST_EXEC_ACTIVE = 2,
} exo_assist_execution_state_t;

typedef enum {
	EXO_ASSIST_STRATEGY_NONE = 0,
	EXO_ASSIST_STRATEGY_TRANSPARENT = 1,
	EXO_ASSIST_STRATEGY_PHASE_PROFILE = 2,
	EXO_ASSIST_STRATEGY_AAN_RESIDUAL = 3,
	EXO_ASSIST_STRATEGY_ANGLE_THRESHOLD = 4,
	EXO_ASSIST_STRATEGY_ADAPTIVE_GAUSSIAN = 5,
} exo_assist_strategy_id_t;

/*
 * suggestion 的语义必须显式携带，不能靠“哪些 float 恰好为零”猜测。
 * PREVIEW_COMMAND 只供 TRANSPARENT/SHADOW 观察；TORQUE_FF_DELTA 表示只有
 * torque_ff_nm 是可供统一合成器消费的增量。
 */
typedef enum {
	EXO_ASSIST_OUTPUT_NONE = 0,
	EXO_ASSIST_OUTPUT_PREVIEW_COMMAND = 1,
	EXO_ASSIST_OUTPUT_TORQUE_FF_DELTA = 2,
} exo_assist_output_kind_t;

typedef enum {
	EXO_ASSIST_UNAVAILABLE_NONE = 0,
	EXO_ASSIST_UNAVAILABLE_NOT_INITIALIZED = 1,
	EXO_ASSIST_UNAVAILABLE_CONFIG_INVALID = 2,
	EXO_ASSIST_UNAVAILABLE_ACTIVE_NOT_ALLOWED = 3,
	EXO_ASSIST_UNAVAILABLE_STRATEGY_NOT_IMPLEMENTED = 4,
	EXO_ASSIST_UNAVAILABLE_REQUIRED_SENSOR_MISSING = 5,
	EXO_ASSIST_UNAVAILABLE_NOT_ARMED = 6,
	EXO_ASSIST_UNAVAILABLE_REQUIRED_GAIT_INVALID = 7,
	/* 步态虽可用，但本轮 activation 后左右尚未各出现一次新的髋屈曲峰。 */
	EXO_ASSIST_UNAVAILABLE_WAITING_NEW_GAIT_EVENTS = 8,
	EXO_ASSIST_UNAVAILABLE_AFO_ACQUIRING = 9,
	EXO_ASSIST_UNAVAILABLE_AFO_REARM_REQUIRED = 10,
	EXO_ASSIST_UNAVAILABLE_AFO_INPUT_FAULT = 11,
} exo_assist_unavailable_reason_t;

typedef struct {
	uint8_t schema_version;
	exo_assist_mode_t mode;
	exo_assist_execution_state_t requested_execution_state;
	exo_assist_strategy_id_t strategy_id;
	uint8_t profile_id;
	uint8_t left_level_percent;
	uint8_t right_level_percent;
	uint8_t reserved_flags;
	/* 最大“附加助力”绝对值，单位 mN·m。15000 表示 15.000 N·m。
	 * 与左右等级一同在 START 锁存；不是电机协议的额定/峰值力矩。
	 */
	uint16_t maximum_assist_torque_mnm;
	/* 人体前抬正角阈值，单位0.01度；只由ANGLE_THRESHOLD消费。 */
	uint16_t angle_threshold_cdeg;
	/* 事件脉冲参数；A/Ta在本侧新事件起点冻结。 */
	exo_pulse_timing_t pulse_timing;
	exo_pulse_window_t pulse_window;
} exo_assist_config_t;

/*
 * START 前只读预检得到的高层运行合同。requested 是用户选择，execution_state 是
 * 按当前策略实现情况推导出的本轮有效状态；selected_generation 用于诊断同一份
 * 配置。该结构不包含 activation_epoch，因为预检不能推进运行代次。
 */
typedef struct {
	exo_assist_execution_state_t requested_execution_state;
	exo_assist_execution_state_t execution_state;
	uint32_t selected_generation;
	exo_assist_strategy_id_t strategy_id;
} exo_assist_run_contract_t;

/* 静态策略元数据让后续算法明确声明依赖，而不是在运行中静默读取不存在的传感器。
 */
typedef struct {
	exo_assist_strategy_id_t strategy_id;
	uint8_t strategy_version;
	exo_assist_mode_t supported_mode;
	uint32_t required_sensor_mask;
	uint32_t required_gait_valid_flags;
	float minimum_gait_confidence_01;
	size_t fixed_workspace_bytes;
	bool implemented;
} exo_assist_policy_descriptor_t;

/*
 * 每个控制周期的高层策略结果显式绑定输入序号和本次START代次。未来阶段10即使
 * 接通真实输出，也只能消费“本周期返回”的result，不能回读一个可能已过期的全局值。
 */
typedef struct {
	uint32_t activation_epoch;
	uint32_t source_sensor_sequence;
	uint32_t source_gait_sequence;
	int64_t timestamp_us;
	exo_assist_execution_state_t execution_state;
	exo_assist_output_kind_t output_kind;
	bool valid;
	exo_joint_command_pair_t suggestion;
} exo_assist_result_t;

/*
 * ESP32上的“可替换策略”是编译期静态ops表，不是动态插件。reset/step都必须无
	 * 动态内存、无阻塞、无日志、无CAN；状态只放在固定workspace中。
 */
typedef struct {
	esp_err_t (*reset)(void *workspace, size_t workspace_bytes,
					   const exo_assist_config_t *config);
	esp_err_t (*step)(void *workspace, size_t workspace_bytes,
					  const exo_assist_config_t *config,
					  const exo_sensor_snapshot_t *sensors,
					  const exo_gait_estimate_t *gait,
					  exo_assist_result_t *result);
} exo_assist_policy_ops_t;

/* UART/BLE读取的是这个诊断副本，不是实时控制线格式。 */
typedef struct {
	bool initialized;
	bool selected_valid;
	bool active_valid;
	bool config_locked;
	bool strategy_ready;
	bool required_sensors_ready;
	bool required_gait_ready;
	bool suggestion_valid;
	/*
	 * 只有本周期 ACTIVE 助力已加入最终命令且至少一侧 CAN 帧发送成功时才为
	 * true。逐侧成功/失败由控制周期结果和遥测 valid 位另行保存，不能用本总标志
	 * 推断两侧都成功。
	 */
	bool real_output_enabled;
	exo_assist_config_t selected;
	exo_assist_config_t active;
	exo_assist_execution_state_t execution_state;
	exo_assist_unavailable_reason_t unavailable_reason;
	uint8_t strategy_version;
	uint32_t required_sensor_mask;
	uint32_t required_gait_valid_flags;
	float minimum_gait_confidence_01;
	uint32_t available_sensor_mask;
	uint32_t valid_sensor_mask;
	uint32_t selected_generation;
	uint32_t active_generation;
	/* 每次arm和disarm都推进，用于丢弃跨START/STOP边界的旧计算。 */
	uint32_t activation_epoch;
	uint64_t step_count;
	uint64_t valid_suggestion_count;
	/* 只有合成后的最终命令对成功经唯一出口发送，才累计 applied_output_count。
	 */
	uint64_t applied_output_count;
	uint64_t rejected_config_count;
	int64_t last_step_us;
	int64_t last_applied_output_us;
	exo_assist_result_t last_result;
	/* 仅由控制任务发布，BLE读取副本，不能越锁访问策略工作区。 */
	exo_swing_pulse_leg_t pulse_left;
	exo_swing_pulse_leg_t pulse_right;
} exo_assist_snapshot_t;

esp_err_t exo_assist_init(void);

/* 只允许未锁存时修改；调用方仍需通过整机IDLE维护门串行化BLE/UART。 */
esp_err_t exo_assist_set_config(const exo_assist_config_t *config);
/* 原子更新下一事件的脉冲参数，可在运行中使用；不能提高START力矩上限。
 * A/Ta已冻结的当前脉冲保持原值；角度窗口收紧立即生效。 */
esp_err_t exo_assist_update_pulse_timing(const exo_pulse_timing_t *timing);
esp_err_t exo_assist_update_pulse_window(const exo_pulse_window_t *window);

/*
 * 纯只读地预览下一次 arm 将得到的有效执行状态。函数不修改 selected/active、
 * 不推进 activation_epoch，也不读取传感器、步态或控制器。
 */
esp_err_t exo_assist_preview_run_contract(exo_assist_run_contract_t *contract);

/* START/STOP生命周期入口；arm锁存selected，disarm清除active和建议。 */
esp_err_t exo_assist_arm(void);
esp_err_t exo_assist_disarm(void);

/*
 * 唯一100 Hz任务调用。函数只计算并返回绑定本周期的建议；是否合成/发送由
 * controller 决定。函数内部无动态内存、无阻塞等待、无日志、无CAN。
 */
esp_err_t exo_assist_process_tick(const exo_sensor_snapshot_t *sensors,
								  const exo_gait_estimate_t *gait,
								  exo_assist_result_t *result);

/*
 * 由整机输出门在“最终发送结果已经确定”后回报。函数会再次核对 activation
 * epoch、输入序号与最近策略结果；旧周期或 SHADOW 结果不能点亮真实输出标志。
 * applied=false 也会明确清除上一周期的 real_output_enabled，避免状态粘滞。
 */
void exo_assist_report_output(const exo_assist_result_t *result, bool applied);

bool exo_assist_get_snapshot(exo_assist_snapshot_t *snapshot);

bool exo_assist_get_policy_descriptor(
	exo_assist_strategy_id_t strategy_id,
	exo_assist_policy_descriptor_t *descriptor);

const char *exo_assist_mode_name(exo_assist_mode_t mode);
const char *exo_assist_execution_state_name(exo_assist_execution_state_t state);
const char *
exo_assist_unavailable_reason_name(exo_assist_unavailable_reason_t reason);

#ifdef __cplusplus
}
#endif

#endif /* EXO_ASSIST_H */
