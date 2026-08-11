#ifndef EXO_ASSIST_H
#define EXO_ASSIST_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "exo_sensor_snapshot.h"
#include "gait_estimator.h"

/*
 * 阶段 9 高层助力策略框架
 * =====================
 *
 * “主动助力/按需辅助”是产品策略；“力矩/阻抗/位置”是阶段 8 已有的底层五元命令
 * 执行方式。二者不能放进同一个枚举。策略只读取传感快照和步态估计，并提出一对
 * exo_joint_command_t；它无权切换系统状态、发送 BLE 或调用 RS01/CAN。
 *
 * ESP32 不支持运行时装载动态插件。这里的“可替换”指编译期静态策略描述表、固定
 * 工作区和版本化接口。阶段 9 只实现透明策略的 SHADOW 建议，真实助力输出恒关闭。
 */

#ifdef __cplusplus
extern "C" {
#endif

#define EXO_ASSIST_CONFIG_SCHEMA_VERSION 1U
#define EXO_ASSIST_POLICY_WORKSPACE_BYTES 64U

typedef enum {
	EXO_ASSIST_MODE_NONE = 0,
	EXO_ASSIST_MODE_TRANSPARENT = 1,
	EXO_ASSIST_MODE_ACTIVE_PHASE = 2,
	EXO_ASSIST_MODE_ASSIST_AS_NEEDED = 3,
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
} exo_assist_strategy_id_t;

typedef enum {
	EXO_ASSIST_UNAVAILABLE_NONE = 0,
	EXO_ASSIST_UNAVAILABLE_NOT_INITIALIZED = 1,
	EXO_ASSIST_UNAVAILABLE_CONFIG_INVALID = 2,
	EXO_ASSIST_UNAVAILABLE_ACTIVE_NOT_ALLOWED = 3,
	EXO_ASSIST_UNAVAILABLE_STRATEGY_NOT_IMPLEMENTED = 4,
	EXO_ASSIST_UNAVAILABLE_REQUIRED_SENSOR_MISSING = 5,
	EXO_ASSIST_UNAVAILABLE_NOT_ARMED = 6,
	EXO_ASSIST_UNAVAILABLE_REQUIRED_GAIT_INVALID = 7,
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
} exo_assist_config_t;

/* 静态策略元数据让后续算法明确声明依赖，而不是在运行中静默读取不存在的传感器。 */
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
	bool valid;
	exo_joint_command_pair_t suggestion;
} exo_assist_result_t;

/*
 * ESP32上的“可替换策略”是编译期静态ops表，不是动态插件。reset/step都必须无
 * 动态内存、无阻塞、无日志、无CAN；状态只能放在固定64-byte workspace中。
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
	/* 阶段 9 永远为false；阶段10必须显式更改并重新验收才能输出真实助力。 */
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
	uint64_t rejected_config_count;
	int64_t last_step_us;
	exo_assist_result_t last_result;
} exo_assist_snapshot_t;

esp_err_t exo_assist_init(void);

/* 只允许未锁存时修改；调用方仍需通过整机IDLE维护门串行化BLE/UART。 */
esp_err_t exo_assist_set_config(const exo_assist_config_t *config);

/* START/STOP生命周期入口；arm锁存selected，disarm清除active和建议。 */
esp_err_t exo_assist_arm(void);
esp_err_t exo_assist_disarm(void);

/*
 * 唯一100 Hz任务调用。阶段9只更新SHADOW建议和诊断，返回值不会被用作真实电机
 * 命令；函数内部无动态内存、无阻塞等待、无日志、无CAN。
 */
esp_err_t exo_assist_process_tick(const exo_sensor_snapshot_t *sensors,
				  const exo_gait_estimate_t *gait,
				  exo_assist_result_t *result);

bool exo_assist_get_snapshot(exo_assist_snapshot_t *snapshot);

bool exo_assist_get_policy_descriptor(
	exo_assist_strategy_id_t strategy_id,
	exo_assist_policy_descriptor_t *descriptor);

const char *exo_assist_mode_name(exo_assist_mode_t mode);
const char *exo_assist_execution_state_name(
	exo_assist_execution_state_t state);
const char *exo_assist_unavailable_reason_name(
	exo_assist_unavailable_reason_t reason);

#ifdef __cplusplus
}
#endif

#endif /* EXO_ASSIST_H */
