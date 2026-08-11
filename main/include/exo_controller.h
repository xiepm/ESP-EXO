#ifndef EXO_CONTROLLER_H
#define EXO_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "exo_types.h"

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

/* 产品控制模式与 RS01 底层 control mode 是两个概念；六种模式都使用 Motion 帧。 */
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

/* 参考源只决定五元目标如何随时间变化；SINE 不是第七种电机模式。 */
typedef enum {
	EXO_REFERENCE_CONSTANT = 0,
	EXO_REFERENCE_SINE = 1,
} exo_reference_source_t;

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

/* UART/BLE 只读诊断快照；它不是连续遥测，也不是 BLE 线格式。 */
typedef struct {
	bool initialized;
	bool armed;
	bool tick_active;
	exo_controller_selection_t selected;
	exo_controller_selection_t active;
	exo_controller_centers_t centers;
	exo_joint_command_pair_t last_output;
	uint32_t selected_generation;
	uint32_t active_generation;
	uint64_t output_count;
	uint64_t rejected_config_count;
	uint64_t output_error_count;
	int64_t start_us;
	int64_t last_output_us;
	esp_err_t last_error;
} exo_controller_snapshot_t;

/* 建立编译期默认选择：IMPEDANCE + SINE，数值保持阶段 7 的已验收行为。 */
esp_err_t exo_controller_init(void);

/*
 * 原子修改 selected 模式和参考源。SINE 当前只允许搭配 IMPEDANCE；NONE 不能被
 * 选中。armed/tick_active 时返回 ESP_ERR_INVALID_STATE。
 */
esp_err_t exo_controller_set_mode(exo_control_mode_t mode,
								  exo_reference_source_t reference);

/* 原子修改一侧 selected 五元参数；另一侧保持不变。只允许未 arm 时调用。 */
esp_err_t exo_controller_set_joint_command(
	exo_hip_joint_id_t joint, const exo_joint_command_t *command);

/* 复制 selected/active、最近输出和计数器；读取不会改变控制状态。 */
bool exo_controller_get_snapshot(exo_controller_snapshot_t *snapshot);

/*
 * 纯计算入口，也是控制器与主机向量测试共用的唯一模式定义。elapsed_us<=0 视为
 * START 时刻。CONSTANT 模式使用同一五次启动渐入；SINE 复用阶段 7 参考生成器。
 * 本函数不读锁、不发 CAN、不调用 RTOS。
 */
bool exo_controller_evaluate(
	const exo_controller_selection_t *selection,
	const exo_controller_centers_t *centers, int64_t elapsed_us,
	exo_joint_command_pair_t *output);

/* START 生命周期入口：锁存 selected、左右起点和时间，但不使能电机、不发 CAN。 */
esp_err_t exo_controller_arm(const exo_controller_centers_t *centers,
							 int64_t start_us);

/* 由唯一 100 Hz 任务调用：生成一对五元命令并经唯一关节命令出口发送。 */
esp_err_t exo_controller_process_tick(int64_t now_us);

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
