#ifndef EXO_CONTROL_LOOP_H
#define EXO_CONTROL_LOOP_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "exo_assist.h"
#include "exo_controller.h"
#include "exo_sensor_snapshot.h"
#include "gait_estimator.h"

/*
 * 永久 100 Hz 控制任务的公开接口和诊断快照。
 *
 * 本模块只提供全工程唯一的周期“时钟”：每个周期读取左右关节快照，再调用整机
 * 输出门。它不处理 START/STOP 硬件序列，也不因为系统处于 IDLE 就销毁任务；
 * IDLE 时任务仍按 100 Hz 运行并更新诊断，只是输出门不会发送周期控制帧。
 *
 * 常见误区：cycle_count 表示任务被调度的次数，不等于电机命令帧数；
 * period_out_of_tolerance_count 表示周期抖动，deadline_miss_count 表示一次执行
 * 已跨过下一完整周期，二者不是同一个指标。
 */

#ifdef __cplusplus
extern "C" {
#endif

/* 某一侧关节在最近控制周期中的输入来源摘要，不包含控制目标或反馈数值。 */
typedef struct {
	uint32_t feedback_count;
	uint64_t feedback_timestamp_us;
	bool joint_valid;
} exo_control_loop_joint_status_t;

/*
 * 控制任务一致诊断快照。写者先在任务私有副本中完成一整个周期，再在短临界区内
 * 一次性发布；读取者不会看到一半属于旧周期、一半属于新周期的字段组合。
 */
typedef struct {
	/* 任务身份和固定配置；任务句柄用于证明运行期间没有重复创建控制任务。 */
	bool initialized;
	bool task_alive;
	uintptr_t task_handle_identity;

	uint32_t configured_frequency_hz;
	/* 所有周期、误差和执行时间字段的单位均为 us，计数器自任务启动累计。 */
	uint32_t target_period_us;
	uint32_t period_tolerance_us;
	uint64_t cycle_count;
	uint64_t period_sample_count;
	uint64_t period_out_of_tolerance_count;
	uint64_t last_cycle_start_us;
	uint32_t last_period_us;
	uint32_t min_period_us;
	uint32_t max_period_us;
	uint32_t mean_period_us;
	int32_t last_period_error_us;
	uint32_t max_abs_period_error_us;
	uint32_t last_execution_us;
	uint32_t max_execution_us;

	/*
	 * 阶段 9 算法分项耗时都包含在上面的整个周期 execution 中。单列这些值可以
	 * 判断新增负担来自快照、步态还是SHADOW策略，而不是只看到总时间变大。
	 */
	uint32_t last_snapshot_build_us;
	uint32_t max_snapshot_build_us;
	uint32_t last_gait_update_us;
	uint32_t max_gait_update_us;
	uint32_t last_assist_update_us;
	uint32_t max_assist_update_us;
	uint64_t algorithm_error_count;
	esp_err_t last_algorithm_error;

	/*
	 * 实时异常与输出结果。deadline 是“本次执行跨过下个周期”，skipped 是因此
	 * 跳过的完整周期数；output error 来自系统输出门/控制器，不代表任务已退出。
	 */
	uint64_t deadline_miss_count;
	uint64_t skipped_cycle_count;
	uint64_t output_error_count;
	esp_err_t last_output_error;

	/* 最近发布周期读取到的左右输入来源，以及任务历史最小剩余栈空间。 */
	exo_control_loop_joint_status_t left;
	exo_control_loop_joint_status_t right;
	/* 最近一个完整控制周期使用的真实算法输入与估计结果。 */
	exo_sensor_snapshot_t sensor_snapshot;
	exo_gait_estimate_t gait_estimate;
	/* 与上面两份输入序号绑定的本周期策略结果；SHADOW/ACTIVE语义由字段明确区分。 */
	exo_assist_result_t assist_result;
	/*
	 * 阶段 10 最终输出纯值：同时包含 base/final、建议/接纳/实发和发送结果。
	 * 遥测应消费这份同周期副本，不能在另一个任务里重新拼接多个全局快照。
	 */
	exo_controller_tick_result_t control_tick_result;
	uint32_t stack_high_water_mark_bytes;
} exo_control_loop_snapshot_t;

/*
 * 幂等创建一次静态分配、固定核的永久控制任务。频率来自只读硬件配置，当前产品
 * 固定为 100 Hz；重复调用不会创建第二个任务。
 */
esp_err_t exo_control_loop_init(void);

/*
 * 只复制最近一次完整发布的诊断快照，不改变控制器或系统状态。返回成功也不表示
 * 左右 joint_valid 或 last_output_error 一定正常，调用者仍需检查相应字段。
 */
bool exo_control_loop_get_snapshot(exo_control_loop_snapshot_t *snapshot);

#ifdef __cplusplus
}
#endif

#endif /* EXO_CONTROL_LOOP_H */
