#ifndef EXO_PHASE_PROFILE_H
#define EXO_PHASE_PROFILE_H

#include <stdbool.h>
#include <stdint.h>

#include "exo_types.h"
#include "gait_estimator.h"

/*
 * 阶段 10 相位力矩曲线
 * ==================
 *
 * 本模块只有“只读曲线表 + 纯计算插值器”，不保存运行状态，也不读取电机、BLE
 * 或系统时钟。这样主机回放与 ESP32 的 100 Hz 调用会使用同一份实现。
 *
 * 曲线横轴是 gait_estimator 已经发布的单侧归一化相位 [0, 1)，纵轴是人体
 * 髋关节坐标下的前馈力矩增量 N·m：正值表示屈曲方向，负值表示伸展方向。
 */

#ifdef __cplusplus
extern "C" {
#endif

#define EXO_PHASE_PROFILE_LUT_POINTS 16U
#define EXO_PHASE_PROFILE_ID_ZERO_DIAGNOSTIC 0U
#define EXO_PHASE_PROFILE_ID_ENGINEERING_TEST_V1 1U
/* PHASE_PROFILE 策略的统一过渡合同，供固件实现和主机向量测试共用。 */
#define EXO_PHASE_PROFILE_START_RAMP_US 1000000LL
#define EXO_PHASE_PROFILE_MAX_SLEW_NM_PER_10MS 0.01f
/* 保留现有工程验收的 0.5 N·m 上限，网页配置只能降低，不能扩大验收范围。 */
#define EXO_PHASE_PROFILE_TORQUE_HARD_MAX_MNM 500U
#define EXO_PHASE_PROFILE_TORQUE_DEFAULT_MNM 500U

/*
 * profile 元数据供固件业务层、BLE 适配层和测试读取。name 指向固件只读 Flash，
 * 调用者只能读取，不能释放或修改。maximum_abs_torque_nm 描述 level=100% 时的
 * 曲线峰值，不包含底层控制器已有的 torque_ff。
 */
typedef struct {
	uint8_t profile_id;
	uint8_t profile_version;
	const char *name;
	exo_gait_phase_origin_t required_phase_origin;
	uint8_t point_count;
	bool active_allowed;
	bool literature_profile;
	float maximum_abs_torque_nm;
} exo_phase_profile_info_t;

/* 查询一个编译进固件的 profile；不存在时清零 info 并返回 false。 */
bool exo_phase_profile_get_info(uint8_t profile_id,
					exo_phase_profile_info_t *info);

/*
 * 对左右侧分别按各自 phase 做周期线性插值，再分别乘 left/right level 百分比。
 * 输出结构的 position/velocity/kp/kd 恒为零，只有 torque_ff_nm 可能非零，表示
 * “待合成的前馈力矩增量”，绝不能把该结构直接发送给电机。
 *
 * 任一输入无效、相位原点不匹配或 profile 不存在时，函数保持 output 全零并
 * 返回 false；它不会沿用上一周期的结果。
 */
bool exo_phase_profile_evaluate(uint8_t profile_id,
					exo_gait_phase_origin_t phase_origin,
					float left_phase_01, float right_phase_01,
					uint8_t left_level_percent,
					uint8_t right_level_percent,
					exo_joint_command_pair_t *output);

/* 产品配置入口：曲线形状 × 配置最大值(N·m) × 单侧等级/100。
 * 该最大值仅约束附加前馈力矩；基础阻尼和电机反馈力矩分别显示。
 * 保留上面的旧入口给历史回放，旧入口使用默认 0.5 N·m。
 */
bool exo_phase_profile_evaluate_with_maximum(
	uint8_t profile_id, exo_gait_phase_origin_t phase_origin,
	float left_phase_01, float right_phase_01,
	uint8_t left_level_percent, uint8_t right_level_percent,
	float maximum_assist_torque_nm, exo_joint_command_pair_t *output);

#ifdef __cplusplus
}
#endif

#endif /* EXO_PHASE_PROFILE_H */
