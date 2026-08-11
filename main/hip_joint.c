#include "hip_joint.h"

#include <math.h>
#include <string.h>

#include "exo_config.h"
#include "freertos/FreeRTOS.h"
#include "rs01_driver.h"

/*
 * 人体髋关节坐标适配层。
 *
 * 数据上行（反馈）：RS01 连续电机 rad -> 换算成 deg -> 减连续软件零位 -> 乘方向
 * 数据下行（目标）：人体关节 deg -> 乘方向 -> 加周期软件零位 -> RS01 电机 rad
 *
 * 这里不控制电机，也不生成轨迹；它只负责回答“某个左/右关节对应哪个电机、
 * 当前人体角是多少、人体目标应换成多少电机角”。把公式集中在一个模块，可以避免
 * 控制器、串口和 BLE 各写一遍后出现符号不一致。
 */

/*
 * 内部长期状态比公开快照更小：只保存不会随每帧反馈改变的映射和 RAM 零位。
 * 最新电机反馈仍由 rs01_driver 保存，读取快照时再组合，避免维护两份反馈真相。
 */
typedef struct {
	exo_hip_joint_id_t joint;
	exo_side_t side;
	exo_actuator_id_t actuator;
	exo_direction_t direction;
	/* 连续零位用于解释反馈；周期零位用于生成 -4π～+4π 内的线上命令。 */
	float software_zero_motor_deg;
	float software_zero_periodic_motor_deg;
	uint64_t zero_source_timestamp_us;
	uint32_t zero_source_feedback_count;
	uint32_t zero_source_position_epoch;
	bool mapping_valid;
	bool direction_valid;
	bool zero_valid;
} hip_joint_t;

static hip_joint_t s_joints[EXO_HIP_JOINT_COUNT];
/* s_initialized 区分“全零内存”与“已经成功建立过两个关节对象”。 */
static bool s_initialized;
/*
 * portMUX 临界区只保护很短的结构复制/标志修改，可同时被双核任务调用。
 * 临界区内不调用 CAN、不打印日志、不做慢速等待，以免阻塞另一个 CPU 核。
 */
static portMUX_TYPE s_joint_lock = portMUX_INITIALIZER_UNLOCKED;

/* 枚举也可能由错误强制转换产生越界值，公开接口入口统一做范围校验。 */
static bool joint_id_is_valid(exo_hip_joint_id_t joint) {
	return (unsigned)joint < (unsigned)EXO_HIP_JOINT_COUNT;
}

static bool actuator_id_is_valid(exo_actuator_id_t actuator) {
	return (unsigned)actuator < (unsigned)EXO_ACTUATOR_COUNT;
}

static bool direction_is_valid(exo_direction_t direction) {
	return direction == EXO_DIRECTION_POSITIVE ||
		   direction == EXO_DIRECTION_NEGATIVE;
}

/* 数组下标 0/1 之外，还核对配置结构中写入的 side，尽早发现左右表填反。 */
static exo_side_t expected_side(exo_hip_joint_id_t joint) {
	return joint == EXO_HIP_JOINT_LEFT ? EXO_SIDE_LEFT : EXO_SIDE_RIGHT;
}

/* CAN 解码结果若含 NaN/Inf，就算帧格式正确也不能送入控制计算。 */
static bool motion_sample_is_finite(const exo_motion_sample_t *sample) {
	return isfinite(sample->position_rad) && isfinite(sample->velocity_rad_s) &&
		   isfinite(sample->torque_nm) && isfinite(sample->temperature_c);
}

/*
 * RS01 原始采样以弧度表示；只有在这里进入人体关节对象时才转换成度。
 * 方向符号同时作用于角度、角速度和力矩，温度不参与坐标变换。
 *
 * 例：右侧 direction=-1，电机从零位减少 10 deg，则人体角为 +10 deg（屈曲）；
 * 电机速度和力矩也乘 -1，保证位置、速度、力矩使用同一个人体正方向。
 */
static void actuator_sample_to_joint_sample(
	const exo_motion_sample_t *actuator_sample,
	float continuous_motor_position_rad, float software_zero_motor_deg,
	exo_direction_t direction, exo_joint_sample_t *joint_sample) {
	const float sign = (float)direction;
	const float motor_position_deg =
		exo_radians_to_degrees(continuous_motor_position_rad);
	joint_sample->position_deg =
		sign * (motor_position_deg - software_zero_motor_deg);
	joint_sample->velocity_deg_s =
		sign * exo_radians_to_degrees(actuator_sample->velocity_rad_s);
	joint_sample->torque_nm = sign * actuator_sample->torque_nm;
	joint_sample->temperature_c = actuator_sample->temperature_c;
}

static bool config_entry_is_valid(exo_hip_joint_id_t joint,
								  const exo_hip_joint_config_t *config) {
	/*
	 * valid 标志与内容必须一致：mapping_valid=false 时 actuator 必须是 COUNT
	 * 哨兵； direction_valid=false 时 direction 必须是
	 * UNKNOWN；有效方向又必须建立在有效 电机映射之上。这样 0
	 * 值不会被误解成已经完成标定。
	 */
	const bool actuator_valid = actuator_id_is_valid(config->actuator);
	const bool direction_valid = direction_is_valid(config->direction);
	const bool mapping_state_valid =
		config->mapping_valid ? actuator_valid
							  : config->actuator == EXO_ACTUATOR_COUNT;
	const bool direction_state_valid =
		config->direction_valid ? direction_valid
								: config->direction == EXO_DIRECTION_UNKNOWN;

	return config->side == expected_side(joint) && mapping_state_valid &&
		   direction_state_valid &&
		   (!config->direction_valid || config->mapping_valid) &&
		   isfinite(config->software_zero_deg) &&
		   (!config->software_zero_valid || config->mapping_valid);
}

esp_err_t hip_joint_init(void) {
	/* 快速路径：已初始化就直接成功，方便上层重复确认而不会重置运行期软件零位。
	 */
	portENTER_CRITICAL(&s_joint_lock);
	const bool already_initialized = s_initialized;
	portEXIT_CRITICAL(&s_joint_lock);
	if (already_initialized) {
		return ESP_OK;
	}

	/*
	 * 先在栈上的临时数组完成全部校验和构造。只有两侧都正确后才整体发布，避免
	 * “左侧初始化成功、右侧失败”留下半初始化的全局状态。
	 */
	hip_joint_t joints[EXO_HIP_JOINT_COUNT] = {0};
	for (unsigned i = 0; i < (unsigned)EXO_HIP_JOINT_COUNT; ++i) {
		const exo_hip_joint_id_t joint = (exo_hip_joint_id_t)i;
		const exo_hip_joint_config_t *config =
			&g_exo_hardware_config.hip_joint[i];
		if (!config_entry_is_valid(joint, config)) {
			return ESP_ERR_INVALID_ARG;
		}

		joints[i].joint = joint;
		joints[i].side = config->side;
		joints[i].actuator = config->actuator;
		joints[i].direction = config->direction;
		joints[i].software_zero_motor_deg = config->software_zero_deg;
		joints[i].software_zero_periodic_motor_deg =
			config->software_zero_deg;
		joints[i].mapping_valid = config->mapping_valid;
		joints[i].direction_valid = config->direction_valid;
		joints[i].zero_valid = config->software_zero_valid;
	}

	if (joints[EXO_HIP_JOINT_LEFT].mapping_valid &&
		joints[EXO_HIP_JOINT_RIGHT].mapping_valid &&
		joints[EXO_HIP_JOINT_LEFT].actuator ==
			joints[EXO_HIP_JOINT_RIGHT].actuator) {
		/* 一个物理执行器不能同时代表左右两个髋关节。 */
		return ESP_ERR_INVALID_ARG;
	}

	portENTER_CRITICAL(&s_joint_lock);
	if (!s_initialized) {
		memcpy(s_joints, joints, sizeof(s_joints));
		s_initialized = true;
	}
	portEXIT_CRITICAL(&s_joint_lock);
	return ESP_OK;
}

bool hip_joint_get_snapshot(exo_hip_joint_id_t joint,
							hip_joint_snapshot_t *snapshot) {
	/*
	 * 先把输出清零：即使后面因未归零/离线提前返回，调用者也不会看到上一次调用
	 * 遗留的旧字段。返回 true 仅代表快照读取流程完成，可用性看 joint_valid。
	 */
	if (snapshot == NULL) {
		return false;
	}
	memset(snapshot, 0, sizeof(*snapshot));
	if (!joint_id_is_valid(joint)) {
		return false;
	}

	/* 临界区内只复制小型配置状态，随后立刻释放锁再访问 RS01 驱动。 */
	hip_joint_t state;
	portENTER_CRITICAL(&s_joint_lock);
	if (!s_initialized) {
		portEXIT_CRITICAL(&s_joint_lock);
		return false;
	}
	state = s_joints[joint];
	portEXIT_CRITICAL(&s_joint_lock);

	snapshot->joint = state.joint;
	snapshot->side = state.side;
	snapshot->actuator = state.actuator;
	snapshot->direction = state.direction;
	snapshot->software_zero_motor_deg = state.software_zero_motor_deg;
	snapshot->software_zero_periodic_motor_deg =
		state.software_zero_periodic_motor_deg;
	snapshot->zero_source_timestamp_us = state.zero_source_timestamp_us;
	snapshot->zero_source_feedback_count = state.zero_source_feedback_count;
	snapshot->zero_source_position_epoch = state.zero_source_position_epoch;
	snapshot->mapping_valid = state.mapping_valid;
	snapshot->direction_valid = state.direction_valid;
	/* 是否曾捕获由内部 state 保存；公开 zero_valid 要等反馈 epoch 核对后才能成立。 */
	snapshot->zero_valid = false;

	if (!state.mapping_valid) {
		/* 配置未建立时仍返回一份可诊断快照，让上层看到 mapping_valid=false。 */
		return true;
	}

	/* RS01 自己管理反馈锁；这里拿到另一份一致快照后再组合成人体关节快照。 */
	rs01_feedback_snapshot_t feedback;
	if (!rs01_get_feedback(state.actuator, &feedback)) {
		return true;
	}

	snapshot->raw_actuator_sample = feedback.sample;
	snapshot->continuous_actuator_position_rad =
		feedback.continuous_position_rad;
	snapshot->actuator_position_wrap_count = feedback.position_wrap_count;
	snapshot->actuator_position_epoch = feedback.position_epoch;
	snapshot->actuator_position_rebase_count =
		feedback.position_rebase_count;
	snapshot->feedback_timestamp_us = feedback.timestamp_us;
	snapshot->source_feedback_count = feedback.feedback_count;
	snapshot->feedback_valid = feedback.valid;
	snapshot->continuous_position_valid = feedback.continuous_position_valid;
	snapshot->online = feedback.online;
	snapshot->zero_epoch_matches =
		state.zero_valid && feedback.position_epoch != 0U &&
		state.zero_source_position_epoch == feedback.position_epoch;
	snapshot->zero_valid = snapshot->zero_epoch_matches;

	/* 只有完成方向和零位标定后才有资格执行人体坐标变换。 */
	const bool sample_finite =
		motion_sample_is_finite(&feedback.sample) &&
		feedback.continuous_position_valid &&
		isfinite(feedback.continuous_position_rad);
	if (state.direction_valid && snapshot->zero_valid && feedback.valid &&
		sample_finite) {
		/*
		 * 人体关节正向公式：
		 * q_joint_deg = direction * (q_motor_deg - zero_motor_deg)。
		 */
		actuator_sample_to_joint_sample(
			&feedback.sample, feedback.continuous_position_rad,
			state.software_zero_motor_deg, state.direction,
			&snapshot->joint_sample);
	}

	/* 只有所有坐标、零位和实时反馈条件同时成立，控制层才可使用采样。 */
	snapshot->joint_valid = state.mapping_valid && state.direction_valid &&
							snapshot->zero_valid && feedback.valid &&
							feedback.online && sample_finite;
	return true;
}

esp_err_t hip_joint_set_zero_from_current(exo_hip_joint_id_t joint) {
	/*
	 * “从当前值归零”的含义：操作者把身体摆到中立位，本函数把此刻电机角记住。
	 * 它不会命令电机转回 0，也不会写入 RS01 Flash，因此不会改变机械零点。
	 */
	if (!joint_id_is_valid(joint)) {
		return ESP_ERR_INVALID_ARG;
	}

	exo_actuator_id_t actuator = EXO_ACTUATOR_COUNT;
	bool mapping_valid = false;
	portENTER_CRITICAL(&s_joint_lock);
	if (s_initialized) {
		actuator = s_joints[joint].actuator;
		mapping_valid = s_joints[joint].mapping_valid;
	}
	portEXIT_CRITICAL(&s_joint_lock);
	if (!mapping_valid || !actuator_id_is_valid(actuator)) {
		return ESP_ERR_INVALID_STATE;
	}

	/* 不能用陈旧或非法反馈归零，否则后续所有人体角都会带固定偏差。 */
	rs01_feedback_snapshot_t feedback;
	if (!rs01_get_feedback(actuator, &feedback) || !feedback.valid ||
		!feedback.online || !feedback.continuous_position_valid ||
		!isfinite(feedback.sample.position_rad) ||
		!isfinite(feedback.continuous_position_rad)) {
		return ESP_ERR_INVALID_STATE;
	}

	/* 零位值和其来源时间/序号一次性更新，读取者不会看到一半新一半旧。 */
	portENTER_CRITICAL(&s_joint_lock);
	s_joints[joint].software_zero_motor_deg =
		exo_radians_to_degrees(feedback.continuous_position_rad);
	s_joints[joint].software_zero_periodic_motor_deg =
		exo_radians_to_degrees(feedback.sample.position_rad);
	s_joints[joint].zero_source_timestamp_us = feedback.timestamp_us;
	s_joints[joint].zero_source_feedback_count = feedback.feedback_count;
	s_joints[joint].zero_source_position_epoch = feedback.position_epoch;
	s_joints[joint].zero_valid = true;
	portEXIT_CRITICAL(&s_joint_lock);
	return ESP_OK;
}

esp_err_t hip_joint_clear_zero(exo_hip_joint_id_t joint) {
	/* 清零的是“零位记录”，不是把关节当前位置强制写成 0。 */
	if (!joint_id_is_valid(joint)) {
		return ESP_ERR_INVALID_ARG;
	}

	portENTER_CRITICAL(&s_joint_lock);
	if (!s_initialized) {
		portEXIT_CRITICAL(&s_joint_lock);
		return ESP_ERR_INVALID_STATE;
	}
	s_joints[joint].software_zero_motor_deg = 0.0f;
	s_joints[joint].software_zero_periodic_motor_deg = 0.0f;
	s_joints[joint].zero_source_timestamp_us = 0U;
	s_joints[joint].zero_source_feedback_count = 0U;
	s_joints[joint].zero_source_position_epoch = 0U;
	s_joints[joint].zero_valid = false;
	portEXIT_CRITICAL(&s_joint_lock);
	return ESP_OK;
}

esp_err_t hip_joint_to_actuator_command(
	exo_hip_joint_id_t joint, const exo_joint_command_t *joint_command,
	exo_actuator_id_t *actuator, rs01_motion_command_t *actuator_command) {
	/*
	 * 输出先设置成明确的无效值。这样调用者即使忘记在错误分支中清理局部变量，
	 * 也不会误用上一次成功映射留下的执行器编号或五元命令。
	 */
	if (actuator != NULL) {
		*actuator = EXO_ACTUATOR_COUNT;
	}
	if (actuator_command != NULL) {
		memset(actuator_command, 0, sizeof(*actuator_command));
	}
	if (actuator == NULL || actuator_command == NULL) {
		return ESP_ERR_INVALID_ARG;
	}

	/*
	 * 人体侧结构还没有执行坐标变换，先只检查所有字段是不是普通有限数。
	 * kp/kd 的协议量程以及映射后的电机量程由下方统一的 RS01 校验器判断。
	 */
	if (!joint_id_is_valid(joint) || joint_command == NULL ||
		!isfinite(joint_command->position_deg) ||
		!isfinite(joint_command->velocity_deg_s) ||
		!isfinite(joint_command->torque_ff_nm) ||
		!isfinite(joint_command->kp) || !isfinite(joint_command->kd)) {
		return ESP_ERR_INVALID_ARG;
	}

	/*
	 * 在同一个短临界区内复制映射、方向和零位，保证五个输出字段使用同一版配置。
	 * 后续浮点换算和量程校验都在锁外进行，不占用双核共享临界区。
	 */
	hip_joint_t state;
	portENTER_CRITICAL(&s_joint_lock);
	if (!s_initialized) {
		portEXIT_CRITICAL(&s_joint_lock);
		return ESP_ERR_INVALID_STATE;
	}
	state = s_joints[joint];
	portEXIT_CRITICAL(&s_joint_lock);

	/* 尚未完成左右映射、方向或软件归零时，五元换算没有物理意义。 */
	if (!state.mapping_valid || !actuator_id_is_valid(state.actuator) ||
		!state.direction_valid || !direction_is_valid(state.direction) ||
		!state.zero_valid) {
		return ESP_ERR_INVALID_STATE;
	}

	/*
	 * 映射前再核对驱动连续坐标世代。长时间掉线或机械置零会改变 epoch，旧 RAM
	 * 零位即使数值仍有限也不得继续生成命令。
	 */
	rs01_feedback_snapshot_t feedback;
	if (!rs01_get_feedback(state.actuator, &feedback) || !feedback.valid ||
		!feedback.online || !feedback.continuous_position_valid ||
		feedback.position_epoch == 0U ||
		feedback.position_epoch != state.zero_source_position_epoch) {
		return ESP_ERR_INVALID_STATE;
	}

	const float sign = (float)state.direction;
	const float periodic_motor_position_rad = exo_degrees_to_radians(
		state.software_zero_periodic_motor_deg +
		sign * joint_command->position_deg);
	const rs01_motion_command_t mapped = {
		/*
		 * 手册只明确反馈会周期回绕，没有证明目标会按最短模差解释；因此下行仍
		 * 使用捕获时的周期零位并严格拒绝跨界目标，绝不自动取模成另一侧角度。
		 */
		.position_rad = periodic_motor_position_rad,
		/* 速度没有零点，只进行 deg/s -> rad/s 和方向转换。 */
		.velocity_rad_s =
			sign * exo_degrees_to_radians(joint_command->velocity_deg_s),
		/* 前馈力矩和位置采用同一个人体正方向。 */
		.torque_nm = sign * joint_command->torque_ff_nm,
		/* 协议增益是非方向量，右腿也不能乘 -1。 */
		.kp = joint_command->kp,
		.kd = joint_command->kd,
	};

	/*
	 * 在发布输出之前检查整个电机命令。这里既能拦截运算溢出的 Inf，也能拦截
	 * 位置、速度、力矩或增益越过 RS01 定点量程的值，不让驱动偷偷裁剪。
	 */
	if (!rs01_motion_command_is_valid(&mapped)) {
		return ESP_ERR_INVALID_ARG;
	}

	*actuator = state.actuator;
	*actuator_command = mapped;
	return ESP_OK;
}

esp_err_t hip_joint_to_actuator_position(exo_hip_joint_id_t joint,
										 float joint_position_deg,
										 exo_actuator_id_t *actuator,
										 float *actuator_position_rad) {
	/* 两个输出指针都是必需的：上层既要知道发给谁，也要知道发什么位置。 */
	if (actuator == NULL || actuator_position_rad == NULL) {
		return ESP_ERR_INVALID_ARG;
	}
	*actuator = EXO_ACTUATOR_COUNT;
	*actuator_position_rad = 0.0f;

	/*
	 * 兼容接口通过“其余四项为零”的五元命令复用唯一换算公式。这样阶段 3～7
	 * 的旧调用者无需修改，同时不会在项目里保留第二套位置方向/零位公式。
	 */
	const exo_joint_command_t joint_command = {
		.position_deg = joint_position_deg,
		.velocity_deg_s = 0.0f,
		.torque_ff_nm = 0.0f,
		.kp = 0.0f,
		.kd = 0.0f,
	};
	rs01_motion_command_t mapped;
	const esp_err_t err = hip_joint_to_actuator_command(
		joint, &joint_command, actuator, &mapped);
	if (err != ESP_OK) {
		return err;
	}

	*actuator_position_rad = mapped.position_rad;
	return ESP_OK;
}
