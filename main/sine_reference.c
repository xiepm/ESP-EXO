#include "sine_reference.h"

#include <math.h>
#include <stddef.h>

/*
 * 阶段 7 正弦参考的纯数学实现。
 *
 * 输出公式：offset_deg = amplitude_deg * ramp(t) * sin(2*pi*f*t)
 * 其中 ramp(t) 在启动初期从 0 平滑到 1。这里的 rad 只用于 sin() 的相位输入，
 * 对外输出仍然是人体关节的度，不应把 phase_rad 当成电机目标。
 */

/* 使用 double 精度的 2*pi 常量做长时间相位折返，再降为 float 交给 sinf。 */
#define SINE_REFERENCE_TWO_PI_D 6.283185307179586476925286766559

/*
 * 根据启动后秒数计算 [0,1] 渐入系数。
 * 分支先处理起点、终点和“禁用渐入”，中间区间才计算五次多项式。
 */
static float quintic_ramp(float elapsed_s, float ramp_time_s) {
	if (ramp_time_s <= 0.0f) {
		/* 配置 0 秒表示不渐入，从第一个有效周期起就是满幅。 */
		return 1.0f;
	}
	if (elapsed_s <= 0.0f) {
		/* START 时刻及之前均从零幅值开始。 */
		return 0.0f;
	}
	if (elapsed_s >= ramp_time_s) {
		/* 渐入完成后固定为 1，避免继续代入多项式而越界。 */
		return 1.0f;
	}

	/* 把实际时间归一化为 s∈(0,1)，后续公式与具体渐入秒数无关。 */
	const float s = elapsed_s / ramp_time_s;
	const float s2 = s * s;
	const float s3 = s2 * s;
	/*
	 * 五次平滑步进 10s^3-15s^4+6s^5 在两端的速度和加速度均为零，
	 * 避免 START 后参考幅值发生阶跃。
	 */
	return s3 * (10.0f - 15.0f * s + 6.0f * s2);
}

bool sine_reference_config_is_valid(const sine_reference_config_t *config) {
	/* isfinite 同时排除 NaN、+Inf 和 -Inf，避免它们污染 100 Hz 控制链。 */
	return config != NULL && isfinite(config->amplitude_deg) &&
		   isfinite(config->frequency_hz) && isfinite(config->ramp_time_s) &&
		   config->amplitude_deg >= 0.0f && config->frequency_hz > 0.0f &&
		   config->ramp_time_s >= 0.0f;
}

bool sine_reference_evaluate(const sine_reference_config_t *config,
							 int64_t elapsed_us,
							 sine_reference_sample_t *sample) {
	if (sample == NULL) {
		return false;
	}
	/* 先给调用者一个确定的零输出，即便后面的配置校验失败也不会残留旧参考。 */
	*sample = (sine_reference_sample_t){0};
	if (!sine_reference_config_is_valid(config)) {
		return false;
	}

	/* 由于任务调度或测试可能传入负差值，将它视为轨迹尚未开始。 */
	const int64_t clamped_elapsed_us = elapsed_us > 0 ? elapsed_us : 0;
	const float elapsed_s = (float)clamped_elapsed_us / 1000000.0f;
	const float ramp = quintic_ramp(elapsed_s, config->ramp_time_s);
	/*
	 * 先用 double 计算完整周期数并折返到 [0,1)，再交给 sinf()。这样长时间
	 * 运行时相位参数不会无限增大。phase_rad 只是三角函数的数学输入；模块对外
	 * 的人体关节偏移仍统一为度。
	 */
	const double cycles =
		(double)clamped_elapsed_us * (double)config->frequency_hz / 1000000.0;
	const double wrapped_cycles = cycles - floor(cycles);
	const float phase_rad = (float)(SINE_REFERENCE_TWO_PI_D * wrapped_cycles);
	/* 先用 ramp 缩放无量纲正弦值，最后才乘以以度表示的幅值。 */
	const float shaped_sine = ramp * sinf(phase_rad);
	const float offset_deg = config->amplitude_deg * shaped_sine;
	if (!isfinite(ramp) || !isfinite(offset_deg)) {
		return false;
	}

	/* 所有中间值合法后再一次性发布结果。 */
	sample->offset_deg = offset_deg;
	sample->ramp = ramp;
	return true;
}
