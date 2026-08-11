#ifndef SINE_REFERENCE_H
#define SINE_REFERENCE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 正弦参考生成器只做数学计算：给定配置和“启动后经过多久”，返回当前位置偏移。
 * 它不知道左右腿、不读取传感器、不发送 CAN，也不创建 FreeRTOS 任务；因此可在
 * PC 上用同一组输入做确定性回放测试。
 */

typedef struct {
	/* 人体关节正弦幅值（deg），不是 RS01 电机绝对位置。 */
	float amplitude_deg;
	/* 频率（Hz）以及从 0 平滑增至满幅的时间（s）。 */
	/* 例如 0.5 Hz 表示 2 秒完成一个完整正弦周期。 */
	float frequency_hz;
	/* 渐入时间为 0 时立即满幅；大于 0 时使用五次曲线平滑放大。 */
	float ramp_time_s;
} sine_reference_config_t;

typedef struct {
	/* 当前人体关节位置偏移（deg）和无量纲渐入系数 [0, 1]。 */
	float offset_deg;
	/* ramp=0 表示未放大，ramp=1 表示已达到配置幅值。 */
	float ramp;
} sine_reference_sample_t;

/*
 * 纯参数校验；本模块不依赖 ESP-IDF、RTOS、驱动、日志或 I/O。
 * 幅值允许为 0，频率必须大于 0，渐入时间允许为 0，且所有值都必须是有限数。
 */
bool sine_reference_config_is_valid(const sine_reference_config_t *config);

/*
 * 计算 elapsed_us 时刻的正弦关节偏移。非正时间视为轨迹起点并返回零偏移；
 * 本函数不保存运行状态，因此同一输入必定得到同一输出。成功返回 true；输出
 * 指针为空或配置非法时返回 false，并把可写的输出先清零。
 */
bool sine_reference_evaluate(const sine_reference_config_t *config,
							 int64_t elapsed_us,
							 sine_reference_sample_t *sample);

#ifdef __cplusplus
}
#endif

#endif /* SINE_REFERENCE_H */
