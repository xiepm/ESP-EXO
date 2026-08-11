#ifndef EXO_JOINT_COMMAND_H
#define EXO_JOINT_COMMAND_H

#include <stdbool.h>

#include "esp_err.h"
#include "exo_types.h"
#include "rs01_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 阶段 8 的统一关节命令出口
 * ==========================
 *
 * 控制器只生成一对 LEFT/RIGHT 人体命令，不需要知道电机 A/B 的安装位置；本模块
 * 先调用 hip_joint 完成两侧坐标换算，再按执行器槽位重排，最后才调用 RS01 驱动。
 * 因此调用链固定为：
 *
 *   控制器 -> exo_joint_command -> hip_joint -> rs01_driver -> TWAI
 *
 * 本模块不创建任务、队列、定时器或动态内存。try_send_pair() 只能从既有的唯一
 * 100 Hz 控制任务调用，并继承 RS01 try 接口的“零等待、忙则立即返回”语义。
 */

/*
 * 检查单侧人体命令的数值是否合法：所有字段必须是有限数，位置、速度、前馈
 * 力矩和 kp/kd 必须落在 RS01/BLE v1 共同可表达的范围。人体 position_deg 还要
 * 叠加该侧软件零位才能得到电机绝对位置，因此最终绝对位置会在 map_pair() 中
 * 再检查一次。
 */
bool exo_joint_command_is_valid(const exo_joint_command_t *command);

/*
 * 一次性校验并映射左右命令，结果数组按 EXO_ACTUATOR_A/B 下标排列。
 *
 * 只有 LEFT 和 RIGHT 都成功映射、且恰好分别占用 A/B 两个不同槽位时才写出结果。
 * 任一侧无效时返回错误，并把 actuator_commands 全部保持为零；函数绝不发送 CAN。
 * 数组至少要有 EXO_ACTUATOR_COUNT（当前为 2）个元素。
 */
esp_err_t exo_joint_command_map_pair(
	const exo_joint_command_pair_t *joint_commands,
	rs01_motion_command_t actuator_commands[EXO_ACTUATOR_COUNT]);

/*
 * 100 Hz 周期出口：先调用 map_pair() 完成“双侧全量预检”，预检失败时零发送；
 * 预检成功后固定先发执行器 A，再发执行器 B。即使 A 发送失败也会继续尝试 B，
 * 最终返回遇到的第一个错误（A 错优先，否则返回 B 的结果）。函数不打印日志。
 */
esp_err_t
exo_joint_command_try_send_pair(const exo_joint_command_pair_t *joint_commands);

#ifdef __cplusplus
}
#endif

#endif /* EXO_JOINT_COMMAND_H */
