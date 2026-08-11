/*
 * 工程 UART 调试控制台。
 *
 * 【它是什么】通过 ESP32 编程串口接收一行文本，例如 `status`、`joint_zero left`
 * 或 `start`，解析后调用对应模块的公开 API，最后把结果打印回串口。
 *
 * 【它不是什么】它不是手机 App 协议。手机通过 exo_ble.c 的 GATT 特征发送有界
 * 二进制帧；不要让 App 依赖这里的英文文本、空格或 printf 输出格式。
 *
 * 【一条命令怎样流动】
 * UART 字符 -> uart_console_task() 拼成一行 -> console_process_line()
 * 分词并查表
 * -> 权限检查 -> cmd_xxx() 参数转换 -> RS01/关节/状态机 API -> debug_printf()。
 *
 * 【单位边界】操作者输入和看到的人体角度统一为 deg/deg/s；RS01 驱动仍使用
 * rad/rad/s，所以所有裸电机运动命令都在本文件调用驱动之前显式换算。
 */

#include "debug_console.h"

#include "driver/gpio.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "exo_assist.h"
#include "exo_ble.h"
#include "exo_config.h"
#include "exo_control_loop.h"
#include "exo_controller.h"
#include "exo_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "hip_joint.h"
#include "rs01_driver.h"

#include <errno.h>
#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

static const char *TAG = "debug_console";

/* 单行最多 255 个可见字节，最后一个字节留给 C 字符串结束符 '\0'。 */
#define LINE_BUF_SIZE 256
/* 控制台是低优先级工程任务，固定在 CPU0；唯一 100 Hz 控制任务不在本文件。 */
#define EXO_DEBUG_CONSOLE_TASK_CORE 0
#define EXO_DEBUG_CONSOLE_TASK_PRIORITY 5
#define EXO_DEBUG_CONSOLE_TASK_STACK_BYTES 4096

/*
 * 命令互斥量保证同一时刻只有一条文本命令处于解析/执行流程。当前只有一个 UART
 * 读任务，但保留互斥可防止后续测试入口并发调用 console_process_line()。
 */
static SemaphoreHandle_t s_cmd_mutex;
/* 非 NULL 表示控制台任务已经创建，也用于阻止重复初始化。 */
static TaskHandle_t s_uart_task;
/*
 * `motor A/B` 只改变后续“裸电机维护命令”的目标槽位，不改变人体左右映射，也
 * 不会影响状态机控制任务。上电默认值来自 g_exo_hardware_config。
 */
static exo_actuator_id_t s_selected_actuator;

static void uart_console_task(void *arg);
static void console_process_line(char *line);
static void debug_printf(const char *fmt, ...);
static bool parse_long_arg(const char *text, long min_value, long max_value,
						   long *value);
static bool parse_float_arg(const char *text, float *value);
static bool parse_joint_arg(const char *text, exo_hip_joint_id_t *joint);
static exo_can_node_id_t selected_can_id(void);
static bool driver_command_succeeded(esp_err_t err);

/* ---- 命令描述类型 ------------------------------------------------------- */
/* 每个命令处理函数收到 argc/argv，形式与普通 C 程序 main(argc, argv) 类似。 */
typedef void (*cmd_handler_t)(int argc, char *argv[]);

/*
 * 命令访问级别：
 * ALWAYS_AVAILABLE：只读诊断等，可在任意系统状态查看；
 * MAINTENANCE：会直接操作电机/零位，必须先从状态机取得 IDLE 维护权；
 * SYSTEM_REQUEST：只投递 START/STOP，请求由后台执行器异步完成。
 */
typedef enum {
	CONSOLE_CMD_ALWAYS_AVAILABLE = 0,
	CONSOLE_CMD_MAINTENANCE,
	CONSOLE_CMD_SYSTEM_REQUEST,
} console_cmd_access_t;

typedef struct {
	/* 用户输入的第一个单词，例如 "status"。 */
	const char *name;
	/* help 命令显示的简要格式说明；尖括号内容表示用户需要提供的参数。 */
	const char *help;
	/* 不含命令名的最少参数数量。 */
	int min_args;
	console_cmd_access_t access;
	/* 查表成功后真正执行命令的函数。 */
	cmd_handler_t handler;
} console_cmd_t;

/* ---- 命令处理函数前置声明 ---------------------------------------------- */
/* 先声明、后在表中引用，让“支持哪些命令”集中在一张表里可见。 */
static void cmd_motor(int argc, char *argv[]);
static void cmd_enable(int argc, char *argv[]);
static void cmd_disable(int argc, char *argv[]);
static void cmd_move(int argc, char *argv[]);
static void cmd_speed(int argc, char *argv[]);
static void cmd_pos_pp(int argc, char *argv[]);
static void cmd_pos_csp(int argc, char *argv[]);
static void cmd_current(int argc, char *argv[]);
static void cmd_set_zero(int argc, char *argv[]);
static void cmd_set_id(int argc, char *argv[]);
static void cmd_save(int argc, char *argv[]);
static void cmd_set_baud(int argc, char *argv[]);
static void cmd_report(int argc, char *argv[]);
static void cmd_switch_mit(int argc, char *argv[]);
static void cmd_switch_private(int argc, char *argv[]);
static void cmd_read_param(int argc, char *argv[]);
static void cmd_mit_enable(int argc, char *argv[]);
static void cmd_mit_disable(int argc, char *argv[]);
static void cmd_mit_control(int argc, char *argv[]);
static void cmd_mit_pos(int argc, char *argv[]);
static void cmd_mit_speed(int argc, char *argv[]);
static void cmd_mit_set_zero(int argc, char *argv[]);
static void cmd_mit_set_type(int argc, char *argv[]);
static void cmd_mit_set_id(int argc, char *argv[]);
static void cmd_mit_save(int argc, char *argv[]);
static void cmd_mit_report(int argc, char *argv[]);
static void cmd_mit_read(int argc, char *argv[]);
static void cmd_mit_write(int argc, char *argv[]);
static void cmd_mit_error(int argc, char *argv[]);
static void cmd_motor_type(int argc, char *argv[]);
static void cmd_status(int argc, char *argv[]);
static void cmd_joint_status(int argc, char *argv[]);
static void cmd_joint_zero(int argc, char *argv[]);
static void cmd_joint_zero_clear(int argc, char *argv[]);
static void cmd_motion_mode(int argc, char *argv[]);
static void cmd_joint_command(int argc, char *argv[]);
static void cmd_control_status(int argc, char *argv[]);
static void cmd_sensor_status(int argc, char *argv[]);
static void cmd_gait_status(int argc, char *argv[]);
static void cmd_assist_status(int argc, char *argv[]);
static void cmd_assist_config(int argc, char *argv[]);
static void cmd_can_status(int argc, char *argv[]);
static void cmd_ble_status(int argc, char *argv[]);
static void cmd_system_start(int argc, char *argv[]);
static void cmd_system_stop(int argc, char *argv[]);
static void cmd_system_status(int argc, char *argv[]);
static void cmd_help(int argc, char *argv[]);

/* ---- 命令注册表 --------------------------------------------------------- */
/*
 * 新增命令通常要做三件事：声明 cmd_xxx、在本表登记名称/参数/权限、实现函数。
 * 表尾 NULL 行是遍历停止标记，不能删除。命令按用途大致分为：私有协议维护、MIT
 * 协议维护、电机/关节诊断、系统与 BLE 状态、START/STOP。
 */
static const console_cmd_t s_cmd_table[] = {
	{"motor", "Select target: <A|B|configured_can_id>", 1,
	 CONSOLE_CMD_ALWAYS_AVAILABLE, cmd_motor},
	{"enable", "Motor enable (private protocol)", 0, CONSOLE_CMD_MAINTENANCE,
	 cmd_enable},
	{"disable", "Motor disable/reset (private protocol)", 0,
	 CONSOLE_CMD_MAINTENANCE, cmd_disable},
	{"move", "Motion: <Nm> <deg> <deg/s> <kp> <kd>", 5, CONSOLE_CMD_MAINTENANCE,
	 cmd_move},
	{"speed", "Speed: <A> <deg/s2> <deg/s>", 3, CONSOLE_CMD_MAINTENANCE,
	 cmd_speed},
	{"pos_pp", "PP position: <deg/s> <deg/s2> <deg>", 3,
	 CONSOLE_CMD_MAINTENANCE, cmd_pos_pp},
	{"pos_csp", "CSP position: <deg/s> <deg>", 2, CONSOLE_CMD_MAINTENANCE,
	 cmd_pos_csp},
	{"current", "Current control: <current>", 1, CONSOLE_CMD_MAINTENANCE,
	 cmd_current},
	{"set_zero", "Set RS01 motor mechanical zero (CAN)", 0,
	 CONSOLE_CMD_MAINTENANCE, cmd_set_zero},
	{"set_id", "Set motor CAN ID: <new_id>", 1, CONSOLE_CMD_MAINTENANCE,
	 cmd_set_id},
	{"save", "Save parameters to flash", 0, CONSOLE_CMD_MAINTENANCE, cmd_save},
	{"set_baud", "Change baud: <1=1M|2=500K|3=250K|4=125K>", 1,
	 CONSOLE_CMD_MAINTENANCE, cmd_set_baud},
	{"report", "Proactive report: <0=off|1=on>", 1, CONSOLE_CMD_MAINTENANCE,
	 cmd_report},
	{"switch_mit", "Switch protocol to MIT", 0, CONSOLE_CMD_MAINTENANCE,
	 cmd_switch_mit},
	{"switch_private", "Switch protocol to private", 0, CONSOLE_CMD_MAINTENANCE,
	 cmd_switch_private},
	{"read_param", "Read parameter (private): <reg_addr>", 1,
	 CONSOLE_CMD_MAINTENANCE, cmd_read_param},
	{"mit_enable", "MIT motor enable", 0, CONSOLE_CMD_MAINTENANCE,
	 cmd_mit_enable},
	{"mit_disable", "MIT motor disable", 0, CONSOLE_CMD_MAINTENANCE,
	 cmd_mit_disable},
	{"mit_control", "MIT: <deg> <deg/s> <kp> <kd> <Nm>", 5,
	 CONSOLE_CMD_MAINTENANCE, cmd_mit_control},
	{"mit_pos", "MIT position: <deg> <deg/s>", 2, CONSOLE_CMD_MAINTENANCE,
	 cmd_mit_pos},
	{"mit_speed", "MIT speed: <deg/s> <A>", 2, CONSOLE_CMD_MAINTENANCE,
	 cmd_mit_speed},
	{"mit_set_zero", "Set MIT motor mechanical zero (CAN)", 0,
	 CONSOLE_CMD_MAINTENANCE, cmd_mit_set_zero},
	{"mit_set_type", "MIT set motor type: <0-2>", 1, CONSOLE_CMD_MAINTENANCE,
	 cmd_mit_set_type},
	{"mit_set_id", "MIT set motor ID: <new_id>", 1, CONSOLE_CMD_MAINTENANCE,
	 cmd_mit_set_id},
	{"mit_save", "MIT save to flash", 0, CONSOLE_CMD_MAINTENANCE, cmd_mit_save},
	{"mit_report", "MIT proactive report: <0=off|1=on>", 1,
	 CONSOLE_CMD_MAINTENANCE, cmd_mit_report},
	{"mit_read", "MIT parameter read: <reg_addr>", 1, CONSOLE_CMD_MAINTENANCE,
	 cmd_mit_read},
	{"mit_write", "MIT parameter write: <reg_addr> <value>", 2,
	 CONSOLE_CMD_MAINTENANCE, cmd_mit_write},
	{"mit_error", "MIT error query/clear: <0-255|0xFF=clear>", 1,
	 CONSOLE_CMD_MAINTENANCE, cmd_mit_error},
	{"motor_type", "Show RS01 driver limits", 0, CONSOLE_CMD_ALWAYS_AVAILABLE,
	 cmd_motor_type},
	{"status", "Show motor feedback status", 0, CONSOLE_CMD_ALWAYS_AVAILABLE,
	 cmd_status},
	{"joint_status", "Show hip joint: <left|right>", 1,
	 CONSOLE_CMD_ALWAYS_AVAILABLE, cmd_joint_status},
	{"joint_zero", "Capture RAM software zero: <left|right>", 1,
	 CONSOLE_CMD_MAINTENANCE, cmd_joint_zero},
	{"joint_zero_clear", "Clear RAM software zero: <left|right>", 1,
	 CONSOLE_CMD_MAINTENANCE, cmd_joint_zero_clear},
	{"motion_mode",
	 "Select product mode: <hold|position|velocity|torque|damping|impedance> "
	 "[constant|sine]",
	 1, CONSOLE_CMD_MAINTENANCE, cmd_motion_mode},
	{"joint_command",
	 "Set IDLE five-tuple: <left|right> <deg> <deg/s> <Nm> <kp> <kd>",
	 6, CONSOLE_CMD_MAINTENANCE, cmd_joint_command},
	{"control_status", "Show the permanent 100 Hz control loop", 0,
	 CONSOLE_CMD_ALWAYS_AVAILABLE, cmd_control_status},
	{"sensor_status", "Show latest phase-9 sensor snapshot", 0,
	 CONSOLE_CMD_ALWAYS_AVAILABLE, cmd_sensor_status},
	{"gait_status", "Show latest gait phase estimate", 0,
	 CONSOLE_CMD_ALWAYS_AVAILABLE, cmd_gait_status},
	{"assist_status", "Show high-level assistance strategy", 0,
	 CONSOLE_CMD_ALWAYS_AVAILABLE, cmd_assist_status},
	{"assist_config",
	 "Set assist: <mode> <disabled|shadow> <strategy> <profile> <left%> <right%>",
	 6, CONSOLE_CMD_MAINTENANCE, cmd_assist_config},
	{"can_status", "Show TWAI controller and error counters", 0,
	 CONSOLE_CMD_ALWAYS_AVAILABLE, cmd_can_status},
	{"ble_status", "Show product BLE transport diagnostics", 0,
	 CONSOLE_CMD_ALWAYS_AVAILABLE, cmd_ble_status},
	{"start", "Request asynchronous exoskeleton START", 0,
	 CONSOLE_CMD_SYSTEM_REQUEST, cmd_system_start},
	{"stop", "Request asynchronous exoskeleton STOP", 0,
	 CONSOLE_CMD_SYSTEM_REQUEST, cmd_system_stop},
	{"system_status", "Show exoskeleton lifecycle state", 0,
	 CONSOLE_CMD_ALWAYS_AVAILABLE, cmd_system_status},
	{"imp_start", "Compatibility alias for start", 0,
	 CONSOLE_CMD_SYSTEM_REQUEST, cmd_system_start},
	{"imp_stop", "Compatibility alias for stop", 0, CONSOLE_CMD_SYSTEM_REQUEST,
	 cmd_system_stop},
	{"help", "Show this help", 0, CONSOLE_CMD_ALWAYS_AVAILABLE, cmd_help},
	{NULL, NULL, 0, CONSOLE_CMD_ALWAYS_AVAILABLE, NULL},
};

static void debug_printf(const char *fmt, ...) {
	/*
	 * 对 vprintf 的统一薄封装。fflush() 让命令结果立即出现在终端，而不是滞留在
	 * 标准输出缓冲区。禁止从 100 Hz 实时控制路径调用此函数。
	 */
	va_list args;
	va_start(args, fmt);
	(void)vprintf(fmt, args);
	va_end(args);
	fflush(stdout);
}

static bool parse_long_arg(const char *text, long min_value, long max_value,
						   long *value) {
	/*
	 * strtol(..., base=0) 同时接受十进制 `16` 和十六进制 `0x10`。除了转换本身，
	 * 还检查空输入、尾随垃圾字符、系统溢出以及调用者给出的上下界。
	 */
	if (text == NULL || value == NULL) {
		return false;
	}

	errno = 0;
	char *end = NULL;
	long parsed = strtol(text, &end, 0);
	if (errno != 0 || end == text || *end != '\0' || parsed < min_value ||
		parsed > max_value) {
		return false;
	}

	*value = parsed;
	return true;
}

static bool parse_float_arg(const char *text, float *value) {
	/* 浮点参数必须完整可解析且为有限值；NaN/Inf 不允许进入电机控制链。 */
	if (text == NULL || value == NULL) {
		return false;
	}

	errno = 0;
	char *end = NULL;
	float parsed = strtof(text, &end);
	if (errno != 0 || end == text || *end != '\0' || !isfinite(parsed)) {
		return false;
	}

	*value = parsed;
	return true;
}

static bool parse_joint_arg(const char *text, exo_hip_joint_id_t *joint) {
	/* 不区分大小写，同时接受 left/l 和 right/r；这里不接受 A/B 以免混淆语义。
	 */
	if (text == NULL || joint == NULL) {
		return false;
	}
	if (strcasecmp(text, "left") == 0 || strcasecmp(text, "l") == 0) {
		*joint = EXO_HIP_JOINT_LEFT;
		return true;
	}
	if (strcasecmp(text, "right") == 0 || strcasecmp(text, "r") == 0) {
		*joint = EXO_HIP_JOINT_RIGHT;
		return true;
	}
	return false;
}

static bool parse_float_args(char *argv[], int first, int count,
							 float values[]) {
	/* 批量解析命令的连续浮点参数，任意一个失败就整体失败，不使用半组旧值。 */
	for (int i = 0; i < count; ++i) {
		if (!parse_float_arg(argv[first + i], &values[i])) {
			return false;
		}
	}
	return true;
}

static exo_can_node_id_t selected_can_id(void) {
	/* A/B 到实际 CAN 节点号的映射始终向 RS01 驱动查询，不在控制台复制常量。 */
	exo_can_node_id_t can_id = 0;
	(void)rs01_get_can_id(s_selected_actuator, &can_id);
	return can_id;
}

static bool driver_command_succeeded(esp_err_t err) {
	/* 统一把 ESP-IDF 错误码翻译成可读名称，命令处理函数只需处理成功分支。 */
	if (err == ESP_OK) {
		return true;
	}

	debug_printf("ERROR: RS01 command failed: %s\r\n", esp_err_to_name(err));
	return false;
}

/* ================================================================
 * 命令处理函数
 *
 * 处理函数不再重复检查参数数量和维护权限，这两项已由统一分派器完成；它们仍负责
 * 检查每个参数的数值格式/范围，并通过模块公开 API 执行操作。
 * ================================================================ */

static void cmd_motor(int argc, char *argv[]) {
	/* 既允许选逻辑槽位 A/B，也允许输入已配置 CAN ID；最终仍保存为 A/B。 */
	exo_actuator_id_t actuator = EXO_ACTUATOR_COUNT;
	if (strcmp(argv[1], "A") == 0 || strcmp(argv[1], "a") == 0) {
		actuator = EXO_ACTUATOR_A;
	} else if (strcmp(argv[1], "B") == 0 || strcmp(argv[1], "b") == 0) {
		actuator = EXO_ACTUATOR_B;
	} else {
		long id;
		if (!parse_long_arg(argv[1], 0, 0x7F, &id) ||
			!rs01_find_actuator((exo_can_node_id_t)id, &actuator)) {
			debug_printf(
				"ERROR: motor must be A, B, or a configured CAN ID\r\n");
			return;
		}
	}

	s_selected_actuator = actuator;
	exo_can_node_id_t can_id = selected_can_id();
	debug_printf("OK: target motor set to 0x%02X (%d), actuator %c\r\n", can_id,
				 can_id, actuator == EXO_ACTUATOR_A ? 'A' : 'B');
}

static void cmd_enable(int argc, char *argv[]) {
	/* 这是直接面向当前选中电机的维护命令，不等同于外骨骼 START。 */
	if (driver_command_succeeded(rs01_enable(s_selected_actuator))) {
		debug_printf("OK: motor 0x%02X enable sent\r\n", selected_can_id());
	}
}

static void cmd_disable(int argc, char *argv[]) {
	/* 同样只操作选中电机；产品级双电机停止应使用 stop 状态机请求。 */
	if (driver_command_succeeded(rs01_disable(s_selected_actuator))) {
		debug_printf("OK: motor 0x%02X disable sent\r\n", selected_can_id());
	}
}

static void cmd_move(int argc, char *argv[]) {
	/* 参数顺序遵循 RS01 Motion 命令：力矩、位置、速度、KP、KD。 */
	float values[5];
	if (!parse_float_args(argv, 1, 5, values)) {
		debug_printf("ERROR: move arguments must be finite numbers\r\n");
		return;
	}
	/* UART 是人机接口，角度输入统一为度；调用 RS01 前集中换算为弧度。 */
	float t = values[0], pos_deg = values[1], spd_deg_s = values[2];
	float kp = values[3], kd = values[4];
	if (driver_command_succeeded(
			rs01_move(s_selected_actuator, t, exo_degrees_to_radians(pos_deg),
					  exo_degrees_to_radians(spd_deg_s), kp, kd))) {
		debug_printf("OK: motor 0x%02X move t=%.2fNm pos=%.2fdeg "
					 "spd=%.2fdeg/s kp=%.2f "
					 "kd=%.2f\r\n",
					 selected_can_id(), (double)t, (double)pos_deg,
					 (double)spd_deg_s, (double)kp, (double)kd);
	}
}

static void cmd_speed(int argc, char *argv[]) {
	/* 串口以 A、deg/s²、deg/s 表达，驱动边界把两个角运动量换为弧度单位。 */
	float values[3];
	if (!parse_float_args(argv, 1, 3, values)) {
		debug_printf("ERROR: speed arguments must be finite numbers\r\n");
		return;
	}
	float cur = values[0], acc_deg_s2 = values[1], spd_deg_s = values[2];
	if (driver_command_succeeded(rs01_speed_control(
			s_selected_actuator, cur, exo_degrees_to_radians(acc_deg_s2),
			exo_degrees_to_radians(spd_deg_s)))) {
		debug_printf("OK: motor 0x%02X speed cur=%.2fA acc=%.2fdeg/s2 "
					 "spd=%.2fdeg/s\r\n",
					 selected_can_id(), (double)cur, (double)acc_deg_s2,
					 (double)spd_deg_s);
	}
}

static void cmd_pos_pp(int argc, char *argv[]) {
	/* PP 是轮廓位置模式：速度/加速度限制加最终位置，均为手工电机维护用途。 */
	float values[3];
	if (!parse_float_args(argv, 1, 3, values)) {
		debug_printf("ERROR: pos_pp arguments must be finite numbers\r\n");
		return;
	}
	float spd_deg_s = values[0], acc_deg_s2 = values[1], ang_deg = values[2];
	if (driver_command_succeeded(rs01_position_pp_control(
			s_selected_actuator, exo_degrees_to_radians(spd_deg_s),
			exo_degrees_to_radians(acc_deg_s2),
			exo_degrees_to_radians(ang_deg)))) {
		debug_printf("OK: motor 0x%02X PP pos spd=%.2fdeg/s "
					 "acc=%.2fdeg/s2 ang=%.2fdeg\r\n",
					 selected_can_id(), (double)spd_deg_s, (double)acc_deg_s2,
					 (double)ang_deg);
	}
}

static void cmd_pos_csp(int argc, char *argv[]) {
	/* CSP 是周期同步位置接口；本控制台只发送一次，不在这里创建周期任务。 */
	float values[2];
	if (!parse_float_args(argv, 1, 2, values)) {
		debug_printf("ERROR: pos_csp arguments must be finite numbers\r\n");
		return;
	}
	float spd_deg_s = values[0], ang_deg = values[1];
	if (driver_command_succeeded(rs01_position_csp_control(
			s_selected_actuator, exo_degrees_to_radians(spd_deg_s),
			exo_degrees_to_radians(ang_deg)))) {
		debug_printf("OK: motor 0x%02X CSP pos spd=%.2fdeg/s ang=%.2fdeg\r\n",
					 selected_can_id(), (double)spd_deg_s, (double)ang_deg);
	}
}

static void cmd_current(int argc, char *argv[]) {
	/* 电流参数不是角度，不做 deg/rad 换算。 */
	float cur;
	if (!parse_float_arg(argv[1], &cur)) {
		debug_printf("ERROR: current must be a finite number\r\n");
		return;
	}
	if (driver_command_succeeded(
			rs01_current_control(s_selected_actuator, cur))) {
		debug_printf("OK: motor 0x%02X current cur=%.2f\r\n", selected_can_id(),
					 (double)cur);
	}
}

static void cmd_set_zero(int argc, char *argv[]) {
	/*
	 * 该命令写 RS01 自身机械零位，和 `joint_zero left/right` 的 RAM
	 * 软件零位不同。 两种归零名称相近但持久性、作用层级和 CAN 行为完全不同。
	 */
	if (driver_command_succeeded(
			rs01_set_mechanical_zero(s_selected_actuator))) {
		debug_printf("OK: motor 0x%02X set zero position sent\r\n",
					 selected_can_id());
	}
}

static void cmd_set_id(int argc, char *argv[]) {
	/* 节点号限制为经典 7 位范围；发送成功后驱动会按其 API 约定更新映射。 */
	long parsed;
	if (!parse_long_arg(argv[1], 0, 0x7F, &parsed)) {
		debug_printf("ERROR: new motor ID must be 0-127\r\n");
		return;
	}
	uint8_t new_id = (uint8_t)parsed;
	exo_can_node_id_t old_id = selected_can_id();
	if (driver_command_succeeded(
			rs01_set_can_id(s_selected_actuator, new_id))) {
		debug_printf("OK: motor 0x%02X change ID to %d\r\n", old_id, new_id);
	}
}

static void cmd_save(int argc, char *argv[]) {
	/* 要求电机把当前参数保存到自身 Flash；ESP32 的关节软件零位不在其中。 */
	if (driver_command_succeeded(rs01_save_parameters(s_selected_actuator))) {
		debug_printf("OK: motor 0x%02X save to flash sent\r\n",
					 selected_can_id());
	}
}

static void cmd_set_baud(int argc, char *argv[]) {
	/* 选择值 1..4 是手册定义的枚举，不是直接输入 1000000 这样的 bit/s。 */
	long parsed;
	if (!parse_long_arg(argv[1], 1, 4, &parsed)) {
		debug_printf("ERROR: baud rate selector must be 1-4\r\n");
		return;
	}
	uint8_t rate = (uint8_t)parsed;
	if (driver_command_succeeded(
			rs01_set_baud_rate(s_selected_actuator, rate))) {
		debug_printf("OK: motor 0x%02X baud rate change sent (rate=%d)\r\n",
					 selected_can_id(), rate);
	}
}

static void cmd_report(int argc, char *argv[]) {
	/* 主动上报开关只影响反馈流量，不使能力矩、不启动 100 Hz 控制律。 */
	long parsed;
	if (!parse_long_arg(argv[1], 0, 1, &parsed)) {
		debug_printf("ERROR: report must be 0 or 1\r\n");
		return;
	}
	uint8_t on = (uint8_t)parsed;
	esp_err_t err = on ? rs01_start_reporting(s_selected_actuator)
					   : rs01_stop_reporting(s_selected_actuator);
	if (driver_command_succeeded(err)) {
		debug_printf("OK: motor 0x%02X proactive report %s\r\n",
					 selected_can_id(), on ? "ON" : "OFF");
	}
}

static void cmd_switch_mit(int argc, char *argv[]) {
	/* 协议切换由电机掉电重启后生效，当前会话不会立即改用另一套帧格式。 */
	if (driver_command_succeeded(
			rs01_set_protocol(s_selected_actuator, RS01_PROTOCOL_MIT))) {
		debug_printf(
			"OK: motor 0x%02X MIT protocol selected; power-cycle motor "
			"to apply\r\n",
			selected_can_id());
	}
}

static void cmd_switch_private(int argc, char *argv[]) {
	/* 当前电机处于 MIT 协议时，用 MIT 管理帧请求切回私有协议。 */
	if (driver_command_succeeded(rs01_mit_set_protocol(
			s_selected_actuator, RS01_PROTOCOL_PRIVATE))) {
		debug_printf("OK: motor 0x%02X private protocol selected; power-cycle "
					 "motor to apply\r\n",
					 selected_can_id());
	}
}

static void cmd_read_param(int argc, char *argv[]) {
	/* 裸寄存器地址按手册解释；驱动只搬运数值，不猜测该寄存器的物理单位。 */
	long parsed;
	if (!parse_long_arg(argv[1], 0, UINT16_MAX, &parsed)) {
		debug_printf("ERROR: register address must be 0-0xFFFF\r\n");
		return;
	}
	uint16_t reg = (uint16_t)parsed;
	if (driver_command_succeeded(
			rs01_read_parameter(s_selected_actuator, reg))) {
		debug_printf("OK: motor 0x%02X read param 0x%04X sent\r\n",
					 selected_can_id(), reg);
	}
}

static void cmd_mit_enable(int argc, char *argv[]) {
	/* 以下 mit_* 命令均是单电机工程维护入口，不参与产品 START/STOP 编排。 */
	if (driver_command_succeeded(rs01_mit_enable(s_selected_actuator))) {
		debug_printf("OK: motor 0x%02X MIT enable sent\r\n", selected_can_id());
	}
}

static void cmd_mit_disable(int argc, char *argv[]) {
	if (driver_command_succeeded(rs01_mit_disable(s_selected_actuator))) {
		debug_printf("OK: motor 0x%02X MIT disable sent\r\n",
					 selected_can_id());
	}
}

static void cmd_mit_control(int argc, char *argv[]) {
	/* MIT 参数仍在 UART 使用 deg/deg/s，调用驱动前集中换成 rad/rad/s。 */
	float values[5];
	if (!parse_float_args(argv, 1, 5, values)) {
		debug_printf("ERROR: mit_control arguments must be finite numbers\r\n");
		return;
	}
	float ang_deg = values[0], spd_deg_s = values[1], kp = values[2];
	float kd = values[3], t = values[4];
	if (driver_command_succeeded(rs01_mit_send_motion(
			s_selected_actuator, exo_degrees_to_radians(ang_deg),
			exo_degrees_to_radians(spd_deg_s), kp, kd, t))) {
		debug_printf("OK: motor 0x%02X MIT control ang=%.2fdeg "
					 "spd=%.2fdeg/s kp=%.2f "
					 "kd=%.2f t=%.2f\r\n",
					 selected_can_id(), (double)ang_deg, (double)spd_deg_s,
					 (double)kp, (double)kd, (double)t);
	}
}

static void cmd_mit_pos(int argc, char *argv[]) {
	/* 位置和速度是电机输出轴目标，不会自动套用人体左右关节映射。 */
	float values[2];
	if (!parse_float_args(argv, 1, 2, values)) {
		debug_printf("ERROR: mit_pos arguments must be finite numbers\r\n");
		return;
	}
	float pos_deg = values[0], spd_deg_s = values[1];
	if (driver_command_succeeded(rs01_mit_position_control(
			s_selected_actuator, exo_degrees_to_radians(pos_deg),
			exo_degrees_to_radians(spd_deg_s)))) {
		debug_printf("OK: motor 0x%02X MIT position pos=%.2fdeg "
					 "spd=%.2fdeg/s\r\n",
					 selected_can_id(), (double)pos_deg, (double)spd_deg_s);
	}
}

static void cmd_mit_speed(int argc, char *argv[]) {
	/* 速度换算为 rad/s；电流保持安培单位原样传递。 */
	float values[2];
	if (!parse_float_args(argv, 1, 2, values)) {
		debug_printf("ERROR: mit_speed arguments must be finite numbers\r\n");
		return;
	}
	float spd_deg_s = values[0], cur = values[1];
	if (driver_command_succeeded(rs01_mit_speed_control(
			s_selected_actuator, exo_degrees_to_radians(spd_deg_s), cur))) {
		debug_printf("OK: motor 0x%02X MIT speed spd=%.2fdeg/s cur=%.2fA\r\n",
					 selected_can_id(), (double)spd_deg_s, (double)cur);
	}
}

static void cmd_mit_set_zero(int argc, char *argv[]) {
	if (driver_command_succeeded(rs01_mit_set_zero(s_selected_actuator))) {
		debug_printf("OK: motor 0x%02X MIT set zero sent\r\n",
					 selected_can_id());
	}
}

static void cmd_mit_set_type(int argc, char *argv[]) {
	long parsed;
	if (!parse_long_arg(argv[1], 0, 2, &parsed)) {
		debug_printf("ERROR: MIT motor type must be 0-2\r\n");
		return;
	}
	uint8_t type = (uint8_t)parsed;
	if (driver_command_succeeded(
			rs01_mit_set_operation_mode(s_selected_actuator, type))) {
		debug_printf("OK: motor 0x%02X MIT set motor type to %d\r\n",
					 selected_can_id(), type);
	}
}

static void cmd_mit_set_id(int argc, char *argv[]) {
	long parsed;
	if (!parse_long_arg(argv[1], 0, 0x7F, &parsed)) {
		debug_printf("ERROR: new MIT motor ID must be 0-127\r\n");
		return;
	}
	uint8_t new_id = (uint8_t)parsed;
	if (driver_command_succeeded(
			rs01_mit_set_can_id(s_selected_actuator, new_id))) {
		debug_printf("OK: motor 0x%02X MIT set motor ID to %d\r\n",
					 selected_can_id(), new_id);
	}
}

static void cmd_mit_save(int argc, char *argv[]) {
	if (driver_command_succeeded(
			rs01_mit_save_parameters(s_selected_actuator))) {
		debug_printf("OK: motor 0x%02X MIT save sent\r\n", selected_can_id());
	}
}

static void cmd_mit_report(int argc, char *argv[]) {
	long parsed;
	if (!parse_long_arg(argv[1], 0, 1, &parsed)) {
		debug_printf("ERROR: mit_report must be 0 or 1\r\n");
		return;
	}
	uint8_t on = (uint8_t)parsed;
	esp_err_t err = on ? rs01_mit_start_reporting(s_selected_actuator)
					   : rs01_mit_stop_reporting(s_selected_actuator);
	if (driver_command_succeeded(err)) {
		debug_printf("OK: motor 0x%02X MIT proactive report %s\r\n",
					 selected_can_id(), on ? "ON" : "OFF");
	}
}

static void cmd_mit_read(int argc, char *argv[]) {
	long parsed;
	if (!parse_long_arg(argv[1], 0, UINT16_MAX, &parsed)) {
		debug_printf("ERROR: MIT register address must be 0-0xFFFF\r\n");
		return;
	}
	if (driver_command_succeeded(
			rs01_mit_read_parameter(s_selected_actuator, (uint16_t)parsed))) {
		debug_printf("OK: motor 0x%02X MIT read param 0x%04lX sent\r\n",
					 selected_can_id(), parsed);
	}
}

static void cmd_mit_write(int argc, char *argv[]) {
	/* 裸写寄存器用于对照电机手册调试，value 不自动进行 deg/rad 换算。 */
	long parsed;
	float value;
	if (!parse_long_arg(argv[1], 0, UINT16_MAX, &parsed) ||
		!parse_float_arg(argv[2], &value)) {
		debug_printf("ERROR: MIT write requires <reg_addr> <finite_value>\r\n");
		return;
	}
	if (driver_command_succeeded(rs01_mit_write_parameter_float(
			s_selected_actuator, (uint16_t)parsed, value))) {
		debug_printf("OK: motor 0x%02X MIT write param 0x%04lX sent\r\n",
					 selected_can_id(), parsed);
	}
}

static void cmd_mit_error(int argc, char *argv[]) {
	long parsed;
	if (!parse_long_arg(argv[1], 0, UINT8_MAX, &parsed)) {
		debug_printf(
			"ERROR: MIT error command must be 0-255 (0xFF clears)\r\n");
		return;
	}
	if (driver_command_succeeded(rs01_mit_clear_or_read_fault(
			s_selected_actuator, (uint8_t)parsed))) {
		debug_printf("OK: motor 0x%02X MIT error command 0x%02lX sent\r\n",
					 selected_can_id(), parsed);
	}
}

static void cmd_motor_type(int argc, char *argv[]) {
	/* 显示当前 RS01 型号的驱动限幅，便于核对 KP/KD/位置/速度合法范围。 */
	const rs01_limits_t *limits = rs01_get_limits();
	debug_printf("Motor driver: RS01\r\n");
	/* 工程台面向人显示度；驱动量程对象仍忠实保留 RS01 手册的弧度单位。 */
	debug_printf("  P:  [%.2f, %.2f] deg\r\n",
				 (double)exo_radians_to_degrees(limits->position_min_rad),
				 (double)exo_radians_to_degrees(limits->position_max_rad));
	debug_printf("  V:  [%.2f, %.2f] deg/s\r\n",
				 (double)exo_radians_to_degrees(limits->velocity_min_rad_s),
				 (double)exo_radians_to_degrees(limits->velocity_max_rad_s));
	debug_printf("  KP: [%.2f, %.2f]\r\n", (double)limits->kp_min,
				 (double)limits->kp_max);
	debug_printf("  KD: [%.2f, %.2f]\r\n", (double)limits->kd_min,
				 (double)limits->kd_max);
	debug_printf("  T:  [%.2f, %.2f] Nm\r\n", (double)limits->torque_min_nm,
				 (double)limits->torque_max_nm);
}

static void cmd_status(int argc, char *argv[]) {
	/*
	 * 一次性读取所选电机的反馈和通信统计快照。这里按需打印，而不是每 10 ms 打印
	 * 主动反馈，避免 UART 带宽反过来影响 CAN 接收与控制周期。
	 */
	rs01_feedback_snapshot_t snapshot;
	if (!rs01_get_feedback(s_selected_actuator, &snapshot)) {
		debug_printf("ERROR: cannot read RS01 feedback snapshot\r\n");
		return;
	}

	exo_can_node_id_t can_id = selected_can_id();
	uint64_t age_us =
		snapshot.valid ? (uint64_t)esp_timer_get_time() - snapshot.timestamp_us
					   : 0U;
	double rate_hz = snapshot.last_interval_us > 0U
						 ? 1000000.0 / (double)snapshot.last_interval_us
						 : 0.0;

	debug_printf("===== Motor 0x%02X Feedback =====\r\n", can_id);
	debug_printf("  Actuator:    %c\r\n",
				 s_selected_actuator == EXO_ACTUATOR_A ? 'A' : 'B');
	debug_printf("  Valid:       %s\r\n", snapshot.valid ? "YES" : "no");
	debug_printf("  Online:      %s\r\n", snapshot.online ? "YES" : "no");
	debug_printf("  Reporting:   %s (requested)\r\n",
				 snapshot.reporting_requested ? "ON" : "OFF");
	debug_printf("  Enabled:     %s (requested)\r\n",
				 snapshot.enabled_requested ? "YES" : "no");
	debug_printf("  Feedback count: %lu\r\n",
				 (unsigned long)snapshot.feedback_count);
	debug_printf("    Type 2:        %lu\r\n",
				 (unsigned long)snapshot.type2_feedback_count);
	debug_printf("    Type 24:       %lu\r\n",
				 (unsigned long)snapshot.type24_feedback_count);
	debug_printf("  Invalid frames: %lu\r\n",
				 (unsigned long)snapshot.invalid_frame_count);
	debug_printf("  Last interval: %llu us (%.2f Hz)\r\n",
				 (unsigned long long)snapshot.last_interval_us, rate_hz);
	debug_printf("  Feedback age: %llu us\r\n", (unsigned long long)age_us);
	debug_printf("  Angle raw:   %.3f deg (periodic)\r\n",
				 (double)exo_radians_to_degrees(snapshot.sample.position_rad));
	if (snapshot.continuous_position_valid) {
		debug_printf("  Angle cont.:  %.3f deg (periods=%ld epoch=%lu rebase=%lu)\r\n",
					 (double)exo_radians_to_degrees(
						 snapshot.continuous_position_rad),
					 (long)snapshot.position_wrap_count,
					 (unsigned long)snapshot.position_epoch,
					 (unsigned long)snapshot.position_rebase_count);
	} else {
		debug_printf("  Angle cont.:  N/A (%s)\r\n",
					 snapshot.position_reset_pending
						 ? "mechanical-zero confirmation pending"
						 : "tracker not initialized");
	}
	debug_printf(
		"  Speed:       %.3f deg/s\r\n",
		(double)exo_radians_to_degrees(snapshot.sample.velocity_rad_s));
	debug_printf("  Torque:      %.3f Nm\r\n",
				 (double)snapshot.sample.torque_nm);
	debug_printf("  Temperature: %.1f C\r\n",
				 (double)snapshot.sample.temperature_c);
	debug_printf("  Mode status: %d ", (int)snapshot.mode);
	switch (snapshot.mode) {
	case RS01_MODE_STATUS_RESET:
		debug_printf("(REST)\r\n");
		break;
	case RS01_MODE_STATUS_CALIBRATION:
		debug_printf("(CALI)\r\n");
		break;
	case RS01_MODE_STATUS_RUNNING:
		debug_printf("(RUNNING)\r\n");
		break;
	default:
		debug_printf("(UNKNOWN)\r\n");
		break;
	}
	debug_printf("  Motor mode:  %d\r\n", (int)snapshot.control_mode);
	debug_printf("  Fault raw:   0x%08lX\r\n",
				 (unsigned long)snapshot.fault_raw);
	debug_printf("  Warning raw: 0x%08lX\r\n",
				 (unsigned long)snapshot.warning_raw);
	debug_printf("  Faults:\r\n");
	debug_printf("    Uncalibrated:         %s\r\n",
				 snapshot.faults.uncalibrated ? "YES" : "no");
	debug_printf("    Stall/Overload:       %s\r\n",
				 snapshot.faults.stall_overload ? "YES" : "no");
	debug_printf("    Magnetic Encoder:     %s\r\n",
				 snapshot.faults.magnetic_encoder ? "YES" : "no");
	debug_printf("    Over Temperature:     %s\r\n",
				 snapshot.faults.over_temperature ? "YES" : "no");
	debug_printf(
		"    Drive/Phase Current:  %s\r\n",
		(snapshot.faults.driver_chip || snapshot.faults.phase_over_current)
			? "YES"
			: "no");
	debug_printf("    Under Voltage:        %s\r\n",
				 snapshot.faults.under_voltage ? "YES" : "no");
}

static const char *joint_name(exo_hip_joint_id_t joint) {
	/* 只用于人类可读输出，不参与左右映射判断。 */
	return joint == EXO_HIP_JOINT_LEFT ? "LEFT" : "RIGHT";
}

static const char *joint_direction_name(exo_direction_t direction) {
	/* 把 +1/-1/未知翻译为诊断文本，便于核对实机安装方向。 */
	if (direction == EXO_DIRECTION_POSITIVE) {
		return "+1";
	}
	if (direction == EXO_DIRECTION_NEGATIVE) {
		return "-1";
	}
	return "UNKNOWN";
}

static void print_joint_status(exo_hip_joint_id_t joint) {
	/*
	 * 同时展示电机原始坐标和人体关节坐标，新手可用两者核对：是否完成软件归零、
	 * 左右映射是否正确、屈曲时关节角是否都朝正方向变化。
	 */
	hip_joint_snapshot_t snapshot;
	if (!hip_joint_get_snapshot(joint, &snapshot)) {
		debug_printf("ERROR: cannot read hip joint snapshot\r\n");
		return;
	}

	uint64_t age_us = 0U;
	const uint64_t now_us = (uint64_t)esp_timer_get_time();
	if (snapshot.feedback_valid && now_us >= snapshot.feedback_timestamp_us) {
		age_us = now_us - snapshot.feedback_timestamp_us;
	}
	const bool raw_available =
		snapshot.mapping_valid && snapshot.feedback_valid &&
		isfinite(snapshot.raw_actuator_sample.position_rad) &&
		isfinite(snapshot.raw_actuator_sample.velocity_rad_s) &&
		isfinite(snapshot.raw_actuator_sample.torque_nm) &&
		isfinite(snapshot.raw_actuator_sample.temperature_c);
	const bool joint_sample_available = snapshot.joint_valid;

	debug_printf("===== Hip Joint %s =====\r\n", joint_name(joint));
	if (snapshot.mapping_valid) {
		debug_printf("  Mapping:       actuator %c (valid)\r\n",
					 snapshot.actuator == EXO_ACTUATOR_A ? 'A' : 'B');
	} else {
		debug_printf("  Mapping:       UNMAPPED\r\n");
	}
	debug_printf("  Direction:     %s (%s)\r\n",
				 joint_direction_name(snapshot.direction),
				 snapshot.direction_valid ? "valid" : "INVALID");
	if (snapshot.zero_valid) {
		debug_printf("  Software zero: %.3f deg continuous (valid, RAM only)\r\n",
					 (double)snapshot.software_zero_motor_deg);
		debug_printf("  Zero on wire:  %.3f deg periodic\r\n",
					 (double)snapshot.software_zero_periodic_motor_deg);
		debug_printf(
			"  Zero source:   feedback count=%lu timestamp=%llu us epoch=%lu (%s)\r\n",
			(unsigned long)snapshot.zero_source_feedback_count,
			(unsigned long long)snapshot.zero_source_timestamp_us,
			(unsigned long)snapshot.zero_source_position_epoch,
			snapshot.zero_epoch_matches ? "current" : "STALE");
	} else if (snapshot.zero_source_feedback_count != 0U) {
		debug_printf("  Software zero: %.3f deg (STALE, RAM only)\r\n",
					 (double)snapshot.software_zero_motor_deg);
		debug_printf(
			"  Zero source:   feedback count=%lu timestamp=%llu us old_epoch=%lu current_epoch=%lu\r\n",
			(unsigned long)snapshot.zero_source_feedback_count,
			(unsigned long long)snapshot.zero_source_timestamp_us,
			(unsigned long)snapshot.zero_source_position_epoch,
			(unsigned long)snapshot.actuator_position_epoch);
	} else {
		debug_printf("  Software zero: N/A (INVALID, RAM only)\r\n");
		debug_printf("  Zero source:   none\r\n");
	}
	if (!snapshot.mapping_valid) {
		debug_printf("  Feedback:      N/A (joint unmapped)\r\n");
	} else if (!snapshot.feedback_valid) {
		debug_printf(
			"  Feedback:      INVALID / OFFLINE, count=%lu, age=N/A\r\n",
			(unsigned long)snapshot.source_feedback_count);
	} else {
		debug_printf("  Feedback:      valid / %s, count=%lu, age=%llu us\r\n",
					 snapshot.online ? "online" : "OFFLINE",
					 (unsigned long)snapshot.source_feedback_count,
					 (unsigned long long)age_us);
	}
	if (raw_available) {
		/* 原始反馈内部仍是 rad，此处只改变人机显示单位。 */
		debug_printf("  Raw actuator:  q=%.3f deg dq=%.3f deg/s tau=%.3f Nm "
					 "temp=%.1f C\r\n",
					 (double)exo_radians_to_degrees(
						 snapshot.raw_actuator_sample.position_rad),
					 (double)exo_radians_to_degrees(
						 snapshot.raw_actuator_sample.velocity_rad_s),
					 (double)snapshot.raw_actuator_sample.torque_nm,
					 (double)snapshot.raw_actuator_sample.temperature_c);
		if (snapshot.continuous_position_valid) {
			debug_printf("  Continuous q: %.3f deg (periods=%ld epoch=%lu)\r\n",
						 (double)exo_radians_to_degrees(
							 snapshot.continuous_actuator_position_rad),
						 (long)snapshot.actuator_position_wrap_count,
						 (unsigned long)snapshot.actuator_position_epoch);
		}
	} else {
		debug_printf("  Raw actuator:  N/A\r\n");
	}
	if (joint_sample_available) {
		debug_printf(
			"  Joint:         q=%.3f deg dq=%.3f deg/s tau=%.3f Nm\r\n",
			(double)snapshot.joint_sample.position_deg,
			(double)snapshot.joint_sample.velocity_deg_s,
			(double)snapshot.joint_sample.torque_nm);
	} else {
		debug_printf("  Joint:         N/A\r\n");
	}
	debug_printf("  Joint valid:   %s\r\n",
				 snapshot.joint_valid ? "YES" : "no");
}

static bool command_joint_from_arg(char *argv[], exo_hip_joint_id_t *joint) {
	/* 关节类命令共用同一个 left/right 错误提示。 */
	if (parse_joint_arg(argv[1], joint)) {
		return true;
	}
	debug_printf("ERROR: hip joint must be left or right\r\n");
	return false;
}

static void cmd_joint_status(int argc, char *argv[]) {
	exo_hip_joint_id_t joint;
	if (command_joint_from_arg(argv, &joint)) {
		print_joint_status(joint);
	}
}

static void cmd_joint_zero(int argc, char *argv[]) {
	/*
	 * 操作者应先把指定髋关节摆在人体中立位。成功后只锁存当前有效反馈到 RAM，
	 * 不会让电机自动寻找中立位，重启设备后需要重新执行。
	 */
	exo_hip_joint_id_t joint;
	if (!command_joint_from_arg(argv, &joint)) {
		return;
	}

	esp_err_t err = hip_joint_set_zero_from_current(joint);
	if (err != ESP_OK) {
		debug_printf("ERROR: %s hip software zero not captured: %s\r\n",
					 joint_name(joint), esp_err_to_name(err));
		return;
	}
	debug_printf(
		"OK: %s hip RAM software zero captured; motor zero unchanged\r\n",
		joint_name(joint));
	print_joint_status(joint);
}

static void cmd_joint_zero_clear(int argc, char *argv[]) {
	/* 清除后关节有效条件不再满足，START 前置检查会据此阻止使用旧零位。 */
	exo_hip_joint_id_t joint;
	if (!command_joint_from_arg(argv, &joint)) {
		return;
	}

	esp_err_t err = hip_joint_clear_zero(joint);
	if (err != ESP_OK) {
		debug_printf("ERROR: %s hip software zero not cleared: %s\r\n",
					 joint_name(joint), esp_err_to_name(err));
		return;
	}
	debug_printf(
		"OK: %s hip RAM software zero cleared; motor zero unchanged\r\n",
		joint_name(joint));
}

static bool parse_control_mode_arg(const char *text,
							   exo_control_mode_t *mode) {
	if (text == NULL || mode == NULL) {
		return false;
	}
	if (strcasecmp(text, "hold") == 0) {
		*mode = EXO_CONTROL_MODE_HOLD;
	} else if (strcasecmp(text, "position") == 0) {
		*mode = EXO_CONTROL_MODE_POSITION;
	} else if (strcasecmp(text, "velocity") == 0) {
		*mode = EXO_CONTROL_MODE_VELOCITY;
	} else if (strcasecmp(text, "torque") == 0) {
		*mode = EXO_CONTROL_MODE_TORQUE;
	} else if (strcasecmp(text, "damping") == 0) {
		*mode = EXO_CONTROL_MODE_DAMPING;
	} else if (strcasecmp(text, "impedance") == 0) {
		*mode = EXO_CONTROL_MODE_IMPEDANCE;
	} else {
		return false;
	}
	return true;
}

static bool parse_reference_arg(const char *text,
								exo_reference_source_t *reference) {
	if (text == NULL || reference == NULL) {
		return false;
	}
	if (strcasecmp(text, "constant") == 0) {
		*reference = EXO_REFERENCE_CONSTANT;
		return true;
	}
	if (strcasecmp(text, "sine") == 0) {
		*reference = EXO_REFERENCE_SINE;
		return true;
	}
	return false;
}

static void cmd_motion_mode(int argc, char *argv[]) {
	/*
	 * 该命令只修改 selected 配置，不使能电机。统一分派器已取得 IDLE 维护权，
	 * 所以 START 不会在模式和参考源更新到一半时锁存配置。
	 */
	if (argc < 2 || argc > 3) {
		debug_printf("Usage: motion_mode <hold|position|velocity|torque|"
					 "damping|impedance> [constant|sine]\r\n");
		return;
	}
	exo_control_mode_t mode;
	exo_reference_source_t reference = EXO_REFERENCE_CONSTANT;
	if (!parse_control_mode_arg(argv[1], &mode) ||
		(argc == 3 && !parse_reference_arg(argv[2], &reference))) {
		debug_printf("ERROR: invalid product mode/reference\r\n");
		return;
	}
	const esp_err_t err = exo_controller_set_mode(mode, reference);
	if (err != ESP_OK) {
		debug_printf("ERROR: motion mode not changed: %s\r\n",
					 esp_err_to_name(err));
		return;
	}
	debug_printf("OK: selected mode=%s reference=%s; use control_status to "
				 "read back before START\r\n",
				 exo_control_mode_name(mode),
				 exo_reference_source_name(reference));
}

static void cmd_joint_command(int argc, char *argv[]) {
	if (argc != 7) {
		debug_printf("Usage: joint_command <left|right> <deg> <deg/s> <Nm> "
					 "<kp> <kd>\r\n");
		return;
	}
	exo_hip_joint_id_t joint;
	float values[5];
	if (!command_joint_from_arg(argv, &joint) ||
		!parse_float_args(argv, 2, 5, values)) {
		debug_printf("ERROR: five-tuple must contain five finite numbers\r\n");
		return;
	}
	const exo_joint_command_t command = {
		.position_deg = values[0],
		.velocity_deg_s = values[1],
		.torque_ff_nm = values[2],
		.kp = values[3],
		.kd = values[4],
	};
	const esp_err_t err = exo_controller_set_joint_command(joint, &command);
	if (err != ESP_OK) {
		debug_printf("ERROR: %s five-tuple rejected: %s\r\n", joint_name(joint),
					 esp_err_to_name(err));
		return;
	}
	debug_printf("OK: %s selected q=%.3f deg dq=%.3f deg/s tau=%.3f Nm "
				 "kp=%.3f kd=%.3f\r\n",
				 joint_name(joint), (double)command.position_deg,
				 (double)command.velocity_deg_s,
				 (double)command.torque_ff_nm, (double)command.kp,
				 (double)command.kd);
}

static void cmd_control_status(int argc, char *argv[]) {
	/*
	 * 展示永久 100 Hz 任务的频率、周期抖动、执行时间和输入反馈。configured rate
	 * 是目标值，mean/last/min/max 才是实际测量值；deadline miss 表示执行超期。
	 */
	exo_control_loop_snapshot_t snapshot;
	if (!exo_control_loop_get_snapshot(&snapshot)) {
		debug_printf("ERROR: control loop is not initialized\r\n");
		return;
	}

	const double measured_hz = snapshot.mean_period_us > 0U
								   ? 1000000.0 / (double)snapshot.mean_period_us
								   : 0.0;

	debug_printf("===== 100 Hz Control Loop =====\r\n");
	debug_printf("  Initialized:       %s\r\n",
				 snapshot.initialized ? "YES" : "no");
	debug_printf("  Task alive:        %s\r\n",
				 snapshot.task_alive ? "YES" : "no");
	debug_printf("  Task identity:     %p\r\n",
				 (void *)snapshot.task_handle_identity);
	debug_printf("  Configured rate:   %lu Hz (%lu us)\r\n",
				 (unsigned long)snapshot.configured_frequency_hz,
				 (unsigned long)snapshot.target_period_us);
	debug_printf("  Cycle count:       %llu\r\n",
				 (unsigned long long)snapshot.cycle_count);
	debug_printf("  Period samples:    %llu; outside +/- %lu us: %llu\r\n",
				 (unsigned long long)snapshot.period_sample_count,
				 (unsigned long)snapshot.period_tolerance_us,
				 (unsigned long long)snapshot.period_out_of_tolerance_count);
	debug_printf("  Last cycle:        %llu us\r\n",
				 (unsigned long long)snapshot.last_cycle_start_us);
	debug_printf(
		"  Period us:         last=%lu min=%lu max=%lu mean=%lu (%.3f Hz)\r\n",
		(unsigned long)snapshot.last_period_us,
		(unsigned long)snapshot.min_period_us,
		(unsigned long)snapshot.max_period_us,
		(unsigned long)snapshot.mean_period_us, measured_hz);
	debug_printf("  Period error us:   last=%ld max_abs=%lu\r\n",
				 (long)snapshot.last_period_error_us,
				 (unsigned long)snapshot.max_abs_period_error_us);
	debug_printf("  Execution us:      last=%lu max=%lu\r\n",
				 (unsigned long)snapshot.last_execution_us,
				 (unsigned long)snapshot.max_execution_us);
	debug_printf("  Deadline misses:   %llu\r\n",
				 (unsigned long long)snapshot.deadline_miss_count);
	debug_printf("  Skipped cycles:    %llu\r\n",
				 (unsigned long long)snapshot.skipped_cycle_count);
	debug_printf("  Output errors:     %llu (last=%s)\r\n",
				 (unsigned long long)snapshot.output_error_count,
				 esp_err_to_name(snapshot.last_output_error));
	debug_printf("  Stack free min:    %lu bytes\r\n",
				 (unsigned long)snapshot.stack_high_water_mark_bytes);
	debug_printf(
		"  LEFT input:        valid=%s count=%lu timestamp=%llu us\r\n",
		snapshot.left.joint_valid ? "YES" : "no",
		(unsigned long)snapshot.left.feedback_count,
		(unsigned long long)snapshot.left.feedback_timestamp_us);
	debug_printf(
		"  RIGHT input:       valid=%s count=%lu timestamp=%llu us\r\n",
		snapshot.right.joint_valid ? "YES" : "no",
		(unsigned long)snapshot.right.feedback_count,
		(unsigned long long)snapshot.right.feedback_timestamp_us);

	/* 控制任务诊断之后再显示模式控制器；两份快照彼此独立且都只读。 */
	exo_controller_snapshot_t controller;
	if (!exo_controller_get_snapshot(&controller)) {
		debug_printf("  Joint controller:  not initialized\r\n");
		return;
	}
	debug_printf("===== Joint Controller =====\r\n");
	debug_printf("  Armed/tick:        %s/%s\r\n",
				 controller.armed ? "YES" : "no",
				 controller.tick_active ? "ACTIVE" : "idle");
	debug_printf("  Selected:          %s + %s (generation=%lu)\r\n",
				 exo_control_mode_name(controller.selected.mode),
				 exo_reference_source_name(controller.selected.reference),
				 (unsigned long)controller.selected_generation);
	debug_printf("  Active:            %s + %s (generation=%lu)\r\n",
				 exo_control_mode_name(controller.active.mode),
				 exo_reference_source_name(controller.active.reference),
				 (unsigned long)controller.active_generation);
	debug_printf("  Selected LEFT:     q=%.3f deg dq=%.3f deg/s tau=%.3f Nm "
				 "kp=%.3f kd=%.3f\r\n",
				 (double)controller.selected.command.left.position_deg,
				 (double)controller.selected.command.left.velocity_deg_s,
				 (double)controller.selected.command.left.torque_ff_nm,
				 (double)controller.selected.command.left.kp,
				 (double)controller.selected.command.left.kd);
	debug_printf("  Selected RIGHT:    q=%.3f deg dq=%.3f deg/s tau=%.3f Nm "
				 "kp=%.3f kd=%.3f\r\n",
				 (double)controller.selected.command.right.position_deg,
				 (double)controller.selected.command.right.velocity_deg_s,
				 (double)controller.selected.command.right.torque_ff_nm,
				 (double)controller.selected.command.right.kp,
				 (double)controller.selected.command.right.kd);
	debug_printf("  Last LEFT output:  q=%.3f deg dq=%.3f deg/s tau=%.3f Nm "
				 "kp=%.3f kd=%.3f\r\n",
				 (double)controller.last_output.left.position_deg,
				 (double)controller.last_output.left.velocity_deg_s,
				 (double)controller.last_output.left.torque_ff_nm,
				 (double)controller.last_output.left.kp,
				 (double)controller.last_output.left.kd);
	debug_printf("  Last RIGHT output: q=%.3f deg dq=%.3f deg/s tau=%.3f Nm "
				 "kp=%.3f kd=%.3f\r\n",
				 (double)controller.last_output.right.position_deg,
				 (double)controller.last_output.right.velocity_deg_s,
				 (double)controller.last_output.right.torque_ff_nm,
				 (double)controller.last_output.right.kp,
				 (double)controller.last_output.right.kd);
	debug_printf("  Output/error:      %llu/%llu (last=%s)\r\n",
				 (unsigned long long)controller.output_count,
				 (unsigned long long)controller.output_error_count,
				 esp_err_to_name(controller.last_error));
}

/* 阶段9三个状态命令都只复制最近发布快照，绝不会触发一次新的传感器采样。 */
static void print_sensor_side(const char *name,
				  const exo_sensor_joint_snapshot_t *joint) {
	debug_printf(
		"  %-5s: available=%s valid=%s online=%s age=%lu us count=%lu "
		"epoch=%lu\r\n",
		name, joint->snapshot_available ? "YES" : "no",
		joint->joint_valid ? "YES" : "no", joint->online ? "YES" : "no",
		(unsigned long)joint->age_us,
		(unsigned long)joint->source_feedback_count,
		(unsigned long)joint->position_epoch);
	debug_printf(
		"         q=%.3f deg dq=%.3f deg/s motor_tau_est=%.3f Nm temp=%.1f C "
		"source=%llu us\r\n",
		(double)joint->joint_sample.position_deg,
		(double)joint->joint_sample.velocity_deg_s,
		(double)joint->joint_sample.torque_nm,
		(double)joint->joint_sample.temperature_c,
		(unsigned long long)joint->source_timestamp_us);
}

static void cmd_sensor_status(int argc, char *argv[]) {
	(void)argc;
	(void)argv;
	exo_control_loop_snapshot_t control;
	if (!exo_control_loop_get_snapshot(&control)) {
		debug_printf("ERROR: control loop is not initialized\r\n");
		return;
	}
	const exo_sensor_snapshot_t *sensor = &control.sensor_snapshot;
	debug_printf("===== Phase 9 Sensor Snapshot =====\r\n");
	debug_printf("  Sequence/time:     %lu / %lld us; reset_epoch=%lu\r\n",
				 (unsigned long)sensor->sequence,
				 (long long)sensor->captured_us,
				 (unsigned long)sensor->reset_epoch);
	debug_printf(
		"  Sensor masks:      available=0x%08lX valid=0x%08lX quality=0x%08lX\r\n",
		(unsigned long)sensor->available_sensor_mask,
		(unsigned long)sensor->valid_sensor_mask,
		(unsigned long)sensor->quality_flags);
	debug_printf("  Pair source skew:  %lu us\r\n",
				 (unsigned long)sensor->pair_skew_us);
	print_sensor_side("LEFT", &sensor->left);
	print_sensor_side("RIGHT", &sensor->right);
}

static void cmd_gait_status(int argc, char *argv[]) {
	(void)argc;
	(void)argv;
	exo_control_loop_snapshot_t control;
	if (!exo_control_loop_get_snapshot(&control)) {
		debug_printf("ERROR: control loop is not initialized\r\n");
		return;
	}
	const exo_gait_estimate_t *gait = &control.gait_estimate;
	const bool left_valid =
		(gait->valid_flags & EXO_GAIT_VALID_LEFT_PHASE) != 0U;
	const bool right_valid =
		(gait->valid_flags & EXO_GAIT_VALID_RIGHT_PHASE) != 0U;
	debug_printf("===== Gait Estimate (encoder proxy) =====\r\n");
	debug_printf("  Estimator/state:   %u / %s; valid=%s motion=%s\r\n",
				 (unsigned)gait->estimator_id,
				 exo_gait_estimator_state_name(gait->state),
				 gait->valid ? "YES" : "no",
				 gait->motion_detected ? "YES" : "no");
	debug_printf("  Phase origin:      %s (not heel strike)\r\n",
				 exo_gait_phase_origin_name(gait->phase_origin));
	debug_printf(
		"  LEFT phase:       %s %.2f %% period=%.3f s event=0x%02X\r\n",
		left_valid ? "VALID" : "invalid", (double)(gait->left_phase_01 * 100.0f),
		(double)gait->left_period_s, (unsigned)gait->left_event_flags);
	debug_printf(
		"  RIGHT phase:      %s %.2f %% period=%.3f s event=0x%02X\r\n",
		right_valid ? "VALID" : "invalid",
		(double)(gait->right_phase_01 * 100.0f),
		(double)gait->right_period_s, (unsigned)gait->right_event_flags);
	debug_printf("  Cadence/confidence: %.2f spm / %.1f %%\r\n",
				 (double)gait->cadence_spm,
				 (double)(gait->confidence_01 * 100.0f));
	debug_printf("  Flags:             valid=0x%08lX quality=0x%08lX\r\n",
				 (unsigned long)gait->valid_flags,
				 (unsigned long)gait->quality_flags);
	debug_printf(
		"  Algorithm us:      snapshot=%lu/%lu gait=%lu/%lu assist=%lu/%lu "
		"(last/max)\r\n",
		(unsigned long)control.last_snapshot_build_us,
		(unsigned long)control.max_snapshot_build_us,
		(unsigned long)control.last_gait_update_us,
		(unsigned long)control.max_gait_update_us,
		(unsigned long)control.last_assist_update_us,
		(unsigned long)control.max_assist_update_us);
	debug_printf("  Algorithm errors:  %llu (last=%s)\r\n",
				 (unsigned long long)control.algorithm_error_count,
				 esp_err_to_name(control.last_algorithm_error));
}

static void cmd_assist_status(int argc, char *argv[]) {
	(void)argc;
	(void)argv;
	exo_assist_snapshot_t assist;
	if (!exo_assist_get_snapshot(&assist)) {
		debug_printf("ERROR: assist strategy is not initialized\r\n");
		return;
	}
	debug_printf("===== High-level Assistance =====\r\n");
	debug_printf("  Selected:          %s / %s strategy=%u profile=%u gen=%lu\r\n",
				 exo_assist_mode_name(assist.selected.mode),
				 exo_assist_execution_state_name(
					 assist.selected.requested_execution_state),
				 (unsigned)assist.selected.strategy_id,
				 (unsigned)assist.selected.profile_id,
				 (unsigned long)assist.selected_generation);
	debug_printf("  Active/locked:     %s / %s / %s\r\n",
				 assist.active_valid
					 ? exo_assist_mode_name(assist.active.mode)
					 : "NONE",
				 exo_assist_execution_state_name(assist.execution_state),
				 assist.config_locked ? "LOCKED" : "unlocked");
	debug_printf("  Strategy metadata: version=%u active_gen=%lu activation_epoch=%lu\r\n",
				 (unsigned)assist.strategy_version,
				 (unsigned long)assist.active_generation,
				 (unsigned long)assist.activation_epoch);
	debug_printf("  Levels L/R:        %u %% / %u %%\r\n",
				 (unsigned)assist.selected.left_level_percent,
				 (unsigned)assist.selected.right_level_percent);
	debug_printf(
		"  Sensors:           required=0x%08lX available=0x%08lX valid=0x%08lX\r\n",
		(unsigned long)assist.required_sensor_mask,
		(unsigned long)assist.available_sensor_mask,
		(unsigned long)assist.valid_sensor_mask);
	debug_printf(
		"  Gait requirement:  flags=0x%08lX min_confidence=%.1f %% ready=%s\r\n",
		(unsigned long)assist.required_gait_valid_flags,
		(double)(assist.minimum_gait_confidence_01 * 100.0f),
		assist.required_gait_ready ? "YES" : "no");
	debug_printf("  Ready/suggestion:  strategy=%s sensors=%s suggestion=%s\r\n",
				 assist.strategy_ready ? "YES" : "no",
				 assist.required_sensors_ready ? "YES" : "no",
				 assist.suggestion_valid ? "VALID" : "invalid");
	debug_printf("  Unavailable:       %s\r\n",
				 exo_assist_unavailable_reason_name(
					 assist.unavailable_reason));
	debug_printf(
		"  REAL OUTPUT:       %s (phase 9 SHADOW never changes CAN output)\r\n",
		assist.real_output_enabled ? "ENABLED" : "DISABLED");
	debug_printf("  Steps/suggestions: %llu / %llu\r\n",
				 (unsigned long long)assist.step_count,
				 (unsigned long long)assist.valid_suggestion_count);
	debug_printf(
		"  Last result:       valid=%s sensor_seq=%lu gait_seq=%lu activation=%lu\r\n",
		assist.last_result.valid ? "YES" : "no",
		(unsigned long)assist.last_result.source_sensor_sequence,
		(unsigned long)assist.last_result.source_gait_sequence,
		(unsigned long)assist.last_result.activation_epoch);
}

static bool parse_assist_mode_arg(const char *text, exo_assist_mode_t *mode) {
	if (text == NULL || mode == NULL) {
		return false;
	}
	if (strcasecmp(text, "transparent") == 0) {
		*mode = EXO_ASSIST_MODE_TRANSPARENT;
		return true;
	}
	if (strcasecmp(text, "active_phase") == 0) {
		*mode = EXO_ASSIST_MODE_ACTIVE_PHASE;
		return true;
	}
	if (strcasecmp(text, "aan") == 0 ||
		strcasecmp(text, "assist_as_needed") == 0) {
		*mode = EXO_ASSIST_MODE_ASSIST_AS_NEEDED;
		return true;
	}
	return false;
}

static bool parse_assist_execution_arg(
	const char *text, exo_assist_execution_state_t *state) {
	if (text == NULL || state == NULL) {
		return false;
	}
	if (strcasecmp(text, "disabled") == 0) {
		*state = EXO_ASSIST_EXEC_DISABLED;
		return true;
	}
	if (strcasecmp(text, "shadow") == 0) {
		*state = EXO_ASSIST_EXEC_SHADOW;
		return true;
	}
	return false;
}

static bool parse_assist_strategy_arg(const char *text,
					  exo_assist_strategy_id_t *strategy) {
	if (text == NULL || strategy == NULL) {
		return false;
	}
	if (strcasecmp(text, "transparent") == 0) {
		*strategy = EXO_ASSIST_STRATEGY_TRANSPARENT;
		return true;
	}
	if (strcasecmp(text, "phase_profile") == 0) {
		*strategy = EXO_ASSIST_STRATEGY_PHASE_PROFILE;
		return true;
	}
	if (strcasecmp(text, "aan_residual") == 0) {
		*strategy = EXO_ASSIST_STRATEGY_AAN_RESIDUAL;
		return true;
	}
	long numeric = 0;
	if (parse_long_arg(text, EXO_ASSIST_STRATEGY_TRANSPARENT,
				   EXO_ASSIST_STRATEGY_AAN_RESIDUAL, &numeric)) {
		*strategy = (exo_assist_strategy_id_t)numeric;
		return true;
	}
	return false;
}

static void cmd_assist_config(int argc, char *argv[]) {
	if (argc != 7) {
		debug_printf(
			"Usage: assist_config <transparent|active_phase|aan> "
			"<disabled|shadow> <transparent|phase_profile|aan_residual> "
			"<profile 0..255> <left 0..100> <right 0..100>\r\n");
		return;
	}
	exo_assist_mode_t mode;
	exo_assist_execution_state_t execution;
	exo_assist_strategy_id_t strategy;
	long profile = 0;
	long left_percent = 0;
	long right_percent = 0;
	if (!parse_assist_mode_arg(argv[1], &mode) ||
		!parse_assist_execution_arg(argv[2], &execution) ||
		!parse_assist_strategy_arg(argv[3], &strategy) ||
		!parse_long_arg(argv[4], 0, UINT8_MAX, &profile) ||
		!parse_long_arg(argv[5], 0, 100, &left_percent) ||
		!parse_long_arg(argv[6], 0, 100, &right_percent)) {
		debug_printf("ERROR: invalid assistance configuration\r\n");
		return;
	}
	const exo_assist_config_t config = {
		.schema_version = EXO_ASSIST_CONFIG_SCHEMA_VERSION,
		.mode = mode,
		.requested_execution_state = execution,
		.strategy_id = strategy,
		.profile_id = (uint8_t)profile,
		.left_level_percent = (uint8_t)left_percent,
		.right_level_percent = (uint8_t)right_percent,
		.reserved_flags = 0U,
	};
	const esp_err_t err = exo_assist_set_config(&config);
	if (err != ESP_OK) {
		debug_printf("ERROR: assist config rejected: %s\r\n",
					 esp_err_to_name(err));
		return;
	}
	debug_printf(
		"OK: assist selected=%s execution=%s strategy=%u profile=%u L/R=%u/%u; "
		"SHADOW never changes CAN output\r\n",
		exo_assist_mode_name(mode),
		exo_assist_execution_state_name(execution), (unsigned)strategy,
		(unsigned)profile, (unsigned)left_percent, (unsigned)right_percent);
}

static void cmd_can_status(int argc, char *argv[]) {
	/* 读取 ESP-IDF TWAI 控制器统计，可区分停机、正常、BUS_OFF 和恢复中。 */
	twai_status_info_t status;
	esp_err_t err = twai_get_status_info(&status);
	if (err != ESP_OK) {
		debug_printf("ERROR: cannot read TWAI status: %s\r\n",
					 esp_err_to_name(err));
		return;
	}

	const char *state = "UNKNOWN";
	switch (status.state) {
	case TWAI_STATE_STOPPED:
		state = "STOPPED";
		break;
	case TWAI_STATE_RUNNING:
		state = "RUNNING";
		break;
	case TWAI_STATE_BUS_OFF:
		state = "BUS_OFF";
		break;
	case TWAI_STATE_RECOVERING:
		state = "RECOVERING";
		break;
	default:
		break;
	}

	debug_printf("===== TWAI Status =====\r\n");
	debug_printf("  State:             %s\r\n", state);
	debug_printf(
		"  RX GPIO%d level:   %d (%s)\r\n", g_exo_hardware_config.twai.rx_gpio,
		gpio_get_level(g_exo_hardware_config.twai.rx_gpio),
		gpio_get_level(g_exo_hardware_config.twai.rx_gpio) ? "recessive/idle"
														   : "dominant/busy");
	debug_printf("  TX queued:         %lu\r\n",
				 (unsigned long)status.msgs_to_tx);
	debug_printf("  RX queued:         %lu\r\n",
				 (unsigned long)status.msgs_to_rx);
	debug_printf("  TX error counter:  %lu\r\n",
				 (unsigned long)status.tx_error_counter);
	debug_printf("  RX error counter:  %lu\r\n",
				 (unsigned long)status.rx_error_counter);
	debug_printf("  TX failed count:   %lu\r\n",
				 (unsigned long)status.tx_failed_count);
	debug_printf("  Bus error count:   %lu\r\n",
				 (unsigned long)status.bus_error_count);
	debug_printf("  Arbitration lost:  %lu\r\n",
				 (unsigned long)status.arb_lost_count);
	debug_printf("  RX missed:         %lu\r\n",
				 (unsigned long)status.rx_missed_count);
}

static void print_system_request_result(const char *request_name,
										exo_request_result_t result) {
	/*
	 * START/STOP 是异步请求：ACCEPTED 只表示后台执行器接单，不表示电机已经完成
	 * 启停。随后使用 system_status 查看状态和 active step。
	 */
	const char *result_name = exo_system_request_result_name(result);

	switch (result) {
	case EXO_REQUEST_ACCEPTED:
		debug_printf("OK: %s request ACCEPTED; transition is asynchronous "
					 "(use system_status)\r\n",
					 request_name);
		break;
	case EXO_REQUEST_ALREADY_SATISFIED: {
		exo_system_snapshot_t snapshot;
		const char *state_name = "UNKNOWN";
		if (exo_system_get_snapshot(&snapshot)) {
			state_name = exo_system_state_name(snapshot.state);
		}
		debug_printf("OK: %s request already satisfied; system is %s and "
					 "no transition was started (result=%s)\r\n",
					 request_name, state_name, result_name);
		break;
	}
	case EXO_REQUEST_ALREADY_PENDING:
		debug_printf("OK: %s request already pending; no duplicate "
					 "transition queued (result=%s)\r\n",
					 request_name, result_name);
		break;
	default:
		debug_printf("ERROR: %s request rejected (result=%s); "
					 "use system_status for details\r\n",
					 request_name, result_name);
		break;
	}
}

static void cmd_system_start(int argc, char *argv[]) {
	/* UART 与 BLE 共用同一个状态机请求函数，不存在两套互相打架的运行状态。 */
	print_system_request_result("START", exo_system_request_start());
}

static void cmd_system_stop(int argc, char *argv[]) {
	print_system_request_result("STOP", exo_system_request_stop());
}

static void cmd_ble_status(int argc, char *argv[]) {
	/*
	 * 显示连接、订阅、队列、重复请求和通知统计。连接成功不等于 App 已订阅响应；
	 * negotiated MTU 是本次连接可承载的 ATT 大小，也不是 CAN 帧长度。
	 */
	exo_ble_snapshot_t snapshot;
	if (!exo_ble_get_snapshot(&snapshot)) {
		debug_printf("ERROR: product BLE is not initialized\r\n");
		return;
	}

	debug_printf("===== Product BLE =====\r\n");
	debug_printf("  Initialized:       %s\r\n",
				 snapshot.initialized ? "YES" : "no");
	debug_printf("  Host synced:       %s\r\n",
				 snapshot.host_synced ? "YES" : "no");
	debug_printf("  Connected:         %s\r\n",
				 snapshot.connected ? "YES" : "no");
	debug_printf("  Response notify:   %s\r\n", snapshot.response_subscribed
													? "subscribed"
													: "not subscribed");
	debug_printf("  Connection handle: 0x%04X\r\n", snapshot.connection_handle);
	debug_printf("  Negotiated MTU:    %u\r\n", snapshot.negotiated_mtu);
	debug_printf("  Session generation:%lu\r\n",
				 (unsigned long)snapshot.session_generation);
	debug_printf("  Boot ID:           0x%08lX\r\n",
				 (unsigned long)snapshot.boot_id);
	debug_printf("  Worker alive:      %s\r\n",
				 snapshot.worker_task_alive ? "YES" : "no");
	debug_printf("  Worker identity:   %p\r\n",
				 (void *)snapshot.worker_task_identity);
	debug_printf("  Worker stack min:  %lu bytes free\r\n",
				 (unsigned long)snapshot.worker_stack_free_min_bytes);
	debug_printf("  RX/queue/process:  %lu/%lu/%lu\r\n",
				 (unsigned long)snapshot.rx_write_count,
				 (unsigned long)snapshot.queued_count,
				 (unsigned long)snapshot.processed_count);
	debug_printf("  Bad/duplicate/id:  %lu/%lu/%lu\r\n",
				 (unsigned long)snapshot.malformed_count,
				 (unsigned long)snapshot.duplicate_count,
				 (unsigned long)snapshot.request_id_conflict_count);
	debug_printf("  Full/stale:        %lu/%lu\r\n",
				 (unsigned long)snapshot.queue_full_count,
				 (unsigned long)snapshot.stale_drop_count);
	debug_printf("  Notify submit ok/fail: %lu/%lu\r\n",
				 (unsigned long)snapshot.notify_success_count,
				 (unsigned long)snapshot.notify_failure_count);
	debug_printf("  Notify TX failures:    %lu\r\n",
				 (unsigned long)snapshot.notify_tx_failure_count);
}

static void cmd_system_status(int argc, char *argv[]) {
	/*
	 * 系统 state 表示生命周期，active step 表示当前启停子步骤；output_allowed
	 * 是 控制任务最终输出门。三者用途不同，不要只看 run_requested
	 * 猜测电机状态。
	 */
	exo_system_snapshot_t snapshot;
	if (!exo_system_get_snapshot(&snapshot)) {
		debug_printf("ERROR: system state machine is not initialized\r\n");
		return;
	}

	debug_printf("===== Exoskeleton System =====\r\n");
	debug_printf("  Initialized:       %s\r\n",
				 snapshot.initialized ? "YES" : "no");
	debug_printf("  State:             %s\r\n",
				 exo_system_state_name(snapshot.state));
	debug_printf("  Active step:       %s\r\n",
				 exo_system_step_name(snapshot.active_step));
	debug_printf("  Ready to start:    %s\r\n",
				 snapshot.ready_to_start ? "YES" : "no");
	debug_printf(
		"  Start block:       %s\r\n",
		exo_system_start_block_reason_name(snapshot.start_block_reason));
	debug_printf("  Run requested:     %s\r\n",
				 snapshot.run_requested ? "YES" : "no");
	debug_printf("  Output allowed:    %s\r\n",
				 snapshot.output_allowed ? "YES" : "no");
	debug_printf("  Maintenance:       %s\r\n",
				 snapshot.maintenance_active ? "ACTIVE" : "inactive");
	debug_printf("  Executor alive:    %s\r\n",
				 snapshot.executor_task_alive ? "YES" : "no");
	debug_printf("  Executor identity: %p\r\n",
				 (void *)snapshot.executor_task_identity);
	debug_printf("  Transitions:       %lu\r\n",
				 (unsigned long)snapshot.transition_count);
	debug_printf("  START:             requests=%lu success=%lu failure=%lu "
				 "cancelled=%lu\r\n",
				 (unsigned long)snapshot.start_request_count,
				 (unsigned long)snapshot.start_success_count,
				 (unsigned long)snapshot.start_failure_count,
				 (unsigned long)snapshot.start_cancel_count);
	debug_printf(
		"  STOP:              requests=%lu success=%lu failure=%lu\r\n",
		(unsigned long)snapshot.stop_request_count,
		(unsigned long)snapshot.stop_success_count,
		(unsigned long)snapshot.stop_failure_count);
	debug_printf("  Rejected requests: %lu\r\n",
				 (unsigned long)snapshot.rejected_request_count);
	debug_printf("  Last result:       %s\r\n",
				 exo_system_request_result_name(snapshot.last_request_result));
	debug_printf("  Last error:        %s (%ld)\r\n",
				 esp_err_to_name(snapshot.last_error),
				 (long)snapshot.last_error);
	debug_printf("  Last failed step:  %s\r\n",
				 exo_system_step_name(snapshot.last_failed_step));
	debug_printf("  State entered:     %llu us\r\n",
				 (unsigned long long)snapshot.state_entered_us);
	debug_printf("  Last request at:   %llu us\r\n",
				 (unsigned long long)snapshot.last_request_us);
	debug_printf("  Last failure:      %llu us\r\n",
				 (unsigned long long)snapshot.last_failure_us);
	debug_printf("  Last completed:    %llu us\r\n",
				 (unsigned long long)snapshot.last_completed_us);
}

static void cmd_help(int argc, char *argv[]) {
	/* 直接遍历注册表生成帮助，避免帮助列表和实际可执行命令逐渐不一致。 */
	debug_printf("\r\nRobStride Motor Control Console\r\n");
	exo_can_node_id_t can_id = selected_can_id();
	debug_printf("Current motor: 0x%02X (%d), actuator %c\r\n", can_id, can_id,
				 s_selected_actuator == EXO_ACTUATOR_A ? 'A' : 'B');
	debug_printf("================================\r\n");
	for (const console_cmd_t *c = s_cmd_table; c->name != NULL; c++) {
		debug_printf("  %-14s %s\r\n", c->name, c->help);
	}
	debug_printf("\r\n");
}

/* ================================================================
 * 命令解析与分派
 *
 * 例：输入 `joint_zero left` 后得到 argc=2，argv[0]="joint_zero"，
 * argv[1]="left"。解析器只按空格/Tab 分词，不支持引号、转义或任意长载荷。
 * ================================================================ */
/* 最多 8 个 token，其中第一个是命令名，所以最多允许 7 个命令参数。 */
#define CMD_MAX_ARGS 8

static void console_process_line(char *line) {
	if (line == NULL || s_cmd_mutex == NULL) {
		return;
	}

	/* 从修剪、分词到命令结束全程串行，防止两个维护命令交错发送。 */
	xSemaphoreTake(s_cmd_mutex, portMAX_DELAY);

	/* 串口终端可能发送 LF、CR 或 CRLF，这里统一去掉行尾字符。 */
	size_t len = strlen(line);
	while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r')) {
		line[--len] = '\0';
	}
	if (len == 0) {
		xSemaphoreGive(s_cmd_mutex);
		return;
	}

	/* strtok_r 会原地把分隔符改成 '\0'，argv 指针都指向同一行缓冲区内部。 */
	int argc = 0;
	char *argv[CMD_MAX_ARGS];
	char *saveptr = NULL;
	char *token = strtok_r(line, " \t", &saveptr);
	while (token != NULL && argc < CMD_MAX_ARGS) {
		argv[argc++] = token;
		token = strtok_r(NULL, " \t", &saveptr);
	}
	if (argc == 0) {
		xSemaphoreGive(s_cmd_mutex);
		return;
	}
	if (token != NULL) {
		debug_printf("ERROR: too many command arguments (max %d)\r\n",
					 CMD_MAX_ARGS - 1);
		xSemaphoreGive(s_cmd_mutex);
		return;
	}

	/* 线性扫描注册表；命令数量很小，可读性比引入哈希表更重要。 */
	for (const console_cmd_t *c = s_cmd_table; c->name != NULL; c++) {
		if (strcmp(argv[0], c->name) == 0) {
			if (argc - 1 < c->min_args) {
				debug_printf("Usage: %s (needs %d args, got %d)\r\n", c->help,
							 c->min_args, argc - 1);
				xSemaphoreGive(s_cmd_mutex);
				return;
			}

			/*
			 * 直接电机命令先向系统状态机申请短暂维护权。只有 IDLE
			 * 才能取得，确保 它不会与 RUNNING 的 100 Hz 输出或 START/STOP
			 * 执行步骤交错。
			 */
			bool maintenance_acquired = false;
			if (c->access == CONSOLE_CMD_MAINTENANCE) {
				maintenance_acquired = exo_system_try_begin_maintenance();
				if (!maintenance_acquired) {
					debug_printf("ERROR: maintenance command rejected; "
								 "system must be IDLE (use system_status)\r\n");
					xSemaphoreGive(s_cmd_mutex);
					return;
				}
			}

			/* 无论处理函数内部成功还是打印错误，已取得的维护权都必须在这里归还。
			 */
			c->handler(argc, argv);
			if (maintenance_acquired) {
				exo_system_end_maintenance();
			}
			xSemaphoreGive(s_cmd_mutex);
			return;
		}
	}
	debug_printf("Unknown command: '%s'. Type 'help' for list.\r\n", argv[0]);
	xSemaphoreGive(s_cmd_mutex);
}

static void uart_console_task(void *arg) {
	/*
	 * 任务逐字符读取标准输入并拼成固定长度行，不使用动态内存。遇到超长行后丢弃
	 * 直到下一个回车，防止“后半截”被误当成另一条危险命令。
	 */
	char line[LINE_BUF_SIZE];
	size_t pos = 0;
	bool line_overflow = false;

	debug_printf("\r\nRobStride Motor Control - Programming Port Console\r\n");
	debug_printf("Type 'help' and press Enter for the command list.\r\n");

	while (1) {
		/* stdin 暂无字符时可能返回 EOF；清除标志并休眠 10 ms，避免空转。 */
		int input = fgetc(stdin);
		if (input == EOF) {
			clearerr(stdin);
			vTaskDelay(pdMS_TO_TICKS(10));
			continue;
		}

		uint8_t ch = (uint8_t)input;
		if (ch == '\n' || ch == '\r') {
			/* CRLF 会触发两次分支，但第二次 pos=0，因此不会重复执行命令。 */
			if (line_overflow) {
				debug_printf("ERROR: command line exceeds %d bytes\r\n",
							 LINE_BUF_SIZE - 1);
				line_overflow = false;
				pos = 0;
			} else if (pos > 0) {
				line[pos] = '\0';
				console_process_line(line);
				pos = 0;
			}
		} else if (pos < sizeof(line) - 1) {
			line[pos++] = (char)ch;
		} else {
			line_overflow = true;
		}
	}
}

esp_err_t debug_console_init(void) {
	/* 每次上电只允许创建一个 UART 读取任务，重复调用不是“重新打开”。 */
	if (s_uart_task != NULL) {
		return ESP_ERR_INVALID_STATE;
	}

	/* 默认选择仅服务裸电机命令；左右髋映射仍由 hip_joint 模块读取。 */
	s_selected_actuator = g_exo_hardware_config.default_actuator;

	/* 先创建互斥量，再创建任务；任务创建失败时把已分配资源完整释放。 */
	s_cmd_mutex = xSemaphoreCreateMutex();
	if (s_cmd_mutex == NULL) {
		ESP_LOGE(TAG, "Failed to create command mutex");
		return ESP_ERR_NO_MEM;
	}

	/* 栈大小参数在 ESP-IDF 中以字节计，任务固定到配置的 ESP32 内核。 */
	if (xTaskCreatePinnedToCore(uart_console_task, "debug_console",
								EXO_DEBUG_CONSOLE_TASK_STACK_BYTES, NULL,
								EXO_DEBUG_CONSOLE_TASK_PRIORITY, &s_uart_task,
								EXO_DEBUG_CONSOLE_TASK_CORE) != pdPASS) {
		ESP_LOGE(TAG, "Failed to create programming UART console task");
		vSemaphoreDelete(s_cmd_mutex);
		s_cmd_mutex = NULL;
		return ESP_ERR_NO_MEM;
	}

	ESP_LOGI(TAG, "Programming UART debug console initialized");
	return ESP_OK;
}
