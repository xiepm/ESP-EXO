#include "debug_console.h"

#include "driver/gpio.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "exo_ble.h"
#include "exo_config.h"
#include "exo_control_loop.h"
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

#define LINE_BUF_SIZE 256
#define EXO_DEBUG_CONSOLE_TASK_CORE 0
#define EXO_DEBUG_CONSOLE_TASK_PRIORITY 5
#define EXO_DEBUG_CONSOLE_TASK_STACK_BYTES 4096

static SemaphoreHandle_t s_cmd_mutex;
static TaskHandle_t s_uart_task;
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

/* ---- Command handler type ---- */
typedef void (*cmd_handler_t)(int argc, char *argv[]);

typedef enum {
	CONSOLE_CMD_ALWAYS_AVAILABLE = 0,
	CONSOLE_CMD_MAINTENANCE,
	CONSOLE_CMD_SYSTEM_REQUEST,
} console_cmd_access_t;

typedef struct {
	const char *name;
	const char *help;
	int min_args;
	console_cmd_access_t access;
	cmd_handler_t handler;
} console_cmd_t;

/* ---- Command handler forward declarations ---- */
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
static void cmd_control_status(int argc, char *argv[]);
static void cmd_can_status(int argc, char *argv[]);
static void cmd_ble_status(int argc, char *argv[]);
static void cmd_system_start(int argc, char *argv[]);
static void cmd_system_stop(int argc, char *argv[]);
static void cmd_system_status(int argc, char *argv[]);
static void cmd_help(int argc, char *argv[]);

/* ---- Command table ---- */
static const console_cmd_t s_cmd_table[] = {
	{"motor", "Select target: <A|B|configured_can_id>", 1,
	 CONSOLE_CMD_ALWAYS_AVAILABLE, cmd_motor},
	{"enable", "Motor enable (private protocol)", 0, CONSOLE_CMD_MAINTENANCE,
	 cmd_enable},
	{"disable", "Motor disable/reset (private protocol)", 0,
	 CONSOLE_CMD_MAINTENANCE, cmd_disable},
	{"move", "Motion control: <t> <pos> <spd> <kp> <kd>", 5,
	 CONSOLE_CMD_MAINTENANCE, cmd_move},
	{"speed", "Speed control: <cur_limit> <acc> <speed>", 3,
	 CONSOLE_CMD_MAINTENANCE, cmd_speed},
	{"pos_pp", "PP position: <speed> <acc> <angle>", 3, CONSOLE_CMD_MAINTENANCE,
	 cmd_pos_pp},
	{"pos_csp", "CSP position: <speed> <angle>", 2, CONSOLE_CMD_MAINTENANCE,
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
	{"mit_control", "MIT control: <ang> <spd> <kp> <kd> <t>", 5,
	 CONSOLE_CMD_MAINTENANCE, cmd_mit_control},
	{"mit_pos", "MIT position: <pos> <spd>", 2, CONSOLE_CMD_MAINTENANCE,
	 cmd_mit_pos},
	{"mit_speed", "MIT speed: <spd> <cur>", 2, CONSOLE_CMD_MAINTENANCE,
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
	{"control_status", "Show the permanent 100 Hz control loop", 0,
	 CONSOLE_CMD_ALWAYS_AVAILABLE, cmd_control_status},
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
	va_list args;
	va_start(args, fmt);
	(void)vprintf(fmt, args);
	va_end(args);
	fflush(stdout);
}

static bool parse_long_arg(const char *text, long min_value, long max_value,
						   long *value) {
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
	for (int i = 0; i < count; ++i) {
		if (!parse_float_arg(argv[first + i], &values[i])) {
			return false;
		}
	}
	return true;
}

static exo_can_node_id_t selected_can_id(void) {
	exo_can_node_id_t can_id = 0;
	(void)rs01_get_can_id(s_selected_actuator, &can_id);
	return can_id;
}

static bool driver_command_succeeded(esp_err_t err) {
	if (err == ESP_OK) {
		return true;
	}

	debug_printf("ERROR: RS01 command failed: %s\r\n", esp_err_to_name(err));
	return false;
}

/* ================================================================
 *  Command handlers (identical to console.c, printf -> debug_printf)
 * ================================================================ */

static void cmd_motor(int argc, char *argv[]) {
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
	if (driver_command_succeeded(rs01_enable(s_selected_actuator))) {
		debug_printf("OK: motor 0x%02X enable sent\r\n", selected_can_id());
	}
}

static void cmd_disable(int argc, char *argv[]) {
	if (driver_command_succeeded(rs01_disable(s_selected_actuator))) {
		debug_printf("OK: motor 0x%02X disable sent\r\n", selected_can_id());
	}
}

static void cmd_move(int argc, char *argv[]) {
	float values[5];
	if (!parse_float_args(argv, 1, 5, values)) {
		debug_printf("ERROR: move arguments must be finite numbers\r\n");
		return;
	}
	float t = values[0], pos = values[1], spd = values[2];
	float kp = values[3], kd = values[4];
	if (driver_command_succeeded(
			rs01_move(s_selected_actuator, t, pos, spd, kp, kd))) {
		debug_printf("OK: motor 0x%02X move t=%.2f pos=%.2f spd=%.2f kp=%.2f "
					 "kd=%.2f\r\n",
					 selected_can_id(), (double)t, (double)pos, (double)spd,
					 (double)kp, (double)kd);
	}
}

static void cmd_speed(int argc, char *argv[]) {
	float values[3];
	if (!parse_float_args(argv, 1, 3, values)) {
		debug_printf("ERROR: speed arguments must be finite numbers\r\n");
		return;
	}
	float cur = values[0], acc = values[1], spd = values[2];
	if (driver_command_succeeded(
			rs01_speed_control(s_selected_actuator, cur, acc, spd))) {
		debug_printf("OK: motor 0x%02X speed cur=%.2f acc=%.2f spd=%.2f\r\n",
					 selected_can_id(), (double)cur, (double)acc, (double)spd);
	}
}

static void cmd_pos_pp(int argc, char *argv[]) {
	float values[3];
	if (!parse_float_args(argv, 1, 3, values)) {
		debug_printf("ERROR: pos_pp arguments must be finite numbers\r\n");
		return;
	}
	float spd = values[0], acc = values[1], ang = values[2];
	if (driver_command_succeeded(
			rs01_position_pp_control(s_selected_actuator, spd, acc, ang))) {
		debug_printf("OK: motor 0x%02X PP pos spd=%.2f acc=%.2f ang=%.2f\r\n",
					 selected_can_id(), (double)spd, (double)acc, (double)ang);
	}
}

static void cmd_pos_csp(int argc, char *argv[]) {
	float values[2];
	if (!parse_float_args(argv, 1, 2, values)) {
		debug_printf("ERROR: pos_csp arguments must be finite numbers\r\n");
		return;
	}
	float spd = values[0], ang = values[1];
	if (driver_command_succeeded(
			rs01_position_csp_control(s_selected_actuator, spd, ang))) {
		debug_printf("OK: motor 0x%02X CSP pos spd=%.2f ang=%.2f\r\n",
					 selected_can_id(), (double)spd, (double)ang);
	}
}

static void cmd_current(int argc, char *argv[]) {
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
	if (driver_command_succeeded(
			rs01_set_mechanical_zero(s_selected_actuator))) {
		debug_printf("OK: motor 0x%02X set zero position sent\r\n",
					 selected_can_id());
	}
}

static void cmd_set_id(int argc, char *argv[]) {
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
	if (driver_command_succeeded(rs01_save_parameters(s_selected_actuator))) {
		debug_printf("OK: motor 0x%02X save to flash sent\r\n",
					 selected_can_id());
	}
}

static void cmd_set_baud(int argc, char *argv[]) {
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
	if (driver_command_succeeded(
			rs01_set_protocol(s_selected_actuator, RS01_PROTOCOL_MIT))) {
		debug_printf(
			"OK: motor 0x%02X MIT protocol selected; power-cycle motor "
			"to apply\r\n",
			selected_can_id());
	}
}

static void cmd_switch_private(int argc, char *argv[]) {
	if (driver_command_succeeded(rs01_mit_set_protocol(
			s_selected_actuator, RS01_PROTOCOL_PRIVATE))) {
		debug_printf("OK: motor 0x%02X private protocol selected; power-cycle "
					 "motor to apply\r\n",
					 selected_can_id());
	}
}

static void cmd_read_param(int argc, char *argv[]) {
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
	float values[5];
	if (!parse_float_args(argv, 1, 5, values)) {
		debug_printf("ERROR: mit_control arguments must be finite numbers\r\n");
		return;
	}
	float ang = values[0], spd = values[1], kp = values[2];
	float kd = values[3], t = values[4];
	if (driver_command_succeeded(
			rs01_mit_send_motion(s_selected_actuator, ang, spd, kp, kd, t))) {
		debug_printf("OK: motor 0x%02X MIT control ang=%.2f spd=%.2f kp=%.2f "
					 "kd=%.2f t=%.2f\r\n",
					 selected_can_id(), (double)ang, (double)spd, (double)kp,
					 (double)kd, (double)t);
	}
}

static void cmd_mit_pos(int argc, char *argv[]) {
	float values[2];
	if (!parse_float_args(argv, 1, 2, values)) {
		debug_printf("ERROR: mit_pos arguments must be finite numbers\r\n");
		return;
	}
	float pos = values[0], spd = values[1];
	if (driver_command_succeeded(
			rs01_mit_position_control(s_selected_actuator, pos, spd))) {
		debug_printf("OK: motor 0x%02X MIT position pos=%.2f spd=%.2f\r\n",
					 selected_can_id(), (double)pos, (double)spd);
	}
}

static void cmd_mit_speed(int argc, char *argv[]) {
	float values[2];
	if (!parse_float_args(argv, 1, 2, values)) {
		debug_printf("ERROR: mit_speed arguments must be finite numbers\r\n");
		return;
	}
	float spd = values[0], cur = values[1];
	if (driver_command_succeeded(
			rs01_mit_speed_control(s_selected_actuator, spd, cur))) {
		debug_printf("OK: motor 0x%02X MIT speed spd=%.2f cur=%.2f\r\n",
					 selected_can_id(), (double)spd, (double)cur);
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
	const rs01_limits_t *limits = rs01_get_limits();
	debug_printf("Motor driver: RS01\r\n");
	debug_printf("  P:  [%.2f, %.2f] rad\r\n", (double)limits->position_min_rad,
				 (double)limits->position_max_rad);
	debug_printf("  V:  [%.2f, %.2f] rad/s\r\n",
				 (double)limits->velocity_min_rad_s,
				 (double)limits->velocity_max_rad_s);
	debug_printf("  KP: [%.2f, %.2f]\r\n", (double)limits->kp_min,
				 (double)limits->kp_max);
	debug_printf("  KD: [%.2f, %.2f]\r\n", (double)limits->kd_min,
				 (double)limits->kd_max);
	debug_printf("  T:  [%.2f, %.2f] Nm\r\n", (double)limits->torque_min_nm,
				 (double)limits->torque_max_nm);
}

static void cmd_status(int argc, char *argv[]) {
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
	debug_printf("  Angle:       %.3f rad\r\n",
				 (double)snapshot.sample.position_rad);
	debug_printf("  Speed:       %.3f rad/s\r\n",
				 (double)snapshot.sample.velocity_rad_s);
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
	return joint == EXO_HIP_JOINT_LEFT ? "LEFT" : "RIGHT";
}

static const char *joint_direction_name(exo_direction_t direction) {
	if (direction == EXO_DIRECTION_POSITIVE) {
		return "+1";
	}
	if (direction == EXO_DIRECTION_NEGATIVE) {
		return "-1";
	}
	return "UNKNOWN";
}

static void print_joint_status(exo_hip_joint_id_t joint) {
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
	const bool joint_sample_available =
		raw_available && snapshot.direction_valid && snapshot.zero_valid;

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
		debug_printf("  Software zero: %.3f rad (valid, RAM only)\r\n",
					 (double)snapshot.software_zero_motor_rad);
		debug_printf(
			"  Zero source:   feedback count=%lu timestamp=%llu us\r\n",
			(unsigned long)snapshot.zero_source_feedback_count,
			(unsigned long long)snapshot.zero_source_timestamp_us);
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
		debug_printf("  Raw actuator:  q=%.3f rad dq=%.3f rad/s tau=%.3f Nm "
					 "temp=%.1f C\r\n",
					 (double)snapshot.raw_actuator_sample.position_rad,
					 (double)snapshot.raw_actuator_sample.velocity_rad_s,
					 (double)snapshot.raw_actuator_sample.torque_nm,
					 (double)snapshot.raw_actuator_sample.temperature_c);
	} else {
		debug_printf("  Raw actuator:  N/A\r\n");
	}
	if (joint_sample_available) {
		debug_printf(
			"  Joint:         q=%.3f rad dq=%.3f rad/s tau=%.3f Nm\r\n",
			(double)snapshot.joint_sample.position_rad,
			(double)snapshot.joint_sample.velocity_rad_s,
			(double)snapshot.joint_sample.torque_nm);
	} else {
		debug_printf("  Joint:         N/A\r\n");
	}
	debug_printf("  Joint valid:   %s\r\n",
				 snapshot.joint_valid ? "YES" : "no");
}

static bool command_joint_from_arg(char *argv[], exo_hip_joint_id_t *joint) {
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

static void cmd_control_status(int argc, char *argv[]) {
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
}

static void cmd_can_status(int argc, char *argv[]) {
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
	print_system_request_result("START", exo_system_request_start());
}

static void cmd_system_stop(int argc, char *argv[]) {
	print_system_request_result("STOP", exo_system_request_stop());
}

static void cmd_ble_status(int argc, char *argv[]) {
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
 *  Command parser (same logic as console.c)
 * ================================================================ */
#define CMD_MAX_ARGS 8

static void console_process_line(char *line) {
	if (line == NULL || s_cmd_mutex == NULL) {
		return;
	}

	xSemaphoreTake(s_cmd_mutex, portMAX_DELAY);

	size_t len = strlen(line);
	while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r')) {
		line[--len] = '\0';
	}
	if (len == 0) {
		xSemaphoreGive(s_cmd_mutex);
		return;
	}

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

	for (const console_cmd_t *c = s_cmd_table; c->name != NULL; c++) {
		if (strcmp(argv[0], c->name) == 0) {
			if (argc - 1 < c->min_args) {
				debug_printf("Usage: %s (needs %d args, got %d)\r\n", c->help,
							 c->min_args, argc - 1);
				xSemaphoreGive(s_cmd_mutex);
				return;
			}

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
	char line[LINE_BUF_SIZE];
	size_t pos = 0;
	bool line_overflow = false;

	debug_printf("\r\nRobStride Motor Control - Programming Port Console\r\n");
	debug_printf("Type 'help' and press Enter for the command list.\r\n");

	while (1) {
		int input = fgetc(stdin);
		if (input == EOF) {
			clearerr(stdin);
			vTaskDelay(pdMS_TO_TICKS(10));
			continue;
		}

		uint8_t ch = (uint8_t)input;
		if (ch == '\n' || ch == '\r') {
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
	if (s_uart_task != NULL) {
		return ESP_ERR_INVALID_STATE;
	}

	s_selected_actuator = g_exo_hardware_config.default_actuator;

	s_cmd_mutex = xSemaphoreCreateMutex();
	if (s_cmd_mutex == NULL) {
		ESP_LOGE(TAG, "Failed to create command mutex");
		return ESP_ERR_NO_MEM;
	}

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
