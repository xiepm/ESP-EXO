/*
 * SPDX-FileCopyrightText: 2010-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * ESP-EXO application entry point.
 *
 * 【给初学者的阅读提示】
 * 1. ESP-IDF 启动完 FreeRTOS 后会自动调用 app_main()，它相当于普通 C 程序的
 *    main()，不需要也不能由业务代码手动调用。
 * 2. 本文件只负责“把各模块按正确顺序启动起来”和“持续接收 CAN 帧”，真正的
 *    100 Hz 运动计算在 exo_control_loop.c，START/STOP 流程在 exo_system.c。
 * 3. ESP32 外设名称叫 TWAI，本质上就是经典 CAN 控制器；板外仍需 CAN 收发器，
 *    TX/RX GPIO 不能直接接到电机的 CANH/CANL。
 *
 * 初始化顺序固定为：NVS -> RS01对象 -> 髋关节/出厂方向 -> TWAI/RX -> 系统 INIT
 * -> UART/BLE 命令入口 -> 唯一 100 Hz 控制任务 -> 发布 IDLE。app_main 本身不
 * 承担周期控制，也不会根据 CAN ID 猜测人体左右侧。发布IDLE后，现有执行器负责
 * 有界请求主动反馈；电机后上电时不再只依赖启动瞬间的一次普通CAN请求。
 */

#include "debug_console.h"
#include "driver/twai.h"
#include "esp_err.h"
#include "esp_log.h"
#include "exo_assist.h"
#include "exo_ble.h"
#include "exo_config.h"
#include "exo_control_loop.h"
#include "exo_controller.h"
#include "exo_factory_direction.h"
#include "exo_wear_zero.h"
#include "exo_system.h"
#include "exo_telemetry.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hip_joint.h"
#include "rs01_driver.h"

static const char *TAG = "main";

/*
 * ESP32 双核任务分工：CAN 接收任务固定在 CPU1。任务所在核只是调度配置，不代表
 * 项目使用了桌面 CPU；这里的 CPU1 仍然是 ESP32 芯片内部的第二个 Xtensa 核。
 */
#define EXO_TWAI_RX_TASK_CORE 1

/*
 * TWAI RX 任务只做三件事：等待一帧 -> 交给 RS01 驱动解码 -> 必要时打印异常。
 * portMAX_DELAY 表示没有 CAN 帧时让任务休眠等待，不会空转占满 CPU。
 * 本任务绝不生成运动目标，因此整个项目只有控制任务能够周期性发送运动命令。
 */
static void twai_rx_task(void *arg) {
	/* rx_msg 是 ESP-IDF 对一帧 CAN 报文的描述：ID、长度、标志位和最多 8
	 * 字节数据。 */
	twai_message_t rx_msg;
	/* 两个时间戳用于“限速打印”，避免坏帧很多时串口日志拖慢系统。 */
	TickType_t last_invalid_log = 0;
	TickType_t last_unhandled_log = 0;
	TickType_t last_rx_error_log = 0;
	bool rx_error_logged = false;
	ESP_LOGI(TAG, "TWAI RX task started");

	while (1) {
		esp_err_t err = twai_receive(&rx_msg, portMAX_DELAY);
		if (err != ESP_OK) {
			/* 驱动异常若立即返回，必须让出CPU，不能高优先级空转刷串口。 */
			const TickType_t now = xTaskGetTickCount();
			if (!rx_error_logged || now - last_rx_error_log >= pdMS_TO_TICKS(1000)) {
				ESP_LOGW(TAG, "RX error: %s", esp_err_to_name(err));
				last_rx_error_log = now;
				rx_error_logged = true;
			}
			vTaskDelay(pdMS_TO_TICKS(10) > 0U ? pdMS_TO_TICKS(10) : 1U);
			continue;
		}

		/*
		 * 先放入 COUNT 哨兵值，驱动只有在识别出某个已配置电机后才会改成 A 或
		 * B。 event 告诉上层这帧是正常反馈、故障、坏帧，还是与 RS01 无关的 CAN
		 * 帧。
		 */
		exo_actuator_id_t actuator = EXO_ACTUATOR_COUNT;
		rs01_rx_event_t event = rs01_process_can_frame(&rx_msg, &actuator);

		if (event == RS01_RX_EVENT_FEEDBACK && actuator < EXO_ACTUATOR_COUNT) {
			/*
			 * 正常反馈在 rs01_process_can_frame()
			 * 内已经写入线程安全快照，这里无需
			 * 再复制或打印。保持快速返回循环，才能及时排空 100 Hz
			 * 主动上报队列； 需要观察数据时再由 UART/BLE 按需读取快照。
			 */
		} else if (event == RS01_RX_EVENT_FAULT &&
				   actuator < EXO_ACTUATOR_COUNT) {
			rs01_feedback_snapshot_t feedback;
			exo_can_node_id_t can_id = 0;
			if (rs01_get_feedback(actuator, &feedback) &&
				rs01_get_can_id(actuator, &can_id)) {
				ESP_LOGW(TAG, "RS01 fault: id=%u fault=0x%08lX warning=0x%08lX",
						 can_id, (unsigned long)feedback.fault_raw,
						 (unsigned long)feedback.warning_raw);
			}
		} else if (event == RS01_RX_EVENT_INVALID) {
			/* 帧格式不符合驱动约定：每秒最多提示一次，但内部统计仍逐帧累计。 */
			TickType_t now = xTaskGetTickCount();
			if ((now - last_invalid_log) >= pdMS_TO_TICKS(1000)) {
				last_invalid_log = now;
				ESP_LOGW(TAG,
						 "Invalid RS01 frame: id=0x%08lX dlc=%u flags=0x%lx",
						 (unsigned long)rx_msg.identifier,
						 rx_msg.data_length_code, (unsigned long)rx_msg.flags);
			}
		} else if (event == RS01_RX_EVENT_NONE) {
			/* 合法 CAN 帧但不是本项目当前处理的 RS01 消息，同样限制日志频率。
			 */
			TickType_t now = xTaskGetTickCount();
			if ((now - last_unhandled_log) >= pdMS_TO_TICKS(1000)) {
				last_unhandled_log = now;
				ESP_LOGI(TAG,
						 "Unhandled CAN frame: id=0x%08lX dlc=%u flags=0x%lx",
						 (unsigned long)rx_msg.identifier,
						 rx_msg.data_length_code, (unsigned long)rx_msg.flags);
			}
		}
	}
}

/* ================================================================
 * TWAI（CAN 控制器）初始化
 *
 * general：GPIO、工作模式和队列；timing：总线位率；filter：接收过滤规则。
 * 当前先接收全部报文，再由 RS01 驱动按协议和节点号判断归属，便于调试双电机。
 * ================================================================ */
static void twai_init(void) {
	twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
		g_exo_hardware_config.twai.tx_gpio, g_exo_hardware_config.twai.rx_gpio,
		TWAI_MODE_NORMAL);
	/*
	 * 某些复合命令会连续产生多帧应答，同时两个电机还可能各自以 100 Hz
	 * 主动上报。 ESP-IDF 默认仅 5
	 * 帧的接收队列容易在这个瞬间溢出，所以使用产品配置中的
	 * 加长队列。队列只缓冲报文，不会改变 CAN 总线位率。
	 */
	g_config.rx_queue_len = g_exo_hardware_config.twai.rx_queue_length;
	twai_timing_config_t t_config = g_exo_hardware_config.twai.timing;
	twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

	/* ESP_ERROR_CHECK
	 * 表示初始化失败就立即停止启动，并打印明确错误，而非带病运行。 */
	ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
	ESP_LOGI(
		TAG, "TWAI driver installed (%luMbps)",
		(unsigned long)(g_exo_hardware_config.twai.bitrate_bps / 1000000U));

	ESP_ERROR_CHECK(twai_start());
	ESP_LOGI(TAG, "TWAI driver started");
}

/*
 * 应用入口只负责按依赖顺序创建对象和任务，初始化结束后立即返回。
 * 各步骤的前后顺序有依赖关系，例如髋关节对象要读取 RS01 快照，控制任务要读取
 * 已初始化的状态机；调整顺序前应先确认依赖，不要仅为了“看起来整齐”而移动。
 */
void app_main(void) {
	ESP_LOGI(TAG, "RobStride Motor Control - TWAI Master");
	ESP_LOGI(
		TAG, "TX: GPIO%d, RX: GPIO%d, Baud: %luMbps",
		g_exo_hardware_config.twai.tx_gpio, g_exo_hardware_config.twai.rx_gpio,
		(unsigned long)(g_exo_hardware_config.twai.bitrate_bps / 1000000U));

	/* NVS先于关节/BLE初始化。出厂方向和候选零位断电保存，本次零位仍只在RAM。
	 * NVS异常不自动擦除；初始化失败时CAN尚未启动，不能带着未知方向进入运行。 */
	ESP_ERROR_CHECK(exo_factory_direction_storage_init());
	/* 先创建纯软件对象，此时还没有启动 CAN 外设，也不会让电机运动。 */
	ESP_ERROR_CHECK(rs01_driver_init());
	ESP_ERROR_CHECK(hip_joint_init());
	ESP_ERROR_CHECK(exo_wear_zero_init()); /* 只读候选，不自动恢复本次RAM零位。 */
	/*
	 * 通用控制器只初始化静态默认配置，不创建任务、不发送 CAN。产品默认是
	 * DAMPING + CONSTANT；工程交替激励必须由上位机显式选择，并受编译开关隔离。
	 */
	ESP_ERROR_CHECK(exo_controller_init());
	/*
	 * 助力对象建立版本化 selected 配置和固定策略工作区，不创建第二个周期任务。
	 * 阶段10真实助力仍只能经唯一控制器合成/发送出口；周期调用继续由后面的
	 * 唯一100 Hz控制任务完成。
	 */
	ESP_ERROR_CHECK(exo_assist_init());
	/*
	 * 遥测对象只有一个固定大小环形缓冲区；初始化不会创建任务、发送 BLE 或
	 * 启动数据流。只有上位机订阅并显式请求 START_TELEMETRY 后，100 Hz 控制
	 * 任务才会按配置分频写入样本。
	 */
	ESP_ERROR_CHECK(exo_telemetry_init() ? ESP_OK : ESP_FAIL);

	/* 启动 CAN 控制器后才能向 RS01 发配置命令或接收反馈。 */
	twai_init();

	/* 高于普通后台任务的优先级 8，目的是尽快取走硬件接收队列中的报文。 */
	BaseType_t rx_task_result = xTaskCreatePinnedToCore(
		twai_rx_task, "twai_rx", 4096, NULL, 8, NULL, EXO_TWAI_RX_TASK_CORE);
	ESP_ERROR_CHECK(rx_task_result == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);

	/* 主动反馈由现有执行器在IDLE中有界恢复，避免电机尚未上电时普通CAN
	 * 反复硬件重传。真正收到新的反馈才在线，不把请求成功当作验收。 */

	/*
	 * 状态机执行器启动在 INIT 状态，平时阻塞等待 START/STOP
	 * 请求。它负责一次性的 启停步骤，不负责 100 Hz
	 * 定时，因此不会形成第二条控制时间基准。
	 */
	ESP_ERROR_CHECK(exo_system_init());

	/*
	 * UART 是工程调试文本命令入口；BLE 是手机产品协议入口。二者最终都调用相同的
	 * 状态机请求接口，不能各自直接维护一套“运行状态”。NimBLE 主机只初始化一次。
	 */
	ESP_ERROR_CHECK(debug_console_init());
	ESP_ERROR_CHECK(exo_ble_init());

	/*
	 * 最后创建全项目唯一的 100 Hz 控制任务，再通知状态机初始化完毕并进入 IDLE。
	 * app_main 返回后，FreeRTOS 会继续调度上面创建的任务；返回不是“程序结束”。
	 */
	ESP_ERROR_CHECK(exo_control_loop_init());
	ESP_ERROR_CHECK(exo_system_finish_initialization());
}
