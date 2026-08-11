#ifndef DEBUG_CONSOLE_H
#define DEBUG_CONSOLE_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 启动仅供工程调试使用的 USB/编程 UART 文本控制台。
 *
 * 手机 App 不调用这个接口，也不发送这里的文本命令；手机走 exo_ble.h 定义的
 * 二进制 BLE 产品协议。控制台初始化成功后会创建一个 FreeRTOS 后台任务，从
 * stdin 逐行读取命令。函数只负责创建资源，不会在这里永久阻塞。
 *
 * 返回 ESP_OK 表示任务创建成功；重复初始化返回 ESP_ERR_INVALID_STATE，内存不足
 * 返回 ESP_ERR_NO_MEM。
 */
esp_err_t debug_console_init(void);

#ifdef __cplusplus
}
#endif

#endif /* DEBUG_CONSOLE_H */
