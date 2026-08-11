#ifndef EXO_BLE_H
#define EXO_BLE_H

/*
 * ESP-EXO 产品 BLE 服务的对外入口。
 *
 * 对 BLE 零基础读者，可以把 GATT 理解成设备公开的一张“数据接口表”：
 *
 *   手机
 *     ├─ Write  Command characteristic：写入一帧产品命令
 *     └─ Notify Response characteristic：订阅并接收对应响应
 *              │
 *              ▼
 *       exo_ble.c 的静态队列与 worker
 *              │
 *              ├─ exo_ble_protocol.c：严格按字节解码/编码
 *              ├─ exo_system：START/STOP 和整机状态
 *              └─ hip_joint：左右髋状态与 RAM 软件零位
 *
 * GATT 回调只复制命令并零等待入队；业务执行和 Notify 都由 BLE worker 完成，
 * 因而不会在 NimBLE 回调里执行启停序列，也不会成为新的 100 Hz 控制源。
 * BLE 连接只是命令传输会话：手机断开不会隐式 STOP，运动状态仍由 exo_system
 * 管理，手机若要停机必须显式发送 STOP 并确认系统回到 IDLE。
 */

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	/* 初始化、NimBLE Host、连接、Notify 订阅和 worker 的存活状态。 */
	bool initialized;
	bool host_synced;
	bool connected;
	bool response_subscribed;
	bool worker_task_alive;

	/* 当前传输会话及任务身份；session_generation 不发送到手机。 */
	uint16_t connection_handle;
	uint16_t negotiated_mtu;
	uint32_t session_generation;
	uint32_t boot_id;
	uintptr_t worker_task_identity;
	uint32_t worker_stack_free_min_bytes;

	/*
	 * 从 Command Write 到 Response Notify 的累计诊断计数。它们用于 UART
	 * 验收和故障定位，不是产品 BLE v1 的遥测载荷。
	 */
	uint32_t rx_write_count;
	uint32_t queued_count;
	uint32_t processed_count;
	uint32_t malformed_count;
	uint32_t duplicate_count;
	uint32_t request_id_conflict_count;
	uint32_t queue_full_count;
	uint32_t stale_drop_count;
	uint32_t notify_success_count;
	uint32_t notify_failure_count;
	uint32_t notify_tx_failure_count;
} exo_ble_snapshot_t;

/*
 * 初始化本固件唯一的 NimBLE Host、产品 GATT Service、静态命令队列和
 * BLE worker。重复调用在已初始化时直接成功，不会再创建第二个 worker。
 */
esp_err_t exo_ble_init(void);

/*
 * 在临界区内复制一份自洽的 BLE 诊断快照。返回 true 表示 BLE 已初始化；
 * 此接口只读，不连接、断开、发送 Notify，也不改变外骨骼状态。
 */
bool exo_ble_get_snapshot(exo_ble_snapshot_t *snapshot);

#ifdef __cplusplus
}
#endif

#endif /* EXO_BLE_H */
