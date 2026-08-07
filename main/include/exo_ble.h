#ifndef EXO_BLE_H
#define EXO_BLE_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	bool initialized;
	bool host_synced;
	bool connected;
	bool response_subscribed;
	bool worker_task_alive;

	uint16_t connection_handle;
	uint16_t negotiated_mtu;
	uint32_t session_generation;
	uint32_t boot_id;
	uintptr_t worker_task_identity;
	uint32_t worker_stack_free_min_bytes;

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

/* Initializes the sole NimBLE host, product GATT service and BLE worker. */
esp_err_t exo_ble_init(void);

/* Copies one coherent diagnostic snapshot. */
bool exo_ble_get_snapshot(exo_ble_snapshot_t *snapshot);

#ifdef __cplusplus
}
#endif

#endif /* EXO_BLE_H */
