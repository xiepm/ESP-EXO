#include "exo_factory_direction.h"

#include <stdbool.h>

#include "nvs.h"
#include "nvs_flash.h"

/* 每侧仅一个32位记录；NVS自身提供Flash页管理和校验，不自建文件/数据库。 */
#define FACTORY_NAMESPACE "exo_factory"
#define RECORD_VERSION 0xD1U

static const char *direction_key(exo_hip_joint_id_t joint) {
    return joint == EXO_HIP_JOINT_LEFT ? "left_dir_v1" : "right_dir_v1";
}

static bool binding_valid(exo_hip_joint_id_t joint, exo_actuator_id_t actuator) {
    return (unsigned)joint < EXO_HIP_JOINT_COUNT &&
        (unsigned)actuator < EXO_ACTUATOR_COUNT;
}

static uint32_t encode_record(exo_actuator_id_t actuator, exo_can_node_id_t can_id,
    exo_direction_t direction) {
    return (RECORD_VERSION << 24) | ((uint32_t)can_id << 16) |
        ((uint32_t)actuator << 8) | (direction == EXO_DIRECTION_POSITIVE ? 1U : 2U);
}

esp_err_t exo_factory_direction_storage_init(void) {
    /* 错误直接上报；不能为修复BLE而自动擦除包含出厂配置的整个NVS分区。 */
    return nvs_flash_init();
}

esp_err_t exo_factory_direction_load(exo_hip_joint_id_t joint,
    exo_actuator_id_t actuator, exo_can_node_id_t can_id, exo_direction_t *direction) {
    if (!direction || !binding_valid(joint, actuator)) return ESP_ERR_INVALID_ARG;
    *direction = EXO_DIRECTION_UNKNOWN;
    nvs_handle_t handle;
    esp_err_t error = nvs_open(FACTORY_NAMESPACE, NVS_READONLY, &handle);
    if (error == ESP_ERR_NVS_NOT_FOUND) return ESP_OK;
    if (error != ESP_OK) return error;
    uint32_t record = 0U;
    error = nvs_get_u32(handle, direction_key(joint), &record);
    nvs_close(handle);
    if (error == ESP_ERR_NVS_NOT_FOUND) return ESP_OK;
    if (error != ESP_OK) return error;
    /* 缺失、旧版本、损坏或映射不一致都保持UNKNOWN；不猜测安装方向。 */
    if (record == encode_record(actuator, can_id, EXO_DIRECTION_POSITIVE))
        *direction = EXO_DIRECTION_POSITIVE;
    else if (record == encode_record(actuator, can_id, EXO_DIRECTION_NEGATIVE))
        *direction = EXO_DIRECTION_NEGATIVE;
    return ESP_OK;
}

esp_err_t exo_factory_direction_save(exo_hip_joint_id_t joint,
    exo_actuator_id_t actuator, exo_can_node_id_t can_id, exo_direction_t direction) {
    if (!binding_valid(joint, actuator) ||
        (direction != EXO_DIRECTION_POSITIVE && direction != EXO_DIRECTION_NEGATIVE))
        return ESP_ERR_INVALID_ARG;
    nvs_handle_t handle;
    esp_err_t error = nvs_open(FACTORY_NAMESPACE, NVS_READWRITE, &handle);
    if (error != ESP_OK) return error;
    const uint32_t record = encode_record(actuator, can_id, direction);
    error = nvs_set_u32(handle, direction_key(joint), record);
    if (error == ESP_OK) error = nvs_commit(handle);
    uint32_t verified = 0U;
    if (error == ESP_OK) error = nvs_get_u32(handle, direction_key(joint), &verified);
    if (error == ESP_OK && verified != record) error = ESP_FAIL;
    nvs_close(handle);
    return error;
}
