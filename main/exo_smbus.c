#include "exo_smbus.h"

#include <stdio.h>
#include <string.h>
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define SMBUS_TRANSFER_TIMEOUT_MS 50
#define SMBUS_LOCK_TIMEOUT_MS 100

static i2c_master_bus_handle_t s_bus;
static SemaphoreHandle_t s_lock;
static exo_smbus_status_t s_status;

static bool take_lock(void)
{
    return s_lock && xSemaphoreTake(s_lock, pdMS_TO_TICKS(SMBUS_LOCK_TIMEOUT_MS)) == pdTRUE;
}

static bool valid_address(uint8_t address)
{
    return address >= 0x08 && address <= 0x77;
}

static uint8_t crc8(const uint8_t *data, size_t length)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (unsigned bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

static esp_err_t add_device(uint8_t address, i2c_master_dev_handle_t *device)
{
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = 100000,
        .scl_wait_us = 25000,
    };
    return i2c_master_bus_add_device(s_bus, &cfg, device);
}

static esp_err_t record_result(esp_err_t err)
{
    ++s_status.transactions;
    s_status.last_error = err;
    if (err != ESP_OK) {
        ++s_status.errors;
    }
    return err;
}

esp_err_t exo_smbus_init(void)
{
    /* Called once from app_main, before dispatching transport commands. */
    if (s_bus) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!s_lock) {
        s_lock = xSemaphoreCreateMutex();
        if (!s_lock) {
            return ESP_ERR_NO_MEM;
        }
    }
    i2c_master_bus_config_t cfg = {
        .i2c_port = I2C_NUM_1,
        .sda_io_num = GPIO_NUM_15,
        .scl_io_num = GPIO_NUM_16,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        /* Board has external 4.7k pullups; both lines remain released idle. */
        .flags.enable_internal_pullup = false,
    };
    esp_err_t err = i2c_new_master_bus(&cfg, &s_bus);
    s_status.initialized = err == ESP_OK;
    s_status.last_error = err;
    return err;
}

esp_err_t exo_smbus_get_status(exo_smbus_status_t *status)
{
    if (!status) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(status, 0, sizeof(*status));
    if (!take_lock()) {
        return s_lock ? ESP_ERR_TIMEOUT : ESP_ERR_INVALID_STATE;
    }
    *status = s_status;
    xSemaphoreGive(s_lock);
    return status->initialized ? ESP_OK : ESP_ERR_INVALID_STATE;
}

esp_err_t exo_smbus_read(uint8_t address, uint8_t command, bool word,
                       bool pec, uint16_t *value)
{
    if (!valid_address(address) || !value) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!take_lock()) {
        return s_lock ? ESP_ERR_TIMEOUT : ESP_ERR_INVALID_STATE;
    }
    if (!s_bus) {
        xSemaphoreGive(s_lock);
        return ESP_ERR_INVALID_STATE;
    }
    i2c_master_dev_handle_t device = NULL;
    uint8_t bytes[3] = {0};
    size_t length = word ? 2 : 1;
    esp_err_t err = add_device(address, &device);
    if (err == ESP_OK) {
        /* One combined transaction gives command + repeated START + read. */
        err = i2c_master_transmit_receive(device, &command, 1, bytes,
                                          length + (pec ? 1 : 0),
                                          SMBUS_TRANSFER_TIMEOUT_MS);
        esp_err_t remove_err = i2c_master_bus_rm_device(device);
        if (err == ESP_OK) {
            err = remove_err;
        }
    }
    if (err == ESP_OK && pec) {
        uint8_t packet[5] = { (uint8_t)(address << 1), command,
                              (uint8_t)((address << 1) | 1), bytes[0], bytes[1] };
        if (crc8(packet, 3 + length) != bytes[length]) {
            ++s_status.pec_errors;
            err = ESP_ERR_INVALID_CRC;
        }
    }
    if (err == ESP_OK) {
        *value = (uint16_t)bytes[0] | (word ? ((uint16_t)bytes[1] << 8) : 0);
    }
    record_result(err);
    xSemaphoreGive(s_lock);
    return err;
}

esp_err_t exo_smbus_write(uint8_t address, uint8_t command, bool word,
                        bool pec, uint16_t value)
{
    if (!valid_address(address) || (!word && value > 0xFF)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!take_lock()) {
        return s_lock ? ESP_ERR_TIMEOUT : ESP_ERR_INVALID_STATE;
    }
    if (!s_bus) {
        xSemaphoreGive(s_lock);
        return ESP_ERR_INVALID_STATE;
    }
    uint8_t packet[5] = { (uint8_t)(address << 1), command,
                          (uint8_t)value, (uint8_t)(value >> 8), 0 };
    size_t length = word ? 3 : 2; /* command + one/two bytes */
    if (pec) {
        packet[1 + length] = crc8(packet, 1 + length);
        ++length;
    }
    i2c_master_dev_handle_t device = NULL;
    esp_err_t err = add_device(address, &device);
    if (err == ESP_OK) {
        err = i2c_master_transmit(device, &packet[1], length, SMBUS_TRANSFER_TIMEOUT_MS);
        esp_err_t remove_err = i2c_master_bus_rm_device(device);
        if (err == ESP_OK) {
            err = remove_err;
        }
    }
    record_result(err);
    xSemaphoreGive(s_lock);
    return err;
}

esp_err_t exo_smbus_scan(char *out, size_t capacity)
{
    /* Reserve enough output for every legal address, rather than truncating. */
    if (!out || capacity < 600) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!take_lock()) {
        return s_lock ? ESP_ERR_TIMEOUT : ESP_ERR_INVALID_STATE;
    }
    if (!s_bus) {
        xSemaphoreGive(s_lock);
        return ESP_ERR_INVALID_STATE;
    }
    size_t used = (size_t)snprintf(out, capacity, "SMBus addresses:");
    unsigned found = 0;
    esp_err_t result = ESP_OK;
    for (uint16_t address = 0x08; address <= 0x77; ++address) {
        esp_err_t err = i2c_master_probe(s_bus, address, 10);
        if (err == ESP_OK) {
            used += (size_t)snprintf(out + used, capacity - used, " 0x%02X", address);
            ++found;
        } else if (err != ESP_ERR_NOT_FOUND) {
            /* Timeout/low clock is a bus fault; do not repeat 112 timeouts. */
            result = err;
            break;
        }
    }
    snprintf(out + used, capacity - used, " (%u found)\n", found);
    record_result(result);
    xSemaphoreGive(s_lock);
    return result;
}
