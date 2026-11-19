#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

typedef struct {
    bool initialized;
    uint32_t transactions;
    uint32_t errors;
    uint32_t pec_errors;
    esp_err_t last_error;
} exo_smbus_status_t;

/* GPIO15 SDA, GPIO16 SCL; I2C1 100kHz. No startup scan/transactions. */
esp_err_t exo_smbus_init(void);
esp_err_t exo_smbus_get_status(exo_smbus_status_t *status);
/* SMBus Read/Write Byte or Word Data. Word data is little endian on wire. */
esp_err_t exo_smbus_read(uint8_t address, uint8_t command, bool word,
                       bool pec, uint16_t *value);
esp_err_t exo_smbus_write(uint8_t address, uint8_t command, bool word,
                        bool pec, uint16_t value);
/* Explicit diagnostic scan; addresses excludes reserved 0x00..07/78..7f. */
esp_err_t exo_smbus_scan(char *out, size_t capacity);
