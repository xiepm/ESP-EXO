#include <assert.h>
#include <stdio.h>
#include "exo_ble_protocol.h"

int main(void) {
    uint8_t frame[20]; size_t length; exo_ble_request_t request;
    const uint8_t read[] = {1, 0x1b, 42, 0}, restore[] = {1, 0x1c, 43, 0};
    assert(exo_ble_protocol_decode_request(read, 4, &request) == EXO_BLE_RESULT_OK);
    assert(exo_ble_protocol_decode_request(restore, 4, &request) == EXO_BLE_RESULT_OK);
    const uint8_t bad[] = {1, 0x1c, 43, 1, 1};
    assert(exo_ble_protocol_decode_request(bad, 5, &request) == EXO_BLE_RESULT_INVALID_PAYLOAD);
    puts("PASS CAN GET and recovery request boundaries");
    exo_ble_can_status_t status = {.state = 1, .flags = 3, .rx_error_counter = 124,
        .bus_error_count = 729178843, .tx_failed_count = 65535, .motor_flags = {21, 7}};
    assert(exo_ble_protocol_encode_can_status(42, &status, frame, 20, &length));
    assert(length == 20 && frame[0] == 1 && frame[1] == 0x9b && frame[2] == 42 && frame[3] == 16);
    assert(frame[8] == 124 && frame[9] == 0 && frame[14] == 255 && frame[15] == 255);
    uint32_t count = (uint32_t)frame[10] | ((uint32_t)frame[11] << 8) |
        ((uint32_t)frame[12] << 16) | ((uint32_t)frame[13] << 24);
    assert(count == 729178843 && frame[18] == 21 && frame[19] == 7);
    puts("PASS 20-byte layout and full cumulative CAN counter");
    assert(!exo_ble_protocol_encode_can_status(42, &status, frame, 19, &length) && length == 0);
    status.state = 5;
    assert(!exo_ble_protocol_encode_can_status(42, &status, frame, 20, &length) && length == 0);
    status.state = 1; status.flags = 4;
    assert(!exo_ble_protocol_encode_can_status(42, &status, frame, 20, &length));
    status.flags = 3; status.motor_flags[0] = 32;
    assert(!exo_ble_protocol_encode_can_status(42, &status, frame, 20, &length));
    puts("PASS capacity, state and reserved flags fail closed");
    puts("RESULT 3/3 PASS");
}
