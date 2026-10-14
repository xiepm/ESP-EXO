/* 纯C协议回放：不加载BLE/电机驱动，向量与网页测试相同。 */
#include "exo_ble_protocol.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

int main(void) {
    exo_ble_request_t req;
    const uint8_t get[] = {1, 0x0a, 0x31, 0};
    assert(exo_ble_protocol_decode_request(get, sizeof(get), &req) == EXO_BLE_RESULT_OK);
    uint8_t set[] = {1, 0x19, 0x32, 10, 1, 2, 2, 2, 1, 10, 50, 0, 200, 0};
    assert(exo_ble_protocol_decode_request(set, sizeof(set), &req) == EXO_BLE_RESULT_OK);
    assert(req.maximum_assist_torque_mnm == 200 && req.assist_config.left_level_percent == 10 && req.assist_config.right_level_percent == 50);
    puts("PASS atomic config+maximum request golden vector");
    set[12] = 0; assert(exo_ble_protocol_decode_request(set, sizeof(set), &req) == EXO_BLE_RESULT_OK);
    set[12] = 244; set[13] = 1;
    assert(exo_ble_protocol_decode_request(set, sizeof(set), &req) == EXO_BLE_RESULT_OK);
    set[12] = 245; assert(exo_ble_protocol_decode_request(set, sizeof(set), &req) == EXO_BLE_RESULT_INVALID_PAYLOAD);
    set[12] = 255; set[13] = 255;
    assert(exo_ble_protocol_decode_request(set, sizeof(set), &req) == EXO_BLE_RESULT_INVALID_PAYLOAD);
    set[12] = 200; set[13] = 0; set[9] = 101;
    assert(exo_ble_protocol_decode_request(set, sizeof(set), &req) == EXO_BLE_RESULT_INVALID_PAYLOAD);
    set[9] = 10; set[11] = 1;
    assert(exo_ble_protocol_decode_request(set, sizeof(set), &req) == EXO_BLE_RESULT_INVALID_PAYLOAD);
    puts("PASS 0/500 boundaries, oversized maximum, percent and reserved flags");
    const uint8_t legacy[] = {1, 0x16, 0x31, 8, 1, 2, 2, 2, 1, 10, 50, 0};
    assert(exo_ble_protocol_decode_request(legacy, sizeof(legacy), &req) == EXO_BLE_RESULT_OK);
    puts("PASS existing 8-byte configuration remains accepted");
    exo_ble_wire_assist_torque_limit_t limit = {1, 200, 0, 500};
    uint8_t frame[20]; size_t length;
    const uint8_t golden[] = {1, 0x8a, 0x31, 8, 1, 1, 200, 0, 0, 0, 244, 1};
    assert(exo_ble_protocol_encode_assist_torque_limit(0x31, &limit, frame, sizeof(frame), &length));
    assert(length == sizeof(golden) && memcmp(frame, golden, length) == 0);
    assert(!exo_ble_protocol_encode_assist_torque_limit(0x31, &limit, frame, 11, &length) && length == 0);
    limit.flags = 3; limit.active_maximum_mnm = 123;
    assert(exo_ble_protocol_encode_assist_torque_limit(0x31, &limit, frame, sizeof(frame), &length));
    assert(frame[8] == 123 && frame[9] == 0);
    limit.flags = 1;
    assert(!exo_ble_protocol_encode_assist_torque_limit(0x31, &limit, frame, sizeof(frame), &length));
    limit.flags = 2; limit.selected_maximum_mnm = 0;
    assert(!exo_ble_protocol_encode_assist_torque_limit(0x31, &limit, frame, sizeof(frame), &length));
    puts("PASS readback golden vector, active flags and capacity");
    puts("RESULT 4/4 PASS");
    return 0;
}
