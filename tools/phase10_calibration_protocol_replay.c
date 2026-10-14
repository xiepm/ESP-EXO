/* 直接链接固件纯C编解码器，验证新增只读帧不改变既有v1、且缺失值不会伪装0。 */
#include "exo_ble_protocol.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

int main(void) {
    exo_ble_request_t request;
    const uint8_t query[] = {1, 9, 0x31, 1, 0};
    assert(exo_ble_protocol_decode_request(query, sizeof(query), &request) == EXO_BLE_RESULT_OK);
    assert(request.side == EXO_BLE_SIDE_LEFT);
    const uint8_t wrong_side[] = {1, 9, 0x31, 1, 255};
    assert(exo_ble_protocol_decode_request(wrong_side, sizeof(wrong_side), &request) == EXO_BLE_RESULT_INVALID_PAYLOAD);
    const uint8_t short_query[] = {1, 9, 0x31, 0};
    assert(exo_ble_protocol_decode_request(short_query, sizeof(short_query), &request) == EXO_BLE_RESULT_INVALID_PAYLOAD);
    puts("PASS request boundaries");

    exo_ble_calibration_status_t status = {
        .side = EXO_BLE_SIDE_LEFT, .actuator = 1, .can_id = 0x20,
        .direction = EXO_BLE_DIRECTION_POSITIVE, .joint_flags = 0x1b,
        .raw_position_mrad = 523, .software_zero_mrad = INT32_MIN, .feedback_age_ms = 1
    };
    uint8_t frame[20];
    size_t length = 0;
    const uint8_t golden[] = {1, 0x89, 0x25, 16, 0, 1, 0x20, 1, 0x1b, 0,
        0x0b, 2, 0, 0, 0, 0, 0, 0x80, 1, 0};
    assert(exo_ble_protocol_encode_calibration_status(0x25, &status, frame, sizeof(frame), &length));
    assert(length == 20 && memcmp(frame, golden, sizeof(golden)) == 0);
    puts("PASS LEFT mapping and absent zero golden vector");
    assert(!exo_ble_protocol_encode_calibration_status(0x25, &status, frame, 19, &length));
    assert(length == 0);
    assert(!exo_ble_protocol_encode_calibration_status(0x25, NULL, frame, 20, &length));
    puts("PASS destination capacity and null status");

    status.actuator = 2;
    assert(!exo_ble_protocol_encode_calibration_status(0x25, &status, frame, 20, &length));
    status.actuator = 0;
    status.can_id = 0x80;
    assert(!exo_ble_protocol_encode_calibration_status(0x25, &status, frame, 20, &length));
    puts("PASS invalid actuator/CAN boundaries");

    status.side = EXO_BLE_SIDE_RIGHT; status.can_id = 0x10;
    status.direction = EXO_BLE_DIRECTION_NEGATIVE; status.joint_flags = 0x3f;
    status.raw_position_mrad = -523; status.software_zero_mrad = -1000;
    assert(exo_ble_protocol_encode_calibration_status(0x25, &status, frame, 20, &length));
    assert(frame[4] == 1 && frame[5] == 0 && frame[6] == 0x10 && frame[7] == 2);
    assert(frame[10] == 0xf5 && frame[11] == 0xfd && frame[12] == 255 && frame[13] == 255);
    puts("PASS RIGHT direction and signed positions");
    puts("RESULT 5/5 PASS");
    return 0;
}
