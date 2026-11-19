#include <assert.h>
#include <stdio.h>
#include "exo_ble_protocol.h"

int main(void) {
    uint8_t frame[14] = {1, EXO_BLE_OPCODE_SET_ASSIST_CONFIG_WITH_LIMIT, 1, 10,
        1, EXO_BLE_ASSIST_MODE_ADAPTIVE_GAUSSIAN, EXO_BLE_ASSIST_EXEC_ACTIVE,
        EXO_BLE_ASSIST_STRATEGY_ADAPTIVE_GAUSSIAN, 0, 100, 100, 0, 0x98, 0x3a};
    exo_ble_request_t request;
    assert(exo_ble_protocol_decode_request(frame, sizeof(frame), &request) == EXO_BLE_RESULT_OK);
    assert(request.maximum_assist_torque_mnm == 15000);
    frame[12]++;
    assert(exo_ble_protocol_decode_request(frame, sizeof(frame), &request) == EXO_BLE_RESULT_INVALID_PAYLOAD);
    exo_ble_wire_assist_config_t status={.schema_version=1,
        .selected_mode=EXO_BLE_ASSIST_MODE_ADAPTIVE_GAUSSIAN,
        .active_mode=EXO_BLE_ASSIST_MODE_ADAPTIVE_GAUSSIAN,
        .requested_execution_state=EXO_BLE_ASSIST_EXEC_ACTIVE,
        .execution_state=EXO_BLE_ASSIST_EXEC_ACTIVE,
        .strategy_id=EXO_BLE_ASSIST_STRATEGY_ADAPTIVE_GAUSSIAN,.strategy_version=7,
        .left_level_percent=25,.right_level_percent=75,
        .assist_flags=EXO_BLE_ASSIST_FLAG_SELECTED_VALID|EXO_BLE_ASSIST_FLAG_ACTIVE_VALID};
    uint8_t reply[20];size_t reply_size;
    for(unsigned reason=9;reason<=11;reason++) {
        status.unavailable_reason=(exo_ble_wire_assist_unavailable_t)reason;
        assert(exo_ble_protocol_encode_assist_config(42,&status,reply,sizeof(reply),&reply_size));
        assert(reply_size==20&&reply[14]==reason&&reply[7]==0x22&&reply[9]==7);
        assert(!(reply[13]&EXO_BLE_ASSIST_FLAG_REAL_OUTPUT_ENABLED));
    }
    status.unavailable_reason=(exo_ble_wire_assist_unavailable_t)12;
    assert(!exo_ble_protocol_encode_assist_config(42,&status,reply,sizeof(reply),&reply_size));
    for(unsigned opcode=EXO_BLE_OPCODE_PREPARE_ADAPTIVE_TRIAL;opcode<=EXO_BLE_OPCODE_CHECK_ADAPTIVE_TRIAL;opcode++) {
        uint8_t trial[5]={1,(uint8_t)opcode,7,0,0};
        assert(exo_ble_protocol_decode_request(trial,4,&request)==EXO_BLE_RESULT_OK);
        trial[3]=1;
        assert(exo_ble_protocol_decode_request(trial,5,&request)==EXO_BLE_RESULT_INVALID_PAYLOAD);
    }
    frame[5] = EXO_BLE_ASSIST_MODE_ACTIVE_PHASE;
    frame[7] = EXO_BLE_ASSIST_STRATEGY_PHASE_PROFILE;
    frame[12] = 0xf4; frame[13] = 1;
    assert(exo_ble_protocol_decode_request(frame, sizeof(frame), &request) == EXO_BLE_RESULT_OK);
    frame[12]++;
    assert(exo_ble_protocol_decode_request(frame, sizeof(frame), &request) == EXO_BLE_RESULT_INVALID_PAYLOAD);
    puts("PASS wire boundaries: AFO 15000 accepted, 15001 rejected; legacy phase profile 500 accepted, 501 rejected; semantic build gate remains downstream");
    return 0;
}
