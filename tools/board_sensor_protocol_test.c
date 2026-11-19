#include "exo_ble_protocol.h"
#include "exo_telemetry.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
int main(void) {
    assert(!exo_telemetry_has_applied_assist(0,0));
    assert(!exo_telemetry_has_applied_assist(NAN,INFINITY));
    assert(exo_telemetry_has_applied_assist(-2,0));
    assert(exo_telemetry_has_applied_assist(0,3));
    uint8_t afo_frame[20];size_t afo_length;
    const exo_ble_afo_status_t afo={.side=1,.faults=44,.first_faults=44,.flags=6,
        .lock_reasons=64,.first_timestamp_ms=14610,.first_q_cdeg=2991,
        .first_v_ddeg_s=-3157,.first_source_interval_ms=40,.first_age_ms=10};
    const uint8_t afo_golden[]={1,0xa9,7,16,1,44,44,6,64,0,0x12,0x39,0,0,0xaf,0x0b,0xab,0xf3,40,10};
    assert(exo_ble_protocol_encode_afo_status(7,&afo,afo_frame,20,&afo_length));
    assert(afo_length==20 && memcmp(afo_frame,afo_golden,20)==0);
    uint8_t afo_request[]={1,0x29,7,1,1};exo_ble_request_t afo_decoded;
    assert(exo_ble_protocol_decode_request(afo_request,5,&afo_decoded)==EXO_BLE_RESULT_OK && afo_decoded.side==1);
    afo_request[4]=2;
    assert(exo_ble_protocol_decode_request(afo_request,5,&afo_decoded)==EXO_BLE_RESULT_INVALID_PAYLOAD);
    assert(!exo_ble_protocol_encode_afo_status(7,&afo,afo_frame,19,&afo_length));
    exo_ble_afo_status_t bad_afo=afo;bad_afo.flags=8;
    assert(!exo_ble_protocol_encode_afo_status(7,&bad_afo,afo_frame,20,&afo_length));
    puts("PASS AFO first-fault wire vector and nonzero applied-output flags");
    uint8_t preview_frame[20];size_t preview_length;
    const uint8_t preview_golden[]={1,0xa8,7,16,1,9,0,0,0x18,0xfc,0xff,0xff,0x30,0xf8,0xff,0xff,0x50,0xc3,10,0};
    assert(exo_ble_protocol_encode_direction_preview(7,1,9,0,0,-1000,-2000,50000,10,preview_frame,20,&preview_length));
    assert(preview_length==20 && memcmp(preview_frame,preview_golden,20)==0);
    uint8_t preview_request[]={1,0x28,7,1,1};exo_ble_request_t preview_decoded;
    assert(exo_ble_protocol_decode_request(preview_request,5,&preview_decoded)==EXO_BLE_RESULT_OK && preview_decoded.side==1);
    preview_request[4]=2;
    assert(exo_ble_protocol_decode_request(preview_request,5,&preview_decoded)==EXO_BLE_RESULT_INVALID_PAYLOAD);
    assert(!exo_ble_protocol_encode_direction_preview(7,1,0,0,0,-1000,-2000,50000,10,preview_frame,20,&preview_length));
    assert(!exo_ble_protocol_encode_direction_preview(7,1,9,0,0,-1000,-2000,50000,51,preview_frame,20,&preview_length));
    puts("PASS exact direction preview request/baseline/raw wire vectors and invalid session rejection");
    const uint8_t golden[] = {1,0xa7,0x42,16,0,11,10,0,0,0,160,63,0,0,32,192,0,0,52,67};
    float v[]={1.25f,-2.5f,180.f}; uint8_t out[20]; size_t n=0;
    assert(exo_ble_protocol_encode_board_sensor(0x42,0,11,10,v,out,sizeof(out),&n));
    assert(n==20 && memcmp(out,golden,20)==0);
    for (uint8_t p=0;p<4;++p) {
        uint8_t req[]={1,0x27,1,1,p}; exo_ble_request_t r;
        assert(exo_ble_protocol_decode_request(req,5,&r)==EXO_BLE_RESULT_OK);
        assert(r.board_sensor_page==p);
    }
    uint8_t bad[]={1,0x27,1,1,4}; exo_ble_request_t r;
    assert(exo_ble_protocol_decode_request(bad,5,&r)==EXO_BLE_RESULT_INVALID_PAYLOAD);
    assert(!exo_ble_protocol_encode_board_sensor(1,0,11,250,v,out,20,&n) && !n);
    assert(!exo_ble_protocol_encode_board_sensor(1,0,1,0,v,out,20,&n));
    assert(!exo_ble_protocol_encode_board_sensor(1,4,0,0,v,out,20,&n));
    assert(!exo_ble_protocol_encode_board_sensor(1,0,0,0,v,out,19,&n));
    v[0]=NAN; assert(!exo_ble_protocol_encode_board_sensor(1,0,0,0,v,out,20,&n));
    puts("PASS board sensor C wire vectors, pages, invalid/stale/NaN and capacity");
}
