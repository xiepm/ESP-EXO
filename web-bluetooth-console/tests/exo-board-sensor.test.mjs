import assert from 'node:assert/strict';
import test from 'node:test';
import {Opcode,encodeRequest,parseResponse,supportsBoardSensors,supportsAdaptiveAssistMode,Capability} from '../lib/exo-ble-protocol.ts';
import {assessDeviceCompatibility} from '../lib/exo-device-compatibility.ts';
import {supportsAdaptive} from '../lib/exo-adaptive.ts';
import {supportsPersistentCalibration,supportsAtomicZeroCapture} from '../lib/exo-zero-calibration.ts';
const golden=Uint8Array.of(1,0xa7,0x42,16,0,11,10,0,0,0,160,63,0,0,32,192,0,0,52,67);
test('board sensor C/TS wire vector and bounded read-only pages',()=>{
  for(let page=0;page<4;page++) assert.deepEqual([...encodeRequest(Opcode.GET_BOARD_SENSOR,1,Uint8Array.of(page))],[1,0x27,1,1,page]);
  assert.throws(()=>encodeRequest(Opcode.GET_BOARD_SENSOR,1,Uint8Array.of(4)));
  const s=parseResponse(golden); assert.equal(s.kind,'boardSensor'); assert.equal(s.valid,true);
  assert.deepEqual(s.values,[1.25,-2.5,180]); assert.equal(s.ageMs,10);
  for(const [index,value] of [[4,4],[5,0x80],[5,1],[6,250]]) {
    const bad=golden.slice(); bad[index]=value; assert.throws(()=>parseResponse(bad));
  }
  const bad=golden.slice();new DataView(bad.buffer).setFloat32(8,NaN,true);assert.throws(()=>parseResponse(bad));
  assert.throws(()=>parseResponse(golden.slice(0,19)));
});
test('new firmware preserves AFO and requires existing capabilities',()=>{
  for(const patch of [12,13,14,15,16,17,18,19,20]) {
    const i={firmware:[0,11,patch],capabilities:0xffff,protocolMinVersion:1,protocolMaxVersion:1,maxFrameSize:20};
    assert.equal(assessDeviceCompatibility(i).accepted,true);
    assert.equal(supportsAdaptiveAssistMode(i),true);
    assert.equal(supportsAdaptive(i.firmware,i.capabilities),true);
    assert.equal(supportsPersistentCalibration(i),true);
    assert.equal(supportsAtomicZeroCapture(i),true);
    assert.equal(supportsAdaptiveAssistMode({...i,capabilities:0}),false);
    for(const bit of [12,13,14,15]) assert.equal(assessDeviceCompatibility({...i,capabilities:i.capabilities&~(1<<bit)}).accepted,false);
    assert.equal(supportsBoardSensors(i),patch>=13);
  }
});
