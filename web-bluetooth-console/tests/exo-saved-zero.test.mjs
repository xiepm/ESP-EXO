import assert from 'node:assert/strict';
import test from 'node:test';
import {Opcode, Side, SavedZeroAction, Capability, encodeRequest, parseResponse,
  assertCorrelated, canReuseSavedWearZero} from '../lib/exo-ble-protocol.ts';
import {assessDeviceCompatibility} from '../lib/exo-device-compatibility.ts';
const golden=Uint8Array.from([1,0x9e,0x42,16,0,1,0x20,1,15,0,0,0,0x58,0x1b,0,0,0xe8,3,0,0]);
function field(bytes,offset,value) {const copy=bytes.slice(); new DataView(copy.buffer).setInt32(offset,value,true); return copy;}
test('saved zero C/TS golden vector; explicitly milli-degrees and correlated side query',()=>{
  assert.deepEqual([...encodeRequest(Opcode.GET_SAVED_ZERO,0x44,Uint8Array.of(Side.RIGHT))],[1,0x1e,0x44,1,1]);
  const status=parseResponse(golden);
  assert.equal(status.kind,'savedZeroStatus'); assert.equal(status.savedZeroDegrees,7);
  assert.equal(status.previewJointDegrees,1); assert.equal(status.reuseReady,true);
  assertCorrelated(status,Opcode.GET_SAVED_ZERO,0x42);
  assert.throws(()=>assertCorrelated(status,Opcode.GET_SAVED_ZERO,0x43));
});
test('save/restore/forget/capture-pair require exact two bytes and explicit confirmation',()=>{
  for(const action of Object.values(SavedZeroAction)) {
    assert.deepEqual([...encodeRequest(Opcode.SAVED_ZERO_ACTION,0x43,Uint8Array.of(action,1))],[1,0x1f,0x43,2,action,1]);
    assert.throws(()=>encodeRequest(Opcode.SAVED_ZERO_ACTION,1,Uint8Array.of(action,0)));
  }
  for(const payload of [Uint8Array.of(5,1),Uint8Array.of(1),Uint8Array.of(1,2),Uint8Array.of(1,1,1)])
    assert.throws(()=>encodeRequest(Opcode.SAVED_ZERO_ACTION,1,payload));
  assert.throws(()=>encodeRequest(Opcode.GET_SAVED_ZERO,1,Uint8Array.of(2)));
  const ack=parseResponse(Uint8Array.of(1,0x9f,0x43,4,0,0,1,0));
  assert.equal(ack.kind,'ack'); assertCorrelated(ack,Opcode.SAVED_ZERO_ACTION,0x43);
});
test('reject malformed saved status, flags, reserved bytes, invalid values and false ready claims',()=>{
  for(const [offset,value] of [[4,2],[5,2],[5,255],[6,128],[6,255],[7,0],[7,3],[8,31],[9,14],[10,1],[11,1]]) {
    const bad=golden.slice(); bad[offset]=value; assert.throws(()=>parseResponse(bad));
  }
  for(const bad of [field(golden,12,-2147483648),field(golden,16,-2147483648),
    field(golden,12,720001),field(golden,16,5001)]) assert.throws(()=>parseResponse(bad));
  assert.throws(()=>parseResponse(golden.slice(0,19)));
  const nonready=golden.slice(); nonready[8]=7; nonready[9]=7;
  assert.equal(parseResponse(field(nonready,16,6000)).reuseReady,false);
  const missing=field(field(golden,12,-2147483648),16,-2147483648); missing[8]=0; missing[9]=1;
  assert.equal(parseResponse(missing).savedZeroDegrees,null);
  assert.equal(parseResponse(missing).previewJointDegrees,null);
});
test('pair reuse requires both sides, distinct motors, positive readiness and bounded finite neutral preview',()=>{
  const left=parseResponse(golden), right={...left,side:Side.RIGHT,actuator:0,canId:0x10};
  assert.equal(canReuseSavedWearZero(left,right),true);
  for(const patch of [{stored:false},{bindingMatches:false},{previewValid:false},{reuseReady:false},
    {reason:7},{previewJointDegrees:5.001},{previewJointDegrees:NaN},{previewJointDegrees:null},{actuator:1},{side:Side.LEFT}])
    assert.equal(canReuseSavedWearZero(left,{...right,...patch}),false);
  assert.equal(canReuseSavedWearZero(left,null),false);
  for(const degrees of [-5,0,5]) assert.equal(canReuseSavedWearZero(left,{...right,previewJointDegrees:degrees}),true);
});
test('0.10.9 saved zero capability required; existing 0.10.8 remains compatible',()=>{
  const identity={protocolMinVersion:1,protocolMaxVersion:1,firmware:[0,10,9],capabilities:0x7f7f,maxFrameSize:20};
  assert.equal(assessDeviceCompatibility(identity).accepted,true);
  assert.equal(assessDeviceCompatibility({...identity,capabilities:0x3f7f}).accepted,false);
  assert.equal(assessDeviceCompatibility({...identity,firmware:[0,10,8],capabilities:0x3f7f}).accepted,true);
  assert.equal(Capability.SAVED_WEAR_ZERO,1<<14);
});
