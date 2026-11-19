import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import test from 'node:test';
import {Opcode, AssistStrategy, Capability, encodeRequest, encodePulseTimingPayload,
  parseResponse, assertPulseTimingCapability, hasExtendedPulseLimits,
  assertAssistTorqueLimitCapability, assistTorqueMaximumNm, pulseRiseLimitNmPerSecond} from '../lib/exo-ble-protocol.ts';
import {assessDeviceCompatibility} from '../lib/exo-device-compatibility.ts';
import {supportsAtomicZeroCapture} from '../lib/exo-zero-calibration.ts';
const timing={durationMode:0,peakNm:15,fixedMs:3000,ratio:0.3,minimumMs:150,maximumMs:1500,periodMinimumMs:400,periodMaximumMs:3000};
const firmware=[0,10,12], capabilities=0xff7f;
test('15 Nm/3000 ms C/TS golden request and readback fit20B; rejects one unit beyond bounds',()=>{
  const wire=encodeRequest(Opcode.SET_PULSE_TIMING,0x42,encodePulseTimingPayload(timing));
  assert.deepEqual([...wire],[1,0x21,0x42,16,1,0,152,58,184,11,44,1,150,0,220,5,144,1,184,11]);
  const response=wire.slice();response[1]=0xa0;
  const parsed=parseResponse(response);assert.equal(parsed.peakNm,15);assert.equal(parsed.fixedMs,3000);
  for(const patch of [{peakNm:15.001},{fixedMs:3001},{maximumMs:2001},{fixedMs:NaN},{peakNm:Infinity}])
    assert.throws(()=>encodePulseTimingPayload({...timing,...patch}));
});
test('extended range requires known0.10.12 and bits13/15; old5 Nm/2000ms request remains valid',()=>{
  assert.equal(hasExtendedPulseLimits(capabilities,firmware),true);
  assert.doesNotThrow(()=>assertPulseTimingCapability(timing,capabilities,firmware));
  for(const old of [[0,10,10],[0,10,11],[0,10,14],[]]) {
    assert.equal(hasExtendedPulseLimits(capabilities,old),false);
    assert.throws(()=>assertPulseTimingCapability(timing,capabilities,old));
    assert.doesNotThrow(()=>assertPulseTimingCapability({...timing,peakNm:5,fixedMs:2000},capabilities,old));
  }
  for(const bit of [Capability.ANGLE_ASSIST_5NM,Capability.SWING_EVENT_PULSE])
    assert.throws(()=>assertPulseTimingCapability(timing,capabilities&~bit,firmware));
});
test('15 Nm limit readback must match new identity; phase remains0.5 and old devices5',()=>{
  const bytes=Uint8Array.from([1,0x8a,0x43,8,1,1,152,58,0,0,152,58]);
  const response=parseResponse(bytes);assert.equal(response.hardMaximumNm,15);assert.equal(response.selectedMaximumNm,15);
  assert.doesNotThrow(()=>assertAssistTorqueLimitCapability(response,capabilities,firmware));
  assert.throws(()=>assertAssistTorqueLimitCapability(response,capabilities,[0,10,11]));
  assert.equal(assistTorqueMaximumNm(AssistStrategy.ANGLE_THRESHOLD,capabilities,15,firmware),15);
  assert.equal(assistTorqueMaximumNm(AssistStrategy.ANGLE_THRESHOLD,capabilities,null,firmware),0.5);
  assert.equal(assistTorqueMaximumNm(AssistStrategy.ANGLE_THRESHOLD,capabilities,15,[0,10,11]),5);
  assert.equal(assistTorqueMaximumNm(AssistStrategy.PHASE_PROFILE,capabilities,15,firmware),0.5);
  const bad=bytes.slice();bad[6]=153;assert.throws(()=>parseResponse(bad));
});
test('new fixed pulse status supports3000ms and actual9.549Nm; old20B format unchanged',()=>{
  const bytes=Uint8Array.from([1,0xa4,0x44,16,0,3,7,0,1,0,1,0,184,11,232,3,77,37,136,19]);
  const p=parseResponse(bytes);assert.equal(p.frozenDurationMs,3000);assert.equal(p.frozenPeakNm,9.549);assert.equal(p.progress,0.5);
  const bad=bytes.slice();bad[12]=185;assert.throws(()=>parseResponse(bad));
});
test('0.10.12 inherits pair-zero and pulse identities, does not bypass required capability bits',()=>{
  const info={protocolMinVersion:1,protocolMaxVersion:1,firmware,capabilities,maxFrameSize:20};
  assert.equal(assessDeviceCompatibility(info).accepted,true);assert.equal(supportsAtomicZeroCapture(info),true);
  for(const bit of [13,14,15]) assert.equal(assessDeviceCompatibility({...info,capabilities:capabilities&~(1<<bit)}).accepted,false);
});
test('build version and actual BLE INFO version agree, guarding the previous unsynced patch number',async()=>{
  const cmake=await readFile(new URL('../../CMakeLists.txt',import.meta.url),'utf8');
  const ble=await readFile(new URL('../../main/exo_ble.c',import.meta.url),'utf8');
  const info=['MAJOR','MINOR','PATCH'].map(p=>Number(ble.match(new RegExp(`#define EXO_FIRMWARE_VERSION_${p} (\\d+)U`))?.[1]));
  assert.equal(cmake.match(/set\(PROJECT_VER "([^"]+)"\)/)?.[1],info.join('.'));
  assert.equal(info.join('.'),'0.11.23');
});
test('20 Nm/s is displayed only for known0.10.13; previous pulse firmwares retain10',()=>{
  assert.equal(pulseRiseLimitNmPerSecond(capabilities,[0,10,13]),20);
  assert.equal(pulseRiseLimitNmPerSecond(capabilities,[0,11,0]),20);
  for(const v of [10,11,12]) assert.equal(pulseRiseLimitNmPerSecond(capabilities,[0,10,v]),10);
  for(const v of [[],[0,10,9],[0,10,14]]) assert.equal(pulseRiseLimitNmPerSecond(capabilities,v),null);
  assert.equal(pulseRiseLimitNmPerSecond(capabilities&~Capability.SWING_EVENT_PULSE,[0,10,13]),null);
});
test('0.10.13 keeps expanded range and pair zero; independent golden15Nm timing bytes remain unchanged',()=>{
  const firmware=[0,10,13];
  assert.equal(hasExtendedPulseLimits(capabilities,firmware),true);
  assert.doesNotThrow(()=>assertPulseTimingCapability(timing,capabilities,firmware));
  assert.equal(supportsAtomicZeroCapture({firmware,capabilities}),true);
  assert.equal(assessDeviceCompatibility({protocolMinVersion:1,protocolMaxVersion:1,firmware,capabilities,maxFrameSize:20}).accepted,true);
});
