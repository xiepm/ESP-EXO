import assert from 'node:assert/strict';
import test from 'node:test';
import {readFile} from 'node:fs/promises';
import {Capability,Opcode,encodePulseTimingPayload,encodePulseWindowPayload,encodeRequest,
  parseResponse,assertCorrelated} from '../lib/exo-ble-protocol.ts';
import {assessDeviceCompatibility} from '../lib/exo-device-compatibility.ts';
const timing={durationMode:0,peakNm:0.2,fixedMs:600,ratio:0.3,minimumMs:150,maximumMs:1500,
  periodMinimumMs:400,periodMaximumMs:3000};
const vector=[1,0x21,0x42,16,1,0,200,0,88,2,44,1,150,0,220,5,144,1,184,11];
test('pulse timing matches actual C golden bytes and fits default ATT payload',()=>{
  const request=encodeRequest(Opcode.SET_PULSE_TIMING,0x42,encodePulseTimingPayload(timing));
  assert.deepEqual([...request],vector);
  const response=Uint8Array.from(vector);response[1]=0xa0;
  const parsed=parseResponse(response);
  assert.equal(parsed.kind,'pulseTiming');assert.equal(parsed.fixedMs,600);assert.equal(parsed.peakNm,0.2);
  assertCorrelated(parsed,Opcode.GET_PULSE_TIMING,0x42);
  assert.throws(()=>assertCorrelated(parsed,Opcode.GET_PULSE_TIMING,0x43));
});
test('pulse timing finite/precision/boundaries and mode invariants',()=>{
  for(const [key,values] of Object.entries({durationMode:[2,-1,0.5],peakNm:[NaN,Infinity,-0.001,15.001,0.1001],
    fixedMs:[99,3001,100.5],ratio:[0.099,0.601,0.1001],minimumMs:[99,1600],maximumMs:[149,2001],
    periodMinimumMs:[299,3000],periodMaximumMs:[400,5001]}))
    for(const value of values) assert.throws(()=>encodePulseTimingPayload({...timing,[key]:value}),`${key}=${value}`);
  for(const peakNm of [0,5,15]) for(const durationMode of [0,1])
    assert.equal(encodePulseTimingPayload({...timing,peakNm,durationMode}).length,16);
  const bad=Uint8Array.from(vector);bad[5]=2;
  assert.throws(()=>encodeRequest(Opcode.SET_PULSE_TIMING,0x42,bad.slice(4)));
  assert.throws(()=>encodeRequest(Opcode.SET_PULSE_TIMING,0x42,bad.slice(4,19)));
});
test('window degrees, signed reset and C response agree',()=>{
  assert.deepEqual([...encodeRequest(Opcode.SET_PULSE_WINDOW,0x43,encodePulseWindowPayload(8,30))],
    [1,0x23,0x43,4,32,3,184,11]);
  const parsed=parseResponse(Uint8Array.from([1,0xa2,0x43,8,1,0,32,3,184,11,244,1]));
  assert.equal(parsed.resetDegrees,8);assert.equal(parsed.maximumDegrees,30);
  assert.equal(parsed.eventSpeedDegreesPerSecond,5);
  for(const reset of [-30,-1,8]) assert.equal(encodePulseWindowPayload(reset,30).length,4);
  for(const [reset,max] of [[NaN,30],[8,Infinity],[-30.1,30],[60,90],[8,9],[8.001,30]])
    assert.throws(()=>encodePulseWindowPayload(reset,max));
});
test('single-side pulse status is distinct from complete gait phase',()=>{
  const bytes=Uint8Array.from([1,0xa4,0x44,16,0,3,7,0,3,0,1,0,88,2,232,3,200,0,136,19]);
  const p=parseResponse(bytes);
  assert.equal(p.kind,'pulseStatus');assert.equal(p.eventCount,3);assert.equal(p.pulseCount,1);
  assert.equal(p.progress,0.5);assert.equal(p.frozenDurationMs,600);assert.equal(p.frozenPeakNm,0.2);
  assert.equal(p.periodTrusted,true);assert.equal(p.rateLimited,true);assert.equal(p.armed,true);
  for(const [index,value] of [[4,2],[5,5],[6,8],[7,8],[19,255]]) {
    const bad=bytes.slice();bad[index]=value;assert.throws(()=>parseResponse(bad));
  }
  assert.throws(()=>encodeRequest(Opcode.GET_PULSE_STATUS,1,new Uint8Array([2])));
});
test('schema2 threshold readback never advertises old plateau ramp',()=>{
  const p=parseResponse(Uint8Array.from([1,0x8b,0x31,12,2,3,232,3,232,3,0,0,0,0,7,0]));
  assert.equal(p.pulseModel,true);assert.equal(p.rampMs,0);assert.equal(p.hysteresisDegrees,0);
  assert.equal(p.selectedThresholdDegrees,10);
});
test('0.10.10 requires pulse capability and older identity remains recognizable',()=>{
  const identity={protocolMinVersion:1,protocolMaxVersion:1,firmware:[0,10,10],capabilities:0xff7f,maxFrameSize:20};
  assert.equal(assessDeviceCompatibility(identity).accepted,true);
  for(const bit of [Capability.SWING_EVENT_PULSE,Capability.ANGLE_THRESHOLD_ASSIST,
    Capability.ANGLE_ASSIST_5NM,Capability.SAVED_WEAR_ZERO])
    assert.equal(assessDeviceCompatibility({...identity,capabilities:identity.capabilities&~bit}).accepted,false);
  assert.equal(assessDeviceCompatibility({...identity,firmware:[0,10,9],capabilities:0x7f7f}).accepted,true);
});
test('motion page exposes both duration modes, independent status and guarded writes',async()=>{
  const page=await readFile(new URL('../app/exo/page.tsx',import.meta.url),'utf8');
  for(const label of ['FIXED · 固定时长','PERIOD_SCALED · 本侧周期比例','下一脉冲申请峰值',
    '复位角（°）','允许前抬角上限','非步态相位','当前脉冲A和Ta冻结']) assert.ok(page.includes(label));
  assert.match(page,/Capability\.SWING_EVENT_PULSE/);
  assert.match(page,/await writePulseConfiguration\(pulse\)/);
  assert.doesNotMatch(page,/持续高于阈值.*一直|angleRiseNmPerSecond/);
});
