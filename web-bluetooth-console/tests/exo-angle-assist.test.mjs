import assert from "node:assert/strict";
import test from "node:test";
import {AssistMode, AssistStrategy, AssistExecutionState, Opcode, Capability,
  encodeAngleAssistConfigPayload, encodeAssistConfigWithLimitPayload, encodeRequest,
  parseResponse, assertCorrelated, assertAssistTorqueLimitCapability, assistTorqueMaximumNm}
  from "../lib/exo-ble-protocol.ts";
import {assessDeviceCompatibility} from "../lib/exo-device-compatibility.ts";
const config = {selectedMode: AssistMode.ANGLE_THRESHOLD,
  requestedExecutionState: AssistExecutionState.ACTIVE, strategyId: AssistStrategy.ANGLE_THRESHOLD,
  profileId: 0, leftLevelPercent: 50, rightLevelPercent: 100};

test("angle atomic request matches firmware golden vector, explicit degrees and N m", () => {
  const payload = encodeAngleAssistConfigPayload(config, 0.2, 10);
  assert.deepEqual([...encodeRequest(Opcode.SET_ANGLE_ASSIST_CONFIG, 0x32, payload)],
    [1,0x1d,0x32,12,1,4,2,4,0,50,100,0,200,0,232,3]);
  assert.equal(encodeRequest(Opcode.GET_ANGLE_ASSIST_CONFIG, 0x31).length,4);
  for (const execution of [0,1,2])
    assert.equal(encodeAngleAssistConfigPayload({...config, requestedExecutionState: execution},0,1)[2],execution);
});
test("angle/torque finite, precision, bounds, mode/profile and legacy opcode guards", () => {
  for (const angle of [0,0.99,60.01,NaN,Infinity,10.001])
    assert.throws(() => encodeAngleAssistConfigPayload(config,0.2,angle));
  for (const torque of [-0.001,15.001,NaN,Infinity,0.1001])
    assert.throws(() => encodeAngleAssistConfigPayload(config,torque,10));
  for (const angle of [1,60]) for (const torque of [0,0.5,5])
    assert.equal(encodeAngleAssistConfigPayload(config,torque,angle).length,12);
  assert.throws(() => encodeAngleAssistConfigPayload({...config,profileId:1},0.2,10));
  assert.throws(() => encodeAngleAssistConfigPayload({...config,strategyId:2},0.2,10));
  const payload = encodeAngleAssistConfigPayload(config,0.2,10);
  assert.throws(() => encodeRequest(Opcode.SET_ASSIST_CONFIG_WITH_LIMIT,1,payload.slice(0,10)));
  const bad = payload.slice(); bad[10]=99; bad[11]=0;
  assert.throws(() => encodeRequest(Opcode.SET_ANGLE_ASSIST_CONFIG,1,bad));
});
test("angle readback generation, active values and fixed hysteresis/ramp golden vector", () => {
  const bytes = Uint8Array.from([1,0x8b,0x31,12,1,3,232,3,232,3,100,0,200,0,7,0]);
  const response = parseResponse(bytes);
  assert.equal(response.kind,"angleAssistConfig");
  assert.equal(response.selectedThresholdDegrees,10);
  assert.equal(response.activeThresholdDegrees,10);
  assert.equal(response.configGeneration,7);
  assert.equal(response.hysteresisDegrees,1);
  assert.equal(response.rampMs,200);
  assertCorrelated(response,Opcode.GET_ANGLE_ASSIST_CONFIG,0x31);
  for (const index of [4,5,10,12,15]) {
    const bad=bytes.slice(); bad[index]=255;
    assert.throws(() => parseResponse(bad));
  }
  const idle = bytes.slice(); idle[5]=1; idle[8]=0; idle[9]=0;
  assert.equal(parseResponse(idle).activeThresholdDegrees,null);
});
test("0.10.7 requires new capability and keeps 0.10.6 supported without sending new commands", () => {
  const identity = {protocolMinVersion:1,protocolMaxVersion:1,firmware:[0,10,7],
    capabilities:0x1f7f,maxFrameSize:20};
  assert.equal(assessDeviceCompatibility(identity).accepted,true);
  assert.equal(assessDeviceCompatibility({...identity,capabilities:0x0f7f}).accepted,false);
  assert.equal(assessDeviceCompatibility({...identity,firmware:[0,10,6],capabilities:0x0f7f}).accepted,true);
  assert.equal(Capability.ANGLE_THRESHOLD_ASSIST,1<<12);
});

test("5 N m C/web atomic vector and phase strategy retains 0.5 N m", () => {
  const payload=encodeAngleAssistConfigPayload(config,5,10);
  assert.deepEqual([...encodeRequest(Opcode.SET_ANGLE_ASSIST_CONFIG,0x32,payload)],
    [1,0x1d,0x32,12,1,4,2,4,0,50,100,0,0x88,0x13,232,3]);
  const bad=payload.slice(); bad[8]=0x99; bad[9]=0x3a;
  assert.throws(()=>encodeRequest(Opcode.SET_ANGLE_ASSIST_CONFIG,1,bad));
  const phase={...config,selectedMode:AssistMode.ACTIVE_PHASE,strategyId:AssistStrategy.PHASE_PROFILE,profileId:1};
  assert.throws(()=>encodeAssistConfigWithLimitPayload(phase,5));
  assert.equal(encodeAssistConfigWithLimitPayload(phase,0.5).length,10);
});

test("5 N m readback must match INFO capability; legacy and per-strategy bounds", () => {
  const bytes=Uint8Array.from([1,0x8a,0x31,8,1,1,0x88,0x13,0,0,0x88,0x13]);
  const response=parseResponse(bytes);
  assert.equal(response.selectedMaximumNm,5);
  assert.equal(response.hardMaximumNm,5);
  assertAssistTorqueLimitCapability(response,0x3f7f);
  assert.throws(()=>assertAssistTorqueLimitCapability(response,0x1f7f));
  assert.equal(assistTorqueMaximumNm(AssistStrategy.ANGLE_THRESHOLD,0x3f7f,response.hardMaximumNm),5);
  assert.equal(assistTorqueMaximumNm(AssistStrategy.PHASE_PROFILE,0x3f7f,response.hardMaximumNm),0.5);
  assert.equal(assistTorqueMaximumNm(AssistStrategy.ANGLE_THRESHOLD,0x1f7f,response.hardMaximumNm),0.5);
  assert.equal(assistTorqueMaximumNm(AssistStrategy.ANGLE_THRESHOLD,0x3f7f,null),0.5);
  const legacy=bytes.slice(); legacy[6]=0xf4; legacy[7]=1; legacy[10]=0xf4; legacy[11]=1;
  const old=parseResponse(legacy);
  assertAssistTorqueLimitCapability(old,0x1f7f);
  assert.throws(()=>assertAssistTorqueLimitCapability(old,0x3f7f));
  const unknown=bytes.slice(); unknown[10]=0x89;
  assert.throws(()=>parseResponse(unknown));
  const above=bytes.slice(); above[6]=0x89;
  assert.throws(()=>parseResponse(above));
});

test("0.10.8 needs angle and 5 N m capabilities without blocking 0.10.7", () => {
  const identity={protocolMinVersion:1,protocolMaxVersion:1,firmware:[0,10,8],capabilities:0x3f7f,maxFrameSize:20};
  assert.equal(assessDeviceCompatibility(identity).accepted,true);
  for(const bit of [Capability.ANGLE_THRESHOLD_ASSIST,Capability.ANGLE_ASSIST_5NM])
    assert.equal(assessDeviceCompatibility({...identity,capabilities:identity.capabilities & ~bit}).accepted,false);
  assert.equal(assessDeviceCompatibility({...identity,firmware:[0,10,7],capabilities:0x1f7f}).accepted,true);
  assert.equal(Capability.ANGLE_ASSIST_5NM,1<<13);
});
