import assert from 'node:assert/strict';
import test from 'node:test';
import {assertCapturedZeroPair, captureAndVerifyBothZeros, supportsAtomicZeroCapture} from '../lib/exo-zero-calibration.ts';
import {assessDeviceCompatibility} from '../lib/exo-device-compatibility.ts';
const flags={mappingValid:true,directionValid:true,zeroValid:true,jointValid:true,feedbackValid:true,online:true};
const joint=(side)=>({side,direction:side===0?2:1,positionDegrees:0,feedbackAgeMs:10,decodedJointFlags:{...flags}});
const calibration=(side)=>({side,actuator:side===0?1:0,direction:side===0?2:1,
  rawPositionDegrees:side===0?-1.49:-6.25,softwareZeroDegrees:side===0?-1.49:-6.25,
  feedbackAgeMs:7,decodedJointFlags:{...flags}});
const assertPair=(left=joint(0),right=joint(1),lc=calibration(0),rc=calibration(1),state=1)=>
  assertCapturedZeroPair(left,right,lc,rc,state);
function transport(patch={}) {
  const calls=[];
  return {calls,
    capturePair:async()=>{calls.push('CAPTURE_PAIR');},
    queryJoint:async(side)=>{calls.push(`JOINT ${side}`); return joint(side);},
    queryCalibration:async(side)=>{calls.push(`CALIBRATION ${side}`); return calibration(side);},
    querySystem:async()=>{calls.push('SYSTEM'); return {state:1};},...patch};
}
test('0.10.11 pair capture requires known version plus calibration/saved-zero capabilities; old reuse remains compatible',()=>{
  const identity={protocolMinVersion:1,protocolMaxVersion:1,firmware:[0,10,11],capabilities:0xff7f,maxFrameSize:20};
  assert.equal(assessDeviceCompatibility(identity).accepted,true);
  assert.equal(supportsAtomicZeroCapture(identity),true);
  assert.equal(supportsAtomicZeroCapture({...identity,firmware:[0,10,12]}),true);
  assert.equal(supportsAtomicZeroCapture({...identity,firmware:[0,10,13]}),true);
  for (const firmware of [[0,10,9],[0,10,10],[0,10,14]]) assert.equal(supportsAtomicZeroCapture({...identity,firmware}),false);
  for (const bit of [8,14]) assert.equal(supportsAtomicZeroCapture({...identity,capabilities:identity.capabilities&~(1<<bit)}),false);
  assert.equal(supportsAtomicZeroCapture(null),false);
  assert.equal(assessDeviceCompatibility({...identity,firmware:[0,10,10]}).accepted,true);
  for (const bit of [14,15]) assert.equal(assessDeviceCompatibility({...identity,capabilities:identity.capabilities&~(1<<bit)}).accepted,false);
});
test('both actual human angles and raw-minus-zero readback must be near zero; valid old right20.51 is rejected',()=>{
  assert.doesNotThrow(()=>assertPair());
  assert.throws(()=>assertPair(joint(0),{...joint(1),positionDegrees:20.51}),/右髋.*未接近0/);
  assert.throws(()=>assertPair({...joint(0),positionDegrees:-20.51}),/左髋.*未接近0/);
  assert.throws(()=>assertPair(joint(0),joint(1),calibration(0),{...calibration(1),softwareZeroDegrees:-26.76}),/右髋.*未接近0/);
});
test('zero check tolerates quantization/small still movement, keeps returned real values and never rewrites displayed angle',()=>{
  const l={...joint(0),positionDegrees:0.49}, r={...joint(1),positionDegrees:-0.49};
  const lc={...calibration(0),rawPositionDegrees:-1.98},rc={...calibration(1),rawPositionDegrees:-6.74};
  assert.doesNotThrow(()=>assertPair(l,r,lc,rc)); assert.equal(l.positionDegrees,0.49); assert.equal(r.positionDegrees,-0.49);
  assert.throws(()=>assertPair({...l,positionDegrees:0.501},r,lc,rc));
});
test('any stale/offline/invalid joint or calibration readback prevents success, even with numeric zero',()=>{
  for (const side of [0,1]) {
    for (const flag of Object.keys(flags)) {
      const joints=[joint(0),joint(1)], calibrations=[calibration(0),calibration(1)];
      joints[side].decodedJointFlags[flag]=false;
      assert.throws(()=>assertPair(...joints,...calibrations));
      joints[side]=joint(side); calibrations[side].decodedJointFlags[flag]=false;
      assert.throws(()=>assertPair(...joints,...calibrations));
    }
    for (const age of [51,65535,NaN,-1]) {
      const joints=[joint(0),joint(1)], calibrations=[calibration(0),calibration(1)];
      joints[side].feedbackAgeMs=age; assert.throws(()=>assertPair(...joints,...calibrations));
      joints[side]=joint(side); calibrations[side].feedbackAgeMs=age; assert.throws(()=>assertPair(...joints,...calibrations));
    }
  }
});
test('missing/nonfinite positions, wrong side/sign/mapping and non-IDLE state do not pass',()=>{
  for (const value of [NaN,Infinity,-Infinity]) assert.throws(()=>assertPair({...joint(0),positionDegrees:value}));
  for (const value of [null,NaN,Infinity]) for(const field of ['rawPositionDegrees','softwareZeroDegrees'])
    assert.throws(()=>assertPair(joint(0),joint(1),{...calibration(0),[field]:value}));
  for (const patch of [{side:0},{direction:2}]) assert.throws(()=>assertPair(joint(0),{...joint(1),...patch}));
  for (const patch of [{actuator:1},{actuator:255},{side:0},{direction:0}])
    assert.throws(()=>assertPair(joint(0),joint(1),calibration(0),{...calibration(1),...patch}));
  for (const state of [0,2,3,4,5]) assert.throws(()=>assertPair(joint(0),joint(1),calibration(0),calibration(1),state));
});
test('calibration workflow captures ONCE then reads both angles, both offsets and final state in BLE queue order',async()=>{
  const t=transport(); await captureAndVerifyBothZeros(t);
  assert.deepEqual(t.calls,['CAPTURE_PAIR','JOINT 0','JOINT 1','CALIBRATION 0','CALIBRATION 1','SYSTEM']);
});
test('rejected/uncertain capture is not retried or treated as successful using already-valid old zeros',async()=>{
  const t=transport(); t.capturePair=async()=>{t.calls.push('CAPTURE_PAIR'); throw Error('ACK timeout');};
  await assert.rejects(()=>captureAndVerifyBothZeros(t),/ACK timeout/);
  assert.deepEqual(t.calls,['CAPTURE_PAIR']);
});
test('post-capture right20.51 refuses complete workflow; no auto repeat',async()=>{
  const t=transport(); t.queryJoint=async(side)=>{t.calls.push(`JOINT ${side}`); return {...joint(side),positionDegrees:side===1?20.51:0};};
  await assert.rejects(()=>captureAndVerifyBothZeros(t),/右髋.*未接近0/);
  assert.equal(t.calls.filter(c=>c==='CAPTURE_PAIR').length,1);
});
test('partial readback/STOP preemption propagates failure and never sends another zero action',async()=>{
  const t=transport(); t.queryJoint=async(side)=>{t.calls.push(`JOINT ${side}`); if(side===1) throw Error('STOP preemption'); return joint(side);};
  await assert.rejects(()=>captureAndVerifyBothZeros(t),/STOP preemption/);
  assert.deepEqual(t.calls,['CAPTURE_PAIR','JOINT 0','JOINT 1']);
});
