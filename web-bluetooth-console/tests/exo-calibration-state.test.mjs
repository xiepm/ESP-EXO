import {directionPreviewDegrees} from "../lib/exo-direction-preview.ts";
import assert from 'node:assert/strict';
import test from 'node:test';
import {parseResponse} from '../lib/exo-ble-protocol.ts';
import {mappingConfirmationAfterReadback, mustReconfirmNeutral, recalibrationFeedbackIssue} from '../lib/exo-calibration-state.ts';

// Valid mapping/direction, but no online feedback/zero/joint sample (flags 0x03).
// Mirrors the protocol state in which the two existing poll handlers disagreed.
function calibration(side=0, flags=3, direction=2) {
  const frame=new Uint8Array(20); frame.set([1,0x89,7,16,side,side===0?1:0,side===0?0x20:0x10,direction,flags,0]);
  const view=new DataView(frame.buffer); view.setInt32(10,-2147483648,true); view.setInt32(14,-2147483648,true); view.setUint16(18,65535,true);
  return parseResponse(frame);
}
function joint(side=0,flags=3,direction=2) {
  const frame=new Uint8Array(17); frame.set([1,0x83,8,13,side,flags,direction]);
  new DataView(frame.buffer).setUint16(15,65535,true); return parseResponse(frame);
}

function rawReady(side) {
  // Raw feedback + mapping/direction, deliberately NO previously valid zero.
  return {...calibration(side,27,side===0?2:1),rawPositionDegrees:side===0?10:-42,feedbackAgeMs:5};
}
test('recalibration identifies both missing raw angles even though persistent directions exist',()=>{
  const issue=recalibrationFeedbackIssue(calibration(0),calibration(1));
  assert.match(issue,/左髋缺少有效角度反馈/);assert.match(issue,/右髋缺少有效角度反馈/);
});
test('a fresh raw encoder pair can be recalibrated without valid old zero flags',()=>{
  const left=rawReady(0),right=rawReady(1);
  assert.equal(left.decodedJointFlags.zeroValid,false);
  assert.equal(recalibrationFeedbackIssue(left,right),null);
});
test('a single stale side blocks calibration at the same 50ms freshness boundary',()=>{
  assert.equal(recalibrationFeedbackIssue({...rawReady(0),feedbackAgeMs:50},rawReady(1)),null);
  const issue=recalibrationFeedbackIssue(rawReady(0),{...rawReady(1),feedbackAgeMs:51});
  assert.match(issue,/右髋反馈已过期/);assert.doesNotMatch(issue,/左髋/);
});
test('missing or nonfinite raw samples, invalid age, swapped sides and invalid direction cannot pass',()=>{
  for(const value of [null,NaN,Infinity])
    assert.match(recalibrationFeedbackIssue({...rawReady(0),rawPositionDegrees:value},rawReady(1)),/左髋/);
  for(const value of [NaN,Infinity,-1,65535])
    assert.match(recalibrationFeedbackIssue({...rawReady(0),feedbackAgeMs:value},rawReady(1)),/左髋/);
  assert.match(recalibrationFeedbackIssue(null,rawReady(1)),/左髋/);
  assert.match(recalibrationFeedbackIssue(rawReady(1),rawReady(0)),/左髋.*右髋/);
  assert.match(recalibrationFeedbackIssue({...rawReady(0),direction:0},rawReady(1)),/出厂方向/);
});

test('repeated real-format calibration/offline-joint poll sequence does not toggle saved factory direction',()=>{
  for(const side of [0,1]) {
    const direction=side===0?2:1;
    const c=calibration(side,3,direction), j=joint(side,3,direction);
    assert.equal(c.rawPositionDegrees,null); assert.equal(j.decodedJointFlags.online,false);
    let confirmed=mappingConfirmationAfterReadback(false,true,c);
    assert.equal(confirmed,true);
    for(let cycle=0;cycle<40;cycle++) {
      confirmed=mappingConfirmationAfterReadback(confirmed,true,j); assert.equal(confirmed,true);
      confirmed=mappingConfirmationAfterReadback(confirmed,true,c); assert.equal(confirmed,true);
      // Invalid feedback still invalidates the user's neutral confirmation.
      assert.equal(mustReconfirmNeutral(j),true);
    }
  }
});

test('joint feedback alone never establishes factory readback after a reset/disconnect',()=>{
  for(const flags of [3,63]) assert.equal(mappingConfirmationAfterReadback(false,true,joint(0,flags)),false);
  assert.equal(mappingConfirmationAfterReadback(false,true,calibration()),true);
});

test('explicit mapping/direction invalidation immediately clears confirmation even if feedback is online',()=>{
  for(const flags of [0,1,2,61,62]) {
    for(const response of [joint(0,flags),calibration(0,flags)]) {
      assert.equal(mappingConfirmationAfterReadback(true,true,response),false);
      assert.equal(mappingConfirmationAfterReadback(true,false,response),false);
    }
    assert.equal(mustReconfirmNeutral(joint(0,flags)),true);
  }
  assert.equal(mappingConfirmationAfterReadback(true,true,joint(0,63,0)),false);
  assert.equal(mustReconfirmNeutral(joint(0,63,0)),true);
});

test('legacy manual mapping confirmation is still invalidated by feedback loss and is never auto-confirmed',()=>{
  assert.equal(mappingConfirmationAfterReadback(true,false,joint()),false);
  assert.equal(mappingConfirmationAfterReadback(true,false,calibration()),false);
  const fresh={...calibration(0,63),rawPositionDegrees:10,feedbackAgeMs:5};
  assert.equal(mappingConfirmationAfterReadback(false,false,fresh),false);
  assert.equal(mappingConfirmationAfterReadback(true,false,fresh),true);
  assert.equal(mappingConfirmationAfterReadback(false,false,joint(0,63)),false);
});

test('loss of either live feedback flag revokes neutral posture while the stored direction stays intact',()=>{
  // feedbackValid=bit3, online=bit4; mapping+direction remain valid in both cases.
  for(const flags of [63 & ~8,63 & ~16]) {
    const response=joint(0,flags);
    assert.equal(mappingConfirmationAfterReadback(true,true,response),true);
    assert.equal(mustReconfirmNeutral(response),true);
  }
  assert.equal(mustReconfirmNeutral(joint(0,63)),false);
});



test('direction preview uses controller baseline even when user moves before the first read',()=>{
  const p={kind:'directionPreview',side:1,token:9,result:0,actuator:0,baselineDegrees:-30,rawPositionDegrees:-50,remainingMs:50000,feedbackAgeMs:5};
  assert.equal(directionPreviewDegrees(p,1,9,1000,1200),-20);
  assert.equal(directionPreviewDegrees({...p,rawPositionDegrees:-60},1,9,1000,1200),-30);
  for(const patch of [{token:8},{side:0},{result:2},{remainingMs:100},{feedbackAgeMs:51},{baselineDegrees:null},{rawPositionDegrees:null}])
    assert.equal(directionPreviewDegrees({...p,...patch},1,9,1000,1200),null);
  assert.equal(directionPreviewDegrees(p,1,9,1000,1601),null);
});
test('direction preview wire roundtrip uses milliradians once and rejects malformed/old session data',()=>{
  const frame=Uint8Array.from([1,0xa8,7,16,1,9,0,0,0x18,0xfc,0xff,0xff,0x30,0xf8,0xff,0xff,0x50,0xc3,10,0]);
  const p=parseResponse(frame);
  assert.equal(p.kind,'directionPreview');assert.equal(p.token,9);
  assert.ok(Math.abs(p.baselineDegrees+180/Math.PI)<1e-10);
  assert.ok(Math.abs(directionPreviewDegrees(p,1,9,1000,1000)+180/Math.PI)<1e-10);
  for(const [offset,value] of [[4,2],[5,0],[6,14],[7,255],[18,51]]) {
    const bad=frame.slice();bad[offset]=value;assert.throws(()=>parseResponse(bad));
  }
});
