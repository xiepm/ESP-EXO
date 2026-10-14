import assert from "node:assert/strict";
import test from "node:test";
import {SCOPE_CHANNELS,WALKING_SCOPE_MASK,SCOPE_BUFFER_SAMPLES,scopeWindow,scopeLatest,scopeRange,
  scopePaths,scopeNearest,scopeReceptionState,scopeX,scopeY,shouldStartScope} from "../lib/exo-scope.ts";
const angle=SCOPE_CHANNELS[0];
function point(seq,time,value=10,extra={}) {return {streamSequence:1,sampleSequence:seq,timestampMs:time,
  receivedChannelMask:1,leftPositionDeg:value,rightPositionDeg:-value,activationEpochLow16:1,
  droppedSincePrevious:false,...extra};}
test("walking default includes both kinematics, phase, confidence and assist without 20Hz overload",()=>{
  assert.equal(WALKING_SCOPE_MASK,0x7b);assert.ok(SCOPE_BUFFER_SAMPLES>=30*20);
});
test("scope uses MCU timestamps, not packet ordinal, and excludes old samples",()=>{
  const p=scopeWindow([point(1,1000),point(2,1050),point(3,1250)],0.21);
  assert.deepEqual(p.map(x=>x.seconds),[-0.2,0]);
  assert.equal(scopeX(0,10),708);assert.equal(scopeX(-10,10),56);
  assert.equal(scopeY(0,[-5,5]),109);
});
test("32bit time wrap remains continuous",()=>{
  const p=scopeWindow([point(1,0xfffffff0),point(2,34)],2);
  assert.deepEqual(p.map(x=>x.seconds),[-0.05,0]);
});
test("stream restart never joins previous session, late same-stream frames do not move origin",()=>{
  const p=scopeWindow([point(50,9000),point(1,100,5,{streamSequence:2}),point(3,300,6,{streamSequence:2}),point(2,200,7,{streamSequence:2})],10);
  assert.deepEqual(p.map(x=>x.sample.sampleSequence),[1,2,3]);assert.equal(p.at(-1).seconds,0);
});
test("reconnecting or rebooting with reused stream id cannot mix previous BLE session",()=>{
  const p=scopeWindow([point(500,80000,30,{webSession:1}),point(1,100,10,{webSession:2}),point(2,200,15,{webSession:2})],30);
  assert.deepEqual(p.map(x=>x.sample.sampleSequence),[1,2]);assert.equal(p.at(-1).seconds,0);
});
test("partial-channel frame preserves latest received reading; explicit invalid clears it",()=>{
  const p=scopeWindow([point(1,100),point(2,200,null,{receivedChannelMask:2})],10);
  assert.equal(scopeLatest(p,angle).sample.leftPositionDeg,10);
  const invalid=scopeWindow([point(1,100),point(2,200,null)],10);
  assert.equal(scopeLatest(invalid,angle).sample.leftPositionDeg,null);
});
test("constant and zero data have visible finite scale; percent axes stay 0..100",()=>{
  const p=scopeWindow([point(1,100,0),point(2,200,0)],10);const r=scopeRange(p,angle,true);
  assert.ok(r[0]<0&&r[1]>0);assert.ok(Number.isFinite(scopeY(0,r)));
  assert.deepEqual(scopeRange(p,SCOPE_CHANNELS[2],true),[0,100]);
  assert.deepEqual(scopeRange(p,angle,false),[-30,60]);
});
test("missing, invalid and unselected channel frames break the line rather than bridge",()=>{
  const p=scopeWindow([point(1,100),point(2,200,null),point(3,300),point(5,500),point(6,600,null,{receivedChannelMask:2}),point(7,700)],10);
  assert.equal(scopePaths(p,angle,"left",10,[-30,60],10).length,4);
  assert.equal(scopePaths(p,SCOPE_CHANNELS[1],"left",10,[-120,120],10).length,0);
});
test("clock gap, activation epoch and dropped flag each break paths",()=>{
  for(const items of [ [point(1,100),point(2,1500)], [point(1,100),point(2,200,10,{activationEpochLow16:2})],
    [point(1,100),point(2,200,10,{droppedSincePrevious:true})] ])
    assert.equal(scopePaths(scopeWindow(items,10),angle,"left",10,[-30,60],10).length,2);
});
test("cursor reads nearest real sample and never interpolates across a long gap",()=>{
  const p=scopeWindow([point(1,100),point(2,200),point(3,9000)],10);
  assert.equal(scopeNearest(p,angle,-8.8,10).sample.sampleSequence,2);
  assert.equal(scopeNearest(p,angle,-4,10),null);
});
test("NaN and Infinity never enter an SVG path",()=>{
  const p=scopeWindow([point(1,100,NaN),point(2,200,Infinity),point(3,300,0)],10);
  const paths=scopePaths(p,angle,"left",10,scopeRange(p,angle,true),10);
  assert.equal(paths.length,1);assert.doesNotMatch(paths.join(),/NaN|Infinity/);
});
test("empty, stopped, disconnected and stale streams cannot claim LIVE",()=>{
  assert.equal(scopeReceptionState(true,true,null,5000),"waiting");
  assert.equal(scopeReceptionState(true,true,3000,5000),"stale");
  assert.equal(scopeReceptionState(true,true,4999,5000),"live");
  assert.equal(scopeReceptionState(false,true,4999,5000),"disconnected");
  assert.equal(scopeReceptionState(true,false,4999,5000),"stopped");
  assert.deepEqual(scopeWindow([],10),[]);
});
test("automatic acquisition requires a verified ready session, respects stop, and tries only once",()=>{
  const ready={livePanel:true,connected:true,ready:true,compatible:true,automatic:true,streaming:false,
    busy:false,session:2,attempted:null};
  assert.equal(shouldStartScope(ready),true);
  for(const property of ["livePanel","connected","ready","compatible","automatic"])
    assert.equal(shouldStartScope({...ready,[property]:false}),false);
  for(const change of [{streaming:null},{streaming:true},{busy:true},{attempted:2}])
    assert.equal(shouldStartScope({...ready,...change}),false);
  assert.equal(shouldStartScope({...ready,session:3,attempted:2}),true);
});
