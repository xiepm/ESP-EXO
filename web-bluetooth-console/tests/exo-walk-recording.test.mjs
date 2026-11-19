import test from 'node:test';
import assert from 'node:assert/strict';
import {WalkRecorder,recordingText,MAX_RECORDING_SAMPLES} from '../lib/exo-walk-recording.ts';

const sample=(n,extra={})=>({webSession:1,streamSequence:2,sampleSequence:n,timestampMs:n*100,
 receivedIsoTime:'2026-11-17T00:00:00.000Z',receivedChannelMask:1,outputAllowed:false,
 realOutputEnabled:false,systemState:1,droppedSincePrevious:false,activationEpochLow16:0,
 leftPositionDeg:12,rightPositionDeg:null,leftVelocityDegS:null,rightVelocityDegS:null,
 leftMotorTorqueNm:null,rightMotorTorqueNm:null,leftPhasePercent:null,rightPhasePercent:null,
 leftAssistSuggestionNm:null,rightAssistSuggestionNm:null,leftConfidencePercent:null,
 rightConfidencePercent:null,leftAppliedTorqueNm:null,rightAppliedTorqueNm:null,...extra});
const store=()=>{
 const rows=new Map();let meta;
 return {rows,get meta(){return meta;},write:async(m,batch)=>{meta=structuredClone(m);for(const r of batch)rows.set(r.key,structuredClone(r));}};
};
test('CSV keeps as-of asynchronous AFO context and never backfills a future fault',()=>{
 const meta={startedAt:'2026-11-17T00:00:00.000Z',context:{info:{firmware:[0,11,20]},
   assistConfig:{strategyName:'ADAPTIVE_GAUSSIAN',leftLevelPercent:100,rightLevelPercent:100,
     unavailableReasonName:'AFO_ACQUIRING'},assistTorqueLimit:{selectedMaximumNm:15}},events:[
   {type:'device_state',hostTime:'2026-11-17T00:00:02.000Z',value:{
     assistConfig:{unavailableReasonName:'AFO_REARM_REQUIRED'},afoStatuses:[
       {faults:44,firstPresent:true,firstFaults:44,firstTimestampMs:14610},null]}}]};
 const csv=recordingText(meta,[sample(1,{receivedIsoTime:'2026-11-17T00:00:01.000Z'}),
   sample(2,{receivedIsoTime:'2026-11-17T00:00:02.000Z'})],'csv');
 const rows=csv.trim().split('\r\n').map(line=>line.split(',').map(c=>c.replace(/^"|"$/g,'')));
 const val=(row,key)=>rows[row][rows[0].indexOf(key)];
 assert.equal(val(1,'firmware'),'0.11.20');assert.equal(val(1,'maximumNm'),'15');
 assert.equal(val(1,'assistUnavailableReason'),'AFO_ACQUIRING');
 assert.equal(val(1,'leftAfoFirstFaults'),'');assert.equal(val(1,'rightAfoFaults'),'');
 assert.equal(val(2,'leftAfoFirstFaults'),'44');assert.equal(val(2,'leftAfoFirstTimestampMs'),'14610');
 assert.equal(val(2,'contextReadHostTime'),'2026-11-17T00:00:02.000Z');
 assert.equal(rows[0].length,rows[1].length);assert.equal(rows[0].length,rows[2].length);
 assert.equal(recordingText({...meta,context:{},events:[]},[],'csv').trim().split('\n').length,1);
});
test('multi-channel updates merge across saves, null channels remain absent, and metadata is immutable',async()=>{
 const db=store(),context={firmware:'0.11.19',limitNm:15},r=new WalkRecorder('one',context,db.write);
 context.limitNm=0;
 r.accept(sample(1));await r.flush();
 r.accept(sample(1,{receivedChannelMask:3,leftVelocityDegS:28}));await r.flush();
 assert.equal(db.rows.size,1);assert.equal(db.meta.count,1);assert.equal(db.meta.context.limitNm,15);
 assert.equal([...db.rows.values()][0].sample.leftVelocityDegS,28);
 assert.equal([...db.rows.values()][0].sample.rightPositionDeg,null);
 const lines=recordingText(r.meta,r.samples(),'jsonl').trim().split('\n').map(JSON.parse);
 assert.equal(lines[0].type,'session');assert.equal(lines[1].rightPositionDeg,null);
 assert.ok(recordingText(r.meta,r.samples(),'csv').includes('leftPositionDeg'));
});
test('packet arriving during pending write is saved next, failure preserves RAM and supports retry',async()=>{
 const db=store();let release;
 const r=new WalkRecorder('race',{},async(m,rows)=>{await new Promise(resolve=>release=resolve);await db.write(m,rows);});
 r.accept(sample(1));const save=r.flush();
 r.accept(sample(1,{leftAppliedTorqueNm:2,receivedChannelMask:65}));release();await save;
 const again=r.flush();release();await again;
 assert.equal([...db.rows.values()][0].sample.leftAppliedTorqueNm,2);
 let fail=true;const retry=new WalkRecorder('retry',{},async(m,rows)=>{if(fail)throw new Error('quota');await db.write(m,rows);});
 retry.accept(sample(2));await assert.rejects(retry.flush(),/quota/);assert.equal(retry.samples().length,1);
 fail=false;await retry.flush();assert.equal(db.meta.id,'retry');assert.equal(db.meta.count,1);
});
test('recording outlives rolling 10000-sample chart; stream restart and reconnect cannot collide',async()=>{
 const db=store(),r=new WalkRecorder('long',{},db.write);
 for(let n=0;n<10010;n++)r.accept(sample(n));
 r.accept(sample(0,{webSession:2}));r.accept(sample(0,{streamSequence:3}));
 assert.equal(r.meta.count,10012);
 r.note('state',{unavailableReason:'AFO_REARM_REQUIRED'});r.end('disconnect');
 r.accept(sample(10011));await r.flush();
 assert.equal(db.meta.count,10012);assert.equal(db.meta.endReason,'disconnect');assert.equal(db.rows.size,10012);
 assert.equal(db.meta.events[0].value.unavailableReason,'AFO_REARM_REQUIRED');
});
test('segment limit never silently overwrites beginning or claims a continuing capture',()=>{
 const r=new WalkRecorder('bound',{},async()=>{});
 for(let n=0;n<=MAX_RECORDING_SAMPLES;n++)r.accept(sample(n));
 assert.equal(r.meta.count,MAX_RECORDING_SAMPLES);assert.ok(r.meta.endedAt);
 assert.equal(r.samples()[0].sampleSequence,0);assert.match(r.meta.endReason,/72000/);
});
