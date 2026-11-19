import assert from 'node:assert/strict';
import test from 'node:test';
import {ADAPTIVE_COLUMNS} from '../lib/exo-adaptive.ts';
import {AdaptiveCaptureSession, CaptureReview, CaptureResetGuard, captureCommandBytes, parseLockRecords,
  captureInputIssue, usableAdaptiveInput} from '../lib/exo-adaptive-capture.ts';

function adapt(t, side, patch = {}) {
  const row = Object.fromEntries(ADAPTIVE_COLUMNS.map(k => [k, 0]));
  Object.assign(row, {timestamp_us:t, side, system_state:1, generation:1, feedback_valid:1, dropped:100}, patch);
  return 'ADAPT,' + ADAPTIVE_COLUMNS.map(k => row[k]).join(',') + '\r\n';
}
function lock(t, side, {kind='ADLOCKSTATE', sequence=10, dropped=0, ...patch}={}) {
  const values = [t,sequence,side,1,0,1,64,0.1,0.8,0.01,0.01,3,1,10,0,0,0,0,0,dropped];
  for (const [index, value] of Object.entries(patch)) values[Number(index)] = value;
  return kind + ',' + values.join(',') + '\r\n';
}
function block(t, {boot='0x1E2F7333', extra='', state='IDLE', output='no', startBlock='NONE', patch={}, rightPatch={}}={}) {
  return extra + `State: ${state}\r\nStart block: ${startBlock}\r\nRun requested: no\r\nOutput allowed: ${output}\r\nBoot ID: ${boot}\r\n` +
    adapt(t,0,patch) + adapt(t,1,rightPatch) + lock(t,0) + lock(t,1) + 'ADAPT max_execution_us=1500 ACTIVE_DEPLOYMENT=OFF_BODY_BENCH_CANDIDATE active_limit_mnm=15000\r\n';
}
const invalidBlock = t => block(t,{startBlock:'LEFT_ZERO_INVALID',patch:{feedback_valid:0,faults:25},rightPatch:{feedback_valid:0,faults:25}});

test('strict diagnostic command allowlist rejects multi-command injection and all motion/configuration',()=>{
  assert.equal(new TextDecoder().decode(captureCommandBytes(['adaptive_dump','adaptive_faults'])), 'adaptive_dump\radaptive_faults\r');
  for (const command of ['start','stop','enable','zero','adaptive_set left enabled 1','system_status\rstart','adaptive_dump\n','motor A']) {
    assert.throws(()=>captureCommandBytes([command]));
  }
  assert.throws(()=>captureCommandBytes([]));
});

test('fresh paired boundary excludes old running/torque rows but preserves distinct exact loss events',()=>{
  const review = new CaptureReview(); review.accept(block(1000));
  review.accept(block(2000, {extra: adapt(800,0,{system_state:3,command_valid:1,final_ff_command_nm:1}) +
    adapt(1500,0) + adapt(1500,1) + lock(900,0,{kind:'ADLOCK'}) + lock(1600,0,{kind:'ADLOCK',sequence:20,dropped:2})}));
  assert.equal(review.rows.size,6); assert.equal(review.historicalRows,1);
  assert.equal(review.events.size,1); assert.equal(review.historicalEvents,1);
  assert.deepEqual(review.eventDroppedRange,[2,2]);
  assert.equal(review.boundary,1000);
});

test('boot, current time, state, output, configuration and malformed diagnostics fail closed',()=>{
  for (const mutation of [
    block(2000,{boot:'0x12345678'}),block(1000),block(999),block(2000,{state:'RUNNING'}),
    block(2000,{output:'YES'}),block(2000,{patch:{command_valid:1}}),block(2000,{patch:{final_ff_command_nm:.01}}),
    block(2000,{patch:{generation:2}}),block(2000,{patch:{q_deg:NaN}}),block(2000).replace('ADLOCKSTATE,2000,10,1','ADLOCKSTATE,2000,10,0'),
    'Brownout detector was triggered\n'+block(2000),block(2000).replace(/Boot ID:[^\r]+/,''),
    block(2000,{extra:lock(1500,0,{kind:'ADLOCK',18:1})})
  ]) {
    const review = new CaptureReview(); review.accept(block(1000));
    assert.throws(()=>review.accept(mutation));
  }
  assert.throws(()=>parseLockRecords('ADLOCK,1,2,3\n'));
});

test('snapshot/drain overlap is deduplicated; changed payload at same timestamp is rejected',()=>{
  const review = new CaptureReview(); review.accept(block(1000));
  review.accept(block(2000,{extra:adapt(1000,0,{dropped:103}) + adapt(1000,1,{dropped:103})}));
  assert.equal(review.rows.size,4); assert.deepEqual(review.droppedRange,[100,103]);
  assert.throws(()=>review.accept(block(3000,{extra:adapt(1000,0,{q_deg:50})})),/冲突/);
});
test('explicit running observation retains commands while old IDLE collector still rejects them',()=>{
 const running=(t,generation=1)=>`State: RUNNING\nRun requested: yes\nOutput allowed: yes\nBoot ID: 0x1E2F7333\n`+
   adapt(t,0,{system_state:3,command_valid:1,final_ff_command_nm:.25,generation})+
   adapt(t,1,{system_state:3,command_valid:1,final_ff_command_nm:-.25,generation})+
   lock(t,0,{3:3,5:generation,17:.25,18:1})+lock(t,1,{3:3,5:generation,17:-.25,18:1});
 const review=new CaptureReview(true);review.accept(block(1000));review.accept(running(2000));
 assert.equal(review.latest[0].final_ff_command_nm,.25);
 assert.equal(review.latest[1].command_valid,1);
 const idle=new CaptureReview();idle.accept(block(1000));assert.throws(()=>idle.accept(running(2000)));
 assert.throws(()=>review.accept(running(3000).replace('0x1E2F7333','0x12345678')),/Boot/);
 assert.throws(()=>review.accept(running(3000).replaceAll('0.25','5.01')),/5 N·m/);
 assert.throws(()=>review.accept(running(3000,2)),/代次/);
});

test('real frozen 0.11.5 snapshot is accepted without treating phase/fault diagnostics as motor output',()=>{
  // Four unchanged lines from worn_zero_output_lock_repeat_20261104_2138.log.
  // Inline fixture keeps this test independent of ignored local output directories.
  const rows = [
    'ADAPT,3463536083,0,1,-13.82496,-5.27006,0.00026,1,0.448519,0.623008,0.34986,1.57985,0.0000,0,0,0.00000,0.00000,0.00000,0.00000,0.00000,0,24,0,1,1048,33927,3431392922,0.623008,0.623008,1.096349,14,765,0.0000,0,2030',
    'ADAPT,3463536083,1,1,-11.51718,0.26927,-0.00026,1,0.134053,0.123406,0.39994,1.26103,0.0000,0,0,0.00000,0.00000,0.00000,0.00000,0.00000,0,24,0,1,1048,33927,3428219435,0.123406,0.123406,-0.066893,15,408,0.0000,0,2030',
    'ADLOCKSTATE,3463536083,346327,0,1,0,1,56,0.000946,0.000000,0.447266,32.143162,0,-13.82496,-5.27006,0,0,24,0.00000,0,0',
    'ADLOCKSTATE,3463536083,346327,1,1,0,1,56,0.000589,0.000000,0.495665,35.316647,0,-11.51718,0.26927,0,0,24,0.00000,0,0',
  ];
  const review = new CaptureReview();
  review.accept('State: IDLE\nRun requested: no\nOutput allowed: no\nBoot ID: 0x5361BC22\n'+rows.join('\n'));
  assert.equal(review.rows.size,2); assert.equal(review.latest[0].q_deg,-13.82496);
});

function fakePort(responses, {disconnect=false, vendor=0x1a86, initial=''}={}) {
  const writes=[]; const received=[]; let controller; let closed=false; let opened=false; let opens=0;
  function emit(response) {
    if (!response) return;
    const bytes=new TextEncoder().encode(response); received.push(bytes);
    // Arbitrary split boundaries, including CR/LF and protocol tokens.
    for(let i=0;i<bytes.length;i+=17) controller.enqueue(bytes.slice(i,i+17));
  }
  const port = {
    readable: new ReadableStream({start(c){controller=c;}}),
    writable: new WritableStream({write(bytes){
      writes.push(new TextDecoder().decode(bytes));
      const response=responses.shift();
      emit(response);
      if(disconnect) controller.error(new Error('simulated disconnect'));
    }}),
    getInfo(){return {usbVendorId:vendor,usbProductId:0x55d3};},
    async open(){opened=true; opens++; emit(initial);}, async setSignals(signals){assert.deepEqual(signals,{dataTerminalReady:false,requestToSend:false});},
    async close(){closed=true;},
  };
  return {port,writes,received,emit,get closed(){return closed;},get opened(){return opened;},get opens(){return opens;}};
}

test('manual stop drains, reads final state, releases port and exports byte-identical raw evidence',async()=>{
  const fake = fakePort([block(1000),block(2000),block(3000)]);
  let stopped=false;
  const session = new AdaptiveCaptureSession(()=>{
    if(session.phase==='collecting' && !stopped){stopped=true; session.mark('人工动作'); session.stop();}
  });
  await session.run(fake.port);
  assert.equal(session.complete,true); assert.equal(session.review.lastSnapshot,3000); assert.ok(fake.closed);
  assert.equal(fake.writes.length,3);
  assert.equal(session.review.boundary,2000);
  for(const write of fake.writes) for(const command of write.split('\r').filter(Boolean)) assert.doesNotThrow(()=>captureCommandBytes([command]));
  assert.deepEqual(Buffer.concat(session.chunks),Buffer.concat(fake.received));
  assert.equal(session.manifest().summary.boot,'0X1E2F7333');
  assert.ok(session.markers.some(m=>m.label==='操作者结束采集'));
});

test('changed boot and split brownout terminate without reconnect or motion writes; evidence remains exportable',async()=>{
  for (const response of [block(3000,{boot:'0xABCDEF12'}),'Brownout detector was triggered\r\n']) {
    const fake=fakePort([block(1000),block(2000),response]); const session=new AdaptiveCaptureSession();
    await session.run(fake.port);
    assert.equal(session.complete,false); assert.ok(fake.closed); assert.equal(fake.writes.length,3);
    assert.ok(session.bytes>0); assert.ok(session.ended); assert.equal(session.phase,'finished');
  }
});

test('connection-time boot/brownout text is retained; only two fresh consistent readbacks establish the take',async()=>{
  const initial='ESP-ROM:esp32c3-api1-20210207\r\nrst:0x15 (USB_UART_CHIP_RESET)\r\nI BOD: Brownout detector was triggered\r\nBoot ID: 0x5361BC22\r\n';
  const fake=fakePort([block(1000),block(2000),block(3000)],{initial});
  const session=new AdaptiveCaptureSession(()=>{if(session.phase==='collecting')session.stop();});
  await session.run(fake.port);
  assert.equal(session.complete,true); assert.equal(session.review.boundary,2000);
  assert.equal(session.review.boot,'0X1E2F7333');
  assert.equal(session.review.rows.size,4);
  assert.equal(session.resetGuard.initialCount,3);
  assert.equal(session.initialBytes,Buffer.byteLength(initial));
  assert.ok(session.readbackStarted); assert.ok(session.baselineConfirmed);
  assert.deepEqual(Buffer.concat(session.chunks),Buffer.concat(fake.received));
  assert.equal(session.manifest().initial_connection.signatures.brownout,1);
  assert.equal(fake.writes[0],'system_status\rble_status\radaptive_status\r');
  assert.equal(fake.writes[1],fake.writes[0]);
});

test('old detector tail cannot become a fresh reset when the readback guard is armed',()=>{
  const guard=new CaptureResetGuard();
  guard.receive('ESP-'); guard.receive('ROM:boot\r\nBrown'); guard.receive('out detector was triggered\r\n');
  assert.equal(guard.initialCount,2);
  guard.arm(); guard.receive('State: IDLE\r\n'); guard.receive('Boot ID: 0x1E2F7333\r\n');
  assert.equal(guard.initialCount,2);
  assert.throws(()=>guard.receive('ESP-ROM:new boot\r\n'),/状态核对开始后/);
});

test('new reset token split across receive chunks is caught after arming',()=>{
  for(const token of ['ESP-ROM:', 'rst:0x15', 'Brownout', 'Guru Meditation', 'Rebooting', 'panic']) {
    const guard=new CaptureResetGuard(); guard.receive('Previous startup finished\r\n'); guard.arm();
    const cut=Math.floor(token.length/2); guard.receive(token.slice(0,cut));
    assert.throws(()=>guard.receive(token.slice(cut)+'\r\n'),/状态核对开始后/);
  }
});

test('startup evidence does not excuse a new boot, reset, stale time or non-IDLE during baseline confirmation',async()=>{
  for (const response of [block(2000,{boot:'0xABCDEF12'}),block(1000),block(2000,{state:'RUNNING'}),
    'ESP-ROM:new boot during confirmation\r\n']) {
    const fake=fakePort([block(1000),response],{initial:'ESP-ROM:initial boot\r\n'});
    const session=new AdaptiveCaptureSession(); await session.run(fake.port);
    assert.equal(session.complete,false); assert.equal(session.review.boot,null);
    assert.equal(session.review.rows.size,0); assert.equal(session.baselineConfirmed,null);
    assert.equal(fake.writes.length,2); assert.ok(fake.closed);
  }
});

test('startup log alone cannot authorize capture or cause diagnostic polling after manual cancellation',async()=>{
  const fake=fakePort([],{initial:'ESP-ROM:initial boot\r\n'});
  const session=new AdaptiveCaptureSession(); session.stop(); await session.run(fake.port);
  assert.equal(session.complete,false); assert.equal(session.review.rows.size,0);
  assert.equal(fake.writes.length,0); assert.ok(fake.closed);
});

test('USB disappearance aborts and wrong VID is rejected before opening',async()=>{
  const fake=fakePort([],{disconnect:true}); const session=new AdaptiveCaptureSession();
  await session.run(fake.port); assert.equal(session.complete,false); assert.ok(fake.closed);
  const wrong=fakePort([],{vendor:0xffff}); const other=new AdaptiveCaptureSession();
  await other.run(wrong.port); assert.equal(wrong.opened,false); assert.equal(wrong.writes.length,0);
});

test('zero-invalid and input-invalid snapshots cannot authorize motion data collection; valid unlocked input can',()=>{
  for(const raw of [invalidBlock(1000),block(1000,{rightPatch:{feedback_valid:0}}),
    block(1000,{patch:{faults:1}}),block(1000,{startBlock:'RIGHT_ZERO_INVALID'})]) {
    const review=new CaptureReview(); review.accept(raw); assert.ok(captureInputIssue(raw,review));
  }
  const raw=block(1000), review=new CaptureReview(); review.accept(raw);
  assert.equal(review.latest[0].locked,0); assert.equal(captureInputIssue(raw,review),null);
  assert.equal(usableAdaptiveInput(null),false);
  const bad=new CaptureReview(); bad.accept(invalidBlock(1000));
  assert.equal(bad.summary().unique_rows,2); assert.equal(bad.summary().valid_input_rows,0);
  assert.equal(bad.summary().invalid_input_rows,2);
});

test('invalid startup waits on the same open USB; only manual recheck can begin after two usable snapshots',async()=>{
  const fake=fakePort([invalidBlock(1000),invalidBlock(2000),block(3000),block(4000),block(5000)]);
  let waitingCount=0, writesWhileWaiting, markedWhileWaiting, rowsWhileWaiting, closedWhileWaiting;
  const session=new AdaptiveCaptureSession(()=>{
    if(session.phase==='waiting') {
      waitingCount++; rowsWhileWaiting=session.review.rows.size; closedWhileWaiting=fake.closed;
      session.mark('must not be recorded'); markedWhileWaiting=session.markers.length;
      setTimeout(()=>{writesWhileWaiting=fake.writes.length; session.recheck();},1100);
    }
    if(session.phase==='collecting') session.stop();
  });
  await session.run(fake.port);
  assert.equal(session.complete,true); assert.equal(waitingCount,1); assert.equal(writesWhileWaiting,2);
  assert.equal(rowsWhileWaiting,0); assert.equal(markedWhileWaiting,0); assert.equal(closedWhileWaiting,false);
  assert.equal(fake.opens,1); assert.equal(fake.writes.length,5); assert.ok(fake.closed);
  assert.equal(session.review.boundary,4000); assert.equal(session.review.summary().valid_input_rows,4);
  assert.equal(session.readinessAttempts.length,2); assert.match(session.readinessAttempts[0].issue,/LEFT_ZERO_INVALID/);
  assert.equal(session.readinessAttempts[1].issue,null);
  for(const write of fake.writes) for(const command of write.split('\r').filter(Boolean)) assert.doesNotThrow(()=>captureCommandBytes([command]));
});

test('manual stop during input preparation closes USB and preserves diagnostic bytes without a capture baseline',async()=>{
  const fake=fakePort([invalidBlock(1000),invalidBlock(2000)]);
  const session=new AdaptiveCaptureSession(()=>{if(session.phase==='waiting')session.stop();});
  await session.run(fake.port);
  assert.equal(session.complete,false); assert.equal(session.review.rows.size,0); assert.equal(session.baselineConfirmed,null);
  assert.equal(session.markers.length,0); assert.equal(fake.writes.length,2); assert.ok(fake.closed);
  assert.deepEqual(Buffer.concat(session.chunks),Buffer.concat(fake.received));
});

test('a reset while waiting remains fatal, and manual recheck cannot silently switch to another Boot',async()=>{
  for(const mode of ['reset','changed-boot']) {
    const fake=fakePort([invalidBlock(1000),invalidBlock(2000),block(3000,{boot:'0xDEADBEEF'})]);
    const session=new AdaptiveCaptureSession(()=>{
      if(session.phase==='waiting') {
        if(mode==='reset')fake.emit('ESP-ROM:new boot while waiting\r\n'); else session.recheck();
      }
    });
    await session.run(fake.port);
    assert.equal(session.complete,false); assert.equal(session.baselineConfirmed,null); assert.ok(fake.closed);
    assert.equal(fake.opens,1); assert.equal(fake.writes.length,mode==='reset'?2:3);
  }
});

test('loss of valid input during capture terminates, and invalid records are retained without a successful outcome',async()=>{
  const fake=fakePort([block(1000),block(2000),invalidBlock(3000)]);
  const session=new AdaptiveCaptureSession(); await session.run(fake.port);
  assert.equal(session.complete,false); assert.match(session.outcome,/输入失效/);
  assert.equal(session.review.summary().valid_input_rows,2); assert.equal(session.review.summary().invalid_input_rows,2);
  assert.equal(fake.writes.length,3); assert.ok(fake.closed);
});
