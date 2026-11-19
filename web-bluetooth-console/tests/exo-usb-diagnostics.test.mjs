import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {TRACE_FIELDS,parseUsbLine,AfoUsbReader,diagnosticsCsv} from '../lib/exo-usb-diagnostics.ts';
import {WalkRecorder,recordingText,MAX_DIAGNOSTIC_ROWS} from '../lib/exo-walk-recording.ts';
import {supportsAdaptiveBenchActive} from '../lib/exo-ble-protocol.ts';

const fixture=readFileSync(new URL('./fixtures/afo-usb-wire.txt',import.meta.url),'utf8').trim().split(/\r?\n/);
const row=(id,side,extra={})=>{
 const parts=fixture.filter(l=>l.startsWith('AFOROW,'))[side].split(',');parts[2]=String(id);
 for(const [key,value] of Object.entries(extra))parts[TRACE_FIELDS.indexOf(key)+2]=String(value);
 return parts.join(',');
};
const meta=(boot=42,oldest=473,latest=600)=>`AFOMETA,1,${boot},0.11.23,${'0'.repeat(64)},${oldest},${latest},128`;

test('production C golden decodes exact source timestamps, pre-fault sample and final command',()=>{
 const f=fixture.map(parseUsbLine);assert.equal(f[0].data.bootId,42);
 const l=f.find(x=>x.kind==='row'&&x.data.side===0).data,r=f.find(x=>x.kind==='row'&&x.data.side===1).data,first=f.find(x=>x.kind==='first').data;
 assert.equal(l.sourceUs,6000000);assert.equal(l.previousSourceUs,5990000);
 assert.equal(l.controlDtUs,10000);assert.equal(l.previousExecutionUs,123);
 assert.equal(l.finalNm,0);assert.equal(l.category,2);assert.equal(r.category,1);
 assert.equal(r.temperatureC,30);assert.equal(r.finalNm,r.suggestionNm);
 assert.equal(first.id,421);assert.equal(first.previousSourceUs,4200000);
 assert.equal(first.sourceUs,4210000);assert.equal(first.faults&1,1);
 assert.equal(parseUsbLine('debug log,not a diagnostic'),null);
 assert.equal(parseUsbLine('> '+row(600,0)).data.id,600);
});
test('damaged schema/length/integer/side rejected; nonfinite inputs stay absent, never zero',()=>{
 assert.throws(()=>parseUsbLine(row(600,0).replace('AFOROW,1','AFOROW,2')));
 assert.throws(()=>parseUsbLine(row(600,0)+',0'));
 assert.throws(()=>parseUsbLine(row(1,0,{side:2})));
 assert.throws(()=>parseUsbLine(row(1,0,{sourceUs:'9007199254740993'})));
 assert.throws(()=>parseUsbLine(row(1,0,{sourceCount:'1.5'})));
 assert.throws(()=>parseUsbLine(row(1,0,{qDeg:'bad'})));
 assert.equal(parseUsbLine(row(1,0,{qDeg:'nan',vDegS:'inf'})).data.qDeg,null);
 assert.equal(parseUsbLine(row(1,0,{qDeg:'nan',vDegS:'inf'})).data.vDegS,null);
 assert.equal(parseUsbLine(row(1,0,{flags:0,finalNm:5})).data.queuedNm,null);
});
test('actual firmware config snapshot keeps CRC and configured limits, rather than inferred defaults',()=>{
 const text=fixture.find(line=>line.startsWith('AFOCFG,'));const f=parseUsbLine(text);
 assert.equal(f.kind,'config');assert.equal(f.data.generation,2);
 assert.equal(f.data.left_minimum_angle_deg,-40);assert.equal(f.data.right_maximum_angle_deg,90);
 assert.equal(f.data.left_maximum_velocity_deg_s,400);assert.equal(f.data.left_torque_limit_nm,15);
 assert.throws(()=>parseUsbLine(text.slice(0,-1)+(text.endsWith('0')?'1':'0')),/CRC/);
});
test('firmware 0.11.23 retains strategy12 and rejects a wrong strategy or unknown release',()=>{
 const identity={firmware:[0,11,23],capabilities:0xff7f,strategyId:5,strategyVersion:12,offBodyBench:false};
 assert.equal(supportsAdaptiveBenchActive(identity),true);
 assert.equal(supportsAdaptiveBenchActive({...identity,strategyVersion:11}),false);
 assert.equal(supportsAdaptiveBenchActive({...identity,firmware:[0,11,24]}),false);
});
function fakePort(respond) {
 let stream,closed=false;const writes=[],signals=[];
 const port={
  readable:new ReadableStream({start(c){stream=c;}}),
  writable:new WritableStream({write(bytes){const text=new TextDecoder().decode(bytes);writes.push(text);
   const response=respond(text,writes.length)+'\r\n';
   // Deliberately fragmented bytes, including split CR/LF and split fields.
   for(let n=0;n<response.length;n+=23)stream.enqueue(new TextEncoder().encode(response.slice(n,n+23)));
  }}),
  getInfo:()=>({usbVendorId:0x303a,usbProductId:0x1001}),
  open:async()=>{},close:async()=>{closed=true;},setSignals:async(s)=>{signals.push(s);}
 };
 return {port,writes,signals,get closed(){return closed;}};
}
test('USB uses only bounded read commands; fragmented stream joins, boot matches, locks release',async()=>{
 let received;const done=new Promise(resolve=>received=resolve),records=[];
 const mock=fakePort((_text,n)=>n===1?fixture.join('\n'):
   [meta(42,474,601),row(601,0),row(601,1),'AFOEND,1,601,601,1'].join('\n'));
 const reader=new AfoUsbReader(mock.port,42,r=>{records.push(r);if(r.type==='usb_trace'&&r.data.side===1)received();},()=>{});
 await reader.connect();await done;await reader.checkpoint();await reader.disconnect();
 assert.deepEqual(mock.signals,[{dataTerminalReady:false,requestToSend:false}]);
 assert.ok(mock.writes.every(s=>/^afo_trace \d+\r$/.test(s)));
 assert.equal(records.filter(r=>r.type==='usb_trace').length,2);
 assert.equal(records.find(r=>r.type==='usb_trace').data.id,601);
 assert.equal(mock.closed,true);assert.equal(mock.port.readable.locked,false);assert.equal(mock.port.writable.locked,false);
});
test('USB cannot mix a different BLE boot, and ring overflow / partial pairs are explicit',async()=>{
 const wrong=fakePort(()=>[meta(99),'AFOEND,1,600,600,0'].join('\n'));
 await assert.rejects(new AfoUsbReader(wrong.port,42,()=>{},()=>{}).connect(),/启动编号/);
 assert.equal(wrong.closed,true);
 let finish;const done=new Promise(resolve=>finish=resolve),records=[];
 const mock=fakePort((_t,n)=>n===1?[meta(),'AFOEND,1,600,600,0'].join('\n'):
   [meta(42,610,611),row(610,0),row(611,0),row(611,1),'AFOEND,1,611,611,2'].join('\n'));
 const reader=new AfoUsbReader(mock.port,42,r=>{records.push(r);if(r.type==='usb_trace'&&r.data.id===611&&r.data.side===1)finish();},()=>{});
 await reader.connect();await done;await reader.checkpoint();await reader.disconnect();
 assert.ok(records.some(r=>r.type==='usb_gap'&&r.data.reason==='device_ring_overflow'&&r.data.fromId===601&&r.data.toId===609));
 assert.ok(records.some(r=>r.type==='usb_gap'&&r.data.reason==='missing_or_partial_pair'));
});
test('USB rows share recording transaction, deduplicate sticky faults, survive retry and export',async()=>{
 const saved=new Map();let fail=true,release;
 const recorder=new WalkRecorder('usb',{},async(_m,_rows,diagnostics)=>{
  if(fail)throw Error('quota');
  if(release===undefined)await new Promise(resolve=>release=resolve);
  for(const r of diagnostics)saved.set(r.key,r);
 });
 const record={type:'usb_trace',bootId:42,hostTime:'2026-11-18T00:00:00Z',data:parseUsbLine(row(1,0)).data};
 recorder.acceptDiagnostic(record);recorder.acceptDiagnostic(record);assert.equal(recorder.meta.diagnosticCount,1);
 await assert.rejects(recorder.flush(),/quota/);fail=false;
 const pending=recorder.flush();recorder.acceptDiagnostic({...record,data:parseUsbLine(row(2,1,{qDeg:'nan'})).data});
 release();await pending;await recorder.flush();assert.equal(saved.size,2);
 const jsonl=recordingText(recorder.meta,[],'jsonl',recorder.diagnostics()).trim().split('\n').map(JSON.parse);
 assert.equal(jsonl[0].schema,2);assert.equal(jsonl[2].data.qDeg,null);
 assert.equal(jsonl[1].type,'usb_trace');assert.equal(jsonl[0].units.usbDeviceTime,'us');
 assert.match(diagnosticsCsv(recorder.diagnostics()),/sourceUs/);
 recorder.end('stop');recorder.acceptDiagnostic({...record,data:parseUsbLine(row(3,0)).data});assert.equal(recorder.meta.diagnosticCount,2);
});
test('diagnostic capacity ends capture visibly without overwriting earlier evidence',()=>{
 const r=new WalkRecorder('capacity',{},async()=>{});
 for(let i=0;i<=MAX_DIAGNOSTIC_ROWS;i++)r.acceptDiagnostic({type:'usb_trace',bootId:42,hostTime:'test',data:{id:i,side:0}});
 assert.equal(r.meta.diagnosticCount,MAX_DIAGNOSTIC_ROWS);assert.ok(r.meta.endedAt);
 assert.match(r.meta.endReason,/120000/);assert.equal(r.diagnostics()[0].data.id,0);
});
