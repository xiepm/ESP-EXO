// Isolated headless browser: native IndexedDB upgrade/reload. No user browser or devices.
import assert from 'node:assert/strict';
import {readFile,writeFile,mkdir} from 'node:fs/promises';
import {existsSync} from 'node:fs';
import {resolve,join} from 'node:path';
import {createServer} from 'node:http';
import {spawn} from 'node:child_process';
import ts from '../web-bluetooth-console/node_modules/typescript/lib/typescript.js';
const root=resolve(import.meta.dirname,'..'),out=resolve(process.argv[2]??'');
assert.ok(out.startsWith(join(root,'output')+'\\')||out.startsWith(join(root,'output')+'/'),'test evidence must be inside output');
await mkdir(out,{recursive:true});const profile=join(out,'browser-profile');
await mkdir(profile); // Refuse reuse of any existing browser profile.
const web=join(root,'web-bluetooth-console');
const server=createServer(async(req,res)=>{
 try {
  const path=new URL(req.url,'http://localhost').pathname;
  if(path==='/'){res.setHeader('Content-Type','text/html');res.end('<title>USB recording storage test</title><p>Isolated storage regression. No hardware.</p>');return;}
  if(!/^\/lib\/[a-zA-Z0-9-]+\.ts$/.test(path)){res.writeHead(404).end();return;}
  const source=await readFile(join(web,path),'utf8');
  res.setHeader('Content-Type','text/javascript');res.end(ts.transpileModule(source,{compilerOptions:{target:ts.ScriptTarget.ES2022,module:ts.ModuleKind.ESNext}}).outputText);
 }catch(e){res.writeHead(500).end(String(e));}
});
await new Promise(r=>server.listen(0,'127.0.0.1',r));
const url=`http://127.0.0.1:${server.address().port}/`;
const browser=['C:/Program Files (x86)/Microsoft/Edge/Application/msedge.exe','C:/Program Files/Google/Chrome/Application/chrome.exe'].find(existsSync);
assert.ok(browser,'Chrome/Edge installed');
const child=spawn(browser,['--headless=new','--disable-gpu','--no-first-run','--no-default-browser-check',`--user-data-dir=${profile}`,'--remote-debugging-port=0','about:blank'],{windowsHide:true,stdio:'ignore'});
let ws;
try {
 let port;
 for(let i=0;i<100;i++) {
  try{port=Number((await readFile(join(profile,'DevToolsActivePort'),'utf8')).split('\n')[0]);break;}catch{await new Promise(r=>setTimeout(r,100));}
 }
 assert.ok(port,'owned browser ready');
 const targets=await (await fetch(`http://127.0.0.1:${port}/json/list`)).json();
 const target=targets.find(t=>t.type==='page'&&t.url==='about:blank');assert.ok(target);
 ws=new WebSocket(target.webSocketDebuggerUrl);const pending=new Map();let next=0;
 await new Promise((resolve,reject)=>{ws.onopen=resolve;ws.onerror=reject;});
 ws.onmessage=event=>{const m=JSON.parse(event.data),p=pending.get(m.id);if(p){pending.delete(m.id);m.error?p.reject(Error(JSON.stringify(m.error))):p.resolve(m.result);}};
 function send(method,params={}){return new Promise((resolve,reject)=>{const id=++next;const timer=setTimeout(()=>{pending.delete(id);reject(Error('timeout '+method));},15000);pending.set(id,{resolve:v=>{clearTimeout(timer);resolve(v);},reject:e=>{clearTimeout(timer);reject(e);}});ws.send(JSON.stringify({id,method,params}));});}
 async function evaluate(expression){const r=await send('Runtime.evaluate',{expression,awaitPromise:true,returnByValue:true});if(r.exceptionDetails)throw Error(JSON.stringify(r.exceptionDetails));return r.result.value;}
 await send('Page.navigate',{url});
 for(let i=0;i<50;i++){if(await evaluate('location.href=== '+JSON.stringify(url)+' && document.readyState==="complete"'))break;await new Promise(r=>setTimeout(r,100));}
 await evaluate(`(async()=>{const req=indexedDB.open('esp-exo-walking-recordings',1);await new Promise((resolve,reject)=>{
 req.onupgradeneeded=()=>{req.result.createObjectStore('sessions',{keyPath:'id'});req.result.createObjectStore('samples',{keyPath:['id','key']}).createIndex('session','id');};
 req.onerror=()=>reject(req.error);req.onsuccess=resolve;});const db=req.result;const tx=db.transaction(['sessions','samples'],'readwrite');
 tx.objectStore('sessions').put({id:'legacy',schema:1,startedAt:'2026-11-17',count:1,context:{},events:[]});
 tx.objectStore('samples').put({id:'legacy',key:'old',ordinal:0,sample:{timestampMs:100}});
 await new Promise((r,j)=>{tx.oncomplete=r;tx.onerror=j;});db.close();return true;})()`);
 const fixture=await readFile(join(web,'tests/fixtures/afo-usb-wire.txt'),'utf8');
 const initial=await evaluate(`(async()=>{const m=await import('/lib/exo-walk-recording.ts');const p=await import('/lib/exo-usb-diagnostics.ts');
 const old=await m.listRecordings();if(old[0].id!=='legacy')throw Error('old session lost');
 const recorder=new m.WalkRecorder('usb-browser',{info:{bootId:42}});
 for(const line of ${JSON.stringify(fixture)}.trim().split(/\\r?\\n/)){const f=p.parseUsbLine(line);if(f.kind==='row'||f.kind==='first'||f.kind==='config')recorder.acceptDiagnostic({type:f.kind==='row'?'usb_trace':f.kind==='config'?'usb_config':'usb_first_fault',bootId:42,hostTime:'2026-11-18T00:00:00Z',data:f.data,rawLine:line});}
 const base=recorder.diagnostics().find(r=>r.type==='usb_trace');for(let i=1000;i<11000;i++)recorder.acceptDiagnostic({...base,data:{...base.data,id:i}});
 recorder.note('usb_connected',{bootId:42});recorder.acceptDiagnostic({type:'usb_gap',bootId:42,hostTime:'2026-11-18T00:00:00Z',data:{reason:'test gap'}});
 await recorder.flush();recorder.end('test complete');await recorder.flush();return {old:old.length,newDiagnostics:recorder.meta.diagnosticCount};})()`);
 assert.equal(initial.newDiagnostics,10005);
 await send('Page.reload');await new Promise(r=>setTimeout(r,250));
 const reload=await evaluate(`(async()=>{const m=await import('/lib/exo-walk-recording.ts');const history=await m.listRecordings();
 const meta=history.find(r=>r.id==='usb-browser'),rows=await m.readRecordingDiagnostics(meta.id);
 const legacy=await m.readRecordingSamples('legacy');const empty=await m.readRecordingDiagnostics('legacy');
 const jsonl=m.recordingText(meta,[],'jsonl',rows).trim().split('\\n').map(JSON.parse);
 return {sessions:history.length,legacySamples:legacy.length,legacyDiagnostics:empty.length,diagnostics:rows.length,
 schema:meta.schema,types:[...new Set(jsonl.map(r=>r.type))],previousSource:rows.find(r=>r.type==='usb_first_fault').data.previousSourceUs,
 csvHasRawLine:m.recordingText(meta,[],'diagnosticsCsv',rows).includes('rawLine')};})()`);
 assert.deepEqual(reload,{sessions:2,legacySamples:1,legacyDiagnostics:0,diagnostics:10005,schema:2,
  types:['session','usb_config','usb_trace','usb_first_fault','usb_gap'],previousSource:4200000,csvHasRawLine:true});
 await writeFile(join(out,'result.json'),JSON.stringify({hardwareActions:[],initial,reload},null,2));
 console.log('PASS native browser IndexedDB v1->v2 upgrade, legacy preservation, USB persistence after reload, merged JSONL and diagnostic CSV');
 await send('Browser.close').catch(()=>{});
} finally {ws?.close();child.kill();server.close();}
