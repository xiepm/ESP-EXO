// Isolated local browser verification. Never attach to the operator's browser.
import assert from 'node:assert/strict';
import { readFile, writeFile } from 'node:fs/promises';
import { join } from 'node:path';
const directory=process.argv[2], input=process.argv[3];
assert.ok(directory&&input);
const port=Number((await readFile(join(directory,'browser-profile','DevToolsActivePort'),'utf8')).split('\n')[0]);
const targets=await (await fetch(`http://127.0.0.1:${port}/json/list`)).json();
const target=targets.find(t=>t.type==='page'&&t.url==='about:blank');
assert.ok(target,'only a newly owned blank target may be used');
const ws=new WebSocket(target.webSocketDebuggerUrl), pending=new Map(), errors=[];
let sequence=0;
await new Promise((resolve,reject)=>{ws.onopen=resolve;ws.onerror=reject;});
ws.onmessage=event=>{
  const message=JSON.parse(event.data);
  if(message.method==='Runtime.exceptionThrown') errors.push(message.params.exceptionDetails);
  const item=pending.get(message.id);
  if(item){pending.delete(message.id);message.error?item.reject(new Error(JSON.stringify(message.error))):item.resolve(message.result);}
};
function send(method,params={}) {
  return new Promise((resolve,reject)=>{
    const id=++sequence;
    const timer=setTimeout(()=>{pending.delete(id);reject(new Error('CDP timeout: '+method));},15000);
    pending.set(id,{resolve:r=>{clearTimeout(timer);resolve(r);},reject:e=>{clearTimeout(timer);reject(e);}});
    ws.send(JSON.stringify({id,method,params}));
  });
}
async function evaluate(expression) {
  const result=await send('Runtime.evaluate',{expression,returnByValue:true,awaitPromise:true});
  if(result.exceptionDetails) throw new Error(JSON.stringify(result.exceptionDetails));
  return result.result.value;
}
async function until(expression) {
  for(let i=0;i<100;i++) {if(await evaluate(expression))return;await new Promise(r=>setTimeout(r,100));}
  throw new Error('condition timed out: '+expression);
}
const chartSelector='section[aria-label="AFO同时间轴方向对照图"]';
try {
  await send('Page.enable');await send('Runtime.enable');
  await send('Emulation.setDeviceMetricsOverride',{width:1400,height:1100,deviceScaleFactor:1,mobile:false});
  await send('Page.addScriptToEvaluateOnNewDocument',{source:`window.hardwareRequests=[];
    Object.defineProperty(navigator,'serial',{value:{requestPort(){window.hardwareRequests.push('serial');throw Error('Hardware blocked in UI test');}}});
    Object.defineProperty(navigator,'bluetooth',{value:{requestDevice(){window.hardwareRequests.push('bluetooth');throw Error('Hardware blocked in UI test');}}});`});
  await send('Page.navigate',{url:'http://127.0.0.1:4173/exo'});
  await until('!!document.querySelector("#exo-nav-motion")');
  await new Promise(r=>setTimeout(r,800));
  await evaluate('document.querySelector("#exo-nav-motion").click()');
  await until('!document.querySelector("section[aria-labelledby=exo-nav-motion]").hidden');
  const doc=await send('DOM.getDocument');
  const node=await send('DOM.querySelector',{nodeId:doc.root.nodeId,selector:'input[aria-label="导入AFO日志"]'});
  await send('DOM.setFileInputFiles',{nodeId:node.nodeId,files:[input]});
  await until(`document.querySelectorAll(${JSON.stringify(chartSelector)}).length===2`);
  // Select the imported historical chart. Its sibling USB chart remains unconnected.
  const root=`document.querySelectorAll(${JSON.stringify(chartSelector)})[1]`;
  await evaluate(`{const root=${root};const select=root.querySelectorAll('select')[1];select.value='5';select.dispatchEvent(new Event('change',{bubbles:true}));}`);
  await evaluate(`{const slider=${root}.querySelector('input[type=range]');Object.getOwnPropertyDescriptor(HTMLInputElement.prototype,'value').set.call(slider,'125500000');slider.dispatchEvent(new Event('input',{bubbles:true}));}`);
  // A range value is snapped relative to its MCU-time minimum and 10 ms step.
  await until(`Math.abs(Number(${root}.querySelector('input[type=range]').value)-125500000)<=10000`);
  // Put the common cursor on the known left-side opposition sample near 124.907 s.
  await evaluate(`{const root=${root};const end=Number(root.querySelector('input[type=range]').value);const svg=root.querySelector('svg');const b=svg.getBoundingClientRect();svg.dispatchEvent(new PointerEvent('pointermove',{bubbles:true,clientX:b.left+b.width*(66+690*(124907220-(end-5000000))/5000000)/780,clientY:b.top+50}));}`);
  await until(`${root}.querySelector('.exo-direction-detail').textContent.includes('124.907220')`);
  const detail5=await evaluate(`${root}.querySelector('.exo-direction-detail').textContent`);
  assert.ok(detail5.includes('-3.8673'),detail5);
  assert.ok(detail5.includes('命令无效'),detail5);
  assert.ok(detail5.includes('角度割线支持'),detail5);
  await evaluate(`{const v=${root}.querySelector('input[type=number]');Object.getOwnPropertyDescriptor(HTMLInputElement.prototype,'value').set.call(v,'.1');v.dispatchEvent(new Event('input',{bubbles:true}));}`);
  await until(`${root}.querySelector('.exo-direction-detail').textContent.includes('-0.0773')`);
  const detail01=await evaluate(`${root}.querySelector('.exo-direction-detail').textContent`);
  await evaluate(`{const v=${root}.querySelector('input[type=number]');Object.getOwnPropertyDescriptor(HTMLInputElement.prototype,'value').set.call(v,'5');v.dispatchEvent(new Event('input',{bubbles:true}));}`);
  await until(`${root}.querySelector('.exo-direction-detail').textContent.includes('-3.8673')`);
  const clip=await evaluate(`{const r=${root}.getBoundingClientRect();({x:r.left+scrollX,y:r.top+scrollY,width:r.width,height:r.height,scale:1});}`);
  const shot=await send('Page.captureScreenshot',{format:'png',clip,captureBeyondViewport:true});
  await writeFile(join(directory,'historical_direction_chart.png'),Buffer.from(shot.data,'base64'),{flag:'wx'});
  await evaluate(`[...document.querySelectorAll('button')].find(b=>b.textContent==='在实时数据中查看此历史图').click()`);
  await until('!document.querySelector("section[aria-labelledby=exo-nav-live]").hidden');
  const liveHistory=await evaluate('document.querySelector("section[aria-labelledby=exo-nav-live] section[aria-label=\\"AFO同时间轴方向对照图\\"]").textContent');
  assert.ok(liveHistory.includes('历史文件 · 非当前状态'));
  assert.ok(liveHistory.includes('0XF872148D'));
  const hardware=await evaluate('window.hardwareRequests');assert.deepEqual(hardware,[]);
  assert.equal(errors.length,0,JSON.stringify(errors));
  const result={scope:'Isolated browser UI with frozen historical input, not hardware validation',
    imported_log:input,detail_at_5nm_comparison:detail5,detail_at_01nm_comparison:detail01,
    hardware_requests:hardware,page_errors:errors,chart_pan_and_shared_cursor:true,live_tab_history_routing:true,
    projection_changed_without_command:true,screenshot:'historical_direction_chart.png'};
  await writeFile(join(directory,'browser_verification.json'),JSON.stringify(result,null,2)+'\n',{flag:'wx'});
  console.log(JSON.stringify(result,null,2));
} catch(error) {
  const state=await evaluate(`({hardware:window.hardwareRequests,charts:[...document.querySelectorAll(${JSON.stringify(chartSelector)})].map(e=>e.textContent),ranges:[...document.querySelectorAll('input[type=range]')].map(e=>({label:e.getAttribute('aria-label'),value:e.value}))})`).catch(()=>null);
  await writeFile(join(directory,'browser_failure.json'),JSON.stringify({error:String(error),state,errors},null,2),{flag:'wx'});
  throw error;
} finally {
  try {await send('Browser.close');}catch{/* The owned process may close the socket first. */}
  ws.close();
}
