"use client";
import { useCallback, useEffect, useRef, useState, type RefObject } from "react";
import { WalkRecorder, listRecordings, readRecordingSamples, readRecordingDiagnostics, recordingText, type RecordingMeta } from "@/lib/exo-walk-recording";
import { AfoUsbReader, browserSerial, type DiagnosticData } from "@/lib/exo-usb-diagnostics";

function download(name: string, text: string, format: "csv" | "jsonl") {
  const url = URL.createObjectURL(new Blob([format === "csv" ? "\ufeff" + text : text],
    {type: format === "csv" ? "text/csv;charset=utf-8" : "application/x-ndjson;charset=utf-8"}));
  const link = document.createElement("a"); link.href=url; link.download=name; link.click();
  window.setTimeout(() => URL.revokeObjectURL(url), 1000);
}
export function useWalkRecording(ref: RefObject<WalkRecorder | null>, connected: boolean, context: unknown) {
  const [active, setActive] = useState(false), [busy, setBusy] = useState(false);
  const [message, setMessage] = useState(""), [history, setHistory] = useState<RecordingMeta[]>([]);
  const [count, setCount] = useState(0);
  const contextRef=useRef(context); contextRef.current=context;
  const usbRef=useRef<AfoUsbReader|null>(null);
  const usbStats=useRef({received:0,gaps:0,lastPublish:0});
  const [usb,setUsb]=useState({connected:false,busy:false,message:"USB未连接：仅记录蓝牙低频数据",received:0,gaps:0,identity:null as DiagnosticData|null});
  async function connectUsb() {
    const serial=browserSerial();
    if(!serial){setUsb(s=>({...s,message:"当前浏览器不支持USB记录，请使用电脑Chrome或Edge的本地/HTTPS页面"}));return;}
    const info=(contextRef.current as {info?:{bootId:number;firmware:number[]}})?.info;
    if(!connected||!info||info.firmware.join(".")!=="0.11.23") {setUsb(s=>({...s,message:"先连接蓝牙并读取0.11.23设备信息，再连接USB"}));return;}
    setUsb(s=>({...s,busy:true}));
    try {
      const port=await serial.requestPort({filters:[{usbVendorId:0x303a,usbProductId:0x1001}]});
      usbStats.current={received:0,gaps:0,lastPublish:0};
      const reader=new AfoUsbReader(port,info.bootId,row=>{
        ref.current?.acceptDiagnostic(row);
        if(row.type==="usb_trace")usbStats.current.received++;
        if(row.type==="usb_gap"||row.type==="usb_error")usbStats.current.gaps++;
        const now=Date.now();
        if(now-usbStats.current.lastPublish>500){usbStats.current.lastPublish=now;setUsb(s=>({...s,received:usbStats.current.received,gaps:usbStats.current.gaps}));}
      },message=>{
        ref.current?.note("usb_disconnected",{message});
        usbRef.current=null;setUsb(s=>({...s,connected:false,message:`USB已断开：${message}。蓝牙记录继续。`}));
      });
      usbRef.current=reader;await reader.connect();
      setUsb(s=>({...s,connected:true,identity:reader.identity,message:"USB诊断已连接，同一设备启动编号已核对"}));
      ref.current?.note("usb_connected",reader.identity);
    }catch(error){usbRef.current=null;ref.current?.note("usb_connection_error",{message:String(error)});setUsb(s=>({...s,connected:false,message:`USB连接失败：${String(error)}`}));}
    finally{setUsb(s=>({...s,busy:false}));}
  }
  async function disconnectUsb() {
    const reader=usbRef.current;usbRef.current=null;
    try{await reader?.disconnect();}catch(error){setMessage(`USB关闭异常：${String(error)}`);}
    ref.current?.note("usb_disconnected",{reason:"用户断开USB诊断"});
    setUsb(s=>({...s,connected:false,message:"USB诊断已断开，蓝牙记录可继续"}));
  }
  useEffect(()=>()=>{void usbRef.current?.disconnect().catch(()=>{});},[]);
  const refresh=useCallback(async () => setHistory(await listRecordings()),[]);
  useEffect(() => { void refresh().catch(error => setMessage(`本地存储不可用：${String(error)}`)); },[refresh]);
  const stop=useCallback(async (reason="手动停止记录") => {
    const recorder=ref.current; if(!recorder)return;
    await usbRef.current?.checkpoint();
    recorder.end(reason);setActive(false);setBusy(true);
    try { await recorder.flush(); await refresh();setMessage(`已保存 ${recorder.meta.count} 个蓝牙时刻、${recorder.meta.diagnosticCount??0} 条USB诊断：${reason}`); }
    catch(error) { setMessage(`保存失败：${String(error)}。请使用“导出当前记录”保留本页数据。`); }
    finally { setBusy(false); }
  },[ref,refresh]);
  const contextBoot=(context as {info?:{bootId:number}})?.info?.bootId;
  useEffect(()=>{
    const reader=usbRef.current;
    if(reader?.identity&&contextBoot!=null&&reader.identity.bootId!==contextBoot) {
      const reason="蓝牙设备启动编号已改变，停止本段以免混合不同设备/启动周期";
      ref.current?.note("device_boot_mismatch",{usbBoot:reader.identity.bootId,bleBoot:contextBoot});
      ref.current?.end(reason);void stop(reason);
      usbRef.current=null;void reader.disconnect().catch(()=>{});
      setUsb(s=>({...s,connected:false,message:reason}));
    }
  },[contextBoot,usb.connected,ref,stop]);
  useEffect(() => {
    if(!active)return;
    let flushing=false;
    const timer=window.setInterval(() => {
      const recorder=ref.current;if(!recorder||flushing)return;
      setCount(recorder.meta.count);
      if(recorder.meta.endedAt) {void stop(recorder.meta.endReason ?? "记录结束");return;}
      flushing=true;
      void recorder.flush().then(()=>setMessage(`本地自动保存 · ${recorder.meta.count} 个蓝牙时刻、${recorder.meta.diagnosticCount??0} 条USB诊断`))
        .catch(error=>{recorder.end("本地保存失败");setActive(false);setMessage(`保存失败：${String(error)}。请导出当前记录。`);})
        .finally(()=>{flushing=false;});
    },1000);
    const beforeUnload=(event:BeforeUnloadEvent)=>{event.preventDefault();};
    window.addEventListener("beforeunload",beforeUnload);
    return ()=>{window.clearInterval(timer);window.removeEventListener("beforeunload",beforeUnload);};
  },[active,ref,stop]);
  useEffect(()=>{
    if(active&&!connected) {
      if(usb.connected)ref.current?.note("ble_disconnected",{usbContinues:true});
      else void stop("蓝牙与USB均未连接，已结束本段记录");
    }
  },[active,connected,usb.connected,ref,stop]);
  useEffect(()=>{
    const recorder=ref.current;
    if(recorder&&!recorder.meta.endedAt)recorder.note("device_state",context);
  },[context,ref]);
  useEffect(()=>()=>{
    const recorder=ref.current;
    if(recorder&&!recorder.meta.endedAt){recorder.end("页面离开");void recorder.flush().catch(()=>{});}
  },[ref]);
  async function start() {
    if(busy||active)return;
    setBusy(true);
    try {
      // Finish saving the previous segment before replacing its RAM fallback.
      if(ref.current)await ref.current.flush();
      const recorder=new WalkRecorder(crypto.randomUUID(),contextRef.current);
      recorder.note("usb_recording_context",{connected:!!usbRef.current?.identity,identity:usbRef.current?.identity??null,
        note:"USB为控制周期快照；BLE为低频遥测；两者不互相插值补零"});
      await recorder.flush();ref.current=recorder;setCount(0);setActive(true);
      setMessage("记录中，等待新的遥测数据；每秒自动保存。");
    } catch(error){setMessage(`无法开始记录：${String(error)}`);}
    finally{setBusy(false);}
  }
  async function exportRecord(meta:RecordingMeta,format:"csv"|"jsonl"|"diagnosticsCsv") {
    try {
      const current=ref.current?.meta.id===meta.id?ref.current:null;
      const samples=current?current.samples():await readRecordingSamples(meta.id);
      const diagnostics=current?current.diagnostics():await readRecordingDiagnostics(meta.id);
      const snapshot=current?structuredClone(current.meta):meta;
      const extension=format==="jsonl"?"jsonl":"csv",suffix=format==="diagnosticsCsv"?"-usb-diagnostics":"";
      download(`exo-walk-${meta.startedAt.replaceAll(":","-")}-${meta.id.slice(0,8)}${suffix}.${extension}`,recordingText(snapshot,samples,format,diagnostics),extension);
    }catch(error){setMessage(`导出失败：${String(error)}`);}
  }
  return {active,busy,message,history,count,start,stop,exportRecord,current:ref.current?.meta ?? null,usb,connectUsb,disconnectUsb};
}
export function WalkRecordingPanel({recording,canStart,start}:{recording:ReturnType<typeof useWalkRecording>;canStart:boolean;start:()=>void}) {
  return <section className="exo-operation-note" aria-label="行走数据记录">
    <h3>行走数据记录</h3>
    <p>建议测试前连接USB烧录线并选择“连接USB诊断”，再按原来的操作流程测试。USB补充目标100 Hz双髋反馈、输出门和首次故障；连接与记录都不启动电机。请先关闭占用同一端口的串口监视器，并保持网页在前台。</p>
    <div className="exo-export-row">
      <button className="exo-button secondary" disabled={recording.usb.busy||recording.usb.connected} onClick={()=>void recording.connectUsb()}>连接USB诊断</button>
      <button className="exo-button ghost" disabled={!recording.usb.connected} onClick={()=>void recording.disconnectUsb()}>断开USB诊断</button>
    </div>
    <p role="status">{recording.usb.message} · 本次USB连接已接收 {recording.usb.received} 条腿快照 · 缺口/解析异常 {recording.usb.gaps} 次</p>
    <p>记录保存在本浏览器，刷新后可从历史记录导出。每秒自动保存；每段最多72000个时刻，暂停图表不会暂停记录。关闭页面前请先停止并保存。</p>
    <div className="exo-export-row">
      <button className="exo-button primary" disabled={!canStart||recording.active||recording.busy} onClick={start}>开始记录</button>
      <button className="exo-button secondary" disabled={!recording.active||recording.busy} onClick={()=>void recording.stop()}>停止并保存记录</button>
      {recording.current&&<button className="exo-button ghost" onClick={()=>void recording.exportRecord(recording.current!,"jsonl")}>导出当前记录 JSONL</button>}
    </div>
    <p role="status">{recording.active?`正在记录 · ${recording.count} 个时刻。 `:""}{recording.message}</p>
    <p>完整JSONL合并保存蓝牙、USB、设备设置、故障和缺帧信息；USB另有诊断CSV。USB异常不会伪造零命令。每段最多120000条USB诊断，达到上限会结束记录；关闭页面前请导出保存。</p>
    <details><summary>已保存记录（{recording.history.length}）</summary>
      {recording.history.map(meta=><div key={meta.id} className="exo-export-row">
        <span>{new Date(meta.startedAt).toLocaleString()} · {meta.count} 个蓝牙时刻 · {meta.diagnosticCount??0} 条USB诊断 · {meta.endReason ?? "未正常结束，已保存部分可导出"}</span>
        <button className="exo-button ghost" onClick={()=>void recording.exportRecord(meta,"csv")}>下载 CSV</button>
        <button className="exo-button ghost" onClick={()=>void recording.exportRecord(meta,"jsonl")}>下载完整 JSONL</button>
        <button className="exo-button ghost" disabled={!meta.diagnosticCount} onClick={()=>void recording.exportRecord(meta,"diagnosticsCsv")}>下载USB诊断 CSV</button>
      </div>)}
    </details>
  </section>;
}
