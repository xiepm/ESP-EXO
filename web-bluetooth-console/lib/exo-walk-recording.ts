import { telemetrySamplesToCsv, type TelemetrySamplePoint } from "./exo-telemetry-stream.ts";
import { diagnosticKey, diagnosticsCsv, type UsbDiagnostic } from "./exo-usb-diagnostics.ts";

export type RecordingEvent = { hostTime: string; type: string; value: unknown };
export type RecordingMeta = {
  id: string; startedAt: string; endedAt: string | null; endReason: string | null;
  count: number; schema: 1|2; context: unknown; events: RecordingEvent[]; diagnosticCount?:number;
};
export type RecordingRow = { id: string; key: string; ordinal: number; sample: TelemetrySamplePoint };
export type DiagnosticRow = {id:string;key:string;ordinal:number;diagnostic:UsbDiagnostic};
export type RecordingWriter = (meta: RecordingMeta, rows: RecordingRow[], diagnostics?:DiagnosticRow[]) => Promise<void>;
export const MAX_RECORDING_SAMPLES = 72000;
export const MAX_DIAGNOSTIC_ROWS = 120000;

/** Independent from the rolling chart buffer. Same-time channel packets replace
 * the same row, including updates arriving while an IndexedDB save is pending. */
export class WalkRecorder {
  meta: RecordingMeta;
  private rows = new Map<string, RecordingRow>();
  private dirty = new Map<string, RecordingRow>();
  private saving: Promise<void> | null = null;
  private diagnosticRows=new Map<string,DiagnosticRow>();
  private diagnosticDirty=new Map<string,DiagnosticRow>();
  private writer: RecordingWriter;
  constructor(id: string, context: unknown, writer: RecordingWriter = saveRecording) {
    this.writer = writer;
    this.meta = { id, startedAt: new Date().toISOString(), endedAt: null, endReason: null,
      count: 0, schema: 2, diagnosticCount:0, context: structuredClone(context), events: [] };
  }
  accept(sample: TelemetrySamplePoint): void {
    if (this.meta.endedAt) return;
    const key = `${sample.webSession}:${sample.streamSequence}:${sample.timestampMs}:${sample.sampleSequence}`;
    const old = this.rows.get(key);
    if (!old && this.rows.size >= MAX_RECORDING_SAMPLES) { this.end("达到72000个采样时刻上限，请开始下一段"); return; }
    const row = { id: this.meta.id, key, ordinal: old?.ordinal ?? this.rows.size, sample: { ...sample } };
    this.rows.set(key, row); this.dirty.set(key, row); this.meta.count = this.rows.size;
  }
  note(type: string, value: unknown): void {
    if (!this.meta.endedAt) this.meta.events.push({ hostTime: new Date().toISOString(), type, value: structuredClone(value) });
  }
  acceptDiagnostic(diagnostic:UsbDiagnostic):void {
    if(this.meta.endedAt)return;
    const key=diagnosticKey(diagnostic)??`${diagnostic.type}:${this.diagnosticRows.size}`;
    const old=this.diagnosticRows.get(key);
    if(old)return; // Same sticky first fault / identity / retransmitted row is not a new sample.
    if(this.diagnosticRows.size>=MAX_DIAGNOSTIC_ROWS){this.end("USB诊断达到120000行上限，请保存并开始下一段");return;}
    const row={id:this.meta.id,key,ordinal:this.diagnosticRows.size,diagnostic:structuredClone(diagnostic)};
    this.diagnosticRows.set(key,row);this.diagnosticDirty.set(key,row);this.meta.diagnosticCount=this.diagnosticRows.size;
  }
  diagnostics():UsbDiagnostic[]{return [...this.diagnosticRows.values()].map(row=>row.diagnostic);}
  end(reason: string): void {
    if (!this.meta.endedAt) { this.meta.endedAt = new Date().toISOString(); this.meta.endReason = reason; }
  }
  async flush(): Promise<void> {
    if (this.saving) { await this.saving; return this.flush(); }
    const batch = [...this.dirty.values()], diagnosticBatch=[...this.diagnosticDirty.values()], meta = structuredClone(this.meta);
    this.saving = this.writer(meta, batch, diagnosticBatch);
    try {
      await this.saving;
      for (const row of batch) if (this.dirty.get(row.key) === row) this.dirty.delete(row.key);
      for(const row of diagnosticBatch)if(this.diagnosticDirty.get(row.key)===row)this.diagnosticDirty.delete(row.key);
    } finally { this.saving = null; }
  }
  samples(): TelemetrySamplePoint[] { return [...this.rows.values()].map(row => row.sample); }
}

let database: Promise<IDBDatabase> | undefined;
function openDatabase(): Promise<IDBDatabase> {
  if (!database) database = new Promise<IDBDatabase>((resolve, reject) => {
    const request = indexedDB.open("esp-exo-walking-recordings", 2);
    request.onupgradeneeded = () => {
      const db=request.result;
      if(!db.objectStoreNames.contains("sessions"))db.createObjectStore("sessions", { keyPath: "id" });
      for(const name of ["samples","diagnostics"])if(!db.objectStoreNames.contains(name)) {
        const rows=db.createObjectStore(name,{keyPath:["id","key"]});rows.createIndex("session","id");
      }
    };
    request.onsuccess = () => resolve(request.result);
    request.onerror = () => reject(request.error ?? new Error("无法打开本地记录存储"));
    request.onblocked = () => reject(new Error("本地记录存储被其他页面占用"));
  }).catch(error => { database = undefined; throw error; });
  return database;
}
function completion(tx: IDBTransaction): Promise<void> {
  return new Promise((resolve, reject) => {
    tx.oncomplete = () => resolve();
    tx.onabort = tx.onerror = () => reject(tx.error ?? new Error("本地保存失败，数据仍在本页内存中"));
  });
}
export async function saveRecording(meta: RecordingMeta, rows: RecordingRow[], diagnostics:DiagnosticRow[]=[]): Promise<void> {
  const db = await openDatabase(), tx = db.transaction(["sessions", "samples","diagnostics"], "readwrite");
  const done = completion(tx);
  tx.objectStore("sessions").put(meta);
  for (const row of rows) tx.objectStore("samples").put(row);
  for(const row of diagnostics)tx.objectStore("diagnostics").put(row);
  await done;
}
export async function listRecordings(): Promise<RecordingMeta[]> {
  const db = await openDatabase();
  return new Promise((resolve, reject) => {
    const request = db.transaction("sessions").objectStore("sessions").getAll();
    request.onsuccess = () => resolve((request.result as RecordingMeta[]).sort((a,b) => b.startedAt.localeCompare(a.startedAt)));
    request.onerror = () => reject(request.error);
  });
}
export async function readRecordingSamples(id: string): Promise<TelemetrySamplePoint[]> {
  const db = await openDatabase();
  return new Promise((resolve, reject) => {
    const request = db.transaction("samples").objectStore("samples").index("session").getAll(id);
    request.onsuccess = () => resolve((request.result as RecordingRow[]).sort((a,b) => a.ordinal-b.ordinal).map(row => row.sample));
    request.onerror = () => reject(request.error);
  });
}
export async function readRecordingDiagnostics(id:string):Promise<UsbDiagnostic[]> {
  const db=await openDatabase();
  return new Promise((resolve,reject)=>{
    const request=db.transaction("diagnostics").objectStore("diagnostics").index("session").getAll(id);
    request.onsuccess=()=>resolve((request.result as DiagnosticRow[]).sort((a,b)=>a.ordinal-b.ordinal).map(row=>row.diagnostic));
    request.onerror=()=>reject(request.error);
  });
}
export function recordingText(meta: RecordingMeta, samples: TelemetrySamplePoint[], format: "csv" | "jsonl" | "diagnosticsCsv", diagnostics:UsbDiagnostic[]=[]): string {
  if(format==="diagnosticsCsv")return diagnosticsCsv(diagnostics);
  if (format === "csv") {
    // Status is an asynchronous readback, not a same-tick measurement. Export
    // its host time alongside each sample and never backfill a later fault.
    const base=telemetrySamplesToCsv(samples).trimEnd().split(/\r?\n/);
    const extra=["contextReadHostTime","firmware","strategy","leftLevelPercent","rightLevelPercent","maximumNm",
      "assistUnavailableReason","leftAfoFaults","rightAfoFaults","leftAfoFirstFaults","rightAfoFirstFaults",
      "leftAfoFirstTimestampMs","rightAfoFirstTimestampMs"];
    type Context={info?:{firmware?:number[]};assistConfig?:{strategyName?:string;leftLevelPercent?:number;
      rightLevelPercent?:number;unavailableReasonName?:string};assistTorqueLimit?:{selectedMaximumNm?:number};
      afoStatuses?:({faults:number;firstPresent:boolean;firstFaults:number;firstTimestampMs:number}|null)[]};
    let context=(meta.context??{}) as Context,at=meta.startedAt,index=0;
    const events=meta.events.filter(e=>e.type==="device_state").slice().sort((a,b)=>a.hostTime.localeCompare(b.hostTime));
    const cell=(value:unknown)=>value==null?"":`"${String(value).replaceAll('"','""')}"`;
    const result=[base[0]+","+extra.join(",")];
    samples.forEach((sample,n)=>{
      while(index<events.length&&events[index].hostTime<=sample.receivedIsoTime){
        context=(events[index].value??{}) as Context;at=events[index].hostTime;index++;
      }
      const a=context.assistConfig,left=context.afoStatuses?.[0],right=context.afoStatuses?.[1];
      result.push(base[n+1]+","+[at,context.info?.firmware?.join("."),a?.strategyName,a?.leftLevelPercent,
        a?.rightLevelPercent,context.assistTorqueLimit?.selectedMaximumNm,a?.unavailableReasonName,
        left?.faults,right?.faults,left?.firstPresent?left.firstFaults:null,right?.firstPresent?right.firstFaults:null,
        left?.firstPresent?left.firstTimestampMs:null,right?.firstPresent?right.firstTimestampMs:null].map(cell).join(","));
    });
    return result.join("\r\n")+"\r\n";
  }
  return [JSON.stringify({ type: "session", ...meta,
    units: { angles: "deg", velocity: "deg/s", torque: "N*m", deviceTime: "ms",usbDeviceTime:"us" },
    note: "applied torque is the added command, not measured human interaction torque; absent channels remain null" }),
    ...samples.map(sample => JSON.stringify({ type: "sample", ...sample })),
    ...diagnostics.map(row=>JSON.stringify(row))].join("\n") + "\n";
}
