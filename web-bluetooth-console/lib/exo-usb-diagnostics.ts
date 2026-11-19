/** Read-only USB diagnostic protocol, separate from motion/BLE commands. */
import { AFO_FIELDS, LEG_FIELDS } from "./exo-adaptive.ts";
export const TRACE_FIELDS = ["id","timeUs","sequence","permitEpoch","coordinateEpoch","generation","systemState",
  "validMask","quality","pairSkewUs","previousExecutionUs","controlDtUs","systemResult","gateFlags","rejectReason","side",
  "sourceUs","sourceCount","ageUs","sourceIntervalUs","sensorFlags","previousSourceUs","previousQ","previousV",
  "qDeg","vDegS","motorEstimateNm","temperatureC","filteredVDegS","phase","periodS","lambda","profileNm","desiredNm",
  "limitedNm","suggestionNm","finalNm","flags","faults","lockReasons","intervals","eventAgeS","category"] as const;
const FLOATS = new Set<string>([...TRACE_FIELDS.slice(22,37),"eventAgeS"]);
const COUNTS = ["side","permitEpoch","total","nonzero","hard","range","acquiring","stop","downstreamZero","profileOrDirectionZero","longestZeroTicks"];
const SYSTEM = ["readTimeUs","state","runRequested","outputAllowed","faultCount","lastFault","faultTimeUs","faultCanValid","faultCanState","faultCanTxErrors","faultCanRxErrors"];
export type DiagnosticData = Record<string, number | string | null>;
export type UsbDiagnostic = { type: "usb_trace"|"usb_first_fault"|"usb_counters"|"usb_system"|"usb_identity"|"usb_config"|"usb_gap"|"usb_error";
  bootId:number; hostTime:string; data:DiagnosticData; rawLine?:string };
export type UsbFrame = { kind:"row"|"first"|"counts"|"system"|"meta"|"config"|"end"; data:DiagnosticData; rawLine:string };

function integer(text:string, signed=false):number {
  if(!(signed?/^-?\d+$/:/^\d+$/).test(text))throw new Error("诊断整数格式无效");
  const n=Number(text);if(!Number.isSafeInteger(n)||(!signed&&n<0))throw new Error("诊断整数越界");return n;
}
export function parseUsbLine(raw:string):UsbFrame|null {
  const line=raw.replace(/\x1b\[[0-9;]*m/g,"").trim();
  const match=line.match(/(?:^|>\s*)(AFO(?:ROW|FIRST|COUNTS|SYS|META|CFG|END),.*)$/);
  if(!match)return null;
  const parts=match[1].split(","), prefix=parts.shift();
  if(parts.shift()!=="1")throw new Error("不支持的USB诊断协议版本");
  if(prefix==="AFOCFG") {
    if(parts.length!==2||! /^[0-9a-f]{392}$/.test(parts[1]))throw new Error("USB配置快照长度无效");
    const bytes=Uint8Array.from(parts[1].match(/../g)!,x=>parseInt(x,16));const view=new DataView(bytes.buffer);
    let crc=0xffffffff;for(const byte of bytes.slice(0,192)){crc^=byte;for(let bit=0;bit<8;bit++)crc=(crc>>>1)^((crc&1)?0xedb88320:0);}
    if(view.getUint32(0,true)!==1||((crc^0xffffffff)>>>0)!==view.getUint32(192,true))throw new Error("USB配置CRC或schema无效");
    const data:DiagnosticData={generation:integer(parts[0]),configHex:parts[1]};
    ["overallStrength",...AFO_FIELDS.map(([key])=>`afo_${key}`),...LEG_FIELDS.map(([key])=>`left_${key}`),
      ...LEG_FIELDS.map(([key])=>`right_${key}`)].forEach((key,i)=>{const n=view.getFloat32(4+4*i,true);data[key]=Number.isFinite(n)?n:null;});
    return {kind:"config",data,rawLine:raw};
  }
  if(prefix==="AFOMETA") {
    if(parts.length!==6||!/^\d+\.\d+\.\d+$/.test(parts[1])||! /^[a-f0-9]{64}$/.test(parts[2]))throw new Error("USB身份帧损坏");
    return {kind:"meta",rawLine:raw,data:{bootId:integer(parts[0]),firmware:parts[1],elfSha256:parts[2],
      oldest:integer(parts[3]),latest:integer(parts[4]),capacity:integer(parts[5])}};
  }
  const fields=prefix==="AFOROW"||prefix==="AFOFIRST"?TRACE_FIELDS:prefix==="AFOCOUNTS"?COUNTS:
    prefix==="AFOSYS"?SYSTEM:["cursor","latest","emitted"];
  if(parts.length!==fields.length)throw new Error(`USB诊断字段不完整：${prefix}`);
  const data:DiagnosticData={};
  fields.forEach((key,i)=>{
    if((prefix==="AFOROW"||prefix==="AFOFIRST")&&FLOATS.has(key)) {
      if(!/^(?:[-+]?(?:\d+\.?\d*|\.\d+)(?:e[-+]?\d+)?|[-+]?(?:nan|inf))$/i.test(parts[i]))throw new Error("诊断浮点格式无效");
      const n=Number(parts[i]);data[key]=Number.isFinite(n)?n:null; // Preserve invalid inputs as null, never zero.
    } else data[key]=integer(parts[i],key==="systemResult");
  });
  if("side" in data&&data.side!==0&&data.side!==1)throw new Error("USB侧别无效");
  if(prefix==="AFOROW"||prefix==="AFOFIRST")data.queuedNm=(Number(data.flags)&8)!==0?data.finalNm:null;
  return {kind:prefix==="AFOROW"?"row":prefix==="AFOFIRST"?"first":prefix==="AFOCOUNTS"?"counts":prefix==="AFOSYS"?"system":"end",data,rawLine:raw};
}
export function diagnosticKey(r:UsbDiagnostic):string|null {
  if(r.type==="usb_trace"||r.type==="usb_first_fault")return `${r.bootId}:${r.type}:${r.data.id}:${r.data.side}`;
  if(r.type==="usb_identity")return `${r.bootId}:${r.type}:${r.data.elfSha256}`;
  if(r.type==="usb_config")return `${r.bootId}:${r.type}:${r.data.generation}`;
  return null;
}
export function diagnosticsCsv(rows:UsbDiagnostic[]):string {
  const keys=[...new Set(rows.flatMap(r=>Object.keys(r.data)))];
  const cell=(v:unknown)=>v==null?"":`"${String(v).replaceAll('"','""')}"`;
  return [["type","bootId","hostTime",...keys,"rawLine"].join(","),...rows.map(r=>
    [r.type,r.bootId,r.hostTime,...keys.map(k=>r.data[k]),r.rawLine].map(cell).join(","))].join("\r\n")+"\r\n";
}

export interface SerialPortLike {
  open(options:{baudRate:number;bufferSize:number;flowControl:"none"}):Promise<void>;
  close():Promise<void>;
  setSignals(signals:{dataTerminalReady:boolean;requestToSend:boolean}):Promise<void>;
  getInfo():{usbVendorId?:number;usbProductId?:number};
  readable:ReadableStream<Uint8Array>|null;writable:WritableStream<Uint8Array>|null;
}
export interface BrowserSerial {requestPort(options:{filters:{usbVendorId:number;usbProductId:number}[]}):Promise<SerialPortLike>}
export function browserSerial():BrowserSerial|undefined {
  return typeof navigator==="undefined"?undefined:(navigator as Navigator&{serial?:BrowserSerial}).serial;
}

/** Single outstanding read command. No raw command entry, reset, START or enable. */
export class AfoUsbReader {
  private port:SerialPortLike;
  private reader?:ReadableStreamDefaultReader<Uint8Array>;
  private writer?:WritableStreamDefaultWriter<Uint8Array>;
  private readTask?:Promise<void>;
  private pending?:{frames:UsbFrame[];resolve:(frames:UsbFrame[])=>void;reject:(e:Error)=>void};
  private closed=false;private cursor=0;private inFlight:Promise<void>=Promise.resolve();
  private line="";private opened=false;private backlog=false;
  private bootId:number;
  private receive:(r:UsbDiagnostic)=>void;
  private stopped:(message:string)=>void;
  identity:DiagnosticData|null=null;
  constructor(port:SerialPortLike,bootId:number,receive:(r:UsbDiagnostic)=>void,stopped:(message:string)=>void) {
    this.port=port;this.bootId=bootId;this.receive=receive;this.stopped=stopped;
  }
  private emit(type:UsbDiagnostic["type"],data:DiagnosticData,rawLine?:string) {
    this.receive({type,bootId:this.bootId,hostTime:new Date().toISOString(),data,rawLine});
  }
  async connect():Promise<void> {
    const info=this.port.getInfo();
    if(info.usbVendorId!==0x303a||info.usbProductId!==0x1001)throw new Error("请选择ESP32原生USB烧录接口");
    try {
      await this.port.open({baudRate:115200,bufferSize:65536,flowControl:"none"});this.opened=true;
      await this.port.setSignals({dataTerminalReady:false,requestToSend:false});
      if(!this.port.readable||!this.port.writable)throw new Error("USB端口不可读写");
      this.reader=this.port.readable.getReader();this.writer=this.port.writable.getWriter();
      this.readTask=this.readLoop().catch(error=>{this.pending?.reject(error instanceof Error?error:new Error(String(error)));});
      await this.poll(true);
      void this.loop();
    } catch(error) {await this.disconnect();throw error;}
  }
  private async readLoop() {
    const decoder=new TextDecoder();
    while(!this.closed) {
      const result=await this.reader!.read();
      if(result.done)throw new Error("USB数据连接已断开");
      this.line+=decoder.decode(result.value,{stream:true});
      let index:number;
      while((index=this.line.indexOf("\n"))>=0) {
        const raw=this.line.slice(0,index).replace(/\r$/,"");this.line=this.line.slice(index+1);
        try {
          const frame=parseUsbLine(raw);if(!frame||!this.pending)continue;
          this.pending.frames.push(frame);
          if(this.pending.frames.length>80){this.pending.reject(new Error("USB响应超过有界帧数"));continue;}
          if(frame.kind==="end")this.pending.resolve(this.pending.frames);
        } catch(error) {this.emit("usb_error",{message:String(error)},raw);}
      }
      if(this.line.length>4096) {this.emit("usb_error",{message:"USB行过长或丢失换行"});this.line="";}
    }
  }
  private async poll(initial=false) {
    let timer:ReturnType<typeof setTimeout>|undefined;
    const response=new Promise<UsbFrame[]>((resolve,reject)=>{
      this.pending={frames:[],resolve,reject};timer=setTimeout(()=>reject(new Error("USB诊断响应超时；请检查端口占用或固件版本")),3000);
    });
    // Install both rejection handlers before write can fail.
    const send=this.writer!.write(new TextEncoder().encode(`afo_trace ${this.cursor}\r`));
    let frames:UsbFrame[];
    try {[,frames]=await Promise.all([send,response]);}
    finally {clearTimeout(timer);this.pending=undefined;}
    const meta=frames.find(f=>f.kind==="meta"),end=frames.at(-1);
    if(!meta||end?.kind!=="end")throw new Error("USB诊断响应缺少身份或结束帧");
    if(meta.data.bootId!==this.bootId)throw new Error("USB与蓝牙启动编号不一致：设备可能重启或选错端口，请重新连接核对");
    if(meta.data.firmware!=="0.11.23")throw new Error("USB诊断需要0.11.23固件");
    if(!initial&&Number(meta.data.latest)<this.cursor)throw new Error("USB序号回退，请重新连接核对设备");
    this.identity=meta.data;this.emit("usb_identity",meta.data,meta.rawLine);
    if(!initial&&Number(meta.data.oldest)>this.cursor+1)
      this.emit("usb_gap",{reason:"device_ring_overflow",fromId:this.cursor+1,toId:Number(meta.data.oldest)-1});
    const pairs=new Map<number,Set<number>>();
    for(const frame of frames) {
      if(frame.kind==="row") {
        if(initial)continue; // Handshake discards pre-connection backlog, explicitly starts at latest.
        const id=Number(frame.data.id),side=Number(frame.data.side);
        if(id<=this.cursor||id>Number(meta.data.latest))throw new Error("USB帧序号超出本批快照范围");
        const legs=pairs.get(id)??new Set<number>();legs.add(side);pairs.set(id,legs);
        this.emit("usb_trace",frame.data,frame.rawLine);
      } else if(frame.kind==="first")this.emit("usb_first_fault",frame.data,frame.rawLine);
      else if(frame.kind==="config")this.emit("usb_config",frame.data,frame.rawLine);
      else if(frame.kind==="counts")this.emit("usb_counters",frame.data,frame.rawLine);
      else if(frame.kind==="system")this.emit("usb_system",frame.data,frame.rawLine);
    }
    if(!initial) {
      let expected=Math.max(this.cursor+1,Number(meta.data.oldest));
      for(const [id,sides] of pairs) {
        if(id!==expected||sides.size!==2)this.emit("usb_gap",{reason:"missing_or_partial_pair",expectedId:expected,id,receivedSides:sides.size});
        expected=id+1;
      }
      if(Number(end.data.cursor)>=expected)this.emit("usb_gap",{reason:"missing_tail",fromId:expected,toId:Number(end.data.cursor)});
    }
    this.cursor=Number(initial?meta.data.latest:end.data.cursor);
    this.backlog=this.cursor<Number(meta.data.latest);
  }
  private async loop() {
    try {
      while(!this.closed) {
        if(!this.backlog)await new Promise(resolve=>setTimeout(resolve,100));
        if(this.closed)break;
        this.inFlight=this.poll();await this.inFlight;
      }
    } catch(error) {
      if(!this.closed){const message=String(error);this.emit("usb_error",{message});this.stopped(message);}
      await this.disconnect();
    }
  }
  async checkpoint():Promise<void> {await this.inFlight.catch(()=>{});}
  async disconnect():Promise<void> {
    if(this.closed)return;this.closed=true;
    this.pending?.reject(new Error("USB诊断已断开"));
    await this.reader?.cancel().catch(()=>{});await this.readTask;
    this.reader?.releaseLock();this.writer?.releaseLock();
    if(this.opened)await this.port.close();
  }
}
