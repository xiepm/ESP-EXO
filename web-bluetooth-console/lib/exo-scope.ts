import { TelemetryChannel } from "./exo-ble-protocol.ts";
import type { TelemetrySamplePoint } from "./exo-telemetry-stream.ts";

export const SCOPE_BUFFER_SAMPLES = 640; // 30秒×20 Hz，并留少量通道组装余量。
export const WALKING_SCOPE_MASK = TelemetryChannel.POSITION | TelemetryChannel.VELOCITY |
  TelemetryChannel.GAIT_PHASE | TelemetryChannel.GAIT_CONFIDENCE |
  TelemetryChannel.ASSIST_SUGGESTION | TelemetryChannel.APPLIED_TORQUE;
type ValueField = keyof Pick<TelemetrySamplePoint, "leftPositionDeg" | "rightPositionDeg" |
  "leftVelocityDegS" | "rightVelocityDegS" | "leftMotorTorqueNm" | "rightMotorTorqueNm" |
  "leftPhasePercent" | "rightPhasePercent" | "leftConfidencePercent" | "rightConfidencePercent" |
  "leftAssistSuggestionNm" | "rightAssistSuggestionNm" | "leftAppliedTorqueNm" | "rightAppliedTorqueNm">;
export type ScopeChannel = { bit: number; title: string; unit: string; left: ValueField;
  right: ValueField; range: readonly [number, number]; minimumSpan: number; unavailable: string };
export const SCOPE_CHANNELS: readonly ScopeChannel[] = [
  {bit:1,title:"髋角度",unit:"°",left:"leftPositionDeg",right:"rightPositionDeg",range:[-30,60],minimumSpan:10,unavailable:"等待有效关节角度；检查反馈及穿戴零位"},
  {bit:2,title:"髋角速度",unit:"°/s",left:"leftVelocityDegS",right:"rightVelocityDegS",range:[-120,120],minimumSpan:20,unavailable:"等待有效关节角速度"},
  {bit:8,title:"步态相位（髋角事件估计）",unit:"%",left:"leftPhasePercent",right:"rightPhasePercent",range:[0,100],minimumSpan:100,unavailable:"等待本侧有效周期运动；此通道不是AFO锁相状态"},
  {bit:32,title:"步态置信度（髋角事件估计）",unit:"%",left:"leftConfidencePercent",right:"rightConfidencePercent",range:[0,100],minimumSpan:100,unavailable:"等待有效髋角事件估计；此通道不是AFO置信度"},
  {bit:16,title:"建议助力",unit:"N·m",left:"leftAssistSuggestionNm",right:"rightAssistSuggestionNm",range:[-5,5],minimumSpan:0.5,unavailable:"策略未产生有效建议；检查运行方式与事件门禁"},
  {bit:64,title:"实际加入命令的助力增量",unit:"N·m",left:"leftAppliedTorqueNm",right:"rightAppliedTorqueNm",range:[-5,5],minimumSpan:0.5,unavailable:"等待有效命令增量；该值并非实测人体力矩"},
  {bit:4,title:"电机反馈力矩",unit:"N·m",left:"leftMotorTorqueNm",right:"rightMotorTorqueNm",range:[-5,5],minimumSpan:0.5,unavailable:"等待电机估算反馈力矩；非独立力矩传感器"},
];
export type ScopePoint = { sample: TelemetrySamplePoint; seconds: number };
export function finiteScopeValue(value: number | null | undefined): number | null {
  return typeof value === "number" && Number.isFinite(value) ? value : null;
}

/** 横轴使用主控采样时间，而非Notify到达顺序；32位毫秒自然回绕仍连续。 */
export function scopeWindow(samples: readonly TelemetrySamplePoint[], seconds: number): ScopePoint[] {
  if (!samples.length || !Number.isFinite(seconds) || seconds <= 0) return [];
  const stream = samples.at(-1)!.streamSequence;
  const session = samples.at(-1)!.webSession;
  let latest = samples.at(-1)!;
  for (const sample of samples) {
    const forward = (sample.sampleSequence - latest.sampleSequence) >>> 0;
    if (sample.webSession === session && sample.streamSequence === stream && forward > 0 && forward < 0x8000_0000) latest = sample;
  }
  return samples.filter((sample) => sample.webSession === session && sample.streamSequence === stream)
    .map((sample) => ({sample, seconds: ((latest.timestampMs - sample.timestampMs) >>> 0) === 0
      ? 0 : -((latest.timestampMs - sample.timestampMs) >>> 0) / 1000}))
    .filter((point) => point.seconds >= -seconds)
    .sort((a,b) => a.seconds - b.seconds);
}

/** 同一时刻分通道发送：最后一个尚未收到此通道的点不能擦掉已收到的读数。
 * 已收到但无效(null)则必须显示缺失，绝不拿旧有效值代替当前故障。 */
export function scopeLatest(points: readonly ScopePoint[], channel: ScopeChannel): ScopePoint | null {
  for (let i=points.length-1;i>=0;i--) {
    if ((points[i].sample.receivedChannelMask & channel.bit) !== 0) return points[i];
  }
  return null;
}
export function scopeRange(points: readonly ScopePoint[], channel: ScopeChannel, auto: boolean): readonly [number,number] {
  if (!auto || channel.unit === "%") return channel.range;
  const values = points.flatMap(({sample}) => [finiteScopeValue(sample[channel.left]),finiteScopeValue(sample[channel.right])])
    .filter((value): value is number => value !== null);
  if (!values.length) return channel.range;
  let low=Math.min(0,...values), high=Math.max(0,...values);
  const span=Math.max(channel.minimumSpan,high-low), midpoint=(low+high)/2;
  low=midpoint-span*0.6; high=midpoint+span*0.6;
  return [low,high];
}

export const SCOPE_PLOT = {left:56, right:708, top:16, bottom:202, width:720, height:240} as const;
export function scopeX(seconds: number, windowSeconds: number): number {
  return SCOPE_PLOT.left+(seconds+windowSeconds)/windowSeconds*(SCOPE_PLOT.right-SCOPE_PLOT.left);
}
export function scopeY(value: number, range: readonly [number,number]): number {
  return SCOPE_PLOT.bottom-(value-range[0])/(range[1]-range[0])*(SCOPE_PLOT.bottom-SCOPE_PLOT.top);
}

/** 缺帧、无效值、切换激活坐标及超时都断线，禁止跨缺失数据连出假波形。 */
export function scopePaths(points: readonly ScopePoint[], channel: ScopeChannel, side: "left"|"right",
  windowSeconds: number, range: readonly [number,number], rateHz: number): string[] {
  const paths: string[]=[]; let path=""; let previous: ScopePoint|null=null;
  for (const point of points) {
    const value=finiteScopeValue(point.sample[channel[side]]);
    const received=(point.sample.receivedChannelMask & channel.bit)!==0;
    const broken=previous && ((point.sample.sampleSequence-previous.sample.sampleSequence)>>>0)!==1;
    const gap=previous && point.seconds-previous.seconds>Math.max(0.35,2.5/rateHz);
    const epoch=previous && point.sample.activationEpochLow16!==previous.sample.activationEpochLow16;
    if (!received || value===null) { if(path) paths.push(path); path=""; previous=null; continue; }
    if (broken || gap || epoch || point.sample.droppedSincePrevious) { if(path) paths.push(path); path=""; }
    const xy=`${scopeX(point.seconds,windowSeconds).toFixed(2)},${scopeY(value,range).toFixed(2)}`;
    path+=`${path?" L":"M"}${xy}`; previous=point;
  }
  if(path) paths.push(path);
  return paths;
}
export function scopeNearest(points: readonly ScopePoint[], channel: ScopeChannel, seconds: number, rateHz: number): ScopePoint|null {
  const candidates=points.filter(({sample}) => (sample.receivedChannelMask & channel.bit)!==0);
  let closest: ScopePoint|null=null;
  for(const point of candidates) if(!closest || Math.abs(point.seconds-seconds)<Math.abs(closest.seconds-seconds)) closest=point;
  return closest && Math.abs(closest.seconds-seconds)<=Math.max(0.15,1.5/rateHz) ? closest : null;
}
export function scopeReceptionState(connected: boolean, streaming: boolean, lastReceivedMs: number|null,
  nowMs: number): "disconnected"|"stopped"|"waiting"|"stale"|"live" {
  if(!connected) return "disconnected";
  if(!streaming) return "stopped";
  if(lastReceivedMs===null) return "waiting";
  return nowMs-lastReceivedMs>1500 ? "stale" : "live";
}
export function shouldStartScope(input: {livePanel:boolean;connected:boolean;ready:boolean;compatible:boolean;
  automatic:boolean;streaming:boolean|null;busy:boolean;session:number;attempted:number|null}): boolean {
  return input.livePanel && input.connected && input.ready && input.compatible && input.automatic &&
    input.streaming===false && !input.busy && input.session!==input.attempted;
}
