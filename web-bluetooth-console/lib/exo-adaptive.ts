/** 0.11.0 工程配置与离线日志。所有导出均为本地文件，不建立硬件连接。 */
export const AFO_FIELDS = [
  ["minimum_hz", "最低频率（Hz）", .2, 2, .01],
  ["maximum_hz", "最高频率（Hz）", .4, 3, .01],
  ["initial_hz", "初始频率（Hz）", .2, 3, .01],
  ["phase_gain", "相位增益", .1, 20, .1],
  ["frequency_gain", "频率增益", .1, 30, .1],
  ["amplitude_gain", "幅值增益", .1, 10, .1],
  ["input_scale_deg", "角度归一化尺度（°）", 5, 100, 1],
  ["maximum_dt_s", "最大输入间隔（s）", .01, .03, .001],
  ["velocity_filter_tau_s", "速度滤波时间常数（s）", 0, .1, .001],
  ["event_velocity_deg_s", "峰事件速度阈值（°/s）", 1, 30, .1],
  ["event_excursion_deg", "峰事件角度幅度（°）", 1, 30, .1],
  ["correction_rate_rad_s", "相位偏置校正速率（rad/s）", .05, 2, .05],
] as const;
export const LEG_FIELDS = [
  ["enabled", "参与建议计算（0关 / 1开）", 0, 1, 1],
  ["strength", "本侧强度系数", 0, 1, .01],
  ["flex_amplitude_nm", "屈髋峰值（N·m）", 0, 15, .01],
  ["flex_peak_phase", "屈髋峰相位（周期）", 0, 1, .01],
  ["flex_width", "屈髋宽度（周期）", .02, .3, .01],
  ["ext_amplitude_nm", "伸髋峰值幅度（N·m）", 0, 15, .01],
  ["ext_peak_phase", "伸髋峰相位（周期）", 0, 1, .01],
  ["ext_width", "伸髋宽度（周期）", .02, .3, .01],
  ["torque_limit_nm", "建议力矩上限（N·m）", 0, 15, .01],
  ["slew_nm_s", "双向变化率（N·m/s）", 0, 250, .1],
  ["positive_power_w", "正向功率上限（W）", 0, 100, .1],
  ["negative_power_w", "吸收功率上限（W）", 0, 100, .1],
  ["minimum_angle_deg", "最小角度（°）", -90, 90, .1],
  ["maximum_angle_deg", "最大角度（°）", -90, 120, .1],
  ["maximum_velocity_deg_s", "最大速度（°/s）", 0, 500, 1],
  ["fade_s", "渐变时间（s）", .1, 10, .1],
  ["transition_s", "换参过渡（s）", .1, 10, .1],
] as const;
export type AdaptiveConfig = {
  schema: number; overall_strength: number;
  afo: Record<typeof AFO_FIELDS[number][0], number>;
  left: Record<typeof LEG_FIELDS[number][0], number>;
  right: Record<typeof LEG_FIELDS[number][0], number>;
};
export function adaptiveDefaults(): AdaptiveConfig {
  const leg = {enabled:0,strength:0,flex_amplitude_nm:0,flex_peak_phase:0,flex_width:.1,
    ext_amplitude_nm:0,ext_peak_phase:0,ext_width:.1,torque_limit_nm:0,slew_nm_s:0,
    positive_power_w:0,negative_power_w:0,minimum_angle_deg:0,maximum_angle_deg:0,
    maximum_velocity_deg_s:0,fade_s:1,transition_s:1};
  return {schema:1,overall_strength:0,afo:{minimum_hz:.3,maximum_hz:2.5,initial_hz:1,
    phase_gain:4,frequency_gain:8,amplitude_gain:3,input_scale_deg:20,maximum_dt_s:.03,
    velocity_filter_tau_s:.025,event_velocity_deg_s:3,event_excursion_deg:5,correction_rate_rad_s:.8},
    left:{...leg},right:{...leg}};
}
const object = (v: unknown): v is Record<string, unknown> => !!v && typeof v === "object" && !Array.isArray(v);
export function adaptiveTrialPreset(): AdaptiveConfig {
  const c=adaptiveDefaults();c.overall_strength=1;
  for(const side of ["left","right"] as const)Object.assign(c[side],{
    enabled:1,strength:1,flex_amplitude_nm:5,flex_peak_phase:.75,flex_width:.1,
    ext_amplitude_nm:5,ext_peak_phase:.25,ext_width:.1,torque_limit_nm:5,
    slew_nm_s:1,positive_power_w:2,negative_power_w:1,
    minimum_angle_deg:-20,maximum_angle_deg:20,maximum_velocity_deg_s:60,fade_s:3,transition_s:3,
  });
  return c;
}
export function adaptiveErrors(value: unknown): string[] {
  if (!object(value)) return ["配置必须是JSON对象"];
  const errors: string[] = [];
  const exact = (v: Record<string, unknown>, keys: readonly string[]) => Object.keys(v).length === keys.length && keys.every(k => Object.hasOwn(v,k));
  if (!exact(value,["schema","overall_strength","afo","left","right"]) || value.schema !== 1) errors.push("需要schema 1及完整配置字段");
  const inRange = (v: unknown, lo: number, hi: number) => typeof v === "number" && Number.isFinite(v) && v >= lo && v <= hi;
  if (!inRange(value.overall_strength,0,1)) errors.push("总体强度需要0–1的有限数值");
  for (const group of ["afo","left","right"] as const) {
    const v = value[group], fields = group === "afo" ? AFO_FIELDS : LEG_FIELDS;
    if (!object(v) || !exact(v,fields.map(f=>f[0]))) { errors.push(`${group}字段不完整或包含未知字段`); continue; }
    for (const [key,label,min,max] of fields) if (!inRange(v[key],min,max)) errors.push(`${group} ${label}：需要${min}–${max}的有限数值`);
    if (group === "afo") {
      if (!(Number(v.minimum_hz) < Number(v.maximum_hz)) || !inRange(v.initial_hz,Number(v.minimum_hz),Number(v.maximum_hz))) errors.push("频率必须满足最低 < 最高，且初始频率在两者之间");
    } else {
      if (v.enabled !== 0 && v.enabled !== 1) errors.push(`${group} enabled只能为0或1`);
      if (v.enabled === 1 && (!(Number(v.minimum_angle_deg) < Number(v.maximum_angle_deg)) ||
        ["torque_limit_nm","slew_nm_s","positive_power_w","negative_power_w","maximum_velocity_deg_s"].some(k=>!(Number(v[k])>0)))) errors.push(`${group}开启建议计算前，需设置非空角度窗口及正值力矩、变化率、功率和速度上限`);
    }
  }
  return errors;
}
export function parseAdaptiveConfig(text: string): AdaptiveConfig {
  const value: unknown = JSON.parse(text), errors = adaptiveErrors(value);
  if (errors.length) throw new Error(errors.join("；"));
  return value as AdaptiveConfig;
}
export function encodeAdaptiveConfig(config: AdaptiveConfig): Uint8Array<ArrayBuffer> {
  const errors=adaptiveErrors(config); if(errors.length) throw new Error(errors.join("；"));
  const bytes=new Uint8Array(196), view=new DataView(bytes.buffer);
  view.setUint32(0,1,true);
  const values=[config.overall_strength,...AFO_FIELDS.map(([k])=>config.afo[k]),...LEG_FIELDS.map(([k])=>config.left[k]),...LEG_FIELDS.map(([k])=>config.right[k])];
  values.forEach((v,i)=>view.setFloat32(4+4*i,v,true));
  let crc=0xffffffff;
  for(const b of bytes.subarray(0,192)) { crc^=b; for(let bit=0;bit<8;bit++) crc=(crc>>>1)^((crc&1)?0xedb88320:0); }
  view.setUint32(192,(~crc)>>>0,true);
  return bytes;
}
export function adaptiveCommands(config: AdaptiveConfig): string {
  encodeAdaptiveConfig(config);
  // 固件每条set均校验完整结构；先关闭建议，临时扩大频率区间以避免合法配置过渡被拒绝。
  const lines=["# IDLE台架配置；逐条执行并检查ESP_OK。此文件不会自动发送。",
    "# 首条恢复默认会替换当前自适应RAM配置；不保存NVS、不START、不采集零位。",
    "adaptive_store defaults","adaptive_set afo minimum_hz 0.2","adaptive_set afo maximum_hz 3",
    `adaptive_set afo initial_hz ${config.afo.initial_hz}`];
  for(const [k] of AFO_FIELDS) if(k!=="initial_hz") lines.push(`adaptive_set afo ${k} ${config.afo[k]}`);
  for(const side of ["left","right"] as const) {
    for(const [k] of LEG_FIELDS) if(k!=="enabled") lines.push(`adaptive_set ${side} ${k} ${config[side][k]}`);
    lines.push(`adaptive_set ${side} enabled ${config[side].enabled}`);
  }
  lines.push(`adaptive_set global overall_strength ${config.overall_strength}`,"adaptive_config");
  return lines.join("\n")+"\n";
}
export function gaussianPreview(config: AdaptiveConfig, side: "left"|"right", phase: number): number {
  const leg=config[side];
  const g=(peak:number,width:number)=>Math.exp((Math.cos(2*Math.PI*(phase-peak))-1)/(2*Math.PI*width)**2);
  return leg.enabled*config.overall_strength*leg.strength*(leg.flex_amplitude_nm*g(leg.flex_peak_phase,leg.flex_width)-leg.ext_amplitude_nm*g(leg.ext_peak_phase,leg.ext_width));
}
export const ADAPTIVE_COLUMNS = "timestamp_us,side,system_state,q_deg,v_deg_s,motor_estimate_nm,feedback_valid,raw_phase,corrected_phase,hz,period_s,confidence,event,locked,flex_nm,ext_nm,desired_nm,limited_preview_nm,final_ff_command_nm,command_valid,faults,rearm,generation,execution_us,dropped,event_timestamp_us,event_phase_before,event_phase_after,phase_offset_rad,accepted_events,rejected_events,lambda,assist_enabled,previous_control_execution_us".split(",");
export type AdaptiveRow = Record<string,number>;
export function parseAdaptiveLog(text: string): {rows: AdaptiveRow[]; malformed: number; resets: number} {
  const rows:AdaptiveRow[]=[], last=[-1,-1]; let malformed=0,resets=0,csv=false;
  for(const line of text.split(/\r?\n/)) {
    const trimmed=line.trim();
    if(trimmed===ADAPTIVE_COLUMNS.join(",")) { csv=true; continue; }
    const at=line.indexOf("ADAPT,");
    if(at<0 && (!csv||!trimmed)) continue;
    const parts=(at>=0?line.slice(at+6):trimmed).trim().split(",");
    const values=parts.map(x=>x.trim()===""?NaN:Number(x));
    if(values.length!==ADAPTIVE_COLUMNS.length || values.some(x=>!Number.isFinite(x)) ||
      ![0,1].includes(values[1]) || !Number.isSafeInteger(values[0]) || values[0]<0 ||
      [6,12,13,19,21,32].some(i=>![0,1].includes(values[i])) || values[11]<0 || values[11]>1 ||
      values[7]<0 || values[7]>1 || values[8]<0 || values[8]>1) {malformed++;continue;}
    const side=values[1];
    if(values[0]<last[side]) { resets++; last[0]=-1;last[1]=-1; }
    last[side]=values[0];
    rows.push(Object.fromEntries(ADAPTIVE_COLUMNS.map((k,i)=>[k,values[i]])));
  }
  return {rows,malformed,resets};
}

/** capability位已占满：策略5按已知版本与原助力配置能力共同判定。 */
export function supportsAdaptive(firmware: readonly number[] = [], capabilities=0): boolean {
  return ["0.11.0","0.11.1","0.11.2","0.11.3", "0.11.4", "0.11.5", "0.11.6", "0.11.7", "0.11.8", "0.11.9", "0.11.10", "0.11.11", "0.11.12", "0.11.13", "0.11.14", "0.11.15", "0.11.16", "0.11.17", "0.11.18", "0.11.19", "0.11.20", "0.11.21", "0.11.22", "0.11.23"].includes(firmware.join(".")) && (capabilities & (1<<5))!==0;
}
