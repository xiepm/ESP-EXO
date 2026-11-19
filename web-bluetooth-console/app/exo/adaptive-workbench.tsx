"use client";

import { useMemo, useRef, useState } from "react";
import AdaptiveCapture from "./adaptive-capture";
import AdaptiveDirectionChart from "./adaptive-direction-chart";
import { chartLog, type AdaptiveChartSource, type AdaptiveChartView } from "@/lib/exo-adaptive-chart";
import { AFO_FIELDS, LEG_FIELDS, adaptiveDefaults, adaptiveErrors, parseAdaptiveConfig,
  encodeAdaptiveConfig, adaptiveCommands, gaussianPreview, parseAdaptiveLog,
  type AdaptiveConfig,adaptiveTrialPreset } from "@/lib/exo-adaptive";

function download(name: string, data: string | Uint8Array<ArrayBuffer>) {
  const url=URL.createObjectURL(new Blob([data]));
  const anchor=document.createElement("a");anchor.href=url;anchor.download=name;anchor.click();
  setTimeout(()=>URL.revokeObjectURL(url),1000);
}

export default function AdaptiveWorkbench({ onChartView }: { onChartView?: (view: AdaptiveChartView) => void }) {
  const [config,setConfig]=useState<AdaptiveConfig>(adaptiveDefaults);
  const [message,setMessage]=useState("");
  const [log,setLog]=useState<ReturnType<typeof parseAdaptiveLog>|null>(null);
  const [logName,setLogName]=useState("");
  const [index,setIndex]=useState(0);
  const [chart,setChart]=useState<AdaptiveChartSource|null>(null);
  const [chartError,setChartError]=useState("");
  const configLoad=useRef(0),logLoad=useRef(0);
  const errors=useMemo(()=>adaptiveErrors(config),[config]);
  const preview=useMemo(()=>errors.length?[]:Array.from({length:121},(_,i)=>({
    x:i/120,left:gaussianPreview(config,"left",i/120),right:gaussianPreview(config,"right",i/120),
  })),[config,errors.length]);
  const scale=Math.max(.01,...preview.flatMap(p=>[Math.abs(p.left),Math.abs(p.right)]));
  const record=log?.rows[index];
  async function importConfig(file?: File) {
    if(!file)return; const token=++configLoad.current;
    try {
      if(file.size>64000)throw new Error("配置文件不得超过64KB");
      const next=parseAdaptiveConfig(await file.text());
      if(token!==configLoad.current)return;
      setConfig(next);setMessage(`已导入 ${file.name}，仅更新本地草稿，尚未写入设备。`);
    } catch(e) { if(token===configLoad.current)setMessage(e instanceof Error?e.message:"配置读取失败"); }
  }
  async function importLog(file?: File) {
    if(!file)return;const token=++logLoad.current;
    try {
      if(file.size>8*1024*1024)throw new Error("日志文件不得超过8MB，请按采集时段拆分");
      const text=await file.text(), next=parseAdaptiveLog(text);
      if(token!==logLoad.current)return;
      if(!next.rows.length)throw new Error(`没有有效AFO记录，拒绝行${next.malformed}；需要adaptive_monitor的CSV或含ADAPT行的UART日志。`);
      setLog(next);setLogName(file.name);setIndex(next.rows.length-1);setMessage("日志已载入。这是离线记录，不代表当前设备状态。");
      try {setChart(chartLog(text,file.name));setChartError("");}
      catch(e) {setChart(null);setChartError(e instanceof Error?e.message:"图表时序核对失败");}
    } catch(e) { if(token===logLoad.current){setLog(null);setLogName("");setChart(null);setChartError("");setMessage(e instanceof Error?e.message:"日志读取失败");} }
  }
  function setField(group:"afo"|"left"|"right",key:string,value:string) {
    configLoad.current++;
    setConfig(c=>({...c,[group]:{...c[group],[key]:value.trim()===""?NaN:Number(value)}}));
  }
  return <article className="exo-card exo-adaptive-workbench" aria-label="AFO参数与日志工作台">
    <div className="exo-card-heading"><div><p className="exo-kicker">自适应双髋 · 参数草稿</p><h2>AFO参数与日志工作台</h2></div>
      <span className="exo-state-chip warn">本地编辑 · 尚未写入设备</span></div>
    <p>策略选择、执行态和等级使用上方BLE设置。0.11.6受限试验配置1可通过上方按钮准备并核对；下方自定义参数仍为本地草稿，导出后通过UART工程流程应用并读回。</p>
    <p className="exo-operation-note">参数草稿只在本地编辑；下方USB采集由人工启动，只读取诊断记录。设备是否允许ACTIVE及其名义上限，请以上方本次连接的读回为准。</p>
    <AdaptiveCapture onChartView={onChartView} />
    <details className="exo-advanced"><summary>左右双高斯与AFO参数草稿</summary>
      <p>默认双侧关闭、强度与力矩为0。数值上限是软件允许的编码范围，不能作为人体安全参数。恢复默认只影响本地草稿。</p>
      <div className="exo-export-row">
        <label className="exo-adaptive-file">导入配置JSON<input aria-label="导入AFO配置JSON" type="file" accept=".json" onChange={e=>{void importConfig(e.target.files?.[0]);e.target.value="";}}/></label>
        <button className="exo-button ghost" onClick={()=>{configLoad.current++;setConfig(adaptiveDefaults());setMessage("本地草稿已恢复默认，设备参数未改变。");}}>恢复本地默认</button>
        <button className="exo-button ghost" onClick={()=>{configLoad.current++;setConfig(adaptiveTrialPreset());setMessage("已载入配置1供查看与导出；写入使用上方受限试验参数按钮。");}}>查看5 N·m受限试验参数</button>
      </div>
      <label className="exo-adaptive-overall">总体强度系数（0–1）<input aria-label="AFO总体强度系数" type="number" min="0" max="1" step="0.01" value={Number.isFinite(config.overall_strength)?config.overall_strength:""}
        onChange={e=>{configLoad.current++;setConfig(c=>({...c,overall_strength:e.target.value===""?NaN:Number(e.target.value)}));}}/></label>
      {(["afo","left","right"] as const).map(group=><fieldset key={group} className="exo-adaptive-fields">
        <legend>{group==="afo"?"AFO与同侧屈髋峰事件":group==="left"?"LEFT · 左髋独立参数":"RIGHT · 右髋独立参数"}</legend>
        <div className="exo-pulse-parameters">{(group==="afo"?AFO_FIELDS:LEG_FIELDS).map(([key,label,min,max,step])=>{
          const value=(config[group] as Record<string,number>)[key];
          return <label key={key}><span>{label}</span><input aria-label={`${group} ${label}`} type="number" min={min} max={max} step={step}
            value={Number.isFinite(value)?value:""} onChange={e=>setField(group,key,e.target.value)}/></label>;
        })}</div>
      </fieldset>)}
      {!!errors.length && <div className="exo-adaptive-errors" role="alert"><strong>草稿未通过校验，导出已禁用</strong><ul>{errors.map(e=><li key={e}>{e}</li>)}</ul></div>}
      <figure className="exo-adaptive-preview"><figcaption>双高斯规划曲线预览 · 左蓝 / 右橙 · 相位0为本侧最大屈髋峰</figcaption>
        <svg viewBox="0 0 640 220" role="img" aria-label="本地双高斯规划曲线，不是实测助力">
          <line x1="45" y1="110" x2="620" y2="110" stroke="#bacbd1"/><line x1="45" y1="20" x2="45" y2="195" stroke="#bacbd1"/>
          <text x="45" y="15" fontSize="12">±{scale.toFixed(3)} N·m</text><text x="40" y="215" fontSize="12">0</text><text x="585" y="215" fontSize="12">1周期</text>
          {(["left","right"] as const).map(side=><polyline key={side} fill="none" stroke={side==="left"?"#167eae":"#c65e26"} strokeWidth="2.5" points={preview.map(p=>`${45+575*p.x},${110-80*p[side]/scale}`).join(" ")}/>)}
          {errors.length>0 && <text x="190" y="65">参数无效，预览暂停</text>}
        </svg>
        <p>预览假定渐入系数为1，未计算动态限幅、功率窗口或BLE建议等级；不代表最终CAN命令或实测力矩。零幅值时曲线为0。</p>
      </figure>
      <div className="exo-export-row">
        <button className="exo-button secondary" disabled={!!errors.length} onClick={()=>download("adaptive_assist.json",JSON.stringify(config,null,2)+"\n")}>导出配置JSON</button>
        <button className="exo-button secondary" disabled={!!errors.length} onClick={()=>download("adaptive_config.bin",encodeAdaptiveConfig(config))}>导出196字节配置</button>
        <button className="exo-button secondary" disabled={!!errors.length} onClick={()=>download("adaptive_commands.txt",adaptiveCommands(config))}>导出UART命令清单</button>
      </div>
      <p className="exo-operation-note">配置.bin不是固件，不能刷入应用区。命令清单首条恢复自适应RAM默认值，需在IDLE逐条执行并核对返回；不含START、归零或NVS保存。导出本身不会连接设备。</p>
    </details>
    <details className="exo-advanced" open><summary>AFO日志查看 · 与原HIP_EVENT_TBE分开</summary>
      <p>导入adaptive_monitor生成的CSV或UART原始.log；保留采样时间和丢弃计数。主页面原步态相位来自HIP_EVENT_TBE，不是本面板的AFO相位。</p>
      <label className="exo-adaptive-file">导入AFO日志<input aria-label="导入AFO日志" type="file" accept=".csv,.log,.txt" onChange={e=>{void importLog(e.target.files?.[0]);e.target.value="";}}/></label>
      {chartError && <p role="alert">{chartError}</p>}
      {chart && onChartView && <button className="exo-button secondary" onClick={() => onChartView({ source: chart, live: false })}>在实时数据中查看此历史图</button>}
      {chart && <AdaptiveDirectionChart key={logName} source={chart} />}
      {!log && <p className="exo-capability-empty">尚未导入AFO记录。这里不会用旧步态或模拟数据填充实测读数。</p>}
      {log && record && <>
        <p role="status">离线文件：{logName} · {log.rows.length}行 · 拒绝{log.malformed}行 · 原文读取顺序回退{log.resets}次（缓存读回可乱序，不据此判定重启；图表独立核对Boot）</p>
        <label className="exo-adaptive-scrubber">记录 {index+1}/{log.rows.length}<input aria-label="AFO日志记录位置" type="range" min="0" max={log.rows.length-1} value={index} onChange={e=>setIndex(Number(e.target.value))}/></label>
        <div className="exo-adaptive-readings">
          {[
            ["侧别 / MCU时间",`${record.side===0?"LEFT":"RIGHT"} / ${(record.timestamp_us/1e6).toFixed(3)} s`],
            ["反馈 / 锁定",`${record.feedback_valid?"有效":"无效"} / ${record.locked?"已锁定":"未锁定"}`],
            ["AFO原始 / 校正相位",record.feedback_valid?`${record.raw_phase.toFixed(4)} / ${record.corrected_phase.toFixed(4)}`:"—"],
            ["频率 / 周期",record.feedback_valid?`${record.hz.toFixed(3)} Hz / ${record.period_s.toFixed(3)} s`:"—"],
            ["规划 / 安全层建议",`${record.desired_nm.toFixed(4)} / ${record.limited_preview_nm.toFixed(4)} N·m`],
            ["基础命令最终FF",record.command_valid?`${record.final_ff_command_nm.toFixed(4)} N·m`:"未发送有效命令"],
            ["电机估计力矩",record.feedback_valid?`${record.motor_estimate_nm.toFixed(4)} N·m`:"—"],
            ["接受 / 拒绝事件",`${record.accepted_events} / ${record.rejected_events}`],
            ["故障位 / 重新授权",`0x${record.faults.toString(16)} / ${record.rearm?"需要":"无请求"}`],
            ["配置代次 / 丢弃计数",`${record.generation} / ${record.dropped}`],
            ["算法 / 前周期耗时",`${record.execution_us} / ${record.previous_control_execution_us} μs`],
          ].map(([label,value])=><div key={label}><span>{label}</span><strong>{value}</strong></div>)}
        </div>
        <p className="exo-operation-note">FF是基础五元命令的前馈分量；电机估计力矩不是独立测力结果。日志中的锁定不代表已获准输出助力。</p>
      </>}
    </details>
    {message && <p role="status" className="exo-operation-note">{message}</p>}
  </article>;
}
