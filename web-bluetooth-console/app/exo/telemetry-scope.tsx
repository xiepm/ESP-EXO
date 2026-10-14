"use client";
import { useMemo, useState } from "react";
import type { TelemetrySamplePoint } from "@/lib/exo-telemetry-stream";
import { SCOPE_CHANNELS, SCOPE_PLOT, finiteScopeValue, scopeLatest, scopeNearest, scopePaths,
  scopeRange, scopeWindow, scopeX, scopeY } from "@/lib/exo-scope";

export default function TelemetryScope({samples, windowSeconds, rateHz, channelMask, autoScale, showLeft, showRight,
  paused, reception}: {samples: readonly TelemetrySamplePoint[];windowSeconds:number;rateHz:number;
  channelMask:number;autoScale:boolean;showLeft:boolean;showRight:boolean;paused:boolean;reception:string}) {
  const points=useMemo(()=>scopeWindow(samples,windowSeconds),[samples,windowSeconds]);
  const [cursor,setCursor]=useState<number|null>(null);
  const [expanded,setExpanded]=useState<number|null>(null);
  const format=(value:number|null|undefined)=>{const v=finiteScopeValue(value);return v===null?"—":v.toFixed(2);};
  return <div className="exo-scope-grid" aria-label="左右髋实时示波器">
    {SCOPE_CHANNELS.filter(c=>c.bit!==4 || (channelMask&4)!==0).map(channel=>{
      const selected=(channelMask&channel.bit)!==0;
      const range=scopeRange(points,channel,autoScale);
      const latest=scopeLatest(points,channel);
      const read=cursor===null?latest:scopeNearest(points,channel,cursor,rateHz);
      const pathsLeft=scopePaths(points,channel,"left",windowSeconds,range,rateHz);
      const pathsRight=scopePaths(points,channel,"right",windowSeconds,range,rateHz);
      const hasValues=points.some(p=>finiteScopeValue(p.sample[channel.left])!==null || finiteScopeValue(p.sample[channel.right])!==null);
      const isInvalid=latest && finiteScopeValue(latest.sample[channel.left])===null && finiteScopeValue(latest.sample[channel.right])===null;
      const message=!selected?"此通道未采集 · 在通道设置中启用":!points.length?"等待实时数据":!hasValues||isInvalid?channel.unavailable:null;
      return <article className={`exo-scope-chart ${expanded===channel.bit?"expanded":""}`} key={channel.bit} aria-label={`${channel.title}波形`}>
        <header><div><h3>{channel.title}</h3><span>{channel.unit}{cursor!==null?` · 游标 ${cursor.toFixed(2)} s`:paused?" · 冻结采样":reception==="live"?" · 最新采样":points.length?" · 历史采样":" · 等待采样"}</span></div>
          <div className="exo-scope-readings">
            {showLeft&&<b className="left">L <strong>{format(read?.sample[channel.left])}</strong></b>}
            {showRight&&<b className="right">R <strong>{format(read?.sample[channel.right])}</strong></b>}
          </div><button type="button" className="exo-scope-expand" aria-label={`${expanded===channel.bit?"收起":"展开"}${channel.title}图`} onClick={()=>setExpanded(expanded===channel.bit?null:channel.bit)}>{expanded===channel.bit?"收起":"展开"}</button>
        </header>
        <div className="exo-scope-screen">
          <svg viewBox={`0 0 ${SCOPE_PLOT.width} ${SCOPE_PLOT.height}`} role="img" aria-label={`${channel.title}最近${windowSeconds}秒左右髋波形`}
            onPointerMove={event=>{const rect=event.currentTarget.getBoundingClientRect();const x=(event.clientX-rect.left)/rect.width*SCOPE_PLOT.width;
              setCursor(Math.max(-windowSeconds,Math.min(0,(x-SCOPE_PLOT.left)/(SCOPE_PLOT.right-SCOPE_PLOT.left)*windowSeconds-windowSeconds)));}}
            onPointerLeave={()=>setCursor(null)}>
            <defs><clipPath id={`scope-clip-${channel.bit}`}><rect x={SCOPE_PLOT.left} y={SCOPE_PLOT.top} width={SCOPE_PLOT.right-SCOPE_PLOT.left} height={SCOPE_PLOT.bottom-SCOPE_PLOT.top}/></clipPath></defs>
            {[0,1,2,3,4].map(i=>{const v=range[1]-(range[1]-range[0])*i/4, y=scopeY(v,range);return <g key={`y${i}`}><line x1={SCOPE_PLOT.left} x2={SCOPE_PLOT.right} y1={y} y2={y} className="grid"/><text x={SCOPE_PLOT.left-8} y={y+4} textAnchor="end">{Math.abs(v)<0.001?"0":v.toFixed(Math.abs(v)<1?2:1)}</text></g>;})}
            {[0,1,2,3,4,5].map(i=>{const s=-windowSeconds+windowSeconds*i/5,x=scopeX(s,windowSeconds);return <g key={`x${i}`}><line x1={x} x2={x} y1={SCOPE_PLOT.top} y2={SCOPE_PLOT.bottom} className="grid"/><text x={x} y={225} textAnchor={i===0?"start":i===5?"end":"middle"}>{s===0?"0 s":`${s.toFixed(1)} s`}</text></g>;})}
            {range[0]<=0&&range[1]>=0&&<line x1={SCOPE_PLOT.left} x2={SCOPE_PLOT.right} y1={scopeY(0,range)} y2={scopeY(0,range)} className="zero"/>}
            <g clipPath={`url(#scope-clip-${channel.bit})`}>
              {showLeft&&pathsLeft.map((d,i)=><path key={`l${i}`} d={d} className="left"/>)}
              {showRight&&pathsRight.map((d,i)=><path key={`r${i}`} d={d} className="right"/>)}
              {cursor!==null&&<line x1={scopeX(cursor,windowSeconds)} x2={scopeX(cursor,windowSeconds)} y1={SCOPE_PLOT.top} y2={SCOPE_PLOT.bottom} className="cursor"/>}
            </g>
          </svg>
          {message&&<p className="exo-scope-empty">{message}</p>}
        </div>
        <footer><span>{paused?"显示冻结 · 采集继续":reception==="live"?"实时滚动":reception==="stale"?"数据中断 · 保留历史":reception==="disconnected"?"连接断开 · 历史波形":"等待采集"}</span><span>主控时间 · {points.length} 个时刻</span></footer>
      </article>;
    })}
  </div>;
}
