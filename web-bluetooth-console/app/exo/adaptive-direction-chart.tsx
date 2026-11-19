"use client";

import { useMemo, useState } from "react";
import { chartSegments, directionPoints, type AdaptiveChartSource, type DirectionPoint } from "@/lib/exo-adaptive-chart";
import { usableAdaptiveInput } from "@/lib/exo-adaptive-capture";

type Series = { name: string; color: string; dashed?: boolean; value: (p: DirectionPoint) => number | null };
const format = (v: number | null | undefined, digits = 2) => v === null || v === undefined ? "—" : v.toFixed(digits);

export default function AdaptiveDirectionChart({ source, live = false, lastReceived, active = false }:
  { source: AdaptiveChartSource; live?: boolean; lastReceived?: number; active?: boolean }) {
  const [side, setSide] = useState(0), [windowSeconds, setWindowSeconds] = useState(10);
  const [amplitude, setAmplitude] = useState(5), [preview, setPreview] = useState(true);
  const [endTime, setEndTime] = useState<number | null>(null), [cursor, setCursor] = useState<number | null>(null);
  const [paused, setPaused] = useState(false);
  const points = useMemo(() => directionPoints(source.rows, side, amplitude), [source.rows, side, amplitude]);
  const first = points[0]?.row.timestamp_us ?? 0, last = points.at(-1)?.row.timestamp_us ?? first;
  const end = endTime === null ? last : Math.max(first, Math.min(last, endTime));
  const start = Math.max(first, end - windowSeconds * 1e6), span = Math.max(1, end - start);
  const visible = points.filter(p => p.row.timestamp_us >= start && p.row.timestamp_us <= end);
  const selected = cursor === null ? visible.at(-1) : visible.reduce<DirectionPoint | undefined>((best, p) =>
    !best || Math.abs(p.row.timestamp_us - cursor) < Math.abs(best.row.timestamp_us - cursor) ? p : best, undefined);
  const x = (time: number) => 66 + 690 * (time - start) / span;
  const lossEvents = source.events.filter(e => e.side === side && e.timestamp_us >= start && e.timestamp_us <= end);
  const opposed = visible.filter(p => p.opposed);
  const rows = [
    { title: "髋角度（°）", series: [{ name: "角度", color: "#136e9a", value: (p: DirectionPoint) => p.row.q_deg }] },
    { title: "速度（°/s）", series: [
      { name: "电机报告速度", color: "#136e9a", value: (p: DirectionPoint) => p.row.v_deg_s },
      { name: "角度中心割线（辅助）", color: "#64748b", dashed: true, value: (p: DirectionPoint) => p.secant }] },
    { title: "AFO相位（周期）", phase: true, series: [
      { name: "校正相位", color: "#8052a4", value: (p: DirectionPoint) => p.row.corrected_phase },
      { name: "原始相位", color: "#9299aa", dashed: true, value: (p: DirectionPoint) => p.row.raw_phase }] },
    { title: "力矩（N·m）", series: [
      { name: "最终FF指令（有效时）", color: "#1260b0", value: (p: DirectionPoint) => p.row.command_valid ? p.row.final_ff_command_nm : null },
      { name: "电机报告估算扭矩", color: "#b15a1e", value: (p: DirectionPoint) => p.row.motor_estimate_nm },
      ...(preview ? [{ name: `${amplitude} N·m假设曲线 · 未下发`, color: "#0c857b", dashed: true, value: (p: DirectionPoint) => p.projection }] : [])] },
  ];
  return <section className="exo-direction-chart" aria-label="AFO同时间轴方向对照图">
    <div className="exo-card-heading"><h3>AFO同时间轴方向对照图</h3>
      <span className="exo-state-chip">{live ? !points.length ? "等待USB只读数据" : active ? paused ? "暂停显示 · 采集继续" : "USB只读采集中" : "采集已结束 · 历史记录" : "历史文件 · 非当前状态"}</span></div>
    <p>{source.name} · Boot {source.boot ?? "文件未注明"} · 最近记录的MCU时刻 {(last / 1e6).toFixed(3)} s
      {live && lastReceived !== undefined ? ` · 最近接收批次 ${new Date(lastReceived).toLocaleTimeString()}` : ""}</p>
    <div className="exo-direction-controls">
      <label>侧别<select value={side} onChange={e => { setSide(Number(e.target.value)); setCursor(null); }}><option value={0}>左髋</option><option value={1}>右髋</option></select></label>
      <label>时间窗口<select value={windowSeconds} onChange={e => setWindowSeconds(Number(e.target.value))}>{[5, 10, 20, 60].map(s => <option value={s} key={s}>{s}秒</option>)}</select></label>
      <label><input type="checkbox" checked={preview} onChange={e => setPreview(e.target.checked)} />显示假设曲线</label>
      <label>仅图中比较幅值（N·m）<input aria-label="仅图中比较幅值" type="number" min={0} max={15} step={.1} value={amplitude}
        onChange={e => { const v = Number(e.target.value); if (Number.isFinite(v) && v >= 0 && v <= 15) setAmplitude(v); }} /></label>
      {live && <button className="exo-button secondary" onClick={() => { setPaused(!paused); setEndTime(paused ? null : end); }}>{paused ? "继续跟随最新" : "暂停图表（继续采集）"}</button>}
    </div>
    <p className="exo-operation-note">比较幅值只改变虚线，不写入设备。屈峰0.75、伸峰0.25、宽0.10；只在有效且锁定的记录上投影，不包含淡入、限幅或CAN输出。</p>
    {!points.length ? <p>等待本段AFO数据；尚无曲线，不使用模拟数据填充。</p> : <>
      <label className="exo-adaptive-scrubber">查看历史窗口<input aria-label="AFO图表窗口位置" type="range" min={first} max={Math.max(first + 1, last)} step={10000} value={end}
        onChange={e => { setEndTime(Number(e.target.value)); setPaused(true); setCursor(null); }} /></label>
      {rows.map(panel => {
        const series: Series[] = panel.series;
        const values = visible.flatMap(p => usableAdaptiveInput(p.row) ? series.flatMap(s => { const v = s.value(p); return v === null ? [] : [v]; }) : []);
        const low = panel.phase ? 0 : Math.min(0, ...values), high = panel.phase ? 1 : Math.max(.001, ...values);
        const padding = panel.phase ? 0 : Math.max(.05, (high - low) * .08), min = low - padding, max = high + padding;
        const y = (v: number) => 125 - 105 * (v - min) / Math.max(.001, max - min);
        return <figure key={panel.title}>
          <figcaption><strong>{panel.title}</strong>{series.map(s => <span key={s.name} style={{ color: s.color }}>{s.dashed ? "┄ " : "━ "}{s.name}</span>)}</figcaption>
          <svg viewBox="0 0 780 155" role="img" aria-label={panel.title + "同时间轴对照"} onPointerMove={e => {
            const bounds = e.currentTarget.getBoundingClientRect();
            setCursor(start + Math.max(0, Math.min(1, ((e.clientX - bounds.left) * 780 / bounds.width - 66) / 690)) * span);
          }}>
            {[min, (min + max) / 2, max].map(v => <g key={v}><line x1={66} x2={756} y1={y(v)} y2={y(v)} stroke="#e1e7ec" /><text x={59} y={y(v) + 4} textAnchor="end">{v.toFixed(panel.phase ? 2 : 1)}</text></g>)}
            {[0, 1, 2, 3, 4].map(i => <text key={i} x={66 + 690 * i / 4} y={148} textAnchor="middle">{((start + span * i / 4) / 1e6).toFixed(2)}s</text>)}
            {visible.filter(p => !p.row.locked).map(p => <line key={p.row.timestamp_us} x1={x(p.row.timestamp_us)} x2={x(p.row.timestamp_us)} y1={16} y2={128} stroke="#a8b2bd" strokeOpacity={.16} strokeWidth={3} />)}
            {lossEvents.map(e => <line key={`${e.sequence}:${e.side}`} x1={x(e.timestamp_us)} x2={x(e.timestamp_us)} y1={16} y2={128} stroke="#c3423e" strokeDasharray="4 3"><title>精确失锁 MCU {(e.timestamp_us / 1e6).toFixed(6)}s · 原因位 {e.values[6]}</title></line>)}
            {series.map(s => <g key={s.name}>{chartSegments(visible, s.value, panel.phase, lossEvents.map(e => e.timestamp_us)).map((segment, i) => <polyline key={i} fill="none" stroke={s.color} strokeWidth={1.7} strokeDasharray={s.dashed ? "5 3" : undefined}
              points={segment.map(p => `${x(p.row.timestamp_us)},${y(s.value(p)!)}`).join(" ")} />)}</g>)}
            {preview && panel.title === "力矩（N·m）" && opposed.map(p => <circle key={p.row.timestamp_us} cx={x(p.row.timestamp_us)} cy={y(p.projection!)} r={4} fill="white" stroke="#c3423e" strokeWidth={1.7} />)}
            {selected && <line x1={x(selected.row.timestamp_us)} x2={x(selected.row.timestamp_us)} y1={12} y2={129} stroke="#344c60" strokeDasharray="2 2" />}
          </svg>
        </figure>;
      })}
      {selected && <div className="exo-direction-detail"><strong>对照点：MCU {(selected.row.timestamp_us / 1e6).toFixed(6)} s · {selected.row.locked ? "已锁定" : "未锁定"} · {usableAdaptiveInput(selected.row) ? "输入有效" : "输入无效，读数不解读"}</strong>
        {usableAdaptiveInput(selected.row) && <p>角度 {format(selected.row.q_deg)}°；报告速度 {format(selected.row.v_deg_s)}°/s；中心割线 {format(selected.secant)}°/s；校正相位 {format(selected.row.corrected_phase, 4)}。<br />
          最终FF：{selected.row.command_valid ? `${format(selected.row.final_ff_command_nm, 4)} N·m` : "命令无效／未授权，不画作已下发零指令"}；电机报告 {format(selected.row.motor_estimate_nm, 4)} N·m；假设曲线 {preview ? format(selected.projection, 4) : "隐藏"} N·m。<br />
          {selected.opposed && preview ? `投影与报告速度异号；角度割线${selected.positionSupport === "supported" ? "支持" : selected.positionSupport === "disagrees" ? "不支持" : "无法确定"}${selected.crossesTurn ? "；相邻区间跨越换向" : ""}。` : "此点无高幅投影异号标记；不代表助力方向验收通过。"}</p>}</div>}
      <p>窗口内高幅投影异号标记 {preview ? opposed.length : "已隐藏"} 个；精确失锁 {lossEvents.length} 次。灰线为未锁定快照，红虚线为失锁时刻。横轴为MCU时间。</p>
    </>}
    <p className="exo-operation-note">图表使用约10Hz记录，USB按批次更新，不能捕获全部100Hz瞬态或充当保护。红圈仅筛选|报告速度|≥3°/s且|假设力矩|≥比较幅值50%的异号点，低于门槛不等于无反向力矩。允许吸收做功时，异号点本身不代表故障。角度割线不是独立传感器；电机报告扭矩不是独立实测关节力矩。缺失、无效、换参及相位绕回处断线。</p>
  </section>;
}
