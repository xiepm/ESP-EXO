"use client";

import { useEffect, useRef, useState } from "react";
import { AdaptiveCaptureSession, usableAdaptiveInput, type CapturePort } from "@/lib/exo-adaptive-capture";
import { orderedChartRows, type AdaptiveChartView } from "@/lib/exo-adaptive-chart";
import AdaptiveDirectionChart from "./adaptive-direction-chart";

type SerialNavigator = Navigator & { serial?: {
  requestPort(options: { filters: { usbVendorId: number; usbProductId: number }[] }): Promise<CapturePort>;
} };
const phaseNames = { opening: "正在打开USB并接收初始日志", checking: "正在核对当前Boot、双侧有效输入和无输出状态", waiting: "USB保持连接 · 等待人工处理零位或反馈", collecting: "正在采集", closing: "正在结束并释放USB", finished: "已结束" };

function save(name: string, parts: BlobPart[], type: string) {
  const url = URL.createObjectURL(new Blob(parts, { type }));
  const a = document.createElement("a"); a.href = url; a.download = name; a.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}
function filename(session: AdaptiveCaptureSession) { return `afo_manual_${session.started.replace(/[:.]/g, "-")}`; }

export default function AdaptiveCapture({ onChartView }: { onChartView?: (view: AdaptiveChartView) => void }) {
  const current = useRef<AdaptiveCaptureSession | null>(null);
  const selecting = useRef(false);
  const mounted = useRef(true);
  const [, redraw] = useState(0);
  const [busy, setBusy] = useState(false);
  const [message, setMessage] = useState("");
  const [takes, setTakes] = useState<AdaptiveCaptureSession[]>([]);
  const [note, setNote] = useState("");
  const [observeRunning,setObserveRunning]=useState(false);
  const [session, setSession] = useState<AdaptiveCaptureSession | null>(null);
  const collecting = session?.phase === "collecting";

  useEffect(() => {
    mounted.current = true;
    const beforeUnload = (event: BeforeUnloadEvent) => {
      if (selecting.current || current.current?.phase !== undefined && current.current.phase !== "finished") {
        event.preventDefault();
      }
    };
    window.addEventListener("beforeunload", beforeUnload);
    return () => { mounted.current = false; current.current?.stop(); window.removeEventListener("beforeunload", beforeUnload); };
  }, []);

  async function start() {
    if (selecting.current || busy || takes.length >= 5) return;
    const serial = (navigator as SerialNavigator).serial;
    if (!serial) { setMessage("此浏览器不支持Web Serial，请使用电脑端Chrome或Edge打开本地页面。"); return; }
    selecting.current = true; setBusy(true); setMessage("");
    try {
      // Must remain directly inside the user's click handler; never auto-select or reconnect.
      const port = await serial.requestPort({ filters: [{ usbVendorId: 0x1a86, usbProductId: 0x55d3 }] });
      if (!mounted.current) return;
      const next = new AdaptiveCaptureSession(() => { if (mounted.current) {
        redraw(v => v + 1);
        onChartView?.({ live: true, active: next.phase === "collecting",
          lastReceived: next.lastReviewedHostIso ? Date.parse(next.lastReviewedHostIso) : undefined,
          source: { name: "本页USB采集记录", boot: next.review.boot,
            rows: orderedChartRows(next.review.rows.values()), events: [...next.review.events.values()] } });
      } },observeRunning);
      current.current = next; setSession(next);
      await next.run(port);
      if (mounted.current) { setTakes(previous => [...previous, next]); setMessage(""); }
    } catch (error) {
      if (mounted.current) setMessage(error instanceof Error ? error.message : String(error));
    } finally {
      selecting.current = false;
      if (mounted.current) setBusy(false);
    }
  }
  const latest = session?.review.latest ?? [null, null];
  const validRows = session ? [...session.review.rows.values()].filter(usableAdaptiveInput).length : 0;
  return <details className="exo-advanced exo-adaptive-capture" open>
    <summary>人工采集AFO · USB只读 · 可选5 N·m运行监测</summary>
    <p>由你点击开始、标记动作阶段、结束和导出。这里读取真实AFO相位及失锁记录，不发送START、归零或参数修改。每段最多180秒。</p>
    <p className="exo-operation-note">连接前关闭其他串口采集器。接入初期的启动日志会单独保留；两次确认当前Boot一致、处于IDLE、时间推进且双侧输入有效后开始。运行监测允许随后由主控制区人工START；重启、配置变化和输入失效仍会结束本段。结束采集不发送电机STOP。</p>
    <label><input type="checkbox" disabled={busy} checked={observeRunning} onChange={e=>setObserveRunning(e.target.checked)}/>记录后续运行状态与±5 N·m指令（只读监测）</label>
    <div className="exo-export-row">
      <button className="exo-button primary" disabled={busy || takes.length >= 5} onClick={() => void start()}>选择USB并开始只读采集</button>
      <button className="exo-button secondary" disabled={!busy || !session || session.phase === "closing" || session.phase === "finished"}
        onClick={() => { session?.stop(); setMessage("已请求结束，正在收尾读取和释放USB。"); }}>结束采集（不停止外骨骼）</button>
    </div>
    <p role="status" aria-live="polite">{busy ? session && session.phase !== "finished" ? phaseNames[session.phase] : "请选择USB串口" : session ? session.outcome : "等待人工开始，尚未连接USB"}</p>
    {session?.phase === "waiting" && <div className="exo-adaptive-notice" role="status">
      <strong>{session.readinessIssue}</strong>
      <p>保持USB连接，先在“连接”栏目连接BLE，再到“设置 → 穿戴后零位校准”人工核对反馈并重新校准。仅用于本次采集时，取消勾选“保存本次零位供下次沿用”。完成后回到这里重新核对，不需要拔插USB。</p>
      <button className="exo-button primary" onClick={() => session.recheck()}>重新核对输入并开始采集</button>
      <p>等待期间不轮询、不自动归零；开始活动等标记保持禁用。可用上方结束按钮释放USB并保留诊断。</p>
    </div>}
    {session && <>
      {(session.resetGuard?.initialCount ?? 0) > 0 && <p className="exo-operation-note">已保留接入初期的启动或异常文本，未据此判为采集中再次重启。{session.baselineConfirmed ? "当前Boot已完成两次核对。" : "当前Boot尚待核对。"}</p>}
      <p>本段Boot：{session.review.boot ?? session.preflightBoot ?? "待核对"} · 已解析 {session.review.rows.size} 行 / 输入有效 {validRows} 行 · 精确失锁 {session.review.events.size} 条 · 已收 {(session.bytes / 1024).toFixed(1)} KiB</p>
      <div className="exo-adaptive-readings">{latest.map((row, side) => <div key={side}>
        <span>{side === 0 ? "左髋" : "右髋"} · {busy ? "最近已读回快照" : "已结束的记录"}</span>
        <strong>{row ? usableAdaptiveInput(row) ? `${row.locked ? "已锁定" : "未锁定"} · 相位 ${row.corrected_phase.toFixed(3)} · ${row.hz.toFixed(2)} Hz` : "输入无效 · 不解读相位和角度占位值" : "尚无本段有效读数"}</strong>
        {row && usableAdaptiveInput(row) && <p>{(row.timestamp_us / 1e6).toFixed(3)} s · {row.q_deg.toFixed(2)}° · {row.v_deg_s.toFixed(2)}°/s</p>}
      </div>)}</div>
      <p>排除旧缓存：{session.review.historicalRows} 行 / {session.review.historicalEvents} 条失锁。ADAPT丢弃计数范围：{session.review.droppedRange?.join(" → ") ?? "暂无"}；ADLOCK事件丢弃计数范围：{session.review.eventDroppedRange?.join(" → ") ?? "无本段事件，不能据此确认零丢弃"}。</p>
    </>}
    <AdaptiveDirectionChart live active={collecting} source={{
      name: session ? "本页USB采集记录" : "尚未连接USB",
      boot: session?.review.boot ?? null,
      rows: session ? orderedChartRows(session.review.rows.values()) : [],
      events: session ? [...session.review.events.values()] : [],
    }} lastReceived={session?.lastReviewedHostIso ? Date.parse(session.lastReviewedHostIso) : undefined} />
    <div className="exo-export-row">{["开始活动", "停止活动", "调整姿势"].map(label => <button key={label} className="exo-button ghost" disabled={!collecting}
      onClick={() => session?.mark(label)}>标记：{label}</button>)}</div>
    <label className="exo-adaptive-file">补充动作标记<input aria-label="AFO采集动作备注" value={note} maxLength={120} placeholder="例如左侧换向、短暂停顿"
      onChange={event => setNote(event.target.value)} /></label>
    <button className="exo-button ghost" disabled={!collecting || !note.trim()} onClick={() => { session?.mark(note.trim()); setNote(""); }}>记录备注</button>
    {session && session.markers.length > 0 && <ol className="exo-capture-markers">{session.markers.slice(-8).map((marker, i) =>
      <li key={`${marker.elapsed_ms}-${i}`}>{(marker.elapsed_ms / 1000).toFixed(1)} s · {marker.label}</li>)}</ol>}
    <p className="exo-operation-note">标记使用电脑时间；最近收到的MCU快照仅供对照。ADAPT约10Hz，不能精确重放100Hz闭环；失锁事件另行保存。停止采集只关闭本采集器，不发送电机STOP。</p>
    {message && <p role="status">{message}</p>}
    {takes.length > 0 && <div className="exo-capture-takes"><h3>本页已保留的采集段</h3>
      <p>每段请分别保存原始UART和标记JSON，再离开或刷新页面。异常中止的记录同样可以导出；下载后请检查浏览器文件。</p>
      {takes.map((take, index) => <div className="exo-capture-take" key={take.started}>
        <p><strong>第{index + 1}段 · {new Date(take.started).toLocaleTimeString()}</strong> · {take.complete ? "通信收尾完成" : "中止 / 未完成"} · {take.outcome}</p>
        <p>已解析 {take.review.rows.size} 行，输入有效 {[...take.review.rows.values()].filter(usableAdaptiveInput).length} 行。通信收尾完成不等于运动数据可用或策略通过。</p>
        <div className="exo-export-row">
          <button className="exo-button secondary" onClick={() => save(`${filename(take)}.log`, take.chunks, "application/octet-stream")}>导出原始UART .log</button>
          <button className="exo-button secondary" onClick={() => save(`${filename(take)}.markers.json`, [JSON.stringify(take.manifest(), null, 2) + "\n"], "application/json")}>导出标记与核对JSON</button>
        </div>
      </div>)}
      {takes.length >= 5 && <p>已保留5段。请导出全部需要的记录，再刷新开始新一批。</p>}
    </div>}
  </details>;
}
