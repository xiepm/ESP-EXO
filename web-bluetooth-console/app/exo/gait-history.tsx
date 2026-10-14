"use client";

import {useCallback, useEffect, useRef, useState} from "react";
import type {AssistConfigResponse, GaitStatusResponse, InfoResponse, JointStatusResponse, SystemStatusResponse} from "@/lib/exo-ble-protocol";
import {ControlMode, ReferenceSource} from "@/lib/exo-ble-protocol";
import {appendGaitPoint, createGaitRecord, gaitPointsToCsv, gaitRecordDate, gaitRecordSummary,
  importGaitBundle, listGaitRecords, readGaitBundle, saveGaitRecord, validateGaitBundle, type GaitRecord, type GaitRecordPoint} from "@/lib/exo-gait-history";

type HistoryInput = {
  connected: boolean;
  info: InfoResponse | null;
  system: SystemStatusResponse | null;
  left: JointStatusResponse | null;
  right: JointStatusResponse | null;
  gait: GaitStatusResponse | null;
  assist: AssistConfigResponse | null;
  reference: number | undefined;
  controlMode: number | undefined;
  humanWorn: boolean;
};
const kindLabels = {observation: "静止 / 调试观察", movement: "运动 / 步态记录", engineering: "工程轨迹记录"};
const endLabels: Record<string, string> = {manual: "已保存", stop: "运动已停止", disconnected: "连接中断",
  restart: "主控重启", midnight: "跨日分段", limit: "已达单段3小时上限", page_closed: "页面已关闭", run_transition: "开始运动，观察已保存", state_exit: "主控退出运行"};
const clockText = (iso: string) => new Date(iso).toLocaleTimeString("zh-CN", {timeZone: "Asia/Shanghai", hour12: false});
const numberText = (value: number | null, suffix = "") => value === null ? "—" : `${value.toFixed(1)}${suffix}`;

function download(filename: string, text: string, mime: string) {
  const url = URL.createObjectURL(new Blob([text], {type: mime}));
  const anchor = document.createElement("a");
  anchor.href = url; anchor.download = filename; anchor.click();
  window.setTimeout(() => URL.revokeObjectURL(url), 1000);
}

function RecordChart({points}: {points: GaitRecordPoint[]}) {
  const step = Math.max(1, Math.ceil(points.length / 600));
  const visible = points.filter((_, index) => index % step === 0 || index === points.length - 1);
  const values = visible.flatMap((point) => [point.leftAngleDeg, point.rightAngleDeg]).filter((v): v is number => v !== null);
  const min = values.length ? Math.min(...values) - 1 : -1;
  const max = values.length ? Math.max(...values) + 1 : 1;
  const firstTime = visible.length ? Date.parse(visible[0].receivedIso) : 0;
  const elapsed = visible.length ? Date.parse(visible[visible.length - 1].receivedIso) - firstTime : 0;
  const path = (side: "leftAngleDeg" | "rightAngleDeg") => {
    let penDown = false;
    return visible.map((point) => {
      const value = point[side];
      if (value === null) { penDown = false; return ""; }
      const result = `${penDown ? "L" : "M"}${(100 * (Date.parse(point.receivedIso) - firstTime) / Math.max(1, elapsed)).toFixed(2)},${(36 - 32 * (value - min) / (max - min)).toFixed(2)}`;
      penDown = true; return result;
    }).join(" ");
  };
  return <div className="exo-history-chart">
    <div><b>左右髋角变化</b><span className="left">● 左髋</span><span className="right">● 右髋</span><small>单位：°</small></div>
    {values.length ? <svg viewBox="0 0 100 40" role="img" aria-label="本次记录左右髋角曲线" preserveAspectRatio="none">
      <path d={path("leftAngleDeg")} className="left"/><path d={path("rightAngleDeg")} className="right"/>
    </svg> : <p>本次尚无有效髋角数据</p>}
    <footer><span>{points.length ? clockText(points[0].receivedIso) : "—"}</span><span>{points.length ? clockText(points[points.length - 1].receivedIso) : "—"}</span></footer>
  </div>;
}

/**
 * 全程挂载的记录器：切换栏目不会卸载它，也不会重新连接BLE。
 * 仅在约1 Hz获取新的真实状态快照；主控100 Hz与独立遥测仍由原控制链负责。
 * 每个原始点和摘要都持久写入IndexedDB，浏览器刷新不会丢掉已落盘数据。
 */
export default function GaitHistory(props: HistoryInput & {hidden: boolean}) {
  const [records, setRecords] = useState<GaitRecord[]>([]);
  const [active, setActive] = useState<GaitRecord | null>(null);
  const [selected, setSelected] = useState<GaitRecord | null>(null);
  const [points, setPoints] = useState<GaitRecordPoint[]>([]);
  const [date, setDate] = useState("");
  const [storageError, setStorageError] = useState("");
  const [loading, setLoading] = useState(false);
  const [exporting, setExporting] = useState(false);
  const inputRef = useRef<HistoryInput>(props);
  const activeRef = useRef<GaitRecord | null>(null);
  const previousStateRef = useRef<number | undefined>(undefined);
  const observedResponseRef = useRef<object | null>(null);
  const writeQueueRef = useRef<Promise<void>>(Promise.resolve());
  const mountedRef = useRef(true);
  const storageFaultRef = useRef(false);
  const selectedIdRef = useRef<string | null>(null);
  const importInputRef = useRef<HTMLInputElement | null>(null);

  useEffect(() => {
    inputRef.current = props;
  }, [props]);

  const reload = useCallback(async () => {
    const result = await listGaitRecords();
    if (mountedRef.current) setRecords(result);
  }, []);
  const persist = useCallback((record: GaitRecord, point?: GaitRecordPoint) => {
    const write = writeQueueRef.current.then(() => saveGaitRecord(record, point));
    writeQueueRef.current = write.catch((error: unknown) => {
      storageFaultRef.current = true;
      activeRef.current = null;
      if (mountedRef.current) {
        setActive(null);
        setStorageError(`记录未能保存：${error instanceof Error ? error.message : "存储失败"}。请导出已有记录并检查浏览器存储空间。`);
      }
    });
    return write;
  }, []);
  const finish = useCallback((reason: string) => {
    const current = activeRef.current;
    if (!current) return;
    const ended = {...current, endedIso: new Date().toISOString(), endReason: reason};
    activeRef.current = null;
    if (mountedRef.current) setActive(null);
    void persist(ended).then(reload).catch(() => undefined);
  }, [persist, reload]);
  const begin = useCallback((kind?: GaitRecord["kind"], retryStorage = false) => {
    const input = inputRef.current;
    if (!input.connected || !input.info || activeRef.current || (storageFaultRef.current && !retryStorage)) return;
    // 重连到RUNNING时先等基础配置读回，不能仅凭状态或CAN ID把工程轨迹
    // 标成人体步态；安装方式与记录类别来自已知配置，不做自动猜测。
    if (input.system?.state === 3 && (input.reference === undefined || input.controlMode === undefined)) return;
    storageFaultRef.current = false;
    const newRecord = createGaitRecord(crypto.randomUUID(), new Date().toISOString(),
      input.info.firmware.join("."), `0x${input.info.bootId.toString(16).padStart(8, "0")}`,
      kind ?? (input.system?.state !== 3 ? "observation" :
        !input.humanWorn || input.reference !== ReferenceSource.CONSTANT || input.controlMode !== ControlMode.DAMPING ? "engineering" : "movement"));
    activeRef.current = newRecord;
    observedResponseRef.current = input.gait ?? input.right;
    setStorageError(""); setActive(newRecord);
    void persist(newRecord).catch(() => undefined);
  }, [persist]);

  useEffect(() => {
    mountedRef.current = true;
    void reload().catch((error: unknown) => setStorageError(error instanceof Error ? error.message : "无法读取本地记录"));
    return () => { mountedRef.current = false; };
  }, [reload]);

  useEffect(() => {
    const state = props.system?.state;
    const current = activeRef.current;
    if (!props.connected) finish("disconnected");
    else if (current && props.info && current.bootId !== `0x${props.info.bootId.toString(16).padStart(8, "0")}`) finish("restart");
    else if (state === 3 && props.system?.decodedSystemFlags.outputAllowed) {
      if (current?.kind === "observation") { finish("run_transition"); begin(); }
      else if (!current) begin();
    }
    // 故障或失去输出许可同样结束本段，避免把后续静止样本算进运动记录。
    else if (current && current.kind !== "observation" && state !== undefined &&
      (state !== 3 || !props.system?.decodedSystemFlags.outputAllowed)) finish(state === 1 ? "stop" : "state_exit");
    else if (previousStateRef.current === 3 && state === 1) finish("stop");
    previousStateRef.current = state;
  }, [props.connected, props.info, props.system, props.reference, props.controlMode, props.humanWorn, begin, finish]);

  useEffect(() => {
    const timer = window.setInterval(() => {
      let current = activeRef.current;
      const input = inputRef.current;
      if (!current || !input.connected || !input.system || !input.left || !input.right) return;
      const response = input.gait ?? input.right;
      // 用户暂停轮询时不重复旧快照；无新数据就不增加时长、步频或样本数。
      if (observedResponseRef.current === response) return;
      observedResponseRef.current = response;
      const iso = new Date().toISOString();
      if (gaitRecordDate(iso) !== current.date) {
        const kind = current.kind; finish("midnight"); begin(kind);
        current = activeRef.current;
        if (!current) return;
      }
      if (current.sampleCount >= 10800) { finish("limit"); return; }
      const validJoint = (joint: JointStatusResponse) => joint.decodedJointFlags.jointValid &&
        joint.decodedJointFlags.online && joint.feedbackAgeMs <= 50;
      const leftValid = validJoint(input.left); const rightValid = validJoint(input.right);
      const gaitValid = Boolean(input.gait?.decodedGaitFlags.overallValid &&
        input.gait.estimateAgeMs !== null && input.gait.estimateAgeMs <= 200);
      const point: GaitRecordPoint = {
        recordId: current.id, index: current.sampleCount, receivedIso: iso, systemState: input.system.state,
        leftAngleDeg: leftValid ? input.left.positionDegrees : null,
        rightAngleDeg: rightValid ? input.right.positionDegrees : null,
        leftVelocityDegS: leftValid ? input.left.velocityDegreesPerSecond : null,
        rightVelocityDegS: rightValid ? input.right.velocityDegreesPerSecond : null,
        leftPhasePercent: gaitValid ? input.gait!.leftPhasePercent : null,
        rightPhasePercent: gaitValid ? input.gait!.rightPhasePercent : null,
        cadenceSpm: gaitValid ? input.gait!.cadenceSpm : null,
        confidencePercent: input.gait?.confidencePercent ?? null, gaitValid,
        qualityFlags: input.gait?.qualityFlags ?? null,
        leftFeedbackAgeMs: input.left.feedbackAgeMs, rightFeedbackAgeMs: input.right.feedbackAgeMs,
        gaitAgeMs: input.gait?.estimateAgeMs ?? null,
        assistExecution: input.assist?.executionStateName ?? "UNKNOWN",
        leftAssistPercent: input.assist?.leftLevelPercent ?? null,
        rightAssistPercent: input.assist?.rightLevelPercent ?? null,
      };
      const next = appendGaitPoint(current, point);
      activeRef.current = next; setActive(next);
      void persist(next, point).catch(() => undefined);
    }, 1000);
    const pageClosed = () => finish("page_closed");
    window.addEventListener("pagehide", pageClosed);
    return () => { window.clearInterval(timer); window.removeEventListener("pagehide", pageClosed); };
  }, [begin, finish, persist]);

  async function openRecord(record: GaitRecord) {
    selectedIdRef.current = record.id;
    setLoading(true); setSelected(record); setPoints([]);
    try {
      const bundle = await readGaitBundle(record.id);
      if (selectedIdRef.current === record.id) { setSelected(bundle.record); setPoints(bundle.points); }
    }
    catch (error) { setStorageError(error instanceof Error ? error.message : "记录读取失败"); }
    finally { if (selectedIdRef.current === record.id) setLoading(false); }
  }
  async function exportRecord(record: GaitRecord, format: "csv" | "json") {
    setExporting(true);
    try {
      await writeQueueRef.current;
      const bundle = await readGaitBundle(record.id);
      const filename = `ESP-EXO-${record.date}-${record.id.slice(0, 8)}`;
      if (format === "csv") download(`${filename}.csv`, gaitPointsToCsv(bundle.points), "text/csv;charset=utf-8");
      else download(`${filename}.json`, JSON.stringify(bundle, null, 2), "application/json;charset=utf-8");
    } catch (error) { setStorageError(error instanceof Error ? error.message : "记录导出失败"); }
    finally { setExporting(false); }
  }
  async function importFile(file: File) {
    setExporting(true); setStorageError("");
    try {
      if (file.size > 12 * 1024 * 1024) throw new Error("记录文件超过12 MB");
      await importGaitBundle(validateGaitBundle(JSON.parse(await file.text())));
      await reload();
    } catch (error) { setStorageError(error instanceof Error ? error.message : "记录导入失败"); }
    finally { setExporting(false); if (importInputRef.current) importInputRef.current.value = ""; }
  }
  const filtered = records.filter((record) => !date || record.date === date);
  const activeSummary = active ? gaitRecordSummary(active) : null;
  const selectedSummary = selected ? gaitRecordSummary(selected) : null;
  return <section className="exo-panel exo-history-panel" hidden={props.hidden} aria-label="数据记录内容">
    <article className="exo-card exo-history-intro">
      <div className="exo-card-heading"><div><p className="exo-kicker">按日期留存</p><h2>我的步态记录</h2></div>
        <span className={`exo-state-chip ${active ? "good" : "muted"}`}>{active ? "正在记录" : "本地保存"}</span></div>
      <p className="exo-operation-note">进入运行状态后自动记录，停止或断连后保存。也可单独记录静止观察，记录按钮只采集数据，不启动外骨骼。</p>
      <div className="exo-history-toolbar">
        <label>记录日期<input aria-label="记录日期" type="date" value={date}
          onInput={(event) => setDate(event.currentTarget.value)} onChange={(event) => setDate(event.target.value)}/></label>
        <button className="exo-button ghost" onClick={() => setDate("")}>全部日期</button>
        <button className="exo-button ghost" onClick={() => void reload().catch((error: unknown) => setStorageError(String(error)))}>刷新记录</button>
        <input ref={importInputRef} hidden type="file" accept=".json,application/json" aria-label="选择步态记录文件"
          onChange={(event) => {const file = event.target.files?.[0]; if (file) void importFile(file);}}/>
        <button className="exo-button ghost" disabled={exporting} onClick={() => importInputRef.current?.click()}>导入记录 JSON</button>
        <button className={`exo-button ${active ? "secondary" : "primary"}`} disabled={!active && (!props.connected || !props.info)}
          onClick={() => active ? finish("manual") : begin(undefined, true)}>{active ? "结束记录（不停止外骨骼）" : "开始观察记录"}</button>
      </div>
      {active && activeSummary && <div className="exo-recording-strip"><span className="exo-recording-dot"/><b>{kindLabels[active.kind]}</b>
        <span>{active.date} · {clockText(active.startedIso)}</span><span>{active.sampleCount} 个真实采样点</span><span>{activeSummary.durationSeconds.toFixed(0)} 秒</span>
        <button className="exo-button ghost" disabled={exporting} onClick={() => void exportRecord(active, "json")}>导出当前记录 JSON</button></div>}
      {storageError && <p className="exo-inline-warning" role="alert">{storageError}</p>}
    </article>
    {filtered.length === 0 ? <article className="exo-card exo-empty-records"><span aria-hidden="true">▤</span><h3>{date ? "这一天还没有记录" : "还没有步态记录"}</h3>
      <p>完成连接与穿戴校准后，真人主动运动的记录会出现在这里。</p><small>静止观察不会被当成行走；工程轨迹会单独标记。</small></article> :
      <div className="exo-record-list">{filtered.map((record) => {
        const summary = gaitRecordSummary(record);
        return <article className="exo-card exo-record-row" key={record.id}>
          <div className="exo-record-date"><strong>{record.date.slice(5).replace("-", " / ")}</strong><span>{record.date.slice(0, 4)} · {clockText(record.startedIso)}</span></div>
          <div className="exo-record-description"><b>{kindLabels[record.kind]}</b><span>{record.id === active?.id ? "记录中" : record.endReason ? endLabels[record.endReason] ?? record.endReason : "未完整结束，已落盘数据可查看"}</span></div>
          <div><small>采样时长</small><b>{summary.durationSeconds.toFixed(0)} 秒</b></div>
          <div><small>有效步态</small><b>{numberText(summary.validPercent, "%")}</b></div>
          <div><small>平均步频</small><b>{numberText(summary.cadence, " 步/分")}</b></div>
          <button className="exo-button secondary" onClick={() => void openRecord(record)}>查看记录</button>
        </article>;
      })}</div>}
    {selected && selectedSummary && <article className="exo-card exo-record-detail">
      <div className="exo-card-heading"><div><p className="exo-kicker">{selected.date} · {clockText(selected.startedIso)}</p><h2>{kindLabels[selected.kind]}</h2></div>
        <button className="exo-button ghost" onClick={() => {selectedIdRef.current = null; setSelected(null); setPoints([]);}}>收起详情</button></div>
      <div className="exo-data-grid"><div><span>左右髋角范围</span><b>{numberText(selectedSummary.leftSpan, "°")} / {numberText(selectedSummary.rightSpan, "°")}</b></div>
        <div><span>原始采样点</span><b>{selected.sampleCount}</b></div><div><span>固件 / 本次启动</span><b>{selected.firmware} / {selected.bootId}</b></div>
        <div><span>记录来源</span><b>蓝牙状态快照 · 约1 Hz</b></div></div>
      {loading ? <p>正在读取记录…</p> : <RecordChart points={points}/>}
      <div className="exo-export-row"><button className="exo-button secondary" disabled={exporting || loading} onClick={() => void exportRecord(selected, "csv")}>导出本次 CSV</button>
        <button className="exo-button ghost" disabled={exporting || loading} onClick={() => void exportRecord(selected, "json")}>导出本次 JSON</button></div>
    </article>}
    <p className="exo-history-footnote">记录保存在当前浏览器的本机存储，清除网站数据会删除记录，重要数据请导出。日期使用北京时间。步态相位来自髋关节编码器周期代理事件；当前尚不计算实际步数或距离。</p>
  </section>;
}
