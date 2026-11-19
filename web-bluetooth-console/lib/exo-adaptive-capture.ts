import { parseAdaptiveLog, type AdaptiveRow } from "./exo-adaptive.ts";

// Deliberately no generic console writer is exported to the page.
const READ_COMMANDS = new Set(["system_status", "ble_status", "adaptive_status", "adaptive_dump", "adaptive_faults"]);
export function captureCommandBytes(commands: readonly string[]): Uint8Array<ArrayBuffer> {
  if (!commands.length || commands.some(command => !READ_COMMANDS.has(command))) throw new Error("采集只允许诊断读取命令");
  return new TextEncoder().encode(commands.join("\r") + "\r");
}

export interface CapturePort {
  readable: ReadableStream<Uint8Array> | null;
  writable: WritableStream<Uint8Array> | null;
  getInfo(): { usbVendorId?: number; usbProductId?: number };
  open(options: { baudRate: number; flowControl: "none" }): Promise<void>;
  setSignals(signals: { dataTerminalReady: boolean; requestToSend: boolean }): Promise<void>;
  close(): Promise<void>;
}
export type CaptureMarker = { label: string; host_iso: string; elapsed_ms: number; last_snapshot_us: number | null };
export type LockRecord = { kind: string; timestamp_us: number; side: number; sequence: number; dropped: number; values: number[] };
export function usableAdaptiveInput(row: AdaptiveRow | null | undefined): boolean {
  return !!row && row.feedback_valid === 1 && (row.faults & 1) === 0;
}

export function parseLockRecords(text: string): LockRecord[] {
  const records: LockRecord[] = [];
  for (const line of text.split(/\r?\n/)) {
    const match = line.match(/\b(ADLOCKSTATE|ADLOCK),(.*)/);
    if (!match) continue;
    const values = match[2].trim().split(",").map(v => v.trim() === "" ? NaN : Number(v));
    if (values.length !== 20 || values.some(v => !Number.isFinite(v)) ||
      ![0, 1].includes(values[2]) || !Number.isSafeInteger(values[0]) || values[0] < 0 ||
      ![0, 1].includes(values[14]) || ![0, 1].includes(values[15]) || ![0, 1].includes(values[18]) ||
      [1, 3, 4, 5, 6, 11, 16, 19].some(i => !Number.isSafeInteger(values[i]) || values[i] < 0) || values[6] > 511) {
      throw new Error("ADLOCK诊断行损坏；保留原文，不能当作有效记录");
    }
    records.push({ kind: match[1], timestamp_us: values[0], side: values[2], sequence: values[1], dropped: values[19], values });
  }
  return records;
}

export class CaptureReview {
  readonly observeRunning: boolean;
  constructor(observeRunning = false) { this.observeRunning=observeRunning; }
  boot: string | null = null;
  boundary: number | null = null;
  lastSnapshot: number | null = null;
  generation: number | null = null;
  rows = new Map<string, AdaptiveRow>();
  events = new Map<string, LockRecord>();
  historicalRows = 0;
  historicalEvents = 0;
  latest: (AdaptiveRow | null)[] = [null, null];
  droppedRange: [number, number] | null = null;
  eventDroppedRange: [number, number] | null = null;

  accept(block: string) {
    assertNoReset(block);
    if (!this.observeRunning && (!/^\s*State:\s+IDLE\s*$/m.test(block) || !/^\s*Run requested:\s+no\s*$/m.test(block) ||
      !/^\s*Output allowed:\s+no\s*$/m.test(block))) throw new Error("未读回完整的IDLE、无运行请求、无输出状态");
    if(this.observeRunning && (!/^\s*State:\s+(IDLE|STARTING|RUNNING|STOPPING|ERROR)\s*$/m.test(block) ||
      !/^\s*Run requested:\s+(yes|no)\s*$/m.test(block)||!/^\s*Output allowed:\s+(yes|no)\s*$/m.test(block)))
      throw new Error("缺少完整运行状态读回，保留日志后结束监测");
    const boots = [...block.matchAll(/Boot ID:\s*(0x[0-9A-Fa-f]{8})\b/g)].map(m => m[1].toUpperCase());
    if (boots.length !== 1 || (this.boot !== null && this.boot !== boots[0])) throw new Error("Boot缺失或已改变，本段结束");
    const locks = parseLockRecords(block);
    const states = locks.filter(r => r.kind === "ADLOCKSTATE");
    if (states.length !== 2 || new Set(states.map(r => r.side)).size !== 2 || states[0].timestamp_us !== states[1].timestamp_us) {
      throw new Error("缺少同周期的双侧AFO状态快照");
    }
    const snapshot = states[0].timestamp_us;
    if (this.lastSnapshot !== null && snapshot <= this.lastSnapshot) throw new Error("AFO当前快照时间未推进或回退，本段结束");
    const parsed = parseAdaptiveLog(block);
    if (parsed.malformed || ![0, 1].every(side => parsed.rows.some(r => r.side === side && r.timestamp_us === snapshot))) {
      throw new Error("AFO采样行损坏或缺少当前双侧读数");
    }
    const boundary = this.boundary ?? snapshot;
    const fresh = parsed.rows.filter(r => r.timestamp_us >= boundary);
    const relevantLocks = locks.filter(r => r.timestamp_us >= boundary);
    if (!this.observeRunning && (fresh.some(r => r.system_state !== 1 || r.command_valid !== 0 || r.final_ff_command_nm !== 0) ||
      relevantLocks.some(r => r.values[3] !== 1 || r.values[18] !== 0 || r.values[17] !== 0))) {
      throw new Error("记录出现运行态或有效输出命令；只读采集结束，请在主控制区核对设备");
    }
    if(this.observeRunning && (fresh.some(r=>r.system_state<1||r.system_state>5||Math.abs(r.final_ff_command_nm)>5.00001)||
      relevantLocks.some(r=>r.values[3]<1||r.values[3]>5||Math.abs(r.values[17])>5.00001)))
      throw new Error("记录超出5 N·m监测范围或状态无效；原始数据已保留，采集器没有发送STOP");
    const generation = this.generation ?? fresh[0].generation;
    if (fresh.some(r => r.generation !== generation) || relevantLocks.some(r => r.values[5] !== generation)) {
      throw new Error("本段AFO配置代次发生变化，请分段分析");
    }
    if (fresh.some(r => r.timestamp_us > snapshot) || relevantLocks.some(r => r.timestamp_us > snapshot)) {
      throw new Error("诊断时序与当前快照冲突");
    }
    this.boot = boots[0]; this.boundary = boundary; this.lastSnapshot = snapshot; this.generation = generation;
    this.historicalRows += parsed.rows.length - fresh.length;
    this.historicalEvents += locks.filter(r => r.kind === "ADLOCK" && r.timestamp_us < boundary).length;
    for (const row of fresh) {
      const key = `${row.timestamp_us}:${row.side}`;
      const old = this.rows.get(key);
      // A snapshot and a later drained ring entry can legitimately have different dropped counters.
      if (old && Object.keys(row).some(k => k !== "dropped" && old[k] !== row[k])) throw new Error("同周期AFO记录内容冲突");
      this.rows.set(key, row);
      if (!this.latest[row.side] || row.timestamp_us >= this.latest[row.side]!.timestamp_us) this.latest[row.side] = row;
      this.droppedRange = this.range(this.droppedRange, row.dropped);
    }
    for (const event of relevantLocks.filter(r => r.kind === "ADLOCK")) {
      const key = `${event.sequence}:${event.side}`;
      const old = this.events.get(key);
      if (old && JSON.stringify(old) !== JSON.stringify(event)) throw new Error("失锁记录序号冲突");
      this.events.set(key, event);
      this.eventDroppedRange = this.range(this.eventDroppedRange, event.dropped);
    }
  }
  private range(previous: [number, number] | null, value: number): [number, number] {
    return previous ? [Math.min(previous[0], value), Math.max(previous[1], value)] : [value, value];
  }
  summary() {
    const validRows = [...this.rows.values()].filter(usableAdaptiveInput).length;
    return { boot: this.boot, boundary_us: this.boundary, last_snapshot_us: this.lastSnapshot,
      config_generation: this.generation, unique_rows: this.rows.size, exact_loss_events: this.events.size,
      valid_input_rows: validRows, invalid_input_rows: this.rows.size - validRows,
      historical_rows_excluded: this.historicalRows, historical_events_excluded: this.historicalEvents,
      adapt_dropped_range: this.droppedRange, adlock_event_dropped_range: this.eventDroppedRange };
  }
}

export function captureInputIssue(block: string, review: CaptureReview): string | null {
  const zeroBlock = block.match(/^\s*Start block:\s+((?:LEFT|RIGHT)_ZERO_INVALID)\b/m)?.[1];
  if (zeroBlock) return `本次Boot零位无效（${zeroBlock}），尚不能采集有效运动数据。`;
  const invalid = [0, 1].filter(side => !usableAdaptiveInput(review.latest[side]));
  const lockInputInvalid = parseLockRecords(block).filter(r => r.kind === "ADLOCKSTATE" && (r.values[6] & 1) !== 0);
  if (invalid.length || lockInputInvalid.length) {
    return "双侧AFO输入尚未全部有效，请核对本次零位及电机反馈；零值占位不能作为运动数据。";
  }
  return null;
}

function assertNoReset(text: string) {
  if (/Brownout|Guru Meditation|ESP-ROM:|rst:0x|Rebooting|panic(?:ed)?/i.test(text)) {
    throw new Error("收到重启、欠压或异常日志；本段中止，原始数据已保留");
  }
}

/** Received before a fresh readback is initial evidence, not proof of a new in-session reset. */
export class CaptureResetGuard {
  private tail = "";
  private characters = 0;
  private lastTokenEnd = 0;
  private armedAt: number | null = null;
  initialSignatures: Record<string, number> = {};
  get initialCount() { return Object.values(this.initialSignatures).reduce((sum, n) => sum + n, 0); }
  arm() { this.armedAt = this.characters; }
  receive(text: string) {
    const scan = this.tail + text;
    const start = this.characters - this.tail.length;
    this.characters += text.length;
    this.tail = scan.slice(-160);
    for (const match of scan.matchAll(/Brownout|Guru Meditation|ESP-ROM:|rst:0x|Rebooting|panic(?:ed)?/gi)) {
      const at = start + match.index;
      const end = at + match[0].length;
      // A chunk overlap must not reclassify an already received startup token.
      if (end <= this.lastTokenEnd) continue;
      this.lastTokenEnd = end;
      if (this.armedAt !== null && at >= this.armedAt) {
        throw new Error("本次状态核对开始后收到新的启动、欠压或异常文本；本段中止，原始数据已保留");
      }
      const key = match[0].toLowerCase();
      this.initialSignatures[key] = (this.initialSignatures[key] ?? 0) + 1;
    }
  }
}
const delay = (ms: number) => new Promise<void>(resolve => setTimeout(resolve, ms));
const CYCLE = ["adaptive_dump", "adaptive_faults", "system_status", "ble_status", "adaptive_status"];
const PREFLIGHT = ["system_status", "ble_status", "adaptive_status"];
const MAX_BYTES = 8 * 1024 * 1024;

/** User-started, one-shot diagnostic reader. Never reconnects, calibrates, enables or stops motors. */
export class AdaptiveCaptureSession {
  review: CaptureReview;
  chunks: Uint8Array<ArrayBuffer>[] = [];
  bytes = 0;
  markers: CaptureMarker[] = [];
  commands: { host_iso: string; commands: readonly string[] }[] = [];
  started = new Date().toISOString();
  ended: string | null = null;
  phase: "opening" | "checking" | "waiting" | "collecting" | "closing" | "finished" = "opening";
  outcome = "尚未完成";
  complete = false;
  resetGuard = new CaptureResetGuard();
  initialBytes = 0;
  readbackStarted: string | null = null;
  baselineConfirmed: string | null = null;
  lastReviewedHostIso: string | null = null;
  readinessIssue: string | null = null;
  preflightBoot: string | null = null;
  readinessAttempts: { host_iso: string; boot: string | null; issue: string | null }[] = [];
  private origin = performance.now();
  private lastReceivedAt = performance.now();
  private stopRequested = false;
  private failure: Error | null = null;
  private reader: ReadableStreamDefaultReader<Uint8Array> | null = null;
  private pending: { text: string; resolve: (value: string) => void; reject: (reason: Error) => void } | null = null;
  private recheckWaiter: { resolve: () => void; reject: (reason: Error) => void } | null = null;
  private onChange: () => void;
  readonly observeRunning: boolean;
  constructor(onChange: () => void = () => {}, observeRunning = false) {
    this.observeRunning=observeRunning;this.onChange = onChange;this.review=new CaptureReview(observeRunning);
  }

  mark(label: string) {
    if (this.phase !== "collecting") return;
    this.markers.push({ label: label.slice(0, 120), host_iso: new Date().toISOString(),
      elapsed_ms: Math.round(performance.now() - this.origin), last_snapshot_us: this.review.lastSnapshot });
    this.onChange();
  }
  stop() { this.stopRequested = true; this.recheckWaiter?.resolve(); }
  recheck() { if (this.phase === "waiting") this.recheckWaiter?.resolve(); }
  private fail(error: unknown) {
    this.failure ??= error instanceof Error ? error : new Error(String(error));
    this.pending?.reject(this.failure);
    this.recheckWaiter?.reject(this.failure);
  }
  private check() { if (this.failure) throw this.failure; }
  private async receive() {
    const decoder = new TextDecoder();
    try {
      while (this.reader) {
        const { value, done } = await this.reader.read();
        if (done) {
          if (this.phase !== "closing") this.fail(new Error("USB读取结束或已断开；原始数据已保留"));
          break;
        }
        if (!value) continue;
        this.lastReceivedAt = performance.now();
        // Save received bytes before parsing, including the bytes explaining a failure.
        const remaining = MAX_BYTES - this.bytes;
        this.chunks.push(new Uint8Array(value.subarray(0, remaining))); this.bytes += Math.min(remaining, value.byteLength);
        if (value.byteLength > remaining) { this.fail(new Error("达到8MiB采集上限；末尾已截断")); break; }
        const text = decoder.decode(value, { stream: true });
        try { this.resetGuard.receive(text); } catch (error) { this.fail(error); }
        if (this.pending) {
          this.pending.text += text;
          if (/ADAPT max_execution_us=[^\r\n]*[\r\n]/.test(this.pending.text)) this.pending.resolve(this.pending.text);
        }
      }
    } catch (error) { if (this.phase !== "closing") this.fail(error); }
    finally { this.reader?.releaseLock(); this.reader = null; }
  }
  private async query(port: CapturePort, commands: readonly string[], review = this.review) {
    this.check();
    const bytes = captureCommandBytes(commands);
    if (!port.writable) throw new Error("USB串口不可写");
    let timer: ReturnType<typeof setTimeout> | undefined;
    let writer: WritableStreamDefaultWriter<Uint8Array> | null = null;
    const reply = new Promise<string>((resolve, reject) => { this.pending = { text: "", resolve, reject }; });
    // Install a handler before awaiting I/O so disconnects cannot cause an unhandled rejection.
    void reply.catch(() => {});
    const timeout = new Promise<never>((_, reject) => {
      timer = setTimeout(() => reject(new Error("USB诊断响应超时；本段中止")), 5000);
    });
    try {
      writer = port.writable.getWriter();
      this.commands.push({ host_iso: new Date().toISOString(), commands: [...commands] });
      await Promise.race([writer.write(bytes), timeout]);
      const block = await Promise.race([reply, timeout]);
      this.check();
      review.accept(block);
      if (review === this.review) this.lastReviewedHostIso = new Date().toISOString();
      if (this.phase === "collecting") {
        const issue = captureInputIssue(block, review);
        if (issue) throw new Error(`采集中输入失效，已结束并保留诊断。${issue}`);
      }
      this.onChange();
      return block;
    } finally { clearTimeout(timer); writer?.releaseLock(); this.pending = null; }
  }
  private async waitForManualRecheck() {
    this.check();
    const request = new Promise<void>((resolve, reject) => { this.recheckWaiter = { resolve, reject }; });
    this.phase = "waiting"; this.onChange();
    // Keep this USB connection; no polling, calibration or automatic retry while the user prepares.
    try { await request; this.check(); }
    finally { this.recheckWaiter = null; }
    if (this.stopRequested) throw new Error("操作者结束准备；尚未开始运动采集，诊断日志已保留");
  }
  private async waitForInitialDrain() {
    const started = performance.now();
    // Bound the drain, rather than ignoring all errors for a fixed opening delay.
    // Repeated boot output or a continuously busy port cannot silently become a baseline.
    while (performance.now() - started < 400 || performance.now() - this.lastReceivedAt < 400) {
      this.check();
      if (this.stopRequested) throw new Error("操作者取消了采集准备");
      if (performance.now() - started >= 8000) throw new Error("连接初期串口持续有输出，尚未建立当前Boot基线；请保留日志核对");
      await delay(40);
    }
    this.check();
    this.initialBytes = this.bytes;
    this.readbackStarted = new Date().toISOString();
    this.resetGuard.arm();
  }
  async run(port: CapturePort) {
    let opened = false;
    let readTask: Promise<void> | null = null;
    try {
      const info = port.getInfo();
      if (info.usbVendorId !== 0x1a86 || info.usbProductId !== 0x55d3) throw new Error("请选择已核对的ESP-EXO USB串口（1A86:55D3）");
      await port.open({ baudRate: 115200, flowControl: "none" }); opened = true;
      if (!port.readable) throw new Error("USB串口不可读");
      this.reader = port.readable.getReader(); readTask = this.receive();
      await port.setSignals({ dataTerminalReady: false, requestToSend: false });
      await this.waitForInitialDrain();
      if (this.stopRequested) throw new Error("操作者取消了采集准备");
      const confirmation = new CaptureReview();
      while (true) {
        this.phase = "checking"; this.onChange();
        const first = await this.query(port, PREFLIGHT, confirmation);
        const firstIssue = captureInputIssue(first, confirmation);
        await delay(100); this.check();
        if (this.stopRequested) throw new Error("操作者在核对阶段结束，本段未进入采集");
        // Two fresh same-Boot snapshots must both have usable bilateral input.
        const baseline = await this.query(port, PREFLIGHT, confirmation);
        if (this.stopRequested) throw new Error("操作者在核对阶段结束，本段未进入采集");
        this.preflightBoot = confirmation.boot;
        this.readinessIssue = captureInputIssue(baseline, confirmation) ?? firstIssue;
        this.readinessAttempts.push({ host_iso: new Date().toISOString(), boot: confirmation.boot, issue: this.readinessIssue });
        if (!this.readinessIssue) { this.review.accept(baseline); break; }
        await this.waitForManualRecheck();
      }
      this.baselineConfirmed = new Date().toISOString();
      this.lastReviewedHostIso = this.baselineConfirmed;
      this.phase = "collecting"; this.mark("采集就绪（本机自动记录）");
      const deadline = performance.now() + 180_000;
      while (!this.stopRequested && performance.now() < deadline) {
        const cycleStart = performance.now();
        await this.query(port, CYCLE);
        while (!this.stopRequested && performance.now() - cycleStart < 1000) { await delay(50); this.check(); }
      }
      this.mark(this.stopRequested ? "操作者结束采集" : "达到180秒单段上限");
      // Last drain plus fresh state readback, still diagnostic reads only.
      await this.query(port, CYCLE);
      this.complete = true; this.outcome = this.observeRunning ? "监测结束；末次状态已读回，未发送电机STOP" :
        this.stopRequested ? "人工结束；末次无输出状态已读回" : "180秒结束；末次无输出状态已读回";
    } catch (error) { this.outcome = error instanceof Error ? error.message : String(error); }
    finally {
      this.phase = "closing"; this.onChange();
      try { await this.reader?.cancel(); await readTask; } catch { /* Keep the original failure. */ }
      if (opened) {
        try { await port.close(); }
        catch { this.complete = false; this.outcome += "；USB关闭未确认，请关闭此标签页释放串口"; }
      }
      if (this.failure) { this.complete = false; this.outcome = this.failure.message; }
      this.ended = new Date().toISOString(); this.phase = "finished"; this.onChange();
    }
  }
  manifest() {
    return { schema: this.observeRunning ? 4 : 3, capture_mode: this.observeRunning ? "bounded_running_observation" : "idle_only",
      kind: "human_started_readonly_afo_capture", started_host_iso: this.started, ended_host_iso: this.ended,
      complete: this.complete, outcome: this.outcome, bytes: this.bytes, summary: this.review.summary(),
      initial_connection: { bytes_before_readback: this.initialBytes, signatures: this.resetGuard.initialSignatures,
        readback_started_host_iso: this.readbackStarted, baseline_confirmed_host_iso: this.baselineConfirmed,
        note: "接入初期收到的文本未用于断言采集中发生新复位；其产生时间及复位原因仍需原始日志核对。" },
      readiness_attempts: this.readinessAttempts,
      markers: this.markers, commands: this.commands,
      notes: ["UART文件是收到的原始字节；旧缓存保留在文件中，但不计入本段统计。",
        "本段起点是两次相同Boot、闭合IDLE且MCU时间推进的第二次快照；状态核对开始后新增异常仍会中止。",
        "两次核对还必须确认双侧输入有效；零位/输入无效时保持USB连接等人工处理，仅点击重新核对才查询，不自动归零。",
        "标记时间来自电脑；last_snapshot_us只是点击时最近一次收到的MCU快照，不能视为同步事件时刻。",
        "ADAPT约10Hz；ADLOCK为100Hz控制周期产生的失锁事件，均可能发生缓存丢弃。",
        "停止采集不发送电机STOP。此记录不证明人体助力放行或名义力矩峰值通过。",
        "adaptive_dump/adaptive_faults会取走诊断队列中的记录，不可同时运行另一采集器。"] };
  }
}
