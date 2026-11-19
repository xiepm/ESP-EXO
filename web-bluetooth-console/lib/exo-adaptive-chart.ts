import { ADAPTIVE_COLUMNS, parseAdaptiveLog, type AdaptiveRow } from "./exo-adaptive.ts";
import { parseLockRecords, usableAdaptiveInput, type LockRecord } from "./exo-adaptive-capture.ts";

export type AdaptiveChartSource = { rows: AdaptiveRow[]; events: LockRecord[]; boot: string | null; name: string };
export type AdaptiveChartView = { source: AdaptiveChartSource; live: boolean; active?: boolean; lastReceived?: number };
export type DirectionPoint = { row: AdaptiveRow; secant: number | null; projection: number | null;
  opposed: boolean; positionSupport: "supported" | "disagrees" | "unknown"; crossesTurn: boolean };

/** Sort only an identified single Boot or a monotonic CSV. Never join reset epochs. */
export function chartLog(text: string, name: string): AdaptiveChartSource {
  const boots = [...new Set([...text.matchAll(/Boot ID:\s*(0x[0-9a-f]{8})\b/gi)].map(m => m[1].toUpperCase()))];
  if (boots.length > 1) throw new Error("文件包含多个Boot，请按Boot分段后查看；图表不会拼接重启前后的数据。");
  const parsed = parseAdaptiveLog(text);
  if (!parsed.rows.length || parsed.malformed) throw new Error(`图表需要完整AFO记录；有效${parsed.rows.length}行，损坏${parsed.malformed}行。`);
  if (!boots.length && parsed.resets) throw new Error("未提供Boot且时间有回退，无法区分缓存顺序与重启，请导入单段有序CSV或带Boot的原始日志。");
  return { rows: orderedChartRows(parsed.rows), events: parseLockRecords(text).filter(e => e.kind === "ADLOCK"),
    boot: boots[0] ?? null, name };
}

export function orderedChartRows(rows: Iterable<AdaptiveRow>): AdaptiveRow[] {
  const unique = new Map<string, AdaptiveRow>();
  for (const row of rows) {
    const key = `${row.timestamp_us}:${row.side}`, old = unique.get(key);
    if (old && ADAPTIVE_COLUMNS.some(k => k !== "dropped" && old[k] !== row[k])) throw new Error("同一MCU时刻存在冲突记录，图表拒绝覆盖。");
    unique.set(key, row);
  }
  return [...unique.values()].sort((a, b) => a.timestamp_us - b.timestamp_us || a.side - b.side);
}

export function comparisonTorque(phase: number, amplitude: number): number {
  if (!Number.isFinite(phase) || !Number.isFinite(amplitude) || amplitude < 0 || amplitude > 15) return NaN;
  if (amplitude === 0) return 0;
  const gaussian = (peak: number) => Math.exp((Math.cos(2 * Math.PI * (phase - peak)) - 1) / (2 * Math.PI * .1) ** 2);
  return amplitude * (gaussian(.75) - gaussian(.25));
}

// 150 ms is only a chart coverage guard, never a firmware freshness threshold.
export function chartContinuous(a: AdaptiveRow, b: AdaptiveRow): boolean {
  return b.timestamp_us > a.timestamp_us && b.timestamp_us - a.timestamp_us <= 150_000 &&
    a.generation === b.generation && usableAdaptiveInput(a) && usableAdaptiveInput(b);
}

export function directionPoints(rows: AdaptiveRow[], side: number, amplitude: number): DirectionPoint[] {
  const series = rows.filter(r => r.side === side);
  return series.map((row, i) => {
    const before = series[i - 1], after = series[i + 1];
    let secant: number | null = null, crossesTurn = false;
    if (before && after && chartContinuous(before, row) && chartContinuous(row, after)) {
      secant = (after.q_deg - before.q_deg) * 1e6 / (after.timestamp_us - before.timestamp_us);
      crossesTurn = (row.q_deg - before.q_deg) * (after.q_deg - row.q_deg) < 0;
    }
    const projection = usableAdaptiveInput(row) && row.locked ? comparisonTorque(row.corrected_phase, amplitude) : null;
    const opposed = projection !== null && amplitude > 0 && Math.abs(row.v_deg_s) >= 3 &&
      Math.abs(projection) >= amplitude * .5 && projection * row.v_deg_s < 0;
    const positionSupport = !opposed || secant === null || Math.abs(secant) < 3 ? "unknown" :
      projection! * secant < 0 ? "supported" : "disagrees";
    return { row, secant, projection, opposed, positionSupport, crossesTurn };
  });
}

export function chartSegments(points: DirectionPoint[], value: (p: DirectionPoint) => number | null, phase = false, breakTimes: number[] = []): DirectionPoint[][] {
  const segments: DirectionPoint[][] = []; let current: DirectionPoint[] = [];
  for (const point of points) {
    const v = value(point), previous = current.at(-1);
    const invalid = v === null || !Number.isFinite(v) || !usableAdaptiveInput(point.row);
    if (invalid || previous && (!chartContinuous(previous.row, point.row) ||
        breakTimes.some(t => t > previous.row.timestamp_us && t <= point.row.timestamp_us) ||
        phase && Math.abs(v! - value(previous)!) > .5)) {
      if (current.length) segments.push(current);
      current = [];
    }
    if (!invalid) current.push(point);
  }
  if (current.length) segments.push(current);
  return segments;
}
