/** 日期记录在浏览器保存；它不发送 BLE 命令，也不参与主控实时控制。 */
export type GaitRecordPoint = {
  recordId: string;
  index: number;
  receivedIso: string;
  systemState: number;
  leftAngleDeg: number | null;
  rightAngleDeg: number | null;
  leftVelocityDegS: number | null;
  rightVelocityDegS: number | null;
  leftPhasePercent: number | null;
  rightPhasePercent: number | null;
  cadenceSpm: number | null;
  confidencePercent: number | null;
  gaitValid: boolean;
  qualityFlags: number | null;
  leftFeedbackAgeMs: number;
  rightFeedbackAgeMs: number;
  gaitAgeMs: number | null;
  assistExecution: string;
  leftAssistPercent: number | null;
  rightAssistPercent: number | null;
};

export type GaitRecord = {
  id: string;
  date: string;
  startedIso: string;
  updatedIso: string;
  endedIso: string | null;
  endReason: string | null;
  kind: "observation" | "movement" | "engineering";
  firmware: string;
  bootId: string;
  source: "BLE_STATUS_POLL";
  phaseOrigin: "HIP_FLEXION_PEAK";
  sampleCount: number;
  validGaitCount: number;
  runningCount: number;
  leftMin: number | null;
  leftMax: number | null;
  rightMin: number | null;
  rightMax: number | null;
  cadenceSum: number;
  cadenceCount: number;
};

/** 用北京时间分组，UTC 午夜前后的样本不能错分到前一天。 */
export function gaitRecordDate(iso: string): string {
  const parts = new Intl.DateTimeFormat("en-CA", {
    timeZone: "Asia/Shanghai", year: "numeric", month: "2-digit", day: "2-digit",
  }).formatToParts(new Date(iso));
  const value = (type: string) => parts.find((part) => part.type === type)!.value;
  return `${value("year")}-${value("month")}-${value("day")}`;
}

export function createGaitRecord(
  id: string, iso: string, firmware: string, bootId: string, kind: GaitRecord["kind"],
): GaitRecord {
  return {
    id, date: gaitRecordDate(iso), startedIso: iso, updatedIso: iso,
    endedIso: null, endReason: null, kind, firmware, bootId,
    source: "BLE_STATUS_POLL", phaseOrigin: "HIP_FLEXION_PEAK",
    sampleCount: 0, validGaitCount: 0, runningCount: 0,
    leftMin: null, leftMax: null, rightMin: null, rightMax: null,
    cadenceSum: 0, cadenceCount: 0,
  };
}

/** 无效数据保留 null，不用0°/0步频冒充测量；统计只包含真实有效值。 */
export function appendGaitPoint(record: GaitRecord, point: GaitRecordPoint): GaitRecord {
  if (point.recordId !== record.id || point.index !== record.sampleCount ||
      Date.parse(point.receivedIso) < Date.parse(record.updatedIso)) {
    throw new Error("记录序号、时间或归属不一致");
  }
  const min = (old: number | null, value: number | null) =>
    value === null ? old : old === null ? value : Math.min(old, value);
  const max = (old: number | null, value: number | null) =>
    value === null ? old : old === null ? value : Math.max(old, value);
  const cadenceValid = point.gaitValid && point.cadenceSpm !== null;
  return {
    ...record, updatedIso: point.receivedIso, sampleCount: record.sampleCount + 1,
    validGaitCount: record.validGaitCount + Number(point.gaitValid),
    runningCount: record.runningCount + Number(point.systemState === 3),
    leftMin: min(record.leftMin, point.leftAngleDeg),
    leftMax: max(record.leftMax, point.leftAngleDeg),
    rightMin: min(record.rightMin, point.rightAngleDeg),
    rightMax: max(record.rightMax, point.rightAngleDeg),
    cadenceSum: record.cadenceSum + (cadenceValid ? point.cadenceSpm! : 0),
    cadenceCount: record.cadenceCount + Number(cadenceValid),
  };
}

export function gaitRecordSummary(record: GaitRecord) {
  return {
    // 时长按最后一个真实采样时刻计算；掉线后的等待不会增加记录时长。
    durationSeconds: Math.max(0, (Date.parse(record.updatedIso) - Date.parse(record.startedIso)) / 1000),
    leftSpan: record.leftMin === null || record.leftMax === null ? null : record.leftMax - record.leftMin,
    rightSpan: record.rightMin === null || record.rightMax === null ? null : record.rightMax - record.rightMin,
    validPercent: record.sampleCount ? 100 * record.validGaitCount / record.sampleCount : null,
    cadence: record.cadenceCount ? record.cadenceSum / record.cadenceCount : null,
  };
}

const columns: (keyof GaitRecordPoint)[] = [
  "recordId", "index", "receivedIso", "systemState", "leftAngleDeg", "rightAngleDeg",
  "leftVelocityDegS", "rightVelocityDegS", "leftPhasePercent", "rightPhasePercent",
  "cadenceSpm", "confidencePercent", "gaitValid", "qualityFlags", "leftFeedbackAgeMs",
  "rightFeedbackAgeMs", "gaitAgeMs", "assistExecution", "leftAssistPercent", "rightAssistPercent",
];
export function gaitPointsToCsv(points: readonly GaitRecordPoint[]): string {
  const cell = (value: unknown) => value === null ? "" : `"${String(value).replaceAll('"', '""')}"`;
  return [columns.join(","), ...points.map((point) => columns.map((key) => cell(point[key])).join(","))].join("\n");
}

let databasePromise: Promise<IDBDatabase> | null = null;
function database(): Promise<IDBDatabase> {
  if (databasePromise) return databasePromise;
  databasePromise = new Promise((resolve, reject) => {
    if (typeof indexedDB === "undefined") { reject(new Error("浏览器不支持本地记录存储")); return; }
    const request = indexedDB.open("esp-exo-gait-history", 1);
    request.onupgradeneeded = () => {
      request.result.createObjectStore("records", {keyPath: "id"});
      const points = request.result.createObjectStore("points", {keyPath: ["recordId", "index"]});
      points.createIndex("recordId", "recordId");
    };
    request.onsuccess = () => {
      request.result.onversionchange = () => { request.result.close(); databasePromise = null; };
      resolve(request.result);
    };
    request.onerror = () => { databasePromise = null; reject(request.error); };
    request.onblocked = () => { databasePromise = null; reject(new Error("本地记录库被其他页面占用，请关闭旧页面后重试")); };
  });
  return databasePromise;
}

/** 摘要与原始点同一事务写入，失败时不能只存摘要而丢掉原始数据。 */
export async function saveGaitRecord(record: GaitRecord, point?: GaitRecordPoint): Promise<void> {
  const db = await database();
  await new Promise<void>((resolve, reject) => {
    const tx = db.transaction(["records", "points"], "readwrite");
    tx.objectStore("records").put(record);
    if (point) tx.objectStore("points").put(point);
    tx.oncomplete = () => resolve();
    tx.onerror = () => reject(tx.error ?? new Error("本地记录写入失败"));
    tx.onabort = () => reject(tx.error ?? new Error("本地记录写入已中断"));
  });
}

export async function listGaitRecords(): Promise<GaitRecord[]> {
  const db = await database();
  return new Promise((resolve, reject) => {
    const request = db.transaction("records").objectStore("records").getAll();
    request.onsuccess = () => resolve((request.result as GaitRecord[]).sort((a, b) => b.startedIso.localeCompare(a.startedIso)));
    request.onerror = () => reject(request.error);
  });
}

export async function readGaitPoints(recordId: string): Promise<GaitRecordPoint[]> {
  const db = await database();
  return new Promise((resolve, reject) => {
    const request = db.transaction("points").objectStore("points").index("recordId").getAll(recordId);
    request.onsuccess = () => resolve((request.result as GaitRecordPoint[]).sort((a, b) => a.index - b.index));
    request.onerror = () => reject(request.error);
  });
}

/** 记录摘要与原始点从同一只读事务取得，避免记录中导出时数量不一致。 */
export async function readGaitBundle(recordId: string): Promise<{schemaVersion: 1; record: GaitRecord; points: GaitRecordPoint[]}> {
  const db = await database();
  return new Promise((resolve, reject) => {
    const tx = db.transaction(["records", "points"]);
    const summary = tx.objectStore("records").get(recordId);
    const points = tx.objectStore("points").index("recordId").getAll(recordId);
    tx.oncomplete = () => summary.result
      ? resolve({schemaVersion: 1, record: summary.result, points: points.result.sort((a: GaitRecordPoint, b: GaitRecordPoint) => a.index - b.index)})
      : reject(new Error("未找到该记录"));
    tx.onerror = () => reject(tx.error);
  });
}

/** 导入只接受本页面导出的结构；统计重新计算，不相信文件自称的步态通过率。 */
export function validateGaitBundle(value: unknown): {schemaVersion: 1; record: GaitRecord; points: GaitRecordPoint[]} {
  if (!value || typeof value !== "object") throw new Error("不是有效的记录文件");
  const bundle = value as {schemaVersion: unknown; record: GaitRecord; points: GaitRecordPoint[]};
  const record = bundle.record;
  if (bundle.schemaVersion !== 1 || !record || !Array.isArray(bundle.points) || bundle.points.length > 10800 ||
      typeof record.id !== "string" || !/^[a-zA-Z0-9_-]{1,128}$/.test(record.id) ||
      !["observation", "movement", "engineering"].includes(record.kind) ||
      typeof record.firmware !== "string" || record.firmware.length > 24 || !/^0x[0-9a-f]{8}$/i.test(record.bootId) ||
      !Number.isFinite(Date.parse(record.startedIso)) || record.source !== "BLE_STATUS_POLL" ||
      record.phaseOrigin !== "HIP_FLEXION_PEAK" || record.date !== gaitRecordDate(record.startedIso)) {
    throw new Error("记录格式、来源或日期无效");
  }
  let normalized = createGaitRecord(record.id, record.startedIso, record.firmware, record.bootId, record.kind);
  for (const [index, point] of bundle.points.entries()) {
    if (!point || point.index !== index || point.recordId !== record.id || !Number.isFinite(Date.parse(point.receivedIso)) ||
        gaitRecordDate(point.receivedIso) !== record.date || typeof point.gaitValid !== "boolean" ||
        !Number.isInteger(point.systemState) || point.systemState < 0 || point.systemState > 5 ||
        typeof point.assistExecution !== "string" || point.assistExecution.length > 32) throw new Error("原始采样点格式无效");
    for (const key of columns) {
      if (["recordId", "receivedIso", "gaitValid", "assistExecution"].includes(key)) continue;
      const number = point[key];
      if (number !== null && (typeof number !== "number" || !Number.isFinite(number) || Math.abs(number) > 1e6)) throw new Error("原始数值无效");
    }
    normalized = appendGaitPoint(normalized, point);
  }
  if (record.endedIso !== null && (!Number.isFinite(Date.parse(record.endedIso)) || Date.parse(record.endedIso) < Date.parse(normalized.updatedIso))) throw new Error("结束时间无效");
  if (record.endReason !== null && (typeof record.endReason !== "string" || record.endReason.length > 64)) throw new Error("结束原因无效");
  normalized.endedIso = record.endedIso;
  normalized.endReason = record.endReason;
  return {schemaVersion: 1, record: normalized, points: bundle.points};
}

export async function importGaitBundle(bundle: ReturnType<typeof validateGaitBundle>): Promise<void> {
  const db = await database();
  await new Promise<void>((resolve, reject) => {
    const tx = db.transaction(["records", "points"], "readwrite");
    const existing = tx.objectStore("records").get(bundle.record.id);
    existing.onsuccess = () => {
      if (existing.result) { reject(new Error("该记录已存在，不会覆盖已有数据")); tx.abort(); return; }
      tx.objectStore("records").add(bundle.record);
      for (const point of bundle.points) tx.objectStore("points").add(point);
    };
    tx.oncomplete = () => resolve();
    tx.onerror = () => reject(tx.error ?? new Error("导入失败"));
    tx.onabort = () => reject(tx.error ?? new Error("导入已中断"));
  });
}
