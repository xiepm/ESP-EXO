import assert from "node:assert/strict";
import test from "node:test";
import {appendGaitPoint, createGaitRecord, gaitPointsToCsv, gaitRecordDate, gaitRecordSummary, validateGaitBundle} from "../lib/exo-gait-history.ts";

const makeRecord = () => createGaitRecord("test-record", "2026-10-09T09:00:00Z", "0.10.2", "0x3503c3b7", "observation");
const point = (overrides = {}) => ({recordId: "test-record", index: 0, receivedIso: "2026-10-09T09:00:01Z",
  systemState: 1, leftAngleDeg: null, rightAngleDeg: null, leftVelocityDegS: null, rightVelocityDegS: null,
  leftPhasePercent: null, rightPhasePercent: null, cadenceSpm: null, confidencePercent: null, gaitValid: false,
  qualityFlags: 4, leftFeedbackAgeMs: 65535, rightFeedbackAgeMs: 65535, gaitAgeMs: null,
  assistExecution: "DISABLED", leftAssistPercent: 0, rightAssistPercent: 0, ...overrides});

test("日期按北京时间跨午夜分组", () => {
  assert.equal(gaitRecordDate("2026-10-09T15:59:59Z"), "2026-10-09");
  assert.equal(gaitRecordDate("2026-10-09T16:00:00Z"), "2026-10-10");
});
test("缺失数据不伪装成0角度、0步频或有效步态", () => {
  const result = appendGaitPoint(makeRecord(), point());
  assert.equal(result.validGaitCount, 0);
  assert.equal(result.runningCount, 0);
  assert.deepEqual(gaitRecordSummary(result), {durationSeconds: 1, leftSpan: null, rightSpan: null, validPercent: 0, cadence: null});
});
test("有效步频统计忽略无效估计，保留左右真实角度跨度", () => {
  const first = appendGaitPoint(makeRecord(), point({systemState: 3, gaitValid: true, cadenceSpm: 60, leftAngleDeg: -5, rightAngleDeg: 3}));
  const second = appendGaitPoint(first, point({index: 1, receivedIso: "2026-10-09T09:00:02Z", cadenceSpm: 999, leftAngleDeg: 7, rightAngleDeg: -2}));
  const summary = gaitRecordSummary(second);
  assert.equal(summary.cadence, 60);
  assert.equal(summary.validPercent, 50);
  assert.equal(summary.leftSpan, 12);
  assert.equal(summary.rightSpan, 5);
  assert.equal(second.kind, "observation");
  assert.equal(first.sampleCount, 1);
});
test("记录拒绝错归属、重复序号与倒退的时间", () => {
  const record = appendGaitPoint(makeRecord(), point());
  assert.throws(() => appendGaitPoint(record, point({recordId: "another", index: 1})));
  assert.throws(() => appendGaitPoint(record, point()));
  assert.throws(() => appendGaitPoint(record, point({index: 1, receivedIso: "2026-10-09T08:00:00Z"})));
});
test("掉线后按最后一个真实样本计算时长", () => {
  const record = appendGaitPoint(makeRecord(), point());
  record.endedIso = "2026-10-09T10:00:00Z";
  assert.equal(gaitRecordSummary(record).durationSeconds, 1);
});
test("CSV保留度值、来源时间、无效空值与双引号转义", () => {
  const csv = gaitPointsToCsv([point({leftAngleDeg: 12.3, assistExecution: 'a"b'})]);
  assert.match(csv, /leftAngleDeg/);
  assert.match(csv, /"12.3",,/);
  assert.match(csv, /"a""b"/);
  assert.equal(csv.split("\n").length, 2);
});
test("导入重新计算统计，不相信文件自称100%有效步态", () => {
  const record = makeRecord();
  record.validGaitCount = 999;
  const bundle = validateGaitBundle({schemaVersion: 1, record, points: [point()]});
  assert.equal(bundle.record.sampleCount, 1);
  assert.equal(bundle.record.validGaitCount, 0);
  assert.equal(bundle.record.kind, "observation");
});
test("导入拒绝不支持的来源、单位字段和跨日数据", () => {
  assert.throws(() => validateGaitBundle({schemaVersion: 2, record: makeRecord(), points: []}));
  assert.throws(() => validateGaitBundle({schemaVersion: 1, record: {...makeRecord(), source: "FAKE"}, points: []}));
  assert.throws(() => validateGaitBundle({schemaVersion: 1, record: makeRecord(), points: [point({leftAngleDeg: "12rad"})]}));
  assert.throws(() => validateGaitBundle({schemaVersion: 1, record: makeRecord(), points: [point({receivedIso: "2026-10-10T09:00:00Z"})]}));
});
