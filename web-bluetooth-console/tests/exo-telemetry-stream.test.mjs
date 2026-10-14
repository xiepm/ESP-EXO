import assert from "node:assert/strict";
import test from "node:test";

import {
  TelemetryChannel,
  parseTelemetryPacket,
} from "../lib/exo-ble-protocol.ts";
import {
  TelemetryStreamAccumulator,
  telemetrySamplesToCsv,
  telemetrySamplesToJsonl,
} from "../lib/exo-telemetry-stream.ts";

function samplePacket(channel, sampleSequence, left, right, streamSequence = 1) {
  const bytes = new Uint8Array(20);
  const view = new DataView(bytes.buffer);
  bytes[0] = 1;
  bytes[1] = 1;
  bytes[2] = channel;
  bytes[3] = 0x5b; // 双侧有效、RUNNING、output_allowed；真实助力保持关闭。
  view.setUint16(4, streamSequence, true);
  view.setUint32(6, sampleSequence, true);
  view.setUint32(10, sampleSequence * 50, true);
  view.setUint16(14, 7, true);
  if (channel === TelemetryChannel.GAIT_PHASE ||
      channel === TelemetryChannel.GAIT_CONFIDENCE) {
    view.setUint16(16, left, true);
    view.setUint16(18, right, true);
  } else {
    view.setInt16(16, left, true);
    view.setInt16(18, right, true);
  }
  const packet = parseTelemetryPacket(bytes);
  assert.equal(packet.kind, "sample");
  return packet;
}

test("combines all channels that share one stream/sample sequence", () => {
  const accumulator = new TelemetryStreamAccumulator();
  const first = accumulator.accept(samplePacket(
    TelemetryChannel.GAIT_PHASE,
    10,
    2500,
    7500,
  ));
  assert.equal(first.isNewSample, true);
  assert.equal(first.sample.leftPhasePercent, 25);

  const second = accumulator.accept(samplePacket(
    TelemetryChannel.ASSIST_SUGGESTION,
    10,
    1000,
    -500,
  ));
  assert.equal(second.isNewSample, false);
  assert.equal(second.sample.leftPhasePercent, 25);
  assert.equal(second.sample.leftAssistSuggestionNm, 1);
  assert.equal(second.sample.rightAssistSuggestionNm, -0.5);
  assert.equal(second.stats.uniqueSampleCount, 1);
  assert.equal(second.stats.packetCount, 2);
});

test("counts forward sample gaps, tolerates stream restart, and detects duplicates", () => {
  const accumulator = new TelemetryStreamAccumulator();
  accumulator.accept(samplePacket(TelemetryChannel.GAIT_PHASE, 1, 0, 0));
  accumulator.accept(samplePacket(TelemetryChannel.GAIT_PHASE, 4, 0, 0));
  const duplicate = accumulator.accept(
    samplePacket(TelemetryChannel.GAIT_PHASE, 4, 0, 0),
  );
  assert.equal(duplicate.stats.webMissingSampleCount, 2);
  assert.equal(duplicate.stats.duplicateChannelCount, 1);

  const restarted = accumulator.accept(
    samplePacket(TelemetryChannel.GAIT_PHASE, 1, 0, 0, 2),
  );
  assert.equal(restarted.stats.webMissingSampleCount, 2);
  assert.equal(restarted.isNewSample, true);
});

test("exports assembled samples as CSV and JSONL", () => {
  const accumulator = new TelemetryStreamAccumulator();
  const point = accumulator.accept(
    samplePacket(TelemetryChannel.GAIT_CONFIDENCE, 3, 8000, 8000),
  ).sample;
  const csv = telemetrySamplesToCsv([point]);
  assert.match(csv, /leftConfidencePercent/);
  assert.match(csv, /80/);
  const jsonl = telemetrySamplesToJsonl([point]);
  assert.deepEqual(JSON.parse(jsonl.trim()).sampleSequence, 3);
});
