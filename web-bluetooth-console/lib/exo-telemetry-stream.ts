import {
  TelemetryChannel,
  type TelemetrySamplePacket,
} from "./exo-ble-protocol.ts";

/**
 * 页面图表和导出使用的“完整采样时刻”。固件会把同一时刻拆成最多7个
 * Notify；累加器依据 streamSequence + sampleSequence 把它们重新拼起来。
 */
export type TelemetrySamplePoint = {
  webSession: number;
  streamSequence: number;
  sampleSequence: number;
  timestampMs: number;
  receivedIsoTime: string;
  receivedChannelMask: number;
  outputAllowed: boolean;
  realOutputEnabled: boolean;
  systemState: number;
  droppedSincePrevious: boolean;
  activationEpochLow16: number;
  leftPositionDeg: number | null;
  rightPositionDeg: number | null;
  leftVelocityDegS: number | null;
  rightVelocityDegS: number | null;
  leftMotorTorqueNm: number | null;
  rightMotorTorqueNm: number | null;
  leftPhasePercent: number | null;
  rightPhasePercent: number | null;
  leftAssistSuggestionNm: number | null;
  rightAssistSuggestionNm: number | null;
  leftConfidencePercent: number | null;
  rightConfidencePercent: number | null;
  leftAppliedTorqueNm: number | null;
  rightAppliedTorqueNm: number | null;
};

export type TelemetryAccumulatorStats = {
  packetCount: number;
  uniqueSampleCount: number;
  duplicateChannelCount: number;
  webMissingSampleCount: number;
  sentinelMismatchCount: number;
};

export type TelemetryAcceptResult = {
  sample: TelemetrySamplePoint;
  isNewSample: boolean;
  stats: TelemetryAccumulatorStats;
};

function emptySample(packet: TelemetrySamplePacket): TelemetrySamplePoint {
  return {
    webSession: 0, // 页面收包时赋予本次BLE会话；该字段不是设备协议字段。
    streamSequence: packet.streamSequence,
    sampleSequence: packet.sampleSequence,
    timestampMs: packet.timestampMs,
    receivedIsoTime: new Date().toISOString(),
    receivedChannelMask: 0,
    outputAllowed: packet.outputAllowed,
    realOutputEnabled: packet.realOutputEnabled,
    systemState: packet.systemState,
    droppedSincePrevious: packet.droppedSincePrevious,
    activationEpochLow16: packet.activationEpochLow16,
    leftPositionDeg: null,
    rightPositionDeg: null,
    leftVelocityDegS: null,
    rightVelocityDegS: null,
    leftMotorTorqueNm: null,
    rightMotorTorqueNm: null,
    leftPhasePercent: null,
    rightPhasePercent: null,
    leftAssistSuggestionNm: null,
    rightAssistSuggestionNm: null,
    leftConfidencePercent: null,
    rightConfidencePercent: null,
    leftAppliedTorqueNm: null,
    rightAppliedTorqueNm: null,
  };
}

/**
 * 无符号32位序号允许自然回绕。大于半圈的“倒退”视为旧包，不把它误算成
 * 数十亿个丢包；正常向前跳跃才累计网页观察到的缺口。
 */
function forwardSequenceGap(previous: number, current: number): number | null {
  const distance = (current - previous) >>> 0;
  if (distance === 0) return 0;
  if (distance >= 0x8000_0000) return null;
  return Math.max(0, distance - 1);
}

export class TelemetryStreamAccumulator {
  private readonly recent = new Map<string, TelemetrySamplePoint>();
  private currentStream: number | null = null;
  private latestSampleSequence: number | null = null;
  private stats: TelemetryAccumulatorStats = {
    packetCount: 0,
    uniqueSampleCount: 0,
    duplicateChannelCount: 0,
    webMissingSampleCount: 0,
    sentinelMismatchCount: 0,
  };

  reset(): void {
    this.recent.clear();
    this.currentStream = null;
    this.latestSampleSequence = null;
    this.stats = {
      packetCount: 0,
      uniqueSampleCount: 0,
      duplicateChannelCount: 0,
      webMissingSampleCount: 0,
      sentinelMismatchCount: 0,
    };
  }

  snapshotStats(): TelemetryAccumulatorStats {
    return { ...this.stats };
  }

  accept(packet: TelemetrySamplePacket): TelemetryAcceptResult {
    this.stats.packetCount += 1;
    if (packet.sentinelMismatch) this.stats.sentinelMismatchCount += 1;

    if (this.currentStream !== packet.streamSequence) {
      this.currentStream = packet.streamSequence;
      this.latestSampleSequence = null;
      this.recent.clear();
    }

    const key = `${packet.streamSequence}:${packet.sampleSequence}`;
    let sample = this.recent.get(key);
    const isNewSample = sample === undefined;
    if (!sample) {
      sample = emptySample(packet);
      this.recent.set(key, sample);
      this.stats.uniqueSampleCount += 1;

      if (this.latestSampleSequence !== null) {
        const gap = forwardSequenceGap(this.latestSampleSequence, packet.sampleSequence);
        if (gap !== null) {
          this.stats.webMissingSampleCount += gap;
          if (gap > 0 || packet.sampleSequence !== this.latestSampleSequence) {
            this.latestSampleSequence = packet.sampleSequence;
          }
        }
      } else {
        this.latestSampleSequence = packet.sampleSequence;
      }

      // 正常固件会连续发送同一样本的通道，保留最近8个即可兼容少量乱序，
      // 同时避免一个长时间连接让 Map 无界增长。
      if (this.recent.size > 8) {
        const oldest = this.recent.keys().next().value as string | undefined;
        if (oldest !== undefined) this.recent.delete(oldest);
      }
    } else if ((sample.receivedChannelMask & packet.channel) !== 0) {
      this.stats.duplicateChannelCount += 1;
    }

    sample.receivedChannelMask |= packet.channel;
    sample.outputAllowed = packet.outputAllowed;
    sample.realOutputEnabled = packet.realOutputEnabled;
    sample.systemState = packet.systemState;
    sample.droppedSincePrevious ||= packet.droppedSincePrevious;
    sample.activationEpochLow16 = packet.activationEpochLow16;

    switch (packet.channel) {
      case TelemetryChannel.POSITION:
        sample.leftPositionDeg = packet.leftValue;
        sample.rightPositionDeg = packet.rightValue;
        break;
      case TelemetryChannel.VELOCITY:
        sample.leftVelocityDegS = packet.leftValue;
        sample.rightVelocityDegS = packet.rightValue;
        break;
      case TelemetryChannel.MOTOR_TORQUE:
        sample.leftMotorTorqueNm = packet.leftValue;
        sample.rightMotorTorqueNm = packet.rightValue;
        break;
      case TelemetryChannel.GAIT_PHASE:
        sample.leftPhasePercent = packet.leftValue;
        sample.rightPhasePercent = packet.rightValue;
        break;
      case TelemetryChannel.ASSIST_SUGGESTION:
        sample.leftAssistSuggestionNm = packet.leftValue;
        sample.rightAssistSuggestionNm = packet.rightValue;
        break;
      case TelemetryChannel.GAIT_CONFIDENCE:
        sample.leftConfidencePercent = packet.leftValue;
        sample.rightConfidencePercent = packet.rightValue;
        break;
      case TelemetryChannel.APPLIED_TORQUE:
        sample.leftAppliedTorqueNm = packet.leftValue;
        sample.rightAppliedTorqueNm = packet.rightValue;
        break;
    }

    return {
      // React 状态必须拿到副本；后续通道到达时，累加器仍会继续修改内部对象。
      sample: { ...sample },
      isNewSample,
      stats: this.snapshotStats(),
    };
  }
}

const csvColumns: Array<keyof TelemetrySamplePoint> = [
  "webSession",
  "streamSequence",
  "sampleSequence",
  "timestampMs",
  "receivedIsoTime",
  "receivedChannelMask",
  "outputAllowed",
  "realOutputEnabled",
  "systemState",
  "droppedSincePrevious",
  "activationEpochLow16",
  "leftPositionDeg",
  "rightPositionDeg",
  "leftVelocityDegS",
  "rightVelocityDegS",
  "leftMotorTorqueNm",
  "rightMotorTorqueNm",
  "leftPhasePercent",
  "rightPhasePercent",
  "leftConfidencePercent",
  "rightConfidencePercent",
  "leftAssistSuggestionNm",
  "rightAssistSuggestionNm",
  "leftAppliedTorqueNm",
  "rightAppliedTorqueNm",
];

function csvCell(value: TelemetrySamplePoint[keyof TelemetrySamplePoint]): string {
  if (value === null) return "";
  const text = String(value);
  return /[",\r\n]/.test(text) ? `"${text.replaceAll('"', '""')}"` : text;
}

export function telemetrySamplesToCsv(samples: readonly TelemetrySamplePoint[]): string {
  const rows = [csvColumns.join(",")];
  for (const sample of samples) {
    rows.push(csvColumns.map((column) => csvCell(sample[column])).join(","));
  }
  return `${rows.join("\r\n")}\r\n`;
}

export function telemetrySamplesToJsonl(samples: readonly TelemetrySamplePoint[]): string {
  return samples.map((sample) => JSON.stringify(sample)).join("\n") + (samples.length ? "\n" : "");
}
