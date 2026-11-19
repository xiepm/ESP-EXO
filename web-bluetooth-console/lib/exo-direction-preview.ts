import type { DirectionPreviewResponse } from "./exo-ble-protocol";

/** Display only: raw encoders remain usable while direction/zero are invalid.
 * This never supplies a direction or a zero to the controller. */
export function directionPreviewDegrees(sample: DirectionPreviewResponse | null,
  side: number, token: number, receivedMs: number, nowMs: number): number | null {
  if (!sample || sample.result!==0 || sample.side!==side || sample.token!==token ||
      sample.remainingMs<=nowMs-receivedMs ||
      sample.baselineDegrees === null || !Number.isFinite(sample.baselineDegrees) ||
      sample.rawPositionDegrees === null || !Number.isFinite(sample.rawPositionDegrees) ||
      !Number.isFinite(sample.feedbackAgeMs) || sample.feedbackAgeMs > 50 || sample.feedbackAgeMs < 0 ||
      !Number.isFinite(receivedMs) || !Number.isFinite(nowMs) ||
      nowMs < receivedMs || nowMs - receivedMs > 600) return null;
  return sample.rawPositionDegrees - sample.baselineDegrees;
}
