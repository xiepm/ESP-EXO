import type { CalibrationStatusResponse, JointStatusResponse } from "./exo-ble-protocol";

type DirectionReadback = Pick<JointStatusResponse, "kind" | "direction" | "decodedJointFlags"> |
  Pick<CalibrationStatusResponse, "kind" | "direction" | "decodedJointFlags" | "rawPositionDegrees">;

/** Persistent factory configuration is distinct from live feedback readiness.
 * Only a calibration response can establish the saved-direction readback.
 * Explicit invalid mapping/direction revokes it immediately; disconnection and
 * Boot changes clear the page's session state separately. */
export function mappingConfirmationAfterReadback(previous: boolean, persistentDirection: boolean,
  response: DirectionReadback): boolean {
  const flags = response.decodedJointFlags;
  if (!flags.mappingValid || !flags.directionValid || ![1, 2].includes(response.direction)) return false;
  if (response.kind === "calibrationStatus") {
    return persistentDirection ? true : response.rawPositionDegrees === null ? false : previous;
  }
  // Legacy manual checks remain session-local and are invalidated by lost feedback.
  if (!persistentDirection && (!flags.feedbackValid || !flags.online)) return false;
  return previous;
}

export function mustReconfirmNeutral(response: Pick<JointStatusResponse, "direction" | "decodedJointFlags">): boolean {
  const flags = response.decodedJointFlags;
  return !flags.feedbackValid || !flags.online || !flags.mappingValid || !flags.directionValid ||
    ![1, 2].includes(response.direction);
}

/** A new zero needs fresh raw encoder angles, not an already-valid old zero. */
export function recalibrationFeedbackIssue(left: CalibrationStatusResponse | null,
  right: CalibrationStatusResponse | null): string | null {
  const issues: string[] = [];
  for (const [name, side, value] of [["左髋", 0, left], ["右髋", 1, right]] as const) {
    if (!value || value.side !== side || !value.decodedJointFlags.feedbackValid ||
        !value.decodedJointFlags.online || value.rawPositionDegrees === null ||
        !Number.isFinite(value.rawPositionDegrees)) {
      issues.push(`${name}缺少有效角度反馈`);
    } else if (!Number.isFinite(value.feedbackAgeMs) || value.feedbackAgeMs < 0 || value.feedbackAgeMs > 50) {
      issues.push(`${name}反馈已过期，需要50毫秒内的新反馈`);
    } else if (!value.decodedJointFlags.mappingValid || !value.decodedJointFlags.directionValid ||
        ![1, 2].includes(value.direction)) {
      issues.push(`${name}的电机映射或出厂方向尚未确认`);
    }
  }
  return issues.length ? `${issues.join("；")}。` : null;
}
