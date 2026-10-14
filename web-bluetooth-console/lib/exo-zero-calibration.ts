import type { CalibrationStatusResponse, InfoResponse, JointStatusResponse, SystemStatusResponse } from "./exo-ble-protocol";

// v1的16个capability位已分配完；action=3需要明确匹配已知固件，不能把
// 0.10.9/0.10.10的“保存零位”能力误当成“采集双零位”能力。
export function supportsAtomicZeroCapture(info: Pick<InfoResponse, "firmware" | "capabilities"> | null): boolean {
  return Boolean(info && ["0.10.11", "0.10.12", "0.10.13"].includes(info.firmware.join(".")) &&
    (info.capabilities & 0x4100) === 0x4100);
}

export const CAPTURE_ZERO_TOLERANCE_DEG = 0.5;
type JointReadback = Pick<JointStatusResponse, "side" | "direction" | "positionDegrees" | "feedbackAgeMs" | "decodedJointFlags">;
type CalibrationReadback = Pick<CalibrationStatusResponse, "side" | "actuator" | "direction" | "rawPositionDegrees" | "softwareZeroDegrees" | "feedbackAgeMs" | "decodedJointFlags">;
type PairZeroTransport = {
  capturePair: () => Promise<void>;
  queryJoint: (side: number) => Promise<JointReadback>;
  queryCalibration: (side: number) => Promise<CalibrationReadback>;
  querySystem: () => Promise<Pick<SystemStatusResponse, "state">>;
};

/** 读回真实位置和设备保存的偏置。0.5°用于容纳协议量化和静止微动，
 * 绝不直接将页面读数写成0；校准后如果又移动，应显示新的真实角度。 */
export function assertCapturedZeroPair(left: JointReadback, right: JointReadback,
  leftCalibration: CalibrationReadback, rightCalibration: CalibrationReadback, state: number): void {
  if (state !== 1) throw new Error("双髋校准未确认：主控不在待机，请核对设备状态。");
  if (leftCalibration.actuator === rightCalibration.actuator ||
      ![leftCalibration.actuator, rightCalibration.actuator].every((actuator) => actuator === 0 || actuator === 1))
    throw new Error("双髋校准未确认：左右电机映射异常。");
  for (const [side, joint, calibration] of [[0, left, leftCalibration], [1, right, rightCalibration]] as const) {
    const label = side === 0 ? "左髋" : "右髋";
    if (joint.side !== side || calibration.side !== side || joint.direction !== calibration.direction ||
        ![1, 2].includes(calibration.direction) ||
        ![joint, calibration].every((value) => Number.isFinite(value.feedbackAgeMs) &&
          value.feedbackAgeMs >= 0 && value.feedbackAgeMs <= 50 &&
          value.decodedJointFlags.mappingValid && value.decodedJointFlags.directionValid &&
          value.decodedJointFlags.zeroValid && value.decodedJointFlags.jointValid &&
          value.decodedJointFlags.feedbackValid && value.decodedJointFlags.online) ||
        !Number.isFinite(joint.positionDegrees) || calibration.rawPositionDegrees === null ||
        calibration.softwareZeroDegrees === null || !Number.isFinite(calibration.rawPositionDegrees) ||
        !Number.isFinite(calibration.softwareZeroDegrees))
      throw new Error(`${label}校准读回无效，请保持双髋静止并核对反馈。`);
    if (Math.abs(joint.positionDegrees) > CAPTURE_ZERO_TOLERANCE_DEG ||
        Math.abs(calibration.rawPositionDegrees - calibration.softwareZeroDegrees) > CAPTURE_ZERO_TOLERANCE_DEG)
      throw new Error(`${label}校准后未接近0°，请保持中立静止并重新核对；本次不标记校准成功。`);
  }
}

/** 单次成对采集，随后顺序读取（共用BLE请求队列），不自动重试写操作。 */
export async function captureAndVerifyBothZeros(transport: PairZeroTransport): Promise<void> {
  await transport.capturePair();
  const left = await transport.queryJoint(0), right = await transport.queryJoint(1);
  const leftCalibration = await transport.queryCalibration(0), rightCalibration = await transport.queryCalibration(1);
  const system = await transport.querySystem();
  assertCapturedZeroPair(left, right, leftCalibration, rightCalibration, system.state);
}
