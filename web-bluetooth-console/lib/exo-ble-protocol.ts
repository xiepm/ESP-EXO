export const EXO_BLE = Object.freeze({
  DEVICE_NAME: "ESP-EXO",
  SERVICE_UUID: "f7a10001-6b5a-4d92-9e7c-3a1c8b640001",
  COMMAND_CHARACTERISTIC_UUID: "f7a10002-6b5a-4d92-9e7c-3a1c8b640001",
  RESPONSE_CHARACTERISTIC_UUID: "f7a10003-6b5a-4d92-9e7c-3a1c8b640001",
  PROTOCOL_VERSION: 1,
  HEADER_SIZE: 4,
  MAX_PAYLOAD_SIZE: 16,
  MAX_FRAME_SIZE: 20,
  RESPONSE_BIT: 0x80,
  SYSTEM_FLAGS_MASK: 0x3f,
  JOINT_FLAGS_MASK: 0x3f,
  CONTROL_CONFIG_FLAGS_MASK: 0x03,
  GAIT_FLAGS_MASK: 0x3f,
  GAIT_QUALITY_FLAGS_MASK: 0x7fff,
  ASSIST_FLAGS_MASK: 0x7f,
  ASSIST_EXECUTION_PACKED_RESERVED_MASK: 0xcc,
  SENSOR_MASK: 0x00ff,
  CAPABILITIES_MASK: 0x003f,
  INVALID_U16: 0xffff,
  PHASE_X10000_MAX: 9999,
  CADENCE_X100_MAX: 30000,
  CONFIDENCE_X10000_MAX: 10000,
  ASSIST_CONFIG_SCHEMA_VERSION: 1,
  ASSIST_LEVEL_PERCENT_MAX: 100,
  JOINT_POSITION_MRAD_MIN: -12570,
  JOINT_POSITION_MRAD_MAX: 12570,
  JOINT_VELOCITY_MRAD_S_MIN: -44000,
  JOINT_VELOCITY_MRAD_S_MAX: 44000,
  JOINT_TORQUE_MNM_MIN: -17000,
  JOINT_TORQUE_MNM_MAX: 17000,
  JOINT_KP_X100_MAX: 50000,
  JOINT_KD_X1000_MAX: 5000,
});

export const Opcode = Object.freeze({
  GET_INFO: 0x01,
  GET_SYSTEM_STATUS: 0x02,
  GET_JOINT_STATUS: 0x03,
  GET_CONTROL_CONFIG: 0x04,
  GET_JOINT_COMMAND: 0x05,
  GET_GAIT_STATUS: 0x06,
  GET_ASSIST_CONFIG: 0x07,
  START: 0x10,
  STOP: 0x11,
  CAPTURE_ZERO: 0x12,
  CLEAR_ZERO: 0x13,
  SET_CONTROL_CONFIG: 0x14,
  SET_JOINT_COMMAND: 0x15,
  SET_ASSIST_CONFIG: 0x16,
});

export const Side = Object.freeze({
  LEFT: 0x00,
  RIGHT: 0x01,
});

export const ControlMode = Object.freeze({
  NONE: 0x00,
  HOLD: 0x01,
  POSITION: 0x02,
  VELOCITY: 0x03,
  TORQUE: 0x04,
  DAMPING: 0x05,
  IMPEDANCE: 0x06,
});

export const ReferenceSource = Object.freeze({
  CONSTANT: 0x00,
  SINE: 0x01,
});

export const Capability = Object.freeze({
  SYSTEM_CONTROL: 1 << 0,
  JOINT_CALIBRATION: 1 << 1,
  ON_DEMAND_STATUS: 1 << 2,
  CONTROL_CONFIGURATION: 1 << 3,
  GAIT_STATUS: 1 << 4,
  ASSIST_CONFIGURATION: 1 << 5,
});

export const GaitEstimator = Object.freeze({
  NONE: 0x00,
  HIP_EVENT_TBE: 0x01,
});

export const GaitEstimatorState = Object.freeze({
  RESET: 0x00,
  STANDING: 0x01,
  ACQUIRING: 0x02,
  LOCKED: 0x03,
  DEGRADED: 0x04,
});

export const GaitPhaseOrigin = Object.freeze({
  UNKNOWN: 0x00,
  HIP_FLEXION_PEAK: 0x01,
  HIP_EXTENSION_PEAK: 0x02,
});

export const AssistMode = Object.freeze({
  NONE: 0x00,
  TRANSPARENT: 0x01,
  ACTIVE_PHASE: 0x02,
  ASSIST_AS_NEEDED: 0x03,
});

export const AssistExecutionState = Object.freeze({
  DISABLED: 0x00,
  SHADOW: 0x01,
  ACTIVE: 0x02,
});

export const AssistStrategy = Object.freeze({
  NONE: 0x00,
  TRANSPARENT: 0x01,
  PHASE_PROFILE: 0x02,
  AAN_RESIDUAL: 0x03,
});

export const AssistUnavailableReason = Object.freeze({
  NONE: 0x00,
  NOT_INITIALIZED: 0x01,
  CONFIG_INVALID: 0x02,
  ACTIVE_NOT_ALLOWED: 0x03,
  STRATEGY_NOT_IMPLEMENTED: 0x04,
  REQUIRED_SENSOR_MISSING: 0x05,
  NOT_ARMED: 0x06,
  REQUIRED_GAIT_INVALID: 0x07,
});

export type ResponseKind =
  | "ack"
  | "info"
  | "systemStatus"
  | "jointStatus"
  | "controlConfig"
  | "jointCommand"
  | "gaitStatus"
  | "assistConfig";

export const resultNames: Readonly<Record<number, string>> = Object.freeze({
  0x00: "OK",
  0x01: "ACCEPTED",
  0x02: "ALREADY_SATISFIED",
  0x03: "ALREADY_PENDING",
  0x10: "REJECTED_NOT_READY",
  0x11: "REJECTED_BUSY",
  0x12: "REJECTED_ERROR",
  0x13: "INTERNAL_ERROR",
  0x20: "UNSUPPORTED_VERSION",
  0x21: "INVALID_LENGTH",
  0x22: "UNKNOWN_OPCODE",
  0x23: "INVALID_REQUEST_ID",
  0x24: "INVALID_PAYLOAD",
  0x25: "REQUEST_ID_CONFLICT",
  0x7f: "NONE",
});

export const stateNames: Readonly<Record<number, string>> = Object.freeze({
  0x00: "INIT",
  0x01: "IDLE",
  0x02: "STARTING",
  0x03: "RUNNING",
  0x04: "STOPPING",
  0x05: "ERROR",
  0xff: "UNKNOWN",
});

export const stepNames: Readonly<Record<number, string>> = Object.freeze({
  0x00: "NONE",
  0x01: "CHECK_PRECONDITIONS",
  0x02: "DISABLE_MOTORS",
  0x03: "SET_MOTION_MODE",
  0x04: "ENABLE_MOTORS",
  0x05: "SEND_ZERO_OUTPUT",
  0x06: "WAIT_FRESH_FEEDBACK",
  0x07: "ARM_CONTROLLER",
  0x08: "DISARM_CONTROLLER",
  0x09: "RELEASE_OUTPUT",
  0x0a: "VERIFY_DISABLED",
  0xff: "UNKNOWN",
});

export const blockReasonNames: Readonly<Record<number, string>> = Object.freeze({
  0x00: "NONE",
  0x01: "INITIALIZING",
  0x02: "SYSTEM_NOT_IDLE",
  0x03: "OPERATION_BUSY",
  0x04: "CONTROL_TASK_NOT_READY",
  0x05: "TWAI_NOT_RUNNING",
  0x06: "LEFT_MAPPING_INVALID",
  0x07: "RIGHT_MAPPING_INVALID",
  0x08: "LEFT_DIRECTION_INVALID",
  0x09: "RIGHT_DIRECTION_INVALID",
  0x0a: "LEFT_ZERO_INVALID",
  0x0b: "RIGHT_ZERO_INVALID",
  0x0c: "LEFT_FEEDBACK_INVALID",
  0x0d: "RIGHT_FEEDBACK_INVALID",
  0x0e: "LEFT_OFFLINE",
  0x0f: "RIGHT_OFFLINE",
  0x10: "LEFT_MOTOR_FAULT",
  0x11: "RIGHT_MOTOR_FAULT",
  0x12: "SYSTEM_ERROR",
  0xff: "UNKNOWN",
});

export const directionNames: Readonly<Record<number, string>> = Object.freeze({
  0x00: "UNKNOWN",
  0x01: "POSITIVE",
  0x02: "NEGATIVE",
});

export const controlModeNames: Readonly<Record<number, string>> = Object.freeze({
  [ControlMode.NONE]: "NONE",
  [ControlMode.HOLD]: "HOLD",
  [ControlMode.POSITION]: "POSITION",
  [ControlMode.VELOCITY]: "VELOCITY",
  [ControlMode.TORQUE]: "TORQUE",
  [ControlMode.DAMPING]: "DAMPING",
  [ControlMode.IMPEDANCE]: "IMPEDANCE",
});

export const referenceSourceNames: Readonly<Record<number, string>> = Object.freeze({
  [ReferenceSource.CONSTANT]: "CONSTANT",
  [ReferenceSource.SINE]: "SINE",
});

export const gaitEstimatorNames: Readonly<Record<number, string>> = Object.freeze({
  [GaitEstimator.NONE]: "NONE",
  [GaitEstimator.HIP_EVENT_TBE]: "HIP_EVENT_TBE",
});

export const gaitEstimatorStateNames: Readonly<Record<number, string>> = Object.freeze({
  [GaitEstimatorState.RESET]: "RESET",
  [GaitEstimatorState.STANDING]: "STANDING",
  [GaitEstimatorState.ACQUIRING]: "ACQUIRING",
  [GaitEstimatorState.LOCKED]: "LOCKED",
  [GaitEstimatorState.DEGRADED]: "DEGRADED",
});

export const gaitPhaseOriginNames: Readonly<Record<number, string>> = Object.freeze({
  [GaitPhaseOrigin.UNKNOWN]: "UNKNOWN",
  [GaitPhaseOrigin.HIP_FLEXION_PEAK]: "HIP_FLEXION_PEAK",
  [GaitPhaseOrigin.HIP_EXTENSION_PEAK]: "HIP_EXTENSION_PEAK",
});

export const assistModeNames: Readonly<Record<number, string>> = Object.freeze({
  [AssistMode.NONE]: "NONE",
  [AssistMode.TRANSPARENT]: "TRANSPARENT",
  [AssistMode.ACTIVE_PHASE]: "ACTIVE_PHASE",
  [AssistMode.ASSIST_AS_NEEDED]: "ASSIST_AS_NEEDED",
});

export const assistExecutionStateNames: Readonly<Record<number, string>> = Object.freeze({
  [AssistExecutionState.DISABLED]: "DISABLED",
  [AssistExecutionState.SHADOW]: "SHADOW",
  [AssistExecutionState.ACTIVE]: "ACTIVE",
});

export const assistStrategyNames: Readonly<Record<number, string>> = Object.freeze({
  [AssistStrategy.NONE]: "NONE",
  [AssistStrategy.TRANSPARENT]: "TRANSPARENT",
  [AssistStrategy.PHASE_PROFILE]: "PHASE_PROFILE",
  [AssistStrategy.AAN_RESIDUAL]: "AAN_RESIDUAL",
});

export const assistUnavailableReasonNames: Readonly<Record<number, string>> = Object.freeze({
  [AssistUnavailableReason.NONE]: "NONE",
  [AssistUnavailableReason.NOT_INITIALIZED]: "NOT_INITIALIZED",
  [AssistUnavailableReason.CONFIG_INVALID]: "CONFIG_INVALID",
  [AssistUnavailableReason.ACTIVE_NOT_ALLOWED]: "ACTIVE_NOT_ALLOWED",
  [AssistUnavailableReason.STRATEGY_NOT_IMPLEMENTED]: "STRATEGY_NOT_IMPLEMENTED",
  [AssistUnavailableReason.REQUIRED_SENSOR_MISSING]: "REQUIRED_SENSOR_MISSING",
  [AssistUnavailableReason.NOT_ARMED]: "NOT_ARMED",
  [AssistUnavailableReason.REQUIRED_GAIT_INVALID]: "REQUIRED_GAIT_INVALID",
});

export type ProtocolErrorCode =
  | "INVALID_SOURCE"
  | "INVALID_ARGUMENT"
  | "UNKNOWN_OPCODE"
  | "INVALID_PAYLOAD"
  | "INVALID_FRAME_LENGTH"
  | "UNSUPPORTED_VERSION"
  | "NOT_A_RESPONSE"
  | "UNKNOWN_RESPONSE"
  | "INVALID_FIELD"
  | "CORRELATION_MISMATCH";

export class ExoBleProtocolError extends Error {
  readonly code: ProtocolErrorCode;

  constructor(code: ProtocolErrorCode, message: string) {
    super(message);
    this.name = "ExoBleProtocolError";
    this.code = code;
  }
}

export interface DecodedSystemFlags {
  raw: number;
  initialized: boolean;
  readyToStart: boolean;
  runRequested: boolean;
  outputAllowed: boolean;
  maintenanceActive: boolean;
  executorAlive: boolean;
}

export interface DecodedJointFlags {
  raw: number;
  mappingValid: boolean;
  directionValid: boolean;
  zeroValid: boolean;
  feedbackValid: boolean;
  online: boolean;
  jointValid: boolean;
}

export interface DecodedGaitFlags {
  raw: number;
  unknown: number;
  overallValid: boolean;
  leftPhaseValid: boolean;
  rightPhaseValid: boolean;
  motionDetected: boolean;
  leftPhaseWrap: boolean;
  rightPhaseWrap: boolean;
}

export interface DecodedGaitQualityFlags {
  raw: number;
  unknown: number;
  leftInputInvalid: boolean;
  rightInputInvalid: boolean;
  lowMotion: boolean;
  leftWaitingEvent: boolean;
  rightWaitingEvent: boolean;
  leftPeriodOutOfRange: boolean;
  rightPeriodOutOfRange: boolean;
  periodUnstable: boolean;
  pairSkewHigh: boolean;
  timeOrSequenceRollback: boolean;
  coordinateReset: boolean;
  leftPhaseTimeout: boolean;
  rightPhaseTimeout: boolean;
  singleSideOnly: boolean;
  bilateralInconsistent: boolean;
}

export interface DecodedAssistFlags {
  raw: number;
  unknown: number;
  selectedValid: boolean;
  activeValid: boolean;
  locked: boolean;
  strategyReady: boolean;
  sensorsReady: boolean;
  suggestionValid: boolean;
  realOutputEnabled: boolean;
}

export interface DecodedSensorMask {
  raw: number;
  unknown: number;
  leftHipKinematics: boolean;
  rightHipKinematics: boolean;
  leftMotorTorqueEstimate: boolean;
  rightMotorTorqueEstimate: boolean;
  lowerLimbImu: boolean;
  footContact: boolean;
  interactionTorque: boolean;
  semg: boolean;
}

export interface CommonResponse {
  kind: ResponseKind;
  version: number;
  opcode: number;
  requestOpcode: number;
  requestId: number;
  payloadLength: number;
  raw: Uint8Array<ArrayBuffer>;
}

export interface AckResponse extends CommonResponse {
  kind: "ack";
  result: number;
  resultName: string;
  state: number;
  stateName: string;
  systemFlags: number;
  decodedSystemFlags: DecodedSystemFlags;
  detail: number;
}

export interface InfoResponse extends CommonResponse {
  kind: "info";
  protocolMinVersion: number;
  protocolMaxVersion: number;
  firmwareMajor: number;
  firmwareMinor: number;
  firmwarePatch: number;
  firmware: readonly [number, number, number];
  /** 设备原样声明的 16 位能力字；未知位也会保留以便向前兼容。 */
  capabilities: number;
  unknownCapabilities: number;
  maxFrameSize: number;
  bootId: number;
}

export interface SystemStatusResponse extends CommonResponse {
  kind: "systemStatus";
  state: number;
  stateName: string;
  activeStep: number;
  activeStepName: string;
  systemFlags: number;
  decodedSystemFlags: DecodedSystemFlags;
  startBlockReason: number;
  startBlockReasonName: string;
  lastRequestResult: number;
  lastRequestResultName: string;
  lastFailedStep: number;
  lastFailedStepName: string;
  transitionCount: number;
}

export interface JointStatusResponse extends CommonResponse {
  kind: "jointStatus";
  side: number;
  jointFlags: number;
  decodedJointFlags: DecodedJointFlags;
  direction: number;
  directionName: string;
  positionMrad: number;
  velocityMradS: number;
  positionDegrees: number;
  velocityDegreesPerSecond: number;
  feedbackAgeMs: number;
}

export interface ControlConfigResponse extends CommonResponse {
  kind: "controlConfig";
  selectedMode: number;
  selectedModeName: string;
  activeMode: number;
  activeModeName: string;
  selectedReference: number;
  selectedReferenceName: string;
  activeReference: number;
  activeReferenceName: string;
  configFlags: number;
  selectedValid: boolean;
  activeValid: boolean;
}

export interface JointCommandResponse extends CommonResponse {
  kind: "jointCommand";
  side: number;
  positionMrad: number;
  velocityMradS: number;
  torqueMNm: number;
  kpX100: number;
  kdX1000: number;
  positionDegrees: number;
  velocityDegreesPerSecond: number;
  torqueNm: number;
  kp: number;
  kd: number;
}

export interface GaitStatusResponse extends CommonResponse {
  kind: "gaitStatus";
  estimatorId: number;
  estimatorName: string;
  estimatorState: number;
  estimatorStateName: string;
  gaitFlags: number;
  decodedGaitFlags: DecodedGaitFlags;
  phaseOrigin: number;
  phaseOriginName: string;
  qualityFlags: number;
  decodedQualityFlags: DecodedGaitQualityFlags;
  leftPhaseX10000: number;
  rightPhaseX10000: number;
  leftPhase01: number | null;
  rightPhase01: number | null;
  leftPhasePercent: number | null;
  rightPhasePercent: number | null;
  cadenceX100: number;
  cadenceSpm: number | null;
  confidenceX10000: number;
  confidence01: number | null;
  confidencePercent: number | null;
  estimateAgeRawMs: number;
  estimateAgeMs: number | null;
}

export interface AssistConfigResponse extends CommonResponse {
  kind: "assistConfig";
  schemaVersion: number;
  selectedMode: number;
  selectedModeName: string;
  activeMode: number;
  activeModeName: string;
  packedExecutionStates: number;
  requestedExecutionState: number;
  requestedExecutionStateName: string;
  executionState: number;
  executionStateName: string;
  strategyId: number;
  strategyName: string;
  strategyVersion: number;
  profileId: number;
  leftLevelPercent: number;
  rightLevelPercent: number;
  assistFlags: number;
  decodedAssistFlags: DecodedAssistFlags;
  unavailableReason: number;
  unavailableReasonName: string;
  configGeneration: number;
  requiredSensorMask: number;
  availableSensorMask: number;
  decodedRequiredSensorMask: DecodedSensorMask;
  decodedAvailableSensorMask: DecodedSensorMask;
}

/** 网页表单使用人体关节单位；编码器会显式量化成 v1 定点线码。 */
export type JointCommandValues = {
  side: number;
  positionDegrees: number;
  velocityDegreesPerSecond: number;
  torqueNm: number;
  kp: number;
  kd: number;
};

export type AssistConfigValues = {
  schemaVersion?: number;
  selectedMode: number;
  requestedExecutionState: number;
  strategyId: number;
  profileId: number;
  leftLevelPercent: number;
  rightLevelPercent: number;
};

export type ExoBleResponse =
  | AckResponse
  | InfoResponse
  | SystemStatusResponse
  | JointStatusResponse
  | ControlConfigResponse
  | JointCommandResponse
  | GaitStatusResponse
  | AssistConfigResponse;

const responseOpcodes = Object.freeze({
  INFO: Opcode.GET_INFO | EXO_BLE.RESPONSE_BIT,
  SYSTEM_STATUS: Opcode.GET_SYSTEM_STATUS | EXO_BLE.RESPONSE_BIT,
  JOINT_STATUS: Opcode.GET_JOINT_STATUS | EXO_BLE.RESPONSE_BIT,
  CONTROL_CONFIG: Opcode.GET_CONTROL_CONFIG | EXO_BLE.RESPONSE_BIT,
  JOINT_COMMAND: Opcode.GET_JOINT_COMMAND | EXO_BLE.RESPONSE_BIT,
  GAIT_STATUS: Opcode.GET_GAIT_STATUS | EXO_BLE.RESPONSE_BIT,
  ASSIST_CONFIG: Opcode.GET_ASSIST_CONFIG | EXO_BLE.RESPONSE_BIT,
});

const requestPayloadLengths: Readonly<Record<number, number>> = Object.freeze({
  [Opcode.GET_INFO]: 0,
  [Opcode.GET_SYSTEM_STATUS]: 0,
  [Opcode.GET_JOINT_STATUS]: 1,
  [Opcode.GET_CONTROL_CONFIG]: 0,
  [Opcode.GET_JOINT_COMMAND]: 1,
  [Opcode.GET_GAIT_STATUS]: 0,
  [Opcode.GET_ASSIST_CONFIG]: 0,
  [Opcode.START]: 0,
  [Opcode.STOP]: 0,
  [Opcode.CAPTURE_ZERO]: 1,
  [Opcode.CLEAR_ZERO]: 1,
  [Opcode.SET_CONTROL_CONFIG]: 2,
  [Opcode.SET_JOINT_COMMAND]: 15,
  [Opcode.SET_ASSIST_CONFIG]: 8,
});

const sidePayloadOpcodes = new Set<number>([
  Opcode.GET_JOINT_STATUS,
  Opcode.GET_JOINT_COMMAND,
  Opcode.CAPTURE_ZERO,
  Opcode.CLEAR_ZERO,
]);

function fail(code: ProtocolErrorCode, message: string): never {
  throw new ExoBleProtocolError(code, message);
}

function copyBufferSource(
  source: BufferSource,
  label: string,
): Uint8Array<ArrayBuffer> {
  if (source instanceof ArrayBuffer) {
    return new Uint8Array(source.slice(0));
  }
  if (ArrayBuffer.isView(source)) {
    return new Uint8Array(
      source.buffer,
      source.byteOffset,
      source.byteLength,
    ).slice();
  }
  return fail("INVALID_SOURCE", `${label} must be an ArrayBuffer or ArrayBufferView`);
}

function requireIntegerInRange(
  value: number,
  minimum: number,
  maximum: number,
  label: string,
): void {
  if (!Number.isInteger(value) || value < minimum || value > maximum) {
    fail(
      "INVALID_ARGUMENT",
      `${label} must be an integer in ${minimum}..${maximum}`,
    );
  }
}

function hasNumericName(names: Readonly<Record<number, string>>, value: number): boolean {
  return Object.prototype.hasOwnProperty.call(names, value);
}

function requireNamedCode(
  value: number,
  names: Readonly<Record<number, string>>,
  label: string,
): string {
  if (!hasNumericName(names, value)) {
    fail("INVALID_FIELD", `${label} has unknown wire value 0x${value.toString(16).padStart(2, "0")}`);
  }
  return names[value];
}

/**
 * 设备响应中的枚举允许未来固件增加新值。网页保留原始线码并给出稳定的
 * UNKNOWN 名称，而 SET 编码仍通过 requireNamedCode() 严格限制当前协议值。
 */
function wireNameOrUnknown(
  names: Readonly<Record<number, string>>,
  value: number,
): string {
  return hasNumericName(names, value)
    ? names[value]
    : `UNKNOWN(0x${value.toString(16).padStart(2, "0")})`;
}

function requireFlags(raw: number, mask: number, label: string): void {
  requireIntegerInRange(raw, 0, 0xff, label);
  if ((raw & ~mask) !== 0) {
    fail("INVALID_FIELD", `${label} contains reserved bits: 0x${raw.toString(16).padStart(2, "0")}`);
  }
}

export function decodeSystemFlags(raw: number): DecodedSystemFlags {
  requireFlags(raw, EXO_BLE.SYSTEM_FLAGS_MASK, "system flags");
  return {
    raw,
    initialized: (raw & (1 << 0)) !== 0,
    readyToStart: (raw & (1 << 1)) !== 0,
    runRequested: (raw & (1 << 2)) !== 0,
    outputAllowed: (raw & (1 << 3)) !== 0,
    maintenanceActive: (raw & (1 << 4)) !== 0,
    executorAlive: (raw & (1 << 5)) !== 0,
  };
}

export function decodeJointFlags(raw: number): DecodedJointFlags {
  requireFlags(raw, EXO_BLE.JOINT_FLAGS_MASK, "joint flags");
  return {
    raw,
    mappingValid: (raw & (1 << 0)) !== 0,
    directionValid: (raw & (1 << 1)) !== 0,
    zeroValid: (raw & (1 << 2)) !== 0,
    feedbackValid: (raw & (1 << 3)) !== 0,
    online: (raw & (1 << 4)) !== 0,
    jointValid: (raw & (1 << 5)) !== 0,
  };
}

export function decodeGaitFlags(raw: number): DecodedGaitFlags {
  requireIntegerInRange(raw, 0, 0xff, "gait flags");
  return {
    raw,
    unknown: raw & ~EXO_BLE.GAIT_FLAGS_MASK,
    overallValid: (raw & (1 << 0)) !== 0,
    leftPhaseValid: (raw & (1 << 1)) !== 0,
    rightPhaseValid: (raw & (1 << 2)) !== 0,
    motionDetected: (raw & (1 << 3)) !== 0,
    leftPhaseWrap: (raw & (1 << 4)) !== 0,
    rightPhaseWrap: (raw & (1 << 5)) !== 0,
  };
}

export function decodeGaitQualityFlags(raw: number): DecodedGaitQualityFlags {
  requireIntegerInRange(raw, 0, 0xffff, "gait quality flags");
  return {
    raw,
    unknown: raw & ~EXO_BLE.GAIT_QUALITY_FLAGS_MASK,
    leftInputInvalid: (raw & (1 << 0)) !== 0,
    rightInputInvalid: (raw & (1 << 1)) !== 0,
    lowMotion: (raw & (1 << 2)) !== 0,
    leftWaitingEvent: (raw & (1 << 3)) !== 0,
    rightWaitingEvent: (raw & (1 << 4)) !== 0,
    leftPeriodOutOfRange: (raw & (1 << 5)) !== 0,
    rightPeriodOutOfRange: (raw & (1 << 6)) !== 0,
    periodUnstable: (raw & (1 << 7)) !== 0,
    pairSkewHigh: (raw & (1 << 8)) !== 0,
    timeOrSequenceRollback: (raw & (1 << 9)) !== 0,
    coordinateReset: (raw & (1 << 10)) !== 0,
    leftPhaseTimeout: (raw & (1 << 11)) !== 0,
    rightPhaseTimeout: (raw & (1 << 12)) !== 0,
    singleSideOnly: (raw & (1 << 13)) !== 0,
    bilateralInconsistent: (raw & (1 << 14)) !== 0,
  };
}

export function decodeAssistFlags(raw: number): DecodedAssistFlags {
  requireIntegerInRange(raw, 0, 0xff, "assist flags");
  return {
    raw,
    unknown: raw & ~EXO_BLE.ASSIST_FLAGS_MASK,
    selectedValid: (raw & (1 << 0)) !== 0,
    activeValid: (raw & (1 << 1)) !== 0,
    locked: (raw & (1 << 2)) !== 0,
    strategyReady: (raw & (1 << 3)) !== 0,
    sensorsReady: (raw & (1 << 4)) !== 0,
    suggestionValid: (raw & (1 << 5)) !== 0,
    realOutputEnabled: (raw & (1 << 6)) !== 0,
  };
}

export function decodeSensorMask(raw: number): DecodedSensorMask {
  requireIntegerInRange(raw, 0, 0xffff, "sensor mask");
  return {
    raw,
    unknown: raw & ~EXO_BLE.SENSOR_MASK,
    leftHipKinematics: (raw & (1 << 0)) !== 0,
    rightHipKinematics: (raw & (1 << 1)) !== 0,
    leftMotorTorqueEstimate: (raw & (1 << 2)) !== 0,
    rightMotorTorqueEstimate: (raw & (1 << 3)) !== 0,
    lowerLimbImu: (raw & (1 << 4)) !== 0,
    footContact: (raw & (1 << 5)) !== 0,
    interactionTorque: (raw & (1 << 6)) !== 0,
    semg: (raw & (1 << 7)) !== 0,
  };
}

function formatBytes(bytes: Uint8Array): string {
  return Array.from(
    bytes,
    (byte) => byte.toString(16).padStart(2, "0").toUpperCase(),
  ).join(" ");
}

export function toHex(source: BufferSource): string {
  return formatBytes(copyBufferSource(source, "bytes"));
}

export function mradToDegrees(value: number): number {
  if (!Number.isFinite(value)) {
    fail("INVALID_ARGUMENT", "mrad value must be finite");
  }
  return value * 180.0 / (Math.PI * 1000.0);
}

function requireFiniteInRange(
  value: number,
  minimum: number,
  maximum: number,
  label: string,
): void {
  if (!Number.isFinite(value) || value < minimum || value > maximum) {
    fail("INVALID_ARGUMENT", `${label} must be finite and in ${minimum}..${maximum}`);
  }
}

/** 把网页显示的度量化为协议 mrad；转换边界集中在此，UI 不手写 57.3 系数。 */
export function degreesToMrad(value: number): number {
  requireFiniteInRange(
    value,
    mradToDegrees(EXO_BLE.JOINT_POSITION_MRAD_MIN),
    mradToDegrees(EXO_BLE.JOINT_POSITION_MRAD_MAX),
    "position degrees",
  );
  return Math.round(value * Math.PI * 1000.0 / 180.0);
}

function degreesPerSecondToMradPerSecond(value: number): number {
  requireFiniteInRange(
    value,
    mradToDegrees(EXO_BLE.JOINT_VELOCITY_MRAD_S_MIN),
    mradToDegrees(EXO_BLE.JOINT_VELOCITY_MRAD_S_MAX),
    "velocity degrees per second",
  );
  return Math.round(value * Math.PI * 1000.0 / 180.0);
}

function selectedControlModeName(value: number, label: string): string {
  if (value === ControlMode.NONE) {
    fail("INVALID_FIELD", `${label} cannot be NONE`);
  }
  return requireNamedCode(value, controlModeNames, label);
}

export function encodeControlConfigPayload(
  selectedMode: number,
  selectedReference: number,
): Uint8Array<ArrayBuffer> {
  requireIntegerInRange(selectedMode, ControlMode.HOLD, ControlMode.IMPEDANCE, "selected mode");
  selectedControlModeName(selectedMode, "selected mode");
  requireNamedCode(selectedReference, referenceSourceNames, "selected reference");
  // 固件阶段 8 只给 IMPEDANCE 定义了 SINE 参考。把组合约束放在公共编码器里，
  // 即使以后出现第二个调用页面，也不会发送一个已知会被设备拒绝的组合。
  if (selectedReference === ReferenceSource.SINE &&
      selectedMode !== ControlMode.IMPEDANCE) {
    fail("INVALID_ARGUMENT", "SINE reference requires IMPEDANCE mode");
  }
  return Uint8Array.of(selectedMode, selectedReference);
}

type AssistConfigWireValues = {
  schemaVersion: number;
  selectedMode: number;
  requestedExecutionState: number;
  strategyId: number;
  profileId: number;
  leftLevelPercent: number;
  rightLevelPercent: number;
  reservedFlags: number;
};

function validateAssistConfigWire(values: AssistConfigWireValues): void {
  if (values.schemaVersion !== EXO_BLE.ASSIST_CONFIG_SCHEMA_VERSION) {
    fail(
      "INVALID_FIELD",
      `assist schema must be ${EXO_BLE.ASSIST_CONFIG_SCHEMA_VERSION}`,
    );
  }
  requireNamedCode(values.selectedMode, assistModeNames, "selected assist mode");
  requireNamedCode(
    values.requestedExecutionState,
    assistExecutionStateNames,
    "requested assist execution state",
  );
  requireNamedCode(values.strategyId, assistStrategyNames, "assist strategy");
  // NONE 只会出现在状态读回；SET 必须选择一个真实产品模式和策略。
  // ACTIVE 是稳定线码，协议层必须允许编码，以便客户端正确解析设备返回的
  // REJECTED_NOT_READY/detail=3。阶段 9 的普通 UI 仍然只展示 DISABLED/SHADOW。
  if (values.selectedMode === AssistMode.NONE) {
    fail("INVALID_FIELD", "selected assist mode cannot be NONE");
  }
  if (values.strategyId === AssistStrategy.NONE) {
    fail("INVALID_FIELD", "assist strategy cannot be NONE");
  }
  const expectedModeByStrategy: Readonly<Record<number, number>> = {
    [AssistStrategy.TRANSPARENT]: AssistMode.TRANSPARENT,
    [AssistStrategy.PHASE_PROFILE]: AssistMode.ACTIVE_PHASE,
    [AssistStrategy.AAN_RESIDUAL]: AssistMode.ASSIST_AS_NEEDED,
  };
  if (expectedModeByStrategy[values.strategyId] !== values.selectedMode) {
    fail("INVALID_FIELD", "assist mode and strategy do not match");
  }
  requireIntegerInRange(values.profileId, 0, 0xff, "assist profile id");
  requireIntegerInRange(
    values.leftLevelPercent,
    0,
    EXO_BLE.ASSIST_LEVEL_PERCENT_MAX,
    "left assist level percent",
  );
  requireIntegerInRange(
    values.rightLevelPercent,
    0,
    EXO_BLE.ASSIST_LEVEL_PERCENT_MAX,
    "right assist level percent",
  );
  if (values.reservedFlags !== 0) {
    fail("INVALID_FIELD", "assist reserved flags must be zero");
  }
}

function readAssistConfigWire(view: DataView, offset: number): AssistConfigWireValues {
  const values = {
    schemaVersion: view.getUint8(offset),
    selectedMode: view.getUint8(offset + 1),
    requestedExecutionState: view.getUint8(offset + 2),
    strategyId: view.getUint8(offset + 3),
    profileId: view.getUint8(offset + 4),
    leftLevelPercent: view.getUint8(offset + 5),
    rightLevelPercent: view.getUint8(offset + 6),
    reservedFlags: view.getUint8(offset + 7),
  };
  validateAssistConfigWire(values);
  return values;
}

/** 编码阶段 9 的 8 字节助力配置；保留字节始终由公共编码器写零。 */
export function encodeAssistConfigPayload(
  values: AssistConfigValues,
): Uint8Array<ArrayBuffer> {
  const wire: AssistConfigWireValues = {
    schemaVersion: values.schemaVersion ?? EXO_BLE.ASSIST_CONFIG_SCHEMA_VERSION,
    selectedMode: values.selectedMode,
    requestedExecutionState: values.requestedExecutionState,
    strategyId: values.strategyId,
    profileId: values.profileId,
    leftLevelPercent: values.leftLevelPercent,
    rightLevelPercent: values.rightLevelPercent,
    reservedFlags: 0,
  };
  validateAssistConfigWire(wire);
  return Uint8Array.of(
    wire.schemaVersion,
    wire.selectedMode,
    wire.requestedExecutionState,
    wire.strategyId,
    wire.profileId,
    wire.leftLevelPercent,
    wire.rightLevelPercent,
    wire.reservedFlags,
  );
}

type JointCommandWireValues = {
  side: number;
  positionMrad: number;
  velocityMradS: number;
  torqueMNm: number;
  kpX100: number;
  kdX1000: number;
};

function validateJointCommandWire(values: JointCommandWireValues): void {
  if (values.side !== Side.LEFT && values.side !== Side.RIGHT) {
    fail("INVALID_FIELD", `joint command side has invalid wire value ${values.side}`);
  }
  requireIntegerInRange(
    values.positionMrad,
    EXO_BLE.JOINT_POSITION_MRAD_MIN,
    EXO_BLE.JOINT_POSITION_MRAD_MAX,
    "position mrad",
  );
  requireIntegerInRange(
    values.velocityMradS,
    EXO_BLE.JOINT_VELOCITY_MRAD_S_MIN,
    EXO_BLE.JOINT_VELOCITY_MRAD_S_MAX,
    "velocity mrad/s",
  );
  requireIntegerInRange(
    values.torqueMNm,
    EXO_BLE.JOINT_TORQUE_MNM_MIN,
    EXO_BLE.JOINT_TORQUE_MNM_MAX,
    "torque mN·m",
  );
  requireIntegerInRange(values.kpX100, 0, EXO_BLE.JOINT_KP_X100_MAX, "KP x100");
  requireIntegerInRange(values.kdX1000, 0, EXO_BLE.JOINT_KD_X1000_MAX, "KD x1000");
}

function readJointCommandWire(view: DataView, offset: number): JointCommandWireValues {
  const values = {
    side: view.getUint8(offset),
    positionMrad: view.getInt32(offset + 1, true),
    velocityMradS: view.getInt32(offset + 5, true),
    torqueMNm: view.getInt16(offset + 9, true),
    kpX100: view.getUint16(offset + 11, true),
    kdX1000: view.getUint16(offset + 13, true),
  };
  validateJointCommandWire(values);
  return values;
}

export function encodeJointCommandPayload(
  command: JointCommandValues,
): Uint8Array<ArrayBuffer> {
  if (command.side !== Side.LEFT && command.side !== Side.RIGHT) {
    fail("INVALID_ARGUMENT", `side must be LEFT or RIGHT, received ${command.side}`);
  }
  const positionMrad = degreesToMrad(command.positionDegrees);
  const velocityMradS = degreesPerSecondToMradPerSecond(
    command.velocityDegreesPerSecond,
  );
  requireFiniteInRange(command.torqueNm, -17, 17, "torque N·m");
  requireFiniteInRange(command.kp, 0, 500, "KP");
  requireFiniteInRange(command.kd, 0, 5, "KD");
  const wire: JointCommandWireValues = {
    side: command.side,
    positionMrad,
    velocityMradS,
    torqueMNm: Math.round(command.torqueNm * 1000),
    kpX100: Math.round(command.kp * 100),
    kdX1000: Math.round(command.kd * 1000),
  };
  validateJointCommandWire(wire);

  const payload = new Uint8Array(15);
  const view = new DataView(payload.buffer);
  view.setUint8(0, wire.side);
  view.setInt32(1, wire.positionMrad, true);
  view.setInt32(5, wire.velocityMradS, true);
  view.setInt16(9, wire.torqueMNm, true);
  view.setUint16(11, wire.kpX100, true);
  view.setUint16(13, wire.kdX1000, true);
  return payload;
}

export function encodeRequest(
  opcode: number,
  requestId: number,
  payload: BufferSource = new Uint8Array(0),
): Uint8Array<ArrayBuffer> {
  requireIntegerInRange(opcode, 0, 0xff, "opcode");
  requireIntegerInRange(requestId, 1, 0xff, "request id");

  if (!Object.prototype.hasOwnProperty.call(requestPayloadLengths, opcode)) {
    fail("UNKNOWN_OPCODE", `unsupported request opcode 0x${opcode.toString(16).padStart(2, "0")}`);
  }

  const payloadBytes = copyBufferSource(payload, "payload");
  if (payloadBytes.length > EXO_BLE.MAX_PAYLOAD_SIZE) {
    fail("INVALID_PAYLOAD", "payload exceeds the 16-byte v1 limit");
  }
  const expectedLength = requestPayloadLengths[opcode];
  if (payloadBytes.length !== expectedLength) {
    fail(
      "INVALID_PAYLOAD",
      `opcode 0x${opcode.toString(16).padStart(2, "0")} requires ${expectedLength} payload byte(s)`,
    );
  }
  if (sidePayloadOpcodes.has(opcode) &&
      payloadBytes[0] !== Side.LEFT && payloadBytes[0] !== Side.RIGHT) {
    fail("INVALID_PAYLOAD", `invalid side value ${payloadBytes[0]}`);
  }
  if (opcode === Opcode.SET_CONTROL_CONFIG) {
    selectedControlModeName(payloadBytes[0], "selected mode");
    requireNamedCode(payloadBytes[1], referenceSourceNames, "selected reference");
  }
  if (opcode === Opcode.SET_JOINT_COMMAND) {
    try {
      readJointCommandWire(new DataView(payloadBytes.buffer), 0);
    } catch (caught) {
      if (caught instanceof ExoBleProtocolError) {
        fail("INVALID_PAYLOAD", caught.message);
      }
      throw caught;
    }
  }
  if (opcode === Opcode.SET_ASSIST_CONFIG) {
    try {
      readAssistConfigWire(new DataView(
        payloadBytes.buffer,
        payloadBytes.byteOffset,
        payloadBytes.byteLength,
      ), 0);
    } catch (caught) {
      if (caught instanceof ExoBleProtocolError) {
        fail("INVALID_PAYLOAD", caught.message);
      }
      throw caught;
    }
  }

  const frame = new Uint8Array(EXO_BLE.HEADER_SIZE + payloadBytes.length);
  frame[0] = EXO_BLE.PROTOCOL_VERSION;
  frame[1] = opcode;
  frame[2] = requestId;
  frame[3] = payloadBytes.length;
  frame.set(payloadBytes, EXO_BLE.HEADER_SIZE);
  return frame;
}

interface ParsedHeader {
  version: number;
  opcode: number;
  requestOpcode: number;
  requestId: number;
  payloadLength: number;
  raw: Uint8Array<ArrayBuffer>;
  view: DataView;
}

function parseHeader(source: BufferSource): ParsedHeader {
  const raw = copyBufferSource(source, "response");
  if (raw.length < EXO_BLE.HEADER_SIZE || raw.length > EXO_BLE.MAX_FRAME_SIZE) {
    fail(
      "INVALID_FRAME_LENGTH",
      `frame length ${raw.length} is outside ${EXO_BLE.HEADER_SIZE}..${EXO_BLE.MAX_FRAME_SIZE}`,
    );
  }

  const version = raw[0];
  const opcode = raw[1];
  const requestId = raw[2];
  const payloadLength = raw[3];
  if (version !== EXO_BLE.PROTOCOL_VERSION) {
    fail("UNSUPPORTED_VERSION", `unsupported protocol version ${version}`);
  }
  if (payloadLength > EXO_BLE.MAX_PAYLOAD_SIZE) {
    fail("INVALID_FRAME_LENGTH", `payload length ${payloadLength} exceeds 16 bytes`);
  }
  if (raw.length !== EXO_BLE.HEADER_SIZE + payloadLength) {
    fail(
      "INVALID_FRAME_LENGTH",
      `declared payload length ${payloadLength} does not match frame length ${raw.length}`,
    );
  }
  if ((opcode & EXO_BLE.RESPONSE_BIT) === 0) {
    fail("NOT_A_RESPONSE", `opcode 0x${opcode.toString(16).padStart(2, "0")} is not a response`);
  }

  return {
    version,
    opcode,
    requestOpcode: opcode & ~EXO_BLE.RESPONSE_BIT,
    requestId,
    payloadLength,
    raw,
    view: new DataView(raw.buffer, raw.byteOffset, raw.byteLength),
  };
}

function commonResponse(header: ParsedHeader, kind: ResponseKind): CommonResponse {
  return {
    kind,
    version: header.version,
    opcode: header.opcode,
    requestOpcode: header.requestOpcode,
    requestId: header.requestId,
    payloadLength: header.payloadLength,
    raw: header.raw,
  };
}

function parseAck(header: ParsedHeader): AckResponse {
  const result = header.raw[4];
  const state = header.raw[5];
  const systemFlags = header.raw[6];
  if (result === 0x7f) {
    fail("INVALID_FIELD", "ACK result cannot be NONE");
  }
  return {
    ...commonResponse(header, "ack"),
    kind: "ack",
    result,
    resultName: requireNamedCode(result, resultNames, "ACK result"),
    state,
    stateName: requireNamedCode(state, stateNames, "ACK state"),
    systemFlags,
    decodedSystemFlags: decodeSystemFlags(systemFlags),
    detail: header.raw[7],
  };
}

function parseInfo(header: ParsedHeader): InfoResponse {
  const protocolMinVersion = header.raw[4];
  const protocolMaxVersion = header.raw[5];
  const capabilities = header.view.getUint16(9, true);
  const maxFrameSize = header.raw[11];
  if (protocolMinVersion === 0 ||
      protocolMinVersion > EXO_BLE.PROTOCOL_VERSION ||
      protocolMaxVersion < EXO_BLE.PROTOCOL_VERSION) {
    fail(
      "INVALID_FIELD",
      `INFO protocol range ${protocolMinVersion}..${protocolMaxVersion} does not contain v1`,
    );
  }
  if (maxFrameSize !== EXO_BLE.MAX_FRAME_SIZE) {
    fail("INVALID_FIELD", `INFO max frame size must be ${EXO_BLE.MAX_FRAME_SIZE}`);
  }
  const firmware = Object.freeze([
    header.raw[6],
    header.raw[7],
    header.raw[8],
  ]) as readonly [number, number, number];
  return {
    ...commonResponse(header, "info"),
    kind: "info",
    protocolMinVersion,
    protocolMaxVersion,
    firmwareMajor: firmware[0],
    firmwareMinor: firmware[1],
    firmwarePatch: firmware[2],
    firmware,
    capabilities,
    unknownCapabilities: capabilities & ~EXO_BLE.CAPABILITIES_MASK,
    maxFrameSize,
    bootId: header.view.getUint32(12, true),
  };
}

function parseSystemStatus(header: ParsedHeader): SystemStatusResponse {
  const state = header.raw[4];
  const activeStep = header.raw[5];
  const systemFlags = header.raw[6];
  const startBlockReason = header.raw[7];
  const lastRequestResult = header.raw[8];
  const lastFailedStep = header.raw[9];
  return {
    ...commonResponse(header, "systemStatus"),
    kind: "systemStatus",
    state,
    stateName: requireNamedCode(state, stateNames, "system state"),
    activeStep,
    activeStepName: requireNamedCode(activeStep, stepNames, "active step"),
    systemFlags,
    decodedSystemFlags: decodeSystemFlags(systemFlags),
    startBlockReason,
    startBlockReasonName: requireNamedCode(
      startBlockReason,
      blockReasonNames,
      "start block reason",
    ),
    lastRequestResult,
    lastRequestResultName: requireNamedCode(
      lastRequestResult,
      resultNames,
      "last request result",
    ),
    lastFailedStep,
    lastFailedStepName: requireNamedCode(
      lastFailedStep,
      stepNames,
      "last failed step",
    ),
    transitionCount: header.view.getUint32(10, true),
  };
}

function parseJointStatus(header: ParsedHeader): JointStatusResponse {
  const side = header.raw[4];
  if (side !== Side.LEFT && side !== Side.RIGHT) {
    fail("INVALID_FIELD", `joint side has invalid wire value ${side}`);
  }
  const jointFlags = header.raw[5];
  const direction = header.raw[6];
  const positionMrad = header.view.getInt32(7, true);
  const velocityMradS = header.view.getInt32(11, true);
  return {
    ...commonResponse(header, "jointStatus"),
    kind: "jointStatus",
    side,
    jointFlags,
    decodedJointFlags: decodeJointFlags(jointFlags),
    direction,
    directionName: requireNamedCode(direction, directionNames, "joint direction"),
    positionMrad,
    velocityMradS,
    positionDegrees: mradToDegrees(positionMrad),
    velocityDegreesPerSecond: mradToDegrees(velocityMradS),
    feedbackAgeMs: header.view.getUint16(15, true),
  };
}

function parseControlConfig(header: ParsedHeader): ControlConfigResponse {
  const selectedMode = header.raw[4];
  const activeMode = header.raw[5];
  const selectedReference = header.raw[6];
  const activeReference = header.raw[7];
  const configFlags = header.raw[8];
  requireFlags(configFlags, EXO_BLE.CONTROL_CONFIG_FLAGS_MASK, "control config flags");
  const selectedValid = (configFlags & (1 << 0)) !== 0;
  const activeValid = (configFlags & (1 << 1)) !== 0;
  if (selectedValid) selectedControlModeName(selectedMode, "selected mode");
  else if (selectedMode !== ControlMode.NONE) {
    fail("INVALID_FIELD", "selected mode must be NONE when selected_valid is clear");
  }
  if (activeValid) selectedControlModeName(activeMode, "active mode");
  else if (activeMode !== ControlMode.NONE) {
    fail("INVALID_FIELD", "active mode must be NONE when active_valid is clear");
  }
  if (activeValid && !selectedValid) {
    fail("INVALID_FIELD", "active control config cannot exist without selected config");
  }
  return {
    ...commonResponse(header, "controlConfig"),
    kind: "controlConfig",
    selectedMode,
    selectedModeName: requireNamedCode(selectedMode, controlModeNames, "selected mode"),
    activeMode,
    activeModeName: requireNamedCode(activeMode, controlModeNames, "active mode"),
    selectedReference,
    selectedReferenceName: requireNamedCode(
      selectedReference,
      referenceSourceNames,
      "selected reference",
    ),
    activeReference,
    activeReferenceName: requireNamedCode(
      activeReference,
      referenceSourceNames,
      "active reference",
    ),
    configFlags,
    selectedValid,
    activeValid,
  };
}

function parseJointCommand(header: ParsedHeader): JointCommandResponse {
  const wire = readJointCommandWire(header.view, 4);
  return {
    ...commonResponse(header, "jointCommand"),
    kind: "jointCommand",
    ...wire,
    positionDegrees: mradToDegrees(wire.positionMrad),
    velocityDegreesPerSecond: mradToDegrees(wire.velocityMradS),
    torqueNm: wire.torqueMNm / 1000,
    kp: wire.kpX100 / 100,
    kd: wire.kdX1000 / 1000,
  };
}

function parsePhaseX10000(
  raw: number,
  valid: boolean,
  label: string,
): { phase01: number | null; phasePercent: number | null } {
  if (!valid) {
    if (raw !== EXO_BLE.INVALID_U16) {
      fail("INVALID_FIELD", `${label} must use 0xffff when its valid flag is clear`);
    }
    return { phase01: null, phasePercent: null };
  }
  if (raw > EXO_BLE.PHASE_X10000_MAX) {
    fail(
      "INVALID_FIELD",
      `${label} must be in 0..${EXO_BLE.PHASE_X10000_MAX} when valid`,
    );
  }
  return { phase01: raw / 10000, phasePercent: raw / 100 };
}

function parseGaitStatus(header: ParsedHeader): GaitStatusResponse {
  const estimatorId = header.raw[4];
  const estimatorState = header.raw[5];
  const gaitFlags = header.raw[6];
  const phaseOrigin = header.raw[7];
  const qualityFlags = header.view.getUint16(8, true);
  const leftPhaseX10000 = header.view.getUint16(10, true);
  const rightPhaseX10000 = header.view.getUint16(12, true);
  const cadenceX100 = header.view.getUint16(14, true);
  const confidenceX10000 = header.view.getUint16(16, true);
  const estimateAgeRawMs = header.view.getUint16(18, true);
  const decodedGaitFlags = decodeGaitFlags(gaitFlags);
  if (decodedGaitFlags.overallValid &&
      (!decodedGaitFlags.leftPhaseValid || !decodedGaitFlags.rightPhaseValid)) {
    fail("INVALID_FIELD", "overall gait validity requires both phase-valid flags");
  }
  const leftPhase = parsePhaseX10000(
    leftPhaseX10000,
    decodedGaitFlags.leftPhaseValid,
    "left phase",
  );
  const rightPhase = parsePhaseX10000(
    rightPhaseX10000,
    decodedGaitFlags.rightPhaseValid,
    "right phase",
  );
  // 阶段 9 的定点状态量统一使用 0xffff 表示“当前无有效估计”。
  // 其他超范围数值不是未来扩展值，而是损坏/不符合协议的帧，必须显式拒绝，
  // 避免网页把异常数据误显示成真实步频或置信度。
  if (cadenceX100 !== EXO_BLE.INVALID_U16 &&
      cadenceX100 > EXO_BLE.CADENCE_X100_MAX) {
    fail(
      "INVALID_FIELD",
      `cadence must be 0..${EXO_BLE.CADENCE_X100_MAX} or 0xffff`,
    );
  }
  if (confidenceX10000 !== EXO_BLE.INVALID_U16 &&
      confidenceX10000 > EXO_BLE.CONFIDENCE_X10000_MAX) {
    fail(
      "INVALID_FIELD",
      `confidence must be 0..${EXO_BLE.CONFIDENCE_X10000_MAX} or 0xffff`,
    );
  }
  const cadenceValid = cadenceX100 !== EXO_BLE.INVALID_U16;
  const confidenceValid = confidenceX10000 !== EXO_BLE.INVALID_U16;
  return {
    ...commonResponse(header, "gaitStatus"),
    kind: "gaitStatus",
    estimatorId,
    estimatorName: wireNameOrUnknown(gaitEstimatorNames, estimatorId),
    estimatorState,
    estimatorStateName: wireNameOrUnknown(gaitEstimatorStateNames, estimatorState),
    gaitFlags,
    decodedGaitFlags,
    phaseOrigin,
    phaseOriginName: wireNameOrUnknown(gaitPhaseOriginNames, phaseOrigin),
    qualityFlags,
    decodedQualityFlags: decodeGaitQualityFlags(qualityFlags),
    leftPhaseX10000,
    rightPhaseX10000,
    leftPhase01: leftPhase.phase01,
    rightPhase01: rightPhase.phase01,
    leftPhasePercent: leftPhase.phasePercent,
    rightPhasePercent: rightPhase.phasePercent,
    cadenceX100,
    cadenceSpm: cadenceValid ? cadenceX100 / 100 : null,
    confidenceX10000,
    confidence01: confidenceValid ? confidenceX10000 / 10000 : null,
    confidencePercent: confidenceValid ? confidenceX10000 / 100 : null,
    estimateAgeRawMs,
    estimateAgeMs: estimateAgeRawMs === EXO_BLE.INVALID_U16 ? null : estimateAgeRawMs,
  };
}

function parseAssistConfig(header: ParsedHeader): AssistConfigResponse {
  const schemaVersion = header.raw[4];
  const selectedMode = header.raw[5];
  const activeMode = header.raw[6];
  const packedExecutionStates = header.raw[7];
  if ((packedExecutionStates & EXO_BLE.ASSIST_EXECUTION_PACKED_RESERVED_MASK) !== 0) {
    fail("INVALID_FIELD", "assist packed execution state uses reserved bits");
  }
  const requestedExecutionState = packedExecutionStates & 0x03;
  const executionState = (packedExecutionStates >> 4) & 0x03;
  const requestedExecutionStateName = requireNamedCode(
    requestedExecutionState,
    assistExecutionStateNames,
    "requested assist execution state",
  );
  const executionStateName = requireNamedCode(
    executionState,
    assistExecutionStateNames,
    "effective assist execution state",
  );
  const strategyId = header.raw[8];
  const leftLevelPercent = header.raw[11];
  const rightLevelPercent = header.raw[12];
  const assistFlags = header.raw[13];
  const decodedAssistFlags = decodeAssistFlags(assistFlags);
  if (schemaVersion !== EXO_BLE.ASSIST_CONFIG_SCHEMA_VERSION) {
    fail(
      "INVALID_FIELD",
      `assist schema must be ${EXO_BLE.ASSIST_CONFIG_SCHEMA_VERSION}`,
    );
  }
  if (leftLevelPercent > EXO_BLE.ASSIST_LEVEL_PERCENT_MAX ||
      rightLevelPercent > EXO_BLE.ASSIST_LEVEL_PERCENT_MAX) {
    fail("INVALID_FIELD", "assist level percent must be in 0..100");
  }
  if (!decodedAssistFlags.selectedValid && selectedMode !== AssistMode.NONE) {
    fail("INVALID_FIELD", "selected assist mode must be NONE when selected_valid is clear");
  }
  if (!decodedAssistFlags.activeValid && activeMode !== AssistMode.NONE) {
    fail("INVALID_FIELD", "active assist mode must be NONE when active_valid is clear");
  }
  if (decodedAssistFlags.activeValid && !decodedAssistFlags.selectedValid) {
    fail("INVALID_FIELD", "active assist config cannot exist without selected config");
  }
  if (decodedAssistFlags.realOutputEnabled &&
      (!decodedAssistFlags.activeValid || executionState !== AssistExecutionState.ACTIVE)) {
    fail("INVALID_FIELD", "real output requires ACTIVE execution and an active config");
  }
  const unavailableReason = header.raw[14];
  const requiredSensorMask = header.view.getUint16(16, true);
  const availableSensorMask = header.view.getUint16(18, true);
  return {
    ...commonResponse(header, "assistConfig"),
    kind: "assistConfig",
    schemaVersion,
    selectedMode,
    selectedModeName: wireNameOrUnknown(assistModeNames, selectedMode),
    activeMode,
    activeModeName: wireNameOrUnknown(assistModeNames, activeMode),
    packedExecutionStates,
    requestedExecutionState,
    requestedExecutionStateName,
    executionState,
    executionStateName,
    strategyId,
    strategyName: wireNameOrUnknown(assistStrategyNames, strategyId),
    strategyVersion: header.raw[9],
    profileId: header.raw[10],
    leftLevelPercent,
    rightLevelPercent,
    assistFlags,
    decodedAssistFlags,
    unavailableReason,
    unavailableReasonName: wireNameOrUnknown(
      assistUnavailableReasonNames,
      unavailableReason,
    ),
    configGeneration: header.raw[15],
    requiredSensorMask,
    availableSensorMask,
    decodedRequiredSensorMask: decodeSensorMask(requiredSensorMask),
    decodedAvailableSensorMask: decodeSensorMask(availableSensorMask),
  };
}

export function parseResponse(source: BufferSource): ExoBleResponse {
  const header = parseHeader(source);

  // Every v1 error and every command acknowledgement has a four-byte payload,
  // including errors returned with GET_* response opcodes.
  if (header.payloadLength === 4) {
    return parseAck(header);
  }
  if (header.requestId === 0) {
    fail("INVALID_FIELD", "structured responses require a non-zero request id");
  }

  switch (header.opcode) {
    case responseOpcodes.INFO:
      if (header.payloadLength !== 12) {
        fail("INVALID_FRAME_LENGTH", "INFO payload must be 12 bytes or a 4-byte ACK");
      }
      return parseInfo(header);
    case responseOpcodes.SYSTEM_STATUS:
      if (header.payloadLength !== 10) {
        fail("INVALID_FRAME_LENGTH", "SYSTEM_STATUS payload must be 10 bytes or a 4-byte ACK");
      }
      return parseSystemStatus(header);
    case responseOpcodes.JOINT_STATUS:
      if (header.payloadLength !== 13) {
        fail("INVALID_FRAME_LENGTH", "JOINT_STATUS payload must be 13 bytes or a 4-byte ACK");
      }
      return parseJointStatus(header);
    case responseOpcodes.CONTROL_CONFIG:
      if (header.payloadLength !== 5) {
        fail("INVALID_FRAME_LENGTH", "CONTROL_CONFIG payload must be 5 bytes or a 4-byte ACK");
      }
      return parseControlConfig(header);
    case responseOpcodes.JOINT_COMMAND:
      if (header.payloadLength !== 15) {
        fail("INVALID_FRAME_LENGTH", "JOINT_COMMAND payload must be 15 bytes or a 4-byte ACK");
      }
      return parseJointCommand(header);
    case responseOpcodes.GAIT_STATUS:
      if (header.payloadLength !== 16) {
        fail("INVALID_FRAME_LENGTH", "GAIT_STATUS payload must be 16 bytes or a 4-byte ACK");
      }
      return parseGaitStatus(header);
    case responseOpcodes.ASSIST_CONFIG:
      if (header.payloadLength !== 16) {
        fail("INVALID_FRAME_LENGTH", "ASSIST_CONFIG payload must be 16 bytes or a 4-byte ACK");
      }
      return parseAssistConfig(header);
    default:
      return fail(
        "UNKNOWN_RESPONSE",
        `unsupported response opcode 0x${header.opcode.toString(16).padStart(2, "0")}`,
      );
  }
}

export function assertCorrelated(
  response: ExoBleResponse,
  requestOpcode: number,
  requestId: number,
): void {
  requireIntegerInRange(requestOpcode, 0, 0xff, "request opcode");
  // Zero is useful when checking the INVALID_REQUEST_ID response to a raw
  // malformed request, although encodeRequest() never generates it.
  requireIntegerInRange(requestId, 0, 0xff, "request id");
  const normalizedRequestOpcode = requestOpcode & ~EXO_BLE.RESPONSE_BIT;
  const expectedResponseOpcode = normalizedRequestOpcode | EXO_BLE.RESPONSE_BIT;
  if (response.opcode !== expectedResponseOpcode ||
      response.requestOpcode !== normalizedRequestOpcode ||
      response.requestId !== requestId) {
    fail(
      "CORRELATION_MISMATCH",
      `response ${formatBytes(response.raw)} does not match opcode 0x${requestOpcode.toString(16).padStart(2, "0")} request id ${requestId}`,
    );
  }
}
