export const EXO_BLE = Object.freeze({
  DEVICE_NAME: "ESP-EXO",
  SERVICE_UUID: "f7a10001-6b5a-4d92-9e7c-3a1c8b640001",
  COMMAND_CHARACTERISTIC_UUID: "f7a10002-6b5a-4d92-9e7c-3a1c8b640001",
  RESPONSE_CHARACTERISTIC_UUID: "f7a10003-6b5a-4d92-9e7c-3a1c8b640001",
  /**
   * 阶段 10 使用独立 Notify 特征承载连续遥测。它不是命令响应通道，
   * 因而遥测拥塞不会占用 request_id，也不会和 START/STOP 的 ACK 混淆。
   */
  TELEMETRY_CHARACTERISTIC_UUID: "f7a10004-6b5a-4d92-9e7c-3a1c8b640001",
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
  CAPABILITIES_MASK: 0xffff,
  ASSIST_TORQUE_HARD_MAX_MNM: 500,
  ANGLE_ASSIST_TORQUE_HARD_MAX_MNM: 15000,
  LEGACY_ANGLE_ASSIST_TORQUE_HARD_MAX_MNM: 5000,
  PULSE_FIXED_MAX_MS: 3000,
  INVALID_U16: 0xffff,
  PHASE_X10000_MAX: 9999,
  CADENCE_X100_MAX: 30000,
  CONFIDENCE_X10000_MAX: 10000,
  ASSIST_CONFIG_SCHEMA_VERSION: 1,
  ASSIST_LEVEL_PERCENT_MAX: 100,
  TELEMETRY_SCHEMA_VERSION: 1,
  TELEMETRY_PACKET_SIZE: 20,
  TELEMETRY_FLAGS_MASK: 0xff,
  TELEMETRY_CHANNEL_MASK: 0x007f,
  TELEMETRY_DEFAULT_CHANNEL_MASK: 0x0078,
  TELEMETRY_INVALID_SIGNED: -32768,
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
  GET_TELEMETRY_STATUS: 0x08,
  GET_CALIBRATION_STATUS: 0x09,
  GET_ASSIST_TORQUE_LIMIT: 0x0a,
  GET_ANGLE_ASSIST_CONFIG: 0x0b,
  START: 0x10,
  STOP: 0x11,
  CAPTURE_ZERO: 0x12,
  CLEAR_ZERO: 0x13,
  SET_CONTROL_CONFIG: 0x14,
  SET_JOINT_COMMAND: 0x15,
  SET_ASSIST_CONFIG: 0x16,
  START_TELEMETRY: 0x17,
  STOP_TELEMETRY: 0x18,
  SET_ASSIST_CONFIG_WITH_LIMIT: 0x19,
  SET_ANGLE_ASSIST_CONFIG: 0x1d,
  TEACH_DIRECTION: 0x1a,
  GET_CAN_STATUS: 0x1b,
  GET_BOARD_SENSOR: 0x27,
  GET_DIRECTION_PREVIEW: 0x28,
  GET_AFO_STATUS: 0x29,
  RESTORE_FEEDBACK: 0x1c,
  GET_SAVED_ZERO: 0x1e,
  SAVED_ZERO_ACTION: 0x1f,
  GET_PULSE_TIMING: 0x20,
  SET_PULSE_TIMING: 0x21,
  GET_PULSE_WINDOW: 0x22,
  SET_PULSE_WINDOW: 0x23,
  GET_PULSE_STATUS: 0x24,
  PREPARE_ADAPTIVE_TRIAL: 0x25,
  CHECK_ADAPTIVE_TRIAL: 0x26,
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
  /**
   * 仅供隔离工程验收使用的左右交替正弦激励。它不是产品助力参考，网页也不会
   * 将它放入普通产品参考源下拉框；只有设备声明 ENGINEERING_TEST_STIMULUS
   * capability 时，独立的工程验证卡片才可以写入这个值。
  */
  ALTERNATING_SINE_TEST: 0x02,
  BENCH_PEAK: 0x03,
});

export const Capability = Object.freeze({
  SYSTEM_CONTROL: 1 << 0,
  JOINT_CALIBRATION: 1 << 1,
  ON_DEMAND_STATUS: 1 << 2,
  CONTROL_CONFIGURATION: 1 << 3,
  GAIT_STATUS: 1 << 4,
  ASSIST_CONFIGURATION: 1 << 5,
  TELEMETRY_STREAM: 1 << 6,
  ENGINEERING_TEST_STIMULUS: 1 << 7,
  CALIBRATION_STATUS: 1 << 8,
  ASSIST_TORQUE_LIMIT: 1 << 9,
  DIRECTION_TEACHING: 1 << 10,
  FEEDBACK_RECOVERY: 1 << 11,
  ANGLE_THRESHOLD_ASSIST: 1 << 12,
  ANGLE_ASSIST_5NM: 1 << 13,
  SAVED_WEAR_ZERO: 1 << 14,
  SWING_EVENT_PULSE: 1 << 15,
});

export const SavedZeroAction = Object.freeze({ SAVE_CURRENT: 0, RESTORE: 1, FORGET: 2, CAPTURE_PAIR: 3, CAPTURE_SAVE_PAIR: 4 });
export const savedZeroReasonDescriptions: Readonly<Record<number, string>> = Object.freeze({
  0: "上次零位可沿用；仍需确认本次穿戴中立静止",
  1: "尚未保存可沿用的零位，请先重新校准并保存",
  2: "左右映射或出厂方向不一致，请重新校准",
  3: "等待两台电机的新反馈",
  4: "两台电机必须均为未使能REST状态",
  5: "存在电机故障，不能沿用零位",
  6: "双腿仍在运动，请回中立位并保持静止",
  7: "中立位与上次相差超过5°，请重新校准",
  8: "主控保存失败，不能认定新零位已断电保留",
  9: "先完成出厂方向标定",
  10: "本次双髋零位无效，不能保存为上次零位",
  11: "请确认本次穿戴完成且中立静止",
  12: "反馈坐标已变化，请重新读取后校准",
  13: "保存记录损坏或版本不符，请重新校准",
});

export const DirectionTeachAction = Object.freeze({ BEGIN: 0, FINISH: 1, CANCEL: 2 });
export const directionTeachFailureDescriptions: Readonly<Record<number, string>> = Object.freeze({
  1: "没有有效基准，请重新记录静止基准",
  2: "基准已超过60秒，请重新记录",
  3: "双电机反馈缺失、过期或没有新样本",
  4: "电机连续坐标已重置，请重新记录基准",
  5: "所选腿相对基准的位移不足，请先记录中立基准再前抬（0.11.16起最小1°）",
  6: "目标腿位移超过30°，请取消后减小幅度重试",
  7: "另一侧也明显移动，请只前抬指定的一条腿",
  8: "姿态仍在明显变化，请保持前抬位置后重试；允许轻微抖动（需固件0.11.15及以上）",
  9: "电机未处于REST，不能进行手动方向标定",
  10: "电机报告故障，请先排除故障",
  11: "已有未完成的方向标定，请取消后重试",
  12: "基准会话不匹配，请重新记录",
  13: "出厂方向保存或读回失败，请重试；当前零位仍无效",
});

/**
 * 遥测包 byte 1 的类型。阶段 10 只有 SAMPLE；解析器会保留未知类型原始帧，
 * 这样未来固件新增包型时，旧网页仍能保持连接并把问题写入事务日志。
 */
export const TelemetryPacketType = Object.freeze({
  SAMPLE: 0x01,
});

/**
 * START_TELEMETRY 的 channel_mask。每个 Notify 只携带其中一个单比特通道，
 * 同一采样时刻的多通道包通过 streamSequence/sampleSequence 重新组合。
 */
export const TelemetryChannel = Object.freeze({
  POSITION: 0x0001,
  VELOCITY: 0x0002,
  MOTOR_TORQUE: 0x0004,
  GAIT_PHASE: 0x0008,
  ASSIST_SUGGESTION: 0x0010,
  GAIT_CONFIDENCE: 0x0020,
  APPLIED_TORQUE: 0x0040,
});

export const TelemetryFlag = Object.freeze({
  LEFT_VALID: 1 << 0,
  RIGHT_VALID: 1 << 1,
  DROPPED_SINCE_PREVIOUS: 1 << 2,
  SYSTEM_STATE_SHIFT: 3,
  SYSTEM_STATE_MASK: 0x38,
  OUTPUT_ALLOWED: 1 << 6,
  REAL_OUTPUT_ENABLED: 1 << 7,
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
  ANGLE_THRESHOLD: 0x04,
  ADAPTIVE_GAUSSIAN: 0x05,
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
  ANGLE_THRESHOLD: 0x04,
  ADAPTIVE_GAUSSIAN: 0x05,
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
  WAITING_NEW_GAIT_EVENTS: 0x08,
  AFO_ACQUIRING: 0x09,
  AFO_REARM_REQUIRED: 0x0a,
  AFO_INPUT_FAULT: 0x0b,
});

export type ResponseKind =
  | "canStatus"
  | "boardSensor"
  | "directionPreview"
  | "afoStatus"
  | "calibrationStatus"
  | "savedZeroStatus"
  | "ack"
  | "info"
  | "systemStatus"
  | "jointStatus"
  | "controlConfig"
  | "jointCommand"
  | "gaitStatus"
  | "assistConfig"
  | "assistTorqueLimit"
  | "angleAssistConfig"
  | "pulseTiming"
  | "pulseWindow"
  | "pulseStatus"
  | "telemetryStatus";

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
  0x13: "RUN_CONTRACT_INVALID",
  0xff: "UNKNOWN",
});

/**
 * 面向操作者的阻止原因说明。线码名称保持英文稳定，中文只用于页面显示，避免
 * 日志/协议字段和产品文案耦合。当前新增原因表示基础控制和助力执行态不能组成
 * 同一个安全、可执行的 RUNNING 合同。
 */
export const blockReasonDescriptions: Readonly<Record<number, string>> = Object.freeze({
  0x05: "CAN未就绪：未运行或收发错误计数已达到128",
  0x13: "控制模式/参考源与助力执行状态不兼容",
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
  [ReferenceSource.ALTERNATING_SINE_TEST]: "ALTERNATING_SINE_TEST",
  [ReferenceSource.BENCH_PEAK]: "BENCH_PEAK",
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
  [AssistMode.ANGLE_THRESHOLD]: "ANGLE_THRESHOLD",
  [AssistMode.ADAPTIVE_GAUSSIAN]: "ADAPTIVE_GAUSSIAN",
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
  [AssistStrategy.ANGLE_THRESHOLD]: "ANGLE_THRESHOLD",
  [AssistStrategy.ADAPTIVE_GAUSSIAN]: "ADAPTIVE_GAUSSIAN",
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
  [AssistUnavailableReason.WAITING_NEW_GAIT_EVENTS]: "WAITING_NEW_GAIT_EVENTS",
  [AssistUnavailableReason.AFO_ACQUIRING]: "AFO_ACQUIRING",
  [AssistUnavailableReason.AFO_REARM_REQUIRED]: "AFO_REARM_REQUIRED",
  [AssistUnavailableReason.AFO_INPUT_FAULT]: "AFO_INPUT_FAULT",
});

/** 面向操作者的助力不可用原因；英文线码名仍作为协议和日志稳定标识。 */
export const assistUnavailableReasonDescriptions: Readonly<Record<number, string>> =
  Object.freeze({
    [AssistUnavailableReason.REQUIRED_GAIT_INVALID]: "所需步态无效",
    [AssistUnavailableReason.WAITING_NEW_GAIT_EVENTS]:
      "等待本次 START 后左右髋各自出现新的屈曲峰",
    [AssistUnavailableReason.AFO_ACQUIRING]: "AFO正在捕获步态或等待回到助力工作窗口；满足条件后从零渐增",
    [AssistUnavailableReason.AFO_REARM_REQUIRED]: "AFO故障保护已锁存，该侧助力清零；排除故障后需停止并重新启动",
    [AssistUnavailableReason.AFO_INPUT_FAULT]: "AFO输入、时序或运动范围异常，受影响侧助力清零",
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

/** 零位偏置与电机连续角来自设备；缺失反馈显示null而不是0°。 */
export interface CalibrationStatusResponse extends CommonResponse {
  kind: "calibrationStatus";
  side: number;
  actuator: number;
  canId: number;
  direction: number;
  directionName: string;
  jointFlags: number;
  decodedJointFlags: DecodedJointFlags;
  rawPositionDegrees: number | null;
  softwareZeroDegrees: number | null;
  feedbackAgeMs: number;
}

export interface SavedZeroStatusResponse extends CommonResponse {
  kind: "savedZeroStatus";
  side: number;
  actuator: number;
  canId: number;
  direction: number;
  stored: boolean;
  bindingMatches: boolean;
  previewValid: boolean;
  reuseReady: boolean;
  reason: number;
  savedZeroDegrees: number | null;
  previewJointDegrees: number | null;
}

export function canReuseSavedWearZero(left: SavedZeroStatusResponse | null,
  right: SavedZeroStatusResponse | null): boolean {
  return Boolean(left && right && left.side === Side.LEFT && right.side === Side.RIGHT &&
    left.actuator !== right.actuator && [left, right].every((s) => s.stored &&
      s.bindingMatches && s.previewValid && s.reuseReady && s.reason === 0 &&
      s.previewJointDegrees !== null && Number.isFinite(s.previewJointDegrees) &&
      Math.abs(s.previewJointDegrees) <= 5));
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

/** 最大附加助力配置快照，mN·m在协议边界显式转换为N·m。 */
export interface AssistTorqueLimitResponse extends CommonResponse {
  kind: "assistTorqueLimit";
  selectedValid: boolean;
  activeValid: boolean;
  selectedMaximumNm: number | null;
  activeMaximumNm: number | null;
  hardMaximumNm: number;
}

/** GET_TELEMETRY_STATUS 的16字节快照；计数器均来自设备而不是网页推算。 */
export interface TelemetryStatusResponse extends CommonResponse {
  kind: "telemetryStatus";
  streaming: boolean;
  rateHz: number;
  channelMask: number;
  streamSequence: number;
  notifySuccessCount: number;
  droppedSampleCount: number;
  queuedSamples: number;
  queueCapacity: number;
}

export type TelemetryStartValues = {
  rateHz: 5 | 10 | 20;
  channelMask: number;
};

type TelemetryCommonPacket = {
  schemaVersion: number;
  packetType: number;
  raw: Uint8Array<ArrayBuffer>;
};

export type UnknownTelemetryPacket = TelemetryCommonPacket & {
  kind: "unknown";
};

export type TelemetrySamplePacket = TelemetryCommonPacket & {
  kind: "sample";
  channel: number;
  channelName: string;
  flags: number;
  unknownFlags: number;
  leftValid: boolean;
  rightValid: boolean;
  outputAllowed: boolean;
  realOutputEnabled: boolean;
  systemState: number;
  droppedSincePrevious: boolean;
  streamSequence: number;
  sampleSequence: number;
  timestampMs: number;
  activationEpochLow16: number;
  leftRaw: number;
  rightRaw: number;
  /** valid 标志与无效哨兵矛盾时置位，页面记录告警但仍保持连接。 */
  sentinelMismatch: boolean;
  /** 已按通道换算成人体显示单位；无效侧为 null。 */
  leftValue: number | null;
  rightValue: number | null;
  unit: "deg" | "deg/s" | "N·m" | "%" | "raw";
};

export type ExoTelemetryPacket = TelemetrySamplePacket | UnknownTelemetryPacket;

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
  adaptiveBenchIdentity?: AdaptiveBenchIdentity;
};

/** 本次连接读回的台架身份；不会编码进BLE帧，也不替代固件检查。 */
export type AdaptiveBenchIdentity = {
  firmware: readonly number[];
  capabilities: number;
  strategyId: number;
  strategyVersion: number;
  offBodyBench: boolean;
};
export function supportsAdaptiveBenchActive(identity?: AdaptiveBenchIdentity): boolean {
  // Regular firmware can select AFO directly from another saved strategy.
  if(supportsAdaptiveAssistMode(identity))return identity!.strategyId!==AssistStrategy.ADAPTIVE_GAUSSIAN || identity!.strategyVersion===(identity!.firmware[2]>=22?12:identity!.firmware[2]>=21?11:identity!.firmware[2]>=20?10:identity!.firmware[2]>=19?9:identity!.firmware[2]>=17?8:identity!.firmware[2]>=8?7:6);
  if(!identity || (identity.capabilities & Capability.ASSIST_CONFIGURATION)===0 ||
      identity.strategyId!==AssistStrategy.ADAPTIVE_GAUSSIAN)return false;
  const version=identity.firmware.join(".");
  if(version==="0.11.6")return identity.strategyVersion===5;
  if(["0.11.3","0.11.4", "0.11.5"].includes(version))return identity.strategyVersion===4 ||
    (identity.offBodyBench && [2,3].includes(identity.strategyVersion));
  return ["0.11.1","0.11.2"].includes(version) && identity.strategyVersion===2 && identity.offBodyBench;
}
export function adaptiveNominalMaximumNm(identity?: AdaptiveBenchIdentity): number {
  if(supportsAdaptiveAssistMode(identity))return identity!.firmware[2]>=17?15:5;
  if(identity?.firmware.join(".")==="0.11.6" && identity.strategyVersion===5 &&
      identity.strategyId===5 && (identity.capabilities & Capability.ASSIST_CONFIGURATION)!==0)return 5;
  return identity && ["0.11.3","0.11.4", "0.11.5"].includes(identity.firmware.join(".")) &&
    (identity.capabilities & Capability.ASSIST_CONFIGURATION)!==0 && identity.strategyId===5 &&
    [3,4].includes(identity.strategyVersion) ? 15 : .5;
}

export function supportsAdaptiveAssistMode(identity?: Pick<AdaptiveBenchIdentity, "firmware" | "capabilities">): boolean {
  return identity !== undefined && ["0.11.7", "0.11.8", "0.11.9", "0.11.10", "0.11.11", "0.11.12", "0.11.13", "0.11.14", "0.11.15", "0.11.16", "0.11.17", "0.11.18", "0.11.19", "0.11.20", "0.11.21", "0.11.22", "0.11.23"].includes(identity.firmware.join(".")) &&
    (identity.capabilities & Capability.ASSIST_CONFIGURATION)!==0;
}

export type CanStatusResponse = CommonResponse & {
  kind: "canStatus";
  state: number;
  stateName: string;
  usable: boolean;
  restoreAllowed: boolean;
  txErrorCounter: number;
  rxErrorCounter: number;
  busErrorCount: number;
  txFailedCount: number;
  rxMissedCount: number;
  motorFlags: readonly [number, number];
};

export type AfoStatusResponse = CommonResponse & {
  kind:"afoStatus"; side:number; faults:number; firstFaults:number; lockReasons:number;
  locked:boolean; rearmRequired:boolean; firstPresent:boolean; firstTimestampMs:number;
  firstPositionDeg:number|null; firstVelocityDegS:number|null; firstSourceIntervalMs:number; firstAgeMs:number;
};
export function afoFaultDescription(faults:number):string {
  return [[1,"反馈失效"],[2,"控制时序异常"],[4,"超出助力工作窗口"],[8,"等待步态锁相"],
    [16,"未授权或停止"],[32,"故障锁存"],[64,"配置异常"]]
    .filter(([bit])=>(faults&Number(bit))!==0).map(([,text])=>text).join("、")||"输出条件正常";
}
export type DirectionPreviewResponse = CommonResponse & {
  kind: "directionPreview"; side: number; token: number; result: number; actuator: number;
  baselineDegrees: number | null; rawPositionDegrees: number | null;
  remainingMs: number; feedbackAgeMs: number;
};
export function supportsDirectionPreview(identity?: {firmware: readonly number[]} | null): boolean {
  return !!identity && ["0.11.18", "0.11.19", "0.11.20", "0.11.21", "0.11.22", "0.11.23"].includes(identity.firmware.join("."));
}
export type BoardSensorResponse = CommonResponse & {
  kind: "boardSensor";
  page: number;
  valid: boolean;
  configured: boolean;
  gyroCalibrated: boolean;
  magFused: boolean;
  ageMs: number;
  values: readonly [number, number, number];
};

export function supportsBoardSensors(identity?: {firmware: readonly number[]} | null): boolean {
  return !!identity && ["0.11.13", "0.11.14", "0.11.15", "0.11.16", "0.11.17", "0.11.18", "0.11.19", "0.11.20", "0.11.21", "0.11.22", "0.11.23"].includes(identity.firmware.join("."));
}

export type ExoBleResponse =
  | DirectionPreviewResponse
  | AfoStatusResponse
  | BoardSensorResponse
  | SavedZeroStatusResponse
  | CanStatusResponse
  | AckResponse
  | InfoResponse
  | SystemStatusResponse
  | JointStatusResponse
  | CalibrationStatusResponse
  | ControlConfigResponse
  | JointCommandResponse
  | GaitStatusResponse
  | AssistConfigResponse
  | AssistTorqueLimitResponse
  | AngleAssistConfigResponse
  | PulseTimingResponse
  | PulseWindowResponse
  | PulseStatusResponse
  | TelemetryStatusResponse;

const responseOpcodes = Object.freeze({
  DIRECTION_PREVIEW: Opcode.GET_DIRECTION_PREVIEW | EXO_BLE.RESPONSE_BIT,
  AFO_STATUS: Opcode.GET_AFO_STATUS | EXO_BLE.RESPONSE_BIT,
  CAN_STATUS: Opcode.GET_CAN_STATUS | EXO_BLE.RESPONSE_BIT,
  BOARD_SENSOR: Opcode.GET_BOARD_SENSOR | EXO_BLE.RESPONSE_BIT,
  INFO: Opcode.GET_INFO | EXO_BLE.RESPONSE_BIT,
  SYSTEM_STATUS: Opcode.GET_SYSTEM_STATUS | EXO_BLE.RESPONSE_BIT,
  JOINT_STATUS: Opcode.GET_JOINT_STATUS | EXO_BLE.RESPONSE_BIT,
  CALIBRATION_STATUS: Opcode.GET_CALIBRATION_STATUS | EXO_BLE.RESPONSE_BIT,
  SAVED_ZERO: Opcode.GET_SAVED_ZERO | EXO_BLE.RESPONSE_BIT,
  CONTROL_CONFIG: Opcode.GET_CONTROL_CONFIG | EXO_BLE.RESPONSE_BIT,
  JOINT_COMMAND: Opcode.GET_JOINT_COMMAND | EXO_BLE.RESPONSE_BIT,
  GAIT_STATUS: Opcode.GET_GAIT_STATUS | EXO_BLE.RESPONSE_BIT,
  ASSIST_CONFIG: Opcode.GET_ASSIST_CONFIG | EXO_BLE.RESPONSE_BIT,
  ASSIST_TORQUE_LIMIT: Opcode.GET_ASSIST_TORQUE_LIMIT | EXO_BLE.RESPONSE_BIT,
  ANGLE_ASSIST_CONFIG: Opcode.GET_ANGLE_ASSIST_CONFIG | EXO_BLE.RESPONSE_BIT,
  PULSE_TIMING: Opcode.GET_PULSE_TIMING | EXO_BLE.RESPONSE_BIT,
  PULSE_WINDOW: Opcode.GET_PULSE_WINDOW | EXO_BLE.RESPONSE_BIT,
  PULSE_STATUS: Opcode.GET_PULSE_STATUS | EXO_BLE.RESPONSE_BIT,
  TELEMETRY_STATUS: Opcode.GET_TELEMETRY_STATUS | EXO_BLE.RESPONSE_BIT,
});

const requestPayloadLengths: Readonly<Record<number, number>> = Object.freeze({
  [Opcode.GET_DIRECTION_PREVIEW]: 1,
  [Opcode.GET_AFO_STATUS]: 1,
  [Opcode.GET_CAN_STATUS]: 0,
  [Opcode.GET_BOARD_SENSOR]: 1,
  [Opcode.RESTORE_FEEDBACK]: 0,
  [Opcode.GET_INFO]: 0,
  [Opcode.PREPARE_ADAPTIVE_TRIAL]: 0,
  [Opcode.CHECK_ADAPTIVE_TRIAL]: 0,
  [Opcode.GET_SYSTEM_STATUS]: 0,
  [Opcode.GET_JOINT_STATUS]: 1,
  [Opcode.GET_CALIBRATION_STATUS]: 1,
  [Opcode.GET_SAVED_ZERO]: 1,
  [Opcode.SAVED_ZERO_ACTION]: 2,
  [Opcode.GET_CONTROL_CONFIG]: 0,
  [Opcode.GET_JOINT_COMMAND]: 1,
  [Opcode.GET_GAIT_STATUS]: 0,
  [Opcode.GET_ASSIST_CONFIG]: 0,
  [Opcode.GET_ASSIST_TORQUE_LIMIT]: 0,
  [Opcode.GET_ANGLE_ASSIST_CONFIG]: 0,
  [Opcode.GET_PULSE_TIMING]: 0,
  [Opcode.SET_PULSE_TIMING]: 16,
  [Opcode.GET_PULSE_WINDOW]: 0,
  [Opcode.SET_PULSE_WINDOW]: 4,
  [Opcode.GET_PULSE_STATUS]: 1,
  [Opcode.GET_TELEMETRY_STATUS]: 0,
  [Opcode.START]: 0,
  [Opcode.STOP]: 0,
  [Opcode.CAPTURE_ZERO]: 1,
  [Opcode.CLEAR_ZERO]: 1,
  [Opcode.TEACH_DIRECTION]: 3,
  [Opcode.SET_CONTROL_CONFIG]: 2,
  [Opcode.SET_JOINT_COMMAND]: 15,
  [Opcode.SET_ASSIST_CONFIG]: 8,
  [Opcode.SET_ASSIST_CONFIG_WITH_LIMIT]: 10,
  [Opcode.SET_ANGLE_ASSIST_CONFIG]: 12,
  [Opcode.START_TELEMETRY]: 3,
  [Opcode.STOP_TELEMETRY]: 0,
});

const sidePayloadOpcodes = new Set<number>([
  Opcode.GET_JOINT_STATUS,
  Opcode.GET_CALIBRATION_STATUS,
  Opcode.GET_SAVED_ZERO,
  Opcode.GET_PULSE_STATUS,
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
  if(selectedReference===ReferenceSource.BENCH_PEAK&&selectedMode!==ControlMode.TORQUE)
    fail("INVALID_FIELD","bench peak reference requires TORQUE mode");
  // 动态参考只给 IMPEDANCE 定义。ALTERNATING_SINE_TEST 还需要调用方在发送前
  // capability；编码器只负责拒绝协议本身不允许的模式/参考组合。
  if ((selectedReference === ReferenceSource.SINE ||
       selectedReference === ReferenceSource.ALTERNATING_SINE_TEST) &&
      selectedMode !== ControlMode.IMPEDANCE) {
    fail("INVALID_ARGUMENT", "dynamic reference requires IMPEDANCE mode");
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
    [AssistStrategy.ANGLE_THRESHOLD]: AssistMode.ANGLE_THRESHOLD,
    [AssistStrategy.ADAPTIVE_GAUSSIAN]: AssistMode.ADAPTIVE_GAUSSIAN,
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
  // 仅限制发送；读回仍保留异常执行态，供界面显示和诊断。
  if (wire.strategyId === AssistStrategy.ADAPTIVE_GAUSSIAN &&
      ((wire.requestedExecutionState === AssistExecutionState.ACTIVE &&
        !supportsAdaptiveBenchActive(values.adaptiveBenchIdentity)) || wire.profileId !== 0)) {
    fail("INVALID_FIELD", "AFO ACTIVE需要本次设备读回支持当前试验范围；profile须为0");
  }
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

/**
 * 编码低频的“开始遥测”配置。采样率和通道掩码只在启动流时写一次；
 * 手机绝不能用这个接口代替 ESP32 本地 100 Hz 控制。
 */
export function encodeTelemetryStartPayload(
  values: TelemetryStartValues,
): Uint8Array<ArrayBuffer> {
  if (values.rateHz !== 5 && values.rateHz !== 10 && values.rateHz !== 20) {
    fail("INVALID_ARGUMENT", "telemetry rate must be 5, 10, or 20 Hz");
  }
  requireIntegerInRange(values.channelMask, 1, 0xffff, "telemetry channel mask");
  if ((values.channelMask & ~EXO_BLE.TELEMETRY_CHANNEL_MASK) !== 0) {
    fail("INVALID_ARGUMENT", "telemetry channel mask contains unsupported bits");
  }

  const payload = new Uint8Array(3);
  const view = new DataView(payload.buffer);
  view.setUint8(0, values.rateHz);
  view.setUint16(1, values.channelMask, true);
  return payload;
}

/** 新固件专用：10-byte原子写入。旧8-byte配置和16-byte读回格式仍保持不变。 */
export function encodeAssistConfigWithLimitPayload(
  values: AssistConfigValues, maximumNm: number,
): Uint8Array<ArrayBuffer> {
  const hardNm = (values.strategyId === AssistStrategy.ANGLE_THRESHOLD
    ? EXO_BLE.ANGLE_ASSIST_TORQUE_HARD_MAX_MNM
    : values.strategyId===AssistStrategy.ADAPTIVE_GAUSSIAN
      ? adaptiveNominalMaximumNm(values.adaptiveBenchIdentity)*1000 : EXO_BLE.ASSIST_TORQUE_HARD_MAX_MNM) / 1000;
  if (!Number.isFinite(maximumNm) || maximumNm < 0 || maximumNm > hardNm) {
    fail("INVALID_PAYLOAD", `最大附加助力须在 0–${hardNm.toFixed(3)} N·m 内`);
  }
  const milliNm = Math.round(maximumNm * 1000);
  if (Math.abs(maximumNm * 1000 - milliNm) > 1e-8) {
    fail("INVALID_PAYLOAD", "最大附加助力精度为 0.001 N·m");
  }
  const payload = new Uint8Array(10);
  payload.set(encodeAssistConfigPayload(values));
  new DataView(payload.buffer).setUint16(8, milliNm, true);
  return payload;
}

export type AngleAssistConfigResponse = CommonResponse & {
  kind: "angleAssistConfig";
  selectedValid: boolean;
  activeValid: boolean;
  selectedThresholdDegrees: number | null;
  activeThresholdDegrees: number | null;
  hysteresisDegrees: number;
  rampMs: number;
  pulseModel: boolean;
  configGeneration: number;
};

/** 0x1D一次保存模式、运行方式、左右等级、力矩和人体正角阈值。 */
export function encodeAngleAssistConfigPayload(values: AssistConfigValues,
  maximumNm: number, thresholdDegrees: number): Uint8Array<ArrayBuffer> {
  if (values.selectedMode !== AssistMode.ANGLE_THRESHOLD ||
      values.strategyId !== AssistStrategy.ANGLE_THRESHOLD || values.profileId !== 0)
    fail("INVALID_PAYLOAD", "角度策略的模式/策略ID/曲线ID不匹配");
  if (!Number.isFinite(thresholdDegrees) || thresholdDegrees < 1 || thresholdDegrees > 60 ||
      Math.abs(thresholdDegrees * 100 - Math.round(thresholdDegrees * 100)) > 1e-8)
    fail("INVALID_PAYLOAD", "前抬角度阈值须在1–60°内，精度0.01°");
  const payload = new Uint8Array(12);
  payload.set(encodeAssistConfigWithLimitPayload(values, maximumNm));
  new DataView(payload.buffer).setUint16(10, Math.round(thresholdDegrees * 100), true);
  return payload;
}

function parseAngleAssistConfig(header: ParsedHeader): AngleAssistConfigResponse {
  const v = header.view;
  const flags = v.getUint8(5), selected = v.getUint16(6, true), active = v.getUint16(8, true);
  const pulseModel = v.getUint8(4) === 2;
  if (![1,2].includes(v.getUint8(4)) || (flags & ~3) !== 0 || selected > 6000 || active > 6000 ||
      (!(flags & 1) && selected !== 0) || (!(flags & 2) && active !== 0) ||
      ((flags & 2) !== 0 && (flags & 1) === 0) || v.getUint16(10, true) !== (pulseModel ? 0 : 100) ||
      v.getUint16(12, true) !== (pulseModel ? 0 : 200) || v.getUint8(15) !== 0)
    fail("INVALID_FIELD", "角度助力配置读回字段无效");
  return {...commonResponse(header, "angleAssistConfig"), kind: "angleAssistConfig",
    selectedValid: (flags & 1) !== 0, activeValid: (flags & 2) !== 0,
    selectedThresholdDegrees: flags & 1 ? selected / 100 : null,
    activeThresholdDegrees: flags & 2 ? active / 100 : null,
    hysteresisDegrees: pulseModel ? 0 : 1, rampMs: pulseModel ? 0 : 200, pulseModel,
    configGeneration: v.getUint8(14)};
}

export const PulseDurationMode = Object.freeze({ FIXED: 0, PERIOD_SCALED: 1 });
export type PulseTimingValues = {
  durationMode: number; peakNm: number; fixedMs: number; ratio: number;
  minimumMs: number; maximumMs: number; periodMinimumMs: number; periodMaximumMs: number;
};
export type PulseTimingResponse = CommonResponse & PulseTimingValues & { kind: "pulseTiming" };
export type PulseWindowResponse = CommonResponse & {
  kind: "pulseWindow"; resetDegrees: number; maximumDegrees: number; eventSpeedDegreesPerSecond: number;
};
export type PulseStatusResponse = CommonResponse & {
  kind: "pulseStatus"; side: number; state: number; periodTrusted: boolean; rateLimited: boolean;
  armed: boolean; exitReason: number; eventCount: number; pulseCount: number;
  frozenDurationMs: number; estimatedPeriodMs: number; frozenPeakNm: number; progress: number;
};
export const pulseStateDescriptions: Readonly<Record<number,string>> = Object.freeze({
  0: "等待本侧复位", 1: "等待新的前抬事件", 2: "确认前抬事件", 3: "本次脉冲进行中", 4: "等待下一次复位",
});
export const pulseExitDescriptions: Readonly<Record<number,string>> = Object.freeze({
  0: "无", 1: "本次脉冲结束", 2: "运动方向反转，已清零", 3: "超出角度窗口，已清零",
  4: "事件未通过确认", 5: "本侧周期尚不可信，未产生助力", 6: "反馈失效，已清零",
  7: "已停止摆腿或低速保持，提前清零",
});
function validatePulseTiming(t: PulseTimingValues): void {
  const integer = (n: number, min: number, max: number) => Number.isInteger(n) && n >= min && n <= max;
  if (!integer(t.durationMode,0,1) || !Number.isFinite(t.peakNm) || t.peakNm<0 || t.peakNm>15 ||
      Math.abs(t.peakNm*1000-Math.round(t.peakNm*1000))>1e-7 ||
      !integer(t.fixedMs,100,EXO_BLE.PULSE_FIXED_MAX_MS) || !Number.isFinite(t.ratio) || t.ratio<0.1 || t.ratio>0.6 ||
      Math.abs(t.ratio*1000-Math.round(t.ratio*1000))>1e-7 ||
      !integer(t.minimumMs,100,2000) || !integer(t.maximumMs,100,2000) || t.minimumMs>t.maximumMs ||
      !integer(t.periodMinimumMs,300,5000) || !integer(t.periodMaximumMs,300,5000) ||
      t.periodMinimumMs>=t.periodMaximumMs) fail("INVALID_PAYLOAD","脉冲参数范围或时长关系无效");
}
/** bit13原义是至少支持5 N·m，新范围必须核对已知固件版本。 */
export function hasExtendedPulseLimits(capabilities: number, firmware: readonly number[] = []): boolean {
  return ["0.10.12", "0.10.13", "0.11.0", "0.11.1", "0.11.2", "0.11.3", "0.11.4", "0.11.5", "0.11.6", "0.11.7", "0.11.8", "0.11.9", "0.11.10", "0.11.11", "0.11.12", "0.11.13", "0.11.14", "0.11.15", "0.11.16", "0.11.17", "0.11.18", "0.11.19", "0.11.20", "0.11.21", "0.11.22", "0.11.23"].includes(firmware.join(".")) &&
    (capabilities & (Capability.ANGLE_ASSIST_5NM | Capability.SWING_EVENT_PULSE)) ===
      (Capability.ANGLE_ASSIST_5NM | Capability.SWING_EVENT_PULSE);
}
export function assertPulseTimingCapability(t: PulseTimingValues, capabilities: number,
  firmware: readonly number[] = []): void {
  validatePulseTiming(t);
  if ((capabilities & Capability.SWING_EVENT_PULSE) === 0 ||
      (!hasExtendedPulseLimits(capabilities, firmware) && (t.peakNm > 5 || t.fixedMs > 2000)))
    fail("INVALID_PAYLOAD", "15 N·m申请峰值与3000 ms固定脉冲需要0.10.12固件，草稿未写入");
}
/** 同一v1帧格式的历史固件保留其真实速率，不能给0.10.12显示20。 */
export function pulseRiseLimitNmPerSecond(capabilities: number, firmware: readonly number[] = []): number | null {
  if ((capabilities & Capability.SWING_EVENT_PULSE) === 0) return null;
  const version=firmware.join(".");
  if (["0.10.13", "0.11.0", "0.11.1", "0.11.2", "0.11.3", "0.11.4", "0.11.5", "0.11.6", "0.11.7", "0.11.8", "0.11.9", "0.11.10", "0.11.11", "0.11.12", "0.11.13", "0.11.14", "0.11.15", "0.11.16", "0.11.17", "0.11.18", "0.11.19", "0.11.20", "0.11.21", "0.11.22", "0.11.23"].includes(version)) return 20;
  return ["0.10.10", "0.10.11", "0.10.12"].includes(version) ? 10 : null;
}
export function encodePulseTimingPayload(t: PulseTimingValues): Uint8Array<ArrayBuffer> {
  validatePulseTiming(t);
  const b = new Uint8Array(16), v = new DataView(b.buffer);
  b[0]=1; b[1]=t.durationMode;
  [Math.round(t.peakNm*1000),t.fixedMs,Math.round(t.ratio*1000),t.minimumMs,t.maximumMs,
    t.periodMinimumMs,t.periodMaximumMs].forEach((n,i)=>v.setUint16(2+i*2,n,true));
  return b;
}
export function encodePulseWindowPayload(resetDegrees: number, maximumDegrees: number): Uint8Array<ArrayBuffer> {
  if (![resetDegrees,maximumDegrees].every(Number.isFinite) || resetDegrees< -30 || resetDegrees>59 ||
      maximumDegrees<3 || maximumDegrees>90 || maximumDegrees-resetDegrees<3 ||
      [resetDegrees,maximumDegrees].some(n=>Math.abs(n*100-Math.round(n*100))>1e-7))
    fail("INVALID_PAYLOAD","复位角/允许角度上限无效，精度0.01°");
  const b=new Uint8Array(4),v=new DataView(b.buffer);
  v.setInt16(0,Math.round(resetDegrees*100),true);v.setUint16(2,Math.round(maximumDegrees*100),true);
  return b;
}
function readPulseTiming(v: DataView, offset: number): PulseTimingValues {
  if(v.getUint8(offset)!==1) fail("INVALID_FIELD","不支持的脉冲参数格式");
  const t={durationMode:v.getUint8(offset+1),peakNm:v.getUint16(offset+2,true)/1000,
    fixedMs:v.getUint16(offset+4,true),ratio:v.getUint16(offset+6,true)/1000,
    minimumMs:v.getUint16(offset+8,true),maximumMs:v.getUint16(offset+10,true),
    periodMinimumMs:v.getUint16(offset+12,true),periodMaximumMs:v.getUint16(offset+14,true)};
  validatePulseTiming(t);return t;
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
  if (opcode === Opcode.GET_BOARD_SENSOR && payloadBytes[0] > 3)
    fail("INVALID_PAYLOAD", "board sensor page must be 0..3");
  if ((opcode === Opcode.GET_DIRECTION_PREVIEW || opcode === Opcode.GET_AFO_STATUS) && payloadBytes[0] > 1)
    fail("INVALID_PAYLOAD", "direction preview side must be 0..1");
  if (opcode===Opcode.SET_PULSE_TIMING) readPulseTiming(new DataView(payloadBytes.buffer,payloadBytes.byteOffset,payloadBytes.byteLength),0);
  if (opcode===Opcode.SET_PULSE_WINDOW) {
    const v=new DataView(payloadBytes.buffer,payloadBytes.byteOffset,payloadBytes.byteLength);
    encodePulseWindowPayload(v.getInt16(0,true)/100,v.getUint16(2,true)/100);
  }
  if ([Opcode.SET_ASSIST_CONFIG, Opcode.SET_ASSIST_CONFIG_WITH_LIMIT, Opcode.SET_ANGLE_ASSIST_CONFIG].some((candidate) => candidate === opcode)) {
    const threshold = payloadBytes[3] === AssistStrategy.ANGLE_THRESHOLD;
    if (threshold !== (opcode === Opcode.SET_ANGLE_ASSIST_CONFIG) || (threshold && payloadBytes[4] !== 0))
      fail("INVALID_PAYLOAD", "角度策略必须通过专用原子配置命令写入");
    if (threshold) {
      const angle = new DataView(payloadBytes.buffer, payloadBytes.byteOffset, payloadBytes.byteLength).getUint16(10, true);
      if (angle < 100 || angle > 6000) fail("INVALID_PAYLOAD", "前抬角度阈值须在1–60°内");
    }
  }
  if (opcode === Opcode.SET_CONTROL_CONFIG) {
    selectedControlModeName(payloadBytes[0], "selected mode");
    requireNamedCode(payloadBytes[1], referenceSourceNames, "selected reference");
  }
  if (opcode === Opcode.TEACH_DIRECTION &&
      ((payloadBytes[0] !== Side.LEFT && payloadBytes[0] !== Side.RIGHT) ||
       payloadBytes[1] > DirectionTeachAction.CANCEL || payloadBytes[2] === 0)) {
    fail("INVALID_PAYLOAD", "方向示教需要有效side、action和非零会话token");
  }
  if (opcode === Opcode.SAVED_ZERO_ACTION && (payloadBytes[0] > SavedZeroAction.CAPTURE_SAVE_PAIR || payloadBytes[1] !== 1))
    fail("INVALID_PAYLOAD", "零位候选操作需要有效action和明确确认标志");
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
  if (opcode === Opcode.SET_ASSIST_CONFIG || opcode === Opcode.SET_ASSIST_CONFIG_WITH_LIMIT || opcode === Opcode.SET_ANGLE_ASSIST_CONFIG) {
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
  if ((opcode === Opcode.SET_ASSIST_CONFIG_WITH_LIMIT || opcode === Opcode.SET_ANGLE_ASSIST_CONFIG) &&
      new DataView(payloadBytes.buffer, payloadBytes.byteOffset, payloadBytes.byteLength)
        .getUint16(8, true) > (opcode === Opcode.SET_ANGLE_ASSIST_CONFIG || payloadBytes[3]===AssistStrategy.ADAPTIVE_GAUSSIAN
          ? EXO_BLE.ANGLE_ASSIST_TORQUE_HARD_MAX_MNM : EXO_BLE.ASSIST_TORQUE_HARD_MAX_MNM)) {
    fail("INVALID_PAYLOAD", "maximum assistance exceeds firmware engineering limit");
  }
  if (opcode === Opcode.START_TELEMETRY) {
    const rateHz = payloadBytes[0];
    const channelMask = new DataView(
      payloadBytes.buffer,
      payloadBytes.byteOffset,
      payloadBytes.byteLength,
    ).getUint16(1, true);
    if ((rateHz !== 5 && rateHz !== 10 && rateHz !== 20) ||
        channelMask === 0 ||
        (channelMask & ~EXO_BLE.TELEMETRY_CHANNEL_MASK) !== 0) {
      fail("INVALID_PAYLOAD", "invalid telemetry rate or channel mask");
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

function parseCalibrationStatus(header: ParsedHeader): CalibrationStatusResponse {
  const side = header.raw[4];
  const actuator = header.raw[5];
  const canId = header.raw[6];
  const direction = header.raw[7];
  const jointFlags = header.raw[8];
  if ((side !== Side.LEFT && side !== Side.RIGHT) ||
      (actuator !== 0 && actuator !== 1 && actuator !== 255) ||
      (canId > 127 && canId !== 255) || header.raw[9] !== 0 ||
      (jointFlags & ~EXO_BLE.JOINT_FLAGS_MASK) !== 0) {
    fail("INVALID_FIELD", "invalid calibration side/mapping/flags/reserved field");
  }
  const flags = decodeJointFlags(jointFlags);
  if (flags.mappingValid && (actuator === 255 || canId === 255)) {
    fail("INVALID_FIELD", "valid calibration mapping cannot be unknown");
  }
  const raw = header.view.getInt32(10, true);
  const zero = header.view.getInt32(14, true);
  // INT32_MIN是缺失标志。离线/无零位时不能把传输填充值显示为真实人体角度。
  return {
    ...commonResponse(header, "calibrationStatus"),
    kind: "calibrationStatus", side, actuator, canId, direction, jointFlags,
    directionName: requireNamedCode(direction, directionNames, "calibration direction"),
    decodedJointFlags: flags,
    rawPositionDegrees: flags.feedbackValid && flags.online && raw !== -2147483648
      ? mradToDegrees(raw) : null,
    softwareZeroDegrees: flags.zeroValid && zero !== -2147483648
      ? mradToDegrees(zero) : null,
    feedbackAgeMs: header.view.getUint16(18, true),
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

function parseAssistTorqueLimit(header: ParsedHeader): AssistTorqueLimitResponse {
  const flags = header.raw[5];
  const selected = header.view.getUint16(6, true);
  const active = header.view.getUint16(8, true);
  const hard = header.view.getUint16(10, true);
  const selectedValid = Boolean(flags & 1);
  const activeValid = Boolean(flags & 2);
  if (header.raw[4] !== 1 || (flags & ~3) !== 0 ||
      (hard !== EXO_BLE.ASSIST_TORQUE_HARD_MAX_MNM && hard !== EXO_BLE.LEGACY_ANGLE_ASSIST_TORQUE_HARD_MAX_MNM &&
        hard !== EXO_BLE.ANGLE_ASSIST_TORQUE_HARD_MAX_MNM) ||
      selected > hard || active > hard || (!selectedValid && selected !== 0) ||
      (!activeValid && active !== 0) || (activeValid && !selectedValid)) {
    fail("INVALID_FIELD", "invalid assistance torque limit snapshot");
  }
  return {...commonResponse(header, "assistTorqueLimit"), kind: "assistTorqueLimit",
    selectedValid, activeValid, selectedMaximumNm: selectedValid ? selected / 1000 : null,
    activeMaximumNm: activeValid ? active / 1000 : null, hardMaximumNm: hard / 1000};
}

/** INFO版本/能力和GET上限必须一致；网页新范围不放宽旧设备。 */
export function assertAssistTorqueLimitCapability(response: AssistTorqueLimitResponse,
  capabilities: number, firmware: readonly number[] = []): void {
  const expected = (hasExtendedPulseLimits(capabilities, firmware) ? EXO_BLE.ANGLE_ASSIST_TORQUE_HARD_MAX_MNM
    : (capabilities & Capability.ANGLE_ASSIST_5NM) !== 0
      ? EXO_BLE.LEGACY_ANGLE_ASSIST_TORQUE_HARD_MAX_MNM : EXO_BLE.ASSIST_TORQUE_HARD_MAX_MNM) / 1000;
  if (response.hardMaximumNm !== expected)
    fail("INVALID_FIELD", "设备助力上限与INFO能力声明不一致，请重新连接核对固件");
}

/** 返回当前设备与策略共同允许的峰值。相位工程曲线始终保持0.5 N·m。 */
export function assistTorqueMaximumNm(strategyId: number, capabilities: number,
  reportedMaximumNm: number | null, firmware: readonly number[] = []): number {
  if(strategyId===AssistStrategy.ADAPTIVE_GAUSSIAN && supportsAdaptiveAssistMode({firmware,capabilities}))
    return Math.min(["0.11.17","0.11.18", "0.11.19", "0.11.20", "0.11.21", "0.11.22", "0.11.23"].includes(firmware.join("."))?15:5,reportedMaximumNm ?? .5);
  const declared = strategyId === AssistStrategy.ANGLE_THRESHOLD &&
    (capabilities & Capability.ANGLE_ASSIST_5NM) !== 0
    ? (hasExtendedPulseLimits(capabilities, firmware) ? EXO_BLE.ANGLE_ASSIST_TORQUE_HARD_MAX_MNM
        : EXO_BLE.LEGACY_ANGLE_ASSIST_TORQUE_HARD_MAX_MNM) / 1000
    : EXO_BLE.ASSIST_TORQUE_HARD_MAX_MNM / 1000;
  return Math.min(declared, reportedMaximumNm ?? EXO_BLE.ASSIST_TORQUE_HARD_MAX_MNM / 1000);
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

function parseTelemetryStatus(header: ParsedHeader): TelemetryStatusResponse {
  const streamingRaw = header.raw[4];
  const rateHz = header.raw[5];
  const channelMask = header.view.getUint16(6, true);
  const queuedSamples = header.raw[18];
  const queueCapacity = header.raw[19];

  if (streamingRaw > 1) {
    fail("INVALID_FIELD", "telemetry streaming flag must be 0 or 1");
  }
  if (rateHz !== 0 && rateHz !== 5 && rateHz !== 10 && rateHz !== 20) {
    fail("INVALID_FIELD", "telemetry status rate must be 0, 5, 10, or 20 Hz");
  }
  if ((channelMask & ~EXO_BLE.TELEMETRY_CHANNEL_MASK) !== 0) {
    fail("INVALID_FIELD", "telemetry status contains unknown channel bits");
  }
  if (streamingRaw === 1 && (rateHz === 0 || channelMask === 0)) {
    fail("INVALID_FIELD", "active telemetry stream needs a rate and at least one channel");
  }
  if (queuedSamples > queueCapacity) {
    fail("INVALID_FIELD", "telemetry queued sample count exceeds capacity");
  }

  return {
    ...commonResponse(header, "telemetryStatus"),
    kind: "telemetryStatus",
    streaming: streamingRaw === 1,
    rateHz,
    channelMask,
    streamSequence: header.view.getUint16(8, true),
    notifySuccessCount: header.view.getUint32(10, true),
    droppedSampleCount: header.view.getUint32(14, true),
    queuedSamples,
    queueCapacity,
  };
}

export const telemetryChannelNames: Readonly<Record<number, string>> = Object.freeze({
  [TelemetryChannel.POSITION]: "POSITION",
  [TelemetryChannel.VELOCITY]: "VELOCITY",
  [TelemetryChannel.MOTOR_TORQUE]: "MOTOR_TORQUE",
  [TelemetryChannel.GAIT_PHASE]: "GAIT_PHASE",
  [TelemetryChannel.ASSIST_SUGGESTION]: "ASSIST_SUGGESTION",
  [TelemetryChannel.GAIT_CONFIDENCE]: "GAIT_CONFIDENCE",
  [TelemetryChannel.APPLIED_TORQUE]: "APPLIED_TORQUE",
});

/**
 * 解析独立 Telemetry characteristic 的固定20字节 Notify。
 * 注意它没有命令协议的4字节帧头，也不参与 request_id 事务关联。
 */
export function parseTelemetryPacket(source: BufferSource): ExoTelemetryPacket {
  const raw = copyBufferSource(source, "telemetry packet");
  if (raw.length !== EXO_BLE.TELEMETRY_PACKET_SIZE) {
    fail(
      "INVALID_FRAME_LENGTH",
      `telemetry packet must be exactly ${EXO_BLE.TELEMETRY_PACKET_SIZE} bytes`,
    );
  }

  const schemaVersion = raw[0];
  const packetType = raw[1];
  if (schemaVersion !== EXO_BLE.TELEMETRY_SCHEMA_VERSION ||
      packetType !== TelemetryPacketType.SAMPLE) {
    return { kind: "unknown", schemaVersion, packetType, raw };
  }

  const channel = raw[2];
  const flags = raw[3];
  const view = new DataView(raw.buffer, raw.byteOffset, raw.byteLength);
  const unsignedChannel = channel === TelemetryChannel.GAIT_PHASE ||
    channel === TelemetryChannel.GAIT_CONFIDENCE;
  const leftRaw = unsignedChannel ? view.getUint16(16, true) : view.getInt16(16, true);
  const rightRaw = unsignedChannel ? view.getUint16(18, true) : view.getInt16(18, true);
  const wireInvalid = unsignedChannel ? EXO_BLE.INVALID_U16 : EXO_BLE.TELEMETRY_INVALID_SIGNED;
  const leftFlagValid = (flags & TelemetryFlag.LEFT_VALID) !== 0;
  const rightFlagValid = (flags & TelemetryFlag.RIGHT_VALID) !== 0;
  const leftValid = leftFlagValid && leftRaw !== wireInvalid;
  const rightValid = rightFlagValid && rightRaw !== wireInvalid;

  let unit: TelemetrySamplePacket["unit"] = "raw";
  let convert = (value: number): number => value;
  switch (channel) {
    case TelemetryChannel.POSITION:
      unit = "deg";
      convert = mradToDegrees;
      break;
    case TelemetryChannel.VELOCITY:
      unit = "deg/s";
      convert = (value) => value * 0.01 * 180 / Math.PI;
      break;
    case TelemetryChannel.MOTOR_TORQUE:
    case TelemetryChannel.ASSIST_SUGGESTION:
    case TelemetryChannel.APPLIED_TORQUE:
      unit = "N·m";
      convert = (value) => value / 1000;
      break;
    case TelemetryChannel.GAIT_PHASE:
    case TelemetryChannel.GAIT_CONFIDENCE:
      unit = "%";
      convert = (value) => value / 100;
      break;
  }

  return {
    kind: "sample",
    schemaVersion,
    packetType,
    raw,
    channel,
    channelName: telemetryChannelNames[channel] ??
      `UNKNOWN(0x${channel.toString(16).padStart(2, "0")})`,
    flags,
    unknownFlags: flags & ~EXO_BLE.TELEMETRY_FLAGS_MASK,
    leftValid,
    rightValid,
    outputAllowed: (flags & TelemetryFlag.OUTPUT_ALLOWED) !== 0,
    realOutputEnabled: (flags & TelemetryFlag.REAL_OUTPUT_ENABLED) !== 0,
    systemState: (flags & TelemetryFlag.SYSTEM_STATE_MASK) >>
      TelemetryFlag.SYSTEM_STATE_SHIFT,
    droppedSincePrevious: (flags & TelemetryFlag.DROPPED_SINCE_PREVIOUS) !== 0,
    streamSequence: view.getUint16(4, true),
    sampleSequence: view.getUint32(6, true),
    timestampMs: view.getUint32(10, true),
    activationEpochLow16: view.getUint16(14, true),
    leftRaw,
    rightRaw,
    sentinelMismatch: leftFlagValid !== (leftRaw !== wireInvalid) ||
      rightFlagValid !== (rightRaw !== wireInvalid),
    leftValue: leftValid ? convert(leftRaw) : null,
    rightValue: rightValid ? convert(rightRaw) : null,
    unit,
  };
}

function parseCanStatus(header: ParsedHeader): CanStatusResponse {
  const state = header.view.getUint8(4);
  const flags = header.view.getUint8(5);
  const a = header.view.getUint8(18), b = header.view.getUint8(19);
  if (state > 4 || (flags & ~3) !== 0 || (a & ~31) !== 0 || (b & ~31) !== 0)
    fail("INVALID_FIELD", "invalid CAN status flags/state");
  return {
    ...commonResponse(header, "canStatus"), kind: "canStatus", state,
    stateName: ["STOPPED", "RUNNING", "BUS_OFF", "RECOVERING", "UNAVAILABLE"][state],
    usable: (flags & 1) !== 0, restoreAllowed: (flags & 2) !== 0,
    txErrorCounter: header.view.getUint16(6, true), rxErrorCounter: header.view.getUint16(8, true),
    busErrorCount: header.view.getUint32(10, true), txFailedCount: header.view.getUint16(14, true),
    rxMissedCount: header.view.getUint16(16, true), motorFlags: [a, b],
  };
}

function parseSavedZeroStatus(header: ParsedHeader): SavedZeroStatusResponse {
  const v = header.view;
  const side = v.getUint8(4), actuator = v.getUint8(5), canId = v.getUint8(6), direction = v.getUint8(7);
  const flags = v.getUint8(8), reason = v.getUint8(9);
  const saved = v.getInt32(12, true), preview = v.getInt32(16, true), invalid = -2147483648;
  const stored = (flags & 1) !== 0, bindingMatches = (flags & 2) !== 0;
  const previewValid = (flags & 4) !== 0, reuseReady = (flags & 8) !== 0;
  if ((side !== Side.LEFT && side !== Side.RIGHT) || (actuator > 1 && actuator !== 255) ||
      (canId > 127 && canId !== 255) || direction > 2 || (flags & ~15) !== 0 || reason > 13 ||
      v.getUint16(10, true) !== 0 || stored !== (saved !== invalid) || previewValid !== (preview !== invalid) ||
      (bindingMatches && (!stored || actuator > 1 || canId > 127 || direction === 0)) ||
      (stored && Math.abs(saved) > 720000) || (previewValid && (!stored || !bindingMatches || Math.abs(preview) > 720000)) ||
      ((reason === 0) !== reuseReady) || (reuseReady && (flags !== 15 || Math.abs(preview) > 5000)))
    fail("INVALID_FIELD", "invalid saved zero status or preview");
  return {...commonResponse(header, "savedZeroStatus"), kind:"savedZeroStatus", side, actuator, canId, direction,
    stored, bindingMatches, previewValid, reuseReady, reason,
    savedZeroDegrees: stored ? saved / 1000 : null,
    previewJointDegrees: previewValid ? preview / 1000 : null};
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
    case responseOpcodes.AFO_STATUS: {
      if(header.payloadLength!==16)fail("INVALID_FRAME_LENGTH","AFO_STATUS payload must be16 bytes");
      const v=header.view,side=v.getUint8(4),faults=v.getUint8(5),firstFaults=v.getUint8(6),flags=v.getUint8(7);
      const lockReasons=v.getUint16(8,true),q=v.getInt16(14,true),speed=v.getInt16(16,true);
      if(side>1||faults>127||firstFaults>127||flags>7||lockReasons>511)fail("INVALID_FIELD","invalid AFO diagnostic flags");
      return {...commonResponse(header,"afoStatus"),kind:"afoStatus",side,faults,firstFaults,lockReasons,
        locked:!!(flags&1),rearmRequired:!!(flags&2),firstPresent:!!(flags&4),firstTimestampMs:v.getUint32(10,true),
        firstPositionDeg:q===-32768?null:q/100,firstVelocityDegS:speed===-32768?null:speed/10,
        firstSourceIntervalMs:v.getUint8(18),firstAgeMs:v.getUint8(19)};
    }
    case responseOpcodes.DIRECTION_PREVIEW: {
      if (header.payloadLength !== 16) fail("INVALID_FRAME_LENGTH", "DIRECTION_PREVIEW payload must be16 bytes");
      const v=header.view,side=v.getUint8(4),token=v.getUint8(5),result=v.getUint8(6),actuator=v.getUint8(7);
      const baseline=v.getInt32(8,true),raw=v.getInt32(12,true),remainingMs=v.getUint16(16,true),feedbackAgeMs=v.getUint16(18,true);
      if(side>1 || result>13 || ![0,1,255].includes(actuator) || remainingMs>60000 ||
        (result===0 && (!token || actuator>1 || !remainingMs || feedbackAgeMs>50 || baseline===-2147483648 || raw===-2147483648)))
        fail("INVALID_FIELD","invalid direction preview session");
      return {...commonResponse(header,"directionPreview"),kind:"directionPreview",side,token,result,actuator,
        baselineDegrees:baseline===-2147483648?null:mradToDegrees(baseline),
        rawPositionDegrees:raw===-2147483648?null:mradToDegrees(raw),remainingMs,feedbackAgeMs};
    }
    case responseOpcodes.BOARD_SENSOR: {
      if (header.payloadLength !== 16) fail("INVALID_FRAME_LENGTH", "BOARD_SENSOR payload must be 16 bytes");
      const v = header.view, page = v.getUint8(4), flags = v.getUint8(5);
      const ageMs = v.getUint16(6, true);
      const values = [v.getFloat32(8, true), v.getFloat32(12, true), v.getFloat32(16, true)] as const;
      if (page > 3 || (flags & ~15) !== 0 || values.some(n => !Number.isFinite(n)) ||
          ((flags & 1) !== 0 && (ageMs >= 250 || (flags & 8) === 0)))
        fail("INVALID_FIELD", "invalid board sensor snapshot");
      return {...commonResponse(header, "boardSensor"), kind: "boardSensor", page,
        valid: (flags & 1) !== 0, configured: (flags & 8) !== 0,
        gyroCalibrated: (flags & 2) !== 0, magFused: (flags & 4) !== 0, ageMs, values};
    }
    case responseOpcodes.SAVED_ZERO:
      if (header.payloadLength !== 16) fail("INVALID_FRAME_LENGTH", "SAVED_ZERO payload must be 16 bytes");
      return parseSavedZeroStatus(header);
    case responseOpcodes.CAN_STATUS:
      if (header.payloadLength !== 16) fail("INVALID_FRAME_LENGTH", "CAN_STATUS payload must be 16 bytes");
      return parseCanStatus(header);
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
    case responseOpcodes.CALIBRATION_STATUS:
      if (header.payloadLength !== 16) {
        fail("INVALID_FRAME_LENGTH", "CALIBRATION_STATUS payload must be 16 bytes or a 4-byte ACK");
      }
      return parseCalibrationStatus(header);
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
    case responseOpcodes.ANGLE_ASSIST_CONFIG:
      if (header.payloadLength !== 12) fail("INVALID_FRAME_LENGTH", "ANGLE_ASSIST_CONFIG payload must be 12 bytes");
      return parseAngleAssistConfig(header);
    case responseOpcodes.PULSE_TIMING:
      if(header.payloadLength!==16) fail("INVALID_FRAME_LENGTH","PULSE_TIMING长度须为16");
      return {...commonResponse(header,"pulseTiming"),kind:"pulseTiming",...readPulseTiming(header.view,4)};
    case responseOpcodes.PULSE_WINDOW: {
      if(header.payloadLength!==8) fail("INVALID_FRAME_LENGTH","PULSE_WINDOW长度须为8");
      const v=header.view,resetDegrees=v.getInt16(6,true)/100,maximumDegrees=v.getUint16(8,true)/100;
      if(v.getUint8(4)!==1 || v.getUint8(5)!==0 || v.getUint16(10,true)!==500)
        fail("INVALID_FIELD","脉冲角度窗口格式无效");
      encodePulseWindowPayload(resetDegrees,maximumDegrees);
      return {...commonResponse(header,"pulseWindow"),kind:"pulseWindow",resetDegrees,maximumDegrees,eventSpeedDegreesPerSecond:5};
    }
    case responseOpcodes.PULSE_STATUS: {
      if(header.payloadLength!==16) fail("INVALID_FRAME_LENGTH","PULSE_STATUS长度须为16");
      const v=header.view,side=v.getUint8(4),state=v.getUint8(5),flags=v.getUint8(6),exitReason=v.getUint8(7);
      const frozenDurationMs=v.getUint16(12,true),estimatedPeriodMs=v.getUint16(14,true),
        frozenPeakNm=v.getUint16(16,true)/1000,progress=v.getUint16(18,true)/10000;
      if(side>1 || state>4 || (flags&~7)!==0 || exitReason>7 || frozenDurationMs>EXO_BLE.PULSE_FIXED_MAX_MS || estimatedPeriodMs>5000 ||
        frozenPeakNm>15 || progress>1) fail("INVALID_FIELD","脉冲状态字段无效");
      return {...commonResponse(header,"pulseStatus"),kind:"pulseStatus",side,state,periodTrusted:(flags&1)!==0,
        rateLimited:(flags&2)!==0,armed:(flags&4)!==0,exitReason,eventCount:v.getUint16(8,true),
        pulseCount:v.getUint16(10,true),frozenDurationMs,estimatedPeriodMs,frozenPeakNm,progress};
    }
    case responseOpcodes.ASSIST_TORQUE_LIMIT:
      if (header.payloadLength !== 8) {
        fail("INVALID_FRAME_LENGTH", "ASSIST_TORQUE_LIMIT payload must be 8 bytes or a 4-byte ACK");
      }
      return parseAssistTorqueLimit(header);
    case responseOpcodes.TELEMETRY_STATUS:
      if (header.payloadLength !== 16) {
        fail("INVALID_FRAME_LENGTH", "TELEMETRY_STATUS payload must be 16 bytes or a 4-byte ACK");
      }
      return parseTelemetryStatus(header);
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
