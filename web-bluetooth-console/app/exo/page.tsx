"use client";

import {
  type AckResponse,
  AssistExecutionState,
  AssistMode,
  AssistStrategy,
  type AssistConfigResponse,
  type AssistTorqueLimitResponse,
  type AngleAssistConfigResponse,
  type PulseTimingResponse,
  type PulseWindowResponse,
  type PulseStatusResponse,
  type PulseTimingValues,
  encodePulseTimingPayload,
  encodePulseWindowPayload,
  pulseStateDescriptions,
  pulseExitDescriptions,
  type CalibrationStatusResponse,
  type SavedZeroStatusResponse,
  SavedZeroAction,
  savedZeroReasonDescriptions,
  canReuseSavedWearZero,
  type CanStatusResponse,
  Capability,
  ControlMode,
  DirectionTeachAction,
  type ControlConfigResponse,
  EXO_BLE,
  type ExoBleResponse,
  type GaitStatusResponse,
  type InfoResponse,
  type JointCommandResponse,
  type JointCommandValues,
  type JointStatusResponse,
  Opcode,
  ReferenceSource,
  Side,
  type SystemStatusResponse,
  type TelemetryStatusResponse,
  TelemetryChannel,
  assertCorrelated,
  assertAssistTorqueLimitCapability,
  assertPulseTimingCapability,
  hasExtendedPulseLimits,
  pulseRiseLimitNmPerSecond,
  assistTorqueMaximumNm,
  assistExecutionStateNames,
  assistModeNames,
  assistUnavailableReasonDescriptions,
  blockReasonDescriptions,
  blockReasonNames,
  controlModeNames,
  directionTeachFailureDescriptions,
  encodeControlConfigPayload,
  encodeAssistConfigPayload,
  encodeAssistConfigWithLimitPayload,
  encodeAngleAssistConfigPayload,
  encodeJointCommandPayload,
  encodeTelemetryStartPayload,
  encodeRequest,
  parseTelemetryPacket,
  parseResponse,
  referenceSourceNames,
  stateNames,
  toHex,
} from "@/lib/exo-ble-protocol";
import { connectGattWithOneRetry, withGattStageTimeout } from "@/lib/exo-ble-connect";
import {
  COMPATIBLE_FIRMWARE_VERSIONS,
  PREFERRED_FIRMWARE_VERSION,
  assessDeviceCompatibility,
} from "@/lib/exo-device-compatibility";
import { isLogNearBottom } from "@/lib/exo-log-scroll";
import { captureAndVerifyBothZeros, supportsAtomicZeroCapture } from "@/lib/exo-zero-calibration";
import {
  TelemetryStreamAccumulator,
  type TelemetryAccumulatorStats,
  type TelemetrySamplePoint,
  telemetrySamplesToCsv,
  telemetrySamplesToJsonl,
} from "@/lib/exo-telemetry-stream";
import Link from "next/link";
import GaitHistory from "./gait-history";
import TelemetryScope from "./telemetry-scope";
import { SCOPE_BUFFER_SAMPLES, WALKING_SCOPE_MASK, scopeReceptionState, shouldStartScope } from "@/lib/exo-scope";
import {
  type ReactNode,
  useCallback,
  useEffect,
  useMemo,
  useRef,
  useState,
} from "react";

type ConnectionState = "idle" | "connecting" | "connected";
type ConnectStage =
  | "idle"
  | "chooser"
  | "gatt"
  | "service"
  | "characteristics"
  | "notify"
  | "handshake"
  | "ready";
type LogDirection = "tx" | "rx" | "system" | "error";

type BluetoothRemoteGATTCharacteristicLike = {
  value?: DataView;
  startNotifications: () => Promise<BluetoothRemoteGATTCharacteristicLike>;
  stopNotifications?: () => Promise<BluetoothRemoteGATTCharacteristicLike>;
  writeValueWithResponse?: (value: BufferSource) => Promise<void>;
  writeValue?: (value: BufferSource) => Promise<void>;
  addEventListener: (
    type: "characteristicvaluechanged",
    listener: (event: Event) => void,
  ) => void;
  removeEventListener: (
    type: "characteristicvaluechanged",
    listener: (event: Event) => void,
  ) => void;
};

type BluetoothRemoteGATTServiceLike = {
  getCharacteristic: (
    uuid: string,
  ) => Promise<BluetoothRemoteGATTCharacteristicLike>;
};

type BluetoothRemoteGATTServerLike = {
  connected: boolean;
  getPrimaryService: (uuid: string) => Promise<BluetoothRemoteGATTServiceLike>;
  disconnect: () => void;
};

type BluetoothDeviceLike = {
  id?: string;
  name?: string;
  gatt?: {
    connected: boolean;
    connect: () => Promise<BluetoothRemoteGATTServerLike>;
    disconnect: () => void;
  };
  addEventListener: (
    type: "gattserverdisconnected",
    listener: (event: Event) => void,
  ) => void;
  removeEventListener: (
    type: "gattserverdisconnected",
    listener: (event: Event) => void,
  ) => void;
};

type BluetoothNavigator = Navigator & {
  bluetooth?: {
    requestDevice: (options: {
      filters: Array<{ name: string; services?: string[] }>;
      optionalServices?: string[];
    }) => Promise<BluetoothDeviceLike>;
  };
};

type PendingRequest = {
  opcode: number;
  requestId: number;
  bytes: Uint8Array<ArrayBuffer>;
  label: string;
  attempts: number;
  timer: ReturnType<typeof setTimeout> | null;
  resolve: (response: ExoBleResponse) => void;
  reject: (reason: Error) => void;
};

type ProtocolLogEntry = {
  id: number;
  direction: LogDirection;
  isoTime: string;
  monotonicMs: number;
  bytes?: Uint8Array<ArrayBuffer>;
  message: string;
  sessionId: string;
  deviceId: string;
};

type JointSample = {
  sessionId: string;
  deviceId: string;
  bootId: number;
  firmware: string;
  runId: number;
  side: "LEFT" | "RIGHT";
  requestId: number;
  receivedIsoTime: string;
  receivedMonotonicMs: number;
  estimatedFeedbackMonotonicMs: number;
  jointFlags: number;
  direction: number;
  positionMrad: number;
  velocityMradS: number;
  positionDegrees: number;
  velocityDegreesPerSecond: number;
  feedbackAgeMs: number;
  systemState: number;
};

type JointCommandDraft = {
  positionDegrees: string;
  velocityDegreesPerSecond: string;
  torqueNm: string;
  kp: string;
  kd: string;
};

/**
 * 助力配置是低频产品参数，不是 100 Hz 控制命令。这里保留字符串形式的
 * 数字输入，允许操作者正常编辑；真正发送前再统一检查整数范围。
 */
type AssistConfigDraft = {
  selectedMode: number;
  executionState: number;
  strategyId: string;
  profileId: string;
  leftLevelPercent: string;
  rightLevelPercent: string;
};

const INITIAL_JOINT_COMMAND_DRAFT: JointCommandDraft = {
  positionDegrees: "0",
  velocityDegreesPerSecond: "0",
  torqueNm: "0",
  kp: "0",
  kd: "0.1",
};

/** 隔离工程激励必须使用非零刚度，否则参考轨迹变化也不会产生关节运动。 */
const ENGINEERING_TEST_JOINT_COMMAND_DRAFT: JointCommandDraft = {
  positionDegrees: "0",
  velocityDegreesPerSecond: "0",
  torqueNm: "0",
  kp: "5",
  kd: "0.1",
};

/** 产品待机恢复为纯低阻尼，不保留工程激励的刚度。 */
const PRODUCT_DAMPING_JOINT_COMMAND_DRAFT: JointCommandDraft = {
  positionDegrees: "0",
  velocityDegreesPerSecond: "0",
  torqueNm: "0",
  kp: "0",
  kd: "0.1",
};

const INITIAL_ASSIST_CONFIG_DRAFT: AssistConfigDraft = {
  selectedMode: AssistMode.TRANSPARENT,
  executionState: AssistExecutionState.DISABLED,
  strategyId: "1",
  profileId: "0",
  leftLevelPercent: "0",
  rightLevelPercent: "0",
};

const RESPONSE_TIMEOUT_MS = 3000;
const WRITE_TIMEOUT_MS = 3000;
const TERMINAL_TIMEOUT_MS = 6000;
const POLL_INTERVAL_MS = 250;
const MAX_LOG_ENTRIES = 5000;
// 完整事务仍保留供导出，页面只渲染最近200条，避免每次Notify重绘上万DOM节点。
const MAX_VISIBLE_LOG_ENTRIES = 200;
const MAX_SAMPLES = 10000;
const MAX_TELEMETRY_SAMPLES = 10000;

const EMPTY_TELEMETRY_STATS: TelemetryAccumulatorStats = {
  packetCount: 0,
  uniqueSampleCount: 0,
  duplicateChannelCount: 0,
  webMissingSampleCount: 0,
  sentinelMismatchCount: 0,
};

/**
 * 把浏览器内部的 BLE 步骤翻译成操作者能理解的中文。出现故障时，
 * 用户可以直接看出是“还没连上”，还是“连上后找不到服务/通知”。
 */
const connectStageLabels: Record<ConnectStage, string> = {
  idle: "等待连接",
  chooser: "等待选择 ESP-EXO",
  gatt: "正在建立 GATT 连接",
  service: "正在发现 Product Service",
  characteristics: "正在读取 Command / Response 特征",
  notify: "正在订阅 Response Notify",
  handshake: "正在读取设备身份与初始状态",
  ready: "连接与初始状态读取完成",
};

function errorMessageOf(caught: unknown): string {
  return caught instanceof Error ? caught.message : String(caught);
}

/** 根据失败阶段给出下一步，而不是只显示浏览器的英文异常。 */
function describeConnectFailure(stage: ConnectStage, caught: unknown): string {
  const raw = errorMessageOf(caught);
  const browserDetail = raw ? `（浏览器信息：${raw}）` : "";

  switch (stage) {
    case "gatt":
      return `GATT 连接失败：已配对不等于已连接。请先断开手机、其他网页或 BLE 工具，再点击“重试已选设备”。${browserDetail}`;
    case "service":
      if (/disconnected|not connected/i.test(raw)) {
        return `蓝牙在读取产品服务时已断开，请点击“重试已选设备”。${browserDetail}`;
      }
      return `已建立 GATT，但没有发现 ESP-EXO Product Service；请核对所选设备和固件版本。${browserDetail}`;
    case "characteristics":
      return `已发现 Product Service，但 Command / Response 特征不完整。${browserDetail}`;
    case "notify":
      return `已找到产品特征，但订阅 Response Notify 失败。${browserDetail}`;
    case "handshake":
      return `BLE 通道已建立，但设备身份或初始状态读取失败；操作仍保持锁定。${browserDetail}`;
    default:
      return `BLE 连接未完成。${browserDetail}`;
  }
}

class StopPreemptionError extends Error {
  constructor() {
    super("当前 BLE 等待已由 STOP 接管");
    this.name = "StopPreemptionError";
  }
}

class GattWriteTimeoutError extends Error {
  constructor() {
    super(`BLE 写入在 ${WRITE_TIMEOUT_MS / 1000} 秒内未完成`);
    this.name = "GattWriteTimeoutError";
  }
}

const delay = (milliseconds: number) =>
  new Promise<void>((resolve) => setTimeout(resolve, milliseconds));

/** 保留稳定线码名，同时给需要现场处理的合同错误提供直接中文解释。 */
function startBlockReasonDisplay(
  reason: number | undefined,
  wireName?: string,
): string {
  if (reason === undefined) return wireName ?? "未知";
  const name = wireName ?? blockReasonNames[reason] ?? `detail=${reason}`;
  const description = blockReasonDescriptions[reason];
  return description ? `${description}（${name}）` : name;
}

function responseSummary(response: ExoBleResponse): string {
  if (response.kind === "ack") {
    const detail = response.requestOpcode === Opcode.START
      ? `，detail=${startBlockReasonDisplay(response.detail)}`
      : "";
    return `ACK ${response.resultName}，state=${response.stateName}${detail}`;
  }
  if (response.kind === "info") {
    return `INFO fw=${response.firmware.join(".")}，boot=0x${response.bootId.toString(16).padStart(8, "0").toUpperCase()}`;
  }
  if (response.kind === "systemStatus") {
    return `SYSTEM ${response.stateName}，step=${response.activeStepName}，flags=0x${response.systemFlags.toString(16).padStart(2, "0").toUpperCase()}`;
  }
  if (response.kind === "controlConfig") {
    return `CONTROL selected=${response.selectedModeName}/${response.selectedReferenceName}，active=${response.activeModeName}/${response.activeReferenceName}`;
  }
  if (response.kind === "jointCommand") {
    const side = response.side === Side.LEFT ? "LEFT" : "RIGHT";
    return `${side} CMD q=${response.positionDegrees.toFixed(2)}°，dq=${response.velocityDegreesPerSecond.toFixed(2)}°/s，tau=${response.torqueNm.toFixed(3)} N·m，KP=${response.kp.toFixed(2)}，KD=${response.kd.toFixed(3)}`;
  }
  if (response.kind === "gaitStatus") {
    const left = response.leftPhasePercent === null
      ? "—"
      : `${response.leftPhasePercent.toFixed(2)}%`;
    const right = response.rightPhasePercent === null
      ? "—"
      : `${response.rightPhasePercent.toFixed(2)}%`;
    const confidence = response.confidencePercent === null
      ? "—"
      : `${response.confidencePercent.toFixed(2)}%`;
    const cadence = response.cadenceSpm === null
      ? "—"
      : `${response.cadenceSpm.toFixed(2)} spm`;
    return `GAIT ${response.estimatorStateName}，L=${left}，R=${right}，cadence=${cadence}，confidence=${confidence}`;
  }
  if (response.kind === "assistConfig") {
    return `ASSIST selected=${response.selectedModeName}，active=${response.activeModeName}，requested=${response.requestedExecutionStateName}，effective=${response.executionStateName}，strategy=${response.strategyName}`;
  }
  if (response.kind === "assistTorqueLimit") {
    return `ASSIST LIMIT selected=${response.selectedMaximumNm ?? "无效"} N·m，active=${response.activeMaximumNm ?? "未锁存"} N·m，hard=${response.hardMaximumNm} N·m`;
  }
  if (response.kind === "angleAssistConfig") {
    return `ANGLE THRESHOLD selected=${response.selectedThresholdDegrees ?? "无效"}°，active=${response.activeThresholdDegrees ?? "未锁存"}°，gen=${response.configGeneration}`;
  }
  if (response.kind === "pulseTiming") return `脉冲${response.durationMode === 0 ? "FIXED" : "PERIOD_SCALED"} 申请峰值=${response.peakNm} N·m`;
  if (response.kind === "pulseWindow") return `复位≤${response.resetDegrees}°；角度上限${response.maximumDegrees}°`;
  if (response.kind === "pulseStatus") return `${response.side === 0 ? "LEFT" : "RIGHT"} ${pulseStateDescriptions[response.state]} 事件=${response.eventCount} 脉冲=${response.pulseCount}`;
  if (response.kind === "telemetryStatus") {
    return `TELEMETRY ${response.streaming ? "STREAMING" : "STOPPED"}，${response.rateHz} Hz，mask=${formatHexMask(response.channelMask)}，stream=${response.streamSequence}`;
  }
  if (response.kind === "canStatus") return `CAN ${response.stateName}, TEC=${response.txErrorCounter}, REC=${response.rxErrorCounter}, bus_errors=${response.busErrorCount}`;
  if (response.kind === "calibrationStatus") {
    return `CALIBRATION ${response.side === Side.LEFT ? "LEFT" : "RIGHT"} CAN=0x${response.canId.toString(16)}，raw=${response.rawPositionDegrees?.toFixed(3) ?? "无效"}°，zero=${response.softwareZeroDegrees?.toFixed(3) ?? "未标定"}°`;
  }
  if (response.kind === "savedZeroStatus") {
    return `SAVED_ZERO ${response.side === Side.LEFT ? "LEFT" : "RIGHT"} stored=${response.stored} preview=${response.previewJointDegrees?.toFixed(3) ?? "无效"}° reason=${response.reason}`;
  }
  const side = response.side === Side.LEFT ? "LEFT" : "RIGHT";
  return `${side} q=${response.positionDegrees.toFixed(2)}°，dq=${response.velocityDegreesPerSecond.toFixed(2)}°/s，age=${response.feedbackAgeMs} ms`;
}

function responseOfKind<K extends ExoBleResponse["kind"]>(
  response: ExoBleResponse,
  kind: K,
): Extract<ExoBleResponse, { kind: K }> {
  if (response.kind !== kind) {
    throw new Error(`设备返回了 ${response.kind}，但当前操作需要 ${kind}`);
  }
  return response as Extract<ExoBleResponse, { kind: K }>;
}

function formatBootId(bootId: number | undefined): string {
  return bootId === undefined
    ? "—"
    : `0x${bootId.toString(16).padStart(8, "0").toUpperCase()}`;
}

function downloadText(filename: string, content: string, type: string): void {
  const blob = new Blob([content], { type });
  const url = URL.createObjectURL(blob);
  const anchor = document.createElement("a");
  anchor.href = url;
  anchor.download = filename;
  anchor.click();
  URL.revokeObjectURL(url);
}

function commandDraftFromResponse(
  response: JointCommandResponse,
): JointCommandDraft {
  return {
    positionDegrees: response.positionDegrees.toFixed(3),
    velocityDegreesPerSecond: response.velocityDegreesPerSecond.toFixed(3),
    torqueNm: response.torqueNm.toFixed(3),
    kp: response.kp.toFixed(2),
    kd: response.kd.toFixed(3),
  };
}

function finiteDraftNumber(value: string, label: string): number {
  const parsed = Number(value);
  if (!Number.isFinite(parsed)) throw new Error(`${label}必须是有限数字`);
  return parsed;
}

function integerDraftNumber(
  value: string,
  label: string,
  minimum = 0,
  maximum = 0xff,
): number {
  const parsed = finiteDraftNumber(value, label);
  if (!Number.isInteger(parsed) || parsed < minimum || parsed > maximum) {
    throw new Error(`${label}必须是 ${minimum}～${maximum} 的整数`);
  }
  return parsed;
}

function assistDraftFromResponse(
  response: AssistConfigResponse,
): AssistConfigDraft {
  return {
    selectedMode: response.selectedMode,
    // 阶段10需要如实显示设备已请求的执行态；真正写 ACTIVE 前仍会在
    // writeAssistConfig() 中再次检查固件能力和 DAMPING + CONSTANT 组合。
    executionState: response.requestedExecutionState,
    strategyId: String(response.strategyId),
    profileId: String(response.profileId),
    leftLevelPercent: String(response.leftLevelPercent),
    rightLevelPercent: String(response.rightLevelPercent),
  };
}

function formatHexMask(value: number | undefined, width = 4): string {
  return value === undefined
    ? "—"
    : `0x${value.toString(16).padStart(width, "0").toUpperCase()}`;
}

function defaultStrategyIdForMode(mode: number): string {
  switch (mode) {
    case AssistMode.TRANSPARENT:
      return "1";
    case AssistMode.ACTIVE_PHASE:
      return "2";
    case AssistMode.ANGLE_THRESHOLD:
      return "4";
    case AssistMode.ASSIST_AS_NEEDED:
      return "3";
    default:
      return "0";
  }
}

function assistUnavailableReasonLabel(
  config: AssistConfigResponse | null,
): string {
  if (!config) return "—";
  const description = assistUnavailableReasonDescriptions[config.unavailableReason];
  return description
    ? `${description}（${config.unavailableReasonName}）`
    : config.unavailableReasonName;
}

function sensorMaskDescription(
  decoded: AssistConfigResponse["decodedAvailableSensorMask"] | undefined,
): string {
  if (!decoded) return "—";
  const names = [
    decoded.leftHipKinematics && "左髋运动学",
    decoded.rightHipKinematics && "右髋运动学",
    decoded.leftMotorTorqueEstimate && "左电机估算力矩",
    decoded.rightMotorTorqueEstimate && "右电机估算力矩",
    decoded.lowerLimbImu && "下肢 IMU",
    decoded.footContact && "足接触",
    decoded.interactionTorque && "交互力矩",
    decoded.semg && "sEMG",
  ].filter(Boolean);
  if (decoded.unknown) names.push(`未知位 ${formatHexMask(decoded.unknown)}`);
  return names.length ? names.join("、") : "无";
}

function jointCommandFromDraft(
  side: number,
  draft: JointCommandDraft,
): JointCommandValues {
  return {
    side,
    positionDegrees: finiteDraftNumber(draft.positionDegrees, "目标角度"),
    velocityDegreesPerSecond: finiteDraftNumber(
      draft.velocityDegreesPerSecond,
      "目标角速度",
    ),
    torqueNm: finiteDraftNumber(draft.torqueNm, "前馈力矩"),
    kp: finiteDraftNumber(draft.kp, "KP"),
    kd: finiteDraftNumber(draft.kd, "KD"),
  };
}

function Flag({ ok, children }: { ok: boolean; children: ReactNode }) {
  return <span className={`exo-flag ${ok ? "ok" : "off"}`}>{children}</span>;
}

function JointCommandEditor({
  title,
  side,
  draft,
  readback,
  unavailable,
  writeDisabled,
  onDraft,
  onRead,
  onWrite,
}: {
  title: string;
  side: number;
  draft: JointCommandDraft;
  readback: JointCommandResponse | null;
  unavailable: boolean;
  writeDisabled: boolean;
  onDraft: (draft: JointCommandDraft) => void;
  onRead: () => void;
  onWrite: () => void;
}) {
  const setField = (field: keyof JointCommandDraft, value: string) =>
    onDraft({ ...draft, [field]: value });
  const sideName = side === Side.LEFT ? "LEFT" : "RIGHT";
  return (
    <fieldset className="exo-command-editor" disabled={unavailable}>
      <legend>{title}</legend>
      <div className="exo-command-fields">
        <label><span>目标角度 q（deg）</span><input disabled={writeDisabled} type="number" step="0.1" value={draft.positionDegrees} onChange={(event) => setField("positionDegrees", event.target.value)} /></label>
        <label><span>目标角速度 dq（deg/s）</span><input disabled={writeDisabled} type="number" step="0.1" value={draft.velocityDegreesPerSecond} onChange={(event) => setField("velocityDegreesPerSecond", event.target.value)} /></label>
        <label><span>前馈力矩 τ（N·m）</span><input disabled={writeDisabled} type="number" step="0.01" value={draft.torqueNm} onChange={(event) => setField("torqueNm", event.target.value)} /></label>
        <label><span>KP（RS01增益）</span><input disabled={writeDisabled} type="number" min="0" max="500" step="0.01" value={draft.kp} onChange={(event) => setField("kp", event.target.value)} /></label>
        <label><span>KD（RS01增益）</span><input disabled={writeDisabled} type="number" min="0" max="5" step="0.001" value={draft.kd} onChange={(event) => setField("kd", event.target.value)} /></label>
      </div>
      <div className="exo-command-buttons">
        <button type="button" className="exo-button ghost" onClick={onRead}>读回 {sideName}</button>
        <button type="button" className="exo-button secondary" disabled={writeDisabled} onClick={onWrite}>写入并读回</button>
      </div>
      <p className="exo-command-readback">
        设备值：{readback
          ? `q ${readback.positionDegrees.toFixed(3)}° · dq ${readback.velocityDegreesPerSecond.toFixed(3)}°/s · τ ${readback.torqueNm.toFixed(3)} N·m · KP ${readback.kp.toFixed(2)} · KD ${readback.kd.toFixed(3)}`
          : "尚未读取"}
      </p>
    </fieldset>
  );
}

function JointCard({
  title,
  expected,
  joint,
}: {
  title: string;
  expected: string;
  joint: JointStatusResponse | null;
}) {
  const flags = joint?.decodedJointFlags;
  return (
    <article className="exo-card exo-joint-card">
      <div className="exo-card-heading">
        <div>
          <p className="exo-kicker">髋关节快照</p>
          <h2>{title}</h2>
        </div>
        <span className={`exo-state-chip ${flags?.jointValid ? "good" : "warn"}`}>
          {flags?.jointValid ? "有效" : "未就绪"}
        </span>
      </div>
      <p className="exo-expected">设备映射：{expected}</p>
      <div className="exo-joint-value">
        <strong>{flags?.jointValid ? joint!.positionDegrees.toFixed(2) : "—"}</strong>
        <span>deg</span>
      </div>
      <div className="exo-data-grid compact">
        <div><span>速度</span><b>{flags?.jointValid ? `${joint!.velocityDegreesPerSecond.toFixed(2)} deg/s` : "—"}</b></div>
        <div><span>反馈年龄</span><b>{joint ? `${joint.feedbackAgeMs} ms` : "—"}</b></div>
        <div><span>方向</span><b>{joint?.directionName ?? "—"}</b></div>
        <div><span>request_id</span><b>{joint?.requestId ?? "—"}</b></div>
      </div>
      <div className="exo-flags">
        <Flag ok={Boolean(flags?.mappingValid)}>映射</Flag>
        <Flag ok={Boolean(flags?.directionValid)}>方向</Flag>
        <Flag ok={Boolean(flags?.zeroValid)}>零位</Flag>
        <Flag ok={Boolean(flags?.feedbackValid)}>反馈</Flag>
        <Flag ok={Boolean(flags?.online)}>在线</Flag>
        <Flag ok={Boolean(flags?.jointValid)}>最终有效</Flag>
      </div>
    </article>
  );
}

const TELEMETRY_CHANNEL_OPTIONS = [
  { bit: TelemetryChannel.POSITION, label: "髋角度" },
  { bit: TelemetryChannel.VELOCITY, label: "髋角速度" },
  { bit: TelemetryChannel.MOTOR_TORQUE, label: "电机反馈力矩" },
  { bit: TelemetryChannel.GAIT_PHASE, label: "步态相位" },
  { bit: TelemetryChannel.ASSIST_SUGGESTION, label: "建议助力" },
  { bit: TelemetryChannel.GAIT_CONFIDENCE, label: "步态置信度" },
  { bit: TelemetryChannel.APPLIED_TORQUE, label: "实际助力增量" },
] as const;

function countMaskBits(mask: number): number {
  let count = 0;
  let remaining = mask >>> 0;
  while (remaining !== 0) {
    count += remaining & 1;
    remaining >>>= 1;
  }
  return count;
}

const CONSOLE_PANELS = [
  {id: "connection", label: "连接", description: "通过蓝牙连接外骨骼，确认设备在线"},
  {id: "settings", label: "设置", description: "人体穿戴后的左右电机核对与零位校准"},
  {id: "motion", label: "运动", description: "调节左右助力等级，启动与停止外骨骼"},
  {id: "live", label: "实时数据", description: "查看左右髋关节与当前步态信息"},
  {id: "records", label: "数据记录", description: "按日期查看与导出真实步态记录"},
] as const;
type ConsolePanel = typeof CONSOLE_PANELS[number]["id"];
const productSystemStateNames: Record<number, string> = {0: "初始化", 1: "待机", 2: "启动中", 3: "运行中", 4: "停止中", 5: "故障"};

export default function ExoConsolePage() {
  // 栏目切换只隐藏内容，不卸载BLE会话和记录器。
  const [activePanel, setActivePanel] = useState<ConsolePanel>("connection");
  const currentPanel = CONSOLE_PANELS.find((panel) => panel.id === activePanel)!;
  const [bleSupported, setBleSupported] = useState<boolean | null>(null);
  const [secureContext, setSecureContext] = useState<boolean | null>(null);
  const [connection, setConnection] = useState<ConnectionState>("idle");
  const [connectStage, setConnectStage] = useState<ConnectStage>("idle");
  const [lastFailedConnectStage, setLastFailedConnectStage] =
    useState<ConnectStage | null>(null);
  const [hasRememberedDevice, setHasRememberedDevice] = useState(false);
  const [deviceName, setDeviceName] = useState<string>(EXO_BLE.DEVICE_NAME);
  const [info, setInfo] = useState<InfoResponse | null>(null);
  const [system, setSystem] = useState<SystemStatusResponse | null>(null);
  const [leftJoint, setLeftJoint] = useState<JointStatusResponse | null>(null);
  const [rightJoint, setRightJoint] = useState<JointStatusResponse | null>(null);
  const [canStatus, setCanStatus] = useState<CanStatusResponse | null>(null);
  const [canErrorDelta, setCanErrorDelta] = useState<number | null>(null);
  const [feedbackRestoreNotice, setFeedbackRestoreNotice] = useState("");
  const previousCanRef = useRef<CanStatusResponse | null>(null);
  const canPollMsRef = useRef(0);
  const [leftCalibration, setLeftCalibration] = useState<CalibrationStatusResponse | null>(null);
  const [rightCalibration, setRightCalibration] = useState<CalibrationStatusResponse | null>(null);
  const [leftSavedZero, setLeftSavedZero] = useState<SavedZeroStatusResponse | null>(null);
  const [rightSavedZero, setRightSavedZero] = useState<SavedZeroStatusResponse | null>(null);
  const [wearZeroChoice, setWearZeroChoice] = useState<"new" | "reuse">("new");
  const [saveWearZeroForReuse, setSaveWearZeroForReuse] = useState(true);
  const [wearZeroNotice, setWearZeroNotice] = useState("");
  const wearZeroChoiceChosenRef = useRef(false);
  // 这是现场人工核对记录，不是固件自动识别左右腿。断连/重启/反馈丢失后必须重做。
  const [leftMappingConfirmed, setLeftMappingConfirmed] = useState(false);
  const [rightMappingConfirmed, setRightMappingConfirmed] = useState(false);
  const [directionTeaching, setDirectionTeaching] = useState<{side: number; token: number} | null>(null);
  const [directionTeachNotice, setDirectionTeachNotice] = useState("");
  const nextDirectionTokenRef = useRef(1);
  const [humanWornMode, setHumanWornMode] = useState(true);
  const [controlConfig, setControlConfig] =
    useState<ControlConfigResponse | null>(null);
  const [leftCommand, setLeftCommand] =
    useState<JointCommandResponse | null>(null);
  const [rightCommand, setRightCommand] =
    useState<JointCommandResponse | null>(null);
  const [gaitStatus, setGaitStatus] = useState<GaitStatusResponse | null>(null);
  const [assistConfig, setAssistConfig] =
    useState<AssistConfigResponse | null>(null);
  const [angleAssistConfig, setAngleAssistConfig] = useState<AngleAssistConfigResponse | null>(null);
  const [angleThresholdDraft, setAngleThresholdDraft] = useState("10.00");
  const [pulseTiming, setPulseTiming] = useState<PulseTimingResponse | null>(null);
  const [pulseWindow, setPulseWindow] = useState<PulseWindowResponse | null>(null);
  const [pulseLeft, setPulseLeft] = useState<PulseStatusResponse | null>(null);
  const [pulseRight, setPulseRight] = useState<PulseStatusResponse | null>(null);
  const pulseTimingKeyRef = useRef("");
  const pulseWindowKeyRef = useRef("");
  const [pulseDraft, setPulseDraft] = useState({durationMode:0, peakNm:"0.500",fixedMs:"600",ratio:"0.300",
    minimumMs:"150",maximumMs:"1500",periodMinimumMs:"400",periodMaximumMs:"3000",
    resetDegrees:"8.00",maximumDegrees:"30.00"});
  const selectedAngleRef = useRef<number | null>(null);
  const [assistTorqueLimit, setAssistTorqueLimit] = useState<AssistTorqueLimitResponse | null>(null);
  const [maximumAssistDraft, setMaximumAssistDraft] = useState("0.500");
  const [telemetryStatus, setTelemetryStatus] =
    useState<TelemetryStatusResponse | null>(null);
  const [telemetryCharacteristicReady, setTelemetryCharacteristicReady] =
    useState(false);
  const [telemetryRateHz, setTelemetryRateHz] = useState<5 | 10 | 20>(10);
  const [telemetryChannelMask, setTelemetryChannelMask] = useState<number>(
    WALKING_SCOPE_MASK,
  );
  const [telemetryAutoStart, setTelemetryAutoStart] = useState(true);
  const [scopeWindowSeconds, setScopeWindowSeconds] = useState(10);
  const [scopeAutoScale, setScopeAutoScale] = useState(true);
  const [scopeShowLeft, setScopeShowLeft] = useState(true);
  const [scopeShowRight, setScopeShowRight] = useState(true);
  const [scopeClockMs, setScopeClockMs] = useState(0);
  const [lastTelemetryReceivedMs, setLastTelemetryReceivedMs] = useState<number | null>(null);
  const [telemetryDisplayPaused, setTelemetryDisplayPaused] = useState(false);
  const [telemetrySamples, setTelemetrySamples] =
    useState<TelemetrySamplePoint[]>([]);
  const [telemetryStats, setTelemetryStats] =
    useState<TelemetryAccumulatorStats>({ ...EMPTY_TELEMETRY_STATS });
  // 产品页面默认是低阻尼基础控制。工程验证激励有独立入口，不能通过普通
  // 产品选择器“顺手”留下一个会主动摆动关节的配置。
  const [selectedMode, setSelectedMode] = useState<number>(ControlMode.DAMPING);
  const [selectedReference, setSelectedReference] =
    useState<number>(ReferenceSource.CONSTANT);
  const [leftCommandDraft, setLeftCommandDraft] = useState<JointCommandDraft>({
    ...INITIAL_JOINT_COMMAND_DRAFT,
  });
  const [rightCommandDraft, setRightCommandDraft] = useState<JointCommandDraft>({
    ...INITIAL_JOINT_COMMAND_DRAFT,
  });
  const [assistConfigDraft, setAssistConfigDraft] =
    useState<AssistConfigDraft>({ ...INITIAL_ASSIST_CONFIG_DRAFT });
  const [lastAck, setLastAck] = useState<AckResponse | null>(null);
  const [operation, setOperation] = useState("");
  const [error, setError] = useState("");
  const [neutralConfirmed, setNeutralConfirmed] = useState(false);
  // 设备已有零位不代表这一次穿戴已校准；只有本页双侧写入并读回成功才记录时间。
  const [lastWearCalibrationAt, setLastWearCalibrationAt] = useState<string | null>(null);
  const [autoPolling, setAutoPolling] = useState(false);
  const [logs, setLogs] = useState<ProtocolLogEntry[]>([]);
  const [followLatestLog, setFollowLatestLog] = useState(true);
  const [samples, setSamples] = useState<JointSample[]>([]);
  const [completedRuns, setCompletedRuns] = useState(0);
  const [currentRunId, setCurrentRunId] = useState(0);

  // rememberedDeviceRef 表示“浏览器已经让用户授权选择过的设备”。
  // deviceRef 只表示“当前这一次活动 GATT 会话”。把两者分开后，建连失败
  // 仍可复用已选设备，而断连清理也不会把设备选择结果一起丢掉。
  const rememberedDeviceRef = useRef<BluetoothDeviceLike | null>(null);
  const deviceRef = useRef<BluetoothDeviceLike | null>(null);
  const serverRef = useRef<BluetoothRemoteGATTServerLike | null>(null);
  const commandRef = useRef<BluetoothRemoteGATTCharacteristicLike | null>(null);
  const responseRef = useRef<BluetoothRemoteGATTCharacteristicLike | null>(null);
  const telemetryRef = useRef<BluetoothRemoteGATTCharacteristicLike | null>(null);
  const connectedRef = useRef(false);
  const pendingRef = useRef(new Map<number, PendingRequest>());
  const nextRequestIdRef = useRef(1);
  const queueRef = useRef<Promise<void>>(Promise.resolve());
  const writeQueueRef = useRef<Promise<void>>(Promise.resolve());
  const connectionGenerationRef = useRef(0);
  const connectAttemptRef = useRef(0);
  const nextLogIdRef = useRef(1);
  const pollBusyRef = useRef(false);
  const operationRef = useRef("");
  const systemRef = useRef<SystemStatusResponse | null>(null);
  const infoRef = useRef<InfoResponse | null>(null);
  const sessionIdRef = useRef("");
  const deviceIdRef = useRef("");
  const runIdRef = useRef(0);
  const startWaitGenerationRef = useRef(0);
  const logContainerRef = useRef<HTMLDivElement | null>(null);
  // ref 供高频日志 effect 读取最新值；state 只负责把“正在跟随/已暂停”
  // 状态显示给用户。这样不会因为闭包拿到旧值而错误地跳回底部。
  const followLatestLogRef = useRef(true);
  const telemetryAccumulatorRef = useRef(new TelemetryStreamAccumulator());
  const telemetryCaptureRef = useRef<TelemetrySamplePoint[]>([]);
  const telemetryDisplayPausedRef = useRef(false);
  const telemetryDirtyRef = useRef(false);
  const telemetryLastReceivedRef = useRef<number | null>(null);
  const telemetryAutoAttemptRef = useRef<number | null>(null);
  const unknownTelemetryLoggedRef = useRef(new Set<string>());
  const telemetryStatusBusyRef = useRef(false);
  const calibrationPollMsRef = useRef(0);
  const assistPollMsRef = useRef(0);
  const assistGenerationRef = useRef<number | null>(null);
  const selectedTorqueRef = useRef<number | null>(null);

  const appendLog = useCallback((
    direction: LogDirection,
    message: string,
    bytes?: Uint8Array<ArrayBuffer>,
  ) => {
    const entry: ProtocolLogEntry = {
      id: nextLogIdRef.current++,
      direction,
      isoTime: new Date().toISOString(),
      monotonicMs: performance.now(),
      bytes: bytes ? Uint8Array.from(bytes) : undefined,
      message,
      sessionId: sessionIdRef.current,
      deviceId: deviceIdRef.current,
    };
    setLogs((current) => [...current.slice(-(MAX_LOG_ENTRIES - 1)), entry]);
  }, []);

  const rejectAllPending = useCallback((reason: string) => {
    for (const pending of pendingRef.current.values()) {
      if (pending.timer) clearTimeout(pending.timer);
      pending.reject(new Error(reason));
    }
    pendingRef.current.clear();
  }, []);

  const cancelPendingForStop = useCallback(() => {
    for (const pending of pendingRef.current.values()) {
      if (pending.timer) clearTimeout(pending.timer);
      pending.reject(new StopPreemptionError());
    }
    pendingRef.current.clear();
  }, []);

  const resetConnection = useCallback((reason?: string) => {
    ++connectAttemptRef.current;
    connectedRef.current = false;
    connectionGenerationRef.current += 1;
    deviceRef.current = null;
    commandRef.current = null;
    responseRef.current = null;
    telemetryRef.current = null;
    serverRef.current = null;
    systemRef.current = null;
    infoRef.current = null;
    pollBusyRef.current = false;
    nextRequestIdRef.current = 1;
    setAutoPolling(false);
    setTelemetryCharacteristicReady(false);
    setTelemetryStatus(null);
    setTelemetrySamples([]);
    telemetryLastReceivedRef.current = null;
    telemetryDirtyRef.current = false;
    setLastTelemetryReceivedMs(null);
    telemetryAccumulatorRef.current.reset();
    setTelemetryStats({ ...EMPTY_TELEMETRY_STATS });
    setNeutralConfirmed(false);
    setLastWearCalibrationAt(null);
    setConnection("idle");
    setDirectionTeaching(null);
    setDirectionTeachNotice("");
    setConnectStage("idle");
    setLastFailedConnectStage(null);
    // 断开后不能把最后一次快照继续显示成“当前状态”。日志和采样仍保留，
    // 重连后再重新读取设备身份、系统状态和双侧关节。
    setInfo(null);
    setCanStatus(null);
    setCanErrorDelta(null);
    previousCanRef.current = null;
    setFeedbackRestoreNotice("");
    setSystem(null);
    setLeftJoint(null);
    setRightJoint(null);
    setLeftCalibration(null);
    setRightCalibration(null);
    setLeftSavedZero(null);
    setRightSavedZero(null);
    setWearZeroChoice("new");
    setWearZeroNotice("");
    wearZeroChoiceChosenRef.current = false;
    setLeftMappingConfirmed(false);
    setRightMappingConfirmed(false);
    setControlConfig(null);
    setLeftCommand(null);
    setRightCommand(null);
    setGaitStatus(null);
    setAssistConfig(null);
    setAssistTorqueLimit(null);
    setAngleAssistConfig(null);
    setPulseTiming(null);setPulseWindow(null);setPulseLeft(null);setPulseRight(null);
    pulseTimingKeyRef.current="";pulseWindowKeyRef.current="";
    selectedAngleRef.current = null;
    setAngleThresholdDraft("10.00");
    setMaximumAssistDraft("0.500");
    assistGenerationRef.current = null;
    selectedTorqueRef.current = null;
    assistPollMsRef.current = 0;
    setAssistConfigDraft({ ...INITIAL_ASSIST_CONFIG_DRAFT });
    setLastAck(null);
    rejectAllPending(reason ?? "BLE 已断开");
  }, [rejectAllPending]);

  const handleDisconnected = useCallback(() => {
    // 当前活动引用需要立即失效；下次建立连接时会先 remove 再 add，
    // 因此同一个已记住设备不会累积多个断连监听器。
    resetConnection("BLE 连接意外断开，命令最终状态未知");
    setError("BLE 连接意外断开；设备仍已记住，可直接重试并先查询系统状态。");
    appendLog("error", "BLE 已断开。断连不会自动 STOP；重连后应先查询系统状态。");
  }, [appendLog, resetConnection]);

  useEffect(() => {
    if (connection !== "connected") return;
    // 浏览器断连事件可能漏报；开发热更新也会先释放 GATT，却保留 React
    // 的旧界面状态。这里只读连接标志，不发 BLE/CAN、不参与实时控制。
    // 一旦会话失效就撤销人工确认和旧快照，不能继续显示成“当前在线”。
    const connectionCheckTimer = window.setInterval(() => {
      if (!connectedRef.current || !deviceRef.current?.gatt?.connected) {
        handleDisconnected();
      }
    }, 1000);
    return () => window.clearInterval(connectionCheckTimer);
  }, [connection, handleDisconnected]);

  const applyResponse = useCallback((response: ExoBleResponse) => {
    if (response.kind === "info") {
      if (infoRef.current && infoRef.current.bootId !== response.bootId) {
        setCanStatus(null);
        setCanErrorDelta(null);
        previousCanRef.current = null;
        setFeedbackRestoreNotice("");
        setDirectionTeaching(null);
        setDirectionTeachNotice("");
        setNeutralConfirmed(false);
        setLastWearCalibrationAt(null);
        setLeftMappingConfirmed(false);
        setRightMappingConfirmed(false);
        setLeftCalibration(null);
        setRightCalibration(null);
        setLeftSavedZero(null);
        setRightSavedZero(null);
        setPulseTiming(null);setPulseWindow(null);setPulseLeft(null);setPulseRight(null);
        pulseTimingKeyRef.current="";pulseWindowKeyRef.current="";
        setWearZeroChoice("new");
        setWearZeroNotice("");
        wearZeroChoiceChosenRef.current = false;
      }
      setInfo(response);
      infoRef.current = response;
      return;
    }
    if (response.kind === "systemStatus") {
      const previousSystem = systemRef.current;
      setSystem(response);
      systemRef.current = response;

      // “双髋处于中立位”只对当前 IDLE 姿态成立。设备一旦开始启停或运行，
      // 必须让操作者下次归零前重新确认，不能沿用旧勾选状态。
      if (response.state !== 0x01) setNeutralConfirmed(false);

      // 页面可能在设备已经 RUNNING 时才重连。此时补建一个恢复轮次，
      // 让关节样本与 CSV 仍能工作，而不是永远停在 run_id=0。
      if (response.state === 0x03 &&
          response.decodedSystemFlags.outputAllowed &&
          runIdRef.current === 0) {
        runIdRef.current = 1;
        setCurrentRunId(1);
        appendLog("system", "检测到设备已处于 RUNNING，已建立第 1 个重连恢复轮次。");
      }

      // 若 STOP 由别的客户端完成，本页也根据状态跃迁结算当前轮次。
      if (previousSystem?.state === 0x03 && response.state === 0x01 &&
          runIdRef.current > 0) {
        setCompletedRuns((count) => Math.max(count, runIdRef.current));
      }
      return;
    }
    if (response.kind === "ack") {
      setLastAck(response);
      return;
    }
    if (response.kind === "canStatus") {
      const previous = previousCanRef.current;
      setCanErrorDelta(previous ? ((response.busErrorCount - previous.busErrorCount) >>> 0) : null);
      previousCanRef.current = response;
      setCanStatus(response);
      return;
    }
    if (response.kind === "savedZeroStatus") {
      if (response.side === Side.LEFT) setLeftSavedZero(response);
      else setRightSavedZero(response);
      return;
    }
    if (response.kind === "calibrationStatus") {
      // 0.10.4的方向有效位仅在NVS加载/保存成功后成立；重连不要求重复出厂标定。
      const persistentDirection = ((infoRef.current?.capabilities ?? 0) & Capability.DIRECTION_TEACHING) !== 0;
      const configured = response.decodedJointFlags.mappingValid && response.decodedJointFlags.directionValid;
      if (response.side === Side.LEFT) {
        setLeftCalibration(response);
        if (persistentDirection) setLeftMappingConfirmed(configured);
        else if (response.rawPositionDegrees === null) setLeftMappingConfirmed(false);
      } else {
        setRightCalibration(response);
        if (persistentDirection) setRightMappingConfirmed(configured);
        else if (response.rawPositionDegrees === null) setRightMappingConfirmed(false);
      }
      return;
    }
    if (response.kind === "controlConfig") {
      setControlConfig(response);
      // 隔离工程激励不回填普通产品选择器。设备即使当前保存了 TEST，产品编辑
      // 草稿仍保持 DAMPING + CONSTANT，避免操作者把工程参考误当产品配置。
      if (response.selectedValid &&
          response.selectedReference !== ReferenceSource.ALTERNATING_SINE_TEST) {
        setSelectedMode(response.selectedMode);
        setSelectedReference(response.selectedReference);
      }
      return;
    }
    if (response.kind === "jointCommand") {
      const draft = commandDraftFromResponse(response);
      if (response.side === Side.LEFT) {
        setLeftCommand(response);
        setLeftCommandDraft(draft);
      } else {
        setRightCommand(response);
        setRightCommandDraft(draft);
      }
      return;
    }
    if (response.kind === "gaitStatus") {
      setGaitStatus(response);
      return;
    }
    if (response.kind === "assistConfig") {
      setAssistConfig(response);
      // 低频更新活动门禁不覆盖用户正在编辑的同一代草稿。
      if (response.decodedAssistFlags.selectedValid &&
          assistGenerationRef.current !== response.configGeneration) {
        assistGenerationRef.current = response.configGeneration;
        setAssistConfigDraft(assistDraftFromResponse(response));
      }
      return;
    }
    if(response.kind === "pulseTiming") {
      setPulseTiming(response);
      const key=JSON.stringify([response.durationMode,response.peakNm,response.fixedMs,response.ratio,
        response.minimumMs,response.maximumMs,response.periodMinimumMs,response.periodMaximumMs]);
      if(pulseTimingKeyRef.current!==key) {
        pulseTimingKeyRef.current=key;
        setPulseDraft(d=>({...d,durationMode:response.durationMode,peakNm:response.peakNm.toFixed(3),
          fixedMs:String(response.fixedMs),ratio:response.ratio.toFixed(3),minimumMs:String(response.minimumMs),
          maximumMs:String(response.maximumMs),periodMinimumMs:String(response.periodMinimumMs),
          periodMaximumMs:String(response.periodMaximumMs)}));
      }
      return;
    }
    if(response.kind === "pulseWindow") {
      setPulseWindow(response);
      const key=`${response.resetDegrees}/${response.maximumDegrees}`;
      if(pulseWindowKeyRef.current!==key) {
        pulseWindowKeyRef.current=key;
        setPulseDraft(d=>({...d,resetDegrees:response.resetDegrees.toFixed(2),maximumDegrees:response.maximumDegrees.toFixed(2)}));
      }
      return;
    }
    if(response.kind === "pulseStatus") {
      if(response.side===Side.LEFT) setPulseLeft(response);else setPulseRight(response);
      return;
    }
    if (response.kind === "angleAssistConfig") {
      setAngleAssistConfig(response);
      if (response.selectedThresholdDegrees !== null && response.selectedThresholdDegrees >= 1 &&
          selectedAngleRef.current !== response.selectedThresholdDegrees) {
        selectedAngleRef.current = response.selectedThresholdDegrees;
        setAngleThresholdDraft(response.selectedThresholdDegrees.toFixed(2));
      }
      return;
    }
    if (response.kind === "assistTorqueLimit") {
      setAssistTorqueLimit(response);
      if (response.selectedMaximumNm !== null &&
          selectedTorqueRef.current !== response.selectedMaximumNm) {
        selectedTorqueRef.current = response.selectedMaximumNm;
        setMaximumAssistDraft(response.selectedMaximumNm.toFixed(3));
      }
      return;
    }
    if (response.kind === "telemetryStatus") {
      setTelemetryStatus(response);
      return;
    }

    // 前面的分支已经穷举除 JOINT_STATUS 外的所有结构化响应。保留显式
    // 判定可防止后续协议新增响应时被误当成关节反馈写入样本。
    if (response.kind !== "jointStatus") return;

    if (response.side === Side.LEFT) setLeftJoint(response);
    else setRightJoint(response);

    const flags = response.decodedJointFlags;
    if (!flags.zeroValid) setLastWearCalibrationAt(null);
    if (!flags.feedbackValid || !flags.online) {
      setNeutralConfirmed(false);
      if (response.side === Side.LEFT) setLeftMappingConfirmed(false);
      else setRightMappingConfirmed(false);
    }
    const activeSystem = systemRef.current;
    const activeInfo = infoRef.current;
    if (!flags.jointValid || response.feedbackAgeMs === 0xffff ||
        runIdRef.current === 0 || activeSystem?.state !== 0x03 ||
        !activeSystem.decodedSystemFlags.outputAllowed || !activeInfo) {
      return;
    }
    const receivedMonotonicMs = performance.now();
    const sample: JointSample = {
      sessionId: sessionIdRef.current,
      deviceId: deviceIdRef.current,
      bootId: activeInfo.bootId,
      firmware: activeInfo.firmware.join("."),
      runId: runIdRef.current,
      side: response.side === Side.LEFT ? "LEFT" : "RIGHT",
      requestId: response.requestId,
      receivedIsoTime: new Date().toISOString(),
      receivedMonotonicMs,
      estimatedFeedbackMonotonicMs:
        receivedMonotonicMs - response.feedbackAgeMs,
      jointFlags: response.jointFlags,
      direction: response.direction,
      positionMrad: response.positionMrad,
      velocityMradS: response.velocityMradS,
      positionDegrees: response.positionDegrees,
      velocityDegreesPerSecond: response.velocityDegreesPerSecond,
      feedbackAgeMs: response.feedbackAgeMs,
      systemState: activeSystem.state,
    };
    setSamples((current) => [...current.slice(-(MAX_SAMPLES - 1)), sample]);
  }, [appendLog]);

  const handleNotification = useCallback((event: Event) => {
    const characteristic = event.target as unknown as BluetoothRemoteGATTCharacteristicLike;
    if (characteristic !== responseRef.current) {
      appendLog("system", "已忽略来自旧 BLE 会话的 Notify。");
      return;
    }
    if (!characteristic.value) return;
    const view = characteristic.value;
    // Notify 的 DataView 可能只是底层缓冲区中的一个窗口。这里复制成独立的
    // ArrayBuffer，既避免偏移量泄漏，也符合新版 TypeScript 对 BufferSource
    // （不接受 SharedArrayBuffer）的严格定义。
    const bytes = new Uint8Array(view.byteLength);
    bytes.set(new Uint8Array(view.buffer, view.byteOffset, view.byteLength));

    try {
      const response = parseResponse(bytes);
      appendLog("rx", responseSummary(response), bytes);

      const pending = pendingRef.current.get(response.requestId);
      if (!pending) {
        appendLog("system", `收到无等待者的 Notify：request_id=${response.requestId}`);
        return;
      }

      try {
        assertCorrelated(response, pending.opcode, pending.requestId);
        if (response.kind === "assistTorqueLimit")
          assertAssistTorqueLimitCapability(response, infoRef.current?.capabilities ?? 0, infoRef.current?.firmware);
      } catch (caught) {
        if (pending.timer) clearTimeout(pending.timer);
        pendingRef.current.delete(response.requestId);
        pending.reject(caught instanceof Error ? caught : new Error("响应关联失败"));
        return;
      }

      // 只有通过 request_id 与 opcode 关联校验的 Notify 才更新界面。
      // 迟到、重复或其他客户端产生的响应只作为原始证据记录。
      applyResponse(response);
      if (pending.timer) clearTimeout(pending.timer);
      pendingRef.current.delete(response.requestId);
      pending.resolve(response);
    } catch (caught) {
      const message = caught instanceof Error ? caught.message : "无法解析 BLE Notify";
      appendLog("error", `响应解析失败：${message}`, bytes);
      setError(message);
    }
  }, [appendLog, applyResponse]);

  /**
   * 独立遥测 Notify 不套4字节命令帧头，也没有 request_id。这里只做定长解析、
   * 序号诊断和内存记录；它绝不会 resolve 命令事务，更不会触发控制写入。
   */
  const handleTelemetryNotification = useCallback((event: Event) => {
    const characteristic = event.target as unknown as BluetoothRemoteGATTCharacteristicLike;
    if (characteristic !== telemetryRef.current || !characteristic.value) return;

    const view = characteristic.value;
    const bytes = new Uint8Array(view.byteLength);
    bytes.set(new Uint8Array(view.buffer, view.byteOffset, view.byteLength));

    try {
      const packet = parseTelemetryPacket(bytes);
      if (packet.kind === "unknown") {
        const key = `${packet.schemaVersion}:${packet.packetType}`;
        if (!unknownTelemetryLoggedRef.current.has(key)) {
          unknownTelemetryLoggedRef.current.add(key);
          appendLog(
            "system",
            `已忽略未来遥测包：schema=${packet.schemaVersion}，type=${packet.packetType}`,
            bytes,
          );
        }
        return;
      }

      const accepted = telemetryAccumulatorRef.current.accept(packet);
      accepted.sample.webSession=connectionGenerationRef.current;
      const capture = telemetryCaptureRef.current;
      if (accepted.isNewSample) {
        capture.push(accepted.sample);
        if (capture.length > MAX_TELEMETRY_SAMPLES) {
          capture.splice(0, capture.length - MAX_TELEMETRY_SAMPLES);
        }
      } else {
        // 同一采样时刻的后续通道通常就是数组最后一项；倒序查找同时兼容
        // 极少量乱序 Notify，又不会遍历完整的一万条历史记录。
        for (let index = capture.length - 1;
          index >= Math.max(0, capture.length - 8);
          index -= 1) {
          const existing = capture[index];
          if (existing.streamSequence === accepted.sample.streamSequence &&
              existing.sampleSequence === accepted.sample.sampleSequence) {
            capture[index] = accepted.sample;
            break;
          }
        }
      }

      // Notify只组装/记录。统一10 Hz刷新界面，避免6个通道各自触发整页渲染。
      telemetryLastReceivedRef.current = Date.now();
      telemetryDirtyRef.current = true;
    } catch (caught) {
      const message = errorMessageOf(caught);
      appendLog("error", `遥测解析失败：${message}`, bytes);
    }
  }, [appendLog]);

  useEffect(() => {
    const timer=window.setInterval(() => {
      if (connection !== "connected") return;
      setScopeClockMs(Date.now());
      if (!telemetryDirtyRef.current) return;
      telemetryDirtyRef.current=false;
      setLastTelemetryReceivedMs(telemetryLastReceivedRef.current);
      setTelemetryStats(telemetryAccumulatorRef.current.snapshotStats());
      if (!telemetryDisplayPausedRef.current)
        setTelemetrySamples(telemetryCaptureRef.current.slice(-SCOPE_BUFFER_SAMPLES));
    },100);
    return ()=>window.clearInterval(timer);
  },[connection]);

  useEffect(() => {
    // 浏览器能力只能在客户端读取。放到下一次事件循环更新，避免服务端渲染
    // 与首次水合结果不一致，也不在 effect 主体里同步触发级联渲染。
    const capabilityTimer = window.setTimeout(() => {
      setBleSupported(Boolean((navigator as BluetoothNavigator).bluetooth));
      setSecureContext(window.isSecureContext);
    }, 0);
    const pendingRequests = pendingRef.current;
    return () => {
      window.clearTimeout(capabilityTimer);
      connectedRef.current = false;
      connectionGenerationRef.current += 1;
      for (const pending of pendingRequests.values()) {
        if (pending.timer) clearTimeout(pending.timer);
        pending.reject(new Error("页面已离开，BLE 事务已取消"));
      }
      pendingRequests.clear();

      const device = deviceRef.current;
      if (device) {
        device.removeEventListener("gattserverdisconnected", handleDisconnected);
      }
      const response = responseRef.current;
      if (response) {
        response.removeEventListener("characteristicvaluechanged", handleNotification);
        void response.stopNotifications?.().catch(() => undefined);
      }
      const telemetry = telemetryRef.current;
      if (telemetry) {
        telemetry.removeEventListener(
          "characteristicvaluechanged",
          handleTelemetryNotification,
        );
        void telemetry.stopNotifications?.().catch(() => undefined);
      }
      if (device?.gatt?.connected) device.gatt.disconnect();
      deviceRef.current = null;
      commandRef.current = null;
      responseRef.current = null;
      telemetryRef.current = null;
      serverRef.current = null;
    };
  }, [handleDisconnected, handleNotification, handleTelemetryNotification]);

  /** 同步更新日志跟随状态，同时避免重复 setState 造成无意义渲染。 */
  const updateLogFollowState = useCallback((next: boolean) => {
    if (followLatestLogRef.current === next) return;
    followLatestLogRef.current = next;
    setFollowLatestLog(next);
  }, []);

  /**
   * 用户向上滚动时自动暂停；滚回底部 32 px 范围内自动恢复。
   * 只观察日志容器本身，不改变整页的滚动位置。
   */
  const handleLogScroll = useCallback(() => {
    const container = logContainerRef.current;
    if (!container) return;
    updateLogFollowState(isLogNearBottom(container));
  }, [updateLogFollowState]);

  /** 点击“回到最新”时恢复跟随，并立即把日志容器移到底部。 */
  const resumeLogFollow = useCallback(() => {
    updateLogFollowState(true);
    const container = logContainerRef.current;
    if (container) container.scrollTop = container.scrollHeight;
  }, [updateLogFollowState]);

  const clearProtocolLogs = useCallback(() => {
    // 清空后恢复默认跟随，下一条日志会自然出现在可见区域。
    followLatestLogRef.current = true;
    setFollowLatestLog(true);
    setLogs([]);
  }, []);

  useEffect(() => {
    if (!followLatestLogRef.current) return;
    const container = logContainerRef.current;
    if (container) container.scrollTop = container.scrollHeight;
  }, [logs]);

  const allocateRequestId = useCallback((): number => {
    for (let checked = 0; checked < 255; checked += 1) {
      const candidate = nextRequestIdRef.current;
      nextRequestIdRef.current = candidate === 0xff ? 1 : candidate + 1;
      if (!pendingRef.current.has(candidate)) return candidate;
    }
    throw new Error("没有可用的 request_id");
  }, []);

  const writeFrame = useCallback(async (
    bytes: Uint8Array<ArrayBuffer>,
  ): Promise<void> => {
    const characteristic = commandRef.current;
    const connectionGeneration = connectionGenerationRef.current;
    if (!connectedRef.current || !characteristic) {
      throw new Error("Product Command 特征尚未连接");
    }
    const copied = Uint8Array.from(bytes);
    const run = writeQueueRef.current
      .catch(() => undefined)
      .then(async () => {
        // 队列中的写入只能落到创建它的 GATT 会话。断线重连后，旧帧必须
        // 失败，不能误写进新的 ESP-EXO 连接。
        if (!connectedRef.current ||
            connectionGenerationRef.current !== connectionGeneration ||
            commandRef.current !== characteristic) {
          throw new Error("BLE 会话已变化，已取消旧会话写入");
        }
        const writePromise = characteristic.writeValueWithResponse
          ? characteristic.writeValueWithResponse(copied.buffer)
          : characteristic.writeValue
            ? characteristic.writeValue(copied.buffer)
            : Promise.reject(new Error("浏览器没有提供 BLE 带响应写入接口"));
        let timeout: ReturnType<typeof setTimeout> | null = null;
        try {
          await Promise.race([
            writePromise,
            new Promise<never>((_resolve, reject) => {
              timeout = setTimeout(
                () => reject(new GattWriteTimeoutError()),
                WRITE_TIMEOUT_MS,
              );
            }),
          ]);
        } finally {
          if (timeout) clearTimeout(timeout);
        }
      });
    writeQueueRef.current = run.then(() => undefined, () => undefined);
    try {
      await run;
    } catch (caught) {
      if (caught instanceof GattWriteTimeoutError &&
          connectionGenerationRef.current === connectionGeneration) {
        // 悬挂的 GATT 写不能安全地与后续 STOP 共用同一连接。使本会话
        // 立即失效并断开，重连后先查系统实际状态，再发送 STOP。
        const device = deviceRef.current;
        appendLog("error", `${caught.message}，当前 BLE 会话已失效。`);
        resetConnection(caught.message);
        if (device?.gatt?.connected) device.gatt.disconnect();
      }
      throw caught;
    }
  }, [appendLog, resetConnection]);

  const transactNow = useCallback((
    opcode: number,
    payload: Uint8Array<ArrayBuffer>,
    label: string,
  ): Promise<ExoBleResponse> => {
    if (!connectedRef.current || !deviceRef.current?.gatt?.connected) {
      // 在分配 request_id/入队写帧之前再次核对真实 GATT，会话已失效时
      // 同步撤销页面的“已连接”和 START 就绪状态。
      handleDisconnected();
      return Promise.reject(new Error("BLE 尚未连接"));
    }
    const requestId = allocateRequestId();
    const bytes = encodeRequest(opcode, requestId, payload);

    return new Promise<ExoBleResponse>((resolve, reject) => {
      const pending: PendingRequest = {
        opcode,
        requestId,
        bytes,
        label,
        attempts: 0,
        timer: null,
        resolve,
        reject,
      };
      pendingRef.current.set(requestId, pending);

      const cleanupWithError = (error: Error) => {
        if (pending.timer) clearTimeout(pending.timer);
        if (pendingRef.current.get(requestId) === pending) {
          pendingRef.current.delete(requestId);
          reject(error);
        }
      };

      const sendAttempt = async () => {
        pending.attempts += 1;
        appendLog(
          "tx",
          `${label}，request_id=${requestId}${pending.attempts > 1 ? "（原帧重试）" : ""}`,
          bytes,
        );
        try {
          await writeFrame(bytes);
        } catch (caught) {
          cleanupWithError(caught instanceof Error ? caught : new Error("BLE 写入失败"));
          return;
        }
        if (pendingRef.current.get(requestId) !== pending) return;
        pending.timer = setTimeout(() => {
          if (pending.attempts < 2) {
            appendLog("system", `${label} 3 秒未收到 Notify，使用相同 request_id 原帧重试一次。`);
            void sendAttempt();
          } else {
            cleanupWithError(new Error(`${label} 等待 Notify 超时`));
          }
        }, RESPONSE_TIMEOUT_MS);
      };

      void sendAttempt();
    });
  }, [allocateRequestId, appendLog, handleDisconnected, writeFrame]);

  const transact = useCallback((
    opcode: number,
    payload: Uint8Array<ArrayBuffer> = new Uint8Array(0),
    label = `opcode 0x${opcode.toString(16)}`,
  ): Promise<ExoBleResponse> => {
    const run = queueRef.current
      .catch(() => undefined)
      .then(() => transactNow(opcode, payload, label));
    queueRef.current = run.then(() => undefined, () => undefined);
    return run;
  }, [transactNow]);

  const queryInfo = useCallback(async (): Promise<InfoResponse> => {
    return responseOfKind(await transact(Opcode.GET_INFO, undefined, "GET_INFO"), "info");
  }, [transact]);

  const querySystem = useCallback(async (): Promise<SystemStatusResponse> => {
    return responseOfKind(
      await transact(Opcode.GET_SYSTEM_STATUS, undefined, "GET_SYSTEM_STATUS"),
      "systemStatus",
    );
  }, [transact]);

  const queryJoint = useCallback(async (
    side: number,
  ): Promise<JointStatusResponse> => {
    const label = side === Side.LEFT ? "GET_JOINT_STATUS LEFT" : "GET_JOINT_STATUS RIGHT";
    const joint = responseOfKind(
      await transact(Opcode.GET_JOINT_STATUS, new Uint8Array([side]), label),
      "jointStatus",
    );
    if (joint.side !== side) {
      throw new Error(
        `${label} 返回了错误侧别 ${joint.side === Side.LEFT ? "LEFT" : "RIGHT"}`,
      );
    }
    return joint;
  }, [transact]);

  const queryCalibration = useCallback(async (side: number): Promise<CalibrationStatusResponse> => {
    const snapshot = responseOfKind(await transact(Opcode.GET_CALIBRATION_STATUS,
      Uint8Array.of(side), `GET_CALIBRATION_STATUS ${side === Side.LEFT ? "LEFT" : "RIGHT"}`),
    "calibrationStatus");
    if (snapshot.side !== side) throw new Error("标定读回侧别不匹配");
    return snapshot;
  }, [transact]);

  const querySavedZero = useCallback(async (side: number): Promise<SavedZeroStatusResponse> => {
    const status = responseOfKind(await transact(Opcode.GET_SAVED_ZERO,
      Uint8Array.of(side), `GET_SAVED_ZERO ${side === Side.LEFT ? "LEFT" : "RIGHT"}`), "savedZeroStatus");
    if (status.side !== side) throw new Error("保存零位读回侧别不匹配");
    return status;
  }, [transact]);

  const queryControlConfig = useCallback(async (): Promise<ControlConfigResponse> => {
    return responseOfKind(
      await transact(
        Opcode.GET_CONTROL_CONFIG,
        undefined,
        "GET_CONTROL_CONFIG",
      ),
      "controlConfig",
    );
  }, [transact]);

  const queryJointCommand = useCallback(async (
    side: number,
  ): Promise<JointCommandResponse> => {
    const label = side === Side.LEFT
      ? "GET_JOINT_COMMAND LEFT"
      : "GET_JOINT_COMMAND RIGHT";
    const command = responseOfKind(
      await transact(
        Opcode.GET_JOINT_COMMAND,
        Uint8Array.of(side),
        label,
      ),
      "jointCommand",
    );
    if (command.side !== side) {
      throw new Error(`${label} 返回了错误侧别`);
    }
    return command;
  }, [transact]);

  const queryGaitStatus = useCallback(async (): Promise<GaitStatusResponse> => {
    return responseOfKind(
      await transact(Opcode.GET_GAIT_STATUS, undefined, "GET_GAIT_STATUS"),
      "gaitStatus",
    );
  }, [transact]);

  const queryAssistTorqueLimit = useCallback(async (): Promise<AssistTorqueLimitResponse> => {
    return responseOfKind(await transact(Opcode.GET_ASSIST_TORQUE_LIMIT,
      undefined, "GET_ASSIST_TORQUE_LIMIT"), "assistTorqueLimit");
  }, [transact]);

  const queryAngleAssistConfig = useCallback(async (): Promise<AngleAssistConfigResponse> => {
    return responseOfKind(await transact(Opcode.GET_ANGLE_ASSIST_CONFIG,
      undefined, "GET_ANGLE_ASSIST_CONFIG"), "angleAssistConfig");
  }, [transact]);

  const queryPulseConfiguration = useCallback(async () => {
    const timing=responseOfKind(await transact(Opcode.GET_PULSE_TIMING,undefined,"GET_PULSE_TIMING"),"pulseTiming");
    const window=responseOfKind(await transact(Opcode.GET_PULSE_WINDOW,undefined,"GET_PULSE_WINDOW"),"pulseWindow");
    return {timing,window};
  },[transact]);
  const queryPulseStatus = useCallback(async () => {
    for(const side of [Side.LEFT,Side.RIGHT])
      await transact(Opcode.GET_PULSE_STATUS,new Uint8Array([side]),"GET_PULSE_STATUS");
  },[transact]);
  const queryAssistConfig = useCallback(async (): Promise<AssistConfigResponse> => {
    const config = responseOfKind(
      await transact(
        Opcode.GET_ASSIST_CONFIG,
        undefined,
        "GET_ASSIST_CONFIG",
      ),
      "assistConfig",
    );
    if (((infoRef.current?.capabilities ?? 0) & Capability.ASSIST_TORQUE_LIMIT) !== 0) {
      await queryAssistTorqueLimit();
    }
    if (((infoRef.current?.capabilities ?? 0) & Capability.ANGLE_THRESHOLD_ASSIST) !== 0) {
      const angle = await queryAngleAssistConfig();
      if (angle.configGeneration !== config.configGeneration)
        throw new Error("助力与角度配置代次不一致，请刷新状态");
    }
    if(((infoRef.current?.capabilities ?? 0)&Capability.SWING_EVENT_PULSE)!==0) {
      await queryPulseConfiguration();await queryPulseStatus();
    }
    return config;
  }, [transact, queryAssistTorqueLimit, queryAngleAssistConfig, queryPulseConfiguration, queryPulseStatus]);

  const queryTelemetryStatus = useCallback(async (): Promise<TelemetryStatusResponse> => {
    return responseOfKind(
      await transact(
        Opcode.GET_TELEMETRY_STATUS,
        undefined,
        "GET_TELEMETRY_STATUS",
      ),
      "telemetryStatus",
    );
  }, [transact]);

  const queryCanStatus = useCallback(async (): Promise<CanStatusResponse> => {
    return responseOfKind(await transact(Opcode.GET_CAN_STATUS, undefined, "GET_CAN_STATUS"), "canStatus");
  }, [transact]);

  const restoreFeedback = useCallback(async (): Promise<void> => {
    const state = await querySystem();
    const health = await queryCanStatus();
    if (state.state !== 1 || !health.restoreAllowed) throw new Error("仅待机且CAN未进入错误被动/BUS_OFF时可恢复反馈。");
    const ack = responseOfKind(await transact(Opcode.RESTORE_FEEDBACK, undefined, "RESTORE_FEEDBACK"), "ack");
    if (ack.result !== 1) throw new Error("恢复请求被拒绝；两次手动请求需间隔10秒，请读取最新CAN状态。");
    setFeedbackRestoreNotice("已提交有限恢复请求，等待电机新反馈；请求接收不代表已恢复。反馈回来后仍需重新校准穿戴零位。");
    await queryCanStatus();
  }, [queryCanStatus, querySystem, transact]);

  const refreshAllCore = useCallback(async (): Promise<void> => {
    const identity = await queryInfo();
    await querySystem();
    if ((identity.capabilities & Capability.FEEDBACK_RECOVERY) !== 0) await queryCanStatus();
    await queryJoint(Side.LEFT);
    await queryJoint(Side.RIGHT);
    if ((identity.capabilities & Capability.CALIBRATION_STATUS) !== 0) {
      await queryCalibration(Side.LEFT);
      await queryCalibration(Side.RIGHT);
    }
    if ((identity.capabilities & Capability.SAVED_WEAR_ZERO) !== 0) {
      const left = await querySavedZero(Side.LEFT), right = await querySavedZero(Side.RIGHT);
      if (!wearZeroChoiceChosenRef.current) setWearZeroChoice(left.stored && right.stored ? "reuse" : "new");
    }
    if ((identity.capabilities & Capability.CONTROL_CONFIGURATION) !== 0) {
      await queryControlConfig();
      await queryJointCommand(Side.LEFT);
      await queryJointCommand(Side.RIGHT);
    }
    if ((identity.capabilities & Capability.GAIT_STATUS) !== 0) {
      await queryGaitStatus();
    }
    if ((identity.capabilities & Capability.ASSIST_CONFIGURATION) !== 0) {
      await queryAssistConfig();
    }
    if ((identity.capabilities & Capability.TELEMETRY_STREAM) !== 0) {
      await queryTelemetryStatus();
    }
  }, [
    queryCanStatus,
    queryCalibration,
    querySavedZero,
    queryAssistConfig,
    queryControlConfig,
    queryGaitStatus,
    queryInfo,
    queryJoint,
    queryJointCommand,
    querySystem,
    queryTelemetryStatus,
  ]);

  const runOperation = useCallback(async (label: string, action: () => Promise<void>) => {
    if (operationRef.current) return;
    operationRef.current = label;
    setOperation(label);
    setError("");
    try {
      await action();
    } catch (caught) {
      if (caught instanceof StopPreemptionError) {
        appendLog("system", "当前操作的 BLE 等待已取消，STOP 正在接管生命周期。");
        return;
      }
      const message = caught instanceof Error ? caught.message : `${label}失败`;
      setError(message);
      appendLog("error", `${label}失败：${message}`);
    } finally {
      // STOP 可以在 STARTING 期间接管操作锁。旧操作结束时不能误清除
      // 新操作的状态。
      if (operationRef.current === label) {
        operationRef.current = "";
        setOperation("");
      }
    }
  }, [appendLog]);

  /**
   * 使用一个已经由用户授权的 BluetoothDevice 完成完整产品握手。
   * stage 是局部变量，因为 React 状态更新是异步的；catch 必须准确知道
   * 本次到底失败在哪一步，不能读取可能仍落后一步的 connectStage。
   */
  async function establishConnection(device: BluetoothDeviceLike) {
    const attempt = ++connectAttemptRef.current;
    let stage: ConnectStage = "gatt";
    const waitStage = async <T,>(pending: Promise<T>): Promise<T> => {
      const value = await withGattStageTimeout(pending, connectStageLabels[stage], stage === "handshake" ? 30000 : 15000);
      if (attempt !== connectAttemptRef.current) throw new Error("连接尝试已取消或已被新会话替代");
      return value;
    };
    let subscribedResponse: BluetoothRemoteGATTCharacteristicLike | null = null;
    let subscribedTelemetry: BluetoothRemoteGATTCharacteristicLike | null = null;

    setConnection("connecting");
    setConnectStage(stage);
    setLastFailedConnectStage(null);
    setError("");

    if (!device.gatt) {
      const message = "所选设备没有 GATT 接口";
      setConnection("idle");
      setConnectStage("idle");
      setLastFailedConnectStage(stage);
      setError(message);
      appendLog("error", message);
      return;
    }

    sessionIdRef.current = typeof crypto.randomUUID === "function"
      ? crypto.randomUUID()
      : `${Date.now()}-${Math.random().toString(16).slice(2)}`;
    deviceIdRef.current = device.id ?? "unknown";
    deviceRef.current = device;
    setDeviceName(device.name ?? EXO_BLE.DEVICE_NAME);

    // 重连同一个设备前先移除旧监听器，再添加一次，保证一个会话只有一个
    // gattserverdisconnected 回调。
    device.removeEventListener("gattserverdisconnected", handleDisconnected);
    device.addEventListener("gattserverdisconnected", handleDisconnected);

    try {
      appendLog("system", "正在建立 GATT 连接…");
      const server = await waitStage(connectGattWithOneRetry(
        () => device.gatt!.connect(),
        {
          delayMs: 800,
          onRetry: (firstError) => {
            appendLog(
              "system",
              `第一次 GATT 连接失败，800 ms 后使用同一设备再试一次：${errorMessageOf(firstError)}`,
            );
          },
        },
      ));
      serverRef.current = server;

      stage = "service";
      setConnectStage(stage);
      appendLog("system", "GATT 已连接，正在发现 Product Service…");
      const service = await waitStage(server.getPrimaryService(EXO_BLE.SERVICE_UUID));

      stage = "characteristics";
      setConnectStage(stage);
      appendLog("system", "Product Service 已发现，正在读取命令、响应与可选遥测特征…");
      const [command, response, telemetry] = await waitStage(Promise.all([
        service.getCharacteristic(EXO_BLE.COMMAND_CHARACTERISTIC_UUID),
        service.getCharacteristic(EXO_BLE.RESPONSE_CHARACTERISTIC_UUID),
        // 0.7～0.9 固件没有独立遥测特征。找不到它不是连接失败，页面会保留
        // 原来的串行轮询；0.10 capability 会在握手后进一步校验。
        service.getCharacteristic(EXO_BLE.TELEMETRY_CHARACTERISTIC_UUID)
          .catch(() => null),
      ]));

      connectionGenerationRef.current += 1;
      commandRef.current = command;
      responseRef.current = response;
      telemetryRef.current = telemetry;
      response.addEventListener("characteristicvaluechanged", handleNotification);
      subscribedResponse = response;

      stage = "notify";
      setConnectStage(stage);
      appendLog("system", "产品特征已就绪，正在订阅 Response Notify…");
      await waitStage(response.startNotifications());
      if (telemetry) {
        telemetry.addEventListener(
          "characteristicvaluechanged",
          handleTelemetryNotification,
        );
        subscribedTelemetry = telemetry;
        await waitStage(telemetry.startNotifications());
        setTelemetryCharacteristicReady(true);
        appendLog("system", "Telemetry Notify 已订阅；需点击“开始遥测”后设备才会自动上报。");
      } else {
        setTelemetryCharacteristicReady(false);
        appendLog("system", "设备未提供独立 Telemetry 特征，将保留阶段9低频轮询。");
      }

      // transact() 只有在 connectedRef 为 true 时才允许发送请求；但页面此时
      // 仍保持“连接中”，直到 INFO、系统状态和双侧关节全部读取成功。
      nextRequestIdRef.current = 1;
      connectedRef.current = true;
      stage = "handshake";
      setConnectStage(stage);
      appendLog("system", "Notify 已订阅，正在读取设备身份与初始状态…");

      operationRef.current = "读取初始状态";
      setOperation("读取初始状态");
      try {
        await waitStage(refreshAllCore());
      } finally {
        if (operationRef.current === "读取初始状态") {
          operationRef.current = "";
          setOperation("");
        }
      }

      stage = "ready";
      setConnectStage(stage);
      setConnection("connected");
      setAutoPolling(true);
      appendLog("system", "ESP-EXO 已连接，身份与初始状态读取完成，可以执行产品命令。");
    } catch (caught) {
      if (attempt !== connectAttemptRef.current) return;
      if (subscribedResponse) {
        subscribedResponse.removeEventListener(
          "characteristicvaluechanged",
          handleNotification,
        );
        try {
          void subscribedResponse.stopNotifications?.().catch(() => undefined);
        } catch {
          // 建连失败时连接可能已经消失，继续释放 GATT 即可。
        }
      }
      if (subscribedTelemetry) {
        subscribedTelemetry.removeEventListener(
          "characteristicvaluechanged",
          handleTelemetryNotification,
        );
        try {
          void subscribedTelemetry.stopNotifications?.().catch(() => undefined);
        } catch {
          // 同上：建连失败时特征句柄可能已失效，继续释放 GATT。
        }
      }

      device.removeEventListener("gattserverdisconnected", handleDisconnected);
      if (device.gatt.connected) device.gatt.disconnect();

      // resetConnection 只清活动会话，不会清 rememberedDeviceRef。因此失败后
      // 页面仍能直接重试同一个设备，无需再次打开系统选择框。
      resetConnection("BLE 连接失败");
      setLastFailedConnectStage(stage);
      const message = describeConnectFailure(stage, caught);
      setError(message);
      appendLog("error", `${connectStageLabels[stage]}失败：${errorMessageOf(caught)}`);
    }
  }

  /** 打开系统设备选择框；选中后把设备授权保留到本页关闭为止。 */
  async function connectBluetooth() {
    const chooserAttempt = ++connectAttemptRef.current;
    const bluetooth = (navigator as BluetoothNavigator).bluetooth;
    if (!bluetooth) return;

    setConnection("connecting");
    setConnectStage("chooser");
    setLastFailedConnectStage(null);
    setError("");
    appendLog("system", "正在选择 ESP-EXO；请先单击设备行使其高亮，再点击“配对”。");

    try {
      const device = await bluetooth.requestDevice({
        // 固件广播中已经带有完整 Product Service UUID。名称与服务同时过滤，
        // 可以避免误选同名设备，并让浏览器授予该服务的访问权限。
        filters: [{
          name: EXO_BLE.DEVICE_NAME,
          services: [EXO_BLE.SERVICE_UUID],
        }],
        optionalServices: [EXO_BLE.SERVICE_UUID],
      });
      if (chooserAttempt !== connectAttemptRef.current) return;
      rememberedDeviceRef.current = device;
      setHasRememberedDevice(true);
      setDeviceName(device.name ?? EXO_BLE.DEVICE_NAME);
      await establishConnection(device);
    } catch (caught) {
      if (chooserAttempt !== connectAttemptRef.current) return;
      resetConnection("已取消设备选择");
      const message = errorMessageOf(caught);
      if (message.toLowerCase().includes("cancel") ||
          (caught instanceof DOMException && caught.name === "NotFoundError")) {
        appendLog("system", "已取消设备选择；没有发送任何命令。");
      } else {
        setLastFailedConnectStage("chooser");
        setError(`设备选择未完成。${message ? `（浏览器信息：${message}）` : ""}`);
        appendLog("error", `设备选择失败：${message}`);
      }
    }
  }

  /** 不再弹选择框，直接复用本页中最近一次选中的 ESP-EXO。 */
  async function retryRememberedDevice() {
    const rememberedDevice = rememberedDeviceRef.current;
    if (!rememberedDevice) {
      setError("当前页面还没有记住设备，请先选择 ESP-EXO。");
      return;
    }
    appendLog("system", `正在重试已选设备 ${rememberedDevice.name ?? EXO_BLE.DEVICE_NAME}…`);
    await establishConnection(rememberedDevice);
  }

  function cancelConnectionAttempt() {
    const device = deviceRef.current;
    device?.removeEventListener("gattserverdisconnected", handleDisconnected);
    device?.gatt?.disconnect();
    resetConnection("已取消蓝牙握手");
    appendLog("system", "已取消连接；没有发送START。已授权设备仍可重试。");
  }

  async function disconnectBluetooth() {
    const latestSystem = systemRef.current;
    const lifecycleActive = Boolean(
      latestSystem && (
        [0x02, 0x03, 0x04].includes(latestSystem.state) ||
        latestSystem.decodedSystemFlags.runRequested ||
        latestSystem.decodedSystemFlags.outputAllowed
      ),
    ) || operationRef.current.includes("START") ||
      operationRef.current.includes("STOP");
    if (lifecycleActive) {
      const confirmed = window.confirm(
        "BLE 断开不会自动 STOP。系统正在启停或仍可能输出，确定要直接断开吗？",
      );
      if (!confirmed) return;
    }
    setAutoPolling(false);
    try {
      const response = responseRef.current;
      if (response) {
        response.removeEventListener("characteristicvaluechanged", handleNotification);
        await response.stopNotifications?.();
      }
    } catch {
      // 连接可能已经消失；仍继续释放本地状态。
    }
    const device = deviceRef.current;
    if (device) {
      device.removeEventListener("gattserverdisconnected", handleDisconnected);
      if (device.gatt?.connected) device.gatt.disconnect();
    }
    resetConnection("用户主动断开 BLE");
    appendLog("system", "BLE 已主动断开。");
  }

  async function sendDirectionTeaching(side: number, action: number, token: number): Promise<AckResponse> {
    if (((infoRef.current?.capabilities ?? 0) & Capability.DIRECTION_TEACHING) === 0)
      throw new Error("当前固件不支持方向标定，请升级至0.10.4");
    const current = await querySystem();
    if (current.state !== 0x01 || current.decodedSystemFlags.outputAllowed)
      throw new Error("方向标定必须在主控待机且输出门关闭时进行");
    const ack = responseOfKind(await transact(Opcode.TEACH_DIRECTION,
      new Uint8Array([side, action, token]),
      `TEACH_DIRECTION ${side === Side.LEFT ? "LEFT" : "RIGHT"} action=${action}`), "ack");
    if (ack.result !== 0) {
      if ([1, 2, 4, 12].includes(ack.detail)) setDirectionTeaching(null);
      const detail = ack.result === 0x10 ? directionTeachFailureDescriptions[ack.detail] : undefined;
      throw new Error(detail ?? `方向标定被拒绝：${ack.resultName}`);
    }
    return ack;
  }

  async function beginDirectionTeaching(side: number): Promise<void> {
    const token = nextDirectionTokenRef.current;
    nextDirectionTokenRef.current = token % 255 + 1;
    setNeutralConfirmed(false);
    setLastWearCalibrationAt(null);
    setDirectionTeachNotice("");
    if (side === Side.LEFT) setLeftMappingConfirmed(false);
    else setRightMappingConfirmed(false);
    // 重连可能丢失网页临时会话；取消只撤销旧等待，不恢复零位、不改符号。
    await sendDirectionTeaching(side, DirectionTeachAction.CANCEL, token);
    setDirectionTeaching(null);
    await sendDirectionTeaching(side, DirectionTeachAction.BEGIN, token);
    setDirectionTeaching({side, token});
    await queryCalibration(Side.LEFT); await queryCalibration(Side.RIGHT);
    await queryJoint(Side.LEFT); await queryJoint(Side.RIGHT);
    setDirectionTeachNotice(`已记录${side === Side.LEFT ? "左" : "右"}腿基准。请在60秒内手动向前抬这一条腿约5～15°，另一腿保持原位；停稳后点击“确认前抬方向”。`);
  }

  async function finishDirectionTeaching(): Promise<void> {
    if (!directionTeaching) throw new Error("请先记录静止基准");
    const {side, token} = directionTeaching;
    const ack = await sendDirectionTeaching(side, DirectionTeachAction.FINISH, token);
    setDirectionTeaching(null);
    const verified = await queryCalibration(side);
    await queryJoint(side); await querySystem();
    if (![1, 2].includes(ack.detail) || verified.direction !== ack.detail ||
        !verified.decodedJointFlags.directionValid || verified.decodedJointFlags.zeroValid)
      throw new Error("方向写入后读回不一致，请重新标定");
    if (side === Side.LEFT) setLeftMappingConfirmed(true);
    else setRightMappingConfirmed(true);
    setNeutralConfirmed(false);
    setDirectionTeachNotice(`${side === Side.LEFT ? "左" : "右"}腿前抬正方向已保存为出厂配置并读回确认，断电后保留。请把腿放回中立位；双侧方向完成后再校准穿戴零位。`);
  }

  async function cancelDirectionTeaching(): Promise<void> {
    if (!directionTeaching) return;
    await sendDirectionTeaching(directionTeaching.side, DirectionTeachAction.CANCEL, directionTeaching.token);
    setDirectionTeaching(null);
    setNeutralConfirmed(false);
    setDirectionTeachNotice("已取消本次方向标定，未覆盖原出厂记录。本次标定侧方向与旧零位保持未就绪，请重新记录基准并完成标定。");
  }

  async function captureZero(side: number): Promise<void> {
    if (directionTeaching) throw new Error("请先完成或取消方向标定，再回中立位校准");
    if (!neutralConfirmed) throw new Error("请先确认双髋已处于中立位");
    if (((infoRef.current?.capabilities ?? 0) & Capability.CALIBRATION_STATUS) !== 0 &&
        (!leftMappingConfirmed || !rightMappingConfirmed)) {
      throw new Error("请先完成本次装配的左右腿电机与方向核对");
    }
    setLastWearCalibrationAt(null);
    const sideName = side === Side.LEFT ? "LEFT" : "RIGHT";
    const before = side === Side.LEFT ? leftJoint : rightJoint;
    let ack: AckResponse;
    try {
      ack = responseOfKind(
        await transact(
          Opcode.CAPTURE_ZERO,
          new Uint8Array([side]),
          `CAPTURE_ZERO ${sideName}`,
        ),
        "ack",
      );
    } catch (caught) {
      if (caught instanceof StopPreemptionError) throw caught;
      if (!connectedRef.current) throw caught;

      appendLog("system", `${sideName} 归零响应不确定，正在核对关节零位。`);
      const after = await queryJoint(side);
      if (before?.decodedJointFlags.zeroValid === false &&
          after.decodedJointFlags.zeroValid &&
          after.decodedJointFlags.jointValid) {
        appendLog("system", `${sideName} 已通过关节快照确认归零生效。`);
        if (((infoRef.current?.capabilities ?? 0) & Capability.CALIBRATION_STATUS) !== 0)
          await queryCalibration(side);
        await querySystem();
        return;
      }
      throw new Error(
        `${sideName} 归零结果未知；请保持中立位，不要盲目重复归零，并结合 UART 核对零位来源。`,
      );
    }
    if (ack.result !== 0x00) {
      throw new Error(`${sideName} 归零被拒绝：${ack.resultName}`);
    }
    const verified = await queryJoint(side);
    if (!verified.decodedJointFlags.zeroValid ||
        !verified.decodedJointFlags.jointValid) {
      throw new Error(`${sideName} ACK 成功，但关节零位状态未生效`);
    }
    if (((infoRef.current?.capabilities ?? 0) & Capability.CALIBRATION_STATUS) !== 0)
      await queryCalibration(side);
    await querySystem();
  }

  async function captureBothZeros(): Promise<void> {
    if (!supportsAtomicZeroCapture(infoRef.current))
      throw new Error("当前固件不支持一次采集双髋零位，请升级0.10.11；不会逐侧写入。");
    if (directionTeaching || !neutralConfirmed || !leftMappingConfirmed || !rightMappingConfirmed)
      throw new Error("请完成左右出厂方向核对，确认双髋中立且静止，再校准。");
    setLastWearCalibrationAt(null);
    setWearZeroNotice("正在采集当前双髋位置并核对两侧0°读回，请保持静止。");
    try {
      await captureAndVerifyBothZeros({
        capturePair: () => savedZeroAction(SavedZeroAction.CAPTURE_PAIR),
        queryJoint, queryCalibration, querySystem,
      });
    } catch (caught) {
      setNeutralConfirmed(false);
      setWearZeroNotice("本次双髋校准未确认；请核对两侧真实读数。不会自动重试或强制显示0°。");
      throw caught;
    }
    setWearZeroNotice("已将当前左右髋位置同时设为中立位，双侧人体角均通过0°附近读回核对。");
    if (((infoRef.current?.capabilities ?? 0) & Capability.SAVED_WEAR_ZERO) !== 0 && saveWearZeroForReuse) {
      try {
        await savedZeroAction(SavedZeroAction.SAVE_CURRENT);
        const left = await querySavedZero(Side.LEFT), right = await querySavedZero(Side.RIGHT);
        if (!left.stored || !right.stored || !left.bindingMatches || !right.bindingMatches)
          throw new Error("候选记录未读回");
        setWearZeroNotice("本次零位已重新校准，并保存到主控，断电后可选择沿用。");
      } catch (caught) {
        setNeutralConfirmed(false);
        setWearZeroNotice("本次RAM零位已校准，但保存结果未确认；不能认定新零位已断电保留。");
        throw caught;
      }
    }
    setLastWearCalibrationAt(new Date().toLocaleTimeString("zh-CN", {hour12: false, timeZone: "Asia/Shanghai"}));
    // 写入确认只消费本次中立姿态；再次校准前必须重新确认穿戴与静止状态。
    setNeutralConfirmed(false);
  }

  async function clearBothZeros(): Promise<void> {
    const confirmed = window.confirm("确定清除左右髋的 RAM 软件零位吗？清除后不能 START。");
    if (!confirmed) return;
    setLastWearCalibrationAt(null);
    for (const side of [Side.LEFT, Side.RIGHT]) {
      const sideName = side === Side.LEFT ? "LEFT" : "RIGHT";
      let ack: AckResponse | null = null;
      try {
        ack = responseOfKind(
          await transact(
            Opcode.CLEAR_ZERO,
            new Uint8Array([side]),
            `CLEAR_ZERO ${sideName}`,
          ),
          "ack",
        );
      } catch (caught) {
        if (caught instanceof StopPreemptionError) throw caught;
        if (!connectedRef.current) throw caught;
        appendLog("system", `${sideName} 清零响应不确定，正在核对关节状态。`);
      }
      if (ack && ack.result !== 0x00) {
        throw new Error(`${sideName} 清零失败：${ack.resultName}`);
      }
      const verified = await queryJoint(side);
      if (verified.decodedJointFlags.zeroValid) {
        throw new Error(`${sideName} 清零结果未知，零位标志仍然有效`);
      }
    }
    setNeutralConfirmed(false);
    await queryJoint(Side.LEFT);
    await queryJoint(Side.RIGHT);
    await querySystem();
  }

  async function savedZeroAction(action: number): Promise<void> {
    if (((infoRef.current?.capabilities ?? 0) & Capability.SAVED_WEAR_ZERO) === 0)
      throw new Error("当前固件不支持保存/沿用零位，需要0.10.9及保存零位能力。");
    if (action === SavedZeroAction.CAPTURE_PAIR && !supportsAtomicZeroCapture(infoRef.current))
      throw new Error("一次采集双髋零位需要0.10.11固件。");
    const state = await querySystem();
    if (state.state !== 1) throw new Error("零位候选操作只允许待机执行。");
    const ack = responseOfKind(await transact(Opcode.SAVED_ZERO_ACTION,
      Uint8Array.of(action, 1), `SAVED_ZERO_ACTION ${action}`), "ack");
    if (ack.result !== 0) throw new Error(savedZeroReasonDescriptions[ack.detail] ?? `零位操作被拒绝：${ack.resultName}`);
  }

  async function reuseSavedZeros(): Promise<void> {
    if (!neutralConfirmed) throw new Error("请确认本次穿戴完成、双髋中立且静止。");
    const left = await querySavedZero(Side.LEFT), right = await querySavedZero(Side.RIGHT);
    if (!canReuseSavedWearZero(left, right))
      throw new Error(savedZeroReasonDescriptions[left.reason || right.reason] ?? "上次零位不适用，请重新校准。");
    await savedZeroAction(SavedZeroAction.RESTORE);
    const currentLeft = await queryJoint(Side.LEFT), currentRight = await queryJoint(Side.RIGHT);
    await queryCalibration(Side.LEFT); await queryCalibration(Side.RIGHT);
    const state = await querySystem();
    if (state.state !== 1 || ![currentLeft, currentRight].every((j) =>
      j.decodedJointFlags.zeroValid && j.decodedJointFlags.jointValid && Math.abs(j.positionDegrees) <= 5.1))
      throw new Error("沿用后的双髋零位未通过读回，请保持中立静止并重新校准。");
    setWearZeroNotice("已沿用上次零位并核对本次双髋读回；没有启动电机。");
    setLastWearCalibrationAt(new Date().toLocaleTimeString("zh-CN", {hour12:false,timeZone:"Asia/Shanghai"}));
    setNeutralConfirmed(false);
  }

  async function forgetSavedZeros(): Promise<void> {
    if (!window.confirm("忘记主控保存的上次零位？本次RAM零位不变，今后需要重新校准并保存。")) return;
    await savedZeroAction(SavedZeroAction.FORGET);
    const left = await querySavedZero(Side.LEFT), right = await querySavedZero(Side.RIGHT);
    if (left.stored || right.stored) throw new Error("保存记录尚未清除，请刷新检查。");
    wearZeroChoiceChosenRef.current = true;
    setWearZeroChoice("new"); setNeutralConfirmed(false);
    setWearZeroNotice("已忘记上次零位；本次RAM零位保持原状。");
  }

  function requireSuccessfulConfigAck(ack: AckResponse, label: string): void {
    if (ack.result !== 0x00) {
      throw new Error(`${label}被拒绝：${ack.resultName}`);
    }
  }

  async function refreshAssistAfterLifecycle(label: string): Promise<void> {
    if (((infoRef.current?.capabilities ?? 0) &
        Capability.ASSIST_CONFIGURATION) === 0) return;
    try {
      await queryAssistConfig();
    } catch (caught) {
      if (caught instanceof StopPreemptionError) throw caught;
      // 系统已经到达 RUNNING/IDLE 终态，随后一次可选状态读回失败不能把
      // 生命周期结果改写成“启动/停止失败”。保留旧快照并提示手动刷新。
      appendLog(
        "error",
        `${label}后读取助力活动状态失败：${errorMessageOf(caught)}；请点击“刷新全部状态”。`,
      );
    }
  }

  async function writeControlConfig(
    mode = selectedMode,
    reference = selectedReference,
  ): Promise<ControlConfigResponse> {
    const payload = encodeControlConfigPayload(mode, reference);
    const ack = responseOfKind(
      await transact(
        Opcode.SET_CONTROL_CONFIG,
        payload,
        "SET_CONTROL_CONFIG",
      ),
      "ack",
    );
    requireSuccessfulConfigAck(ack, "模式/参考源设置");
    const verified = await queryControlConfig();
    if (!verified.selectedValid || verified.selectedMode !== mode ||
        verified.selectedReference !== reference) {
      throw new Error("模式/参考源 ACK 成功，但设备读回值不一致");
    }
    return verified;
  }

  async function writeJointCommand(
    side: number,
    draft: JointCommandDraft,
  ): Promise<JointCommandResponse> {
    const payload = encodeJointCommandPayload(jointCommandFromDraft(side, draft));
    const sideName = side === Side.LEFT ? "LEFT" : "RIGHT";
    const ack = responseOfKind(
      await transact(
        Opcode.SET_JOINT_COMMAND,
        payload,
        `SET_JOINT_COMMAND ${sideName}`,
      ),
      "ack",
    );
    requireSuccessfulConfigAck(ack, `${sideName} 五元命令设置`);
    const verified = await queryJointCommand(side);
    if (toHex(verified.raw.slice(EXO_BLE.HEADER_SIZE)) !== toHex(payload)) {
      throw new Error(`${sideName} 五元命令 ACK 成功，但设备读回值不一致`);
    }
    return verified;
  }

  function requestedPulse() {
    if(Object.values(pulseDraft).some(value => String(value).trim()==="")) throw new Error("请填写脉冲参数");
    const timing: PulseTimingValues={durationMode:pulseDraft.durationMode,peakNm:Number(pulseDraft.peakNm),
      fixedMs:Number(pulseDraft.fixedMs),ratio:Number(pulseDraft.ratio),minimumMs:Number(pulseDraft.minimumMs),
      maximumMs:Number(pulseDraft.maximumMs),periodMinimumMs:Number(pulseDraft.periodMinimumMs),periodMaximumMs:Number(pulseDraft.periodMaximumMs)};
    const reset=Number(pulseDraft.resetDegrees),maximum=Number(pulseDraft.maximumDegrees),trigger=Number(angleThresholdDraft);
    encodePulseTimingPayload(timing);encodePulseWindowPayload(reset,maximum);
    if(reset>trigger-1 || maximum<trigger+2) throw new Error("复位角至少低于触发角1°；允许上限至少高于触发角2°");
    return {timing,reset,maximum};
  }
  async function writePulseConfiguration(request: ReturnType<typeof requestedPulse>) {
    if(((infoRef.current?.capabilities ?? 0)&Capability.SWING_EVENT_PULSE)===0)
      throw new Error("单次事件脉冲需要0.10.10固件，草稿未写入");
    assertPulseTimingCapability(request.timing, infoRef.current?.capabilities ?? 0, infoRef.current?.firmware);
    const state=await querySystem();
    if(state.state!==1 && state.state!==3) throw new Error("仅待机或运行中可更新脉冲参数");
    const limit=await queryAssistTorqueLimit(), angle=await queryAngleAssistConfig();
    const ceiling=state.state===3?limit.activeMaximumNm:limit.selectedMaximumNm;
    if(ceiling==null || request.timing.peakNm>ceiling) throw new Error("申请峰值不能超过当前锁存力矩上限；提高上限需先STOP");
    const trigger=angle.selectedThresholdDegrees;
    if(trigger==null || request.reset>trigger-1 || request.maximum<trigger+2)
      throw new Error("窗口与主控触发阈值不一致，请先保存基础助力设置");
    const timingPayload=encodePulseTimingPayload(request.timing);
    const windowPayload=encodePulseWindowPayload(request.reset,request.maximum);
    requireSuccessfulConfigAck(responseOfKind(await transact(Opcode.SET_PULSE_TIMING,timingPayload,"SET_PULSE_TIMING"),"ack"),"下一脉冲参数");
    requireSuccessfulConfigAck(responseOfKind(await transact(Opcode.SET_PULSE_WINDOW,windowPayload,"SET_PULSE_WINDOW"),"ack"),"复位与角度窗口");
    const verified=await queryPulseConfiguration();
    if(toHex(encodePulseTimingPayload(verified.timing))!==toHex(timingPayload) ||
      toHex(encodePulseWindowPayload(verified.window.resetDegrees,verified.window.maximumDegrees))!==toHex(windowPayload))
      throw new Error("脉冲参数读回不一致，请刷新；当前脉冲仍使用原冻结A/Ta");
    await queryPulseStatus();
  }

  async function writeAssistConfig(
    draft: AssistConfigDraft = assistConfigDraft,
  ): Promise<AssistConfigResponse> {
    if (systemRef.current?.state !== 0x01) {
      throw new Error("助力配置只能在 IDLE 写入；请先 STOP 并等待 IDLE");
    }
    if (draft.executionState === AssistExecutionState.ACTIVE) {
      const identity = infoRef.current;
      if (!identity ||
          (identity.capabilities & Capability.TELEMETRY_STREAM) === 0) {
        throw new Error("当前设备不是阶段10固件；网页不会向旧固件发送 ACTIVE 请求");
      }
      const phase = draft.selectedMode === AssistMode.ACTIVE_PHASE && Number(draft.strategyId) === 2;
      const threshold = draft.selectedMode === AssistMode.ANGLE_THRESHOLD && Number(draft.strategyId) === 4 &&
        (identity.capabilities & Capability.ANGLE_THRESHOLD_ASSIST) !== 0;
      if (!phase && !threshold) throw new Error("ACTIVE 只允许相位曲线或设备支持的角度阈值策略");
      if (!controlConfig?.selectedValid ||
          controlConfig.selectedMode !== ControlMode.DAMPING ||
          controlConfig.selectedReference !== ReferenceSource.CONSTANT) {
        throw new Error(
          "ACTIVE 目前只允许叠加在 DAMPING + CONSTANT；请先应用并读回基础控制配置",
        );
      }
    }
    const requested = {
      selectedMode: draft.selectedMode,
      requestedExecutionState: draft.executionState,
      strategyId: integerDraftNumber(
        draft.strategyId,
        "策略 ID",
      ),
      profileId: integerDraftNumber(
        draft.profileId,
        "曲线 ID",
      ),
      leftLevelPercent: integerDraftNumber(
        draft.leftLevelPercent,
        "LEFT 助力等级",
        0,
        100,
      ),
      rightLevelPercent: integerDraftNumber(
        draft.rightLevelPercent,
        "RIGHT 助力等级",
        0,
        100,
      ),
    };
    const supportsLimit = Boolean((infoRef.current?.capabilities ?? 0) & Capability.ASSIST_TORQUE_LIMIT);
    if (supportsLimit && !assistTorqueLimit?.selectedValid) throw new Error("尚未读回最大附加助力配置，请先刷新全部状态");
    const maximumNm = Number(maximumAssistDraft);
    if (supportsLimit && maximumAssistDraft.trim() === "") throw new Error("请输入最大附加助力力矩");
    const threshold = requested.strategyId === AssistStrategy.ANGLE_THRESHOLD;
    if (threshold && ((infoRef.current?.capabilities ?? 0) & Capability.SWING_EVENT_PULSE) === 0)
      throw new Error("事件脉冲需要0.10.10固件；旧版持续阈值策略不能当作脉冲使用");
    if (supportsLimit) {
      // 保存前读取当前设备上限，避免重连/更换设备后沿用网页旧快照。
      const currentLimit = await queryAssistTorqueLimit();
      const allowedNm = assistTorqueMaximumNm(requested.strategyId,
        infoRef.current?.capabilities ?? 0, currentLimit.hardMaximumNm, infoRef.current?.firmware);
      if (!Number.isFinite(maximumNm) || maximumNm < 0 || maximumNm > allowedNm)
        throw new Error(threshold && maximumNm > 0.5 && allowedNm <= 0.5
          ? "5 N·m角度助力需要0.10.8固件，当前草稿尚未写入设备"
          : `当前策略与设备的助力峰值范围为0–${allowedNm.toFixed(3)} N·m`);
    }
    if (threshold && angleThresholdDraft.trim() === "") throw new Error("请输入前抬角度阈值");
    const thresholdDegrees = Number(angleThresholdDraft);
    const payload = threshold ? encodeAngleAssistConfigPayload(requested, maximumNm, thresholdDegrees)
      : supportsLimit ? encodeAssistConfigWithLimitPayload(requested, maximumNm) : encodeAssistConfigPayload(requested);
    const opcode = threshold ? Opcode.SET_ANGLE_ASSIST_CONFIG
      : supportsLimit ? Opcode.SET_ASSIST_CONFIG_WITH_LIMIT : Opcode.SET_ASSIST_CONFIG;
    const ack = responseOfKind(
      await transact(
        opcode, payload,
        threshold ? "SET_ANGLE_ASSIST_CONFIG" : supportsLimit ? "SET_ASSIST_CONFIG_WITH_LIMIT" : "SET_ASSIST_CONFIG",
      ),
      "ack",
    );
    requireSuccessfulConfigAck(ack, "助力策略设置");

    // ACK 只说明命令处理结果。必须立即 GET 读回，页面上的“已选择/本轮活动”
    // 永远以设备实际保存值为准。
    const verified = await queryAssistConfig();
    if (!verified.decodedAssistFlags.selectedValid ||
        verified.selectedMode !== requested.selectedMode ||
        verified.requestedExecutionState !== requested.requestedExecutionState ||
        verified.strategyId !== requested.strategyId ||
        verified.profileId !== requested.profileId ||
        verified.leftLevelPercent !== requested.leftLevelPercent ||
        verified.rightLevelPercent !== requested.rightLevelPercent) {
      throw new Error("助力配置 ACK 成功，但设备读回值不一致");
    }
    if (supportsLimit) {
      const limit = await queryAssistTorqueLimit();
      if (!limit.selectedValid || limit.selectedMaximumNm !== maximumNm)
        throw new Error("最大附加助力 ACK 成功，但设备读回值不一致；请刷新状态");
      setMaximumAssistDraft(maximumNm.toFixed(3));
    }
    if (threshold) {
      const angle = await queryAngleAssistConfig();
      if (!angle.selectedValid || angle.selectedThresholdDegrees !== thresholdDegrees ||
          angle.configGeneration !== verified.configGeneration)
        throw new Error("角度阈值ACK成功但读回不一致，当前不能确认配置完成");
      setAngleThresholdDraft(thresholdDegrees.toFixed(2));
    }
    setAssistConfigDraft(assistDraftFromResponse(verified));
    return verified;
  }

  /**
   * 工程激励只为打通“编码器反馈 → 步态估相 → SHADOW 建议 → 遥测”链路。
   * 它始终请求 SHADOW，绝不请求 ACTIVE，也复用与普通页面相同的串行事务函数。
   */
  async function prepareEngineeringTestStimulus(): Promise<void> {
    const identity = infoRef.current;
    if (!identity ||
        (identity.capabilities & Capability.ENGINEERING_TEST_STIMULUS) === 0) {
      throw new Error("当前固件没有声明 ENGINEERING_TEST_STIMULUS capability");
    }
    if (systemRef.current?.state !== 0x01) {
      appendLog("system", "准备工程验证激励前先执行 STOP 并等待 IDLE。");
      await stopExoskeleton();
    }
    const idle = await querySystem();
    if (idle.state !== 0x01) throw new Error("系统未进入 IDLE，不能准备工程验证激励");

    await writeControlConfig(
      ControlMode.IMPEDANCE,
      ReferenceSource.ALTERNATING_SINE_TEST,
    );
    await writeJointCommand(Side.LEFT, ENGINEERING_TEST_JOINT_COMMAND_DRAFT);
    await writeJointCommand(Side.RIGHT, ENGINEERING_TEST_JOINT_COMMAND_DRAFT);
    await writeAssistConfig({
      selectedMode: AssistMode.ACTIVE_PHASE,
      executionState: AssistExecutionState.SHADOW,
      strategyId: String(AssistStrategy.PHASE_PROFILE),
      profileId: "1",
      leftLevelPercent: "10",
      rightLevelPercent: "10",
    });
    appendLog(
      "system",
      "工程验证已准备并逐项读回：IMPEDANCE + ALTERNATING_SINE_TEST，左右 Kp=5.0/Kd=0.1，ACTIVE_PHASE + SHADOW。此配置不会启用真实助力；请另行 START。",
    );
  }

  /** 恢复明确的产品待机配置，并清除工程 SHADOW 请求。 */
  async function restoreProductControlConfiguration(): Promise<void> {
    if (systemRef.current?.state !== 0x01) {
      appendLog("system", "恢复产品配置前先执行 STOP 并等待 IDLE。");
      await stopExoskeleton();
    }
    const idle = await querySystem();
    if (idle.state !== 0x01) throw new Error("系统未进入 IDLE，不能恢复产品配置");

    await writeControlConfig(ControlMode.DAMPING, ReferenceSource.CONSTANT);
    await writeJointCommand(Side.LEFT, PRODUCT_DAMPING_JOINT_COMMAND_DRAFT);
    await writeJointCommand(Side.RIGHT, PRODUCT_DAMPING_JOINT_COMMAND_DRAFT);
    await writeAssistConfig({
      selectedMode: AssistMode.TRANSPARENT,
      executionState: AssistExecutionState.DISABLED,
      strategyId: String(AssistStrategy.TRANSPARENT),
      profileId: "0",
      leftLevelPercent: "0",
      rightLevelPercent: "0",
    });
    setSelectedMode(ControlMode.DAMPING);
    setSelectedReference(ReferenceSource.CONSTANT);
    appendLog(
      "system",
      "已逐项读回产品待机配置：DAMPING + CONSTANT，左右 Kp=0/Kd=0.1，TRANSPARENT + DISABLED。",
    );
  }

  const startTelemetry = useCallback(async (): Promise<void> => {
    if (!infoRef.current ||
        (infoRef.current.capabilities & Capability.TELEMETRY_STREAM) === 0) {
      throw new Error("当前固件没有声明 TELEMETRY_STREAM capability");
    }
    if (!telemetryRef.current || !telemetryCharacteristicReady) {
      throw new Error("设备声明了遥测能力，但独立 Telemetry Notify 特征不可用");
    }
    if (!assessDeviceCompatibility(infoRef.current).accepted)
      throw new Error("当前设备协议或固件身份未通过兼容性核对，不能开启遥测");

    const payload = encodeTelemetryStartPayload({
      rateHz: telemetryRateHz,
      channelMask: telemetryChannelMask,
    });
    // 先重置组装统计，首帧可能早于ACK到达，不能在ACK之后再把它删掉。
    telemetryAccumulatorRef.current.reset();
    telemetryLastReceivedRef.current=null;
    setLastTelemetryReceivedMs(null);
    telemetryAutoAttemptRef.current=connectionGenerationRef.current;
    const ack = responseOfKind(
      await transact(
        Opcode.START_TELEMETRY,
        payload,
        "START_TELEMETRY",
      ),
      "ack",
    );
    requireSuccessfulConfigAck(ack, "开始遥测");
    const verified = await queryTelemetryStatus();
    if (!verified.streaming || verified.rateHz !== telemetryRateHz ||
        verified.channelMask !== telemetryChannelMask) {
      throw new Error("START_TELEMETRY ACK 成功，但设备有效遥测配置与请求不一致");
    }

    setTelemetryStats(telemetryAccumulatorRef.current.snapshotStats());
    telemetryDisplayPausedRef.current=false;
    setTelemetryDisplayPaused(false);
    setTelemetrySamples(telemetryCaptureRef.current.slice(-SCOPE_BUFFER_SAMPLES));
    setAutoPolling(false);
    appendLog(
      "system",
      `遥测流已启动：${verified.rateHz} Hz，mask=${formatHexMask(verified.channelMask)}；已暂停4 Hz轮询以释放BLE带宽。`,
    );
  }, [appendLog, queryTelemetryStatus, telemetryChannelMask, telemetryCharacteristicReady, telemetryRateHz, transact]);

  // 只自动启动数据流：每个新BLE会话最多尝试一次；失败留给明确的重试按钮。
  // 用户停止采集会关闭自动采集，不能由状态轮询反复重新打开。
  useEffect(() => {
    if (!shouldStartScope({livePanel:activePanel==="live",connected:connection==="connected",
      ready:telemetryCharacteristicReady,compatible:assessDeviceCompatibility(info).accepted,
      automatic:telemetryAutoStart,streaming:telemetryStatus?.streaming ?? null,busy:Boolean(operation),
      session:connectionGenerationRef.current,attempted:telemetryAutoAttemptRef.current})) return;
    const timer=window.setTimeout(()=>{
      telemetryAutoAttemptRef.current=connectionGenerationRef.current;
      void runOperation("自动开始实时采集",startTelemetry);
    },0);
    return ()=>window.clearTimeout(timer);
  },[activePanel,connection,info,operation,runOperation,startTelemetry,telemetryAutoStart,telemetryCharacteristicReady,telemetryStatus]);

  async function stopTelemetry(): Promise<void> {
    setTelemetryAutoStart(false);
    const ack = responseOfKind(
      await transact(
        Opcode.STOP_TELEMETRY,
        undefined,
        "STOP_TELEMETRY",
      ),
      "ack",
    );
    requireSuccessfulConfigAck(ack, "停止遥测");
    const verified = await queryTelemetryStatus();
    if (verified.streaming) {
      throw new Error("STOP_TELEMETRY ACK 成功，但设备仍报告正在推流");
    }
    appendLog("system", "遥测流已停止；这不会向外骨骼发送 STOP。");
  }

  function toggleTelemetryChannel(channel: number): void {
    setTelemetryChannelMask((current) => {
      const next = current ^ channel;
      // 协议禁止空掩码；至少保留一个通道，避免按钮点击后到发送时才报错。
      return next === 0 ? current : next;
    });
  }

  function setTelemetryDisplayPause(paused: boolean): void {
    telemetryDisplayPausedRef.current = paused;
    setTelemetryDisplayPaused(paused);
    if (!paused) {
      setTelemetrySamples(telemetryCaptureRef.current.slice(-SCOPE_BUFFER_SAMPLES));
    }
  }

  function clearTelemetrySamples(): void {
    telemetryCaptureRef.current = [];
    telemetryAccumulatorRef.current.reset();
    setTelemetrySamples([]);
    setTelemetryStats({ ...EMPTY_TELEMETRY_STATS });
    telemetryDirtyRef.current=false;
    telemetryLastReceivedRef.current=null;
    setLastTelemetryReceivedMs(null);
  }

  function exportTelemetryCsv(): void {
    downloadText(
      `esp-exo-telemetry-${new Date().toISOString().replaceAll(":", "-")}.csv`,
      telemetrySamplesToCsv(telemetryCaptureRef.current),
      "text/csv;charset=utf-8",
    );
  }

  function exportTelemetryJsonl(): void {
    downloadText(
      `esp-exo-telemetry-${new Date().toISOString().replaceAll(":", "-")}.jsonl`,
      telemetrySamplesToJsonl(telemetryCaptureRef.current),
      "application/x-ndjson;charset=utf-8",
    );
  }

  async function applyAllControlConfiguration(): Promise<void> {
    if (systemRef.current?.state !== 0x01) {
      appendLog("system", "配置前系统不是 IDLE，先执行 STOP 并等待 IDLE。");
      await stopExoskeleton();
    }
    const idle = await querySystem();
    if (idle.state !== 0x01) throw new Error("系统未进入 IDLE，不能写控制配置");
    await writeControlConfig();
    await writeJointCommand(Side.LEFT, leftCommandDraft);
    await writeJointCommand(Side.RIGHT, rightCommandDraft);
    appendLog("system", "模式、参考源和左右五元命令均已写入并逐项读回一致；可单独按 START。 ");
  }

  async function pollSystemUntil(
    target: "running" | "idle",
    startGeneration?: number,
  ): Promise<SystemStatusResponse> {
    const deadline = performance.now() + TERMINAL_TIMEOUT_MS;
    while (performance.now() < deadline) {
      if (target === "running" &&
          startGeneration !== startWaitGenerationRef.current) {
        throw new StopPreemptionError();
      }
      const snapshot = await querySystem();
      if (target === "running" &&
          startGeneration !== startWaitGenerationRef.current) {
        throw new StopPreemptionError();
      }
      if (snapshot.state === 0x05) {
        throw new Error(`系统进入 ERROR，失败步骤 ${snapshot.lastFailedStepName}`);
      }
      if (target === "running" && snapshot.state === 0x03 &&
          snapshot.decodedSystemFlags.outputAllowed) {
        return snapshot;
      }
      if (target === "idle" && snapshot.state === 0x01 &&
          !snapshot.decodedSystemFlags.runRequested &&
          !snapshot.decodedSystemFlags.outputAllowed) {
        return snapshot;
      }
      if (target === "running" && snapshot.state === 0x01) {
        throw new Error(`START 未到 RUNNING 就回到 IDLE，失败步骤 ${snapshot.lastFailedStepName}`);
      }
      await delay(200);
    }
    throw new Error(`${target === "running" ? "START" : "STOP"} 在 ${TERMINAL_TIMEOUT_MS / 1000} 秒内未到终态`);
  }

  async function startExoskeleton(): Promise<void> {
    if(((infoRef.current?.capabilities ?? 0)&Capability.ASSIST_CONFIGURATION)!==0) {
      const saved=await queryAssistConfig();
      if(saved.strategyId===AssistStrategy.ANGLE_THRESHOLD &&
          ((infoRef.current?.capabilities ?? 0)&Capability.SWING_EVENT_PULSE)===0)
        throw new Error("设备仍是旧持续阈值策略；升级0.10.10并读回事件脉冲参数后才能START");
    }
    if (((infoRef.current?.capabilities ?? 0) & Capability.CALIBRATION_STATUS) !== 0 &&
        (!leftMappingConfirmed || !rightMappingConfirmed)) {
      throw new Error("本次连接尚未完成左右腿电机核对，不能START");
    }
    // 穿戴入口在START前再次读回，不能把网页草稿或上次台架配置当成当前合同。
    if (humanWornMode) {
      const base = await queryControlConfig();
      const left = await queryJointCommand(Side.LEFT);
      const right = await queryJointCommand(Side.RIGHT);
      if (base.selectedMode !== ControlMode.DAMPING ||
          base.selectedReference !== ReferenceSource.CONSTANT ||
          ![left, right].every((command) => command.positionDegrees === 0 &&
            command.velocityDegreesPerSecond === 0 && command.torqueNm === 0 &&
            command.kp === 0 && command.kd <= 0.1)) {
        throw new Error("人体穿戴只允许DAMPING + CONSTANT、零位置增益/前馈、Kd≤0.1；请恢复产品配置");
      }
    }
    // 发出 START 后人体姿态会改变，旧的“中立位确认”立即失效。
    setNeutralConfirmed(false);
    const startGeneration = startWaitGenerationRef.current + 1;
    startWaitGenerationRef.current = startGeneration;
    let ack: AckResponse | null = null;
    try {
      ack = responseOfKind(
        await transact(Opcode.START, undefined, "START"),
        "ack",
      );
    } catch (caught) {
      if (caught instanceof StopPreemptionError) throw caught;

      // 写入成功但 ACK/Notify 丢失时，设备可能已经开始执行 START。
      // 先用新 request_id 查询实际状态，不能仅凭网页超时宣告失败。
      appendLog("system", "START 响应不确定，正在查询设备实际状态。");
      const snapshot = await querySystem();
      const startMayBeActive = [0x02, 0x03].includes(snapshot.state) ||
        snapshot.decodedSystemFlags.runRequested ||
        snapshot.decodedSystemFlags.outputAllowed;
      if (!startMayBeActive) throw caught;
    }
    if (ack && ![0x01, 0x02, 0x03].includes(ack.result)) {
      const block = ack.result === 0x10
        ? startBlockReasonDisplay(ack.detail)
        : ack.resultName;
      throw new Error(`START 被拒绝：${block}`);
    }
    await pollSystemUntil("running", startGeneration);
    await refreshAssistAfterLifecycle("START 到 RUNNING");
    const nextRunId = runIdRef.current + 1;
    runIdRef.current = nextRunId;
    setCurrentRunId(nextRunId);
    appendLog("system", `第 ${nextRunId} 轮已进入 RUNNING。`);
  }

  async function stopExoskeleton(): Promise<void> {
    let ack: AckResponse | null = null;
    let snapshot: SystemStatusResponse | null = null;
    try {
      ack = responseOfKind(
        await transact(Opcode.STOP, undefined, "STOP"),
        "ack",
      );
    } catch (caught) {
      if (!connectedRef.current) throw caught;
      appendLog("system", "STOP 响应不确定，正在查询设备实际状态。");
      snapshot = await querySystem();
      const alreadyIdle = snapshot.state === 0x01 &&
        !snapshot.decodedSystemFlags.runRequested &&
        !snapshot.decodedSystemFlags.outputAllowed;
      if (!alreadyIdle && snapshot.state !== 0x04) {
        // 第一个 STOP 的两次 Notify 都可能丢失。若设备仍未进入 STOPPING，
        // 用新的 request_id 再提交一次幂等 STOP。
        ack = responseOfKind(
          await transact(Opcode.STOP, undefined, "STOP 状态对账重提"),
          "ack",
        );
      }
    }
    if (ack && ![0x01, 0x02, 0x03].includes(ack.result)) {
      throw new Error(`STOP 被拒绝：${ack.resultName}`);
    }
    const alreadyIdle = snapshot?.state === 0x01 &&
      !snapshot.decodedSystemFlags.runRequested &&
      !snapshot.decodedSystemFlags.outputAllowed;
    if (!alreadyIdle) {
      await pollSystemUntil("idle");
    }
    if (runIdRef.current > 0) {
      setCompletedRuns((count) => Math.max(count, runIdRef.current));
    }
    appendLog("system", "STOP 已完成，系统回到 IDLE。", ack?.raw);
    await queryJoint(Side.LEFT);
    await queryJoint(Side.RIGHT);
    await refreshAssistAfterLifecycle("STOP 到 IDLE");
  }

  async function runStopOperation(): Promise<void> {
    const label = "STOP 到 IDLE";
    if (operationRef.current === label) return;

    // STOP 是生命周期取消命令，必须允许在 START 到 RUNNING 的等待期间接管。
    // 先取消网页端尚在等待的 Notify，使 STOP 不必被一个 6 秒超时挡在队尾。
    startWaitGenerationRef.current += 1;
    cancelPendingForStop();
    operationRef.current = label;
    setOperation(label);
    setError("");
    try {
      await stopExoskeleton();
      setError("");
    } catch (caught) {
      const message = caught instanceof Error ? caught.message : `${label}失败`;
      // 即使STOP落到ERROR，也读回已关闭的策略门，避免还显示上次ACTIVE状态。
      await refreshAssistAfterLifecycle("STOP失败");
      setError(message);
      appendLog("error", `${label}失败：${message}`);
    } finally {
      if (operationRef.current === label) {
        operationRef.current = "";
        setOperation("");
      }
    }
  }

  const pollOnce = useCallback(async (): Promise<void> => {
    if (!connectedRef.current || pollBusyRef.current || operationRef.current) return;
    pollBusyRef.current = true;
    try {
      await querySystem();
      await queryJoint(Side.LEFT);
      await queryJoint(Side.RIGHT);
      // 标定是低频只读显示：IDLE每秒更新一次，不在100 Hz控制链中运行。
      if (systemRef.current?.state === 0x01 &&
          ((infoRef.current?.capabilities ?? 0) & Capability.CALIBRATION_STATUS) !== 0 &&
          performance.now() - calibrationPollMsRef.current >= 1000) {
        calibrationPollMsRef.current = performance.now();
        await queryCalibration(Side.LEFT);
        await queryCalibration(Side.RIGHT);
        if (((infoRef.current?.capabilities ?? 0) & Capability.SAVED_WEAR_ZERO) !== 0) {
          await querySavedZero(Side.LEFT); await querySavedZero(Side.RIGHT);
        }
      }
      if (((infoRef.current?.capabilities ?? 0) & Capability.GAIT_STATUS) !== 0) {
        await queryGaitStatus();
      }
      if (((infoRef.current?.capabilities ?? 0) & Capability.FEEDBACK_RECOVERY) !== 0 &&
          performance.now() - canPollMsRef.current >= 2000) {
        canPollMsRef.current = performance.now();
        await queryCanStatus();
      }
      // 门禁和有效执行态每秒读一次，保持串行事务，不参与100 Hz控制。
      if (((infoRef.current?.capabilities ?? 0) & Capability.ASSIST_CONFIGURATION) !== 0 &&
          performance.now() - assistPollMsRef.current >= 1000) {
        assistPollMsRef.current = performance.now();
        await queryAssistConfig();
      }
    } catch (caught) {
      if (caught instanceof StopPreemptionError) return;
      const message = caught instanceof Error ? caught.message : "自动轮询失败";
      appendLog("error", `自动轮询暂停：${message}`);
      setError(message);
      setAutoPolling(false);
    } finally {
      pollBusyRef.current = false;
    }
  }, [appendLog, queryCanStatus, queryAssistConfig, queryCalibration, querySavedZero, queryGaitStatus, queryJoint, querySystem]);

  useEffect(() => {
    if (connection !== "connected" || !autoPolling) return;
    let cancelled = false;
    let timer: ReturnType<typeof setTimeout> | null = null;
    const tick = async () => {
      if (cancelled) return;
      const startedAtMs = Date.now();
      await pollOnce();
      // 250 ms 是“轮询起点到下一轮起点”的目标周期；不能在一轮 BLE
      // 往返结束后再固定等待 250 ms，否则四个串行查询会把步态刷新率拖到
      // 明显低于 4 Hz。若 BLE 本轮已超时，则仍保持严格串行而不并发补帧。
      const remainingMs = Math.max(0, POLL_INTERVAL_MS - (Date.now() - startedAtMs));
      if (!cancelled) timer = setTimeout(tick, remainingMs);
    };
    void tick();
    return () => {
      cancelled = true;
      if (timer) clearTimeout(timer);
    };
  }, [autoPolling, connection, pollOnce]);

  // 设备丢样本计数和队列深度只存在于 GET_TELEMETRY_STATUS。推流期间每2秒
  // 低频读取一次，不把每个高速 Notify 写进事务队列，也不与4 Hz备用轮询并发。
  useEffect(() => {
    if (connection !== "connected" || !telemetryStatus?.streaming) return;
    let cancelled = false;
    const timer = window.setInterval(() => {
      if (cancelled || telemetryStatusBusyRef.current || operationRef.current) return;
      telemetryStatusBusyRef.current = true;
      void (async () => {
        await queryTelemetryStatus();
        await querySystem();
        if (((infoRef.current?.capabilities ?? 0) & Capability.ASSIST_CONFIGURATION) !== 0)
          await queryAssistConfig();
      })()
        .catch((caught) => {
          appendLog("error", `遥测状态低频刷新失败：${errorMessageOf(caught)}`);
        })
        .finally(() => {
          telemetryStatusBusyRef.current = false;
        });
    }, 2000);
    return () => {
      cancelled = true;
      window.clearInterval(timer);
    };
  }, [appendLog, connection, queryAssistConfig, querySystem, queryTelemetryStatus, telemetryStatus?.streaming]);

  const connected = connection === "connected";
  const systemFlags = system?.decodedSystemFlags;
  const compatibility = useMemo(() => assessDeviceCompatibility(info), [info]);
  const infoIssues = compatibility.blockingIssues;
  const infoWarnings = compatibility.warnings;
  const stage7InfoAccepted = info !== null && compatibility.accepted;
  const hasControlConfiguration = Boolean(
    info &&
    (info.capabilities & Capability.CONTROL_CONFIGURATION) !== 0,
  );
  const hasGaitStatus = Boolean(
    info && (info.capabilities & Capability.GAIT_STATUS) !== 0,
  );
  const hasAssistConfiguration = Boolean(
    info && (info.capabilities & Capability.ASSIST_CONFIGURATION) !== 0,
  );
  const hasPulseAssist = Boolean(info && (info.capabilities & Capability.SWING_EVENT_PULSE) !== 0);
  const hasExtendedPulse = hasExtendedPulseLimits(info?.capabilities ?? 0, info?.firmware);
  const pulseRiseLimit = info ? pulseRiseLimitNmPerSecond(info.capabilities, info.firmware) : 20;
  const fixedPulseMaximumMs = !info || hasExtendedPulse ? 3000 : 2000;
  const pulsePeakMaximumNm = !info ? 15 : assistTorqueMaximumNm(AssistStrategy.ANGLE_THRESHOLD,
    info.capabilities, assistTorqueLimit?.hardMaximumNm ?? null, info.firmware);
  const angleStrategyDraft = Number(assistConfigDraft.strategyId) === AssistStrategy.ANGLE_THRESHOLD;
  const hasAssistTorqueLimit = Boolean(info && (info.capabilities & Capability.ASSIST_TORQUE_LIMIT) !== 0);
  // 旧0.10.x的工程profile=1峰值固定0.5；升级前显示固定值，不发送新命令。
  const savedMaximumNm = hasAssistTorqueLimit ? assistTorqueLimit?.selectedMaximumNm ?? null
    : assistConfig?.profileId === 1 ? 0.5 : null;
  const maximumHardNm = !info && angleStrategyDraft ? 15 : assistTorqueMaximumNm(
    Number(assistConfigDraft.strategyId), info?.capabilities ?? 0,
    assistTorqueLimit?.hardMaximumNm ?? null, info?.firmware);
  const draftMaximumNm = hasAssistTorqueLimit || angleStrategyDraft ? Number(maximumAssistDraft) : savedMaximumNm;
  const maximumDraftAllowed = (!hasAssistTorqueLimit && !angleStrategyDraft) ||
    (draftMaximumNm !== null && maximumAssistDraft.trim() !== "" &&
     Number.isFinite(draftMaximumNm) && draftMaximumNm >= 0 && draftMaximumNm <= maximumHardNm);
  const hasTelemetryStream = Boolean(
    info && (info.capabilities & Capability.TELEMETRY_STREAM) !== 0,
  );
  const hasEngineeringTestStimulus = Boolean(
    info && (info.capabilities & Capability.ENGINEERING_TEST_STIMULUS) !== 0,
  );
  const canEditControlConfiguration = connected && stage7InfoAccepted &&
    hasControlConfiguration && !operation && system?.state === 0x01;
  const canApplyControlConfiguration = connected && stage7InfoAccepted &&
    hasControlConfiguration && !operation;
  const canConfigureEngineeringTest = connected && stage7InfoAccepted &&
    hasControlConfiguration && hasAssistConfiguration &&
    hasEngineeringTestStimulus && !operation;
  const canEditAssistConfiguration = connected && stage7InfoAccepted &&
    hasAssistConfiguration && !operation && system?.state === 0x01;
  const canStartTelemetry = connected && stage7InfoAccepted && hasTelemetryStream &&
    telemetryCharacteristicReady && !operation && !telemetryStatus?.streaming;
  const canStopTelemetry = connected && hasTelemetryStream && !operation &&
    Boolean(telemetryStatus?.streaming);
  const activePhaseBaseCompatible = Boolean(
    controlConfig?.selectedValid &&
    controlConfig.selectedMode === ControlMode.DAMPING &&
    controlConfig.selectedReference === ReferenceSource.CONSTANT,
  );
  const engineeringTestPrepared = Boolean(
    controlConfig?.selectedValid &&
    controlConfig.selectedMode === ControlMode.IMPEDANCE &&
    controlConfig.selectedReference === ReferenceSource.ALTERNATING_SINE_TEST &&
    assistConfig?.decodedAssistFlags.selectedValid &&
    assistConfig.selectedMode === AssistMode.ACTIVE_PHASE &&
    assistConfig.requestedExecutionState === AssistExecutionState.SHADOW,
  );
  const assistActiveDraftAllowed = assistConfigDraft.executionState !==
    AssistExecutionState.ACTIVE || (
      hasTelemetryStream &&
      ((assistConfigDraft.selectedMode === AssistMode.ACTIVE_PHASE && Number(assistConfigDraft.strategyId) === 2) ||
       (hasPulseAssist && assistConfigDraft.selectedMode === AssistMode.ANGLE_THRESHOLD && angleStrategyDraft)) &&
      activePhaseBaseCompatible
    );
  const selectedTelemetryChannelCount = countMaskBits(telemetryChannelMask);
  const estimatedNotifyRate = telemetryRateHz * selectedTelemetryChannelCount;
  const latestTelemetry = telemetrySamples.at(-1) ?? null;
  const scopeReception = scopeReceptionState(connected,Boolean(telemetryStatus?.streaming),lastTelemetryReceivedMs,scopeClockMs);
  const gaitFlags = gaitStatus?.decodedGaitFlags;
  const gaitQuality = gaitStatus?.decodedQualityFlags;
  const assistFlags = assistConfig?.decodedAssistFlags;
  const leftFeedbackReady = Boolean(
    leftJoint?.decodedJointFlags.feedbackValid &&
    leftJoint.decodedJointFlags.online,
  );
  const rightFeedbackReady = Boolean(
    rightJoint?.decodedJointFlags.feedbackValid &&
    rightJoint.decodedJointFlags.online,
  );
  const bothFeedbackReady = leftFeedbackReady && rightFeedbackReady;
  const bothJointsValid = Boolean(
    leftJoint?.decodedJointFlags.jointValid &&
    rightJoint?.decodedJointFlags.jointValid,
  );
  const hasCalibrationStatus = Boolean(info && (info.capabilities & Capability.CALIBRATION_STATUS));
  const hasDirectionTeaching = Boolean(info && (info.capabilities & Capability.DIRECTION_TEACHING));
  const hasSavedWearZero = Boolean(info && (info.capabilities & Capability.SAVED_WEAR_ZERO));
  const hasAtomicZeroCapture = supportsAtomicZeroCapture(info);
  const savedPairExists = Boolean(leftSavedZero?.stored && rightSavedZero?.stored);
  const savedPairReuseReady = canReuseSavedWearZero(leftSavedZero, rightSavedZero);
  const canTeachDirection = connected && stage7InfoAccepted && hasDirectionTeaching &&
    !operation && system?.state === 0x01 && bothFeedbackReady &&
    [leftCalibration, rightCalibration].every((snapshot) => snapshot !== null &&
      snapshot.rawPositionDegrees !== null && snapshot.feedbackAgeMs <= 50);
  const factoryMappingConfirmed = !hasCalibrationStatus ||
    (leftMappingConfirmed && rightMappingConfirmed);
  const humanBaseCompatible = activePhaseBaseCompatible && [leftCommand, rightCommand]
    .every((command) => command !== null && command.positionDegrees === 0 &&
      command.velocityDegreesPerSecond === 0 && command.torqueNm === 0 &&
      command.kp === 0 && command.kd <= 0.1);
  const canConfirmNeutral = connected && stage7InfoAccepted && !operation &&
    !directionTeaching && system?.state === 0x01 && bothFeedbackReady && factoryMappingConfirmed;
  const canCaptureLeftZero = canConfirmNeutral && neutralConfirmed &&
    leftFeedbackReady;
  const canCaptureRightZero = canConfirmNeutral && neutralConfirmed &&
    rightFeedbackReady;
  const canCaptureBothZeros = canCaptureLeftZero && canCaptureRightZero && hasAtomicZeroCapture;
  const canReuseBothZeros = canCaptureLeftZero && canCaptureRightZero && hasSavedWearZero && savedPairReuseReady;
  const canCheckLeftMapping = connected && !operation && system?.state === 0x01 &&
    Boolean(leftCalibration?.decodedJointFlags.mappingValid && leftCalibration.decodedJointFlags.directionValid &&
      leftCalibration.rawPositionDegrees !== null && leftCalibration.feedbackAgeMs <= 50);
  const canCheckRightMapping = connected && !operation && system?.state === 0x01 &&
    Boolean(rightCalibration?.decodedJointFlags.mappingValid && rightCalibration.decodedJointFlags.directionValid &&
      rightCalibration.rawPositionDegrees !== null && rightCalibration.feedbackAgeMs <= 50);
  const bothZerosReadBack = bothJointsValid && Boolean(leftJoint?.decodedJointFlags.zeroValid && rightJoint?.decodedJointFlags.zeroValid);
  const wearCalibrationBlockText = !connected ? "先在连接栏目连接外骨骼。" : !stage7InfoAccepted
    ? "设备身份未通过，当前不能校准。" : system?.state === 0x05
      ? `主控故障，校准已锁定。失败步骤：${system.lastFailedStepName}。${!bothFeedbackReady ? "左右电机反馈未就绪，不能写入零位。" : "需要先排除故障并恢复待机。"} STOP未能确认失能时，先切断电机动力电源。` : system?.state !== 0x01
      ? "请先停止外骨骼并等待主控进入待机，再进行零位校准。" : operation
        ? "正在执行操作，请等待完成。" : directionTeaching
          ? "先完成前抬正方向标定，再把双腿放回中立位。" : !bothFeedbackReady
          ? "等待左右髋有效反馈，不能用缺失读数校准。" : !factoryMappingConfirmed
            ? hasDirectionTeaching ? "先在出厂配置中保存左右前抬正方向，再确认穿戴中立位。" : "先分别核对左腿、右腿对应与屈曲方向，再勾选穿戴中立位确认。" : !hasAtomicZeroCapture
              ? "重新校准需要升级0.10.11，才能一次采集并更新左右当前零位；已有沿用功能仍可使用。" : !neutralConfirmed
              ? "站稳保持双髋中立位，勾选穿戴确认后即可写入零位。" : "可以校准：将当前双髋位置写为人体角度0°。";
  const legacyThresholdSelected=assistConfig?.strategyId===AssistStrategy.ANGLE_THRESHOLD && !hasPulseAssist;
  const canStart = connected && !operation && system?.state === 0x01 && !legacyThresholdSelected &&
    stage7InfoAccepted && Boolean(systemFlags?.readyToStart) && bothJointsValid && factoryMappingConfirmed &&
    (!humanWornMode || humanBaseCompatible);
  // STOP 在任何已连接状态都可用。这样即使 START 的 ACK 和状态 Notify
  // 同时丢失，操作者仍有明确的停止入口。
  const canStop = connected && operation !== "STOP 到 IDLE" &&
    operation !== "读取初始状态";

  const actionReadinessText = !connected
    ? "先连接 ESP-EXO。"
    : !stage7InfoAccepted
      ? `设备身份未通过：${infoIssues.join("；")}。`
      : system?.state !== 0x01
        ? `系统当前为 ${system?.stateName ?? "未知"}，归零和 START 需要 IDLE。`
        : legacyThresholdSelected
          ? "设备仍使用旧持续阈值策略；升级0.10.10并读回事件脉冲参数后才能START。"
        : !bothFeedbackReady
          ? "等待左右电机主动反馈在线；当前不能用无效反馈归零。"
          : !factoryMappingConfirmed
            ? "先完成本次左右腿电机与屈曲方向核对。"
          : humanWornMode && !humanBaseCompatible
            ? "人体穿戴模式需要恢复并读回低阻尼产品配置。"
          : !bothJointsValid
              ? neutralConfirmed ? "可以执行归零；双侧零位都生效后 START 才会解锁。" : "请在设置中核对当前中立位并保存双髋软件零位。"
              : systemFlags?.readyToStart
                ? "归零和关节状态有效，START 已就绪。"
                : `等待系统可启动；当前阻止原因 ${startBlockReasonDisplay(
                  system?.startBlockReason,
                  system?.startBlockReasonName,
                )}。`;

  const currentRunSamples = useMemo(
    () => samples.filter((sample) => sample.runId === currentRunId && sample.runId > 0),
    [currentRunId, samples],
  );
  const runStats = useMemo(() => {
    const span = (side: "LEFT" | "RIGHT") => {
      const values = currentRunSamples
        .filter((sample) => sample.side === side)
        .map((sample) => sample.positionDegrees);
      if (values.length < 2) return null;
      return Math.max(...values) - Math.min(...values);
    };
    return {
      leftCount: currentRunSamples.filter((sample) => sample.side === "LEFT").length,
      rightCount: currentRunSamples.filter((sample) => sample.side === "RIGHT").length,
      leftSpan: span("LEFT"),
      rightSpan: span("RIGHT"),
    };
  }, [currentRunSamples]);

  function exportSamples() {
    const header = [
      "session_id", "device_id", "boot_id", "firmware", "run_id",
      "system_state", "side", "received_iso", "received_monotonic_ms",
      "estimated_feedback_monotonic_ms", "request_id", "joint_flags",
      "direction", "position_mrad", "velocity_mrad_s", "position_deg",
      "velocity_deg_s", "feedback_age_ms",
    ];
    const rows = samples.map((sample) => [
      sample.sessionId,
      sample.deviceId,
      `0x${sample.bootId.toString(16).padStart(8, "0")}`,
      sample.firmware,
      sample.runId,
      sample.systemState,
      sample.side,
      sample.receivedIsoTime,
      sample.receivedMonotonicMs.toFixed(3),
      sample.estimatedFeedbackMonotonicMs.toFixed(3),
      sample.requestId,
      `0x${sample.jointFlags.toString(16).padStart(2, "0")}`,
      sample.direction,
      sample.positionMrad,
      sample.velocityMradS,
      sample.positionDegrees.toFixed(9),
      sample.velocityDegreesPerSecond.toFixed(9),
      sample.feedbackAgeMs,
    ]);
    const csv = [header, ...rows]
      .map((row) => row.map((cell) => JSON.stringify(String(cell))).join(","))
      .join("\n");
    downloadText(`esp-exo-joints-${Date.now()}.csv`, csv, "text/csv;charset=utf-8");
  }

  function exportTransactions() {
    const jsonl = logs.map((entry) => JSON.stringify({
      session_id: entry.sessionId || null,
      device_id: entry.deviceId || null,
      iso_time: entry.isoTime,
      monotonic_ms: entry.monotonicMs,
      direction: entry.direction,
      message: entry.message,
      bytes: entry.bytes ? toHex(entry.bytes) : null,
      length: entry.bytes?.length ?? 0,
    })).join("\n");
    downloadText(`esp-exo-ble-${Date.now()}.jsonl`, jsonl, "application/x-ndjson;charset=utf-8");
  }

  const browserReady = bleSupported === true && secureContext === true;
  const connectionBadgeText = connection === "connected"
    ? `${deviceName} 已连接`
    : connection === "connecting"
      ? connectStageLabels[connectStage]
      : lastFailedConnectStage
        ? `${connectStageLabels[lastFailedConnectStage]}失败`
        : "尚未连接";
  const connectionGuidance = connection === "connected"
    ? "Product Service、Response Notify 与初始状态均已验证。"
    : connection === "connecting"
      ? connectStageLabels[connectStage]
      : lastFailedConnectStage
        ? `上次失败步骤：${connectStageLabels[lastFailedConnectStage]}。设备已记住时可直接重试。`
        : hasRememberedDevice
          ? `已记住 ${deviceName}；可直接重试，不必再次打开设备选择框。`
          : "在系统弹窗中先单击 ESP-EXO 设备行使其高亮，再点击“配对”。";


  return (
    <main className="exo-page exo-console">
      <aside className="exo-sidebar">
        <div className="exo-sidebar-brand"><span className="exo-brand-mark">EXO</span><div><strong>ESP-EXO</strong><small>髋关节外骨骼</small></div></div>
        <div className="exo-sidebar-caption">上位机控制台</div>
        <nav aria-label="上位机栏目">{CONSOLE_PANELS.map((panel, index) => <button key={panel.id}
          id={`exo-nav-${panel.id}`} className={activePanel === panel.id ? "active" : ""}
          aria-current={activePanel === panel.id ? "page" : undefined} onClick={() => setActivePanel(panel.id)}>
          <span className="exo-nav-number">0{index + 1}</span><span>{panel.label}</span><span className="exo-nav-arrow" aria-hidden="true">›</span>
        </button>)}</nav>
        <div className="exo-sidebar-device"><span className={`exo-sidebar-dot ${connected ? "online" : ""}`}/><div><b>{connected ? "设备已连接" : "设备未连接"}</b><small>{info ? `固件 ${info.firmware.join(".")}` : "等待蓝牙连接"}</small></div></div>
        <div className="exo-sidebar-bottom"><span>手机 / 网页负责配置与显示</span><small>ESP32 负责本地实时控制</small></div>
      </aside>
      <div className="exo-main-content">
        <header className="exo-topbar exo-console-topbar"><div><p className="exo-kicker">ESP-EXO / 上位机</p><h1>{currentPanel.label}</h1><p>{currentPanel.description}</p></div>
          <div className="exo-global-controls"><div className={`exo-connection ${connection}`}><span/>{connectionBadgeText}</div>
            <span className={`exo-state-chip state-${system?.state ?? "unknown"}`}>{system ? productSystemStateNames[system.state] ?? system.stateName : "主控状态未读取"}</span>
            <button className="exo-button stop" disabled={!canStop} onClick={() => void runStopOperation()}>停止外骨骼</button></div></header>
        {error && <section className="exo-alert error" role="alert"><strong>操作未完成</strong><span>{error}</span></section>}
      {info && !stage7InfoAccepted && (
        <section className="exo-alert error" role="alert">
          <strong>设备身份未通过产品协议验收。</strong>
          <span>{infoIssues.join("；")}。归零和 START 已锁定，STOP 仍可使用。</span>
        </section>
      )}
      {info && stage7InfoAccepted && infoWarnings.length > 0 && (
        <section className="exo-alert" role="status">
          <strong>兼容固件提示</strong>
          <span>{infoWarnings.join("；")}。该提示不会锁定 BLE v1 功能。</span>
        </section>
      )}
        {(activePanel === "connection" || activePanel === "settings" || activePanel === "live") && <section className="exo-card exo-can-panel" aria-label="电机反馈与CAN诊断">
          <p className="exo-kicker">蓝牙与电机分别核对</p>
          <h2>电机反馈与CAN诊断</h2>
          <p>蓝牙连接只证明网页能访问主控。校准还需要两台电机的新反馈。</p>
          {canStatus ? <>
            <div className="exo-data-grid compact">
              <div><span>CAN状态</span><b>{canStatus.stateName}</b></div>
              <div><span>发送 / 接收错误计数</span><b>{canStatus.txErrorCounter} / {canStatus.rxErrorCounter}</b></div>
              <div><span>历史总线错误</span><b>{canStatus.busErrorCount}</b></div>
              <div><span>较上次读取新增错误</span><b>{canErrorDelta === null ? "等待第二次读取" : canErrorDelta}</b></div>
            </div>
            <p>电机A：{(canStatus.motorFlags[0] & 2) ? "反馈在线" : "缺少新反馈"}；电机B：{(canStatus.motorFlags[1] & 2) ? "反馈在线" : "缺少新反馈"}。</p>
            {canStatus.motorFlags.some((flags) => (flags & 16) !== 0) && <p>{canStatus.restoreAllowed ? "自动恢复预算已用尽。确认供电与接线后，可手动提交下一轮有限请求。" : "有限自动请求已停止，当前CAN状态尚不满足手动恢复条件。"}</p>}
            {!canStatus.usable && <p className="exo-inline-warning">CAN已停止、正在恢复或错误计数已达128；反馈恢复请求被锁定，请检查总线并处理主控故障。</p>}
            <small>错误总数从本次主控启动累计，不单凭历史总数认定当前总线故障；已请求上报也不代表电机在线。</small>
          </> : <><p>{connected ? (info && (info.capabilities & Capability.FEEDBACK_RECOVERY) ? "等待读取CAN诊断。" : "当前固件未提供CAN诊断；升级至0.10.5后可在这里读取总线状态并恢复反馈。") : "连接主控后读取双电机反馈与CAN状态。"}</p>
            {connected && <p>左髋：{leftCalibration?.decodedJointFlags.online ? "反馈在线" : "等待新反馈"}；右髋：{rightCalibration?.decodedJointFlags.online ? "反馈在线" : "等待新反馈"}。</p>}</>}
          <div className="exo-connect-actions">
            <button className="exo-button secondary" disabled={!connected || !stage7InfoAccepted || !(info && (info.capabilities & Capability.FEEDBACK_RECOVERY)) || Boolean(operation)} onClick={() => void runOperation("读取CAN诊断", async () => { await queryCanStatus(); })}>读取CAN诊断</button>
            <button className="exo-button primary" disabled={!connected || !stage7InfoAccepted || !canStatus?.restoreAllowed || system?.state !== 1 || Boolean(operation) || canStatus.motorFlags.every((flags) => (flags & 2) !== 0)} onClick={() => void runOperation("恢复电机反馈", restoreFeedback)}>恢复电机反馈</button>
          </div>
          {feedbackRestoreNotice && <p role="status">{canStatus?.motorFlags.every((flags) => (flags & 2) !== 0) ? "左右电机已收到新反馈；请确认出厂方向，再执行本次穿戴零位校准。" : feedbackRestoreNotice}</p>}
          <p>仅请求状态上报，真实在线后才开放校准；恢复不会使能或启动电机。</p>
        </section>}
        <section className="exo-panel" hidden={activePanel !== "connection"} aria-labelledby="exo-nav-connection">
          <section className="exo-hero exo-card">
        <div>
          <p className="exo-kicker">蓝牙连接</p>
          <h2>连接你的外骨骼</h2>
          <p>
            打开外骨骼电源，在蓝牙弹窗中选择 ESP-EXO。连接后读取主控与左右髋状态，连接操作不会启动电机。
          </p>
        </div>
        <div className="exo-connect-panel">
          <div className="exo-connect-actions">
            {connected ? (
              <button
                className="exo-button secondary"
                disabled={Boolean(operation)}
                onClick={() => void disconnectBluetooth()}
              >
                断开连接
              </button>
            ) : (
              <>
                <button
                  className="exo-button primary"
                  disabled={!browserReady || connection === "connecting"}
                  onClick={() => void connectBluetooth()}
                >
                  {connectStage === "chooser" && connection === "connecting"
                    ? "等待设备选择…"
                    : connection === "connecting" ? "连接中…" : "选择 ESP-EXO"}
                </button>
                {connection === "connecting" && connectStage !== "chooser" && <button className="exo-button secondary" onClick={cancelConnectionAttempt}>取消连接</button>}
                {hasRememberedDevice && (
                  <button
                    className="exo-button secondary"
                    disabled={!browserReady || connection === "connecting"}
                    onClick={() => void retryRememberedDevice()}
                  >
                    重试已选设备
                  </button>
                )}
              </>
            )}
            <button
              className="exo-button ghost"
              disabled={!connected || Boolean(operation)}
              onClick={() => void runOperation("刷新全部状态", refreshAllCore)}
            >
              刷新全部状态
            </button>
          </div>
          <p className="exo-connect-progress" aria-live="polite">
            {connectionGuidance}
          </p>
        </div>
      </section>
          <div className="exo-setup-flow"><div><b>01</b><strong>蓝牙连接</strong><span>读取设备当前状态</span></div><div><b>02</b><strong>穿戴校准</strong><span>核对双腿与中立位</span></div><div><b>03</b><strong>配置助力</strong><span>从观察模式开始</span></div></div>
          <details className="exo-advanced"><summary>设备信息与连接诊断</summary><p className="exo-operation-note">支持固件 {COMPATIBLE_FIRMWARE_VERSIONS.join(" / ")} · 推荐 {PREFERRED_FIRMWARE_VERSION}</p>
            <div className="exo-overview-grid"><article className="exo-card exo-device-card">
          <div className="exo-card-heading">
            <div><p className="exo-kicker">设备身份</p><h2>协议与固件</h2></div>
            <span className={`exo-state-chip ${stage7InfoAccepted ? infoWarnings.length ? "warn" : "good" : info ? "warn" : "muted"}`}>
              {stage7InfoAccepted ? infoWarnings.length ? "兼容" : "验收匹配" : info ? "不匹配" : "等待"}
            </span>
          </div>
          <div className="exo-data-grid">
            <div><span>协议范围</span><b>{info ? `v${info.protocolMinVersion}–v${info.protocolMaxVersion}` : "—"}</b></div>
            <div><span>固件</span><b>{info ? info.firmware.join(".") : "—"}</b></div>
            <div><span>Capabilities</span><b>{info ? `0x${info.capabilities.toString(16).padStart(4, "0").toUpperCase()}` : "—"}</b></div>
            <div><span>最大帧长</span><b>{info ? `${info.maxFrameSize} B` : "—"}</b></div>
            <div className="wide"><span>Boot ID</span><b>{formatBootId(info?.bootId)}</b></div>
          </div>
        </article> <article className="exo-card exo-system-card">
          <div className="exo-card-heading">
            <div><p className="exo-kicker">整机生命周期</p><h2>系统状态</h2></div>
            <span className={`exo-state-chip state-${system?.state ?? "unknown"}`}>{system?.stateName ?? "未知"}</span>
          </div>
          <div className="exo-data-grid">
            <div><span>活动步骤</span><b>{system?.activeStepName ?? "—"}</b></div>
            <div><span>START 阻止</span><b>{system
              ? startBlockReasonDisplay(system.startBlockReason, system.startBlockReasonName)
              : "—"}</b></div>
            <div><span>最近结果</span><b>{system?.lastRequestResultName ?? "—"}</b></div>
            <div><span>失败步骤</span><b>{system?.lastFailedStepName ?? "—"}</b></div>
            <div><span>状态迁移</span><b>{system?.transitionCount ?? "—"}</b></div>
            <div><span>最近 ACK</span><b>{lastAck?.resultName ?? "—"}</b></div>
          </div>
          <div className="exo-flags">
            <Flag ok={Boolean(systemFlags?.initialized)}>初始化</Flag>
            <Flag ok={Boolean(systemFlags?.readyToStart)}>可启动</Flag>
            <Flag ok={Boolean(systemFlags?.runRequested)}>运行请求</Flag>
            <Flag ok={Boolean(systemFlags?.outputAllowed)}>输出许可</Flag>
            <Flag ok={Boolean(systemFlags?.executorAlive)}>执行器</Flag>
          </div>
        </article></div></details>
        </section>
        <section className="exo-panel" hidden={activePanel !== "settings"} aria-labelledby="exo-nav-settings">
          <section className="exo-card exo-direction-teaching" aria-label="前抬正方向标定">
            <div className="exo-card-heading"><div><p className="exo-kicker">首次装配或更换电机后进行</p>
              <h2>出厂配置</h2><p>标定前抬腿为正</p></div>
              <span className={`exo-state-chip ${leftMappingConfirmed && rightMappingConfirmed ? "good" : "muted"}`}>
                {hasDirectionTeaching && leftMappingConfirmed && rightMappingConfirmed ? "出厂方向已读回" : directionTeaching ? "等待手动前抬" : "先方向，后零位"}</span></div>
            <p>双腿保持中立位，先记录一侧静止基准。手动向前抬这一条腿约5～15°并停稳，再确认方向；另一腿保持原位。左右分别完成后，回到中立位校准零位。</p>
            {!hasDirectionTeaching && <p className="exo-inline-warning">{connected ? "当前固件仅支持读回安装方向。升级至0.10.4后可写入方向标定，角度、步态与助力会统一采用前抬为正。" : "连接外骨骼后按设备能力启用方向标定。"}</p>}
            <div className="exo-wear-joints">
              {([
                [Side.LEFT, "左腿", leftCalibration, leftMappingConfirmed],
                [Side.RIGHT, "右腿", rightCalibration, rightMappingConfirmed],
              ] as const).map(([side, label, calibration, confirmed]) => <article key={side}
                className={`exo-wear-joint ${side === Side.LEFT ? "left" : "right"}`}>
                <span>{label} · 前抬正方向</span>
                <strong className="exo-direction-result">{hasDirectionTeaching && confirmed ? "出厂方向已保存" : directionTeaching?.side === side ? "请前抬并停稳" : "等待标定"}</strong>
                <small>电机连续原始角：{calibration?.rawPositionDegrees != null ? `${calibration.rawPositionDegrees.toFixed(2)}°` : "—"}</small>
                <small>当前前抬映射：{calibration?.direction === 1 ? "原始角增加 → 人体为正" : calibration?.direction === 2 ? "原始角减少 → 人体为正" : "方向未知"}</small>
                <button className="exo-button secondary" disabled={!canTeachDirection || directionTeaching !== null}
                  onClick={() => void runOperation(`${label}记录方向基准`, () => beginDirectionTeaching(side))}>记录{label}静止基准</button>
                {directionTeaching?.side === side && <button className="exo-button primary" disabled={!canTeachDirection}
                  onClick={() => void runOperation(`${label}确认前抬方向`, finishDirectionTeaching)}>确认前抬方向</button>}
              </article>)}
            </div>
            {directionTeachNotice && <p className="exo-wear-readiness" role="status">{directionTeachNotice}</p>}
            {directionTeaching && <button className="exo-button ghost" disabled={!connected || Boolean(operation) || system?.state !== 0x01}
              onClick={() => void runOperation("取消方向标定", cancelDirectionTeaching)}>取消方向标定</button>}
            <p className="exo-wear-footnote">方向仅在待机且双电机REST时标定，保存到主控NVS，断电后保留。开始标定会撤销该侧当前方向有效位和旧零位；角度、速度、力矩统一使用前抬为正。更换电机或调整安装后请重新标定。</p>
          </section>
          <section className="exo-card exo-wear-calibration" aria-label="穿戴后零位校准">
            <div className="exo-card-heading">
              <div><p className="exo-kicker">穿戴后确认 · 沿用上次或重新校准</p><h2>穿戴后零位校准</h2></div>
              <span className={`exo-state-chip ${system?.state === 0x05 ? "warn" : lastWearCalibrationAt && bothZerosReadBack ? "good" : "muted"}`}>
                {!connected ? "尚未连接" : system?.state === 0x05 ? "故障锁定" : !bothFeedbackReady ? "等待有效反馈" : lastWearCalibrationAt && bothZerosReadBack ? "本次零位已确认" : bothZerosReadBack ? "设备已有零位" : "等待确认零位"}
              </span>
            </div>
            <p className="exo-wear-description">穿戴完成后站稳，双髋回到中立位并保持静止。可以沿用已保存的上次零位，也可以重新把当前位置校准为人体角度 0°。</p>
            <p className="exo-note">选择重新校准后，一次采集左右当前位置，同时写为本次中立位；读回两侧人体角均接近0°后才确认成功。校准不转动电机，请在读回完成前保持静止。</p>
            <fieldset className="exo-zero-choice" disabled={Boolean(operation)}>
              <legend>本次零位选择</legend>
              <label><input type="radio" name="wear-zero-choice" checked={wearZeroChoice === "reuse"}
                disabled={!hasSavedWearZero || !savedPairExists} onChange={() => {
                  wearZeroChoiceChosenRef.current = true; setWearZeroChoice("reuse"); setNeutralConfirmed(false);
                }}/><span>沿用上次零位</span></label>
              <label><input type="radio" name="wear-zero-choice" checked={wearZeroChoice === "new"}
                onChange={() => {wearZeroChoiceChosenRef.current = true; setWearZeroChoice("new"); setNeutralConfirmed(false);}}/>
                <span>重新校准</span></label>
            </fieldset>
            {!hasSavedWearZero && <p className="exo-operation-note">{connected
              ? "当前固件只保留本次RAM零位。升级到0.10.9后，可以保存并选择沿用上次零位；之前未保存的偏置无法从主控恢复。"
              : "连接后读取主控保存的上次零位；保存与沿用功能需要固件0.10.9。"}</p>}
            {hasSavedWearZero && <div className="exo-operation-note" aria-label="上次零位与沿用预览">
              {([['左髋',leftSavedZero],['右髋',rightSavedZero]] as const).map(([label,saved]) =>
                <p key={label}>{label} · 上次电机零位：{saved?.savedZeroDegrees != null ? `${saved.savedZeroDegrees.toFixed(2)}°` : "尚未保存"}
                  {' '}· 沿用后人体角：{saved?.previewJointDegrees != null ? `${saved.previewJointDegrees.toFixed(2)}°` : "等待有效预览"}</p>)}
              <p>{!leftSavedZero || !rightSavedZero ? "等待候选读回" : savedZeroReasonDescriptions[leftSavedZero.reason || rightSavedZero.reason]}。
                沿用保留原偏置，不把当前位置强制改成0°；任一侧相差超过5°时请重新校准。</p>
            </div>}
            <div className="exo-wear-joints">
              {([
                [Side.LEFT, "左髋", leftCalibration, leftJoint, leftMappingConfirmed, setLeftMappingConfirmed, canCheckLeftMapping],
                [Side.RIGHT, "右髋", rightCalibration, rightJoint, rightMappingConfirmed, setRightMappingConfirmed, canCheckRightMapping],
              ] as const).map(([side, label, calibration, joint, confirmed, setConfirmed, canCheck]) => (
                <article key={side} className={`exo-wear-joint ${side === Side.LEFT ? "left" : "right"}`}>
                  <span>{label} · 当前人体角度</span>
                  <div className="exo-wear-angle"><strong>{joint?.decodedJointFlags.jointValid ? joint.positionDegrees.toFixed(2) : "—"}</strong><b>°</b></div>
                  <small>零位偏置：{calibration?.softwareZeroDegrees !== null && calibration?.softwareZeroDegrees !== undefined ? `${calibration.softwareZeroDegrees.toFixed(2)}°` : "尚未读取"}</small>
                  <small className={joint?.decodedJointFlags.online && joint.decodedJointFlags.feedbackValid ? "" : "exo-feedback-invalid"}>
                    {!joint ? "尚未读取电机反馈" : joint.decodedJointFlags.online && joint.decodedJointFlags.feedbackValid ? "电机反馈在线" : "电机反馈无效或已离线"}
                    {joint ? ` · 反馈年龄 ${joint.feedbackAgeMs === 65535 ? "≥65.5秒/未知" : `${(joint.feedbackAgeMs / 1000).toFixed(3)} 秒`}` : ""}
                  </small>
                  <small>{calibration ? `对应电机 ${calibration.actuator === 0 ? "A" : calibration.actuator === 1 ? "B" : "—"} · 屈曲时原始角${calibration.direction === 1 ? "增加" : calibration.direction === 2 ? "减少" : "方向待核对"}` : "等待设备配置读回"}</small>
                  {hasCalibrationStatus && !hasDirectionTeaching && <label className="exo-confirm-row"><input type="checkbox" checked={confirmed}
                    disabled={!canCheck} onChange={(event) => {setConfirmed(event.target.checked); setNeutralConfirmed(false);}}/>
                    <span>已核对{side === Side.LEFT ? "左" : "右"}腿对应与屈曲方向</span></label>}
                  {hasDirectionTeaching && <small className="exo-confirm-row">{confirmed ? "✓ 已读回有效出厂方向" : "请先在上方完成这条腿的出厂方向标定"}</small>}
                </article>
              ))}
            </div>
            <label className="exo-confirm-row exo-wear-confirm"><input type="checkbox" checked={neutralConfirmed}
              disabled={!canConfirmNeutral} onChange={(event) => setNeutralConfirmed(event.target.checked)}/>
              <span>我已确认穿戴完成，双髋处于中立位并保持静止</span></label>
            {hasSavedWearZero && wearZeroChoice === "new" && <label className="exo-confirm-row">
              <input type="checkbox" checked={saveWearZeroForReuse} disabled={Boolean(operation)}
                onChange={(event) => setSaveWearZeroForReuse(event.target.checked)}/>
              <span>保存本次零位供下次沿用（断电保留）；不勾选只用于本次，旧候选保留</span>
            </label>}
            <div className="exo-wear-actions">
              <button className="exo-button primary" disabled={wearZeroChoice === "reuse" ? !canReuseBothZeros : !canCaptureBothZeros}
                onClick={() => void runOperation(wearZeroChoice === "reuse" ? "沿用上次零位" : "穿戴零位校准",
                  wearZeroChoice === "reuse" ? reuseSavedZeros : captureBothZeros)}>
                {operation === "穿戴零位校准" || operation === "沿用上次零位" ? "正在写入并读回零位…"
                  : wearZeroChoice === "reuse" ? "确认并沿用上次零位" : "校准并写入双髋零位"}
              </button>
              <button className="exo-button ghost" disabled={!connected || Boolean(operation)}
                onClick={() => void runOperation("读回双髋零位", async () => {
                  await querySystem(); await queryJoint(Side.LEFT); await queryJoint(Side.RIGHT);
                  if (hasCalibrationStatus) {await queryCalibration(Side.LEFT); await queryCalibration(Side.RIGHT);}
                  if (hasSavedWearZero) {await querySavedZero(Side.LEFT); await querySavedZero(Side.RIGHT);}
                })}>刷新零位读数</button>
            </div>
            {hasSavedWearZero && savedPairExists && <button className="exo-button ghost" disabled={!canConfirmNeutral}
              onClick={() => void runOperation("忘记上次零位", forgetSavedZeros)}>忘记已保存零位</button>}
            <p className="exo-wear-readiness">{wearZeroChoice === "reuse" && canConfirmNeutral
              ? !savedPairReuseReady ? savedZeroReasonDescriptions[leftSavedZero?.reason || rightSavedZero?.reason || 1]
                : neutralConfirmed ? "可以沿用：恢复上次偏置并核对本次双髋读回，不启动电机。" : "站稳保持中立静止，勾选本次穿戴确认后即可沿用。"
              : wearCalibrationBlockText}</p>
            {wearZeroNotice && <p className="exo-wear-readiness" role="status">{wearZeroNotice}</p>}
            {lastWearCalibrationAt && bothZerosReadBack && <p className="exo-wear-success" role="status">
              {lastWearCalibrationAt} 双髋软件零位已确认并读回。请检查上方左右人体髋角是否符合中立位。
            </p>}
            <p className="exo-wear-footnote">本次生效零位在RAM；已保存的上次零位在主控NVS，重启后需选择沿用并确认当前穿戴姿态，或重新校准。调整安装、更换电机或机械置零后请重新校准。上述操作不启动电机、不修改电机机械零位。</p>
          </section>
          <details className="exo-advanced"><summary>左右电机详情与穿戴调试选项</summary>
          <section className="exo-card exo-calibration-panel" aria-label="出厂配置与穿戴零位">
        <div className="exo-card-heading">
          <div><p className="exo-kicker">设备只读详情</p><h2>左右电机与标定读回</h2></div>
          <span className={`exo-state-chip ${leftMappingConfirmed && rightMappingConfirmed ? "good" : "warn"}`}>
            {leftMappingConfirmed && rightMappingConfirmed ? hasDirectionTeaching ? "出厂方向已读回" : "本次人工核对完成" : "等待方向配置"}
          </span>
        </div>
        <p className="exo-operation-note">
          以下映射来自主控配置读回。先在 IDLE 手动小幅屈曲左腿、右腿，分别观察对应电机读数；
          方向 +1 时屈髋电机角增加，−1 时减少。核对按钮不使能、不摆腿。
          {hasDirectionTeaching ? "左右电机映射只读，实际方向来自主控保存的出厂记录；更改方向请使用上方出厂配置。" : "映射为只读出厂参数；此处确认只对本次连接有效，不写入电机或永久保存。"}
        </p>
        {!hasCalibrationStatus && <p className="exo-inline-warning">连接0.10.2后可读取出厂映射、电机原始角和RAM软件零位。</p>}
        <div className="exo-calibration-grid">
          {([
            [Side.LEFT, "LEFT 左腿", leftCalibration, leftJoint, leftMappingConfirmed],
            [Side.RIGHT, "RIGHT 右腿", rightCalibration, rightJoint, rightMappingConfirmed],
          ] as const).map(([side, label, calibration, joint, confirmed]) => (
            <article className="exo-calibration-side" key={side}>
              <h3>{label}</h3>
              <div className="exo-data-grid compact">
                <div><span>执行器槽位</span><b>{calibration?.actuator === 0 ? "A" : calibration?.actuator === 1 ? "B" : "—"}</b></div>
                <div><span>CAN ID</span><b>{calibration && calibration.canId !== 255 ? `0x${calibration.canId.toString(16).padStart(2, "0").toUpperCase()}` : "—"}</b></div>
                <div><span>屈曲方向</span><b>{calibration?.direction === 1 ? "+1（原始角增加）" : calibration?.direction === 2 ? "−1（原始角减少）" : "—"}</b></div>
                <div><span>反馈年龄</span><b>{calibration && calibration.feedbackAgeMs !== 65535 ? `${calibration.feedbackAgeMs} ms` : "无有效数据"}</b></div>
                <div><span>电机连续原始角</span><b>{calibration?.rawPositionDegrees !== null && calibration?.rawPositionDegrees !== undefined ? `${calibration.rawPositionDegrees.toFixed(3)}°` : "无有效数据"}</b></div>
                <div><span>中立位电机偏置</span><b>{calibration?.softwareZeroDegrees !== null && calibration?.softwareZeroDegrees !== undefined ? `${calibration.softwareZeroDegrees.toFixed(3)}°` : "尚未标定"}</b></div>
                <div><span>当前人体髋角</span><b>{joint?.decodedJointFlags.jointValid ? `${joint.positionDegrees.toFixed(3)}°` : "尚未就绪"}</b></div>
                <div><span>软件零位</span><b>{!calibration ? "尚未读取" : calibration.decodedJointFlags.zeroValid ? "本次RAM零位有效" : "无效，需要中立位标定"}</b></div>
              </div>
              <p className="exo-operation-note">{confirmed ? hasDirectionTeaching ? "已读回有效出厂方向" : "本次对应与方向已人工核对" : "请在上方配置本侧对应与方向"}</p>
            </article>
          ))}
        </div>
        <div className="exo-export-row">
          <button className="exo-button ghost" disabled={!connected || Boolean(operation) || !hasCalibrationStatus}
            onClick={() => void runOperation("读回出厂配置与零位", async () => {
              await querySystem(); await queryCalibration(Side.LEFT); await queryCalibration(Side.RIGHT);
              await queryJoint(Side.LEFT); await queryJoint(Side.RIGHT);
            })}>刷新出厂配置与零位</button>
          <label className="exo-confirm-row"><input type="checkbox" checked={autoPolling} disabled={!connected}
            onChange={(event) => setAutoPolling(event.target.checked)} /><span>自动刷新读数（只读）</span></label>
          <label className="exo-confirm-row"><input type="checkbox" checked={humanWornMode}
            disabled={Boolean(operation) || (connected && system?.state !== 0x01)}
            onChange={(event) => { setHumanWornMode(event.target.checked); setNeutralConfirmed(false); }} />
            <span>人体穿戴调试：START只允许低阻尼产品配置</span></label>
        </div>
        <div className="exo-export-row">
          <button className="exo-button ghost" disabled={!connected || Boolean(operation) || !hasControlConfiguration}
            onClick={() => void runOperation("恢复穿戴产品待机", restoreProductControlConfiguration)}>恢复产品低阻尼待机</button>
        </div>
        <p className="exo-operation-note">
          {hasCalibrationStatus && !factoryMappingConfirmed ? "先核对左右腿，再确认中立位并保存零位。" : "零位状态由设备读回确认。"}
          {humanWornMode && !humanBaseCompatible && " 当前基础配置尚未读回为低阻尼产品配置，START保持锁定。"}
          真人先做SHADOW观察，实际附加助力为0；通过后再分级验证ACTIVE。
        </p>
      </section>
          </details>
          <details className="exo-advanced"><summary>单侧校准与清除零位</summary>
          <p className="exo-operation-note">单侧校准同样需要上方当前穿戴中立位确认。</p>

          <div className="exo-action-grid">
            <button className="exo-button secondary" disabled={!canCaptureLeftZero} onClick={() => void runOperation("左髋归零", () => captureZero(Side.LEFT))}>左髋归零</button>
            <button className="exo-button secondary" disabled={!canCaptureRightZero} onClick={() => void runOperation("右髋归零", () => captureZero(Side.RIGHT))}>右髋归零</button>
            <button className="exo-button secondary wide" disabled={!canCaptureBothZeros} onClick={() => void runOperation("双髋依次归零", captureBothZeros)}>双髋依次归零</button>


            <button className="exo-button danger wide" disabled={!connected || !stage7InfoAccepted || Boolean(operation) || system?.state !== 0x01} onClick={() => void runOperation("清除双侧零位", clearBothZeros)}>清除双侧 RAM 零位</button>
          </div>

          </details>
        </section>
        <section className="exo-panel" hidden={activePanel !== "motion"} aria-labelledby="exo-nav-motion">

<article className="exo-card exo-motion-plan">
  <div className="exo-card-heading"><div><p className="exo-kicker">左右独立调节</p><h2>助力等级</h2></div>
    <span className={`exo-state-chip ${assistFlags?.realOutputEnabled ? "warn" : "muted"}`}>{!assistConfig ? "助力状态未读取" : assistFlags?.realOutputEnabled ? "助力输出中" : "附加助力关闭"}</span></div>
  <p className="exo-operation-note">在待机时设置，保存并读回后单独启动。人体先主动运动，主控按所选策略提供助力。</p>
  <div className="exo-motion-controls"><label><span>助力策略</span>
    <select aria-label="助力策略" value={assistConfigDraft.strategyId}
      disabled={Boolean(operation) || (connected && system?.state !== 0x01)}
      onChange={(event) => {
        const strategy = Number(event.target.value);
        setAssistConfigDraft((draft) => ({...draft, strategyId: String(strategy),
          selectedMode: strategy === 4 ? AssistMode.ANGLE_THRESHOLD : strategy === 2 ? AssistMode.ACTIVE_PHASE : AssistMode.TRANSPARENT,
          profileId: strategy === 2 ? "1" : "0", executionState: AssistExecutionState.SHADOW}));
        // 切回相位曲线时收回草稿范围，不隐式修改设备已保存配置。
        if (strategy !== AssistStrategy.ANGLE_THRESHOLD && Number(maximumAssistDraft) > 0.5)
          setMaximumAssistDraft("0.500");
      }}>
      <option value="1">透明观察 · 不施加附加助力</option>
      <option value="2">步态相位曲线助力</option>
      <option value="4" disabled={connected && !hasPulseAssist}>摆腿事件脉冲助力 · 单向前抬</option>
      {Number(assistConfigDraft.strategyId) === 3 && <option value="3">按需辅助 · 尚未实现</option>}
    </select></label></div>
  <p className="exo-motion-note">设备已保存策略：{assistConfig?.strategyName ?? "尚未读取"}。切换选项只修改草稿。</p>
  {angleStrategyDraft && <div className="exo-assist-maximum">
    <label><span>前抬角度阈值（°）</span><div>
      <input aria-label="前抬角度阈值（°）" type="number" min="1" max="60" step="0.01"
        value={angleThresholdDraft} disabled={Boolean(operation) || (connected && system?.state !== 0x01)}
        onChange={(event) => setAngleThresholdDraft(event.target.value)} /><b>°</b></div></label>
    <p>设备已保存：<strong>{angleAssistConfig?.selectedThresholdDegrees != null && angleAssistConfig.selectedThresholdDegrees >= 1
      ? `${angleAssistConfig.selectedThresholdDegrees.toFixed(2)}°` : "尚未读取"}</strong>
      {angleAssistConfig?.activeValid && ` · 本轮锁存 ${angleAssistConfig.activeThresholdDegrees?.toFixed(2)}°`}</p>
    <small>本侧完成复位→新的正向越阈摆腿事件→一次平滑脉冲→等待下一次复位。持续抬腿不会重复助力，左右不要求交替；反向、停止摆腿或低速保持100 ms、角度越界、STOP或反馈失效时提前清零。</small>
    {!hasPulseAssist && <p role="status">{info ? "当前固件使用旧策略；事件脉冲需要升级0.10.10，当前保存已锁定。" : "尚未连接：当前仅预览草稿，事件脉冲需要0.10.10固件。"}</p>}
    <div className="exo-pulse-parameters">
      <label><span>脉冲时长模式</span><select aria-label="脉冲时长模式" value={pulseDraft.durationMode}
        disabled={Boolean(operation) || (connected && (!hasPulseAssist || ![1,3].includes(system?.state ?? -1)))}
        onChange={event => setPulseDraft(d=>({...d,durationMode:Number(event.target.value)}))}>
        <option value="0">FIXED · 固定时长（初次调试）</option><option value="1">PERIOD_SCALED · 本侧周期比例</option>
      </select></label>
      <label><span>下一脉冲申请峰值（N·m，100%等级）</span><input aria-label="下一脉冲申请峰值（N·m）" type="number" min="0" max={pulsePeakMaximumNm} step="0.001"
        value={pulseDraft.peakNm} disabled={Boolean(operation) || (connected && !hasPulseAssist)}
        onChange={event=>setPulseDraft(d=>({...d,peakNm:event.target.value}))}/></label>
      {pulseDraft.durationMode===0 && <label><span>固定脉冲时长（ms）</span><input aria-label="固定脉冲时长（ms）" type="number" min="100" max={fixedPulseMaximumMs} step="10"
        value={pulseDraft.fixedMs} disabled={Boolean(operation) || (connected && !hasPulseAssist)}
        onChange={event=>setPulseDraft(d=>({...d,fixedMs:event.target.value}))}/></label>}
      {pulseDraft.durationMode===1 && ([
        ["ratio","本侧周期比例 r",0.1,0.6,0.001], ["minimumMs","最短脉冲时长（ms）",100,2000,10],
        ["maximumMs","最长脉冲时长（ms）",100,2000,10], ["periodMinimumMs","合理周期下限（ms）",300,5000,10],
        ["periodMaximumMs","合理周期上限（ms）",300,5000,10],
      ] as const).map(([field,label,min,max,step])=><label key={field}><span>{label}</span><input aria-label={label} type="number" min={min} max={max} step={step}
        value={pulseDraft[field]} disabled={Boolean(operation) || (connected && !hasPulseAssist)}
        onChange={event=>setPulseDraft(d=>({...d,[field]:event.target.value}))}/></label>)}
      {([
        ["resetDegrees","复位角（°）",-30,59], ["maximumDegrees","允许前抬角上限（°）",3,90],
      ] as const).map(([field,label,min,max])=><label key={field}><span>{label}</span><input aria-label={label} type="number" min={min} max={max} step="0.01"
        value={pulseDraft[field]} disabled={Boolean(operation) || (connected && !hasPulseAssist)}
        onChange={event=>setPulseDraft(d=>({...d,[field]:event.target.value}))}/></label>)}
    </div>
    <p className="exo-motion-note">τ=A×sin²(πu)，u仅为本次脉冲进度。上升速率上限：{pulseRiseLimit ?? "尚未确认"} N·m/s{!info ? "（新版草稿，需连接0.10.13核对）" : "（依据当前固件）"}。A先乘本侧等级，再受上升速率×Ta/π限制。
      {pulseRiseLimit !== null && <> 100%等级、申请15 N·m时，完整峰值需要至少{Math.ceil(Math.PI*15/pulseRiseLimit*1000)} ms；3秒的速率约束峰值最多{Math.min(15,pulseRiseLimit*3/Math.PI).toFixed(2)} N·m。仍需新事件及有效反馈，反向、停止摆腿或窗口越界会提前结束。</>}
      PERIOD_SCALED每侧独立确认两个合理事件间隔后才出力，Ta=clamp(r×T,最短,最长)，无新事件就无新脉冲。</p>
    <p>设备已保存脉冲：{pulseTiming ? `${pulseTiming.durationMode===0?"FIXED":"PERIOD_SCALED"} · 申请峰值 ${pulseTiming.peakNm.toFixed(3)} N·m · 固定 ${pulseTiming.fixedMs} ms · r=${pulseTiming.ratio.toFixed(3)}` : "尚未读取"}
      {pulseWindow && ` · 复位≤${pulseWindow.resetDegrees.toFixed(2)}° / 上限${pulseWindow.maximumDegrees.toFixed(2)}°`}</p>
    <button className="exo-button secondary" disabled={!connected || !hasPulseAssist || Boolean(operation) ||
        ![1,3].includes(system?.state ?? -1) || assistConfig?.strategyId!==AssistStrategy.ANGLE_THRESHOLD}
      onClick={()=>void runOperation("保存下一脉冲参数",async()=>{await writePulseConfiguration(requestedPulse());})}>
      保存下一脉冲参数并读回</button>
    <small>运行中可更新上述参数：当前脉冲A和Ta冻结，下一次新事件才使用新值；角度窗口收紧立即生效。峰值不得超过本轮START锁存上限。</small>
    <div className="exo-assist-level-grid">{([["LEFT",pulseLeft],["RIGHT",pulseRight]] as const).map(([side,status])=><div key={side}>
      <strong>{side} · {status ? pulseStateDescriptions[status.state] : "尚未读取脉冲状态"}</strong>
      {status && <><p>有效事件 {status.eventCount} / 脉冲 {status.pulseCount} · {status.armed?"已锁存运行方式":"未启动"}</p>
        <p>冻结峰值 {status.frozenPeakNm.toFixed(3)} N·m / 时长 {status.frozenDurationMs} ms{status.rateLimited?"（受上升速率限制）":""}</p>
        <p>本侧周期 {status.periodTrusted?`${status.estimatedPeriodMs} ms` : "尚不可信"} · 脉冲进度 {(status.progress*100).toFixed(0)}%（非步态相位）</p>
        <small>{pulseExitDescriptions[status.exitReason]}</small></>}
    </div>)}</div>
  </div>}
  <div className="exo-assist-maximum">
    <label><span>{angleStrategyDraft ? "START锁存最大助力力矩（N·m）" : "最大附加助力力矩（N·m）"}</span>
      <div><input aria-label="最大附加助力力矩（N·m）" type="number" min="0" max={maximumHardNm} step="0.001"
        value={hasAssistTorqueLimit || angleStrategyDraft ? maximumAssistDraft : "0.500"}
        disabled={!(angleStrategyDraft && !connected && !operation) && (!canEditAssistConfiguration || !hasAssistTorqueLimit || !assistTorqueLimit?.selectedValid)}
        onChange={(event) => setMaximumAssistDraft(event.target.value)} /><b>N·m</b></div></label>
    <p>设备已保存：<strong>{savedMaximumNm !== null ? `${savedMaximumNm.toFixed(3)} N·m` : "尚未读取"}</strong>
      {assistTorqueLimit?.activeValid && ` · 本轮锁存 ${assistTorqueLimit.activeMaximumNm?.toFixed(3)} N·m`}</p>
    <small>{angleStrategyDraft ? "锁存上限约束下一脉冲申请峰值；实际力矩按单次sin²曲线升降" : "最大值 × 单侧等级 = 该侧峰值附加力矩；相位曲线决定正负方向"}，不含基础阻尼。当前工程上限 {maximumHardNm.toFixed(3)} N·m。</small>
    {angleStrategyDraft && (!info || !hasPulseAssist) && <small role="status">事件脉冲需要0.10.10起的固件；15 N·m/3秒范围需要0.10.12起，20 N·m/s需要0.10.13。当前仅为草稿，尚未写入设备。</small>}
    {!maximumDraftAllowed && <small role="status">草稿超出当前设备或策略范围，尚未写入，保存已锁定。</small>}
    {!hasAssistTorqueLimit && <small>{angleStrategyDraft ? "当前力矩仅为草稿，尚未写入设备。" : info ? "当前固件的曲线峰值固定；0.10.3固件支持写入力矩上限。" : "连接0.10.3后可设置并读回，当前输入未生效。"}</small>}
  </div>
  <div className="exo-assist-level-grid">{([
    ["左髋助力等级", "leftLevelPercent", "LEFT"], ["右髋助力等级", "rightLevelPercent", "RIGHT"],
  ] as const).map(([label, field, side]) => <label className={`exo-assist-level ${side.toLowerCase()}`} key={field}>
    <span>{side} · {label}</span><div><strong>{assistConfigDraft[field]}</strong><b>%</b></div>
    <input aria-label={label} type="range" min="0" max="100" step="1" value={assistConfigDraft[field]}
      disabled={!canEditAssistConfiguration && !(angleStrategyDraft && !connected && !operation)} onChange={(event) => setAssistConfigDraft((draft) => ({...draft, [field]: event.target.value}))}/>
    <footer><span>0% 关闭</span><span>100%</span></footer>
    <small>草稿峰值：{draftMaximumNm !== null && Number.isFinite(draftMaximumNm) && draftMaximumNm >= 0 && draftMaximumNm <= maximumHardNm
      ? `${(draftMaximumNm * Number(assistConfigDraft[field]) / 100).toFixed(3)} N·m` : "—"}</small>
    <small>设备已保存：{assistConfig ? `${field === "leftLevelPercent" ? assistConfig.leftLevelPercent : assistConfig.rightLevelPercent}%` : "尚未读取"}</small>
    <small>已保存峰值：{savedMaximumNm !== null && assistConfig
      ? `${(savedMaximumNm * (field === "leftLevelPercent" ? assistConfig.leftLevelPercent : assistConfig.rightLevelPercent) / 100).toFixed(3)} N·m` : "尚未读取"}</small>
  </label>)}</div>
  <div className="exo-motion-controls"><label><span>运行方式</span><select aria-label="运行方式" value={assistConfigDraft.executionState}
    disabled={!canEditAssistConfiguration && !(angleStrategyDraft && !connected && !operation)} onChange={(event) => setAssistConfigDraft((draft) => ({...draft, executionState: Number(event.target.value)}))}>
    <option value={AssistExecutionState.DISABLED}>关闭附加助力</option><option value={AssistExecutionState.SHADOW}>观察模式 · 仅计算建议</option>
    <option value={AssistExecutionState.ACTIVE} disabled={!hasTelemetryStream || !activePhaseBaseCompatible || (angleStrategyDraft && !hasPulseAssist) || ![2,4].includes(Number(assistConfigDraft.strategyId))}>真实助力 · ACTIVE</option></select></label>
    <button className="exo-button primary" disabled={!canEditAssistConfiguration || !assistActiveDraftAllowed || !maximumDraftAllowed || (angleStrategyDraft && !hasPulseAssist)}
      onClick={() => void runOperation("保存产品助力等级", async () => {
        const pulse=angleStrategyDraft?requestedPulse():null;
        if(pulse && pulse.timing.peakNm>Number(maximumAssistDraft)) throw new Error("申请峰值不能超过START力矩上限");
        await writeAssistConfig(assistConfigDraft);
        if(pulse) await writePulseConfiguration(pulse);
      })}>保存助力设置并读回</button></div>
  <p className="exo-motion-note">观察模式的实际附加助力为0。当前助力曲线仍在验收，真人调试从观察模式开始，再分级验证真实助力。</p>
</article>
<article className="exo-card exo-motion-start">
  <div className="exo-card-heading"><div><p className="exo-kicker">穿戴校准后执行</p><h2>开始与停止</h2></div>
    <span className={`exo-state-chip ${operation ? "working" : "muted"}`}>{operation || "等待操作"}</span></div>
  <div className="exo-motion-actions"><button className="exo-button start wide" disabled={!canStart} onClick={() => void runOperation("START 到 RUNNING", startExoskeleton)}>START 并等待 RUNNING</button> <button className="exo-button stop wide" disabled={!canStop} onClick={() => void runStopOperation()}>STOP 并等待 IDLE</button></div>
  <p className="exo-operation-note">{actionReadinessText}</p>
  <p className="exo-operation-note">启动与停止均等待主控确认最终状态。BLE断开不会自动停止外骨骼。</p>
</article>

          <details className="exo-advanced"><summary>高级助力参数与设备门禁</summary><article className="exo-card exo-assist-card">
          <div className="exo-card-heading">
            <div>
              <p className="exo-kicker">阶段 10 · 相位助力策略</p>
              <h2>助力模式与策略</h2>
            </div>
            <span className={`exo-state-chip ${
              !hasAssistConfiguration
                ? "muted"
                : assistFlags?.realOutputEnabled ? "warn"
                  : assistFlags?.selectedValid ? "good" : "warn"
            }`}>
              {!hasAssistConfiguration
                ? info ? "固件未支持" : "等待连接"
                : assistConfig?.executionStateName ?? "等待读回"}
            </span>
          </div>

          <p className="exo-active-contract">
            <strong>产品 ACTIVE 启动合同</strong>
            产品 ACTIVE 不下发位置步态轨迹。START 后必须由人体先带动双髋；
            相位曲线策略：固件只有看到本次 START 之后 LEFT、RIGHT 各自出现至少一个新的髋屈曲峰，
            且相位与置信度达到要求，才会从 0 开始渐入助力力矩。
            角度阈值策略：每侧先回到退出阈值以下，再前抬超过正阈值才触发；无需相位锁定。
            ALTERNATING_SINE_TEST 工程激励只能配合 SHADOW 验证，禁止用于 ACTIVE。
          </p>

          {!hasAssistConfiguration ? (
            <p className="exo-capability-empty">
              {info
                ? "当前固件未声明 ASSIST_CONFIGURATION capability，网页不会发送未知助力命令。"
                : "连接后将根据设备 capability 自动发现并读取助力配置。"}
            </p>
          ) : (
            <>
              <div className="exo-control-summary exo-assist-summary">
                <div><span>已选择</span><b>{assistFlags?.selectedValid ? assistConfig?.selectedModeName : "—"}</b></div>
                <div><span>本轮活动</span><b>{assistFlags?.activeValid ? assistConfig?.activeModeName : "NONE"}</b></div>
                <div><span>请求执行</span><b>{assistConfig?.requestedExecutionStateName ?? "—"}</b></div>
                <div><span>当前有效</span><b>{assistConfig?.executionStateName ?? "—"}</b></div>
                <div><span>策略</span><b>{assistConfig ? `${assistConfig.strategyName} v${assistConfig.strategyVersion}` : "—"}</b></div>
                <div><span>曲线 / 代次</span><b>{assistConfig ? `profile ${assistConfig.profileId} / gen ${assistConfig.configGeneration}` : "—"}</b></div>
                <div><span>不可用原因</span><b>{assistUnavailableReasonLabel(assistConfig)}</b></div>
              </div>

              <div className="exo-assist-sensors">
                <div>
                  <span>所需传感器 {formatHexMask(assistConfig?.requiredSensorMask)}</span>
                  <b>{sensorMaskDescription(assistConfig?.decodedRequiredSensorMask)}</b>
                </div>
                <div>
                  <span>已有传感器 {formatHexMask(assistConfig?.availableSensorMask)}</span>
                  <b>{sensorMaskDescription(assistConfig?.decodedAvailableSensorMask)}</b>
                </div>
              </div>

              <div className="exo-control-selectors exo-assist-selectors">
                <label>
                  <span>产品助力模式</span>
                  <select
                    value={assistConfigDraft.selectedMode}
                    disabled={!canEditAssistConfiguration}
                    onChange={(event) => setAssistConfigDraft((draft) => ({
                      ...draft,
                      selectedMode: Number(event.target.value),
                      profileId: Number(event.target.value) === AssistMode.ACTIVE_PHASE ? "1" : "0",
                      executionState: AssistExecutionState.SHADOW,
                      strategyId: defaultStrategyIdForMode(
                        Number(event.target.value),
                      ),
                    }))}
                  >
                    {[AssistMode.TRANSPARENT, AssistMode.ACTIVE_PHASE, AssistMode.ASSIST_AS_NEEDED, AssistMode.ANGLE_THRESHOLD]
                      .map((mode) => <option key={mode} value={mode}>{assistModeNames[mode]}</option>)}
                  </select>
                </label>
                <label>
                  <span>请求执行态</span>
                  <select
                    value={assistConfigDraft.executionState}
                    disabled={!canEditAssistConfiguration}
                    onChange={(event) => setAssistConfigDraft((draft) => ({
                      ...draft,
                      executionState: Number(event.target.value),
                    }))}
                  >
                    {[AssistExecutionState.DISABLED, AssistExecutionState.SHADOW,
                      AssistExecutionState.ACTIVE]
                      .map((state) => (
                        <option
                          key={state}
                          value={state}
                          disabled={state === AssistExecutionState.ACTIVE && (
                            !hasTelemetryStream || !activePhaseBaseCompatible ||
                            (assistConfigDraft.selectedMode !== AssistMode.ACTIVE_PHASE && !(hasPulseAssist && angleStrategyDraft))
                          )}
                        >
                          {assistExecutionStateNames[state]}
                        </option>
                      ))}
                  </select>
                </label>
                {([
                  ["策略 ID", "strategyId", 255],
                  ["曲线 ID", "profileId", 255],
                  ["LEFT 助力等级（%）", "leftLevelPercent", 100],
                  ["RIGHT 助力等级（%）", "rightLevelPercent", 100],
                ] as const).map(([label, field, maximum]) => (
                  <label key={field}>
                    <span>{label}</span>
                    <input
                      type="number"
                      min="0"
                      max={maximum}
                      step="1"
                      value={assistConfigDraft[field]}
                      disabled={!canEditAssistConfiguration}
                      onChange={(event) => setAssistConfigDraft((draft) => ({
                        ...draft,
                        [field]: event.target.value,
                      }))}
                    />
                  </label>
                ))}
              </div>

              <div className="exo-flags" aria-label="助力策略标志">
                <Flag ok={Boolean(assistFlags?.selectedValid)}>选择有效</Flag>
                <Flag ok={Boolean(assistFlags?.activeValid)}>活动有效</Flag>
                <Flag ok={Boolean(assistFlags?.locked)}>配置锁存</Flag>
                <Flag ok={Boolean(assistFlags?.strategyReady)}>策略就绪</Flag>
                <Flag ok={Boolean(assistFlags?.sensorsReady)}>传感器齐备</Flag>
                <Flag ok={Boolean(assistFlags?.suggestionValid)}>建议命令有效</Flag>
                <Flag ok={assistConfig?.executionState === AssistExecutionState.ACTIVE
                  ? Boolean(assistFlags?.realOutputEnabled)
                  : !Boolean(assistFlags?.realOutputEnabled)}>
                  {assistFlags?.realOutputEnabled ? "真实助力已开启" : "真实输出关闭"}
                </Flag>
                {Boolean(assistFlags?.unknown) && <Flag ok={false}>未知助力位</Flag>}
              </div>

              <div className="exo-assist-actions">
                <div className="exo-assist-contracts">
                  <p className="exo-shadow-note">
                    <strong>SHADOW 只计算建议命令，不改变真实 CAN 输出。</strong>
                    阶段10的 ACTIVE 允许相位曲线或角度阈值策略叠加在
                    DAMPING + CONSTANT，并且配置只能在 IDLE 写入。
                    {!activePhaseBaseCompatible && hasTelemetryStream && (
                      <em> 当前基础配置不兼容，ACTIVE 已锁定；请先在下方应用 DAMPING + CONSTANT。</em>
                    )}
                  </p>
                </div>
                <div className="exo-command-buttons">
                  <button
                    className="exo-button ghost"
                    disabled={!connected || !hasAssistConfiguration || Boolean(operation)}
                    onClick={() => void runOperation("读回助力配置", async () => {
                      await queryAssistConfig();
                    })}
                  >
                    读回助力配置
                  </button>
                  <button
                    className="exo-button primary"
                    disabled={!canEditAssistConfiguration || !assistActiveDraftAllowed || (angleStrategyDraft && !hasPulseAssist)}
                    onClick={() => void runOperation("写入助力配置并读回", async () => {
                      await writeAssistConfig();
                    })}
                  >
                    写入助力配置并读回
                  </button>
                </div>
              </div>
            </>
          )}
        </article></details>
          <details className="exo-advanced"><summary>基础控制与工程调试</summary><p className="exo-operation-note">工程轨迹只用于台架验证，人体穿戴请保留低阻尼配置。</p><section className="exo-card exo-control-card">
        <div className="exo-card-heading">
          <div>
            <p className="exo-kicker">阶段 8 控制配置</p>
            <h2>模式、参考源与左右五元命令</h2>
          </div>
          <span className={`exo-state-chip ${hasControlConfiguration ? "good" : "muted"}`}>
            {hasControlConfiguration ? "设备支持" : "固件未声明能力"}
          </span>
        </div>

        <div className="exo-control-summary">
          <div>
            <span>已选择</span>
            <b>{controlConfig?.selectedValid
              ? `${controlConfig.selectedModeName} / ${controlConfig.selectedReferenceName}`
              : "—"}</b>
          </div>
          <div>
            <span>本轮活动</span>
            <b>{controlConfig?.activeValid
              ? `${controlConfig.activeModeName} / ${controlConfig.activeReferenceName}`
              : "NONE（尚未 START）"}</b>
          </div>
          <div>
            <span>修改门</span>
            <b>{system?.state === 0x01 ? "IDLE：允许写入" : `${system?.stateName ?? "未知"}：先 STOP`}</b>
          </div>
        </div>

        <div className="exo-control-selectors">
          <label>
            <span>控制模式</span>
            <select
              value={selectedMode}
              disabled={!canEditControlConfiguration}
              onChange={(event) => {
                const mode = Number(event.target.value);
                setSelectedMode(mode);
                // SINE 在阶段 8 只属于阻抗模式。离开 IMPEDANCE 时立即恢复
                // CONSTANT，避免界面保留一个固件必然拒绝的组合。
                if (mode !== ControlMode.IMPEDANCE) {
                  setSelectedReference(ReferenceSource.CONSTANT);
                }
              }}
            >
              {[
                ControlMode.HOLD,
                ControlMode.POSITION,
                ControlMode.VELOCITY,
                ControlMode.TORQUE,
                ControlMode.DAMPING,
                ControlMode.IMPEDANCE,
              ].map((mode) => <option key={mode} value={mode}>{controlModeNames[mode]}</option>)}
            </select>
          </label>
          <label>
            <span>参考源</span>
            <select
              value={selectedReference}
              disabled={!canEditControlConfiguration ||
                selectedMode !== ControlMode.IMPEDANCE}
              onChange={(event) => setSelectedReference(Number(event.target.value))}
            >
              <option value={ReferenceSource.CONSTANT}>{referenceSourceNames[ReferenceSource.CONSTANT]}</option>
              <option
                value={ReferenceSource.SINE}
                disabled={selectedMode !== ControlMode.IMPEDANCE}
              >
                {referenceSourceNames[ReferenceSource.SINE]}
              </option>
            </select>
          </label>
          <div className="exo-command-buttons">
            <button
              className="exo-button ghost"
              disabled={!connected || !hasControlConfiguration || Boolean(operation)}
              onClick={() => void runOperation("读回控制配置", async () => {
                await queryControlConfig();
                await queryJointCommand(Side.LEFT);
                await queryJointCommand(Side.RIGHT);
              })}
            >
              读回全部配置
            </button>
            <button
              className="exo-button primary"
              disabled={!canEditControlConfiguration}
              onClick={() => void runOperation("写入模式并读回", async () => {
                await writeControlConfig();
              })}
            >
              写入模式并读回
            </button>
          </div>
        </div>

        <div className="exo-command-grid">
          <JointCommandEditor
            title="LEFT 左髋五元命令"
            side={Side.LEFT}
            draft={leftCommandDraft}
            readback={leftCommand}
            unavailable={!connected || !hasControlConfiguration || Boolean(operation)}
            writeDisabled={!canEditControlConfiguration}
            onDraft={setLeftCommandDraft}
            onRead={() => void runOperation("读回 LEFT 五元命令", async () => {
              await queryJointCommand(Side.LEFT);
            })}
            onWrite={() => void runOperation("写入 LEFT 五元命令", async () => {
              await writeJointCommand(Side.LEFT, leftCommandDraft);
            })}
          />
          <JointCommandEditor
            title="RIGHT 右髋五元命令"
            side={Side.RIGHT}
            draft={rightCommandDraft}
            readback={rightCommand}
            unavailable={!connected || !hasControlConfiguration || Boolean(operation)}
            writeDisabled={!canEditControlConfiguration}
            onDraft={setRightCommandDraft}
            onRead={() => void runOperation("读回 RIGHT 五元命令", async () => {
              await queryJointCommand(Side.RIGHT);
            })}
            onWrite={() => void runOperation("写入 RIGHT 五元命令", async () => {
              await writeJointCommand(Side.RIGHT, rightCommandDraft);
            })}
          />
        </div>

        <div className="exo-control-footer">
          <p className="exo-operation-note">
            手机只在 IDLE 写一次静态配置；ESP32 在本地 100 Hz 生成命令。网页不会通过 BLE 连续发送五元命令。IMPEDANCE + SINE 时，q 表示相对本次 START 中心的偏置（默认 0°），不是电机绝对角度。RUNNING 中请先 STOP，再修改并读回，最后单独 START。
          </p>
          <button
            className="exo-button secondary"
            disabled={!canApplyControlConfiguration}
            onClick={() => void runOperation(
              "STOP、应用全部控制配置并读回",
              applyAllControlConfiguration,
            )}
          >
            STOP → 应用全部 → 读回
          </button>
        </div>
      </section> <section className="exo-card exo-engineering-test-card">
        <div className="exo-card-heading">
          <div>
            <p className="exo-kicker">隔离工程验收 · 非产品功能</p>
            <h2>工程验证激励</h2>
          </div>
          <span className={`exo-state-chip ${
            engineeringTestPrepared ? "warn" : hasEngineeringTestStimulus ? "good" : "muted"
          }`}>
            {engineeringTestPrepared
              ? "TEST + SHADOW 已准备"
              : hasEngineeringTestStimulus ? "设备支持" : info ? "固件未支持" : "等待连接"}
          </span>
        </div>

        <p className="exo-operation-note">
          该入口让外骨骼以低幅左右交替正弦产生可重复的编码器运动，只用于验证
          <strong>反馈、步态估相、建议力矩和遥测</strong>链路。它固定使用
          IMPEDANCE + ALTERNATING_SINE_TEST 和 ACTIVE_PHASE + SHADOW，
          <strong>禁止 ACTIVE，不代表产品助力模式，也不能用于人体助力效果评价。</strong>
        </p>

        {!hasEngineeringTestStimulus ? (
          <p className="exo-capability-empty">
            {info
              ? "当前固件未声明 ENGINEERING_TEST_STIMULUS capability；网页不会发送工程激励参考。0.10.0 产品功能仍保持兼容。"
              : "连接 0.10.1 工程固件后，页面才会开放这个隔离入口。"}
          </p>
        ) : (
          <div className="exo-command-buttons">
            <button
              className="exo-button primary"
              disabled={!canConfigureEngineeringTest}
              onClick={() => void runOperation(
                "准备隔离工程验证激励",
                prepareEngineeringTestStimulus,
              )}
            >
              一键准备 TEST + SHADOW
            </button>
            <button
              className="exo-button secondary"
              disabled={!canConfigureEngineeringTest}
              onClick={() => void runOperation(
                "恢复产品待机配置",
                restoreProductControlConfiguration,
              )}
            >
              恢复产品 DAMPING + CONSTANT
            </button>
          </div>
        )}
        <p className="exo-operation-note">
          “一键准备”只写入并读回配置，不会自动 START。结束验证后点击恢复按钮，
          页面会写入 TRANSPARENT + DISABLED 和 DAMPING + CONSTANT。工程配置固定左右
          q=0°、dq=0°/s、τ=0 N·m、Kp=5.0、Kd=0.1；恢复后 Kp=0、Kd=0.1。
        </p>
      </section></details>
        </section>
        <section className="exo-panel" hidden={activePanel !== "live"} aria-labelledby="exo-nav-live">
          <section className="exo-card exo-telemetry-card">
        <div className="exo-card-heading">
          <div>
            <p className="exo-kicker">左右髋 · 实时采集</p>
            <h2>实时波形示波器</h2>
          </div>
          <span className={`exo-state-chip ${
            telemetryStatus?.streaming ? "good" : hasTelemetryStream ? "muted" : "warn"
          }`}>
            {telemetryStatus?.streaming
              ? `${telemetryStatus.rateHz} Hz 推流中`
              : hasTelemetryStream ? "已停止" : info ? "固件未支持" : "等待连接"}
          </span>
        </div>

        <div className="exo-scope-toolbar">
          <div className={`exo-scope-status ${scopeReception}`} role="status">
            <i/><strong>{scopeReception==="live"?"LIVE · 数据更新中":scopeReception==="stale"?"数据中断":scopeReception==="waiting"?"等待首帧":scopeReception==="disconnected"?"请连接外骨骼":"采集已停止"}</strong>
            <span>{lastTelemetryReceivedMs===null?"尚无新数据":`末帧 ${Math.max(0,scopeClockMs-lastTelemetryReceivedMs)/1000 < 1 ? "<1" : (Math.max(0,scopeClockMs-lastTelemetryReceivedMs)/1000).toFixed(1)} s前`}</span>
          </div>
          <label>时间窗口<select aria-label="波形时间窗口" value={scopeWindowSeconds} onChange={e=>setScopeWindowSeconds(Number(e.target.value))}>{[2,5,10,20,30].map(v=><option key={v} value={v}>{v} 秒</option>)}</select></label>
          <label>纵轴<select aria-label="波形纵轴" value={scopeAutoScale?"auto":"fixed"} onChange={e=>setScopeAutoScale(e.target.value==="auto")}><option value="auto">自动量程</option><option value="fixed">固定量程</option></select></label>
          <label className="scope-check left"><input type="checkbox" checked={scopeShowLeft} onChange={e=>setScopeShowLeft(e.target.checked)}/>LEFT 左髋</label>
          <label className="scope-check right"><input type="checkbox" checked={scopeShowRight} onChange={e=>setScopeShowRight(e.target.checked)}/>RIGHT 右髋</label>
          <label className="scope-check"><input type="checkbox" checked={telemetryAutoStart} onChange={e=>{telemetryAutoAttemptRef.current=null;setTelemetryAutoStart(e.target.checked);}}/>进入实时页自动采集</label>
        </div>
        {scopeReception==="stale"&&<p className="exo-inline-warning">超过1.5秒未收到新遥测，当前保留的是历史波形。请检查蓝牙与设备推流；重新开始采集不会启动电机。</p>}
        {scopeReception==="stopped"&&<p className="exo-scope-hint">采集已停止，点击“开始遥测”后显示真实波形。连接蓝牙不等于开启遥测。</p>}
        {!hasTelemetryStream ? (
          <p className="exo-capability-empty">
            {info
              ? "当前设备未声明 TELEMETRY_STREAM；仍可使用上方阶段9低频轮询。"
              : "连接0.10.0设备后，可按需启动独立遥测流。"}
          </p>
        ) : (
          <>
            {!telemetryCharacteristicReady && (
              <p className="exo-inline-warning">
                固件声明了遥测能力，但 GATT 中没有找到 Telemetry Notify 特征；
                命令操作仍可用，请核对固件与网页协议版本。
              </p>
            )}

            <details className="exo-scope-settings"><summary>采样与通道设置 · {telemetryStatus?.streaming?telemetryStatus.rateHz:telemetryRateHz} Hz · {selectedTelemetryChannelCount}通道</summary><div className="exo-telemetry-config">
              <label>
                <span>采样率</span>
                <select
                  value={telemetryRateHz}
                  disabled={Boolean(telemetryStatus?.streaming) || Boolean(operation)}
                  onChange={(event) => setTelemetryRateHz(Number(event.target.value) as 5 | 10 | 20)}
                >
                  {[5, 10, 20].map((rate) => <option key={rate} value={rate}>{rate} Hz</option>)}
                </select>
              </label>
              <div className="exo-telemetry-channels" aria-label="遥测通道">
                {TELEMETRY_CHANNEL_OPTIONS.map((channel) => (
                  <label key={channel.bit}>
                    <input
                      type="checkbox"
                      checked={(telemetryChannelMask & channel.bit) !== 0}
                      disabled={Boolean(telemetryStatus?.streaming) || Boolean(operation)}
                      onChange={() => toggleTelemetryChannel(channel.bit)}
                    />
                    <span>{channel.label}</span>
                  </label>
                ))}
              </div>
              <div className="exo-telemetry-rate-note">
                <span>有效 mask {formatHexMask(telemetryChannelMask)}</span>
                <b>预计 {estimatedNotifyRate} Notify/s</b>
                <small>
                  默认行走波形：角度、速度、相位、置信度与两种助力，10 Hz / 0x007B。多通道优先使用10 Hz；缺失或无效的数据不会补成0。
                </small>
              </div>
            </div>

            </details>
            {estimatedNotifyRate>80&&<p className="exo-inline-warning">当前预计{estimatedNotifyRate} Notify/s；多通道20 Hz可能积压。建议改为10 Hz，并观察丢样本和队列计数。</p>}
            <div className="exo-command-buttons exo-telemetry-actions">
              <button
                className="exo-button primary"
                disabled={!canStartTelemetry}
                onClick={() => void runOperation("开始遥测", startTelemetry)}
              >
                开始遥测
              </button>
              <button
                className="exo-button secondary"
                disabled={!canStopTelemetry}
                onClick={() => void runOperation("停止遥测", stopTelemetry)}
              >
                停止遥测（不停止外骨骼）
              </button>
              <button
                className="exo-button ghost"
                disabled={!connected || !hasTelemetryStream || Boolean(operation)}
                onClick={() => void runOperation("读取遥测状态", async () => {
                  await queryTelemetryStatus();
                })}
              >
                刷新设备计数
              </button>
              <button
                className="exo-button ghost"
                disabled={!telemetryCaptureRef.current.length}
                onClick={() => setTelemetryDisplayPause(!telemetryDisplayPaused)}
              >
                {telemetryDisplayPaused ? "恢复实时显示" : "暂停显示（继续采集）"}
              </button>
            </div>

            <div className="exo-data-grid exo-telemetry-stats">
              <div><span>STREAM</span><b>{telemetryStatus?.streamSequence ?? "—"}</b></div>
              <div><span>ACTIVATION EPOCH</span><b>{latestTelemetry?.activationEpochLow16 ?? "—"}</b></div>
              <div><span>网页完整时刻</span><b>{telemetryStats.uniqueSampleCount}</b></div>
              <div><span>接收 Notify</span><b>{telemetryStats.packetCount}</b></div>
              <div><span>设备成功 Notify</span><b>{telemetryStatus?.notifySuccessCount ?? "—"}</b></div>
              <div><span>设备丢样本</span><b>{telemetryStatus?.droppedSampleCount ?? "—"}</b></div>
              <div><span>网页序号缺口</span><b>{telemetryStats.webMissingSampleCount}</b></div>
              <div><span>队列</span><b>{telemetryStatus ? `${telemetryStatus.queuedSamples}/${telemetryStatus.queueCapacity}` : "—"}</b></div>
              <div><span>系统状态</span><b>{latestTelemetry ? stateNames[latestTelemetry.systemState] ?? latestTelemetry.systemState : "—"}</b></div>
              <div><span>实际助力门</span><b>{latestTelemetry?.realOutputEnabled ? "ENABLED" : "OFF"}</b></div>
            </div>

            <p className="exo-operation-note">
              {telemetryDisplayPaused
                ? "图表已冻结，但内存采集、序号检查和设备推流仍在继续。"
                : `横轴为主控时间，显示最近${scopeWindowSeconds}秒；缺帧处断线，完整记录最多保留10000个时刻。`}
              遥测断开只停止数据流，不会向系统状态机发送 STOP。
            </p>

            <div className="exo-export-row">
              <button className="exo-button ghost" disabled={!telemetryCaptureRef.current.length} onClick={exportTelemetryCsv}>导出遥测 CSV</button>
              <button className="exo-button ghost" disabled={!telemetryCaptureRef.current.length} onClick={exportTelemetryJsonl}>导出遥测 JSONL</button>
              <button className="exo-button ghost" disabled={!telemetryCaptureRef.current.length} onClick={clearTelemetrySamples}>清空遥测样本</button>
            </div>
          </>
        )}
        <TelemetryScope samples={telemetrySamples} windowSeconds={scopeWindowSeconds} rateHz={telemetryStatus?.rateHz ?? telemetryRateHz}
          channelMask={telemetryStatus?.streaming?telemetryStatus.channelMask:telemetryChannelMask} autoScale={scopeAutoScale}
          showLeft={scopeShowLeft} showRight={scopeShowRight} paused={telemetryDisplayPaused} reception={scopeReception}/>
      </section>
          <p className="exo-operation-note">下方是最近一次关节与步态诊断快照；推流时备用轮询暂停，请在上方示波器查看实时变化。</p>
          <section className="exo-joint-grid">
        <JointCard title="LEFT 左髋" expected={leftCalibration ? `执行器 ${leftCalibration.actuator === 0 ? "A" : leftCalibration.actuator === 1 ? "B" : "未知"} / ${leftCalibration.directionName}` : "尚未读取出厂映射"} joint={leftJoint} />
        <JointCard title="RIGHT 右髋" expected={rightCalibration ? `执行器 ${rightCalibration.actuator === 0 ? "A" : rightCalibration.actuator === 1 ? "B" : "未知"} / ${rightCalibration.directionName}` : "尚未读取出厂映射"} joint={rightJoint} />
      </section>
          <div className="exo-live-info"><span>角度 ° · 角速度 °/s · 力矩 N·m</span><label className="exo-confirm-row"><input type="checkbox" checked={autoPolling} disabled={!connected} onChange={(event) => setAutoPolling(event.target.checked)}/><span>自动更新关节状态</span></label></div>
          <article className="exo-card exo-gait-card">
          <div className="exo-card-heading">
            <div>
              <p className="exo-kicker">阶段 9 · 传感快照</p>
              <h2>步态估计</h2>
            </div>
            <span className={`exo-state-chip ${
              !hasGaitStatus
                ? "muted"
                : gaitFlags?.overallValid ? "good" : "warn"
            }`}>
              {!hasGaitStatus
                ? info ? "固件未支持" : "等待连接"
                : gaitStatus?.estimatorStateName ?? "等待快照"}
            </span>
          </div>

          {!hasGaitStatus ? (
            <p className="exo-capability-empty">
              {info
                ? "当前固件未声明 GAIT_STATUS capability，网页不会发送未知步态查询。"
                : "连接后将根据设备 capability 自动发现并读取步态状态。"}
            </p>
          ) : (
            <>
              <div className="exo-phase-grid">
                {([
                  ["LEFT 左腿相位", gaitStatus?.leftPhasePercent],
                  ["RIGHT 右腿相位", gaitStatus?.rightPhasePercent],
                ] as const).map(([label, phase]) => (
                  <div className="exo-phase" key={label}>
                    <div>
                      <span>{label}</span>
                      <strong>{phase === null || phase === undefined
                        ? "—"
                        : `${phase.toFixed(2)}%`}</strong>
                    </div>
                    <progress
                      aria-label={label}
                      max={100}
                      value={phase ?? 0}
                    />
                  </div>
                ))}
              </div>
              <div className="exo-data-grid compact exo-stage9-data">
                <div><span>估计器</span><b>{gaitStatus ? `${gaitStatus.estimatorName} / ID ${gaitStatus.estimatorId}` : "—"}</b></div>
                <div><span>相位基准</span><b>{gaitStatus?.phaseOriginName ?? "—"}</b></div>
                <div><span>步频</span><b>{gaitStatus?.cadenceSpm === null || gaitStatus?.cadenceSpm === undefined ? "—" : `${gaitStatus.cadenceSpm.toFixed(2)} step/min`}</b></div>
                <div><span>置信度</span><b>{gaitStatus?.confidencePercent === null || gaitStatus?.confidencePercent === undefined ? "—" : `${gaitStatus.confidencePercent.toFixed(2)}%`}</b></div>
                <div><span>估计年龄</span><b>{gaitStatus?.estimateAgeMs === null || gaitStatus?.estimateAgeMs === undefined ? "—" : `${gaitStatus.estimateAgeMs} ms`}</b></div>
                <div><span>质量标志</span><b>{formatHexMask(gaitStatus?.qualityFlags)}</b></div>
              </div>
              <div className="exo-flags" aria-label="步态有效性">
                <Flag ok={Boolean(gaitFlags?.overallValid)}>有效</Flag>
                <Flag ok={gaitStatus?.estimatorStateName === "LOCKED"}>已锁定</Flag>
                <Flag ok={Boolean(gaitFlags?.motionDetected)}>检测到行走</Flag>
                <Flag ok={Boolean(gaitFlags?.leftPhaseValid)}>LEFT 相位</Flag>
                <Flag ok={Boolean(gaitFlags?.rightPhaseValid)}>RIGHT 相位</Flag>
                {gaitFlags?.leftPhaseWrap && <Flag ok>LEFT 本帧回绕</Flag>}
                {gaitFlags?.rightPhaseWrap && <Flag ok>RIGHT 本帧回绕</Flag>}
                {Boolean(gaitFlags?.unknown) && <Flag ok={false}>未知步态位</Flag>}
              </div>
              <div className="exo-quality-flags" aria-label="步态质量原因">
                {!gaitStatus ? (
                  <Flag ok={false}>等待质量快照</Flag>
                ) : gaitStatus.qualityFlags === 0 ? (
                  <Flag ok>无质量告警</Flag>
                ) : (
                  <>
                    {gaitQuality?.leftInputInvalid && <Flag ok={false}>左输入无效</Flag>}
                    {gaitQuality?.rightInputInvalid && <Flag ok={false}>右输入无效</Flag>}
                    {gaitQuality?.lowMotion && <Flag ok={false}>运动幅值低</Flag>}
                    {gaitQuality?.leftWaitingEvent && <Flag ok={false}>左侧等待事件</Flag>}
                    {gaitQuality?.rightWaitingEvent && <Flag ok={false}>右侧等待事件</Flag>}
                    {gaitQuality?.leftPeriodOutOfRange && <Flag ok={false}>左周期越界</Flag>}
                    {gaitQuality?.rightPeriodOutOfRange && <Flag ok={false}>右周期越界</Flag>}
                    {gaitQuality?.periodUnstable && <Flag ok={false}>周期不稳</Flag>}
                    {gaitQuality?.pairSkewHigh && <Flag ok={false}>左右偏差大</Flag>}
                    {gaitQuality?.timeOrSequenceRollback && <Flag ok={false}>时间或序号回退</Flag>}
                    {gaitQuality?.coordinateReset && <Flag ok={false}>坐标复位</Flag>}
                    {gaitQuality?.leftPhaseTimeout && <Flag ok={false}>左相位超时</Flag>}
                    {gaitQuality?.rightPhaseTimeout && <Flag ok={false}>右相位超时</Flag>}
                    {gaitQuality?.singleSideOnly && <Flag ok={false}>仅单侧有效</Flag>}
                    {gaitQuality?.bilateralInconsistent && (
                      <Flag ok={false}>左右周期/相位不一致</Flag>
                    )}
                    {Boolean(gaitQuality?.unknown) && <Flag ok={false}>未知质量位</Flag>}
                  </>
                )}
              </div>
              <p className="exo-operation-note">
                相位来自髋关节周期代理事件，不等同于足跟着地或足尖离地；自动轮询约 4 Hz，仅用于验收显示。
              </p>
            </>
          )}
        </article>
        </section>
        <GaitHistory hidden={activePanel !== "records"} connected={connected} info={info} system={system}
          left={leftJoint} right={rightJoint} gait={gaitStatus} assist={assistConfig} reference={controlConfig?.selectedReference}
          controlMode={controlConfig?.selectedMode} humanWorn={humanWornMode}/>
        <section className="exo-panel" hidden={activePanel !== "records"} aria-label="原始数据导出">
          <details className="exo-advanced"><summary>本次连接的原始样本与通信日志</summary><article className="exo-card exo-sampling-card">
          <div className="exo-card-heading">
            <div><p className="exo-kicker">阶段9兼容</p><h2>低频轮询与关节样本</h2></div>
            <label className="exo-toggle-row">
              <input type="checkbox" checked={autoPolling} disabled={!connected} onChange={(event) => setAutoPolling(event.target.checked)} />
              <span />自动轮询
            </label>
          </div>
          <div className="exo-data-grid">
            <div><span>当前轮次</span><b>{currentRunId || "—"}</b></div>
            <div><span>已完成轮次</span><b>{completedRuns}</b></div>
            <div><span>LEFT 样本</span><b>{runStats.leftCount}</b></div>
            <div><span>RIGHT 样本</span><b>{runStats.rightCount}</b></div>
            <div><span>LEFT 跨度</span><b>{runStats.leftSpan === null ? "—" : `${runStats.leftSpan.toFixed(2)}°`}</b></div>
            <div><span>RIGHT 跨度</span><b>{runStats.rightSpan === null ? "—" : `${runStats.rightSpan.toFixed(2)}°`}</b></div>
          </div>
          <p className="exo-operation-note">
            轮询为严格串行，每轮查询 SYSTEM、LEFT、RIGHT；固件声明 GAIT_STATUS 时再按需查询步态。静态助力配置不会每轮读取。CSV 中同时保存原始 mrad、度值和 feedback age。
          </p>
          <div className="exo-export-row">
            <button className="exo-button ghost" disabled={!samples.length} onClick={exportSamples}>导出关节 CSV</button>
            <button className="exo-button ghost" disabled={!logs.length} onClick={exportTransactions}>导出 BLE JSONL</button>
            <button className="exo-button ghost" disabled={!samples.length} onClick={() => setSamples([])}>清空样本</button>
          </div>
        </article> <section className="exo-card exo-log-card">
        <div className="exo-card-heading">
          <div><p className="exo-kicker">原始证据</p><h2>BLE 事务日志（内存最多 {MAX_LOG_ENTRIES} 条）</h2></div>
          <div className="exo-log-actions">
            <span className={`exo-log-follow-state ${followLatestLog ? "active" : "paused"}`}>
              {followLatestLog ? "正在跟随最新" : "已暂停跟随"}
            </span>
            <button
              className="exo-button ghost"
              onClick={followLatestLog
                ? () => updateLogFollowState(false)
                : resumeLogFollow}
            >
              {followLatestLog ? "暂停跟随" : "回到最新"}
            </button>
            <button className="exo-button ghost" disabled={!logs.length} onClick={clearProtocolLogs}>清空日志</button>
          </div>
        </div>
        <p className="exo-log-help">显示最近 {MAX_VISIBLE_LOG_ENTRIES} 条，完整内存日志可导出 BLE JSONL。向上滚动会自动暂停跟随；日志仍继续记录。</p>
        <div
          className="exo-log"
          ref={logContainerRef}
          onScroll={handleLogScroll}
          aria-live="polite"
        >
          {logs.length === 0 ? (
            <p className="exo-log-empty">连接设备后，这里会记录每个 TX、Notify、解析结果和超时。</p>
          ) : logs.slice(-MAX_VISIBLE_LOG_ENTRIES).map((entry) => (
            <article className={`exo-log-entry ${entry.direction}`} key={entry.id}>
              <time>{entry.isoTime.slice(11, 23)}</time>
              <span>{entry.direction.toUpperCase()}</span>
              <div><p>{entry.message}</p>{entry.bytes && <code>{toHex(entry.bytes)}</code>}</div>
              <small>{entry.bytes ? `${entry.bytes.length} B` : ""}</small>
            </article>
          ))}
        </div>
      </section></details>
        </section>
        <footer className="exo-footer"><span>人体先运动，外骨骼依据反馈估计步态后助力</span><Link href="/">工程通信终端</Link></footer>
      </div>
    </main>
  );
}
