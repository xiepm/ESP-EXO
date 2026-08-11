"use client";

import {
  type AckResponse,
  AssistExecutionState,
  AssistMode,
  AssistUnavailableReason,
  type AssistConfigResponse,
  Capability,
  ControlMode,
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
  assertCorrelated,
  assistExecutionStateNames,
  assistModeNames,
  blockReasonNames,
  controlModeNames,
  encodeControlConfigPayload,
  encodeAssistConfigPayload,
  encodeJointCommandPayload,
  encodeRequest,
  parseResponse,
  referenceSourceNames,
  toHex,
} from "@/lib/exo-ble-protocol";
import { connectGattWithOneRetry } from "@/lib/exo-ble-connect";
import {
  COMPATIBLE_FIRMWARE_VERSIONS,
  PREFERRED_FIRMWARE_VERSION,
  assessDeviceCompatibility,
} from "@/lib/exo-device-compatibility";
import { isLogNearBottom } from "@/lib/exo-log-scroll";
import Link from "next/link";
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
  kp: "0.2",
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
const MAX_SAMPLES = 10000;

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

function responseSummary(response: ExoBleResponse): string {
  if (response.kind === "ack") {
    const detail = response.requestOpcode === Opcode.START
      ? `，detail=${blockReasonNames[response.detail] ?? response.detail}`
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
    // 阶段 9 页面故意不生成 ACTIVE 请求。即使未来固件返回 ACTIVE，
    // 表单也降为 SHADOW，避免一次“读回后再写入”意外请求真实输出。
    executionState: response.requestedExecutionState === AssistExecutionState.ACTIVE
      ? AssistExecutionState.SHADOW
      : response.requestedExecutionState,
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
  if (config.unavailableReason === AssistUnavailableReason.REQUIRED_GAIT_INVALID) {
    return "所需步态无效";
  }
  return config.unavailableReasonName;
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
      <p className="exo-expected">期望映射：{expected}</p>
      <div className="exo-joint-value">
        <strong>{joint ? joint.positionDegrees.toFixed(2) : "—"}</strong>
        <span>deg</span>
      </div>
      <div className="exo-data-grid compact">
        <div><span>速度</span><b>{joint ? `${joint.velocityDegreesPerSecond.toFixed(2)} deg/s` : "—"}</b></div>
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

export default function ExoAcceptancePage() {
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
  const [controlConfig, setControlConfig] =
    useState<ControlConfigResponse | null>(null);
  const [leftCommand, setLeftCommand] =
    useState<JointCommandResponse | null>(null);
  const [rightCommand, setRightCommand] =
    useState<JointCommandResponse | null>(null);
  const [gaitStatus, setGaitStatus] = useState<GaitStatusResponse | null>(null);
  const [assistConfig, setAssistConfig] =
    useState<AssistConfigResponse | null>(null);
  const [selectedMode, setSelectedMode] = useState<number>(ControlMode.IMPEDANCE);
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
  const connectedRef = useRef(false);
  const pendingRef = useRef(new Map<number, PendingRequest>());
  const nextRequestIdRef = useRef(1);
  const queueRef = useRef<Promise<void>>(Promise.resolve());
  const writeQueueRef = useRef<Promise<void>>(Promise.resolve());
  const connectionGenerationRef = useRef(0);
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
    connectedRef.current = false;
    connectionGenerationRef.current += 1;
    deviceRef.current = null;
    commandRef.current = null;
    responseRef.current = null;
    serverRef.current = null;
    systemRef.current = null;
    infoRef.current = null;
    pollBusyRef.current = false;
    nextRequestIdRef.current = 1;
    setAutoPolling(false);
    setNeutralConfirmed(false);
    setConnection("idle");
    setConnectStage("idle");
    setLastFailedConnectStage(null);
    // 断开后不能把最后一次快照继续显示成“当前状态”。日志和采样仍保留，
    // 重连后再重新读取设备身份、系统状态和双侧关节。
    setInfo(null);
    setSystem(null);
    setLeftJoint(null);
    setRightJoint(null);
    setControlConfig(null);
    setLeftCommand(null);
    setRightCommand(null);
    setGaitStatus(null);
    setAssistConfig(null);
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

  const applyResponse = useCallback((response: ExoBleResponse) => {
    if (response.kind === "info") {
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
    if (response.kind === "controlConfig") {
      setControlConfig(response);
      if (response.selectedValid) {
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
      if (response.decodedAssistFlags.selectedValid) {
        setAssistConfigDraft(assistDraftFromResponse(response));
      }
      return;
    }

    if (response.side === Side.LEFT) setLeftJoint(response);
    else setRightJoint(response);

    const flags = response.decodedJointFlags;
    if (!flags.feedbackValid || !flags.online) {
      setNeutralConfirmed(false);
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
      if (device?.gatt?.connected) device.gatt.disconnect();
      deviceRef.current = null;
      commandRef.current = null;
      responseRef.current = null;
      serverRef.current = null;
    };
  }, [handleDisconnected, handleNotification]);

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
    if (!connectedRef.current) return Promise.reject(new Error("BLE 尚未连接"));
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
  }, [allocateRequestId, appendLog, writeFrame]);

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

  const queryAssistConfig = useCallback(async (): Promise<AssistConfigResponse> => {
    return responseOfKind(
      await transact(
        Opcode.GET_ASSIST_CONFIG,
        undefined,
        "GET_ASSIST_CONFIG",
      ),
      "assistConfig",
    );
  }, [transact]);

  const refreshAllCore = useCallback(async (): Promise<void> => {
    const identity = await queryInfo();
    await querySystem();
    await queryJoint(Side.LEFT);
    await queryJoint(Side.RIGHT);
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
  }, [
    queryAssistConfig,
    queryControlConfig,
    queryGaitStatus,
    queryInfo,
    queryJoint,
    queryJointCommand,
    querySystem,
  ]);

  async function runOperation(label: string, action: () => Promise<void>) {
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
  }

  /**
   * 使用一个已经由用户授权的 BluetoothDevice 完成完整产品握手。
   * stage 是局部变量，因为 React 状态更新是异步的；catch 必须准确知道
   * 本次到底失败在哪一步，不能读取可能仍落后一步的 connectStage。
   */
  async function establishConnection(device: BluetoothDeviceLike) {
    let stage: ConnectStage = "gatt";
    let subscribedResponse: BluetoothRemoteGATTCharacteristicLike | null = null;

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
      const server = await connectGattWithOneRetry(
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
      );
      serverRef.current = server;

      stage = "service";
      setConnectStage(stage);
      appendLog("system", "GATT 已连接，正在发现 Product Service…");
      const service = await server.getPrimaryService(EXO_BLE.SERVICE_UUID);

      stage = "characteristics";
      setConnectStage(stage);
      appendLog("system", "Product Service 已发现，正在读取 Command / Response 特征…");
      const [command, response] = await Promise.all([
        service.getCharacteristic(EXO_BLE.COMMAND_CHARACTERISTIC_UUID),
        service.getCharacteristic(EXO_BLE.RESPONSE_CHARACTERISTIC_UUID),
      ]);

      connectionGenerationRef.current += 1;
      commandRef.current = command;
      responseRef.current = response;
      response.addEventListener("characteristicvaluechanged", handleNotification);
      subscribedResponse = response;

      stage = "notify";
      setConnectStage(stage);
      appendLog("system", "产品特征已就绪，正在订阅 Response Notify…");
      await response.startNotifications();

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
        await refreshAllCore();
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
      if (subscribedResponse) {
        subscribedResponse.removeEventListener(
          "characteristicvaluechanged",
          handleNotification,
        );
        try {
          await subscribedResponse.stopNotifications?.();
        } catch {
          // 建连失败时连接可能已经消失，继续释放 GATT 即可。
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

      rememberedDeviceRef.current = device;
      setHasRememberedDevice(true);
      setDeviceName(device.name ?? EXO_BLE.DEVICE_NAME);
      await establishConnection(device);
    } catch (caught) {
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

  async function captureZero(side: number): Promise<void> {
    if (!neutralConfirmed) throw new Error("请先确认双髋已处于中立位");
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
    await querySystem();
  }

  async function captureBothZeros(): Promise<void> {
    await captureZero(Side.LEFT);
    await captureZero(Side.RIGHT);
    await queryJoint(Side.LEFT);
    await queryJoint(Side.RIGHT);
    await querySystem();
  }

  async function clearBothZeros(): Promise<void> {
    const confirmed = window.confirm("确定清除左右髋的 RAM 软件零位吗？清除后不能 START。");
    if (!confirmed) return;
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

  async function writeControlConfig(): Promise<ControlConfigResponse> {
    const payload = encodeControlConfigPayload(selectedMode, selectedReference);
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
    if (!verified.selectedValid || verified.selectedMode !== selectedMode ||
        verified.selectedReference !== selectedReference) {
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

  async function writeAssistConfig(): Promise<AssistConfigResponse> {
    if (systemRef.current?.state !== 0x01) {
      throw new Error("助力配置只能在 IDLE 写入；请先 STOP 并等待 IDLE");
    }
    if (assistConfigDraft.executionState === AssistExecutionState.ACTIVE) {
      throw new Error("阶段 9 页面不允许请求 ACTIVE，只能选择 DISABLED 或 SHADOW");
    }
    const requested = {
      selectedMode: assistConfigDraft.selectedMode,
      requestedExecutionState: assistConfigDraft.executionState,
      strategyId: integerDraftNumber(
        assistConfigDraft.strategyId,
        "策略 ID",
      ),
      profileId: integerDraftNumber(
        assistConfigDraft.profileId,
        "曲线 ID",
      ),
      leftLevelPercent: integerDraftNumber(
        assistConfigDraft.leftLevelPercent,
        "LEFT 助力等级",
        0,
        100,
      ),
      rightLevelPercent: integerDraftNumber(
        assistConfigDraft.rightLevelPercent,
        "RIGHT 助力等级",
        0,
        100,
      ),
    };
    const ack = responseOfKind(
      await transact(
        Opcode.SET_ASSIST_CONFIG,
        encodeAssistConfigPayload(requested),
        "SET_ASSIST_CONFIG",
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
    return verified;
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
        ? blockReasonNames[ack.detail] ?? `detail=${ack.detail}`
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
      if (((infoRef.current?.capabilities ?? 0) & Capability.GAIT_STATUS) !== 0) {
        await queryGaitStatus();
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
  }, [appendLog, queryGaitStatus, queryJoint, querySystem]);

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
  const canEditControlConfiguration = connected && stage7InfoAccepted &&
    hasControlConfiguration && !operation && system?.state === 0x01;
  const canApplyControlConfiguration = connected && stage7InfoAccepted &&
    hasControlConfiguration && !operation;
  const canEditAssistConfiguration = connected && stage7InfoAccepted &&
    hasAssistConfiguration && !operation && system?.state === 0x01;
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
  const canConfirmNeutral = connected && stage7InfoAccepted && !operation &&
    system?.state === 0x01 && bothFeedbackReady;
  const canCaptureLeftZero = canConfirmNeutral && neutralConfirmed &&
    leftFeedbackReady;
  const canCaptureRightZero = canConfirmNeutral && neutralConfirmed &&
    rightFeedbackReady;
  const canCaptureBothZeros = canCaptureLeftZero && canCaptureRightZero;
  const canStart = connected && !operation && system?.state === 0x01 &&
    stage7InfoAccepted && Boolean(systemFlags?.readyToStart) && bothJointsValid;
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
        : !bothFeedbackReady
          ? "等待左右电机主动反馈在线；当前不能用无效反馈归零。"
          : !neutralConfirmed
            ? "双侧反馈已在线；确认机械中立位并勾选后即可归零。"
            : !bothJointsValid
              ? "可以执行归零；双侧零位都生效后 START 才会解锁。"
              : systemFlags?.readyToStart
                ? "归零和关节状态有效，START 已就绪。"
                : `等待系统可启动；当前阻止原因 ${system?.startBlockReasonName ?? "未知"}。`;

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
    <main className="exo-page">
      <header className="exo-topbar">
        <div>
          <p className="exo-kicker">ESP-EXO · 阶段 9</p>
          <h1>真机 BLE 验收台</h1>
          <p>
            Product Service v1 · 兼容固件 {COMPATIBLE_FIRMWARE_VERSIONS.join(" / ")} ·
            推荐 {PREFERRED_FIRMWARE_VERSION} · 人体关节显示单位为度
          </p>
        </div>
        <div className={`exo-connection ${connection}`}>
          <span />
          {connectionBadgeText}
        </div>
      </header>

      {!browserReady && bleSupported !== null && secureContext !== null && (
        <section className="exo-alert" role="alert">
          <strong>当前页面不能使用 Web Bluetooth。</strong>
          <span>
            请在 Windows Chrome/Edge 中通过 localhost 打开；手机必须使用 Android Chrome 和可信 HTTPS。iPhone Safari 不支持 Web Bluetooth。
          </span>
        </section>
      )}

      <section className="exo-hero exo-card">
        <div>
          <p className="exo-kicker">第一步</p>
          <h2>连接产品 BLE，不再手工发送 HEX</h2>
          <p>
            页面会自动发现正确的 Product Service，先订阅 Response Notify，再以带响应方式写入完整二进制帧。每次轮询都会使用新的 request_id。
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

      <section className="exo-overview-grid">
        <article className="exo-card exo-device-card">
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
        </article>

        <article className="exo-card exo-system-card">
          <div className="exo-card-heading">
            <div><p className="exo-kicker">整机生命周期</p><h2>系统状态</h2></div>
            <span className={`exo-state-chip state-${system?.state ?? "unknown"}`}>{system?.stateName ?? "未知"}</span>
          </div>
          <div className="exo-data-grid">
            <div><span>活动步骤</span><b>{system?.activeStepName ?? "—"}</b></div>
            <div><span>START 阻止</span><b>{system?.startBlockReasonName ?? "—"}</b></div>
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
        </article>
      </section>

      <section className="exo-joint-grid">
        <JointCard title="LEFT 左髋" expected="执行器 B / +1" joint={leftJoint} />
        <JointCard title="RIGHT 右髋" expected="执行器 A / −1" joint={rightJoint} />
      </section>

      <section className="exo-stage9-grid">
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

        <article className="exo-card exo-assist-card">
          <div className="exo-card-heading">
            <div>
              <p className="exo-kicker">阶段 9 · 高层策略</p>
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
                      strategyId: defaultStrategyIdForMode(
                        Number(event.target.value),
                      ),
                    }))}
                  >
                    {[AssistMode.TRANSPARENT, AssistMode.ACTIVE_PHASE, AssistMode.ASSIST_AS_NEEDED]
                      .map((mode) => <option key={mode} value={mode}>{assistModeNames[mode]}</option>)}
                  </select>
                </label>
                <label>
                  <span>阶段 9 请求执行</span>
                  <select
                    value={assistConfigDraft.executionState}
                    disabled={!canEditAssistConfiguration}
                    onChange={(event) => setAssistConfigDraft((draft) => ({
                      ...draft,
                      executionState: Number(event.target.value),
                    }))}
                  >
                    {[AssistExecutionState.DISABLED, AssistExecutionState.SHADOW]
                      .map((state) => <option key={state} value={state}>{assistExecutionStateNames[state]}</option>)}
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
                <Flag ok={!Boolean(assistFlags?.realOutputEnabled)}>
                  {assistFlags?.realOutputEnabled
                    ? "异常：真实助力已开启"
                    : "真实输出关闭"}
                </Flag>
                {Boolean(assistFlags?.unknown) && <Flag ok={false}>未知助力位</Flag>}
              </div>

              <div className="exo-assist-actions">
                <p className="exo-shadow-note">
                  <strong>SHADOW 只计算建议命令，不改变真实 CAN 输出。</strong>
                  阶段 9 页面不会提供 ACTIVE 选项；配置只能在 IDLE 写入。
                </p>
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
                    disabled={!canEditAssistConfiguration}
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
        </article>
      </section>

      <section className="exo-card exo-control-card">
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
      </section>

      <section className="exo-work-grid">
        <article className="exo-card exo-actions-card">
          <div className="exo-card-heading">
            <div><p className="exo-kicker">操作链</p><h2>归零、启动与停止</h2></div>
            <span className={`exo-state-chip ${operation ? "working" : "muted"}`}>{operation || "等待操作"}</span>
          </div>

          <label className="exo-confirm-row">
            <input
              type="checkbox"
              checked={neutralConfirmed}
              disabled={!canConfirmNeutral}
              onChange={(event) => setNeutralConfirmed(event.target.checked)}
            />
            <span>我已确认双髋当前处于中立位</span>
          </label>

          <div className="exo-action-grid">
            <button className="exo-button secondary" disabled={!canCaptureLeftZero} onClick={() => void runOperation("左髋归零", () => captureZero(Side.LEFT))}>左髋归零</button>
            <button className="exo-button secondary" disabled={!canCaptureRightZero} onClick={() => void runOperation("右髋归零", () => captureZero(Side.RIGHT))}>右髋归零</button>
            <button className="exo-button secondary wide" disabled={!canCaptureBothZeros} onClick={() => void runOperation("双髋依次归零", captureBothZeros)}>双髋依次归零</button>
            <button className="exo-button start wide" disabled={!canStart} onClick={() => void runOperation("START 到 RUNNING", startExoskeleton)}>START 并等待 RUNNING</button>
            <button className="exo-button stop wide" disabled={!canStop} onClick={() => void runStopOperation()}>STOP 并等待 IDLE</button>
            <button className="exo-button danger wide" disabled={!connected || !stage7InfoAccepted || Boolean(operation) || system?.state !== 0x01} onClick={() => void runOperation("清除双侧零位", clearBothZeros)}>清除双侧 RAM 零位</button>
          </div>

          <p className="exo-operation-note">
            当前门控：{actionReadinessText}<br />
            START/STOP 的 ACK 只表示状态机接单；本页会继续轮询到 RUNNING 或 IDLE 才报告完成。BLE 断开不会自动 STOP。
          </p>
        </article>

        <article className="exo-card exo-sampling-card">
          <div className="exo-card-heading">
            <div><p className="exo-kicker">运行数据</p><h2>轮询与样本</h2></div>
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
        </article>
      </section>

      <section className="exo-card exo-log-card">
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
        <p className="exo-log-help">向上滚动会自动暂停跟随；日志仍继续记录。滚回底部或点击“回到最新”即可恢复。</p>
        <div
          className="exo-log"
          ref={logContainerRef}
          onScroll={handleLogScroll}
          aria-live="polite"
        >
          {logs.length === 0 ? (
            <p className="exo-log-empty">连接设备后，这里会记录每个 TX、Notify、解析结果和超时。</p>
          ) : logs.map((entry) => (
            <article className={`exo-log-entry ${entry.direction}`} key={entry.id}>
              <time>{entry.isoTime.slice(11, 23)}</time>
              <span>{entry.direction.toUpperCase()}</span>
              <div><p>{entry.message}</p>{entry.bytes && <code>{toHex(entry.bytes)}</code>}</div>
              <small>{entry.bytes ? `${entry.bytes.length} B` : ""}</small>
            </article>
          ))}
        </div>
      </section>

      <footer className="exo-footer">
        <span>Product Service v1</span>
        <span>阶段 9：传感快照 · 步态估计 · 助力策略 SHADOW</span>
        <Link href="/">返回原工程通信终端</Link>
      </footer>
    </main>
  );
}
