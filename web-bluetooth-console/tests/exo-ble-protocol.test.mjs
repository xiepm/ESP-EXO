import assert from "node:assert/strict";
import test from "node:test";

import {
  AssistExecutionState,
  AssistMode,
  AssistStrategy,
  AssistUnavailableReason,
  Capability,
  ControlMode,
  EXO_BLE,
  ExoBleProtocolError,
  GaitEstimator,
  GaitEstimatorState,
  GaitPhaseOrigin,
  Opcode,
  ReferenceSource,
  Side,
  assertCorrelated,
  blockReasonNames,
  decodeJointFlags,
  decodeSystemFlags,
  directionNames,
  encodeRequest,
  encodeControlConfigPayload,
  encodeAssistConfigPayload,
  encodeJointCommandPayload,
  mradToDegrees,
  parseResponse,
  resultNames,
  stateNames,
  stepNames,
  toHex,
} from "../lib/exo-ble-protocol.ts";

function fromHex(value) {
  const compact = value.replace(/\s+/g, "");
  assert.equal(compact.length % 2, 0, "test vector must contain whole bytes");
  return Uint8Array.from(
    compact.match(/.{2}/g) ?? [],
    (pair) => Number.parseInt(pair, 16),
  );
}

function expectProtocolError(action, code) {
  assert.throws(action, (error) => {
    assert.ok(error instanceof ExoBleProtocolError);
    assert.equal(error.code, code);
    return true;
  });
}

test("exports the product BLE v1 constants", () => {
  assert.deepEqual(EXO_BLE, {
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
  assert.equal(Opcode.START, 0x10);
  assert.deepEqual(Side, { LEFT: 0, RIGHT: 1 });
  assert.equal(Capability.CONTROL_CONFIGURATION, 1 << 3);
  assert.equal(Capability.GAIT_STATUS, 1 << 4);
  assert.equal(Capability.ASSIST_CONFIGURATION, 1 << 5);
  assert.equal(Opcode.GET_GAIT_STATUS, 0x06);
  assert.equal(Opcode.GET_ASSIST_CONFIG, 0x07);
  assert.equal(Opcode.SET_ASSIST_CONFIG, 0x16);
  assert.equal(ControlMode.IMPEDANCE, 6);
  assert.equal(ReferenceSource.SINE, 1);
});

test("encodes every published request as exactly one v1 frame", () => {
  const vectors = [
    [Opcode.GET_INFO, 0x2a, undefined, "01 01 2A 00"],
    [Opcode.GET_SYSTEM_STATUS, 0x2b, undefined, "01 02 2B 00"],
    [Opcode.GET_JOINT_STATUS, 0x2c, Uint8Array.of(Side.LEFT), "01 03 2C 01 00"],
    [Opcode.GET_JOINT_STATUS, 0x2d, Uint8Array.of(Side.RIGHT), "01 03 2D 01 01"],
    [Opcode.GET_CONTROL_CONFIG, 0x34, undefined, "01 04 34 00"],
    [Opcode.GET_JOINT_COMMAND, 0x35, Uint8Array.of(Side.LEFT), "01 05 35 01 00"],
    [Opcode.GET_GAIT_STATUS, 0x38, undefined, "01 06 38 00"],
    [Opcode.GET_ASSIST_CONFIG, 0x39, undefined, "01 07 39 00"],
    [Opcode.START, 0x2e, undefined, "01 10 2E 00"],
    [Opcode.STOP, 0x2f, undefined, "01 11 2F 00"],
    [Opcode.CAPTURE_ZERO, 0x30, Uint8Array.of(Side.LEFT), "01 12 30 01 00"],
    [Opcode.CAPTURE_ZERO, 0x31, Uint8Array.of(Side.RIGHT), "01 12 31 01 01"],
    [Opcode.CLEAR_ZERO, 0x32, Uint8Array.of(Side.LEFT), "01 13 32 01 00"],
    [Opcode.CLEAR_ZERO, 0x33, Uint8Array.of(Side.RIGHT), "01 13 33 01 01"],
    [
      Opcode.SET_CONTROL_CONFIG,
      0x36,
      encodeControlConfigPayload(ControlMode.IMPEDANCE, ReferenceSource.SINE),
      "01 14 36 02 06 01",
    ],
    [
      Opcode.SET_JOINT_COMMAND,
      0x37,
      encodeJointCommandPayload({
        side: Side.LEFT,
        positionDegrees: 90,
        velocityDegreesPerSecond: 0,
        torqueNm: 1.5,
        kp: 0.2,
        kd: 0.1,
      }),
      "01 15 37 0F 00 23 06 00 00 00 00 00 00 DC 05 14 00 64 00",
    ],
    [
      Opcode.SET_ASSIST_CONFIG,
      0x44,
      encodeAssistConfigPayload({
        selectedMode: AssistMode.ACTIVE_PHASE,
        requestedExecutionState: AssistExecutionState.SHADOW,
        strategyId: AssistStrategy.PHASE_PROFILE,
        profileId: 7,
        leftLevelPercent: 30,
        rightLevelPercent: 40,
      }),
      "01 16 44 08 01 02 01 02 07 1E 28 00",
    ],
  ];

  for (const [opcode, requestId, payload, expected] of vectors) {
    assert.equal(toHex(encodeRequest(opcode, requestId, payload)), expected);
  }
});

test("encodeRequest copies BufferSource payloads and enforces request schema", () => {
  const backing = Uint8Array.of(0xaa, Side.RIGHT, 0xbb);
  const payload = new DataView(backing.buffer, 1, 1);
  const frame = encodeRequest(Opcode.GET_JOINT_STATUS, 1, payload);
  backing[1] = Side.LEFT;
  assert.equal(toHex(frame), "01 03 01 01 01");

  expectProtocolError(() => encodeRequest(Opcode.GET_INFO, 0), "INVALID_ARGUMENT");
  expectProtocolError(() => encodeRequest(Opcode.GET_INFO, 0x100), "INVALID_ARGUMENT");
  expectProtocolError(() => encodeRequest(0x7e, 1), "UNKNOWN_OPCODE");
  expectProtocolError(() => encodeRequest(0x81, 1), "UNKNOWN_OPCODE");
  expectProtocolError(
    () => encodeRequest(Opcode.GET_INFO, 1, Uint8Array.of(0)),
    "INVALID_PAYLOAD",
  );
  expectProtocolError(
    () => encodeRequest(Opcode.GET_JOINT_STATUS, 1),
    "INVALID_PAYLOAD",
  );
  expectProtocolError(
    () => encodeRequest(Opcode.GET_JOINT_STATUS, 1, Uint8Array.of(2)),
    "INVALID_PAYLOAD",
  );
  expectProtocolError(
    () => encodeRequest(Opcode.GET_INFO, 1, new Uint8Array(17)),
    "INVALID_PAYLOAD",
  );
});

test("parses the Python INFO golden frame and all common fields", () => {
  const source = fromHex(
    "01 81 2A 0C 01 01 00 06 00 07 00 14 78 56 34 12",
  );
  const response = parseResponse(source);
  source[0] = 0xff;

  assert.equal(response.kind, "info");
  assert.equal(response.version, 1);
  assert.equal(response.opcode, 0x81);
  assert.equal(response.requestOpcode, Opcode.GET_INFO);
  assert.equal(response.requestId, 0x2a);
  assert.equal(response.payloadLength, 12);
  assert.equal(response.raw[0], 1, "parser must retain its own byte copy");
  assert.equal(response.protocolMinVersion, 1);
  assert.equal(response.protocolMaxVersion, 1);
  assert.deepEqual(response.firmware, [0, 6, 0]);
  assert.equal(response.capabilities, 0x0007);
  assert.equal(response.maxFrameSize, 20);
  assert.equal(response.bootId, 0x12345678);
  assert.doesNotThrow(() => assertCorrelated(response, Opcode.GET_INFO, 0x2a));
});

test("parses the current 0.7.1 INFO layout", () => {
  const response = parseResponse(
    fromHex("01 81 01 0C 01 01 00 07 01 07 00 14 78 56 34 12"),
  );
  assert.equal(response.kind, "info");
  assert.deepEqual(response.firmware, [0, 7, 1]);
});

test("preserves unknown INFO capability bits for forward compatibility", () => {
  const response = parseResponse(
    fromHex("01 81 01 0C 01 01 00 09 00 3F 80 14 78 56 34 12"),
  );
  assert.equal(response.kind, "info");
  assert.equal(response.capabilities, 0x803f);
  assert.equal(response.unknownCapabilities, 0x8000);
});

test("parses the Python SYSTEM_STATUS golden frame as little-endian", () => {
  const response = parseResponse(
    fromHex("01 82 2B 0A 03 07 2F 00 01 00 78 56 34 12"),
  );
  assert.equal(response.kind, "systemStatus");
  assert.equal(response.requestOpcode, Opcode.GET_SYSTEM_STATUS);
  assert.equal(response.state, 3);
  assert.equal(response.stateName, "RUNNING");
  assert.equal(response.activeStep, 7);
  assert.equal(response.activeStepName, "ARM_CONTROLLER");
  assert.equal(response.systemFlags, 0x2f);
  assert.equal(response.decodedSystemFlags.initialized, true);
  assert.equal(response.decodedSystemFlags.readyToStart, true);
  assert.equal(response.decodedSystemFlags.runRequested, true);
  assert.equal(response.decodedSystemFlags.outputAllowed, true);
  assert.equal(response.decodedSystemFlags.maintenanceActive, false);
  assert.equal(response.decodedSystemFlags.executorAlive, true);
  assert.equal(response.startBlockReason, 0);
  assert.equal(response.lastRequestResult, 1);
  assert.equal(response.lastFailedStep, 0);
  assert.equal(response.transitionCount, 0x12345678);
});

test("parses signed JOINT_STATUS values and converts mrad to degrees", () => {
  const raw = fromHex(
    "01 83 2C 0D 00 3F 02 2E FB FF FF 2E 16 00 00 2A 00",
  );
  const padded = new Uint8Array(raw.length + 2);
  padded.set(raw, 1);
  const response = parseResponse(new DataView(padded.buffer, 1, raw.length));

  assert.equal(response.kind, "jointStatus");
  assert.equal(response.side, Side.LEFT);
  assert.equal(response.jointFlags, 0x3f);
  assert.deepEqual(response.decodedJointFlags, {
    raw: 0x3f,
    mappingValid: true,
    directionValid: true,
    zeroValid: true,
    feedbackValid: true,
    online: true,
    jointValid: true,
  });
  assert.equal(response.direction, 2);
  assert.equal(response.directionName, "NEGATIVE");
  assert.equal(response.positionMrad, -1234);
  assert.equal(response.velocityMradS, 5678);
  assert.equal(response.feedbackAgeMs, 42);
  assert.ok(
    Math.abs(response.positionDegrees - (-70.70299191914359)) < 1e-12,
  );
  assert.ok(
    Math.abs(response.velocityDegreesPerSecond - 325.32543607528146) < 1e-12,
  );
});

test("encodes and parses stage 8 control configuration without ACK ambiguity", () => {
  assert.equal(
    toHex(encodeControlConfigPayload(ControlMode.POSITION, ReferenceSource.CONSTANT)),
    "02 00",
  );

  const selected = parseResponse(fromHex("01 84 40 05 06 00 01 00 01"));
  assert.equal(selected.kind, "controlConfig");
  assert.equal(selected.selectedModeName, "IMPEDANCE");
  assert.equal(selected.activeModeName, "NONE");
  assert.equal(selected.selectedReferenceName, "SINE");
  assert.equal(selected.selectedValid, true);
  assert.equal(selected.activeValid, false);

  const ack = parseResponse(fromHex("01 84 40 04 11 01 23 00"));
  assert.equal(ack.kind, "ack");
  assert.equal(ack.resultName, "REJECTED_BUSY");

  expectProtocolError(
    () => encodeControlConfigPayload(ControlMode.NONE, ReferenceSource.CONSTANT),
    "INVALID_ARGUMENT",
  );
  expectProtocolError(
    () => encodeControlConfigPayload(ControlMode.HOLD, 2),
    "INVALID_FIELD",
  );
  expectProtocolError(
    () => encodeControlConfigPayload(ControlMode.HOLD, ReferenceSource.SINE),
    "INVALID_ARGUMENT",
  );
});

test("encodes and parses one side five-element command with degree display units", () => {
  const payload = encodeJointCommandPayload({
    side: Side.RIGHT,
    positionDegrees: -90,
    velocityDegreesPerSecond: 180,
    torqueNm: -1.5,
    kp: 0.2,
    kd: 0.1,
  });
  assert.equal(
    toHex(payload),
    "01 DD F9 FF FF 46 0C 00 00 24 FA 14 00 64 00",
  );

  const response = parseResponse(fromHex(
    "01 85 41 0F 01 DD F9 FF FF 46 0C 00 00 24 FA 14 00 64 00",
  ));
  assert.equal(response.kind, "jointCommand");
  assert.equal(response.side, Side.RIGHT);
  assert.equal(response.positionMrad, -1571);
  assert.equal(response.velocityMradS, 3142);
  assert.equal(response.torqueMNm, -1500);
  assert.equal(response.kp, 0.2);
  assert.equal(response.kd, 0.1);
  assert.ok(Math.abs(response.positionDegrees - (-90.01166961505233)) < 1e-12);

  expectProtocolError(
    () => encodeJointCommandPayload({
      side: Side.LEFT,
      positionDegrees: 0,
      velocityDegreesPerSecond: 0,
      torqueNm: 17.001,
      kp: 0.2,
      kd: 0.1,
    }),
    "INVALID_ARGUMENT",
  );
});

test("parses the stage 9 gait status golden frame with fixed-point units", () => {
  const response = parseResponse(fromHex(
    "01 86 42 10 01 03 1F 01 84 41 C4 09 4C 1D 11 2B 2E 22 2A 00",
  ));
  assert.equal(response.kind, "gaitStatus");
  assert.equal(response.estimatorId, GaitEstimator.HIP_EVENT_TBE);
  assert.equal(response.estimatorName, "HIP_EVENT_TBE");
  assert.equal(response.estimatorState, GaitEstimatorState.LOCKED);
  assert.equal(response.phaseOrigin, GaitPhaseOrigin.HIP_FLEXION_PEAK);
  assert.equal(response.leftPhase01, 0.25);
  assert.equal(response.rightPhasePercent, 75);
  assert.equal(response.cadenceSpm, 110.25);
  assert.equal(response.confidence01, 0.875);
  assert.equal(response.estimateAgeMs, 42);
  assert.equal(response.decodedGaitFlags.leftPhaseWrap, true);
  assert.equal(response.decodedGaitFlags.rightPhaseWrap, false);
  assert.equal(response.decodedQualityFlags.lowMotion, true);
  assert.equal(response.decodedQualityFlags.periodUnstable, true);
  assert.equal(response.decodedQualityFlags.pairSkewHigh, true);
  assert.equal(response.decodedQualityFlags.bilateralInconsistent, true);
});

test("accepts one valid side while preserving the overall-invalid distinction", () => {
  const response = parseResponse(fromHex(
    "01 86 45 10 01 02 02 01 00 20 C4 09 FF FF 70 17 88 13 0A 00",
  ));
  assert.equal(response.kind, "gaitStatus");
  assert.equal(response.decodedGaitFlags.overallValid, false);
  assert.equal(response.decodedGaitFlags.leftPhaseValid, true);
  assert.equal(response.decodedGaitFlags.rightPhaseValid, false);
  assert.equal(response.leftPhasePercent, 25);
  assert.equal(response.rightPhasePercent, null);
  assert.equal(response.decodedQualityFlags.singleSideOnly, true);
});

test("maps every unavailable gait fixed-point field to null", () => {
  const response = parseResponse(fromHex(
    "01 86 46 10 01 01 00 00 04 00 FF FF FF FF FF FF FF FF FF FF",
  ));
  assert.equal(response.kind, "gaitStatus");
  assert.equal(response.leftPhase01, null);
  assert.equal(response.rightPhase01, null);
  assert.equal(response.cadenceX100, 0xffff);
  assert.equal(response.cadenceSpm, null);
  assert.equal(response.confidenceX10000, 0xffff);
  assert.equal(response.confidence01, null);
  assert.equal(response.estimateAgeRawMs, 0xffff);
  assert.equal(response.estimateAgeMs, null);
});

test("rejects non-sentinel out-of-range gait scalar values", () => {
  expectProtocolError(
    () => parseResponse(fromHex(
      "01 86 47 10 01 03 07 01 00 00 00 00 00 00 31 75 10 27 00 00",
    )),
    "INVALID_FIELD",
  );
  expectProtocolError(
    () => parseResponse(fromHex(
      "01 86 48 10 01 03 07 01 00 00 00 00 00 00 30 75 11 27 00 00",
    )),
    "INVALID_FIELD",
  );
});

test("parses the stage 9 assist configuration golden frame without ACK ambiguity", () => {
  const response = parseResponse(fromHex(
    "01 87 43 10 01 02 00 01 02 01 07 1E 28 3D 07 09 03 00 0F 00",
  ));
  assert.equal(response.kind, "assistConfig");
  assert.equal(response.schemaVersion, 1);
  assert.equal(response.selectedModeName, "ACTIVE_PHASE");
  assert.equal(response.activeModeName, "NONE");
  assert.equal(response.packedExecutionStates, 0x01);
  assert.equal(response.requestedExecutionState, AssistExecutionState.SHADOW);
  assert.equal(response.requestedExecutionStateName, "SHADOW");
  assert.equal(response.executionState, AssistExecutionState.DISABLED);
  assert.equal(response.executionStateName, "DISABLED");
  assert.equal(response.strategyName, "PHASE_PROFILE");
  assert.equal(response.profileId, 7);
  assert.equal(response.leftLevelPercent, 30);
  assert.equal(response.rightLevelPercent, 40);
  assert.equal(response.decodedAssistFlags.selectedValid, true);
  assert.equal(response.decodedAssistFlags.activeValid, false);
  assert.equal(response.decodedAssistFlags.suggestionValid, true);
  assert.equal(response.unavailableReason, AssistUnavailableReason.REQUIRED_GAIT_INVALID);
  assert.equal(response.unavailableReasonName, "REQUIRED_GAIT_INVALID");
  assert.equal(response.requiredSensorMask, 0x0003);
  assert.equal(response.availableSensorMask, 0x000f);
  assert.equal(response.decodedRequiredSensorMask.leftHipKinematics, true);
  assert.equal(response.decodedAvailableSensorMask.rightMotorTorqueEstimate, true);
});

test("parses the effective execution state from the packed high nibble", () => {
  const response = parseResponse(fromHex(
    "01 87 44 10 01 02 02 11 02 01 07 1E 28 3F 00 09 03 00 0F 00",
  ));
  assert.equal(response.kind, "assistConfig");
  assert.equal(response.packedExecutionStates, 0x11);
  assert.equal(response.requestedExecutionState, AssistExecutionState.SHADOW);
  assert.equal(response.executionState, AssistExecutionState.SHADOW);
  assert.equal(response.decodedAssistFlags.selectedValid, true);
  assert.equal(response.decodedAssistFlags.activeValid, true);
});

test("stage 9 response enums and future flag bits remain observable", () => {
  const gait = parseResponse(fromHex(
    "01 86 51 10 7A 7B 8F 7C 04 80 C4 09 4C 1D 00 00 FF FF FF FF",
  ));
  assert.equal(gait.estimatorName, "UNKNOWN(0x7a)");
  assert.equal(gait.estimatorStateName, "UNKNOWN(0x7b)");
  assert.equal(gait.phaseOriginName, "UNKNOWN(0x7c)");
  assert.equal(gait.decodedGaitFlags.unknown, 0x80);
  assert.equal(gait.decodedQualityFlags.unknown, 0x8000);

  const assist = parseResponse(fromHex(
    "01 87 52 10 01 7A 00 01 7C 01 00 00 00 81 7D 00 00 80 00 00",
  ));
  assert.equal(assist.selectedModeName, "UNKNOWN(0x7a)");
  assert.equal(assist.requestedExecutionStateName, "SHADOW");
  assert.equal(assist.executionStateName, "DISABLED");
  assert.equal(assist.strategyName, "UNKNOWN(0x7c)");
  assert.equal(assist.unavailableReasonName, "UNKNOWN(0x7d)");
  assert.equal(assist.decodedAssistFlags.unknown, 0x80);
  assert.equal(assist.decodedRequiredSensorMask.unknown, 0x8000);
});

test("strictly validates stage 9 SET payloads and response relationships", () => {
  const valid = encodeAssistConfigPayload({
    selectedMode: AssistMode.TRANSPARENT,
    requestedExecutionState: AssistExecutionState.SHADOW,
    strategyId: AssistStrategy.TRANSPARENT,
    profileId: 0,
    leftLevelPercent: 100,
    rightLevelPercent: 0,
  });
  assert.equal(toHex(valid), "01 01 01 01 00 64 00 00");

  expectProtocolError(() => encodeAssistConfigPayload({
    schemaVersion: 2,
    selectedMode: AssistMode.TRANSPARENT,
    requestedExecutionState: AssistExecutionState.SHADOW,
    strategyId: AssistStrategy.TRANSPARENT,
    profileId: 0,
    leftLevelPercent: 0,
    rightLevelPercent: 0,
  }), "INVALID_FIELD");
  expectProtocolError(() => encodeAssistConfigPayload({
    selectedMode: 0x7a,
    requestedExecutionState: AssistExecutionState.SHADOW,
    strategyId: AssistStrategy.TRANSPARENT,
    profileId: 0,
    leftLevelPercent: 0,
    rightLevelPercent: 0,
  }), "INVALID_FIELD");
  expectProtocolError(() => encodeAssistConfigPayload({
    selectedMode: AssistMode.NONE,
    requestedExecutionState: AssistExecutionState.DISABLED,
    strategyId: AssistStrategy.NONE,
    profileId: 0,
    leftLevelPercent: 0,
    rightLevelPercent: 0,
  }), "INVALID_FIELD");
  const activePayload = encodeAssistConfigPayload({
    selectedMode: AssistMode.TRANSPARENT,
    requestedExecutionState: AssistExecutionState.ACTIVE,
    strategyId: AssistStrategy.TRANSPARENT,
    profileId: 0,
    leftLevelPercent: 0,
    rightLevelPercent: 0,
  });
  assert.equal(toHex(activePayload), "01 01 02 01 00 00 00 00");
  assert.equal(
    toHex(encodeRequest(Opcode.SET_ASSIST_CONFIG, 0x53, activePayload)),
    "01 16 53 08 01 01 02 01 00 00 00 00",
  );
  const activeRejected = parseResponse(fromHex(
    "01 96 53 04 10 01 23 03",
  ));
  assert.equal(activeRejected.kind, "ack");
  assert.equal(activeRejected.resultName, "REJECTED_NOT_READY");
  assert.equal(activeRejected.detail, 3);
  assert.doesNotThrow(() => assertCorrelated(
    activeRejected,
    Opcode.SET_ASSIST_CONFIG,
    0x53,
  ));
  expectProtocolError(() => encodeAssistConfigPayload({
    selectedMode: AssistMode.ACTIVE_PHASE,
    requestedExecutionState: AssistExecutionState.SHADOW,
    strategyId: AssistStrategy.TRANSPARENT,
    profileId: 0,
    leftLevelPercent: 0,
    rightLevelPercent: 0,
  }), "INVALID_FIELD");
  expectProtocolError(() => encodeAssistConfigPayload({
    selectedMode: AssistMode.TRANSPARENT,
    requestedExecutionState: AssistExecutionState.SHADOW,
    strategyId: AssistStrategy.TRANSPARENT,
    profileId: 0,
    leftLevelPercent: 101,
    rightLevelPercent: 0,
  }), "INVALID_ARGUMENT");
  expectProtocolError(
    () => encodeRequest(
      Opcode.SET_ASSIST_CONFIG,
      1,
      fromHex("01 01 01 01 00 00 00 01"),
    ),
    "INVALID_PAYLOAD",
  );

  expectProtocolError(
    () => parseResponse(fromHex(
      "01 86 01 10 01 03 01 01 00 00 00 00 FF FF 00 00 00 00 00 00",
    )),
    "INVALID_FIELD",
  );
  expectProtocolError(
    () => parseResponse(fromHex(
      "01 87 01 10 01 01 00 01 01 01 00 65 00 01 00 00 03 00 03 00",
    )),
    "INVALID_FIELD",
  );
  expectProtocolError(
    () => parseResponse(fromHex(
      "01 87 54 10 01 01 00 04 01 01 00 00 00 01 00 01 03 00 0F 00",
    )),
    "INVALID_FIELD",
  );
  expectProtocolError(
    () => parseResponse(fromHex(
      "01 87 55 10 01 01 00 40 01 01 00 00 00 01 00 01 03 00 0F 00",
    )),
    "INVALID_FIELD",
  );
  expectProtocolError(
    () => parseResponse(fromHex(
      "01 87 56 10 01 01 00 03 01 01 00 00 00 01 00 01 03 00 0F 00",
    )),
    "INVALID_FIELD",
  );
  expectProtocolError(
    () => parseResponse(fromHex(
      "01 87 57 10 01 01 00 30 01 01 00 00 00 01 00 01 03 00 0F 00",
    )),
    "INVALID_FIELD",
  );
});

test("recognizes a four-byte GET response payload as an ACK", () => {
  const response = parseResponse(fromHex("01 81 2D 04 25 01 23 00"));
  assert.equal(response.kind, "ack");
  assert.equal(response.requestOpcode, Opcode.GET_INFO);
  assert.equal(response.result, 0x25);
  assert.equal(response.resultName, "REQUEST_ID_CONFLICT");
  assert.equal(response.state, 1);
  assert.equal(response.stateName, "IDLE");
  assert.equal(response.systemFlags, 0x23);
  assert.equal(response.detail, 0);
  assert.doesNotThrow(() => assertCorrelated(response, Opcode.GET_INFO, 0x2d));
});

test("parses protocol-error ACKs for unknown and zero-id raw requests", () => {
  const unknown = parseResponse(fromHex("01 FE 07 04 22 01 23 00"));
  assert.equal(unknown.kind, "ack");
  assert.equal(unknown.requestOpcode, 0x7e);
  assert.equal(unknown.resultName, "UNKNOWN_OPCODE");
  assert.doesNotThrow(() => assertCorrelated(unknown, 0x7e, 7));

  const zeroId = parseResponse(fromHex("01 81 00 04 23 01 23 00"));
  assert.equal(zeroId.resultName, "INVALID_REQUEST_ID");
  assert.doesNotThrow(() => assertCorrelated(zeroId, Opcode.GET_INFO, 0));
});

test("strictly rejects malformed frame envelopes and response shapes", () => {
  const cases = [
    [new Uint8Array(0), "INVALID_FRAME_LENGTH"],
    [fromHex("01 81 01"), "INVALID_FRAME_LENGTH"],
    [new Uint8Array(21), "INVALID_FRAME_LENGTH"],
    [fromHex("02 81 01 00"), "UNSUPPORTED_VERSION"],
    [fromHex("01 01 01 00"), "NOT_A_RESPONSE"],
    [fromHex("01 81 01 0C 01"), "INVALID_FRAME_LENGTH"],
    [fromHex("01 81 01 00 FF"), "INVALID_FRAME_LENGTH"],
    [fromHex("01 81 01 11"), "INVALID_FRAME_LENGTH"],
    [fromHex("01 81 01 05 00 00 00 00 00"), "INVALID_FRAME_LENGTH"],
    [fromHex("01 82 01 09 00 00 00 00 00 00 00 00 00"), "INVALID_FRAME_LENGTH"],
    [fromHex("01 83 01 0C 00 00 00 00 00 00 00 00 00 00 00 00"), "INVALID_FRAME_LENGTH"],
    [fromHex("01 84 01 06 01 00 00 00 01 00"), "INVALID_FRAME_LENGTH"],
    [fromHex("01 85 01 0E 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00"), "INVALID_FRAME_LENGTH"],
    [fromHex("01 FE 01 05 00 00 00 00 00"), "UNKNOWN_RESPONSE"],
  ];

  for (const [raw, code] of cases) {
    expectProtocolError(() => parseResponse(raw), code);
  }
});

test("strictly rejects reserved legacy flag bits", () => {
  expectProtocolError(
    () => parseResponse(fromHex("01 90 01 04 01 02 40 00")),
    "INVALID_FIELD",
  );
  expectProtocolError(
    () => parseResponse(fromHex("01 82 01 0A 01 00 40 00 01 00 00 00 00 00")),
    "INVALID_FIELD",
  );
  expectProtocolError(
    () => parseResponse(fromHex("01 83 01 0D 00 40 01 00 00 00 00 00 00 00 00 00 00")),
    "INVALID_FIELD",
  );
  expectProtocolError(
    () => parseResponse(fromHex("01 84 01 05 01 00 00 00 04")),
    "INVALID_FIELD",
  );
});

test("strictly enforces ACK and INFO encoder invariants", () => {
  expectProtocolError(
    () => parseResponse(fromHex("01 90 01 04 7F 01 23 00")),
    "INVALID_FIELD",
  );
  expectProtocolError(
    () => parseResponse(fromHex("01 81 00 0C 01 01 00 07 01 07 00 14 00 00 00 00")),
    "INVALID_FIELD",
  );
  expectProtocolError(
    () => parseResponse(fromHex("01 81 01 0C 00 01 00 07 01 07 00 14 00 00 00 00")),
    "INVALID_FIELD",
  );
  expectProtocolError(
    () => parseResponse(fromHex("01 81 01 0C 02 02 00 07 01 07 00 14 00 00 00 00")),
    "INVALID_FIELD",
  );
  expectProtocolError(
    () => parseResponse(fromHex("01 81 01 0C 01 00 00 07 01 07 00 14 00 00 00 00")),
    "INVALID_FIELD",
  );
  expectProtocolError(
    () => parseResponse(fromHex("01 81 01 0C 01 01 00 07 01 07 00 13 00 00 00 00")),
    "INVALID_FIELD",
  );
});

test("strictly rejects invalid sides, directions, and stable wire enums", () => {
  expectProtocolError(
    () => parseResponse(fromHex("01 83 01 0D 02 3F 01 00 00 00 00 00 00 00 00 00 00")),
    "INVALID_FIELD",
  );
  expectProtocolError(
    () => parseResponse(fromHex("01 83 01 0D 00 3F 03 00 00 00 00 00 00 00 00 00 00")),
    "INVALID_FIELD",
  );
  expectProtocolError(
    () => parseResponse(fromHex("01 90 01 04 26 01 23 00")),
    "INVALID_FIELD",
  );
  expectProtocolError(
    () => parseResponse(fromHex("01 90 01 04 01 06 23 00")),
    "INVALID_FIELD",
  );
  expectProtocolError(
    () => parseResponse(fromHex("01 82 01 0A 01 0B 23 00 01 00 00 00 00 00")),
    "INVALID_FIELD",
  );
  expectProtocolError(
    () => parseResponse(fromHex("01 82 01 0A 01 00 23 13 01 00 00 00 00 00")),
    "INVALID_FIELD",
  );
});

test("assertCorrelated rejects opcode and request-id mismatches", () => {
  const response = parseResponse(fromHex("01 81 2D 04 25 01 23 00"));
  expectProtocolError(
    () => assertCorrelated(response, Opcode.GET_SYSTEM_STATUS, 0x2d),
    "CORRELATION_MISMATCH",
  );
  expectProtocolError(
    () => assertCorrelated(response, Opcode.GET_INFO, 0x2e),
    "CORRELATION_MISMATCH",
  );
});

test("decodes all published flag bits and rejects reserved bits", () => {
  assert.deepEqual(decodeSystemFlags(0x3f), {
    raw: 0x3f,
    initialized: true,
    readyToStart: true,
    runRequested: true,
    outputAllowed: true,
    maintenanceActive: true,
    executorAlive: true,
  });
  assert.deepEqual(decodeJointFlags(0), {
    raw: 0,
    mappingValid: false,
    directionValid: false,
    zeroValid: false,
    feedbackValid: false,
    online: false,
    jointValid: false,
  });
  expectProtocolError(() => decodeSystemFlags(0x40), "INVALID_FIELD");
  expectProtocolError(() => decodeJointFlags(0x80), "INVALID_FIELD");
});

test("exports stable wire-code names", () => {
  assert.equal(resultNames[0x10], "REJECTED_NOT_READY");
  assert.equal(stateNames[3], "RUNNING");
  assert.equal(stepNames[10], "VERIFY_DISABLED");
  assert.equal(blockReasonNames[10], "LEFT_ZERO_INVALID");
  assert.equal(blockReasonNames[18], "SYSTEM_ERROR");
  assert.equal(directionNames[2], "NEGATIVE");
});

test("formats bytes and converts only finite mrad values", () => {
  const padded = Uint8Array.of(0xaa, 0x01, 0x0a, 0xff, 0xbb);
  assert.equal(toHex(new DataView(padded.buffer, 1, 3)), "01 0A FF");
  assert.equal(mradToDegrees(0), 0);
  assert.ok(Math.abs(mradToDegrees(1000) - 180 / Math.PI) < 1e-12);
  expectProtocolError(() => mradToDegrees(Number.NaN), "INVALID_ARGUMENT");
  expectProtocolError(() => mradToDegrees(Number.POSITIVE_INFINITY), "INVALID_ARGUMENT");
});
