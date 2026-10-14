#!/usr/bin/env python3
"""阶段 10.1 主动测试激励真机验收工具。

这个脚本刻意把两条因果链分开：

* 产品链：人体运动 -> 编码器反馈 -> 步态估计 -> ACTIVE_PHASE 助力；
* 测试链：ALTERNATING_SINE_TEST 主动带动空载/台架 -> 编码器反馈 ->
  步态估计 -> SHADOW 建议与遥测。

测试链只能证明反馈、步态估计、SHADOW 建议和遥测能够首尾相接，不能替代
穿戴者主动行走条件下的 ACTIVE 验收。默认硬件运行只读状态，绝不会 START。

真正允许运动必须同时给出三道门：

    --allow-motion
    --case alternating-sine-test
    --confirm-motion "ESP-EXO PHASE10 ALTERNATING SINE MOTION"

任何已通过三道门的硬件流程都会在 ``finally`` 中请求整机 STOP；脚本不会归零，
也不会执行 RS01 机械置零。
"""

from __future__ import annotations

import argparse
import asyncio
from dataclasses import dataclass
import math
import struct
import sys
import time
from typing import Any, Sequence

try:
    from .phase6_ble_acceptance import (
        AcceptanceError,
        BleTransport,
        DEFAULT_DEVICE_NAME,
        OP_GET_INFO,
        OP_GET_JOINT_STATUS,
        OP_GET_SYSTEM_STATUS,
        OP_START,
        OP_STOP,
        PROTOCOL_VERSION,
        RESPONSE_CHARACTERISTIC_UUID,
        RESULT_ACCEPTED,
        RESULT_ALREADY_PENDING,
        RESULT_ALREADY_SATISFIED,
        RESULT_OK,
        Reporter,
        decode_frame,
        encode_request,
        parse_info,
        parse_joint_status,
        parse_system_status,
        require,
        require_correlated,
        resolve_device,
    )
    from .phase9_ble_acceptance import (
        ASSIST_EXEC_SHADOW,
        ASSIST_MODE_ACTIVE_PHASE,
        ASSIST_STRATEGY_PHASE_PROFILE,
        OP_GET_ASSIST_CONFIG,
        OP_GET_GAIT_STATUS,
        OP_SET_ASSIST_CONFIG,
        make_assist_set_payload,
        parse_assist_status,
        parse_gait_status,
    )
    from .phase10_ble_acceptance import (
        CHANNEL_APPLIED_TORQUE,
        CHANNEL_ASSIST_SUGGESTION,
        CHANNEL_GAIT_CONFIDENCE,
        CHANNEL_GAIT_PHASE,
        CHANNEL_POSITION,
        FLAG_OUTPUT_ALLOWED,
        FLAG_REAL_OUTPUT_ENABLED,
        OP_GET_TELEMETRY_STATUS,
        OP_START_TELEMETRY,
        OP_STOP_TELEMETRY,
        TELEMETRY_CHARACTERISTIC_UUID,
        TelemetryPacket,
        make_start_payload,
        parse_telemetry_packet,
        parse_telemetry_status,
        validate_sample_groups,
    )
except ImportError:
    from phase6_ble_acceptance import (  # type: ignore[no-redef]
        AcceptanceError,
        BleTransport,
        DEFAULT_DEVICE_NAME,
        OP_GET_INFO,
        OP_GET_JOINT_STATUS,
        OP_GET_SYSTEM_STATUS,
        OP_START,
        OP_STOP,
        PROTOCOL_VERSION,
        RESPONSE_CHARACTERISTIC_UUID,
        RESULT_ACCEPTED,
        RESULT_ALREADY_PENDING,
        RESULT_ALREADY_SATISFIED,
        RESULT_OK,
        Reporter,
        decode_frame,
        encode_request,
        parse_info,
        parse_joint_status,
        parse_system_status,
        require,
        require_correlated,
        resolve_device,
    )
    from phase9_ble_acceptance import (  # type: ignore[no-redef]
        ASSIST_EXEC_SHADOW,
        ASSIST_MODE_ACTIVE_PHASE,
        ASSIST_STRATEGY_PHASE_PROFILE,
        OP_GET_ASSIST_CONFIG,
        OP_GET_GAIT_STATUS,
        OP_SET_ASSIST_CONFIG,
        make_assist_set_payload,
        parse_assist_status,
        parse_gait_status,
    )
    from phase10_ble_acceptance import (  # type: ignore[no-redef]
        CHANNEL_APPLIED_TORQUE,
        CHANNEL_ASSIST_SUGGESTION,
        CHANNEL_GAIT_CONFIDENCE,
        CHANNEL_GAIT_PHASE,
        CHANNEL_POSITION,
        FLAG_OUTPUT_ALLOWED,
        FLAG_REAL_OUTPUT_ENABLED,
        OP_GET_TELEMETRY_STATUS,
        OP_START_TELEMETRY,
        OP_STOP_TELEMETRY,
        TELEMETRY_CHARACTERISTIC_UUID,
        TelemetryPacket,
        make_start_payload,
        parse_telemetry_packet,
        parse_telemetry_status,
        validate_sample_groups,
    )


EXPECTED_FIRMWARE = (0, 10, 1)
PRODUCT_CAPABILITIES = 0x007F
ENGINEERING_STIMULUS_CAPABILITY = 0x0080
KNOWN_0101_CAPABILITIES = PRODUCT_CAPABILITIES | ENGINEERING_STIMULUS_CAPABILITY
MOTION_CONFIRMATION = "ESP-EXO PHASE10 ALTERNATING SINE MOTION"
MOTION_CASE = "alternating-sine-test"

OP_GET_CONTROL_CONFIG = 0x04
OP_GET_JOINT_COMMAND = 0x05
OP_SET_CONTROL_CONFIG = 0x14
OP_SET_JOINT_COMMAND = 0x15

SYSTEM_STATE_IDLE = 1
SYSTEM_STATE_RUNNING = 3
CONTROL_MODE_IMPEDANCE = 6
REFERENCE_ALTERNATING_SINE_TEST = 2
SIDE_LEFT = 0
SIDE_RIGHT = 1

# 0.10.1 的工程激励参数是固件常量，而不是手机任意可调参数。
TEST_AMPLITUDE_DEG = 10.0
TEST_FREQUENCY_HZ = 0.5
TEST_RAMP_S = 3.0
TEST_PHASE_SEPARATION = 0.5

MOTION_CHANNEL_MASK = (
    CHANNEL_POSITION
    | CHANNEL_GAIT_PHASE
    | CHANNEL_ASSIST_SUGGESTION
    | CHANNEL_GAIT_CONFIDENCE
    | CHANNEL_APPLIED_TORQUE
)


@dataclass(frozen=True)
class ControlConfig:
    selected_mode: int
    active_mode: int
    selected_reference: int
    active_reference: int
    flags: int


@dataclass(frozen=True)
class MotionMetrics:
    complete_samples: int
    left_range_deg: float
    right_range_deg: float
    bilateral_correlation: float
    phase_valid_fraction: float
    confident_fraction: float
    nonzero_suggestion_fraction: float


class RequestIds:
    def __init__(self) -> None:
        self._next = 1

    def take(self) -> int:
        require(self._next <= 0xFF, "request_id sequence exhausted")
        value = self._next
        self._next += 1
        return value


def validate_motion_gate(args: argparse.Namespace) -> None:
    """在导入 bleak 和连接设备之前拒绝任何不完整的运动授权。"""

    if args.case is None:
        require(not args.allow_motion, "--allow-motion requires an explicit --case")
        require(
            args.confirm_motion is None,
            "--confirm-motion is invalid without an explicit --case",
        )
        return
    require(args.allow_motion, "motion case requires --allow-motion")
    require(args.case == MOTION_CASE, f"unsupported motion case: {args.case}")
    require(
        args.confirm_motion == MOTION_CONFIRMATION,
        f"--confirm-motion must exactly equal: {MOTION_CONFIRMATION!r}",
    )
    require(12.0 <= args.duration <= 60.0, "motion duration must be 12..60 s")
    require(0.0 < args.kp <= 5.0, "test kp must be in (0,5]")
    require(0.0 < args.kd <= 1.0, "test kd must be in (0,1]")


def parse_control_config(payload: bytes) -> ControlConfig:
    require(len(payload) == 5, "CONTROL_CONFIG payload must be 5 bytes")
    mode, active_mode, reference, active_reference, flags = payload
    require((flags & ~0x03) == 0, "CONTROL_CONFIG contains unknown flags")
    return ControlConfig(mode, active_mode, reference, active_reference, flags)


def make_joint_command_payload(side: int, kp: float, kd: float) -> bytes:
    require(side in (SIDE_LEFT, SIDE_RIGHT), "invalid joint side")
    kp_x100 = round(kp * 100.0)
    kd_x1000 = round(kd * 1000.0)
    require(0 <= kp_x100 <= 50000, "kp cannot be encoded")
    require(0 <= kd_x1000 <= 5000, "kd cannot be encoded")
    # position/velocity/torque 全为零；工程参考源自己产生固定 10 deg 目标。
    return struct.pack("<Bii hHH", side, 0, 0, 0, kp_x100, kd_x1000)


def validate_0101_identity(info: Any, *, require_engineering: bool) -> None:
    require(info.firmware == EXPECTED_FIRMWARE, f"firmware={info.firmware}, expected={EXPECTED_FIRMWARE}")
    require(
        (info.capabilities & PRODUCT_CAPABILITIES) == PRODUCT_CAPABILITIES,
        f"missing product capabilities: 0x{info.capabilities:04X}",
    )
    require(
        (info.capabilities & ~KNOWN_0101_CAPABILITIES) == 0,
        f"unknown 0.10.1 capabilities: 0x{info.capabilities:04X}",
    )
    if require_engineering:
        require(
            info.capabilities & ENGINEERING_STIMULUS_CAPABILITY,
            "firmware does not advertise the engineering stimulus capability",
        )


def _correlation(left: Sequence[float], right: Sequence[float]) -> float:
    require(len(left) == len(right) and len(left) >= 4, "not enough paired samples")
    left_mean = sum(left) / len(left)
    right_mean = sum(right) / len(right)
    dl = [value - left_mean for value in left]
    dr = [value - right_mean for value in right]
    denominator = math.sqrt(sum(value * value for value in dl) * sum(value * value for value in dr))
    require(denominator > 1e-9, "joint signals have no measurable variation")
    return sum(a * b for a, b in zip(dl, dr)) / denominator


def analyze_motion_packets(packets: Sequence[TelemetryPacket]) -> MotionMetrics:
    expected_channels = {
        CHANNEL_POSITION,
        CHANNEL_GAIT_PHASE,
        CHANNEL_ASSIST_SUGGESTION,
        CHANNEL_GAIT_CONFIDENCE,
        CHANNEL_APPLIED_TORQUE,
    }
    complete = validate_sample_groups(packets, expected_channels)
    by_sample: dict[int, dict[int, TelemetryPacket]] = {}
    for packet in packets:
        by_sample.setdefault(packet.sample_sequence, {})[packet.channel] = packet
    groups = [group for _, group in sorted(by_sample.items()) if set(group) == expected_channels]
    require(len(groups) >= 100, "too few complete motion samples")

    # 忽略 3 s 渐入和开始边界，以免把预期的低幅启动误判为轨迹失败。
    steady = groups[max(1, round(len(groups) * 0.25)) :]
    left_deg: list[float] = []
    right_deg: list[float] = []
    phase_valid = 0
    confident = 0
    suggestion_nonzero = 0
    for group in steady:
        position = group[CHANNEL_POSITION]
        require(position.left_raw is not None and position.right_raw is not None, "position invalid")
        left_deg.append(position.left_raw * 180.0 / (math.pi * 1000.0))
        right_deg.append(position.right_raw * 180.0 / (math.pi * 1000.0))

        phase = group[CHANNEL_GAIT_PHASE]
        if phase.left_raw is not None and phase.right_raw is not None:
            phase_valid += 1
        confidence = group[CHANNEL_GAIT_CONFIDENCE]
        if (
            confidence.left_raw is not None
            and confidence.right_raw is not None
            and min(confidence.left_raw, confidence.right_raw) >= 4000
        ):
            confident += 1
        suggestion = group[CHANNEL_ASSIST_SUGGESTION]
        if (
            suggestion.left_raw is not None
            and suggestion.right_raw is not None
            and (abs(suggestion.left_raw) >= 1 or abs(suggestion.right_raw) >= 1)
        ):
            suggestion_nonzero += 1

        applied = group[CHANNEL_APPLIED_TORQUE]
        require(applied.left_raw == 0 and applied.right_raw == 0, "SHADOW applied nonzero torque")
        for packet in group.values():
            require(packet.system_state == SYSTEM_STATE_RUNNING, "sample is not RUNNING")
            require(packet.flags & FLAG_OUTPUT_ALLOWED, "output gate was not open")
            require(not (packet.flags & FLAG_REAL_OUTPUT_ENABLED), "SHADOW reported real assist output")

    metrics = MotionMetrics(
        complete_samples=complete,
        left_range_deg=max(left_deg) - min(left_deg),
        right_range_deg=max(right_deg) - min(right_deg),
        bilateral_correlation=_correlation(left_deg, right_deg),
        phase_valid_fraction=phase_valid / len(steady),
        confident_fraction=confident / len(steady),
        nonzero_suggestion_fraction=suggestion_nonzero / len(steady),
    )
    # 目标为 +/-10 deg，因此理想峰峰值为20 deg；真实台架允许跟踪误差，但必须
    # 明显超过步态估计器的6 deg最小摆幅，并保持左右反相。
    require(10.0 <= metrics.left_range_deg <= 30.0, f"left range={metrics.left_range_deg:.2f} deg")
    require(10.0 <= metrics.right_range_deg <= 30.0, f"right range={metrics.right_range_deg:.2f} deg")
    require(metrics.bilateral_correlation <= -0.70, f"bilateral correlation={metrics.bilateral_correlation:.3f}")
    require(metrics.phase_valid_fraction >= 0.50, f"phase valid={metrics.phase_valid_fraction:.1%}")
    require(metrics.confident_fraction >= 0.25, f"confidence>=0.4 only {metrics.confident_fraction:.1%}")
    require(metrics.nonzero_suggestion_fraction >= 0.25, f"nonzero suggestion only {metrics.nonzero_suggestion_fraction:.1%}")
    return metrics


async def _transact(transport: BleTransport, ids: RequestIds, opcode: int, payload: bytes = b"") -> bytes:
    request_id = ids.take()
    raw = await transport.transact(encode_request(opcode, request_id, payload))
    frame = decode_frame(raw)
    require_correlated(frame, opcode, request_id)
    return frame.payload


async def _expect_ack(
    transport: BleTransport,
    ids: RequestIds,
    opcode: int,
    payload: bytes = b"",
    accepted: set[int] | None = None,
) -> None:
    accepted = {RESULT_OK} if accepted is None else accepted
    response = await _transact(transport, ids, opcode, payload)
    require(len(response) == 4, "ACK payload must be 4 bytes")
    require(response[0] in accepted, f"opcode 0x{opcode:02X} result=0x{response[0]:02X}")


async def _wait_state(
    transport: BleTransport, ids: RequestIds, wanted: int, timeout_s: float
) -> None:
    deadline = time.monotonic() + timeout_s
    last_state = -1
    while time.monotonic() < deadline:
        payload = await _transact(transport, ids, OP_GET_SYSTEM_STATUS)
        frame = decode_frame(
            bytes((PROTOCOL_VERSION, OP_GET_SYSTEM_STATUS | 0x80, 1, len(payload))) + payload
        )
        # parse_system_status只依赖payload，但仍接收完整Frame；这里构造本地只读Frame。
        status = parse_system_status(frame)
        last_state = status.state
        if status.state == wanted:
            return
        await asyncio.sleep(0.1)
    raise AcceptanceError(f"system state timeout: wanted={wanted}, last={last_state}")


async def run_read_only(transport: BleTransport, reporter: Reporter, ids: RequestIds) -> None:
    info_payload = await _transact(transport, ids, OP_GET_INFO)
    info_frame = decode_frame(bytes((PROTOCOL_VERSION, OP_GET_INFO | 0x80, 1, len(info_payload))) + info_payload)
    info = parse_info(info_frame)
    validate_0101_identity(info, require_engineering=False)
    reporter.passed_test("0.10.1 identity", f"caps=0x{info.capabilities:04X}")

    system_payload = await _transact(transport, ids, OP_GET_SYSTEM_STATUS)
    system = parse_system_status(decode_frame(bytes((1, 0x82, 1, len(system_payload))) + system_payload))
    reporter.passed_test("read-only system status", f"state={system.state} flags=0x{system.flags:02X}")
    for side, name in ((SIDE_LEFT, "LEFT"), (SIDE_RIGHT, "RIGHT")):
        payload = await _transact(transport, ids, OP_GET_JOINT_STATUS, bytes((side,)))
        joint = parse_joint_status(decode_frame(bytes((1, 0x83, 1, len(payload))) + payload))
        reporter.passed_test(f"read-only {name} joint", f"position={joint.position_deg:.2f} deg flags=0x{joint.flags:02X}")

    control = parse_control_config(await _transact(transport, ids, OP_GET_CONTROL_CONFIG))
    reporter.passed_test("read-only control config", f"mode={control.selected_mode} reference={control.selected_reference}")
    assist_payload = await _transact(transport, ids, OP_GET_ASSIST_CONFIG)
    assist = parse_assist_status(decode_frame(bytes((1, 0x87, 1, len(assist_payload))) + assist_payload))
    reporter.passed_test("read-only assist config", f"requested={assist.requested_execution} effective={assist.effective_execution}")


async def run_motion(
    transport: BleTransport,
    telemetry_queue: asyncio.Queue[bytes],
    reporter: Reporter,
    ids: RequestIds,
    args: argparse.Namespace,
) -> None:
    """执行唯一获准的主动激励用例；调用者负责finally STOP。"""

    info_payload = await _transact(transport, ids, OP_GET_INFO)
    info = parse_info(decode_frame(bytes((1, 0x81, 1, len(info_payload))) + info_payload))
    validate_0101_identity(info, require_engineering=True)
    status_payload = await _transact(transport, ids, OP_GET_SYSTEM_STATUS)
    system = parse_system_status(decode_frame(bytes((1, 0x82, 1, len(status_payload))) + status_payload))
    require(system.state == SYSTEM_STATE_IDLE, f"motion precondition requires IDLE, state={system.state}")
    require(system.flags & 0x02, "system is not ready to START")
    for side, name in ((SIDE_LEFT, "LEFT"), (SIDE_RIGHT, "RIGHT")):
        payload = await _transact(transport, ids, OP_GET_JOINT_STATUS, bytes((side,)))
        joint = parse_joint_status(decode_frame(bytes((1, 0x83, 1, len(payload))) + payload))
        require((joint.flags & 0x3F) == 0x3F, f"{name} joint flags=0x{joint.flags:02X}")
        require(abs(joint.position_deg) <= args.neutral_tolerance_deg, f"{name} is not neutral: {joint.position_deg:.2f} deg")
    reporter.passed_test("motion preconditions", "IDLE, ready, both joints valid and neutral")

    # 测试参考必须与SHADOW组合；工具没有任何写入ACTIVE的代码路径。
    await _expect_ack(
        transport,
        ids,
        OP_SET_CONTROL_CONFIG,
        bytes((CONTROL_MODE_IMPEDANCE, REFERENCE_ALTERNATING_SINE_TEST)),
    )
    await _expect_ack(transport, ids, OP_SET_JOINT_COMMAND, make_joint_command_payload(SIDE_LEFT, args.kp, args.kd))
    await _expect_ack(transport, ids, OP_SET_JOINT_COMMAND, make_joint_command_payload(SIDE_RIGHT, args.kp, args.kd))
    shadow_payload = make_assist_set_payload(
        mode=ASSIST_MODE_ACTIVE_PHASE,
        execution=ASSIST_EXEC_SHADOW,
        strategy=ASSIST_STRATEGY_PHASE_PROFILE,
        profile_id=1,
        left_level_percent=10,
        right_level_percent=10,
    )
    await _expect_ack(transport, ids, OP_SET_ASSIST_CONFIG, shadow_payload)
    reporter.passed_test("test configuration", "IMPEDANCE + ALTERNATING_SINE_TEST; assist=SHADOW")

    # Windows Web Bluetooth/Bleak 常见协商 MTU=23。五个通道在20 Hz时会形成
    # 约100次Notify/s，可能耗尽NimBLE mbuf并在STOP事务前制造资源拥塞；0.5 Hz
    # 轨迹用10 Hz采样已有20点/周期，足够判断幅值、反相、相位和助力建议。
    await _expect_ack(
        transport,
        ids,
        OP_START_TELEMETRY,
        make_start_payload(10, MOTION_CHANNEL_MASK),
    )
    await _expect_ack(transport, ids, OP_START, accepted={RESULT_ACCEPTED})
    await _wait_state(transport, ids, SYSTEM_STATE_RUNNING, 8.0)
    reporter.passed_test("START reached RUNNING")

    packets: list[TelemetryPacket] = []
    deadline = time.monotonic() + args.duration
    while time.monotonic() < deadline:
        try:
            raw = await asyncio.wait_for(telemetry_queue.get(), timeout=0.5)
        except asyncio.TimeoutError:
            continue
        packets.append(parse_telemetry_packet(raw))
    require(packets, "no telemetry received during motion")
    metrics = analyze_motion_packets(packets)
    reporter.passed_test(
        "alternating stimulus -> feedback -> gait -> SHADOW telemetry",
        f"samples={metrics.complete_samples} range=({metrics.left_range_deg:.1f},"
        f"{metrics.right_range_deg:.1f})deg corr={metrics.bilateral_correlation:.3f} "
        f"phase={metrics.phase_valid_fraction:.0%} suggestion={metrics.nonzero_suggestion_fraction:.0%}",
    )

    # 不在任意相位直接STOP。继续读取最多3秒，等左右反馈同时经过产品既有的
    # 中立位容差范围，
    # 再交给finally停机；这样工程测试不会把失能后的机构留在摆幅端点。
    neutral_deadline = time.monotonic() + 3.0
    while time.monotonic() < neutral_deadline:
        try:
            packet = parse_telemetry_packet(
                await asyncio.wait_for(telemetry_queue.get(), timeout=0.5)
            )
        except asyncio.TimeoutError:
            continue
        if (
            packet.channel == CHANNEL_POSITION
            and packet.left_raw is not None
            and packet.right_raw is not None
        ):
            left_deg = packet.left_raw * 180.0 / (math.pi * 1000.0)
            right_deg = packet.right_raw * 180.0 / (math.pi * 1000.0)
            if (
                abs(left_deg) <= args.neutral_tolerance_deg
                and abs(right_deg) <= args.neutral_tolerance_deg
            ):
                reporter.passed_test(
                    "neutral crossing before STOP",
                    f"LEFT={left_deg:.2f} deg RIGHT={right_deg:.2f} deg",
                )
                return
    raise AcceptanceError(
        "trajectory did not cross bilateral neutral tolerance before STOP"
    )


async def restore_product_defaults(
    transport: BleTransport, reporter: Reporter, ids: RequestIds
) -> None:
    """STOP到IDLE后恢复不会自行产生轨迹的产品默认选择并逐项读回。"""

    await _expect_ack(
        transport,
        ids,
        OP_SET_CONTROL_CONFIG,
        bytes((5, 0)),  # DAMPING + CONSTANT
    )
    expected_left = make_joint_command_payload(SIDE_LEFT, 0.0, 0.1)
    expected_right = make_joint_command_payload(SIDE_RIGHT, 0.0, 0.1)
    await _expect_ack(transport, ids, OP_SET_JOINT_COMMAND, expected_left)
    await _expect_ack(transport, ids, OP_SET_JOINT_COMMAND, expected_right)
    await _expect_ack(
        transport,
        ids,
        OP_SET_ASSIST_CONFIG,
        make_assist_set_payload(
            mode=1,
            execution=0,
            strategy=1,
            profile_id=0,
            left_level_percent=0,
            right_level_percent=0,
        ),
    )

    control = parse_control_config(await _transact(transport, ids, OP_GET_CONTROL_CONFIG))
    require(
        control.selected_mode == 5 and control.selected_reference == 0,
        "DAMPING + CONSTANT readback mismatch",
    )
    require(
        await _transact(transport, ids, OP_GET_JOINT_COMMAND, bytes((SIDE_LEFT,)))
        == expected_left,
        "LEFT product joint command readback mismatch",
    )
    require(
        await _transact(transport, ids, OP_GET_JOINT_COMMAND, bytes((SIDE_RIGHT,)))
        == expected_right,
        "RIGHT product joint command readback mismatch",
    )
    assist_payload = await _transact(transport, ids, OP_GET_ASSIST_CONFIG)
    assist = parse_assist_status(decode_frame(bytes((1, 0x87, 1, len(assist_payload))) + assist_payload))
    require(
        assist.selected_mode == 1
        and assist.requested_execution == 0
        and assist.strategy_id == 1
        and assist.profile_id == 0
        and assist.left_level_percent == 0
        and assist.right_level_percent == 0,
        "TRANSPARENT + DISABLED readback mismatch",
    )
    reporter.passed_test("restore product defaults", "DAMPING+CONSTANT, kd=0.1, assist disabled")


async def run_hardware(args: argparse.Namespace, reporter: Reporter) -> None:
    try:
        from bleak import BleakClient, BleakScanner
    except ImportError as error:
        raise AcceptanceError("hardware run requires bleak") from error

    device = await resolve_device(BleakScanner, args.address, args.name, args.scan_timeout)
    async with BleakClient(device, timeout=args.connect_timeout) as client:
        require(client.is_connected, "BLE connection failed")
        reporter.passed_test("BLE connect")
        transport = BleTransport(client, args.response_timeout)
        telemetry_queue: asyncio.Queue[bytes] = asyncio.Queue()

        def on_telemetry(_sender: Any, data: bytearray) -> None:
            telemetry_queue.put_nowait(bytes(data))

        await client.start_notify(RESPONSE_CHARACTERISTIC_UUID, transport.notification_callback)
        telemetry_notifying = False
        motion_authorized = args.case is not None
        ids = RequestIds()
        try:
            if motion_authorized:
                await client.start_notify(TELEMETRY_CHARACTERISTIC_UUID, on_telemetry)
                telemetry_notifying = True
                await run_motion(transport, telemetry_queue, reporter, ids, args)
            else:
                await run_read_only(transport, reporter, ids)
        finally:
            if motion_authorized:
                # 不依赖“START ACK是否收到”来决定清理：Write可能已执行而Notify恰好丢失。
                try:
                    await _expect_ack(
                        transport,
                        ids,
                        OP_STOP,
                        accepted={RESULT_ACCEPTED, RESULT_ALREADY_PENDING, RESULT_ALREADY_SATISFIED},
                    )
                    await _wait_state(transport, ids, SYSTEM_STATE_IDLE, 10.0)
                    reporter.passed_test("finally STOP", "system=IDLE")
                except BaseException as error:
                    reporter.failed_test("finally STOP", str(error))
                try:
                    await restore_product_defaults(transport, reporter, ids)
                except BaseException as error:
                    reporter.failed_test("restore product defaults", str(error))
                try:
                    await _expect_ack(
                        transport,
                        ids,
                        OP_STOP_TELEMETRY,
                        accepted={RESULT_OK, RESULT_ALREADY_SATISFIED},
                    )
                except BaseException as error:
                    reporter.failed_test("STOP_TELEMETRY cleanup", str(error))
            if telemetry_notifying:
                await client.stop_notify(TELEMETRY_CHARACTERISTIC_UUID)
            await client.stop_notify(RESPONSE_CHARACTERISTIC_UUID)


def run_self_test(reporter: Reporter) -> None:
    parser = build_argument_parser()
    for argv in (
        ["--allow-motion"],
        ["--case", MOTION_CASE],
        ["--allow-motion", "--case", MOTION_CASE, "--confirm-motion", "wrong"],
    ):
        args = parser.parse_args(argv)
        try:
            validate_motion_gate(args)
        except AcceptanceError:
            continue
        raise AcceptanceError(f"unsafe arguments were accepted: {argv}")
    reporter.passed_test("three independent motion gates")

    args = parser.parse_args([])
    validate_motion_gate(args)
    reporter.passed_test("default invocation is read-only")

    left: list[float] = []
    right: list[float] = []
    for tick in range(1000):
        elapsed_s = tick * 0.01
        ramp_x = min(1.0, elapsed_s / TEST_RAMP_S)
        ramp = ramp_x**3 * (10.0 + ramp_x * (-15.0 + 6.0 * ramp_x))
        angle = ramp * TEST_AMPLITUDE_DEG * math.sin(2.0 * math.pi * TEST_FREQUENCY_HZ * elapsed_s)
        left.append(angle)
        right.append(-angle)
    require(max(abs(a + b) for a, b in zip(left, right)) < 1e-9, "oracle is not anti-phase")
    require(abs(_correlation(left[300:], right[300:]) + 1.0) < 1e-9, "oracle correlation")
    require(max(left) <= 10.0 and min(left) >= -10.0, "oracle amplitude")
    require(
        PRODUCT_CAPABILITIES == 0x007F
        and ENGINEERING_STIMULUS_CAPABILITY == 0x0080
        and KNOWN_0101_CAPABILITIES == 0x00FF,
        "0.10.1 capability contract drift",
    )
    defaults = parser.parse_args([])
    require(defaults.kp == 5.0 and defaults.kd == 0.1, "motion gain defaults drift")
    require(
        make_joint_command_payload(SIDE_LEFT, defaults.kp, defaults.kd)
        == bytes.fromhex("00 00 00 00 00 00 00 00 00 00 00 F4 01 64 00"),
        "5.0/0.1 gain payload drift",
    )
    reporter.passed_test("10 deg / 0.5 Hz / 3 s anti-phase oracle")


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="ESP-EXO 0.10.1 gated motion acceptance")
    parser.add_argument("--self-test", action="store_true")
    target = parser.add_mutually_exclusive_group()
    target.add_argument("--address")
    target.add_argument("--name", default=DEFAULT_DEVICE_NAME)
    parser.add_argument("--scan-timeout", type=float, default=10.0)
    parser.add_argument("--connect-timeout", type=float, default=10.0)
    parser.add_argument("--response-timeout", type=float, default=3.0)
    parser.add_argument("--allow-motion", action="store_true")
    parser.add_argument("--case", choices=(MOTION_CASE,))
    parser.add_argument("--confirm-motion")
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--neutral-tolerance-deg", type=float, default=5.0)
    parser.add_argument("--kp", type=float, default=5.0)
    parser.add_argument("--kd", type=float, default=0.1)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_argument_parser()
    args = parser.parse_args(argv)
    reporter = Reporter()
    try:
        validate_motion_gate(args)
        if args.self_test:
            require(args.case is None, "--self-test cannot be combined with a motion case")
            run_self_test(reporter)
        else:
            asyncio.run(run_hardware(args, reporter))
    except (AcceptanceError, OSError, asyncio.TimeoutError) as error:
        reporter.failed_test("acceptance", str(error))
    return reporter.summary()


if __name__ == "__main__":
    sys.exit(main())
