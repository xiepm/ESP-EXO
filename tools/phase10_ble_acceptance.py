#!/usr/bin/env python3
"""ESP-EXO 阶段 10 BLE 遥测验收工具。

默认真机流程只读取 INFO/遥测状态，并启动一个短时的独立 Telemetry 流；它不会
发送外骨骼 START/STOP、归零、控制模式或助力配置命令，因此不会让电机运动。

常用命令：

    python tools/phase10_ble_acceptance.py --self-test
    python tools/phase10_ble_acceptance.py --name ESP-EXO --duration 5

``--self-test`` 是纯主机黄金向量测试，不导入 bleak，也不需要蓝牙适配器。
"""

from __future__ import annotations

import argparse
import asyncio
from dataclasses import dataclass
import math
import struct
import sys
import time
from typing import Any, Callable, Sequence


try:
    from .phase6_ble_acceptance import (
        AcceptanceError,
        BleTransport,
        COMMAND_CHARACTERISTIC_UUID,
        DEFAULT_DEVICE_NAME,
        Frame,
        MAX_FRAME_SIZE,
        OP_GET_INFO,
        PROTOCOL_VERSION,
        RESPONSE_CHARACTERISTIC_UUID,
        RESULT_ALREADY_SATISFIED,
        RESULT_OK,
        Reporter,
        decode_frame,
        encode_request,
        expect_ack,
        parse_info,
        require,
        require_correlated,
        resolve_device,
    )
except ImportError:
    from phase6_ble_acceptance import (  # type: ignore[no-redef]
        AcceptanceError,
        BleTransport,
        COMMAND_CHARACTERISTIC_UUID,
        DEFAULT_DEVICE_NAME,
        Frame,
        MAX_FRAME_SIZE,
        OP_GET_INFO,
        PROTOCOL_VERSION,
        RESPONSE_CHARACTERISTIC_UUID,
        RESULT_ALREADY_SATISFIED,
        RESULT_OK,
        Reporter,
        decode_frame,
        encode_request,
        expect_ack,
        parse_info,
        require,
        require_correlated,
        resolve_device,
    )


TELEMETRY_CHARACTERISTIC_UUID = "f7a10004-6b5a-4d92-9e7c-3a1c8b640001"

OP_GET_TELEMETRY_STATUS = 0x08
OP_START_TELEMETRY = 0x17
OP_STOP_TELEMETRY = 0x18

SUPPORTED_FIRMWARES = {(0, 10, 0), (0, 10, 1), (0, 10, 2), (0, 10, 3)}
PRODUCT_CAPABILITIES = 0x007F
ENGINEERING_STIMULUS_CAPABILITY = 0x0080
KNOWN_0101_CAPABILITIES = PRODUCT_CAPABILITIES | ENGINEERING_STIMULUS_CAPABILITY
CALIBRATION_STATUS_CAPABILITY = 0x0100
KNOWN_0102_CAPABILITIES = KNOWN_0101_CAPABILITIES | CALIBRATION_STATUS_CAPABILITY
ASSIST_TORQUE_LIMIT_CAPABILITY = 0x0200
KNOWN_0103_CAPABILITIES = KNOWN_0102_CAPABILITIES | ASSIST_TORQUE_LIMIT_CAPABILITY
TELEMETRY_CAPABILITY = 1 << 6
TELEMETRY_SCHEMA = 1
TELEMETRY_SAMPLE_TYPE = 1
TELEMETRY_FRAME_SIZE = 20
TELEMETRY_CHANNELS_MASK = 0x007F
TELEMETRY_DEFAULT_RATE_HZ = 20
TELEMETRY_DEFAULT_CHANNEL_MASK = 0x0078
TELEMETRY_RING_CAPACITY = 32

CHANNEL_POSITION = 0x01
CHANNEL_VELOCITY = 0x02
CHANNEL_MOTOR_TORQUE = 0x04
CHANNEL_GAIT_PHASE = 0x08
CHANNEL_ASSIST_SUGGESTION = 0x10
CHANNEL_GAIT_CONFIDENCE = 0x20
CHANNEL_APPLIED_TORQUE = 0x40
UNSIGNED_CHANNELS = {CHANNEL_GAIT_PHASE, CHANNEL_GAIT_CONFIDENCE}

FLAG_LEFT_VALID = 1 << 0
FLAG_RIGHT_VALID = 1 << 1
FLAG_DROPPED = 1 << 2
FLAG_SYSTEM_STATE_MASK = 0x38
FLAG_OUTPUT_ALLOWED = 1 << 6
FLAG_REAL_OUTPUT_ENABLED = 1 << 7
SIGNED_INVALID_BITS = 0x8000
UNSIGNED_INVALID = 0xFFFF


@dataclass(frozen=True)
class TelemetryStatus:
    streaming: bool
    rate_hz: int
    channel_mask: int
    stream_sequence: int
    notify_success_count: int
    dropped_sample_count: int
    queued_samples: int
    queue_capacity: int


@dataclass(frozen=True)
class TelemetryPacket:
    channel: int
    flags: int
    stream_sequence: int
    sample_sequence: int
    timestamp_ms: int
    activation_epoch_lsb: int
    left_raw: int | None
    right_raw: int | None

    @property
    def system_state(self) -> int:
        return (self.flags & FLAG_SYSTEM_STATE_MASK) >> 3

    @property
    def output_allowed(self) -> bool:
        return bool(self.flags & FLAG_OUTPUT_ALLOWED)

    @property
    def real_output_enabled(self) -> bool:
        return bool(self.flags & FLAG_REAL_OUTPUT_ENABLED)


class RequestIds:
    def __init__(self) -> None:
        self._next = 1

    def take(self) -> int:
        require(self._next <= 0xFF, "request_id sequence exhausted")
        value = self._next
        self._next += 1
        return value


def validate_stage10_identity(firmware: tuple[int, int, int], capabilities: int) -> None:
    """版本分别校验：0.10.2增加标定读回位，不向旧0.10.1放开该位。"""

    require(firmware in SUPPORTED_FIRMWARES, f"unsupported firmware={firmware}")
    require(
        (capabilities & PRODUCT_CAPABILITIES) == PRODUCT_CAPABILITIES,
        f"missing stage-10 product capabilities: 0x{capabilities:04X}",
    )
    if firmware == (0, 10, 0):
        require(capabilities == PRODUCT_CAPABILITIES, "0.10.0 identity drift")
    elif firmware == (0, 10, 1):
        require(
            (capabilities & ~KNOWN_0101_CAPABILITIES) == 0,
            f"unknown 0.10.1 capabilities: 0x{capabilities:04X}",
        )
    elif firmware == (0, 10, 2):
        require((capabilities & CALIBRATION_STATUS_CAPABILITY) != 0,
                "0.10.2 missing calibration status capability")
        require((capabilities & ~KNOWN_0102_CAPABILITIES) == 0,
                f"unknown 0.10.2 capabilities: 0x{capabilities:04X}")
    else:
        require((capabilities & (CALIBRATION_STATUS_CAPABILITY | ASSIST_TORQUE_LIMIT_CAPABILITY)) ==
                (CALIBRATION_STATUS_CAPABILITY | ASSIST_TORQUE_LIMIT_CAPABILITY),
                "0.10.3 missing calibration or torque limit capability")
        require((capabilities & ~KNOWN_0103_CAPABILITIES) == 0,
                f"unknown 0.10.3 capabilities: 0x{capabilities:04X}")


def make_start_payload(rate_hz: int, channel_mask: int) -> bytes:
    require(rate_hz in (5, 10, 20), "telemetry rate must be 5, 10 or 20 Hz")
    require(
        channel_mask != 0 and (channel_mask & ~TELEMETRY_CHANNELS_MASK) == 0,
        "telemetry channel mask is empty or contains unknown bits",
    )
    return struct.pack("<BH", rate_hz, channel_mask)


def parse_telemetry_status(frame: Frame) -> TelemetryStatus:
    require(len(frame.payload) == 16, "TELEMETRY_STATUS payload must be 16 bytes")
    streaming, rate, mask, stream, notify_count, dropped, queued, capacity = (
        struct.unpack("<BBHHIIBB", frame.payload)
    )
    require(streaming in (0, 1), "streaming must be 0 or 1")
    require(rate in (5, 10, 20), "invalid telemetry rate")
    require(mask != 0 and (mask & ~TELEMETRY_CHANNELS_MASK) == 0, "invalid mask")
    require(queued <= capacity, "queued samples exceed queue capacity")
    require(capacity == TELEMETRY_RING_CAPACITY, "unexpected ring capacity")
    require(not streaming or stream != 0, "active stream sequence must be nonzero")
    return TelemetryStatus(
        bool(streaming), rate, mask, stream, notify_count, dropped, queued, capacity
    )


def _decode_value(channel: int, bits: int, valid: bool, side: str) -> int | None:
    invalid = UNSIGNED_INVALID if channel in UNSIGNED_CHANNELS else SIGNED_INVALID_BITS
    require(
        (bits != invalid) if valid else (bits == invalid),
        f"{side} valid flag and invalid sentinel disagree",
    )
    if not valid:
        return None
    if channel in UNSIGNED_CHANNELS:
        if channel == CHANNEL_GAIT_PHASE:
            require(bits <= 9999, f"{side} phase exceeds [0,1)")
        else:
            require(bits <= 10000, f"{side} confidence exceeds [0,1]")
        return bits
    return struct.unpack("<h", struct.pack("<H", bits))[0]


def parse_telemetry_packet(raw: bytes) -> TelemetryPacket:
    raw = bytes(raw)
    require(len(raw) == TELEMETRY_FRAME_SIZE, "telemetry packet must be 20 bytes")
    schema, packet_type, channel, flags = raw[:4]
    require(schema == TELEMETRY_SCHEMA, "unsupported telemetry schema")
    require(packet_type == TELEMETRY_SAMPLE_TYPE, "unknown telemetry packet type")
    require(
        channel != 0
        and (channel & ~TELEMETRY_CHANNELS_MASK) == 0
        and (channel & (channel - 1)) == 0,
        "telemetry channel must contain exactly one known bit",
    )
    stream, sample, timestamp, epoch, left_bits, right_bits = struct.unpack_from(
        "<HIIHHH", raw, 4
    )
    require(stream != 0, "stream sequence must be nonzero")
    require(sample != 0, "sample sequence must be nonzero")
    state = (flags & FLAG_SYSTEM_STATE_MASK) >> 3
    require(state <= 7, "system state does not fit three bits")
    left = _decode_value(channel, left_bits, bool(flags & FLAG_LEFT_VALID), "left")
    right = _decode_value(
        channel, right_bits, bool(flags & FLAG_RIGHT_VALID), "right"
    )
    return TelemetryPacket(channel, flags, stream, sample, timestamp, epoch, left, right)


def validate_sample_groups(
    packets: Sequence[TelemetryPacket], expected_channels: set[int]
) -> int:
    """校验序号连续且除采集边界最后一组外，每个样本的通道完整且无重复。"""
    require(packets, "no telemetry packets to group")
    groups: dict[int, set[int]] = {}
    for packet in packets:
        channels = groups.setdefault(packet.sample_sequence, set())
        require(
            packet.channel not in channels,
            f"duplicate channel 0x{packet.channel:02X} in sample {packet.sample_sequence}",
        )
        channels.add(packet.channel)

    sequences = sorted(groups)
    for previous, current in zip(sequences, sequences[1:]):
        require(
            current == previous + 1,
            f"sample sequence gap: {previous} -> {current}",
        )

    complete = 0
    # 采集在任意帧处截止，只有最后一个样本允许是边界半包；所有内部样本必须完整。
    for sequence in sequences[:-1]:
        require(
            groups[sequence] == expected_channels,
            f"sample {sequence} channels={groups[sequence]} expected={expected_channels}",
        )
        complete += 1
    if groups[sequences[-1]] == expected_channels:
        complete += 1
    return complete


def _expect_failure(function: Callable[[], object], message: str) -> None:
    try:
        function()
    except AcceptanceError:
        return
    raise AcceptanceError(message)


def run_self_test(reporter: Reporter) -> None:
    checks: list[tuple[str, Callable[[], None]]] = []

    def check(name: str) -> Callable[[Callable[[], None]], Callable[[], None]]:
        def decorate(function: Callable[[], None]) -> Callable[[], None]:
            checks.append((name, function))
            return function

        return decorate

    @check("golden telemetry requests stay within 20 bytes")
    def _requests() -> None:
        require(
            encode_request(OP_GET_TELEMETRY_STATUS, 1)
            == bytes.fromhex("01 08 01 00"),
            "GET_TELEMETRY_STATUS bytes differ",
        )
        require(
            encode_request(
                OP_START_TELEMETRY,
                2,
                make_start_payload(20, TELEMETRY_DEFAULT_CHANNEL_MASK),
            )
            == bytes.fromhex("01 17 02 03 14 78 00"),
            "START_TELEMETRY bytes differ",
        )
        require(
            encode_request(OP_STOP_TELEMETRY, 3)
            == bytes.fromhex("01 18 03 00"),
            "STOP_TELEMETRY bytes differ",
        )

    @check("0.10.0 and 0.10.1 identities are explicit")
    def _identities() -> None:
        validate_stage10_identity((0, 10, 3), 0x037F)
        validate_stage10_identity((0, 10, 3), 0x03FF)
        _expect_failure(lambda: validate_stage10_identity((0, 10, 3), 0x017F),
                        "0.10.3 missing torque capability accepted")
        validate_stage10_identity((0, 10, 2), 0x017F)
        validate_stage10_identity((0, 10, 2), 0x01FF)
        _expect_failure(lambda: validate_stage10_identity((0, 10, 2), 0x007F),
                        "0.10.2 missing calibration capability accepted")
        validate_stage10_identity((0, 10, 0), 0x007F)
        validate_stage10_identity((0, 10, 1), 0x007F)
        validate_stage10_identity((0, 10, 1), 0x00FF)
        _expect_failure(
            lambda: validate_stage10_identity((0, 10, 1), 0x01FF),
            "unknown high capability was accepted",
        )
        _expect_failure(
            lambda: validate_stage10_identity((0, 10, 0), 0x00FF),
            "0.10.0 unexpectedly advertised engineering capability",
        )

    @check("golden telemetry status decodes frame-notify counter")
    def _status() -> None:
        frame = decode_frame(
            bytes.fromhex("01 88 09 10 01 14 78 00 34 12 40 00 00 00 02 00 00 00 03 20")
        )
        require_correlated(frame, OP_GET_TELEMETRY_STATUS, 9)
        status = parse_telemetry_status(frame)
        require(status.stream_sequence == 0x1234, "stream sequence mismatch")
        require(status.notify_success_count == 64, "notify counter mismatch")
        require(status.dropped_sample_count == 2, "drop counter mismatch")

    @check("signed packet exposes state/output/epoch and i16 values")
    def _signed_packet() -> None:
        packet = parse_telemetry_packet(
            bytes.fromhex("01 01 10 DB 07 00 2A 00 00 00 E8 03 00 00 34 12 85 FF FA 00")
        )
        require(packet.left_raw == -123 and packet.right_raw == 250, "i16 decode")
        require(packet.system_state == 3, "RUNNING state flags")
        require(packet.output_allowed and packet.real_output_enabled, "output flags")
        require(packet.activation_epoch_lsb == 0x1234, "activation epoch")

    @check("unsigned phase packet preserves an invalid right side")
    def _phase_packet() -> None:
        packet = parse_telemetry_packet(
            bytes.fromhex("01 01 08 01 01 00 01 00 00 00 01 00 00 00 02 00 C4 09 FF FF")
        )
        require(packet.left_raw == 2500, "phase fixed-point decode")
        require(packet.right_raw is None, "invalid phase sentinel was displayed")

    @check("sentinel mismatch is rejected")
    def _sentinel_mismatch() -> None:
        _expect_failure(
            lambda: parse_telemetry_packet(
                bytes.fromhex(
                    "01 01 40 00 01 00 01 00 00 00 01 00 00 00 00 00 00 00 00 80"
                )
            ),
            "valid numeric value was accepted while valid flag was clear",
        )

    @check("multi-bit and unknown channel masks are rejected")
    def _invalid_channels() -> None:
        valid = bytearray.fromhex(
            "01 01 08 00 01 00 01 00 00 00 01 00 00 00 00 00 FF FF FF FF"
        )
        valid[2] = 0x18
        _expect_failure(lambda: parse_telemetry_packet(valid), "multi-bit channel")
        _expect_failure(lambda: make_start_payload(20, 0x80), "unknown mask")

    @check("default stream load is explicit")
    def _default_load() -> None:
        channel_count = TELEMETRY_DEFAULT_CHANNEL_MASK.bit_count()
        require(channel_count == 4, "default mask must select four channels")
        require(
            TELEMETRY_DEFAULT_RATE_HZ * channel_count == 80,
            "default stream should request 80 Notify/s",
        )

    @check("sample grouping rejects gaps and incomplete interior samples")
    def _sample_groups() -> None:
        def packet(sequence: int, channel: int) -> TelemetryPacket:
            return TelemetryPacket(channel, 0, 1, sequence, sequence, 0, None, None)

        expected = {CHANNEL_GAIT_PHASE, CHANNEL_ASSIST_SUGGESTION}
        valid = [
            packet(1, CHANNEL_GAIT_PHASE),
            packet(1, CHANNEL_ASSIST_SUGGESTION),
            packet(2, CHANNEL_GAIT_PHASE),
            packet(2, CHANNEL_ASSIST_SUGGESTION),
            packet(3, CHANNEL_GAIT_PHASE),
        ]
        require(validate_sample_groups(valid, expected) == 2, "boundary grouping")
        _expect_failure(
            lambda: validate_sample_groups(
                [
                    packet(1, CHANNEL_GAIT_PHASE),
                    packet(2, CHANNEL_GAIT_PHASE),
                    packet(2, CHANNEL_ASSIST_SUGGESTION),
                ],
                expected,
            ),
            "incomplete interior sample was accepted",
        )
        _expect_failure(
            lambda: validate_sample_groups(
                [packet(1, CHANNEL_GAIT_PHASE), packet(3, CHANNEL_GAIT_PHASE)],
                {CHANNEL_GAIT_PHASE},
            ),
            "sample sequence gap was accepted",
        )

    for name, function in checks:
        try:
            function()
        except Exception as error:
            reporter.failed_test(name, str(error))
        else:
            reporter.passed_test(name)


async def _transact_frame(
    transport: BleTransport, opcode: int, request_id: int, payload: bytes = b""
) -> Frame:
    raw = await transport.transact(encode_request(opcode, request_id, payload))
    frame = decode_frame(raw)
    require_correlated(frame, opcode, request_id)
    return frame


async def run_hardware(args: argparse.Namespace, reporter: Reporter) -> None:
    try:
        from bleak import BleakClient, BleakScanner
    except ImportError as error:
        raise AcceptanceError(
            "hardware run requires bleak (install with: python -m pip install bleak)"
        ) from error

    device = await resolve_device(
        BleakScanner, args.address, args.name, args.scan_timeout
    )
    reporter.passed_test(
        "BLE discovery",
        f"name={getattr(device, 'name', None)} address={getattr(device, 'address', '?')}",
    )
    async with BleakClient(device, timeout=args.connect_timeout) as client:
        require(client.is_connected, "BLE connection was not established")
        reporter.passed_test("BLE connect")
        transport = BleTransport(client, args.response_timeout)
        packets: asyncio.Queue[bytes] = asyncio.Queue()

        def telemetry_callback(_sender: Any, data: bytearray) -> None:
            packets.put_nowait(bytes(data))

        await client.start_notify(
            RESPONSE_CHARACTERISTIC_UUID, transport.notification_callback
        )
        await client.start_notify(TELEMETRY_CHARACTERISTIC_UUID, telemetry_callback)
        reporter.passed_test("response and telemetry notifications enabled")
        ids = RequestIds()
        stream_started = False
        try:
            request_id = ids.take()
            info_frame = await _transact_frame(transport, OP_GET_INFO, request_id)
            info = parse_info(info_frame)
            validate_stage10_identity(info.firmware, info.capabilities)
            require(info.max_frame_size == MAX_FRAME_SIZE, "max frame is not 20")
            require(
                info.capabilities & TELEMETRY_CAPABILITY,
                f"capabilities=0x{info.capabilities:04X}",
            )
            reporter.passed_test(
                "GET_INFO",
                f"fw={'.'.join(map(str, info.firmware))} "
                f"capabilities=0x{info.capabilities:04X}",
            )

            request_id = ids.take()
            initial = parse_telemetry_status(
                await _transact_frame(
                    transport, OP_GET_TELEMETRY_STATUS, request_id
                )
            )
            reporter.passed_test(
                "initial telemetry status",
                f"streaming={int(initial.streaming)} rate={initial.rate_hz} mask=0x{initial.channel_mask:04X}",
            )

            start_id = ids.take()
            start_raw = await transport.transact(
                encode_request(
                    OP_START_TELEMETRY,
                    start_id,
                    make_start_payload(args.rate, args.channel_mask),
                )
            )
            expect_ack(start_raw, OP_START_TELEMETRY, start_id, RESULT_OK)
            stream_started = True
            reporter.passed_test(
                "START_TELEMETRY",
                f"rate={args.rate} mask=0x{args.channel_mask:04X}",
            )

            received: list[TelemetryPacket] = []
            deadline = time.monotonic() + args.duration
            while time.monotonic() < deadline:
                timeout = max(0.01, deadline - time.monotonic())
                try:
                    raw = await asyncio.wait_for(packets.get(), timeout=timeout)
                except asyncio.TimeoutError:
                    break
                received.append(parse_telemetry_packet(raw))
            require(received, "no telemetry packets received")
            measured_frame_count = len(received)
            measured_samples = {packet.sample_sequence for packet in received}
            measured_last_sample = max(measured_samples)

            # 再读到下一样本的首帧，使测量窗口最后一个样本成为可严格校验的内部组。
            grace_deadline = time.monotonic() + 1.0
            while time.monotonic() < grace_deadline:
                try:
                    raw = await asyncio.wait_for(
                        packets.get(), timeout=grace_deadline - time.monotonic()
                    )
                except asyncio.TimeoutError:
                    break
                packet = parse_telemetry_packet(raw)
                received.append(packet)
                if packet.sample_sequence != measured_last_sample:
                    break
            streams = {packet.stream_sequence for packet in received}
            require(len(streams) == 1, f"mixed stream sequences: {sorted(streams)}")
            seen_channels = {packet.channel for packet in received}
            expected_channels = {
                1 << bit
                for bit in range(7)
                if args.channel_mask & (1 << bit)
            }
            require(
                seen_channels == expected_channels,
                f"channels got={seen_channels} expected={expected_channels}",
            )
            complete_samples = validate_sample_groups(received, expected_channels)
            sample_rate = len(measured_samples) / args.duration
            require(
                args.rate * 0.75 <= sample_rate <= args.rate * 1.25,
                f"sample rate {sample_rate:.2f} Hz outside tolerance",
            )
            reporter.passed_test(
                "telemetry sample stream",
                f"frames={measured_frame_count} samples={len(measured_samples)} "
                f"complete={complete_samples} measured={sample_rate:.2f}Hz",
            )

            status_id = ids.take()
            running = parse_telemetry_status(
                await _transact_frame(
                    transport, OP_GET_TELEMETRY_STATUS, status_id
                )
            )
            require(running.streaming, "status says stream is stopped")
            require(running.rate_hz == args.rate, "effective rate mismatch")
            require(running.channel_mask == args.channel_mask, "effective mask mismatch")
            require(
                running.notify_success_count >= len(received),
                "device notify counter is smaller than received frame count",
            )
            require(
                running.dropped_sample_count == 0,
                f"device dropped {running.dropped_sample_count} samples",
            )
            reporter.passed_test(
                "streaming telemetry status",
                f"notify={running.notify_success_count} dropped=0 queued={running.queued_samples}/{running.queue_capacity}",
            )
        finally:
            if stream_started:
                stop_id = ids.take()
                stop_raw = await transport.transact(
                    encode_request(OP_STOP_TELEMETRY, stop_id)
                )
                expect_ack(
                    stop_raw,
                    OP_STOP_TELEMETRY,
                    stop_id,
                    {RESULT_OK, RESULT_ALREADY_SATISFIED},
                )
                status_id = ids.take()
                stopped = parse_telemetry_status(
                    await _transact_frame(
                        transport, OP_GET_TELEMETRY_STATUS, status_id
                    )
                )
                require(not stopped.streaming, "stream remained active after STOP")
                reporter.passed_test("STOP_TELEMETRY", "streaming=0")
            await client.stop_notify(TELEMETRY_CHARACTERISTIC_UUID)
            await client.stop_notify(RESPONSE_CHARACTERISTIC_UUID)


def _parse_mask(value: str) -> int:
    try:
        result = int(value, 0)
    except ValueError as error:
        raise argparse.ArgumentTypeError("mask must be an integer such as 0x78") from error
    if result <= 0 or result & ~TELEMETRY_CHANNELS_MASK:
        raise argparse.ArgumentTypeError("mask must be a nonzero subset of 0x7F")
    return result


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="ESP-EXO Stage 10 motion-free BLE telemetry acceptance"
    )
    parser.add_argument("--self-test", action="store_true")
    target = parser.add_mutually_exclusive_group()
    target.add_argument("--address")
    target.add_argument("--name", default=DEFAULT_DEVICE_NAME)
    parser.add_argument("--scan-timeout", type=float, default=10.0)
    parser.add_argument("--connect-timeout", type=float, default=10.0)
    parser.add_argument("--response-timeout", type=float, default=3.0)
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--rate", type=int, choices=(5, 10, 20), default=20)
    parser.add_argument(
        "--channel-mask", type=_parse_mask, default=TELEMETRY_DEFAULT_CHANNEL_MASK
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_argument_parser().parse_args(argv)
    reporter = Reporter()
    started = time.monotonic()
    try:
        for name in ("scan_timeout", "connect_timeout", "response_timeout", "duration"):
            require(getattr(args, name) > 0, f"--{name.replace('_', '-')} must be > 0")
        if args.self_test:
            run_self_test(reporter)
        else:
            print(
                "[INFO] motion-free run: no exoskeleton START/STOP, zero or control command is sent"
            )
            asyncio.run(run_hardware(args, reporter))
    except (AcceptanceError, OSError, asyncio.TimeoutError) as error:
        reporter.failed_test("phase10 acceptance", str(error))
    except KeyboardInterrupt:
        reporter.failed_test("phase10 acceptance", "interrupted by user")
    except Exception as error:
        reporter.failed_test("phase10 acceptance", f"{type(error).__name__}: {error}")
    print(f"elapsed={time.monotonic() - started:.3f}s")
    return reporter.summary()


if __name__ == "__main__":
    sys.exit(main())
