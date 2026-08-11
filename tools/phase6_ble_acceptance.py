#!/usr/bin/env python3
"""Stage 6 product-BLE v1 acceptance client for ESP-EXO.

The default hardware run is intentionally motion-free.  It performs only
information/status reads plus protocol replay, request-id conflict and
malformed-frame checks.  ``bleak`` is imported only for a hardware run, so
``--self-test`` needs no third-party packages or BLE adapter.

Examples:

    python tools/phase6_ble_acceptance.py --self-test
    python tools/phase6_ble_acceptance.py --name ESP-EXO
    python tools/phase6_ble_acceptance.py --address AA:BB:CC:DD:EE:FF

Dangerous commands are unreachable unless all three gates are supplied:

    --allow-dangerous
    --dangerous-command <command>
    --confirm-dangerous "ESP-EXO STAGE6 DANGEROUS"
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


PROTOCOL_VERSION = 1
HEADER_SIZE = 4
MAX_PAYLOAD_SIZE = 16
MAX_FRAME_SIZE = 20
RESPONSE_BIT = 0x80

OP_GET_INFO = 0x01
OP_GET_SYSTEM_STATUS = 0x02
OP_GET_JOINT_STATUS = 0x03
OP_START = 0x10
OP_STOP = 0x11
OP_CAPTURE_ZERO = 0x12
OP_CLEAR_ZERO = 0x13

SIDE_LEFT = 0
SIDE_RIGHT = 1

RESULT_OK = 0x00
RESULT_ACCEPTED = 0x01
RESULT_ALREADY_SATISFIED = 0x02
RESULT_ALREADY_PENDING = 0x03
RESULT_REJECTED_NOT_READY = 0x10
RESULT_UNSUPPORTED_VERSION = 0x20
RESULT_INVALID_LENGTH = 0x21
RESULT_UNKNOWN_OPCODE = 0x22
RESULT_INVALID_REQUEST_ID = 0x23
RESULT_INVALID_PAYLOAD = 0x24
RESULT_REQUEST_ID_CONFLICT = 0x25

SYSTEM_FLAGS_MASK = 0x3F
JOINT_FLAGS_MASK = 0x3F
CAPABILITIES_REQUIRED = 0x0007

PRODUCT_SERVICE_UUID = "f7a10001-6b5a-4d92-9e7c-3a1c8b640001"
COMMAND_CHARACTERISTIC_UUID = "f7a10002-6b5a-4d92-9e7c-3a1c8b640001"
RESPONSE_CHARACTERISTIC_UUID = "f7a10003-6b5a-4d92-9e7c-3a1c8b640001"
DEFAULT_DEVICE_NAME = "ESP-EXO"

DANGEROUS_CONFIRMATION = "ESP-EXO STAGE6 DANGEROUS"
DANGEROUS_COMMANDS: dict[str, tuple[int, bytes]] = {
    "start": (OP_START, b""),
    "stop": (OP_STOP, b""),
    "capture-zero-left": (OP_CAPTURE_ZERO, bytes((SIDE_LEFT,))),
    "capture-zero-right": (OP_CAPTURE_ZERO, bytes((SIDE_RIGHT,))),
    "clear-zero-left": (OP_CLEAR_ZERO, bytes((SIDE_LEFT,))),
    "clear-zero-right": (OP_CLEAR_ZERO, bytes((SIDE_RIGHT,))),
}

# This tuple is audited by the offline self-test.  No motion/calibration opcode
# may enter the default acceptance sequence.
DEFAULT_TEST_OPCODES = (
    OP_GET_INFO,
    OP_GET_SYSTEM_STATUS,
    OP_GET_JOINT_STATUS,
)


class AcceptanceError(RuntimeError):
    """A protocol, transport, or acceptance assertion failed."""


@dataclass(frozen=True)
class Frame:
    version: int
    opcode: int
    request_id: int
    payload: bytes

    @property
    def raw(self) -> bytes:
        return bytes(
            (self.version, self.opcode, self.request_id, len(self.payload))
        ) + self.payload


@dataclass(frozen=True)
class InfoStatus:
    protocol_min: int
    protocol_max: int
    firmware: tuple[int, int, int]
    capabilities: int
    max_frame_size: int
    boot_id: int


@dataclass(frozen=True)
class SystemStatus:
    state: int
    active_step: int
    flags: int
    start_block_reason: int
    last_result: int
    last_failed_step: int
    transition_count: int


@dataclass(frozen=True)
class JointStatus:
    # BLE v1 的线格式必须保留 mrad；以下属性负责转换为产品界面的 deg。
    side: int
    flags: int
    direction: int
    position_mrad: int
    velocity_mrad_s: int
    feedback_age_ms: int

    @property
    def position_deg(self) -> float:
        """Return the human-facing joint position in degrees."""
        return self.position_mrad * 180.0 / (math.pi * 1000.0)

    @property
    def velocity_deg_s(self) -> float:
        """Return the human-facing joint velocity in degrees per second."""
        return self.velocity_mrad_s * 180.0 / (math.pi * 1000.0)


@dataclass(frozen=True)
class Ack:
    result: int
    state: int
    flags: int
    detail: int


class Reporter:
    def __init__(self) -> None:
        self.passed = 0
        self.failed = 0

    def passed_test(self, name: str, detail: str = "") -> None:
        self.passed += 1
        suffix = f" - {detail}" if detail else ""
        print(f"[PASS] {name}{suffix}")

    def failed_test(self, name: str, detail: str) -> None:
        self.failed += 1
        print(f"[FAIL] {name} - {detail}")

    def summary(self) -> int:
        exit_code = 0 if self.failed == 0 else 1
        outcome = "PASS" if exit_code == 0 else "FAIL"
        print(
            f"\n[{outcome}] summary: {self.passed} passed, "
            f"{self.failed} failed; exit_code={exit_code}"
        )
        return exit_code


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AcceptanceError(message)


def parse_firmware_version(value: str) -> tuple[int, int, int]:
    """Parse an exact ``major.minor.patch`` firmware expectation."""
    try:
        parts = tuple(int(part, 10) for part in value.split("."))
    except ValueError as error:
        raise argparse.ArgumentTypeError(
            "firmware must be formatted as major.minor.patch"
        ) from error
    if len(parts) != 3 or any(part < 0 or part > 255 for part in parts):
        raise argparse.ArgumentTypeError(
            "firmware must contain three byte values: major.minor.patch"
        )
    return parts


def encode_request(opcode: int, request_id: int, payload: bytes = b"") -> bytes:
    payload = bytes(payload)
    require(0 <= opcode < RESPONSE_BIT, f"invalid request opcode 0x{opcode:02X}")
    require(1 <= request_id <= 0xFF, "request_id must be in 1..255")
    require(len(payload) <= MAX_PAYLOAD_SIZE, "payload exceeds 16 bytes")
    frame = bytes((PROTOCOL_VERSION, opcode, request_id, len(payload))) + payload
    require(len(frame) <= MAX_FRAME_SIZE, "frame exceeds 20 bytes")
    return frame


def decode_frame(raw: bytes, *, require_response: bool = True) -> Frame:
    raw = bytes(raw)
    require(HEADER_SIZE <= len(raw) <= MAX_FRAME_SIZE, "invalid frame size")
    version, opcode, request_id, payload_len = raw[:HEADER_SIZE]
    require(version == PROTOCOL_VERSION, f"unexpected version {version}")
    require(payload_len <= MAX_PAYLOAD_SIZE, "payload length exceeds v1 limit")
    require(len(raw) == HEADER_SIZE + payload_len, "declared length mismatch")
    if require_response:
        require((opcode & RESPONSE_BIT) != 0, "notification is not a response")
    return Frame(version, opcode, request_id, raw[HEADER_SIZE:])


def require_correlated(frame: Frame, opcode: int, request_id: int) -> None:
    require(
        frame.opcode == (opcode | RESPONSE_BIT),
        f"response opcode 0x{frame.opcode:02X}, expected 0x{opcode | RESPONSE_BIT:02X}",
    )
    require(
        frame.request_id == request_id,
        f"response request_id {frame.request_id}, expected {request_id}",
    )


def parse_info(frame: Frame) -> InfoStatus:
    require(len(frame.payload) == 12, "INFO payload must be 12 bytes")
    payload = frame.payload
    return InfoStatus(
        protocol_min=payload[0],
        protocol_max=payload[1],
        firmware=(payload[2], payload[3], payload[4]),
        capabilities=struct.unpack_from("<H", payload, 5)[0],
        max_frame_size=payload[7],
        boot_id=struct.unpack_from("<I", payload, 8)[0],
    )


def parse_system_status(frame: Frame) -> SystemStatus:
    require(len(frame.payload) == 10, "SYSTEM_STATUS payload must be 10 bytes")
    payload = frame.payload
    require((payload[2] & ~SYSTEM_FLAGS_MASK) == 0, "unknown system flag bits")
    return SystemStatus(
        state=payload[0],
        active_step=payload[1],
        flags=payload[2],
        start_block_reason=payload[3],
        last_result=payload[4],
        last_failed_step=payload[5],
        transition_count=struct.unpack_from("<I", payload, 6)[0],
    )


def parse_joint_status(frame: Frame) -> JointStatus:
    require(len(frame.payload) == 13, "JOINT_STATUS payload must be 13 bytes")
    payload = frame.payload
    require(payload[0] in (SIDE_LEFT, SIDE_RIGHT), "invalid joint side")
    require((payload[1] & ~JOINT_FLAGS_MASK) == 0, "unknown joint flag bits")
    require(payload[2] in (0, 1, 2), "invalid joint direction")
    position, velocity, age = struct.unpack_from("<iiH", payload, 3)
    return JointStatus(
        side=payload[0],
        flags=payload[1],
        direction=payload[2],
        position_mrad=position,
        velocity_mrad_s=velocity,
        feedback_age_ms=age,
    )


def parse_ack(frame: Frame) -> Ack:
    require(len(frame.payload) == 4, "ACK payload must be 4 bytes")
    payload = frame.payload
    require((payload[2] & ~SYSTEM_FLAGS_MASK) == 0, "unknown ACK flag bits")
    return Ack(payload[0], payload[1], payload[2], payload[3])


def expect_ack(
    raw: bytes,
    opcode: int,
    request_id: int,
    expected_result: int | set[int],
) -> Ack:
    frame = decode_frame(raw)
    require_correlated(frame, opcode, request_id)
    ack = parse_ack(frame)
    allowed = (
        expected_result if isinstance(expected_result, set) else {expected_result}
    )
    require(
        ack.result in allowed,
        f"ACK result 0x{ack.result:02X}, expected "
        + "/".join(f"0x{value:02X}" for value in sorted(allowed)),
    )
    return ack


def validate_dangerous_options(args: argparse.Namespace) -> None:
    supplied = bool(
        args.allow_dangerous
        or args.dangerous_command is not None
        or args.confirm_dangerous is not None
        or getattr(args, "expect_rejected_not_ready", False)
        or getattr(args, "repeat_dangerous", False)
        or getattr(args, "dangerous_only", False)
    )
    if not supplied:
        return
    require(args.allow_dangerous, "dangerous command requires --allow-dangerous")
    require(
        args.dangerous_command is not None,
        "--allow-dangerous requires --dangerous-command",
    )
    require(
        args.confirm_dangerous == DANGEROUS_CONFIRMATION,
        "dangerous confirmation does not exactly match "
        f"'{DANGEROUS_CONFIRMATION}'",
    )
    if getattr(args, "expect_rejected_not_ready", False):
        require(
            args.dangerous_command == "start",
            "--expect-rejected-not-ready is valid only with START",
        )
    if getattr(args, "dangerous_only", False):
        require(
            args.dangerous_command is not None,
            "--dangerous-only requires --dangerous-command",
        )


def run_self_test(reporter: Reporter) -> None:
    checks: list[tuple[str, Callable[[], None]]] = []

    def check(name: str) -> Callable[[Callable[[], None]], Callable[[], None]]:
        def decorate(function: Callable[[], None]) -> Callable[[], None]:
            checks.append((name, function))
            return function

        return decorate

    @check("golden request: GET_INFO")
    def _golden_info_request() -> None:
        require(encode_request(OP_GET_INFO, 0x2A) == bytes.fromhex("01 01 2A 00"), "byte mismatch")

    @check("firmware expectation parser")
    def _firmware_expectation_parser() -> None:
        require(parse_firmware_version("0.7.1") == (0, 7, 1), "version mismatch")

    @check("golden request: right JOINT_STATUS")
    def _golden_joint_request() -> None:
        require(
            encode_request(OP_GET_JOINT_STATUS, 0x2B, bytes((SIDE_RIGHT,)))
            == bytes.fromhex("01 03 2B 01 01"),
            "byte mismatch",
        )

    @check("golden response: INFO")
    def _golden_info_response() -> None:
        frame = decode_frame(
            bytes.fromhex("01 81 2A 0C 01 01 00 06 00 07 00 14 78 56 34 12")
        )
        require_correlated(frame, OP_GET_INFO, 0x2A)
        status = parse_info(frame)
        require(status.firmware == (0, 6, 0), "firmware decode mismatch")
        require(status.capabilities == 0x0007, "capabilities decode mismatch")
        require(status.boot_id == 0x12345678, "boot_id decode mismatch")

    @check("golden response: SYSTEM_STATUS")
    def _golden_system_response() -> None:
        frame = decode_frame(
            bytes.fromhex("01 82 2B 0A 03 07 2F 00 01 00 78 56 34 12")
        )
        require_correlated(frame, OP_GET_SYSTEM_STATUS, 0x2B)
        status = parse_system_status(frame)
        require(status.state == 3 and status.active_step == 7, "state decode mismatch")
        require(status.transition_count == 0x12345678, "counter decode mismatch")

    @check("golden response: signed JOINT_STATUS")
    def _golden_joint_response() -> None:
        frame = decode_frame(
            bytes.fromhex(
                "01 83 2C 0D 00 3F 02 2E FB FF FF 2E 16 00 00 2A 00"
            )
        )
        require_correlated(frame, OP_GET_JOINT_STATUS, 0x2C)
        status = parse_joint_status(frame)
        require(status.position_mrad == -1234, "signed position decode mismatch")
        require(status.velocity_mrad_s == 5678, "velocity decode mismatch")
        require(status.feedback_age_ms == 42, "age decode mismatch")
        require(
            math.isclose(
                status.position_deg,
                -1234.0 * 180.0 / (math.pi * 1000.0),
                rel_tol=0.0,
                abs_tol=1e-12,
            ),
            "position degree conversion mismatch",
        )
        require(
            math.isclose(
                status.velocity_deg_s,
                5678.0 * 180.0 / (math.pi * 1000.0),
                rel_tol=0.0,
                abs_tol=1e-12,
            ),
            "velocity degree conversion mismatch",
        )

    @check("golden response: request-id conflict ACK")
    def _golden_ack_response() -> None:
        ack = expect_ack(
            bytes.fromhex("01 81 2D 04 25 01 23 00"),
            OP_GET_INFO,
            0x2D,
            RESULT_REQUEST_ID_CONFLICT,
        )
        require(ack.state == 1 and ack.flags == 0x23, "ACK decode mismatch")

    @check("strict frame length rejection")
    def _reject_bad_length() -> None:
        try:
            decode_frame(bytes.fromhex("01 81 01 0C 01"))
        except AcceptanceError:
            return
        raise AcceptanceError("truncated frame was accepted")

    @check("default plan contains no dangerous opcode")
    def _safe_default_plan() -> None:
        dangerous = {OP_START, OP_STOP, OP_CAPTURE_ZERO, OP_CLEAR_ZERO}
        require(dangerous.isdisjoint(DEFAULT_TEST_OPCODES), "dangerous opcode leaked")

    @check("dangerous gate rejects missing confirmation")
    def _dangerous_gate() -> None:
        args = argparse.Namespace(
            allow_dangerous=True,
            dangerous_command="start",
            confirm_dangerous=None,
        )
        try:
            validate_dangerous_options(args)
        except AcceptanceError:
            return
        raise AcceptanceError("dangerous command bypassed confirmation")

    for name, function in checks:
        try:
            function()
        except Exception as error:  # Keep self-test output complete.
            reporter.failed_test(name, str(error))
        else:
            reporter.passed_test(name)


class BleTransport:
    def __init__(self, client: Any, timeout_s: float) -> None:
        self.client = client
        self.timeout_s = timeout_s
        self.notifications: asyncio.Queue[bytes] = asyncio.Queue()

    def notification_callback(self, _sender: Any, data: bytearray) -> None:
        self.notifications.put_nowait(bytes(data))

    async def transact(self, request: bytes) -> bytes:
        require(self.notifications.empty(), "unexpected queued notification")
        await self.client.write_gatt_char(
            COMMAND_CHARACTERISTIC_UUID, request, response=True
        )
        try:
            return await asyncio.wait_for(
                self.notifications.get(), timeout=self.timeout_s
            )
        except asyncio.TimeoutError as error:
            raise AcceptanceError(
                f"notification timeout after {self.timeout_s:.1f}s"
            ) from error


async def resolve_device(bleak_scanner: Any, address: str | None, name: str, timeout_s: float) -> Any:
    if address:
        device = await bleak_scanner.find_device_by_address(address, timeout=timeout_s)
        require(device is not None, f"BLE address not found: {address}")
        return device

    devices = await bleak_scanner.discover(timeout=timeout_s)
    matches = [device for device in devices if getattr(device, "name", None) == name]
    require(matches, f"BLE device name not found: {name}")
    require(len(matches) == 1, f"multiple devices named {name}; use --address")
    return matches[0]


async def run_default_hardware_tests(
    transport: BleTransport,
    reporter: Reporter,
    expected_firmware: tuple[int, int, int] | None,
) -> None:
    info_raw = await transport.transact(encode_request(OP_GET_INFO, 1))
    info_frame = decode_frame(info_raw)
    require_correlated(info_frame, OP_GET_INFO, 1)
    info = parse_info(info_frame)
    require(
        info.protocol_min <= PROTOCOL_VERSION <= info.protocol_max,
        "device does not advertise protocol v1",
    )
    require(info.max_frame_size == MAX_FRAME_SIZE, "device max frame is not 20")
    require(
        (info.capabilities & CAPABILITIES_REQUIRED) == CAPABILITIES_REQUIRED,
        "required capability bits are absent",
    )
    if expected_firmware is not None:
        require(
            info.firmware == expected_firmware,
            "firmware mismatch: "
            f"got {'.'.join(map(str, info.firmware))}, "
            f"expected {'.'.join(map(str, expected_firmware))}",
        )
    reporter.passed_test(
        "GET_INFO",
        f"fw={'.'.join(map(str, info.firmware))} boot_id=0x{info.boot_id:08X}",
    )

    system_raw = await transport.transact(
        encode_request(OP_GET_SYSTEM_STATUS, 2)
    )
    system_frame = decode_frame(system_raw)
    require_correlated(system_frame, OP_GET_SYSTEM_STATUS, 2)
    system = parse_system_status(system_frame)
    reporter.passed_test(
        "GET_SYSTEM_STATUS",
        f"state={system.state} flags=0x{system.flags:02X} transitions={system.transition_count}",
    )

    for request_id, side, label in (
        (3, SIDE_LEFT, "left"),
        (4, SIDE_RIGHT, "right"),
    ):
        raw = await transport.transact(
            encode_request(OP_GET_JOINT_STATUS, request_id, bytes((side,)))
        )
        frame = decode_frame(raw)
        require_correlated(frame, OP_GET_JOINT_STATUS, request_id)
        joint = parse_joint_status(frame)
        require(joint.side == side, f"returned side {joint.side}, expected {side}")
        reporter.passed_test(
            f"GET_JOINT_STATUS {label}",
            f"flags=0x{joint.flags:02X} pos={joint.position_deg:.3f}deg "
            f"vel={joint.velocity_deg_s:.3f}deg/s "
            f"age={joint.feedback_age_ms}ms",
        )

    replay_request = encode_request(OP_GET_INFO, 5)
    replay_first = await transport.transact(replay_request)
    replay_second = await transport.transact(replay_request)
    require(replay_second == replay_first, "duplicate response was not byte-exact")
    replay_frame = decode_frame(replay_second)
    require_correlated(replay_frame, OP_GET_INFO, 5)
    parse_info(replay_frame)
    reporter.passed_test("duplicate request replay", replay_second.hex(" "))

    conflict_id = 6
    original = await transport.transact(encode_request(OP_GET_INFO, conflict_id))
    original_frame = decode_frame(original)
    require_correlated(original_frame, OP_GET_INFO, conflict_id)
    parse_info(original_frame)
    conflict = await transport.transact(
        encode_request(OP_GET_SYSTEM_STATUS, conflict_id)
    )
    expect_ack(
        conflict,
        OP_GET_SYSTEM_STATUS,
        conflict_id,
        RESULT_REQUEST_ID_CONFLICT,
    )
    reporter.passed_test("request-id conflict", conflict.hex(" "))

    malformed_cases: Sequence[tuple[str, bytes, int, int, int]] = (
        (
            "unknown opcode",
            bytes((PROTOCOL_VERSION, 0x7E, 7, 0)),
            0x7E,
            7,
            RESULT_UNKNOWN_OPCODE,
        ),
        (
            "response-bit opcode used as request",
            bytes((PROTOCOL_VERSION, 0xFF, 11, 0)),
            0xFF,
            11,
            RESULT_UNKNOWN_OPCODE,
        ),
        (
            "unsupported version",
            bytes((2, OP_GET_INFO, 8, 0)),
            OP_GET_INFO,
            8,
            RESULT_UNSUPPORTED_VERSION,
        ),
        (
            "declared length mismatch",
            bytes((PROTOCOL_VERSION, OP_GET_INFO, 9, 1)),
            OP_GET_INFO,
            9,
            RESULT_INVALID_LENGTH,
        ),
        (
            "invalid side payload",
            bytes((PROTOCOL_VERSION, OP_GET_JOINT_STATUS, 10, 1, 2)),
            OP_GET_JOINT_STATUS,
            10,
            RESULT_INVALID_PAYLOAD,
        ),
        (
            "zero request id",
            bytes((PROTOCOL_VERSION, OP_GET_INFO, 0, 0)),
            OP_GET_INFO,
            0,
            RESULT_INVALID_REQUEST_ID,
        ),
    )
    for name, raw, opcode, request_id, result in malformed_cases:
        response = await transport.transact(raw)
        expect_ack(response, opcode, request_id, result)
        reporter.passed_test(f"malformed: {name}", response.hex(" "))


async def run_dangerous_command(
    transport: BleTransport,
    reporter: Reporter,
    command_name: str,
    expect_rejected_not_ready: bool,
    repeat_dangerous: bool,
) -> None:
    opcode, payload = DANGEROUS_COMMANDS[command_name]
    request_id = 0xF0
    request = encode_request(opcode, request_id, payload)
    print(f"[DANGER] executing explicitly authorized command: {command_name}")
    response = await transport.transact(request)
    if expect_rejected_not_ready:
        acceptable = {RESULT_REJECTED_NOT_READY}
    elif opcode in (OP_START, OP_STOP):
        acceptable = {
            RESULT_ACCEPTED,
            RESULT_ALREADY_SATISFIED,
            RESULT_ALREADY_PENDING,
        }
    else:
        acceptable = {RESULT_OK}
    ack = expect_ack(response, opcode, request_id, acceptable)
    reporter.passed_test(
        f"dangerous command: {command_name}",
        f"result=0x{ack.result:02X} state={ack.state} detail={ack.detail}",
    )
    if repeat_dangerous:
        replay = await transport.transact(request)
        require(replay == response, "dangerous duplicate response was not byte-exact")
        reporter.passed_test(
            f"dangerous duplicate replay: {command_name}", replay.hex(" ")
        )


async def run_hardware(args: argparse.Namespace, reporter: Reporter) -> None:
    try:
        from bleak import BleakClient, BleakScanner
    except ImportError as error:
        raise AcceptanceError(
            "hardware run requires optional dependency 'bleak' "
            "(install with: python -m pip install bleak)"
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
        await client.start_notify(
            RESPONSE_CHARACTERISTIC_UUID, transport.notification_callback
        )
        reporter.passed_test("response notifications enabled")
        try:
            if not args.dangerous_only:
                await run_default_hardware_tests(
                    transport, reporter, args.expect_firmware
                )
            if args.dangerous_command is not None:
                await run_dangerous_command(
                    transport,
                    reporter,
                    args.dangerous_command,
                    args.expect_rejected_not_ready,
                    args.repeat_dangerous,
                )
        finally:
            await client.stop_notify(RESPONSE_CHARACTERISTIC_UUID)


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="ESP-EXO Stage 6 product-BLE v1 acceptance client"
    )
    parser.add_argument(
        "--self-test",
        action="store_true",
        help="run offline golden-frame tests; does not import bleak",
    )
    target = parser.add_mutually_exclusive_group()
    target.add_argument("--address", help="BLE address (recommended when names collide)")
    target.add_argument(
        "--name",
        default=DEFAULT_DEVICE_NAME,
        help=f"exact BLE device name (default: {DEFAULT_DEVICE_NAME})",
    )
    parser.add_argument("--scan-timeout", type=float, default=10.0)
    parser.add_argument("--connect-timeout", type=float, default=10.0)
    parser.add_argument("--response-timeout", type=float, default=3.0)
    parser.add_argument(
        "--expect-firmware",
        type=parse_firmware_version,
        metavar="MAJOR.MINOR.PATCH",
        help="require an exact GET_INFO firmware version (for example 0.7.1)",
    )
    parser.add_argument(
        "--allow-dangerous",
        action="store_true",
        help="first gate for START/STOP/zero commands (default: disabled)",
    )
    parser.add_argument(
        "--dangerous-command",
        choices=sorted(DANGEROUS_COMMANDS),
        help="one explicitly requested motion/calibration command",
    )
    parser.add_argument(
        "--confirm-dangerous",
        help=f"must exactly equal: {DANGEROUS_CONFIRMATION}",
    )
    parser.add_argument(
        "--expect-rejected-not-ready",
        action="store_true",
        help="for the uncalibrated START gate test, require result 0x10",
    )
    parser.add_argument(
        "--repeat-dangerous",
        action="store_true",
        help="resend the exact dangerous frame and require byte-exact replay",
    )
    parser.add_argument(
        "--dangerous-only",
        action="store_true",
        help="skip the default read-only suite and execute only the gated command",
    )
    return parser


def validate_timeouts(args: argparse.Namespace) -> None:
    for name in ("scan_timeout", "connect_timeout", "response_timeout"):
        require(getattr(args, name) > 0.0, f"--{name.replace('_', '-')} must be > 0")


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_argument_parser()
    args = parser.parse_args(argv)
    reporter = Reporter()
    started = time.monotonic()

    try:
        validate_timeouts(args)
        validate_dangerous_options(args)
    except AcceptanceError as error:
        reporter.failed_test("arguments", str(error))
        return reporter.summary()

    if args.self_test:
        if args.dangerous_command is not None:
            reporter.failed_test(
                "arguments", "--self-test cannot execute a dangerous command"
            )
        else:
            run_self_test(reporter)
        print(f"elapsed={time.monotonic() - started:.3f}s")
        return reporter.summary()

    if args.dangerous_only:
        print("[DANGER] default suite skipped by explicit --dangerous-only")
    else:
        print(
            "[INFO] default suite is motion-free: GET_INFO, system/joint status, "
            "replay, conflict, malformed frames"
        )
    try:
        asyncio.run(run_hardware(args, reporter))
    except (AcceptanceError, OSError, asyncio.TimeoutError) as error:
        reporter.failed_test("hardware acceptance", str(error))
    except KeyboardInterrupt:
        reporter.failed_test("hardware acceptance", "interrupted by user")
    except Exception as error:  # Bleak exposes backend-specific exception types.
        reporter.failed_test(
            "hardware acceptance", f"{type(error).__name__}: {error}"
        )

    print(f"elapsed={time.monotonic() - started:.3f}s")
    return reporter.summary()


if __name__ == "__main__":
    sys.exit(main())
