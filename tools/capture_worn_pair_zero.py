"""Capture one operator-confirmed worn neutral RAM pair zero; never starts motors or writes NVS."""
import argparse
import asyncio
from dataclasses import asdict
from datetime import datetime
import json
import math
from pathlib import Path
import struct
from zoneinfo import ZoneInfo

from bleak import BleakClient, BleakScanner
import phase6_ble_acceptance as protocol


def need(condition, message):
    if not condition:
        raise RuntimeError(message)


def calibration(frame):
    need(len(frame.payload) == 16, "Calibration reply length invalid")
    side, actuator, can_id, direction, flags, reserved, raw, zero, age = struct.unpack("<6BiiH", frame.payload)
    need(reserved == 0 and flags & ~63 == 0, "Calibration reply fields invalid")
    scale = 180 / (math.pi * 1000)
    return {"side": side, "actuator": actuator, "can_id": can_id, "direction": direction, "flags": flags,
            "raw_degrees": raw * scale if raw != -2147483648 else None,
            "zero_degrees": zero * scale if zero != -2147483648 else None, "age_ms": age}


async def run(expected_boot):
    root = Path(__file__).resolve().parents[1]
    now = datetime.now(ZoneInfo("Asia/Shanghai"))
    dest = root / "output/adaptive" / now.strftime("%Y%m%d") / ("worn_pair_zero_" + now.strftime("%Y%m%d_%H%M%S_%f"))
    dest.mkdir(parents=True)
    result = {"at": now.isoformat(),
              "operator_report": "worn, supported, neutral standing posture held still",
              "expected_boot": hex(expected_boot), "capture_sent": False, "verified": False,
              "frames": [], "error": None}
    try:
        device = await protocol.resolve_device(BleakScanner, "44:1B:F6:FE:22:C2", "ESP-EXO", 5)
        async with BleakClient(device, timeout=15) as client:
            transport = protocol.BleTransport(client, 5)
            request_id = 0
            await client.start_notify(protocol.RESPONSE_CHARACTERISTIC_UUID, transport.notification_callback)

            async def request(opcode, payload=b""):
                nonlocal request_id
                need(opcode in (1, 2, 3, 9) or
                     (opcode == 0x1f and payload == b"\x03\x01" and not result["capture_sent"]),
                     "Command outside one-shot read/capture allowlist")
                request_id += 1
                wire = protocol.encode_request(opcode, request_id, payload)
                entry = {"at": datetime.now().isoformat(), "request": wire.hex()}
                result["frames"].append(entry)
                if opcode == 0x1f:
                    result["capture_sent"] = True
                raw = await transport.transact(wire)
                entry["response"] = raw.hex()
                frame = protocol.decode_frame(raw)
                protocol.require_correlated(frame, opcode, request_id)
                return frame

            info = protocol.parse_info(await request(1))
            result["info"] = asdict(info)
            need(info.firmware == (0, 11, 5) and info.boot_id == expected_boot and
                 info.capabilities & 0x4100 == 0x4100, "Unexpected firmware/boot/capability")
            state = protocol.parse_system_status(await request(2))
            result["before_system"] = asdict(state)
            need(state.state == 1 and state.flags & 0x1c == 0, "System not closed IDLE")
            before = [calibration(await request(9, bytes((side,)))) for side in (0, 1)]
            result["before_calibration"] = before
            for side, value in enumerate(before):
                need(value["side"] == side and value["flags"] & 0x1b == 0x1b and value["age_ms"] <= 50,
                     "Pre-capture mapping/direction/feedback invalid")
            need(before[0]["actuator"] != before[1]["actuator"], "Same actuator bound to both sides")

            ack = protocol.parse_ack(await request(0x1f, b"\x03\x01"))
            result["capture_ack"] = asdict(ack)
            need(ack.result == protocol.RESULT_OK, "Pair capture rejected")
            joints = [protocol.parse_joint_status(await request(3, bytes((side,)))) for side in (0, 1)]
            after = [calibration(await request(9, bytes((side,)))) for side in (0, 1)]
            result["after_joints"] = [dict(asdict(j), position_degrees=j.position_deg,
                                                 velocity_degrees_s=j.velocity_deg_s) for j in joints]
            result["after_calibration"] = after
            state = protocol.parse_system_status(await request(2))
            result["after_system"] = asdict(state)
            need(state.state == 1 and state.flags & 0x1c == 0, "System changed after capture")
            for side, (joint, value) in enumerate(zip(joints, after)):
                need(joint.side == value["side"] == side and joint.flags == value["flags"] == 63 and
                     joint.feedback_age_ms <= 50 and value["age_ms"] <= 50,
                     "Post-capture validity check failed")
                need(abs(joint.position_deg) <= 0.5 and
                     value["raw_degrees"] is not None and value["zero_degrees"] is not None and
                     abs(value["raw_degrees"] - value["zero_degrees"]) <= 0.5,
                     f"Side {side}: post-capture zero-position readback outside 0.5 deg tolerance")
                need(abs(joint.velocity_deg_s) <= 5,
                     f"Side {side}: post-capture reported velocity {joint.velocity_deg_s:.6f} deg/s exceeds 5 deg/s; position change is not established by this check")
            result["verified"] = True
            await client.stop_notify(protocol.RESPONSE_CHARACTERISTIC_UUID)
        result["disconnected"] = True
    except Exception as error:
        result["error"] = repr(error)
    finally:
        (dest / "result.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
        print(json.dumps({"directory": str(dest), "capture_sent": result["capture_sent"],
                          "verified": result["verified"], "error": result["error"]}, ensure_ascii=False))
    return 1 if result["error"] else 0


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--expected-boot", type=lambda value: int(value, 0), required=True)
    parser.add_argument("--operator-confirmed-neutral", action="store_true")
    args = parser.parse_args()
    need(args.operator_confirmed_neutral, "Current operator neutral-posture confirmation required")
    raise SystemExit(asyncio.run(run(args.expected_boot)))
