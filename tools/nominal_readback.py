"""Read-only UART/BLE cross-check for a newly deployed nominal candidate.

No START, parameter writes, zero capture, or feedback restoration. Motor selector
commands only change which existing feedback the console displays.
"""
import argparse
import asyncio
from dataclasses import asdict
from datetime import datetime
import json
from pathlib import Path
import re
from nominal_peak_run import Console, check_rest, field, need, verify_idle
import phase6_ble_acceptance as protocol


async def read_ble(expected_boot, expected_firmware):
    from bleak import BleakClient, BleakScanner
    device = await protocol.resolve_device(BleakScanner, "44:1B:F6:FE:22:C2", "ESP-EXO", 5)
    result = {"address": device.address, "frames": []}
    async with BleakClient(device, timeout=15) as client:
        transport = protocol.BleTransport(client, 5)
        await client.start_notify(protocol.RESPONSE_CHARACTERISTIC_UUID, transport.notification_callback)
        for opcode, label, parser in ((1, "info", protocol.parse_info), (2, "system", protocol.parse_system_status)):
            wire = protocol.encode_request(opcode, opcode)
            raw = await transport.transact(wire)
            result["frames"].append({"request": wire.hex(), "response": raw.hex()})
            frame = protocol.decode_frame(raw)
            protocol.require_correlated(frame, opcode, opcode)
            result[label] = asdict(parser(frame))
        await client.stop_notify(protocol.RESPONSE_CHARACTERISTIC_UUID)
    need(tuple(result["info"]["firmware"]) == expected_firmware
         and result["info"]["boot_id"] == expected_boot, "BLE firmware/boot differs from UART deployment")
    need(result["system"]["state"] == 1 and result["system"]["flags"] & 0x1c == 0,
         "BLE system is not closed IDLE")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--expected-boot", type=lambda value: int(value, 0), required=True)
    parser.add_argument("--motor-power-restored", action="store_true")
    parser.add_argument("--expected-firmware", type=protocol.parse_firmware_version, default=(0, 11, 4))
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    destination = root / "output/adaptive/20261104" / ("nominal_readback_" + datetime.now().strftime("%Y%m%d_%H%M%S"))
    destination.mkdir()
    result = {"at": datetime.now().astimezone().isoformat(), "expected_boot": hex(args.expected_boot),
              "power_restored_report": args.motor_power_restored, "rounds": [], "passed": False, "error": None}
    console = None
    try:
        console = Console(destination)
        console.receive(.5)
        counts = []
        for _ in range(2):
            data = {command: console.command(command) for command in
                    ("system_status", "ble_status", "control_status", "assist_status", "adaptive_status", "can_status")}
            result["rounds"].append(data)
            verify_idle(data["system_status"])
            need(field(data["system_status"], "START").startswith("requests=0 "), "Unexpected START since reboot")
            need(int(field(data["ble_status"], "Boot ID"), 16) == args.expected_boot, "Boot changed")
            need(field(data["control_status"], "Output/error").startswith("0/0 ")
                 and field(data["control_status"], "Armed/tick") == "no/idle", "Unexpected motion output")
            need("ACTIVE_DEPLOYMENT=OFF_BODY_BENCH_CANDIDATE active_limit_mnm=15000" in data["adaptive_status"],
                 "Unexpected deployment variant")
            need(field(data["assist_status"], "Selected").startswith("TRANSPARENT / DISABLED strategy=1 "),
                 "Unexpected assist selection")
            if args.motor_power_restored:
                can = data["can_status"]
                need(field(can, "State") == "RUNNING" and int(field(can, "TX error counter")) < 128
                     and int(field(can, "RX error counter")) < 128, "CAN unhealthy")
                check_rest(console)
                this_count = []
                for side in ("left", "right"):
                    joint = console.command("joint_status " + side)
                    data["joint_status " + side] = joint
                    feedback = re.search(r"valid / online, count=(\d+), age=(\d+) us", field(joint, "Feedback"))
                    need(feedback is not None and int(feedback[2]) <= 50000, "Joint feedback stale")
                    this_count.append(int(feedback[1]))
                counts.append(this_count)
        if args.motor_power_restored:
            need(all(after > before for before, after in zip(*counts)), "Feedback count did not advance")
        console.close()
        console = None
        result["ble"] = asyncio.run(read_ble(args.expected_boot, args.expected_firmware))
        result["passed"] = True
    except Exception as error:
        result["error"] = repr(error)
    finally:
        if console:
            console.close()
        (destination / "result.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
        print(json.dumps({"directory": str(destination), "passed": result["passed"], "error": result["error"]}, ensure_ascii=False))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
