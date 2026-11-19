"""One candidate deployment after the operator confirmed motor power OFF, USB ON.
No START/enable/zero/parameter commands. Preserves and verifies NVS+PHY bytes.
"""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys
import time
import zipfile
import serial
from serial.tools import list_ports

ROOT = Path(__file__).resolve().parents[1]
DEST_ROOT = ROOT / "output/adaptive/20261104"
PACKAGE = DEST_ROOT / "ESP-EXO-0.11.4-nominal-soft-peak-candidate.zip"
PACKAGE_SHA = None


def need(ok, message):
    if not ok:
        raise RuntimeError(message)


def sha(data):
    return hashlib.sha256(data).hexdigest()


def uart_readback(dest):
    connection = serial.Serial(port=None, baudrate=115200, timeout=.05, write_timeout=1)
    connection.dtr = False
    connection.rts = False
    connection.port = "COM5"
    values = {}
    with (dest / "uart_raw.log").open("xb") as raw:
        def receive(seconds):
            end = time.monotonic() + seconds
            data = bytearray()
            while time.monotonic() < end:
                chunk = connection.read(max(1, min(connection.in_waiting, 4096)))
                data.extend(chunk)
                raw.write(chunk)
            raw.flush()
            return re.sub(r"\x1b\[[0-9;]*m", "", data.decode("utf-8", errors="replace")).replace("\r", "")
        try:
            connection.open()
            receive(1)
            for label in ("system_status", "can_status", "ble_status", "control_status", "sensor_status",
                          "assist_status", "adaptive_status", "adaptive_config", "joint_status left", "joint_status right",
                          "ble_status_after", "system_status_after"):
                cmd = label.removesuffix("_after")
                raw.write(("\n[HOST " + datetime.now().isoformat() + "] " + cmd + "\n").encode())
                connection.write((cmd + "\r").encode("ascii"))
                values[label] = receive(1.3 if cmd == "adaptive_config" else .8)
                (dest / (label.replace(" ", "_") + ".txt")).write_text(values[label], encoding="utf-8")
        finally:
            connection.close()
    capture = (dest / "uart_raw.log").read_text(encoding="utf-8", errors="replace")
    need("Brownout detector was triggered" not in capture and "Guru Meditation" not in capture, "Reset/fault observed in UART capture")
    for pattern in (r"^  State:\s+IDLE$", r"^  Run requested:\s+no$", r"^  Output allowed:\s+no$",
                    r"^  START:\s+requests=0 ", r"^  Runtime faults:\s+0;"):
        need(re.search(pattern, values["system_status"], re.M), "Unsafe/unexpected post-flash system state: " + pattern)
        need(re.search(pattern, values["system_status_after"], re.M), "Unexpected final system state: " + pattern)
    need(re.search(r"^  Output/error:\s+0/0 ", values["control_status"], re.M), "Unexpected output count")
    need(re.search(r"^  Armed/tick:\s+no/", values["control_status"], re.M), "Controller unexpectedly armed")
    for pattern in (r"^  Selected:\s+TRANSPARENT / DISABLED strategy=1 profile=0", r"^  Levels L/R:\s+0 % / 0 %$"):
        need(re.search(pattern, values["assist_status"], re.M), "Unexpected assist defaults")
    need("ACTIVE_DEPLOYMENT=OFF_BODY_BENCH_CANDIDATE active_limit_mnm=15000" in values["adaptive_status"], "Wrong deployment variant")
    need("overall_strength=0.000000" in values["adaptive_config"], "Adaptive strength not zero")
    for side in ("left", "right"):
        for field in ("enabled", "strength", "flex_amplitude_nm", "ext_amplitude_nm", "torque_limit_nm"):
            need(re.search(r"^ADAPT config " + side + " " + field + r" 0.000000$", values["adaptive_config"], re.M), "Nonzero adaptive default")
        need("Software zero: N/A (INVALID, RAM only)" in values["joint_status " + side], "Unexpected RAM zero")
    boot = re.search(r"Boot ID:\s+(0x[0-9A-Fa-f]+)", values["ble_status"])
    need(boot is not None, "Missing Boot ID")
    last_boot = re.search(r"Boot ID:\s+(0x[0-9A-Fa-f]+)", values["ble_status_after"])
    need(last_boot is not None and last_boot[1] == boot[1], "Boot ID changed during readback")
    return {"boot_id": boot[1], "idle_gate_closed": True, "start_requests": 0, "motion_outputs": 0,
            "adaptive_deployment": "OFF_BODY_BENCH_CANDIDATE", "active_limit_mnm": 15000,
            "adaptive_defaults_zero": True, "serial_closed": not connection.is_open}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--motor-power-off-confirmed", action="store_true")
    parser.add_argument("--check-only", action="store_true")
    parser.add_argument("--package-sha256", required=True)
    args = parser.parse_args()
    global PACKAGE_SHA
    PACKAGE_SHA = args.package_sha256.lower()
    need(re.fullmatch(r"[a-f0-9]{64}", PACKAGE_SHA), "Expected exact SHA256")
    need(sha(PACKAGE.read_bytes()) == PACKAGE_SHA, "Candidate package hash mismatch")
    with zipfile.ZipFile(PACKAGE) as archive:
        manifest = json.loads(archive.read("candidate-manifest.json"))
        need(manifest["version"] == "0.11.4" and manifest["strategy_version"] == 3
             and manifest["deployed_active_max_mnm"] == 15000 and manifest["bounded_peak_test_entry"]
             and manifest["bounded_peak_profile"] == 2,
             "Wrong nominal candidate identity")
        for entry in manifest["files"]:
            need(sha(archive.read(entry["path"])) == entry["sha256"], "Archive member mismatch")
        need(archive.read("firmware/partition_table/partition-table.bin") ==
             (DEST_ROOT / "deployment_094325/partition_table/partition-table.bin").read_bytes(),
             "Partition layout changed")
    if args.check_only:
        print("PASS pinned 0.11.4 nominal candidate, all member hashes, original partition; no device access")
        return 0
    need(args.motor_power_off_confirmed, "Current operator confirmation is required")
    ports = [p for p in list_ports.comports() if p.device == "COM5"]
    need(len(ports) == 1 and ports[0].vid == 0x1A86 and ports[0].pid == 0x55D3 and ports[0].serial_number == "5C94152132", "COM5 identity changed")
    need(sha(PACKAGE.read_bytes()) == PACKAGE_SHA, "Candidate package hash mismatch")
    dest = DEST_ROOT / ("nominal_deployment_" + datetime.now().strftime("%Y%m%d_%H%M%S"))
    dest.mkdir()
    result = {"created_at": datetime.now().astimezone().isoformat(), "package_sha256": PACKAGE_SHA,
              "operator_report": "motor power OFF, MCU USB retained", "port": "COM5",
              "flash_verified": False, "nvs_phy_unchanged": False, "error": None}
    try:
        with zipfile.ZipFile(PACKAGE) as archive:
            manifest = json.loads(archive.read("candidate-manifest.json"))
            for entry in manifest["files"]:
                need(sha(archive.read(entry["path"])) == entry["sha256"], "Archive manifest mismatch")
            for name in ("twai_network_master.bin", "bootloader/bootloader.bin", "partition_table/partition-table.bin", "flasher_args.json"):
                path = dest / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(archive.read("firmware/" + name))
            (dest / "candidate-manifest.json").write_bytes(archive.read("candidate-manifest.json"))
        need((dest / "partition_table/partition-table.bin").read_bytes() ==
             (DEST_ROOT / "deployment_094325/partition_table/partition-table.bin").read_bytes(), "Partition layout changed")
        flash = json.loads((dest / "flasher_args.json").read_text())
        need(flash["flash_files"] == {"0x0": "bootloader/bootloader.bin", "0x10000": "twai_network_master.bin", "0x8000": "partition_table/partition-table.bin"}, "Unexpected flash addresses")
        with (dest / "flash.log").open("xb") as log:
            def run(label, after, command):
                argv = [sys.executable, "-m", "esptool", "--chip", "esp32s3", "--port", "COM5", "--baud", "460800",
                        "--before", "default_reset", "--after", after] + command
                log.write(("\n[" + datetime.now().isoformat() + "] " + label + "\n").encode())
                log.flush()
                proc = subprocess.run(argv, cwd=dest, stdout=log, stderr=subprocess.STDOUT, timeout=90)
                need(proc.returncode == 0, label + " failed; keep motor power OFF; see flash.log")
                print("PASS " + label, flush=True)
            run("backup NVS+PHY", "no_reset", ["read_flash", "0x9000", "0x7000", "nvs_phy_before.bin"])
            addresses = [item for pair in flash["flash_files"].items() for item in pair]
            run("write candidate images", "no_reset", ["write_flash"] + flash["write_flash_args"] + addresses)
            run("verify candidate images", "no_reset", ["verify_flash"] + addresses)
            result["flash_verified"] = True
            run("read NVS+PHY and reboot", "hard_reset", ["read_flash", "0x9000", "0x7000", "nvs_phy_after.bin"])
        before, after = (dest / "nvs_phy_before.bin").read_bytes(), (dest / "nvs_phy_after.bin").read_bytes()
        need(len(before) == 0x7000 and before == after, "NVS/PHY bytes differ")
        result["nvs_phy_unchanged"] = True
        result["nvs_phy_sha256"] = sha(before)
        result["uart"] = uart_readback(dest)
    except Exception as exc:
        result["error"] = str(exc)
    finally:
        (dest / "result.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
        print(json.dumps({"directory": str(dest), **result}, ensure_ascii=False), flush=True)
    return 1 if result["error"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
