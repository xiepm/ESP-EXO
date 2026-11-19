"""Verify/finally deploy the pinned 0.11.6 app; preserves partition/NVS/PHY.

--flash is used only after the current operator power-off confirmation.
No parameter preparation, zero capture, motor enable or START is performed.
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
PACKAGE = ROOT / "output/adaptive/20261105/ESP-EXO-0.11.6-bounded-trial-5nm-20261105.zip"
PACKAGE_SHA = "7d9011e3150d3274b7e9f542855bdd962f8e1b19dde3fe0362079b1dc63f65a2"


def need(condition, message):
    if not condition:
        raise RuntimeError(message)


def sha(data):
    return hashlib.sha256(data).hexdigest()


def port_identity():
    ports = [p for p in list_ports.comports() if p.device == "COM5"]
    need(len(ports) == 1 and ports[0].vid == 0x1A86 and ports[0].pid == 0x55D3
         and ports[0].serial_number == "5C94152132", "COM5 physical USB identity missing/changed")


def readback(dest):
    c = serial.Serial(port=None, baudrate=115200, timeout=.05, write_timeout=1)
    c.dtr = False
    c.rts = False
    c.port = "COM5"
    blocks = {}
    with (dest / "uart_raw.log").open("xb") as raw:
        def receive(seconds):
            end = time.monotonic() + seconds
            data = bytearray()
            while time.monotonic() < end:
                chunk = c.read(max(1, min(c.in_waiting, 4096)))
                raw.write(chunk)
                data.extend(chunk)
            raw.flush()
            return re.sub(r"\x1b\[[0-9;]*m", "", data.decode("utf-8", errors="replace")).replace("\r", "")
        try:
            port_identity()
            c.open()
            receive(1)
            for label in ["system_status", "ble_status", "assist_status", "control_status",
                          "adaptive_status", "adaptive_config", "ble_status_after", "system_status_after"]:
                cmd = label.removesuffix("_after")
                c.write((cmd + "\r").encode("ascii"))
                blocks[label] = receive(1.3 if cmd == "adaptive_config" else .8)
                (dest / (label + ".txt")).write_text(blocks[label], encoding="utf-8")
        finally:
            c.close()
    for key in ["system_status", "system_status_after"]:
        for pattern in [r"^  State:\s+IDLE$", r"^  Run requested:\s+no$", r"^  Output allowed:\s+no$",
                        r"^  START:\s+requests=0 "]:
            need(re.search(pattern, blocks[key], re.M), key + " unexpected: " + pattern)
    boots = [re.findall(r"Boot ID:\s*(0x[0-9A-Fa-f]{8})", blocks[k]) for k in ["ble_status", "ble_status_after"]]
    need(len(boots[0]) == len(boots[1]) == 1 and boots[0] == boots[1], "Post-flash Boot changed/missing")
    need("ACTIVE_DEPLOYMENT=BOUNDED_MANUAL_TRIAL active_limit_mnm=5000" in blocks["adaptive_status"], "Wrong trial identity/cap")
    need("overall_strength=0.000000" in blocks["adaptive_config"], "Nonzero automatic preset")
    for side in ["left", "right"]:
        for field in ["enabled", "strength", "flex_amplitude_nm", "ext_amplitude_nm", "torque_limit_nm"]:
            need(f"ADAPT config {side} {field} 0.000000" in blocks["adaptive_config"], "Nonzero boot default")
    need(re.search(r"Armed/tick:\s+no/", blocks["control_status"]), "Controller unexpectedly armed")
    return {"boot": boots[0][0], "state": "IDLE", "run_requested": False, "output_allowed": False,
            "active_limit_mnm": 5000, "internal_defaults_zero": True, "manual_ram_zero_required": True}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--flash", action="store_true")
    args = p.parse_args()
    need(sha(PACKAGE.read_bytes()) == PACKAGE_SHA, "Pinned package hash changed")
    with zipfile.ZipFile(PACKAGE) as z:
        manifest = json.loads(z.read("trial-manifest.json"))
        need(manifest["version"] == "0.11.6" and manifest["strategy_version"] == 5
             and manifest["active_max_mnm"] == 5000, "Wrong trial manifest")
        for name, info in manifest["files"].items():
            need(sha(z.read(name)) == info["sha256"], "Package member hash changed: " + name)
        app = z.read("firmware/twai_network_master.bin")
        partition = z.read("firmware/partition_table/partition-table.bin")
    with zipfile.ZipFile(ROOT / "output/adaptive/20261104/ESP-EXO-0.11.5-nominal-trace-peak-candidate.zip") as old:
        need(partition == old.read("firmware/partition_table/partition-table.bin"), "Partition layout differs from previous delivery")
    port_identity()
    print("PASS pinned package/all members, unchanged partition and physical COM5 USB identity", flush=True)
    if not args.flash:
        return
    dest = ROOT / "output/adaptive/20261105" / ("trial_deployment_" + datetime.now().strftime("%H%M%S_%f"))
    dest.mkdir()
    (dest / "app.bin").write_bytes(app)
    result = {"package_sha256": PACKAGE_SHA, "version": "0.11.6", "strategy_version": 5,
              "operator_report": "动力已切断，USB已接好，串口已释放", "port": "COM5",
              "flash_verified": False, "partition_nvs_phy_unchanged": False, "error": None}
    try:
        with (dest / "flash.log").open("xb") as log:
            def run(label, after, command):
                port_identity()
                cmd = [sys.executable, "-m", "esptool", "--chip", "esp32s3", "--port", "COM5", "--baud", "460800",
                       "--before", "default_reset", "--after", after] + command
                log.write(("\n" + label + "\n").encode())
                log.flush()
                proc = subprocess.run(cmd, cwd=dest, stdout=log, stderr=subprocess.STDOUT, timeout=90)
                need(proc.returncode == 0, label + " failed; motor power remains off; see flash.log")
                print("PASS " + label, flush=True)
            run("backup partition/NVS/PHY", "no_reset", ["read_flash", "0x8000", "0x8000", "metadata_before.bin"])
            before = (dest / "metadata_before.bin").read_bytes()
            need(before[:len(partition)] == partition, "On-device partition table differs; app was not written")
            run("write app only", "no_reset", ["write_flash", "0x10000", "app.bin"])
            run("verify app", "no_reset", ["verify_flash", "0x10000", "app.bin"])
            result["flash_verified"] = True
            run("verify metadata and reboot", "hard_reset", ["read_flash", "0x8000", "0x8000", "metadata_after.bin"])
        need(before == (dest / "metadata_after.bin").read_bytes(), "Partition/NVS/PHY bytes changed")
        result["partition_nvs_phy_unchanged"] = True
        result["metadata_sha256"] = sha(before)
        result["readback"] = readback(dest)
    except Exception as exc:
        result["error"] = str(exc)
    finally:
        (dest / "result.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
        print(json.dumps({"directory": str(dest), **result}, ensure_ascii=False), flush=True)
    need(result["error"] is None, "Deployment incomplete; see saved result and keep motor power off")


if __name__ == "__main__":
    main()
