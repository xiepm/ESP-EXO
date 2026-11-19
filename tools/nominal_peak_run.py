"""Plan or execute MCU-timed pulses on a secured, mechanically loaded off-body bench.

Default is plan-only. Execution never captures a zero, writes NVS, clears faults
for another attempt, retries a pulse, or sends unrestricted torque commands.
Motor telemetry is an estimate, not an independent joint torque measurement.
"""
import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import re
import time

ROOT = Path(__file__).resolve().parents[1]
STAGES = (1, 3, 6, 10, 15)
PLAN = {
    "fixture_type": "restrained-output",
    "excluded_fixture": "waist-only fixation with freely rotating leg links",
    "basis": "nominal command and motor-reported torque estimate",
    "sequence": [{"side": side, "signed_nm": sign * peak}
                 for peak in STAGES for side in ("left", "right") for sign in (1, -1)],
    "mcu_profile": 2,
    "mcu_waveform": "0.5 s zero; symmetric rise/fall: 2 Nm/s below 1 Nm, 20 Nm/s above; plateau 0.1 s; permanent zero",
    "maximum_pulse_seconds": 3.0,
    "minimum_rest_seconds": 10,
    "sent_peak_tolerance_nm": 0.01,
    "estimate_plateau_tolerance": "max(0.25 Nm, 20% of requested magnitude)",
    "minimum_fresh_plateau_samples": 6,
    "pre_start_max_temperature_c": 45,
    "firmware_guard": "both sides: displacement<=2 deg, speed<=20 deg/s, temperature<=50 C, feedback age<=50 ms",
    "stop_policy": "first fault/rejection/missing result/failed nominal tolerance stops the run; no retry",
    "human_start": False,
    "stationary_preflight": "one fresh 3s 20Hz BLE window; paired valid samples; no drops; angle range<=0.1deg; abs mean speed<=1deg/s; median abs speed<=5deg/s; raw abs speed<=20deg/s; abs torque<=0.5Nm; no retries",
}


def need(ok, message):
    if not ok:
        raise RuntimeError(message)


def field(text, label):
    match = re.search(r"^\s*" + re.escape(label) + r":\s*(.+)$", text, re.M)
    need(match is not None, "Missing status field: " + label)
    return match[1].strip()


def verify_closed_idle(text):
    need(field(text, "State") == "IDLE" and field(text, "Run requested") == "no"
         and field(text, "Output allowed") == "no", "System is not closed IDLE")
def verify_idle(text):
    verify_closed_idle(text)
    need(field(text, "Runtime faults").startswith("0;"), "Runtime fault recorded; stop and review")


def parse_peak(text):
    rows = re.findall(r"^BENCHPEAK (.+)$", text, re.M)
    need(len(rows) == 1, "Missing/ambiguous retained peak result")
    result = {k: float(v) for k, v in re.findall(r"(\w+)=([^\s]+)", rows[0])}
    required = "profile initialized complete fault side requested generated_peak sent_peak estimate_peak plateau_mean plateau_samples samples max_displacement max_speed max_temperature start_us last_us".split()
    need(all(k in result and math.isfinite(result[k]) for k in required), "Incomplete/nonfinite peak result")
    return result


def verify_peak(text, side, peak, previous_start):
    result = parse_peak(text)
    need(result["profile"] == PLAN["mcu_profile"], "Wrong MCU pulse profile")
    need(result["initialized"] == result["complete"] == 1 and result["fault"] == 0,
         "Pulse incomplete or guard tripped: " + str(result))
    need(result["side"] == (0 if side == "left" else 1) and abs(result["requested"] - peak) < 0.001,
         "Peak result does not belong to requested side/magnitude")
    need(result["start_us"] > previous_start and result["last_us"] > result["start_us"], "Stale peak result")
    expected_seconds = .6 + 2 * (min(abs(peak), 1) / 2 + max(abs(peak) - 1, 0) / 20)
    elapsed_seconds = (result["last_us"] - result["start_us"]) / 1e6
    need(abs(elapsed_seconds - expected_seconds) <= .03, "MCU pulse duration differs from the selected profile")
    for key in ("sent_peak", "generated_peak"):
        need(abs(result[key] - abs(peak)) <= PLAN["sent_peak_tolerance_nm"], "Requested peak not sent")
    need(result["plateau_samples"] >= 6, "Insufficient fresh motor plateau samples")
    tolerance = max(.25, .2 * abs(peak))
    need(abs(result["plateau_mean"] - abs(peak)) <= tolerance,
         "Motor estimate plateau outside declared nominal tolerance")
    need(abs(result["estimate_peak"] - abs(peak)) <= tolerance,
         "Motor estimate peak outside declared nominal tolerance")
    need(result["max_displacement"] <= 2 and result["max_speed"] <= 20
         and 0 <= result["max_temperature"] <= 50, "Mechanical/thermal guard result invalid")
    return result


class Console:
    def __init__(self, destination):
        import serial
        from serial.tools import list_ports
        matches = [p for p in list_ports.comports() if p.device == "COM5"]
        need(len(matches) == 1 and matches[0].vid == 0x1A86 and matches[0].pid == 0x55D3
             and matches[0].serial_number == "5C94152132", "COM5 identity mismatch")
        self.raw = (destination / "uart.log").open("xb")
        self.port = serial.Serial(port=None, baudrate=115200, timeout=.05, write_timeout=1)
        self.port.dtr = self.port.rts = False
        self.port.port = "COM5"
        try:
            self.port.open()
        except BaseException:
            self.raw.close()
            raise

    def receive(self, seconds):
        end = time.monotonic() + seconds
        data = bytearray()
        while time.monotonic() < end:
            part = self.port.read(max(1, min(self.port.in_waiting, 4096)))
            self.raw.write(part)
            data.extend(part)
        self.raw.flush()
        text = re.sub(r"\x1b\[[0-9;]*m", "", data.decode("utf-8", errors="replace")).replace("\r", "")
        need(not any(s in text for s in ("Brownout detector", "Guru Meditation", "ESP-ROM:", "rst:0x")),
             "MCU reset/panic during observation")
        return text

    def command(self, command, seconds=.4):
        allowed = {"system_status", "control_status", "assist_status", "adaptive_status", "adaptive_config",
                   "adaptive_faults", "ble_status",
                   "can_status", "joint_status left", "joint_status right", "motor A", "motor B", "status",
                   "start", "stop", "bench_peak_status", "bench_peak_trace", "motion_mode damping constant",
                   "joint_command left 0 0 0 0 0", "joint_command right 0 0 0 0 0"}
        allowed.update(f"bench_peak {side} {sign * peak}" for peak in STAGES
                       for side in ("left", "right") for sign in (1, -1))
        need(command in allowed, "Command is outside runner allowlist")
        self.raw.write(("\n[HOST " + datetime.now().isoformat() + "] " + command + "\n").encode())
        self.port.write((command + "\r").encode("ascii"))
        return self.receive(seconds)

    def close(self):
        self.port.close()
        self.raw.close()


def check_rest(c):
    for motor in ("A", "B"):
        c.command("motor " + motor, .15)
        text = c.command("status")
        need(field(text, "Actuator") == motor and field(text, "Valid") == "YES"
             and field(text, "Online") == "YES", "Motor identity/feedback invalid")
        need(field(text, "Mode status") == "0 (REST)" and field(text, "Enabled").startswith("no "),
             "Motor has not confirmed REST")
        need(int(field(text, "Feedback age").split()[0]) <= 50000, "Stale motor feedback")
        need(int(field(text, "Fault raw"), 16) == 0, "Motor fault")
        need(0 <= float(field(text, "Temperature").split()[0]) <= 45, "Wait for motor temperature <=45 C")


def preflight(c, expected_boot):
    verify_idle(c.command("system_status"))
    need(int(field(c.command("ble_status"), "Boot ID"), 16) == expected_boot, "Boot changed; RAM zero invalid")
    need("ACTIVE_DEPLOYMENT=OFF_BODY_BENCH_CANDIDATE active_limit_mnm=15000" in c.command("adaptive_status"),
         "Wrong candidate identity/ceiling")
    need(parse_peak(c.command("bench_peak_status"))["profile"] == PLAN["mcu_profile"], "Wrong MCU peak profile; do not START")
    assist = c.command("assist_status")
    need(field(assist, "Selected").startswith("TRANSPARENT / DISABLED strategy=1 ")
         and field(assist, "Levels L/R") == "0 % / 0 %", "Expected disabled zero-level assistance")
    can = c.command("can_status")
    need(field(can, "State") == "RUNNING" and int(field(can, "TX error counter")) < 128
         and int(field(can, "RX error counter")) < 128, "CAN not healthy")
    check_rest(c)
    for side, motor, direction in (("left", "B", "-1"), ("right", "A", "+1")):
        joint = c.command("joint_status " + side)
        need(field(joint, "Mapping") == f"actuator {motor} (valid)"
             and field(joint, "Direction") == f"{direction} (valid)"
             and field(joint, "Joint valid") == "YES", "Joint mapping/direction/zero invalid")
        need("(valid, RAM only)" in field(joint, "Software zero"), "Current RAM zero required")
        age = re.search(r"valid / online, count=\d+, age=(\d+) us", field(joint, "Feedback"))
        need(age is not None and int(age[1]) <= 50000, "Stale joint feedback")
        values = re.search(r"q=([-\d.]+) deg dq=([-\d.]+) deg/s tau=([-\d.]+) Nm", field(joint, "Joint"))
        need(values is not None and abs(float(values[2])) <= 20 and abs(float(values[3])) <= .5,
             "Fixture not stationary or torque not near zero")
    from stationary_ble_gate import check as check_stationary
    check_stationary(Path(c.raw.name).parent, expected_boot)
    verify_idle(c.command('system_status'))
    need(int(field(c.command('ble_status'), 'Boot ID'), 16) == expected_boot,
         'Boot changed after stationary capture')
    return parse_peak(c.command('bench_peak_status')).get('trace_schema') == 1



def execute(args):
    validate_fixture(args)
    need(args.fixture_confirmed and args.expected_boot is not None,
         "Execution needs current secured-load/no-contact confirmation and verified boot ID")
    destination = ROOT / "output/adaptive/20261104" / ("nominal_peak_" + datetime.now().strftime("%Y%m%d_%H%M%S"))
    destination.mkdir()
    result = {"created_at": datetime.now().astimezone().isoformat(), "plan": PLAN,
              "expected_boot": hex(args.expected_boot), "operator_fixture_confirmed": True,
              "stages": [], "passed": False, "error": None, "cleanup_error": None}
    c = None
    touched = False
    try:
        c = Console(destination)
        c.receive(.5)
        previous_start = 0
        for item in PLAN["sequence"]:
            trace_supported = preflight(c, args.expected_boot)
            side, peak = item["side"], item["signed_nm"]
            entry = {**item, "started_at": datetime.now().isoformat()}
            result["stages"].append(entry)
            touched = True
            need("OK: bench_peak prepare ESP_OK" in c.command(f"bench_peak {side} {peak}"), "Peak preparation rejected")
            prepared = c.command("control_status")
            need(field(prepared, "Selected").startswith("TORQUE + BENCH_PEAK "), "Peak selection readback mismatch")
            for checked_side in ("left", "right"):
                selected = field(prepared, "Selected " + checked_side.upper())
                match = re.fullmatch(r"q=([-\d.]+) deg dq=([-\d.]+) deg/s tau=([-\d.]+) Nm kp=([-\d.]+) kd=([-\d.]+)", selected)
                need(match is not None, "Missing five-tuple readback")
                q, dq, tau, kp, kd = map(float, match.groups())
                need(q == dq == kp == kd == 0 and abs(tau - (peak if checked_side == side else 0)) < .001,
                     "Prepared command differs from finite-pulse plan")
            print(f"START bounded pulse {side} {peak:+d} Nm", flush=True)
            entry["start_reply"] = c.command("start", 4.5)
            entry["peak_reply"] = c.command("bench_peak_status")
            # STOP is unconditional and independent of acceptance of the result.
            c.command("stop", 1.0)
            entry["system_after"] = c.command("system_status")
            verify_closed_idle(entry["system_after"])
            check_rest(c)
            entry["rest_confirmed"] = True
            entry["peak_observed"] = parse_peak(entry["peak_reply"])
            if trace_supported:
                trace = c.command('bench_peak_trace', 10)
                entry['trace'] = trace
                need('BENCHTRACE_END' in trace and 'dropped=0' in trace,
                     'Trace incomplete; retain evidence and stop')
            entry["peak"] = verify_peak(entry["peak_reply"], side, peak, previous_start)
            verify_idle(entry["system_after"])
            previous_start = entry["peak"]["start_us"]
            entry["passed"] = True
            print(f"PASS {side} {peak:+d}: sent={entry['peak']['sent_peak']:.3f}, motor plateau={entry['peak']['plateau_mean']:.3f} Nm", flush=True)
            (destination / "result.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
            c.receive(10)
        result["passed"] = len(result["stages"]) == len(PLAN["sequence"]) and all(s.get("passed") for s in result["stages"])
    except BaseException as error:
        result["error"] = repr(error)
        result["passed"] = False
    finally:
        if c:
            try:
                if touched:
                    c.command("stop", 1.0)
                    verify_closed_idle(c.command("system_status"))
                    check_rest(c)
                    result["cleanup_rest_confirmed"] = True
                    for command in ("motion_mode damping constant", "joint_command left 0 0 0 0 0", "joint_command right 0 0 0 0 0"):
                        need("OK:" in c.command(command), "Cleanup selection rejected")
            except BaseException as error:
                result["cleanup_error"] = repr(error)
                result["passed"] = False
            finally:
                c.close()
        (destination / "result.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
        print(json.dumps({"directory": str(destination), "passed": result["passed"],
                          "error": result["error"], "cleanup_error": result["cleanup_error"]}, ensure_ascii=False), flush=True)
    return 0 if result["passed"] else 1


def validate_fixture(args):
    need(getattr(args, 'fixture_type', None) == 'restrained-output',
         'This peak runner requires a mechanically restrained joint output. '
         'Waist-only fixation/free leg links require a separate motion test plan; no device opened.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--fixture-confirmed", action="store_true")
    parser.add_argument("--fixture-type", choices=("restrained-output", "free-leg"),
                        help="Actual output constraint; waist fixation alone is free-leg")
    parser.add_argument("--expected-boot", type=lambda value: int(value, 0))
    args = parser.parse_args()
    if not args.execute:
        print(json.dumps(PLAN, ensure_ascii=False, indent=2))
        return 0
    return execute(args)


if __name__ == "__main__":
    raise SystemExit(main())
