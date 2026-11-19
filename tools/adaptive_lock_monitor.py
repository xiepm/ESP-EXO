"""Read-only exact-cycle lock-loss capture/parser for 0.11.2; no motor commands."""
import argparse
import csv
import json
import math
from pathlib import Path
import re
import time
from adaptive_monitor import parse as parse_adapt

FIELDS = "timestamp_us,sequence,side,system_state,permit_epoch,generation,reasons,residual,phase_error_rad,frequency_error,event_age_s,intervals,q_deg,v_deg_s,locked,rearm,faults,final_ff_nm,command_valid,dropped".split(",")
REASONS = {1: "input", 2: "time", 4: "stopped", 8: "insufficient_intervals", 16: "event_age",
           32: "frequency_mismatch", 64: "phase_error", 128: "afo_residual", 256: "emergency"}


def parse(source):
    rows, invalid = [], 0
    text = source.read_text(encoding="utf-8", errors="replace")
    for line in text.splitlines():
        match = re.search(r"(ADLOCKSTATE|ADLOCK),(-?\d+,.*)", line)
        if not match:
            continue
        try:
            values = [float(v) for v in match[2].split(",")]
            if len(values) != len(FIELDS) or not all(math.isfinite(v) for v in values):
                raise ValueError("length/nonfinite")
            row = dict(zip(FIELDS, values))
            if row["side"] not in (0, 1) or row["reasons"] != int(row["reasons"]) or int(row["reasons"]) & ~511:
                raise ValueError("invalid side/reasons")
            row["kind"] = match[1]
            rows.append(row)
        except ValueError:
            invalid += 1
    states = [r for r in rows if r["kind"] == "ADLOCKSTATE"]
    boundary = min((r["timestamp_us"] for r in states), default=None)
    events = [r for r in rows if r["kind"] == "ADLOCK" and (boundary is None or r["timestamp_us"] >= boundary)]
    summary = {"source": str(source), "invalid_rows": invalid, "capture_boundary_us": boundary,
               "historical_events_discarded": sum(r["kind"] == "ADLOCK" for r in rows) - len(events),
               "events": [{**r, "reason_names": [name for bit, name in REASONS.items() if int(r["reasons"]) & bit]} for r in events],
               "brownout_observed": "Brownout detector was triggered" in text,
               "panic_observed": "Guru Meditation" in text,
               "note": "Exact 100Hz loss records; ADLOCKSTATE is a snapshot; no interpolation or inferred causes"}
    with source.with_suffix(".lock.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, ["kind"] + FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    source.with_suffix(".lock.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    parse_adapt(source, source.with_suffix(".csv"))
    print(json.dumps({"lock_rows": len(rows), "new_loss_events": len(events), "invalid_rows": invalid,
                      "reasons": {name: sum(int(r["reasons"]) & bit != 0 for r in events) for bit, name in REASONS.items()}}, ensure_ascii=False))


def capture(port, seconds, destination):
    import serial
    if not 1 <= seconds <= 180:
        raise ValueError("Capture must be 1..180 seconds")
    connection = serial.Serial(port=None, baudrate=115200, timeout=.05, write_timeout=1)
    connection.dtr = False
    connection.rts = False
    connection.port = port
    with destination.open("xb") as raw:
        def receive(duration):
            data = bytearray()
            end = time.monotonic() + duration
            while time.monotonic() < end:
                chunk = connection.read(max(1, min(connection.in_waiting, 4096)))
                data.extend(chunk)
                raw.write(chunk)
            raw.flush()
            return data.decode("utf-8", errors="replace")
        try:
            connection.open()
            receive(.3)
            connection.write(b"system_status\r")
            state = receive(.8)
            if not all(re.search(p, state) for p in (r"State:\s+IDLE\b", r"Output allowed:\s+no\b", r"Run requested:\s+no\b")):
                raise RuntimeError("Expected closed IDLE before capture")
            connection.write(b"adaptive_status\r")
            if "ADLOCKSTATE," not in receive(1.0):
                raise RuntimeError("Current firmware does not return exact-cycle diagnostics")
            print("CAPTURE READY: read-only IDLE; no START/enable/parameter writes", flush=True)
            start, next_dump = time.monotonic(), 0.0
            while time.monotonic() - start < seconds:
                now = time.monotonic()
                if now >= next_dump:
                    connection.write(b"adaptive_dump\radaptive_faults\r")
                    next_dump = now + 1.0
                receive(.1)
            connection.write(b"adaptive_dump\radaptive_faults\rsystem_status\r")
            final = receive(2.0)
            if not all(re.search(p, final) for p in (r"State:\s+IDLE\b", r"Output allowed:\s+no\b", r"Run requested:\s+no\b")):
                raise RuntimeError("Final closed IDLE was not verified")
        finally:
            connection.close()
    parse(destination)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="action", required=True)
    offline = sub.add_parser("parse")
    offline.add_argument("source", type=Path)
    live = sub.add_parser("capture")
    live.add_argument("--port", required=True)
    live.add_argument("--seconds", type=int, default=55)
    live.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.action == "parse":
        parse(args.source)
    else:
        capture(args.port, args.seconds, args.output)
