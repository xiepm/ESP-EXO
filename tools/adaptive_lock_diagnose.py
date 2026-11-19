"""Offline diagnostics of existing ADAPT CSV outputs; no interpolation or I/O to devices.

CSV snapshots omit residual, target phase error and stable-interval count, so this
tool reports visible candidate blockers and never claims a unique root cause.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path


def diagnose(source):
    with source.open(encoding="utf-8-sig") as stream:
        rows = [{key: float(value) for key, value in row.items()} for row in csv.DictReader(stream)]
    if not rows or not all(math.isfinite(v) for row in rows for v in row.values()):
        raise ValueError("empty or nonfinite input")
    required = {"timestamp_us", "side", "hz", "period_s", "locked", "event_timestamp_us",
                "accepted_events", "rejected_events", "faults", "q_deg", "v_deg_s", "dropped",
                "system_state", "command_valid", "final_ff_command_nm"}
    if not all(required <= row.keys() and row["side"] in (0, 1) for row in rows):
        raise ValueError("wrong ADAPT schema or side")
    boundary = rows[0]["timestamp_us"]
    result = {"source": str(source), "sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
              "capture_boundary_us": boundary, "raw_rows": len(rows),
              "method": "Existing firmware outputs only; discard history older than first adaptive_status; no resampling",
              "unobserved": ["AFO residual EWMA", "target offset error", "stable interval count",
                             "100Hz samples between log rows", "independent physical motion/torque"], "sides": {}}
    for side in (0, 1):
        unique = {}
        duplicates = 0
        for row in rows:
            if row["side"] != side or row["timestamp_us"] < boundary:
                continue
            stamp = row["timestamp_us"]
            if stamp in unique:
                if unique[stamp] != row:
                    raise ValueError("conflicting values at the same side/timestamp")
                duplicates += 1  # adaptive_status and the ring can contain the same snapshot.
            else:
                unique[stamp] = row
        data = sorted(unique.values(), key=lambda r: r["timestamp_us"])
        if not data:
            raise ValueError("missing side")
        frequencies = [abs(r["hz"]*r["period_s"]-1) if r["period_s"] > 0 else None for r in data]
        def reasons(row, error):
            visible = []
            if row["period_s"] <= 0:
                visible.append("no_valid_period")
            # Margin avoids claiming an exact crossing from rounded/asynchronous outputs.
            if error is not None and error >= .252:
                visible.append("frequency_period_mismatch_over_25pct")
            if row["period_s"] > 0 and (row["timestamp_us"]-row["event_timestamp_us"])/1e6 > 1.6*row["period_s"]+.05:
                visible.append("event_age_over_1p6_periods")
            if int(row["faults"]) & 3:
                visible.append("input_or_time_fault")
            return visible
        candidates = [reasons(row, error) for row, error in zip(data, frequencies)]
        unlocked = [i for i, row in enumerate(data) if row["locked"] == 0]
        transitions = []
        for index in range(1, len(data)):
            before, after = data[index-1], data[index]
            if before["locked"] == 1 and after["locked"] == 0:
                transitions.append({"time_s": (after["timestamp_us"]-boundary)/1e6,
                                    "observation_gap_ms": (after["timestamp_us"]-before["timestamp_us"])/1000,
                                    "frequency_hz": after["hz"], "period_s": after["period_s"],
                                    "frequency_period_error": frequencies[index],
                                    "visible_candidates": candidates[index]})
        all_reasons = sorted({reason for index in unlocked for reason in candidates[index]})
        bins = []
        for start_s in range(0, int((data[-1]["timestamp_us"]-boundary)/1e6)+1, 10):
            segment = [r for r in data if start_s <= (r["timestamp_us"]-boundary)/1e6 < start_s+10]
            if segment:
                bins.append({"start_s": start_s, "end_s": start_s+10, "rows": len(segment),
                             "locked_rows": sum(r["locked"] == 1 for r in segment),
                             "q_min_max_deg": [min(r["q_deg"] for r in segment), max(r["q_deg"] for r in segment)],
                             "events_first_last": [segment[0]["accepted_events"], segment[-1]["accepted_events"]]})
        run_begin = None
        longest = {"span_s": 0, "start_s": None, "end_s": None}
        for index, row in enumerate(data):
            # Do not claim continuous lock across a missing diagnostic interval.
            gap = index > 0 and row["timestamp_us"]-data[index-1]["timestamp_us"] > 150000
            if row["locked"] != 1 or gap:
                run_begin = None
            if row["locked"] == 1:
                if run_begin is None:
                    run_begin = row["timestamp_us"]
                span = (row["timestamp_us"]-run_begin)/1e6
                if span > longest["span_s"]:
                    longest = {"span_s": span, "start_s": (run_begin-boundary)/1e6,
                               "end_s": (row["timestamp_us"]-boundary)/1e6}
        result["sides"][str(side)] = {
            "rows": len(data), "identical_duplicates_removed": duplicates,
            "locked_rows": len(data)-len(unlocked), "unlocked_rows": len(unlocked),
            "unlocked_with_visible_candidate": sum(bool(candidates[index]) for index in unlocked),
            "unlocked_without_visible_candidate": sum(not candidates[index] for index in unlocked),
            "candidate_counts_not_exclusive": {reason: sum(reason in candidates[index] for index in unlocked) for reason in all_reasons},
            "lock_loss_transitions": transitions,
            "accepted_events_first_last": [data[0]["accepted_events"], data[-1]["accepted_events"]],
            "rejected_events_first_last": [data[0]["rejected_events"], data[-1]["rejected_events"]],
            "dropped_first_last": [data[0]["dropped"], data[-1]["dropped"]],
            "observed_system_states": sorted({int(r["system_state"]) for r in data}),
            "command_valid_rows": sum(r["command_valid"] != 0 for r in data),
            "nonzero_ff_rows": sum(r["final_ff_command_nm"] != 0 for r in data),
            "longest_observed_locked_run": longest,
            "ten_second_bins": bins,
        }
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    report = diagnose(args.source)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
