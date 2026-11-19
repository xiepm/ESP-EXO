"""Analyze synthetic native-C probe traces only; never opens a device.

Usage: python tools/analyze_afo_phase_probe.py EVIDENCE_DIRECTORY
Outputs are exclusive-create. Phase truth is defined by the synthetic input,
not inferred from the operator's historical 10 Hz UART capture.
"""
import csv
import hashlib
import json
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def circular_cycles(value):
    return (value + 0.5) % 1.0 - 0.5


def summarize(rows):
    result = {}
    for case in range(3):
        selected = [r for r in rows if r["case"] == case]
        assert len(selected) == 6000
        assert all(r["timestamp_us"] == 10000 + n * 10000
                   for n, r in enumerate(selected))
        moving = [r for r in selected if abs(r["true_velocity_deg_s"]) >= 3]
        opposed = [r for r in moving if abs(r["limited_nm"]) > 0.0001
                   and r["limited_nm"] * r["true_velocity_deg_s"] < 0]
        nonzero = [r for r in selected if abs(r["limited_nm"]) > 0.0001]
        assert all(r["limited_nm"] == 0 for r in selected if r["segment"] == 2)
        peak = max(opposed, key=lambda r: abs(r["limited_nm"]), default=None)
        result[str(case)] = {
            "nonzero_ticks_above_0_0001": len(nonzero),
            "first_nonzero_us": nonzero[0]["timestamp_us"] if nonzero else None,
            "opposed_ticks": len(opposed),
            "max_abs_opposed_nm": abs(peak["limited_nm"]) if peak else 0,
            "strongest_opposed_sample": peak,
            "old_permit_after_hold_nonzero_ticks": 0,
        }
    return result


def main():
    root = Path(sys.argv[1]).resolve(strict=True)
    names = {
        "10nm_baseline": "trace.csv",
        "10nm_candidate": "trace_after.csv",
        "5nm_baseline": "trace_5nm_baseline.csv",
        "5nm_candidate": "trace_5nm_candidate.csv",
    }
    data = {}
    report = {
        "scope": "synthetic 100 Hz production-C engine; no plant, CAN, runtime adapter or hardware",
        "candidate_status": "isolated, not retained in product sources, not deployed",
        "opposed_definition": "abs(true velocity)>=3 deg/s, abs(limited)>0.0001 Nm, product<0",
        "limits": "opposed power is allowed by current configuration; not every opposed sample is an implementation defect",
        "variants": {},
    }
    for label, filename in names.items():
        path = root / filename
        with path.open(newline="", encoding="utf-8") as stream:
            rows = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(stream)]
        assert len(rows) == 18000 and all(math.isfinite(v) for r in rows for v in r.values())
        data[label] = rows
        report["variants"][label] = {
            "file": filename,
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            "cases": summarize(rows),
        }
    baseline = data["5nm_baseline"]
    peak = next(r for r in baseline if r["case"] == 0 and r["timestamp_us"] == 5590000)
    report["periodic_case_5590ms"] = {
        "sample": peak,
        "phase_lead_cycles": circular_cycles(peak["phase"] - peak["true_phase"]),
        "equivalent_seconds_at_period_1_4": 1.4 * circular_cycles(peak["phase"] - peak["true_phase"]),
        "next_peak_us": 5610000,
        "next_event_confirmation_us": 5650000,
    }
    # This candidate changes only the event acceptance gate, not phase learning.
    for case in range(3):
        before = [r for r in data["5nm_baseline"] if r["case"] == case]
        after = [r for r in data["5nm_candidate"] if r["case"] == case]
        assert all(a["phase"] == b["phase"] for a, b in zip(before, after))
    with (root / "phase_analysis.json").open("x", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)

    fig, axes = plt.subplots(3, 2, figsize=(14, 9), sharex="col", constrained_layout=True)
    for col, case in enumerate([0, 2]):
        before = [r for r in data["5nm_baseline"] if r["case"] == case
                  and 4100000 <= r["timestamp_us"] <= 8600000]
        after = [r for r in data["5nm_candidate"] if r["case"] == case
                 and 4100000 <= r["timestamp_us"] <= 8600000]
        x = [r["timestamp_us"] / 1e6 for r in before]
        axes[0, col].set_title(["Periodic analytic input", "Dwell + synthetic velocity disturbance"][col])
        for key, label, color in [("true_phase", "Known input phase", "#183b56"),
                                  ("phase", "AFO corrected phase", "#c34832")]:
            y = [r[key] for r in before]
            # Break phase-wrap lines; do not draw an artificial diagonal.
            for n in range(len(y) - 1, 0, -1):
                if abs(y[n] - y[n-1]) > .5:
                    y[n] = float("nan")
            axes[0, col].plot(x, y, label=label, color=color, lw=1.6)
        axes[1, col].plot(x, [r["true_velocity_deg_s"] for r in before], label="Known input velocity", color="#183b56")
        axes[1, col].plot(x, [r["reported_velocity_deg_s"] for r in before], label="Reported velocity", color="#558b66", alpha=.7)
        axes[2, col].plot(x, [r["limited_nm"] for r in before], label="Baseline limited", color="#c34832", lw=1.8)
        axes[2, col].plot(x, [r["limited_nm"] for r in after], label="Candidate limited", color="#307dae", linestyle="--", lw=1.6)
        bad = [r for r in before if abs(r["true_velocity_deg_s"]) >= 3
               and abs(r["limited_nm"]) > .0001 and r["limited_nm"] * r["true_velocity_deg_s"] < 0]
        axes[2, col].scatter([r["timestamp_us"]/1e6 for r in bad], [r["limited_nm"] for r in bad],
                             s=14, color="#b10e32", label="Baseline opposing input motion", zorder=3)
        for ax in axes[:, col]:
            ax.grid(alpha=.18)
            ax.legend(loc="lower left", fontsize=8)
        axes[1, col].axhline(0, color="grey", linewidth=.6)
        axes[2, col].axhline(0, color="grey", linewidth=.6)
        axes[2, col].set_ylim(-5.4, 5.4)
        axes[2, col].set_xlabel("Synthetic timestamp (s)")
    axes[0, 0].set_ylabel("Phase (cycle)")
    axes[1, 0].set_ylabel("Velocity (deg/s)")
    axes[2, 0].set_ylabel("Computed torque (Nm)")
    fig.suptitle("5 Nm numerical amplitude/cap: phase and limited torque\nHost synthetic diagnostic only; no motor commands or human-test acceptance", fontsize=14)
    with (root / "phase_diagnostic_5nm.png").open("xb") as stream:
        fig.savefig(stream, format="png", dpi=150)
    plt.close(fig)
    print(json.dumps({k: {c: {f: s[f] for f in ["opposed_ticks", "max_abs_opposed_nm", "first_nonzero_us"]}
                         for c, s in v["cases"].items()} for k, v in report["variants"].items()}, indent=2))


if __name__ == "__main__":
    main()
