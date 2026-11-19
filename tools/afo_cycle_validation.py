"""Offline native-C gait-cycle acceptance. Never opens a device.

Inputs are the preserved 10 Hz recordings interpolated to 100 Hz by
analyze_walk_recordings.py. The angle-only branch differentiates only the
previous/current angle; the controller never sees future extrema. SciPy peak
segmentation is an OFFLINE evaluator, independent of controller lock flags.
"""
import argparse
import collections
import csv
import json
import subprocess
from pathlib import Path
import numpy as np
from scipy.signal import find_peaks


def read(path):
    with path.open(encoding="utf-8-sig") as stream:
        return list(csv.DictReader(stream))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("input", type=Path)
    ap.add_argument("baseline", type=Path)
    ap.add_argument("candidate", type=Path)
    ap.add_argument("output", type=Path)
    ap.add_argument("--minimum-angle", type=float, default=-40)
    ap.add_argument("--maximum-angle", type=float, default=90)
    ap.add_argument("--maximum-velocity", type=float, default=400)
    ap.add_argument("--baseline-label", default="0.11.20")
    args = ap.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    limits = dict(minimum_angle_deg=args.minimum_angle, maximum_angle_deg=args.maximum_angle,
                  maximum_velocity_deg_s=args.maximum_velocity, baseline_label=args.baseline_label)
    (args.output / "limits.json").write_text(json.dumps(limits, indent=2))
    original = read(args.input)
    angles = []
    previous = None
    for row in original:
        out = dict(row)
        for side in ("left", "right"):
            same = previous is not None and previous["trial"] == row["trial"]
            dt = (int(row["time_us"]) - int(previous["time_us"])) / 1e6 if same else .01
            out[side + "_v"] = (float(row[side + "_q"]) - float(previous[side + "_q"])) / dt if same else 0
        angles.append(out)
        previous = row
    angle_input = args.output / "angle_only_input.csv"
    with angle_input.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=angles[0].keys())
        writer.writeheader()
        writer.writerows(angles)
    summaries, cycles = [], []
    plot_groups = {}
    for mode, source in [("angle_only", angle_input), ("recorded_velocity", args.input)]:
        traces = {}
        for name, executable in [("baseline", args.baseline), ("candidate", args.candidate)]:
            trace = args.output / f"{mode}_{name}.csv"
            subprocess.run([str(executable.resolve()), str(source.resolve()), str(trace.resolve())], check=True)
            groups = collections.defaultdict(list)
            for row in read(trace):
                groups[(int(row["trial"]), int(row["side"]))].append(row)
            traces[name] = groups
        for key, rows in traces["candidate"].items():
            base = traces["baseline"][key]
            q = np.array([float(r["q"]) for r in rows])
            v = np.array([float(r["v"]) for r in rows])
            tau = np.array([float(r["torque"]) for r in rows])
            old = np.array([float(r["torque"]) for r in base])
            outside = (q < args.minimum_angle) | (q > args.maximum_angle) | (np.abs(v) > args.maximum_velocity)
            assert np.all(np.isfinite(tau)) and np.max(np.abs(tau)) <= 15.0001
            assert np.all(tau[outside] == 0)
            assert np.all(tau * v >= -1e-5), "Reverse added torque"
            assert not any(int(r["rearm"]) for r in rows), "Unexpected hard latch"
            # Fixed before evaluating torque: >=10 deg prominence, >=400 ms.
            peaks, _ = find_peaks(q, prominence=10, distance=40)
            counts = collections.Counter()
            for index, (a, b) in enumerate(zip(peaks, peaks[1:])):
                prior_range = np.flatnonzero(outside[:a + 1])
                last = int(prior_range[-1]) if len(prior_range) else -1
                # Two complete ipsilateral intervals require three peaks.
                category = "window" if np.any(outside[a:b + 1]) else "acquisition" if np.sum((peaks <= a) & (peaks > last)) < 3 else "eligible"
                positive, negative = float(np.max(tau[a:b])), float(np.min(tau[a:b]))
                passed = positive > .01 and negative < -.01
                baseline_pass = np.max(old[a:b]) > .01 and np.min(old[a:b]) < -.01
                counts[category] += 1
                if category == "eligible":
                    counts["candidate_pass"] += passed
                    counts["baseline_pass"] += bool(baseline_pass)
                cycles.append(dict(mode=mode, walk=key[0] + 1, side=key[1], cycle=index + 1,
                    start_s=a / 100, end_s=b / 100, category=category,
                    positive_peak_nm=positive, negative_peak_nm=negative,
                    both_directions=passed, baseline_both_directions=bool(baseline_pass)))
            summaries.append(dict(mode=mode, walk=key[0] + 1, side=key[1],
                cycles=len(peaks) - 1, **dict(counts), peak_nm=float(np.max(np.abs(tau)))))
            if mode == "angle_only":
                plot_groups[key] = (q, tau, old, peaks, outside)
    with (args.output / "cycles.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=cycles[0].keys())
        writer.writeheader()
        writer.writerows(cycles)
    (args.output / "cycle_summary.json").write_text(json.dumps(summaries, indent=2))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(4, 2, figsize=(16, 10), layout="constrained")
    for (trial, side), (q, tau, old, peaks, outside) in plot_groups.items():
        t = np.arange(len(q)) / 100
        angle, torque = axes[2 * side, trial], axes[2 * side + 1, trial]
        angle.plot(t, q, color="#334e68", lw=.9)
        angle.plot(t[peaks], q[peaks], ".", color="#d47716", ms=3)
        angle.axhline(args.maximum_angle, color="#d47716", ls="--", lw=.7)
        angle.set_title(f"Walk {trial+1}, {'left' if side==0 else 'right'}")
        angle.set_ylabel("Angle (deg)")
        torque.plot(t, old, color="#b4b7ba", lw=.7, label=args.baseline_label)
        torque.plot(t, tau, color="#00877c", lw=.8, label="0.11.21 candidate")
        torque.set_ylabel("Added command (N m)")
        torque.set_xlabel("Elapsed time (s)")
        torque.legend(fontsize=8, loc="upper right")
        angle.grid(alpha=.2);torque.grid(alpha=.2)
    fig.suptitle("Recorded-angle-only replay / native C controller\nDots: independent offline cycle boundaries; commands are NOT measured joint torque", fontsize=12)
    fig.savefig(args.output / "cycle_comparison.png", dpi=150)
    print(json.dumps(summaries, indent=2))
    assert all(s["candidate_pass"] == s["eligible"] for s in summaries), "Missed eligible stride"


if __name__ == "__main__":
    main()
