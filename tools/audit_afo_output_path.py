"""Offline output-path sensitivity audit. Never opens serial, BLE or CAN.

Consumes simulate_afo_recording.py outputs, compiles production C with host
transport fakes, preserves the same cycle boundaries, and reports failures.
Exit success means analysis completed, NOT that every stride is assisted.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
CORE = "exo_adaptive exo_afo exo_adaptive_event exo_adaptive_profile exo_adaptive_config exo_adaptive_safety".split()
EXTRA = "exo_adaptive_runtime exo_assist exo_swing_pulse exo_phase_profile exo_controller exo_bench_peak exo_command_composer sine_reference exo_sensor_snapshot gait_estimator".split()


def read_csv(path):
    with path.open(encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


def save(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("simulation_dir", type=Path)
    ap.add_argument("output", type=Path, help="New directory; existing evidence is never overwritten")
    ap.add_argument("--compiler", type=Path, default=ROOT / "build/native_toolchain/zig-windows-x86_64-0.13.0/zig.exe")
    args = ap.parse_args()
    source, out = args.simulation_dir.resolve(), args.output.resolve()
    cycles = [c for c in read_csv(source / "cycles.csv") if c["mode"] == "angle_only"]
    if not cycles:
        raise ValueError("Expected angle_only cycles from the original offline segmentation")
    out.mkdir(parents=True, exist_ok=False)
    exe = out / "chain.exe"
    command = [str(args.compiler.resolve()), "cc", "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror", "-pedantic",
               "-Itools/host_fakes", "-Itools/host_fakes/factory", "-Itools/host_fakes/system", "-Imain/include",
               "-DCONFIG_EXO_ADAPTIVE_ASSIST_MODE=1", "tools/adaptive_recorded_chain_replay.c",
               *[f"main/{name}.c" for name in CORE + EXTRA], "-o", str(exe)]
    save(out / "build_command.json", command)
    result = subprocess.run(command, cwd=ROOT, capture_output=True)
    (out / "build.log").write_bytes(result.stdout + result.stderr)
    result.check_returncode()
    summary, zero_cycles, first_faults, traces = [], [], [], {}
    for mode in ("angle_only", "recorded_velocity"):
        for scenario in ("fresh", "async", "quantized"):
            trace = out / f"{mode}_{scenario}.csv"
            subprocess.run([str(exe), str(source / f"{mode}_input.csv"), str(trace), scenario], cwd=ROOT, check=True)
            rows = read_csv(trace)
            traces[mode, scenario] = rows
            previous, seen = {}, set()
            for row in rows:
                key = (row["segment"], row["side"])
                stamp = int(row["time_us"]) - int(row["age_us"])
                before = previous.get(key)
                if int(row["faults"]) & 1 and key not in seen:
                    seen.add(key)
                    fault = dict(mode=mode, scenario=scenario, segment=int(key[0]), side=int(key[1]), row=row)
                    if before:
                        dt = (stamp - before[0]) / 1e6
                        fault.update(source_dt_s=dt, previous_row=before[1],
                                     displacement_deg=abs(float(row["q"]) - float(before[1]["q"])),
                                     allowed_displacement_deg=max(abs(float(row["v"])), abs(float(before[1]["v"]))) * dt + 3)
                    first_faults.append(fault)
                if before is None or stamp != before[0]:
                    previous[key] = (stamp, row)
            for side in (0, 1):
                leg = [r for r in rows if int(r["side"]) == side]
                group = [c for c in cycles if int(c["side"]) == side]
                counts = dict(mode=mode, scenario=scenario, side=side, total_cycles=len(group), any_gt_001=0,
                              both_gt_001=0, both_gt_1=0, post_initial_zero=0,
                              rearm_samples=sum(int(r["rearm"]) for r in leg),
                              maximum_age_us=max(int(r["age_us"]) for r in leg),
                              maximum_skew_us=max(int(r["pair_skew_us"]) for r in leg))
                for cycle in group:
                    window = [r for r in leg if r["segment"] == cycle["segment"] and
                              float(cycle["start_s"]) <= int(r["time_us"]) / 1e6 < float(cycle["end_s"])]
                    if not window:
                        raise ValueError(f"No replay samples for {cycle}")
                    torque = [float(r["torque"]) for r in window]
                    lo, hi = min(torque), max(torque)
                    any_assist = max(abs(lo), abs(hi)) > .01
                    counts["any_gt_001"] += any_assist
                    counts["both_gt_001"] += lo < -.01 and hi > .01
                    counts["both_gt_1"] += lo < -1 and hi > 1
                    if not any_assist and int(cycle["cycle"]) > 2:
                        counts["post_initial_zero"] += 1
                        zero_cycles.append(dict(mode=mode, scenario=scenario, cycle=cycle,
                                                peak_abs_nm=max(abs(lo), abs(hi)),
                                                faults=sorted(set(int(r["faults"]) for r in window)),
                                                lock_reasons=sorted(set(int(r["lock_reasons"]) for r in window)),
                                                maximum_lambda=max(float(r["lambda"]) for r in window)))
                summary.append(counts)
    save(out / "scenario_summary.json", summary)
    save(out / "post_initial_zero_cycles.json", zero_cycles)
    save(out / "first_input_faults.json", first_faults)
    hashed = [ROOT / "tools/adaptive_recorded_chain_replay.c", Path(__file__),
              *[ROOT / f"main/{name}.c" for name in CORE + EXTRA],
              source / "cycles.csv", source / "angle_only_input.csv", source / "recorded_velocity_input.csv"]
    save(out / "provenance.json", dict(hardware_actions=[], firmware_built=False,
         scope="sensor builder -> runtime -> assist -> controller -> composer; transport and system IO are host fakes",
         assumptions=["Linear interpolation of low-rate exports is not original CAN data",
                      "Each segment has a fresh simulated START; gap splitting is not hardware fault recovery",
                      "async: left source every 20 ms, right every 30 ms, control every 10 ms",
                      "quantized: exported 1 mrad angle step; not encoder resolution",
                      "First-two-cycle exclusion is only an evaluation label, not a firmware exemption",
                      "Counterexamples are retained; no claim of hardware stability"],
         sha256={str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in hashed}))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(3, 1, figsize=(14, 8), sharex=True)
    for side, color in ((0, "tab:blue"), (1, "tab:orange")):
        for scenario, style in (("fresh", "-"), ("async", "--")):
            leg = [r for r in traces["angle_only", scenario] if int(r["side"]) == side]
            # Draw each export segment separately; never bridge missing intervals.
            for index, segment in enumerate(sorted(set(r["segment"] for r in leg), key=int)):
                part = [r for r in leg if r["segment"] == segment]
                times = [int(r["time_us"]) / 1e6 for r in part]
                label = f"{'Left' if side == 0 else 'Right'} {scenario}" if index == 0 else None
                if scenario == "fresh":
                    axes[0].plot(times, [float(r["q"]) for r in part], color=color, lw=.7, label=label)
                axes[side + 1].plot(times, [float(r["torque"]) for r in part], style, color=color,
                                    lw=.8 if scenario == "fresh" else 1, alpha=.8, label=label)
    axes[0].set_ylabel("Angle (deg)")
    axes[1].set_ylabel("Left command (Nm)")
    axes[2].set_ylabel("Right command (Nm)")
    axes[2].set_xlabel("Replay time (s)")
    for ax in axes:
        ax.grid(alpha=.2)
        ax.legend(loc="upper right")
    fig.suptitle("Angle-input output-path simulation: synchronous vs asynchronous sources\nComputed commands; not measured joint torque or hardware validation")
    fig.tight_layout()
    fig.savefig(out / "angle_input_timing.png", dpi=160)
    fig.savefig(out / "angle_input_timing.pdf")
    plt.close(fig)
    print(json.dumps(summary, indent=2))
    print(f"Analysis complete: {out}. Counterexamples are in post_initial_zero_cycles.json and first_input_faults.json.")


if __name__ == "__main__":
    main()
