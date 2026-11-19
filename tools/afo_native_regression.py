"""Compile/run offline C regression groups; no device access."""
import argparse
from pathlib import Path
import subprocess

ap = argparse.ArgumentParser(description=__doc__)
ap.add_argument("output", type=Path)
args = ap.parse_args()
args.output.mkdir(parents=True, exist_ok=True)
cc = Path("build/native_toolchain/zig-windows-x86_64-0.13.0/zig.exe")
core = "main/exo_adaptive.c main/exo_afo.c main/exo_adaptive_event.c main/exo_adaptive_profile.c main/exo_adaptive_config.c main/exo_adaptive_safety.c".split()
extra = "main/exo_adaptive_runtime.c main/exo_assist.c main/exo_swing_pulse.c main/exo_phase_profile.c main/exo_controller.c main/exo_bench_peak.c main/exo_command_composer.c main/sine_reference.c main/exo_sensor_snapshot.c main/gait_estimator.c".split()
cases = [
    ("core", "adaptive_replay", [], False),
    ("fault", "adaptive_fault_replay", [], False),
    ("walking", "adaptive_walking_replay", [], False),
    ("observer", "adaptive_walking_observer_test", [], False),
    ("continuity", "adaptive_continuity_replay", [], False),
    ("integration", "adaptive_integration_replay", [], True),
    ("active500", "adaptive_active_replay", ["-DEXO_ADAPTIVE_HOST_ACTIVE_MAX_MNM=500"], True),
    ("active15000", "adaptive_active_replay", ["-DEXO_ADAPTIVE_HOST_ACTIVE_MAX_MNM=15000"], True),
    ("lock", "adaptive_lock_replay", ["-DEXO_ADAPTIVE_HOST_ACTIVE_MAX_MNM=500"], True),
    ("regular", "adaptive_mode_replay", ["-DCONFIG_EXO_ADAPTIVE_ASSIST_MODE=1"], True),
    ("trial", "adaptive_trial_replay", ["-DCONFIG_EXO_ADAPTIVE_BOUNDED_TRIAL=1", "-DEXO_ADAPTIVE_HOST_ACTIVE_MAX_MNM=5000"], True),
]
for name, source, flags, integrated in cases:
    exe = args.output / (name + ".exe")
    command = [str(cc), "cc", "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror", "-pedantic",
               "-Itools/host_fakes", "-Itools/host_fakes/factory", "-Itools/host_fakes/system",
               "-Imain/include", *flags, "tools/" + source + ".c", *core,
               *(extra if integrated else []), "-o", str(exe)]
    (args.output / (name + "_command.txt")).write_text(subprocess.list2cmdline(command))
    for stage, cmd in [("build", command), ("run", [str(exe.resolve())])]:
        result = subprocess.run(cmd, capture_output=True, text=True)
        (args.output / (name + "_" + stage + ".log")).write_text(result.stdout + result.stderr)
        if result.returncode:
            print(result.stdout + result.stderr)
            raise SystemExit(f"FAIL {name} {stage}: {result.returncode}")
    print("PASS", name, flush=True)
