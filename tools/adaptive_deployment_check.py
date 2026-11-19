"""Compile-only deployment boundary checks. Does not open any device port."""
from pathlib import Path
import os
import subprocess


def main():
    root = Path(__file__).resolve().parents[1]
    zig = root / "build/native_toolchain/zig-windows-x86_64-0.13.0/zig.exe"
    compiler = [str(zig), "cc"] if os.name == "nt" and zig.exists() else ["gcc"]
    cases = [
        ("ordinary_shadow", [], 0, 500, None),
        ("esp_bench_500", ["ESP_PLATFORM", "CONFIG_EXO_ADAPTIVE_BENCH_ACTIVE=1"], 1, 500, None),
        ("host_15000", ["EXO_ADAPTIVE_HOST_ACTIVE_MAX_MNM=15000"], 1, 15000, None),
        ("esp_nominal_15000", ["ESP_PLATFORM", "CONFIG_EXO_ADAPTIVE_BENCH_ACTIVE=1",
                               "CONFIG_EXO_ADAPTIVE_NOMINAL_15NM=1"], 1, 15000, None),
        ("esp_release_15000", ["ESP_PLATFORM", "CONFIG_EXO_ADAPTIVE_BENCH_ACTIVE=1",
                               "CONFIG_EXO_ADAPTIVE_NOMINAL_15NM=1",
                               "CONFIG_EXO_ADAPTIVE_ACTIVE_RELEASE=1"], 1, 15000, None),
        ("esp_rejects_release_without_nominal", ["ESP_PLATFORM", "CONFIG_EXO_ADAPTIVE_BENCH_ACTIVE=1",
                               "CONFIG_EXO_ADAPTIVE_ACTIVE_RELEASE=1"], None, None,
                               "Manual ACTIVE release requires the explicit nominal 15 Nm build"),
        ("esp_nominal_flag_alone_stays_shadow", ["ESP_PLATFORM", "CONFIG_EXO_ADAPTIVE_NOMINAL_15NM=1"],
                               0, 500, None),
        ("esp_rejects_release_with_peak_entry", ["ESP_PLATFORM", "CONFIG_EXO_ADAPTIVE_BENCH_ACTIVE=1",
                               "CONFIG_EXO_ADAPTIVE_NOMINAL_15NM=1", "CONFIG_EXO_ADAPTIVE_ACTIVE_RELEASE=1",
                               "CONFIG_EXO_BENCH_PEAK_TEST=1"], None, None,
                               "Manual ACTIVE release must not expose the bench peak test entry"),
        ("esp_rejects_host_override", ["ESP_PLATFORM", "EXO_ADAPTIVE_HOST_ACTIVE_MAX_MNM=15000"],
         None, None, "Host ACTIVE validation is not a deployable ESP configuration"),
        ("host_rejects_zero", ["EXO_ADAPTIVE_HOST_ACTIVE_MAX_MNM=0"],
         None, None, "Host ACTIVE limit must be in 1..15000 mNm"),
        ("host_rejects_above_15", ["EXO_ADAPTIVE_HOST_ACTIVE_MAX_MNM=15001"],
         None, None, "Host ACTIVE limit must be in 1..15000 mNm"),
    ]
    for name, defines, allowed, cap, diagnostic in cases:
        source = '#include "exo_adaptive_deployment.h"\n'
        if diagnostic is None:
            source += f'_Static_assert(EXO_ADAPTIVE_ACTIVE_ALLOWED == {allowed}, "active");\n'
            source += f'_Static_assert(EXO_ADAPTIVE_CONFIG_MAX_MNM == {cap}, "cap");\n'
            released = int("CONFIG_EXO_ADAPTIVE_ACTIVE_RELEASE=1" in defines)
            source += f'_Static_assert(EXO_ADAPTIVE_RELEASE_ALLOWED == {released}, "release");\n'
        check_dir = root / "build/deployment_boundary_checks"
        check_dir.mkdir(parents=True, exist_ok=True)
        check_file = check_dir / (name + ".c")
        check_file.write_text(source, encoding="utf-8")
        result = subprocess.run(
            compiler + ["-std=c11", "-Werror", "-c", str(check_file),
                        "-o", str(check_dir / (name + ".o")),
                        "-Imain/include", "-Itools/host_fakes"] + ["-D" + d for d in defines],
            cwd=root, text=True, capture_output=True, encoding="utf-8", errors="replace",
        )
        passed = (result.returncode == 0) if diagnostic is None else (
            result.returncode != 0 and diagnostic in result.stderr)
        if not passed:
            raise SystemExit(f"FAIL {name}: {result.returncode}\n{result.stdout}\n{result.stderr}")
        print(f"PASS {name}" + (f" (expected rejection: {diagnostic})" if diagnostic else ""))
    print(f"PASS deployment boundaries: {len(cases)}/{len(cases)}; compile only; no device access")


if __name__ == "__main__":
    main()
