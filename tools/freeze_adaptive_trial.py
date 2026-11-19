"""Freeze a new 0.11.6 bounded-trial package. No hardware/flash operations."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import zipfile

ROOT = Path(__file__).resolve().parents[1]


def sha(data):
    return hashlib.sha256(data).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build", type=Path, required=True)
    parser.add_argument("--evidence", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    build, evidence, target = args.build.resolve(), args.evidence.resolve(), args.output.resolve()
    assert build.is_relative_to(ROOT / "build") and evidence.is_relative_to(ROOT / "output")
    assert target.is_relative_to(ROOT / "output") and target.suffix == ".zip" and not target.exists()
    desc = json.loads((build / "project_description.json").read_text(encoding="utf-8"))
    assert desc["project_version"] == "0.11.6" and desc["target"] == "esp32s3"
    sdk = (build / "sdkconfig").read_text(encoding="utf-8")
    assert "CONFIG_EXO_ADAPTIVE_BOUNDED_TRIAL=y" in sdk
    for flag in ["EXO_ADAPTIVE_BENCH_ACTIVE", "EXO_ADAPTIVE_NOMINAL_15NM", "EXO_ADAPTIVE_ACTIVE_RELEASE",
                 "EXO_BENCH_PEAK_TEST", "EXO_ENGINEERING_GAIT_STIMULUS"]:
        assert f"CONFIG_{flag}=y" not in sdk, flag
    frozen = ROOT / "output/adaptive/20261103/ESP-EXO-0.11.0-stage1-4.zip"
    assert sha(frozen.read_bytes()) == "b183565619baf9d85f16afaccbcbe85c6f386fa745a3dcbf43553584703eba57"
    paths = subprocess.check_output(["git", "ls-files", "-z", "--cached", "--others", "--exclude-standard", "--",
                                     "main", "tools", "config", "web-bluetooth-console", "docs/adaptive"], cwd=ROOT).decode().split("\0")
    paths += ["CMakeLists.txt", "sdkconfig", "README.md"]
    payload = {"source/" + p: (ROOT / p).read_bytes() for p in sorted(set(paths)) if p and (ROOT / p).is_file()}
    for p in ["twai_network_master.bin", "twai_network_master.elf", "twai_network_master.map", "sdkconfig",
              "project_description.json", "flasher_args.json", "flash_args", "bootloader/bootloader.bin",
              "partition_table/partition-table.bin"]:
        payload["firmware/" + p] = (build / p).read_bytes()
    for p in sorted(evidence.rglob("*")):
        if p.is_file():
            payload["evidence/" + p.relative_to(evidence).as_posix()] = p.read_bytes()
    payload["evidence/git-status.txt"] = subprocess.check_output(["git", "status", "--short"], cwd=ROOT)
    manifest = {
        "version": "0.11.6", "target": "esp32s3", "strategy_id": 5, "strategy_version": 5,
        "deployment": "BOUNDED_MANUAL_TRIAL", "active_max_mnm": 5000,
        "profile": 1, "defaults": "disabled and zero; explicit profile prepare and manual START required",
        "power_w": {"positive": 2, "absorbed": 1}, "angle_deg": [-20, 20], "velocity_deg_s": 60,
        "slew_nm_s": 1, "fade_s": 3, "human_validation": "NOT RUN", "flashed": False,
        "bench_peak_entry": False, "engineering_trajectory": False, "nominal_15nm_release": False,
        "base_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT).decode().strip(),
        "source_kind": "dirty working tree snapshot; original research and frozen archives retained on disk",
        "files": {name: {"bytes": len(data), "sha256": sha(data)} for name, data in payload.items()},
    }
    payload["trial-manifest.json"] = json.dumps(manifest, ensure_ascii=False, indent=2).encode("utf-8")
    payload["SHA256SUMS.txt"] = "".join(f"{sha(data)}  {name}\n" for name, data in sorted(payload.items())).encode()
    target.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(target, "x", compression=zipfile.ZIP_DEFLATED) as archive:
        for name, data in payload.items():
            archive.writestr(name, data)
    with zipfile.ZipFile(target) as archive:
        assert archive.testzip() is None
        for name, info in manifest["files"].items():
            assert sha(archive.read(name)) == info["sha256"], name
    print(json.dumps({"package": str(target), "sha256": sha(target.read_bytes()), "files": len(payload),
                      "flashed": False, "active_max_mnm": 5000}, ensure_ascii=False))


if __name__ == "__main__":
    main()
