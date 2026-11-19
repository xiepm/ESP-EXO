"""Freeze a new, unique ACTIVE bench candidate; never flash or replace an archive."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import re
import subprocess
import zipfile

ROOT = Path(__file__).resolve().parents[1]


def sha(data):
    return hashlib.sha256(data).hexdigest()


def git(*args):
    return subprocess.check_output(["git", *args], cwd=ROOT)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--evidence-directory", type=Path)
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    target = args.output.resolve()
    if not target.is_relative_to((ROOT / "output").resolve()) or target.suffix != ".zip":
        raise SystemExit("Output must be a new zip inside this repository's output directory")
    if target.exists():
        raise SystemExit("Refusing to replace an existing package")
    build = ROOT / "build/adaptive_bench"
    description = json.loads((build / "project_description.json").read_text(encoding="utf-8"))
    version = re.search(r'set\(PROJECT_VER "([^"]+)"', (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")).group(1)
    if version not in ("0.11.1", "0.11.2", "0.11.3", "0.11.4", "0.11.5") or description["project_version"] != version or description["target"] != "esp32s3":
        raise SystemExit("Candidate version/target mismatch")
    config = (build / "sdkconfig").read_text(encoding="utf-8")
    if "CONFIG_EXO_ADAPTIVE_BENCH_ACTIVE=y" not in config:
        raise SystemExit("Expected bench sdkconfig")
    nominal = "CONFIG_EXO_ADAPTIVE_NOMINAL_15NM=y" in config
    released = "CONFIG_EXO_ADAPTIVE_ACTIVE_RELEASE=y" in config
    peak_test = "CONFIG_EXO_BENCH_PEAK_TEST=y" in config
    if released:
        raise SystemExit("Release packaging requires separate verified hardware evidence; candidate packager refuses release variants")
    if nominal and (version not in ("0.11.3", "0.11.4", "0.11.5") or not peak_test):
        raise SystemExit("Nominal candidate requires 0.11.3/0.11.4 and bounded peak-test entry")
    if "# CONFIG_EXO_ADAPTIVE_BENCH_ACTIVE is not set" not in (ROOT / "sdkconfig").read_text(encoding="utf-8"):
        raise SystemExit("Ordinary build must remain SHADOW")
    frozen = ROOT / "output/adaptive/20261103/ESP-EXO-0.11.0-stage1-4.zip"
    frozen_sha = sha(frozen.read_bytes())
    if frozen_sha != "b183565619baf9d85f16afaccbcbe85c6f386fa745a3dcbf43553584703eba57":
        raise SystemExit("Original frozen archive changed")
    sources = set(git("ls-files", "-z", "--cached", "--others", "--exclude-standard", "--",
                      "main", "tools", "config", "web-bluetooth-console").decode("utf-8").split("\0")) - {""}
    sources.update(["CMakeLists.txt", "sdkconfig", "README.md", "CONSOLE_COMMANDS.md",
                    "docs/adaptive/ACTIVE_CANDIDATE_20261104.md", "docs/adaptive/NOMINAL_TORQUE_SCOPE_20261104.md",
                    "docs/adaptive/RECOVERY_20261104.md", "docs/adaptive/POWER_CUT_20261104.md"])
    if args.report:
        sources.add(args.report.resolve().relative_to(ROOT).as_posix())
    payload = {"source/" + name: (ROOT / name).read_bytes() for name in sorted(sources) if (ROOT / name).is_file()}
    for name in ["twai_network_master.bin", "bootloader/bootloader.bin", "partition_table/partition-table.bin",
                 "flasher_args.json", "flash_args", "sdkconfig", "project_description.json",
                 "twai_network_master.elf", "twai_network_master.map"]:
        payload["firmware/" + name] = (build / name).read_bytes()
    evidence = args.evidence_directory.resolve() if args.evidence_directory else ROOT / "output/adaptive/20261104"
    if not evidence.is_relative_to(ROOT / "output"):
        raise SystemExit("Evidence must be inside this repository's output directory")
    for log in sorted(evidence.glob("*" if args.evidence_directory else "active_candidate_*.log")):
        if log.is_file() and log.suffix in (".log", ".json"):
            payload["evidence/" + log.name] = log.read_bytes()
    payload["evidence/working-tree-status.txt"] = git("status", "--short")
    payload["evidence/tracked-worktree.patch"] = git("diff", "--binary", "HEAD", "--", ".", ":(exclude)research")
    manifest = {
        "created_at": datetime.now().astimezone().isoformat(), "version": version, "target": "esp32s3",
        "base_commit": git("rev-parse", "HEAD").decode().strip(),
        "source_kind": "uncommitted working tree snapshot; source excludes research; status may list its name",
        "variant": "off-body fixed-bench candidate", "strategy_id": 5, "strategy_version": 3 if nominal else 2,
        "deployed_active_max_mnm": 15000 if nominal else 500, "host_only_active_max_mnm": 15000,
        "bounded_peak_test_entry": peak_test,
        "bounded_peak_profile": 2 if version in ("0.11.4", "0.11.5") else 1 if peak_test else 0,
        "bounded_peak_trace_schema": 1 if version == "0.11.5" else 0,
        "exact_cycle_lock_loss_diagnostics": version in ("0.11.2", "0.11.3", "0.11.4", "0.11.5"),
        "hardware_actions_this_candidate": [], "flashed": False, "nonzero_bench_test": "NOT RUN",
        "operator_report_previous_nonzero_and_abnormal_exit": "PASSED per operator; not independently rerun for this diagnostic candidate",
        "nominal_15Nm_verification_basis": "command plus motor-reported estimated torque; no independent precision torque calibration required for this scope",
        "human_active_15Nm": "NOT RELEASED", "target_WCET": "NOT MEASURED",
        "original_0_11_0_archive_sha256": frozen_sha,
        "files": [{"path": name, "bytes": len(data), "sha256": sha(data)} for name, data in sorted(payload.items())],
    }
    payload["candidate-manifest.json"] = (json.dumps(manifest, ensure_ascii=False, indent=2) + "\n").encode("utf-8")
    payload["SHA256SUMS.txt"] = "".join(f"{sha(data)}  {name}\n" for name, data in sorted(payload.items())).encode("utf-8")
    target.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(target, "x", compression=zipfile.ZIP_DEFLATED, compresslevel=6) as archive:
        for name, data in sorted(payload.items()):
            archive.writestr(name, data)
    with zipfile.ZipFile(target) as archive:
        if archive.testzip() is not None:
            raise RuntimeError("Zip integrity error")
        for name, data in payload.items():
            if sha(archive.read(name)) != sha(data):
                raise RuntimeError(f"Archive mismatch: {name}")
    digest = sha(target.read_bytes())
    with target.with_suffix(".sha256").open("x", encoding="utf-8") as output:
        output.write(f"{digest}  {target.name}\n")
    print(f"PASS candidate package verified: {len(payload)} entries, {target.stat().st_size} bytes")
    print(f"SHA256 {digest}\n{target}")


if __name__ == "__main__":
    main()
