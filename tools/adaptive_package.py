"""冻结已构建的阶段1–4交付包，不编译、不烧录、不提交、不上传。

源码来自当前工作区，保留基础Git提交和逐文件SHA256；排除用户research目录。
manifest明确硬件未验，不能将构建产物当成部署/人体试验结果。
"""
import hashlib
import json
import re
import subprocess
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT / "docs/adaptive/evidence"
DEST = ROOT / "output/adaptive/20261103"


def sha(data):
    return hashlib.sha256(data).hexdigest()


def git(*args):
    return subprocess.check_output(["git", *args], cwd=ROOT)


def main():
    version = re.search(r'set\(PROJECT_VER "([^"]+)"', (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")).group(1)
    description = json.loads((ROOT / "build/project_description.json").read_text(encoding="utf-8"))
    if description["project_version"] != version or description["target"] != "esp32s3":
        raise RuntimeError("build version/target mismatch")
    files = sorted(set(git("ls-files", "-z", "--cached", "--others", "--exclude-standard").decode().split("\0")) - {""})
    files = [p for p in files if not p.startswith("research/") and p != "docs/adaptive/evidence/delivery-manifest.json"]
    changed = sorted(set(git("diff", "--name-only", "-z").decode().split("\0") +
                         git("ls-files", "--others", "--exclude-standard", "-z").decode().split("\0")) - {""})
    changed = [p for p in changed if p in files]
    payload = {"source/" + p: (ROOT / p).read_bytes() for p in files if (ROOT / p).is_file()}
    payload["source/sdkconfig"] = (ROOT / "sdkconfig").read_bytes()
    # 使用原构建的相对目录与地址，避免重新命名app导致烧错地址。
    for source, name in [
        ("bootloader/bootloader.bin", "bootloader/bootloader.bin"),
        ("partition_table/partition-table.bin", "partition_table/partition-table.bin"),
        ("twai_network_master.bin", "twai_network_master.bin"),
        ("twai_network_master.elf", "debug/twai_network_master.elf"),
        ("twai_network_master.map", "debug/twai_network_master.map"),
        ("flash_args", "flash_args"), ("flasher_args.json", "flasher_args.json"),
        ("project_description.json", "debug/project_description.json"),
        ("adaptive_host/asymmetric.csv", "replay/asymmetric.csv"),
        ("adaptive_host/real_scope_replay.csv", "replay/real_scope_replay.csv"),
    ]:
        payload[name] = (ROOT / "build" / source).read_bytes()
    stacks = []
    for path in sorted((ROOT / "build/esp-idf/main/CMakeFiles/__idf_main.dir").glob("*.su")):
        for line in path.read_text(encoding="utf-8").splitlines():
            if any(re.search(r":" + name + r"\s", line) for name in
                   ["control_task", "exo_adaptive_runtime_update", "exo_adaptive_engine_update", "exo_afo_update"]):
                stacks.append(line)
    manifest = {
        "version": version, "date": "2026-11-03", "base_commit": git("rev-parse", "HEAD").decode().strip(),
        "source_kind": "uncommitted working tree snapshot; research excluded",
        "target": "esp32s3", "idf": "5.5", "flash_size": "16MB",
        "can_tx_gpio": 4, "can_rx_gpio": 1, "can_bitrate": 1000000,
        "stage_1_4": "software implemented / host tested / target compiled",
        "adaptive_active": "blocked in assist and controller; SHADOW only",
        "hardware_actions_performed": [], "target_WCET": "NOT MEASURED",
        "stage_5_bench": "NOT RUN", "stage_6_human": "NOT RUN",
        "known_issue": "Historical CAN_ERROR_PASSIVE physical cause unresolved",
        "control_stack_allocated_bytes": 6144, "compiler_static_stack_frames": stacks,
        "changed_files": changed,
        "files": [{"path": name, "bytes": len(data), "sha256": sha(data)} for name, data in sorted(payload.items())],
    }
    encoded = (json.dumps(manifest, ensure_ascii=False, indent=2) + "\n").encode()
    (EVIDENCE / "delivery-manifest.json").write_bytes(encoded)
    payload["delivery-manifest.json"] = encoded
    sums = "".join(f"{sha(data)}  {name}\n" for name, data in sorted(payload.items()))
    payload["SHA256SUMS.txt"] = sums.encode()
    DEST.mkdir(parents=True, exist_ok=True)
    target = DEST / f"ESP-EXO-{version}-stage1-4.zip"
    with zipfile.ZipFile(target, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=6) as archive:
        for name, data in sorted(payload.items()):
            archive.writestr(name, data)
    with zipfile.ZipFile(target) as archive:
        assert archive.testzip() is None
        for name, data in payload.items():
            if sha(archive.read(name)) != sha(data):
                raise RuntimeError(f"archive mismatch: {name}")
    digest = sha(target.read_bytes())
    (DEST / "SHA256SUMS.txt").write_text(f"{digest}  {target.name}\n", encoding="utf-8")
    print(f"PASS package verified entries={len(payload)} changed_files={len(changed)} bytes={target.stat().st_size}")
    print(f"SHA256 {digest}\n{target}")


if __name__ == "__main__":
    main()
