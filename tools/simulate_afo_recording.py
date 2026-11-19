"""Read an ESP-EXO CSV/JSONL export and replay the CURRENT native C walking AFO.

No device access. The output is a conditional algorithm simulation, not a
reconstruction of missing CAN frames, initial RAM, or measured joint torque.
Run with --help; without an input, the newest exo-walk export in Downloads is used.
"""
from __future__ import annotations

import argparse
import collections
import csv
from datetime import datetime
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys

import numpy as np
from scipy.signal import find_peaks

ROOT = Path(__file__).resolve().parents[1]
CORE = [f"main/{name}.c" for name in (
    "exo_adaptive", "exo_afo", "exo_adaptive_event", "exo_adaptive_profile",
    "exo_adaptive_config", "exo_adaptive_safety")]
FIELDS = ["leftPositionDeg", "rightPositionDeg", "leftVelocityDegS", "rightVelocityDegS"]
CONTEXT_FIELDS = ["firmware", "strategy", "leftLevelPercent", "rightLevelPercent", "maximumNm"]


def save_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def write_csv(path, rows, fields=None):
    with path.open("w", encoding="utf-8-sig", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields or list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def number(value):
    if value in (None, ""):
        return float("nan")
    return float(value)


def truth(value):
    return str(value).lower() in ("true", "1")


def read_recording(path):
    """JSONL context uses the same asynchronous host-time join as the web CSV."""
    if path.suffix.lower() == ".csv":
        with path.open(encoding="utf-8-sig", newline="") as f:
            return list(csv.DictReader(f)), None
    if path.suffix.lower() != ".jsonl":
        raise ValueError("Expected .csv or .jsonl (the spreadsheet icon may hide .csv)")
    objects = [json.loads(line) for line in path.read_text(encoding="utf-8-sig").splitlines() if line.strip()]
    sessions = [x for x in objects if x.get("type") == "session"]
    if len(sessions) != 1:
        raise ValueError("JSONL must contain exactly one session header")
    meta = sessions[0]
    events = sorted((e for e in meta.get("events", []) if e.get("type") == "device_state"), key=lambda e: e["hostTime"])
    context, at, index = meta.get("context", {}), meta["startedAt"], 0
    rows = []
    for obj in objects:
        if obj.get("type") != "sample":
            continue
        row = dict(obj)
        while index < len(events) and events[index]["hostTime"] <= row["receivedIsoTime"]:
            context, at = events[index].get("value") or {}, events[index]["hostTime"]
            index += 1
        a = context.get("assistConfig") or {}
        row.update(contextReadHostTime=at,
            firmware=".".join(map(str, (context.get("info") or {}).get("firmware", []))),
            strategy=a.get("strategyName"), leftLevelPercent=a.get("leftLevelPercent"),
            rightLevelPercent=a.get("rightLevelPercent"),
            maximumNm=(context.get("assistTorqueLimit") or {}).get("selectedMaximumNm"),
            assistUnavailableReason=a.get("unavailableReasonName"))
        for side, status in zip(("left", "right"), context.get("afoStatuses") or [None, None]):
            status = status or {}
            row[side + "AfoFaults"] = status.get("faults")
            row[side + "AfoFirstFaults"] = status.get("firstFaults") if status.get("firstPresent") else None
        rows.append(row)
    return rows, meta


def prepare(rows, max_gap_ms):
    if len(rows) < 2:
        raise ValueError("At least two samples required")
    t = np.array([number(r.get("timestampMs")) for r in rows])
    if not np.isfinite(t).all() or np.any(np.diff(t) <= 0):
        raise ValueError("Non-increasing/invalid device timestamps: split sessions before replay")
    # This runner handles a fixed AFO ACTIVE configuration, not arbitrary mode changes.
    for key in CONTEXT_FIELDS + ["activationEpochLow16", "webSession", "streamSequence"]:
        values = {str(r.get(key)) for r in rows}
        if len(values) != 1 or next(iter(values)) in ("", "None"):
            raise ValueError(f"Missing/changing {key}: split into constant-configuration recordings")
    if rows[0]["strategy"] != "ADAPTIVE_GAUSSIAN":
        raise ValueError("Recording is not AFO")
    if any(int(r["systemState"]) != 3 or not truth(r["outputAllowed"]) for r in rows):
        raise ValueError("This replay requires an ACTIVE, output-authorized segment")
    peak = number(rows[0]["maximumNm"])
    levels = [number(rows[0][s + "LevelPercent"]) for s in ("left", "right")]
    if not 0 <= peak <= 15 or not all(0 <= x <= 100 for x in levels):
        raise ValueError("Invalid recorded torque cap/levels")
    data = np.array([[number(r.get(k)) for k in FIELDS] for r in rows])
    valid = np.isfinite(data).all(axis=1)
    # Missing angle/velocity rows and long telemetry gaps break the independent runs.
    segments, current = [], []
    for i in range(len(rows)):
        if not valid[i] or (current and t[i] - t[current[-1]] > max_gap_ms):
            if len(current) >= 2:
                segments.append(current)
            current = []
        if valid[i]:
            current.append(i)
    if len(current) >= 2:
        segments.append(current)
    if not segments:
        raise ValueError("No contiguous valid segment with at least two samples")
    gaps = [{"start_s": float((t[i]-t[0])/1000), "end_s": float((t[i+1]-t[0])/1000),
             "duration_ms": float(t[i+1]-t[i])} for i in np.flatnonzero(np.diff(t) > 150)]
    return t, data, segments, peak, levels, gaps


def resample(t, data, segments, mode):
    result = []
    for segment, indices in enumerate(segments):
        ts = t[indices]
        # 100 Hz synthetic control/source samples. Interpolation uses future exported
        # endpoints; only the C observer's processing of this synthetic stream is causal.
        grid = ts[0] + np.arange(int((ts[-1]-ts[0]) // 10) + 1) * 10
        q = [np.interp(grid, ts, data[indices, side]) for side in (0, 1)]
        v = [np.r_[0., np.diff(x)/.01] for x in q] if mode == "angle_only" else [
            np.interp(grid, ts, data[indices, side + 2]) for side in (0, 1)]
        for i, ms in enumerate(grid):
            result.append(dict(segment=segment, time_us=int(round(10000+(ms-t[0])*1000)),
                left_q=q[0][i], left_v=v[0][i], right_q=q[1][i], right_v=v[1][i]))
    return result


def compiler_command(compiler):
    if compiler:
        path = Path(compiler).resolve() if Path(compiler).is_file() else shutil.which(compiler)
    else:
        bundled = ROOT / "build/native_toolchain/zig-windows-x86_64-0.13.0/zig.exe"
        path = bundled if bundled.is_file() else next((shutil.which(c) for c in ("zig", "gcc", "clang") if shutil.which(c)), None)
    if not path:
        raise ValueError("No host C compiler: provide --compiler PATH_TO_ZIG_OR_GCC_OR_CLANG")
    return [str(path)] + (["cc"] if Path(path).stem.lower() == "zig" else [])


def build_native(out, compiler):
    source_names = CORE + ["tools/afo_recording_replay.c", "tools/simulate_afo_recording.py"]
    source_names += [p.relative_to(ROOT).as_posix() for p in sorted((ROOT/"main/include").glob("*.h"))]
    source_names += ["CMakeLists.txt", "main/exo_assist.c", "main/exo_ble.c"]
    hashes = {p: hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in source_names}
    exe = out / ("afo_replay.exe" if sys.platform == "win32" else "afo_replay")
    cmd = compiler_command(compiler) + ["-std=c11", "-O2", "-Wall", "-Wextra", "-Werror", "-pedantic",
        "-Imain/include", "tools/afo_recording_replay.c", *CORE, "-o", str(exe)]
    if sys.platform != "win32":
        cmd += ["-lm"]
    save_json(out/"build_command.json", cmd)
    process = subprocess.run(cmd, cwd=ROOT, capture_output=True)
    (out/"build.log").write_bytes(process.stdout + process.stderr)
    if process.returncode:
        raise RuntimeError(f"Host C build failed: see {out/'build.log'}")
    if any(hashlib.sha256((ROOT/p).read_bytes()).hexdigest() != h for p,h in hashes.items()):
        raise RuntimeError("Source changed during compilation")
    save_json(out/"source_sha256.json", hashes)
    return exe, hashes


def analyze(trace, rows, t, mode, limits, threshold):
    groups = collections.defaultdict(list)
    for row in trace:
        groups[int(row["segment"]), int(row["side"])].append(row)
    cycles, summaries = [], []
    for (segment, side), group in groups.items():
        x = lambda key: np.array([float(r[key]) for r in group])
        q, v, torque = x("q"), x("v"), x("torque")
        seconds = (x("time_us")-10000)/1e6
        outside = (q < limits["minimum_angle_deg"]) | (q > limits["maximum_angle_deg"]) | (abs(v) > limits["maximum_velocity_deg_s"])
        if not np.isfinite(torque).all() or np.any(abs(torque) > limits["requested_peak_nm"]+.001):
            raise RuntimeError("Nonfinite/over-limit simulation output")
        if np.any(abs(torque[outside]) > .00001) or np.any(torque*v < -.001):
            raise RuntimeError("Window/sign guard violation")
        peaks, _ = find_peaks(q, prominence=10, distance=40)
        key = ("left" if side == 0 else "right") + "AppliedTorqueNm"
        recorded = np.array([number(r.get(key)) for r in rows])
        actual_t = (t-t[0])/1000
        for index, (a,b) in enumerate(zip(peaks, peaks[1:])):
            tau = torque[a:b]
            observed = recorded[(actual_t >= seconds[a]) & (actual_t < seconds[b])]
            observed = observed[np.isfinite(observed)]
            fault_counts = collections.Counter(int(r["faults"]) for r in group[a:b])
            cycles.append(dict(mode=mode, segment=segment, side=side, cycle=index+1,
                start_s=seconds[a], end_s=seconds[b], positive_peak_nm=float(max(tau)),
                negative_peak_nm=float(min(tau)), both_directions=bool(max(tau)>threshold and min(tau)<-threshold),
                any_assist=bool(np.any(abs(tau)>threshold)), outside_samples=int(outside[a:b].sum()),
                rearm_samples=int(x("rearm")[a:b].sum()), locked_fraction=float(x("locked")[a:b].mean()),
                fault_codes=json.dumps(dict(fault_counts), sort_keys=True),
                recorded_samples=len(observed),
                recorded_positive_peak_nm=float(max(observed)) if len(observed) else None,
                recorded_negative_peak_nm=float(min(observed)) if len(observed) else None,
                recorded_both_directions=bool(max(observed)>threshold and min(observed)<-threshold) if len(observed) else ""))
        summaries.append(dict(mode=mode, segment=segment, side=side, samples=len(group),
            start_s=float(seconds[0]), end_s=float(seconds[-1]),
            peak_nm=float(max(abs(torque))), nonzero_fraction=float(np.mean(abs(torque)>threshold)),
            hard_latched_samples=int(x("rearm").sum()), outside_samples=int(outside.sum()),
            fault_counts=dict(collections.Counter(r["faults"] for r in group))))
    return cycles, summaries


def plot(out, rows, t, traces, gaps, limits, zoom):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    if Path("C:/Windows/Fonts/msyh.ttc").is_file():
        font_manager.fontManager.addfont("C:/Windows/Fonts/msyh.ttc")
        plt.rcParams["font.family"] = "Microsoft YaHei"
    plt.rcParams["axes.unicode_minus"] = False
    colors = {"angle_only":"#047c81", "recorded_velocity":"#bd5a18"}
    labels = {"angle_only":"仿真：角度差分速度", "recorded_velocity":"仿真：记录速度"}
    for name, span in [("comparison_full", None), ("comparison_zoom", zoom)]:
        fig, axes = plt.subplots(3, 2, figsize=(16, 10), sharex="col", layout="constrained")
        for side, prefix in enumerate(("left", "right")):
            raw_t = (t-t[0])/1000
            axes[0,side].plot(raw_t, [number(r.get(prefix+"PositionDeg")) for r in rows], lw=.9, color="#24445b")
            for bound in (limits["minimum_angle_deg"], limits["maximum_angle_deg"]):
                axes[0,side].axhline(bound, ls="--", lw=.7, color="#aa7777")
            axes[0,side].set_title("左腿" if side==0 else "右腿")
            axes[1,side].plot(raw_t, [number(r.get(prefix+"AppliedTorqueNm")) for r in rows], lw=.8, color="#454545", label="设备记录：已发送附加助力命令")
            for mode, trace in traces.items():
                ids = sorted({int(r["segment"]) for r in trace})
                for j, segment in enumerate(ids):
                    subset = [r for r in trace if int(r["side"])==side and int(r["segment"])==segment]
                    st = [(int(r["time_us"])-10000)/1e6 for r in subset]
                    axes[1,side].plot(st, [float(r["torque"]) for r in subset], color=colors[mode], lw=.8, alpha=.85, label=labels[mode] if j==0 else None)
                    axes[2,side].plot(st, [float(r["locked"]) for r in subset], color=colors[mode], lw=.8, label=labels[mode] if j==0 else None)
            for i, ylabel in enumerate(("角度 (°)", "助力命令 (N·m)", "仿真就绪 / locked")):
                ax=axes[i,side]; ax.set_ylabel(ylabel); ax.grid(alpha=.2)
                for gap in gaps: ax.axvspan(gap["start_s"],gap["end_s"],color="#b2b7bf",alpha=.3)
                if span: ax.set_xlim(span)
            axes[1,side].legend(fontsize=8,loc="upper right")
            axes[2,side].set_xlabel("记录起点后的时间 (s)")
        fig.suptitle("当前固件AFO源码回放：角度—助力命令对比\n灰带：导出记录缺口；分段重置是仿真假设；曲线不是实测人体关节力矩",fontsize=12)
        for ext in ("png", "pdf"):
            fig.savefig(out/f"{name}.{ext}",dpi=170)
        plt.close(fig)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("input", nargs="?", type=Path, help="CSV or JSONL; default: newest Downloads/exo-walk export")
    ap.add_argument("--output", type=Path, help="New output directory (never overwrites an existing run)")
    ap.add_argument("--velocity", choices=["both","angle_only","recorded_velocity"], default="both")
    ap.add_argument("--max-gap-ms", type=float, default=150, help="Larger gaps split independent simulations; default 150 ms")
    ap.add_argument("--threshold-nm", type=float, default=.01, help="Cycle presence threshold, not an efficacy criterion")
    ap.add_argument("--zoom", type=float, nargs=2, default=[20,30], metavar=("START_S","END_S"))
    ap.add_argument("--compiler", help="Host zig/gcc/clang executable")
    args=ap.parse_args(argv)
    if not np.isfinite(args.max_gap_ms) or args.max_gap_ms < 100 or not np.isfinite(args.threshold_nm) or args.threshold_nm <= 0:
        ap.error("max-gap-ms must be finite and >=100; threshold-nm must be finite and >0")
    if not all(np.isfinite(args.zoom)) or args.zoom[0] < 0 or args.zoom[0] >= args.zoom[1]:
        ap.error("zoom must satisfy 0 <= START_S < END_S")
    if args.input is None:
        files = list((Path.home()/"Downloads").glob("exo-walk-*.jsonl")) + list((Path.home()/"Downloads").glob("exo-walk-*.csv"))
        if not files: ap.error("No recording found; specify its path")
        args.input=max(files,key=lambda p:p.stat().st_mtime_ns)
    source=args.input.resolve(); rows,meta=read_recording(source)
    t,data,segments,peak,levels,gaps=prepare(rows,args.max_gap_ms)
    out=(args.output or ROOT/"output/afo_simulation"/datetime.now().strftime("%Y%m%d_%H%M%S_%f")).resolve()
    out.mkdir(parents=True,exist_ok=False)
    shutil.copy2(source,out/source.name)
    print(f"Input: {source}\nOutput: {out}\nSamples={len(rows)}, segments={len(segments)}, telemetry gaps={len(gaps)}",flush=True)
    exe,hashes=build_native(out,args.compiler)
    normalized=[]
    context_keys=["contextReadHostTime",*CONTEXT_FIELDS,"assistUnavailableReason",
        "leftAfoFaults","rightAfoFaults","leftAfoFirstFaults","rightAfoFirstFaults"]
    for row in rows:
        normalized.append({k:row.get(k) for k in ["timestampMs","receivedIsoTime",*FIELDS,
            "leftAppliedTorqueNm","rightAppliedTorqueNm",*context_keys]})
    write_csv(out/"recorded_samples.csv",normalized)
    modes=["angle_only","recorded_velocity"] if args.velocity=="both" else [args.velocity]
    traces,cycles,summaries={},[],[]
    for mode in modes:
        replay=resample(t,data,segments,mode)
        # C fgets/sscanf expects ASCII header/data without a BOM.
        with (out/f"{mode}_input.csv").open("w",encoding="ascii",newline="") as f:
            w=csv.DictWriter(f,fieldnames=list(replay[0])); w.writeheader(); w.writerows(replay)
        trace_path=out/f"{mode}_trace.csv"
        subprocess.run([str(exe),str(out/f"{mode}_input.csv"),str(trace_path),str(out/"limits.json"),str(peak),*map(str,levels)],check=True)
        with trace_path.open(encoding="utf-8") as f: trace=list(csv.DictReader(f))
        limits=json.loads((out/"limits.json").read_text(encoding="utf-8"))
        c,s=analyze(trace,rows,t,mode,limits,args.threshold_nm)
        traces[mode]=trace; cycles+=c; summaries+=s
    write_csv(out/"cycles.csv",cycles,fields=list(cycles[0]) if cycles else ["mode","segment","side","cycle","both_directions"])
    aggregate=[]
    for mode in modes:
        for side in (0,1):
            group=[c for c in cycles if c["mode"]==mode and c["side"]==side]
            aggregate.append(dict(mode=mode,side=side,cycles=len(group),both_directions=sum(c["both_directions"] for c in group),
                any_assist=sum(c["any_assist"] for c in group),no_assist=sum(not c["any_assist"] for c in group)))
    actual=[]
    for side in (0,1):
        group=[c for c in cycles if c['mode']==modes[0] and c['side']==side]
        observed=[c for c in group if c['recorded_samples']>0]
        for threshold in [.01,1,3]:
            any_count=sum(max(abs(c['recorded_positive_peak_nm']),abs(c['recorded_negative_peak_nm']))>threshold for c in observed)
            both_count=sum(c['recorded_positive_peak_nm']>threshold and c['recorded_negative_peak_nm']<-threshold for c in observed)
            actual.append(dict(side=side,threshold_nm=threshold,total_cycles=len(group),recorded_any=any_count,
                recorded_both=both_count,recorded_none=len(observed)-any_count,unknown=len(group)-len(observed)))
    save_json(out/'actual_recorded_cycle_statistics.json',actual)
    version=re.search(r'set\(PROJECT_VER "([^"]+)"', (ROOT/"CMakeLists.txt").read_text(encoding="utf-8")).group(1)
    summary=dict(input=str(source),input_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        firmware_source_version=version,recorded_firmware=rows[0]["firmware"],samples=len(rows),duration_s=float((t[-1]-t[0])/1000),
        recorded_peak_nm=peak,levels=levels,limits=limits,segments=len(segments),gaps=gaps,max_gap_ms=args.max_gap_ms,
        missing_kinematic_rows=int(np.sum(~np.isfinite(data).all(axis=1))),
        input_rows_used=sum(map(len,segments)),
        isolated_valid_rows_omitted=int(np.isfinite(data).all(axis=1).sum())-sum(map(len,segments)),
        dropped_flags=sum(truth(r.get("droppedSincePrevious")) for r in rows),threshold_nm=args.threshold_nm,
        segmentation="Offline positive peaks: prominence>=10 deg, distance>=0.4 s; not heel-strike ground truth",
        assumptions=["100 Hz linear interpolation of low-rate exports, not original CAN samples",
            "Each segment starts with empty observer memory and simulated permit=1; no hardware START is sent",
            "Gap splitting does not mean real hardware reset or recovered from a hard fault",
            "Native walking engine plus level scaling; excludes RTOS timing, CAN delivery and final system gates",
            "Recorded applied torque is added command, not measured joint torque; missing torque stays missing",
            "Context/fault readbacks are asynchronous and are not exact triggering frames"],
        cycle_summary=aggregate,actual_recorded_cycle_summary=actual,segment_summary=summaries,
        fault_bits={"1":"invalid input", "2":"time", "4":"operating window", "8":"phase/acquisition", "16":"stop", "32":"hard rearm required", "64":"configuration"},
        hardware_actions=[])
    candidate=ROOT/"output/can/20261118/v21_candidate/manifest.json"
    if candidate.is_file():
        frozen=json.loads(candidate.read_text(encoding="utf-8"))
        normalized_hashes={k.replace("\\","/"):v for k,v in frozen["sources"].items()}
        compared=CORE+["main/include/exo_adaptive.h"]
        summary["v21_candidate_algorithm_sources_match"]=all(hashes[p]==normalized_hashes.get(p) for p in compared)
    save_json(out/"summary.json",summary)
    plot(out,rows,t,traces,gaps,limits,args.zoom)
    report=["# AFO记录回放结果", "",f"输入：`{source}`", "",
        f"当前源码版本：{version}；记录版本：{rows[0]['firmware']}；样本：{len(rows)}；时长：{summary['duration_s']:.3f}秒。",
        f"灰带标出{len(gaps)}处大于150 ms的遥测缺口。按{args.max_gap_ms:g} ms阈值分成{len(segments)}个独立仿真片段，每段从空状态重新捕获；孤立有效样本省略{summary['isolated_valid_rows_omitted']}个。", "",
        "| 输入 | 侧别 | 离线分段周期 | 双向助力 | 完全无助力 |", "|---|---|---:|---:|---:|"]
    for item in aggregate:
        report.append(f"| {item['mode']} | {'左' if item['side']==0 else '右'} | {item['cycles']} | {item['both_directions']} | {item['no_assist']} |")
    report += ["", "上述表格全部是仿真，不是设备实测输出周期数。原始设备命令统计另见actual_recorded_cycle_statistics.json；不插值助力，缺失命令不补零。", "",
        f"双向助力仅指单周期同时出现大于+{args.threshold_nm:g}和小于−{args.threshold_nm:g} N·m的计算命令，不代表有效助力强度或实机验证通过。所有已分割周期均计入，没有删除未通过周期。", "",
        "`comparison_full.png/pdf`为全程图，`comparison_zoom.png/pdf`为局部图；`cycles.csv`为逐周期结果，`*_trace.csv`为100 Hz仿真状态，`recorded_samples.csv`为原始采样及异步上下文。", "",
        "- 直接编译本仓库当前C算法；限值由C代码输出到limits.json。",
        "- 低频角度线性插值使用相邻导出点，不是对原始CAN数据的重建。角度差分只使用当前及前一仿真点。",
        "- 分段初始化、记录开始前的未知RAM状态、原始反馈时间戳和调度均会使仿真与实机不同。",
        "- 图中实机曲线是设备报告的附加助力命令，不是人体关节力矩；缺失命令不补零。",
        "- fault位：1输入、2时间、4窗口、8相位/捕获、16停步、32硬锁止需重新授权、64配置。位可组合；异步原始故障不能定位毫秒级触发帧。",
        "- 未访问串口、BLE或电机，也未修改固件源码。", ""]
    (out/"REPORT.md").write_text("\n".join(report),encoding="utf-8")
    print('RECORDED DEVICE COMMANDS (not simulation; no torque interpolation):')
    print(json.dumps(actual,indent=2))
    print('SIMULATION ONLY (not actual device results):')
    print(json.dumps(aggregate,indent=2))
    print("PASS: host replay/plots completed; no device access. See summary.json and cycles.csv.")
    return out


if __name__ == "__main__":
    try:
        main()
    except (ValueError, RuntimeError, OSError, subprocess.CalledProcessError) as error:
        raise SystemExit(str(error)) from error
