"""Chinese angle / old command / new command / every-cycle comparison figures."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties, fontManager
from matplotlib.patches import Patch, Rectangle

ap = argparse.ArgumentParser(description=__doc__)
ap.add_argument("validation", type=Path)
ap.add_argument("output", type=Path)
ap.add_argument("--window-comparison", action="store_true")
args = ap.parse_args()
args.output.mkdir(parents=True, exist_ok=True)
fontManager.addfont("C:/Windows/Fonts/msyh.ttc")
plt.rcParams.update({"font.family": FontProperties(fname="C:/Windows/Fonts/msyh.ttc").get_name(),
    "axes.unicode_minus": False, "font.size": 10, "axes.spines.top": False,
    "axes.spines.right": False, "savefig.facecolor": "white"})


def read(name):
    with (args.validation / name).open(encoding="utf-8-sig") as f:
        return list(csv.DictReader(f))


old = read("angle_only_baseline.csv")
new = read("angle_only_candidate.csv")
cycles = [r for r in read("cycles.csv") if r["mode"] == "angle_only"]
limits_path=args.validation / "limits.json"
limits=json.loads(limits_path.read_text()) if limits_path.exists() else {
    "minimum_angle_deg":-30,"maximum_angle_deg":60,"maximum_velocity_deg_s":300,
    "baseline_label":"0.11.20"}
colors = {"pass": "#07877c", "miss": "#ce4952", "acquisition": "#d6d9dd", "window": "#f4d5a2"}


def figure(walk, interval, filename):
    fig, axes = plt.subplots(4, 2, figsize=(15, 10), sharex="col",
                             gridspec_kw={"height_ratios": [1.25, 1, 1, .43]})
    fig.subplots_adjust(left=.065, right=.98, top=.88, bottom=.12, hspace=.24, wspace=.16)
    for side in (0, 1):
        rr = [r for r in new if int(r["trial"]) == walk - 1 and int(r["side"]) == side]
        oo = [r for r in old if int(r["trial"]) == walk - 1 and int(r["side"]) == side]
        t = np.array([(int(r["time_us"]) - 10000) / 1e6 for r in rr])
        q = np.array([float(r["q"]) for r in rr])
        a = np.array([float(r["torque"]) for r in oo])
        b = np.array([float(r["torque"]) for r in rr])
        lo, hi = interval if interval else (0, float(t[-1]))
        visible = (t >= lo) & (t <= hi)
        cc = [c for c in cycles if int(c["walk"]) == walk and int(c["side"]) == side]
        axes[0, side].plot(t[visible], q[visible], color="#344f6b", lw=1.1)
        axes[0, side].axhline(limits["maximum_angle_deg"], color="#bf761b", ls="--", lw=.8)
        axes[0, side].axhline(limits["minimum_angle_deg"], color="#bf761b", ls="--", lw=.8)
        axes[0, side].set_ylim(limits["minimum_angle_deg"]-5, limits["maximum_angle_deg"]+10)
        axes[0, side].set_title("左腿" if side == 0 else "右腿", fontsize=13)
        axes[1, side].plot(t[visible], a[visible], color="#c07832", lw=1.1)
        axes[2, side].plot(t[visible], b[visible], color=colors["pass"], lw=1.1)
        before="原窗口" if args.window_comparison else "旧策略"
        after="新窗口" if args.window_comparison else "新策略"
        for row, label in enumerate(["记录角度（°）", before+"命令（N·m）", after+"命令（N·m）"]):
            axes[row, side].set_ylabel(label)
            axes[row, side].grid(alpha=.18)
            if row:
                axes[row, side].set_ylim(-16, 16)
                axes[row, side].set_yticks([-15, 0, 15])
                axes[row, side].axhline(0, color="#555555", lw=.4)
        for c in cc:
            start, end = float(c["start_s"]), float(c["end_s"])
            if end < lo or start > hi:
                continue
            status = c["category"]
            if status != "eligible":
                for row in range(3):
                    axes[row, side].axvspan(start, end, color=colors[status], alpha=.23, lw=0)
            elif interval:
                for row in range(3):
                    axes[row, side].axvline(start, color="#aaaaaa", lw=.5, ls=":")
            for ypos, field in [(1, "baseline_both_directions"), (0, "both_directions")]:
                color = colors[status] if status != "eligible" else colors["pass" if c[field] == "True" else "miss"]
                axes[3, side].add_patch(Rectangle((start, ypos - .3), end-start, .6,
                    facecolor=color, edgecolor="white", linewidth=.35))
            if interval:
                mid=(start+end)/2
                if lo < mid < hi:
                    axes[0, side].text(mid, 65, c["cycle"], ha="center", fontsize=8, color="#555555")
        axes[3, side].set_ylim(-.55, 1.55)
        axes[3, side].set_yticks([0, 1], [after, before])
        axes[3, side].set_ylabel("逐周期")
        axes[3, side].set_xlabel("记录内时间（s）")
        axes[3, side].set_xlim(lo, hi)
        if interval:
            axes[3, side].set_xticks(np.arange(np.ceil(lo), hi + .01, 1))
    subtitle = f"第{walk}次记录" + (f"，局部 {interval[0]}–{interval[1]} s" if interval else "，完整时段")
    title="同一新策略：原窗口（−30°～60°，300°/s）与新窗口对比" if args.window_comparison else f"同一记录角度输入：基准（{limits['baseline_label']}）与新策略对比"
    fig.suptitle(f"{title}\n{subtitle}；新窗口 {limits['minimum_angle_deg']:g}°～{limits['maximum_angle_deg']:g}°，速度≤{limits['maximum_velocity_deg_s']:g}°/s", fontsize=15, y=.97)
    fig.legend(handles=[Patch(color=colors["pass"], label="该周期屈伸命令均有"),
                        Patch(color=colors["miss"], label="该周期缺少屈/伸命令"),
                        Patch(color=colors["acquisition"], label="捕获/重新捕获周期"),
                        Patch(color=colors["window"], label="含工作窗口越界的周期")],
               loc="lower center", bbox_to_anchor=(.5, .046), ncol=4, frameon=False, fontsize=9)
    note="周期资格统一按新窗口判断；原窗口的缺失可能是其自身越界保护。" if args.window_comparison else "阴影表示整周期分类，不代表该周期全程零输出。"
    fig.text(.5, .018,
        "角度10 Hz记录插值至100 Hz；速度由当前及前一角度计算。周期判定：屈、伸峰值各超过0.01 N·m。\n"
        +note+"曲线是仿真附加命令，不是实测关节力矩。",
        ha="center", fontsize=9, color="#555555")
    fig.savefig(args.output / (filename + ".png"), dpi=180)
    fig.savefig(args.output / (filename + ".pdf"))
    plt.close(fig)


figure(1, None, "walk1_full")
figure(2, None, "walk2_full")
figure(1, (20, 28), "walk1_zoom_20_28")
figure(2, (12, 20), "walk2_double_peak_12_20")
print(args.output.resolve())
