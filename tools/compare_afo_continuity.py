"""Compare two simulate_afo_recording outputs using identical replay inputs."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager

def read(p):
    with p.open(encoding='utf-8-sig') as f:return list(csv.DictReader(f))

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('before',type=Path);ap.add_argument('after',type=Path);ap.add_argument('output',type=Path)
    args=ap.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    metrics=[]
    for mode in ['angle_only','recorded_velocity']:
        assert (args.before/f'{mode}_input.csv').read_bytes()==(args.after/f'{mode}_input.csv').read_bytes()
        for label,folder in [('0.11.21',args.before),('0.11.22',args.after)]:
            cycles=read(folder/'cycles.csv')
            for side in [0,1]:
                group=[r for r in cycles if r['mode']==mode and int(r['side'])==side]
                post=[r for r in group if int(r['cycle'])>2]
                metrics.append(dict(version=label,mode=mode,side=side,all_cycles=len(group),
                    post_initial_cycles=len(post),post_initial_zero=sum(r['any_assist']=='False' for r in post),
                    **{f'post_initial_both_gt_{threshold:g}nm':sum(float(r['positive_peak_nm'])>threshold and float(r['negative_peak_nm'])<-threshold for r in post) for threshold in [.01,.1,1,3]}))
    (args.output/'metrics.json').write_text(json.dumps(metrics,indent=2),encoding='utf-8')
    font=Path('C:/Windows/Fonts/msyh.ttc')
    if font.is_file():font_manager.fontManager.addfont(str(font));plt.rcParams['font.family']='Microsoft YaHei'
    plt.rcParams['axes.unicode_minus']=False
    before=read(args.before/'recorded_velocity_trace.csv');after=read(args.after/'recorded_velocity_trace.csv')
    recorded=read(args.after/'recorded_samples.csv');t0=float(recorded[0]['timestampMs'])
    summary=json.loads((args.after/'summary.json').read_text(encoding='utf-8'))
    for name,span in [('full',None),('zoom_50_60',(50,60))]:
        fig,axes=plt.subplots(4,2,figsize=(16,11),sharex='col',layout='constrained')
        for side,prefix in enumerate(['left','right']):
            rt=[(float(r['timestampMs'])-t0)/1000 for r in recorded]
            axes[0,side].plot(rt,[float(r[prefix+'PositionDeg']) for r in recorded],color='#254965',lw=.8)
            axes[1,side].plot(rt,[float(r[prefix+'AppliedTorqueNm']) if r[prefix+'AppliedTorqueNm'] else np.nan for r in recorded],color='#4d4d4d',lw=.8)
            for j,rows,color in [(2,before,'#be6a39'),(3,after,'#00877c')]:
                for segment in sorted({r['segment'] for r in rows}):
                    selected=[r for r in rows if r['segment']==segment and int(r['side'])==side]
                    axes[j,side].plot([(int(r['time_us'])-10000)/1e6 for r in selected],[float(r['torque']) for r in selected],color=color,lw=.9)
            axes[0,side].set_title('左腿' if side==0 else '右腿')
            for j,label in enumerate(['记录角度 (°)','0.11.21设备命令 (N·m)','0.11.21仿真 (N·m)','0.11.22仿真 (N·m)']):
                ax=axes[j,side];ax.set_ylabel(label);ax.grid(alpha=.2)
                if j>0:ax.set_ylim(-16,16)
                if span:ax.set_xlim(span)
                for gap in summary['gaps']:ax.axvspan(gap['start_s'],gap['end_s'],color='#aaaaaa',alpha=.25)
            axes[3,side].set_xlabel('记录起点后的时间 (s)')
        fig.suptitle('同一角度 + 记录速度输入：修复前后助力命令\n灰带为遥测缺口；缺口处分段初始化；0.11.22尚未实机验证',fontsize=12)
        for ext in ['png','pdf']:fig.savefig(args.output/f'continuity_{name}.{ext}',dpi=170)
        plt.close(fig)
    print(json.dumps(metrics,indent=2))
if __name__=='__main__':main()
