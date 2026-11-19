"""Analyze paired motion-free telemetry; never infer a release time from a peak."""
import argparse
import csv
import json
import math
import statistics
from pathlib import Path


def analyze(path):
    data=json.loads(path.read_text(encoding='utf-8'))
    if data['error'] or data.get('decode_errors'):
        raise ValueError('Capture or decode failed')
    if data['info']['boot_id'] != data['final_info']['boot_id']:
        raise ValueError('Boot changed')
    if data['running']['dropped_sample_count'] or data['stopped']['streaming']:
        raise ValueError('Dropped samples or stream not stopped')
    groups={}
    for f in data['frames']:
        if f['flags'] != 11 or f['stream_sequence'] != data['running']['stream_sequence']:
            raise ValueError('Invalid feedback/state/output/stream flags')
        g=groups.setdefault(f['sample_sequence'],{})
        if f['channel'] in g:raise ValueError('Duplicate channel')
        g[f['channel']]=f
    seq=sorted(groups)
    if any(b != a+1 for a,b in zip(seq,seq[1:])):raise ValueError('Missing sample')
    if set(groups[seq[-1]])!={1,2,4}:seq=seq[:-1]
    rows=[]
    for n in seq:
        g=groups[n]
        if set(g)!={1,2,4} or len({f['timestamp_ms'] for f in g.values()})!=1:
            raise ValueError('Incomplete/mismatched sample')
        row={'sample':n,'timestamp_s':g[1]['timestamp_ms']/1000}
        for label,raw in (('left','left_raw'),('right','right_raw')):
            if any(g[c][raw] is None for c in (1,2,4)):raise ValueError('Invalid channel')
            row[label+'_q_deg']=math.degrees(g[1][raw]/1000)
            row[label+'_v_deg_s']=math.degrees(g[2][raw]/100)
            row[label+'_estimate_nm']=g[4][raw]/1000
        rows.append(row)
    start=rows[0]['timestamp_s'];end=rows[-1]['timestamp_s']
    gaps=[b['timestamp_s']-a['timestamp_s'] for a,b in zip(rows,rows[1:])]
    if not gaps or min(gaps)<=0 or max(gaps)>.15:raise ValueError('Invalid timing')
    summary={'source':str(path),'boot':hex(data['info']['boot_id']),'samples':len(rows),
        'duration_s':end-start,'max_gap_s':max(gaps),'sides':{},
        'release_time':'not operator-labeled; do not infer a free-decay period from manually driven motion'}
    for side in ('left','right'):
        key=side+'_q_deg';q=[r[key] for r in rows]
        baseline=statistics.median(r[key] for r in rows if r['timestamp_s']<=start+2)
        tail=[r[key] for r in rows if r['timestamp_s']>=end-5]
        summary['sides'][side]={'baseline_first_2s_deg':baseline,'min_deg':min(q),'max_deg':max(q),
            'range_deg':max(q)-min(q),'tail_median_deg':statistics.median(tail),
            'tail_range_deg':max(tail)-min(tail),
            'max_abs_speed_report_deg_s':max(abs(r[side+'_v_deg_s']) for r in rows),
            'max_abs_motor_estimate_nm':max(abs(r[side+'_estimate_nm']) for r in rows),
            'one_second_angle_ranges':[]}
        for second in range(math.ceil(end-start)):
            sub=[r[key] for r in rows if second<=r['timestamp_s']-start<second+1]
            if sub:summary['sides'][side]['one_second_angle_ranges'].append([second,min(sub),max(sub)])
    with path.with_name('paired.csv').open('w',newline='',encoding='utf-8') as f:
        w=csv.DictWriter(f,list(rows[0]));w.writeheader();w.writerows(rows)
    path.with_name('analysis.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
    return summary


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('capture',type=Path);args=p.parse_args()
    print(json.dumps(analyze(args.capture),indent=2))
