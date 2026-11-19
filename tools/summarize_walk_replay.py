"""Compare labelled offline replay only; this is not measured motor output."""
import collections
import csv
import hashlib
import json
from pathlib import Path

def summarize(path):
    groups=collections.defaultdict(list)
    with path.open() as stream:
        for row in csv.DictReader(stream):
            groups[(int(row['trial']),int(row['side']))].append(row)
    result=[]
    for (trial,side),rows in sorted(groups.items()):
        hard=[r for r in rows if int(r['rearm'])]
        after=[r for r in rows if int(r['time_us'])>14610000 and abs(float(r['torque']))>.001]
        result.append(dict(trial=trial+1,side=side,samples=len(rows),
            nonzero=sum(abs(float(r['torque']))>.001 for r in rows),
            peak_command_nm=max(abs(float(r['torque'])) for r in rows),
            hard_latched_samples=len(hard),
            first_latch_us=int(hard[0]['time_us']) if hard else None,
            first_nonzero_after_14_61_s_us=int(after[0]['time_us']) if after else None))
    return result

if __name__=='__main__':
    import argparse
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('directory',type=Path)
    root=ap.parse_args().directory
    result={'limitation':'10 Hz position and velocity linearly interpolated independently to 100 Hz, each segment starts a fresh engine/permit. Not actual CAN trace or measured torque.',
        'v19':summarize(root/'replay_v19.csv'),'v20':summarize(root/'replay_v20.csv'), 'recorded_context':[]}
    for path in sorted((root/'five_walks').glob('*.jsonl')):
        with path.open(encoding='utf-8-sig') as stream:meta=json.loads(next(stream))
        states=[dict(hostTime=meta['startedAt'],value=meta['context'])]+[
            e for e in meta['events'] if e['type']=='device_state']
        info=meta['context']['info'];config=meta['context']['assistConfig']
        latched=[s for s in states if s['value'].get('assistConfig',{}).get('unavailableReason')==10]
        result['recorded_context'].append(dict(file=path.name,sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
            firmware=info['firmware'],boot_id=info['bootId'],strategy=config['strategyName'],
            levels=[config['leftLevelPercent'],config['rightLevelPercent']],
            selected_max_nm=meta['context']['assistTorqueLimit']['selectedMaximumNm'],
            first_latch_status_host_time=latched[0]['hostTime'] if latched else None))
    (root/'replay_comparison.json').write_text(json.dumps(result,indent=2),encoding='utf8')
    print(json.dumps(result,indent=2))
