"""Preserve/describe exported BLE samples and generate labelled offline replay input.

10 Hz exports do not contain the original 100 Hz CAN trace. Replay uses linear
interpolation of BOTH recorded position and velocity, only within each segment;
never interpret interpolated transitions as the measured first fault cause.
"""
import argparse, collections, csv, hashlib, json
from pathlib import Path

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('output',type=Path);ap.add_argument('files',type=Path,nargs='+')
    args=ap.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    summary=[]
    with (args.output/'replay_input.csv').open('w',newline='') as stream:
        writer=csv.writer(stream);writer.writerow(['trial','time_us','left_q','left_v','right_q','right_v'])
        for trial,path in enumerate(sorted(args.files)):
            data=path.read_bytes();(args.output/path.name).write_bytes(data)
            rows=list(csv.DictReader(data.decode('utf-8-sig').splitlines()))
            start=float(rows[0]['timestampMs'])
            record={'file':path.name,'sha256':hashlib.sha256(data).hexdigest(),'samples':len(rows),
                'duration_s':(float(rows[-1]['timestampMs'])-start)/1000,
                'states':dict(collections.Counter(r['systemState'] for r in rows)),
                'activation_epochs':sorted(set(r['activationEpochLow16'] for r in rows)),
                'output_allowed':dict(collections.Counter(r['outputAllowed'] for r in rows)),
                'real_output_flag':dict(collections.Counter(r['realOutputEnabled'] for r in rows)),
                'fields':{},'operating_window_crossings':{}}
            for key in ['leftPositionDeg','rightPositionDeg','leftVelocityDegS','rightVelocityDegS',
                'leftAssistSuggestionNm','rightAssistSuggestionNm','leftAppliedTorqueNm','rightAppliedTorqueNm']:
                valid=[(i,float(r[key])) for i,r in enumerate(rows) if r[key]!='']
                nz=[i for i,v in valid if abs(v)>.001]
                record['fields'][key]={'min':min(v for _,v in valid),'max':max(v for _,v in valid),
                    'valid':len(valid),'nonzero':len(nz),'first_nonzero_s':(float(rows[nz[0]]['timestampMs'])-start)/1000 if nz else None,
                    'last_nonzero_s':(float(rows[nz[-1]]['timestampMs'])-start)/1000 if nz else None}
            for side in ['left','right']:
                record['operating_window_crossings'][side]=[
                    {'time_s':(float(r['timestampMs'])-start)/1000,'q':float(r[side+'PositionDeg']),
                     'v':float(r[side+'VelocityDegS'])} for r in rows
                    if r[side+'PositionDeg'] and r[side+'VelocityDegS'] and
                    (not -30<=float(r[side+'PositionDeg'])<=60 or abs(float(r[side+'VelocityDegS']))>300)]
            fields=['leftPositionDeg','leftVelocityDegS','rightPositionDeg','rightVelocityDegS']
            usable=[r for r in rows if all(r[k]!='' for k in fields)]
            first=float(usable[0]['timestampMs']);last=float(usable[-1]['timestampMs']);j=0
            for ms in range(0,int(last-first)+1,10):
                t=first+ms
                while j+1<len(usable) and float(usable[j+1]['timestampMs'])<t:j+=1
                a,b=usable[j],usable[min(j+1,len(usable)-1)]
                ta,tb=float(a['timestampMs']),float(b['timestampMs'])
                if tb-ta>150:raise ValueError('Cannot interpolate a missing-data gap')
                w=(t-ta)/(tb-ta) if tb>ta else 0
                writer.writerow([trial,10000+ms*1000]+[float(a[k])*(1-w)+float(b[k])*w for k in fields])
            summary.append(record)
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False),encoding='utf8')
    print(json.dumps(summary,indent=2,ensure_ascii=False))
if __name__=='__main__':main()
