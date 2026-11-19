"""Compare offline curves against existing logged phases. No device access."""
import csv
import hashlib
import json
import math
import subprocess
from datetime import datetime
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
AUDIT=ROOT/'output/adaptive/20261104/strategy_audit_20261104_210145'
PROFILES={'early':(.65,.15),'center':(.75,.25),'late':(.85,.35)}
SOURCES=[('left','diagnostic_idle_left_continuous_20261104_172953',0),
         ('right','right_controlled_20261104_151349',1)]


def main():
    dest=ROOT/'output/adaptive/20261104'/('profile_drafts_'+datetime.now().strftime('%Y%m%d_%H%M%S'))
    dest.mkdir()
    exe=ROOT/'build/adaptive_host_native/strategy_profile_compare.exe'
    report={'scope':'curve projection on existing historical phase; not full engine replay or actual output',
        'amplitude_nm_for_comparison':.1,'width_cycles':.1,
        'deployment_allowed':False,'device_parameters_changed':False,'metrics':[]}
    draft={'type':'OFFLINE_CURVE_ONLY_NOT_DEVICE_CONFIG','phase_zero':'ipsilateral maximum hip flexion; not heel strike',
        'sign':'positive flexion, negative extension in joint coordinates',
        'overall_strength_for_projection':1,'lambda_for_projection':1,
        'profiles':{name:{'flex_peak_phase':f,'ext_peak_phase':e,'flex_amplitude_nm':.1,
                         'ext_amplitude_nm':.1,'flex_width':.1,'ext_width':.1} for name,(f,e) in PROFILES.items()},
        'unconfigured':['physical angle/velocity/power limits','validated amplitude','fade/rearm/ACTIVE execution'],
        'note':'0.1Nm is a comparison scale, not an approved hardware starting torque. No UART commands or deployable config blob generated.'}
    (dest/'offline_profile_drafts.json').write_text(json.dumps(draft,indent=2),encoding='utf-8')
    for side,name,index in SOURCES:
        original=ROOT/'output/adaptive/20261104'/(name+'.csv')
        logged=list(csv.DictReader(original.open(encoding='utf-8-sig')))
        boundary=int(logged[0]['timestamp_us'])
        lock={int(r['timestamp_us']):int(r['locked']) for r in logged
              if int(r['side'])==index and int(r['timestamp_us'])>=boundary}
        for profile,(flex,ext) in PROFILES.items():
            output=dest/(side+'_'+profile+'.csv')
            subprocess.run([str(exe),str(AUDIT/(name+'_input.csv')),str(output),str(index),str(flex),str(ext),'.1'],check=True)
            rows=list(csv.DictReader(output.open()))
            eligible=[r for r in rows if lock[int(r['timestamp_us'])] and int(r['valid']) and abs(float(r['velocity_deg_s']))>=3]
            assert eligible
            torque=[float(r['projected_desired_nm']) for r in eligible]
            positive=[float(r['projected_desired_nm'])*float(r['velocity_deg_s'])>0 for r in eligible]
            assert max(abs(float(r['projected_desired_nm'])) for r in rows)<=.100001
            report['metrics'].append({'side':side,'profile':profile,'source_sha256':hashlib.sha256(original.read_bytes()).hexdigest(),
                'total_rows':len(rows),'eligible_historical_locked_moving_rows':len(eligible),
                'same_direction_fraction':sum(positive)/len(positive),
                'torque_weighted_same_direction_fraction':sum(abs(t) for t,ok in zip(torque,positive) if ok)/sum(map(abs,torque)),
                'peak_abs_projected_nm':max(map(abs,torque)),
                'historical_unlocked_rows':sum(not lock[int(r['timestamp_us'])] for r in rows)})
    # Targeted mathematical reader check, not a repeat of hardware/algorithm suites.
    src=dest/'curve_check_input.csv'
    src.write_text('timestamp_us,ql,qr,vl,vr,valid_l,valid_r,phase_l,phase_r\n10000,0,0,1,1,1,1,.25,.25\n20000,0,0,1,1,1,1,.75,.75\n',encoding='utf-8')
    dst=dest/'curve_check_output.csv'
    subprocess.run([str(exe),str(src),str(dst),'0','.75','.25','.1'],check=True)
    values=[float(r['projected_desired_nm']) for r in csv.DictReader(dst.open())]
    assert values[0]<0<values[1] and abs(values[0]+values[1])<1e-7
    report['targeted_check']='quarter-cycle extension negative; three-quarter flexion positive; symmetric equal amplitudes'
    report['limitations']='Direction agreement is descriptive on the same hand-motion data, not independent efficacy, gait validation or selection of a clinical optimum. No inference of actual ACTIVE output or lock continuity between decimated samples.'
    (dest/'summary.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(dest)
    for r in report['metrics']:print(r['side'],r['profile'],r['eligible_historical_locked_moving_rows'],round(100*r['same_direction_fraction'],2),round(100*r['torque_weighted_same_direction_fraction'],2))


if __name__=='__main__':main()
