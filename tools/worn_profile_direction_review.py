"""Offline projection on frozen worn-motion phases; no device/control access."""
import csv
import hashlib
import json
import math
import subprocess
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

ROOT = Path(__file__).resolve().parents[1]
NAME = "worn_zero_output_lock_repeat_20261104_2138"
SOURCE = ROOT / "output/adaptive/20261104" / (NAME + ".csv")
LOCK = SOURCE.with_suffix(".lock.json")


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    assert sha(SOURCE) == "fba0bf6672a4c624015ea6827e240435748a05876211a61e2e6e09e9e2fd160c"
    assert sha(LOCK) == "4053eb69f0ceb62fb27d62234ebb771b9e8b058cf92236f25bdfdb54936de21f"
    lock = json.loads(LOCK.read_text())
    assert not lock['invalid_rows'] and not lock['brownout_observed'] and not lock['panic_observed']
    boundary = int(lock['capture_boundary_us'])
    events = lock['events']
    assert all(e['dropped'] == 0 for e in events)
    rows, discarded, duplicates = {}, 0, 0
    with SOURCE.open(encoding='utf-8-sig', newline='') as stream:
        for row in csv.DictReader(stream):
            assert all(math.isfinite(float(v)) for v in row.values())
            t, side = int(row['timestamp_us']), int(row['side'])
            assert side in (0, 1)
            if t < boundary:
                discarded += 1
                continue
            key = (t, side)
            if key in rows:
                assert rows[key] == row, 'Conflicting duplicate'
                duplicates += 1
            else:
                rows[key] = row
    times = sorted({t for t, side in rows})
    assert all((t, side) in rows for t in times for side in (0, 1))
    assert all(int(r['system_state']) == 1 and int(r['command_valid']) == 0 and
               float(r['final_ff_command_nm']) == 0 for r in rows.values())
    now = datetime.now(ZoneInfo('Asia/Shanghai'))
    dest = ROOT / 'output/adaptive' / now.strftime('%Y%m%d') / ('worn_direction_' + now.strftime('%H%M%S_%f'))
    dest.mkdir(parents=True)
    source = dest / 'paired_input.csv'
    with source.open('x', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow('timestamp_us,ql,qr,vl,vr,valid_l,valid_r,phase_l,phase_r'.split(','))
        for t in times:
            l, r = rows[t, 0], rows[t, 1]
            writer.writerow([t, l['q_deg'], r['q_deg'], l['v_deg_s'], r['v_deg_s'],
                             l['feedback_valid'], r['feedback_valid'], l['corrected_phase'], r['corrected_phase']])
    core = ROOT / 'main/exo_adaptive_profile.c'
    assert sha(core) == 'e157203639ae5d9425f15e5ca80921a81aff0d55d1f8c29c02ccf6c681408fd3'
    exe = dest / 'profile_projection.exe'
    compiler = ROOT / 'build/native_toolchain/zig-windows-x86_64-0.13.0/zig.exe'
    harness = ROOT / 'tools/strategy_profile_compare.c'
    subprocess.run([str(compiler), 'cc', '-std=c11', '-O1', '-Wall', '-Wextra', '-Werror',
                    '-I' + str(ROOT / 'main/include'), str(harness), str(core), '-o', str(exe)], check=True)
    # Longest simultaneous sampled lock segment, split at every recorded exact loss.
    runs, current = [], []
    for t in times:
        if current and any(current[-1] < int(e['timestamp_us']) <= t for e in events):
            runs.append(current)
            current = []
        if all(int(rows[t, s]['locked']) and int(rows[t, s]['feedback_valid']) for s in (0, 1)):
            current.append(t)
        elif current:
            runs.append(current)
            current = []
    if current:
        runs.append(current)
    stable = max(runs, key=lambda x: x[-1] - x[0], default=[])
    report = {'scope': 'Historical worn-motion phase projection only; no feedback replay, fade, safety limiting, or CAN output',
              'comparison_amplitude_nm': 0.1, 'approved_human_amplitude': False,
              'phase_zero': 'ipsilateral max hip flexion', 'flex_peak': 0.75, 'ext_peak': 0.25, 'width': 0.1,
              'discarded_pre_boundary_rows': discarded, 'duplicates_removed': duplicates,
              'paired_rows': len(times), 'boundary_us': boundary,
              'exact_lock_losses': len(events),
              'longest_common_sampled_lock': {'start_us': stable[0], 'end_us': stable[-1],
                  'span_s': (stable[-1] - stable[0]) / 1e6, 'samples_per_side': len(stable)},
              'sides': [], 'provenance': {str(p.relative_to(ROOT)): sha(p) for p in (SOURCE, LOCK, core, harness)},
              'limitations': '10Hz sparse phases and position secants cannot establish true within-cycle direction or authorize human ACTIVE. Recorded loss events split lock spans; absent events do not restore missing inputs. No threshold/config change.'}
    for side, label in enumerate(('left', 'right')):
        output = dest / (label + '_projection.csv')
        subprocess.run([str(exe), str(source), str(output), str(side), '.75', '.25', '.1'], check=True)
        with output.open(newline='') as stream:
            projected = {int(r['timestamp_us']): r for r in csv.DictReader(stream)}
        assert set(projected) == set(times)
        details, eligible, high = [], [], []
        for i, t in enumerate(times):
            r, p = rows[t, side], projected[t]
            torque, speed = float(p['projected_desired_nm']), float(r['v_deg_s'])
            assert abs(torque) <= .100001
            if not (int(r['locked']) and int(r['feedback_valid']) and abs(speed) >= 3):
                continue
            eligible.append(t)
            if abs(torque) < .05:
                continue
            high.append(t)
            if torque * speed >= 0:
                continue
            secant = None
            if 0 < i < len(times) - 1:
                a, b = times[i - 1], times[i + 1]
                if int(rows[a, side]['feedback_valid']) and int(rows[b, side]['feedback_valid']):
                    secant = (float(rows[b, side]['q_deg']) - float(rows[a, side]['q_deg'])) * 1e6 / (b - a)
            future = [e for e in events if int(e['side']) == side and int(e['timestamp_us']) >= t]
            details.append({'timestamp_us': t, 'relative_s': (t - boundary) / 1e6,
                'phase': float(r['corrected_phase']), 'projected_nm': torque, 'reported_velocity_deg_s': speed,
                'centered_secant_deg_s': secant,
                'position_supports_opposition': None if secant is None or abs(secant) < 3 else torque * secant < 0,
                'in_longest_common_lock': t in stable,
                'next_recorded_loss_delta_s': (int(future[0]['timestamp_us']) - t) / 1e6 if future else None})
        report['sides'].append({'side': label, 'eligible_rows': len(eligible), 'high_projection_rows': len(high),
            'opposed_high_rows': len(details), 'position_supported': sum(r['position_supports_opposition'] is True for r in details),
            'position_disagrees': sum(r['position_supports_opposition'] is False for r in details),
            'position_indeterminate': sum(r['position_supports_opposition'] is None for r in details),
            'longest_common_lock_high_rows': sum(t in stable for t in high),
            'longest_common_lock_opposed_rows': sum(r['in_longest_common_lock'] for r in details),
            'details': details})
    report['projection_executable_sha256'] = sha(exe)
    (dest / 'summary.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(dest)
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
