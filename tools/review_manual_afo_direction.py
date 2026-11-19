"""Offline direction/timing review of an archived manual AFO capture. No device I/O."""
import argparse
import csv
import hashlib
import json
import math
import subprocess
from collections import Counter
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
REASONS = {1: 'INPUT', 2: 'TIME', 4: 'STOPPED', 8: 'INTERVALS',
           16: 'EVENT_AGE', 32: 'FREQUENCY', 64: 'PHASE_ERROR',
           128: 'RESIDUAL', 256: 'EMERGENCY'}
# Analysis coverage guard only: never changes firmware's 30 ms input limit.
MAX_NEIGHBOR_GAP_US = 150_000


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_csv(path):
    with path.open(newline='', encoding='utf-8-sig') as stream:
        return [{k: float(v) for k, v in row.items()} for row in csv.DictReader(stream)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('review_directory', type=Path)
    args = parser.parse_args()
    source = args.review_directory.resolve()
    intake = json.loads((source / 'summary.json').read_text(encoding='utf-8'))
    capture = intake['capture']
    raw, = source.glob('*.log')
    manifest, = source.glob('*.markers.json')
    assert sha(raw) == intake['provenance']['log']['sha256']
    assert sha(manifest) == intake['provenance']['markers']['sha256']
    rows = read_csv(source / 'deduplicated_adapt.csv')
    assert rows and len(rows) == capture['unique_rows']
    assert all(math.isfinite(v) for r in rows for v in r.values())
    assert all(r['feedback_valid'] == 1 and int(r['faults']) & 1 == 0 and
               r['system_state'] == 1 and r['command_valid'] == 0 and
               r['final_ff_command_nm'] == 0 for r in rows)
    by_key = {(int(r['timestamp_us']), int(r['side'])): r for r in rows}
    times = sorted({t for t, s in by_key})
    assert len(by_key) == len(rows) == 2 * len(times)
    assert all((t, s) in by_key for t in times for s in (0, 1))
    boundary = capture['boundary_us']
    assert min(times) >= boundary
    events = []
    for line in raw.read_text(encoding='utf-8').splitlines():
        if not line.startswith('ADLOCK,'):
            continue
        values = list(map(float, line[7:].split(',')))
        assert len(values) == 20 and all(math.isfinite(v) for v in values)
        if values[0] < boundary:
            continue
        names = ('timestamp_us sequence side system_state permit_epoch generation reasons residual '
                 'phase_error_rad frequency_error event_age_s intervals q_deg v_deg_s locked rearm '
                 'faults final_ff_nm command_valid dropped').split()
        e = dict(zip(names, values))
        assert e['side'] in (0, 1) and e['locked'] == 0 and e['dropped'] == 0
        e['reason_names'] = [name for bit, name in REASONS.items() if int(e['reasons']) & bit]
        events.append(e)
    events.sort(key=lambda e: e['timestamp_us'])
    assert len(events) == capture['exact_lock_losses']
    assert len({(e['timestamp_us'], e['side']) for e in events}) == len(events)

    core, harness = ROOT / 'main/exo_adaptive_profile.c', ROOT / 'tools/strategy_profile_compare.c'
    exe = ROOT / 'output/adaptive/20261105/worn_direction_094343_799012/profile_projection.exe'
    expected = {core: 'e157203639ae5d9425f15e5ca80921a81aff0d55d1f8c29c02ccf6c681408fd3',
                harness: '8fbedf057f0d676d4490aead63cff4b7b9f8a34271f48202c7bee37b3ff808f8',
                exe: '94db5e598c2e2af88451609c88a7599cc6f0bb30664874763462dfac6a582798'}
    assert all(sha(p) == digest for p, digest in expected.items()), 'Reused projection provenance changed'
    dest = source / ('direction_' + datetime.now().strftime('%H%M%S_%f'))
    dest.mkdir()
    paired = dest / 'paired_input.csv'
    with paired.open('x', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow('timestamp_us,ql,qr,vl,vr,valid_l,valid_r,phase_l,phase_r'.split(','))
        for t in times:
            left, right = by_key[t, 0], by_key[t, 1]
            writer.writerow([t, left['q_deg'], right['q_deg'], left['v_deg_s'], right['v_deg_s'],
                             1, 1, left['corrected_phase'], right['corrected_phase']])

    def window(t):
        for w in intake['activity_windows']:
            lo, hi = w['approximate_mcu_bounds_us']
            if lo <= t < hi:
                return w['index']
        return 0

    def lock_runs(sides):
        runs, current = [], []
        for t in times:
            if current and (t - current[-1] > MAX_NEIGHBOR_GAP_US or any(
                    e['side'] in sides and current[-1] < e['timestamp_us'] <= t for e in events)):
                runs.append(current)
                current = []
            if all(by_key[t, s]['locked'] == 1 for s in sides):
                current.append(t)
            elif current:
                runs.append(current)
                current = []
        if current:
            runs.append(current)
        return runs

    common_runs = lock_runs((0, 1))
    common = max(common_runs, key=lambda run: run[-1] - run[0], default=[])
    report = {
        'scope': 'Recorded phase projected through production curve only; no oscillator replay, fade, limiter, or CAN output',
        'boot_as_recorded': capture['boots'], 'comparison_amplitude_nm': 0.1,
        'approved_human_amplitude': False, 'flex_peak': 0.75, 'ext_peak': 0.25, 'width': 0.1,
        'analysis_max_neighbor_gap_ms': MAX_NEIGHBOR_GAP_US / 1000,
        'longest_common_sampled_lock': None if not common else {
            'start_us': common[0], 'end_us': common[-1],
            'span_s': (common[-1] - common[0]) / 1e6, 'samples_per_side': len(common)},
        'loss_reason_counts': dict(Counter(n for e in events for n in e['reason_names'])),
        'loss_events': events, 'sides': [],
        'provenance': {str(p): sha(p) for p in (raw, manifest, source / 'summary.json',
            source / 'deduplicated_adapt.csv', core, harness, exe, Path(__file__))},
        'limitations': [
            'Manual marker MCU bounds are last received snapshots, not synchronized motion times.',
            'About 10 Hz snapshots do not reconstruct the 100 Hz loop or motion inside a bracket.',
            'Secants are corroboration, not independent sensor measurements. Turning brackets are reported separately.',
            'Lock runs are sampled spans split at recorded losses and gaps, not proof of unobserved continuity.',
            'Zero-output recording and offline projection do not authorize human ACTIVE or any torque amplitude.',
            'No device connection, firmware/configuration/threshold change or original evidence overwrite.']}
    for side, label in enumerate(('left', 'right')):
        projection_path = dest / (label + '_projection.csv')
        subprocess.run([str(exe), str(paired), str(projection_path), str(side), '.75', '.25', '.1'], check=True)
        projected = {int(r['timestamp_us']): r for r in read_csv(projection_path)}
        assert set(projected) == set(times)
        side_events = [e for e in events if e['side'] == side]
        runs = lock_runs((side,))
        run_starts = {t: run[0] for run in runs for t in run}
        details, eligible, high, peak_events = [], [], [], []
        last_count = int(by_key[times[0], side]['accepted_events'])
        for i, t in enumerate(times):
            r, p = by_key[t, side], projected[t]
            count = int(r['accepted_events'])
            assert count >= last_count
            if count > last_count:
                peak_events.append({'first_observed_us': t, 'peak_timestamp_us': int(r['event_timestamp_us']),
                    'accepted_events': count, 'count_increment': count - last_count,
                    'first_observed_phase': r['corrected_phase'], 'locked': int(r['locked'])})
            last_count = count
            torque, velocity = p['projected_desired_nm'], r['v_deg_s']
            assert abs(torque) <= .100001
            if not (r['locked'] == 1 and abs(velocity) >= 3):
                continue
            eligible.append(t)
            if abs(torque) < .05:
                continue
            high.append(t)
            if torque * velocity >= 0:
                continue
            secant = left_slope = right_slope = None
            if 0 < i < len(times) - 1:
                a, b = times[i - 1], times[i + 1]
                if max(t - a, b - t) <= MAX_NEIGHBOR_GAP_US:
                    q0, q1 = by_key[a, side]['q_deg'], by_key[b, side]['q_deg']
                    secant = (q1 - q0) * 1e6 / (b - a)
                    left_slope = (r['q_deg'] - q0) * 1e6 / (t - a)
                    right_slope = (q1 - r['q_deg']) * 1e6 / (b - t)
            support = None if secant is None or abs(secant) < 3 else torque * secant < 0
            bracket = (None if secant is None else
                       left_slope * right_slope < 0)
            previous_loss = [e for e in side_events if e['timestamp_us'] <= t]
            next_loss = [e for e in side_events if e['timestamp_us'] >= t]
            details.append({'timestamp_us': t, 'relative_s': (t - boundary) / 1e6,
                'activity_window': window(t), 'q_deg': r['q_deg'], 'phase': r['corrected_phase'],
                'raw_phase': r['raw_phase'], 'phase_offset_rad': r['phase_offset_rad'],
                'projected_nm': torque, 'reported_velocity_deg_s': velocity,
                'centered_secant_deg_s': secant, 'left_slope_deg_s': left_slope,
                'right_slope_deg_s': right_slope, 'brackets_sampled_turn': bracket,
                'position_supports_opposition': support,
                'both_adjacent_slopes_support_opposition': secant is not None and
                    abs(left_slope) >= 3 and abs(right_slope) >= 3 and
                    torque * left_slope < 0 and torque * right_slope < 0,
                'in_longest_common_lock': t in common, 'since_observed_lock_start_s': (t - run_starts[t]) / 1e6,
                'since_last_accepted_peak_s': (t - r['event_timestamp_us']) / 1e6,
                'frequency_hz': r['hz'], 'event_period_s': r['period_s'],
                'previous_loss_delta_s': (t - previous_loss[-1]['timestamp_us']) / 1e6 if previous_loss else None,
                'next_loss_delta_s': (next_loss[0]['timestamp_us'] - t) / 1e6 if next_loss else None,
                'next_loss_reasons': next_loss[0]['reason_names'] if next_loss else []})
        windows = []
        for w in [0, *(w['index'] for w in intake['activity_windows'])]:
            subset = [d for d in details if d['activity_window'] == w]
            window_periods = [(b['peak_timestamp_us'] - a['peak_timestamp_us']) / 1e6
                for a, b in zip(peak_events, peak_events[1:])
                if b['count_increment'] == 1 and w != 0 and
                window(a['peak_timestamp_us']) == w == window(b['peak_timestamp_us'])]
            windows.append({'index': w, 'high_projection_rows': sum(window(t) == w for t in high),
                'opposed_rows': len(subset),
                'centered_secant_supported': sum(d['position_supports_opposition'] is True for d in subset),
                'both_adjacent_slopes_supported': sum(d['both_adjacent_slopes_support_opposition'] for d in subset),
                'exact_losses': sum(window(e['timestamp_us']) == w for e in side_events),
                'consecutive_accepted_peak_spacing_s_range': [min(window_periods), max(window_periods)]
                    if window_periods else None})
        periods = [(b['peak_timestamp_us'] - a['peak_timestamp_us']) / 1e6
                   for a, b in zip(peak_events, peak_events[1:]) if b['count_increment'] == 1]
        side_report = {'side': label, 'eligible_rows': len(eligible), 'high_projection_rows': len(high),
            'opposed_high_rows': len(details),
            'position_supported': sum(d['position_supports_opposition'] is True for d in details),
            'position_disagrees': sum(d['position_supports_opposition'] is False for d in details),
            'position_indeterminate': sum(d['position_supports_opposition'] is None for d in details),
            'both_adjacent_slopes_supported': sum(d['both_adjacent_slopes_support_opposition'] for d in details),
            'longest_common_lock_high_rows': sum(t in common for t in high),
            'longest_common_lock_opposed_rows': sum(d['in_longest_common_lock'] for d in details),
            'accepted_peak_spacing_s_range_including_pauses': [min(periods), max(periods)] if periods else None,
            'activity_windows': windows, 'peak_events': peak_events, 'details': details}
        report['sides'].append(side_report)
        if details:
            with (dest / (label + '_opposed.csv')).open('x', newline='', encoding='utf-8') as stream:
                writer = csv.DictWriter(stream, fieldnames=details[0])
                writer.writeheader()
                writer.writerows(details)
    (dest / 'direction_review.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({'out': str(dest), **{k: v for k, v in report.items()
        if k not in ('provenance', 'loss_events', 'sides', 'limitations')},
        'sides': [{k: v for k, v in s.items() if k not in ('peak_events', 'details')} for s in report['sides']]},
        ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
