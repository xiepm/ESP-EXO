"""Single-window off-body bench preflight; never retries or sends motion commands."""
import asyncio
import json
import math
from datetime import datetime
from stationary_window_candidate import evaluate


def require(ok, message):
    if not ok:
        raise RuntimeError(message)


def assess(data, boot):
    require(not data['error'] and not data.get('decode_errors'), 'BLE capture/decode failed')
    for key in ('info', 'final_info'):
        require(data[key]['boot_id'] == boot and tuple(data[key]['firmware']) in ((0,11,4),(0,11,5)), 'Boot/version changed')
    for key in ('before', 'after'):
        require(data[key]['state'] == 1 and data[key]['flags'] & 0x1c == 0, 'Not closed IDLE')
    for key in ('running', 'stopped'):
        s = data[key]
        require(s['rate_hz'] == 20 and s['channel_mask'] == 7 and s['dropped_sample_count'] == 0, 'Stream settings/drop failure')
    require(data['running']['streaming'] and not data['stopped']['streaming'], 'Stream lifecycle failure')
    groups = {}
    epochs = set()
    for f in data['frames']:
        require(f['stream_sequence'] == data['running']['stream_sequence'], 'Mixed streams')
        require(f['flags'] == 11, 'Invalid feedback, drop, state or output flags')
        require(f['channel'] in (1,2,4), 'Unexpected channel')
        epochs.add(f['activation_epoch_lsb'])
        group = groups.setdefault(f['sample_sequence'], {})
        require(f['channel'] not in group, 'Duplicate channel')
        group[f['channel']] = f
    require(len(epochs) == 1 and len(groups) >= 42, 'Epoch/coverage failure')
    seq = sorted(groups)
    require(all(b == a+1 for a,b in zip(seq,seq[1:])), 'Missing sample')
    # Only the final group can be truncated by STOP_TELEMETRY.
    if set(groups[seq[-1]]) != {1,2,4}:
        seq = seq[:-1]
    rows = [[], []]
    for index in seq:
        g = groups[index]
        require(set(g) == {1,2,4}, 'Incomplete interior sample')
        require(len({f['timestamp_ms'] for f in g.values()}) == 1, 'Unsynchronized channels')
        for side, field in enumerate(('left_raw', 'right_raw')):
            require(all(g[c][field] is not None for c in (1,2,4)), 'Invalid value')
            rows[side].append((g[1]['timestamp_ms']/1000,
                math.degrees(g[1][field]/1000), math.degrees(g[2][field]/100), g[4][field]/1000, True))
    result = []
    for side, values in enumerate(rows):
        ok, reason = evaluate(values)
        require(ok, f'Stationary BLE side={side}: {reason}')
        result.append({'samples': len(values), 'span_s': values[-1][0]-values[0][0],
            'angle_range_deg': max(v[1] for v in values)-min(v[1] for v in values),
            'max_abs_speed_deg_s': max(abs(v[2]) for v in values)})
    return result


def check(destination, boot):
    from stationary_ble_capture import main as capture
    folder = destination / ('stationary_'+datetime.now().strftime('%H%M%S_%f'))
    data = asyncio.run(capture(folder, seconds=3, expected_boot=boot))
    result = assess(data, boot)
    (folder/'accepted.json').write_text(json.dumps(result, indent=2))
    return result
