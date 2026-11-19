"""Pure stationary-window predicate; BLE gate validates transport and device state.

This function alone does not authorize START or change firmware guards.
"""
import math
import statistics


def evaluate(rows):
    """Rows: (seconds, angle_deg, velocity_deg_s, torque_nm, valid)."""
    if len(rows) < 20:
        return False, 'insufficient samples'
    if any(not r[4] or not all(math.isfinite(x) for x in r[:4]) for r in rows):
        return False, 'invalid feedback'
    t, q, v, torque, _ = zip(*rows)
    gaps = [b-a for a, b in zip(t, t[1:])]
    if t[-1]-t[0] < 2 or min(gaps) <= 0 or max(gaps) > .15:
        return False, 'coverage or timestamp failure'
    if max(q)-min(q) > .1:
        return False, 'angle excursion'
    if max(map(abs, v)) > 20:
        return False, 'raw speed limit'
    if abs(statistics.mean(v)) > 1 or statistics.median(map(abs, v)) > 5:
        return False, 'persistent speed'
    if max(map(abs, torque)) > .5:
        return False, 'nonzero torque'
    return True, 'stationary window accepted'
