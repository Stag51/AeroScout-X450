#!/usr/bin/env python3
"""Compare distance estimates with Gazebo ground truth from a metrics_logger CSV.
usage: python3 analyze_distance.py flight.csv [lag_samples]
"""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "ros"))
import csv, math, statistics as st, sys
from distance_estimator import ground_distance, height_proxy

path = sys.argv[1]
rows = list(csv.DictReader(open(path)))
F = lambda r, k: float(r[k])

def usable(r):
    return (r['found'] == '1.0000' and F(r, 'bbox_age') < 0.15 and F(r, 'drone_z') > 1.0
            and 0.05 < F(r, 'exp_v') < 0.95)

def pct(v, p):
    v = sorted(v); return v[min(len(v) - 1, int(p / 100 * len(v)))]

def evaluate(lag, k=None, verbose=False):
    out = []
    for i, r in enumerate(rows):
        j = i - lag
        if not usable(r) or not (0 <= j < len(rows)):
            continue
        t = rows[j]                                  # truth sampled `lag` rows earlier
        g = ground_distance(F(r, 'cx'), F(r, 'cy') + F(r, 'h') / 2, F(t, 'cam_z'),
                            math.radians(F(t, 'roll_deg')), math.radians(F(t, 'pitch_deg')),
                            math.radians(F(t, 'yaw_deg')))
        if g is None:
            continue
        out.append((F(t, 'true_range'), g['range'], F(t, 'true_ground'), g['ground'],
                    F(r, 'h'), F(t, 'true_fwd'), g['fwd'], F(t, 'true_lat'), g['lat']))
    return out

print(f'{path}: {len(rows)} rows, {sum(usable(r) for r in rows)} usable')
print('\nLag test (is the box older than the pose?). Mean abs range error by lag:')
best = None
for lag in range(0, 5):
    o = evaluate(lag)
    e = st.mean(abs(a[1] - a[0]) for a in o)
    print(f'  lag {lag} sample(s) = {lag * 0.1:.1f} s  ->  {e:.3f} m   (n={len(o)})')
    if best is None or e < best[1]:
        best = (lag, e)
lag = int(sys.argv[2]) if len(sys.argv) > 2 else best[0]
o = evaluate(lag)
print(f'\nUsing lag = {lag} sample(s)')

# fit the box-height proxy constant on range: range ~ k / h  (best case for the old method)
k = st.median(a[0] * a[4] for a in o)
print(f'Old method: range = k / box_height, k fitted = {k:.3f} (best case: fitted on this same data)')

def report(name, errs, ref):
    ab = [abs(e) for e in errs]
    rel = [abs(e) / r * 100 for e, r in zip(errs, ref)]
    print(f'{name:30s} bias {st.mean(errs):+.3f} m | mean abs {st.mean(ab):.3f} m | median {st.median(ab):.3f} m | '
          f'p95 {pct(ab, 95):.3f} m | mean {st.mean(rel):.1f}% | p95 {pct(rel, 95):.1f}%')

print('\nRANGE (camera to feet), error = estimate - truth')
report('ground-plane (truth attitude)', [a[1] - a[0] for a in o], [a[0] for a in o])
report('old box-height proxy', [height_proxy(a[4], k) - a[0] for a in o], [a[0] for a in o])
print('\nGROUND DISTANCE')
report('ground-plane (truth attitude)', [a[3] - a[2] for a in o], [a[2] for a in o])
print('\nFORWARD / SIDEWAYS offset (heading frame)')
report('forward', [a[6] - a[5] for a in o], [a[5] for a in o])
report('lateral (m, % unreliable near 0)', [a[8] - a[7] for a in o], [max(abs(a[7]), 0.5) for a in o])

print('\nRange error by true range (ground-plane | old proxy), mean abs in m:')
for lo, hi in [(2, 3), (3, 4), (4, 5), (5, 7)]:
    s = [a for a in o if lo <= a[0] < hi]
    if len(s) > 5:
        print(f'  {lo}-{hi} m  n={len(s):4d}   ground-plane {st.mean(abs(a[1]-a[0]) for a in s):.3f}   '
              f'old proxy {st.mean(abs(height_proxy(a[4], k)-a[0]) for a in s):.3f}')