#!/usr/bin/env python3
"""Make the result plots and print the results table from a flight CSV that has est_* columns.
usage: python3 make_plots.py flight3.csv [output_folder]
The 'old method' is range = K_OLD / box_height, with K_OLD fitted on flight 1 and not changed since.
"""
import csv, math, os, statistics as st, sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import distance_estimator as de

K_OLD = 1.226          # fitted on flight 1
BASE_Z_OLD = 0.087     # spawn height used before the correction (for the 'uncorrected' numbers)
SURFACE, INK, INK2, GRID = '#fcfcfb', '#0b0b0b', '#52514e', '#e4e3df'
BLUE, ORANGE = '#2a78d6', '#eb6834'      # validated pair (CVD dE 24.7)

path = sys.argv[1]
out = sys.argv[2] if len(sys.argv) > 2 else os.path.dirname(os.path.abspath(path))
os.makedirs(out, exist_ok=True)
F = lambda r, k: float(r[k])
rows = list(csv.DictReader(open(path)))

def ok(r):
    return (r['found'] == '1.0000' and F(r, 'bbox_age') < 0.15 and F(r, 'drone_z') > 1.0 and
            0.05 < F(r, 'exp_v') < 0.95 and r['est_valid'] == '1.0000' and F(r, 'est_age') < 0.15)

D = []
for i in range(1, len(rows)):
    r, t = rows[i], rows[i - 1]           # truth from the row before: the image is slightly older than the pose
    if not ok(r):
        continue
    raw = de.ground_distance(F(r, 'cx'), F(r, 'cy') + F(r, 'h') / 2, F(r, 'est_h') + (BASE_Z_OLD - 0.053),
                             math.radians(F(r, 'est_roll')), math.radians(F(r, 'est_pitch')), 0)
    D.append(dict(true=F(t, 'true_range'), new=F(r, 'est_range'), raw=raw['range'],
                  old=K_OLD / F(r, 'h'), lat_t=F(t, 'true_lat'), lat_e=F(r, 'est_lat')))
n = len(D)
ref = [d['true'] for d in D]
mabs = lambda e: st.mean(abs(x) for x in e)
mpct = lambda e: st.mean(abs(x) / r * 100 for x, r in zip(e, ref))
p95 = lambda e: sorted(abs(x) for x in e)[int(0.95 * len(e))]
E = {k: [d[k] - d['true'] for d in D] for k in ('old', 'raw', 'new')}

print(f'{path}: {n} usable frames')
print('| Method | Mean error | % of distance | Worst 5% | Largest |')
print('|---|---|---|---|---|')
for name, k in [('Old box-height proxy', 'old'), ('Ground-plane geometry, uncorrected', 'raw'),
                ('Ground-plane geometry, calibrated', 'new')]:
    e = E[k]
    print(f'| {name} | {mabs(e):.3f} m | {mpct(e):.1f}% | {p95(e):.3f} m | {max(abs(x) for x in e):.2f} m |')

def style(ax):
    ax.set_facecolor(SURFACE)
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)
    for s in ('left', 'bottom'):
        ax.spines[s].set_color(GRID)
    ax.tick_params(colors=INK2, labelsize=9, length=0)
    ax.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)

# ---- plot 1: estimate vs truth ------------------------------------------------------------
fig, axes = plt.subplots(1, 2, figsize=(9.6, 4.6), dpi=160, sharex=True, sharey=True)
fig.patch.set_facecolor(SURFACE)
lim = (2.5, 6.5)
for ax, key, title, colour in [(axes[0], 'old', 'Old: box height only', ORANGE),
                               (axes[1], 'new', 'New: ground-plane geometry', BLUE)]:
    style(ax)
    ax.plot(lim, lim, color=INK2, linewidth=1, linestyle=(0, (4, 3)), zorder=1)
    ax.scatter(ref, [d[key] for d in D], s=7, color=colour, alpha=0.35, linewidths=0, zorder=2)
    ax.set_xlim(lim); ax.set_ylim(2.0, 7.2)
    ax.set_title(f'{title}\nmean error {mabs(E[key]):.2f} m ({mpct(E[key]):.1f}%)', color=INK, fontsize=10.5,
                 loc='left', pad=8)
    ax.set_xlabel('True distance from Gazebo (m)', color=INK2, fontsize=9)
axes[0].set_ylabel('Estimated distance (m)', color=INK2, fontsize=9)
axes[0].text(6.4, 2.2, 'dashed line = perfect', color=INK2, fontsize=8, ha='right')
fig.suptitle(f'Distance to the person: estimate vs truth ({n} frames from flight 3)', color=INK, fontsize=12,
             x=0.01, ha='left')
fig.tight_layout(rect=(0, 0, 1, 0.94))
fig.savefig(os.path.join(out, 'estimate_vs_truth.png'), facecolor=SURFACE)

# ---- plot 2: error by distance --------------------------------------------------------------
bins = [(2, 3), (3, 4), (4, 5), (5, 6), (6, 9)]
labels, old_v, new_v = [], [], []
for lo, hi in bins:
    s = [d for d in D if lo <= d['true'] < hi]
    if len(s) < 5:
        continue
    labels.append((f'{lo}-{hi} m' if hi < 9 else f'{lo}+ m') + f'\nn={len(s)}')
    old_v.append(mabs([d['old'] - d['true'] for d in s]))
    new_v.append(mabs([d['new'] - d['true'] for d in s]))
fig, ax = plt.subplots(figsize=(9.6, 4.6), dpi=160)
fig.patch.set_facecolor(SURFACE); style(ax); ax.grid(axis='x', visible=False)
x = list(range(len(labels))); w = 0.36
b1 = ax.bar([i - w / 2 - 0.01 for i in x], old_v, w, color=ORANGE, label='Old: box height only', zorder=3)
b2 = ax.bar([i + w / 2 + 0.01 for i in x], new_v, w, color=BLUE, label='New: ground-plane geometry', zorder=3)
for bars in (b1, b2):
    for b in bars:
        ax.text(b.get_x() + b.get_width() / 2, b.get_height() + 0.012, f'{b.get_height():.2f}', ha='center',
                color=INK, fontsize=8.5)
ax.set_xticks(x); ax.set_xticklabels(labels)
ax.set_ylabel('Mean error (m)', color=INK2, fontsize=9)
ax.set_ylim(0, max(old_v + new_v) * 1.18)
ax.legend(frameon=False, fontsize=9, labelcolor=INK, loc='upper left')
ax.set_title('Mean distance error by how far away the person is (flight 3, lower is better)', color=INK,
             fontsize=11.5, loc='left', pad=10)
fig.tight_layout()
fig.savefig(os.path.join(out, 'error_by_distance.png'), facecolor=SURFACE)
print('saved plots to', out)
