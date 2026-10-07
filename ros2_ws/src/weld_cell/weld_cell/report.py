"""Post-processing of a weld run: deviation statistics and plots for the paper."""
import csv
import json
import math
import os

import numpy as np


def _read(csv_path):
    with open(csv_path) as f:
        r = csv.reader(f)
        header = next(r)
        rows = list(r)
    cols = {h: [row[i] for row in rows] for i, h in enumerate(header)}
    num = {}
    for h, v in cols.items():
        if h == 'seam':
            num[h] = np.array(v)
        else:
            num[h] = np.array([float(x) if x not in ('', 'nan') else math.nan for x in v])
    return num


def add_deviation_stats(summary, csv_path):
    d = _read(csv_path)
    for s in summary['seams']:
        if s.get('status') != 'done':
            continue
        m = (d['seam'] == s['name']) & (d['arc'] > 0)
        for key, col in (('dev_nominal', 'dev_nominal_mm'), ('dev_actual_seam', 'dev_actual_seam_mm')):
            v = d[col][m]
            v = v[~np.isnan(v)]
            if len(v):
                s[f'{key}_mean_mm'] = round(float(v.mean()), 3)
                s[f'{key}_max_mm'] = round(float(v.max()), 3)
                s[f'{key}_rms_mm'] = round(float(np.sqrt((v ** 2).mean())), 3)
        sp = d['speed_mm_s'][m]
        if len(sp) > 10:
            core = sp[len(sp) // 10: -len(sp) // 10 or None]
            s['speed_measured_mm_s'] = round(float(np.median(core)), 2)
    return summary


def make_plots(log_dir):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    d = _read(os.path.join(log_dir, 'tcp_log.csv'))
    with open(os.path.join(log_dir, 'summary.json')) as f:
        summary = json.load(f)
    seams = [s['name'] for s in summary['seams'] if s.get('status') == 'done']
    t = d['t']
    arc = d['arc'] > 0

    # 1) TCP path, top view, arc-on in colour
    fig, ax = plt.subplots(figsize=(7, 6))
    ax.plot(d['x'], d['y'], color='0.75', lw=0.8, label='air / approach moves')
    for name in dict.fromkeys(seams):
        m = arc & (d['seam'] == name)
        ax.plot(d['x'][m], d['y'][m], '.', ms=2, label=f'weld: {name}')
    ax.set_xlabel('X, m')
    ax.set_ylabel('Y, m')
    ax.set_aspect('equal')
    ax.set_title('TCP path (top view)')
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(log_dir, 'tcp_path_top.png'), dpi=150)
    plt.close(fig)

    # 2) deviation during welding
    fig, axes = plt.subplots(len(set(seams)) or 1, 1, figsize=(8, 2.4 * max(1, len(set(seams)))), squeeze=False)
    for ax, name in zip(axes[:, 0], dict.fromkeys(seams)):
        m = arc & (d['seam'] == name)
        ax.plot(t[m] - t[m][0], d['dev_nominal_mm'][m], label='vs programmed path')
        if not np.all(np.isnan(d['dev_actual_seam_mm'][m])) and \
                np.nanmax(np.abs(d['dev_actual_seam_mm'][m] - d['dev_nominal_mm'][m])) > 0.05:
            ax.plot(t[m] - t[m][0], d['dev_actual_seam_mm'][m], label='vs real seam (part offset)')
        ax.set_ylabel('deviation, mm')
        ax.set_title(name, fontsize=9)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    axes[-1, 0].set_xlabel('time from arc on, s')
    fig.tight_layout()
    fig.savefig(os.path.join(log_dir, 'tcp_deviation.png'), dpi=150)
    plt.close(fig)

    # 3) TCP speed vs commanded
    fig, ax = plt.subplots(figsize=(9, 3.2))
    ax.plot(t, d['speed_mm_s'], lw=0.6, color='0.6', label='TCP speed (measured)')
    ax.plot(t[arc], d['speed_mm_s'][arc], '.', ms=1.5, color='tab:red', label='arc on')
    for s in summary['seams']:
        if s.get('status') == 'done':
            ax.axhline(s['speed_mm_s'], ls='--', lw=0.7, color='tab:blue')
    ax.set_ylim(0, max(40.0, np.nanpercentile(d['speed_mm_s'], 90) * 1.2))
    ax.set_xlabel('time, s')
    ax.set_ylabel('mm/s')
    ax.set_title('TCP speed (dashed: programmed weld speed)')
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(log_dir, 'tcp_speed.png'), dpi=150)
    plt.close(fig)

    # 4) joints
    fig, ax = plt.subplots(figsize=(9, 3.6))
    for j in ('j1', 'j2', 'j3', 'j4', 'j5', 'j6'):
        ax.plot(t, d[j], lw=0.9, label=j)
    ymin, ymax = ax.get_ylim()
    ax.fill_between(t, ymin, ymax, where=arc, color='tab:red', alpha=0.08, label='arc on')
    ax.set_xlabel('time, s')
    ax.set_ylabel('rad')
    ax.set_title('Joint positions over the cycle')
    ax.legend(fontsize=7, ncol=7)
    fig.tight_layout()
    fig.savefig(os.path.join(log_dir, 'joints.png'), dpi=150)
    plt.close(fig)
