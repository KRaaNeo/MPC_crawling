"""
plot_torso6d.py — Visualization for torso 6D cooperative control simulation.

Produces 5-panel figure:
    ① Torso advancement (x axis)
    ② EE distance to target (log scale)
    ③ Torso tracking error
    ④ Joint torques (both arms)
    ⑤ Structure drift
"""

import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def plot_results(log, save_path='sim_torso6d_results.png', baseline_log=None):
    """Plot simulation results.

    Parameters
    ----------
    log : dict
        Simulation log from sim_torso6d.run_simulation().
    save_path : str
        Output file path.
    baseline_log : dict, optional
        Baseline log (old CoM architecture) for comparison.
    """
    t = np.array(log['t'])
    pt = np.array(log['p_torso'])
    pt_ref = np.array(log['p_torso_ref'])
    d = np.array(log['d_grip_b'])
    da = np.array(log['d_grip_a'])
    tau = np.array(log['tau'])
    et = np.array(log['e_torso_pos'])
    sp = np.array(log['struct_pos'])
    ph = log['phase']

    def shade(ax):
        for i in range(len(t)):
            if ph[i] == 'DS':
                ax.axvspan(t[i]-.05, t[i]+.05, alpha=.08, color='blue')
            elif ph[i] == 'EXT':
                ax.axvspan(t[i]-.05, t[i]+.05, alpha=.08, color='red')
        for i in range(1, len(ph)):
            if ph[i] != ph[i-1]:
                ax.axvline(t[i], color='gray', ls=':', alpha=.5)

    fig, axes = plt.subplots(5, 1, figsize=(14, 20), sharex=True)

    docked = d.min() < 0.005
    title = f'VISPA — Tâche Torse 6D (τ_max=10 Nm)'
    if docked:
        title += f' — DOCK RÉEL à {d.min()*1000:.1f}mm'
    fig.suptitle(title, fontsize=14, fontweight='bold')

    # ① Torso x
    ax = axes[0]; shade(ax)
    ax.plot(t, pt[:,0]*100, 'r-', lw=2.5, label='torse x (tâche torse 6D)')
    ax.plot(t, pt_ref[:,0]*100, 'r--', lw=1.5, alpha=.5, label='torse x ref')
    if baseline_log:
        t0 = np.array(baseline_log['t'])
        pt0 = np.array(baseline_log['p_torso'])
        n = min(len(t0), len(pt0))
        ax.plot(t0[:n], pt0[:n,0]*100, 'b-', lw=1.5, alpha=.4,
                label='torse x (ancien: CoM)')
    ax.axhline(0, color='k', ls='-', alpha=.15)
    ax.set_ylabel('Position x [cm]')
    ax.set_title('① Avancement du torse')
    ax.legend(fontsize=9); ax.grid(True, alpha=.3)

    # ② EE distance (log)
    ax = axes[1]; shade(ax)
    ax.semilogy(t, d*100, 'r-', lw=2.5, label='||grip_B−4b|| (torse 6D)')
    if baseline_log:
        d0 = np.array(baseline_log['d_grip_b'])
        n = min(len(t0), len(d0))
        ax.semilogy(t0[:n], d0[:n]*100, 'b-', lw=1.5, alpha=.4,
                    label='||grip_B−4b|| (ancien)')
    ax.axhline(0.5, color='g', ls='--', lw=2, label='seuil dock 5mm')
    i_min = np.argmin(d)
    ax.plot(t[i_min], d[i_min]*100, 'r*', ms=15, zorder=5,
            label=f'min: {d[i_min]*1000:.1f}mm @ {t[i_min]:.1f}s')
    ax.set_ylabel('Distance EE [cm] (log)')
    ax.set_title('② Tracking effecteur B → ancre 4b')
    ax.legend(fontsize=9); ax.grid(True, alpha=.3, which='both')
    ax.set_ylim([0.1, 200])

    # ③ Torso error
    ax = axes[2]; shade(ax)
    ax.plot(t, et*100, 'r-', lw=2, label='||e_torso_pos||')
    ax.set_ylabel('Erreur torse [cm]')
    ax.set_title('③ Erreur de tracking torse 6D')
    ax.legend(fontsize=9); ax.grid(True, alpha=.3)

    # ④ Torques
    ax = axes[3]; shade(ax)
    for j in range(6):
        ax.plot(t, tau[:,j], '-', color='C0', alpha=.3, lw=1)
    for j in range(6, 12):
        ax.plot(t, tau[:,j], '-', color='C1', alpha=.3, lw=1)
    ax.plot(t, np.max(np.abs(tau), axis=1), 'k-', lw=2, label='max |τ|')
    ax.axhline(10, color='r', ls='--', lw=1.5, label='τ_max=10Nm')
    ax.axhline(-10, color='r', ls='--', lw=1.5)
    ax.plot([], [], '-', color='C0', lw=2, label='bras A (stance/push)')
    ax.plot([], [], '-', color='C1', lw=2, label='bras B (swing)')
    ax.set_ylabel('Couple [Nm]')
    ax.set_title('④ Couples articulaires')
    ax.legend(loc='upper right', fontsize=9); ax.grid(True, alpha=.3)

    # ⑤ Structure drift
    ax = axes[4]; shade(ax)
    sd = np.linalg.norm(sp - sp[0], axis=1) * 100
    ax.plot(t, sd, 'r-', lw=2, label='||Δp_struct|| (torse 6D)')
    if baseline_log:
        sp0 = np.array(baseline_log['struct_pos'])
        sd0 = np.linalg.norm(sp0 - sp0[0], axis=1) * 100
        n = min(len(t0), len(sd0))
        ax.plot(t0[:n], sd0[:n], 'b-', lw=1.5, alpha=.4,
                label='||Δp_struct|| (ancien)')
    ax.plot(t, (sp[:,0] - sp[0,0])*100, 'r--', lw=1, alpha=.6,
            label='Δx struct')
    ax.set_ylabel('Drift [cm]'); ax.set_xlabel('Temps [s]')
    ax.set_title('⑤ Dérive de la structure (500 kg)')
    ax.legend(fontsize=9); ax.grid(True, alpha=.3)

    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    print(f"Saved: {save_path}")
    return fig


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('logfile', default='sim_torso6d_log.json')
    parser.add_argument('--baseline', default=None)
    parser.add_argument('-o', '--output', default='sim_torso6d_results.png')
    args = parser.parse_args()

    with open(args.logfile) as f:
        log = json.load(f)
    baseline = None
    if args.baseline:
        with open(args.baseline) as f:
            baseline = json.load(f)

    plot_results(log, save_path=args.output, baseline_log=baseline)
