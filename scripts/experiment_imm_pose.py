#!/usr/bin/python3
"""Paired deterministic replay: CV/CA IMM versus Pose-KF motion hypotheses.
Run with PYTHONNOUSERSITE=1 to use the system NumPy/Matplotlib ABI.
"""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import subprocess

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
ALGORITHMS = ['IMM_CV_CA', 'IMM_CV_CA_POSE', 'IMM_CV_POSE', 'POSE_MODEL_PV']
SCENARIOS = {
    'hover': {'trajectory': 'hover'},
    'straight': {'trajectory': 'straight'},
    'circle': {'trajectory': 'circle'},
    'helix': {'trajectory': 'helix'},
    'figure8': {'trajectory': 'figure8'},
    'aggressive': {'trajectory': 'figure8', 'frequency': .65},
    'vertical': {'trajectory': 'vertical'},
    'bad_attitude': {'trajectory': 'figure8', 'attitude_std': .2},
    'bad_velocity': {'trajectory': 'figure8', 'velocity_std': .5},
}


def truth(t, scenario):
    kind = scenario['trajectory']
    p = np.zeros((len(t), 3)); p[:, 2] = 2
    v = np.zeros_like(p); a = np.zeros_like(p)
    if kind == 'straight':
        v[:] = [1, .4, 0]; p += t[:, None] * v
    elif kind in ('circle', 'helix'):
        w = .35
        p[:, 0] = 4 * np.cos(w*t); p[:, 1] = 4 * np.sin(w*t)
        v[:, 0] = -4*w*np.sin(w*t); v[:, 1] = 4*w*np.cos(w*t)
        a[:, 0] = -4*w*w*np.cos(w*t); a[:, 1] = -4*w*w*np.sin(w*t)
        if kind == 'helix': p[:, 2] += .2*t; v[:, 2] = .2
    elif kind == 'figure8':
        w = scenario.get('frequency', .35)
        for axis, amplitude, frequency in [(0, 4, w), (1, 3, 2*w), (2, .6, .5)]:
            p[:, axis] += amplitude*np.sin(frequency*t)
            v[:, axis] = amplitude*frequency*np.cos(frequency*t)
            a[:, axis] = -amplitude*frequency**2*np.sin(frequency*t)
    elif kind == 'vertical':
        p[:, 2] += 2*np.sin(t); v[:, 2] = 2*np.cos(t); a[:, 2] = -2*np.sin(t)
    # Same construction and 9.8 gravity as fake_drone/poscmd_2_odom, yaw = 0.
    alpha = a + [0, 0, 9.8]
    xb = np.cross(np.array([0, 1, 0]), alpha)
    xb /= np.linalg.norm(xb, axis=1)[:, None]
    yb = np.cross(alpha, xb); yb /= np.linalg.norm(yb, axis=1)[:, None]
    zb = np.cross(xb, yb)
    rotation = Rotation.from_matrix(np.stack([xb, yb, zb], axis=2))
    return p, v, rotation


def dataset(path, scenario, seed, duration):
    t = np.arange(round(duration/.02)+1)*.02
    p, v, rotation = truth(t, scenario)
    rng = np.random.default_rng(seed)
    mp = p + rng.normal(size=p.shape)*.15
    mv = v + rng.normal(size=v.shape)*scenario.get('velocity_std', .1)
    q = (rotation * Rotation.from_rotvec(rng.normal(size=p.shape)*scenario.get('attitude_std', .05))).as_quat()
    data = np.column_stack([t, p, v, mp, mv, q[:, 3], q[:, :3]])
    np.savetxt(path, data, delimiter=',', fmt='%.17g', comments='',
               header='t,px,py,pz,vx,vy,vz,mx,my,mz,mvx,mvy,mvz,qw,qx,qy,qz')
    return hashlib.sha256(path.read_bytes()).hexdigest()


def analyze(output, scenarios, seeds):
    per_seed = []
    metrics = ['position', 'velocity', 'trajectory', 'fde_05', 'fde_1', 'fde_2']
    for scene in scenarios:
        for seed in seeds:
            with (output/scene/f'scores_{seed}.csv').open() as stream:
                rows = list(csv.DictReader(stream))
            origins = [[float(r['t']) for r in rows if r['algorithm'] == algorithm] for algorithm in ALGORITHMS]
            if any(times != origins[0] for times in origins[1:]):
                raise ValueError('Algorithms were not scored at identical origins')
            for algorithm in ALGORITHMS:
                selected = [r for r in rows if r['algorithm'] == algorithm]
                if not selected: raise ValueError(f'No scored rows for {scene}/{seed}/{algorithm}')
                record = {'scenario': scene, 'seed': seed, 'algorithm': algorithm, 'origins': len(selected)}
                for metric in metrics:
                    values = np.array([float(r[metric+'_sq']) for r in selected])
                    if not np.all(np.isfinite(values)) or np.any(values < 0):
                        raise ValueError('Invalid/nonfinite squared error in replay')
                    record[metric+'_rmse'] = float(np.sqrt(np.mean(values)))
                for metric in ['p_pose_kf', 'update_us', 'forecast_us']:
                    record[metric] = float(np.mean([float(r[metric]) for r in selected]))
                per_seed.append(record)
    summary = {}
    for scene in scenarios:
        summary[scene] = {}
        for algorithm in ALGORITHMS:
            rows = [r for r in per_seed if r['scenario'] == scene and r['algorithm'] == algorithm]
            summary[scene][algorithm] = {}
            for metric in [m+'_rmse' for m in metrics] + ['p_pose_kf', 'update_us', 'forecast_us']:
                values = [r[metric] for r in rows]
                summary[scene][algorithm][metric] = {'mean': float(np.mean(values)),
                    'std': float(np.std(values, ddof=1)) if len(values)>1 else 0}
        base = summary[scene][ALGORITHMS[0]]
        fused = summary[scene][ALGORITHMS[1]]
        gains = {}
        for metric in ['position', 'velocity', 'fde_05', 'fde_1', 'fde_2']:
            key = metric+'_rmse'
            gains[metric+'_percent'] = 100*(1-fused[key]['mean']/base[key]['mean'])
            pairs = [(next(r[key] for r in per_seed if r['scenario']==scene and r['seed']==seed and r['algorithm']==ALGORITHMS[0]),
                      next(r[key] for r in per_seed if r['scenario']==scene and r['seed']==seed and r['algorithm']==ALGORITHMS[1])) for seed in seeds]
            gains[metric+'_seeds_improved'] = sum(f < b for b, f in pairs)
        summary[scene]['fusion_gain'] = gains
    result = {'per_seed': per_seed, 'summary': summary,
              'note': 'RMSE mean/std across seeds; positive gain is improvement. Paired inputs, no tuning. '
                      'Metrics use common forecast origins after 3s with a full 2s truth horizon. '
                      'Trajectory RMSE averages squared displacement over 20 steps; it is not ADE.'}
    (output/'results.json').write_text(json.dumps(result, indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    x = np.arange(len(scenarios)); labels = list(scenarios)
    fig, axes = plt.subplots(2, 2, figsize=(16, 10), layout='constrained')
    colors = ['#64748b', '#0284c7', '#f59e0b', '#a855f7']
    for ax, metric, title in [(axes[0,0], 'fde_1_rmse', '1 s endpoint RMSE'),
                              (axes[0,1], 'fde_2_rmse', '2 s endpoint RMSE')]:
        for j, algorithm in enumerate(ALGORITHMS):
            values = [summary[s][algorithm][metric]['mean'] for s in scenarios]
            errors = [summary[s][algorithm][metric]['std'] for s in scenarios]
            ax.bar(x+(j-1.5)*.2, values, .19, yerr=errors, capsize=2, label=algorithm, color=colors[j])
        ax.set_title(title); ax.set_ylabel('m'); ax.set_xticks(x, labels, rotation=25, ha='right'); ax.grid(axis='y', alpha=.2)
    axes[0,0].legend(fontsize=8)
    ax = axes[1,0]
    for j, metric in enumerate(['fde_05', 'fde_1', 'fde_2']):
        ax.bar(x+(j-1)*.25, [summary[s]['fusion_gain'][metric+'_percent'] for s in scenarios], .24,
               label={'fde_05':'.5 s','fde_1':'1 s','fde_2':'2 s'}[metric])
    ax.axhline(0, color='black', linewidth=.8); ax.set_ylabel('reduction (%)'); ax.set_title('CV/CA/Pose versus CV/CA; positive is better')
    ax.set_xticks(x, labels, rotation=25, ha='right'); ax.legend(); ax.grid(axis='y', alpha=.2)
    ax = axes[1,1]
    ax.bar(x, [summary[s][ALGORITHMS[1]]['p_pose_kf']['mean'] for s in scenarios], color=colors[1])
    ax.set_ylim(0,1); ax.set_title('Mean posterior probability of Pose hypothesis'); ax.set_ylabel('probability')
    ax.set_xticks(x, labels, rotation=25, ha='right'); ax.grid(axis='y', alpha=.2)
    fig.suptitle(f'Paired IMM Pose experiment | {len(seeds)} seeds | same p/v/attitude observations | bars: seed mean +/- SD', fontsize=14)
    fig.savefig(output/'comparison.png', dpi=150); plt.close(fig)
    print(json.dumps({s:summary[s]['fusion_gain'] for s in scenarios}, indent=2), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT/'outputs/imm_pose_experiment_20261111')
    parser.add_argument('--executable', type=Path, default=ROOT/'build/predictor/imm_pose_replay')
    parser.add_argument('--seeds', type=int, nargs='+', default=[11,22,33,44,55])
    parser.add_argument('--duration', type=float, default=20)
    parser.add_argument('--scenarios', nargs='+', choices=list(SCENARIOS), default=list(SCENARIOS))
    parser.add_argument('--analyze-only', action='store_true')
    args = parser.parse_args()
    if args.duration <= 5 or abs(args.duration/.02-round(args.duration/.02)) > 1e-8:
        raise ValueError('duration must exceed 5 seconds and be a multiple of .02')
    output = args.output.resolve(); output.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('MPLCONFIGDIR', str(output/'.matplotlib'))
    scenarios = {name:SCENARIOS[name] for name in args.scenarios}
    manifest = {'duration': args.duration, 'dt': .02, 'warmup':3, 'forecast_interval': .2,
                'forecast_step': .1, 'forecast_count':20, 'seeds':args.seeds, 'scenarios':scenarios,
                'executable_sha256': hashlib.sha256(args.executable.read_bytes()).hexdigest(),
                'position_std':.15, 'velocity_std':.1, 'attitude_std':.05,
                'cv_accel_density':.3, 'pose_accel_density':.3, 'ca_jerk_density':1.,
                'angular_accel_density':.3, 'stay_probability':.98,
                'pose_gravity':9.81, 'simulator_gravity':9.8, 'observation_sha256':{}}
    if not args.analyze_only:
        for scene, scenario in scenarios.items():
            directory = output/scene; directory.mkdir(exist_ok=True)
            for seed in args.seeds:
                samples = directory/f'observations_{seed}.csv'; scores = directory/f'scores_{seed}.csv'
                if scores.exists(): raise FileExistsError(f'Refusing to overwrite {scores}')
                digest = dataset(samples, scenario, seed, args.duration)
                manifest['observation_sha256'][f'{scene}/{seed}'] = digest
                print(f'Running {scene}, seed {seed}', flush=True)
                subprocess.run([str(args.executable), str(samples), str(scores),
                                str(scenario.get('velocity_std', .1)), str(scenario.get('attitude_std', .05))], check=True)
        (output/'experiment.json').write_text(json.dumps(manifest, indent=2)+'\n')
    analyze(output, scenarios, args.seeds)


if __name__ == '__main__':
    main()
