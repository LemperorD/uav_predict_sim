#!/usr/bin/python3
"""Diagnose the Pose-KF scalar thrust deviation w without adding a w estimator.
Use PYTHONNOUSERSITE=1 /usr/bin/python3 to use system scientific libraries.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess

import numpy as np
from scipy.spatial.transform import Rotation
from experiment_imm_pose import SCENARIOS, truth

ROOT = Path(__file__).resolve().parents[1]
GRAVITY = 9.8  # Match fake_drone attitude construction; isolate the thrust hypothesis.
DT = .02


def acceleration(t, scenario):
    a = np.zeros((len(t), 3))
    kind = scenario['trajectory']
    if kind in ('circle', 'helix'):
        a[:, 0] = -4*.35**2*np.cos(.35*t)
        a[:, 1] = -4*.35**2*np.sin(.35*t)
    elif kind == 'figure8':
        w = scenario.get('frequency', .35)
        for k, amp, freq in [(0,4,w), (1,3,2*w), (2,.6,.5)]:
            a[:, k] = -amp*freq**2*np.sin(freq*t)
    elif kind == 'vertical':
        a[:, 2] = -2*np.sin(t)
    return a


def decompose(a, axis, gravity=GRAVITY):
    specific_force = a + np.array([0, 0, gravity])
    thrust_per_mass = np.sum(specific_force*axis, axis=1)
    w = thrust_per_mass-gravity
    nominal = gravity*(axis-np.array([0, 0, 1]))
    perpendicular = specific_force-thrust_per_mass[:, None]*axis
    return w, a-nominal, perpendicular


def correlation(values, lag):
    centered = values-np.mean(values)
    energy = np.dot(centered, centered)
    if energy < 1e-18: return None
    return float(np.dot(centered[:-lag], centered[lag:])/energy) if lag else 1.


def rms(values):
    return float(np.sqrt(np.mean(np.asarray(values)**2)))


def write_csv(path, columns):
    lengths = {len(v) for v in columns.values()}
    if len(lengths) != 1: raise ValueError('CSV column length mismatch')
    np.savetxt(path, np.column_stack(list(columns.values())), fmt='%.17g', delimiter=',',
               comments='', header=','.join(columns))


def proxy_from_velocity(t, velocity, axis, window):
    # Diagnostic using velocity observations, which standalone PoseKF does NOT use.
    raw_a = np.full_like(velocity, np.nan)
    raw_a[1:] = np.diff(velocity, axis=0)/DT
    raw = np.sum((raw_a+[0,0,GRAVITY])*axis, axis=1)-GRAVITY
    slope = np.full_like(velocity, np.nan)
    axis_average = np.full_like(axis, np.nan)
    for i in range(window-1, len(t)):
        times = t[i-window+1:i+1]
        times = times-np.mean(times)
        slope[i] = np.sum(times[:, None]*velocity[i-window+1:i+1], axis=0)/np.dot(times,times)
        direction = np.mean(axis[i-window+1:i+1], axis=0)
        axis_average[i] = direction/np.linalg.norm(direction)
    smooth = np.sum((slope+[0,0,GRAVITY])*axis_average, axis=1)-GRAVITY
    return raw, smooth


def run_scene(name, scenario, args, output):
    directory = output/name
    directory.mkdir(exist_ok=True)
    t = np.arange(round(args.duration/DT)+1)*DT
    p, v, rotation = truth(t, scenario)
    a = acceleration(t, scenario)
    axis = rotation.apply(np.tile([0,0,1],(len(t),1)))
    # The generation model must satisfy force alignment, otherwise a scalar w
    # alone cannot explain its acceleration, even with perfect attitude.
    force = a+[0,0,GRAVITY]
    w_true, residual_true, perpendicular_true = decompose(a, axis)
    assert np.max(np.linalg.norm(perpendicular_true, axis=1)) < 1e-10
    assert np.max(np.abs(w_true-(np.linalg.norm(force,axis=1)-GRAVITY))) < 1e-10
    assert np.max(np.abs(w_true-((GRAVITY+a[:,2])/axis[:,2]-GRAVITY))) < 1e-10
    # Validate the hand-written analytic acceleration against the shared velocity generator.
    if len(t)>2:
        assert np.max(np.abs(np.gradient(v,DT,axis=0)[1:-1]-a[1:-1])) < .002
    rng = np.random.default_rng(args.seed)
    measured_p = p+rng.normal(size=p.shape)*.15
    measured_v = v+rng.normal(size=v.shape)*.1
    attitude_std = scenario.get('attitude_std',.05)
    measured_rotation = rotation*Rotation.from_rotvec(rng.normal(size=p.shape)*attitude_std)
    measured_axis = measured_rotation.apply(np.tile([0,0,1],(len(t),1)))
    window = round(args.window/DT)+1
    raw, smooth = proxy_from_velocity(t,measured_v,measured_axis,window)
    w_projected, residual_observed, perpendicular_observed = decompose(a, measured_axis)
    # Oracle w with noisy attitude is still limited by attitude error.
    residual_oracle = a-((GRAVITY+w_true)[:,None]*measured_axis-[0,0,GRAVITY])
    outputs = {}
    for variant, q in [('observed_attitude',measured_rotation),('true_attitude',rotation)]:
        quaternion = q.as_quat()
        observations = {'t':t, **{f'm{xyz}':measured_p[:,k] for k,xyz in enumerate('xyz')},
                        'qw':quaternion[:,3], **{f'q{xyz}':quaternion[:,k] for k,xyz in enumerate('xyz')}}
        source = directory/f'{variant}_input.csv'
        target = directory/f'{variant}_filter.csv'
        write_csv(source,observations)
        subprocess.run([str(args.executable),str(source),str(target),str(GRAVITY)],check=True)
        outputs[variant] = np.genfromtxt(target,delimiter=',',names=True)
    columns = {'t':t, 'w_nominal_mean':np.zeros_like(t), 'w_required_truth':w_true,
               'thrust_per_mass_truth':GRAVITY+w_true,
               'tilt_deg':np.degrees(np.arccos(np.clip(axis[:,2],-1,1))),
               'w_level_altitude':GRAVITY*(1/axis[:,2]-1),
               'w_velocity_difference_raw':raw,'w_velocity_window_proxy':smooth,
               'w_projected_using_truth_accel_noisy_attitude':w_projected}
    for prefix, values in [('truth_p',p),('truth_v',v),('truth_a',a),('measurement_p',measured_p),
                           ('measurement_v',measured_v),('true_b3',axis),('measured_b3',measured_axis),
                           ('nominal_residual_true_attitude',residual_true),
                           ('nominal_residual_noisy_attitude',residual_observed),
                           ('oracle_w_residual_noisy_attitude',residual_oracle),
                           ('perpendicular_residual_noisy_attitude',perpendicular_observed)]:
        for k, xyz in enumerate('xyz'): columns[f'{prefix}_{xyz}'] = values[:,k]
    for label, values in [('true_attitude',perpendicular_true),('noisy_attitude',perpendicular_observed)]:
        columns[f'perpendicular_norm_{label}'] = np.linalg.norm(values,axis=1)
    for label, result in outputs.items():
        for field in result.dtype.names:
            if field!='t': columns[f'pose_{label}_{field}'] = result[field]
        estimate_v = np.column_stack([result['estimate_v'+xyz] for xyz in 'xyz'])
        columns[f'pose_{label}_velocity_error'] = np.linalg.norm(estimate_v-v,axis=1)
    write_csv(directory/'diagnostics.csv',columns)
    steady = t>=3
    delayed_reference = np.interp(t-args.window/2,t,w_true)
    valid = np.isfinite(smooth)&steady
    summary = {
        'samples':len(t),'w_mean':float(np.mean(w_true)), 'w_std':float(np.std(w_true)),
        'w_min':float(np.min(w_true)), 'w_max':float(np.max(w_true)), 'w_rms':rms(w_true),
        'w_lag_20ms':correlation(w_true,1),'w_lag_500ms':correlation(w_true,25),
        'w_lag_1s':correlation(w_true,50), 'maximum_tilt_deg':float(np.max(columns['tilt_deg'])),
        'nominal_acceleration_rmse_true_attitude':rms(np.linalg.norm(residual_true,axis=1)),
        'nominal_acceleration_rmse_noisy_attitude':rms(np.linalg.norm(residual_observed,axis=1)),
        'oracle_w_acceleration_rmse_noisy_attitude':rms(np.linalg.norm(residual_oracle,axis=1)),
        'minimum_scalar_fit_acceleration_rmse_noisy_attitude':rms(np.linalg.norm(perpendicular_observed,axis=1)),
        'raw_velocity_proxy_rmse':rms(raw[1:]-w_true[1:]),
        'window_proxy_rmse_current_time':rms(smooth[valid]-w_true[valid]),
        'window_proxy_rmse_half_window_delayed':rms(smooth[valid]-delayed_reference[valid]),
        'pose_observed_attitude_velocity_rmse_after_3s':rms(columns['pose_observed_attitude_velocity_error'][steady]),
        'pose_true_attitude_velocity_rmse_after_3s':rms(columns['pose_true_attitude_velocity_error'][steady]),
        'attitude_std':attitude_std,
    }
    for variant in outputs:
        summary[f'{variant}_input_sha256'] = hashlib.sha256((directory/f'{variant}_input.csv').read_bytes()).hexdigest()
    return columns,summary


def plots(output, all_columns, summaries, window, seed):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False})
    blue='#0284c7'; orange='#ea580c'; gray='#94a3b8'; green='#16a34a'
    fig, axes = plt.subplots(2,3,figsize=(16,8),layout='constrained')
    for ax,(scene,c) in zip(axes.flat,all_columns.items()):
        ax.plot(c['t'],c['w_required_truth'],color=blue,label='required w (truth diagnostic)',linewidth=1.7)
        ax.plot(c['t'],c['w_velocity_window_proxy'],color=orange,alpha=.75,linewidth=.8,label=f'causal {window:g}s velocity proxy')
        ax.axhline(0,color='black',linestyle='--',linewidth=1,label='w used in mean propagation')
        ax.set_title(scene); ax.set_xlabel('time (s)'); ax.set_ylabel('w (m/s²)'); ax.grid(alpha=.2)
    axes[0,0].legend(fontsize=8,loc='upper right')
    duration=next(iter(all_columns.values()))['t'][-1]
    fig.suptitle(f'Pose-KF thrust-deviation audit | 50 Hz, {duration:g} s, seed {seed} | w is not a filter state',fontsize=15)
    fig.savefig(output/'w_overview.png',dpi=160); plt.close(fig)
    scene='aggressive';c=all_columns[scene];t=c['t']
    fig,axes=plt.subplots(3,2,figsize=(16,12),layout='constrained')
    ax=axes[0,0]
    ax.plot(t,c['w_required_truth'],color=blue,label='required w')
    ax.plot(t,c['w_level_altitude'],color=green,linestyle=':',label='g (sec(tilt) - 1), if a_z = 0')
    ax.plot(t,c['w_velocity_window_proxy'],color=orange,alpha=.7,label=f'causal {window:g}s velocity proxy')
    ax.axhline(0,color='black',linestyle='--',label='nominal w = 0');ax.set_title('Thrust variation has maneuver structure');ax.set_ylabel('m/s²');ax.legend(fontsize=8)
    ax=axes[0,1]
    ax.plot(t,c['w_velocity_difference_raw'],color=gray,linewidth=.6,label='raw velocity-difference proxy')
    ax.plot(t,c['w_required_truth'],color=blue,label='required w');ax.set_title('Differentiating noisy velocity amplifies noise');ax.set_ylabel('m/s²');ax.legend(fontsize=8)
    ax=axes[1,0]
    ax.plot(t,c['tilt_deg'],color=blue,label='true tilt');ax.set_title('Tilt and vertical acceleration');ax.set_ylabel('tilt (deg)')
    other=ax.twinx();other.plot(t,c['truth_a_z'],color=orange,label='true vertical acceleration');other.set_ylabel('a_z (m/s²)',color=orange);other.spines['right'].set_visible(True)
    ax=axes[1,1]
    def norm(prefix): return np.linalg.norm(np.column_stack([c[prefix+'_'+xyz] for xyz in 'xyz']),axis=1)
    ax.plot(t,norm('nominal_residual_true_attitude'),color=blue,label='w=0, true attitude (= |w|)')
    ax.plot(t,norm('nominal_residual_noisy_attitude'),color=gray,alpha=.5,linewidth=.7,label='w=0, noisy attitude')
    ax.plot(t,norm('oracle_w_residual_noisy_attitude'),color=orange,alpha=.8,linewidth=.7,label='true w, noisy attitude (oracle only)')
    ax.set_title('Acceleration-model error; oracle is not a deployed estimator');ax.set_ylabel('error norm (m/s²)');ax.legend(fontsize=8)
    ax=axes[2,0]
    for name in ['figure8','aggressive','vertical']:
        series=all_columns[name]['w_required_truth'];lags=np.arange(101)
        ax.plot(lags*DT,[correlation(series,int(k)) for k in lags],label=name)
    ax.axhline(0,color='black',linestyle='--',linewidth=.8);ax.set_title('Centered autocorrelation: persistent, not white');ax.set_xlabel('lag (s)');ax.set_ylabel('autocorrelation');ax.legend(fontsize=8)
    ax=axes[2,1]
    ax.plot(t,c['pose_observed_attitude_velocity_error'],color=orange,label='production PoseKF, noisy attitude')
    ax.plot(t,c['pose_true_attitude_velocity_error'],color=blue,label='production PoseKF, true attitude')
    ax.set_title('Position-only PoseKF still uses w = 0 in both runs');ax.set_ylabel('velocity error (m/s)');ax.legend(fontsize=8)
    for i,ax in enumerate(axes.flat):
        ax.grid(alpha=.2)
        if i!=4:ax.set_xlabel('time (s)')
    fig.suptitle('Aggressive figure-eight | thrust deviation, observability, and model residuals',fontsize=15)
    fig.savefig(output/'w_aggressive_detail.png',dpi=160);plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=ROOT/'outputs/pose_w_experiment_20261111')
    parser.add_argument('--executable',type=Path,default=ROOT/'build/predictor/pose_kf_replay')
    parser.add_argument('--plot-only',action='store_true',help='redraw existing diagnostics CSV files without replay')
    parser.add_argument('--duration',type=float,default=40)
    parser.add_argument('--seed',type=int,default=42)
    parser.add_argument('--window',type=float,default=.5)
    args=parser.parse_args()
    if not np.isfinite(args.duration) or args.duration<=5 or abs(args.duration/DT-round(args.duration/DT))>1e-8:
        raise ValueError('duration must exceed 5 s and be a multiple of .02')
    if not np.isfinite(args.window) or args.window<2*DT or args.window>args.duration/2:
        raise ValueError('window must be between .04 s and duration/2')
    if abs(args.window/DT-round(args.window/DT))>1e-8: raise ValueError('window must be a multiple of .02')
    output=args.output.resolve()
    if args.plot_only:
        result=json.loads((output/'results.json').read_text())
        os.environ.setdefault('MPLCONFIGDIR',str(output/'.matplotlib'))
        columns={}
        for name in result['experiment']['scenarios']:
            data=np.genfromtxt(output/name/'diagnostics.csv',delimiter=',',names=True)
            columns[name]={field:data[field] for field in data.dtype.names}
        plots(output,columns,result['summary'],result['experiment']['velocity_proxy_window'],result['experiment']['seed'])
        return
    if output.exists() and any(output.iterdir()): raise FileExistsError(f'Choose an empty output directory: {output}')
    output.mkdir(parents=True,exist_ok=True)
    os.environ.setdefault('MPLCONFIGDIR',str(output/'.matplotlib'))
    columns={};summaries={}
    for name in ['hover','circle','figure8','aggressive','vertical','bad_attitude']:
        print(f'Recording {name}',flush=True)
        columns[name],summaries[name]=run_scene(name,SCENARIOS[name],args,output)
    manifest={'dt':DT,'duration':args.duration,'seed':args.seed,'velocity_proxy_window':args.window,
              'gravity':GRAVITY,'pose_process_variance':.01,'pose_measurement_variance':.1,
              'position_noise_std':.15,'velocity_noise_std':.1,
              'executable_sha256':hashlib.sha256(args.executable.read_bytes()).hexdigest(),
              'script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'pose_source_sha256':hashlib.sha256((ROOT/'src/predictor/src/Pose-KF.cpp').read_bytes()).hexdigest(),
              'meaning':'w_required_truth = (a + g*e3) dot b3 - g; synthetic force-aligned kinematic oracle, not measured thrust. '
                        'w_nominal_mean = 0 is used by unchanged production PoseKF. Velocity-based proxies are offline diagnostics only. '
                        'No thrust sensor, no physical motor model, no w state or w sampling.',
              'scenarios':{name:SCENARIOS[name] for name in columns}}
    result={'experiment':manifest,'summary':summaries}
    (output/'results.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    plots(output,columns,summaries,args.window,args.seed)
    print(json.dumps(summaries,indent=2,allow_nan=False),flush=True)


if __name__=='__main__':main()
