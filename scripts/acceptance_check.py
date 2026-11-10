#!/usr/bin/python3
"""Real ROS/EGO pipeline regression, including CSV input and non-default file locations."""
import csv
import datetime
import json
import math
from pathlib import Path
import subprocess
import sys
import yaml

ROOT = Path(__file__).resolve().parents[1]


def main():
    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    base = ROOT/'outputs'/f'acceptance_{stamp}'
    base.mkdir(parents=True)
    template = yaml.safe_load((ROOT/'src/predictor/config/simulation.yaml').read_text())
    results = {}
    for name in ('straight_noisy', 'acceleration_noisy', 'figure8_clean', 'hover_noisy',
                 'circle_noisy', 'helix_noisy', 'sinusoid_noisy', 'csv_replay',
                 'three_models', 'single_body_model'):
        config = yaml.safe_load(yaml.safe_dump(template))
        p = config['/**']['ros__parameters']
        p.update(output_dir=str(base/name), duration=8.0, show_viewer=False, record_video=False)
        p['trajectory'] = 'acceleration' if name == 'acceleration_noisy' else 'straight'
        if name == 'figure8_clean':
            p.update(trajectory='figure8', inject_noise=False)
        if name in ('hover_noisy', 'circle_noisy', 'helix_noisy', 'sinusoid_noisy'):
            p.update(trajectory=name.removesuffix('_noisy'),
                     trajectory_origin=[1.0, -2.0, 4.0], trajectory_radius=2.5,
                     trajectory_angular_speed=-0.5, trajectory_circle_phase=0.3,
                     trajectory_vertical_speed=0.15,
                     trajectory_amplitude=[2.0, 1.0, 0.4],
                     trajectory_frequency=[0.4, 0.6, 0.3],
                     trajectory_phase=[0.2, 0.1, -0.2])
        if name == 'three_models':
            p.update(trajectory='circle', motion_models=['body_velocity', 'ca', 'cv'],
                     model_initial_probability=[0.4, 0.3, 0.3],
                     model_transition=[0.96, 0.02, 0.02, 0.02, 0.96, 0.02, 0.02, 0.02, 0.96])
        if name == 'single_body_model':
            p.update(motion_models=['body_velocity'])
        if name == 'csv_replay':
            # Exercise configured nested output names and input path containing spaces.
            source = base/'kinematic input.csv'
            with source.open('w') as f:
                writer = csv.writer(f, lineterminator='\n')
                writer.writerow(['t','px','py','pz','vx','vy','vz','ax','ay','az'])
                for i in range(501):
                    t=i*.02
                    writer.writerow([t,t,.4*t,2,1,.4,0,0,0,0])
            p.update(input_csv=str(source), trajectory='ignored_for_csv', samples_file='data/observations.csv',
                     predictions_file='data/forecasts.csv', plot_file='charts/results.png',
                     metrics_file='stats/result.json', snapshot_file='config/resolved.yaml',
                     launch_log_file='logs/ros.log')
        path=base/f'{name}.yaml'
        path.write_text(yaml.safe_dump(config))
        subprocess.run([str(ROOT/'scripts/run_simulation.sh'),'--config',str(path),'--headless'],check=True)
        metrics=json.loads((Path(p['output_dir'])/p['metrics_file']).read_text())
        assert metrics['samples']>500, f'{name}: missing odometry'
        assert metrics['scored_predictions']>300, f'{name}: missing aligned forecasts'
        assert metrics['horizon_position_rmse_m'] is not None
        if name in ('straight_noisy','acceleration_noisy','csv_replay'):
            assert metrics['filtered_position_rmse_m']<metrics['measurement_position_rmse_m']*.6, name
        if name=='figure8_clean':
            assert metrics['filtered_position_rmse_m']<.12, name
        with (Path(p['output_dir'])/p['samples_file']).open() as f:
            observations = list(csv.DictReader(f))
        if name in ('three_models', 'single_body_model'):
            assert 'p_body_velocity' in observations[0], name
            for row in observations:
                assert abs(sum(float(v) for k, v in row.items() if k.startswith('p_'))-1) < 1e-8, name
                if name == 'single_body_model':
                    assert float(row['p_body_velocity']) == 1, name
        if name == 'hover_noisy':
            for row in observations:
                assert all(abs(float(row[f'truth_{a}']) - v) < 1e-10
                           for a, v in zip('xyz', p['trajectory_origin'])), name
        if name in ('circle_noisy', 'helix_noisy'):
            for row in observations:
                radius = math.hypot(float(row['truth_x']) - 1, float(row['truth_y']) + 2)
                assert abs(radius - 2.5) < 1e-8, name
            if name == 'helix_noisy':
                assert float(observations[-1]['truth_z']) > 5.0, name
        results[name]=metrics
    invalid=yaml.safe_load(yaml.safe_dump(template))
    invalid['/**']['ros__parameters'].update(output_dir=str(base/'invalid_config'),
        stay_probability=1.0, duration=3.0, show_viewer=False, record_video=False)
    invalid_path=base/'invalid_config.yaml'
    invalid_path.write_text(yaml.safe_dump(invalid))
    rejected=subprocess.run([str(ROOT/'scripts/run_simulation.sh'),'--config',str(invalid_path),'--headless'])
    assert rejected.returncode!=0, 'invalid transition probability was accepted'
    results['invalid_config_rejected']=True
    invalid_cases = [ {'trajectory': 'missing'}, {'trajectory_radius': -1.0},
                      {'trajectory_origin': [1.0, 2.0]} ]
    for i, overrides in enumerate(invalid_cases):
        parameters = {'trajectory': 'circle', **overrides}
        path = base/f'invalid_trajectory_{i}.yaml'
        path.write_text(yaml.safe_dump({'/**': {'ros__parameters': parameters}}))
        run = subprocess.run(['ros2', 'run', 'predictor', 'kinematic_source',
                              '--ros-args', '--params-file', str(path)],
                             capture_output=True, text=True, timeout=5)
        assert run.returncode != 0, f'invalid trajectory accepted: {overrides}'
        assert ('trajectory' in run.stderr or 'radius' in run.stderr), run.stderr
    results['invalid_trajectories_rejected']=True
    (base/'acceptance.json').write_text(json.dumps(results,indent=2)+'\n')
    print(f'All EGO integration checks passed: {base}')


if __name__=='__main__':
    main()
