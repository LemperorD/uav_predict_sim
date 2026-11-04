#!/usr/bin/python3
"""Real ROS/EGO pipeline regression, including CSV input and non-default file locations."""
import csv
import datetime
import json
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
    for name in ('straight_noisy', 'acceleration_noisy', 'figure8_clean', 'csv_replay'):
        config = yaml.safe_load(yaml.safe_dump(template))
        p = config['/**']['ros__parameters']
        p.update(output_dir=str(base/name), duration=8.0, show_viewer=False, record_video=False)
        p['trajectory'] = 'acceleration' if name == 'acceleration_noisy' else 'straight'
        if name == 'figure8_clean':
            p.update(trajectory='figure8', inject_noise=False)
        if name == 'csv_replay':
            # Exercise configured nested output names and input path containing spaces.
            source = base/'kinematic input.csv'
            with source.open('w') as f:
                writer = csv.writer(f, lineterminator='\n')
                writer.writerow(['t','px','py','pz','vx','vy','vz','ax','ay','az'])
                for i in range(501):
                    t=i*.02
                    writer.writerow([t,t,.4*t,2,1,.4,0,0,0,0])
            p.update(input_csv=str(source), samples_file='data/observations.csv',
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
        results[name]=metrics
    invalid=yaml.safe_load(yaml.safe_dump(template))
    invalid['/**']['ros__parameters'].update(output_dir=str(base/'invalid_config'),
        stay_probability=1.0, duration=3.0, show_viewer=False, record_video=False)
    invalid_path=base/'invalid_config.yaml'
    invalid_path.write_text(yaml.safe_dump(invalid))
    rejected=subprocess.run([str(ROOT/'scripts/run_simulation.sh'),'--config',str(invalid_path),'--headless'])
    assert rejected.returncode!=0, 'invalid transition probability was accepted'
    results['invalid_config_rejected']=True
    (base/'acceptance.json').write_text(json.dumps(results,indent=2)+'\n')
    print(f'All EGO integration checks passed: {base}')


if __name__=='__main__':
    main()
