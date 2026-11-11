#!/usr/bin/python3
"""Run, finalize CSV/PNG/MP4, and produce machine-readable acceptance metrics."""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import yaml

ROOT = Path(__file__).resolve().parents[1]


def output_path(base, value):
    path = Path(value).expanduser()
    return path if path.is_absolute() else base / path


def rmse(values):
    values = [x for x in values if math.isfinite(x)]
    return math.sqrt(sum(x*x for x in values) / len(values)) if values else None


def summarize(samples, forecasts, destination):
    with samples.open() as f:
        rows = [{k: float(v) for k, v in row.items()} for row in csv.DictReader(f)]
    if len(rows) < 20:
        raise RuntimeError('too few samples; simulation failed')
    with forecasts.open() as f:
        predictions = list(csv.DictReader(f))
    # Warm-up is excluded from aggregate filter metrics, retained in raw files.
    settled = [r for r in rows if r['t'] >= min(2.0, rows[-1]['t'] / 4)]
    for row in rows:
        if not all(math.isfinite(v) for k, v in row.items() if k != 'horizon_error'):
            raise RuntimeError('nonfinite filter output')
        if abs(sum(value for key, value in row.items() if key.startswith('p_')) - 1) > 1e-8:
            raise RuntimeError('invalid model probabilities')
    result = {
        'samples': len(rows), 'scored_predictions': len(predictions), 'duration_s': rows[-1]['t'],
        'measurement_position_rmse_m': rmse([math.sqrt(sum((r[f'measurement_{a}'] - r[f'truth_{a}'])**2 for a in 'xyz')) for r in settled]),
        'filtered_position_rmse_m': rmse([r['position_error'] for r in settled]),
        'velocity_rmse_mps': rmse([r['velocity_error'] for r in settled]),
        'horizon_position_rmse_m': rmse([float(r['error']) for r in predictions]),
        'mean_position_nees': sum(r['position_nees'] for r in settled)/len(settled),
        'position_2sigma_axis_coverage': {
            a: sum(abs(r[f'estimate_{a}']-r[f'truth_{a}'])<=2*r[f'std_{a}'] for r in settled)/len(settled)
            for a in 'xyz'
        },
        'unscored_tail_note': 'Predictions whose target is beyond the last truth sample are excluded.',
    }
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps(result, indent=2))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT/'src/predictor/config/simulation.yaml')
    parser.add_argument('--visualization-config', type=Path,
                        default=ROOT/'src/predictor/config/visualization.yaml',
                        help='Pangolin appearance YAML (loaded at startup)')
    parser.add_argument('--headless', action='store_true', help='disable Pangolin/video; keep numerical run and plots')
    parser.add_argument('--finalize-only', action='store_true', help='regenerate plots/metrics/video from existing data')
    args = parser.parse_args()
    config = args.config.expanduser().resolve()
    params = yaml.safe_load(config.read_text())['/**']['ros__parameters']
    output = output_path(config.parent, params['output_dir']).resolve()
    output.mkdir(parents=True, exist_ok=True)
    samples = output_path(output, params.get('samples_file', 'samples.csv'))
    forecasts = output_path(output, params.get('predictions_file', 'predictions.csv'))
    launch_log = output_path(output, params.get('launch_log_file', 'launch.log'))
    metrics_file = output_path(output, params['metrics_file'])
    plot_file = output_path(output, params['plot_file'])
    frames = output_path(output, params['frame_dir'])
    video = output_path(output, params['video_file'])
    show = params.get('show_viewer', True) and not args.headless
    if show and not os.environ.get('DISPLAY') and not args.finalize_only:
        raise RuntimeError('Pangolin needs DISPLAY. Run in a graphical terminal or use --headless.')
    if not args.finalize_only:
        # Refuse overwrites so failed/new runs cannot masquerade as an earlier success.
        if samples.exists() or forecasts.exists() or any(frames.glob('frame_*.png')):
            raise RuntimeError(f'Output already contains a run: {output}. Choose a new output_dir.')
        command = ['ros2','launch','predictor','simulation.launch.py',f'config:={config}',
                   f'visualization_config:={args.visualization_config.expanduser().resolve()}']
        if args.headless:
            command.append('viewer:=false')
        launch_log.parent.mkdir(parents=True, exist_ok=True)
        with launch_log.open('w') as log:
            run_env = dict(os.environ, ROS_LOG_DIR=str(output_path(output, params.get('ros_log_dir', 'ros_logs'))))
            child = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True, env=run_env)
            try:
                code = child.wait(timeout=float(params['duration'])+35)
            except (KeyboardInterrupt, subprocess.TimeoutExpired):
                os.killpg(child.pid, signal.SIGINT)
                try:
                    child.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGKILL)
                    child.wait()
                raise RuntimeError(f'Simulation interrupted/timed out; inspect {launch_log}')
        log_text = launch_log.read_text()
        if code or 'process has died' in log_text or '[ERROR]' in log_text:
            raise RuntimeError(f'ROS run failed; inspect {launch_log}')
    result = summarize(samples, forecasts, metrics_file)
    if not args.finalize_only and result['duration_s'] < float(params['duration']) - 1.0:
        raise RuntimeError('Simulation terminated before configured duration')
    env = dict(os.environ, MPLBACKEND='Agg', PYTHONNOUSERSITE='1', MPLCONFIGDIR=str(output_path(output, params.get('matplotlib_cache_dir', '.matplotlib'))))
    subprocess.run(['ros2','run','prediction_visualize','prediction_plot',str(samples),str(plot_file)], check=True, env=env)
    if params.get('record_video', True) and show:
        if not any(frames.glob('frame_*.png')):
            raise RuntimeError('Recording requested but no Pangolin frames were captured')
        video.parent.mkdir(parents=True, exist_ok=True)
        ffmpeg = str(output_path(config.parent, params.get('ffmpeg_path', '/usr/bin/ffmpeg')))
        times_file = output_path(output, params.get('frame_timestamps_file', 'frame_times.csv'))
        with times_file.open() as f:
            times = [(int(r['frame']), float(r['elapsed_s'])) for r in csv.DictReader(f)]
        if len(times) < 2:
            raise RuntimeError('Recording has fewer than two timestamped frames')
        manifest = output_path(output, params.get('video_manifest_file', 'video_frames.ffconcat'))
        manifest.parent.mkdir(parents=True, exist_ok=True)
        def entry(index):
            # ffconcat uses single-quoted filenames; escape literal apostrophes.
            name = str((frames/f'frame_{index:06d}.png').resolve()).replace("'", "'\\''")
            return f"file '{name}'\n"
        with manifest.open('w') as f:
            f.write('ffconcat version 1.0\n')
            for i, (index, timestamp) in enumerate(times):
                interval = times[i+1][1]-timestamp if i+1<len(times) else 1/params['video_fps']
                if not math.isfinite(interval) or interval <= 0:
                    raise RuntimeError('Invalid recording timestamps')
                f.write(entry(index)+f'duration {interval:.9f}\n')
            f.write(entry(times[-1][0]))
        subprocess.run([ffmpeg,'-y','-loglevel','error','-f','concat','-safe','0',
                        '-i',str(manifest),'-vsync','vfr','-c:v','libx264','-pix_fmt','yuv420p',str(video)], check=True)
    print(f'Acceptance artifacts: {output}')


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        sys.exit(1)
