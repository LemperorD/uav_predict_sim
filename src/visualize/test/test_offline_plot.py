#!/usr/bin/python3
"""Exercise the real matplotlib-cpp executable, including old/reordered CSV schemas."""
import csv
import os
from pathlib import Path
import subprocess
import sys
import tempfile

executable = sys.argv[1]
with tempfile.TemporaryDirectory() as directory:
    root = Path(directory)
    os.environ.update(MPLBACKEND='Agg', MPLCONFIGDIR=str(root / 'cache'), PYTHONNOUSERSITE='1')
    names = ['t', 'truth_x', 'truth_y', 'measurement_x', 'measurement_y', 'estimate_x',
             'estimate_y', 'std_x', 'std_y', 'std_z', 'position_error', 'horizon_error',
             'velocity_error', 'p_cv', 'p_ca']
    source = root / 'current.csv'
    baseline = root / 'baseline with spaces.csv'
    for path, fields in [(source, names), (baseline, list(reversed(names)))]:
        with path.open('w') as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            for i in range(25):
                row = {name: .1 for name in names}
                row.update(t=i * .1, p_cv=float('nan'), p_ca=float('nan'))
                writer.writerow(row)
    def run(*options):
        return subprocess.run([executable, str(source), str(root / 'plot.png'), *options],
                              capture_output=True, text=True)
    result = run('--profile', 'pose_kf', '--baseline', 'KF', str(baseline))
    assert result.returncode == 0, result.stderr
    assert (root / 'plot.png').read_bytes()[:8] == b'\x89PNG\r\n\x1a\n'
    result = run()  # Legacy two-argument CLI, all optional model probabilities absent.
    assert result.returncode == 0, result.stderr
    result = run('--baseline', 'missing', str(root / 'absent.csv'))
    assert result.returncode == 1 and 'cannot open' in result.stderr, result.stderr
    source.write_text('t,truth_x\n0,1\n')
    result = run()
    assert result.returncode == 1 and 'missing CSV column' in result.stderr, result.stderr
