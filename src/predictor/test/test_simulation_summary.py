"""Prediction-only algorithms must not need fabricated filter diagnostics."""
import contextlib
import csv
import importlib.util
import io
import json
import math
from pathlib import Path
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location('simulation_runner', ROOT/'scripts/run_simulation.py')
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


class SummaryTest(unittest.TestCase):
    def test_missing_diagnostics_preserve_prediction_metrics(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            samples = path/'samples.csv'
            keys = ['t'] + [f'{kind}_{axis}' for kind in ('truth', 'measurement', 'estimate')
                            for axis in 'xyz']
            keys += ['std_x', 'std_y', 'std_z', 'p_cv', 'p_ca', 'position_error',
                     'horizon_error', 'velocity_error', 'position_nees']
            with samples.open('w') as stream:
                writer = csv.DictWriter(stream, fieldnames=keys)
                writer.writeheader()
                for i in range(30):
                    row = dict.fromkeys(keys, math.nan)
                    row['t'] = i*.1
                    for kind in ('truth', 'measurement'):
                        for axis in 'xyz':
                            row[f'{kind}_{axis}'] = 0.
                    writer.writerow(row)
            forecasts = path/'predictions.csv'
            forecasts.write_text('error\n0.2\n')
            metrics = path/'metrics.json'
            with contextlib.redirect_stdout(io.StringIO()):
                result = runner.summarize(samples, forecasts, metrics)
            self.assertEqual(result['horizon_position_rmse_m'], .2)
            self.assertIsNone(result['filtered_position_rmse_m'])
            self.assertIsNone(result['velocity_rmse_mps'])
            self.assertIsNone(result['mean_position_nees'])
            self.assertEqual(result['position_2sigma_axis_coverage'], dict.fromkeys('xyz'))
            self.assertEqual(json.loads(metrics.read_text()), result)
            # Missing diagnostics are allowed; malformed required observations are not.
            samples.write_text(samples.read_text().replace(',0.0,', ',inf,', 1))
            with self.assertRaises(RuntimeError):
                runner.summarize(samples, forecasts, metrics)


if __name__ == '__main__':
    unittest.main()
