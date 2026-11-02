"""End-to-end checks using only the Python standard library."""
import csv
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile


def rows(directory, name):
    with (directory / name).open() as stream:
        return list(csv.DictReader(stream))


def check(condition, message):
    if not condition:
        raise RuntimeError(message)


binary = str(Path(sys.argv[1]).resolve())
with tempfile.TemporaryDirectory(prefix="uav-experiment-") as temporary:
    root = Path(temporary)
    first, second = root / "first", root / "second"
    environment = dict(os.environ, MPLCONFIGDIR=str(root / "matplotlib-cache"))
    for directory in (first, second):
        extra = ["--plot"] if "--plot" in sys.argv[2:] and directory == first else []
        subprocess.run([binary, "--duration", "4", "--output", str(directory), *extra],
                       check=True, env=environment)
    if "--plot" in sys.argv[2:]:
        png = (first / "summary.png").read_bytes()
        check(png.startswith(b"\x89PNG\r\n\x1a\n") and len(png) > 10000, "invalid plot PNG")
    for name in ("states.csv", "observations.csv", "predictions.csv", "lead_metrics.csv"):
        check((first / name).read_bytes() == (second / name).read_bytes(),
              f"fixed-seed output changed: {name}")

    predictions = rows(first, "predictions.csv")
    metrics = rows(first, "metrics.csv")
    check(len(metrics) == 12 and len(predictions) == 240, "unexpected forecast count")
    grouped = {}
    for row in predictions:
        origin, time, lead = (float(row[key]) for key in ("origin", "time", "lead"))
        check(time > origin and time <= 4 + 1e-10, "invalid forecast time")
        check(abs(time - origin - lead) < 1e-10, "lead mismatch")
        error = math.sqrt(sum((float(row[axis]) - float(row["true_" + axis])) ** 2
                              for axis in ("x", "y", "z")))
        check(abs(error - float(row["error_m"])) < 1e-10, "position error mismatch")
        grouped.setdefault((row["model"], row["origin"]), []).append(error)
    for row in metrics:
        errors = grouped[(row["model"], row["origin"])]
        check(abs(sum(errors) / len(errors) - float(row["ade_m"])) < 1e-10, "ADE mismatch")
        check(abs(errors[-1] - float(row["fde_m"])) < 1e-10, "FDE mismatch")
    for row in rows(first, "lead_metrics.csv"):
        errors = [float(item["error_m"]) for item in predictions
                  if item["model"] == row["model"] and
                  abs(float(item["lead"]) - float(row["lead"])) < 1e-10]
        rmse = math.sqrt(sum(error ** 2 for error in errors) / len(errors))
        check(int(row["samples"]) == len(errors), "RMSE sample count mismatch")
        check(abs(rmse - float(row["position_rmse_m"])) < 1e-10, "RMSE mismatch")

    original = (first / "states.csv").read_bytes()
    result = subprocess.run([binary, "--output", str(first)], capture_output=True)
    check(result.returncode != 0 and (first / "states.csv").read_bytes() == original,
          "existing output was overwritten")
    for arguments in (["--dt", "0"], ["--prediction-period", "0.15"],
                      ["--duration", "1"], ["--noise", "nan"], ["--seed", "-1"],
                      ["--scene", "unknown"], ["--history", "2"], ["--duration", "4junk"]):
        output = root / "invalid"
        result = subprocess.run([binary, "--output", str(output), *arguments], capture_output=True)
        check(result.returncode != 0 and not output.exists(), f"invalid arguments accepted: {arguments}")
print("Experiment CSV, metrics, reproducibility and error handling checks passed")
