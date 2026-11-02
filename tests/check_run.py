"""Independent CSV/physics checks and CLI/replay regression (stdlib only)."""
import csv
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile


def require(ok, text):
    if not ok:
        raise RuntimeError(text)


def rows(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


binary = str(Path(sys.argv[1]).resolve())
with tempfile.TemporaryDirectory(prefix="uav-run-") as temporary:
    root = Path(temporary)
    config = root / "case.yaml"
    config.write_text("""simulation: {duration: 3, dt: 0.001}
trajectory: {scene: eight}
observation: {period: 0.02, noise_std: 0.01, seed: 42}
prediction: {period: 0.1, horizon: 0.5, step: 0.1, history_duration: 0.3}
""")
    environment = dict(os.environ, MPLCONFIGDIR=str(root / "mpl"))
    first, second = root / "first", root / "second"
    for directory in (first, second):
        flags = ["--plot"] if "--plot" in sys.argv[2:] and directory == first else []
        subprocess.run([binary, "--config", str(config), "--output", str(directory), *flags],
                       check=True, env=environment)
    for file in ("state.csv", "reference.csv", "control.csv", "observation.csv", "prediction.csv", "lead_metrics.csv", "config.yaml"):
        require((first / file).read_bytes() == (second / file).read_bytes(), f"not reproducible: {file}")
    state, control = rows(first / "state.csv"), rows(first / "control.csv")
    require(len(state) == 3001 and len(control) == len(state), "state count")
    require(max(float(s["z"]) for s in state) - min(float(s["z"]) for s in state) > 0.5, "flight is not 3D")
    kf, mass, gravity = 8.98132e-9, 0.98, 9.81
    for s, c in zip(state, control):
        qw, qx, qy, qz = (float(s[k]) for k in ("qw", "qx", "qy", "qz"))
        require(abs(qw*qw + qx*qx + qy*qy + qz*qz - 1) < 1e-12, "quaternion norm")
        b3 = [2*(qx*qz+qw*qy), 2*(qy*qz-qw*qx), 1-2*(qx*qx+qy*qy)]
        thrust = kf * sum(float(s[f"rpm{i}"]) ** 2 for i in range(1, 5))
        require(abs(thrust - float(c["thrust_actual"])) < 1e-10, "rotor thrust mismatch")
        for axis, acceleration in enumerate(("ax", "ay", "az")):
            expected = thrust * b3[axis] / mass - (gravity if axis == 2 else 0)
            require(abs(expected - float(s[acceleration])) < 1e-10, "rigid-body thrust axis violation")
    forecasts = rows(first / "prediction.csv")
    metrics = rows(first / "metrics.csv")
    grouped = {}
    for row in forecasts:
        grouped.setdefault(row["forecast_id"], []).append(row)
        origin, time, lead = (float(row[k]) for k in ("origin", "time", "lead"))
        require(time > origin and abs(time-origin-lead) < 1e-10, "prediction time mismatch")
        if row["true_x"]:
            truth = state[round(time / 0.001)]
            require(all(abs(float(row["true_"+axis])-float(truth[axis])) < 1e-12 for axis in ("x", "y", "z")), "truth alignment")
            distance = math.sqrt(sum((float(row[a])-float(row["true_"+a])) ** 2 for a in ("x", "y", "z")))
            require(abs(distance-float(row["error_m"])) < 1e-10, "prediction distance mismatch")
    complete_ids = set()
    for metric in metrics:
        data = grouped[metric["forecast_id"]]
        if metric["status"] == "complete":
            complete_ids.add(metric["forecast_id"])
            errors = [float(row["error_m"]) for row in data]
            require(abs(sum(errors)/len(errors)-float(metric["ade_m"])) < 1e-10, "ADE mismatch")
            require(abs(errors[-1]-float(metric["fde_m"])) < 1e-10, "FDE mismatch")
        else:
            require(metric["ade_m"] == metric["fde_m"] == "", "incomplete horizon has aggregate metrics")
    for lead in rows(first / "lead_metrics.csv"):
        data = [row for row in forecasts if row["forecast_id"] in complete_ids and
                row["model"] == lead["model"] and abs(float(row["lead"])-float(lead["lead"])) < 1e-10]
        rmse = math.sqrt(sum(float(row["error_m"])**2 for row in data)/len(data))
        require(abs(rmse-float(lead["position_rmse_m"])) < 1e-10, "RMSE mismatch")
        require(int(lead["samples"]) == len(data), "complete sample count")
    before = (first / "state.csv").read_bytes()
    subprocess.run([binary, "--replay", str(first)], check=True)
    require((first / "state.csv").read_bytes() == before, "replay changed numerical data")
    require(subprocess.run([binary, "--output", str(first)], capture_output=True).returncode != 0, "overwrote old run")
    for text in ("simulation: {dt: 0}", "trajectory: {scen: eight}",
                 "observation: {noise_std: -.1}", "observation: {seed: -1}",
                 "prediction: {period: .015}", "vehicle: {mass: .nan}",
                 "trajectory: {time_scale: .01}"):
        config.write_text(text)
        result = subprocess.run([binary, "--config", str(config), "--output", str(root / "invalid")], capture_output=True)
        require(result.returncode != 0 and not (root / "invalid").exists(), "invalid config accepted: " + text)
    if "--plot" in sys.argv[2:]:
        for name in ("trajectory3d.png", "tracking.png", "dynamics.png", "prediction_errors.png"):
            data = (first / name).read_bytes()
            require(data.startswith(b"\x89PNG\r\n\x1a\n") and len(data) > 10000, "invalid plot: " + name)
    # A truncated file must fail clearly rather than play inconsistent frames.
    (first / "state.csv").write_text("time,x\n0,0\n")
    require(subprocess.run([binary, "--replay", str(first)], capture_output=True).returncode != 0, "truncated replay accepted")
print("Independent physics, metrics, config, reproducibility and replay checks passed")
