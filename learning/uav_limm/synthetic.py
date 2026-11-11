"""Deterministic synthetic flights for engineering checks, not paper datasets."""
from pathlib import Path

import numpy as np


def generate(output, seed=42, samples=320, dt=1.):
    if samples < 60 or dt <= 0 or not np.isfinite(dt):
        raise ValueError("Need >=60 samples and finite positive dt")
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    rng = np.random.default_rng(seed)
    # Entire flights are disjoint; test includes frequencies absent from training.
    for split, count in (("train", 6), ("val", 2), ("test", 3)):
        directory = output/split
        directory.mkdir()
        for flight in range(count):
            t = np.arange(samples)*dt
            frequency = rng.uniform(.025, .10, 3) * (1.5 if split == "test" else 1)
            amplitude = rng.uniform(2, 12, 3)
            phase = rng.uniform(-np.pi, np.pi, 3)
            position = amplitude*np.sin(t[:, None]*frequency+phase)
            position += t[:, None]*rng.uniform(-.3, .3, 3)
            position -= position[0]
            measurement = position + rng.normal(0, .03, position.shape)
            np.savetxt(directory/f"flight_{flight:02d}.csv",
                       np.column_stack([t, measurement, position]), delimiter=",", comments="",
                       header="t,measurement_x,measurement_y,measurement_z,truth_x,truth_y,truth_z")
