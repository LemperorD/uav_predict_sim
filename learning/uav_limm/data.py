"""Local Cartesian CSV loading, causal resampling and split-before-windowing."""
from pathlib import Path

import numpy as np
import torch
from torch.utils.data import Dataset


def resample_csv(path, dt=1.0, columns=None, target_columns=None, max_gap=None):
    """Read one flight per CSV. No interpolation over missing-flight gaps.

    Defaults: simulator samples.csv (measurement/truth) or t,px,py,pz.
    A target at time t is allowed as label only; it never enters history.
    Linear interpolation uses the bracketing logged samples, suitable for offline
    preprocessing. Online use must instead sample at arrival (see runtime.py).
    """
    if not np.isfinite(dt) or dt <= 0:
        raise ValueError("dt must be positive")
    data = np.genfromtxt(path, delimiter=",", names=True, dtype=float)
    data = np.atleast_1d(data)
    names = data.dtype.names or ()
    if columns is None:
        columns = (("measurement_x", "measurement_y", "measurement_z")
                   if "measurement_x" in names else ("px", "py", "pz"))
    if target_columns is None:
        target_columns = (("truth_x", "truth_y", "truth_z")
                          if "truth_x" in names else columns)
    required = {"t", *columns, *target_columns}
    if len(columns) != 3 or len(target_columns) != 3 or not required.issubset(names):
        raise ValueError(f"{path}: need t and XYZ columns {sorted(required)}")
    t = data["t"]
    obs = np.column_stack([data[c] for c in columns])
    target = np.column_stack([data[c] for c in target_columns])
    if len(t) < 2 or not np.isfinite(np.column_stack([t, obs, target])).all():
        raise ValueError(f"{path}: require at least two finite samples")
    gaps = np.diff(t)
    if (gaps <= 0).any():
        raise ValueError(f"{path}: timestamps must be strictly increasing")
    if max_gap is None:
        max_gap = 2 * dt
    if not np.isfinite(max_gap) or max_gap <= 0 or (gaps > max_gap).any():
        raise ValueError(f"{path}: data gap exceeds {max_gap}s; split flights first")
    grid = t[0] + np.arange(int(np.floor((t[-1]-t[0])/dt + 1e-9))+1)*dt
    # Common observed flight origin: never use future truth to center input.
    origin = obs[0].copy()
    def interpolate(values):
        return np.stack([np.interp(grid, t, values[:, d]) for d in range(3)], -1) - origin
    return grid, interpolate(obs), interpolate(target)


class Windows(Dataset):
    def __init__(self, flights, observation=40, horizon=20, stride=1):
        if stride < 1:
            raise ValueError("stride must be positive")
        self.flights = flights
        self.observation = observation
        self.horizon = horizon
        self.indices = [(f, i) for f, (_, x, _) in enumerate(flights)
                        for i in range(0, len(x)-observation-horizon+1, stride)]
        if not self.indices:
            raise ValueError("No windows: each split needs observation+horizon samples")

    def __len__(self):
        return len(self.indices)

    def __getitem__(self, index):
        f, i = self.indices[index]
        _, x, y = self.flights[f]
        end = i + self.observation
        # Full future path starts with last observed point for causal differences.
        return (torch.tensor(x[i:end], dtype=torch.float32),
                torch.tensor(y[end:end+self.horizon], dtype=torch.float32))


def load_splits(paths, config, stride=1, columns=None, target_columns=None):
    """Each independent flight is chronologically split 7:1:2 before windows.

    Also supports explicit train/val/test directories (preferred when flights are
    already assigned chronologically). No window may cross a flight or split.
    """
    groups = [[], [], []]
    paths = [Path(p) for p in paths]
    explicit = len(paths) == 1 and paths[0].is_dir() and (paths[0]/"train").is_dir()
    if explicit:
        for group, name in zip(groups, ("train", "val", "test")):
            files = sorted((paths[0]/name).glob("*.csv"))
            if not files:
                raise ValueError(f"No CSV files in {paths[0]/name}")
            group.extend(resample_csv(p, config.dt, columns, target_columns) for p in files)
    else:
        files = sorted({p for root in paths for p in
                        (root.glob("*.csv") if root.is_dir() else [root])})
        if not files:
            raise ValueError("No input CSV files")
        for path in files:
            t, x, y = resample_csv(path, config.dt, columns, target_columns)
            a, b = int(len(t)*.7), int(len(t)*.8)
            for group, sl in zip(groups, (slice(None, a), slice(a, b), slice(b, None))):
                group.append((t[sl], x[sl], y[sl]))
    return [Windows(group, config.observation, config.horizon, stride) for group in groups]


def features(positions, dt):
    velocity = positions.diff(dim=-2)/dt
    return positions, velocity, velocity.diff(dim=-2)/dt


def future_features(history, future, dt):
    # Boundary acceleration uses the last *historical* finite-difference velocity.
    joined = torch.cat([history[:, -2:], future], dim=1)
    velocity = joined.diff(dim=1)/dt
    return future, velocity[:, 1:], velocity.diff(dim=1)/dt
