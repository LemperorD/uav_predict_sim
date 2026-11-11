"""Position-only streaming contract; no ROS imports or access to future truth."""
from collections import deque
import math

import numpy as np
import torch

from .training import load_checkpoint


class StreamingPredictor:
    def __init__(self, model, device="cpu", tolerance=0.05):
        self.model = model.to(device).eval()
        self.device = device
        self.tolerance = tolerance
        if not 0 <= tolerance < .5:
            raise ValueError("tolerance must be a fraction in [0,.5)")
        self.reset()

    @classmethod
    def from_checkpoint(cls, path, device="cpu"):
        return cls(load_checkpoint(path, device)[0], device)

    def reset(self):
        self.history = deque(maxlen=self.model.config.observation)
        self.timestamp = None
        self.origin = None

    @property
    def ready(self):
        return len(self.history) == self.model.config.observation

    def update(self, timestamp, position):
        p = np.asarray(position, dtype=np.float64)
        if p.shape != (3,) or not np.isfinite(p).all() or not math.isfinite(timestamp):
            raise ValueError("Require finite timestamp and XYZ position")
        if self.timestamp is not None:
            delta = timestamp-self.timestamp
            if delta <= 0:
                raise ValueError("Timestamps must increase")
            if abs(delta-self.model.config.dt) > self.model.config.dt*self.tolerance + 1e-9:
                raise ValueError("Sampling differs from checkpoint dt; reset after gaps")
        if self.origin is None:
            self.origin = p.copy()
        self.history.append(p-self.origin)
        self.timestamp = timestamp

    @torch.inference_mode()
    def predict(self, step=None, count=None):
        config = self.model.config
        if step is not None and not math.isclose(step, config.dt, rel_tol=1e-6):
            raise ValueError("Prediction step must equal checkpoint dt")
        if count is None:
            count = config.horizon
        if type(count) is not int or not 1 <= count <= config.horizon:
            raise ValueError("Count must be within trained horizon")
        if not self.ready:
            raise RuntimeError(f"Need {config.observation} historical samples")
        history = torch.tensor(np.array(self.history), dtype=torch.float32, device=self.device)[None]
        positions = self.model(history)["positions"][0, :count].cpu().numpy()
        result = positions+self.origin
        if not np.isfinite(result).all():
            raise FloatingPointError("Nonfinite forecast")
        return result
