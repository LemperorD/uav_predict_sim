import torch
from torch import nn

from .data import features, future_features
from .imm import LearningIMM
from .model import DifferentialForecaster


class LIMM(nn.Module):
    def __init__(self, config):
        super().__init__()
        self.config = config
        self.upstream = DifferentialForecaster(config)
        self.fusion = LearningIMM(config)

    def freeze_upstream(self):
        self.upstream.requires_grad_(False)
        self.upstream.eval()

    def train(self, mode=True):
        super().train(mode)
        if not any(p.requires_grad for p in self.upstream.parameters()):
            self.upstream.eval()
        return self

    def forward(self, history):
        pva = features(history, self.config.dt)
        normalized = [scaler(series)[0] for scaler, series in zip(self.upstream.scalers, pva)]
        encoded_history = torch.cat([value[:, -self.config.observation+2:] for value in normalized], -1)
        scope = self.upstream.scalers[1].scope(pva[1])
        upstream = self.upstream(history)
        measurements = upstream["velocities"]/scope.unsqueeze(-2)
        result = self.fusion(encoded_history, pva[1][:, -1]/scope[:, 0], measurements)
        result["velocities"] = result["normalized_velocity"]*scope
        result["positions"] = history[:, -1:] + result["velocities"].cumsum(1)*self.config.dt
        result["velocity_covariance"] = result["normalized_covariance"]*scope.unsqueeze(-1)*scope.unsqueeze(-2)
        result["upstream_positions"] = upstream["positions"]
        return result

    def loss(self, history, future):
        # §5.3: learn future velocity sequences, then integrate to positions.
        target = future_features(history, future, self.config.dt)[1]
        scope = self.upstream.scalers[1].scope(features(history, self.config.dt)[1])
        return (self(history)["normalized_velocity"]-target/scope).square().mean()
